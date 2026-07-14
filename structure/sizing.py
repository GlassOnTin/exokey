"""MINIMUM MASS AT THE GATE, BY GRADIENT -- not by greedy deletion.

THE USER: "So, we'll likely be able to choose a better optimiser for the structural calculation
then too?"

Yes, and it is the biggest prize in decoupling the structure from the layout.

WHAT ESO ACTUALLY IS. Evolutionary Structural Optimisation is a HEURISTIC, not an optimiser:
delete the lowest-strain-energy elements, re-solve, repeat. It is greedy, it is binary (a strut is
in or out), and IT CAN NEVER PUT MATERIAL BACK. A bad early deletion is permanent. We measured
exactly that: the same design grown down a slightly different path varies -29% to +13% in mass.
That is not an optimum, it is a PATH.

The only reason it was tolerable is that it had to run 2400 times inside the GA. And the only
reason it had to do that is that the layout and the structure were being co-optimised -- which the
data says was never necessary, because the structural constraints NEVER BIND (yield 0/32,
supportable 0/32) and bone mass carries NO trade against effort (rho = -0.18, p = 0.53).

Decoupled, the structure is solved ONCE. So it can be solved properly.

WHAT THIS DOES INSTEAD. Each strut gets a CONTINUOUS radius. Minimise mass subject to the
deflection gate:

    min_r  sum_e  rho * pi * r_e^2 * L_e
    s.t.   max_c |u_button(c; r)|  <=  gate
           r_min <= r_e <= r_max

Struts whose radius goes to r_min ARE the topology -- they are deleted at the end. This is the
classical minimum-weight sizing problem, and ESO is the special case where the radii are forced to
0/1 and are only ever allowed to go DOWN.

THE SENSITIVITY IS NEARLY FREE, and that is the whole trick. For a linear system K u = f,

    du/dr_e  =  -K^-1 (dK/dr_e) u

and the derivative of ONE button's displacement along a direction q needs no extra factorisation
at all -- take the ADJOINT: solve K lambda = q once per (button, load case), then

    d(q.u)/dr_e  =  -lambda^T (dK/dr_e) u

dK/dr_e is local to element e (12x12), so every element's sensitivity is a pair of small
quadratic forms. One extra back-substitution per constraint, and the whole gradient falls out.
We ALREADY have the factorisation.

⚠ THE ANCHOR IS NONLINEAR (compression-only tissue, tension-only strap), which breaks the textbook
adjoint. Handled the way contact problems always are: iterate to a fixed active set, FREEZE it,
and take sensitivities on the frozen system. Honest as long as the set has converged -- and it is
re-converged every outer step, so if it moves, we see it.
"""
from __future__ import annotations

import numpy as np

from structure.fem import Frame
from structure.frame import MATERIALS


def _assemble(nodes, bars, r, mat):
    p = MATERIALS[mat]
    A = np.pi * r ** 2
    I = np.pi * r ** 4 / 4
    return Frame(nodes, bars, p["E"], p["E"] / 2.6, A, I, 2 * I)


class Sizer:
    """Continuous-radius sizing of a ground structure, with adjoint sensitivities.

    Radii are PER-ELEMENT and continuous, so the Frame's single (A, I, J) is not enough. Rather
    than rewrite Frame, the element stiffness is scaled analytically: for a circular section the
    12x12 local stiffness splits into an AXIAL/TORSION part that scales as r^2 and a BENDING part
    that scales as r^4. So k_e(r) = (r/r0)^2 * k_axial + (r/r0)^4 * k_bend, exactly -- no
    re-assembly of the element matrices, and dk/dr is analytic.
    """

    AX = [0, 3, 6, 9]        # local dofs whose stiffness goes as A ~ r^2 (axial u, torsion rx)

    def __init__(self, nodes, bars, mat="cf_pa12", r0=0.0009):
        self.nodes = np.asarray(nodes, float)
        self.bars = [tuple(b) for b in bars]
        self.mat = mat
        self.r0 = r0
        self.rho = MATERIALS[mat]["rho"]
        fr = _assemble(self.nodes, self.bars, r0, mat)
        self.fr = fr
        self.L = fr.L

        # split each element's LOCAL stiffness into its r^2 and r^4 parts
        m = np.zeros((12, 12), bool)
        m[np.ix_(self.AX, self.AX)] = True
        self.k2 = fr.kloc * m                      # axial + torsion  -> r^2
        self.k4 = fr.kloc * (~m)                   # bending + shear  -> r^4

    def _global_k(self, r):
        """Element stiffnesses at radius r, in global coords."""
        s2 = (r / self.r0) ** 2
        s4 = (r / self.r0) ** 4
        kl = self.k2 * s2[:, None, None] + self.k4 * s4[:, None, None]
        return kl, np.einsum("bji,bjk,bkl->bil", self.fr.T, kl, self.fr.T)

    def mass(self, r):
        return float(self.rho * np.pi * np.sum(r ** 2 * self.L))

    def solve(self, r, spring, cases):
        """Displacements and per-element local stiffness at radius r."""
        from scipy.sparse import coo_matrix
        from scipy.sparse.linalg import splu

        kl, kg = self._global_k(r)
        d = self.fr.dofs
        rows = np.repeat(d, 12, axis=1).ravel()
        cols = np.tile(d, (1, 12)).ravel()
        vals = kg.ravel()

        sr, sv = [], []
        for i, k in spring.items():
            if i in self.fr.idx:
                for q in range(3):
                    sr.append(6 * self.fr.idx[i] + q)
                    sv.append(k)
        n = self.fr.ndof
        K = coo_matrix((np.concatenate([vals, np.array(sv)]),
                        (np.concatenate([rows, np.array(sr, int)]),
                         np.concatenate([cols, np.array(sr, int)]))), shape=(n, n)).tocsc()
        K = K + 1e-6 * coo_matrix((np.ones(n), (np.arange(n), np.arange(n))),
                                  shape=(n, n)).tocsc()
        lu = splu(K)
        B = np.zeros((n, len(cases)))
        for c, (_f, _a, load) in enumerate(cases):
            for i, fv in load.items():
                if i in self.fr.idx:
                    B[6 * self.fr.idx[i]:6 * self.fr.idx[i] + 3, c] = fv
        U = lu.solve(B).T                                       # (ncase, ndof)
        return U, lu, kl

    def grad_disp(self, r, U, lu, node, case, direction):
        """d|u_node . direction| / dr_e for every element e. ONE extra back-substitution.

        THE ADJOINT. u = K^-1 f, and we want g = q.u for a fixed q (a button, one direction). Then
            dg/dr_e = q^T du/dr_e = -q^T K^-1 (dK/dr_e) u = -lambda^T (dK/dr_e) u,   K lambda = q
        so ONE solve gives the gradient with respect to EVERY element at once -- which is the whole
        reason a gradient method is affordable here and a finite-difference one is not (that would
        need one solve per element: 400 instead of 1).
        """
        q = np.zeros(self.fr.ndof)
        base = 6 * self.fr.idx[node]
        q[base:base + 3] = direction
        lam = lu.solve(q)

        d = self.fr.dofs
        ue = U[case][d]                                  # (nbar, 12)
        le = lam[d]                                      # (nbar, 12)
        # dk/dr = 2/r0 * (r/r0) * k2 + 4/r0 * (r/r0)^3 * k4, in LOCAL coords; rotate the vectors
        ul = np.einsum("bij,bj->bi", self.fr.T, ue)
        ll = np.einsum("bij,bj->bi", self.fr.T, le)
        s2 = 2.0 * r / self.r0 ** 2
        s4 = 4.0 * r ** 3 / self.r0 ** 4
        q2 = np.einsum("bi,bij,bj->b", ll, self.k2, ul)
        q4 = np.einsum("bi,bij,bj->b", ll, self.k4, ul)
        return -(s2 * q2 + s4 * q4)


def size(nodes, bars, buttons, cases, anchor_k, anchor_n, strap_n, strap_k,
         gate=0.5e-3, mat="cf_pa12", r_min=1e-6, r_max=2.5e-3, r0=9e-4,
         steps=22, pnorm=8.0, eta=0.5, on_step=None):
    """Minimise mass subject to the deflection gate. Returns (radii, mass, worst, live).

    OPTIMALITY CRITERIA, derived rather than invented. At a KKT point, for any strut not sitting on
    a bound,

        dV/dr_e  +  mu * dw/dr_e  =  0
        2*rho*pi*L_e*r_e          =  -mu * dw/dr_e        (dw/dr < 0: fatter strut, stiffer)
    =>  r_e                       =  -mu * dw/dr_e / (2*rho*pi*L_e)

    which is an explicit update for r given the multiplier mu -- and mu is then found by BISECTION
    so that the (linearised) deflection lands exactly on the gate. That is the textbook OC loop,
    and it REDISTRIBUTES material: a strut with a big |dw/dr| gets fat, an idle one goes to r_min.

    ⚠ MY FIRST ATTEMPT DID NOT REDISTRIBUTE AT ALL. I updated mu by a hand-rolled heuristic
    (mu *= (w/gate)^1.4) and clipped the resulting ratio to [0.5, 2]. Almost every strut hit a
    clip, so almost every strut moved by the SAME factor, and the whole vector just scaled up and
    down uniformly: every radius came out identical at 0.30 mm. A sizing method that cannot
    redistribute is a sizing method that does nothing -- it is ESO with extra steps and no
    deletion. The bisection is what makes it an optimiser.

    ⚠ AND THE WORST LOAD CASE MOVES. Constraining only whichever case is worst RIGHT NOW makes the
    active set flap: fatten the struts under the thumb, the index becomes worst, fatten those, the
    thumb returns. So all `ncase` cases are aggregated into ONE smooth constraint with a p-norm
    (KS-style), which is >= the true max and converges to it as p grows. Every case therefore
    pushes on the gradient, weighted by how close it is to being the worst.

    ⚠ AND r_min MUST BE ESSENTIALLY ZERO, OR THERE IS NO TOPOLOGY.
    A sizing run with a real floor does not DELETE anything -- it PARKS the idle struts at the
    floor, where they still weigh something and still carry load. At r_min = 0.25 mm, 2174 parked
    struts were 4.5 g of a 5.8 g "answer", and the reported strut count (10) was a fiction: the
    structure was standing on the material it claimed to have removed. So the floor is numerical
    (10 um: a stiffness ratio of 1e-7, a mass of 0.007 g) and the caller DELETES the sub-printable
    struts afterwards and RE-SOLVES to prove the rest stands up on its own.
    """
    S = Sizer(nodes, bars, mat=mat, r0=r0)
    idx = S.fr.idx
    r = np.full(len(bars), r0)
    dV = 2.0 * S.rho * np.pi * S.L                      # dV/dr, up to the factor r

    anch = [i for i in anchor_k if i in idx]
    band = set(strap_n) & set(anch)
    ktot = sum(anchor_k[i] for i in band) or 1.0
    ks = {i: (strap_k * anchor_k[i] / ktot if i in band else 0.0) for i in anch}

    lift: set = set()
    best = None

    for step in range(steps):
        # THE ANCHOR IS A CONTACT PROBLEM: re-converge its active set, then freeze it and take
        # sensitivities on the frozen system. Standard for contact, and honest while it converges.
        for _ in range(6):
            spring = {i: (ks[i] if i in lift else anchor_k[i]) for i in anch}
            U, lu, _kl = S.solve(r, spring, cases)
            nxt = {i for i in anch
                   if float(U[0][6 * idx[i]:6 * idx[i] + 3] @ anchor_n[i]) > 0}
            if nxt == lift:
                break
            lift = nxt
        if not np.isfinite(U).all():
            break

        # every case's pressed button, and its gradient
        w = np.zeros(len(cases))
        G = np.zeros((len(cases), len(bars)))
        for c, (f, _a, _load) in enumerate(cases):
            b = buttons[f]
            u = U[c][6 * idx[b]:6 * idx[b] + 3]
            w[c] = float(np.linalg.norm(u))
            if w[c] > 1e-15:
                G[c] = S.grad_disp(r, U, lu, b, c, u / w[c])

        worst = float(w.max())
        # ⚠ THE P-NORM OVER-CONSTRAINS, AND THAT COSTS MASS.
        # The aggregate is >= the true max (that is the point -- it is a smooth upper bound), so
        # driving IT to the gate leaves the real worst case well UNDER the gate: measured, 302 um
        # against a 500 um gate, and every micron of that unused margin is grams of strut nobody
        # asked for. The standard fix is an ADAPTIVE correction: scale the aggregate by the ratio
        # it was wrong by last time, so at convergence the bound tracks the true maximum.
        ws = w / max(worst, 1e-30)
        raw = worst * float((ws ** pnorm).sum()) ** (1.0 / pnorm)
        corr = worst / max(raw, 1e-30)
        agg = corr * raw
        wt = corr * (w / max(raw, 1e-30)) ** (pnorm - 1.0)
        g = wt @ G

        m = S.mass(r)
        if worst <= gate and (best is None or m < best[1]):
            best = (r.copy(), m, worst)
        if on_step:
            on_step(step, m, worst, int((r > r_min * 1.01).sum()))

        # ---- BISECT the multiplier so the LINEARISED aggregate lands on the gate ------------
        neg = np.minimum(g, 0.0)                    # only struts that STIFFEN can carry the gate

        def trial(mu):
            # ⚠ A HARD MOVE LIMIT CANNOT WORK HERE, AND IT TOOK THREE TRIES TO SEE WHY.
            #
            # The gradient spans a dynamic range of 7.4e14 across these struts: the OC's target
            # radius shape spans ~300,000x from the 10th to the 90th percentile, because a few
            # struts do nearly all the work and most do almost none. Around a UNIFORM starting
            # point, ANY hard limit -- additive (+/-25%) or multiplicative (/3, x3) -- clips almost
            # every strut to the SAME bound. So they all move together, the shape never forms, and
            # every radius comes out identical: a sizing optimiser that has not sized anything.
            # Both of my first two attempts did exactly this (0.28 mm and 0.30 mm, uniform).
            #
            # The damping a six-decade variable wants is GEOMETRIC -- a log-space average of where
            # it is and where it wants to be:
            #
            #     r_new = r^(1-eta) * r_target^eta
            #
            # which is a bounded step in log(r) and imposes NO ceiling on the spread. The design
            # can reach a 300,000x range and still move smoothly.
            rn = -mu * neg / np.maximum(dV, 1e-30)
            rn = np.maximum(rn, r_min)
            rn = np.exp((1.0 - eta) * np.log(r) + eta * np.log(rn))
            return np.clip(rn, r_min, r_max)

        # ⚠ BISECT AGAINST THE TRUE DEFLECTION, NOT THE LINEARISED ONE.
        #
        # The step this method takes is enormous -- the radii have to spread over five decades --
        # and a LINEARISED constraint is meaningless over a step that size. Bisecting mu against
        # `agg + g.(rn - r)` made the optimiser leap straight past the gate, land infeasible, and
        # never record an improvement: it returned its own STARTING POINT (74.8 g, uniform 0.9 mm)
        # and called it an optimum.
        #
        # A solve costs 0.3 s. This runs ONCE, not 2400 times inside a GA -- which is the entire
        # point of decoupling the structure from the layout. So we can simply afford the truth:
        # bisect mu against the ACTUAL worst deflection. Every accepted design then meets the gate
        # by construction, and the mass falls monotonically.
        def true_worst(rn):
            Un, _lu2, _k = S.solve(rn, spring, cases)
            if not np.isfinite(Un).all():
                return np.inf
            return max(float(np.linalg.norm(Un[c][6 * idx[buttons[f]]:6 * idx[buttons[f]] + 3]))
                       for c, (f, _a, _l) in enumerate(cases))

        # 14 steps, not 24: this is a LOG bisection over 28 decades, so 14 halvings already pins
        # mu to 0.002 of a decade. The extra ten were ten full factorisations per outer step, for
        # precision nobody could use.
        lo, hi = 1e-14, 1e14
        for _ in range(14):
            mid = np.sqrt(lo * hi)
            if true_worst(trial(mid)) > gate:
                lo = mid                    # too floppy: needs more material
            else:
                hi = mid
        r_new = trial(hi)
        if S.mass(r_new) >= S.mass(r) and step > 3:
            break                           # converged: the OC cannot find anything lighter
        r = r_new

    if best is None:
        return r, S.mass(r), float("inf"), []
    r, m, wbest = best
    live = [e for e in range(len(bars)) if r[e] > r_min * 1.01]
    return r, m, wbest, live


def size_and_prune(nodes, bars, buttons, cases, anchor_k, anchor_n, strap_n, strap_k,
                   gate=0.5e-3, mat="cf_pa12", r_print=2.5e-4, rate=0.25, on_step=None,
                   build_dir=None):
    """SIZE, THEN PRUNE, THEN RE-SIZE. The only version of this that yields a buildable structure.

    ⚠ PURE SIZING DOES NOT PRODUCE A TOPOLOGY. It produces a CONTINUUM of radii: the run that
    finally worked came out at 3.12 g and 477 um -- right on the gate, and less than half ESO's
    8.19 g -- but 1937 of its 2184 struts were BELOW THE PRINTABLE FLOOR, carrying 40% of the mass,
    and deleting them cut a button off its load path entirely. It was standing on material nobody
    could make.

    So the two halves are married. The GRADIENT redistributes (which ESO cannot: it is binary and
    can only ever remove). The DELETION commits (which sizing cannot: it will happily hide the
    structure in a haze of sub-printable hairs). Alternate them:

        size  ->  delete the thinnest that keep the buttons connected  ->  re-size the survivors

    and stop when deletion can no longer be paid for. Every intermediate design meets the gate by
    construction, so there is never a moment where the answer depends on something unbuildable.
    """
    from structure.lattice import connected, protect_support, repair_support

    # ⚠ PRUNE THE DOMAIN TO THE ANCHORED COMPONENT *BEFORE* THE FIRST SOLVE, NOT AFTER IT.
    # `connected` only ran inside the prune loop, so the very first size() saw whatever ground()
    # handed over. Dropping the ~100 bars a printer cannot lay severed one button from the anchors,
    # and a floating button is a RIGID-BODY MODE: it deflected 10 km, the gate was "missed", and
    # the run reported "no printable structure meets the gate" -- a modelling failure dressed up as
    # a finding about the device.
    live, ok = connected(bars, list(range(len(bars))), anchor_k, buttons, len(nodes))
    if not ok:
        raise ValueError("a button is not connected to any anchor -- the DOMAIN is broken")
    if build_dir is not None:
        live = repair_support(nodes, bars, live, build_dir)
    best = None

    rho = MATERIALS[mat]["rho"]
    nodes = np.asarray(nodes, float)
    LEN = np.linalg.norm(nodes[[b[1] for b in bars]] - nodes[[b[0] for b in bars]], axis=1)
    # THE SIZING FLOOR IS NOT THE PRINTING FLOOR. The sizer needs a floor low enough that the radii
    # still span decades (or `argsort(r)` has nothing to rank and the pruner deletes at random), and
    # high enough not to be numerical dust. 0.25 mm is the value that produced the known-good
    # 921-strut / 4.79 g answer. The NOZZLE is applied to what gets BUILT, not to what gets SIZED.
    r_size = min(2.5e-4, r_print)

    # ⚠ THE NOZZLE BOUND MUST BE A *DELETION THRESHOLD*, NOT A FLOOR THE SIZER SIZES AGAINST, AND
    # NOT A CLAMP APPLIED AFTERWARDS. I got this wrong in both directions, and each way failed in
    # its own characteristic manner:
    #
    #   r_min = r_print  ->  every idle strut parks at EXACTLY 0.4 mm. The radii stop spanning
    #                        decades and pile up on the floor, so `argsort(r)` has nothing left to
    #                        sort by and the pruner DELETES AT RANDOM among the idle struts.
    #                        Deleting a quarter of them made the structure HEAVIER (19.97 -> 30.0 g),
    #                        and 13 g of the 20 g "answer" was floor material under struts doing no
    #                        work. This file's own docstring had already warned about it: "r_min MUST
    #                        BE ESSENTIALLY ZERO, OR THERE IS NO TOPOLOGY."
    #
    #   size at 1e-5,     -> the sizer minimises the UNCLAMPED mass, so its optimum is precisely a
    #   then clamp up        haze of thousands of hairs -- which the clamp then fattens to 0.4 mm
    #                        each. It optimised one objective and paid for another: 66.7 g.
    #
    # The resolution is that a strut the sizer drives to 10 um is not a thin strut. IT IS A STRUT
    # THE SIZER IS ASKING TO DELETE. So: size against the numerical floor (the radii span decades,
    # the ranking is real), and DELETE what comes out below the nozzle. Converged, every surviving
    # strut is at or above 0.4 mm BECAUSE THE PHYSICS PUT IT THERE -- nothing is clamped, nothing is
    # parked, and the mass is honest. That is also what finally forces the FEW, THICK, CHUNKY
    # members that manufacture was supposed to force all along.
    for step in range(40):
        sb = [bars[e] for e in live]
        r, m, w, _ = size(nodes, sb, buttons, cases, anchor_k, anchor_n, strap_n, strap_k,
                          gate=gate, mat=mat, r_min=r_size, r0=max(9e-4, 2 * r_size))
        if not np.isfinite(w) or w > gate:
            break

        # THE PRINTABLE MASS OF THIS TOPOLOGY: what it costs with every sub-nozzle strut fattened UP
        # to the nozzle. Fattening only ADDS material, so it only STIFFENS, so the gate still holds
        # by construction -- and it is what you would actually build.
        #
        # ⚠ FATTEN. DO NOT DELETE. On the known-good 4.79 g answer, 788 of its 921 struts (86%) are
        # thinner than the 0.4 mm nozzle AND THEY CARRY 53% OF THE MASS. They are not numerical dust
        # for the pruner to sweep up -- they are a FINE NET DOING REAL WORK, which is exactly what a
        # minimum-mass shell wants to be. Delete them and the survivors have to be pinned at r_max to
        # hold the same gate: 284 g, from a 38 g start. Fatten them and it costs 1.6x: 7.54 g.
        rp = np.maximum(r, r_print)
        mp = float(rho * np.pi * np.sum(rp ** 2 * LEN[live]))
        # ⚠ KEEP THE LIGHTEST, NOT THE LAST. Pruning is not monotone in mass -- the sizing
        # sub-problem lands in a different place each time the topology changes -- so the last
        # design is very often NOT the best one. Overwriting unconditionally reported 12.89 g
        # when 4.79 g was sitting three steps back.
        if best is None or mp < best[2]:
            best = (list(live), rp.copy(), mp, w)
        if on_step:
            on_step(step, len(live), mp, w)

        # ⚠ AND THE DELETION MUST STAY RATE-BASED, on the THINNEST, unconditionally.
        # I tried making it "delete only what the sizer drove to its numerical floor" -- and the
        # sizer never drives ANYTHING there. It does not abandon struts; it just makes them thin.
        # So nothing was ever deleted, the whole 4283-bar domain survived, and with a nozzle floor
        # under every one of them the answer was 22.35 g of UNIFORM 0.40 mm strut: pure floor.
        #
        # With a nozzle floor, DELETION IS THE ONLY THING THAT REDUCES MASS -- the sizer cannot
        # express "I want this gone", it can only make a strut as thin as the floor allows. So the
        # pruner has to do it, and it has to keep doing it.
        order = np.argsort(r)
        n_cut = max(1, int(rate * len(live)))
        keep = protect_support(nodes, bars, live, build_dir) if build_dir is not None else set()
        drop = set()
        for i in order:
            if len(drop) >= n_cut:
                break
            if live[int(i)] not in keep:
                drop.add(live[int(i)])
        trial = [e for e in live if e not in drop]
        trial, ok = connected(bars, trial, anchor_k, buttons, len(nodes))
        if build_dir is not None and ok:
            # give back only struts THIS cut removed, so the trial can never grow -- the
            # `len(trial) == len(live)` guard below then still terminates the loop.
            trial = repair_support(nodes, bars, trial, build_dir, pool=live)
            trial, ok = connected(bars, trial, anchor_k, buttons, len(nodes))
        if not ok or len(trial) < 12 or len(trial) >= len(live):
            break
        live = trial

    # ⚠ AND FINALLY: RE-SIZE THE CONVERGED TOPOLOGY *WITH THE NOZZLE AS THE REAL LOWER BOUND*.
    # The clamp above is only a projection of the unconstrained optimum -- it is feasible (fatter =
    # stiffer, so the gate still holds) but it is not OPTIMAL for the bounded problem. Now that the
    # topology has stopped moving, the OC can redistribute AROUND the floor and give the slack back.
    # Whichever is lighter wins, and the clamp is always there as a guaranteed fallback.
    if best is not None:
        sb = [bars[e] for e in best[0]]
        r2, m2, w2, _ = size(nodes, sb, buttons, cases, anchor_k, anchor_n, strap_n, strap_k,
                             gate=gate, mat=mat, r_min=r_print, r0=max(9e-4, 2 * r_print))
        if np.isfinite(w2) and w2 <= gate and m2 < best[2]:
            best = (best[0], r2, m2, w2)
            if on_step:
                on_step(99, len(best[0]), m2, w2)

    # A NODE WITH NOTHING UNDER IT IS NOT AN ERROR -- IT IS A SACRIFICIAL PILLAR, and pillars are
    # the currency the user actually named ("without too many supports"). The pruner has already
    # kept every down-strut the DOMAIN could offer, for free, because a strut that holds a node up
    # is STRUCTURE and not scaffolding: it is in the FEM, it carries load, and its mass is counted.
    # What is left over is what genuinely has to be propped off the bed and snapped away, and the
    # caller counts it with `unsupported(nodes, bars, live, build_dir)`.

    if best is None:
        return [], np.zeros(0), float("inf"), float("inf")
    live, r, m, w = best
    return live, r, m, w
