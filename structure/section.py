"""BONE IS NOT A ROUND ROD. It is an ELLIPSE, turned to meet the bending, and it TAPERS.

THE USER: "I think the thickness of struts should be a spline too, with a major and minor radius,
and principal orientation as a spline."

WHY THIS IS NOT COSMETIC, AND IT IS THE FIRST THING IN THIS PROJECT WITH A LARGE STRUCTURAL PAYOFF.

A CIRCLE IS THE WORST POSSIBLE SECTION FOR A MEMBER THAT BENDS IN ONE PLANE. For an ellipse of
semi-axes a, b:

    mass          A  = pi a b
    stiffness     Iz = pi a^3 b / 4   (bending in one plane)
                  Iy = pi a b^3 / 4   (bending in the other)

so AT CONSTANT AREA -- at constant MASS -- material can be moved out of the direction nothing is
pushing and into the direction that is being bent. A round rod spends material providing stiffness
in a direction nothing loads. That is why I-beams exist, and it is why a long bone's cross-section
is an ellipse whose principal axis lines up with the bending it actually sees.

THE REPARAMETERISATION THAT MAKES IT FREE. Write

    a = s * sqrt(k),   b = s / sqrt(k)          s = the SCALE,  k = the ASPECT RATIO (a/b)

and then

    A  = pi s^2                 <-- THE MASS DEPENDS ONLY ON s. The aspect is free, mass-wise.
    Iz = pi s^4 k / 4
    Iy = pi s^4 / (4 k)
    J  = pi s^4 / (k + 1/k)

Every term is still s^2 or s^4, EXACTLY as in the circular case -- so the existing optimality-criteria
sizer works on `s` unchanged, and `k` merely modulates the bending coefficients.

AND NEITHER k NOR THE ROLL NEEDS A GRADIENT. Both fall out of the solved moments:

    ROLL  = the PRINCIPAL MOMENT DIRECTION.  Turn the section to meet the bending.
    k     = |M1| / |M2|.                     Minimising  M1^2/k + M2^2 k  over k gives exactly the
                                             moment ratio -- so the section is proportioned to the
                                             bending it sees, in both planes.

Solve, align, re-solve. That is a fixed point, and it is LITERALLY WOLFF'S LAW: align with the
principal stress, and proportion to it.

⚠ AND THE SECTION IS NOT AN ELLIPSE. IT IS A STADIUM, AND THE REASON IS THE WHOLE POINT.

An ELLIPSE was the obvious thing and it FAILS -- measured, +4% mass, and only 14 of 153 members even
went elliptical. FOR AN ELLIPSE, FLAT *MEANS* SHARP. Its tightest convex curvature is at the ends of
the major axis, radius b^2/a, so flattening it drives that radius to ZERO: a 2:1 ellipse with
a = 2 mm has a 0.5 mm tip, sharper than the 1.41 mm circle of the same area. And the user's entire
reason for wanting this was that a keyboard must not be a knuckle-duster, so the ergonomic floor
applies to exactly that radius:

    b^2 / a  >=  SKIN_R          <=>          s >= SKIN_R * k^1.5

which is brutal: a 3:1 ellipse would need s >= 1.5 * 3^1.5 = 7.8 mm. Measured, the aspect capped out
at 1.41:1 and the section bought nothing. THE ERGONOMIC FLOOR AND THE ELLIPSE ARE IN DIRECT CONFLICT.

A STADIUM IS BOTH. It is a rectangle with SEMICIRCULAR ENDS -- the sweep of a circle of radius b
along a straight segment of half-length c:

    FLAT SIDES        -> all the bending efficiency the ellipse promised
    ROUND CAPS        -> the minimum surface radius is simply b. Not b^2/a. JUST b.

so the friendly constraint collapses to  b >= SKIN_R, satisfiable at ANY flatness. A stadium can be
as flat as you like and still be blunt. It is also a nicer object in every other way: it is the sweep
of a SPHERE along a RIBBON, which is precisely the capsule SDF the mesher already speaks.

    A      = 4bc + pi b^2
    I_flat = 4bc^3/3 + pi b^4/4 + pi b^2 c^2      (resists bending in the FLAT plane -- the big one)
    I_thin = 4cb^3/3 + pi b^4/4                   (across it -- the small one)
    J      = 4 I_flat I_thin / (I_flat + I_thin)  <-- EXACT for a circle AND for an ellipse, so it is
                                                      a well-founded approximation for a stadium too

and c = 0 recovers the circle exactly. THAT is the regression.
"""
from __future__ import annotations

import numpy as np

from design.params import P, Source
from structure.fem import Frame, local_axes
from structure.frame import MATERIALS

K_MAX = P("ASPECT_MAX", 3.0, "-", Source.GUESS,
          "The flattest a member is allowed to get (total width / total height). Unbounded, the "
          "optimiser would drive a purely-bent member to a ribbon -- which buckles laterally, prints "
          "badly on edge, and is not what a bone does either. 3:1 is about the flattest a trabecula "
          "gets. NOT derived: a guess, and the mass it costs is swept.")


def _basis(nodes, bars, E, G):
    """The four 12x12 'basis' stiffnesses of each element, one per section property.

    The element stiffness is LINEAR in (A, Iy, Iz, J), so k_e = A*K_A + Iy*K_Iy + Iz*K_Iz + J*K_J
    with the K's constant. Every gradient with respect to the section then follows immediately, and
    nothing has to be re-assembled.
    """
    from structure.fem import _element_k

    out = []
    for (i, j) in bars:
        L = float(np.linalg.norm(np.asarray(nodes)[j] - np.asarray(nodes)[i]))
        KA = _element_k(L, E, G, 1.0, 0.0, 0.0, 0.0)
        KIy = _element_k(L, E, G, 0.0, 1.0, 0.0, 0.0)
        KIz = _element_k(L, E, G, 0.0, 0.0, 1.0, 0.0)
        KJ = _element_k(L, E, G, 0.0, 0.0, 0.0, 1.0)
        out.append((KA, KIy, KIz, KJ))
    return [np.array([o[q] for o in out]) for q in range(4)]


class Ellipse:
    """A frame whose members are ELLIPSES: a scale `s`, an aspect `k`, and a roll, per element."""

    def __init__(self, nodes, bars, mat="cf_pa12"):
        self.nodes = np.asarray(nodes, float)
        self.bars = [tuple(b) for b in bars]
        p = MATERIALS[mat]
        self.E, self.G, self.rho = p["E"], p["E"] / 2.6, p["rho"]
        self.L = np.array([np.linalg.norm(self.nodes[j] - self.nodes[i]) for i, j in self.bars])
        self.KA, self.KIy, self.KIz, self.KJ = _basis(self.nodes, self.bars, self.E, self.G)
        # a Frame only for its node map and dof bookkeeping -- the stiffness is built here
        self.fr = Frame(self.nodes, self.bars, self.E, self.G, 1e-6, 1e-12, 2e-12)

    #                                    A TUBE.
    #
    # ⚠ AND THE STADIUM BOUGHT NOTHING, WHICH IS THE MOST USEFUL THING IT COULD HAVE DONE.
    # Measured: 0 of 153 members flattened, +5% mass. And the reason is the finding of the whole
    # exercise -- 146 OF THE 153 MEMBERS (95%) SIT ON THE ERGONOMIC FLOOR. Their size is set by a
    # HAND, not by a force: they are already thicker than their load requires. A clever SECTION can
    # only pay where the section is set by the LOAD, and here almost none of it is.
    #
    # THE FRIENDLY GAUNTLET IS NOT STIFFNESS-LIMITED OR STRENGTH-LIMITED. IT IS TOUCH-LIMITED.
    #
    # Which hands us the answer, and it is the one a bone already uses. If the OUTER surface is what
    # has to be friendly, and the inside is doing nothing -- HOLLOW IT OUT. A tube of outer radius b
    # and wall w is EXACTLY as blunt (its outer radius is still b) and far lighter, and it barely
    # loses any stiffness, because bending stiffness lives in the OUTER FIBRES: the material near
    # the axis was contributing almost nothing. Measured below.
    #
    # And it is free to print: a 0.8 mm wall is TWO PERIMETERS of a 0.4 mm nozzle. A hollow strut is
    # a strut printed with no infill. The printer was going to do this anyway.
    #
    # A long bone is a tube with a marrow cavity. That is not an analogy -- it is the same
    # optimisation, under the same constraint.
    WALL = 8.0e-4        # m -- two perimeters of a 0.4 mm nozzle. The thinnest wall FDM will lay.

    def area(self, b, t):
        """The annulus.  `t` is retained for the stadium experiment; a tube ignores it."""
        _ = t
        ri = np.maximum(b - self.WALL, 0.0)
        return np.pi * (b ** 2 - ri ** 2)

    def mass(self, b, t):
        return float(self.rho * np.sum(self.area(b, t) * self.L))

    def props(self, b, t):
        """A TUBE of outer radius b and wall WALL. Isotropic, so Iy = Iz and the roll is irrelevant.

        A wall thicker than the radius is a solid rod, which is what a thin member becomes -- so this
        degrades gracefully to the round rod and does not have to be special-cased.
        """
        _ = t
        ri = np.maximum(b - self.WALL, 0.0)
        A = np.pi * (b ** 2 - ri ** 2)
        I = np.pi * (b ** 4 - ri ** 4) / 4.0
        return A, I, I, 2.0 * I

    def dprops(self, b, t):
        """d(A, Iy, Iz, J)/db. The inner radius moves with the outer one, so both terms count."""
        _ = t
        ri = np.maximum(b - self.WALL, 0.0)
        dri = (ri > 0).astype(float)               # dri/db = 1 while there IS a bore, else 0
        dA = 2.0 * np.pi * (b - ri * dri)
        dI = np.pi * (b ** 3 - ri ** 3 * dri)
        return dA, dI, dI, 2.0 * dI

    def _T(self, roll):
        """The 12x12 rotation of each element, at its own roll."""
        T = np.zeros((len(self.bars), 12, 12))
        for e, (i, j) in enumerate(self.bars):
            ex, ey, ez, _L = local_axes(self.nodes[j] - self.nodes[i], float(roll[e]))
            R = np.vstack([ex, ey, ez])
            for q in range(4):
                T[e, 3 * q:3 * q + 3, 3 * q:3 * q + 3] = R
        return T

    def solve(self, s, k, roll, spring, cases):
        """Displacements, and the element end-forces, at this section."""
        from scipy.sparse import coo_matrix
        from scipy.sparse.linalg import splu

        A, Iy, Iz, J = self.props(s, k)
        kl = (A[:, None, None] * self.KA + Iy[:, None, None] * self.KIy
              + Iz[:, None, None] * self.KIz + J[:, None, None] * self.KJ)
        T = self._T(roll)
        kg = np.einsum("bji,bjk,bkl->bil", T, kl, T)

        d = self.fr.dofs
        n = self.fr.ndof
        rows = np.repeat(d, 12, axis=1).ravel()
        cols = np.tile(d, (1, 12)).ravel()
        sr, sv = [], []
        for i, kk in spring.items():
            if i in self.fr.idx:
                for q in range(3):
                    sr.append(6 * self.fr.idx[i] + q)
                    sv.append(kk)
        K = coo_matrix((np.concatenate([kg.ravel(), np.array(sv)]),
                        (np.concatenate([rows, np.array(sr, int)]),
                         np.concatenate([cols, np.array(sr, int)]))), shape=(n, n)).tocsc()
        K = K + 1e-6 * coo_matrix((np.ones(n), (np.arange(n), np.arange(n))), shape=(n, n)).tocsc()
        lu = splu(K)
        B = np.zeros((n, len(cases)))
        for c, (_f, _a, load) in enumerate(cases):
            for i, fv in load.items():
                if i in self.fr.idx:
                    B[6 * self.fr.idx[i]:6 * self.fr.idx[i] + 3, c] = fv
        U = lu.solve(B).T
        return U, lu, kl, T

    def align(self, U, T, kl, k_max=None, b=None, b_min=None):
        """WOLFF'S LAW, as a fixed point: turn each section to the bending, and flatten it to match.

        From the solved end-forces, the moment about each local axis. The section is turned so its
        FLAT direction takes the LARGER moment, and its flatness is set so that I_flat/I_thin equals
        the ratio of the two moments -- because minimising the compliance  M1^2/I_flat + M2^2/I_thin
        at fixed AREA gives exactly that.

        ⚠ AND THE ERGONOMIC FLOOR NO LONGER APPEARS HERE AT ALL. For an ellipse it did, and it
        strangled the whole idea: the tip radius b^2/a meant flat implied sharp, and the aspect
        capped out at 1.41:1. A STADIUM's minimum surface radius is just `b`, so the friendly
        constraint is a plain lower bound on b -- handled by the sizer, uncoupled from the flatness.
        The section can be as flat as it likes and stay blunt.
        """
        k_max = float(K_MAX) if k_max is None else float(k_max)
        ul = np.einsum("bij,cbj->cbi", T, U[:, self.fr.dofs])
        f = np.einsum("bij,cbj->cbi", kl, ul)                    # (ncase, nbar, 12)
        My = np.abs(np.stack([f[:, :, 4], f[:, :, 10]])).max(axis=(0, 1))
        Mz = np.abs(np.stack([f[:, :, 5], f[:, :, 11]])).max(axis=(0, 1))

        # turn the section so its FLAT plane (the one Iz resists) takes the resultant moment
        dtheta = np.arctan2(My, np.maximum(Mz, 1e-30))
        M1 = np.hypot(My, Mz)
        M2 = np.minimum(My, Mz)
        ratio = np.clip(M1 / np.maximum(M2, 1e-3 * M1 + 1e-30), 1.0, 1e4)
        t = _t_for_ratio(ratio, max(k_max - 1.0, 0.0))

        # ⚠ ONLY FLATTEN A MEMBER WHOSE SIZE IS SET BY *LOAD*, NOT BY THE *FLOOR*.
        #
        # This is the whole reason the first attempt bought nothing. Every one of the 153 members
        # WANTS to be flat -- the moment ratio is 1.9 median, up to 11 -- but most of them are
        # sitting ON the ergonomic floor, and a member at the floor is already THICKER THAN ITS LOAD
        # REQUIRES. Flattening it raises the area (A = b^2(4t+pi) > pi b^2) to buy stiffness it does
        # not need: pure mass, and the optimiser rightly threw it away.
        #
        # A flat section pays only where the section is set by the LOAD, because only there can the
        # member then SHRINK. So: floor-bound members stay round. That is also the right answer for
        # them -- for a given area, a circle is the BLUNTEST section there is.
        if b is not None and b_min is not None:
            t = np.where(b > float(b_min) * 1.02, t, 0.0)
        return dtheta, t, M1, M2


def _t_for_ratio(ratio, t_max):
    """The stadium flatness t whose I_flat / I_thin equals a given moment ratio.

    I_flat/I_thin is monotone in t and t = 0 gives exactly 1 (a circle), so a bisection inverts it.
    """
    lo = np.zeros_like(ratio)
    hi = np.full_like(ratio, max(float(t_max), 1e-9))
    for _ in range(28):
        mid = 0.5 * (lo + hi)
        r = ((4.0 * mid ** 3 / 3.0 + np.pi / 4.0 + np.pi * mid ** 2)
             / (4.0 * mid / 3.0 + np.pi / 4.0))
        lo = np.where(r < ratio, mid, lo)
        hi = np.where(r < ratio, hi, mid)
    return 0.5 * (lo + hi)


def _grad_b(EL, b, t, U, lu, T, node, case, direction):
    """d|u_node . direction| / db_e for every element. ONE back-substitution -- the adjoint again.

    The element stiffness is LINEAR in (A, Iy, Iz, J), and every one of those is homogeneous in b
    (degree 2 for the area, degree 4 for the moments), so dk_e/db is just another combination of the
    same four basis matrices. Nothing is re-assembled and nothing is finite-differenced.
    """
    q = np.zeros(EL.fr.ndof)
    base = 6 * EL.fr.idx[node]
    q[base:base + 3] = direction
    lam = lu.solve(q)

    d = EL.fr.dofs
    ul = np.einsum("bij,bj->bi", T, U[case][d])
    ll = np.einsum("bij,bj->bi", T, lam[d])
    dA, dIy, dIz, dJ = EL.dprops(b, t)
    dk = (dA[:, None, None] * EL.KA + dIy[:, None, None] * EL.KIy
          + dIz[:, None, None] * EL.KIz + dJ[:, None, None] * EL.KJ)
    return -np.einsum("bi,bij,bj->b", ll, dk, ul)


def size_stadium(nodes, bars, buttons, cases, anchor_k, anchor_n, strap_n, strap_k,
                 gate=0.5e-3, mat="cf_pa12", b_min=4.0e-4, b_max=2.5e-3, b0=9.0e-4,
                 outer=5, steps=14, eta=0.5, on_step=None, k_max=None):
    """Minimum mass at the gate, with STADIUM sections turned and flattened to meet the bending.

    TWO NESTED FIXED POINTS, and only the inner one needs a gradient:

      INNER -- optimality criteria on `b`, the BLUNTNESS, which is also the section's minimum
               surface radius. So the ergonomic floor is simply b >= SKIN_R: a plain lower bound.
      OUTER -- WOLFF'S LAW on the shape: turn each section's FLAT direction to its own principal
               moment, and set its flatness so I_flat/I_thin matches the ratio of the two moments.
               No gradient: read straight off the solved end-forces.

    t = 0 everywhere is the round-rod structure, EXACTLY. That is the regression.
    """
    EL = Ellipse(nodes, bars, mat=mat)
    idx = EL.fr.idx
    nb = len(bars)
    b = np.full(nb, b0)
    t = np.zeros(nb)
    roll = np.zeros(nb)

    anch = [i for i in anchor_k if i in idx]
    band = set(strap_n) & set(anch)
    ktot = sum(anchor_k[i] for i in band) or 1.0
    ks = {i: (strap_k * anchor_k[i] / ktot if i in band else 0.0) for i in anch}

    best = None
    lift: set = set()

    def solve_at(bv, tv, rv):
        nonlocal lift
        for _ in range(6):
            spring = {i: (ks[i] if i in lift else anchor_k[i]) for i in anch}
            U, lu, kl, T = EL.solve(bv, tv, rv, spring, cases)
            nxt = {i for i in anch
                   if float(U[0][6 * idx[i]:6 * idx[i] + 3] @ anchor_n[i]) > 0}
            if nxt == lift:
                break
            lift = nxt
        return U, lu, kl, T

    def worst_at(bv, tv, rv):
        U, _lu, _kl, _T = solve_at(bv, tv, rv)
        if not np.isfinite(U).all():
            return np.inf
        return max(float(np.linalg.norm(U[c][6 * idx[buttons[f]]:6 * idx[buttons[f]] + 3]))
                   for c, (f, _a, _l) in enumerate(cases))

    for it in range(outer):
        for step in range(steps):
            U, lu, kl, T = solve_at(b, t, roll)
            if not np.isfinite(U).all():
                break
            dV = EL.rho * EL.L * (2.0 * b * (4.0 * t + np.pi))       # dMass/db
            w = np.zeros(len(cases))
            Gm = np.zeros((len(cases), nb))
            for c, (f, _a, _l) in enumerate(cases):
                bt = buttons[f]
                u = U[c][6 * idx[bt]:6 * idx[bt] + 3]
                w[c] = float(np.linalg.norm(u))
                if w[c] > 1e-15:
                    Gm[c] = _grad_b(EL, b, t, U, lu, T, bt, c, u / w[c])
            worst = float(w.max())
            m = EL.mass(b, t)
            if worst <= gate and (best is None or m < best[3]):
                best = (b.copy(), t.copy(), roll.copy(), m, worst)
            if on_step:
                on_step(it, step, m, worst, float(np.mean(1.0 + t)))

            ws = w / max(worst, 1e-30)
            raw = worst * float((ws ** 8.0).sum()) ** (1.0 / 8.0)
            corr = worst / max(raw, 1e-30)
            g = (corr * (w / max(raw, 1e-30)) ** 7.0) @ Gm
            neg = np.minimum(g, 0.0)

            def trial(mu, b=b, dV=dV):
                bn = -mu * neg / np.maximum(dV, 1e-30) * b
                bn = np.maximum(bn, b_min)
                bn = np.exp((1.0 - eta) * np.log(b) + eta * np.log(bn))
                return np.clip(bn, b_min, b_max)

            lo, hi = 1e-14, 1e14
            for _ in range(12):
                mid = np.sqrt(lo * hi)
                if worst_at(trial(mid), t, roll) > gate:
                    lo = mid
                else:
                    hi = mid
            b_new = trial(hi)
            if EL.mass(b_new, t) >= EL.mass(b, t) and step > 2:
                break
            b = b_new

        # ---- WOLFF: turn the sections to the bending, and flatten them to match -----------------
        U, _lu, kl, T = solve_at(b, t, roll)
        dtheta, t_new, _M1, _M2 = EL.align(U, T, kl, k_max=k_max, b=b, b_min=b_min)
        roll = roll + dtheta
        t = t_new

    if best is None:
        return b, t, roll, EL.mass(b, t), float("inf"), EL
    b, t, roll, m, w = best
    return b, t, roll, m, w, EL
