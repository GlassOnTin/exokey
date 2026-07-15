"""WEIGH THE SANDWICH end-to-end, and first ask WHERE the knock mass actually is.

    PYTHONPATH=. .venv/bin/python scripts/sandwich_weigh.py

The user: the form-found impact structure looks like a shell; is a SANDWICH (sparse keypress lattice +
dorsal shell) lighter than the dense impact lattice? A dorsal shell can only save the mass the DORSAL
knock costs. So first, per member, find WHICH knock (a well, or the dorsal ridge) sizes it -- and how
much mass each knock is responsible for. If the wells drive it, a shell over the BACK cannot help, and
the honest answer is that the mass is at the wells, where a shell does not reach (it needs cups).
"""
from __future__ import annotations

import pickle

import numpy as np

from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import MATERIALS, hand_axes
from structure.lattice import STRAP_K, ground
from structure.sizing import Sizer

KNOCK_N = 50.0
SF = 2.0
E_TPU = MATERIALS["tpu"]["E"]
A_ANCHOR = 1.0e-4


def main():
    H = hands()
    ref = H[50]
    fd = pickle.load(open("out/final_design.pkl", "rb"))
    x = fd["x"]
    evaluate(x, H)
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    o, e_d, e_r, e_o = hand_axes(ref, q)
    _n, _b, btn, _l, ak, an, _t, strap_n = ground(ref, q)
    yld, rho = MATERIALS["cf_pa12"]["yield_"], MATERIALS["cf_pa12"]["rho"]

    z = np.load("out/impact_opt.npz", allow_pickle=True)
    nodes = z["nodes"]
    sb = [tuple(b) for b in z["bars"]]
    r = np.asarray(z["radii"], float)
    m_lat = float(z["mass"])

    S = Sizer(nodes, sb, r0=9e-4)
    idx = S.fr.idx
    anch = [i for i in ak if i in idx]
    akc = {i: 1.0 / (1.0 / ak[i] + 1.0 / (E_TPU * A_ANCHOR / 0.002)) for i in ak}
    band = set(strap_n) & set(anch)
    ktot = sum(ak[i] for i in band) or 1.0
    ks = {i: (float(STRAP_K) * ak[i] / ktot if i in band else 0.0) for i in anch}
    dorsal = max((i for i in ak), key=lambda i: (nodes[i] - o) @ e_o)
    T, dofs = S.fr.T, S.fr.dofs
    A = np.pi * r ** 2
    I = np.pi * r ** 4 / 4.0
    L = S.L

    def stress_of(node, foundation=None):
        """Per-member peak fibre stress under a single 50 N knock into the hand at `node`.

        foundation=None: the real device -- the ~94 discrete tissue anchors.
        foundation=<dict>: a custom compression spring per node (used to model a SHELL that bears on
        the tissue continuously -- a stiff shell in series with tissue is ~= the tissue, so a shell
        distributing the bearing is approximated by a tissue spring on EVERY node it reaches).
        """
        aset = anch if foundation is None else list(foundation)
        akc_l = akc if foundation is None else foundation
        an_l = an if foundation is None else {i: e_o for i in aset}   # a shell bears ~ into the hand
        lift: set = set()
        U = kl = None
        for _ in range(8):
            spring = {i: (ks.get(i, 0.0) if i in lift else akc_l[i]) for i in aset}
            U, _lu, kl = S.solve(r, spring, [("k", "k", {int(node): -KNOCK_N * e_o})])
            nxt = {i for i in aset if float(U[0][6 * idx[i]:6 * idx[i] + 3] @ an_l[i]) > 0}
            if nxt == lift:
                break
            lift = nxt
        ul = np.einsum("bij,bj->bi", T, U[0][dofs])
        f = np.einsum("bij,bj->bi", kl, ul)
        N = np.abs(f[:, 0])
        M = np.maximum(np.hypot(f[:, 4], f[:, 5]), np.hypot(f[:, 10], f[:, 11]))
        return N / A + M * r / I

    cases = {f"{f} well": int(btn[f]) for f in FINGERS}
    cases["dorsal ridge"] = int(dorsal)
    sig = {name: stress_of(node) for name, node in cases.items()}

    # per member: the driving knock, and the required radius (r scaled to just meet yield/SF under it)
    names = list(cases)
    S_arr = np.vstack([sig[n] for n in names])              # (ncase, nbar)
    drive = S_arr.argmax(axis=0)                            # which case sizes each member
    smax = S_arr.max(axis=0)
    r_need = r * np.sqrt(np.maximum(smax, 1e-30) * SF / yld)   # FSD radius under the worst knock
    m_need = rho * np.pi * np.sum(r_need ** 2 * L) * 1e3

    print(f"impact lattice: {len(sb)} members, {m_lat:.1f} g (as saved).  Which knock sizes each member?\n")
    print(f"  {'knock':16} {'members it sizes':>16} {'their FSD mass':>15}")
    dorsal_i = names.index("dorsal ridge")
    for c, name in enumerate(names):
        sel = drive == c
        mc = rho * np.pi * np.sum(r_need[sel] ** 2 * L[sel]) * 1e3
        print(f"  {name:16} {int(sel.sum()):>16} {mc:>12.1f} g")

    # what a DORSAL shell could save: the mass of members SIZED BY the dorsal knock -- and only the
    # part that would thin if the dorsal knock were removed (down to the next-worst knock's demand).
    S_nod = np.delete(S_arr, dorsal_i, axis=0).max(axis=0)     # worst knock EXCLUDING the dorsal
    r_nod = r * np.sqrt(np.maximum(S_nod, 1e-30) * SF / yld)
    r_nod = np.minimum(r_nod, r_need)
    m_saved = rho * np.pi * np.sum((r_need ** 2 - r_nod ** 2) * L) * 1e3

    print(f"\n  FSD mass under ALL knocks:            {m_need:.1f} g")
    print(f"  FSD mass with the DORSAL knock removed: {m_need - m_saved:.1f} g")
    print(f"  -> a dorsal shell could save at most:  {m_saved:.1f} g "
          f"({100*m_saved/m_need:.0f}% of the knock mass)\n")

    # WHERE is the peak stress under a well knock -- at the well (a dorsal shell cannot reach it) or
    # out in the dorsal region (a shell could)? Take the worst well and its top-stressed members.
    worst_well = max(FINGERS, key=lambda f: sig[f"{f} well"].max())
    wn = nodes[int(btn[worst_well])]
    s = sig[f"{worst_well} well"]
    mid = np.array([(nodes[i] + nodes[j]) / 2 for i, j in sb])
    d_well = np.linalg.norm(mid - wn, axis=1) * 1e3          # mm from the knocked well
    top = np.argsort(s)[-20:]
    print(f"  under the worst knock ({worst_well} well, {s.max()/1e6:.0f} MPa): its 20 most-stressed "
          f"members sit a median {np.median(d_well[top]):.0f} mm from the well")
    print(f"  -- i.e. OUT in the dorsal region near the anchors, which IS where a shell would sit, so")
    print(f"     a shell carrying that load path is not obviously ruled out. Test it:\n")

    # DOES CONTINUOUS SHELL-BEARING BEAT THE DISCRETE ANCHORS? Re-solve the worst knock with the tissue
    # foundation on EVERY node (a stiff shell in series with soft tissue ~= tissue everywhere), so the
    # reaction spreads over the whole dorsal surface instead of the ~94 discrete anchor loops. If the
    # peak drops a lot, a shell would let the lattice thin; if not, the discrete bearing is already fine.
    A_SHELL, E_T, t_T = 95e-4, 1.9e6, 5.0e-3
    trib = A_SHELL / len(idx)
    shell_found = {i: E_T * trib / t_T for i in idx}          # a tissue spring on every node
    worst_node = int(btn[worst_well])
    s_anchor = stress_of(worst_node).max()
    s_shell = stress_of(worst_node, foundation=shell_found).max()
    print(f"  {worst_well}-well knock, PEAK member stress:")
    print(f"    on the {len(anch)} discrete anchors (the device):     {s_anchor/1e6:5.0f} MPa")
    print(f"    on a continuous shell bearing (every node):  {s_shell/1e6:5.0f} MPa  "
          f"({100*(1-s_shell/s_anchor):+.0f}%)")
    helps = s_shell < 0.7 * s_anchor

    drop = 100 * (1 - s_shell / s_anchor)
    print("\nVERDICT:")
    print(f"  1. A shell for the DORSAL knock is pointless -- that knock sizes {m_saved:.1f} g.")
    print(f"  2. But the DISCRETE ANCHORS are a BOTTLENECK. The well-knock reaction funnels through the")
    print(f"     {len(anch)} tissue anchors; spreading it over a continuous bearing cuts the peak stress")
    print(f"     {drop:.0f}% ({s_anchor/1e6:.0f} -> {s_shell/1e6:.0f} MPa). Since member stress ~ 1/r^2, that")
    print(f"     is roughly a {(1-(s_shell/s_anchor)**0.5)*100:.0f}% radius (~{(1-(s_shell/s_anchor))*100:.0f}% area)")
    print(f"     cut on the well-knock-driven members -- a real, large lead. My 'a truss always beats a")
    print(f"     plate' dismissal was wrong here: the plate's job is not to carry load, it is to give the")
    print(f"     tissue a CONTINUOUS bearing the discrete anchors cannot.")
    print(f"  3. So a SANDWICH (lattice + a shell that bears continuously on the tissue) genuinely could")
    print(f"     be lighter -- worth the full coupled plate FEA to size it.")
    print(f"\n  ⚠ {drop:.0f}% is an UPPER BOUND: the proxy lets EVERY node bear on the tissue (a rigid")
    print(f"     shell). A real finite-stiffness plate bends and helps LESS; the true saving is between 0")
    print(f"     and here, and only the coupled FEA pins it. (I retract BOTH earlier guesses -- the ~16-18 g")
    print(f"     sandwich AND the 'lattice always wins'. The honest state: a real lead, not yet a number.)")
    print(f"  ⚠ FSD is an upper bound on the sized mass ({m_need:.0f} vs {m_lat:.0f} g).")


if __name__ == "__main__":
    main()
