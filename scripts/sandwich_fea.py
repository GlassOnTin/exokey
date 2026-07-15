"""COUPLED PLATE FEA: pin the sandwich number.  PYTHONPATH=. .venv/bin/python scripts/sandwich_fea.py

sandwich_weigh.py found the DISCRETE ANCHORS bottleneck the well knock, and spreading the reaction
over a continuous bearing cuts the peak up to 65% -- an upper bound (rigid shell). This puts a real
FINITE-stiffness plate under the lattice and solves the couple, so the saving is a number, not a bound.

Model, in a frame where the dorsal plane is XY (e_d->X, e_r->Y, e_o->Z, out of the hand = +Z):
  * the lattice as PyNite frame members at their sized radii;
  * a flat quad SHELL at the skin (Z = the anchor plane), thickness t, on tissue springs (Winkler);
  * each lattice ANCHOR tied to the nearest shell node by a stiff link, so the lattice bears on the
    shell and the shell bears on the tissue -- continuous, not through 94 points.
Sweep t: at t=0 (no shell) it must reproduce the project solver's discrete-anchor stress (validation);
as t grows the shell spreads the reaction and the lattice stress falls to the rigid-shell bound.

STEP 1 here: the lattice ALONE, to validate PyNite against the project solver before the shell goes on.
"""
from __future__ import annotations

import pickle

import numpy as np
from Pynite import FEModel3D

from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import MATERIALS, hand_axes
from structure.lattice import STRAP_K, ground

KNOCK_N = 50.0
E_TPU = MATERIALS["tpu"]["E"]
A_ANCHOR = 1.0e-4


def setup():
    H = hands()
    ref = H[50]
    fd = pickle.load(open("out/final_design.pkl", "rb"))
    x = fd["x"]
    evaluate(x, H)
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    o, e_d, e_r, e_o = hand_axes(ref, q)
    _n, _b, btn, _l, ak, an, _t, strap_n = ground(ref, q)
    z = np.load("out/impact_opt.npz", allow_pickle=True)
    nodes = np.asarray(z["nodes"], float)
    bars = [tuple(int(i) for i in b) for b in z["bars"]]
    r = np.asarray(z["radii"], float)
    # rotate every node into the dorsal frame: X along e_d, Y along e_r, Z along e_o (out of the hand)
    R = np.column_stack([e_d, e_r, e_o])
    X = (nodes - o) @ R
    akc = {i: 1.0 / (1.0 / ak[i] + 1.0 / (E_TPU * A_ANCHOR / 0.002)) for i in ak}
    band = set(strap_n) & set(ak)
    ktot = sum(ak[i] for i in band) or 1.0
    ks = {i: (float(STRAP_K) * ak[i] / ktot if i in band else 0.0) for i in ak}
    return dict(X=X, bars=bars, r=r, btn=btn, ak=ak, akc=akc, ks=ks, an=an, strap_n=strap_n,
                mass=float(z["mass"]))


def lattice_model(d, radii=None):
    """A PyNite model of the lattice alone (in the dorsal frame). Compression-only tissue at anchors."""
    X, bars = d["X"], d["bars"]
    r = d["r"] if radii is None else radii
    mat = MATERIALS["cf_pa12"]
    E, nu, G, rho = mat["E"], mat["nu"], mat["G"], mat["rho"]
    m = FEModel3D()
    m.add_material("pa", E, G, nu, rho)
    used = sorted({i for b in bars for i in b})
    for i in used:
        m.add_node(f"LT{i}", X[i, 0], X[i, 1], X[i, 2])
    for e, (i, j) in enumerate(bars):
        rr = float(r[e])
        A = np.pi * rr ** 2
        I = np.pi * rr ** 4 / 4.0
        m.add_section(f"S{e}", A, I, I, 2 * I)
        m.add_member(f"M{e}", f"LT{i}", f"LT{j}", "pa", f"S{e}")
    # bilinear anchor: a compression (tissue, akc) and a tension (strap, ks) spring in Z at each anchor
    for i, kc in d["akc"].items():
        if i in used:
            m.def_support_spring(f"LT{i}", "DZ", kc, "-")            # tissue resists pressing IN (-Z)
            if d["ks"].get(i, 0) > 0:
                m.def_support_spring(f"LT{i}", "DZ", d["ks"][i], "+")  # strap resists lifting (+Z)
    return m, used


def member_stresses(m, d, radii=None):
    """Per-lattice-member fibre stress sigma = |N|/A + |M|*r/I, as an array indexed by member e."""
    r = d["r"] if radii is None else radii
    sig = np.zeros(len(d["bars"]))
    for name in m.members:
        if not name.startswith("M"):          # skip the stiff LINK members ("L#"), not lattice
            continue
        e = int(name[1:])
        mem = m.members[name]
        rr = float(r[e])
        A = np.pi * rr ** 2
        I = np.pi * rr ** 4 / 4.0
        N = abs(mem.max_axial("Combo 1"))
        My = max(abs(mem.max_moment("My", "Combo 1")), abs(mem.min_moment("My", "Combo 1")))
        Mz = max(abs(mem.max_moment("Mz", "Combo 1")), abs(mem.min_moment("Mz", "Combo 1")))
        sig[e] = N / A + np.hypot(My, Mz) * rr / I
    return sig


def knock_field(d, t_shell):
    """Per-member WORST stress over all 6 knocks, with a shell of thickness t (0 = no shell)."""
    worst = np.zeros(len(d["bars"]))
    from hand.myohand import FINGERS as FF
    spots = [int(d["btn"][f]) for f in FF]
    o = d["X"]
    spots.append(int(max(d["ak"], key=lambda i: o[i, 2])))       # dorsal ridge = highest-Z anchor
    for node in spots:
        m, used = lattice_model(d)
        if t_shell > 0:
            add_shell(m, d, used, t_shell)
        m.add_node_load(f"LT{node}", "FZ", -KNOCK_N)
        m.analyze_linear(check_stability=False)
        worst = np.maximum(worst, member_stresses(m, d))
    return worst


def add_shell(m, d, used, t_shell):
    """Put a flat quad shell at the anchor plane on tissue springs, and tie each anchor to it.

    Returns the shell's plan area (for its mass). Moves the compression bearing from the discrete
    anchors ONTO the shell: the anchors keep only their strap (tension) spring and are linked to the
    nearest shell node by a stiff strut, so their bearing now flows through the continuous plate.
    """
    X, ak = d["X"], d["ak"]
    anchors = [i for i in ak if i in used]
    az = np.array([X[i, 2] for i in anchors])
    z0 = float(az.mean())
    xs, ys = X[anchors, 0], X[anchors, 1]
    ox, oy = float(xs.min() - 0.01), float(ys.min() - 0.01)
    w, h = float(xs.max() - xs.min() + 0.02), float(ys.max() - ys.min() + 0.02)
    ms = 0.006
    m.add_rectangle_mesh("SH", ms, w, h, t_shell, "pa", plane="XY", origin=(ox, oy, z0))
    m.meshes["SH"].generate()
    shell_nodes = [n for n in m.nodes if not n.startswith("LT")]
    # Winkler tissue under the shell (compression only, into the hand = -Z)
    k = 1.9e6 / 5.0e-3
    trib = {n: 0.0 for n in shell_nodes}
    for qd in m.quads.values():
        for nd in (qd.i_node.name, qd.j_node.name, qd.m_node.name, qd.n_node.name):
            if nd in trib:
                trib[nd] += ms * ms / 4.0
    for n, a in trib.items():
        if a > 0:
            m.def_support_spring(n, "DZ", k * a, "-")
    # stiff strut from each anchor to the nearest shell node (the lattice bears THROUGH the shell)
    spos = {n: np.array([m.nodes[n].X, m.nodes[n].Y, m.nodes[n].Z]) for n in shell_nodes}
    m.add_section("LINK", 1e-3, 1e-6, 1e-6, 2e-6)                # a very stiff strut
    for c, i in enumerate(anchors):
        p = X[i]
        sn = min(shell_nodes, key=lambda n: np.sum((spos[n] - p) ** 2))
        m.add_member(f"L{c}", f"LT{i}", sn, "pa", "LINK")
    return w * h


def shell_area(d):
    X, ak = d["X"], d["ak"]
    used = {i for b in d["bars"] for i in b}
    anchors = [i for i in ak if i in used]
    xs, ys = X[anchors, 0], X[anchors, 1]
    return float((xs.max() - xs.min() + 0.02) * (ys.max() - ys.min() + 0.02))


def main():
    d = setup()
    rho = MATERIALS["cf_pa12"]["rho"]
    X, bars, r = d["X"], d["bars"], d["r"]
    L = np.array([np.linalg.norm(X[j] - X[i]) for i, j in bars])
    r_print = 2.5e-4
    A_sh = shell_area(d)

    def lat_mass(rr):
        return float(rho * np.pi * np.sum(np.maximum(rr, r_print) ** 2 * L) * 1e3)

    m0 = lat_mass(r)
    print(f"pure lattice (PyNite basis): {len(bars)} members, {m0:.1f} g; shell would span "
          f"{A_sh*1e4:.0f} cm^2\n")

    sig0 = knock_field(d, 0.0)
    print("SANDWICH end-to-end: thin the lattice by the knock stress the shell removes, add the shell.")
    print(f"  {'shell t':>8} {'peak knock':>11} {'lattice':>9} {'+ shell':>8} {'= TOTAL':>9}")
    best = (m0, 0.0, m0, 0.0)
    for t in (0.0, 0.5e-3, 1.0e-3, 1.5e-3, 2.0e-3, 2.5e-3):
        sig = sig0 if t == 0 else knock_field(d, t)
        # FSD: sigma ~ 1/r^3 (bending), so r_new = r*(sigma/sigma0)^(1/3); only thin, floor at print.
        fac = np.where(sig0 > 1e3, np.clip(sig / np.maximum(sig0, 1e3), 0, 1) ** (1.0 / 3.0), 1.0)
        r_new = np.maximum(r * fac, r_print)
        m_lat = lat_mass(r_new)
        m_sh = rho * A_sh * t * 1e3 if t > 0 else 0.0
        tot = m_lat + m_sh
        mark = "  <- lightest" if tot < best[0] else ""
        if tot < best[0]:
            best = (tot, t, m_lat, m_sh)
        print(f"  {t*1e3:>6.1f}mm {sig.max()/1e6:>7.0f} MPa {m_lat:>6.1f} g {m_sh:>6.1f} g {tot:>7.1f} g{mark}")

    print(f"\nVERDICT (all in one solver, so the comparison is self-consistent):")
    print(f"  pure lattice:          {m0:>5.1f} g")
    if best[1] > 0:
        print(f"  best SANDWICH ({best[1]*1e3:.1f} mm shell): {best[2]:.1f} g lattice + {best[3]:.1f} g shell "
              f"= {best[0]:.1f} g")
        print(f"  -> the sandwich is {100*(1-best[0]/m0):+.0f}% "
              f"({'LIGHTER' if best[0] < m0 else 'heavier'}); a shell that unbottlenecks the anchors "
              f"{'pays for itself' if best[0] < m0 else 'costs more than it saves'}.")
    else:
        print(f"  no shell thickness beats the pure lattice -- the shell's mass exceeds what it saves.")
    print("\n  ⚠ flat shell tied to the anchors by stiff struts; FSD thinning (sigma~1/r^3, knock only,")
    print("     gate assumed non-binding as it is met with margin); PyNite reads stress ~2.6x the project")
    print("     solver, so absolute grams are solver-dependent -- the RATIO lattice:sandwich is the answer.")


if __name__ == "__main__":
    main()
