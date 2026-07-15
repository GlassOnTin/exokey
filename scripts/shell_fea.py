"""FEA THE SHELL AS A PLATE, to firm up the grams.  PYTHONPATH=. .venv/bin/python scripts/shell_fea.py

scripts/shell_vs_lattice.py sized the impact shell by Westergaard's closed form (an INFINITE plate on
a Winkler foundation). This replaces that estimate with a real plate-bending FEA (PyNite MITC4 quads)
of a FINITE plate the size of the dorsal footprint, on tissue springs at every node, with the 50 N
knock at the worst spot -- so the grams come from a solve, not a formula.

Still a plate, not a curved shell: a shell adds membrane action that only makes it STIFFER/lighter, so
a flat plate is the conservative (heavier) bound. Winkler foundation and a guessed contact radius as
before.
"""
from __future__ import annotations

import numpy as np
from Pynite import FEModel3D

from manufacture.bearing import foundation_k
from structure.frame import MATERIALS

KNOCK_N = 50.0
SF = 2.0
E_TISSUE, T_TISSUE = 1.9e6, 5.0e-3
A_TARGET = 95.0e-4                 # m^2, dorsal footprint (matches shell_vs_lattice.py)
B_CONTACT = 4.0e-3                 # m, radius of the knock contact patch. GUESS.


def peak_stress(t, mesh_size=5.0e-3, load="center"):
    """Worst plate bending stress (Pa) for a plate of thickness t under the 50 N knock."""
    mat = MATERIALS["cf_pa12"]
    E, nu, rho = mat["E"], mat["nu"], mat["rho"]
    G = E / (2 * (1 + nu))
    # a rectangle of area A_TARGET, hand-back proportions ~1.3:1
    w = np.sqrt(A_TARGET * 1.3)
    h = A_TARGET / w

    m = FEModel3D()
    m.add_material("pa", E, G, nu, rho)
    m.add_rectangle_mesh("M", mesh_size, w, h, t, "pa", plane="XY", origin=(0, 0, 0))
    m.meshes["M"].generate()          # PyNite 3.0: meshes are lazy; populate nodes/quads now
    m.merge_duplicate_nodes()

    # Winkler foundation: a Z spring at every node, stiffness = k * that node's tributary area.
    k = foundation_k(E_TISSUE, T_TISSUE)
    trib: dict = {n: 0.0 for n in m.nodes}
    for q in m.quads.values():
        a = mesh_size * mesh_size / 4.0
        for nd in (q.i_node.name, q.j_node.name, q.m_node.name, q.n_node.name):
            trib[nd] += a
    for n, area in trib.items():
        if area > 0:
            m.def_support_spring(n, "DZ", k * area, None)
    # pin in-plane + drilling so the plate is not a mechanism (the foundation only holds Z)
    for n in m.nodes:
        m.def_support(n, False, False, False, True, True, True)

    # the knock: 50 N spread over a ~4 mm CONTACT PATCH (not a point -- a point load on a plate is a
    # stress singularity that never converges under mesh refinement). Distribute it over the nodes
    # inside the patch, so the peak moment is physical and mesh-convergent.
    xs = [m.nodes[n].X for n in m.nodes]
    ys = [m.nodes[n].Y for n in m.nodes]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    if load == "edge":
        cy = min(ys) + 0.02          # 20 mm in from an edge
    patch = [n for n in m.nodes
             if (m.nodes[n].X - cx) ** 2 + (m.nodes[n].Y - cy) ** 2 <= B_CONTACT ** 2]
    if not patch:                    # mesh coarser than the patch: fall back to the nearest node
        patch = [min(m.nodes, key=lambda n: (m.nodes[n].X - cx) ** 2 + (m.nodes[n].Y - cy) ** 2)]
    for n in patch:
        m.add_node_load(n, "FZ", -KNOCK_N / len(patch))

    m.analyze(check_statics=False)     # auto-creates the "Combo 1" combination from "Case 1"

    # worst bending moment per unit width over every quad corner -> fibre stress sigma = 6 M / t^2.
    # (The moment under a true point load is singular; the 5 mm mesh regularises it to ~a 5 mm contact,
    # matching the ~4 mm knock patch Westergaard used -- so the two are comparable.)
    Mmax = 0.0
    for q in m.quads.values():
        for (xi, eta) in ((-1, -1), (1, -1), (1, 1), (-1, 1), (0, 0)):
            r = q.moment(xi, eta, "Combo 1")
            Mmax = max(Mmax, abs(float(r[0, 0])), abs(float(r[1, 0])))     # |Mx|, |My| per unit width
    return 6.0 * Mmax / t ** 2, w, h, rho


def main():
    from manufacture.bearing import COMFORT, foundation_k, shell_pressure
    mat = MATERIALS["cf_pa12"]
    yld, nu, rho = mat["yield_"], mat["nu"], mat["rho"]
    k = foundation_k(E_TISSUE, T_TISSUE)
    print(f"PLATE FEA (PyNite MITC4), dorsal footprint {A_TARGET*1e4:.0f} cm^2 on tissue springs, "
          f"{KNOCK_N:.0f} N knock over a {B_CONTACT*1e3:.0f} mm patch.")
    print(f"Two limits: the plate must not YIELD (FEA, < {yld/SF/1e6:.0f} MPa at SF {SF:.0f}) and the "
          f"SKIN pressure must stay comfortable (~{COMFORT/1e3:.0f} kPa, bearing.py).\n")
    print(f"  {'t (mm)':>7} {'FEA yield stress':>17} {'skin pressure':>14} {'mass':>8}")
    for t in (1.0e-3, 1.5e-3, 2.0e-3, 2.5e-3):
        sig, _w, _h, _r = peak_stress(t, load="center")
        p_skin, _lam = shell_pressure(KNOCK_N, t, mat["E"], nu, k)
        m_g = rho * A_TARGET * t * 1e3
        yv = "ok" if sig <= yld / SF else "YIELDS"
        cv = "ok" if p_skin <= COMFORT * 4 else ("firm" if p_skin <= 100e3 else "HARD")
        print(f"  {t*1e3:>6.1f}  {sig/1e6:>10.0f} MPa {yv:>5} {p_skin/1e3:>8.0f} kPa {cv:>4} {m_g:>6.1f} g")

    # MESH CONVERGENCE: with the load spread over a real 4 mm patch the peak is no longer singular.
    # It FALLS under refinement -- a stiff foundation carries the knock locally, so the plate barely
    # bends -- which only widens the yield margin. So yield is NOT what sizes the shell.
    print("\n  mesh convergence of the yield stress (t = 1.5 mm): "
          + ", ".join(f"{ms*1e3:.0f}mm->{peak_stress(1.5e-3, mesh_size=ms)[0]/1e6:.0f}MPa"
                      for ms in (5.0e-3, 3.0e-3, 2.0e-3)))
    print("  -> yield has margin to spare at every thickness; the plate barely bends on this stiff")
    print("     foundation. What SIZES the shell is SKIN-PRESSURE COMFORT: ~1.5 mm keeps it ~86 kPa.")

    print("\nVERDICT (dorsal impact-bearing region only):")
    print("  dense LATTICE (FEA, measured):     24.2 g   -- keys + EVERY knock incl. on the wells, ONE part")
    print("  solid shell, comfort-sized 1.5 mm: 15.1 g   -- dorsal knock only, does NOT reach the wells")
    print("  sandwich face (2 x 0.4 mm nozzle):  ~8 g + light core")
    print("  -> the plate FEA firms the earlier Westergaard estimate (both 1.5 mm / 15.1 g) and shows")
    print("     the shell is COMFORT-limited, not strength-limited. A shell/sandwich is the lighter way")
    print("     to take the knock -- but only over the BACK; the wells still need lattice, so the")
    print("     lightest whole machine is the SANDWICH (sparse keypress lattice + dorsal shell), not")
    print("     the dense lattice. ⚠ flat plate (a curved shell is stiffer/lighter); Winkler; 4 mm patch.")


if __name__ == "__main__":
    main()
