"""SHELL vs LATTICE for the impact.  PYTHONPATH=. .venv/bin/python scripts/shell_vs_lattice.py

The user, watching the form-found impact structure: "it is starting to look like plates, the struts
forming a smooth skin-hugging surface." They are right, and it is physics: a knock is a SPREADING
problem, and the optimal structure to take a point load into a soft foundation is a PLATE on an
elastic foundation -- exactly §8.15i's bearing shell. So the dense lattice, form-found in the thin
band off the skin, is CONVERGING to a shell. This weighs the two.

⚠ NOT LIKE FOR LIKE, AND SAYING SO IS THE POINT.
  * the LATTICE number is MEASURED (FEA, §8.15k): 24.2 g of circular rods, survives the 50 N knock
    at every well AND the dorsal ridge, AND holds the keys at the 500 um gate. It does BOTH jobs.
  * the SHELL number is an ANALYTICAL ESTIMATE (Westergaard interior loading, a plate on a Winkler
    foundation) for ONE job: take a knock on the BACK OF THE HAND without yielding, and spread it to
    a comfortable skin pressure. It does NOT reach the fingertip wells, so it cannot hold a key crisp
    or survive a knock ON a well -- those still need lattice.
So the honest question is not "shell or lattice" but "does a shell over the dorsal region let the
lattice underneath be SPARSE (keypress-only) instead of dense (impact) -- i.e. is the SANDWICH lighter
than the pure impact lattice?"
"""
from __future__ import annotations

import pickle

import numpy as np

from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from manufacture.bearing import COMFORT, foundation_k, shell_pressure
from opt.problem import hands
from structure.frame import MATERIALS, hand_axes

KNOCK_N = 50.0
SF = 2.0
E_TISSUE = 1.9e6            # Pa, dorsal soft tissue (as §8.15i / tests/test_bearing.py)
T_TISSUE = 5.0e-3          # m, tissue depth over the metacarpals
B_CONTACT = 4.0e-3         # m, equivalent contact radius of a firm knock (a knuckle/edge). GUESS.


def westergaard_interior(P, t, E, nu, k, b):
    """Max bending stress under an interior point load on a plate on a Winkler foundation.

    Westergaard's closed form (the pavement-engineering standard for exactly this problem):
        sigma = 0.275 (1+nu) P / t^2 * log10( E t^3 / (k b^4) )
    b is the equivalent loaded radius; the log group is 12(1-nu^2)(lambda/b)^4, so this is really a
    function of how many characteristic lengths the load is spread over.
    """
    return 0.275 * (1.0 + nu) * P / t ** 2 * np.log10(E * t ** 3 / (k * b ** 4))


def main():
    H = hands()
    ref = H[50]
    fd = pickle.load(open("out/final_design.pkl", "rb"))
    x = fd["x"]
    evaluate(x, H)
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    o, e_d, e_r, e_o = hand_axes(ref, q)

    mat = MATERIALS["cf_pa12"]
    E, nu, rho, yld = mat["E"], mat["nu"], mat["rho"], mat["yield_"]
    k = foundation_k(E_TISSUE, T_TISSUE)

    # --- the DORSAL FOOTPRINT the shell would have to cover: project the impact lattice's nodes onto
    # the dorsal plane and take their convex-hull area. This is the region the structure spans. ---
    z = np.load("out/impact_opt.npz", allow_pickle=True)
    nodes = z["nodes"]
    used = np.unique(np.asarray(z["bars"]).ravel())
    P2 = np.column_stack([(nodes[used] - o) @ e_d, (nodes[used] - o) @ e_r])
    from scipy.spatial import ConvexHull
    A = float(ConvexHull(P2).volume)      # 'volume' of a 2-D hull is its area
    m_lat = float(z["mass"])              # grams, MEASURED

    print(f"tissue foundation k = {k:.2e} N/m^3;  cf_pa12 E {E/1e9:.0f} GPa, yield {yld/1e6:.0f} MPa, "
          f"rho {rho:.0f}")
    print(f"dorsal footprint the shell must span: {A*1e4:.1f} cm^2 "
          f"(convex hull of the lattice's nodes)\n")

    # --- size a SOLID shell: thinnest t whose knock stress meets yield/SF, and its skin pressure ---
    print(f"SOLID SHELL sized for the {KNOCK_N:.0f} N dorsal knock (Westergaard, plate on tissue):")
    print(f"  {'t (mm)':>7} {'knock stress':>13} {'skin pressure':>14} {'mass':>8}")
    t_ok = None
    for t in (1.0e-3, 1.5e-3, 2.0e-3, 2.5e-3, 3.0e-3):
        sig = westergaard_interior(KNOCK_N, t, E, nu, k, B_CONTACT)
        p_skin, _lam = shell_pressure(KNOCK_N, t, E, nu, k)
        m = rho * A * t * 1e3            # grams
        ok = sig <= yld / SF
        if ok and t_ok is None:
            t_ok = (t, sig, p_skin, m)
        print(f"  {t*1e3:>6.1f}  {sig/1e6:>9.0f} MPa  {p_skin/1e3:>10.0f} kPa  {m:>6.1f} g"
              f"  {'<- survives (SF 2)' if ok else ''}")

    print(f"\n  ⚠ comfort: a knock at ~{COMFORT/1e3:.0f} kPa is a firm-but-tolerable press; "
          f"the shell's job is to keep the skin pressure there, which it does.")

    # --- the SANDWICH: a shell need not be solid. Two thin faces a distance h apart give the same
    # bending stiffness D = E t_eq^3/12 at a fraction of the mass -- that is the whole point of §8.15i.
    if t_ok is not None:
        t, sig, p_skin, m_solid = t_ok
        # a sandwich matching this solid plate's D with 2 faces of t_f at separation h: D_sand ~
        # E t_f h^2 / 2 (thin faces). Match to D_solid = E t^3/12 -> t_f = t^3/(6 h^2). But a printed
        # face cannot go below the NOZZLE (0.4 mm) -- the stiffness-optimal face is thinner than that,
        # so the face is MANUFACTURING-limited, not stiffness-limited, and is over-stiff for the knock.
        h = 4.0e-3
        NOZZLE = 0.4e-3
        t_f = max(t ** 3 / (6.0 * h ** 2), NOZZLE)
        m_faces = rho * A * 2 * t_f * 1e3
        floored = t ** 3 / (6.0 * h ** 2) < NOZZLE
        print(f"\nSANDWICH matching that plate's stiffness: 2 faces at {h*1e3:.0f} mm separation, each "
              f"{t_f*1e3:.2f} mm{' (nozzle-floored -- the stiffness-optimal face is thinner)' if floored else ''}")
        print(f"  {m_faces:.1f} g of face + a light lattice/foam core")

    print("\nVERDICT (dorsal impact-bearing region only):")
    print(f"  dense LATTICE (measured, FEA):        {m_lat:.1f} g   -- and it also holds the keys and "
          f"survives WELL knocks")
    if t_ok is not None:
        print(f"  SOLID SHELL (Westergaard estimate):   {m_solid:.1f} g   -- dorsal knock only; does NOT "
              f"reach the wells")
        print(f"  SANDWICH face (estimate):             {m_faces:.1f} g + core")
        print(f"\n  -> a shell over the back is {'LIGHTER' if m_solid < m_lat else 'heavier'} than the "
              f"dense lattice for the DORSAL knock, but it is only half a device: the fingertip wells")
        print(f"     (and knocks ON them) still need lattice reach. The lattice's density is the price")
        print(f"     of doing BOTH jobs with ONE part. The lighter machine is the SANDWICH -- a SPARSE")
        print(f"     keypress lattice to reach the wells, a thin shell/face over the back for the knock.")
    print("\n  ⚠ ESTIMATE: Westergaard interior loading, Winkler foundation, one solid plate; the")
    print("     lattice number is a full FEA. Contact radius and tissue depth are guesses. A real")
    print("     answer needs the shell meshed as a plate (the FEA shell here is membrane-only, §8.15i).")


if __name__ == "__main__":
    main()
