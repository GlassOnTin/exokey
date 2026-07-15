"""THE INNER BEARING SHELL, sized.  PYTHONPATH=. .venv/bin/python scripts/bearing.py

How thick a glass-nylon shell has to be to spread a knock into the skin gently -- the IMPACT, not
the steady preload, is what sets it. Loads enter the shell at the lattice junctions (and a knock
lands anywhere), as concentrated forces; the shell on the soft tissue spreads each over lambda.
"""
from __future__ import annotations

import numpy as np

from manufacture.bearing import (CAPILLARY, COMFORT, KNOCK_N, foundation_k, shell_pressure)

E, NU = 6.0e9, 0.40                 # glass-nylon shell
K = foundation_k(1.9e6, 0.005)      # tissue Winkler modulus (TISSUE_E over ~5 mm dorsal)


def main():
    print(f"tissue foundation k = {K:.2e} N/m^3   (glass-nylon shell, E = {E/1e9:.0f} GPa)\n")
    print("peak SKIN PRESSURE under a concentrated load, spread by the shell (P / 8 lambda^2):")
    print(f"  {'shell t':>8} {'lambda':>8} {'spread dia':>11}    {'2 N':>6} {'5 N':>6} "
          f"{'10 N':>6} {'50 N knock':>11}")
    for tmm in (0.6, 1.0, 1.5, 2.0, 3.0):
        t = tmm * 1e-3
        _, lam = shell_pressure(1.0, t, E, NU, K)
        row = [shell_pressure(P, t, E, NU, K)[0] / 1e3 for P in (2, 5, 10, KNOCK_N)]
        print(f"  {tmm:8.1f} {lam*1e3:7.1f}mm {2*np.pi*lam*1e3:9.0f}mm    {row[0]:5.0f}k {row[1]:5.0f}k "
              f"{row[2]:5.0f}k {row[3]:10.0f}k")

    print(f"\nvs a BARE 1.5 mm foot ({np.pi*1.5**2:.0f} mm^2): a {KNOCK_N:.0f} N knock -> "
          f"{KNOCK_N/(np.pi*(1.5e-3)**2)/1e6:.1f} MPa. The shell is the fix, and it is worth ~450x.")
    print(f"\nrefs (kPa): capillary occlusion {CAPILLARY/1e3:.0f} (all-day sore line), "
          f"comfortable-worn ~{COMFORT/1e3:.0f}, a painful knock ~200.")
    print("READING: the steady preload is shared over several junctions (<1 N each) -> comfortable at")
    print("any thickness. A KNOCK lands as one point, so it binds: ~1.5-2 mm keeps a 50 N knock to")
    print("~60-90 kPa (felt, not injurious). ⚠ quasi-static + infinite-plate + Winkler -> a LOWER bound.")

    # mass: a shell over the dorsal bearing region (~a conservative 60 cm^2) at 1.5 mm
    for tmm, area_cm2 in ((1.5, 60), (2.0, 60)):
        m = tmm * 1e-3 * area_cm2 * 1e-4 * 1060.0
        print(f"  mass of a {tmm:.1f} mm shell over ~{area_cm2} cm^2: {m*1e3:.0f} g "
              f"(the bone is 11 g -- a real cost).")


if __name__ == "__main__":
    main()
