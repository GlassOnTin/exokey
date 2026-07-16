"""THE MAGNETIC READ-OUT -- the field a moving magnet presents to the Hall.

This is the gap `manufacture/flexure.py` flags: it sizes the restoring SPRING, but says
nothing about the SIGNAL. A well is a five-way joystick (design.qwerty.ACTIONS); the cradle
carries a disc magnet over a fixed 3-axis Hall (TLx493D-class), and each direction moves the
magnet a different way, so each presents a different delta-field. The questions this settles,
all as numbers a test can check:

  * does a full keypress swing the field far above the sensor's noise and LSB?   (yes, ~400 LSB)
  * are the five directions mutually distinguishable in (Bx,By,Bz)?              (~90 deg apart)
  * does a neighbouring well's magnet leak enough to false-trigger?              (< 1 LSB, baselined)
  * do Earth's field and temperature drift matter?                              (static -> baselined)

TWO FIELD MODELS, EACH WHERE IT IS HONEST.
  * PLUNGE (click) is on the magnet axis, where a point dipole is 15-35% high in the near
    field. So the plunge uses the EXACT on-axis cylinder formula -- the truth for that channel.
  * The LATERAL tilts move the magnet OFF axis, where there is no simple closed form. They use
    a point DIPOLE, but only ever as a DIFFERENCE (posed - rest), so the dipole's on-axis bias
    cancels and what is left -- the transverse field a sideways shift produces -- is what the
    sensor actually keys on.

Frame: the sensor's own. z = plunge (`click`), +z from the Hall up toward the magnet; x = the
`forward` tilt axis, y = the `left` tilt axis. The magnet's north points down the -z axis at the
Hall; sign is a convention and cancels in every delta.

NOT MODELLED (stated, not hidden): the dome's tactile snap (buckling, not covered here), eddy/
hysteresis in the sensor, and the exact TLI493D-W2BW noise in its chosen range mode -- all of
which the stage-1 bench measures. The signal budget here is a PREDICTION to be checked, not a
measurement.
"""
from __future__ import annotations

import numpy as np

from design.params import (CRADLE_LEVER, EARTH_B, HALL_LSB, HALL_NOISE, HALL_RANGE, MAGNET_BR,
                           MAGNET_D, MAGNET_L, REST_GAP, SVALBOARD)
from design.qwerty import ACTIONS

MU0 = 4e-7 * np.pi
TRAVEL = float(SVALBOARD.travel)          # 1.5 mm
PLUNGE_STOP = 0.0018                       # m. The hard PA over-travel shelf (wellmod).


def moment(br: float = float(MAGNET_BR), d: float = float(MAGNET_D),
           L: float = float(MAGNET_L)) -> float:
    """Dipole moment of a uniformly-magnetised disc: m = Br * V / mu0  [A m^2]."""
    return br * (np.pi * (0.5 * d) ** 2 * L) / MU0


def cyl_axial_B(z: float, br: float = float(MAGNET_BR), d: float = float(MAGNET_D),
                L: float = float(MAGNET_L)) -> float:
    """EXACT axial field of a cylinder magnet, at distance `z` from its NEAR face  [T].

    B(z) = (Br/2)[ (z+L)/sqrt((z+L)^2 + R^2) - z/sqrt(z^2 + R^2) ],  R = d/2.
    The plunge truth: no dipole approximation on the one axis where it is worst.
    """
    R = 0.5 * d
    z = np.asarray(z, float)
    return 0.5 * br * ((z + L) / np.sqrt((z + L) ** 2 + R ** 2) - z / np.sqrt(z ** 2 + R ** 2))


def dipole_B(p: np.ndarray, m_vec: np.ndarray) -> np.ndarray:
    """Field at the Hall (origin) from a point dipole `m_vec` whose CENTRE is at `p`  [T].

    B = (mu0/4pi)[ 3(m.rhat)rhat - m ] / r^3,  r = origin - p = -p.
    """
    p = np.asarray(p, float)
    m_vec = np.asarray(m_vec, float)
    r = -p
    rn = np.linalg.norm(r)
    if rn < 1e-12:
        return np.zeros(3)
    rhat = r / rn
    return (MU0 / (4 * np.pi)) * (3 * (m_vec @ rhat) * rhat - m_vec) / rn ** 3


def cradle_pose(action: str, s: float = 1.0, *, gap: float = float(REST_GAP),
                lever: float = float(CRADLE_LEVER), tilt_deg: float = 8.0,
                L: float = float(MAGNET_L)):
    """(magnet centre, unit moment) at travel fraction `s` of SVALBOARD.travel, sensor frame.

    click   : the magnet PLUNGES straight down +z toward the Hall (gap shrinks), axis unchanged.
    the tilts: the magnet TRANSLATES sideways `lever * s * travel` and TILTS `tilt_deg * s` about
               the crown -- both raise a transverse field the Hall reads. forward/back on +/-x,
               left/right on +/-y (design.vector.action_dirs' t_long / t_lat).
    """
    zc = gap + 0.5 * L                                   # magnet CENTRE height at rest
    th = np.radians(tilt_deg) * s
    rho = lever * s * TRAVEL                             # lever is a dimensionless mm-per-mm ratio
    if action == "click":
        return np.array([0.0, 0.0, zc - s * TRAVEL]), np.array([0.0, 0.0, 1.0])
    axis = {"forward": (1, 0), "back": (-1, 0), "left": (0, 1), "right": (0, -1)}[action]
    ex, ey = axis
    centre = np.array([ex * rho, ey * rho, zc])
    m_hat = np.array([ex * np.sin(th), ey * np.sin(th), np.cos(th)])
    return centre, m_hat / np.linalg.norm(m_hat)


def _B_lateral(centre, m_hat, m0):
    return dipole_B(centre, m0 * m_hat)


def delta_B(action: str, s: float = 1.0, **geom) -> np.ndarray:
    """Field CHANGE at the Hall from rest to travel fraction `s`, sensor frame  [T, 3-vector].

    Plunge uses the exact axial formula; the tilts use a dipole difference (bias cancels).
    """
    gap = geom.get("gap", float(REST_GAP))
    br = geom.get("br", float(MAGNET_BR))
    d = geom.get("d", float(MAGNET_D))
    L = geom.get("L", float(MAGNET_L))
    if action == "click":
        z = gap - s * TRAVEL
        return np.array([0.0, 0.0, cyl_axial_B(z, br, d, L) - cyl_axial_B(gap, br, d, L)])
    m0 = moment(br, d, L)
    c1, u1 = cradle_pose(action, s, **{k: geom[k] for k in ("gap", "lever", "tilt_deg", "L")
                                       if k in geom})
    c0, u0 = cradle_pose(action, 0.0, **{k: geom[k] for k in ("gap", "lever", "tilt_deg", "L")
                                         if k in geom})
    return _B_lateral(c1, u1, m0) - _B_lateral(c0, u0, m0)


def direction_map(s: float = 1.0, **geom) -> dict[str, np.ndarray]:
    """{action: delta-B at travel fraction `s`} for all five ACTIONS  [T]."""
    return {a: delta_B(a, s, **geom) for a in ACTIONS}


def discriminability(dmap: dict[str, np.ndarray]) -> tuple[float, float]:
    """(weakest signal |dB| over the five, smallest pairwise angle in degrees).

    A direction is only readable if its signal clears the noise; two are only tellable apart
    if their delta-B vectors point far enough apart that noise cannot rotate one into the other.
    """
    vs = list(dmap.values())
    mags = [float(np.linalg.norm(v)) for v in vs]
    ang = 180.0
    for i in range(len(vs)):
        for j in range(i + 1, len(vs)):
            a, b = vs[i], vs[j]
            c = (a @ b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-30)
            ang = min(ang, float(np.degrees(np.arccos(np.clip(c, -1, 1)))))
    return min(mags), ang


def classify_mc(dmap: dict[str, np.ndarray], noise: float = float(HALL_NOISE),
                n: int = 100_000, seed: int = 0) -> float:
    """Nearest-template misclassification rate: add N(0,noise) to each true delta-B, classify
    by the closest of the five templates, return the error fraction. Zero is what we want."""
    acts = list(dmap)
    T = np.array([dmap[a] for a in acts])                # (5,3) templates
    rng = np.random.default_rng(seed)
    errs = 0
    per = max(1, n // len(acts))
    for i, a in enumerate(acts):
        draws = T[i] + rng.normal(scale=noise, size=(per, 3))
        d2 = ((draws[:, None, :] - T[None, :, :]) ** 2).sum(-1)   # (per,5)
        errs += int((d2.argmin(1) != i).sum())
    return errs / (per * len(acts))


def crosstalk(spacing: float, travel: float = TRAVEL, **geom) -> tuple[float, float]:
    """(static offset, full-travel modulation) a neighbour well leaks onto this Hall  [T].

    The neighbour's magnet sits `spacing` away (worst case: on-axis, the 1/r^3 factor-of-2
    lobe). STATIC part is a constant the baseline removes. MODULATION is how much that leak
    CHANGES when the neighbour is pressed -- the only part a baseline cannot catch -- estimated
    as the neighbour's magnet moving `travel` along the line of centres (d|B|/|B| = 3 dr/r).
    """
    m0 = moment(geom.get("br", float(MAGNET_BR)), geom.get("d", float(MAGNET_D)),
                geom.get("L", float(MAGNET_L)))
    static = (MU0 / (4 * np.pi)) * m0 * 2.0 / spacing ** 3        # on-axis lobe
    modulation = static * 3.0 * travel / spacing                 # worst-case radial shift
    return static, modulation


def scan_power(rate_hz: float = 500.0, n_sensors: int = 5, i_active: float = 3.7e-3,
               t_conv: float = 30e-6, i_mcu_avg: float = 1.2e-3) -> dict:
    """A duty-cycled power SKETCH (all figures SPEC/estimate, bench-verified at stage 1).

    Each sensor draws `i_active` only during its ~`t_conv` conversion, so its average is
    i_active * t_conv * rate. Add a rough MCU+BLE average. Returns currents (A) and a life
    estimate on a 100 mAh cell.
    """
    per_sensor = i_active * t_conv * rate_hz
    sensors = n_sensors * per_sensor
    total = sensors + i_mcu_avg
    return {
        "per_sensor_A": per_sensor,
        "sensors_A": sensors,
        "mcu_avg_A": i_mcu_avg,
        "total_A": total,
        "life_h_100mAh": 0.100 / total,
    }
