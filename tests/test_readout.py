"""The magnetic read-out, as executable claims.

These pin the signal budget that lets the device "see the movements": if someone later shrinks
the magnet, widens the gap, or crowds the wells, the claim that a keypress is legible fails here
LOUDLY -- before anything is printed. Every number is a PREDICTION from manufacture.readout, to
be checked again on the stage-1 bench; the tests guard the model's internal consistency and the
margins it promises.
"""
import numpy as np
import pytest

from design.params import (EARTH_B, HALL_LSB, HALL_NOISE, HALL_RANGE, MAGNET_D, MAGNET_L,
                           REST_GAP)
from manufacture import readout as ro

LSB = float(HALL_LSB)
NOISE = float(HALL_NOISE)
RANGE = float(HALL_RANGE)
GAP = float(REST_GAP)


def test_the_magnet_moment_matches_its_volume():
    """m = Br V / mu0 -- a Ø3x1 mm N42 disc is ~7.3 mA m^2. A wrong magnet is caught at the source."""
    assert ro.moment() == pytest.approx(7.26e-3, rel=0.02)


def test_the_full_plunge_dwarfs_the_noise_and_the_lsb():
    """A `click` swings the axial field far past anything the sensor could miss."""
    dbz = float(np.linalg.norm(ro.delta_B("click")))
    assert dbz >= 20e-3                 # >= 20 mT
    assert dbz / LSB >= 200             # >= 200 counts
    assert dbz / NOISE >= 50            # >= 50x the noise floor


def test_the_weakest_direction_still_clears_the_noise():
    """Over the whole plausible cradle geometry (lever 0.5-1.0, tilt 0-15 deg), even the faintest
    of the five directions stays an order of magnitude above the noise -- so none is unreadable."""
    worst = min(
        ro.discriminability(ro.direction_map(lever=lv, tilt_deg=t))[0]
        for lv in (0.5, 0.6, 0.7, 0.8, 0.9, 1.0)
        for t in (0.0, 4.0, 8.0, 12.0, 15.0)
    )
    assert worst >= 2e-3               # >= 2 mT
    assert worst / NOISE >= 10         # >= 10x noise


def test_the_hard_stop_does_not_clip_the_sensor():
    """Even bottomed onto the over-travel shelf, the field stays inside the sensor's range."""
    b_stop = abs(float(ro.cyl_axial_B(GAP - ro.PLUNGE_STOP)))
    assert b_stop <= 0.8 * RANGE
    assert b_stop < RANGE              # never clips


def test_the_five_directions_are_mutually_discriminable():
    """Their delta-B vectors point far enough apart that Gaussian noise cannot confuse them:
    a nearest-template classifier makes zero errors in 1e5 draws at the datasheet noise."""
    dmap = ro.direction_map()
    _, angle = ro.discriminability(dmap)
    assert angle >= 25.0               # comfortably more than noise can rotate
    assert ro.classify_mc(dmap, noise=NOISE, n=100_000) == 0.0


def test_the_dipole_agrees_with_the_exact_cylinder_where_it_is_used():
    """The lateral channels use a point dipole; it is only trustworthy in the far-ish field, so
    check it tracks the exact on-axis cylinder within 15% by a 4 mm gap and tightens beyond."""
    def err(gap):
        zc = gap + 0.5 * float(MAGNET_L)
        b_dip = abs(ro.dipole_B([0, 0, zc], [0, 0, ro.moment()])[2])
        b_exact = abs(float(ro.cyl_axial_B(gap)))
        return abs(b_dip - b_exact) / b_exact
    assert err(4e-3) <= 0.15
    assert err(8e-3) <= 0.06
    assert err(8e-3) < err(4e-3)       # the bias is a near-field effect: it shrinks with gap


def test_earth_field_sits_inside_the_hysteresis_band():
    """Earth's ~50 uT is a static per-orientation offset; twice it is a small fraction of the
    weakest direction's switch-on threshold, so it cannot on its own trip or block a key."""
    weakest, _ = ro.discriminability(ro.direction_map())
    theta_on = 0.6 * weakest           # the firmware Schmitt on-threshold
    assert 2 * float(EARTH_B) <= 0.10 * theta_on


def test_misalignment_is_absorbed_by_calibration():
    """A +-0.5 mm build error in the magnet-Hall gap barely dents discrimination -- the per-well
    calibration (which reads each direction's real delta-B once) absorbs it."""
    for dz in (-0.5e-3, +0.5e-3):
        _, angle = ro.discriminability(ro.direction_map(gap=GAP + dz))
        assert angle >= 15.0


def test_the_power_budget_lasts_a_working_day():
    """Duty-cycled at 500 Hz, five sensors plus the MCU sketch run well over a day on 100 mAh.
    A sketch, not a measurement -- but a wrong order of magnitude would show here."""
    p = ro.scan_power(rate_hz=500.0)
    assert p["total_A"] < 5e-3         # milliamps, not tens
    assert p["life_h_100mAh"] >= 12    # at least a long session


def _button_pair_distances():
    z = np.load("out/final.npz", allow_pickle=True)
    P = np.array(z["nodes"], float)[np.array(z["buttons"], int)]   # metres
    return [float(np.linalg.norm(P[i] - P[j]))
            for i in range(len(P)) for j in range(i + 1, len(P))]


@pytest.mark.skipif(not __import__("os").path.exists("out/final.npz"),
                    reason="needs out/final.npz (the shipped design's button positions)")
def test_neighbour_crosstalk_is_below_the_noise():
    """At every real pair spacing in the shipped design, a neighbour's full-travel field change
    at this Hall is below the noise floor -- and its static part is baselined out anyway."""
    for spacing in _button_pair_distances():
        static, modulation = ro.crosstalk(spacing)
        assert modulation <= 0.1e-3            # <= 0.1 mT, below the 0.2 mT noise
        assert static <= 0.3e-3                # the baselined part stays small too
