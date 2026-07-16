"""The well module geometry, as executable claims.

The magnet, the Hall and the wires have to live in a printable part that a finger can bear to
wear. These pin the design rules -- press-fit, travel-before-stop, stop-before-fatigue, a cup
stiffer than its own flexure, a lip that snaps and holds, features the voxel can resolve, grooves
that do not gut the struts -- so a later edit that breaks one fails here, not on the printer.
"""
import numpy as np
import pytest

from design.params import MAGNET_D, MAGNET_L
from manufacture import wellmod as wm
from manufacture.flexure import dome_stress, fatigue_strain
from manufacture.friendly import SKIN_R
from manufacture.mesh import VOXEL
from manufacture.readout import PLUNGE_STOP, TRAVEL
from structure.frame import MATERIALS

TPU = MATERIALS["tpu"]


def test_the_insert_and_seat_mesh_watertight_and_one_piece():
    """Every bench coupon -- the TPU cradle at each dome variant, and the PA seat -- is a single
    watertight solid. A sealed void or a floating piece (both real bugs we hit) fails here."""
    for name, m in wm.coupon_meshes().items():
        assert m.is_watertight, name
        assert m.body_count == 1, f"{name}: {m.body_count} pieces"
        assert m.volume > 0, name


def test_the_magnet_pocket_is_a_press_fit():
    """0.1 mm smaller than the disc (press fit) and 0.2 mm deeper (glue relief)."""
    assert wm.MAGNET_POCKET_D == pytest.approx(float(MAGNET_D) - 0.1e-3)
    assert wm.MAGNET_POCKET_DEPTH == pytest.approx(float(MAGNET_L) + 0.2e-3)
    assert wm.MAGNET_POCKET_D < float(MAGNET_D)          # it MUST be an interference fit


def test_travel_reaches_the_switch_before_the_stop():
    """The key actuates at 1.5 mm; the hard PA shelf is past that, but not so far the dome over-flexes."""
    assert TRAVEL < PLUNGE_STOP <= 2.0e-3


def test_the_stop_engages_before_the_dome_fatigues():
    """Bottomed on the over-travel shelf, the dome's bending strain still sits under its fatigue
    strain with a safety factor -- so a hard knock cannot crack the flexure."""
    strain_at_stop = dome_stress(wm.K * PLUNGE_STOP, wm.DOME_T) / TPU["E"]
    merit = fatigue_strain(TPU["fatigue"], TPU["E"])         # sigma_fat / E
    assert strain_at_stop <= merit / 2.0                     # SAFETY_FACTOR = 2


def test_the_cup_is_stiff_relative_to_the_dome():
    """The cup wall must be far stiffer than the dome, or the fingertip's motion is lost bending the
    cup instead of the flexure. Plate stiffness goes as thickness cubed, so the 2.5 mm cup is
    hundreds of times stiffer than the 0.32 mm dome -- well over the 10x that keeps >=90% of travel."""
    assert (wm.CUP_WALL / wm.DOME_T) ** 3 >= 10.0


def test_the_keying_lip_installs_and_retains():
    """PA and TPU do not weld, so the insert is held by a snap lip. The hoop strain to stretch the
    skirt over the lip stays inside TPU's reach, and the lip seats deep enough to hold."""
    hoop = wm.SKIRT_LIP / (wm.DOME_A + wm.SKIRT_WALL)
    assert hoop <= 0.08                                      # TPU stretches over the lip, elastically
    assert wm.SKIRT_ENGAGE >= 0.6e-3                         # and seats at least 0.6 mm deep


def test_every_structural_feature_survives_the_voxel():
    """Every load-bearing wall is at least two voxels thick, so the mesher resolves it -- EXCEPT the
    dome membrane, which is deliberately the sub-nozzle flexure (design.flexure/VISION 8.15g): it
    needs a 0.25 mm nozzle or a corrugation, and that is flagged, not silently too thin."""
    for feat in (wm.PA_WALL, wm.CUP_WALL, wm.SKIRT_WALL, wm.BASE_T, 2 * wm.GROOVE_R):
        assert feat >= 2 * VOXEL, feat
    assert wm.DOME_T < 0.4e-3                                # the flagged single-perimeter membrane


def _groove_loss(member_r, gr=wm.GROOVE_R, bury=wm.GROOVE_BURY):
    """Fraction of a round member's section a surface groove removes: the part of the groove circle
    (centre `bury` below the surface) that lies inside the member. The cap sticking out is free."""
    h_cap = gr - bury                                        # how far the groove protrudes past the surface
    cap = gr ** 2 * np.arccos((gr - h_cap) / gr) - (gr - h_cap) * np.sqrt(2 * gr * h_cap - h_cap ** 2)
    removed = np.pi * gr ** 2 - cap
    return removed / (np.pi * member_r ** 2)


def test_a_groove_does_not_gut_even_a_floor_member():
    """A wire groove in the THINNEST possible member (at the ergonomic floor radius) still removes
    under 15% of its section; every thicker member loses less. So the harness never guts a strut."""
    assert _groove_loss(float(SKIN_R)) <= 0.15
    assert _groove_loss(2.0e-3) < _groove_loss(float(SKIN_R))   # monotone: thicker loses less


@pytest.fixture(scope="module")
def design_posture():
    import os
    import pickle
    if not os.path.exists("out/final_design.pkl"):
        pytest.skip("needs out/final_design.pkl (the shipped typing posture)")
    from design.vector import posture, tm_of, tp_of
    from hand.myohand import FINGERS
    from opt.problem import hands
    H = hands()
    ref = H[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    return ref, q


def _frame_gap(ref, q, fa, fb):
    from scipy.spatial import cKDTree
    a = wm.frame_mesh(ref, q, fa)
    b = wm.frame_mesh(ref, q, fb)
    return float(cKDTree(a.vertices).query(b.vertices)[0].min())


def test_all_but_the_tightest_pair_of_modules_clear(design_posture):
    """Every pair of well modules EXCEPT the tightest leaves clear air between the printed frames."""
    import itertools

    from hand.myohand import FINGERS
    ref, q = design_posture
    for fa, fb in itertools.combinations(FINGERS, 2):
        if {fa, fb} == {"middle", "ring"}:
            continue                                    # the known-tight pair, tested below
        assert _frame_gap(ref, q, fa, fb) >= 2.0e-3, f"{fa}/{fb} frames too close"


@pytest.mark.xfail(strict=True, reason="per-finger modules interpenetrate at the tightest "
                   "(middle-ring) pitch; needs cluster-level packing -- see VISION 8.15l")
def test_the_tightest_pair_needs_cluster_packing(design_posture):
    """MEASURED LIMITATION, stated not hidden: sized independently, the middle and ring sensor tails
    run nearly parallel and INTERPENETRATE in the typing posture -- their frames do not clear. A
    coupled cluster (shared walls or staggered tail depth) is the fix, and is future work. Marked
    xfail(strict) so that whoever implements that fix is told, here, to retire this note."""
    ref, q = design_posture
    assert _frame_gap(ref, q, "middle", "ring") >= 1.0e-3
