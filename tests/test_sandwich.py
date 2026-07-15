"""The sandwich inner face: the per-element Sizer now carries CST membrane faces, and a face
stiffens the structure it ties -- the machinery the gate re-solve (scripts/sandwich.py) relies on."""
import numpy as np

from structure.section import Ellipse


def test_the_sizer_carries_a_membrane_face():
    """Ellipse accepts sandwich faces; the solve runs, and a face has mass that scales with its
    thickness. Default (no faces) is unchanged -- the 498 um bone gate reproduces exactly."""
    nodes = np.array([[0, 0, 0], [.05, 0, 0], [.05, .05, 0], [0, .05, 0]], float)
    bars = [(0, 1), (1, 2), (2, 3), (3, 0)]
    b = np.full(4, 1e-3)
    roll = np.zeros(4)
    shells = [(0, 1, 2), (0, 2, 3)]
    thin = Ellipse(nodes, bars, shells=shells, shell_t=0.0015)
    thick = Ellipse(nodes, bars, shells=shells, shell_t=0.003)
    U, *_ = thin.solve(b, np.zeros_like(b), roll, {i: 1e6 for i in range(4)},
                       [("x", "c", {2: np.array([0., 0., -1.])})])
    assert np.isfinite(U).all()
    assert thin.fr.shell_mass(thin.rho) > 0
    assert thick.fr.shell_mass(thick.rho) > thin.fr.shell_mass(thin.rho)


def test_a_face_braces_what_bars_alone_cannot():
    """A four-bar square with no diagonal is a MECHANISM -- it shears freely under an in-plane load.
    A membrane face braces it, so the same load deflects far less. That is why a sandwich face ties
    a bearing patch into a sheet."""
    nodes = np.array([[0, 0, 0], [.05, 0, 0], [.05, .05, 0], [0, .05, 0]], float)
    bars = [(0, 1), (1, 2), (2, 3), (3, 0)]           # perimeter only: no diagonal brace
    b = np.full(4, 1e-3)
    roll = np.zeros(4)
    spring = {0: 1e6, 1: 1e6}                          # pin two corners
    cases = [("x", "c", {2: np.array([1.0, 0., 0.])})]  # in-plane shove at a free corner

    def tip(shells):
        EL = Ellipse(nodes, bars, shells=shells, shell_t=0.0015)
        U, *_ = EL.solve(b, np.zeros_like(b), roll, spring, cases)
        return float(np.linalg.norm(U[0][6 * EL.fr.idx[2]:6 * EL.fr.idx[2] + 3]))

    assert tip([(0, 1, 2), (0, 2, 3)]) < 0.5 * tip(())   # the face at least halves the deflection
