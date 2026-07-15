"""The inner bearing shell's sizing facts, as executable claims."""
import numpy as np

from manufacture.bearing import KNOCK_N, foundation_k, shell_pressure

E, NU = 6.0e9, 0.40
K = foundation_k(1.9e6, 0.005)


def test_the_shell_spreads_what_a_bare_foot_concentrates():
    """The whole reason for a shell: a knock through it is far gentler than through a point foot."""
    p_foot = KNOCK_N / (np.pi * (1.5e-3) ** 2)          # a 1.5 mm SKIN_R foot
    p_shell, _ = shell_pressure(KNOCK_N, 1.5e-3, E, NU, K)
    assert p_shell < p_foot / 20                         # >20x gentler


def test_a_thicker_shell_spreads_a_knock_further():
    ps = [shell_pressure(KNOCK_N, t, E, NU, K)[0] for t in (0.6e-3, 1.5e-3, 3.0e-3)]
    assert ps[0] > ps[1] > ps[2]


def test_the_impact_not_the_preload_sets_the_thickness():
    """A 50 N knock and a 5 N preload through the same shell: the knock is the binding pressure, so
    it is what earns the thickness (the preload is nearly free, and shared over several junctions)."""
    p_knock, _ = shell_pressure(KNOCK_N, 1.5e-3, E, NU, K)
    p_pre, _ = shell_pressure(5.0, 1.5e-3, E, NU, K)
    assert p_knock > 5 * p_pre


def test_a_two_mm_shell_softens_a_firm_knock():
    p, lam = shell_pressure(KNOCK_N, 2.0e-3, E, NU, K)
    assert p < 200e3            # under a painful/injurious point pressure
    assert lam > 0.008          # and it spreads the knock over an >8 mm radius
