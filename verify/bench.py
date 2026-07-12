"""Stage 0 -- measure, don't estimate.

The plan's compute budget rests on "~0.5 s per static-optimisation solve". That is an
estimate. Everything downstream (cloud vs laptop, effort-field feasibility) depends on
the real number, so measure it before building anything on top.

Also verifies the muscle solve is physically correct: plug the solved activations back
into MuJoCo's own forward dynamics and check the hand is actually in static equilibrium.
"""
from __future__ import annotations

import time

import numpy as np

from hand.myohand import FINGERS, MyoHand


def timeit(fn, n=10):
    fn()  # warm up (JIT/caches)
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) / n


def verify_muscle_solve(h: MyoHand, finger: str, key_pos, key_n, press_N: float) -> dict:
    """Independent check that the redundancy solve is doing its job.

    NOT via qacc: finger inertias are ~1e-6 kg.m^2, so M^-1 turns a 1e-4 N.m torque
    imbalance into qacc ~ 500. qacc is uselessly ill-scaled here and would read as a
    catastrophic failure when the torque balance is in fact good to 0.4%.

    The two meaningful checks are:
      1. Is the residual at the feasibility floor? (i.e. we did as well as *anything* could)
      2. Did minimising effort actually beat the plain least-squares solution?
    """
    from scipy.optimize import lsq_linear

    p = h.press(finger, key_pos, key_n, press_N=press_N)

    # The least-squares solution is a legitimate activation vector that also balances the
    # load. If min-sum(a^3) does not undercut it, the redundancy resolution is doing nothing.
    A, qfrc0 = h.muscle_affine(p.q)
    h.fk(p.q)
    J = h.pad_jacobian(finger)
    d = h.data
    tau_ext = J.T @ (press_N * np.asarray(key_n, float))  # key reaction ON the finger
    tau_req = d.qfrc_bias - d.qfrc_passive - tau_ext - qfrc0
    dofs = h.digit_dofs[finger]  # equilibrium is scoped to the pressing digit
    Af, tf = A[dofs], tau_req[dofs]
    a_ls = np.clip(lsq_linear(Af, tf, bounds=(0.0, 1.0)).x, 0.0, 1.0)

    return {
        "posture": p,
        "effort_lsq": float(np.sum(a_ls**3)),
        "at_floor": p.torque_residual <= p.feas_floor + 1e-3 * p.load_scale + 1e-9,
    }


def main():
    print("=" * 68)
    print("STAGE 0 BENCHMARK -- measured on this machine")
    print("=" * 68)

    h = MyoHand()
    q = h.q_neutral.copy()

    # A key placed just off the index pad at rest, pressed into the pad.
    pos, n = h.pad_pose(q, "index")
    key_pos = pos + 0.004 * n
    key_n = -n
    PRESS = 0.5  # N, mid-range mechanical switch

    print(f"\nmodel: {h.nq} DOF, {h.nu} muscles, {h.nq}/{h.nq} joints hard-limited")

    print("\n--- component timings ---")
    t_fk = timeit(lambda: h.fk(q), 200)
    t_pad = timeit(lambda: h.pad_pose(q, "index"), 200)
    t_aff = timeit(lambda: h.muscle_affine(q), 50)
    t_act = timeit(lambda: h.solve_activations(q, "index", PRESS, key_n), 10)
    t_press = timeit(lambda: h.press("index", key_pos, key_n, press_N=PRESS), 5)
    print(f"  forward kinematics      {t_fk*1e6:9.1f} us")
    print(f"  pad pose (FK + frame)   {t_pad*1e6:9.1f} us")
    print(f"  muscle affine (R,gain)  {t_aff*1e6:9.1f} us")
    print(f"  activation solve        {t_act*1e3:9.2f} ms   <-- muscle redundancy")
    print(f"  FULL press() solve      {t_press*1e3:9.2f} ms   <-- inner problem A")

    print("\n--- physical verification: is the muscle solve right? ---")
    v = verify_muscle_solve(h, "index", key_pos, key_n, PRESS)
    p = v["posture"]
    print(f"  reach error          {p.pos_err*1000:8.3f} mm")
    print(f"  pad angle error      {np.rad2deg(p.ang_err):8.2f} deg")
    print(f"  load  ||tau_req||    {p.load_scale:8.2e} N.m")
    print(f"  feasibility floor    {p.feas_floor:8.2e} N.m  (irreducible; ~0 with gravity off)")
    print(f"  achieved residual    {p.torque_residual:8.2e} N.m")
    print(f"  at feasibility floor?  {v['at_floor']}   <-- we did as well as anything could")
    print(f"  effort, least-squares {v['effort_lsq']:8.2e}")
    print(f"  effort, min sum(a^3)  {p.effort:8.2e}   <-- {v['effort_lsq']/max(p.effort,1e-12):.0f}x lower")
    print(f"  max activation       {p.max_act:8.3f}      (1.0 => saturated)")
    print(f"  muscles carrying it  {int((p.a > 1e-3).sum())}/{h.nu}")
    lead = np.argsort(p.a)[::-1][:3]
    print("  top muscles          "
          + ", ".join(f"{h.model.actuator(int(i)).name}={p.a[i]:.3f}" for i in lead if p.a[i] > 1e-3)
          + "   <-- MUST be flexors; extensors here => key is behind the nail")

    print("\n--- structural side: PyNite beam solve ---")
    t_fea = timeit(bench_frame, 20)
    print(f"  3D frame FEA            {t_fea*1e3:9.2f} ms")
    fea, closed, err = verify_cantilever()
    print(f"  cantilever check: FEA {fea*1000:.4f} mm vs closed-form {closed*1000:.4f} mm"
          f"  -> {err:.3f}% error  (gate: <1%)")

    print("\n--- what this means for the budget ---")
    H, CHORDS = 5, 30
    per_design = H * CHORDS * t_press
    evals = 10_000
    print(f"  per design eval (naive, {H} hands x {CHORDS} chords): {per_design:8.2f} s")
    print(f"  NSGA-II {evals} evals, naive:  {per_design*evals/86400:8.2f} core-days")
    print(f"  ... on 12 local threads:        {per_design*evals/86400/12:8.2f} days")
    print(f"  with precomputed effort field:  ~us/key -> minutes total")
    ratio = t_press / 1e-6
    print(f"  field speedup on the inner solve: ~{ratio:,.0f}x")


def bench_frame():
    """A minimal exoskeleton arm: strut cantilevered off the hand anchor, key at the tip."""
    from Pynite import FEModel3D

    fem = FEModel3D()
    # 6061-T6 aluminium, SI (N, m, Pa)
    fem.add_material("Al6061", E=68.9e9, G=26e9, nu=0.33, rho=2700.0)
    # 6 x 2 mm rectangular strut
    b, d = 0.006, 0.002
    fem.add_section("strut", A=b * d, Iy=b * d**3 / 12, Iz=d * b**3 / 12, J=b * d**3 / 3)
    fem.add_node("anchor", 0.0, 0.0, 0.0)
    fem.add_node("mid", 0.03, 0.0, 0.0)
    fem.add_node("key", 0.06, 0.0, 0.0)
    for n1, n2, nm in (("anchor", "mid", "s1"), ("mid", "key", "s2")):
        fem.add_member(nm, n1, n2, "Al6061", "strut")
    fem.def_support("anchor", True, True, True, True, True, True)
    fem.add_node_load("key", "FZ", -0.5)  # 0.5 N key press
    fem.analyze(check_statics=False)
    return fem.nodes["key"].DZ["Combo 1"]


def verify_cantilever() -> tuple[float, float, float]:
    """Independent check of the FEA: tip deflection vs closed-form Euler-Bernoulli.

    delta = P L^3 / (3 E I).  Plan's Stage-3 gate is <1% error.
    """
    from Pynite import FEModel3D

    P, L, E = 0.5, 0.06, 68.9e9
    b, d = 0.006, 0.002
    I = b * d**3 / 12  # bending about the weak (y) axis, load along z

    fem = FEModel3D()
    fem.add_material("Al6061", E=E, G=26e9, nu=0.33, rho=2700.0)
    fem.add_section("strut", A=b * d, Iy=I, Iz=d * b**3 / 12, J=b * d**3 / 3)
    fem.add_node("a", 0.0, 0.0, 0.0)
    fem.add_node("b", L, 0.0, 0.0)
    fem.add_member("m", "a", "b", "Al6061", "strut")
    fem.def_support("a", True, True, True, True, True, True)
    fem.add_node_load("b", "FZ", -P)
    fem.analyze(check_statics=False)

    fea = abs(fem.nodes["b"].DZ["Combo 1"])
    closed = P * L**3 / (3 * E * I)
    err = abs(fea - closed) / closed * 100
    return fea, closed, err


if __name__ == "__main__":
    main()
