"""How hard can a digit actually push? The instrument, built BEFORE the fix.

WHY THIS EXISTS. We are about to add muscles to MyoHand (adductor pollicis and friends).
Adding a muscle is easy; knowing whether you made the model BETTER or WORSE is the whole
problem -- and right now we cannot tell, because we have no validated strength measurement.
Two earlier attempts produced nonsense (0 N for the middle finger; 400 N for the thumb), and
a fix validated by a broken instrument is not a fix, it is a coin toss.

So: build the instrument, check it against digits whose real strength IS known (the fingers),
and only then point it at the thumb.

THE PHYSICS. At a fixed posture with zero velocity, MuJoCo's static balance on the digit's
own dofs is

    qfrc_actuator(a) + qfrc_passive - qfrc_bias + J^T f_ext = 0

and muscle force is affine in activation at fixed length, so qfrc_actuator(a) = qfrc0 + A a
(`muscle_affine`, which builds A from MuJoCo's own forward pass rather than unpacking the
sparse moment matrix -- exact by construction).

If the digit PUSHES on the world with force s*d, the world pushes BACK on the fingertip with
f_ext = -s*d. Substituting:

    A a = s * (J^T d) - qfrc0 - qfrc_passive + qfrc_bias

Maximising s subject to 0 <= a <= 1 is a LINEAR PROGRAM -- exactly solvable, no local minima,
no initial guess. That is the right shape for this question: "can the muscles do it at all"
is a feasibility question, not an optimisation one.

⚠ WHAT THIS IS NOT. It is a single-digit, quasi-static, isometric ceiling with the rest of the
hand held rigid. It ignores whether the wrist/arm could brace against the reaction, and it
takes MuJoCo's peak forces at face value. It is a MODEL of strength, and it is being used to
compare a model against itself before and after a change -- which is what it is good for.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import linprog

from hand.myohand import FINGERS, MyoHand

S_MAX = 500.0  # N. A ceiling, so an unbounded LP reports "unbounded" rather than hanging.


def max_push(h: MyoHand, q: np.ndarray, finger: str, d: np.ndarray) -> float:
    """Largest force `finger` can exert at its pad along unit vector `d`, activations <= 1.

    Returns N. A value at S_MAX means the direction costs almost no joint torque (you are
    pushing along the bone), not that the digit is superhuman -- read it as "unbounded".
    """
    d = np.asarray(d, float)
    d = d / (np.linalg.norm(d) + 1e-12)

    h.fk(q)
    A, qfrc0 = h.muscle_affine(q)
    dofs = h.digit_dofs[finger]
    J = h.pad_jacobian(finger)

    # everything that is NOT muscle activation: passive joint forces, and bias (coriolis +
    # gravity; gravity is off in this model and velocity is zero, so bias is ~0 -- but read
    # it rather than assume it)
    rest = qfrc0[dofs] + h.data.qfrc_passive[dofs] - h.data.qfrc_bias[dofs]
    Ad = A[np.ix_(dofs, np.arange(A.shape[1]))]
    g = (J[:, dofs].T @ d)  # generalised force per newton of push

    n = Ad.shape[1]
    c = np.zeros(n + 1)
    c[-1] = -1.0  # maximise s
    A_eq = np.hstack([Ad, -g.reshape(-1, 1)])
    b_eq = -rest
    r = linprog(c, A_eq=A_eq, b_eq=b_eq,
                bounds=[(0.0, 1.0)] * n + [(0.0, S_MAX)], method="highs")
    if not r.success:
        return 0.0
    return float(r.x[-1])


def press_strength(h: MyoHand, q: np.ndarray) -> dict[str, float]:
    """Max press force for every digit, each along ITS OWN pad normal -- i.e. a keypress."""
    out = {}
    for f in FINGERS:
        _, n = h.pad_pose(q, f)
        out[f] = max_push(h, q, f, n)
    return out


def best_press(h: MyoHand, finger: str, n_grid: int = 9) -> tuple[float, np.ndarray]:
    """Max press force over POSTURE as well as activation. Returns (N, best q).

    POSTURE IS PART OF STRENGTH, and leaving it out gives an absurd answer. At `q_neutral`
    this model says the MIDDLE FINGER CANNOT PRESS AT ALL (0 N) -- and that is not a bug in
    MyoHand, it is a bug in the question. The long flexors are SHARED: FDP3 drives the MCP,
    PIP and DIP in one fixed torque ratio, so at a mid-range posture no activation vector can
    balance all four dofs at once and still deliver a fingertip force.

    A real finger does not press from mid-range. It presses with the distal joint BRACED
    AGAINST ITS EXTENSION LIMIT -- the ligament supplies the torque the muscles cannot, and the
    finger acts as a strut. Published pinch and press strengths are likewise measured at a
    FUNCTIONAL posture, not an arbitrary one. So the instrument has to search posture too, or
    it is not measuring the same quantity the literature reports.

    Note what this means for the effort model: `press()` gates on `press_travel`, which keeps
    it away from the limits. That is the right call for TYPING (you must have travel left) and
    it is why effort, not strength, is the objective. This function answers a different
    question -- the strength CEILING -- and it is used only to validate the model.
    """
    from design.vector import posture

    best, best_q = 0.0, h.q_neutral.copy()
    for tp in np.linspace(0.05, 0.95, n_grid):
        for tm in np.linspace(0.05, 0.95, n_grid):
            q = posture(h, finger, float(tp), float(tm), 0.0)
            _, n = h.pad_pose(q, finger)
            s = max_push(h, q, finger, n)
            if s > best:
                best, best_q = s, q
    return best, best_q


def recruited(h: MyoHand, q: np.ndarray, finger: str, d: np.ndarray, top: int = 4):
    """Which muscles the LP actually uses. The check that no amount of self-consistency
    survives: if a 'thumb press' recruits a ring lumbrical, the model is answering the wrong
    question, and only the muscle names give it away."""
    import mujoco

    d = np.asarray(d, float) / (np.linalg.norm(d) + 1e-12)
    h.fk(q)
    A, qfrc0 = h.muscle_affine(q)
    dofs = h.digit_dofs[finger]
    J = h.pad_jacobian(finger)
    rest = qfrc0[dofs] + h.data.qfrc_passive[dofs] - h.data.qfrc_bias[dofs]
    Ad = A[np.ix_(dofs, np.arange(A.shape[1]))]
    g = J[:, dofs].T @ d
    n = Ad.shape[1]
    c = np.zeros(n + 1)
    c[-1] = -1.0
    r = linprog(c, A_eq=np.hstack([Ad, -g.reshape(-1, 1)]), b_eq=-rest,
                bounds=[(0.0, 1.0)] * n + [(0.0, S_MAX)], method="highs")
    if not r.success:
        return []
    a = r.x[:n]
    names = [mujoco.mj_id2name(h.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(n)]
    idx = np.argsort(-a)[:top]
    return [(names[i], float(a[i])) for i in idx if a[i] > 1e-3]


if __name__ == "__main__":
    h = MyoHand()
    print("MAX PRESS FORCE along each digit's own pad normal, over ALL postures (a <= 1):\n")
    for f in FINGERS:
        s, q = best_press(h, f)
        cap = "  <-- AT CEILING (pushing along the bone; read as unbounded)" \
            if s >= S_MAX - 1e-6 else ""
        print(f"  {f:7s} {s:7.1f} N{cap}")
        for nm, a in recruited(h, q, f, h.pad_pose(q, f)[1]):
            print(f"            {nm:9s} a={a:.2f}")
    print("""
Published human values, for calibration:
  index finger tip press   ~30-50 N
  thumb-index tip pinch    ~45-70 N       a REAL thumb is STRONGER than a real index.""")
