"""The well as a CRADLE, not a pin.

THE ERROR THIS FIXES. Every effort and feasibility number so far applied the key reaction as
a SINGLE POINT FORCE AT THE PAD, and demanded the digit's own muscles balance the entire
resulting joint torque. On that model an open hand "cannot press" (32-35% irreducible torque
residual) -- a claim flatly contradicted by the fact that billions of people type on flat
keyboards with semi-extended fingers every day. When a model says something impossible that
people do hourly, the model is wrong.

Piano technique names the missing physics. A pianist does not GENERATE the force with the
finger; the finger is a STRUT that TRANSMITS it, braced, while the arm supplies the weight.
The finger is not stabilising itself against the key.

And a well does exactly that bracing. It is a U-CHANNEL that CRADLES the distal phalanx:

    floor      palmar, running the length of the bone   -> pushes the finger DORSALLY
    two flanks lateral, either side                     -> push the finger INWARD
    end stop   distal                                   -> pushes the finger PROXIMALLY
    (open dorsally and proximally: the finger gets in, and can lift out)

Each is a UNILATERAL contact: it can PUSH and never pull, so lambda >= 0. Together they
supply a distributed wrench, and the crucial consequence is geometric: a reaction spread
along the phalanx has a FAR SMALLER MOMENT ARM ABOUT THE DIP than the same force concentrated
at the fingertip. The cradle carries the torque the muscles could not.

THIS IS WHY A WELL EXISTS. DataHand and Svalboard actuate at 20 gf precisely because the cup
does the stabilising. Modelling the cup as a point force throws away the only thing it is for.

WHAT IS DERIVED, AND WHAT IS ASSUMED:
  * contact POINTS and NORMALS: derived from `well_frame` (the bone's own axis, its palmar
    normal, its flesh radius). No new geometry is invented.
  * `N_ALONG`: how many contact points to discretise the floor/flanks into. A numerical
    choice, not a physical one -- the answer must be shown not to move with it.
  * ⚠ FRICTIONLESS. Every contact pushes along its own normal only. That is CONSERVATIVE:
    friction could only add tangential help. So a "cannot press" verdict from this model is
    still trustworthy; a "can press" verdict is the optimistic end.
"""
from __future__ import annotations

import mujoco
import numpy as np
from scipy.optimize import linprog, lsq_linear

from hand.myohand import MyoHand

N_ALONG = 3  # contact points along the channel. Swept in the sensitivity check.


def contacts(h: MyoHand, q: np.ndarray, finger: str) -> dict[str, list[tuple[np.ndarray, np.ndarray]]]:
    """The cradle's contact points and push directions, in world coords.

    Returns {surface: [(point, normal_pushing_ON_the_finger), ...]} for
    'floor', 'left', 'right', 'end'. All derived from the bone's own frame.
    """
    wf = h.well_frame(q, finger)
    pos, axis, floor, lat = wf["pos"], wf["axis"], wf["floor"], wf["lateral"]
    L, r = 2.0 * wf["half"], wf["radius"]

    # points along the PALMAR surface of the phalanx, from the pad tip going proximally.
    # `pos` is already on the palmar surface (it is the pulp), so walking back along -axis
    # stays on it.
    along = [pos - s * axis for s in np.linspace(0.0, L, N_ALONG)]

    return {
        # the floor is palmar of the finger, so it pushes the finger DORSALLY: -floor
        "floor": [(p, -floor) for p in along],
        # the flank at -lateral pushes the finger toward +lateral, and vice versa
        "left": [(p - r * lat, +lat) for p in along],
        "right": [(p + r * lat, -lat) for p in along],
        # the end stop is distal of the tip and pushes back PROXIMALLY
        "end": [(pos + 0.004 * axis, -axis)],
    }


# which contact surface each joystick action pushes against. `back` has NONE: the channel is
# open proximally (that is how the finger gets in), so there is nothing to push against, and
# the digit must produce that one with muscle alone. Stated, not hidden.
SENSED = {"click": "floor", "left": "left", "right": "right", "forward": "end", "back": None}


def _cols(h: MyoHand, q: np.ndarray, finger: str):
    """(A over the digit's dofs, rest torque, {surface: G matrix})."""
    h.fk(q)
    A, qfrc0 = h.muscle_affine(q)
    dofs = h.digit_dofs[finger]
    rest = qfrc0[dofs] + h.data.qfrc_passive[dofs] - h.data.qfrc_bias[dofs]
    Ad = A[np.ix_(dofs, np.arange(A.shape[1]))]

    bid = h.pad[finger][0]
    G = {}
    for surf, pts in contacts(h, q, finger).items():
        cols = []
        for p, n in pts:
            jacp = np.zeros((3, h.model.nv))
            mujoco.mj_jac(h.model, h.data, jacp, None, np.asarray(p, float), bid)
            cols.append(jacp[:, dofs].T @ np.asarray(n, float))
        G[surf] = np.array(cols).T  # (ndof, npts)
    return Ad, rest, G


def residual(h: MyoHand, q: np.ndarray, finger: str, action: str, press_N: float) -> float:
    """Fraction of the required joint torque the muscles CANNOT produce, WITH the cradle.

    The cradle's contacts are free, non-negative unknowns: the finger may lean on the floor,
    the flanks and the end stop as hard as it likes, and the well pushes back. What it may
    NOT do is pull on them. The switch actuates when the SENSED surface carries `press_N`.
    """
    Ad, rest, G = _cols(h, q, finger)
    surf = SENSED[action]
    if surf is None:  # nothing to push against: muscles alone, the old model
        from design.vector import action_dirs

        d = action_dirs(h, q, finger)[action]
        h.fk(q)
        # the reaction ON the fingertip when it pushes along +d is -press_N*d, so
        #     Ad a + rest + J^T(-press_N d) = 0   =>   Ad a = -rest + press_N J^T d
        # (getting this sign backwards made `back` come out WORSE with the cradle than
        # without it, which is impossible -- the fallback IS the no-cradle model.)
        tau_ext = h.pad_jacobian(finger)[:, h.digit_dofs[finger]].T @ (-press_N * d)
        tgt = -rest - tau_ext
        a = np.clip(lsq_linear(Ad, tgt, bounds=(0.0, 1.0)).x, 0.0, 1.0)
        return float(np.linalg.norm(Ad @ a - tgt) / (np.linalg.norm(tgt) + 1e-12))

    # ONLY THE SENSED SURFACE IS LOADED, AND ITS CONTACTS SUM TO press_N.
    #
    # The first version let the finger lean on the floor, BOTH walls and the end stop at
    # once, with unbounded forces. Those SELF-CANCEL -- a left-wall push balanced by a
    # right-wall push -- so the LP conjured a spurious internal preload that costs no muscle
    # at all, and the switch "actuated" with the finger completely passive. With gravity off,
    # a passive finger in a well exerts ZERO force; it cannot press a key by being squeezed.
    #
    # The control caught it: that model said the THUMB could press 4 of 5 directions. The
    # thumb has no adductor. A cradle cannot lend a digit a muscle it does not have, and any
    # model that says otherwise is wrong.
    #
    # What a cradle ACTUALLY buys is one thing only, and it is geometric: the reaction bears
    # on the WHOLE PALMAR SURFACE of the distal phalanx, not on a point at the fingertip. So
    # the CENTRE OF PRESSURE is free to sit anywhere along the bone, and a reaction bearing
    # near the DIP has a far smaller moment arm about it than the same force at the tip. That
    # is the finger-as-a-strut, and it is the whole of what the well contributes.
    G_s = G[surf]                      # (ndof, npts) on the sensed surface only
    nu, npts = Ad.shape[1], G_s.shape[1]
    ndof = Ad.shape[0]

    # slack e (free sign) is the torque muscles + cradle CANNOT supply. Minimise ||e||_1.
    #     Ad a + G_s lam + rest + e = 0,   sum(lam) = press_N,   a in [0,1],  lam >= 0
    c = np.concatenate([np.zeros(nu + npts), np.ones(2 * ndof)])
    A_eq = np.vstack([
        np.hstack([Ad, G_s, np.eye(ndof), -np.eye(ndof)]),
        np.concatenate([np.zeros(nu), np.ones(npts), np.zeros(2 * ndof)])[None, :],
    ])
    b_eq = np.concatenate([-rest, [press_N]])
    r = linprog(
        c, A_eq=A_eq, b_eq=b_eq,
        bounds=[(0.0, 1.0)] * nu + [(0.0, None)] * npts + [(0.0, None)] * (2 * ndof),
        method="highs",
    )
    if not r.success:
        return 1.0
    e = r.x[nu + npts:nu + npts + ndof] - r.x[nu + npts + ndof:]
    # scale by the torque a POINT reaction at the pad would demand -- the pin model's own
    # denominator, so cradle and pin residuals are directly comparable.
    from design.vector import action_dirs

    d = action_dirs(h, q, finger)[action]
    h.fk(q)
    tgt = -rest - (h.pad_jacobian(finger)[:, h.digit_dofs[finger]].T @ (-press_N * d))
    return float(np.linalg.norm(e) / (np.linalg.norm(tgt) + 1e-12))


def solve(h: MyoHand, q: np.ndarray, finger: str, action: str, press_N: float):
    """(activations, effort=sum a^3, relative residual, max activation) with the cradle.

    LEXICOGRAPHIC, the same shape as MyoHand.solve_activations and for the same reason: a
    soft trade between "balance the load" and "be comfortable" is what killed v1.
      Phase 1  LP: minimise the unbalanceable torque.      -> can the digit do this AT ALL?
      Phase 2  NLP: minimise sum(a^3) subject to that.     -> what does it COST?
    """
    from scipy.optimize import minimize

    Ad, rest, G = _cols(h, q, finger)
    surf = SENSED[action]

    if surf is None:  # nothing to push against -- muscles alone
        from design.vector import action_dirs

        d = action_dirs(h, q, finger)[action]
        h.fk(q)
        tgt = -rest - (h.pad_jacobian(finger)[:, h.digit_dofs[finger]].T @ (-press_N * d))
        a_ls = np.clip(lsq_linear(Ad, tgt, bounds=(0.0, 1.0)).x, 0.0, 1.0)
        rel = float(np.linalg.norm(Ad @ a_ls - tgt) / (np.linalg.norm(tgt) + 1e-12))
        tau_star = Ad @ a_ls
        cons = [{"type": "eq", "fun": lambda a: Ad @ a - tau_star, "jac": lambda a: Ad}]
        res = minimize(lambda a: float(np.sum(a**3)), a_ls, jac=lambda a: 3.0 * a**2,
                       bounds=[(0.0, 1.0)] * Ad.shape[1], constraints=cons, method="SLSQP",
                       options={"maxiter": 200, "ftol": 1e-10})
        a = np.clip(res.x, 0.0, 1.0)
        return a, float(np.sum(a**3)), rel, float(a.max())

    G_s = G[surf]

    # THE FINGER IS STILL RESTING IN THE WELL. When it presses any wall OTHER than the floor
    # (a lateral tilt, or the end stop), the FLOOR still cradles it -- a free, non-negative dorsal
    # support that stabilises the DIP/IP exactly as it does for a click. The old model withheld it
    # and so demanded a MUSCLE for a lateral load the floor (and, in a real finger, the DIP
    # collateral ligaments -- which MyoHand does not model) actually carry. That is what flagged
    # middle/right and ring/right as infeasible over ~1 mN·m of DIP torque.
    #
    # ⚠ ONLY THE FLOOR, NEVER THE OPPOSING WALL. The self-cancelling-preload bug (a left-wall push
    # balanced by a right-wall push, conjuring a keypress from a passive finger) needs two opposed
    # surfaces. The floor's contact normals ALL point dorsally, so they cannot self-cancel, and the
    # floor supplies no lateral force -- so it cannot fake the wall press. The control in
    # test_design (a thumb with no adductor still presses nothing) is what verifies this.
    G_f = G["floor"] if surf != "floor" else np.zeros((Ad.shape[0], 0))
    nsup = G_f.shape[1]
    nu, npts, ndof = Ad.shape[1], G_s.shape[1], Ad.shape[0]

    # Phase 1: the smallest torque the muscles + cradle cannot supply. Variables: a (muscles),
    # lam (sensed-wall forces, summing to press_N), mu (free floor support), e+/e- (slack).
    c = np.concatenate([np.zeros(nu + npts + nsup), np.ones(2 * ndof)])
    A_eq = np.vstack([
        np.hstack([Ad, G_s, G_f, np.eye(ndof), -np.eye(ndof)]),
        np.concatenate([np.zeros(nu), np.ones(npts), np.zeros(nsup + 2 * ndof)])[None, :],
    ])
    b_eq = np.concatenate([-rest, [press_N]])
    bnds = ([(0.0, 1.0)] * nu + [(0.0, None)] * npts + [(0.0, None)] * nsup
            + [(0.0, None)] * (2 * ndof))
    r1 = linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=bnds, method="highs")
    if not r1.success:
        return np.zeros(nu), float("inf"), 1.0, 1.0

    off = nu + npts + nsup
    e = r1.x[off:off + ndof] - r1.x[off + ndof:]
    from design.vector import action_dirs

    d = action_dirs(h, q, finger)[action]
    h.fk(q)
    tgt = -rest - (h.pad_jacobian(finger)[:, h.digit_dofs[finger]].T @ (-press_N * d))
    rel = float(np.linalg.norm(e) / (np.linalg.norm(tgt) + 1e-12))

    # Phase 2: min sum(a^3) at that feasibility. The residual is PINNED to what phase 1
    # achieved -- not bounded by it -- so effort can never be bought with torque error.
    e_star = e

    def eq(z):
        a, lam, mu = z[:nu], z[nu:nu + npts], z[nu + npts:]
        return np.concatenate([Ad @ a + G_s @ lam + G_f @ mu + rest + e_star,
                               [lam.sum() - press_N]])

    z0 = np.concatenate([r1.x[:nu], r1.x[nu:nu + npts], r1.x[nu + npts:nu + npts + nsup]])
    res = minimize(
        lambda z: float(np.sum(z[:nu] ** 3)),
        z0,
        jac=lambda z: np.concatenate([3.0 * z[:nu] ** 2, np.zeros(npts + nsup)]),
        bounds=[(0.0, 1.0)] * nu + [(0.0, None)] * (npts + nsup),
        constraints=[{"type": "eq", "fun": eq}],
        method="SLSQP", options={"maxiter": 200, "ftol": 1e-10},
    )
    a = np.clip(res.x[:nu], 0.0, 1.0)
    return a, float(np.sum(a**3)), rel, float(a.max())
