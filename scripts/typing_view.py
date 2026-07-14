"""WATCH IT TYPE.  PYTHONPATH=. .venv/bin/python scripts/typing_view.py

THE USER: "What would be very nice would be to animate the hand pose through each of the various
target poses."

And it is not decoration -- it is the first thing that would show whether the device WORKS. Every
number in this project says the layout is feasible; none of them shows a finger actually reaching
into its well and pressing in the direction it is supposed to. A picture does, and this project's
entire history is of pictures catching what numbers did not.

So: for every WIRED (digit, direction) -- the 15 the layout actually assigns a character to --
pose the hand INTO that press and render it.

THE POSE IS DERIVED, NOT DRAWN. The fingertip has to move `travel` millimetres along the action's
own direction, so the joint angles come from the pad JACOBIAN:

    dq = J+ (travel * direction)          restricted to that digit's own dofs

which is the same Jacobian the effort model uses to turn a switch force into joint torques. The
hand you see pressing is the hand the optimiser was reasoning about -- not an artist's impression
of it.

Each frame carries what it cost: the muscle effort (sum a^3) and the equilibrium residual, so a
direction that is expensive or that the digit cannot actually balance is visible as well as
tabulated.
"""
from __future__ import annotations

import json
import pickle

import numpy as np

from design.params import DEFLECTION_MAX  # noqa: F401  (kept: the gate this structure met)
from design.qwerty import used_actions
from design.vector import PRESS_N, SWITCH_TRAVEL, action_dirs, evaluate, posture, tm_of, tp_of
from hand.cradle import solve as cradle_solve
from hand.flesh import CARPUS, METACARPALS, PHALANGES
from hand.myohand import FINGERS, FLEXION_JOINTS
from opt.problem import hands
from structure.frame import hand_axes

FINGER_COLOUR = {"thumb": "#e8590c", "index": "#12639b", "middle": "#2f9e44",
                 "ring": "#9c36b5", "little": "#c2255c"}


def digit_dofs(h, finger):
    """The joint-velocity indices this digit -- and only this digit -- can move."""
    import mujoco

    m = h.model
    out = []
    for j in FLEXION_JOINTS[finger]:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        if jid >= 0:
            out.append(int(m.jnt_dofadr[jid]))
    if finger != "thumb":
        d = {"index": "2", "middle": "3", "ring": "4", "little": "5"}[finger]
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"mcp{d}_abduction")
        if jid >= 0:
            out.append(int(m.jnt_dofadr[jid]))
    return sorted(set(out))


def pressed(h, q, finger, direction, travel):
    """The posture in which `finger` has pushed its pad `travel` along `direction`.

    Derived from the pad Jacobian and restricted to that digit's own dofs -- pressing a key with
    the index finger does not move the ring finger, and letting it would invent whole-hand
    contortions that no one performs.
    """
    h.fk(q)
    J = h.pad_jacobian(finger)[:, digit_dofs(h, finger)]
    dq = np.linalg.lstsq(J, np.asarray(direction, float) * travel, rcond=None)[0]
    q2 = np.array(q, float).copy()
    for k, v in zip(digit_dofs(h, finger), dq):
        q2[k] += v
    return q2


def bones(h, q):
    """The hand as a stick skeleton -- THE HAND, not the room and the human figure that
    myohand.xml also ships."""
    import mujoco

    m = h.model
    h.fk(q)
    keep = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
            for b in CARPUS + METACARPALS + PHALANGES}
    keep.discard(-1)
    # ⚠ BOTH ENDS MUST BE HAND BONES. The parent of a CARPAL is the FOREARM, whose origin is at
    # the ELBOW -- so a parent-to-child segment there draws a bone from the elbow to the wrist,
    # straight across the render.
    x, y, z = [], [], []
    for b in keep:
        p = int(m.body_parentid[b])
        if p <= 0 or p not in keep:
            continue
        a, c = h.data.xpos[p], h.data.xpos[b]
        if np.linalg.norm(c - a) > 1e-4:
            x += [float(a[0]), float(c[0]), None]
            y += [float(a[1]), float(c[1]), None]
            z += [float(a[2]), float(c[2]), None]

    # ⚠ AND THE FINGERTIPS. A parent-to-child segment cannot draw the LAST bone, because a distal
    # phalanx has no child -- so the fingers stopped at the DIP joint and never reached the wells
    # they are supposed to be pressing. Which is the one thing this whole animation exists to show.
    for f in FINGERS:
        tip = h.pad_pose(q, f)[0]
        dip = h.data.xpos[h.pad[f][0]]
        x += [float(dip[0]), float(tip[0]), None]
        y += [float(dip[1]), float(tip[1]), None]
        z += [float(dip[2]), float(tip[2]), None]
    return x, y, z


def main():
    H = hands()
    ref = H[50]
    fd = pickle.load(open("out/final_design.pkl", "rb"))
    x = fd["x"]
    amap = fd["action_map"]
    r = evaluate(x, H)
    wired = used_actions(amap)

    q0 = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                      for f in FINGERS})
    travel = float(SWITCH_TRAVEL)

    # THE CHARACTER EACH (DIGIT, DIRECTION) TYPES.
    # `action_map` is {finger: {qwerty_row: direction}} -- it says which DIRECTION stands for
    # which ROW on that finger. The letter itself comes from QWERTY, and the thumb carries the
    # modifiers (space, shift), which is the whole point of `qwerty_plus_thumb`.
    from design.layout import MODIFIERS
    from design.qwerty import QWERTY_LEFT

    char_of = {}
    for fng, rows in amap.items():
        for row, act in rows.items():
            ch = QWERTY_LEFT.get((fng, row)) or MODIFIERS.get(row) and row or row
            if fng == "thumb":
                ch = row                       # "space" / "shift"
            char_of[(fng, act)] = str(ch)

    z = np.load("out/final.npz", allow_pickle=True)
    nodes, bars = z["nodes"], [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    gx, gy, gz = [], [], []
    for e in live:
        a, b = nodes[bars[e][0]], nodes[bars[e][1]]
        gx += [float(a[0]), float(b[0]), None]
        gy += [float(a[1]), float(b[1]), None]
        gz += [float(a[2]), float(b[2]), None]

    frames = []
    print(f"posing the hand into each of the {sum(len(v) for v in wired.values())} wired presses\n")
    print(f"  {'char':>5s} {'digit':>7s} {'direction':>10s} {'travel':>8s} {'effort':>11s} "
          f"{'residual':>9s}")
    for f in FINGERS:
        dirs = action_dirs(ref, q0, f)
        for act in sorted(wired.get(f, [])):
            d = np.asarray(dirs[act], float)
            q = pressed(ref, q0, f, d, travel)

            _a, eff, resid, _smax = cradle_solve(ref, q0, f, act, PRESS_N)
            moved = float(np.linalg.norm(ref.pad_pose(q, f)[0] - ref.pad_pose(q0, f)[0]))
            ch = char_of.get((f, act), "·")
            bx, by, bz = bones(ref, q)
            frames.append(dict(
                char=str(ch), finger=f, action=act,
                effort=float(eff), residual=float(resid), travel_mm=moved * 1000,
                bones=[bx, by, bz],
                colour=FINGER_COLOUR[f]))
            print(f"  {str(ch):>5s} {f:>7s} {act:>10s} {moved*1000:6.2f}mm {eff:11.3e} "
                  f"{resid*100:8.1f}%")

    o, e_d, e_r, e_o = hand_axes(ref, q0)
    eye = 1.65 * e_o - 0.88 * e_d + 0.22 * e_r
    rest_b = bones(ref, q0)

    payload = dict(
        frames=frames,
        rest=[rest_b[0], rest_b[1], rest_b[2]],
        gauntlet=[gx, gy, gz],
        camera=dict(eye=[float(v) for v in eye], up=[float(v) for v in e_d]),
    )
    with open("out/typing.json", "w") as fh:
        json.dump(payload, fh)
    print(f"\n  wrote out/typing.json ({len(frames)} presses)")
    print("  browser view: out/typing.html")


if __name__ == "__main__":
    main()
