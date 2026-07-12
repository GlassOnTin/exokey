"""MyoHand wrapper: forward kinematics, muscle redundancy, and effort.

The model (MyoSuite `myo_sim`, Apache-2.0) is 23 DOF / 39 Hill-type muscles.
All 23 joints carry hard limits, which we treat as hard constraints everywhere --
this is the defect that stalled CapaChord v1 (its IK ran unbounded).

Effort is muscle activation, sum(a**3) (Crowninshield-Brand), NOT a geometric proxy.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass

os.environ.setdefault("MUJOCO_GL", "disable")  # physics only; we render via Plotly

import mujoco
import numpy as np
from scipy.optimize import lsq_linear, minimize

HAND_XML = os.path.join(os.path.dirname(__file__), "..", "data", "myo_sim", "hand", "myohand.xml")

# Distal-phalanx bodies.
PAD_BODIES = {
    "thumb": "distal_thumb",
    "index": "distph2",
    "middle": "distph3",
    "ring": "distph4",
    "little": "distph5",
}
FINGERS = tuple(PAD_BODIES)

# The model's own fingertip markers.
TIP_SITES = {
    "thumb": "THtip", "index": "IFtip", "middle": "MFtip",
    "ring": "RFtip", "little": "LFtip",
}

# (flexor, extensor) tendon insertion sites on each distal phalanx.
#
# These are how we find the PALMAR side of the bone, and they are the only unambiguous
# way to do it. Flexor tendons run on the palmar side, extensors on the dorsal side --
# that is anatomy, encoded in the model itself -- so the vector from the extensor
# insertion to the flexor insertion points dorsal->palmar. No convention to guess.
#
# The obvious-looking alternative is a trap. Each distal body also carries a flat
# `class="skin"` ellipsoid that reads like a finger pad; it is NOT. On distph2 it sits at
# z=+0.0055 while the flexor FDP2 inserts at z=-0.004 -- the ellipsoid is on the EXTENSOR
# side. It is the FINGERNAIL. Using it (and MuJoCo gives no guarantee its thin axis points
# outward either) put every key behind the nail and made a "keypress" recruit EIP, an
# extensor. The physics was self-consistent and the answer was to the wrong question.
PAD_TENDONS = {
    "thumb": ("FPL-P11", "EPL-P12"),
    "index": ("FDP2-P10", "EDC2-P9"),
    "middle": ("FDP3-P10", "EDC3-P9"),
    "ring": ("FDP4-P10", "EDC4-P9"),
    "little": ("FDP5-P10", "EDC5-P9"),
}

# The joints of each digit. Effort for a key is scoped to the digit that presses it --
# see solve_activations().
DIGIT_JOINTS = {
    "thumb": ("cmc_abduction", "cmc_flexion", "mp_flexion", "ip_flexion"),
    "index": ("mcp2_flexion", "mcp2_abduction", "pm2_flexion", "md2_flexion"),
    "middle": ("mcp3_flexion", "mcp3_abduction", "pm3_flexion", "md3_flexion"),
    "ring": ("mcp4_flexion", "mcp4_abduction", "pm4_flexion", "md4_flexion"),
    "little": ("mcp5_flexion", "mcp5_abduction", "pm5_flexion", "md5_flexion"),
}

# The FLEXION chain of each digit, proximal -> distal. Named explicitly rather than
# indexed out of DIGIT_JOINTS: the thumb's tuple starts with cmc_ABDUCTION, and taking
# "joints[0]" as the flexion axis swept the thumb's abduction instead -- while the
# DIP-analogue it did pick, ip_flexion, has range [-75, +25] deg. Sweeping "5% of range"
# then drove the thumb to -72 deg, deeply HYPEREXTENDED, and that posture won on effort.
#
# The four fingers cannot hyperextend (every flexion joint is [0, 90]); the thumb's four
# joints ALL go negative. Anything sweeping postures must clamp to the flexion side --
# see flexion_span().
FLEXION_JOINTS = {
    "thumb": ("cmc_flexion", "mp_flexion", "ip_flexion"),
    "index": ("mcp2_flexion", "pm2_flexion", "md2_flexion"),
    "middle": ("mcp3_flexion", "pm3_flexion", "md3_flexion"),
    "ring": ("mcp4_flexion", "pm4_flexion", "md4_flexion"),
    "little": ("mcp5_flexion", "pm5_flexion", "md5_flexion"),
}

# The digit's own long flexor. Used to DERIVE which way each joint flexes -- never assume it.
#
# A joint called `*_flexion` does not tell you which sign is flexion, and in this model the
# THUMB IS NOT EVEN SELF-CONSISTENT:
#
#     cmc_flexion   FPL = +0.46  -> flexion is POSITIVE
#     mp_flexion    FPL = -1.52  -> flexion is NEGATIVE
#     ip_flexion    FPL = -1.76  -> flexion is NEGATIVE
#
# Confirmed by geometry: driving mp/ip negative curls the thumb toward the index; positive
# pushes it away. Read ip_flexion's range [-75, +25] with the correct sign and it is 75 deg
# of flexion and 25 deg of hyperextension, which is a thumb. Read it backwards -- as I did
# -- and it is 75 deg of HYPERextension, which is nothing.
#
# Everything thumb-shaped that looked wrong traces to this: "ip_flexion = -25 is
# hyperextended" (it is normal flexion), the rest posture that put the MP and IP into
# EXTENSION, the posture sweeps that ran the thumb backwards, the 159 deg between its press
# direction and its pad normal, and its zero press travel.
#
# The flexor's moment arm is the model's own unambiguous statement of which way is flexion.
FLEXORS = {
    "thumb": "FPL", "index": "FDP2", "middle": "FDP3", "ring": "FDP4", "little": "FDP5",
}


@dataclass
class Posture:
    """Result of a posture solve: joint angles, activations, and what they cost."""

    q: np.ndarray  # (23,) joint angles, rad
    a: np.ndarray  # (39,) muscle activations in [0,1]
    effort: float  # sum(a**3)
    pos_err: float  # m, fingertip to key target
    ang_err: float  # rad, pad normal to key normal
    torque_residual: float  # N*m, how badly muscles fail to balance the load
    feas_floor: float  # N*m, irreducible residual (best any activation can do)
    load_scale: float  # N*m, ||tau_required||, for normalising the two above
    max_act: float  # max muscle activation; 1.0 => saturated
    ok: bool

    @property
    def saturated(self) -> bool:
        """A muscle is maxed out: the key demands more than the finger can deliver.

        This -- not the gravity floor -- is the design-relevant feasibility signal.
        The gravity floor is a constant of the model+posture (MyoHand cannot fully
        balance gravity on the weakly-actuated MCP abduction dofs, ~7% of the gravity
        torque), identical for every candidate design, so gating on it would reject all
        designs equally and say nothing about the keyboard.
        """
        return self.max_act > 0.95


class MyoHand:
    def __init__(self, xml: str = HAND_XML, gravity: bool = False, scale: float = 1.0):
        """`gravity=False` (default) is a deliberate modelling choice, not an oversight.

        A hand-mounted keyboard is used with the hand in every orientation -- typing while
        standing, walking, arm at your side. There is no single right gravity vector, so
        fixing one bakes in an arbitrary assumption. Measured, it is also a small and
        actively harmful one:

          * the gravity torque on a digit is ~10x SMALLER than the torque from a 0.5 N
            keypress, and near-orthogonal to it (cos = -0.03 for the index);
          * it lands mostly on the MCP *abduction* dofs, which MyoHand actuates weakly
            (max|moment arm| ~0.3-0.75 vs 1.5-3.5 for flexion), so it is expensive per
            unit torque and MyoHand cannot even balance it exactly -- an irreducible
            residual that is a property of the model, not of any keyboard;
          * worst, it makes effort NON-MONOTONIC in press force for the index: pressing
            harder can redirect the demand onto a well-actuated muscle and come out
            cheaper. An optimiser handed that would happily choose stiffer switches to
            reduce "effort". Perverse, and traceable entirely to gravity.

        Turning it off removes all three. Set gravity=True to measure the sensitivity --
        scripts/stage1_view.py reports how far the effort ranking actually moves.
        """
        self.model = mujoco.MjModel.from_xml_path(os.path.normpath(xml))
        self.scale = float(scale)
        if self.scale != 1.0:
            from hand.scaling import scale_model

            scale_model(self.model, self.scale)
        self.data = mujoco.MjData(self.model)
        self.gravity = gravity
        if not gravity:
            self.model.opt.gravity[:] = 0.0
        m = self.model
        self.nq, self.nu = m.nq, m.nu
        self.lo = m.jnt_range[:, 0].copy()
        self.hi = m.jnt_range[:, 1].copy()
        # Neutral posture: mid-range. A stand-in for the hand's functional rest pose;
        # replaced by a measured rest pose in Stage 1 if it matters.
        self.pad = {f: self._pad_frame(f) for f in FINGERS}
        self.flexion_axes = self._flexion_axes()  # must precede q_neutral: it uses them
        self.q_neutral = self._functional_rest()

        # Equilibrium is imposed on the PRESSING DIGIT's dofs. Nothing else.
        #
        # Twice now, widening this scope has poisoned the effort signal with a large,
        # design-independent constant, and twice the symptom was the same: effort stopped
        # rising with press force.
        #
        #   * Include the forearm/shoulder, and ||tau_req|| is dominated by holding the ARM
        #     up against gravity. The device straps to the wrist, so the arm carries its own
        #     weight regardless of key layout -- this is not a modelling convenience, it is
        #     what the device physically does.
        #   * Include the other four digits, and ~90% of sum(a^3) is THEIR gravity: an
        #     "index press" came out led by LU_RB4 (a ring lumbrical) and EPB (a thumb
        #     extensor), with the actual press (FDP2) contributing ~10%. A light press could
        #     then incidentally relieve some other digit's gravity torque and REDUCE effort.
        #
        # The plan's effort field is per-finger -- E_{f,h}(x, n) -- so the pressing digit is
        # the right scope. All 39 muscles stay available; min sum(a^3) simply zeroes the
        # ones that do not help, so no hand-picked muscle subset is needed.
        self.digit_dofs = {}
        self.digit_qadr = {}  # qpos addresses, for composing a posture per-digit
        for f, joints in DIGIT_JOINTS.items():
            dofs, qadr = [], []
            for name in joints:
                jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
                if jid < 0:
                    raise ValueError(f"joint {name} not in model")
                dofs.append(int(m.jnt_dofadr[jid]))
                qadr.append(int(m.jnt_qposadr[jid]))
            self.digit_dofs[f] = np.array(sorted(dofs), dtype=int)
            self.digit_qadr[f] = np.array(sorted(qadr), dtype=int)

    def _flexion_axes(self) -> dict[str, list[tuple[int, int, float]]]:
        """Per digit, its flexion chain as (qpos adr, dof adr, FULL-FLEXION angle).

        The full-flexion angle is signed: it is the joint limit lying in the direction the
        digit's own long flexor drives that joint. Derived, never assumed -- see FLEXORS.
        Straight is always 0.0, so the flexion fraction t in [0,1] maps to q = t * limit
        and hyperextension is unreachable by construction.
        """
        m = self.model
        A, _ = self.muscle_affine(np.zeros(m.nq))  # moment arms at the straight hand
        axes = {}
        for f, joints in FLEXION_JOINTS.items():
            aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, FLEXORS[f])
            chain = []
            for j in joints:
                jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
                if jid < 0:
                    raise ValueError(f"joint {j} not in model")
                dof = int(m.jnt_dofadr[jid])
                lo, hi = m.jnt_range[jid]
                arm = float(A[dof, aid])
                if abs(arm) < 1e-6:
                    raise ValueError(f"{FLEXORS[f]} has no moment arm on {j}")
                limit = float(hi) if arm > 0 else float(lo)  # the flexion-side limit
                chain.append((int(m.jnt_qposadr[jid]), dof, limit))
            axes[f] = chain
        return axes

    def flexion_span(self, joint: str) -> tuple[int, float, float]:
        """(qpos address, straight, full-flexion) for a flexion joint.

        Straight is 0.0; full flexion is the SIGNED limit on the flexor's side. This is
        what keeps hyperextension out of every posture sweep -- and it must be derived
        from the flexor, not from the sign of the joint limits, because the thumb flexes
        POSITIVE at the CMC and NEGATIVE at the MP and IP.
        """
        for f, joints in FLEXION_JOINTS.items():
            if joint in joints:
                qadr, _, limit = self.flexion_axes[f][joints.index(joint)]
                return qadr, 0.0, limit
        raise ValueError(f"{joint} is not a flexion joint")

    def flexor_pull(self, q: np.ndarray, finger: str) -> tuple[np.ndarray, np.ndarray]:
        """What the digit's long flexor does, at posture q.

        Returns (joint-space direction it drives the digit's dofs, unit pad force it
        produces there).

        This replaces every hand-picked flexion synergy, and it is the right primitive:
        the flexor's moment-arm vector over the digit's dofs IS the direction it pulls
        them, signs included, so the flexion-sign mess resolves itself. It also uses ALL
        the digit's dofs -- crucially the thumb's `cmc_abduction`, which is most of how a
        thumb actually presses, and which a flexion-only synergy leaves out (that omission
        is what put the thumb's press direction 66 deg off its pad and gave it zero travel
        on every key).

        The pad force follows from statics: the flexor makes joint torque tau = r*F, and a
        pad force f satisfies J^T f = tau, so f = pinv(J^T) tau.
        """
        A, _ = self.muscle_affine(q)
        aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, FLEXORS[finger])
        dofs = self.digit_dofs[finger]
        r = A[dofs, aid]
        r = r / (np.linalg.norm(r) + 1e-12)

        self.fk(q)
        J = self.pad_jacobian(finger)[:, dofs]
        f = np.linalg.pinv(J.T) @ r
        return r, f / (np.linalg.norm(f) + 1e-12)

    def press_dir_flexor(self, q: np.ndarray, finger: str) -> np.ndarray:
        """Unit direction the pad pushes when the digit's flexor fires. The key's switch
        axis should lie along this: force off this axis is wasted as shear."""
        return self.flexor_pull(q, finger)[1]

    def press_dir(self, q: np.ndarray, finger: str, weights=(1.0, 1.0, 2.0 / 3.0)) -> np.ndarray:
        """The unit direction the pad actually PUSHES when the digit flexes.

        A switch registers travel along its own axis, so force the digit cannot apply along
        that axis is wasted as shear. Measured against the pad normal at rest:

            index 17 deg, middle 20, ring 19, little 12   -- a pad-facing key is fine
            thumb 66 deg                                  -- it is NOT

        The thumb's CMC is a saddle joint: flexing it sweeps the thumb ACROSS the palm in
        opposition rather than straight into its own pad, and this synergy uses only its
        three flexion joints, not the adduction it really presses with. It can still press
        a pad-facing key (20 mm of travel at rest) but not squarely. Key ORIENTATION is a
        design variable in the plan for exactly this reason; Stage 4 should use it,
        especially for the thumb.

        (With the flexion signs wrong this read 159 deg -- the thumb pushing backwards out
        of the back of its own pad.)
        """
        self.fk(q)
        J = self.pad_jacobian(finger)
        dq = np.array([w * lim for w, (_, _, lim) in zip(weights, self.flexion_axes[finger])])
        dofs = [dof for _, dof, _ in self.flexion_axes[finger]]
        v = J[:, dofs] @ dq
        return v / (np.linalg.norm(v) + 1e-12)

    def press_travel(
        self, q: np.ndarray, finger: str, key_normal, weights=(1.0, 1.0, 2.0 / 3.0)
    ) -> float:
        """Metres the pad can still advance INTO the key by flexing further, before a
        joint limit stops it.

        THIS IS WHAT MAKES A KEY PRESSABLE, and leaving it out makes the whole effort
        optimisation degenerate. Effort goes to ZERO at full flexion -- the flexors go
        slack, so passive tension vanishes and the press torque is small -- so the
        unconstrained argmin of Sigma a^3 always parks the finger at its flexion limit,
        curled into the palm. That posture is free precisely BECAUSE the finger is doing
        nothing, and a finger at its limit cannot press a key at all: there is no travel
        left. (It showed up as the ring finger sitting fully curled while its neighbours
        stayed relaxed -- an optimum that is both unusable and, given the neighbours,
        physiologically absurd.)

        A key must therefore be scored on effort AND checked for travel. A mechanical
        switch actuates at ~2 mm and bottoms at ~4 mm.
        """
        d = np.asarray(key_normal, float)
        d = -d / np.linalg.norm(d)  # the digit presses along -key_normal

        r, _ = self.flexor_pull(q, finger)  # joint direction the flexor drives
        dofs = self.digit_dofs[finger]
        self.fk(q)
        J = self.pad_jacobian(finger)[:, dofs]

        # how far the joints can run along r before the first one hits a limit
        alpha = np.inf
        for k, dof in enumerate(dofs):
            jid = int(np.where(self.model.jnt_dofadr == dof)[0][0])
            qa = int(self.model.jnt_qposadr[jid])
            lo, hi = self.model.jnt_range[jid]
            if r[k] > 1e-9:
                alpha = min(alpha, (float(hi) - q[qa]) / r[k])
            elif r[k] < -1e-9:
                alpha = min(alpha, (float(lo) - q[qa]) / r[k])
        alpha = max(float(alpha), 0.0)
        if not np.isfinite(alpha):
            return 0.0

        rate = float((J @ r) @ d)  # pad speed into the key per unit of that motion
        return max(rate, 0.0) * alpha

    def travel_along(self, q: np.ndarray, finger: str, d) -> float:
        """Metres the pad can move along ANY unit direction `d` before a joint limit.

        press_travel() answers this only for the direction the FLEXOR happens to drive, so
        it is useless for a 3-position key: `lift` is driven by the extensors and `contort`
        by the interossei, and asking "how far can the flexor push the pad backwards" is not
        a question about either.

        Direction-agnostic and exact: take the joint motion that moves the pad along d
        (the least-norm solution dq = J^+ d), then run it until the first joint hits a limit.
        Since |J dq| = |d| = 1, the distance travelled IS that step length.
        """
        d = np.asarray(d, float)
        d = d / (np.linalg.norm(d) + 1e-12)
        dofs = self.digit_dofs[finger]
        self.fk(q)
        J = self.pad_jacobian(finger)[:, dofs]
        dq = np.linalg.pinv(J) @ d
        if np.linalg.norm(J @ dq - d) > 1e-6:  # the digit cannot move that way at all
            return 0.0

        alpha = np.inf
        for k, dof in enumerate(dofs):
            jid = int(np.where(self.model.jnt_dofadr == dof)[0][0])
            qa = int(self.model.jnt_qposadr[jid])
            lo, hi = self.model.jnt_range[jid]
            if dq[k] > 1e-9:
                alpha = min(alpha, (float(hi) - q[qa]) / dq[k])
            elif dq[k] < -1e-9:
                alpha = min(alpha, (float(lo) - q[qa]) / dq[k])
        return max(float(alpha), 0.0) if np.isfinite(alpha) else 0.0

    def can_press(self, q: np.ndarray, finger: str, key_normal, travel: float = 0.003) -> bool:
        """Is there enough flexion left to actually actuate a switch at this key?"""
        return self.press_travel(q, finger, key_normal) >= travel

    def compose(self, per_digit: dict[str, np.ndarray]) -> np.ndarray:
        """One hand posture assembled from a chosen posture per digit.

        Each finger reaches its own key independently, so the hand pressing its whole key
        set is just each digit taking its own solved pose. Used to draw the hand ON the
        keys rather than at rest.
        """
        q = self.q_neutral.copy()
        for f, qf in per_digit.items():
            adr = self.digit_qadr[f]
            q[adr] = np.asarray(qf, float)[adr]
        return np.clip(q, self.lo, self.hi)

    def _functional_rest(self) -> np.ndarray:
        """The hand's functional rest posture ("position of function").

        NOT the midrange of the joint limits. Midrange looks like a reasonable default
        and is not: it puts the thumb IP joint at -25 deg (hyperextended) and the MP
        slightly extended, a posture from which the thumb cannot generate force. In that
        pose a 0.5 N thumb press saturates APL on its own -- which reads as "the thumb is
        too weak to press a key", a conclusion about the posture masquerading as a
        conclusion about the hand.

        Values are the standard clinical position of function (wrist neutral, fingers
        moderately flexed, thumb OPPOSED to the fingers).

        THE THUMB IS NOT WHAT ITS JOINT NAMES SUGGEST. `cmc_abduction` in this model is
        RADIAL abduction -- sweeping it +30 deg swings the thumb tip from 19 mm to 96 mm
        radially, straight out to the side, while barely changing how far it stands off the
        palm. It is not palmar abduction. Setting it to +30 "for palmar abduction" produced
        a hitchhiker's thumb sticking out sideways, which is both unusable and painful, and
        a test asserting `cmc_abduction > 15 => palmar abduction` cheerfully passed.

        So the thumb rest is set by the criterion that actually DEFINES the position of
        function -- OPPOSITION -- and verified, not asserted: the thumb pulp must face the
        index/middle pulps (cos = +0.92) across a natural gap (44 mm). Pinned by
        test_thumb_rest_is_opposed.
        """
        q = np.zeros(self.model.nq)

        def deg(name, v):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"joint {name} not in model")
            q[self.model.jnt_qposadr[jid]] = np.deg2rad(v)

        def flex(name, degrees):
            """`degrees` of FLEXION, in whichever direction this joint actually flexes."""
            qadr, _, limit = self.flexion_span(name)
            q[qadr] = math.copysign(np.deg2rad(degrees), limit)

        for w in ("pro_sup", "deviation", "flexion"):
            deg(w, 0.0)  # wrist neutral

        # Thumb OPPOSED to the fingers. cmc_abduction is RADIAL abduction here (+30 swings
        # the tip 96 mm out to the side -- a hitchhiker's thumb), so opposition needs it
        # negative. Verified by test_thumb_rest_is_opposed, not asserted.
        deg("cmc_abduction", 10.0)
        flex("cmc_flexion", 12.0)
        flex("mp_flexion", 12.0)
        flex("ip_flexion", 12.0)

        for d in "2345":  # index, middle, ring, little
            deg(f"mcp{d}_abduction", 0.0)
            flex(f"mcp{d}_flexion", 40.0)
            flex(f"pm{d}_flexion", 45.0)  # PIP
            flex(f"md{d}_flexion", 15.0)  # DIP

        return np.clip(q, self.lo, self.hi)  # respect the model's own limits

    def _pad_frame(self, finger: str) -> tuple[int, np.ndarray, np.ndarray]:
        """Locate the finger PULP from the model's own anatomy.

        Returns (body id, pad point in body coords, palmar unit vector in body coords).

        Recipe, all of it read out of the model rather than assumed:
          bone axis  <- the flesh capsule's local z (MuJoCo capsules extend along z)
          palmar     <- (flexor insertion - extensor insertion), with the along-bone
                        component projected out, so it is a pure dorsal->palmar direction
          pad point  <- the model's tip site, pushed out along `palmar` by the flesh
                        radius, i.e. from the bone's tip to the skin of the pulp
        """
        m = self.model
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, PAD_BODIES[finger])

        def site(name: str) -> np.ndarray:
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, name)
            if sid < 0:
                raise ValueError(f"site {name} not in model")
            return m.site_pos[sid].copy()

        caps = [
            g
            for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid])
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE
        ]
        if not caps:
            raise ValueError(f"no flesh capsule on {PAD_BODIES[finger]}")
        g = caps[0]
        Rg = np.zeros(9)
        mujoco.mju_quat2Mat(Rg, m.geom_quat[g])
        bone = Rg.reshape(3, 3)[:, 2]
        r_flesh = float(m.geom_size[g][0])

        flex, ext = PAD_TENDONS[finger]
        v = site(flex) - site(ext)
        v = v - (v @ bone) * bone  # kill the along-bone part: insertions sit at different depths
        palmar = v / np.linalg.norm(v)

        pad = site(TIP_SITES[finger]) + r_flesh * palmar
        return bid, pad, palmar

    # ---- kinematics -------------------------------------------------------

    def fk(self, q: np.ndarray) -> None:
        d = self.data
        d.qpos[:] = q
        d.qvel[:] = 0.0
        mujoco.mj_forward(self.model, d)

    def pad_pose(self, q: np.ndarray, finger: str) -> tuple[np.ndarray, np.ndarray]:
        """Fingertip pulp centre (m) and outward (palmar) pad normal (unit), in world."""
        self.fk(q)
        bid, pad_l, palmar_l = self.pad[finger]
        R = self.data.xmat[bid].reshape(3, 3)
        pos = self.data.xpos[bid] + R @ pad_l
        n = R @ palmar_l
        return pos, n / np.linalg.norm(n)

    def pad_jacobian(self, finger: str) -> np.ndarray:
        """(3, nv) translational Jacobian of the pulp point. Call after fk()."""
        bid, pad_l, _ = self.pad[finger]
        R = self.data.xmat[bid].reshape(3, 3)
        point = self.data.xpos[bid] + R @ pad_l
        jacp = np.zeros((3, self.model.nv))
        mujoco.mj_jac(self.model, self.data, jacp, None, point, bid)
        return jacp

    # ---- muscle redundancy ------------------------------------------------

    def muscle_affine(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Generalised actuator force is exactly affine in activation at fixed posture.

        Muscle force is  F(a) = bias(l) + gain(l,v)*a,  and qfrc_actuator = R^T F, so at
        fixed q (fixed lengths, zero velocity)  qfrc_actuator(a) = qfrc0 + A @ a.

        We build A column-by-column from MuJoCo's own forward pass rather than unpacking
        the sparse `actuator_moment` and re-deriving muscle gains by hand -- that is exact
        by construction and immune to sparse-format and sign-convention mistakes.

        Returns (A, qfrc0) with A of shape (nv, nu).

        Cached on the posture. This costs 40 mj_forward calls (3.4 ms) and press() asked for
        it TWICE at the SAME posture -- once in solve_activations and once in press_travel's
        flexor_pull. A one-entry memo removes half of that for nothing.
        """
        m, d = self.model, self.data
        key = q.tobytes()
        hit = getattr(self, "_affine_cache", None)
        if hit is not None and hit[0] == key:
            return hit[1], hit[2]
        self.fk(q)

        d.act[:] = 0.0
        mujoco.mj_forward(m, d)
        qfrc0 = d.qfrc_actuator.copy()

        A = np.zeros((m.nv, m.nu))
        e = np.zeros(m.nu)
        for i in range(m.nu):
            e[:] = 0.0
            e[i] = 1.0
            d.act[:] = e
            mujoco.mj_forward(m, d)
            A[:, i] = d.qfrc_actuator - qfrc0

        d.act[:] = 0.0
        mujoco.mj_forward(m, d)
        self._affine_cache = (key, A, qfrc0)
        return A, qfrc0

    def solve_activations(
        self, q: np.ndarray, finger: str, press_N: float, key_normal: np.ndarray
    ) -> tuple[np.ndarray, float, float, float, float]:
        """Min sum(a**3) muscle activations holding posture q while pressing a key.

        Sign convention (get this wrong and the flexors/extensors swap roles):
          `key_normal` is the key's OUTWARD surface normal -- it points out of the key
          face, back toward the finger. The pad therefore faces anti-parallel to it.
          The finger presses INTO the key along -key_normal, so by Newton's third law
          the key's reaction ON the fingertip is  +press_N * key_normal.

        Returns (a, effort, residual, feasibility_floor, load_scale).
        """
        m, d = self.model, self.data
        A, qfrc0 = self.muscle_affine(q)

        self.fk(q)
        J = self.pad_jacobian(finger)
        # Reaction of the key on the fingertip, along the key's outward normal.
        tau_ext = J.T @ (press_N * np.asarray(key_normal, float))

        # Static equilibrium (qacc = 0):
        #   qfrc_actuator + qfrc_passive + tau_ext = qfrc_bias
        # with qfrc_actuator(a) = qfrc0 + A @ a.
        tau_req = d.qfrc_bias - d.qfrc_passive - tau_ext - qfrc0

        # Two-phase (lexicographic), because exact equality is generally NOT achievable:
        # with a in [0,1] the muscle set cannot null gravity on the non-pressing fingers
        # exactly, leaving an irreducible residual (~0.4% of ||tau_req||). Pretending that
        # equality is a hard constraint would make every posture "infeasible"; pretending
        # it away with a tunable soft weight is exactly the trap v1 fell into.
        #
        #   Phase 1: find the feasibility floor   r* = min ||A a - tau||, 0 <= a <= 1
        #   Phase 2: min sum(a^3)  s.t.  ||A a - tau|| <= r* + tol
        #
        # Feasibility is then a real, reportable quantity (r* relative to ||tau_req||),
        # and effort is minimised strictly *within* the feasible set.
        # Equilibrium is imposed on the PRESSING DIGIT's dofs only: the wrist is strapped
        # to the device, and the other four digits' gravity is a design-independent
        # constant that would swamp the signal. See __init__.
        dofs = self.digit_dofs[finger]
        Af = A[dofs]
        tau_f = tau_req[dofs]

        ls = lsq_linear(Af, tau_f, bounds=(0.0, 1.0))
        a_ls = np.clip(ls.x, 0.0, 1.0)
        floor = float(np.linalg.norm(Af @ a_ls - tau_f))
        scale = float(np.linalg.norm(tau_f)) + 1e-12

        # Phase 2 must be a HARD equality, not a norm-ball with slack. Any slack lets the
        # solver buy a large effort reduction by paying a little torque error, and the
        # trade shifts with the load -- which makes effort non-monotonic in press force
        # and is precisely the "optimizer buys its way out of a soft constraint" failure
        # that stalled v1. So we pin the demand to the *achievable* torque tau* = A @ a_ls
        # (feasible by construction, since a_ls attains it) and require equality exactly.
        tau_star = Af @ a_ls

        res = minimize(
            lambda a: float(np.sum(a**3)),
            a_ls,
            jac=lambda a: 3.0 * a**2,
            bounds=[(0.0, 1.0)] * self.nu,
            constraints=[
                {"type": "eq", "fun": lambda a: Af @ a - tau_star, "jac": lambda a: Af}
            ],
            method="SLSQP",
            options={"maxiter": 300, "ftol": 1e-10},
        )
        a = np.clip(res.x, 0.0, 1.0)
        residual = float(np.linalg.norm(Af @ a - tau_f))
        # Report the residual relative to the irreducible floor: this, not qacc, is the
        # meaningful measure. (Finger inertias are ~1e-6 kg.m^2, so M^-1 turns a 1e-4 N.m
        # imbalance into a qacc of ~100 -- qacc is uselessly ill-scaled here.)
        return a, float(np.sum(a**3)), residual, floor, scale

    # ---- the inner problem ------------------------------------------------

    def press(
        self,
        finger: str,
        key_pos: np.ndarray,
        key_normal: np.ndarray,
        press_N: float = 0.5,
        w_posture: float = 1e-3,
        q0: np.ndarray | None = None,
    ) -> Posture:
        """Least-effort posture for `finger` to press a key at (key_pos, key_normal).

        Inner problem A of the formulation. Joint limits are HARD (box bounds).
        Reach and pad-orientation are driven to zero and then checked, so an
        unreachable key is reported as infeasible rather than silently costed.

        `key_normal` is the key's OUTWARD normal (points out of the key face toward the
        finger); the pad ends up anti-parallel to it. See solve_activations().
        """
        key_pos = np.asarray(key_pos, float)
        key_normal = np.asarray(key_normal, float)
        key_normal /= np.linalg.norm(key_normal)
        q0 = self.q_neutral.copy() if q0 is None else q0.copy()

        # Stage 1: pose the finger onto the key (reach + pad alignment + stay near neutral).
        # Cheap, smooth, gradient-friendly -- no mesh penetration terms.
        #
        # Solve over the PRESSING DIGIT's dofs only, not all 23. Reaching a key with the
        # index does not move the ring finger, so the other dofs are not free variables --
        # letting them move is both wrong (it invents whole-hand contortions to reach one
        # key) and slow (L-BFGS-B takes numerical gradients, so 23 dofs costs 6x the forward
        # passes of 4). Measured: 85 ms -> 16 ms, and it is the more correct model.
        adr = self.digit_qadr[finger]
        lo_d, hi_d = self.lo[adr], self.hi[adr]

        # LEXICOGRAPHIC, not weighted. Reaching the key is a CONSTRAINT; pad alignment and
        # posture comfort are preferences among the postures that reach it. Rolling them
        # into one weighted sum lets the solver BUY reach error with comfort -- which is
        # exactly the failure that stalled v1, and it happened here too: written in raw
        # units, reach entered as metres SQUARED (1 mm -> 1e-6) against a normalised O(1e-5)
        # comfort penalty, so comfort outweighed reach by ~4e10 and a hand could not reach a
        # key placed at its OWN fingertip (off by 1.5-3.2 mm). That single units mismatch
        # made `reach` unsatisfiable and the whole NSGA-II run infeasible.
        #
        # Same shape as solve_activations: satisfy the hard thing first, optimise within it.
        bnds = list(zip(lo_d, hi_d))  # HARD joint limits
        dofs = self.digit_dofs[finger]  # nq == nv (all hinges), so these pair with `adr`
        bid, pad_l, palmar_l = self.pad[finger]

        def _state(qd):
            """Pad pose AND its Jacobians, from one forward pass."""
            q = q0.copy()
            q[adr] = qd
            self.fk(q)
            R = self.data.xmat[bid].reshape(3, 3)
            pos = self.data.xpos[bid] + R @ pad_l
            n = R @ palmar_l
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jac(self.model, self.data, jacp, jacr, pos, bid)
            return pos, n, jacp[:, dofs], jacr[:, dofs]

        # ANALYTIC GRADIENTS. Both phases used numerical differencing, which costs ~5 forward
        # passes per gradient and made the posture solve 25 of press()'s 32 ms -- while MuJoCo
        # computes the exact Jacobian for free in the same forward pass we already do.
        #   d/dq |x(q) - k|^2  = 2 (x - k)^T Jp
        #   dn/dq_i            = Jr[:,i] x n     (the pad normal rotates with the body)
        def reach2_and_grad(qd):
            pos, _, Jp, _ = _state(qd)
            d = pos - key_pos
            return float(d @ d), 2.0 * (d @ Jp)

        def reach2(qd):
            return reach2_and_grad(qd)[0]

        # Phase 1: reach, and nothing else.
        rA = minimize(reach2_and_grad, q0[adr], jac=True, bounds=bnds,
                      method="L-BFGS-B", options={"maxiter": 200})
        qA = np.clip(rA.x, lo_d, hi_d)
        err_min = float(np.sqrt(max(reach2(qA), 0.0)))

        # Phase 2: among the postures that reach it, take the comfortable, square one.
        cap = max(err_min + 1e-5, 1e-4)  # 0.1 mm slack, not negotiable beyond that
        span = hi_d - lo_d + 1e-9

        def comfort_and_grad(qd):
            _, n, _, Jr = _state(qd)
            e_ang = 1.0 + float(n @ key_normal)  # pad should face into the key
            dq = (qd - self.q_neutral[adr]) / span
            f = e_ang**2 + w_posture * float(dq @ dq)
            dn = np.cross(Jr.T, n)  # (ndof, 3): dn/dq_i = Jr[:,i] x n
            g = 2.0 * e_ang * (dn @ key_normal) + 2.0 * w_posture * dq / span
            return f, g

        rB = minimize(
            comfort_and_grad, qA, jac=True, bounds=bnds, method="SLSQP",
            constraints=[{
                "type": "ineq",
                "fun": lambda qd: cap**2 - reach2(qd),
                "jac": lambda qd: -reach2_and_grad(qd)[1],
            }],
            options={"maxiter": 100, "ftol": 1e-9},
        )
        qB = np.clip(rB.x, lo_d, hi_d)
        qd = qB if (rB.success and np.sqrt(max(reach2(qB), 0.0)) <= cap) else qA

        q = q0.copy()
        q[adr] = qd

        pos, n = self.pad_pose(q, finger)
        pos_err = float(np.linalg.norm(pos - key_pos))
        ang_err = float(np.arccos(np.clip(-(n @ key_normal), -1.0, 1.0)))

        # Stage 2: muscle redundancy at that posture, under the press load.
        a, effort, residual, floor, scale = self.solve_activations(q, finger, press_N, key_normal)

        max_act = float(a.max())
        return Posture(
            q=q,
            a=a,
            effort=effort,
            pos_err=pos_err,
            ang_err=ang_err,
            torque_residual=residual,
            feas_floor=floor,
            load_scale=scale,
            max_act=max_act,
            # Feasibility gates only on what the DESIGN controls: can the finger get to
            # the key (reach + pad orientation), and can it press it without saturating.
            ok=bool(pos_err < 5e-3 and ang_err < np.deg2rad(20.0) and max_act <= 0.95),
        )

    def rest_pads(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        return {f: self.pad_pose(self.q_neutral, f) for f in FINGERS}
