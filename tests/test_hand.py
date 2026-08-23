

def test_the_whole_hand_is_the_measured_hand():
    """EVERY MEASURED DIMENSION OF THE USER'S HAND, PINNED -- because three optimisation runs
    were spent on a hand that was not theirs.

    The history this guards, in order of discovery:
      1. MyoHand's distal capsules were a slim model, so printed cups came out 12-15 mm across
         and the first gauntlet could not be worn -> FINGERTIP_BREADTH + _fit_fingertips.
      2. Only the PADS were fitted, so the shaft stayed up to 1.77x too thin -- the model's
         fingers were NARROWER THAN THEIR OWN MEASURED BREADTHS, which is impossible, and the
         PIP that jammed the print lives on that shaft -> PIP_BREADTH + _fit_shafts.
      3. Widening flesh did not widen the HAND: the four fingers still spanned 64.9 mm at the
         knuckles (user: 104) because the span is set by the metacarpals, not the flesh. The
         user, holding the print: "too narrow for my hand" -> KNUCKLE_BREADTH + _widen_knuckles.

    Each was invisible to every other test, so this one asserts the measurements directly.
    """
    import mujoco
    import numpy as np

    from hand.flesh import skin
    from hand.myohand import (FINGERTIP_BREADTH, KNUCKLE_BREADTH, PAD_BODIES, PIP_BREADTH,
                              SHAFT_BODIES, MyoHand)
    from structure.frame import hand_axes

    h = MyoHand()
    q = h.q_neutral
    V, _F, L = skin(h, q, labels=True)
    V, L = np.asarray(V), np.asarray(L)

    for f in ("index", "middle", "ring", "little"):
        lat = np.asarray(h.well_frame(q, f)["lateral"], float)
        for bname in SHAFT_BODIES[f]:                      # the shaft: the PIP that has to fit in
            bid = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, bname)
            P = V[L == bid]
            w = float((P @ lat).max() - (P @ lat).min())
            assert w >= float(PIP_BREADTH[f]) * 0.97, (f, bname, w * 1e3,
                                                       float(PIP_BREADTH[f]) * 1e3)
        bid = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, PAD_BODIES[f])
        P = V[L == bid]
        w = float((P @ lat).max() - (P @ lat).min())
        assert w >= float(FINGERTIP_BREADTH[f]) * 0.88, (f, "pad", w * 1e3)

    _o, _ed, e_r, _eo = hand_axes(h, q)
    ids = [mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, b)
           for b in ("proxph2", "proxph3", "proxph4", "proxph5")]
    P = V[np.isin(L, ids)]
    width = float((P @ e_r).max() - (P @ e_r).min())
    assert width >= float(KNUCKLE_BREADTH) * 0.95, (width * 1e3, float(KNUCKLE_BREADTH) * 1e3)
