"""QWERTY on a 3-position Hall key. The layout the device actually has to type.

ONE key per finger, sensing THREE finger actions -- push (flexion), lift (extension),
contort (ab/adduction). Verified against the muscles: each recruits a different group
(flexors / extensors / interossei), so the effort model scores all three natively.

    push    FDP, FDS, FPL
    lift    EDC, EIP, EDM, EPL
    contort RI, UI, LU (+ extensor co-contraction)

WHY THIS AND NOT THE TWIN KEY. The twin key (two switches on one stem) is Typeware's, and
patented. A 3-position Hall key gets the third state from SENSING A DIFFERENT MUSCLE GROUP
rather than from more hardware -- so it needs no second stem, no second row to pack, and
the key-overlap problem that dominated the previous model simply evaporates (5 keys, one
per finger). It is a different mechanism, not a workaround.

THE ASYMMETRY IS THE WHOLE POINT, and it is large:

    finger   push       lift       contort
    middle   1.6e-07    4.0e-05    2.5e-05      <- push is 250x cheaper than lift
    ring     4.1e-07    1.6e-04    2.0e-03
    index    2.9e-06    1.3e-03    7.1e-06

Push is 10-1000x cheaper than lift or contort. So WHICH HALL STATE MEANS WHICH QWERTY ROW
is a free choice -- it is firmware, not geometry -- and it should put the cheap action on
the frequent row. QWERTY's own row frequencies are wildly uneven, so this is worth real
effort:

    top    (QWERT)  30.3%   <- the MOST used row
    home   (ASDFG)  23.0%
    bottom (ZXCVB)   5.5%

Note what that means: 'home row' is a fiction inherited from a mechanical typewriter. On
this device the top row is used most, so the cheapest action belongs there.
"""
from __future__ import annotations

import itertools

# THE FINGERTIP SITS IN A U-SHAPED CAVITY ON A MINIATURE JOYSTICK (user's design).
#
# A console thumbstick gives up/down, left/right AND a click -- so FIVE inputs per finger,
# not three. In the pad's own frame that is one axis along the pad NORMAL (press into the
# cavity floor = the click) and two axes in the TANGENT PLANE (tilt the stick):
#
#   click    +n            press into the floor          (flexors -- the cheap one)
#   forward  +t_long       slide the pad distally        (extensors of the distal joint)
#   back     -t_long       drag the pad proximally       (deep flexor, curling the tip)
#   left     +t_lat        push sideways                 (interossei)
#   right    -t_lat        push the other way            (interossei)
#
# 5 fingers x 5 = 25 inputs, against 15 letters for a QWERTY half-hand -- comfortable, with
# room for modifiers. Every one is just a force direction at the fingertip, so the muscle
# model scores them all natively, and their costs differ by orders of magnitude.
#
# This is why the twin key is not needed at all: the extra states come from SENSING MORE
# MUSCLE GROUPS, not from more hardware. No second stem, no second row to pack, and the
# key-overlap constraint that dominated the previous model evaporates -- there are 5 keys.
ACTIONS = ("click", "forward", "back", "left", "right")
ROWS = ("top", "home", "bottom")

# Left hand, standard touch-typing. The index finger owns TWO columns; the second is
# reached by a lateral hand shift (Typeware detect this with an IMU), so it costs the same
# actions on the same key.
#   little  Q A Z      ring  W S X      middle  E D C
#   index   R F V  (column 1)   T G B  (column 2, via shift)
QWERTY_LEFT = {
    ("little", "top"): "q", ("little", "home"): "a", ("little", "bottom"): "z",
    ("ring", "top"): "w", ("ring", "home"): "s", ("ring", "bottom"): "x",
    ("middle", "top"): "e", ("middle", "home"): "d", ("middle", "bottom"): "c",
    ("index", "top"): "r", ("index", "home"): "f", ("index", "bottom"): "v",
    ("index2", "top"): "t", ("index2", "home"): "g", ("index2", "bottom"): "b",
}
# index2 is the same finger and the same key -- just a shifted hand. Same effort.
FINGER_OF = {"little": "little", "ring": "ring", "middle": "middle",
             "index": "index", "index2": "index"}

# English letter frequency, %. Standard corpus figures.
FREQ = {
    "e": 12.70, "t": 9.06, "a": 8.17, "o": 7.51, "i": 6.97, "n": 6.75, "s": 6.33,
    "h": 6.09, "r": 5.99, "d": 4.25, "l": 4.03, "c": 2.78, "u": 2.76, "m": 2.41,
    "w": 2.36, "f": 2.23, "g": 2.02, "y": 1.97, "p": 1.93, "b": 1.49, "v": 0.98,
    "k": 0.77, "j": 0.15, "x": 0.15, "q": 0.10, "z": 0.07,
}

# The shift a hand must make to reach the index's second column. It is not free -- the
# whole hand translates -- but it is not a finger action either, so it is charged as a flat
# adder rather than modelled. ⚠ NOT VERIFIED: this number is a placeholder.
COLUMN_SHIFT_COST = 5e-6


def row_frequencies() -> dict[str, float]:
    out = {r: 0.0 for r in ROWS}
    for (slot, row), ch in QWERTY_LEFT.items():
        out[row] += FREQ.get(ch, 0.0)
    return out


def best_action_map(effort: dict[tuple[str, str], float]) -> tuple[dict, float]:
    """Choose WHICH 3 of the 5 joystick directions each finger uses, and which QWERTY ROW
    each one means, to minimise the frequency-weighted effort of typing English.

    This is firmware and layout, not geometry, so it is FREE -- and it is worth a great deal,
    because the five directions differ in cost by up to 1.9 MILLION x (measured: middle/click
    1.4e-07, index/left 2.7e-01, which is near muscle saturation and effectively impossible).

    THE KEY FREEDOM: 5 fingers x 5 directions = 25 inputs, and a QWERTY half-hand needs only
    15 letters. So TEN DIRECTIONS CAN SIMPLY BE DISCARDED -- and they should be the ten worst.
    Requiring all five to be usable would drag the whole design down to the cost of the worst
    one.

    Exact, not heuristic: choose 3 of 5 (10 ways) x assign to 3 rows (6 ways) = 60 per
    finger, and the fingers decouple, so each is solved by exhaustion.
    """
    slots: dict[str, list[tuple[str, str]]] = {}
    for (slot, row), ch in QWERTY_LEFT.items():
        slots.setdefault(slot, []).append((row, ch))

    mapping, total = {}, 0.0
    for slot, entries in slots.items():
        f = FINGER_OF[slot]
        extra = COLUMN_SHIFT_COST if slot.endswith("2") else 0.0
        best, best_cost = None, float("inf")
        for chosen in itertools.combinations(ACTIONS, len(ROWS)):
            for perm in itertools.permutations(chosen):
                row_to_action = dict(zip(ROWS, perm))
                c = sum(
                    FREQ.get(ch, 0.0) * (effort[(f, row_to_action[row])] + extra)
                    for row, ch in entries
                )
                if c < best_cost:
                    best, best_cost = row_to_action, c
        mapping[slot] = best
        total += best_cost
    return mapping, total / sum(FREQ.get(ch, 0.0) for ch in QWERTY_LEFT.values())


def used_actions(mapping: dict) -> dict[str, set]:
    """Which directions each finger actually uses. The rest are unwired."""
    out: dict[str, set] = {}
    for slot, m in mapping.items():
        out.setdefault(FINGER_OF[slot], set()).update(m.values())
    return out


def cost_of(mapping: dict, effort: dict[tuple[str, str], float]) -> float:
    """Frequency-weighted effort per character of a GIVEN mapping on a GIVEN hand.

    The mapping is firmware -- one layout for everyone -- so it is chosen once from the
    population and then every hand is charged for that one choice.
    """
    tot = 0.0
    for (slot, row), ch in QWERTY_LEFT.items():
        f = FINGER_OF[slot]
        extra = COLUMN_SHIFT_COST if slot.endswith("2") else 0.0
        tot += FREQ.get(ch, 0.0) * (effort[(f, mapping[slot][row])] + extra)
    return tot / sum(FREQ.get(ch, 0.0) for ch in QWERTY_LEFT.values())
