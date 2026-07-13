"""Which character goes on which finger-direction. Solved exactly, not heuristically.

THE THUMB WAS IDLE. It has five performable directions and it is the CHEAPEST digit on the
hand (hand/thenar.py: 66.8 N pinch, inside the published human band) -- while the LITTLE
FINGER, the weakest digit there is, carried three QWERTY rows. That is backwards, and it was
invisible while the model believed the thumb could not press at all.

AND SPACE WAS NOT MODELLED AT ALL. It is the most frequent keystroke in English -- roughly one
character in five -- it lives on the thumb on every keyboard ever built, and it did not appear
anywhere in the effort objective. A layout scored without it is scored without its single
biggest term.

WHY THIS IS AN EXACT PROBLEM AND NOT A QAP. The original plan deferred character assignment as
a quadratic assignment problem, which is NP-hard -- correct for a CHORDING keyboard, where the
cost of a chord depends on which other keys are in it. This device has ONE key per action and
no chords, so the cost of a character is just its frequency times its slot's effort:

    cost = sum_c  freq(c) * effort(slot(c))

which is a LINEAR assignment problem. `scipy.optimize.linear_sum_assignment` solves it exactly
in milliseconds. The NP-hard framing was inherited from a device we no longer build.

THREE LAYOUTS ARE SCORED, because they are different products, not different tunings:

  QWERTY-STRICT   the letters stay on the fingers QWERTY put them on, and the thumb is idle.
                  This is what was being optimised. It is the baseline.

  QWERTY+THUMB    the letters stay where QWERTY put them; the thumb takes SPACE and SHIFT.
                  Costs the user NOTHING to learn, because nobody's muscle memory says which
                  *direction* means which row -- that mapping is new either way.

  FREE            every character is assigned to its cheapest available slot. The lower bound
                  on what this hand can do -- and the user has to learn a new layout to get it.

⚠ THE MODIFIER FREQUENCIES ARE GUESSES (design/params.py). Space at ~18 keystrokes per 100
letters is a standard figure and the conclusion is not delicate about it; shift is a rougher
estimate. Both are declared, and the sensitivity of the verdict to SPACE_FREQ is reported.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import linear_sum_assignment

from design.qwerty import ACTIONS, FINGER_OF, FREQ, QWERTY_LEFT, ROWS
from hand.myohand import FINGERS

# ⚠ GUESS / LITERATURE. Per 100 letter-keystrokes of English prose.
#
# Space: English averages ~4.5 letters per word, so ~18-20 spaces per 100 letters. This is a
# standard figure. Note what it implies and why it changes the problem: the LEFT hand's 15
# QWERTY letters carry only 58.7 of those 100 letter-units, so SPACE ALONE IS ~23% OF THE
# LEFT HAND'S ENTIRE KEYSTROKE LOAD -- bigger than any letter, bigger than 'e'.
from design.params import SHIFT_FREQ as _SHIFT, SPACE_FREQ as _SPACE

SPACE_FREQ = float(_SPACE)
SHIFT_FREQ = float(_SHIFT)

MODIFIERS = {"space": SPACE_FREQ, "shift": SHIFT_FREQ}


def slots(resid: dict, residual_max: float) -> list[tuple[str, str]]:
    """Every (digit, direction) the hand can ACTUALLY perform. An action the digit cannot do
    is not an expensive slot -- it is not a slot."""
    return [(f, a) for f in FINGERS for a in ACTIONS
            if resid.get((f, a), 1.0) <= residual_max]


def _left_hand_letters() -> dict[str, float]:
    return {ch: FREQ.get(ch, 0.0) for ch in QWERTY_LEFT.values()}


def qwerty_strict(effort: dict, resid: dict, residual_max: float) -> tuple[dict, float]:
    """Letters on the fingers QWERTY put them on; thumb idle. The baseline.

    Each finger still CHOOSES which three of its directions mean which row -- that is firmware
    and free -- but it may not hand a letter to another digit.
    """
    from design.qwerty import best_action_map, cost_of

    avail = {k: v <= residual_max for k, v in resid.items()}
    amap, _ = best_action_map(effort, avail)
    mapping = {}
    for (slot, row), ch in QWERTY_LEFT.items():
        mapping[ch] = (FINGER_OF[slot], amap[slot][row])
    total = sum(FREQ.get(ch, 0.0) * effort[mapping[ch]] for ch in mapping)
    n = sum(_left_hand_letters().values())
    return mapping, total / n


def qwerty_plus_thumb(effort: dict, resid: dict, residual_max: float) -> tuple[dict, float]:
    """QWERTY letters stay put; the THUMB takes space and shift.

    Free to learn: nobody's muscle memory encodes which DIRECTION means which row, so this
    changes nothing the user already knows. It just stops wasting the best digit on the hand.
    """
    mapping, _ = qwerty_strict(effort, resid, residual_max)

    thumb_slots = sorted(
        (a for a in ACTIONS if resid.get(("thumb", a), 1.0) <= residual_max),
        key=lambda a: effort[("thumb", a)],
    )
    if len(thumb_slots) < len(MODIFIERS):
        raise ValueError("the thumb cannot perform enough directions to take the modifiers")
    for (name, _), act in zip(sorted(MODIFIERS.items(), key=lambda kv: -kv[1]), thumb_slots):
        mapping[name] = ("thumb", act)

    total = sum(FREQ.get(ch, MODIFIERS.get(ch, 0.0)) * effort[mapping[ch]] for ch in mapping)
    n = sum(_left_hand_letters().values()) + sum(MODIFIERS.values())
    return mapping, total / n


def free(effort: dict, resid: dict, residual_max: float) -> tuple[dict, float]:
    """Every character to its cheapest available slot. EXACT (Hungarian), not greedy.

    This is the lower bound on what the hand can do, and the price is that the user has to
    learn a layout that is not QWERTY.
    """
    chars = {**_left_hand_letters(), **MODIFIERS}
    sl = slots(resid, residual_max)
    if len(sl) < len(chars):
        raise ValueError(f"{len(sl)} performable slots for {len(chars)} characters")

    names = list(chars)
    C = np.array([[chars[c] * effort[s] for s in sl] for c in names])
    r, c = linear_sum_assignment(C)
    mapping = {names[i]: sl[j] for i, j in zip(r, c)}
    return mapping, float(C[r, c].sum()) / sum(chars.values())
