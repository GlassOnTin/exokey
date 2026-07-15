# Timestamp — proof of prior-art publication date

This repository contains a **defensive publication** ([VISION.md § 8](VISION.md#8-disclosed-variants-defensive-publication)).
Prior art is only worth anything if its **date is provable to a third party**.

**Git commit dates are worthless for this.** They are set by the committer, trivially forged,
and rewritten by any rebase. So the disclosure is anchored independently.

## What is anchored

The disclosure has been **extended and re-anchored** seven times. **All eight stamps stand**, and each
one proves what was disclosed *at that moment*. An earlier proof is not invalidated by a later one —
it is a *floor* on the date, and floors do not move.

### Current — the IMPACT STRUCTURE, SHAPE-CONVERGED (§8.15k, revised)

The impact re-optimisation, taken to convergence in *shape*, not only topology. The co-sized skeleton
came off the lattice **staircased** — ~8% of its nodes turned a load path past 75° — and never got the
**form-finding** pass every reported structure gets (grow runs it during the search, but the dense
impact grow *starved* it). Adding `relax_nodes` after the sizing straightens it (**kinks > 75°:
40 → 11**) and, because a starved dense lattice had members sized thick to resist *bending* at their
kinks, lets them carry *axially* instead: **29.3 g → 24.2 g**. So in-the-loop is now **34% lighter**
than the bolt-on (was 19%), and no longer jagged — the mass was hiding in the un-converged shape.

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `4f47c62663c1ba702823088afec7c7c8eb2132a387af54e230daa0fa70c58a7a` |
| `MANIFEST.sha256` (hashes of all 86 source + doc files) | `5d233591b2dde6ac48746c70fb97b96dac2239a40d4074c3ae6427af0b967d54` |

Stamped: **2026-07-15T19:23:27Z** (UTC, submission time). Proofs: `VISION.md.ots`,
`MANIFEST.sha256.ots`.

⚠ `TIMESTAMP.md` is deliberately **not** in the manifest. It is written *after* the stamp — it holds
the stamp's own hashes and time — so including it would guarantee `sha256sum -c` failed forever.

### Seventh — the KNOCK RE-SIZES THE BONE (§8.15k, as first disclosed)

Impact is the binding structural load, not the keypress. A 50 N knock breaks the deflection-optimised
bone (**348 MPa** against a 70 MPa yield), while fatigue has a 16× margin. And the knock wants a
*different* skeleton — broad and load-sharing, not the sparse keypress one thickened: grown with the
knock in the load set, the two topologies share only **20% of their members** (Jaccard 0.20). Growing
WITH the knock and co-sizing for the gate AND the stress is **19% lighter** than bolting the impact on
afterward (before the shape-convergence pass above took it to 34%).

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-15, 17:57Z) | `037c9c00e9f3bd4f68e1d06ee1fa05405a1fb20e6fafd45f1a98e5fa1872a215` |
| `MANIFEST.sha256` (84 files) | `391d83a5a57ad04690d6f63e44d1c59ab7ca95e8ad9d24b308ae2d0240db1507` |

Stamped: **2026-07-15T17:57:47Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-15d.ots`, `timestamps/MANIFEST.sha256.2026-07-15d.ots`.

### Sixth — the GAUNTLET ON THE OUTSIDE OF THE STRAP (§8.15j)

The design decision that the *strap*, not the gauntlet, is what meets the hand: the gauntlet mounts
on the OUTER face of the soft TPU strap, so the strap is the sole hand interface — cushion, tension
tether, and load-spreader in one part, attached by loops printed into the strap itself. Re-solved:
the 500 µm gate holds (**499 µm**, +0%) with the soft strap in the load path, because TPU is stiffer
in through-thickness compression than the tissue it sits on. This **supersedes the inner bearing
shell (§8.15i)** as the skin interface.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-15, 14:31Z) | `d92e96496c21acec7568bbbfe53db0aa1fbba4c6caac4fa556554b4f41d0a7b5` |
| `MANIFEST.sha256` (80 files) | `4775e3d98f00977352b0d077cd85b0d867876eff7e164f1db57c29ee071cc0de` |

Stamped: **2026-07-15T14:31:37Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-15c.ots`, `timestamps/MANIFEST.sha256.2026-07-15c.ots`.

### Fifth — the SANDWICH GATE RE-SOLVE (§8.15i)

The sandwich inner face added to the per-element solver, and the 500 µm key-deflection gate re-solved
at the bone's real sections: the buttons hold at **485 µm**, so the face does not compromise
key-crispness (its value is the IMPACT, not the gate).

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-15, 13:43Z) | `772124bf2730861e27aa572b66adb057230a8b4362403353904642b1b2bfec0d` |
| `MANIFEST.sha256` (77 files) | `ce4971b13227485c8944c995465a29696cf6361222e5af854892c1a29197bdd2` |

Stamped: **2026-07-15T13:43:52Z** (UTC). Proofs: `timestamps/VISION.md.2026-07-15b.ots`,
`timestamps/MANIFEST.sha256.2026-07-15b.ots`.

### Fourth — the SENSOR, the STRAP ANCHOR, and the BEARING SHELL (§8.15g–i)

Adds the wearable's two practical subsystems and its skin interface: the **contactless-Hall finger
well** — a magnet on a printed **TPU dome** over a 3-axis Hall — with the flexure material chosen by
**σ_fatigue/E** (the maximum recoverable bending strain) and the plunge that must *bend*, not
compress; the measured result that **every well is five-way** (the ulnar "three-way" limit was a
cradle artefact — the interossei are adequate, and the extensor hood a genuine but *non-operative*
MyoHand gap); the **strap anchor** — the band routed as the convex hull of (skin ∪ device) so it
rides *over* the structure it holds down, a **watch-lug** capturing a pin in shear, one adjustable
strap fitting the 5th–95th percentile hand; and the **inner bearing shell** as an **impact
distributor** — a plate on the soft-tissue elastic foundation, sized by the *knock* not the preload —
built as a **sandwich** with the topology-optimised lattice as its core.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-15, 09:32Z) | `4d663f98528165d61fa3abbe4327db7dc7e64fb934ba447619bb626d85a6b9ad` |
| `MANIFEST.sha256` (75 files) | `3ebcb9b285d300494a746acddc0d23f233083ec204a69b7e2be079b5222d0138` |

Stamped: **2026-07-15T09:32:00Z** (UTC). Proofs: `timestamps/VISION.md.2026-07-15a.ots`,
`timestamps/MANIFEST.sha256.2026-07-15a.ots`.

### Third — HUMAN FACTORS as the organising principle (§5g), and the whole structural stack

Adds: **human factors as the organising principle** (§5g) — nearly every constraint here is a fact
about PEOPLE, only three are facts about a machine, and reproducibility ("one person, one printer")
is a HUMANIST constraint; the **ergonomic floor** `SKIN_R` and the finding that it, not the nozzle,
is what makes a topology-optimised structure **trabecular**; **curved (spline) load paths**;
**oriented elliptical and stadium sections** and the proof that a circle is the worst section for a
member that bends; and the central measured result — **the device is TOUCH-limited, not
load-limited** (95% of its members are as thick as they are because a HAND must bear them), so **the
bone is HOLLOW**.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-14, 21:40Z) | `15d99f392e9fe34fdec8908cb602bd49e32349c0667c398d005790318296866b` |
| `MANIFEST.sha256` (63 files) | `f0e4fb4db6eb348e8e760464a40b1d58e5eefab1abe560ad1b2a0a3388335d91` |

Stamped: **2026-07-14T21:40:41Z** (UTC). Proofs: `timestamps/VISION.md.2026-07-14b.ots`,
`timestamps/MANIFEST.sha256.2026-07-14b.ots`.

### Second — the dorsal gauntlet, the structure, the anchor, the manufacture (§8.8–8.14)

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-14, 15:24Z) | `a327fa03b832e334dff709dabfccf7fb8dc01ca760da70f380d64b1930cebb14` |
| `MANIFEST.sha256` (53 files) | `e16f938ae7641e1758f625408e21ca4b6269a8b2893b2205da22a168bf0ebf4b` |

Stamped: **2026-07-14T15:24:50Z**. Proofs: `timestamps/VISION.md.2026-07-14a.ots`,
`timestamps/MANIFEST.sha256.2026-07-14a.ots`.

### Original — the palmar body and the layout method (§8.1–8.7)

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-12) | `a1d7c32e743780be7fee98dccf2ef727d4ea26fda8d2b970862b7357f91232be` |
| `MANIFEST.sha256` (27 files) | `4c45f8cdd21e1f5b48e0ad9852ad195cf5c4a07d89d1b46ba3262ef52367c1e4` |

Stamped: **2026-07-12T22:50:22Z**. Proofs: `timestamps/VISION.md.2026-07-12.ots`,
`timestamps/MANIFEST.sha256.2026-07-12.ots`.

⚠ The original proofs cover the *original* file contents. To verify them you need that version of
`VISION.md` — `git show <commit>:VISION.md`. This is why the manifest is hashed separately: the
manifest pins the whole tree at that instant.

## How the proof works

[OpenTimestamps](https://opentimestamps.org/) aggregates the hash into a Merkle tree and
commits the root into the **Bitcoin blockchain**. Once a block confirms it, the proof shows
the file existed *before that block was mined* — a fact anchored in the most expensive
public ledger in existence, verifiable by anyone, forever, with no trusted third party.

The attestation matures in a few hours (it needs a Bitcoin block). Until then `ots verify`
reports a *pending* attestation from the calendar servers; afterwards it reports a
**Bitcoin block height and time**.

## Verify it yourself

```bash
pip install opentimestamps-client

ots verify VISION.md.ots          # -> "Success! Bitcoin block <N> attests existence as of <date>"
ots verify MANIFEST.sha256.ots

# and check the manifest still matches the tree it covers
sha256sum -c MANIFEST.sha256
```

If `ots verify` reports "pending", upgrade the proof once the block is mined:

```bash
ots upgrade VISION.md.ots MANIFEST.sha256.ots
```

## Belt and braces

The Bitcoin anchor is the strong one, but redundancy is cheap:

- **Zenodo DOI** — archival, independently timestamped, and the venue patent examiners and
  courts actually accept. See `CITATION.cff`.
- **Internet Archive** — snapshot the public repository URL.
- **IP.com / Linux Defenders** — purpose-built defensive-publication venues that examiners
  search.

⚠ Not legal advice.
