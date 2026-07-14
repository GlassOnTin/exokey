# Timestamp — proof of prior-art publication date

This repository contains a **defensive publication** ([VISION.md § 8](VISION.md#8-disclosed-variants-defensive-publication)).
Prior art is only worth anything if its **date is provable to a third party**.

**Git commit dates are worthless for this.** They are set by the committer, trivially forged,
and rewritten by any rebase. So the disclosure is anchored independently.

## What is anchored

The disclosure has been **extended and re-anchored** twice. **All three stamps stand**, and each one
proves what was disclosed *at that moment*. An earlier proof is not invalidated by a later one — it
is a *floor* on the date, and floors do not move.

### Current — HUMAN FACTORS as the organising principle (§5g), and the whole structural stack

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
| `VISION.md` (the disclosure) | `15d99f392e9fe34fdec8908cb602bd49e32349c0667c398d005790318296866b` |
| `MANIFEST.sha256` (hashes of all 63 source + doc files) | `f0e4fb4db6eb348e8e760464a40b1d58e5eefab1abe560ad1b2a0a3388335d91` |

Stamped: **2026-07-14T21:40:41Z** (UTC, submission time). Proofs: `VISION.md.ots`,
`MANIFEST.sha256.ots`.

⚠ `TIMESTAMP.md` is deliberately **not** in the manifest. It is written *after* the stamp — it holds
the stamp's own hashes and time — so including it would guarantee `sha256sum -c` failed forever.

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
