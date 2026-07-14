# Timestamp — proof of prior-art publication date

This repository contains a **defensive publication** ([VISION.md § 8](VISION.md#8-disclosed-variants-defensive-publication)).
Prior art is only worth anything if its **date is provable to a third party**.

**Git commit dates are worthless for this.** They are set by the committer, trivially forged,
and rewritten by any rebase. So the disclosure is anchored independently.

## What is anchored

The disclosure has been **extended and re-anchored** twice. **All three stamps stand**, and each one
proves what was disclosed *at that moment*. An earlier proof is not invalidated by a later one — it
is a *floor* on the date, and floors do not move.

### Current — MANUFACTURABILITY as a constraint on the design space (§8.15, §8.15b)

Adds the printability method and its measured results: the minimum-feature bound imposed by
**delete-then-fatten-then-re-size**; **support-reachability** as a topological constraint on the
member graph; the **bridge/self-support distinction**; that **sacrificial support is exempt from the
wearer-clearance constraint, because the wearer is not in the printer**; the **build direction as a
design variable minimised on support VOLUME, not support COUNT**; and the finding that a shell which
conforms to a curved limb **cannot be FDM-printed support-free in any orientation**.

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `b137ecc9036552239ae6bf794217e62aa0a208e457e2cb0054c4727508808159` |
| `MANIFEST.sha256` (hashes of all 57 source + doc files) | `9a3dd5f44928abfdb28cc7ddc2481a5daf6bf7200d2d112ba14dbf5e3f61a5d2` |

Stamped: **2026-07-14T16:50:24Z** (UTC, submission time). Proofs: `VISION.md.ots`,
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
