#!/usr/bin/env bash
# Regenerate MANIFEST.sha256 -- the hash set the Bitcoin-anchored defensive publication rests on.
# Until now this was a manual/undocumented step; this reproduces it exactly.
#
#   scripts/manifest.sh           regenerate + verify MANIFEST.sha256
#   scripts/manifest.sh --stamp   also run `ots stamp` (needs the OpenTimestamps client)
#
# Selection rule (reverse-engineered to reproduce the existing manifest byte-for-byte): every
# tracked .py/.sh/.cff/.md file EXCEPT TIMESTAMP.md (the ledger that records this hash -- it
# cannot contain itself) and scripts/clean_history.py (a history-rewriting utility, not the work).
set -euo pipefail
cd "$(dirname "$0")/.."

git ls-files -- '*.py' '*.sh' '*.cff' '*.md' \
  | grep -vx 'TIMESTAMP.md' \
  | grep -vx 'scripts/clean_history.py' \
  | sed 's|^|./|' | sort \
  | while IFS= read -r f; do sha256sum "$f"; done > MANIFEST.sha256
  # plain `sort` = the owner's UTF-8 collation, which is how the original manifest was ordered
  # (README/VISION sort among the lowercase paths, not before them as LC_ALL=C would put them).

echo "wrote MANIFEST.sha256 ($(wc -l < MANIFEST.sha256) files)"
sha256sum -c MANIFEST.sha256 >/dev/null && echo "verified: all hashes match working tree"

if [ "${1:-}" = "--stamp" ]; then
  command -v ots >/dev/null || { echo "ots (OpenTimestamps client) not found -- pip install opentimestamps-client"; exit 1; }
  ots stamp MANIFEST.sha256
  echo "stamped MANIFEST.sha256.ots"
fi

cat <<'NEXT'

To re-anchor the disclosure (a deliberate provenance act):
  1. scripts/manifest.sh --stamp            # or: ots stamp MANIFEST.sha256 VISION.md
  2. move the new .ots into timestamps/ with today's date suffix
  3. add the next anchor to TIMESTAMP.md (number, UTC date, the two sha256 values)
  4. later: ots upgrade timestamps/*.ots     # once the Bitcoin attestation confirms
NEXT
