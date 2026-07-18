#!/usr/bin/env bash
# Publish the printable payload as a GitHub Release asset. Keeps the 18 MB STL out of git
# history while giving builders a clean download. Requires the `gh` CLI, authenticated.
#
#   scripts/release.sh v0.1
#
# Assets: the median print file, the two design intermediates a builder needs to re-fit the
# STL to their own hand (so they never have to run the optimiser), and the flexure coupons.
set -euo pipefail
cd "$(dirname "$0")/.."

TAG="${1:?usage: scripts/release.sh <tag>, e.g. v0.1}"

command -v gh >/dev/null || { echo "gh CLI not found -- https://cli.github.com/"; exit 1; }

# Regenerate the median STL if it is missing (needs the shipped design intermediates).
if [ ! -e out/gauntlet.stl ]; then
  echo "out/gauntlet.stl missing -- building it (make stl)..."
  make stl
fi

ASSETS=(out/gauntlet.stl)
for f in out/final_design.pkl out/bone.npz out/coupon_*.stl; do
  [ -e "$f" ] && ASSETS+=("$f")
done

echo "Releasing $TAG with:"; printf '  %s\n' "${ASSETS[@]}"

gh release create "$TAG" "${ASSETS[@]}" \
  --title "ExoKey $TAG — printable structure" \
  --notes "Printable gauntlet (median 185 mm hand) plus the design intermediates
(\`final_design.pkl\`, \`bone.npz\`) so you can re-fit to your own hand with
\`make fit MM=<yours>\` — no optimiser run needed. See BUILD.md and BOM.md.

⚠ Simulation-only research: the structure prints, but there is no firmware or
proven electronics yet. Do not order the BOM in quantity before the stage-1 coupon."
