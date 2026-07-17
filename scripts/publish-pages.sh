#!/usr/bin/env bash
# Publish the current out/ render gallery to the gh-pages branch -> GitHub Pages at
#   https://glassontin.github.io/exokey/out/
#
# The renders are GENERATED ARTEFACTS. Most are kept OFF master (they are large and regenerable);
# they live on an orphan gh-pages branch so a normal clone of master stays lean. This is the one
# command that syncs the local out/ gallery onto that branch and pushes it. Pages rebuilds itself.
#
#   1. regenerate whatever changed:   PYTHONPATH=. .venv/bin/python scripts/<x>_view.py
#   2. publish:                        scripts/publish-pages.sh
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

# the gallery = every viewer page + the data the fetch-based ones load. The optimisation renders
# (anim/progress/pareto) were dropped: they can only be refreshed by re-running the whole NSGA-II
# search, so they went stale and confused the gallery -- the design is shown by the live renders.
pages=(index onstrap strap typing impact entry harness)
data=(typing.json)

WT="$(mktemp -d)"
cleanup() { git worktree remove --force "$WT" 2>/dev/null || true; }
trap cleanup EXIT

git worktree add -q "$WT" gh-pages
for p in "${pages[@]}"; do [ -f "out/$p.html" ] && cp "out/$p.html" "$WT/out/"; done
for d in "${data[@]}";  do [ -f "out/$d" ]      && cp "out/$d"      "$WT/out/"; done

# prune any hosted page that is no longer in the gallery (e.g. a superseded render)
for f in "$WT"/out/*.html; do
  b="$(basename "$f" .html)"
  [[ " ${pages[*]} " == *" $b "* ]] || rm -f "$f"
done

git -C "$WT" add -A
if git -C "$WT" diff --cached --quiet; then
  echo "gh-pages already up to date — nothing to publish."
else
  git -C "$WT" commit -q -m "gh-pages: sync render gallery"
  git -C "$WT" push -q origin gh-pages
  echo "published -> https://glassontin.github.io/exokey/out/ (Pages rebuilds in ~1 min)"
fi
