# ExoKey build helpers. Hides the PYTHONPATH=. OMP_NUM_THREADS=1 incantations so a builder
# who just owns a printer never has to type them. See BUILD.md.
#
#   make deps      one-time: venv + pinned deps + the MyoHand submodule
#   make test      the gates (needs deps)
#   make stl       regenerate out/gauntlet.stl from the shipped design (median hand)
#   make fit MM=192   your-hand STL: out/gauntlet_192mm.stl  (measure wrist->fingertip)
#   make optimise  the expensive NSGA-II search that PRODUCES the design (hours; or cloud/hetzner.sh)
#   make release TAG=v0.1   build + attach STL/design to a GitHub Release (needs gh)
#   make clean     remove regenerable out/ artefacts (keeps the committed viewers)

PY  := .venv/bin/python
RUN := PYTHONPATH=. OMP_NUM_THREADS=1 $(PY)
DESIGN := out/final_design.pkl out/bone.npz     # the optimiser's output the mesher needs

.PHONY: deps test stl fit optimise release clean
.DEFAULT_GOAL := help

help:
	@sed -n 's/^#   //p' Makefile

deps:
	python3 -m venv .venv
	$(PY) -m pip install -r requirements.txt
	git submodule update --init --recursive

test: | .venv
	$(RUN) -m pytest -q

# The mesher needs the optimiser's design (out/final_design.pkl + a topology .npz). These ship as
# GitHub Release assets; a fresh clone does NOT have them -- run `make optimise` (or download the
# release) first. Guarded so the failure is a sentence, not a traceback.
stl: | .venv
	@for f in $(DESIGN); do [ -e $$f ] || { \
	  echo "MISSING $$f -- run 'make optimise' (hours) or download the design from the Release."; exit 1; }; done
	$(RUN) scripts/export_stl.py

fit: | .venv
	@[ -n "$(MM)" ] || { echo "usage: make fit MM=<your hand length in mm, e.g. 192>"; exit 1; }
	@for f in $(DESIGN); do [ -e $$f ] || { \
	  echo "MISSING $$f -- run 'make optimise' or download the design from the Release."; exit 1; }; done
	$(RUN) scripts/export_stl.py --hand-mm $(MM)

# The design itself, from scratch: NSGA-II search -> pick the winner -> the printable-bone stages.
# Long (a musculoskeletal hand in the loop). This is a MODEST local run (opt.run defaults, pop 60
# gen 40); cloud/hetzner.sh bursts the big pop-200/gen-150 search onto a box and deletes it.
optimise: | .venv
	$(RUN) -m opt.run $(ARGS)   # -> out/pareto.pkl   (bigger search: make optimise ARGS="--pop 200 --gen 150")
	$(RUN) scripts/final.py     # -> out/final.npz + out/final_design.pkl
	$(RUN) scripts/printable.py # -> out/printable.npz
	$(RUN) scripts/ergonomic.py # -> out/friendly.npz
	$(RUN) scripts/bone.py      # -> out/bone.npz

release: | .venv
	@[ -n "$(TAG)" ] || { echo "usage: make release TAG=v0.1"; exit 1; }
	scripts/release.sh "$(TAG)"

clean:
	rm -f out/*.npz out/*.pkl out/gauntlet*.stl out/coupon_*.stl

.venv:
	@echo "no .venv -- run 'make deps' first"; exit 1
