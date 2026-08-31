"""The shipping knee design, grown at FULL resolution WITH THE KIT AS PAYLOAD.

final.py grows the co-optimised knee against five self-built wells. This is the same fine
grow with the 2026-08-29 carrier seam attached: the structure must now hold the Svalboard
kit's mount deck and carry its mass, not our own wells. It is the verification that the
shipping path (full resolution, nodes free, donning keep-out) still passes the deflection
gate once the load is a purchased payload.

pareto.pkl is not in the repo (gitignored), but final_design.pkl banks the knee's design
vector x, its wired actions and its action_map -- enough to re-evaluate and re-compose the
exact shipping posture without the Pareto front. If final_design.pkl is absent too, fall
back to the neutral posture with all actions wired: a legitimate fine grow, just not the
co-optimised knee.
"""
import os, pickle, time
for v in ("OMP_NUM_THREADS","OPENBLAS_NUM_THREADS","MKL_NUM_THREADS"): os.environ.setdefault(v,"1")
import warnings; warnings.filterwarnings("ignore")
import numpy as np
from design.params import DEFLECTION_MAX
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.lattice import grow
from manufacture.carrier import carrier_from_bracket

H = hands(); ref = H[50]

if os.path.exists("out/final_design.pkl"):
    d = pickle.load(open("out/final_design.pkl", "rb"))
    x = d["x"]
    r = evaluate(x, H)
    wired = {k: set(v) for k, v in d["wired"].items()}
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    curls = r["curls"]
    src = "co-optimised knee (final_design.pkl)"
else:
    x, wired, curls, q = None, None, None, np.zeros(ref.model.nq)
    src = "neutral posture, all actions (no final_design.pkl)"

carrier = carrier_from_bracket(ref, q)
print(f"carrier: deck {carrier.deck.shape[0]} nodes, mass {carrier.mass*1e3:.0f} g, "
      f"{len(carrier.keywells)} keywells   [{src}]")

t0 = time.time()
N, bars, live, btn, cases, ak, an, hist, pc, _sh, _ls = grow(
    ref, q, wired=wired, gate=float(DEFLECTION_MAX), relax=True, curls=curls, reach=3.6,
    carrier=carrier)
print(f"FINE GROW WITH THE KIT AS PAYLOAD, full resolution, nodes free  [{time.time()-t0:.0f}s]")
print(f"  {hist[0][0]} candidates -> {len(live)} struts ({100*(1-len(live)/hist[0][0]):.1f}% deleted)")
print(f"  bone {hist[-1][2]*1000:.1f} g   buttons {hist[-1][1]*1e6:.0f} um (gate 500)   "
      f"strap {hist[-1][3]:.2f} N")
print(f"  worst load case: {max(pc, key=pc.get)}  at {max(pc.values())*1e6:.0f} um")

np.savez("out/final_carrier.npz", nodes=N, bars=np.array(bars), live=np.array(live),
         buttons=np.array([btn[f] for f in FINGERS]), fingers=np.array(FINGERS),
         anchors=np.array(sorted(ak)), bone_g=hist[-1][2]*1000, button_um=hist[-1][1]*1e6,
         strap_N=hist[-1][3], bars0=hist[0][0], mass0=hist[0][2], defl0=hist[0][1],
         deck=carrier.deck, com=carrier.com, carrier_mass=carrier.mass,
         keywells=np.array([carrier.keywells[f] for f in FINGERS]))
print("  wrote out/final_carrier.npz")
