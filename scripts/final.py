"""The knee design of the co-optimised front, grown at FULL resolution with the nodes free."""
import os, pickle, time
for v in ("OMP_NUM_THREADS","OPENBLAS_NUM_THREADS","MKL_NUM_THREADS"): os.environ.setdefault(v,"1")
import warnings; warnings.filterwarnings("ignore")
import numpy as np
from design.params import DEFLECTION_MAX
from design.qwerty import used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.lattice import grow
H=hands(); ref=H[50]
d=pickle.load(open("out/pareto.pkl","rb")); X=d["X"]; F=np.atleast_2d(d["F"])
Fn=(F-F.min(0))/(F.max(0)-F.min(0)+1e-12); i=int(np.argmin((Fn**2).sum(1))); x=X[i]
r=evaluate(x,H); wired=used_actions(r["action_map"])
q=ref.compose({f: posture(ref,f,tp_of(x,f),tm_of(x,f),float(x.get(f"ab_{f}",0.0))) for f in FINGERS})
t0=time.time()
# ⚠ REACH 3.6, NOT THE DEFAULT 2.2, AND THE DONNING KEEP-OUT IS WHY. The keep-out reserves a
# ~3.5 mm corridor for the entering finger; at this 4 mm pitch that moat is as wide as the node
# spacing, and bars of 2.2*pitch = 8.8 mm cannot route around it -- the lattice is severed. It
# fails LOUDLY but confusingly: the SOLID lattice (13141 bars, every candidate present) deflected
# 8084 um against a 500 um gate, so ESO deleted nothing and returned 243 g of bone, while the
# COARSER 8 mm grow the GA uses was fine at 48 um. More material, 170x floppier -- the giveaway
# that it was severance, not stiffness. Measured at 4 mm: reach 2.2 -> 8084 um (broken),
# 3.0 -> 116 um, 3.6 -> 48 um, matching the coarse solid exactly.
N,bars,live,btn,cases,ak,an,hist,pc,_sh,_ls = grow(ref,q,wired=wired,gate=float(DEFLECTION_MAX),
                                                  relax=True,curls=r["curls"],reach=3.6)
print(f"KNEE OF THE CO-OPTIMISED FRONT, full resolution, nodes free  [{time.time()-t0:.0f}s]")
print(f"  {hist[0][0]} candidates -> {len(live)} struts ({100*(1-len(live)/hist[0][0]):.1f}% deleted)")
print(f"  bone {hist[-1][2]*1000:.1f} g   buttons {hist[-1][1]*1e6:.0f} um (gate 500)   "
      f"strap {hist[-1][3]:.2f} N")
print(f"  worst load case: {max(pc, key=pc.get)}  at {max(pc.values())*1e6:.0f} um")
np.savez("out/final.npz", nodes=N, bars=np.array(bars), live=np.array(live),
         buttons=np.array([btn[f] for f in FINGERS]), fingers=np.array(FINGERS),
         anchors=np.array(sorted(ak)), bone_g=hist[-1][2]*1000, button_um=hist[-1][1]*1e6,
         strap_N=hist[-1][3], bars0=hist[0][0], mass0=hist[0][2], defl0=hist[0][1],
         effort=r["F"][0], design_index=i)
# THE ENTRY ANIMATION'S DATA, banked with the result. The same interpolation the entry-route
# constraint is judged on, so a render of it shows what the optimiser actually promised.
from manufacture.entry import donning_frames
_fr = donning_frames(ref, r["curls"])
np.savez("out/donning.npz", q=np.array([f[0] for f in _fr]),
         offset=np.array([f[1] for f in _fr]), fingers=np.array(FINGERS))
print(f"  wrote out/donning.npz  ({len(_fr)} entry frames, for the donning animation)")
pickle.dump(dict(x=dict(x), wired={k:sorted(v) for k,v in wired.items()},
                 action_map=r["action_map"]), open("out/final_design.pkl","wb"))
