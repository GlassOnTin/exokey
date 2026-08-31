"""Export full 3D assembly STL (Biomorphic Carrier Gauntlet + Svalboard 5-Way Key Units).

    PYTHONPATH=. .venv/bin/python scripts/export_svalboard_assembly_stl.py
"""
from __future__ import annotations

import os
import pickle
import numpy as np
import trimesh

from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from manufacture.svalboard import build_all_svalboard_units
from manufacture.carrier_gauntlet import build_organic_carrier_gauntlet


def main():
    h = hands()[50]
    if os.path.exists("out/final_design.pkl"):
        d = pickle.load(open("out/final_design.pkl", "rb"))
        x = d["x"]
        q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                       for f in FINGERS})
    else:
        q = np.zeros(h.model.nq)

    # 1. Build the Svalboard units
    units = build_all_svalboard_units(h, q)
    
    # 2. Build the organic exoskeleton carrier gauntlet
    gauntlet_meshes = build_organic_carrier_gauntlet(h, q, units)
    chassis = gauntlet_meshes["chassis"]

    # Export gauntlet chassis alone (in mm)
    chassis_export = chassis.copy()
    chassis_export.apply_scale(1000.0)
    chassis_path = "out/gauntlet_carrier.stl"
    chassis_export.export(chassis_path)
    print(f"wrote {chassis_path} ({os.path.getsize(chassis_path)/1e6:.1f} MB, {len(chassis_export.faces)} faces)")

    # Collect key meshes
    key_meshes = []
    for f, u in units.items():
        if u["pod"] is not None:
            key_meshes.append(u["pod"])
        if u["cradle"] is not None:
            key_meshes.append(u["cradle"])
        for pmesh in u["paddles"].values():
            key_meshes.append(pmesh)

    all_keys = trimesh.util.concatenate(key_meshes)
    
    # Export keys alone (in mm)
    keys_export = all_keys.copy()
    keys_export.apply_scale(1000.0)
    keys_path = "out/svalboard_keys.stl"
    keys_export.export(keys_path)
    print(f"wrote {keys_path} ({os.path.getsize(keys_path)/1e6:.1f} MB, {len(keys_export.faces)} faces)")

    # Combine gauntlet + keys into one complete assembly STL
    assembly = trimesh.util.concatenate([chassis, all_keys])
    assembly.apply_scale(1000.0)
    
    assembly_path = "out/gauntlet_carrier_svalboard_assembly.stl"
    assembly.export(assembly_path)
    print(f"wrote {assembly_path} ({os.path.getsize(assembly_path)/1e6:.1f} MB, {len(assembly.faces)} faces)")


if __name__ == "__main__":
    main()
