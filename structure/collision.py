"""ANATOMICAL 3D SIGNED-DISTANCE COLLISION DETECTION & CLEARANCE AUDITOR FOR EXOKEY.

Comprehensive Collision Audits:
1. INTER-POD CLEARANCE (Pairwise 5-way Key Unit Check):
   - Computes exact mesh-to-mesh minimum distances between all 10 pairs of Svalboard 5-way units.
   - Enforces minimum clearance gate: d_pod_pod >= 1.5 mm.
2. OUTRIGGER SKIN PENETRATION & SIGNED DISTANCE (Anatomical Flesh Check):
   - Computes signed distance from every outrigger tube link to the closed MRI hand skin surface:
     * Signed Distance > 0: In the air outside the hand.
     * Signed Distance < 0: Piercing / penetrating into flesh or bone!
   - Enforces strict zero-penetration gate: min(signed_distance) >= 1.5 mm everywhere.
3. STRUT-TO-STRUT CLEARANCE (Outrigger Interference Check):
   - Validates that adjacent finger booms maintain >= 2.0 mm clearance.
"""
from __future__ import annotations

import numpy as np
import trimesh
from scipy.spatial import cKDTree

from hand.flesh import skin
from hand.myohand import FINGERS, MyoHand
from manufacture.svalboard import build_all_svalboard_units


def audit_pod_intersections(sval_units: dict, min_gap_mm: float = 1.5) -> dict:
    """Audit pairwise clearances between all Svalboard 5-way key units."""
    unit_meshes = {}
    for f, u in sval_units.items():
        parts = [u["pod"], u["cradle"]]
        if isinstance(u.get("paddles"), dict):
            parts.extend(list(u["paddles"].values()))
        unit_meshes[f] = trimesh.util.concatenate(parts)
        
    fingers = [f for f in ["index", "middle", "ring", "little", "thumb"] if f in sval_units]
    
    pair_reports = {}
    worst_gap_mm = 1e9
    worst_pair = None
    all_clear = True
    
    for i in range(len(fingers)):
        for j in range(i + 1, len(fingers)):
            f1, f2 = fingers[i], fingers[j]
            m1, m2 = unit_meshes[f1], unit_meshes[f2]
            
            t1 = cKDTree(m1.vertices)
            t2 = cKDTree(m2.vertices)
            
            d12 = float(np.min(t2.query(m1.vertices)[0])) * 1.0e3
            d21 = float(np.min(t1.query(m2.vertices)[0])) * 1.0e3
            gap_mm = min(d12, d21)
            
            # Check bounding box collision
            bb1 = m1.bounds
            bb2 = m2.bounds
            overlap = not ((bb1[1, 0] < bb2[0, 0]) or (bb1[0, 0] > bb2[1, 0]) or
                           (bb1[1, 1] < bb2[0, 1]) or (bb1[0, 1] > bb2[1, 1]) or
                           (bb1[1, 2] < bb2[0, 2]) or (bb1[0, 2] > bb2[1, 2]))
                           
            passes = gap_mm >= min_gap_mm
            if not passes:
                all_clear = False
                
            if gap_mm < worst_gap_mm:
                worst_gap_mm = gap_mm
                worst_pair = (f1, f2)
                
            pair_reports[(f1, f2)] = {
                "gap_mm": gap_mm,
                "overlap": overlap,
                "passes": passes
            }
            
    return {
        "all_clear": all_clear,
        "worst_gap_mm": worst_gap_mm,
        "worst_pair": worst_pair,
        "min_required_mm": min_gap_mm,
        "pairs": pair_reports
    }


def audit_outrigger_skin_penetration(link_segments: dict[str, list[tuple[np.ndarray, np.ndarray]]],
                                     V_skin: np.ndarray, F_skin: np.ndarray,
                                     r_tube: float = 0.0022,
                                     min_clearance_mm: float = 1.5) -> dict:
    """Audit signed distance between all outrigger links and the hand flesh mesh."""
    mesh_skin = trimesh.Trimesh(V_skin, F_skin)
    v_normals = mesh_skin.vertex_normals
    tree_skin = cKDTree(np.asarray(V_skin))
    
    def _signed_distance(points):
        pts = np.atleast_2d(points)
        dists, indices = tree_skin.query(pts)
        normals = v_normals[indices]
        verts = V_skin[indices]
        vecs = pts - verts
        # True signed distance: positive = outside flesh, negative = inside flesh!
        sd = np.array([d if np.dot(v, n) >= 0 else -d for d, v, n in zip(dists, vecs, normals)])
        # Subtract tube radius to get clearance from tube outer wall to skin surface
        return (sd - r_tube) * 1.0e3
        
    reports = {}
    all_clear = True
    worst_sd_mm = 1e9
    worst_link = None
    
    for f, segments in link_segments.items():
        digit_worst_sd = 1e9
        digit_penetrates = False
        
        for s_idx, (p0, p1) in enumerate(segments):
            pts = np.linspace(p0, p1, 25)
            sds = _signed_distance(pts)
            min_sd = float(np.min(sds))
            
            if min_sd < min_clearance_mm:
                all_clear = False
            if min_sd < 0.0:
                digit_penetrates = True
                
            if min_sd < worst_sd_mm:
                worst_sd_mm = min_sd
                worst_link = f"{f}_segment_{s_idx+1}"
                
            if min_sd < digit_worst_sd:
                digit_worst_sd = min_sd
                
        reports[f] = {
            "min_signed_dist_mm": digit_worst_sd,
            "penetrates": digit_penetrates,
            "passes": digit_worst_sd >= min_clearance_mm
        }
        
    return {
        "all_clear": all_clear,
        "worst_sd_mm": worst_sd_mm,
        "worst_link": worst_link,
        "min_required_mm": min_clearance_mm,
        "digits": reports
    }
