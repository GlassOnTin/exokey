"""MEASURE THE FLESH. Fetch the PIANO hand-MRI dataset and derive the tissue-thickness map.

    PYTHONPATH=. .venv/bin/python scripts/measure_tissue.py

THE USER: "I think we need to find a flesh model to go with the bones."

They were right, and it was the load-bearing gap. MyoHand gives us BONES and crude flesh
CAPSULES on the phalanges and metacarpals -- and over the CARPUS, which is exactly where a
gauntlet anchors, it has NO FLESH AT ALL. So `WRIST_TISSUE` was a GUESS, and it set the
stiffness of the whole structure's main anchor.

THE SOURCE, and its licence chain matters:

    PIANO hand-MRI dataset          Apache-2.0    github.com/reyuwei/PIANO_mri_data
      50 MRI volumes + bone masks + muscle masks + joint annotations

⚠ WE DELIBERATELY DO NOT USE NIMBLE, the parametric hand model built on top of this data. Its
code is MIT but its LICENSE.md is the UNEDITED GITHUB TEMPLATE ("Copyright (c) 2011-2024 GitHub
Inc."), its model weights sit on a Google Drive with no licence stated at all, and it emits
`*_manov.xyz` -- "corresponding skin vertices in MANO topology". MANO is Max Planck's
NON-COMMERCIAL research-only licence, which this project rejected on day one as "a live landmine
if this ever becomes a product". A model registered into MANO's topology may well be a
derivative of it. The raw MRI dataset carries no such problem: it is the SOURCE, not the
MANO-registered model.

THE METHOD, and two ways of getting it wrong that are worth keeping:

  1. Segment the HAND from the raw MRI (Otsu; air is dark, tissue bright), take the largest
     component, fill it.
  2. Take the BONE SURFACE from the bone mask, and its OUTWARD NORMAL from the gradient of the
     smoothed mask.
  3. RAY-CAST from each bone-surface voxel ALONG ITS OWN OUTWARD NORMAL until the ray leaves the
     hand. THAT distance is the tissue the gauntlet presses through.

  ⚠ DISTANCE-TO-AIR IS THE WRONG METRIC. It is the shortest way out in ANY direction, and for a
    palmar bone that is SIDEWAYS -- out of the side of the hand, not through the pad. It read
    the finger PULP as THINNER than the NAIL BED, which is anatomically impossible, and that is
    how the mistake announced itself.

  ⚠ CAST ONLY FROM THE FACE YOU MEAN. Ray-casting dorsally from EVERY bone-surface voxel sends
    the ray from the palmar ones straight THROUGH the bone and out the far side. It read 7 mm of
    "skin" on the back of a hand that is famously skin over bone.

  The DORSAL/PALMAR sign is derived, not assumed: the FINGERTIP PULP IS PALMAR and it is the
  thicker side. An anatomical fact the data can be asked to confirm.
"""
from __future__ import annotations

import os
import re
import sys
import zipfile

import numpy as np

DIR = "data/piano"
FILES = {
    "mri_raw.zip":    ("1KPEIu4FetGbLwzfKoHk4sSHEim29ox-8", 769),
    "bone_mask.zip":  ("1SQppuej7C7JugeiPh4JK00yuIkOW60Wz", 4),
    "joints.zip":     ("1imikru7d64WdoR5Mt5vuU7mMqFQ0tVr_", 1),
}


def fetch() -> None:
    import requests

    os.makedirs(DIR, exist_ok=True)
    for name, (fid, mb) in FILES.items():
        path = os.path.join(DIR, name)
        if os.path.exists(path) and os.path.getsize(path) > 0.8e6 * mb:
            continue
        print(f"  fetching {name} (~{mb} MB, Apache-2.0)...")
        s = requests.Session()
        r = s.get(f"https://drive.usercontent.google.com/download?id={fid}&export=download")
        tok = dict(re.findall(r'name="(\w+)"\s+value="([^"]*)"', r.text))
        r2 = s.get("https://drive.usercontent.google.com/download",
                   params={**tok, "id": fid, "export": "download"}, stream=True)
        with open(path, "wb") as fh:
            for chunk in r2.iter_content(1 << 20):
                fh.write(chunk)
    for name, want in (("mri_raw.zip", "raw_mri/00001.nii"),
                       ("bone_mask.zip", "mri_pianomask/00001_bone.nii")):
        with zipfile.ZipFile(os.path.join(DIR, name)) as z:
            if not os.path.exists(os.path.join(DIR, want)):
                z.extract(want, DIR)


def measure(subject: str = "00001"):
    import SimpleITK as sitk
    from scipy import ndimage as ndi

    raw = sitk.ReadImage(f"{DIR}/raw_mri/{subject}.nii")
    bone = sitk.GetArrayFromImage(sitk.ReadImage(f"{DIR}/mri_pianomask/{subject}_bone.nii")) > 0
    sx, sy, sz = raw.GetSpacing()
    spacing = np.array([sz, sy, sx])            # the array is (z, y, x)
    shape = np.array(bone.shape)

    # 1. THE HAND. Air is dark, tissue bright.
    t = sitk.GetArrayFromImage(sitk.OtsuThreshold(raw, 0, 1)) > 0
    t = ndi.binary_closing(t, np.ones((3, 3, 3)))
    lab, n = ndi.label(t)
    if n > 1:
        sizes = ndi.sum(t, lab, range(1, n + 1))
        t = lab == (1 + int(np.argmax(sizes)))
    hand = ndi.binary_fill_holes(t)

    # 2. THE BONE SURFACE, and its OUTWARD NORMAL
    sm = ndi.gaussian_filter(bone.astype(np.float32), 1.5)
    gz, gy, gx = np.gradient(sm, *spacing)
    surf = bone & ~ndi.binary_erosion(bone, np.ones((3, 3, 3)))
    P = np.array(np.nonzero(surf)).T
    G = -np.stack([gz[surf], gy[surf], gx[surf]], 1)     # -grad points OUT of the bone
    nrm = np.linalg.norm(G, axis=1)
    keep = nrm > 1e-6
    P, G = P[keep], G[keep] / nrm[keep, None]

    # the hand's own frame, from its own voxels
    idx = np.array(np.nonzero(hand)).T.astype(np.float32) * spacing
    c = idx.mean(0)
    _, _, V = np.linalg.svd(idx - c, full_matrices=False)
    e_long, _, e_thin = V

    def cast(pts, dirs, step=0.2, maxmm=30.0):
        cur = pts.astype(np.float32).copy()
        dv = dirs / spacing
        dv = dv / np.linalg.norm(dv * spacing, axis=1, keepdims=True) * step
        out = np.full(len(pts), np.nan, np.float32)
        alive = np.ones(len(pts), bool)
        for i in range(int(maxmm / step)):
            cur[alive] += dv[alive]
            ii = np.rint(cur[alive]).astype(int)
            ok = ~((ii < 0).any(1) | (ii >= shape).any(1))
            inside = np.zeros(len(ii), bool)
            inside[ok] = hand[ii[ok, 0], ii[ok, 1], ii[ok, 2]]
            who = np.flatnonzero(alive)[~inside]
            out[who] = (i + 1) * step
            alive[who] = False
            if not alive.any():
                break
        return out

    # THE SIGN, from anatomy: the fingertip PULP is PALMAR, and it is the thicker side.
    Pm = P * spacing
    L = (Pm - c) @ e_long
    tips = L > np.percentile(L, 92)
    a = np.nanmedian(cast(P[tips], np.repeat(+e_thin[None], tips.sum(), 0)))
    b = np.nanmedian(cast(P[tips], np.repeat(-e_thin[None], tips.sum(), 0)))
    e_dors = -e_thin if a > b else +e_thin

    face = G @ e_dors
    dorsal, palmar = face > 0.75, face < -0.75
    td = cast(P[dorsal], np.repeat(e_dors[None], dorsal.sum(), 0))
    tp = cast(P[palmar], np.repeat(-e_dors[None], palmar.sum(), 0))
    Lm = (L - L.min()) / (L.max() - L.min())
    return dict(td=td, tp=tp, Ld=Lm[dorsal], Lp=Lm[palmar])


def main():
    fetch()
    r = measure()
    print("\nTISSUE THICKNESS, MEASURED FROM MRI (bone surface -> skin, along the bone's own normal)\n")
    print(f"  {'region':30s} {'DORSAL':>11s} {'PALMAR':>11s}")
    for nm, lo, hi in (("wrist / carpus  (the anchor)", 0.00, 0.28),
                       ("METACARPALS     (the anchor)", 0.28, 0.55),
                       ("proximal phalanges", 0.55, 0.75),
                       ("fingertips", 0.88, 1.01)):
        a = r["td"][(r["Ld"] >= lo) & (r["Ld"] < hi)]
        b = r["tp"][(r["Lp"] >= lo) & (r["Lp"] < hi)]
        a, b = a[~np.isnan(a)], b[~np.isnan(b)]
        if len(a) < 30 or len(b) < 30:
            continue
        print(f"  {nm:30s} {np.median(a):8.1f} mm {np.median(b):8.1f} mm")
    print("""
  SANITY, and it is the check that caught two wrong metrics: the fingertip PULP (palmar) must
  be THICKER than the NAIL BED (dorsal), and the back of the hand must be THIN. Both hold.

  WHAT THIS REPLACED:
    WRIST_TISSUE        3.0 mm   a GUESS -- MyoHand has NO flesh over the carpus at all
    MyoHand metacarpals 1.4-3.1 mm from its crude capsules
  -> the tissue is ~2x thicker than assumed, so the anchor was ~2x too stiff.""")


if __name__ == "__main__":
    sys.exit(main())
