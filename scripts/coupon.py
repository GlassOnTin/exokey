"""BENCH COUPONS -- print these to measure the read-out for real.

    PYTHONPATH=. .venv/bin/python scripts/coupon.py

Stage 1 of the read-out verification ladder (VISION 8.15l): a family of TPU cradle inserts across
the dome thickness/radius the arithmetic brackets, plus one PA seat plate for the Hall, so the
bench can measure k (weights), mT-vs-displacement (feeler gauges + a TLV493D breakout), the
five-direction confusion matrix, and 1k-cycle creep -- the numbers manufacture.readout only
PREDICTS. Writes out/coupon_*.stl. Nothing here is on the critical print path; it is test hardware.
"""
from __future__ import annotations

from manufacture import wellmod as wm


def main():
    meshes = wm.coupon_meshes()
    print(f"BENCH COUPONS  ({len(meshes)} parts)\n")
    for name, m in sorted(meshes.items()):
        path = f"out/coupon_{name}.stl"
        m.export(path)
        tag = "watertight" if m.is_watertight else "⚠ NOT watertight"
        print(f"  {name:22s} {m.volume*1e9:7.1f} mm^3  {m.body_count} piece  {tag}  -> {path}")
    print("\n  print the inserts in TPU 95A, the seat in PA (or any rigid plastic for the bench).")
    print("  drop a Ø3x1 mm N42 disc into each insert; sit the insert's skirt on the seat's PCB rim.")


if __name__ == "__main__":
    main()
