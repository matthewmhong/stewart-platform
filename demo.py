"""End-to-end run on the design to build.

Prints the geometry summary and the evaluator's report, draws the platform
tilted to R1 into ``demo.png``, then round-trips random poses around home
through ``ik`` and ``fk``.
"""
from __future__ import annotations

import matplotlib
matplotlib.use("Agg")  # headless: just write the PNG

import numpy as np

from stewart import kinematics
from stewart.geometry import make_geometry
from stewart.performance import evaluate, tilt_pose, Requirements
from stewart.plotting import arm_circles, draw_pose
from stewart.roundtrip import random_poses, round_trip

PNG = "demo.png"
_RULE = "=" * 64

# The design to build (STATUS.md, 2026-09-18).  Angles in degrees, lengths mm.
DESIGN = dict(r_b=80, beta=10, delta=0, r_p=70, beta_p=35, a=22.5, d=70)


def main():
    geom = make_geometry(**DESIGN)
    req = Requirements()

    print(_RULE)
    geom.summary()

    print(_RULE)
    rep = evaluate(geom, req=req)
    print(rep.summary())
    z = rep.home_z_mm

    print(_RULE)
    R, T = tilt_pose(np.radians(req.tilt_deg), np.radians(30.0), z)
    h = kinematics.ik(geom, R, T)
    q = R @ geom.p + T[:, None]
    ax = draw_pose(geom, q, h=h, R=R, T=T,
                   title=f"design to build, tilted {req.tilt_deg} deg (R1)")
    arm_circles(geom, ax)
    ax.figure.savefig(PNG, dpi=120, bbox_inches="tight")
    print(f"wrote {PNG}")

    print(_RULE)
    print("round trip, random poses within R1 around home:")
    poses = random_poses(8, max_translation_mm=3.0, max_tilt_deg=req.tilt_deg,
                         centre_mm=(0.0, 0.0, z))
    round_trip(geom, poses, kinematics.ik, kinematics.fk)


if __name__ == "__main__":
    main()
