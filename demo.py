"""End-to-end smoke run.  Works with everything in ``stewart.kinematics`` stubbed.

Prints the geometry summary, reports which of the five kinematics functions are
written versus stubbed, draws what is possible into ``demo.png``, then runs the
round-trip report.
"""
from __future__ import annotations

import inspect

import matplotlib
matplotlib.use("Agg")  # headless: just write the PNG

import numpy as np

from stewart import kinematics
from stewart.geometry import smoke_geometry
from stewart.plotting import arm_circles, draw_pose
from stewart.roundtrip import known_poses, round_trip

PNG = "demo.png"
_RULE = "=" * 64


def _kin_status():
    print("kinematics: written vs stubbed")
    for name in ("stage1", "legs", "arm_tips", "ik", "fk"):
        fn = getattr(kinematics, name)
        try:
            stubbed = "raise NotImplementedError" in inspect.getsource(fn)
        except OSError:
            stubbed = None
        mark = {True: "stub", False: "written", None: "unknown"}[stubbed]
        print(f"  {name:<9} {mark}")


def main():
    geom = smoke_geometry(seed=0)

    print(_RULE)
    geom.summary()

    print(_RULE)
    _kin_status()

    print(_RULE)
    # No IK yet: place the platform ring with a plain rigid transform (not
    # kinematics), draw the dashed b_i -> q_i version, and overlay the arm
    # circles.
    R = np.eye(3)
    T = np.array([0.0, 0.0, 130.0])
    q = R @ geom.p + T[:, None]
    ax = draw_pose(geom, q, h=None, R=R, T=T, title="smoke geometry - NOT a design")
    arm_circles(geom, ax)
    ax.figure.savefig(PNG, dpi=120, bbox_inches="tight")
    print(f"wrote {PNG}")

    print(_RULE)
    print("round trip (ik/fk stubbed -> every row reports 'not implemented'):")
    round_trip(geom, known_poses(), kinematics.ik, kinematics.fk)


if __name__ == "__main__":
    main()
