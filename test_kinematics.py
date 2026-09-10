"""Verification table for the kinematics, re-established from scratch.

The four rows of ``stewart-ik-derivation.md`` sec.7.  The document recorded
these as passing, but every function they exercise raised
``NotImplementedError`` in the repo, so the table was treated as unverified
and is re-run here.

Plain numpy, no pytest (CLAUDE.md: numpy + matplotlib only).

    python test_kinematics.py        # exit 0 all pass, 1 otherwise

All geometry below is a TEST FIXTURE, not a design decision.
"""
from __future__ import annotations

import sys

import numpy as np

from stewart.geometry import Geometry, base_ring, make_geometry
from stewart.kinematics import arm_tips, legs, stage1

TOL = 1e-12

# ---- FIXTURES (not decisions) -------------------------------------------- #
FIX = dict(r_b=1.0, beta=20.0, delta=40.0, r_p=0.85, beta_p=40.0,
           c_p=0.10, a=0.20, d=1.20)


def rot_z(deg):
    t = np.deg2rad(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def fixture_geom():
    return make_geometry(FIX["r_b"], FIX["beta"], FIX["delta"],
                         FIX["r_p"], FIX["beta_p"],
                         FIX["a"], FIX["d"], c_p=FIX["c_p"])


ROWS = []


def row(name, residual, note=""):
    ROWS.append((name, float(residual), residual <= TOL, note))


# ---- row 1 : stage1(R=I, T=0) == p --------------------------------------- #
def check_stage1_identity():
    g = fixture_geom()
    q = stage1(g, np.eye(3), np.zeros(3))
    row("stage1(R=I, T=0) == p",
        np.max(np.abs(q - g.p)),
        f"shape {q.shape}")


# ---- row 2 : 90 deg about z sends (10,0,0) -> (0,10,0) ------------------- #
def check_rotation_not_transposed():
    """Purpose-built Geometry so the probe column is literally (10, 0, 0).

    A transposed R would send it to (0, -10, 0), so the residual separates the
    two unambiguously.
    """
    b, n, u = base_ring(FIX["r_b"], FIX["beta"], FIX["delta"])
    p = np.zeros((3, 6))
    p[:, 0] = (10.0, 0.0, 0.0)
    p[0, 1:] = 1.0                      # filler columns, any finite value
    g = Geometry(p=p, b=b, n=n, u=u, a=FIX["a"], d=FIX["d"])

    q = stage1(g, rot_z(90.0), np.zeros(3))
    row("Rz(90) sends (10,0,0) -> (0,10,0)",
        np.max(np.abs(q[:, 0] - np.array([0.0, 10.0, 0.0]))),
        f"got {np.round(q[:, 0], 12).tolist()}")


# ---- row 3 : arm_tips(0) is exactly a from every shaft ------------------- #
def check_tip_distance():
    g = fixture_geom()
    tips = arm_tips(g, np.zeros(6))
    dist = np.linalg.norm(tips - g.b, axis=0)
    row("|arm_tips(0) - b| == a for all six",
        np.max(np.abs(dist - g.a)),
        f"spread {dist.max() - dist.min():.3e}")


# ---- row 4 : arm_tips(0) == b + a*u -------------------------------------- #
def check_tip_direction():
    """Not redundant with row 3.

    ``n`` is also unit length, so a tip placed along ``n`` instead of ``u`` is
    still exactly ``a`` from the shaft and row 3 passes anyway.  This row is
    what catches a u/n swap.
    """
    g = fixture_geom()
    tips = arm_tips(g, np.zeros(6))
    row("arm_tips(0) == b + a*u",
        np.max(np.abs(tips - (g.b + g.a * g.u))),
        "catches a u/n swap that row 3 cannot")

    # and confirm the swap really would survive row 3
    swapped = g.b + g.a * g.n
    d_sw = np.linalg.norm(swapped - g.b, axis=0)
    ROWS.append((
        "  (control) b + a*n also passes row 3",
        float(np.max(np.abs(d_sw - g.a))),
        True,
        "confirms row 3 alone is insufficient",
    ))


def main():
    check_stage1_identity()
    check_rotation_not_transposed()
    check_tip_distance()
    check_tip_direction()

    print("=" * 76)
    print("VERIFICATION TABLE  (derivation sec.7)   tol = 1e-12")
    print(f"fixture: {FIX}")
    print("=" * 76)
    print(f"{'row':<42} {'residual':>12}  {'':4} note")
    print("-" * 76)
    ok = True
    for name, res, passed, note in ROWS:
        mark = "PASS" if passed else "FAIL"
        if not passed:
            ok = False
        print(f"{name:<42} {res:>12.3e}  {mark:4} {note}")
    print("-" * 76)
    print("ALL PASS" if ok else "FAILURES PRESENT")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
