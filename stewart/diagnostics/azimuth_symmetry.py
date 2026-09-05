"""Does tilt azimuth need sweeping over the full circle?  (Diagnostic (a).)

    python -m stewart.diagnostics.azimuth_symmetry

**The claim under test** (his, 2026-09-05, unverified when written): tilt
azimuth need only be swept over a 60-degree window rather than ``[0, 360)``,
because the leg set has D3 symmetry - tilting at ``psi`` and ``psi + 120`` is
the same configuration with the legs permuted, a mirror halves it again, and
both candidate aggregates (``max_i |w_i|`` and ``min_i (C_i - |P_i|)/C_i``) are
invariant under a leg permutation.

Nothing here assumes the claim.  Azimuth is swept over the **full circle** and
the two invariances are measured:

  P1  period 120 :  A(psi) == A(psi + 120)
  M0  mirror at 0:  A(psi) == A(-psi)          <- what "[0, 60]" needs
  M90 mirror at 90: A(psi) == A(180 - psi)     <- what "[30, 90]" needs

Only one of M0 / M90 can be expected to hold in general, and which one it is
decides **where** the 60-degree window sits.  The window's *width* follows from
P1 plus either mirror; its *position* does not.

A negative control runs the identical tests on a geometry with the D3 symmetry
deliberately broken (one platform anchor displaced), so a pass cannot be a pass
for the wrong reason.

Degrees at the boundary, radians internally.  Arrays are (3, 6).  numpy only.
Reports findings; always exits 0.
"""
from __future__ import annotations

import numpy as np

from ..geometry import Geometry, make_geometry
from .envelope import TILT_LIMIT_DEG, tilt_R

# --------------------------------------------------------------------------- #
# fixtures - geometries, not decisions
# --------------------------------------------------------------------------- #
# Each entry is (label, kwargs for make_geometry, z_home).  Spread over beta,
# beta_p, delta, r_p/r_b, a/r_b, d/r_b and h_p so a symmetry that held only at
# a symmetric point would show up.  r_b = 1 throughout; everything normalised.
FIXTURES = [
    ("A  beta=20 beta_p=40 delta=40",
     dict(r_b=1.0, beta=20.0, delta=40.0, r_p=0.85, beta_p=40.0,
          a=0.20, d=1.20, h_p=0.10), 0.95),
    ("B  beta=30 beta_p=30 delta=0   (beta_p == beta, delta at the endpoint)",
     dict(r_b=1.0, beta=30.0, delta=0.0, r_p=1.00, beta_p=30.0,
          a=0.15, d=1.30, h_p=0.05), 1.05),
    ("C  beta=10 beta_p=55 delta=137 (both rings far from regular)",
     dict(r_b=1.0, beta=10.0, delta=137.0, r_p=0.60, beta_p=55.0,
          a=0.35, d=1.10, h_p=0.20), 0.90),
    ("D  beta=48 beta_p=8  delta=90  (delta on a mirror-fixed value)",
     dict(r_b=1.0, beta=48.0, delta=90.0, r_p=1.20, beta_p=8.0,
          a=0.10, d=1.55, h_p=0.00), 1.00),
]

#: Azimuth resolution for the sweep.  0.25 deg over the full circle = 1440
#: samples, chosen so that 120 and 180 are both exact multiples of the step -
#: the invariance tests then compare *sampled* values, with no interpolation
#: error to confuse for a symmetry breakdown.
AZ_STEP_DEG = 0.25

#: Tilt magnitudes at which to test.  The limit itself plus interior values;
#: 0 is excluded because at zero tilt every azimuth gives the same pose and
#: the tests would pass vacuously.
TEST_MAGNITUDES = [TILT_LIMIT_DEG, 0.5 * TILT_LIMIT_DEG, 2.0]


# --------------------------------------------------------------------------- #
# aggregates
# --------------------------------------------------------------------------- #
def aggregates(geom: Geometry, R: np.ndarray, z_home: float):
    """``(max_i |w_i|, min_i (C_i - |P_i|)/C_i)`` for each rotation in ``R``.

    ``R`` has shape ``(K, 3, 3)``; returns two ``(K,)`` arrays.  Written out
    here rather than called through :func:`~stewart.kinematics.ik` because the
    margin is wanted even where it is negative, and ``ik`` raises there.
    """
    T = np.array([0.0, 0.0, z_home])
    q = np.einsum("kxy,yi->kxi", R, geom.p) + T[None, :, None]
    L = q - geom.b[None]
    w = np.einsum("kxi,xi->ki", L, geom.n)
    M = np.einsum("kxi,xi->ki", L, geom.u)
    v = np.cross(geom.n, geom.u, axis=0)
    N = np.einsum("kxi,xi->ki", L, v)
    LL = np.einsum("kxi,kxi->ki", L, L)
    P = (LL + geom.a ** 2 - geom.d ** 2) / (2.0 * geom.a)
    C = np.hypot(M, N)
    margin = (C - np.abs(P)) / C
    return np.max(np.abs(w), axis=1), np.min(margin, axis=1)


def broken_geometry(kwargs: dict) -> Geometry:
    """The same machine with D3 deliberately broken: one anchor displaced.

    Leg 3 (0-indexed 2) is pushed 3% of ``r_b`` outward in ``x``.  Nothing else
    changes, so any test that still passes is passing for a reason other than
    the symmetry.
    """
    g = make_geometry(**kwargs)
    p = np.array(g.p, copy=True)
    p[0, 2] += 0.03 * kwargs["r_b"]
    return Geometry(p=p, b=g.b, n=g.n, u=g.u, a=g.a, d=g.d)


def circular_shift(a: np.ndarray, k: int) -> np.ndarray:
    return np.roll(a, -k)


def run_one(label: str, geom: Geometry, z_home: float, mags) -> dict:
    """Measure P1, M0 and M90 on one geometry.  Returns worst deviations."""
    az = np.arange(0.0, 360.0, AZ_STEP_DEG)
    n = az.size
    assert abs(120.0 / AZ_STEP_DEG - round(120.0 / AZ_STEP_DEG)) < 1e-12
    k120 = int(round(120.0 / AZ_STEP_DEG))

    out = {"label": label, "rows": []}
    for mag in mags:
        R = tilt_R(az, mag)
        A_w, A_m = aggregates(geom, R, z_home)

        # scale of the quantity, so deviations can be read as relative
        s_w = float(np.max(np.abs(A_w)))
        s_m = float(np.max(np.abs(A_m)))

        # P1 : A(psi) == A(psi + 120)
        p1_w = float(np.max(np.abs(A_w - circular_shift(A_w, k120))))
        p1_m = float(np.max(np.abs(A_m - circular_shift(A_m, k120))))

        # M0 : A(psi) == A(-psi).  index j maps to (-j) mod n
        idx_m0 = (-np.arange(n)) % n
        m0_w = float(np.max(np.abs(A_w - A_w[idx_m0])))
        m0_m = float(np.max(np.abs(A_m - A_m[idx_m0])))

        # M90 : A(psi) == A(180 - psi)
        j180 = int(round(180.0 / AZ_STEP_DEG))
        idx_m90 = (j180 - np.arange(n)) % n
        m90_w = float(np.max(np.abs(A_w - A_w[idx_m90])))
        m90_m = float(np.max(np.abs(A_m - A_m[idx_m90])))

        out["rows"].append(dict(mag=mag, s_w=s_w, s_m=s_m,
                                p1_w=p1_w, p1_m=p1_m,
                                m0_w=m0_w, m0_m=m0_m,
                                m90_w=m90_w, m90_m=m90_m))
    return out


def print_block(res: dict) -> None:
    print(f"\n  {res['label']}")
    print(f"    {'tilt':>7} {'quantity':>10} {'scale':>12} "
          f"{'P1 (+120)':>12} {'M0 (-psi)':>12} {'M90 (180-psi)':>15}")
    for r in res["rows"]:
        print(f"    {r['mag']:>7.3f} {'max|w|':>10} {r['s_w']:>12.4e} "
              f"{r['p1_w']:>12.3e} {r['m0_w']:>12.3e} {r['m90_w']:>15.3e}")
        print(f"    {'':>7} {'min margin':>10} {r['s_m']:>12.4e} "
              f"{r['p1_m']:>12.3e} {r['m0_m']:>12.3e} {r['m90_m']:>15.3e}")


def main() -> None:
    print("=" * 78)
    print("DIAGNOSTIC (a) - is a 60-degree azimuth window sufficient, and where")
    print("=" * 78)
    print(f"azimuth swept over [0, 360) at {AZ_STEP_DEG} deg "
          f"({int(360 / AZ_STEP_DEG)} samples); 120 and 180 are exact multiples")
    print("of the step, so the tests compare sampled values with no interpolation.")
    print("tilt limit = %.4f deg.  T = (0, 0, z_home); dxy = dz = yaw = 0."
          % TILT_LIMIT_DEG)
    print("\nA deviation is 'float noise' at <= 1e-12 relative to the quantity's")
    print("own scale; the negative control shows what a genuine break looks like.")

    print("\n" + "-" * 78)
    print("SYMMETRIC GEOMETRIES")
    print("-" * 78)
    results = []
    for label, kw, z in FIXTURES:
        geom = make_geometry(**kw)
        res = run_one(label, geom, z, TEST_MAGNITUDES)
        results.append(res)
        print_block(res)

    print("\n" + "-" * 78)
    print("NEGATIVE CONTROL - leg 3's anchor displaced by 0.03 r_b in x")
    print("-" * 78)
    controls = []
    for label, kw, z in FIXTURES[:2]:
        geom = broken_geometry(kw)
        res = run_one("BROKEN " + label, geom, z, TEST_MAGNITUDES[:1])
        controls.append(res)
        print_block(res)

    # ----------------------------------------------------------------- #
    # verdict
    # ----------------------------------------------------------------- #
    def worst(rs, key):
        return max(max(r[key] / max(r["s_w" if key.endswith("_w") else "s_m"],
                                    1e-300)
                       for r in res["rows"]) for res in rs)

    p1 = max(worst(results, "p1_w"), worst(results, "p1_m"))
    m0 = max(worst(results, "m0_w"), worst(results, "m0_m"))
    m90 = max(worst(results, "m90_w"), worst(results, "m90_m"))
    cp1 = max(worst(controls, "p1_w"), worst(controls, "p1_m"))
    cm90 = max(worst(controls, "m90_w"), worst(controls, "m90_m"))

    TOL = 1e-12
    print("\n" + "=" * 78)
    print("VERDICT  (worst relative deviation over every fixture and magnitude)")
    print("=" * 78)
    print(f"  P1   period 120        : {p1:.3e}   "
          f"{'HOLDS' if p1 <= TOL else 'FAILS'}")
    print(f"  M0   mirror at psi=0   : {m0:.3e}   "
          f"{'HOLDS' if m0 <= TOL else 'FAILS'}")
    print(f"  M90  mirror at psi=90  : {m90:.3e}   "
          f"{'HOLDS' if m90 <= TOL else 'FAILS'}")
    print(f"  negative control, P1    : {cp1:.3e}   "
          f"{'HOLDS (control ineffective!)' if cp1 <= TOL else 'FAILS as intended'}")
    print(f"  negative control, M90   : {cm90:.3e}   "
          f"{'HOLDS (control ineffective!)' if cm90 <= TOL else 'FAILS as intended'}")

    print()
    if p1 <= TOL and m90 <= TOL and m0 > TOL:
        print("  FINDING.  The claim is RIGHT about the WIDTH and WRONG about the")
        print("  POSITION.  Period 120 holds and one mirror holds, so a 60-degree")
        print("  window is sufficient - but the mirror is at psi = 90 (and 30, by")
        print("  the C3), NOT at psi = 0.  The mirror lines in azimuth sit at")
        print("  30 + 60k.  So the window is [30, 90], not [0, 60].")
        print()
        print("  [0, 60] is reflection-symmetric about its own centre 30, so it")
        print("  covers each orbit it touches TWICE and misses the orbits around")
        print("  psi = 75 entirely - e.g. the orbit {75, 105} mod 120 meets [0,60]")
        print("  nowhere.  Sweeping it would silently omit part of the envelope.")
    elif p1 <= TOL and m0 <= TOL:
        print("  The claim holds as stated: [0, 60] is sufficient.")
    elif p1 <= TOL:
        print("  Period 120 holds but NEITHER mirror does.  A 120-degree window is")
        print("  sufficient; a 60-degree one is not.")
    else:
        print("  Period 120 FAILS.  The full circle is required.  Pose counts")
        print("  elsewhere in this package are wrong and must be recomputed.")

    print()
    print("  Why the geometric mirror at azimuth 0 gives an azimuth mirror at 90:")
    print("  reflection M in the vertical plane at azimuth m has det M = -1, and")
    print("  M R(n, th) M^T = R(det(M) M n, th).  For an axis at azimuth psi that")
    print("  is an axis at azimuth 2m + 180 - psi.  With mirror planes at m = 0,")
    print("  60, 120 the fixed azimuths are 90, 150, 30 (mod 180) - i.e. 30 + 60k.")
    print("  Physically: tilting about an axis lying IN a mirror plane reflects to")
    print("  the OPPOSITE tilt; it is the axis PERPENDICULAR to the plane that is")
    print("  self-symmetric.  notation.md sec.10's w-pattern table already says")
    print("  this ('tilt about an axis perpendicular to a mirror plane').")


if __name__ == "__main__":
    main()
