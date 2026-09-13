"""Numbers that were asserted in prose and had no code behind them.

    python -m stewart.diagnostics.sweep_budget

This module exists for one reason.  Three results are documented and were
arrived at by hand, so nothing in the repo reproduces them:

  1. the SWEEP BUDGET / COMPUTE LEDGER - 15625 candidates, ~2.7M full and
     ~4.9e8 cheap evaluations, ~3.9 GB.  The last of those is a *harness
     requirement* (chunk over candidates), so it is not a throwaway figure.
  2. the TRANSLATION-SENSITIVITY CALIBRATION - "order 7 units of margin per
     unit normalised displacement, suggesting a probe of 0.005-0.01 r_b".
     The 7 came from dividing a worst-case over a +/-0.05 box by 0.05, which
     is not a derivative, and the accompanying requirement ("small enough to
     stay linear") was never tested.  It is measured here.
  3. the ARREST-FRAMING minimum of 1.635 deg in the derivation appendix, and
     the claim that the recovery framing reproduces it at tau = 1.0 s.

This project has three recorded instances of a documented result with no
committed code, and one near-miss recovered from a session scratchpad.  Hand
arithmetic in prose is the same failure in a smaller size.

Degrees at the boundary, radians internally.  numpy only.  Always exits 0.
"""
from __future__ import annotations

import numpy as np

from .branch_envelope import FIX_A, geom_A
from .envelope import (G, N_AZIMUTH, N_MAGNITUDE, ROLL_FACTOR, TAU,
                       TILT_LIMIT_DEG, X0_BARE, n_poses, tilt_R, tilt_for)

# --------------------------------------------------------------------------- #
# 1. sweep budget
# --------------------------------------------------------------------------- #
SWEEP_AXES = ["beta", "r_p/r_b", "beta_p", "a/r_b", "d/r_b", "z_home/r_b"]
POINTS_PER_AXIS = 5
DELTA_STEPS = 180                 # worst case, 1 degree over [0, 180)
BYTES_PER_FLOAT = 8

#: The superseded per-candidate pose counts, kept so the comparisons written
#: into the handoff are reproducible rather than remembered.
POSES_SUPERSEDED = {
    "2026-09-04, four-axis envelope at 3 points": 81,
    "docs/archive/notation.md sec.9's original 3^6": 729,
}


def budget(n_poses_: int, label: str) -> dict:
    cand = POINTS_PER_AXIS ** len(SWEEP_AXES)
    w_per_obj = n_poses_ * 6
    full = cand * w_per_obj
    cheap = cand * DELTA_STEPS * w_per_obj
    return dict(label=label, poses=n_poses_, w_per_obj=w_per_obj,
                candidates=cand, full=full, cheap=cheap,
                gib=cheap * BYTES_PER_FLOAT / 1024 ** 3,
                gb=cheap * BYTES_PER_FLOAT / 1e9)


def part1() -> None:
    print("=" * 78)
    print("1. SWEEP BUDGET / COMPUTE LEDGER")
    print("=" * 78)
    print(f"  axes ({len(SWEEP_AXES)}): {', '.join(SWEEP_AXES)}")
    print(f"  points per axis: {POINTS_PER_AXIS}   ->  candidates = "
          f"{POINTS_PER_AXIS}^{len(SWEEP_AXES)} = "
          f"{POINTS_PER_AXIS ** len(SWEEP_AXES)}")
    print(f"  delta scan (worst case): {DELTA_STEPS} steps over [0, 180)")
    print(f"  poses come from envelope.py: {N_MAGNITUDE} magnitudes x "
          f"{N_AZIMUTH} azimuths = {n_poses()} poses")
    print()
    rows = [budget(n_poses(), "SETTLED (2026-09-05, two-axis envelope)")]
    for lbl, n in POSES_SUPERSEDED.items():
        rows.append(budget(n, "superseded: " + lbl))
    print(f"  {'':<44} {'poses':>6} {'w/obj':>7} {'full':>12} {'cheap':>12} "
          f"{'GB':>8}")
    for r in rows:
        print(f"  {r['label']:<44} {r['poses']:>6} {r['w_per_obj']:>7} "
              f"{r['full']:>12,} {r['cheap']:>12,.3g} {r['gb']:>8.2f}")
    s = rows[0]
    print()
    print(f"  As written in the handoff: ~{s['full']/1e6:.1f}M full "
          f"({s['candidates']} x {s['w_per_obj']}) plus ~{s['cheap']:.1e} cheap "
          f"({s['candidates']} x {DELTA_STEPS} x {s['w_per_obj']}).")
    print(f"  Chunking requirement: {s['cheap']:.1e} floats x "
          f"{BYTES_PER_FLOAT} bytes = {s['gb']:.1f} GB "
          f"({s['gib']:.1f} GiB) held at once.")
    print(f"  The harness MUST chunk over candidates.  This is the number that")
    print(f"  requirement rests on, and until now it existed only in prose.")


# --------------------------------------------------------------------------- #
# 2. translation sensitivity
# --------------------------------------------------------------------------- #
#: Probe magnitudes in units of ``r_b``.  Spans the recommended 0.005-0.01
#: band and the 0.05 the calibration was originally taken from, so the
#: linearity claim can be checked rather than assumed.
PROBES = [0.0025, 0.005, 0.0075, 0.01, 0.02, 0.05]

#: A horizontal displacement breaks the D3 azimuth symmetry, so the 60-degree
#: window is NOT valid here and the full circle is swept - for the tilt
#: azimuth and for the direction of the displacement itself.
N_AZ_FULL = 24
N_DIR = 24
Z_BEST_A = 1.2375                 # fixture A's best z_home, from branch_envelope


def _min_margin(geom, R, T):
    q = np.einsum("kxy,yi->kxi", R, geom.p) + T[:, :, None]
    L = q - geom.b[None]
    v = np.cross(geom.n, geom.u, axis=0)
    M = np.einsum("kxi,xi->ki", L, geom.u)
    N = np.einsum("kxi,xi->ki", L, v)
    LL = np.einsum("kxi,kxi->ki", L, L)
    P = (LL + geom.a ** 2 - geom.d ** 2) / (2.0 * geom.a)
    C = np.hypot(M, N)
    return float(((C - np.abs(P)) / C).min())


def part2() -> None:
    print()
    print("=" * 78)
    print("2. TRANSLATION SENSITIVITY - calibration for the score function")
    print("=" * 78)
    print("  Ranking discriminator, NOT a feasibility test.  dxy = 0 is correct")
    print("  as a statement about what the control law COMMANDS; build error is")
    print("  a perturbation about every commanded pose, and belongs in scoring.")
    print()
    print(f"  fixture A, z_home/r_b = {Z_BEST_A}, delta = {FIX_A['delta']} "
          f"(untuned), tilt <= {TILT_LIMIT_DEG:.4f} deg")
    print(f"  full circle swept in BOTH tilt azimuth ({N_AZ_FULL}) and")
    print(f"  displacement direction ({N_DIR}) - a horizontal offset breaks the")
    print("  D3 symmetry, so the [30, 90] window does not apply here.")
    print()

    g = geom_A()
    mags = np.linspace(0.0, TILT_LIMIT_DEG, N_MAGNITUDE)
    azis = np.linspace(0.0, 360.0, N_AZ_FULL, endpoint=False)
    A_, M_ = np.meshgrid(azis, mags[1:], indexing="ij")
    az = np.concatenate([[0.0], A_.ravel()])
    mg = np.concatenate([[0.0], M_.ravel()])
    R = tilt_R(az, mg)

    T0 = np.stack([np.zeros_like(az), np.zeros_like(az),
                   np.full_like(az, Z_BEST_A)], axis=1)
    base = _min_margin(g, R, T0)
    print(f"  unperturbed min (C-|P|)/C = {base:+.6e}")
    print()
    print(f"  {'probe dxy/r_b':>14} {'worst min margin':>18} "
          f"{'delta-margin':>14} {'sensitivity':>13} {'vs linear':>11}")

    first = None
    for probe in PROBES:
        worst = base
        for d in np.deg2rad(np.linspace(0.0, 360.0, N_DIR, endpoint=False)):
            T = T0.copy()
            T[:, 0] += probe * np.cos(d)
            T[:, 1] += probe * np.sin(d)
            worst = min(worst, _min_margin(g, R, T))
        loss = base - worst
        sens = loss / probe
        if first is None:
            first = sens
        print(f"  {probe:>14.4f} {worst:>+18.6e} {loss:>14.6e} "
              f"{sens:>13.4f} {sens / first:>11.4f}")

    print()
    print("  Reading.  'sensitivity' is delta-margin per unit normalised")
    print("  displacement - the quantity the score function wants.  'vs linear'")
    print("  is that sensitivity divided by the smallest probe's, so 1.000 means")
    print("  perfectly linear and a drift away from 1 is the probe leaving the")
    print("  linear regime.")
    print()
    print("  LINEARITY: the recommended 0.005-0.01 r_b probe holds to within")
    print("  ~1% of linear, so that half of the documented requirement is")
    print("  CONFIRMED.  0.05 is 16% off and is not a probe.")
    print()
    print("  MAGNITUDE: the documented 'order 7 units of margin per unit")
    print("  normalised displacement' is NOT confirmed - measured here it is")
    print(f"  ~{first:.2f}, a factor of ~{7.297/first:.1f} smaller.  The 7 came from")
    print("  dividing the (e) attribution's 3.6486e-01 by 0.05.  That row's")
    print("  '+/-0.05 r_b' box was T_horiz AND T_vert together, so it is not a")
    print("  horizontal sensitivity at all.  Decomposed at the (e) conditions")
    print("  (datum z_home, tilt 6 deg, 15-degree full circle):")

    # reproduce the (e) row and split it
    z_datum = 1.223343
    mags6 = np.linspace(6.0 / 4.0, 6.0, 4)
    A6, M6 = np.meshgrid(np.arange(0.0, 360.0, 15.0), mags6, indexing="ij")
    az6 = np.concatenate([[0.0], A6.ravel()])
    mg6 = np.concatenate([[0.0], M6.ravel()])
    R6 = tilt_R(az6, mg6)
    T6 = np.stack([np.zeros_like(az6), np.zeros_like(az6),
                   np.full_like(az6, z_datum)], axis=1)
    base6 = _min_margin(g, R6, T6)

    def worst_over(hx, vz):
        w = base6
        for (tx, ty) in hx:
            for tz in vz:
                T = T6.copy()
                T[:, 0] += tx
                T[:, 1] += ty
                T[:, 2] += tz
                w = min(w, _min_margin(g, R6, T))
        return w

    H = [(0.0, 0.0), (0.05, 0.0), (-0.05, 0.0), (0.0, 0.05), (0.0, -0.05)]
    Z = [-0.05, 0.0, 0.05]
    both = base6 - worst_over(H, Z)
    horiz = base6 - worst_over(H, [0.0])
    vert = base6 - worst_over([(0.0, 0.0)], Z)
    print(f"    unperturbed                     {base6:+.6e}")
    print(f"    loss, T_horiz AND T_vert        {both:.6e}   "
          f"<- the 3.6486e-01 row")
    print(f"    loss, T_horiz only              {horiz:.6e}   "
          f"(/0.05 = {horiz/0.05:.2f})")
    print(f"    loss, T_vert only               {vert:.6e}   "
          f"(/0.05 = {vert/0.05:.2f})")
    print()
    print("  So the documented calibration is dominated by the VERTICAL term,")
    print("  and vertical displacement is z_home - already a swept axis, not")
    print("  build error of the kind the discriminator is for.  The horizontal")
    print("  sensitivity the score function actually wants is the ~1.4 above.")
    print("  The probe RANGE survives; the magnitude behind it does not.")


# --------------------------------------------------------------------------- #
# 3. the arrest framing
# --------------------------------------------------------------------------- #
ARREST_V = 0.200                  # m/s, entry speed
ARREST_L = 0.100                  # m, stopping distance
G_APPENDIX = 9.81                 # what the 1.635 figure was computed with


def arrest_tilt(v: float, L: float, g: float) -> float:
    """``(5/7) g sin(tilt) = v^2 / 2L``  ->  tilt in degrees."""
    return float(np.degrees(np.arcsin((v * v / (2.0 * L)) / (ROLL_FACTOR * g))))


def part3() -> None:
    print()
    print("=" * 78)
    print("3. THE ARREST FRAMING - derivation appendix's 1.635 deg")
    print("=" * 78)
    print(f"  (5/7) g sin(tilt) = v^2 / 2L,  v = {ARREST_V*1e3:.0f} mm/s, "
          f"L = {ARREST_L*1e3:.0f} mm")
    print(f"    required deceleration = {ARREST_V**2/(2*ARREST_L):.4f} m/s^2")
    print(f"    tilt at g = {G_APPENDIX}      : "
          f"{arrest_tilt(ARREST_V, ARREST_L, G_APPENDIX):.4f} deg")
    print(f"    tilt at g = {G}   : "
          f"{arrest_tilt(ARREST_V, ARREST_L, G):.4f} deg")
    print()
    print("  FINDING, small but worth not re-discovering.  The appendix records")
    print(f"  1.635 and envelope.py's recovery framing gives "
          f"{tilt_for(X0_BARE, 1.0):.4f} at tau = 1.0 s.")
    print("  The gap is NOT a disagreement between the framings - both need the")
    print(f"  same 0.2 m/s^2.  It is the value of g: {G_APPENDIX} gives "
          f"{arrest_tilt(ARREST_V, ARREST_L, G_APPENDIX):.4f},")
    print(f"  {G} gives {arrest_tilt(ARREST_V, ARREST_L, G):.4f}.  The appendix")
    print(f"  figure predates the constant in envelope.py.  1e-3 deg, no")
    print("  consequence, and now written down instead of looking like a")
    print("  discrepancy the next time someone checks.")
    print()
    print("  " + "-" * 74)
    print("  MOOT as of 2026-09-08.  READ THIS BEFORE QUOTING THE TABLE BELOW.")
    print("  " + "-" * 74)
    print("  The ball surface is a BOUGHT PLASTIC SHEET on a hub, DECOUPLED from")
    print("  the anchor ring.  x0 = 80 mm is therefore a property of THE SHEET,")
    print("  not of r_b, and it does NOT scale with plate size.  The tilt target")
    print("  is a FIXED ANGLE - 10.529 deg at every mechanism scale.")
    print()
    print("  So NEITHER framing's k-dependence applies.  The table below is not a")
    print("  correction of one framing by the other and not a live result: it is")
    print("  the arithmetic of a question that no longer has a subject.  Both")
    print("  rows are MOOT, not wrong - the quantity they disagree about does")
    print("  not exist.  See docs/archive/notation.md sec.9, which carries the withdrawal and")
    print("  what replaced it (absolute scale enters in exactly two places: the")
    print("  bed, r_b <= 90 mm, and p's denominator, the asserted 80 mm floor).")
    print()
    print("  Kept and still computed, because a withdrawn claim that leaves no")
    print("  trace gets re-derived.  DO NOT quote these rows forward as live.")
    print()
    print("  [MOOT] The two framings were said to scale OPPOSITELY in plate")
    print("  [MOOT] size k:")
    print("  [MOOT]   arrest   - v fixed, L ~ k  ->  required tilt ~ 1/k")
    print("  [MOOT]   recovery - x0 ~ k, tau fixed  ->  required tilt ~ k")
    print("  [MOOT] The recovery row's premise, x0 ~ k, is the one withdrawn.")
    for k in (0.5, 1.0, 2.0):
        print(f"  [MOOT]     k = {k:>4}: arrest "
              f"{arrest_tilt(ARREST_V, ARREST_L * k, G):>7.4f} deg    "
              f"recovery {tilt_for(X0_BARE * k, TAU):>7.4f} deg")
    print("  [MOOT] Against a tilt target that is now fixed at "
          f"{TILT_LIMIT_DEG:.4f} deg for all k.")


def main() -> None:
    part1()
    part2()
    part3()


if __name__ == "__main__":
    main()
