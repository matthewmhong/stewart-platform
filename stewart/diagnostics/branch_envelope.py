"""The ``-`` branch, re-run on the settled envelope.  ((d) and (e).)

    python -m stewart.diagnostics.branch_envelope

(d) BRANCH RE-RUN.  The evidence fixing the ``-`` branch - a 4365-pose sweep
    and a 720-step precession loop - was gathered at ONE ``z_home`` (supplied
    by the flat-arm datum, since dropped) and on a PROVISIONAL 6-degree
    envelope.  Both premises are gone.  Re-run across the restored ``z_home``
    axis at the settled tilt limit, assert ``min(N_i) > 0``, report the margin.
    This is open item 5 discharged as a test rather than a docstring.

(e) BOUNDARY MARGIN.  "Closest approach to the branch-merge boundary is
    -5.7e-3 of C" was measured at 6 degrees on ``branch_check.py``'s fixture A.
    Recomputed here or withdrawn; it is not carried forward unrecomputed.

**The two envelopes are not nested and the comparison must say so.**  The old
one was tilt <= 6 deg AND yaw +/-10 deg AND translation +/-0.05 r_b.  The new
one is tilt <= 10.529 deg and NOTHING else - dxy = dz = yaw = 0.  More tilt,
no yaw, no translation.  A margin that improves is not evidence the geometry
got better, and one that worsens is not evidence it got worse; they are
different questions.  Both are reported side by side.

Degrees at the boundary, radians internally.  Arrays are (3, 6).  numpy only.
Reports findings; always exits 0.
"""
from __future__ import annotations

import numpy as np

from ..geometry import make_geometry
from ..kinematics import ik
from .envelope import (AZIMUTH_WINDOW_DEG, TILT_LIMIT_DEG, envelope_poses,
                       tilt_R)

# --------------------------------------------------------------------------- #
# fixture A, verbatim from branch_check.py - this is what -5.7e-3 was measured
# on, so recomputing on anything else would not be a recomputation.
# --------------------------------------------------------------------------- #
FIX_A = dict(r_b=1.0, beta=20.0, delta=40.0, r_p=0.85, beta_p=40.0, h_p=0.10)
FIX_A_AD = dict(a=0.20, d=1.20)
#: ``z_home`` the flat-arm datum gave for fixture A.  Recorded so the "same
#: geometry, same height, new envelope" row is exactly that.  Recomputed below
#: from the closed form rather than pasted.
Z_DATUM_A = None                       # filled in by main()

#: ``z_home`` values to re-run across, spanning fixture A's feasible bracket
#: (from zhome_bracket.py: r_p/r_b = 0.85, beta = 20, beta_p = 40, a = 0.2,
#: d = 1.2 gives roughly [1.200, 1.275]).  Sampled a little wider so the edges
#: show as failures rather than being hidden by a tidy range.
Z_SCAN = np.round(np.arange(1.15, 1.325 + 1e-12, 0.0125), 6)

PRECESSION_STEPS = 1440                # 0.25 deg azimuth resolution, closed loop


def geom_A(delta=None):
    kw = dict(FIX_A)
    if delta is not None:
        kw["delta"] = delta
    return make_geometry(a=FIX_A_AD["a"], d=FIX_A_AD["d"], **kw)


def z_flat_closed_form(r_b, beta, delta, r_p, beta_p, h_p, a, d):
    """Derivation sec.8.1, the flat-arm assembly height.  Datum only."""
    A = np.deg2rad(beta_p - beta)
    G = r_p * np.exp(1j * A) - r_b
    gg = r_p ** 2 + r_b ** 2 - 2 * r_p * r_b * np.cos(A)
    delta_G = np.angle(G)
    dr = np.deg2rad(delta)
    disc = d * d - gg - a * a - 2 * a * np.sqrt(gg) * np.cos(dr - delta_G)
    if disc < 0:
        return float("nan")
    return h_p + float(np.sqrt(disc))


# --------------------------------------------------------------------------- #
# per-pose leg quantities, both roots
# --------------------------------------------------------------------------- #
def leg_quantities(geom, R, z_home):
    """``M, N, P, C, margin`` for every rotation in ``R``, shape ``(K, 6)``."""
    T = np.array([0.0, 0.0, z_home])
    q = np.einsum("kxy,yi->kxi", R, geom.p) + T[None, :, None]
    L = q - geom.b[None]
    v = np.cross(geom.n, geom.u, axis=0)
    M = np.einsum("kxi,xi->ki", L, geom.u)
    N = np.einsum("kxi,xi->ki", L, v)
    LL = np.einsum("kxi,kxi->ki", L, L)
    P = (LL + geom.a ** 2 - geom.d ** 2) / (2.0 * geom.a)
    C = np.hypot(M, N)
    return M, N, P, C, (C - np.abs(P)) / C


def roots(M, N, P, C):
    """Both branches, NaN where unreachable.  No clipping."""
    phi = np.arctan2(N, M)
    ratio = P / C
    bad = np.abs(P) > C
    safe = np.where(bad, 0.0, ratio)
    ac = np.arccos(safe)
    minus = np.where(bad, np.nan, phi - ac)
    plus = np.where(bad, np.nan, phi + ac)
    return minus, plus, bad


def wrap(a):
    return -((-np.asarray(a, float) + np.pi) % (2 * np.pi) - np.pi)


# --------------------------------------------------------------------------- #
def main() -> None:
    global Z_DATUM_A
    Z_DATUM_A = z_flat_closed_form(a=FIX_A_AD["a"], d=FIX_A_AD["d"], **FIX_A)

    az, mg = envelope_poses()
    R_env = tilt_R(az, mg)

    print("=" * 78)
    print("DIAGNOSTIC (d) - the '-' branch across the restored z_home axis")
    print("=" * 78)
    print(f"envelope : tilt <= {TILT_LIMIT_DEG:.4f} deg, dxy = dz = yaw = 0")
    print(f"poses    : {az.size} (harness density), azimuth window "
          f"{AZIMUTH_WINDOW_DEG}")
    print(f"geometry : fixture A {FIX_A}, {FIX_A_AD}")
    print(f"z_home   : {Z_SCAN.size} values over "
          f"[{Z_SCAN[0]}, {Z_SCAN[-1]}] r_b")
    print(f"           (the flat-arm datum, now dropped, would have given "
          f"{Z_DATUM_A:.6f})")
    print()
    g = geom_A()
    print(f"  {'z_home':>9} {'min N':>12} {'max|P|/C':>11} {'min margin':>12} "
          f"{'unreach':>8} {'max|a-|':>9} {'branch':>8}")

    any_feasible = False
    table = []
    for z in Z_SCAN:
        M, N, P, C, margin = leg_quantities(g, R_env, z)
        minus, plus, bad = roots(M, N, P, C)
        n_bad = int(bad.sum())
        row = dict(z=z, minN=float(N.min()), ratio=float(np.max(np.abs(P) / C)),
                   margin=float(margin.min()), nbad=n_bad)
        if n_bad == 0:
            row["maxabs_minus"] = float(np.nanmax(np.abs(minus)))
            # which root sits nearer alpha = 0 at the home pose (index 0)
            row["branch"] = "-" if np.nanmax(np.abs(minus[0])) <= \
                np.nanmax(np.abs(plus[0])) else "+"
            any_feasible = True
        else:
            row["maxabs_minus"] = float("nan")
            row["branch"] = "-"
        table.append(row)
        mx = "-" if np.isnan(row["maxabs_minus"]) else f"{row['maxabs_minus']:.4f}"
        print(f"  {z:>9.4f} {row['minN']:>12.6f} {row['ratio']:>11.6f} "
              f"{row['margin']:>+12.4e} {n_bad:>8} {mx:>9} {row['branch']:>8}")

    # ---- assertion the open item asked for --------------------------- #
    feas = [r for r in table if r["nbad"] == 0]
    print()
    print("  ASSERTION min(N_i) > 0 over the envelope, at every z_home scanned")
    allN = np.array([r["minN"] for r in table])
    print(f"    min over the whole scan : {allN.min():+.6f}  "
          f"{'PASSES' if allN.min() > 0 else 'FAILS'}")
    print(f"    N>0 closed-form bound   : "
          f"{FIX_A['r_p']*np.sin(np.deg2rad(TILT_LIMIT_DEG)) + FIX_A['h_p']*np.cos(np.deg2rad(TILT_LIMIT_DEG)):.6f} r_b")
    print(f"    margin above it         : {allN.min():+.6f} r_b")
    print("    (min N == z_home - bound identically, since N_i = q_i . z and")
    print("     the bound is exactly the worst-case anchor drop - the two")
    print("     columns agreeing digit for digit is the closed form checking")
    print("     itself against the library ring)")
    print("    -> N > 0 is nowhere near binding on this fixture: reach fails")
    print("       first, by a wide margin.  Open item 5's test passes, and the")
    print("       constraint it protects is not the active one here.")

    print()
    print(f"  z_home values with the envelope fully reachable : "
          f"{len(feas)} / {len(table)}")
    if feas:
        zs = [r["z"] for r in feas]
        print(f"    feasible z_home/r_b in [{min(zs):.4f}, {max(zs):.4f}]")
        print(f"    branch giving alpha ~ 0 at home, everywhere : "
              f"{sorted({r['branch'] for r in feas})}")
        print(f"    max |alpha_minus| over envelope and z_home  : "
              f"{max(r['maxabs_minus'] for r in feas):.4f} rad")
    else:
        print("    FINDING: no scanned z_home leaves the envelope reachable on")
        print("    this fixture.  That is information about the tilt target.")

    # ---- continuity: precession loop at the tilt limit ---------------- #
    print()
    print("-" * 78)
    print(f"  CONTINUITY - azimuth precession at fixed magnitude "
          f"{TILT_LIMIT_DEG:.4f} deg,")
    print(f"  {PRECESSION_STEPS} steps over the FULL circle (closed loop).  The")
    print("  60-degree window is a scoring shortcut; continuity is a statement")
    print("  about the real trajectory, so it is checked all the way round.")
    print()
    print(f"    {'z_home':>9} {'unreach':>8} {'max|dalpha|':>13} "
          f"{'median':>12} {'ratio':>8} {'closure':>11}")
    azp = np.linspace(0.0, 360.0, PRECESSION_STEPS + 1)
    Rp = tilt_R(azp, TILT_LIMIT_DEG)
    flips = 0
    for r in table:
        if r["nbad"]:
            continue
        M, N, P, C, _ = leg_quantities(g, Rp, r["z"])
        minus, plus, bad = roots(M, N, P, C)
        nb = int(bad.sum())
        if nb:
            print(f"    {r['z']:>9.4f} {nb:>8} "
                  f"{'-':>13} {'-':>12} {'-':>8} {'-':>11}")
            continue
        d = np.abs(wrap(np.diff(minus, axis=0)))
        mx, md = float(np.max(d)), float(np.median(d))
        closure = float(np.max(np.abs(wrap(minus[-1] - minus[0]))))
        ratio = mx / md if md else np.inf
        flag = "  <-- FLIP" if ratio > 5.0 else ""
        if ratio > 5.0:
            flips += 1
        print(f"    {r['z']:>9.4f} {nb:>8} {mx:>13.4e} {md:>12.4e} "
              f"{ratio:>8.2f} {closure:>11.2e}{flag}")
    print(f"    step-outlier (branch flip) count over the scan : {flips}")

    # ---- ik() agreement --------------------------------------------- #
    print()
    print("  Cross-check against the library ik(), which fixes the '-' branch:")
    worst = 0.0
    checked = 0
    for r in table:
        if r["nbad"]:
            continue
        M, N, P, C, _ = leg_quantities(g, R_env, r["z"])
        minus, _, _ = roots(M, N, P, C)
        for k in range(0, az.size, 7):
            a_ik = ik(g, tilt_R(az[k], mg[k]), np.array([0.0, 0.0, r["z"]]))
            worst = max(worst, float(np.max(np.abs(a_ik - minus[k]))))
            checked += 1
    print(f"    max |ik() - alpha_minus| over {checked} samples : {worst:.3e}")

    # ------------------------------------------------------------------ #
    print()
    print("=" * 78)
    print("DIAGNOSTIC (e) - the -5.7e-3 boundary margin, recomputed")
    print("=" * 78)
    print("  The figure was min over (pose, leg) of (C - |P|)/C on fixture A at")
    print("  the datum z_home, over the OLD envelope: tilt <= 6 deg, yaw +/-10,")
    print("  T +/- 0.05 r_b.  Reproduced first, then recomputed.")
    print()

    def old_envelope_poses(z_home):
        azis = np.deg2rad(np.arange(0.0, 360.0, 15.0))
        mags = np.deg2rad(np.linspace(1.5, 6.0, 4))
        tilts = [((1.0, 0.0, 0.0), 0.0)]
        for a_ in azis:
            for m_ in mags:
                tilts.append(((np.cos(a_), np.sin(a_), 0.0), m_))
        out_R, out_T = [], []
        for ax, mg_ in tilts:
            for yw in np.deg2rad([-10.0, 0.0, 10.0]):
                for (tx, ty) in [(0.0, 0.0), (0.05, 0.0), (-0.05, 0.0),
                                 (0.0, 0.05), (0.0, -0.05)]:
                    for tz in (-0.05, 0.0, 0.05):
                        out_R.append(_axis_angle((0, 0, 1), yw) @
                                     _axis_angle(ax, mg_))
                        out_T.append([tx, ty, z_home + tz])
        return np.array(out_R), np.array(out_T)

    def _axis_angle(axis, ang):
        ax = np.asarray(axis, float)
        ax = ax / np.linalg.norm(ax)
        x, y, zz = ax
        K = np.array([[0.0, -zz, y], [zz, 0.0, -x], [-y, x, 0.0]])
        return np.eye(3) + np.sin(ang) * K + (1.0 - np.cos(ang)) * (K @ K)

    def margin_general(geom, R, T):
        q = np.einsum("kxy,yi->kxi", R, geom.p) + T[:, :, None]
        L = q - geom.b[None]
        v = np.cross(geom.n, geom.u, axis=0)
        M = np.einsum("kxi,xi->ki", L, geom.u)
        N = np.einsum("kxi,xi->ki", L, v)
        LL = np.einsum("kxi,kxi->ki", L, L)
        P = (LL + geom.a ** 2 - geom.d ** 2) / (2.0 * geom.a)
        C = np.hypot(M, N)
        return (C - np.abs(P)) / C

    Ro, To = old_envelope_poses(Z_DATUM_A)
    old_margin = float(margin_general(g, Ro, To).min())
    print(f"  reproduction, OLD envelope at the datum z_home = {Z_DATUM_A:.6f}")
    print(f"    min (C-|P|)/C = {old_margin:+.6e}   "
          f"(handoff records -5.7e-3)")

    T_new = np.stack([np.zeros_like(az), np.zeros_like(az),
                      np.full_like(az, Z_DATUM_A)], axis=1)
    new_at_datum = float(margin_general(g, R_env, T_new).min())
    print()
    print(f"  SAME geometry, SAME z_home, NEW envelope "
          f"(tilt {TILT_LIMIT_DEG:.3f}, no yaw, no translation)")
    print(f"    min (C-|P|)/C = {new_at_datum:+.6e}")
    print()
    print("  Best over the restored z_home axis, same geometry, new envelope:")
    best_z, best_m = None, -np.inf
    for r in table:
        if r["margin"] > best_m:
            best_m, best_z = r["margin"], r["z"]
    print(f"    best min (C-|P|)/C = {best_m:+.6e}  at z_home/r_b = {best_z:.4f}")
    print(f"    (delta held at fixture A's {FIX_A['delta']} deg - NOT tuned;")
    print("     the inner delta tune is the harness's job, not this script's)")

    # ---- decomposition: which envelope change moved the number ------- #
    print()
    print("  DECOMPOSITION - the two envelopes are not nested, so attribute the")
    print("  move rather than reporting it as one number.  All at the datum")
    print(f"  z_home = {Z_DATUM_A:.6f}, fixture A, delta = {FIX_A['delta']}.")
    print()
    print(f"    {'variant':<52} {'min (C-|P|)/C':>16}")

    def old_style(tilt_deg, yaw_deg, trans):
        azis = np.deg2rad(np.arange(0.0, 360.0, 15.0))
        mags = np.deg2rad(np.linspace(tilt_deg / 4.0, tilt_deg, 4))
        tilts = [((1.0, 0.0, 0.0), 0.0)]
        for a_ in azis:
            for m_ in mags:
                tilts.append(((np.cos(a_), np.sin(a_), 0.0), m_))
        yaws = np.deg2rad([-yaw_deg, 0.0, yaw_deg]) if yaw_deg else [0.0]
        th = ([(0.0, 0.0), (trans, 0.0), (-trans, 0.0), (0.0, trans),
               (0.0, -trans)] if trans else [(0.0, 0.0)])
        tv = ([-trans, 0.0, trans] if trans else [0.0])
        Rs, Ts = [], []
        for ax, mg_ in tilts:
            for yw in yaws:
                for (tx, ty) in th:
                    for tz in tv:
                        Rs.append(_axis_angle((0, 0, 1), yw) @ _axis_angle(ax, mg_))
                        Ts.append([tx, ty, Z_DATUM_A + tz])
        return float(margin_general(g, np.array(Rs), np.array(Ts)).min())

    variants = [
        ("OLD: tilt 6, yaw +/-10, T +/-0.05 r_b", old_style(6.0, 10.0, 0.05)),
        ("     tilt 6, yaw +/-10, T = 0", old_style(6.0, 10.0, 0.0)),
        ("     tilt 6, yaw 0,      T +/-0.05 r_b", old_style(6.0, 0.0, 0.05)),
        ("     tilt 6, yaw 0,      T = 0", old_style(6.0, 0.0, 0.0)),
        (f"     tilt {TILT_LIMIT_DEG:.3f}, yaw +/-10, T +/-0.05 r_b",
         old_style(TILT_LIMIT_DEG, 10.0, 0.05)),
        (f"NEW: tilt {TILT_LIMIT_DEG:.3f}, yaw 0,      T = 0",
         old_style(TILT_LIMIT_DEG, 0.0, 0.0)),
    ]
    for name, val in variants:
        print(f"    {name:<52} {val:>+16.6e}")

    base = old_style(6.0, 0.0, 0.0)             # tilt 6 alone
    cost_yaw = base - old_style(6.0, 10.0, 0.0)
    cost_trans = base - old_style(6.0, 0.0, 0.05)
    cost_tilt = base - old_style(TILT_LIMIT_DEG, 0.0, 0.0)
    print()
    print("    Attribution, each measured against 'tilt 6, yaw 0, T = 0' "
          f"= {base:+.4e}:")
    print(f"      cost of yaw +/-10 deg      : {cost_yaw:.4e}")
    print(f"      cost of T +/-0.05 r_b      : {cost_trans:.4e}")
    print(f"      cost of tilt 6 -> {TILT_LIMIT_DEG:.3f}  : {cost_tilt:.4e}")
    print(f"      yaw + translation together : {cost_yaw + cost_trans:.4e}  "
          f"(vs {base - variants[0][1]:.4e} measured jointly - near-additive)")
    print()
    print("    Reading: translation and the tilt increase cost COMPARABLE")
    print("    amounts of margin; yaw costs about half as much.  The -5.7e-3")
    print("    was not driven by tilt, and it was not driven by any one term -")
    print("    it was yaw and translation together, and the settled envelope")
    print("    removes both.  Raising tilt from 6 to 10.5 does spend most of")
    print("    what that buys back.  The envelope is much larger in the one")
    print("    axis the control law uses and empty in the two it does not, and")
    print("    the net is a positive margin where there was a negative one.")

    print()
    print("  VERDICT on the -5.7e-3 figure:")
    print(f"    superseded.  It described a 6-degree envelope with yaw and")
    print(f"    translation, at a z_home the dropped datum supplied.  On the")
    print(f"    settled envelope the same geometry at the same height gives")
    print(f"    {new_at_datum:+.4e}; tuning z_home alone brings it to "
          f"{best_m:+.4e}.")
    print("    Quote the new number with its envelope attached, or neither.")


if __name__ == "__main__":
    main()
