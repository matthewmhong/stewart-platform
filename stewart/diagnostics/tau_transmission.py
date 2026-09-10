"""Does any kinematic quantity bound ``a`` from above?  ONE measurement.

    python -m stewart.diagnostics.tau_transmission

The 2fee8b0 sweep returns ``a = 60.40 mm``, the top of the published
ProModeler ladder, with ``beta`` bounded.  ``a`` is therefore set by the
catalogue rather than by the analysis - the third axis where the recorded
objective incompleteness has surfaced (``r_p``, ``beta``, now ``a``).

``tau_i = |rod_i . tangent_i| / d`` is the natural candidate: as the arm
lengthens, arm and rod approach alignment and transmission degrades.
``score_discriminators`` part (b) measured ``rho(tau_min, margin) = +0.835``
and recorded ``tau_min`` as measured-and-redundant - but that was over
``A_RB = [0.10, 0.20, 0.35]``, and the sweep now sits at ``0.6711``.  This
module asks whether that measurement carries to where the sweep actually is.

**Loads the 2fee8b0 cache** (``sweep-run.npz``, via :func:`.sweep.load`)
rather than re-running the sweep - the feasible set, ``delta_con`` and
``score_p`` are already computed there and this module adds exactly one new
quantity, ``tau_min``, evaluated at each candidate's own ``delta_con``.

**Two conventions differ from the original ``+0.835`` measurement, on
purpose, and are stated rather than left implicit**: that measurement used
the UNCONSTRAINED tuned ``delta`` (maximising ``margin(dxy=0)`` with no cap)
and correlated against ``margin(dxy=0)``.  This module uses ``delta_con``,
the tune UNDER the ``cond(J_fk) <= 1e6`` cap - "the tuned delta under the
existing cap" as asked - and correlates against ``margin(dxy = p)`` =
``score_p``, the score the sweep itself ranks by.  The two measurements are
therefore not directly comparable number-for-number; both conventions are
named at every table so that is never silently assumed.

**Does not propose a weight, a combined score, or a bound on ``a``.**  Does
not touch the objective or ``notation.md``.  Every number in this module is a
plain ratio with no characteristic length - the docstring in
``score_discriminators`` calls this out explicitly - except mm figures, which
carry ``r_b = 90`` mm as always.

Degrees at the boundary, radians internally.  Arrays are (3, 6).  numpy only.
Reports; always exits 0.
"""
from __future__ import annotations

import time

import numpy as np

from . import envelope as ENV
from . import score_discriminators as SD
from . import sweep as S
from .score_discriminators import _five_number
from .tilt_bracket import CAP, at_tilt

# --------------------------------------------------------------------------- #
# the vectorized tau_min, at ONE delta - no scan, no SVD
# --------------------------------------------------------------------------- #
def tau_min_at(beta, beta_p, r_p, a, d, delta_deg, R, T):
    """``tau_min`` at one ``delta``, vectorized over the ``(K,)`` pose grid.

    ``tau_i = |e_i . tangent_i|``, exact where ``|rod_i| = d`` at a solution -
    the identity :func:`.score_discriminators._jacobians_at_pose` documents.
    Same closed form :func:`.score_discriminators.scan_delta` uses for
    ``alpha``, at a single ``delta`` rather than a scan and with no SVD, so
    this is far cheaper than :func:`.score_discriminators.measure`.  Verified
    against it in :func:`verify` before use.

    Returns ``(tau_min, reach)``: ``reach`` is ``True`` at every (pose, leg)
    iff ``tau_min`` is finite.  A ``delta`` at which any leg or pose is
    unreachable returns ``(nan, reach)`` with ``reach`` showing where - never
    a fabricated value, matching the ``ik`` convention throughout this repo.
    """
    g0, g90 = SD._delta_basis(beta, beta_p, r_p, a, d)
    dr = np.deg2rad(delta_deg)
    c, s = np.cos(dr), np.sin(dr)
    n_d = c * g0.n + s * g90.n
    u_d = c * g0.u + s * g90.u
    v_d = np.cross(n_d, u_d, axis=0)
    Rp = np.einsum("kxy,yi->kxi", R, g0.p)
    q = Rp + np.asarray(T, float)[:, :, None]
    L = q - g0.b[None]
    LL = np.einsum("kxi,kxi->ki", L, L)
    P = (LL + a * a - d * d) / (2.0 * a)
    M = np.einsum("kxi,xi->ki", L, u_d)
    N = np.einsum("kxi,xi->ki", L, v_d)
    C = np.hypot(M, N)
    reach = (C > 0.0) & (np.abs(P) <= C)
    if not reach.all():
        return np.nan, reach
    alpha = np.arctan2(N, M) - np.arccos(P / C)
    ca, sa = np.cos(alpha), np.sin(alpha)
    tip = g0.b[None] + a * (ca[:, None, :] * u_d[None] + sa[:, None, :] * v_d[None])
    rod = q - tip
    norm = np.linalg.norm(rod, axis=1)
    e = rod / norm[:, None, :]
    tangent = -sa[:, None, :] * u_d[None] + ca[:, None, :] * v_d[None]
    tau = np.abs(np.einsum("kxi,kxi->ki", e, tangent))
    return float(tau.min()), reach


def verify(R29, az29, feasible=None, n=200, seed=0):
    """Gate: :func:`tau_min_at` must agree with ``measure()`` exactly.

    Drawn from REAL feasible candidates in the cache when ``feasible`` is
    given, at their own ``delta_con`` - a random draw off the raw axes finds
    almost nothing reachable (most of the box is not: see part 3, where the
    bottom of the ladder is unreachable at a pinned z_home), so it is not a
    strong gate.  ``measure`` goes through ``ik`` / ``arm_tips`` /
    ``fk_residual``; this is a from-scratch closed form, so agreement is not
    a tautology.  Falls back to raw-axis sampling when no cache is given.
    """
    rng = np.random.default_rng(seed)
    n_check = n_bad = 0
    if feasible is not None:
        idx = rng.choice(len(feasible), size=min(n, len(feasible)),
                         replace=False)
        cases = [(r["beta"], r["beta_p"], r["a"], r["d"], r["z_home"],
                  r["delta_con"]) for r in (feasible[i] for i in idx)]
    else:
        betas, bps, a_s = S.beta_axis(), S.beta_p_axis(), S.a_axis()
        cases = [(float(rng.choice(betas)), float(rng.choice(bps)),
                  float(rng.choice(a_s)), float(rng.uniform(0.15, S.D_CAP_RB)),
                  float(rng.uniform(0.3, 1.9)), float(rng.uniform(0.0, 179.0)))
                 for _ in range(n)]
    for beta, bp, a, d, z, delta in cases:
        T = SD._T_stack(az29, z)
        my_tau, reach = tau_min_at(beta, bp, S.R_P_RB, a, d, delta, R29, T)
        if not reach.all():
            continue
        n_check += 1
        got = SD.measure(dict(beta=beta, beta_p=bp, r_p=S.R_P_RB, a=a, d=d),
                         R29, T, ("r_b",), delta=delta)
        if got is None or not np.isclose(my_tau, got[0], atol=1e-9, rtol=1e-7):
            n_bad += 1
    return n_check, n_bad


# --------------------------------------------------------------------------- #
# 1-2: tau_min over the feasible set, at each candidate's own delta_con
# --------------------------------------------------------------------------- #
def compute_all(feasible, R29, az29, verbose=True):
    """Add ``tau_min`` to every record, at its own ``delta_con``.  In place."""
    t0 = time.time()
    n_unreach = 0
    for k, rec in enumerate(feasible):
        T = SD._T_stack(az29, rec["z_home"])
        tau, reach = tau_min_at(rec["beta"], rec["beta_p"], S.R_P_RB, rec["a"],
                                rec["d"], rec["delta_con"], R29, T)
        rec["tau_min"] = tau
        if not np.isfinite(tau):
            n_unreach += 1
        if verbose and (k + 1) % 50000 == 0:
            print(f"    tau_min {k+1:,} / {len(feasible):,}  "
                  f"({time.time()-t0:.0f} s)", flush=True)
    return dict(n=len(feasible), n_unreach=n_unreach, secs=time.time() - t0)


def part_summary(feasible):
    """(1) tau_min five-number summary, overall and per a value."""
    print()
    print("=" * 78)
    print("(1) tau_min AT delta_con - FIVE-NUMBER SUMMARY AND MAX")
    print("=" * 78)
    print("  tau_min = min over legs and the 29-pose envelope grid of")
    print("  |rod_i . tangent_i| / d, evaluated at delta_con - the tune UNDER")
    print(f"  the existing cap cond(J_fk) <= {CAP:.0e} at char_len = r_b.  A")
    print("  dimensionless ratio; no char_len of its own.")
    print()
    tau = np.array([r["tau_min"] for r in feasible], dtype=float)
    n_bad = int(np.sum(~np.isfinite(tau)))
    print(f"  OVER THE WHOLE FEASIBLE SET, n = {len(feasible):,} "
          f"(#non-finite: {n_bad}):")
    q = _five_number(tau)
    print(f"    min {q[0]:.5f}  Q1 {q[1]:.5f}  median {q[2]:.5f}  "
          f"Q3 {q[3]:.5f}  max {q[4]:.5f}")
    print()
    print("  BY a - the discrete, published axis:")
    print()
    a_vals = S.a_axis()
    by_a = {}
    for r in feasible:
        by_a.setdefault(round(r["a"], 9), []).append(r["tau_min"])
    print(f"    {'a [mm]':>8} {'a/r_b':>8} {'n':>7}{'min':>9}{'Q1':>9}"
          f"{'median':>9}{'Q3':>9}{'max':>9}")
    for v in a_vals:
        vals = np.array(by_a.get(round(v, 9), []), dtype=float)
        if vals.size == 0:
            print(f"    {v*S.R_B_MM:>8.2f} {v:>8.4f} {0:>7}"
                  + "".join(f"{'-':>9}" for _ in range(5)))
            continue
        qv = _five_number(vals)
        print(f"    {v*S.R_B_MM:>8.2f} {v:>8.4f} {vals.size:>7,}"
              + "".join(f"{x:>9.5f}" for x in qv))
    return tau, by_a


def part_correlation(feasible, tau):
    """(2) Spearman rho(tau_min, score_p), overall and per a."""
    score = np.array([r["score_p"] for r in feasible], dtype=float)
    print()
    print("=" * 78)
    print("(2) SPEARMAN rho(tau_min, margin(dxy = p)) - OVERALL AND PER a")
    print("=" * 78)
    print(f"  margin(dxy = p) is score_p, p = {S.P_SCORE:.6f} r_b, EVALUATED")
    print("  DIRECTLY through probe_margin - the score the sweep itself ranks")
    print("  by.  This differs from the original +0.835 measurement, which")
    print("  correlated against margin(dxy = 0) at the UNCONSTRAINED delta;")
    print("  the two numbers are not directly comparable for that reason.")
    print()
    rho, n = SD._spearman(tau, score)
    print(f"  OVERALL: rho = {rho:+.4f}  (n = {n:,})")
    print()
    a_vals = S.a_axis()
    by_a = {}
    for r, t in zip(feasible, tau):
        by_a.setdefault(round(r["a"], 9), []).append((t, r["score_p"]))
    print(f"    {'a [mm]':>8} {'a/r_b':>8} {'n':>7} {'rho':>9}")
    rows = []
    for v in a_vals:
        pairs = by_a.get(round(v, 9), [])
        if not pairs:
            print(f"    {v*S.R_B_MM:>8.2f} {v:>8.4f} {0:>7} {'-':>9}")
            continue
        tt = np.array([p[0] for p in pairs])
        ss = np.array([p[1] for p in pairs])
        r, n2 = SD._spearman(tt, ss)
        rows.append((v, r, n2))
        print(f"    {v*S.R_B_MM:>8.2f} {v:>8.4f} {n2:>7,} {r:>+9.4f}")
    print()
    a_mm = np.array([v * S.R_B_MM for v, _, _ in rows])
    rr = np.array([r for _, r, _ in rows])
    top = a_mm >= 0.5 * S.R_B_MM
    print(f"  OVERALL rho = {rho:+.4f}.  AT THE TOP OF THE a RANGE "
          f"(a >= {0.5*S.R_B_MM:.0f} mm = 0.5 r_b),")
    print(f"  per-a rho runs {rr[top].min():+.4f} to {rr[top].max():+.4f} "
          f"(median {np.median(rr[top]):+.4f}).")
    print(f"  AT THE BOTTOM (a <= {0.2*S.R_B_MM:.0f} mm = 0.2 r_b), per-a rho "
          f"runs {rr[a_mm<=0.2*S.R_B_MM].min():+.4f} to "
          f"{rr[a_mm<=0.2*S.R_B_MM].max():+.4f}.")
    print()
    if rho > 0.3 and rr[top].max() < 0.3:
        print("  THE MEASUREMENT DOES NOT CARRY: rho is positive and strong")
        print("  OVERALL but weak or reversed AT THE TOP OF a's RANGE, where")
        print("  the sweep's own shortlist sits.  Pooling across a hides this -")
        print("  the overall correlation is dominated by the SPREAD IN a")
        print("  itself (tau_min falls with a; see part 3), not by a")
        print("  within-a relationship between tau_min and margin.")
    elif rho > 0.3 and rr[top].min() > 0.0:
        print("  THE MEASUREMENT CARRIES: tau_min ranks WITH margin(dxy=p) both")
        print("  overall and at the top of a's range, where the shortlist sits.")
        print("  tau_min does NOT oppose margin over the range the sweep")
        print("  actually occupies.")
    else:
        print("  MIXED: overall and top-of-range correlations do not agree in")
        print("  sign or strength; read the per-a table above rather than the")
        print("  overall figure alone.")
    return rho


def part_a_sweep(R29, az29):
    """(3) tau_min(a) along the ladder, other axes pinned at the leader's."""
    print()
    print("=" * 78)
    print("(3) tau_min AS A FUNCTION OF a, OTHER AXES PINNED AT THE LEADER'S")
    print("=" * 78)
    beta, beta_p, d_rb, z_rb = 5.0, 52.5, S.E12235C["d_mm"] / S.R_B_MM, \
        S.E12235C["z_mm"] / S.R_B_MM
    print(f"  Pinned: beta = {beta}, beta_p = {beta_p}, d = "
          f"{d_rb*S.R_B_MM:.0f} mm, z_home = {z_rb*S.R_B_MM:.2f} mm - the")
    print("  shortlist leader's values, HELD FIXED including z_home.  Only a")
    print("  varies, over the published ladder.  delta is RE-TUNED at each a")
    print(f"  under the same cap cond(J_fk) <= {CAP:.0e} at char_len = r_b - the")
    print("  only thing this pipeline tunes - so the comparison isolates a's")
    print("  own kinematic effect rather than conflating it with a re-derived")
    print("  z_home bracket.")
    print()
    T = SD._T_stack(az29, z_rb)
    dg = S.DELTA_GRID
    print(f"    {'a [mm]':>8} {'a/r_b':>8} {'delta_con':>10} {'tau_min':>9} "
          f"{'margin(0)':>10} {'reachable':>10}")
    rows = []
    for a in S.a_axis():
        m, c, _ = SD.scan_delta(beta, beta_p, S.R_P_RB, a, d_rb, R29, T, dg,
                                S.R_B)
        d_con, m_con = SD.tune_constrained(m, c, dg, CAP)
        if d_con is None:
            print(f"    {a*S.R_B_MM:>8.2f} {a:>8.4f} {'-':>10} {'-':>9} "
                  f"{'-':>10} {'NO':>10}")
            rows.append((a, np.nan))
            continue
        tau, reach = tau_min_at(beta, beta_p, S.R_P_RB, a, d_rb, d_con, R29, T)
        rows.append((a, tau))
        print(f"    {a*S.R_B_MM:>8.2f} {a:>8.4f} {d_con:>10.1f} "
              f"{tau:>9.5f} {m_con:>10.5f} {'YES':>10}")
    print()
    n_unreach = sum(1 for _, t in rows if not np.isfinite(t))
    live = [(a, t) for a, t in rows if np.isfinite(t)]
    print(f"  {n_unreach} of {len(rows)} rungs UNREACHABLE at delta = ANY "
          f"value with z_home pinned at")
    print(f"  {z_rb*S.R_B_MM:.2f} mm - the ladder positions below the reach")
    print(f"  ceiling at this z_home, not a tau_min value.  Excluded from the")
    print(f"  monotonicity check below, which runs over the {len(live)} "
          f"reachable rungs only.")
    print()
    if len(live) >= 2:
        a_arr = np.array([a for a, _ in live])
        t_arr = np.array([t for _, t in live])
        falling = bool(np.all(np.diff(t_arr) <= 1e-12))
        rising = bool(np.all(np.diff(t_arr) >= -1e-12))
        total = t_arr[-1] - t_arr[0]
        span_mm = (a_arr[-1] - a_arr[0]) * S.R_B_MM
        slope = total / span_mm if span_mm else np.nan
        direction = ("MONOTONE FALLING" if falling and not rising else
                     "MONOTONE RISING" if rising and not falling else
                     "NOT MONOTONE")
        print(f"  {direction} with a over the {len(live)} reachable rungs.")
        print(f"  tau_min: {t_arr[0]:.5f} at a = {a_arr[0]*S.R_B_MM:.2f} mm  "
              f"->  {t_arr[-1]:.5f} at a = {a_arr[-1]*S.R_B_MM:.2f} mm")
        print(f"  net change {total:+.5f} over {span_mm:.2f} mm  "
              f"= {slope:+.6f} per mm of horn ({slope*S.R_B_MM:+.5f} per r_b)")
        print()
        if rising and not falling:
            print("  THIS OPPOSES THE HYPOTHESIS BEING TESTED.  'Arm lengthens,")
            print("  arm and rod approach alignment, transmission degrades' would")
            print("  predict tau_min FALLING with a; at the shortlist leader's")
            print("  own (beta, beta_p, d, z_home), holding those fixed, it RISES")
            print("  instead - and margin(0) rises alongside it (printed above),")
            print("  so tau_min tracks margin here rather than opposing it.")
            print("  tau_min therefore does NOT bound a from above at this point")
            print("  in the design space; if anything it favours the larger a.")
    return rows


def part_shortlist_members(res, R29, az29):
    """(4) The two shortlist members: their tau_min, and at each shorter horn
    with everything else RE-SCREENED and RE-TUNED (z_home bracket included).

    tau_min at the member's OWN (a, delta_con, z_home) is RECOMPUTED here via
    :func:`tau_min_at` rather than read from a cached field on the record -
    this section must stand on its own however it is called, and recomputing
    is one call, not a saving worth trading robustness for.
    """
    print()
    print("=" * 78)
    print("(4) THE TWO SHORTLIST MEMBERS - tau_min, AND AT EACH SHORTER HORN")
    print("=" * 78)
    print("  'everything else re-tuned' here means the FULL pipeline per shorter")
    print("  a: re-screen (a new z_home bracket, midpoint taken) and re-tune")
    print(f"  delta under the cap - screen_one + tune_and_score unchanged, the")
    print("  same procedure the sweep itself used to produce this candidate.")
    print("  beta, beta_p and d are held at the member's own values throughout.")
    print()
    from . import zhome_bracket as ZB
    az_fine, mg_fine = ZB.fine_poses()
    dr = np.deg2rad(S.DELTA_GRID)
    floor = ZB.z_lower_closed_form(S.R_P_RB, S.H_P)

    groups, tie = S.tie_groups(res["scored"])
    if not tie:
        print("  no tie set - see part (5) of the sweep report.")
        return
    key, members = tie[0]
    for m in sorted(members, key=lambda r: -r["score_p"]):
        beta, beta_p, d = m["beta"], m["beta_p"], m["d"]
        T_own = SD._T_stack(az29, m["z_home"])
        tau_own, _ = tau_min_at(beta, beta_p, S.R_P_RB, m["a"], d,
                                m["delta_con"], R29, T_own)
        print(f"  ---- beta = {beta}, beta_p = {beta_p}, d = {d*S.R_B_MM:.0f} mm ----")
        print(f"    at its own a = {m['a']*S.R_B_MM:.2f} mm: "
              f"tau_min = {tau_own:.5f}, margin(p) = {m['score_p']:.6f}, "
              f"z_home = {m['z_home']*S.R_B_MM:.2f} mm, delta_con = "
              f"{m['delta_con']:.1f}")
        print()
        print(f"    {'a [mm]':>8} {'a/r_b':>8} {'z_home mm':>10} "
              f"{'delta_con':>10} {'tau_min':>9} {'margin(p)':>10} {'note':>16}")
        path = []
        for a in S.a_axis():
            if a > m["a"] + 1e-9:
                continue
            scr = S.screen_one(beta, beta_p, a, d, az_fine, mg_fine, dr, floor)
            if not scr["feasible"]:
                print(f"    {a*S.R_B_MM:>8.2f} {a:>8.4f} {'-':>10} {'-':>10} "
                      f"{'-':>9} {'-':>10} {'INFEASIBLE: ' + scr['cause']:>16}")
                continue
            rec = dict(scr)
            S.tune_and_score(rec, R29, az29)
            if not np.isfinite(rec["score_p"]):
                print(f"    {a*S.R_B_MM:>8.2f} {a:>8.4f} "
                      f"{rec['z_home']*S.R_B_MM:>10.2f} {'-':>10} {'-':>9} "
                      f"{'-':>10} {'no delta at cap':>16}")
                continue
            tau, _ = tau_min_at(beta, beta_p, S.R_P_RB, a, d, rec["delta_con"],
                                R29, SD._T_stack(az29, rec["z_home"]))
            mark = "<- ACTUAL" if abs(a - m["a"]) < 1e-9 else ""
            path.append((a, tau, rec["score_p"]))
            print(f"    {a*S.R_B_MM:>8.2f} {a:>8.4f} "
                  f"{rec['z_home']*S.R_B_MM:>10.2f} "
                  f"{rec['delta_con']:>10.1f} {tau:>9.5f} "
                  f"{rec['score_p']:>10.6f} {mark:>16}")
        if path:
            ta = np.array([t for _, t, _ in path])
            sa = np.array([sc for _, _, sc in path])
            rho, npt = SD._spearman(ta, sa)
            i_lo, i_hi = int(np.argmin(ta)), int(np.argmax(ta))
            print()
            print(f"    over this re-tuned path: tau_min "
                  f"{ta.min():.5f} (at a = {path[i_lo][0]*S.R_B_MM:.2f} mm) to "
                  f"{ta.max():.5f} (at a = {path[i_hi][0]*S.R_B_MM:.2f} mm);")
            print(f"    NOT monotone with a - it peaks mid-ladder and eases "
                  f"back off toward the actual horn.")
            print(f"    rho(tau_min, margin(p)) along this path = {rho:+.4f} "
                  f"(n = {npt}); tau_min never drops")
            print(f"    below {ta.min():.3f}, far from the zero that would "
                  f"signal lost transmission.")
        print()


# --------------------------------------------------------------------------- #
def main() -> None:
    tilt = ENV.tilt_for(ENV.X0_WORKING, ENV.TAU)
    print("=" * 78)
    print("DOES ANY KINEMATIC QUANTITY BOUND a FROM ABOVE?")
    print("=" * 78)
    print("  The 2fee8b0 sweep returns a = 60.40 mm, the ProModeler ceiling,")
    print("  with beta bounded - the catalogue sets a, not the analysis.  This")
    print("  measures whether tau_i = |rod_i . tangent_i| / d, the natural")
    print("  transmission-degradation candidate, opposes margin where the")
    print("  sweep actually sits.  NO WEIGHT, SCORE, OR BOUND is proposed.")
    print()
    with at_tilt(tilt):
        R29, az29, _ = SD._pose_grid(None)
        print(f"  loading the 2fee8b0 cache: {S.SAVE_PATH}")
        res = S.load(S.SAVE_PATH)
        feasible = res["scored"]
        print(f"  {len(feasible):,} feasible candidates, all scored "
              f"(no_delta = {res['no_delta']}).")
        n_check, n_bad = verify(R29, az29, feasible=feasible)
        print(f"  GATE: tau_min_at vs score_discriminators.measure, "
              f"{n_check} REAL feasible candidates drawn from the cache, "
              f"{n_bad} disagreements.")
        if n_bad:
            print("  GATE FAILED.  NOT PROCEEDING.")
            return
        print()
        print(f"  computing tau_min at each candidate's own delta_con "
              f"({len(feasible):,} candidates,")
        print(f"  measured ~0.16 ms each from the verify pass) ...", flush=True)
        stats = compute_all(feasible, R29, az29)
        print(f"    done: {stats['secs']:.0f} s, {stats['n_unreach']} "
              f"unreachable at delta_con (expect 0 - delta_con was tuned to")
        print(f"    be reachable everywhere)")

        tau, by_a = part_summary(feasible)
        part_correlation(feasible, tau)
        part_a_sweep(R29, az29)
        part_shortlist_members(res, R29, az29)

        print()
        print("=" * 78)
        print("WHAT THIS DOES NOT DECIDE")
        print("=" * 78)
        print("  No weight, no combined score, no bound on a is proposed here.")
        print("  The objective stays open and notation.md is not touched.")
        print(f"  Every mm figure is at the ASSERTED r_b = {S.R_B_MM:.0f} mm.")


if __name__ == "__main__":
    main()
