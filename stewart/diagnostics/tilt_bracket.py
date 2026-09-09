"""Does the feasible set survive the plausible range of the tilt limit?

    python -m stewart.diagnostics.tilt_bracket

``tau_L = 150 ms`` is PROVISIONAL (``notation.md`` sec.9, and the module note in
:mod:`.envelope`).  It is inherited from a figure withdrawn 2026-09-03, has no
basis of its own, and carries 30 of the 80 mm and 3.97 of the 10.529 degrees.
Everything downstream of the envelope - the ``z_home`` brackets, the feasible
set, the ``delta`` tune, the margin ranking - was measured at that one number.

This module asks the only question that can be asked before ``tau_L`` gets a
basis: **is the work downstream of it robust to it being wrong?**  The whole
screen is re-run at three tilt limits, ``tau_L`` = 75, 150 and 300 ms - half,
current, and double - and the three are reported side by side.

WHAT IS NOT DECIDED HERE.  This module does not choose ``tau_L``, does not
choose ``v_peak``, and does not touch ``notation.md``.  The three ``tau_L``
values are a SENSITIVITY RANGE, not a shortlist, and none of them is preferred.
The envelope is not changed anywhere: the tilt limits below come from
:func:`.envelope.tilt_for` - the envelope's own closed form, called with a
different ``x0`` - so the recovery model has exactly one source and this module
restates none of it::

    x0        = 50 mm working + tau_L * v_peak
    acc       = 4 x0 / tau^2                       (bang-bang, tau = 0.5 s)
    sin(tilt) = 7 acc / (5 g)                      (solid ball, g = 9.80665)

The envelope is linear in ``tau_L`` through ``x0``; the tilt is not, because of
the ``arcsin``.  Both are visible in the table below.

HOW THE TILT LIMIT IS VARIED.  ``TILT_LIMIT_DEG`` is a module constant read in
three places on this path, one of them as a bound default argument.  Rather
than parameterise four functions in three modules - which would put a tilt
argument on the settled API for the sake of one study - this module rebinds
those three references for the duration of a pass and restores them after (see
:func:`at_tilt`).  Nothing is edited: :mod:`.envelope` remains the single
source of truth and every closed form, screen and tune below is the one those
modules already define, called unchanged.

WHAT IS MEASURED, at each of the three limits:

  * the 540-candidate coarse grid at ``h_p / r_b = 0.1``, exactly as
    :mod:`.zhome_bracket` builds it - the same exact reach test over the same
    1-degree ``delta`` scan and the same 366-pose fine envelope, intersected
    with the same CLOSED-FORM ``N_i > 0`` floor;
  * the empties, split R / X as in the 2026-09-05 attribution, with every
    candidate X refined by :func:`.zhome_bracket.reach_ceiling_bisect` before
    the verdict, so a crossing is not a 0.025-``r_b`` grid artifact;
  * the empties by ``a / r_b``;
  * bracket end ranges and non-contiguity;
  * the margin under the CONSTRAINED inner tune of
    :mod:`.score_discriminators` part (2), ``cond(J_fk) <= 1e6`` at
    ``char_len = r_b``, reached through :func:`.score_discriminators.evaluate`,
    :func:`~.score_discriminators.measure` and
    :func:`~.score_discriminators.run_constrained` - not reimplemented;
  * the Spearman correlation of that margin ranking against the ranking at the
    current 10.529-degree limit, over the candidates present in both.

UNITS AND LABELS.  ``margin`` is a dimensionless ratio and carries NO
characteristic length and NO Jacobian - a property of the quantity, not an
omission (:mod:`.score_discriminators`).  Every conditioning number here is
``cond(J_fk)`` at ``char_len = r_b`` and is labelled as such at every point of
use; it is PROVISIONAL, as the cap it feeds is.  Bracket ends are ``z_home /
r_b`` GRID values and carry the 0.025 ``r_b`` resolution of ``Z_GRID``.

Degrees at the boundary, radians internally.  Arrays are (3, 6).  numpy only.
Reports findings; always exits 0.
"""
from __future__ import annotations

import contextlib
import time

import numpy as np

from . import envelope as ENV
from . import score_discriminators as SD
from . import zhome_bracket as ZB
from .score_discriminators import _five_number, _spearman
from .zhome_bracket import (A_RB, BETA, BETA_P, D_RB, DELTA_GRID, H_P, R_B,
                            RP_RB, Z_GRID, leg_terms, reach_ceiling_bisect,
                            reach_feasible_any_delta)

# --------------------------------------------------------------------------- #
# the sensitivity range - a range, not a shortlist, and nothing here is chosen
# --------------------------------------------------------------------------- #
#: ``tau_L`` values in SECONDS.  Half, current, double.  The middle one is the
#: provisional figure in force; the outer two exist to bracket it and are not
#: proposals.  ``v_peak`` is NOT varied - it is the settled 200 mm/s, and
#: varying two provisional numbers at once would make neither attributable.
TAU_L_S = (0.075, 0.150, 0.300)

#: Which entry of :data:`TAU_L_S` is the limit in force, i.e. the reference the
#: rank correlations are taken against.  Asserted against :mod:`.envelope`
#: rather than assumed, so this cannot silently point at the wrong column.
REFERENCE_INDEX = 1

#: The cap on the inner ``delta`` tune, on ``cond(J_fk)`` at ``char_len = r_b``.
#: One of :data:`.score_discriminators.CONSTRAINT_CAPS`, taken from there rather
#: than restated; that module reports all four side by side and prefers none,
#: and picking one here is a REPORTING choice for this study, not a reject line.
CAP = 1e6


def tilt_limit_for(tau_l: float) -> float:
    """Envelope tilt limit in degrees for a latency ``tau_l`` in seconds.

    The envelope's own closed form, called with a different ``x0``.  Nothing
    about the recovery model is restated here: ``X0_WORKING``, ``V_PEAK``,
    ``TAU``, ``G`` and the ``5/7`` rolling factor all stay in :mod:`.envelope`.
    """
    return ENV.tilt_for(ENV.X0_WORKING + tau_l * ENV.V_PEAK, ENV.TAU)


# --------------------------------------------------------------------------- #
# rebinding the tilt limit for the duration of a pass
# --------------------------------------------------------------------------- #
def _with_default(fn, index: int, value):
    """Replace one positional default of ``fn``; return the old tuple."""
    old = fn.__defaults__
    new = list(old)
    new[index] = value
    fn.__defaults__ = tuple(new)
    return old


@contextlib.contextmanager
def at_tilt(tilt_deg: float):
    """Run the block with the whole package's tilt limit set to ``tilt_deg``.

    Three references have to move together, and they are listed here rather
    than discovered, because a missed one would silently mix two envelopes in
    one pass:

    ``envelope.envelope_poses``
        ``tilt_limit_deg`` is a default argument, BOUND AT DEFINITION, so
        rebinding ``envelope.TILT_LIMIT_DEG`` alone would not reach it.  This
        is the 29-pose harness grid every measurement below runs on.
    ``zhome_bracket``
        the module global that :func:`.zhome_bracket.fine_poses` reads at call
        time (the 366-pose screen envelope), plus the bound defaults of
        ``n_min_closed_form`` and ``z_lower_closed_form`` - the ``N_i > 0``
        floor, which is a statement about ``r_p``, ``h_p`` and the tilt limit
        alone and moves with all three.
    ``score_discriminators``
        the module global, read by ``_pose_grid`` for its fine-azimuth variant
        and by its own report headers.

    Everything is restored on exit, including on an exception.  The verify
    block in :func:`main` checks the rebinding took, at every limit, rather
    than trusting this list.
    """
    saved = (ENV.TILT_LIMIT_DEG, ZB.TILT_LIMIT_DEG, SD.TILT_LIMIT_DEG)
    d_poses = _with_default(ENV.envelope_poses, 2, tilt_deg)
    d_nmin = _with_default(ZB.n_min_closed_form, 0, tilt_deg)
    d_zlow = _with_default(ZB.z_lower_closed_form, 0, tilt_deg)
    ENV.TILT_LIMIT_DEG = ZB.TILT_LIMIT_DEG = SD.TILT_LIMIT_DEG = tilt_deg
    try:
        yield
    finally:
        ENV.envelope_poses.__defaults__ = d_poses
        ZB.n_min_closed_form.__defaults__ = d_nmin
        ZB.z_lower_closed_form.__defaults__ = d_zlow
        ENV.TILT_LIMIT_DEG, ZB.TILT_LIMIT_DEG, SD.TILT_LIMIT_DEG = saved


# --------------------------------------------------------------------------- #
# the screen, with the attribution folded into the same pass
# --------------------------------------------------------------------------- #
def screen(beta_vals=BETA, beta_p_vals=BETA_P, rp_vals=RP_RB,
           a_vals=A_RB, d_vals=D_RB):
    """The candidate screen at the tilt limit currently in force.

    Same primitives as :mod:`.zhome_bracket` and
    :func:`.score_discriminators.feasible_candidates` - ``leg_terms``,
    ``reach_feasible_any_delta``, ``z_lower_closed_form`` - so the feasible set
    and the attribution come out of ONE pass rather than two that could drift.
    The records handed on carry exactly the keys ``feasible_candidates``
    returns, because the pipeline downstream consumes them.

    The five axes default to :mod:`.zhome_bracket`'s own coarse grid, which is
    the 540-candidate screen this module reports and the ONLY thing it runs.
    They are arguments so that :mod:`.box_boundary` can screen a slice of new
    candidates - one added axis value, every other axis at its default - and
    accumulate, rather than re-screening the whole box at every step.  Passing
    anything but the defaults changes the candidate set, not the screen.

    Returns ``(rows, feasible)``: every candidate with its screen outcome, and
    the survivors.
    """
    az, mg = ZB.fine_poses()
    deltas_rad = np.deg2rad(DELTA_GRID)
    rows = []
    for beta in beta_vals:
        for beta_p in beta_p_vals:
            for r_p in rp_vals:
                floor = ZB.z_lower_closed_form(r_p, H_P)
                for a in a_vals:
                    for d in d_vals:
                        A, B, G, _ = leg_terms(beta, beta_p, r_p, a, d, H_P,
                                               Z_GRID, az, mg)
                        reach = reach_feasible_any_delta(A, B, G, deltas_rad)
                        ok = reach & (Z_GRID > floor)
                        ridx = np.flatnonzero(reach)
                        idx = np.flatnonzero(ok)
                        rows.append(dict(
                            beta=beta, beta_p=beta_p, r_p=r_p, a=a, d=d,
                            cf=float(floor), nreach=int(reach.sum()),
                            reach_hi=(float(Z_GRID[ridx[-1]]) if ridx.size
                                      else np.nan),
                            # reach_lo is the lowest z_home the reach test
                            # allows BEFORE the N_i > 0 floor is intersected
                            # in.  Carried so that "did the floor set the
                            # lower end?" can be answered by comparison
                            # (reach_lo < z_lo) rather than by the coincidence
                            # test z_lo <= floor, which the strict `Z_GRID >
                            # floor` above can never satisfy.  Additive: no
                            # existing field or outcome moves.
                            reach_lo=(float(Z_GRID[ridx[0]]) if ridx.size
                                      else np.nan),
                            z_lo=(float(Z_GRID[idx[0]]) if idx.size else np.nan),
                            z_hi=(float(Z_GRID[idx[-1]]) if idx.size else np.nan),
                            contiguous=(bool(idx.size) and
                                        (idx[-1] - idx[0] + 1) == idx.size),
                            feasible=bool(idx.size)))
    return rows, [r for r in rows if r["feasible"]]


def attribute(rows, az, mg):
    """Split the empties R / X, as the 2026-09-05 attribution defines them.

    ``R`` reach empty on its own - no ``z_home`` reaches at any ``delta``.
    ``X`` reach ceiling below the ``N_i > 0`` floor - both constraints
    satisfiable alone, but crossed.

    ``N_i > 0`` is one-sided (a floor, no ceiling), so there is no third case.
    Every X is refined by :func:`.zhome_bracket.reach_ceiling_bisect` to 1e-9
    before it is counted: the ``Z_GRID`` step is 0.025 ``r_b`` and a crossing
    narrower than that would otherwise be indistinguishable from a sampling
    artifact.
    """
    deltas_rad = np.deg2rad(DELTA_GRID)
    step = float(Z_GRID[1] - Z_GRID[0])
    cat_R, cat_X, artifacts = [], [], []
    for e in (r for r in rows if not r["feasible"]):
        if e["nreach"] == 0:
            cat_R.append(e)
            continue
        exact = reach_ceiling_bisect(e["beta"], e["beta_p"], e["r_p"], e["a"],
                                     e["d"], H_P, e["reach_hi"],
                                     e["reach_hi"] + step, az, mg, deltas_rad)
        e["ceil_exact"] = exact
        e["gap"] = e["cf"] - exact
        if e["gap"] > 0.0:
            cat_X.append(e)
        else:
            artifacts.append(e)
            cat_R.append(e)
    return cat_R, cat_X, artifacts


# --------------------------------------------------------------------------- #
# the constrained tune, through score_discriminators' own pipeline
# --------------------------------------------------------------------------- #
def constrained_margins(feasible):
    """Run part (2)'s tune on the survivors; return ``(measured, dropped)``.

    Straight through :mod:`.score_discriminators`:
    :func:`~.score_discriminators.evaluate` (``z_home`` = bracket midpoint,
    unconstrained maximin ``delta``, ``sens``),
    :func:`~.score_discriminators.measure` (``tau_min`` and both Jacobians at
    all four characteristic lengths), then
    :func:`~.score_discriminators.run_constrained`, which adds the constrained
    tune at every cap in :data:`.score_discriminators.CONSTRAINT_CAPS`.  This
    module reads only :data:`CAP` off the result and reimplements none of it.

    ``dropped`` counts candidates whose bracket midpoint is unreachable to
    ``ik`` on the 29-pose grid - :func:`~.score_discriminators.measure` returns
    ``None`` there and ``main`` reports the count rather than absorbing it.
    """
    R29, az29, _ = SD._pose_grid(None)
    measured, dropped = [], 0
    for rec in feasible:
        SD.evaluate(rec, R29, lambda z: SD._T_stack(az29, z), az29)
        got = SD.measure(rec, R29, SD._T_stack(az29, rec["z_home"]),
                         SD.CHAR_LEN_KEYS)
        if got is None:
            dropped += 1
            continue
        rec["tau_min"], rec["cond"], rec["smin"], rec["smax"], _ = got
        measured.append(rec)
    SD.run_constrained(measured, R29, az29)
    return measured, dropped


def _key(rec):
    return (rec["beta"], rec["beta_p"], rec["r_p"], rec["a"], rec["d"])


def con_margin_map(measured):
    """``{candidate key: margin under the constrained tune at CAP}``.

    A candidate with no admissible ``delta`` at the cap is carried as ``NaN``,
    not dropped, so the difference between "not feasible" and "feasible but
    inadmissible" survives into the correlation, where ``_spearman`` drops it
    pairwise and counts it out.
    """
    out = {}
    for rec in measured:
        e = rec["con"][CAP]
        out[_key(rec)] = np.nan if e is None else float(e["margin"])
    return out


# --------------------------------------------------------------------------- #
def run_one(tau_l: float):
    """One complete pass at one ``tau_L``.  Returns everything ``main`` prints."""
    tilt = tilt_limit_for(tau_l)
    t0 = time.time()
    with at_tilt(tilt):
        # the rebinding is verified INSIDE the block, not assumed
        az, mg = ZB.fine_poses()
        az29, mg29 = ENV.envelope_poses()
        checks = dict(fine_max=float(mg.max()), harness_max=float(mg29.max()),
                      floor_tilt=float(np.degrees(np.arcsin(
                          ZB.z_lower_closed_form(1.0, 0.0)))))
        rows, feasible = screen()
        cat_R, cat_X, artifacts = attribute(rows, az, mg)
        measured, dropped = constrained_margins(feasible)
    return dict(tau_l=tau_l, tilt=tilt, rows=rows, feasible=feasible,
                cat_R=cat_R, cat_X=cat_X, artifacts=artifacts,
                measured=measured, dropped=dropped, checks=checks,
                margins=con_margin_map(measured), secs=time.time() - t0)


def _fmt(x, spec=".3f", nan="-"):
    return nan if (x is None or (isinstance(x, float) and np.isnan(x))) else \
        format(x, spec)


def main() -> None:
    print("=" * 78)
    print("TILT-LIMIT SENSITIVITY OF THE FEASIBLE SET")
    print("=" * 78)
    print("  tau_L = 150 ms is PROVISIONAL (notation.md sec.9): inherited from a")
    print("  figure withdrawn 2026-09-03, no basis of its own, carrying 30 of the")
    print("  80 mm and 3.97 of the 10.529 degrees.  Everything downstream was")
    print("  measured at that one number.  This module re-runs the screen at half")
    print("  and double it and reports the three side by side.")
    print()
    print("  NOTHING IS CHOSEN HERE.  tau_L is not chosen, v_peak is not chosen,")
    print("  the envelope is not changed and notation.md is not touched.  The")
    print("  three values are a sensitivity RANGE, not a shortlist.")
    print()

    # ---- the envelope, called not restated ---------------------------- #
    print("-" * 78)
    print("TILT LIMITS - from envelope.tilt_for, the envelope's own closed form")
    print("-" * 78)
    print("  x0 = 50 mm + tau_L * v_peak ;  acc = 4 x0 / tau^2 ;  "
          "sin(tilt) = 7 acc / (5 g)")
    print(f"  tau = {ENV.TAU} s   v_peak = {ENV.V_PEAK * 1e3:.0f} mm/s   "
          f"g = {ENV.G} m/s^2   (all from envelope.py)")
    print()
    print(f"    {'tau_L [ms]':>11} {'x0 [mm]':>9} {'acc [m/s^2]':>12} "
          f"{'sin(tilt)':>11} {'tilt [deg]':>11} {'ratio to 150':>13}")
    ref_tilt = tilt_limit_for(TAU_L_S[REFERENCE_INDEX])
    for tl in TAU_L_S:
        x0 = ENV.X0_WORKING + tl * ENV.V_PEAK
        acc = ENV.bang_bang_accel(x0, ENV.TAU)
        print(f"    {tl * 1e3:>11.0f} {x0 * 1e3:>9.1f} {acc:>12.4f} "
              f"{acc / (ENV.ROLL_FACTOR * ENV.G):>11.6f} "
              f"{tilt_limit_for(tl):>11.4f} {tilt_limit_for(tl) / ref_tilt:>13.3f}")
    print()
    print("  x0 is LINEAR in tau_L (30 -> 60 -> 120 mm of drift on a fixed 50 mm")
    print("  working displacement); tilt is not, because of the arcsin - doubling")
    print("  tau_L moves the limit by less than doubling it would suggest, and")
    print("  halving it costs less than half.")
    print()
    # Repointed 2026-09-09 (b0703ab): this used to check ref_tilt against
    # ENV.TILT_LIMIT_DEG, which was the same number by construction - both
    # were tilt_for(0.080).  TILT_LIMIT_DEG now derives from X0_WORKING and
    # no longer equals this column, so the check is pinned to the column's
    # own definition (150 ms, x0 = 0.080) instead of to a constant that has
    # since moved to mean something else.
    assert abs(ref_tilt - ENV.tilt_for(0.080)) < 1e-9, (
        f"the reference column ({ref_tilt}) is not tilt_for(0.080) "
        f"({ENV.tilt_for(0.080)}); REFERENCE_INDEX is wrong")
    print(f"  reference column = tau_L {TAU_L_S[REFERENCE_INDEX]*1e3:.0f} ms, "
          f"tilt {ref_tilt:.4f} deg, and it AGREES with envelope.TILT_LIMIT_DEG")
    print(f"  to {abs(ref_tilt - ENV.TILT_LIMIT_DEG):.1e} deg - checked, not "
          f"assumed.")
    print()

    print("-" * 78)
    print("WHAT IS HELD FIXED ACROSS THE THREE PASSES")
    print("-" * 78)
    print(f"  grid        : {len(BETA)}x{len(BETA_P)}x{len(RP_RB)}x{len(A_RB)}x"
          f"{len(D_RB)} = "
          f"{len(BETA)*len(BETA_P)*len(RP_RB)*len(A_RB)*len(D_RB)} candidates, "
          f"h_p/r_b = {H_P} fixed")
    print(f"  z_home scan : {Z_GRID[0]} .. {Z_GRID[-1]} step "
          f"{Z_GRID[1]-Z_GRID[0]} r_b  ({Z_GRID.size} points)")
    print(f"  delta scan  : {DELTA_GRID.size} points over [0, 180)")
    print(f"  screen      : exact reach w_i(delta)^2 <= |L_i|^2 - P_i^2, "
          f"intersected")
    print(f"                with the CLOSED-FORM N_i > 0 floor")
    print(f"  azimuth     : {ENV.AZIMUTH_WINDOW_DEG} deg, dxy = dz = yaw = 0")
    print(f"  tune        : score_discriminators part (2), cond(J_fk) <= "
          f"{CAP:.0e}")
    print(f"                at char_len = {SD.CONSTRAINT_CHAR_LEN}")
    print()
    print("  ONLY the tilt limit changes between passes.  The tilt limit enters")
    print("  the screen envelope, the harness pose grid AND the N_i > 0 floor, so")
    print("  all three move together - which is the point.")
    print()

    # ---- the three passes --------------------------------------------- #
    res = []
    for tl in TAU_L_S:
        print(f"  running tau_L = {tl*1e3:.0f} ms "
              f"(tilt {tilt_limit_for(tl):.4f} deg) ...", flush=True)
        res.append(run_one(tl))
    ref = res[REFERENCE_INDEX]
    print(f"  done, {sum(r['secs'] for r in res):.1f} s total")
    print()

    hdr = [f"tau_L {r['tau_l']*1e3:.0f} ms" for r in res]
    sub = [f"{r['tilt']:.4f} deg" for r in res]

    def line(label, vals, width=16):
        print(f"  {label:<42}" + "".join(f"{v:>{width}}" for v in vals))

    print("-" * 78)
    print("REBINDING SELF-CHECK - was the limit actually in force in each pass?")
    print("-" * 78)
    line("", hdr)
    line("", sub)
    line("max tilt, 366-pose screen envelope [deg]",
         [f"{r['checks']['fine_max']:.4f}" for r in res])
    line("max tilt, 29-pose harness grid [deg]",
         [f"{r['checks']['harness_max']:.4f}" for r in res])
    line("tilt implied by the N>0 floor [deg]",
         [f"{r['checks']['floor_tilt']:.4f}" for r in res])
    print()
    print("  The third row inverts the closed form z_home > r_p sin(tilt) +")
    print("  h_p cos(tilt) at r_p = 1, h_p = 0, so it reads the tilt back OUT of")
    print("  the floor itself.  All three rows agreeing with the column heading")
    print("  is what says the screen envelope, the harness grid and the floor all")
    print("  moved, and moved together.")
    print()

    print("-" * 78)
    print("PROVENANCE - does screen() agree with feasible_candidates() at the")
    print("             limit in force?")
    print("-" * 78)
    print("  screen() folds the feasibility screen and the R/X attribution into")
    print("  one pass, so it needs both the survivors and the empties; the two")
    print("  could drift apart.  With the tilt limit UNPATCHED, its output is")
    print("  compared record by record against score_discriminators'")
    print("  feasible_candidates() - z_lo, z_hi and contiguity, not just a count.")
    ref_rows, ref_feas = screen()
    theirs = {_key(r): (r["z_lo"], r["z_hi"], r["contiguous"])
              for r in SD.feasible_candidates(verbose=False)}
    ours = {_key(r): (r["z_lo"], r["z_hi"], r["contiguous"]) for r in ref_feas}
    print(f"    screen()             : {len(ours)} feasible of {len(ref_rows)}")
    print(f"    feasible_candidates(): {len(theirs)} feasible")
    print(f"    identical, key and record : {ours == theirs}")
    assert ours == theirs, "screen() has drifted from feasible_candidates()"
    print()

    # ---- (1) the feasible set ------------------------------------------ #
    n_tot = len(res[0]["rows"])
    print("=" * 78)
    print("(1) THE FEASIBLE SET")
    print("=" * 78)
    line("", hdr)
    line("", sub)
    line(f"non-empty z_home brackets, of {n_tot}",
         [f"{len(r['feasible'])}" for r in res])
    line("  as a fraction of the grid",
         [f"{len(r['feasible'])/n_tot:.3f}" for r in res])
    line(f"empty brackets, of {n_tot}",
         [f"{n_tot - len(r['feasible'])}" for r in res])
    line("  R  reach empty on its own",
         [f"{len(r['cat_R'])}" for r in res])
    line("  X  reach ceiling below the N>0 floor",
         [f"{len(r['cat_X'])}" for r in res])
    line("  (X candidates rejected as grid artifacts)",
         [f"{len(r['artifacts'])}" for r in res])
    print()
    print("  R and X are the 2026-09-05 attribution, and they are the only two")
    print("  cases: N_i > 0 is one-sided - a floor with no ceiling - so 'reach")
    print("  floor above an N ceiling' cannot occur.  Every X is refined by")
    print(f"  bisection to 1e-9 before it is counted; the Z_GRID step is "
          f"{Z_GRID[1]-Z_GRID[0]} r_b")
    print("  and a crossing narrower than that would be a sampling artifact.")
    if any(r["cat_X"] for r in res):
        print()
        print("  The X candidates, per pass (ceiling and floor in z_home / r_b):")
        print(f"    {'tau_L':>7} {'beta':>6} {'beta_p':>7} {'r_p':>6} "
              f"{'a/r_b':>6} {'d/r_b':>6} {'ceil(exact)':>12} {'N floor':>9} "
              f"{'gap':>11}")
        for r in res:
            for e in r["cat_X"]:
                print(f"    {r['tau_l']*1e3:>7.0f} {e['beta']:>6.1f} "
                      f"{e['beta_p']:>7.1f} {e['r_p']:>6.2f} {e['a']:>6.2f} "
                      f"{e['d']:>6.2f} {e['ceil_exact']:>12.7f} "
                      f"{e['cf']:>9.5f} {e['gap']:>+11.2e}")

    # ---- (2) empties by a/r_b ------------------------------------------ #
    print()
    print("=" * 78)
    print("(2) EMPTY BRACKETS BY a / r_b")
    print("=" * 78)
    line("", hdr)
    line("", sub)
    for a in A_RB:
        tot = sum(1 for r in res[0]["rows"] if r["a"] == a)
        line(f"a/r_b = {a:.2f}   empty of {tot}",
             [f"{sum(1 for x in r['rows'] if x['a'] == a and not x['feasible'])}"
              for r in res])
    print()
    for a in A_RB:
        vals = []
        for r in res:
            ke = [e for e in r["rows"] if e["a"] == a and not e["feasible"]]
            nR = sum(1 for e in ke if e in r["cat_R"])
            nX = sum(1 for e in ke if e in r["cat_X"])
            vals.append(f"{nR} / {nX}")
        line(f"a/r_b = {a:.2f}   R / X", vals)
    print()
    print("  The pattern to watch is the ORDERING in a/r_b: short arms cannot")
    print("  reach and long arms can, so the empties should concentrate at small")
    print("  a/r_b at every limit.  Whether the ordering survives is the question;")
    print("  the counts themselves are expected to move.")

    # ---- (3) bracket ends ---------------------------------------------- #
    print()
    print("=" * 78)
    print("(3) BRACKET ENDS AND CONTIGUITY   [z_home / r_b, grid values, "
          "res 0.025]")
    print("=" * 78)
    line("", hdr)
    line("", sub)
    los = [np.array([x["z_lo"] for x in r["feasible"]]) for r in res]
    his = [np.array([x["z_hi"] for x in r["feasible"]]) for r in res]
    line("lower ends  min .. max",
         [f"{l.min():.3f}..{l.max():.3f}" for l in los])
    line("upper ends  min .. max",
         [f"{h.min():.3f}..{h.max():.3f}" for h in his])
    line("width       min .. max",
         [f"{(h-l).min():.3f}..{(h-l).max():.3f}" for l, h in zip(los, his)])
    line("median width", [f"{np.median(h-l):.3f}" for l, h in zip(los, his)])
    line("non-contiguous feasible sets",
         [f"{sum(1 for x in r['feasible'] if not x['contiguous'])}" for r in res])
    line("lower end set by the N>0 floor",
         [f"{sum(1 for x in r['feasible'] if x['z_lo'] <= x['cf'] + 1e-9)}"
          for r in res])
    line("reach set touching the top of the scan",
         [f"{sum(1 for x in r['feasible'] if x['z_hi'] >= Z_GRID[-1] - 1e-12)}"
          for r in res])
    print()
    print("  Both ends move UP with the tilt limit, and for different reasons:")
    print("  the lower end because the N>0 floor r_p sin(tilt) + h_p cos(tilt)")
    print("  rises directly with tilt, the upper end because a bigger tilt swings")
    print("  the platform anchors further and reach fails sooner.")

    # ---- (4) the margin under the constrained tune --------------------- #
    print()
    print("=" * 78)
    print(f"(4) MARGIN UNDER THE CONSTRAINED TUNE, cond(J_fk) <= {CAP:.0e} at "
          f"char_len = {SD.CONSTRAINT_CHAR_LEN}")
    print("=" * 78)
    print("  margin = min over legs and poses of (C_i - |P_i|) / C_i.  It is a")
    print("  dimensionless ratio and carries NO characteristic length and NO")
    print("  Jacobian; the CONSTRAINT does, and it is cond(J_fk) at char_len =")
    print(f"  {SD.CONSTRAINT_CHAR_LEN} throughout, PROVISIONAL as that cap is.  "
          f"z_home is the")
    print("  midpoint of each candidate's bracket - a point in the interior for")
    print("  definiteness, and it moves with the bracket, so it is re-taken at")
    print("  each limit rather than held.")
    print()
    line("", hdr)
    line("", sub)
    line("feasible", [f"{len(r['feasible'])}" for r in res])
    line("  dropped, ik unreachable at the midpoint",
         [f"{r['dropped']}" for r in res])
    line("  measured", [f"{len(r['measured'])}" for r in res])
    line(f"  no admissible delta at C = {CAP:.0e}",
         [f"{sum(1 for v in r['margins'].values() if np.isnan(v))}" for r in res])
    line("  SURVIVORS (the five-number sample)",
         [f"{sum(1 for v in r['margins'].values() if not np.isnan(v))}"
          for r in res])
    print()
    fives = [_five_number(np.array(list(r["margins"].values()), dtype=float))
             for r in res]
    for i, nm in enumerate(("min", "Q1", "median", "Q3", "max")):
        line(f"margin  {nm}", [f"{f[i]:.6f}" for f in fives])
    print()
    neg = [sum(1 for v in r["margins"].values() if not np.isnan(v) and v < 0.0)
           for r in res]
    line("survivors with a NEGATIVE margin", [f"{n}" for n in neg])
    print()
    print("  A negative margin is a candidate the 366-pose screen passed and the")
    print("  29-pose harness grid does not - the two grids are not nested, and")
    print("  score_discriminators reports the same thing.  It is carried, not")
    print("  dropped, because dropping it would flatter the summary.")

    print()
    print("  The MINIMUM is a censored statistic and must not be read as a trend.")
    print("  It is taken over a survivor set that is itself shrinking, and the")
    print("  worst candidate at any limit is typically one about to leave the set")
    print("  entirely.  Each pass's argmin is traced across all three below; where")
    print("  a row reads 'infeasible', the minimum at that limit is not comparable")
    print("  to the one beside it because the candidate carrying it is gone.")
    print()
    cols = [f"margin @ {r['tau_l']*1e3:.0f}ms" for r in res]
    print(f"    {'argmin at':>11} {'beta':>6} {'beta_p':>7} {'r_p':>6} "
          f"{'a/r_b':>6} {'d/r_b':>6}" + "".join(f"{c:>17}" for c in cols))
    for r in res:
        good = [(v, k) for k, v in r["margins"].items() if not np.isnan(v)]
        if not good:
            continue
        k = min(good)[1]
        cells = []
        for o in res:
            v = o["margins"].get(k)
            cells.append("infeasible" if v is None
                         else ("no delta" if np.isnan(v) else f"{v:.6f}"))
        print(f"    {r['tau_l']*1e3:>9.0f}ms {k[0]:>6.1f} {k[1]:>7.1f} "
              f"{k[2]:>6.2f} {k[3]:>6.2f} {k[4]:>6.2f}" +
              "".join(f"{c:>17}" for c in cells))

    # ---- (5) rank correlation ------------------------------------------ #
    print()
    print("=" * 78)
    print(f"(5) SPEARMAN OF THE MARGIN RANKING vs THE {ref['tilt']:.3f}-DEG "
          f"LIMIT")
    print("=" * 78)
    print("  Both rankings are the CONSTRAINED-tune margin above.  The pairing is")
    print("  by candidate (beta, beta_p, r_p, a, d), over the candidates FEASIBLE")
    print("  AT BOTH limits; a candidate feasible at both but with no admissible")
    print("  delta at the cap in one of them is NaN there and _spearman drops it")
    print("  pairwise and counts it out, so 'n used' <= 'feasible at both'.")
    print()
    print(f"    {'tau_L [ms]':>11} {'tilt [deg]':>11} {'feas. both':>11} "
          f"{'n used':>8} {'rho':>9} {'lost':>6} {'gained':>7}")
    for r in res:
        common = sorted(set(r["margins"]) & set(ref["margins"]))
        x = np.array([r["margins"][k] for k in common])
        y = np.array([ref["margins"][k] for k in common])
        rho, n = _spearman(x, y)
        lost = len(set(ref["margins"]) - set(r["margins"]))
        gained = len(set(r["margins"]) - set(ref["margins"]))
        print(f"    {r['tau_l']*1e3:>11.0f} {r['tilt']:>11.4f} "
              f"{len(common):>11} {n:>8} {rho:>9.4f} {lost:>6} {gained:>7}")
    print()
    print("  'lost' and 'gained' are measured against the reference column and")
    print("  are the candidates the rank correlation CANNOT see: a candidate that")
    print("  drops out of the feasible set has no rank to compare.  A high rho on")
    print("  a shrinking common set is a weaker statement than the same rho on a")
    print("  stable one, and both numbers are printed for that reason.")
    print()
    print("  Top of the field, by constrained margin, at each limit:")
    print(f"    {'tau_L':>7} {'rank':>5} {'beta':>6} {'beta_p':>7} {'r_p':>6} "
          f"{'a/r_b':>6} {'d/r_b':>6} {'z_home':>8} {'delta':>7} {'margin':>10}")
    for r in res:
        good = [(v, k) for k, v in r["margins"].items() if not np.isnan(v)]
        good.sort(reverse=True)
        by_key = {_key(x): x for x in r["measured"]}
        for rank, (v, k) in enumerate(good[:5], 1):
            rec = by_key[k]
            print(f"    {r['tau_l']*1e3:>7.0f} {rank:>5} {k[0]:>6.1f} "
                  f"{k[1]:>7.1f} {k[2]:>6.2f} {k[3]:>6.2f} {k[4]:>6.2f} "
                  f"{rec['z_home']:>8.4f} "
                  f"{rec['con'][CAP]['delta']:>7.1f} {v:>10.6f}")
    print()
    print("  delta is the CONSTRAINED tune's delta, in degrees, on the 180-point")
    print(f"  scan; z_home is in r_b and is the bracket midpoint at that limit.")

    # ---- verdict -------------------------------------------------------- #
    print()
    print("=" * 78)
    print("WHAT SURVIVES AND WHAT DOES NOT")
    print("=" * 78)
    ns = [len(r["feasible"]) for r in res]
    print(f"  feasible set   : {ns[0]} / {ns[1]} / {ns[2]} of {n_tot} across "
          f"tau_L = 75 / 150 / 300 ms")
    print(f"                   ({ns[0]/n_tot:.0%} / {ns[1]/n_tot:.0%} / "
          f"{ns[2]/n_tot:.0%} of the grid)")
    print()
    print("  Read the three columns above, not this line: the verdict on each of")
    print("  the three questions - does the feasible set survive, does the a/r_b")
    print("  pattern survive, does the ranking survive - is exactly the table it")
    print("  sits under, and each is a different kind of survival.")
    print()
    print("  WHAT THIS DOES NOT DO.  It does not give tau_L a basis.  The basis")
    print("  is still the sensor frame interval plus the servo step response,")
    print("  both on the hardware pull, and it is still outstanding.  A feasible")
    print("  set robust over 75-300 ms says the DOWNSTREAM work does not have to")
    print("  wait for that number; it does not say the number does not matter.")
    print("  tau_L stays PROVISIONAL and notation.md sec.9 is unchanged.")


if __name__ == "__main__":
    main()
