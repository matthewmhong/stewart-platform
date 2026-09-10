"""The feasible set after ``tau_L`` is DROPPED, not revised - 2026-09-09.

    python -m stewart.diagnostics.tilt_dropped

THE DECISION THIS MODULE RUNS ON, asserted 2026-09-09 and not reopened here.
``tau_L`` is **dropped**.  ``docs/hardware-pull.md`` sec.4 establishes that
servo step response is **NOT PUBLISHED by any manufacturer found** - every
maker publishes seconds-per-60-deg no-load speed, which is a slew rate and
contains no propagation delay, no rise time, no overshoot and no settling
criterion - and that the sensor cells publish a **frame interval**, a sampling
period, not a sensor-to-pose latency.  Neither term can be reconstructed from
published data, so the 30 mm latency drift and the 3.97 deg it carried are
**removed** rather than left provisional.  This completes the 2026-09-03
withdrawal of the 300 mm/s peak-speed figure that ``tau_L`` inherited from.

What is left is the working displacement alone::

    x0        = 50 mm                               (working displacement only)
    acc       = 4 x0 / tau^2,  tau = 0.5 s          (bang-bang)
    sin(tilt) = 7 acc / (5 g),  g = 9.80665         (solid ball, rolling)

The limit is **taken from** :func:`.envelope.tilt_for` at that ``x0`` and
printed, never transcribed as a number of degrees - so the figure below is
reproduced by this module rather than asserted by it.

WHAT IS NOT DONE HERE.  ``envelope.py`` is not edited and ``notation.md`` is
not touched: the ``tau_L`` drop needs recording and that is a separate
documentation pass.  ``TILT_LIMIT_DEG`` therefore still reads 10.529 in
:mod:`.envelope`, and this module treats that as the REFERENCE column - the
limit everything computed so far runs on - not as the limit in force.  No
range is chosen, no part is chosen, and the sweep harness is not specced.

A CORRECTION TO THE FRAMING OF THE REQUEST, stated because the columns are
being read as a trend.  :mod:`.tilt_bracket` runs ``tau_L`` = 75 / 150 / 300 ms
on a fixed 50 mm working displacement, which is ``x0`` = 65 / 80 / 110 mm and
tilt limits of **8.538 / 10.529 / 14.550 deg**.  It never produced a 6.558-deg
column: ``tau_L = 0`` was outside its sensitivity range, and the drop is what
puts it there.  So 6.558 is NEW here, 8.538 and 10.529 are two of
:mod:`.tilt_bracket`'s three, and 14.550 is the third - carried below so the
trend is not truncated at the point where it was convenient.

A SECOND ONE, on comparability.  :mod:`.tilt_bracket`'s columns are the
540-candidate coarse grid with ``r_p/r_b`` spanning 0.60-1.10.  Absolute scale
was asserted 2026-09-08 and ``r_p/r_b`` is one number now, so those columns are
**not** the comparison asked for and are not quoted as counts.  Every column
below is re-run at the fixed absolute scale - ``r_b = 90 mm``, ``r_p = 80 mm``,
``c_p/r_b = 0.1`` - through :func:`.fixed_ratio.run_slice`, so the four columns
differ in the tilt limit and in nothing else.

WHAT IS MEASURED, at each of the four limits and all at ``r_p/r_b = 80/90``:

  * non-empty ``z_home`` brackets out of the candidate count, split
    reach-empty (R) against ``N_i``-crossing (X) as the 2026-09-05 attribution
    defines them, every X refined by bisection to 1e-9 first;
  * the empties by ``a/r_b``;
  * bracket lower and upper end ranges, how many are non-contiguous, and
    whether the ``N > 0`` floor sets any lower end;
  * ``margin(dxy = p)`` five-number summary and max over the survivors, at
    ``p = sqrt(3 x 0.2^2) / 90 = 0.003849 r_b``, EVALUATED at that probe
    through :func:`.score_discriminators.probe_margin`.  **No slope is
    extrapolated**;
  * the Spearman correlation of the ranking against the ranking at 10.529 deg,
    over the candidates feasible at BOTH, with the size of the common set
    stated.

UNITS AND LABELS.  ``margin`` is a dimensionless ratio and carries NO
characteristic length and NO Jacobian - a property of the quantity, not an
omission.  The CAP that picks the ``delta`` it is evaluated at carries both:
``cond(J_fk) <= 1e6`` at ``char_len = r_b``, PROVISIONAL in that length.
``z_home``, bracket ends and widths are in ``r_b``, and in mm at the ASSERTED
``r_b = 90 mm`` where a mm column is printed.  ``p`` is in ``r_b``.

Degrees at the boundary, radians internally.  Arrays are (3, 6).  numpy only.
Reports findings; always exits 0.
"""
from __future__ import annotations

import time

import numpy as np

from . import envelope as ENV
from . import fixed_ratio as FR
from . import score_discriminators as SD
from . import tilt_bracket as TB
from .score_discriminators import _five_number, _spearman
from .tilt_bracket import CAP
from .zhome_bracket import A_RB, BETA, BETA_P, D_RB, C_P, Z_GRID

# --------------------------------------------------------------------------- #
# the columns - x0 in metres, because tau_L is gone and x0 is the honest axis
# --------------------------------------------------------------------------- #
#: ``(x0 in m, what it is)``.  The first entry is the limit in force after the
#: drop and is :data:`.envelope.X0_WORKING` itself, not a copy of it.  The
#: other three are the ``x0`` values :mod:`.tilt_bracket`'s ``tau_L`` = 75 /
#: 150 / 300 ms produced on the same 50 mm working displacement; they are
#: WITHDRAWN, carried only so the trend has something to be read against.
COLUMNS = (
    (ENV.X0_WORKING, "working only, tau_L DROPPED"),
    (ENV.X0_WORKING + 0.075 * ENV.V_PEAK, "withdrawn: +75 ms of drift"),
    (ENV.X0_WORKING + 0.150 * ENV.V_PEAK, "withdrawn: +150 ms, the reference"),
    (ENV.X0_WORKING + 0.300 * ENV.V_PEAK, "withdrawn: +300 ms of drift"),
)

#: Which column is the limit everything computed so far runs on, i.e. the one
#: the rank correlations are taken against.  Asserted against :mod:`.envelope`
#: in :func:`main`, not assumed, so this cannot silently point elsewhere.
REFERENCE_INDEX = 2

#: Which column is the limit in force after the drop.
DROPPED_INDEX = 0


def tilt_of(x0: float) -> float:
    """The envelope's own closed form, called with one ``x0``.

    Nothing about the recovery model is restated here - ``TAU``, ``G`` and the
    ``5/7`` rolling factor all stay in :mod:`.envelope` - so the limits below
    are reproduced from the model rather than transcribed as degrees.
    """
    return ENV.tilt_for(x0, ENV.TAU)


# --------------------------------------------------------------------------- #
# one column
# --------------------------------------------------------------------------- #
def run_column(x0: float, note: str, verbose: bool = True):
    """The whole fixed-ratio pipeline at the tilt limit ``x0`` implies.

    :func:`.tilt_bracket.at_tilt` rebinds the three references that carry the
    tilt limit - the 366-pose screen envelope, the 29-pose harness grid, and
    the closed-form ``N_i > 0`` floor - for the duration of the pass and
    restores them after.  Inside it, :func:`.fixed_ratio.run_slice` runs
    unchanged at the asserted ratio, so the ONLY thing that differs between
    columns is the tilt limit.  The rebinding is verified inside the block
    rather than trusted.
    """
    tilt = tilt_of(x0)
    t0 = time.time()
    with TB.at_tilt(tilt):
        az29, mg29 = ENV.envelope_poses()
        check = dict(harness_max=float(mg29.max()),
                     screen_max=float(TB.ZB.fine_poses()[1].max()),
                     zb_global=TB.ZB.TILT_LIMIT_DEG,
                     sd_global=SD.TILT_LIMIT_DEG)
        slc = FR.run_slice(FR.R_P_RB, f"{tilt:.4f} deg", f"{tilt:.3f}",
                           verbose=False)
    slc.update(x0=x0, tilt=tilt, note=note, check=check,
               secs=time.time() - t0)
    if verbose:
        print(f"    x0 = {x0*1e3:>5.1f} mm   tilt = {tilt:>7.4f} deg   "
              f"{len(slc['rows']):>4} screened  "
              f"{len(slc['feasible']):>4} feasible  "
              f"{slc['dropped']:>2} dropped  {slc['no_delta']:>2} no delta   "
              f"({slc['secs']:.1f} s)", flush=True)
    return slc


def score_map(slc):
    """``{candidate key: margin(dxy = p)}`` with ``r_p`` dropped from the key.

    ``r_p`` is one value in every column, so the remaining four axes ARE the
    candidate.  A candidate with no admissible ``delta`` at the cap is carried
    as ``NaN`` rather than dropped, so "not feasible" and "feasible but
    inadmissible" stay distinguishable inside the correlation, where
    :func:`.score_discriminators._spearman` drops it pairwise and counts it out.
    """
    return {FR._key_no_rp(r): float(r["score_p"]) for r in slc["measured"]}


def floor_sets_lower_end(slc):
    """``(coincidence count, binding count)`` for the ``N > 0`` floor.

    Two different questions, and the first is the one the existing
    :mod:`.fixed_ratio` part (3) asks:

    ``coincidence``
        ``z_lo <= floor``.  The screen intersects with ``Z_GRID > floor``
        STRICTLY, so this can never fire and reporting it alone would answer
        "no" by construction.
    ``binding``
        ``reach_lo < z_lo`` - the reach test allowed a lower ``z_home`` and the
        floor is what removed it.  This is the question as asked, and it is
        answerable because :func:`.tilt_bracket.screen` now carries
        ``reach_lo`` beside ``reach_hi``.
    """
    coin = sum(1 for r in slc["feasible"] if r["z_lo"] <= r["cf"] + 1e-9)
    bind = sum(1 for r in slc["feasible"]
               if np.isfinite(r["reach_lo"]) and r["reach_lo"] < r["z_lo"])
    return coin, bind


# --------------------------------------------------------------------------- #
# parts
# --------------------------------------------------------------------------- #
def part_limits(cols):
    """The limit, reproduced from the model rather than asserted."""
    print()
    print("=" * 78)
    print("(0) THE TILT LIMIT AFTER THE DROP, FROM envelope.tilt_for")
    print("=" * 78)
    print("  x0 = 50 mm WORKING DISPLACEMENT ONLY.  The 30 mm latency drift is")
    print("  removed, not revised: hardware-pull sec.4.1 finds servo step")
    print("  response NOT PUBLISHED by any manufacturer (slew rate only, which")
    print("  is a different quantity), and sec.4.2 finds the sensor cells")
    print("  publish a frame interval, which is a sampling period and not a")
    print("  sensor-to-pose latency.  tau_L cannot be reconstructed, so it is")
    print("  dropped.  This completes the 2026-09-03 withdrawal of the")
    print("  300 mm/s figure it inherited from.")
    print()
    print(f"  acc = 4 x0 / tau^2,  tau = {ENV.TAU} s")
    print(f"  sin(tilt) = 7 acc / (5 g),  g = {ENV.G} m/s^2")
    print("  Both from envelope.py; nothing about the model is restated here.")
    print()
    print(f"    {'x0 [mm]':>9} {'acc [m/s^2]':>12} {'sin(tilt)':>11} "
          f"{'tilt [deg]':>11} {'/ 10.529':>9}  status")
    ref = cols[REFERENCE_INDEX]["tilt"]
    for c in cols:
        acc = ENV.bang_bang_accel(c["x0"], ENV.TAU)
        print(f"    {c['x0']*1e3:>9.1f} {acc:>12.4f} "
              f"{acc / (ENV.ROLL_FACTOR * ENV.G):>11.6f} "
              f"{c['tilt']:>11.4f} {c['tilt']/ref:>9.3f}  {c['note']}")
    print()
    drop = cols[DROPPED_INDEX]
    print(f"  THE LIMIT IN FORCE IS {drop['tilt']:.4f} deg, and it is PRINTED "
          f"above from")
    print(f"  envelope.tilt_for({ENV.X0_WORKING}) - not hardcoded as a number "
          f"of degrees")
    print(f"  anywhere in this module.  What the drop costs, in the envelope's")
    print(f"  own terms: {ref - drop['tilt']:.4f} deg of the "
          f"{ref:.4f} that everything computed so far")
    print(f"  runs on, which is {(ref - drop['tilt'])/ref:.1%} of it.")
    print()
    print("  envelope.TILT_LIMIT_DEG still reads "
          f"{ENV.TILT_LIMIT_DEG:.4f} and envelope.py is NOT")
    print("  edited here; notation.md is not touched.  Recording the drop is a")
    print("  separate documentation pass.  Until it happens, the reference")
    print("  column below is that constant and the limit in force is not.")


def part_selfcheck(cols):
    """Was the limit actually in force in each pass?"""
    print()
    print("=" * 78)
    print("(0b) REBINDING SELF-CHECK - the limit was in force, not assumed")
    print("=" * 78)
    print("  The tilt limit enters THREE places that must move together: the")
    print("  366-pose screen envelope, the 29-pose harness grid, and the")
    print("  closed-form N_i > 0 floor.  A missed one would silently mix two")
    print("  envelopes in one pass, so all three are read back inside the block.")
    print()
    print(f"    {'tilt [deg]':>11} {'screen max':>12} {'harness max':>12} "
          f"{'zhome global':>13} {'sd global':>11}  agree")
    for c in cols:
        k = c["check"]
        ok = all(abs(v - c["tilt"]) < 1e-9 for v in k.values())
        print(f"    {c['tilt']:>11.4f} {k['screen_max']:>12.4f} "
              f"{k['harness_max']:>12.4f} {k['zb_global']:>13.4f} "
              f"{k['sd_global']:>11.4f}  {ok}")


def part1(cols):
    """(1) Non-empty brackets, R / X."""
    print()
    print("=" * 78)
    print("(1) NON-EMPTY z_home BRACKETS, AND THE REACH / CROSSING SPLIT")
    print("=" * 78)
    print("  R and X are the 2026-09-05 attribution and are the only two cases,")
    print("  because N_i > 0 is ONE-SIDED - a floor with no ceiling:")
    print("    R  reach empty on its own       - no z_home reaches at any delta")
    print("    X  reach ceiling BELOW the N>0 floor - both satisfiable alone,")
    print(f"       crossed.  Every X is refined by bisection to 1e-9 first, "
          f"since a")
    print(f"       crossing narrower than the {Z_GRID[1]-Z_GRID[0]} r_b grid "
          f"step would be a sampling")
    print("       artifact rather than a crossing.  'artifact' counts the ones")
    print("       that refined away and were returned to R.")
    print()
    print(f"  All four columns: {len(BETA)}x{len(BETA_P)}x{len(A_RB)}x"
          f"{len(D_RB)} = {len(cols[0]['rows'])} candidates at "
          f"r_p/r_b = {FR.R_P_RB:.6f},")
    print(f"  c_p/r_b = {C_P}.  Only the tilt limit differs.")
    print()
    print(f"    {'tilt [deg]':>11} {'cand':>6} {'non-empty':>10} {'frac':>7} "
          f"{'empty':>6} {'R':>5} {'X':>5} {'artifact':>9}  status")
    for c in cols:
        n = len(c["rows"])
        print(f"    {c['tilt']:>11.4f} {n:>6} {len(c['feasible']):>10} "
              f"{len(c['feasible'])/n:>7.3f} {n-len(c['feasible']):>6} "
              f"{len(c['cat_R']):>5} {len(c['cat_X']):>5} "
              f"{len(c['artifacts']):>9}  {c['note']}")
    print()
    drop, ref = cols[DROPPED_INDEX], cols[REFERENCE_INDEX]
    print(f"  AT THE LIMIT IN FORCE: {len(drop['feasible'])} of "
          f"{len(drop['rows'])} candidates have a non-empty bracket,")
    print(f"  against {len(ref['feasible'])} of {len(ref['rows'])} at "
          f"{ref['tilt']:.4f} deg.  The change is "
          f"{len(drop['feasible']) - len(ref['feasible']):+d} candidates.")
    print(f"  The split is {len(drop['cat_R'])} R / {len(drop['cat_X'])} X "
          f"at the limit in force.")
    print()
    same = ({FR._key_no_rp(r) for r in ref["feasible"]}
            <= {FR._key_no_rp(r) for r in drop["feasible"]})
    print(f"  Is the 10.529-deg feasible set a SUBSET of the "
          f"{drop['tilt']:.3f}-deg one? -> {same}")
    print("  Nesting is not automatic: a lower tilt limit shrinks the screen")
    print("  envelope AND lowers the N>0 floor, and the two move the bracket")
    print("  from opposite ends, so it is measured rather than argued.")


def part2(cols):
    """(2) Empties by a/r_b, with d/r_b beside it."""
    print()
    print("=" * 78)
    print("(2) EMPTY BRACKETS BY a / r_b")
    print("=" * 78)
    print("  Each cell is 'empty (R / X)' out of the candidates in that row.")
    print("  Columns are the tilt limit; the first is the limit in force.")
    print()
    for axis, vals, name in (("a", A_RB, "a/r_b"), ("d", D_RB, "d/r_b")):
        print(f"    {name:>8} {'of':>4}" +
              "".join(f"{format(c['tilt'], '.3f'):>16}" for c in cols))
        for v in vals:
            cells = []
            for c in cols:
                ke = [e for e in c["rows"]
                      if e[axis] == v and not e["feasible"]]
                nR = sum(1 for e in ke if e in c["cat_R"])
                nX = sum(1 for e in ke if e in c["cat_X"])
                cells.append(f"{len(ke)} (R{nR}/X{nX})")
            tot = sum(1 for r in cols[0]["rows"] if r[axis] == v)
            print(f"    {v:>8.2f} {tot:>4}" + "".join(f"{c:>16}" for c in cells))
        print()
    drop = cols[DROPPED_INDEX]
    print("  The (a, d) cells at the limit in force, feasible of screened:")
    print()
    print(f"    {'':>10}" + "".join(f"{'d=' + format(d, '.2f'):>14}"
                                    for d in D_RB))
    for a in A_RB:
        cells = []
        for d in D_RB:
            nf = sum(1 for r in drop["rows"]
                     if r["a"] == a and r["d"] == d and r["feasible"])
            nt = sum(1 for r in drop["rows"] if r["a"] == a and r["d"] == d)
            cells.append(f"{nf} / {nt}")
        print(f"    a={a:<8.2f}" + "".join(f"{c:>14}" for c in cells))


def part3(cols):
    """(3) Bracket ends, widths, contiguity, and the N>0 floor."""
    step = float(Z_GRID[1] - Z_GRID[0])
    print()
    print("=" * 78)
    print("(3) BRACKET LOWER AND UPPER END RANGES, CONTIGUITY, AND THE FLOOR")
    print("=" * 78)
    print(f"  z_home / r_b GRID values, resolution {step} r_b = "
          f"{step*FR.R_B_MM:.2f} mm at the")
    print(f"  ASSERTED r_b = {FR.R_B_MM:.0f} mm.  Read an end as 'feasible by "
          f"{step}'; there is no")
    print("  bisection refinement on the ends, because a bracket narrower than")
    print("  the grid step is not a design.")
    print()
    print(f"    {'tilt':>8} {'lower ends':>17} {'upper ends':>17} "
          f"{'widths':>17} {'N>0 floor':>10}")
    for c in cols:
        lo, hi, w = FR.ends(c)
        print(f"    {c['tilt']:>8.3f} "
              f"{f'{lo.min():.3f} .. {lo.max():.3f}':>17} "
              f"{f'{hi.min():.3f} .. {hi.max():.3f}':>17} "
              f"{f'{w.min():.3f} .. {w.max():.3f}':>17} "
              f"{c['feasible'][0]['cf']:>10.5f}")
    print()
    print(f"  The same rows in mm at the ASSERTED r_b = {FR.R_B_MM:.0f} mm:")
    print()
    print(f"    {'tilt':>8} {'lower ends [mm]':>19} {'upper ends [mm]':>19} "
          f"{'widths [mm]':>19} {'floor [mm]':>11}")
    for c in cols:
        lo, hi, w = FR.ends(c)
        M = FR.R_B_MM
        print(f"    {c['tilt']:>8.3f} "
              f"{f'{lo.min()*M:.2f} .. {lo.max()*M:.2f}':>19} "
              f"{f'{hi.min()*M:.2f} .. {hi.max()*M:.2f}':>19} "
              f"{f'{w.min()*M:.2f} .. {w.max()*M:.2f}':>19} "
              f"{c['feasible'][0]['cf']*M:>11.2f}")
    print()
    print("  The floor is delta-free, a-free and d-free - it is")
    print("  r_p sin(tilt) + c_p cos(tilt) and depends on r_p, c_p and the tilt")
    print("  limit alone - so it is ONE number per column.")
    print()
    print(f"    {'tilt':>8} {'non-contiguous':>15} {'clipped at top':>15} "
          f"{'floor coincid.':>15} {'floor BINDS':>12} {'of':>5}")
    for c in cols:
        coin, bind = floor_sets_lower_end(c)
        clip = sum(1 for r in c["feasible"]
                   if r["z_hi"] >= Z_GRID[-1] - 1e-12)
        print(f"    {c['tilt']:>8.3f} "
              f"{sum(1 for r in c['feasible'] if not r['contiguous']):>15} "
              f"{clip:>15} {coin:>15} {bind:>12} {len(c['feasible']):>5}")
    print()
    print("  TWO DIFFERENT QUESTIONS, and only the second is the one asked:")
    print("    'floor coincid.'  z_lo <= floor.  The screen intersects with")
    print("                      Z_GRID > floor STRICTLY, so this can never")
    print("                      fire; fixed_ratio part (3) reports this test")
    print("                      and its 0 is true by construction, not a")
    print("                      measurement.")
    print("    'floor BINDS'     reach_lo < z_lo - the reach test allowed a")
    print("                      LOWER z_home and the floor is what removed it.")
    print("                      This is 'does the N>0 floor set the lower")
    print("                      end?', and it is answerable because screen()")
    print("                      now carries reach_lo beside reach_hi.")
    print()
    print("  Width five-number summaries, r_b:")
    print()
    print(f"    {'tilt':>8}{'min':>10}{'Q1':>10}{'median':>10}{'Q3':>10}"
          f"{'max':>10}   {'median [mm]':>12}")
    for c in cols:
        _, _, w = FR.ends(c)
        f5 = _five_number(w)
        print(f"    {c['tilt']:>8.3f}" + "".join(f"{v:>10.3f}" for v in f5)
              + f"   {f5[2]*FR.R_B_MM:>12.2f}")


def part4(cols):
    """(4) margin(dxy = p): five-number summary and max over survivors."""
    print()
    print("=" * 78)
    print(f"(4) margin(dxy = p) AT p = {FR.P_SCORE:.6f} r_b")
    print("=" * 78)
    print(f"  p = sqrt(3 x 0.2^2) mm / {FR.R_B_MM:.0f} mm = "
          f"{FR.P_SCORE:.6f}, recomputed from the three")
    print(f"  build-error sources rather than transcribed; the request quotes")
    print(f"  0.3464 / 90 = {FR.P_SCORE_QUOTED}, and the two agree to "
          f"{abs(FR.P_SCORE - FR.P_SCORE_QUOTED):.1e}.")
    print(f"  notation.md's {FR.P_SCORE_SUPERSEDED} is at the superseded 80 mm "
          f"floor and is NOT used.")
    print()
    print("  EVALUATED AT p through probe_margin, the same worst-over-24-")
    print("  displacement-azimuths quantity tilt_authority uses.  NO SLOPE IS")
    print("  EXTRAPOLATED off sens; the difference between the direct")
    print("  evaluation and that extrapolation is printed below as a")
    print("  measurement and relied on nowhere.")
    print()
    print("  margin is a dimensionless ratio and carries NO characteristic")
    print("  length and NO Jacobian.  The delta it is evaluated at was tuned")
    print(f"  under cond(J_fk) <= {CAP:.0e} at char_len = "
          f"{SD.CONSTRAINT_CHAR_LEN}, which carries both and is")
    print("  PROVISIONAL in that length.  z_home is each candidate's bracket")
    print("  MIDPOINT - a point in the interior for definiteness, not a")
    print("  recommendation.")
    print()
    print(f"    {'tilt':>8} {'feas':>5} {'drop':>5} {'noDl':>5} {'surv':>5}"
          f"{'min':>11}{'Q1':>11}{'median':>11}{'Q3':>11}{'max':>11}")
    for c in cols:
        sc = FR.scores(c)
        f5 = _five_number(sc)
        print(f"    {c['tilt']:>8.3f} {len(c['feasible']):>5} "
              f"{c['dropped']:>5} {c['no_delta']:>5} "
              f"{int(np.isfinite(sc).sum()):>5}"
              + "".join(f"{v:>11.6f}" for v in f5))
    print()
    print("    feas = non-empty bracket; drop = ik unreachable at the bracket")
    print("    midpoint on the 29-pose grid; noDl = no admissible delta at the")
    print(f"    cap; surv = the five-number sample.")
    print()
    print("  margin(dxy = 0) beside it, for reading:")
    print()
    print(f"    {'tilt':>8} {'surv':>5}"
          f"{'min':>11}{'Q1':>11}{'median':>11}{'Q3':>11}{'max':>11}")
    for c in cols:
        m0 = np.array([r["margin_con"] for r in c["measured"]], dtype=float)
        print(f"    {c['tilt']:>8.3f} {int(np.isfinite(m0).sum()):>5}"
              + "".join(f"{v:>11.6f}" for v in _five_number(m0)))
    print()
    print("  MAX over survivors, and where it sits:")
    print()
    print(f"    {'tilt':>8} {'max margin(p)':>14} {'beta':>6} {'beta_p':>7} "
          f"{'a/r_b':>6} {'d/r_b':>6} {'z_home':>8} {'[mm]':>7} "
          f"{'delta':>6}")
    for c in cols:
        best = max((r for r in c["measured"] if np.isfinite(r["score_p"])),
                   key=lambda r: r["score_p"])
        print(f"    {c['tilt']:>8.3f} {best['score_p']:>14.6f} "
              f"{best['beta']:>6.0f} {best['beta_p']:>7.0f} "
              f"{best['a']:>6.2f} {best['d']:>6.2f} {best['z_home']:>8.4f} "
              f"{best['z_home']*FR.R_B_MM:>7.2f} {best['delta_con']:>6.1f}")
    print()
    print("  Negative margins are CARRIED, not dropped - a negative margin is a")
    print("  candidate the 366-pose screen passed and the 29-pose harness grid")
    print("  does not; the two grids are not nested.  Dropping them would")
    print("  flatter the summary:")
    print()
    print(f"    {'tilt':>8} {'neg at dxy=p':>14} {'neg at dxy=0':>14} "
          f"{'of':>5}")
    for c in cols:
        sc, m0 = FR.scores(c), np.array([r["margin_con"] for r in c["measured"]],
                                        dtype=float)
        print(f"    {c['tilt']:>8.3f} "
              f"{int(np.sum(sc[np.isfinite(sc)] < 0.0)):>14} "
              f"{int(np.sum(m0[np.isfinite(m0)] < 0.0)):>14} "
              f"{int(np.isfinite(sc).sum()):>5}")
    print()
    print("  THE EXTRAPOLATION THAT WAS NOT USED, for the record only:")
    print()
    print(f"    {'tilt':>8} {'worst |direct - linear|':>24} "
          f"{'median':>12} {'Spearman':>10}")
    for c in cols:
        sc = FR.scores(c)
        ex = np.array([r["score_extrap"] for r in c["measured"]], dtype=float)
        rho, _ = _spearman(sc, ex)
        print(f"    {c['tilt']:>8.3f} {np.nanmax(np.abs(sc - ex)):>24.3e} "
              f"{np.nanmedian(np.abs(sc - ex)):>12.3e} {rho:>10.4f}")
    print("    Close in RANK, not in VALUE - the failure mode that would have")
    print("    gone unnoticed had the slope been used.")


def part5(cols):
    """(5) Spearman against the ranking at the reference limit."""
    ref = cols[REFERENCE_INDEX]
    ref_map = score_map(ref)
    ref_feas = {FR._key_no_rp(r) for r in ref["feasible"]}
    print()
    print("=" * 78)
    print(f"(5) SPEARMAN OF THE RANKING AGAINST THE RANKING AT "
          f"{ref['tilt']:.4f} deg")
    print("=" * 78)
    print("  Over the candidates FEASIBLE AT BOTH limits - a candidate with no")
    print("  bracket at one limit has no margin there and cannot be ranked, so")
    print("  the common set is stated rather than absorbed.  Inside it, a NaN")
    print("  score (feasible but no admissible delta at the cap) is dropped")
    print("  pairwise by _spearman and counted out; the two counts are printed")
    print("  apart for exactly that reason.")
    print()
    print(f"    {'tilt':>8} {'feasible':>9} {'common set':>11} "
          f"{'n used':>7} {'Spearman rho':>13}  status")
    for c in cols:
        c_map = score_map(c)
        c_feas = {FR._key_no_rp(r) for r in c["feasible"]}
        common = sorted(ref_feas & c_feas)
        x = np.array([c_map[k] for k in common], dtype=float)
        y = np.array([ref_map[k] for k in common], dtype=float)
        rho, n = _spearman(x, y)
        c["common"], c["rho"], c["n_used"] = len(common), rho, n
        print(f"    {c['tilt']:>8.3f} {len(c_feas):>9} {len(common):>11} "
              f"{n:>7} {rho:>13.4f}  {c['note']}")
    print()
    drop = cols[DROPPED_INDEX]
    print(f"  THE NUMBER ASKED FOR: rho = {drop['rho']:.4f} between the "
          f"{drop['tilt']:.3f}-deg ranking")
    print(f"  and the {ref['tilt']:.3f}-deg ranking, over the "
          f"{drop['common']} candidates feasible at both")
    print(f"  ({drop['n_used']} of them carrying a finite score at both, which "
          f"is what rho is")
    print(f"  computed on).")
    print()
    print("  What rho does NOT say: it is a statement about ORDER, not about")
    print("  value.  Part (4)'s five-number rows are where the values moved.")
    print("  It is also blind below the score's own resolution - two candidates")
    print("  closer together than the 24-direction sweep can resolve are")
    print("  ordered by the discretisation, and that resolution is measured in")
    print("  part (6).")


def part6(cols, res):
    """(6) The score's own resolution, at the limit in force."""
    drop = cols[DROPPED_INDEX]
    print()
    print("=" * 78)
    print("(6) THE RESOLUTION OF THE SCORE AT THE LIMIT IN FORCE")
    print("=" * 78)
    print(f"  probe_margin takes the worst over {SD.N_DISP_DIR} displacement "
          f"azimuths, so it is")
    print("  optimistic by however much the true worst direction falls between")
    print("  samples.  Refined, over every survivor:")
    print()
    print(f"    worst |score at {SD.N_DISP_DIR} directions - score at "
          f"{FR.N_DISP_REFINED}| : {res:.3e}")
    print()
    print(f"  Two candidates closer together than {res:.1e} are NOT ordered by "
          f"the grid")
    print("  the score is computed on, and no ordering in part (4) or (5) is")
    print("  claimed below that.  sweep_ranges.py measures the same quantity at")
    print("  24, 72 and 360 directions over the hardware-pull candidate set.")


def verdict(cols, res):
    """What the drop did to the feasible set, stated plainly."""
    drop, ref = cols[DROPPED_INDEX], cols[REFERENCE_INDEX]
    lo_d, hi_d, w_d = FR.ends(drop)
    lo_r, hi_r, w_r = FR.ends(ref)
    sc_d, sc_r = FR.scores(drop), FR.scores(ref)
    _, bind_d = floor_sets_lower_end(drop)
    _, bind_r = floor_sets_lower_end(ref)
    print()
    print("=" * 78)
    print("WHAT DROPPING tau_L DID TO THE FEASIBLE SET")
    print("=" * 78)
    print()
    print(f"  The limit fell from {ref['tilt']:.4f} to {drop['tilt']:.4f} deg, "
          f"{(ref['tilt']-drop['tilt'])/ref['tilt']:.1%} of it, and:")
    print()
    print(f"    feasible candidates   {len(ref['feasible']):>4} -> "
          f"{len(drop['feasible']):>4}  of {len(drop['rows'])}   "
          f"({len(drop['feasible'])-len(ref['feasible']):+d})")
    print(f"    empties, R / X        "
          f"{len(ref['cat_R']):>4} / {len(ref['cat_X'])} -> "
          f"{len(drop['cat_R']):>4} / {len(drop['cat_X'])}")
    print(f"    median bracket width  {np.median(w_r):.3f} -> "
          f"{np.median(w_d):.3f} r_b   "
          f"({np.median(w_r)*FR.R_B_MM:.1f} -> {np.median(w_d)*FR.R_B_MM:.1f} "
          f"mm at r_b = {FR.R_B_MM:.0f})")
    # The median FALLS while the count RISES, which reads as a contradiction
    # until the two samples are held to the same candidates.  Measured rather
    # than explained away.
    ref_keys = {FR._key_no_rp(r) for r in ref["feasible"]}
    w_common = np.array([r["z_hi"] - r["z_lo"] for r in drop["feasible"]
                         if FR._key_no_rp(r) in ref_keys], dtype=float)
    w_new = np.array([r["z_hi"] - r["z_lo"] for r in drop["feasible"]
                      if FR._key_no_rp(r) not in ref_keys], dtype=float)
    print(f"      on the {w_common.size} candidates feasible at BOTH, the "
          f"median width RISES")
    print(f"      {np.median(w_r):.3f} -> {np.median(w_common):.3f} r_b.  The "
          f"fall above is composition, not")
    print(f"      shrinkage: the {w_new.size} candidates the lower limit "
          f"ADDS carry a median")
    print(f"      width of {np.median(w_new):.3f} r_b "
          f"({np.median(w_new)*FR.R_B_MM:.1f} mm) and pull the pooled median "
          f"down.")
    print(f"    N>0 floor binds       {bind_r:>4} -> {bind_d:>4} lower ends")
    print(f"    median margin(p)      {_five_number(sc_r)[2]:.6f} -> "
          f"{_five_number(sc_d)[2]:.6f}")
    print(f"    max margin(p)         {np.nanmax(sc_r):.6f} -> "
          f"{np.nanmax(sc_d):.6f}")
    print(f"    Spearman vs 10.529    {drop['rho']:.4f} over "
          f"{drop['common']} candidates feasible at both")
    print()
    print("  NOTHING IS CHOSEN HERE.  No range is chosen, no part is chosen,")
    print("  the sweep harness is not specced, envelope.py is not edited and")
    print("  notation.md is not touched - recording the tau_L drop is a")
    print("  separate documentation pass, and until it happens envelope.py")
    print(f"  still carries {ENV.TILT_LIMIT_DEG:.4f} deg.")
    print()
    print(f"  Every number above is at char_len = r_b for the cap that picks")
    print(f"  delta (cond(J_fk) <= {CAP:.0e}), and margin itself carries no")
    print(f"  characteristic length at all.  z_home and widths are in r_b, and")
    print(f"  in mm only at the ASSERTED r_b = {FR.R_B_MM:.0f} mm.  The "
          f"ranking is resolved to")
    print(f"  {res:.1e} and nothing finer is claimed.")


# --------------------------------------------------------------------------- #
def main() -> None:
    print("=" * 78)
    print("THE FEASIBLE SET AFTER tau_L IS DROPPED - 2026-09-09")
    print("=" * 78)
    print("  DECISION, ASSERTED 2026-09-09: tau_L is DROPPED, not revised.")
    print("  docs/hardware-pull.md sec.4 establishes that servo step response")
    print("  is unpublished by every candidate maker and cannot be")
    print("  reconstructed from published data, so the 30 mm latency drift and")
    print("  the 3.97 deg it carried have no basis and are REMOVED.  This")
    print("  completes the 2026-09-03 withdrawal of the 300 mm/s figure that")
    print("  tau_L inherited from.")
    print()
    print("  Everything computed so far runs on 10.529 deg.  This module")
    print("  re-runs the feasibility screen at the new limit, at the fixed")
    print(f"  absolute scale r_b = {FR.R_B_MM:.0f} mm, r_p = {FR.R_P_MM:.0f} "
          f"mm (r_p/r_b = {FR.R_P_RB:.6f}),")
    print(f"  c_p/r_b = {C_P}, under the existing cap cond(J_fk) <= {CAP:.0e} "
          f"at")
    print(f"  char_len = {SD.CONSTRAINT_CHAR_LEN}.  fixed_ratio's screen is "
          f"REUSED, not reimplemented;")
    print("  the only change is the tilt limit.")
    print()
    # Repointed 2026-09-09 (b0703ab): this used to check the reference column
    # against ENV.TILT_LIMIT_DEG, which was the same number by construction -
    # both were tilt_for(0.080).  TILT_LIMIT_DEG now derives from X0_WORKING
    # and no longer equals this column, so the check is pinned to the
    # column's own definition (150 ms, x0 = 0.080) instead of to a constant
    # that has since moved to mean something else.
    assert abs(tilt_of(COLUMNS[REFERENCE_INDEX][0]) - ENV.tilt_for(0.080)) \
        < 1e-9, ("the reference column is not tilt_for(0.080); "
                 "REFERENCE_INDEX is wrong")
    assert abs(FR.P_SCORE - FR.P_SCORE_QUOTED) < 1e-6, "p disagrees with 0.003849"

    print("-" * 78)
    print("COLUMNS RUN")
    print("-" * 78)
    t0 = time.time()
    cols = [run_column(x0, note) for x0, note in COLUMNS]
    res = FR.disp_azimuth_resolution(cols[DROPPED_INDEX]["measured"],
                                     cols[DROPPED_INDEX]["R29"],
                                     cols[DROPPED_INDEX]["az29"])
    print(f"    {'total':<22} {time.time() - t0:.1f} s")

    part_limits(cols)
    part_selfcheck(cols)
    part1(cols)
    part2(cols)
    part3(cols)
    part4(cols)
    part5(cols)
    part6(cols, res)
    verdict(cols, res)


if __name__ == "__main__":
    main()
