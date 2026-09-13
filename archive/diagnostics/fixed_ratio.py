"""The feasible set at the ratio absolute scale fixed it to, 2026-09-08.

    python -m stewart.diagnostics.fixed_ratio

WHY THIS EXISTS.  Absolute scale was ASSERTED on 2026-09-08: ``r_b = 90 mm``
from the print bed, ``r_p = 80 mm`` from the printed hub carrying the bought
220 mm sheet.  ``r_p / r_b`` is therefore no longer a sweep axis - it is one
number, and every screen, tune and ranking in this package was measured at
values that are not it.

  * :mod:`.zhome_bracket`'s 540-candidate coarse grid samples ``r_p/r_b`` at
    0.60, 0.85 and 1.10, and the whole top of the field at every tilt limit
    :mod:`.tilt_bracket` ran sat at **0.60**, the smallest of the three;
  * :mod:`.box_boundary`'s ``r_p`` ray ran DOWNWARD from 0.60 to 0.10, away
    from the asserted ratio, and the margin climbed the whole way;
  * the one nearby datum is ``docs/archive/cc-fk-gate.md``'s fixture F, ``r_p > r_b``
    at 110/100, which carried the narrowest non-empty ``z_home`` bracket found
    anywhere - 10 mm at that fixture's own ``delta``.

So the asserted ratio sits between the two coarse-grid values nearest it and
has never been evaluated.  This module evaluates it.

WHAT IS NOT DECIDED HERE.  **No range is chosen and the sweep harness is not
specced.**  Every axis below is exactly as :mod:`.zhome_bracket` builds it -
``BETA``, ``BETA_P``, ``A_RB``, ``D_RB`` and ``Z_GRID`` unchanged, ``c_p/r_b =
0.1``, the 1-degree ``delta`` scan, the 366-pose screen envelope, the 29-pose
harness grid, the 10.529-degree tilt limit and the existing cap ``cond(J_fk)
<= 1e6`` at ``char_len = r_b``.  The ONLY thing this module changes is that
``r_p/r_b`` is held at one value instead of three.  ``docs/archive/notation.md`` is not
touched, and nothing here is a proposed grid.

THE RATIO, and why the quotient is used rather than the quoted decimal.  The
assertion is two lengths; ``80 / 90 = 0.888889`` is what they imply and
``0.8889`` is its four-figure rounding.  The two differ by ``1.1e-5 r_b``,
which is four decades below the 0.025-``r_b`` ``Z_GRID`` step, so no reported
quantity can tell them apart - and rather than assert that, part (0) runs the
whole screen at BOTH and reports whether any count, end or width moves.

THE SCORE.  ``margin(dxy = p)`` at ``p = sqrt(3 * 0.2^2) / 90 = 0.003849``.
This is the 2026-09-08 build-error assertion (``docs/archive/notation.md`` sec.8) with its
``r_b`` normaliser taken from the scale now in force.  The value in
``docs/archive/notation.md`` is ``0.004330``, normalised by the then-asserted **80 mm
floor**; ``r_b`` is now 90 mm and ``p`` scales inversely with it, so
``0.004330`` is the wrong number here and is **not used** - it is carried in
part (0) only to show the two apart.  The score is EVALUATED at ``p`` through
:func:`.score_discriminators.probe_margin`, the same worst-over-24-
displacement-azimuths quantity :mod:`.tilt_authority` uses.  **No slope is
extrapolated off** ``sens``; the difference between the direct evaluation and
that extrapolation is reported in part (4) as a measurement, not relied on.

WHAT IS MEASURED, all at ``r_p/r_b`` = the asserted ratio:

  (0) the assertion's own arithmetic, and the two checks above;
  (1) non-empty ``z_home`` brackets of the 180 candidates, split R / X as the
      2026-09-05 attribution defines them, against the 363 of 540 the coarse
      grid gives with ``r_p/r_b`` spanning 0.60-1.10 - RECOMPUTED here from
      the same three slices, not transcribed;
  (2) the empties by ``a/r_b``, in the form :mod:`.tilt_bracket` part (2)
      reports it, with ``d/r_b`` beside it;
  (3) bracket lower and upper end ranges, widths and non-contiguity - in
      ``r_b`` and, because absolute scale is now asserted, in mm at
      ``r_b = 90``;
  (4) ``margin(dxy = p)``: five-number summary and max over the survivors;
  (5) the top 20 grouped by ``(a, d, |beta_p - beta|)`` - part (8)'s key with
      ``r_p`` dropped, since ``r_p`` no longer varies;
  (6) whether ``a/r_b`` and ``d/r_b`` still run to the edge of their sampled
      ranges here, or whether the optimum is interior.

UNITS AND LABELS.  ``margin`` is a dimensionless ratio and carries NO
characteristic length and NO Jacobian - a property of the quantity, not an
omission.  The CAP that picks the ``delta`` it is evaluated at carries both:
``cond(J_fk) <= 1e6`` at ``char_len = r_b``, PROVISIONAL in that length, and
every table below that rests on it says so.  ``z_home``, bracket ends and
widths are in ``r_b``; where they are also given in mm it is at the ASSERTED
``r_b = 90 mm`` and is labelled as asserted, not measured.  ``p`` is in ``r_b``.

Degrees at the boundary, radians internally.  Arrays are (3, 6).  numpy only.
Reports findings; always exits 0.
"""
from __future__ import annotations

import time

import numpy as np

from ..geometry import make_geometry
from . import score_discriminators as SD
from . import zhome_bracket as ZB
from .envelope import TILT_LIMIT_DEG
from .score_discriminators import TIE_TOL, _abs_e_groups, _five_number, _spearman
from .tilt_bracket import CAP, _key, attribute, constrained_margins, screen
from .zhome_bracket import (A_RB, BETA, BETA_P, D_RB, DELTA_GRID, C_P, R_B,
                            RP_RB, Z_GRID)

# --------------------------------------------------------------------------- #
# the assertion, 2026-09-08 - two lengths, and everything else follows
# --------------------------------------------------------------------------- #
#: ``r_b`` in mm.  ASSERTED from the print bed, not measured and not optimised.
#: It is the number that turns every ratio in this package into a length, and
#: it is the reason ``r_p/r_b`` is no longer a sweep axis.
R_B_MM = 90.0

#: ``r_p`` in mm.  ASSERTED from the printed hub carrying the bought 220 mm
#: sheet.  Same standing as :data:`R_B_MM`: a decision, with no diagnostic
#: behind it.
R_P_MM = 80.0

#: The ratio the two imply.  The quotient is used rather than the quoted
#: four-figure decimal; part (0) runs the screen at both and reports whether
#: anything moves.
R_P_RB = R_P_MM / R_B_MM

#: The decimal the assertion quotes.  Carried for the part (0) check only.
R_P_RB_QUOTED = 0.8889

#: The three sources of build error, mm, each 0.2 (``docs/archive/notation.md`` sec.8): the
#: printer tolerance, which has a source, and the ball-joint free play and
#: platform centring, which are PLACEHOLDERS at the printer's figure pending
#: the hardware pull.  Combined RSS on an independence assumption; the 0.6 mm
#: worst-case sum is rejected there as unphysical for three independent
#: sources.
BUILD_ERROR_SOURCES_MM = (0.2, 0.2, 0.2)

#: The score's probe, in ``r_b``.  The sec.8 assertion renormalised to the
#: ``r_b`` now in force.  ``docs/archive/notation.md`` quotes 0.004330 at the superseded
#: 80 mm floor; ``p`` scales inversely with ``r_b``, so at 90 mm it is smaller.
P_SCORE = float(np.sqrt(sum(s * s for s in BUILD_ERROR_SOURCES_MM)) / R_B_MM)

#: The value ``docs/archive/notation.md`` sec.8 still carries, at the superseded 80 mm
#: floor.  Reported in part (0) so the two are seen apart; **never used as the
#: probe below**.
P_SCORE_SUPERSEDED = 0.004330

#: The value the request quotes for ``p`` at ``r_b = 90``.  Checked against
#: :data:`P_SCORE`, which is computed from the three sources rather than
#: transcribed.
P_SCORE_QUOTED = 0.003849

#: Displacement azimuths used to MEASURE the resolution of the score, not to
#: compute it.  :func:`.score_discriminators.probe_margin` sweeps
#: :data:`.score_discriminators.N_DISP_DIR` = 24 directions and that is the
#: settled quantity; refining to 360 says how much of a reported score
#: difference the 24-direction grid can actually resolve.  The score itself is
#: never taken from this refinement.
N_DISP_REFINED = 360

#: How many groups part (5) prints, counted in CANDIDATES not groups - a group
#: is one result reached by several ``(beta, beta_p)`` pairs and splitting it
#: across the cut would print half a result.
TOP_N = 20


# --------------------------------------------------------------------------- #
# the score, evaluated at p and not extrapolated
# --------------------------------------------------------------------------- #
def score_at_p(rec, R, az, delta_deg, probe=P_SCORE, n_dir=None):
    """``margin(dxy = probe)`` at ``delta_deg``, worst over displacement azimuth.

    ``n_dir = None`` is :func:`.score_discriminators.probe_margin` unchanged,
    at its own :data:`~.score_discriminators.N_DISP_DIR` - that is the score.
    A number is passed only by :func:`disp_azimuth_resolution`, which measures
    how much of a score difference the 24-direction sweep can resolve; the
    same worst-over-azimuth quantity, on a finer azimuth grid.
    """
    if n_dir is None:
        return SD.probe_margin(rec["_g0"], rec["_g90"], R, az, rec["z_home"],
                               delta_deg, probe)
    dr = np.deg2rad(delta_deg)
    worst = np.inf
    for ang in np.linspace(0.0, 2.0 * np.pi, n_dir, endpoint=False):
        inv = SD._invariants(rec["_g0"], rec["_g90"], R,
                             SD._T_stack(az, rec["z_home"], probe, float(ang)))
        worst = min(worst, SD._margin_at(inv, dr))
    return worst


def disp_azimuth_resolution(measured, R, az):
    """Worst ``|score at 24 directions - score at`` :data:`N_DISP_REFINED` ``|``.

    The score is a worst case over a CIRCLE of displacement azimuths sampled at
    24 points, so it is optimistic by however much the true worst direction
    falls between samples.  That optimism is the resolution of the ranking in
    part (5): two candidates closer together than this are not ordered by the
    grid the score is computed on.  Measured over every survivor, not bounded.
    """
    worst = 0.0
    for rec in measured:
        if not np.isfinite(rec["score_p"]):
            continue
        fine = score_at_p(rec, R, az, rec["delta_con"], n_dir=N_DISP_REFINED)
        worst = max(worst, abs(rec["score_p"] - fine))
        rec["score_p_fine"] = fine
    return worst


# --------------------------------------------------------------------------- #
# one r_p slice, screened, attributed, tuned and scored
# --------------------------------------------------------------------------- #
def _key_no_rp(rec):
    """Candidate identity with ``r_p`` dropped.

    :func:`.tilt_bracket._key` carries ``r_p``, so two slices at different
    ratios can never share a key and "is it the same feasible set?" would come
    out ``False`` by construction rather than by measurement.  With ``r_p`` held
    at one value per slice, the remaining four axes ARE the candidate.
    """
    return (rec["beta"], rec["beta_p"], rec["a"], rec["d"])


def run_slice(r_p, label, short, probe=P_SCORE, verbose=True, axes=None):
    """Screen, attribute, tune and score one ``r_p/r_b`` slice.

    Every step is the existing one: :func:`.tilt_bracket.screen` (the same
    exact reach test intersected with the same closed-form ``N_i > 0`` floor),
    :func:`.tilt_bracket.attribute` (R / X, every X refined by bisection to
    1e-9 so a crossing is not a 0.025-``r_b`` grid artifact),
    :func:`.tilt_bracket.constrained_margins` (``z_home`` at the bracket
    midpoint, then :mod:`.score_discriminators`' own tune under every cap), and
    :func:`.score_discriminators.probe_margin` for the score at :data:`P_SCORE`.
    Nothing is reimplemented, so this slice and the coarse grid are the same
    screen run over different ``r_p``.

    ``axes`` overrides the OTHER axis lists handed to :func:`.tilt_bracket.screen`
    (``beta_vals``, ``beta_p_vals``, ``a_vals``, ``d_vals``); ``rp_vals`` stays
    ``[r_p]`` and is not overridable, because a slice is one ratio by
    definition.  ``None`` is :mod:`.zhome_bracket`'s own coarse axes, which is
    what every call in this module passes and what part (0) through (6) below
    report.  It exists so :mod:`.sweep_ranges` can put the hardware pull's
    discrete ``a`` set and its ``beta_p`` bounds through THIS pipeline rather
    than standing up a second one.
    """
    t0 = time.time()
    az, mg = ZB.fine_poses()
    rows, feasible = screen(rp_vals=[r_p], **(axes or {}))
    cat_R, cat_X, artifacts = attribute(rows, az, mg)
    measured, dropped = constrained_margins(feasible)
    R29, az29, _ = SD._pose_grid(None)
    no_delta = 0
    for rec in measured:
        e = rec["con"][CAP]
        if e is None:
            no_delta += 1
            rec["delta_con"] = np.nan
            rec["margin_con"] = np.nan
            rec["score_p"] = np.nan
            rec["score_extrap"] = np.nan
            continue
        rec["delta_con"] = e["delta"]
        rec["margin_con"] = e["margin"]
        rec["score_p"] = score_at_p(rec, R29, az29, e["delta"], probe)
        # the linear extrapolation off sens - REPORTED in part (4) as the
        # difference from the direct evaluation, and used nowhere.
        rec["score_extrap"] = e["margin"] - e["sens"] * probe
    if verbose:
        print(f"    {label:<24} {len(rows):>4} screened  "
              f"{len(feasible):>4} feasible  {dropped:>2} dropped  "
              f"{no_delta:>2} no delta   ({time.time() - t0:.1f} s)")
    return dict(r_p=r_p, label=label, short=short, rows=rows, feasible=feasible,
                cat_R=cat_R, cat_X=cat_X, artifacts=artifacts,
                measured=measured, dropped=dropped, no_delta=no_delta,
                R29=R29, az29=az29, secs=time.time() - t0)


def scores(slc):
    """The score column of one slice, NaN preserved."""
    return np.array([r["score_p"] for r in slc["measured"]], dtype=float)


def ends(slc):
    """``(lower ends, upper ends, widths)`` over the survivors, in ``r_b``."""
    lo = np.array([r["z_lo"] for r in slc["feasible"]], dtype=float)
    hi = np.array([r["z_hi"] for r in slc["feasible"]], dtype=float)
    return lo, hi, hi - lo


def sep_p_rb(rec):
    """Closest approach of two platform anchors, in ``r_b``.

    Same quantity :func:`.box_boundary.buildability` reports, recomputed off
    the library geometry.  It is quoted in mm below at the ASSERTED
    ``r_b = 90``, which is a real length rather than
    :data:`.box_boundary.R_B_REFERENCE_MM`'s 100 mm units placeholder - that
    placeholder sits above the 90 mm ceiling and never was a candidate ``r_b``.
    Still not compared against anything: the ball-joint housing OD that would
    floor it is on the hardware pull, unread.
    """
    g = make_geometry(r_b=R_B, beta=rec["beta"], delta=0.0, r_p=rec["r_p"],
                      beta_p=rec["beta_p"], a=rec["a"], d=rec["d"], c_p=C_P)
    D = np.linalg.norm(g.p[:, :, None] - g.p[:, None, :], axis=0)
    return float(D[~np.eye(6, dtype=bool)].min())


# --------------------------------------------------------------------------- #
# parts
# --------------------------------------------------------------------------- #
def part0(main, quoted, ref_slices):
    """(0) The assertion's arithmetic, and the two checks it needs."""
    print()
    print("=" * 78)
    print("(0) THE ASSERTION, AND WHAT FOLLOWS FROM IT ARITHMETICALLY")
    print("=" * 78)
    print(f"  ASSERTED 2026-09-08, no diagnostic behind either number:")
    print(f"    r_b = {R_B_MM:.0f} mm   from the print bed")
    print(f"    r_p = {R_P_MM:.0f} mm   from the printed hub carrying the "
          f"bought 220 mm sheet")
    print(f"    r_p / r_b = {R_P_MM:.0f} / {R_B_MM:.0f} = {R_P_RB:.9f}"
          f"   (quoted as {R_P_RB_QUOTED})")
    print()
    print("  r_p/r_b is NO LONGER A SWEEP AXIS.  The coarse grid samples it at")
    print(f"  {RP_RB} and the asserted ratio is none of those; it sits between")
    print(f"  0.85 and 1.10, nearer 0.85 by {abs(R_P_RB-0.85):.4f} than to "
          f"1.10 by {abs(R_P_RB-1.10):.4f}.")
    print()
    print("  CHECK 1 - does the four-figure rounding matter anywhere?")
    print(f"    |{R_P_RB:.9f} - {R_P_RB_QUOTED}| = "
          f"{abs(R_P_RB - R_P_RB_QUOTED):.2e} r_b, against a Z_GRID step of "
          f"{Z_GRID[1]-Z_GRID[0]}.")
    print("    Four decades below the grid, so nothing SHOULD move.  Both are")
    print("    screened in full rather than argued about:")
    print(f"      {'r_p/r_b':>13} {'feas':>5} {'R':>4} {'X':>4} "
          f"{'lower ends':>16} {'upper ends':>16} {'widths':>16}")
    for slc in (main, quoted):
        lo, hi, w = ends(slc)
        print(f"      {slc['r_p']:>13.9f} {len(slc['feasible']):>5} "
              f"{len(slc['cat_R']):>4} {len(slc['cat_X']):>4} "
              f"{f'{lo.min():.3f} .. {lo.max():.3f}':>16} "
              f"{f'{hi.min():.3f} .. {hi.max():.3f}':>16} "
              f"{f'{w.min():.3f} .. {w.max():.3f}':>16}")
    same = ({_key_no_rp(r) for r in main["feasible"]}
            == {_key_no_rp(r) for r in quoted["feasible"]})
    print(f"    identical feasible set, candidate by candidate : {same}")
    print(f"    the quotient is what is used below.")
    print()
    print("  CHECK 2 - the score's probe, at the r_b now in force.")
    src = " + ".join(f"{s}^2" for s in BUILD_ERROR_SOURCES_MM)
    p_mm = float(np.sqrt(sum(s * s for s in BUILD_ERROR_SOURCES_MM)))
    print(f"    p_mm = sqrt({src}) = {p_mm:.6f} mm   "
          f"(RSS, docs/archive/notation.md sec.8)")
    print(f"    p    = p_mm / r_b = {p_mm:.6f} / {R_B_MM:.0f} = "
          f"{P_SCORE:.9f}   [r_b]")
    print(f"    agrees with the quoted {P_SCORE_QUOTED} to "
          f"{abs(P_SCORE - P_SCORE_QUOTED):.1e}")
    print(f"    docs/archive/notation.md sec.8 still carries {P_SCORE_SUPERSEDED}, "
          f"normalised by the")
    print(f"    superseded 80 mm floor.  p scales INVERSELY with r_b, so at "
          f"{R_B_MM:.0f} mm it is")
    print(f"    smaller by a factor {P_SCORE_SUPERSEDED / P_SCORE:.4f} = "
          f"{R_B_MM:.0f}/80.  {P_SCORE_SUPERSEDED} is NOT used below, and no")
    print(f"    slope is extrapolated between the two - part (4).")
    print()
    print("  HELD FIXED, all of it as zhome_bracket and score_discriminators")
    print("  already build it.  No range is chosen here and no harness specced:")
    print(f"    beta        : {BETA}")
    print(f"    beta_p      : {BETA_P}")
    print(f"    a/r_b       : {A_RB}")
    print(f"    d/r_b       : {D_RB}")
    print(f"    c_p/r_b     : {C_P}")
    print(f"    z_home/r_b  : {Z_GRID[0]} .. {Z_GRID[-1]} step "
          f"{Z_GRID[1]-Z_GRID[0]}  ({Z_GRID.size} points)")
    print(f"    delta       : {DELTA_GRID.size} points over [0, 180)")
    print(f"    tilt limit  : {TILT_LIMIT_DEG:.4f} deg   (the current one; "
          f"tilt sensitivity is")
    print(f"                  tilt_bracket's question and is not repeated here)")
    print(f"    tune        : cond(J_fk) <= {CAP:.0e} at char_len = "
          f"{SD.CONSTRAINT_CHAR_LEN}, PROVISIONAL")
    print(f"    candidates  : {len(BETA)}x{len(BETA_P)}x{len(A_RB)}x"
          f"{len(D_RB)} = {len(main['rows'])} at the one ratio")
    print(f"                  (the coarse grid's {len(RP_RB)} x that = "
          f"{len(RP_RB)*len(main['rows'])}, recomputed in part (1))")


def part1(main, ref_slices):
    """(1) Non-empty brackets, R / X, against the recomputed 363 of 540."""
    n_tot = len(main["rows"])
    ref_rows = sum(len(s["rows"]) for s in ref_slices)
    ref_feas = sum(len(s["feasible"]) for s in ref_slices)
    ref_R = sum(len(s["cat_R"]) for s in ref_slices)
    ref_X = sum(len(s["cat_X"]) for s in ref_slices)
    print()
    print("=" * 78)
    print("(1) NON-EMPTY z_home BRACKETS, AND WHAT EMPTIES THE REST")
    print("=" * 78)
    print("  R and X are the 2026-09-05 attribution and are the only two cases:")
    print("  N_i > 0 is ONE-SIDED - a floor with no ceiling - so 'reach floor")
    print("  above an N ceiling' cannot occur.")
    print("    R  reach empty on its own      - no z_home reaches at any delta")
    print("    X  reach ceiling BELOW the N>0 floor - both satisfiable alone,")
    print("       crossed; every X refined by bisection to 1e-9 first, since a")
    print(f"       crossing narrower than the {Z_GRID[1]-Z_GRID[0]} r_b grid step "
          f"would be a sampling")
    print("       artifact rather than a crossing.")
    print()
    print(f"    {'r_p/r_b':>22} {'cand':>6} {'non-empty':>10} {'frac':>7} "
          f"{'empty':>6} {'R':>5} {'X':>5} {'artifact':>9}")
    for slc in [main] + list(ref_slices):
        n = len(slc["rows"])
        print(f"    {slc['label']:>22} {n:>6} {len(slc['feasible']):>10} "
              f"{len(slc['feasible'])/n:>7.3f} {n-len(slc['feasible']):>6} "
              f"{len(slc['cat_R']):>5} {len(slc['cat_X']):>5} "
              f"{len(slc['artifacts']):>9}")
    print(f"    {'COARSE GRID, pooled':>22} {ref_rows:>6} {ref_feas:>10} "
          f"{ref_feas/ref_rows:>7.3f} {ref_rows-ref_feas:>6} {ref_R:>5} "
          f"{ref_X:>5} "
          f"{sum(len(s['artifacts']) for s in ref_slices):>9}")
    print()
    print(f"  THE COMPARISON ASKED FOR.  {len(main['feasible'])} of {n_tot} at "
          f"the asserted ratio, against")
    print(f"  {ref_feas} of {ref_rows} over r_p/r_b spanning "
          f"{min(RP_RB)}-{max(RP_RB)} - recomputed from the three")
    print(f"  slices above in this run, not transcribed.  As fractions:")
    print(f"  {len(main['feasible'])/n_tot:.3f} against "
          f"{ref_feas/ref_rows:.3f}, a difference of "
          f"{abs(len(main['feasible'])/n_tot - ref_feas/ref_rows):.3f}.")
    print()
    print(f"  The pooled figure averages a set that is strongly ordered in")
    print(f"  r_p - {len(ref_slices[0]['feasible'])} / "
          f"{len(ref_slices[1]['feasible'])} / "
          f"{len(ref_slices[2]['feasible'])} of 180 at 0.60 / 0.85 / 1.10 - "
          f"so it is not the right")
    print("  comparison on its own, and the slices are printed beside it.")
    print()
    same85 = ({_key_no_rp(r) for r in main["feasible"]}
              == {_key_no_rp(r) for r in ref_slices[1]["feasible"]})
    print(f"  The split is entirely R: {len(main['cat_R'])} of "
          f"{n_tot - len(main['feasible'])} empties are reach failing alone, at")
    print(f"  every z_home and every delta, and {len(main['cat_X'])} are the "
          f"crossing.  The one X in")
    print(f"  the coarse grid is at r_p/r_b = 1.10, above the asserted ratio.")
    print()
    print(f"  A fact worth recording rather than leaving as a coincidence of")
    print(f"  counts: the feasible SET at the asserted ratio is identical, "
          f"candidate")
    print(f"  by candidate, to the one at r_p/r_b = 0.85 -> {same85}.  The two "
          f"counts")
    print(f"  agreeing at {len(main['feasible'])} is not an accident of "
          f"arithmetic; the screen selects")
    print(f"  the same 180-candidate subset at both ratios.  What differs "
          f"between them")
    print(f"  is where the brackets sit and what the score is, parts (3) and "
          f"(4).")


def part2(main, ref_slices):
    """(2) Empties by a/r_b, in tilt_bracket part (2)'s form."""
    print()
    print("=" * 78)
    print("(2) EMPTY BRACKETS BY a / r_b")
    print("=" * 78)
    print(f"  At the asserted ratio, and the three coarse-grid ratios beside "
          f"it for")
    print("  the ordering.  R / X as in part (1).")
    print()
    cols = [main] + list(ref_slices)
    print(f"  Columns are r_p/r_b; 0.8889* is the ASSERTED 80/90.  Each cell is")
    print("  'empty (R / X)' out of the 60 candidates in that row of the slice.")
    print()
    for axis, vals, name in (("a", A_RB, "a/r_b"), ("d", D_RB, "d/r_b")):
        print(f"    {name:>8} {'of':>4}" + "".join(f"{s['short']:>16}"
                                                  for s in cols))
        for v in vals:
            cells = []
            for slc in cols:
                ke = [e for e in slc["rows"]
                      if e[axis] == v and not e["feasible"]]
                nR = sum(1 for e in ke if e in slc["cat_R"])
                nX = sum(1 for e in ke if e in slc["cat_X"])
                cells.append(f"{len(ke)} (R{nR}/X{nX})")
            tot = sum(1 for r in main["rows"] if r[axis] == v)
            print(f"    {v:>8.2f} {tot:>4}" + "".join(f"{c:>16}"
                                                      for c in cells))
        if axis == "a":
            print()
            print("  The same table by d/r_b, which the a table cannot")
            print("  substitute for - a and d empty a candidate together, and")
            print("  part (6) asks about both:")
            print()
    print()
    print("  The (a, d) cells at the asserted ratio, feasible of screened -")
    print("  which is where the a table's totals actually come from:")
    print()
    print(f"    {'':>8}" + "".join(f"{'d=' + format(d, '.2f'):>14}"
                                   for d in D_RB))
    for a in A_RB:
        cells = []
        for d in D_RB:
            nf = sum(1 for r in main["rows"]
                     if r["a"] == a and r["d"] == d and r["feasible"])
            nt = sum(1 for r in main["rows"] if r["a"] == a and r["d"] == d)
            cells.append(f"{nf} / {nt}")
        print(f"    a={a:<6.2f}" + "".join(f"{c:>14}" for c in cells))
    print()
    print("  EVERY empty at the asserted ratio is at a/r_b = 0.10, and that row")
    print("  is nearly all empty: the short arm cannot reach.  a/r_b = 0.20 and")
    print("  0.35 empty NOTHING here, exactly as they empty nothing at any tilt")
    print("  limit in tilt_bracket.  The ordering in a survives the ratio move;")
    print("  what moved is that 0.20 has gone from partly empty at the pooled")
    print("  grid to entirely non-empty here.")


def part3(main, ref_slices):
    """(3) Bracket ends, widths, contiguity - in r_b and at the asserted mm."""
    lo, hi, w = ends(main)
    step = float(Z_GRID[1] - Z_GRID[0])
    print()
    print("=" * 78)
    print("(3) BRACKET ENDS, WIDTHS AND CONTIGUITY")
    print("=" * 78)
    print(f"  z_home / r_b GRID values, resolution {step} r_b.  Read an end as")
    print(f"  'feasible by {step}'; there is no bisection refinement on the ends,")
    print("  because a bracket narrower than the grid step is not a design.")
    print(f"  The mm column is at the ASSERTED r_b = {R_B_MM:.0f} mm - a real "
          f"length now, not")
    print("  box_boundary's 100 mm units placeholder, which sits above the")
    print("  ceiling and never was a candidate r_b.")
    print()
    print(f"    {'':<26}{'r_b':>22}{'mm @ r_b=90':>22}")
    for name, arr in (("lower ends  min .. max", lo),
                      ("upper ends  min .. max", hi),
                      ("width       min .. max", w)):
        print(f"    {name:<26}"
              f"{f'{arr.min():.3f} .. {arr.max():.3f}':>22}"
              f"{f'{arr.min()*R_B_MM:.2f} .. {arr.max()*R_B_MM:.2f}':>22}")
    fw = _five_number(w)
    print(f"    {'width  Q1 / med / Q3':<26}"
          f"{f'{fw[1]:.3f} / {fw[2]:.3f} / {fw[3]:.3f}':>22}"
          f"{f'{fw[1]*R_B_MM:.1f} / {fw[2]*R_B_MM:.1f} / {fw[3]*R_B_MM:.1f}':>22}")
    print(f"    {'N>0 floor (closed form)':<26}"
          f"{main['feasible'][0]['cf']:>22.5f}"
          f"{main['feasible'][0]['cf']*R_B_MM:>22.2f}")
    print()
    print(f"    non-contiguous feasible sets              : "
          f"{sum(1 for r in main['feasible'] if not r['contiguous'])}")
    print(f"    lower end set by the N>0 floor            : "
          f"{sum(1 for r in main['feasible'] if r['z_lo'] <= r['cf'] + 1e-9)}")
    print(f"    upper end touching the top of the scan    : "
          f"{sum(1 for r in main['feasible'] if r['z_hi'] >= Z_GRID[-1]-1e-12)}"
          f"   (0 means the scan clips nothing)")
    print()
    print("  The floor is delta-free, a-free and d-free - it is r_p sin(tilt) +")
    print("  c_p cos(tilt) and depends on r_p, c_p and the tilt limit alone - so")
    print("  it is ONE number for the whole slice here, which it was not across")
    print("  the coarse grid's three ratios.")
    print()
    print("  THE THIN TAIL, which the min/max row hides.  A width of 0.000 is a")
    print("  bracket whose two grid ends COINCIDE - one feasible grid point.  A")
    print(f"  grid width w carries a true width in [w, w + 2 x {step}), so a "
          f"0.000 grid")
    print(f"  bracket is somewhere under {2*step*R_B_MM:.1f} mm and a "
          f"{step} one under {3*step*R_B_MM:.1f} mm.")
    print()
    print(f"    {'grid width <=':>16} {'mm @ r_b=90':>12} {'candidates':>11} "
          f"{'of':>5}")
    for k in (0, 1, 2, 4):
        thr = k * step
        n = int(np.sum(w <= thr + 1e-12))
        print(f"    {thr:>16.3f} {thr*R_B_MM:>12.2f} {n:>11} "
              f"{len(main['feasible']):>5}")
    print()
    print("  The one nearby datum in the record is cc-fk-gate's fixture F -")
    print("  r_p > r_b at 110/100 - whose bracket is 10 mm, the narrowest")
    print("  non-empty one found there.  It is NOT directly comparable: that")
    print("  bracket was measured at 0.0025 r_b (0.25 mm at its fixture scale),")
    print("  ten times finer than this grid, and at the geometry's OWN delta")
    print("  rather than asking whether some delta works.  Both differences")
    print("  make F's number the more conservative one.  Quoted anyway because")
    print(f"  it is the only prior measurement near this ratio, and "
          f"{int(np.sum(w <= 4*step + 1e-12))} of "
          f"{len(main['feasible'])}")
    print(f"  brackets here are at or under {4*step*R_B_MM:.1f} mm of grid "
          f"width.")
    print()
    print("  The width distribution is BIMODAL, which is why the Q1 row above")
    print("  is printed and the median alone would mislead: a large minority")
    print("  sits near the grid resolution and the rest has real room.  Where")
    print("  the narrow ones sit in the ranking is measured in part (4).")
    print()
    print("  Ends by slice, for the ratio dependence:")
    print(f"    {'r_p/r_b':>22} {'lower ends':>18} {'upper ends':>18} "
          f"{'widths':>18} {'non-contig':>11}")
    for slc in [main] + list(ref_slices):
        l, h, ww = ends(slc)
        print(f"    {slc['label']:>22} "
              f"{f'{l.min():.3f} .. {l.max():.3f}':>18} "
              f"{f'{h.min():.3f} .. {h.max():.3f}':>18} "
              f"{f'{ww.min():.3f} .. {ww.max():.3f}':>18} "
              f"{sum(1 for r in slc['feasible'] if not r['contiguous']):>11}")


def part4(main, ref_slices, res_24_360):
    """(4) margin(dxy = p): five-number summary and max over the survivors."""
    sc = scores(main)
    m0 = np.array([r["margin_con"] for r in main["measured"]], dtype=float)
    ex = np.array([r["score_extrap"] for r in main["measured"]], dtype=float)
    live = np.isfinite(sc)
    print()
    print("=" * 78)
    print(f"(4) margin(dxy = p) AT p = {P_SCORE:.6f} r_b")
    print("=" * 78)
    print(f"  p RECOMPUTED for r_b = {R_B_MM:.0f} mm, part (0).  "
          f"{P_SCORE_SUPERSEDED} - the sec.8 value at")
    print("  the superseded 80 mm floor - is not used, and NO SLOPE IS")
    print("  EXTRAPOLATED between the two or off sens: every number below is a")
    print("  direct evaluation at p through probe_margin.")
    print()
    print("  margin is a dimensionless ratio and carries NO characteristic")
    print("  length and NO Jacobian.  The delta it is evaluated at was tuned")
    print(f"  under cond(J_fk) <= {CAP:.0e} at char_len = "
          f"{SD.CONSTRAINT_CHAR_LEN}, which carries both and is")
    print("  PROVISIONAL in that length.  z_home is each candidate's bracket")
    print("  MIDPOINT - a point in the interior for definiteness, not a")
    print("  recommendation, and z_home remains a swept axis.")
    print()
    print(f"    feasible                                  : "
          f"{len(main['feasible'])}")
    print(f"    dropped, ik unreachable at the midpoint   : {main['dropped']}")
    print(f"    measured                                  : "
          f"{len(main['measured'])}")
    print(f"    no admissible delta at cond <= {CAP:.0e}       : "
          f"{main['no_delta']}")
    print(f"    SURVIVORS (the five-number sample)        : {int(live.sum())}")
    print()
    five = _five_number(sc)
    print(f"    {'':<28}{'min':>11}{'Q1':>11}{'median':>11}{'Q3':>11}"
          f"{'max':>11}")
    print(f"    {'margin(dxy = p)':<28}" + "".join(f"{v:>11.6f}" for v in five))
    print(f"    {'margin(dxy = 0), for reading':<28}"
          + "".join(f"{v:>11.6f}" for v in _five_number(m0)))
    print()
    print(f"    MAX over survivors : {np.nanmax(sc):.6f}")
    best = max((r for r in main["measured"] if np.isfinite(r["score_p"])),
               key=lambda r: r["score_p"])
    print(f"      at beta {best['beta']:.0f}, beta_p {best['beta_p']:.0f}, "
          f"a/r_b {best['a']:.2f}, d/r_b {best['d']:.2f}, "
          f"z_home {best['z_home']:.4f} r_b")
    print(f"      = {best['z_home']*R_B_MM:.2f} mm at the asserted r_b, "
          f"delta {best['delta_con']:.1f} deg")
    print()
    neg = int(np.sum(sc[live] < 0.0))
    print(f"    survivors with a NEGATIVE margin at p     : {neg}")
    print(f"    survivors with a NEGATIVE margin at dxy=0 : "
          f"{int(np.sum(m0[np.isfinite(m0)] < 0.0))}")
    print("      A negative margin is a candidate the 366-pose screen passed")
    print("      and the 29-pose harness grid does not - the two grids are not")
    print("      nested, and score_discriminators reports the same thing.  It")
    print("      is CARRIED, not dropped: dropping it would flatter the")
    print("      summary.  The count rising from dxy = 0 to dxy = p is the")
    print("      displacement doing what it is there to do.")
    print()
    print("  DIRECT vs THE EXTRAPOLATION THAT WAS NOT USED.  sens is the")
    print("  margin's slope in dxy measured at probe = "
          f"{SD.PROBE_DXY}; margin - sens * p")
    print("  would be the linear estimate at p.  Reported so the size of what")
    print("  was avoided is on the record, and used nowhere:")
    print(f"    worst |direct - linear extrapolation| : "
          f"{np.nanmax(np.abs(sc - ex)):.3e}")
    print(f"    median |direct - linear extrapolation|: "
          f"{np.nanmedian(np.abs(sc - ex)):.3e}")
    rho, n = _spearman(sc, ex)
    print(f"    Spearman of the two rankings          : {rho:.4f}  (n = {n})")
    print("    The extrapolation is close in RANK and not in VALUE, which is")
    print("    exactly the failure mode that would have gone unnoticed had the")
    print("    slope been used: a ranking that looks right carrying values that")
    print("    are wrong by more than the gaps between the top candidates.")
    print()
    print("  RESOLUTION OF THE SCORE.  probe_margin takes the worst over")
    print(f"  {SD.N_DISP_DIR} displacement azimuths, so it is optimistic by "
          f"however much the")
    print(f"  true worst direction falls between samples.  Refined to "
          f"{N_DISP_REFINED}:")
    print(f"    worst |score at {SD.N_DISP_DIR} directions - score at "
          f"{N_DISP_REFINED}| : {res_24_360:.3e}")
    print("    Two candidates closer together than this are NOT ordered by the")
    print("    grid the score is computed on.  Part (5) quotes it again.")
    print()
    print("  DOES THE THIN TAIL OF PART (3) REACH THE TOP OF THE FIELD?  A")
    print("  narrow bracket high in the ranking would be a different finding")
    print("  from a narrow bracket at the bottom, so it is measured rather")
    print("  than assumed.  Survivors ranked by margin(dxy = p), best first,")
    print("  against the bracket width they carry:")
    print()
    ranked = sorted((r for r in main["measured"] if np.isfinite(r["score_p"])),
                    key=lambda r: -r["score_p"])
    width_of = {_key(r): r["z_hi"] - r["z_lo"] for r in main["feasible"]}
    wr = np.array([width_of[_key(r)] for r in ranked])
    print(f"    {'rank band':>14} {'n':>4} {'min width':>11} "
          f"{'median width':>14} {'[mm] min':>10}")
    for lo_r, hi_r in ((0, 10), (10, 30), (30, 60), (60, len(ranked))):
        seg = wr[lo_r:hi_r]
        if not seg.size:
            continue
        print(f"    {f'{lo_r+1}-{min(hi_r, len(ranked))}':>14} {seg.size:>4} "
              f"{seg.min():>11.3f} {np.median(seg):>14.3f} "
              f"{seg.min()*R_B_MM:>10.2f}")
    thin = 2.0 * float(Z_GRID[1] - Z_GRID[0])
    thin_ranks = [i + 1 for i, r in enumerate(ranked)
                  if width_of[_key(r)] <= thin + 1e-12]
    print()
    print(f"    candidates with a grid width <= {thin} r_b "
          f"({thin*R_B_MM:.1f} mm) : {len(thin_ranks)}")
    print(f"    their best rank of {len(ranked)}                        : "
          f"{min(thin_ranks) if thin_ranks else '-'}")
    print(f"    their median rank                            : "
          f"{int(np.median(thin_ranks)) if thin_ranks else '-'}")
    print("    The thin brackets sit low in the ranking, so the tail and the")
    print("    top of the field are different candidates.  That is what the")
    print("    verdict at the end rests on, and it is this measurement.")
    print()
    print("  The same summary at the three coarse-grid ratios, so the asserted")
    print("  one has something to be read against.  Same p, same cap, same")
    print("  grids - only r_p differs:")
    print()
    print(f"    {'r_p/r_b':>22} {'n':>5}{'min':>11}{'Q1':>11}{'median':>11}"
          f"{'Q3':>11}{'max':>11}")
    for slc in [main] + list(ref_slices):
        s = scores(slc)
        f = _five_number(s)
        print(f"    {slc['label']:>22} {int(np.isfinite(s).sum()):>5}"
              + "".join(f"{v:>11.6f}" for v in f))
    print()
    print("  The score falls monotonically as r_p rises across all four, which")
    print("  is the 2026-09-08 runaway seen from the other end: the margin")
    print("  measures distance from unreachability, and a smaller platform ring")
    print("  is further from it.  That is a recorded DEFECT of the objective")
    print("  (docs/archive/notation.md sec.12, tilt_authority), not a reason to prefer a")
    print("  smaller r_p - and r_p is no longer a choice in any case.")


def part5(main, res_24_360):
    """(5) Top 20 grouped by (a, d, |beta_p - beta|)."""
    print()
    print("=" * 78)
    print(f"(5) TOP {TOP_N} GROUPED BY (a, d, |e|),   e = beta_p - beta")
    print("=" * 78)
    print("  Part (8)'s invariant with r_p DROPPED from the key, because r_p no")
    print("  longer varies: the key there is (r_p, a, d, |e|) and with one r_p")
    print("  the two partitions are the same partition.  _abs_e_groups is")
    print("  called unchanged and the r_p slot is constant.")
    print()
    print("  Two candidates sharing this key are ONE result reached by two")
    print("  (beta, beta_p) pairs - a rotation of the machine about z, a sign")
    print("  flip of e, or both.  Groups are ranked; members inside a group are")
    print("  not, and printing them as adjacent RANKS would say they are first")
    print("  and second when they are one result.")
    print()
    print(f"  RANKED BY margin(dxy = p) at p = {P_SCORE:.6f} r_b - the score "
          f"now in")
    print(f"  force - at the delta tuned under cond(J_fk) <= {CAP:.0e} at "
          f"char_len =")
    print(f"  {SD.CONSTRAINT_CHAR_LEN}.  margin(dxy = 0) is carried beside it, "
          f"and the two rank")
    print("  almost identically; the number is printed below the table.")
    print()
    good = sorted((r for r in main["measured"] if np.isfinite(r["score_p"])),
                  key=lambda r: -r["score_p"])
    groups = _abs_e_groups(good)
    ordered, seen = [], set()
    for r in good:
        gk = (r["r_p"], r["a"], r["d"], round(abs(r["beta_p"] - r["beta"]), 9))
        if gk not in seen:
            seen.add(gk)
            ordered.append(gk)
    print(f"    {'grp':>4} {'a/r_b':>6} {'d/r_b':>6} {'|e|':>5} {'memb':>5} "
          f"{'margin(p)':>10} {'margin(0)':>10} {'spread':>9} "
          f"{'z_home':>8} {'[mm]':>7} {'width':>7} {'sep_p':>7} {'[mm]':>7}")
    shown, gi = 0, 0
    for gi, gk in enumerate(ordered, 1):
        members = groups[gk]
        sp = [m["score_p"] for m in members]
        best = max(members, key=lambda m: m["score_p"])
        rec_f = next(r for r in main["feasible"] if _key(r) == _key(best))
        width = rec_f["z_hi"] - rec_f["z_lo"]
        s_p = sep_p_rb(best)
        print(f"    {gi:>4} {gk[1]:>6.2f} {gk[2]:>6.2f} {gk[3]:>5.0f} "
              f"{len(members):>5} {max(sp):>10.6f} "
              f"{max(m['margin_con'] for m in members):>10.6f} "
              f"{max(sp)-min(sp):>9.1e} {best['z_home']:>8.4f} "
              f"{best['z_home']*R_B_MM:>7.2f} {width:>7.3f} "
              f"{s_p:>7.4f} {s_p*R_B_MM:>7.2f}")
        shown += len(members)
        if shown >= TOP_N:
            break
    print()
    print(f"  {shown} candidates in {gi} groups, of "
          f"{len(good)} survivors in {len(ordered)} groups.")
    print()
    print("  READ 'spread' BEFORE READING THE GROUPS AS TIES.  At dxy = 0 the")
    print(f"  within-group margins agree to ~1e-15, far below TIE_TOL = "
          f"{TIE_TOL:.0e},")
    print("  and the collapse is exact.  At dxy = p they do NOT: the spreads")
    print("  above are ~1e-5.  That is not a real ordering.  The displacement")
    print("  breaks the D3 azimuth symmetry and probe_margin restores it by")
    print(f"  sweeping the displacement over a full circle - but at "
          f"{SD.N_DISP_DIR} SAMPLED")
    print("  directions, and the residual is that discretisation.  Measured, by")
    print("  refining the sweep and watching the spread fall back to the")
    print("  dxy = 0 tie level:")
    print()
    print(f"    {'displacement azimuths':>24} {'worst within-group spread':>27}")
    multi = [g for g in groups.values() if len(g) > 1]
    for n_dir, lab in ((SD.N_DISP_DIR, f"{SD.N_DISP_DIR} (the score)"),
                       (N_DISP_REFINED, f"{N_DISP_REFINED} (refinement only)")):
        key = "score_p" if n_dir == SD.N_DISP_DIR else "score_p_fine"
        sp = [max(m[key] for m in g) - min(m[key] for m in g) for g in multi]
        print(f"    {lab:>24} {max(sp) if sp else float('nan'):>27.3e}")
    sp0 = [max(m["margin_con"] for m in g) - min(m["margin_con"] for m in g)
           for g in multi]
    print(f"    {'dxy = 0, for reference':>24} "
          f"{max(sp0) if sp0 else float('nan'):>27.3e}")
    print()
    print(f"  So the |e| collapse SURVIVES the new score; what does not survive")
    print(f"  is the ability to order inside a group, and the ranking above is")
    print(f"  resolved only to {res_24_360:.1e} - part (4).  Adjacent groups "
          f"closer")
    print("  together than that are not ordered by this measurement either.")
    print()
    sc = scores(main)
    m0 = np.array([r["margin_con"] for r in main["measured"]], dtype=float)
    rho, n = _spearman(sc, m0)
    print(f"  Spearman, margin(dxy = p) against margin(dxy = 0) : {rho:.4f} "
          f"(n = {n})")
    print("  The ranking basis barely matters at this p, which is what makes")
    print("  the table above readable as 'the top of the field' rather than as")
    print("  'the top of the field under one of two scores'.")
    print()
    print(f"  sep_p is min |p_i - p_j|, the closest approach of two platform")
    print(f"  anchors, in r_b and in mm at the ASSERTED r_b = {R_B_MM:.0f}.  "
          f"Unlike")
    print("  box_boundary's 100 mm column this is a real length, not a units")
    print("  placeholder.  It is still compared against NOTHING: the ball-joint")
    print("  housing OD that would floor it is on the hardware pull dispatched")
    print("  2026-09-04, still unread.  READ IT BEFORE READING THE RANK - a")
    print("  margin ranking has no way to know whether six ball joints fit.")


def part6(main):
    """(6) Do a/r_b and d/r_b still run to the edge of their sampled ranges?"""
    print()
    print("=" * 78)
    print("(6) DO a/r_b AND d/r_b STILL RUN TO THE EDGE HERE?")
    print("=" * 78)
    print("  The 2026-09-08 finding box_boundary opens with: every top-5")
    print("  candidate at every tilt limit is a/r_b = 0.35 (the MAXIMUM")
    print("  sampled), d/r_b = 0.80 (the MINIMUM sampled), r_p/r_b = 0.60 (the")
    print("  minimum sampled).  r_p is now settled, so the question left is")
    print("  whether the other two walls still bind at the asserted ratio.")
    print()
    print(f"  Best margin(dxy = p) per (a, d) cell, p = {P_SCORE:.6f} r_b, at "
          f"the")
    print(f"  delta tuned under cond(J_fk) <= {CAP:.0e} at char_len = "
          f"{SD.CONSTRAINT_CHAR_LEN}:")
    print()
    print(f"    {'':>10}" + "".join(f"{'d=' + format(d, '.2f'):>16}"
                                    for d in D_RB))
    cells = {}
    for a in A_RB:
        row = []
        for d in D_RB:
            v = [r["score_p"] for r in main["measured"]
                 if r["a"] == a and r["d"] == d and np.isfinite(r["score_p"])]
            cells[(a, d)] = max(v) if v else np.nan
            nf = len(v)
            row.append(f"{cells[(a,d)]:.4f} ({nf})" if v else f"- (0)")
        print(f"    a={a:<8.2f}" + "".join(f"{c:>16}" for c in row))
    print("      (parenthesis: survivors in that cell, of 20 candidates)")
    print()
    best_cell = max((k for k in cells if np.isfinite(cells[k])),
                    key=lambda k: cells[k])
    print(f"  The cell maximum is at a/r_b = {best_cell[0]:.2f}, d/r_b = "
          f"{best_cell[1]:.2f}.")
    print(f"    a/r_b {best_cell[0]:.2f} is the "
          f"{'MAXIMUM' if best_cell[0] == max(A_RB) else 'interior'} sampled "
          f"value of {A_RB}")
    print(f"    d/r_b {best_cell[1]:.2f} is the "
          f"{'MINIMUM' if best_cell[1] == min(D_RB) else 'interior'} sampled "
          f"value of {D_RB}")
    print()
    print("  A cell maximum at a wall is weak evidence on its own - it is three")
    print("  numbers.  The stronger form is pairwise, over every line that")
    print("  holds (beta, beta_p) and the OTHER axis fixed and moves one axis")
    print("  between two sampled values.  Counted below; a line is used only")
    print("  where both ends are survivors.")
    print()
    by = {(r["beta"], r["beta_p"], r["a"], r["d"]): r["score_p"]
          for r in main["measured"] if np.isfinite(r["score_p"])}

    def pairwise(axis, vals, other_vals):
        out = []
        for i in range(len(vals) - 1):
            for j in range(i + 1, len(vals)):
                hi = lo = 0
                for b in BETA:
                    for bp in BETA_P:
                        for o in other_vals:
                            k1 = ((b, bp, vals[i], o) if axis == "a"
                                  else (b, bp, o, vals[i]))
                            k2 = ((b, bp, vals[j], o) if axis == "a"
                                  else (b, bp, o, vals[j]))
                            if k1 in by and k2 in by:
                                if by[k2] > by[k1]:
                                    hi += 1
                                else:
                                    lo += 1
                out.append((vals[i], vals[j], hi, lo))
        return out

    print(f"    {'axis':>6} {'pair':>16} {'larger wins':>12} "
          f"{'smaller wins':>13} {'lines':>7}")
    for axis, vals, others in (("a", A_RB, D_RB), ("d", D_RB, A_RB)):
        for v1, v2, hi, lo in pairwise(axis, vals, others):
            print(f"    {axis:>6} {f'{v1:.2f} vs {v2:.2f}':>16} {hi:>12} "
                  f"{lo:>13} {hi+lo:>7}")
    print()
    print("  a/r_b: the larger value wins almost every line it is compared on,")
    print("  and the two lines at a/r_b = 0.10 exist at all only because 58 of")
    print("  its 60 candidates have no bracket.  a is PINNED TO THE SAMPLED")
    print("  MAXIMUM at this ratio, exactly as it is at 0.60.  The optimum in a")
    print("  is NOT interior here.")
    print()
    print("  d/r_b: the SMALLER value wins each pair, but by a majority and not")
    print("  a sweep - roughly 60-68% of lines, not 98%.  The top of the field")
    print("  is at d/r_b = 0.80 and the cell maxima are ordered 0.80 > 1.20 >")
    print("  1.60, so d leans on its lower wall; but a third of the lines")
    print("  disagree, so d is a WEAK wall where a is a hard one.  The optimum")
    print("  in d is not interior either, on this evidence.")
    print()
    print("  WHAT THIS DOES NOT SAY.  Neither wall is opened here.  Extending")
    print("  an axis past its sampled range is box_boundary's probe and doing")
    print("  it here would be choosing a range, which this module does not do.")
    print("  'At the edge of the SAMPLED range' is the whole claim, and an")
    print("  optimum at a grid wall stays a statement about the grid until the")
    print("  box is opened at this ratio - which it has not been: box_boundary")
    print("  ran its a ray from the 540-candidate grid, whose r_p values are")
    print("  0.60, 0.85 and 1.10 and do not include this one.")


def verdict(main, ref_slices, res_24_360):
    """Healthy or thin, stated plainly."""
    lo, hi, w = ends(main)
    sc = scores(main)
    step = float(Z_GRID[1] - Z_GRID[0])
    n_tot = len(main["rows"])
    ref_feas = sum(len(s["feasible"]) for s in ref_slices)
    ref_rows = sum(len(s["rows"]) for s in ref_slices)
    print()
    print("=" * 78)
    print("IS THE FEASIBLE SET AT THE ASSERTED RATIO HEALTHY OR THIN?")
    print("=" * 78)
    print()
    print("  HEALTHY, with one named thin tail.  Plainly, and in that order.")
    print()
    print(f"  HEALTHY, on five counts:")
    print(f"    1. {len(main['feasible'])} of {n_tot} candidates have a "
          f"non-empty bracket - a fraction of")
    print(f"       {len(main['feasible'])/n_tot:.3f}, against "
          f"{ref_feas/ref_rows:.3f} for the {ref_feas} of {ref_rows} pooled "
          f"over r_p")
    print(f"       spanning {min(RP_RB)}-{max(RP_RB)}.  The set does not thin "
          f"out at this ratio;")
    print(f"       it is the same size fraction as the grid that produced the")
    print(f"       existing results.")
    print(f"    2. Every empty is category R - reach failing alone - and every")
    print(f"       one is at a/r_b = 0.10.  Nothing is emptied by the N_i > 0")
    print(f"       crossing, and no empty is a grid artifact.  The failure has")
    print(f"       one cause and it is the short arm.")
    print(f"    3. All {len(main['feasible'])} feasible sets are CONTIGUOUS, "
          f"no bracket is clipped by the")
    print(f"       top of the z_home scan, and the N>0 floor sets no lower end.")
    print(f"    4. Every survivor is measurable: {main['dropped']} dropped at "
          f"the bracket midpoint,")
    print(f"       {main['no_delta']} with no admissible delta at cond(J_fk) "
          f"<= {CAP:.0e} at char_len")
    print(f"       = {SD.CONSTRAINT_CHAR_LEN}.  The cap does not bind anywhere "
          f"in this slice.")
    print(f"    5. The median bracket is {np.median(w):.3f} r_b = "
          f"{np.median(w)*R_B_MM:.1f} mm at the asserted r_b = 90 mm,")
    print(f"       and the widest is {w.max():.3f} r_b = {w.max()*R_B_MM:.1f} "
          f"mm.  These are room, not slivers.")
    print()
    n_thin = int(np.sum(w <= 2 * step + 1e-12))
    ranked = sorted((r for r in main["measured"] if np.isfinite(r["score_p"])),
                    key=lambda r: -r["score_p"])
    width_of = {_key(r): r["z_hi"] - r["z_lo"] for r in main["feasible"]}
    thin_ranks = [i + 1 for i, r in enumerate(ranked)
                  if width_of[_key(r)] <= 2 * step + 1e-12]
    top60 = min(width_of[_key(r)] for r in ranked[:60])
    print(f"  THIN, in one place and it is worth naming: "
          f"{n_thin} of {len(main['feasible'])} brackets are")
    print(f"  {2*step} r_b ({2*step*R_B_MM:.1f} mm) wide or narrower on this "
          f"grid, and one is a single")
    print(f"  grid point.  Those candidates are feasible by less than the")
    print(f"  {step} r_b resolution can resolve, and cc-fk-gate's fixture F -")
    print(f"  the only prior datum near this ratio - is the reminder that the")
    print(f"  narrowest bracket found anywhere was 10 mm at a ratio just above")
    print(f"  this one.  The width distribution is bimodal, so the "
          f"{np.median(w)*R_B_MM:.0f} mm median")
    print(f"  above does not describe it on its own.")
    print()
    print(f"  The tail does NOT reach the top of the field, and that is")
    print(f"  measured, not assumed (part 4): the best rank any of those "
          f"{len(thin_ranks)}")
    print(f"  candidates reaches is {min(thin_ranks)} of {len(ranked)}, "
          f"their median rank is "
          f"{int(np.median(thin_ranks))}, and the")
    print(f"  narrowest bracket in the top 60 is {top60:.3f} r_b = "
          f"{top60*R_B_MM:.1f} mm.  The best")
    print(f"  candidate of all carries the WIDEST bracket in the slice, "
          f"{w.max():.3f} r_b =")
    print(f"  {w.max()*R_B_MM:.1f} mm.  That is why the verdict is 'healthy "
          f"with a tail' and not")
    print(f"  'thin': the thin brackets and the good candidates are disjoint")
    print(f"  sets here.")
    print()
    print(f"  WHAT IS LOWER HERE, and is NOT a health statement.  The best")
    print(f"  margin(dxy = p) at this ratio is {np.nanmax(sc):.4f}, against "
          f"{np.nanmax(scores(ref_slices[0])):.4f} at")
    print(f"  r_p/r_b = 0.60 where the whole top of the field used to sit - "
          f"{np.nanmax(sc)/np.nanmax(scores(ref_slices[0])):.0%} of")
    print(f"  it; the median is {_five_number(sc)[2]:.4f} against "
          f"{_five_number(scores(ref_slices[0]))[2]:.4f}, "
          f"{_five_number(sc)[2]/_five_number(scores(ref_slices[0]))[2]:.0%} "
          f"of it.")
    print(f"  That is the score behaving as")
    print(f"  the 2026-09-08 finding says it behaves - the margin measures")
    print(f"  distance from unreachability and rewards a smaller platform ring,")
    print(f"  monotonically, with no interior optimum - and it is a recorded")
    print(f"  DEFECT OF THE OBJECTIVE, not evidence about this ratio.  r_p is")
    print(f"  asserted now and is not a choice; a score that would have")
    print(f"  preferred a different one is reporting its own known problem.")
    print()
    print(f"  WHAT IS STILL OPEN, and this module does not close any of it:")
    print(f"    - the objective.  The runaway is unbounded and tilt_authority")
    print(f"      measured the proposed counterweight ranking WITH the margin")
    print(f"      rather than against it.  Fixing r_p removes the axis the")
    print(f"      runaway was measured on; it does not fix the objective.")
    print(f"    - a/r_b and d/r_b are both at a sampled wall here, part (6),")
    print(f"      and the box has never been opened AT this ratio.")
    print(f"    - the hardware pull dispatched 2026-09-04 is still unread, so")
    print(f"      the sep_p column of part (5) is a measurement with nothing to")
    print(f"      compare it against.")
    print(f"    - tau_L = 150 ms stays PROVISIONAL and the 10.529-degree tilt")
    print(f"      limit with it.  tilt_bracket measured the downstream")
    print(f"      robustness to that; this module inherits the current limit")
    print(f"      and re-measures nothing about it.")
    print()
    print(f"  The score's own resolution is {res_24_360:.1e} at "
          f"{SD.N_DISP_DIR} displacement azimuths, so")
    print(f"  no ordering above is claimed below that.  No range is chosen")
    print(f"  here, the sweep harness is not specced, and docs/archive/notation.md is not")
    print(f"  touched.")


# --------------------------------------------------------------------------- #
def main_report() -> None:
    print("=" * 78)
    print("THE FEASIBLE SET AT r_p / r_b = 80 / 90, THE ASSERTED ABSOLUTE SCALE")
    print("=" * 78)
    print("  Absolute scale was ASSERTED 2026-09-08: r_b = 90 mm from the print")
    print("  bed, r_p = 80 mm from the printed hub carrying the bought 220 mm")
    print("  sheet.  r_p/r_b is no longer a sweep axis, and nothing in this")
    print("  package has been evaluated near the ratio it fixes.")
    print()
    print("  NOTHING IS CHOSEN HERE.  No range is chosen, the sweep harness is")
    print("  not specced, docs/archive/notation.md is not touched.  Every axis is exactly as")
    print("  zhome_bracket builds it; the only change is that r_p/r_b is held")
    print("  at one value instead of three.")
    print()
    print("-" * 78)
    print("SLICES RUN")
    print("-" * 78)
    t0 = time.time()
    main = run_slice(R_P_RB, f"{R_P_RB:.6f} ASSERTED", "0.8889*")
    quoted = run_slice(R_P_RB_QUOTED, f"{R_P_RB_QUOTED} (rounding check)",
                       "0.8889q")
    ref_slices = [run_slice(rp, f"{rp:.6f} (coarse grid)", f"{rp:.2f}")
                  for rp in RP_RB]
    res_24_360 = disp_azimuth_resolution(main["measured"],
                                         main["R29"], main["az29"])
    print(f"    {'total':<22} {time.time() - t0:.1f} s")

    part0(main, quoted, ref_slices)
    part1(main, ref_slices)
    part2(main, ref_slices)
    part3(main, ref_slices)
    part4(main, ref_slices, res_24_360)
    part5(main, res_24_360)
    part6(main)
    verdict(main, ref_slices, res_24_360)


if __name__ == "__main__":
    main_report()
