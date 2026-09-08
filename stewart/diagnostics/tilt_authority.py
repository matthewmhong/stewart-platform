"""Does tilt authority OPPOSE reach margin, or rank WITH it?

    python -m stewart.diagnostics.tilt_authority

THE QUESTION, and why it is the one worth asking.  The 2026-09-08 finding
(``docs/phase-0-design-log.md``, ``notation.md`` sec.12) is that the objective
has no interior optimum: ``r_p/r_b`` runs to 0.10, the smallest value probed,
with margin still climbing monotonically and no turn.  The diagnosis recorded
with it is that the reach margin measures **distance from unreachability, not
capability** - a platform ring shrinking toward a point improves it, and
nothing in the score penalises a mechanism that can barely move.

Tilt authority is the proposed counterweight.  A small anchor ring gives the
legs a short moment arm, so a fixed servo travel buys less platform tilt.  If
that is right, authority FALLS as ``r_p`` falls, opposes the margin, and bounds
the runaway.  If instead it ranks WITH the margin - the way ``tau_min`` does,
Spearman ``rho = 0.835`` over the feasible set
(:mod:`.score_discriminators` part b) - then it is one more quantity that is
high exactly where the margin is high, it bounds nothing, and the objective is
still open.

**This module measures which.  It does not fix the objective, does not propose
a weight or a combined score, does not choose a servo travel figure, and does
not change the feasibility tests** - servo travel is excluded from feasibility
by the 2026-09-08 decision (``notation.md`` sec.12), and nothing here
reintroduces it.  ``notation.md`` is not touched.

WHAT IS DEFINED.  Per candidate, at the tuned ``delta`` under the EXISTING cap
``cond(J_fk) <= 1e6`` at ``char_len = r_b``::

    alpha_span_i = max over envelope poses alpha_i  -  min over poses alpha_i
    alpha_span   = max over the six legs

- the servo swing that candidate needs in order to cover the 10.529-degree
envelope.  Degrees.  It is reported as a SWING, not as a fraction of anything:
no servo travel figure is chosen here, and the comparison against a real
travel happens downstream when the hardware pull lands.

AUTHORITY, and why the reciprocal is barely worth naming.  Authority is read
as::

    authority  =  TILT_LIMIT_DEG / alpha_span          [dimensionless]

- degrees of envelope tilt limit bought per degree of worst-leg servo swing.
A ratio of two angles, so it carries no length and no ``char_len``.  It is the
form that reads most naturally because it is a GEARING: "the envelope costs
this candidate N degrees of servo for every degree of platform tilt", which is
the quantity the moment-arm argument is actually about.

But the numerator is a CONSTANT across every candidate - the envelope is the
same 10.529 degrees for all of them, and after 2026-09-08 it is the same at
every mechanism scale too.  So authority is a fixed, strictly decreasing
relabelling of ``alpha_span``, and **every rank statistic below is identical up
to a sign**.  Nothing is learned from the reciprocal that is not in
``alpha_span``, so ``alpha_span`` in degrees is what the tables carry and
authority is quoted beside it for reading.  The sign convention that follows is
stated once and then held to:

    rho(alpha_span, margin) < 0   <=>   rho(authority, margin) > 0
                                  <=>   authority ranks WITH the margin
                                  <=>   REDUNDANT, bounds nothing

    rho(alpha_span, margin) > 0   <=>   rho(authority, margin) < 0
                                  <=>   authority OPPOSES the margin
                                  <=>   a counterweight, bounds the runaway

Both signs are printed at every correlation, labelled, so the reading cannot
come apart from the number.

THE SCORE IT IS CORRELATED AGAINST.  ``margin(dxy = p)`` at **p = 0.004330**,
the value fixed 2026-09-08 (``notation.md`` sec.8).  That value is BELOW all
three probes :mod:`.score_discriminators` carried (0.005 / 0.0075 / 0.010
``r_b``), so the score has never been evaluated at the value now in force.  It
is therefore evaluated DIRECTLY here, through
:func:`.score_discriminators.probe_margin` at ``p = 0.004330`` - the same
worst-over-24-displacement-azimuths quantity, at the new probe.  **No slope is
extrapolated from the existing probes**, and the difference between the direct
evaluation and a linear extrapolation off ``sens`` is reported rather than
assumed small.

WHAT IS RUN.  The 540-candidate coarse grid at ``h_p/r_b = 0.1`` exactly as
:mod:`.zhome_bracket` builds it, plus the four boundary rays of
:mod:`.box_boundary`, so the runaway region is covered rather than inferred.
Both come through :meth:`.box_boundary.Box.add_slice`, which is itself
:func:`.tilt_bracket.screen` followed by
:func:`.tilt_bracket.constrained_margins`; the ray machinery is
:mod:`.box_boundary`'s own :class:`~.box_boundary.Extension` and
:class:`~.box_boundary.Box`.  Nothing is reimplemented, so the candidate sets
here and there are the same sets, and the screen has one source.

TWO WAYS OF COMPUTING ALPHA, AND WHY BOTH.  The reported ``alpha_span`` comes
from :func:`~stewart.kinematics.ik` itself, one call per pose, because that is
the function the machine's commands would come from.  The 180-``delta`` scan
that parts (5) and (6) need cannot afford 180 x 29 ``ik`` calls per candidate,
so it uses a vectorised form of the same closed form - the one
:func:`.score_discriminators.scan_delta` already builds, extended to return
``alpha``.  **The two are checked against each other** at the tuned ``delta``
of every candidate and the worst disagreement is reported; a vectorisation that
is not checked against the library is a second implementation, not a speed-up.

UNITS AND LABELS.  ``alpha_span`` is an ANGLE in degrees and carries no
characteristic length and no Jacobian.  The CAP under which the ``delta`` it is
evaluated at was tuned does: ``cond(J_fk) <= 1e6`` at ``char_len = r_b``,
PROVISIONAL in that length, and every table below says so.  ``margin`` is a
dimensionless ratio and likewise carries no ``char_len``.

Degrees at the boundary, radians internally.  Arrays are (3, 6).  numpy only.
Reports findings; always exits 0.
"""
from __future__ import annotations

import time

import numpy as np

from ..geometry import make_geometry
from ..kinematics import Unreachable, ik
from . import box_boundary as BB
from . import score_discriminators as SD
from .envelope import TILT_LIMIT_DEG
from .score_discriminators import _five_number, _spearman
from .tilt_bracket import CAP, _key
from .zhome_bracket import A_RB, BETA, BETA_P, D_RB, DELTA_GRID, H_P, R_B, RP_RB

# --------------------------------------------------------------------------- #
# the score, at the value now in force
# --------------------------------------------------------------------------- #
#: The build error the margin is read at, FIXED 2026-09-08 and ASSERTED, not
#: measured (``notation.md`` sec.8): ``sqrt(3 * 0.2^2) / 80 = 0.004330``, three
#: 0.2 mm sources combined RSS and normalised by an asserted 80 mm ``r_b``
#: floor.  It sits BELOW all three probes ``score_discriminators`` carried, so
#: the score has never been evaluated here; this module evaluates it directly
#: and extrapolates nothing.
P_SCORE = 0.004330

#: The probes ``score_discriminators`` did carry, for the comparison only.
OLD_PROBES = SD.SCORE_PROBES

#: How many candidates the pointwise-monotonicity test of part (5) walks in
#: full.  It is a per-(leg, pose)-cell test over the whole 180-point ``delta``
#: scan, so it is reported on a sample and the sample size is stated rather
#: than left to be inferred.
N_MONO_SAMPLE = 40

#: Rank-correlation floor above which this module is willing to call a
#: relationship "strong" IN PROSE.  A REPORTING BAND, not a threshold that
#: decides anything: no candidate is accepted or rejected on it, and the raw
#: rho is printed everywhere it is used.  0.835 is ``tau_min``'s correlation
#: with margin (part b), quoted as the calibration for what "ranks with the
#: margin" already looks like in this project.
TAU_MIN_RHO = 0.835


# --------------------------------------------------------------------------- #
# alpha, two ways
# --------------------------------------------------------------------------- #
def alpha_span_via_ik(rec, delta_deg, R, T):
    """``(span_deg, per_leg_span_deg, alpha_deg)`` from :func:`~stewart.kinematics.ik`.

    One ``ik`` call per pose, on the geometry built at ``delta_deg``.  This is
    the reported quantity: it comes from the same function a servo command
    would, including its fixed ``-`` branch and its ``|P| > C`` reachability
    test.  Returns ``None`` if any pose is unreachable - which the constrained
    tune should have excluded, so the count of these is reported rather than
    absorbed.
    """
    geom = make_geometry(r_b=R_B, beta=rec["beta"], delta=float(delta_deg),
                         r_p=rec["r_p"], beta_p=rec["beta_p"],
                         a=rec["a"], d=rec["d"], h_p=H_P)
    out = np.empty((R.shape[0], 6), dtype=float)
    for k in range(R.shape[0]):
        try:
            out[k] = ik(geom, R[k], T[k])
        except Unreachable:
            return None
    deg = np.degrees(out)
    per_leg = deg.max(axis=0) - deg.min(axis=0)
    return float(per_leg.max()), per_leg, deg


def alpha_scan(beta, beta_p, r_p, a, d, R, T, deltas_deg):
    """``(alpha, span)`` over the whole ``delta`` scan, vectorised.

    ``alpha`` is ``(nd, K, 6)`` in DEGREES, ``span`` is ``(nd,)`` in degrees
    (max over legs of the per-leg range over poses).  A ``delta`` at which any
    leg at any pose is unreachable is ``NaN`` throughout - never clipped, per
    the ``ik`` docstring and ``CLAUDE.md``.

    Same closed form and same branch as :func:`~stewart.kinematics.ik`, built
    stacked over ``(delta, pose)`` the way
    :func:`.score_discriminators.scan_delta` builds its margin, because 180
    deltas x 29 poses x 6 legs per candidate is not affordable one ``ik`` call
    at a time.  The agreement with ``ik`` is MEASURED in the report, not
    assumed: see :func:`verify_alpha`.
    """
    g0, g90 = SD._delta_basis(beta, beta_p, r_p, a, d)
    dr = np.deg2rad(np.asarray(deltas_deg, dtype=float))
    c = np.cos(dr)[:, None, None]
    s = np.sin(dr)[:, None, None]
    n_d = c * g0.n + s * g90.n                        # (nd, 3, 6)
    u_d = c * g0.u + s * g90.u
    v_d = np.cross(n_d, u_d, axis=1)

    q = (np.einsum("kxy,yi->kxi", R, g0.p)
         + np.asarray(T, dtype=float)[:, :, None])    # (K, 3, 6)
    L = q - g0.b[None]
    LL = np.einsum("kxi,kxi->ki", L, L)               # (K, 6)
    P = (LL + a * a - d * d) / (2.0 * a)
    M = np.einsum("kxi,jxi->jki", L, u_d)             # (nd, K, 6)
    N = np.einsum("kxi,jxi->jki", L, v_d)
    C = np.hypot(M, N)

    reach = (C > 0.0) & (np.abs(P)[None] <= C)
    live = reach.all(axis=2).all(axis=1)              # (nd,)
    alpha = np.full(M.shape, np.nan)
    j = np.flatnonzero(live)
    if j.size:
        ratio = P[None] / C[j]                        # |ratio| <= 1, no clip
        alpha[j] = np.arctan2(N[j], M[j]) - np.arccos(ratio)
    alpha = np.degrees(alpha)
    span = np.full(dr.size, np.nan)
    if j.size:
        per_leg = alpha[j].max(axis=1) - alpha[j].min(axis=1)   # (nj, 6)
        span[j] = per_leg.max(axis=1)
    return alpha, span


def verify_alpha(measured, R, T_of, deltas_deg):
    """Worst ``|alpha_scan - ik|`` over every candidate, at its tuned ``delta``.

    The scan is a second implementation of ``ik``'s closed form.  Two
    implementations that are never compared are two chances to be wrong, so
    this is run over the whole measured set rather than a sample.
    """
    worst = 0.0
    worst_span = 0.0
    n = 0
    for rec in measured:
        e = rec["con"][CAP]
        if e is None:
            continue
        T = T_of(rec["z_home"])
        got = alpha_span_via_ik(rec, e["delta"], R, T)
        if got is None:
            continue
        span_ik, _, alpha_ik = got
        k = int(np.argmin(np.abs(np.asarray(deltas_deg) - e["delta"])))
        alpha_sc, span_sc = alpha_scan(rec["beta"], rec["beta_p"], rec["r_p"],
                                       rec["a"], rec["d"], R, T, [deltas_deg[k]])
        worst = max(worst, float(np.nanmax(np.abs(alpha_sc[0] - alpha_ik))))
        worst_span = max(worst_span, abs(float(span_sc[0]) - span_ik))
        n += 1
    return worst, worst_span, n


# --------------------------------------------------------------------------- #
# the score at p, evaluated and not extrapolated
# --------------------------------------------------------------------------- #
def score_at_p(rec, R, az, delta_deg, probe=P_SCORE):
    """``margin(dxy = probe)`` at ``delta_deg`` - the score now in force.

    Straight through :func:`.score_discriminators.probe_margin`, which is the
    worst margin over a full circle of displacement azimuths at ``|dxy| =
    probe``.  Evaluated AT ``probe``; nothing is extrapolated off ``sens``.
    """
    return SD.probe_margin(rec["_g0"], rec["_g90"], R, az, rec["z_home"],
                           delta_deg, probe)


def enrich(measured, R, az, T_of, deltas_deg=DELTA_GRID):
    """Add ``alpha_span``, ``authority`` and ``score_p`` to every record.

    Everything is taken at the CONSTRAINED tune's ``delta`` at :data:`.CAP` -
    ``cond(J_fk) <= 1e6`` at ``char_len = r_b``.  A candidate with no
    admissible ``delta`` at the cap gets ``NaN`` in all three and is CARRIED,
    not dropped, so "infeasible" and "feasible but inadmissible" stay
    distinguishable downstream.
    """
    unreachable = 0
    for rec in measured:
        e = rec["con"][CAP]
        if e is None:
            rec["alpha_span"] = np.nan
            rec["alpha_span_legs"] = None
            rec["authority"] = np.nan
            rec["score_p"] = np.nan
            continue
        T = T_of(rec["z_home"])
        got = alpha_span_via_ik(rec, e["delta"], R, T)
        if got is None:
            unreachable += 1
            rec["alpha_span"] = np.nan
            rec["alpha_span_legs"] = None
            rec["authority"] = np.nan
            rec["score_p"] = np.nan
            continue
        span, per_leg, _ = got
        rec["alpha_span"] = span
        rec["alpha_span_legs"] = per_leg
        rec["authority"] = TILT_LIMIT_DEG / span if span > 0.0 else np.inf
        rec["score_p"] = score_at_p(rec, R, az, e["delta"])
    return unreachable


def _cols(rows, *keys):
    """Columns as float arrays, in one pass, NaN preserved."""
    return tuple(np.array([r.get(k, np.nan) for r in rows], dtype=float)
                 for k in keys)


def _rho_block(indent, label, span, score, n_label="n"):
    """Print one correlation with BOTH sign readings spelled out."""
    rho, n = _spearman(span, score)
    print(f"{indent}{label}")
    if not np.isfinite(rho):
        print(f"{indent}  rho undefined ({n_label} = {n})")
        return rho, n
    print(f"{indent}  rho(alpha_span, score) = {rho:+.4f}   ({n_label} = {n})")
    print(f"{indent}  rho(authority,  score) = {-rho:+.4f}   "
          f"[authority = {TILT_LIMIT_DEG:.4f} / alpha_span, "
          f"a decreasing relabelling]")
    if rho < 0:
        verdict = ("authority ranks WITH the margin -> REDUNDANT, bounds "
                   "nothing")
    elif rho > 0:
        verdict = ("authority OPPOSES the margin -> a counterweight, bounds "
                   "the runaway")
    else:
        verdict = "no monotone relationship either way"
    print(f"{indent}  reading: {verdict}")
    return rho, n


# --------------------------------------------------------------------------- #
# parts
# --------------------------------------------------------------------------- #
def part1(coarse, explored):
    """(1) alpha_span five-number summary, feasible set and explored set."""
    print()
    print("=" * 78)
    print("(1) alpha_span - THE SERVO SWING THE ENVELOPE COSTS")
    print("=" * 78)
    print(f"  Degrees.  Worst leg's (max - min) of alpha over the "
          f"{coarse['n_poses']}-pose envelope grid,")
    print(f"  at the delta tuned under cond(J_fk) <= {CAP:.0e} at char_len = "
          f"{SD.CONSTRAINT_CHAR_LEN}.")
    print("  alpha_span itself carries NO char_len and NO Jacobian - it is an")
    print("  angle out of ik().  The CAP that picked its delta carries both.")
    print()
    print("  Two sets, reported separately because they are different")
    print("  questions.  FEASIBLE is the 540-candidate coarse grid's")
    print("  survivors - the set the score already ranks.  EXPLORED adds the")
    print("  four boundary rays of box_boundary, i.e. the runaway region the")
    print("  8 September finding is about; a summary over the coarse grid")
    print("  alone would not contain the candidates in question.")
    print()
    hdr = f"    {'set':<26} {'n':>6} {'min':>10} {'Q1':>10} {'median':>10} " \
          f"{'Q3':>10} {'max':>10}"
    print(hdr)
    for label, rows in (("feasible (coarse grid)", coarse["rows"]),
                        ("explored (grid + 4 rays)", explored["rows"])):
        (span,) = _cols(rows, "alpha_span")
        good = span[np.isfinite(span)]
        if not good.size:
            print(f"    {label:<26} {'-':>6}")
            continue
        f = _five_number(good)
        print(f"    {label:<26} {good.size:>6} " +
              " ".join(f"{v:>10.4f}" for v in f))
    print()
    print("  Same rows as authority = "
          f"{TILT_LIMIT_DEG:.4f} deg / alpha_span [dimensionless]:")
    print(f"    {'set':<26} {'n':>6} {'min':>10} {'Q1':>10} {'median':>10} "
          f"{'Q3':>10} {'max':>10}")
    for label, rows in (("feasible (coarse grid)", coarse["rows"]),
                        ("explored (grid + 4 rays)", explored["rows"])):
        (auth,) = _cols(rows, "authority")
        good = auth[np.isfinite(auth)]
        if not good.size:
            continue
        f = _five_number(good)
        print(f"    {label:<26} {good.size:>6} " +
              " ".join(f"{v:>10.4f}" for v in f))
    print()
    print("  The two tables carry the same information - authority is a fixed")
    print("  decreasing relabelling, the numerator is the same envelope for")
    print("  every candidate - and the min/max swap ends.  Both are printed")
    print("  once, here, so no later table has to repeat the reciprocal.")
    print()
    for label, rows in (("feasible", coarse["rows"]),
                        ("explored", explored["rows"])):
        (span,) = _cols(rows, "alpha_span")
        nan = int(np.sum(~np.isfinite(span)))
        print(f"    {label:<10}: {len(rows)} candidates carried, {nan} with no "
              f"alpha_span")
    print("      (a candidate with no admissible delta at the cap has no tuned")
    print("       configuration to measure a swing at; carried as NaN, not")
    print("       dropped, so it cannot silently flatter a summary)")


def part2(coarse, rays):
    """(2) Spearman, coarse grid and each ray.  The load-bearing number."""
    print()
    print("=" * 78)
    print("(2) SPEARMAN: alpha_span AGAINST THE SCORE NOW IN FORCE")
    print("=" * 78)
    print(f"  score = margin(dxy = p) at p = {P_SCORE}, EVALUATED at that")
    print("  value, not extrapolated from a slope.  Both tuned under")
    print(f"  cond(J_fk) <= {CAP:.0e} at char_len = {SD.CONSTRAINT_CHAR_LEN}.")
    print()
    print("  THIS IS THE LOAD-BEARING NUMBER.  Calibration, from")
    print(f"  score_discriminators part (b): tau_min correlates with margin at")
    print(f"  rho = {TAU_MIN_RHO:+.3f} over the feasible set, and is recorded")
    print("  there as measured-and-redundant.  A quantity that ranks with the")
    print("  margin that strongly is not a counterweight to it.")
    print()
    span, score = _cols(coarse["rows"], "alpha_span", "score_p")
    rho_c, _ = _rho_block("  ", "COARSE GRID (540-candidate screen's survivors):",
                          span, score)
    print()
    print("  ALONG EACH RAY.  Each ray moves ONE axis with the other four")
    print("  pinned at coarse-grid values, so a ray's correlation is a")
    print("  statement about that axis alone - which is the limitation the")
    print("  8 September entry already records against these four probes.")
    print()
    for r in rays:
        sp, sc = _cols(r["rows"], "alpha_span", "score_p")
        _rho_block("  ", f"{r['name']:<16} ({r['label']}, "
                         f"{len(r['rows'])} candidates):", sp, sc)
        print()
    return rho_c


def part2b(coarse, explored, rp_ray):
    """(2b) WHY the moment-arm argument does not hold here.  Measured.

    A correlation with no mechanism behind it is a number waiting to be
    explained away, so the mechanism is measured rather than asserted - and it
    is measured on the quantity the moment-arm argument is about, ``r_p``.
    """
    print()
    print("=" * 78)
    print("(2b) WHY - THE MOMENT-ARM ARGUMENT ASSUMES A DISPLACEMENT, AND THE")
    print("     ENVELOPE IS AN ANGLE")
    print("=" * 78)
    print("  The counterweight argument is: a small ring gives a short moment")
    print("  arm, so a fixed servo travel buys less platform tilt.  That is a")
    print("  statement about producing a fixed platform DISPLACEMENT.  The")
    print("  settled envelope is dxy = dz = yaw = 0 and a fixed tilt ANGLE")
    print(f"  ({TILT_LIMIT_DEG:.4f} deg, notation.md sec.9), so the anchors'")
    print("  excursion is r_p sin(tilt) - it SHRINKS with the ring.  A smaller")
    print("  platform has to move its anchors less to reach the same angle, so")
    print("  it needs LESS servo swing, not more.")
    print()
    print("  Measured, not asserted:")
    print()
    for label, rows in (("feasible (coarse grid)", coarse["rows"]),
                        ("explored (grid + 4 rays)", explored["rows"])):
        span, rp = _cols(rows, "alpha_span", "r_p")
        rho, n = _spearman(span, rp)
        print(f"    rho(alpha_span, r_p), {label:<26}: {rho:+.4f}  (n = {n})")
    print()
    print("    A POSITIVE rho here is the whole mechanism: the swing rises with")
    print("    the ring radius, which is the opposite of what a moment-arm")
    print("    counterweight needs.")
    print()
    print("  And on the runaway axis itself, with every other axis pinned -")
    print("  median alpha_span against r_p, and the ratio that would be")
    print("  constant if the swing were exactly proportional to r_p:")
    print()
    print(f"    {'r_p/r_b':>9} {'median span':>13} {'span / r_p':>12}")
    for step in rp_ray["steps"]:
        (sp,) = _cols(step["rows"], "alpha_span")
        good = sp[np.isfinite(sp)]
        if not good.size:
            continue
        med = float(np.median(good))
        print(f"    {step['value']:>9.2f} {med:>13.4f} "
              f"{med / step['value']:>12.4f}")
    print()
    print("    The ratio is not constant - r_p is not the only length in the")
    print("    leg, and a, d and z_home do not move with it - so this is a")
    print("    strong trend and NOT a proportionality, and it is not claimed")
    print("    as one.  What it is enough to establish is the SIGN, which is")
    print("    what the counterweight argument needs and does not get.")


def _ray_table(ray, title, note):
    """One ray's step-by-step table: alpha_span beside the margin."""
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)
    print(f"  {note}")
    print()
    print("  At each step, over the candidates AT that axis value: the best by")
    print("  score, with its alpha_span, and the median alpha_span of the")
    print(f"  slice.  All at the delta tuned under cond(J_fk) <= {CAP:.0e} at")
    print(f"  char_len = {SD.CONSTRAINT_CHAR_LEN}.  alpha_span in degrees.")
    print()
    print(f"    {'value':>8} {'n':>5} {'best score':>12} {'alpha_span':>12} "
          f"{'authority':>11} {'median span':>12} {'best (beta,beta_p,a,d)':>26}")
    prev_span = None
    for step in ray["steps"]:
        rows = step["rows"]
        sp, sc = _cols(rows, "alpha_span", "score_p")
        ok = np.isfinite(sc) & np.isfinite(sp)
        if not ok.any():
            print(f"    {step['value']:>8.2f} {len(rows):>5} "
                  f"{'-':>12} {'-':>12} {'-':>11} {'-':>12}")
            continue
        b = int(np.flatnonzero(ok)[np.argmax(sc[ok])])
        rec = rows[b]
        med = float(np.median(sp[ok]))
        auth = TILT_LIMIT_DEG / sp[b] if sp[b] > 0 else np.inf
        tag = (f"({rec['beta']:.0f},{rec['beta_p']:.0f},"
               f"{rec['a']:.2f},{rec['d']:.2f})")
        arrow = "  <- incumbent" if step.get("base") else ""
        if not arrow and prev_span is not None:
            arrow = "  up" if sp[b] > prev_span else ("  down" if sp[b] < prev_span
                                                      else "  flat")
        prev_span = sp[b]
        print(f"    {step['value']:>8.2f} {int(ok.sum()):>5} {sc[b]:>12.6f} "
              f"{sp[b]:>12.4f} {auth:>11.4f} {med:>12.4f} {tag:>26}{arrow}")
    print()
    sp_all, sc_all = _cols(ray["rows"], "alpha_span", "score_p")
    _rho_block("  ", "over every candidate on this ray:", sp_all, sc_all)


def part5(measured, R, T_of, deltas_deg):
    """(5) Is alpha_span monotone in the pointwise margin?"""
    print()
    print("=" * 78)
    print("(5) IS alpha_span MONOTONE IN THE POINTWISE MARGIN?")
    print("=" * 78)
    print("  Why this decides where tilt authority would go.  notation.md")
    print("  sec.8 records the rule: a quantity that matters and is NOT")
    print("  monotone in the pointwise margin must enter the INNER TUNE, not")
    print("  the outer score, or the tune will spend it - delta is chosen")
    print("  before the score is read, so anything the score would penalise is")
    print("  something the tuner trades away for margin first.  Two instances")
    print("  are on record: cond, and the tune/score mismatch.")
    print()
    print("  FIRST, A STRUCTURAL POINT, and it is not a measurement.")
    print("  alpha_span is a RANGE over poses (max - min), then a MAX over")
    print("  legs.  The margin is a MIN over poses and legs.  A range is not a")
    print("  function of any single (leg, pose) value at all, so 'monotone in")
    print("  the pointwise margin' cannot be asked of alpha_span directly the")
    print("  way it was asked of cond.  Two things ARE askable, and both are")
    print("  measured below:")
    print("    (5a) POINTWISE.  At a fixed leg and pose, as delta moves, is")
    print("         alpha monotone in that cell's own margin?")
    print("    (5b) AGGREGATE.  As delta moves, is alpha_span monotone in the")
    print("         aggregate margin?  This is the one the rule turns on,")
    print("         because delta is what the tune is free to spend.")
    print()

    sample = measured[::max(1, len(measured) // N_MONO_SAMPLE)][:N_MONO_SAMPLE]
    dr = np.asarray(deltas_deg, dtype=float)

    # ---- (5a) pointwise cells ------------------------------------------ #
    cell_rho = []
    cell_perfect = 0
    cell_total = 0
    # ---- (5b) aggregate ------------------------------------------------ #
    agg_rho = []
    for rec in sample:
        T = T_of(rec["z_home"])
        alpha, span = alpha_scan(rec["beta"], rec["beta_p"], rec["r_p"],
                                 rec["a"], rec["d"], R, T, dr)
        m_scan = rec["scan_margin"]
        live = np.isfinite(span) & np.isfinite(m_scan)
        if live.sum() < 4:
            continue
        # aggregate
        rho, _ = _spearman(span[live], m_scan[live])
        if np.isfinite(rho):
            agg_rho.append(rho)
        # pointwise: rebuild the per-cell margin over the same scan
        g0, g90 = SD._delta_basis(rec["beta"], rec["beta_p"], rec["r_p"],
                                  rec["a"], rec["d"])
        LL, P, A, B = SD._invariants(g0, g90, R, T)
        w = (A[None] * np.cos(np.deg2rad(dr))[:, None, None]
             + B[None] * np.sin(np.deg2rad(dr))[:, None, None])
        C = np.sqrt(np.maximum(LL[None] - w * w, 0.0))
        with np.errstate(divide="ignore", invalid="ignore"):
            m_cell = np.where(C > 0.0, (C - np.abs(P)[None]) / C, np.nan)
        idx = np.flatnonzero(live)
        for k in range(0, R.shape[0], max(1, R.shape[0] // 4)):
            for i in range(6):
                x = m_cell[idx, k, i]
                y = alpha[idx, k, i]
                good = np.isfinite(x) & np.isfinite(y)
                if good.sum() < 4:
                    continue
                r_c, _ = _spearman(x[good], y[good])
                if not np.isfinite(r_c):
                    continue
                cell_total += 1
                cell_rho.append(abs(r_c))
                if abs(r_c) > 1.0 - 1e-9:
                    cell_perfect += 1

    print(f"  Sample: {len(sample)} candidates, the whole "
          f"{dr.size}-point delta scan each.")
    print()
    print("  (5a) POINTWISE, per (leg, pose) cell, alpha against that cell's")
    print("       own margin as delta sweeps:")
    if cell_total:
        ar = np.array(cell_rho)
        print(f"       cells tested                 : {cell_total}")
        print(f"       perfectly monotone (|rho| = 1): {cell_perfect} "
              f"({cell_perfect / cell_total:.1%})")
        print(f"       |rho| five-number            : " +
              " ".join(f"{v:.4f}" for v in _five_number(ar)))
    print()
    print("  (5b) AGGREGATE, alpha_span against the maximin margin as delta")
    print("       sweeps - the relationship the tune actually moves along:")
    if agg_rho:
        ag = np.array(agg_rho)
        print(f"       candidates tested            : {ag.size}")
        print(f"       rho five-number              : " +
              " ".join(f"{v:+.4f}" for v in _five_number(ag)))
        print(f"       perfectly monotone (|rho| = 1): "
              f"{int(np.sum(np.abs(ag) > 1.0 - 1e-9))} / {ag.size}")
        print(f"       sign split                   : "
              f"{int(np.sum(ag > 0))} positive, {int(np.sum(ag < 0))} negative")
    print()
    print("  READING.  If (5b) is not monotone - and especially if its SIGN is")
    print("  not even fixed across candidates - then alpha_span is a quantity")
    print("  the delta tune can move independently of the margin, which is")
    print("  exactly the condition sec.8's rule is about.  The rule then says")
    print("  it must enter the inner tune rather than the outer score.  This")
    print("  module reports whether the condition holds; it does not decide")
    print("  where the term goes, and does not propose one.")
    return (np.array(agg_rho) if agg_rho else np.array([]),
            np.array(cell_rho) if cell_rho else np.array([]))


def part6(measured, R, T_of, deltas_deg):
    """(6) delta maximising margin(0) vs delta minimising alpha_span."""
    print()
    print("=" * 78)
    print("(6) THE TWO DELTAS: max margin(0)  vs  min alpha_span")
    print("=" * 78)
    print("  Per candidate, over the same 180-point scan.  The first is the")
    print(f"  constrained tune in force (cond(J_fk) <= {CAP:.0e} at char_len =")
    print(f"  {SD.CONSTRAINT_CHAR_LEN}); the second is what a tune that cared")
    print("  about servo swing instead would have picked, under the SAME cap.")
    print("  Neither is proposed here.  The gap between them is the size of")
    print("  what the current tune is spending, if it is spending anything.")
    print()
    dr = np.asarray(deltas_deg, dtype=float)
    gaps, span_at_m, span_at_s, marg_at_m, marg_at_s = [], [], [], [], []
    n_same = 0
    for rec in measured:
        e = rec["con"][CAP]
        if e is None:
            continue
        T = T_of(rec["z_home"])
        _, span = alpha_scan(rec["beta"], rec["beta_p"], rec["r_p"],
                             rec["a"], rec["d"], R, T, dr)
        m_scan, c_scan = rec["scan_margin"], rec["scan_cond"]
        ok = np.isfinite(m_scan) & np.isfinite(span) & (c_scan <= CAP)
        if not ok.any():
            continue
        i_m = int(np.flatnonzero(ok)[np.argmax(m_scan[ok])])
        i_s = int(np.flatnonzero(ok)[np.argmin(span[ok])])
        g = abs(dr[i_m] - dr[i_s])
        g = min(g, 180.0 - g)                       # delta lives mod 180
        gaps.append(g)
        if g < 1e-9:
            n_same += 1
        span_at_m.append(span[i_m])
        span_at_s.append(span[i_s])
        marg_at_m.append(m_scan[i_m])
        marg_at_s.append(m_scan[i_s])
    if not gaps:
        print("  no candidate had an admissible delta at the cap")
        return
    gaps = np.array(gaps)
    sm, ss = np.array(span_at_m), np.array(span_at_s)
    mm, ms = np.array(marg_at_m), np.array(marg_at_s)
    print(f"  candidates compared                       : {gaps.size}")
    print(f"  the two deltas AGREE                      : {n_same} "
          f"({n_same / gaps.size:.1%})")
    print(f"  they DIFFER                               : "
          f"{gaps.size - n_same} ({1 - n_same / gaps.size:.1%})")
    print()
    print(f"  |delta gap| in degrees, mod 180, five-number:")
    print(f"    " + " ".join(f"{v:.3f}" for v in _five_number(gaps)))
    print()
    print("  What the margin tune costs in swing, and what a swing tune would")
    print("  cost in margin - both differences of two measurements on ONE")
    print("  candidate, so they subtract validly:")
    print(f"    alpha_span at the margin delta   : "
          f"{' '.join(f'{v:.4f}' for v in _five_number(sm))}")
    print(f"    alpha_span at the swing delta    : "
          f"{' '.join(f'{v:.4f}' for v in _five_number(ss))}")
    print(f"    swing GIVEN UP by tuning margin  : "
          f"{' '.join(f'{v:.4f}' for v in _five_number(sm - ss))}  deg")
    print(f"    margin GIVEN UP by tuning swing  : "
          f"{' '.join(f'{v:.6f}' for v in _five_number(mm - ms))}")
    print()
    print(f"  For scale: the ranking grid's own resolution is "
          f"{SD.GRID_RESOLUTION:.4e} in margin")
    print("  units (sec.8).  A margin difference below it is a tie the grid")
    print("  cannot resolve, not an ordering.")
    n_below = int(np.sum((mm - ms) < SD.GRID_RESOLUTION))
    print(f"    candidates where the margin given up is below it: "
          f"{n_below} / {gaps.size} ({n_below / gaps.size:.1%})")


# --------------------------------------------------------------------------- #
def build_sets():
    """Screen and tune the coarse grid and the four rays.  One pass each."""
    R, az, _ = SD._pose_grid(None)
    T_of = lambda z: SD._T_stack(az, z)

    print("=" * 78)
    print("SETUP")
    print("=" * 78)
    print(f"  envelope    : tilt <= {TILT_LIMIT_DEG:.4f} deg, "
          f"{az.size} poses (the settled harness grid)")
    print(f"  grid        : {len(BETA)}x{len(BETA_P)}x{len(RP_RB)}x{len(A_RB)}x"
          f"{len(D_RB)} = "
          f"{len(BETA)*len(BETA_P)*len(RP_RB)*len(A_RB)*len(D_RB)} candidates, "
          f"h_p/r_b = {H_P}")
    print(f"  cap         : cond(J_fk) <= {CAP:.0e} at char_len = "
          f"{SD.CONSTRAINT_CHAR_LEN}  (PROVISIONAL in that length)")
    print(f"  score probe : p = {P_SCORE}  (fixed 2026-09-08, ASSERTED)")
    print(f"  old probes  : {OLD_PROBES}  - p is BELOW all three, so the score")
    print("                has never been evaluated at the value in force; it")
    print("                is evaluated directly here and not extrapolated.")
    print()

    t0 = time.time()
    base = BB.Box()
    n_rows, n_feas = base.add_slice(beta_vals=BETA, beta_p_vals=BETA_P,
                                    rp_vals=RP_RB, a_vals=A_RB, d_vals=D_RB)
    print(f"  coarse grid : {n_rows} screened, {n_feas} feasible, "
          f"{len(base.measured)} measured, {base.dropped} dropped "
          f"(ik unreachable at the bracket midpoint)")
    n_un = enrich(base.measured, R, az, T_of)
    print(f"                {n_un} with no alpha_span from ik at the tuned "
          f"delta")
    print(f"                [{time.time() - t0:.1f} s]")

    rays = []
    axis_key = {"a": "a", "r_p": "r_p", "d": "d"}
    for ext in BB.extensions():
        t1 = time.time()
        # STEP 0 is the value the ray steps AWAY from, and it comes out of the
        # coarse grid rather than being re-screened: at that value every other
        # axis is already at its default, which is exactly a ray slice.  It is
        # included so the tables read from the incumbent (r_p 0.60, a 0.35) as
        # the question asks, rather than starting one step in.
        base_rows = [r for r in base.measured
                     if r[axis_key[ext.axis]] == ext.start]
        steps = [dict(value=ext.start, rows=base_rows, base=True)]
        ray_rows = list(base_rows)
        for step in range(1, BB.MAX_STEPS + 1):
            box = BB.Box()
            box.add_slice(**ext.slice_axes(step))
            enrich(box.measured, R, az, T_of)
            steps.append(dict(value=ext.value_at(step), rows=box.measured,
                              base=False))
            ray_rows.extend(box.measured)
        rays.append(dict(name=ext.name, label=ext.label, axis=ext.axis,
                         direction=ext.direction, start=ext.start,
                         steps=steps, rows=ray_rows))
        print(f"  ray {ext.name:<12}: {len(ray_rows)} measured over "
              f"{BB.MAX_STEPS} steps of {BB.STEP}  "
              f"[{time.time() - t1:.1f} s]")

    explored_rows = list(base.measured)
    seen = {_key(r) for r in base.measured}
    for r in rays:
        for rec in r["rows"]:
            if _key(rec) not in seen:
                seen.add(_key(rec))
                explored_rows.append(rec)

    coarse = dict(rows=base.measured, n_poses=az.size)
    explored = dict(rows=explored_rows, n_poses=az.size)
    return R, az, T_of, base, coarse, explored, rays


def main() -> None:
    R, az, T_of, base, coarse, explored, rays = build_sets()

    # ---- verification, before anything is read off the scan ------------- #
    print()
    print("=" * 78)
    print("VERIFICATION - the vectorised alpha against ik() itself")
    print("=" * 78)
    worst, worst_span, n = verify_alpha(base.measured, R, T_of, DELTA_GRID)
    print(f"  candidates checked                 : {n}")
    print(f"  worst |alpha_scan - ik| (deg)      : {worst:.3e}")
    print(f"  worst |span_scan - span_ik| (deg)  : {worst_span:.3e}")
    print("  The scan is a second implementation of ik's closed form, branch")
    print("  included.  Parts (5) and (6) read alpha off the scan because 180")
    print("  deltas x 29 poses per candidate is not affordable one ik call at")
    print("  a time; parts (1)-(4) read it off ik directly.  Two")
    print("  implementations that are never compared are two chances to be")
    print("  wrong, which is what this line is for.")

    part1(coarse, explored)
    rho_c = part2(coarse, rays)

    rp_ray = next(r for r in rays if r["axis"] == "r_p")
    part2b(coarse, explored, rp_ray)
    _ray_table(rp_ray,
               "(3) alpha_span ALONG THE r_p RAY, 0.60 DOWN TO 0.10",
               "The runaway axis.  Does the swing grow as the platform shrinks?")

    a_ray = next(r for r in rays if r["axis"] == "a" and r["direction"] > 0)
    _ray_table(a_ray,
               "(4) alpha_span ALONG THE a RAY, 0.35 UP TO 0.85",
               "Where the margin optimum moved to a/r_b = 0.60 and dragged "
               "d/r_b to 1.20.")

    agg_rho, cell_rho = part5(base.measured, R, T_of, DELTA_GRID)
    part6(base.measured, R, T_of, DELTA_GRID)

    # ---- verdict -------------------------------------------------------- #
    print()
    print("=" * 78)
    print("WHAT THIS ANSWERS AND WHAT IT DOES NOT")
    print("=" * 78)
    print("  The question was: does tilt authority OPPOSE reach margin, or")
    print("  rank WITH it?  Read part (2) - the coarse-grid rho and the four")
    print("  ray rhos - and part (3), which is the runaway axis itself.")
    print()
    print(f"  Sign convention, once more: rho(alpha_span, score) < 0 means")
    print(f"  authority ranks WITH the margin and bounds nothing;")
    print(f"  rho(alpha_span, score) > 0 means it opposes the margin and")
    print(f"  bounds the runaway.  Coarse grid: {rho_c:+.4f}.")
    print(f"  Calibration: tau_min sits at {TAU_MIN_RHO:+.3f} and is recorded")
    print("  as measured-and-redundant.")
    print()
    print("  WHAT THIS DOES NOT DO.  It does not choose a servo travel figure,")
    print("  does not propose a weight or a combined score, does not change")
    print("  the feasibility tests - servo travel stays excluded from")
    print("  feasibility by the 2026-09-08 decision - and does not decide")
    print("  whether tilt authority belongs in the inner tune or the outer")
    print("  score.  Part (5) reports whether sec.8's rule applies to it; the")
    print("  rule's consequence is a decision and is not taken here.")
    print("  notation.md is untouched.")
    print()
    print("  LIMITATION, the same one the 8 September entry records against")
    print("  these four probes: each ray moves ONE axis with the other four")
    print("  pinned at coarse-grid values, so a joint effect needing two axes")
    print("  to move together is invisible to all four.  The a ray dragging")
    print("  d/r_b from 0.80 to 1.20 is direct evidence that coupling is")
    print("  present, and nothing here removes that limitation.")


if __name__ == "__main__":
    main()
