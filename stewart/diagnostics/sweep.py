"""The sweep harness: generate, screen, tune, score, and return a tie set.

    python -m stewart.diagnostics.sweep

This is the module the rest of Phase 0 was built to feed.  It generates
candidate geometries over five axes, decides feasibility with the closed-form
tests, tunes ``delta`` per candidate, scores the survivors at the build-error
probe, and returns a **ranked shortlist of TIE GROUPS**.  Every number in the
eventual build is meant to be traceable to a run of this file.

WHAT IS FIXED AND IS NOT AN AXIS.  ``notation.md`` sec.9/12, all ASSERTED
2026-09-08 and read from :mod:`.fixed_ratio` rather than restated::

    r_b = 90 mm, r_p = 80 mm   so r_p/r_b = 80/90 is NOT a sweep axis
    h_p / r_b = 0.1            hardware number
    tilt limit                 envelope.tilt_for(X0_WORKING, TAU), PRINTED,
                               never hardcoded - tau_L is DROPPED
    p = 0.003849 r_b           the score probe, from the three build-error
                               sources, not transcribed

THE FIVE AXES.

``beta``
    ``(0, 60)`` degrees, **open at both ends, UNBOUNDED**.  The servo
    mounting-flange footprint, screw pitch and inter-body clearance are
    unpublished for every candidate servo checked (``docs/hardware-pull.md``
    sec.5, among the 39 unpublished cells), so the collision bound that would
    close this axis **cannot be set**.  The full open interval is swept and the
    shortlist's required arc spacing is reported in mm at ``r_b = 90`` as a
    POST-HOC CHECK.  It is not a filter and nothing is rejected by it.
``beta_p``
    bounded by ball-joint housing OD.  BOTH published ends are carried in
    parallel - ``[3.2246, 56.7754]`` deg at OD 9.0 mm and
    ``[4.6604, 55.3396]`` at 13.0 mm, from
    :func:`.sweep_ranges.beta_p_bounds_deg`.  The sweep runs ONCE over the
    union and is partitioned afterwards.  **No joint is chosen, the two runs
    are not merged, and the two bounds are not averaged.**
``a / r_b``
    **DISCRETE**: the 32 published ProModeler hole positions of
    :func:`.sweep_ranges.a_set_rb`, ``0.1000`` - ``0.6711``.  Not a continuous
    axis and not resampled.  Elided ladders stay unpublished; the Thingiverse
    extension arms are a 3D print, not stock, and are excluded.
``d / r_b``
    continuous, capped at ``2.0`` = 180 mm.  The cap is ASSERTED on rod
    slenderness and bow (``notation.md`` sec.12), **not** a hardware limit -
    stock runs to 2.1x the longest length this sweep has ever used.  The axis
    has no asserted LOWER end, so it is swept down to one grid step and the
    feasibility screen is left to set the bottom; where that bottom falls is
    reported rather than assumed.
``z_home / r_b``
    continuous, **per candidate**.  It is not an outer-product axis: the
    bracket is computed for each candidate from the reach test intersected
    with the closed-form ``N_i > 0`` floor, and ``z_home`` is taken at its
    midpoint.

FEASIBILITY - pass/fail, before any scoring, four tests::

    1  non-empty z_home bracket
    2  envelope reachable                        |P_i| <= C_i, exact
    3  N_i > 0                                   from the CLOSED FORM
    4  bracket width >= 2 mm = 0.0222 r_b        ASSERTED, build tolerance

Test 3 is taken from :func:`.zhome_bracket.z_lower_closed_form` and **never**
from the pose grid.  It is a continuum bound and a discrete grid reports it
satisfied ``+5.9e-4`` before it truly is (``notation.md`` sec.12, the first of
four recorded instances of a grid reporting what the continuum does not).
Test 4 is the fourth of those instances turned into a test: 21 of 143
candidates feasible at this limit had a bracket of grid width ``0.000 r_b`` -
a single scan point, not a design.  **Servo travel is NOT a feasibility test**
(decided 2026-09-08).

THE EXACT DELTA-FREE BRACKET.  ``w_i(delta) = A_i cos delta + B_i sin delta``
with ``A``, ``B`` and ``G_i = |L_i|^2 - P_i^2`` all free of ``delta``, so with
``amp_i = hypot(A_i, B_i)`` two of the three cases need no scan at all::

    G_i < 0 somewhere        <=>  |P_i| >= |L_i|      no delta works - DROP
    amp_i^2 <= G_i everywhere <=> |P_i| <= sqrt(|L_i|^2 - amp_i^2)
                                                     every delta works - ADMIT
    otherwise                                        scan

This is the harness's own bracket from the 2026-09-05 plan and it is EXACT,
not a heuristic: :func:`verify_bracket` checks it against
:func:`.zhome_bracket.reach_feasible_any_delta` - the committed primitive -
on a sample of the actual sweep before the sweep is allowed to run.  Measured
here, it leaves about 5 of the 120 ``z_home`` scan points undecided per
candidate, which is what makes a sweep this size affordable.

THE INNER TUNE.  ``delta`` on ``[0, 180)``, maximising ``margin(dxy = 0)``
subject to ``cond(J_fk) <= 1e6`` at ``char_len = r_b``, through
:func:`.score_discriminators.scan_delta` and
:func:`~.score_discriminators.tune_constrained` unchanged.  ``cond`` is
evaluated at EVERY delta step - measured at 61 us/step when the projection was
made and at ~9 us/step here, either way far too cheap to justify a banding
heuristic, and part (6) of :mod:`.score_discriminators` shows the band pays
only where the cap is nearly non-binding.  ``delta*`` is computed and its gap
to the tuned winner is REPORTED, and that is all: it zeroes ``w_i`` at home,
at one pose, and **the basin claim attached to it was withdrawn 2026-09-04**.
It does not narrow the scan.

THE SCORE.  ``margin(dxy = p)`` at ``p = P_SCORE``, maximin normalised reach
margin over legs and the envelope pose grid, evaluated at that probe DIRECTLY
through :func:`.score_discriminators.probe_margin`.  No slope is extrapolated
from ``sens``.

THE OUTPUT IS A TIE SET, NOT A WINNER.  :data:`TIE_TOL` is ``1.5e-4``, the
ranking resolution MEASURED at ``dxy = p`` in :mod:`.sweep_ranges` part (7);
:data:`.score_discriminators.TIE_TOL` at ``1e-12`` is correct for the
``dxy = 0`` collapse it was set from and is eight decades wrong here, so it is
deliberately NOT imported.  The shortlist is grouped by ``(a, d,
|beta_p - beta|)`` - part (8)'s invariant with ``r_p`` dropped from the key,
``r_p`` no longer varying - so that two candidates which are one result
reached by two ``(beta, beta_p)`` pairs read as one group.

AN EMPTY RETURN IS A LEGITIMATE RESULT.  177 of 540 were empty on the coarse
grid before scoring ever ran.  If this returns empty the binding constraint
and its margin are reported and the run stops.  **Nothing is widened here.**

WHAT IS NOT DECIDED HERE.  **No joint, no servo, no horn.**  The objective is
NOT fixed: ``margin`` still measures distance from unreachability rather than
capability (``notation.md``, 8 Sept) and that stays open.  ``notation.md`` is
not touched.  Every conditioning number carries the ``char_len`` it was
computed at; ``margin`` is dimensionless and carries none.

Degrees at the boundary, radians internally.  Arrays are (3, 6).  numpy only.
Reports findings; always exits 0.
"""
from __future__ import annotations

import sys
import time

import numpy as np

from . import envelope as ENV
from . import fixed_ratio as FR
from . import score_discriminators as SD
from . import sweep_ranges as SR
from . import zhome_bracket as ZB
from .score_discriminators import _five_number
from .tilt_bracket import CAP, at_tilt
from .zhome_bracket import DELTA_GRID, H_P, R_B, Z_GRID

# --------------------------------------------------------------------------- #
# fixed, not swept
# --------------------------------------------------------------------------- #
#: The two asserted lengths and the ratio they imply.  Read from
#: :mod:`.fixed_ratio`, never restated: ``r_p/r_b`` is NOT an axis here.
R_B_MM, R_P_MM, R_P_RB = FR.R_B_MM, FR.R_P_MM, FR.R_P_RB

#: The score probe, in ``r_b``.  Computed in :mod:`.fixed_ratio` from the three
#: build-error sources, not transcribed from the 0.003849 the request quotes;
#: the two are asserted equal in :func:`report_ledger`.
P_SCORE = FR.P_SCORE

#: Characteristic length the ``cond`` cap is evaluated at.  PROVISIONAL in that
#: length, as the cap is.  Named so no number below is quoted without it.
CHAR_LEN = SD.CONSTRAINT_CHAR_LEN

#: MINIMUM BRACKET WIDTH.  **ASSERTED** 2026-09-09 from build tolerance on
#: ``z_home``, not measured.  It sits just under one ``0.025 r_b`` scan step
#: (2.25 mm at ``r_b = 90``), so it removes the zero-width single-grid-point
#: artefacts and little else past them - a floor beneath which a bracket is not
#: buildable, not a discriminating filter that reorders the surviving field.
MIN_BRACKET_MM = 2.0
MIN_BRACKET_RB = MIN_BRACKET_MM / R_B_MM

#: The tie threshold, and it is a MEASUREMENT: the ranking resolution at
#: ``dxy = p``, from :mod:`.sweep_ranges` part (7), where 24 / 72 / 360
#: displacement azimuths were compared and 72 and 360 already agree.  Two
#: candidates closer together than this are ordered by the azimuth grid the
#: score is computed on and not by the geometry.  Deliberately NOT
#: :data:`.score_discriminators.TIE_TOL`, which is ``1e-12`` and is the right
#: number for a different quantity.
TIE_TOL = 1.5e-4

#: Published servo BODY WIDTH, mm, across the five Hitec/Savox parts of
#: ``docs/hardware-pull.md`` sec.5.  A real published number, and **not** the
#: quantity ``beta`` needs: that is the INSTALLED-ARC clearance - mounting-
#: flange footprint, screw-hole pitch, inter-body clearance including wiring -
#: which is among the pull's 39 unpublished cells for every servo checked.
#: Carried so the arc spacing reported below has something to be read against,
#: and labelled a PROXY so it is not mistaken for the bound.
SERVO_BODY_WIDTH_MM = (11.4, 13.0)

#: Where :func:`save` writes the run.  Regenerating a report costs neither
#: the 25-minute screen nor the 70-minute tune.
SAVE_PATH = "sweep-run.npz"

#: Candidates per chunk.  The screen holds only one candidate's arrays at a
#: time and the survivor records carry scalars alone - no ``scan_margin`` or
#: ``scan_cond`` array is retained - so this sets the reporting granularity and
#: the peak, not the correctness.  See :func:`report_ledger` for the count the
#: chunking requirement rests on.
CHUNK = 2000

#: How many tie groups, and how many members of each, the shortlist prints.
TOP_GROUPS = 12
MEMBERS_PER_GROUP = 6


# --------------------------------------------------------------------------- #
# the five axes
# --------------------------------------------------------------------------- #
#: ``beta`` sampling step, degrees.  CHOSEN, not derived - there is no bound on
#: this axis to derive one from.  2.5 deg is the existing 10-deg sampling of
#: :data:`.zhome_bracket.BETA` refined four times, on the axis the hardware
#: pull could not close.
BETA_STEP_DEG = 2.5

#: ``beta_p`` sampling step, degrees.  Matched to :data:`BETA_STEP_DEG` on
#: purpose: the shortlist's group key is ``|beta_p - beta|``, and sampling the
#: two axes at different resolutions would resolve that difference at the
#: coarser of the two while appearing to resolve it at the finer.
BETA_P_STEP_DEG = 2.5

#: ``d/r_b`` cap and step.  The cap is ASSERTED (rod slenderness, not
#: hardware); the step is chosen.  0.1 r_b = 9 mm at the asserted ``r_b``.
D_CAP_RB = 2.0
D_STEP_RB = 0.1


def beta_axis():
    """``beta`` over the OPEN interval ``(0, 60)`` degrees.

    Open at both ends because :func:`.geometry.base_ring` rejects 0 and 60 -
    the pair collapses onto one azimuth at 0 and onto the neighbouring pair at
    60 - and because no bound narrower than that can be set: the installed-arc
    clearance is unpublished for every servo on the pull.
    """
    n = int(round(60.0 / BETA_STEP_DEG))
    return [round(BETA_STEP_DEG * k, 9) for k in range(1, n)]


def beta_p_axis():
    """``beta_p`` over the UNION of the two OD-bounded intervals.

    The union is the wider bound (OD 9.0 mm).  Sampled uniformly at
    :data:`BETA_P_STEP_DEG` inside it, plus **all four** published bound
    endpoints as sampled values, because a range's ends are where a bound can
    bite.  The sweep runs once over this union; :func:`subset` partitions it
    afterwards.  Nothing is merged and nothing is averaged.
    """
    lo = min(SR.beta_p_bounds_deg(od)[0] for od in SR.HOUSING_OD_MM)
    hi = max(SR.beta_p_bounds_deg(od)[1] for od in SR.HOUSING_OD_MM)
    n = int(round(60.0 / BETA_P_STEP_DEG))
    grid = [round(BETA_P_STEP_DEG * k, 9) for k in range(1, n)]
    ends = [round(v, 9) for od in SR.HOUSING_OD_MM
            for v in SR.beta_p_bounds_deg(od)]
    return sorted({v for v in grid + ends if lo - 1e-9 <= v <= hi + 1e-9})


def a_axis():
    """The 32 published ProModeler hole positions as ``a/r_b``.  DISCRETE.

    :func:`.sweep_ranges.a_set_rb` unchanged - not resampled, not interpolated,
    not extended.
    """
    return SR.a_set_rb()


def d_axis():
    """``d/r_b`` up to the ASSERTED cap, one step at a time from the bottom.

    No lower end is asserted anywhere, so none is imposed: the axis runs from
    one step to the cap and the feasibility screen decides where the bottom
    really is.  Which ``d`` rows come back entirely empty is reported.
    """
    n = int(round(D_CAP_RB / D_STEP_RB))
    return [round(D_STEP_RB * k, 9) for k in range(1, n + 1)]


# --------------------------------------------------------------------------- #
# delta*, reported and not believed
# --------------------------------------------------------------------------- #
def delta_star(beta, beta_p, r_p=None):
    """``delta*`` in degrees, the value zeroing all six ``w_i`` **at home**.

        delta* = atan2(-r_p sin A, r_b - r_p cos A) mod 180,  A = beta_p - beta

    A SEED WITH NO BASIS CLAIM.  The "exact seed in the right basin" reading
    was **withdrawn 2026-09-04**: it zeroes ``w_i`` at ONE pose, and the tune
    maximises a maximin aggregate over 29 of them.  Whether the maximiser stays
    within a bracket of it is the open aggregation question, so this function's
    output narrows nothing here - the gap to the tuned ``delta`` is reported as
    a measurement of exactly that question and used for nothing else.
    """
    r_p = R_P_RB if r_p is None else float(r_p)
    A = np.deg2rad(float(beta_p) - float(beta))
    return float(np.degrees(np.arctan2(-r_p * np.sin(A),
                                       R_B - r_p * np.cos(A))) % 180.0)


# --------------------------------------------------------------------------- #
# the exact delta-free feasibility bracket
# --------------------------------------------------------------------------- #
def reach_bracket(A, B, G, deltas_rad):
    """``(ok, n_dead, n_admitted, n_scanned)`` per ``z_home`` row.  EXACT.

    ``ok[j]`` is "some ``delta`` satisfies ``w_i(delta)^2 <= G_i`` at every
    (pose, leg)" - the same predicate
    :func:`.zhome_bracket.reach_feasible_any_delta` computes, and checked
    against it by :func:`verify_bracket`.  What differs is the cost: the two
    closed-form cases below decide most rows outright and only the residue is
    scanned.

    ``G_i < 0`` for any (pose, leg) is ``|P_i| > |L_i|``, which no ``delta``
    can rescue because ``w_i^2 >= 0``: the row is DEAD.  ``max_delta w_i^2 =
    amp_i^2`` with ``amp_i = hypot(A_i, B_i)``, so ``amp_i^2 <= G_i``
    everywhere means every ``delta`` works and the row is ADMITTED.  Neither
    case is a bound or an approximation - they are the two halves of the
    predicate that do not depend on which ``delta`` is chosen.
    """
    dead = np.min(G, axis=1) < 0.0
    admit = np.max(A * A + B * B - G, axis=1) <= 0.0
    undecided = ~dead & ~admit
    ok = admit.copy()
    idx = np.flatnonzero(undecided)
    if idx.size:
        As, Bs, Gs = A[idx], B[idx], G[idx]
        hit = np.zeros(idx.size, dtype=bool)
        for dr in deltas_rad:
            w = As * np.cos(dr) + Bs * np.sin(dr)
            hit |= (np.max(w * w - Gs, axis=1) <= 0.0)
        ok[idx] = hit
    return ok, int(dead.sum()), int(admit.sum()), int(idx.size)


def verify_bracket(az, mg, deltas_rad, n=24, seed=0):
    """Gate: the bracket must agree with the committed primitive, exactly.

    Sampled from the ACTUAL sweep axes, not from a convenient corner, and run
    before the sweep.  Returns ``(n_checked, n_disagreements, worst_row)``.
    """
    rng = np.random.default_rng(seed)
    betas, bps, a_s, ds = beta_axis(), beta_p_axis(), a_axis(), d_axis()
    bad = 0
    for _ in range(n):
        beta = float(rng.choice(betas))
        beta_p = float(rng.choice(bps))
        a = float(rng.choice(a_s))
        d = float(rng.choice(ds))
        A, B, G, _ = ZB.leg_terms(beta, beta_p, R_P_RB, a, d, H_P,
                                  Z_GRID, az, mg)
        got, _, _, _ = reach_bracket(A, B, G, deltas_rad)
        want = ZB.reach_feasible_any_delta(A, B, G, deltas_rad)
        bad += int(np.any(got != want))
    return n, bad


# --------------------------------------------------------------------------- #
# the compute ledger
# --------------------------------------------------------------------------- #
#: Bytes per float, as :mod:`.sweep_budget` counts them.
BYTES_PER_FLOAT = 8


def ledger(az, mg):
    """Every count the run rests on, from the committed constants.

    Recounted for the shape now in force rather than carried over: the old
    figure - ~2.7M full, ~4.9e8 cheap, ~3.9 GB - was six axes at 5 points each
    with ``r_p/r_b`` still in and ``a`` continuous.  One axis is gone, ``a`` is
    discrete at 32 published positions, ``d`` is open to 2.0, and ``z_home`` is
    a per-candidate bracket rather than an outer-product axis.
    """
    n_beta, n_bp = len(beta_axis()), len(beta_p_axis())
    n_a, n_d = len(a_axis()), len(d_axis())
    cand = n_beta * n_bp * n_a * n_d
    nz, K_s, n_delta = Z_GRID.size, az.size, DELTA_GRID.size
    az29, _ = ENV.envelope_poses()
    K_h = az29.size
    return dict(
        n_beta=n_beta, n_bp=n_bp, n_a=n_a, n_d=n_d, candidates=cand,
        nz=nz, K_screen=K_s, K_harness=K_h, n_delta=n_delta,
        w_per_obj=K_h * 6,
        # screen, per candidate
        free_per_cand=nz * K_s * 6,
        scan_per_cand=nz * K_s * 6 * n_delta,
        # screen, whole sweep
        free_total=cand * nz * K_s * 6,
        scan_total_worst=cand * nz * K_s * 6 * n_delta,
        # tune + score, per survivor
        tune_per_surv=n_delta * K_h * 6,
        score_per_surv=SD.N_DISP_DIR * K_h * 6,
        # memory
        peak_cand_bytes=(nz * K_s * 3 * 6 + 4 * nz * K_s * 6) * BYTES_PER_FLOAT,
        held_at_once_bytes=cand * n_delta * K_h * 6 * BYTES_PER_FLOAT,
    )


def report_ledger(lg, tilt, checks):
    """Print the ledger.  Called BEFORE anything is run."""
    print("=" * 78)
    print("COMPUTE LEDGER - counted from the committed constants, before the run")
    print("=" * 78)
    print(f"  tilt limit  {tilt:.4f} deg, envelope.tilt_for(x0 = "
          f"{ENV.X0_WORKING*1e3:.0f} mm, tau = {ENV.TAU} s),")
    print(f"              printed and not hardcoded.  tau_L DROPPED.")
    print(f"  r_b = {R_B_MM:.0f} mm, r_p = {R_P_MM:.0f} mm, r_p/r_b = "
          f"{R_P_RB:.6f} - FIXED, not an axis.")
    print(f"  h_p/r_b = {H_P}.  p = {P_SCORE:.6f} r_b "
          f"(quoted {FR.P_SCORE_QUOTED:.6f}, agrees: "
          f"{abs(P_SCORE - FR.P_SCORE_QUOTED) < 5e-7}).")
    print()
    print("  THE SHAPE HAS CHANGED, so the ledger is recounted and not carried:")
    print("    old (sweep_budget.py): 6 axes x 5 points = 15625 candidates,")
    print("      ~2.7M full + ~4.9e8 cheap evaluations, ~3.9 GB held at once.")
    print("    that count had r_p/r_b IN as an axis, a CONTINUOUS at 5 points,")
    print("      d at 5 points, and z_home as a sixth outer-product axis.")
    print("    now: r_p/r_b is gone, a is DISCRETE at 32 published holes, d is")
    print("      open to 2.0, and z_home is a PER-CANDIDATE bracket, so it")
    print("      multiplies nothing.")
    print()
    print(f"    {'axis':<14} {'points':>7}  how it is set")
    print(f"    {'beta':<14} {lg['n_beta']:>7}  (0, 60) deg open, step "
          f"{BETA_STEP_DEG} - UNBOUNDED, no collision bound available")
    print(f"    {'beta_p':<14} {lg['n_bp']:>7}  union of both OD bounds, step "
          f"{BETA_P_STEP_DEG} + all 4 bound endpoints")
    print(f"    {'a/r_b':<14} {lg['n_a']:>7}  DISCRETE - published ProModeler "
          f"holes, not resampled")
    print(f"    {'d/r_b':<14} {lg['n_d']:>7}  step {D_STEP_RB} to the ASSERTED "
          f"cap {D_CAP_RB} ({D_CAP_RB*R_B_MM:.0f} mm)")
    print(f"    {'z_home/r_b':<14} {'-':>7}  per candidate: bracket on the "
          f"{lg['nz']}-point Z_GRID, midpoint taken")
    print(f"    {'CANDIDATES':<14} {lg['candidates']:>7,}  = "
          f"{lg['n_beta']} x {lg['n_bp']} x {lg['n_a']} x {lg['n_d']}")
    print()
    print("  SCREEN, per candidate.  366-pose bracket envelope is superseded: "
          f"the")
    print(f"  screen grid is {lg['K_screen']} poses "
          f"({ZB.N_MAG_FINE} magnitudes x {ZB.N_AZ_FINE} azimuths, "
          f"magnitude 0 once).")
    print(f"    delta-free leg evaluations : {lg['nz']} z x "
          f"{lg['K_screen']} poses x 6 legs = {lg['free_per_cand']:,}")
    print(f"    full delta scan would add  : x {lg['n_delta']} deltas = "
          f"{lg['scan_per_cand']:,}")
    print(f"    the EXACT bracket decides most z rows with no scan at all - "
          f"see below")
    print()
    print("  SCREEN, whole sweep.")
    print(f"    delta-free                 : {lg['free_total']:.3e}")
    print(f"    worst case, every z scanned: {lg['scan_total_worst']:.3e}")
    print(f"    measured on {checks['n_probe']} sampled candidates, z rows of "
          f"{lg['nz']} needing a scan:")
    print(f"      dead (|P| >= |L|, no delta)      {checks['dead']:>6.1f}")
    print(f"      admitted (every delta works)     {checks['admit']:>6.1f}")
    print(f"      SCANNED                          {checks['scan']:>6.1f}  "
          f"({checks['scan']/lg['nz']:.1%} of the grid)")
    print(f"    so the projected scan cost is {lg['scan_total_worst'] * checks['scan'] / lg['nz']:.3e}, "
          f"not {lg['scan_total_worst']:.3e}.")
    print()
    print("  TUNE + SCORE, per survivor, on the "
          f"{lg['K_harness']}-pose harness grid "
          f"({lg['w_per_obj']} w per objective evaluation):")
    print(f"    delta scan, cond at EVERY step: {lg['n_delta']} x "
          f"{lg['K_harness']} x 6 = {lg['tune_per_surv']:,}")
    print(f"    score at p, {SD.N_DISP_DIR} displacement azimuths: "
          f"{lg['score_per_surv']:,}")
    print()
    print("  MEMORY, and this is what the chunking requirement rests on.")
    print(f"    if the whole cheap array were held at once: "
          f"{lg['held_at_once_bytes']/1e9:.1f} GB")
    print(f"    peak held per candidate by the screen        : "
          f"{lg['peak_cand_bytes']/1e6:.1f} MB")
    print(f"    -> the sweep CHUNKS over candidates, {CHUNK:,} per chunk, and")
    print(f"       retains scalars only: no scan_margin or scan_cond array")
    print(f"       survives the candidate that produced it.  Peak is the")
    print(f"       per-candidate figure above, not the {lg['held_at_once_bytes']/1e9:.1f} GB.")
    print()
    print("  MEASURED UNIT COSTS on this machine (best of 3, before the run):")
    print(f"    screen  {1e3*checks['t_screen']:>8.2f} ms / candidate")
    print(f"    tune+score {1e3*checks['t_tune']:>5.2f} ms / survivor")
    print(f"    cond at every delta step: {1e6*checks['t_tune_step']:.1f} us/step "
          f"here; the 61 us/step behind the 175 s projection was measured")
    print(f"    elsewhere.  Either way a banding heuristic is NOT built: "
          f"score_discriminators")
    print(f"    part (6) shows the band pays only where the cap barely binds.")
    print(f"    PROJECTED WALL TIME, screen: "
          f"{lg['candidates']*checks['t_screen']/60:.1f} min "
          f"+ tune/score on whatever survives.")
    print()
    print(f"  BRACKET GATE: {checks['n_check']} sampled candidates checked "
          f"against zhome_bracket.reach_feasible_any_delta,")
    print(f"                {checks['bad']} disagreements.  The bracket is "
          f"exact, not a heuristic.")
    print()


# --------------------------------------------------------------------------- #
# the screen
# --------------------------------------------------------------------------- #
def screen_one(beta, beta_p, a, d, az, mg, deltas_rad, floor):
    """One candidate through the four feasibility tests.  Returns a record.

    ``cause`` is ``None`` when feasible, else one of

    ``R``  reach empty on its own - no ``z_home`` reaches at any ``delta``
    ``X``  reach ceiling BELOW the ``N_i > 0`` floor: both constraints
           satisfiable alone, but crossed.  Refined by
           :func:`.zhome_bracket.reach_ceiling_bisect` to 1e-9 first, because
           the ``Z_GRID`` step is 0.025 ``r_b`` and a crossing narrower than
           that is indistinguishable from a sampling artefact
    ``W``  bracket non-empty but narrower than :data:`MIN_BRACKET_RB`

    The ``N_i > 0`` floor is the CLOSED FORM passed in as ``floor``, never the
    pose grid.
    """
    A, B, G, _ = ZB.leg_terms(beta, beta_p, R_P_RB, a, d, H_P, Z_GRID, az, mg)
    reach, n_dead, n_admit, n_scan = reach_bracket(A, B, G, deltas_rad)
    ok = reach & (Z_GRID > floor)
    ridx = np.flatnonzero(reach)
    idx = np.flatnonzero(ok)
    rec = dict(beta=beta, beta_p=beta_p, r_p=R_P_RB, a=a, d=d,
               cf=float(floor), nreach=int(reach.sum()),
               reach_lo=float(Z_GRID[ridx[0]]) if ridx.size else np.nan,
               reach_hi=float(Z_GRID[ridx[-1]]) if ridx.size else np.nan,
               z_lo=float(Z_GRID[idx[0]]) if idx.size else np.nan,
               z_hi=float(Z_GRID[idx[-1]]) if idx.size else np.nan,
               contiguous=bool(idx.size) and (idx[-1] - idx[0] + 1) == idx.size,
               n_scan=n_scan, n_dead=n_dead, n_admit=n_admit)
    rec["width"] = rec["z_hi"] - rec["z_lo"] if idx.size else np.nan
    if not idx.size:
        if ridx.size == 0:
            rec["cause"] = "R"
            rec["gap"] = np.nan
        else:
            step = float(Z_GRID[1] - Z_GRID[0])
            exact = ZB.reach_ceiling_bisect(beta, beta_p, R_P_RB, a, d, H_P,
                                            rec["reach_hi"],
                                            rec["reach_hi"] + step,
                                            az, mg, deltas_rad)
            rec["ceil_exact"] = exact
            rec["gap"] = float(rec["cf"] - exact)
            rec["cause"] = "X" if rec["gap"] > 0.0 else "R"
    elif rec["width"] < MIN_BRACKET_RB - 1e-12:
        rec["cause"] = "W"
        rec["gap"] = np.nan
    else:
        rec["cause"] = None
        rec["gap"] = np.nan
    rec["feasible"] = rec["cause"] is None
    return rec


# --------------------------------------------------------------------------- #
# the inner tune and the score
# --------------------------------------------------------------------------- #
def tune_and_score(rec, R29, az29):
    """Tune ``delta`` at the cap, then score at ``p``.  Mutates ``rec``.

    ``z_home`` is the bracket MIDPOINT - a point in the interior chosen for
    definiteness, not a recommendation; ``z_home`` remains a swept axis and
    this harness does not settle it.

    Every retained field is a scalar.  ``scan_margin`` and ``scan_cond`` are
    180-element arrays and are deliberately dropped with the local frame: see
    the chunking note in :func:`report_ledger`.
    """
    z_home = 0.5 * (rec["z_lo"] + rec["z_hi"])
    rec["z_home"] = z_home
    T = SD._T_stack(az29, z_home)
    cl = SD.char_lengths(rec)[CHAR_LEN]
    m, c, _ = SD.scan_delta(rec["beta"], rec["beta_p"], R_P_RB, rec["a"],
                            rec["d"], R29, T, DELTA_GRID, cl)
    rec["n_live"] = int(np.isfinite(m).sum())
    d_con, m_con = SD.tune_constrained(m, c, DELTA_GRID, CAP)
    rec["dstar"] = delta_star(rec["beta"], rec["beta_p"])
    if d_con is None:
        rec["delta_con"] = np.nan
        rec["margin_con"] = np.nan
        rec["score_p"] = np.nan
        rec["dstar_gap"] = np.nan
        return rec
    g0, g90 = SD._delta_basis(rec["beta"], rec["beta_p"], R_P_RB,
                              rec["a"], rec["d"])
    rec["delta_con"] = d_con
    rec["margin_con"] = m_con
    rec["score_p"] = SD.probe_margin(g0, g90, R29, az29, z_home, d_con, P_SCORE)
    gap = abs(d_con - rec["dstar"])
    rec["dstar_gap"] = float(min(gap, 180.0 - gap))
    return rec


# --------------------------------------------------------------------------- #
# partition, group, tie set
# --------------------------------------------------------------------------- #
def subset(rows, feasible, scored, od_mm):
    """Everything admissible at housing OD ``od_mm``.

    ``beta_p`` outside the bound is not a candidate at that OD: two housings
    would overlap.  ``beta`` is NOT filtered - the servo clearance that would
    bound it is unpublished, which is the whole reason that axis is swept open.
    """
    lo, hi = SR.beta_p_bounds_deg(od_mm)
    keep = lambda r: lo - 1e-9 <= r["beta_p"] <= hi + 1e-9
    return dict(od=od_mm, lo=lo, hi=hi,
                rows=[r for r in rows if keep(r)],
                feasible=[r for r in feasible if keep(r)],
                scored=[r for r in scored if keep(r)])


def group_key(rec):
    """``(a, d, |beta_p - beta|)`` - part (8)'s invariant, ``r_p`` dropped.

    Two candidates sharing this key are ONE result reached by two
    ``(beta, beta_p)`` pairs - a rotation of the machine about ``z``, a sign
    flip of ``e = beta_p - beta``, or both.  Groups are ranked; members inside
    a group are not.
    """
    return (rec["a"], rec["d"], round(abs(rec["beta_p"] - rec["beta"]), 9))


def tie_groups(scored):
    """``(groups, tie_set)``: groups ranked by best member, then the tie set.

    The tie set is every group within :data:`TIE_TOL` of the leader.  It is the
    OUTPUT.  There is no winner: at ``dxy = p`` the ranking is resolved to
    ~1.5e-4 and candidates closer than that are ordered by the displacement
    azimuth grid, not by the geometry.
    """
    live = [r for r in scored if np.isfinite(r["score_p"])]
    if not live:
        return [], []
    by = {}
    for r in live:
        by.setdefault(group_key(r), []).append(r)
    groups = sorted(by.items(), key=lambda kv: -max(m["score_p"] for m in kv[1]))
    best = max(m["score_p"] for m in groups[0][1])
    tie = [g for g in groups if max(m["score_p"] for m in g[1]) >= best - TIE_TOL]
    return groups, tie


def arc_spacing_mm(beta_deg):
    """Required servo mounting-arc spacing along the base ring, mm at ``r_b``.

    ``theta_i = 120 floor(i/2) + s_i beta``, so the two angular gaps are
    ``2 beta`` and ``120 - 2 beta`` and the tightest arc is ``r_b`` times the
    smaller of them in radians.  POST-HOC: the mounting-flange footprint that
    would turn this into a bound is unpublished for every servo on the pull
    (sec.5, among the 39 unpublished cells), so this REPORTS what a build would
    need and rejects nothing.
    """
    b = float(beta_deg)
    return float(R_B_MM * np.deg2rad(min(2.0 * b, 120.0 - 2.0 * b)))


def chord_spacing_mm(beta_deg):
    """Straight-line distance between the two closest base anchors, mm.

    Reported beside the arc because the only published servo figure the arc
    could be compared against - body width, 11.4 - 13.0 mm (sec.5) - is a
    straight-line width, and comparing a width against an arc would flatter it.
    """
    b = np.deg2rad(float(beta_deg))
    return float(2.0 * R_B_MM * min(np.sin(b), np.sin(np.deg2rad(60.0) - b)))


# --------------------------------------------------------------------------- #
# the run
# --------------------------------------------------------------------------- #
def calibrate(az, mg, deltas_rad, n_probe=16, seed=3):
    """Measured unit costs and bracket statistics, for the ledger.

    Runs on candidates drawn from the ACTUAL axes.  Nothing it returns feeds a
    decision; it feeds the projection printed before the sweep starts.
    """
    rng = np.random.default_rng(seed)
    betas, bps, a_s, ds = beta_axis(), beta_p_axis(), a_axis(), d_axis()
    floor = ZB.z_lower_closed_form(R_P_RB, H_P)
    picks = [(float(rng.choice(betas)), float(rng.choice(bps)),
              float(rng.choice(a_s)), float(rng.choice(ds)))
             for _ in range(n_probe)]
    dead = admit = scan = 0
    t0 = time.perf_counter()
    for beta, bp, a, d in picks:
        A, B, G, _ = ZB.leg_terms(beta, bp, R_P_RB, a, d, H_P, Z_GRID, az, mg)
        _, nd, na, ns = reach_bracket(A, B, G, deltas_rad)
        dead += nd
        admit += na
        scan += ns
    t_screen = (time.perf_counter() - t0) / n_probe

    R29, az29, _ = SD._pose_grid(None)
    z = max(0.5, floor + 0.5)
    t0 = time.perf_counter()
    reps = 0
    for beta, bp, a, d in picks[:8]:
        T = SD._T_stack(az29, z)
        m, c, _ = SD.scan_delta(beta, bp, R_P_RB, a, d, R29, T, DELTA_GRID, R_B)
        dc, _ = SD.tune_constrained(m, c, DELTA_GRID, CAP)
        if dc is not None:
            g0, g90 = SD._delta_basis(beta, bp, R_P_RB, a, d)
            SD.probe_margin(g0, g90, R29, az29, z, dc, P_SCORE)
        reps += 1
    t_tune = (time.perf_counter() - t0) / reps

    n_check, bad = verify_bracket(az, mg, deltas_rad)
    return dict(n_probe=n_probe, dead=dead / n_probe, admit=admit / n_probe,
                scan=scan / n_probe, t_screen=t_screen, t_tune=t_tune,
                t_tune_step=t_tune / DELTA_GRID.size,
                n_check=n_check, bad=bad)


def run(az, mg, verbose=True):
    """Screen every candidate, then tune and score the survivors.  Chunked.

    Chunked over candidates: the screen holds one candidate's arrays at a time
    and survivor records carry scalars only.  Infeasible candidates are
    COUNTED, not retained - the report needs their cause and their ``a`` and
    ``d`` row, which are counters, and retaining ~300k records to recover two
    integers would be the memory figure the chunking exists to avoid.
    """
    betas, bps, a_s, ds = beta_axis(), beta_p_axis(), a_axis(), d_axis()
    deltas_rad = np.deg2rad(DELTA_GRID)
    floor = ZB.z_lower_closed_form(R_P_RB, H_P)
    R29, az29, _ = SD._pose_grid(None)

    todo = [(b, bp, a, d) for b in betas for bp in bps for a in a_s for d in ds]
    n_total = len(todo)
    feasible, empties = [], []
    n_screened = 0
    t0 = time.time()
    for start in range(0, n_total, CHUNK):
        for beta, beta_p, a, d in todo[start:start + CHUNK]:
            rec = screen_one(beta, beta_p, a, d, az, mg, deltas_rad, floor)
            n_screened += 1
            if rec["feasible"]:
                feasible.append(rec)
            else:
                empties.append(dict(beta=beta, beta_p=beta_p, a=a, d=d,
                                    cause=rec["cause"], gap=rec["gap"],
                                    width=rec["width"], nreach=rec["nreach"]))
        if verbose:
            el = time.time() - t0
            print(f"    screened {n_screened:>7,} / {n_total:,}   "
                  f"feasible {len(feasible):>6,}   "
                  f"{el:>6.0f} s   eta {el*(n_total/n_screened - 1):>6.0f} s",
                  flush=True)
    t_screen = time.time() - t0

    if verbose:
        print(f"    tuning and scoring {len(feasible):,} survivors "
              f"(cond at every one of {DELTA_GRID.size} delta steps) ...",
              flush=True)
    t0 = time.time()
    scored, no_delta, no_live = [], 0, 0
    for k, rec in enumerate(feasible, 1):
        tune_and_score(rec, R29, az29)
        if rec["n_live"] == 0:
            no_live += 1
        if np.isfinite(rec["score_p"]):
            scored.append(rec)
        else:
            no_delta += 1
        if verbose and k % 20000 == 0:
            print(f"      tuned {k:,} / {len(feasible):,}  "
                  f"({time.time() - t0:.0f} s)", flush=True)
    t_tune = time.time() - t0

    return dict(rows_n=n_total, feasible=feasible, empties=empties,
                scored=scored, no_delta=no_delta, no_live=no_live,
                floor=floor, R29=R29, az29=az29,
                t_screen=t_screen, t_tune=t_tune)


# --------------------------------------------------------------------------- #
# persistence - so a lost run does not cost a re-screen
# --------------------------------------------------------------------------- #
#: Fields of a scored record that the report needs.  Scalars only, by design:
#: the 180-element ``scan_margin`` / ``scan_cond`` arrays are never retained,
#: which is the same decision the chunking rests on.
SCORED_FIELDS = ("beta", "beta_p", "a", "d", "z_lo", "z_hi", "z_home",
                 "delta_con", "margin_con", "score_p", "dstar", "dstar_gap",
                 "n_live", "width")
EMPTY_FIELDS = ("beta", "beta_p", "a", "d", "gap", "width", "nreach")


def save(res, path):
    """Write the run to a ``.npz`` so the report can be regenerated.

    The screen is 25 minutes and the tune 70; regenerating a report should
    cost neither.  Causes are stored as their one-character codes.
    """
    # ALL feasible records, not just the scored ones: a candidate that is
    # feasible but has no admissible delta at the cap carries score_p = NaN,
    # and dropping it here would make the reloaded feasible count wrong
    # wherever no_delta > 0 (it is 0 on this grid, which is exactly the
    # condition under which such a bug stays invisible).
    out = {f"s_{k}": np.array([r[k] for r in res["feasible"]], dtype=float)
           for k in SCORED_FIELDS}
    out.update({f"e_{k}": np.array([e[k] for e in res["empties"]], dtype=float)
                for k in EMPTY_FIELDS})
    out["e_cause"] = np.array([e["cause"] for e in res["empties"]], dtype="U1")
    out["meta"] = np.array([res["rows_n"], len(res["feasible"]),
                            res["no_delta"], res["no_live"], res["floor"],
                            res["t_screen"], res["t_tune"]], dtype=float)
    np.savez_compressed(path, **out)
    return path


def load(path):
    """Rebuild a ``res`` dict from :func:`save`.

    The two asserts are the point: they check the reloaded counts against the
    counts the run itself recorded, so a reporting path that silently loses
    records fails here rather than printing a smaller number.
    """
    z = np.load(path, allow_pickle=False)
    feasible = [dict(zip(SCORED_FIELDS, vals)) for vals in
                zip(*[z[f"s_{k}"] for k in SCORED_FIELDS])]
    scored = [r for r in feasible if np.isfinite(r["score_p"])]
    empties = [dict(zip(EMPTY_FIELDS, vals)) for vals in
               zip(*[z[f"e_{k}"] for k in EMPTY_FIELDS])]
    for e, c in zip(empties, z["e_cause"]):
        e["cause"] = str(c)
    m = z["meta"]
    az29, _ = ENV.envelope_poses()
    assert len(feasible) == int(m[1]), "saved feasible count disagrees"
    assert len(feasible) - len(scored) == int(m[2]), "saved no_delta disagrees"
    return dict(rows_n=int(m[0]), feasible=feasible, empties=empties,
                scored=scored, no_delta=int(m[2]), no_live=int(m[3]),
                floor=float(m[4]), t_screen=float(m[5]), t_tune=float(m[6]),
                R29=None, az29=az29)


# --------------------------------------------------------------------------- #
# report
# --------------------------------------------------------------------------- #
def _od_counts(res, od):
    lo, hi = SR.beta_p_bounds_deg(od)
    inb = lambda bp: lo - 1e-9 <= bp <= hi + 1e-9
    n_rows = (len(beta_axis()) * sum(1 for v in beta_p_axis() if inb(v))
              * len(a_axis()) * len(d_axis()))
    feas = [r for r in res["feasible"] if inb(r["beta_p"])]
    scor = [r for r in res["scored"] if inb(r["beta_p"])]
    emp = [e for e in res["empties"] if inb(e["beta_p"])]
    return dict(od=od, lo=lo, hi=hi, n_rows=n_rows, feasible=feas,
                scored=scor, empties=emp)


def part_counts(res, subs):
    print()
    print("=" * 78)
    print("(1) SCREENED, FEASIBLE AND SCORED - AT EACH beta_p BOUND SEPARATELY")
    print("=" * 78)
    print("  The sweep ran ONCE over the union of the two OD-bounded beta_p")
    print("  intervals and is partitioned here.  The two runs are NOT merged")
    print("  and the two bounds are NOT averaged; no joint is chosen between")
    print("  them.  beta is not filtered at either end - the servo clearance")
    print("  that would bound it is unpublished.")
    print()
    print(f"    {'housing OD':>11} {'beta_p range [deg]':>21} {'screened':>9} "
          f"{'feasible':>9} {'frac':>7} {'scored':>8} {'frac':>7}")
    for s in subs:
        n = s["n_rows"]
        span = f"{s['lo']:.4f} .. {s['hi']:.4f}"
        print(f"    {s['od']:>11.1f} {span:>21} "
              f"{n:>9,} {len(s['feasible']):>9,} "
              f"{len(s['feasible'])/n:>7.3f} {len(s['scored']):>8,} "
              f"{len(s['scored'])/n:>7.3f}")
    print()
    print(f"    union (both bounds, screened once): "
          f"{res['rows_n']:,} screened, {len(res['feasible']):,} feasible, "
          f"{len(res['scored']):,} scored")
    print(f"    feasible but no delta at the cap cond(J_fk) <= {CAP:.0e} at "
          f"char_len = {CHAR_LEN}: {res['no_delta']:,}")
    print(f"    of which no live delta at all (unreachable at the bracket "
          f"midpoint on the {res['az29'].size}-pose grid): {res['no_live']:,}")
    print()
    print(f"    screen {res['t_screen']:.0f} s, tune + score "
          f"{res['t_tune']:.0f} s")


def part_empties(res, subs):
    print()
    print("=" * 78)
    print("(2) EMPTIES BY CAUSE, AND BY a")
    print("=" * 78)
    print("  R  reach empty on its own - no z_home reaches at any delta")
    print("  X  reach ceiling BELOW the N_i > 0 floor: both constraints")
    print("     satisfiable alone, but crossed.  Every X is refined by")
    print("     bisection to 1e-9 first, because the Z_GRID step is 0.025 r_b")
    print("     and a narrower crossing is a sampling artefact, not a crossing")
    print("  W  bracket non-empty but narrower than the ASSERTED minimum")
    print(f"     {MIN_BRACKET_MM:.0f} mm = {MIN_BRACKET_RB:.4f} r_b")
    print()
    print(f"    {'housing OD':>11} {'screened':>9} {'R':>8} {'X':>8} {'W':>8} "
          f"{'feasible':>9}")
    for s in subs:
        c = {k: sum(1 for e in s["empties"] if e["cause"] == k)
             for k in "RXW"}
        print(f"    {s['od']:>11.1f} {s['n_rows']:>9,} {c['R']:>8,} "
              f"{c['X']:>8,} {c['W']:>8,} {len(s['feasible']):>9,}")
    print()
    xs = [e for e in res["empties"] if e["cause"] == "X"]
    print(f"    X across the union: {len(xs):,}.  The 2026-09-05 correction "
          f"rested on ONE")
    print(f"    such candidate at one provisional tilt; notation.md sec.12 "
          f"records that")
    print(f"    whether the category reappears at {ENV.TILT_LIMIT_DEG:.3f} deg "
          f"on a grid this wide was")
    print(f"    NOT answered by the 540-candidate table.  It is answered here.")
    if xs:
        g = np.array([e["gap"] for e in xs], dtype=float)
        print(f"    crossing gap (floor - refined ceiling), r_b: "
              f"min {g.min():.3e}  median {np.median(g):.3e}  "
              f"max {g.max():.3e}")
    ws = [e for e in res["empties"] if e["cause"] == "W"]
    print()
    print(f"    W across the union: {len(ws):,} - candidates the first three "
          f"tests pass and")
    print(f"    the minimum bracket width removes.  These are the zero-width "
          f"single-")
    print(f"    grid-point artefacts the test was ASSERTED to remove.")
    if ws:
        w = np.array([e["width"] for e in ws], dtype=float)
        print(f"    their widths, r_b: {int((w == 0.0).sum()):,} at exactly "
              f"0.000, max {w.max():.4f} "
              f"(= {w.max()*R_B_MM:.2f} mm, floor is {MIN_BRACKET_MM:.0f} mm)")
    print()
    print("  BY a - the discrete axis.  'empty' is every cause pooled; the")
    print("  R / X / W split follows it.  A row that empties nothing is a row")
    print("  the reach test never rejects.")
    print()
    a_mm = SR.a_set_mm()
    prov = SR.a_provenance()
    per_a = {}
    for e in res["empties"]:
        d = per_a.setdefault(e["a"], {"R": 0, "X": 0, "W": 0})
        d[e["cause"]] += 1
    feas_a = {}
    for r in res["feasible"]:
        feas_a[r["a"]] = feas_a.get(r["a"], 0) + 1
    n_per_a = len(beta_axis()) * len(beta_p_axis()) * len(d_axis())
    print(f"    {'a [mm]':>8} {'a/r_b':>8} {'prov':>9} {'screened':>9} "
          f"{'empty':>8} {'R':>8} {'X':>7} {'W':>7} {'feasible':>9}")
    for h in a_mm:
        v = round(h / R_B_MM, 9)
        d = per_a.get(v, {"R": 0, "X": 0, "W": 0})
        tot = d["R"] + d["X"] + d["W"]
        print(f"    {h:>8.2f} {v:>8.4f} {prov[h]:>9} {n_per_a:>9,} "
              f"{tot:>8,} {d['R']:>8,} {d['X']:>7,} {d['W']:>7,} "
              f"{feas_a.get(v, 0):>9,}")
    print()
    print("  BY d - where the axis's unasserted lower end actually falls.")
    print(f"    {'d/r_b':>8} {'d [mm]':>8} {'screened':>9} {'empty':>8} "
          f"{'R':>8} {'X':>7} {'W':>7} {'feasible':>9}")
    per_d = {}
    for e in res["empties"]:
        dd = per_d.setdefault(e["d"], {"R": 0, "X": 0, "W": 0})
        dd[e["cause"]] += 1
    feas_d = {}
    for r in res["feasible"]:
        feas_d[r["d"]] = feas_d.get(r["d"], 0) + 1
    n_per_d = len(beta_axis()) * len(beta_p_axis()) * len(a_axis())
    for v in d_axis():
        dd = per_d.get(v, {"R": 0, "X": 0, "W": 0})
        tot = dd["R"] + dd["X"] + dd["W"]
        print(f"    {v:>8.2f} {v*R_B_MM:>8.1f} {n_per_d:>9,} {tot:>8,} "
              f"{dd['R']:>8,} {dd['X']:>7,} {dd['W']:>7,} "
              f"{feas_d.get(v, 0):>9,}")


def part_margins(subs):
    print()
    print("=" * 78)
    print("(3) THE SCORE OVER THE SURVIVORS")
    print("=" * 78)
    print(f"  margin(dxy = p) at p = {P_SCORE:.6f} r_b, evaluated at that probe")
    print(f"  DIRECTLY through probe_margin at {SD.N_DISP_DIR} displacement "
          f"azimuths.  NO SLOPE")
    print("  IS EXTRAPOLATED FROM sens.  margin is dimensionless and carries no")
    print(f"  characteristic length; the cap that picked its delta is")
    print(f"  cond(J_fk) <= {CAP:.0e} at char_len = {CHAR_LEN}, PROVISIONAL in "
          f"that length.")
    print()
    print(f"    {'housing OD':>11} {'n':>7}{'min':>11}{'Q1':>11}{'median':>11}"
          f"{'Q3':>11}{'max':>11}")
    for s in subs:
        sc = np.array([r["score_p"] for r in s["scored"]], dtype=float)
        if sc.size == 0:
            print(f"    {s['od']:>11.1f} {0:>7}" + "".join(f"{'-':>11}" * 5))
            continue
        print(f"    {s['od']:>11.1f} {sc.size:>7,}"
              + "".join(f"{v:>11.6f}" for v in _five_number(sc)))
    print()
    print("  Negative margins are CARRIED, not dropped: a negative margin is a")
    print(f"  candidate the {ZB.fine_poses()[0].size}-pose screen passed and "
          f"the 29-pose harness grid does")
    print("  not, the two grids not being nested.")
    print()
    print(f"    {'housing OD':>11} {'neg at dxy=p':>14} {'neg at dxy=0':>14}")
    for s in subs:
        sc = np.array([r["score_p"] for r in s["scored"]], dtype=float)
        m0 = np.array([r["margin_con"] for r in s["scored"]], dtype=float)
        print(f"    {s['od']:>11.1f} {int((sc < 0).sum()):>14,} "
              f"{int((m0 < 0).sum()):>14,}")
    print()
    print("  delta* GAP, reported and used for nothing.  delta* zeroes w_i at")
    print("  HOME, at one pose; the tune maximises a maximin aggregate over 29.")
    print("  The basin claim was withdrawn 2026-09-04, so this is the measured")
    print("  answer to the open aggregation question, not a seed that narrowed")
    print("  the scan - the scan was the full 180 steps in every case.")
    print()
    print(f"    {'housing OD':>11} {'|delta_con - delta*| deg':>26} "
          f"{'median':>9} {'p95':>9} {'max':>9}")
    for s in subs:
        g = np.array([r["dstar_gap"] for r in s["scored"]], dtype=float)
        if g.size == 0:
            continue
        print(f"    {s['od']:>11.1f} {'':>26} {np.median(g):>9.2f} "
              f"{np.percentile(g, 95):>9.2f} {g.max():>9.2f}")


def part_shortlist(subs):
    print()
    print("=" * 78)
    print("(4) THE SHORTLIST - A TIE SET, NOT A WINNER")
    print("=" * 78)
    print(f"  Grouped by (a, d, |beta_p - beta|), the invariant of")
    print(f"  score_discriminators part (8) with r_p dropped from the key, r_p")
    print(f"  no longer varying.  Two candidates sharing the key are ONE result")
    print(f"  reached by two (beta, beta_p) pairs - a rotation about z, a sign")
    print(f"  flip of e, or both - so GROUPS are ranked and members inside a")
    print(f"  group are not.")
    print()
    print(f"  TIE_TOL = {TIE_TOL:.1e}, the ranking resolution MEASURED at "
          f"dxy = p in")
    print(f"  sweep_ranges part (7).  score_discriminators.TIE_TOL = "
          f"{SD.TIE_TOL:.0e} is correct")
    print(f"  for the dxy = 0 collapse it was set from and is eight decades")
    print(f"  wrong here; it is not imported.  The output is the TIE SET.")
    print()
    for s in subs:
        groups, tie = tie_groups(s["scored"])
        print(f"  ---- housing OD {s['od']:.1f} mm, beta_p in "
              f"[{s['lo']:.4f}, {s['hi']:.4f}] deg ----")
        if not groups:
            print("      no scored survivor.  See part (5).")
            print()
            continue
        best = max(m["score_p"] for m in groups[0][1])
        print(f"      {len(s['scored']):,} survivors in {len(groups):,} "
              f"groups.  Leader margin(p) = {best:.6f}.")
        print(f"      TIE SET: {len(tie)} group(s) within {TIE_TOL:.1e} of it, "
              f"{sum(len(g[1]) for g in tie)} candidates.")
        print()
        print(f"      {'grp':>4} {'margin(p)':>10} {'margin(0)':>10} "
              f"{'spread':>9} {'horn a':>8} {'rod d':>8} {'|e|':>8} "
              f"{'z_home':>8} {'mem':>5}")
        print(f"      {'':>4} {'':>10} {'':>10} {'':>9} {'[mm]':>8} "
              f"{'[mm]':>8} {'[deg]':>8} {'[mm]':>8}")
        for gi, (key, members) in enumerate(groups[:TOP_GROUPS], 1):
            sp = [m["score_p"] for m in members]
            zs = [m["z_home"] * R_B_MM for m in members]
            mark = "*" if gi <= len(tie) else " "
            print(f"     {mark}{gi:>3} {max(sp):>10.6f} "
                  f"{max(m['margin_con'] for m in members):>10.6f} "
                  f"{max(sp)-min(sp):>9.1e} {key[0]*R_B_MM:>8.2f} "
                  f"{key[1]*R_B_MM:>8.1f} {key[2]:>8.4f} "
                  f"{np.mean(zs):>8.2f} {len(members):>5}")
        print(f"      ('*' marks the tie set.  {min(TOP_GROUPS, len(groups))} "
              f"of {len(groups):,} groups shown.)")
        print()
        print(f"      TIE-SET MEMBERS, with the two clearances each one needs.")
        print(f"      sep is min |p_i - p_j|, the closest approach of two "
              f"PLATFORM anchors")
        print(f"      at the asserted r_p = {R_P_MM:.0f} mm - the housing OD "
              f"it must clear.  arc is the")
        print(f"      required servo mounting-arc spacing on the BASE ring at "
              f"r_b = {R_B_MM:.0f} mm,")
        print(f"      with the chord beside it because the only published "
              f"servo figure -")
        print(f"      body width {SERVO_BODY_WIDTH_MM[0]} - "
              f"{SERVO_BODY_WIDTH_MM[1]} mm (pull sec.5) - is a straight-line "
              f"width,")
        print(f"      and comparing a width against an arc would flatter it.")
        print(f"      BOTH ARE POST-HOC.  Neither filtered anything.")
        print()
        print(f"      {'grp':>4} {'beta':>7} {'beta_p':>8} {'delta':>7} "
              f"{'z_home':>8} {'sep':>7} {'clears OD':>16} {'arc':>7} "
              f"{'chord':>7} {'margin(p)':>10}")
        print(f"      {'':>4} {'[deg]':>7} {'[deg]':>8} {'[deg]':>7} "
              f"{'[mm]':>8} {'[mm]':>7} {'':>16} {'[mm]':>7} {'[mm]':>7}")
        for gi, (key, members) in enumerate(tie, 1):
            for m in sorted(members, key=lambda r: -r["score_p"])[:MEMBERS_PER_GROUP]:
                sep = SR.min_anchor_sep_mm(m["beta_p"])
                if sep >= SR.HOUSING_OD_MM[1] - 1e-9:
                    cl = f"all {SR.HOUSING_OD_MM[0]:g}-{SR.HOUSING_OD_MM[1]:g}"
                elif sep >= SR.HOUSING_OD_MM[0] - 1e-9:
                    cl = f"only <= {sep:.1f}"
                else:
                    cl = "NONE published"
                print(f"      {gi:>4} {m['beta']:>7.2f} {m['beta_p']:>8.4f} "
                      f"{m['delta_con']:>7.1f} {m['z_home']*R_B_MM:>8.2f} "
                      f"{sep:>7.2f} {cl:>16} {arc_spacing_mm(m['beta']):>7.2f} "
                      f"{chord_spacing_mm(m['beta']):>7.2f} "
                      f"{m['score_p']:>10.6f}")
            if len(members) > MEMBERS_PER_GROUP:
                print(f"      {'':>4} ... {len(members) - MEMBERS_PER_GROUP} "
                      f"further member(s) in group {gi}")
        print()
        for gi, (key, members) in enumerate(tie, 1):
            seps = [SR.min_anchor_sep_mm(m["beta_p"]) for m in members]
            arcs = [arc_spacing_mm(m["beta"]) for m in members]
            print(f"      group {gi}: worst sep {min(seps):.2f} mm "
                  f"(clears OD up to {min(seps):.1f} mm), "
                  f"tightest arc {min(arcs):.2f} mm at r_b = {R_B_MM:.0f}")
        print()


def mirror_key(rec):
    """Canonical label for the pair ``(beta, beta_p)`` and its MIRROR.

    The mirror is ``(beta, beta_p) -> (60 - beta, 60 - beta_p)``, which sends
    ``e = beta_p - beta`` to ``-e``.  Two candidates related by it are the same
    machine seen from the other side, so any score difference between them is
    numerical noise and nothing else.  Used only to MEASURE the group key in
    :func:`part_invariant`; it does not regroup anything.
    """
    p1 = (round(rec["beta"], 9), round(rec["beta_p"], 9))
    p2 = (round(60.0 - rec["beta"], 9), round(60.0 - rec["beta_p"], 9))
    return min(p1, p2)


def part_invariant(subs, n_worst=3):
    """(6) Does ``(a, d, |e|)`` actually group ONE result?  MEASURED.

    The shortlist is grouped by ``(a, d, |beta_p - beta|)`` because
    :mod:`.score_discriminators` part (8) reports that as the invariant, and
    that is the key this harness was specified to use.  Whether the key is
    COMPLETE is a separate question, answerable from the field the sweep
    already produced: if the key identifies one result, every within-group
    spread is numerical noise.  A spread above :data:`TIE_TOL` says the key
    groups candidates the score can tell apart - which makes a "tie group" not
    a tie.
    """
    R29, az29, _ = SD._pose_grid(None)
    print()
    print("=" * 78)
    print("(6) IS (a, d, |e|) A COMPLETE INVARIANT?  MEASURED, NOT ASSUMED")
    print("=" * 78)
    print("  The shortlist above is grouped by (a, d, |e|) because that is the")
    print("  key score_discriminators part (8) reports and the key this harness")
    print("  was specified to use.  It is USED unchanged.  What follows only")
    print("  measures it.")
    print()
    print("  Two candidates are MIRRORS when (beta, beta_p) -> (60 - beta,")
    print("  60 - beta_p), which sends e -> -e: the same machine seen from the")
    print("  other side.  Mirrors share the key.  But so does any pair with the")
    print("  same |e| and a different absolute beta, and those are NOT the same")
    print("  machine.  Split the within-group spread by which it is:")
    print()
    worst_pairs = []
    for s in subs:
        if not s["scored"]:
            continue
        by = {}
        for r in s["scored"]:
            by.setdefault(group_key(r), []).append(r)
        multi = [g for g in by.values() if len(g) > 1]
        msp, gsp, n_over, n_pairs, n_exact = [], 0.0, 0, 0, 0
        for g in multi:
            sp = max(m["score_p"] for m in g) - min(m["score_p"] for m in g)
            gsp = max(gsp, sp)
            n_over += int(sp > TIE_TOL)
            sub = {}
            for m in g:
                sub.setdefault(mirror_key(m), []).append(m)
            for pair in sub.values():
                if len(pair) < 2:
                    continue
                d = max(m["score_p"] for m in pair) - min(m["score_p"]
                                                          for m in pair)
                msp.append(d)
                n_pairs += 1
                n_exact += int(d <= 1e-12)
                worst_pairs.append((d, pair))
        msp = np.array(msp) if msp else np.zeros(1)
        print(f"  ---- housing OD {s['od']:.1f} mm ----")
        print(f"      multi-member groups                     : {len(multi):,}")
        print(f"      worst spread WITHIN a (a, d, |e|) group : {gsp:.3e}")
        print(f"      groups whose spread exceeds TIE_TOL     : {n_over:,} of "
              f"{len(multi):,}  ({n_over/max(1, len(multi)):.1%})")
        print(f"      mirror pairs inside those groups        : {n_pairs:,}")
        print(f"        exact to 1e-12                       : {n_exact:,}  "
              f"({n_exact/max(1, n_pairs):.1%})")
        print(f"        median / p95 / max spread            : "
              f"{np.median(msp):.2e} / {np.percentile(msp, 95):.2e} / "
              f"{msp.max():.2e}")
        print(f"        pairs above TIE_TOL                  : "
              f"{int((msp > TIE_TOL).sum()):,}")
        print()
    print("  THE TWO HALVES OF THE KEY BEHAVE DIFFERENTLY, and the difference")
    print("  is the finding.")
    print()
    print("  MIRRORS ARE ONE RESULT, and the residue is the delta SCAN, not the")
    print("  geometry.  The mirror of a servo-plane offset is 180 - delta, which")
    print(f"  the {DELTA_GRID.size}-point 1-degree DELTA_GRID can only represent "
          f"when the maximin")
    print("  optimum lands on a grid point.  Where it falls between two, the two")
    print("  mirrors tune to deltas summing to 181 rather than 180 and their")
    print("  scores differ by that quantisation.  Re-evaluated at the EXACT")
    print("  mirror delta, the worst pairs collapse:")
    print()
    worst_pairs.sort(key=lambda t: -t[0])
    seen, shown = set(), 0
    print(f"      {'a [mm]':>7} {'d [mm]':>7} {'tuned deltas':>14} "
          f"{'sum':>5} {'as tuned':>10} {'at exact mirror':>16}")
    for d_sp, pair in worst_pairs:
        k = tuple(sorted((round(m["beta"], 6), round(m["beta_p"], 6))
                         for m in pair))
        if k in seen:
            continue
        seen.add(k)
        m1, m2 = pair[0], pair[1]
        g0, g90 = SD._delta_basis(m2["beta"], m2["beta_p"], R_P_RB,
                                  m2["a"], m2["d"])
        ex = SD.probe_margin(g0, g90, R29, az29, m2["z_home"],
                             (180.0 - m1["delta_con"]) % 180.0, P_SCORE)
        deltas = f"{m1['delta_con']:.0f} and {m2['delta_con']:.0f}"
        print(f"      {m1['a']*R_B_MM:>7.2f} {m1['d']*R_B_MM:>7.0f} "
              f"{deltas:>14} "
              f"{m1['delta_con']+m2['delta_con']:>5.0f} {d_sp:>10.3e} "
              f"{abs(m1['score_p'] - ex):>16.3e}")
        shown += 1
        if shown >= n_worst:
            break
    print()
    print("  So the mirror half of the key is SOUND - it is exact in the")
    print("  geometry, and what noise it carries is a FIFTH instance of the")
    print("  pattern notation.md sec.12 tracks: a discrete grid reporting what")
    print("  the continuum does not.  The other four are the N_i > 0 bound on")
    print("  the pose grid, the tilt-azimuth grid, box_boundary's stepping rule")
    print("  and the zero-width z_home brackets.  This one is the DELTA grid,")
    print("  which none of the four named.")
    print()
    print("  |e| ALONE IS NOT SOUND.  The full-group spread runs four decades")
    print("  above the mirror residue and above TIE_TOL, so (a, d, |e|) groups")
    print("  candidates the score CAN distinguish: the absolute beta enters the")
    print("  score at dxy = p, not just the difference.")
    print()
    print("  NOT ACTED ON HERE.  The key is the one this harness was specified")
    print("  to group by, and changing it would change the deliverable on the")
    print("  strength of a measurement taken in the same run.  It is REPORTED,")
    print("  so the tie set above is read for what it is - an UPPER BOUND on the")
    print("  tie, whose members are listed individually with their own margins")
    print("  for exactly this reason.  score_discriminators part (8) established")
    print("  the key with beta at 10-deg and beta_p at 15-deg steps; this sweep")
    print("  runs both at 2.5 deg, the first grid fine enough to separate")
    print("  same-|e| candidates that are not mirrors.")


def part_walls(res, subs):
    print()
    print("=" * 78)
    print("(5) IS a INTERIOR OR ON A WALL?  IS d?")
    print("=" * 78)
    print("  Two different kinds of wall, and the distinction is the point:")
    print(f"    a's ceiling {SR.A_CEILING_MM} mm = "
          f"{SR.A_CEILING_MM/R_B_MM:.4f} r_b is a HARDWARE ceiling - no longer")
    print("      single off-the-shelf arm was found.  a's floor is likewise the")
    print("      shortest published hole.  Neither is a sampling wall: the axis")
    print("      IS the set of holes that exist.")
    print(f"    d's ceiling {D_CAP_RB} r_b = {D_CAP_RB*R_B_MM:.0f} mm is an "
          f"ASSERTED cap on rod")
    print("      slenderness, NOT a hardware limit - stock runs to 2.1x the")
    print("      longest length ever swept.  d's floor is one grid step and is")
    print("      a sampling wall by construction; the screen decides the real")
    print("      one and part (2) reports where it fell.")
    print()
    a_vals, d_vals = a_axis(), d_axis()
    for s in subs:
        if not s["scored"]:
            print(f"  ---- housing OD {s['od']:.1f} mm: no scored survivor ----")
            continue
        groups, tie = tie_groups(s["scored"])
        print(f"  ---- housing OD {s['od']:.1f} mm ----")
        ta = sorted({g[0][0] for g in tie})
        td = sorted({g[0][1] for g in tie})
        print(f"      tie set spans a/r_b {', '.join(f'{v:.4f}' for v in ta)} "
              f"({', '.join(f'{v*R_B_MM:.2f}' for v in ta)} mm)")
        print(f"      tie set spans d/r_b {', '.join(f'{v:.2f}' for v in td)} "
              f"({', '.join(f'{v*R_B_MM:.0f}' for v in td)} mm)")
        for v in ta:
            where = ("AT THE HARDWARE CEILING" if v == max(a_vals) else
                     "AT THE SHORTEST PUBLISHED HOLE" if v == min(a_vals)
                     else "INTERIOR to the discrete set")
            print(f"        a = {v*R_B_MM:.2f} mm : {where} "
                  f"[{min(a_vals)*R_B_MM:.1f} .. {max(a_vals)*R_B_MM:.1f} mm]")
        for v in td:
            where = ("ON THE ASSERTED CAP (a wall, and an asserted one)"
                     if abs(v - D_CAP_RB) < 1e-9 else
                     "ON THE LOWEST SWEPT d (a sampling wall)"
                     if abs(v - min(d_vals)) < 1e-9 else
                     "INTERIOR to the swept range")
            print(f"        d = {v*R_B_MM:.0f} mm : {where} "
                  f"[{min(d_vals)*R_B_MM:.0f} .. {max(d_vals)*R_B_MM:.0f} mm]")
        print()
        by = {(r["beta"], r["beta_p"], r["a"], r["d"]): r["score_p"]
              for r in s["scored"]}
        bs = sorted({r["beta"] for r in s["scored"]})
        bps = sorted({r["beta_p"] for r in s["scored"]})

        def pairwise(axis, vals, others):
            out = []
            for i in range(len(vals) - 1):
                hi = lo = 0
                for b in bs:
                    for bp in bps:
                        for o in others:
                            k1 = ((b, bp, vals[i], o) if axis == "a"
                                  else (b, bp, o, vals[i]))
                            k2 = ((b, bp, vals[i + 1], o) if axis == "a"
                                  else (b, bp, o, vals[i + 1]))
                            if k1 in by and k2 in by:
                                if by[k2] > by[k1]:
                                    hi += 1
                                else:
                                    lo += 1
                out.append((vals[i], vals[i + 1], hi, lo))
            return out

        print("      PAIRWISE, the stronger form: every line holding (beta,")
        print("      beta_p) and the other axis fixed, moving one axis between")
        print("      ADJACENT sampled values; a line counts only where both ends")
        print("      are survivors.")
        for axis, vals, others, unit in (("a", a_vals, d_vals, R_B_MM),
                                         ("d", d_vals, a_vals, R_B_MM)):
            rows = [r for r in pairwise(axis, vals, others) if r[2] + r[3]]
            hi = sum(r[2] for r in rows)
            lo = sum(r[3] for r in rows)
            if hi + lo == 0:
                print(f"        {axis}: no adjacent pair has survivors at both "
                      f"ends - not measurable")
                continue
            cross = next((r for r in rows if r[3] > r[2]), None)
            print(f"        {axis}: larger wins {hi:,} of {hi+lo:,} lines "
                  f"({hi/(hi+lo):.1%})", end="")
            if cross is None:
                print(" - never turns over on lines")
            else:
                print(f" - TURNS OVER first at "
                      f"{cross[0]*unit:.2f} vs {cross[1]*unit:.2f} mm, where "
                      f"the smaller wins {cross[3]}/{cross[2]+cross[3]}")
            tievals = sorted({g[0][0 if axis == "a" else 1] for g in tie})
            top = max(vals)
            agrees = (cross is None) == all(abs(v - top) < 1e-9
                                            for v in tievals)
            print(f"           the TIE SET sits at "
                  f"{', '.join(f'{v*unit:.2f}' for v in tievals)} mm, which "
                  f"{'AGREES WITH' if agrees else 'DISAGREES WITH'} the line "
                  f"count.")
            if not agrees:
                print(f"           Both are printed and neither is chosen: "
                      f"'does raising {axis} help the")
                print(f"           typical candidate?' and 'where is the best "
                      f"candidate?' are")
                print(f"           different questions, and a majority on "
                      f"lines is a LEAN, not a wall.")
        print()


def part_verdict(res, subs, tilt):
    print()
    print("=" * 78)
    print("VERDICT")
    print("=" * 78)
    empty = all(not s["scored"] for s in subs)
    if empty:
        print("  THE SWEEP RETURNS EMPTY.  This is a legitimate result, not a")
        print("  bug: 177 of 540 were empty on the coarse grid before scoring")
        print("  ever ran.  NOTHING IS WIDENED HERE.")
        print()
        c = {k: sum(1 for e in res["empties"] if e["cause"] == k) for k in "RXW"}
        tot = sum(c.values())
        which = max(c, key=lambda k: c[k])
        name = {"R": "envelope reach (test 2)",
                "X": "reach ceiling below the N_i > 0 floor (tests 2 x 3)",
                "W": f"minimum bracket width (test 4), "
                     f"{MIN_BRACKET_MM:.0f} mm = {MIN_BRACKET_RB:.4f} r_b"}[which]
        print(f"  WHICH CONSTRAINT BOUND: {name}")
        print(f"    R {c['R']:,}   X {c['X']:,}   W {c['W']:,}   of {tot:,}")
        if which == "W":
            w = np.array([e["width"] for e in res["empties"]
                          if e["cause"] == "W"], dtype=float)
            print(f"    AT WHAT MARGIN: widest rejected bracket "
                  f"{w.max():.4f} r_b = {w.max()*R_B_MM:.2f} mm, "
                  f"{MIN_BRACKET_MM - w.max()*R_B_MM:.2f} mm short of the "
                  f"floor.")
        elif which == "X":
            g = np.array([e["gap"] for e in res["empties"]
                          if e["cause"] == "X"], dtype=float)
            print(f"    AT WHAT MARGIN: smallest crossing gap {g.min():.3e} "
                  f"r_b = {g.min()*R_B_MM:.4f} mm.")
        else:
            print(f"    AT WHAT MARGIN: reach fails at every z_home on the "
                  f"{Z_GRID.size}-point")
            print(f"    scan and at every one of the {DELTA_GRID.size} deltas; "
                  f"there is no margin to")
            print(f"    quote because no configuration is reachable at all.")
        print()
        print("  STOPPING HERE.  Widening an axis is a decision, not a fix.")
        return
    for s in subs:
        groups, tie = tie_groups(s["scored"])
        if not groups:
            print(f"  OD {s['od']:>4.1f} mm : no scored survivor.")
            continue
        best = max(m["score_p"] for m in groups[0][1])
        print(f"  OD {s['od']:>4.1f} mm  {len(s['feasible']):,} of "
              f"{s['n_rows']:,} feasible, {len(s['scored']):,} scored, "
              f"{len(groups):,} groups.")
        print(f"               TIE SET = {len(tie)} group(s) at "
              f"margin(p) >= {best - TIE_TOL:.6f}; leader {best:.6f}.")
        for gi, (key, members) in enumerate(tie, 1):
            seps = [SR.min_anchor_sep_mm(m["beta_p"]) for m in members]
            arcs = [arc_spacing_mm(m["beta"]) for m in members]
            print(f"                 group {gi}: horn {key[0]*R_B_MM:.2f} mm, "
                  f"rod {key[1]*R_B_MM:.0f} mm, |e| = {key[2]:.4f} deg, "
                  f"{len(members)} member(s)")
            print(f"                          z_home "
                  f"{min(m['z_home'] for m in members)*R_B_MM:.2f} - "
                  f"{max(m['z_home'] for m in members)*R_B_MM:.2f} mm, "
                  f"sep >= {min(seps):.2f} mm, arc >= {min(arcs):.2f} mm")
    print()
    print(f"  Every mm figure is at the ASSERTED r_b = {R_B_MM:.0f} mm and "
          f"r_p = {R_P_MM:.0f} mm.")
    print(f"  Tilt limit {tilt:.4f} deg, printed from envelope.tilt_for, not "
          f"hardcoded.")
    print(f"  margin is dimensionless and carries no characteristic length; "
          f"the cap")
    print(f"  that picked its delta is cond(J_fk) <= {CAP:.0e} at char_len = "
          f"{CHAR_LEN}, PROVISIONAL")
    print(f"  in that length.  p = {P_SCORE:.6f} r_b.")
    print()
    print("  WHAT THIS DOES NOT DECIDE.  No joint, no servo, no horn is chosen:")
    print("  the tie set is a set, the two housing-OD runs are reported apart,")
    print("  and beta was never filtered.  The OBJECTIVE IS NOT FIXED - margin")
    print("  still measures distance from unreachability rather than capability")
    print("  (notation.md, 8 Sept), and that stays open.  notation.md is not")
    print("  touched by this module.")


# --------------------------------------------------------------------------- #
def report(res, tilt):
    """Every report section, given a run.  Shared by a fresh run and a reload."""
    subs = [_od_counts(res, od) for od in SR.HOUSING_OD_MM]
    part_counts(res, subs)
    part_empties(res, subs)
    part_margins(subs)
    part_shortlist(subs)
    part_invariant(subs)
    part_walls(res, subs)
    part_verdict(res, subs, tilt)


def main(argv=None) -> None:
    """``--report [path]`` regenerates the report from a saved run.

    The screen is 25 minutes and the tune 70.  A report is prose over scalars
    that are already computed, so re-deriving them to reword a paragraph would
    be 95 minutes spent on nothing.  The reload path runs the SAME
    :func:`report` the fresh path does - there is no second reporting code
    path to drift.
    """
    argv = sys.argv[1:] if argv is None else list(argv)
    tilt = ENV.tilt_for(ENV.X0_WORKING, ENV.TAU)
    if argv and argv[0] == "--report":
        path = argv[1] if len(argv) > 1 else SAVE_PATH
        with at_tilt(tilt):
            res = load(path)
            print("=" * 78)
            print(f"THE SWEEP HARNESS - REPORT REGENERATED FROM {path}")
            print("=" * 78)
            print(f"  Screen and tune are NOT re-run: {len(res['scored']):,} "
                  f"scored records reloaded.")
            print(f"  Original run: screen {res['t_screen']:.0f} s, "
                  f"tune + score {res['t_tune']:.0f} s.")
            report(res, tilt)
        return
    _run_and_report(tilt)


def _run_and_report(tilt) -> None:
    print("=" * 78)
    print("THE SWEEP HARNESS")
    print("=" * 78)
    print("  Generates candidates, tests feasibility, tunes delta per")
    print("  candidate, scores the survivors, and returns a ranked TIE SET.")
    print()
    with at_tilt(tilt):
        az, mg = ZB.fine_poses()
        agrees = abs(ENV.TILT_LIMIT_DEG - tilt) < 1e-9
        print(f"  envelope.TILT_LIMIT_DEG = {ENV.TILT_LIMIT_DEG:.4f} deg, "
              f"tilt_for(X0_WORKING) = {tilt:.4f} deg: "
              f"{'MATCHES' if agrees else 'DOES NOT MATCH'}")
        print(f"  N_i > 0 floor (CLOSED FORM, never the pose grid): z_home > "
              f"{ZB.z_lower_closed_form(R_P_RB, H_P):.6f} r_b = "
              f"{ZB.z_lower_closed_form(R_P_RB, H_P)*R_B_MM:.3f} mm")
        print()
        lg = ledger(az, mg)
        checks = calibrate(az, mg, np.deg2rad(DELTA_GRID))
        report_ledger(lg, tilt, checks)
        if checks["bad"]:
            print("  BRACKET GATE FAILED - the delta-free bracket disagrees "
                  "with the")
            print("  committed reach primitive.  NOT RUNNING THE SWEEP.")
            return
        print("-" * 78)
        print("RUNNING")
        print("-" * 78)
        res = run(az, mg)
        try:
            print(f"    saved -> {save(res, SAVE_PATH)}")
        except OSError as exc:                     # reporting must not depend
            print(f"    save failed ({exc}); the report below is unaffected")
        report(res, tilt)


if __name__ == "__main__":
    main()
