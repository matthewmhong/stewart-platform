"""Is the optimum interior to the sampled box, or pinned to its corner?

    python -m stewart.diagnostics.box_boundary

Every top-5 candidate at every tilt limit in :mod:`.tilt_bracket` is
``r_p/r_b = 0.60``, ``a/r_b = 0.35``, ``d/r_b = 0.80`` - the MAXIMUM of the
sampled ``a``, the MINIMUM of the sampled ``r_p``, and the minimum of the
sampled ``d``.  Three of the five axes are at a wall.  Nothing empties at
``a/r_b = 0.35`` at any tilt limit, so that wall is not a constraint the
problem imposes; it is where :mod:`.zhome_bracket`'s grid stops.

An optimum at a grid wall is not an optimum.  It is a statement that the search
was not allowed to go further, and it stays that until the box is opened.  This
module opens it, one axis at a time.

WHAT IS NOT DECIDED HERE.  This module does not choose ranges, does not spec
the sweep harness, and does not touch ``notation.md``.  The extensions below
are DIAGNOSTIC PROBES, run to find out where the optimum actually sits; the
grid that a sweep harness should eventually run is a separate decision and this
module does not make it.  The coarse grid is held exactly as
:mod:`.zhome_bracket` builds it in every axis but the one under test, including
``c_p / r_b = 0.1``, and the tilt limit is the current 10.529 degrees
throughout - :mod:`.tilt_bracket` already measured the tilt sensitivity and
this module does not repeat it.

THE FOUR EXTENSIONS, each 0.05 per step, each from the default grid::

    a/r_b    UP    from 0.35   0.40 0.45 ... 0.85       watch max(a)
    r_p/r_b  DOWN  from 0.60   0.55 0.50 ... 0.10       watch min(r_p)
    d/r_b    DOWN  from 0.80   0.75 0.70 ... 0.30       watch min(d)
    d/r_b    UP    from 0.80   0.85 0.90 ... 1.30       watch d = 0.80

The stopping rule asked for is "until the top-5 stops containing the largest
(smallest) sampled value, or 10 steps".  **All ten steps are run regardless**
and the step at which the rule WOULD have fired is reported.  Two reasons: a
rule that fires at step 3 leaves steps 4-10 unmeasured, and "did it come back"
is exactly the question a boundary probe is for; and the rule is ambiguous in
the ``d`` UP direction, where the largest sampled ``d`` is the pre-existing
1.60 and can never be in the top 5, so the watched value there is stated
explicitly per extension rather than inferred.  Running everything and naming
the watched value removes both problems and costs nothing that matters.

EFFICIENCY.  Each step adds one axis value, so it adds exactly one slice of new
candidates; only the slice is screened and tuned, and the box is accumulated.
The measured numbers are identical to re-running the whole box each step - the
screen and the tune are per-candidate and carry no cross-candidate state - and
the work is linear in the number of candidates rather than quadratic in steps.

PHYSICAL REACH.  The hardware pull dispatched 2026-09-04 is STILL UNREAD, and
it carries the three numbers that would bound these axes: horn lengths (the
discrete set ``a`` lives in), ball-joint housing OD (which floors the anchor
spacing and so ``r_p``), and rod stock (``d``).  There is therefore no basis
for a limit, so every axis is extended the full ten steps and the buildability
quantities are REPORTED per step instead:

  * ``min |p_i - p_j|``, the closest approach of two platform anchors, which is
    what a ball-joint housing OD floors.  Reported in ``r_b`` and in mm at
    ``r_b = 100 mm`` - the fixture scale ``docs/cc-fk-gate.md`` sec.2.1 already
    uses, a UNITS PLACEHOLDER and NOT a chosen or candidate ``r_b``.  It is
    ABOVE the ``r_b <= 90 mm`` bed ceiling decided 2026-09-08 (``notation.md``
    sec.12); see :data:`R_B_REFERENCE_MM`, which carries why that costs nothing
    and why nothing is rescaled.
  * ``d / a``, and a flag at ``d <= a``.  :func:`~stewart.geometry.make_geometry`
    documents that corner explicitly: ``d > a`` is not a validity condition and
    is not checked, but ball-joint angular travel is the real constraint there
    and it belongs to the component - another unread number.
  * ``a`` against the base anchor spacing, since an arm of radius ``a`` sweeps a
    circle about its own base anchor.

None of these is a threshold.  They are the quantities to check the pull
against when it lands, and the step at which each stops being obviously
buildable is flagged as UNVERIFIABLE, not as a stop.

UNITS AND LABELS.  ``margin`` is a dimensionless ratio and carries NO
characteristic length and NO Jacobian.  The constraint on the inner ``delta``
tune is ``cond(J_fk) <= 1e6`` at ``char_len = r_b``, throughout, PROVISIONAL as
that cap is.  ``z_home`` and every length ratio are in ``r_b``.

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
from .score_discriminators import TIE_TOL, _abs_e_groups
from .tilt_bracket import CAP, _key, constrained_margins, screen
from .zhome_bracket import A_RB, BETA, BETA_P, D_RB, C_P, R_B, RP_RB

#: Step size on every extended axis, in ``r_b``.  One number, used in all four
#: directions, so no axis is probed more finely than another by accident.
STEP = 0.05

#: Maximum steps per extension.  Ten, and all ten are run; see the module note
#: on the stopping rule.
MAX_STEPS = 10

#: How many candidates the "is the extreme still winning?" test looks at.
TOP_N = 5

#: Reference scale for the buildability columns ONLY, mm.  A UNITS PLACEHOLDER,
#: and NOT a candidate ``r_b`` - it never was one.  This is the fixture scale
#: ``docs/cc-fk-gate.md`` sec.2.1 runs at, quoted here for one purpose: so that
#: a separation expressed in ``r_b`` can be read as a length.
#:
#: IT IS ABOVE THE CEILING.  Absolute scale was decided 2026-09-08:
#: ``r_b <= 90 mm``, from a 180 x 180 mm print volume (``notation.md`` sec.12).
#: 100 > 90, so this number is not a value ``r_b`` can take.  That costs
#: nothing here and nothing is rescaled: the kinematics is homogeneous of
#: degree one and the envelope is purely angular, so every ratio, margin and
#: residual this module reports is unchanged by the ceiling, and only the
#: ``[mm]`` buildability column is denominated in it.  Read that column as
#: "the ``r_b = 1`` separation, printed in units of 100 mm".
R_B_REFERENCE_MM = 100.0


# --------------------------------------------------------------------------- #
# the box, accumulated one slice at a time
# --------------------------------------------------------------------------- #
class Box:
    """The candidate set of one extension, grown a slice at a time.

    Holds every candidate screened so far and the constrained-tune margin of
    every survivor.  :meth:`add_slice` screens and tunes ONLY the new
    candidates; nothing already in the box is recomputed, and nothing in the
    box depends on what else is in it.
    """

    def __init__(self):
        self.rows = []          # every candidate screened, feasible or not
        self.measured = []      # survivors carried through the tune
        self.margins = {}       # key -> constrained margin at CAP, NaN if none
        self.dropped = 0

    def add_slice(self, **axes):
        """Screen and tune one product slice; fold it into the box."""
        rows, feasible = screen(**axes)
        measured, dropped = constrained_margins(feasible)
        self.rows.extend(rows)
        self.measured.extend(measured)
        self.dropped += dropped
        for rec in measured:
            e = rec["con"][CAP]
            self.margins[_key(rec)] = (np.nan if e is None
                                       else float(e["margin"]))
        return len(rows), len(feasible)

    def copy(self):
        """A box sharing the records but with its own containers.

        The records themselves are never mutated after a slice is folded in, so
        sharing them is safe and is what lets the 540-candidate base screen be
        paid for ONCE and reused by all four extensions.
        """
        b = Box()
        b.rows = list(self.rows)
        b.measured = list(self.measured)
        b.margins = dict(self.margins)
        b.dropped = self.dropped
        return b

    def merge(self, other):
        """Fold another box in, de-duplicated by candidate key.

        The four extensions share the base grid and each adds its own ray, so
        the union is taken by key rather than by concatenation; a candidate
        screened in two extensions is the same candidate and must appear once.
        """
        seen = {_key(r) for r in self.measured}
        rowseen = {(r["beta"], r["beta_p"], r["r_p"], r["a"], r["d"])
                   for r in self.rows}
        for r in other.rows:
            k = (r["beta"], r["beta_p"], r["r_p"], r["a"], r["d"])
            if k not in rowseen:
                rowseen.add(k)
                self.rows.append(r)
        for r in other.measured:
            if _key(r) not in seen:
                seen.add(_key(r))
                self.measured.append(r)
        for k, v in other.margins.items():
            self.margins.setdefault(k, v)
        return self

    @property
    def n_feasible(self):
        return sum(1 for r in self.rows if r["feasible"])

    def top(self, n):
        """``[(margin, record)]``, best first, over survivors with a margin."""
        by_key = {_key(r): r for r in self.measured}
        good = [(v, k) for k, v in self.margins.items() if not np.isnan(v)]
        good.sort(key=lambda t: (-t[0], t[1]))
        return [(v, by_key[k]) for v, k in good[:n]]


# --------------------------------------------------------------------------- #
# buildability - reported, never thresholded
# --------------------------------------------------------------------------- #
def _min_sep(X):
    """Closest approach of two columns of a ``(3, 6)`` anchor array."""
    D = np.linalg.norm(X[:, :, None] - X[:, None, :], axis=0)
    return float(D[~np.eye(6, dtype=bool)].min())


def buildability(rec):
    """The quantities the unread hardware pull would bound, for one candidate.

    Every one is MEASURED off the library geometry, and none is compared
    against a threshold, because there is no threshold to compare against yet.
    ``beta`` and ``beta_p`` come from the record, so the separations are the
    real ones for that candidate rather than a representative value.
    """
    g = make_geometry(r_b=R_B, beta=rec["beta"], delta=0.0, r_p=rec["r_p"],
                      beta_p=rec["beta_p"], a=rec["a"], d=rec["d"], c_p=C_P)
    sep_p = _min_sep(g.p)
    sep_b = _min_sep(g.b)
    return dict(sep_p=sep_p, sep_b=sep_b,
                sep_p_mm=sep_p * R_B_REFERENCE_MM,
                d_over_a=rec["d"] / rec["a"],
                rod_short=(rec["d"] <= rec["a"]),
                arm_vs_sep=rec["a"] / sep_b)


# --------------------------------------------------------------------------- #
# one extension
# --------------------------------------------------------------------------- #
class Extension:
    """One axis opened in one direction, with its own watched value.

    ``watch(values, rec)`` answers "is the value this extension is watching
    still present in this record?", and ``label`` says in words what that value
    is.  Making it explicit is what keeps the ``d`` UP direction honest: the
    largest sampled ``d`` there is the pre-existing 1.60 and could never be in
    the top 5, so the watched value is the incumbent 0.80 instead.
    """

    def __init__(self, axis, direction, base, start, label, watch):
        self.axis = axis                  # "a", "r_p" or "d"
        self.direction = direction        # +1 or -1
        self.base = list(base)
        #: The value stepped away from, stated EXPLICITLY rather than taken as
        #: max/min of the base axis.  The two differ for ``d`` UP: the question
        #: is asked about ``d/r_b = 0.80``, the incumbent optimum, which is the
        #: MINIMUM of the sampled ``d`` - so an up-step from ``max(D_RB)``
        #: would probe 1.65, 1.70, ... and never go near what was asked.
        self.start = start
        self.label = label
        self.watch = watch

    @property
    def name(self):
        return f"{self.axis}/r_b {'UP' if self.direction > 0 else 'DOWN'}"

    def value_at(self, step):
        return round(self.start + self.direction * STEP * step, 9)

    def values_through(self, step):
        vals = self.base + [self.value_at(k) for k in range(1, step + 1)]
        return sorted(set(vals))

    def slice_axes(self, step):
        """The axis lists selecting ONLY the candidates new at this step."""
        axes = dict(beta_vals=BETA, beta_p_vals=BETA_P, rp_vals=RP_RB,
                    a_vals=A_RB, d_vals=D_RB)
        axes[{"a": "a_vals", "r_p": "rp_vals", "d": "d_vals"}[self.axis]] = \
            [self.value_at(step)]
        return axes


def extensions():
    """The four extensions, in the order the question asks for them."""
    return [
        Extension("a", +1, A_RB, max(A_RB), "the largest sampled a/r_b",
                  lambda vals, rec: rec["a"] == max(vals)),
        Extension("r_p", -1, RP_RB, min(RP_RB), "the smallest sampled r_p/r_b",
                  lambda vals, rec: rec["r_p"] == min(vals)),
        Extension("d", -1, D_RB, min(D_RB), "the smallest sampled d/r_b",
                  lambda vals, rec: rec["d"] == min(vals)),
        # UP from 0.80 - the incumbent, which is min(D_RB), NOT max(D_RB).  The
        # values it adds (0.85 ... 1.30) are INTERIOR to the existing
        # [0.80, 1.60] range, so this direction opens no wall; it refines the
        # grid above the incumbent.  The "largest sampled value" reading of the
        # stopping rule is vacuous here - 1.60 is the largest and is nowhere
        # near the top - so what is watched is whether the incumbent 0.80
        # survives a finer grid above it, which is the live question.
        Extension("d", +1, D_RB, 0.80, "the incumbent d/r_b = 0.80",
                  lambda vals, rec: rec["d"] == 0.80),
    ]


def run_extension(ext, base):
    """Run one extension for all :data:`MAX_STEPS` steps.

    ``base`` is the already-screened 540-candidate coarse grid, copied rather
    than re-screened.  Returns ``(per-step rows, box, seconds)``.
    """
    box = base.copy()
    t0 = time.time()
    out = []
    for step in range(0, MAX_STEPS + 1):
        # A step can land on a value the base axis already carries - d UP does,
        # at step 8, where 0.80 + 8 * 0.05 = 1.20.  Screening that slice again
        # would put the same candidate in the box twice and inflate every count
        # that follows, so the step is recorded and the slice is not re-added.
        if step > 0 and ext.value_at(step) not in ext.base:
            box.add_slice(**ext.slice_axes(step))
        vals = ext.values_through(step)
        top = box.top(TOP_N)
        held = any(ext.watch(vals, rec) for _, rec in top)
        out.append(dict(step=step, value=(None if step == 0
                                          else ext.value_at(step)),
                        n_cand=len(box.rows), n_feas=box.n_feasible,
                        top=top, held=held, vals=vals))
    return out, box, time.time() - t0


# --------------------------------------------------------------------------- #
def _fire_step(rows):
    """First step at which the stopping rule would have fired, or ``None``."""
    for r in rows:
        if r["step"] > 0 and not r["held"]:
            return r["step"]
    return None


def _verdict(ext, rows):
    """Interior / still on the boundary / came off, in words, with the step."""
    fired = _fire_step(rows)
    best = rows[-1]["top"][0][1] if rows[-1]["top"] else None
    at = f"{ext.axis}/r_b reaching {ext.value_at(MAX_STEPS):.2f}"
    if fired is None:
        return (f"STILL ON THE BOUNDARY - {ext.label} is still in the top "
                f"{TOP_N} after all {MAX_STEPS} steps, with {at}")
    after = [r for r in rows if r["step"] > fired and r["held"]]
    where = (f"; best at step {MAX_STEPS} is {ext.axis}/r_b = "
             f"{best[ext.axis]:.2f}" if best is not None else "")
    if after:
        return (f"CAME OFF at step {fired} ({ext.axis}/r_b = "
                f"{rows[fired]['value']:.2f}) but RETURNED at step(s) "
                f"{', '.join(str(r['step']) for r in after)} - not a clean "
                f"interior optimum{where}")
    return (f"INTERIOR - {ext.label} left the top {TOP_N} at step {fired} "
            f"({ext.axis}/r_b = {rows[fired]['value']:.2f}) and stayed "
            f"out{where}")


def _print_extension(ext, rows, secs):
    print()
    print("=" * 78)
    print(f"EXTENSION: {ext.name} from {ext.start:.2f}, step {STEP}, "
          f"{MAX_STEPS} steps")
    print("=" * 78)
    print(f"  watched value : {ext.label}")
    print(f"  base axis     : {ext.base}")
    print(f"  every other axis is zhome_bracket's coarse grid, c_p/r_b = {C_P}")
    print(f"  margin is under the constrained tune, cond(J_fk) <= {CAP:.0e} at")
    print(f"  char_len = {SD.CONSTRAINT_CHAR_LEN}; margin itself carries no "
          f"char_len.")
    print()
    print(f"    {'step':>4} {ext.axis + '/r_b':>8} {'cands':>6} {'feas':>6} "
          f"{'top-5 margins (r_p, a, d)':<46} {'watched':>8}")
    for r in rows:
        val = "base" if r["value"] is None else f"{r['value']:.2f}"
        cells = " ".join(f"{m:.4f}" for m, _ in r["top"])
        print(f"    {r['step']:>4} {val:>8} {r['n_cand']:>6} {r['n_feas']:>6} "
              f"{cells:<46} {'YES' if r['held'] else 'no':>8}")
    print()
    print(f"    {'step':>4} {'rank':>4} {'beta':>6} {'beta_p':>7} {'r_p':>6} "
          f"{'a/r_b':>6} {'d/r_b':>6} {'z_home':>8} {'delta':>7} "
          f"{'margin':>10}")
    for r in rows:
        if r["step"] not in (0, 1, MAX_STEPS // 2, MAX_STEPS):
            continue
        for rank, (m, rec) in enumerate(r["top"], 1):
            print(f"    {r['step']:>4} {rank:>4} {rec['beta']:>6.1f} "
                  f"{rec['beta_p']:>7.1f} {rec['r_p']:>6.2f} {rec['a']:>6.2f} "
                  f"{rec['d']:>6.2f} {rec['z_home']:>8.4f} "
                  f"{rec['con'][CAP]['delta']:>7.1f} {m:>10.6f}")
    print()
    fired = _fire_step(rows)
    print(f"  stopping rule : " + ("would have fired at step "
                                   f"{fired} ({ext.axis}/r_b = "
                                   f"{rows[fired]['value']:.2f}); all "
                                   f"{MAX_STEPS} steps run anyway"
                                   if fired else
                                   f"never fired in {MAX_STEPS} steps"))
    print(f"  VERDICT       : {_verdict(ext, rows)}")
    print(f"  ({secs:.1f} s)")

    # ---- buildability of the winner at each step --------------------- #
    print()
    print("  BUILDABILITY of the step's best candidate.  No thresholds: the")
    print("  horn set, the ball-joint housing OD and the rod stock are all on")
    print("  the hardware pull dispatched 2026-09-04 and STILL UNREAD.  These")
    print("  are the quantities to check it against when it lands.")
    print(f"    {'step':>4} {ext.axis + '/r_b':>8} {'min|p_i-p_j|':>13} "
          f"{'[mm @ r_b=100]':>15} {'d/a':>7} {'a/min|b_i-b_j|':>15} "
          f"{'flag':>12}")
    for r in rows:
        if not r["top"]:
            continue
        rec = r["top"][0][1]
        bd = buildability(rec)
        flags = []
        if bd["rod_short"]:
            flags.append("d<=a")
        if bd["arm_vs_sep"] > 0.5:
            flags.append("arm>sep/2")
        val = "base" if r["value"] is None else f"{r['value']:.2f}"
        print(f"    {r['step']:>4} {val:>8} {bd['sep_p']:>13.4f} "
              f"{bd['sep_p_mm']:>15.2f} {bd['d_over_a']:>7.2f} "
              f"{bd['arm_vs_sep']:>15.3f} "
              f"{(','.join(flags) if flags else '-'):>12}")
    print("    d<=a      : the corner make_geometry documents as NOT a validity")
    print("                condition and NOT checked - ball-joint angular travel")
    print("                is the real constraint there, and it is unread.")
    print("    arm>sep/2 : the arm sweep radius exceeds half the base anchor")
    print("                spacing, so two adjacent arm circles overlap in plan.")
    print("                Whether they collide depends on the servo planes and")
    print("                on horn thickness - geometry, not this screen.")


# --------------------------------------------------------------------------- #
# the grouped top 20 on the landed box
# --------------------------------------------------------------------------- #
def report_groups(box, title, n=20):
    """Top ``n`` by constrained margin, GROUPED by ``(r_p, a, d, |e|)``.

    Part (8)'s key, taken from :func:`.score_discriminators._abs_e_groups`
    rather than restated.  Members of one group have the same margin to
    ``TIE_TOL`` - they differ by a rotation of the whole machine about ``z``, by
    a sign flip of ``e = beta_p - beta``, or by both - so printing them as
    adjacent RANKS says they are first and second when they are one result.
    Groups are ranked; members inside a group are not.
    """
    print()
    print("=" * 78)
    print(f"TOP {n} GROUPED BY (r_p, a, d, |e|),  e = beta_p - beta")
    print(f"  {title}")
    print("=" * 78)
    print("  Part (8)'s invariant.  Two candidates sharing this key have the")
    print("  same margin at every one of the 180 deltas, not merely at the")
    print("  tuned one, so they are ONE result reached by two (beta, beta_p)")
    print("  pairs - a rotation of the machine about z, a sign flip of e, or")
    print("  both.  They are ranked as one group here and the members are")
    print("  listed unranked inside it.")
    print()
    good = [(v, k) for k, v in box.margins.items() if not np.isnan(v)]
    good.sort(key=lambda t: (-t[0], t[1]))
    by_key = {_key(r): r for r in box.measured}
    groups = _abs_e_groups([by_key[k] for _, k in good])

    seen, ordered = set(), []
    for _, k in good:
        rec = by_key[k]
        gk = (rec["r_p"], rec["a"], rec["d"],
              round(abs(rec["beta_p"] - rec["beta"]), 9))
        if gk not in seen:
            seen.add(gk)
            ordered.append(gk)

    if not ordered:
        print("  no candidate has a margin under the cap; nothing to group.")
        return
    shown, gi, biggest = 0, 0, 1
    print(f"    {'grp':>4} {'r_p':>6} {'a/r_b':>6} {'d/r_b':>6} {'|e|':>6} "
          f"{'memb':>5} {'margin':>10} {'spread':>9} {'sep_p':>7} "
          f"{'[mm]':>6} {'d/a':>6}   (beta, beta_p) pairs")
    for gi, gk in enumerate(ordered, 1):
        members = groups[gk]
        ms = [box.margins[_key(r)] for r in members]
        spread = max(ms) - min(ms)
        pairs = " ".join(f"({r['beta']:.0f},{r['beta_p']:.0f})"
                         for r in sorted(members,
                                         key=lambda r: (r["beta"], r["beta_p"])))
        tie = "TIE" if spread < TIE_TOL else "SPREAD"
        bd = buildability(members[0])
        print(f"    {gi:>4} {gk[0]:>6.2f} {gk[1]:>6.2f} {gk[2]:>6.2f} "
              f"{gk[3]:>6.0f} {len(members):>5} {max(ms):>10.6f} "
              f"{spread:>9.1e} {bd['sep_p']:>7.4f} {bd['sep_p_mm']:>6.1f} "
              f"{bd['d_over_a']:>6.2f}   {pairs}  [{tie}]")
        shown += len(members)
        biggest = max(biggest, len(members))
        if shown >= n:
            break
    print()
    print(f"  {shown} candidates shown in {gi} groups.  'spread' is the worst")
    print(f"  within-group margin difference; below {TIE_TOL:.0e} the members are")
    print("  the same result, and the group is ONE line of the ranking rather")
    print(f"  than up to {biggest} adjacent ones.")
    print()
    print("  sep_p is min |p_i - p_j| in r_b for the group's first member, and")
    print(f"  [mm] is that at the r_b = {R_B_REFERENCE_MM:.0f} mm fixture scale of")
    print("  docs/cc-fk-gate.md sec.2.1 - a UNITS PLACEHOLDER, NOT a chosen r_b")
    print("  and NOT a candidate one: absolute scale was decided 2026-09-08 at")
    print("  r_b <= 90 mm from the bed (notation.md sec.12), so 100 is above the")
    print("  ceiling.  Nothing here is rescaled - every ratio and margin below is")
    print("  unchanged by it, and only this column is denominated in it.")
    print("  It is carried here because this ranking is by margin alone, and a")
    print("  margin ranking has no way to know whether six ball joints fit on")
    print("  the ring it just won on.  The ball-joint housing OD that would")
    print("  settle that is on the unread hardware pull.  READ THIS COLUMN")
    print("  BEFORE READING THE RANK.")


# --------------------------------------------------------------------------- #
def main() -> None:
    print("=" * 78)
    print("IS THE OPTIMUM INTERIOR, OR PINNED TO THE CORNER OF THE BOX?")
    print("=" * 78)
    print("  Every top-5 candidate at every tilt limit measured in tilt_bracket")
    print("  is r_p/r_b = 0.60, a/r_b = 0.35, d/r_b = 0.80 - the MAXIMUM sampled")
    print("  a, the MINIMUM sampled r_p, the MINIMUM sampled d.  Nothing empties")
    print("  at a/r_b = 0.35 at any tilt limit, so that wall is not a constraint")
    print("  the problem imposes; it is where the grid stops.  An optimum at a")
    print("  grid wall is a statement about the grid until the box is opened.")
    print()
    print("  NOTHING IS CHOSEN HERE.  No range is chosen, the sweep harness is")
    print("  not specced, notation.md is not touched.  These are diagnostic")
    print("  probes to locate the optimum, not a proposed grid.")
    print()
    print("-" * 78)
    print("HELD FIXED")
    print("-" * 78)
    print(f"  tilt limit  : {TILT_LIMIT_DEG:.4f} deg (current; tilt sensitivity")
    print(f"                is tilt_bracket's question, not this module's)")
    print(f"  c_p/r_b     : {C_P}")
    print(f"  beta        : {BETA}")
    print(f"  beta_p      : {BETA_P}")
    print(f"  z_home      : midpoint of each candidate's bracket")
    print(f"  tune        : cond(J_fk) <= {CAP:.0e} at char_len = "
          f"{SD.CONSTRAINT_CHAR_LEN}")
    print(f"  step        : {STEP} r_b on every extended axis, {MAX_STEPS} steps")
    print()
    print("  All ten steps of every extension are run and the step at which the")
    print("  stopping rule WOULD have fired is reported.  A rule that fires at")
    print("  step 3 leaves 4-10 unmeasured, and whether the extreme comes BACK")
    print("  is exactly what a boundary probe is for.")
    print()

    print("-" * 78)
    print("BASE GRID - screened once and reused by all four extensions")
    print("-" * 78)
    base = Box()
    t0 = time.time()
    base.add_slice(beta_vals=BETA, beta_p_vals=BETA_P, rp_vals=RP_RB,
                   a_vals=A_RB, d_vals=D_RB)
    print(f"  {len(base.rows)} candidates, {base.n_feasible} feasible, "
          f"{base.dropped} dropped at the midpoint  ({time.time()-t0:.1f} s)")

    union = base.copy()
    verdicts = []
    for ext in extensions():
        rows, box, secs = run_extension(ext, base)
        _print_extension(ext, rows, secs)
        verdicts.append((ext, rows))
        union.merge(box)

    # ---- summary of the four verdicts -------------------------------- #
    print()
    print("=" * 78)
    print("WHERE THE OPTIMUM SITS, PER AXIS")
    print("=" * 78)
    print("  Each line is one axis opened alone.  'Interior' means the extreme")
    print("  stopped winning and did not come back within the ten steps; it does")
    print("  NOT mean the joint optimum over all axes at once is interior, which")
    print("  no one-axis-at-a-time probe can establish.")
    print()
    for ext, rows in verdicts:
        print(f"  {ext.name:<14} {_verdict(ext, rows)}")

    report_groups(union,
                  f"the EXPLORED SET: the {len(base.rows)}-candidate coarse "
                  f"grid plus the four extension rays, "
                  f"{len(union.rows)} candidates in all.  Not a product box - "
                  f"each ray moves ONE axis - so it is the set actually "
                  f"measured above, named as such.")

    print()
    print("=" * 78)
    print("WHAT THIS DOES NOT SETTLE")
    print("=" * 78)
    print("  One axis at a time.  Each extension holds the other four at the")
    print("  coarse grid, so an optimum that needs two axes to move together is")
    print("  invisible to all four probes.  The corner being three walls deep is")
    print("  the reason to expect exactly that.")
    print()
    print("  The buildability columns are measurements, not limits.  Horn")
    print("  lengths, ball-joint housing OD and rod stock are on the hardware")
    print("  pull dispatched 2026-09-04, still unread, and until it lands no")
    print("  step of any of these extensions can be called reachable or not.")
    print("  Nothing here should be read as saying a value is buildable.")


if __name__ == "__main__":
    main()
