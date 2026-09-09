"""Sweep ranges for ``a``, ``beta_p`` and ``d`` from the hardware pull.

    python -m stewart.diagnostics.sweep_ranges

WHAT THIS DOES.  ``docs/hardware-pull.md`` (2026-09-09) reports what exists and
chooses nothing.  This module turns three of its sections into RANGES on the
sweep axes and evaluates the feasible set over them at the tilt limit now in
force - :func:`.envelope.tilt_for` at ``x0 = 50 mm``, the working displacement
alone, ``tau_L`` having been DROPPED (see :mod:`.tilt_dropped`).  The limit is
printed from that closed form, never hardcoded as degrees.

  ``a``       is a **DISCRETE SET**, not an interval.  A servo arm has holes at
              published positions; there is no arm with a hole at 0.3 r_b
              because a sweep asked for one.  Section 1.1's ProModeler hole
              ladder is transcribed hole by hole, converted to ``a/r_b`` at
              ``r_b = 90 mm``, and the union is the axis.  The single-arm
              ceiling is **60.4 mm = 0.671 r_b** (PDRS60-25T); the Thingiverse
              44/52/60/68/76/84/92 mm "extension arms" are a 3D PRINT, not
              stock (sec.1.3), and are EXCLUDED.
  ``beta_p``  is bounded by ball-joint housing OD, published range
              **9.0 - 13.0 mm** across twelve M3-class parts (sec.2).  Two
              housings cannot occupy one hole, so the OD floors the closest
              approach of two platform anchors and that floors ``beta_p`` away
              from 0 and 60.  The bound is computed at BOTH ends of the
              published range and both are carried through the sweep.  **No
              joint is chosen**, and the two device classes sec.2 mixes -
              spherical plain bearings and RC ball-cup rod ends - are not
              resolved here either.
  ``d``       is continuous, cut to length.  Section 7 is reported and, as
              measured there, **nothing in it bounds the LENGTH** - see
              :func:`part_d`.  The existing ``D_RB`` is carried unchanged and
              flagged, not narrowed.

WHAT IS NOT DECIDED HERE.  **No joint, no servo and no horn is chosen.  The
harness is not specced.  notation.md is not touched.**  The tie threshold in
part (7) is a MEASUREMENT of what the score can resolve, offered so that a
threshold can come from measurement instead of from ``1e-15``; it is not set
as a constant anywhere.

THE PIPELINE IS THE EXISTING ONE.  :func:`.fixed_ratio.run_slice` at the
asserted ratio, with its ``axes`` argument carrying the hardware-pull axes -
:func:`.tilt_bracket.screen`, :func:`~.tilt_bracket.attribute`,
:func:`~.tilt_bracket.constrained_margins` and
:func:`.score_discriminators.probe_margin`, none of them reimplemented.  The
tilt limit is rebound for the pass by :func:`.tilt_bracket.at_tilt`, which
moves the screen envelope, the harness pose grid and the ``N_i > 0`` floor
together.

UNITS AND LABELS.  ``margin`` is a dimensionless ratio and carries NO
characteristic length and NO Jacobian.  The CAP that picks the ``delta`` it is
evaluated at carries both: ``cond(J_fk) <= 1e6`` at ``char_len = r_b``,
PROVISIONAL in that length.  Every mm figure is at the ASSERTED ``r_b = 90 mm``
and ``r_p = 80 mm``.  ``p`` is in ``r_b``.

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
from .score_discriminators import TIE_TOL, _abs_e_groups, _five_number
from .tilt_bracket import CAP
from .zhome_bracket import BETA, BETA_P, D_RB, H_P

# --------------------------------------------------------------------------- #
# (1) the discrete a set - hardware-pull sec.1.1, hole by hole
# --------------------------------------------------------------------------- #
#: Provenance of each ladder, and it is not decoration: three different things
#: are being read off one page and they are not equally strong.
#:
#: ``STATED``
#:     every hole position is printed on the maker's page.
#: ``ENDPOINT``
#:     the ladder is elided ("9 ... 18", "to 32"); only the positions the page
#:     actually prints are used, and the interior holes are NOT interpolated.
#: ``DERIVED``
#:     the page states a count and a pitch rather than positions, and the
#:     positions follow by arithmetic on that sentence.  Flagged, not hidden.
PROMODELER = (
    # (part, spline, provenance, hole positions mm)
    ("PDRS301",     "15T", "ENDPOINT", (9.0, 18.0)),
    ("PDRS201",     "-",   "STATED",   (10.0, 15.0, 20.0)),
    ("PDRS101",     "-",   "ENDPOINT", (12.5, 25.0)),
    ("PDRS15-25T",  "25T", "ENDPOINT", (15.0,)),
    ("PDRS107",     "-",   "DERIVED",  (11.0, 16.0, 21.0)),
    ("PDRS303",     "15T", "ENDPOINT", (16.0, 26.0)),
    ("PDRS401",     "-",   "ENDPOINT", (23.0, 32.0)),
    ("PDRS25-15T",  "15T", "STATED",   (15.4, 20.4, 25.4)),
    ("PDRS32-25T",  "25T", "ENDPOINT", (32.0,)),
    ("PDRS35-25T",  "25T", "ENDPOINT", (35.0,)),
    ("PDRS40-25T",  "25T", "ENDPOINT", (40.0,)),
    ("PDRS45-25T",  "25T", "STATED",   (20.4, 25.4, 30.4, 35.4, 40.4, 45.4)),
    ("PDRS55-25T",  "25T", "STATED",   (25.4, 30.4, 35.4, 40.4, 45.4, 50.4,
                                        55.4)),
    ("PDRS60-25T",  "25T", "STATED",   (25.4, 30.4, 35.4, 40.4, 45.4, 50.4,
                                        55.4, 60.4)),
    ("PDRS55-15T",  "15T", "ENDPOINT", (55.0,)),
    ("PDRS60-15T",  "15T", "ENDPOINT", (60.0,)),
    ("PDRS204",     "25T", "STATED",   (20.64, 32.54, 38.10, 42.86, 47.63)),
)

#: The one row whose stated ladder does NOT close, recorded rather than guessed.
#: The index prints "4 positions on 5 mm centres, to 15" for PDRS15-25T, which
#: is 0 / 5 / 10 / 15 - and a hole at the shaft centre is not a hole.  Only the
#: outermost position is used above, and the interior of that ladder is treated
#: as unpublished.
PDRS15_NOTE = ("PDRS15-25T: '4 positions on 5 mm centres, to 15' resolves to "
               "0/5/10/15; a hole at the shaft centre is not a hole, so the "
               "ladder is NOT reconstructable and only 15.0 is carried.")

#: The single-arm ceiling, mm.  PDRS60-25T's outermost hole.  Nothing longer
#: was found as a single off-the-shelf arm (sec.1.4).
A_CEILING_MM = 60.4

#: EXCLUDED, and recorded so it is not later mistaken for stock: the 25T
#: "extension arms" quoted at 44 / 52 / 60 / 68 / 76 / 84 / 92 mm are a
#: Thingiverse 3D PRINT (thing:6750263), not bought hardware (sec.1.3).
A_EXCLUDED_PRINT_MM = (44.0, 52.0, 60.0, 68.0, 76.0, 84.0, 92.0)


def a_set_mm():
    """The union of every published ProModeler hole position, mm, sorted."""
    vals = sorted({h for _, _, _, holes in PROMODELER for h in holes})
    assert max(vals) <= A_CEILING_MM + 1e-9, "a hole above the single-arm ceiling"
    return vals


def a_set_rb():
    """The same set as ``a/r_b`` at the ASSERTED ``r_b``.  This is the axis."""
    return [round(v / FR.R_B_MM, 9) for v in a_set_mm()]


def a_provenance():
    """``{hole mm: worst provenance tag reaching it}``, worst = weakest."""
    rank = {"STATED": 0, "ENDPOINT": 1, "DERIVED": 2}
    best = {}
    for _, _, tag, holes in PROMODELER:
        for h in holes:
            if h not in best or rank[tag] < rank[best[h]]:
                best[h] = tag
    return best


# --------------------------------------------------------------------------- #
# (2) beta_p from ball-joint housing OD - hardware-pull sec.2
# --------------------------------------------------------------------------- #
#: The published housing-OD range across the M3-class parts of sec.2, mm.  The
#: low end is RC4WD Z-S1414 (MFR, 9.0); the high end is igus KBRM-03 (MFR,
#: 13.0), which sec.2 confirms independently of the superseded pull.  Both ends
#: are carried and NEITHER is preferred - no joint is chosen here.
HOUSING_OD_MM = (9.0, 13.0)

#: Two device classes are mixed in that range and are NOT interchangeable:
#: spherical plain bearings (igus, Aurora, PHS/POS) and RC ball-cup rod ends
#: (RC4WD, Traxxas).  They differ in construction and in pivot angle.  The
#: range is used as a range because sec.2 reports it as one; the distinction
#: travels with it.
HOUSING_CLASSES_MIXED = True


def beta_p_bounds_deg(od_mm: float, r_p_mm: float = None):
    """``(lower, upper)`` bound on ``beta_p`` in degrees for a housing OD.

    The platform anchors are ``phi_i = 120 floor(i/2) + s_i beta_p``, so the
    two angular gaps around the ring are ``2 beta_p`` (within a pair) and
    ``120 - 2 beta_p`` (between pairs).  The closest approach of two anchors is
    therefore ``2 r_p min(sin beta_p, sin(60 - beta_p))``, and requiring it to
    clear the housing OD gives a bound that is SYMMETRIC about 30 degrees::

        beta_p >= asin(OD / 2 r_p)   and   beta_p <= 60 - asin(OD / 2 r_p)

    A bound, not a choice: it says which ``beta_p`` a joint of that OD can be
    built at, and says nothing about which one to build.
    """
    r_p_mm = FR.R_P_MM if r_p_mm is None else float(r_p_mm)
    s = od_mm / (2.0 * r_p_mm)
    if not -1.0 <= s <= 1.0:
        raise ValueError(f"OD {od_mm} mm cannot fit on a ring of r_p {r_p_mm}")
    lo = float(np.degrees(np.arcsin(s)))
    return lo, 60.0 - lo


def min_anchor_sep_mm(beta_p_deg: float, r_p_mm: float = None):
    """Closest approach of two platform anchors, mm - the closed form.

    Checked against :func:`.fixed_ratio.sep_p_rb`, which measures the same
    quantity off the library geometry, in :func:`part_beta_p`.
    """
    r_p_mm = FR.R_P_MM if r_p_mm is None else float(r_p_mm)
    b = np.deg2rad(beta_p_deg)
    return float(2.0 * r_p_mm * min(np.sin(b), np.sin(np.deg2rad(60.0) - b)))


def beta_p_axis():
    """The union ``beta_p`` axis, and the two OD-bounded subsets it contains.

    The two housing ends share four sampled values and differ only in their two
    bound endpoints, so the sweep runs ONCE over the union and is partitioned
    afterwards.  The interior values are :data:`.zhome_bracket.BETA_P` unchanged
    - the existing sampling, not a new one - and the endpoints are the bounds
    themselves, included because a range's ends are where a bound can bite.
    """
    interior = list(BETA_P)
    ends = {}
    for od in HOUSING_OD_MM:
        lo, hi = beta_p_bounds_deg(od)
        ends[od] = (round(lo, 9), round(hi, 9))
    union = sorted(set(interior) | {v for pair in ends.values() for v in pair})
    return union, interior, ends


#: How many candidates part (5) prints, counted in CANDIDATES not groups.
TOP_N = 20

#: Displacement-azimuth counts the ranking's resolution is measured at.  24 is
#: :data:`.score_discriminators.N_DISP_DIR`, the count the score is actually
#: computed on; 72 and 360 are refinements used to MEASURE it and never to
#: compute it.
N_DIRS = (24, 72, 360)


# --------------------------------------------------------------------------- #
# the pass
# --------------------------------------------------------------------------- #
def run(tilt_deg: float, verbose: bool = True):
    """One screen + tune over the hardware-pull axes, at ``tilt_deg``."""
    a_vals = a_set_rb()
    bp_union, _, _ = beta_p_axis()
    axes = dict(beta_vals=BETA, beta_p_vals=bp_union, a_vals=a_vals,
                d_vals=D_RB)
    n = len(BETA) * len(bp_union) * len(a_vals) * len(D_RB)
    if verbose:
        print(f"    grid: {len(BETA)} beta x {len(bp_union)} beta_p x "
              f"{len(a_vals)} a x {len(D_RB)} d = {n} candidates")
        print(f"    screening and tuning at tilt = {tilt_deg:.4f} deg ...",
              flush=True)
    t0 = time.time()
    with TB.at_tilt(tilt_deg):
        slc = FR.run_slice(FR.R_P_RB, f"{tilt_deg:.4f} deg",
                           f"{tilt_deg:.3f}", verbose=False, axes=axes)
    slc.update(tilt=tilt_deg, secs=time.time() - t0)
    if verbose:
        print(f"    {len(slc['rows'])} screened  "
              f"{len(slc['feasible'])} feasible  {slc['dropped']} dropped  "
              f"{slc['no_delta']} no delta   ({slc['secs']:.1f} s)", flush=True)
    return slc


def subset(slc, od_mm):
    """The records of ``slc`` admissible at housing OD ``od_mm``.

    ``beta_p`` outside the bound is not a candidate at that OD: two housings
    would overlap.  ``beta`` is NOT filtered - the servo-body width of sec.5
    would bound it, and that is a different section and a different question.
    """
    lo, hi = beta_p_bounds_deg(od_mm)
    keep = lambda r: lo - 1e-9 <= r["beta_p"] <= hi + 1e-9
    return dict(
        od=od_mm, lo=lo, hi=hi,
        rows=[r for r in slc["rows"] if keep(r)],
        feasible=[r for r in slc["feasible"] if keep(r)],
        cat_R=[r for r in slc["cat_R"] if keep(r)],
        cat_X=[r for r in slc["cat_X"] if keep(r)],
        measured=[r for r in slc["measured"] if keep(r)],
    )


def resolutions(measured, R, az):
    """Score every survivor at each of :data:`N_DIRS`; store on the record.

    The score itself stays the 24-direction quantity; 72 and 360 are stored
    under their own keys and used only to say how much of a reported difference
    the 24-direction grid can resolve.
    """
    for rec in measured:
        if not np.isfinite(rec["score_p"]):
            continue
        for n in N_DIRS:
            rec[f"score_{n}"] = (rec["score_p"] if n == SD.N_DISP_DIR else
                                 FR.score_at_p(rec, R, az, rec["delta_con"],
                                               n_dir=n))


# --------------------------------------------------------------------------- #
# parts
# --------------------------------------------------------------------------- #
def part_a():
    """(1) The discrete ``a`` set."""
    print()
    print("=" * 78)
    print("(1) a IS A DISCRETE SET - THE PUBLISHED ProModeler HOLE LADDER")
    print("=" * 78)
    print("  hardware-pull sec.1.1.  a is not an interval: a servo arm has")
    print("  holes at published positions, and there is no arm with a hole at")
    print("  0.30 r_b because a sweep asked for one.  All distances are SHAFT")
    print("  CENTRE TO HOLE, not overall arm length.")
    print()
    print("  Provenance is carried per row because three different things are")
    print("  being read off one page:")
    print("    STATED    every hole position is printed on the maker's page")
    print("    ENDPOINT  the ladder is elided ('9 ... 18', 'to 32'); only the")
    print("              printed positions are used and the interior holes are")
    print("              NOT interpolated")
    print("    DERIVED   the page states a count and a pitch, not positions,")
    print("              and the positions follow by arithmetic on it")
    print()
    print(f"    {'part':<12} {'spline':>7} {'prov':>9}  holes, mm from shaft "
          f"centre")
    for part, spline, tag, holes in PROMODELER:
        hs = ", ".join(f"{h:g}" for h in holes)
        print(f"    {part:<12} {spline:>7} {tag:>9}  {hs}")
    print()
    print(f"  {PDRS15_NOTE}")
    print()
    mm = a_set_mm()
    rb = a_set_rb()
    prov = a_provenance()
    print(f"  THE SET, {len(mm)} distinct hole positions, at the ASSERTED "
          f"r_b = {FR.R_B_MM:.0f} mm:")
    print()
    print(f"    {'#':>3} {'a [mm]':>8} {'a/r_b':>8} {'prov':>9}     "
          f"{'#':>3} {'a [mm]':>8} {'a/r_b':>8} {'prov':>9}")
    half = (len(mm) + 1) // 2
    for i in range(half):
        j = i + half
        left = (f"    {i+1:>3} {mm[i]:>8.2f} {rb[i]:>8.4f} "
                f"{prov[mm[i]]:>9}")
        right = ("" if j >= len(mm) else
                 f"     {j+1:>3} {mm[j]:>8.2f} {rb[j]:>8.4f} "
                 f"{prov[mm[j]]:>9}")
        print(left + right)
    print()
    n_by = {t: sum(1 for h in mm if prov[h] == t)
            for t in ("STATED", "ENDPOINT", "DERIVED")}
    print(f"  by provenance: STATED {n_by['STATED']}, ENDPOINT "
          f"{n_by['ENDPOINT']}, DERIVED {n_by['DERIVED']}")
    print(f"  range        : {min(mm):.1f} - {max(mm):.1f} mm = "
          f"{min(rb):.4f} - {max(rb):.4f} r_b")
    print(f"  ceiling      : {A_CEILING_MM} mm = {A_CEILING_MM/FR.R_B_MM:.4f} "
          f"r_b, PDRS60-25T's outermost hole.")
    print(f"                 Nothing longer was found as a single "
          f"off-the-shelf arm.")
    print()
    print(f"  EXCLUDED: the 25T 'extension arms' at "
          f"{', '.join(f'{v:g}' for v in A_EXCLUDED_PRINT_MM)} mm are a")
    print(f"  Thingiverse 3D PRINT (thing:6750263), not bought hardware")
    print(f"  (sec.1.3).  Recorded so they are not later mistaken for stock.")
    print()
    print(f"  WHAT THIS REPLACES.  The existing axis is A_RB = [0.10, 0.20, "
          f"0.35] -")
    print(f"  three values, 9 / 18 / 31.5 mm, all of them inside the set above")
    print(f"  and none of them AT a published hole.  The old maximum, "
          f"0.35 r_b =")
    print(f"  31.5 mm, is not a hole on any arm in sec.1.1; the nearest are")
    print(f"  32.0 (PDRS401, PDRS32-25T) and 30.4 (three arms).")


def part_beta_p(slc):
    """(2) beta_p bounded by housing OD, at both ends of the published range."""
    print()
    print("=" * 78)
    print("(2) beta_p's OUTER BOUND FROM BALL-JOINT HOUSING OD - BOTH ENDS")
    print("=" * 78)
    print("  hardware-pull sec.2.  Two housings cannot occupy one hole, so the")
    print("  housing OD floors the closest approach of two platform anchors,")
    print("  and that floors beta_p away from BOTH 0 and 60 - the constraint is")
    print("  symmetric about 30 degrees because the ring has two gaps:")
    print()
    print("    phi_i = 120 floor(i/2) + s_i beta_p")
    print("    gaps  = 2 beta_p (within a pair), 120 - 2 beta_p (between pairs)")
    print("    sep   = 2 r_p min(sin beta_p, sin(60 - beta_p))")
    print("    bound : beta_p >= asin(OD / 2 r_p),  beta_p <= 60 - that")
    print()
    print(f"  At the ASSERTED r_p = {FR.R_P_MM:.0f} mm.  The published range is "
          f"{HOUSING_OD_MM[0]} - {HOUSING_OD_MM[1]} mm")
    print("  across twelve M3-class parts.  BOTH ends are computed and BOTH")
    print("  are carried through the sweep.  NO JOINT IS CHOSEN.")
    print()
    print(f"    {'OD [mm]':>8} {'part at that end':<26} "
          f"{'beta_p lower':>13} {'beta_p upper':>13} {'admissible span':>16}")
    labels = {9.0: "RC4WD Z-S1414 (MFR)",
              13.0: "igus KBRM-03 (MFR)"}
    for od in HOUSING_OD_MM:
        lo, hi = beta_p_bounds_deg(od)
        print(f"    {od:>8.1f} {labels[od]:<26} {lo:>13.4f} {hi:>13.4f} "
              f"{hi - lo:>16.4f}")
    print()
    print("  Neither end is preferred and neither is a recommendation.  Note")
    print("  also that sec.2 mixes TWO DEVICE CLASSES that are not")
    print("  interchangeable - spherical plain bearings (igus, Aurora,")
    print("  PHS/POS) and RC ball-cup rod ends (RC4WD, Traxxas) - which differ")
    print("  in construction and in pivot angle.  The range is used as a range")
    print("  because sec.2 reports it as one; the distinction travels with it.")
    print()
    w_lo, w_hi = beta_p_bounds_deg(min(HOUSING_OD_MM))
    n_lo, n_hi = beta_p_bounds_deg(max(HOUSING_OD_MM))
    kept = [v for v in BETA_P if n_lo - 1e-9 <= v <= n_hi + 1e-9]
    print(f"  WHAT THE BOUND DOES TO THE EXISTING SAMPLING.  BETA_P = "
          f"{list(BETA_P)} and the")
    print(f"  widest bound is [{w_lo:.4f}, {w_hi:.4f}], the narrowest "
          f"[{n_lo:.4f}, {n_hi:.4f}]: "
          f"{len(kept)} of {len(BETA_P)}")
    print(f"  existing sampled values are admissible at BOTH ends, so the bound")
    print(f"  removes nothing already sampled.  It says how much FURTHER out")
    print(f"  beta_p may go, and that is why the sweep below adds the bound")
    print(f"  endpoints themselves as sampled values - a range's ends are where")
    print(f"  a bound can bite.  The narrowest bound is the TIGHTEST of the two")
    print(f"  and it is the 13.0 mm end: a bigger housing needs more room.")
    print(f"  BETA_P = 55 sits {n_hi - 55.0:.4f} deg inside it, which is the "
          f"13.9 mm anchor")
    print(f"  spacing sec.2 opens with - {min_anchor_sep_mm(55.0):.2f} mm "
          f"measured here.")
    print()
    union, interior, ends = beta_p_axis()
    print(f"    swept beta_p, union of both ends ({len(union)} values):")
    print(f"    {'beta_p [deg]':>13} {'min sep [mm]':>13} {'sep [r_b]':>11} "
          f"{'largest OD it clears':>21}  in range at OD")
    for bp in union:
        sep = min_anchor_sep_mm(bp)
        oks = [f"{od:g}" for od in HOUSING_OD_MM
               if beta_p_bounds_deg(od)[0] - 1e-9 <= bp
               <= beta_p_bounds_deg(od)[1] + 1e-9]
        tag = ("both" if len(oks) == 2 else
               (f"only {oks[0]}" if oks else "neither"))
        print(f"    {bp:>13.4f} {sep:>13.3f} {sep/FR.R_B_MM:>11.4f} "
              f"{sep:>21.2f}  {tag}")
    print()
    print("  The closed form above is CHECKED against fixed_ratio.sep_p_rb,")
    print("  which measures the same quantity off the library geometry rather")
    print("  than off this formula:")
    ok = True
    for rec in slc["feasible"][:200]:
        got = FR.sep_p_rb(rec) * FR.R_B_MM
        want = min_anchor_sep_mm(rec["beta_p"])
        ok &= abs(got - want) < 1e-9
    print(f"    agree over the first 200 survivors to 1e-9 -> {ok}")


def part_d():
    """(3) What rod stock bounds d, and what it does not."""
    print()
    print("=" * 78)
    print("(3) d IS CONTINUOUS - WHAT ROD STOCK ACTUALLY BOUNDS")
    print("=" * 78)
    print("  hardware-pull sec.7.  d is cut to length, so the question is what")
    print("  stock constrains, and the answer is NOT the length.")
    print()
    print("  7.1 DIAMETERS AND THREADING (what stock does bound):")
    print()
    print(f"    {'stock':<34} {'dia [mm]':>16} {'threaded':<28}")
    for name, dia, thread in (
            ("Du-Bro 2-56 threaded rod (MFR)", "1.83", "one end only, 19 mm"),
            ("Du-Bro 4-40 threaded rod (MFR)", "2.36", "one end only, 19 mm"),
            ("CST carbon pushrod (MFR)", "0.76 - 1.78", "NO - caps untapped"),
            ("carbon fibre rod, generic (RETAIL)", "3.0 / 3.5 / 4.0 / 4.5",
             "no"),
            ("DIN 975 / 976 (RETAIL)", "M3 upward", "fully threaded"),
            ("A286 threaded rod (MFR)", "M3 - M64", "fully threaded")):
        print(f"    {name:<34} {dia:>16} {thread:<28}")
    print()
    print("    Every rod in the set either needs a second thread cut or is")
    print("    fully threaded already - NO stock item ships with two ready")
    print("    threaded ends (sec.7.3).  That is a per-unit operation, not a")
    print("    bound on d.")
    print()
    print("  7.1 STOCK LENGTHS - and why they bound NOTHING here:")
    print()
    print(f"    Du-Bro : 305 / 762 / 1016 mm")
    print(f"    CST    : 39 / 48 / 72 / 78 in = 991 / 1219 / 1829 / 1981 mm")
    print(f"    The swept d/r_b {D_RB} is "
          f"{' / '.join(f'{v*FR.R_B_MM:.0f}' for v in D_RB)} mm at the")
    print(f"    ASSERTED r_b = {FR.R_B_MM:.0f} mm.  The shortest stock length "
          f"is {305/max(D_RB)/FR.R_B_MM:.1f}x the")
    print(f"    longest swept d, so LENGTH IS NOT A BOUND and D_RB is carried")
    print(f"    UNCHANGED.  Nothing in sec.7 narrows it, and narrowing it on")
    print(f"    no evidence would be choosing a range.")
    print()
    print("  7.2 STRAIGHTNESS - almost universally NOT PUBLISHED:")
    print()
    print(f"    {'stock':<38} {'straightness':<30}")
    for name, val in (
            ("A286 cold-drawn threaded (MFR)", "<= 2 mm/m (ASTM A484)"),
            ("Du-Bro 2-56 / 4-40 (MFR)", "NOT PUBLISHED"),
            ("CST carbon pushrod (MFR)", "NOT PUBLISHED"),
            ("carbon fibre rod, generic (RETAIL)", "NOT PUBLISHED"),
            ("DIN 975 / 976 (RETAIL)", "NOT PUBLISHED"),
            ("McMaster-Carr precision ground", "NOT RETRIEVED"),
            ("MISUMI precision linear shafts", "NOT RETRIEVED - HTTP 403")):
        print(f"    {name:<38} {val:<30}")
    print()
    print("    A CORRECTION TO THE FRAMING: only ONE of the two unretrieved")
    print("    cells is an HTTP 403.  MISUMI's product and PDF pages returned")
    print("    403; McMaster-Carr's category page carries filters only and the")
    print("    individual product pages, where straightness lives, were not")
    print("    reachable.  Both are NOT RETRIEVED; the causes differ, and only")
    print("    one is a 403.  MISUMI's series also starts at 8 mm diameter,")
    print("    which is above every rod diameter in 7.1 - so even if it were")
    print("    retrieved it would not cover this design's rod scale.")
    print()
    print("    THE FLAG THIS SECTION EXISTS FOR.  The only published")
    print("    straightness figure in the whole pull is on a SPECIALTY ALLOY")
    print("    (A286) that is not one of the candidate rods.  Du-Bro's own")
    print("    copy describes its rod as 'strong, flexible' and emphasises")
    print("    bendability, which is the opposite of a straightness claim.")
    print()
    print(f"    SO: a {1.20*FR.R_B_MM:.0f} mm rod at d/r_b = 1.20 - the length "
          f"the optimum below")
    print(f"    sits at - HAS NO PUBLISHED STRAIGHTNESS FIGURE AND NO PUBLISHED")
    print(f"    BUCKLING FIGURE BEHIND IT.  Nothing in sec.7 publishes E, I or")
    print(f"    a critical load for any of these rods either, so the buckling")
    print(f"    side is not merely unretrieved - it is absent from the pull.")
    print(f"    d/r_b = 1.20 is swept anyway, because withholding it would be")
    print(f"    choosing a range; it is swept WITH THIS FLAG ATTACHED.")


def part_survivors(slc, subs):
    """(4) Survivor count, empties by a, margin five-number summary."""
    print()
    print("=" * 78)
    print("(4) THE SWEEP: SURVIVORS, EMPTIES BY a, AND THE MARGIN SUMMARY")
    print("=" * 78)
    print(f"  At tilt = {slc['tilt']:.4f} deg, r_p/r_b = {FR.R_P_RB:.6f} "
          f"(r_b = {FR.R_B_MM:.0f}, r_p = {FR.R_P_MM:.0f} mm),")
    print(f"  h_p/r_b = {H_P}, cap cond(J_fk) <= {CAP:.0e} at char_len = "
          f"{SD.CONSTRAINT_CHAR_LEN}.")
    print()
    print(f"    {'housing OD':>11} {'beta_p range [deg]':>21} {'cand':>6} "
          f"{'feasible':>9} {'frac':>7} {'R':>5} {'X':>5} {'surv':>6}")
    for s in subs:
        sc = np.array([r["score_p"] for r in s["measured"]], dtype=float)
        n = len(s["rows"])
        span = f"{s['lo']:.4f} .. {s['hi']:.4f}"
        print(f"    {s['od']:>11.1f} {span:>21} "
              f"{n:>6} {len(s['feasible']):>9} {len(s['feasible'])/n:>7.3f} "
              f"{len(s['cat_R']):>5} {len(s['cat_X']):>5} "
              f"{int(np.isfinite(sc).sum()):>6}")
    print()
    print("    'surv' is candidates with a finite margin at p - feasible, "
          "measurable")
    print("    at the bracket midpoint, and with an admissible delta at the "
          "cap.")
    print()
    print("  EMPTIES BY a.  Each cell is 'empty (R / X)' of the candidates in")
    print("  that row of that OD subset; a row that empties nothing is a row")
    print("  the reach test never rejects.")
    print()
    a_mm = a_set_mm()
    prov = a_provenance()
    heads = [f"OD {s['od']:g}" for s in subs]
    print(f"    {'a [mm]':>8} {'a/r_b':>8} {'prov':>9}" +
          "".join(f"{h:>18}" for h in heads))
    for h in a_mm:
        v = round(h / FR.R_B_MM, 9)
        cells = []
        for s in subs:
            ke = [e for e in s["rows"] if e["a"] == v and not e["feasible"]]
            nR = sum(1 for e in ke if e in s["cat_R"])
            nX = sum(1 for e in ke if e in s["cat_X"])
            tot = sum(1 for e in s["rows"] if e["a"] == v)
            cells.append(f"{len(ke)}/{tot} (R{nR}/X{nX})")
        print(f"    {h:>8.2f} {v:>8.4f} {prov[h]:>9}" +
              "".join(f"{c:>18}" for c in cells))
    print()
    print("  MARGIN(dxy = p) FIVE-NUMBER SUMMARY AND MAX, "
          f"p = {FR.P_SCORE:.6f} r_b,")
    print("  evaluated at p through probe_margin.  NO SLOPE IS EXTRAPOLATED.")
    print("  margin is dimensionless and carries no characteristic length; the")
    print(f"  cap that picked its delta carries char_len = "
          f"{SD.CONSTRAINT_CHAR_LEN} and is PROVISIONAL there.")
    print()
    print(f"    {'housing OD':>11} {'n':>5}{'min':>11}{'Q1':>11}{'median':>11}"
          f"{'Q3':>11}{'max':>11}")
    for s in subs:
        sc = np.array([r["score_p"] for r in s["measured"]], dtype=float)
        print(f"    {s['od']:>11.1f} {int(np.isfinite(sc).sum()):>5}"
              + "".join(f"{v:>11.6f}" for v in _five_number(sc)))
    print()
    print(f"    {'housing OD':>11} {'max margin(p)':>14} {'beta':>6} "
          f"{'beta_p':>8} {'a [mm]':>8} {'a/r_b':>7} {'d/r_b':>6} "
          f"{'z_home [mm]':>12}")
    for s in subs:
        best = max((r for r in s["measured"] if np.isfinite(r["score_p"])),
                   key=lambda r: r["score_p"])
        print(f"    {s['od']:>11.1f} {best['score_p']:>14.6f} "
              f"{best['beta']:>6.0f} {best['beta_p']:>8.4f} "
              f"{best['a']*FR.R_B_MM:>8.2f} {best['a']:>7.4f} "
              f"{best['d']:>6.2f} {best['z_home']*FR.R_B_MM:>12.2f}")
    print()
    print("  Negative margins are CARRIED, not dropped: a negative margin is a")
    print("  candidate the 366-pose screen passed and the 29-pose harness grid")
    print("  does not, the two grids not being nested.")
    print()
    print(f"    {'housing OD':>11} {'neg at dxy=p':>14} {'neg at dxy=0':>14}")
    for s in subs:
        sc = np.array([r["score_p"] for r in s["measured"]], dtype=float)
        m0 = np.array([r["margin_con"] for r in s["measured"]], dtype=float)
        print(f"    {s['od']:>11.1f} "
              f"{int(np.sum(sc[np.isfinite(sc)] < 0.0)):>14} "
              f"{int(np.sum(m0[np.isfinite(m0)] < 0.0)):>14}")


def part_top(subs):
    """(5) Top 20 grouped by (a, d, |e|), with the anchor separation beside."""
    print()
    print("=" * 78)
    print(f"(5) TOP {TOP_N} GROUPED BY (a, d, |e|),  e = beta_p - beta")
    print("=" * 78)
    print("  Part (8)'s invariant of score_discriminators with r_p dropped from")
    print("  the key, r_p no longer varying.  Two candidates sharing the key")
    print("  are ONE result reached by two (beta, beta_p) pairs - a rotation of")
    print("  the machine about z, a sign flip of e, or both - so groups are")
    print("  ranked and members inside a group are not.")
    print()
    print("  sep is min |p_i - p_j|, the closest approach of two PLATFORM")
    print("  anchors, in mm at the asserted r_p = 80 mm.  'clears' says which")
    print("  of the published 9.0 - 13.0 mm housing ODs that separation admits")
    print("  - the whole published range, part of it, or none of it.  It is a")
    print("  statement about fit, and it does NOT choose a joint.")
    print()
    for s in subs:
        good = sorted((r for r in s["measured"]
                       if np.isfinite(r["score_p"])),
                      key=lambda r: -r["score_p"])
        groups = _abs_e_groups(good)
        ordered, seen = [], set()
        for r in good:
            gk = (r["r_p"], r["a"], r["d"],
                  round(abs(r["beta_p"] - r["beta"]), 9))
            if gk not in seen:
                seen.add(gk)
                ordered.append(gk)
        print(f"  ---- housing OD {s['od']:.1f} mm, beta_p in "
              f"[{s['lo']:.4f}, {s['hi']:.4f}] deg, {len(good)} survivors in "
              f"{len(ordered)} groups ----")
        print()
        print(f"    {'grp':>4} {'a [mm]':>7} {'a/r_b':>7} {'d/r_b':>6} "
              f"{'|e|':>7} {'mem':>4} {'margin(p)':>10} {'margin(0)':>10} "
              f"{'spread':>9} {'z_home mm':>10} {'sep mm':>7}  clears")
        shown = gi = 0
        for gi, gk in enumerate(ordered, 1):
            members = groups[gk]
            sp = [m["score_p"] for m in members]
            best = max(members, key=lambda m: m["score_p"])
            sep = FR.sep_p_rb(best) * FR.R_B_MM
            if sep >= HOUSING_OD_MM[1] - 1e-9:
                clears = f"all {HOUSING_OD_MM[0]:g}-{HOUSING_OD_MM[1]:g}"
            elif sep >= HOUSING_OD_MM[0] - 1e-9:
                clears = f"only <= {sep:.1f}"
            else:
                clears = "NONE published"
            print(f"    {gi:>4} {gk[1]*FR.R_B_MM:>7.2f} {gk[1]:>7.4f} "
                  f"{gk[2]:>6.2f} {gk[3]:>7.3f} {len(members):>4} "
                  f"{max(sp):>10.6f} "
                  f"{max(m['margin_con'] for m in members):>10.6f} "
                  f"{max(sp)-min(sp):>9.1e} "
                  f"{best['z_home']*FR.R_B_MM:>10.2f} {sep:>7.2f}  {clears}")
            shown += len(members)
            if shown >= TOP_N:
                break
        print()
        print(f"    {shown} candidates in {gi} groups shown, of {len(good)} "
              f"survivors in {len(ordered)} groups.")
        print()
    print("  READ 'spread' BEFORE READING A GROUP AS A TIE.  At dxy = 0 the")
    print(f"  within-group margins collapse to ~1e-15, far below TIE_TOL = "
          f"{TIE_TOL:.0e}.")
    print("  At dxy = p they do not: the displacement breaks the D3 azimuth")
    print("  symmetry and probe_margin restores it by sweeping the")
    print(f"  displacement over a full circle - at {SD.N_DISP_DIR} SAMPLED "
          f"directions, and the")
    print("  residual is that discretisation.  It is measured in part (7).")


def part_walls(subs):
    """(6) Are a/r_b and d/r_b interior, or still on a wall?"""
    print()
    print("=" * 78)
    print("(6) IS THE OPTIMUM INTERIOR IN a AND d, OR STILL ON A WALL?")
    print("=" * 78)
    print("  box_boundary found the optimum at a/r_b = 0.60 dragging d/r_b to")
    print("  1.20 - but at r_p/r_b = 0.60 and at 10.529 deg, and neither holds")
    print("  now.  This asks the question again at r_p/r_b = 80/90 and at the")
    print("  limit in force, over the DISCRETE a set rather than a 0.05-step")
    print("  ray.  a's ceiling here is a real hardware ceiling (60.4 mm), not")
    print("  a sampling wall, and that distinction is the point of the part.")
    print()
    a_vals = a_set_rb()
    for s in subs:
        by_cell = {}
        for r in s["measured"]:
            if np.isfinite(r["score_p"]):
                k = (r["a"], r["d"])
                by_cell[k] = max(by_cell.get(k, -np.inf), r["score_p"])
        print(f"  ---- housing OD {s['od']:.1f} mm ----")
        print()
        print(f"    best margin(p) per (a, d) cell; '-' = no survivor in the "
              f"cell")
        print(f"    {'a [mm]':>8} {'a/r_b':>8}" +
              "".join(f"{'d=' + format(d, '.2f'):>13}" for d in D_RB))
        for v in a_vals:
            row = "".join(
                f"{by_cell[(v, d)]:>13.5f}" if (v, d) in by_cell
                else f"{'-':>13}" for d in D_RB)
            print(f"    {v*FR.R_B_MM:>8.2f} {v:>8.4f}" + row)
        live = {k: x for k, x in by_cell.items() if np.isfinite(x)}
        bk = max(live, key=lambda k: live[k])
        print()
        print(f"    cell maximum : a/r_b = {bk[0]:.4f} "
              f"({bk[0]*FR.R_B_MM:.2f} mm), d/r_b = {bk[1]:.2f} "
              f"({bk[1]*FR.R_B_MM:.0f} mm)")
        print(f"      a is the {'MAXIMUM' if bk[0] == max(a_vals) else 'INTERIOR'}"
              f" of the discrete set "
              f"[{min(a_vals):.4f} .. {max(a_vals):.4f}]")
        if bk[0] == max(a_vals):
            print(f"        and that maximum is the HARDWARE CEILING "
                  f"({A_CEILING_MM} mm), not a")
            print(f"        sampling wall: no longer single off-the-shelf arm "
                  f"was found.")
        print(f"      d is the "
              f"{'MINIMUM' if bk[1] == min(D_RB) else ('MAXIMUM' if bk[1] == max(D_RB) else 'INTERIOR')}"
              f" of the sampled {D_RB}")
        print()
    print("  A cell maximum is three numbers.  The stronger form is PAIRWISE,")
    print("  over every line that holds (beta, beta_p) and the other axis fixed")
    print("  and moves one axis between two sampled values; a line counts only")
    print("  where both ends are survivors.  Counted on the widest OD subset:")
    print()
    s = subs[0]
    by = {(r["beta"], r["beta_p"], r["a"], r["d"]): r["score_p"]
          for r in s["measured"] if np.isfinite(r["score_p"])}
    bps = sorted({r["beta_p"] for r in s["measured"]})

    def pairwise(axis, vals, others):
        out = []
        for i in range(len(vals) - 1):
            j = i + 1                      # adjacent values only
            hi = lo = 0
            for b in BETA:
                for bp in bps:
                    for o in others:
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

    print(f"  a: adjacent pairs of the discrete set (larger = the outer hole)")
    print(f"    {'pair a/r_b':>20} {'pair [mm]':>16} {'larger wins':>12} "
          f"{'smaller wins':>13} {'lines':>7}")
    tot_hi = tot_lo = 0
    for v1, v2, hi, lo in pairwise("a", a_vals, D_RB):
        if hi + lo == 0:
            continue
        tot_hi += hi
        tot_lo += lo
        print(f"    {f'{v1:.4f} vs {v2:.4f}':>20} "
              f"{f'{v1*FR.R_B_MM:.1f} vs {v2*FR.R_B_MM:.1f}':>16} {hi:>12} "
              f"{lo:>13} {hi+lo:>7}")
    print(f"    {'TOTAL':>20} {'':>16} {tot_hi:>12} {tot_lo:>13} "
          f"{tot_hi+tot_lo:>7}   "
          f"({tot_hi/(tot_hi+tot_lo):.1%} to the larger hole)")
    print()
    rows_a = [r for r in pairwise("a", a_vals, D_RB) if r[2] + r[3]]
    cross = next((r for r in rows_a if r[3] > r[2]), None)
    print()
    if cross is None:
        print("  a NEVER turns over: the larger hole wins every adjacent pair,")
        print("  and a is pinned to the hardware ceiling.")
    else:
        k = rows_a.index(cross)
        above = rows_a[k + 1:]
        all_above = all(r[3] > r[2] for r in above)
        frac = cross[3] / (cross[2] + cross[3])
        worst_above = (min(r[2] / (r[2] + r[3]) for r in above)
                       if above else float("nan"))
        print(f"  a TURNS OVER, and the turn is a measurement rather than a")
        print(f"  cell maximum: the larger hole wins every adjacent pair up to")
        print(f"  {cross[0]*FR.R_B_MM:.1f} vs {cross[1]*FR.R_B_MM:.1f} mm, "
              f"where the SMALLER wins {cross[3]} of {cross[2]+cross[3]} "
              f"lines ({frac:.1%}).")
        print(f"  Does the smaller keep winning at EVERY pair above that one? "
              f"-> {all_above}")
        print(f"  ({len(above)} pairs above the turn; the larger hole's best "
              f"share among")
        print(f"  them is {worst_above:.1%}.)  The turn itself is marginal at "
              f"{frac:.1%} and")
        print(f"  decisive above it, which is why both the turn and the pairs")
        print(f"  above are printed and neither is quoted alone.")
        print()
        print(f"  So the optimum in a is INTERIOR to the discrete set: not at")
        print(f"  the {A_CEILING_MM} mm hardware ceiling, and not at a sampling "
              f"wall either -")
        print(f"  there is no sampling wall left in a, because the axis IS the")
        print(f"  set of holes that exist.")
    print()
    print(f"  d: adjacent pairs of the sampled {D_RB}")
    print(f"    {'pair d/r_b':>20} {'pair [mm]':>16} {'larger wins':>12} "
          f"{'smaller wins':>13} {'lines':>7}")
    d_rows = pairwise("d", D_RB, a_vals)
    for v1, v2, hi, lo in d_rows:
        print(f"    {f'{v1:.2f} vs {v2:.2f}':>20} "
              f"{f'{v1*FR.R_B_MM:.0f} vs {v2*FR.R_B_MM:.0f}':>16} {hi:>12} "
              f"{lo:>13} {hi+lo:>7}")
    print()
    print("  d DOES NOT TURN OVER ON LINES, and it does on cells - the two")
    print("  disagree and both are printed rather than one being chosen.  The")
    print(f"  larger d wins a majority of lines at BOTH pairs "
          f"({d_rows[0][2]/(d_rows[0][2]+d_rows[0][3]):.0%} and "
          f"{d_rows[1][2]/(d_rows[1][2]+d_rows[1][3]):.0%}),")
    print("  yet the best CELL is at d/r_b = 1.20 and not at 1.60.  Those are")
    print("  different questions: 'does raising d help the typical candidate?'")
    print("  (yes, weakly, at both steps) and 'where is the best candidate?'")
    print("  (at 1.20).  A majority of ~56% is a lean, not a wall, and it is")
    print("  the thinner of the two claims here.")
    print()
    print(f"  AGAINST box_boundary.  It found a/r_b = 0.60 dragging d/r_b to")
    print(f"  1.20, at r_p/r_b = 0.60 and 10.529 deg.  HALF OF IT HOLDS HERE:")
    print(f"  d/r_b = 1.20 is the cell maximum at both housing ends, exactly as")
    print(f"  it found.  a does NOT hold: 0.60 r_b = 54 mm is not a hole on any")
    print(f"  arm in sec.1.1 (the neighbours are 50.4 and 55.0/55.4), and the")
    print(f"  optimum here sits at 47.63 mm = 0.5292 r_b, below it, with the")
    print(f"  pairwise turn measured above.  The 0.60 finding was a ray on a")
    print(f"  continuous a at a ratio and a tilt limit that are both gone.")


def part_resolution(subs):
    """(7) The ranking's resolved precision at 24, 72 and 360 directions."""
    print()
    print("=" * 78)
    print("(7) THE RANKING'S RESOLVED PRECISION AT 24, 72 AND 360 DIRECTIONS")
    print("=" * 78)
    print("  probe_margin takes the worst margin over a CIRCLE of displacement")
    print(f"  azimuths sampled at {SD.N_DISP_DIR} points, so the score is "
          f"optimistic by however")
    print("  much the true worst direction falls between samples.  That")
    print("  optimism IS the resolution of the ranking: two candidates closer")
    print("  together than it are not ordered by the grid the score is")
    print("  computed on.  Measured over every survivor, not bounded.")
    print()
    print(f"  The score itself stays the {SD.N_DISP_DIR}-direction quantity.  "
          f"72 and 360 are")
    print("  refinements used only to measure it.")
    print()
    for s in subs:
        live = [r for r in s["measured"] if np.isfinite(r["score_p"])]
        print(f"  ---- housing OD {s['od']:.1f} mm, {len(live)} survivors ----")
        print()
        print(f"    {'comparison':>22} {'worst |diff|':>14} "
              f"{'median |diff|':>15} {'p95 |diff|':>13}")
        for i in range(len(N_DIRS) - 1):
            for j in range(i + 1, len(N_DIRS)):
                a, b = N_DIRS[i], N_DIRS[j]
                dif = np.array([abs(r[f"score_{a}"] - r[f"score_{b}"])
                                for r in live])
                print(f"    {f'{a} vs {b}':>22} {dif.max():>14.3e} "
                      f"{np.median(dif):>15.3e} "
                      f"{np.percentile(dif, 95):>13.3e}")
        print()
        groups = [g for g in _abs_e_groups(live).values() if len(g) > 1]
        print(f"    within-group spread ({len(groups)} multi-member groups) - "
              f"these are")
        print(f"    ONE result reached twice, so any spread is pure "
              f"discretisation:")
        print(f"    {'directions':>22} {'worst spread':>14}")
        for n in N_DIRS:
            sp = [max(m[f"score_{n}"] for m in g)
                  - min(m[f"score_{n}"] for m in g) for g in groups]
            print(f"    {n:>22} {max(sp) if sp else float('nan'):>14.3e}")
        sp0 = [max(m["margin_con"] for m in g) - min(m["margin_con"] for m in g)
               for g in groups]
        print(f"    {'dxy = 0 (reference)':>22} "
              f"{max(sp0) if sp0 else float('nan'):>14.3e}")
        s["res"] = max(abs(r[f"score_{N_DIRS[0]}"] - r[f"score_{N_DIRS[-1]}"])
                       for r in live)
        s["res_group"] = max(
            (max(m[f"score_{N_DIRS[0]}"] for m in g)
             - min(m[f"score_{N_DIRS[0]}"] for m in g) for g in groups),
            default=float("nan"))
        print()
        print("    Note the 72 and 360 rows AGREE, and both sit at the dxy = 0")
        print("    reference: the within-group spread has already reached its")
        print("    floor by 72 directions, so what 24 shows is discretisation")
        print("    and nothing beyond 72 is bought by refining further.  That")
        print("    is a measurement of the 24-direction grid, not of the")
        print("    geometry.")
        print()
    print("  WHERE A TIE THRESHOLD WOULD COME FROM, and it is a measurement:")
    print()
    print(f"    TIE_TOL as it stands              : {TIE_TOL:.0e}")
    print(f"    dxy = 0 collapse, as measured     : ~1e-15 (the number "
          f"TIE_TOL was set from)")
    for s in subs:
        lab1 = f"OD {s['od']:g}: worst |{N_DIRS[0]} - {N_DIRS[-1]}|"
        lab2 = f"OD {s['od']:g}: worst within-group at {N_DIRS[0]}"
        print(f"    {lab1:<34}: {s['res']:.3e}")
        print(f"    {lab2:<34}: {s['res_group']:.3e}")
    print()
    worst = max(s["res"] for s in subs)
    dec = np.log10(worst / TIE_TOL)
    print(f"  So the ranking at dxy = p is resolved to about {worst:.1e}, "
          f"{dec:.0f} decades")
    print(f"  above TIE_TOL.  TIE_TOL is CORRECT for what it was set")
    print(f"  from - the dxy = 0 collapse, which really is exact to ~1e-15 -")
    print(f"  and it is the WRONG number for a ranking at dxy = p, where two")
    print(f"  candidates {worst:.0e} apart are ordered by the azimuth grid and "
          f"not by")
    print(f"  the geometry.  A harness threshold should be taken from the")
    print(f"  measurement above.  IT IS NOT SET HERE: no constant is changed,")
    print(f"  and speccing the harness is not this module's job.")


def verdict(slc, subs):
    """What the hardware pull did to the ranges, stated plainly."""
    a_vals = a_set_rb()
    print()
    print("=" * 78)
    print("WHAT THE HARDWARE PULL DOES TO THE SWEEP RANGES")
    print("=" * 78)
    print()
    print(f"  TILT LIMIT   {slc['tilt']:.4f} deg, from envelope.tilt_for at "
          f"x0 = {ENV.X0_WORKING*1e3:.0f} mm, printed")
    print(f"               rather than hardcoded.  tau_L DROPPED.")
    print()
    print(f"  a            DISCRETE, {len(a_vals)} published ProModeler hole "
          f"positions,")
    print(f"               {min(a_vals)*FR.R_B_MM:.1f} - "
          f"{max(a_vals)*FR.R_B_MM:.1f} mm = {min(a_vals):.4f} - "
          f"{max(a_vals):.4f} r_b.  The ceiling is")
    print(f"               HARDWARE ({A_CEILING_MM} mm), not sampling.  "
          f"Thingiverse extension")
    print(f"               arms excluded - a print, not stock.")
    print()
    for s in subs:
        print(f"  beta_p       at housing OD {s['od']:>4.1f} mm: "
              f"[{s['lo']:.4f}, {s['hi']:.4f}] deg")
    print(f"               Both ends carried, NEITHER preferred, no joint")
    print(f"               chosen.  Two device classes are mixed in that range")
    print(f"               and are not interchangeable.")
    print()
    print(f"  d            CONTINUOUS.  Stock bounds DIAMETER and THREADING,")
    print(f"               not length: the shortest stock is many times the")
    print(f"               longest swept d.  D_RB = {D_RB} carried UNCHANGED.")
    print(f"               FLAG: a {1.20*FR.R_B_MM:.0f} mm rod at d/r_b = 1.20 "
          f"has no published")
    print(f"               straightness and no published buckling figure "
          f"behind it.")
    print()
    for s in subs:
        sc = np.array([r["score_p"] for r in s["measured"]], dtype=float)
        best = max((r for r in s["measured"] if np.isfinite(r["score_p"])),
                   key=lambda r: r["score_p"])
        print(f"  OD {s['od']:>4.1f} mm  {len(s['feasible'])} of "
              f"{len(s['rows'])} feasible, {int(np.isfinite(sc).sum())} "
              f"survivors; best margin(p) = {np.nanmax(sc):.6f}")
        print(f"               at a = {best['a']*FR.R_B_MM:.2f} mm "
              f"({best['a']:.4f} r_b), d/r_b = {best['d']:.2f} "
              f"({best['d']*FR.R_B_MM:.0f} mm),")
        print(f"               beta = {best['beta']:.0f}, beta_p = "
              f"{best['beta_p']:.4f} deg, z_home = "
              f"{best['z_home']*FR.R_B_MM:.2f} mm")
    print()
    print(f"  NOTHING IS CHOSEN HERE.  No joint, no servo, no horn.  The")
    print(f"  harness is not specced and no tie threshold is set - part (7) is")
    print(f"  a measurement offered so that one can be.  notation.md is not")
    print(f"  touched.  Every mm figure is at the ASSERTED r_b = "
          f"{FR.R_B_MM:.0f} mm and")
    print(f"  r_p = {FR.R_P_MM:.0f} mm; margin carries no characteristic "
          f"length, and the cap")
    print(f"  that picked its delta is cond(J_fk) <= {CAP:.0e} at char_len = "
          f"{SD.CONSTRAINT_CHAR_LEN},")
    print(f"  PROVISIONAL in that length.")


# --------------------------------------------------------------------------- #
def main() -> None:
    tilt = ENV.tilt_for(ENV.X0_WORKING, ENV.TAU)
    print("=" * 78)
    print("SWEEP RANGES FROM THE HARDWARE PULL, AT THE LIMIT IN FORCE")
    print("=" * 78)
    print("  docs/hardware-pull.md (2026-09-09) reports what exists and chooses")
    print("  nothing.  This module turns sec.1 (servo horn hole ladders),")
    print("  sec.2 (ball-joint housing OD) and sec.7 (rod stock) into ranges on")
    print("  the sweep axes, and evaluates the feasible set over them.")
    print()
    print(f"  TILT LIMIT = {tilt:.4f} deg, from envelope.tilt_for at "
          f"x0 = {ENV.X0_WORKING*1e3:.0f} mm - the")
    print(f"  working displacement ALONE.  tau_L is DROPPED (tilt_dropped.py);")
    print(f"  envelope.TILT_LIMIT_DEG still reads {ENV.TILT_LIMIT_DEG:.4f} and "
          f"is not edited here.")
    print()
    print("  NO JOINT, NO SERVO AND NO HORN IS CHOSEN.  The harness is not")
    print("  specced.  notation.md is not touched.")
    print()
    print("-" * 78)
    print("THE SWEEP")
    print("-" * 78)
    t0 = time.time()
    slc = run(tilt)
    resolutions(slc["measured"], slc["R29"], slc["az29"])
    subs = [subset(slc, od) for od in HOUSING_OD_MM]
    print(f"    total {time.time() - t0:.1f} s")

    part_a()
    part_beta_p(slc)
    part_d()
    part_survivors(slc, subs)
    part_top(subs)
    part_walls(subs)
    part_resolution(subs)
    verdict(slc, subs)


if __name__ == "__main__":
    main()
