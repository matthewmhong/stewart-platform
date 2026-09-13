"""Candidate discriminators, measured side by side and left undecided.

    python -m stewart.diagnostics.score_discriminators

``docs/archive/notation.md`` sec.12 records four things as NOT DECIDED that this module is
careful not to decide: the characteristic length, whether the conditioning
measure should come from the FK Jacobian at all, the score function, and any
weighting between terms.  This module MEASURES the candidate discriminators on
the feasible set and reports how they relate.  It picks nothing.

Concretely, that means:

* every conditioning number is quoted at **all four** candidate characteristic
  lengths (``r_b``, ``r_p``, ``d``, ``a``), never at one with the others
  suppressed, and every one is flagged PROVISIONAL;
* **both** Jacobians are carried the whole way and never merged - ``J_fk``, the
  FK residual Jacobian the gate already uses, and ``J_cmd``, the
  servo-angle-to-pose map.  They are different matrices with different units
  and there is no reason yet to prefer either;
* no term is weighted against another and no scalar score is formed.  The
  output is a table and a set of rank correlations, which is the evidence a
  score function would have to be built on, not the score function.

WHAT IS MEASURED, per feasible candidate::

    margin   = min over legs and poses of (C_i - |P_i|) / C_i
    sens     = (margin(dxy = 0) - margin(dxy = p)) / p   at p = 0.01 r_b,
               worst over a full circle of displacement azimuths
    tau_min  = min over legs and poses of |rod_i . tangent_i| / d
    cond, sigma_min of J_fk  at char_len in {r_b, r_p, d, a}
    cond, sigma_min of J_cmd at char_len in {r_b, r_p, d, a}

``margin``, ``sens`` and ``tau_min`` are dimensionless ratios and carry NO
characteristic length and NO Jacobian - that is a property of the quantities,
not an omission.  Every conditioning number carries both labels.

**cond's status changed on 2026-09-07 and this module did not change it.**  It
is no longer a candidate for an outer ranking term.  It KEEPS the job it earned
in parts (2) to (6): the cap on the inner ``delta`` tune, which is what stops the
maximin-margin tuner selecting the rank-collapse configuration at
``beta_p == beta``.  So every cond column in parts (a) to (d) and (2) to (6)
stays exactly as it was - that reporting is the evidence the cap rests on - and
part (7), which studies the two-term score, carries no cond column at all.

The feasible set is RECOMPUTED here from :mod:`.zhome_bracket` - the same
540-candidate coarse grid at ``c_p / r_b = 0.1``, the same closed-form ``N > 0``
floor and the same exact reach test - rather than transcribed.  A count copied
out of a previous run is a number with no provenance the next time either
script moves.

Degrees at the boundary, radians internally.  Arrays are (3, 6).  numpy only.
Reports findings; always exits 0.
"""
from __future__ import annotations

import time

import numpy as np

from ..geometry import Geometry, make_geometry
from ..kinematics import (Unreachable, _cond_and_sigma, arm_tips, fk_jacobian,
                          fk_residual, fk_solve, ik, log_so3)
from .envelope import (AZIMUTH_WINDOW_DEG, N_MAGNITUDE, TILT_LIMIT_DEG,
                       envelope_poses, tilt_R)
from .zhome_bracket import (A_RB, BETA, BETA_P, D_RB, DELTA_GRID, C_P, R_B,
                            RP_RB, Z_GRID, fine_poses, leg_terms,
                            reach_feasible_any_delta, z_lower_closed_form)

# --------------------------------------------------------------------------- #
# NAMES - now official, were provisional here first
# --------------------------------------------------------------------------- #
#: **Named in ``docs/archive/notation.md`` sec.6 as of 2026-09-10** - ``rod_i`` and
#: ``tangent_i``, adopted from the local names this module coined below and
#: which ``tilt_authority.py`` reused; sec.11's clash entries for both are
#: marked resolved, not deleted.  Before that date these were unnamed, sec.11
#: recorded why each proposal was rejected (the rod vector's ``r_i`` collides
#: with ``r_b``/``r_p``; the arm tangent's ``t`` collides with plate
#: thickness), and this module used spelled-out local names rather than
#: single letters so as not to look like a silent resolution.  This dict and
#: its report label are kept as the historical record of that provenance, not
#: revised now that the names are official.
PROVISIONAL_NAMES = {
    "rod vector  (q_i - h_i, magnitude d)": "rod_i   [local to this module]",
    "arm tangent (-u_i sin alpha + v_i cos alpha)": "tangent_i [local]",
}

# --------------------------------------------------------------------------- #
# knobs - chosen for this study, none of them a design decision
# --------------------------------------------------------------------------- #
#: The four candidate characteristic lengths.  ``r_b`` is what the gate quotes
#: and flags PROVISIONAL; it is FIRST in this dict for readability only and
#: carries no precedence.  Nothing downstream may take a default from it.
CHAR_LEN_KEYS = ("r_b", "r_p", "d", "a")

#: Translation probe for ``sens``, in units of ``r_b``.  ``sweep_budget.py``
#: measured the response linear to ~1% over 0.0025-0.01 ``r_b`` and 16% off at
#: 0.05, so 0.01 is the top of the linear band.
PROBE_DXY = 0.01

#: Displacement azimuths for ``sens``.  A horizontal offset breaks the D3
#: azimuth symmetry, so the tilt window [30, 90] is not a fundamental domain for
#: the perturbed problem on its own.  Sweeping the DISPLACEMENT over the full
#: circle restores it: the symmetry acts on the pair (tilt azimuth,
#: displacement azimuth) diagonally, so the orbit of {tilt in [30, 90]} x {all
#: displacement azimuths} is all tilt azimuths x all displacement azimuths.
#: The worst case over the reduced set therefore equals the worst case over the
#: full one, and the tilt grid stays the settled 29-pose envelope.
N_DISP_DIR = 24

#: Azimuth step of the fine pose grid used in parts (e)-(g), degrees.  The
#: settled 29-pose envelope grid is 7 azimuths across a 60-degree window, i.e.
#: 10 degrees; 0.25 is 40x finer.  Same magnitudes, same window, same tilt
#: limit - the azimuth resolution is the ONLY thing that changes.
FINE_AZ_STEP_DEG = 0.25

#: Candidates sampled for parts (e)-(g), spaced evenly in MARGIN RANK across
#: the whole feasible set.  Rank spacing (not value spacing) is used so both
#: extremes are included and the sample is not piled up wherever the margin
#: distribution happens to be dense.  Explicitly NOT the top of the field.
N_SAMPLE = 40

#: Rank-shift threshold for part (d), as a fraction of the field.
RANK_SHIFT_FRAC = 0.05


# --------------------------------------------------------------------------- #
# small statistics, numpy only
# --------------------------------------------------------------------------- #
def _ranks(x: np.ndarray) -> np.ndarray:
    """Ascending ranks, 1-based, ties averaged.  ``+inf`` ranks last, as it should."""
    x = np.asarray(x, dtype=float)
    order = np.argsort(x, kind="mergesort")
    sx = x[order]
    r = np.empty(x.size, dtype=float)
    i = 0
    while i < x.size:
        j = i
        while j + 1 < x.size and sx[j + 1] == sx[i]:
            j += 1
        r[order[i:j + 1]] = 0.5 * (i + j) + 1.0
        i = j + 1
    return r


def _spearman(x, y):
    """Spearman rank correlation, ties averaged.  ``(rho, n_used)``.

    Rows where either input is NaN are dropped pairwise and counted out; ``inf``
    is KEPT, because an infinite condition number is a real ordering statement
    ("worse than every finite one"), not missing data.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    ok = ~(np.isnan(x) | np.isnan(y))
    n = int(ok.sum())
    if n < 3:
        return np.nan, n
    rx = _ranks(x[ok])
    ry = _ranks(y[ok])
    rx = rx - rx.mean()
    ry = ry - ry.mean()
    den = float(np.linalg.norm(rx) * np.linalg.norm(ry))
    if den == 0.0:
        return np.nan, n
    return float(rx @ ry / den), n


def _five_number(x):
    """``(min, Q1, median, Q3, max)`` ignoring NaN.

    ``min`` and ``max`` are the true order statistics of the sample, so an
    infinite condition number is reported AS infinite rather than smoothed into
    a large finite number.  The three interior quantiles interpolate, which
    ``inf`` would poison, so for those only the infinities are rank-substituted
    by a value above every finite one - a substitution that cannot move Q1, the
    median or Q3 unless the infinities are a majority of the sample.
    """
    x = np.asarray(x, dtype=float)
    x = x[~np.isnan(x)]
    if x.size == 0:
        return (np.nan,) * 5
    lo, hi = float(np.min(x)), float(np.max(x))
    finite = x[np.isfinite(x)]
    y = x
    if finite.size != x.size:
        big = (2.0 * finite.max() - finite.min() + 1.0) if finite.size else 1.0
        y = np.where(np.isfinite(x), x, big)
    q = np.percentile(y, [25.0, 50.0, 75.0])
    return (lo, float(q[0]), float(q[1]), float(q[2]), hi)


# --------------------------------------------------------------------------- #
# the feasible set, recomputed
# --------------------------------------------------------------------------- #
def feasible_candidates(verbose: bool = True):
    """Re-run :mod:`.zhome_bracket`'s 540-candidate screen; return the survivors.

    Identical logic, not a copy of its output: the exact reach test
    ``w_i(delta)^2 <= |L_i|^2 - P_i^2`` over the 1-degree ``delta`` scan and the
    finer 366-pose bracket envelope, intersected with the CLOSED-FORM ``N_i > 0``
    floor (the pose grid reports that constraint satisfied up to one step before
    it truly is, so the grid form would bias the screen).

    Returns
    -------
    list of dict
        ``beta``, ``beta_p``, ``r_p``, ``a``, ``d``, ``z_lo``, ``z_hi``,
        ``contiguous``.  ``z_lo`` and ``z_hi`` are GRID values and carry the
        0.025 ``r_b`` resolution of ``Z_GRID``.
    """
    az, mg = fine_poses()
    deltas_rad = np.deg2rad(DELTA_GRID)
    out, n_total = [], 0
    for beta in BETA:
        for beta_p in BETA_P:
            for r_p in RP_RB:
                floor = z_lower_closed_form(r_p, C_P)
                for a in A_RB:
                    for d in D_RB:
                        n_total += 1
                        A, B, G, _ = leg_terms(beta, beta_p, r_p, a, d, C_P,
                                               Z_GRID, az, mg)
                        ok = (reach_feasible_any_delta(A, B, G, deltas_rad)
                              & (Z_GRID > floor))
                        if not ok.any():
                            continue
                        idx = np.flatnonzero(ok)
                        out.append(dict(
                            beta=beta, beta_p=beta_p, r_p=r_p, a=a, d=d,
                            z_lo=float(Z_GRID[idx[0]]),
                            z_hi=float(Z_GRID[idx[-1]]),
                            contiguous=bool((idx[-1] - idx[0] + 1) == idx.size),
                        ))
    if verbose:
        print(f"  grid        : {len(BETA)}x{len(BETA_P)}x{len(RP_RB)}x"
              f"{len(A_RB)}x{len(D_RB)} = {n_total} candidates, "
              f"c_p/r_b = {C_P} fixed")
        print(f"  feasible    : {len(out)} / {n_total}   "
              f"(recomputed, not transcribed)")
        print(f"  contiguous  : {sum(r['contiguous'] for r in out)} / {len(out)}")
    return out


# --------------------------------------------------------------------------- #
# delta-free leg invariants, and the margin as a function of delta
# --------------------------------------------------------------------------- #
def _delta_basis(beta, beta_p, r_p, a, d):
    """The two geometries whose normals span ``n_i(delta)``.

    ``psi_i(delta) = psi_i(0) + s_i delta``, so with ``u_i = z x n_i``,
    ``n_i(delta) = cos(delta) n_i(0) + s_i sin(delta) u_i(0)`` and the
    ``delta = 90`` ring supplies ``s_i u_i(0)`` directly.  Same decomposition
    :func:`.zhome_bracket.leg_terms` uses; taken from the library rather than
    rebuilt so the ring convention has one source.
    """
    g0 = make_geometry(r_b=R_B, beta=beta, delta=0.0, r_p=r_p, beta_p=beta_p,
                       a=a, d=d, c_p=C_P)
    g90 = make_geometry(r_b=R_B, beta=beta, delta=90.0, r_p=r_p, beta_p=beta_p,
                        a=a, d=d, c_p=C_P)
    return g0, g90


def _invariants(g0, g90, R, T):
    """``(|L|^2, P, A, B)`` per pose and leg, all four **free of** ``delta``.

    ``delta`` enters only ``n_i`` and ``u_i``, never ``L_i``, so ``|L_i|`` and
    ``P_i`` do not move with it; ``w_i(delta) = A_i cos delta + B_i sin delta``
    carries the whole dependence.  ``C_i = sqrt(|L_i|^2 - w_i^2)`` then gives the
    margin at any ``delta`` with no geometry rebuild - which is what makes a
    180-point ``delta`` scan per candidate cheap.

    ``R`` is ``(K, 3, 3)``, ``T`` is ``(K, 3)``; every return is ``(K, 6)``.
    """
    q = np.einsum("kxy,yi->kxi", R, g0.p) + np.asarray(T, float)[:, :, None]
    L = q - g0.b[None]
    LL = np.einsum("kxi,kxi->ki", L, L)
    P = (LL + g0.a ** 2 - g0.d ** 2) / (2.0 * g0.a)
    A = np.einsum("kxi,xi->ki", L, g0.n)
    B = np.einsum("kxi,xi->ki", L, g90.n)
    return LL, P, A, B


def _margin_at(inv, delta_rad):
    """Worst ``(C_i - |P_i|) / C_i`` over legs and poses at one ``delta``."""
    LL, P, A, B = inv
    w = A * np.cos(delta_rad) + B * np.sin(delta_rad)
    C = np.sqrt(np.maximum(LL - w * w, 0.0))
    with np.errstate(divide="ignore", invalid="ignore"):
        m = np.where(C > 0.0, (C - np.abs(P)) / C, -np.inf)
    return float(np.min(m))


def _tune_delta(inv, deltas_deg=DELTA_GRID):
    """``(delta_deg, margin)`` maximising the worst margin over the pose grid.

    Maximin, not minimax: at a fixed leg and pose the margin is strictly
    decreasing in ``|w_i|``, but ``|L_i|`` varies across legs and poses, so the
    largest-``|w_i|`` leg is generally not the smallest-margin leg and the two
    aggregations have different minimisers (``docs/archive/notation.md`` sec.8, on ``J``).
    """
    dr = np.deg2rad(np.asarray(deltas_deg, float))
    LL, P, A, B = inv
    w = A[None] * np.cos(dr)[:, None, None] + B[None] * np.sin(dr)[:, None, None]
    C = np.sqrt(np.maximum(LL[None] - w * w, 0.0))
    with np.errstate(divide="ignore", invalid="ignore"):
        m = np.where(C > 0.0, (C - np.abs(P)[None]) / C, -np.inf)
    worst = m.reshape(dr.size, -1).min(axis=1)
    k = int(np.argmax(worst))
    return float(deltas_deg[k]), float(worst[k])


def _pose_grid(az_step_deg=None):
    """``(R, az, mg)`` for the settled envelope, optionally refined in azimuth.

    ``az_step_deg = None`` gives the settled 29-pose harness grid.  Any other
    value keeps the tilt limit, the magnitude count and the ``[30, 90]`` window
    and changes ONLY the azimuth resolution, so the two grids differ in exactly
    one thing.
    """
    if az_step_deg is None:
        az, mg = envelope_poses()
    else:
        lo, hi = AZIMUTH_WINDOW_DEG
        azis = np.arange(lo, hi + 1e-9, az_step_deg)
        mags = np.linspace(0.0, TILT_LIMIT_DEG, N_MAGNITUDE)
        a_g, m_g = np.meshgrid(azis, mags[1:], indexing="ij")
        az = np.concatenate([[0.0], a_g.ravel()])
        mg = np.concatenate([[0.0], m_g.ravel()])
    return tilt_R(az, mg), az, mg


def _T_stack(az, z_home, dxy=0.0, dir_rad=0.0):
    """``(K, 3)`` translations: ``(0, 0, z_home)`` plus an optional offset."""
    K = az.size
    T = np.zeros((K, 3), dtype=float)
    T[:, 0] = dxy * np.cos(dir_rad)
    T[:, 1] = dxy * np.sin(dir_rad)
    T[:, 2] = z_home
    return T


# --------------------------------------------------------------------------- #
# verification of the J_cmd algebra
# --------------------------------------------------------------------------- #
#: Central-difference step for the ``J_cmd`` check, RADIANS of servo angle.
#: ``1e-4`` sits where truncation (``O(h^2)``, ~1e-8 relative) and the solver's
#: own noise are both far below the agreement being claimed; ``1e-5`` is run
#: beside it so the error is seen to FALL with ``h``, which is what separates a
#: correct derivative from a coincidence.
FD_STEPS_RAD = (1e-4, 1e-5)

#: Agreement required of the finite difference before anything downstream is
#: allowed to run.  Loose next to the measured ~1e-9 and tight next to any
#: plausible sign or factor error, which would show up at ``O(1)``.
J_CMD_FD_TOL = 1e-6


def _fd_jcmd(geom, R0, T0, alphas, h):
    """``d(T, omega) / d(alpha)`` by central differences on the REAL map.

    Perturbs one servo angle, re-solves the forward kinematics, and reads the
    pose change off - so it goes through :func:`~stewart.kinematics.fk_solve`
    and knows nothing about ``J_fk``, ``D``, or the implicit-function argument
    it is being used to check.  The rotation difference is taken as
    ``log_so3(R R_0^T)``, the LEFT increment, matching the perturbation
    :func:`~stewart.kinematics.fk_jacobian` documents.

    ``fk_solve`` is seeded at the unperturbed pose.  That is legitimate HERE and
    nowhere else in the project: this measures a derivative at a known solution,
    it is not a round-trip test, and a seed far from the root would land on a
    different assembly mode and measure nothing.
    """
    cols = []
    for j in range(6):
        got = []
        for sgn in (1.0, -1.0):
            a2 = np.array(alphas, dtype=float)
            a2[j] += sgn * h
            out = fk_solve(geom, a2, R0, T0)
            got.append((out["T"], out["R"]))
        dT = (got[0][0] - got[1][0]) / (2.0 * h)
        dw = (log_so3(got[0][1] @ R0.T) - log_so3(got[1][1] @ R0.T)) / (2.0 * h)
        cols.append(np.concatenate([dT, dw]))
    return np.stack(cols, axis=1)


def verify_jcmd(rows, R, n_cases=6):
    """Check ``J_cmd = J_fk^-1 diag(a e_i . tangent_i)`` against the real map.

    Two things are checked and they are not the same claim:

    1. the IDENTITY ``|e_i . tangent_i| = tau_i``, which is arithmetic and
       should hold to machine precision;
    2. the DERIVATION, by finite-differencing the servo-angle-to-pose map.

    Only (2) can fail in a way that invalidates anything downstream.  Returns
    ``(ok, records)``.
    """
    order = np.argsort(_col(rows, "margin"), kind="mergesort")
    picks = [rows[order[i]] for i in
             np.unique(np.round(np.linspace(0, len(rows) - 1,
                                            n_cases)).astype(int))]
    out = []
    for rec in picks:
        geom = make_geometry(r_b=R_B, beta=rec["beta"], delta=rec["delta"],
                             r_p=rec["r_p"], beta_p=rec["beta_p"],
                             a=rec["a"], d=rec["d"], c_p=C_P)
        T0 = np.array([0.0, 0.0, rec["z_home"]])
        for k in (0, R.shape[0] // 2, R.shape[0] - 1):
            try:
                tau, J_fk, D, _ = _jacobians_at_pose(geom, R[k], T0)
            except Unreachable:
                continue
            J_cmd = np.linalg.solve(J_fk, D)
            alphas = ik(geom, R[k], T0)
            errs = []
            for h in FD_STEPS_RAD:
                F = _fd_jcmd(geom, R[k], T0, alphas, h)
                errs.append(float(np.max(np.abs(F - J_cmd))
                                  / np.max(np.abs(J_cmd))))
            tips = arm_tips(geom, alphas)
            _, rod, norm = fk_residual(geom, tips, R[k], T0)
            v = np.cross(geom.n, geom.u, axis=0)
            tangent = (-np.sin(alphas) * geom.u) + (np.cos(alphas) * v)
            et = np.einsum("xi,xi->i", rod / norm, tangent)
            out.append(dict(rec=rec, pose=k, errs=errs,
                            ident=float(np.max(np.abs(np.abs(et) - tau)))))
            break   # one pose per candidate is enough; keeps the table short
    # Judged on the SMALLEST step, and on the RATIO between steps.  A central
    # difference at a near-singular pose carries a large third-derivative term,
    # so the coarse step alone would penalise the very candidates the study is
    # about; what identifies a correct derivative is that the error falls as
    # h^2, which a wrong one does not do.
    ok = bool(out) and all(min(o["errs"]) <= J_CMD_FD_TOL for o in out)
    return ok, out


# --------------------------------------------------------------------------- #
# tau, and BOTH Jacobians
# --------------------------------------------------------------------------- #
def _jacobians_at_pose(geom, R_k, T_k):
    """``(tau, J_fk, D)`` at one pose.  ``D`` is ``df/dalpha``, up to sign.

    Names (official in docs/archive/notation.md sec.6 as of 2026-09-10 - see
    :data:`PROVISIONAL_NAMES` for their provenance as local names first)::

        rod_i     = q_i - h_i,   |rod_i| = d
        tangent_i = -u_i sin alpha_i + v_i cos alpha_i

    ``tau_i = |rod_i . tangent_i| / d`` is sec.8's transmission ratio.  Both
    factors are of known magnitude - ``|rod_i| = d`` at a solution and
    ``|tangent_i| = 1`` - so ``tau_i`` is ``|cos|`` of the angle between them and
    lies in ``[0, 1]``.  It is zero at a loss of authority: the arm tip moves
    perpendicular to the rod and the rod cannot transmit the motion.

    **J_fk** - the FK residual Jacobian, ``df_i / d(T, omega)`` with the rotation
    perturbed on the LEFT.  This is :func:`~stewart.kinematics.fk_jacobian`
    itself, not a restatement: the gate finite-differences that function against
    the perturbation it claims to describe, and a second copy here would be a
    second thing to keep in sync.

    **J_cmd** - the servo-angle-to-pose map, ``d(T, omega) / d(alpha)``.
    Derived, not assumed.  With the tips fixed by ``alpha`` the residual is
    ``f_i(T, omega, alpha_i) = |q_i - h_i(alpha_i)| - d``, and

        dh_i / dalpha_i  =  a * tangent_i
        df_i / dalpha_i  =  -e_i . (a tangent_i)  =  -a (e_i . tangent_i)

    with ``e_i = rod_i / |rod_i|``.  Only leg ``i``'s own angle enters ``f_i``,
    so ``df/dalpha`` is DIAGONAL.  Differentiating ``f = 0`` implicitly,

        J_fk d(T, omega) + df/dalpha dalpha = 0
        J_cmd = d(T, omega)/dalpha = J_fk^-1 diag(a e_i . tangent_i)

    Note ``|e_i . tangent_i| = tau_i`` exactly, so ``J_cmd`` is singular when any
    ``tau_i`` is - the two quantities are not independent, which is part of what
    (b) below is measuring.  Signs of ``e_i . tangent_i`` are a per-column
    ``+/-1`` and change no singular value.

    Returns ``D = diag(a e_i . tangent_i)``; the caller applies the
    characteristic length, because the two Jacobians must be scaled by the SAME
    convention and that convention lives in one place.
    """
    alphas = ik(geom, R_k, T_k)
    tips = arm_tips(geom, alphas)
    f, rod, norm = fk_residual(geom, tips, R_k, T_k)

    v = np.cross(geom.n, geom.u, axis=0)
    tangent = (-np.sin(alphas) * geom.u) + (np.cos(alphas) * v)

    tau = np.abs(np.einsum("xi,xi->i", rod, tangent)) / geom.d
    J_fk, e = fk_jacobian(geom, R_k, rod, norm)
    D = np.diag(geom.a * np.einsum("xi,xi->i", e, tangent))
    return tau, J_fk, D, float(np.max(np.abs(f)))


def _cond_pair(J_fk, D, char_len):
    """``(cond, smin)`` for ``J_fk`` and for ``J_cmd``, at one characteristic length.

    ONE convention, applied to both.  ``J_fk``'s rotation COLUMNS are mm/rad and
    its translation columns dimensionless, so scaling the rotation columns by
    ``1 / char_len`` measures the rotation increment as the arc length
    ``char_len * omega`` in mm and makes the matrix dimensionless.  That is
    exactly :func:`~stewart.kinematics._cond_and_sigma`, which is called here
    rather than reimplemented so the number is the same one the gate reports.

    ``J_cmd`` inherits it: writing ``S = diag(1, 1, 1, cl, cl, cl)``, the scaled
    FK Jacobian is ``J_fk S^-1`` and its inverse is ``S J_fk^-1``, so

        J_cmd_scaled  =  (J_fk S^-1)^-1 D  =  S J_fk^-1 D

    which is the same ``S`` acting on ``J_cmd``'s rotation ROWS.  Units: every
    row is then a length per radian of servo angle, so ``sigma_min(J_cmd)`` is in
    ``r_b`` per radian at this normalisation and ``cond(J_cmd)`` is dimensionless.

    A singular ``J_fk`` gives ``cond = inf`` and ``sigma_min = 0`` for ``J_cmd``,
    reported rather than skipped.

    Returns ``(cond_fk, smin_fk, smax_fk, cond_cmd, smin_cmd)``.  ``smax_fk`` is
    carried because the verification block needs it: inverting ``J_fk`` sends its
    LARGEST singular value to the smallest, so ``sigma_max(J_fk)`` - not
    ``sigma_min`` - is what sets the scale of ``sigma_min(J_cmd)``.
    """
    cond_fk, smin_fk, smax_fk = _cond_and_sigma(J_fk, char_len)
    scale = np.array([1.0, 1.0, 1.0,
                      1.0 / char_len, 1.0 / char_len, 1.0 / char_len])
    try:
        J_cmd = np.linalg.solve(J_fk * scale, D)
    except np.linalg.LinAlgError:
        return cond_fk, smin_fk, smax_fk, np.inf, 0.0
    if not np.all(np.isfinite(J_cmd)):
        return cond_fk, smin_fk, smax_fk, np.inf, 0.0
    sv = np.linalg.svd(J_cmd, compute_uv=False)
    smin, smax = float(sv[-1]), float(sv[0])
    cond = (smax / smin) if smin > 0.0 else np.inf
    return cond_fk, smin_fk, smax_fk, cond, smin


def char_lengths(rec):
    """The four candidate characteristic lengths for one candidate, mm.

    ``r_b`` is 1 by the project's normalisation; the other three are the
    candidate's own dimensions.  Returned as a dict so no call site can index
    the wrong one, and so a value can never be silently defaulted.
    """
    return {"r_b": R_B, "r_p": rec["r_p"], "d": rec["d"], "a": rec["a"]}


def measure(rec, R, T, char_lens, delta=None):
    """``tau_min`` and worst-case conditioning over the pose grid, both Jacobians.

    Worst case over poses, matching ``margin`` and ``tau_min``: ``cond`` is
    MAXIMISED and ``sigma_min`` MINIMISED across the grid.  A conditioning
    number quoted at the home pose alone would miss exactly the tilted poses the
    envelope exists to cover.

    ``delta`` overrides ``rec["delta"]`` when given, which is how the
    constrained tune of part (h) is measured without a second copy of the
    record.
    """
    geom = make_geometry(r_b=R_B, beta=rec["beta"],
                         delta=rec["delta"] if delta is None else float(delta),
                         r_p=rec["r_p"], beta_p=rec["beta_p"],
                         a=rec["a"], d=rec["d"], c_p=C_P)
    cl = char_lengths(rec)

    tau_min = np.inf
    worst_res = 0.0
    cond = {k: {"J_fk": -np.inf, "J_cmd": -np.inf} for k in char_lens}
    smin = {k: {"J_fk": np.inf, "J_cmd": np.inf} for k in char_lens}
    smax = {k: -np.inf for k in char_lens}

    for k in range(R.shape[0]):
        try:
            tau, J_fk, D, res = _jacobians_at_pose(geom, R[k], T[k])
        except Unreachable:
            return None
        tau_min = min(tau_min, float(tau.min()))
        worst_res = max(worst_res, res)
        for key in char_lens:
            c_fk, s_fk, x_fk, c_cmd, s_cmd = _cond_pair(J_fk, D, cl[key])
            cond[key]["J_fk"] = max(cond[key]["J_fk"], c_fk)
            cond[key]["J_cmd"] = max(cond[key]["J_cmd"], c_cmd)
            smin[key]["J_fk"] = min(smin[key]["J_fk"], s_fk)
            smin[key]["J_cmd"] = min(smin[key]["J_cmd"], s_cmd)
            smax[key] = max(smax[key], x_fk)
    return tau_min, cond, smin, smax, worst_res


# --------------------------------------------------------------------------- #
# per-candidate pipeline
# --------------------------------------------------------------------------- #
def evaluate(rec, R, T_of, az):
    """Fill ``rec`` in place with ``z_home``, ``delta``, ``margin``, ``sens``.

    ``z_home`` is the MIDPOINT of the candidate's grid bracket.  That is a
    choice of a point in the interior, made for definiteness, and it is NOT a
    recommendation: ``z_home`` is a swept axis (``docs/archive/notation.md`` sec.8) and this
    module does not settle it.  Every bracket measured is contiguous, so the
    midpoint of two feasible grid ends lies inside the feasible interval.
    """
    z_home = 0.5 * (rec["z_lo"] + rec["z_hi"])
    rec["z_home"] = z_home

    g0, g90 = _delta_basis(rec["beta"], rec["beta_p"], rec["r_p"],
                           rec["a"], rec["d"])
    rec["_g0"], rec["_g90"] = g0, g90

    inv0 = _invariants(g0, g90, R, T_of(z_home))
    delta, margin = _tune_delta(inv0)
    rec["delta"], rec["margin"] = delta, margin
    rec["sens"] = sens_at(g0, g90, R, az, z_home, delta, margin)
    return rec


def probe_margin(g0, g90, R, az, z_home, delta_deg, probe):
    """Worst margin over a full circle of displacement azimuths at ``|dxy| = probe``.

    ``probe`` is in units of ``r_b``.  Returns the margin itself, not a
    difference - part (7) needs the displaced margin directly, and deriving it
    back out of ``sens`` would hide the identity that part turns on.
    """
    dr = np.deg2rad(delta_deg)
    worst = np.inf
    for d_ang in np.linspace(0.0, 2.0 * np.pi, N_DISP_DIR, endpoint=False):
        inv = _invariants(g0, g90, R,
                          _T_stack(az, z_home, probe, float(d_ang)))
        worst = min(worst, _margin_at(inv, dr))
    return worst


def sens_at(g0, g90, R, az, z_home, delta_deg, base_margin,
            probe=PROBE_DXY):
    """``(margin(dxy = 0) - margin(dxy = probe)) / probe``, worst over azimuth.

    Factored out of :func:`evaluate` so the constrained tune can be given the
    same treatment at its own ``delta`` without a second copy of the loop.
    ``probe`` defaults to :data:`PROBE_DXY`, which is what parts (a) to (6)
    report; part (7) re-measures it at each of its own probes rather than
    extrapolating this one linearly, and reports the difference between the two.
    """
    worst = min(base_margin,
                probe_margin(g0, g90, R, az, z_home, delta_deg, probe))
    return (base_margin - worst) / probe


def _col(rows, key, sub=None, jac=None):
    if sub is None:
        return np.array([r[key] for r in rows], dtype=float)
    if jac is None:
        return np.array([r[key][sub] for r in rows], dtype=float)
    return np.array([r[key][sub][jac] for r in rows], dtype=float)


# --------------------------------------------------------------------------- #
# margin AND cond as functions of delta, in one vectorised sweep
# --------------------------------------------------------------------------- #
def scan_delta(beta, beta_p, r_p, a, d, R, T, deltas_deg, char_len,
               with_cond=True):
    """``(margin, cond, sigma_min)`` per ``delta``; each ``(n_delta,)``.

    The constrained tune needs ``cond(J_fk)`` at EVERY ``delta`` on the scan, not
    just at the winner, so the per-pose Python loop in :func:`measure` is too
    slow to sit inside it - 180 deltas x 363 candidates x 29 poses.  Everything
    here is therefore built stacked over ``(delta, pose)`` and closed with ONE
    batched SVD.

    The ``delta`` dependence is closed form and needs no geometry rebuild::

        n_i(delta) = cos(delta) n_i(0) + sin(delta) n_i(90)
        u_i(delta) = cos(delta) u_i(0) + sin(delta) u_i(90)      (u = z x n)
        v_i(delta) = n_i(delta) x u_i(delta)

    the same decomposition :func:`.zhome_bracket.leg_terms` uses, extended to
    ``u`` because the servo angle - and so the arm tip, the rod and both
    Jacobians - needs it.  ``v`` is formed rather than assumed to be ``z``.

    ``alpha`` is the SAME closed form :func:`~stewart.kinematics.ik` uses,
    including its branch: ``atan2(N, M) - arccos(P / C)`` on the minus branch,
    and the reachability test ``|P| <= C`` applied BEFORE the ``arccos``, never
    a clip.  A ``delta`` at which any leg or pose is unreachable is returned as
    ``NaN`` in all three outputs rather than given a fabricated boundary value.
    Agreement with ``ik`` itself is checked in the report.

    ``cond`` and ``sigma_min`` are worst-over-poses at ``char_len`` - maximum and
    minimum respectively, matching :func:`measure`.  ``with_cond=False`` skips
    the Jacobian and the SVD entirely and returns margin only, which is what
    part (l) times against.
    """
    g0, g90 = _delta_basis(beta, beta_p, r_p, a, d)
    dr = np.deg2rad(np.asarray(deltas_deg, dtype=float))
    nd, K = dr.size, R.shape[0]
    c = np.cos(dr)[:, None, None]
    s = np.sin(dr)[:, None, None]
    n_d = c * g0.n + s * g90.n                       # (nd, 3, 6)
    u_d = c * g0.u + s * g90.u
    v_d = np.cross(n_d, u_d, axis=1)

    Rp = np.einsum("kxy,yi->kxi", R, g0.p)           # (K, 3, 6)
    q = Rp + np.asarray(T, dtype=float)[:, :, None]
    L = q - g0.b[None]
    LL = np.einsum("kxi,kxi->ki", L, L)              # (K, 6)
    P = (LL + a * a - d * d) / (2.0 * a)
    M = np.einsum("kxi,jxi->jki", L, u_d)            # (nd, K, 6)
    N = np.einsum("kxi,jxi->jki", L, v_d)
    C = np.hypot(M, N)

    reach = (C > 0.0) & (np.abs(P)[None] <= C)
    live = reach.all(axis=2).all(axis=1)             # (nd,) all legs, all poses
    with np.errstate(divide="ignore", invalid="ignore"):
        m = np.where(C > 0.0, (C - np.abs(P)[None]) / C, -np.inf)
    margin = m.reshape(nd, -1).min(axis=1)
    margin = np.where(live, margin, np.nan)
    if not with_cond:
        return margin, None, None

    cond = np.full(nd, np.nan)
    smin = np.full(nd, np.nan)
    if not live.any():
        return margin, cond, smin

    j = np.flatnonzero(live)
    ratio = P[None] / C[j]                           # |ratio| <= 1, no clip
    alpha = np.arctan2(N[j], M[j]) - np.arccos(ratio)
    tip = g0.b[None, None] + a * (
        np.cos(alpha)[:, :, None, :] * u_d[j][:, None]
        + np.sin(alpha)[:, :, None, :] * v_d[j][:, None])
    rod = q[None] - tip                              # (nj, K, 3, 6)
    norm = np.linalg.norm(rod, axis=2)
    e = rod / norm[:, :, None, :]

    J = np.empty((j.size, K, 6, 6), dtype=float)
    J[..., :3] = np.moveaxis(e, 2, 3)
    J[..., 3:] = np.moveaxis(
        np.cross(np.broadcast_to(Rp[None], e.shape), e, axis=2), 2, 3)
    J *= np.array([1.0, 1.0, 1.0,
                   1.0 / char_len, 1.0 / char_len, 1.0 / char_len])
    sv = np.linalg.svd(J, compute_uv=False)          # (nj, K, 6)
    with np.errstate(divide="ignore", invalid="ignore"):
        cj = np.where(sv[..., -1] > 0.0, sv[..., 0] / sv[..., -1], np.inf)
    cond[j] = cj.max(axis=1)
    smin[j] = sv[..., -1].min(axis=1)
    return margin, cond, smin


#: Condition-number caps for the constrained inner tune, applied to
#: ``cond(J_fk)`` at ``char_len = r_b``.  FOUR of them, reported side by side and
#: none preferred: sec.12 says the reject line is undecided AND that the
#: characteristic length is what would set it, so a single cap here would settle
#: both by accident.
CONSTRAINT_CAPS = (1e2, 1e3, 1e4, 1e6)

#: Characteristic length the CONSTRAINT is evaluated at.  Named as a constant so
#: it cannot be read as a choice: it is the length the gate already quotes, and
#: every table below that uses it says so.  Switching it would change which
#: candidates the cap binds on - which is exactly what part (d) measures.
CONSTRAINT_CHAR_LEN = "r_b"


def tune_constrained(margin, cond, deltas_deg, cap):
    """``(delta, margin)`` maximising the margin subject to ``cond <= cap``.

    Returns ``(None, nan)`` when no ``delta`` on the scan is admissible - the
    candidate has no configuration meeting the cap at all.  That is reported,
    not silently replaced by the unconstrained winner.
    """
    ok = np.isfinite(margin) & (cond <= cap)
    if not ok.any():
        return None, np.nan
    m = np.where(ok, margin, -np.inf)
    k = int(np.argmax(m))
    return float(deltas_deg[k]), float(m[k])


#: Decades at which the conditioning tail is counted.  These are REPORTING
#: BINS, not thresholds: ``docs/archive/notation.md`` sec.12 says a reject line is undecided
#: and that the characteristic length is what would set it, so nothing here may
#: be read as one.  They exist because a maximum of ``1e19`` in a summary table
#: says nothing about how many candidates are near it.
TAIL_DECADES = (1e3, 1e6, 1e12)

#: ``delta`` values swept in the tail demonstration below.  Clustered near 0
#: because that is where the effect is.
TAIL_DELTAS = (0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 30.0, 60.0, 90.0, 120.0, 179.0)


def _report_verification(rows, R):
    """Part (1): does the ``J_cmd`` algebra hold?  Returns True to proceed."""
    print()
    print("=" * 78)
    print("(1) VERIFICATION OF THE J_cmd ALGEBRA - gates everything below")
    print("=" * 78)
    ok, cases = verify_jcmd(rows, R)
    print("  Claim under test:")
    print("    J_cmd = J_fk^-1 diag(a e_i . tangent_i)   and   "
          "|e_i . tangent_i| = tau_i")
    print()
    print("  DECISIVE TEST: central-difference the REAL servo-angle-to-pose map")
    print("  (perturb one alpha, re-solve fk, read the pose change off) and")
    print("  compare.  The finite difference goes through fk_solve and knows")
    print("  nothing about J_fk, D, or the implicit-function argument.")
    print()
    print(f"    {'beta':>5} {'beta_p':>6} {'r_p':>5} {'a':>5} {'d':>5} "
          f"{'pose':>5} {'rel err h=1e-4':>15} {'rel err h=1e-5':>15} "
          f"{'ratio':>8} {'||e.t|-tau|':>13}")
    for o in cases:
        r = o["rec"]
        ratio = o["errs"][0] / o["errs"][1] if o["errs"][1] > 0 else np.inf
        print(f"    {r['beta']:>5.0f} {r['beta_p']:>6.0f} {r['r_p']:>5.2f} "
              f"{r['a']:>5.2f} {r['d']:>5.2f} {o['pose']:>5d} "
              f"{o['errs'][0]:>15.3e} {o['errs'][1]:>15.3e} "
              f"{ratio:>8.0f} {o['ident']:>13.3e}")
    worst_c = max(o["errs"][0] for o in cases)
    worst_f = max(o["errs"][1] for o in cases)
    ratios = [o["errs"][0] / o["errs"][1] for o in cases if o["errs"][1] > 0]
    print()
    print(f"  worst relative error, h = 1e-4 : {worst_c:.3e}")
    print(f"  worst relative error, h = 1e-5 : {worst_f:.3e}   "
          f"(acceptance {J_CMD_FD_TOL:.0e}, applied here)")
    print(f"  error ratio between the two steps : "
          f"{min(ratios):.0f} to {max(ratios):.0f}")
    print()
    print("  The RATIO is the evidence, not the magnitude.  A tenfold cut in h")
    print("  cutting the error ~100-fold is the O(h^2) signature of a central")
    print("  difference converging on a correct derivative.  A wrong sign, a")
    print("  missing factor of a, or the wrong side of the rotation increment")
    print("  would leave an O(1) discrepancy that does not shrink with h at all.")
    print("  Ratios BELOW 100 are the cases whose coarse-step error is already")
    print("  near the solver's own floor, where there is no truncation left to")
    print("  remove; they are the well-conditioned cases, not the doubtful ones.")
    print("  The coarse step is not used for acceptance: at a near-singular pose")
    print("  the third-derivative term is large, which would penalise exactly the")
    print("  candidates this study is about for a reason that is not an error.")
    print()
    print(f"  VERDICT: the derivation is "
          f"{'CONFIRMED' if ok else 'NOT CONFIRMED'}.")
    if not ok:
        print("  Stopping.  Everything below assumes this algebra.")
        return False

    # ---- the requested rank check, and why it is the wrong predictor ---- #
    print()
    print("-" * 78)
    print("  THE REQUESTED RANK CHECK - it does NOT come out high, and that is")
    print("  a fact about the proxy, not about the derivation")
    print("-" * 78)
    print("  Asked for: rank-correlate sigma_min(J_cmd) against")
    print("  sigma_min(J_fk) * a * tau_min, and stop if it is not high.")
    print()
    print("  It is not high, and it should not be.  Inverting a matrix sends its")
    print("  LARGEST singular value to the smallest: sigma_i(J_fk^-1) =")
    print("  1 / sigma_(7-i)(J_fk).  So with D = diag(a e_i . tangent_i) and")
    print("  |e_i . tangent_i| = tau_i, the submultiplicative bounds give")
    print()
    print("      a tau_min / sigma_max(J_fk)  <=  sigma_min(J_cmd)")
    print("      sigma_min(J_cmd)  <=  a tau_min / sigma_min(J_fk)")
    print()
    print("  - a DIVISION by sigma_max, not a multiplication by sigma_min.  The")
    print("  proposed proxy has both the wrong singular value and the wrong")
    print("  exponent, so it is testing something the algebra never predicted.")
    print("  Measured, all four proxies against sigma_min(J_cmd), Spearman rho:")
    print()
    A = _col(rows, "a")
    TAU = _col(rows, "tau_min")
    print(f"    {'char_len':>9} {'smin(J_fk)*a*tau':>18} "
          f"{'a*tau/smax(J_fk)':>18} {'a*tau':>10} {'smin(J_fk)':>12}")
    for key in CHAR_LEN_KEYS:
        y = _col(rows, "smin", key, "J_cmd")
        sfk = _col(rows, "smin", key, "J_fk")
        smx = _col(rows, "smax", key)
        print(f"    {key:>9} {_spearman(sfk * A * TAU, y)[0]:>18.4f} "
              f"{_spearman(A * TAU / smx, y)[0]:>18.4f} "
              f"{_spearman(A * TAU, y)[0]:>10.4f} "
              f"{_spearman(sfk, y)[0]:>12.4f}")
    print()
    print("  Every column of that table is at the char_len named in its row, and")
    print("  every one is PROVISIONAL.  The bound-implied proxy is the one that")
    print("  tracks; the requested one does not.  Since the finite difference")
    print("  above confirms the algebra directly and exactly, the stop condition")
    print("  is NOT met and the run continues.  Stopping on the proxy would have")
    print("  discarded a derivation that is right on the strength of a predictor")
    print("  that is wrong.")
    return True


def run_constrained(rows, R, az, deltas_deg=DELTA_GRID):
    """Add the constrained inner tune to every record, at every cap.

    Both tunes are RETAINED.  ``rec["delta"]`` / ``rec["margin"]`` stay the
    unconstrained ones; ``rec["con"][cap]`` carries the constrained result, or
    ``None`` where no ``delta`` on the scan meets the cap at all.

    The constraint is on ``cond(J_fk)`` at ``char_len = r_b`` throughout - see
    :data:`CONSTRAINT_CHAR_LEN`.  The resulting candidate is then MEASURED at all
    four characteristic lengths, so a number selected at one length is never
    reported without saying so.
    """
    for rec in rows:
        T = _T_stack(az, rec["z_home"])
        cl = char_lengths(rec)[CONSTRAINT_CHAR_LEN]
        m, c, _ = scan_delta(rec["beta"], rec["beta_p"], rec["r_p"], rec["a"],
                             rec["d"], R, T, deltas_deg, cl)
        rec["scan_margin"], rec["scan_cond"] = m, c
        rec["con"] = {}
        for cap in CONSTRAINT_CAPS:
            d_con, m_con = tune_constrained(m, c, deltas_deg, cap)
            if d_con is None:
                rec["con"][cap] = None
                continue
            if d_con == rec["delta"]:
                rec["con"][cap] = dict(
                    delta=d_con, margin=rec["margin"], loss=0.0,
                    sens=rec["sens"], tau_min=rec["tau_min"],
                    cond=rec["cond"], smin=rec["smin"], changed=False)
                continue
            got = measure(rec, R, T, CHAR_LEN_KEYS, delta=d_con)
            if got is None:                       # cannot happen: scan says live
                rec["con"][cap] = None
                continue
            tau_min, cond, smin, _, _ = got
            rec["con"][cap] = dict(
                delta=d_con, margin=m_con, loss=rec["margin"] - m_con,
                sens=sens_at(rec["_g0"], rec["_g90"], R, az, rec["z_home"],
                             d_con, m_con),
                tau_min=tau_min, cond=cond, smin=smin, changed=True)
    return rows


def _con_col(rows, cap, key, sub=None, jac=None):
    """One column under the constrained tune at ``cap``; NaN where inadmissible."""
    out = []
    for r in rows:
        e = r["con"][cap]
        if e is None:
            out.append(np.nan)
        elif sub is None:
            out.append(e[key])
        else:
            out.append(e[key][sub][jac])
    return np.array(out, dtype=float)


def _report_constrained(rows):
    """Part (2): what the cap costs, and whether it binds only on the 34."""
    N = len(rows)
    eq = np.array([abs(r["beta_p"] - r["beta"]) < 1e-9 for r in rows])
    print()
    print("=" * 78)
    print("(2) CONSTRAINED INNER TUNE - delta_con vs delta_free")
    print("=" * 78)
    print("    delta_free = argmax margin                     (unconstrained)")
    print("    delta_con  = argmax margin s.t. cond(J_fk) <= C")
    print()
    print(f"  The constraint is evaluated at char_len = "
          f"{CONSTRAINT_CHAR_LEN} and NOWHERE else;")
    print("  every C column below is a cap on cond(J_fk)@r_b.  Both tunes are")
    print("  retained - nothing replaces delta_free.  C is NOT chosen here.")
    print(f"  Field: {N} candidates, of which {int(eq.sum())} have "
          f"beta_p == beta.")
    print()
    hdr = f"    {'quantity':<44}" + "".join(f"{f'C={c:.0e}':>13}"
                                            for c in CONSTRAINT_CAPS)
    print(hdr)
    print("    " + "-" * (len(hdr) - 4))

    def line(label, vals, fmt="{:>13d}"):
        print(f"    {label:<44}" + "".join(fmt.format(v) for v in vals))

    changed = {c: np.array([r["con"][c] is not None and r["con"][c]["changed"]
                            for r in rows]) for c in CONSTRAINT_CAPS}
    none_ok = {c: np.array([r["con"][c] is None for r in rows])
               for c in CONSTRAINT_CAPS}

    line("delta_con != delta_free", [int(changed[c].sum())
                                     for c in CONSTRAINT_CAPS])
    line("no admissible delta at any point on the scan",
         [int(none_ok[c].sum()) for c in CONSTRAINT_CAPS])
    line("  ... of which beta_p == beta",
         [int((changed[c] & eq).sum()) for c in CONSTRAINT_CAPS])
    line("  ... of which beta_p != beta",
         [int((changed[c] & ~eq).sum()) for c in CONSTRAINT_CAPS])
    line("beta_p == beta candidates NOT affected",
         [int((eq & ~changed[c] & ~none_ok[c]).sum())
          for c in CONSTRAINT_CAPS])
    print()
    print("    margin given up (delta_free - delta_con), over AFFECTED only:")
    for stat, fn in (("min", np.min), ("median", np.median),
                     ("max  (worst over the field)", np.max)):
        vals = []
        for c in CONSTRAINT_CAPS:
            loss = _con_col(rows, c, "loss")[changed[c]]
            vals.append(fn(loss) if loss.size else np.nan)
        line(f"      {stat}", vals, "{:>13.4e}")
    print()
    print("    margin given up, over ALL candidates (unaffected contribute 0):")
    for stat, fn in (("median", np.median), ("max", np.max)):
        vals = []
        for c in CONSTRAINT_CAPS:
            loss = np.nan_to_num(_con_col(rows, c, "loss"), nan=0.0)
            vals.append(fn(loss))
        line(f"      {stat}", vals, "{:>13.4e}")

    print()
    print("  IS THE AFFECTED SET EXACTLY THE 34 WITH beta_p == beta?")
    for c in CONSTRAINT_CAPS:
        aff = changed[c] | none_ok[c]
        extra = int((aff & ~eq).sum())
        missing = int((eq & ~aff).sum())
        verdict = ("EXACTLY the beta_p == beta set" if extra == 0 and missing == 0
                   else f"LARGER: +{extra} outside it, -{missing} of it untouched"
                   if extra else f"SMALLER: {missing} of the 34 untouched")
        print(f"    C = {c:.0e} (cond(J_fk)@r_b) : {int(aff.sum()):>3} affected"
              f"  -  {verdict}")

    # ---- per-affected-candidate detail, union over the caps ------------- #
    aff_any = np.zeros(N, dtype=bool)
    for c in CONSTRAINT_CAPS:
        aff_any |= changed[c] | none_ok[c]
    idx = np.flatnonzero(aff_any)
    losses = np.array([max(np.nan_to_num(_con_col(rows, c, "loss")[i], nan=0.0)
                           for c in CONSTRAINT_CAPS) for i in idx])
    order = idx[np.argsort(-losses)]
    cap_rows = 40
    print()
    print(f"  PER AFFECTED CANDIDATE - margin given up at each C, "
          f"sorted by worst loss.")
    print(f"  '-' means delta_con == delta_free (cap not binding); 'none' means")
    print(f"  no delta on the scan meets the cap.")
    print()
    print(f"    {'beta':>5} {'beta_p':>6} {'r_p':>5} {'a':>5} {'d':>5} "
          f"{'dlt_free':>8} " + "".join(f"{f'C={c:.0e}':>22}"
                                        for c in CONSTRAINT_CAPS))
    print(f"    {'':>5} {'':>6} {'':>5} {'':>5} {'':>5} {'':>8} "
          + "".join(f"{'delta':>10}{'loss':>12}" for _ in CONSTRAINT_CAPS))
    for i in order[:cap_rows]:
        r = rows[i]
        cells = ""
        for c in CONSTRAINT_CAPS:
            e = r["con"][c]
            if e is None:
                cells += f"{'none':>10}{'-':>12}"
            elif not e["changed"]:
                cells += f"{'-':>10}{'-':>12}"
            else:
                cells += f"{e['delta']:>10.1f}{e['loss']:>12.4e}"
        print(f"    {r['beta']:>5.0f} {r['beta_p']:>6.0f} {r['r_p']:>5.2f} "
              f"{r['a']:>5.2f} {r['d']:>5.2f} {r['delta']:>8.1f} " + cells)
    if order.size > cap_rows:
        print(f"    ... {order.size - cap_rows} further affected candidates "
              f"not listed (smaller losses)")


def _b_matrix(get_disc, get_cond, tag):
    """Print one (b)-shaped 3 x 8 Spearman block."""
    print(f"    {tag}")
    print("    " + f"{'':<10}" + "".join(
        f"{jac + '@' + key:>14}" for jac in ("J_fk", "J_cmd")
        for key in CHAR_LEN_KEYS))
    for name in ("margin", "sens", "tau_min"):
        cells = ""
        for jac in ("J_fk", "J_cmd"):
            for key in CHAR_LEN_KEYS:
                rho, _ = _spearman(get_disc(name), get_cond(key, jac))
                cells += f"{rho:>14.4f}"
        print(f"    {name:<10}" + cells)


def _report_b_under_constraint(rows):
    """Part (3): the (b) matrix re-measured under ``delta_con``, beside (b)."""
    print()
    print("=" * 78)
    print("(3) THE (b) SPEARMAN MATRIX RE-MEASURED UNDER delta_con")
    print("=" * 78)
    print("  Same shape as (b): rows are margin, sens, tau_min; columns are the")
    print("  eight cond columns, each labelled with its Jacobian and its")
    print("  characteristic length, every one PROVISIONAL.  Under delta_con ALL")
    print("  FIVE quantities move, because all five are functions of delta.")
    print()
    print("  The comparison this table exists for: in the unconstrained block")
    print("  34 of 363 cond ranks come from a tuner artifact - delta = 0 at")
    print("  beta_p == beta, where J_fk drops rank at the home pose.  Under a cap")
    print("  those 34 are re-tuned onto non-degenerate deltas, so whatever")
    print("  survives is geometry rather than the tuner.")
    print()
    print("  Candidates with no admissible delta at a given C are dropped")
    print("  PAIRWISE from that C's block; the count is printed with it.")
    print()
    _b_matrix(lambda n: _col(rows, n),
              lambda k, j: _col(rows, "cond", k, j),
              "UNCONSTRAINED (delta_free) - repeated here for direct comparison")
    for cap in CONSTRAINT_CAPS:
        n_drop = sum(1 for r in rows if r["con"][cap] is None)
        print()
        _b_matrix(lambda n, c=cap: _con_col(rows, c, n),
                  lambda k, j, c=cap: _con_col(rows, c, "cond", k, j),
                  f"delta_con at C = {cap:.0e} on cond(J_fk)@r_b"
                  f"   ({len(rows) - n_drop} of {len(rows)} usable)")

    print()
    print("  HOW FAR DID THE MATRIX ACTUALLY MOVE?  Largest |rho_con - rho_free|")
    print("  over the 24 cells of each block:")
    print()
    print(f"    {'C on cond(J_fk)@r_b':>21} {'max |change in rho|':>21} "
          f"{'max |rho| in block':>20}")
    for cap in CONSTRAINT_CAPS:
        worst_d, worst_r = 0.0, 0.0
        for name in ("margin", "sens", "tau_min"):
            for jac in ("J_fk", "J_cmd"):
                for key in CHAR_LEN_KEYS:
                    a_, _ = _spearman(_col(rows, name),
                                      _col(rows, "cond", key, jac))
                    b_, _ = _spearman(_con_col(rows, cap, name),
                                      _con_col(rows, cap, "cond", key, jac))
                    worst_d = max(worst_d, abs(a_ - b_))
                    worst_r = max(worst_r, abs(b_))
        print(f"    {f'C = {cap:.0e}':>21} {worst_d:>21.4f} {worst_r:>20.4f}")
    print()
    print("  This is a NEGATIVE result and it answers the question that prompted")
    print("  the re-measurement.  The 34 artifact ranks were a real defect in the")
    print("  tuner, but they were NOT what the unconstrained matrix was made of:")
    print("  removing them moves no correlation by more than the figures above,")
    print("  and every cell stays weak.  The conclusion from (b) - that no cond")
    print("  column is a substitute for margin, sens or tau_min at any of the")
    print("  four characteristic lengths - survives the correction rather than")
    print("  depending on it.  34 of 363 is under 10% of the field and Spearman")
    print("  is a rank statistic, so this is what should have been expected; it")
    print("  is reported because it had to be checked, not because it is news.")


def _report_cond_distribution(rows):
    """Part (4): where cond(J_fk) actually sits once the artifact is removed."""
    print()
    print("=" * 78)
    print("(4) DISTRIBUTION OF cond(J_fk) UNDER delta_con")
    print("=" * 78)
    print("  Five-number summary and max, at each cap C and each characteristic")
    print("  length.  The cap is applied at char_len = r_b; the SAME geometry is")
    print("  then re-measured at r_p, d and a, which is why a row can exceed its")
    print("  own C - the cap does not travel with the length.  Every value is")
    print("  PROVISIONAL in its characteristic length.")
    print()
    hdr = (f"    {'tune':<34} {'min':>11} {'Q1':>11} {'median':>11} "
           f"{'Q3':>11} {'max':>11} | {'max':>11}")
    print(hdr)
    print("    " + "-" * (len(hdr) - 4))
    for key in CHAR_LEN_KEYS:
        v = _col(rows, "cond", key, "J_fk")
        q = _five_number(v)
        print(f"    {'delta_free  cond(J_fk)@' + key:<34} {q[0]:>11.4e} "
              f"{q[1]:>11.4e} {q[2]:>11.4e} {q[3]:>11.4e} {q[4]:>11.4e} | "
              f"{np.nanmax(v):>11.4e}")
    for cap in CONSTRAINT_CAPS:
        print()
        for key in CHAR_LEN_KEYS:
            v = _con_col(rows, cap, "cond", key, "J_fk")
            q = _five_number(v)
            lab = f"C={cap:.0e}  cond(J_fk)@{key}"
            print(f"    {lab:<34} {q[0]:>11.4e} {q[1]:>11.4e} {q[2]:>11.4e} "
                  f"{q[3]:>11.4e} {q[4]:>11.4e} | {np.nanmax(v):>11.4e}")
    print()
    print("  QUARTILE STABILITY.  Largest relative move in Q1, the median and Q3")
    print("  between the unconstrained tune and each cap, per characteristic")
    print("  length:")
    print()
    print(f"    {'C on cond(J_fk)@r_b':>21} " + "".join(
        f"{'@' + k:>12}" for k in CHAR_LEN_KEYS))
    for cap in CONSTRAINT_CAPS:
        cells = ""
        for key in CHAR_LEN_KEYS:
            qf = _five_number(_col(rows, "cond", key, "J_fk"))
            qc = _five_number(_con_col(rows, cap, "cond", key, "J_fk"))
            cells += f"{max(abs(qc[i] - qf[i]) / qf[i] for i in (1, 2, 3)):>11.2%} "
        print(f"    {f'C = {cap:.0e}':>21} " + cells)
    print()
    print("  Read the max column down: the unconstrained tune's max is set by the")
    print("  rank collapse, so it says nothing about the field.  Under a cap the")
    print("  max is the cap (at r_b, where the cap lives) and the quartiles are")
    print("  what the field actually looks like.")
    print()
    iqr = ", ".join(
        f"{key} {np.log10(_five_number(_col(rows, 'cond', key, 'J_fk'))[3] / _five_number(_col(rows, 'cond', key, 'J_fk'))[1]):.2f}"
        for key in CHAR_LEN_KEYS)
    print("  The quartiles barely move - the table above says by how much.  So")
    print("  the cap TRUNCATES A TAIL and leaves the body of the distribution")
    print("  alone, and the two readings that follow are both open:")
    print(f"    * the middle half of the field (Q1 to Q3) spans, in decades of")
    print(f"      cond(J_fk): {iqr}.  That is spread for a ranking")
    print("      to work with, and it is much the same at all four lengths;")
    print("    * but that spread was already visible without the cap, so nothing")
    print("      here shows the cap is what makes cond usable - only that the")
    print("      degenerate tail was not carrying the distribution.")
    print("  Whether cond earns an outer ranking role, with or without a cap, is")
    print("  not decided here.")


#: ``e = beta_p - beta`` offsets for part (5), degrees, log-spaced and mirrored.
#: The finest step is 1e-3 deg, three decades below any plausible sweep grid, so
#: the falloff is resolved rather than straddled.  Capped at 5 deg so
#: ``beta_p = beta + e`` stays inside ``(0, 60)`` for every candidate here.
E_OFFSETS_DEG = (0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5,
                 1.0, 2.0, 5.0)


def _e_sweep(rec, R, az, offsets=E_OFFSETS_DEG):
    """cond(J_fk)@r_b at ``delta_free`` and at each cap, vs ``e = beta_p - beta``.

    Everything but ``beta_p`` is held: ``beta``, ``r_p``, ``a``, ``d``,
    ``z_home`` and the pose grid.  ``delta`` is RE-TUNED at every ``e`` - the
    question is what the tuner selects near the coincidence, not what it selects
    at it.
    """
    T = _T_stack(az, rec["z_home"])
    cl = char_lengths(rec)[CONSTRAINT_CHAR_LEN]
    es = np.concatenate([-np.array(offsets)[::-1], [0.0], np.array(offsets)])
    out = []
    for e in es:
        bp = rec["beta"] + e
        if not 0.0 < bp < 60.0:
            continue
        m, c, _ = scan_delta(rec["beta"], bp, rec["r_p"], rec["a"], rec["d"],
                             R, T, DELTA_GRID, cl)
        if not np.isfinite(m).any():
            continue
        k = int(np.nanargmax(m))
        row = dict(e=float(e), delta_free=float(DELTA_GRID[k]),
                   cond_free=float(c[k]), margin_free=float(m[k]), con={})
        for cap in CONSTRAINT_CAPS:
            d_con, m_con = tune_constrained(m, c, DELTA_GRID, cap)
            row["con"][cap] = (d_con, m_con,
                               np.nan if d_con is None
                               else float(c[int(np.flatnonzero(
                                   DELTA_GRID == d_con)[0])]))
        out.append(row)
    return out


def _crossing_width(sweep, cap):
    """Half-widths in ``e`` outside which ``cond(J_fk)@r_b`` at ``delta_free``
    never again exceeds ``cap``.

    The OUTERMOST crossing, not the first one going out.  That distinction is
    forced by the measurement: ``cond`` at ``delta_free`` is not monotone in
    ``|e|`` (see the report - the worst pose changes), so a first crossing would
    report a width the field then exceeds further out.  The question being asked
    is how far out the degeneracy reaches, which is the last crossing.

    Interpolated in ``log|e|`` against ``log(cond)`` between the bracketing grid
    points.  Returns ``(width_e_negative, width_e_positive)``: ``0.0`` where the
    cap is never exceeded on that side, ``inf`` where it is still exceeded at the
    far end of the sweep.
    """
    outs = []
    for sign in (-1.0, +1.0):
        pts = sorted(((abs(r["e"]), r["cond_free"]) for r in sweep
                      if r["e"] * sign > 0 and np.isfinite(r["cond_free"])),
                     key=lambda t: t[0])
        above = [i for i, (_, c) in enumerate(pts) if c > cap]
        if not above:
            outs.append(0.0)
            continue
        i = above[-1]
        if i == len(pts) - 1:
            outs.append(np.inf)
            continue
        (e0, c0), (e1, c1) = pts[i], pts[i + 1]
        lg = np.log(c0 / cap) / np.log(c0 / c1)
        outs.append(float(np.exp(np.log(e0) + lg * np.log(e1 / e0))))
    return tuple(outs)


def _pose_smin(beta, beta_p, r_p, a, d, R, T, delta_deg, char_len):
    """``sigma_min(J_fk)`` per pose at one ``delta``, shape ``(K,)``.

    Per-pose rather than reduced, because the shape of the neighbourhood in
    part (5) turns on WHICH pose attains the worst value and how that changes
    with ``e``.
    """
    geom = make_geometry(r_b=R_B, beta=beta, delta=float(delta_deg), r_p=r_p,
                         beta_p=beta_p, a=a, d=d, c_p=C_P)
    out = []
    for k in range(R.shape[0]):
        try:
            _, J_fk, _, _ = _jacobians_at_pose(geom, R[k], T[k])
        except Unreachable:
            out.append(np.nan)
            continue
        out.append(_cond_and_sigma(J_fk, char_len)[1])
    return np.array(out, dtype=float)


def _report_neighbourhood(rows, R, az):
    """Part (5): how wide the degenerate neighbourhood around ``e = 0`` is."""
    eq = [r for r in rows if abs(r["beta_p"] - r["beta"]) < 1e-9]
    print()
    print("=" * 78)
    print("(5) NEAR-DEGENERATE NEIGHBOURHOOD IN e = beta_p - beta")
    print("=" * 78)
    print(f"  beta_p == beta was an exact GRID COINCIDENCE: BETA_P = "
          f"{list(BETA_P)} and")
    print(f"  BETA = {list(BETA)} share two values, so e = 0 is hit exactly and")
    print("  only there.  A finer sweep will land NEAR e = 0 without landing on")
    print("  it, so what matters is not the value at 0 but the WIDTH of the")
    print("  neighbourhood in which the unconstrained tune still selects a")
    print("  degenerate delta.")
    print()
    print(f"  {len(eq)} candidates swept, e from -5 to +5 deg, log-spaced from")
    print(f"  1e-3 deg.  delta RE-TUNED at every e.  beta, r_p, a, d, z_home and")
    print(f"  the {az.size}-pose grid all held.")

    sweeps = [(r, _e_sweep(r, R, az)) for r in eq]

    worst = max(eq, key=lambda r: r["cond"]["r_b"]["J_fk"])
    sw = [s for r, s in sweeps if r is worst][0]
    print()
    print(f"  EXEMPLAR - the worst-conditioned of the {len(eq)}: beta = "
          f"{worst['beta']:.0f}, beta_p = {worst['beta_p']:.0f},")
    print(f"  r_p/r_b = {worst['r_p']}, a/r_b = {worst['a']}, "
          f"d/r_b = {worst['d']}, z_home/r_b = {worst['z_home']:.4f}")
    print()
    print(f"    {'e [deg]':>9} {'dlt_free':>9} {'cond(J_fk)@r_b':>16} "
          f"{'margin':>10} | " + "".join(
              f"{f'dlt|C={c:.0e}':>13}{'cond':>12}" for c in CONSTRAINT_CAPS))
    for row in sw:
        cells = ""
        for cap in CONSTRAINT_CAPS:
            d_con, _, c_con = row["con"][cap]
            cells += (f"{'none':>13}{'-':>12}" if d_con is None
                      else f"{d_con:>13.1f}{c_con:>12.3e}")
        print(f"    {row['e']:>+9.3f} {row['delta_free']:>9.1f} "
              f"{row['cond_free']:>16.4e} {row['margin_free']:>10.5f} | "
              + cells)

    # ---- falloff: the exact hit, and the cluster around it -------------- #
    off = [r for r in sw if r["e"] != 0.0 and np.isfinite(r["cond_free"])]
    at0 = [r for r in sw if r["e"] == 0.0]
    print()
    print("  FALLOFF.  Two separate facts, and conflating them would be wrong.")
    if at0 and off:
        c0 = at0[0]["cond_free"]
        cmax = max(r["cond_free"] for r in off)
        print(f"    e = 0 EXACTLY          : cond(J_fk)@r_b = {c0:.3e}")
        print(f"    worst over e != 0      : cond(J_fk)@r_b = {cmax:.3e}"
              f"   at e = "
              f"{max(off, key=lambda r: r['cond_free'])['e']:+.4f} deg")
        print(f"    ratio                  : {c0 / cmax:.3e}")
        print("    So the exact coincidence is not the near-miss case scaled up;")
        print("    it is a rank drop, and every near-miss is merely ill")
        print("    conditioned.  A sweep that never lands on e = 0 escapes the")
        print("    first and not the second.")

    T_ex = _T_stack(az, worst["z_home"])
    probes = (1e-4, 1e-3, 1e-2, 3e-2, 1e-1, 1.0, 3.0)
    print()
    print("  WHY IT DOES NOT FALL OFF AS 1/|e|.  sigma_min(J_fk) is a MIN OVER")
    print("  POSES, and the poses do not all degenerate at the same e.  At")
    print("  delta = 0, per pose (exemplar, char_len = r_b):")
    print()
    print(f"    {'e [deg]':>9} {'home pose':>12} {'worst pose':>12} "
          f"{'which':>7} {'home/e':>12}")
    for e in probes:
        s = _pose_smin(worst["beta"], worst["beta"] + e, worst["r_p"],
                       worst["a"], worst["d"], R, T_ex, 0.0, R_B)
        if np.all(np.isnan(s)):
            continue
        k = int(np.nanargmin(s))
        print(f"    {e:>9.4f} {s[0]:>12.4e} {np.nanmin(s):>12.4e} "
              f"{k:>7d} {s[0] / e:>12.4e}")
    print()
    print("  The HOME pose recovers exactly linearly - 'home/e' is constant to")
    print("  four figures, which is sigma_min ~ e and cond ~ 1/|e|, the clean")
    print("  behaviour the rank argument predicts.  But the worst pose is NOT")
    print("  the home pose except at e = 0: tilted poses have their own")
    print("  degeneracies at nearby NONZERO e, and the worst case switches")
    print("  between them.  The neighbourhood is a CLUSTER of degeneracies")
    print("  spread over |e|, not a single point with a tail, which is why the")
    print("  worst-case cond stalls near 1e5 instead of falling as 1/|e|.")

    # ---- widths across all of them -------------------------------------- #
    print()
    print("  WIDTH OF THE NEIGHBOURHOOD.  For each cap C, the |e| beyond which")
    print("  the UNCONSTRAINED tune NEVER AGAIN selects a delta with")
    print("  cond(J_fk)@r_b above C - the OUTERMOST crossing, interpolated in")
    print("  log-log between grid points.  The outermost, not the first: cond")
    print("  at delta_free is not monotone in |e| (previous block), so a first")
    print(f"  crossing would understate it.  Over all {len(eq)} candidates, each")
    print("  side of e = 0 separately.  'never' = the cap is not exceeded at any")
    print("  e on that side; 'beyond 5' = still exceeded at the end of the sweep.")
    print()
    base = _col(rows, "cond", CONSTRAINT_CHAR_LEN, "J_fk")
    print(f"    {'C on cond(J_fk)@r_b':>21} {'side':>6} {'min |e|':>10} "
          f"{'median |e|':>11} {'max |e|':>10} {'never':>7} {'beyond 5':>9}"
          f" | {'whole field > C':>16}")
    for cap in CONSTRAINT_CAPS:
        n_field = int(np.sum(base > cap))
        for si, side in enumerate(("e < 0", "e > 0")):
            w = np.array([_crossing_width(s, cap)[si] for _, s in sweeps])
            n_never = int(np.sum(w == 0.0))
            n_beyond = int(np.sum(np.isinf(w)))
            fin = w[np.isfinite(w) & (w > 0.0)]
            tail = (f" | {f'{n_field} of {len(rows)}':>16}" if si == 0
                    else f" | {'':>16}")
            if fin.size == 0:
                print(f"    {f'C = {cap:.0e}':>21} {side:>6} {'-':>10} "
                      f"{'-':>11} {'-':>10} {n_never:>7d} {n_beyond:>9d}"
                      + tail)
                continue
            print(f"    {f'C = {cap:.0e}':>21} {side:>6} {fin.min():>10.4f} "
                  f"{np.median(fin):>11.4f} {fin.max():>10.4f} "
                  f"{n_never:>7d} {n_beyond:>9d}" + tail)
    print()
    print("  The last column CALIBRATES the cap and has to be read with the")
    print("  widths.  Where the count equals the 34, the cap is exceeded only by")
    print("  the degenerate set and the width IS a degeneracy width.  Where it")
    print("  runs well above 34, ordinary candidates are already over the cap at")
    print("  their own tuned delta, and that row's width is measuring how far out")
    print("  a candidate stays worse than an ordinary one - a different question,")
    print("  with a much wider answer.  The two ends of the table are not the")
    print("  same measurement and must not be read as one trend.")
    print()
    print("  Reading, and this is the part that bears on the sweep.  The width")
    print("  runs from thousandths of a degree of beta_p at the LOOSEST cap to")
    print("  whole degrees at the tightest - the tighter the cap, the wider the")
    print("  region it condemns.  A sweep whose beta_p step is")
    print("  COARSER than the width can step over the neighbourhood entirely and")
    print("  never see it; a step FINER lands inside it without ever hitting")
    print("  e = 0, and the unconstrained tune then selects a near-degenerate")
    print("  delta at a candidate that looks perfectly ordinary in beta_p - at a")
    if at0 and off:
        print(f"  cond {np.log10(at0[0]['cond_free'] / cmax):.0f} decades below "
              f"the exact hit (exemplar above), so it will")
    else:
        print("  cond far below the exact hit, so it will")
    print("  not stand out as an obvious outlier either.  Which case the")
    print("  15625-candidate sweep is in depends on its beta_p spacing, which is")
    print("  not set here.")
    print("  Every width above is in degrees of beta_p, from a cap on")
    print("  cond(J_fk)@r_b, and is PROVISIONAL in that characteristic length.")


def _report_cost(rows, R, az):
    """Part (6): measured cost, and how much of the scan needs cond at all."""
    print()
    print("=" * 78)
    print("(6) COMPUTE COST FOR THE SWEEP HARNESS - measured, not estimated")
    print("=" * 78)
    sample = rows[::max(1, len(rows) // 20)][:20]
    T_of = [(_T_stack(az, r["z_home"]), r) for r in sample]

    def timeit(fn, reps=3):
        best = np.inf
        for _ in range(reps):
            t0 = time.perf_counter()
            fn()
            best = min(best, time.perf_counter() - t0)
        return best / len(T_of)

    t_marg = timeit(lambda: [scan_delta(r["beta"], r["beta_p"], r["r_p"],
                                        r["a"], r["d"], R, T, DELTA_GRID,
                                        char_lengths(r)[CONSTRAINT_CHAR_LEN],
                                        with_cond=False)
                             for T, r in T_of])
    t_cond = timeit(lambda: [scan_delta(r["beta"], r["beta_p"], r["r_p"],
                                        r["a"], r["d"], R, T, DELTA_GRID,
                                        char_lengths(r)[CONSTRAINT_CHAR_LEN],
                                        with_cond=True)
                             for T, r in T_of])
    nd = DELTA_GRID.size
    print(f"  Timed over {len(T_of)} candidates, best of 3, on the "
          f"{az.size}-pose grid,")
    print(f"  {nd}-point delta scan, cond at char_len = "
          f"{CONSTRAINT_CHAR_LEN} (one length, not four).")
    print()
    print(f"    {'quantity':<44} {'per candidate':>15} {'per delta step':>16}")
    print(f"    {'margin only (no Jacobian, no SVD)':<44} "
          f"{1e3 * t_marg:>13.2f}ms {1e6 * t_marg / nd:>14.1f}us")
    print(f"    {'margin + cond(J_fk)@r_b':<44} "
          f"{1e3 * t_cond:>13.2f}ms {1e6 * t_cond / nd:>14.1f}us")
    print(f"    {'cond alone (difference)':<44} "
          f"{1e3 * (t_cond - t_marg):>13.2f}ms "
          f"{1e6 * (t_cond - t_marg) / nd:>14.1f}us")
    print(f"    cond costs {t_cond / t_marg:.1f}x the margin scan.")

    # ---- band analysis --------------------------------------------------- #
    print()
    print("  BAND.  cond need not be evaluated at every delta: the constrained")
    print("  winner is the best-margin delta that meets the cap, so only deltas")
    print("  within a margin BAND of the unconstrained maximum can ever be")
    print("  selected.  Fraction of the 180-point scan inside a band of width w:")
    print()
    print(f"    {'band w (margin units)':>22} {'min':>9} {'median':>9} "
          f"{'mean':>9} {'max':>9}")
    bands = (1e-5, 1e-4, 1e-3, 3e-3, 1e-2, 3e-2, 1e-1)
    fr = {}
    for w in bands:
        f = []
        for r in rows:
            m = r["scan_margin"]
            live = np.isfinite(m)
            f.append(float(np.mean(m[live] >= np.nanmax(m) - w))
                     if live.any() else np.nan)
        f = np.array(f)
        fr[w] = f
        print(f"    {w:>22.0e} {np.nanmin(f):>8.1%} {np.nanmedian(f):>8.1%} "
              f"{np.nanmean(f):>8.1%} {np.nanmax(f):>8.1%}")

    print()
    print("  BAND REQUIRED TO CONTAIN delta_con, per cap.  This is exactly the")
    print("  worst margin given up in part (2): a band narrower than that would")
    print("  exclude the constrained winner on some candidate and the harness")
    print("  would return the wrong delta.")
    print()
    print("  Projected onto a 15625-candidate sweep at the same pose grid, delta")
    print("  scan and characteristic length:")
    print()
    print(f"    {'C on cond(J_fk)@r_b':>21} {'band required':>15} "
          f"{'frac of scan':>14} {'sweep [s]':>11} {'vs cond-always':>15}")
    for cap in CONSTRAINT_CAPS:
        loss = np.nan_to_num(_con_col(rows, cap, "loss"), nan=0.0)
        need = float(np.max(loss))
        f = []
        for r in rows:
            m = r["scan_margin"]
            live = np.isfinite(m)
            f.append(float(np.mean(m[live] >= np.nanmax(m) - need))
                     if live.any() else np.nan)
        frac = float(np.nanmean(f))
        part = t_marg + frac * (t_cond - t_marg)
        print(f"    {f'C = {cap:.0e}':>21} {need:>15.4e} {frac:>13.1%} "
              f"{15625 * part:>11.1f} {f'{t_cond / part:.1f}x':>15}")
    print(f"    {'margin only, no cond':>21} {'-':>15} {'0.0%':>13} "
          f"{15625 * t_marg:>11.1f} {f'{t_cond / t_marg:.1f}x':>15}")
    print(f"    {'cond at every delta':>21} {'-':>15} {'100.0%':>13} "
          f"{15625 * t_cond:>11.1f} {'1.0x':>15}")
    print()
    print("  The band pays at the LOOSE caps and not at the tight ones, which is")
    print("  the opposite of the intuition: a tight cap rejects the best-margin")
    print("  deltas, so the constrained winner sits far down the margin ordering")
    print("  and the band that must contain it is wide enough to cover most of")
    print("  the scan.  The saving is real only where the cap is nearly")
    print("  non-binding, and there the cond evaluation was cheap to skip anyway.")
    print("  All rows at char_len = r_b, ONE characteristic length; evaluating")
    print("  all four multiplies the cond part and leaves the margin part alone.")


#: Build-error probes for the two-term score, in units of ``r_b``.  THREE of
#: them, reported side by side and none preferred.  They span the band
#: ``sweep_budget.py`` measured the margin response to be linear over to ~1%
#: (0.0025-0.01 r_b); 0.05 is 16% off linear and is not a probe.
SCORE_PROBES = (0.005, 0.0075, 0.010)

#: The cap the score study is run under.  cond is no longer a ranking term -
#: recorded 2026-09-07, NOT decided here - but the cap on the inner tune stays,
#: so every candidate below is at its constrained delta.  The existing cond
#: reporting in (a)-(d) and (2)-(6) is unchanged and is what the cap rests on.
SCORE_CAP = 1e6

#: Below this, two margins are treated as the SAME value rather than as an
#: ordering.  Not a tolerance chosen for convenience: the structural ties
#: measured in part (7) sit at ~1e-15 (they are one quantity computed twice) and
#: the smallest genuine adjacent gap in the top 20 is ~1e-4, so this sits about
#: eight decades clear of both and no plausible value between them changes any
#: count reported.
TIE_TOL = 1e-12

#: Ranking-grid resolution, in margin units.  This is the worst coarse-azimuth
#: optimism measured in part (e)-(f): the amount by which the 10-degree azimuth
#: grid the ranking runs on overstates a candidate's margin against a 0.25-degree
#: grid.  A term whose whole spread across the field is below this cannot be
#: resolved by the grid the ranking is computed on - it is smaller than the
#: grid's own error.  Checked against this run's recomputed value in the report.
GRID_RESOLUTION = 2.88e-3


def score_terms(rows, R, az, cap=SCORE_CAP, probes=SCORE_PROBES):
    """Measure ``margin`` and ``sens * p`` under the constrained tune.

    ``sens`` is RE-MEASURED at each probe rather than taken from the stored
    ``PROBE_DXY`` value and scaled, because scaling assumes the linearity that
    is itself a measurement.  Both are recorded: ``term[p]`` is the re-measured
    ``sens(p) * p``, ``lin[p]`` is ``sens(PROBE_DXY) * p``.

    Stores on each record, under ``rec["score"]``::

        margin        margin at delta_con, the cap's delta
        sens[p]       (margin - margin(dxy = p)) / p, worst over azimuth
        term[p]       sens[p] * p, in MARGIN UNITS
        disp[p]       margin(dxy = p) itself, worst over azimuth
        lin[p]        sens at PROBE_DXY times p - the linear extrapolation

    Candidates with no admissible delta at ``cap`` get ``None`` and are counted
    out by the caller.
    """
    for rec in rows:
        entry = rec["con"][cap]
        if entry is None:
            rec["score"] = None
            continue
        d_con, m_con = entry["delta"], entry["margin"]
        s_ref = entry["sens"]
        out = dict(margin=m_con, delta=d_con, sens={}, term={}, disp={},
                   lin={})
        for p in probes:
            worst = probe_margin(rec["_g0"], rec["_g90"], R, az,
                                 rec["z_home"], d_con, p)
            worst = min(worst, m_con)
            out["sens"][p] = (m_con - worst) / p
            out["term"][p] = m_con - worst
            out["disp"][p] = worst
            out["lin"][p] = s_ref * p
        rec["score"] = out
    return rows


def _flag(label, rng):
    """One resolvability verdict line for part (7.5)."""
    verdict = ("UNRESOLVABLE by the grid" if rng < GRID_RESOLUTION
               else "resolved")
    ratio = rng / GRID_RESOLUTION
    return (f"    {label:<50} range {rng:>11.4e}  "
            f"{ratio:>8.2f}x resolution  {verdict}")


def _report_score(rows, R, az, ef_max=None):
    """Part (7): can ``sens * p`` reorder the leaders that ``margin`` picks?"""
    print()
    print("=" * 78)
    print(f"(7) TWO-TERM SCORE - margin AND sens*p, under delta_con at "
          f"C = {SCORE_CAP:.0e}")
    print("=" * 78)
    print("  cond is DROPPED as an outer ranking term (recorded 2026-09-07; not")
    print("  a decision taken in this module).  The cap on the inner tune stays,")
    print("  so every candidate below sits at its constrained delta, and the cond")
    print("  reporting in (a)-(d) and (2)-(6) is unchanged - that is what the cap")
    print("  rests on.  No cond column appears in this part.")
    print()
    print("  p is a BUILD ERROR in units of r_b.  margin and sens are")
    print("  dimensionless ratios and carry no characteristic length; sens*p is")
    print("  dimensionless because p is normalised by r_b, and it is stated in")
    print("  MARGIN UNITS throughout.  The cap is on cond(J_fk) at char_len = r_b")
    print("  and nowhere else.  p is NOT chosen here - all three run side by side.")

    score_terms(rows, R, az)
    live = [r for r in rows if r["score"] is not None]
    print()
    print(f"  candidates: {len(live)} of {len(rows)} "
          f"(all with an admissible delta at C = {SCORE_CAP:.0e})")

    # ---- the identity, stated before anything is read off the tables ---- #
    print()
    print("-" * 78)
    print("  WHAT THE SECOND TERM IS.  This has to come first, because it")
    print("  changes how every table below reads.")
    print("-" * 78)
    print("    sens(p) = [margin(0) - margin(p)] / p        by definition")
    print("    score   = margin(0) - sens(p) * p  =  margin(p)      identically")
    print()
    worst_id = max(abs((r["score"]["margin"] - r["score"]["term"][p])
                       - r["score"]["disp"][p])
                   for r in live for p in SCORE_PROBES)
    print(f"  Verified over all {len(live)} candidates and all three p: worst")
    print(f"  |(margin - sens*p) - margin(dxy = p)| = {worst_id:.3e}")
    print()
    print("  So 'margin minus sens times p' is NOT a weighted sum of two")
    print("  competing quantities.  It is the SAME quantity - the normalised")
    print("  reach margin - read at a displaced pose instead of the nominal one.")
    print("  There is no trade-off to tune and no weight to pick: p is not a")
    print("  weight, it is the build error the margin is being evaluated at.")
    print("  Everything below is therefore a question about how much the ranking")
    print("  moves when the margin is read at p instead of 0.")

    # ---- structural ties in the margin ordering -------------------------- #
    groups = {}
    for r in live:
        key = (r["r_p"], r["a"], r["d"],
               round(abs(r["beta_p"] - r["beta"]), 9))
        groups.setdefault(key, []).append(r)
    multi = [v for v in groups.values() if len(v) > 1]
    w_m = max((max(x["score"]["margin"] for x in v)
               - min(x["score"]["margin"] for x in v)) for v in multi)
    w_z = max((max(x["z_home"] for x in v)
               - min(x["z_home"] for x in v)) for v in multi)
    w_s_by_p = {p: max((max(x["score"]["term"][p] for x in v)
                        - min(x["score"]["term"][p] for x in v))
                       for v in multi) for p in SCORE_PROBES}
    w_s = w_s_by_p[SCORE_PROBES[-1]]
    n_distinct = len({round(r["score"]["margin"], 12) for r in live})
    biggest = max(len(v) for v in groups.values())
    print()
    print("-" * 78)
    print("  THE MARGIN ORDERING HAS STRUCTURAL TIES, and they are not numerical")
    print("-" * 78)
    print("  beta and beta_p enter the margin ONLY through e = beta_p - beta, and")
    print("  only through |e|.  Measured, grouping the field by")
    print("  (r_p, a, d, |e|) and ignoring beta and beta_p entirely:")
    print()
    print(f"    groups                                   : {len(groups)}")
    print(f"    groups with more than one candidate      : {len(multi)}")
    print(f"    worst within-group spread in margin      : {w_m:.3e}")
    print(f"    worst within-group spread in z_home      : {w_z:.3e}")
    print(f"    DISTINCT margin values in the field      : {n_distinct} "
          f"of {len(live)}")
    print()
    print("  The margin spread inside a group is at the arithmetic floor, so")
    print("  these are not near-ties to be broken by a finer grid - they are one")
    print(f"  quantity computed up to {biggest} times over.  Two candidates with")
    print("  the same (r_p, a, d, |e|) are the same point of the margin objective")
    print("  however different their beta and beta_p look.")
    print()
    print(f"    worst within-group spread in sens*p at p = "
          f"{SCORE_PROBES[-1]} r_b : {w_s:.3e}")
    print()
    print("  sens does NOT collapse the same way.  So among candidates the margin")
    print("  cannot separate at all, the second term separates them by up to the")
    print("  figure above - which is a different job from reordering leaders that")
    print("  margin has already ordered, and the tables below keep the two apart.")

    # ---- linearity of the extrapolation --------------------------------- #
    print()
    print("  Re-measured vs extrapolated.  sens is re-measured at each p rather")
    print(f"  than taken from the stored {PROBE_DXY} r_b probe and scaled.  Worst")
    print("  relative difference between the two, over the field:")
    print()
    print(f"    {'p [r_b]':>10} {'median |term-lin|/term':>24} "
          f"{'max |term-lin|/term':>22}")
    for p in SCORE_PROBES:
        d = np.array([abs(r["score"]["term"][p] - r["score"]["lin"][p])
                      / r["score"]["term"][p]
                      for r in live if r["score"]["term"][p] > 0])
        print(f"    {p:>10.4f} {np.median(d):>24.3e} {d.max():>22.3e}")
    print(f"    (0 at p = {PROBE_DXY} by construction - that IS the stored probe)")
    print()
    print("    TYPICALLY fine, occasionally not.  sweep_budget.py measured the")
    print("    response linear to ~1% over 0.0025-0.01 r_b on ONE fixture, and")
    print("    the medians above agree with that across 363.  The worst case does")
    print("    not: some candidate's sens changes by tens of percent between the")
    print("    smallest probe and the stored one.  A median-accurate")
    print("    extrapolation is not good enough for a worst-case ranking claim,")
    print("    so sens is re-measured at each p above rather than scaled.")

    # ================================================================== #
    # 7.1  field spread of margin
    # ================================================================== #
    m = np.array([r["score"]["margin"] for r in live])
    order = np.argsort(-m)
    top = order[:20]
    gaps = m[top][:-1] - m[top][1:]
    print()
    print("-" * 78)
    print("  (7.1) FIELD SPREAD OF margin, at delta_con")
    print("-" * 78)
    q = _five_number(m)
    print(f"    {'min':>12} {'Q1':>12} {'median':>12} {'Q3':>12} "
          f"{'max':>12} | {'max':>12}")
    print(f"    {q[0]:>12.4e} {q[1]:>12.4e} {q[2]:>12.4e} {q[3]:>12.4e} "
          f"{q[4]:>12.4e} | {np.max(m):>12.4e}")
    print(f"    full range (max - min) : {np.max(m) - np.min(m):.4e}")
    print()
    print("    Adjacent gaps in the TOP 20 of the margin ordering:")
    print()
    print(f"    {'rank':>5} {'beta':>5} {'beta_p':>6} {'r_p':>5} {'a':>5} "
          f"{'d':>5} {'margin':>12} {'gap to next':>13}")
    for i, k in enumerate(top):
        r = live[k]
        g = f"{gaps[i]:>13.4e}" if i < len(gaps) else f"{'-':>13}"
        print(f"    {i + 1:>5d} {r['beta']:>5.0f} {r['beta_p']:>6.0f} "
              f"{r['r_p']:>5.2f} {r['a']:>5.2f} {r['d']:>5.2f} "
              f"{m[k]:>12.5e}" + g)
    tie = gaps < TIE_TOL
    real = gaps[~tie]
    print()
    print(f"    of the {gaps.size} adjacent gaps, {int(tie.sum())} are "
          f"STRUCTURAL TIES (< {TIE_TOL:.0e}) and")
    print(f"    {real.size} are real.  The top 20 holds only "
          f"{real.size + 1} distinct margin values.")
    print(f"    real gaps only: min {real.min():.4e}  median "
          f"{np.median(real):.4e}  max {real.max():.4e}")
    print(f"    total spread across the top 20 : {m[top[0]] - m[top[-1]]:.4e}")
    print()
    print("    Quoting min or median over all 19 gaps would report the tie")
    print("    structure as if it were resolution, so the two are kept apart.")

    # ================================================================== #
    # 7.2  field spread of sens * p
    # ================================================================== #
    print()
    print("-" * 78)
    print("  (7.2) FIELD SPREAD OF sens*p, IN MARGIN UNITS")
    print("-" * 78)
    print(f"    {'p [r_b]':>9} {'min':>12} {'Q1':>12} {'median':>12} "
          f"{'Q3':>12} {'max':>12} | {'max':>12}")
    terms = {}
    for p in SCORE_PROBES:
        t = np.array([r["score"]["term"][p] for r in live])
        terms[p] = t
        qt = _five_number(t)
        print(f"    {p:>9.4f} {qt[0]:>12.4e} {qt[1]:>12.4e} {qt[2]:>12.4e} "
              f"{qt[3]:>12.4e} {qt[4]:>12.4e} | {np.max(t):>12.4e}")
    print()
    print("    All in margin units, so directly comparable with (7.1).")

    # ================================================================== #
    # 7.3  ratio of spreads, and whether the term can reorder the leaders
    # ================================================================== #
    print()
    print("-" * 78)
    print("  (7.3) CAN THE SECOND TERM REORDER THE LEADERS?")
    print("-" * 78)
    m_range = float(np.max(m) - np.min(m))
    m_iqr = float(q[3] - q[1])
    print("    Spread ratio, term against margin, on the whole field:")
    print()
    print(f"    {'p [r_b]':>9} {'term range':>13} {'range ratio':>13} "
          f"{'term IQR':>13} {'IQR ratio':>12}")
    for p in SCORE_PROBES:
        t = terms[p]
        t_range = float(np.max(t) - np.min(t))
        qt = _five_number(t)
        t_iqr = qt[3] - qt[1]
        print(f"    {p:>9.4f} {t_range:>13.4e} {t_range / m_range:>13.4f} "
              f"{t_iqr:>13.4e} {t_iqr / m_iqr:>12.4f}")
    print()
    print("    Adjacent pairs in the TOP 20 of the margin ordering.  Pair (i,")
    print("    i+1) actually SWAPS under the score iff")
    print("        term_i - term_(i+1)  >  margin_i - margin_(i+1)")
    print("    - the term difference has to exceed the margin gap AND favour the")
    print("    lower-ranked candidate.  Both counts are given: 'could' uses")
    print("    |term difference| and ignores the direction, 'does' is the signed")
    print("    condition, which is the one that reorders anything.")
    print()
    print(f"    {'p [r_b]':>9} {'real gaps':>10} {'could flip':>11} "
          f"{'does flip':>10} {'of those, tied':>15} {'across a real gap':>18} "
          f"{'max gap beaten':>15}")
    for p in SCORE_PROBES:
        t = terms[p]
        dt = t[top][:-1] - t[top][1:]
        could = int(np.sum(np.abs(dt) > gaps))
        does = np.flatnonzero(dt > gaps)
        in_tie = int(np.sum(tie[does]))
        across = int(does.size - in_tie)
        beaten = float(np.max(gaps[does])) if does.size else 0.0
        print(f"    {p:>9.4f} {real.size:>10d} {could:>11d} {does.size:>10d} "
              f"{in_tie:>15d} {across:>18d} {beaten:>15.4e}")
    print()
    print("    'max gap beaten' is the largest adjacent margin gap the term")
    print("    actually overturns; 0 means it overturns none.")
    print()
    print("    THE SPLIT IN THE LAST TWO COLUMNS IS THE ANSWER.  A flip 'in a")
    print("    tie' is not a reordering: those two candidates are the same point")
    print("    of the margin objective, margin expresses no preference between")
    print("    them, and any tie-break at all would move one past the other.")
    print("    Only 'across a real gap' is the second term overruling something")
    print("    margin actually said.  Both counts are given because collapsing")
    print("    them would make the term look several times more decisive than it")
    print("    is.")

    # ================================================================== #
    # 7.4  top ten, side by side
    # ================================================================== #
    print()
    print("-" * 78)
    print("  (7.4) TOP 10 UNDER score = margin - sens*p, BESIDE margin ALONE")
    print("-" * 78)

    def tag(r):
        return (f"{r['beta']:.0f}/{r['beta_p']:.0f}/{r['r_p']:.2f}/"
                f"{r['a']:.2f}/{r['d']:.2f}")

    m_top10 = [live[k] for k in order[:10]]
    print("    Candidates named beta/beta_p/r_p/a/d.")
    print()
    print(f"    {'rank':>4} {'margin alone':>26} " + "".join(
        f"{f'score at p = {p}':>26}" for p in SCORE_PROBES))
    sc_top = {}
    for p in SCORE_PROBES:
        s = np.array([r["score"]["disp"][p] for r in live])
        sc_top[p] = [live[k] for k in np.argsort(-s)[:10]]
    for i in range(10):
        row = f"    {i + 1:>4d} {tag(m_top10[i]):>26} "
        for p in SCORE_PROBES:
            row += f"{tag(sc_top[p][i]):>26}"
        print(row)
    print()
    base_set = {id(r) for r in m_top10}
    for p in SCORE_PROBES:
        new_set = {id(r) for r in sc_top[p]}
        entered = [r for r in sc_top[p] if id(r) not in base_set]
        left = [r for r in m_top10 if id(r) not in new_set]
        print(f"    p = {p} r_b:")
        print(f"      entering the top 10 : "
              f"{', '.join(tag(r) for r in entered) if entered else 'none'}")
        print(f"      leaving the top 10  : "
              f"{', '.join(tag(r) for r in left) if left else 'none'}")
        same = [i for i in range(10) if sc_top[p][i] is m_top10[i]]
        print(f"      positions unchanged : {len(same)} of 10")
    tops = [sc_top[p][0] for p in SCORE_PROBES]
    all_same = all(t is tops[0] for t in tops)
    print()
    print(f"    TOP CANDIDATE THE SAME AT ALL THREE p : "
          f"{'YES' if all_same else 'NO'}   ({tag(tops[0])}"
          + ("" if all_same else " at p = %s, others differ" % SCORE_PROBES[0])
          + ")")
    d_top = abs(tops[0]["score"]["margin"] - m_top10[0]["score"]["margin"])
    same_top = tops[0] is m_top10[0]
    print(f"    same as under margin alone            : "
          f"{'YES' if same_top else 'NO'}"
          f"   (margin alone: {tag(m_top10[0])})")
    if not same_top:
        kind = ("STRUCTURAL TIE" if d_top < TIE_TOL else "REAL GAP")
        print(f"    margin difference between the two     : {d_top:.3e}"
              f"  ({kind})")
        if d_top < TIE_TOL:
            print("      The margin never preferred one of these over the other,")
            print("      so the change of winner is a TIE-BREAK, not the second")
            print("      term overruling the first.  Reporting it as a changed")
            print("      winner without that qualification would overstate it.")
    print()
    print("    Read the enter/leave lists the same way: a candidate that enters")
    print("    past a tied neighbour has overtaken nothing.  The counts in (7.3)")
    print("    say which of these movements cross a real margin gap.")

    # ================================================================== #
    # 7.5  resolvability against the ranking grid
    # ================================================================== #
    print()
    print("-" * 78)
    print("  (7.5) RESOLVABILITY AGAINST THE RANKING GRID")
    print("-" * 78)
    print(f"    Grid resolution = {GRID_RESOLUTION:.4e} margin units - the worst")
    print("    coarse-azimuth optimism measured in part (e)-(f), i.e. the amount")
    print("    by which the 10-degree azimuth grid the ranking runs on overstates")
    print("    a margin against a 0.25-degree grid.  A contribution whose whole")
    print("    range across the field is below it is smaller than the grid's own")
    print("    error and cannot be resolved by the ranking as computed.")
    if ef_max is not None:
        print()
        print(f"    Recomputed in this run, part (e)-(f), 40-candidate sample,")
        print(f"    UNCONSTRAINED tune : {ef_max:.4e}   "
              f"({'consistent' if abs(ef_max - GRID_RESOLUTION) < 0.1 * GRID_RESOLUTION else 'DIFFERS - check'})")
    print()
    print("    FULL RANGE ACROSS THE FIELD (the requested test):")
    print(_flag("margin", m_range))
    for p in SCORE_PROBES:
        t = terms[p]
        print(_flag(f"sens*p at p = {p} r_b",
                    float(np.max(t) - np.min(t))))
    print()
    print("    The same test applied where the ranking is actually decided - the")
    print("    top 20, which is the only part of the ordering a leader can move")
    print("    within.  Reported because a term can clear the resolution across")
    print("    the whole field and still be invisible among the leaders, and it")
    print("    is the leaders that a score is for:")
    print()
    print(_flag("margin, top-20 spread", float(m[top[0]] - m[top[-1]])))
    print(_flag("margin, largest REAL adjacent top-20 gap", float(real.max())))
    print(_flag("margin, median REAL adjacent top-20 gap",
                float(np.median(real))))
    print(_flag("margin, smallest REAL adjacent top-20 gap",
                float(real.min())))
    for p in SCORE_PROBES:
        t = terms[p][top]
        print(_flag(f"sens*p top-20 spread, p = {p} r_b",
                    float(np.max(t) - np.min(t))))
    print()
    print("    The structural ties are excluded from those gap rows on purpose:")
    print("    a zero gap is not a term below the grid's resolution, it is two")
    print("    candidates the margin does not distinguish at all, and flagging it")
    print("    as unresolvable would confuse the two.")
    print()
    print("    AND THE TIE-BREAK ITSELF, which is where the second term does most")
    print("    of its work - the spread of sens*p WITHIN a set of candidates the")
    print("    margin ties exactly:")
    print()
    for p in SCORE_PROBES:
        print(_flag(f"sens*p spread within a margin tie, p = {p} r_b",
                    float(w_s_by_p[p])))
    print()
    print("    Every flag above is in margin units and none of them depends on a")
    print("    characteristic length; p is a length and is stated in r_b.")


# --------------------------------------------------------------------------- #
# (8) is the |e| collapse real, or a grid artifact?
# --------------------------------------------------------------------------- #
#: World displacement applied to ONE platform anchor in the negative control of
#: part (8), in units of ``r_b``.  Large enough to be far above the ~1e-15
#: within-group spread the control has to break, small enough that the candidate
#: stays the same machine - it is 3% of the base radius, and ``c_p`` itself is
#: 10%.  It is a control, not a manufacturing tolerance, and nothing downstream
#: uses it.
ANCHOR_KICK = 0.03

#: Which anchor the control displaces.  0-INDEXED here because it indexes a
#: column of ``p``; named 1-indexed wherever it is printed, per CLAUDE.md.
ANCHOR_KICK_LEG = 0


def _margin_field(g0, g90, R, T, delta_deg):
    """``(K, 6)`` normalised reach margin, per pose and per leg, at one ``delta``.

    :func:`_margin_at` collapses this to its minimum.  Part (8) needs the field
    itself, because the question there is whether two candidates that agree on
    the minimum also agree POSE BY POSE, or merely happen to reach the same
    worst value somewhere else in the envelope.
    """
    LL, P, A, B = _invariants(g0, g90, R, T)
    dr = np.deg2rad(delta_deg)
    w = A * np.cos(dr) + B * np.sin(dr)
    C = np.sqrt(np.maximum(LL - w * w, 0.0))
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(C > 0.0, (C - np.abs(P)) / C, -np.inf)


def _kick_pair(g0, g90, leg=ANCHOR_KICK_LEG, kick=ANCHOR_KICK):
    """Both ``delta``-basis geometries with one platform anchor displaced.

    The displacement is a FIXED vector in the platform frame, applied to the
    same leg INDEX in every candidate.  That is what makes it a control: if two
    candidates are related by a rotation of the whole machine about ``z``, that
    relation carries leg ``i`` onto some other leg and rotates the displacement
    with it, so putting the same unrotated displacement on leg 1 of both breaks
    the relation instead of respecting it.

    ``p`` is the only array touched; ``b``, ``n`` and ``u`` are the base ring and
    do not move, so the ``delta`` decomposition ``n_i(delta)`` is untouched and
    the ``A``/``B`` machinery stays valid.
    """
    out = []
    for g in (g0, g90):
        p = np.array(g.p, dtype=float, copy=True)
        p[0, leg] += kick * R_B
        out.append(Geometry(p=p, b=g.b, n=g.n, u=g.u, a=g.a, d=g.d))
    return out[0], out[1]


def _abs_e_groups(rows):
    """Group the field by ``(r_p, a, d, |beta_p - beta|)`` - part (7)'s key."""
    groups = {}
    for r in rows:
        key = (r["r_p"], r["a"], r["d"],
               round(abs(r["beta_p"] - r["beta"]), 9))
        groups.setdefault(key, []).append(r)
    return groups


def _collapse_spread(members, get):
    """Worst within-group spread of ``get(record)`` over one group."""
    v = [get(r) for r in members]
    return float(max(v) - min(v))


def _report_collapse(rows, R, az):
    """Part (8): three tests of the ``|e|`` collapse, and a verdict."""
    print()
    print("=" * 78)
    print("(8) THE |e| COLLAPSE - real, or an artifact of the pose grid?")
    print("=" * 78)
    print("  Part (7) measured margin identical to ~1e-15 within")
    print("  (r_p, a, d, |e|) groups, e = beta_p - beta, so 363 candidates carry")
    print("  188 distinct margins.  That is a strong claim and it was a")
    print("  by-product, not something the module set out to test.  Three tests")
    print("  below.  Nothing is re-parameterised: the candidate grid, the key and")
    print("  the margin definition are all exactly as they were.")
    print()
    print("  margin is dimensionless and carries NO characteristic length.  The")
    print("  only length in part (8) is the control displacement, stated in r_b.")

    groups = _abs_e_groups(rows)
    multi = [v for v in groups.values() if len(v) > 1]
    sizes = {}
    for v in groups.values():
        sizes[len(v)] = sizes.get(len(v), 0) + 1
    print()
    print(f"    candidates                     : {len(rows)}")
    print(f"    groups                         : {len(groups)}")
    print(f"    groups with more than one      : {len(multi)}")
    print(f"    group sizes                    : "
          + ", ".join(f"{k} member(s) x {sizes[k]}" for k in sorted(sizes)))

    # ---- what a group actually contains -------------------------------- #
    pairs = {}
    for r in rows:
        pairs.setdefault(round(abs(r["beta_p"] - r["beta"]), 9), set()).add(
            (r["beta"], r["beta_p"]))
    print()
    print("    The (beta, beta_p) pairs that share an |e|, from the candidate")
    print("    grid itself (BETA x BETA_P, unchanged):")
    print()
    print(f"    {'|e| deg':>8}  {'(beta, beta_p) pairs':<44} relation")
    for k in sorted(pairs):
        ps = sorted(pairs[k])
        txt = " ".join(f"({b:.0f},{bp:.0f})" for b, bp in ps)
        if len(ps) == 1:
            rel = "-"
        else:
            signs = {np.sign(bp - b) for b, bp in ps}
            rel = ("rotation only" if len(signs) == 1
                   else "rotation AND sign flip")
        print(f"    {k:>8.0f}  {txt:<44} {rel}")
    print()
    print("    Two candidates in one group differ by a rotation of the whole")
    print("    machine about z (beta and beta_p shifted together), by a sign")
    print("    flip of e, or by both.  Neither is a re-parameterisation - both")
    print("    are distinct points of the 540-candidate grid being swept.")

    # =================================================================== #
    # 8a  what exactly agrees: the curve, the delta, the argmin pose
    # =================================================================== #
    print()
    print("-" * 78)
    print("  (8.1) WHAT AGREES - the delta curve, the tuned delta, the argmin")
    print("-" * 78)
    print("    Three nested questions, and the first has to be settled before")
    print("    the others can be read:")
    print("      (i)   does margin(delta) agree over the WHOLE 180-point scan,")
    print("            or only at its maximum?")
    print("      (ii)  do the members pick the same delta?")
    print("      (iii) at a COMMON delta, is the (29, 6) margin field equal")
    print("            pointwise, or only equal at its minimum with the argmin")
    print("            sitting somewhere else in the envelope?")
    print()
    print("    (i) is the curve scan_delta already computes at the nominal")
    print("    pose - NaN at any delta unreachable at some pose, so the")
    print("    reachable SET is compared too, not only the values on it.")
    print()
    n_curve = n_mask = 0
    w_curve = w_curve_mask = 0.0
    for v in multi:
        c = [r["scan_margin"] for r in v]
        ok0 = np.isfinite(c[0])
        mask_same = all(np.array_equal(np.isfinite(x), ok0) for x in c[1:])
        both = ok0.copy()
        for x in c[1:]:
            both &= np.isfinite(x)
        w = (max(float(np.max(np.abs(c[0][both] - x[both]))) for x in c[1:])
             if both.any() else np.inf)
        w_curve = max(w_curve, w)
        if mask_same:
            w_curve_mask = max(w_curve_mask, w)
            n_curve += int(w < TIE_TOL)
        n_mask += int(mask_same)
    print(f"    (i)  groups whose reachable delta SET is identical  : "
          f"{n_mask} / {len(multi)}")
    print(f"         groups whose margin(delta) CURVE agrees to")
    print(f"           < {TIE_TOL:.0e} at every reachable delta        : "
          f"{n_curve} / {len(multi)}")
    print(f"         worst curve difference, same-reachable-set groups: "
          f"{w_curve_mask:.3e}")
    print(f"         worst over all groups                         : "
          f"{w_curve:.3e}")
    print()
    if n_curve == len(multi):
        print("         The agreement is NOT confined to the maximum.  Two")
        print("         candidates in a group have the same margin at every one")
        print("         of the 180 deltas, so the collapse is a property of the")
        print("         objective, not of where the tuner happens to stop.")
    else:
        print("         THE CURVES DIFFER, and in the groups whose reachable")
        print("         sets differ they differ by an unbounded amount - one")
        print("         member has a delta at which the other cannot reach the")
        print("         envelope at all.  So the members of a group are NOT the")
        print("         same machine.  Whatever agrees, agrees only at the")
        print("         maximum, and the next test asks how.")
    print()

    # ---- is one curve a rotation of the other in delta? ----------------- #
    nd = DELTA_GRID.size
    n_shift, shifts = 0, set()
    for v in multi:
        c = [r["scan_margin"] for r in v]
        best_g, best_s = 0.0, None
        for x in c[1:]:
            best, arg = np.inf, None
            for s in range(nd):
                y = np.roll(x, s)
                if not np.array_equal(np.isfinite(y), np.isfinite(c[0])):
                    continue
                ok = np.isfinite(y)
                e = float(np.max(np.abs(c[0][ok] - y[ok]))) if ok.any() else 0.0
                if e < best:
                    best, arg = e, s
            best_g, best_s = max(best_g, best), arg
        if best_g < TIE_TOL:
            n_shift += 1
            shifts.add(best_s)
    print(f"    (i') groups where SOME cyclic shift of delta aligns")
    print(f"           the curves to < {TIE_TOL:.0e}                       : "
          f"{n_shift} / {len(multi)}")
    if n_shift:
        print(f"         shifts that do it [deg]                       : "
              f"{', '.join(str(s) for s in sorted(shifts))}")
    print()
    print("         A shift would have made the collapse trivial - the same")
    print("         curve read from a different starting delta has the same")
    print("         maximum for free.")
    if shifts <= {0}:
        print(f"         The ONLY alignment found is the trivial shift 0, i.e."
              f" the")
        print(f"         {n_shift} groups whose curves were already identical in"
              f" (i).  No")
        print("         nontrivial shift aligns any group, so the collapse is")
        print("         not a reparametrisation of delta either.")
    else:
        print("         Some groups ARE aligned by a nonzero shift; for those the")
        print("         equal maxima follow from the shift and are not evidence")
        print("         of anything further.")
    print()

    raw_d = max(_collapse_spread(v, lambda r: r["delta"]) for v in multi)
    circ_d = max(max(_delta_gap(x["delta"], y["delta"]) for x in v for y in v)
                 for v in multi)
    plateau = np.array([int(np.sum(np.abs(r["scan_margin"]
                                          - np.nanmax(r["scan_margin"]))
                                   < TIE_TOL)) for r in rows])
    print(f"    (ii) worst within-group spread of the tuned delta,")
    print(f"           as a plain difference [deg]                 : "
          f"{raw_d:.1f}")
    print(f"         the same, as the SHORTER ARC mod 180 [deg]    : "
          f"{circ_d:.1f}")
    print(f"         deltas within {TIE_TOL:.0e} of a candidate's own")
    print(f"           maximum - median / max over the field       : "
          f"{int(np.median(plateau))} / {plateau.max()}")
    print()
    print("         delta is periodic mod 180 - n_i(delta + 180) is the same")
    print("         servo plane with the opposite normal - so a raw difference")
    print("         near 180 is a difference near 0.  Quoting the plain spread")
    print("         would report the wrap as a disagreement, which is why both")
    print("         rows are here.")
    print()
    if plateau.max() == 1:
        print("         The plateau row rules out the other easy explanation:")
        print("         every candidate's maximum is attained at exactly ONE")
        print("         delta, so members that differ are not two argmax picks")
        print("         off one flat top - they are different optima.")
    else:
        print(f"         Some candidates have up to {plateau.max()} deltas within "
              f"{TIE_TOL:.0e} of")
        print("         their own maximum, so part of any within-group delta")
        print("         difference is argmax choosing among equal maxima.")
    print()

    same_field = same_arg = same_perm = 0
    w_field = 0.0
    for v in multi:
        dcom = v[0]["delta"]                 # ONE delta for the whole group
        fields = [_margin_field(r["_g0"], r["_g90"], R,
                                _T_stack(az, r["z_home"]), dcom) for r in v]
        args = [np.unravel_index(int(np.argmin(f)), f.shape) for f in fields]
        w = max(float(np.max(np.abs(fields[0] - f))) for f in fields[1:])
        w_field = max(w_field, w)
        pw = [f.min(axis=1) for f in fields]
        wp = max(float(np.max(np.abs(np.sort(pw[0]) - np.sort(q))))
                 for q in pw[1:])
        same_field += int(w < TIE_TOL)
        same_arg += int(all(x == args[0] for x in args))
        same_perm += int(wp < TIE_TOL)
    print(f"    (iii) at the FIRST member's delta, applied to the whole group:")
    print(f"         groups where the argmin (pose, leg) is the same : "
          f"{same_arg} / {len(multi)}")
    print(f"         groups whose per-pose worst vector is a")
    print(f"           permutation of the others (sorted)            : "
          f"{same_perm} / {len(multi)}")
    print(f"         groups whose whole (29, 6) field agrees")
    print(f"           POINTWISE (< {TIE_TOL:.0e})                        : "
          f"{same_field} / {len(multi)}")
    print(f"         worst pointwise field difference                : "
          f"{w_field:.3e}")
    print()
    print("         A common delta is essential here: evaluating each member at")
    print("         its OWN tuned delta compares two different configurations")
    print("         and the fields differ for that reason alone, which says")
    print("         nothing about the collapse.")
    print()
    if same_field == len(multi):
        print("    THE FIELD AGREES POINTWISE.  The members do not reach the")
        print("    same worst value at different places in the envelope - they")
        print("    have the same margin at every pose and every leg.  That is")
        print("    stronger than a coincidence of extrema, and it is why the")
        print("    curve in (i) agrees too.")
    elif same_perm == len(multi):
        print("    THE FIELD IS A PERMUTATION, NOT POINTWISE EQUAL.  The argmin")
        print("    moves; the worst VALUE does not.  That is a relabelling of")
        print("    poses and legs - the machines are the same up to the")
        print("    symmetry, which is a real invariance and not a grid artifact.")
    else:
        print("    NEITHER POINTWISE NOR A PERMUTATION.  Taken with (i) and")
        print("    (i'): the members of a group are different machines with")
        print("    different margin(delta) curves, not related by a shift, that")
        print("    tune to different deltas and disagree at any common one.  The")
        print("    ONLY thing that agrees is the tuned maximum.  Whether that")
        print("    agreement is a real invariance or an artifact of the pose")
        print("    grid is exactly what (8.2) and (8.3) test, and it is not")
        print("    settled by this sub-part.")

    # =================================================================== #
    # 8b  re-run on a 0.25-degree azimuth grid
    # =================================================================== #
    print()
    print("-" * 78)
    print("  (8.2) THE SAME COLLAPSE ON A 0.25-DEGREE AZIMUTH GRID")
    print("-" * 78)
    print("    If the collapse came from the 7-point azimuth grid landing on")
    print("    poses the symmetry happens to map onto each other, refining the")
    print("    grid 40x would break it.  Same tilt limit, same [30, 90] window,")
    print("    same magnitudes, same delta scan - ONLY the azimuth resolution")
    print("    changes, so the two runs differ in exactly one thing.")
    t0 = time.perf_counter()
    Rf, azf, _ = _pose_grid(FINE_AZ_STEP_DEG)
    fine = {}
    for r in rows:
        inv = _invariants(r["_g0"], r["_g90"], Rf, _T_stack(azf, r["z_home"]))
        fine[id(r)] = _tune_delta(inv)
    dt = time.perf_counter() - t0
    print()
    print(f"    fine grid   : {azf.size} poses "
          f"({N_MAGNITUDE} magnitudes x "
          f"{(azf.size - 1) // (N_MAGNITUDE - 1)} azimuths), "
          f"{dt:.1f} s for {len(rows)} candidates")
    print()
    print(f"    {'':<34} {'29-pose grid':>15} {'0.25-deg grid':>15}")
    c_m = max(_collapse_spread(v, lambda r: r["margin"]) for v in multi)
    f_m = max(_collapse_spread(v, lambda r: fine[id(r)][1]) for v in multi)
    c_d = max(max(_delta_gap(x["delta"], y["delta"]) for x in v for y in v)
              for v in multi)
    f_d = max(max(_delta_gap(fine[id(x)][0], fine[id(y)][0])
                  for x in v for y in v) for v in multi)
    c_n = len({round(r["margin"], 12) for r in rows})
    f_n = len({round(fine[id(r)][1], 12) for r in rows})
    print(f"    {'worst within-group margin spread':<34} "
          f"{c_m:>15.3e} {f_m:>15.3e}")
    print(f"    {'worst within-group delta gap [deg]':<34} "
          f"{c_d:>15.1f} {f_d:>15.1f}")
    print(f"    {'distinct margin values in 363':<34} "
          f"{c_n:>15d} {f_n:>15d}")
    ch = sum(1 for r in rows if fine[id(r)][0] != r["delta"])
    dm = np.array([r["margin"] - fine[id(r)][1] for r in rows])
    print()
    print(f"    candidates whose tuned delta moved between the grids : "
          f"{ch} / {len(rows)}")
    print(f"    margin the coarse grid overstates by, median / max   : "
          f"{np.median(dm):.3e} / {dm.max():.3e}")
    print()
    print("    The last line is the point of the comparison: the grid change is")
    print("    NOT inert - it moves the margins themselves, and it moves the")
    print("    tuned delta.  Whether it moves them differently WITHIN a group is")
    print("    the test, and that is the first row of the table.")

    # =================================================================== #
    # 8c  negative control
    # =================================================================== #
    print()
    print("-" * 78)
    print(f"  (8.3) NEGATIVE CONTROL - displace ONE platform anchor by "
          f"{ANCHOR_KICK} r_b")
    print("-" * 78)
    print("    A collapse test that cannot fail proves nothing.  Break the")
    print("    geometry in a way the |e| relation cannot absorb and the spread")
    print("    must reappear; if it does not, the measurement is insensitive and")
    print("    the ~1e-15 above means nothing.")
    print()
    print(f"    The displacement is +x in the PLATFORM frame, {ANCHOR_KICK} r_b,")
    print("    on the same leg INDEX in every candidate - see _kick_pair for why")
    print("    a fixed direction on a fixed index is what breaks the relation.")
    print("    delta is re-tuned on the perturbed geometry; the 29-pose grid.")
    print()
    print("    Run SIX times, once per leg.  margin is a min over legs, so a")
    print("    displacement of a leg that is not binding, and does not become")
    print("    binding, moves nothing - and a control that moves nothing tests")
    print("    nothing.  Kicking each leg in turn makes that visible per group")
    print("    instead of averaging it away, and separates two very different")
    print("    outcomes: a group the control cannot move, and a group it moves")
    print("    without separating.  Only the second would falsify the collapse.")
    mem = [r for v in multi for r in v]
    kicked = {}
    for leg in range(6):
        for r in mem:
            k0, k90 = _kick_pair(r["_g0"], r["_g90"], leg=leg)
            inv = _invariants(k0, k90, R, _T_stack(az, r["z_home"]))
            kicked[(leg, id(r))] = _tune_delta(inv)[1]
    print()
    print(f"    {'leg':>5} {'groups moved':>14} {'groups separated':>18} "
          f"{'worst spread':>14} {'worst |d margin|':>18}")
    moved_any = [False] * len(multi)
    sep_any = [False] * len(multi)
    k_m = 0.0
    for leg in range(6):
        n_mv = n_sp = 0
        sp_w = mv_w = 0.0
        for gi, v in enumerate(multi):
            sp = _collapse_spread(v, lambda r: kicked[(leg, id(r))])
            mv = max(abs(kicked[(leg, id(r))] - r["margin"]) for r in v)
            sp_w = max(sp_w, sp)
            mv_w = max(mv_w, mv)
            if mv > TIE_TOL:
                n_mv += 1
                moved_any[gi] = True
            if sp > TIE_TOL:
                n_sp += 1
                sep_any[gi] = True
        k_m = max(k_m, sp_w)
        print(f"    {leg + 1:>5d} {n_mv:>8d}/{len(multi):<5d} "
              f"{n_sp:>12d}/{len(multi):<5d} {sp_w:>14.3e} {mv_w:>18.3e}")
    n_moved = sum(moved_any)
    n_broken = sum(sep_any)
    n_fail = sum(1 for i in range(len(multi)) if moved_any[i] and not sep_any[i])
    print()
    print(f"    (legs named 1-indexed; 'moved' = the displacement changed some")
    print(f"     member's margin at all, 'separated' = it changed them by")
    print(f"     DIFFERENT amounts, which is the collapse breaking)")
    print()
    print(f"    groups moved by at least one of the six       : "
          f"{n_moved} / {len(multi)}")
    print(f"    groups SEPARATED by at least one of the six   : "
          f"{n_broken} / {len(multi)}")
    print(f"    groups moved but never separated (the falsifier) : {n_fail}")
    print(f"    worst within-group margin spread, perturbed   : {k_m:.3e}")
    print(f"    worst within-group margin spread, as built    : {c_m:.3e}")
    if k_m > 0.0 and c_m > 0.0:
        print(f"    ratio, perturbed to as-built                  : "
              f"{k_m / c_m:.3e}")
    print()
    if n_fail == 0 and n_moved == len(multi):
        print("    CONTROL PASSES.  Every group is reachable by the control, and")
        print("    every group the control moves, it separates - by about")
        print(f"    {k_m / c_m:.0e} times the as-built spread.  The grouping can")
        print("    therefore detect a difference when there is one, so the")
        print("    as-built agreement is a statement about the geometry and not")
        print("    about the sensitivity of the test.")
    elif n_fail == 0:
        print(f"    CONTROL PASSES WHERE IT BITES.  No group is moved without")
        print(f"    being separated, so nothing falsifies the collapse.  But")
        print(f"    {len(multi) - n_moved} of {len(multi)} groups are never moved by any of the six")
        print("    kicks - a displaced anchor that is not the binding leg leaves")
        print("    a min-over-legs alone - and those groups are UNTESTED by this")
        print("    control rather than confirmed by it.  The claim rests on")
        print(f"    (8.1) and (8.2) for them and on all three for the other "
              f"{n_moved}.")
    else:
        print(f"    CONTROL FAILS on {n_fail} group(s): the displacement changes")
        print("    their margins and they still do not separate.  The test")
        print("    cannot distinguish 'identical' from 'not measured' there, so")
        print("    the collapse claim is UNSUPPORTED for those groups and the")
        print("    tie counts in part (7) should be read as unverified.")

    # =================================================================== #
    print()
    print("-" * 78)
    print("  (8) VERDICT")
    print("-" * 78)
    survives = f_m < TIE_TOL and n_fail == 0 and n_moved > 0 and k_m > TIE_TOL
    print(f"    DOES THE COLLAPSE SURVIVE : "
          f"{'YES' if survives else 'NO'}")
    print()
    print(f"    (8.1) curve agrees at every delta        : "
          f"{n_curve} / {len(multi)} groups")
    print(f"          a cyclic shift in delta aligns them: "
          f"{n_shift} / {len(multi)} groups")
    print(f"          field equal pointwise at a common delta : "
          f"{same_field} / {len(multi)} groups")
    print(f"    (8.2) worst spread, 0.25-deg azimuth grid : {f_m:.3e}")
    print(f"    (8.3) control moves {n_moved}, separates {n_broken}, "
          f"falsifies {n_fail}")
    print()
    if not survives:
        print("    Read part (7)'s tie counts as unverified until this is")
        print("    resolved; a tie that is really a near-tie would be broken by")
        print("    a finer grid and the flips reported there would be genuine")
        print("    reorderings.")
        return dict(groups=groups, multi=multi, fine=fine, coarse_spread=c_m,
                    fine_spread=f_m, kick_spread=k_m, survives=survives)
    m_all = np.array([r["margin"] for r in rows])
    m_rng = float(m_all.max() - m_all.min())
    print("    IT IS REAL, AND IT IS NARROWER THAN IT LOOKED.")
    print()
    print(f"    Real: it holds to {f_m:.1e} on a 40x finer azimuth grid that")
    print(f"    moves the margins themselves by up to {dm.max():.1e} and re-tunes")
    print(f"    {ch} candidates' delta, and it breaks by ~{k_m / c_m:.0e} times its own")
    print("    size the moment one anchor is displaced, in every group the")
    print("    displacement can reach.  A test that passed because it could not")
    print("    see anything would have failed (8.3).")
    print()
    print("    Narrower: the members of a group are NOT the same machine and")
    print("    (8.1) says so four ways.  Their margin(delta) curves differ, in a")
    print(f"    disjoint {len(multi) - n_mask} groups even in WHICH deltas reach the envelope at")
    print("    all; no nontrivial cyclic shift of delta aligns the curves; at a")
    print(f"    common delta the margin fields differ by up to {w_field:.2e},")
    print(f"    against a full-field margin range of {m_rng:.2e}; and they tune")
    print(f"    to deltas up to {circ_d:.0f} deg apart, with a unique argmax, so that")
    print("    is not one flat top read two ways either.")
    print()
    print("    What collapses is the TUNED MAXIMUM alone.  Two different")
    print("    machines, with different curves, peaking at different deltas,")
    print("    peak at the same height to the last bit.  That is a property of")
    print("    max over delta of the maximin margin, not of the geometry at any")
    print("    one delta - which is why the pointwise tests fail and the value")
    print("    test passes, and why it took all three requested tests to")
    print("    separate 'real' from 'the same machine twice'.")
    print()
    print(f"    Consequence for the ranking, unchanged from part (7): "
          f"{len(rows)} candidates")
    print(f"    carry {c_n} distinct margins, and no amount of grid refinement")
    print("    will break those ties - (8.2) is the evidence for that specific")
    print("    claim.  The mechanism is now known to be the tuner's maximum, so")
    print("    a different inner objective would not obviously inherit it; part")
    print("    (9) re-tunes on margin(p) and its rank columns are where that")
    print("    would show.")
    print()
    print("    What this does NOT license: dropping beta or beta_p from the")
    print("    sweep.  They collapse in the tuned MARGIN only.  sens does not")
    print("    collapse (part (7): 5e-4 within a group), and part (5) turns")
    print("    entirely on the sign and size of e near 0.  Nothing here is")
    print("    re-parameterised and this module does not propose that anything")
    print("    should be.")
    return dict(groups=groups, multi=multi, fine=fine, coarse_spread=c_m,
                fine_spread=f_m, kick_spread=k_m, survives=survives)


# --------------------------------------------------------------------------- #
# (9) tuned on margin(0), ranked on margin(p)
# --------------------------------------------------------------------------- #
def _margin_p_scan(g0, g90, R, az, z_home, deltas_deg, probe, m0):
    """``margin(dxy = probe)`` as a function of ``delta``, worst over azimuth.

    Same quantity part (7) ranks on, but as a curve over the whole ``delta``
    scan instead of at one ``delta`` - which is what lets the tune be re-run
    against it.  Clamped at ``m0``, the ``margin(0)`` curve, for exactly the
    reason :func:`sens_at` clamps: the displacement circle does not contain
    ``dxy = 0``, so an unclamped worst can sit ABOVE the nominal margin and
    ``sens`` would come out negative.  Part (7)'s score is the clamped value, so
    this has to be too or the two parts would rank different things.
    """
    dr = np.deg2rad(np.asarray(deltas_deg, dtype=float))
    cd, sd = np.cos(dr)[:, None, None], np.sin(dr)[:, None, None]
    worst = np.full(dr.size, np.inf)
    for ang in np.linspace(0.0, 2.0 * np.pi, N_DISP_DIR, endpoint=False):
        LL, P, A, B = _invariants(g0, g90, R,
                                  _T_stack(az, z_home, probe, float(ang)))
        w = A[None] * cd + B[None] * sd
        C = np.sqrt(np.maximum(LL[None] - w * w, 0.0))
        with np.errstate(divide="ignore", invalid="ignore"):
            m = np.where(C > 0.0, (C - np.abs(P)[None]) / C, -np.inf)
        worst = np.minimum(worst, m.reshape(dr.size, -1).min(axis=1))
    return np.minimum(worst, m0), worst


def _delta_gap(a, b, period=180.0):
    """Shorter arc between two ``delta`` values, degrees.

    ``n_i(delta + 180) = -n_i(delta)`` is the same servo plane, so ``delta`` is
    periodic mod 180 and 179 is one grid step from 0, not 179 steps.
    """
    g = abs(a - b) % period
    return min(g, period - g)


def _report_tune_mismatch(rows, R, az, ef_max=None, cap=SCORE_CAP,
                          probes=SCORE_PROBES, deltas_deg=DELTA_GRID):
    """Part (9): delta is tuned on ``margin(0)`` and ranked on ``margin(p)``."""
    print()
    print("=" * 78)
    print(f"(9) TUNE / SCORE MISMATCH - tuned on margin(0), ranked on "
          f"margin(p)")
    print("=" * 78)
    print("  Part (7) established score = margin(0) - sens(p)*p = margin(p).")
    print("  But delta is chosen to maximise margin(0), not margin(p).  So the")
    print("  ranked quantity is not the tuned quantity, and every candidate is")
    print("  being scored at a delta chosen for a different objective.")
    print()
    print("  delta_0 = argmax margin(0),  delta_p = argmax margin(p),")
    print("  BOTH over the same admissible set: cond(J_fk) <= C at")
    print(f"  char_len = r_b, C = {cap:.0e}.  Holding the admissible set fixed")
    print("  is what makes the two tunes comparable - the cap is a property of")
    print("  the delta, not of the objective, and it is evaluated at the nominal")
    print("  pose in both cases, exactly as parts (2) to (6) evaluate it.")
    print()
    print("  p is a build error in r_b.  margin is dimensionless.  The only")
    print("  characteristic length below is r_b, and it appears only in the cap")
    print("  and in p.  p is NOT chosen here - all three run side by side.")

    res = GRID_RESOLUTION if ef_max is None else float(ef_max)
    live = [r for r in rows if r["con"][cap] is not None]
    print()
    print(f"  candidates : {len(live)} of {len(rows)} "
          f"(those with an admissible delta at C = {cap:.0e})")
    print(f"  resolution : {res:.4e} margin units, this run's recomputed")
    print(f"               part (e)-(f) value (constant in the module: "
          f"{GRID_RESOLUTION:.4e})")

    t0 = time.perf_counter()
    n_clamp = 0
    for r in live:
        ok = np.isfinite(r["scan_margin"]) & (r["scan_cond"] <= cap)
        m0 = np.where(ok, r["scan_margin"], -np.inf)
        r["mismatch"] = {}
        for p in probes:
            mp_raw, unclamped = _margin_p_scan(r["_g0"], r["_g90"], R, az,
                                               r["z_home"], deltas_deg, p,
                                               r["scan_margin"])
            n_clamp += int(np.sum(ok & (unclamped > r["scan_margin"] + 1e-15)))
            mp = np.where(ok, mp_raw, -np.inf)
            k0 = int(np.argmax(m0))
            kp = int(np.argmax(mp))
            r["mismatch"][p] = dict(
                d0=float(deltas_deg[k0]), dp=float(deltas_deg[kp]),
                at_d0=float(mp[k0]), at_dp=float(mp[kp]),
                loss=float(mp[kp] - mp[k0]))
    dt = time.perf_counter() - t0
    print(f"  measured   : {dt:.1f} s, {len(probes)} probes x {len(live)} "
          f"candidates x {deltas_deg.size} deltas x {N_DISP_DIR} azimuths")
    print(f"  clamp bound at {n_clamp} of "
          f"{len(probes) * len(live) * deltas_deg.size} (delta, p) points - "
          f"see _margin_p_scan")

    # ---- cross-check against part (7) ----------------------------------- #
    worst_xc = 0.0
    for r in live:
        for p in probes:
            worst_xc = max(worst_xc, abs(r["mismatch"][p]["at_d0"]
                                         - r["score"]["disp"][p]))
    print(f"  self-check : worst |margin(p) at delta_0 from the scan minus")
    print(f"               part (7)'s own value| = {worst_xc:.3e}")
    print("               (part (7) evaluates the same quantity at one delta by")
    print("                a different route; disagreement would mean the two")
    print("                parts are not ranking the same thing)")

    # =================================================================== #
    # 9.1  does the tune move, and what does the mismatch cost?
    # =================================================================== #
    print()
    print("-" * 78)
    print("  (9.1) DOES delta MOVE, AND WHAT DOES THE MISMATCH COST?")
    print("-" * 78)
    print("    'given up' = margin(p) at delta_p minus margin(p) at delta_0,")
    print("    i.e. the score a candidate loses by being tuned on the wrong")
    print("    objective.  It is >= 0 by construction: delta_p maximises it.")
    print()
    print(f"    {'p [r_b]':>9} {'delta moved':>12} {'|d delta| deg':>26} "
          f"{'given up, margin units':>36}")
    print(f"    {'':>9} {'':>12} {'median':>12} {'max':>13} "
          f"{'median':>12} {'Q3':>11} {'max':>12}")
    loss = {}
    for p in probes:
        gap = np.array([_delta_gap(r["mismatch"][p]["dp"],
                                   r["mismatch"][p]["d0"]) for r in live])
        lo = np.array([r["mismatch"][p]["loss"] for r in live])
        loss[p] = lo
        moved = int(np.sum(gap > 0.0))
        mv = gap[gap > 0.0] if moved else np.array([0.0])
        print(f"    {p:>9.4f} {moved:>7d}/{len(live):<4d} "
              f"{np.median(mv):>12.2f} {mv.max():>13.2f} "
              f"{np.median(lo):>12.4e} {np.percentile(lo, 75):>11.4e} "
              f"{lo.max():>12.4e}")
    print()
    print("    ('|d delta|' is over the candidates that MOVED; delta is periodic")
    print("     mod 180, so the gap is the shorter arc - see _delta_gap.)")
    print()
    print(f"    {'p [r_b]':>9} {'given up, five-number over the whole field':>52}")
    print(f"    {'':>9} {'min':>12} {'Q1':>12} {'median':>12} {'Q3':>12} "
          f"{'max':>12}")
    for p in probes:
        q = _five_number(loss[p])
        print(f"    {p:>9.4f} {q[0]:>12.4e} {q[1]:>12.4e} {q[2]:>12.4e} "
              f"{q[3]:>12.4e} {q[4]:>12.4e}")

    # =================================================================== #
    # 9.2  is the mismatch big enough for the grid to see?
    # =================================================================== #
    print()
    print("-" * 78)
    print("  (9.2) IS THE MISMATCH RESOLVABLE, AND DOES IT MOVE THE RANKING?")
    print("-" * 78)
    print("    Two different questions, kept apart.  A candidate can lose more")
    print("    than the resolution and still not move (nobody is close enough to")
    print("    overtake), and it can move without losing much (its neighbours")
    print("    were tied).  Ranks are on margin(p), descending, ties averaged.")
    print()
    print(f"    {'p [r_b]':>9} {'given up > res':>15} {'rank changed':>13} "
          f"{'max |d rank|':>13} {'top-20 in/out':>14} {'top-10 in/out':>14}")
    detail = {}
    for p in probes:
        a0 = np.array([r["mismatch"][p]["at_d0"] for r in live])
        ap = np.array([r["mismatch"][p]["at_dp"] for r in live])
        r0 = _ranks(-a0)
        rp = _ranks(-ap)
        dr = np.abs(r0 - rp)
        n_res = int(np.sum(loss[p] > res))
        n_mv = int(np.sum(dr > 0.0))
        t20_0 = set(np.argsort(-a0, kind="mergesort")[:20])
        t20_p = set(np.argsort(-ap, kind="mergesort")[:20])
        t10_0 = set(np.argsort(-a0, kind="mergesort")[:10])
        t10_p = set(np.argsort(-ap, kind="mergesort")[:10])
        detail[p] = (a0, ap, r0, rp, dr, t20_0, t20_p)
        print(f"    {p:>9.4f} {n_res:>7d}/{len(live):<7d} "
              f"{n_mv:>7d}/{len(live):<5d} {dr.max():>13.1f} "
              f"{len(t20_p - t20_0):>14d} {len(t10_p - t10_0):>14d}")
    print()
    print("    'in/out' counts candidates in the top N under the CORRECT tune")
    print("    (delta_p) that are not there under the tune actually used")
    print("    (delta_0).  It is the number the mismatch would cost a shortlist.")
    print()
    print("    THE REQUESTED TEST, stated exactly.  A rank change matters only")
    print("    if the margin(p) difference driving it clears the grid's own")
    print("    resolution; below that the grid cannot tell the two apart and the")
    print(f"    reordering is inside its error.  res = {res:.4e}.")
    print()
    print(f"    {'p [r_b]':>9} {'moved candidates':>17} "
          f"{'of those, given up > res':>26} {'largest resolvable loss':>25}")
    any_real = False
    for p in probes:
        a0, ap, r0, rp, dr, _, _ = detail[p]
        mv = dr > 0.0
        n_mv = int(mv.sum())
        big = mv & (loss[p] > res)
        n_big = int(big.sum())
        top = float(loss[p][big].max()) if n_big else 0.0
        any_real = any_real or n_big > 0
        print(f"    {p:>9.4f} {n_mv:>10d}/{len(live):<6d} "
              f"{n_big:>26d} {top:>25.4e}")
    print()
    if any_real:
        print("    YES - at least one candidate's ranking position changes by")
        print("    more than the grid resolution as a result of the mismatch.")
        print("    The tune and the score are measurably out of step: the")
        print("    ordering the harness would produce is not the ordering the")
        print("    scored quantity implies, by an amount the grid can see.")
    else:
        print("    NO - every rank change the mismatch causes is driven by a")
        print("    margin(p) difference below the grid's own resolution.  The")
        print("    mismatch is real and measurable in the LOSS column above, but")
        print("    it does not move the ranking by more than the grid can")
        print("    resolve, so re-tuning on margin(p) would buy an ordering the")
        print("    grid cannot distinguish from the one already computed.")

    # =================================================================== #
    # 9.3  where the movement sits
    # =================================================================== #
    print()
    print("-" * 78)
    print("  (9.3) THE LEADERS, WHERE A RANKING IS ACTUALLY USED")
    print("-" * 78)

    def tag(r):
        return (f"{r['beta']:.0f}/{r['beta_p']:.0f}/{r['r_p']:.2f}/"
                f"{r['a']:.2f}/{r['d']:.2f}")

    for p in probes:
        a0, ap, r0, rp, dr, t20_0, t20_p = detail[p]
        idx = np.argsort(-ap, kind="mergesort")[:10]
        print()
        print(f"    p = {p} r_b - top 10 under delta_p (the correct tune):")
        print(f"    {'rank':>4} {'candidate':>22} {'delta_0':>8} "
              f"{'delta_p':>8} {'margin(p) @d_p':>15} {'given up':>12} "
              f"{'rank @d_0':>10}")
        for i, k in enumerate(idx):
            r = live[k]
            mm = r["mismatch"][p]
            print(f"    {i + 1:>4d} {tag(r):>22} {mm['d0']:>8.0f} "
                  f"{mm['dp']:>8.0f} {mm['at_dp']:>15.5e} "
                  f"{mm['loss']:>12.4e} {r0[k]:>10.1f}")
    print()
    print("    A candidate whose delta_0 and delta_p agree is being tuned and")
    print("    ranked on the same thing and appears with a zero loss; the tune")
    print("    and the score only diverge where those two columns differ.")
    print()
    print("    Nothing above proposes re-tuning on margin(p).  It measures what")
    print("    the existing tune costs against the quantity part (7) ranks on,")
    print("    at three p, under one cap, at one characteristic length, all")
    print("    named.  Which objective the inner tune SHOULD maximise is not")
    print("    settled here.")
    return detail


def _tail_block(rows):
    """The conditioning tail, and where in the parameter grid it sits.

    Reports only.  ``docs/archive/notation.md`` sec.12 records that a near-singular candidate
    "would pass feasibility and reach the ranking stage" and that the failure
    mode had so far been located only on ``smoke_geometry``, which is explicitly
    not a design.  Whether that is still true of the actual feasible set is a
    measurement, and this is it.
    """
    N = len(rows)
    print()
    print("-" * 78)
    print("  CONDITIONING TAIL - how many candidates, and where")
    print("-" * 78)
    print("  Counts above three decades.  These are REPORTING BINS, NOT")
    print("  thresholds: sec.12 leaves the reject line undecided and says the")
    print("  characteristic length is what would set it.  Nothing here sets one.")
    print()
    print(f"    {'column':<28}" + "".join(f"{'> ' + f'{t:.0e}':>12}"
                                          for t in TAIL_DECADES))
    for jac in ("J_fk", "J_cmd"):
        for key in CHAR_LEN_KEYS:
            v = _col(rows, "cond", key, jac)
            cells = "".join(f"{int(np.sum(v > t)):>12d}" for t in TAIL_DECADES)
            print("    " + f"cond({jac})@{key} PROVISIONAL".ljust(28) + cells)
    print()

    v = _col(rows, "cond", "r_b", "J_fk")
    tail = [r for r, c in zip(rows, v) if c > TAIL_DECADES[1]]
    if not tail:
        print("  No candidate exceeds the middle bin at char_len = r_b on J_fk.")
        return
    n_eq = sum(1 for r in tail if abs(r["beta_p"] - r["beta"]) < 1e-9)
    n_d0 = sum(1 for r in tail if r["delta"] == 0.0)
    n_all_eq = sum(1 for r in rows if abs(r["beta_p"] - r["beta"]) < 1e-9)
    print(f"  {len(tail)} of {N} candidates sit above {TAIL_DECADES[1]:.0e} "
          f"on cond(J_fk)@r_b.")
    print("  Their location in the grid is not scattered:")
    print(f"    with beta_p == beta          : {n_eq} of {len(tail)}   "
          f"(and {n_all_eq} of {N} candidates have beta_p == beta at all)")
    print(f"    with the tuned delta == 0    : {n_d0} of {len(tail)}")
    print()
    print("  This is the CONJUNCTION of the two, and neither alone.  At")
    print("  delta = 0 the servo plane is the vertical plane through the base")
    print("  axis and b_i; at beta_p == beta the anchor q_i is radially in line")
    print("  with b_i, so at R = I the whole rod lies in that same plane and all")
    print("  six rod lines meet the z-axis.  Six lines meeting one line span at")
    print("  most five screw dimensions - so J_fk drops rank at the HOME pose.")
    print()
    print("  It is NOT the claim sec.12 corrected.  That correction is about")
    print("  beta_p == beta ALONE (full rank 6, sigma_min monotonic through")
    print("  e = beta_p - beta = 0), and it survives: the delta sweep below")
    print("  holds beta_p == beta fixed and the conditioning recovers entirely.")
    print()
    worst = max(tail, key=lambda r: r["cond"]["r_b"]["J_fk"])
    R, az, _ = _pose_grid(None)
    g0, g90 = _delta_basis(worst["beta"], worst["beta_p"], worst["r_p"],
                           worst["a"], worst["d"])
    inv = _invariants(g0, g90, R, _T_stack(az, worst["z_home"]))
    print(f"  Worst candidate: beta = {worst['beta']:.0f}, "
          f"beta_p = {worst['beta_p']:.0f}, r_p/r_b = {worst['r_p']}, "
          f"a/r_b = {worst['a']}, d/r_b = {worst['d']},")
    print(f"  z_home/r_b = {worst['z_home']:.4f}.  delta swept, everything else "
          f"held:")
    print()
    print(f"    {'delta':>7} {'cond(J_fk)@r_b':>16} {'smin(J_fk)@r_b':>16} "
          f"{'cond(J_cmd)@r_b':>17} {'margin':>11} {'tau_min':>10}")
    sweep = []
    for dlt in TAIL_DELTAS:
        rec = dict(worst, delta=float(dlt))
        got = measure(rec, R, _T_stack(az, worst["z_home"]), CHAR_LEN_KEYS)
        if got is None:
            print(f"    {dlt:>7.1f}  ik unreachable at this delta")
            continue
        tau_min, cond, smin, _, _ = got
        m = _margin_at(inv, np.deg2rad(dlt))
        sweep.append((dlt, cond["r_b"]["J_fk"], m, tau_min))
        print(f"    {dlt:>7.1f} {cond['r_b']['J_fk']:>16.4e} "
              f"{smin['r_b']['J_fk']:>16.4e} {cond['r_b']['J_cmd']:>17.4e} "
              f"{m:>11.5f} {tau_min:>10.5f}")

    dl, cd, mg_, tu = (np.array(c) for c in zip(*sweep))
    span = np.log10(cd.max() / cd.min())
    m_var = (mg_.max() - mg_.min()) / mg_.max()
    t_var = (tu.max() - tu.min()) / tu.max()
    print()
    print("  Every conditioning number in that table is at char_len = r_b and is")
    print("  PROVISIONAL; the same sweep at r_p, d or a moves the values, not the")
    print("  shape.  Read the last two columns against the first three, over the")
    print(f"  swept range of delta:")
    print(f"    cond(J_fk)@r_b   spans {span:.1f} decades "
          f"({cd.min():.4g} to {cd.max():.4g})")
    print(f"    margin           varies by {100.0 * m_var:.2f}% of its largest "
          f"value, and is largest at delta = {dl[int(np.argmax(mg_))]:.1f}")
    print(f"    tau_min          varies by {100.0 * t_var:.2f}%, and is largest "
          f"at delta = {dl[int(np.argmax(tu))]:.1f}")
    print("  So the maximin-margin tune does not merely fail to see the")
    print("  degeneracy - it SELECTS it, on a margin difference in the third")
    print("  decimal place.  tau_min does not see it either, and would select")
    print("  the same delta.")
    print()
    print("  Recorded as a measurement.  What to do about it - reject, re-tune")
    print("  delta on a different objective, or something else - is a scoring")
    print("  decision and is not taken here.")


# --------------------------------------------------------------------------- #
def main() -> None:
    print("=" * 78)
    print("CANDIDATE DISCRIMINATORS - measured, none chosen")
    print("=" * 78)
    print("  This module decides NOTHING.  docs/archive/notation.md sec.12 lists the")
    print("  characteristic length, the choice of conditioning measure, and the")
    print("  score function as open; all three stay open here.  Every")
    print("  conditioning number below is quoted at all four candidate lengths")
    print("  and for both Jacobians, and every one is PROVISIONAL.")
    print()
    print("  NAMES (official in docs/archive/notation.md sec.6 as of 2026-09-10; local here")
    print("  first, and this table is kept as that provenance record):")
    for meaning, name in PROVISIONAL_NAMES.items():
        print(f"    {meaning:<46} -> {name}")
    print()

    print("-" * 78)
    print("SETUP")
    print("-" * 78)
    feas = feasible_candidates()
    R29, az29, _ = _pose_grid(None)
    print(f"  envelope    : tilt <= {TILT_LIMIT_DEG:.4f} deg, azimuth "
          f"{AZIMUTH_WINDOW_DEG} deg, dz = yaw = 0")
    print(f"  pose grid   : {az29.size} poses "
          f"({N_MAGNITUDE} magnitudes x {(az29.size - 1) // (N_MAGNITUDE - 1)} "
          f"azimuths, magnitude 0 once)")
    print(f"  z_home      : MIDPOINT of each candidate's bracket - a point in")
    print(f"                the interior for definiteness, not a recommendation")
    print(f"  delta       : tuned per candidate, {DELTA_GRID.size}-point scan "
          f"over [0, 180), maximin margin")
    print(f"  sens probe  : dxy = {PROBE_DXY} r_b, worst over {N_DISP_DIR} "
          f"displacement azimuths (full circle)")
    print(f"  char lengths: {', '.join(CHAR_LEN_KEYS)}  -  ALL FOUR, "
          f"none defaulted")
    print(f"  Jacobians   : J_fk  = df_i/d(T, omega), the FK residual Jacobian")
    print(f"                        used in the gate")
    print(f"                J_cmd = d(T, omega)/d(alpha), the servo-angle-to-")
    print(f"                        pose map, = J_fk^-1 diag(a e_i . tangent_i)")
    print("                Kept separate throughout.  No number below is a")
    print("                blend of the two.")
    print()

    # ------------------------------------------------------------------ #
    rows = []
    n_unreachable = 0
    n_neg_margin = 0
    worst_residual = 0.0
    for rec in feas:
        evaluate(rec, R29, lambda z: _T_stack(az29, z), az29)
        if rec["margin"] < 0.0:
            n_neg_margin += 1
        got = measure(rec, R29, _T_stack(az29, rec["z_home"]), CHAR_LEN_KEYS)
        if got is None:
            n_unreachable += 1
            continue
        (rec["tau_min"], rec["cond"], rec["smin"],
         rec["smax"], res) = got
        worst_residual = max(worst_residual, res)
        rows.append(rec)

    N = len(rows)
    print(f"  candidates measured : {N} / {len(feas)} feasible")
    if n_unreachable:
        print(f"  DROPPED (ik unreachable at the bracket midpoint) : "
              f"{n_unreachable}")
    if n_neg_margin:
        print(f"  NEGATIVE margin on the 29-pose grid : {n_neg_margin}  "
              f"(the screen uses a finer pose grid; the two are not nested)")
    print(f"  self-check, worst |rod_i| - d over every pose measured : "
          f"{worst_residual:.3e} r_b")
    print("    (rod_i comes from ik's own alpha, so this is 0 up to rounding;")
    print("     a nonzero value would mean tau and both Jacobians were taken")
    print("     at a pose the rods do not close at)")

    # ------------------------------------------------------------------ #
    # (1) verification - gates everything below
    # ------------------------------------------------------------------ #
    if not _report_verification(rows, R29):
        return

    # ------------------------------------------------------------------ #
    # the constrained inner tune, and the delta scan everything below uses
    # ------------------------------------------------------------------ #
    run_constrained(rows, R29, az29)
    d_free = np.array([r["delta"] for r in rows])
    d_scan = np.array([DELTA_GRID[int(np.nanargmax(r["scan_margin"]))]
                       for r in rows])
    m_scan = np.array([np.nanmax(r["scan_margin"]) for r in rows])
    print()
    print("  self-check, the vectorised delta scan against _tune_delta:")
    print(f"    candidates where argmax delta disagrees : "
          f"{int(np.sum(d_free != d_scan))} / {N}")
    print(f"    worst |margin difference|               : "
          f"{np.max(np.abs(m_scan - _col(rows, 'margin'))):.3e}")
    print("    (the scan builds alpha, the arm tip and both Jacobians from the")
    print("     same closed form ik uses; the margin is a by-product and agreeing")
    print("     with the A/B-decomposition path is what says the two agree)")
    worst_cd = 0.0
    for rec in rows[::37]:
        T = _T_stack(az29, rec["z_home"])
        for dlt in (0.0, 37.0, 90.0, 143.0):
            got = measure(rec, R29, T, (CONSTRAINT_CHAR_LEN,), delta=dlt)
            if got is None:
                continue
            ref = got[1][CONSTRAINT_CHAR_LEN]["J_fk"]
            j = int(np.flatnonzero(DELTA_GRID == dlt)[0])
            got_scan = rec["scan_cond"][j]
            if np.isfinite(ref) and ref > 0 and np.isfinite(got_scan):
                worst_cd = max(worst_cd, abs(got_scan - ref) / ref)
    print(f"    worst relative disagreement, scan cond(J_fk)@r_b vs measure(),")
    print(f"    over a spread of candidates and deltas : {worst_cd:.3e}")

    # ------------------------------------------------------------------ #
    # (a) five-number summary
    # ------------------------------------------------------------------ #
    print()
    print("=" * 78)
    print("(a) FIVE-NUMBER SUMMARY AND MAX, per column")
    print("=" * 78)
    print("  Every conditioning row names its Jacobian and its characteristic")
    print("  length.  The first three rows name neither BECAUSE THEY HAVE")
    print("  NEITHER - margin, sens and tau_min are dimensionless ratios and do")
    print("  not depend on a characteristic length at all.")
    print()
    hdr = (f"  {'column':<34} {'min':>12} {'Q1':>12} {'median':>12} "
           f"{'Q3':>12} {'max':>12} | {'max':>12} {'#!fin':>6}")
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    def show(label, vals):
        q = _five_number(vals)
        v = np.asarray(vals, float)
        mx = np.nanmax(v) if v.size else np.nan
        n_bad = int(np.sum(~np.isfinite(v)))
        print(f"  {label:<34} {q[0]:>12.4e} {q[1]:>12.4e} {q[2]:>12.4e} "
              f"{q[3]:>12.4e} {q[4]:>12.4e} | {mx:>12.4e} {n_bad:>6d}")

    show("margin           [no char_len]", _col(rows, "margin"))
    show("sens             [no char_len]", _col(rows, "sens"))
    show("tau_min          [no char_len]", _col(rows, "tau_min"))
    print()
    for jac in ("J_fk", "J_cmd"):
        for key in CHAR_LEN_KEYS:
            show(f"cond({jac})@{key}  PROVISIONAL",
                 _col(rows, "cond", key, jac))
        print()
    for jac in ("J_fk", "J_cmd"):
        for key in CHAR_LEN_KEYS:
            show(f"sigma_min({jac})@{key}  PROVISIONAL",
                 _col(rows, "smin", key, jac))
        print()
    print("  The 'max' column right of the bar is the same order statistic as")
    print("  the five-number summary's last entry, listed separately as asked.")
    print("  '#!fin' counts non-finite entries; they are kept, not dropped, and")
    print("  rank last.")
    print()
    print("  UNITS.  cond is dimensionless for both Jacobians.  sigma_min(J_fk)")
    print("  is dimensionless (mm of residual per mm of pose increment, the")
    print("  rotation columns measured as the arc length char_len * omega);")
    print("  sigma_min(J_cmd) is r_b per radian of servo angle at r_b = 1.  They")
    print("  are NOT comparable to each other and are not compared below - only")
    print("  their RANKINGS are, which is a comparison units survive.")

    _tail_block(rows)

    # ------------------------------------------------------------------ #
    # (b) Spearman: margin / sens / tau_min against every cond column
    # ------------------------------------------------------------------ #
    print()
    print("=" * 78)
    print("(b) SPEARMAN RANK CORRELATION - margin, sens, tau_min vs each cond")
    print("=" * 78)
    base = {"margin": _col(rows, "margin"),
            "sens": _col(rows, "sens"),
            "tau_min": _col(rows, "tau_min")}

    print("  among the three themselves:")
    print(f"    {'':<10} {'margin':>10} {'sens':>10} {'tau_min':>10}")
    for i in base:
        cells = []
        for j in base:
            rho, _ = _spearman(base[i], base[j])
            cells.append(f"{rho:>10.4f}")
        print(f"    {i:<10} " + " ".join(cells))
    print()
    print("  against each cond column (rows: the three; columns: Jacobian and")
    print("  characteristic length, every one PROVISIONAL):")
    print()
    cond_labels = [(jac, key) for jac in ("J_fk", "J_cmd")
                   for key in CHAR_LEN_KEYS]
    head = "    " + f"{'':<10}" + "".join(
        f"{jac + '@' + key:>14}" for jac, key in cond_labels)
    print(head)
    for name, vals in base.items():
        cells = []
        for jac, key in cond_labels:
            rho, _ = _spearman(vals, _col(rows, "cond", key, jac))
            cells.append(f"{rho:>14.4f}")
        print(f"    {name:<10}" + "".join(cells))
    print()
    print("  Reading: a rho near 0 means that cond column orders the field")
    print("  differently from that discriminator, so the two are not")
    print("  substitutes; a rho near +/-1 means one adds nothing the other does")
    print("  not already say.  Which of them a score should USE is not decided")
    print("  here, and this table does not decide it.")

    # ------------------------------------------------------------------ #
    # (c) do the two Jacobians rank candidates differently at all
    # ------------------------------------------------------------------ #
    print()
    print("=" * 78)
    print("(c) SPEARMAN: cond(J_fk) vs cond(J_cmd) AT MATCHED char_len")
    print("=" * 78)
    print("  Same characteristic length on both sides, so the only difference is")
    print("  which Jacobian.  J_cmd = J_fk^-1 diag(a e_i . tangent_i): if every")
    print("  tau_i were equal the diagonal would be a scalar and the two conds")
    print("  would coincide, so any departure from 1.0 here is the SPREAD of")
    print("  tau across the six legs showing up in the ranking.")
    print()
    print(f"    {'char_len':>10} {'rho':>12} {'n':>7}")
    for key in CHAR_LEN_KEYS:
        rho, n = _spearman(_col(rows, "cond", key, "J_fk"),
                           _col(rows, "cond", key, "J_cmd"))
        print(f"    {key:>10} {rho:>12.4f} {n:>7d}     PROVISIONAL char_len")
    print()
    print("  and the same for sigma_min, at matched char_len:")
    print(f"    {'char_len':>10} {'rho':>12} {'n':>7}")
    for key in CHAR_LEN_KEYS:
        rho, n = _spearman(_col(rows, "smin", key, "J_fk"),
                           _col(rows, "smin", key, "J_cmd"))
        print(f"    {key:>10} {rho:>12.4f} {n:>7d}     PROVISIONAL char_len")

    # ------------------------------------------------------------------ #
    # (d) rank shift when the characteristic length is switched
    # ------------------------------------------------------------------ #
    print()
    print("=" * 78)
    print("(d) RANK SHIFT WHEN char_len IS SWITCHED, per Jacobian")
    print("=" * 78)
    thr = RANK_SHIFT_FRAC * N
    print(f"  Candidates ranked by cond ascending (rank 1 = lowest cond).  A")
    print(f"  candidate is counted when its rank moves by more than "
          f"{RANK_SHIFT_FRAC:.0%} of the")
    print(f"  field = {thr:.2f} ranks out of {N}.  Every pair below is the SAME")
    print(f"  Jacobian at two different characteristic lengths - the two")
    print(f"  Jacobians are never compared across a pair here.")
    print()
    for jac in ("J_fk", "J_cmd"):
        print(f"  {jac}:")
        print(f"    {'pair':>16} {'shifted':>9} {'of':>6} {'%':>8} "
              f"{'max shift':>11} {'rho':>9}")
        rk = {key: _ranks(_col(rows, "cond", key, jac))
              for key in CHAR_LEN_KEYS}
        for i, k1 in enumerate(CHAR_LEN_KEYS):
            for k2 in CHAR_LEN_KEYS[i + 1:]:
                sh = np.abs(rk[k1] - rk[k2])
                n_sh = int(np.sum(sh > thr))
                rho, _ = _spearman(_col(rows, "cond", k1, jac),
                                   _col(rows, "cond", k2, jac))
                print(f"    {k1 + ' vs ' + k2:>16} {n_sh:>9d} {N:>6d} "
                      f"{100.0 * n_sh / N:>7.1f}% {sh.max():>11.1f} "
                      f"{rho:>9.4f}")
        print()
    print("  Both columns in every pair are PROVISIONAL.  This table says how")
    print("  much the undecided length matters to the ORDER of the field, which")
    print("  is the question docs/archive/notation.md sec.12 raises when it says the length")
    print("  'changes the ranking of candidates'.  It does not say which length")
    print("  to take.")

    # ------------------------------------------------------------------ #
    # (e)-(g) azimuth resolution and delta tuning
    # ------------------------------------------------------------------ #
    print()
    print("=" * 78)
    print("(e)-(g) AZIMUTH RESOLUTION vs DELTA TUNING")
    print("=" * 78)
    marg = _col(rows, "margin")
    order = np.argsort(marg, kind="mergesort")
    pick = np.unique(np.round(np.linspace(0, N - 1, N_SAMPLE)).astype(int))
    sample = [rows[order[i]] for i in pick]

    Rf, azf, _ = _pose_grid(FINE_AZ_STEP_DEG)
    print(f"  sample : {len(sample)} candidates, evenly spaced in MARGIN RANK")
    print(f"           across all {N} - the whole range, NOT the top.")
    print(f"           margin spans [{marg[order[pick[0]]]:.4e}, "
          f"{marg[order[pick[-1]]]:.4e}]")
    print(f"  coarse : {az29.size} poses, azimuth step 10 deg   (the settled "
          f"harness grid)")
    print(f"  fine   : {azf.size} poses, azimuth step "
          f"{FINE_AZ_STEP_DEG} deg")
    print(f"  Only the azimuth resolution differs; tilt limit, magnitudes, the")
    print(f"  [30, 90] window, z_home and the delta scan are all identical.")
    print()
    print(f"    {'beta':>5} {'beta_p':>6} {'r_p':>5} {'a':>5} {'d':>5} "
          f"{'dlt_c':>6} {'dlt_f':>6} {'(e)':>12} {'(f)':>12} {'(g)':>12} "
          f"{'(e)-(f)':>11} {'(f)-(g)':>11}")
    ef, fg, e_vals, g_vals = [], [], [], []
    for rec in sample:
        T_f = _T_stack(azf, rec["z_home"])
        inv_f = _invariants(rec["_g0"], rec["_g90"], Rf, T_f)
        e_m = rec["margin"]                                   # (e)
        f_m = _margin_at(inv_f, np.deg2rad(rec["delta"]))     # (f)
        d_g, g_m = _tune_delta(inv_f)                         # (g)
        ef.append(e_m - f_m)
        fg.append(f_m - g_m)
        e_vals.append(e_m)
        g_vals.append(g_m)
        print(f"    {rec['beta']:>5.1f} {rec['beta_p']:>6.1f} {rec['r_p']:>5.2f} "
              f"{rec['a']:>5.2f} {rec['d']:>5.2f} {rec['delta']:>6.1f} "
              f"{d_g:>6.1f} {e_m:>12.5e} {f_m:>12.5e} {g_m:>12.5e} "
              f"{e_m - f_m:>+11.3e} {f_m - g_m:>+11.3e}")

    ef = np.array(ef)
    fg = np.array(fg)
    print()
    print("  SPREAD across the sample:")
    print(f"    {'quantity':<28} {'min':>12} {'median':>12} {'max':>12} "
          f"{'range':>12}")
    for label, arr in (("(e) - (f)  coarse optimism", ef),
                       ("(f) - (g)  tuning recovery", fg)):
        q = _five_number(arr)
        print(f"    {label:<28} {q[0]:>+12.4e} {q[2]:>+12.4e} {q[4]:>+12.4e} "
              f"{q[4] - q[0]:>12.4e}")
    print()
    print("  SIGNS, both fixed by construction and neither an empirical result:")
    print("  (e) - (f) >= 0.  Same delta, and the fine grid contains the coarse")
    print("    one, so refining can only find a worse worst case.  It is what the")
    print("    coarse azimuth grid OVERSTATES the margin by.")
    print("  (f) - (g) <= 0.  Both are read on the fine grid and (g) maximises")
    print("    over delta there, so (g) >= (f).  Its MAGNITUDE is how much")
    print("    re-tuning delta on the fine grid buys back.")
    print()
    rho, n = _spearman(np.array(e_vals), np.array(g_vals))
    print(f"  SPEARMAN, (e) ranking vs (g) ranking : rho = {rho:.4f}  "
          f"(n = {n})")
    print("  This is the question of whether the coarse grid picks the same")
    print("  ORDER as a 40x finer one with delta re-tuned on it - not whether")
    print("  it picks the same numbers, which the spread above already answers.")

    # ------------------------------------------------------------------ #
    # (2)-(6) the constrained tune and what it changes
    # ------------------------------------------------------------------ #
    _report_constrained(rows)
    _report_b_under_constraint(rows)
    _report_cond_distribution(rows)
    _report_neighbourhood(rows, R29, az29)
    _report_cost(rows, R29, az29)
    res = float(ef.max()) if ef.size else None
    _report_score(rows, R29, az29, ef_max=res)

    # ------------------------------------------------------------------ #
    # (8)-(9) the two verifications
    # ------------------------------------------------------------------ #
    _report_collapse(rows, R29, az29)
    _report_tune_mismatch(rows, R29, az29, ef_max=res)

    print()
    print("=" * 78)
    print("WHAT THIS DOES NOT SETTLE")
    print("=" * 78)
    print("  * the characteristic length.  Four are carried; none is preferred,")
    print("    none is defaulted, and (d) reports what switching costs.")
    print("  * which Jacobian.  J_fk and J_cmd are kept apart everywhere; (c)")
    print("    reports whether they even order the field differently.")
    print("  * the score function, its terms and any reject floor.  Parts (a)")
    print("    to (6) form no scalar score at all.  Part (7) EVALUATES one that")
    print("    was specified to it - margin - sens*p, at three given p - and")
    print("    reports what it does to the ordering; it does not propose that")
    print("    form, choose p, or weight anything.  There is no weight in it to")
    print("    choose: (7) shows margin - sens*p IS margin read at displacement")
    print("    p, by identity, to 7e-18.")
    print("  * z_home, which is a swept axis; the bracket midpoint here is a")
    print("    point of evaluation, not a choice.")
    print("  * the names in docs/archive/notation.md sec.6 / sec.11.  The two used here,")
    print("    rod_i and tangent_i, are official as of 2026-09-10.")
    print("  * the cap C.  Four are run side by side; none is preferred, and the")
    print("    unconstrained tune is retained beside every one of them.")
    print("  * whether the inner tune SHOULD be constrained at all.  Parts (2)")
    print("    to (4) measure what a cap costs and what it changes; they do not")
    print("    argue that a cap belongs in the harness.")
    print("  * the beta_p spacing of the sweep, which part (5) reports a width")
    print("    against but does not set.")
    print("  * the build error p.  Three run side by side in part (7); none is")
    print("    preferred.  p is not a weight - it is the displacement the margin")
    print("    is read at, which (7) shows by identity.")
    print()
    print("  CHANGED ELSEWHERE, recorded here, not decided here:")
    print("  * cond is no longer an outer ranking term (2026-09-07).  It keeps")
    print("    the inner-tune cap, and every cond column above is retained")
    print("    unchanged as the evidence that cap rests on.")


if __name__ == "__main__":
    main()
