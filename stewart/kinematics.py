"""Inverse and forward kinematics for a 6-RSS Stewart platform.

Signatures, type hints and docstrings only - every function body raises
``NotImplementedError``.  Millimetres and radians; a servo angle is in radians,
measured from ``u_i`` toward ``v_i = n_i x u_i``.
"""
from __future__ import annotations

import numpy as np

from .geometry import Geometry


class Unreachable(ValueError):
    """A leg cannot reach its commanded platform anchor.

    The servo-arm tip is confined to a circle of radius ``a`` in the servo
    plane; no point on that circle lies within one push-rod length ``d`` of
    the anchor - equivalently ``|P_i| > C_i`` in the notation of :func:`ik`.

    Attributes
    ----------
    leg : int
        Failing leg, **1-indexed** (matches construction-time messages).
    direction : str
        ``"far"``  - anchor beyond reach (``P_i > C_i``);
        ``"near"`` - anchor too close, the rod cannot compress
        (``P_i < -C_i``).
    ratio : float | None
        The offending ``P_i / C_i`` when available (``|ratio| > 1``).
    """

    def __init__(self, leg: int, direction: str, ratio: float | None = None) -> None:
        self.leg = int(leg)
        self.direction = str(direction)
        self.ratio = None if ratio is None else float(ratio)
        detail = "" if self.ratio is None else f", P/C = {self.ratio:+.4f}"
        super().__init__(
            f"leg {self.leg}: platform anchor unreachable "
            f"({self.direction}{detail})"
        )


def _v(geom: Geometry) -> np.ndarray:
    """In-plane basis partner of ``u``: ``v_i = n_i x u_i``, shape ``(3, 6)``.

    Equals world ``z`` exactly while every ``n_i`` is horizontal (derivation
    sec.8), but it is computed rather than assumed so a canted shaft does not
    silently give a wrong answer.
    """
    return np.cross(geom.n, geom.u, axis=0)


def stage1(geom: Geometry, R: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Stage 1 (derivation sec.3): platform anchors in the world frame.

        q_i = T + R p_i

    ``R @ p`` - the rotation acts from the left on the column ``p_i``;
    ``p @ R`` would silently apply ``R`` transposed.

    Parameters
    ----------
    geom : Geometry
    R : ndarray, shape (3, 3)
        Platform orientation, world-from-platform.
    T : ndarray, shape (3,)
        Platform origin in the world frame, mm.

    Returns
    -------
    q : ndarray, shape (3, 6)
        Column ``i`` is anchor ``i``.

    Notes
    -----
    This signature follows derivation sec.3, which names ``q_i = T + R p_i``
    "stage 1", and the sec.7 check ``stage1(R=I, T=0) == p``.  The previous
    stub docstring described a different function - the branch-independent
    coefficients ``(A, B, P, C)``.  Those now live inside :func:`ik`, which is
    their only consumer.
    """
    R = np.asarray(R, dtype=float)
    if R.shape != (3, 3):
        raise ValueError(f"R must have shape (3, 3); got {R.shape}")
    T = np.asarray(T, dtype=float).reshape(3, 1)
    return R @ geom.p + T


def legs(geom: Geometry, R: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Leg vectors from each servo centre to each platform anchor.

        L_i = (R @ p_i + T) - b_i

    Note ``R @ p`` - the rotation acts from the left on the column ``p_i``.
    ``p @ R`` silently applies ``R`` transposed.

    Parameters
    ----------
    geom : Geometry
    R : ndarray, shape (3, 3)
        Platform orientation, world-from-platform.
    T : ndarray, shape (3,)
        Platform origin in the world frame, mm.

    Returns
    -------
    L : ndarray, shape (3, 6)
    """
    return stage1(geom, R, T) - geom.b


def w(geom: Geometry, R: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Signed out-of-plane offsets ``w_i = L_i . n_i`` (notation key), shape ``(6,)``.

    Only the horizontal part of ``L_i`` contributes while ``n_i`` is
    horizontal.  Takes ``L`` from :func:`legs`; the inlined copy carried here
    while ``legs`` was a stub is gone.
    """
    return np.einsum("ij,ij->j", legs(geom, R, T), geom.n)


def arm_tips(geom: Geometry, alphas: np.ndarray) -> np.ndarray:
    """Servo-arm tip positions for given servo angles.

        tip_i = b_i + a (cos(alpha_i) u_i + sin(alpha_i) v_i),   v_i = n_i x u_i

    Parameters
    ----------
    geom : Geometry
    alphas : ndarray, shape (6,)
        Servo angles, **radians**.

    Returns
    -------
    tips : ndarray, shape (3, 6)
    """
    alphas = np.asarray(alphas, dtype=float).reshape(-1)
    if alphas.shape != (6,):
        raise ValueError(f"alphas must have 6 entries; got shape {alphas.shape}")
    return geom.b + geom.a * (np.cos(alphas) * geom.u + np.sin(alphas) * _v(geom))


def ik(geom: Geometry, R: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Inverse kinematics: platform pose -> six servo angles.

    For leg ``i``::

        q_i      = R @ p_i + T                                anchor, world frame
        L_i      = q_i - b_i                                  servo centre -> anchor
        v_i      = n_i x u_i                                  in-plane basis with u_i
        tip_i(a) = b_i + a (cos a * u_i + sin a * v_i)        a = servo angle

    The rod constraint ``|q_i - tip_i(alpha_i)| = d`` reduces to
    ``M_i cos(alpha_i) + N_i sin(alpha_i) = P_i`` (derivation sec.5.2) with

        M_i = L_i . u_i
        N_i = L_i . v_i
        P_i = (|L_i|^2 + a^2 - d^2) / (2 a)
        C_i = hypot(M_i, N_i)

    inverted (sec.5.3) as

        alpha_i = phi_i - arccos(P_i / C_i),    phi_i = atan2(N_i, M_i)

    ``atan2``, never ``arctan``: ``arctan(N/M)`` spans only half a turn and
    the division destroys the sign that separates ``(M, N)`` from
    ``(-M, -N)``, which puts half the servos 180 degrees out.

    Branch
    ------
    The **minus** branch is fixed, not a parameter.  Under horizontal shafts
    ``v_i = z`` exactly (sec.8), so

        N_i = L_i . z = q_i . z = anchor height above the base plate > 0

    for every leg at every pose the platform can physically hold.  A positive
    ``N_i`` puts ``phi_i = atan2(N_i, M_i)`` in the upper half-plane
    ``(0, pi)`` whatever the sign of ``M_i``, and the minus branch is then the
    root continuously connected to the assembly datum ``alpha_i = 0``, for all
    six legs at once.  Measured 2026-09-03: at the datum every leg returns
    ``alpha_i = 0`` on the minus branch to ``9e-16``, and across a 4365-pose
    envelope the branch never became undefined except where *both* roots
    vanish at the workspace boundary.

    **This rests entirely on horizontal shafts.**  Cant them and ``v_i`` is no
    longer ``z``, ``N_i`` may change sign, and the branch choice reopens.

    Reachability
    ------------
    The test is ``|P_i| > C_i``  ->  raise :class:`Unreachable`.

    It is **not** the two-sphere bound ``|d - a| < |L_i| < d + a``.  That
    bound treats the arm tip as free on a *sphere* of radius ``a`` about
    ``b_i``.  The tip is actually on a *circle* - the sphere intersected with
    the servo plane.  The component of ``L_i`` along the plane normal ``n_i``
    still enters ``P_i`` through ``|L_i|^2`` but never enters ``C_i``, which
    sees only ``L_i . u_i`` and ``L_i . v_i``.  So a leg whose ``L_i`` is
    tilted well out of its servo plane can satisfy the two-sphere bound and
    still have ``|P_i| > C_i``.  The two-sphere bound is necessary, not
    sufficient.

    Do **not** ``np.clip(P_i / C_i, -1, 1)`` before ``acos``.  Clipping maps
    every unreachable leg onto exactly ``+/-1`` and returns a boundary angle
    that looks like a solution and is not.  Test, then raise.

    Parameters
    ----------
    geom : Geometry
    R : ndarray, shape (3, 3)
        Platform orientation, world-from-platform.  Applied as ``R @ p``.
    T : ndarray, shape (3,)
        Platform origin in the world frame, mm.

    Returns
    -------
    alphas : ndarray, shape (6,)
        Servo angles in **radians**, measured from ``u_i`` toward
        ``v_i = n_i x u_i``, positive sense right-handed about ``n_i``.
        ``alpha_i = 0`` lays the arm along ``u_i``, flat in the base plane.
        Returned as the raw ``phi_i - arccos(P_i / C_i)``, not wrapped; with
        ``N_i > 0`` this already lies in ``(-pi, pi)``.

    Raises
    ------
    Unreachable
        If ``|P_i| > C_i`` for any leg.  Carries the 1-indexed leg and the
        direction (``"far"`` if ``P_i > C_i``, ``"near"`` if ``P_i < -C_i``).
        The lowest-numbered failing leg is reported.
    """
    L = legs(geom, R, T)
    M = np.einsum("ij,ij->j", L, geom.u)
    N = np.einsum("ij,ij->j", L, _v(geom))
    P = (np.einsum("ij,ij->j", L, L) + geom.a ** 2 - geom.d ** 2) / (2.0 * geom.a)
    C = np.hypot(M, N)

    # Reachability BEFORE arccos.  Clipping P/C would map every unreachable
    # leg onto +/-1 and hand back a boundary angle that is not a solution.
    bad = np.abs(P) > C
    if bad.any():
        i = int(np.flatnonzero(bad)[0])
        ratio = float(P[i] / C[i]) if C[i] != 0.0 else None
        raise Unreachable(i + 1, "far" if P[i] > 0.0 else "near", ratio)

    # |P| <= C, so P/C lands in [-1, 1] under IEEE rounding; no clip needed.
    return np.arctan2(N, M) - np.arccos(P / C)


# --------------------------------------------------------------------------- #
# forward kinematics
# --------------------------------------------------------------------------- #
class FKNotConverged(RuntimeError):
    """:func:`fk` hit the iteration cap without meeting the tolerance.

    Raised rather than returning a best effort.  A pose that did not converge
    is not a pose; handing one back silently is how a round-trip gate comes to
    pass while wrong.

    Attributes
    ----------
    residual_mm : float
        ``max_i |f_i|`` at the last iterate, mm.  ``f_i = |q_i - h_i| - d``.
    iterations : int
        Iterations actually taken (equals the cap unless the solve broke down).
    tol_mm : float
        The tolerance that was not met, mm.
    reason : str
        ``"cap"`` - ran out of iterations; ``"stalled"`` - neither the Newton
        step nor the Levenberg-Marquardt fallback could reduce the residual;
        ``"singular"`` - the step could not be formed at all.
    """

    def __init__(self, residual_mm: float, iterations: int, tol_mm: float,
                 reason: str = "cap") -> None:
        self.residual_mm = float(residual_mm)
        self.iterations = int(iterations)
        self.tol_mm = float(tol_mm)
        self.reason = str(reason)
        super().__init__(
            f"fk did not converge ({self.reason}): residual "
            f"{self.residual_mm:.3e} mm > tol {self.tol_mm:.3e} mm after "
            f"{self.iterations} iterations"
        )


#: Convergence tolerance for :func:`fk`, **millimetres**.  The residual is
#: unsquared by choice - ``f_i = |q_i - h_i| - d`` is literally the amount by
#: which rod ``i`` fails to close - so the tolerance is a physical length and
#: needs no conversion factor.
#:
#: **It is an ACCEPTANCE threshold, not a stopping rule**, and that distinction
#: is the whole reason the value below is defensible.  The iteration stops when
#: a step can no longer reduce ``max_i |f_i|`` - at the arithmetic floor - and
#: the tolerance is then applied once, to decide whether the converged residual
#: is small enough to return.  Stopping AT the tolerance instead was tried and
#: is wrong for a gate: with the tolerance as the stopping rule, roughly 40% of
#: poses halt on the first iterate that crosses it, so the worst residual over
#: any pose grid sits just under the tolerance whatever the tolerance is, and
#: the worst pose error tracks it linearly - measured, ``9.7e-10 mm`` at
#: ``1e-9`` down to ``1.3e-13 mm`` at ``1e-13``, a straight line.  A gate built
#: that way reports its own stopping rule back to itself and can never show the
#: tolerance is not what limits accuracy, because it always is.
#:
#: With stagnation as the stopping rule, the value has to clear a floor and a
#: ceiling, both lengths:
#:
#: * FLOOR.  At the gate's working scale (``r_b = 100 mm``, rods ~120 mm) a
#:   double resolves a length to ``eps * 120 ~ 2.7e-14 mm``, and forming
#:   ``|q - h| - d`` cancels two same-sized quantities.  The measured floor is
#:   ``1.4e-14`` to ``7.1e-14 mm``.  ``1e-9 mm`` sits between four and five
#:   decades above it, so acceptance never fails for a numerical reason.
#: * CEILING.  Any tolerance the hardware can mean is micrometres at best
#:   (``1e-3 mm``).  ``1e-9 mm`` is six decades below that, so it cannot be
#:   mistaken for a manufacturing or control tolerance, and a solve that
#:   genuinely failed to converge lands orders above it, not just outside it.
#:
#: The claim that the tolerance is not what limits accuracy is still MEASURED,
#: not argued from those two bounds: ``stewart/diagnostics/roundtrip.py``
#: tightens it tenfold and checks the round-trip pose error does not move.  It
#: is free to be, because the answer no longer depends on the tolerance at all
#: over the range where acceptance succeeds - which is the point.
FK_TOL_MM = 1e-9

#: Iteration cap for :func:`fk`.  Newton from the home seed converges in single
#: digits everywhere the gate samples; the cap exists to turn a non-converging
#: pose into a raised exception rather than a hang.
FK_MAX_ITER = 100


def _skew(v: np.ndarray) -> np.ndarray:
    """``[v]_x``, the matrix with ``[v]_x w == v x w``."""
    x, y, z = np.asarray(v, dtype=float).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def exp_so3(omega: np.ndarray) -> np.ndarray:
    """Rotation vector -> ``SO(3)`` (Rodrigues).  ``|omega|`` is the angle, rad.

    The rotation vector is the solver's parameterisation of ``R`` and is
    **not** a rotation convention: it names an axis and an angle, with no
    ordered sequence of elementary rotations and nothing to fix the order of.
    Derivation sec.6 stays open and is not touched by anything here.
    """
    omega = np.asarray(omega, dtype=float).reshape(3)
    th = float(np.linalg.norm(omega))
    K = _skew(omega)
    if th < 1e-12:
        # Below 1e-12 rad the series is exact in double to the terms kept, and
        # sin(th)/th would be 0/0.
        return np.eye(3) + K + 0.5 * (K @ K)
    return (np.eye(3)
            + (np.sin(th) / th) * K
            + ((1.0 - np.cos(th)) / (th * th)) * (K @ K))


def log_so3(R: np.ndarray) -> np.ndarray:
    """``SO(3)`` -> rotation vector.  Inverse of :func:`exp_so3`.

    Returns the axis-angle vector ``omega`` with ``exp_so3(omega) == R`` and
    ``|omega| <= pi``; ``|omega|`` is the rotation angle in radians.  Like
    :func:`exp_so3` this is not a rotation convention - it names an axis and an
    angle, with no ordered sequence of elementary rotations.

    **Use this, not ``arccos((tr(R) - 1) / 2)``, to measure a small rotation.**
    Both compute the same angle, but ``arccos`` is ill-conditioned exactly where
    rotation errors are measured.  Near the identity ``tr(R) = 3 - theta^2 +
    O(theta^4)``, so the trace carries ``theta`` only at second order: an
    ``O(eps)`` error in the trace becomes an ``O(sqrt(eps))`` error in the
    angle, and ``arccos`` of ``1 - eps`` returns ``~sqrt(2 eps) = 1.5e-8 rad =
    8.5e-7 deg`` for a rotation that is exactly the identity to machine
    precision.  That is a FLOOR: no rotation smaller than it can be resolved by
    that formula at all.  The antisymmetric part used below is LINEAR in
    ``theta``, so it has no such floor and resolves to ``~1e-15 rad``.

    Branches.  ``s = |vee(R - R^T)| / 2 = sin(theta)`` and
    ``c = (tr(R) - 1) / 2 = cos(theta)``, so ``theta = atan2(s, c)`` is exact
    and well conditioned for ``theta`` away from ``pi``.  As ``theta -> pi`` the
    antisymmetric part vanishes and the axis has to come from the symmetric
    part instead, via ``(R + R^T)/2 = cos(theta) I + (1 - cos(theta)) a a^T``.
    The switch is at ``cos(theta) < -0.9`` (``theta > 154 deg``), well away from
    anything this project measures, but present so the function is total.
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    A = R - R.T
    v = 0.5 * np.array([A[2, 1], A[0, 2], A[1, 0]])   # = sin(theta) * axis
    s = float(np.linalg.norm(v))
    c = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))

    if c > -0.9:
        theta = float(np.arctan2(s, c))
        if s < 1e-12:
            # sin(theta)/theta -> 1, so v IS the rotation vector to this order.
            # Includes theta == 0 exactly, where v == 0 and the answer is 0.
            return v
        return v * (theta / s)

    # theta near pi: recover a a^T from the symmetric part, then fix the sign.
    theta = float(np.arctan2(s, c))
    aat = (0.5 * (R + R.T) - c * np.eye(3)) / (1.0 - c)
    k = int(np.argmax(np.diag(aat)))
    axis = aat[:, k] / np.sqrt(max(aat[k, k], 0.0))
    axis = axis / np.linalg.norm(axis)
    if float(axis @ v) < 0.0:
        axis = -axis
    return axis * theta


def geodesic_angle(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """Geodesic angle between two rotations, **radians**.

    ``|log_so3(R_a^T R_b)|`` - the magnitude of the rotation vector taking one
    to the other, which is the same quantity as
    ``arccos((tr(R_a^T R_b) - 1) / 2)`` and has none of its floor.  See
    :func:`log_so3`.
    """
    return float(np.linalg.norm(log_so3(np.asarray(R_a).T @ np.asarray(R_b))))


def fk_residual(geom: Geometry, tips: np.ndarray, R: np.ndarray,
                T: np.ndarray):
    """The six rod-closure residuals, **millimetres**, and their ingredients.

        f_i = |q_i - h_i| - d,       q_i = T + R p_i,  h_i = arm tip i

    UNSQUARED.  ``f_i`` is a length - the gap rod ``i`` cannot close - so the
    convergence tolerance is a length too, with no conversion factor.

    Returns
    -------
    f : ndarray, shape (6,)
    rvec : ndarray, shape (3, 6)
        ``q_i - h_i``, tip to anchor.
    norm : ndarray, shape (6,)
        ``|q_i - h_i|``.
    """
    q = R @ geom.p + np.asarray(T, dtype=float).reshape(3, 1)
    rvec = q - tips
    norm = np.linalg.norm(rvec, axis=0)
    return norm - geom.d, rvec, norm


def fk_jacobian(geom: Geometry, R: np.ndarray, rvec: np.ndarray,
                norm: np.ndarray):
    """Analytic ``6 x 6`` Jacobian of :func:`fk_residual` in ``(T, omega)``.

    With ``e_i = (q_i - h_i) / |q_i - h_i|`` the unit vector from tip to
    anchor, and the rotation perturbed on the **LEFT**,
    ``R -> exp([omega]_x) R``::

        df_i/dT      =  e_i^T
        df_i/domega  = -e_i^T [R p_i]_x  =  (R p_i x e_i)^T

    The second identity is why the code forms a cross product rather than a
    skew matrix: ``-e^T [v]_x w = -e . (v x w) = -(e x v) . w = (v x e) . w``.

    Sign, and which side.  Left perturbation gives
    ``dq_i = [omega]_x (R p_i) = -[R p_i]_x omega``, hence the minus.  Under a
    RIGHT perturbation ``R -> R exp([omega]_x)`` the same derivation gives
    ``+e_i^T R [p_i]_x``, which differs from the above by more than a sign, so
    the two cannot be reconciled by flipping one.  This form is
    finite-differenced against the left perturbation it claims to describe in
    ``stewart/diagnostics/roundtrip.py``; do not change the sign without
    re-running it.

    ``e_i`` is built from the ACTUAL ``|q_i - h_i|``, not from ``d``.  The two
    agree at a solution, but away from one only the actual norm is the true
    derivative, and the seed is deliberately not a solution.

    UNITS.  Columns 0-2 are dimensionless (mm of residual per mm of
    translation); columns 3-5 are mm per radian.  Any singular value or
    condition number taken from this matrix therefore depends on a
    characteristic length - see :func:`fk_solve`.

    Returns
    -------
    J : ndarray, shape (6, 6)
        Row ``i`` is leg ``i``; columns are ``(T_x, T_y, T_z, w_x, w_y, w_z)``.
    e : ndarray, shape (3, 6)
    """
    e = rvec / norm
    Rp = R @ geom.p
    J = np.empty((6, 6), dtype=float)
    J[:, :3] = e.T
    J[:, 3:] = np.cross(Rp, e, axis=0).T
    return J, e


def _cond_and_sigma(J: np.ndarray, char_len: float | None):
    """``(cond, sigma_min, sigma_max)`` of ``J`` with the rotation columns scaled.

    ``J``'s rotation columns are mm/rad and its translation columns are
    dimensionless, so its singular values are not comparable as they stand.
    Scaling the rotation columns by ``1 / char_len`` measures the rotation
    increment as an arc length ``char_len * omega`` in mm and makes the whole
    matrix dimensionless.

    ``char_len`` is **not defaulted**.  It is the same undecided choice
    ``docs/archive/notation.md`` sec.12 records for the scoring conditioning measure, and
    picking one here silently would settle it by accident.  Passing ``None``
    returns ``(None, None, None)``.
    """
    if char_len is None:
        return None, None, None
    scale = np.array([1.0, 1.0, 1.0,
                      1.0 / char_len, 1.0 / char_len, 1.0 / char_len])
    sv = np.linalg.svd(J * scale, compute_uv=False)
    smin, smax = float(sv[-1]), float(sv[0])
    return (float(smax / smin) if smin > 0.0 else np.inf), smin, smax


def fk_solve(
    geom: Geometry,
    alphas: np.ndarray,
    R0: np.ndarray,
    T0: np.ndarray,
    *,
    tol: float = FK_TOL_MM,
    max_iter: int = FK_MAX_ITER,
    char_len: float | None = None,
):
    """:func:`fk` with the whole solve record returned instead of just the pose.

    Same solver; ``fk`` is a two-line wrapper.  This is the entry point for the
    gate, which needs the residual, the iteration count, the condition number
    and whether the Levenberg-Marquardt fallback fired.

    Method.  The arm tips are closed form from ``alphas``
    (:func:`arm_tips`), so they are six fixed world points and the problem is
    square: six rod-closure equations ``f_i = 0`` in the six unknowns
    ``(T, omega)``.  Newton, with the analytic Jacobian of
    :func:`fk_jacobian`, a backtracking line search on ``max_i |f_i|``, and
    Levenberg-Marquardt only where Newton fails to reduce that norm.

    ``R`` is carried as a matrix and updated on the **left**,
    ``R <- exp([omega]_x) R``; ``omega`` is a local increment, re-zeroed each
    iteration, never accumulated.  So the iterate never leaves ``SO(3)`` and no
    rotation convention is involved.

    STOPPING is stagnation: iterate until no step - Newton, backtracked
    Newton, or Levenberg-Marquardt - can reduce ``max_i |f_i|`` any further.
    ``tol`` is then applied ONCE, to the converged residual, as an
    **acceptance** test.  It is deliberately not the stopping rule; see
    :data:`FK_TOL_MM` for the measurement that settled that, and note the
    consequence: within the range where acceptance succeeds, the returned pose
    does not depend on ``tol`` at all.

    The LM fallback is **reported, never silent** (``lm_steps`` in the record).
    Newton failing on a square system is a conditioning statement about the
    mechanism at that pose, and hiding it behind a fallback that quietly
    succeeds throws that information away.  Its damping is Marquardt's scaled
    form, ``(J^T J + lam * diag(J^T J)) dx = -J^T f``, rather than
    ``lam * I``: ``I`` would add a millimetre to a radian, which needs exactly
    the characteristic length this module refuses to pick.

    Parameters
    ----------
    geom : Geometry
    alphas : ndarray, shape (6,)
        Servo angles, **radians**.
    R0, T0 : ndarray
        Seed pose, ``(3, 3)`` and ``(3,)``.  The gate seeds HOME - ``R0 = I``,
        ``T0 = (0, 0, z_home)`` - fixed and neutral, never the commanded pose.
        A solver seeded at the answer starts with a zero residual and returns
        immediately, which would let the round trip pass for any ``ik`` at all.
    tol : float
        Residual tolerance, **mm**; see :data:`FK_TOL_MM` for why the value is
        what it is.  Convergence is ``max_i |f_i| <= tol``.
    max_iter : int
        Iteration cap.  Exceeding it raises.
    char_len : float or None
        Characteristic length, mm, used ONLY to make ``cond(J)`` meaningful.
        No default - see :func:`_cond_and_sigma`.

    Returns
    -------
    dict
        ``R``, ``T``, ``residual_mm``, ``iterations``, ``lm_steps``,
        ``cond``, ``sigma_min``, ``sigma_max``, ``char_len``,
        ``so3_drift`` (``max |R^T R - I|`` before the final polar projection),
        ``residual_history``.

    Raises
    ------
    FKNotConverged
        Iteration cap reached, or the step broke down, without meeting ``tol``.
    """
    tips = arm_tips(geom, alphas)
    R = np.array(R0, dtype=float).reshape(3, 3).copy()
    T = np.array(T0, dtype=float).reshape(3).copy()

    lm_steps = 0
    history = []
    reason = "cap"
    res = np.inf

    it = 0
    for it in range(1, int(max_iter) + 1):
        f, rvec, norm = fk_residual(geom, tips, R, T)
        res = float(np.max(np.abs(f)))
        history.append(res)

        if res == 0.0:
            break

        if np.any(norm <= 0.0) or not np.all(np.isfinite(f)):
            raise FKNotConverged(res, it, tol, "singular")

        J, _ = fk_jacobian(geom, R, rvec, norm)

        # ---- Newton, then backtrack on max|f| ---------------------------- #
        step = None
        try:
            step = np.linalg.solve(J, -f)
            if not np.all(np.isfinite(step)):
                step = None
        except np.linalg.LinAlgError:
            step = None

        accepted = None
        if step is not None:
            t = 1.0
            for _ in range(30):                    # 2^-30 ~ 1e-9 of the step
                cand = _apply(R, T, t * step)
                fc, _, _ = fk_residual(geom, tips, *cand)
                if np.max(np.abs(fc)) < res:
                    accepted = cand
                    break
                t *= 0.5

        # ---- Levenberg-Marquardt fallback, counted ----------------------- #
        # Counted only when a step is ACCEPTED.  Every converged solve ends
        # with one iteration where nothing reduces the residual - that is the
        # stagnation stopping rule firing at the arithmetic floor, and Newton
        # "failing" there is not a conditioning event.  Counting the attempt
        # instead of the acceptance reported LM on 96% of solves and would
        # have buried a real conditioning problem in the noise.
        if accepted is None:
            JtJ = J.T @ J
            Jtf = J.T @ f
            diag = np.diag(JtJ).copy()
            diag[diag <= 0.0] = 1.0
            lam = 1e-3
            for _ in range(40):
                try:
                    dx = np.linalg.solve(JtJ + lam * np.diag(diag), -Jtf)
                except np.linalg.LinAlgError:
                    lam *= 10.0
                    continue
                cand = _apply(R, T, dx)
                fc, _, _ = fk_residual(geom, tips, *cand)
                if np.max(np.abs(fc)) < res:
                    accepted = cand
                    lm_steps += 1
                    break
                lam *= 10.0
            if accepted is None:
                # Nothing can reduce the residual any further.  That is the
                # STOPPING rule - the arithmetic floor, or a genuine stall.
                # Which of the two it is, is decided by the acceptance test
                # below, not here.
                reason = "stalled"
                break

        R, T = accepted
    else:
        reason = "cap"

    # ---- acceptance, applied ONCE, to the converged residual ------------- #
    f, rvec, norm = fk_residual(geom, tips, R, T)
    res = float(np.max(np.abs(f)))
    if res > tol:
        raise FKNotConverged(res, it, tol, reason)

    # Final Jacobian, and how far the accumulated exp updates drifted off SO(3)
    # before the projection below.
    J, _ = fk_jacobian(geom, R, rvec, norm)
    cond, smin, smax = _cond_and_sigma(J, char_len)
    drift = float(np.max(np.abs(R.T @ R - np.eye(3))))
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt

    return {
        "R": R,
        "T": T,
        "residual_mm": float(np.max(np.abs(f))),
        "iterations": it,
        "lm_steps": lm_steps,
        "cond": cond,
        "sigma_min": smin,
        "sigma_max": smax,
        "char_len": char_len,
        "so3_drift": drift,
        "residual_history": history,
    }


def _apply(R: np.ndarray, T: np.ndarray, dx: np.ndarray):
    """Apply an increment ``dx = (dT, omega)``: ``T + dT`` and ``exp([w]_x) R``."""
    return exp_so3(dx[3:]) @ R, T + dx[:3]


def fk(
    geom: Geometry,
    alphas: np.ndarray,
    R0: np.ndarray,
    T0: np.ndarray,
    *,
    tol: float = FK_TOL_MM,
    max_iter: int = FK_MAX_ITER,
):
    """Forward kinematics: six servo angles -> platform pose, numerically.

    With the arm tips fixed by ``alphas``, solve for ``(R, T)`` such that every
    rod length equals ``d``.  Newton on the six unsquared rod-closure residuals
    in the six unknowns ``(T, omega)``; see :func:`fk_solve`, which this wraps
    and which returns the residual, the iteration count and the conditioning.

    The seed must **not** be the true pose.  Seeded at the truth the residual
    is already zero, the solver returns immediately, and the round-trip test
    then passes for *any* ``ik`` - including one that returns zeros.
    :func:`stewart.roundtrip.round_trip` offsets the seed for that reason;
    ``stewart/diagnostics/roundtrip.py`` goes further and seeds HOME, a fixed
    neutral pose that knows nothing about the commanded one.

    **A 6-RSS forward kinematics has several real solutions.**  The platform
    can assemble in genuinely different poses from the same six angles.  This
    function returns the root its seed leads to, and a small residual is
    therefore evidence that the legs close - not that the pose is the one that
    was commanded.  Anything comparing an ``fk`` output against a commanded
    pose has to distinguish "wrong" from "a different assembly mode", and the
    residual alone cannot do it.

    Parameters
    ----------
    geom : Geometry
    alphas : ndarray, shape (6,)
        Servo angles, **radians**.
    R0 : ndarray, shape (3, 3)
    T0 : ndarray, shape (3,)
        Seed pose.
    tol : float
        Convergence tolerance on the rod-length residual, mm.  See
        :data:`FK_TOL_MM`.
    max_iter : int
        Iteration cap.

    Returns
    -------
    R : ndarray, shape (3, 3)
    T : ndarray, shape (3,)

    Raises
    ------
    FKNotConverged
        Cap reached, or the step broke down, without meeting ``tol``.  Raised
        rather than returning a best effort.

    Notes
    -----
    The stub docstring listed :class:`Unreachable` here.  It does not apply.
    ``Unreachable`` is a statement about one leg's servo circle failing to
    reach a COMMANDED anchor, which is an inverse-kinematics condition; in
    forward kinematics the anchors are what is being solved for and no such
    per-leg test exists.  The forward failure mode is non-convergence, which
    is what :class:`FKNotConverged` reports.
    """
    out = fk_solve(geom, alphas, R0, T0, tol=tol, max_iter=max_iter)
    return out["R"], out["T"]
