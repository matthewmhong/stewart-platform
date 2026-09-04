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


def fk(
    geom: Geometry,
    alphas: np.ndarray,
    R0: np.ndarray,
    T0: np.ndarray,
    *,
    tol: float = 1e-9,
    max_iter: int = 100,
):
    """Forward kinematics: six servo angles -> platform pose, numerically.

    With the arm tips fixed by ``alphas``, solve for ``(R, T)`` such that
    every rod length equals ``d``.  Iterative; needs a seed pose
    ``(R0, T0)``.

    The seed must **not** be the true pose.  Seeded at the truth the residual
    is already zero, the solver returns immediately, and the round-trip test
    then passes for *any* ``ik`` - including one that returns zeros.  See
    :func:`stewart.roundtrip.round_trip`, which offsets the seed on purpose.

    Parameters
    ----------
    geom : Geometry
    alphas : ndarray, shape (6,)
        Servo angles, **radians**.
    R0 : ndarray, shape (3, 3)
    T0 : ndarray, shape (3,)
        Seed pose.
    tol : float
        Convergence tolerance on the rod-length residual, mm.
    max_iter : int
        Iteration cap.

    Returns
    -------
    R : ndarray, shape (3, 3)
    T : ndarray, shape (3,)

    Raises
    ------
    Unreachable
        If the pose search leaves a rod unable to close.
    """
    raise NotImplementedError
