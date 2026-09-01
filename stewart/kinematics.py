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


def stage1(geom: Geometry, q: np.ndarray):
    """First IK stage: the branch-independent per-leg coefficients.

    Given world-frame platform anchors ``q`` (shape ``(3, 6)``), form, for
    each leg, the coefficients of

        A_i cos(alpha_i) + B_i sin(alpha_i) = P_i

    with (``L_i = q_i - b_i``, ``v_i = n_i x u_i``)

        A_i = 2 a (L_i . u_i)
        B_i = 2 a (L_i . v_i)
        P_i = |L_i|^2 + a^2 - d^2
        C_i = hypot(A_i, B_i)

    ``C_i`` is the amplitude used by the reachability test ``|P_i| > C_i``
    (see :func:`ik`).

    Parameters
    ----------
    geom : Geometry
    q : ndarray, shape (3, 6)
        Platform anchors in the world frame, mm.

    Returns
    -------
    A, B, P, C : ndarray, each shape (6,)
    """
    raise NotImplementedError


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
    raise NotImplementedError


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
    raise NotImplementedError


def ik(geom: Geometry, R: np.ndarray, T: np.ndarray, *, elbow: str = "up") -> np.ndarray:
    """Inverse kinematics: platform pose -> six servo angles.

    For leg ``i``::

        q_i      = R @ p_i + T                                anchor, world frame
        L_i      = q_i - b_i                                  servo centre -> anchor
        v_i      = n_i x u_i                                  in-plane basis with u_i
        tip_i(a) = b_i + a (cos a * u_i + sin a * v_i)        a = servo angle

    The rod constraint ``|q_i - tip_i(alpha_i)| = d`` reduces to

        A_i cos(alpha_i) + B_i sin(alpha_i) = P_i

        A_i = 2 a (L_i . u_i)
        B_i = 2 a (L_i . v_i)
        P_i = |L_i|^2 + a^2 - d^2
        C_i = hypot(A_i, B_i)

    solved as ``alpha_i = atan2(B_i, A_i) +/- acos(P_i / C_i)``, the sign
    chosen by ``elbow``.

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
    elbow : {"up", "down"}
        Which ``acos`` branch / servo-arm configuration to take.

    Returns
    -------
    alphas : ndarray, shape (6,)
        Servo angles, **radians**, measured from ``u_i`` toward ``v_i``.

    Raises
    ------
    Unreachable
        If ``|P_i| > C_i`` for any leg.  Carries the 1-indexed leg and the
        direction (``"far"`` if ``P_i > C_i``, ``"near"`` if ``P_i < -C_i``).
    """
    raise NotImplementedError


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
