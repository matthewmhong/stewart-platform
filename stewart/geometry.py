"""Ring geometry and the frozen :class:`Geometry` container for a 6-RSS platform.

Millimetres and radians throughout.  Every anchor array is ``(3, 6)`` - column
``i`` is leg ``i``.  Vectors are columns and rotations act from the left
(``R @ p``); a ``(6, 3)`` array would broadcast in many places but silently
apply the rotation transposed, so shapes are checked at construction.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# --------------------------------------------------------------------------- #
# validation helpers
# --------------------------------------------------------------------------- #
def _as_36(name: str, arr) -> np.ndarray:
    """Return ``arr`` as a fresh ``(3, 6)`` float array or raise ``ValueError``."""
    a = np.array(arr, dtype=float)
    if a.shape != (3, 6):
        raise ValueError(
            f"{name} must have shape (3, 6) with column i = leg i; got {a.shape}. "
            f"A (6, 3) array runs without error in many places but silently "
            f"applies every rotation transposed - transpose it before "
            f"constructing Geometry."
        )
    return a


def _bad_legs(mask: np.ndarray) -> str:
    """Comma-joined 1-indexed leg numbers where ``mask`` is true."""
    return ", ".join(str(int(k) + 1) for k in np.nonzero(mask)[0])


def _check_unit(name: str, arr: np.ndarray, tol: float = 1e-6) -> None:
    norms = np.linalg.norm(arr, axis=0)
    bad = np.abs(norms - 1.0) > tol
    if bad.any():
        raise ValueError(
            f"{name}: every column must be a unit vector; leg(s) {_bad_legs(bad)} "
            f"have norm {np.round(norms[bad], 6).tolist()}"
        )


def _check_orthogonal(n: np.ndarray, u: np.ndarray, tol: float = 1e-6) -> None:
    dots = np.sum(n * u, axis=0)
    bad = np.abs(dots) > tol
    if bad.any():
        raise ValueError(
            f"u . n must be 0 for every leg; leg(s) {_bad_legs(bad)} have "
            f"u . n = {np.round(dots[bad], 6).tolist()}"
        )


# --------------------------------------------------------------------------- #
# container
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Geometry:
    """Immutable anchor geometry for a 6-RSS Stewart platform.

    Attributes
    ----------
    p : ndarray, shape (3, 6)
        Platform-frame anchor points {P}, mm.  Column ``i`` = leg ``i``.
    b : ndarray, shape (3, 6)
        World-frame servo-axis centres {W}, mm.
    n : ndarray, shape (3, 6)
        Unit normal of each servo plane.
    u : ndarray, shape (3, 6)
        Unit in-plane reference direction.  The servo angle is measured from
        ``u_i`` toward ``v_i = n_i x u_i``.  ``u . n == 0``.
    a : float
        Servo-arm length, mm (> 0).
    d : float
        Push-rod length, mm (> 0).

    Construction validates shapes, positive lengths, unit norms on ``n`` and
    ``u``, and ``u . n = 0``; failing legs are named 1-indexed.  The four
    arrays are copied and made read-only.
    """

    p: np.ndarray
    b: np.ndarray
    n: np.ndarray
    u: np.ndarray
    a: float
    d: float

    def __post_init__(self) -> None:
        p = _as_36("p", self.p)
        b = _as_36("b", self.b)
        n = _as_36("n", self.n)
        u = _as_36("u", self.u)
        a = float(self.a)
        d = float(self.d)
        if not a > 0.0:
            raise ValueError(f"a (servo-arm length) must be > 0; got {a}")
        if not d > 0.0:
            raise ValueError(f"d (push-rod length) must be > 0; got {d}")
        _check_unit("n", n)
        _check_unit("u", u)
        _check_orthogonal(n, u)
        for field, value in (("p", p), ("b", b), ("n", n), ("u", u)):
            value.flags.writeable = False
            object.__setattr__(self, field, value)
        object.__setattr__(self, "a", a)
        object.__setattr__(self, "d", d)

    def summary(self) -> str:
        """Print (and return) ``a``, ``d`` and the bounds ``|d - a|`` and ``d + a``."""
        lo = abs(self.d - self.a)
        hi = self.d + self.a
        text = (
            "Geometry summary\n"
            f"  servo-arm length  a       = {self.a:9.3f} mm\n"
            f"  push-rod length   d       = {self.d:9.3f} mm\n"
            f"  leg-length bounds |d - a| = {lo:9.3f} mm\n"
            f"                    d + a   = {hi:9.3f} mm"
        )
        print(text)
        return text


# --------------------------------------------------------------------------- #
# base ring  (implemented - transcribed from a finished derivation)
# --------------------------------------------------------------------------- #
def base_ring(r_b: float, beta: float, delta: float):
    """Servo-axis centres and servo-plane frames for the base ring.

    Transcribed from a completed derivation, not re-derived here::

        s        = (-1, +1, -1, +1, -1, +1)
        theta_i  = 120 * floor(i / 2) + s_i * beta          # degrees, i = 0..5
        b_i      = r_b (cos theta_i, sin theta_i, 0)
        psi_i    = theta_i + 90 + s_i * delta
        n_i      = (cos psi_i, sin psi_i, 0)
        u_i      = z x n_i

    Parameters
    ----------
    r_b : float
        Base ring radius, mm (> 0).
    beta : float
        Pair half-split, **degrees**, in the open interval ``(0, 60)``.
        ``beta == 30`` reproduces a regular hexagon.
    delta : float
        Servo-plane yaw offset, **degrees**, in ``[0, 180)``.

    Returns
    -------
    b, n, u : ndarray, each shape (3, 6)

    Raises
    ------
    ValueError
        If ``r_b <= 0``, ``beta`` is not in ``(0, 60)``, or ``delta`` is not
        in ``[0, 180)``.
    """
    r_b = float(r_b)
    beta = float(beta)
    delta = float(delta)
    if not r_b > 0.0:
        raise ValueError(f"r_b must be > 0; got {r_b}")
    if not 0.0 < beta < 60.0:
        raise ValueError(f"beta must be in (0, 60) degrees; got {beta}")
    if not 0.0 <= delta < 180.0:
        raise ValueError(f"delta must be in [0, 180) degrees; got {delta}")

    i = np.arange(6)
    s = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0])

    theta_deg = 120.0 * np.floor(i / 2.0) + s * beta
    psi_deg = theta_deg + 90.0 + s * delta
    theta = np.deg2rad(theta_deg)
    psi = np.deg2rad(psi_deg)

    zc = np.tile(np.array([[0.0], [0.0], [1.0]]), (1, 6))
    b = np.vstack((r_b * np.cos(theta), r_b * np.sin(theta), np.zeros(6)))
    n = np.vstack((np.cos(psi), np.sin(psi), np.zeros(6)))
    u = np.cross(zc, n, axis=0)  # u_i = z x n_i  == (-sin psi_i, cos psi_i, 0)

    assert b.shape == n.shape == u.shape == (3, 6)
    assert np.allclose(np.linalg.norm(n, axis=0), 1.0), "n columns are not unit"
    assert np.allclose(np.linalg.norm(u, axis=0), 1.0), "u columns are not unit"
    assert np.allclose(np.sum(n * u, axis=0), 0.0), "n . u != 0"

    if abs(beta - 30.0) < 1e-9:
        ang = np.sort(np.mod(theta_deg, 360.0))
        gaps = np.diff(np.concatenate((ang, ang[:1] + 360.0)))
        assert np.allclose(gaps, 60.0, atol=1e-7), (
            "beta = 30 must reproduce a regular hexagon; got angular gaps "
            f"{np.round(gaps, 6).tolist()}"
        )

    return b, n, u


# --------------------------------------------------------------------------- #
# stubs  (yours to write)
# --------------------------------------------------------------------------- #
def platform_ring(*args, **kwargs):
    """The six platform-frame anchor points ``p`` (shape ``(3, 6)``).

    Yours to write.  Expected contract: return ``p`` in the platform frame
    {P}, millimetres, column ``i`` = leg ``i``, ordered so that leg ``i`` of
    the platform pairs with leg ``i`` of the base ring.  Mirror
    :func:`base_ring`'s signature style (a radius plus angular parameters in
    degrees, validated on entry).
    """
    raise NotImplementedError("platform_ring is yours to write")


def make_geometry(*args, **kwargs) -> "Geometry":
    """Compose a base ring and a platform ring with ``a`` and ``d``.

    Yours to write.  Call :func:`base_ring` and :func:`platform_ring`, choose
    the servo-arm length ``a`` and push-rod length ``d``, and return
    ``Geometry(p=..., b=..., n=..., u=..., a=..., d=...)`` - construction runs
    every shape and unit-norm check for you.
    """
    raise NotImplementedError("make_geometry is yours to write")


# --------------------------------------------------------------------------- #
# smoke geometry  (NOT a design - shape-checking only)
# --------------------------------------------------------------------------- #
def smoke_geometry(seed: int = 0) -> Geometry:
    """A :class:`Geometry` that passes every construction check and means nothing.

    **NOT A DESIGN.**  The base-ring frames ``(b, n, u)`` come from
    :func:`base_ring` with arbitrary parameters; the platform anchors ``p``
    are a deliberately mismatched, jittered ring; ``a`` and ``d`` are picked
    out of a hat.  It exists only so that :mod:`stewart.plotting`,
    :func:`stewart.plotting.animate` and :func:`stewart.roundtrip.round_trip`
    can be shape-checked before a real :func:`make_geometry` exists.

    Plotted, it **will look wrong**: the platform ring is not aligned to the
    base, the rods do not close, nothing is symmetric.  Do not read any
    geometry off it.
    """
    rng = np.random.default_rng(seed)
    b, n, u = base_ring(r_b=90.0, beta=25.0, delta=12.0)

    ang = np.deg2rad(np.array([-20.0, 25.0, 95.0, 145.0, 215.0, 265.0]))
    r_p = 60.0
    p = np.vstack((r_p * np.cos(ang), r_p * np.sin(ang), np.zeros(6)))
    p = p + rng.normal(scale=3.0, size=(3, 6))

    return Geometry(p=p, b=b, n=n, u=u, a=18.0, d=120.0)
