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
# platform ring
# --------------------------------------------------------------------------- #
def platform_ring(r_p: float, beta_p: float, c_p: float = 0.0):
    """The six platform-frame anchor points.

    Same generating skeleton as :func:`base_ring`, with its own radius and pair
    half-split and no servo frames (the platform carries ball joints, not
    actuators)::

        s       = (-1, +1, -1, +1, -1, +1)
        phi_i   = 120 * floor(i / 2) + s_i * beta_p        # degrees, i = 0..5
        p_i     = (r_p cos phi_i, r_p sin phi_i, -c_p)

    Parameters
    ----------
    r_p : float
        Platform ring radius, mm (> 0).
    beta_p : float
        Pair half-split, **degrees**, in the open interval ``(0, 60)``.
        ``beta_p == 30`` reproduces a regular hexagon.
    c_p : float, default 0.0
        Distance the anchor plane sits **below** ``{P}``'s origin, mm; every
        anchor gets ``z = -c_p``.  ``c_p = 0`` puts the origin in the anchor
        plane.  Not range-checked.

    Returns
    -------
    p : ndarray, shape (3, 6)

    Raises
    ------
    ValueError
        If ``r_p <= 0`` or ``beta_p`` is not in ``(0, 60)``.
 
    Notes
    -----
    There is no separate rotation-relative-to-the-base parameter because a
    continuous one is not admissible.  Requiring the *assembly* to keep D3 -
    the legs, not merely the anchor points - forces the platform's mirror
    lines onto the base's, which quantises the relative rotation to multiples
    of 60 degrees.  Two admissible positions, not a continuum, and ``beta_p``
    straddling 30 reaches both.
 
    The labelling is the real content, and coinciding point sets are not
    enough to establish it.  Rotating ``p`` by 60 degrees with the leg labels
    kept produces the *same six points* as raising ``beta_p`` past 30, but is
    not D3 as an assembly: the mirror then sends base legs 0<->1 while sending
    platform legs 0<->5.  Building ``phi_i`` from the same
    ``120 * floor(i / 2) + s_i * (.)`` skeleton as ``theta_i`` is what makes
    the two permutations come out identical (both ``[1, 0, 5, 4, 3, 2]``), and
    that agreement is what lets one scalar ``delta`` serve all six servo
    planes.
    """
    r_p = float(r_p)
    beta_p = float(beta_p)
    c_p = float(c_p)
    if not r_p > 0.0:
        raise ValueError(f"r_p must be > 0; got {r_p}")
    # The open interval is a HARDWARE exclusion, not a degeneracy one.  Both
    # endpoints are real architectures: beta_p -> 0 (and -> 60) merge the
    # anchors into three pairs, which is the 3-6 Stewart platform.  What rules
    # them out is the ball-joint housing diameter - two housings cannot occupy
    # one hole - so the true lower bound is set by that diameter and is not
    # known yet (archive/docs/notation.md sec.12).  The rank collapse once claimed here was
    # DISPROVED on 2026-09-03: sigma_min stays O(1) as beta_p -> 0 because the
    # shafts stay split, so the six leg lines remain distinct.
    if not 0.0 < beta_p < 60.0:
        raise ValueError(f"beta_p must be in (0, 60) degrees; got {beta_p}")

    i = np.arange(6)
    s = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0])

    phi_deg = 120.0 * np.floor(i / 2.0) + s * beta_p
    phi = np.deg2rad(phi_deg)

    # z = -c_p: the anchors are coplanar, a distance c_p below {P}'s origin
    # (c_p = 0 puts the origin in the anchor plane).  Both are design
    # decisions - see make_geometry's Notes.
    p = np.vstack((r_p * np.cos(phi), r_p * np.sin(phi), np.full(6, -c_p)))

    assert p.shape == (3, 6)
    assert np.allclose(np.hypot(p[0], p[1]), r_p), "p columns are off the ring"
    assert np.allclose(p[2], -c_p), "p columns are off the anchor plane"
 
    if abs(beta_p - 30.0) < 1e-9:
        ang = np.sort(np.mod(phi_deg, 360.0))
        gaps = np.diff(np.concatenate((ang, ang[:1] + 360.0)))
        assert np.allclose(gaps, 60.0, atol=1e-7), (
            "beta_p = 30 must reproduce a regular hexagon; got angular gaps "
            f"{np.round(gaps, 6).tolist()}"
        )
 
    return p
 
 
# --------------------------------------------------------------------------- #
# composition
# --------------------------------------------------------------------------- #
def make_geometry(r_b: float, beta: float, delta: float,
                  r_p: float, beta_p: float,
                  a: float, d: float, c_p: float = 0.0) -> "Geometry":
    """Compose a base ring and a platform ring with given ``a`` and ``d``.

    Plumbing only: calls :func:`base_ring` and :func:`platform_ring`, passes
    the servo frames straight through, and hands everything to
    :class:`Geometry`, which validates it.

    Parameters
    ----------
    r_b, beta, delta : float
        Passed to :func:`base_ring`, which validates them.
    r_p, beta_p : float
        Passed to :func:`platform_ring`, which validates them.
    a, d : float
        Servo-arm and push-rod lengths, mm.  **Required.**  See Notes.
    c_p : float, default 0.0
        Anchor-plane drop below ``{P}``'s origin, mm; passed to
        :func:`platform_ring`.  ``0.0`` keeps the origin in the anchor plane.
 
    Returns
    -------
    Geometry
 
    Raises
    ------
    ValueError
        Propagated from :func:`base_ring`, :func:`platform_ring` or
        :class:`Geometry`.
 
    Notes
    -----
    **Two assumptions are baked in and neither is derived.**
 
    First, ``p`` has ``z = -c_p`` throughout, which bundles two decisions: the
    six anchors are coplanar (a flat plate rather than a dished or stepped
    one), and ``{P}``'s origin sits a fixed ``c_p`` above that plane (``c_p``
    is a parameter now, but a single scalar - the plate is still flat).  The
    offset is the one with teeth, because the commanded ``T`` is the position
    of ``{P}``'s origin.  If the ball rolls on a surface above the anchor
    plane, ``T`` is not the position of the rolling surface and every
    commanded height is offset by the remaining gap.  Anchor plane, plate top
    and ball centre are three different origins and ``c_p`` only reconciles the
    first with ``{P}``.
 
    Second, nothing enforces ``d > a``.  An earlier version raised on it with
    the justification that the rod could not otherwise clear the arm circle,
    which is wrong: the reachable annulus ``|a - rho| <= C <= a + rho`` is
    well defined either way, and ``d < a`` merely moves the inner bound out.
    ``d > a`` is what any platform sitting well above its base will satisfy by
    a wide margin, not a validity condition, so it is documented rather than
    checked.  Ball-joint angular travel is the real constraint in that corner
    and it belongs to the component, not the geometry.
 
    ``a`` and ``d`` are arguments, not choices made here, which is a departure
    from the stub docstring.  Two reasons.  ``a`` is restricted to servo horns
    that commercially exist, so any continuous formula for it pre-empts the
    servo shortlist.  And a formula would have to be justified, which makes it
    a design decision rather than plumbing.
 
    For the record, one rule was tried and does not do what it looks like it
    does.  Setting ``a^2 + d^2 = |L_home|^2`` gives ``P = a`` exactly, since
    ``P = (|L|^2 + a^2 - d^2) / 2a`` and the two ``|L|^2`` cancel.  That is a
    clean result but it does **not** make the home pose reachable.  Writing
    ``a = k |L|``, the condition ``|P| <= C`` becomes::
 
        a <= C = sqrt((L . u)^2 + z_home^2)     i.e.   |w| <= |L| sqrt(1 - k^2)
 
    which is a statement about how far the leg lies out of its servo plane, so
    it depends on ``delta`` and can fail.  The two-sphere bound
    ``|d - a| < |L| < d + a`` is necessary but not sufficient and must not be
    used to argue otherwise.
 
    There is no rank guard here, and ``beta_p == beta`` is **not** the reason
    there might need to be one.  This paragraph used to claim that case was an
    affine architecture singularity spanning only three of the six wrench
    dimensions, and that the test could not be written until a solver and a
    branch rule existed.  **Both halves were wrong.**  The correction is kept
    here because the mistake is an easy one to repeat.

    Measured 2026-09-04 with the true rod lines ``q_i - h_i`` (``h_i`` from
    :func:`~stewart.kinematics.ik`, whose branch is fixed): at
    ``beta_p == beta`` the wrench matrix is **full rank 6**.  Writing
    ``e = beta_p - beta``, ``sigma_min`` increases *monotonically* through
    ``e = 0`` - not a dip, not a local minimum, not distinguished at all.  It
    scales linearly in the arm length, ``sigma_min / a`` holding near 0.93
    across a 14x span of ``a``, so the rank is bought by the arm and
    degenerates only as ``a -> 0``.

    The rank-3 result came from the proxy screw axis ``q_i - b_i``.  At
    ``beta_p == beta`` the anchors are ``p_i = c b_i`` in-plane with
    ``c = r_p / r_b``, so at ``R = I``, ``T = (0, 0, z_home)`` the proxy line
    through ``b_i`` is::

        X_i(t) = b_i [1 + t(c - 1)] + t (0, 0, z_home - c_p)

    whose ``b_i`` coefficient vanishes at ``t = 1 / (1 - c)``, leaving::

        X = (0, 0, -(z_home - c_p) / (c - 1))        independent of i

    All six proxy lines pass through that one point - verified concurrent to
    5e-16 - and six concurrent lines span only three dimensions.  The rank
    drop was a property of the proxy, not of the mechanism.  The true rod
    lines are not concurrent; a best-fit common point leaves a residual of
    order ``a``.

    The underlying error was importing a 6-UPS intuition.  There the leg
    genuinely *is* the ``b -> q`` line, so an affine anchor map really does
    give an architecture singularity.  In a 6-RSS the constraint is the rod,
    the arm stands between ``b_i`` and it, and the result does not transfer.

    ``q_i - b_i`` is still wrong by the arm for any conditioning measure built
    on it.  The angle between the two at ``q`` depends only on the three side
    lengths ``a``, ``d`` and ``|L|``, by the law of cosines on the triangle
    ``b h q``::
 
        cos(angle at q) = (d^2 + |L|^2 - a^2) / (2 d |L|)
        angle at q      <= arcsin(a / |L|)     equality iff the elbow is square
 
    which runs about 1 to 27 degrees over ``a`` in [15, 60] mm and ``d`` in
    [105, 155] mm on a 134 mm home leg.  Rejecting candidates is a scoring
    decision in any case.
    """
    b, n, u = base_ring(r_b, beta, delta)
    p = platform_ring(r_p, beta_p, c_p)
    return Geometry(p=p, b=b, n=n, u=u, a=a, d=d)

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
