"""Drawing helpers for a 6-RSS Stewart platform.

Nothing here solves kinematics.  Every function is handed points - or a servo
angle, which only places a point on a circle that is already known from the
geometry - and draws them.  Millimetres in; degrees appear only on the servo-
angle plot.
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as manim
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  registers the '3d' projection


_RING = list(range(6)) + [0]  # column order for a closed ring

_C_BASE = "#555555"
_C_PLAT = "#1f77b4"
_C_ARM = "#d62728"
_C_ROD = "#333333"
_C_GUIDE = "#999999"


# --------------------------------------------------------------------------- #
# internal geometry (point placement, not a kinematic solve)
# --------------------------------------------------------------------------- #
def _v(geom):
    """In-plane basis partner of ``u``: ``v_i = n_i x u_i``."""
    return np.cross(geom.n, geom.u, axis=0)


def _arm_tip_points(geom, h):
    """Place each arm tip on its circle for servo angle ``h_i`` (radians).

    ``tip_i = b_i + a (cos h_i * u_i + sin h_i * v_i)``.  This is point
    placement on a circle fixed by the geometry, not a kinematic solve.
    """
    h = np.asarray(h, dtype=float).reshape(-1)
    if h.shape != (6,):
        raise ValueError(f"h must have 6 entries; got shape {h.shape}")
    v = _v(geom)
    return geom.b + geom.a * (np.cos(h) * geom.u + np.sin(h) * v)


def _cube(point_sets, pad=0.05):
    """Centre and half-width of a cube enclosing every column of every array."""
    allp = np.concatenate(
        [np.asarray(p, dtype=float).reshape(3, -1) for p in point_sets], axis=1
    )
    lo = allp.min(axis=1)
    hi = allp.max(axis=1)
    centre = 0.5 * (lo + hi)
    radius = 0.5 * float(np.max(hi - lo))
    radius = radius * (1.0 + pad) if radius > 0.0 else 1.0
    return centre, radius


def _apply_cube(ax, centre, radius):
    ax.set_xlim(centre[0] - radius, centre[0] + radius)
    ax.set_ylim(centre[1] - radius, centre[1] + radius)
    ax.set_zlim(centre[2] - radius, centre[2] + radius)
    try:  # matplotlib >= 3.3
        ax.set_box_aspect((1.0, 1.0, 1.0))
    except Exception:
        pass


def _triad(ax, origin, R, scale, name):
    o = np.asarray(origin, dtype=float).reshape(3)
    R = np.asarray(R, dtype=float).reshape(3, 3)
    for k, colour in enumerate((_C_ARM, "#2ca02c", _C_PLAT)):
        d = R[:, k] * scale
        ax.quiver(o[0], o[1], o[2], d[0], d[1], d[2], color=colour, linewidth=1.5)
    ax.text(o[0], o[1], o[2], f"  {{{name}}}", fontsize=9)


# --------------------------------------------------------------------------- #
# public API
# --------------------------------------------------------------------------- #
def draw_pose(geom, q, h=None, T=None, R=None, ax=None, title=None, labels=True):
    """Draw one platform pose.

    Parameters
    ----------
    geom : Geometry
    q : ndarray, shape (3, 6)
        Platform anchors in the world frame, mm.
    h : ndarray, shape (6,), optional
        Servo angles, radians.  Given -> draw servo arms ``b_i -> tip_i`` and
        rods ``tip_i -> q_i``.  Omitted -> draw dashed ``b_i -> q_i`` guide
        lines instead.
    T, R : ndarray, optional
        Platform origin ``(3,)`` and orientation ``(3, 3)``.  Both given ->
        draw the ``{P}`` triad.  The ``{W}`` triad is always drawn at the
        world origin.
    ax : mpl 3d Axes, optional
        Draw into this axes; otherwise a new figure and axes are made.
    title : str, optional
    labels : bool
        Number the legs 1..6 at the base anchors.

    Returns
    -------
    ax : the 3d Axes drawn into.
    """
    q = np.asarray(q, dtype=float)
    if q.shape != (3, 6):
        raise ValueError(f"q must have shape (3, 6); got {q.shape}")
    b = geom.b

    if ax is None:
        fig = plt.figure(figsize=(7.0, 6.0))
        ax = fig.add_subplot(111, projection="3d")

    ax.plot(b[0, _RING], b[1, _RING], b[2, _RING],
            color=_C_BASE, linewidth=2.0)
    ax.plot(q[0, _RING], q[1, _RING], q[2, _RING],
            color=_C_PLAT, linewidth=2.0)

    point_sets = [b, q]

    if h is not None:
        tips = _arm_tip_points(geom, h)
        point_sets.append(tips)
        for i in range(6):
            ax.plot([b[0, i], tips[0, i]], [b[1, i], tips[1, i]],
                    [b[2, i], tips[2, i]], color=_C_ARM, linewidth=2.0)
            ax.plot([tips[0, i], q[0, i]], [tips[1, i], q[1, i]],
                    [tips[2, i], q[2, i]], color=_C_ROD, linewidth=1.0)
    else:
        for i in range(6):
            ax.plot([b[0, i], q[0, i]], [b[1, i], q[1, i]], [b[2, i], q[2, i]],
                    color=_C_GUIDE, linewidth=1.0, linestyle="--")

    if labels:
        for i in range(6):
            ax.text(b[0, i], b[1, i], b[2, i], f"  {i + 1}",
                    fontsize=9, color=_C_BASE)

    ring_r = float(np.linalg.norm(b, axis=0).max())
    tri = 0.35 * ring_r if ring_r > 0.0 else 1.0
    _triad(ax, np.zeros(3), np.eye(3), tri, "W")
    if R is not None and T is not None:
        _triad(ax, T, R, tri, "P")

    centre, radius = _cube(point_sets)
    _apply_cube(ax, centre, radius)
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.set_zlabel("z [mm]")
    if title:
        ax.set_title(title)
    return ax


def arm_circles(geom, ax, n_points=64):
    """Overlay the circle each servo-arm tip sweeps.

    Radius ``a`` about each ``b_i``, in that servo's plane, parameterised by
    ``u_i`` and ``v_i = n_i x u_i``.

    Returns
    -------
    list of Line3D
    """
    t = np.linspace(0.0, 2.0 * np.pi, int(n_points))
    v = _v(geom)
    handles = []
    for i in range(6):
        circ = (
            geom.b[:, i:i + 1]
            + geom.a * (np.cos(t) * geom.u[:, i:i + 1] + np.sin(t) * v[:, i:i + 1])
        )
        (line,) = ax.plot(circ[0], circ[1], circ[2],
                          color=_C_ARM, linewidth=0.8, alpha=0.5)
        handles.append(line)
    return handles


def compare_branches(geom, q, h_plus, h_minus, titles=None):
    """Two side-by-side subplots: the same pose ``q``, the two arm branches.

    Both axes share one (unioned) cube of limits.

    Returns
    -------
    (fig, (ax_left, ax_right))
    """
    q = np.asarray(q, dtype=float)
    if titles is None:
        titles = ("elbow +", "elbow -")
    fig = plt.figure(figsize=(12.0, 6.0))
    axl = fig.add_subplot(121, projection="3d")
    axr = fig.add_subplot(122, projection="3d")

    draw_pose(geom, q, h=h_plus, ax=axl, title=titles[0])
    draw_pose(geom, q, h=h_minus, ax=axr, title=titles[1])

    tips_p = _arm_tip_points(geom, h_plus)
    tips_m = _arm_tip_points(geom, h_minus)
    centre, radius = _cube([geom.b, q, tips_p, tips_m])
    _apply_cube(axl, centre, radius)
    _apply_cube(axr, centre, radius)
    return fig, (axl, axr)


def animate(geom, q_frames, h_frames=None, interval_ms=40):
    """Animate a sequence of platform poses.

    Parameters
    ----------
    geom : Geometry
    q_frames : ndarray, shape (F, 3, 6)
        World-frame platform anchors per frame.
    h_frames : ndarray, shape (F, 6), optional
        Servo angles per frame.
    interval_ms : int
        Delay between frames.

    Axis limits are fixed (a single cube over every frame) so the motion is
    not hidden by autoscaling.

    Returns
    -------
    (fig, anim)
    """
    q_frames = np.asarray(q_frames, dtype=float)
    if q_frames.ndim != 3 or q_frames.shape[1:] != (3, 6):
        raise ValueError(f"q_frames must have shape (F, 3, 6); got {q_frames.shape}")
    n_frames = q_frames.shape[0]

    have_h = h_frames is not None
    if have_h:
        h_frames = np.asarray(h_frames, dtype=float)
        if h_frames.shape != (n_frames, 6):
            raise ValueError(
                f"h_frames must have shape ({n_frames}, 6); got {h_frames.shape}"
            )

    pool = [geom.b] + [q_frames[k] for k in range(n_frames)]
    if have_h:
        pool += [_arm_tip_points(geom, h_frames[k]) for k in range(n_frames)]
    centre, radius = _cube(pool)

    fig = plt.figure(figsize=(7.0, 6.0))
    ax = fig.add_subplot(111, projection="3d")

    def _update(k):
        ax.cla()
        draw_pose(
            geom, q_frames[k],
            h=(h_frames[k] if have_h else None),
            ax=ax, title=f"frame {k + 1}/{n_frames}",
        )
        _apply_cube(ax, centre, radius)
        return ()

    anim = manim.FuncAnimation(
        fig, _update, frames=n_frames, interval=interval_ms, blit=False
    )
    return fig, anim


def plot_angles(t, alphas):
    """Six servo-angle traces.  ``alphas`` in radians, plotted in degrees.

    Parameters
    ----------
    t : ndarray, shape (N,)
    alphas : ndarray, shape (6, N) or (N, 6)

    Returns
    -------
    (fig, ax)
    """
    t = np.asarray(t, dtype=float).reshape(-1)
    A = np.asarray(alphas, dtype=float)
    if A.shape == (t.size, 6) and A.shape != (6, t.size):
        A = A.T
    if A.shape != (6, t.size):
        raise ValueError(
            f"alphas must be (6, {t.size}) or ({t.size}, 6); got {A.shape}"
        )
    fig, ax = plt.subplots(figsize=(8.0, 4.0))
    for i in range(6):
        ax.plot(t, np.rad2deg(A[i]), label=f"leg {i + 1}")
    ax.set_xlabel("t")
    ax.set_ylabel("servo angle [deg]")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)
    return fig, ax
