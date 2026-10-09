"""Plan view of the base plate: servos, brackets, lugs and drill holes.

Writes ``base-layout.png`` and prints the twelve M3 hole centres.  Bracket
dimensions are the as-designed C bracket (``docs/hardware.md`` section 2);
the case and arm come from ``Body`` / ``Horn`` in ``stewart/performance.py``.
"""
from __future__ import annotations

import matplotlib
matplotlib.use("Agg")  # headless: just write the PNG

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon

from demo import DESIGN
from stewart.geometry import make_geometry
from stewart.performance import Body, Horn

PNG = "base-layout.png"

# C bracket, in its own frame: window face at Y = 0, C opening toward +Y,
# Z up, shaft on X = SHAFT_X.  Lug centres (X, Y); holes are M3 clearance.
BRACKET_W, BRACKET_D, SHAFT_X = 20.35, 20.0, 10.175
LUGS = ((-3.0, 3.0), (23.35, 17.0))
LUG_R, HOLE_R = 3.0, 1.6
# Window face behind the rod-end ball plane: 16.0 measured from the arm's
# outer face, plus 6.21 from that face out to the ball centre.
WINDOW_BEHIND_BALL = 16.0 + 6.21

INK, MUTED, GRID = "#1f2328", "#6b7280", "#c9ccd1"
CASE, BRK, ARM, ROD, HOLE = "#d6d9de", "#eef0f2", "#2563eb", "#9ca3af", "#b91c1c"


def bracket_frames(geom, body):
    """Per leg: bracket origin and in-plane axes ``X``, ``Y`` (all ``(3,)``, z = 0)."""
    zhat = np.array([0.0, 0.0, 1.0])
    out = []
    for i in range(6):
        b = geom.b[:, i].copy()
        b[2] = 0.0
        facing = -body.sides[i] * geom.n[:, i]  # the way the horn faces
        Y = -facing
        X = np.cross(Y, zhat)
        origin = b - SHAFT_X * X + WINDOW_BEHIND_BALL * Y
        out.append((b, origin, X, Y, facing))
    return out


def main():
    geom = make_geometry(**DESIGN)
    body, horn = Body(), Horn()
    frames = bracket_frames(geom, body)

    fig, ax = plt.subplots(figsize=(9.5, 9.5), dpi=150)
    ax.set_aspect("equal")

    def poly(pts, **kw):
        ax.add_patch(Polygon([p[:2] for p in pts], closed=True, **kw))

    holes, reach = [], 0.0
    for i, (b, O, X, Y, facing) in enumerate(frames):
        u = geom.u[:, i]
        corners = [O, O + BRACKET_W * X, O + BRACKET_W * X + BRACKET_D * Y, O + BRACKET_D * Y]
        poly(corners, fc=BRK, ec=INK, lw=0.9, zorder=2)
        reach = max(reach, max(np.hypot(*c[:2]) for c in corners))
        for lx, ly in LUGS:
            c = O + lx * X + ly * Y
            holes.append((i + 1, c))
            reach = max(reach, np.hypot(*c[:2]) + LUG_R)
            ax.add_patch(Circle(c[:2], LUG_R, fc=BRK, ec=INK, lw=0.9, zorder=2))
            ax.add_patch(Circle(c[:2], HOLE_R, fc="white", ec=HOLE, lw=1.2, zorder=4))
            ax.plot(*c[:2], "+", color=HOLE, ms=5, mew=0.8, zorder=5)
        poly([b - t * facing + w * u for t, w in ((body.front, -6.15), (body.back, -6.15),
                                                  (body.back, 6.15), (body.front, 6.15))],
             fc=CASE, ec=INK, lw=0.6, zorder=3)
        mid = b - horn.offset * facing
        poly([mid + r * u + t * facing for r, t in ((-horn.behind, -horn.thickness / 2),
                                                    (horn.length - horn.behind, -horn.thickness / 2),
                                                    (horn.length - horn.behind, horn.thickness / 2),
                                                    (-horn.behind, horn.thickness / 2))],
             fc=ARM, ec=ARM, alpha=0.85, zorder=4)
        ax.plot(*b[:2], "o", mfc="white", mec=INK, ms=4, zorder=6)
        tip = b + geom.a * u
        ax.plot([tip[0], geom.p[0, i]], [tip[1], geom.p[1, i]], color=ROD, lw=1, zorder=1)
        ax.plot(*geom.p[:2, i], "o", mfc="white", mec=ROD, ms=4, zorder=1)
        lab = b[:2] / np.linalg.norm(b[:2]) * 30 + b[:2] + 16 * (-facing[:2])
        ax.text(*lab, f"{i + 1}", ha="center", va="center", fontsize=11, weight="bold",
                color=INK, zorder=7)

    ax.add_patch(Circle((0, 0), reach, fill=False, ec=GRID, ls=":", lw=1))
    ax.text(-0.707 * reach, -0.707 * reach, f"minimum plate\nr = {reach:.1f} incl. lugs",
            color=MUTED, fontsize=7, ha="right", va="top")
    ax.add_patch(Circle((0, 0), 80, fill=False, ec=GRID, ls="--", lw=0.8))
    ax.add_patch(Circle((0, 0), DESIGN["r_p"], fill=False, ec=GRID, ls=(0, (1, 3)), lw=0.8))
    ax.annotate("", (35, 0), (0, 0), arrowprops=dict(arrowstyle="->", color=MUTED, lw=1))
    ax.annotate("", (0, 35), (0, 0), arrowprops=dict(arrowstyle="->", color=MUTED, lw=1))
    ax.text(36, 0, "+x", color=MUTED, fontsize=8, va="center")
    ax.text(0, 36, "+y", color=MUTED, fontsize=8, ha="center", va="bottom")
    ax.plot(0, 0, "+", color=INK, ms=8)

    ax.plot([], [], "s", mfc=BRK, mec=INK, label=f"bracket ({BRACKET_W} × {BRACKET_D}) + lugs")
    ax.plot([], [], "s", mfc=CASE, mec=INK, label="servo case (gear boss drawn full width)")
    ax.plot([], [], "s", mfc=ARM, mec=ARM, label="arm extension at home")
    ax.plot([], [], "o", mfc="white", mec=HOLE, label="M3 hole, Ø3.2")
    ax.plot([], [], "o", mfc="white", mec=INK, label="shaft point in the ball plane")
    ax.plot([], [], color=ROD, label="rod, plan view")
    ax.legend(loc="lower right", fontsize=7, frameon=False)
    lim = reach + 8
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_title(f"Base plate, looking down.  Bracket window faces {WINDOW_BEHIND_BALL:.2f} "
                 "behind the ball plane.  mm.", fontsize=10, color=INK, loc="left")
    ax.tick_params(labelsize=7, colors=MUTED)
    for sp in ax.spines.values():
        sp.set_color(GRID)
    ax.grid(color=GRID, lw=0.4, alpha=0.5)
    fig.savefig(PNG, bbox_inches="tight", facecolor="white")

    print("M3 hole centres, base-plate frame (mm):")
    for leg, c in holes:
        print(f"  leg {leg}: ({c[0]:8.2f}, {c[1]:8.2f})")
    print(f"minimum plate radius incl. lugs: {reach:.1f} mm")
    print(f"wrote {PNG}")


if __name__ == "__main__":
    main()
