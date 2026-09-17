"""Check one design against the pass/fail requirements in ``STATUS.md``.

`STATUS.md` states R1-R3 in plate-tilt terms and estimates the gearing as
``G ~ a / r_p``.  This module replaces that estimate with the real kinematics:
every sensitivity here is a numerical derivative of :func:`stewart.kinematics.ik`
or :func:`stewart.kinematics.fk`, so it sees the per-leg variation and the pose
dependence that a single scalar ``G`` cannot.

Requirements checked (values from ``STATUS.md``, 2026-09-16):

==  ==========================================================  =============
R1  tilt ``req.tilt_deg`` in every azimuth, at home height       >= 4.5 deg
R2  tilt error from deadband + joint play, in quadrature         <= 0.25 deg
R3  tilt rate at the binding leg                                 >= 65 deg/s
J   rod-end misalignment stays inside the joint's cone           <= cone_deg
==  ==========================================================  =============

Conventions follow the rest of the package: mm and radians internally, degrees
only in the report; anchors are ``(3, 6)``; failing legs are named 1-indexed.

Three approximations, all deliberate and all visible here rather than buried:

1. **Joint play enters as an equivalent servo-angle error** ``j / a`` rad.
   :class:`~stewart.geometry.Geometry` carries one scalar ``d``, so a per-leg
   rod-length error cannot be expressed directly.  The substitution is exact
   when the rod is perpendicular to the arm and optimistic as that angle
   closes; :func:`worst_rod_arm_angle` reports the worst angle in the envelope
   so the optimism is measurable.
2. **R2 is judged in quadrature over the six legs** (decided 2026-09-16).
   Within one leg the errors add, as ``STATUS.md``'s budget says; across legs
   they are independent, so summing them assumes all six conspire in the same
   tilt direction at the same instant.  The worst-case sum is still computed
   and reported - it is the wear-and-surprise reserve - but ``Report.ok``
   judges on ``tilt_error_rss_deg``.
3. **Backlash is taken as zero**, measured 2026-09-16 under one-way preload.
   Pass ``servo.backlash_deg`` if that assumption is ever revisited.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .geometry import Geometry, make_geometry
from .kinematics import FKNotConverged, Unreachable, arm_tips, fk, ik

# --------------------------------------------------------------------------- #
# measured hardware and the requirements it has to meet
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Servo:
    """One servo's measured behaviour.  Degrees, because that is how it was measured.

    Defaults are the MG90S bench test of 2026-09-16 (``STATUS.md``): 0.087
    deg/us, a 4 us deadband, 530-2480 us of travel about a 1505 us centre, and
    60 deg in 0.24 s at ~150 g.cm.
    """

    deadband_deg: float = 0.35
    backlash_deg: float = 0.0  # ~0 under one-way preload, measured
    travel_deg: float = 83.0  # +/- about centre, after a 20 us back-off
    speed_dps: float = 250.0


@dataclass(frozen=True)
class Requirements:
    """The pass/fail numbers.  Defaults are ``tau = 0.7 s``, ``e = +/-8 mm``."""

    tilt_deg: float = 4.5  # R1
    precision_deg: float = 0.25  # R2
    rate_dps: float = 65.0  # R3
    joint_play_mm: float = 0.1  # total per leg
    cone_deg: float = 13.0  # rod-end misalignment limit, MEASURE THIS
    n_azimuth: int = 24  # envelope resolution


@dataclass
class Report:
    """What :func:`evaluate` found.  ``ok`` is the conjunction of the four checks."""

    ok: bool = False
    r1_ok: bool = False
    r2_ok: bool = False
    r3_ok: bool = False
    cone_ok: bool = False
    max_tilt_deg: float = float("nan")
    tilt_error_deg: float = float("nan")
    tilt_error_rss_deg: float = float("nan")
    rate_dps: float = float("nan")
    max_misalign_deg: float = float("nan")
    max_servo_deg: float = float("nan")
    worst_rod_arm_deg: float = float("nan")
    home_z_mm: float = float("nan")
    axes_base: np.ndarray | None = None
    axes_platform: np.ndarray | None = None
    notes: list[str] = field(default_factory=list)

    def summary(self) -> str:
        def mark(ok: bool) -> str:
            return "PASS" if ok else "FAIL"

        text = (
            f"  home height              {self.home_z_mm:8.2f} mm\n"
            f"  R1 tilt reached          {self.max_tilt_deg:8.2f} deg   {mark(self.r1_ok)}\n"
            f"  R2 tilt error (judged)   {self.tilt_error_rss_deg:8.3f} deg   {mark(self.r2_ok)}\n"
            f"     (worst case, reserve)  {self.tilt_error_deg:8.3f} deg\n"
            f"  R3 tilt rate             {self.rate_dps:8.1f} deg/s {mark(self.r3_ok)}\n"
            f"  J  rod-end misalignment  {self.max_misalign_deg:8.2f} deg   {mark(self.cone_ok)}\n"
            f"  servo angle used         {self.max_servo_deg:8.2f} deg\n"
            f"  worst rod-arm angle      {self.worst_rod_arm_deg:8.2f} deg"
        )
        if self.notes:
            text += "\n  " + "\n  ".join(self.notes)
        return text


# --------------------------------------------------------------------------- #
# poses
# --------------------------------------------------------------------------- #


def tilt_pose(theta: float, azimuth: float, z: float):
    """Pose for ``theta`` rad of tilt about the horizontal axis at ``azimuth``.

    Tilt only - no translation in the plane, no yaw - which is the envelope
    ``STATUS.md`` fixes.  The rotation axis is ``(-sin az, cos az, 0)``, so the
    plate's high side faces ``azimuth``.
    """
    axis = np.array([-np.sin(azimuth), np.cos(azimuth), 0.0])
    k = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
    )
    R = np.eye(3) + np.sin(theta) * k + (1.0 - np.cos(theta)) * (k @ k)
    return R, np.array([0.0, 0.0, z])


def home_pose(geom: Geometry):
    """The datum pose: every servo angle zero.

    Found by forward kinematics from ``alphas = 0``, seeded with a rod-closure
    estimate of the height.  Returns ``(R, T)``; ``R`` is identity to solver
    tolerance for a symmetric design but is returned as solved, not assumed.
    """
    tips = arm_tips(geom, np.zeros(6))
    horiz = np.hypot(tips[0] - geom.p[0], tips[1] - geom.p[1])
    inside = geom.d**2 - horiz**2
    if np.any(inside <= 0.0):
        bad = " ".join(str(i + 1) for i in np.flatnonzero(inside <= 0.0))
        raise ValueError(f"rods too short to close at the datum; legs {bad}")
    z_seed = float(np.mean(np.sqrt(inside)) - float(np.mean(geom.p[2])))
    return fk(geom, np.zeros(6), np.eye(3), np.array([0.0, 0.0, z_seed]))


def tilt_of(R: np.ndarray) -> float:
    """Angle between the plate normal and vertical, rad."""
    return float(np.arccos(np.clip(R[2, 2], -1.0, 1.0)))


# --------------------------------------------------------------------------- #
# the four checks
# --------------------------------------------------------------------------- #


def _envelope(req: Requirements):
    return np.linspace(0.0, 2.0 * np.pi, req.n_azimuth, endpoint=False)


def reach(geom: Geometry, servo: Servo, req: Requirements, z: float):
    """R1: can every azimuth hold ``req.tilt_deg`` within the servo's travel?

    Returns ``(ok, max_servo_deg, notes)``.  A leg that cannot reach raises
    :class:`~stewart.kinematics.Unreachable` inside ``ik``; it is caught and
    reported rather than propagated, because a failing candidate is an ordinary
    outcome of a search, not an error.
    """
    theta = np.radians(req.tilt_deg)
    limit = np.radians(servo.travel_deg)
    worst = 0.0
    notes: list[str] = []
    for az in _envelope(req):
        R, T = tilt_pose(theta, az, z)
        try:
            alphas = ik(geom, R, T)
        except Unreachable as exc:
            notes.append(
                f"R1: leg {exc.leg} unreachable ({exc.direction}) at azimuth "
                f"{np.degrees(az):.0f} deg"
            )
            return False, float("nan"), notes
        worst = max(worst, float(np.max(np.abs(alphas))))
        if worst > limit:
            notes.append(
                f"R1: servo travel exceeded at azimuth {np.degrees(az):.0f} deg "
                f"({np.degrees(worst):.1f} > {servo.travel_deg} deg)"
            )
            return False, np.degrees(worst), notes
    return True, np.degrees(worst), notes


def _dalpha_dtilt(geom: Geometry, req: Requirements, z: float, az: float,
                  h: float = 1e-4):
    """Central difference of ``ik`` in tilt: ``d alpha_i / d theta``, rad/rad."""
    plus = ik(geom, *tilt_pose(h, az, z))
    minus = ik(geom, *tilt_pose(-h, az, z))
    return (plus - minus) / (2.0 * h)


def tilt_rate(geom: Geometry, servo: Servo, req: Requirements, z: float):
    """R3: fastest plate tilt rate, set by the leg needing the most servo travel.

    ``rate = omega / max_i |d alpha_i / d theta|`` over the envelope.  This is
    the real gearing; ``STATUS.md``'s ``G ~ a / r_p`` is its scalar caricature.
    """
    worst = 0.0
    for az in _envelope(req):
        worst = max(worst, float(np.max(np.abs(_dalpha_dtilt(geom, req, z, az)))))
    if worst == 0.0:
        return float("inf")
    return servo.speed_dps / worst


def tilt_error(geom: Geometry, servo: Servo, req: Requirements, z: float):
    """R2: plate tilt error from deadband, backlash and joint play.

    Returns ``(worst_case, quadrature)`` in degrees.  The six legs' errors are
    independent, so which one to judge against R2 is a modelling choice:
    worst case assumes they conspire, quadrature assumes they are random and
    uncorrelated.  Worst case is what ``STATUS.md``'s budget implies; both are
    reported because the answer can fall between them.

    The sensitivity ``d tilt / d alpha_i`` comes from perturbing one servo and
    solving ``fk`` - the only honest way to get it, since a servo error moves
    the plate in all six degrees of freedom, not just the two being commanded.
    Errors are summed over legs (worst case, per the R2 budget).
    """
    eps = np.radians(0.5 * (servo.deadband_deg + servo.backlash_deg))
    play = req.joint_play_mm / geom.a  # rad, equivalent servo error
    R0, T0 = home_pose(geom)
    base_tilt = tilt_of(R0)
    sens = np.zeros(6)
    for i in range(6):
        alphas = np.zeros(6)
        probe = np.radians(0.5)  # big enough to beat fk's tolerance
        alphas[i] = probe
        R, _ = fk(geom, alphas, R0, T0)
        sens[i] = abs(tilt_of(R) - base_tilt) / probe  # rad tilt per rad servo
    err = eps + play
    return np.degrees(sens.sum() * err), np.degrees(np.linalg.norm(sens) * err)


def _best_axis(dirs: np.ndarray):
    """Bolt axis most perpendicular to a set of rod directions, and the worst
    deviation from perpendicular it leaves, in degrees.

    ``dirs`` is ``(3, k)`` of unit rod directions **in the frame the bolt is
    fixed to**.  The axis is the smallest left-singular vector, i.e. the
    direction the rods align with least; it minimises the sum of squared dot
    products, which is the tractable proxy for minimising the worst one.
    """
    u, _, _ = np.linalg.svd(dirs)
    axis = u[:, -1]
    dev = np.degrees(np.arcsin(np.clip(np.abs(axis @ dirs), -1.0, 1.0)))
    return axis, float(dev.max())


def misalignment(geom: Geometry, req: Requirements, z: float):
    """J: worst rod-end misalignment over the envelope, with the best bolt axes.

    A rod end is a ball in a housing: the rod nominally sits **perpendicular**
    to the bolt through the ball, and binds once it leaves a cone of a few
    degrees either side of perpendicular.

    The bolt orientation at each end is a **design choice**, not a given - a
    tab can be drilled at any angle - so this searches for the best axis per
    leg per end and reports what that best mounting still costs.  Each end is
    worked in the frame the bolt is fixed to, because that is the frame the
    rod sweeps relative to:

    - **base end**: the bolt turns with the servo arm, so rods are expressed in
      the arm frame ``(u_i, v_i, n_i)`` de-rotated by the servo angle.
    - **platform end**: the bolt is fixed to the plate, so rods are expressed
      in the plate frame, ``R.T @ rod``.

    Returns ``(worst_deg, axes_base, axes_platform)``; the axes are ``(3, 6)``
    and are what the CAD needs.
    """
    base_dirs = [[] for _ in range(6)]
    plat_dirs = [[] for _ in range(6)]
    v = np.cross(geom.n, geom.u, axis=0)
    for az in _envelope(req):
        R, T = tilt_pose(np.radians(req.tilt_deg), az, z)
        alphas = ik(geom, R, T)
        tips = arm_tips(geom, alphas)
        rods = R @ geom.p + T[:, None] - tips
        rods = rods / np.linalg.norm(rods, axis=0)
        plate_rods = R.T @ rods
        for i in range(6):
            # de-rotate into the arm frame: components along (u, v, n), then
            # undo the servo rotation by -alpha_i in the (u, v) plane
            cu, cv, cn = geom.u[:, i] @ rods[:, i], v[:, i] @ rods[:, i], geom.n[:, i] @ rods[:, i]
            ca, sa = np.cos(-alphas[i]), np.sin(-alphas[i])
            base_dirs[i].append([cu * ca - cv * sa, cu * sa + cv * ca, cn])
            plat_dirs[i].append(plate_rods[:, i])
    worst = 0.0
    axes_base = np.zeros((3, 6))
    axes_plat = np.zeros((3, 6))
    for i in range(6):
        axes_base[:, i], dev_b = _best_axis(np.array(base_dirs[i]).T)
        axes_plat[:, i], dev_p = _best_axis(np.array(plat_dirs[i]).T)
        worst = max(worst, dev_b, dev_p)
    return worst, axes_base, axes_plat


def worst_rod_arm_angle(geom: Geometry, req: Requirements, z: float):
    """Smallest rod-to-arm angle in the envelope, degrees.

    90 deg is ideal: arm motion then goes entirely into rod length.  As this
    closes, the ``j / a`` substitution in :func:`tilt_error` grows optimistic
    and the leg loses mechanical advantage.
    """
    worst = 90.0
    for az in _envelope(req):
        R, T = tilt_pose(np.radians(req.tilt_deg), az, z)
        alphas = ik(geom, R, T)
        tips = arm_tips(geom, alphas)
        arms = tips - geom.b
        arms = arms / np.linalg.norm(arms, axis=0)
        rods = R @ geom.p + T[:, None] - tips
        rods = rods / np.linalg.norm(rods, axis=0)
        cos = np.abs(np.einsum("ij,ij->j", arms, rods))
        worst = min(worst, float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))).min()))
    return worst


# --------------------------------------------------------------------------- #
# one design, all four checks
# --------------------------------------------------------------------------- #


def evaluate(geom: Geometry, servo: Servo | None = None,
             req: Requirements | None = None) -> Report:
    """Run R1, R2, R3 and the joint-cone check on one geometry."""
    servo = servo or Servo()
    req = req or Requirements()
    rep = Report()
    try:
        R0, T0 = home_pose(geom)
    except (ValueError, FKNotConverged) as exc:
        rep.notes.append(f"home pose failed: {exc}")
        return rep
    z = float(T0[2])
    rep.home_z_mm = z

    rep.r1_ok, rep.max_servo_deg, notes = reach(geom, servo, req, z)
    rep.notes += notes
    rep.max_tilt_deg = req.tilt_deg if rep.r1_ok else float("nan")
    if not rep.r1_ok:
        return rep

    rep.rate_dps = tilt_rate(geom, servo, req, z)
    rep.r3_ok = rep.rate_dps >= req.rate_dps

    rep.tilt_error_deg, rep.tilt_error_rss_deg = tilt_error(geom, servo, req, z)
    rep.r2_ok = rep.tilt_error_rss_deg <= req.precision_deg

    rep.max_misalign_deg, rep.axes_base, rep.axes_platform = misalignment(geom, req, z)
    rep.cone_ok = rep.max_misalign_deg <= req.cone_deg

    rep.worst_rod_arm_deg = worst_rod_arm_angle(geom, req, z)
    rep.ok = rep.r1_ok and rep.r2_ok and rep.r3_ok and rep.cone_ok
    return rep


def search(candidates, servo: Servo | None = None, req: Requirements | None = None):
    """Evaluate an iterable of parameter dicts; return ``(params, report)`` pairs.

    Candidates that fail to build (``make_geometry`` rejects them) or fail to
    close at the datum are skipped with a note, not raised: a search walks
    through impossible corners as a matter of course.
    """
    out = []
    for params in candidates:
        try:
            geom = make_geometry(**params)
        except ValueError as exc:
            rep = Report()
            rep.notes.append(f"rejected: {exc}")
            out.append((params, rep))
            continue
        out.append((params, evaluate(geom, servo, req)))
    return out


# --------------------------------------------------------------------------- #
# clearance - does anything hit anything?
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Body:
    """The servo case as a box around the shaft centre, in the servo frame.

    Axes are the servo frame's own: ``n`` along the shaft, ``u`` in-plane, and
    ``v = n x u`` (vertical under horizontal shafts).  Defaults are the MG90S
    measured 2026-09-16 - 12.3 mm along the shaft, 32.2 mm across the mounting
    tabs, 35.3 mm tall - with the case hanging below the shaft.

    ``STATUS.md`` flags that which measured span is which was never confirmed.
    If 35.3 is the tab span rather than the height, swap ``along_u`` and
    ``below``; the check is only as good as these four numbers.
    """

    along_n: float = 12.3  # full thickness, centred on the shaft
    along_u: float = 32.2  # full width across the tabs, centred
    below: float = 30.0  # case bottom below the shaft centre
    above: float = 5.0  # case top above the shaft centre


def _point_box_distance(pts_frame: np.ndarray, body: Body) -> np.ndarray:
    """Distance from points to the axis-aligned case box, in the servo frame.

    ``pts_frame`` is ``(3, k)`` in ``(n, u, v)`` components.  Zero inside.
    """
    half = np.array([body.along_n / 2.0, body.along_u / 2.0])
    outside = np.abs(pts_frame[:2]) - half[:, None]
    lo, hi = -body.below, body.above
    outside_v = np.maximum(lo - pts_frame[2], pts_frame[2] - hi)
    gaps = np.vstack([outside, outside_v])
    return np.linalg.norm(np.maximum(gaps, 0.0), axis=0)


def clearance(geom: Geometry, req: Requirements, z: float,
              body: Body | None = None, n_samples: int = 25):
    """Minimum separations over the envelope, mm.

    Returns a dict with

    ``rod_rod``
        closest approach between two different rods.
    ``rod_body_other``
        closest approach between a rod and a *different* leg's servo case.
        This is the one that bites: a rod sweeping across its neighbour.
    ``rod_body_own``
        rod against its own servo's case.  Small by construction - the rod
        starts at that servo's arm tip - so judge it against the arm radius,
        not against zero.

    Rods are sampled as ``n_samples`` points; the true minimum can sit between
    samples, so the figures are slightly optimistic.  25 samples over a 70 mm
    rod is a 3 mm step, which is fine for spotting a collision and too coarse
    to certify a 0.5 mm gap.
    """
    body = body or Body()
    v = np.cross(geom.n, geom.u, axis=0)
    t = np.linspace(0.0, 1.0, n_samples)
    out = {"rod_rod": np.inf, "rod_body_other": np.inf, "rod_body_own": np.inf}
    for az in _envelope(req):
        R, T = tilt_pose(np.radians(req.tilt_deg), az, z)
        tips = arm_tips(geom, ik(geom, R, T))
        anchors = R @ geom.p + T[:, None]
        # (6, 3, n_samples) sampled points along each rod
        rods = np.stack([tips[:, i, None] + t * (anchors[:, i] - tips[:, i])[:, None]
                         for i in range(6)])
        for i in range(6):
            for j in range(6):
                if i < j:  # rod i against rod j, sample-to-sample
                    diff = rods[i][:, :, None] - rods[j][:, None, :]
                    out["rod_rod"] = min(out["rod_rod"],
                                         float(np.linalg.norm(diff, axis=0).min()))
                rel = rods[i] - geom.b[:, j, None]
                frame = np.vstack([geom.n[:, j] @ rel, geom.u[:, j] @ rel, v[:, j] @ rel])
                dist = float(_point_box_distance(frame, body).min())
                key = "rod_body_own" if i == j else "rod_body_other"
                out[key] = min(out[key], dist)
    return out


@dataclass(frozen=True)
class Horn:
    """The printed horn extension as a box that turns with the servo arm.

    Measured 2026-09-18: 32.3 x 12 x 4.95 mm overall.  Placed in the arm frame
    with ``length`` radial (along the arm), ``width`` across it inside the servo
    plane, and ``thickness`` along the shaft, since the part is a sandwich over
    the stock horn.  ``behind`` is how far it reaches back past the spline
    centre; the rest of ``length`` reaches outward past the ``a = 22 mm`` bolt.
    """

    length: float = 32.3
    width: float = 12.0
    thickness: float = 4.95
    behind: float = 8.0

    def points(self, n_r: int = 7, n_w: int = 3, n_t: int = 2) -> np.ndarray:
        """A ``(3, k)`` grid over the box in arm-frame ``(radial, across, axial)``."""
        r = np.linspace(-self.behind, self.length - self.behind, n_r)
        w = np.linspace(-self.width / 2.0, self.width / 2.0, n_w)
        t = np.linspace(-self.thickness / 2.0, self.thickness / 2.0, n_t)
        grid = np.meshgrid(r, w, t, indexing="ij")
        return np.vstack([g.ravel() for g in grid])


def horn_clearance(geom: Geometry, req: Requirements, z: float,
                   body: Body | None = None, horn: Horn | None = None,
                   n_samples: int = 25):
    """Minimum separations involving the printed horn extension, mm.

    Returns ``horn_body`` (horn against another leg's servo case), ``horn_rod``
    (horn against another leg's rod) and ``horn_horn``.  A leg's own rod and
    case are excluded: the horn is bolted to one and carries the other.

    The horn is sampled on a grid rather than treated as a solid, so a gap
    between two flat faces can read slightly large; with ~5 mm steps it is a
    collision detector, not a certificate for millimetre gaps.
    """
    body = body or Body()
    horn = horn or Horn()
    v = np.cross(geom.n, geom.u, axis=0)
    box = horn.points()
    t = np.linspace(0.0, 1.0, n_samples)
    out = {"horn_body": np.inf, "horn_rod": np.inf, "horn_horn": np.inf}
    for az in _envelope(req):
        R, T = tilt_pose(np.radians(req.tilt_deg), az, z)
        alphas = ik(geom, R, T)
        tips = arm_tips(geom, alphas)
        anchors = R @ geom.p + T[:, None]
        rods = np.stack([tips[:, i, None] + t * (anchors[:, i] - tips[:, i])[:, None]
                         for i in range(6)])
        # horn points in world: radial/across rotate with alpha in the servo plane
        horns = []
        for i in range(6):
            ca, sa = np.cos(alphas[i]), np.sin(alphas[i])
            radial = ca * geom.u[:, i] + sa * v[:, i]
            across = -sa * geom.u[:, i] + ca * v[:, i]
            horns.append(geom.b[:, i, None] + np.outer(radial, box[0])
                         + np.outer(across, box[1]) + np.outer(geom.n[:, i], box[2]))
        for i in range(6):
            for j in range(6):
                if i == j:
                    continue
                rel = horns[i] - geom.b[:, j, None]
                frame = np.vstack([geom.n[:, j] @ rel, geom.u[:, j] @ rel, v[:, j] @ rel])
                out["horn_body"] = min(out["horn_body"],
                                       float(_point_box_distance(frame, body).min()))
                diff = horns[i][:, :, None] - rods[j][:, None, :]
                out["horn_rod"] = min(out["horn_rod"],
                                      float(np.linalg.norm(diff, axis=0).min()))
                if i < j:
                    diff = horns[i][:, :, None] - horns[j][:, None, :]
                    out["horn_horn"] = min(out["horn_horn"],
                                           float(np.linalg.norm(diff, axis=0).min()))
    return out
