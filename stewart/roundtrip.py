"""Round-trip test harness: pose -> ik -> fk -> pose, with an offset FK seed.

``ik`` and ``fk`` are passed in so implementations can be swapped without
editing this file.  Millimetres and radians in; the report is millimetres and
degrees.
"""
from __future__ import annotations

import numpy as np

from .kinematics import Unreachable, geodesic_angle


# --------------------------------------------------------------------------- #
# rotation helpers
# --------------------------------------------------------------------------- #
def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _axis_angle(axis, angle):
    """An SO(3) member from axis-angle (Rodrigues).

    Used ONLY to synthesise valid rotations for testing.  It is not a claim
    about any Euler convention - none is chosen here.
    """
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        return np.eye(3)
    x, y, z = axis / norm
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def _geodesic_deg(R, R_hat):
    """Geodesic angle between two rotations, degrees.

    ``|log_so3(R^T R_hat)|``.  **Was** ``arccos((tr(R^T R_hat) - 1) / 2)``;
    changed 2026-09-07 because that form has a floor of ``~8.5e-7 deg`` near
    the identity - the trace carries the angle only at second order, so half
    the digits are gone before ``arccos`` is called - and a round trip that is
    correct to ``1e-13 deg`` was being reported at ``1e-6``.  Same quantity,
    verified to ``1e-12`` relative wherever ``arccos`` is well conditioned by
    ``check_metric_agreement`` in ``archive/diagnostics/roundtrip.py``.

    Neither form is a rotation convention; both name an axis and an angle.
    """
    return float(np.degrees(geodesic_angle(R, R_hat)))


def _unpack(pose):
    if len(pose) == 3:
        name, R, T = pose
    elif len(pose) == 2:
        R, T = pose
        name = "pose"
    else:
        raise ValueError("each pose must be (name, R, T) or (R, T)")
    return (
        str(name),
        np.asarray(R, dtype=float).reshape(3, 3),
        np.asarray(T, dtype=float).reshape(3),
    )


# --------------------------------------------------------------------------- #
# pose sets
# --------------------------------------------------------------------------- #
def known_poses():
    """Hand-checkable poses as a list of ``(name, R, T)`` (mm, radians)."""
    return [
        ("R = I, T = 0", np.eye(3), np.zeros(3)),
        ("yaw +90 deg about z", _rot_z(np.pi / 2.0), np.zeros(3)),
        ("small roll +3 deg about x", _rot_x(np.radians(3.0)), np.zeros(3)),
        ("small pitch +3 deg about y", _rot_y(np.radians(3.0)), np.zeros(3)),
    ]


def random_poses(n, *, max_translation_mm=15.0, max_tilt_deg=12.0,
                 centre_mm=(0.0, 0.0, 0.0), seed=0):
    """``n`` bounded random poses as ``(name, R, T)``.

    Rotations are axis-angle about a uniform random axis, angle in
    ``[0, max_tilt_deg]``; axis-angle is only a generator of valid SO(3)
    members for testing and implies no Euler convention.  Translation is
    uniform in a box of half-width ``max_translation_mm`` about ``centre_mm``.
    """
    rng = np.random.default_rng(seed)
    centre = np.asarray(centre_mm, dtype=float).reshape(3)
    out = []
    for k in range(int(n)):
        axis = rng.normal(size=3)
        angle = np.radians(rng.uniform(0.0, max_tilt_deg))
        R = _axis_angle(axis, angle)
        T = centre + rng.uniform(-1.0, 1.0, size=3) * max_translation_mm
        out.append((f"random #{k + 1}", R, T))
    return out


# --------------------------------------------------------------------------- #
# the round trip
# --------------------------------------------------------------------------- #
def round_trip(geom, poses, ik, fk, *, seed_offset_mm=5.0, seed_offset_deg=3.0,
               seed=0, verbose=True):
    """Run ``pose -> ik -> fk -> pose`` for each pose and report the residuals.

    ``ik(geom, R, T) -> alphas`` (shape ``(6,)``, radians).
    ``fk(geom, alphas, R0, T0) -> (R_hat, T_hat)``, seeded at ``(R0, T0)``.

    The FK seed is deliberately offset from the true pose by
    ``seed_offset_mm`` and ``seed_offset_deg``.  Seeded at the truth a
    numerical FK sees zero residual and returns immediately, so the round trip
    would pass for *any* ``ik`` - including one that returns zeros.
    ``seed_offset_* = 0`` is allowed but prints a warning that the result
    proves nothing.

    ``NotImplementedError`` and :class:`~stewart.kinematics.Unreachable` are
    caught per pose and reported, not raised.

    Returns
    -------
    list of dict
        One row per pose: ``name``, ``status``, ``pos_err_mm``,
        ``ang_err_deg`` (errors are NaN when a status other than ``"ok"``).
    """
    rng = np.random.default_rng(seed)

    if seed_offset_mm == 0.0 and seed_offset_deg == 0.0:
        print(
            "WARNING: round_trip FK seed offset is zero.  A numerical FK seeded "
            "at the true pose returns with zero residual, so this run proves "
            "nothing - any ik (even one returning zeros) would pass."
        )

    results = []
    for pose in poses:
        name, R, T = _unpack(pose)
        row = {"name": name, "status": "ok",
               "pos_err_mm": np.nan, "ang_err_deg": np.nan}

        try:
            alphas = ik(geom, R, T)
        except NotImplementedError:
            row["status"] = "ik not implemented"
            results.append(row)
            continue
        except Unreachable as exc:
            row["status"] = f"ik Unreachable (leg {exc.leg}, {exc.direction})"
            results.append(row)
            continue

        axis = rng.normal(size=3)
        R_seed = _axis_angle(axis, np.radians(seed_offset_deg)) @ R
        step = rng.normal(size=3)
        norm = np.linalg.norm(step)
        step = step / norm if norm > 0.0 else np.zeros(3)
        T_seed = T + seed_offset_mm * step

        try:
            R_hat, T_hat = fk(geom, alphas, R_seed, T_seed)
        except NotImplementedError:
            row["status"] = "fk not implemented"
            results.append(row)
            continue
        except Unreachable as exc:
            row["status"] = f"fk Unreachable (leg {exc.leg}, {exc.direction})"
            results.append(row)
            continue

        T_hat = np.asarray(T_hat, dtype=float).reshape(3)
        R_hat = np.asarray(R_hat, dtype=float).reshape(3, 3)
        row["pos_err_mm"] = float(np.linalg.norm(T_hat - T))
        row["ang_err_deg"] = _geodesic_deg(R, R_hat)
        results.append(row)

    if verbose:
        print(_format(results))
    return results


def _format(rows):
    widths = (30, 34, 14, 14)
    head = ("pose", "status", "pos err (mm)", "ang err (deg)")

    def line(cells):
        return "  ".join(str(c).ljust(w) for c, w in zip(cells, widths))

    out = [line(head), "-" * (sum(widths) + 2 * (len(widths) - 1))]
    for r in rows:
        pe = f"{r['pos_err_mm']:.4f}" if np.isfinite(r["pos_err_mm"]) else "-"
        ae = f"{r['ang_err_deg']:.4f}" if np.isfinite(r["ang_err_deg"]) else "-"
        out.append(line((r["name"], r["status"], pe, ae)))
    return "\n".join(out)
