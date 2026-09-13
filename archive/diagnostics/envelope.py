"""The working envelope, and the recovery model the tilt limit comes from.

    python -m stewart.diagnostics.envelope

**Single source of truth.**  Every other diagnostic in this package imports the
envelope from here rather than restating it, so the tilt limit cannot drift
between scripts the way the provisional 6 degrees did.

The envelope is settled (2026-09-05) and is **not** a parameter of these
scripts::

    dxy   = 0          the control law commands tilt only
    dz    = 0          as above
    yaw   = 0          axisymmetric circular plate
    tilt <= 10.5 deg   recovery envelope, derived below

Consequence worth stating once: with ``dxy = dz = yaw = 0`` the envelope is
**purely angular**.  It carries no length dimension, so it does not scale with
the uniform length factor ``k`` at all, and the old warning about scaling the
envelope alongside the geometry no longer applies to it.  Two axes remain -
tilt magnitude and tilt azimuth - not the four (``x``, ``y``, magnitude,
azimuth) recorded on 2026-09-04.

Degrees at the boundary, radians internally.  Arrays are (3, 6).  numpy only.
Reports; always exits 0.
"""
from __future__ import annotations

import numpy as np

# --------------------------------------------------------------------------- #
# the recovery model
# --------------------------------------------------------------------------- #
G = 9.80665                  # m/s^2, standard gravity
ROLL_FACTOR = 5.0 / 7.0      # solid sphere rolling without slip: acc = (5/7) g sin

TAU = 0.5                    # s, bang-bang recovery time
X0_WORKING = 0.050           # m, working displacement to recover from
X0_LATENCY = 0.030           # m, drift during the latency window
TAU_L = 0.150                # s, PROVISIONAL - see module note below
V_PEAK = 0.200               # m/s, peak ball speed used for the drift figure

# X0_LATENCY is exactly TAU_L * V_PEAK; kept as its own constant so the sum
# below reads as "working + latency" rather than hiding a product.
assert abs(X0_LATENCY - TAU_L * V_PEAK) < 1e-12

X0_BARE = X0_WORKING                 # m, the requirement
X0_ENVELOPE = X0_WORKING + X0_LATENCY  # m, the envelope

#: ``tau_L = 150 ms`` is **PROVISIONAL**.  It is inherited from a figure
#: withdrawn 2026-09-03 (the 300 mm/s peak speed) and has no basis of its own.
#: It needs one: sensor frame interval plus servo step response, both on the
#: hardware pull.  It is not promoted here and must not be promoted silently -
#: the 30 mm of the 80 mm total rests on it, and so does 3.97 of the 10.53
#: degrees.
TAU_L_IS_PROVISIONAL = True


def bang_bang_accel(x0: float, tau: float) -> float:
    """Constant-magnitude acceleration that recovers ``x0`` in ``tau``.

    Accelerate for ``tau/2``, decelerate for ``tau/2``; the distance covered is
    ``2 * (1/2) acc (tau/2)^2 = acc tau^2 / 4``, hence ``acc = 4 x0 / tau^2``.
    """
    return 4.0 * x0 / (tau * tau)


def tilt_for(x0: float, tau: float = TAU) -> float:
    """Tilt in **degrees** needed to produce the bang-bang acceleration.

    A solid ball rolling without slip on a plane tilted by ``theta`` sees
    ``acc = (5/7) g sin(theta)``, so ``sin(theta) = 7 acc / (5 g)``.
    """
    s = bang_bang_accel(x0, tau) / (ROLL_FACTOR * G)
    if not -1.0 <= s <= 1.0:
        raise ValueError(f"sin(tilt) = {s} out of range; x0={x0} tau={tau}")
    return float(np.degrees(np.arcsin(s)))


#: The requirement: recover the 50 mm working displacement in ``tau``.
TILT_BARE_DEG = tilt_for(X0_BARE)
#: The envelope.  ``tau_L`` was DROPPED 2026-09-08 (``docs/archive/notation.md`` sec.9):
#: ``docs/hardware.md`` finds neither of its two terms reconstructable
#: from published data, so the 30 mm of latency drift it produced has no
#: basis and is removed rather than left provisional.  What is left is the
#: working displacement alone, so this is derived from ``X0_WORKING`` through
#: ``tilt_for`` - the same call ``TILT_BARE_DEG`` makes - rather than sitting
#: as a literal that can go stale the way the withdrawn 10.5290 did.
#: SUPERSEDED value, kept visible rather than deleted: ``tilt_for(X0_ENVELOPE)``
#: = ``10.5290`` deg, computed at ``x0 = 80 mm`` (50 mm working + `tau_L`
#: latency drift) while ``tau_L`` was still provisional.  Every result
#: committed before 2026-09-09 that reads this constant was computed at that
#: value; see the commit that makes this change for which modules those are.
TILT_LIMIT_DEG = tilt_for(X0_WORKING)

DXY = 0.0
DZ = 0.0
YAW_DEG = 0.0


# --------------------------------------------------------------------------- #
# poses
# --------------------------------------------------------------------------- #
def tilt_R(azimuth_deg, magnitude_deg) -> np.ndarray:
    """Rotation(s) about a horizontal axis, Rodrigues, shape ``(..., 3, 3)``.

    ``azimuth_deg`` is the azimuth of the *rotation axis*, so azimuth 0 tilts
    about world ``x`` and lifts the ``+y`` side.  Broadcasts.
    """
    psi = np.deg2rad(np.asarray(azimuth_deg, float))
    th = np.deg2rad(np.asarray(magnitude_deg, float))
    psi, th = np.broadcast_arrays(psi, th)
    cx, sx = np.cos(psi), np.sin(psi)
    zero = np.zeros_like(psi)
    K = np.stack([
        np.stack([zero, zero, sx], -1),
        np.stack([zero, zero, -cx], -1),
        np.stack([-sx, cx, zero], -1),
    ], -2)
    eye = np.broadcast_to(np.eye(3), K.shape).copy()
    s = np.sin(th)[..., None, None]
    c = (1.0 - np.cos(th))[..., None, None]
    return eye + s * K + c * (K @ K)


#: Fundamental domain for tilt azimuth, in degrees, **as measured** by
#: ``azimuth_symmetry.py``.  See that module: the leg set's mirror lines in
#: azimuth sit at ``30 + 60k``, not at ``0 + 60k``, so the 60-degree window
#: starts at 30 degrees.  Changing this constant without re-running that
#: diagnostic is exactly the mistake it exists to prevent.
AZIMUTH_WINDOW_DEG = (30.0, 90.0)

#: Grid density.  Chosen, not derived: 5 magnitudes x 7 azimuths.  7 azimuths
#: puts a sample every 10 degrees across the 60-degree window, matching the
#: 15-degree resolution of the superseded full-circle sweep with a little to
#: spare; 5 magnitudes because nothing has shown the worst case to be
#: monotonic in tilt magnitude, so the interior is sampled rather than assumed.
N_MAGNITUDE = 5
N_AZIMUTH = 7


def envelope_poses(n_mag: int = N_MAGNITUDE, n_az: int = N_AZIMUTH,
                   tilt_limit_deg: float = TILT_LIMIT_DEG,
                   window_deg=AZIMUTH_WINDOW_DEG, full_circle: bool = False):
    """Envelope pose grid as ``(azimuth_deg, magnitude_deg)``, both 1-D.

    Magnitude 0 is the home pose and is azimuth-independent, so it appears
    **once** rather than ``n_az`` times.  Returned arrays have length
    ``1 + (n_mag - 1) * n_az``.
    """
    mags = np.linspace(0.0, tilt_limit_deg, n_mag)
    if full_circle:
        azis = np.linspace(0.0, 360.0, n_az, endpoint=False)
    else:
        azis = np.linspace(window_deg[0], window_deg[1], n_az)
    az_grid, mag_grid = np.meshgrid(azis, mags[1:], indexing="ij")
    return (np.concatenate([[0.0], az_grid.ravel()]),
            np.concatenate([[0.0], mag_grid.ravel()]))


def n_poses(n_mag: int = N_MAGNITUDE, n_az: int = N_AZIMUTH) -> int:
    return 1 + (n_mag - 1) * n_az


# --------------------------------------------------------------------------- #
# report
# --------------------------------------------------------------------------- #
def main() -> None:
    print("=" * 78)
    print("WORKING ENVELOPE - settled 2026-09-05")
    print("=" * 78)
    print(f"  dxy = {DXY}   dz = {DZ}   yaw = {YAW_DEG} deg")
    print(f"  tilt limit           = {TILT_LIMIT_DEG:.4f} deg   (envelope)")
    print(f"  bare requirement     = {TILT_BARE_DEG:.4f} deg")
    print(f"  latency margin       = {TILT_LIMIT_DEG - TILT_BARE_DEG:.4f} deg")
    print()
    print("  recovery model: bang-bang, acc = 4 x0 / tau^2,")
    print("                  sin(tilt) = 7 acc / (5 g),  g = %.5f m/s^2" % G)
    print(f"  tau  = {TAU} s")
    print(f"  x0   = {X0_WORKING*1e3:.0f} mm working + {X0_LATENCY*1e3:.0f} mm "
          f"latency drift = {X0_ENVELOPE*1e3:.0f} mm")
    print(f"         latency drift = tau_L * v_peak = {TAU_L*1e3:.0f} ms * "
          f"{V_PEAK*1e3:.0f} mm/s")
    print(f"  acc  = {bang_bang_accel(X0_ENVELOPE, TAU):.4f} m/s^2 (envelope), "
          f"{bang_bang_accel(X0_BARE, TAU):.4f} m/s^2 (bare)")
    print()
    print("  *** tau_L = 150 ms is PROVISIONAL ***")
    print("      inherited from the 300 mm/s figure withdrawn 2026-09-03.")
    print("      Needs its own basis: sensor frame interval + servo step")
    print("      response, both on the hardware pull.  It carries 30 of the")
    print(f"      80 mm and {TILT_LIMIT_DEG - TILT_BARE_DEG:.2f} of the "
          f"{TILT_LIMIT_DEG:.2f} degrees.  Do not promote silently.")
    print()

    print("-" * 78)
    print("SENSITIVITY - tilt goes as 1/tau^2 (exactly, in sin(tilt))")
    print("-" * 78)
    print(f"  {'tau [s]':>9} {'bare [deg]':>12} {'envelope [deg]':>16} "
          f"{'ratio to tau=0.5':>18}")
    for tau in (0.35, 0.40, 0.45, 0.50, 0.60, 0.75, 1.00):
        try:
            bare = tilt_for(X0_BARE, tau)
            env = tilt_for(X0_ENVELOPE, tau)
            print(f"  {tau:>9.2f} {bare:>12.3f} {env:>16.3f} "
                  f"{env / TILT_LIMIT_DEG:>18.3f}")
        except ValueError:
            print(f"  {tau:>9.2f} {'-':>12} {'sin(tilt) > 1':>16} {'-':>18}")
    dlog = 2.0 * 0.10
    print(f"  a 10% error in tau moves the required sin(tilt) by ~{dlog*100:.0f}%")
    print("  -> the margin is SENSITIVE TO tau and the sensitivity travels with")
    print("     the number wherever it is quoted.")
    print()

    print("-" * 78)
    print("POSE GRID")
    print("-" * 78)
    print(f"  envelope axes: 2 (tilt magnitude, tilt azimuth).  dxy = dz = 0")
    print(f"  removes x and y, so the four-axis envelope of 2026-09-04 is now")
    print(f"  two axes, and being purely angular it does not scale with k.")
    az, mg = envelope_poses()
    print(f"  magnitudes  : {N_MAGNITUDE} over [0, {TILT_LIMIT_DEG:.4f}] deg")
    print(f"  azimuths    : {N_AZIMUTH} over {AZIMUTH_WINDOW_DEG} deg "
          f"(60-deg fundamental domain)")
    print(f"  poses       : {len(az)}  = 1 + ({N_MAGNITUDE}-1) * {N_AZIMUTH}")
    print(f"                (magnitude 0 is azimuth-independent, counted once)")
    print(f"  w evals per objective evaluation : {len(az) * 6}")
    print(f"  full-circle equivalent would be  : "
          f"{n_poses(N_MAGNITUDE, N_AZIMUTH * 6)} poses, "
          f"{n_poses(N_MAGNITUDE, N_AZIMUTH * 6) * 6} w evals")
    print()
    print("  (The 60-degree window is verified, not assumed: "
          "azimuth_symmetry.py)")


if __name__ == "__main__":
    main()
