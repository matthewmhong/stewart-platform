"""Stage 1: tilt the platform with the joystick.  PC side.

The Arduino (``firmware/joystick/``) streams the stick and outputs pulses; this
script does the maths in between::

    stick -> tilt vector -> pose (R, T) -> ik -> six arm angles -> six pulses

Pushing the stick toward a direction lowers that side of the plate, so a ball
would roll the way the stick points.  Full deflection is ``--tilt`` degrees
(default 4.5, R1).  The button toggles *hold level*.

Pulses: ``us = zero + sign * alpha_deg / DEG_PER_US`` per leg, with ``zero``
and ``sign`` read from the Arduino (saved by ``firmware/servo_cal/``), so the
calibration has one home: the board's EEPROM.

Needs pyserial (``pip install pyserial``) for the real run; ``--selftest``
runs the maths alone and needs nothing but numpy.

Run::

    python joystick.py                 # auto-finds /dev/cu.usbmodem*
    python joystick.py --port /dev/cu.usbmodem11301
    python joystick.py --selftest      # no hardware
"""
from __future__ import annotations

import argparse
import glob
import sys
import time

import numpy as np

from demo import DESIGN
from stewart import kinematics
from stewart.kinematics import arm_tips
from stewart.geometry import make_geometry
from stewart.performance import home_pose, tilt_pose

BAUD = 115200
DEG_PER_US = 0.087          # servo 1, bench test 2026-09-16; used for all six
MAX_OFF_US = 350            # the firmware refuses more than this from zero
TILT_DEG = 4.5              # R1
CONE_DEG = 28.0             # rod-end bind is ~30 (2026-09-18); 2 deg of margin

# Bolt axes as built, for the rod-end cone check: arm end parallel to the shaft
# (``n_i``, so the check is on the rod's component along n in the arm frame);
# plate end horizontal, parallel to each pair's bisector (hardware.md sec 3).
PLATE_BOLT = np.array([[np.cos(np.radians(a)), np.sin(np.radians(a)), 0.0]
                       for a in (0.0, 0.0, 120.0, 120.0, 240.0, 240.0)]).T

# Stick handling.  If pushing right tilts the wrong way, fix it here.
STICK_ROT_DEG = 0.0         # rotates the stick frame onto the plate's +x
FLIP_X = False
FLIP_Y = False
DEADZONE = 0.08             # fraction of full throw ignored around centre
SMOOTH = 0.3                # 0-1, fraction of the way to the new tilt per sample (50 Hz)

# Calibration as of 2026-09-22 (STATUS.md), for --selftest only.  The real run
# takes it from the Arduino.
SELFTEST_ZERO = [1550, 1500, 1410, 1590, 1560, 1470]
SELFTEST_SIGN = [+1, -1, +1, -1, +1, -1]


class Platform:
    """Tilt vector in, six pulse widths out."""

    def __init__(self, zero, sign):
        self.geom = make_geometry(**DESIGN)
        _, self.T = home_pose(self.geom)
        self.zero = np.asarray(zero, dtype=float)
        self.sign = np.asarray(sign, dtype=float)
        if self.zero.shape != (6,) or self.sign.shape != (6,):
            raise ValueError("need six zeros and six signs")

    def cone(self, R: np.ndarray, T: np.ndarray, alpha: np.ndarray) -> float:
        """Worst rod-end misalignment at this pose, degrees, both ends.

        A rod end binds once the rod leaves a cone about perpendicular to its
        bolt (~30 deg with the insert spacers).  The arm end already sits at
        ~25 deg at home, by choice: the bolt is straight, parallel to the
        shaft.  Same method as :func:`performance.misalignment`, but for one
        pose rather than the envelope.
        """
        tips = arm_tips(self.geom, alpha)
        rods = R @ self.geom.p + T[:, None] - tips
        rods = rods / np.linalg.norm(rods, axis=0)
        plate_rods = R.T @ rods
        n_comp = np.abs(np.sum(self.geom.n * rods, axis=0))          # arm end: bolt along n
        p_comp = np.abs(np.sum(PLATE_BOLT * plate_rods, axis=0))     # plate end: pair bisector
        return float(np.degrees(np.arcsin(np.clip(max(n_comp.max(), p_comp.max()), 0.0, 1.0))))

    def pulses_at(self, R: np.ndarray, T: np.ndarray) -> np.ndarray:
        """Pulses for any pose.

        Raises :class:`kinematics.Unreachable` if a leg can't get there, and
        ``ValueError`` if a pulse would go past what the firmware accepts or a
        rod end would bind.
        """
        alpha = kinematics.ik(self.geom, R, T)
        worst = self.cone(R, T, alpha)
        if worst > CONE_DEG:
            raise ValueError(f"rod end at {worst:.1f} deg, past {CONE_DEG} - would bind")
        us = np.rint(self.zero + self.sign * np.degrees(alpha) / DEG_PER_US)
        off = np.abs(us - self.zero)
        if np.any(off > MAX_OFF_US):
            leg = int(np.argmax(off)) + 1
            raise ValueError(f"leg {leg}: {off.max():.0f} us from zero, past {MAX_OFF_US}")
        return us.astype(int)

    def pulses(self, tilt: np.ndarray) -> np.ndarray:
        """Pulses for a tilt vector ``(tx, ty)`` in rad, pointing at the low side."""
        theta = float(np.hypot(*tilt))
        # tilt_pose lowers the side at `azimuth`, which is where the vector points.
        az = float(np.arctan2(tilt[1], tilt[0]))
        return self.pulses_at(*tilt_pose(theta, az, float(self.T[2])))


def stick_to_tilt(x: int, y: int, centre, tilt_max: float) -> np.ndarray:
    """Raw stick reading (0-1023 each) -> tilt vector in rad, deadzone applied."""
    v = np.array([(x - centre[0]) / 512.0, (y - centre[1]) / 512.0])
    if FLIP_X:
        v[0] = -v[0]
    if FLIP_Y:
        v[1] = -v[1]
    c, s = np.cos(np.radians(STICK_ROT_DEG)), np.sin(np.radians(STICK_ROT_DEG))
    v = np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])
    m = float(np.hypot(*v))
    if m < DEADZONE:
        return np.zeros(2)
    scaled = min(1.0, (m - DEADZONE) / (1.0 - DEADZONE))
    return v / m * scaled * tilt_max


def selftest(tilt_deg: float) -> int:
    """Sweep the full circle at the full tilt and report the pulses."""
    plat = Platform(SELFTEST_ZERO, SELFTEST_SIGN)
    home = plat.pulses(np.zeros(2))
    print(f"home pulses: {home.tolist()}  (should equal the zeros)")
    worst = 0
    for az in np.radians(np.arange(0, 360, 15)):
        tilt = np.radians(tilt_deg) * np.array([np.cos(az), np.sin(az)])
        us = plat.pulses(tilt)
        worst = max(worst, int(np.abs(us - plat.zero).max()))
        print(f"  low side at {np.degrees(az):5.0f} deg: {us.tolist()}")
    print(f"largest move from zero: {worst} us ({worst * DEG_PER_US:.1f} deg of arm); "
          f"firmware limit {MAX_OFF_US}")
    ok = np.array_equal(home, plat.zero.astype(int))
    print("PASS" if ok else "FAIL: home is not the zeros")
    return 0 if ok else 1


def find_port() -> str:
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("no Arduino found - plug it in, or pass --port")
    if len(ports) > 1:
        print(f"several ports, using {ports[0]}: {ports}")
    return ports[0]


def run(port: str, tilt_deg: float) -> None:
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is missing: pip install pyserial")

    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except serial.SerialException as err:
        if "busy" in str(err).lower():
            sys.exit(f"{port} is busy: close the Arduino IDE's Serial Monitor "
                     "(or quit the IDE) and run this again")
        sys.exit(f"could not open {port}: {err}")
    print(f"opened {port}; waiting for the Arduino to boot...")
    time.sleep(2.0)                      # opening the port resets an Uno
    ser.reset_input_buffer()
    ser.write(b"?\n")

    plat = None
    centre = None
    samples = []
    tilt = np.zeros(2)
    level = False
    last_button = 0
    last_print = 0.0
    tilt_max = np.radians(tilt_deg)

    def send(us):
        ser.write(("P " + " ".join(str(int(u)) for u in us) + "\n").encode())

    try:
        while True:
            raw = ser.readline().decode(errors="replace").strip()
            if not raw:
                continue
            kind, *fields = raw.split()

            if kind == "C" and plat is None:
                try:
                    vals = [int(f) for f in fields]
                except ValueError:
                    continue
                if len(vals) != 12:
                    continue
                plat = Platform(vals[:6], vals[6:])
                print(f"calibration from the Arduino: zero {vals[:6]}, sign {vals[6:]}")
                print("hands off the stick - reading its centre...")
            elif kind in ("E", "W"):
                print("arduino:", raw)
            elif kind == "J" and plat is not None:
                try:
                    x, y, b = (int(f) for f in fields)
                except ValueError:
                    continue                 # a partial line, e.g. at boot
                if centre is None:
                    samples.append((x, y))
                    if len(samples) >= 25:   # half a second
                        centre = np.mean(samples, axis=0)
                        print(f"stick centre {centre.round(0).tolist()}; "
                              f"full throw = {tilt_deg} deg.  Button: hold level.  Ctrl-C: stop.")
                    continue
                if b and not last_button:
                    level = not level
                    print("HOLD LEVEL" if level else "stick live")
                last_button = b

                want = np.zeros(2) if level else stick_to_tilt(x, y, centre, tilt_max)
                tilt = tilt + SMOOTH * (want - tilt)
                try:
                    us = plat.pulses(tilt)
                except (kinematics.Unreachable, ValueError) as err:
                    print(f"skipped: {err}")
                    continue
                send(us)

                now = time.time()
                if now - last_print > 0.2:
                    last_print = now
                    t = np.degrees(tilt)
                    print(f"\rtilt {np.hypot(*t):4.2f} deg toward "
                          f"{np.degrees(np.arctan2(t[1], t[0])):6.1f}  pulses {us.tolist()}   ",
                          end="", flush=True)
    except KeyboardInterrupt:
        print("\nstopping: back to level")
    finally:
        if plat is not None:
            send(plat.zero.astype(int))
            time.sleep(0.5)
        ser.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", help="serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("--tilt", type=float, default=TILT_DEG,
                    help=f"tilt at full stick, degrees (default {TILT_DEG}, R1)")
    ap.add_argument("--selftest", action="store_true", help="check the maths, no hardware")
    args = ap.parse_args()
    if args.selftest:
        sys.exit(selftest(args.tilt))
    run(args.port or find_port(), args.tilt)


if __name__ == "__main__":
    main()
