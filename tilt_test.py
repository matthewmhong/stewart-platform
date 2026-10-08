"""Measure the top plate's tilt against what was commanded.  R1 and R2 on the machine.

Every number so far is a prediction from the evaluator.  This script holds a
commanded tilt still, you read it with a level on the plate, and it logs the
pair to a CSV.  Two guided tests::

    r1 [deg]        level, then `deg` (default 4.5) toward every 30 deg of
                    azimuth, then level again.  Reports measured / commanded.
    r2 [deg] [n]    one target (default 3.0 deg toward +x), approached n times
                    (default 5) from below (level) and n times from above
                    (2 x deg).  The gap between the two means is the deadband
                    the loop can't see; the scatter within each is repeatability.

And by hand::

    tilt <deg> <az>   hold that tilt, low side toward az (deg from +x, CCW from above)
    level             hold level
    quit              level, then exit

**Readings.**  Type what the level shows: one number (total tilt) or two
(``x y``, a two-axis level with its x edge along the plate's +x, which points
at the midpoint of servos 1 and 2).  Two numbers are combined as
``hypot(x, y)``.  ``s`` skips a point.

**Zero the level on the plate at level** before `r1` / `r2`, and keep it on
the same spot, centred.  Then every reading is relative to home, which is what
the IK commands.  Whether home itself is parallel to the base is a separate
reading: zero on the base plate, then read the plate at level.

Run::

    python tilt_test.py               # auto-finds the Arduino
    python tilt_test.py --selftest    # every pose the tests command, no hardware
"""
from __future__ import annotations

import argparse
import csv
import queue
import sys
import threading
import time

import numpy as np

from joystick import (BAUD, SELFTEST_SIGN, SELFTEST_ZERO, Platform, find_port)
from motion import reader
from stewart.kinematics import Unreachable

R1_DEG = 4.5
R1_AZ = np.arange(0, 360, 30)
R2_DEG = 3.0
R2_N = 5
SETTLE_S = 1.5          # after each move, before asking for a reading
RATE_HZ = 25.0          # resend rate; the firmware levels after 0.5 s of silence


def tilt_vec(deg: float, az_deg: float) -> np.ndarray:
    a = np.radians(az_deg)
    return np.radians(deg) * np.array([np.cos(a), np.sin(a)])


def parse_reading(line: str) -> float | None:
    """'4.4' or '3.1 3.2' -> total tilt in degrees; 's' -> None.  ValueError otherwise."""
    parts = line.replace(",", " ").split()
    if parts == ["s"]:
        return None
    vals = [float(p) for p in parts]
    if len(vals) == 1:
        return abs(vals[0])
    if len(vals) == 2:
        return float(np.hypot(*vals))
    raise ValueError("one number, two numbers, or s")


def r1_plan(deg: float) -> list[tuple[float, float]]:
    return [(0.0, 0.0)] + [(deg, float(az)) for az in R1_AZ] + [(0.0, 0.0)]


def r2_plan(deg: float, n: int) -> list[tuple[str, float]]:
    """(approach, start tilt) pairs, alternating below / above."""
    plan = []
    for _ in range(n):
        plan += [("below", 0.0), ("above", 2.0 * deg)]
    return plan


def r1_summary(rows: list[tuple[float, float, float]]) -> str:
    """rows: (commanded deg, az deg, measured deg)."""
    out = ["az    cmd   meas   ratio"]
    ratios = []
    for cmd, az, meas in rows:
        if cmd == 0.0:
            out.append(f"level        {meas:5.2f}   (should read 0)")
            continue
        ratios.append(meas / cmd)
        out.append(f"{az:3.0f}  {cmd:5.2f}  {meas:5.2f}   {meas / cmd:5.3f}")
    if ratios:
        r = np.array(ratios)
        tilted = [m for c, _, m in rows if c > 0]
        out.append(f"ratio mean {r.mean():.3f}, range {r.min():.3f}-{r.max():.3f}; "
                   f"smallest tilt reached {min(tilted):.2f} deg (R1 needs >= {R1_DEG})")
    return "\n".join(out)


def r2_summary(target: float, rows: list[tuple[str, float]]) -> str:
    """rows: (approach, measured deg)."""
    out = []
    means = {}
    for side in ("below", "above"):
        v = np.array([m for s, m in rows if s == side])
        if len(v) == 0:
            continue
        means[side] = v.mean()
        sd = v.std(ddof=1) if len(v) > 1 else float("nan")
        out.append(f"from {side}: n {len(v)}, mean {v.mean():.3f}, sd {sd:.3f}, "
                   f"range {v.max() - v.min():.3f} deg")
    if len(means) == 2:
        gap = means["above"] - means["below"]
        out.append(f"gap above - below {gap:+.3f} deg (deadband + backlash seen at the plate)")
    allv = np.array([m for _, m in rows])
    if len(allv):
        out.append(f"all readings span {allv.max() - allv.min():.3f} deg "
                   f"around a {target:.2f} deg target; R2 allows 0.25 total")
    return "\n".join(out)


def selftest() -> int:
    plat = Platform(SELFTEST_ZERO, SELFTEST_SIGN)
    bad = 0
    poses = [(d, az) for d, az in r1_plan(R1_DEG)]
    poses += [(start, 0.0) for _, start in r2_plan(R2_DEG, 1)] + [(R2_DEG, 0.0)]
    for deg, az in poses:
        try:
            us = plat.pulses(tilt_vec(deg, az))
        except (Unreachable, ValueError) as err:
            print(f"  {deg:4.1f} deg toward {az:3.0f}: REFUSED {err}")
            bad += 1
            continue
        print(f"  {deg:4.1f} deg toward {az:3.0f}: worst {int(np.abs(us - plat.zero).max()):3d} us from zero")
    assert parse_reading("4.4") == 4.4 and parse_reading("s") is None
    assert abs(parse_reading("3 4") - 5.0) < 1e-12
    print(r1_summary([(0.0, 0.0, 0.1)] + [(4.5, float(a), 4.4) for a in R1_AZ]))
    print(r2_summary(3.0, [("below", 2.9), ("above", 3.1), ("below", 2.95), ("above", 3.05)]))
    print("PASS" if not bad else f"FAIL: {bad} pose(s) refused")
    return 0 if not bad else 1


class Rig:
    """Holds a tilt by resending it, and asks for readings without letting go."""

    def __init__(self, ser, plat: Platform, q: "queue.Queue[str]"):
        self.ser, self.plat, self.q = ser, plat, q
        self.us = plat.zero.astype(int)

    def hold(self, deg: float, az: float) -> bool:
        try:
            self.us = self.plat.pulses(tilt_vec(deg, az))
        except (Unreachable, ValueError) as err:
            print(f"refused: {err}")
            return False
        return True

    def tick(self) -> None:
        self.ser.write(("P " + " ".join(str(int(u)) for u in self.us) + "\n").encode())
        while self.ser.in_waiting:       # the board streams the stick; report problems only
            raw = self.ser.readline().decode(errors="replace").strip()
            if raw.startswith(("E", "W")):
                print("arduino:", raw)

    def wait(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.tick()
            time.sleep(1.0 / RATE_HZ)

    def line(self) -> str:
        while True:
            try:
                return self.q.get(timeout=1.0 / RATE_HZ)
            except queue.Empty:
                self.tick()

    def reading(self, label: str) -> float | None | str:
        """Measured tilt, None if skipped, 'abort' to stop the test."""
        while True:
            print(f"{label} - reading? ", end="", flush=True)
            raw = self.line().strip()
            if raw in ("q", "quit", "abort"):
                return "abort"
            try:
                return parse_reading(raw)
            except ValueError:
                print("one number, two numbers (x y), s to skip, q to stop the test")


def run(port: str) -> None:
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is missing: pip install pyserial")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except serial.SerialException as err:
        if "busy" in str(err).lower():
            sys.exit(f"{port} is busy: close the Arduino IDE's Serial Monitor and try again")
        sys.exit(f"could not open {port}: {err}")

    print(f"opened {port}; waiting for the Arduino to boot...")
    time.sleep(2.0)                      # opening the port resets an Uno
    ser.reset_input_buffer()
    ser.write(b"?\n")
    plat = None
    while plat is None:
        raw = ser.readline().decode(errors="replace").strip()
        if raw.startswith("C "):
            vals = [int(f) for f in raw.split()[1:]]
            if len(vals) == 12:
                plat = Platform(vals[:6], vals[6:])
                print(f"calibration from the Arduino: zero {vals[:6]}, sign {vals[6:]}")
        elif raw.startswith(("E", "W")):
            print("arduino:", raw)

    stamp = time.strftime("%Y%m%d-%H%M")
    path = f"tilt-test-{stamp}.csv"
    f = open(path, "w", newline="")
    log = csv.writer(f)
    log.writerow(["time", "test", "approach", "cmd_deg", "az_deg", "measured_deg"])
    print(f"logging to {path}")

    q: "queue.Queue[str]" = queue.Queue()
    threading.Thread(target=reader, args=(q,), daemon=True).start()
    rig = Rig(ser, plat, q)
    print("Zero the level on the plate at level first.  "
          "Commands: r1 [deg], r2 [deg] [n], tilt <deg> <az>, level, quit")

    try:
        while True:
            cmd, *rest = (rig.line().split() or [""])
            try:
                args = [float(r) for r in rest]
            except ValueError:
                print("numbers only after the command")
                continue

            if cmd == "quit":
                break
            elif cmd == "level":
                rig.hold(0.0, 0.0)
            elif cmd == "tilt" and len(args) == 2:
                if rig.hold(*args):
                    print(f"holding {args[0]:.2f} deg toward {args[1]:.0f}")
            elif cmd == "r1":
                deg = args[0] if args else R1_DEG
                rows = []
                for cmd_deg, az in r1_plan(deg):
                    if not rig.hold(cmd_deg, az):
                        continue
                    rig.wait(SETTLE_S)
                    m = rig.reading(f"{cmd_deg:.2f} deg toward {az:3.0f}")
                    if m == "abort":
                        break
                    if m is None:
                        continue
                    rows.append((cmd_deg, az, m))
                    log.writerow([time.strftime("%H:%M:%S"), "r1", "", cmd_deg, az, m])
                    f.flush()
                rig.hold(0.0, 0.0)
                print(r1_summary(rows))
            elif cmd == "r2":
                deg = args[0] if args else R2_DEG
                n = int(args[1]) if len(args) > 1 else R2_N
                rows = []
                for i, (side, start) in enumerate(r2_plan(deg, n)):
                    if not (rig.hold(start, 0.0)):
                        break
                    rig.wait(SETTLE_S)
                    rig.hold(deg, 0.0)
                    rig.wait(SETTLE_S)
                    m = rig.reading(f"{i + 1}/{2 * n} from {side}")
                    if m == "abort":
                        break
                    if m is None:
                        continue
                    rows.append((side, m))
                    log.writerow([time.strftime("%H:%M:%S"), "r2", side, deg, 0.0, m])
                    f.flush()
                rig.hold(0.0, 0.0)
                print(r2_summary(deg, rows))
            elif cmd:
                print("commands: r1 [deg], r2 [deg] [n], tilt <deg> <az>, level, quit")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        rig.hold(0.0, 0.0)
        rig.wait(0.5)
        ser.close()
        f.close()
        print(f"level; port closed; readings in {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", help="serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("--selftest", action="store_true", help="check every pose, no hardware")
    args = ap.parse_args()
    if args.selftest:
        sys.exit(selftest())
    run(args.port or find_port())


if __name__ == "__main__":
    main()
