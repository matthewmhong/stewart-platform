"""Oscillate the platform in one degree of freedom at a time.

Type a motion name and it oscillates until you type ``stop``::

    roll  pitch  yaw          rotations, degrees of amplitude
    surge sway  heave         translations, mm of amplitude
    stop                      ease back to level
    amp 3 / period 5          change the running motion's amplitude / period
    list                      show the amplitudes and their limits
    quit                      level, then exit

The pose is a sinusoid about home, ``x(t) = A sin(2 pi t / period)``, eased in
and out over ``EASE_S`` so nothing starts or stops with a jerk.  Pulses come
from ``joystick.Platform``, which refuses any pose that would push a servo past
its limit or bind a rod end.

**Amplitudes are not free.**  This machine was sized for tilt, so roll and
pitch have room (the servos give out near 9 deg), but yaw, surge and sway bind
the rod ends at about 3 deg / 3 mm - the arm-end joint already sits at 25 deg
of its ~30 deg cone at home, by design (straight bolt, 2026-09-18).  The
defaults below are ~2/3 of each measured limit.

Run::

    python motion.py                  # auto-finds the Arduino
    python motion.py --selftest       # check every motion, no hardware
"""
from __future__ import annotations

import argparse
import queue
import sys
import threading
import time

import numpy as np

from joystick import (BAUD, SELFTEST_SIGN, SELFTEST_ZERO, Platform, find_port)
from stewart.kinematics import Unreachable, exp_so3, ik

PERIOD_S = 4.0          # one full cycle
EASE_S = 1.0            # ease in / out
RATE_HZ = 50.0

#: name -> (axis or offset, default amplitude, measured limit, unit).  Limits
#: measured 2026-09-24 against the firmware's 350 us and a 28 deg rod-end cone.
MOTIONS = {
    "roll":  ("rot", np.array([1.0, 0.0, 0.0]), 4.5, 9.0, "deg"),
    "pitch": ("rot", np.array([0.0, 1.0, 0.0]), 4.5, 9.8, "deg"),
    "yaw":   ("rot", np.array([0.0, 0.0, 1.0]), 2.0, 3.0, "deg"),
    "surge": ("tr",  np.array([1.0, 0.0, 0.0]), 2.0, 3.0, "mm"),
    "sway":  ("tr",  np.array([0.0, 1.0, 0.0]), 2.0, 3.0, "mm"),
    "heave": ("tr",  np.array([0.0, 0.0, 1.0]), 6.0, 11.0, "mm"),
}


def pose(plat: Platform, name: str, value: float):
    """Pose for one motion at displacement ``value`` (deg or mm)."""
    kind, axis, *_ = MOTIONS[name]
    if kind == "rot":
        return exp_so3(axis * np.radians(value)), plat.T.copy()
    return np.eye(3), plat.T + axis * value


def reader(q: "queue.Queue[str]") -> None:
    """Feed typed lines to the main loop; EOF (Ctrl-D) quits."""
    for line in sys.stdin:
        q.put(line.strip().lower())
    q.put("quit")


class Oscillator:
    """Sinusoid in one motion, eased in and out."""

    def __init__(self, period: float = PERIOD_S):
        self.name: str | None = None
        self.amp = 0.0
        self.period = period
        self.phase = 0.0            # rad, carried across a motion change
        self.gain = 0.0             # ease envelope, 0-1
        self.stopping = False
        self.pending: tuple[str, float | None] | None = None

    def start(self, name: str, amp: float | None = None) -> None:
        if self.name is not None and self.name != name:
            self.pending = (name, amp)   # ease the old one out first, then swap
            self.stopping = True
            return
        self.name = name
        self.amp = MOTIONS[name][2] if amp is None else amp
        self.phase = 0.0
        self.stopping = False

    def stop(self) -> None:
        self.stopping = True

    def step(self, dt: float) -> float:
        """Advance by ``dt`` and return the displacement."""
        if self.name is None:
            return 0.0
        self.gain += (-1.0 if self.stopping else 1.0) * dt / EASE_S
        self.gain = min(1.0, max(0.0, self.gain))
        if self.stopping and self.gain == 0.0:
            self.name = None
            if self.pending is not None:
                name, amp = self.pending
                self.pending = None
                self.start(name, amp)
            return 0.0
        self.phase += 2.0 * np.pi * dt / self.period
        return self.amp * self.gain * float(np.sin(self.phase))


def describe() -> str:
    rows = [f"  {n:6s} {m[2]:5.1f} {m[4]:3s} amplitude   (limit {m[3]:4.1f} {m[4]})"
            for n, m in MOTIONS.items()]
    return "motions:\n" + "\n".join(rows)


def selftest() -> int:
    """Run every motion through a full cycle and report the worst numbers."""
    plat = Platform(SELFTEST_ZERO, SELFTEST_SIGN)
    bad = 0
    for name, (_, _, amp, limit, unit) in MOTIONS.items():
        worst_us, worst_cone, fail = 0, 0.0, ""
        for ph in np.linspace(0, 2 * np.pi, 73):
            value = amp * np.sin(ph)
            R, T = pose(plat, name, value)
            try:
                us = plat.pulses_at(R, T)
            except (Unreachable, ValueError) as err:
                fail = str(err)
                break
            worst_us = max(worst_us, int(np.abs(us - plat.zero).max()))
            worst_cone = max(worst_cone, plat.cone(R, T, ik(plat.geom, R, T)))
        if fail:
            bad += 1
            print(f"  {name:6s} FAIL at {amp} {unit}: {fail}")
        else:
            print(f"  {name:6s} +/-{amp:4.1f} {unit:3s} ok: worst {worst_us:3d} us from zero "
                  f"(limit 350), rod end {worst_cone:4.1f} deg (limit 28, {limit:4.1f} {unit} binds)")
    print("PASS" if not bad else f"FAIL: {bad} motion(s)")
    return 0 if not bad else 1


def run(port: str, period: float) -> None:
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
    while plat is None:                  # the board sends its calibration on boot and on '?'
        raw = ser.readline().decode(errors="replace").strip()
        if raw.startswith("C "):
            vals = [int(f) for f in raw.split()[1:]]
            if len(vals) == 12:
                plat = Platform(vals[:6], vals[6:])
                print(f"calibration from the Arduino: zero {vals[:6]}, sign {vals[6:]}")
        elif raw.startswith(("E", "W")):
            print("arduino:", raw)

    print(describe())
    print(f"period {period:.1f} s.  Type a motion, `stop`, or `quit`.")

    q: "queue.Queue[str]" = queue.Queue()
    threading.Thread(target=reader, args=(q,), daemon=True).start()

    osc = Oscillator(period)
    last = time.monotonic()
    quitting = False
    try:
        while True:
            while not q.empty():
                cmd, *rest = (q.get().split() or [""])
                if cmd in MOTIONS:
                    amp = float(rest[0]) if rest else None
                    limit = MOTIONS[cmd][3]
                    if amp is not None and amp > limit:
                        print(f"{cmd} is limited to {limit} {MOTIONS[cmd][4]}; using that")
                        amp = limit
                    osc.start(cmd, amp)
                    print(f"{cmd}: +/-{osc.amp:.1f} {MOTIONS[cmd][4]}, period {osc.period:.1f} s")
                elif cmd == "stop":
                    osc.stop()
                    print("stopping")
                elif cmd == "quit":
                    osc.pending = None
                    osc.stop()
                    quitting = True
                elif cmd == "list":
                    print(describe())
                elif cmd == "amp" and rest and osc.name:
                    osc.amp = min(float(rest[0]), MOTIONS[osc.name][3])
                    print(f"amplitude {osc.amp:.1f} {MOTIONS[osc.name][4]}")
                elif cmd == "period" and rest:
                    osc.period = max(0.5, float(rest[0]))
                    print(f"period {osc.period:.1f} s")
                elif cmd:
                    print(f"unknown: {cmd}.  Try: {' '.join(MOTIONS)} stop list amp period quit")

            now = time.monotonic()
            dt = now - last
            if dt < 1.0 / RATE_HZ:
                time.sleep(1.0 / RATE_HZ - dt)
                now = time.monotonic()
                dt = now - last
            last = now

            name = osc.name
            value = osc.step(dt)
            R, T = pose(plat, name, value) if name else (np.eye(3), plat.T)
            try:
                us = plat.pulses_at(R, T)
            except (Unreachable, ValueError) as err:
                print(f"\nrefused: {err}")
                osc.stop()
                continue
            ser.write(("P " + " ".join(str(int(u)) for u in us) + "\n").encode())

            if name:
                print(f"\r{name} {value:+6.2f} {MOTIONS[name][4]}   ", end="", flush=True)
            if quitting and osc.name is None:
                break

            while ser.in_waiting:        # the board also streams the stick; report problems only
                raw = ser.readline().decode(errors="replace").strip()
                if raw.startswith(("E", "W")):
                    print("\narduino:", raw)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        ser.write(("P " + " ".join(str(int(u)) for u in plat.zero.astype(int)) + "\n").encode())
        time.sleep(0.5)
        ser.close()
        print("level; port closed")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", help="serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("--period", type=float, default=PERIOD_S, help=f"seconds per cycle (default {PERIOD_S})")
    ap.add_argument("--selftest", action="store_true", help="check every motion, no hardware")
    args = ap.parse_args()
    if args.selftest:
        sys.exit(selftest())
    run(args.port or find_port(), args.period)


if __name__ == "__main__":
    main()
