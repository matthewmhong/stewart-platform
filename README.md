# stewart-platform

Design toolkit for a 6-RSS (rotary servo) Stewart platform built to balance a
ping-pong ball: base and platform ring geometry, closed-form inverse
kinematics, numerical forward kinematics, and plotting.

**Where it stands.** The kinematics are done and tested. The design is being
chosen against pass/fail requirements (tilt range, tilt precision, tilt speed,
latency), with TowerPro MG90S servos and their stock horns, picking the
simplest design to build that passes. Details in [STATUS.md](STATUS.md).

## Conventions

- **Millimetres and radians** internally.  Degrees appear only when printing a
  summary or building a servo command.
- **Every anchor array is `(3, 6)`; column `i` is leg `i`.**  Vectors are
  columns and rotations hit from the left: `R @ p`, never `p @ R`.  A `(6, 3)`
  array will broadcast in many places and silently apply the rotation
  transposed, so shapes and unit norms are checked at construction and failing
  legs are named 1-indexed.
- Pose order is `(R, T)` in every signature.
- Python 3, **numpy + matplotlib only**.

## Run

    python -m pip install -r requirements.txt
    python test_kinematics.py                  # unit checks
    python demo.py                             # writes demo.png, prints the round-trip report

## Layout

    stewart/
      geometry.py        Geometry, base_ring, platform_ring, make_geometry
      kinematics.py      stage1, legs, arm_tips, ik, fk (+ Jacobian, solver)
      plotting.py        drawing only - never solves kinematics
      roundtrip.py       pose -> ik -> fk -> pose harness
    test_kinematics.py
    demo.py
    archive/             frozen: old docs, Phase 0 diagnostics and sweep outputs

## Documents

| file | what it is |
|------|------------|
| [STATUS.md](STATUS.md) | requirements, open items, next steps - edited in place |
| [docs/derivation.md](docs/derivation.md) | the maths, and the symbol glossary (§1) |
| [docs/design-log.md](docs/design-log.md) | dated narrative of the design work and why |
| [docs/hardware.md](docs/hardware.md) | sourced specs for horns, joints, servos, rods |
| [archive/](archive/) | superseded material, frozen |
