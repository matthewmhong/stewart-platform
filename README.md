# stewart-platform

Design toolkit for a 6-RSS (rotary servo) Stewart platform built as a
ball-balancing plate: base and platform ring geometry, closed-form inverse
kinematics, numerical forward kinematics, plotting, and the diagnostics that
swept the design space to choose the dimensions.

**Phase 0 is done.** The sweep picked `a = 60.4 mm`, `d = 126 mm`, `beta = 5°`,
`beta_p = 52.5°` at `r_b = 90 mm`, `r_p = 80 mm`. What's open and what comes
next is in [STATUS.md](STATUS.md).

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
    python -m stewart.diagnostics.roundtrip    # FK round-trip gate (exit 0 = pass)
    python -m stewart.diagnostics.sweep        # the design sweep

## Layout

    stewart/
      geometry.py        Geometry, base_ring, platform_ring, make_geometry
      kinematics.py      stage1, legs, arm_tips, ik, fk (+ Jacobian, solver)
      plotting.py        drawing only - never solves kinematics
      roundtrip.py       pose -> ik -> fk -> pose harness
      diagnostics/       one script per design question; each runnable with -m
    test_kinematics.py
    demo.py

## Documents

| file | what it is |
|------|------------|
| [STATUS.md](STATUS.md) | current result, open items, next steps - edited in place |
| [docs/derivation.md](docs/derivation.md) | the maths, and the symbol glossary (§1) |
| [docs/design-log.md](docs/design-log.md) | dated narrative of the design work and why |
| [docs/hardware.md](docs/hardware.md) | sourced specs for horns, joints, servos, rods |
| [docs/archive/](docs/archive/) | superseded handoffs and reports, frozen |
