# CLAUDE.md

6-RSS Stewart platform design project.  Rotary servos: each leg is
Revolute-Spherical-Spherical - a servo arm of length `a` swept in a fixed
servo plane, then a fixed-length push-rod `d` to the platform.

## Conventions (non-negotiable)

- **Units:** millimetres and radians internally.  Degrees only when printing a
  summary or emitting a servo command.
- **Anchor arrays are `(3, 6)`**, column `i` is leg `i`.  Column vectors, hit
  from the left: `R @ p`, never `p @ R`.  A `(6, 3)` array runs without error
  in many places and silently applies the rotation transposed - so shapes are
  checked at construction.
- **Validate at construction:** shapes, positive lengths, unit norms on `n`
  and `u`, `u . n = 0`.  Name failing legs **1-indexed** in messages.
- **Deps:** Python 3, numpy + matplotlib only.

## Layout

| file | state |
|------|-------|
| `stewart/geometry.py` | DONE - `Geometry` + validation, `base_ring`, `platform_ring`, `make_geometry`, `smoke_geometry` |
| `stewart/kinematics.py` | DONE - `stage1`, `legs`, `w`, `arm_tips`, `ik`, `fk` (+ `fk_solve`, `fk_jacobian`) |
| `stewart/performance.py` | DONE - `Servo`, `Requirements`, `evaluate`, `search`; R1-R3 + rod-end cone against the real kinematics |
| `stewart/plotting.py` | DONE - draws points only, never solves kinematics |
| `stewart/roundtrip.py` | DONE - `pose -> ik -> fk -> pose` harness; `ik`/`fk` are passed in |
| `test_kinematics.py`, `demo.py` | DONE |
| `firmware/servo_test/` | DONE - MG90S bench-test sketch (Arduino, `writeMicroseconds`, serial commands) |
| `cad/` | the user's CAD - not in the repo yet.  The printed horn extension's requirements are `docs/hardware.md` §12 |
| `archive/` | frozen: old docs, the Phase 0 diagnostics and sweep outputs - do not edit or build on |

## Docs

| file | job |
|------|-----|
| `STATUS.md` | requirements, open items, next steps |
| `docs/derivation.md` | maths + the symbol glossary (§1) |
| `docs/design-log.md` | dated narrative (the user's portfolio piece, in their voice) |
| `docs/hardware.md` | sourced part specs |

**Session state lives in `STATUS.md` - update it in place.  Do not create new
dated handoff or session-summary documents.**  **Add to `docs/design-log.md` as
work happens** - a dated entry for each decision or result, not a catch-up
later.

## Notes for whoever picks this up

- `base_ring` is transcribed from a finished derivation.  Treat it as ground
  truth; do not "correct" it without the derivation in hand.
- `smoke_geometry()` is **not** a layout - it exists only to shape-check the
  plotting and the round trip before `make_geometry` is written.  It looks
  wrong when plotted, by design.
- **IK reachability:** the test is `|P| > C` (see the `ik` docstring), never
  the two-sphere bound `|d - a| < |L| < d + a` alone, and never
  `np.clip(P / C, -1, 1)` - clipping fabricates a boundary "solution".
- Errors are raised, not returned.  `Unreachable(ValueError)` carries `.leg`
  (1-indexed) and `.direction` (`"far"` / `"near"`).

## Run

    python demo.py
