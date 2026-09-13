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
| `stewart/plotting.py` | DONE - draws points only, never solves kinematics |
| `stewart/roundtrip.py` | DONE - `pose -> ik -> fk -> pose` harness; `ik`/`fk` are passed in |
| `stewart/diagnostics/` | one module per design question, run with `python -m stewart.diagnostics.<name>` |
| `test_kinematics.py`, `demo.py` | DONE |

## Docs

| file | job |
|------|-----|
| `STATUS.md` | current result, open items, next steps |
| `docs/derivation.md` | maths + the symbol glossary (§1) |
| `docs/design-log.md` | dated narrative (the user's portfolio piece, in their voice) |
| `docs/hardware.md` | sourced part specs |
| `docs/archive/` | frozen superseded docs - do not edit |

**Session state lives in `STATUS.md` - update it in place.  Do not create new
dated handoff or session-summary documents.**

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
