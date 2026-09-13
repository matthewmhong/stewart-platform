# Status

Where the project stands right now. **Edit this file in place each session**;
`git log -p STATUS.md` is the history. Last updated 2026-09-13.

## Direction (since 2026-09-13)

- **Goal:** balance a ping-pong ball on the plate. The design to build is the
  **simplest one to make that passes every requirement below**, not the
  highest-scoring one.
- **Servo:** TowerPro MG90S, using the **stock horn from the box**, not a
  bought or printed arm. `a` can only be one of that horn's hole distances.
- **Kept from Phase 0:** the kinematics (`stewart/`), the derivation, and the
  tilt-range derivation. **Archived:** the reach-margin sweep and all its
  diagnostics (`archive/`). It measured how far the platform can reach, not
  tilt precision, speed or repeatability.

---

## Requirements

Everything follows from the ball. One fact links the plate to the ball:
**1° of tilt accelerates the ball at 0.122 m/s²**, from `(5/7) g sin θ` for a
solid ball rolling without slip.

### Inputs you choose

| input | meaning | value | status |
|---|---|---|---|
| `e` | how far the ball may wander from target once settled | ±5 mm | PROPOSED |
| `x0`, `τ` | disturbance to recover from, and how fast | 50 mm in 0.5 s | settled (Phase 0) |
| `k` | fraction of `τ` the plate may spend slewing between tilts | 0.2 | PROPOSED |
| latency fraction | loop delay as a fraction of `τ` | 0.1 | PROPOSED (rule of thumb) |

### Pass/fail requirements

| # | requirement | formula | value |
|---|---|---|---|
| R1 | **Tilt range**, every direction, at home height | `sin θ = 7 (4 x0 / τ²) / 5g` = 6.558°, rounded up | **≥ 7°** |
| R2 | **Tilt precision**: resolution and repeatability together | drift `½ (5/7) g sin δ · τ² ≤ e` → `δ ≤ 0.327°` | **≤ 0.3° total** |
| R3 | **Tilt speed** | swing `2 × R1` in `k τ` = 14° / 0.1 s | **≥ 140°/s** |
| R4 | **Loop latency**: camera + processing + servo | `0.1 τ` | **≤ 50 ms** |
| R5 | **Torque** | holding load + ball, with margin | ≤ 25% of stall (sanity check) |

What sets R2: any tilt error the loop can't remove, held for `τ`, lets the ball
drift. 0.1° → 1.5 mm, 0.3° → 4.6 mm, 0.5° → 7.6 mm, 1° → 15.3 mm. The error
budget R2 has to hold is

    G × (servo deadband + gear backlash)  +  ball-joint play / r_p   ≤  0.3°

where `G` is the gearing, degrees of plate tilt per degree of servo rotation,
roughly `a / r_p`. Deadband and backlash add because camera feedback removes a
steady offset but not random play.

### MG90S parameters

| parameter | published | measured | how to measure |
|---|---|---|---|
| stock horn hole distances | — | | calipers, spline centre to each hole |
| spline tooth count | — | | count |
| body + mounting-tab footprint | — | | calipers (bounds `beta`) |
| degrees per µs | ~0.09 (estimated, not published) | | protractor at 1000 / 1500 / 2000 µs |
| usable travel | ~90–180° (listings vary) | | same test, find the ends |
| deadband | 5 µs (≈ 0.45°) | | smallest µs step that moves a 100 mm pointer |
| gear backlash (powered, holding) | not published | | rock the horn, read the pointer tip |
| speed under load | 0.10 s/60° at 4.8 V, no load (600°/s) | | 240 fps video of a 60° step |
| stall torque | ~1.8–2.2 kg·cm (listings vary) | | not needed if R5 passes easily |

### What the requirements imply for gearing (estimate, UNVERIFIED)

From `G ≈ a / r_p` and the published figures, with usable travel taken as ±45°:

| requirement | limit on `G` |
|---|---|
| R1: 7° from ±45° of servo | `G ≥ 0.16` |
| R3: 140°/s from 600°/s (less under load) | `G ≥ 0.23` |
| R2 with backlash 0 / 0.25° / 0.5° / 1.0° (joint play 0.1 mm) | `G ≤ 0.51 / 0.33 / 0.24 / 0.16` |

**Backlash decides whether any design passes.** At about 0.5° the window closes
to 0.23–0.24; at 1° it's empty, so the fix would be a looser `e`, spring-preloaded
horns, or a different servo. A ~15 mm horn hole at `r_p = 80 mm` gives
`G ≈ 0.19`, probably too slow, so **`r_p` is now the main design knob** (the
80 mm hub is no longer fixed). The evaluator replaces these estimates with the
real kinematics.

---

## Still valid from Phase 0

- Conventions: mm/rad, `(3, 6)` anchors, pose order `(R, T)`.
- IK branch `-` for all six legs (reopens if shafts are canted).
- Horizontal servo shafts, base ring and servo-plane parameterisation
  (`docs/derivation.md` §8), `z_flat` datum (§9), one shared home angle (§10).
- Envelope: tilt only, no translation or yaw; 29-pose grid over the `[30°, 90°]`
  azimuth window.
- `r_b ≤ 90 mm` from the 180 × 180 mm print bed.

Phase 0's sweep result (`a = 60.4 mm`, `d = 126 mm`, `beta = 5°`,
`beta_p = 52.5°`) is **superseded**: it needed a long aftermarket arm and ranked
on reach alone.

## Open

- **Confirm `e`, `k` and the latency fraction** (PROPOSED above).
- **Servo controller** not chosen. Its command step must be finer than the
  deadband: Arduino `Servo.write()` moves in ~11 µs, so use
  `writeMicroseconds()`; a PCA9685 board steps in ~4.9 µs.
- **Camera** not chosen. A 30 fps camera uses 33 ms of R4's 50 ms by itself.
- **Ball joints** not chosen; their free play enters R2.
- **Platform mass** unknown; needed for R5.
- **Rotation convention** behind roll/pitch/yaw not chosen.
- **Naming clashes** with no agreed answer: `R` (sinusoid amplitude), `s`
  (screw direction), `t` (plate thickness), `a` / `A_i`, `p` (pitch).
- **No git remote.** Every commit lives on one machine.

## Next steps

1. **Before the battery arrives:** measure the horn holes, spline and servo
   footprint; write the servo test sketch; build the pointer rig; measure
   camera latency.
2. **Battery check:** MG90S is 4.8–6 V. 4× AA is fine; 2S LiPo needs a
   regulator. Common ground with the Arduino; don't power servos from USB.
3. **Bench-test one MG90S** and fill in the measured column.
4. **Write the evaluator** (`stewart/performance.py`): for one design, check
   R1–R3 over the envelope using `ik` and its derivatives.
5. **List a few dozen easy-to-build designs** (stock horn holes, `delta = 0`,
   round-number angles, easy rod lengths), evaluate them, and pick the simplest
   that passes.
6. CAD, then order the remaining parts.
