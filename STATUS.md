# Status

Where the project stands right now. **Edit this file in place each session**;
`git log -p STATUS.md` is the history. Last updated 2026-09-16.

## Direction (since 2026-09-13)

- **Goal:** balance a ping-pong ball on the plate. The design to build is the
  **simplest one to make that passes every requirement below**, not the
  highest-scoring one.
- **Servo:** TowerPro MG90S.  **Superseded 2026-09-16:** the stock horn is
  kept only as the *spline interface* — a printed two-plate clamp bolts to it
  through two existing holes and carries the rod-end bolt at `a ≈ 22 mm`
  (requirements: `docs/hardware.md` §12).  Reasons: the stock holes (1 mm) are
  too small for an M3 rod end, aluminium 20T horns for the 4.8 mm micro spline are
  not reliably available (sources disagree 20T vs 21T), a printed spline is out
  (0.75 mm tooth pitch), and `a = 22 mm` beats the stock 15.95 mm.  (The
  "longer is always better" version of that last reason is wrong — see the
  leading-candidate section: past ~22 mm the extra gearing amplifies the
  deadband faster than it dilutes the play.)
- **Build order (since 2026-09-15):** stage 1 is the platform driven by a
  joystick, no camera. Stage 2 adds the camera and self-balancing. The
  geometry built in stage 1 is still sized to R1–R3, because `G` is fixed by
  the hardware and decides whether stage 2 can pass. Only what needs a camera
  (R4, the latency fraction, camera choice) waits for stage 2.
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
| `e` | how far the ball may wander from target once settled | **±8 mm** | CONFIRMED 2026-09-16 (was ±5; loosened so the joints and `r_p` are buildable) |
| `x0`, `τ` | disturbance to recover from, and how fast | **50 mm in 0.7 s** | CONFIRMED 2026-09-16 (was 0.5 s; R3 ∝ 1/τ³ made 0.5 s force `r_p ≈ 22 mm`) |
| `k` | fraction of `τ` the plate may spend slewing between tilts | 0.2 | CONFIRMED 2026-09-16 (R1 carries a ×1.25 slew factor to match) |
| latency fraction | loop delay as a fraction of `τ` | 0.1 | DEFERRED to stage 2 (rule of thumb) |

### Pass/fail requirements

| # | requirement | formula | value |
|---|---|---|---|
| R1 | **Tilt range**, every direction, at home height | `sin θ = 7 (4 x0 / τ²) / 5g`, × 1.25 for the `k = 0.2` slew profile: 4.18° at `τ = 0.7 s` | **≥ 4.5°** |
| R2 | **Tilt precision**: resolution and repeatability together | drift `½ (5/7) g sin δ · τ² ≤ e` → `δ ≤ 0.267°` at ±8 mm / 0.7 s | **≤ 0.25° total** |
| R3 | **Tilt speed** | swing `2 × R1` in `k τ` = 9° / 0.14 s | **≥ 65°/s** |
| R4 | **Loop latency**: camera + processing + servo | `0.1 τ` | **≤ 70 ms** (stage 2) |
| R5 | **Torque** | holding load + ball, with margin | ≤ 25% of stall (sanity check) |

What sets R2: any tilt error the loop can't remove, held for `τ`, lets the ball
drift `½ (5/7) g sin δ τ²`. At `τ = 0.7 s`: 0.1° → 3.0 mm, 0.25° → 7.5 mm,
0.5° → 15 mm. The error budget R2 has to hold is

    G × (servo deadband + gear backlash)  +  ball-joint play / r_p   ≤  0.25°

where `G` is the gearing, degrees of plate tilt per degree of servo rotation,
roughly `a / r_p`. Deadband and backlash add because camera feedback removes a
steady offset but not random play. **Backlash is ~0 under preload** (measured
2026-09-16), so the live terms are the 0.35° deadband and the joint play.

### MG90S parameters

Measured 2026-09-16 on one MG90S (supply voltage not recorded).

| parameter | published | **measured** | how to measure |
|---|---|---|---|
| stock horn hole distances | — | **7 holes, 4.15 → 15.95 mm in 1.967 mm steps**: 4.15, 6.12, 8.08, 10.05, 12.02, 13.99, 15.95 | calipers, spline centre to each hole |
| spline tooth count | 20 or 21 (listings disagree) | **20** (counted 2026-09-16) → horn fits every 18°, so up to 9° of trim per leg (≈ 207 µs) | count |
| body + mounting-tab footprint | — | **35.3 mm along the shaft axis, 12.3 thick, 32.2** (which span is 32.2 needs confirming) | calipers (bounds `beta`) |
| degrees per µs | ~0.09 (estimated, not published) | **0.087** (133° / 90° / 46° at 1000 / 1500 / 2000 µs; linear to ±2%) | protractor at 1000 / 1500 / 2000 µs |
| usable travel | ~90–180° (listings vary) | **530–2480 µs = 169.7°**, centred on 1505 µs (within 5 µs of nominal centre); working limits 550–2460 µs = ±83° | same test, find the ends |
| deadband | 5 µs (≈ 0.45°) | **4 µs = 0.35°** | smallest µs step that moves a 100 mm pointer |
| gear backlash (powered, holding) | not published | **1–2° free; NEGLIGIBLE under one-way preload** (2026-09-16) — the platform's weight preloads every servo, so the free figure does not apply in service | rock the horn, read the pointer tip |
| speed under load | 0.10 s/60° at 4.8 V, no load (600°/s) | **286°/s no load; 250 to 150 g·cm; 240 at 191; 207 at 382; 162 at 556** (less than half the published no-load figure) | 240 fps video of a 60° step |
| stall torque | ~1.8–2.2 kg·cm (listings vary) | not measured (not needed; R5 has huge margin) | not needed if R5 passes easily |

### Leading candidate (2026-09-16, from `stewart/performance.py`)

    r_b = 90, beta = 5, delta = 0, r_p = 70, beta_p = 35, a = 22, d = 70
    home height 60.2 mm, servo angle used +/-14.6 deg of the +/-83 available

| check | value | limit | |
|---|---|---|---|
| R1 tilt, 72 azimuths | 4.5° held | ≥ 4.5° | PASS |
| R2 tilt error, **in quadrature** (judged) | 0.117° | ≤ 0.25° | PASS |
| R2 tilt error, worst case (reserve) | 0.287° | — | — |
| R3 tilt rate | 78.6°/s | ≥ 65°/s | PASS |
| rod-end misalignment | 4.2° | ≤ 13° | PASS |

**This candidate passes every check.**  R2 is judged in quadrature over the six
legs (decided 2026-09-16, see the design log): within a leg the errors add, but
across legs they are independent, and summing assumes all six conspire at once.
The worst-case sum is kept as the wear-and-surprise reserve.

Joint play is the sensitive input either way (0.1 mm at `a = 22` is 0.26° of
equivalent servo error, against 0.175° of deadband).  How much this candidate
can take:

| play (total per leg) | 0.10 mm | 0.20 mm | 0.25 mm | 0.29 mm |
|---|---|---|---|---|
| R2, quadrature | 0.117° | 0.187° | 0.222° | **0.250° — limit** |

So **anything under ~0.28 mm passes**, which ordinary M3 rod ends should clear
comfortably.  (On the worst-case reading the limit would have been 0.06 mm, and
no geometry in a 540-candidate sweep met it.  The cost of the looser reading is
0.6 mm of extra ball wander if the six errors ever do align: ±8.6 mm instead of
±8 mm.)

Two findings worth keeping:

- **`a = 25` is worse than `a = 22`** at fixed `r_p` (0.312° vs 0.287°).  The
  earlier "longer arm always helps" reasoning held `G` fixed by scaling `r_p`;
  with `r_p` capped by the print bed, a longer arm just raises `G` and
  amplifies the deadband more than it shrinks `play / a`.
- **The rod-end bolt at the arm must NOT be parallel to the servo shaft.**
  Parallel gives 31° of misalignment and binds.  The best axis per leg is
  ~30° off the shaft axis, tilted toward the direction of arm rotation
  (`evaluate().axes_base`).  At the plate the best axis is in-plane and
  roughly tangential (`axes_platform`), as expected.

### Gearing after the preload re-test (2026-09-16)

**Kept for the `τ` / `e` reasoning; its geometry numbers are superseded** by
the leading-candidate section above, which uses the real kinematics and
`a = 22 mm` rather than `G ≈ a / r_p` and the stock horn.

**Chosen spec: `τ = 0.7 s`, `e = ±8 mm`** → R1 4.5°, R3 65°/s, `δ` 0.267°.

Backlash under one-way preload is negligible, so the binding budget is

    rate_max  =  ω δ / (deadband + joint play / a)

with `ω = 250°/s` (measured, ~150 g·cm), deadband 0.35°, `a = 15.95 mm`.
**Ball-joint play is now the dominant unknown** — 0.1 mm at `a` is 0.36°, as
large as the deadband — and it is a purchasing decision, not a measurement.

What each (`τ`, `e`) asks of the joints (total play per leg, both ends,
`ω = 250°/s`):

| `τ` | `e` | R1 | R3 | `δ` | max joint play |
|---|---|---|---|---|---|
| 0.5 s | ±5 mm | 8.2° | 164°/s | 0.327° | 0.04 mm |
| 0.5 s | ±10 mm | 8.2° | 164°/s | 0.654° | 0.18 mm |
| 0.7 s | ±8 mm | 4.2° | 60°/s | 0.267° | 0.21 mm |
| 0.8 s | ±10 mm | 3.2° | 40°/s | 0.256° | 0.35 mm |

Passing the inequality is not enough — the `G` window also has to give a
buildable `r_p = a / G`:

| case | `G` window | `r_p` |
|---|---|---|
| `τ` 0.5, `e` ±10, play 0.15 mm | 0.66–0.74 | **22–24 mm** — anchor circle far too small |
| `τ` 0.7, `e` ±8, play 0.1 mm | 0.24–0.38 | **42–67 mm** |
| `τ` 0.8, `e` ±10, play 0.05 mm | 0.16–0.48 | **33–100 mm** |

R3 ∝ 1/τ³ and it sets `G ≥ R3/ω`, which caps `r_p`.  Holding `τ = 0.5 s`
forces a stubby 22 mm anchor circle; **relaxing `τ` to 0.7–0.8 s is what buys a
sane platform.**  All of this uses `G ≈ a / r_p`; the evaluator replaces it
with the real kinematics.

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

- **Ball joints:** M3 rod ends (spherical bearings, M3 female shank, 3 mm
  bore) are the candidate — metal-on-metal, and the M3 threaded push-rod makes
  `d` adjustable per leg.  Two checks before buying twelve: measured play
  ≤ 0.1 mm each (budget 0.21 mm per leg), and the **misalignment cone**
  (a rod end binds at ±13–15° off its bolt axis — the evaluator must check leg
  swing against it, and the bolt-axis orientation on base and plate is a design
  choice).
- **Horn extension:** the user is designing it; requirements are in
  `docs/hardware.md` §12.  Its flex budget is 54 µm of hysteresis at the tip —
  the only part of R2 that is not deadband or joint play.
- **Check `e = ±8 mm` against camera noise `σ`** in stage 2 (`e ≥ 3σ`).  The
  rest of `e` is settled.
- **Joystick control path** not chosen: IK on the Arduino (port `ik`) or on a
  PC sending servo commands over serial.
- **Servo controller** not chosen. Its command step must be finer than the
  deadband: Arduino `Servo.write()` moves in ~11 µs, so use
  `writeMicroseconds()`; a PCA9685 board steps in ~4.9 µs.
- **Camera** not chosen (stage 2). A 30 fps camera uses 33 ms of R4's 70 ms
  by itself.
- **Ball joints** not chosen; their free play enters R2.
- **Platform mass** unknown; needed for R5.
- **Rotation convention** behind roll/pitch/yaw not chosen.
- **Naming clashes** with no agreed answer: `R` (sinusoid amplitude), `s`
  (screw direction), `t` (plate thickness), `a` / `A_i`, `p` (pitch).
- **No git remote.** Every commit lives on one machine.

## Next steps

1. **Before the battery arrives:** measure the horn holes, spline and servo
   footprint; build the pointer rig.  The servo test sketch is written:
   `firmware/servo_test/` (compiles for Uno; test procedures in its header).
   Install the Servo library in the Arduino IDE before uploading.
2. **Battery check:** MG90S is 4.8–6 V. 4× AA is fine; 2S LiPo needs a
   regulator. Common ground with the Arduino; don't power servos from USB.
3. ~~Bench-test one MG90S~~ **DONE 2026-09-16.**  Re-tests that decide whether
   the servo can pass at all:
   - ~~Backlash under preload~~ **DONE 2026-09-16: negligible.**
   - **Voltage and re-timed speed** (optional now; worth up to 2× if needed).
     Measure the pack voltage under load; count video frames from *first
     movement* to stop, not from the LED.
   - Find the **low end of travel** (2480 µs is only the high end) and count
     the spline teeth.
4. ~~Write the evaluator~~ **DONE 2026-09-16** (`stewart/performance.py`): R1,
   R2, R3 and the rod-end cone, from numerical derivatives of `ik`/`fk`.  No
   unit tests yet — that is the gap.
5. ~~List a few dozen easy-to-build designs~~ **DONE 2026-09-16**: 540
   candidates evaluated.  Leading candidate below; the pick is gated on the
   measured joint play, not on more searching.
6. Settle the `k` vs R1 conflict before step 5 picks `G`.
7. CAD, then order the remaining parts.
8. Stage 1 done when: the platform follows a joystick through ±R1 tilt in
   every direction.
9. Stage 2: camera, latency measurement, confirm `e` and the latency
   fraction, closed-loop balancing.
