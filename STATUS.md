# Status

Where the project stands right now. **Edit this file in place each session**;
`git log -p STATUS.md` is the history. Last updated 2026-10-02.

## Direction (since 2026-09-13)

- **Goal:** balance a ping-pong ball on the plate. The design to build is the
  **simplest one to make that passes every requirement below**, not the
  highest-scoring one.
- **Servo:** TowerPro MG90S.  The stock horn is kept only as the *spline
  interface*: a printed arm is over-printed around it (print pause, horn
  dropped in) and carries the rod-end bolt at **`a = 22.5 mm`** (requirements:
  `docs/hardware.md` §12).  Reasons: the stock holes (1 mm) are
  too small for an M3 rod end, aluminium 20T horns for the 4.8 mm micro spline are
  not reliably available (sources disagree 20T vs 21T), a printed spline is out
  (0.75 mm tooth pitch), and `a = 22.5 mm` beats the stock 15.95 mm.  (The
  "longer is always better" version of that last reason is wrong: with `r_p`
  fixed, `a = 25` gave a worse R2 than 22, because the extra gearing
  amplifies the deadband faster than it dilutes the play — design log,
  16 September.)
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

**R2 is judged in quadrature over the six legs** (decided 2026-09-16): within
a leg the errors add, but across legs they are independent, and summing
assumes all six conspire at once.  The worst-case sum (0.342° at the design to
build) is kept as the wear-and-surprise reserve; if the six ever did align, the
ball would wander ±8.6 mm instead of ±8.

### MG90S parameters

Measured 2026-09-16 on **servo 1** (supply voltage not recorded).  Figures
below are that unit; per-unit numbers go in the calibration table that follows.

| parameter | published | **measured** | how to measure |
|---|---|---|---|
| stock horn hole distances | — | **7 holes, 4.15 → 15.95 mm in 1.967 mm steps**: 4.15, 6.12, 8.08, 10.05, 12.02, 13.99, 15.95 | calipers, spline centre to each hole |
| spline tooth count | 20 or 21 (listings disagree) | **20** (counted 2026-09-16) → horn fits every 18°, so up to 9° of trim per leg (≈ 207 µs) | count |
| body + mounting-tab footprint | — | **measured 2026-09-18** (below) | calipers |
| degrees per µs | ~0.09 (estimated, not published) | **0.087** (133° / 90° / 46° at 1000 / 1500 / 2000 µs; linear to ±2%) | protractor at 1000 / 1500 / 2000 µs |
| usable travel | ~90–180° (listings vary) | **530–2480 µs = 169.7°**, centred on 1505 µs (within 5 µs of nominal centre); working limits 550–2460 µs = ±83° | same test, find the ends |
| deadband | 5 µs (≈ 0.45°) | **4 µs = 0.35°** | smallest µs step that moves a 100 mm pointer |
| gear backlash (powered, holding) | not published | **1–2° free; NEGLIGIBLE under one-way preload** (2026-09-16) — the platform's weight preloads every servo, so the free figure does not apply in service | rock the horn, read the pointer tip |
| speed under load | 0.10 s/60° at 4.8 V, no load (600°/s) | **286°/s no load; 250 to 150 g·cm; 240 at 191; 207 at 382; 162 at 556** (less than half the published no-load figure) | 240 fps video of a 60° step |
| stall torque | ~1.8–2.2 kg·cm (listings vary) | not measured (not needed; R5 has huge margin) | not needed if R5 passes easily |

### MG90S case (calipers, 2026-09-18)

Along the shaft, from the top: stock-horn top → case bottom **34.6**; spline
top → case bottom **32.65**.  Stacked from the spline top:

| step | mm | from spline top |
|---|---|---|
| spline top → gear-boss top | 4 | 4 |
| gear boss (to the main case's top face) | 6 | 10 |
| main case top → tabs | 1.8 | 11.8 |
| tab thickness | 2.75 | 14.55 |
| tabs → case bottom | 18 | 32.55 |

Main case top → bottom is **22.55** (= 1.8 + 2.75 + 18); the stack closes to
0.1 mm against the direct 32.65.  Across: body **22.75 × 12.3**, tabs **32.2**
across, shaft centre **16** from the far end of the body (6.75 from the near
end), spline **⌀4.6**.  So with the case hanging below the shaft: body 6.75
above / 16 below the shaft centre, tabs 11.5 above / 20.7 below.  At a 30 mm
shaft height the lower tab clears the plate by 9.3 mm.  Tab slot spacing is
left to the bracket CAD.

### Per-servo calibration

Label the servos 1–6 with tape **before** assembly; each leg's firmware needs
its own centre and deg/µs.  Servo 1 is the unit bench-tested 2026-09-16.

| servo | low end µs | high end µs | centre µs | travel ° | deg/µs | deadband µs | zero µs (arm level) | +µs raises tip? | notes |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 530 | 2480 | 1505 | 169.7 | 0.087 | 4 | 1550 | yes | full bench test 2026-09-16 |
| 2 | 520 | 2480 | 1500 | 170.5* | | 4 | 1500 | no | measured 2026-09-18 |
| 3 | 520 | 2480 | 1500 | 170.5* | | 4 | 1410 | yes | measured 2026-09-18 |
| 4 | 510 | 2480 | 1495 | 171.4* | | 4 | 1590 | no | measured 2026-09-18 |
| 5 | 510 | 2490 | 1500 | 172.3* | | 4 | 1560 | yes | measured 2026-09-18 |
| 6 | 510 | 2490 | 1500 | 172.3* | | 4 | 1470 | no | measured 2026-09-18 |

\* travel ° computed with servo 1's 0.087 deg/µs until each unit's own
figure is measured.

**All six measured 2026-09-18**, deadband 4 µs on every unit.  Low ends span 510–530 µs, high ends
2480–2490, centres 1495–1505 — tight enough to drive the set from one common
limit pair and leave the rest to per-leg trim.  The range every unit reaches is
530–2480 µs, so the **common working limits are 550–2460 µs = ±83° about 1500**,
which is exactly the `travel_deg = 83` that `stewart/performance.py` assumes
(previously extrapolated from servo 1 alone, now confirmed for the set).

Working limits are 20 µs inside each end.  Centre = midpoint of the two ends;
servo 1 landed within 5 µs of the nominal 1500.  Only travel is needed from
2–6 right now — deg/µs and deadband can follow, and matter per leg only for
the trim table.

### DESIGN TO BUILD (2026-09-18)

    r_b = 80, beta = 10, delta = 0, r_p = 70, beta_p = 35, a = 22.5, d = 70
    home height 63.2 mm above the shaft plane
    (a = 22.5 is the printed arm as made, 2026-09-18; was 22)
    shaft height 30 mm above the base-plate top (chosen 2026-09-18), so the
    platform ball centres sit 93.2 mm above the plate at home.  The printed
    arm reaches 26.5 mm from the shaft axis (measured 2026-09-18), 26.35 mm
    below it at the ±83° end stop: 3.65 mm to the plate, room for M3 pan heads.

| check | value | limit | |
|---|---|---|---|
| R1 tilt, every azimuth | 4.5° held, 14.4° of servo travel used | ≥ 4.5° | PASS |
| R2 tilt error (quadrature) | 0.140° | ≤ 0.25° | PASS |
| R3 tilt rate | 80.7°/s | ≥ 65°/s | PASS |
| rod-end misalignment, arm end (straight bolt) | 25.1° | ≤ ~30° | PASS |
| rod-end misalignment, plate end (best bolt axis) | 4.2° | ≤ ~30° | PASS |
| rod–rod | 20.1 mm | — | clear |
| rod–own case / own bracket | 14.6 / 15.8 mm | — | clear |
| rod–other case / other bracket | 36.9 / 44.8 mm | — | clear |
| arm–other case | 42.4 mm | — | clear |
| arm–arm | 30.8 mm | — | clear |

Clearances as of 2026-09-18 with every part at its measured position along
the shaft (bracket window face 22.21 mm behind the rod-end ball plane).

Supersedes the 2026-09-16 candidate (`r_b = 90, beta = 5`): the **horn
extension** (32.3 × 12 × 4.95 mm, measured 2026-09-18) turned out to be the
tight part, not the rods, and `beta` is what governs it.

**Case clearances re-run 2026-09-18** with `Body` corrected (it had the case
12.3 mm along the shaft, centred on it; the real case runs 28.65 mm back from
the gear-boss top).  **Build constraint: in each pair the horns face each other and the
cases point away.**  The pair's shafts are 27.8 mm apart, so cases pointing
inward collide (horn–case 0.0 mm).  `Body.sides` encodes the choice.

**Sensitivity (2026-09-18), one parameter at a time about the design above.**
The grid of the 540-candidate search that produced `r_p = 70, beta_p = 35,
d = 70, delta = 0` was never recorded, so these are checked after the fact:

| varied | range tried | result |
|---|---|---|
| `r_p` | 55–80 | all pass; R3 falls from 100 to 69°/s as `r_p` grows |
| `beta_p` | 20–50 | all pass; R2 0.180° at 20, flat at 0.135° from 40 up |
| `d` | 55–90 | all pass; R1–R3 unchanged, only the home height moves (46–85 mm) |
| `delta` | 0–40, 170 | R1–R3 pass, R2 improves slightly (0.124° at 40), **but the parts collide**: horn–horn 5.0 / 0.5 / 0.1 mm at 10 / 20 / 40, horn–case 0.0 at 170 (= −10) |
| `a` | 18–26 | **18 fails R3** (64.5°/s); 20–26 pass, R2 rising with `a` |

So the design sits in a flat passing region: the values are *a* passing
point, not an optimum, which is what "simplest that passes" asked for.
`delta = 0` (servo planes tangent) is the simplest to build, and it is also
**the only `delta` with room**: turning the servos one way swings a pair's
horns into each other, the other way swings each horn into its partner's case.

### Mounting: shafts horizontal (assumption, load-bearing)

**Every servo shaft must sit parallel to the base plane**, with the arm
sweeping in a vertical plane.  This is not a convenience — the IK's fixed
*minus* branch rests on it (`ik` docstring; `docs/derivation.md` §8): with
horizontal shafts `v_i = z` exactly, so `N_i = L_i · z` is the anchor height
above the base, positive at every pose the platform can hold, which is what
makes one branch correct for all six legs at once.  **Cant the shafts and the
branch choice reopens**, and `base_ring`'s parameterisation no longer describes
the machine.

Tolerance: a cant of `ε` displaces the arm tip out of its plane by
`a sin(α) sin(ε)`, at most (α ≤ 14.4°, a = 22.5 mm):

| cant | tip error |
|---|---|
| 0.5° | 0.05 mm |
| 1° | 0.10 mm |
| 2° | 0.19 mm |
| 3° | 0.29 mm |

**Aim for ≤ 1°.**  Unlike joint play this error is *systematic* — a repeatable
function of `α`, not random — so calibration and camera feedback absorb most of
it, which is why 1° is a target rather than a hard limit.  It is still worth
the jig: six brackets each canted a different way is six different systematic
errors.

`Body` in `performance.py` assumes the same thing (case hanging below the
shaft), so the clearance figures inherit it.

### Base radius and clearance (2026-09-18)

`r_b = 80` (chosen 2026-09-18): every `r_b` from 70 to 90 mm passes, R3 and
the joint cone don't depend on it, and 80 costs 0.018° of R2 against 90 for
10 mm of radial room.  The comparison table is in the design log and git
history.

**`beta` decides whether the horns fit.**  With the horn extension modelled
(`horn_clearance()`), horn-to-neighbouring-case at `r_b = 80` is:

| beta | 2° | 5° | 10° | 15° |
|---|---|---|---|---|
| horn–case, old `Body` | **0.0 — collides** | 2.7 mm | 13.6 mm | 24.1 mm |
| horn–case, corrected `Body`, before the final axial placement | 5.2 mm | 12.8 mm | 25.1 mm | 36.6 mm |

With the case modelled correctly `beta = 5` would also have cleared, so
`beta = 10` was chosen on a wrong model.  It still costs only 0.002° of R2 and
nothing in R3 or the cone, so it stays.  How far the horn reaches
back past the spline (`Horn.behind`) changes none of this: 4 mm and 16 mm give
the same answer to 0.1 mm, because what nearly touches is the horn's flank
against the neighbour, not its tail.

Clearances come from `clearance()` and `horn_clearance()`, which model the
servo case as a box (`Body`, measured 2026-09-18) and the arm as a box
(`Horn`), and sample each rod at 25 points.  **Not modelled:** the base plate
itself and the plate underside.

**Base footprint (2026-09-18, final).**  With the real brackets and their
lugs, the footprint reaches r = 104.1 mm, so the base plate is at least
~210 mm across and is cut from sheet.  The earlier 177–180 mm figure counted
the servo cases only.

---

## Still valid from Phase 0

- Conventions: mm/rad, `(3, 6)` anchors, pose order `(R, T)`.
- IK branch `-` for all six legs (reopens if shafts are canted).
- Horizontal servo shafts, base ring and servo-plane parameterisation
  (`docs/derivation.md` §8), `z_flat` datum (§9), one shared home angle (§10).
- Envelope: tilt only, no translation or yaw; 29-pose grid over the `[30°, 90°]`
  azimuth window.
- ~~`r_b ≤ 90 mm` from the 180 × 180 mm print bed~~ — the base plate is cut
  from sheet (≥ ~210 mm), so the bed no longer bounds `r_b`; `r_b = 80` stands.

Phase 0's sweep result (`a = 60.4 mm`, `d = 126 mm`, `beta = 5°`,
`beta_p = 52.5°`) is **superseded**: it needed a long aftermarket arm and ranked
on reach alone.

## Open

- **Ball joints:** M3 rod ends (spherical bearings, M3 female shank, 3 mm
  bore), **arrived and measured 2026-09-18**: ball 4.37 wide, housing 2.53
  thick and Ø9.89, 18.05 overall (housing edge to shank end, so ball centre →
  shank end **13.1**), thread 5.17 deep.  For `d = 70` centre to centre the
  exposed rod is 43.8, so an **M3 rod cut to 54.1** bottoms out in both ends.
  **Six rods cut 2026-09-21.**
  **Bind angle ~22°** bare; **~30° at both ends with an M3 threaded insert
  on each side of the ball as a spacer** (2026-09-18, by hand; inserts 4.02
  long, so the ball centre sits 4.02 + 2.19 = **6.21 mm** out from the arm's
  outer face).  That lets the
  arm keep a **straight bolt, parallel to the shaft**: the rod end then sees
  25.1° over the envelope (5° inside 30°), the plate end 4.2°.  **Play: felt, "very very minor", not quantified** (no dial
  indicator); budget 0.23 mm total per leg.  It is very likely **preloaded
  out**, like the servo backlash: every rod stays in compression under the
  platform's weight, 0.18–0.19 of the weight per rod over the whole envelope,
  and a rough R3 slew (9° in 0.14 s) accelerates the anchors at ~0.2 g, well
  short of unloading them.  Confirm on the assembled platform.
- **Arm extension:** 32.3 × 12 × 4.95 mm, over-printed on the stock horn,
  straight M3 hole at 22.5 mm; requirements in `docs/hardware.md` §12.
  **Not stiffness-tested** — its flex budget is 54 µm of hysteresis at the
  tip, the only part of R2 that is not deadband or joint play.
- ~~Rod-end ball centre along the shaft~~ **settled 2026-09-18:** ball centre
  6.21 mm out from the arm face, arm face 16.0 mm from the bracket window
  face, so each window face is 22.21 mm behind the ball plane.
- **Arm M3 hole: straight, parallel to the shaft** (decided 2026-09-18,
  superseding the 25° tilt).  Base-end misalignment over the envelope at
  `a = 22.5`: straight 25.1°, 15° → 10.7°, 20° → 5.9°, 25° → 0.3°.  The tilt
  needed a seat, a side lobe and two mirrored arms; spacing the ball out on
  threaded inserts raised the bind angle to ~30° instead, so straight passes
  with 5° to spare and one arm fits all six.  If more margin is ever wanted,
  moving the top plate to `r_p 65, beta_p 30` drops the straight-bolt angle
  to 18.6° (stiffness cond 4.8 against 3.8).
- **Servo bracket:** designed 2026-09-18 (C bracket, dimensions in
  `docs/hardware.md` §13) and placed on the base plate (twelve M3 holes,
  §13).  Load case (§13): per 1 N of rod force (±0.085, −0.423, −0.902) N
  at the ball, 22.21 mm in front of the window face, which pulls the upper
  tab screw out.  Filament **eSUN PLA+** (E ≈ 1.9 GPa).  Hand calc: **67 µm
  along the rod at 1 N**, over B4's 60 only at that design load; 31–37 µm at
  the real 0.46–0.56 N.  R2 passes even counting all of it as random (0.204°
  at 1 N, against 0.25°).  **Accepted as printed, 4 mm spine; no FEA, no
  bench check** (2026-09-18).  Lugs, foot and screws are unverified; suspect
  them first if R2 falls short on the assembled platform.
- **Top plate: 3D-printed in PLA** (as of 2026-09-22), anchors to
  `docs/hardware.md` §14 (spec written 2026-09-18, CAD the user's).  Bolt horizontal, parallel to the pair's bisector: 4.2°
  of misalignment (the optimum), one part for all six, no insert spacers.
  Ball-centre depth below the plate is the user's choice (~8 mm suggested).
  Tab radius ≤ 4 mm and a 1 mm seat boss keep the rod-end shank clear.
- **Base plate: laser-cut 5 mm acrylic** (as of 2026-09-22), servos
  mounted on it.  Supersedes the three printed PLA+ wedges planned
  2026-09-19.
- **Check `e = ±8 mm` against camera noise `σ`** in stage 2 (`e ≥ 3σ`).  The
  rest of `e` is settled.
- **Joystick control path: IK on the PC** (decided 2026-09-19).  The Python
  `ik` computes the six pulse widths and sends them over USB serial; the
  Arduino only outputs them.  The joystick is the Arduino kit's analog
  stick, read by the Arduino and streamed to the PC, so the PC side needs
  only a serial library (pyserial) beyond numpy + matplotlib.  **Control
  software written 2026-09-22** (`joystick.py`, `firmware/joystick/`).
  The Arduino reads the calibration from EEPROM and sends it to the PC,
  refuses any pulse > 350 µs from a leg's zero, ramps every move, and
  returns to level if the PC goes quiet for 0.5 s.
- **Servo controller** not chosen. Its command step must be finer than the
  deadband: Arduino `Servo.write()` moves in ~11 µs, so use
  `writeMicroseconds()`; a PCA9685 board steps in ~4.9 µs.
- **Camera** not chosen (stage 2). A 30 fps camera uses 33 ms of R4's 70 ms
  by itself.
- **Platform mass: printed PLA top plate 50.15 g** (weighed 2026-09-22),
  against the 200–250 g the load case assumed.  Rod force from weight is
  ~0.09 N per rod, not ~0.45.  **Stage 1 runs on the printed plate alone**
  (decided 2026-09-22).  A larger plywood deck comes with stage 2; before
  it goes on, check the rod forces over the envelope with its real size and
  mass (a big deck adds inertia faster than weight, and could unload rods
  during an R3 slew, bringing the joint play back).
- **Rotation convention** behind roll/pitch/yaw not chosen.
- **Naming clashes** with no agreed answer: `R` (sinusoid amplitude), `s`
  (screw direction), `t` (plate thickness), `a` / `A_i`, `p` (pitch).

## Next steps

1. ~~Measure the horn holes, spline and servo footprint~~ **DONE** (servo
   test sketch: `firmware/servo_test/`).
2. ~~Battery check~~ **DONE 2026-09-19: 4× AA** for the servos (MG90S is
   4.8–6 V).  Common ground with the Arduino; don't power servos from USB.
3. ~~Bench-test one MG90S~~ **DONE 2026-09-16.**  Re-tests that decide whether
   the servo can pass at all:
   - ~~Backlash under preload~~ **DONE 2026-09-16: negligible.**
   - **Voltage and re-timed speed** (optional now; worth up to 2× if needed).
     Measure the pack voltage under load; count video frames from *first
     movement* to stop, not from the LED.
   - ~~Find the low end of travel, count the spline teeth~~ **DONE.**
   - ~~Calibrate servos 2–6 (travel ends)~~ **DONE 2026-09-18**; per-unit
     deg/µs still open (only needed for the trim table).
4. ~~Write the evaluator~~ **DONE 2026-09-16** (`stewart/performance.py`): R1,
   R2, R3 and the rod-end cone, from numerical derivatives of `ik`/`fk`.  No
   unit tests yet — that is the gap.  Since 2026-09-18 it checks the arm-end
   joint with the as-built straight bolt (`Requirements.base_bolt`).
5. ~~List a few dozen easy-to-build designs~~ **DONE 2026-09-16**: 540
   candidates evaluated; superseded by the design to build (2026-09-18).
6. ~~Settle the `k` vs R1 conflict~~ **DONE 2026-09-16** (`k = 0.2`, R1 × 1.25).
7. CAD, then order the remaining parts.
   - ~~Rod ends: measure play, bind angle, dimensions, ball offset~~ **DONE
     2026-09-18.**
   - ~~FEA of the servo bracket~~ **dropped 2026-09-18** for a hand calc.
     ~~Print the other five brackets~~ **DONE; all six servos mounted on
     the base 2026-09-21.**
   - ~~Assembly~~ **DONE 2026-09-22** (step 8).  Home pose: **plate
     level, no binding, 105 mm from base-plate top to platform top.**
     Expected ball centres at 93.2, so anchor depth + plate thickness
     should total ~11.8 mm; not yet checked against the CAD.
   - **Joystick control written 2026-09-22**, not yet run on the platform:
     `firmware/joystick/` + `joystick.py` (`--selftest` passes: home =
     the zeros, 4.5° uses ≤ 165 µs; direction checked through `fk`).
     **First run 2026-10-02: the joystick drives the tilt and the platform
     follows** (stick orientation left at the defaults).  **The tilt angle
     itself is not yet measured**, so R1–R3 remain predictions.
   - **`motion.py` 2026-09-24**: oscillates one DOF at a time.  **Envelope
     measured** (firmware's 350 µs and a 28° rod-end cone, 2° inside the
     ~30° bind): roll **9.0°**, pitch **9.8°**, yaw **3.0°**, surge/sway
     **3.0 mm**, heave **11.0 mm**.  Roll, pitch and heave are servo-limited;
     **yaw, surge and sway are limited by the rod-end cone** — the arm end
     already sits at 25.1° of ~30° at home (straight bolt), so off-tilt DOFs
     have little room.  Defaults run at ~2/3 of each limit.
   - ~~Base-plate drill positions~~ **DONE 2026-09-18**: twelve M3 holes in
     `docs/hardware.md` §13, plate ≥ ~210 mm across.
8. **Assembly (from 2026-09-21).**  Home is `alpha = 0` on every leg: arm
   horizontal, pointing along `u_i` (inward), rod-end ball centre at the
   shaft height, 30 mm above the plate.  Per servo, before any rod goes on:
   check the shaft is level (≤ 1° cant); drive it to its centre µs; fit the
   arm at the spline tooth nearest horizontal-inward (≤ 9° off); nudge the
   µs until the arm is level and record that as the leg's **zero µs**; note
   whether +µs raises the tip (the two servos in a pair should be opposite).
   Then rods, then the top plate, with every servo held at its zero: ball
   centres should sit 93.2 mm above the plate and the plate should be level.
   Zero µs and sign go in the calibration table (**arms zeroed
   2026-09-22**, 10 µs steps ≈ 0.9°; all offsets within the ±9° a tooth
   allows; directions alternate in every pair).  PC side:
   `ZERO_US = [1550, 1500, 1410, 1590, 1560, 1470]`,
   `SIGN = [+1, -1, +1, -1, +1, -1]` (+1: +µs raises the tip = +alpha),
   so `us = ZERO_US + SIGN · alpha_deg / 0.087`.  Tool:
   `firmware/servo_cal/` (written 2026-09-21; `save`, then `table`).
   **Wiring (2026-09-21):** legs 1–6 signal on D2–D7; joystick VRx/VRy on
   A0/A1, button on D8 (`INPUT_PULLUP`); servo power from the 4× AA rail
   with the Arduino GND tied to it; D0/D1/D13 free.  No capacitor on hand:
   short power leads, star ground at the battery −, moves ramped in
   firmware; add ≥ 100 µF if the servos jitter.
9. Stage 1 done when: the platform follows a joystick through ±R1 tilt in
   every direction.
10. Stage 2: camera, latency measurement, confirm `e` and the latency
   fraction, closed-loop balancing.
