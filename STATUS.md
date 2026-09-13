# Status

Where the project stands right now. **Edit this file in place each session**;
`git log -p STATUS.md` is the history. Last updated 2026-09-13.

**Phase 0 (kinematics and design method) is complete: the sweep ran and chose
the dimensions.** Nothing has been purchased and no CAD exists yet.

---

## Current result

Two candidates, a mirror pair, identical at both published housing bounds
(sweep commit `2fee8b0`: 368 000 screened, 327 780 feasible, all scored):

| | |
|---|---|
| `a` | 60.40 mm (`0.6711 r_b`), ProModeler PDRS60-25T outermost hole |
| `d` | 126 mm (`1.40 r_b`) |
| `beta` | 5° |
| `beta_p` | 52.5° |
| `z_home` | 95.62 mm |
| score, `margin(dxy = 0.003849)` | 0.874876 (tie floor 0.874726) |

Fixed upstream, all asserted: `r_b = 90 mm` (180 × 180 print bed), `r_p = 80 mm`
(printed hub carrying a bought 220 mm sheet), `c_p / r_b = 0.1`, tilt limit
6.558°.

**Build checks pass with room to spare.** `min(C_i - |P_i|) = 90.96 mm` against a
0.3464 mm build-error stack (263×). The margin survives a 37.9 mm rod-cutting
error. Worst leg force 37.7 mN (2.7 g ball); Euler buckling passes by 19× at the
thinnest published rod. Anchor separation clears the full 9–13 mm ball-joint
housing range; both members have 2.71 mm of arc slack in `beta`.

**The one caveat.** `a = 60.40 mm` is the top of the published horn ladder, not
an optimum the analysis found. Every dimension traces to the analysis except
`a`, which traces to a catalogue.

## Settled decisions

- **Score:** `score = margin(dxy = p)`, the maximin normalised reach margin
  `(C_i - |P_i|) / C_i` over legs and the 29-pose envelope, read at build error
  `p = 0.003849`. `delta` is tuned per candidate to maximise `margin(0)` subject
  to `cond(J_fk) <= 1e6` at `char_len = r_b`. No weights. Output is a tie set.
- **Feasibility** (pass/fail before scoring): non-empty `z_home` bracket,
  envelope reachable (`|P_i| <= C_i`), `N_i > 0`, bracket width ≥ 2 mm. Servo
  travel is deliberately **not** a feasibility test.
- **IK branch:** `-` for all six legs. Reopens if the shafts are ever canted.
- **Pose order** `(R, T)` in every signature.
- **Envelope:** tilt only, 6.558° limit (`tau_L` latency term dropped — servo
  step response is unpublished).
- `tau_min` and tilt authority both rank *with* the margin, so neither is a
  score term.

## Open

- **The objective.** Reach margin measures distance from unreachability, not
  capability. Three axes (`r_p`, `beta`, `a`) got their limits from hardware or
  decisions, never from the objective. Nothing is blocked by this.
- **Pairing argument / discrete `mu` residue.** `mu ∈ {0, 180}` was derived
  under the identity pairing, but the pairing result ranged over all 720
  bijections. Needs the leg-1 angular-span multiset check (never run). Changes
  how the result is stated, not the sweep.
- **`tau_L` re-check.** If servo step response is ever measured and the
  envelope moves, re-run the shortlist before CAD.
- **Servo mounting-arc footprint.** `beta`'s bound uses bare case width
  (13.0 mm); flanges and wiring will only tighten it.
- **`char_len`.** Still needed to quote the `cond` cap and to bound FK residual →
  pose error. Also open: whether `cond(J_fk)` is the right measure at all.
- **Tie threshold.** `TIE_TOL = 1e-12` is right for `dxy = 0` but the score at
  `dxy = p` is only resolved to ~`1.5e-4`.
- **`X = 1` crossing correction** was measured at 8.5–14.6° tilt; not re-run at
  6.558°.
- **Rod straightness** unpublished for the candidate stock; the `d / r_b <= 2.0`
  cap is a judgement.
- **Platform self-weight** is not in the leg-force check (no material chosen).
- **Rotation convention** behind roll/pitch/yaw is not chosen.
- **Naming clashes** without an agreed answer: `R` (sinusoid amplitude), `s`
  (screw direction), `t` (plate thickness), `a` / `A_i`, `p` (pitch).
- **No git remote.** Every commit lives on one machine.

## Next steps

1. **Pick the mirror, then CAD.** The two shortlist members are the same
   machine reflected (arms on opposite sides); choosing is a build decision.
2. **Choose the joint, servo and rod stock.** Servo on torque and deadband; the
   Hitec HS-5055MG deadband conflict (6 µs manufacturer vs 2 µs retailer) is
   unresolved and affects tilt resolution. Specs: `docs/hardware.md`.
3. **Order parts.**
4. Rewrite "Where Phase 0 stands" at the end of `docs/design-log.md` in your own
   voice.
