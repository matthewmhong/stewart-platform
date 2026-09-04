# Notation reference

Every quantity used anywhere in the project, in one place.

**Provenance column.** `key` = already in `stewart-ik-derivation.md`'s quantities
table. `body` = used in the derivation body but not in its key. `new` =
introduced during working sessions and not yet in any document — these are the
ones to accept or reject, not to assume.

**Conventions.** Millimetres and degrees at every API boundary; radians
internally. Legs are 0-indexed in code (`i = np.arange(6)`, `floor(i/2)`) and
1-indexed in error messages and prose (`floor((i-1)/2)`). Both appear; check
which you are reading.

---

## 1. Frames

| Symbol | Meaning | Provenance |
|---|---|---|
| `{W}` | World frame. Fixed to base and table, never moves. Origin at the base ring centre, `z` up, base plate at `z = 0`. | key |
| `{P}` | Platform frame. Origin at the platform centre, axes glued to the plate, tilts with it. Its origin's height relative to the anchor plane is **not yet decided** — see §11. | key |

## 2. Commanded pose

| Symbol | Meaning | Frame / units | Known before solving? | Provenance |
|---|---|---|---|---|
| `T` | position of the platform centre | `{W}`, mm | **yes** — half the commanded pose | key |
| `R` | platform orientation; converts `{P}` components to `{W}` components | `{P} → {W}` | **yes** — the other half | key |
| roll, pitch, yaw | the three angles parameterising `R` | deg | **yes** | new |

The order and axis convention behind roll/pitch/yaw is an **open item (§6)**. Any
code written before it is settled is provisional.

## 3. Base ring

| Symbol | Meaning | Units | Status | Provenance |
|---|---|---|---|---|
| `r_b` | base ring radius | mm | swept | key |
| `beta` | pair half-split | deg | swept; range set by servo-body collision | key |
| `s_i` | within-pair sign, `(-1, +1, -1, +1, -1, +1)`. Also the `C3` orbit label: legs 0,2,4 in one orbit, 1,3,5 in the other. | — | fixed by the parameterisation | body |
| `theta_i` | shaft azimuth, `120*floor(i/2) + s_i*beta` | deg | derived | key/body |
| `b_i` | servo shaft position, `r_b (cos theta_i, sin theta_i, 0)` | mm, `{W}` | design constant | key |
| `delta` | servo-plane twist: how far each servo is rotated on its mounting hole, measured from tangential, alternating within each pair | deg, `[0, 180)` | **inner tune**, not a sweep axis | key |
| `psi_i` | azimuth of `n_i`, `theta_i + 90 + s_i*delta`. The `+90` is forced by the mirror condition, not chosen. | deg | derived | body |
| `n_i` | unit normal of servo `i`'s rotation plane | `{W}` | design constant | key |
| `u_i` | unit vector in that plane, direction of `alpha_i = 0`. `u_i = z x n_i`, so `alpha = 0` lays the arm flat in the base plane. | `{W}` | design constant | key |
| `v_i` | `n_i x u_i`. Equals `z` exactly while `n_i` is horizontal, so positive `alpha_i` raises every arm tip. | `{W}` | follows from `n_i`, `u_i` | key |

## 4. Platform ring

| Symbol | Meaning | Units | Status | Provenance |
|---|---|---|---|---|
| `r_p` | platform ring radius | mm | swept | new |
| `beta_p` | pair half-split | deg | swept; range set by ball-joint housing diameter, **not yet known** | new |
| `phi_i` | anchor azimuth, `120*floor(i/2) + s_i*beta_p` — same skeleton as `theta_i`, which is what makes leg `i` pair with leg `i` | deg | derived | new |
| `p_i` | platform anchor position | mm, `{P}` | design constant | key |
| `h_p` | plate offset: the common `z` component of `p_i` in `{P}`, so `p_i = (r_p cos phi_i, r_p sin phi_i, -h_p)` | mm | **quantity settled 2026-09-03**; *symbol* still open, see §11 | new |
| `mu` | a rotation of the whole platform ring inside `{P}` | deg | **not a parameter.** Gauge under `q_i = T + R p_i`, and leg-set D3 pins it. Recorded so it is not reintroduced. | new |

**Plate offset — settled 2026-09-03.** The origin of `{P}` sits on the **plate
top**, or one ball radius above it if what is being commanded is the ball-centre
plane. The six anchors are **coplanar**, so all six share one `z` in `{P}` and the
offset is a single scalar rather than six. That scalar is plate thickness plus the
joint stack down to the ball-joint centres: a **hardware number that belongs in the
model, not a sweep axis** — it is measured off the built plate, not searched over.

It **cannot be absorbed into `T`**. Writing `p_i = p_i^flat - h_p * z_hat`,

```
q_i  =  T  +  R p_i^flat  -  h_p (R z_hat)
```

and `R z_hat = z_hat` only when `R` fixes the vertical — i.e. under pure yaw. Under
any tilt the offset term swings with the plate, so folding `h_p` into `T` is exact
at yaw and wrong everywhere else.

Only the *quantity* is settled. The *symbol* `h_p` is still contested — see §11.

## 5. Link lengths

| Symbol | Meaning | Units | Status | Provenance |
|---|---|---|---|---|
| `a` | servo arm length | mm | design variable, restricted to horns you can buy | key |
| `d` | push-rod length | mm | design variable; *measure after cutting*, do not assume nominal | key |

## 6. Per-leg intermediates

| Symbol | Meaning | Frame / units | Provenance |
|---|---|---|---|
| `q_i` | world position of platform anchor `i`, `T + R p_i` | `{W}`, mm | key |
| `L_i` | leg vector, shaft to anchor, `q_i - b_i` | `{W}`, mm | key |
| `h_i` | position of servo `i`'s arm tip, `b_i + a(u_i cos alpha_i + v_i sin alpha_i)` | `{W}`, mm | key |
| `w_i` | `L_i . n_i`. Signed distance of the anchor off servo `i`'s plane. Only the horizontal part of `L_i` contributes, since `n_i` is horizontal. | mm | body |
| `rho_i` | `sqrt(d^2 - w_i^2)`. The rod's length once projected into the plane — what is left of `d` after clearing it. `rho_i <= d`, equal only at `w_i = 0`. | mm | new |
| `M_i` | `L_i . u_i` | mm | key/body |
| `N_i` | `L_i . v_i`. Equals the anchor's height above the base plate, because `b_i . z = 0`. | mm | key/body |
| `C_i` | `sqrt(M_i^2 + N_i^2) = sqrt(\|L_i\|^2 - w_i^2)`. In-plane magnitude of the **leg**. Distinct from `rho_i`, which is the in-plane magnitude of the **rod**; equal only if `\|L_i\| = d`. | mm | key/body |
| `P_i` | `(\|L_i\|^2 + a^2 - d^2) / (2a)`. Contains no `delta`. Equals `a` exactly under `a^2 + d^2 = \|L_home\|^2`. | mm | key/body |
| *(rod vector)* | `q_i - h_i`, magnitude `d` | `{W}`, mm | **unnamed** — see §11 |
| *(arm tangent)* | unit vector along `dh_i/dalpha_i`; `-u_i sin alpha + v_i cos alpha` | `{W}` | **unnamed** |

## 7. The unknown

| Symbol | Meaning | Units | Provenance |
|---|---|---|---|
| `alpha_i` | servo `i`'s angle, measured from `u_i` toward `v_i` | deg | key |

## 8. Analysis quantities

Not part of the mechanism. Used for tuning `delta` and for scoring.

| Symbol | Meaning | Units | Provenance |
|---|---|---|---|
| `z_home` | platform height above the base plane at the home pose | mm | new |
| `A_i`, `B_i` | coefficients in `w_i(delta) = A_i cos delta + B_i sin delta`; both independent of `delta`, which is what makes the `delta` scan cheap | mm | new |
| *(amplitude)* | `sqrt(A_i^2 + B_i^2)`, so `w_i(delta) = amplitude * cos(delta - phase)` | mm | **unnamed** — collides with `R` |
| *(phase)* | `atan2(B_i, A_i)` | deg | **unnamed** |
| `J(delta)` | worst `\|w\|` over the envelope and all six legs. **Superseded** by the reach margin below; retained only for reasoning about `delta` before `a` and `d` exist. | mm | new |
| *(reach margin)* | `C_i - \|P_i\|`. Positive means leg `i` solves; this is the quantity to maximise. Negative means the candidate is infeasible, which `J` cannot detect. | mm | **unnamed** |
| `tau_i` | transmission ratio, `\|rod . tangent\| / d`. Zero at a loss-of-authority configuration. | — | new |
| `k` | uniform length scale factor. `alpha_i` is exactly invariant under scaling every length by `k`, envelope included. | — | new |

## 9. Working envelope

Not yet specified. These are the slots.

| Symbol | Meaning | Units | Scales with `k`? |
|---|---|---|---|
| `dxy` | translation half-range in `x` and `y` | mm | **yes** |
| `dz` | translation half-range in `z` | mm | **yes** |
| *(tilt limit)* | roll and pitch half-range | deg | no |
| *(yaw limit)* | yaw half-range | deg | no |
| *(grid)* | samples per axis. Currently 3, giving `3^6 = 729` poses and `729 x 6 = 4374` `w` evaluations per `J(delta)`. | — | — |

Because translations carry length dimension and angles do not, the envelope must
be scaled along with the geometry or scale invariance fails. If the task fixes
the required travel in absolute mm, the natural unit for normalising the whole
sweep is that task length, not `r_b`.

## 10. Symmetry objects

| Symbol | Meaning |
|---|---|
| `C3` | rotation by 120 degrees about `z`. Leg permutation `[2, 3, 4, 5, 0, 1]`. |
| `m0`, `m60`, `m120` | the three mirror planes, at azimuths 0, 60, 120 **mod 180**. `m0` leg permutation `[1, 0, 5, 4, 3, 2]`. |
| `D3` | the group they generate, order 6. The requirement is on the **legs** — one permutation carrying shafts to shafts *and* anchors to anchors — not on either point set alone. The two are not equivalent. |
| `sigma` | the leg permutation induced by a group element. |

Under a mirror, `w` picks up a sign: `w_sigma(i) = -w_i`, because `n` flips
handedness while `L` does not. Under `C3` there is no flip. This gives a free
test on any `w` implementation:

| pose | expected `\|w\|` pattern |
|---|---|
| home, or pure heave | all six equal, signs alternating with `s_i` |
| pure yaw | two values, one per `C3` orbit |
| tilt about an axis perpendicular to a mirror plane, `T` in that plane | three equal-magnitude pairs, opposite signs |
| anything else | six distinct values |

## 11. Collisions and unresolved names

Every one of these is live. None should be resolved silently.

| Clash | Detail | Suggested resolution |
|---|---|---|
| `h` | `h_i` is the arm tip. The plate offset in §4 was written `h_p` during working, which reads as a seventh arm tip. | rename the plate offset, e.g. `c_p` |
| `R` | `R` is the platform orientation matrix; the sinusoid amplitude in §8 also wants `R`. | give the amplitude another letter |
| rod vector | proposed `r_i`, which collides with `r_b` and `r_p` | needs a third option |
| `P` | `P_i` is a scalar; `{P}` is the platform frame. Pre-existing. | leave, but never write `P` unsubscripted |
| `C` | `C_i` is a length; `C3` is a rotation | leave; the subscript disambiguates |
| `s` | `s_i` is the within-pair sign; screw direction also conventionally `s` | rename the screw direction |
| `p` | `p_i` is an anchor; `pitch` abbreviates to `p` | spell pitch out |
| `t` | arm tangent wants `t`; plate thickness also wants `t` | pick one |
| `a` / `A_i` | arm length vs sinusoid coefficient, distinguished only by case | acceptable, but flag in code |
| `w` | out-of-plane component. Must **not** be reused for the rod vector, which it was in scratch code. | reserved for `L_i . n_i` |
| indexing | 0-based in code, 1-based in prose and error messages | record which, per document |

**On `h` / `h_p`.** The clash above is **still open** — 2026-09-03 settled the
*quantity* (§4), not the symbol. The quantity is now in code as `h_p`, so a rename
is a pending decision with real call sites, not a free edit:
`stewart/geometry.py` — `platform_ring` (parameter, docstring, body, assert) and
`make_geometry` (parameter, docstring, pass-through) — and `test_kinematics.py`
(fixture key). `stewart/kinematics.py` has **no** `h_p` call site; it reads the
offset only through `geom.p`, so it is unaffected by a rename.

## 12. Not yet decided

Listed so nothing in this file is mistaken for a settled value.

- ~~The `z` component of `p_i` in `{P}`, and where `{P}`'s origin sits.~~
  **Settled 2026-09-03 — see §4.** Plate top (or one ball radius above for the
  ball-centre plane), coplanar anchors, one shared `z`. What remains open is only
  the *symbol*, §11.
- The rotation convention behind roll/pitch/yaw (§6).
- `a` and `d`.
- `beta_p`'s range, which needs a **ball-joint housing diameter** — two housings
  cannot occupy one hole.
- `beta`'s usable range, which needs a **servo body diameter** — two servo bodies
  cannot occupy one mounting arc.

**What does *not* constrain those two ranges.** The exclusions above are
**hardware**, not rank degeneracy, on both rings. Recorded because the opposite was
believed earlier and acted on:

- `beta_p -> 0` merges the six anchors onto three points and is the **3-6 Stewart
  platform** — a real architecture, not a degenerate one. Measured 2026-09-03:
  `sigma_min` stays O(1) all the way down, because the shafts stay split and the
  six leg lines remain distinct. (The earlier note here called both endpoints the
  "octahedral 3-3 architecture"; that is wrong unless *both* rings collapse at
  once, and `beta` and `beta_p` are independent.)
- `beta_p = beta` is **not** a rank hole. Verified 2026-09-04 with the true rod
  lines `q_i - h_i`: full rank 6, and `sigma_min` is *monotonic* through
  `e = beta_p - beta = 0` — not even a local minimum. The earlier rank-3 result
  came from the `q_i - b_i` proxy, whose six lines are concurrent on the `z`-axis
  by construction; the arm breaks that concurrency. Recovered `sigma_min` scales
  linearly with `a` (log-log slope 0.992), so it vanishes only as `a -> 0`, which
  is exactly where the proxy becomes exact.
- The working envelope.
- The scoring function, including the characteristic length used to normalise
  the moment rows of any conditioning measure. That choice changes the ranking
  of candidates, so it is a requirement to be stated, not a constant to be
  picked.
