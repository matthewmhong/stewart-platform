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

**Pose argument and return order is `(R, T)` — orientation first, everywhere.**
Settled 2026-09-07 so it stops being a per-function question. It is what
`stage1(geom, R, T)`, `legs(geom, R, T)`, `w(geom, R, T)` and `ik(geom, R, T)`
already took, so `fk(geom, alphas, R0, T0) -> (R, T)` and
`stewart/roundtrip.py`'s `R_hat, T_hat = fk(...)` match rather than depart. The
mnemonic that keeps it straight is the pose equation itself, `q_i = T + R p_i`:
`R` is the operator and `T` is the offset, so `R` binds first in every signature
even though it is written second in the formula. Note the ordering rule is
independent of §2's table, which lists `T` first — that table is a glossary, not
a signature.

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
| `mu` | a rotation of the whole platform ring inside `{P}` | deg | **not a parameter.** Gauge under `q_i = T + R p_i`: send `p -> Rz(mu) p` and `R -> R Rz(-mu)` and every world anchor is identical at every pose, so nothing measurable distinguishes them. Leg-set D3 pins the discrete residue — but see the caveat below. Recorded so it is not reintroduced. | new |

**Plate offset — settled 2026-09-03.** The origin of `{P}` sits on the **plate
top**, or one ball radius above it if what is being commanded is the ball-centre
plane, **because that is the surface the control law reasons about**. The six
anchors are **coplanar**, so all six share one `z` in `{P}` and the offset is a
single scalar rather than six. That scalar is plate thickness plus the joint stack
down to the ball-joint centres: a **hardware number that belongs in the model, not a
sweep axis** — it is measured off the built plate, not searched over.

Putting the origin in the **anchor plane** instead is tempting — it gives
`p_i,z = 0` and tidier algebra — and it is **wrong**: it makes a commanded tilt do
something other than tilt the surface the ball is on. Recorded because the tidier
algebra is what will argue for it again.

It **cannot be absorbed into `T`**. Writing `p_i = p_i^flat - h_p * z_hat`,

```
q_i  =  T  +  R p_i^flat  -  h_p (R z_hat)
```

and `R z_hat = z_hat` only when `R` fixes the vertical — i.e. under pure yaw. Under
any tilt the offset acquires a **horizontal** component, so it enters
`w_i = L_i · n_i` (with `n_i` horizontal), **so it changes the tuned `delta`**.
Folding `h_p` into `T` is therefore exact at yaw and wrong everywhere else. Pure
algebra, no geometry needed.

Sequencing consequence: `delta` cannot be tuned before `h_p` is fixed. `h_p` comes
from components, so component selection has to precede the inner `delta` tune, not
follow it.

Only the *quantity* is settled. The *symbol* `h_p` is still contested — see §11.

**`mu` and the datum for `R = I` — convention adopted 2026-09-03.** Write the
platform ring with **no `mu` term**, and read `R = I` as *the orientation in which
the platform pair centres line up with the base pair centres*. The gauge argument
above kills `mu` as a degree of freedom; this sentence is what gives the remaining
`R = I` a physical meaning rather than leaving it a bare formula. `mu` becomes real
only if something in `{P}` pins the frame — a marked front, a non-axisymmetric
mount, a cable exit — and a circular plate with a ball rolling on it pins nothing.

This interacts with the open rotation convention in derivation §6: settle that and
the datum above stays true under either reading.

*Caveat, still open (handoff 2026-09-03, open item 2).* The **continuous** `mu` is
dead either way — the gauge argument does not depend on the pairing. What is not
closed is the **discrete residue**: `mu ∈ {0, 180}` was derived under the identity
pairing, while the pairing result ranged over all 720 bijections. Each argument
froze what the other varied. This changes only how the result is *stated*, not
whether `mu` is a sweep axis, but it should be stated correctly before it is
written down as final.

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
| `z_home` | platform height above the base plane at the home pose. **Status: swept; lower bracket closed-form, upper per-candidate** — an outer sweep axis, restored 2026-09-04 when the `z_home = z_flat` datum was dropped. *(This supersedes the same-day entry that made it determined and removed it from the sweep.)* Bracketed 2026-09-05: §12. | mm | new |
| `z_flat` | plate height at which the arms lie flat (`alpha_i = 0`) with the rods attached, at `R = I` and no horizontal translation. Closed form and residuals: derivation §8.1. **An assembly datum only** — the 2026-09-04 identification `z_home = z_flat` was made and dropped the same day. | mm | new |
| `A_i`, `B_i` | coefficients in `w_i(delta) = A_i cos delta + B_i sin delta`; both independent of `delta`, which is what makes the `delta` scan cheap | mm | new |
| *(amplitude)* | `sqrt(A_i^2 + B_i^2)`, so `w_i(delta) = amplitude * cos(delta - phase)` | mm | **unnamed** — collides with `R` |
| *(phase)* | `atan2(B_i, A_i)` | deg | **unnamed** |
| `J(delta)` | worst `\|w\|` over the envelope and all six legs. **Superseded** by the normalised reach margin below. Reason corrected 2026-09-04: not that `J` misses infeasibility, but that the two differ in **aggregation**. `delta` is absent from `L_i`, so `P_i` is `delta`-free and only `C_i` moves; at a fixed leg and pose the margin is strictly decreasing in `\|w_i\|`, so maximising it *is* minimising `\|w_i\|` exactly. But `J` is a minimax over `\|w_i\|` while the margin is a maximin over `(C_i-\|P_i\|)/C_i`, and since `\|L_i\|` varies across legs and poses the largest-`\|w_i\|` leg is generally not the smallest-margin leg. Different worst cases, different minimisers. Retained for reasoning about `delta` before `a` and `d` exist. | mm | new |
| *(normalised reach margin)* | `(C_i - \|P_i\|) / C_i`, maximin over legs and envelope poses. Positive means leg `i` solves; negative means the candidate is infeasible. **The division by `C_i` is required**: `C` and `P` both carry length, so the raw difference `C_i - \|P_i\|` scales with `k` and breaks the normalised sweep, where candidates differing only in `r_b` must score identically. This is the form the branch check reports, and where `-5.7e-3` came from. *(Was `C_i - \|P_i\|` here; normalised 2026-09-04.)* | — | **unnamed** |
| `tau_i` | transmission ratio, `\|rod . tangent\| / d`. Zero at a loss-of-authority configuration. | — | new |
| `k` | uniform length scale factor. `alpha_i` is exactly invariant under scaling every length by `k`, envelope included. | — | new |

**The score function — settled 2026-09-07.**

```
score  =  margin(dxy = p)
```

The same normalised reach margin as the row above — maximin `(C_i - |P_i|)/C_i`
over the six legs and the 29-pose envelope grid of §9 — evaluated at a platform
displacement `p` instead of at the nominal pose, with `delta` tuned per candidate
to maximise `margin(dxy = 0)` **subject to `cond(J_fk) <= 1e6` at
`char_len = r_b`**. Feasibility stays pass/fail before scoring, unchanged. **`p`
is not yet fixed** — three probes (`0.005`, `0.0075`, `0.010 r_b`) were carried
side by side and none preferred.

Everything in this block is measured in
`stewart/diagnostics/score_discriminators.py`; the part number is cited per claim.

- **One term, and there is nothing to weight.** `sens(p)` is *defined* as
  `[margin(0) - margin(p)] / p`, so `margin - sens(p)*p` **is** `margin(dxy = p)`
  identically — verified over the whole field and all three probes at **`6.9e-18`**
  (part 7). The "two-term score" was one quantity read at a displaced pose, not two
  competing ones. The plan's requirement of a weighted scalar with stated weights is
  met by a single functional with one stated requirement, `p`. `p` is not a weight;
  it is the build error the margin is read at.
- **`tau_i` is measured-and-redundant.** Spearman `rho = 0.835` against `margin`
  over the feasible set (part b). Not weighted into the score, and not dropped from
  the measurement.
- **`cond` is not a ranking term.** It is the cap on the inner tune — next block.

**The cap on the inner `delta` tune — `cond(J_fk) <= 1e6` at `char_len = r_b`.**
The maximin-margin tune does not merely fail to see a rank degeneracy, it
*selects* it: at `beta_p == beta` and `delta = 0` the anchor is radially in line
with the shaft and the whole rod lies in the servo plane, so at `R = I` all six rod
lines meet the `z`-axis and `J_fk` drops rank at the home pose — and the tuner buys
that configuration for a margin difference in the third decimal place. Measured
(parts 2, 5):

- at `C = 1e6` the cap binds on **exactly** the 34 of 363 candidates with
  `beta_p == beta` — nothing outside that set is touched, and none of the 34 is
  left alone;
- it costs **`4.87e-5`** of margin, worst case over the field;
- the exact coincidence sits **13 decades** above the worst near-miss at
  `e = beta_p - beta != 0`. The cap separates a rank drop from a merely
  ill-conditioned neighbour; it is not cutting into a continuum.

`C` is a `cond` value, so quoting it requires the characteristic length — §12.

**Rule, and it is general.** A quantity that matters and is **not monotone in the
pointwise margin** must enter the **inner tune**, not the outer score, or the tune
will spend it. `delta` is chosen before the score is read, so anything the score
would have penalised is something the tuner is free to trade away for margin
first — and the score then never sees the configuration it would have rejected.
Two instances are on record:

- **`cond`.** As an outer ranking term, or as a reject floor, it arrives too late:
  the tune has already selected the degenerate `delta` (part 5). As a cap on the
  tune it works, at the cost above. *(This supersedes the same-day entry that made
  `cond` a discriminator with a hard floor — §12.)*
- **The tune/score mismatch.** `delta` is tuned on `margin(0)`; the score is
  `margin(p)`. So every candidate is scored at a `delta` chosen for a different
  objective. Real, and **bounded**: worst margin given up **`1.37e-3`**, below the
  ranking grid's own resolution of **`2.8801e-3`**; maximum rank movement **2
  places**, and nothing enters or leaves the top 20 (part 9). Left as it stands, on
  that bound. **Re-check if `p` is ever taken above `0.01 r_b`** — the bound was
  measured at and below it.

**What the score returns is a tie set, not a winner.**

- **The margin collapses on `|e| = |beta_p - beta|`.** Grouping the field by
  `(r_p, a, d, |e|)` and ignoring `beta` and `beta_p` entirely, the tuned margin
  agrees within a group to **`1.33e-15`**, and still does on an azimuth grid 40x
  finer (0.25°). **The collapse is real and narrow** (part 8): only the **tuned
  maximum** collapses. At a **common** `delta` the members' margin fields differ
  pointwise by up to **`7.81e-1`**, against a full-field margin range of
  **`7.93e-1`**, and **0 of 124** groups are equal at a common `delta`. The members
  of a group are not the same machine. **This licenses nothing about the sweep**:
  the six axes stand and `beta` and `beta_p` are sampled independently.
- **Ties below `2.88e-3` are not an ordering.** That is the ranking grid's own
  resolution — what the 10° azimuth grid overstates a margin by against 0.25°
  (§9). Candidates closer together than it are ties the grid cannot separate.
  The top-of-field ties are exact to machine precision and persist under
  refinement; they break on **hardware ranges downstream** — horn set, ball-joint
  housing OD, rod stock — not on anything the sweep computes.

**Open item 12 closes.** Feasibility comes from the closed forms; the ranking runs
on the 10° azimuth grid and is optimistic by a bounded **`2.88e-3`**. That is the
second of the two candidate resolutions, taken with the bound used as a
*resolution* rather than assumed uniform across candidates — the uniformity the
item doubted is not claimed.

**Open, and no candidate explanation is verified.** Why the tuned maximum
collapses to `1.33e-15` when the fields at a common `delta` differ by `7.81e-1`.
Two different machines, with different `margin(delta)` curves, peak at the same
height to the last bit. Recorded as a property of `max over delta` of the maximin
margin rather than of the geometry at any one `delta` — which is a restatement of
where it sits, not a mechanism.

## 9. Working envelope

**Specified 2026-09-05.** This closes handoff open item 8 and open item 1. The
slots below are filled, not proposed.

| Symbol | Value | Units | Basis |
|---|---|---|---|
| `dxy` | **0** | mm | The control law commands **tilt only**. |
| `dz` | **0** | mm | As above. |
| *(tilt limit)* | **10.529** | deg | Recovery envelope; derived below. |
| *(yaw limit)* | **0** | deg | A circular plate is axisymmetric, so yaw does not move the ball. |
| *(grid)* | 5 magnitudes x 7 azimuths = **29 poses**, `29 x 6 = 174` `w` evaluations per objective evaluation | — | See *Grid* below. |

**Where the tilt limit comes from.** Bang-bang recovery, not arrest. To return
the ball from a displacement `x0` in a time `tau`, accelerate for `tau/2` and
decelerate for `tau/2`; the distance covered is `acc * tau^2 / 4`, so

```
acc  =  4 x0 / tau^2
sin(tilt)  =  7 acc / (5 g)          solid ball rolling without slip
```

With `tau = 0.5 s` and `g = 9.80665 m/s^2`:

| | `x0` | `acc` | tilt |
|---|---|---|---|
| **bare requirement** | 50 mm working displacement | 0.800 m/s² | **6.558°** |
| **envelope** | 50 mm + 30 mm latency drift = 80 mm | 1.280 m/s² | **10.529°** |

Both are recorded on purpose. **6.6° is the requirement; 10.5° is the envelope;
the difference is the latency margin.** Everything is evaluated on 10.529°.

**`tau_L = 150 ms` is PROVISIONAL.** The 30 mm of latency drift is
`tau_L * v_peak = 150 ms * 200 mm/s`, and `tau_L` is inherited from the 300 mm/s
figure **withdrawn 2026-09-03 as invented**. It has no basis of its own and it
needs one: **sensor frame interval plus servo step response**, both on the
hardware pull. It carries 30 of the 80 mm and 3.97 of the 10.53 degrees. Do not
promote it silently.

**Sensitivity — this travels with the number wherever it is quoted.** Tilt goes
as `1/tau^2`, so a 10% error in `tau` moves the required `sin(tilt)` by ~20%.
Measured:

| `tau` [s] | bare [deg] | envelope [deg] |
|---|---|---|
| 0.40 | 10.280 | 16.590 |
| 0.45 | 8.106 | 13.038 |
| **0.50** | **6.558** | **10.529** |
| 0.60 | 4.549 | 7.290 |
| 0.75 | 2.910 | 4.658 |

`tau` is the least-defended number in the envelope and the envelope is most
sensitive to it. Both facts belong together.

**The envelope is two axes, and it does not scale with `k`.** With
`dxy = dz = yaw = 0` the only free parameters are **tilt magnitude** and **tilt
azimuth**. The four-axis envelope recorded on 2026-09-04 (`x`, `y`, magnitude,
azimuth) is superseded: `x` and `y` are gone. Being purely angular, the envelope
carries **no length dimension at all**, so it is invariant under scaling every
length by `k` and needs no scaling alongside the geometry. *(The note that used
to stand here — that translations carry length dimension so the envelope must be
scaled with the geometry or scale invariance fails — is deleted, not softened.
It described an envelope with translations in it. This one has none. Absolute
scale still enters the sweep upstream, but through the tilt target's dependence
on plate size, not through the envelope's units — see the handoff.)*

**Grid.** Tilt azimuth is swept over a **60° window, `[30°, 90°]`**, not the full
circle. The leg set has D₃ symmetry, so the leg aggregates are periodic in 120°
and mirror-symmetric; **verified numerically**, not assumed — see
`stewart/diagnostics/azimuth_symmetry.py` and §10 below for where the mirror
lines actually sit. 5 magnitudes over `[0, 10.529°]` and 7 azimuths across the
window, with magnitude 0 counted **once** because at zero tilt every azimuth is
the same pose: `1 + 4 x 7 = 29` poses, `174` `w` evaluations per objective
evaluation. The full-circle equivalent at the same resolution would be 169 poses
and 1014 evaluations.

**This grid is optimistic on worst-case margin, by about `1.9e-3`.** Measured at
one fixture, varying only the azimuth sampling: 15° gives `+1.534192e-01`, this
grid's 10° gives `+1.550398e-01`, and a 0.25° reference gives `+1.531859e-01` —
7 samples at 10° miss the worst azimuth by more than 24 at 15° happen to. It is
the same failure mode as the `z_home` lower bound in §12: a discrete pose grid
flatters a worst case, in the unsafe direction. Fine for **ranking**, which is
what a scoring grid is for; **not** a source for a quoted worst-case number, and
not a source for a **feasibility** decision — those come from the closed forms.
See handoff open item 12.

*(This replaces the stale row "samples per axis, currently 3, giving `3^6 = 729`
poses and `4374` `w` evaluations". That row counted six envelope axes; there are
two. The real numbers are 29 and 174.)*

## 10. Symmetry objects

| Symbol | Meaning |
|---|---|
| `C3` | rotation by 120 degrees about `z`. Leg permutation `[2, 3, 4, 5, 0, 1]`. |
| `m0`, `m60`, `m120` | the three mirror planes, at azimuths 0, 60, 120 **mod 180**. `m0` leg permutation `[1, 0, 5, 4, 3, 2]`. |
| `D3` | the group they generate, order 6. The requirement is on the **legs** — one permutation carrying shafts to shafts *and* anchors to anchors — not on either point set alone. The two are not equivalent. |
| `sigma` | the leg permutation induced by a group element. |

**Where the mirrors land in tilt *azimuth* — measured 2026-09-05.** The mirror
*planes* sit at azimuths 0, 60, 120. The azimuths of the *tilt axes* they fix do
**not**: they sit at **30 + 60k**. A reflection `M` in the vertical plane at
azimuth `m` has `det M = -1`, and `M R(n, th) M^T = R(det(M) M n, th)`, so an
axis at azimuth `psi` maps to one at `2m + 180 - psi`. With `m = 0, 60, 120` the
fixed azimuths are 90, 150 and 30. Physically: tilting about an axis lying **in**
a mirror plane reflects to the **opposite** tilt; it is the axis **perpendicular**
to the plane that is self-symmetric — which is what the `\|w\|` table below
already says in its third row.

**Consequence, and it is easy to get wrong.** The leg aggregates are periodic in
120° and mirror-symmetric, so a **60° azimuth window suffices** — but it is
`[30°, 90°]`, **not** `[0°, 60°]`. `[0°, 60°]` is symmetric about its own centre,
so it covers the orbits it touches twice and misses others entirely: the orbit
`{75°, 105°}` mod 120 meets it nowhere. Verified numerically over the full circle
with a negative control: `stewart/diagnostics/azimuth_symmetry.py`. Period-120
and the `psi -> 180 - psi` mirror hold to `1e-13` relative; the `psi -> -psi`
mirror that `[0°, 60°]` would need **fails at `2.3e-1`**.

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
- ~~**`z_home`'s range**~~ *(reopened 2026-09-04; bracketed 2026-09-05 — this
  supersedes the entry that said nothing supplied a range)*. Both ends are now
  computed, from `stewart/diagnostics/zhome_bracket.py`.

  **Lower, closed form, from `N_i > 0`:**

  ```
  z_home  >  r_p sin(tilt) + h_p cos(tilt)      = 0.182733 r_p + 0.983163 h_p
  ```

  at `tilt = 10.529°`. It is **`delta`-free, `a`-free and `d`-free**: `v_i = z`
  exactly under horizontal shafts and `b_i . z = 0`, so `N_i = q_i . z` and the
  bound involves only `r_p`, `h_p` and the tilt limit. Verified against
  `make_geometry`: `min N_i` at the bound is `0` to `5.6e-17`.

  Two things about it. It is a **continuum** bound — a discrete pose grid reports
  the constraint satisfied slightly *before* it truly is (measured `+5.9e-4` on the
  29-pose harness grid at `beta_p = 25°`), so the harness must take this bound from
  the formula, **not** from its own grid. And the back-of-envelope
  `z_home - h_p > r_p sin(tilt)` is **not** this bound: it drops the `cos(tilt)`,
  overstating the requirement by `h_p(1 - cos tilt) = 1.68e-3 r_b` at
  `h_p = 0.1 r_b`. Conservative, so it errs safe, but it is not the bound.

  **Upper, per candidate, from reach `|P_i| <= C_i`.** No closed form. It depends
  on `a`, `d`, `r_p`, `beta`, `beta_p` and on `delta` being free to tune, so it is
  **not a single number** and cannot be written as one. Over a 540-candidate grid
  the feasible intervals ran `[0.225, 1.650]` at the low end and `[0.425, 1.875]`
  at the high end, all contiguous, widest `0.825 r_b` and narrowest below the
  `0.025 r_b` scan resolution.

  **`N_i > 0` never shapes the interior, and is load-bearing at the edges.**
  *(Corrected 2026-09-05. The sentence first written here — "the lower bound was
  never the binding one … at this tilt it is not what shapes the axis", from
  `0 of 363` — rested on a **survivorship sample**: the 363 are the candidates
  where the constraints did **not** cross, so a counterexample could not have
  appeared among them.)*

  In 0 of the 363 candidates with a non-empty bracket does `N_i > 0` set the lower
  end — that part stands. Attributing the **177 empties** as well:

  - **176 of 177** — reach fails on its own, at every `z_home` and every `delta`.
  - **1 of 177** — the constraints cross: reach ceiling **below** the `N_i` floor,
    at `beta = 10°, beta_p = 55°, r_p/r_b = 1.10, a/r_b = 0.10, d/r_b = 0.80`.
    Ceiling `0.2920624` (bisected to `1e-9`, because the grid gap of `0.024` sat
    inside one `0.025` step), floor `0.29932`, gap **`+7.26e-03`**. Real.
  - **6 candidates** have a non-contiguous *reach* set — a spurious low component at
    `z_home ≈ 0.025–0.125 r_b`, the platform on the base plate — and in **6 of 6**
    it lies entirely below the `N_i` floor. `N_i > 0` is what removes it and keeps
    the feasible set an interval. (This is why "0 non-contiguous feasible sets" and
    "6 non-contiguous reach sets" are both true: different sets.)

  The categories are fixed by the constraints' shapes — `N_i > 0` is **one-sided**,
  a floor; reach is a **two-sided interval**. There is no `N_i` ceiling, so a reach
  floor cannot sit above one.

  `z_flat(delta)` (§8, derivation §8.1) remains a useful reference point inside the
  bracket even though home no longer sits on it.
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
- ~~The scoring function.~~ **Settled 2026-09-07 — see §8.**
  `score = margin(dxy = p)`: one term, no weights, `delta` tuned under a
  `cond(J_fk) <= 1e6` cap at `char_len = r_b`. What remains open under it is
  **`p`**, the build error the margin is read at — three probes were carried side
  by side and none preferred.
- The characteristic length used to normalise the moment rows of any conditioning
  measure. That choice changes the ranking of candidates, so it is a requirement
  to be stated, not a constant to be picked. **Still open — but narrowed the same
  day, and it no longer blocks the score. Read the block below with the note that
  closes it.**

  **The FK gate turned this from a tidiness item into a blocking one
  (2026-09-07).** `cond(J)` now has a job in the score, and the characteristic
  length is what sets the threshold, so the number cannot stay provisional.

  The finding, from `docs/cc-fk-gate.md` §6. A 6-RSS forward kinematics has
  several real solutions, and on the four gate fixtures the alternative
  assembly modes are far away — 30 to 260 mm, and every one either below the
  base plate or tilted 50–86°, so none is reachable or confusable. Those
  fixtures sit at `cond(J)` **4–10**. On `smoke_geometry`, at `cond(J)` **9045**
  and `sigma_min` **1.94e-4**, a second mode sits **0.32 mm and 0.73°** from the
  commanded pose, with `ik` returning **the same six angles to 2.0e-13 deg** at
  the recovered pose.

  That last clause is the whole point, and it is not an FK problem. Two poses
  0.32 mm apart that produce **identical servo commands** are two poses the
  machine cannot distinguish from its own commands — no control law, no
  calibration and no better solver separates them, because the information is
  not in the command. Which of the two the platform assembles into is decided
  by history and by which side of the fold it was on, not by the command. So
  near-singular geometry is where assembly modes coalesce, **and nothing in the
  current feasibility set excludes it**: `|P_i| <= C_i` and `N_i > 0` are both
  satisfied throughout, so such a candidate would pass feasibility and reach
  the ranking stage.

  Consequences for the score, none of them yet decided:

  - `cond(J)` (or `sigma_min`) becomes a **discriminator with a floor** —
    a hard reject below some threshold, not merely a term to be traded off —
    alongside the reach margin `(C - |P|)/C` and the translation sensitivity
    already recorded. A weighted sum would let a candidate buy its way past a
    fold with margin elsewhere.
  - The threshold is a `cond` value, and `cond` is not defined until the
    characteristic length is. **The length now decides where a reject line
    sits**, not just how a ranking sorts, which is why it can no longer be
    carried as provisional.
  - The gate quotes everything at `char_len = r_b` and flags it PROVISIONAL.
    Measured spread over four candidate lengths on the gate fixtures: `r_b`
    3.7–7.4, `r_p` 3.1–6.4, `d` 4.1–6.7, `a` 3.7–19.5 — a factor of 3 on one
    fixture, so the choice is not cosmetic even at these benign values.
  - Open, and not answered by any of the above: whether the right quantity is
    `cond(J)` of the FK Jacobian at all, or the wrench matrix's `sigma_min`,
    or the mode separation measured directly. The FK Jacobian is what the gate
    happened to have; that is a reason to look, not a reason to adopt it.

  `smoke_geometry` is explicitly not a design, so this is a located failure
  mode rather than a live one. It is recorded here because the feasibility set
  does not currently exclude it and the sweep has not yet run.

  **What the length is still needed for — 2026-09-07, later the same day.** Of
  the three consequences above, one is superseded and the rest narrow. Evidence:
  `stewart/diagnostics/score_discriminators.py`, part numbers cited.

  - **Superseded: there is no reject line, and `cond` is not a discriminator with
    a floor.** It is a **cap on the inner `delta` tune** — §8 carries the value,
    what it catches and what it costs. A floor on the outer score arrives too
    late: the tune selects the degenerate `delta` before any outer test reads the
    candidate (part 5). So the characteristic length no longer decides where a
    reject line sits, and **it no longer blocks the score**, which is settled
    above without it.
  - **Still open, for two narrower reasons.** *(1) Quoting `C`.* The cap is a
    `cond` value, so `C = 1e6` says nothing without the length it is evaluated
    at; it is quoted at `char_len = r_b` throughout and is PROVISIONAL in that
    length. Its *effect* is measured: at `r_b` it binds on exactly the 34
    `beta_p == beta` candidates and sits 13 decades clear of the worst near-miss
    (parts 2, 5), so it is not balanced on a boundary — though that is a reading
    of the isolation, not a re-measurement at the other three lengths.
    *(2) The FK residual-to-pose bound.* The residual is in mm and the pose has a
    rotation part, so converting one into the other needs a length. That is
    independent of scoring and is where the question now lives.
  - **Unchanged and still open:** whether the right quantity is `cond(J_fk)` at
    all, rather than the wrench matrix's `sigma_min` or mode separation measured
    directly. `J_fk` and `J_cmd` were carried side by side throughout
    `score_discriminators.py` and never merged; its part (c) reports whether the
    two even order the field differently. Nothing in the cap turns on this — the
    cap is calibrated on what it catches, not on the measure being the right one.
