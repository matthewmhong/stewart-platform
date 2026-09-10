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
| `r_b` | base ring radius | mm | **fixed at 90 mm — ASSERTED 2026-09-08**, the full print bed; not swept. See §12. | key |
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
| `r_p` | platform ring radius | mm | **fixed at 80 mm — ASSERTED 2026-09-08**, from the printed hub carrying the bought 220 mm sheet; not swept. See §12. | new |
| `beta_p` | pair half-split | deg | swept; range bounded by ball-joint housing OD, **published 9.0–13.0 mm, both ends on record — §12**; no joint chosen | new |
| `phi_i` | anchor azimuth, `120*floor(i/2) + s_i*beta_p` — same skeleton as `theta_i`, which is what makes leg `i` pair with leg `i` | deg | derived | new |
| `p_i` | platform anchor position | mm, `{P}` | design constant | key |
| `c_p` | plate offset: the common `z` component of `p_i` in `{P}`, so `p_i = (r_p cos phi_i, r_p sin phi_i, -c_p)` | mm | **quantity settled 2026-09-03; symbol resolved 2026-09-10** — was `h_p`, see §11 | new |
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

It **cannot be absorbed into `T`**. Writing `p_i = p_i^flat - c_p * z_hat`,

```
q_i  =  T  +  R p_i^flat  -  c_p (R z_hat)
```

and `R z_hat = z_hat` only when `R` fixes the vertical — i.e. under pure yaw. Under
any tilt the offset acquires a **horizontal** component, so it enters
`w_i = L_i · n_i` (with `n_i` horizontal), **so it changes the tuned `delta`**.
Folding `c_p` into `T` is therefore exact at yaw and wrong everywhere else. Pure
algebra, no geometry needed.

Sequencing consequence: `delta` cannot be tuned before `c_p` is fixed. `c_p` comes
from components, so component selection has to precede the inner `delta` tune, not
follow it.

**Symbol resolved 2026-09-10 — see §11.** The quantity was settled 2026-09-03;
the letter was `h_p` throughout that discussion and everywhere else in the repo
until today, which read as a seventh arm tip (`h_i`). Renamed to `c_p`, call
sites included; §11's clash entry is marked resolved, not deleted.

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

**Assessed 2026-09-10: cannot be honestly restated yet — flagged, not guessed.**
The pairing result (`docs/session-handoff-2026-09-04.md`, open item 10) found 720
bijections reducing to 6 survivors in one D3 orbit, with two readings on record
and **neither tested**: that the rotations among the six are the same machine
relabelled, and that the reflections give the mirror-image machine. Restating the
`mu ∈ {0, 180}` result correctly requires knowing whether it holds unchanged
across all 6 survivors (if the "same machine, relabelled" reading is right) or
only across the 3 related by rotation, with reflection flipping or altering the
residue (if reflections are genuinely the mirror machine and mu behaves
differently under one). Neither is known. The test named in the handoff — leg
1's angular span `gamma_tau(1) - theta_1` compared as a multiset across the six
survivors at `beta = 20, beta_p = 40` — has not been run. Until it is, any single
sentence combining the two results would be asserting one of two live
possibilities as though it were settled, which is not stating it correctly; it is
guessing. Left flagged, per the pairing item's own standing status.

## 5. Link lengths

| Symbol | Meaning | Units | Status | Provenance |
|---|---|---|---|---|
| `a` | servo arm length | mm | design variable, restricted to horns you can buy — **now a discrete published set, §12** | key |
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
| `rod_i` | `q_i - h_i`, magnitude `d` | `{W}`, mm | **named 2026-09-10** — was unnamed, see §11. Adopted from the provisional local name already in use in `score_discriminators.py` and `tilt_authority.py`. | key/body |
| `tangent_i` | unit vector along `dh_i/dalpha_i`; `-u_i sin alpha + v_i cos alpha` | `{W}` | **named 2026-09-10** — was unnamed, see §11. Same provenance as `rod_i`. | key/body |

## 7. The unknown

| Symbol | Meaning | Units | Provenance |
|---|---|---|---|
| `alpha_i` | servo `i`'s angle, measured from `u_i` toward `v_i` | deg | key |

## 8. Analysis quantities

Not part of the mechanism. Used for tuning `delta` and for scoring.

| Symbol | Meaning | Units | Provenance |
|---|---|---|---|
| `z_home` | platform height above the base plane at the home pose. **Status: swept; lower bracket closed-form, upper per-candidate** — an outer sweep axis, restored 2026-09-04 when the `z_home = z_flat` datum was dropped. *(This supersedes the same-day entry that made it determined and removed it from the sweep.)* Bracketed 2026-09-05: §12. | mm | new |
| `z_flat` | plate height at which the arms lie flat (`alpha_i = 0`) with the rods attached, at `R = I` and no horizontal translation. Closed form and residuals: derivation §9. **An assembly datum only** — the 2026-09-04 identification `z_home = z_flat` was made and dropped the same day. | mm | new |
| `A_i`, `B_i` | coefficients in `w_i(delta) = A_i cos delta + B_i sin delta`; both independent of `delta`, which is what makes the `delta` scan cheap | mm | new |
| *(amplitude)* | `sqrt(A_i^2 + B_i^2)`, so `w_i(delta) = amplitude * cos(delta - phase)` | mm | **unnamed** — collides with `R` |
| *(phase)* | `atan2(B_i, A_i)` | deg | **unnamed** |
| `J(delta)` | worst `\|w\|` over the envelope and all six legs. **Superseded** by the normalised reach margin below. Reason corrected 2026-09-04: not that `J` misses infeasibility, but that the two differ in **aggregation**. `delta` is absent from `L_i`, so `P_i` is `delta`-free and only `C_i` moves; at a fixed leg and pose the margin is strictly decreasing in `\|w_i\|`, so maximising it *is* minimising `\|w_i\|` exactly. But `J` is a minimax over `\|w_i\|` while the margin is a maximin over `(C_i-\|P_i\|)/C_i`, and since `\|L_i\|` varies across legs and poses the largest-`\|w_i\|` leg is generally not the smallest-margin leg. Different worst cases, different minimisers. Retained for reasoning about `delta` before `a` and `d` exist. | mm | new |
| *(normalised reach margin)* | `(C_i - \|P_i\|) / C_i`, maximin over legs and envelope poses. Positive means leg `i` solves; negative means the candidate is infeasible. **The division by `C_i` is required**: `C` and `P` both carry length, so the raw difference `C_i - \|P_i\|` scales with `k` and breaks the normalised sweep, where candidates differing only in `r_b` must score identically. This is the form the branch check reports, and where `-5.7e-3` came from. *(Was `C_i - \|P_i\|` here; normalised 2026-09-04.)* | — | **unnamed** |
| `tau_i` | transmission ratio, `\|rod_i . tangent_i\| / d`. Zero at a loss-of-authority configuration. | — | new |
| `k` | uniform length scale factor. `alpha_i` is exactly invariant under scaling every length by `k`, envelope included. **The justification for normalising the sweep by it has changed, not the invariance itself — 2026-09-08/09.** It was "absolute scale is unknown"; with `r_b` and `r_p` now fixed (§12) it is "ratios are the natural parameterisation." `k`-invariance is still true and is now **descriptive of the geometry rather than load-bearing for the sweep design**. | — | new |

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
  the axes stand and `beta` and `beta_p` are sampled independently. *(Was "the
  six axes" when this was measured 2026-09-07; the sweep is **five** axes as of
  2026-09-08/09 — `r_p/r_b` fixed, §12. The finding does not change: it was
  never about the count.)*

  **SUPERSEDED 2026-09-10, not deleted — the key is not complete.** Measured
  on the `2fee8b0` sweep at `beta` and `beta_p` sampled at `2.5°` — finer than
  the `10°`/`15°` this bullet was established at — `(a, d, |e|)` (`r_p` now
  fixed, dropped from the tuple) produces `48,930` groups with worst
  within-group spread `3.74e-2` and `10,338` groups exceeding `TIE_TOL`
  measured at `dxy = p` (`1.5e-4`, this section below). The MIRROR half of
  what this bullet found is exact — `(beta, beta_p) -> (60-beta, 60-beta_p)`
  reduces to `164,181` groups with worst spread `1.62e-4`, one group over
  `TIE_TOL` — but `|e|` alone is not a complete invariant: the absolute
  `beta` enters the tuned score, not only its difference from `beta_p`. The
  `10°`/`15°` sampling this bullet ran at was too coarse to see that. See the
  9 September entry, `docs/phase-0-design-log.md`.
- **Ties below `2.88e-3` are not an ordering.** That is the ranking grid's own
  resolution — what the 10° azimuth grid overstates a margin by against 0.25°
  (§9). Candidates closer together than it are ties the grid cannot separate.
  The top-of-field ties are exact to machine precision and persist under
  refinement; they break on **hardware ranges downstream** — horn set, ball-joint
  housing OD, rod stock — not on anything the sweep computes.
- **A second, coarser resolution applies once `dxy = p`, and `TIE_TOL` is not
  it — 2026-09-09.** `2.88e-3` above is the *tilt-azimuth* grid's resolution
  (10° against 0.25°). Scoring at `dxy = p` sweeps a second, independent
  azimuth — the *displacement* direction, 24 samples around the circle in
  `probe_margin` — and refining it 24 -> 72 -> 360 shows the ranking resolved
  to only **`~1.5e-4`**, eight decades above the code constant
  `TIE_TOL = 1e-12` the `|e|` collapse above was verified against. `TIE_TOL` is
  **correct for the `dxy = 0` collapse it was set from** — the group agreement
  really is exact to `~1e-15` there — and **wrong by eight decades as a tie
  threshold for the score now in force**, which is read at `dxy = p`.
  Measured, not changed: no constant is edited here, and this is a harness
  requirement to carry forward, not a code fix.
  (`stewart/diagnostics/sweep_ranges.py` part 7, `2af4f3a`.)

**Breaking a tie on hardware is legitimate. An optimum located by a hardware
limit is not. — stated 2026-09-08.** The two look alike and are opposites, so the
distinction is recorded rather than left to be inferred:

- **Breaking a TIE on hardware ranges is legitimate.** The score has *genuinely
  said all it can say* — the candidates are equal to machine precision, or closer
  together than the grid's own `2.88e-3` resolution — and something has to
  choose. Handing that choice to the horn set, the housing OD or the rod stock
  takes nothing away from the analysis, because the analysis has no preference
  left to express.
- **An OPTIMUM located by a hardware limit is the failure.** There the score has
  *not* run out of preference — it is still descending when it hits the wall —
  and the wall is what stops it. Then **the hardware chose the geometry rather
  than the analysis**, which is the 19 August rule inverted.

**Both statements stand.** The paragraph above is the first case; the objective
finding of 2026-09-08 — `r_p` running to the smallest value probed, margin
climbing monotonically, no turn — is the second. See the **8 September entry in
`docs/phase-0-design-log.md`**. Neither is softened by the other: the sweep may
end in a tie set broken by what you can buy, and must not end at an optimum set
by what you can buy.

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

**`p` is fixed — 2026-09-08. ASSERTED, not measured.**

```
p_mm  =  sqrt(0.2^2 + 0.2^2 + 0.2^2)  =  0.3464 mm
p     =  0.3464 / 80                  =  0.004330
```

normalised by an **asserted `r_b` floor of 80 mm** (§12). This supersedes the
"**`p` is not yet fixed**" sentence in the score block above, and the same
statement in §12.

It is a **decision taken in discussion. No diagnostic stands behind it**, and
none is cited. The three probes that were carried side by side were `0.005`,
`0.0075` and `0.010 r_b`; **`0.004330` is below all three**, so the score has not
been evaluated at the value now in force. Recorded as asserted so it is not later
read back as measured.

**The three sources, each 0.2 mm:**

| source | status |
|---|---|
| printer tolerance | the one figure with a source |
| ball-joint free play | **placeholder at the printer's figure**, pending the hardware pull |
| platform centring | **placeholder at the printer's figure**, pending the hardware pull |

**Combined RSS, on an independence assumption.** The worst-case sum of `0.6 mm`
is **rejected as unphysical for three independent sources** — it requires all
three to err in the same direction at once.

Two properties that travel with the number:

- **`p` scales inversely with the `r_b` floor.** 80 mm is a *floor*, so raising
  it lowers `p` and lowering it raises `p`: **raising the floor is safe, lowering
  it is not.** The floor is itself asserted — §12.
- **Rod cut tolerance is deliberately excluded.** It perturbs `d`, not the
  platform pose, so it does not belong in a platform displacement. It is checked
  **separately, after `r_b` is chosen**, alongside the already-owed
  `min(C_i - |P_i|)` millimetre check.

**Consequence.** `p = 0.004330 < 0.010 r_b`, so part (9)'s tune/score mismatch
bound of **`1.37e-3`** carries and **needs no re-measurement** — that bound was
measured at and below `0.01 r_b`, and the re-check clause above is not triggered.

**`p`'s denominator corrected — 2026-09-09. ASSERTED, not measured.**

The block above normalises by "an asserted `r_b` floor of 80 mm." That labelling
is **wrong as of the absolute-scale decision below (§12)**: `r_b` is now fixed
at `90 mm`, not `80 mm`, and `80 mm` is `r_p`. The number the block divides by
was never an `r_b` floor at all — it is the platform-ring radius, read under the
wrong name. Not a new decision and not a re-measurement: the same three 0.2 mm
sources and the same RSS argument, redivided by the length that is actually
`r_b`.

```
p_mm  =  sqrt(0.2^2 + 0.2^2 + 0.2^2)  =  0.3464 mm
p     =  0.3464 / 90                  =  0.003849
```

`0.004330` is **retained above**, unedited, as the record of what was written
2026-09-08 and why; it is **superseded, not deleted**, and it must not be read
back as the value in force. `0.003849` is what the RSS argument means now that
`r_b` has a fixed value.

- **The "raises the floor" property no longer applies.** The struck note above
  — "raising the `r_b` floor is safe, lowering it is not" — was about a *floor*
  on an unresolved `r_b`. `r_b` is fixed, not floored, so there is nothing left
  to raise or lower; the sentence retires along with the mislabel that produced
  it, not carried forward under the new value.
- **The consequence is unchanged in substance.** `p = 0.003849 < 0.010 r_b`,
  still below all three probes carried side by side (`0.005`, `0.0075`,
  `0.010 r_b`), so part (9)'s tune/score mismatch bound of `1.37e-3` still
  carries and still needs no re-measurement — the re-check clause fires on
  `p > 0.01 r_b`, and `0.003849` is further from that line than `0.004330` was,
  not closer.
- **`p` has since been evaluated directly, not just fixed.** `probe_margin` at
  `p = 0.003849` over the fixed-ratio feasible set: five-number margin
  `-0.0062 / 0.2010 / 0.3960 / 0.4618 / 0.6425` over 122 survivors at the
  10.529-deg reference (`fixed_ratio.py`, `76adc0e`); repeated at every tilt
  limit in the 6.558–14.552 deg range (`tilt_dropped.py`, `2af4f3a`) and over
  the hardware-pull `a`/`beta_p` axes (`sweep_ranges.py`, `2af4f3a`). No slope
  is extrapolated off `sens` anywhere in that work; the direct-vs-extrapolated
  difference is measured and reported alongside it every time.

## 9. Working envelope

**Specified 2026-09-05.** This closes handoff open item 8 and open item 1. The
slots below are filled, not proposed. **Amended 2026-09-09**: the tilt limit is
revised by the `tau_L` withdrawal below; the slots that do not depend on it —
`dxy`, `dz`, yaw limit, grid shape — are unchanged.

| Symbol | Value | Units | Basis |
|---|---|---|---|
| `dxy` | **0** | mm | The control law commands **tilt only**. |
| `dz` | **0** | mm | As above. |
| *(tilt limit)* | ~~10.529~~ **6.558** | deg | Recovery envelope; derived below. `tau_L` DROPPED 2026-09-09 — see below. |
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

Both rows are recorded on purpose and **both still stand as arithmetic** —
nothing above changed. What changed 2026-09-09 is which row is *in force*.
**`tau_L` is DROPPED, not revised** (below), so the envelope row's 30 mm of
latency drift has no basis, and **the bare-requirement row is now the limit:
6.558°, not 10.529°.** The envelope row is retained as the historical record of
what every result before 2026-09-09 was evaluated on.

**`tau_L` is DROPPED, not revised — 2026-09-09. ASSERTED, not measured.**
`docs/hardware-pull.md` establishes that servo step response is **unpublished
by every candidate maker** — Hitec, Savox and ROBOTIS each publish
seconds-per-60° no-load speed, which is a **slew rate**, not a step response:
no propagation delay, no rise time to a small commanded step, no overshoot, no
settling criterion. The one third-party dynamic measurement that exists is on
**two servos that are not candidates**, and its author identifies the closest
thing to a delay figure in it (~10 ms at the start of a 15° step) as an
**artefact of the measurement rig's 100 Hz sampling**, not a servo quantity.
Frame intervals *are* published — 4.85–33.33 ms across the cameras checked —
but exposure, sensor readout, transfer and detection are not, so that term has
a **floor and no value**. Neither term of `tau_L = 150 ms` can be reconstructed
from published data. The 30 mm of latency drift it produced, and the 3.97 of
the 10.529 degrees that drift carried, are therefore **removed, not left
provisional** — nothing found supports any particular value, including the
withdrawn one. This **completes** the 2026-09-03 withdrawal of the 300 mm/s
figure `tau_L` inherited from; that withdrawal left `tau_L` provisional and
unable to be closed, and it is now closed by removal.

```
x0         =  50 mm                    working displacement only
tilt limit =  6.558 deg                envelope.tilt_for(0.050), printed, not
                                        hardcoded — see stewart/diagnostics/
                                        tilt_dropped.py
```

**`envelope.py` still reads `TILT_LIMIT_DEG = 10.529`, unedited.** The drop is
recorded here and is not yet a code change — `stewart/diagnostics/envelope.py`
carries the constant every other module reads, and every module downstream of
it still runs at 10.529° until that file is edited. That edit is **pending, not
done**, and this paragraph is not a substitute for it.

**Sensitivity — this travels with the number wherever it is quoted.** Tilt goes
as `1/tau^2`, so a 10% error in `tau` moves the required `sin(tilt)` by ~20%.
Measured, and **unchanged by the `tau_L` drop** — the arithmetic did not move,
only which column is read. The `bare` column is now simply **the tilt limit**;
the `envelope` column is **historical**, the figure everything before
2026-09-09 was evaluated on, kept so the record does not go silent about what
changed:

| `tau` [s] | bare [deg] (now: the limit) | envelope [deg] (historical, withdrawn) |
|---|---|---|
| 0.40 | 10.280 | 16.590 |
| 0.45 | 8.106 | 13.038 |
| **0.50** | **6.558** | 10.529 |
| 0.60 | 4.549 | 7.290 |
| 0.75 | 2.910 | 4.658 |

`tau` is the least-defended number in the envelope and the envelope is most
sensitive to it. Both facts belong together, and neither is touched by the
`tau_L` drop — `tau` is a different number from `tau_L`, unrelated in the
derivation, and the drop removes a term added to `x0`, not the sensitivity of
`sin(tilt)` to `tau`.

**Flagged, not edited: two passages below still read `x0 = 80 mm` /
`10.529 deg`.** *The tilt target is a FIXED ANGLE* block later in this section,
and its `x0 = 50 mm working + 30 mm latency drift = 80 mm` arithmetic, and the
*Bought-part specification* sheet-sizing note that follows it (`~110 mm` radius
from an `80 mm` contact-point reach) — both **rest on the dropped latency term**
and are out of the scope named for this pass (the tilt-limit block, the `tau_L`
PROVISIONAL block, and the sensitivity table only). They are not corrected
here. The scale-invariance ARGUMENT those passages make — that the tilt target
does not scale with `k` because the ball surface is bought, not printed — does
not depend on which `x0` is in force and is unaffected; only the **arithmetic
inside** them is now stale.

**The envelope is two axes, and it does not scale with `k`.** With
`dxy = dz = yaw = 0` the only free parameters are **tilt magnitude** and **tilt
azimuth**. The four-axis envelope recorded on 2026-09-04 (`x`, `y`, magnitude,
azimuth) is superseded: `x` and `y` are gone. Being purely angular, the envelope
carries **no length dimension at all**, so it is invariant under scaling every
length by `k` and needs no scaling alongside the geometry. *(The note that used
to stand here — that translations carry length dimension so the envelope must be
scaled with the geometry or scale invariance fails — is deleted, not softened.
It described an envelope with translations in it. This one has none.)*

*(The clause that stood here until 2026-09-08 — "absolute scale still enters the
sweep upstream, but through the tilt target's dependence on plate size, not
through the envelope's units" — is **withdrawn**. Its second half stands; its
first half rested on a premise that no longer holds. See the block below.)*

**The tilt target is a FIXED ANGLE, not a function of plate size — decided
2026-09-08. ASSERTED, not measured.**

The **ball surface is a bought plastic sheet on a hub, decoupled from the anchor
ring** (§12). So `x0 = 80 mm` is a property of **the sheet**, not of `r_b`, and
nothing in the recovery model varies with the mechanism scale:

```
x0  =  50 mm working  +  30 mm latency drift     property of the sheet
tilt limit  =  10.529 deg     at EVERY mechanism scale
```

**Withdrawn with it: the whole `k`-dependence of the tilt target.** Both earlier
results about it are now **moot rather than wrong** — the quantity they disagreed
about does not exist:

- the 2026-09-04 result that the tilt target goes as `1/k` against translations'
  `k` (arrest framing), and
- the 2026-09-05 inversion of it, that under the recovery framing `x0` scales
  with `k` so required tilt goes as `k` and *grows* with plate size.

Both assumed the ball surface scales with the mechanism. **It does not — it is
bought.** Neither is corrected here, because there is no longer a scaling to get
right.

**Absolute scale therefore enters in exactly two places, and they are both in
§12:**

1. **the bed**, giving `r_b <= 90 mm`; and
2. **`p`'s denominator**, the asserted 80 mm `r_b` floor (§8).

Nothing else in the sweep carries an absolute length. In particular the envelope
does not, per the paragraph above.

**Bought-part specification, not a constraint.** The sheet needs a radius of
about **110 mm**: contact point out to **80 mm**, ball hanging **20 mm** past it,
plus edge margin. Recorded so it is **ordered at the right size** — it is not a
bound on `r_p`, on `r_b`, or on anything the sweep searches over.

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
| `h` | ~~`h_i` is the arm tip. The plate offset in §4 was written `h_p` during working, which reads as a seventh arm tip.~~ **RESOLVED 2026-09-10** — the plate offset is renamed `c_p`, everywhere: §4, code call sites, this file, the design log. Kept here, not deleted, as the record of the clash and why it was named that way in the first place. | ~~rename the plate offset, e.g. `c_p`~~ **done** |
| `R` | `R` is the platform orientation matrix; the sinusoid amplitude in §8 also wants `R`. | give the amplitude another letter |
| rod vector | ~~proposed `r_i`, which collides with `r_b` and `r_p`~~ **RESOLVED 2026-09-10** — named `rod_i`, adopted from the provisional local name already used in `score_discriminators.py` and `tilt_authority.py`. Recorded in §6. | ~~needs a third option~~ **`rod_i`** |
| `P` | `P_i` is a scalar; `{P}` is the platform frame. Pre-existing. | leave, but never write `P` unsubscripted |
| `C` | `C_i` is a length; `C3` is a rotation | leave; the subscript disambiguates |
| `s` | `s_i` is the within-pair sign; screw direction also conventionally `s` | rename the screw direction |
| `p` | `p_i` is an anchor; `pitch` abbreviates to `p` | spell pitch out |
| `t` | ~~arm tangent wants `t`; plate thickness also wants `t`~~ **arm tangent's HALF resolved 2026-09-10** — named `tangent_i` (§6), adopted from the same provisional local name, so it no longer wants `t`. Plate thickness's own choice is still open; the collision that named it is gone but the symbol is not chosen. | ~~pick one~~ **`tangent_i`** for arm tangent; plate thickness still flagged |
| `a` / `A_i` | arm length vs sinusoid coefficient, distinguished only by case | acceptable, but flag in code |
| `w` | out-of-plane component. Must **not** be reused for the rod vector, which it was in scratch code. | reserved for `L_i . n_i` |
| indexing | 0-based in code, 1-based in prose and error messages | record which, per document |

**On `h` / `h_p` → `c_p` — RESOLVED 2026-09-10.** 2026-09-03 settled the
*quantity* (§4); the symbol sat open from then until now. Renamed at every real
call site: `stewart/geometry.py` — `platform_ring` (parameter, docstring, body,
assert) and `make_geometry` (parameter, docstring, pass-through) — and
`test_kinematics.py` (fixture key). `stewart/kinematics.py` was checked, not
assumed, and confirmed to have **no** `h_p` call site: it reads the offset only
through `geom.p`. The diagnostics under `stewart/diagnostics/` carried `h_p` in
constants, docstrings and report text across sixteen files and were renamed with
it, consistently. `python -m pytest`-equivalent here is `test_kinematics.py` and
`demo.py`; both were re-run after the rename and both pass, `demo.py`'s output
byte-identical to before it.

**On the rod vector → `rod_i`, and arm tangent → `tangent_i` — RESOLVED
2026-09-10.** Both were unnamed in §6 and both had a provisional LOCAL name
already doing real work in code — `rod_i` in `score_discriminators.py`, reused
by `tilt_authority.py`; `tangent_i` the same. Adopting the name already in use
costs nothing and ends two live "unnamed" rows. Recorded in §6; no call site
changes, since the code already used these names locally and notation.md is what
was silent. The remaining clashes in this section — `R` (sinusoid amplitude),
`s` (screw direction), `t` (plate thickness alone now), `a`/`A_i`, `p` (pitch) —
have no agreed answer and are left flagged, not guessed at.

## 12. Not yet decided

Listed so nothing in this file is mistaken for a settled value.

- ~~The `z` component of `p_i` in `{P}`, and where `{P}`'s origin sits.~~
  **Settled 2026-09-03 — see §4.** Plate top (or one ball radius above for the
  ball-centre plane), coplanar anchors, one shared `z`. What remains open is only
  the *symbol*, §11.
- The rotation convention behind roll/pitch/yaw (§6).
- ~~`a` and `d`.~~ **Ranges now populated from the hardware pull — 2026-09-09**
  (`docs/hardware-pull.md`; `stewart/diagnostics/sweep_ranges.py`, `2af4f3a`).
  Neither is *chosen* — no horn, joint or servo is picked by this entry — but
  neither is an unconstrained real interval any more either.

  **`a` is a DISCRETE set, not an interval.** 32 published ProModeler hole
  positions, `9.0`–`60.4 mm` = `0.1000`–`0.6711 r_b` at the fixed `r_b = 90 mm`,
  each tagged `STATED` (every position printed), `ENDPOINT` (the ladder is
  elided on the maker's page and only the printed ends are used — interior
  holes are **not interpolated**) or `DERIVED` (positions follow by arithmetic
  on a stated count and pitch, not read off the page). The ceiling, `60.4 mm`,
  is a **hardware** ceiling — no single off-the-shelf arm was found longer —
  not a sampling wall; the Thingiverse 44–92 mm "extension arms" are a 3D
  print, not stock, and are excluded.

  ~~**`a`'s optimum is INTERIOR**, and it is the first axis to come out clean end
  to end: best `margin(dxy = p) = 0.847657` at `a = 47.63 mm` (`0.5292 r_b`),
  with published holes on both sides of it.~~ **STALE 2026-09-10.** That was
  measured with `d` at three values (`0.8`/`1.2`/`1.6 r_b`) and `beta`/`beta_p`
  at `10°`/`15°` sampling. On the `2fee8b0` sweep — `d` open to `2.0` at `0.1`
  steps, both angles at `2.5°` — the optimum sits at the `60.40 mm` hardware
  ceiling instead. The earlier reading — `box_boundary`'s `a/r_b = 0.60`
  pairwise-favoured direction — pointed at a **gap in the hole ladder**, not a
  wall: `54 mm` is not a hole on any published arm; that reading still stands,
  and the pairwise line count on the finer grid still turns over near `45.40`
  vs `47.63 mm` — the typical candidate still leans toward a mid-range hole.
  Only *the optimum* moved to the ceiling. See the 9 September entry,
  `docs/phase-0-design-log.md`.

  **`beta_p` is bounded by ball-joint housing OD**, published range
  `9.0`–`13.0 mm` across twelve M3-class parts. Computed at **both** ends:
  `[3.2246°, 56.7754°]` at `9.0 mm`, `[4.6604°, 55.3396°]` at `13.0 mm`. Both
  carried in parallel through the sweep; **no joint is chosen between them**.

  **`d/r_b` capped at `2.0`, i.e. `180 mm` at `r_b = 90` — ASSERTED, a
  judgement about rod slenderness and bow, NOT a hardware limit.** Rod stock
  runs to `2.1x` the longest length this sweep has ever used (§7 of the
  hardware pull), so length is not what bounds `d`. **Both straightness cells
  came back NOT RETRIEVED** — MISUMI's product and PDF pages returned HTTP
  403; McMaster-Carr's category page carries filters only and its product
  pages, where straightness lives, were unreachable for a different reason.
  So a `108 mm` rod at `d/r_b = 1.20` — which is where the sweep's own optimum
  currently sits — has **no published straightness or buckling figure behind
  it**, and `d/r_b = 1.20` sits on the cell maximum in both directions the box
  has been opened, so **the wall is untested, not merely unconfirmed**.

  **`beta`'s usable range still needs the installed-arc clearance, not just
  body width.** The pull's §5 publishes body width for five Hitec/Savox
  parts, `11.4`–`13.0 mm` (`0.127`–`0.144 r_b`) — a real number, and it is
  **not** among the pull's 39 unpublished cells. But the quantity §12
  actually needs — *two bodies cannot occupy one mounting arc* — is the
  **installed clearance**, and the mounting-flange footprint, screw-hole
  pitch and required inter-body clearance (including wiring) are explicitly
  **among the 39**, listed under §5 as unpublished for every servo checked.
  Body width is a proxy, on record; the clearance the bullet below asks for is
  not.

  **`beta` is bounded, PERMISSIVELY, not closed — 2026-09-10.** The
  installed-arc clearance above is still not published; nothing here supplies
  it. What is used instead is the largest published sub-micro **CASE SIZE**,
  `13.0 mm` (§5, MFR, Hitec HS-5085MG / HS-5087MH) — the bare body, not the
  installed footprint. Requiring the arc at `r_b = 90` to clear it on both
  gaps (`2 beta` within a pair, `120 - 2 beta` between pairs) gives
  `beta` in `[4.1380°, 55.8620°]`, derived from the clearance rather than
  chosen as degrees; both endpoints give exactly `13.00 mm` of arc, and both
  collision ends (`beta -> 0` a pair closing on itself, `beta -> 60` a pair
  closing on its neighbour) are checked. **This bound is PERMISSIVE and
  cannot loosen**: flanges, screw pitch and wiring only ADD to case width, so
  it cannot exclude a candidate that would have been buildable, and it
  tightens rather than loosens once the installed figure exists. Swept in
  `stewart/diagnostics/sweep.py` (`2fee8b0`); measured NOT to have been
  carrying the `e12235c` shortlist's result — see the 9 September entry,
  `docs/phase-0-design-log.md`.
- ~~**`z_home`'s range**~~ *(reopened 2026-09-04; bracketed 2026-09-05 — this
  supersedes the entry that said nothing supplied a range)*. Both ends are now
  computed, from `stewart/diagnostics/zhome_bracket.py`.

  **Lower, closed form, from `N_i > 0`:**

  ```
  z_home  >  r_p sin(tilt) + c_p cos(tilt)      = 0.182733 r_p + 0.983163 c_p
  ```

  at `tilt = 10.529°`. It is **`delta`-free, `a`-free and `d`-free**: `v_i = z`
  exactly under horizontal shafts and `b_i . z = 0`, so `N_i = q_i . z` and the
  bound involves only `r_p`, `c_p` and the tilt limit. Verified against
  `make_geometry`: `min N_i` at the bound is `0` to `5.6e-17`.

  **At the tilt limit now in force (6.558°, §9) the same closed form gives:**

  ```
  z_home  >  r_p sin(tilt) + c_p cos(tilt)      = 0.114208 r_p + 0.993457 c_p
  ```

  **This bound is tilt-dependent by construction — sin and cos of the limit —
  so it moves whenever the limit does, and does not carry over from the
  10.529° figure above.** The `0.182733 r_p + 0.983163 c_p` form is retained as
  the value everything before 2026-09-09 was screened against; it is not the
  bound in force. (`stewart/diagnostics/tilt_dropped.py`, `2af4f3a`, verifies
  the rebinding reaches all three places the limit enters — the screen
  envelope, the harness grid and this floor — together, at every limit
  checked.)

  Two things about it. It is a **continuum** bound — a discrete pose grid reports
  the constraint satisfied slightly *before* it truly is (measured `+5.9e-4` on the
  29-pose harness grid at `beta_p = 25°`), so the harness must take this bound from
  the formula, **not** from its own grid. And the back-of-envelope
  `z_home - c_p > r_p sin(tilt)` is **not** this bound: it drops the `cos(tilt)`,
  overstating the requirement by `c_p(1 - cos tilt) = 1.68e-3 r_b` at
  `c_p = 0.1 r_b`. Conservative, so it errs safe, but it is not the bound.

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

  **The `X = 1` crossing is limit-dependent — 2026-09-08**, from
  `stewart/diagnostics/tilt_bracket.py`. That single candidate is the *whole* of
  the counterexample: it is what refuted "`N_i > 0` is not binding". Re-running
  the same 540-candidate screen at three tilt limits — `tau_L` = 75 / 150 /
  300 ms, tilt **8.5383 / 10.5290 / 14.5520 deg** — the count of that category is
  **0 / 1 / 2**:

  - at **8.54 deg**, `X = 0`, and the claim "`N_i > 0` is not binding" **stands
    unrefuted**;
  - at **10.529 deg**, `X = 1` — the candidate above;
  - at **14.55 deg**, `X = 2` — it fails twice.

  So **the correction rests on one candidate at one provisional tilt**, and
  `tau_L` is the provisional number (§9). The candidate is real — its reach
  ceiling was refined by bisection, not read off the `0.025 r_b` grid — but *the
  existence of the category* is a property of the tilt limit, not of the
  constraint set.

  **Flagged, not re-run — 2026-09-09.** `tau_L` is no longer provisional; it is
  **dropped** (§9), and the `75`/`150`/`300 ms` sensitivity range this
  correction was measured across never included `tau_L = 0`. The limit now in
  force, `6.558°`, sits **below** the lowest tilt this three-point range
  considered (`8.538°` at `tau_L = 75 ms`). Whether the `X = 1` category
  reappears at `6.558°` specifically is **not answered by the table above** and
  is not re-measured here — `tilt_dropped.py` (`2af4f3a`) re-runs the *fixed-
  ratio* screen at `6.558°` and finds `0` of that category there, but that is a
  **different, narrower** screen (`r_p/r_b` fixed at one value, not the three
  this paragraph's 540-candidate grid spans), so it does not stand in for a
  re-run of *this* grid at the new limit.

  **What survives the full range, stated separately.** `N_i > 0` **never sets the
  lower end of a surviving bracket**: 0 of **425 / 363 / 259** at the three
  limits. That is the limit-independent part of the 2026-09-05 sentence — and it
  is still a **survivorship sample** of exactly the kind the 2026-09-05
  correction named, now taken three times instead of once.

  Two figures from the same module that must not be read without their caveats:

  - **The censored margin minimum.** The survivors' five-number summary has a
    minimum that is **non-monotone in the tilt limit and is not a trend**. It is
    taken over a survivor set that is itself shrinking, and the worst candidate
    at any limit is typically **one about to leave the set entirely**. Each
    pass's argmin is traced across all three limits and marked `infeasible`
    where the carrier is gone — so that the three minima are not differenced
    against each other.
  - **The rank correlation, and the shrinking common set behind it.** Spearman
    of the constrained-tune margin ranking against the 10.529-deg reference:
    **`rho = 0.987`** downward (8.54 deg) and **`rho = 0.959`** upward
    (14.55 deg). Both are over candidates feasible at **both** limits, and that
    common set shrinks as the limit rises. **A high `rho` on a shrinking common
    set is a weaker statement than the same `rho` on a stable one**; the lost and
    gained counts are reported beside it for that reason.

  `z_flat(delta)` (§8, derivation §9) remains a useful reference point inside the
  bracket even though home no longer sits on it.

  **The `z_lo <= floor` coincidence test cannot fire, and a different test that
  can now runs beside it — 2026-09-09.** `fixed_ratio.py`'s report of the lower
  end has always asked `z_lo <= floor`; the screen it reads intersects reach
  with `Z_GRID > floor` **strictly**, so that test is `0` **by construction**,
  not by measurement — the screen cannot produce a survivor that fails it. The
  real question, *does the `N_i > 0` floor set the lower end the reach test
  alone would have given?*, needs the reach test's own lower end before the
  floor is intersected in — call it `reach_lo` — and asks `reach_lo < z_lo`.
  `tilt_dropped.py` (`2af4f3a`) adds `reach_lo` to the screen record and
  reports both tests side by side: at every one of the four tilt limits
  measured (6.558 / 8.538 / 10.529 / 14.552°), the coincidence test reads **0**
  and the binding test reads **8** — the floor sets 8 lower ends at every limit
  checked, unchanged by the limit moving. **Same shape, same constraint,
  second instance**: the 2026-09-05 correction above found the `N_i > 0` claim
  resting on a **survivorship sample** (the 363 candidates measured were the
  ones that had *already* survived, so a crossing among them could not have
  appeared); this is a test that reports zero **because of how the screen is
  built**, not because nothing binds. Both are "the metric was structurally
  prevented from showing the effect it was checked for." `N_i > 0` is proving
  load-bearing **more often than each individual measurement of it
  suggested** — first at the edges of the bracket set (2026-09-05), now at the
  lower end of every surviving bracket (2026-09-09).

  **Zero-width brackets are a scan artefact, and a minimum width is now a
  feasibility test — 2026-09-09.** At the 6.558° limit, **21 of the 143**
  candidates with a non-empty bracket have a grid width of **`0.000 r_b`** — a
  single feasible point at the `0.025 r_b` scan resolution, not a design. This
  overstates the 143-vs-122 improvement over the 10.529° reference by
  **composition**: on the **122** candidates common to both limits the median
  width **rises**, `0.337 -> 0.462 r_b`; the **21** the lower limit *adds*
  carry a median width of `0.000 r_b`, and it is that addition, not a widening
  of the existing set, that pulls the pooled median down
  (`0.337 -> 0.225 r_b`, the wrong-direction reading a pooled figure alone
  would give). **`MINIMUM BRACKET WIDTH = 2 mm = 0.0222 r_b` — ASSERTED, from
  build tolerance on `z_home`, not measured.** This sits **just under one
  `0.025 r_b` grid step** (`2.25 mm` at `r_b = 90`), so it removes the 21
  zero-width candidates and **little else past them** — a floor beneath which
  a bracket is not buildable, not a discriminating filter that reorders the
  surviving field.

  **This is the FOURTH recorded instance of a discrete grid reporting
  something the continuum does not**, and the other three are named so the
  pattern is on the record rather than implied:
  1. **This same `N_i > 0` lower bound**, above: the 29-pose harness grid
     reports it satisfied **`+5.911e-4`** before it truly is (2026-09-05).
  2. **The tilt-azimuth pose grid**, §9: the 10° grid overstates worst-case
     margin by **`~1.9e-3`** against a 0.25° reference (2026-09-05), bounded
     at **`2.88e-3`** and used as the ranking's resolution (2026-09-07).
  3. **`box_boundary`'s per-ray stepping rule**, the 8 September entry in
     `docs/phase-0-design-log.md`: on the `a` ray the stopping rule **would
     have stopped at step 5 and been wrong** — the top-5 lost the sampled
     extreme and then got it back at a later step. Named there as "the same
     failure mode as the 15/10/0.25° azimuth aliasing," i.e. instance 2.
  4. **Zero-width `z_home` brackets**, here: a single grid point registering
     as a full bracket, overstating the feasible count's *improvement* rather
     than any one candidate's feasibility.

  All four are the same shape at different resolutions — a grid point
  standing in for an interval too narrow for the grid to see — and none has
  yet been wrong about **feasibility itself**: each is a resolution problem in
  a *derived* quantity (a bound's satisfaction margin, a ranking's optimism, a
  probe's stopping decision, a bracket's apparent width), not a case of the
  closed-form feasibility tests giving the wrong verdict.
- ~~`beta_p`'s range, which needs a **ball-joint housing diameter**~~ — **closed
  2026-09-09, below.** Published range 9.0–13.0 mm, both ends computed, no
  joint chosen.
- ~~`beta`'s usable range, which needs a **servo body diameter**~~ — **bounded,
  PERMISSIVELY, 2026-09-10, below.** Body width is published, installed-arc
  clearance is not; bounded instead at the largest published sub-micro CASE
  SIZE, `13.0 mm`, giving `beta` in `[4.1380°, 55.8620°]` at `r_b = 90`. This
  is a substitute for the still-unpublished clearance, not that clearance
  itself, and it tightens rather than loosens once that figure exists — NOT
  the same standing as `beta_p`'s closure above.

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
  by side and none preferred. — **`p` fixed 2026-09-08 at `0.004330`, ASSERTED
  and not measured: §8.**

  **Restated 2026-09-08 — the strike-through above is kept, and it is only half
  the story.** Two different things were being tracked under one line:

  - **The FORM is settled** (2026-09-07). `score = margin(dxy = p)`; one term and
    nothing to weight; the `cond(J_fk) <= 1e6` cap on the inner `delta` tune, not
    a floor on the outer score; the output a **tie set, not a winner**. All of it
    measured, all of it in §8. **Not reopened.**
  - **The OBJECTIVE it encodes is OPEN** (2026-09-08). **`margin` has no interior
    optimum.** It measures distance from unreachability, not capability, so a
    platform ring shrinking toward a point improves it and nothing penalises a
    mechanism that can barely move. Opening the sampled box moves the optimum
    *further onto* the boundary: `r_p/r_b` runs to `0.10`, the smallest value
    probed, margin climbing monotonically to `0.963815` with no turn.

  So the function is well-specified and computes what it says; **what it says is
  not yet the thing worth maximising.** A settled form over an open objective is
  not a settled scoring function, and the strike-through should not be read as
  though it were. Sources: **§8** for the form and the tie/optimum distinction,
  and the **8 September entry in `docs/phase-0-design-log.md`** for the objective
  finding, its options — none chosen — and its residuals.

  **The tilt-authority option is CLOSED, and the reason is itself a finding —
  2026-09-08/09.** `stewart/diagnostics/tilt_authority.py` (`3a01bd3`) measures
  `rho(authority, score) = +0.9088` over the 363 feasible candidates —
  authority ranks **with** the margin, more strongly than `tau_min`'s `+0.835`
  already recorded above as measured-and-redundant — and **negative on all
  four `box_boundary` rays too** (`rho(alpha_span, score)`: `a` up `-0.2237`,
  `r_p` down `-0.6251`, `d` down `-0.2556`, `d` up `-0.9387` — negative
  `alpha_span` correlation is positive `authority` correlation, same sign as
  the coarse-grid figure throughout). **On the runaway axis it agrees with the
  runaway, not against it**: down the `r_p` ray from the incumbent `0.60` to
  `0.10`, best score climbs `0.784 -> 0.951` while the winner's `alpha_span`
  **falls** `18.89 -> 1.89°`. A shrinking platform is better on both readings
  at once.

  **Why, measured not asserted (part 2b of that module).** The moment-arm
  argument for a tilt-authority term assumes an envelope commanding a fixed
  platform **displacement**. The settled envelope (§9) is `dxy = dz = yaw = 0`
  and a fixed tilt **angle**, so anchor excursion goes as `r_p sin(tilt)` and
  **shrinks with the ring** — a smaller platform reaches the same commanded
  angle with less servo swing, not more. **General form, stated once so it need
  not be re-derived per candidate term: under a purely angular envelope, no
  kinematic quantity bounds `r_p` from below.** That is the reason `r_p` was
  **fixed** rather than **bounded** — no term measured on this envelope was
  going to supply the missing lower bound. (Recorded for if authority is
  revisited: the margin-tuned and swing-tuned `delta` differ on `349` of `363`
  candidates, median gap `6°`, so a resurrected term would have to enter the
  **inner tune**, not the outer score — the same rule `cond` follows, above.)

  **Fixing `r_p` is not the same move as closing the objective, and the
  distinction is worth restating precisely because the two happened in the
  same session.** Fixing `r_p` (below, *Absolute scale*) removed the **axis**
  the runaway was measured on — `r_p/r_b` is no longer free to run to `0.10`,
  because it is no longer free at all. It did **not** fix what was wrong with
  the runaway: `margin` still measures **distance from unreachability**, not
  **capability**, on every axis that remains free. **The objective stays
  open** under the same diagnosis as the 8 September entry in
  `docs/phase-0-design-log.md`, and fixing `r_p` should not be read as having
  closed it — it closed off one candidate's-worth of counterweight (tilt
  authority) and one axis of the runaway (`r_p/r_b`), and left the objective
  itself exactly as open as it was.
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

**Absolute scale — 2026-09-08. ASSERTED, not measured.** Decisions taken in
discussion. **No diagnostic stands behind any of them**, and none is cited.

**Absolute scale enters in exactly two places.** Both are below; there is no
third, and in particular **not** the tilt target, whose plate-size dependence was
withdrawn the same day (§9):

1. **The bed.** Print volume **180 x 180 mm**, so **`r_b <= 90 mm`**. This is the
   **upstream absolute length**, and **it arrived from the bed, not from torque**.
2. **`p`'s denominator** — the asserted **80 mm `r_b` floor** that normalises
   `p = 0.3464 / 80 = 0.004330` (§8).

**Sharpened 2026-09-09 — `r_b` is fixed, not floored, and `r_p` joins it.**
Both numbers above have moved:

1. **`r_b = 90 mm`, exactly — the full bed, not a ceiling on it.** The
   2026-09-08 wording, `r_b <= 90 mm`, left the exact value open; the design
   now uses the whole bed and `r_b` is fixed at `90 mm`.
2. **`r_p = 80 mm` — ASSERTED, from the printed hub carrying the bought
   `220 mm` sheet** (the sheet sized under *Bought-part specification* below:
   `~110 mm` radius = `220 mm` diameter). This is a **new** entry, not on the
   original 2026-09-08 list of two.
3. **`p`'s denominator was item 2 above, and it was `r_p` under the wrong
   name.** The `80 mm` that normalised `p` was labelled "an asserted `r_b`
   floor"; it is the same `80 mm` as `r_p` here. `r_b`'s actual fixed value is
   `90 mm`. §8 carries the correction and the recomputed `p = 0.003849`.

**Consequence for the sweep: `r_p/r_b = 80/90` and `r_p/r_b` is NO LONGER A
SWEEP AXIS.** The sweep is **five** axes — `beta`, `beta_p`, `a/r_b`, `d/r_b`,
`z_home/r_b` — not six. **This is not the "five axes" struck on 4 September**
(`docs/phase-0-design-log.md`): that entry's five dropped `z_home/r_b`, which
is back and stays; this five drops `r_p/r_b` instead, which the 4 September
entry never touched. Same count, different set — read as a coincidence of
arithmetic, not a reversal of that decision. **The normalised
parameterisation's justification has changed with it, not the invariance
itself (§8's `k` row): it was "absolute scale is unknown," so every length
was carried as a ratio because there was nothing to divide by yet; it is now
"ratios are the natural parameterisation" of a mechanism whose two absolute
lengths are fixed and small in number. The `k`-invariance results —
`alpha_i` exactly invariant under a uniform scale — stay true and stay
measured; they are now DESCRIPTIVE of the geometry rather than LOAD-BEARING
for why the sweep is normalised at all.**

**The ball surface is a bought plastic sheet on a hub**, decoupled from the
anchor ring. Three things follow:

- **Plate radius does not bound `r_p`.** The sheet is bought, not printed, so
  `x0 = 80 mm` **survives a bed that could not carry a printed plate of the
  required radius**.
- **The tilt target is a fixed angle** — 10.529 deg at every mechanism scale,
  because `x0` is a property of the sheet. §9 carries this and what it withdrew.
- **`c_p` change from the hub-and-sheet arrangement is judged negligible**, and
  **leg forces are re-ruled-out with the bought sheet in mind**. The 20 August
  force ruling (2.7 g ball) **predates the sheet**, so this is a **fresh
  judgement, not that ruling carrying forward**.

**Bought-part specification, not a constraint.** Sheet radius about **110 mm** —
contact point to 80 mm, ball hanging 20 mm past it, plus edge margin. It is a
number for the order form. **It bounds nothing the sweep searches over.**

**Consequence for the objective, and it is the live one.** The decoupling
**removes the one physical argument that would have bounded `r_p` from below**.
The `r_p` runaway measured in `box_boundary.py` is therefore **live** — nothing
about the plate stops it. See the 8 September design-log entry.

**The `r_b = 100 mm` fixtures predate the bed constraint.** `cc-fk-gate.md`'s
four fixtures and `box_boundary.py`'s buildability columns are quoted at
`r_b = 100 mm`, which is **above the 90 mm ceiling above**. Both label it a
**units placeholder** and neither is a choice of `r_b`; the kinematics is
homogeneous of degree one and the envelope is purely angular, so that figure is a
**change of units, and every ratio, margin and residual measured at it is
unaffected**. Nothing is rescaled and nothing is re-run. Recorded so the number
is not later read as a candidate `r_b` — **it is not one, and it is now out of
range**.

**Servo travel is excluded from feasibility — 2026-09-08. A decision, not an
omission. ASSERTED.** Feasibility stays the **three** tests, unchanged:

1. non-empty `z_home` bracket,
2. envelope reachable, `|P_i| <= C_i`,
3. `N_i > 0`.

The angular range `alpha_i` is allowed to sweep is **not** a fourth test.
Recorded here so it is not reintroduced later as an oversight.
