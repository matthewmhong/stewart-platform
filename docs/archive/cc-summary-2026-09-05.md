# Claude Code session summary — 2026-09-05

Envelope propagation and the diagnostics it unblocked. Three commits:
`c9d14b2` (diagnostics), `3d0fc42` (documents), plus this file.

No production code touched. `stewart/kinematics.py`, `stewart/geometry.py` and
`test_kinematics.py` are byte-identical to their state at the start of the
session; `test_kinematics.py` and all six diagnostics exit 0.

---

## Decisions taken under latitude

| # | Decision | Reason |
|---|---|---|
| 1 | Four modules, not one: `envelope.py`, `azimuth_symmetry.py`, `zhome_bracket.py`, `branch_envelope.py` | Each answers one question and can be re-run alone; a single script would couple (e) to (a)'s runtime. |
| 2 | `envelope.py` is the single source of truth and every other module imports the tilt limit from it | The provisional 6° drifted across scripts and documents once already; a constant that lives in one place cannot. |
| 3 | The tilt limit is **computed** from the recovery model at import, not typed as `10.5` | A typed constant cannot be checked against its own derivation. It prints as 10.5290°. |
| 4 | Azimuth swept at 0.25° over the full circle in (a) — 1440 samples | 120 and 180 are exact multiples of the step, so the invariance tests compare *sampled* values and no interpolation error can be mistaken for a symmetry breaking. |
| 5 | Harness pose grid **5 magnitudes × 7 azimuths = 29 poses**, magnitude 0 counted once | 7 azimuths gives a sample every 10° across the 60° window, beating the superseded sweep's 15°. Magnitude 0 is azimuth-independent, so counting it 7 times would be 6 duplicate poses. 5 magnitudes because nothing has shown the worst case monotonic in tilt magnitude. |
| 6 | Bracket study uses a **finer** envelope than the harness (6 × 61 = 306 poses) | A bracket is a worst case, and a worst case sampled coarsely is not a bracket. |
| 7 | Geometry grid 5 `beta` × 4 `beta_p` × 3 `r_p/r_b` × 3 `a/r_b` × 3 `d/r_b` = 540; `h_p/r_b = 0.1` fixed | `h_p` is a hardware number measured off the plate, not a sweep axis (`notation.md` §4). 540 runs in 20 s, which keeps the diagnostic re-runnable. |
| 8 | `z_home/r_b` scanned 0.025→3.0 at 0.025, **no bisection refinement** *(amended: crossings ARE now bisected — decision 19)* | These brackets decide whether a candidate survives; they do not set a dimension. A feasible island narrower than 0.025 `r_b` is not a design. Reported endpoints are grid values and say so. |
| 9 | Float noise threshold **1e-12 relative** to each quantity's own scale | The measured invariances land at 1e-13…1e-16 and the negative control at 1e-1. The threshold is nowhere near either, so it is not doing any work. |
| 10 | Negative control in (a) = one platform anchor displaced 0.03 `r_b` in `x` | Breaks D₃ without touching anything else, so a surviving pass cannot be passing for another reason. It fails at 6.6e-1 / 3.7e-1. |
| 11 | (e) recomputed on `branch_check.py`'s **fixture A verbatim**, and the old envelope reproduced first | Recomputing on a different geometry would not be a recomputation. The old figure reproduces to `−5.713988e-03`, which validates the comparison harness before it is used. |
| 12 | (e) reports an **attribution table**, not one replacement number *(**withdrawn** — Amendments (2))* | The two envelopes are not nested — more tilt, no yaw, no translation — so a single before/after would be uninterpretable. |
| 13 | Continuity in (d) checked over the **full circle**, not the 60° window | The 60° window is a scoring shortcut. Continuity is a claim about the trajectory the machine actually follows. |
| 14 | Reach test implemented as `w_i(delta)² ≤ |L_i|² − P_i²` | Exactly equivalent to `|P| ≤ C` after squaring, and all three terms are `delta`-free, which turns a nested search into one pass. |
| 15 | The 1.635° appendix entry **kept and relabelled**, not struck | It is correct for what it measures, and the two framings scale oppositely in `k` — worth not re-deriving. Only the 45 mm / 300 mm/s bullet was struck, as instructed. |
| 16 | Design-log skeleton placed under the `## 4 September` heading as asked, but headed `### 5 September` | Placement as instructed; misdating a day's work to tidy the structure would be worse than the odd nesting. Flagged here because it is a deviation in spirit from "under the 4 September heading". |
| 17 | Handoff open items 1, 5, 8, 11 struck through and annotated rather than deleted | Same convention the 2026-09-04 pass established: a reader hitting a closed item must be able to see what it said. |

---

## Findings

### (a) Azimuth range — **the claim is right about the width and wrong about the position**

| invariance | worst relative deviation | verdict |
|---|---|---|
| period 120° | `1.026e-13` | holds |
| mirror `psi → 180 − psi` | `5.648e-14` | holds |
| mirror `psi → −psi` | **`2.303e-01`** | **fails** |
| negative control, period 120° | `6.588e-01` | fails as intended |
| negative control, `180 − psi` | `3.736e-01` | fails as intended |

A 60° window **is** sufficient — but it is **`[30°, 90°]`, not `[0°, 60°]`**.

A reflection in the vertical plane at azimuth `m` has `det M = −1`, and
`M R(n,θ) Mᵀ = R(det(M)·Mn, θ)`, so a tilt axis at `psi` maps to one at
`2m + 180 − psi`. With mirror planes at 0°, 60°, 120° the fixed azimuths are
90°, 150°, 30° — i.e. `30 + 60k`. Physically: tilting about an axis lying *in* a
mirror plane reflects to the *opposite* tilt; the self-symmetric axis is the one
*perpendicular* to the plane. `notation.md` §10's `|w|` table already said this
in its third row.

`[0°, 60°]` is symmetric about its own centre, so it covers the orbits it
touches **twice** and misses others outright — the orbit `{75°, 105°}` mod 120
meets it nowhere. Sweeping it would silently omit part of the envelope.

**The pose count is unchanged** (a 60° window either way), so Part 1's grid row
did not move on this account. Only the window's position changed.

### (b) `z_home` lower bracket — closed form ~~and never binding~~ *(see Amendments (3))*

```
z_home  >  r_p sin(tilt) + h_p cos(tilt)   =  0.182733 r_p + 0.983163 h_p
```

`delta`-free, `a`-free, `d`-free: `v_i = z` exactly under horizontal shafts and
`b_i·z = 0`, so `N_i = q_i·z`.

| check | residual |
|---|---|
| `min N_i` at the closed-form bound, vs `make_geometry` over 540 grid points | `5.551e-17` |
| `max\|v_i − z\|` | `1.110e-16` |
| `min N` vs `z_home − bound`, digit for digit | identical (the closed form checking itself against the library ring) |

Three findings beyond what was asked:

1. **The back-of-envelope estimate is not the bound.** `z_home − h_p > r_p sin(tilt)`
   drops the `cos(tilt)`, overstating by `h_p(1 − cos tilt) = 1.684e-3 r_b` at
   `h_p = 0.1 r_b`. It is *conservative*, so it errs safe — but it is not the bound
   and the exact one is free.
2. **It is a continuum bound, and a pose grid gets it wrong in the unsafe
   direction.** The 29-pose harness grid reports `min N = +5.911e-4` at the exact
   bound where the truth is 0 — i.e. it says the constraint is satisfied slightly
   *before* it is. The harness must take this bound from the formula.
3. **`N_i > 0` is not the binding constraint.** In **0 of 363** candidates with a
   non-empty bracket did it set the lower end. Reach binds first, everywhere on
   the grid. The test must stay — the `-` branch rests on it — but it is not what
   shapes the axis at this tilt. — **CORRECTED, see Amendments (3).** The sample
   was survivorship; it binds in 1 of the 177 empties and keeps the feasible set
   connected in 6 more.

### (c) `z_home` upper bracket — per candidate, no closed form

540 candidates, `z_home/r_b` scanned 0.025→3.0 at 0.025, `delta` at 1°:

| | |
|---|---|
| non-empty bracket | **363 / 540** |
| empty | 177 / 540 |
| non-contiguous feasible sets | **0** |
| lower ends | `[0.225, 1.650]` |
| upper ends | `[0.425, 1.875]` |
| widest / narrowest bracket | `0.825` / below the 0.025 scan resolution |

Empty brackets concentrate hard in `a/r_b`: **144 of 180** at `a/r_b = 0.10`,
33 of 180 at `0.20`, **0 of 180** at `0.35`. By `d/r_b` the spread is mild
(42 / 61 / 74 of 180). The short-arm corner of the coarse grid is mostly dead,
and that is information for the sweep, not a problem with it.

### (d) `-` branch re-run — the branch stands

Fixture A, tilt 10.529°, `z_home` swept past its bracket both ways, precession
over the full circle at 0.25° (1440 steps):

| check | result |
|---|---|
| `min(N_i) > 0` over envelope, every `z_home` | **passes**, worst `+0.896361 r_b` |
| envelope fully reachable | 8 of 15 `z_home`, `[1.2000, 1.2875] r_b` |
| branch reaching `alpha ≈ 0` at home | `-`, at every feasible `z_home` |
| branch-flip step outliers | **0** |
| loop closure `max_i\|alpha_i(360°) − alpha_i(0°)\|` | `≤ 1.78e-15` |
| `max\|ik() − alpha_minus\|` | `8.882e-16` |

Open item 5 is discharged as a test, and open item 11 as a re-run.

### (e) Boundary margin — superseded, with the move attributed

Old envelope reproduced exactly at the datum `z_home = 1.223343`:
**`−5.713988e-03`**. Settled envelope, same geometry, same height:
**`+1.550398e-01`**. Tuning `z_home` alone at fixture A's untuned `delta = 40°`:
**`+2.276643e-01`** at `z_home/r_b = 1.2375`.

**The four-row attribution table that stood here is WITHDRAWN — see
Amendments (2).** The individual costs survive: yaw ±10° `1.8033e-01`,
translation ±0.05 `r_b` `3.6486e-01`, tilt 6° → 10.529° `3.6147e-01`.

**The `−5.7e-3` was not driven by tilt.** Translation and the tilt increase cost
comparable amounts; yaw about half. It was yaw and translation together, and the
settled envelope has neither. Raising tilt 6° → 10.529° spends most of what that
buys back; the net is positive.

This matters for open item 1's warning about not shrinking the envelope to clear
the boundary: **nothing was shrunk**. The envelope grew by 75% in the one axis
the control law uses and went to zero in two it does not.

---

## Hit a BOUNDARY and stopped

**Nothing.** No boundary item blocked the work.

For the record, the boundaries held rather than went untested:

- The tilt limit was never reduced. (b), (c) and (e) were all evaluated at
  10.529°, and (c) returned **177 empty brackets out of 540** — reported as a
  finding, with the `a/r_b = 0.10` concentration named, not repaired.
- `dxy`, `dz` and `yaw` stayed at 0 throughout. The old envelope's yaw and
  translation appear in (e) **only** inside the reproduction and attribution of
  the superseded figure, never as a proposal.
- No branch rule chosen; `fk()`'s residual and convergence criterion untouched;
  derivation §9 untouched; no `h_p → c_p` rename; nothing renumbered.
- No first-person reasoning in `phase-0-design-log.md` — facts, residuals and
  `TODO(him): reasoning` only.

---

## Stale or wrong, and not in the prompt

**1. The envelope is TWO axes, not four.** The handoff records "the envelope is
four axes, not six — `x, y, tilt magnitude, tilt azimuth`". `dxy = 0` removes
`x` and `y`. What remains is tilt magnitude and tilt azimuth. Corrected in
`notation.md` §9 and the handoff.

**2. Every downstream compute figure in the handoff was wrong**, because they
were all built on 486 `w` evaluations per objective evaluation and the real
number is **174**. The compute ledger's "~7.6M full plus ~1.4e9 cheap" becomes
**~2.7M full plus ~4.9e8 cheap**, and the chunking constraint's "1.4e9 floats is
~11 GB" becomes **~3.9 GB**. The chunking requirement survives; the number in it
did not. All updated, with the old figures quoted.

**3. `notation.md` §12 said "nothing currently supplies a range" for `z_home`.**
(b) supplies the lower one in closed form. Updated — and the §8 status row
"swept, range undecided" with it.

**4. The `1/k` correction is larger than a sign flip in one sentence.** The old
mechanism appears in the handoff's scaling paragraph, in open item 1's
justification, and by implication in the derivation appendix's 1.635°. All three
now carry the recovery framing. The two framings agreeing at `tau = 1.0 s`
(recovery gives 1.636° against arrest's 1.635°, the same 0.2 m/s² by another
route) is a cross-check worth having and was not asked for.

**5. `notation.md` §10 already contained the fact that refutes `[0°, 60°]`.**
Its `|w|`-pattern table lists "tilt about an axis **perpendicular to a mirror
plane**" as the symmetric case. That is precisely why the fixed azimuths are at
`30 + 60k` and not `0 + 60k`. The information was in the document; the claim was
made without consulting it. §10 now states the reflection map explicitly.

**6. `branch_check.py`'s `[2b]` block reports the margin as `(C−|P|)/C`
already.** The normalised form adopted on 2026-09-04 as a *revision* was the form
the code had been computing all along. Not wrong, but the handoff reads as though
the normalisation were new; it was the prose that lagged.

**7. The `ik()` docstring's branch justification names a dropped datum.** It says
the `-` root "is the root continuously connected to the assembly datum
`alpha_i = 0`" and cites a 2026-09-03 measurement at that datum. The datum was
dropped 2026-09-04. **The argument is unaffected** — it rests on `N_i > 0`, not
on the datum — and (d) re-establishes the same conclusion at 10.529° across the
restored `z_home` axis (`ik()` agrees with the `-` root to `8.882e-16`). I did
**not** edit it: the prompt forbids touching production code, and the docstring
is not wrong, only phrased around something no longer current. Flagging it as a
one-line docstring edit for whenever production code is next open.

**8. The design log's undated "Where Phase 0 stands" section is still stale.**
It reads "My immediate open question is the branch rule", which was fixed on
2026-09-04. Flagged in the previous session's report and still true. It is
first-person prose in his voice, so I have not touched it.

---

## Files

| Path | Status |
|---|---|
| `stewart/diagnostics/envelope.py` | new — envelope + recovery model, single source of truth |
| `stewart/diagnostics/azimuth_symmetry.py` | new — (a) |
| `stewart/diagnostics/zhome_bracket.py` | new — (b), (c) |
| `stewart/diagnostics/branch_envelope.py` | new — (d), (e) |
| `docs/notation.md` | §9 filled, §10 azimuth mirror lines, §12 `z_home` brackets, §8 status row |
| `docs/session-handoff-2026-09-04.md` | open items 1, 5, 8, 11 closed; `1/k` corrected; envelope, compute ledger and harness plan updated; 2026-09-05 log row (Engagement left empty) |
| `docs/stewart-ik-derivation.md` | appendix only: 1.635° relabelled, 45 mm bullet struck |
| `docs/phase-0-design-log.md` | `### 5 September` skeleton under the 4 September heading |
| `stewart/kinematics.py`, `stewart/geometry.py`, `test_kinematics.py` | **untouched** |

---

# Amendments — 2026-09-05, correction pass

Four corrections. Two commits: `1b98973` (diagnostic), and the documents commit
that carries this section.

## Decisions under latitude

| # | Decision | Reason |
|---|---|---|
| 18 | Attribution uses the **closed-form** `N_i` floor, not the pose grid | The grid reports `N_i > 0` satisfied before it is, which would bias every attribution towards blaming reach. On this grid the two agree at 0 of 64800 `z` points, so it changes no count here — made so it cannot change one later. |
| 19 | Every crossing is **bisected to `1e-9`** before it is called a crossing | The one candidate found had a grid gap of `0.024` on a `0.025` step — inside one step, so reporting it unrefined would have been reporting a possible sampling artifact as a finding. |
| 20 | The `1.62e-3` discrepancy was **diagnosed but not chased** | You said not to chase it; the cause was readable off my own code in one run, so it is stated as confirmed rather than left as "likeliest". No further work. |
| 21 | Individual costs kept as three standalone measurements, not as a decomposition | Each is a difference of two measurements on **one** pose set, so each is valid on its own. Only the arithmetic that spanned pose sets was withdrawn. |

## Findings

**(1) `dxy` — no leak.** Every `dxy` in the repo is `0` (`DXY = 0.0`, or
`dxy = dz = yaw = 0`). `notation.md` §9's deleted note is still deleted — the only
line matching it is the record *of* the deletion. Envelope stays two axes, azimuth
`[30°, 90°]`, 174 `w` evaluations, ~3.9 GB chunking. **No revert needed.**

**(2) Attribution table withdrawn; cause confirmed, and it is worse than a
bookkeeping error.** The rows and the settled figure are maximins over **different
azimuth samples** — magnitude grids identical, azimuth 15° vs 10°. Confirmed by
measuring the same fixture at four samplings:

| azimuth sampling | `min (C−\|P\|)/C` |
|---|---|
| 15° over `[0°, 360°)` (the table's rows) | `+1.534192e-01` |
| 15° over `[30°, 90°]` | `+1.534192e-01` |
| **10° over `[30°, 90°]` — the harness grid, and the settled figure** | **`+1.550398e-01`** |
| 0.25° over `[30°, 90°]` — reference | `+1.531859e-01` |

Your diagnosis was right in kind. The detail differs: the rows were sampled on the
**full circle at 15°**, not on `[0°, 60°]`. And the sting is in the third row —
**the 10° harness grid is the outlier, not the 15° one.** `+1.550398e-01` is itself
grid-optimistic by ~`1.9e-3`; `+1.53e-1` is the defensible number. Withdrawn in the
2026-09-04 convention (quoted, reasoned, not deleted) in `branch_envelope.py`, the
handoff, the design log and above. Kept: `−5.713988e-03`, `+1.550398e-01`, the three
individual costs, and the conclusion — which rests on gaps of `1e-1`, not on
arithmetic that failed at `1.6e-3`.

**(3) The 177 empties attributed — and `N_i > 0` does bind.** You were right that
the sample was survivorship. The categories are fixed by the constraints' shapes:
`N_i > 0` is **one-sided** (a floor), reach is a **two-sided interval**, so there is
no `N_i` ceiling and "reach floor above the `N_i` ceiling" cannot occur. Two cases
exist:

| | count |
|---|---|
| reach empty on its own | **176 / 177** |
| reach ceiling below the `N_i` floor | **1 / 177** |

The one: `beta = 10°, beta_p = 55°, r_p/r_b = 1.10, a/r_b = 0.10, d/r_b = 0.80`.
Grid gap `0.024` on a `0.025` step — inside one step, so bisected to `1e-9`: ceiling
`0.2920624`, floor `0.29932`, gap **`+7.26e-03`**. **Real, not an artifact.** By
`a/r_b`: 144 empties at `0.10` (143 R, 1 X), 33 at `0.20` (all R), 0 at `0.35`. By
`d/r_b`: all three grid values produce empties (42 / 61 / 74), but the single
crossing is at `d/r_b = 0.80`.

**A third role for `N_i > 0`, not asked for and not anticipated.** Six candidates
have a **non-contiguous reach set** — a spurious low component at
`z_home ≈ 0.025–0.125 r_b`, the platform essentially on the base plate,
geometrically reachable and physically nonsense. In **6 of 6** it lies entirely
below the `N_i` floor, so `N_i > 0` removes it. That resolves what looked like a
contradiction with the "0 non-contiguous feasible sets" already reported: both are
true, they measure different sets. It also means `N_i > 0` is what keeps the
feasible set an **interval** at all — a bracket-based harness would be wrong without
it.

**Corrected claim, propagated to `notation.md` §12, the handoff open item 5 and its
session-log row, and the design log:** `N_i > 0` does not shape the interior of the
feasible set and is not why most candidates fail, but it **closes the bracket
outright in 1 of 177 and keeps the set connected in 6**. It is load-bearing at the
edges and cannot be dropped.

**(4) Design-log heading promoted** to a top-level `## 5 September`, with the
nesting apology removed from the note.

**(1, cont.) Translation sensitivity recorded** in the handoff's score-function plan
line: `Δmargin / (dxy/r_b)` at the tuned `delta`, per candidate, a **ranking
discriminator and not a feasibility test**, probe `0.005–0.01 r_b`, with
`dxy = 0` recorded explicitly as a statement about what the control law
**commands** and build error as a perturbation belonging in scoring. **The
calibration behind the probe was subsequently measured and does not hold — see
Provenance (E).**

## Stale or wrong, not in this prompt

**A. The settled figure `+1.550398e-01` is itself grid-optimistic.** Fell out of
(2). It is quoted in the handoff, the design log and this summary. All three now
carry the caveat and point at `+1.53e-1`. This was not part of the correction you
asked for — the table was the target; the figure it was being compared against
turned out to have the same disease.

**B. The scoring grid flatters worst cases in the unsafe direction, twice now.**
`N_i > 0` on the pose grid (`+5.9e-4` early) and the margin on the azimuth grid
(`+1.9e-3` high). Neither is large; both run the same way. Raised as **handoff open
item 12**, framed as a decision rather than a fix: refine the scoring grid, or
accept that a scoring grid may be optimistic provided **feasibility** is decided by
closed forms and only the **ranking** comes from the grid. That is a score-function
decision, so I have not taken it.

**C. `notation.md` §8's `k` row still reads "envelope included."** Written when the
envelope had translations in it. Now vacuous rather than false — an angular envelope
is trivially invariant — so I left it. Flagging it as a wording tidy for whenever §8
is next open.

**D. Still open from the last pass, unchanged:** the `ik()` docstring names the
dropped flat-arm datum (untouched, as instructed), and the design log's undated
"Where Phase 0 stands" still says the branch rule is his immediate open question.

---

# Provenance audit — 2026-09-05, end of session

Asked to confirm state is in the repo rather than in the chat. Audit method: for
every numeric literal added to `docs/` today (102 distinct), check whether some
committed diagnostic prints it. 99 did. **Three did not**, and they are now
backed by `stewart/diagnostics/sweep_budget.py`.

**No remote is configured** (`git remote -v` is empty), so every commit is local
only. Worth knowing before the machine is the single copy.

| Gap | Was | Now |
|---|---|---|
| **Compute ledger** — 15625 candidates, ~2.7M full, ~4.9e8 cheap, **~3.9 GB** | hand arithmetic in prose; the GB figure carries a *harness requirement* (chunk over candidates) | computed from the committed envelope constants, with the superseded 81- and 729-pose ledgers alongside so the handoff's comparisons reproduce |
| **Translation-sensitivity calibration** — "order 7", probe `0.005–0.01 r_b`, "small enough to stay linear" | a division done in prose; linearity never tested | measured across six probe magnitudes, full circle in both tilt azimuth and displacement direction |
| **Arrest-framing 1.635°** and "the two framings agree at `tau = 1.0 s`" | never had code, on either side | `(5/7) g sin = v²/2L` computed, and the `1/k` vs `k` scaling tabulated |

## What giving them code changed

**(E) The "order 7" calibration is wrong, and wrong in kind.** Measured
horizontal sensitivity at fixture A is **~1.4**, a factor of ~5 smaller. The 7
came from dividing the (e) attribution's `3.6486e-01` by `0.05` — and that row's
`±0.05 r_b` box was `T_horiz` **and** `T_vert` together. Decomposed at the same
conditions:

| perturbation | loss in margin | ÷ 0.05 |
|---|---|---|
| `T_horiz` **and** `T_vert` (the quoted row) | `3.648645e-01` | 7.30 |
| `T_horiz` only | `7.990445e-02` | **1.60** |
| `T_vert` only | `2.809069e-01` | 5.62 |

**The figure was dominated by the vertical term, and vertical displacement is
`z_home` — already a swept axis, not build error of the kind the discriminator is
for.** Corrected in the handoff's score-function line and marked withdrawn there.

**The probe range survives.** Across `0.0025–0.01 r_b` the sensitivity holds to
within ~1% of linear; `0.05 r_b` is 16% off. `0.005–0.01 r_b` meets the linearity
requirement — that half was right.

**(F) The appendix's 1.635° and `envelope.py`'s 1.636° differ only in `g`.**
`9.81` gives `1.6356`, `9.80665` gives `1.6361`. Not a disagreement between the
framings — both need the same `0.2 m/s²`. The appendix figure predates the
constant. `1e-3` degrees, no consequence, recorded so it does not read as a
discrepancy next time someone checks.

## Not in this category, but named because the question was asked

- **`0.024`** (the pre-refinement grid gap in the one crossing) is the difference
  of two numbers the diagnostic prints (`0.29932 − 0.275`), not an independent
  claim.
- **`729` / `4374` / `81` / `486` / `1.4e9` / `11 GB`** appear only inside quoted
  *superseded* text. The ledger script now reproduces the 729 and 81 rows anyway.
- **`1.635°`, `100 mm`, `200 mm/s`** in the appendix predate this session; the
  arrest side now has code, the choice of 100 mm and 200 mm/s still does not —
  they are inputs, not results.
