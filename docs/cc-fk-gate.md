# `fk()` and the round-trip gate

**2026-09-05, revised 2026-09-07.** The gate **PASSES**. Worst round-trip
translation error `1.85e-13 mm`, worst geodesic rotation error `7.82e-14 deg`,
over **14 436 poses** across **4 geometries**, seeded HOME everywhere, with **0**
non-convergences and **0** different-mode returns.

*Revision 2026-09-07, three items, none of which change the verdict.* The
rotation metric was **fixed rather than annotated** — the `arccos` trace form
has a `~8.5e-7 deg` floor, so the rotation error above was previously reported
as `2.09e-06 deg` when it is really `7.82e-14` (§5). The `(R, T)` pose order was
**resolved as the repo convention** and written into `notation.md` rather than
carried as a per-function deviation (§2.2). And the near-mode case was
**reclassified as a scoring finding**, which promotes the characteristic length
from provisional to blocking (§6).

    python -m stewart.diagnostics.roundtrip     # exit 0 on a pass, 1 on a fail

Code: `fk`, `fk_solve`, `fk_jacobian`, `fk_residual`, `exp_so3`,
`FKNotConverged` in `stewart/kinematics.py`; the gate in
`stewart/diagnostics/roundtrip.py`.

The four settled decisions were taken as given and are not revisited: unsquared
residual, rotation vector in the solve, a convergence criterion that has to
carry its own conversion factor, and a HOME seed. No rotation convention was
introduced; derivation §6 is untouched. `ik`, `stage1`, `legs`, `arm_tips`, `w`
and every geometry function are unmodified.

---

## 1. What the gate is measuring, and what it is not

`pose -> ik -> six angles -> fk -> pose`, compared without a rotation
convention: `|T_fk - T_cmd|` in mm, and the geodesic angle in degrees. The
angle is `|log_so3(R_cmd^T R_fk)|`, **not** the `arccos` trace form originally
specified — same quantity, but the `arccos` form has a `~8.5e-7 deg` floor that
sat seven orders above the real error. Changed 2026-09-07; see §5.

A pass licenses the downstream plan — `ik` and `fk` are mutually consistent to
the arithmetic floor, so a scoring function built on them is measuring the
mechanism rather than a solver bug. It does **not** validate the derivation
against the physical machine. `ik` and `fk` share `stage1`, `legs` and
`arm_tips`, so a sign error common to both would round-trip perfectly. What
guards that is the §7 table in `test_kinematics.py`, which checks those three
against hand-computed values rather than against each other. That table still
passes (re-run: all rows, max residual `6.1e-16`).

---

## 2. Decisions taken under latitude

### 2.1 Scale: the fixtures run at `r_b = 100 mm`

Every geometry in the project is normalised to `r_b = 1`. A tolerance that is a
physical length in millimetres cannot live at that scale, so the gate runs the
same geometries at `r_b = 100 mm`. The kinematics is homogeneous of degree one
in length and the envelope is purely angular (`envelope.py`), so this is a
change of units and nothing else — every angle, iteration count and relative
quantity is identical to the `r_b = 1` fixture.

`100 mm` is a **fixture, not a decision**. Absolute scale is still upstream of
the sweep. It is the placeholder from the 2026-09-03 handoff, and it puts `a`
and `d` in the `[15, 60]` and `[105, 155]` mm ranges the derivation quotes.

### 2.2 Pose order is `(R, T)` — now the recorded repo convention

The prompt specified `(T, R)`; `fk` takes and returns `(R, T)`. **Resolved
2026-09-07 in favour of `(R, T)`, and written into `notation.md`'s Conventions
block so it stops being a per-function question.** It is not a deviation to be
tolerated but the repo's existing rule: `stage1(geom, R, T)`,
`legs(geom, R, T)`, `w(geom, R, T)` and `ik(geom, R, T)` all take orientation
first, and `stewart/roundtrip.py` unpacks `R_hat, T_hat = fk(...)`. The mnemonic
recorded with it: in `q_i = T + R p_i`, `R` is the operator and `T` the offset,
so `R` binds first in every signature even though it is written second in the
formula.

`fk` returns the pose. `fk_solve` returns the full record — residual, iteration
count, LM steps, `cond`, `sigma_min`, SO(3) drift, residual history — and is
what the gate calls.

### 2.3 Solver internals

Newton on the square 6×6 system, analytic Jacobian, backtracking line search on
`max_i |f_i|` (halving, up to 2^-30 of the step), Levenberg-Marquardt only where
Newton fails to reduce that norm.

`R` is carried as a matrix and updated on the **left**, `R <- exp([omega]_x) R`,
with `omega` a local increment re-zeroed each iteration and never accumulated.
The iterate therefore never leaves SO(3) — measured drift `max |R^T R - I|`
before the final polar projection is `2.2e-16` — and no rotation convention is
involved anywhere.

**LM damping is `lam * diag(J^T J)`, not `lam * I`.** `lam * I` would add a
millimetre to a radian, which needs exactly the characteristic length this work
must not pick. Marquardt's scaled form is invariant to column scaling and so
sidesteps the choice.

### 2.4 Grids

Coarse → medium → fine at `(n_magnitude, n_azimuth, n_zhome)` = `(5, 7, 3)`,
`(9, 13, 5)`, `(17, 25, 9)` — 87, 525 and 3609 poses per geometry. The coarse
level is `envelope.py`'s own harness grid, so the gate's first level is directly
comparable with the rest of the project. `z_home` is sampled in the **interior**
of each bracket, at fractions `k/(n+1)` of the span, because the bracket ends
are exactly where `ik` sits on its reach boundary.

The `z_home` bracket per geometry is recomputed here at 0.25 mm resolution
(`0.0025 r_b`, ten times finer than `zhome_bracket.py`), holding `delta` at the
geometry's own value rather than asking whether *some* `delta` works — the gate
runs a specific geometry, not a candidate family.

### 2.5 Geometries

| | β | β_p | δ | r_p | a | d | h_p | bracket (mm) |
|---|---|---|---|---|---|---|---|---|
| **A** | 20 | 40 | 40 | 85 | 20 | 120 | 10 | `[120.000, 129.000]` |
| **C** | 10 | 55 | 137 | 60 | 35 | 110 | 20 | `[57.500, 134.750]` |
| **E** | 50 | 25 | 30 | 60 | 35 | 80 | 10 | `[21.000, 101.500]` |
| **F** | 45 | 15 | 20 | 110 | 25 | 140 | 5 | `[128.750, 138.750]` |

A is `branch_check.py`'s fixture A verbatim (×100). C is `azimuth_symmetry.py`'s
fixture C (×100).

**`azimuth_symmetry.py`'s fixtures B and D have empty brackets** — at their own
`delta` and at every `delta` on a 5° scan. They cannot carry a round trip, so E
and F replace them, picked for the corners rather than for comfort: E has
`a/d = 0.44` against A's `0.17` (long arm, short rod) and the widest bracket
found; F has `r_p > r_b` and the narrowest non-empty bracket found, 10 mm. B and
D are kept in the fixture list so the bracket table reports them as empty rather
than silently omitting them.

---

## 3. The Jacobian: verified, and the handed-over form was right

The claim, flagged unverified:

    df_i/dT      =  e_i^T
    df_i/domega  = -e_i^T [R p_i]_x        e_i = (q_i - h_i) / |q_i - h_i|

under the **left** perturbation `R -> exp([omega]_x) R`. Central-differenced
against the same left perturbation — differencing a right perturbation against a
left-perturbation formula would report a mismatch that is a property of the test
— at 348 samples spanning 4 geometries, 3 `z_home` values per geometry and 29
poses, all perturbed off the solution.

| h (mm) | h (rad) | worst entrywise rel. | median | worst Frobenius |
|---|---|---|---|---|
| 5e-2 | 5e-3 | 1.609e-05 | 1.050e-05 | 1.076e-05 |
| 5e-3 | 5e-4 | 1.609e-07 | 1.050e-07 | 1.076e-07 |
| **5e-4** | **5e-5** | **9.552e-08** | 1.168e-09 | 1.078e-09 |
| 5e-5 | 5e-6 | 1.568e-06 | 3.948e-09 | 3.916e-11 |
| 5e-6 | 5e-7 | 1.142e-05 | 3.720e-08 | 3.869e-10 |

**Worst relative deviation at the bottom of the U: `9.55e-08`** — the requested
~1e-7. The sign and the perturbation side **as handed over are correct. Nothing
was changed.**

Two notes on the measurement. The minimum sits at `h = 5e-4 mm`, an order above
the textbook `h ~ (eps*scale)^(1/3) ≈ 5e-5`, because the residual `|q - h| - d`
cancels two ~120 mm quantities and so carries far more round-off than its own
magnitude suggests. And the entrywise measure is the strict one but is punishing
on entries near zero for geometric reasons; the Frobenius ratio is reported
beside it and is `1.08e-09` at the same step.

### Negative control

A finite-difference check that cannot tell the candidate forms apart proves
nothing about the one it endorses. The three forms actually at risk:

| rotation block | worst entrywise rel. dev. | |
|---|---|---|
| `-e^T [R p]_x` — left, as handed over | **9.781e-10** | endorsed |
| `+e^T [R p]_x` — left, sign flipped | 2.000e+00 | rejected |
| `-e^T R [p]_x` — right perturbation | 4.307e-01 | rejected |
| `+e^T R [p]_x` — right, sign flipped | 1.974e+00 | rejected |

Nine orders of separation, so the endorsement is worth something. Worth
recording *why* the two suspicions had to be tested separately: since
`[R p]_x = R [p]_x R^T`, the right-perturbation form differs from the left one
by an `R^T` and not merely by a sign — "flip the sign if it does not match"
would not have found the bug had there been one.

---

## 4. The convergence criterion, and a mistake worth recording

### The tolerance is an ACCEPTANCE test, not a stopping rule

This is the substantive finding of the work, and the first version of the gate
got it wrong.

`FK_TOL_MM = 1e-9 mm`, with the reasoning at the definition in
`stewart/kinematics.py`. The residual is unsquared, `f_i = |q_i - h_i| - d`, so
it is literally the gap rod `i` fails to close and the tolerance is a length
with no conversion factor. Floor: at the working scale a double resolves a
length to `eps * 120 ≈ 2.7e-14 mm`, and the measured attainable residual floor
is `1.4e-14` to `7.1e-14 mm`; `1e-9` sits four to five decades above it, so
acceptance never fails for a numerical reason. Ceiling: any tolerance the
hardware can mean is micrometres at best, six decades above.

**But those two bounds are not what makes the value defensible.** With the
tolerance used as the *stopping* rule — stop on the first iterate with
`max|f| <= tol` — the required demonstration cannot ever pass, for a structural
reason. Measured, on the coarse sweep:

| tol (mm) | worst \|dT\| (mm) |
|---|---|
| 1e-9 | 9.744e-10 |
| 1e-10 | 1.291e-10 |
| 1e-11 | 1.661e-11 |
| 1e-12 | 1.844e-12 |
| 1e-13 | 1.268e-13 |

A straight line: one decade of pose error per decade of tolerance. The cause is
that ~40% of poses (48 of 117 on fixture A at `tol = 1e-9`) halt on the first
iterate that crosses the tolerance while the rest run on to `~1e-14`, and a
**max over a grid** is taken over exactly the poses that stopped early. So the
worst residual sits just under the tolerance whatever the tolerance is. A gate
built that way reports its own stopping rule back to itself.

The fix is not to tune the number. **`fk` stops on stagnation** — iterate until
no step, Newton or backtracked-Newton or LM, can reduce `max_i |f_i|` — and
applies `tol` **once, to the converged residual, as acceptance.** Re-measured:

| tol (mm) | worst \|dT\| (mm) | worst residual | max it | raised |
|---|---|---|---|---|
| 1e-8 | 1.2681e-13 | 2.842e-14 | 7 | 0 |
| **1e-9** | **1.2681e-13** | **2.842e-14** | 7 | 0 |
| 1e-10 | 1.2681e-13 | 2.842e-14 | 7 | 0 |
| 1e-11 | 1.2681e-13 | 2.842e-14 | 7 | 0 |
| 1e-12 | 1.2681e-13 | 2.842e-14 | 7 | 0 |
| 1e-13 | 1.2681e-13 | 2.842e-14 | 7 | 0 |
| 1e-14 | — | — | 7 | 234 ← below the arithmetic floor |
| 1e-15 | — | — | 7 | 234 ← below the arithmetic floor |

**Tightening tenfold moves the worst pose error by exactly zero, at every step
across six decades.** The tolerance is not what limits accuracy; the arithmetic
floor is. Both numbers requested are in the table, and the range over which the
answer is tolerance-independent is bounded on both sides — it ends at `1e-14`,
where acceptance starts failing because the tolerance is below the floor.

The iteration cap is `FK_MAX_ITER = 100`; observed max is 8. Non-convergence
raises `FKNotConverged`, carrying the residual, the iteration count, the
tolerance and a reason (`cap` / `stalled` / `singular`). Nothing is returned as
a best effort.

### `cond(J)` and the conversion factor

`J`'s translation columns are dimensionless and its rotation columns are mm/rad,
so no singular value means anything until a characteristic length names the
exchange rate. **That is the same undecided choice `notation.md` §12 records for
the scoring conditioning measure, and it is not settled here.** All numbers are
quoted at `char_len = r_b` and flagged **PROVISIONAL**. The size of the
dependence, worst `cond` over the coarse grid at each fixture's mid-bracket:

| fixture | ℓ = r_b | ℓ = r_p | ℓ = d | ℓ = a |
|---|---|---|---|---|
| A | 4.183 | 4.175 | 4.20 | 12.81 |
| C | 3.712 | 3.075 | 4.05 | 3.737 |
| E | 7.397 | 6.285 | 6.695 | 7.185 |
| F | 6.317 | 6.365 | 6.54 | 19.51 |

Nothing downstream may take one of these columns as "the" conditioning until §12
is closed. The spread is a factor of 3 on F.

The conversion factor is **checked per pose, not asserted**:

    ||dx|| <= sqrt(6) * (max_i |f_i| + eta) / sigma_min,   dx = (dT, r_b * omega)

Worst measured/bound over all 14 436 poses: **0.074**. Three corrections were
needed to make that check honest, and each was wrong first:

- taking the worst residual and the worst `1/sigma_min` from *different* poses
  and multiplying them is not a bound on anything;
- the bound is on `||f||_2`, so `max_i |f_i|` must be inflated by `sqrt(6)`;
- `eta = 8 eps (d + |T|)`, the round-off in **evaluating** a residual, must be
  added. At the floor `eta` is the same size as the residual itself. Omitting
  it, the check reads **1.535 — i.e. violated**, which is a statement about
  floating-point residual evaluation and not about the solver.

---

## 5. The gate result

Tolerance `1e-9 mm`. Seed HOME (`R = I`, `T = (0,0,z_home)`) everywhere.
`cond` and `1/sigma_min` at `char_len = r_b`, PROVISIONAL.

| fixture | level | poses | ok | other mode | non-conv | `ik` unreach | worst \|dT\| mm | worst ang deg | worst resid | max cond |
|---|---|---|---|---|---|---|---|---|---|---|
| A | coarse | 87 | 87 | 0 | 0 | 0 | 5.847e-14 | 2.412e-14 | 1.421e-14 | 4.261 |
| A | medium | 525 | 525 | 0 | 0 | 0 | 6.829e-14 | 3.522e-14 | 2.842e-14 | 4.295 |
| A | **fine** | 3609 | 3609 | 0 | 0 | 0 | **8.595e-14** | 3.932e-14 | 2.842e-14 | 4.327 |
| C | fine | 3609 | 3609 | 0 | 0 | 0 | 4.876e-14 | **7.820e-14** | 2.842e-14 | 6.221 |
| E | fine | 3609 | 3609 | 0 | 0 | 0 | 1.068e-13 | 7.052e-14 | 2.842e-14 | 9.518 |
| F | fine | 3609 | 3609 | 0 | 0 | 0 | **1.853e-13** | 6.257e-14 | 2.842e-14 | 6.381 |

**Worst case overall: `1.853e-13 mm` on fixture F, at azimuth 62.50°, tilt
1.974°, `z_home = 133.750 mm`.** Worst rotation error `7.820e-14 deg` on fixture
C at azimuth 57.50°, tilt 8.555°, `z_home = 65.225 mm`.

### The rotation metric was changed, 2026-09-07

The gate originally used `arccos((tr(R_cmd^T R_fk) - 1)/2)` as specified. **That
formula has a floor of `~8.5e-7 deg` and it was being read as the error.** Near
the identity `tr(R) = 3 - theta^2 + O(theta^4)`, so the trace carries the angle
only at second order: an `O(eps)` error in the trace becomes `O(sqrt(eps))` in
the angle, and `arccos(1 - eps)` returns `~sqrt(2 eps) = 8.5e-7 deg` for a
rotation that is the identity to machine precision. No rotation smaller than
that is resolvable by it at all.

The metric is now `|log_so3(R_cmd^T R_fk)|` — the magnitude of the rotation
vector taking one to the other. Still convention-free: an axis and an angle, no
ordered sequence of elementary rotations. The antisymmetric part it is built
from is **linear** in the angle, so it has no floor.

**Same quantity, not an easier one**, and that is measured rather than asserted
(`check_metric_agreement`, section (1a) of the report). Both forms are compared
against a *known* angle built by `exp_so3` about a random axis, so neither
formula defines the answer:

| true angle (rad) | `\|log\|` rel. err | `arccos` rel. err | `\|log\|` abs. (deg) | `arccos` abs. (deg) |
|---|---|---|---|---|
| 3.11 | 3.19e-16 | 9.57e-15 | 5.68e-14 | 1.71e-12 |
| 1e-2 | 8.91e-15 | 4.59e-12 | 5.11e-15 | 2.63e-12 |
| 1e-4 | 4.87e-13 | 1.75e-07 | 2.79e-15 | 1.00e-09 |
| 1e-6 | 4.60e-11 | 1.07e-03 | 2.63e-15 | 6.11e-08 |
| 1e-8 | 6.97e-09 | 3.22e+00 | 3.99e-15 | 1.84e-06 |
| 1e-12 | 5.35e-05 | 4.22e+04 | 3.07e-15 | 2.42e-06 |
| 1e-15 | 6.23e-02 | 2.98e+07 | 3.57e-15 | 1.71e-06 |

For angles `>= 1e-2 rad`, where `arccos` is still well conditioned, the two
differ from each other by at most `4.6e-12` relative and each matches the known
angle to the same — **one quantity**. Below `~1e-6 rad` the `arccos` absolute
error stops improving and saturates at `2.4e-06 deg` against the predicted
`1.2e-06`; `|log_so3|`'s absolute error tracks the angle all the way down,
`~3e-15 deg` throughout. Its *relative* error does grow below `1e-12 rad`
(`6.2e-02` at `1e-15 rad`), and that is the rotation matrix's limit rather than
the formula's — a double cannot hold a `1e-15 rad` rotation in its entries to
full relative precision. Five decades below anything measured here.

The effect on the gate: worst rotation error was reported as `2.091e-06 deg`
and is actually **`7.820e-14 deg`**, `2.7e7` times lower. The gate passed either
way. `stewart/roundtrip.py`'s `_geodesic_deg` carried the same defect and was
fixed with it — leaving one copy floored while fixing the other would be worse
than either.

### Does the worst case move under refinement?

**Yes, and the movement means nothing here.**

| fixture | coarse | medium | fine | growth |
|---|---|---|---|---|
| A | 5.847e-14 | 6.829e-14 | 8.595e-14 | 1.47× |
| C | 4.279e-14 | 4.633e-14 | 4.876e-14 | 1.14× |
| E | 8.297e-14 | 8.192e-14 | 1.068e-13 | 1.29× |
| F | 1.268e-13 | 1.580e-13 | 1.853e-13 | 1.46× |

It rises monotonically on three fixtures of four, so the coarse grid flattered
the worst case in the unsafe direction a fourth time. But the quantity that
moves is at the arithmetic floor: a 40× refinement in pose count buys 1.5× in
the max, which is the signature of drawing more samples from a fixed round-off
distribution, not of finding a real worst case.

**What this does not establish.** This gate is not where the grid-refinement
worry gets settled. Its worst case is `1e-13 mm` everywhere, so it has no
dynamic range in which a genuine worst-case pose could show itself, and a gate
that passes by twelve orders of magnitude cannot distinguish a well-sampled
envelope from a badly sampled one. **Open item 12 stands unaffected** — it
belongs to the scoring function, where the quantity being maximised is O(1) and
the sampling genuinely decides the answer.

### Iteration counts and the basin

| fixture | n | min | median | mean | max | histogram | LM accepted | home-seed failures |
|---|---|---|---|---|---|---|---|---|
| A | 3600 | 4 | 5 | 5.04 | 7 | 4:535 5:2407 6:644 7:14 | 36 | **0** |
| C | 3600 | 4 | 5 | 5.28 | 8 | 4:204 5:2299 6:971 7:124 8:2 | 60 | **0** |
| E | 3600 | 4 | 5 | 5.43 | 8 | 4:194 5:1786 6:1484 7:134 8:2 | 246 | **0** |
| F | 3600 | 4 | 5 | 5.22 | 7 | 4:540 5:1746 6:1278 7:36 | 289 | **0** |

**The home seed converges at every pose in the envelope, on all four
geometries.** No basin finding to report.

The LM counts are of **accepted** steps, 1–8% of solves. Counting *attempts*
instead reported LM on 96% of solves, because every converged solve ends with
one iteration where nothing reduces the residual — that is the stagnation
stopping rule firing at the arithmetic floor, not a conditioning event. The
first version of this report would have buried a real conditioning problem in
that noise.

**The home pose is excluded from the iteration statistics and counted
separately** (9 per fixture, one per `z_home`). At tilt 0 the seed *is* the
commanded pose, so the residual is zero before the first step. That is precisely
the "seeded at the truth proves nothing" case, and it is unavoidable — home is
both the seed and a member of the envelope. It can only be named and excluded.

---

## 6. Assembly modes

**From the home seed: 0 different-mode returns, across every fixture, every grid
level and every `z_home`.** Reported as its own line so a zero is a claim rather
than a silence.

The classifier cuts at `1e-4 mm`. It is not a tuned parameter — the `|dT|`
distribution over all 14 436 converged solves spans `1e-17` to `1e-12 mm` and
stops there, so any cut in the nine empty decades above classifies identically.

### The modes are real, and the classifier does fire

A zero is weak on its own: it is equally consistent with "the home seed is
reliable" and with "the classifier never fires". So the same six angles were
re-solved from **400 random seeds** per fixture — rotations up to a half turn,
translations up to `0.8 r_b` off — and the converged roots clustered. Fixture A,
`z_home = 124.5 mm`, tilt 7.897°, azimuth 60°:

| seeds | residual | \|dT\| mm | ang deg | min q_z | tilt | verdict |
|---|---|---|---|---|---|---|
| 162 | 1.421e-14 | 0.000 | 0.000 | 103.094 | 7.897 | **commanded mode** |
| 37 | 1.421e-14 | 176.697 | 80.684 | -111.670 | 85.771 | anchors below the plate |
| 30 | 1.421e-14 | 114.758 | 81.362 | -48.942 | 86.457 | anchors below the plate |
| 27 | 1.421e-14 | 124.159 | 83.325 | -58.348 | 80.442 | anchors below the plate |
| 26 | 1.421e-14 | 103.591 | 79.511 | -37.590 | 72.116 | anchors below the plate |
| 22 | 1.421e-14 | 167.619 | 83.875 | -100.086 | 81.073 | anchors below the plate |
| 14 | 1.421e-14 | 180.139 | 80.470 | -107.051 | 73.128 | anchors below the plate |
| 5 | 1.421e-14 | 225.450 | 0.462 | -122.052 | 7.640 | anchors below the plate |

323 of the 400 seeds converged. The rest raised — roughly two thirds `stalled`
and one third `cap`, with residuals from `3.2` to `83 mm`, i.e. nowhere near the
tolerance. That is the intended behaviour for a seed far outside any basin, and
it is worth noting that no failure landed *near* the tolerance: the converged and
failed populations are separated by fourteen orders of magnitude, so acceptance
at `1e-9 mm` is not a close call anywhere.

**8 distinct roots per fixture, 32 across the four.** Every one converges to the
same arithmetic floor as the commanded root, so **the residual genuinely cannot
tell them apart** — which is exactly why the gate classifies on pose distance
and reports the count separately.

Of the 28 non-commanded roots, **1** has all anchors above the base plate
(fixture C: `min q_z = +3.27 mm`, 32.5 mm and 47.7° from the commanded pose) and
**0** are inside the tilt envelope — that one is tilted 54.8°, five times the
10.529° limit. The other 27 put platform anchors below the base plate,
`min q_z < 0`, and so violate the `N_i > 0` condition `ik`'s fixed minus branch
rests on: real solutions of the rod equations, not poses the machine can hold
this way up.

**Plausibility here is a weak test and should not be read as more.** Two things
are checked — anchors above the plate, and tilt inside the envelope. Ball-joint
angular travel and rod/arm interference are not modelled anywhere in this
project yet, and both would rule out roots that pass these two. A root called
upright is one this code cannot rule out, not one shown to be assemblable.

### One real nearby-mode case, on `smoke_geometry`

`demo.py` now reports a non-zero round trip on the "yaw +90°" row:
`pos err 0.3163 mm`, `ang err 0.7296 deg`. **This is a different assembly mode,
not a failure**, and it is worth recording because it is the case the gate's
fixtures do not produce:

- residual at the recovered root `1.42e-14 mm` — fully converged;
- `ik` at the recovered pose returns **the same six angles** to `2.0e-13 deg`,
  so it is a genuine second root of the same forward problem;
- `cond(J) = 9045`, `sigma_min = 1.94e-4` at `char_len = r_b = 90` — against
  `cond` 4–10 and `sigma_min` 0.25–0.5 on the real fixtures.

Four orders worse conditioning is why two roots sit `0.32 mm` apart there at
all: `smoke_geometry` is near a singularity where assembly modes coalesce. It is
explicitly not a design, and the offset seed of `stewart/roundtrip.py` is enough
to cross between the two.

### This is a SCORING finding, not an FK finding

*Reclassified 2026-09-07; recorded in `notation.md` §12.* The FK solver is doing
the right thing here — it finds a real root of the system it was given, and
reports a residual at the arithmetic floor because the root is real. Nothing
about `fk` needs to change.

The `ik` clause is what makes it a design problem. **Two poses `0.32 mm` and
`0.73°` apart produce identical servo commands** — the same six angles to
`2.0e-13 deg`. They are therefore two poses the machine cannot distinguish from
its own commands. No control law, no calibration and no better solver separates
them, because the information is not in the command; which of the two the
platform assembles into is decided by history and by which side of the fold it
was on. On the four real fixtures, at `cond` 4–10, the alternative modes are
30–260 mm away and every one is either below the base plate or tilted 50–86°, so
the question does not arise.

**Nothing in the current feasibility set excludes the bad case.** `|P_i| <= C_i`
and `N_i > 0` are both satisfied throughout on `smoke_geometry`, so a candidate
of that kind would pass feasibility and reach the ranking stage. That argues for
`cond(J)` (or `sigma_min`) as a score **discriminator with a floor** — a hard
reject below a threshold, not a term to be traded off — alongside the reach
margin `(C - |P|)/C` and the translation sensitivity already recorded. A
weighted sum would let a candidate buy its way past a fold with margin
elsewhere.

And that is a **concrete reason to settle the characteristic length rather than
carry it as provisional**: the threshold is a `cond` value, `cond` is not defined
until the length is, so the length now decides where a *reject line* sits and not
merely how a ranking sorts. The measured spread over four candidate lengths is a
factor of 3 even at the benign values of the real fixtures (§4).

Left open, and not answered by any of the above: whether the right quantity is
`cond(J)` of the FK Jacobian at all, or the wrench matrix's `sigma_min`, or the
mode separation measured directly. The FK Jacobian is what this gate happened to
have in hand — a reason to look, not a reason to adopt it.

---

## 7. Stale or wrong, found on the way, not in the prompt

1. **`fk`'s stub docstring listed `Unreachable` as its exception.** It does not
   apply. `Unreachable` is a statement about one leg's servo circle failing to
   reach a *commanded* anchor — an inverse-kinematics condition. In forward
   kinematics the anchors are what is being solved for and no such per-leg test
   exists; the failure mode is non-convergence. Replaced with
   `FKNotConverged(RuntimeError)`, and the reason recorded in the docstring.
   **`Unreachable` is unchanged and `ik` still raises it.**

2. **`README.md` §"Stubs, in the order they pay off" is now fully discharged** —
   items 1–6, `platform_ring` through `fk`, are all written. The file still
   presents them as to-do. `CLAUDE.md`'s layout table still shows
   `stewart/kinematics.py` as "`stage1`, `legs`, `arm_tips`, `ik`, `fk` STUB
   (signatures + docstrings only)", which has been wrong since 2026-09-04 for
   the first four and is now wrong for all five.

3. **`demo.py` prints a stale banner**: "round trip (ik/fk stubbed -> every row
   reports 'not implemented')". Nothing is stubbed; three of its four rows now
   report `ik Unreachable` and one reports a real round trip. The banner should
   go, and the `ik Unreachable` rows are correct — `smoke_geometry` is not a
   layout and its home pose is genuinely unreachable.

4. **`azimuth_symmetry.py`'s fixture `z_home` column is infeasible on three of
   its four rows.** Measured against the bracket recomputed here:

   | fixture | listed `z_home` | bracket (`r_b`) | |
   |---|---|---|---|
   | A | 0.95 | `[1.2000, 1.2900]` | below its own reach floor |
   | B | 1.05 | empty at every `delta` | no feasible `z_home` exists |
   | C | 0.90 | `[0.5750, 1.3475]` | **inside** |
   | D | 1.00 | empty at every `delta` | no feasible `z_home` exists |

   This does not invalidate `azimuth_symmetry.py` — it tests a symmetry of
   `w_i`, which is defined whether or not `ik` can solve there, and does not
   call `ik` at all. But the `z_home` column reads like a feasible operating
   point and is not one on three rows of four.

5. ~~**The `arccos` geodesic metric has a `~8.5e-7 deg` floor.**~~ **FIXED
   2026-09-07**, in both places, rather than annotated. The metric is now
   `|log_so3(R_cmd^T R_fk)|` in `stewart/kinematics.py`, used by the gate and by
   `stewart/roundtrip.py`'s `_geodesic_deg`; `check_metric_agreement` measures
   that it is the same quantity. See §5. Nothing else in the project uses the
   trace form — checked.

6. **The rotation convention (§6) stays open and is untouched**, deliberately:
   the rotation-vector parameterisation needs none, and `log_so3` is its inverse,
   so neither introduces one.

7. **Open item 12 on grid refinement stands.** §5 explains why this gate cannot
   bear on it: no dynamic range.

8. **The characteristic length (§12) is no longer merely provisional — it is
   blocking.** *Changed 2026-09-07.* It was listed here as an open item carried
   PROVISIONAL at `r_b`. §6 upgrades it: if `cond(J)` becomes a score
   discriminator with a floor, the length decides where a reject line sits, not
   just how a ranking sorts. Recorded in `notation.md` §12 with the measurement
   behind it.
