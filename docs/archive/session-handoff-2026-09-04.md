# Handoff — Stewart platform, Phase 0, after 2026-09-04

Paste this into a new chat. Attach `stewart-ik-derivation.md`, `notation.md` and
`phase-0-design-log.md`. `how-to-help.md` was retired 2026-09-04; the rules below
are the working set.

---

## Rules, short form

- Blunt about wasted time and money. Concise. No preambles.
- **Code lives with Claude Code in VS Code, which has the full repo.** Do not write
  scaffolding in chat. If something needs building, say so and write the Claude Code
  prompt. Chat is for derivation, review and challenge. You may ask for the output
  of a check; you do not produce the file that runs it.
- Watch for a fully-formed derivation arriving in a register unlike his own.
- Name files and functions. Do not refer to numbered list items across sessions.
- **[Y] items are not delegated.** This was tested twice on 2026-09-04 — the branch
  rule and the tilt target — and holding the line was right both times.

---

## State

### Code, as of 2026-09-04

Implemented and verified in `stewart/kinematics.py`: `stage1`, `legs`, `arm_tips`,
`w`, `ik`, `Unreachable`. All arrays are **`(3,6)`**, not `(6,3)` — `CLAUDE.md`
calls this non-negotiable and a transposed shape runs without error while silently
applying the rotation transposed.

`stage1(geom, R, T) -> q` per derivation §3. This overrode the stub's docstring,
which had `stage1` returning the branch-independent coefficients `(A,B,P,C)`; those
moved inside `ik`, their only consumer. `_v(geom)` computes `v_i = n_i × u_i`
rather than assuming `z`, so a canted shaft cannot silently pass.

`geometry.py`: `h_p` threaded through `platform_ring` and `make_geometry`, both
defaulting to 0. The `beta_p > 0` guard stays, with its reason corrected to
ball-joint housing diameter.

Verification table re-established in `test_kinematics.py`, plain numpy, exit 0:

| Check | Residual |
|---|---|
| `stage1(R=I, T=0) == p` | 0 |
| `Rz(90)` sends `(10,0,0)` → `(0,10,0)` | 6.1e-16 |
| `\|arm_tips(0) - b\| == a` | 5.6e-17 |
| `arm_tips(0) == b + a·u` | 0 |
| *control:* `b + a·n` also passes row 3 | 5.6e-17 |

The control row is new and makes §7's point testable rather than only documented.

`w()` was repointed from its inlined `L_i` to `legs()`. Regression over 4000 random
poses: bitwise identical, and both 2026-09-03 checks reproduce every digit.

### Derived and verified 2026-09-04

**Branch fixed as `-`.** `alpha_i = phi - arccos(P/C)`, all six legs, no elbow
parameter. Justification: `N = L_i·z` is the anchor height above the base plate,
positive under horizontal shafts (§8), so `phi` sits in the upper half-plane at
every reachable pose and only the `-` root reaches `alpha = 0` at the datum.
**Reopens if the shafts are canted**, along with the rest of §8.

Diagnostic behind it: over 4365 poses (tilt ≤ 6°, yaw ±10°, translation ±0.05·r_b)
the `-` root stays defined and continuous except at 4 corner samples where the `+`
root fails too — workspace boundary, not a branch problem. A 720-step precession
loop showed no flip. §5.5's round trip recovers every angle to 6e-15; legs 4 and 5
land on `+` there because that test places six independent anchors, so `N_i` is
free — a rigid plate cannot produce it.

*Both diagnostics ran at a **single** `z_home`, the one the flat-arm datum supplied,
and on a provisional 6° envelope. **Re-run 2026-09-05** across the restored `z_home`
axis at the settled 10.529°: the branch **stands** — `-` at every feasible `z_home`,
**0** branch-flip step outliers over a full-circle 1440-step precession, loop
closure `≤ 1.8e-15`, and `ik()` agreeing with the `-` root to `8.9e-16`. The
justification's phrase "reaches `alpha = 0` at the datum" should now be read as
"reaches `alpha ≈ 0` at home", since the flat-arm datum it named is dropped; the
`N_i > 0` argument it rests on is untouched. Open item 11, discharged.*

**~~Closest approach to the branch-merge boundary is −5.7e-3 of `C`.~~
SUPERSEDED 2026-09-05.** The figure was measured on a **6°** envelope that also
carried yaw ±10° and translation ±0.05·r_b, at a `z_home` the dropped flat-arm
datum supplied. It reproduces exactly (`−5.713988e-03`) and it no longer
describes anything the machine is being asked to do.

Recomputed on the settled envelope, same geometry, same height:
**`+1.550398e-01`**. Tuning `z_home` alone, still at fixture A's untuned
`delta = 40°`: **`+2.276643e-01`** at `z_home/r_b = 1.2375`.

Individual costs, each a difference of two measurements on **one** pose set:
yaw ±10° costs `1.8033e-01`, translation ±0.05·r_b costs `3.6486e-01`, tilt
6° → 10.529° costs `3.6147e-01`. Translation and the tilt increase cost
comparable amounts; yaw about half.

**The `−5.7e-3` was not driven by tilt** — it was yaw and translation together,
and the settled envelope commands neither. That conclusion stands: it rests on
gaps of order `1e-1`.

> **WITHDRAWN 2026-09-05 — the four-row attribution table.** A table stood here
> presenting those costs as a decomposition against a baseline
> `tilt 6°, yaw 0, T = 0` = `+5.1489e-01`, with a fourth row calling
> yaw + translation near-additive. **The decomposition does not close.** Baseline
> minus the tilt cost gives `+1.534192e-01` against a settled figure of
> `+1.550398e-01` — a **`1.62e-3`** discrepancy, ~1% of the quantity the table
> existed to explain and three orders above every other residual in the package.
>
> Cause, confirmed rather than guessed: the rows and the settled figure are
> **maximins over different pose sets**. The magnitude grids are identical; only
> the azimuth sampling differs (15° for the rows, 10° for the settled figure), so
> subtracting one from the other was never a valid operation. Withdrawn, not
> repaired — the conclusion above never depended on it.

**Quote the new number with its envelope attached, or neither** — and see open
item 12 before quoting `+1.550398e-01` itself.
`stewart/diagnostics/branch_envelope.py`.

**`z_home = z_flat` is DROPPED.** *(Decision 2026-09-04, later the same day. This
**supersedes** the earlier 2026-09-04 paragraph in this section which read "`z_home`
is determined, not free, and the sweep is five axes not six — 3125 candidates rather
than 15625", and which required a feasibility guard for a missing-`z_home` region.
That paragraph is withdrawn in full; see withdrawals 1 and 2 below. Where the two
readings conflict, **this one is current**.)*

`z_flat` — the plate height at which the arms lie flat with the rods attached —
**remains an assembly datum only**. It is no longer identified with the home pose.
`z_home` **returns as an outer sweep axis**. Its closed form, its leg-independence
and its residuals are written up in derivation §8.1; it keeps its value as a datum,
it just does not set home.

What the reversal propagates:

- the sweep is **six** normalised axes, not five: `beta`, `r_p/r_b`, `beta_p`,
  `a/r_b`, `d/r_b`, `z_home/r_b`;
- **15625** candidates at 5 points per axis, not 3125;
- `delta ∈ [0°, 180°)` is **restored as sufficient** — see the recorded note in
  derivation §8, which is what would lift that gauge if the datum ever came back;
- **`z_home`'s range is UNDECIDED and nothing currently supplies one.** Lower end
  from `N_i > 0`, upper end from reach `|P| ≤ C`. See open item 5 and
  `notation.md` §12.

The decision rests on the gauge argument (derivation §8's recorded note), **not on
compute** — see the ledger below, where the two paths are a wash.

**`alpha` at home is one scalar shared by all six legs.** Not a new measurement — it
follows from §8.1's two residuals: at home `M_i` and `|g_i|²` are leg-independent
and `N_i = z_home - h_p` is common, so `C_i`, `P_i`, `phi_i` and the home angle are
common too. Two uses. All six servos read the same angle at home, which is a by-eye
build check that **recovers at home what `z_flat` was going to provide**; and a
non-zero home angle is absorbable by horn mounting angle, so servo mid-travel is
bought with the spline instead of with the datum. Written up as derivation §8.2.

**Symmetry structure of the six `w_i`.** The legs' relation at a pose is the
stabiliser of `(T,R)` in D₃, and the `w` structure is its orbit structure. Verified
signed, not in magnitude, with a negative control:

- Home, full D₃: `w_i = s_i·k`, with
  `k = r_p sin(beta_p - beta - delta) + r_b sin delta`.
- Yaw only: `w_1 = w_3 = w_5`, `w_2 = w_4 = w_6`, exactly.
- Mirror at 0° (tilt about world y, `T` in the mirror plane), `sigma = (12)(36)(45)`,
  normal flips: `w_1 = -w_2`, `w_3 = -w_6`, `w_4 = -w_5`.
- Negative control (tilt about x, generic `T`): every identity O(1). Passes are not
  passing for the wrong reason.

This replaces the withdrawn "legs stop being related the moment `R ≠ I`".

**Closed form for `delta*`**, the value zeroing all six `w_i` at home:

```
delta* = atan2(-r_p sin A, r_b - r_p cos A) mod 180,   A = beta_p - beta
```

*(Corrected 2026-09-04, superseding the sentence that stood here earlier the same
day — "an exact seed for the inner 1-D tune and puts it in the right basin". The
basin claim is **withdrawn**; see withdrawal 3.)*

Stated correctly: `delta*` **zeroes `w_i` at home**, which is the pointwise quantity
both candidate objectives are monotone in, **at one pose**. It is **not** the
maximiser of the margin aggregate and **not** the minimiser of `J`. It is a seed,
with no basin claim attached.

The open test is whether the margin's maximiser stays within a bracket of `delta*`.
That is a question about **aggregation**, not about the function — see the objective
paragraph below.

**Objective for the inner `delta` tune, revised.** `J(delta) = max|w_i|` is
superseded by the **normalised reach margin** `(C_i - |P_i|) / C_i`, maximin over
legs and envelope poses. The division by `C_i` is **required**: `C` and `P` both
carry length, so the raw difference scales with `k` and breaks the normalised sweep,
where candidates differing only in `r_b` must score identically. It is the same form
the branch check already reports, which is where the `-5.7e-3` above came from.

The relation between the two, precisely: `delta` does not appear in `L_i`, so `P_i`
is `delta`-free and only `C_i = sqrt(|L_i|² - w_i²)` moves. At a fixed leg and fixed
pose the margin is therefore **strictly decreasing in `|w_i|`** — maximising the
margin *is* minimising `|w_i|`, exactly. The divergence is entirely in the
**aggregation**: `J` is a minimax over `|w_i|`, the margin is a maximin over
`(C_i - |P_i|)/C_i`, and because `|L_i|` varies across legs and poses the leg with
the largest `|w_i|` is generally **not** the leg with the smallest margin. Different
worst cases, different minimisers, related quantities.

**Open item 3 is closed. `beta_p = beta` is not a rank hole.** The rank-3 result
was an artifact: with the proxy screw axis `q_i - b_i`, at `p_i = c·b_i` all six
lines pass through `(0,0,-(z_home-h_p)/(c-1))` — one point, independent of `i`, so
rank 3 by concurrency. Verified: proxy lines concurrent to 5e-16. With true rod
lines `q_i - h_i` the arm breaks it. `sigma_min` at `e = 0` is 0.185 not 1e-17,
full rank 6, **monotonically increasing through `e = 0`** — not a dip, not a local
minimum, not distinguished at all. Scales linearly in `a`: `sigma_min/a ≈ 0.93`,
drifting 2% over a 14× span, so the rank is bought entirely by the arm and vanishes
as `a → 0` exactly where the proxy becomes exact.

The affine claim came from a 6-UPS intuition, where the leg genuinely is the `b→q`
line. It does not transfer to a 6-RSS.

**`beta_p → 0` is the 3-6 Stewart platform**, a named non-degenerate architecture —
three anchor points each carrying two rods to two *different* shafts, six distinct
lines. No rank drop. So **both rings' endpoint exclusions are hardware** — housing
diameter and servo body — not degeneracy, and the two range arguments collapse to
one as anticipated. `beta_p`'s admissible set is a plain interval; only the outer
bound is unknown, and the hardware pull fetches it.

**~~The envelope is four axes, not six.~~ TWO axes — superseded 2026-09-05.**
The 2026-09-04 reduction dropped yaw and vertical translation and left
`x, y, tilt magnitude, tilt azimuth`, 81 poses and 486 `w` evaluations at 3
points. The envelope settled 2026-09-05 (`notation.md` §9) sets `dxy = 0` as
well — the control law commands tilt only — so **`x` and `y` go too**. What
remains is **tilt magnitude and tilt azimuth**, and nothing else.

Two consequences, both larger than the axis count:

- **The envelope carries no length dimension.** It is purely angular, so it is
  invariant under scaling every length by `k` and does not have to be scaled
  alongside the geometry. `notation.md` §9's note about translations forcing the
  envelope to scale is **deleted**, not softened.
- **Azimuth needs a 60° window, not the full circle** — periodic in 120° by D₃
  and mirror-symmetric. But the window is **`[30°, 90°]`**, not `[0°, 60°]`; see
  the findings under open item 11. At 5 magnitudes × 7 azimuths that is
  **29 poses, 174 `w` evaluations** per objective evaluation, against the 486 of
  2026-09-04 and §9's superseded 4374.

**The tilt target does not scale with the kinematics.** *(Conclusion unchanged;
**mechanism corrected 2026-09-05**, and the sign is the opposite of what stood
here. The superseded sentence read: "Arresting a rolling ball needs
`(5/7) g sin(tilt) = v²/2L`, so with plate size scaled by `k` the required tilt
goes as `1/k` while translations go as `k`." That is right **under an arrest
framing** — fixed entry speed `v`, stopping distance `L` proportional to `k` —
which is not the framing now adopted.)*

Under the **recovery framing** settled 2026-09-05 (`notation.md` §9) the ball is
returned from a displacement `x0` in a fixed time `tau`, and `x0` is a fraction of
the plate, so `x0` scales with `k`. Then

```
acc = 4 x0 / tau²   ~  k          and   sin(tilt) = 7 acc / (5 g)  ~  k
```

**Required tilt GROWS with plate size.** Opposite sign to the arrest result. A
bigger plate does not buy a gentler tilt; at fixed recovery time it costs a
steeper one.

The conclusion survives untouched: the tilt target is **not** scale-invariant, so
absolute scale re-enters *upstream* of the sweep, and either `r_b` is fixed before
the tilt target or the sweep re-runs per `r_b`. Only the direction of the
dependence changes — and it changes the intuition, which is why the old sentence
is quoted above rather than deleted.

The 1.635° remains welded to 100 mm of plate and 200 mm/s of ball, and it is now
an **arrest-framing minimum**, superseded as a design basis by the recovery
numbers. Derivation appendix carries the same label.

### Sweep budget as it stands

*(Rewritten 2026-09-04, **superseding** the same-day "Five axes after normalising by
`r_b` … 3125 candidates … ≈ 273 million `w` evaluations" that stood here. `z_home/r_b`
is **not** removed by the assembly datum — the datum is dropped. This section is the
current one.)*

**Six** axes after normalising by `r_b`: `beta`, `r_p/r_b`, `beta_p`, `a/r_b`,
`d/r_b`, **`z_home/r_b`**. `mu` and the pairing eliminated by argument; `h_p` fixed
by components; `delta` an inner 1-D tune. `z_home/r_b`'s **range is undecided** and
nothing supplies one yet, so the axis exists before its bracket does.

At 5 points per axis: **15625** candidates, not 3125.

**Precompute feasibility bracket** — available only because `P_i` is `delta`-free.
With `z_home` fixed per candidate, `P_i` is a per-pose constant, and without
scanning `delta` at all

```
C_i  ∈  [ sqrt(|L_i|² - amp_i²),  |L_i| ] ,        amp_i = sqrt(A_i² + B_i²)
```

which gives two **exact** delta-free tests:

- `|P_i| ≥ |L_i|` for any (pose, leg) → **infeasible at every `delta`**. Drop the
  candidate before the scan.
- `|P_i| ≤ sqrt(|L_i|² - amp_i²)` for all (pose, leg) → **feasible at every
  `delta`**. Scan for score only.

Only the undecided middle pays the 180 steps. This is a **requirement** on the
harness, not an optimisation to consider.

**Compute ledger, correcting an overstatement made in session.** The datum path was
273M *full* evaluations. *(Updated 2026-09-05: the per-pose count fell from 486
to 174 when the envelope lost `x` and `y` and azimuth lost the full circle, so
the figures below are smaller than the ones first written here — ~7.6M full and
~1.4e9 cheap. The conclusion is unchanged.)* This path is **~2.7M full**
(15625 × 174) **plus ~4.9e8 cheap scan evaluations** —
`w = amp·cos(delta - phase)`, then `C`, then margin (15625 × 180 × 174). Both
land near 10⁹–10¹⁰ flops. **Compute is a wash, not a win.**
The decision to drop the datum rests on the gauge argument in derivation §8, not on
speed, and should not be re-argued on speed.

New constraint that comes with it: 4.9e8 floats is **~3.9 GB**, so the scan
**must still chunk over candidates**. Also a harness requirement. *(Was ~11 GB at
486 poses per candidate; the smaller envelope shrinks it but does not remove it.)*

Coarse-first was about learning whether the feasible set is empty, not about compute.
At 10 points per axis it is 64× and the picture changes.

### Withdrawn or corrected 2026-09-04

- The "excluded hole" at `beta_p = beta`. Disproved above. Remove the language
  wherever it appears.
- `beta ∈ (0°,60°)` and `beta_p`'s range as degeneracy bounds. They are hardware
  bounds. Same interval, different reason, and the reason decides what fetches the
  number.
- The claimed secondary near-degeneracy at fixture B `beta_p = 10` "for
  `delta = 95`". The proxy wrench uses `q_i - b_i`, which contains no `n_i` and
  therefore no `delta`. It was the affine dip at `e = -2` seen from the other sweep.
- Every conditioning magnitude from the proxy: the 1° width, `cond > 18`, the
  π/2 slope. None of it survives.

Three further withdrawals, all reversing claims made **earlier the same day** and
recorded here rather than deleted, so the contradiction is legible:

1. **"8 of 432 grid combinations produce no valid `z_home`, all at `d/r_b = 0.8`, so
   the harness needs a feasibility guard." WITHDRAWN.** An artifact of holding
   `delta = 40`. **All 8 are feasible on `[180°, 360°)`. No geometry among them is
   infeasible.** The sentence is struck, and so is the feasibility-guard requirement
   it created — which appeared both in the `z_home` paragraph above and in the
   sweep-harness plan line below. Both are now rewritten. Run:
   `stewart/diagnostics/zhome_datum.py`, check B; original grid in
   `stewart/diagnostics/branch_check.py`, block `[1c]`.
2. **"`z_home` is determined, not free, and the sweep is five axes not six."
   WITHDRAWN**, per the decision recorded in the `z_home` paragraph above. Six axes,
   15625 candidates, `delta ∈ [0°, 180°)` restored as sufficient, `z_home`'s range
   undecided.
3. **`delta*` as an "exact seed … in the right basin". WITHDRAWN.** Correct
   statement: `delta*` zeroes `w_i` at home, which is the pointwise quantity both
   candidate objectives are monotone in, **at one pose**. It is not the maximiser of
   the margin aggregate and not the minimiser of `J`. **It is a seed with no basin
   claim.** The open test is whether the margin's maximiser stays within a bracket
   of it — a question about aggregation, not about the function.

---

## Open

**1. ~~Tilt target [Y].~~ CLOSED 2026-09-05 — see `notation.md` §9.** The whole
envelope is specified there: `dxy = dz = 0`, `yaw = 0`, tilt limit **10.529°**
from a bang-bang recovery model, with the **6.558°** bare requirement recorded
alongside it and the difference named as the latency margin. It no longer gates
the harness.

Three things carried out of it rather than closed with it:

- **`tau_L = 150 ms` is still provisional** and still leans on the withdrawn
  300 mm/s figure. It needs sensor frame interval plus servo step response — on
  the hardware pull. It carries 3.97 of the 10.53 degrees.
- **Sensitivity travels with the number.** Tilt goes as `1/tau²`; a 10% error in
  `tau` moves required `sin(tilt)` by ~20%. `tau` is the least-defended input and
  the one the answer is most sensitive to.
- **The boundary-grazing worry is resolved, and not by shrinking anything.**
  Measured 2026-09-05 on fixture A: the `-5.7e-3` was bought by the **yaw and
  translation** terms, not by tilt. Removing them (the control law commands
  neither) clears the boundary outright; raising tilt 6° → 10.529° spends most of
  what that buys back, and the net is `+1.55e-1`. The envelope got **larger** in
  the one axis the control law uses and empty in two it does not. Nothing was
  fitted to the geometry. See open item 11 and
  `stewart/diagnostics/branch_envelope.py`.

**2. `fk()` [Y for the criterion, CC for the wiring].** Four decisions, all his:

- Residual: `|q_i - h_i| - d` or the squared form. Same zero, different
  conditioning and different units.
- Convergence criterion. **The failure mode is a gate that passes while wrong.** A
  residual tolerance says the legs are consistent, not that the pose is right; it
  must be tight enough that round-trip pose error is dominated by something other
  than early stopping.
- Parameterisation of `R` in the solve — three angles, quaternion with a norm
  constraint, or rotation vector. Interacts with §6's open rotation convention.
- Seeding. The gate has the commanded pose available, but seeding from the answer
  gives a solver that works only when you already know the result.

The harness already reaches the stub in `roundtrip`, so the wiring is small.

**3. Round-trip gate — hard stop.** Nothing below is worth writing until it passes.

**4. Two derivations from 2026-09-04 — half-settled.** *(Updated later the same
day.)* The item asked for both to be challenged rather than ridden on assertion.
**Both were challenged and both moved** — which is the outcome the item was asking
for, and worth recording as such.

- **`delta*` half — resolved**, as withdrawal 3 states. The basin claim is gone;
  `delta*` is a seed that zeroes `w_i` at one pose. What is left open is not the
  claim but a **test**: does the margin's maximiser stay within a bracket of
  `delta*`? That is an aggregation question, and it belongs with the harness.
- **`z_home` assembly-datum half — resolved by reversal.** The datum is dropped,
  `z_home` is a swept axis again, and the axis-removal it bought is given back.
  What is left open is its **range**, which nothing currently supplies — open
  item 5 and `notation.md` §12.

Not closed outright, because each half leaves something live. Neither is still
unchallenged.

**5. `N_i > 0` now sets `z_home`'s LOWER bracket.** *(Rewritten 2026-09-04,
superseding the same-day version of this item, which asked only for a
docstring-to-test upgrade. That upgrade is still wanted; it is no longer the point
of the item.)* With `z_home` restored as a swept axis, `N_i > 0` is what stops the
axis from below, so it is a **range constraint** before it is a test.

`N_i` at home is `z_home - h_p`; tilt drops the low anchors by roughly
`r_p sin(tilt)`. The upper end comes from reach, `|P_i| ≤ C_i`. Neither end is a
number yet — see `notation.md` §12.

The test still stands on its own: sweep the envelope, assert `min(N_i) > 0`, report
the margin. Cheap, and it fails loudly if the envelope ever grows past where the
fixed `-` branch argument holds.

**DISCHARGED 2026-09-05**, both halves, in
`stewart/diagnostics/zhome_bracket.py` and `branch_envelope.py`.

- **As a bracket, in closed form:** `z_home > r_p sin(tilt) + h_p cos(tilt)`
  = `0.182733 r_p + 0.983163 h_p` at 10.529°. `delta`-free, `a`-free, `d`-free,
  because `v_i = z` exactly and `b_i·z = 0` make `N_i = q_i·z`. Verified against
  `make_geometry`: `min N_i` at the bound is 0 to **5.6e-17**.
- **As a test:** `min(N_i) > 0` passes at every `z_home` scanned, margin
  `+0.896 r_b` at the bottom of the scan.

Two findings that were not in the item.

1. **The back-of-envelope bound is not the bound.** `z_home - h_p > r_p sin(tilt)`
   drops the `cos(tilt)` and overstates the requirement by `h_p(1 - cos tilt)` —
   `1.68e-3 r_b` at `h_p = 0.1 r_b`. Conservative, so it errs safe; still not the
   bound, and the closed form is free.
2. **`N_i > 0` is load-bearing at the edges, though it never shapes the interior.**
   *(Corrected 2026-09-05. The claim first written here — "`N_i > 0` is not the
   binding constraint at this tilt", on the strength of **0 of 363** candidates
   with a non-empty bracket — rested on a **survivorship sample**: it ranged only
   over candidates where the constraints did **not** cross, so it structurally
   could not have contained a counterexample. A sample that could, does.)*

   The `0 of 363` is still true and still means it never sets the lower end of a
   surviving bracket. Attributing the **177 empties** as well:

   - **176 of 177** are reach failing on its own, at every `z_home` and every
     `delta`.
   - **1 of 177** is the two constraints crossing — reach ceiling **below** the
     `N_i` floor: `beta = 10°, beta_p = 55°, r_p/r_b = 1.10, a/r_b = 0.10,
     d/r_b = 0.80`. The grid gap was `0.024` on a `0.025` step, inside one step, so
     it was refined by bisection to `1e-9`: ceiling `0.2920624`, floor `0.29932`,
     gap **`+7.26e-03`**. **Real, not a sampling artifact.**
   - **6 candidates** have a *non-contiguous reach set* — a spurious low component
     at `z_home ≈ 0.025–0.125 r_b`, the platform essentially on the base plate,
     geometrically reachable and physically nonsense. In **6 of 6** it lies
     entirely below the `N_i` floor, so `N_i > 0` removes it. That is what keeps
     the feasible set an interval, and it is why "0 non-contiguous feasible sets"
     and "6 non-contiguous reach sets" are both true — they measure different sets.

   Correct form: `N_i > 0` does not shape the interior of the feasible set and is
   not why most candidates fail, but it **closes the bracket outright in 1 of 177
   and keeps the set connected in 6**. It cannot be dropped.

   Note the categories are fixed by the constraints' shapes: `N_i > 0` is
   **one-sided**, a floor; reach is a **two-sided interval**. There is no `N_i`
   ceiling, so "reach floor above the `N_i` ceiling" is not a case that can occur.

And a trap for the harness: this is a **continuum** bound. A discrete pose grid
reports it satisfied slightly *before* it truly is (`+5.9e-4` on the 29-pose grid
at `beta_p = 25°`), so the harness must take the lower bracket from the **formula**,
not from its own poses.

**6. `h_p`'s symbol.** `notation.md` §11 flags the clash with `h_i`, the arm tip,
and proposes `c_p`. The quantity is settled; the letter is not. Three call sites —
`platform_ring`, `make_geometry`, `kinematics.py`. Cheapest to rename now.

**7. Provenance. Three instances, and it is a process problem.**

- The rank guard normalising by a fixed 100: not in the repo. Locate or withdraw
  the corollary. The requirement itself is right and follows from scale invariance.
- The §7 verification table was recorded as passing while all three functions
  raised `NotImplementedError`. Now re-established, but the pattern is that
  verified results reach the documents without the code that produced them
  reaching the repo.
- The four-axis envelope existed only in a chat log until this handoff.

**Partly discharged 2026-09-04, and the wording matters.** `branch_check.py` — the
script behind "8 of 432" and the `-5.7e-3` boundary margin — was **recovered from a
session scratchpad, not from the repo**, and committed as
`stewart/diagnostics/branch_check.py` (verbatim but for one import line, recorded in
its docstring). Its grid also sits in `stewart/diagnostics/zhome_datum.py`.
**Recovered is not the same as never lost:** the script existed only outside version
control for a day, and had the scratchpad gone, the numbers in this handoff would
have had nothing behind them. The process problem is unchanged.

**8. ~~`notation.md` §9's envelope slots.~~ CLOSED 2026-09-05.** Every slot is
filled: `dxy = 0`, `dz = 0`, `yaw = 0`, tilt limit **10.529°** with the **6.558°**
bare requirement recorded beside it, and a grid row carrying the real numbers
(**29 poses, 174 `w` evaluations**) rather than the stale `3^6 = 729 / 4374`. The
`k`-scaling note is deleted: with `dxy = dz = 0` the envelope is purely angular and
carries no length dimension at all.

**9. Unchanged from `stewart-ik-derivation.md` §6.** Rotation convention for `R`.
Numerical FK. Round trip. The branch rule row can now be struck.

**10. The pairing argument is still not closed.** 720 bijections, 6 survivors, one
D₃ orbit. Two readings unchallenged — "the rotations are the same machine
relabeled" (test: leg 1's angular span `gamma_tau(1) - theta_1` across the six
survivors must agree as a multiset, at `beta = 20, beta_p = 40`), and "the
reflections give the mirror-image machine". Also `mu ∈ {0,180}` was derived under
the identity pairing while the pairing result ranged over all 720; each froze what
the other varied. Only affects how the result is stated — the gauge argument kills
`mu` either way.

**11. ~~The `-` branch evidence was gathered at one `z_home`.~~ RE-RUN, and the
branch STANDS** *(opened 2026-09-04 by the reversal, discharged 2026-09-05)*.
`stewart/diagnostics/branch_envelope.py`, fixture A, tilt 10.529°, `z_home` swept
across and past its feasible bracket, precession checked over the **full** circle
at 0.25° (1440 steps — the 60° window is a scoring shortcut, continuity is a claim
about the real trajectory):

| Check | Result |
|---|---|
| `min(N_i) > 0` over envelope, every `z_home` | passes, worst `+0.896 r_b` |
| `z_home` values with the envelope fully reachable | 8 of 15, `[1.2000, 1.2875] r_b` |
| branch reaching `alpha ≈ 0` at home | `-`, at every feasible `z_home` |
| branch-flip step outliers in precession | **0** |
| loop closure `max_i \|alpha_i(360°) - alpha_i(0°)\|` | `≤ 1.8e-15` |
| `max \|ik() - alpha_minus\|` | `8.9e-16` |

**A finding the item did not ask for, and the one to read first.** The claim that
tilt azimuth need only be swept over `[0°, 60°]` is **right about the width and
wrong about the position**. Measured over the full circle with a negative control
(`stewart/diagnostics/azimuth_symmetry.py`): period-120 holds to `1.0e-13` and the
`psi → 180 - psi` mirror to `5.6e-14`, but the `psi → -psi` mirror that `[0°, 60°]`
would need **fails at `2.3e-1`**. The mirror lines in azimuth sit at **30 + 60k**,
not 0 + 60k, because a reflection maps a tilt axis at `psi` to one at
`2m + 180 - psi` — tilting about an axis *in* a mirror plane reflects to the
*opposite* tilt. `[0°, 60°]` is symmetric about its own centre: it double-counts
what it touches and **misses the orbit `{75°, 105°}` entirely**. The window is
**`[30°, 90°]`**. `notation.md` §9 and §10 carry the correction.

**12. The 29-pose harness grid overstates worst-case margins** *(found 2026-09-05
while withdrawing the attribution table; not a task item)*. Same fixture, same
height, varying **only** the azimuth sampling:

| azimuth sampling | `min (C-\|P\|)/C` |
|---|---|
| 15° over `[0°, 360°)` | `+1.534192e-01` |
| 15° over `[30°, 90°]` | `+1.534192e-01` |
| **10° over `[30°, 90°]` — the harness grid, and the settled figure** | **`+1.550398e-01`** |
| 0.25° over `[30°, 90°]` — reference | `+1.531859e-01` |

7 azimuths at 10° miss the worst azimuth by more than 24 at 15° happen to, so the
harness grid is **optimistic by ~1.9e-3** and so is the `+1.550398e-01` quoted
above. **`+1.53e-1` is the defensible number.**

This is the **same failure mode** as the `N_i > 0` bound in open item 5 — a
discrete pose grid flattering a worst case — and it is now two for two. Neither
is large, and both run in the unsafe direction. Open question, not settled here:
whether the scoring grid needs refining, or whether a scoring grid is allowed to
be optimistic as long as **feasibility** is decided by closed forms and the
ranking is what the grid produces. That is a decision about the score function,
so it belongs with it. `stewart/diagnostics/branch_envelope.py`.

---

## Plan, in order

Tags: **[Y]** his, **[CC]** Claude Code, **[bg]** background.

- **Hardware pull — dispatched 2026-09-04 [CC/bg], result not yet read.** Servo
  travel and deadband; horn lengths (the discrete set `a` lives in); ball-joint
  housing OD (fixes `beta_p`'s outer bound); rod stock; **servo horn spline tooth
  count / mounting-angle resolution** *(added 2026-09-04 by the reversal — it
  decides whether a non-zero home angle costs anything, since the shared home angle
  is absorbable by horn mounting angle; see the `alpha`-at-home paragraph and
  derivation §8.2)*. Also worth checking on
  return: the joints' angular misalignment range, which nothing in the derivation
  has looked at — at the envelope extremes the rod makes some angle with the plate
  normal, and past the joint's range it binds regardless of the kinematics.
- ~~**Tilt target [Y].** Open item 1. Gates the harness.~~ **Done 2026-09-05** —
  `notation.md` §9. No longer gates anything. `tau_L` goes on the hardware pull.
- **`fk()` [Y] then [CC].** Open item 2.
- **Round-trip gate [Y]. Hard stop.**
- **Score function — cap at 2 h [Y].** Weighted scalar, weights stated, plus a
  sensitivity check that the ranking survives perturbing them. Not a Pareto front.
  Feasibility is pass/fail before scoring: envelope reachable, `N_i > 0`. *("Valid
  `z_home`" struck 2026-09-04 — `z_home` is a swept axis, not a derived quantity
  that can fail to exist; see withdrawals 1 and 2.)* Discriminators available: the
  normalised margin `min (C-|P|)/C` at the tuned `delta`, which is the one that buys
  tolerance for a rod cut long, and is now also the inner objective;
  servo travel used; tilt resolution. The last two are meaningless until the
  hardware pull returns.

  **Translation sensitivity — added 2026-09-05.** Margin lost per unit normalised
  displacement, `Δmargin / (dxy/r_b)`, evaluated **once at the tuned `delta`**, per
  candidate.

  - It is a **ranking discriminator, NOT a feasibility test.** The platform is
    displaced from where the model thinks it is by build error; candidates that
    tolerate that are better, but **none are excluded for it**.
  - **Probe `0.005–0.01 r_b`, and it is measured, not estimated**
    (`stewart/diagnostics/sweep_budget.py`). Across `0.0025–0.01 r_b` the
    sensitivity holds to within **~1%** of linear; at `0.05 r_b` it is **16%**
    off and is not a probe. The linearity requirement is met by that range.
  - **The "order 7 units of margin per unit normalised displacement"
    calibration is WITHDRAWN — measured `~1.4`.** *(Corrected 2026-09-05 when
    the figure was given code. It came from dividing the (e) attribution's
    `3.6486e-01` by `0.05`, and that row's `±0.05 r_b` box was `T_horiz` **and**
    `T_vert` together. Decomposed at the same conditions: horizontal `7.99e-02`
    (÷0.05 = **1.60**), vertical `2.81e-01` (÷0.05 = **5.62**). The number was
    dominated by the **vertical** term — and vertical displacement is `z_home`,
    already a swept axis, not build error of the kind this discriminator is
    for.)* The horizontal sensitivity the score function actually wants is
    **~1.4–1.6** at fixture A. The probe range survives; the magnitude behind it
    did not.

  **Recorded so it is not re-derived as a control requirement.** `dxy = 0` is
  correct, and it is a statement about what the control law **commands**. Build
  error is a **perturbation about every commanded pose** — a different object, and
  it belongs in scoring, not in the envelope. A non-zero `dxy` was discussed and
  reversed before dispatch; verified 2026-09-05 that nothing was committed against
  it. The envelope stays two axes, tilt magnitude and tilt azimuth over
  `[30°, 90°]`; 174 `w` evaluations per objective evaluation and the ~3.9 GB
  chunking figure stand.
- **Sweep harness [CC].** *(Rewritten 2026-09-04, superseding the same-day line
  "Five normalised axes … feasibility guard for the missing-`z_home` region".)*
  **Six** normalised axes including `z_home/r_b`. *(Updated 2026-09-05: the bracket
  now exists — lower in closed form, upper per candidate, `notation.md` §12 — and
  the tilt target is settled, so neither blocks this line any more.)*
  **Two-axis envelope**, 29 poses, azimuth over `[30°, 90°]` — **not** `[0°, 60°]`,
  see open item 11.
  `delta` inner tune maximising the **normalised** margin `(C_i - |P_i|)/C_i`,
  maximin over legs and poses, seeded at `delta*` **with no basin assumption**.
  Coarse at 5 points → 15625 candidates. Two requirements, not options:
  - **Precompute feasibility bracket** from `C_i ∈ [sqrt(|L_i|² - amp_i²), |L_i|]`,
    available because `P_i` is `delta`-free. Drop candidates with `|P_i| ≥ |L_i|` at
    any (pose, leg) before scanning; scan for score only when
    `|P_i| ≤ sqrt(|L_i|² - amp_i²)` everywhere; only the middle pays 180 steps.
  - **Chunk the scan over candidates.** 4.9e8 floats is ~3.9 GB and will not be
    held. *(~11 GB before the envelope shrank; still a requirement.)*
  - **Take the `N_i > 0` lower bracket from the closed form, not from the pose
    grid** — the grid reports it satisfied before it is. Open item 5.

  The old missing-`z_home` feasibility guard is **struck**: it answered an artifact
  of holding `delta = 40`.
- **Run, expect empty, widen [Y].** An empty return is information about the tilt
  target or the envelope, not a bug. Then refine, pick `r_b` from torque and build
  volume, then CAD, then order.

**Not on the critical path.** Grübler/Kutzbach returns 6, he knows it returns 6,
the sweep never calls it. Batch with the document work.

---

## For the document session, later

*Status after the 2026-09-04 documentation pass: §8 moved to follow §7 (no
renumbering), §8.1 (`z_flat`) and §8.2 (shared home angle) written, §8's `delta`
tuning paragraph moved to the normalised margin, the `[0°,180°)` gauge note
recorded, §6's branch-rule row struck, `notation.md` §8 and §12 updated, and the
design log's 4 September skeleton laid in for him to write up. The rest of this list
still stands.*

*Status after the 2026-09-05 pass: `notation.md` §9 filled (open item 8), §10 given
the azimuth mirror lines, §12 given `z_home`'s brackets; the handoff's `1/k`
paragraph corrected; the derivation appendix's 1.635° relabelled as the
arrest-framing minimum and the 45 mm bullet struck. `stewart-ik-derivation.md`
still needs its §9 for the platform ring, and §6's rows for the anchor
parameterisation and the rotation convention.*

`stewart-ik-derivation.md` needs a §9 for the platform ring, and §6's rows for the
anchor parameterisation and the rotation convention updating; the branch-rule row
can be struck. §8's `(0°,60°)` needs its reason corrected to hardware. §5.5 should
record the fixed `-` branch and its dependence on horizontal shafts. The symmetry
stabiliser result and the `delta*` closed form both want writing up. `notation.md`
§9's envelope slots per open item 8. `phase-0-design-log.md` needs a 4 September
entry.

Session log row, engagement not yet given:

| Date | Phase | Engagement | Notes |
|---|---|---|---|
| 2026-09-04 | 0 | — | `stage1`, `legs`, `arm_tips`, `ik` implemented; §7 table re-established with a control row proving the distance check alone is insufficient. Branch fixed as `-`, justified by `N > 0` under horizontal shafts — evidence gathered at one `z_home`, so it needs re-running (open item 11). **`z_home = z_flat` tried and dropped the same day**: `z_flat` stays an assembly datum with a verified closed form (leg-independence 9.948e-14, closed form vs library 1.637e-11, round trip 2.220e-16), `z_home` returns as the sixth sweep axis with **no range yet**, and the "8 of 432 no valid `z_home`" claim is withdrawn as an artifact of holding `delta = 40`. Inner `delta` objective moved from `J = max\|w\|` to the normalised margin `(C-\|P\|)/C`; `delta*`'s basin claim withdrawn. **Pose envelope** reduced to four axes — unchanged by the reversal, and not to be confused with the six-axis geometry sweep. Symmetry of the six `w_i` derived as a D₃ stabiliser argument and verified with a negative control; closed form for `delta*` at home. Open item 3 closed: `beta_p = beta` is not a rank hole — the rank-3 result was a proxy concurrency artifact, disproved with true rod lines. Tilt target shown not to be scale invariant, moving absolute scale upstream of the sweep. Three provenance failures found where documented results had no code behind them; `branch_check.py` recovered from scratchpad and committed. |
| 2026-09-05 | 0 | — | Working envelope specified and open item 1 closed: `dxy = dz = 0`, `yaw = 0`, tilt limit **10.529°** from a bang-bang recovery model (`acc = 4x0/tau²`, `sin tilt = 7acc/5g`, `x0 = 50 mm + 30 mm latency drift`, `tau = 0.5 s`), bare requirement **6.558°** recorded alongside; `tau_L = 150 ms` flagged provisional and pushed to the hardware pull, sensitivity `1/tau²` recorded with it. The `1/k` tilt-scaling result **inverted**: under the recovery framing required tilt grows as `k`, not `1/k` — conclusion unchanged, mechanism opposite. Envelope is **two** axes, not four, and being purely angular does not scale with `k` at all. Four diagnostics committed. Azimuth: a 60° window is sufficient but it is **`[30°, 90°]`, not `[0°, 60°]`** — period-120 holds to 1.0e-13, the `180-psi` mirror to 5.6e-14, the `-psi` mirror fails at 2.3e-1; `[0°,60°]` misses the orbit `{75°,105°}`. `z_home` lower bracket in closed form `> r_p sin(tilt) + h_p cos(tilt)`, verified to 5.6e-17; upper bracket per candidate, 363 of 540 non-empty. `N_i > 0` never sets the lower end of a surviving bracket (0 of 363) but **is load-bearing at the edges** — it closes the bracket outright in 1 of the 177 empties (gap +7.26e-03, bisected) and removes a spurious low-`z` reach component in 6, which is what keeps the feasible set an interval. *(The unqualified "never the binding constraint" first written here is corrected: it rested on a survivorship sample.)* The 29-pose scoring grid is **optimistic by ~1.9e-3** on worst-case margin, so the settled figure below is too; +1.53e-1 is the defensible value. The four-row attribution table behind the −5.7e-3 comparison is **withdrawn** — the decomposition does not close (1.62e-3), because its rows are maximins over different azimuth samples; the conclusion it supported stands on the magnitudes. Translation sensitivity added to the score function as a ranking discriminator, not a feasibility test. `-` branch re-run across the restored axis at 10.529°: stands, 0 flips, closure 1.8e-15, `ik()` agrees to 8.9e-16. The **−5.7e-3** boundary margin is superseded — it was bought by yaw and translation, not tilt; on the settled envelope the same geometry gives **+1.55e-1**. Open items 1, 5, 8 and 11 closed; open item 12 raised. |
