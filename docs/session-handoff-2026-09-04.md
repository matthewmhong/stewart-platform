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

*Both diagnostics ran at a **single** `z_home`, the one the flat-arm datum supplied.
That datum is dropped, `z_home` is a swept axis again, and the evidence has to be
re-run across it — open item 11. The branch conclusion is not withdrawn; its
evidence base is narrower than it looked.*

**Closest approach to the branch-merge boundary is −5.7e-3 of `C`.** The provisional
6° envelope grazes its own limit. Not a margin. See open item 1.

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

**The envelope is four axes, not six.** Yaw and vertical translation dropped: a
circular plate is axisymmetric so yaw does not move the ball, and the control law
commands tilt, not height. `x, y, tilt magnitude, tilt azimuth` — and the last two
are a disc, better swept as magnitude and azimuth than as a square grid. At 3
points that is 81 poses, 486 `w` evaluations per `J`, against §9's unwritten 729
and 4374. Factor of nine on the inner loop.

**The tilt target does not scale with the kinematics.** Arresting a rolling ball
needs `(5/7) g sin(tilt) = v²/2L`, so with plate size scaled by `k` the required
tilt goes as `1/k` while translations go as `k`. The 1.635° is welded to 100 mm of
plate and 200 mm/s of ball. Absolute scale therefore re-enters *upstream* of the
sweep through the envelope, not only downstream in scoring — this was missing from
the 2026-09-03 list. Either `r_b` is fixed before the tilt target, or the sweep
re-runs per `r_b`.

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
273M *full* evaluations. This path is **~7.6M full** (15625 × 486) **plus ~1.4e9
cheap scan evaluations** — `w = amp·cos(delta - phase)`, then `C`, then margin
(15625 × 180 × 486). Both land near 10¹⁰ flops. **Compute is a wash, not a win.**
The decision to drop the datum rests on the gauge argument in derivation §8, not on
speed, and should not be re-argued on speed.

New constraint that comes with it: 1.4e9 floats is **~11 GB**, so the scan **must
chunk over candidates**. Also a harness requirement.

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

**1. Tilt target [Y].** One number, margin over 1.635° argued from latency and
disturbance rather than picked. Note the 45 mm-at-150 ms figure leans on 300 mm/s,
which was withdrawn 2026-09-03 as invented — it needs its own basis. This now gates
the sweep harness for two reasons: the `1/k` scaling ties it to `r_b`, and the
branch check came back grazing the boundary at a provisional 6°. Do not shrink the
envelope to clear the boundary — that fits the specification to the geometry.

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

**8. `notation.md` §9's envelope slots.** Deliberately left untouched in the last
documentation pass. Writing them is a decision about what the control law commands,
worth doing as its own edit.

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

**11. The `-` branch evidence was gathered at one `z_home`** *(opened 2026-09-04 by
the reversal)*. The 4365-pose diagnostic and the 720-step precession loop both ran
at the single `z_home` the flat-arm datum supplied. That datum is dropped and
`z_home` is a swept axis again, so **both must be re-run across the restored axis**.
The `-` branch is not withdrawn — its evidence base is narrower than it read.

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
- **Tilt target [Y].** Open item 1. Gates the harness.
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
- **Sweep harness [CC].** *(Rewritten 2026-09-04, superseding the same-day line
  "Five normalised axes … feasibility guard for the missing-`z_home` region".)*
  **Six** normalised axes including `z_home/r_b` — whose bracket does not exist yet,
  so this line is blocked on it as well as on the tilt target. Four-axis envelope.
  `delta` inner tune maximising the **normalised** margin `(C_i - |P_i|)/C_i`,
  maximin over legs and poses, seeded at `delta*` **with no basin assumption**.
  Coarse at 5 points → 15625 candidates. Two requirements, not options:
  - **Precompute feasibility bracket** from `C_i ∈ [sqrt(|L_i|² - amp_i²), |L_i|]`,
    available because `P_i` is `delta`-free. Drop candidates with `|P_i| ≥ |L_i|` at
    any (pose, leg) before scanning; scan for score only when
    `|P_i| ≤ sqrt(|L_i|² - amp_i²)` everywhere; only the middle pays 180 steps.
  - **Chunk the scan over candidates.** 1.4e9 floats is ~11 GB and will not be held.

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
