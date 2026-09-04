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

**Closest approach to the branch-merge boundary is −5.7e-3 of `C`.** The provisional
6° envelope grazes its own limit. Not a margin. See open item 1.

**`z_home` from the assembly datum.** Requiring `alpha_i = 0` at home (§8's
arms-flat datum) gives `(z_home - h_p)² = d² - |q_i^flat - h_i^flat|²`, identical
for all six legs by D₃. So `z_home` is determined, not free, and the sweep is five
axes not six — 3125 candidates rather than 15625. Cost: home stops being a design
choice. 8 of 432 grid combinations produce no valid `z_home`, all at `d/r_b = 0.8`,
so the harness needs a feasibility guard before evaluating.

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

Home is in the envelope so `J(delta*) ≥ 0` and this is not §8's minimiser — but it
is an exact seed for the inner 1-D tune and puts it in the right basin. **Untested
against the actual envelope minimiser.**

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

Five axes after normalising by `r_b`: `beta`, `r_p/r_b`, `beta_p`, `a/r_b`,
`d/r_b`. `z_home/r_b` removed by the assembly datum. `mu` and the pairing
eliminated by argument; `h_p` fixed by components; `delta` an inner 1-D tune.

At 5 points per axis: 3125 candidates. Per candidate, worst case 180 `delta` trials
× 486 `w` evaluations = 87,480. Total ≈ 273 million `w` evaluations, each a couple
of dot products — minutes to low hours vectorised. Coarse-first was about learning
whether the feasible set is empty, not about compute. At 10 points per axis it is
64× and the picture changes.

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

**4. Two derivations from 2026-09-04 are unchallenged.** The `z_home` assembly
datum and the `delta*` closed form. Both are the assistant's, not his. One removes
a sweep axis, the other cheapens the inner loop — which is exactly why neither
should ride on assertion. Same treatment the `mu` gauge argument got.

**5. `N_i > 0` should be a test, not a docstring.** The fixed `-` branch rests on
it. Sweep the envelope, assert `min(N_i) > 0`, report the margin. Cheap, and it
fails loudly if the envelope ever grows past where the argument holds.

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

---

## Plan, in order

Tags: **[Y]** his, **[CC]** Claude Code, **[bg]** background.

- **Hardware pull — dispatched 2026-09-04 [CC/bg], result not yet read.** Servo
  travel and deadband; horn lengths (the discrete set `a` lives in); ball-joint
  housing OD (fixes `beta_p`'s outer bound); rod stock. Also worth checking on
  return: the joints' angular misalignment range, which nothing in the derivation
  has looked at — at the envelope extremes the rod makes some angle with the plate
  normal, and past the joint's range it binds regardless of the kinematics.
- **Tilt target [Y].** Open item 1. Gates the harness.
- **`fk()` [Y] then [CC].** Open item 2.
- **Round-trip gate [Y]. Hard stop.**
- **Score function — cap at 2 h [Y].** Weighted scalar, weights stated, plus a
  sensitivity check that the ranking survives perturbing them. Not a Pareto front.
  Feasibility is pass/fail before scoring: valid `z_home`, envelope reachable,
  `N_i > 0`. Discriminators available: `J(delta*)`; boundary margin
  `min(C-|P|)/C`, which is the one that buys tolerance for a rod cut long;
  servo travel used; tilt resolution. The last two are meaningless until the
  hardware pull returns.
- **Sweep harness [CC].** Five normalised axes, `delta` inner seeded at `delta*`,
  four-axis envelope, feasibility guard for the missing-`z_home` region. Coarse at
  5 points.
- **Run, expect empty, widen [Y].** An empty return is information about the tilt
  target or the envelope, not a bug. Then refine, pick `r_b` from torque and build
  volume, then CAD, then order.

**Not on the critical path.** Grübler/Kutzbach returns 6, he knows it returns 6,
the sweep never calls it. Batch with the document work.

---

## For the document session, later

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
| 2026-09-04 | 0 | — | `stage1`, `legs`, `arm_tips`, `ik` implemented; §7 table re-established with a control row proving the distance check alone is insufficient. Branch fixed as `-`, justified by `N > 0` under horizontal shafts. `z_home` determined by the assembly datum, removing a sweep axis. Envelope reduced to four axes. Symmetry of the six `w_i` derived as a D₃ stabiliser argument and verified with a negative control; closed form for `delta*` at home. Open item 3 closed: `beta_p = beta` is not a rank hole — the rank-3 result was a proxy concurrency artifact, disproved with true rod lines. Tilt target shown not to be scale invariant, moving absolute scale upstream of the sweep. Three provenance failures found where documented results had no code behind them. |
