# Handoff — Stewart platform, Phase 0, after 2026-09-03

Paste this into a new chat. Attach `how-to-help.md` and `stewart-ik-derivation.md`
as well — they are the authority on rules and notation and this file does not
replace them.

**Do not attach `phase-0-design-log.md` to a working session.** It is 26 KB, it is
a portfolio artefact, and it restates `stewart-ik-derivation.md` in narrative form.
It is billed on every turn. Attach it only when the session's job is writing it.

---

## Rules, short form

Read `how-to-help.md` in full. The parts most likely to be dropped:

- Do not write the IK, the solver, or any geometry selection. Review, ask what he
  thinks first, and when he is wrong say *where* without the fix unless asked twice.
- Vocabulary and numpy idiom get answered directly. Derivation steps get the
  questioning. Socratising a vocabulary gap wastes both of you.
- Blunt about wasted time and money. Concise. No preambles.
- End the session by asking which phase and engagement 1–5.
- **Code lives with Claude Code in VS Code, which has the full repo.** Do not write
  scaffolding in chat. If something needs building, say so and it goes there.
  Chat is for derivation, review and challenge. You may ask for the output of a
  check; you do not produce the file that runs it.
- Watch for a fully-formed derivation arriving in a register unlike his own. It has
  happened once, it carried three substantive errors, and naming it plainly worked.
- Name files and functions. Do not refer to numbered list items across sessions.
- Rating tracks proportion of session spent deriving. Protect the derivation block;
  batch document work to the end.

---

## State

Derived, verified, unchanged: stage 1, the leg vector, the stage 2 closed form,
the base ring, the servo-plane rule, the `|P| > C` reachability test. All in
`stewart-ik-derivation.md`. Implemented and tested: `stage1`, `legs`, `arm_tips`.

Notation added this session: **`w = L_i · n_i`**, the out-of-plane component of the
leg vector. Was briefly written `lambda`; `w` is the name, it is in the derivation
document, use it.

### Settled 2026-09-03 — platform anchor parameterisation

**Symmetry object is the legs, not the points.** A symmetry `g` must permute legs:
one permutation `sigma` with `g(b_i) = b_sigma(i)` *and* `g(p_i) = p_sigma(i)`, the
same `sigma`. Point-set invariance of each ring separately is strictly weaker and
admits configurations that are not symmetric machines. The mirror argument that
forced the `90` in §8 was a leg-correspondence argument, which is why this is the
right object.

**`mu`, the platform ring's rotation offset, is not a degree of freedom.** Gauge
argument, and it is clean: send `p → Rz(mu) p` and `R → R Rz(-mu)` and every world
anchor is identical at every pose. Nothing measurable distinguishes them. `mu`
becomes meaningful only if something in `{P}` pins the frame — a marked front, a
non-axisymmetric mount, a cable exit. A circular plate with a ball rolling on it
pins nothing. **Convention adopted: write the platform ring with no `mu` term.
`R = I` is the orientation in which the platform pair centres line up with the base
pair centres.** This interacts with the open rotation-convention item in §6 of the
derivation document; settle that item and this stays true under either reading.

**`h_p` changes the kinematics and cannot be absorbed into `T`.** With
`p_i = (r_p cos phi_i, r_p sin phi_i, -h_p)`:

```
q_i = T + R p_i = T + R p_i^flat - h_p * (R z_hat)
```

`R z_hat = z_hat` only when `R` fixes the vertical. Under any tilt the offset
acquires a horizontal component, so it enters `w_i = L_i · n_i` (with `n_i`
horizontal), so it changes the tuned `delta`. Pure algebra, no geometry needed.

**Origin of `{P}` goes on the plate top** — or one ball radius above it, if
commanding the ball-centre plane — because that is the surface the control law
reasons about. Putting it in the anchor plane is tempting (`p_i,z = 0`, tidier
algebra) and wrong: it makes a commanded tilt do something other than tilt the
surface the ball is on. Coplanar anchors assumed (flat plate, not stepped or
dished), so all six share one `z` in `{P}`. `h_p` is then plate thickness plus the
joint stack down to the ball-joint centres — a hardware number, present in the
model, **not** a sweep axis.

**The sweep is exactly scale invariant.** Verified numerically: identical `alpha_i`
at `k = 0.25, 1, 3, 17`, max difference at machine epsilon. Reason: `C` and `P`
both scale by `k`, so `P/C` and `phi = atan2(N, M)` are untouched. Consequence:
of the five lengths `r_b, r_p, a, d, z_home`, only four ratios matter. Normalise by
`r_b` and sweep `beta, r_p/r_b, beta_p, a/r_b, d/r_b, z_home/r_b`; pick `r_b`
afterwards. One axis fewer — a factor of ten at ten points per axis.

Corollary already applied: the rank guard must normalise its moment rows by a
geometry length, not a fixed constant. With a fixed 100 the same machine ranks four
different ways depending on how big it was built.

Absolute scale re-enters at: servo torque; ball dynamics (`g` sets a timescale the
kinematics has not got); build volume and plate stiffness; and `a`, restricted to
horns that exist and so not continuously rescalable. All of these belong to scoring
and component selection, not to the kinematic feasibility search.

### Withdrawn 2026-09-03 — do not let these propagate

- The figures `9.10 → 7.76 → 5.74` for `|w|` on leg 1 versus `h_p`. They rested on
  six placeholder values (`r_b=100, beta=12, delta=132, r_p=85, beta_p=48,
  z_home=120`), none of which are decisions. Same class as the invented 300 mm/s.
  The algebraic claim above stands without them.
- The claim that `h_p` is what makes the six legs unequal. It is not — the six
  `w_i` were already unequal at `h_p = 0` at that pose. A roll is not a D₃
  operation, so the legs stop being related once `R ≠ I` at any `h_p`.
- The statement of `beta_p`'s range as `(0°, 60°)`. See open item 3 below.

---

## Open

**1. Five checks, all numerical one-liners against existing code. Run these first —
two of them can change how the section above is written up.**

- The corrected claim says the six legs "stop being related the moment `R ≠ I`".
  Too strong: some tilts leave a residual symmetry. Which ones, and what structure
  should the six `|w_i|` show at such a pose? A free test of the `w` computation,
  same class as `arm_tips(0) == b + a·u`.
- Run the rank guard as `beta_p → 0`. Three anchor points carrying two rods each —
  is that rank-deficient, or a named architecture? If nothing drops, the endpoint
  exclusion is hardware on both rings and the two range arguments collapse to one.
- Write out the constraint line for leg `i` and check whether `b_i` appears in it.
  If it does not, the excluded hole near `beta_p = beta` is not where it has been
  placed.
- Does the pose envelope scale with `k`? Missing from the list of places absolute
  scale re-enters, and it is in the appendix of the derivation document.
- Do `r_b`, `r_p` and `√(r_b r_p)` rank two candidates with different `r_p/r_b` in
  the same order? If not, an arbitrary constant has moved from the normalisation
  into the scoring function, where it is harder to see.

**2. The pairing argument is not closed.** Brute force over 720 bijections leaves 6
survivors forming a single D₃ orbit — the count is his and is right. Two readings
of it are still unchallenged:

- *"The rotations are the same machine relabeled."* Test: for each survivor compute
  leg 1's angular span `gamma_tau(1) − theta_1`. If they were relabelings of one
  machine those six spans must agree as a multiset. Run at `beta = 20`,
  `beta_p = 40`.
- *"The reflections give the mirror-image machine, a handedness choice."* Leg-set
  invariance under reflection was the hypothesis. Ask whether any survivor can have
  a mirror image distinct from itself.

Both matter because "zero sweep axes for the pairing" rests on them.

Also outstanding: `mu ∈ {0, 180}` was derived under the identity pairing while the
pairing result ranged over all 720. Each froze what the other varied. The joint
case only changes how the result is *stated* — the gauge argument kills `mu` either
way — but it should be stated correctly.

**3. `beta_p`'s range cannot currently be stated, and that is the honest position.**
The base ring's `(0°, 60°)` rests on two different arguments that were being
conflated. Degeneracy — six points collapse to three at either end — is pure
geometry and transfers, *subject to the rank-guard check above*. Servo collision
bites well before degeneracy does and does **not** transfer, because a ball-joint
seat in a plate is a hole, not a body. So `beta_p`'s outer bound is set by
ball-joint housing diameter, a hardware number not yet in hand. There is also an
exclusion `beta` has not got: `beta_p = beta` puts every anchor above its own shaft,
`p = (r_p/r_b) b`, an affine image, rank 3. So the admissible set is an interval
with a hole in it, and neither the outer bound nor the hole width is known yet.
Still one axis; the earlier "two axes" count for the platform ring was right but
incomplete as a statement.

**4. Sweep budget as it stands.** Platform ring contributes `r_p` and `beta_p`,
matching the base ring's `r_b` and `beta`. `mu` and the pairing are eliminated by
argument, `h_p` is fixed by components. `delta` is an inner 1-D tune per candidate,
not an axis. After normalising by `r_b`: `beta, r_p/r_b, beta_p, a/r_b, d/r_b,
z_home/r_b`.

**5. Unchanged from `stewart-ik-derivation.md` §6.** Branch rule for the `±`.
Rotation convention for `R`. Numerical FK. Round trip.

---

## Plan, in order

Tags: **[Y]** his, **[CC]** Claude Code, **[bg]** background.

0. **Hardware spec pull — ~1 h [CC/bg].** Servo shortlist with travel range and
   deadband; available horn lengths (the discrete set `a` lives in); ball-joint
   housing outside diameter; rod stock. No purchases. Unblocks `beta_p`'s bound and
   `a`, and it has lead time, so it must not sit behind derivation work.
1. **The five checks — one block, ~1 h [Y].**
2. **Trace the sweep loop on paper — 15 min [Y].** List what one candidate
   evaluation calls. Decides whether `fk()` is on the critical path or beside it.
3. **Branch rule [Y].** Pose-only. Test is reproducibility, not smoothness: same
   pose, same angles, no history. Then check across a smooth trajectory for a step
   in one servo trace. Not delegated, any of it.
4. **`ik()` [Y].** `|P| > C` tested before `arccos`. Never `np.clip`.
5. **`fk()` — residual and convergence criterion [Y], optimiser wiring [CC].**
   Correct, not fast; it runs once per gate, not once per candidate, unless step 2
   says otherwise.
6. **Round-trip gate — hard stop [Y].** Nothing below is worth writing until this
   passes.
7. **Tilt target — 30 min [Y].** One number, margin over 1.635° argued from latency
   and disturbance rather than picked. Sets the pose envelope.
8. **Score function — cap at 2 h [Y].** Weighted scalar, weights stated, plus a
   sensitivity check that the ranking survives perturbing them. Not a Pareto front.
9. **Sweep harness [CC].** Normalised axes as above, `delta` inner. Coarse first —
   5 points per axis, not 10 — to find out whether the feasible set is empty before
   paying for resolution.
10. **Run, expect empty, widen [Y].** An empty return is information about the tilt
    target or the envelope, not a bug. Then refine, pick `r_b` from torque and build
    volume, then CAD, then order.

**Not on the critical path.** Grübler/Kutzbach returns 6, he knows it returns 6, and
the sweep never calls it. Portfolio completeness. Batch it with the document work at
the end of Phase 0.

---

## For the document session, later

`stewart-ik-derivation.md` needs a §9 for the platform ring, and §6's rows for the
anchor parameterisation and the rotation convention need updating. `w` is already
in. `phase-0-design-log.md` needs a 3 September entry. None of that belongs in a
derivation session.

Session log row, engagement not yet given:

| Date | Phase | Engagement | Notes |
|---|---|---|---|
| 2026-09-03 | 0 | — | Platform anchor parameterisation. Symmetry object identified as the legs; `mu` eliminated by a gauge argument; `h_p` shown to enter the kinematics and placed in the model but not the sweep; sweep proved exactly scale invariant, removing an axis and fixing the rank guard's normalisation. Two withdrawals, both self-caught after challenge: placeholder numbers presented as results, and a false attribution of leg inequality to `h_p`. Pairing and `beta_p` range left open rather than overstated. |
