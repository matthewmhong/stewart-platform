# Handoff — Stewart platform, Phase 0, after 2026-09-05

Paste this into a new chat. Attach `stewart-ik-derivation.md`, `notation.md`,
`phase-0-design-log.md`, `cc-summary-2026-09-05.md` and `cc-fk-gate.md`.

The task for this session is the **score function**. It is [Y], capped at 2 h.
Everything below is context for that decision, not a menu of things to work on.

---

## Rules, short form

- Blunt about wasted time and money. Concise. No preambles.
- **Code lives with Claude Code in VS Code, which has the full repo.** Do not write
  scaffolding in chat. If something needs building, say so and write the Claude Code
  prompt. Chat is for derivation, review and challenge. You may ask for the output
  of a check; you do not produce the file that runs it.
- Watch for a fully-formed derivation arriving in a register unlike his own.
- Name files and functions. Do not refer to numbered list items across sessions.
- **[Y] items are not delegated.** Tested repeatedly and holding the line has been
  right every time. The score function is [Y]: the weights and the discriminators
  are his, and an assistant proposing a weighted sum for him to approve has taken
  the decision.
- Anything the assistant derives is unverified until a diagnostic says otherwise,
  and must be labelled as such when written. Three documented-result-without-code
  failures are on record; a fourth was nearly lost in a scratchpad.

---

## State

**The round-trip gate passes. The hard stop is cleared.** `fk()` and
`stewart/diagnostics/roundtrip.py` are done: worst translation error `1.85e-13 mm`,
worst rotation error `7.8e-14 deg` (the `2.09e-06` in the report is an `arccos`
metric floor, not the error), 14 436 poses, 4 geometries, home seed, 0
non-convergences. Unsquared residual in mm, rotation-vector parameterisation,
analytic Jacobian `df_i/domega = -e_i^T [R p_i]_x` verified to `9.55e-08` against
central differences with a negative control that rejects three wrong forms by nine
orders. Convergence stops on stagnation with the tolerance applied once as
acceptance — worst pose error moves by exactly zero across six decades of tolerance.
Derivation §7's blocked row and handoff open item 3 are discharged.

**Envelope, settled.** `dxy = dz = yaw = 0`. Tilt magnitude to 10.529°, azimuth
`[30°, 90°]` — not `[0°, 60°]`; the fixed azimuths of the mirror planes are at
`30 + 60k`, verified, and `[0°, 60°]` double-counts what it touches and misses the
orbit `{75°, 105°}`. Tilt limit derived from bang-bang recovery, `x0 = 80 mm`,
`tau = 0.5 s`; bare requirement 6.6°, the difference is latency margin at
`tau_L = 150 ms`, which is provisional and inherited from a withdrawn figure.
Envelope is purely angular so it does not scale with `k`.

**Sweep is six normalised axes**, 15625 candidates at 5 points: `beta`, `r_p/r_b`,
`beta_p`, `a/r_b`, `d/r_b`, `z_home/r_b`. `z_home` was briefly removed by a flat-arm
assembly datum and that identification was dropped — `z_flat` is a build-time datum
only. `delta` is an inner 1-D tune, `[0, 180)`, on the normalised margin
`(C_i - |P_i|)/C_i`, maximin over legs and envelope poses. `J = max|w_i|` is
retired; the two objectives are monotone in the same pointwise quantity but differ
in aggregation.

**Feasibility is settled and is pass/fail before scoring:** non-empty `z_home`
bracket, envelope reachable, `N_i > 0`. Lower bracket closed form
`z_home > r_p sin(tilt) + h_p cos(tilt)`, verified `5.551e-17`. Over a 540-candidate
coarse grid: 363 non-empty, 177 empty — 176 reach alone, 1 a genuine `N_i` crossing.
`N_i > 0` also keeps the feasible set connected in 6 candidates with spurious
low-`z_home` reach components. Empties concentrate at `a/r_b = 0.10` (144 of 180),
none at `0.35`.

---

## The task

The score function ranks the feasible survivors. What it needs, and what is
already known about each:

**Discriminators on the table.**

- *Margin at the tuned `delta`* — `min_i (C_i - |P_i|)/C_i` over the envelope. This
  is what the inner loop already maximises, so ranking on it means ranking
  candidates by the thing each was individually optimised for. Defensible, but it
  is one discriminator, not two, and the handoff previously listed it twice under
  different names.
- *Translation sensitivity* — margin lost per unit normalised displacement,
  `Δmargin / (dxy/r_b)`, at the tuned `delta`. This is where build error lives:
  `dxy = 0` is correct about what the control law commands, but the platform is
  displaced from where the model thinks it is by rod cut error, centring and joint
  clearance. A ranking discriminator, never a feasibility test. Calibration from
  fixture A: order 7 units of margin per unit normalised displacement, so a probe
  of `0.005–0.01 r_b` stays linear. Probe magnitude is undecided.
- *`cond(J)`* — new, and there is a concrete reason for it. The FK found 8 distinct
  assembly modes per fixture, all at the same arithmetic floor, so the residual
  cannot separate them. On `smoke_geometry` at `cond 9045` two modes sit 0.32 mm
  apart and `ik` returns the same six angles at both — two poses the machine cannot
  distinguish from its own commands. Real fixtures sit at `cond 4–10`. Nothing in
  feasibility excludes the near-singular region. Argues for `cond` with a floor.
- *Servo travel used* and *tilt resolution* — both meaningless until the hardware
  pull is read.

**Blocking sub-decision: the characteristic length.** `J` has translation columns in
mm/mm and rotation columns in mm/rad, so `cond(J)` is meaningless without a length
to normalise the moment rows. `notation.md` §12 lists this as undecided and states
that the choice changes the ranking, so it is a requirement to be stated rather than
a constant to be picked. CC quoted `cond` at `r_b` as provisional and refused to
default it; the spread over four candidates is a factor of 3. If `cond` becomes a
discriminator with a floor, this number decides where the floor sits.

**Open item 12 belongs to this session.** The pose grid flatters the worst case in
the unsafe direction: 10° azimuth sampling gives `+1.550398e-01` against
`+1.531859e-01` at a 0.25° reference — optimistic by `1.9e-3`. Two candidate
resolutions: refine the grid, or accept an optimistic ranking grid provided
feasibility comes from closed forms. The second is only safe if the optimism is
roughly uniform across candidates, and there is no reason it would be, since the
worst-case azimuth location depends on geometry. Testable cheaply: re-rank the top
handful on the 0.25° grid and confirm the order holds. This is the third instance of
a discrete grid flattering a worst case — the `N_i > 0` bound and the azimuth window
were the first two.

**Required of the result**, from the plan: a weighted scalar, weights stated, plus a
sensitivity check that the ranking survives perturbing them. Not a Pareto front.
Feasibility stays pass/fail before scoring.

---

## Also open, not this session

- **Hardware pull, dispatched 2026-09-04, still unread.** Servo travel and deadband;
  horn lengths (the discrete set `a` lives in); ball-joint housing OD (fixes
  `beta_p`'s outer bound); rod stock; joint angular misalignment range; horn spline
  tooth count. It gates `beta`, `beta_p`, `a`, `d` — all sweep inputs — and two score
  discriminators. The `a/r_b = 0.10` result gives it a sharp question: the feasible
  region wants long horns relative to `r_b`, so a short horn set constrains `r_b`
  from below before the sweep runs. This is the only thing available that needs no
  decision from him.
- Rotation convention for `R` (derivation §6). The rotation-vector FK does not need
  it and the gate compares `R` matrices directly, so it no longer blocks anything.
- The pairing argument, `h_p → c_p` rename, Grübler, derivation section renumber
  (file reads 1–7, 8, appendix, no §9). Batch as document work.
- `arccos` geodesic metric floor at `8.5e-7 deg` in `roundtrip.py` — replace with the
  rotation-vector magnitude of `R_cmd^T R_fk`. Cosmetic, gate passes either way.
- `fk()` returns `(R, T)` not `(T, R)`. This matches `ik(geom, R, T)` and
  `stage1(geom, R, T)`, so it is the repo convention; record it in `notation.md`.

---

## Plan, in order

Tags: **[Y]** his, **[CC]** Claude Code, **[bg]** background.

- **Score function — cap at 2 h [Y].** This session.
- **Sweep harness [CC].** Six normalised axes, `delta` inner on `[0, 180)` seeded at
  `delta*` with no basis claim, two-axis envelope, exact delta-free feasibility
  bracket (`|P_i| >= |L_i|` drops a candidate before the scan;
  `|P_i| <= sqrt(|L_i|^2 - amp_i^2)` skips it), chunked — ~2.7M full plus ~4.9e8
  cheap evaluations, ~3.9 GB.
- **Run, expect empty, widen [Y].** An empty return is information about the tilt
  target, not a bug. 177 of 540 were already empty on the coarse grid before
  scoring. Then refine, pick `r_b` from torque and build volume, then CAD, then
  order.
