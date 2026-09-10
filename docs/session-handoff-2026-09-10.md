# Handoff — Stewart platform, end of Phase 0, after 2026-09-10

Paste this into a new chat. Attach `stewart-ik-derivation.md`, `notation.md`,
`phase-0-design-log.md`, `hardware-pull.md` and the sweep report.

**Phase 0 is complete on its own terms: the sweep ran and chose the dimensions.**
The task for the next session is [ ], capped at [ ].

---

## Rules, short form

- Blunt about wasted time and money. Concise. No preambles.
- **Code lives with Claude Code in VS Code, which has the full repo.** Do not write
  scaffolding in chat. If something needs building, say so and write the Claude Code
  prompt, and say whether it is a Sonnet or an Opus job. Chat is for derivation,
  review and challenge. You may ask for the output of a check; you do not produce
  the file that runs it.
- Watch for a fully-formed derivation arriving in a register unlike his own.
- Name files and functions. Do not refer to numbered list items across sessions.
- **[Y] items are not delegated.** Tested repeatedly and holding the line has been
  right every time.
- Anything the assistant derives is unverified until a diagnostic says otherwise,
  and must be labelled as such when written. Four documented-result-without-code
  failures are on record.
- **Check that an open item is still open before writing a prompt against it.** Two
  items on the last handoff's open list — the `arccos` metric floor and `fk()`'s
  `(R, T)` convention — had already been closed in the repo and were dispatched
  anyway. CC checked rather than edited, which is why nothing was broken.

---

## The result

Two candidates, a mirror pair, identical at both published housing bounds:

| | |
|---|---|
| `a` | 60.40 mm (`0.6711 r_b`), PDRS60-25T outermost hole |
| `d` | 126 mm (`1.40 r_b`) |
| `beta` | 5° |
| `beta_p` | 52.5° |
| `z_home` | 95.62 mm |
| `margin(dxy = 0.003849)` | 0.874876, tie floor 0.874726 |

Fixed upstream, all asserted: `r_b = 90 mm` (print bed 180 × 180), `r_p = 80 mm`
(printed hub carrying a bought 220 mm sheet), `c_p/r_b = 0.1`, tilt limit 6.5580°.

Commit `2fee8b0`. 368 000 screened, 327 780 feasible (89.1%), all scored.
Anchor separation clears the full published 9–13 mm housing range; both members sit
interior in `beta` with 2.71 mm of arc slack.

**Both millimetre checks pass, and not narrowly.** `min(C_i − |P_i|) = 90.96 mm`
against a 0.3464 mm build stack — 263×, and a property of the region, not the
winner: the next seven groups sit in the 81–92 mm band. Rod cut tolerance has no
published figure, so the question was inverted rather than linearised: the margin
is not exhausted until a 37.9 mm cutting error on a 126 mm target. Worst leg force
37.7 mN from the actual statics (`J_fk^T f = −W_ext`, 2.7 g ball); Euler buckling
passes by 19× at the thinnest published stock. That closes the 19 August force
ruling with a number rather than a fourth judgement.

---

## The result's one caveat, and it is the objective

`a = 60.40 mm` is the top of the published ProModeler ladder. It is not an artefact
of the unbounded `beta` — it survives the bound. `tau_min` was checked as the
kinematic candidate for bounding it and does not: `rho(tau_min, margin) = +0.626`
overall, `+0.485` to `+0.526` at `a ≥ 45 mm`, positive throughout and never
approaching zero; the median rises to a peak near 40 mm and gives back 1.3% by the
ceiling.

**Three axes have now had their limit set by hardware or by decision and never by
the objective** — `r_p` (no kinematic quantity bounds it under a purely angular
envelope), `beta` (servo bodies), `a` (the catalogue). That is one property of reach
margin measured three times: it measures distance from unreachability, not
capability. The objective stays **open**, and this is its strongest statement.

The consequence for the build is narrow and worth stating plainly: every dimension
traces to the analysis except `a`, which traces to a catalogue, with the reason
recorded and the kinematic alternative measured and rejected.

---

## The score function, settled

`score = margin(dxy = p)`, maximin normalised reach margin `(C_i − |P_i|)/C_i` over
legs and the 29-pose envelope grid at platform displacement `p = 0.003849`, with
`delta` tuned per candidate to maximise `margin(0)` subject to
`cond(J_fk) ≤ 1e6` at `char_len = r_b`. Feasibility pass/fail before scoring.
Output is a tie set.

There are no weights. `margin − sens(p)·p ≡ margin(dxy = p)` to 6.9e-18, so what the
plan called a weighted scalar is a single functional at a displaced pose, and the
one number in it is a stated requirement rather than a tuned constant. `tau_min` is
measured-and-redundant. `cond` is a cap on the inner tune, not a ranking term.

Feasibility is four tests: non-empty `z_home` bracket, envelope reachable,
`N_i > 0`, and minimum bracket width ≥ 2 mm. The fourth removed 12 078 candidates,
**all at exactly zero width** — precisely the artefacts it was asserted for and
nothing else. Servo travel is excluded by decision.

---

## Open, carried forward

- **The objective.** Above. Nothing else is blocked by it; the sweep has run.
- **The pairing argument and the discrete `mu` residue.** `mu ∈ {0, 180}` was derived
  under the identity pairing while the pairing result ranged over all 720 bijections
  — each froze what the other varied. Restating it correctly needs the leg-1
  angular-span multiset check, named in the 2026-09-03 handoff and never run. CC has
  written out exactly what that test is. Changes how the result is stated, not
  whether `mu` is a sweep axis.
- **`tau_L` re-check.** Dropped 2026-09-08 because servo step response is unpublished
  by every candidate maker and cannot be reconstructed. If it is ever measured and
  the envelope moves, the shortlist re-runs before CAD.
- **Servo mounting-arc footprint.** `beta`'s bound is permissive — derived from bare
  case width, 13.0 mm. Flanges and wiring only add, so it tightens rather than
  loosens when the installed figure exists. A mechanical-drawing search was
  identified as the cheap route and not taken.
- **`char_len`.** Still open, narrowed to two uses: quoting the cap's `C`, and the FK
  residual-to-pose bound.
- **The `X = 1` crossing correction** is outside its tested tilt range for the second
  time. It was measured at 8.538–14.552°; the limit is now 6.558°.
- **Remaining §11 clashes** with no agreed answer: `R` for sinusoid amplitude, `s`
  for screw direction, `t` for plate thickness, `a`/`A_i`, `p` for pitch.
- **"Where Phase 0 stands"** at the end of the design log is stale and in his voice,
  left unedited with a dated pointer above it.
- **No git remote.** `git remote -v` has been empty since the 5 September audit.
  Every commit from `9514899` through the current head is on one machine.

---

## The pattern worth carrying into Phase 1

Five instances now of a check that could not have failed for the right reason, or
succeeded for one: the single-leg kick that left 80 of 124 groups untouched; the
shared-delta field comparison; the `sigma_min(J_cmd)` proxy that would have rejected
a correct derivation; the `margin_con` reference that compared a normalised minimum
against an unnormalised one; and a stale `.pyc` producing a bogus `TypeError`.

Alongside four instances of a discrete grid flattering a worst case in the unsafe
direction — the `N_i > 0` bound, the azimuth window, the harness pose grid, the
zero-width `z_home` brackets — and one that looked like a fifth and was not: the
mirror-pair spread, which turned out to be the fixed `−` branch selecting the
opposite arm configuration under `n → −n`, a symmetry the solver breaks that the
geometry does not.

**Two mirrors are one linkage and one score but not one machine to build.** The arms
sit on opposite sides. The branch rule was fixed as `−` on 4 September without this
in view.

---

## Plan, in order

Tags: **[Y]** his, **[CC]** Claude Code, **[bg]** background.

- **Rewrite "Where Phase 0 stands" [CC].** Stale on two points it already admits:
  the branch rule it names as the immediate open question was fixed 2026-09-04, and
  the numerical FK and passing round-trip test it lists as remaining are done. It is
  in his voice, so a rewrite is a rewrite of voice as well as fact — read it before
  dispatching.
- **Pick the mirror, then CAD [Y + CC].** The two shortlist members are the same
  machine reflected; picking one is a build decision, not a ranking one.
- **Choose the joint, the servo and the rod stock [Y].** All three were deliberately
  deferred until the geometry existed. The housing OD is now a check the shortlist
  passes at both ends rather than a constraint; the servo is chosen on torque and
  deadband, with the Hitec HS-5055MG deadband conflict — 6 µs MFR against 2 µs
  RETAIL — unresolved and landing on tilt resolution.
- **Order [Y].** Nothing has been purchased. Every dimension in the build traces to
  the analysis, with `a` the recorded exception.
