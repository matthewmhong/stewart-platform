# Inverse Kinematics — 6-RSS Stewart Platform

**Phase 0 working document.** Rotary servos, fixed-length rods.

> **Note on completeness.** Everything below marked *derived* is yours, from
> sessions on 2026-08-20 and 2026-08-28. Stage 2's closed form was the last
> substantial piece of Phase 0's mathematics; it is now solved and verified
> numerically. What remains is the branch choice, the rotation convention, the
> anchor parameterisation and the numerical FK — all listed in §6.

---

## 1. Notation

The single glossary for the project. Current definitions only; the dated history
of how each symbol was settled is in `docs/archive/notation.md` and the design
log. Current *values* (the chosen dimensions) are in `STATUS.md`.

**Conventions.** Millimetres and radians internally; degrees only in printed
summaries and servo commands. All vectors are **column** vectors. Legs are
0-indexed in code (`i = 0..5`, `floor(i/2)`) and 1-indexed in prose and error
messages (`floor((i-1)/2)`). **Pose order is `(R, T)` in every signature** —
`ik(geom, R, T)`, `fk(...) -> (R, T)` — because in `q_i = T + R p_i`, `R` is the
operator and `T` the offset.

### Frames and pose

| Symbol | Meaning | Units |
|---|---|---|
| `{W}` | world frame. Fixed to the base; origin at the base ring centre, `z` up, base plate at `z = 0` | — |
| `{P}` | platform frame. Origin at the platform centre on the plate top, axes glued to the plate | — |
| `T` | position of the platform centre, in `{W}` | mm |
| `R` | platform orientation; converts `{P}` components to `{W}` components | — |

### Base ring

| Symbol | Meaning | Units |
|---|---|---|
| `r_b` | base ring radius (fixed, 90 mm — the print bed) | mm |
| `beta` | base pair half-split | deg |
| `s_i` | within-pair sign, `(-1, +1, -1, +1, -1, +1)`; also the `C3` orbit label | — |
| `theta_i` | shaft azimuth, `120 floor(i/2) + s_i beta` | deg |
| `b_i` | servo shaft position, `r_b (cos theta_i, sin theta_i, 0)` | mm |
| `delta` | servo-plane twist from tangential, alternating within each pair, `[0, 180)`. Tuned per candidate, not swept | deg |
| `psi_i` | azimuth of `n_i`, `theta_i + 90 + s_i delta` (the `+90` is forced by the mirror condition) | deg |
| `n_i` | unit normal of servo `i`'s rotation plane (horizontal) | — |
| `u_i` | `z × n_i`; in-plane direction of `alpha_i = 0` (arm flat) | — |
| `v_i` | `n_i × u_i`, equal to `z` while `n_i` is horizontal; positive `alpha_i` raises the tip | — |

### Platform ring

| Symbol | Meaning | Units |
|---|---|---|
| `r_p` | platform ring radius (fixed, 80 mm — the printed hub) | mm |
| `beta_p` | platform pair half-split | deg |
| `phi_i` | anchor azimuth, `120 floor(i/2) + s_i beta_p` | deg |
| `c_p` | plate offset: anchor-plane depth below the `{P}` origin, a measured hardware number | mm |
| `p_i` | platform anchor, `(r_p cos phi_i, r_p sin phi_i, -c_p)` in `{P}` | mm |
| `mu` | rotation of the platform ring inside `{P}`. **Not a parameter** — a gauge; `R = I` means platform pair centres line up with base pair centres | deg |

### Links, per-leg quantities, the unknown

| Symbol | Meaning | Units |
|---|---|---|
| `a` | servo arm length (a discrete set: published horn holes) | mm |
| `d` | push-rod length — measure after cutting | mm |
| `q_i` | world anchor position, `T + R p_i` | mm |
| `L_i` | leg vector, `q_i - b_i` | mm |
| `h_i` | arm tip, `b_i + a (u_i cos alpha_i + v_i sin alpha_i)` | mm |
| `rod_i` | `q_i - h_i`, magnitude `d` | mm |
| `tangent_i` | unit arm tangent, `-u_i sin alpha_i + v_i cos alpha_i` | — |
| `w_i` | `L_i · n_i`, anchor's signed distance off servo `i`'s plane | mm |
| `rho_i` | `sqrt(d² - w_i²)`, rod length projected into the plane | mm |
| `M_i`, `N_i` | `L_i · u_i`, `L_i · v_i`; `N_i` is the anchor height above the base plate | mm |
| `C_i` | `hypot(M_i, N_i) = sqrt(\|L_i\|² - w_i²)`, in-plane magnitude of the leg | mm |
| `P_i` | `(\|L_i\|² + a² - d²) / 2a`; contains no `delta` | mm |
| `alpha_i` | **the unknown** — servo `i`'s angle from `u_i` toward `v_i` | rad (deg at the servo) |

`(u_i, v_i)` is an orthonormal basis for servo *i*'s rotation plane, written in
world components. It is what lets a 2D problem be posed inside a tilted 3D plane.

### Analysis quantities

| Symbol | Meaning | Units |
|---|---|---|
| `z_home` | platform height at the home pose; a sweep axis, bracketed below by `N_i > 0` and above by reach | mm |
| `z_flat` | plate height with all arms flat (`alpha_i = 0`), `R = I`. Assembly datum only, §9 | mm |
| `A_i`, `B_i` | `w_i(delta) = A_i cos delta + B_i sin delta`; both `delta`-free | mm |
| margin | normalised reach margin `(C_i - \|P_i\|) / C_i`, maximin over legs and envelope poses. Negative = infeasible | — |
| `p` | build error the score is read at, `0.3464 mm / r_b = 0.003849` (RSS of three 0.2 mm sources) | — |
| score | `margin(dxy = p)`, `delta` tuned to maximise `margin(0)` subject to `cond(J_fk) <= 1e6` at `char_len = r_b` | — |
| `tau_i` | transmission ratio, `\|rod_i · tangent_i\| / d`; zero at loss of authority | — |
| `J_fk` | forward-kinematics Jacobian | — |
| `k` | uniform length scale; `alpha_i` is invariant under scaling every length by `k` | — |

### Working envelope

Tilt only: `dxy = dz = 0`, yaw 0. **Tilt limit 6.558°** from bang-bang recovery of
a 50 mm ball displacement in `tau = 0.5 s`: `acc = 4 x0 / tau²`,
`sin(tilt) = 7 acc / 5g`. Tilt goes as `1/tau²`, so a 10% error in `tau` moves
`sin(tilt)` ~20%. Grid: 5 magnitudes × 7 azimuths over `[30°, 90°]` = 29 poses
(D₃ symmetry makes a 60° window sufficient — but it is `[30°, 90°]`, not
`[0°, 60°]`). The 10° azimuth grid is optimistic on worst-case margin by up to
`2.88e-3`; use it for ranking, not for feasibility.

### Symmetry

| Symbol | Meaning |
|---|---|
| `C3` | rotation by 120° about `z`; leg permutation `[2, 3, 4, 5, 0, 1]` |
| `m0`, `m60`, `m120` | mirror planes at azimuths 0, 60, 120 (mod 180); `m0` permutation `[1, 0, 5, 4, 3, 2]`. The tilt axes they fix sit at 30 + 60k |
| `D3` | the order-6 group they generate — a requirement on the *legs* (shafts and anchors together) |
| `sigma` | leg permutation induced by a group element |

Under a mirror `w_sigma(i) = -w_i`; under `C3` no sign flip. Free test on any `w`:
home or pure heave gives six equal `|w|`, alternating sign; pure yaw gives two
values, one per `C3` orbit.

---

## 2. What the function does

```
input:   a pose  (T, R)          — 6 numbers
output:  alpha_1 … alpha_6       — 6 servo angles
```

Six legs, solved **independently**. Leg 3's problem contains no reference to
servos 1, 2, 4, 5 or 6, nor to the base as a whole. Six separate problems that
share only the pose they started from.

---

## 3. Stage 1 — anchor world position *(derived)*

Anchor *i* is a hole in the printed platform ring. Its coordinates in `{P}` are a
constant. Its coordinates in `{W}` are not.

You cannot add a `{W}` vector to a `{P}` vector componentwise — one unit along
the world x-axis is not one unit along the platform's x-axis unless the platform
happens to be unrotated. `R` is what reconciles them.

```
q_i  =  T  +  R p_i
```

**Check it:** set `R = I` (platform level). The expression must collapse to plain
vector addition. Set `T = 0` and rotate 90° about z; the anchor must swing to
where your hand says it should.

---

## 4. The leg vector *(derived)*

Leg *i*'s geometry problem involves exactly two known points: the platform anchor
`q_i`, and servo *i*'s shaft `b_i`. The intersection is unchanged if the whole
machine is carried across the room, so what matters is not either absolute
position but the vector between them:

```
L_i  =  q_i  -  b_i  =  T  +  R p_i  -  b_i
```

**Careful:** `|L_i|` is *not* the rod length. The servo arm makes up the
difference — that's the entire point of stage 2.

---

## 5. Stage 2 — servo angle *(derived)*

The geometry: the arm tip is confined to a **circle** of radius `a`, centred on
`b_i`, lying in servo *i*'s rotation plane. The fixed-length rod confines it to
the surface of a **sphere** of radius `d` centred on `q_i`. The arm tip is where
the two meet.

Constraints on the unknown `h_i`:

```
|h_i - b_i|  =  a                 arm tip lies on the circle
|q_i - h_i|  =  d                 rod length is fixed
h_i - b_i    lies in servo i's rotation plane
```

### 5.1 Eliminate `h_i`

`h_i` looks like three unknowns. It is not. Constraints 1 and 3 together confine
the tip to a curve, and a point on a curve takes **one** number to specify — and
that number is `alpha_i`, the quantity you were solving for anyway. So those two
constraints are spent not by being solved but by collapsing `h_i` to:

```
h_i  =  b_i  +  a ( u_i cos alpha_i  +  v_i sin alpha_i )
```

`u_i` and `v_i` carry the direction; the two scalars are the tip's coordinates
*inside* the plane. There is no third term — the tip never leaves the plane, so
nothing multiplies `n_i`.

That leaves one unused constraint and one unknown. That is why a closed form
exists.

### 5.2 Substitute and expand

Into `|q_i - h_i| = d`, grouping `q_i - b_i` into `L_i`:

```
| L_i  -  a ( u_i cos alpha_i + v_i sin alpha_i ) |  =  d
```

Square both sides — the bars are vector magnitude, and `|A|² = A·A`, so squaring
removes the root cleanly and loses nothing (both sides non-negative). Then
`|A - B|² = |A|² - 2 A·B + |B|²`:

```
|L_i|²  -  2 a ( L_i·u_i cos alpha_i  +  L_i·v_i sin alpha_i )  +  a²  =  d²
```

Two simplifications did the work:

- `|B|² = a²`. The four cross terms collapse via `u·u = v·v = 1`, `u·v = 0`,
  leaving `a²(cos² + sin²)`. Physically forced — `B` is the arm, and its length
  is `a` whichever way it points. If the algebra had produced anything else,
  something upstream was wrong.
- `L_i·u_i` and `L_i·v_i` are **known scalars** — the anchor's coordinates within
  servo *i*'s plane. No `alpha` in either.

Rearranged, everything unknown on the left:

```
M cos alpha_i  +  N sin alpha_i  =  P

M = L_i·u_i        N = L_i·v_i        P = ( |L_i|² + a² - d² ) / 2a
```

### 5.3 Invert

`M cos + N sin` is a weighted sum of two sinusoids at the same frequency, so it
collapses to a single shifted one. Treat `(M, N)` as a point in an abstract plane
— nothing to do with the servo's — and write it in polar form, `M = C cos phi`,
`N = C sin phi`. Substituting gives the cosine difference identity:

```
M cos alpha + N sin alpha  =  C ( cos phi cos alpha + sin phi sin alpha )
                           =  C cos( alpha - phi )
```

`alpha` now appears exactly once, so it inverts directly:

```
C     =  sqrt(M² + N²)          =  hypot(M, N)
phi   =  atan2(N, M)
alpha_i  =  phi  ±  arccos( P / C )
```

**`atan2`, never `arctan`.** `arctan(N/M)` has range `(-90°, 90°)` — half a turn
— and the division destroys the sign information that distinguishes `(M, N)` from
`(-M, -N)`. Half the servos come out 180° wrong, which on the bench looks like a
wiring fault.

### 5.4 Solution count

Falls straight out of `arccos` needing its argument in `[-1, 1]`:

| Condition | Solutions | Meaning |
|---|---|---|
| `\|P\| < C` | 2 | generic; circle cuts sphere twice |
| `\|P\| = C` | 1 | tangent; **workspace boundary** |
| `\|P\| > C` | 0 | unreachable — raise, do not clip |

`np.clip` on `P/C` silently manufactures a fake solution at the boundary. Fine in
a throwaway check where everything is known to reach; a bug in `ik()`. Test
`|P| > C` first.

**Resolved 2026-09-01.** `|P| > C` does catch both failure directions. But the
intuitive two-sphere bound

```
|d - a|  <  |L_i|  <  d + a
```

is **necessary, not sufficient**, and must not be substituted for it in code. The
arm tip lies on a *circle*, not a sphere. Decompose `L_i` into in-plane and
out-of-plane parts, `|L_i|² = rho² + w²` with `w = L_i·n_i`. Then `M² + N² = rho²`
— the in-plane part only. The out-of-plane offset `w` enters `P` but never `C`.

Counter-example, `a = 20`, `d = 120`: anchor at `M = 0`, `N = 0`, `w = 110`. Then
`|L_i| = 110`, inside `(100, 140)`, yet `C = 0` and `|P| = 1900`. Unreachable.

Test `|P| > C`. Never the two-sphere bound.

### 5.5 Verification, 2026-08-28

Round-trip against `arm_tips()`: pick six angles, compute the tips, place anchors
at exactly `d` from each, run the closed form. Every true angle was recovered —
five legs on the `-` branch, one on the `+`. Formula confirmed; branch rule is
what decides which.

**Still open: the branch choice.** The `±` is real — both roots are valid arm
configurations. `ik()` must pick, and the rule has to be a function of the pose
alone, or the same pose returns different angles on different calls and nothing
downstream reproduces. Choose badly and the platform visibly snaps mid-move.

---

## 6. Also still yours

| Item | Why it isn't here |
|---|---|
| ~~Branch rule for the `±`~~ | **Struck 2026-09-04.** Fixed as `-` for all six legs; see §5.5 and the handoff. Reopens if the shafts are ever canted. |
| `R` composed from three angles | The rotation convention is a choice — pick it, state it, stick to it |
| Parameterisation of `p_i` | Radius, angular pattern, and rotation relative to the base ring. `b_i` and `n_i` are done — see §8 |
| Numerical forward kinematics | Needed for the round-trip check. No closed form exists — that's the interesting part |

---

## 7. Verification, before trusting any of it

1. **Units.** Every line. Degrees plus millimetres is never valid.
2. **Known cases.** `R = I`. `T = 0`. 90° about z. Answers you can check by hand.
3. **Round trip.** Pose → IK → six angles → numerical FK → recover the pose. If
   it doesn't close, the derivation is wrong and you found out for free.

### Done so far

| Check | Result |
|---|---|
| `stage1(R=I, T=0) == p` | passes |
| 90° about z sends `(10,0,0)` → `(0,10,0)` | passes — `R` not transposed |
| `arm_tips(0)` is exactly `a` from every shaft | passes |
| `arm_tips(0) == b + a·u` | passes — `u`/`n` not swapped |
| Stage 2 closed form recovers known angles | passes, §5.5 |
| Full round trip | **blocked** — needs `ik` and `fk` |

The distance check alone cannot catch a `u`/`n` swap: `n` is also unit length, so
a tip placed along it is also exactly `a` out. Hence the second check.

---

## 8. Base ring and servo planes *(derived 2026-09-01)*

```
s_i      = (-1, +1, -1, +1, -1, +1)
theta_i  = 120·floor((i-1)/2) + s_i·beta
b_i      = r_b (cos theta_i, sin theta_i, 0)

psi_i    = theta_i + 90 + s_i·delta
n_i      = (cos psi_i, sin psi_i, 0)

u_i      = z × n_i          v_i = n_i × u_i = z
```

Parameters: `r_b`, `beta ∈ (0°, 60°)` (30° = regular hexagon), `delta ∈ [0°, 180°)`
— sufficient, but only under a gauge that an assembly datum would lift; see the
note after the `c = 270` paragraph below.

**Why the `beta` interval is open — and why it is not what it looks like.** The
excluded endpoints are a **hardware** limit, not a rank one. `beta → 0` merges the
six shafts onto three points, and two servo bodies cannot occupy one mounting arc;
the same argument on the platform ring, with ball-joint housing diameter in place
of servo body diameter, is what bounds `beta_p`. Neither endpoint is degenerate as
*geometry*: `beta_p → 0` is the **3-6 Stewart platform**, a real architecture, and
`sigma_min` was measured O(1) all the way down on 2026-09-03 — the shafts stay
split, so the six leg lines remain distinct. The true bounds are therefore set by a
servo body dimension and a ball-joint housing diameter, **neither of which is known
yet**; `(0°, 60°)` is a placeholder standing in for them.

**`beta_p = beta` is not a rank hole either** *(verified 2026-09-04)*. With the
platform anchors an affine image of the shafts it was believed the six legs spanned
only three wrench dimensions. Recomputed with the **true rod lines** `q_i - h_i`
rather than the `q_i - b_i` proxy: full rank 6, and `sigma_min` runs *monotonically*
through `e = beta_p - beta = 0` without so much as a local minimum. The rank-3
result was an artifact — the six proxy lines are concurrent on the `z`-axis by
construction, and the servo arm breaks that concurrency. The recovered `sigma_min`
scales linearly with `a` (log-log slope 0.992), vanishing only as `a → 0`, which is
precisely the limit in which the proxy becomes exact.

The `90` is forced by the mirror condition within each pair, not chosen. `c = 270`
is the same planes with normals flipped and is absorbed by the half-open `delta`
range. The alternating `s_i` is what makes the family mirror-symmetric.

> **Recorded so it is not reintroduced — a flat-arm *datum* lifts the gauge that
> makes `delta ∈ [0°, 180°)` sufficient** *(measured 2026-09-04)*. The
> justification above is sound as stated: `n → -n` gives the same set of planes.
> But `u = z × n` flips with the normal, so `alpha = 0` puts the arm on the
> **opposite side** of the shaft. Under a relabelling of `alpha` that is the same
> machine — which is why the half-open range is fine as things stand. Under a
> **flat-arm datum**, which fixes what `alpha = 0` means physically, the two are
> **different assemblies** and the range is no longer sufficient.
>
> Measured, with `z_home = z_flat` imposed: **3 of 432** grid combinations are
> feasible only on `[180°, 360°)`. All three sit at a *simultaneous four-axis grid
> corner* — `beta` min, `beta_p` max, `r_p/r_b` max, `d/r_b` min — so the boundary
> lies **outside the sampled region and its extent is unknown**.
>
> Also worth recording: `delta*` reduced mod 180 returns to `delta_G = arg G` of
> §9, which is exactly where `(z_flat - c_p)²` is **minimised**. Under the datum
> the closed-form seed therefore pointed at the *tightest-feasibility* `delta`.
>
> None of this is live: the `z_home = z_flat` datum was **dropped 2026-09-04** and
> `delta ∈ [0°, 180°)` is restored as sufficient. It is written down because the
> datum is algebraically tempting and will be proposed again.
> Grid and run: `stewart/diagnostics/zhome_datum.py`, check B.

`delta` is tuned, not fixed. **Objective revised 2026-09-04 — this supersedes the
`J(delta) = max over envelope and legs of |L_i·n_i|` stated here previously.**
Maximise instead the **normalised reach margin**

```
margin_i  =  ( C_i - |P_i| ) / C_i          maximin over legs and envelope poses
```

The division by `C_i` is **required**, not cosmetic: `C` and `P` both carry length,
so the raw difference `C_i - |P_i|` scales with the uniform length factor `k` and
breaks the normalised sweep, in which candidates differing only in `r_b` must score
identically. This is the same form the branch check already reports, which is where
the `-5.7e-3` boundary figure came from.

**How the two objectives relate, precisely.** `delta` does not appear in `L_i`, so
`P_i` is `delta`-free and only `C_i = sqrt(|L_i|² - w_i²)` moves with it. At a fixed
leg and a fixed pose the margin is therefore **strictly decreasing in `|w_i|`**:
maximising the margin *is* minimising `|w_i|`, exactly. The divergence between the
two is entirely in the **aggregation** — `J` is a minimax over `|w_i|`, the margin
is a maximin over `(C_i - |P_i|)/C_i` — and because `|L_i|` varies across legs and
poses, the leg with the largest `|w_i|` is generally **not** the leg with the
smallest margin. Different worst cases, different minimisers, related quantities.

1-D search either way. Note the objective depends on `p_i`, so `delta` is an inner
optimisation inside the sweep, not a frozen constant.

**Consequence worth keeping.** `n_i·z = 0`, so `z` lies in every servo plane and

```
N  =  L_i·z  =  q_i·z  =  anchor height above the base plate
C  =  sqrt(M² + N²)  ≥  |N|
```

`C` cannot reach zero unless an anchor sits on the base plate. No loss of
authority through `C → 0`, no excluded region of `delta`. **This rests entirely on
horizontal shafts.** Cant them and the argument goes, and the degeneracy question
reopens.

At `alpha = 0` the arm lies flat at base-plate level — servo mid-travel, and an
assembly datum checkable by eye. **This is a build-time configuration, not the home
pose.** *(Clarified 2026-09-04, superseding any reading of the earlier wording that
put home on this configuration: the identification `z_home = z_flat` was tried on
2026-09-04 and* **dropped** *the same day. `z_flat` — §9 — is an assembly datum
only, and `z_home` is a swept axis. See the handoff.)*

## 9. `z_flat`, the flat-arm assembly height *(derived and verified 2026-09-04)*

`z_flat` is the plate height at which the arms lie flat — `alpha_i = 0` — with the
rods attached, at `R = I` and no horizontal translation. **It is an assembly datum.
It is not the home pose**; the attempt to identify the two was dropped the same day
it was made.

Write `g_i` for the horizontal offset `q_i^{xy} - b_i`, and

```
A        =  beta_p - beta
G        =  r_p e^{iA}  -  r_b
delta_G  =  arg G
```

Then

```
|g|²             =  r_p²  +  r_b²  -  2 r_p r_b cos A
g · u            =  r_b cos delta  -  r_p cos(A - delta)

(z_flat - c_p)²  =  d²  -  |g|²  -  a²  -  2 a |g| cos(delta - delta_G)
```

**Positive root**, because `z_flat - c_p` is `N_i` at that configuration and
`N_i > 0`.

**Why it is leg-independent, and why no group argument is needed.** Both `|g|²` and
`g · u` above are free of `s_i`. The leg index simply does not appear, so
leg-independence falls out of the **algebra**. This is worth stating explicitly
because the tempting route is a D₃ argument, and that route needs `n_i` to flip
under a mirror *and* `u_i = z × n_i` to flip again — a double sign change that in
practice passes by assertion rather than by being checked.

**Verified against `make_geometry`** — taking `b_i`, `u_i`, `n_i` and `p_i` from the
library, not from a ring rebuilt out of the formulas above, so the derivation is not
being tested against itself. Over 540 grid points:

| Check | Residual |
|---|---|
| leg-independence, `max_i - min_i` | ≤ 9.948e-14 (1.501e-15 relative) |
| closed form vs library geometry | ≤ 1.637e-11 (2.103e-15 relative) |
| `max\|u_i · z\|` | 0 exactly |

Round-trip confirmation of the formula, independent of the algebra: build the
geometry at the derived height, set `alpha = 0`, and check `|q_i - arm_tips(0)_i|`
against `d`. **Worst residual 2.220e-16.**

Run: `stewart/diagnostics/zhome_datum.py`, check A.

## 10. `alpha` at home is one scalar, shared by all six legs *(2026-09-04)*

Not a new measurement — it follows from §9's two residuals. At home `M_i = g_i·u_i`
and `|g_i|²` are leg-independent, and `N_i = z_home - c_p` is common to all six. So
`C_i`, `P_i` and `phi_i` are common, and hence so is the home angle.

Two uses:

- **All six servos read the same angle at home.** That is a by-eye build check, and
  it recovers at home exactly what the flat-arm datum was going to provide — without
  costing a sweep axis.
- **A non-zero home angle is absorbable by horn mounting angle.** Servo mid-travel
  can be bought with the spline instead of with the datum. What it costs depends on
  the horn's spline tooth count, which is on the hardware pull.

## 11. Grubler-Kutzbach degree-of-freedom check *(CC-derived, 2026-09-10 — UNVERIFIED)*

**CC-derived and unverified, per the standing rule** (`docs/archive/session-handoff-
2026-09-05.md`: "anything the assistant derives is unverified until a diagnostic
says otherwise, and must be labelled as such when written"). Nothing in the repo
tests mobility; this is pen-and-paper only, checked against the known result for
this architecture family and not against a simulation of this one.

**Links, `N = 14`.** One fixed base (ground) plus, per leg: one servo arm and
one push-rod — `6 x 2 = 12` moving links — plus the platform: `1 + 12 + 1 = 14`.

**Joints, `J = 18`, freedoms `sum f_i = 42`.** Per leg: one **revolute** at the
base (servo shaft to arm, `f = 1`) and two **spherical** (arm tip to rod, rod to
platform anchor, `f = 3` each) — matching R-S-S. `6` R `+` `12` S `= 18` joints;
`6(1) + 12(3) = 6 + 36 = 42`.

**Spatial Grubler-Kutzbach:**

```
M  =  6(N - 1 - J) + sum f_i
   =  6(14 - 1 - 18) + 42
   =  6(-5) + 42
   =  12
```

**Raw count is 12, not 6 — the standard S-S-leg artefact, resolved by
subtracting the idle spins.** Each push-rod is a binary link joined by a
spherical joint at both ends and nothing else, so it is free to spin about the
line joining the two joint centres without moving the arm or the platform at
all — a **passive, kinematically inert** freedom, not a useful one. There are
six rods, so six such idle freedoms:

```
M_actual  =  M_raw  -  (idle rod spins)  =  12 - 6  =  6
```

matching the platform's known mobility. This is the textbook correction for any
S-S leg (Merlet, *Parallel Robots*; also stated for the general 6-6 Gough-
Stewart platform), applied here to the R-S-S case rather than re-derived from
first principles — flagged as such, not claimed as new.

**What this does and does not check.** It counts freedoms; it says nothing about
**which six** — a rank-6 wrench system at a given pose, which the FK Jacobian
`J_fk` gate already exercises numerically pose by pose, and which is a different
(stronger, local) statement than a global mobility count. A mobility count of 6
does not by itself rule out a special pose where the instantaneous mobility
rises above 6 (a parallel singularity) or the six freedoms fail to include the
ones wanted; neither question is asked here.

---

## Appendix — results from Phase 0 analysis

- Minimum tilt to arrest a 200 mm/s ball over 100 mm: **1.635°**.
  *This is a minimum, not a design target*, and it is specifically an
  **arrest-framing** minimum — fixed entry speed, fixed stopping distance,
  `(5/7) g sin(tilt) = v²/2L`. Latency, disturbance rejection and friction
  variation all demand margin above it.

  **Superseded as a design basis, 2026-09-05.** The envelope is now set by a
  **recovery** framing — return the ball from a displacement `x0` in a time `tau`,
  bang-bang, `acc = 4x0/tau²` and `sin(tilt) = 7acc/5g` — giving a **6.558°**
  requirement and a **10.529°** envelope. See `docs/archive/notation.md` §9; the number above
  is kept because it is correct for what it measures and because the two framings
  scale **oppositely** in plate size (arrest: tilt as `1/k`; recovery: tilt as
  `k`), which is worth not re-deriving from scratch. For the record the two agree
  where they should: the recovery requirement at `tau = 1.0 s` is 1.636°, the same
  0.2 m/s² of acceleration by a different route.

- ~~Ball travel during a 150 ms latency window at 300 mm/s: **45 mm**.~~
  **Struck 2026-09-05.** The 300 mm/s was withdrawn 2026-09-03 as invented, and
  nothing depends on the bullet any more: the envelope's latency term is
  `150 ms × 200 mm/s = 30 mm`, computed in `docs/archive/notation.md` §9 from the peak speed
  that survived. The 150 ms itself is still **provisional** and still needs a
  basis — sensor frame interval plus servo step response, on the hardware pull.
- Distinguishable tilt steps = usable servo travel ÷ servo deadband. The geometry
  ratio cancels — **geometry does not set how many tilt steps you get, only how
  they are spent**: wide range with coarse steps, or narrow range with fine ones.
- Worst-case leg force is not a live constraint at this scale (2.7 g ball).
