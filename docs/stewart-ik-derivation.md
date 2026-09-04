# Inverse Kinematics — 6-RSS Stewart Platform

**Phase 0 working document.** Rotary servos, fixed-length rods.

> **Note on completeness.** Everything below marked *derived* is yours, from
> sessions on 2026-08-20 and 2026-08-28. Stage 2's closed form was the last
> substantial piece of Phase 0's mathematics; it is now solved and verified
> numerically. What remains is the branch choice, the rotation convention, the
> anchor parameterisation and the numerical FK — all listed in §6.

---

## 1. Notation key

### Frames

| Symbol | Meaning |
|---|---|
| `{W}` | **World / base frame.** Origin fixed to the base, axes fixed to the table. Never moves. |
| `{P}` | **Platform frame.** Origin at the platform centre, axes glued to the plate — they tilt when it tilts. |

### Quantities

| Symbol | Meaning | Frame | Known before solving? |
|---|---|---|---|
| `T` | position of the platform centre | `{W}` | **yes** — half of the commanded pose |
| `R` | platform orientation; converts a vector's `{P}` components into `{W}` components | `{P} → {W}` | **yes** — the other half of the pose |
| `p_i` | position of platform anchor *i* on the plate | `{P}` | **yes** — a design constant, off the ring drawing. Never changes. |
| `b_i` | position of servo *i*'s shaft | `{W}` | **yes** — a design constant, bolted to the base |
| `a` | servo arm length | scalar | **yes** — design parameter, constrained to horns you can buy |
| `d` | push-rod length | scalar | **yes** — *measure it after cutting*, don't assume nominal |
| `q_i` | world position of platform anchor *i* | `{W}` | intermediate (stage 1 output) |
| `L_i` | leg vector, servo shaft → platform anchor | `{W}` | intermediate |
| `h_i` | position of servo *i*'s arm tip | `{W}` | **no** — determined by `alpha_i` |
| `alpha_i` | servo *i*'s angle | scalar | **no — this is the unknown** |
| `n_i` | unit normal of servo *i*'s rotation plane | `{W}` | **yes** — design constant |
| `u_i` | unit vector in that plane; direction of `alpha_i = 0` | `{W}` | **yes** — your choice of zero |
| `v_i` | `n_i × u_i`; the second in-plane axis | `{W}` | **yes** — follows from `n_i`, `u_i` |
| `w_i` | `L_i · n_i`; signed distance of anchor *i* off servo *i*'s plane. Only the horizontal part of `L_i` contributes, since `n_i` is horizontal | scalar | intermediate |
| `rho_i` | `sqrt(d² − w_i²)`; the rod's length once projected into the plane — what is left of `d` after clearing it. `rho_i ≤ d`, equal only at `w_i = 0` | scalar | intermediate |

`(u_i, v_i)` is an orthonormal basis for servo *i*'s rotation plane, written in
world components. It is what lets a 2D problem be posed inside a tilted 3D plane.

Index `i` runs 1…6. All vectors are **column** vectors.

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
| Branch rule for the `±` | §5.5. The only thing between the closed form and a working `ik()` |
| `R` composed from three angles | The rotation convention is a choice — pick it, state it, stick to it |
| Parameterisation of `p_i` | Radius, angular pattern, and rotation relative to the base ring. `b_i` and `n_i` are done — see §8 |
| Numerical forward kinematics | Needed for the round-trip check. No closed form exists — that's the interesting part |

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

Parameters: `r_b`, `beta ∈ (0°, 60°)` (30° = regular hexagon), `delta ∈ [0°, 180°)`.

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

`delta` is tuned, not fixed: minimise `J(delta) = max over envelope and legs of
|L_i·n_i|` — the out-of-plane component, which inflates `P` while contributing
nothing to `C`. 1-D search. Note `J` depends on `p_i`, so `delta` is an inner
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
assembly datum checkable by eye.

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

## Appendix — results from Phase 0 analysis

- Minimum tilt to arrest a 200 mm/s ball over 100 mm: **1.635°**.
  *This is a minimum, not a design target.* Latency, disturbance rejection and
  friction variation all demand margin above it.
- Ball travel during a 150 ms latency window at 300 mm/s: **45 mm**.
- Distinguishable tilt steps = usable servo travel ÷ servo deadband. The geometry
  ratio cancels — **geometry does not set how many tilt steps you get, only how
  they are spent**: wide range with coarse steps, or narrow range with fine ones.
- Worst-case leg force is not a live constraint at this scale (2.7 g ball).
