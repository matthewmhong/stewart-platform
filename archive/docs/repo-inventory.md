# Repository inventory

**2026-09-08.** Everything below is read off the code in `stewart/`,
`test_kinematics.py` and `demo.py`. `docs/` was consulted only for §5, the
cross-reference, and where a convention has to be attributed to the document
that settles it. Where the code implements something other than what a document
says, this file records what the code does and flags the divergence.

Every residual quoted under **VERIFIED BY** comes from a re-run of the module on
2026-09-08 on this working tree, not from a document. Command and exit status
per module are in §6.

Symbols are `docs/notation.md`'s. Two things it leaves unnamed appear in code
under local names — `rod_i = q_i - h_i` and `tangent_i = -u_i sin alpha_i +
v_i cos alpha_i`, both declared PROVISIONAL and local in
`stewart/diagnostics/score_discriminators.py` (`PROVISIONAL_NAMES`). Legs are
0-indexed in code (`i = 0..5`) and 1-indexed in messages.

---

## Contents

1. [`stewart/geometry.py`](#1-stewartgeometrypy)
2. [`stewart/kinematics.py`](#2-stewartkinematicspy)
3. [`stewart/plotting.py`](#3-stewartplottingpy) · [`stewart/roundtrip.py`](#4-stewartroundtrippy) · [`stewart/__init__.py`](#5-stewart__init__py)
4. [`stewart/diagnostics/`](#6-stewartdiagnostics) — eight modules
5. [`test_kinematics.py`](#7-test_kinematicspy)
6. [Call graph](#8-call-graph)
7. [Cross-reference with the documents](#9-cross-reference-with-the-documents)
8. [State of the repo](#10-state-of-the-repo)
9. [Conventions embedded in code, and what settles each](#11-conventions-embedded-in-code-and-what-settles-each)
10. [Why each function is necessary](#12-why-each-function-is-necessary)

---

# 1. `stewart/geometry.py`

## `Geometry` (frozen dataclass)

**(a) SIGNATURE**

```
Geometry(p, b, n, u, a, d)
    p, b, n, u : ndarray (3, 6)      a, d : float
    __post_init__(self) -> None      validates and freezes; raises ValueError
    summary(self) -> str             prints and returns
```

**(b) IN WORDS.** Holds the complete list of quantities a leg's inverse
kinematics consumes — six platform anchors, six shaft centres, six servo-plane
frames, and the two link lengths — and nothing else about the machine. Its job
beyond storage is to refuse, at construction, any array that would run without
error but mean something different: a transposed anchor block, a non-unit
normal, a reference direction that does not lie in its own plane.

**(c) IN SYMBOLS**

```
Geometry  =  ( {p_i}, {b_i}, {n_i}, {u_i}, a, d ),     i = 0..5

    p_i in {P}, mm       b_i, n_i, u_i in {W}      a, d in mm

Every array is (3, 6); column i is leg i, so R q is R applied to each column.

Construction rejects unless ALL of:

    shape(p) = shape(b) = shape(n) = shape(u) = (3, 6)      exactly
    a > 0,   d > 0
    | ||n_i|| - 1 |  <=  1e-6           for every i
    | ||u_i|| - 1 |  <=  1e-6           for every i
    | n_i . u_i   |  <=  1e-6           for every i

Failing legs are named i+1 in the message.  On success the four arrays are
copied and set read-only, and the dataclass is frozen: a Geometry cannot be
mutated after construction.
```

`d > a` is **not** checked (`make_geometry`'s Notes give the reason). `p_i`'s
`z` is not constrained here — the anchor-plane convention lives in
`platform_ring`.

**(d) VERIFIED BY.** The validators themselves are **UNVERIFIED**: nothing in
the repo constructs a deliberately bad `Geometry` and checks that it raises. The
happy path is exercised by `test_kinematics.check_rotation_not_transposed`
(direct construction) and by every `make_geometry` call in every diagnostic.

---

## `Geometry.summary`

**(a) SIGNATURE** `summary(self) -> str` (also prints to stdout).

**(b) IN WORDS.** Prints the two link lengths and the annulus their difference
and sum bound, as a by-eye check on a candidate before anything is solved.

**(c) IN SYMBOLS**

```
prints    a,  d,   lo = |d - a|,   hi = d + a          mm, 3 decimal places
```

The printed `lo`/`hi` are the **two-sphere bound**, which `ik`'s docstring
records as necessary and not sufficient. `summary` prints it without that
caveat.

**(d) VERIFIED BY.** No test. Exercised by `demo.py`, which printed
`a = 18.000`, `d = 120.000`, `|d - a| = 102.000`, `d + a = 138.000` mm on
`smoke_geometry`. No residual.

---

## `base_ring`

**(a) SIGNATURE** `base_ring(r_b: float, beta: float, delta: float) -> (b, n, u)`,
each `(3, 6)`. `beta`, `delta` in **degrees**.

**(b) IN WORDS.** Places the six servo shafts on a ring in three mirror-symmetric
pairs, and gives each servo the plane its arm sweeps in. It answers where the
motors are bolted and which way each faces — the half of the geometry that is
fixed to the base and never moves.

**(c) IN SYMBOLS**

```
s        =  (-1, +1, -1, +1, -1, +1)                        within-pair sign

theta_i  =  120 * floor(i / 2)  +  s_i * beta          deg,  i = 0..5
b_i      =  r_b ( cos theta_i, sin theta_i, 0 )        mm, {W},  b_i . z = 0

psi_i    =  theta_i  +  90  +  s_i * delta             deg
n_i      =  ( cos psi_i, sin psi_i, 0 )                horizontal, unit
u_i      =  z x n_i  =  ( -sin psi_i, cos psi_i, 0 )   so  u_i . z = 0

Domain:   r_b > 0,   beta in (0, 60) OPEN,   delta in [0, 180) HALF-OPEN
Codomain: three (3, 6) arrays.

Self-asserts at every call:  unit |n_i| , |u_i| ;  n_i . u_i = 0 ;  and at
beta = 30 exactly, the sorted theta gaps are 60 deg to 1e-7 (regular hexagon).
```

Index base: the code writes `floor(i/2)` with `i = 0..5`; derivation §8 writes
`120·floor((i-1)/2)` with `i = 1..6`. Same ring, different index base.

**(d) VERIFIED BY.** Its own asserts, which run on every call in every module.
No test names it. Indirectly: `test_kinematics` rows 2–4, which are built on it
(`6.123e-16`, `5.551e-17`, `0.000e+00`), and
`zhome_datum.check_flat_arm_leg_independence`, which reports
`max |u_i . z| = 0.000e+00` exactly over 540 built geometries.

---

## `platform_ring`

**(a) SIGNATURE** `platform_ring(r_p: float, beta_p: float, h_p: float = 0.0) -> p`,
`(3, 6)`. `beta_p` in **degrees**.

**(b) IN WORDS.** Places the six ball-joint anchors on the moving plate, in the
platform's own frame, from the same three-pair skeleton the base uses. It fixes
the one thing the platform contributes to a leg's problem: where the rod's upper
end is attached, and how far that plane sits below the origin the control law
commands.

**(c) IN SYMBOLS**

```
s       =  (-1, +1, -1, +1, -1, +1)
phi_i   =  120 * floor(i / 2)  +  s_i * beta_p              deg, i = 0..5
p_i     =  ( r_p cos phi_i,  r_p sin phi_i,  -h_p )         mm, {P}

Domain:   r_p > 0,   beta_p in (0, 60) OPEN,   h_p UNRESTRICTED (not checked,
          may be 0 or negative)
Codomain: (3, 6)

Self-asserts:  hypot(p_x, p_y) = r_p ;  p_z = -h_p ;  hexagon gaps at
beta_p = 30.
```

Two conventions are embedded and neither is a free choice here: the six anchors
are **coplanar** (one shared `z`), and the plane sits `h_p` **below** `{P}`'s
origin — settled 2026-09-03, `notation.md` §4. There is deliberately **no**
platform-rotation parameter `mu`; `notation.md` §4 records it as a gauge, and
the docstring records that `phi_i` shares `theta_i`'s skeleton so that the base
and platform mirror permutations come out identical (`[1, 0, 5, 4, 3, 2]`),
which is what lets one scalar `delta` serve all six planes.

**(d) VERIFIED BY.** No dedicated test. `zhome_datum` check A exercises it over
540 built geometries and reports the guard firing: **360 of 900** requested grid
points were refused by `0 < beta_p < 60` (at `beta_p = 60` and `80`), reported
rather than worked around. Its outputs enter `test_kinematics`'s fixture
(`h_p = 0.10`) whose rows 1, 3, 4 give `0.000e+00`, `5.551e-17`, `0.000e+00`.

---

## `make_geometry`

**(a) SIGNATURE**
`make_geometry(r_b, beta, delta, r_p, beta_p, a, d, h_p=0.0) -> Geometry`.

**(b) IN WORDS.** Composes a base ring and a platform ring into the single
object every solver takes. Plumbing only: it computes nothing, and `a` and `d`
are arguments rather than anything derived here.

**(c) IN SYMBOLS**

```
(b, n, u)  =  base_ring(r_b, beta, delta)
p          =  platform_ring(r_p, beta_p, h_p)
return        Geometry(p, b, n, u, a, d)

Validation is entirely delegated: ValueError propagates from base_ring,
platform_ring or Geometry.  NO check that d > a is made anywhere.
```

**(d) VERIFIED BY.** `test_kinematics.fixture_geom`, rows 1/3/4:
`0.000e+00`, `5.551e-17`, `0.000e+00` (tol `1e-12`). Independently by
`zhome_datum` check A, whose closed form is written in the input parameters and
compared against the library's anchors over 540 grid points:

| residual | max | relative |
|---|---|---|
| leg-independence of `\|g_i - a u_i\|`, `max_i - min_i` | `9.948e-14` | `1.501e-15` |
| `\|g_i - a u_i\|^2` vs closed form | `1.637e-11` | `2.103e-15` |

verdict `AGREE` at a 1e-12 relative threshold.

---

## `smoke_geometry`

**(a) SIGNATURE** `smoke_geometry(seed: int = 0) -> Geometry`.

**(b) IN WORDS.** A `Geometry` that passes every construction check and means
nothing. It exists so the plotting and the round-trip harness can be
shape-checked without a real layout, and it is documented to look wrong when
plotted.

**(c) IN SYMBOLS**

```
(b, n, u)  =  base_ring(r_b = 90, beta = 25, delta = 12)

ang        =  (-20, 25, 95, 145, 215, 265) deg          NOT the s_i skeleton
p_i        =  60 ( cos ang_i, sin ang_i, 0 )  +  eta_i,
                eta_i ~ N(0, 3 mm) i.i.d. in ALL THREE components,
                from np.random.default_rng(seed)

a = 18 mm,   d = 120 mm
```

Because the jitter hits the `z` row too, `smoke_geometry`'s anchors are **not
coplanar** — it does not satisfy `platform_ring`'s `p_z = -h_p` convention, by
design.

**(d) VERIFIED BY.** `demo.py`, re-run 2026-09-08. Three of the four
`known_poses` rows return `ik Unreachable (leg 1, near)`; the `yaw +90 deg about
z` row round-trips at `pos err 0.3163 mm`, `ang err 0.7296 deg` — the
near-singular second assembly mode `docs/cc-fk-gate.md` §6 records. No residual
is asserted anywhere; nothing tests it.

---

# 2. `stewart/kinematics.py`

## `Unreachable(ValueError)`

**(a) SIGNATURE** `Unreachable(leg: int, direction: str, ratio: float | None = None)`;
attributes `.leg` (1-indexed), `.direction`, `.ratio`.

**(b) IN WORDS.** The exception one leg raises when no point on its servo-arm
circle lies within one rod length of the commanded anchor. It carries which leg
failed, which way, and by how much, so a caller can report a workspace boundary
rather than a crash.

**(c) IN SYMBOLS**

```
raised  <=>  |P_i| > C_i                       (see ik)

direction  =  "far"   if  P_i > 0             ( <=> P_i >  C_i , since C_i >= 0 )
           =  "near"  otherwise               ( <=> P_i < -C_i )
ratio      =  P_i / C_i   if C_i != 0,  else None       ( |ratio| > 1 )
leg        =  i + 1                                     1-INDEXED
```

The docstring states the two directions as `P_i > C_i` / `P_i < -C_i`; the code
branches on `sign(P_i)`. The two agree wherever the exception can be raised.
The **lowest-numbered** failing leg is the one reported (`flatnonzero(...)[0]`);
the others are not.

**(d) VERIFIED BY.** No dedicated test. Exercised: `demo.py` (`leg 1, near`, three
rows); `branch_check` `[2a]`, 4 unreachable `(pose, leg)` pairs out of
`4365 x 6`; the gate reports `poses ik called unreachable: 0` over 14 436.

---

## `stage1`

**(a) SIGNATURE** `stage1(geom, R: (3,3), T: (3,)) -> q: (3, 6)`.

**(b) IN WORDS.** Converts the six anchor positions from the plate's own frame
into world coordinates for a commanded pose. It answers the first question any
leg calculation needs: given where the platform is and how it is tilted, where
in the world is each ball joint.

**(c) IN SYMBOLS**

```
q_i  =  T  +  R p_i                     i = 0..5

Map:      (R, T) in SO(3) x R^3   ->   q in R^(3x6)
Applied:  R @ p   (from the LEFT, on columns).  p @ R would transpose R
          silently.

Raises ValueError unless shape(R) = (3, 3).  T is reshaped to (3, 1) and
broadcast across the six columns.
```

Argument order `(R, T)` — orientation first — is the repo convention recorded in
`notation.md`'s Conventions block (settled 2026-09-07).

**(d) VERIFIED BY.** `test_kinematics` rows 1 and 2:
`stage1(R=I, T=0) == p` residual **`0.000e+00`**, and
`Rz(90) sends (10,0,0) -> (0,10,0)` residual **`6.123e-16`** — the row that
separates `R @ p` from `p @ R`, since a transposed `R` would give `(0,-10,0)`.

---

## `legs`

**(a) SIGNATURE** `legs(geom, R, T) -> L: (3, 6)`.

**(b) IN WORDS.** The vector each leg has to span: from its own servo shaft to
its own anchor. It is what makes the six problems independent — leg `i`'s vector
mentions no other leg.

**(c) IN SYMBOLS**

```
L_i  =  q_i - b_i  =  R p_i + T - b_i             {W}, mm

|L_i| is NOT the rod length; the arm makes up the difference.
```

No validation of its own; `stage1`'s applies.

**(d) VERIFIED BY.** No test names it. Reached indirectly through `ik`:
`branch_envelope` rebuilds `L` from `base_ring`/`platform_ring` outputs
independently and compares the resulting closed-form angle against the library
`ik` — `max |ik() - alpha_minus| = 8.882e-16` over 40 samples.

---

## `w`

**(a) SIGNATURE** `w(geom, R, T) -> (6,)`.

**(b) IN WORDS.** The part of each leg vector that sticks out of its servo's
plane — the component the servo has no authority over, and the quantity the
inner `delta` tune was originally built to minimise.

**(c) IN SYMBOLS**

```
w_i  =  L_i . n_i                        mm, signed

Under horizontal shafts only the horizontal part of L_i contributes.
w_i enters |L_i|^2 (hence P_i) but never C_i, which is the whole content of
the "two-sphere bound is not sufficient" result.
```

**(d) VERIFIED BY.** **UNVERIFIED, and uncalled.** No module, test or diagnostic
in the repo calls `w()` — the diagnostics that need `w_i` (`azimuth_symmetry`,
`zhome_bracket`) each form it inline. `docs/phase-0-design-log.md` records a
4000-pose bitwise regression when `w` was repointed at `legs()`; that check is
not in the repo.

---

## `arm_tips`

**(a) SIGNATURE** `arm_tips(geom, alphas: (6,) radians) -> tips: (3, 6)`.

**(b) IN WORDS.** Places each servo's arm tip in the world, given the six servo
angles. The tip is confined to a circle of radius `a` about the shaft, lying in
that servo's rotation plane, so one angle fixes it.

**(c) IN SYMBOLS**

```
h_i  =  b_i  +  a ( u_i cos alpha_i  +  v_i sin alpha_i ),      i = 0..5
with  v_i = n_i x u_i = z   (exactly, while n_i is horizontal)

so    alpha_i = 0  =>  h_i = b_i + a u_i,  and  h_i . z = 0   (arm flat)

Returns (3, 6), column i is leg i.  alpha in RADIANS.
Raises ValueError unless alphas has exactly 6 entries.
```

`v_i` is **computed** as `n_i x u_i`, not assumed to be `z`, so a canted shaft
would not silently give a wrong answer.

**(d) VERIFIED BY.** `test_kinematics` rows 3 and 4:
`| |arm_tips(0) - b| - a | = 5.551e-17` (spread `1.110e-16`), and
`arm_tips(0) == b + a*u` **`0.000e+00`** exactly — the row that catches a
`u`/`n` swap, with a control row showing `b + a*n` passes row 3 at the same
`5.551e-17`. Independently, `zhome_datum.confirm_zhome_against_arm_tips`:
`max_i | |q_i - h_i| - d | = 2.220e-16` over 8 samples.

---

## `ik`

**(a) SIGNATURE** `ik(geom, R, T) -> alphas: (6,) radians`. Raises `Unreachable`.

**(b) IN WORDS.** The inverse kinematics: a commanded platform pose in, six
servo angles out, solved in closed form and leg by leg. It answers the only
question the machine will be asked at run time, and it raises rather than
returning a boundary value when a leg cannot reach.

**(c) IN SYMBOLS**

```
For each leg i = 0..5:

    q_i  =  R p_i + T
    L_i  =  q_i - b_i
    v_i  =  n_i x u_i

    M_i  =  L_i . u_i
    N_i  =  L_i . v_i                              = q_i . z  (b_i . z = 0)
    P_i  =  ( |L_i|^2 + a^2 - d^2 ) / ( 2 a )      delta-free
    C_i  =  hypot(M_i, N_i)  =  sqrt(|L_i|^2 - w_i^2)

    REACHABILITY, tested BEFORE the arccos:
        |P_i| > C_i   =>   raise Unreachable(i+1, ...)
        no np.clip(P_i / C_i, -1, 1) anywhere

    BRANCH, fixed:
        alpha_i  =  atan2(N_i, M_i)  -  arccos( P_i / C_i )        MINUS only

Map:  (R, T)  ->  alpha in R^6, radians, measured from u_i toward v_i,
      positive right-handed about n_i;  alpha_i = 0 lays the arm along u_i.
Returned RAW (not wrapped); with N_i > 0 it already lies in (-pi, pi).
```

The minus branch is fixed, not a parameter: under horizontal shafts `v_i = z`
exactly, so `N_i = q_i . z > 0` for any pose the platform can hold, `phi_i` lies
in `(0, pi)`, and the minus root is the one continuously connected to
`alpha_i = 0`. The docstring states the dependency: cant the shafts and the
branch choice reopens.

**(d) VERIFIED BY.** Three independent measurements:

* `branch_check` `[1b]` — at the flat-arm datum every leg returns
  `alpha_i = 0` on the minus branch, `max |alpha| = 8.882e-16`, same branch on
  all six.
* `branch_envelope` — `max |ik() - alpha_minus| = 8.882e-16` over 40
  `(pose, z_home)` samples against an independently coded closed form; branch
  giving `alpha ~ 0` at home is `-` at every feasible `z_home`; branch-flip step
  outliers over a 1440-step full-circle precession: **0**; loop closure
  `<= 1.78e-15`.
* the gate — 14 436 poses, `0` unreachable, round trip closing at
  `1.8532e-13 mm`.

`score_discriminators`'s own self-check reports
`worst |rod_i| - d = 4.441e-16 r_b` over every pose measured, i.e. the rods do
close at `ik`'s own angles.

---

## `FKNotConverged(RuntimeError)`

**(a) SIGNATURE**
`FKNotConverged(residual_mm, iterations, tol_mm, reason="cap")`; attributes
`.residual_mm`, `.iterations`, `.tol_mm`, `.reason`.

**(b) IN WORDS.** What `fk` raises instead of handing back a best effort. A pose
that did not converge is not a pose, and returning one silently is how a
round-trip gate comes to pass while wrong.

**(c) IN SYMBOLS**

```
raised  <=>  max_i |f_i|  >  tol   at the stopping iterate

reason in { "cap"      : iteration cap reached,
            "stalled"  : neither Newton, backtracked Newton nor LM reduced
                         max_i |f_i|,
            "singular" : |q_i - h_i| = 0 for some i, or f not finite }
```

**(d) VERIFIED BY.** Gate part (3): with `tol = 1e-14` and `1e-15` — below the
measured arithmetic floor — acceptance fails on **234** poses, `raised = 234`,
against `0` at every tolerance from `1e-8` to `1e-13`. Gate part (6b): of 400
random seeds per fixture, 55–77 raise, with residuals far from tolerance.

---

## `FK_TOL_MM`, `FK_MAX_ITER`

**(a) SIGNATURE** `FK_TOL_MM = 1e-9` (mm), `FK_MAX_ITER = 100`.

**(b) IN WORDS.** The acceptance threshold on the rod-closure residual and the
cap that turns a non-converging pose into an exception rather than a hang. The
tolerance is deliberately not the stopping rule.

**(c) IN SYMBOLS**

```
ACCEPT  <=>  max_i |f_i|  <=  FK_TOL_MM,     f_i = |q_i - h_i| - d    [mm]

applied ONCE, to the converged iterate.  Floor: eps * 120 ~ 2.7e-14 mm at the
gate's working scale; measured attainable floor 1.4e-14 .. 7.1e-14 mm.
Ceiling: any hardware tolerance is >= 1e-3 mm.  1e-9 sits between.
```

**(d) VERIFIED BY.** Gate part (3), re-run: worst `|dT|` is
**`1.2681e-13 mm` identically at `tol = 1e-8, 1e-9, ..., 1e-13`** — six decades,
zero movement, `0` raised; `1e-14` and `1e-15` fail acceptance on 234 poses.
Max iterations observed **8** against a cap of 100.

---

## `exp_so3`, `log_so3`, `geodesic_angle`

**(a) SIGNATURE**
`exp_so3(omega: (3,)) -> R: (3,3)` ·
`log_so3(R: (3,3)) -> omega: (3,)` ·
`geodesic_angle(R_a, R_b) -> float` (radians).

**(b) IN WORDS.** The rotation-vector parameterisation the FK solver iterates in,
its inverse, and the angle between two orientations. They exist so the solver
and the gate can talk about rotation without choosing a roll/pitch/yaw
convention — an axis and an angle name a rotation without an ordered sequence of
elementary ones.

**(c) IN SYMBOLS**

```
exp_so3:   K = [omega]_x ,  th = |omega|

    th >= 1e-12 :  R = I + (sin th / th) K + ((1 - cos th) / th^2) K^2
    th <  1e-12 :  R = I + K + K^2 / 2                (series; avoids 0/0)

log_so3:   A = R - R^T ,  v = vee(A)/2 = sin(th) * axis ,
           s = |v| ,  c = clip((tr R - 1)/2, -1, 1)

    c > -0.9   :  th = atan2(s, c) ;  return v * (th / s)
                  ( s < 1e-12  ->  return v itself, exact to this order )
    c <= -0.9  :  th near pi; a a^T = ( (R + R^T)/2 - c I ) / (1 - c),
                  axis from the column of largest diagonal, normalised,
                  sign fixed by  axis . v >= 0 ;  return axis * th

geodesic_angle(R_a, R_b)  =  | log_so3( R_a^T R_b ) |          radians
```

The docstrings record why this and not `arccos((tr R - 1)/2)`: the trace carries
`theta` only at second order, so that form has a floor of `~sqrt(2 eps)`, while
the antisymmetric part is linear in `theta` and has none.

**(d) VERIFIED BY.** Gate part (1a) and its implementation check, re-run:

| quantity | measured |
|---|---|
| `\|log_so3\|` absolute error vs a known angle, `3` rad down to `1e-15` rad | `5.684e-14` .. `2.633e-15` deg, no floor |
| `arccos` absolute error at the same angles | saturates at `2.41e-06` deg, predicted `sqrt(2 eps) = 1.21e-06` deg |
| the two forms against each other, angles `>= 1e-2` rad | agree to `4.6e-12` relative — one quantity |
| `max \| log_so3(exp_so3(w)) - w \|`, 20 000 rotations, angles drawn at both the `0` and `pi` ends | **`1.404e-15`** (worst at `theta = 3.141593`) |
| `log_so3(exp_so3((pi,0,0)))` / `log_so3(I)` | `[3.14159265359, 0, 0]` / `[0, 0, 0]` |

---

## `fk_residual`

**(a) SIGNATURE**
`fk_residual(geom, tips: (3,6), R, T) -> (f: (6,), rvec: (3,6), norm: (6,))`.

**(b) IN WORDS.** How far each of the six rods is from closing at a trial pose,
with the arm tips already fixed. It is the function the forward solve drives to
zero, and it is a length, so the tolerance on it is a length too.

**(c) IN SYMBOLS**

```
q_i    =  T + R p_i
rvec_i =  q_i - h_i                       ( = rod_i , tip to anchor )
norm_i =  |q_i - h_i|
f_i    =  norm_i  -  d                    UNSQUARED, mm, signed
```

**(d) VERIFIED BY.** Not tested in isolation. Every gate number is built on it;
`zhome_datum.confirm_zhome_against_arm_tips` measures the same quantity
independently at `2.220e-16`.

---

## `fk_jacobian`

**(a) SIGNATURE**
`fk_jacobian(geom, R, rvec, norm) -> (J: (6,6), e: (3,6))`.

**(b) IN WORDS.** How the six rod-closure errors move when the platform is
nudged — three translations and three rotations. It is what makes the forward
solve a Newton method rather than a search, and its sign depends on which side
the rotation increment is applied.

**(c) IN SYMBOLS**

```
e_i  =  (q_i - h_i) / |q_i - h_i|             from the ACTUAL norm, not from d

    df_i / dT      =   e_i^T
    df_i / domega  =  -e_i^T [R p_i]_x   =   ( R p_i  x  e_i )^T

under the LEFT perturbation      R  ->  exp([omega]_x) R
(right perturbation would give  +e_i^T R [p_i]_x , which differs by an R^T as
well as a sign, so the two cannot be reconciled by flipping one)

J is (6, 6): row i is leg i; columns are (T_x, T_y, T_z, w_x, w_y, w_z).
UNITS: columns 0-2 dimensionless (mm of residual per mm), columns 3-5 mm/rad.
```

**(d) VERIFIED BY.** Gate part (1), re-run — central differences taken through
the same left perturbation, 348 samples over 4 geometries × 3 `z_home` × 29
poses, all perturbed off the solution:

| `h` (mm) | worst entrywise rel. | median | worst Frobenius |
|---|---|---|---|
| 5e-2 | 1.609e-05 | 1.050e-05 | 1.076e-05 |
| 5e-3 | 1.609e-07 | 1.050e-07 | 1.076e-07 |
| **5e-4** | **9.552e-08** | 1.168e-09 | 1.078e-09 |
| 5e-5 | 1.568e-06 | 3.948e-09 | 3.916e-11 |
| 5e-6 | 1.142e-05 | 3.720e-08 | 3.869e-10 |

Negative control at fixture A, one pose: endorsed form `9.781e-10`; sign-flipped
left `2.000e+00`; right perturbation `4.307e-01`; right sign-flipped
`1.974e+00` — the test discriminates by 8–10 orders.

---

## `_cond_and_sigma` (private, but imported across modules)

**(a) SIGNATURE**
`_cond_and_sigma(J, char_len: float | None) -> (cond, sigma_min, sigma_max)`,
`(None, None, None)` when `char_len is None`.

**(b) IN WORDS.** Turns the Jacobian's mixed units into one dimensionless matrix
so its singular values mean something, by naming a length at which a radian of
rotation is worth so many millimetres. It refuses to pick that length itself.

**(c) IN SYMBOLS**

```
S^-1  =  diag( 1, 1, 1, 1/l, 1/l, 1/l ),      l = char_len

sv    =  svd( J S^-1 )
cond  =  sigma_max / sigma_min      ( = inf when sigma_min = 0 )

char_len = None  ->  (None, None, None).  NO DEFAULT: the choice is
notation.md sec.12's open item and defaulting it here would settle it by
accident.
```

**(d) VERIFIED BY.** The number itself is not tested — it cannot be until the
length is settled. Its **dependence** is measured and reported, twice: gate
part (2), worst `cond` over the coarse grid at four candidate lengths —

| fixture | `r_b` | `r_p` | `d` | `a` |
|---|---|---|---|---|
| A | 4.183 | 4.175 | 4.20 | 12.81 |
| C | 3.712 | 3.075 | 4.05 | 3.737 |
| E | 7.397 | 6.285 | 6.695 | 7.185 |
| F | 6.317 | 6.365 | 6.54 | 19.51 |

— and `score_discriminators` part (d), which counts rank shifts when the length
is switched (e.g. `r_p` vs `d` on `J_fk`: 90 of 363 candidates move by more than
5% of the field, `rho = 0.9852`).

---

## `fk_solve`

**(a) SIGNATURE**

```
fk_solve(geom, alphas, R0, T0, *, tol=FK_TOL_MM, max_iter=FK_MAX_ITER,
         char_len=None) -> dict
    dict keys: R, T, residual_mm, iterations, lm_steps, cond, sigma_min,
               sigma_max, char_len, so3_drift, residual_history
```

**(b) IN WORDS.** The forward kinematics with its working shown: six servo
angles in, a pose out, plus the residual, the iteration count, whether the
damped fallback fired and how well conditioned the mechanism was there. It is
the entry point the gate uses, because a pass has to carry its own evidence.

**(c) IN SYMBOLS**

```
Unknowns:  x = (T, omega) in R^6 .   Equations:  f_i(x) = 0 ,  i = 0..5.
Tips are fixed by alphas, so the system is SQUARE.

iterate:
    f, rvec, norm  =  fk_residual(...)
    res            =  max_i |f_i| ;   res = 0  ->  stop
    breakdown ( norm_i <= 0  or  f not finite )  ->  raise "singular"

    Newton:   J dx = -f
    line search: t = 1, halve up to 30 times ( to 2^-30 ~ 1e-9 of the step );
                 accept the first t with max|f(x + t dx)| < res

    if nothing accepted -> Levenberg-Marquardt, MARQUARDT scaling:
        ( J^T J + lam diag(J^T J) ) dx = -J^T f ,   lam = 1e-3, x10, up to 40
        ( lam * I is NOT used: it would add a millimetre to a radian and so
          need exactly the characteristic length this module refuses to pick )
        accept the first lam whose step reduces res;  count it in lm_steps

    if still nothing accepted -> STOP, reason = "stalled"        <- stopping rule
    update:  R <- exp([omega]_x) R  (LEFT),  T <- T + dT ;  omega re-zeroed

acceptance, ONCE, on the converged iterate:
    max_i |f_i| <= tol   else raise FKNotConverged(res, it, tol, reason)

then:   cond, sigma_min, sigma_max = _cond_and_sigma(J, char_len)
        so3_drift = max | R^T R - I |            (before projection)
        R <- U V^T   from svd(R)                 (polar projection onto SO(3))
```

Stopping is **stagnation**, not the tolerance; the tolerance is an acceptance
test applied once. `lm_steps` counts **accepted** LM steps only — counting
attempts reported LM on 96% of solves, because every converged solve ends with
one iteration where nothing reduces the residual.

**(d) VERIFIED BY.** Gate parts (3), (4), (7), re-run:

| quantity | measured |
|---|---|
| poses | 14 436 over 4 fixtures × 3 grid levels |
| commanded mode returned | 14 436; different mode `0`; non-convergence `0`; `ik` unreachable `0` |
| worst `\|T_fk - T_cmd\|` | **`1.8532e-13 mm`** (fixture F, az 62.50°, tilt 1.974°, `z_home` 133.750 mm) |
| worst `\|log_so3(R_cmd^T R_fk)\|` | **`7.8203e-14 deg`** (fixture C) |
| worst residual at convergence | `2.842e-14 mm` (tol `1e-9`) |
| iterations | min 4, median 5, mean 5.04–5.43, max 8, cap 100 |
| LM steps accepted | 36 / 60 / 246 / 289 of 3600 per fixture |
| worst `cond`, `1/sigma_min` at `char_len = r_b` | 9.518, 3.980 — PROVISIONAL |
| residual-to-pose bound `\|\|dx\|\| <= sqrt(6)(max_i\|f_i\| + eta)/sigma_min`, `eta = 8 eps (d + \|T\|)` | worst measured/bound **`0.074`**; with `eta` omitted it reads `1.535`, i.e. violated |

---

## `fk`

**(a) SIGNATURE**
`fk(geom, alphas, R0, T0, *, tol=FK_TOL_MM, max_iter=FK_MAX_ITER) -> (R, T)`.
Raises `FKNotConverged`.

**(b) IN WORDS.** The two-line wrapper that answers the question the round trip
asks: six angles in, the pose they put the platform in, out. A small residual
says the legs close, not that the pose is the one that was commanded — a 6-RSS
forward kinematics has several real solutions and this returns the one its seed
leads to.

**(c) IN SYMBOLS**

```
fk  =  pi_(R,T)  o  fk_solve                       returns (R, T), in that order

Domain:   alphas in R^6 (radians), seed (R0, T0)
Codomain: (R, T) in SO(3) x R^3, mm

The SEED must not be the true pose: seeded at the truth the residual is zero
and the solver returns immediately, so the round trip would pass for ANY ik.
Unreachable does NOT apply here — in forward kinematics the anchors are what
is being solved for; the failure mode is FKNotConverged.
```

**(d) VERIFIED BY.** The gate, as above (via `fk_solve`), and `demo.py` via
`stewart.roundtrip.round_trip`, where the `yaw +90` row returns a genuine
second assembly mode at `0.3163 mm` / `0.7296 deg`.

---

# 3. `stewart/plotting.py`

Nothing here solves kinematics; every function is handed points, or an angle
that only places a point on a circle already fixed by the geometry.

## `draw_pose`

**(a) SIGNATURE**
`draw_pose(geom, q: (3,6), h=None, T=None, R=None, ax=None, title=None, labels=True) -> ax`.

**(b) IN WORDS.** Draws one platform pose in 3D — base ring, platform ring, and
either the real arms and rods or dashed shaft-to-anchor guide lines. It is the
debugger for errors that produce entirely plausible numbers, such as a
transposed rotation.

**(c) IN SYMBOLS**

```
draws:  ring through b_[0..5,0] ,  ring through q_[0..5,0]

  h given     :  tips = b_i + a(cos h_i u_i + sin h_i v_i) ;
                 segments b_i -> tip_i (arm)  and  tip_i -> q_i (rod)
  h omitted   :  dashed segments  b_i -> q_i

  triad {W} at the origin always;  triad {P} at T with axes R[:,k] iff both
  R and T are given;  leg labels i+1 (1-INDEXED) at b_i

  axis cube: centred on the enclosing box of every drawn point set,
             half-width 0.5 max(hi - lo) * 1.05, equal aspect

Raises ValueError unless shape(q) = (3, 6).  h in RADIANS.
```

**(d) VERIFIED BY.** **UNVERIFIED.** Called only by `demo.py` (which writes
`demo.png` and checks nothing) and by `compare_branches`/`animate`, neither of
which is called anywhere.

## `arm_circles`

**(a) SIGNATURE** `arm_circles(geom, ax, n_points=64) -> list[Line3D]`.

**(b) IN WORDS.** Overlays the circle each arm tip is confined to, so that a
reachability failure can be seen rather than inferred from a raised exception.

**(c) IN SYMBOLS**

```
circle_i(t)  =  b_i  +  a ( cos t  u_i  +  sin t  v_i ),   t in [0, 2 pi),
                n_points samples,  v_i = n_i x u_i
```

**(d) VERIFIED BY.** **UNVERIFIED.** Called only by `demo.py`.

## `compare_branches`

**(a) SIGNATURE**
`compare_branches(geom, q, h_plus, h_minus, titles=None) -> (fig, (ax_l, ax_r))`.

**(b) IN WORDS.** Puts the two arm branches for one pose side by side under a
single shared set of axis limits, so the `±` in the closed form can be looked at
rather than argued about.

**(c) IN SYMBOLS**

```
left  = draw_pose(geom, q, h = h_plus)      right = draw_pose(geom, q, h = h_minus)
both axes given the SAME cube over  { b, q, tips(h_plus), tips(h_minus) }
```

**(d) VERIFIED BY.** **UNVERIFIED, and uncalled** — nothing in the repo calls it.
(The branch question it was built for was settled numerically instead, by
`branch_check` and `branch_envelope`.)

## `animate`

**(a) SIGNATURE** `animate(geom, q_frames: (F,3,6), h_frames=None, interval_ms=40) -> (fig, anim)`.

**(b) IN WORDS.** Plays a sequence of poses with the axis limits fixed over the
whole sequence, so motion is not hidden by autoscaling.

**(c) IN SYMBOLS**

```
frame k  ->  draw_pose(geom, q_frames[k], h = h_frames[k] or None)
cube fixed once over  { b } u { q_frames[k] } u { tips(h_frames[k]) }

Raises ValueError unless shape(q_frames) = (F, 3, 6), and, when given,
shape(h_frames) = (F, 6).
```

**(d) VERIFIED BY.** **UNVERIFIED, and uncalled.**

## `plot_angles`

**(a) SIGNATURE** `plot_angles(t: (N,), alphas: (6,N) or (N,6)) -> (fig, ax)`.

**(b) IN WORDS.** Plots the six servo traces against a parameter, in degrees.
The failure mode it exists to expose is a branch flip mid-trajectory: a step in
one trace while the other five stay smooth.

**(c) IN SYMBOLS**

```
plots  deg(alpha_i(t))  for i = 0..5, labelled "leg i+1"   (1-INDEXED)
accepts (6, N) or (N, 6) and transposes the latter;  raises ValueError
otherwise.  Radians in, DEGREES on the axis.
```

**(d) VERIFIED BY.** **UNVERIFIED, and uncalled.** The branch-flip check it was
meant to serve is done numerically instead — `branch_check` `[2d]` and
`branch_envelope`'s continuity block, both by `max|dalpha| / median` step-outlier
ratio.

---

# 4. `stewart/roundtrip.py`

The generic harness: `ik` and `fk` are passed in as arguments. Distinct from
`stewart/diagnostics/roundtrip.py`, which is the specific settled gate.

## `known_poses`

**(a) SIGNATURE** `known_poses() -> list[(name, R, T)]`.

**(b) IN WORDS.** The four poses whose answers can be checked by hand — the ones
derivation §7 lists as the first thing to run before trusting anything.

**(c) IN SYMBOLS**

```
[ ( "R = I, T = 0",              I,               0 ),
  ( "yaw +90 deg about z",       Rz(pi/2),        0 ),
  ( "small roll +3 deg about x", Rx(3 deg),       0 ),
  ( "small pitch +3 deg about y",Ry(3 deg),       0 ) ]
```

The names say roll/pitch/yaw, but the matrices are elementary single-axis
rotations built directly; no composition order is involved, so derivation §6's
open rotation convention is not touched.

**(d) VERIFIED BY.** No residual of its own. Consumed by `demo.py`, where three
of four rows return `Unreachable` on `smoke_geometry` (expected — it is not a
layout) and one round-trips at `0.3163 mm`.

## `random_poses`

**(a) SIGNATURE**
`random_poses(n, *, max_translation_mm=15.0, max_tilt_deg=12.0, centre_mm=(0,0,0), seed=0) -> list[(name, R, T)]`.

**(b) IN WORDS.** Bounded random poses for a broader sweep than four hand cases,
generated so that they are valid rotations without implying any Euler
convention.

**(c) IN SYMBOLS**

```
axis ~ N(0, I_3) ,  angle ~ U[0, max_tilt_deg] ,  R = Rodrigues(axis, angle)
T = centre + U[-1, 1]^3 * max_translation_mm            mm
rng = default_rng(seed)
```

**(d) VERIFIED BY.** **UNVERIFIED, and uncalled** — nothing in the repo calls it.

## `round_trip`

**(a) SIGNATURE**

```
round_trip(geom, poses, ik, fk, *, seed_offset_mm=5.0, seed_offset_deg=3.0,
           seed=0, verbose=True) -> list[dict]
    row keys: name, status, pos_err_mm, ang_err_deg
```

**(b) IN WORDS.** Runs pose → `ik` → six angles → `fk` → pose for each pose and
tabulates how far the pose came back. It is the test that says the derivation is
self-consistent, and it deliberately starts the forward solve away from the
answer so that a solver returning its own seed cannot pass.

**(c) IN SYMBOLS**

```
per pose (R, T):
    alphas          =  ik(geom, R, T)
    R_seed          =  Rodrigues(axis ~ N(0,I), seed_offset_deg) @ R
    T_seed          =  T  +  seed_offset_mm * u,   u a random unit vector
    (R_hat, T_hat)  =  fk(geom, alphas, R_seed, T_seed)

    pos_err_mm      =  | T_hat - T |                             mm
    ang_err_deg     =  deg | log_so3( R^T R_hat ) |              deg

seed_offset_mm = seed_offset_deg = 0 is ALLOWED but prints a warning that the
run proves nothing.  NotImplementedError and Unreachable are caught per pose
and reported as a status, not raised; errors are NaN on any status != "ok".
```

The rotation metric was `arccos((tr - 1)/2)` until 2026-09-07 and is now
`|log_so3(.)|` — `_geodesic_deg` delegates to `kinematics.geodesic_angle`.

**(d) VERIFIED BY.** No self-check. Exercised by `demo.py`. The metric it now
uses is verified by the gate's `check_metric_agreement` (`1.404e-15` inversion,
no floor); the harness's own seed-offset logic is **UNVERIFIED** — no test
asserts that a zero offset would let a null `ik` pass.

---

# 5. `stewart/__init__.py`

**(a) SIGNATURE** exports `__all__ = [Geometry, base_ring, platform_ring,
make_geometry, smoke_geometry, Unreachable, stage1, legs, arm_tips, ik, fk]`;
`__version__ = "0.0.1"`.

**(b) IN WORDS.** The package's public surface as declared.

**(c) IN SYMBOLS**

```
exported     : 11 names
NOT exported : w, fk_solve, fk_residual, fk_jacobian, exp_so3, log_so3,
               geodesic_angle, FKNotConverged, FK_TOL_MM, FK_MAX_ITER,
               and every name in plotting.py and roundtrip.py
```

So the public list predates `fk_solve` and the SO(3) helpers: the diagnostics
import those from `stewart.kinematics` directly, and
`score_discriminators` imports the **private** `_cond_and_sigma` across the
module boundary.

**(d) VERIFIED BY.** Nothing imports the package by `__all__`; every module uses
explicit submodule imports. **UNVERIFIED.**

---

# 6. `stewart/diagnostics/`

`__init__.py` is a one-line docstring; the package carries no code. Every module
is a `python -m` entry point that reports and, except the gate, always exits 0.

---

## 6.1 `envelope.py` — the working envelope and the recovery model

**(a) SIGNATURE**

```
python -m stewart.diagnostics.envelope
bang_bang_accel(x0, tau) -> float           tilt_for(x0, tau=TAU) -> deg
tilt_R(azimuth_deg, magnitude_deg) -> (..., 3, 3)
envelope_poses(n_mag=5, n_az=7, tilt_limit_deg=TILT_LIMIT_DEG,
               window_deg=(30,90), full_circle=False) -> (az_deg, mag_deg)
n_poses(n_mag=5, n_az=7) -> int
constants: G, ROLL_FACTOR, TAU, X0_WORKING, X0_LATENCY, TAU_L, V_PEAK,
           TILT_BARE_DEG, TILT_LIMIT_DEG, DXY, DZ, YAW_DEG,
           AZIMUTH_WINDOW_DEG, N_MAGNITUDE, N_AZIMUTH
```

**(b) IN WORDS.** The single source of truth for what the platform is required
to do: how far it must tilt, over which azimuths, and on what pose grid. Every
other diagnostic imports the tilt limit from here rather than restating it, so
the number cannot drift between scripts the way the provisional 6° once did. The
limit is **computed** from the recovery model at import, not typed.

**(c) IN SYMBOLS**

```
Envelope (settled 2026-09-05):   dxy = 0 ,  dz = 0 ,  yaw = 0 ,  tilt <= limit
    -> purely ANGULAR: no length dimension, invariant under uniform scaling k

Recovery model (bang-bang, accelerate tau/2 then decelerate tau/2):

    acc(x0, tau)  =  4 x0 / tau^2                     m/s^2
    sin(tilt)     =  7 acc / ( 5 g )                  solid ball, rolling
                                                      without slip, (5/7) g sin
    tilt_for(x0)  =  deg arcsin( acc / (ROLL_FACTOR * g) ),  raises if |.| > 1

    g = 9.80665 ,  ROLL_FACTOR = 5/7 ,  tau = 0.5 s
    x0_bare     = 0.050 m                     -> TILT_BARE_DEG
    x0_envelope = 0.050 + tau_L * v_peak      -> TILT_LIMIT_DEG
                = 0.050 + 0.150 * 0.200 = 0.080 m
    ( asserted in code:  | X0_LATENCY - TAU_L * V_PEAK | < 1e-12 )

Poses:  tilt_R(psi, th) = Rodrigues about the HORIZONTAL AXIS at azimuth psi
        (azimuth 0 tilts about world x and lifts the +y side)

    mags = linspace(0, tilt_limit, 5) ,  azis = linspace(30, 90, 7)
    grid = { (0, 0) } u { (az, mag) : mag != 0 }
    n_poses = 1 + (n_mag - 1) * n_az = 29        magnitude 0 counted ONCE
```

`TAU_L = 150 ms` is flagged PROVISIONAL in code (`TAU_L_IS_PROVISIONAL = True`)
and in the printed report.

**(d) VERIFIED BY.** Self-reporting, re-run: `TILT_LIMIT_DEG = 10.5290 deg`,
`TILT_BARE_DEG = 6.5580 deg`, latency margin `3.9710 deg`,
`acc = 1.2800 / 0.8000 m/s^2`, grid `29` poses / `174` `w` evaluations,
full-circle equivalent `169` / `1014`. Sensitivity table `tau = 0.35 .. 1.00`.
There is **no test** of `tilt_for` against an independent value; the
cross-check that exists is `sweep_budget` part 3, which reproduces the arrest
framing's `1.635°` by a different route (`1.6356` at `g = 9.81`, `1.6361` at
`g = 9.80665`).

---

## 6.2 `azimuth_symmetry.py` — is a 60° azimuth window sufficient, and where

**(a) SIGNATURE**

```
python -m stewart.diagnostics.azimuth_symmetry
aggregates(geom, R: (K,3,3), z_home) -> (max_i |w_i| : (K,), min_i margin : (K,))
broken_geometry(kwargs) -> Geometry          circular_shift(a, k) -> ndarray
run_one(label, geom, z_home, mags) -> dict   print_block(res) -> None
FIXTURES (4), AZ_STEP_DEG = 0.25, TEST_MAGNITUDES = [limit, limit/2, 2.0]
```

**(b) IN WORDS.** Tests, rather than assumes, the claim that tilt azimuth need
only be swept over a 60° window because the leg set has D₃ symmetry. It sweeps
the **full** circle and measures the two invariances the claim needs, with a
negative control on a geometry whose symmetry has been deliberately broken.

**(c) IN SYMBOLS**

```
Aggregates, per pose, computed inline (not through ik, because the margin is
wanted where it is negative and ik raises there):

    A_w(psi)  =  max_i | w_i |                    w_i = L_i . n_i
    A_m(psi)  =  min_i ( C_i - |P_i| ) / C_i

Invariances tested, at psi sampled every 0.25 deg over [0, 360):

    P1  : A(psi)  =  A(psi + 120)          period, from C3
    M0  : A(psi)  =  A(-psi)               mirror at 0   -> would give [0, 60]
    M90 : A(psi)  =  A(180 - psi)          mirror at 90  -> gives   [30, 90]

criterion:   worst deviation / (quantity's own scale)  <=  1e-12   = HOLDS

Negative control:  p_0 -> p_0 + (0.03 r_b, 0, 0)  in {P}, D3 broken, same tests.

120 and 180 are exact multiples of the 0.25 deg step, so the tests compare
SAMPLED values with no interpolation error to mistake for a symmetry breaking.
```

**(d) VERIFIED BY.** Its own verdict, re-run 2026-09-08 (worst relative
deviation over 4 fixtures × 3 magnitudes):

| invariance | measured | verdict |
|---|---|---|
| P1, period 120 | `1.026e-13` | HOLDS |
| M0, mirror at `psi = 0` | **`2.303e-01`** | **FAILS** |
| M90, mirror at `psi = 90` | `5.648e-14` | HOLDS |
| negative control, P1 | `6.588e-01` | fails as intended |
| negative control, M90 | `3.736e-01` | fails as intended |

so the window is `[30, 90]`, not `[0, 60]` — which is exactly what
`envelope.AZIMUTH_WINDOW_DEG` encodes.

Note recorded in `docs/cc-fk-gate.md` §7 and confirmed here: the `z_home` column
of this module's `FIXTURES` is outside the feasible bracket on 3 of its 4 rows.
It does not invalidate the module — the aggregates are defined whether or not
`ik` can solve there, and `ik` is never called.

---

## 6.3 `branch_check.py` — the `-` branch, on the provisional envelope

**(a) SIGNATURE**

```
python -m stewart.diagnostics.branch_check         (part1() then part2(chosen))
axis_angle(axis, ang)   wrap(a) -> (-pi, pi]   geom(...) -> dict
closed_form(L, u, n, v, a, d) -> dict      legs_L(g, R, T) -> (3,6)
solve_z_home(r_b, beta, delta, r_p, beta_p, h_p, a, d) -> (z, rhs, g)
build_pose(...)   envelope_poses(z_home, r_b)   part1() -> "+"/"-"   part2(chosen)
```

**(b) IN WORDS.** The script the `-` branch decision was originally taken on. It
computes the flat-arm assembly height for a fixture, feeds it back through the
closed form to see which root returns `alpha = 0` at the datum, and then asks
whether that root stays defined and continuous across a provisional envelope.
Every geometric number in it is labelled a FIXTURE, not a decision.

**(c) IN SYMBOLS**

```
[1a] flat-arm datum, R = I, T_xy = 0, alpha_i = 0 so h_i = b_i + a u_i:

     rhs_i    =  d^2 - | q_i^flat - h_i^flat |^2         (horizontal only)
     z_home   =  h_p + sqrt( mean_i rhs_i )              UPPER root
     criterion: rhs_i is leg-independent (D3)  ->  spread max-min ~ 0

[1b] feed z_home back:  alpha_i^- = phi_i - arccos(P_i/C_i)  must be 0
     criterion: max_i | alpha_i^- |  ~  0 , and the SAME branch on all six
     ( asserted in code:  max|alpha_minus| < 1e-12 )

[1c] disc = d^2 - |q^flat - h^flat|^2  over a 4x4x3x3x3 = 432 grid at
     delta = 40 held;  counts combinations with disc < 0 (no valid z_home)

[2a] |P| > C anywhere in the envelope?          [2b] min (C-|P|)/C per leg
[2c] per-leg angular span, chosen vs other root
[2d] closed precession loop, 720 steps:  step-outlier ratio  max|dalpha| /
     median|dalpha| > 5  flags a branch flip;  endpoint closure
     max_i |alpha_i(360) - alpha_i(0)|

ENVELOPE HERE IS PROVISIONAL and superseded: tilt <= 6 deg (24 az x 4 mag),
yaw +/-10, T_horiz +/-0.05 r_b, T_vert +/-0.05 r_b -> 4365 poses.
```

**(d) VERIFIED BY.** Self-reporting, re-run:

| check | result |
|---|---|
| `[1a]` per-leg `rhs` spread | `2.220e-16`; `z_home = 1.223343109435593 r_b` |
| `[1b]` `max\|alpha_minus\|` at the datum | **`8.882e-16`**, `-` branch on all six |
| `[1c]` grid spread over 432 | `8.882e-16`; `8` combinations with `disc < 0`, all at `d/r_b = 0.8` |
| `[2a]` chosen root undefined | 4 `(pose, leg)` of `4365 x 6` |
| `[2b]` global `min (C-\|P\|)/C` | **`-5.713988e-03`** |
| `[2d]` step-outlier ratios | 1.40 on every leg (no flip); endpoint closure `0.000e+00` |

The `-5.7e-3` here is the figure `branch_envelope` was written to recompute; it
belongs to the superseded envelope.

---

## 6.4 `zhome_datum.py` — the flat-arm height, and `z_home(delta)`

**(a) SIGNATURE**

```
python -m stewart.diagnostics.zhome_datum
check_flat_arm_leg_independence() -> (r1, r2)
sweep_zhome_delta() -> rows            confirm_zhome_against_arm_tips(n=8) -> worst
_flat_arm_dist(geom)   _closed_form_sq(r_b, r_p, beta, beta_p, delta, a)
_delta_G_deg(r_b, r_p, beta, beta_p)   _zhome_sq(d, g2, a, gu)
```

**(b) IN WORDS.** Two checks on the flat-arm assembly datum: that the distance
the rod must span with the arms flat is the same for all six legs and matches a
closed form written in the input parameters, and how the resulting height varies
with `delta` around the full circle. It also recovers, verbatim, the grid behind
the withdrawn "8 of 432" claim.

**(c) IN SYMBOLS**

```
Independence discipline: b_i, u_i, n_i, p_i come from make_geometry; the only
things formed here are the closed forms, written in (r_b, r_p, beta, beta_p,
delta, a, d) and never in the anchors.

CHECK A, with  g_i = q_i^{xy} - b_i^{xy}  at R = I, T_xy = 0:

    residual 1 :  max_i |g_i - a u_i|  -  min_i |g_i - a u_i|      -> 0
    residual 2 :  max_i | |g_i - a u_i|^2  -  CF |                 -> 0
        CF  =  |g|^2 + a^2 - 2 a (g.u)
        |g|^2  =  r_p^2 + r_b^2 - 2 r_p r_b cos A ,   A = beta_p - beta
        g.u    =  r_b cos delta  -  r_p cos(A - delta)
    criterion:  max(rel_1, rel_2) < 1e-12  ->  AGREE

CHECK B, over a recovered 4x4x3x3x3 = 432 grid, delta swept [0, 360):

    (z_home - h_p)^2  =  d^2 - |g|^2 - a^2 + 2 a (g.u)
    feasible(delta)   <=>  that expression > 0
    delta_G           =  arg( r_p e^{iA} - r_b )   mod 360
                         ( where (z_flat - h_p)^2 is MINIMISED )
    counts feasibility per half-circle;  the question is whether [0, 180) is a
    sufficient delta range once a flat-arm datum is imposed

CONFIRMATION, independent of the algebra:
    residual  =  max_i | |q_i - arm_tips(0)_i| - d |   at the derived height
```

**(d) VERIFIED BY.** Self-reporting, re-run:

| residual | max | relative |
|---|---|---|
| A1 leg-independence | `9.948e-14` | `1.501e-15` |
| A2 closed form vs library geometry | `1.637e-11` | `2.103e-15` |
| `max \|u_i . z\|` | `0.000e+00` | — |
| confirmation `max\| \|q - tip\| - d \|`, 8 samples | **`2.220e-16`** | OK (`< 1e-12`) |

Check B, re-run: 432 grid points; `8` infeasible at `delta = 40` and **all 8
feasible on the other half-circle** (the withdrawal); by half-circle
`429` both / `0` low-only / `3` high-only / `0` neither; the 3 sit at a
simultaneous four-axis grid corner (`beta = 10`, `beta_p = 55`, `r_p/r_b = 1.2`,
`d/r_b = 0.8`), flagged GRID EDGE by the module itself. `z_home` swing over the
feasible `delta` set: min `0.0112`, median `0.1220`, max `1.2705`.

It also reports what the library refuses: **360 of 900** check-A grid points
rejected by `platform_ring`'s `0 < beta_p < 60` guard.

---

## 6.5 `zhome_bracket.py` — lower and upper brackets on `z_home`

**(a) SIGNATURE**

```
python -m stewart.diagnostics.zhome_bracket
fine_poses() -> (az, mg)                                        6 x 61 = 306 poses
n_min_closed_form(r_p, h_p, z_home, tilt_deg=TILT_LIMIT_DEG) -> float
z_lower_closed_form(r_p, h_p, tilt_deg=TILT_LIMIT_DEG) -> float
n_min_numeric(r_p, beta_p, h_p, z_home, az, mg) -> float
leg_terms(beta, beta_p, r_p, a, d, h_p, z_grid, az, mg) -> (A, B, G, N)
reach_feasible_any_delta(A, B, G, deltas_rad) -> (nz,) bool
reach_ceiling_bisect(..., lo, hi, ..., tol=1e-9) -> float
check_n_and_v(beta, beta_p, r_p, a, d, h_p) -> float
grids: BETA(5) x BETA_P(4) x RP_RB(3) x A_RB(3) x D_RB(3) = 540, H_P = 0.10,
       Z_GRID 0.025..3.0 step 0.025, DELTA_GRID 0..179 step 1
```

**(b) IN WORDS.** Brackets the restored `z_home` sweep axis from both ends: below
by the condition the fixed branch rests on, above by reach. The lower end comes
out in closed form and is free of `delta`, `a` and `d`; the upper end has no
closed form and is scanned per candidate. It then attributes every empty
bracket, because the "never binding" claim it started from rested on a sample
that could not have contained a counterexample.

**(c) IN SYMBOLS**

```
LOWER, from N_i > 0.  With p_i = (r_p cos phi_i, r_p sin phi_i, -h_p) and a
tilt of magnitude th about the horizontal axis at azimuth psi,

    (R p_i)_z  =  r_p sin(th) sin(phi_i - psi)  -  h_p cos(th)
    N_i        =  z_home + (R p_i)_z                  (T = (0,0,z_home))

    min N  =  z_home  -  max_th [ r_p sin th + h_p cos th ]

    z_home  >  r_p sin(tilt) + h_p cos(tilt)          <- the bound
              ( = hypot(r_p, h_p) if tilt + atan2(h_p, r_p) > 90 deg )

    delta-FREE, a-FREE, d-FREE, because v_i = z exactly and b_i . z = 0.
    Back-of-envelope  z_home - h_p > r_p sin(tilt)  drops the cos and
    OVERSTATES by  h_p (1 - cos tilt).

    criterion: at z_home = the bound, min N over the sampled envelope = 0;
    residual reported is | min N |, which can only be >= 0 (a discrete grid
    misses the worst azimuth).

UPPER, from reach.  Squaring |P_i| <= C_i = sqrt(|L_i|^2 - w_i^2) gives

    w_i(delta)^2  <=  G_i  =  |L_i|^2 - P_i^2 ,
    w_i(delta)    =  A_i cos delta + B_i sin delta ,
    A_i = L_i . n_i(0) ,   B_i = L_i . n_i(90)      both delta-free

    feasible(z) <=> exists delta in [0,180): max over (pose, leg) of
                    ( w^2 - G ) <= 0
    bracket(candidate) = { z in Z_GRID : reach(z) and z > z_lower_closed_form }

ATTRIBUTION of empty brackets, categories fixed by the constraints' shapes
(N_i > 0 is a one-sided FLOOR; reach is a two-sided INTERVAL, so "reach floor
above the N ceiling" cannot occur):

    R : reach empty on its own, at every z and every delta
    X : reach ceiling BELOW the N floor -> ceiling bisected to 1e-9 before
        the crossing is called real, since the grid step is 0.025
```

**(d) VERIFIED BY.** Self-reporting, re-run:

| check | result |
|---|---|
| `\| min N \|` at the closed-form bound, over the 306-pose envelope | worst **`5.551e-17`** |
| `max \|v_i - z\|` over candidates | `1.110e-16` |
| back-of-envelope overstatement at `h_p = 0.1 r_b` | `1.684e-03` |
| grid-vs-continuum, 29-pose harness grid at `beta_p = 25`, `55` | `+5.911e-04` (reports the constraint satisfied *before* it is) |
| non-empty brackets | **363 / 540**; empty 177; non-contiguous feasible sets **0** |
| lower ends / upper ends / widest | `[0.225, 1.650]` / `[0.425, 1.875]` / `0.825` |
| empty by `a/r_b` | 144/180 at `0.10`, 33/180 at `0.20`, 0/180 at `0.35` |
| attribution of the 177 | **R = 176, X = 1** |
| the crossing | `beta=10, beta_p=55, r_p/r_b=1.10, a/r_b=0.10, d/r_b=0.80`: ceiling `0.2920624` (bisected), floor `0.29932`, gap **`+7.26e-03`**, REAL |
| non-contiguous *reach* sets | 6, low component below the `N` floor in **6 of 6** |
| closed-form vs grid `N` disagreements | `0` z points |

---

## 6.6 `branch_envelope.py` — the `-` branch re-run, and the `-5.7e-3` recomputed

**(a) SIGNATURE**

```
python -m stewart.diagnostics.branch_envelope
geom_A(delta=None) -> Geometry
z_flat_closed_form(r_b, beta, delta, r_p, beta_p, h_p, a, d) -> float
leg_quantities(geom, R, z_home) -> (M, N, P, C, margin)
roots(M, N, P, C) -> (minus, plus, bad)            wrap(a) -> (-pi, pi]
Z_SCAN = 1.15 .. 1.325 step 0.0125 (15),  PRECESSION_STEPS = 1440
```

**(b) IN WORDS.** Re-runs the branch evidence on the settled envelope and across
the restored `z_home` axis, because the original evidence was gathered at one
height under a provisional 6° envelope. It then recomputes the `-5.7e-3`
boundary margin on the same geometry so that the two numbers can be compared —
and says explicitly that the two envelopes are not nested.

**(c) IN SYMBOLS**

```
(d) BRANCH:
    minus_i  =  atan2(N_i, M_i) - arccos(P_i/C_i)    NaN where |P_i| > C_i
    plus_i   =  atan2(N_i, M_i) + arccos(P_i/C_i)

    ASSERTION       min over (z, pose, leg) of N_i  >  0
    closed bound    r_p sin(tilt) + h_p cos(tilt)      (zhome_bracket's)
    identity        min N  ==  z_home - bound          digit for digit

    CONTINUITY, 1440 steps over the FULL circle at fixed magnitude:
        flip criterion  max_k |d alpha| / median_k |d alpha|  >  5
        closure         max_i | alpha_i(360) - alpha_i(0) |
    CROSS-CHECK        max | ik(...) - alpha_minus |

(e) MARGIN:  min over (pose, leg) of (C_i - |P_i|)/C_i, on fixture A, at the
    datum z_home, on the OLD envelope, then on the NEW one, then over the
    restored z_home axis;  plus per-axis costs, each a difference of two
    measurements on ONE pose set.
```

**(d) VERIFIED BY.** Self-reporting, re-run:

| check | result |
|---|---|
| `min(N_i) > 0`, every scanned `z_home` | PASSES, worst `+0.896361 r_b`; `N` floor `0.253639 r_b` |
| envelope fully reachable | `8 of 15` `z_home`, `[1.2000, 1.2875] r_b` |
| branch reaching `alpha ~ 0` at home | `-` at every feasible `z_home` |
| branch-flip step outliers, 1440-step loop | **0**; ratios 1.12–1.26; closure `<= 1.78e-15` |
| `max \|ik() - alpha_minus\|`, 40 samples | **`8.882e-16`** |
| old envelope at the datum `z_home = 1.223343` | **`-5.713988e-03`** (reproduced exactly) |
| same geometry, same height, settled envelope | **`+1.550398e-01`** |
| best over the `z_home` axis, `delta` untuned at 40° | **`+2.276643e-01`** at `z_home/r_b = 1.2375` |
| individual costs | yaw ±10° `1.8033e-01`; `T` ±0.05 `r_b` `3.6486e-01`; tilt 6→10.529 `3.6147e-01` |
| the withdrawn decomposition's discrepancy | `1.62e-03`, cause confirmed: different azimuth samples (15° `+1.534192e-01`, 10° `+1.550398e-01`, 0.25° `+1.531859e-01`) |

---

## 6.7 `sweep_budget.py` — three numbers that had been asserted in prose

**(a) SIGNATURE**

```
python -m stewart.diagnostics.sweep_budget      (part1(), part2(), part3())
budget(n_poses_, label) -> dict        _min_margin(geom, R, T) -> float
arrest_tilt(v, L, g) -> deg
SWEEP_AXES(6), POINTS_PER_AXIS = 5, DELTA_STEPS = 180, PROBES(6),
Z_BEST_A = 1.2375, ARREST_V = 0.200, ARREST_L = 0.100, G_APPENDIX = 9.81
```

**(b) IN WORDS.** Gives code to three documented results that were arrived at by
hand: the sweep's compute ledger (whose GB figure carries a harness
requirement), the translation-sensitivity calibration behind the score's probe
range, and the derivation appendix's arrest-framing tilt.

**(c) IN SYMBOLS**

```
1. LEDGER
     candidates    =  5^6  =  15625
     w per obj     =  n_poses * 6  =  29 * 6  =  174
     full evals    =  candidates * w_per_obj
     cheap evals   =  candidates * 180 * w_per_obj
     memory        =  cheap * 8 bytes                 -> chunking requirement

2. SENSITIVITY   (ranking discriminator, NOT a feasibility test)
     sens(p)  =  [ margin(dxy = 0)  -  min over 24 displacement azimuths of
                   margin(dxy = p) ]  /  p                  worst over azimuth
     linearity criterion:  sens(p) / sens(p_min)  ~  1
     The full circle is swept in BOTH tilt azimuth (24) and displacement
     direction (24), because a horizontal offset breaks the D3 symmetry.
     Decomposition at the (e) conditions separates T_horiz from T_vert.

3. ARREST FRAMING
     (5/7) g sin(tilt)  =  v^2 / (2 L)          v = 0.2 m/s, L = 0.1 m
     scaling in plate size k:  arrest tilt ~ 1/k ,  recovery tilt ~ k
```

**(d) VERIFIED BY.** Self-reporting, re-run:

| result | measured |
|---|---|
| ledger, settled | 15625 candidates, `2,718,750` full, `4.89e+08` cheap, **`3.92 GB`** (3.6 GiB) |
| superseded ledgers reproduced | 81-pose: `7,593,750` / `1.37e+09` / `10.94 GB`; 729-pose: `68,343,750` / `1.23e+10` / `98.42 GB` |
| sensitivity at fixture A, `z_home/r_b = 1.2375` | `1.3608` at `p = 0.0025`, rising to `1.5830` at `p = 0.05`; **within ~1% of linear over `0.0025–0.01`**, 16% off at `0.05` |
| the documented "order 7" | **NOT confirmed** — measured `~1.36`, a factor `~5.4` smaller |
| decomposition of the `3.648645e-01` row | `T_horiz` only `7.990445e-02` (÷0.05 = **1.60**); `T_vert` only `2.809069e-01` (÷0.05 = 5.62) |
| arrest framing | `1.6356 deg` at `g = 9.81`, `1.6361 deg` at `g = 9.80665`; recovery at `tau = 1.0 s` gives the same `1.6361` |

---

## 6.8 `roundtrip.py` (diagnostics) — the round-trip **gate**

The only module that exits non-zero on failure.

**(a) SIGNATURE**

```
python -m stewart.diagnostics.roundtrip          # exit 0 on a pass, 1 on a fail
geodesic_deg(R_cmd, R_fk) -> deg                 geodesic_deg_arccos(...) -> deg
check_metric_agreement(verbose=True) -> rows     _check_log_inverts_exp(n=20000)
envelope_grid(n_mag, n_az) -> (az, mg)
bracket(kw, z_grid=Z_SCAN_MM, n_mag=6, n_az=61) -> z_ok
z_samples(z_ok, n) -> ndarray
fd_jacobian(geom, tips, R, T, h_mm, h_rad) -> (6,6)
check_jacobian(verbose=True) -> (best, rows)     _jacobian_control(h_mm)
sweep(kw, z_list, n_mag, n_az, *, tol, char_len, max_iter=100) -> rows
summarise(rows) / worst_of(ok) / report_brackets / report_cond_vs_length /
report_tolerance / report_gate / report_movement / plausibility /
report_modes / probe_modes / report_iterations / _choose_tol / _verdict
FIXTURES A..F (r_b = 100 mm), LEVELS (5,7,3) (9,13,5) (17,25,9),
Z_SCAN_MM 10..300 step 0.25, SAME_MODE_MM = SAME_MODE_DEG = 1e-4
```

**(b) IN WORDS.** The hard stop on the plan: it commands a pose, runs the
inverse kinematics, then solves the forward kinematics **from a fixed neutral
home seed** and asks whether the pose comes back. It also verifies the pieces
the answer rests on — the rotation metric, the analytic Jacobian, and whether
the convergence tolerance is what limits the accuracy it reports.

**(c) IN SYMBOLS**

```
GATE:   (R_cmd, T_cmd)  ->  alpha = ik(...)  ->  (R_fk, T_fk) = fk_solve(
            alphas, seed R0 = I, T0 = (0, 0, z_home) )

    pos error  =  | T_fk - T_cmd |                        mm
    ang error  =  deg | log_so3( R_cmd^T R_fk ) |         deg, convention-free
    classifier :  pos > 1e-4 mm  OR  ang > 1e-4 deg   ->  "other-mode"
                  ( not a tuned cut: the |dT| distribution stops at 1e-12 mm,
                    so any cut in the empty decades classifies identically )

BRACKET, per fixture at its OWN delta (unlike zhome_bracket, which asks whether
SOME delta works):

    z feasible  <=>  for all poses, all legs:  |P_i| <= C_i  AND  N_i > 0
    z sampled in the INTERIOR, at fractions k/(n+1) of the span

JACOBIAN check:  central differences through the SAME left perturbation

    J_fd[:, k]     =  ( f(T + h e_k) - f(T - h e_k) ) / 2h
    J_fd[:, 3+k]   =  ( f(exp([h e_k]_x) R) - f(exp([-h e_k]_x) R) ) / 2h
    deviation      =  max over 36 entries of |J_a - J_fd| / max(|J_a|,|J_fd|)
    verdict        =  BEST step's worst  <=  1e-7          (bottom of the U)

TOLERANCE check:   sweep tol over 1e-8 .. 1e-15 and ask whether the worst pose
    error MOVES.  If it does, the gate was measuring where Newton stopped.

CONVERSION, checked per pose, not asserted:
    ||dx||  <=  sqrt(6) ( max_i |f_i| + eta ) / sigma_min ,
    dx = (dT, char_len * omega),   eta = 8 eps ( d + |T| )

PASS  <=>  worst_jacobian_dev <= 1e-7  AND  non-convergences = 0
           AND  worst |dT| < 1e-6 mm
```

Scale: the fixtures are the `r_b = 1` geometries at `r_b = 100 mm`, so a
tolerance can be a length in mm; the kinematics is homogeneous of degree one and
the envelope is purely angular, so this is a change of units.

**(d) VERIFIED BY.** Itself, exit code, re-run 2026-09-08 — **GATE PASSES**:

| line | measured |
|---|---|
| brackets: A `[120.000, 129.000]`, C `[57.500, 134.750]`, E `[21.000, 101.500]`, F `[128.750, 138.750]` mm | B and D **EMPTY**, dropped, and reported as such |
| Jacobian worst relative deviation vs FD | **`9.552e-08`** at `h = 5e-4 mm` |
| poses returning the commanded mode | **14 436**; different mode `0`; non-converged `0`; unreachable `0` |
| worst `\|T_fk - T_cmd\|` | **`1.8532e-13 mm`** |
| worst `\|log_so3(dR)\|` | **`7.8203e-14 deg`** (the superseded `arccos` form would say `2.0913e-06`, its own floor) |
| worst residual | `2.842e-14 mm` at tol `1e-9` |
| worst `cond`, `1/sigma_min` at `char_len = r_b` | `9.518`, `3.980`, PROVISIONAL |
| conversion bound, worst measured/bound | **`0.074`** (`1.535`, i.e. violated, with `eta` omitted) |
| worst case under refinement | rises `1.47x / 1.14x / 1.29x / 1.46x` — at the arithmetic floor, so sampling, not a real worst case |
| assembly modes | 32 distinct roots over 4 fixtures from 400 random seeds each; **1** non-commanded root with all anchors above the plate, **0** inside the tilt envelope |

---

## 6.9 `score_discriminators.py` — candidate discriminators, measured, none chosen

**(a) SIGNATURE**

```
python -m stewart.diagnostics.score_discriminators
feasible_candidates(verbose=True) -> list[dict]
evaluate(rec, R, T_of, az) -> rec          measure(rec, R, T, char_lens, delta=None)
char_lengths(rec) -> {"r_b","r_p","d","a"}
probe_margin(g0, g90, R, az, z_home, delta_deg, probe) -> float
sens_at(g0, g90, R, az, z_home, delta_deg, base_margin, probe=0.01) -> float
scan_delta(beta, beta_p, r_p, a, d, R, T, deltas_deg, char_len, with_cond=True)
tune_constrained(margin, cond, deltas_deg, cap) -> (delta, margin)
run_constrained(rows, R, az, deltas_deg=DELTA_GRID)
verify_jcmd(rows, R, n_cases=6) -> (ok, records)
score_terms(rows, R, az, cap=1e6, probes=(0.005,0.0075,0.010))
plus _report_* for parts (a)-(d), (1)-(9);  main() -> None
knobs: PROBE_DXY = 0.01, N_DISP_DIR = 24, FINE_AZ_STEP_DEG = 0.25,
       N_SAMPLE = 40, RANK_SHIFT_FRAC = 0.05, CONSTRAINT_CAPS = (1e2..1e6),
       CONSTRAINT_CHAR_LEN = "r_b", SCORE_CAP = 1e6, TIE_TOL = 1e-12,
       GRID_RESOLUTION = 2.88e-3, ANCHOR_KICK = 0.03 on leg index 0
```

**(b) IN WORDS.** Measures every candidate discriminator side by side on the
feasible set and reports how they relate, without forming a score. It carries
both Jacobians and all four candidate characteristic lengths the whole way and
merges neither, because `notation.md` §12 records those as open. It also
verifies the one piece of new algebra it introduces before anything downstream
uses it.

**(c) IN SYMBOLS**

```
FEASIBLE SET (recomputed, not transcribed, from zhome_bracket's own tests):
    540 candidates ->  reach(any delta) AND z > z_lower_closed_form

PER CANDIDATE, at z_home = the bracket MIDPOINT (a point of evaluation, not a
recommendation), on the 29-pose grid:

    margin   =  min over legs and poses of ( C_i - |P_i| ) / C_i
    delta    =  argmax over the 180-point scan of margin        (MAXIMIN)
    sens(p)  =  [ margin(0) - min over 24 azimuths margin(dxy = p) ] / p
    tau_min  =  min over legs and poses of | rod_i . tangent_i | / d
                rod_i     =  q_i - h_i             |rod_i| = d
                tangent_i = -u_i sin alpha_i + v_i cos alpha_i     unit
                so tau_i = |cos(angle between them)| in [0, 1]; 0 = loss of
                authority

    J_fk     =  df_i / d(T, omega)             the gate's Jacobian, LEFT
    J_cmd    =  d(T, omega) / d(alpha)
             =  J_fk^-1 diag( a e_i . tangent_i )        by implicit function,
                since  df_i/dalpha_i = -a (e_i . tangent_i)  and df/dalpha is
                DIAGONAL (only leg i's own angle enters f_i)

    cond, sigma_min of BOTH, at char_len in { r_b, r_p, d, a }, all four,
    every one flagged PROVISIONAL.  One scaling convention, applied to both:
    S = diag(1,1,1,l,l,l),  J_fk S^-1  and  S J_fk^-1 D.

DELTA DEPENDENCE, closed form, no geometry rebuild:
    n_i(delta) = cos(delta) n_i(0) + sin(delta) n_i(90)
    u_i(delta) = cos(delta) u_i(0) + sin(delta) u_i(90)
    v_i(delta) = n_i(delta) x u_i(delta)                    formed, not assumed
    alpha      = atan2(N, M) - arccos(P/C)   with |P| <= C tested first, and
                 NaN (not a clipped value) at any delta where a leg is
                 unreachable

CONSTRAINED INNER TUNE:
    delta_con(C)  =  argmax { margin(delta) : cond(J_fk)@r_b (delta) <= C },
                     (None, NaN) when no delta on the scan is admissible
    caps C in { 1e2, 1e3, 1e4, 1e6 }, reported side by side, none preferred

SCORE STUDY (part 7), evaluating a form specified to it, not proposing one:
    score  =  margin - sens(p) * p    ==  margin(dxy = p)      IDENTICALLY
    resolvability criterion: a spread below GRID_RESOLUTION = the 10-deg
    azimuth grid's own optimism cannot be resolved by the grid it is computed
    on

COLLAPSE TEST (part 8): group by (r_p, a, d, |e|), e = beta_p - beta;
    compare the tuned maximum, the whole margin FIELD at a COMMON delta, and
    a negative control that displaces one anchor by 0.03 r_b, run once per leg.
```

**(d) VERIFIED BY.** Self-reporting, re-run 2026-09-08; the `J_cmd` algebra is
gated on its own finite-difference check before anything below it runs:

| check | measured |
|---|---|
| feasible set | **363 / 540**, contiguous 363/363, measured 363/363 |
| self-check, worst `\|rod_i\| - d` over every pose | `4.441e-16 r_b` |
| `J_cmd` vs central differences of the real map | worst `2.473e-07` at `h = 1e-4`, `2.517e-09` at `h = 1e-5`; ratio 10–98 (the `O(h^2)` signature); acceptance `1e-6` — **CONFIRMED** |
| identity `\| \|e_i . tangent_i\| - tau_i \|` | `1.110e-16` .. `3.331e-16` |
| vectorised `delta` scan vs `_tune_delta` | argmax disagreements `0 / 363` |
| Spearman `margin` vs `tau_min` | **`0.8348`** (measured-and-redundant) |
| Spearman `margin` vs `sens` | `-0.1166` |
| cap `C = 1e6` on `cond(J_fk)@r_b` | binds on **exactly the 34 of 363** with `beta_p == beta`, all of which also have tuned `delta = 0`; worst margin cost **`4.8652e-05`** |
| the degeneracy | `cond(J_fk)@r_b` spans **17.9 decades** over `delta` on the exemplar (`1.068e+19` at `delta = 0`), while `margin` varies 2.25% and is **largest at `delta = 0`** — the maximin tune *selects* it |
| coarse-azimuth optimism `(e)-(f)` | max **`2.8801e-03`** = `GRID_RESOLUTION` |
| identity `\|(margin - sens*p) - margin(p)\|` | **`6.939e-18`** |
| `\|e\|` collapse | within-group tuned-margin spread `1.332e-15`; at a COMMON `delta` the fields differ pointwise by `7.805e-01` against a field range of `7.9298e-01`; groups equal at a common `delta`: **0 of 124** |
| tune/score mismatch (part 9) | worst margin given up `1.3651e-03` (< resolution); max rank movement **2 places**; top-20 in/out **0** |
| compute cost (part 6) | `cond` costs **18.1x** the margin scan; projected sweep 9.6 s (margin only) to 174.5 s (`cond` at every `delta`) |

---

# 7. `test_kinematics.py`

Plain numpy, no pytest. `python test_kinematics.py` → exit 0 on all pass, 1
otherwise. `TOL = 1e-12`. All geometry is a fixture:
`r_b=1, beta=20, delta=40, r_p=0.85, beta_p=40, h_p=0.10, a=0.20, d=1.20`.
Re-run 2026-09-08: **exit 0, ALL PASS**.

## Row 1 — `check_stage1_identity`

**(a) SIGNATURE** `check_stage1_identity() -> None`, appends one row.

**(b) IN WORDS.** Pins that with the platform level and at the origin, the world
anchors are exactly the design constants — that `stage1` adds and rotates rather
than doing something more elaborate.

**(c) IN SYMBOLS**

```
residual  =  max | stage1(g, I, 0)  -  p |          criterion: <= 1e-12
```

**(d) VERIFIED BY / RESIDUAL.** **`0.000e+00`**, PASS. Shape reported `(3, 6)`.

## Row 2 — `check_rotation_not_transposed`

**(a) SIGNATURE** `check_rotation_not_transposed() -> None`.

**(b) IN WORDS.** Pins that the rotation acts from the left on columns. It builds
a purpose-made `Geometry` whose first anchor is literally `(10, 0, 0)`, because
a transposed `R` sends it to `(0, -10, 0)` and the residual then separates the
two unambiguously — this is the error that produces plausible numbers and a
platform tilting backwards.

**(c) IN SYMBOLS**

```
p_0 = (10, 0, 0),  other columns filler.
residual  =  max | stage1(g, Rz(90 deg), 0)[:, 0]  -  (0, 10, 0) |
transposed R would give (0, -10, 0), i.e. a residual of 20.
```

**(d) VERIFIED BY / RESIDUAL.** **`6.123e-16`**, PASS, `got [0.0, 10.0, 0.0]`.

## Row 3 — `check_tip_distance`

**(a) SIGNATURE** `check_tip_distance() -> None`.

**(b) IN WORDS.** Pins that at zero servo angle every arm tip is exactly one arm
length from its own shaft — the length constraint, on all six legs at once.

**(c) IN SYMBOLS**

```
residual  =  max_i | | arm_tips(g, 0)_i - b_i |  -  a |
also reported: spread = max_i(.) - min_i(.)
```

**(d) VERIFIED BY / RESIDUAL.** **`5.551e-17`**, PASS, spread `1.110e-16`.

## Row 4 — `check_tip_direction` (and its control row)

**(a) SIGNATURE** `check_tip_direction() -> None`, appends two rows.

**(b) IN WORDS.** Pins the *direction* of the zero, not just the distance: at
`alpha = 0` the tip lies along `u`, not along `n`. The control row is the point
of the test — it shows that a tip placed along `n` passes row 3 identically, so
row 3 alone cannot catch a `u`/`n` swap.

**(c) IN SYMBOLS**

```
residual        =  max | arm_tips(g, 0)  -  ( b + a u ) |
control residual=  max_i | | (b + a n)_i - b_i | - a |      <- passes row 3 too,
                   because n is also a unit vector
```

**(d) VERIFIED BY / RESIDUAL.** Row 4 **`0.000e+00`** exactly, PASS. Control row
**`5.551e-17`** — identical to row 3's residual, which is the whole content of
the control.

**Coverage note.** The four rows are derivation §7's table. They exercise
`stage1`, `legs` (through `stage1` only implicitly — `legs` itself is never
called), `arm_tips`, `Geometry`, `base_ring`, `platform_ring` and
`make_geometry`. They do **not** exercise `ik`, `fk`, `w`, `Unreachable`,
`plotting` or `roundtrip`.

---

# 8. Call graph

## 8.1 Within `stewart/` (core)

```
geometry.Geometry.__post_init__  ->  _as_36, _check_unit, _check_orthogonal
_check_unit / _check_orthogonal  ->  _bad_legs
geometry.make_geometry           ->  base_ring, platform_ring, Geometry
geometry.smoke_geometry          ->  base_ring, Geometry

kinematics.stage1                ->  (leaf)
kinematics.legs                  ->  stage1
kinematics.w                     ->  legs
kinematics.arm_tips              ->  _v
kinematics.ik                    ->  legs, _v, Unreachable
kinematics.exp_so3               ->  _skew
kinematics.geodesic_angle        ->  log_so3
kinematics._apply                ->  exp_so3
kinematics.fk_solve              ->  arm_tips, fk_residual, fk_jacobian,
                                     _apply, _cond_and_sigma, FKNotConverged
kinematics.fk                    ->  fk_solve

plotting.draw_pose               ->  _arm_tip_points, _cube, _apply_cube, _triad
plotting.compare_branches        ->  draw_pose, _arm_tip_points, _cube, _apply_cube
plotting.animate                 ->  draw_pose (via _update), _arm_tip_points, _cube
plotting.arm_circles             ->  _v
plotting._arm_tip_points         ->  _v        (a SECOND copy of arm_tips' formula,
                                                independent of kinematics.arm_tips)

roundtrip.known_poses            ->  _rot_x, _rot_y, _rot_z
roundtrip.random_poses           ->  _axis_angle
roundtrip.round_trip             ->  _unpack, _axis_angle, ik, fk, _geodesic_deg,
                                     _format
roundtrip._geodesic_deg          ->  kinematics.geodesic_angle
```

## 8.2 Which caller reaches each library symbol

| symbol | reached from |
|---|---|
| `Geometry` | `geometry.make_geometry`, `geometry.smoke_geometry`, `azimuth_symmetry.broken_geometry`, `score_discriminators._kick_pair`, `test_kinematics.check_rotation_not_transposed` |
| `base_ring` | `make_geometry`, `smoke_geometry`, `branch_check.geom`, `test_kinematics` |
| `platform_ring` | `make_geometry`, `branch_check.geom` |
| `make_geometry` | `azimuth_symmetry`, `branch_envelope.geom_A`, gate (`bracket`, `sweep`, `check_jacobian`, `_jacobian_control`, `report_cond_vs_length`, `plausibility`, `probe_modes`), `score_discriminators` (`_delta_basis`, `measure`, `verify_jcmd`, `_pose_smin`), `zhome_bracket` (`leg_terms`, `n_min_numeric`, `check_n_and_v`), `zhome_datum._geom`, `test_kinematics.fixture_geom` |
| `smoke_geometry`, `Geometry.summary` | `demo.py` only |
| `stage1` | `kinematics.legs`, `test_kinematics` (rows 1–2), `zhome_datum.confirm_zhome_against_arm_tips` |
| `legs` | `kinematics.ik`, `kinematics.w` — **no external caller** |
| `w` | **NO CALLER ANYWHERE** |
| `arm_tips` | `fk_solve`, gate (`check_jacobian`, `_jacobian_control`, `report_cond_vs_length`), `score_discriminators` (`_jacobians_at_pose`, `verify_jcmd`), `zhome_datum`, `test_kinematics` (rows 3–4) |
| `ik` | `stewart.roundtrip.round_trip`, gate (`sweep`, `check_jacobian`, `_jacobian_control`, `report_cond_vs_length`, `plausibility`, `probe_modes`), `score_discriminators` (`_jacobians_at_pose`, `verify_jcmd`), `branch_envelope.main` |
| `fk` | `stewart.roundtrip.round_trip` only (which `demo.py` runs) |
| `fk_solve` | `kinematics.fk`, gate (`sweep`, `probe_modes`), `score_discriminators._fd_jcmd` |
| `fk_residual` | `fk_solve`, gate (`check_jacobian`, `fd_jacobian`, `_jacobian_control`, `report_cond_vs_length`), `score_discriminators` |
| `fk_jacobian` | `fk_solve`, gate (`check_jacobian`, `report_cond_vs_length`), `score_discriminators._jacobians_at_pose` |
| `exp_so3` | `kinematics._apply`, gate (`fd_jacobian`, `check_jacobian`, `_jacobian_control`, `check_metric_agreement`, `_check_log_inverts_exp`, `probe_modes`) |
| `log_so3` | `geodesic_angle`, gate `_check_log_inverts_exp`, `score_discriminators._fd_jcmd` |
| `geodesic_angle` | `stewart.roundtrip._geodesic_deg`, gate `geodesic_deg` |
| `_cond_and_sigma` | `fk_solve`, `score_discriminators` (`_cond_pair`, `_pose_smin`) — a **private** symbol crossing a package boundary |
| `Unreachable` | raised by `ik`; caught by `stewart.roundtrip.round_trip`, gate (`sweep`, `check_jacobian`, `report_cond_vs_length`, `plausibility`, `probe_modes`), `score_discriminators` (`measure`, `verify_jcmd`) |
| `FKNotConverged` | raised by `fk_solve`; caught by gate (`sweep`, `probe_modes`) |
| `draw_pose`, `arm_circles` | `demo.py` (and `draw_pose` from `compare_branches`/`animate`, which nothing calls) |
| `known_poses`, `round_trip` | `demo.py` |

## 8.3 Which diagnostic exercises which library function

| module | exercises |
|---|---|
| `envelope.py` | nothing in `stewart/` — self-contained (model + pose grid) |
| `azimuth_symmetry.py` | `make_geometry`, `Geometry`; forms `w`, `M`, `N`, `P`, `C` inline; **never calls `ik`** |
| `branch_check.py` | `base_ring`, `platform_ring`; its own closed form; **never calls `ik`** |
| `zhome_datum.py` | `make_geometry`, `stage1`, `arm_tips` |
| `zhome_bracket.py` | `make_geometry`; forms the reach test inline |
| `branch_envelope.py` | `make_geometry`, **`ik`** (as the cross-check against its own closed form) |
| `sweep_budget.py` | `branch_envelope.geom_A` → `make_geometry`; margin formed inline |
| `roundtrip.py` (gate) | `make_geometry`, `ik`, `arm_tips`, `fk_solve`, `fk_residual`, `fk_jacobian`, `exp_so3`, `log_so3`, `geodesic_angle`, `Unreachable`, `FKNotConverged`, `FK_TOL_MM` |
| `score_discriminators.py` | `make_geometry`, `Geometry`, `ik`, `arm_tips`, `fk_residual`, `fk_jacobian`, `fk_solve`, `log_so3`, `_cond_and_sigma`, `Unreachable`, plus `zhome_bracket`'s and `envelope.py`'s functions |

## 8.4 Public functions no test and no diagnostic calls — FLAGGED

| symbol | status |
|---|---|
| `kinematics.w` | **no caller anywhere in the repo.** The only public solver function that is entirely dead. |
| `plotting.compare_branches` | no caller |
| `plotting.animate` | no caller |
| `plotting.plot_angles` | no caller |
| `roundtrip.random_poses` | no caller |
| `plotting.draw_pose`, `plotting.arm_circles` | called only by `demo.py`, which asserts nothing about the result |
| `Geometry.summary` | called only by `demo.py` |
| `kinematics.legs` | no caller outside `stewart/kinematics.py` itself. `test_kinematics.py` **imports** it and never calls it |
| `stewart.roundtrip.round_trip`, `known_poses` | called only by `demo.py`; the gate uses its own `sweep` instead |
| `Geometry`'s validation branches | no caller constructs an invalid `Geometry` |

---

# 9. Cross-reference with the documents

Documents consulted: `docs/notation.md`, `docs/stewart-ik-derivation.md`,
`docs/phase-0-design-log.md`, `docs/cc-fk-gate.md`,
`docs/cc-summary-2026-09-05.md` — as they stand in the working tree (two of them
have uncommitted modifications; see §10).

## 9.1 (a) In `stewart/`, not mentioned in any of those documents

**Whole modules**

* **`stewart/plotting.py`** — the file, and all five public functions
  (`draw_pose`, `arm_circles`, `compare_branches`, `animate`, `plot_angles`),
  and its private second copy of the arm-tip formula (`_arm_tip_points`). No
  document names any of them. `phase-0-design-log.md` describes the debugger
  role ("visible in one frame of a 3D wireframe") without naming the module.

**Functions and classes**

* `base_ring` — the *formulas* are in derivation §8 and the design log, but the
  function name appears in no document.
* `Geometry` the class, its validators (`_as_36`, `_check_unit`,
  `_check_orthogonal`, `_bad_legs`), and `Geometry.summary`. (`cc-summary`'s
  only "Geometry" is "Geometry grid", a sweep count.)
* `stewart/roundtrip.py`'s API: `known_poses`, `random_poses`, `round_trip`,
  `_rot_x/_rot_y/_rot_z`, `_axis_angle`, `_unpack`, `_format`. The file is named
  in three documents, but only ever for `_geodesic_deg` and for the fact that it
  unpacks `R, T`.
* `kinematics._v`, `_skew`, `_apply`, `_cond_and_sigma`. `_cond_and_sigma` is
  the one that matters: it is imported by `score_discriminators` and it is where
  the undecided characteristic length physically enters the code.
* `FK_MAX_ITER` is named only in `cc-fk-gate.md`; `FK_TOL_MM` in two documents.
* Every function inside the gate except `check_metric_agreement`:
  `bracket`, `z_samples`, `envelope_grid`, `fd_jacobian`, `check_jacobian`,
  `_jacobian_control`, `sweep`, `summarise`, `worst_of`, `plausibility`,
  `probe_modes`, `report_*`, `_choose_tol`, `_verdict`,
  `geodesic_deg_arccos`, `_check_log_inverts_exp`, and the constants
  `SAME_MODE_MM`, `SAME_MODE_DEG`, `LEVELS`, `Z_SCAN_MM`,
  `CHAR_LEN_CANDIDATES`, `CHAR_LEN_PROVISIONAL_LABEL`.
* Every function in `score_discriminators.py`. The module is cited by part
  number in `notation.md` §8/§12 and in the design log, but no function,
  constant or knob in it is named — including `PROVISIONAL_NAMES`,
  `CONSTRAINT_CAPS`, `CONSTRAINT_CHAR_LEN`, `SCORE_CAP`, `SCORE_PROBES`,
  `TIE_TOL`, `GRID_RESOLUTION`, `ANCHOR_KICK`, `N_SAMPLE`, `RANK_SHIFT_FRAC`,
  `E_OFFSETS_DEG`, `verify_jcmd`, `scan_delta`, `tune_constrained`,
  `feasible_candidates`.
* Every function in `envelope.py` (`bang_bang_accel`, `tilt_for`, `tilt_R`,
  `envelope_poses`, `n_poses`) and every constant in it
  (`G`, `ROLL_FACTOR`, `TAU`, `X0_WORKING`, `X0_LATENCY`, `TAU_L`, `V_PEAK`,
  `TILT_BARE_DEG`, `TILT_LIMIT_DEG`, `AZIMUTH_WINDOW_DEG`, `N_MAGNITUDE`,
  `N_AZIMUTH`, `TAU_L_IS_PROVISIONAL`). The *numbers* are all in `notation.md`
  §9; the code identifiers are not.
* Similarly for `azimuth_symmetry` (`aggregates`, `broken_geometry`, `run_one`),
  `zhome_datum` (`check_flat_arm_leg_independence`, `sweep_zhome_delta`,
  `confirm_zhome_against_arm_tips`), `zhome_bracket` (`n_min_closed_form`,
  `z_lower_closed_form`, `leg_terms`, `reach_feasible_any_delta`,
  `reach_ceiling_bisect`, `fine_poses`, `check_n_and_v`), `branch_envelope`
  (`z_flat_closed_form`, `leg_quantities`, `roots`, `geom_A`, `Z_SCAN`,
  `PRECESSION_STEPS`), `branch_check` (`closed_form`, `solve_z_home`,
  `envelope_poses`, `part1`, `part2`), `sweep_budget` (`budget`, `arrest_tilt`,
  `_min_margin`, `PROBES`, `Z_BEST_A`, `SWEEP_AXES`).
* `stewart/__init__.py`'s `__all__` and `__version__ = "0.0.1"`.
* `stewart/diagnostics/__init__.py` (a one-line docstring, no code).

**Behaviour in code that no document records**

* `smoke_geometry`'s anchor jitter is applied to **all three** components, so
  its anchors are not coplanar — it violates the `p_z = -h_p` convention that
  `notation.md` §4 settles. Documented in code as "not a design"; the
  non-coplanarity itself is nowhere in `docs/`.
* `Unreachable` reports only the **lowest-numbered** failing leg.
* `ik` returns the angle **unwrapped** (raw `phi - arccos`).
* `platform_ring` does **not** range-check `h_p`; a negative `h_p` is accepted
  and puts the anchor plane above `{P}`'s origin.
* `make_geometry` does not enforce `d > a` — the docstring says so; no document
  does.
* `fk_solve`'s final **polar projection** `R <- U V^T` and its `so3_drift`
  output. `cc-fk-gate.md` §2.3 quotes the drift figure (`2.2e-16`) but does not
  record that the returned `R` is projected.
* `fk_solve`'s backtracking limit (30 halvings) and the LM escalation schedule
  (`lam = 1e-3`, ×10, up to 40 attempts).
* The gate's `plausibility` re-runs `ik` at every recovered pose and reports
  whether it returns the same six angles — the mechanism behind the
  "identical servo commands" finding; the document reports the finding, not the
  function.

## 9.2 (b) Numbers quoted in those documents that no committed module prints

Method, as in the 2026-09-05 provenance audit: every numeric literal in the five
documents (**970 distinct**) was matched against the numbers actually printed by
a re-run of all eight diagnostics, `test_kinematics.py` and `demo.py`. Matching
is at the literal's own significant-figure precision. Dates and heading numbers
excluded.

The automated pass flags **19** literals of three or more significant figures
with no producing module (15 distinct values; some appear in more than one
document). Two of those 19 are artifacts of the method and are **not** gaps:
`5.713988e-03` in `cc-summary` is written with a Unicode minus that the
tokeniser split, and `branch_envelope` does print it; `98973` is part of the
commit hash `1b98973`. Two real gaps were caught by **inspection rather than by
the matcher** — `1.94e-4` and `2.0e-13` carry only three and two significant
figures, so they matched some unrelated printed value by coincidence, while no
module computes either. What follows is the corrected list, in five groups.

**1. The near-mode case on `smoke_geometry` — the conditioning numbers are unbacked**

| literal | where | status |
|---|---|---|
| `cond(J) = 9045` | `notation.md` L528, `cc-fk-gate.md` L465, design log L913 | **UNBACKED** |
| `sigma_min = 1.94e-4` | same three places | **UNBACKED** |
| `2.0e-13 deg` (`ik` agreement at the recovered pose) | same | **UNBACKED** |

No committed module computes `cond` or `sigma_min` on `smoke_geometry`, or
re-runs `ik` at that recovered pose. The gate's fixtures are A/C/E/F at
`r_b = 100`; `smoke_geometry` is reached only by `demo.py`, which prints the
pose error and nothing else. The *pose* figures **are** backed: `demo.py` prints
`0.3163 mm` and `0.7296 deg`. This is the load-bearing gap in the group — those
three numbers are the entire basis of `notation.md` §12's "near-singular
geometry is where assembly modes coalesce".

**2. The superseded tolerance-as-stopping-rule table**

| literal | where | status |
|---|---|---|
| `9.744e-10`, `1.291e-10`, `1.661e-11`, `1.844e-12` | `cc-fk-gate.md` §4 table | **UNBACKED** |
| `48 of 117` poses halting early | `cc-fk-gate.md` L216, design log L869 | **UNBACKED** |

These describe a variant of `fk` that no longer exists: the committed solver
stops on stagnation, so no committed module can produce them. `kinematics.py`'s
`FK_TOL_MM` docstring and the gate's part (3) text both *quote* the sequence at
two significant figures ("9.7e-10, 1.3e-10, 1.7e-11, 1.8e-12, 1.3e-13") as
prose — a printed string, not a computed value. The fifth entry `1.268e-13` is
backed, coincidentally: it equals the worst `|dT|` the current gate reports.

**3. Superseded compute-ledger figures**

| literal | where | status |
|---|---|---|
| `273M` full evaluations (the datum path) | design log L624 | **UNBACKED** |
| `3125` candidates (five-axis sweep) | design log L644 | **UNBACKED** |

`sweep_budget.py` reproduces the 15 625/81/729-pose ledgers but not the datum
path or the five-axis count. Both appear only inside quoted superseded text.

**4. Inputs and hand-worked counter-examples, never claimed as results**

| literal | where | note |
|---|---|---|
| `w = 110`, `(100, 140)`, `\\|P\\| = 1900` | derivation §5.4, design log | The unreachability counter-example at `a = 20`, `d = 120`. Hand arithmetic, stated as an illustration. |
| `c = 270` | derivation §8, design log | An algebraic alternative that is ruled out in prose. |
| `[15, 60]` and `[105, 155]` mm | `cc-fk-gate.md` §2.1 | Ranges the derivation quotes for `a` and `d`; inputs, not results. |
| `4000` random poses (`w()` regression) | design log L584 | The check itself is not in the repo (see `w`, §2). |
| `64800` `z` points | `cc-summary` decision 18 | `540 x 120`; `zhome_bracket` prints the disagreement count (`0`) but not the denominator. |
| `3.11` rad | design log L1006, `cc-fk-gate.md` L324 | The metric table's top row. The module prints that column as `3e+00` (`%.0e`), so the two-decimal value is a formatting difference, not a missing measurement. |
| `8.5e-7 deg`, `2.7e7x` | several | Rounded restatements of measured values (`2.41e-06` floor; the ratio the gate prints as `30653057x`). |
| `0.5750`, `1.3475` | `cc-fk-gate.md` §7 table | Fixture C's bracket in `r_b` units; the gate prints it in mm (`57.500`, `134.750`) at `r_b = 100`. |

**5. Flagged by the matcher, not gaps** — `98973` (part of the commit hash
`1b98973`); `5.713988e-03` in `cc-summary` decision 11, which `branch_envelope`
prints as `-5.713988e-03` (the tokeniser split its Unicode minus); and section
numbers such as `2.3`, `2.4`.

**Everything else is backed.** Notably, all of the headline results now have a
module behind them: the envelope (`10.5290`, `6.5580`, `3.9710`, `1.2800`),
the azimuth verdict (`1.026e-13`, `5.648e-14`, `2.303e-01`, `6.588e-01`,
`3.736e-01`), the `z_home` brackets (`0.182733`, `0.983163`, `363/540`,
`176`/`1`, `0.2920624`, `0.29932`, `7.26e-03`, `5.551e-17`, `5.911e-04`,
`1.684e-03`), the branch re-run (`8.882e-16`, `1.78e-15`, `0.896361`), the
margin history (`-5.713988e-03`, `+1.550398e-01`, `+2.276643e-01`,
`1.534192e-01`, `1.531859e-01`, `1.8033e-01`, `3.6486e-01`, `3.6147e-01`,
`1.62e-3`), the ledger (`15625`, `2.7M`, `4.9e8`, `3.9 GB`), the sensitivity
correction (`1.60`, `5.62`, `7.30`), the arrest framing (`1.6356`, `1.6361`),
the gate (`9.552e-08`, `1.8532e-13`, `7.8203e-14`, `2.842e-14`, `0.074`,
`1.535`, `14436`, the four `cond` columns, the iteration histograms, the 32
roots), and the score study (`363`, `34`, `4.8652e-05`, `2.8801e-03`,
`6.939e-18`, `0.8348`, `1.332e-15`, `7.805e-01`, `7.9298e-01`, `0 of 124`,
`1.3651e-03`).

## 9.3 Divergences between code and document

Read off the code, flagged, not fixed.

1. **Index base.** Derivation §8 and §1 write `theta_i = 120·floor((i-1)/2)`
   with `i` running 1…6. `base_ring` implements `120*floor(i/2)` with
   `i = 0..5`. Same ring; `notation.md`'s Conventions block records that both
   appear.
2. **`Unreachable`'s direction test.** Docstring: `"far"` iff `P_i > C_i`. Code:
   `"far" if P[i] > 0.0`. Equivalent wherever the exception can be raised.
3. **`Geometry.summary` prints the two-sphere bound** `|d - a|`, `d + a` without
   the necessary-not-sufficient caveat that derivation §5.4, `ik`'s docstring
   and the design log all attach to it.
4. **`notation.md` §6 defines `rho_i = sqrt(d^2 - w_i^2)`; no code computes it.**
   The same is true of §8's unnamed *(amplitude)* `sqrt(A_i^2 + B_i^2)` and
   *(phase)* `atan2(B_i, A_i)`, and of §8's `k`. The design log's
   "precompute feasibility bracket" using `amp_i` is not in the repo.
5. **`J(delta) = max over envelope and legs of |w_i|`** (superseded objective)
   is not implemented as such; `azimuth_symmetry.aggregates` computes
   `max_i |w_i|` per pose as one of its two invariance test quantities, which is
   the closest thing in code.
6. **`delta*`'s closed form.** The design log records
   `delta* = atan2(-r_p sin A, r_b - r_p cos A) mod 180`.
   `zhome_datum._delta_G_deg` computes `arg(r_p e^{iA} - r_b) mod 360`, which is
   the same value mod 180. Only `delta_G` exists in code, under that name.
7. **`roll`, `pitch`, `yaw`** (`notation.md` §2) exist in code only as the
   *names* of two `known_poses` rows, whose matrices are single-axis rotations.
   No composition order is implemented anywhere, consistent with derivation §6
   being open.
8. **`azimuth_symmetry.py`'s `FIXTURES` carry a `z_home` column that is outside
   the feasible bracket on three of its four rows** (`cc-fk-gate.md` §7 records
   this). The module does not call `ik`, so nothing it measures is affected.
9. **`README.md` and `CLAUDE.md` describe five stubbed kinematics functions.**
   All five are implemented. See §10.

---

# 10. State of the repo

## 10.1 Git

* Branch **`phase-0-kinematics-through-ik`**, HEAD **`9514899`** "Commit the
  score discriminator study" (2026-09-08 00:53 +0200, Minseo Hong).
* **22 commits total**, 2026-09-01 → 2026-09-08. `main` is at **`369ffa4`**
  ("add platform_ring", 2026-09-02); the working branch is **20 commits ahead**
  of it and `main` has nothing the branch lacks.
* **No remote is configured** — `git remote -v` is empty. Every commit is local
  only, on one machine.

Commit sequence (newest first):

```
9514899  Commit the score discriminator study
0099f68  Fix the rotation metric, settle (R, T), reclassify the near-mode case
56a33e9  Record the fk gate in the design log
1d792a4  Implement fk() and the round-trip gate; the gate passes
73bdb8c  Give code to the three numbers that only existed in prose
c139369  Correction pass: withdraw the attribution table, correct the N_i > 0 claim
1b98973  Attribute the 177 empty z_home brackets; N_i > 0 does bind
280ed6a  Add the 2026-09-05 session summary
3d0fc42  Specify the working envelope, and propagate it through the documents
c9d14b2  Add envelope diagnostics: azimuth range, z_home brackets, branch re-run
46ce26f  Drop z_home = z_flat; restore z_home as the sixth sweep axis
2d16f03  Move §8 to follow §7, so the derivation reads 1..7, 8, appendix
e903a95  Commit branch_check.py, the script behind "8 of 432" and "-5.7e-3"
166733b  Add z_home datum diagnostics, and recover the grid behind "8 of 432"
42ace6a  Transcribe three sec.4 items the handoff settled but notation.md dropped
6e54917  Retract the affine rank-3 claim in make_geometry's docstring
f565162  Move design documents into docs/, and record the 2026-09-03/04 results
733703a  Implement kinematics through ik(), and re-establish the sec.7 table
d73f7f2  Thread h_p through the platform ring
9c8b70d  Add Phase 0 design documents
369ffa4  add platform_ring
7c163b4  Scaffold 6-RSS Stewart platform design project
```

## 10.2 Working tree

**Modified, uncommitted (2 files, +240 / -4):**

* `docs/notation.md` — +139/-4
* `docs/phase-0-design-log.md` — +105/-0

Both carry the 2026-09-07 score-function block (`score = margin(dxy = p)`, the
`cond <= 1e6` cap, the `|e|` collapse, the tune/score mismatch). This inventory
reads them as they stand in the working tree, not as committed.

**Untracked:** none that are not ignored. `.gitignore` covers `.venv/`,
`__pycache__/`, `*.png`, `.DS_Store`, so `demo.png`, `leg_constraints_table.png`
and the `.DS_Store` files are ignored rather than untracked. `leg_diagram.excalidraw`
**is** tracked.

Note: at the start of this session `git status` showed
`stewart/diagnostics/score_discriminators.py` as untracked and HEAD at
`0099f68`; commit `9514899` has since added it, so the file is now tracked.

**Tracked files (29):** `.gitignore`, `CLAUDE.md`, `README.md`, `demo.py`,
`requirements.txt`, `leg_diagram.excalidraw`, `test_kinematics.py`, the eight
documents in `docs/`, four modules in `stewart/`, and nine in
`stewart/diagnostics/` (including `__init__.py`).

## 10.3 Everything runs

Re-run 2026-09-08 on this tree, from the repo root:

| command | exit |
|---|---|
| `python test_kinematics.py` | **0** — ALL PASS |
| `python demo.py` | **0** — writes `demo.png` |
| `python -m stewart.diagnostics.envelope` | 0 |
| `python -m stewart.diagnostics.azimuth_symmetry` | 0 |
| `python -m stewart.diagnostics.branch_check` | 0 |
| `python -m stewart.diagnostics.zhome_datum` | 0 |
| `python -m stewart.diagnostics.zhome_bracket` | 0 |
| `python -m stewart.diagnostics.branch_envelope` | 0 |
| `python -m stewart.diagnostics.sweep_budget` | 0 |
| `python -m stewart.diagnostics.roundtrip` | **0 — GATE PASSES** |
| `python -m stewart.diagnostics.score_discriminators` | 0 |

## 10.4 Stale against the current code

**`README.md` — STALE, in three places.**

1. "What is mine, what is scaffolding" lists `platform_ring`, `make_geometry`,
   `stage1`, `legs`, `arm_tips`, `ik`, `fk` under **"Mine (to write)"**. All
   seven are written.
2. "Stubs, in the order they pay off" presents items 1–6 as a to-do list. All
   six are discharged.
3. Item 3 of that list describes `stage1` as *"the branch-independent
   coefficients `A cos(a) + B sin(a) = P` and the amplitude `C = hypot(A, B)`"*.
   That is not what `stage1` does — it is `q_i = T + R p_i`, and those
   coefficients now live inside `ik`. The implemented `stage1` docstring records
   the change; the README does not.
4. The Layout block says `kinematics.py  all solvers stubbed; Unreachable done`.
5. Not mentioned at all: `stewart/diagnostics/` (nine modules),
   `test_kinematics.py`, `docs/`.

**`CLAUDE.md` — STALE, in two places.**

1. The layout table's `stewart/kinematics.py` row reads
   *"`Unreachable` DONE; `stage1`, `legs`, `arm_tips`, `ik`, `fk` STUB
   (signatures + docstrings only)"*. Wrong for the first four since 2026-09-04
   and for `fk` since 2026-09-05.
2. The `stewart/geometry.py` row reads *"`platform_ring`, `make_geometry` STUB"*.
   Both are implemented.
3. "Implement the stubs in the order given in `README.md`" points at a list that
   is fully discharged.
4. The table has no row for `stewart/diagnostics/` or `test_kinematics.py`.

Still accurate in `CLAUDE.md`: the units and array-shape conventions, the
validate-at-construction rule, the 1-indexed error messages, the deps (the only
third-party imports anywhere in the tree are `numpy` and `matplotlib`; the rest
are stdlib — `sys`, `inspect`, `time`, `dataclasses`, `__future__`),
the note that `base_ring` is transcribed ground truth, the note that
`smoke_geometry` is not a layout, and the IK reachability rule (`|P| > C`, never
the two-sphere bound, never `np.clip`), which the code follows exactly.

**`demo.py` — STALE, in one place, and its `_kin_status` now proves it.**

The banner prints
`round trip (ik/fk stubbed -> every row reports 'not implemented')`. Nothing is
stubbed: the run above prints `stage1 written`, `legs written`,
`arm_tips written`, `ik written`, `fk written`, and the four rows report three
`ik Unreachable (leg 1, near)` and one real round trip at
`0.3163 mm` / `0.7296 deg`. The `Unreachable` rows are correct —
`smoke_geometry` is not a layout and its home pose genuinely does not reach.

`_kin_status()` itself still works, by searching each function's source for
`"raise NotImplementedError"`; it will keep reporting correctly, but its
premise — that some of them are stubs — no longer holds.

---

# 11. Conventions embedded in code, and what settles each

| convention | where it is in code | settled by |
|---|---|---|
| Millimetres and radians internally; degrees only at a boundary | throughout; `base_ring`/`platform_ring`/`tilt_R` take degrees, `arm_tips`/`ik`/`fk` radians | `CLAUDE.md`, `notation.md` Conventions |
| Anchor arrays `(3, 6)`, column `i` = leg `i`; `R @ p` from the left | `Geometry._as_36` (raises on any other shape) | `CLAUDE.md`, `README.md`, `notation.md` |
| Legs 0-indexed in code, 1-indexed in messages | `_bad_legs`, `Unreachable.leg = i + 1`, plot labels `i + 1`, `ANCHOR_KICK_LEG = 0` | `notation.md` Conventions |
| **Pose order `(R, T)`** — orientation first, in and out | `stage1`, `legs`, `w`, `ik`, `fk_solve`, `fk` | `notation.md` Conventions, **settled 2026-09-07** |
| **Branch: minus, fixed, not a parameter** | `ik`: `atan2(N, M) - arccos(P/C)` | derivation §6 (struck 2026-09-04) + `branch_check`, `branch_envelope`. Reopens if shafts are canted |
| **Reachability is `\|P\| > C`, tested before `arccos`, never clipped** | `ik`; also `scan_delta`, `roots`, `closed_form`, `bracket` | derivation §5.4, `CLAUDE.md` |
| Rotation increment applied on the **LEFT**, `R <- exp([omega]_x) R` | `fk_solve._apply`, `fk_jacobian`, `fd_jacobian`, `_fd_jcmd` | `cc-fk-gate.md` §3, verified at `9.552e-08` with a negative control |
| Residual **unsquared**, so the tolerance is a length | `fk_residual`, `FK_TOL_MM` | `cc-fk-gate.md` §4 |
| Tolerance is an **acceptance** test, stopping is stagnation | `fk_solve` | `cc-fk-gate.md` §4, measured |
| Rotation error is `\|log_so3(R_a^T R_b)\|`, not the `arccos` trace form | `geodesic_angle`, `_geodesic_deg`, `geodesic_deg` | `cc-fk-gate.md` §5, **changed 2026-09-07** |
| Anchor plane at `p_z = -h_p`, coplanar; no `mu` parameter | `platform_ring` | `notation.md` §4, settled 2026-09-03 |
| `delta in [0, 180)`, `beta`, `beta_p in (0, 60)` open | `base_ring`, `platform_ring` guards | derivation §8; the open endpoints are a **hardware** exclusion whose true bounds are unknown (`notation.md` §12) |
| Envelope: `dxy = dz = yaw = 0`, `tilt <= 10.5290 deg`, azimuth `[30, 90]`, 29 poses | `envelope.py`, imported by every other diagnostic | `notation.md` §9/§10, settled 2026-09-05, azimuth window measured |
| **Characteristic length: UNSETTLED** | `_cond_and_sigma(J, char_len)` has **no default** and returns `(None, None, None)`; the gate quotes `r_b` and labels it PROVISIONAL; `score_discriminators` carries all four | `notation.md` §12 — **open**. Narrowed 2026-09-07: it no longer blocks the score, and is still needed to quote `C` and for the residual-to-pose bound |
| Inner-tune cap `cond(J_fk) <= 1e6` at `char_len = r_b` | `SCORE_CAP` in `score_discriminators`; four caps carried side by side, none preferred | `notation.md` §8, recorded 2026-09-07; the module explicitly does **not** choose it |
| Score `= margin(dxy = p)`, `p` not fixed | measured in `score_discriminators` part (7) at three probes; **no scoring code exists elsewhere in the repo** | `notation.md` §8, settled 2026-09-07; `p` open |
| Rotation convention (roll/pitch/yaw order) | **not implemented anywhere** — the solver uses a rotation vector, which needs none | derivation §6 — **open, untouched** |
| `d > a` | **not enforced** | `make_geometry` Notes: documented, deliberately not checked |
| Two-sphere bound `\|d - a\| < \|L\| < d + a` | printed by `Geometry.summary`, used nowhere as a test | derivation §5.4: necessary, **not** sufficient |

---

# 12. Why each function is necessary

Necessity as the repo establishes it, not as it could be argued. Each entry says
**what depends on the function** and **what is lost if it is removed** — read off
the call graph in §8 and the measurements in §1–7. Five kinds appear:

* **STRUCTURAL** — something else cannot be written without it.
* **GUARD** — it refuses an input or a result that would otherwise pass silently.
* **DISCRIMINATING** — it catches an error that nothing else in the repo catches,
  or it is the control that gives another check its force.
* **PROVENANCE** — it puts a documented number under code.
* **NOT ESTABLISHED** — nothing depends on it as the repo stands. Listed in
  §12.7 rather than argued for.

---

## 12.1 `stewart/geometry.py`

| function | what depends on it | what is lost without it |
|---|---|---|
| `Geometry` | every solver, every diagnostic, `test_kinematics` | **STRUCTURAL + GUARD.** It is the one place the `(3, 6)` convention is enforceable. A `(6, 3)` array broadcasts through `R @ p` and applies the rotation transposed with no exception raised — the failure mode `CLAUDE.md`, `README.md` and the design log all name as producing entirely plausible numbers and a platform that tilts backwards. Checked once at construction instead of at every call site. Freezing and the read-only arrays are what make a `Geometry` safe to hand to 363 candidates in a loop. |
| `_as_36` | `Geometry.__post_init__` | **GUARD.** The shape test itself. Without it the transposition above is undetectable until a plot looks wrong. |
| `_check_unit` | `__post_init__` | **GUARD.** `ik` builds `M_i = L_i . u_i` and `N_i = L_i . v_i` and divides by `C_i = hypot(M, N)`. A `u_i` of norm ≠ 1 rescales both without changing any downstream residual, so nothing later in the repo would notice. |
| `_check_orthogonal` | `__post_init__` | **GUARD.** `v_i = n_i x u_i` is only an orthonormal in-plane partner when `u_i . n_i = 0`. If it is not, `(u_i, v_i)` stops being a basis for the servo plane and `arm_tips` places tips off the circle — while still returning six finite numbers. |
| `_bad_legs` | both checks above | Names the failing legs **1-indexed**, which is what makes a construction failure actionable against the hardware rather than against an array index. |
| `base_ring` | `make_geometry`, `smoke_geometry`, `branch_check.geom`, `test_kinematics` | **STRUCTURAL.** Written raw, the six servo normals are 18 components and the six shafts 18 more. `base_ring` reduces both to three scalars (`r_b`, `beta`, `delta`), which is what makes the sweep searchable at all — and it is where the mirror condition that forces `psi_i = theta_i + 90 + s_i delta` is imposed, so the `+90` is not a free constant anyone can drift. Its own asserts run on every call. |
| `platform_ring` | `make_geometry`, `branch_check.geom` | **STRUCTURAL.** The same reduction for the platform's 18 components, and the only place the settled anchor-plane convention (`p_z = -h_p`, six anchors coplanar) is applied. Sharing `theta_i`'s `120 floor(i/2) + s_i (.)` skeleton is what makes the base and platform mirror permutations come out identical, which is what lets one scalar `delta` serve all six servo planes. |
| `make_geometry` | 19 call sites across 10 modules (§8.2) | **STRUCTURAL + DISCRIMINATING.** It is the single composition point, and that is what gives `zhome_datum`'s independence discipline its force: the closed forms under test are written in the input parameters, and the anchors they are checked against come from the library. If each diagnostic rebuilt its own ring from the derivation's formulas, the checks would be testing the algebra against itself and would verify nothing. |
| `smoke_geometry` | `demo.py` | **STRUCTURAL** while the stubs stood — it let `plotting` and `round_trip` be shape-checked before a real layout existed. It has since become **DISCRIMINATING** for a reason it was not built for: it is the only geometry in the repo that is near-singular, and it is where the two-poses-one-command assembly-mode case was found. The gate's four fixtures sit at `cond` 4–10 and do not produce it. |
| `Geometry.summary` | `demo.py` | **NOT ESTABLISHED** — §12.7. |

---

## 12.2 `stewart/kinematics.py`

| function | what depends on it | what is lost without it |
|---|---|---|
| `Unreachable` | raised by `ik`; caught in six places | **GUARD.** The alternative the docstrings name explicitly is `np.clip(P/C, -1, 1)`, which maps every unreachable leg onto ±1 and returns a boundary angle that looks like a solution. Carrying `.leg` and `.direction` is what lets the gate count `ik unreachable: 0` **separately** from `fk not converged: 0`; collapsed into one number, a workspace boundary and a solver failure would be indistinguishable. |
| `stage1` | `legs`, and through it everything; `test_kinematics` rows 1–2; `zhome_datum` | **STRUCTURAL + DISCRIMINATING.** The frame conversion the whole derivation starts from. It is also one of only three functions checked against **hand-computed** values rather than against each other — and the gate's own verdict text records why that matters: `ik` and `fk` share `stage1`, `legs` and `arm_tips`, so a sign error common to both would round-trip perfectly at `1e-13 mm` and the gate would pass while wrong. Row 2 is the check that separates `R @ p` from `p @ R`. |
| `legs` | `ik`, `w` | **STRUCTURAL.** One home for the leg vector, so a sign error has one place to be rather than being inlined at each consumer. The design log records that `w` was repointed from an inlined copy at exactly this cost. |
| `arm_tips` | `fk_solve`, five diagnostics, `test_kinematics` rows 3–4 | **STRUCTURAL.** It is what makes the forward problem square: with the tips closed-form in `alphas`, six rod equations have six unknowns `(T, omega)` and Newton applies. Without it FK would be solving for the tips as well. Rows 3 and 4 pin both the length and the direction of the zero. |
| `ik` | the deliverable; `round_trip`, the gate, `branch_envelope`, `score_discriminators` | **STRUCTURAL.** The closed form the project exists to produce — pose in, six servo commands out. Everything downstream that ranks a candidate calls it, directly or through the quantities it defines. |
| `w` | nothing | **NOT ESTABLISHED** — §12.7. |
| `_v` | `arm_tips`, `ik`, `plotting` | **GUARD.** `v_i = n_i x u_i` equals `z` exactly while the shafts are horizontal, and the whole minus-branch argument rests on that. Computing it rather than substituting `z` is what makes a canted shaft change the answer visibly instead of silently: `zhome_bracket.check_n_and_v` measures `max \|v - z\| = 1.110e-16` today, and would measure the departure if the assumption were ever dropped. |
| `FKNotConverged` | raised by `fk_solve`, caught by the gate | **GUARD.** Its docstring states the cost of the alternative: a pose that did not converge is not a pose, and returning a best effort silently is how a round-trip gate comes to pass while wrong. The `reason` field (`cap` / `stalled` / `singular`) is what distinguishes running out of iterations from an arithmetic breakdown. |
| `FK_TOL_MM`, `FK_MAX_ITER` | `fk_solve`, `fk`, the gate | **GUARD.** The acceptance threshold and the cap that turns a non-converging pose into an exception rather than a hang. The tolerance's necessity is measured, not asserted: gate part (3) shows the worst pose error is `1.2681e-13 mm` at every tolerance from `1e-8` to `1e-13`, and acceptance failing on 234 poses at `1e-14` is what locates the arithmetic floor. |
| `exp_so3` | `fk_solve._apply`, six gate functions | **STRUCTURAL.** The solver's parameterisation of the rotation update. It is what keeps the iterate on `SO(3)` (measured drift `2.2e-16` before the final projection) **and** what keeps derivation §6 untouched: a rotation vector names an axis and an angle, with no ordered sequence of elementary rotations, so nothing here settles the open rotation convention by accident. |
| `_skew` | `exp_so3` | **STRUCTURAL.** `[v]_x` with `[v]_x w = v x w`, the one place the cross-product matrix is written. |
| `log_so3` | `geodesic_angle`, `_check_log_inverts_exp`, `_fd_jcmd` | **DISCRIMINATING.** It exists because the obvious alternative is wrong where it is used: `arccos((tr R - 1)/2)` has a floor of `~sqrt(2 eps)`, measured at `2.41e-06 deg`, because the trace carries the angle only at second order. The gate's real rotation error is `7.82e-14 deg` — seven orders under that floor, and it was being reported as `2.09e-06` until the metric was changed. The antisymmetric part `log_so3` uses is linear in the angle and has no floor. |
| `geodesic_angle` | `stewart.roundtrip._geodesic_deg`, `gate.geodesic_deg` | **STRUCTURAL.** One convention-free way to compare two orientations, shared by the harness and the gate. Keeping it in one place is what stopped the two copies diverging when the metric was fixed — the design log records that leaving one floored while fixing the other would have been worse than either. |
| `fk_residual` | `fk_solve`, the gate, `score_discriminators` | **STRUCTURAL.** The function being driven to zero. Unsquared by choice: `f_i = \|q_i - h_i\| - d` is literally the gap rod `i` fails to close, so the tolerance on it is a physical length and needs no conversion factor. |
| `fk_jacobian` | `fk_solve`, the gate, `score_discriminators._jacobians_at_pose` | **STRUCTURAL.** It is what makes the forward solve Newton rather than a search — median 5 iterations, max 8, against a cap of 100. Analytic rather than finite-differenced so that it can itself be verified: gate part (1) matches it to `9.552e-08` and rejects the three wrong candidate forms by 8–10 orders. It is also the matrix both conditioning studies are built on. |
| `_apply` | `fk_solve` | **STRUCTURAL.** Applies the increment on the **left** and re-zeroes `omega` each iteration, which is what keeps the rotation from being accumulated as a vector. |
| `_cond_and_sigma` | `fk_solve`, `score_discriminators` (`_cond_pair`, `_pose_smin`) | **GUARD.** `J`'s translation columns are dimensionless and its rotation columns are mm/rad, so no singular value means anything until a length names the exchange rate. The function exists as the single point where that length enters — and it takes **no default**, returning `(None, None, None)`, precisely so that `notation.md` §12's open item cannot be settled by a call site quietly picking one. Its necessity is as a refusal, not as a computation. |
| `fk_solve` | `fk`, the gate, `score_discriminators._fd_jcmd` | **STRUCTURAL.** The forward kinematics has no closed form — the asymmetry the design log opens with. Returning the whole record rather than just the pose is what lets a pass carry its own evidence: the gate's verdict is built from `residual_mm`, `iterations`, `lm_steps`, `cond` and `sigma_min`, and none of that would survive a bare `(R, T)` return. Reporting the LM fallback rather than hiding it is the same argument — counting *accepted* steps rather than attempts is what kept a real conditioning signal from being buried under the 96% of solves that end in one stagnant iteration. |
| `fk` | `stewart.roundtrip.round_trip` | **STRUCTURAL.** The round trip needs a pose-in, pose-out map; `fk_solve`'s record is the wrong shape for it. |

---

## 12.3 `stewart/plotting.py`

| function | what depends on it | what is lost without it |
|---|---|---|
| `draw_pose` | `demo.py`, `compare_branches`, `animate` | **DISCRIMINATING, by design rather than by measurement.** The design log states the job: a transposed rotation matrix produces plausible numbers and a platform that tilts the wrong way — visible in one frame of a 3D wireframe and effectively invisible in a column of floats. The `{W}` and `{P}` triads and the 1-indexed leg labels are what make that visible. Nothing asserts on its output, so the necessity is structural to the debugging path, not verified. |
| `arm_circles` | `demo.py` | Draws the constraint `ik`'s reachability test encodes — the circle, not the sphere. It is the picture of why the two-sphere bound is not sufficient. |
| `_arm_tip_points` | `draw_pose`, `compare_branches`, `animate` | **STRUCTURAL to `plotting`, and a second copy of `arm_tips`' formula.** It exists so that `plotting` never solves kinematics — it places a point on a circle already fixed by the geometry. The cost is that the arm-tip formula now lives in two files. |
| `_cube`, `_apply_cube` | all three drawing functions | Fix one set of axis limits over every drawn point set, so that motion is not hidden by autoscaling and two branches can be compared on the same scale. |
| `_triad` | `draw_pose` | Draws the two frames the whole derivation is about. Frame confusion is the error the design log names as its first real one. |
| `compare_branches`, `animate`, `plot_angles` | nothing | **NOT ESTABLISHED** — §12.7. |

---

## 12.4 `stewart/roundtrip.py`

| function | what depends on it | what is lost without it |
|---|---|---|
| `round_trip` | `demo.py` | **STRUCTURAL, with one load-bearing detail.** It is the generic harness: `ik` and `fk` are passed in, which is what let it run against stubs and report `not implemented` per row rather than crashing. The detail that carries the test is the **seed offset** — seeded at the truth a numerical FK sees zero residual and returns immediately, so the round trip would pass for any `ik`, including one that returns zeros. The zero-offset case is allowed but prints a warning saying the run proves nothing. |
| `known_poses` | `demo.py` | Derivation §7's rule — check `R = I`, `T = 0` and 90° about `z` before trusting anything, because those are answers that can be confirmed by hand. |
| `_rot_x`, `_rot_y`, `_rot_z` | `known_poses` | Build those three cases as single-axis rotations, so no composition order is implied and derivation §6 stays untouched. |
| `_axis_angle` | `round_trip`, `random_poses` | Generates a valid `SO(3)` member for the seed offset. Its docstring is explicit that it is a generator for testing, not a claim about any Euler convention. |
| `_geodesic_deg` | `round_trip` | Delegates to `kinematics.geodesic_angle`, so the harness and the gate measure rotation error with one formula rather than two. It carried the `arccos` floor until 2026-09-07 and was fixed with the gate's copy. |
| `_unpack`, `_format` | `round_trip` | Accept `(name, R, T)` or `(R, T)`, and print the table. |
| `random_poses` | nothing | **NOT ESTABLISHED** — §12.7. |

---

## 12.5 `stewart/diagnostics/` — why each module exists

| module | why it is necessary |
|---|---|
| `envelope.py` | **STRUCTURAL, single source of truth.** Every other diagnostic imports the tilt limit from here. The reason is on the record: the provisional 6° drifted across scripts and documents once already, and a constant that lives in one place cannot. The limit is **computed** from the recovery model at import rather than typed, so it cannot disagree with its own derivation. |
| `azimuth_symmetry.py` | **DISCRIMINATING.** It tests a claim that was made without being checked, and half-refutes it: period-120 holds (`1.026e-13`) and the mirror at `psi = 90` holds (`5.648e-14`), but the mirror at `psi = 0` **fails at `2.303e-01`**. The consequence is not cosmetic — `[0, 60]` is symmetric about its own centre, so it covers some orbits twice and misses the orbit `{75, 105}` entirely; sweeping it would silently omit part of the envelope. `envelope.AZIMUTH_WINDOW_DEG = (30, 90)` is this measurement. |
| `branch_check.py` | **PROVENANCE.** It is the script behind two documented figures (`-5.7e-3` and "8 of 432"), recovered from a session scratchpad and committed because it had never been in git history. The design log records it as the third instance of a documented result with no code behind it. |
| `zhome_datum.py` | **PROVENANCE + DISCRIMINATING.** Recovers the 432-point grid verbatim and re-derives the original 8 as a check on the recovery, then tests the flat-arm closed form against the library's anchors — never against a rebuilt ring, which would test the algebra against itself. Its `[B0]` block is what turned "8 combinations are infeasible" into "all 8 are feasible on the other half-circle", i.e. an artifact of holding `delta = 40`. |
| `zhome_bracket.py` | **STRUCTURAL to the sweep.** `z_home` is one of the six sweep axes and had no range; this brackets it from both ends. The lower bound comes out in closed form and is `delta`-, `a`- and `d`-free, and the module's own attribution block is what corrected the claim it started from: the "0 of 363" sample could not have contained a counterexample, and the sample that could does — `N_i > 0` closes one bracket outright and removes a spurious low-`z` reach component in six more. |
| `branch_envelope.py` | **DISCRIMINATING.** The branch evidence was gathered at one `z_home` under a provisional 6° envelope; both premises were later withdrawn. This re-runs it across the restored axis at the settled limit, and recomputes the `-5.7e-3` on the same geometry so the two numbers can be compared — while stating that the two envelopes are not nested, so a margin that improves is not evidence the geometry got better. |
| `sweep_budget.py` | **PROVENANCE.** Three documented results had been arrived at by hand. One of them, the `3.9 GB` figure, carries a **harness requirement** (chunk over candidates). Giving them code changed one: the documented "order 7" translation sensitivity is not confirmed — measured `~1.36`, because the row it was divided out of was `T_horiz` and `T_vert` together, and the vertical part is `z_home`, already a swept axis. |
| `roundtrip.py` (gate) | **STRUCTURAL, the hard stop.** Nothing downstream is worth writing until `ik` and `fk` are shown mutually consistent. It is the only module that exits non-zero on failure. Its seed is HOME — fixed and neutral — because a solver seeded from the answer works only when the answer is already known. |
| `score_discriminators.py` | **STRUCTURAL to the sweep, and a refusal.** It measures every candidate discriminator on the feasible set and reports how they relate, while picking none: both Jacobians carried the whole way and never merged, all four characteristic lengths quoted, no scalar score formed. Its part (5) is what showed the maximin-margin tuner does not merely fail to see the rank degeneracy at `beta_p == beta` and `delta = 0` — it **selects** it, buying `cond = 1.07e+19` for a margin difference in the third decimal place. |

### Within the diagnostics, the checks that carry the others

| function | why it is necessary |
|---|---|
| `azimuth_symmetry.broken_geometry` | **DISCRIMINATING (negative control).** Displaces one platform anchor by `0.03 r_b` so D₃ is broken and nothing else changes. Without it a pass at `1e-13` could be a pass for the wrong reason; with it, the same tests fail at `6.6e-1` and `3.7e-1` as intended. |
| `gate._jacobian_control` | **DISCRIMINATING (negative control).** A finite-difference check that cannot tell the candidate forms apart proves nothing about the one it endorses. The three at risk — sign-flipped, right-perturbation, and right-and-flipped — are rejected at `2.000`, `4.307e-01` and `1.974` against `9.781e-10`. The right forms differ by an `R^T` as well as a sign, so "flip the sign if it does not match" would not have found the bug had there been one. |
| `gate.fd_jacobian` | **DISCRIMINATING.** Differences through the **same left perturbation** the analytic form claims. Differencing a right perturbation against a left-perturbation formula would report a mismatch that is a property of the test, not of the formula. Sweeping five step sizes is what separates "the formula is right" from "the step was lucky". |
| `gate.check_metric_agreement` | **DISCRIMINATING.** Replacing a metric inside a gate invites the suspicion that the gate now passes because it measures something easier. Both forms are compared against a **known** angle built by `exp_so3`, so neither formula defines the answer: they agree to `4.6e-12` where `arccos` is well conditioned, and only then does the floor argument apply. |
| `gate._check_log_inverts_exp` | **DISCRIMINATING.** The conditioning table says the metric resolves small angles; it does not say the implementation is right. `log_so3` carries a hand-written branch for `theta` near `pi` that nothing else in the project exercises, so angles are drawn to hit it deliberately — worst inversion error `1.404e-15` over 20 000 rotations. |
| `gate.bracket`, `gate.z_samples` | **STRUCTURAL.** Every gate pose has to be one `ik` can actually solve. `z_samples` takes the **interior** of the bracket because the ends are exactly where `ik` sits on its reach boundary and a finer pose grid steps outside. |
| `gate.probe_modes` | **DISCRIMINATING.** "0 different-mode returns" is equally consistent with "the home seed is reliable" and with "the classifier never fires". Re-solving the same six angles from 400 random seeds finds 8 distinct roots per fixture, every one at the same arithmetic floor — so the classifier is exercised, and the zero means something. |
| `gate.plausibility` | **DISCRIMINATING.** Re-runs `ik` at each recovered pose. That clause is what turns a second root into a design finding: two poses `0.32 mm` apart returning the **same six angles** are two poses the machine cannot distinguish from its own commands. |
| `gate.report_movement` | **GUARD.** A discrete pose grid had flattered a worst case in the unsafe direction three times before this gate; running coarse/medium/fine and reporting whether the answer moves is what stops a fourth going unnoticed. It does move (1.14×–1.47×), and the module says why that means nothing here. |
| `zhome_bracket.reach_ceiling_bisect` | **GUARD.** The `z` grid step is `0.025`, and the one crossing found had a grid gap of `0.024` — inside a single step. Refining to `1e-9` before calling it a crossing is what separates a finding from a sampling artifact. |
| `zhome_bracket.z_lower_closed_form` | **GUARD.** A discrete pose grid reports `N_i > 0` satisfied slightly *before* it truly is (`+5.911e-04` on the harness grid). Taking the bound from the continuum formula rather than the grid is what keeps the attribution from being biased towards blaming reach. |
| `zhome_bracket.leg_terms` / `reach_feasible_any_delta` | **STRUCTURAL.** `w_i(delta)^2 <= \|L_i\|^2 - P_i^2` is exactly `\|P\| <= C` squared, and `A`, `B`, `G` are all `delta`-free — which turns a nested search into one pass and is what makes a 540-candidate × 120-height × 180-`delta` scan runnable. |
| `score_discriminators._delta_basis` / `_invariants` / `scan_delta` | **STRUCTURAL.** `delta` enters only `n_i` and `u_i`, never `L_i`, so `\|L_i\|` and `P_i` do not move with it and the whole dependence sits in `w_i(delta) = A_i cos delta + B_i sin delta`. That is what makes a 180-point `delta` scan per candidate cheap enough to run 363 times, and `scan_delta` closes it with one batched SVD. |
| `score_discriminators.verify_jcmd` / `_fd_jcmd` | **DISCRIMINATING, and it gates the module.** `J_cmd` is the one piece of new algebra the module introduces, so it is finite-differenced against the **real** servo-angle-to-pose map — perturb one `alpha`, re-solve `fk`, read the pose change off — which knows nothing about `J_fk`, `D`, or the implicit-function argument. The evidence is the `O(h^2)` **ratio** (10–98 for a tenfold step cut), not the magnitude: a wrong sign or a missing factor of `a` would leave an `O(1)` discrepancy that does not shrink with `h`. |
| `score_discriminators._kick_pair` | **DISCRIMINATING (negative control).** Displaces one anchor by a fixed vector on the same leg index in every candidate, which is what breaks a rotation relation between two candidates instead of respecting it. The design log records that the first version of this control was run on one leg only, and `margin` is a min over legs — so a displacement on a leg that is not binding moves nothing, and a control that moves nothing tests nothing. |
| `score_discriminators.tune_constrained` | **GUARD.** Returns `(None, nan)` when no `delta` on the scan meets the cap, rather than silently falling back to the unconstrained winner — so a candidate with no admissible configuration is reported as such instead of being scored as if it had one. |
| `score_discriminators._ranks`, `_spearman`, `_five_number` | **STRUCTURAL.** numpy-only statistics, per `CLAUDE.md`'s dependency rule. Each handles `inf` deliberately: an infinite condition number is a real ordering statement ("worse than every finite one"), not missing data, so it is kept and ranked last rather than dropped. |
| `envelope.tilt_R`, `envelope_poses` | **STRUCTURAL.** The pose grid every other module sweeps. Magnitude 0 appears **once** because at zero tilt every azimuth is the same pose; counting it seven times would be six duplicates. |
| `envelope.bang_bang_accel`, `tilt_for` | **PROVENANCE.** They are why the tilt limit is a computed `10.5290` rather than a typed `10.5` — a typed constant cannot be checked against its own derivation. |

---

## 12.6 `test_kinematics.py` — why four rows and not two

| row | why it is necessary |
|---|---|
| 1, `stage1(R=I, T=0) == p` | The base case: with the platform level and at the origin the expression must collapse to the design constants. Residual `0.000e+00`. |
| 2, `Rz(90) sends (10,0,0) -> (0,10,0)` | **DISCRIMINATING.** The purpose-built `Geometry` puts the probe column at literally `(10, 0, 0)`, because a transposed `R` sends it to `(0, -10, 0)` — a residual of 20 against the `6.123e-16` measured. Nothing else in the repo separates `R @ p` from `p @ R`. |
| 3, `\|arm_tips(0) - b\| == a` | The length constraint on all six legs. Residual `5.551e-17`. |
| 4, `arm_tips(0) == b + a*u`, **and its control row** | **DISCRIMINATING, and the control is the point.** `n` is also a unit vector, so a tip placed along `n` instead of `u` is still exactly `a` from the shaft and passes row 3 — the control row measures that directly, at `5.551e-17`, the same residual row 3 reports for the correct tips. Row 3 alone therefore cannot catch a `u`/`n` swap; row 4 can, at `0.000e+00`. |
| the table as a whole | **STRUCTURAL to the gate's claim.** The gate's own verdict text names it: `ik` and `fk` share `stage1`, `legs` and `arm_tips`, so a sign error common to both round-trips perfectly. These four rows are the only place those three are checked against hand-computed values rather than against each other. |

---

## 12.7 Functions whose necessity the repo does not establish

Listed rather than argued for. Each is reachable, documented and passes import;
nothing depends on any of them, and no test or diagnostic calls them.

| function | position |
|---|---|
| `kinematics.w` | **No caller anywhere.** The quantity is load-bearing — `w_i = L_i . n_i` is what the inner `delta` tune trades against, and it is the whole content of the "two-sphere bound is not sufficient" result — but every consumer forms it inline (`azimuth_symmetry.aggregates`, `zhome_bracket.leg_terms`, `score_discriminators._invariants`). The function is the only public solver entry point with no caller. |
| `plotting.compare_branches` | Built for the `±` branch question. That question was settled numerically instead — `branch_check` `[1b]`/`[2d]` and `branch_envelope`'s continuity block — so nothing calls it. |
| `plotting.animate` | No caller. No module in the repo generates a pose sequence. |
| `plotting.plot_angles` | Built for the failure mode the design log names — one servo trace stepping while the other five stay smooth. That is now tested numerically by the step-outlier ratio `max\|dalpha\| / median\|dalpha\| > 5` in `branch_check` `[2d]` and `branch_envelope`, both of which report `0` outliers. Nothing calls the plot. |
| `roundtrip.random_poses` | No caller. `round_trip` is only ever handed `known_poses`; the gate builds its own envelope grid instead. |
| `Geometry.summary` | Called only by `demo.py`, and what it prints — `\|d - a\|`, `d + a` — is the two-sphere bound, which derivation §5.4, `ik`'s docstring and `CLAUDE.md` all record as necessary and **not** sufficient, and which is used as a test nowhere in the repo. |
| `kinematics.legs` (externally) | Called only from inside `stewart/kinematics.py`. `test_kinematics.py` imports it and never calls it. |
| `Geometry`'s validation branches | No caller constructs a deliberately invalid `Geometry`, so the shape, unit-norm and orthogonality rejections have never fired in a committed run. |
| `stewart/__init__.py`'s `__all__` | Nothing imports the package through it; every module uses explicit submodule imports, and two of them import names (`fk_solve`, `_cond_and_sigma`) that `__all__` does not list. |
