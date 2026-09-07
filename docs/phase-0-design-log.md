# Stewart Platform — Design Log

**Phase 0: kinematics and design method.** A 6-DOF parallel manipulator built as a
ball-balancing platform. Six rotary servos, rigid arms, fixed-length push rods.
100 hours budgeted across the project.

A chronological record of the design work: the reasoning, the decisions and why I
made them, the mistakes, and the results. Dates are the order things were actually
worked out, not a retrospective tidy-up.

---

## 19–20 August

### The decision that shaped the project

The project brief I started from wasn't my own, and I hadn't internalised it.
Tested on six questions about the mechanism at the outset, I could answer none. I
did not know what a pose was, or what a degree of freedom was.

What I decided to do about that shaped everything since. Carrying an unexamined
plan through to a built machine would leave me unable to answer the only question
that matters about a mechanism design: *why this geometry?* A demonstrator I can't
defend is worth less to me than no demonstrator.

So I fixed the method before any of the engineering: derive the kinematics myself,
in symbols, and let the mathematics produce the dimensions — rather than choosing
dimensions and constructing a justification afterwards.

One rule follows from that and I've held to it since: **nothing is purchased and
no CAD is drawn until the geometry sweep has chosen the dimensions.** Every number
in the eventual build has to be traceable to the analysis.

### The mechanism

Six legs connect a fixed base to a moving platform. Each leg is a servo bolted to
the base, a rigid arm on the servo shaft, and a push rod of fixed length running
from the arm tip to an anchor point on the platform. Six servo angles in, one
platform pose out.

A *pose* is position plus orientation: three numbers for where the platform centre
sits, three for how it's tilted. Six inputs, six outputs — hence six degrees of
freedom.

Two frames are needed throughout:

| | |
|---|---|
| `{W}` | World frame. Fixed to the base and the table. Never moves. |
| `{P}` | Platform frame. Attached to the moving plate; tilts with it. |

Labelling every vector with the frame it's expressed in turned out to be the
single most valuable habit I adopted, and its absence caused my first real error.

There's an asymmetry in this class of mechanism worth stating early, because it
shapes the whole approach. For a *parallel* manipulator the inverse kinematics —
pose in, joint angles out — has a closed-form solution, while the forward
kinematics does not and has to be solved numerically. That's the reverse of the
serial-arm case most people meet first.

### Stage 1: locating the platform anchors

The platform anchors are holes in a printed ring. Their coordinates in `{P}` are
fixed design constants, denoted `p_i`. Their coordinates in `{W}` change whenever
the platform moves.

My first instinct was to add the platform centre position `T` to `p_i`
componentwise. Working out why that's wrong was the moment the frame concept
became concrete for me: one unit along the world x-axis is not one unit along the
platform's x-axis unless the platform happens to be level. The two vectors are
measured against different rulers, and the rotation matrix `R` is what reconciles
them — it converts a vector's `{P}` components into `{W}` components.

```
q_i  =  T  +  R p_i
```

I checked it two ways before proceeding. With `R = I` the expression must collapse
to plain vector addition, and it does. With `T = 0` and a 90° rotation about z, an
anchor must swing to a position I can confirm by hand, and it does.

### Stage 2 geometry: circle meets sphere

The servo arm swings in a fixed plane, so the arm tip is confined to a **circle**
of radius `a` centred on the servo shaft.

The push rod is rigid, so the tip is always exactly `d` from the platform anchor —
a **sphere** of radius `d` centred on `q_i`.

The arm tip is wherever the two surfaces meet. Three cases follow, and I worked
out all three geometrically before attempting any algebra:

- **Two intersections** — the general case. The arm can be swung either way to
  place the tip correctly.
- **One** — circle tangent to sphere. The limit of that leg's reach.
- **None** — the leg can't reach. This happens two distinct ways: the anchor too
  far for arm and rod to span between them, or too close, with the circle sitting
  entirely inside the sphere.

The third case is what will define the machine's workspace boundary. Any pose in
which any one leg has zero solutions is a pose the platform physically cannot hold.

### The leg vector, and a productive dead end

This took several attempts and was the hardest part of the first session.

I went looking for a single global reference point for the machine as a whole. I
tried the world origin, then treating the platform as one rigid body, then the
centroid of the six servo positions — which felt clever at the time and leads
nowhere. The reason it leads nowhere is the useful part.

The resolution: **the inverse kinematics is six completely independent problems.**
Leg 3's geometry makes no reference to servos 1, 2, 4, 5 or 6, nor to the base as
a whole. Six separate problems that share only the pose they start from. Leg *i*'s
circle is centred on servo *i*'s own shaft and nothing else.

Once that's established, only two points matter for leg *i*: the anchor `q_i` and
the shaft `b_i`. And since the geometry is unchanged if the whole machine is
carried across the room, what matters is neither absolute position but the vector
between them:

```
L_i  =  q_i  -  b_i  =  T  +  R p_i  -  b_i
```

An important distinction I nearly missed: `|L_i|` is **not** the rod length. It's
the distance the arm and rod have to span *together*. Dividing that span between
them is precisely the stage 2 problem.

### Sizing the problem before solving it

Four quantitative results, produced to establish what the machine actually has to
do before I chose any dimensions.

**Minimum tilt to arrest the ball: 1.635°.** By energy method, for a 200 mm/s ball
brought to rest within 100 mm of travel.

I'm treating that figure carefully. It's a *minimum*, not a design target — what's
required with zero margin, assuming no latency, no friction variation and no
external disturbance. Designing to it is designing to fail. It's a floor to stay
well clear of, and the actual target with justified margin is still mine to set.

**Ball travel during a 150 ms latency window at 300 mm/s: 45 mm.** On a plate of
this size that's a substantial fraction of the working area, which establishes
that control latency is a first-order concern rather than a refinement.

**Worst-case leg force is not a live constraint.** The ball is 2.7 g. No plausible
geometry will leave the servos struggling to support it. I ruled it out early so I
wouldn't spend more time on it.

**The result that most changed how I'm approaching the sweep.** How many
distinguishable tilt steps does the platform have? It's usable servo travel
divided by servo deadband. The geometry ratio — how much platform tilt results
from one degree of servo rotation — appears in both numerator and denominator and
**cancels**.

The consequence isn't obvious and it reframes the whole optimisation: geometry
does not determine how many tilt steps I get. It determines how they're *spent* —
a wide tilt range with coarse steps, or a narrow range with fine ones. There's no
geometry that offers "more control" in the resolution sense; that quantity is
fixed by the servo. The design problem is choosing where to spend a fixed budget
of resolution, which is a very different question from the one I started with.

One process note. Partway through the analysis a 300 mm/s ball speed appeared in
my working with no traceable source. I stopped and asked where it came from rather
than building on it.

---

## 28 August

### Why there is software at all

Before writing any code I wanted to be explicit that the software isn't an
implementation of a finished design. It has three jobs, and only the third
resembles what people usually mean by "the code".

**It's how the geometry gets chosen.** A design sweep means: generate candidate
dimensions, run the inverse kinematics across a grid of poses, score the result,
and repeat some thousands of times. That requires an IK I can *call*. Symbolic
results on paper can't be swept. The Python is therefore a design instrument — the
mechanism by which my derivation becomes the numbers I order parts against.

**It's how I find out the derivation is wrong.** The round-trip test runs a pose
through the IK to six servo angles, then through a numerical forward kinematics
back to a pose. If the pose returns, the mathematics is self-consistent. If it
doesn't, there's a sign or frame error — found now, rather than after I've cut six
push rods to a length computed from a bad equation.

**It's the debugger.** A transposed rotation matrix produces entirely plausible
numbers and a platform that tilts the wrong way. That's visible in one frame of a
3D wireframe and effectively invisible in a column of floats.

### Establishing what is and is not a design variable

The complete list of quantities a leg's inverse kinematics consumes: the platform
anchor positions `p_i`, the servo shaft positions `b_i`, the arm length `a`, the
rod length `d`, and the orientation of each servo's rotation plane. Nothing else
about the machine enters the calculation, so those are my design variables and
there are no others.

I can't sweep them as raw numbers. Six anchors at three coordinates each, twice
over, is thirty-six values before `a` and `d` are counted. A thirty-eight
dimensional search isn't a sweep. The anchors therefore need a *parameterisation*
— a small set of controls that generate all six positions from structure I can
defend. Three or four is tractable; twelve isn't.

Two further distinctions I recorded so I don't conflate them later:

- `a` and `d` are different kinds of variable. `a` is restricted to servo horns
  that commercially exist — a small discrete set. `d` I cut to length myself and
  it's effectively continuous.
- The servo rotation planes are a design variable, not fixed scenery. Which
  direction each servo faces changes what the mechanism can do. Easy to overlook
  because, unlike the others, it isn't a length.

### Conventions

Fixed deliberately and early, since convention errors in this domain are silent:

- **Millimetres and radians** throughout the code. Degrees appear only when
  printing or building a servo command. Units checked on every line — I added a
  rotation to a radius once.
- **Column vectors.** Anchors stored as (3, 6) arrays with column *i* being leg
  *i*. This matters more than it appears: stored row-wise as (6, 3), the
  expression `p @ R` executes without error and returns a result in which every
  value is wrong, because it applies the rotation transposed and tilts the
  platform backwards. No exception is raised. The correct form is `R @ p`.
- **Every equation tested on a known case** at the moment it's written, not at the
  end. `R = I`, `T = 0`, 90° about z.

### Stage 1 in code

```python
return R @ geom.p + T.reshape(3, 1)
```

The reshape is load-bearing. `T` arrives with shape (3,), and numpy would attempt
to align it against the last axis — the six legs — rather than the three spatial
components. Forcing it to (3, 1) broadcasts it across the columns as intended. The
matrix product handles each anchor independently: (3,3) @ (3,6) yields column *i*
as `R p_i`.

Verified against both known cases. `R = I, T = 0` returns the design constants
unchanged; a 90° rotation about z sends an anchor at (10, 0, 0) to (0, 10, 0),
confirming `R` isn't transposed.

The leg vector follows. I wrote it in terms of the stage 1 function rather than
expanding it inline, so a change propagates and there's a single place to make a
sign error:

```python
return stage1(geom, T, R) - geom.b
```

### Locating the arm tip

This is the forward half of stage 2 and the expression the closed form inverts.

The observation that makes it tractable: the arm tip position `h_i` appears to be
three unknown numbers, but it isn't. The tip is confined to a circle, and **a
point on a curve requires one number to specify**. That number is the servo angle
— which is the quantity I'm solving for in any case. The two circle constraints
are therefore not solved but *spent*, collapsing `h_i` from three unknowns to a
function of one.

Writing it down requires a coordinate system inside the servo's plane, expressed
in world components. The plane is tilted in general, so the world x and y axes
won't serve. I need two perpendicular unit vectors lying in the plane:

- `u_i` — the direction I define as `alpha_i = 0`. A free choice.
- `v_i = n_i × u_i` — the cross product with the plane's normal. Perpendicular to
  `n_i` and therefore lying *in* the plane, perpendicular to `u_i`, and of unit
  length because both inputs are unit and mutually perpendicular.

Giving:

```
h_i  =  b_i  +  a ( u_i cos alpha_i  +  v_i sin alpha_i )
```

A conceptual error on the way to this, worth recording because I suspect it's a
common one. I first tried to write the bracket as a coordinate triple,
`(u cos α, v sin α, 0)`. That object doesn't exist. A tuple `(x, y, z)` has
meaning only once the basis its numbers are measured against has been fixed
elsewhere; the basis vectors can't appear inside it. And the zero has nowhere to
live — a third component would require a third basis vector, which would be `n`,
and the tip never leaves the plane. The correct object is a **sum of vectors**,
not a triple.

Verified two ways, the second being the one that matters. Every tip is exactly `a`
from its shaft; and at `alpha = 0`, `h` equals `b + a·u`. The distance check alone
can't detect `u` and `n` being swapped, since `n` is also a unit vector — a tip
placed along it is also exactly `a` away, pointing in an entirely wrong direction.

### The closed form

One constraint remained unused and one unknown remained in it.

**Substituting.** Into `|q_i - h_i| = d`, with `h_i` now a function of `alpha_i`,
and grouping `q_i - b_i` back into `L_i`:

```
| L_i  -  a ( u cos alpha + v sin alpha ) |  =  d
```

Two errors here that I had to be corrected on. First, I read the bars as absolute
value and concluded squaring wasn't permitted. They denote vector **magnitude**,
and `|A| = √(A·A)` — so squaring is precisely what removes the root, cleanly,
since both sides are non-negative. Second, I initially wrote the expansion as
`|A - B|² = A² - B²`, which is the difference of two squares and isn't true even
for scalars. The correct expansion:

```
|A - B|²  =  (A-B)·(A-B)  =  |A|²  -  2 A·B  +  |B|²
```

**Expanding.** Taking `|B|²` first, with `B = a(u cos α + v sin α)`. Multiplied out
as a product of two sums, four terms result; `u·u = 1`, `v·v = 1` and `u·v = 0`
kill the two cross terms, leaving `a²(cos² + sin²) = a²`.

That outcome is forced, and recognising it gives me a check worth keeping. `B` is
the servo arm, and the arm's length is `a` regardless of where it points. Had the
algebra produced anything else, something upstream was wrong.

The middle term:

```
L·B  =  a ( L·u cos alpha  +  L·v sin alpha )
```

`L·u` and `L·v` are scalars — the anchor's coordinates *within* the servo's plane
— computable the moment the pose is known, and containing no `alpha`. Sorting
every term into known and unknown is what made the structure of the solution
visible to me.

Assembled and rearranged with the unknowns isolated:

```
d²  =  |L_i|²  -  2a ( L·u cos alpha  +  L·v sin alpha )  +  a²

M cos alpha  +  N sin alpha  =  P

M = L·u        N = L·v        P = ( |L_i|² + a² - d² ) / 2a
```

**Inverting.** A weighted sum of a sine and a cosine at the same frequency
collapses to a single shifted sinusoid. I didn't accept the identity on assertion
— its derivation is short and worth keeping. Treat `(M, N)` as a point in an
abstract plane, unrelated to the servo's plane, and express it in polar form as
`M = C cos φ`, `N = C sin φ`. That's always possible; it's Cartesian to polar and
nothing is lost. Substituting:

```
M cos α + N sin α  =  C cos φ cos α + C sin φ sin α
                   =  C ( cos φ cos α + sin φ sin α )
                   =  C cos( α - φ )
```

The bracket is the cosine difference identity. The unknown now appears exactly
once and the expression inverts directly:

```
C        =  sqrt(M² + N²)
phi      =  atan2(N, M)
alpha_i  =  phi  ±  arccos( P / C )
```

**A trap worth documenting.** I first wrote the phase as `arctan(N/M)`. Its range
is only (−90°, 90°) — half a turn — and the division discards the sign information
distinguishing `(M, N)` from `(−M, −N)`. Roughly half the servos would be 180° out,
presenting on the bench as an apparent wiring fault. `atan2` retains both signs and
covers the full circle.

**The three cases, recovered.** Everything I'd predicted geometrically now appears
as a condition on `arccos` requiring its argument in [−1, 1]:

| | | |
|---|---|---|
| `\|P\| < C` | 2 solutions | circle cuts sphere twice |
| `\|P\| = C` | 1 | tangent — the workspace boundary |
| `\|P\| > C` | 0 | unreachable |

The algebra produced nothing I hadn't already reasoned out with a pencil, which is
the strongest evidence available to me that both are correct.

### Verification

I checked the closed form by running the problem backwards. Six arbitrary servo
angles chosen; the forward map computes the corresponding arm tips; anchors placed
at exactly `d` from each tip; then the closed form asked to recover the original
angles.

It did, on every leg. Five returned on the negative branch and one on the positive
— not a defect in the formula, but the two-solution ambiguity appearing in
concrete form for the first time.

---

## 1 September

### Why the servo planes needed parameterising first

The sweep needs a candidate geometry described by a handful of numbers. Written
raw, the servo rotation planes are six unit normals — eighteen components, and
they sit alongside eighteen more for the platform anchors and eighteen for the
shaft positions. That isn't searchable.

The second reason is the governing rule of the project. Six independently chosen
normals have no answer to *why this geometry?* A generating rule does.

An ordering constraint I hadn't seen until I tried to start: the normals sit on
top of the shaft positions, so whatever symmetry the base layout has, the normal
rule has to respect it. That means the base layout has to be fixed first. Trying
to write the normal rule before deciding the base pattern is assuming a symmetry
that may not exist, which is why the problem felt like it had no entry point.

### The base ring

Three pairs on a threefold orbit, mirror-symmetric within each pair:

```
s_i      = (-1, +1, -1, +1, -1, +1)
theta_i  = 120·floor((i-1)/2)  +  s_i·beta
b_i      = r_b (cos theta_i, sin theta_i, 0)
```

Two parameters, `r_b` and `beta`. The range is `beta ∈ (0°, 60°)` — open at both
ends, because either end collides adjacent servos. `beta = 30°` collapses to the
regular hexagon, which is a useful limiting case to test against rather than a
default to adopt.

### The rule

```
psi_i  =  theta_i  +  90  +  s_i·delta
n_i    =  (cos psi_i, sin psi_i, 0)
```

One scalar, `delta ∈ [0°, 180°)`, generating all six planes. The alternating
`s_i` is what makes the rule mirror-symmetric within each pair, matching the base.

With the in-plane basis fixed as

```
u_i  =  z × n_i          v_i  =  n_i × u_i  =  z
```

since `n_i` is horizontal.

### The 90° is forced, not chosen

I first wrote it as a convenient reference. It isn't a choice. Putting a general
constant in and imposing the mirror condition on a pair:

```
psi_i  =  theta_i + c + s_i·delta

psi_1  =  -beta + c - delta         mirror:  180 - psi_1  =  180 + beta - c + delta
psi_2  =  +beta + c + delta

equal  =>  c = 180 - c  =>  c = 90
```

Only `c = 90` survives, mod 180. `c = 270` is not a second option — it is the same
planes with every normal flipped, giving identical arm circles with `alpha`
relabelled as `180 − alpha`. That is the `delta ↔ delta + 180` redundancy already
accounted for by the half-open range.

**What it buys.** At `alpha = 0`, the tip is at `b_i + a·u_i`, and
`u_i · z = (z × n_i) · z = 0`, so the tip sits exactly at base-plate level. The arm
lies flat. That corresponds to a servo at mid-travel and gives an unambiguous
assembly datum — arms level, checkable by eye against the plate.

### Horizontal shafts, and what the choice pays for

Constraining every shaft axis to lie in the base plane is a decision, not a
consequence. The obvious benefit is manufacturing: servos bolt flat, no brackets.

The non-obvious benefit is that it removes a whole class of failure. Because
`n_i · z = 0`, the vertical direction lies *inside* every servo plane, so the
vertical component of the leg vector passes into the solution intact:

```
N  =  L_i · v_i  =  L_i · z  =  (q_i - b_i) · z  =  q_i · z
```

using `b_i · z = 0` since the shafts sit in the base plane. So `N` is simply the
height of anchor *i* above the base plate, and since

```
C  =  sqrt(M² + N²)  ≥  |N|
```

`C` cannot reach zero unless an anchor is sitting on the base plate itself. On any
machine where the platform stays above the base, the leg cannot lose authority
through `C → 0`. That is established by construction rather than by searching the
parameter space for degenerate values, and it means no region of `delta` needs
excluding.

Worth recording the dependency: this argument rests entirely on `n_i · z = 0`. If
canted shafts are ever revisited, it disappears and the degeneracy question comes
back open.

### Tuning delta

`delta` is chosen, not fixed by symmetry. The objective:

```
J(delta)  =  max over pose envelope, over i, of  |L_i · n_i|
delta*    =  argmin J on [0°, 180°)
```

`L_i · n_i` is the component of the leg vector perpendicular to the servo's plane
— the part the servo has no authority over. It is pure penalty: it never appears
in `C = √(M² + N²)`, but it does contribute to `|L_i|²` and therefore to `P`. So
out-of-plane offset inflates `|P|` while adding nothing to `C`, pushing legs
toward the unreachable condition `|P| > C`. Minimising the worst case across legs
and poses is minimising the tightest reachability constraint on the machine.

No closed form; a 1-D search. Checked against the quantity actually cared about —
the margin `C − |P|` — across the envelope, and the two track.

One structural consequence. `J` needs `L_i`, which needs the platform anchors. So
`delta` cannot be tuned once and frozen; it is an inner optimisation inside the
sweep, run per candidate geometry. Worth knowing before the sweep is written
rather than discovering it when the sweep is slow.

### A correction to the reachability condition

Working on this exposed an error in how I had been thinking about when a leg can
reach. The intuitive condition is the two-sphere one:

```
|d - a|  <  |L_i|  <  d + a
```

That is necessary but **not sufficient**, because the arm tip lies on a circle,
not a sphere. Decomposing the leg vector into in-plane and out-of-plane parts,
`|L_i|² = rho² + w²` with `w = L_i · n_i`, gives `M² + N² = rho²` — the in-plane
part only. The out-of-plane offset `w` appears in `P` but never in `C`.

Concretely, with `a = 20` and `d = 120`: an anchor at `M = 0`, `N = 0`,
`w = 110` has `|L_i| = 110`, comfortably inside `(100, 140)`, and is unreachable —
`C = 0` while `|P| = 1900`.

This closes the item that had been sitting marked *unverified* in the derivation
document: `|P| > C` does catch both failure directions, but the two-sphere bound
is not an equivalent statement of it and must not be substituted for it in code.

### Errors on the way

The first version of the tuning rule was indexed by leg — a `delta_i` per servo,
derived from the direction each leg pointed at home. That defeats the entire point
of the exercise: six numbers is not a parameterisation. The fix was to keep the
one-scalar family and choose the scalar by optimisation over the envelope, rather
than by a per-leg construction.

---

## 4 September

> **Skeleton only — not written up.** The entries below are the factual record of
> what was established, what was withdrawn, and the residuals. Every point marked
> `TODO(him): reasoning` is where this log's other entries carry an explanation of
> *why*, and that explanation is not written yet. Nothing here is in his voice, and
> nothing here should be read as his account of the session until he replaces it.
>
> Sources: `docs/session-handoff-2026-09-04.md`, `stewart-ik-derivation.md` §8/§8.1/§8.2,
> `stewart/diagnostics/zhome_datum.py`, `stewart/diagnostics/branch_check.py`.
>
> **Note on ordering.** Several results below reverse other results from the *same
> day*. Where that happens it is stated, with what was replaced. The reversals are
> not tidied away.

### Implemented

- `stage1`, `legs`, `arm_tips`, `w`, `ik`, `Unreachable` in `stewart/kinematics.py`.
  `h_p` threaded through `platform_ring` and `make_geometry`.
- §7 verification table re-established in `test_kinematics.py`, exit 0. Residuals:
  `stage1(R=I,T=0) == p` → 0; `Rz(90)` on `(10,0,0)` → 6.1e-16;
  `|arm_tips(0) - b| == a` → 5.6e-17; `arm_tips(0) == b + a·u` → 0; control row
  `b + a·n` also passes the distance check → 5.6e-17.
- `w()` repointed from an inlined `L_i` to `legs()`; regression over 4000 random
  poses bitwise identical.
- `TODO(him): reasoning` — why the control row was added, and what it shows that the
  distance check alone cannot.

### Established

- **Branch fixed as `-`** for all six legs, resting on `N_i > 0` under horizontal
  shafts. Closest approach to the branch-merge boundary over the provisional
  envelope: **−5.7e-3 of `C`**. Both supporting diagnostics — the 4365-pose sweep and
  the 720-step precession loop — ran at a **single** `z_home`, and that basis is now
  narrower than it was when the branch was fixed (see *Reopened*).
  `TODO(him): reasoning`
- **`z_flat` closed form** (derivation §8.1), with `g_i = q_i^{xy} - b_i`,
  `A = beta_p - beta`, `G = r_p e^{iA} - r_b`, `delta_G = arg G`:
  `|g|² = r_p² + r_b² - 2 r_p r_b cos A`,
  `g·u = r_b cos delta - r_p cos(A - delta)`,
  `(z_flat - h_p)² = d² - |g|² - a² - 2 a |g| cos(delta - delta_G)`, positive root.
  **Leg-independence is algebraic** — neither `|g|²` nor `g·u` contains `s_i` — and
  needs no D₃ argument. Verified against `make_geometry`, not a rebuilt ring, over
  540 grid points: leg-independence `max_i - min_i` ≤ **9.948e-14** (1.501e-15
  relative); closed form vs library geometry ≤ **1.637e-11** (2.103e-15 relative);
  `max|u_i·z| = 0` exactly. Round-trip confirmation at the derived height with
  `alpha = 0`, `|q_i - arm_tips(0)_i|` against `d`: worst residual **2.220e-16**.
  `TODO(him): reasoning` — including why the D₃ route was not taken.
- **`alpha` at home is one scalar shared by all six legs** (derivation §8.2). Follows
  from the two residuals above, not a new measurement. Two consequences recorded: all
  six servos read the same angle at home, a by-eye build check; and a non-zero home
  angle is absorbable by horn mounting angle. `TODO(him): reasoning`
- **Inner `delta` objective is the normalised reach margin** `(C_i - |P_i|)/C_i`,
  maximin over legs and envelope poses, superseding `J(delta) = max|w_i|`. The
  division by `C_i` is required for scale invariance. `delta` is absent from `L_i`,
  so `P_i` is `delta`-free and only `C_i` moves; at fixed leg and pose the margin is
  strictly decreasing in `|w_i|`. The two objectives differ **only in aggregation** —
  minimax over `|w_i|` against maximin over the margin. `TODO(him): reasoning`
- **Precompute feasibility bracket**, available because `P_i` is `delta`-free:
  `C_i ∈ [sqrt(|L_i|² - amp_i²), |L_i|]` with `amp_i = sqrt(A_i² + B_i²)`, giving two
  exact delta-free tests. Only the undecided middle pays 180 `delta` steps.
  `TODO(him): reasoning`
- **Compute is a wash, not a win** — correcting an overstatement made in session. The
  datum path was 273M full evaluations; this path is ~7.6M full plus ~1.4e9 cheap
  scan evaluations. Both near 10¹⁰ flops. 1.4e9 floats is ~11 GB, so the scan must
  chunk over candidates. `TODO(him): reasoning`
- **Symmetry structure of the six `w_i`** as the D₃ stabiliser of the pose, verified
  signed with a negative control. **Closed form for `delta*`**,
  `atan2(-r_p sin A, r_b - r_p cos A) mod 180`. `TODO(him): reasoning`
- **`beta_p = beta` is not a rank hole.** True rod lines `q_i - h_i` give full rank 6,
  `sigma_min` monotonic through `e = 0`, scaling linearly in `a` (log-log slope
  0.992). The rank-3 result was a concurrency artifact of the `q_i - b_i` proxy.
  `TODO(him): reasoning`
- **Tilt target does not scale with the kinematics** — `1/k` against translations'
  `k`, so absolute scale re-enters upstream of the sweep. `TODO(him): reasoning`

### Decided, reversing an earlier decision the same day

- **`z_home = z_flat` is dropped.** `z_flat` remains an **assembly datum only** and is
  no longer identified with the home pose; **`z_home` returns as an outer sweep
  axis**. This replaces the earlier 2026-09-04 decision that `z_home` was determined
  by the flat-arm datum. Propagated: the sweep is **six** normalised axes, not five —
  `beta`, `r_p/r_b`, `beta_p`, `a/r_b`, `d/r_b`, `z_home/r_b`; **15625** candidates at
  5 points per axis, not 3125; `delta ∈ [0°, 180°)` **restored as sufficient**;
  `z_home`'s range **undecided, with nothing currently supplying one**.
  `TODO(him): reasoning` — this is the entry that most needs it: what the datum bought,
  what it cost, and why the cost was judged the larger.

### Withdrawn

- **"8 of 432 grid combinations produce no valid `z_home`, all at `d/r_b = 0.8`, so
  the harness needs a feasibility guard."** An artifact of holding `delta = 40`. All
  8 are feasible on `[180°, 360°)`; **no geometry among them is infeasible**. The
  feasibility-guard requirement it created is struck in both places it appeared.
  `TODO(him): reasoning`
- **"`z_home` is determined, not free, and the sweep is five axes not six."** Per the
  reversal above. `TODO(him): reasoning`
- **`delta*` as an "exact seed … in the right basin."** Correct statement: `delta*`
  zeroes `w_i` at home — the pointwise quantity both candidate objectives are
  monotone in — **at one pose**. It is not the maximiser of the margin aggregate and
  not the minimiser of `J`. **A seed with no basin claim.** The open test is whether
  the margin's maximiser stays within a bracket of it: a question about aggregation,
  not about the function. `TODO(him): reasoning`

### Recorded so it is not reintroduced

- **The flat-arm datum lifts the gauge that makes `delta ∈ [0°, 180°)` sufficient.**
  §8 justifies the half-open range by `n → -n` giving the same planes, which it does
  — but `u = z × n` flips with it, so `alpha = 0` puts the arm on the opposite side.
  Same machine under a relabelling of `alpha`; **different assemblies under a
  flat-arm datum**. Measured with `z_home = z_flat` imposed: **3 of 432** combinations
  are feasible only on `[180°, 360°)`, all three at a simultaneous four-axis grid
  corner (`beta` min, `beta_p` max, `r_p/r_b` max, `d/r_b` min), **so the boundary
  lies outside the sampled region and its extent is unknown**.
- Also: **`delta*` reduced mod 180 returns to `delta_G`**, which is where
  `(z_flat - h_p)²` is **minimised** — under the datum the closed-form seed pointed at
  the tightest-feasibility `delta`.
- `TODO(him): reasoning` — why this is written down at all: the datum is
  algebraically tempting and will be proposed again.

### Reopened by the reversal

- The `-` branch evidence — the 4365-pose diagnostic and the 720-step precession loop
  — was gathered at **one** `z_home` and must be re-run across the restored axis. The
  branch conclusion is not withdrawn; its evidence base is narrower than it read.
- `N_i > 0` now sets the **lower bracket of the `z_home` axis**, not merely a
  docstring-to-test upgrade. `N_i` at home is `z_home - h_p`; tilt drops the low
  anchors by roughly `r_p sin(tilt)`.
- `z_home`'s range: lower from `N_i > 0`, upper from reach `|P| ≤ C`. `z_flat(delta)`
  remains the natural reference for setting the bracket even though home no longer
  sits on it. **Undecided.**
- Hardware pull gains **servo horn spline tooth count / mounting-angle resolution** —
  it decides whether a non-zero home angle costs anything.
- `TODO(him): reasoning`

### Provenance

- `branch_check.py` — the script behind "8 of 432" and the −5.7e-3 boundary margin —
  was **recovered from a session scratchpad, not from the repo**, and committed as
  `stewart/diagnostics/branch_check.py`. It had never been in git history. This is
  the **third** recorded instance of a documented result with no code behind it.
  **Recovered is not the same as never lost.** `TODO(him): reasoning`

---

## 5 September

> Skeleton only, same terms as the 4 September entries above: facts and
> residuals, no prose, nothing in his voice.
>
> Sources: `docs/notation.md` sec.9/10/12, `docs/session-handoff-2026-09-04.md`,
> and the four modules in `stewart/diagnostics/` named below.

**Envelope specified.** `dxy = 0`, `dz = 0`, `yaw = 0`, tilt limit **10.529 deg**.
Bang-bang recovery: `acc = 4 x0 / tau^2`, `sin(tilt) = 7 acc / (5 g)`,
`x0 = 50 mm` working `+ 30 mm` latency drift, `tau = 0.5 s`, `g = 9.80665`.
Bare requirement **6.558 deg** at `x0 = 50 mm`; the 3.971 deg difference is the
latency margin. Both recorded. `TODO(him): reasoning`

- `tau_L = 150 ms` is **provisional**, inherited from the 300 mm/s figure
  withdrawn 2026-09-03. It carries 30 of the 80 mm and 3.97 of the 10.53 deg.
  Needs sensor frame interval plus servo step response. `TODO(him): reasoning`
- Sensitivity: tilt goes as `1/tau^2`; a 10% error in `tau` moves required
  `sin(tilt)` by ~20%. `tau = 0.45` gives 13.038 deg, `tau = 0.60` gives 7.290.
  `TODO(him): reasoning`

**The `1/k` tilt-scaling result is inverted.** Under the arrest framing (fixed
`v`, `L` proportional to `k`) required tilt goes as `1/k`. Under the recovery
framing now adopted, `x0` scales with `k` at fixed `tau`, so `acc` scales with `k`
and **required tilt scales as `k`** - it *grows* with plate size. The conclusion
survives unchanged (absolute scale enters upstream, `r_b` fixed before the sweep);
the mechanism is the opposite sign. `TODO(him): reasoning`

**Envelope is two axes, not four.** `dxy = 0` removes `x` and `y`; what remains is
tilt magnitude and tilt azimuth. Being purely angular it carries no length
dimension and does not scale with `k` at all. `notation.md` sec.9's note about
translations forcing the envelope to scale is deleted. `TODO(him): reasoning`

**Azimuth window - a claim tested and half-refuted.** Claim: a 60-degree azimuth
window suffices by D3. Measured over the full circle at 0.25 deg, four geometries,
two aggregates, with a negative control (one anchor displaced 0.03 `r_b`):

| invariance | worst relative deviation | verdict |
|---|---|---|
| period 120 | 1.026e-13 | holds |
| mirror `psi -> 180 - psi` | 5.648e-14 | holds |
| mirror `psi -> -psi` | 2.303e-01 | **fails** |
| negative control, period 120 | 6.588e-01 | fails as intended |
| negative control, `180 - psi` | 3.736e-01 | fails as intended |

Width right, position wrong: the window is **`[30, 90]`**, not `[0, 60]`. Mirror
lines in azimuth sit at `30 + 60k`, because a reflection in the plane at azimuth
`m` maps a tilt axis at `psi` to one at `2m + 180 - psi`. `[0, 60]` is symmetric
about its own centre and misses the orbit `{75, 105}` entirely.
`stewart/diagnostics/azimuth_symmetry.py`. `TODO(him): reasoning`

**Pose grid.** 5 magnitudes x 7 azimuths, magnitude 0 counted once:
**29 poses, 174 `w` evaluations** per objective evaluation. Supersedes
`notation.md` sec.9's `3^6 = 729` / `4374` and 2026-09-04's 81 / 486.
`TODO(him): reasoning`

**`z_home` lower bracket, closed form.** `z_home > r_p sin(tilt) + h_p cos(tilt)`
`= 0.182733 r_p + 0.983163 h_p` at 10.529 deg. `delta`-free, `a`-free, `d`-free,
because `v_i = z` exactly and `b_i . z = 0` give `N_i = q_i . z`. Verified against
`make_geometry`: `min N_i` at the bound is 0 to **5.551e-17**; `max |v_i - z|`
**1.110e-16**. `TODO(him): reasoning`

- The back-of-envelope `z_home - h_p > r_p sin(tilt)` drops the `cos(tilt)` and
  overstates by `h_p(1 - cos tilt)` = **1.684e-3** at `h_p = 0.1 r_b`.
  Conservative, so it errs safe; not the bound. `TODO(him): reasoning`
- It is a **continuum** bound. A discrete pose grid reports it satisfied before it
  is - **+5.911e-4** on the 29-pose grid at `beta_p = 25`. The harness must take it
  from the formula. `TODO(him): reasoning`

**`z_home` upper bracket, per candidate, from `|P| <= C`.** No closed form. Over
540 candidates (5 `beta` x 4 `beta_p` x 3 `r_p/r_b` x 3 `a/r_b` x 3 `d/r_b`,
`h_p/r_b = 0.1`), `z_home/r_b` scanned 0.025..3.0 at 0.025, `delta` at 1 deg:

- **363 of 540** have a non-empty bracket; 177 empty; **0 non-contiguous**.
- lower ends `[0.225, 1.650]`, upper ends `[0.425, 1.875]`, widest 0.825.
- empty by `a/r_b`: **144 of 180** at `a/r_b = 0.10`, 33 of 180 at 0.20, 0 at 0.35.
- **`N_i > 0` set the lower end in 0 of 363** - but that is a survivorship sample.
  Attributing the 177 empties (2026-09-05): **176 reach-empty alone, 1 where the
  reach ceiling falls below the `N_i` floor** (`beta=10, beta_p=55, r_p/r_b=1.10,
  a/r_b=0.10, d/r_b=0.80`; ceiling `0.2920624` refined by bisection against a floor
  of `0.29932`, gap **`+7.26e-03`**, real and not a grid artifact). And **6**
  candidates have a non-contiguous *reach* set whose spurious low component
  (`z_home ~ 0.025-0.125 r_b`, platform on the base plate) sits entirely below the
  `N_i` floor in 6 of 6 - so `N_i > 0` is what keeps the feasible set an interval.
  Corrected claim: it does not shape the interior and is not why most candidates
  fail, but it is **load-bearing at the edges** and cannot be dropped.
`TODO(him): reasoning`

**`-` branch re-run across the restored `z_home` axis at 10.529 deg.** Fixture A,
`z_home` swept past its bracket both ways, precession over the full circle at
0.25 deg (1440 steps):

| check | result |
|---|---|
| `min(N_i) > 0`, every `z_home` | passes, worst `+0.896361 r_b` |
| envelope fully reachable | 8 of 15 `z_home`, `[1.2000, 1.2875] r_b` |
| branch reaching `alpha ~ 0` at home | `-`, at every feasible `z_home` |
| branch-flip step outliers | **0** |
| loop closure | `<= 1.78e-15` |
| `max abs(ik() - alpha_minus)` | **8.882e-16** |

`TODO(him): reasoning`

**The `-5.7e-3` boundary margin is superseded.** Reproduced exactly
(`-5.713988e-03`) on the old envelope at the datum `z_home = 1.223343`. On the
settled envelope, same geometry, same height: **`+1.550398e-01`**. Tuning `z_home`
alone at fixture A's untuned `delta = 40`: **`+2.276643e-01`** at
`z_home/r_b = 1.2375`. Individual costs, each a difference of two measurements on
one pose set: yaw `+/-10` **1.8033e-01**, translation `+/-0.05 r_b` **3.6486e-01**,
tilt 6 -> 10.529 **3.6147e-01**. The negative figure was bought by yaw and
translation, not by tilt. `TODO(him): reasoning`

- **WITHDRAWN 2026-09-05: the four-row attribution table.** The decomposition does
  not close - baseline `+5.148872e-01` minus tilt cost `3.614680e-01` gives
  `+1.534192e-01` against a settled figure of `+1.550398e-01`, a **1.62e-3**
  discrepancy, ~1% of the quantity the table existed to explain and three orders
  above every other residual recorded here. Cause confirmed: the rows and the
  settled figure are maximins over **different azimuth samples** (magnitude grids
  identical; 15 deg vs 10 deg), so subtracting one from the other was never valid.
  The conclusion is kept - it rests on gaps of `1e-1`, not on the arithmetic.
  `TODO(him): reasoning`
- **The 29-pose harness grid is grid-optimistic.** Same fixture, same height, by
  azimuth sampling: 15 deg `+1.534192e-01`, **10 deg (the settled figure)
  `+1.550398e-01`**, 0.25 deg reference `+1.531859e-01`. The harness grid
  **overstates the margin by ~1.9e-3**. Same failure mode as the `N_i > 0` bound.
  `TODO(him): reasoning`

**Closed:** handoff open items 1, 5, 8 and 11. `TODO(him): reasoning`

### `fk()` and the round-trip gate — the gate passes

> Same terms as above: facts and residuals, no prose, nothing in his voice.
> Source: `docs/cc-fk-gate.md`, `stewart/kinematics.py`,
> `stewart/diagnostics/roundtrip.py`.

**Gate result.** `pose -> ik -> six angles -> fk -> pose`, seeded **HOME**
(`R = I`, `T = (0,0,z_home)`) at every pose. **14 436 poses, 4 geometries, 3
grid levels.** Worst `|T_fk - T_cmd|` **1.853e-13 mm**; worst
`arccos((tr(R_cmd^T R_fk) - 1)/2)` **2.091e-06 deg**. **0** non-convergences,
**0** different-mode returns, **0** `ik` unreachable. `TODO(him): reasoning`

**Jacobian verified; the handed-over form was correct and was not changed.**
`df_i/dT = e_i^T`, `df_i/domega = -e_i^T [R p_i]_x`, left perturbation
`R -> exp([omega]_x) R`. Central differences, 348 samples, 5 step sizes:

| h (mm) | worst entrywise rel. | worst Frobenius |
|---|---|---|
| 5e-2 | 1.609e-05 | 1.076e-05 |
| 5e-3 | 1.609e-07 | 1.076e-07 |
| **5e-4** | **9.552e-08** | 1.078e-09 |
| 5e-5 | 1.568e-06 | 3.916e-11 |
| 5e-6 | 1.142e-05 | 3.869e-10 |

Bottom of the U at `h = 5e-4 mm`, an order above the textbook
`(eps*scale)^(1/3)`, because `|q-h|-d` cancels two ~120 mm quantities.
Negative control, one pose: sign-flipped left **2.000**, right perturbation
**4.307e-01**, right sign-flipped **1.974**, against **9.781e-10** for the
endorsed form — nine orders, so the test discriminates. `[R p]_x = R [p]_x R^T`,
so the right form differs by an `R^T` as well as a sign. `TODO(him): reasoning`

**The tolerance is an acceptance test, not a stopping rule.** First version got
this wrong. With `tol` as the stopping rule the required demonstration is
structurally impossible: ~40% of poses halt on the first iterate that crosses it
(48 of 117, fixture A at `1e-9`), so a max over a grid is taken over exactly
those, and worst `|dT|` tracks `tol` linearly — **9.7e-10, 1.3e-10, 1.7e-11,
1.8e-12, 1.3e-13 mm** at `tol = 1e-9 .. 1e-13`. `fk` now stops on **stagnation**
and applies `FK_TOL_MM = 1e-9 mm` **once**, to the converged residual. Worst
`|dT|` then **1.2681e-13 mm at every tolerance from `1e-8` to `1e-13`** — zero
movement across six decades. Acceptance fails at `1e-14` (234 poses), which is
below the measured arithmetic floor of **1.4e-14 to 7.1e-14 mm**.
`TODO(him): reasoning`

**Characteristic length not picked; sec.12 stays open.** All `cond` and
`sigma_min` quoted at `r_b` and flagged PROVISIONAL. Worst `cond` over the
coarse grid, by candidate length:

| fixture | `r_b` | `r_p` | `d` | `a` |
|---|---|---|---|---|
| A | 4.183 | 4.175 | 4.20 | 12.81 |
| C | 3.712 | 3.075 | 4.05 | 3.737 |
| E | 7.397 | 6.285 | 6.695 | 7.185 |
| F | 6.317 | 6.365 | 6.54 | 19.51 |

Spread a factor of 3 on F. LM damping is `lam*diag(J^T J)`, not `lam*I`, which
would add a millimetre to a radian and so need exactly that length.
`TODO(him): reasoning`

- Residual-to-pose-error conversion **checked per pose, not asserted**:
  `||dx|| <= sqrt(6)(max_i|f_i| + eta)/sigma_min`, worst measured/bound
  **0.074**. Three corrections were needed: maxima from different poses do not
  multiply into a bound; the bound is on `||f||_2` so `max_i|f_i|` needs
  `sqrt(6)`; and `eta = 8 eps (d + |T|)`, the round-off in *evaluating* a
  residual, must be added — at the floor it is the same size as the residual.
  Without `eta` the check reads **1.535**, i.e. violated. `TODO(him): reasoning`

**Assembly modes are real.** 0 different-mode returns from the home seed. Because
that is equally consistent with "the classifier never fires", the same six angles
were re-solved from **400 random seeds per fixture**: **8 distinct roots each,
32 in all**, every one at the same arithmetic floor (`1.42e-14 mm`) as the
commanded root — **the residual cannot separate them**, which is why the gate
classifies on pose distance. Of 28 non-commanded roots: **1** has all anchors
above the plate, **0** are inside the tilt envelope. The other 27 have
`min q_z < 0`, violating `N_i > 0`. `TODO(him): reasoning`

- A genuine nearby-mode case exists on `smoke_geometry` (`demo.py`'s "yaw +90"
  row): `|dT|` **0.3163 mm**, ang **0.7296 deg**, residual **1.421e-14**, and
  `ik` at the recovered pose returns the **same six angles to 2.0e-13 deg**. It
  sits at `cond` **9045**, `sigma_min` **1.94e-4** (at `r_b = 90`) against `cond`
  4–10 on the real fixtures. Near-singular geometry is where modes coalesce and
  a pose-distance classifier would call a second root a failure.
  `TODO(him): reasoning`

**Grids, and the refinement question.** Coarse/medium/fine at `(5,7,3)`,
`(9,13,5)`, `(17,25,9)` magnitudes x azimuths x `z_home`; coarse is
`envelope.py`'s own 29-pose harness grid. `z_home` sampled in the bracket
**interior**. Worst case **does move** under refinement — 1.47x, 1.14x, 1.29x,
1.46x, monotonic on 3 of 4 fixtures — so a coarse grid flattered it in the unsafe
direction a fourth time. But 40x the poses buying 1.5x in the max is sampling a
fixed round-off distribution at `1e-13 mm`, not finding a worst case. **Open item
12 is unaffected**: this gate has no dynamic range to bear on it.
`TODO(him): reasoning`

**Iteration counts, home seed.** Median **5** on all four fixtures, max **8**,
min 4; cap 100. **Home seed failed to converge at 0 poses** — no basin finding.
LM steps *accepted* 36 / 60 / 246 / 289 of 3600. Counting LM *attempts* instead
reported it on 96% of solves, because every converged solve ends with one
iteration where nothing reduces the residual — that is stagnation firing at the
floor, not a conditioning event, and it would have buried a real conditioning
problem. The home pose is excluded from the statistics and named: at tilt 0 the
seed **is** the commanded pose, so it is the "seeded at the truth proves nothing"
case, unavoidable because home is both the seed and a member of the envelope.
`TODO(him): reasoning`

**Fixtures.** A = `branch_check.py`'s fixture A verbatim, x100. Geometries run at
`r_b = 100 mm` so the tolerance can be a length in mm; the kinematics is
homogeneous of degree one and the envelope is purely angular, so this is a change
of units and `100 mm` remains the 2026-09-03 placeholder, not a decision.

| | β | β_p | δ | r_p | a | d | h_p | bracket (mm) |
|---|---|---|---|---|---|---|---|---|
| A | 20 | 40 | 40 | 85 | 20 | 120 | 10 | `[120.000, 129.000]` |
| C | 10 | 55 | 137 | 60 | 35 | 110 | 20 | `[57.500, 134.750]` |
| E | 50 | 25 | 30 | 60 | 35 | 80 | 10 | `[21.000, 101.500]` |
| F | 45 | 15 | 20 | 110 | 25 | 140 | 5 | `[128.750, 138.750]` |

`TODO(him): reasoning`

**Found on the way, not asked for.** `TODO(him): reasoning`

- **`azimuth_symmetry.py`'s fixtures B and D have empty `z_home` brackets at
  every `delta`** and cannot carry a round trip; E and F replace them in the
  gate, picked for the corners (E at `a/d = 0.44` against A's 0.17; F with
  `r_p > r_b` and the narrowest non-empty bracket found). Its `z_home` column is
  infeasible on **3 rows of 4** — A lists `0.95` against a bracket of
  `[1.2000, 1.2900] r_b`; only C's `0.90` is inside. Does not invalidate that
  module, which tests a symmetry of `w_i` and never calls `ik`.
- ~~**The `arccos` geodesic metric has a `~8.5e-7 deg` floor.**~~ **Fixed
  2026-09-07**, in both places — see the 7 September entry below.
- **`fk`'s stub docstring listed `Unreachable`**, which is an inverse condition —
  in forward kinematics the anchors are what is being solved for and no per-leg
  reach test exists. Replaced by `FKNotConverged(RuntimeError)`. `Unreachable` and
  `ik` are unchanged.
- `fk` returns **`(R, T)`, not `(T, R)`** as specified, because
  `stewart/roundtrip.py` is marked DONE and unpacks `R, T`. **Resolved
  2026-09-07 in favour of `(R, T)`** and recorded as a repo convention in
  `notation.md` — see the 7 September entry below.
- **`README.md`, `CLAUDE.md` and `demo.py` are stale**: the stub list is fully
  discharged, `CLAUDE.md`'s layout table still calls all five kinematics
  functions STUB, and `demo.py` prints "ik/fk stubbed -> every row reports 'not
  implemented'" when nothing is.
- **"Where Phase 0 stands" below is now stale on two points**: the branch rule it
  names as the immediate open question was fixed on 2026-09-04 and re-run on the
  settled envelope above, and the "numerical forward kinematics, a passing
  round-trip test" it lists as remaining are done. Left unedited — it is in his
  voice. `TODO(him): rewrite`

---

## 7 September

> Same terms: facts and residuals, no prose, nothing in his voice.
> Sources: `docs/cc-fk-gate.md` (revised), `docs/notation.md` Conventions and
> sec.12, `stewart/kinematics.py`, `stewart/diagnostics/roundtrip.py`.

Three corrections to the 5 September gate work. **The gate still passes**; none
of the three changes the verdict.

**Rotation metric fixed, not annotated.** The `arccos((tr - 1)/2)` geodesic form
was a spec error and is replaced by `|log_so3(R_cmd^T R_fk)|` — same quantity,
no floor. `arccos` near the identity: `tr(R) = 3 - theta^2 + O(theta^4)`, so the
trace carries the angle at **second order** and an `O(eps)` trace error becomes
`O(sqrt(eps))` in the angle. Floor `sqrt(2 eps)` = **1.21e-06 deg** predicted,
**2.41e-06 deg** measured. The antisymmetric part is **linear** in the angle and
has none. `TODO(him): reasoning`

- **Same quantity, measured not asserted.** Both forms compared against a known
  angle built by `exp_so3` about a random axis, so neither defines the answer:

| true angle (rad) | `\|log\|` rel | `arccos` rel | `\|log\|` abs (deg) | `arccos` abs (deg) |
|---|---|---|---|---|
| 3.11 | 3.19e-16 | 9.57e-15 | 5.68e-14 | 1.71e-12 |
| 1e-2 | 8.91e-15 | 4.59e-12 | 5.11e-15 | 2.63e-12 |
| 1e-4 | 4.87e-13 | 1.75e-07 | 2.79e-15 | 1.00e-09 |
| 1e-6 | 4.60e-11 | 1.07e-03 | 2.63e-15 | 6.11e-08 |
| 1e-8 | 6.97e-09 | 3.22e+00 | 3.99e-15 | 1.84e-06 |
| 1e-12 | 5.35e-05 | 4.22e+04 | 3.07e-15 | 2.42e-06 |
| 1e-15 | 6.23e-02 | 2.98e+07 | 3.57e-15 | 1.71e-06 |

  For angles `>= 1e-2 rad` the two differ from each other by `<= 4.6e-12`
  relative and each matches truth to the same — one quantity, not two.
  `|log_so3|`'s absolute error is `~3e-15 deg` throughout; its relative error
  grows below `1e-12 rad`, which is the **rotation matrix's** limit, not the
  formula's. `TODO(him): reasoning`
- **Effect on the gate:** worst rotation error was reported **2.091e-06 deg**,
  is actually **7.820e-14 deg** — `2.7e7x` lower. Worst `|dT|` unchanged at
  **1.853e-13 mm**. `TODO(him): reasoning`
- Fixed in **both** places: `stewart/kinematics.py` gains `log_so3` and
  `geodesic_angle`; `stewart/roundtrip.py`'s `_geodesic_deg` uses them. Leaving
  one copy floored while fixing the other would be worse than either. Checked:
  nothing else in the project uses the trace form — the remaining `arccos` calls
  are all `arccos(P/C)`, the IK branch. `TODO(him): reasoning`

**`(R, T)` pose order recorded as a repo convention.** Written into
`notation.md`'s Conventions block, so it stops being a per-function question.
Orientation first, everywhere: `stage1(geom, R, T)`, `legs`, `w`, `ik` all
already took it, and `fk(geom, alphas, R0, T0) -> (R, T)` matches rather than
departs. Mnemonic recorded with it: in `q_i = T + R p_i`, `R` is the operator
and `T` the offset, so `R` binds first even though it is written second. Note
sec.2's table lists `T` first — it is a glossary, not a signature.
`TODO(him): reasoning`

**The near-mode case is a SCORING finding, not an FK finding.** `smoke_geometry`,
`cond(J)` **9045**, `sigma_min` **1.94e-4**: a second assembly mode sits **0.32
mm** and **0.73 deg** from the commanded pose, residual **1.42e-14**, and `ik` at
the recovered pose returns **the same six angles to 2.0e-13 deg**.
`TODO(him): reasoning`

- The `ik` clause is what makes it a design problem, not a solver one. Two poses
  0.32 mm apart producing **identical servo commands** are two poses the machine
  cannot distinguish from its own commands — no control law, calibration or
  better solver separates them, because the information is not in the command.
  Which mode it assembles into is set by history, not by the command.
  `TODO(him): reasoning`
- **Nothing in the current feasibility set excludes it.** `|P_i| <= C_i` and
  `N_i > 0` both hold throughout on that geometry, so such a candidate would
  pass feasibility and reach ranking. The four real fixtures sit at `cond` 4–10
  with modes 30–260 mm apart, all below the plate or tilted 50–86 deg.
  `TODO(him): reasoning`
- Argues for `cond(J)` (or `sigma_min`) as a score **discriminator with a
  floor** — hard reject below a threshold, not a term to trade off, alongside
  the reach margin and the translation sensitivity. A weighted sum would let a
  candidate buy its way past a fold. `TODO(him): decision`
- **Promotes the characteristic length from provisional to blocking.** The
  threshold is a `cond` value; `cond` is undefined until the length is; so the
  length now decides where a **reject line** sits, not just how a ranking sorts.
  Measured spread over four candidate lengths on the gate fixtures: `r_b`
  3.7–7.4, `r_p` 3.1–6.4, `d` 4.1–6.7, `a` 3.7–19.5 — a factor of 3 on one
  fixture at benign values. Recorded in `notation.md` sec.12.
  `TODO(him): decision`
- **Open, and not settled by the above:** whether the right quantity is
  `cond(J)` of the FK Jacobian at all, or the wrench matrix's `sigma_min`, or
  mode separation measured directly. The FK Jacobian is what the gate happened
  to have in hand. `TODO(him): decision`

---

## Where Phase 0 stands

The inverse kinematics is derived and verified. The frame conversion, the leg
vector, and the forward arm-tip map are implemented and each tested against known
cases. The base ring and the servo rotation planes are parameterised — two
scalars for the ring, one for the planes, all three with stated ranges and a
symmetry argument behind them. Nothing has been purchased, no CAD drawn, and no
dimensions chosen — which is the intended state, since the dimensions are an
output of the analysis rather than an input to it.

My immediate open question is the branch rule. The closed form yields two valid
arm configurations per leg and the solver has to select one. The selection has to
be a function of the commanded pose alone: a rule that depends on what was
previously commanded would return different angles for the same pose depending on
history, which breaks the round-trip test and makes any design sweep
irreproducible. The failure mode I'm designing against is a platform following a
smooth trajectory while one leg's rule flips mid-motion, snapping that servo
through a large angle instantaneously — detectable as a step in one servo trace
while the other five stay smooth.

Beyond that, Phase 0 completes with a numerical forward kinematics, a passing
round-trip test, a formal degree-of-freedom check, and the design-objective work
that turns the geometry sweep from an open search into a sorted one: a justified
tilt target with margin above the 1.635° floor, a scoring function, servo
candidates whose travel and deadband feed the resolution analysis, and the anchor
parameterisation. The sweep then selects the dimensions, and only then does
anything get ordered.
