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

## 4–10 September

### The reach-margin sweep

This stretch was the Phase 0 plan carried through to the end. The kinematics
got finished and verified: `ik` and its pieces went into
`stewart/kinematics.py`, the IK branch was fixed for all six legs (it holds
because the shafts are horizontal), and `fk` passed a round trip of
`pose -> ik -> fk -> pose` over 14,436 poses with a worst position error of
1.9e-13 mm. That code is still what the platform runs on.

The rest was the sweep. I specified the envelope as tilt only, up to 10.529°
(50 mm of recovery in 0.5 s, plus 30 mm of drift during latency), and settled
the score as the worst-case normalised reach margin: how far the most
stretched leg is from being unable to reach, over that envelope. On
9 September it ran on 368,000 candidates, 89% of them feasible, and returned
`a = 60.4 mm`, `d = 126 mm`, `beta = 5°`, `beta_p = 52.5°`.

The warning signs were already in the record. When I opened the sampled box
on 8 September, the optimum moved further onto its walls instead of coming
off them, and by 9 September three axes — `r_p`, `beta` and `a` — had their
limits set by hardware or by a decision, never by the objective. `a = 60.4`
was simply the longest arm in the catalogue. Reach margin measures distance
from unreachability, and a mechanism can score well on it while barely being
able to move. That is where 13 September picks up.

The day-by-day record, with every residual and the decisions that reversed
each other, is in `archive/docs/design-log-phase0-sweep.md`.

---

## 13 September

> Numbers marked ESTIMATE have not been checked against the kinematics.

### I was optimising the wrong thing

The sweep ranked geometries by reach margin — how far the platform is from a
leg being unable to reach. Balancing a ball doesn't depend on that. It depends
on **tilt resolution** (how finely the plate can be set), **bandwidth** (how
fast it responds) and **repeatability** (whether the same command gives the
same tilt), and the sweep measured none of them.

The result gave it away. The winning `a = 60.4 mm` sat at the top of an
aftermarket horn catalogue, because the objective kept asking for more arm until
the catalogue ran out. A longer arm gives more reach and *worse* resolution, so
reach margin was pushing the design the wrong way.

### Two decisions

**Use the servo's stock horn.** No printed or bought arm. The servo is the
TowerPro MG90S; `a` is now whichever hole on its own horn works, a handful of
values instead of a continuous axis.

**Pick the easiest design to build that passes, and stop optimising.** The
goal is a ball balanced on the plate. Once a design clears every requirement,
extra margin buys nothing and a harder build costs real time. So the design
question turned into a set of pass/fail requirements and a search through
simple-to-build candidates for one that passes.

### Requirements, worked back from the ball

The link from the plate to the ball is `(5/7) g sin θ` for a solid ball
rolling without slip: **1° of tilt accelerates the ball at 0.122 m/s²**.

- **Range ≥ 7°.** Phase 0's recovery derivation (50 mm back in 0.5 s,
  bang-bang) gives 6.558°, rounded up.
- **Precision ≤ 0.3° total.** A tilt error the loop can't remove, held for
  `τ = 0.5 s`, drifts the ball `½ (5/7) g sin δ τ²`: 1.5 mm at 0.1°, 4.6 mm at
  0.3°, 15.3 mm at 1°. For a ±5 mm hold tolerance, `δ ≤ 0.327°`. Resolution
  and repeatability share this one budget:
  `G × (deadband + backlash) + joint play / r_p ≤ 0.3°`, where `G` is degrees
  of tilt per degree of servo. Camera feedback removes a steady offset but not
  random play, which is why they add.
- **Speed ≥ 140°/s.** Swing +7° to −7° in 20% of `τ`.
- **Latency ≤ 50 ms**, 10% of `τ`, a rule of thumb.
- **Torque**: a sanity check, not expected to bind with a 2.7 g ball.

The ±5 mm tolerance, the 20% slew fraction and the 10% latency fraction are
proposed, not yet confirmed.

### What that says about the MG90S (ESTIMATE)

With `G ≈ a / r_p` and published figures (5 µs deadband ≈ 0.45°, 600°/s
unloaded), range needs `G ≥ 0.16` and speed needs `G ≥ 0.23`. Precision caps
`G` at 0.51 with no backlash, 0.24 at 0.5° backlash, and 0.16 at 1°. **Gear
backlash decides whether any geometry passes**: at 1° the window is empty. It
isn't published, so the first bench test measures it.

Range and resolution pull against each other through `G`, as the Phase 0
appendix already noted: geometry doesn't change how many distinguishable tilt
steps there are, only how they are spent. Speed pulls the other way. With the
horn length fixed, the free variable is `r_p`: a ~15 mm hole at `r_p = 80 mm`
gives `G ≈ 0.19`, likely too slow, so the 80 mm hub is reopened.

### Tidied

Nine overlapping documents became three plus `STATUS.md`. The sweep, its 18
diagnostic scripts (~15,000 lines) and its outputs moved to `archive/`. The
kinematics library (~1,700 lines, tests passing) is what carries forward.

Next: measure the stock horn and servo footprint while waiting for the battery
pack, then bench-test backlash, deadband, degrees per µs and loaded speed.

---

## 15 September

**Build in two stages.** Stage 1 is the platform moved by a joystick; stage 2
adds a camera and closes the loop to balance the ball. The camera is too far
ahead of everything else to plan around yet.

The hardware is still sized to the full requirements. A joystick doesn't need
0.3° precision or 140°/s, but the horn hole and `r_p` set `G`, and `G` decides
whether stage 2 can pass without a rebuild. Only the parts that need a camera
wait for stage 2: the latency requirement, its 10% rule of thumb, and the
camera noise check on the ±5 mm tolerance.

### Slew fraction vs tilt range (UNVERIFIED quick calculation)

R1's 6.558° assumes the plate reverses tilt instantly. If it takes `k τ` to
swing across, the ball spends less time at full tilt, so reaching 50 mm in
0.5 s needs more tilt:

| `k` | tilt needed |
|---|---|
| 0 | 6.56° |
| 0.05 | 6.90° |
| 0.1 | 7.29° |
| 0.2 | 8.20° |
| 0.3 | 9.38° |

A 7° range only allows `k ≈ 0.08`, well short of the proposed 0.2. Still open:
raise R1 to about 8.5°, or ask for a faster swing.

### Servo bench-test sketch

`firmware/servo_test/servo_test.ino` drives one MG90S in microseconds from the
Serial Monitor, so the bench tests don't need a new upload per test. It uses
`writeMicroseconds()` because `Servo.write()` moves in ~11 µs steps, coarser
than the 5 µs deadband being measured. A `jump` command flashes the pin 13 LED
at the instant of the command, so 240 fps video can time the step from that
frame. Limits default to 900–2100 µs until `wide` is sent, so a typo can't
drive the horn into its end stop. Compiles for an Uno; not yet run on hardware.

---

## 16 September

Bench-tested one MG90S with the pointer rig and the serial sketch.

| parameter | measured |
|---|---|
| horn holes | 7, from 4.15 to 15.95 mm, in 1.967 mm steps |
| degrees per µs | 0.087 (133° / 90° / 46° at 1000 / 1500 / 2000 µs) |
| deadband | 4 µs = 0.35° |
| gear backlash, free | 1–2° |
| speed | 286°/s no load, 250 at 150 g·cm, 207 at 382, 162 at 556 |
| travel | 530–2480 µs = 169.7°, centred on 1505 µs |

The horn is linear to about ±2% across the middle of its range, which is a
relief: `a` can be treated as a fixed lever.

**Decision: R1 = 8.5°, `k` = 0.2 (option A).** The 15 September calculation
said a 0.2 slew fraction needs 8.20°, so 7° and 0.2 were never compatible.
Raising the range is the cheaper side to give: it costs servo travel, of which
there is plenty (85° per side), while tightening `k` would have demanded a
faster swing than the servo has. R3 follows: 17° in 0.1 s, **170°/s**.

### What the backlash does to the design

Precision needs `G (deadband + backlash) + play / r_p ≤ 0.3°` and speed needs
`G ω ≥ 170°/s`. Both contain `G`, so eliminating `r_p` gives the fastest tilt
rate that still holds precision, for *any* geometry:

    rate_max = ω δ / (deadband + backlash + play / a)

At `ω = 250°/s`, `a = 15.95 mm` and 0.1 mm of joint play, that is 44°/s at 1°
of backlash and 28°/s at 2°. R3 asks for 170. **The design is 4–6× short, and
no choice of `r_p` changes it** — `r_p` trades speed for precision one for one,
which is why it cancels out of the formula above. Even with zero backlash it
is 1.6× short: the 0.35° deadband and the joint play nearly spend the whole
0.3° budget on their own.

Phase 0's appendix said geometry doesn't change how many distinguishable tilt
steps exist, only how they are spent. This is the same result with numbers
in it, and they come out too small.

### Preload rescues it

Re-measured with ~100 g hanging from the horn so the load never reverses:
**backlash is negligible**. The 1–2° is free play in an unloaded gear train,
and the real platform never unloads it — its own weight holds every servo
against one flank, which wins back the 3.9× for free.

What binds now is `deadband + joint play / a`, and at 0.1 mm the ball joints
contribute 0.36° — as much as the whole deadband. So the biggest unknown is
now which joints I buy, and the servo bench can't tell me that.

Passing the inequality also doesn't make a design buildable. R3 goes as `1/τ³` and sets a floor on `G`, which caps `r_p = a / G`.
Holding the 0.5 s recovery forces an anchor circle of about 22 mm — a plate
balanced on a stub. Letting `τ` out to 0.7 s drops R1 to 4.2° and R3 to 60°/s,
and `r_p` lands at 42–67 mm, which is a real platform. The fast spec could
only ever have been met by a miniature.

### The horn rule falls

The M3 rod ends I found need a 3 mm bolt; the stock horn's holes are 1.3 mm and
its arm is ~4 mm wide, so drilling one out would leave almost no material. The
obvious fix, an aluminium horn, turns out not to exist for this servo: micro
metal horns are cut for 21T/23T/25T splines, and sources can't even agree
whether the MG90S is 20T or 21T (I counted 20). Ordering one would be a bet on
a fit I'd only find out about on arrival, and a loose spline is a bad place to
lose precision.

Printing the whole horn is worse — 20 teeth on a 4.8 mm shaft is a 0.75 mm
pitch, finer than FDM holds.

So: **keep the moulded spline, print an arm that clamps to it.** Two plates
sandwich the stock horn, two M2 bolts pass through holes that already exist,
and the printed arm carries the M3 rod-end bolt at `a ≈ 22 mm`. The longer arm
helps too: the joint-play term is `play / a`, so it *shrinks* as the arm grows,
and `r_p` moves to a comfortable 58–70 mm.

It does add one new error source, the flex of a printed part under reversing
load. The R2 budget leaves 0.047° for it, which at 22 mm is 54 µm of hysteresis
at the tip. That's the number the part has to hold, and it's why it's a fat
two-bolt sandwich rather than a tab.

**Chosen: `τ` = 0.7 s, `e` = ±8 mm.** The requirements that follow are R1 ≥
4.5°, R2 ≤ 0.25°, R3 ≥ 65°/s, R4 ≤ 70 ms, and ball joints with no more than
0.21 mm of total play per leg. `r_p` lands between 42 and 67 mm. Giving up
0.2 s of recovery time and 3 mm of hold tolerance bought a platform three times
the size and joints I can actually buy.

What can still move: gravity preloads every servo one way on the real platform,
so the free play I measured by rocking the horn may never appear in service —
worth up to 3.9× for free, and the next test. 286°/s against a published 600
suggests the supply voltage or my frame counting is costing another 2×. After
that, the requirements themselves: recovery time `τ` and hold tolerance `e`
each buy their factor linearly.

### The evaluator, and what it says

Wrote `stewart/performance.py`: R1, R2, R3 and a rod-end cone check, every
sensitivity a numerical derivative of `ik` or `fk` instead of the `G ~ a/r_p`
approximation. 540 candidates in under three seconds.

The leading design is `r_b = 90, beta = 5, r_p = 70, beta_p = 35, a = 22,
d = 70`, sitting 60 mm high and using only ±14.6° of the ±83° of servo travel
available. R1, R3 and the joint cone pass with room. R2 is the open one:
0.287° worst case against a 0.25° budget, or 0.117° if the six legs' errors are
treated as independent rather than conspiring. Joint play is 60% of that error,
so tomorrow's measurement decides it — 0.06 mm passes outright, 0.1 mm passes
only on the generous reading. No geometry in the sweep gets around this, so
it comes down to the joints.

**Decided: judge R2 in quadrature.** Summing the six legs assumes their
independent errors all point the same way at the same instant; they don't, and
the inputs are already pessimistic (half a deadband plus the full per-leg play,
both ends counted). The penalty for being wrong is bounded and small — ±8.6 mm
of ball wander instead of ±8 mm — while the strict reading would have demanded
0.06 mm joints, which means waiting on a second order. The worst-case number
stays in the report as the reserve I'd spend if the joints wear or a servo
unloads mid-slew.

With that, the leading candidate passes every check, and the joint spec relaxes
from 0.06 mm to about 0.28 mm — inside what ordinary M3 rod ends manage.

Two things the evaluator caught that I had wrong:

**A longer arm is not always better.** I had argued `play / a` shrinks with a
longer arm, which is true only if `r_p` grows to keep `G` fixed. With `r_p`
capped by the print bed, `a = 25` comes out *worse* than `a = 22` (0.312 vs
0.287), because the extra gearing amplifies the deadband faster than it dilutes
the play.

**The rod-end bolt at the arm must not be parallel to the servo shaft.** The
rods lean about 31° out of the servo plane, so a shaft-parallel bolt sits 31°
off perpendicular and the joint binds — against a cone of maybe 13°. Tilt the
bolt ~30° toward the direction of arm rotation and the worst misalignment over
the whole envelope drops to 4.2°. That's a hole angle in the printed horn
extension, and without the check I'd have drawn it parallel.

## 18 September

Labelled the bench-tested unit **servo 1** and started a per-servo calibration
table in `STATUS.md`. Each leg carries its own centre and deg/µs into the
firmware trim, so the six have to be told apart — tape on the case before
anything is assembled, because afterwards they are identical.

Found the travel ends of all six. They agree closely: low ends 510–530 µs,
high ends 2480–2490, centres 1495–1505. The range every unit reaches is
530–2480 µs, so a common 550–2460 µs (±83° about 1500) drives the whole set and
per-leg trim mops up the rest. That confirms the `travel_deg = 83` the
evaluator had been assuming from servo 1 alone. R1 only needs ±14.6° of it, so
travel is nowhere near the limit; precision is what's tight.

Also added a clearance check — rod against rod, rod against every servo case —
because nothing in the project modelled interference, and a platform that
passes on paper can still bind on the bench. At `r_b = 80` the closest
approaches are 10.2 mm rod-to-rod and 7.3 mm rod-to-neighbouring-case, which is
comfortable. It also settled the base radius: `r_b`
turns out not to matter to R3 or the joint cone at all, so 90 mm — which would
have put the shafts on a circle as wide as the print bed — buys only 0.018° of
precision over 80 mm. Taking 80.

Then the horn extension came off the CAD at 32.3 × 12 × 4.95 mm, and modelling
it instead of a bare rod changed the design. The tight part is the horn: at
`beta = 5` it passes 2.7 mm from the neighbouring servo case, and at `beta = 2`
it collides outright. `beta` — the angular spacing within a servo pair — sets
all of it. Opening it to 10° costs 0.002° of R2 and nothing in speed or the
joint cone, and roughly doubles every clearance in the machine.

**The design to build: `r_b = 80, beta = 10, delta = 0, r_p = 70,
beta_p = 35, a = 22, d = 70`**, sitting 63.2 mm high.

Also wrote down something that had been assumed silently since Phase 0: **the
servo shafts must sit parallel to the base plane.** The IK picks one branch for
all six legs, and that choice is only valid because horizontal shafts make
`v_i = z`, so `N_i` is the anchor height and always positive. Cant the shafts
and the branch reopens — the maths stops describing the machine. A cant of 1°
costs 0.10 mm of out-of-plane tip error, systematic rather than random, so it's
a target for the mounting jig rather than a hard limit. It now sits in
`STATUS.md` next to the design, and as B1 in the bracket spec
(`docs/hardware.md` §13), which I wrote ahead of the FEA so the thresholds come
from the error budget instead of from whatever the CAD happens to give.

The 16 September candidate would have been built with 2.7 mm between a
printed part and a servo case, because the clearance check modelled the rods as
thin lines and the horn extension hadn't been drawn yet.

Before starting the base plate I checked what it depends on, and found the
clearance model had the servo lying the wrong way. `Body` treated the case as
12.3 mm along the shaft, centred on it. The real MG90S runs 35.3 mm back along
the shaft from the horn, which is how it sits in the mounting I'm copying. Two
consequences:

- **In each pair, the horns face each other and the cases point away.** The
  pair's shafts are only 27.8 mm apart, so cases pointing inward overlap. The
  kinematics don't care which way a servo faces along its shaft line, but the
  build does.
- The clearances got *better*: horn to neighbouring case went from 13.6 to
  25.1 mm, rod to case from 16.4 to 29.3 mm. Opening `beta` to 10° was chosen
  on the wrong model, since `beta = 5` would have cleared too. It costs nothing,
  so it stays.

Two other things for the plate. With the cases pointing outward, the six of
them fill a 177–180 mm square, the whole print bed, so the plate gets cut from
sheet or made in pieces rather than printed in one go. And the joint-play
budget I'd been quoting (0.28 mm) belonged to the old `r_b = 90` candidate. At
80 it is 0.24 mm, so the rod-end play still decides whether this base radius
stands.

Asked myself how each number was justified, and `r_p = 70, beta_p = 35,
d = 70, delta = 0` had the weakest paper trail: they came out of the
16 September search, whose grid I never saved. So I checked them one at a
time instead. Every value of `r_p` from 55 to 80, `beta_p` from 20 to 50, `d`
from 55 to 90 and `delta` from 0 to 40 passes. `d` doesn't touch R1–R3 at all
and only sets the height. The one edge nearby is the arm: `a = 18` fails R3.
So the design sits in a flat region where everything nearby passes, which is
what I asked for on 13 September.

`delta` was the exception. Its performance numbers pass everywhere, and R2
even improves a little as it grows, but the clearance check fails it. Yaw the servos one way and each pair's horns swing into
each other: 5.0 mm apart at 10°, 0.5 mm at 20°. Yaw them the other way and
each horn swings into its partner's case. `delta = 0` was picked because it's
the simplest layout, every servo square to its radius, and it turns out to be
the only one with room.

Measured the MG90S case properly with calipers. The shaft isn't centred in the
body: it's 16 mm from one end and 6.75 from the other. So with the case hanging
below, the lower tab reaches 20.7 mm under the shaft centre, and at a 30 mm
shaft height it clears the plate by 9 mm. From the gear-boss top to the case
bottom is 28.65 mm: a 6 mm gear boss, then 22.55 mm of main case, with the
2.75 mm tabs sitting 1.8 mm below the case top. The stack closes to 0.1 mm
against the overall measurement.

The printed arm came out at **a = 22.5 mm**, not 22. It still passes
everything (R2 0.140°, R3 80.7°/s), and the joint-play limit tightens slightly
to 0.23 mm. The design now uses 22.5, because the kinematics have to describe
the part that exists.

The arm CAD has the M3 hole straight through, parallel to the shaft. That
would put the rod end 25° out of line and bind it. Tilting the hole 25° across
the arm's width brings the misalignment down to 0.3°, and anything from 20 to
30° is fine. Because each pair's horns face each other, the tilts come out
mirrored: legs 1, 3, 5 need one hand and 2, 4, 6 the other, so it's one body
mirrored in CAD, three of each.

Sketched the servo casing as a C: a wall with a window the servo drops through,
a top arm and a foot. The tabs screw to the wall, and the foot screws up from
under the base plate so no screw heads show, which also keeps the plate clear
under the arms. The C has to open toward the case, because the horn is on the
tab side of the wall. Kept symmetric, one bracket fits all six. The thing to
watch is the tab screws: the slots sit only ~2.5 mm past the window edge,
which is too little wall for an M2 heat-set insert.

Finished the bracket after several test prints to dial in the fit around the
printer's tolerances. The tabs self-tap: 1.85 mm holes for the servo's own
2.06 mm screws, so there are no inserts. The servo sits on a seat 14 mm up,
which puts the shaft at 30. A notch at the top of the window lets the wires
through. Two diagonal lugs at the base take the screws into the plate. Against
the rods it has at least 10 mm everywhere. The lugs push the base footprint out
to about 205 mm across, so the plate has to come from sheet. The top tab screw
breaks into the wire notch. I'm accepting that because the screw still bites
on three sides and the load on it is small. The lugs take M3 bolts and nuts
through the plate.

The rod ends arrived. There's play I can feel but can't measure without a dial
indicator. It probably doesn't matter, for the same reason the servo backlash
didn't: the platform's weight keeps every rod in compression, at 18–19% of the
weight each in every pose, so each ball stays pressed against one side of its
housing. Even a full-speed slew only accelerates the anchors at about 0.2 g,
nowhere near enough to unload them. The bind angle is about 22°, against 4.2°
needed. To get 70 mm centre to centre, the M3 rod is 54.1 mm and bottoms out in
both ends.

Bolted straight onto the arm, the rod end's housing hit the arm face long
before its own 22° limit, because the ball only stands 0.9 mm proud of the
housing. The tilted hole makes it worse, since the arm face rises toward the
housing on one side. The fix is a seat: a small boss square to the bolt that
lifts the ball clear. 1.4 mm just lets it sit, and 2.8 mm frees the full 22°.
The arm end only ever sees 0.3° of misalignment, so I'm using 3 mm for margin,
not because the joint needs it.

In the end I didn't build the tilted version. It needed a seat, a side lobe so
the hole stayed inside the part, a self-tapped bolt instead of the captive
nut, and two mirror-image arms. I went at it from the other side: spacing the
ball away from the arm. A nut as a spacer got the joint to 15°, not enough for
the 25° a straight bolt needs. M3 threaded inserts on both sides of the ball
got it to about 30°. So the arm keeps its straight hole, one part fits all six
legs, and the top-plate geometry doesn't have to change. (Keeping a straight
bolt at 15° would have meant pulling the top-plate joints toward their
servos, `r_p 60, beta_p 22`, which roughly halves the platform's sideways
stiffness.)

The last measurement for the base plate: with the servo fitted, the arm's outer
face sits 16.0 mm from the bracket's window face. The rod-end ball is 6.21 mm
further out, so each bracket sits 22.21 mm behind the plane the kinematics call
the base. That turned into twelve M3 hole positions, and the plate needs to be
about 210 mm across. The arms sit further back than I'd been modelling, so
every clearance got bigger: arm to arm went from 13.7 mm to 30.8.

End-of-day sweep. One real gap turned up: the evaluator was still grading the
rod-end angle with the ideal bolt direction and reporting 4.2°. The arm I'm
building has a straight bolt, where the joint sees 25.1°. It now checks what's
built by default (`Requirements.base_bolt`), and 25.1° passes against the ~30°
the insert spacers allow. `demo.py` had also fallen behind. It still ran on the
deliberately wrong smoke geometry and described the kinematics as stubs. It now
runs the design to build, and ik→fk round-trips exactly.

**Shaft height: 28 mm** above the base-plate top, the same for all six. The
kinematics don't see it (everything is measured from the shaft plane), so
it's purely a packaging choice. The horn swinging to an end stop reaches
24.9 mm below the shaft, and the case about 22 mm. Going higher only makes
the bracket wall a longer cantilever, and its flex grows with the cube of
its height, so I took the lowest height that clears with a few millimetres
spare.

Then measured the printed arm itself: its lowest point is **26.5 mm** from the
shaft axis, not the 24.9 the model had guessed. At the ±83° end stop that
leaves 1.65 mm between the arm and the plate at 28 mm, which is enough for a
bare plate and not enough for a screw head. **Shaft height is now 30 mm**:
3.65 mm of clearance, room for M3 pan heads. The bracket flexes about 23% more
at that height, which a thicker wall wins back. The platform's ball centres
sit 93.2 mm above the plate at home.

The arm extension isn't pinned and glued as the spec first imagined. I pause
the print partway, drop the stock horn into a pocket and print over it, so the
horn ends up fully enclosed. It fits with zero wiggle, it needs no drilling,
pins or epoxy, and the moulded spline still does the indexing.

Before the bracket FEA I worked out the load case from the kinematics instead
of trusting the "~1 N, ~40% out of plane" I'd written down. The estimate held
up. With a 250 g platform every rod carries 0.44–0.46 N of compression across
the whole tilt envelope, and 0.35–0.56 N during a full-speed slew, so no rod
ever goes into tension. At the arm tip the force is 90% tangential and 42%
along the shaft, and that split hardly changes with pose.

I hadn't pictured where that force lands on the bracket. The ball
sits 22.21 mm in front of the window face, so the rod force pries the servo
off the wall, and the upper tab screw ends up pulling out with about 1 N for
every newton on the rod. A hand calc with the servo as a rigid block hinged at
the lower screw and the upper screw held by the two 4 × 4 mm posts beside the
window puts the ball at **67 µm along the rod per newton** in eSUN PLA+
(E ≈ 1.9 GPa, softer than plain PLA). B4 allows 60, so on paper the bracket
fails.

It fails only at B4's 1 N, though, and the rods never carry that: 0.46 N
holding, 0.56 N at the peak of a fast tilt, which gives 31–37 µm. Even if I
count every micron of it as random error, R2 comes out at 0.176°, or 0.204° at
the full 1 N, against 0.25°. And most of it isn't random: the rod force varies
by 5% across the envelope, so the bracket sits at nearly the same deflection
all the time, and trim and the camera loop remove a fixed offset.

**So the 4 mm brackets I've already printed stay, with no FEA and no bench
test.** An FEA would mostly model the parts I'm least sure of, the lugs and the
screw threads, with guesses. If tilt precision comes up short once it's
assembled, they're the first thing to check. The load case and the numbers
are in `docs/hardware.md` §13.

Next, the top-plate end of the rods. The evaluator had only ever reported the
ideal bolt axis there (4.2°), never one I'd actually build. The simplest mount
turns out to match it: a **horizontal bolt parallel to the line from
the plate centre to the middle of the pair** gives the same 4.2°. The true
optimum is just 2.1° off that line and 0.7° off horizontal. So both mounts in
a pair face the same way, one part does all six, and at 4.2° the rod end can
bolt flat against it without the insert spacers the arm end needs. The other
obvious choices are worse: a vertical bolt binds at 69°, and a bolt along the
anchor's own radius costs 18°. The spec is `docs/hardware.md` §14.

Turning that into a part turned up one more constraint. The rod end's shank
starts 4.95 mm from the ball centre, and the rod heads steeply downward, so
a tab with a rounded end wider than about 4 mm in radius runs into it. At
4.5 mm it clears by 0.2 mm, at 4.0 by about 0.7. A 1 mm boss between the tab
and the ball holds the housing off the tab too.

## 19 September

The base plate came out 223 mm corner to corner once it cleared every lug by
5 mm, too big for the 180 mm bed. Rather than wait for acrylic, I'm printing
it as three identical wedges split along the gaps between the servo pairs, so
each wedge carries one pair of brackets whole and only the empty gaps get
glued. If the acrylic arrives, it replaces the print.

Settled the control path for stage 1: **IK runs on the PC**, and the Arduino
only turns six pulse widths into `writeMicroseconds()` calls. The kinematics
are already written and tested in Python, porting them would mean testing
them all over again, and stage 2's camera needs a PC anyway. The servos run
off **4× AA**, with the ground shared with the Arduino.

## 21 September

The blade came and all six rods are cut to 54.1 mm, so they bottom out in both
rod ends at 70 mm centre to centre. All six brackets are printed and the
servos are screwed down to the base. Everything for stage 1 is now on the
bench except the control software.

Home on this design is the arms flat: `alpha = 0` on every leg, arm level and
pointing inward, so the rod-end balls sit at the shaft height. That makes
zeroing a spirit-level job. Each servo goes to its centre pulse, the arm goes
on at the spline tooth closest to level (18° per tooth, so at most 9° off),
and the rest is trimmed in microseconds and written down per leg, before any
rod goes on.

## 22 September

Zeroed all six arms with a new calibration sketch (`firmware/servo_cal/`):
every servo held at 1500 µs, arm pressed on at the tooth nearest level, then
trimmed in microseconds until level and saved to the Arduino's EEPROM. The
zeros came out at 1550, 1500, 1410, 1590, 1560 and 1470 µs, so the biggest
correction was 90 µs, about 8°, inside the 9° a 20-tooth spline can leave.
The direction check came out the way the geometry says it should: in every
pair one servo raises its arm on +µs and the other lowers it, because the
two face each other.

The base plate ended up as **laser-cut 5 mm acrylic** after all, not the
three printed wedges: one flat piece, no glue seams, and the twelve bracket
holes cut straight from the layout. The servos are mounted on it. The top
plate is **3D-printed in PLA**, with the six rod-end anchors from
`docs/hardware.md` §14.

With the zeros saved I put it together — arms held at home, rods on, top
plate on with its notch over servos 1 and 2 — so **the platform is
assembled.**

At home the plate sits level, 105 mm from the top of the base plate to the
top of the platform, and nothing binds.

Then the stage 1 control code: the Arduino streams the joystick and outputs
six pulses, the PC turns the stick into a tilt, runs the IK and sends the
pulses back. Checking it before the first run caught a real bug. The
docstring of `tilt_pose` said the plate's *high* side faces the azimuth; the
maths actually lowers that side. Everything before this swept every
azimuth, so no result depended on which way round it was, but the joystick
would have tilted the plate away from the stick. I only found it by pushing
the pulses back through the forward kinematics and asking which edge went
down.

One scare on the way: a faint buzz I first pinned on servo 6. Every leg
turned out to be commanded to the same 1500 µs with the arms off, so it
wasn't the code; it was the normal hum of servos holding position.

## 24 September

Wrote `motion.py` to exercise the platform one degree of freedom at a time:
type `yaw` and it oscillates in yaw until I type `stop`, eased in and out so
nothing starts with a jerk.

Before letting it run I measured how far each DOF can actually go, because
the machine was only ever sized for tilt. Roll and pitch reach 9.0° and 9.8°,
and heave 11 mm, all stopped by the servos. **Yaw, surge and sway stop at
about 3° and 3 mm, and the rod ends are what stop them.** That
follows from a decision I'd already made without thinking about it in these
terms: the arm-end bolt is straight, parallel to the shaft, which costs 25.1°
of the joint's ~30° cone at home. Tilt keeps the rod sweeping in a plane the
joint likes; yaw and the in-plane translations push it the other way, and the
remaining 5° goes quickly. If the off-tilt DOFs ever matter, the fix is the
tilted arm bolt I dropped on 18 September, not more servo travel.

The demo runs each motion at about two thirds of its measured limit.

## 2 October

First run with the platform assembled: **the joystick drives the tilt and the
plate follows it**, with the stick orientation left at the defaults. That is
the stage 1 loop working end to end, from the stick on the Arduino, through
the IK on the PC, back to six pulse widths.

It isn't a measurement yet, though. Every performance number in this log is
still a prediction from the evaluator: how much tilt the plate actually
reaches, how precisely it holds an angle and how fast it slews are all
unmeasured on the machine. That is the next job, and R1 (4.5°) is the first
one to check, since it is the cheapest to measure and the one the whole
geometry was sized for.

## 8 October

Wrote `tilt_test.py` to turn the predictions into measurements. Neither the
joystick nor `motion.py` holds a pose still long enough to read, so this one
holds a commanded tilt and waits while I read it off a level on the plate.
It runs two tests. `r1` commands 4.5° toward every 30° of azimuth and
compares what I measure with what was asked. `r2` goes to the same 3° tilt
ten times, half from level and half from 6°. The scatter within each half is
repeatability, and the gap between the halves is the deadband the loop will
never see. Neither has been run yet.

---

## Where Phase 0 stands

See `STATUS.md` at the repo root for the current state. The earlier version of
this section went stale and is in git history.
