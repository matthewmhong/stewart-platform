# Hardware

Specs for the parts made for the stage 1 build: what each one has to meet, and
how it was made.  The measured servo and rod-end figures are in `STATUS.md`
(MG90S parameters; Ball joints).  The Phase 0 survey of off-the-shelf horns,
joints, servos and rod stock is in `archive/docs/hardware-pull-2026-09-09.md`.

---

## 1. Horn extension — the spec the printed part has to meet (2026-09-16)

Design decided 2026-09-16 (see `docs/design-log.md`); the CAD is the user's.
This section is the requirement list, not a design.

**Job.** Carry the M3 rod-end bolt at `a ≈ 22 mm` from the spline centre, using
the MG90S stock horn as the spline interface.  Nothing critical is printed: the
moulded spline does the indexing.

| # | constraint | why |
|---|---|---|
| H1 | pivot radius `a = 20–25 mm`, one value, same on all six — **as made: 22.5 mm** (2026-09-18) | `G = a / r_p`; with `G` 0.24–0.38 this puts `r_p` at 58–70 mm, inside the 180 mm bed |
| H2 | **hysteresis at the rod-end hole ≤ 54 µm** under reversing rod load (~1 N, up to 40% of it out of the horn plane) | the whole R2 slack: 0.047° of the 0.25° budget, the rest being 0.116° deadband + 0.087° joint play |
| H3 | no drilling of the stock horn | holes are 1 mm in a ~4 mm arm; opening them to 2 mm leaves no material |
| H4 | shear carried mechanically, not by adhesive — **met by over-printing** (see As built) | CA does not bond POM, and MG90S horns are POM or nylon.  Two 1 mm steel pins (straightened paperclip) through the existing holes, plus a pocket that captures the arm against rotation |
| H5 | ~~adhesive is epoxy~~ — not needed with over-printing | epoxy tolerates POM/nylon and fills the gaps a printed pocket will have |
| H6 | bolt heads and the part clear the servo case through ±83° of travel | that is the measured travel; a collision at an end stop stalls the servo |
| H7 | load in-plane with the print layers | flat on the bed; layer adhesion is the weak axis |
| H8 | the six assemblies identical to within the R2 budget — one part for all six (straight rod-end hole; the tilted-hole design needed two mirror-image hands) | any per-leg difference is a fixed offset — trimmable in firmware, unlike play |

**As built (2026-09-18): over-printed, not pinned.**  The print pauses partway,
the stock horn is dropped into a pocket, and the rest of the part prints over
it, fully enclosing the horn's arm.  That meets H3 (no drilling) and H4 (shear
carried by the enclosure, zero play on assembly) without pins, and makes H5's
epoxy unnecessary.  To keep an eye on: the spline bore and the horn's centre
screw must stay open; the layer printed onto the horn is the weakest
interface; every part should use the same pause layer and pocket so H8 holds.

**Superseded 2026-09-18 — the arm keeps a straight hole**; threaded-insert
spacers either side of the ball give ~30° of bind, enough for the 25.1° a
straight bolt needs.  Kept for the reasoning.

**Rod-end seat (2026-09-18).**  The ball stands only 0.92 mm proud of the
housing faces (ball 4.37 wide, housing 2.53 thick, Ø9.89), so a flat face
against the ball stops the housing at ~12° of tilt.  With the hole tilted 25°
the arm face also rises toward the housing on one side, so the ball needs a
**seat: a boss square to the bolt, no wider than the ball's flat face**, of
height `H` (along the bolt, at the bolt axis) of at least:

| rod-end tilt the arm allows | 0° (just seats) | 10° | 15° | 22° (full bind) |
|---|---|---|---|---|
| `H` | 1.39 | 2.09 | 2.39 | 2.76 |

The arm end needs only 0.3° (25° hole), so the full 22° is margin; **H = 3 mm**
recommended.  The ball centre is then `H + 2.19` = 5.19 up the bolt: 4.70
above the arm face along the shaft and 2.19 sideways across the arm, so the
hole's entry is offset 2.19 the other way to keep the ball on the arm's
centreline at 22.5.  At the platform a flat tab square to the bolt is enough
(~12° against the 4.2° needed).

The tilted axis runs from 2.19 mm off the centreline at the outer face to
4.50 mm off it at the inner face, past the R4 end of the current arm.  So v2
adds a **lobe, R4.5 at (22.5, −3.35)**, to the outline (reach 27.25 mm, inside
the rod end's own 27.45).  The bolt is an **M3×12 self-tapped into a Ø2.5
pilot**, about 7.6 mm of thread.  The captive nut goes: on a 25° axis it would
stand proud of the pause layer.

Permanence is acceptable: each MG90S ships with spare horns, and the 18°
spline granularity is trimmed in firmware (tooth count: `archive/docs/hardware-pull-2026-09-09.md` §6), so nothing needs re-indexing.

---

## 2. Servo bracket — the spec (2026-09-18)

Design is the user's; this is the requirement list.  Written before the stiffness check so
the check has thresholds that came from the error budget rather than from
the CAD.

| # | constraint | why |
|---|---|---|
| B1 | **shaft axis parallel to the base plane, within 1°** | the IK's fixed minus branch rests on horizontal shafts (`docs/derivation.md` §8); 1° of cant is 0.10 mm of out-of-plane tip error, systematic rather than random |
| B2 | shaft centres on `r_b = 80 mm`, at the `beta = 10, delta = 0` positions from `base_ring` | the design of 2026-09-18 |
| B3 | case hangs below the shaft: body 6.75 above / 16 below the shaft centre, tabs 11.5 above / 20.7 below | matches `Body` in `performance.py`, which the clearance figures assume |
| B4 | compliance at the arm tip ≤ **~60 µm** under the load case below, shared with the horn extension's 54 µm (§1 H2) | bracket flex enters R2 exactly like gearing error; the split between bracket and horn is still to be allocated |
| B5 | mounts with the MG90S tabs: body 22.75 × 12.3, tabs 32.2 across and 2.75 thick, tab band 11.8–14.55 mm below the spline top; slot pattern is the CAD's | measured 2026-09-18 (`STATUS.md`, MG90S case) |
| B6 | leaves the horn's swept volume clear: 32.3 × 12 × 4.95 mm turning about the shaft | `horn_clearance()`; at the final placement the nearest approach to a neighbouring case is 42.4 mm (2026-09-18) |
| B7 | **shaft centre 30 mm above the base-plate top, the same on all six** | the printed arm reaches 26.5 mm from the shaft axis (measured 2026-09-18), 26.35 mm below it at the ±83° end stop, so **3.65 mm** to the plate: room for M3 pan heads (~2.4 mm) and print tolerance.  The lower tab hangs 20.7 mm (measured 2026-09-18), 9.3 mm clear.  28 was tried first and left only 1.65 mm.  Lower is stiffer (bracket flex ~ height³), so thicken the wall rather than go higher |
| B8 | **casing reaches no more than ~12 mm from the shaft centre toward the plate centre** (radially inward); outward, above and behind are not limited by the rods | own-rod clearance vs casing half-width inward: 6.15 (bare case) → 14.2 mm, 10 → 10.4, 12 → 8.4, 14 → 6.5, 16 → 4.5.  Height above the shaft barely matters (20 mm above: 13.6).  These figures predate the axial placement; the as-designed bracket in its final position clears its own rod by 15.8 mm |
| B9 | nothing within 27.5 mm of the shaft axis comes closer than 1 mm to the arm extension's back face; under the arm, nothing taller than ~2.5 mm on the plate | the arm sweeps a 26.5 mm radius and reaches 3.65 mm above the plate at the end stop |
| B10 | located along its shaft by the rod-end ball plane: the (x, y) in B2 lies in that plane, and the tabs sit `ball_offset + 11.8` to `ball_offset + 14.55` behind it | **settled 2026-09-18:** the window face (the tabs' far face) is 22.21 mm behind the ball plane (ball 6.21 out from the arm face, arm face 16.0 from the window face) |

**Concept (2026-09-18, the user's):** a C-shaped bracket, one wall with the
body window plus a top arm and a foot, tabs fixed to the wall, the foot screwed
up from under the base plate into heat-set inserts.  What the constraints above
mean for it:

- the C opens toward the **case** side; the window face is the horn side (the
  horn sits on the tab side of the wall, so a C opening that way would be in
  the arm's sweep);
- window 14–36.75 mm above the plate, tabs 9.3–41.5 on the horn-side face;
  the foot may rise to 14 and support the case's lower end;
- keep it symmetric about the shaft and within B8 on both sides, and one
  bracket fits all six;
- **tab-screw edge distance:** slot centres sit ~2.5 mm past the window edge
  (typical, measure), so an M2 heat-set insert (~3.2 mm hole) leaves < 1 mm of
  wall; the MG90S's own self-tappers in a ~1.5 mm pilot hole avoid it;
- locate the foot with dowels or a pocket, not the screws alone: clearance-hole
  float of ±0.3 mm over a 20 mm screw pitch is up to 1.7° of yaw (B1 wants 1°);
- print with the C profile flat on the bed, so every layer holds a whole C.

**As designed (2026-09-18, CAD dimensions; fit found by test prints):**

| feature | value |
|---|---|
| overall | 20.35 wide × 20.0 deep × 42.5 tall |
| C profile | top arm 11.5 deep × 5.5 thick, spine 4.0, foot 20.0 deep × 14.0 tall (all from the window face) |
| window | 12.35 × 23.00, 4.00 walls each side, bottom at 14.00 (the servo seat), top at 37.00; 5.50 above it |
| tab holes | Ø1.85 for the 2.06 mm MG90S self-tappers, centred (10.175), 2.53 above and below the window: 39.53 and 11.47 above the base, 28.06 apart |
| lugs | two, diagonal: Ø6.00 outer, Ø3.20 hole, ~2 thick; centres 3.0 outside the side faces (13.175 from the shaft centreline), one 3 behind the window face, the other 17 behind it (3 from the back); **M3 bolts and nuts** through the plate |
| wire notch | at the top of the window, window-face side (dimensions not given) |

Checked as a solid envelope at its final position: own rod 15.8 mm, other
rods 44.8, neighbouring arms 51.0.  With the lugs, the base footprint is
**r = 104.1 mm, ~210 mm across**, so the plate comes from sheet, not the
180 mm bed.  The top tab hole
breaks into the wire notch (no wall below the screw); accepted by the user,
since the screw still holds on the other three sides.

**Placement on the base plate (2026-09-18).**  Along its shaft, each bracket
is set by the rod-end ball: ball centre 6.21 mm out from the arm's outer face
(4.02 insert + 2.19), and the arm's outer face **16.0 mm** from the window
face (measured with the servo fitted), so the window face is **22.21 mm**
behind the ball plane, the plane that contains the B2 shaft points.  The
bracket's own axes: window face at Y = 0, C opening toward +Y, Z up, shaft on
X = 10.175; lug A at (−3, 3), lug B at (23.35, 17).  Hole centres (Ø3.2, M3
bolts and nuts), base-plate frame, mm:

| leg | lug A (x, y) | lug B (x, y) |
|---|---|---|
| 1 | 87.38, −41.01 | 59.00, −50.22 |
| 2 | 61.43, 36.43 | 84.95, 54.79 |
| 3 | −8.18, 96.18 | 13.99, 76.21 |
| 4 | −62.27, 34.99 | −89.93, 46.17 |
| 5 | −79.20, −55.17 | −72.99, −25.99 |
| 6 | 0.83, −71.42 | 4.98, −100.97 |

Origin at the plate centre, +x toward the midpoint of servos 1 and 2.  The
footprint reaches r = 104.1 mm with the lugs, so the plate is at least
~210 mm across.  Clearances with everything placed: rod to own bracket
15.8 mm, rod to other brackets 44.8, arm to arm 30.8, rod to rod 20.1.
Regenerate the table and plan with `python layout.py` (writes `base-layout.png`).

**Load case** (all at one servo):

1. reaction torque about the shaft, `rod force × a` ≈ 22 N·mm at ~1 N of rod
   force, higher transiently during a slew;
2. rod force at the arm tip, ~1 N with ~40% out of the servo plane — this is
   what twists the bracket rather than merely bending it;
3. screw preload, if the tabs are modelled in detail.

Per-leg rod force from the platform's weight is **0.18–0.19 × the weight**
per rod, compressive, over the whole envelope (computed 2026-09-18); at the
200–250 g estimate that is ~0.45 N.  The ~1 N above covers dynamics with
margin.  Buckling is not a concern and
does not need FEA: Euler for a 70 mm M3 steel rod is ~650 N against ~1 N
working load.

**Load case from the kinematics (2026-09-18).**  Platform taken as 250 g
(W = 2.45 N), and for the slew as a 170 mm disc tilting 9° in 0.14 s
(bang-bang, 32 rad/s² peak):

| | value |
|---|---|
| rod force, static, over the R1 envelope | 0.440–0.464 N (0.179–0.189 W), always compression |
| rod force during an R3 slew | 0.345–0.557 N, still compression |
| direction at the arm tip | 90% tangential, **42% along the shaft** (0.420–0.423 over the envelope), 8.5% radial |
| shaft torque | 8.5–9.5 N·mm static, 11.3 in a slew |

In the bracket frame (window face at Y = 0, C opening toward +Y, Z up, origin
on the shaft line at the window face), per **1 N** of rod compression, at home:

| legs | ball centre (X, Y, Z) mm | force on the ball (X, Y, Z) N |
|---|---|---|
| 1, 3, 5 | (+22.50, −22.21, 0) | (+0.085, −0.423, −0.902) |
| 2, 4, 6 | (−22.50, −22.21, 0) | (−0.085, −0.423, −0.902) |

Resolved at the origin (leg 1) that is 20.0 N·mm about X, 20.3 about Y (the
shaft torque) and −7.6 about Z.  The −Y component and the 22.21 mm lever pull
the servo off the wall: the **upper tab screw is in tension**, ~1 N per newton
of rod force, and the case's back end lifts toward the top arm rather than
bearing on the foot.

**Hand calc (2026-09-18).**  Servo rigid, hinged at the lower tab screw (the
foot is rigid by comparison), upper screw carried by the two 4 × 4 mm posts
beside the window (23 mm tall), servo case not touching the top arm (0.25 mm
gap).  In **eSUN PLA+** (E = 1900 MPa: its datasheet flexural modulus,
~1970 MPa on moulded bars, rounded down for printing; toughened, so softer than
plain PLA's ~2.3 GPa), the ball moves **67 µm along the rod per newton**.  The
model leaves out the lugs, the foot and the screw threads, which add
compliance.  Stress is ~1 MPa against ~50 MPa yield, so strength is not a
question.

**B4 is exceeded only at its 1 N design load**, which is about twice what the
rods carry: 0.46 N static gives 31 µm, and the 0.56 N slew peak 37 µm.  R2
passes even counting all of the flex as random error (quadrature, 0.1 mm joint
play):

| flex counted | R2 |
|---|---|
| none | 0.140° |
| 31 µm (static load) | 0.170° |
| 37 µm (slew peak) | 0.176° |
| 67 µm (B4's 1 N) | 0.204° |

against 0.25°.  Most of it does not count at all: the static rod force varies
5% over the envelope, so the deflection is nearly a fixed offset that trim and
the camera absorb.  Play and flex share one budget, so if the joint play turns
out near its 0.23 mm ceiling, R2 is tight whatever the bracket does.

**Accepted as printed, 4 mm spine (decided 2026-09-18).**  No FEA and no bench
check.  Not verified: the lugs, the foot and the screw threads, which the hand
calc treats as rigid.  If R2 comes up short on the assembled platform, check
them first.

## 3. Top-plate anchors — the spec (2026-09-18)

Design is the user's; this is the requirement list.  Six rod-end mounts hanging
under the top plate, one per leg.

**Anchor positions** (ball centres), plate frame: origin at the plate centre
in the plane of the six ball centres, +x toward the midpoint of anchors 1 and
2, the same frame as the base-plate table in §2 at home.  `r_p = 70`,
`beta_p = 35`:

| leg | x | y | bolt axis (horizontal) |
|---|---|---|---|
| 1 | 57.34 | −40.15 | along 0° (x) |
| 2 | 57.34 | 40.15 | along 0° |
| 3 | 6.10 | 69.73 | along 120° |
| 4 | −63.44 | 29.58 | along 120° |
| 5 | −63.44 | −29.58 | along 240° |
| 6 | 6.10 | −69.73 | along 240° |

**Bolt: horizontal, parallel to the pair's bisector** (the line from the plate
centre to the midpoint of the pair).  Worst rod-end misalignment over the
envelope is **4.2°**, the same as the optimum axis (which is 2.1° off the
bisector and 0.7° off horizontal).  So both mounts of a pair face the same
way and all six are one part.  For comparison: a vertical bolt gives 69°
(binds), a bolt along the anchor's own radius 18.3°, tangential 24.6°.

At home each rod leaves its ball 64.5° below horizontal, heading 126° from
radially outward.  4.2° is well inside the rod end's ~12° flat-face limit
(§1, rod-end seat), so **no insert spacers are needed at this end**: the rod
end bolts flat against the mount.

| # | constraint | why |
|---|---|---|
| T1 | ball centres at the positions above, all six in one plane | the kinematics' `p`; a height difference between anchors is a fixed tilt offset, trimmable but better avoided |
| T2 | bolt horizontal and parallel to the pair bisector, within a few degrees | 4.2° used of ~12° (flat face) or ~22° (bare bind) |
| T3 | the mount must not reach into the rod's path below and beside the ball | the rod leaves steeply downward; keep the mount above the ball centre plus the housing radius (Ø9.89) |
| T4 | ball-centre depth below the plate underside: the user's choice, the same for all six | free as far as the kinematics go (it sets the plate height, not `p`); shallower means a stiffer tab.  About 8 mm clears the housing (radius 4.95) with 3 mm to spare |

**Tab shape (2026-09-18).**  The rod-end shank starts 4.95 mm from the ball
centre and the rod leaves steeply downward, so a tab whose rounded end is
wider than ~4 mm in radius meets the shank.  Checked over the envelope with the
shank as a cylinder of radius 2.5–3.5 (not measured): a tab radius of 4.5 mm
clears by only 0.2 mm, **4.0 by ~0.7 mm**.  A **1 mm seat boss** (Ø5, no
wider than the ball's flat face) between tab and ball keeps the housing clear
as well.

**Loads** are the same rod force as the base end, 0.44–0.56 N compression,
so a short printed tab is ample.  The top end sees 4.2° of misalignment
against the base end's 25.1°, so it is the easier end of the rod.
