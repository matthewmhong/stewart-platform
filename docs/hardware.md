# Hardware pull — 2026-09-09

**Supersedes the previous shortlist entirely.** That shortlist is not in the repo and
its provenance cannot be checked, so nothing in it is carried forward, cited, or
amended here. Every figure below was pulled fresh from a named source on
2026-09-09 and carries its own URL.

## Scope and rules used

Absolute scale is fixed and is *not* revisited here: `r_b = 90 mm`, `r_p = 80 mm`,
220 mm bought sheet on a printed hub, 2.7 g ping-pong ball, tilt envelope
10.529 deg. Every length is given in millimetres **and** as its ratio at
`r_b = 90`.

Evidence tags:

| tag | meaning |
|-----|---------|
| **MFR** | the manufacturer's own page, catalogue or datasheet |
| **RETAIL** | a distributor or marketplace listing; test method unstated |
| **REVIEW** | third party, user-measured, or community-compiled |
| **NONE** | no source found |

Where a spec is not published the cell is left empty and marked **NOT PUBLISHED**.
Nothing is estimated, interpolated, or inferred from a similar part. Where two
sources disagree, both are reported and neither is resolved.

**No part is recommended and no sweep range is chosen.** This document reports what
exists.

---

## 1. Servo horn lengths — the discrete set `a` lives in

The previous pull's 12–32 mm was a sampling artefact of a single arm set. It is.
ProModeler alone publishes a hole ladder that runs continuously from 9 mm to
**60.4 mm**, in 5 mm steps above 20 mm, with the hole positions stated per part.

**The sweep's optimum at `a/r_b = 0.60` (54 mm) is inside the off-the-shelf set,
not beyond it.** Two stock ProModeler arms bracket it: PDRS55-25T carries holes at
50.4 and 55.4 mm, PDRS60-25T at 50.4, 55.4 and 60.4 mm. Nothing longer than
**60.4 mm** was found as a single off-the-shelf arm.

All distances below are **shaft centre to the outermost usable hole** (and the full
ladder where the maker publishes it), not overall arm length.

### 1.1 ProModeler — 7075-T6, M3-threaded hole ladder (MFR)

Source: <https://www.promodeler.com/arms>

| part | spline | hole positions from shaft centre (mm) | outermost (mm) | outermost `a/r_b` | source |
|------|--------|----------------------------------------|----------------|-------------------|--------|
| PDRS301 | 15T | 9 … 18 | 18 | 0.200 | MFR <https://www.promodeler.com/PDRS301> |
| PDRS201 | — | 10, 15, 20 | 20 | 0.222 | MFR <https://www.promodeler.com/PDRS201> |
| PDRS101 | — | 12.5 … 25 | 25 | 0.278 | MFR <https://www.promodeler.com/PDRS101> |
| PDRS15-25T | 25T | 4 positions on 5 mm centres, to 15 | 15 | 0.167 | MFR <https://www.promodeler.com/PDRS15-25T> |
| PDRS107 (round) | — | 3 holes/side on 5 mm centres, to 21 | 21 | 0.233 | MFR <https://www.promodeler.com/PDRS107> |
| PDRS303 | 15T | 16 … 26 | 26 | 0.289 | MFR <https://www.promodeler.com/PDRS303> |
| PDRS401 | — | 23 … 32 | 32 | 0.356 | MFR <https://www.promodeler.com/PDRS401> |
| PDRS25-15T | 15T | 15.4, 20.4, 25.4 | 25.4 | 0.282 | MFR <https://www.promodeler.com/arms> |
| PDRS32-25T | 25T | to 32 | 32 | 0.356 | MFR <https://www.promodeler.com/arms> |
| PDRS35-25T | 25T | to 35 | 35 | 0.389 | MFR <https://www.promodeler.com/arms> |
| PDRS40-25T | 25T | to 40 | 40 | 0.444 | MFR <https://www.promodeler.com/arms> |
| PDRS45-25T | 25T | 20.4, 25.4, 30.4, 35.4, 40.4, 45.4 | 45.4 | 0.504 | MFR <https://www.promodeler.com/PDRS45-25T> |
| **PDRS55-25T** | 25T | 25.4, 30.4, 35.4, 40.4, 45.4, 50.4, **55.4** | **55.4** | **0.616** | MFR <https://www.promodeler.com/PDRS55-25T> |
| **PDRS60-25T** | 25T | 25.4, 30.4, 35.4, 40.4, 45.4, 50.4, 55.4, **60.4** | **60.4** | **0.671** | MFR <https://www.promodeler.com/PDRS60-25T> |
| PDRS55-15T | 15T | to 55 (ladder NOT PUBLISHED on index) | 55 | 0.611 | MFR <https://www.promodeler.com/arms> |
| PDRS60-15T | 15T | to 60 (ladder NOT PUBLISHED on index) | 60 | 0.667 | MFR <https://www.promodeler.com/arms> |

PDRS55-25T and PDRS60-25T are stated to be **single-sided** arms in 7075-T6, with
25T = ø6 mm standard spline, threaded M3 at each position.

**Double arm, inch-dimensioned** — PDRS204, "Arm, servo, 3-3/4"", 25-spline
Futaba-compatible (MFR, <https://www.promodeler.com/PDRS204>). Holes stated from
centre:

| hole | inch from centre | mm | `a/r_b` |
|------|------------------|----|---------|
| 1 | 13/16" | 20.64 | 0.229 |
| 2 | 1-9/32" | 32.54 | 0.362 |
| 3 | 1-1/2" | 38.10 | 0.423 |
| 4 | 1-11/16" | 42.86 | 0.476 |
| 5 | 1-7/8" | 47.63 | 0.529 |

*Note on ProModeler's naming, not a contradiction between sources:* the inch label
in each part name is a rounded nominal that sits ~1–1.4 mm below the outermost
published hole (PDRS55-25T is named "2-1/8"" = 53.98 mm but holes to 55.4 mm;
PDRS45-25T is "1-3/4"" = 44.45 mm but holes to 45.4 mm). The **mm hole ladder is
the specification**; the inch label is the product name.

### 1.2 Other makers

| maker / part | spline | hole positions from shaft centre (mm) | outermost `a/r_b` | tag | source |
|---|---|---|---|---|---|
| FingerTech Aluminum Servo Arm | 25T (Futaba) | 14.75, 19.75, 24.75, 29.75 (first hole 14.75, four holes on 5.0 mm centres); overall length 37.3 mm | 0.331 | MFR | <https://www.fingertechrobotics.com/proddetail.php?prod=metal-servo-arm> |
| Secraft V2, 15 mm, Futaba 25T | 25T | arm length 15.0 mm, 6061-T6 | 0.167 | RETAIL | <https://www.gator-rc.com/products/secraft-15mm-servo-arm-v2-futaba> |
| Secraft V1, 17 mm, 25T | 25T | arm length 17 mm | 0.189 | RETAIL | <https://superstitionhobbies.com/products/secraft-servo-arm-v1-17mm-m2-red-futaba> |
| Secraft V2, 1.25", Futaba 25T | 25T | 31.75 mm | 0.353 | RETAIL | <https://gator-rc.com/secraft-servo-arm-v2-1-25-futaba> |
| Hobbypark long 25T single-sided | 25T | listing text gives hole centres at 13 and 35 mm from output; body 46 × 16 × 9 mm | 0.389 | RETAIL | <https://www.amazon.com/Hobbypark-Aluminum-Steering-Threads-Accessories/dp/B07TDMGJJR> |
| REV Robotics Aluminum Servo Horn V2 (REV-41-1828) | 25T | **NOT PUBLISHED** — page states "6 M3 clearance holes" and additional tapped M3 holes but gives no hole distances or overall length | | MFR | <https://www.revrobotics.com/rev-41-1828/> |
| Xpert Aluminum Servo Arm (25T) | 25T | **NOT PUBLISHED** | | RETAIL | <https://www.amainhobbies.com/xpert-aluminum-servo-arm-25t-xptm-25t-0/p999978> |

### 1.3 What is *not* an off-the-shelf horn

Search surfaced 25T "extension arms" quoted at 44 / 52 / 60 / 68 / 76 / 84 / 92 mm.
That ladder is a **3D-printed Thingiverse part**
(<https://www.thingiverse.com/thing:6750263>), not bought hardware, and is excluded
from the set above. Recording it only so it is not later mistaken for stock.

### 1.4 Answer to the question as asked

Off-the-shelf single servo arms with a published hole distance exist up to
**60.4 mm (`a/r_b = 0.671`)**. Above 60.4 mm nothing was found. The `a/r_b = 0.60`
optimum is served by stock parts at 55.4 mm (`0.616`) and by the 50.4 mm hole
(`0.560`) on the same two arms.

---

## 2. Ball-joint housing OD — the outer bound on `beta_p`

Constraint: two housings cannot occupy one hole, and the current design has anchor
pairs **13.9 mm** apart.

The previous pull rested this on one data point (igus KBRM-03, 13.0 mm, giving
0.9 mm clearance). **That figure is confirmed, and it is the largest of the M3-class
set, not the only member.** Eleven further M3-class parts publish a housing OD, and
the published range is **9.0 – 13.0 mm**.

| part | class | housing / head OD (mm) | OD `/r_b` | clearance at 13.9 mm pair spacing | tag | source |
|---|---|---|---|---|---|---|
| igus KBRM-03 / KBLM-03 igubal | plastic spherical rod end, M3 female | **13.0** (`d2`) | 0.144 | 0.9 mm | MFR | <https://media-pim.rubix.com/medias/technical_datasheet/83/65/1300000086583/igubal_KBRM_W300_GBen.pdf> |
| Aurora MM-M3 | precision metal male rod end, M3×0.5 | **12.50** (`D`, head diameter) | 0.139 | 1.4 mm | MFR | <https://cad.timken.com/item/metric-rod-ends/ies-male-rod-ends-metric-general-purpose-precision/mm-m3> |
| Aurora MW-M3 | precision metal female rod end, M3×0.5 | **12.50** (`D`, head diameter) | 0.139 | 1.4 mm | MFR | <https://cad.timken.com/item/metric-rod-ends/s-female-rod-ends-metric-general-purpose-precision/mw-m3> |
| PHS03L | metal spherical rod end, M3×0.5 | **12** (page label "Outer Diameter") | 0.133 | 1.9 mm | RETAIL | <https://bearingsdirect.com/phs03l-rod-end-bearing-left-hand-rod-10mm-x-bore-3mm-mg-m3/> |
| POS3 | metal male rod end, M3×0.5 | **12** (page's `S` column) | 0.133 | 1.9 mm | RETAIL | <https://vxb.com/products/pos3-male-rod-end-3mm-right-hand-bearing> |
| RC4WD Z-S0401 / Z-S0400 | M3 offset plastic rod end | **11.0** | 0.122 | 2.9 mm | MFR | <https://www.rc4wd.com/ProductImages/PDFs/Rod%20Ends.pdf> |
| RC4WD Z-S0359 / Z-S1434 | M3 high-precision billet tie rod end | **10.0** | 0.111 | 3.9 mm | MFR | same PDF |
| RC4WD Z-S1362 / S1346 / S1370 / S1354 | M3 mini aluminium Axial-style | **10.0** | 0.111 | 3.9 mm | MFR | same PDF |
| RC4WD Z-S0074 / S0398 / S0399 / S0402 / S0403 | M3 plastic rod ends | **10.0** | 0.111 | 3.9 mm | MFR | same PDF |
| RC4WD Z-S0048 / S0055 / S0057 / S0427 / S1448 | M3 aluminium rod end with steel ball | **9.8** | 0.109 | 4.1 mm | MFR | same PDF |
| RC4WD Z-S0146 / S0424 / S0481 | M3 steely / bend rod ends | **9.8** | 0.109 | 4.1 mm | MFR | same PDF |
| RC4WD Z-S0138 / S0139 / S0140 | M3 high-precision billet tie rod end | **9.8** | 0.109 | 4.1 mm | MFR | same PDF |
| **RC4WD Z-S1414** | Truck Specs rod end (M3) | **9.0** | 0.100 | 4.9 mm | MFR | same PDF |
| igus KBRM-02 (for scale below M3) | plastic rod end, M2 female | 9.0 (`d2`) | 0.100 | 4.9 mm | MFR | igus datasheet above |
| RC4WD Z-S0486 / S0531 / S1454 / S1456 (below M3) | M2 micro aluminium rod end | 5.0 | 0.056 | 8.9 mm | MFR | same PDF |
| igus KBRM-05 (above M3, for scale) | plastic rod end, M5 female | 18.0 (`d2`) | 0.200 | −4.1 mm (will not fit) | MFR | igus datasheet above |
| REV Robotics M3 Ball Joint Rod Ends | M3 ball joint | **NOT PUBLISHED** — page gives only "Ball-hole Diameter: M3 Clearance" | | MFR | <https://www.revrobotics.com/M3-Ball-Joint-Rod-Ends/> |
| Traxxas rod ends / hollow ball connectors | RC ball-cup rod end | **NOT PUBLISHED** — no dimensional spec on any Traxxas product or parts-finder page | | MFR | <https://traxxas.com/products/parts/1942> |
| Bambu Lab M3 Aluminum Rod End Ball Joint | M3 ball joint | **NOT PUBLISHED** | | MFR | <https://us.store.bambulab.com/products/m3-aluminum-rod-end-ball-joint> |
| RJX M3 aluminium rod ends | M3 ball joint | **NOT PUBLISHED** | | MFR | <https://www.rjxhobby.com/rjx-m3-aluminum-metal-rod-ends-ball-joint-cw-2-ccw-2-for-traxxas-axial-redcat-racing-rc-car-airplane> |

### Provenance caveats on this section

- **igus `d2`.** The igus dimension table's drawing legend runs
  `d2 l2 h1 d5 d4 B C1 d1 l1 W d3`; `d2` is the only dimension that scales as the
  head across the whole series (9 at M2, 13 at M3, 18 at M5, 20 at M6, 50 at M30).
  Read as the housing head OD. The 13.0 mm figure is therefore confirmed
  independently of the previous pull.
- **RC4WD column headers.** The PDF's text layer contains the data rows but not the
  header row (the header is an image). The column order
  `Part Number | Name | Image | Ball Hole ID | Head OD | Ball Width | Shaft OD |
  Center hole to …` was recovered from the search index's title extraction of the
  same PDF, and is corroborated by the M2 rows reading 2.0 mm ball hole / 5.0 mm
  head OD against the M3 rows' 3.1 / 9.8. **The header assignment is inferred from
  document structure, not read off the page.** Flagged rather than silently used.
- **POS3 `S`.** The vxb page presents a metric dimensions table; `S = 12 mm` was
  reported as the head/housing OD, but the page does not print a drawing legend
  tying `S` to the head. Treat as RETAIL and unconfirmed.
- **Two device classes are mixed above** and are not interchangeable: spherical
  plain bearings (igus, Aurora, PHS/POS) and RC ball-cup rod ends (RC4WD,
  Traxxas). Both are "ball joints"; they differ in construction and in pivot angle.
  Reported together because both publish an OD, separated so the distinction is not
  lost.

### What this does to the 0.9 mm figure

Nothing is resolved here. The fact is that **housing OD is a 9.0–13.0 mm published
range across M3-class parts**, and the clearance at 13.9 mm pair spacing runs from
**4.9 mm down to 0.9 mm** across that range. The 0.9 mm case is the worst member,
and it no longer rests on a single data point.

---

## 3. Servo travel and deadband

Travel is excluded from feasibility by decision but is recorded here for
downstream. Deadband feeds tilt resolution.

| servo | travel (deg) | deadband (µs) | deadband (deg) | pulse range (µs) | tag | source |
|---|---|---|---|---|---|---|
| Hitec HS-5055MG | 75° with average radio system; 126° with travel tuner / wide-signal | **6 µs** | NOT PUBLISHED | NOT PUBLISHED | MFR | <https://hitecrcd.com/hs-5055mg-economy-metal-gear-feather-servo/> |
| Hitec HS-5055MG | 75° stock / 126° with travel tuner; 119° / 178° reprogrammed | **2 µs** | NOT PUBLISHED | 750–2250 | RETAIL | <https://www.servocity.com/hs-5055mg-servo/> |
| Hitec HS-5055MG | NOT PUBLISHED | NOT PUBLISHED (page lists "Dead Band" only as a *programmable function*, no value) | NOT PUBLISHED | NOT PUBLISHED | MFR | <https://www.hiteccs.com/actuators/product-details/HS-5055MG> |
| Savox SH-0255MG+ | NOT PUBLISHED | NOT PUBLISHED | NOT PUBLISHED | NOT PUBLISHED | MFR | <https://savox-servo.com/en/product/SH-0255MGplus/> |
| Savox SH0255MG | 90° | NOT PUBLISHED | NOT PUBLISHED | 750–2250, neutral 1500 | REVIEW | <https://servodatabase.com/servo/savox/sh0255mg> |
| ROBOTIS Dynamixel XL330-M288-T | 0–360° (position control mode) | n/a — digital bus servo, no PWM deadband published | **0.0879°/pulse** (4096 pulse/rev) | n/a | MFR | <https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/> |
| Hitec full servo specification chart | NOT PUBLISHED — the chart's columns are MODEL, CASE SIZE, SPLINE, GEAR TYPE, STALL TORQUE, NO-LOAD SPEED only. **No travel column, no deadband column** | | | | MFR | <https://hitecrcd.com/servo-specification-chart/> |

### Disagreement, reported not resolved

**Hitec HS-5055MG deadband: 6 µs (MFR, hitecrcd.com) vs 2 µs (RETAIL,
servocity.com).** A factor of three. Both are recorded; neither is preferred here.

### On converting deadband to degrees

**No source publishes the conversion.** The cells above are therefore empty.

What *is* available: ServoCity's single page carries both a travel figure (126°
with a wide-signal controller) and a pulse range (750–2250 µs, a 1500 µs span). If
those two figures describe the same endpoints — which that page does not state —
the scaling is 0.084 °/µs, putting the 2 µs deadband at 0.168° and the 6 µs
deadband at 0.504°. **That is arithmetic on two same-page figures, not a published
spec**, and it is recorded here as a footnote rather than a table cell precisely so
it is not later cited as one.

The Dynamixel resolution figure (0.0879°/pulse) *is* a published angular
quantisation, but it is a commanded-position resolution on a digital bus servo, not
a PWM deadband, and the two are not the same quantity.

---

## 4. Servo step response and sensor frame interval — the two terms of `tau_L = 150 ms`

`tau_L = 150 ms` is **provisional**, inherited from a figure withdrawn as invented,
and carries 3.97 of the 10.529 deg tilt limit. Both of its terms were searched
specifically.

### 4.1 Servo step response — NOT PUBLISHED by any manufacturer found

Every hobby servo maker checked publishes **seconds-per-60-deg no-load speed**.
That is a **slew rate**: the maximum rate of change at full command. It is **not a
step response**: it contains no propagation delay, no rise time to a small
commanded step, no overshoot, and no settling criterion. A servo can have a fast
slew rate and a slow settling time if it is lightly damped, and one cannot be
computed from the other. Stated explicitly here because the two are routinely
treated as equivalent.

| source | what it publishes | step response / settling time | tag | url |
|---|---|---|---|---|
| Hitec HS-5055MG | 0.20 sec/60° @ 4.8 V, 0.17 sec/60° @ 6.0 V — **slew rate only** | **NOT PUBLISHED** | MFR | <https://www.hiteccs.com/actuators/product-details/HS-5055MG> |
| Savox SH-0255MG+ | 0.16 s/60° @ 4.8 V, 0.13 s/60° @ 6.0 V — **slew rate only** | **NOT PUBLISHED** | MFR | <https://savox-servo.com/en/product/SH-0255MGplus/> |
| ROBOTIS XL330-M288-T | 103 rev/min no-load @ 5.0 V — **slew rate only**; control frequency / update rate absent from the spec table | **NOT PUBLISHED** | MFR | <https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/> |
| Hitec servo specification chart (whole product line) | NO-LOAD SPEED column only | **NOT PUBLISHED** | MFR | <https://hitecrcd.com/servo-specification-chart/> |

**The only measured dynamic data found is third party:**

| figure | value | conditions | tag | url |
|---|---|---|---|---|
| Closed-loop bandwidth, small amplitude | ~8 Hz at 90° phase lag | Corona DS919MG @ 5 V and Savox SV1232MG @ 7.4 V | REVIEW | <https://domwil.co.uk/posts/servo-performance/> |
| Closed-loop bandwidth, higher amplitude | 4 Hz (Corona) vs 5 Hz (Savox) | same rig | REVIEW | same |
| Delay at start of a 15° step | ~10 ms, and the author attributes it to the 100 Hz sampling interval, not to the servo | same rig | REVIEW | same |
| Peak velocity, ~120° step | ~1000 °/s ("0.06 seconds for 60 degrees, quite close to the spec") | same rig | REVIEW | same |
| Method | relative encoder, 0.09° resolution, motor updated and position sampled at 100 Hz, serial to MATLAB | | REVIEW | same |
| Settling time to a stated band | **NOT PUBLISHED / NOT MEASURED** | | NONE | — |

Note the ~10 ms figure is a **measurement-rig artefact the author identifies as
such**, not a servo delay, and neither servo tested is one of the candidates above.

### 4.2 Sensor frame interval — published, for cameras

| sensor | mode | fps | frame interval | tag | url |
|---|---|---|---|---|---|
| Sony PlayStation Eye | 640 × 480 | 60 | 16.67 ms | MFR | <https://blog.playstation.com/2007/10/10/playstation-eye-a-little-more-info/> |
| Sony PlayStation Eye | 320 × 240 | 120 | 8.33 ms | MFR | same |
| Sony PlayStation Eye | 320 × 240 | 187 | 5.35 ms | REVIEW (community drivers: FreeTrack / LinuxTrack; not a Sony figure) | <https://en.wikipedia.org/wiki/PlayStation_Eye> |
| Sony PlayStation Eye | 640 × 480 | 75 | 13.33 ms | REVIEW (same drivers) | same |
| Raspberry Pi Camera Module 3 | 1536 × 864 | 120 | 8.33 ms | MFR | <https://www.raspberrypi.com/documentation/accessories/camera.html> |
| Raspberry Pi Camera Module 3 | 2304 × 1296 | 56 | 17.86 ms | MFR | same |
| Raspberry Pi Camera Module 3 | 2304 × 1296 HDR | 30 | 33.33 ms | MFR | same |
| Raspberry Pi Camera Module 2 | 640 × 480 | 206 | 4.85 ms | MFR | same |
| Raspberry Pi Camera Module 2 | 1640 × 1232 | 41 | 24.39 ms | MFR | same |
| Raspberry Pi Camera Module 2 | 1080p | 47 | 21.28 ms | MFR | same |
| Raspberry Pi Global Shutter Camera | 1456 × 1088 | 60 | 16.67 ms | MFR | same |

**What is not published for any of these:** exposure time, sensor readout time,
USB/CSI transfer time, and detection latency. **Frame interval is a sampling period,
not a sensor-to-pose latency**, and the difference is not recoverable from any
published figure found.

### 4.3 Status of `tau_L = 150 ms`

Neither term is closed:

- servo step response — **no manufacturer publishes it at all**; one third-party
  bandwidth measurement exists, on two servos that are not candidates;
- sensor frame interval — **published and well covered**, spanning 4.85 ms to
  33.33 ms depending on sensor and mode, but it is only the sampling period, not the
  full sensor path.

`tau_L` cannot be reconstructed from published data. It remains provisional, and the
3.97 deg of the 10.529 deg tilt limit that hangs on it remains unsupported.

---

## 5. Servo body diameter / mounting-arc width — `beta`'s usable range

Constraint: two servo bodies cannot occupy one mounting arc. The dimension that
matters is the body width across the mounting arc.

Hitec's specification chart publishes a dedicated **CASE SIZE** column, which is the
body width.

| servo | case size / width (mm) | width `/r_b` | full body L × W × H (mm) | tag | source |
|---|---|---|---|---|---|
| Hitec HS-5065MG | **11.4** | 0.127 | NOT PUBLISHED on chart | MFR | <https://hitecrcd.com/servo-specification-chart/> |
| Hitec HS-5055MG | **11.6** | 0.129 | 22.8 × 11.6 × 24.0 | MFR | <https://www.hiteccs.com/actuators/product-details/HS-5055MG> |
| Savox SH-0255MG+ | **12.0** | 0.133 | 22.8 × 12.0 × 29.4 | MFR | <https://savox-servo.com/en/product/SH-0255MGplus/> |
| Hitec HS-5085MG | **13.0** | 0.144 | NOT PUBLISHED on chart | MFR | <https://hitecrcd.com/servo-specification-chart/> |
| Hitec HS-5087MH | **13.0** | 0.144 | NOT PUBLISHED on chart | MFR | same |
| ROBOTIS XL330-M288-T | **20.0** (width) | 0.222 | 20.0 × 34.0 × 26.0 | MFR | <https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/> |

Published sub-micro / micro body width range: **11.4 – 13.0 mm**
(`0.127 – 0.144 r_b`). The Dynamixel at 20.0 mm (`0.222 r_b`) is a different class
and is listed for the upper bound.

**Not published anywhere found:** mounting-flange footprint, screw-hole pitch, and
the required clearance between adjacent bodies including wiring. The width above is
the body, not the installed arc.

---

## 6. Horn spline tooth count and mounting-angle resolution

Mounting-angle resolution is `360°/N` — the coarsest non-zero home angle a spline
can be clocked to.

| spline | tooth count | spline ø (mm) | mounting-angle resolution `360/N` | tag | source |
|---|---|---|---|---|---|
| Hitec micro ("A15T", "Feather 15") | 15 | **4.0** | **24.000°** | MFR | <https://www.hiteccs.com/actuators/product-details/HS-5055MG> |
| Savox micro | 21 | **4.8** | **17.143°** | MFR | <https://savox-servo.com/en/product/SH-0255MGplus/> |
| Airtronics / JR / KO / MRC / Multiplex / Sanwa / Spektrum | 23 | NOT PUBLISHED | **15.652°** | RETAIL | <https://www.robotdigg.com/news/168/RC-Servo-Spline-Chart> |
| Hitec standard | 24 | NOT PUBLISHED | **15.000°** | MFR | <https://hitecrcd.com/servo-specification-chart/> (SPLINE column: "Standard 24") |
| Ace/Thunder, Blue Bird, Cirrus, Duratrax, Futaba, HPI, Hobbico, Power HD, Savox, Tamiya, Team Associated, Tower, Traxxas, Xpert | 25 | **6.0** (ProModeler: "25T = ø6 mm standard-spline") | **14.400°** | RETAIL (list) / MFR (ø) | <https://www.robotdigg.com/news/168/RC-Servo-Spline-Chart> ; <https://www.promodeler.com/PDRS55-25T> |

**Caveat published by ServoCity:** tooth count alone does not determine
interchangeability — two splines with the same tooth count can have different
diameters (their worked example is A15T vs D15T, both 15 teeth, "the D15T is a much
larger spline size than the A15T"). Source: <https://www.servocity.com/glossary/>.
A consolidated tooth-count-to-diameter chart was **NOT PUBLISHED** on any source
found; the two diameters above are from individual product pages.

**Answer to what this gates:** a non-zero home angle costs, at worst, **half the
spline pitch** — 7.2° on 25T, 7.5° on 24T, 7.83° on 23T, 8.57° on 21T, 12.0° on a
15T micro spline — unless the horn is repositioned mechanically. Whether that cost
is material is not decided here.

---

## 7. Rod stock — bounds on `d`

`d` is cut to length so it is continuous, but stock bounds it: available diameters,
straightness, and whether ends are threaded.

### 7.1 Available diameters

| stock | diameter (mm) | `/r_b` | threaded? | tag | source |
|---|---|---|---|---|---|
| Du-Bro 2-56 threaded rod | 0.072 in = **1.83** | 0.020 | yes — 3/4" (19 mm) of 2-56 machine thread on **one end only** | MFR | <https://www.dubro.com/products/2-56-threaded-rods-12-305-mm-6-pkg> |
| Du-Bro 4-40 threaded rod | 0.093 in = **2.36** | 0.026 | yes — 3/4" (19 mm) of 4-40 thread on **one end only** | MFR | <https://www.dubro.com/products/4-40-threaded-rods-12-305-mm-6-pkg> |
| CST carbon pushrod stock | 0.030 / 0.040 / 0.050 / 0.060 / 0.070 in = **0.76 / 1.02 / 1.27 / 1.52 / 1.78** | 0.008 / 0.011 / 0.014 / 0.017 / 0.020 | **no** — supplied end caps are stated "untapped"; thread sizes NOT PUBLISHED | MFR | <https://www.cstsales.com/carbon_pushrod.html> |
| Carbon fibre rod, generic | **3.0 / 3.5 / 4.0 / 4.5** OD × 500 mm | 0.033 / 0.039 / 0.044 / 0.050 | no | RETAIL | <https://www.ebay.de/itm/293282493508> |
| DIN 975 / DIN 976 metric threaded rod | **M3** upward in 1/2 mm pitch increments | 0.033 at M3 | fully threaded, full length | RETAIL | <https://fullerfasteners.com/tech/din-975-din-976-specifications-threaded-rods-stud-bolts/> |
| A286 threaded rod | M3–M64 | 0.033 at M3 | fully threaded | MFR | <https://alloya286.com/threaded-rod> |

Du-Bro stock lengths: 12" (305 mm), 30" (762 mm), 48" (1016 mm). CST lengths: 39",
48", 72", 78". Both far exceed any `d` at this scale, so length is not a bound.

### 7.2 Straightness

**Almost universally NOT PUBLISHED.** One positive figure was found, and it is on a
specialty alloy, not on hobby stock:

| stock | straightness | tag | source |
|---|---|---|---|
| A286 cold-drawn threaded rod, M3–M64 | **≤ 2 mm/m**, stated twice: "Multi-roll straightener for ≤ 2 mm/m straightness tolerance" and "Straightness check per ASTM A484, typical ≤ 2 mm/m for cold-drawn threaded" rod | MFR | <https://alloya286.com/threaded-rod> |
| Du-Bro 2-56 and 4-40 threaded rod | **NOT PUBLISHED** — product copy instead describes the rod as "strong, flexible" and emphasises bendability | MFR | <https://www.dubro.com/products/2-56-threaded-rods-12-305-mm-6-pkg> |
| CST carbon pushrod stock | **NOT PUBLISHED** | MFR | <https://www.cstsales.com/carbon_pushrod.html> |
| Generic carbon fibre rod | **NOT PUBLISHED** | RETAIL | <https://www.ebay.de/itm/293282493508> |
| DIN 975 / DIN 976 | **NOT PUBLISHED** in any accessible spec sheet; the standard covers thread dimensions and tolerances, not straightness | RETAIL | <https://www.aspenfasteners.com/content/pdf/Metric_DIN_975_spec.pdf> |
| McMaster-Carr precision ground rods | **NOT RETRIEVED** — category page carries filters only; straightness lives on individual product pages, which were not reachable | RETAIL | <https://www.mcmaster.com/products/rods/mechanical-finish~precision-ground/> |
| MISUMI precision linear shafts | **NOT RETRIEVED** — product and PDF pages returned HTTP 403. Search indicates MISUMI publishes straightness, and that the series starts at 8 mm diameter, which is above this design's rod scale | RETAIL | <https://us.misumi-ec.com/vona2/detail/110302634310/> |

### 7.3 Threaded ends

Du-Bro rods are threaded **one end only**, 19 mm of thread; the other end is plain
for bonding into a tube or for a second operation. Carbon pushrod stock is
unthreaded and relies on bonded or crimped ends; CST states its end caps are
untapped. DIN 975/976 and A286 rod are threaded over the full length. **Every rod
in the set therefore either needs a second thread cut or is fully threaded already**
— no stock item ships with two ready threaded ends.

---

## 8. Torque, for the record

The 2.7 g ball weighs **0.0265 N**. Reported so the ruling can be checked rather
than assumed a third time.

| servo | stall torque | in N·m | tag | source |
|---|---|---|---|---|
| Hitec HS-5055MG | 1.3 kgf·cm @ 4.8 V / **1.6 kgf·cm @ 6.0 V** (18.05 / 22.20 oz·in) | 0.127 / **0.157** | MFR | <https://www.hiteccs.com/actuators/product-details/HS-5055MG> |
| Hitec HS-5065MG | 2.2 kgf·cm @ 6.0 V (30.55 oz·in) | 0.216 | MFR | <https://hitecrcd.com/servo-specification-chart/> |
| Hitec HS-5085MG | 4.3 kgf·cm @ 6.0 V (59.71 oz·in) | 0.422 | MFR | same |
| Hitec HS-5087MH | 4.3 kgf·cm @ 7.4 V (59.71 oz·in) | 0.422 | MFR | same |
| Savox SH-0255MG+ | 3.1 kgf·cm @ 4.8 V / **3.9 kgf·cm @ 6.0 V** | 0.304 / **0.383** | MFR | <https://savox-servo.com/en/product/SH-0255MGplus/> |
| ROBOTIS XL330-M288-T | 0.42 N·m @ 3.7 V / **0.52 N·m @ 5.0 V** / 0.60 N·m @ 6.0 V | **0.52** (= 5.30 kgf·cm) | MFR | <https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/> |

Published stall torque across the candidate class spans **0.127 – 0.52 N·m**. No
check against leg forces is performed here; the figures are recorded so one can be.

Also recorded, since a torque check needs them: Hitec HS-5055MG running current
120–150 mA, stall current 500–700 mA (MFR,
<https://www.hiteccs.com/actuators/product-details/HS-5055MG>); XL330-M288-T stall
current 1.47 A @ 5.0 V (MFR, emanual link above).

---

## 9. Unpublished-cell count

Counted the same way the previous pull's 16 was: one cell per (part or source ×
spec) pair that was sought and found absent.

| § | unpublished cell | part / source |
|---|---|---|
| 1 | hole distances from shaft centre | REV Robotics REV-41-1828 |
| 1 | overall length | REV Robotics REV-41-1828 |
| 1 | hole distances / arm length | Xpert XPTM-25T-0 |
| 1 | hole ladder | ProModeler PDRS55-15T |
| 1 | hole ladder | ProModeler PDRS60-15T |
| 2 | housing OD | REV Robotics M3 Ball Joint Rod Ends |
| 2 | housing OD | Traxxas rod ends / hollow balls |
| 2 | housing OD | Bambu Lab M3 aluminum rod end |
| 2 | housing OD | RJX M3 aluminum rod ends |
| 3 | deadband (µs) | Hitec HS-5055MG on hiteccs.com |
| 3 | deadband (µs) | Savox SH-0255MG+ on savox-servo.com |
| 3 | deadband (µs) | Savox SH0255MG, all sources |
| 3 | deadband in degrees | every servo, every source |
| 3 | travel (deg) | Savox SH-0255MG+ on savox-servo.com |
| 3 | travel (deg) | Hitec HS-5055MG on hiteccs.com |
| 3 | pulse-width range | Hitec HS-5055MG on hitecrcd.com |
| 3 | travel column | Hitec full servo specification chart |
| 3 | deadband column | Hitec full servo specification chart |
| 4 | step response / settling time | Hitec HS-5055MG |
| 4 | step response / settling time | Savox SH-0255MG+ |
| 4 | step response / settling time | ROBOTIS XL330-M288-T |
| 4 | step response / settling time | Hitec full specification chart (whole line) |
| 4 | control frequency / update rate | ROBOTIS XL330-M288-T |
| 4 | settling time to a stated band | Dom Wilson third-party measurement |
| 4 | exposure + readout + transfer latency | every camera listed |
| 5 | full body L×W×H | Hitec HS-5065MG |
| 5 | full body L×W×H | Hitec HS-5085MG |
| 5 | full body L×W×H | Hitec HS-5087MH |
| 5 | mounting-flange footprint / screw pitch | every servo |
| 5 | required inter-body clearance incl. wiring | every servo |
| 6 | spline diameter for 23T | all sources |
| 6 | spline diameter for 24T | all sources |
| 6 | consolidated tooth-count → diameter chart | all sources |
| 7 | straightness | Du-Bro 2-56 and 4-40 |
| 7 | straightness | CST carbon pushrod stock |
| 7 | straightness | generic carbon fibre rod |
| 7 | straightness | DIN 975 / DIN 976 |
| 7 | end-cap thread size | CST carbon pushrod kits |
| 7 | inside diameter | CST carbon pushrod stock |

**Total: 39 unpublished cells.**

Two further cells are **NOT RETRIEVED** rather than not published — the source is
believed to publish the figure but the page could not be reached (McMaster-Carr
precision ground rod straightness; MISUMI precision linear shaft straightness, both
403). They are excluded from the 39.

The previous pull reported 16. This pull covers more parts and more specs, so the
counts are not directly comparable as a ratio — but the shape of the result stands
and is sharper than before: **the design's geometric dimensions are well published
and its dynamic ones are not.** Of the 39, twenty-one (§3 and §4 in full, plus the
control-frequency and latency cells) concern **timing and resolution**. Sections 1,
2, 5, 6 and 7 — every purely geometric quantity except spline diameter and rod
straightness — came back with real numbers from manufacturers' own pages.

---

## 10. Disagreements between sources — reported, not resolved

1. **Hitec HS-5055MG deadband: 6 µs vs 2 µs.**
   MFR <https://hitecrcd.com/hs-5055mg-economy-metal-gear-feather-servo/> gives
   6 µs. RETAIL <https://www.servocity.com/hs-5055mg-servo/> gives 2 µs. A factor of
   three, directly on tilt resolution. The MFR figure is the larger one.

2. **Savox SH0255MG weight: 16 g vs 15.8 g.**
   MFR <https://savox-servo.com/en/product/SH-0255MGplus/> gives 16 g. REVIEW
   <https://servodatabase.com/servo/savox/sh0255mg> gives 0.56 oz (15.8 g). Minor,
   recorded for completeness.

3. **igus KBRM-03 housing OD 13.0 mm vs RC4WD M3-class 9.0–11.0 mm.**
   Not strictly a disagreement — different constructions (plastic spherical plain
   bearing vs RC ball-cup rod end) — but they are both sold as M3 ball joints and
   they differ by up to 4 mm, which is 4.4 % of `r_b`. Both reported; the
   distinction is flagged rather than averaged away.

4. **Provenance warning on servodatabase.com.** The site states of its own data:
   "Most data is sourced from the manufacturer. Some data is sourced from our loyal
   community and with the help of AI." Every servodatabase figure in this document
   is tagged REVIEW on that basis, and none is the sole source for anything.

---

## 11. What is missing, and what the missing figures gate

### Blocking

- **Servo step response.** No manufacturer publishes it; searched across Hitec,
  Savox, ROBOTIS and Hitec's whole-line chart. **Gates `tau_L`**, which is
  provisional, inherited from a withdrawn figure, and carries **3.97 of the
  10.529 deg tilt limit**. The only measured data found is one third-party
  bandwidth study (~8 Hz small-amplitude, 4–5 Hz at higher amplitude) on two servos
  that are not candidates. Closing this needs a bench measurement or a servo whose
  maker publishes settling time — none was found.

- **Sensor-path latency beyond the frame interval.** Frame intervals are well
  published (4.85–33.33 ms across the cameras listed) but exposure, readout,
  transfer and detection are not. **Gates the other term of `tau_L`.** The frame
  interval is a floor on that term, not the term.

### Constrains but does not block

- **Deadband in degrees.** Not published by anyone. The µs figures exist but
  disagree by 3× on the one servo that publishes them at all. **Gates tilt
  resolution.** Needs either a maker who publishes travel and deadband against the
  same pulse endpoints, or a bench measurement.

- **Spline diameter at 23T and 24T.** **Gates cross-brand horn interchange** — the
  §1 hole ladder is 25T and 15T, so a 23T or 24T servo cannot be assumed to take a
  ProModeler arm. Tooth count is published everywhere, diameter almost nowhere, and
  ServoCity states explicitly that equal tooth counts need not interchange.

- **Rod straightness for the stock actually at this scale.** The one published
  figure (≤ 2 mm/m) is on M3+ A286 threaded rod, not on the sub-2.4 mm Du-Bro rod or
  the carbon pushrod stock. **Gates how much of `d`'s length error is stock-borne
  rather than assembly-borne.**

- **Mounting-flange footprint and inter-body clearance.** Body widths are published
  (11.4–13.0 mm); the installed arc a servo actually occupies, including flange and
  wiring, is not. **Gates `beta`'s lower bound** — the widths above are a floor on
  it, not the bound itself.

### Answered, and no longer gating

- **Horn length `a`.** Off-the-shelf arms exist to **60.4 mm (`a/r_b = 0.671`)**
  with published hole ladders. The `a/r_b = 0.60` optimum is inside the stock set,
  not beyond it. The previous 12–32 mm ceiling was an artefact and is withdrawn.

- **Ball-joint housing OD.** Twelve M3-class parts publish it, spanning
  **9.0–13.0 mm (`0.100–0.144 r_b`)**. At 13.9 mm anchor-pair spacing, clearance
  runs **4.9 mm down to 0.9 mm**. The igus 13.0 mm figure is confirmed and is the
  worst case, not the only case. **`beta_p`'s outer bound now rests on a range, not
  a point.**

- **Servo body width.** 11.4–13.0 mm (`0.127–0.144 r_b`) published by Hitec and
  Savox.

- **Mounting-angle resolution.** 14.4° (25T) to 24.0° (15T micro); worst-case cost
  of a non-zero home angle is half a pitch, 7.2° to 12.0°.

- **Stall torque.** 0.127–0.52 N·m published across the class, against a 0.0265 N
  ball weight. Recorded so the ruling can be checked a third time by arithmetic
  rather than assertion.

### Out of scope by instruction

No part is recommended. No sweep range is chosen. No purchase, order, or account was
made — this was research only.

---

## 12. Horn extension — the spec the printed part has to meet (2026-09-16)

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
spline granularity is trimmed in firmware (§6), so nothing needs re-indexing.

---

## 13. Servo bracket — the spec (2026-09-18)

Design is the user's; this is the requirement list.  Written before the stiffness check so
the check has thresholds that came from the error budget rather than from
the CAD.

| # | constraint | why |
|---|---|---|
| B1 | **shaft axis parallel to the base plane, within 1°** | the IK's fixed minus branch rests on horizontal shafts (`docs/derivation.md` §8); 1° of cant is 0.10 mm of out-of-plane tip error, systematic rather than random |
| B2 | shaft centres on `r_b = 80 mm`, at the `beta = 10, delta = 0` positions from `base_ring` | the design of 2026-09-18 |
| B3 | case hangs below the shaft: body 6.75 above / 16 below the shaft centre, tabs 11.5 above / 20.7 below | matches `Body` in `performance.py`, which the clearance figures assume |
| B4 | compliance at the arm tip ≤ **~60 µm** under the load case below, shared with the horn extension's 54 µm (§12 H2) | bracket flex enters R2 exactly like gearing error; the split between bracket and horn is still to be allocated |
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

## 14. Top-plate anchors — the spec (2026-09-18)

Design is the user's; this is the requirement list.  Six rod-end mounts hanging
under the top plate, one per leg.

**Anchor positions** (ball centres), plate frame: origin at the plate centre
in the plane of the six ball centres, +x toward the midpoint of anchors 1 and
2, the same frame as the base-plate table in §13 at home.  `r_p = 70`,
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
(§12, rod-end seat), so **no insert spacers are needed at this end**: the rod
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
