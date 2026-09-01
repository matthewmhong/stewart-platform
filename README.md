# stewart-platform

Design toolkit for a 6-RSS (rotary) Stewart platform: base / platform ring
geometry, inverse and forward kinematics, and drawing / animation helpers.

## Conventions

- **Millimetres and radians** internally.  Degrees appear only when printing a
  summary or building a servo command.
- **Every anchor array is `(3, 6)`; column `i` is leg `i`.**  Vectors are
  columns and rotations hit from the left: `R @ p`, never `p @ R`.  A `(6, 3)`
  array will broadcast in many places and silently apply the rotation
  transposed, so shapes and unit norms are checked at construction and failing
  legs are named 1-indexed.
- Python 3, **numpy + matplotlib only**.

## What is mine, what is scaffolding

**Mine (to write):**

| file | functions |
|------|-----------|
| `stewart/geometry.py`   | `platform_ring`, `make_geometry` |
| `stewart/kinematics.py` | `stage1`, `legs`, `arm_tips`, `ik`, `fk` |

**Scaffolding (done):**

- `stewart/geometry.py` - `Geometry` (frozen, self-validating), `base_ring`,
  `smoke_geometry`
- `stewart/kinematics.py` - `Unreachable`
- `stewart/plotting.py` - everything; operates only on points
- `stewart/roundtrip.py` - everything; `ik` / `fk` are passed in as arguments
- `demo.py`, `README.md`, `CLAUDE.md`, `requirements.txt`, `.gitignore`

## Stubs, in the order they pay off

1. **`platform_ring`**, then **`make_geometry`** - a real `Geometry` to design
   against.  Until they exist there is only `smoke_geometry()`, which is not a
   layout.
2. **`legs`** - `L_i = (R @ p_i + T) - b_i`.  Everything downstream is built on
   it; a couple of lines.
3. **`stage1`** - the branch-independent coefficients
   `A cos(a) + B sin(a) = P` and the amplitude `C = hypot(A, B)`.  This is
   where the reachability quantities live.
4. **`ik`** - per-leg solve, raising `Unreachable` when `|P| > C`.  First real
   servo output: `draw_pose(..., h=ik(...))` and `compare_branches` become
   meaningful.
5. **`arm_tips`** - tip points from angles; lets the plots show true arm / rod
   closure and so visually confirm `ik`.
6. **`fk`** - numerical, seeded deliberately off the true pose; turns
   `roundtrip.round_trip` into a real test.

## Run

    python -m pip install -r requirements.txt
    python demo.py            # writes demo.png, prints the round-trip report

## Layout

    stewart-platform/
      CLAUDE.md
      demo.py                 scaffolding - end-to-end smoke run
      README.md
      requirements.txt
      .gitignore
      stewart/
        __init__.py
        geometry.py           base_ring + Geometry done; platform_ring, make_geometry mine
        kinematics.py         all solvers stubbed; Unreachable done
        plotting.py           scaffolding - points in, drawing out
        roundtrip.py          scaffolding - pose -> ik -> fk -> pose
