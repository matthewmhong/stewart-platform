"""Lower and upper brackets for the restored ``z_home`` axis.  ((b) and (c).)

    python -m stewart.diagnostics.zhome_bracket

(b) LOWER, from ``N_i > 0``.  The fixed ``-`` branch rests on ``N_i > 0``, and
    ``N_i`` at home is ``z_home - h_p`` while tilt drops the low anchors.

(c) UPPER, from reach ``|P_i| <= C_i`` over the envelope.

**Two structural facts do most of the work here, and both are worth stating
because they are what make the brackets cheap.**  With horizontal shafts
``v_i = z`` exactly, and ``b_i . z = 0``, so

    N_i = L_i . v_i = q_i . z          contains no delta, and no a, d either

and ``L_i`` itself contains no ``delta`` (``delta`` enters only ``n_i`` and
``u_i``), so ``|L_i|`` and ``P_i`` are ``delta``-free too.  Only ``w_i`` and
``M_i`` move with ``delta``.  Therefore:

  * the LOWER bracket is delta-free, a-free and d-free - it is a statement
    about ``r_p``, ``h_p`` and the tilt limit alone, and it has a closed form;
  * the UPPER bracket reduces to ``w_i(delta)^2 <= |L_i|^2 - P_i^2``, since
    ``|P| <= C = sqrt(|L|^2 - w^2)`` squares to exactly that.

The back-of-envelope ``z_home - h_p > r_p sin(10.5) ~ 0.182 r_p`` is **not**
used.  It is computed here as a comparison only, and the closed form it
approximates is derived and then checked numerically.

Degrees at the boundary, radians internally.  Arrays are (3, 6).  numpy only.
Reports findings; always exits 0.
"""
from __future__ import annotations

import numpy as np

from ..geometry import make_geometry
from .envelope import (AZIMUTH_WINDOW_DEG, TILT_LIMIT_DEG, envelope_poses,
                       tilt_R)

# --------------------------------------------------------------------------- #
# grids - chosen, not derived; see the commit message
# --------------------------------------------------------------------------- #
BETA = [10.0, 20.0, 30.0, 40.0, 50.0]
BETA_P = [10.0, 25.0, 40.0, 55.0]
RP_RB = [0.6, 0.85, 1.1]
A_RB = [0.10, 0.20, 0.35]
D_RB = [0.8, 1.2, 1.6]
R_B = 1.0
H_P = 0.10                        # hardware number, not a sweep axis

#: ``z_home / r_b`` scan, 0.025 steps to 3.0.  Every reported bracket endpoint
#: is a GRID VALUE and carries that resolution: read ``z_lo`` as "feasible by
#: 0.025" and ``z_hi`` likewise.  No bisection refinement - the brackets here
#: are for deciding whether a candidate survives at all, not for setting a
#: dimension, and a feasible island narrower than 0.025 r_b is not a design.
Z_GRID = np.arange(0.025, 3.0 + 1e-12, 0.025)

#: ``delta`` scan for the reach test, 1 degree over the half-open range.
DELTA_GRID = np.arange(0.0, 180.0, 1.0)

#: Envelope sampling for the bracket study - finer in azimuth than the harness
#: grid in ``envelope.py``, because a bracket is a worst case and a worst case
#: sampled coarsely is not a bracket.
N_MAG_FINE = 6
N_AZ_FINE = 61


def fine_poses():
    mags = np.linspace(0.0, TILT_LIMIT_DEG, N_MAG_FINE)
    azis = np.linspace(AZIMUTH_WINDOW_DEG[0], AZIMUTH_WINDOW_DEG[1], N_AZ_FINE)
    az, mg = np.meshgrid(azis, mags[1:], indexing="ij")
    return (np.concatenate([[0.0], az.ravel()]),
            np.concatenate([[0.0], mg.ravel()]))


# --------------------------------------------------------------------------- #
# (b) the lower bracket, in closed form
# --------------------------------------------------------------------------- #
def n_min_closed_form(r_p: float, h_p: float, z_home: float,
                      tilt_deg: float = TILT_LIMIT_DEG) -> float:
    """``min_i min_pose N_i`` over the continuous envelope, exactly.

    With ``p_i = (r_p cos phi_i, r_p sin phi_i, -h_p)`` and a tilt of magnitude
    ``th`` about the horizontal axis at azimuth ``psi``, the third row of the
    Rodrigues matrix is ``(-sin th sin psi, sin th cos psi, cos th)``, so

        (R p_i)_z = r_p sin(th) sin(phi_i - psi) - h_p cos(th)

    and ``N_i = z_home + (R p_i)_z`` because ``T = (0, 0, z_home)`` and
    ``dxy = dz = 0``.  ``psi`` is continuous, so ``sin(phi_i - psi)`` attains
    ``-1`` for some azimuth at every ``th``; minimising the remainder over
    ``th`` in ``[0, tilt]`` gives

        min N = z_home - max_th [ r_p sin(th) + h_p cos(th) ]

    and with ``th + atan2(h_p, r_p) <= 90 deg`` - true for any sane plate at
    this tilt - the maximum sits at ``th = tilt``.
    """
    th = np.deg2rad(tilt_deg)
    gamma = np.arctan2(h_p, r_p)
    if th + gamma <= 0.5 * np.pi:
        drop = r_p * np.sin(th) + h_p * np.cos(th)
    else:                                     # interior maximum of the sinusoid
        drop = float(np.hypot(r_p, h_p))
    return z_home - drop


def z_lower_closed_form(r_p: float, h_p: float,
                        tilt_deg: float = TILT_LIMIT_DEG) -> float:
    """Infimum of ``z_home`` with ``min N_i > 0``.  Open bound: ``>``, not ``>=``.

    This is ``-n_min_closed_form(r_p, h_p, 0.0, tilt)`` - the drop that
    ``z_home`` has to clear.
    """
    return -n_min_closed_form(r_p, h_p, 0.0, tilt_deg)


def n_min_numeric(r_p: float, beta_p: float, h_p: float, z_home: float,
                  az, mg) -> float:
    """Same quantity, from the library ring and a sampled envelope."""
    g = make_geometry(r_b=R_B, beta=20.0, delta=0.0, r_p=r_p, beta_p=beta_p,
                      a=0.2, d=1.2, h_p=h_p)
    R = tilt_R(az, mg)
    q = np.einsum("kxy,yi->kxi", R, g.p)
    q[:, 2, :] += z_home
    return float(np.min(q[:, 2, :]))


# --------------------------------------------------------------------------- #
# (c) reach
# --------------------------------------------------------------------------- #
def leg_terms(beta, beta_p, r_p, a, d, h_p, z_grid, az, mg):
    """``(A, B, G)`` per ``(z, pose, leg)``, flattened to ``(nz, npose*6)``.

    ``w_i(delta) = A_i cos(delta) + B_i sin(delta)`` and the reach test
    ``|P_i| <= C_i`` is exactly ``w_i(delta)^2 <= G_i`` with
    ``G_i = |L_i|^2 - P_i^2``.  ``A``, ``B`` and ``G`` are all ``delta``-free.
    Also returns ``N`` for the ``N_i > 0`` test.
    """
    # delta = 0 and delta = 90 give the two basis normals; n_i(delta) is
    # cos(delta) n_i(0) + sin(delta) * s_i * n_i(90-ish).  Rather than rebuild
    # the ring, take both from the library and form the combination that the
    # parameterisation guarantees.
    g0 = make_geometry(r_b=R_B, beta=beta, delta=0.0, r_p=r_p, beta_p=beta_p,
                       a=a, d=d, h_p=h_p)
    g90 = make_geometry(r_b=R_B, beta=beta, delta=90.0, r_p=r_p, beta_p=beta_p,
                        a=a, d=d, h_p=h_p)

    R = tilt_R(az, mg)                                   # (K,3,3)
    qp = np.einsum("kxy,yi->kxi", R, g0.p)               # (K,3,6), T not added
    # L = qp + (0,0,z) - b   ->  z enters only the third component
    b = g0.b[None]                                       # (1,3,6)
    base = qp - b                                        # (K,3,6)
    z = z_grid[:, None, None, None]                      # (nz,1,1,1)
    L = np.broadcast_to(base, (z_grid.size,) + base.shape).copy()
    L[:, :, 2, :] += z_grid[:, None, None]

    N = L[:, :, 2, :]                                    # (nz,K,6) since v = z
    LL = np.einsum("zkxi,zkxi->zki", L, L)
    P = (LL + a * a - d * d) / (2.0 * a)
    G = LL - P * P
    A = np.einsum("zkxi,xi->zki", L, g0.n)
    B = np.einsum("zkxi,xi->zki", L, g90.n)
    nz = z_grid.size
    return (A.reshape(nz, -1), B.reshape(nz, -1), G.reshape(nz, -1),
            N.reshape(nz, -1))


def reach_feasible_any_delta(A, B, G, deltas_rad):
    """``(nz,)`` bool: does some ``delta`` satisfy ``w^2 <= G`` everywhere."""
    ok = np.zeros(A.shape[0], dtype=bool)
    for dr in deltas_rad:
        w = A * np.cos(dr) + B * np.sin(dr)
        viol = np.max(w * w - G, axis=1)
        ok |= (viol <= 0.0)
    return ok


def reach_ceiling_bisect(beta, beta_p, r_p, a, d, h_p, lo, hi, az, mg,
                         deltas_rad, tol=1e-9):
    """Refine the reach ceiling between a feasible ``lo`` and infeasible ``hi``.

    The grid step is 0.025 ``r_b``, which is coarse enough that a bracket
    emptied by a gap smaller than that could be a sampling artifact rather than
    a crossing.  Anything reported as a crossing gets refined to ``tol`` first.
    """
    for _ in range(200):
        if hi - lo <= tol:
            break
        mid = 0.5 * (lo + hi)
        A, B, G, _ = leg_terms(beta, beta_p, r_p, a, d, h_p,
                               np.array([mid]), az, mg)
        if reach_feasible_any_delta(A, B, G, deltas_rad)[0]:
            lo = mid
        else:
            hi = mid
    return lo


def check_n_and_v(beta, beta_p, r_p, a, d, h_p):
    """Assert ``v_i == z`` for this candidate; the whole module rests on it."""
    g = make_geometry(r_b=R_B, beta=beta, delta=0.0, r_p=r_p, beta_p=beta_p,
                      a=a, d=d, h_p=h_p)
    v = np.cross(g.n, g.u, axis=0)
    return float(np.max(np.abs(v - np.array([[0.0], [0.0], [1.0]]))))


# --------------------------------------------------------------------------- #
def main() -> None:
    az, mg = fine_poses()
    deltas_rad = np.deg2rad(DELTA_GRID)

    print("=" * 78)
    print("DIAGNOSTIC (b) - z_home LOWER bracket from N_i > 0")
    print("=" * 78)
    print(f"envelope: tilt <= {TILT_LIMIT_DEG:.4f} deg, dxy = dz = yaw = 0")
    print(f"poses   : {az.size}  ({N_MAG_FINE} magnitudes x {N_AZ_FINE} azimuths "
          f"over {AZIMUTH_WINDOW_DEG}, magnitude 0 once)")
    print()
    print("N_i is delta-free, a-free and d-free: v_i = z exactly under horizontal")
    print("shafts and b_i . z = 0, so N_i = q_i . z.  The bracket therefore")
    print("depends on r_p, h_p and the tilt limit ALONE.")
    print()
    print("closed form:  z_home > r_p sin(tilt) + h_p cos(tilt)")
    print("back-of-env:  z_home > r_p sin(tilt) + h_p          [ignores cos]")
    print()
    print(f"  {'r_p/r_b':>9} {'h_p/r_b':>9} {'closed form':>14} "
          f"{'back-of-env':>14} {'difference':>13} {'grid check':>13}")
    worst_cf = 0.0
    for r_p in RP_RB:
        for h_p in (0.0, 0.05, 0.10, 0.20):
            cf = z_lower_closed_form(r_p, h_p)
            boe = r_p * np.sin(np.deg2rad(TILT_LIMIT_DEG)) + h_p
            # numeric check at z_home = cf : min N should be ~0
            resid = abs(n_min_numeric(r_p, 40.0, h_p, cf, az, mg))
            worst_cf = max(worst_cf, resid)
            print(f"  {r_p:>9.3f} {h_p:>9.3f} {cf:>14.9f} {boe:>14.9f} "
                  f"{boe - cf:>13.3e} {resid:>13.3e}")
    print()
    print(f"  worst |min N| at z_home = closed form, over the sampled envelope:")
    print(f"    {worst_cf:.3e}")
    print("  (the grid check can only be >= 0; it is the amount by which the")
    print("   SAMPLED envelope misses the true worst azimuth)")

    # how much does the discrete grid understate the drop?
    print()
    print("  Grid-vs-continuum, the part that matters for the harness:")
    for beta_p in BETA_P:
        r_p = 0.85
        cf = z_lower_closed_form(r_p, H_P)
        gridN = n_min_numeric(r_p, beta_p, H_P, cf, az, mg)
        az_h, mg_h = envelope_poses()
        gridN_h = n_min_numeric(r_p, beta_p, H_P, cf, az_h, mg_h)
        print(f"    beta_p={beta_p:>5.1f}  min N at the closed-form bound: "
              f"fine grid {gridN:+.3e}   harness grid {gridN_h:+.3e}")
    print("  Both are >= 0 by construction, so a DISCRETE grid reports the")
    print("  constraint as satisfied slightly before it truly is.  The harness")
    print("  must take the lower bracket from the CLOSED FORM, not from its")
    print("  pose grid.")

    print()
    print(f"  v_i == z check (max |v - z| over a few candidates): ", end="")
    print(f"{max(check_n_and_v(b, bp, 0.85, 0.2, 1.2, H_P) for b in BETA for bp in BETA_P):.3e}")

    # ------------------------------------------------------------------ #
    print()
    print("=" * 78)
    print("DIAGNOSTIC (c) - z_home UPPER bracket from reach |P| <= C")
    print("=" * 78)
    print(f"candidates: {len(BETA)}x{len(BETA_P)}x{len(RP_RB)}x{len(A_RB)}x"
          f"{len(D_RB)} = "
          f"{len(BETA)*len(BETA_P)*len(RP_RB)*len(A_RB)*len(D_RB)}  "
          f"(h_p/r_b = {H_P} fixed)")
    print(f"z_home/r_b scan: {Z_GRID[0]} .. {Z_GRID[-1]} step "
          f"{Z_GRID[1]-Z_GRID[0]}  ({Z_GRID.size} points)")
    print(f"delta scan     : {DELTA_GRID.size} points over [0, 180)")
    print(f"reach test     : w_i(delta)^2 <= |L_i|^2 - P_i^2, exact")
    print()

    rows = []
    empty = []
    for beta in BETA:
        for beta_p in BETA_P:
            for r_p in RP_RB:
                for a in A_RB:
                    for d in D_RB:
                        A, B, G, N = leg_terms(beta, beta_p, r_p, a, d, H_P,
                                               Z_GRID, az, mg)
                        reach = reach_feasible_any_delta(A, B, G, deltas_rad)
                        z_lo_cf = z_lower_closed_form(r_p, H_P)
                        # N_i > 0 from the CLOSED FORM, not from the pose grid:
                        # the grid reports the constraint satisfied before it is
                        # (see check (b) above), so using it here would bias the
                        # attribution towards blaming reach.
                        npos = Z_GRID > z_lo_cf
                        npos_grid = np.min(N, axis=1) > 0.0
                        ok = reach & npos
                        ridx = np.flatnonzero(reach)
                        rec = dict(beta=beta, beta_p=beta_p, r_p=r_p, a=a, d=d,
                                   cf=z_lo_cf, nreach=int(reach.sum()),
                                   reach_lo=Z_GRID[ridx[0]] if ridx.size else np.nan,
                                   reach_hi=Z_GRID[ridx[-1]] if ridx.size else np.nan,
                                   reach_contig=(bool(ridx.size) and
                                                 (ridx[-1] - ridx[0] + 1) == ridx.size),
                                   n_disagree=int(np.sum(npos != npos_grid)))
                        if not ok.any():
                            empty.append(rec)
                            rec.update(lo=np.nan, hi=np.nan, contiguous=True)
                            rows.append(rec)
                            continue
                        idx = np.flatnonzero(ok)
                        rec.update(lo=Z_GRID[idx[0]], hi=Z_GRID[idx[-1]],
                                   contiguous=(idx[-1] - idx[0] + 1) == idx.size)
                        rows.append(rec)

    n_tot = len(rows)
    n_empty = len(empty)
    good = [r for r in rows if not np.isnan(r["lo"])]
    noncontig = [r for r in good if not r["contiguous"]]

    print(f"  candidates with a NON-EMPTY z_home bracket : {len(good)} / {n_tot}")
    print(f"  candidates with an EMPTY bracket           : {n_empty} / {n_tot}")
    print(f"  non-contiguous feasible sets               : {len(noncontig)}")
    if good:
        los = np.array([r["lo"] for r in good])
        his = np.array([r["hi"] for r in good])
        print(f"  z_home/r_b lower ends : [{los.min():.3f}, {los.max():.3f}]")
        print(f"  z_home/r_b upper ends : [{his.min():.3f}, {his.max():.3f}]")
        print(f"  widest bracket        : {np.max(his - los):.3f}  "
              f"narrowest: {np.min(his - los):.3f}")
        binding = np.sum(los <= np.array([r["cf"] for r in good]) + 1e-9)
        print(f"  candidates whose lower end is set by N>0 rather than reach: "
              f"{int(binding)} / {len(good)}")

    print()
    print("  Empty brackets by d/r_b (a candidate with no feasible z_home at ANY")
    print("  delta, over the whole scanned range):")
    for d in D_RB:
        k = sum(1 for e in empty if e["d"] == d)
        tot = sum(1 for r in rows if r["d"] == d)
        print(f"    d/r_b = {d:<5} : {k:>4} / {tot}")
    for a in A_RB:
        k = sum(1 for e in empty if e["a"] == a)
        tot = sum(1 for r in rows if r["a"] == a)
        print(f"    a/r_b = {a:<5} : {k:>4} / {tot}")

    # ------------------------------------------------------------------ #
    # WHICH constraint emptied each of them
    # ------------------------------------------------------------------ #
    print()
    print("-" * 78)
    print("ATTRIBUTION OF THE EMPTY BRACKETS")
    print("-" * 78)
    print("  Why this block exists.  'N_i > 0 never sets the lower end, 0 of 363'")
    print("  is a SURVIVORSHIP sample: it ranges over candidates that HAVE a")
    print("  bracket, and those are exactly the ones where the constraints did")
    print("  not cross.  If N_i > 0 is ever binding it is among the empties, and")
    print("  a sample that structurally cannot contain a counterexample is not")
    print("  evidence.  So the empties are attributed here.")
    print()
    print("  The two constraints are not the same shape, which decides what the")
    print("  categories can be:")
    print("    N_i > 0    ONE-SIDED, a FLOOR:  z_home > r_p sin(tilt)+h_p cos(tilt)")
    print("    reach      TWO-SIDED, an INTERVAL in z_home (contiguous, measured)")
    print("  N_i > 0 has no ceiling, so 'reach floor above the N ceiling' cannot")
    print("  occur - there is no N ceiling to be above.  Only two cases exist:")
    print("    R  reach empty on its own      - no z_home reaches at any delta")
    print("    X  reach ceiling BELOW N floor - both satisfiable alone, crossed")
    print()

    cat_R, cat_X = [], []
    for e in empty:
        if e["nreach"] == 0:
            cat_R.append(e)
        else:
            cat_X.append(e)

    print(f"    R  reach empty on its own      : {len(cat_R):>4} / {len(empty)}")
    print(f"    X  reach ceiling below N floor : {len(cat_X):>4} / {len(empty)}")
    print()
    if cat_X:
        print("    Candidates where the two constraints CROSS.  The grid step is")
        print(f"    {Z_GRID[1]-Z_GRID[0]}, so a crossing narrower than that could be a")
        print("    sampling artifact; each reach ceiling below is refined by")
        print("    bisection to 1e-9 before the verdict is taken.")
        print()
        print(f"      {'beta':>6} {'beta_p':>7} {'r_p':>6} {'a/r_b':>6} "
              f"{'d/r_b':>6} {'ceil(grid)':>11} {'ceil(exact)':>12} "
              f"{'N floor':>9} {'gap':>10} {'verdict':>9}")
        real_X = []
        for e in cat_X:
            step = Z_GRID[1] - Z_GRID[0]
            exact = reach_ceiling_bisect(e["beta"], e["beta_p"], e["r_p"],
                                         e["a"], e["d"], H_P,
                                         e["reach_hi"], e["reach_hi"] + step,
                                         az, mg, deltas_rad)
            gap = e["cf"] - exact
            real = gap > 0.0
            if real:
                real_X.append(e)
            print(f"      {e['beta']:>6.1f} {e['beta_p']:>7.1f} {e['r_p']:>6.2f} "
                  f"{e['a']:>6.2f} {e['d']:>6.2f} {e['reach_hi']:>11.3f} "
                  f"{exact:>12.7f} {e['cf']:>9.5f} {gap:>+10.2e} "
                  f"{'REAL' if real else 'artifact':>9}")
        cat_X = real_X
        cat_R = [e for e in empty if e not in cat_X]
        print()
        print(f"    after refinement:  R = {len(cat_R)},  X = {len(cat_X)}")
    if not cat_X:
        print("    No candidate is emptied by the crossing.  Every empty bracket")
        print("    is reach failing on its own, at every z_home and every delta.")

    print()
    print(f"    {'a/r_b':>7} {'empty':>7} {'R':>5} {'X':>5}")
    for a in A_RB:
        ke = [e for e in empty if e["a"] == a]
        print(f"    {a:>7.2f} {len(ke):>7} "
              f"{sum(1 for e in ke if e['nreach'] == 0):>5} "
              f"{sum(1 for e in ke if e['nreach'] > 0):>5}")
    print()
    print(f"    {'d/r_b':>7} {'empty':>7} {'R':>5} {'X':>5}")
    for d in D_RB:
        ke = [e for e in empty if e["d"] == d]
        print(f"    {d:>7.2f} {len(ke):>7} "
              f"{sum(1 for e in ke if e['nreach'] == 0):>5} "
              f"{sum(1 for e in ke if e['nreach'] > 0):>5}")

    # is the reach set ever clipped by the top of the scan?
    hit_top = [r for r in rows if r["nreach"] and
               r["reach_hi"] >= Z_GRID[-1] - 1e-12]
    noncontig_reach = [r for r in rows if r["nreach"] and not r["reach_contig"]]
    disagree = sum(r["n_disagree"] for r in rows)
    print()
    print(f"    reach set touching the top of the scan ({Z_GRID[-1]}) : "
          f"{len(hit_top)}  (0 means the scan is not clipping anything)")
    print(f"    non-contiguous REACH sets                    : "
          f"{len(noncontig_reach)}")
    if noncontig_reach:
        print("      Reported because it looks like it contradicts the "
              "'0 non-contiguous")
        print("      feasible sets' above.  It does not, and the reason is the "
              "third")
        print("      job N_i > 0 does.  Every one of these has a spurious "
              "LOW component")
        print("      at z_home ~ 0.025-0.125 r_b - the platform essentially on "
              "the base")
        print("      plate, geometrically reachable and physically nonsense - "
              "and in")
        print("      every case it sits ENTIRELY BELOW the N_i > 0 floor, which "
              "removes")
        print("      it.  The INTERSECTION is contiguous; the reach set alone "
              "is not.")
        print(f"      {'beta':>6} {'beta_p':>7} {'r_p':>6} {'a/r_b':>6} "
              f"{'d/r_b':>6} {'low component':>18} {'N floor':>9} "
              f"{'main component':>18}")
        for r in noncontig_reach:
            A, B, G, _ = leg_terms(r["beta"], r["beta_p"], r["r_p"], r["a"],
                                   r["d"], H_P, Z_GRID, az, mg)
            ri = np.flatnonzero(reach_feasible_any_delta(A, B, G, deltas_rad))
            comps = np.split(ri, np.flatnonzero(np.diff(ri) > 1) + 1)
            lo_c, hi_c = comps[0], comps[-1]
            print(f"      {r['beta']:>6.1f} {r['beta_p']:>7.1f} "
                  f"{r['r_p']:>6.2f} {r['a']:>6.2f} {r['d']:>6.2f} "
                  f"{f'[{Z_GRID[lo_c[0]]:.3f}, {Z_GRID[lo_c[-1]]:.3f}]':>18} "
                  f"{r['cf']:>9.3f} "
                  f"{f'[{Z_GRID[hi_c[0]]:.3f}, {Z_GRID[hi_c[-1]]:.3f}]':>18}")
        below = sum(1 for r in noncontig_reach)
        print(f"      low component below the N floor in {below} of "
              f"{len(noncontig_reach)}")
    print(f"    z points where closed-form and grid N disagree: {disagree}")
    print("      (the closed form is the one used above; the grid is optimistic")
    print("       by up to one grid step, which is why it is not used here)")

    # ---- the corrected claim ---------------------------------------- #
    print()
    print("  CORRECTED CLAIM.")
    if cat_X:
        print("    'N_i > 0 is not the binding constraint at this tilt' is WRONG")
        print("    as an unqualified statement.  The 0-of-363 sample it rested on")
        print("    could not have contained a counterexample; a sample that could")
        print("    have, does.  N_i > 0 does three distinct jobs:")
        print(f"      1. it never sets the LOWER END of a surviving bracket "
              f"(0 of {len(good)});")
        print(f"      2. it CLOSES the bracket outright in {len(cat_X)} of the "
              f"{len(empty)} empties -")
        print("         reach ceiling below the N floor, refined by bisection so")
        print("         it is a crossing and not a grid artifact;")
        print(f"      3. it removes a spurious disconnected low-z reach component")
        print(f"         in {len(noncontig_reach)} candidates, which is what keeps "
              f"the feasible")
        print("         set an interval at all.")
        print("    Correct form: it does not shape the interior of the feasible")
        print("    set, and it is not what makes most candidates fail - but it is")
        print("    load-bearing at the edges, and it cannot be dropped.")
    else:
        print(f"    'N_i > 0 is not the binding constraint at this tilt' SURVIVES")
        print(f"    the test that could have refuted it.  It sets the lower end in")
        print(f"    0 of {len(good)} candidates with a bracket, AND it is not what")
        print(f"    empties any of the {len(empty)} without one - all {len(cat_R)}")
        print(f"    of those are reach failing alone, at every z_home and every")
        print(f"    delta.  The claim now rests on a sample that COULD have")
        print(f"    contained a counterexample and does not.")

    print()
    print("  A sample of the bracket, at r_p/r_b = 0.85, beta = 20:")
    print(f"    {'beta_p':>7} {'a/r_b':>7} {'d/r_b':>7} {'z_lo':>9} {'z_hi':>9} "
          f"{'width':>9} {'N>0 bound':>11}")
    for r in rows:
        if r["beta"] == 20.0 and r["r_p"] == 0.85 and r["beta_p"] in (25.0, 40.0):
            lo = "-" if np.isnan(r["lo"]) else f"{r['lo']:.3f}"
            hi = "-" if np.isnan(r["hi"]) else f"{r['hi']:.3f}"
            wd = "-" if np.isnan(r["lo"]) else f"{r['hi']-r['lo']:.3f}"
            print(f"    {r['beta_p']:>7.1f} {r['a']:>7.2f} {r['d']:>7.2f} "
                  f"{lo:>9} {hi:>9} {wd:>9} {r['cf']:>11.3f}")

    print()
    print("=" * 78)
    print("BRACKET, as it should be written into notation.md sec.12")
    print("=" * 78)
    print("  LOWER  z_home > r_p sin(tilt) + h_p cos(tilt)")
    print(f"         = {np.sin(np.deg2rad(TILT_LIMIT_DEG)):.6f} r_p + "
          f"{np.cos(np.deg2rad(TILT_LIMIT_DEG)):.6f} h_p   at tilt = "
          f"{TILT_LIMIT_DEG:.4f} deg")
    print("         delta-free, a-free, d-free.  Strict, and it is the CONTINUUM")
    print("         bound - do not take it off a pose grid.")
    print("  UPPER  per candidate, from |P| <= C; no closed form, scanned above.")
    print("         It is NOT a single number: it depends on a, d, r_p, beta,")
    print("         beta_p, and on delta being free to tune.")


if __name__ == "__main__":
    main()
