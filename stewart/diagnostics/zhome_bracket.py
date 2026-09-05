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
                        npos = np.min(N, axis=1) > 0.0
                        ok = reach & npos
                        z_lo_cf = z_lower_closed_form(r_p, H_P)
                        if not ok.any():
                            empty.append((beta, beta_p, r_p, a, d))
                            rows.append(dict(beta=beta, beta_p=beta_p, r_p=r_p,
                                             a=a, d=d, lo=np.nan, hi=np.nan,
                                             cf=z_lo_cf, contiguous=True,
                                             nreach=int(reach.sum())))
                            continue
                        idx = np.flatnonzero(ok)
                        lo, hi = Z_GRID[idx[0]], Z_GRID[idx[-1]]
                        contiguous = (idx[-1] - idx[0] + 1) == idx.size
                        rows.append(dict(beta=beta, beta_p=beta_p, r_p=r_p,
                                         a=a, d=d, lo=lo, hi=hi, cf=z_lo_cf,
                                         contiguous=contiguous,
                                         nreach=int(reach.sum())))

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
        k = sum(1 for e in empty if e[4] == d)
        tot = sum(1 for r in rows if r["d"] == d)
        print(f"    d/r_b = {d:<5} : {k:>4} / {tot}")
    for a in A_RB:
        k = sum(1 for e in empty if e[3] == a)
        tot = sum(1 for r in rows if r["a"] == a)
        print(f"    a/r_b = {a:<5} : {k:>4} / {tot}")

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
