"""Two numerical diagnostics on the ``z_home`` assembly datum.

    python -m stewart.diagnostics.zhome_datum

Check A - leg-independence of the flat-arm distance ``|g_i - a u_i|``, and that
          distance against a closed form in the input parameters.
Check B - ``z_home`` as a function of ``delta`` over the full circle, on the
          432-point grid that produced the "8 with no valid z_home" claim.

**Independence discipline.**  ``b_i``, ``u_i``, ``n_i`` and ``p_i`` are taken from
:func:`stewart.geometry.make_geometry`.  Nothing here rebuilds a ring from
``theta_i = 120 floor(i/2) + s_i beta``.  A diagnostic that re-derived the rings
from the derivation's own formulas would be testing the algebra against itself and
would verify nothing.  The only quantities this module forms independently are the
closed forms under test, written in terms of the *input parameters*
(``r_b, r_p, beta, beta_p, delta, a, d``), never in terms of the anchors.

Degrees at the boundary, radians internally.  Arrays are (3, 6).  numpy only.
Reports findings; always exits 0.
"""
from __future__ import annotations

import numpy as np

from ..geometry import make_geometry
from ..kinematics import arm_tips, stage1

# --------------------------------------------------------------------------- #
# Check A grid (as specified for this task)
# --------------------------------------------------------------------------- #
A_BETA    = [10.0, 20.0, 30.0, 40.0, 50.0]
A_BETA_P  = [5.0, 20.0, 40.0, 60.0, 80.0]
A_DELTA   = [0.0, 30.0, 60.0, 90.0, 120.0, 150.0]
A_RP_RB   = [0.6, 1.0, 1.4]
A_A_RB    = [0.15, 0.30]
A_R_B     = 100.0
A_C_P     = 10.0
A_D_NOMINAL = 120.0          # unused by check A; make_geometry requires d > 0

# --------------------------------------------------------------------------- #
# Check B grid - RECOVERED VERBATIM, not reconstructed.
# Source: scratchpad/branch_check.py, part1() block "[1c] discriminant", the run
# of 2026-09-03 that produced "8 of 432 ... all at d/r_b = 0.8".  That script was
# never committed; see this module's report header.  Held delta = 40 there; here
# delta becomes the swept variable.
# --------------------------------------------------------------------------- #
B_BETA    = [10.0, 20.0, 30.0, 40.0]
B_BETA_P  = [10.0, 25.0, 40.0, 55.0]
B_RP_RB   = [0.6, 0.85, 1.2]
B_A_RB    = [0.10, 0.20, 0.35]
B_D_RB    = [0.8, 1.2, 1.6]
B_R_B     = 1.0
B_C_P     = 0.1
B_DELTA_ORIGINAL = 40.0      # the value held fixed in the original run


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def _geom(r_b, beta, delta, r_p_rb, beta_p, a_rb, d_rb, c_p):
    """Geometry from the library.  Returns None if the library rejects it."""
    try:
        return make_geometry(r_b, beta, delta, r_p_rb * r_b, beta_p,
                             a_rb * r_b, d_rb * r_b, c_p=c_p)
    except ValueError:
        return None


def _flat_arm_dist(geom):
    """``|g_i - a u_i|`` per leg, from library anchors only.

    ``g_i = q_i[:2] - b_i[:2]`` with ``q_i[:2] = p_i[:2]`` at ``R = I`` and
    horizontal ``T = 0``.  Purely horizontal, so no ``z_home`` enters.
    """
    g = np.zeros((3, 6))
    g[:2] = geom.p[:2] - geom.b[:2]
    return np.linalg.norm(g - geom.a * geom.u, axis=0), g


def _closed_form_sq(r_b, r_p, beta, beta_p, delta, a):
    """``|g|^2 + a^2 - 2a(g.u)`` in input parameters.  Returns (value, |g|^2, g.u)."""
    A = np.deg2rad(beta_p - beta)
    dl = np.deg2rad(delta)
    g2 = r_p ** 2 + r_b ** 2 - 2.0 * r_p * r_b * np.cos(A)
    gu = r_b * np.cos(dl) - r_p * np.cos(A - dl)
    return g2 + a ** 2 - 2.0 * a * gu, g2, gu


def _delta_G_deg(r_b, r_p, beta, beta_p):
    """``arg(r_p e^{iA} - r_b)`` in degrees, wrapped to [0, 360)."""
    A = np.deg2rad(beta_p - beta)
    return np.rad2deg(np.arctan2(r_p * np.sin(A), r_p * np.cos(A) - r_b)) % 360.0


# --------------------------------------------------------------------------- #
# CHECK A
# --------------------------------------------------------------------------- #
def check_flat_arm_leg_independence():
    """Leg-independence of ``|g_i - a u_i|`` and agreement with the closed form."""
    print("=" * 78)
    print("CHECK A - flat-arm distance |g_i - a u_i|")
    print("=" * 78)
    print("anchors from make_geometry; closed form written in input parameters only")
    print(f"grid: beta {A_BETA} x beta_p {A_BETA_P} x delta {A_DELTA}")
    print(f"      r_p/r_b {A_RP_RB} x a/r_b {A_A_RB},  r_b = {A_R_B}, c_p = {A_C_P}")

    worst_ind = (-1.0, None)     # residual 1: leg-independence
    worst_cf = (-1.0, None)      # residual 2: closed form vs repo geometry
    n_ok = n_rejected = 0
    rejected_beta_p = set()
    max_uz = 0.0

    for beta in A_BETA:
        for beta_p in A_BETA_P:
            for delta in A_DELTA:
                for rp in A_RP_RB:
                    for a_rb in A_A_RB:
                        g = _geom(A_R_B, beta, delta, rp, beta_p, a_rb,
                                  A_D_NOMINAL / A_R_B, A_C_P)
                        if g is None:
                            n_rejected += 1
                            rejected_beta_p.add(beta_p)
                            continue
                        n_ok += 1
                        max_uz = max(max_uz, float(np.max(np.abs(g.u[2]))))
                        dist, _ = _flat_arm_dist(g)
                        loc = dict(beta=beta, beta_p=beta_p, delta=delta,
                                   rp=rp, a_rb=a_rb)

                        r1 = float(dist.max() - dist.min())
                        if r1 > worst_ind[0]:
                            worst_ind = (r1, dict(loc, scale=float(dist.mean())))

                        cf, _, _ = _closed_form_sq(A_R_B, rp * A_R_B, beta,
                                                   beta_p, delta, a_rb * A_R_B)
                        r2 = float(np.max(np.abs(dist ** 2 - cf)))
                        if r2 > worst_cf[0]:
                            worst_cf = (r2, dict(loc, scale=abs(float(cf))))

    print(f"\nbuilt {n_ok} of {n_ok + n_rejected} grid points; "
          f"{n_rejected} rejected by the library guard")
    if rejected_beta_p:
        print(f"  !! beta_p values refused by platform_ring's 0 < beta_p < 60 guard: "
              f"{sorted(rejected_beta_p)}")
        print("     These are grid points the task asked for that the repo cannot "
              "build.\n     Reported, not worked around.")
    print(f"max |u_i . z| over all built geometries = {max_uz:.3e}  (0 => u horizontal)")

    print(f"\n{'residual':<52} {'max':>12}  {'relative':>11}")
    print("-" * 78)
    r1, l1 = worst_ind
    r2, l2 = worst_cf
    print(f"{'1. leg-independence  max_i(.) - min_i(.)':<52} {r1:>12.3e}  "
          f"{r1 / l1['scale']:>11.3e}")
    print(f"{'2. |g_i - a u_i|^2 vs closed form':<52} {r2:>12.3e}  "
          f"{r2 / l2['scale']:>11.3e}")
    print("-" * 78)
    print(f"argmax 1: beta={l1['beta']} beta_p={l1['beta_p']} delta={l1['delta']} "
          f"r_p/r_b={l1['rp']} a/r_b={l1['a_rb']}")
    print(f"argmax 2: beta={l2['beta']} beta_p={l2['beta_p']} delta={l2['delta']} "
          f"r_p/r_b={l2['rp']} a/r_b={l2['a_rb']}")
    verdict = "AGREE" if max(r1 / l1["scale"], r2 / l2["scale"]) < 1e-12 else "DISAGREE"
    print(f"\nverdict: {verdict} (threshold 1e-12 relative, not tuned)")
    return r1, r2


# --------------------------------------------------------------------------- #
# CHECK B
# --------------------------------------------------------------------------- #
def _zhome_sq(d, g2, a, gu):
    """``(z_home - c_p)^2 = d^2 - |g|^2 - a^2 + 2a(g.u)``."""
    return d ** 2 - g2 - a ** 2 + 2.0 * a * gu


def sweep_zhome_delta():
    """``z_home(delta)`` over [0, 360) on the recovered 432-point grid."""
    print("\n" + "=" * 78)
    print("CHECK B - z_home(delta) over the full circle, recovered 432-point grid")
    print("=" * 78)

    deltas = np.arange(0.0, 360.0, 1.0)
    dl = np.deg2rad(deltas)
    rows = []

    for beta in B_BETA:
        for beta_p in B_BETA_P:
            for rp in B_RP_RB:
                for a_rb in B_A_RB:
                    for d_rb in B_D_RB:
                        r_b, a, d = B_R_B, a_rb * B_R_B, d_rb * B_R_B
                        r_p = rp * B_R_B
                        A = np.deg2rad(beta_p - beta)
                        g2 = r_p ** 2 + r_b ** 2 - 2 * r_p * r_b * np.cos(A)
                        gu = r_b * np.cos(dl) - r_p * np.cos(A - dl)
                        sq = _zhome_sq(d, g2, a, gu)
                        feas = sq > 0.0
                        lower, upper = feas[:180], feas[180:]
                        dG = _delta_G_deg(r_b, r_p, beta, beta_p)

                        z = np.full_like(sq, np.nan)
                        z[feas] = B_C_P + np.sqrt(sq[feas])
                        if feas.any():
                            zf = z[feas]
                            swing = float((zf.max() - zf.min()) / zf.mean())
                        else:
                            swing = np.nan

                        # feasible arc, as a wrapped [start, end) in degrees
                        if feas.all():
                            arc = (0.0, 360.0)
                        elif feas.any():
                            idx = np.flatnonzero(~feas)
                            # infeasible run is contiguous mod 360 (single dip)
                            gaps = np.diff(idx)
                            brk = np.flatnonzero(gaps > 1)
                            if brk.size == 0:
                                arc = (float(deltas[idx[-1]] + 1) % 360.0,
                                       float(deltas[idx[0]]))
                            else:
                                arc = (float(deltas[idx[brk[0]]] + 1) % 360.0,
                                       float(deltas[idx[brk[0] + 1]]))
                        else:
                            arc = (np.nan, np.nan)

                        # the original run held delta = 40
                        i40 = int(B_DELTA_ORIGINAL)
                        rows.append(dict(
                            arc=arc,
                            beta=beta, beta_p=beta_p, rp=rp, a_rb=a_rb, d_rb=d_rb,
                            lower=bool(lower.any()), upper=bool(upper.any()),
                            n_feas=int(feas.sum()), dG=float(dG),
                            dG_lower=bool(dG < 180.0), swing=swing,
                            raw_swing=float(4.0 * a * np.sqrt(g2)),
                            feas_at_40=bool(feas[i40]),
                        ))

    n = len(rows)
    print(f"grid points: {n}   (expect 432)")

    # -- provenance cross-check: reproduce the original 8 at delta = 40 -------
    fail40 = [r for r in rows if not r["feas_at_40"]]
    print(f"\n[B0] reproduce the original claim at delta = {B_DELTA_ORIGINAL:.0f}")
    print(f"     combinations with no valid z_home : {len(fail40)}   (claim: 8)")
    if fail40:
        dset = sorted({r["d_rb"] for r in fail40})
        print(f"     d/r_b values among them          : {dset}   (claim: all 0.8)")
        print(f"\n     {'beta':>5} {'beta_p':>7} {'r_p/r_b':>8} {'a/r_b':>6} {'d/r_b':>6}"
              f" {'feasible [180,360)?':>20} {'delta_G':>9}")
        for r in fail40:
            print(f"     {r['beta']:>5.0f} {r['beta_p']:>7.0f} {r['rp']:>8.2f} "
                  f"{r['a_rb']:>6.2f} {r['d_rb']:>6.1f} {str(r['upper']):>20} "
                  f"{r['dG']:>9.3f}")

    # -- Q1 ------------------------------------------------------------------
    rescued = [r for r in fail40 if r["upper"]]
    print(f"\n[B1] of the {len(fail40)} failures, feasible on the other half-circle: "
          f"{len(rescued)}")

    # -- half-circle classification -----------------------------------------
    both = [r for r in rows if r["lower"] and r["upper"]]
    only_lo = [r for r in rows if r["lower"] and not r["upper"]]
    only_hi = [r for r in rows if r["upper"] and not r["lower"]]
    neither = [r for r in rows if not r["lower"] and not r["upper"]]
    print(f"\n[B2] feasibility by half-circle over all {n}:")
    print(f"     both halves      : {len(both)}")
    print(f"     [0,180) only     : {len(only_lo)}")
    print(f"     [180,360) only   : {len(only_hi)}")
    print(f"     neither          : {len(neither)}")

    mismatch = [r for r in rows
                if (r["lower"] or r["upper"])
                and not (r["dG_lower"] and r["lower"] or
                         (not r["dG_lower"]) and r["upper"])]
    print(f"\n     feasible half differs from the half containing delta_G: "
          f"{len(mismatch)} of {n}")
    print("     (delta_G is where (z_home-c_p)^2 is MINIMISED, so a nonzero count")
    print("      is expected and is not by itself evidence that [0,180) is unsafe)")

    lo_insufficient = [r for r in rows if r["upper"] and not r["lower"]]
    print(f"\n     combinations feasible ONLY outside [0,180): {len(lo_insufficient)}")
    print("     ^ this is the number that decides whether [0,180) is a sufficient")
    print("       delta range once the flat-arm datum is imposed")

    # -- Q3 : does the answer move with the grid coordinates? ---------------
    print(f"\n[B3] does the half-circle answer vary across the grid?")
    for key, vals in (("d/r_b", B_D_RB), ("a/r_b", B_A_RB), ("r_p/r_b", B_RP_RB),
                      ("beta", B_BETA), ("beta_p", B_BETA_P)):
        k = {"d/r_b": "d_rb", "a/r_b": "a_rb", "r_p/r_b": "rp",
             "beta": "beta", "beta_p": "beta_p"}[key]
        cells = []
        for v in vals:
            sub = [r for r in rows if r[k] == v]
            cells.append(f"{v}:both={sum(r['lower'] and r['upper'] for r in sub)}"
                         f"/none={sum(not r['lower'] and not r['upper'] for r in sub)}")
        print(f"     {key:<9} " + "  ".join(cells))

    dG_lo = sum(r["dG_lower"] for r in rows)
    print(f"\n     delta_G in [0,180): {dG_lo} of {n}    in [180,360): {n - dG_lo}")

    # -- B5 : exactly where the half-circle answer changes -------------------
    odd = [r for r in rows if not (r["lower"] and r["upper"])]
    print(f"\n[B5] every combination that is NOT feasible on both halves ({len(odd)}):")
    print(f"     {'beta':>5} {'beta_p':>7} {'r_p/r_b':>8} {'a/r_b':>6} {'d/r_b':>6}"
          f" {'feasible delta arc':>22} {'delta_G':>9} {'n_feas':>7}")
    for r in sorted(odd, key=lambda r: (r["beta"], r["beta_p"], r["a_rb"])):
        arc = f"[{r['arc'][0]:.0f}, {r['arc'][1]:.0f})"
        print(f"     {r['beta']:>5.0f} {r['beta_p']:>7.0f} {r['rp']:>8.2f} "
              f"{r['a_rb']:>6.2f} {r['d_rb']:>6.1f} {arc:>22} {r['dG']:>9.3f} "
              f"{r['n_feas']:>7}")
    if odd:
        keys = ("beta", "beta_p", "rp", "d_rb")
        print("\n     shared coordinates among them:")
        for k in keys:
            vals = sorted({r[k] for r in odd})
            edge = ""
            axis = {"beta": B_BETA, "beta_p": B_BETA_P,
                    "rp": B_RP_RB, "d_rb": B_D_RB}[k]
            if len(vals) == 1 and vals[0] in (min(axis), max(axis)):
                edge = "   <-- GRID EDGE: the boundary may lie outside this sweep"
            print(f"       {k:<8} {vals}{edge}")

    # -- swing ---------------------------------------------------------------
    sw = np.array([r["swing"] for r in rows if np.isfinite(r["swing"])])
    rs = np.array([r["raw_swing"] for r in rows])
    print(f"\n[B4] z_home swing over the feasible delta set:")
    print(f"     (max-min)/mean      min {sw.min():.4f}  median {np.median(sw):.4f}"
          f"  max {sw.max():.4f}")
    print(f"     raw 4a|g| swing in (z_home-c_p)^2   "
          f"min {rs.min():.4f}  median {np.median(rs):.4f}  max {rs.max():.4f}")
    return rows


# --------------------------------------------------------------------------- #
# independent confirmation of the z_home formula
# --------------------------------------------------------------------------- #
def confirm_zhome_against_arm_tips(n_samples=8):
    """Take z_home from the formula, then check |q_i - arm_tips(0)_i| == d."""
    print("\n" + "=" * 78)
    print("CONFIRMATION - z_home formula vs arm_tips(0), rod length d")
    print("=" * 78)
    print("if this residual is not ~1e-13 the formula is wrong and A and B are void")
    rng = np.random.default_rng(0)
    picks, tries = [], 0
    while len(picks) < n_samples and tries < 4000:
        tries += 1
        beta = float(rng.choice(B_BETA)); beta_p = float(rng.choice(B_BETA_P))
        rp = float(rng.choice(B_RP_RB)); a_rb = float(rng.choice(B_A_RB))
        d_rb = float(rng.choice(B_D_RB)); delta = float(rng.uniform(0.0, 180.0))
        r_b, r_p, a, d = B_R_B, rp * B_R_B, a_rb * B_R_B, d_rb * B_R_B
        _, g2, gu = _closed_form_sq(r_b, r_p, beta, beta_p, delta, a)
        sq = _zhome_sq(d, g2, a, gu)
        if sq <= 0.0:
            continue
        z_home = B_C_P + np.sqrt(sq)
        g = _geom(r_b, beta, delta, rp, beta_p, a_rb, d_rb, B_C_P)
        if g is None:
            continue
        q = stage1(g, np.eye(3), np.array([0.0, 0.0, z_home]))
        tips = arm_tips(g, np.zeros(6))
        res = float(np.max(np.abs(np.linalg.norm(q - tips, axis=0) - d)))
        picks.append((beta, beta_p, rp, a_rb, d_rb, delta, z_home, res))

    print(f"\n  {'beta':>5} {'beta_p':>7} {'r_p/r_b':>8} {'a/r_b':>6} {'d/r_b':>6}"
          f" {'delta':>8} {'z_home':>10} {'max||q-tip|-d|':>16}")
    worst = 0.0
    for row in picks:
        worst = max(worst, row[7])
        print(f"  {row[0]:>5.0f} {row[1]:>7.0f} {row[2]:>8.2f} {row[3]:>6.2f} "
              f"{row[4]:>6.1f} {row[5]:>8.3f} {row[6]:>10.6f} {row[7]:>16.3e}")
    print(f"\n  worst residual over {len(picks)} samples: {worst:.3e}  "
          f"{'OK' if worst < 1e-12 else 'FORMULA WRONG'}")
    return worst


def main():
    print("PROVENANCE OF THE CHECK B GRID")
    print("-" * 78)
    print("The script behind '8 of 432 ... all at d/r_b = 0.8' is NOT in the repo")
    print("and NOT in git history - the only occurrence of '432' anywhere in the")
    print("history is the prose claim in docs/session-handoff-2026-09-04.md:78.")
    print("The grid below is RECOVERED VERBATIM from the surviving scratchpad copy")
    print("of the script that produced it, not reconstructed from the prose.")
    print("Block [B0] re-derives the original 8 as a check on that recovery.")
    print()
    check_flat_arm_leg_independence()
    sweep_zhome_delta()
    confirm_zhome_against_arm_tips()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
