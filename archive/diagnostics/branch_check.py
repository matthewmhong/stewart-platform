"""Branch-constancy check.  DIAGNOSTIC ONLY.

No branch rule is chosen here and no sign vector is written to tracked code.
Every numeric geometry value below is a FIXTURE, labelled as such, not a design
decision.  F1 (c_p threaded through platform_ring / make_geometry) has been
applied to tracked code; nothing else in stewart/ is touched.
"""
from __future__ import annotations

import numpy as np

from stewart.geometry import base_ring, platform_ring

np.set_printoptions(linewidth=220, suppress=False)
TAU = 2.0 * np.pi


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def axis_angle(axis, ang):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(ang) * K + (1.0 - np.cos(ang)) * (K @ K)


def wrap(a):
    """to (-pi, pi]."""
    return -((-np.asarray(a, float) + np.pi) % TAU - np.pi)


def geom(r_b, beta, delta, r_p, beta_p, c_p):
    b, n, u = base_ring(r_b, beta, delta)
    p = platform_ring(r_p, beta_p, c_p)
    v = np.cross(n, u, axis=0)                 # v_i = n_i x u_i  (== z per sec.8)
    return dict(b=b, n=n, u=u, v=v, p=p)


def closed_form(L, u, n, v, a, d):
    """Sec.5.3 closed form for one leg.

    M = L.u   N = L.v   P = (|L|^2 + a^2 - d^2)/(2a)   C = hypot(M,N)
    alpha = phi +/- arccos(P/C),  phi = atan2(N, M).
    Guards |P| > C before arccos (no clip).  Returns wrapped roots in (-pi, pi].
    """
    M = float(L @ u)
    N = float(L @ v)
    w = float(L @ n)
    P = (float(L @ L) + a * a - d * d) / (2.0 * a)
    C = float(np.hypot(M, N))
    phi = float(np.arctan2(N, M))
    sgn = float(np.sign(abs(P) - C))           # -1 reachable, 0 boundary, +1 not
    out = dict(M=M, N=N, w=w, P=P, C=C, phi=phi, ratio=P / C if C else np.inf,
               margin=C - abs(P), sign_absP_minus_C=sgn, reachable=abs(P) <= C)
    if abs(P) > C:
        out["alpha_plus"] = np.nan
        out["alpha_minus"] = np.nan
        out["ac"] = np.nan
        return out
    r = P / C
    assert -1.0 <= r <= 1.0, f"P/C={r!r} outside [-1,1] after the |P|<=C guard"
    ac = float(np.arccos(r))
    out["ac"] = ac
    out["alpha_plus"] = wrap(phi + ac)
    out["alpha_minus"] = wrap(phi - ac)
    out["alpha_plus_raw"] = phi + ac
    out["alpha_minus_raw"] = phi - ac
    return out


def legs_L(g, R, T):
    """L_i = (R p_i + T) - b_i, shape (3,6)."""
    return (R @ g["p"] + np.asarray(T, float).reshape(3, 1)) - g["b"]


def solve_z_home(r_b, beta, delta, r_p, beta_p, c_p, a, d):
    """Upper root of (z_home - c_p)^2 = d^2 - |q_i^flat - h_i^flat|^2 at
    R = I, alpha_i = 0  (h_i = b_i + a u_i).  Returns (z_home, rhs_per_leg)."""
    g = geom(r_b, beta, delta, r_p, beta_p, c_p)
    q_flat = g["p"][:2]                                   # R=I, T_xy=0
    h_flat = (g["b"] + a * g["u"])[:2]
    rhs = d * d - np.sum((q_flat - h_flat) ** 2, axis=0)  # (6,)
    m = float(np.mean(rhs))
    z = c_p + np.sqrt(m) if m >= 0.0 else np.nan          # upper root
    return z, rhs, g


def d15(x):
    return np.array2string(np.asarray(x, float), precision=15, floatmode="maxprec")


# ========================================================================= #
# PART 1
# ========================================================================= #
# ---- fixtures (NOT decisions) -------------------------------------------- #
FIX_A = dict(r_b=1.0, beta=20.0, delta=40.0, r_p=0.85, beta_p=40.0, c_p=0.10)
FIX_A_AD = dict(a=0.20, d=1.20)          # a/r_b, d/r_b : fixture, mid-grid pick
DELTA_1C = 40.0                          # fixture: delta held at fixture-A value


def part1():
    print("=" * 78)
    print("PART 1 - z_home from the assembly datum")
    print("=" * 78)
    print("closed form: sec.5.3, M=L.u N=L.v P=(|L|^2+a^2-d^2)/2a C=hypot(M,N)")
    print("             alpha = atan2(N,M) +/- arccos(P/C); |P|>C guarded, no clip")
    print(f"FIXTURE geometry A : {FIX_A}")
    print(f"FIXTURE a,d (norm. r_b): {FIX_A_AD}   [mid-grid pick, not a decision]")

    a, d = FIX_A_AD["a"], FIX_A_AD["d"]
    z, rhs, g = solve_z_home(a=a, d=d, **FIX_A)

    # 1a --------------------------------------------------------------------
    print("\n[1a] D3: all six legs give the same RHS  d^2 - |q^flat - h^flat|^2")
    print("     rhs_i =", d15(rhs))
    print(f"     spread max-min = {rhs.max() - rhs.min():.3e}   "
          f"(mean {rhs.mean():.15e})")
    print(f"     -> z_home (upper root) = c_p + sqrt(mean) = {z:.15e}   (z_home/r_b = {z/FIX_A['r_b']:.15e})")

    # 1b --------------------------------------------------------------------
    print("\n[1b] feed z_home back; every alpha_i must be 0 on one branch")
    R = np.eye(3)
    T = np.array([0.0, 0.0, z])
    L = legs_L(g, R, T)
    rows = []
    for i in range(6):
        cf = closed_form(L[:, i], g["u"][:, i], g["n"][:, i], g["v"][:, i], a, d)
        rows.append(cf)
        print(f"   leg {i+1}: P={cf['P']:+.12e}  C={cf['C']:.12e}  P/C={cf['ratio']:+.12e}  "
              f"sign(|P|-C)={cf['sign_absP_minus_C']:+.0f}")
        print(f"          alpha_minus={cf['alpha_minus']:+.3e}   alpha_plus={cf['alpha_plus']:+.3e}")
    amin = np.array([r["alpha_minus"] for r in rows])
    aplus = np.array([r["alpha_plus"] for r in rows])
    which = ["-" if abs(m) < abs(p) else "+" for m, p in zip(amin, aplus)]
    print(f"   branch giving alpha_i ~ 0 per leg : {which}")
    print(f"   max|alpha| on that branch         : "
          f"{max(abs(m) if wch=='-' else abs(p) for m, p, wch in zip(amin, aplus, which)):.3e}")
    print(f"   same branch for all six?          : {len(set(which)) == 1}  "
          f"(-> '{which[0]}' branch)" if len(set(which)) == 1 else f"   MIXED: {which}")
    chosen = which[0] if len(set(which)) == 1 else None
    assert max(abs(amin)) < 1e-12, f"alpha_minus not ~0: {amin}"

    # 1c --------------------------------------------------------------------
    print("\n[1c] discriminant  disc = d^2 - |q^flat - h^flat|^2  over the grid")
    print(f"     held fixture: delta = {DELTA_1C} deg,  c_p/r_b = 0.1,  r_b = 1")
    betas = [10, 20, 30, 40]
    beta_ps = [10, 25, 40, 55]
    rps = [0.6, 0.85, 1.2]
    a_s = [0.10, 0.20, 0.35]
    d_s = [0.8, 1.2, 1.6]
    c_p = 0.1
    neg = []
    pos_z = []
    max_leg_spread = 0.0
    total = 0
    for be in betas:
        for bp in beta_ps:
            for rp in rps:
                for aa in a_s:
                    for dd in d_s:
                        total += 1
                        try:
                            _, rhs, _ = solve_z_home(1.0, be, DELTA_1C, rp, bp, c_p, aa, dd)
                        except Exception as exc:
                            print(f"     build failed be={be} bp={bp} rp={rp} a={aa} d={dd}: {exc!r}")
                            continue
                        max_leg_spread = max(max_leg_spread, rhs.max() - rhs.min())
                        disc = float(np.mean(rhs))
                        if disc < 0.0:
                            neg.append((be, bp, rp, aa, dd, disc))
                        else:
                            pos_z.append(c_p + np.sqrt(disc))
    print(f"     grid size = {total}   (4 x 4 x 3 x 3 x 3)")
    print(f"     max per-leg rhs spread over the whole grid = {max_leg_spread:.3e}  (D3 holds)")
    print(f"     combos with disc < 0 (no valid z_home): {len(neg)}")
    for row in neg:
        print(f"        beta={row[0]:<3} beta_p={row[1]:<3} r_p={row[2]:<4} a={row[3]:<4} "
              f"d={row[4]:<4}  disc={row[5]:+.4e}")
    if pos_z:
        pz = np.array(pos_z)
        print(f"     where disc >= 0 : z_home/r_b in [{pz.min():.6f}, {pz.max():.6f}]  "
              f"(median {np.median(pz):.6f}, n={pz.size})")
    return chosen


# ========================================================================= #
# PART 2
# ========================================================================= #
def build_pose(tilt_axis, tilt_mag_rad, yaw_rad, tx, ty, tz, z_home):
    R = axis_angle((0, 0, 1), yaw_rad) @ axis_angle(tilt_axis, tilt_mag_rad)
    T = np.array([tx, ty, z_home + tz])
    return R, T


def envelope_poses(z_home, r_b):
    """PROVISIONAL envelope - the real tilt target is not settled.
    tilt <= 6 deg about arbitrary horizontal axes (24 az x 4 mag) + zero,
    yaw +/-10 deg, T horizontal +/-0.05 r_b, T vertical +/-0.05 r_b.
    """
    azis = np.deg2rad(np.arange(0.0, 360.0, 15.0))            # 24
    mags = np.deg2rad(np.linspace(1.5, 6.0, 4))               # 4
    tilts = [((1.0, 0.0, 0.0), 0.0)]
    for az in azis:
        ax = (np.cos(az), np.sin(az), 0.0)
        for mg in mags:
            tilts.append((ax, mg))
    yaws = np.deg2rad([-10.0, 0.0, 10.0])
    th = [(0.0, 0.0), (0.05, 0.0), (-0.05, 0.0), (0.0, 0.05), (0.0, -0.05)]
    tv = [-0.05, 0.0, 0.05]
    poses = []
    for (ax, mg) in tilts:
        for yw in yaws:
            for (tx, ty) in th:
                for tzf in tv:
                    R, T = build_pose(ax, mg, yw, tx * r_b, ty * r_b, tzf * r_b, z_home)
                    poses.append(dict(R=R, T=T, tilt_axis=ax, tilt_deg=np.degrees(mg),
                                      yaw_deg=np.degrees(yw), tx=tx, ty=ty, tz=tzf))
    return poses


def part2(chosen):
    print("\n\n" + "=" * 78)
    print("PART 2 - does the branch hold across an envelope")
    print("=" * 78)
    a, d = FIX_A_AD["a"], FIX_A_AD["d"]
    z_home, _, g = solve_z_home(a=a, d=d, **FIX_A)
    br = chosen if chosen in ("+", "-") else "-"
    print(f"chosen branch (from 1b) : '{br}'    a,d fixture = {FIX_A_AD}")
    print(f"z_home/r_b = {z_home:.12f}")
    print("ENVELOPE is PROVISIONAL (tilt target not settled): tilt<=6deg, 24 az x 4 mag,")
    print("+ zero; yaw +/-10; T_horiz +/-0.05 r_b; T_vert +/-0.05 r_b.")

    poses = envelope_poses(z_home, FIX_A["r_b"])
    P = len(poses)
    print(f"poses sampled : {P}")

    key = "alpha_minus" if br == "-" else "alpha_plus"
    other = "alpha_plus" if br == "-" else "alpha_minus"

    chosen_alpha = np.full((P, 6), np.nan)
    other_alpha = np.full((P, 6), np.nan)
    marginC = np.full((P, 6), np.nan)          # (C-|P|)/C
    undef = []                                 # (pose_idx, leg) chosen root undefined

    for k, ps in enumerate(poses):
        L = legs_L(g, ps["R"], ps["T"])
        for i in range(6):
            cf = closed_form(L[:, i], g["u"][:, i], g["n"][:, i], g["v"][:, i], a, d)
            marginC[k, i] = cf["margin"] / cf["C"] if cf["C"] else np.nan
            if not cf["reachable"]:
                undef.append((k, i))
                continue
            chosen_alpha[k, i] = cf[key]
            other_alpha[k, i] = cf[other]

    # 2a ------------------------------------------------------------------
    print("\n[2a] chosen root undefined (|P| > C) anywhere in the envelope?")
    if not undef:
        print("     count = 0   -> chosen branch defined at every (pose, leg)")
    else:
        by_leg = {}
        for k, i in undef:
            by_leg.setdefault(i + 1, []).append(k)
        print(f"     count = {len(undef)}  over {len({k for k,_ in undef})} poses")
        for lg, ks in sorted(by_leg.items()):
            print(f"       leg {lg}: {len(ks)} poses, e.g. "
                  f"{[ (round(poses[j]['tilt_deg'],1), round(poses[j]['yaw_deg'],1), poses[j]['tx'], poses[j]['ty'], poses[j]['tz']) for j in ks[:3] ]}")
    print("     (discontinuity of the chosen root is examined by the trajectory in 2d)")

    # 2b ------------------------------------------------------------------
    print("\n[2b] min over envelope of (C - |P|)/C per leg  (0 => on the branch-merge boundary)")
    for i in range(6):
        col = marginC[:, i]
        j = int(np.nanargmin(col))
        ps = poses[j]
        print(f"     leg {i+1}: min (C-|P|)/C = {col[j]:+.6e}   at "
              f"tilt {ps['tilt_deg']:.1f} deg about ({ps['tilt_axis'][0]:+.3f},{ps['tilt_axis'][1]:+.3f}), "
              f"yaw {ps['yaw_deg']:+.0f}, T=({ps['tx']:+.2f},{ps['ty']:+.2f},{ps['tz']:+.2f})r_b")
    print(f"     global min over all legs/poses = {np.nanmin(marginC):+.6e}")

    # 2c ------------------------------------------------------------------
    print("\n[2c] per-leg angular span (max-min), chosen branch vs other branch  [radians]")
    print("     no servo data exists -> numbers only, no range judgement")
    print(f"     {'leg':>4} {'chosen min':>13} {'chosen max':>13} {'chosen span':>13} "
          f"{'other min':>13} {'other max':>13} {'other span':>13}")
    for i in range(6):
        c = chosen_alpha[:, i][~np.isnan(chosen_alpha[:, i])]
        o = other_alpha[:, i][~np.isnan(other_alpha[:, i])]
        print(f"     {i+1:>4} {c.min():>13.6f} {c.max():>13.6f} {c.max()-c.min():>13.6e} "
              f"{o.min():>13.6f} {o.max():>13.6f} {o.max()-o.min():>13.6e}")
    sp = np.nanmax(chosen_alpha, 0) - np.nanmin(chosen_alpha, 0)
    if np.any(sp > np.pi):
        print("     NOTE: a chosen-branch span exceeds pi -> wrap ambiguity, read with care")

    # 2d ------------------------------------------------------------------
    print("\n[2d] smooth closed trajectory: tilt axis precesses at fixed magnitude")
    tilt_mag_deg = 4.0                      # FIXTURE (provisional)
    nstep = 720
    print(f"     FIXTURE: tilt magnitude {tilt_mag_deg} deg, yaw 0, T=(0,0,z_home), "
          f"{nstep} steps, azimuth 0->360 (closed)")
    az = np.deg2rad(np.linspace(0.0, 360.0, nstep + 1))
    seq_ch = np.full((nstep + 1, 6), np.nan)
    seq_ot = np.full((nstep + 1, 6), np.nan)
    bad = 0
    for k, a_ in enumerate(az):
        R, T = build_pose((np.cos(a_), np.sin(a_), 0.0), np.deg2rad(tilt_mag_deg),
                          0.0, 0.0, 0.0, 0.0, z_home)
        L = legs_L(g, R, T)
        for i in range(6):
            cf = closed_form(L[:, i], g["u"][:, i], g["n"][:, i], g["v"][:, i], a, d)
            if not cf["reachable"]:
                bad += 1
                continue
            seq_ch[k, i] = cf[key]
            seq_ot[k, i] = cf[other]
    dstep_ch = np.abs(wrap(np.diff(seq_ch, axis=0)))
    dstep_ot = np.abs(wrap(np.diff(seq_ot, axis=0)))
    print(f"     unreachable steps on chosen branch : {bad}")
    print(f"     {'leg':>4} {'chosen max|dalpha|':>20} {'chosen median':>16} "
          f"{'ratio max/med':>14} {'other max|dalpha|':>18}")
    for i in range(6):
        mx = np.nanmax(dstep_ch[:, i])
        md = np.nanmedian(dstep_ch[:, i])
        omx = np.nanmax(dstep_ot[:, i])
        flag = "  <-- STEP OUTLIER (branch flip)" if md > 0 and mx / md > 5.0 else ""
        print(f"     {i+1:>4} {mx:>20.6e} {md:>16.6e} {mx/md if md else np.inf:>14.2f} "
              f"{omx:>18.6e}{flag}")
    print(f"     endpoint closure max_i |alpha_i(360) - alpha_i(0)| chosen = "
          f"{np.nanmax(np.abs(wrap(seq_ch[-1] - seq_ch[0]))):.3e}")

    # supplementary continuity probes (not in the brief; to attribute 2c's span)
    print("\n[2d-supp] extra smooth 1-D sweeps on the chosen branch (max per-step |dalpha|):")
    def sweep(name, mk):
        n = 400
        A = np.full((n, 6), np.nan)
        for k in range(n):
            R, T = mk(k / (n - 1))
            L = legs_L(g, R, T)
            for i in range(6):
                cf = closed_form(L[:, i], g["u"][:, i], g["n"][:, i], g["v"][:, i], a, d)
                if cf["reachable"]:
                    A[k, i] = cf[key]
        ds = np.abs(wrap(np.diff(A, axis=0)))
        rng = np.nanmax(A, 0) - np.nanmin(A, 0)
        print(f"     {name:<28} max|dstep| = {np.nanmax(ds):.4e}   "
              f"per-leg span = {np.array2string(rng, precision=4)}")
    sweep("yaw  -10 -> +10 deg",
          lambda t: build_pose((1, 0, 0), 0.0, np.deg2rad(-10 + 20 * t), 0, 0, 0, z_home))
    sweep("tilt  0 -> 6 deg (az=0)",
          lambda t: build_pose((1, 0, 0), np.deg2rad(6 * t), 0.0, 0, 0, 0, z_home))
    sweep("T_z  -0.05 -> +0.05 r_b",
          lambda t: build_pose((1, 0, 0), 0.0, 0.0, 0, 0, -0.05 + 0.10 * t, z_home))


if __name__ == "__main__":
    chosen = part1()
    part2(chosen)
