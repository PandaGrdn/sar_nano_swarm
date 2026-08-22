#!/usr/bin/env python3
"""ci_fusion.py — Covariance Intersection + split-CI for neighbor updates (P2-4).

No rclpy. Plan §4.4 / D6.

Classic CI (same quantity, unknown correlation):
    P_f^{-1} = ω P_i^{-1} + (1-ω) P_j^{-1}
    x_f     = P_f (ω P_i^{-1} x_i + (1-ω) P_j^{-1} x_j)

ω: 'fast' = tr(P_j)/(tr(P_i)+tr(P_j)); 'grid' = 21-point min-trace.

A relative UWB measurement is not the same quantity. Apply CI by split-CI
(SCI): inflate each side's covariance, then do a standard EKF update of
drone i only:
    S = H_i (P_i/ω) H_iᵀ + H_j (P_j/(1-ω)) H_jᵀ + R

Usage:
    python3 perception/swarm_loc/ci_fusion.py --selftest
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[2]
for _p in ("perception/swarm_loc", "perception/uwb_sim"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

from state import IDX_P, N_STATE, symmetrize  # noqa: E402

OMEGA_EPS = 1e-3
N_GRID = 21


def omega_fast(P_i: np.ndarray, P_j: np.ndarray) -> float:
    """Closed-form trace-min approximation (plan §4.4)."""
    ti = float(np.trace(P_i))
    tj = float(np.trace(P_j))
    s = ti + tj
    if s < 1e-18:
        return 0.5
    w = tj / s
    return float(np.clip(w, OMEGA_EPS, 1.0 - OMEGA_EPS))


def fused_P(P_i: np.ndarray, P_j: np.ndarray, omega: float) -> np.ndarray:
    w = float(np.clip(omega, 0.0, 1.0))
    if w <= OMEGA_EPS:
        return 0.5 * (P_j + P_j.T)
    if w >= 1.0 - OMEGA_EPS:
        return 0.5 * (P_i + P_i.T)
    Ai = np.linalg.inv(0.5 * (P_i + P_i.T))
    Aj = np.linalg.inv(0.5 * (P_j + P_j.T))
    Pf = np.linalg.inv(w * Ai + (1.0 - w) * Aj)
    return 0.5 * (Pf + Pf.T)


def fused_x(
    x_i: np.ndarray, P_i: np.ndarray, x_j: np.ndarray, P_j: np.ndarray, omega: float
) -> np.ndarray:
    w = float(np.clip(omega, OMEGA_EPS, 1.0 - OMEGA_EPS))
    Pf = fused_P(P_i, P_j, w)
    Ai = np.linalg.inv(0.5 * (P_i + P_i.T))
    Aj = np.linalg.inv(0.5 * (P_j + P_j.T))
    return Pf @ (w * (Ai @ x_i) + (1.0 - w) * (Aj @ x_j))


def omega_grid(P_i: np.ndarray, P_j: np.ndarray, n: int = N_GRID) -> float:
    """21-point sweep minimizing trace(P_fused)."""
    best_w = 0.5
    best_tr = float("inf")
    for w in np.linspace(OMEGA_EPS, 1.0 - OMEGA_EPS, int(n)):
        tr = float(np.trace(fused_P(P_i, P_j, float(w))))
        if tr < best_tr:
            best_tr = tr
            best_w = float(w)
    return best_w


def choose_omega(P_i: np.ndarray, P_j: np.ndarray, method: str = "fast") -> float:
    if method == "grid":
        return omega_grid(P_i, P_j)
    return omega_fast(P_i, P_j)


def fuse(
    x_i: np.ndarray,
    P_i: np.ndarray,
    x_j: np.ndarray,
    P_j: np.ndarray,
    method: str = "fast",
    omega: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray, float]:
    """Classic CI. Returns (x_fused, P_fused, omega)."""
    Pi = 0.5 * (np.asarray(P_i, dtype=np.float64) + np.asarray(P_i, dtype=np.float64).T)
    Pj = 0.5 * (np.asarray(P_j, dtype=np.float64) + np.asarray(P_j, dtype=np.float64).T)
    w = choose_omega(Pi, Pj, method) if omega is None else float(omega)
    xf = fused_x(np.asarray(x_i, dtype=np.float64).reshape(-1), Pi, np.asarray(x_j, dtype=np.float64).reshape(-1), Pj, w)
    Pf = fused_P(Pi, Pj, w)
    return xf, Pf, w


def naive_fuse(
    x_i: np.ndarray, P_i: np.ndarray, x_j: np.ndarray, P_j: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Kalman fusion assuming uncorrelated errors. Overconfident if they are not."""
    Pi = 0.5 * (np.asarray(P_i, dtype=np.float64) + np.asarray(P_i, dtype=np.float64).T)
    Pj = 0.5 * (np.asarray(P_j, dtype=np.float64) + np.asarray(P_j, dtype=np.float64).T)
    Ai = np.linalg.inv(Pi)
    Aj = np.linalg.inv(Pj)
    Pf = np.linalg.inv(Ai + Aj)
    xf = Pf @ (Ai @ np.asarray(x_i, dtype=np.float64).reshape(-1) + Aj @ np.asarray(x_j, dtype=np.float64).reshape(-1))
    return xf, 0.5 * (Pf + Pf.T)


def sci_S(
    H_i: np.ndarray,
    P_i: np.ndarray,
    H_j: np.ndarray,
    P_j: np.ndarray,
    R: np.ndarray,
    omega: float,
) -> np.ndarray:
    w = float(np.clip(omega, OMEGA_EPS, 1.0 - OMEGA_EPS))
    Pi = P_i / w
    Pj = P_j / (1.0 - w)
    S = H_i @ Pi @ H_i.T + H_j @ Pj @ H_j.T + R
    return 0.5 * (S + S.T)


def sci_updated_P(
    H_i: np.ndarray,
    P_i: np.ndarray,
    H_j: np.ndarray,
    P_j: np.ndarray,
    R: np.ndarray,
    omega: float,
) -> np.ndarray:
    w = float(np.clip(omega, OMEGA_EPS, 1.0 - OMEGA_EPS))
    P_inf = P_i / w
    R_eff = H_j @ (P_j / (1.0 - w)) @ H_j.T + R
    R_eff = 0.5 * (R_eff + R_eff.T)
    S = H_i @ P_inf @ H_i.T + R_eff
    S = 0.5 * (S + S.T)
    K = np.linalg.solve(S, (P_inf @ H_i.T).T).T
    I = np.eye(P_i.shape[0])
    KH = K @ H_i
    P = (I - KH) @ P_inf @ (I - KH).T + K @ R_eff @ K.T
    return symmetrize(P)


def omega_grid_sci(
    H_i: np.ndarray,
    P_i: np.ndarray,
    H_j: np.ndarray,
    P_j: np.ndarray,
    R: np.ndarray,
    n: int = N_GRID,
) -> float:
    best_w = 0.5
    best_tr = float("inf")
    for w in np.linspace(OMEGA_EPS, 1.0 - OMEGA_EPS, int(n)):
        try:
            P = sci_updated_P(H_i, P_i, H_j, P_j, R, float(w))
        except np.linalg.LinAlgError:
            continue
        tr = float(np.trace(P[IDX_P, IDX_P]) if P.shape[0] >= 3 else np.trace(P))
        if tr < best_tr:
            best_tr = tr
            best_w = float(w)
    return best_w


def choose_omega_sci(
    H_i: np.ndarray,
    P_i: np.ndarray,
    H_j: np.ndarray,
    P_j: np.ndarray,
    R: np.ndarray,
    method: str = "fast",
) -> float:
    if method == "grid":
        return omega_grid_sci(H_i, P_i, H_j, P_j, R)
    return omega_fast(
        P_i[0:3, 0:3] if P_i.shape[0] >= 3 else P_i,
        P_j[0:3, 0:3] if P_j.shape[0] >= 3 else P_j,
    )


def implied_position(p_i, residual_body, R_bw, P_j_pos, R_cart):
    """World-frame own-position implied by a 3D relative measurement + neighbor p.

    p_impl = p_i - R_bw @ residual_body   (because residual = z - Rᵀ(p_j-p_i))
    P_impl = P_j_pos + R_bw R_cart R_bwᵀ     (measurement + neighbor, not own prior)
    """
    R_bw = np.asarray(R_bw, dtype=np.float64)
    r = np.asarray(residual_body, dtype=np.float64).reshape(3)
    p_impl = np.asarray(p_i, dtype=np.float64).reshape(3) - R_bw @ r
    P_impl = np.asarray(P_j_pos, dtype=np.float64) + R_bw @ np.asarray(R_cart, dtype=np.float64) @ R_bw.T
    return p_impl, 0.5 * (P_impl + P_impl.T)


def apply_position_ci(x, P, p_impl, P_impl, method: str = "fast"):
    """Replace the position block of (x, P) by CI with (p_impl, P_impl)."""
    Ppp = 0.5 * (P[0:3, 0:3] + P[0:3, 0:3].T)
    xf, Pf, w = fuse(x[0:3], Ppp, p_impl, P_impl, method=method)
    try:
        T = Pf @ np.linalg.inv(Ppp)
    except np.linalg.LinAlgError:
        T = np.eye(3)
    out_x = np.array(x, dtype=np.float64, copy=True)
    out_P = np.array(P, dtype=np.float64, copy=True)
    out_x[0:3] = xf
    out_P[0:3, 0:3] = Pf
    out_P[0:3, 3:] = T @ P[0:3, 3:]
    out_P[3:, 0:3] = out_P[0:3, 3:].T
    return out_x, symmetrize(out_P), w


def run_selftest() -> int:
    ok = True
    n_pass = 0
    n_fail = 0

    def check(name: str, cond: bool, detail: str = ""):
        nonlocal ok, n_pass, n_fail
        if not cond:
            ok = False
            n_fail += 1
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))
        else:
            n_pass += 1
            print(f"[selftest] PASS {name}")

    # 1. Classic CI algebra
    P_i = np.diag([4.0, 1.0])
    P_j = np.diag([1.0, 4.0])
    w = omega_fast(P_i, P_j)
    check("1 fast omega in (0,1)", OMEGA_EPS < w < 1.0 - OMEGA_EPS)
    xf, Pf, w2 = fuse(np.array([0.0, 0.0]), P_i, np.array([1.0, 1.0]), P_j, method="fast")
    check("1b fused P SPD", np.min(np.linalg.eigvalsh(Pf)) > 0)
    wg = omega_grid(P_i, P_j)
    Pg = fused_P(P_i, P_j, wg)
    # grid trace should be <= fast trace (it explicitly minimizes)
    check(
        "1c grid trace <= fast trace",
        float(np.trace(Pg)) <= float(np.trace(Pf)) + 1e-9,
        f"grid={np.trace(Pg):.4f} fast={np.trace(Pf):.4f}",
    )

    # 2. Fully correlated 1D estimates: naive overconfident, CI consistent
    # Both report x=1, P=1, truth=0, errors identical (rho=1).
    xi = np.array([1.0])
    Pi = np.array([[1.0]])
    x_true = 0.0
    xn, Pn = naive_fuse(xi, Pi, xi, Pi)
    xc, Pc, _ = fuse(xi, Pi, xi, Pi, method="fast")
    nees_n = float((xn[0] - x_true) ** 2 / Pn[0, 0])
    nees_c = float((xc[0] - x_true) ** 2 / Pc[0, 0])
    check("2 naive P smaller than either", Pn[0, 0] < 1.0 - 1e-9, f"P={Pn[0,0]:.3f}")
    check("2b naive NEES overconfident", nees_n > 1.5, f"NEES={nees_n:.3f}")
    check("2c CI P not collapsed", Pc[0, 0] >= 1.0 - 1e-6, f"P={Pc[0,0]:.3f}")
    check("2d CI NEES consistent (~1)", 0.5 <= nees_c <= 2.0, f"NEES={nees_c:.3f}")

    # 3. Two-drone relative MC
    print("[selftest] running two-drone pair Monte Carlo ...")
    from rio_stub import load_config, resolve_config_path  # noqa: E402

    cfg = load_config(resolve_config_path("configs/estimation/swarm_loc.yaml"))
    bearing = run_pair_mc(cfg, use_bearing=True, fusion="ci", n_runs=200, n_steps=18, seed=4)
    rng_only = run_pair_mc(cfg, use_bearing=False, fusion="ci", n_runs=200, n_steps=18, seed=4)
    naive = run_pair_mc(cfg, use_bearing=True, fusion="naive", n_runs=200, n_steps=18, seed=4)
    print(
        f"[selftest] CI+bearing rel_p50={bearing['rel_p50']:.3f} mean_NEES={bearing['mean_nees']:.2f} "
        f"frac95={bearing['frac_in_95']:.3f}"
    )
    print(
        f"[selftest] CI+range  rel_p50={rng_only['rel_p50']:.3f} mean_NEES={rng_only['mean_nees']:.2f}"
    )
    print(
        f"[selftest] naive+bearing rel_p50={naive['rel_p50']:.3f} mean_NEES={naive['mean_nees']:.2f} "
        f"frac95={naive['frac_in_95']:.3f}"
    )
    check(
        "3 bearing rel-err < range-only",
        bearing["rel_p50"] < 0.85 * rng_only["rel_p50"],
        f"bearing={bearing['rel_p50']:.3f} range={rng_only['rel_p50']:.3f}",
    )
    check(
        "3b CI NEES not overconfident",
        bearing["frac_in_95"] >= 0.85 and bearing["mean_nees"] < 5.0,
        f"mean={bearing['mean_nees']:.2f} frac95={bearing['frac_in_95']:.3f} hi={bearing['nees_hi']:.2f}",
    )
    check(
        "3c naive NEES overconfident vs CI",
        naive["mean_nees"] > bearing["mean_nees"] * 1.5
        and naive["mean_nees"] > naive["nees_hi"],
        f"naive={naive['mean_nees']:.2f} CI={bearing['mean_nees']:.2f} naive_hi={naive['nees_hi']:.2f}",
    )

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def run_pair_mc(
    cfg: dict,
    use_bearing: bool,
    fusion: str,
    n_runs: int = 200,
    n_steps: int = 18,
    seed: int = 4,
) -> dict:
    """Two static drones exchanging relative UWB. Shared init bias is unobservable."""
    from ekf import chi2_ppf, position_nees, update  # noqa: E402
    from measurements import range_only, relpos  # noqa: E402
    from state import IDX_PSI, SwarmState, wrap_psi  # noqa: E402
    from uwb_model import bearing_xyz  # noqa: E402

    sigma_d = 0.0873
    sigma_az = math.radians(4.795)
    sigma_el = math.radians(4.795)
    rng = np.random.default_rng(seed)
    p0_true = SwarmState.from_launch(cfg, 0).p.copy()
    p1_true = SwarmState.from_launch(cfg, 1).p.copy()
    rel_true = p1_true - p0_true
    nees_all = []
    rel_err = []
    nees_rel = []

    def synth_z(p_obs, psi_obs, p_peer, rng_):
        R = __import__("state").rpy_to_R(float(psi_obs), 0.0, 0.0)
        z_true = R.T @ (p_peer - p_obs)
        d = float(np.linalg.norm(z_true))
        az = math.atan2(float(z_true[1]), float(z_true[0]))
        el = math.asin(float(np.clip(z_true[2] / max(d, 1e-12), -1.0, 1.0)))
        d_m = max(d + float(rng_.normal(0.0, sigma_d)), 0.05)
        az_m = az + float(rng_.normal(0.0, sigma_az))
        el_m = float(np.clip(el + rng_.normal(0.0, sigma_el), -1.56, 1.56))
        x, y, z = bearing_xyz(d_m, az_m, el_m)
        return np.array([x, y, z]), d_m, az_m, el_m

    for _ in range(n_runs):
        s0 = SwarmState.from_launch(cfg, 0)
        s1 = SwarmState.from_launch(cfg, 1)
        # Independent draws from P0, plus a shared position bias whose variance
        # is folded into P (marginally consistent). Naive fusion treats the two
        # filters as uncorrelated and collapses P; CI does not.
        common_sigma = 0.08
        common = rng.normal(0.0, common_sigma, size=3)
        extra = (common_sigma**2) * np.eye(3)
        s0.P[0:3, 0:3] = s0.P[0:3, 0:3] + extra
        s1.P[0:3, 0:3] = s1.P[0:3, 0:3] + extra
        P0i = SwarmState.from_launch(cfg, 0).P
        P1i = SwarmState.from_launch(cfg, 1).P
        s0.x = s0.x + rng.multivariate_normal(np.zeros(N_STATE), P0i)
        s1.x = s1.x + rng.multivariate_normal(np.zeros(N_STATE), P1i)
        s0.x[IDX_P] = s0.x[IDX_P] + common
        s1.x[IDX_P] = s1.x[IDX_P] + common
        s0.x[IDX_PSI] = wrap_psi(float(s0.x[IDX_PSI]))
        s1.x[IDX_PSI] = wrap_psi(float(s1.x[IDX_PSI]))

        for _k in range(n_steps):
            b0, b1 = s0.copy(), s1.copy()
            z01, d01, az01, el01 = synth_z(p0_true, 0.0, p1_true, rng)
            z10, d10, az10, el10 = synth_z(p1_true, 0.0, p0_true, rng)
            if use_bearing:
                m01 = relpos(
                    s0.p, b1.p, s0.psi, s0.pitch, s0.roll, z01, sigma_d, sigma_az, sigma_el, d01, az01, el01
                )
                m10 = relpos(
                    s1.p, b0.p, s1.psi, s1.pitch, s1.roll, z10, sigma_d, sigma_az, sigma_el, d10, az10, el10
                )
            else:
                m01 = range_only(s0.p, b1.p, d01, sigma_d)
                m10 = range_only(s1.p, b0.p, d10, sigma_d)
            if m01 is not None:
                s0, _ = update(s0, m01, cfg, P_j=b1.P, fusion=fusion)
            if m10 is not None:
                s1, _ = update(s1, m10, cfg, P_j=b0.P, fusion=fusion)

        nees_all.append(position_nees(s0, p0_true))
        nees_all.append(position_nees(s1, p1_true))
        rel_e = (s1.p - s0.p) - rel_true
        Prel = s0.P[0:3, 0:3] + s1.P[0:3, 0:3]
        Prel = 0.5 * (Prel + Prel.T)
        try:
            nees_rel.append(float(rel_e @ np.linalg.solve(Prel, rel_e)))
        except np.linalg.LinAlgError:
            nees_rel.append(float("inf"))
        rel_err.append(float(np.linalg.norm(rel_e)))

    nees = np.asarray(nees_all, dtype=np.float64)
    n_ok = int(np.sum(np.isfinite(nees)))
    mean_nees = float(np.mean(nees[np.isfinite(nees)]))
    lo = chi2_ppf(0.025, 3 * n_ok) / n_ok
    hi = chi2_ppf(0.975, 3 * n_ok) / n_ok
    chi3_lo = chi2_ppf(0.025, 3)
    chi3_hi = chi2_ppf(0.975, 3)
    frac_in = float(np.mean((nees >= chi3_lo) & (nees <= chi3_hi)))
    nr = np.asarray(nees_rel, dtype=np.float64)
    n_rel = int(np.sum(np.isfinite(nr)))
    mean_nees_rel = float(np.mean(nr[np.isfinite(nr)])) if n_rel else float("inf")
    hi_rel = chi2_ppf(0.975, 3 * max(n_rel, 1)) / max(n_rel, 1)
    lo_rel = chi2_ppf(0.025, 3 * max(n_rel, 1)) / max(n_rel, 1)
    frac_rel = float(np.mean((nr >= chi3_lo) & (nr <= chi3_hi))) if n_rel else 0.0
    return {
        "mean_nees": mean_nees,
        "nees_lo": lo,
        "nees_hi": hi,
        "frac_in_95": frac_in,
        "mean_nees_rel": mean_nees_rel,
        "nees_rel_lo": lo_rel,
        "nees_rel_hi": hi_rel,
        "frac_rel_95": frac_rel,
        "rel_p50": float(np.median(rel_err)),
        "rel_p95": float(np.quantile(rel_err, 0.95)),
        "n": n_runs,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    parser.print_help()
    sys.exit(2)


if __name__ == "__main__":
    main()
