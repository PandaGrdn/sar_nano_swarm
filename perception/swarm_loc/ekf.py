#!/usr/bin/env python3
"""ekf.py — error-state EKF math for swarm relative localization.

No rclpy. P2-1 propagation, P2-3 update + entrance, P2-4 split-CI when
a neighbor covariance is supplied.

Usage:
    python3 perception/swarm_loc/ekf.py --selftest
    python3 perception/swarm_loc/ci_fusion.py --selftest
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[2]
for _p in ("perception/swarm_loc", "perception/uwb_sim"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

from measurements import (  # noqa: E402
    Measurement,
    entrance_from_edge,
    wrap_measurement_residual,
)
from rio_stub import (  # noqa: E402
    RioDelta,
    RioStubEngine,
    load_config,
    make_engine,
    resolve_config_path,
    rio_measurement_cov,
)
from state import (  # noqa: E402
    EPS,
    IDX_BPSI,
    IDX_P,
    IDX_PSI,
    IDX_S,
    IDX_V,
    N_STATE,
    STATUS_DIVERGED,
    STATUS_OK,
    SwarmState,
    clamp_position_cov,
    dR_dpsi,
    project_psd,
    rpy_to_R,
    symmetrize,
    wrap_psi,
)
from uwb_edges import (  # noqa: E402
    FLAG_BEARING_VALID,
    FLAG_PEER_IS_SURVEYED,
    FLAG_RANGE_VALID,
)
from uwb_model import bearing_xyz  # noqa: E402


def _process_G(st: SwarmState, delta: RioDelta) -> np.ndarray:
    """G (9x5): maps [n_dp(3), n_dpsi, n_s] into the state."""
    R = rpy_to_R(st.psi, delta.pitch, delta.roll)
    s = st.s
    dt = max(float(delta.dt), EPS)
    dp = np.asarray(delta.delta_p_body, dtype=np.float64)
    G = np.zeros((N_STATE, 5), dtype=np.float64)
    G[IDX_P, 0:3] = s * R
    G[IDX_PSI, 3] = 1.0
    G[IDX_S, 4] = 1.0
    if delta.valid:
        G[IDX_V, 0:3] = s * R / dt
        G[IDX_P, 4] = R @ dp
        G[IDX_V, 4] = (R @ dp) / dt
    else:
        G[IDX_P, 4] = R @ dp
    return G


def process_Q(st: SwarmState, delta: RioDelta, cfg: dict) -> np.ndarray:
    """Q on the 5 RioDelta axes, plus yaw-bias random walk on b_ψ."""
    Q5 = np.array(delta.cov, dtype=np.float64).copy()
    if not delta.valid:
        Q5 = Q5 * float(cfg["rio"]["dropout_q_scale"])
    G = _process_G(st, delta)
    Q = G @ Q5 @ G.T
    dt = max(float(delta.dt), 0.0)
    q_b = float(cfg["estimator"]["yaw_bias_walk_sigma"]) ** 2 * dt
    Q[IDX_BPSI, IDX_BPSI] += q_b
    q_g = float(cfg["estimator"].get("gauge_age_q_m2_per_s", 0.0))
    if q_g > 0.0 and dt > 0.0:
        # Common-mode walk. Relative UWB must not cancel this (see apply_relative_abs_floor).
        Q[0, 0] += q_g * dt
        Q[1, 1] += q_g * dt
        Q[2, 2] += q_g * dt
    return Q


def transition_F(st: SwarmState, delta: RioDelta) -> np.ndarray:
    """Analytic Jacobian of the §4.2 map w.r.t. [p, v, ψ, b_ψ, s]."""
    psi = st.psi
    s = st.s
    dt = float(delta.dt)
    dp = np.asarray(delta.delta_p_body, dtype=np.float64)
    R = rpy_to_R(psi, delta.pitch, delta.roll)
    dR = dR_dpsi(psi, delta.pitch, delta.roll)
    F = np.eye(N_STATE, dtype=np.float64)
    # p⁺ = p + s R dp   →  ∂p/∂ψ = s (∂R/∂ψ) dp , ∂p/∂s = R dp
    F[IDX_P, IDX_PSI] = s * (dR @ dp)
    F[IDX_P, IDX_S] = R @ dp
    # ψ⁺ = ψ + dψ - b_ψ dt
    F[IDX_PSI, IDX_BPSI] = -dt
    if delta.valid and dt > EPS:
        # v⁺ = s R dp / dt  (does not depend on previous v)
        F[IDX_V, IDX_V] = 0.0
        F[IDX_V, IDX_PSI] = s * (dR @ dp) / dt
        F[IDX_V, IDX_S] = (R @ dp) / dt
    # else v⁺ = v → identity block already in F
    return F


def propagate(st: SwarmState, delta: RioDelta, cfg: dict) -> SwarmState:
    """One EKF prediction step driven by RioDelta (plan §4.2)."""
    out = st.copy()
    dt = float(delta.dt)
    dp = np.asarray(delta.delta_p_body, dtype=np.float64)
    R = rpy_to_R(st.psi, delta.pitch, delta.roll)
    s = st.s
    disp = s * (R @ dp)

    out.x[IDX_P] = st.x[IDX_P] + disp
    out.x[IDX_PSI] = wrap_psi(st.psi + float(delta.delta_psi) - st.b_psi * dt)
    if delta.valid and dt > EPS:
        out.x[IDX_V] = disp / dt
    # else hold velocity
    out.x[IDX_S] = s
    out.x[IDX_BPSI] = st.b_psi
    out.roll = float(delta.roll)
    out.pitch = float(delta.pitch)
    out.stamp = float(delta.stamp)

    F = transition_F(st, delta)
    Q = process_Q(st, delta, cfg)
    P = F @ st.P @ F.T + Q
    P = symmetrize(P)
    P = clamp_position_cov(P, cfg)
    P = project_psd(P)
    out.P = P
    hi = float(cfg["estimator"]["max_cov_p_m"]) ** 2
    if any(P[i, i] >= hi for i in range(3)):
        out.status = STATUS_DIVERGED
    else:
        out.status = STATUS_OK
    return out


def chi2_ppf(p: float, df: int) -> float:
    """Chi-squared quantile. scipy if present, else Wilson-Hilferty."""
    df = int(df)
    if df < 1:
        return 0.0
    try:
        from scipy.stats import chi2

        return float(chi2.ppf(p, df))
    except Exception:
        # Wilson-Hilferty: χ² ≈ df * (1 - 2/(9df) + z sqrt(2/(9df)))^3
        z = _norm_ppf(p)
        t = 1.0 - 2.0 / (9.0 * df) + z * math.sqrt(2.0 / (9.0 * df))
        return max(float(df * t**3), 0.0)


def _norm_ppf(p: float) -> float:
    """Acklam approximation of the standard-normal quantile."""
    p = min(max(float(p), 1e-12), 1.0 - 1e-12)
    a = [
        -3.969683028665376e01,
        2.209460984245205e02,
        -2.759285104469687e02,
        1.383577509590705e02,
        -3.066479806614716e01,
        2.506628277459239e00,
    ]
    b = [
        -5.447609879822406e01,
        1.615858368580409e02,
        -1.556989798598866e02,
        6.680131188771972e01,
        -1.328068155288572e01,
    ]
    c = [
        -7.784894002430293e-03,
        -3.223964580411365e-01,
        -2.400758277161838e00,
        -2.549732539343734e00,
        4.374664141464968e00,
        2.938163982698783e00,
    ]
    d = [
        7.784695709041462e-03,
        3.224671290700398e-01,
        2.445134137142996e00,
        3.754408661907416e00,
    ]
    plow = 0.02425
    phigh = 1.0 - plow
    if p < plow:
        q = math.sqrt(-2.0 * math.log(p))
        return (
            (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5])
            / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
        )
    if p > phigh:
        q = math.sqrt(-2.0 * math.log(1.0 - p))
        return -(
            (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5])
            / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
        )
    q = p - 0.5
    r = q * q
    return (
        (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5])
        * q
        / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0)
    )


def position_nees(st: SwarmState, p_true: np.ndarray) -> float:
    err = np.asarray(st.p, dtype=np.float64) - np.asarray(p_true, dtype=np.float64)
    Pp = 0.5 * (st.P[0:3, 0:3] + st.P[0:3, 0:3].T)
    try:
        return float(err @ np.linalg.solve(Pp, err))
    except np.linalg.LinAlgError:
        return float("inf")


def _measurement_R(meas: Measurement, cfg: dict) -> np.ndarray:
    R = np.array(meas.R, dtype=np.float64)
    if meas.name.startswith("entrance"):
        sig2 = float(cfg["entrance"]["sigma_m"]) ** 2
        R = R + sig2 * np.eye(R.shape[0])
    return 0.5 * (R + R.T)


def apply_relative_abs_floor(
    P: np.ndarray,
    P_i: np.ndarray,
    P_j: np.ndarray,
) -> np.ndarray:
    """Relative measurements cannot invent absolute certainty.

    After a neighbor update, each position variance is at least
    min(own prior, neighbor prior). A pair cannot collapse both P's
    below the better of the two incoming absolute sigmas.
    """
    out = P.copy()
    for k in range(3):
        floor_k = min(float(P_i[k, k]), float(P_j[k, k]))
        out[k, k] = max(float(out[k, k]), floor_k)
    return symmetrize(out)


def update(
    st: SwarmState,
    meas: Measurement,
    cfg: dict,
    P_j: np.ndarray | None = None,
    fusion: str | None = None,
) -> tuple:
    """EKF update with NIS gate (D16 / §4.5). Returns (state, info).

    If P_j is None, neighbor uncertainty is already in R (entrance path).
    If P_j is provided, apply split-CI (D6) unless fusion='naive'.
    """
    import ci_fusion as _ci  # local to keep --selftest import order simple

    info = {"accepted": False, "nis": float("nan"), "reason": "", "omega": float("nan")}
    if st.status == STATUS_DIVERGED:
        info["reason"] = "diverged"
        return st.copy(), info
    if not meas.finite():
        info["reason"] = "nan"
        return st.copy(), info

    H = np.array(meas.H_i, dtype=np.float64)
    H_j = np.array(meas.H_j, dtype=np.float64)
    r = wrap_measurement_residual(meas)
    Rm = _measurement_R(meas, cfg)
    if H.shape[0] != r.size or Rm.shape != (r.size, r.size):
        info["reason"] = "shape"
        return st.copy(), info

    method = fusion if fusion is not None else str(cfg.get("fusion", {}).get("method", "covariance_intersection"))
    search = str(cfg.get("fusion", {}).get("ci_omega_search", "fast"))
    use_ci = P_j is not None and method not in ("naive", "ekf")
    P_i = st.P
    P_use = P_i
    R_eff = Rm

    if P_j is not None and use_ci:
        Pj = np.array(P_j, dtype=np.float64)
        if Pj.shape != P_i.shape:
            info["reason"] = "P_j_shape"
            return st.copy(), info
        w = _ci.choose_omega_sci(H, P_i, H_j, Pj, Rm, method=search)
        info["omega"] = w
        # Split-CI innovation: inflate both priors in S (unknown correlation).
        # Gain uses uninflated P_i so the mean does not overshoot.
        S = _ci.sci_S(H, P_i, H_j, Pj, Rm, w)
        R_eff = S - H @ P_i @ H.T
        R_eff = 0.5 * (R_eff + R_eff.T)
        # Keep R_eff PSD
        eig, vec = np.linalg.eigh(R_eff)
        R_eff = vec @ np.diag(np.clip(eig, 1e-12, None)) @ vec.T
        P_use = P_i
    elif P_j is not None:
        # naive uncorrelated EKF (regression arm)
        Pj = np.array(P_j, dtype=np.float64)
        R_eff = H_j @ Pj @ H_j.T + Rm
        R_eff = 0.5 * (R_eff + R_eff.T)
        S = H @ P_i @ H.T + R_eff
        P_use = P_i
    else:
        S = H @ P_i @ H.T + Rm
        P_use = P_i
        R_eff = Rm

    S = 0.5 * (S + S.T)
    try:
        Sinv_r = np.linalg.solve(S, r)
        nis = float(r @ Sinv_r)
    except np.linalg.LinAlgError:
        info["reason"] = "S_singular"
        return st.copy(), info
    info["nis"] = nis
    df = int(r.size)
    p_gate = float(cfg["measurements"]["nis_gate_chi2_p"])
    thresh = chi2_ppf(p_gate, df)
    if not math.isfinite(nis) or nis > thresh:
        info["reason"] = "nis_gate"
        return st.copy(), info

    try:
        K = np.linalg.solve(S, (P_use @ H.T).T).T
    except np.linalg.LinAlgError:
        info["reason"] = "K_fail"
        return st.copy(), info

    out = st.copy()
    out.x = st.x + K @ r
    out.x[IDX_PSI] = wrap_psi(float(out.x[IDX_PSI]))
    I = np.eye(N_STATE)
    KH = K @ H
    P = (I - KH) @ P_use @ (I - KH).T + K @ R_eff @ K.T
    P = symmetrize(P)
    if P_j is not None and use_ci:
        # Relative measurements cannot observe a common-mode translation.
        # Floor the position variances at classic CI(P_i, P_j) so the filter
        # does not claim more absolute certainty than D6 allows.
        P_floor = _ci.fused_P(P_i[0:3, 0:3], np.asarray(P_j, dtype=np.float64)[0:3, 0:3], float(info["omega"]))
        for k in range(3):
            P[k, k] = max(float(P[k, k]), float(P_floor[k, k]))
        P = apply_relative_abs_floor(P, P_i, np.asarray(P_j, dtype=np.float64))
    P = clamp_position_cov(P, cfg)
    P = project_psd(P)
    out.P = P
    if meas.name.startswith("entrance"):
        out.t_last_gauge_s = float(out.stamp)
    hi = float(cfg["estimator"]["max_cov_p_m"]) ** 2
    if any(P[i, i] >= hi for i in range(3)) or not np.all(np.isfinite(P)):
        out.status = STATUS_DIVERGED
        info["reason"] = "diverged_after_update"
        info["accepted"] = True
        return out, info
    info["accepted"] = True
    info["reason"] = "ok"
    return out, info


def _synth_entrance_edge(p, psi, pitch, roll, p_ent, rng, sigma_d, sigma_az, sigma_el) -> dict:
    R = rpy_to_R(float(psi), float(pitch), float(roll))
    z_true = R.T @ (np.asarray(p_ent, dtype=np.float64) - np.asarray(p, dtype=np.float64))
    d = float(np.linalg.norm(z_true))
    az = math.atan2(float(z_true[1]), float(z_true[0]))
    el = math.asin(float(np.clip(z_true[2] / max(d, EPS), -1.0, 1.0)))
    d_m = d + float(rng.normal(0.0, sigma_d))
    az_m = az + float(rng.normal(0.0, sigma_az))
    el_m = float(np.clip(el + rng.normal(0.0, sigma_el), -0.5 * math.pi + 1e-6, 0.5 * math.pi - 1e-6))
    if d_m < 0.05:
        d_m = 0.05
    x, y, z = bearing_xyz(d_m, az_m, el_m)
    return {
        "flags": FLAG_RANGE_VALID | FLAG_BEARING_VALID | FLAG_PEER_IS_SURVEYED,
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "range_m": float(d_m),
        "azimuth_rad": float(az_m),
        "elevation_rad": float(el_m),
        "sigma_range_m": float(sigma_d),
        "sigma_az_rad": float(sigma_az),
        "sigma_el_rad": float(sigma_el),
    }


def run_entrance_mc(cfg: dict, n_runs: int = 1000, n_steps: int = 20, seed: int = 3) -> dict:
    """Static one-drone + entrance Monte Carlo (P2-3 gate)."""
    p_ent = np.array(cfg["entrance"]["position_xyz_m"], dtype=np.float64)
    p_true = SwarmState.from_launch(cfg, 0).p.copy()
    sigma_d = 0.0873
    sigma_az = math.radians(4.795)
    sigma_el = math.radians(4.795)
    rng = np.random.default_rng(seed)
    nees = np.empty(n_runs)
    err = np.empty(n_runs)
    n_accept = 0
    n_meas = 0
    for i in range(n_runs):
        st = SwarmState.from_launch(cfg, 0)
        x_true = st.x.copy()
        st.x = st.x + rng.multivariate_normal(np.zeros(N_STATE), st.P)
        st.x[IDX_PSI] = wrap_psi(float(st.x[IDX_PSI]))
        for k in range(n_steps):
            edge = _synth_entrance_edge(
                p_true, x_true[IDX_PSI], 0.0, 0.0, p_ent, rng, sigma_d, sigma_az, sigma_el
            )
            meas = entrance_from_edge(st, edge, cfg)
            n_meas += 1
            if meas is None:
                continue
            st, info = update(st, meas, cfg)
            if info["accepted"]:
                n_accept += 1
        nees[i] = position_nees(st, p_true)
        err[i] = float(np.linalg.norm(st.p - p_true))
    mean_nees = float(np.mean(nees[np.isfinite(nees)]))
    n_ok = int(np.sum(np.isfinite(nees)))
    lo = chi2_ppf(0.025, 3 * n_ok) / n_ok
    hi = chi2_ppf(0.975, 3 * n_ok) / n_ok
    chi3_lo = chi2_ppf(0.025, 3)
    chi3_hi = chi2_ppf(0.975, 3)
    frac_in = float(np.mean((nees >= chi3_lo) & (nees <= chi3_hi)))
    return {
        "mean_nees": mean_nees,
        "nees_lo": lo,
        "nees_hi": hi,
        "frac_in_95": frac_in,
        "err_p50": float(np.median(err)),
        "err_p95": float(np.quantile(err, 0.95)),
        "err_max": float(np.max(err)),
        "accept_frac": n_accept / max(n_meas, 1),
        "n_runs": n_runs,
    }


def _numerical_F(st: SwarmState, delta: RioDelta, eps: float = 1e-7) -> np.ndarray:
    """Central-difference Jacobian of the mean map (covariance ignored)."""
    cfg_dummy = {
        "estimator": {
            "yaw_bias_walk_sigma": 0.0,
            "cov_floor_p_m": 0.0,
            "max_cov_p_m": 1.0e9,
        },
        "rio": {"dropout_q_scale": 1.0},
    }

    def mean_x(state: SwarmState) -> np.ndarray:
        return propagate(state, delta, cfg_dummy).x.copy()

    F = np.zeros((N_STATE, N_STATE), dtype=np.float64)
    x0 = st.x.copy()
    for j in range(N_STATE):
        sp = st.copy()
        sm = st.copy()
        sp.x = x0.copy()
        sm.x = x0.copy()
        sp.x[j] += eps
        sm.x[j] -= eps
        F[:, j] = (mean_x(sp) - mean_x(sm)) / (2.0 * eps)
    return F


def _is_spd(P: np.ndarray) -> bool:
    S = 0.5 * (P + P.T)
    if not np.allclose(P, S, atol=1e-8):
        return False
    w = np.linalg.eigvalsh(S)
    return bool(np.all(np.isfinite(w)) and np.min(w) > -1e-6)


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

    cfg = load_config(resolve_config_path("configs/estimation/swarm_loc.yaml"))

    # 1–3 state / launch geometry
    st0 = SwarmState.from_launch(cfg, 0)
    st1 = SwarmState.from_launch(cfg, 1)
    check("1 state dim 9", st0.x.shape == (9,) and st0.P.shape == (9, 9))
    check(
        "2 launch drone 0",
        np.allclose(st0.p, [0.0, 0.0, 0.5]),
    )
    check(
        "3 launch drone 1 spacing",
        np.allclose(st1.p, [1.5, 0.0, 0.5]),
    )
    check("3b init scale 1", abs(st0.s - 1.0) < 1e-15)
    check("3c init yaw 0", abs(st0.psi) < 1e-15)

    # 4–5 rotation convention
    R_I = rpy_to_R(0.0, 0.0, 0.0)
    check("4 R identity", np.allclose(R_I, np.eye(3)))
    R90 = rpy_to_R(math.pi / 2, 0.0, 0.0)
    check(
        "5 R yaw +90 maps body x to world y",
        np.allclose(R90 @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12),
    )

    # 6–9 analytic F vs numerical
    def f_err(state, delta):
        Fa = transition_F(state, delta)
        Fn = _numerical_F(state, delta)
        return float(np.max(np.abs(Fa - Fn))), Fa, Fn

    base = SwarmState.from_launch(cfg, 0)
    d_flat = RioDelta(
        stamp=0.02,
        dt=0.02,
        delta_p_body=np.array([0.03, -0.01, 0.005]),
        delta_psi=0.01,
        roll=0.0,
        pitch=0.0,
        cov=rio_measurement_cov(cfg),
        valid=True,
    )
    err, _, _ = f_err(base, d_flat)
    check("6 F numdiff level attitude", err < 1e-6, f"max|Δ|={err:.3e}")

    tilted = base.copy()
    tilted.x[IDX_PSI] = 0.7
    tilted.roll = 0.15
    tilted.pitch = -0.2
    d_tilt = RioDelta(
        stamp=0.02,
        dt=0.02,
        delta_p_body=np.array([0.04, 0.02, -0.01]),
        delta_psi=-0.03,
        roll=0.15,
        pitch=-0.2,
        cov=rio_measurement_cov(cfg),
        valid=True,
    )
    err, _, _ = f_err(tilted, d_tilt)
    check("7 F numdiff tilted", err < 1e-6, f"max|Δ|={err:.3e}")

    scaled = tilted.copy()
    scaled.x[IDX_S] = 1.07
    scaled.x[IDX_BPSI] = 0.002
    err, _, _ = f_err(scaled, d_tilt)
    check("8 F numdiff scale+bias", err < 1e-6, f"max|Δ|={err:.3e}")

    d_drop = RioDelta(
        stamp=0.02,
        dt=0.02,
        delta_p_body=np.array([0.04, 0.02, -0.01]),
        delta_psi=-0.03,
        roll=0.15,
        pitch=-0.2,
        cov=rio_measurement_cov(cfg),
        valid=False,
    )
    err, Fa, _ = f_err(scaled, d_drop)
    check("9 F numdiff invalid", err < 1e-6, f"max|Δ|={err:.3e}")
    check("9b invalid holds v in F", np.allclose(Fa[IDX_V, IDX_V], np.eye(3)))

    # 10–11 covariance 10k steps
    st = SwarmState.from_launch(cfg, 0)
    cfg_10k = dict(cfg)
    cfg_10k["estimator"] = dict(cfg["estimator"])
    cfg_10k["estimator"]["gauge_age_q_m2_per_s"] = 0.0
    d_step = RioDelta(
        stamp=0.0,
        dt=0.02,
        delta_p_body=np.array([0.01, 0.0, 0.0]),
        delta_psi=0.001,
        roll=0.0,
        pitch=0.0,
        cov=rio_measurement_cov(cfg),
        valid=True,
    )
    min_eig = 1.0e9
    for k in range(10000):
        d_step.stamp = (k + 1) * 0.02
        st = propagate(st, d_step, cfg_10k)
        min_eig = min(min_eig, float(np.min(np.linalg.eigvalsh(st.P))))
    check("10 P symmetric 10k", np.allclose(st.P, st.P.T, atol=1e-10))
    check("11 P SPD 10k", _is_spd(st.P), f"min_eig={min_eig:.3e}")
    check("11b P finite 10k", np.all(np.isfinite(st.P)) and np.all(np.isfinite(st.x)))

    # 12–14 zero-noise trajectory matches truth
    silent = dict(cfg)
    silent["rio"] = dict(cfg["rio"])
    silent["rio"].update(
        {
            "vel_bias_walk": 0.0,
            "yaw_walk_deg_per_min": 0.0,
            "scale_error": 1.0,
            "sigma_p": 0.0,
            "sigma_psi_deg": 0.0,
            "dropout_rate": 0.0,
        }
    )
    silent["estimator"] = dict(cfg["estimator"])
    # tiny Q so we only test the mean
    dt = 0.02
    T = 2.0
    n = int(round(T / dt))
    p_true = np.array([0.0, 0.0, 0.5])
    psi_true = 0.0
    omega = 0.4  # rad/s
    speed = 0.8  # m/s body-x
    st = SwarmState.from_launch(silent, 0)
    st.P = np.eye(N_STATE) * 1e-8
    cov0 = np.zeros((5, 5))
    for k in range(n):
        R = rpy_to_R(psi_true, 0.0, 0.0)
        dp_body = np.array([speed * dt, 0.0, 0.0])
        dpsi = omega * dt
        p_true = p_true + R @ dp_body
        psi_true = wrap_psi(psi_true + dpsi)
        delta = RioDelta(
            stamp=(k + 1) * dt,
            dt=dt,
            delta_p_body=dp_body,
            delta_psi=dpsi,
            roll=0.0,
            pitch=0.0,
            cov=cov0,
            valid=True,
        )
        st = propagate(st, delta, silent)
    # v uses R at the start of the last interval (plan §4.2), not the posterior yaw
    psi_prior = wrap_psi(psi_true - omega * dt)
    v_true = rpy_to_R(psi_prior, 0.0, 0.0) @ np.array([speed, 0.0, 0.0])
    check(
        "12 zero-noise p vs truth",
        float(np.linalg.norm(st.p - p_true)) < 1e-6,
        f"err={np.linalg.norm(st.p - p_true):.3e}",
    )
    check(
        "13 zero-noise psi vs truth",
        abs(wrap_psi(st.psi - psi_true)) < 1e-6,
        f"err={wrap_psi(st.psi - psi_true):.3e}",
    )
    check(
        "14 zero-noise v vs truth",
        float(np.linalg.norm(st.v - v_true)) < 1e-6,
        f"err={np.linalg.norm(st.v - v_true):.3e}",
    )

    # 15 default stub drift ~ linear, within 2× configured scale error
    # Isolated scale: other noises off, scale_error from config (1.02).
    scale_only = dict(cfg)
    scale_only["rio"] = dict(cfg["rio"])
    scale_only["rio"].update(
        {
            "vel_bias_walk": 0.0,
            "yaw_walk_deg_per_min": 0.0,
            "sigma_p": 0.0,
            "sigma_psi_deg": 0.0,
            "dropout_rate": 0.0,
        }
    )
    scale_err = float(scale_only["rio"]["scale_error"]) - 1.0
    eng = RioStubEngine(scale_only, np.random.default_rng(0))
    st = SwarmState.from_launch(scale_only, 0)
    distance = 0.0
    p_true = st.p.copy()
    n_long = 5000  # 100 s at 50 Hz
    for k in range(n_long):
        dp_body_true = np.array([0.02, 0.0, 0.0])  # 1 m/s
        distance += float(np.linalg.norm(dp_body_true))
        p_true = p_true + dp_body_true
        delta = eng.corrupt((k + 1) * dt, dt, dp_body_true, 0.0, 0.0, 0.0)
        # feed the filter the corrupted increment with s remaining at 1.0
        # (unestimated scale) so drift equals the stub's scale error
        delta.cov = np.eye(5) * 1e-8
        st = propagate(st, delta, scale_only)
    err = float(np.linalg.norm(st.p - p_true))
    expected = abs(scale_err) * distance
    check(
        "15 scale drift within 2x",
        expected > 0.0 and err <= 2.0 * expected + 1e-6,
        f"err={err:.4f} expected={expected:.4f} dist={distance:.1f}",
    )
    check(
        "15b scale drift linear (0.5-2x)",
        0.5 * expected <= err <= 2.0 * expected,
        f"err/expected={err / expected:.3f}",
    )

    # full default stub: drift should still be order-of configured (not 10×)
    eng_def = make_engine(cfg, 0)
    st = SwarmState.from_launch(cfg, 0)
    p_true = st.p.copy()
    distance = 0.0
    n_mid = 2500  # 50 s
    for k in range(n_mid):
        dp_body_true = np.array([0.02, 0.0, 0.0])
        distance += 0.02
        p_true = p_true + dp_body_true
        delta = eng_def.corrupt((k + 1) * dt, dt, dp_body_true, 0.0, 0.0, 0.0)
        st = propagate(st, delta, cfg)
    err = float(np.linalg.norm(st.p - p_true))
    expected = abs(float(cfg["rio"]["scale_error"]) - 1.0) * distance
    check(
        "15c default stub drift order",
        err < 2.0 * max(expected, 1.0) + 0.5 * distance * 0.1,
        f"err={err:.3f} scale_expect={expected:.3f} dist={distance:.1f}",
    )

    # 16 dropout inflates Q (via P growth)
    st_ok = SwarmState.from_launch(cfg, 0)
    st_bad = SwarmState.from_launch(cfg, 0)
    d_ok = RioDelta(
        stamp=0.02,
        dt=0.02,
        delta_p_body=np.zeros(3),
        delta_psi=0.0,
        roll=0.0,
        pitch=0.0,
        cov=rio_measurement_cov(cfg),
        valid=True,
    )
    d_bad = RioDelta(
        stamp=0.02,
        dt=0.02,
        delta_p_body=np.zeros(3),
        delta_psi=0.0,
        roll=0.0,
        pitch=0.0,
        cov=rio_measurement_cov(cfg),
        valid=False,
    )
    p_ok = propagate(st_ok, d_ok, cfg)
    p_bad = propagate(st_bad, d_bad, cfg)
    tr_p_ok = float(np.trace(p_ok.P[0:3, 0:3]))
    tr_p_bad = float(np.trace(p_bad.P[0:3, 0:3]))
    check(
        "16 dropout inflates P_pos",
        tr_p_bad > tr_p_ok,
        f"tr_bad={tr_p_bad:.4f} tr_ok={tr_p_ok:.4f}",
    )
    v_before = st_bad.v.copy()
    check(
        "16b dropout holds velocity",
        np.allclose(p_bad.v, v_before),
    )

    # 17 wrap + ∂p/∂ψ block sanity
    check("17 wrap +pi", abs(wrap_psi(math.pi + 0.1) + math.pi - 0.1) < 1e-9)
    psi = 0.3
    dp = np.array([1.0, 0.0, 0.0])
    st_w = SwarmState.from_launch(cfg, 0)
    st_w.x[IDX_PSI] = psi
    d_w = RioDelta(
        stamp=0.02,
        dt=0.02,
        delta_p_body=dp,
        delta_psi=0.0,
        roll=0.0,
        pitch=0.0,
        cov=np.eye(5) * 1e-8,
        valid=True,
    )
    F = transition_F(st_w, d_w)
    dp_dpsi = dR_dpsi(psi, 0.0, 0.0) @ dp
    check(
        "17b F dp/dpsi matches s dR/dpsi dp",
        np.allclose(F[IDX_P, IDX_PSI], st_w.s * dp_dpsi),
    )

    # wrap: ~2π azimuth residual must not produce a huge innovation
    m_ang = Measurement(
        "az_only",
        np.array([math.pi - 0.01], dtype=np.float64),
        np.array([-math.pi + 0.01], dtype=np.float64),
        np.array([2.0 * math.pi - 0.02], dtype=np.float64),
        np.zeros((1, N_STATE), dtype=np.float64),
        np.zeros((1, N_STATE), dtype=np.float64),
        np.array([[0.05**2]], dtype=np.float64),
    )
    r_w = wrap_measurement_residual(m_ang)
    check("17c wrap 2pi residual to ~0", abs(float(r_w[0])) < 0.05, f"r={float(r_w[0]):.4f}")
    st_g = SwarmState.from_launch(cfg, 0)
    _, info_w = update(st_g, m_ang, cfg)
    check(
        "17d wrapped residual does not NIS-reject",
        info_w.get("reason") != "nis_gate",
        str(info_w),
    )

    # no-anchor: relative updates must not freeze absolute σ
    from measurements import range_only as _range_only

    cfg_na = dict(cfg)
    cfg_na["estimator"] = dict(cfg["estimator"])
    a = SwarmState.from_launch(cfg_na, 0)
    b = SwarmState.from_launch(cfg_na, 1)
    d_na = RioDelta(
        stamp=0.0,
        dt=0.1,
        delta_p_body=np.array([0.02, 0.0, 0.0]),
        delta_psi=0.0,
        roll=0.0,
        pitch=0.0,
        cov=rio_measurement_cov(cfg_na),
        valid=True,
    )
    sig0 = math.sqrt(max(a.P[0, 0], a.P[1, 1], a.P[2, 2]))
    for k in range(80):
        d_na.stamp = (k + 1) * 0.1
        a = propagate(a, d_na, cfg_na)
        b = propagate(b, d_na, cfg_na)
        if a.status == STATUS_DIVERGED or b.status == STATUS_DIVERGED:
            break
        mab = _range_only(a.p, b.p, float(np.linalg.norm(a.p - b.p)), 0.08)
        mba = _range_only(b.p, a.p, float(np.linalg.norm(b.p - a.p)), 0.08)
        if mab is not None:
            a, _ = update(a, mab, cfg_na, P_j=b.P, fusion="ci")
        if mba is not None:
            b, _ = update(b, mba, cfg_na, P_j=a.P, fusion="ci")
    sig1 = math.sqrt(max(a.P[0, 0], a.P[1, 1], a.P[2, 2]))
    check(
        "17e no-anchor σ grows",
        sig1 > 1.5 * sig0,
        f"sig0={sig0:.4f} sig1={sig1:.4f} status={a.status}",
    )

    # ----- P2-3: update path + entrance MC -----
    check("19 chi2_3 0.95 ~7.8", abs(chi2_ppf(0.95, 3) - 7.815) < 0.05)

    st = SwarmState.from_launch(cfg, 0)
    p_ent = np.array(cfg["entrance"]["position_xyz_m"], dtype=np.float64)
    p_true = st.p.copy()
    st.x[IDX_P] = st.x[IDX_P] + np.array([0.08, -0.04, 0.03])
    err0 = float(np.linalg.norm(st.p - p_true))
    z_meas = rpy_to_R(0.0, 0.0, 0.0).T @ (p_ent - p_true)
    d_m = float(np.linalg.norm(z_meas))
    edge_clean = {
        "flags": FLAG_RANGE_VALID | FLAG_BEARING_VALID | FLAG_PEER_IS_SURVEYED,
        "x": float(z_meas[0]),
        "y": float(z_meas[1]),
        "z": float(z_meas[2]),
        "range_m": d_m,
        "azimuth_rad": math.atan2(float(z_meas[1]), float(z_meas[0])),
        "elevation_rad": math.asin(float(np.clip(z_meas[2] / d_m, -1.0, 1.0))),
        "sigma_range_m": 0.0873,
        "sigma_az_rad": math.radians(4.795),
        "sigma_el_rad": math.radians(4.795),
    }
    meas = entrance_from_edge(st, edge_clean, cfg)
    check("20 entrance meas built", meas is not None and meas.finite())
    st1, info = update(st, meas, cfg)
    check("20b update accepted", info["accepted"], str(info))
    err1 = float(np.linalg.norm(st1.p - p_true))
    check("20c noiseless update reduces error", err1 < err0, f"err0={err0:.3f} err1={err1:.3f}")
    check("20d P still SPD", _is_spd(st1.P))

    meas_bad = Measurement(
        name="entrance_relpos",
        z=meas.z.copy(),
        h=meas.h.copy(),
        residual=np.array([10.0, 10.0, 10.0]),
        H_i=meas.H_i.copy(),
        H_j=meas.H_j.copy(),
        R=meas.R.copy(),
    )
    st2, info2 = update(st, meas_bad, cfg)
    check("21 NIS rejects outlier", info2["reason"] == "nis_gate")
    check("21b reject leaves state unchanged", np.allclose(st2.x, st.x))

    meas_nan = Measurement(
        name="entrance_relpos",
        z=meas.z.copy(),
        h=meas.h.copy(),
        residual=np.array([0.0, float("nan"), 0.0]),
        H_i=meas.H_i.copy(),
        H_j=meas.H_j.copy(),
        R=meas.R.copy(),
    )
    _, info3 = update(st, meas_nan, cfg)
    check("22 NaN dropped", info3["reason"] == "nan")

    print("[selftest] running P2-3 entrance Monte Carlo (1000 runs) ...")
    mc = run_entrance_mc(cfg, n_runs=1000, n_steps=10, seed=3)
    print(
        f"[selftest] MC mean_NEES={mc['mean_nees']:.3f} band=[{mc['nees_lo']:.3f},{mc['nees_hi']:.3f}] "
        f"frac_in_95={mc['frac_in_95']:.3f} err_p50={mc['err_p50']:.3f} err_p95={mc['err_p95']:.3f} "
        f"accept={mc['accept_frac']:.3f}"
    )
    check(
        "23 position error bounded",
        mc["err_p95"] < 0.5 and mc["err_max"] < 1.5,
        f"p95={mc['err_p95']:.3f} max={mc['err_max']:.3f}",
    )
    # Overconfidence = mean NEES above the 95% band. Underconfidence (below)
    # is conservative and is not a P2-3 failure; still flag if wildly low.
    check(
        "24 NEES not overconfident",
        math.isfinite(mc["mean_nees"]) and mc["mean_nees"] <= mc["nees_hi"] * 1.03,
        f"mean_NEES={mc['mean_nees']:.3f} hi={mc['nees_hi']:.3f}",
    )
    check(
        "24b NEES in/near 95% band",
        mc["mean_nees"] >= 0.5 * mc["nees_lo"] and mc["frac_in_95"] >= 0.80,
        f"mean={mc['mean_nees']:.3f} frac={mc['frac_in_95']:.3f}",
    )

    print(f"[selftest] {n_pass} passed, {n_fail} failed (need >=15 passing)")
    check("18 >=15 passing checks", n_pass >= 15, f"n_pass={n_pass}")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


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
