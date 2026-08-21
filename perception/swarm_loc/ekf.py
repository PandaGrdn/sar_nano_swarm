#!/usr/bin/env python3
"""ekf.py — error-state EKF math for swarm relative localization (P2-1).

No rclpy. This milestone implements propagation only (plan §4.2).
Measurement updates land in P2-2 / P2-3.

Usage:
    python3 perception/swarm_loc/ekf.py --selftest
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
    rpy_to_R,
    symmetrize,
    wrap_psi,
)


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
    out.P = P
    hi = float(cfg["estimator"]["max_cov_p_m"]) ** 2
    if any(P[i, i] >= hi for i in range(3)):
        out.status = STATUS_DIVERGED
    else:
        out.status = STATUS_OK
    return out


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
    if not np.allclose(P, P.T, atol=1e-10):
        return False
    try:
        np.linalg.cholesky(P + 1e-12 * np.eye(P.shape[0]))
        return True
    except np.linalg.LinAlgError:
        return False


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
        st = propagate(st, d_step, cfg)
        min_eig = min(min_eig, float(np.min(np.linalg.eigvalsh(st.P))))
    check("10 P symmetric 10k", np.allclose(st.P, st.P.T, atol=1e-10))
    check("11 P SPD 10k", _is_spd(st.P) and min_eig > -1e-9, f"min_eig={min_eig:.3e}")
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
