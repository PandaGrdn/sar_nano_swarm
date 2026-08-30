#!/usr/bin/env python3
"""d8_d11.py — P2-6 offline gates: reciprocal bearing (D8) + mutual yaw (D11).

D8 (blocking): corridor, drone 0 sees drone 1 (bearing), drone 1 sees only
the rear (range / range-rate). Drone 1 with the rebroadcast bearing must beat
range-only and d·d'.

D11 (correctness, not live pair-rate): facing observers, mutual-yaw on vs off.
Live ±45° cone pair rate is reported by swarm_loc_node, not gated here.

Usage:
    python3 perception/swarm_loc/d8_d11.py --selftest
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

from ekf import update  # noqa: E402
from measurements import (  # noqa: E402
    cartesian_cov_from_spherical,
    mutual_yaw,
    range_only,
    range_rate,
    reciprocal_relpos,
    relpos,
)
from rio_stub import load_config, resolve_config_path  # noqa: E402
from state import IDX_P, IDX_PSI, N_STATE, SwarmState, rpy_to_R, wrap_psi  # noqa: E402
from uwb_model import bearing_xyz  # noqa: E402

SIGMA_D = 0.0873
SIGMA_AZ = math.radians(4.795)
SIGMA_EL = math.radians(4.795)


def _synth_bearing(p_obs, psi_obs, pitch, roll, p_peer, rng):
    R = rpy_to_R(float(psi_obs), float(pitch), float(roll))
    z_true = R.T @ (np.asarray(p_peer, dtype=np.float64) - np.asarray(p_obs, dtype=np.float64))
    d = float(np.linalg.norm(z_true))
    az = math.atan2(float(z_true[1]), float(z_true[0]))
    el = math.asin(float(np.clip(z_true[2] / max(d, 1e-12), -1.0, 1.0)))
    d_m = max(d + float(rng.normal(0.0, SIGMA_D)), 0.05)
    az_m = az + float(rng.normal(0.0, SIGMA_AZ))
    el_m = float(np.clip(el + rng.normal(0.0, SIGMA_EL), -1.56, 1.56))
    x, y, z = bearing_xyz(d_m, az_m, el_m)
    return np.array([x, y, z]), d_m, az_m, el_m


def run_d8_corridor_mc(
    cfg: dict,
    mode: str,
    n_runs: int = 200,
    n_steps: int = 18,
    seed: int = 6,
) -> dict:
    """Two-filter corridor: drone 0 has bearing; drone 1 does not.

    mode for drone 1: reciprocal | range | range_rate
    """
    rng = np.random.default_rng(seed)
    p0 = SwarmState.from_launch(cfg, 0).p.copy()
    p1 = SwarmState.from_launch(cfg, 1).p.copy()
    err1 = []
    rel_err = []
    for _ in range(n_runs):
        s0 = SwarmState.from_launch(cfg, 0)
        s1 = SwarmState.from_launch(cfg, 1)
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
        # Lateral miss on the rear drone — range is a sphere, bearing is not.
        s1.x[1] = s1.x[1] + 0.30
        s1.P[1, 1] = s1.P[1, 1] + 0.30**2
        s0.x[IDX_PSI] = wrap_psi(float(s0.x[IDX_PSI]))
        s1.x[IDX_PSI] = wrap_psi(float(s1.x[IDX_PSI]))
        for _k in range(n_steps):
            b0, b1 = s0.copy(), s1.copy()
            z01, d01, az01, el01 = _synth_bearing(p0, 0.0, 0.0, 0.0, p1, rng)
            m01 = relpos(
                s0.p, b1.p, s0.psi, s0.pitch, s0.roll, z01, SIGMA_D, SIGMA_AZ, SIGMA_EL, d01, az01, el01
            )
            s0, _ = update(s0, m01, cfg, P_j=b1.P, fusion="ci")
            if mode == "reciprocal":
                m10 = reciprocal_relpos(
                    s1.p,
                    b0.p,
                    b0.psi,
                    b0.pitch,
                    b0.roll,
                    z01,
                    SIGMA_D,
                    SIGMA_AZ,
                    SIGMA_EL,
                    d01,
                    az01,
                    el01,
                )
            else:
                m10 = range_only(s1.p, b0.p, d01, SIGMA_D)
            if m10 is not None:
                s1, _ = update(s1, m10, cfg, P_j=b0.P, fusion="ci")
            if mode == "range_rate":
                mrr = range_rate(s1.p, s1.v, b0.p, b0.v, float(rng.normal(0.0, 0.05)), 0.05)
                if mrr is not None:
                    s1, _ = update(s1, mrr, cfg, P_j=b0.P, fusion="ci")
        err1.append(float(np.linalg.norm(s1.p - p1)))
        rel_err.append(float(np.linalg.norm((s1.p - s0.p) - (p1 - p0))))
    e = np.asarray(err1, dtype=np.float64)
    r = np.asarray(rel_err, dtype=np.float64)
    return {
        "p50": float(np.median(e)),
        "p95": float(np.quantile(e, 0.95)),
        "rel_p50": float(np.median(r)),
        "mean": float(np.mean(e)),
        "n": n_runs,
    }


def run_d11_facing_mc(
    cfg: dict,
    use_mutual_yaw: bool,
    n_runs: int = 200,
    n_steps: int = 25,
    seed: int = 7,
) -> dict:
    """Facing pair: drone 0 yaw=0, drone 1 yaw=π. Position known; yaw is not."""
    rng = np.random.default_rng(seed)
    p0 = SwarmState.from_launch(cfg, 0).p.copy()
    p1 = SwarmState.from_launch(cfg, 1).p.copy()
    psi0_t, psi1_t = 0.0, math.pi
    yaw_err = []
    for _ in range(n_runs):
        s0 = SwarmState.from_launch(cfg, 0)
        s1 = SwarmState.from_launch(cfg, 1)
        s0.x[IDX_P] = p0
        s1.x[IDX_P] = p1
        s0.P[0:3, 0:3] = (0.02**2) * np.eye(3)
        s1.P[0:3, 0:3] = (0.02**2) * np.eye(3)
        s0.P[IDX_PSI, IDX_PSI] = math.radians(25.0) ** 2
        s1.P[IDX_PSI, IDX_PSI] = math.radians(25.0) ** 2
        s0.x[IDX_PSI] = wrap_psi(psi0_t + float(rng.normal(0.0, math.radians(20.0))))
        s1.x[IDX_PSI] = wrap_psi(psi1_t + float(rng.normal(0.0, math.radians(20.0))))
        for _k in range(n_steps):
            z01, d01, az01, el01 = _synth_bearing(p0, psi0_t, 0.0, 0.0, p1, rng)
            z10, d10, az10, el10 = _synth_bearing(p1, psi1_t, 0.0, 0.0, p0, rng)
            if not use_mutual_yaw:
                continue
            Rij = cartesian_cov_from_spherical(d01, az01, el01, SIGMA_D, SIGMA_AZ, SIGMA_EL)
            Rji = cartesian_cov_from_spherical(d10, az10, el10, SIGMA_D, SIGMA_AZ, SIGMA_EL)
            m0 = mutual_yaw(
                s0.psi, s0.pitch, s0.roll, z01, s1.psi, s1.pitch, s1.roll, z10, Rij, Rji
            )
            m1 = mutual_yaw(
                s1.psi, s1.pitch, s1.roll, z10, s0.psi, s0.pitch, s0.roll, z01, Rji, Rij
            )
            if m0 is not None:
                s0, _ = update(s0, m0, cfg, P_j=s1.P, fusion="ci")
            if m1 is not None:
                s1, _ = update(s1, m1, cfg, P_j=s0.P, fusion="ci")
        e0 = abs(wrap_psi(s0.psi - psi0_t))
        e1 = abs(wrap_psi(s1.psi - psi1_t))
        yaw_err.append(0.5 * (e0 + e1))
    e = np.asarray(yaw_err, dtype=np.float64)
    return {
        "yaw_p50_rad": float(np.median(e)),
        "yaw_p50_deg": float(np.degrees(np.median(e))),
        "yaw_p95_deg": float(np.degrees(np.quantile(e, 0.95))),
        "n": n_runs,
    }


def run_selftest() -> int:
    ok = True
    n_pass = 0
    n_fail = 0

    def check(name: str, cond: bool, detail: str = ""):
        nonlocal ok, n_pass, n_fail
        if cond:
            n_pass += 1
            print(f"[selftest] PASS {name}")
        else:
            ok = False
            n_fail += 1
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))

    cfg = load_config(resolve_config_path("configs/estimation/swarm_loc.yaml"))

    p_i = np.array([1.5, 0.0, 0.5])
    p_j = np.array([0.0, 0.0, 0.5])
    z_ji = rpy_to_R(0.0, 0.0, 0.0).T @ (p_i - p_j)
    m = reciprocal_relpos(p_i, p_j, 0.0, 0.0, 0.0, z_ji, SIGMA_D, SIGMA_AZ, SIGMA_EL)
    check("1 reciprocal residual 0 at truth", float(np.linalg.norm(m.residual)) < 1e-12)
    check("1b H_i position is +R_j^T", np.allclose(m.H_i[:, IDX_P], np.eye(3), atol=1e-12))

    print("[selftest] running D8 corridor Monte Carlo …")
    rec = run_d8_corridor_mc(cfg, "reciprocal")
    rng_only = run_d8_corridor_mc(cfg, "range")
    rr = run_d8_corridor_mc(cfg, "range_rate")
    print(
        f"[selftest] D8 drone1 p50 reciprocal={rec['p50']:.3f} "
        f"range={rng_only['p50']:.3f} range_rate={rr['p50']:.3f}  "
        f"rel_p50 rec={rec['rel_p50']:.3f} range={rng_only['rel_p50']:.3f} "
        f"range_rate={rr['rel_p50']:.3f}"
    )
    check(
        "2 D8 drone-1 error < range-only",
        rec["p50"] < 0.85 * rng_only["p50"],
        f"rec={rec['p50']:.3f} range={rng_only['p50']:.3f}",
    )
    check(
        "3 D8 drone-1 error < d·d'",
        rec["p50"] < 0.85 * rr["p50"],
        f"rec={rec['p50']:.3f} range_rate={rr['p50']:.3f}",
    )

    print("[selftest] running D11 facing-pair Monte Carlo …")
    on = run_d11_facing_mc(cfg, True)
    off = run_d11_facing_mc(cfg, False)
    print(
        f"[selftest] D11 yaw_p50_deg on={on['yaw_p50_deg']:.2f} off={off['yaw_p50_deg']:.2f}"
    )
    check(
        "4 D11 facing mutual-yaw beats init",
        on["yaw_p50_deg"] < 0.70 * off["yaw_p50_deg"],
        f"on={on['yaw_p50_deg']:.2f} off={off['yaw_p50_deg']:.2f}",
    )
    check("5 D11 off stays large", off["yaw_p50_deg"] > 8.0, f"off={off['yaw_p50_deg']:.2f}")

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
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
