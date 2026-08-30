#!/usr/bin/env python3
"""central_reference.py — batch least-squares reference trajectory (P2-7).

Reads per-drone measurement logs (not a rosbag). Solves all drone poses at
all keyframes in one problem. That is the centralized upper bound.

Usage:
    python3 eval_scripts/central_reference.py --selftest
    python3 eval_scripts/central_reference.py --logs <dir> --out ref.npz
"""
from __future__ import annotations

import argparse
import copy
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in ("perception/swarm_loc", "perception/uwb_sim", "eval_scripts"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

from meas_log import (  # noqa: E402
    EST_DTYPE,
    KIND_ENTRANCE_RANGE,
    KIND_ENTRANCE_RELPOS,
    KIND_RANGE,
    KIND_RELPOS,
    MeasurementLogger,
    RIO_DTYPE,
    UWB_DTYPE,
    kind_from_edge,
    load_run,
)
from measurements import cartesian_cov_from_spherical  # noqa: E402
from rio_stub import load_config, resolve_config_path  # noqa: E402
from state import IDX_P, IDX_PSI, N_STATE, SwarmState, launch_position, rpy_to_R, wrap_psi  # noqa: E402
from uwb_edges import FLAG_BEARING_VALID, FLAG_PEER_IS_SURVEYED, FLAG_RANGE_VALID  # noqa: E402
from uwb_model import bearing_xyz  # noqa: E402

ENTRANCE_ID = 1000
POSE_DIM = 4  # px, py, pz, psi


def _prior_p(cfg: dict, drone_id: int) -> np.ndarray:
    extra = cfg.get("_prior_p")
    if extra is not None:
        return np.asarray(extra[int(drone_id)], dtype=np.float64)
    return launch_position(cfg, drone_id)


def _whiten_3(R: np.ndarray, r: np.ndarray) -> np.ndarray:
    S = 0.5 * (R + R.T) + 1e-12 * np.eye(3)
    try:
        L = np.linalg.cholesky(S)
        return np.linalg.solve(L, r)
    except np.linalg.LinAlgError:
        w, V = np.linalg.eigh(S)
        W = V @ np.diag(1.0 / np.sqrt(np.clip(w, 1e-12, None))) @ V.T
        return W @ r


def _z_body(row) -> np.ndarray:
    z = np.array([float(row["z0"]), float(row["z1"]), float(row["z2"])], dtype=np.float64)
    if np.all(np.isfinite(z)) and float(np.linalg.norm(z)) > 1e-9:
        return z
    return np.asarray(
        bearing_xyz(float(row["range_m"]), float(row["azimuth_rad"]), float(row["elevation_rad"])),
        dtype=np.float64,
    )


def _R_cart(row, extra_m: float = 0.0) -> np.ndarray:
    Rc = cartesian_cov_from_spherical(
        float(row["range_m"]),
        float(row["azimuth_rad"]),
        float(row["elevation_rad"]),
        float(row["sigma_range_m"]),
        float(row["sigma_az_rad"]),
        float(row["sigma_el_rad"]),
    )
    if extra_m > 0.0:
        Rc = Rc + (extra_m**2) * np.eye(3)
    return Rc


class PoseGraph:
    """Keyframe poses: x[i, k] = (p, psi)."""

    def __init__(self, drone_ids: List[int], stamps: Dict[int, np.ndarray], cfg: dict):
        self.ids = [int(i) for i in sorted(drone_ids)]
        self.id_to_slot = {i: k for k, i in enumerate(self.ids)}
        self.stamps = {int(i): np.asarray(stamps[i], dtype=np.float64) for i in self.ids}
        self.n_kf = {i: int(self.stamps[i].size) for i in self.ids}
        self.cfg = cfg
        self.offsets = {}
        off = 0
        for i in self.ids:
            self.offsets[i] = off
            off += self.n_kf[i] * POSE_DIM
        self.n_x = off

    def nearest_k(self, drone_id: int, stamp: float) -> int:
        t = self.stamps[int(drone_id)]
        return int(np.argmin(np.abs(t - float(stamp))))

    def pack_index(self, drone_id: int, k: int) -> int:
        return self.offsets[int(drone_id)] + int(k) * POSE_DIM

    def get_pose(self, x: np.ndarray, drone_id: int, k: int) -> Tuple[np.ndarray, float]:
        i0 = self.pack_index(drone_id, k)
        return x[i0 : i0 + 3].copy(), float(x[i0 + 3])

    def x0_from_launch(self) -> np.ndarray:
        x = np.zeros(self.n_x, dtype=np.float64)
        yaw0 = math.radians(float(self.cfg["launch"]["init_yaw_deg"]))
        for i in self.ids:
            p = _prior_p(self.cfg, i)
            for k in range(self.n_kf[i]):
                i0 = self.pack_index(i, k)
                x[i0 : i0 + 3] = p
                x[i0 + 3] = yaw0
        return x

    def x0_from_estimates(self, logs: Dict[int, dict]) -> np.ndarray:
        x = self.x0_from_launch()
        for i in self.ids:
            est = logs[i]["estimate"]
            if est.size == 0:
                continue
            for k, stamp in enumerate(self.stamps[i]):
                j = int(np.argmin(np.abs(est["stamp"] - stamp)))
                i0 = self.pack_index(i, k)
                x[i0] = float(est[j]["p_x"])
                x[i0 + 1] = float(est[j]["p_y"])
                x[i0 + 2] = float(est[j]["p_z"])
                x[i0 + 3] = float(est[j]["psi"])
        return x


def build_keyframes(logs: Dict[int, dict], dt_s: float = 0.5) -> Dict[int, np.ndarray]:
    """One pose per drone if there is no RIO; otherwise decimate RIO stamps."""
    stamps = {}
    for i, rec in logs.items():
        rio = rec["rio"]
        if rio.size == 0:
            stamps[int(i)] = np.array([0.0], dtype=np.float64)
            continue
        t0 = float(rio["stamp"][0])
        t1 = float(rio["stamp"][-1])
        if t1 - t0 < float(dt_s) * 0.5:
            stamps[int(i)] = np.array([t1], dtype=np.float64)
        else:
            stamps[int(i)] = np.arange(t0, t1 + 0.25 * dt_s, float(dt_s), dtype=np.float64)
    return stamps


def _compound_rio(rio, t_a: float, t_b: float) -> Optional[Tuple[np.ndarray, float]]:
    """World-frame displacement and yaw increment on (t_a, t_b] from body RIO."""
    dp_w = np.zeros(3, dtype=np.float64)
    psi = 0.0
    dpsi = 0.0
    n = 0
    for row in rio:
        ts = float(row["stamp"])
        if ts <= t_a or ts > t_b:
            continue
        if int(row["valid"]) == 0:
            continue
        inc = np.array([float(row["dp_x"]), float(row["dp_y"]), float(row["dp_z"])])
        R = rpy_to_R(psi, float(row["pitch"]), float(row["roll"]))
        dp_w = dp_w + R @ inc
        dpsi += float(row["dpsi"])
        psi = wrap_psi(psi + float(row["dpsi"]))
        n += 1
    if n == 0:
        return None
    return dp_w, dpsi


def residuals(x: np.ndarray, graph: PoseGraph, logs: Dict[int, dict]) -> np.ndarray:
    cfg = graph.cfg
    p_ent = np.array(cfg["entrance"]["position_xyz_m"], dtype=np.float64)
    sig_ent = float(cfg["entrance"]["sigma_m"])
    # Loose launch prior — entrance + UWB are the gauge. A 5 cm prior would
    # pin every drone to the surveyed spawn and hide the decentralization gap.
    sig_p = 1.0
    sig_psi = max(math.radians(float(cfg["estimator"]["state_init_sigma_psi_deg"])), math.radians(10.0))
    rio_sig = max(float(cfg["rio"]["sigma_p"]), 0.01)
    rio_sig_psi = max(math.radians(float(cfg["rio"]["sigma_psi_deg"])), 1e-4)
    rs: List[np.ndarray] = []

    for i in graph.ids:
        p0 = _prior_p(cfg, i)
        p, psi = graph.get_pose(x, i, 0)
        rs.append((p - p0) / sig_p)
        rs.append(np.array([wrap_psi(psi - math.radians(float(cfg["launch"]["init_yaw_deg"]))) / sig_psi]))

    for i, rec in logs.items():
        rio = rec["rio"]
        nkf = graph.n_kf[i]
        for k in range(nkf - 1):
            t_a = float(graph.stamps[i][k])
            t_b = float(graph.stamps[i][k + 1])
            cmpd = _compound_rio(rio, t_a, t_b)
            if cmpd is None:
                continue
            dp_w, dpsi = cmpd
            pa, psia = graph.get_pose(x, i, k)
            pb, psib = graph.get_pose(x, i, k + 1)
            # Compound was integrated from a local yaw of 0; rotate into world at ψ_a.
            R0 = rpy_to_R(psia, 0.0, 0.0)
            pred = pa + R0 @ dp_w
            rs.append((pb - pred) / rio_sig)
            rs.append(np.array([wrap_psi(psib - psia - dpsi) / rio_sig_psi]))

        for row in rec["uwb"]:
            kind = int(row["kind"])
            obs = int(row["observer_id"])
            peer = int(row["peer_id"])
            if obs not in graph.id_to_slot:
                continue
            ko = graph.nearest_k(obs, float(row["stamp"]))
            po, psio = graph.get_pose(x, obs, ko)
            roll, pitch = float(row["roll_obs"]), float(row["pitch_obs"])
            # Prefer the optimized yaw; logged psi_obs is a fallback if we
            # ever logged a measurement without a pose slot.
            R = rpy_to_R(psio, pitch, roll)
            if kind in (KIND_ENTRANCE_RELPOS, KIND_ENTRANCE_RANGE) or peer == ENTRANCE_ID:
                if kind == KIND_ENTRANCE_RANGE or kind == KIND_RANGE:
                    d = float(np.linalg.norm(p_ent - po))
                    sig = math.hypot(float(row["sigma_range_m"]), sig_ent)
                    rs.append(np.array([(d - float(row["range_m"])) / max(sig, 1e-6)]))
                else:
                    h = R.T @ (p_ent - po)
                    z = _z_body(row)
                    rs.append(_whiten_3(_R_cart(row, sig_ent), z - h))
                continue
            if peer not in graph.id_to_slot:
                continue
            kp = graph.nearest_k(peer, float(row["stamp"]))
            pp, _ = graph.get_pose(x, peer, kp)
            if kind == KIND_RELPOS:
                h = R.T @ (pp - po)
                rs.append(_whiten_3(_R_cart(row), _z_body(row) - h))
            else:
                d = float(np.linalg.norm(pp - po))
                if d < 1e-9:
                    continue
                sig = max(float(row["sigma_range_m"]), 1e-4)
                rs.append(np.array([(d - float(row["range_m"])) / sig]))
    if not rs:
        return np.zeros(1)
    return np.concatenate([np.asarray(r, dtype=np.float64).reshape(-1) for r in rs])


def solve(logs: Dict[int, dict], cfg: dict, x0: Optional[np.ndarray] = None) -> dict:
    """Batch LS. Returns poses, cost, and a per-drone last-keyframe snapshot."""
    from scipy.optimize import least_squares

    stamps = build_keyframes(logs)
    graph = PoseGraph(list(logs.keys()), stamps, cfg)
    if x0 is None:
        x0 = graph.x0_from_launch()
    fun = lambda x: residuals(x, graph, logs)
    out = least_squares(fun, x0, method="trf", ftol=1e-9, xtol=1e-9, gtol=1e-8, max_nfev=400)
    x = out.x.copy()
    poses = {}
    last = {}
    for i in graph.ids:
        n = graph.n_kf[i]
        p = np.zeros((n, 3), dtype=np.float64)
        psi = np.zeros(n, dtype=np.float64)
        for k in range(n):
            pk, yk = graph.get_pose(x, i, k)
            p[k] = pk
            psi[k] = wrap_psi(yk)
        poses[i] = {"stamp": graph.stamps[i], "p": p, "psi": psi}
        last[i] = {"p": p[-1], "psi": float(psi[-1])}
    return {
        "poses": poses,
        "last": last,
        "cost": float(out.cost),
        "success": bool(out.success) or float(out.cost) < 1e3,
        "nfev": int(out.nfev),
        "graph": graph,
        "x": x,
    }


def write_reference(path: str, result: dict) -> None:
    payload = {}
    for i, rec in result["poses"].items():
        payload[f"cf_{i}_stamp"] = rec["stamp"]
        payload[f"cf_{i}_p"] = rec["p"]
        payload[f"cf_{i}_psi"] = rec["psi"]
    payload["drone_ids"] = np.array(sorted(result["poses"].keys()), dtype=np.int32)
    payload["cost"] = np.float64(result["cost"])
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, **payload)


def _synth_edge(p_obs, psi, p_peer, rng, bearing: bool, surveyed: bool, sigma_d, sigma_az, sigma_el):
    R = rpy_to_R(float(psi), 0.0, 0.0)
    z_true = R.T @ (np.asarray(p_peer) - np.asarray(p_obs))
    d = float(np.linalg.norm(z_true))
    az = math.atan2(float(z_true[1]), float(z_true[0]))
    el = math.asin(float(np.clip(z_true[2] / max(d, 1e-12), -1.0, 1.0)))
    if rng is None:
        d_m, az_m, el_m = d, az, el
    else:
        d_m = max(d + float(rng.normal(0.0, sigma_d)), 0.05)
        az_m = az + float(rng.normal(0.0, sigma_az))
        el_m = float(np.clip(el + rng.normal(0.0, sigma_el), -1.56, 1.56))
    x, y, z = bearing_xyz(d_m, az_m, el_m)
    flags = FLAG_RANGE_VALID
    if bearing:
        flags |= FLAG_BEARING_VALID
    if surveyed:
        flags |= FLAG_PEER_IS_SURVEYED
    return {
        "flags": flags,
        "x": float(x),
        "y": float(y),
        "z": float(z) if bearing else float("nan"),
        "range_m": float(d_m),
        "azimuth_rad": float(az_m),
        "elevation_rad": float(el_m),
        "sigma_range_m": float(sigma_d),
        "sigma_az_rad": float(sigma_az),
        "sigma_el_rad": float(sigma_el),
        "peer_id": ENTRANCE_ID if surveyed else -1,
    }


def _log_edge(log: MeasurementLogger, st, edge, peer_id: int, kind: int) -> None:
    z = None
    if kind in (KIND_RELPOS, KIND_ENTRANCE_RELPOS):
        z = [edge["x"], edge["y"], edge["z"]]
    log.add_uwb(
        float(st.stamp),
        kind,
        st.drone_id,
        peer_id,
        edge["range_m"],
        edge["azimuth_rad"],
        edge["elevation_rad"],
        edge["sigma_range_m"],
        edge["sigma_az_rad"],
        edge["sigma_el_rad"],
        st.psi,
        st.roll,
        st.pitch,
        z,
    )


def run_synthetic(
    cfg: dict,
    mode: str,
    n_steps: int = 16,
    seed: int = 11,
    noise: bool = True,
) -> dict:
    """Offline stand-in for a logged live run.

    easy: 2 drones, full bearing + entrance on drone 0 (distributed should be good).
    hard: 3 drones, range-only mesh, entrance bearing only on drone 0, large
    init error on drone 2 (CI sequential vs joint LS).
    """
    from ekf import update
    from measurements import from_edge

    rng = np.random.default_rng(seed) if noise else None
    sigma_d = 0.0873
    sigma_az = math.radians(4.795)
    sigma_el = math.radians(4.795)
    cfg = copy.deepcopy(cfg)
    p_ent = np.array(cfg["entrance"]["position_xyz_m"], dtype=np.float64)
    n = 2 if mode == "easy" else 3
    truth = {i: launch_position(cfg, i).copy() for i in range(n)}
    if mode == "hard":
        # Non-collinear surveyed spawn (still a deployment prior, not truth leak).
        truth[2] = np.array([0.75, 1.20, 0.50], dtype=np.float64)
    cfg["_prior_p"] = {i: truth[i].copy() for i in range(n)}

    states = {i: SwarmState.from_launch(cfg, i) for i in range(n)}
    for i, st in states.items():
        st.x[IDX_P] = truth[i]
        if rng is not None:
            st.x = st.x + rng.multivariate_normal(np.zeros(N_STATE), st.P)
            st.x[IDX_PSI] = wrap_psi(float(st.x[IDX_PSI]))
        st.stamp = 0.02
    if mode == "hard" and rng is not None:
        states[2].x[IDX_P] = states[2].x[IDX_P] + np.array([0.15, 0.55, 0.10])
        states[2].P[0:3, 0:3] = states[2].P[0:3, 0:3] + (0.40**2) * np.eye(3)

    logs = {i: MeasurementLogger(i) for i in range(n)}
    use_bearing = mode == "easy"

    for step in range(n_steps):
        stamp = (step + 1) * 0.1
        for i in range(n):
            states[i].stamp = stamp
        snap = {i: states[i].copy() for i in range(n)}

        # drone 0 ↔ entrance
        e0 = _synth_edge(truth[0], 0.0, p_ent, rng, True, True, sigma_d, sigma_az, sigma_el)
        e0["peer_id"] = ENTRANCE_ID
        m = from_edge(states[0], None, e0, cfg)
        if m is not None:
            states[0], _ = update(states[0], m, cfg)
        _log_edge(logs[0], states[0], e0, ENTRANCE_ID, kind_from_edge(e0, True))

        pairs = [(0, 1)] if n == 2 else [(0, 1), (0, 2), (1, 2)]
        for a, b in pairs:
            ea = _synth_edge(truth[a], 0.0, truth[b], rng, use_bearing, False, sigma_d, sigma_az, sigma_el)
            ea["peer_id"] = b
            eb = _synth_edge(truth[b], 0.0, truth[a], rng, use_bearing, False, sigma_d, sigma_az, sigma_el)
            eb["peer_id"] = a
            ma = from_edge(states[a], snap[b].p, ea, cfg)
            mb = from_edge(states[b], snap[a].p, eb, cfg)
            if ma is not None:
                states[a], _ = update(states[a], ma, cfg, P_j=snap[b].P, fusion="ci")
            if mb is not None:
                states[b], _ = update(states[b], mb, cfg, P_j=snap[a].P, fusion="ci")
            _log_edge(logs[a], states[a], ea, b, kind_from_edge(ea, use_bearing))
            _log_edge(logs[b], states[b], eb, a, kind_from_edge(eb, use_bearing))

        for i in range(n):
            st = states[i]
            logs[i].add_est(st.stamp, st.p, st.v, st.psi, st.status)

    dist = {i: {"p": states[i].p.copy(), "psi": states[i].psi} for i in range(n)}
    loaded = {}
    # in-memory view matching load_run
    for i, lg in logs.items():
        loaded[i] = {
            "drone_id": i,
            "rio": np.zeros(0, dtype=RIO_DTYPE),
            "uwb": np.array(lg.uwb, dtype=UWB_DTYPE) if lg.uwb else np.zeros(0, dtype=UWB_DTYPE),
            "estimate": np.array(lg.est, dtype=EST_DTYPE) if lg.est else np.zeros(0, dtype=EST_DTYPE),
        }
    return {"logs": loaded, "loggers": logs, "truth": truth, "dist": dist, "n": n, "cfg": cfg}


def _ate(est: dict, truth: dict) -> float:
    errs = [float(np.linalg.norm(est[i]["p"] - truth[i])) for i in truth]
    return float(np.mean(errs))


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

    print("[selftest] noiseless 2-drone recover …")
    syn0 = run_synthetic(cfg, "easy", n_steps=8, seed=0, noise=False)
    ref0 = solve(syn0["logs"], syn0["cfg"])
    ate0 = _ate(ref0["last"], syn0["truth"])
    check("1 noiseless ATE < 1e-3", ate0 < 1e-3, f"ate={ate0:.3e}")

    print("[selftest] easy run (bearing + entrance) …")
    easy = run_synthetic(cfg, "easy", n_steps=16, seed=11, noise=True)
    ref_e = solve(easy["logs"], easy["cfg"])
    ate_e_c = _ate(ref_e["last"], easy["truth"])
    ate_e_d = _ate(easy["dist"], easy["truth"])
    agree = float(
        np.mean([np.linalg.norm(ref_e["last"][i]["p"] - easy["dist"][i]["p"]) for i in easy["truth"]])
    )
    print(f"[selftest] easy ATE cent={ate_e_c:.3f} dist={ate_e_d:.3f} |c-d|={agree:.3f}")
    check("2 easy both good", ate_e_c < 0.15 and ate_e_d < 0.15, f"c={ate_e_c:.3f} d={ate_e_d:.3f}")
    check("3 easy agree closely", agree < 0.12, f"|c-d|={agree:.3f}")

    print("[selftest] hard run (range-only mesh, hop-2 init error) …")
    hard = run_synthetic(cfg, "hard", n_steps=16, seed=13, noise=True)
    ref_h = solve(hard["logs"], hard["cfg"])
    ate_h_c = _ate(ref_h["last"], hard["truth"])
    ate_h_d = _ate(hard["dist"], hard["truth"])
    print(f"[selftest] hard ATE cent={ate_h_c:.3f} dist={ate_h_d:.3f}")
    check(
        "4 hard centralized better",
        ate_h_c < 0.90 * ate_h_d,
        f"c={ate_h_c:.3f} d={ate_h_d:.3f}",
    )
    check("5 hard centralized still useful", ate_h_c < 0.25, f"c={ate_h_c:.3f}")

    import tempfile

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        for i, lg in easy["loggers"].items():
            lg.save(td / f"cf_{i}.npz")
        loaded = load_run(td)
        check("6 reload two drones", set(loaded) == {0, 1})
        ref_r = solve(loaded, easy["cfg"])
        drel = float(np.linalg.norm(ref_r["last"][0]["p"] - ref_e["last"][0]["p"]))
        check("6b solve from npz matches", drel < 1e-6, f"d={drel:.3e}")
        write_reference(str(td / "ref.npz"), ref_r)
        check("6c wrote ref.npz", (td / "ref.npz").is_file())

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--logs", default="", help="directory of cf_*.npz or a single npz")
    parser.add_argument("--out", default="central_ref.npz")
    parser.add_argument("--config", default="configs/estimation/swarm_loc.yaml")
    args = parser.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    if not args.logs:
        parser.print_help()
        sys.exit(2)
    cfg = load_config(resolve_config_path(args.config))
    logs = load_run(args.logs)
    if not logs:
        print(f"[central_reference] no logs in {args.logs}", file=sys.stderr)
        sys.exit(2)
    result = solve(logs, cfg)
    write_reference(args.out, result)
    print(f"[central_reference] drones={sorted(logs)} cost={result['cost']:.4f} -> {args.out}")
    for i, rec in result["last"].items():
        print(f"  cf_{i} p={rec['p']} psi={rec['psi']:.3f}")
    sys.exit(0)


if __name__ == "__main__":
    main()
