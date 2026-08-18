#!/usr/bin/env python3
"""uwb_gate.py — Phase 1 M4 exit gate for inter-drone UWB PDoA mesh.

Validates simulated UWB range+bearing on /uwb/edges_all against /uwb/edges_truth
while two Crazyflie SITL drones hold, yaw, and climb through scripted legs.

NLOS is not tested live — deterministic box occlusion is covered by
uwb_model.py --selftest (plan §8.1 checks 8–10).

Prereq: running sim from phase0_gate.sh, e.g.
    ./eval_scripts/phase0_gate.sh -w phase1_pid_tune -n 2 --spacing 2.0 \\
        --no-radar --headless --no-flow

Usage (setup_env.sh sourced):
    python3 -u eval_scripts/uwb_gate.py --config configs/sensors/uwb_pdoa.yaml
"""
from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time
import warnings
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import mlflow
import numpy as np
import yaml

warnings.filterwarnings("ignore", category=DeprecationWarning, module="cflib.*")
warnings.filterwarnings("ignore", category=UserWarning, module="cflib.*")

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_UWB_SIM = os.path.join(_REPO_ROOT, "perception", "uwb_sim")
if _UWB_SIM not in sys.path:
    sys.path.insert(0, _UWB_SIM)

from uwb_edges import (  # noqa: E402
    FLAG_BEARING_VALID,
    FLAG_IN_AOA_CONE,
    FLAG_PEER_IS_SURVEYED,
    FLAG_RANGE_VALID,
    unpack_edges,
)

STAMP_PAIR_MAX_S = 0.050
RANGE_ERR_MEAN_MAX_M = 0.25
RANGE_ERR_STD_MULT = 2.0
AZ_RMS_MULT = 3.0
BEARING_VALID_FRAC_MIN = 0.90
NAN_FRAC_MIN = 1.00
REL_POS_ERR_MEAN_MAX_M = 0.45
EDGE_RATE_TOL = 0.20
ENTRANCE_PEER_ID = 1000
_RESTART_MSG = (
    "[uwb_gate] Restart sim before retrying:\n"
    "  pkill -9 -f 'gz sim'; pkill -9 -x cf2; pkill -9 -f uwb_node\n"
    "  ./eval_scripts/phase0_gate.sh -w phase1_pid_tune -n 2 --spacing 2.0 --no-radar\n"
    "  python3 -u eval_scripts/uwb_gate.py   # first cflib client — no connect tests before this"
)


class _Timeout(Exception):
    pass


def _alarm(sig, frame):
    raise _Timeout()


def _cflib_cache_dir() -> str:
    cache = os.path.join(_REPO_ROOT, "cache")
    os.makedirs(cache, exist_ok=True)
    return cache


def _count_cf2() -> int:
    try:
        out = subprocess.run(
            ["pgrep", "-x", "cf2"], capture_output=True, text=True, check=False
        )
    except FileNotFoundError:
        return -1
    return len([ln for ln in out.stdout.splitlines() if ln.strip()])


def _port_bound(port: int) -> bool:
    try:
        out = subprocess.run(
            ["ss", "-ulnp"], capture_output=True, text=True, check=False
        )
    except FileNotFoundError:
        return False
    needle = f":{port}"
    return needle in out.stdout


def _wait_for_sitl(args) -> None:
    ports = [19850 + args.cf_id_0, 19850 + args.cf_id_1]
    deadline = time.time() + args.connect_wait
    print(
        f"[uwb_gate] waiting for SITL on UDP {ports[0]} and {ports[1]} "
        f"(up to {args.connect_wait:.0f}s) …"
    )
    while time.time() < deadline:
        if all(_port_bound(p) for p in ports):
            n_cf2 = _count_cf2()
            if n_cf2 >= 0 and n_cf2 < 2:
                print(
                    f"[uwb_gate] WARN: only {n_cf2}/2 cf2 process(es) — handshake may fail",
                    file=sys.stderr,
                )
            print("[uwb_gate] SITL UDP ports up")
            return
        time.sleep(0.5)
    raise SystemExit(
        f"[uwb_gate] TIMEOUT: cflib ports not ready after {args.connect_wait:.0f}s.\n"
        f"{_RESTART_MSG}"
    )


def _open_sync_crazyflie(uri: str, cache: str, label: str, timeout_s: float):
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    print(f"[uwb_gate] connecting {label} at {uri} (timeout {timeout_s:.0f}s) …")
    holder: dict = {"scf": None, "err": None}

    def _worker():
        try:
            scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache))
            scf.__enter__()
            holder["scf"] = scf
        except Exception as exc:
            holder["err"] = exc

    thread = threading.Thread(target=_worker, daemon=True)
    thread.start()
    thread.join(timeout_s)
    if thread.is_alive():
        raise SystemExit(
            f"[uwb_gate] TIMEOUT ({timeout_s:.0f}s) connecting {label} at {uri}.\n"
            f"{_RESTART_MSG}"
        )
    if holder["err"] is not None:
        raise SystemExit(
            f"[uwb_gate] FAIL connecting {label} at {uri}: {holder['err']}\n"
            f"{_RESTART_MSG}"
        )
    return holder["scf"]


@dataclass
class EdgeSample:
    recv_t: float
    stamp: float
    observer_id: int
    peer_id: int
    range_m: float
    azimuth_rad: float
    elevation_rad: float
    sigma_range_m: float
    x: float
    y: float
    z: float
    flags: int


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def stamp_to_float(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class UwbRecorder:
    def __init__(self):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2

        class _Node(Node):
            def __init__(self):
                super().__init__("uwb_gate_recorder")
                self.measured: List[EdgeSample] = []
                self.truth: List[EdgeSample] = []
                qos = qos_profile_sensor_data
                self.create_subscription(
                    PointCloud2, "/uwb/edges_all", self._on_meas, qos
                )
                self.create_subscription(
                    PointCloud2, "/uwb/edges_truth", self._on_truth, qos
                )

            def _parse(self, msg, buf: list):
                recv = time.time()
                st = stamp_to_float(msg.header.stamp)
                rows = unpack_edges(msg)
                for r in rows:
                    buf.append(
                        EdgeSample(
                            recv_t=recv,
                            stamp=st,
                            observer_id=int(r["observer_id"]),
                            peer_id=int(r["peer_id"]),
                            range_m=float(r["range_m"]),
                            azimuth_rad=float(r["azimuth_rad"]),
                            elevation_rad=float(r["elevation_rad"]),
                            sigma_range_m=float(r["sigma_range_m"]),
                            x=float(r["x"]),
                            y=float(r["y"]),
                            z=float(r["z"]),
                            flags=int(r["flags"]),
                        )
                    )

            def _on_meas(self, msg):
                self._parse(msg, self.measured)

            def _on_truth(self, msg):
                self._parse(msg, self.truth)

        rclpy.init()
        self.node = _Node()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        import rclpy

        try:
            rclpy.shutdown()
        except Exception:
            pass

    def nearest_truth(
        self, s: EdgeSample, window_s: float = STAMP_PAIR_MAX_S
    ) -> Optional[EdgeSample]:
        best = None
        best_dt = window_s
        for t in self.node.truth:
            if t.observer_id != s.observer_id or t.peer_id != s.peer_id:
                continue
            dt = abs(t.stamp - s.stamp)
            if dt < best_dt:
                best_dt = dt
                best = t
        return best

    def samples_in(self, buf: str, t0: float, t1: float) -> List[EdgeSample]:
        data = self.node.measured if buf == "meas" else self.node.truth
        return [s for s in data if t0 <= s.recv_t <= t1]


def run_dual_flight(args, windows: dict) -> bool:
    from pid_gains import load_gains, apply_gains, reset_estimator, reset_pose
    import cflib.crtp
    from cflib.positioning.motion_commander import MotionCommander

    _wait_for_sitl(args)

    gains = load_gains(args.gains)
    cache = _cflib_cache_dir()
    cflib.crtp.init_drivers()
    signal.signal(signal.SIGALRM, _alarm)
    diverged = False

    uri0 = f"udp://127.0.0.1:{19850 + args.cf_id_0}"
    uri1 = f"udp://127.0.0.1:{19850 + args.cf_id_1}"

    scf0 = scf1 = None
    try:
        scf0 = _open_sync_crazyflie(uri0, cache, "drone 0", args.connect_timeout)
        scf1 = _open_sync_crazyflie(uri1, cache, "drone 1", args.connect_timeout)
        print("[uwb_gate] both drones connected — starting flight legs …")

        for idx, scf in ((0, scf0), (1, scf1)):
            cf = scf.cf
            apply_gains(cf, gains)
            try:
                reset_pose(
                    args.world,
                    f"{args.model_prefix}_{idx}",
                    xyz=(float(idx * args.spacing), 0.0, args.hover_height),
                )
            except Exception as e:
                print(f"[uwb_gate] reset_pose drone {idx} skipped: {e}", file=sys.stderr)
            reset_estimator(cf, "kalman")
        time.sleep(2.0)
        for scf in (scf0, scf1):
            try:
                scf.cf.platform.send_arming_request(True)
            except Exception:
                pass
        time.sleep(0.5)

        signal.setitimer(signal.ITIMER_REAL, 300.0)
        mc0 = MotionCommander(scf0, default_height=args.hover_height)
        mc1 = MotionCommander(scf1, default_height=args.hover_height)
        with mc0, mc1:
            print("[uwb_gate] leg 0: takeoff/settle 3 s …")
            time.sleep(3.0)

            print("[uwb_gate] leg 1: hold 6 s (window A) …")
            windows["A"] = (time.time(), None)
            time.sleep(6.0)
            windows["A"] = (windows["A"][0], time.time())

            print("[uwb_gate] leg 2: drone0 turn_left 150°, settle 2 s, hold (window B) …")
            mc0.turn_left(150)
            time.sleep(2.0)
            windows["B"] = (time.time(), None)
            time.sleep(5.0)
            windows["B"] = (windows["B"][0], time.time())
            print(f"[uwb_gate] window B captured {windows['B']}")

            print("[uwb_gate] leg 3: drone0 turn_right 150°, drone1 up 0.6 m (window C) …")
            mc0.turn_right(150)
            time.sleep(2.0)
            mc1.up(0.6)
            windows["C"] = (time.time(), None)
            time.sleep(5.0)
            windows["C"] = (windows["C"][0], time.time())

            print("[uwb_gate] leg 4: stop …")
            mc0.stop()
            mc1.stop()
    except _Timeout:
        diverged = True
        print("[uwb_gate] TIMEOUT during flight", file=sys.stderr)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        for scf in (scf1, scf0):
            if scf is not None:
                try:
                    scf.__exit__(None, None, None)
                except Exception:
                    pass

    return diverged


def check_bidirectional_identity(samples: List[EdgeSample]) -> float:
    """Fraction of matched ticks where range 0→1 == range 1→0 (bit-identical)."""
    by_stamp: Dict[float, dict] = {}
    for s in samples:
        by_stamp.setdefault(s.stamp, {})[(s.observer_id, s.peer_id)] = s.range_m
    total = 0
    identical = 0
    for rows in by_stamp.values():
        if (0, 1) in rows and (1, 0) in rows:
            total += 1
            if rows[(0, 1)] == rows[(1, 0)]:
                identical += 1
    return identical / total if total else 0.0


def dedupe_edge_samples(samples: List[EdgeSample]) -> List[EdgeSample]:
    """One row per (odom stamp, observer, peer); drops duplicate ROS deliveries."""
    seen: set = set()
    out: List[EdgeSample] = []
    for s in samples:
        key = (s.stamp, s.observer_id, s.peer_id)
        if key in seen:
            continue
        seen.add(key)
        out.append(s)
    return out


def measured_samples(samples: List[EdgeSample]) -> List[EdgeSample]:
    """Drop truth-oracle rows (sigma_range_m == 0) if present on the measured topic."""
    return [s for s in samples if s.sigma_range_m > 0.0]


def scheduled_pair_count(cfg: dict, num_drones: int = 2) -> int:
    """Pairs within max_range for drones + static_peers (gate layout)."""
    n = num_drones * (num_drones - 1) // 2
    n += num_drones * len(cfg.get("static_peers", []))
    return max(n, 1)


def rel_pos_errors(
    recorder: UwbRecorder, samples: List[EdgeSample]
) -> List[float]:
    """Compare measured body-frame xyz to truth oracle for bearing-valid rows."""
    errs = []
    for s in samples:
        if not (s.flags & FLAG_BEARING_VALID):
            continue
        if any(math.isnan(v) for v in (s.x, s.y, s.z)):
            continue
        tr = recorder.nearest_truth(s)
        if tr is None or not (tr.flags & FLAG_BEARING_VALID):
            continue
        if any(math.isnan(v) for v in (tr.x, tr.y, tr.z)):
            continue
        dm = np.array([s.x, s.y, s.z])
        dt = np.array([tr.x, tr.y, tr.z])
        errs.append(float(np.linalg.norm(dm - dt)))
    return errs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/sensors/uwb_pdoa.yaml")
    parser.add_argument("--gains", default="configs/airframe/pid_gains_loaded.yaml")
    parser.add_argument("--world", default="phase1_pid_tune")
    parser.add_argument("--model-prefix", default="crazyflie")
    parser.add_argument("--cf-id-0", type=int, default=0)
    parser.add_argument("--cf-id-1", type=int, default=1)
    parser.add_argument("--spacing", type=float, default=2.0)
    parser.add_argument("--hover-height", type=float, default=0.5)
    parser.add_argument(
        "--connect-wait",
        type=float,
        default=90.0,
        help="Seconds to wait for cflib UDP ports before connecting (default: 90)",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=25.0,
        help="Per-drone SyncCrazyflie connect timeout in seconds (default: 25)",
    )
    parser.add_argument("--no-mlflow", action="store_true")
    args = parser.parse_args()

    cfg_path = args.config
    if not os.path.isabs(cfg_path):
        cfg_path = os.path.join(_REPO_ROOT, cfg_path)
    cfg = load_config(cfg_path)

    n_pairs = scheduled_pair_count(cfg, num_drones=2)
    ranging_rate = float(cfg["ranging_rate_hz"])
    budget = float(cfg["max_exchanges_per_s"])
    effective_rate = min(ranging_rate, budget / n_pairs)
    sigma_r = float(cfg["sigma_range_los_m"])
    sigma_b_deg = float(cfg["sigma_boresight_deg"])

    recorder = UwbRecorder()

    windows: Dict[str, Tuple[float, Optional[float]]] = {}
    diverged = run_dual_flight(args, windows)
    time.sleep(0.5)
    recorder.shutdown()

    if "A" not in windows or windows["A"][1] is None:
        print("[uwb_gate] FAIL — flight leg A did not complete", file=sys.stderr)
        sys.exit(1)

    tA0, tA1 = windows.get("A", (0, 0))
    tB0, tB1 = windows.get("B", (0, 0))
    tC0, tC1 = windows.get("C", (0, 0))

    sampA = dedupe_edge_samples(
        measured_samples(recorder.samples_in("meas", tA0, tA1 or time.time()))
    )
    sampB = dedupe_edge_samples(
        measured_samples(recorder.samples_in("meas", tB0, tB1 or time.time()))
    )
    sampC = dedupe_edge_samples(
        measured_samples(recorder.samples_in("meas", tC0, tC1 or time.time()))
    )

    pair01A = [s for s in sampA if s.observer_id == 0 and s.peer_id == 1]
    pair01B = [s for s in sampB if s.observer_id == 0 and s.peer_id == 1]

    window_a_s = (tA1 or time.time()) - tA0
    edge_rate = (len(pair01A) / window_a_s) if pair01A and window_a_s > 0 else None
    rate_ok = (
        edge_rate is not None
        and abs(edge_rate - effective_rate) / effective_rate <= EDGE_RATE_TOL
    )

    range_errs = []
    for s in pair01A:
        tr = recorder.nearest_truth(s)
        if tr:
            range_errs.append(abs(s.range_m - tr.range_m))
    range_mean = sum(range_errs) / len(range_errs) if range_errs else float("inf")
    range_std = (
        float(np.std(range_errs)) if len(range_errs) > 1 else float("inf")
    )

    bidir_frac = check_bidirectional_identity(sampA)

    bearing_valid_A = (
        sum(1 for s in pair01A if s.flags & FLAG_BEARING_VALID) / len(pair01A)
        if pair01A
        else 0.0
    )

    az_errs = []
    for s in pair01A:
        if not (s.flags & FLAG_BEARING_VALID):
            continue
        tr = recorder.nearest_truth(s)
        if tr and not math.isnan(tr.azimuth_rad):
            az_errs.append((s.azimuth_rad - tr.azimuth_rad) ** 2)
    az_rms_deg = math.degrees(math.sqrt(sum(az_errs) / len(az_errs))) if az_errs else float("inf")

    pair01B_obs0 = pair01B
    bearing_valid_B = (
        sum(1 for s in pair01B_obs0 if s.flags & FLAG_BEARING_VALID) / len(pair01B_obs0)
        if pair01B_obs0
        else 0.0
    )
    in_cone_B = (
        sum(1 for s in pair01B_obs0 if s.flags & FLAG_IN_AOA_CONE) / len(pair01B_obs0)
        if pair01B_obs0
        else 0.0
    )
    range_valid_B = (
        sum(1 for s in pair01B_obs0 if s.flags & FLAG_RANGE_VALID) / len(pair01B_obs0)
        if pair01B_obs0
        else 0.0
    )

    def all_nan_fields(s: EdgeSample) -> bool:
        return all(
            math.isnan(v)
            for v in (s.azimuth_rad, s.elevation_rad, s.x, s.y, s.z)
        )

    nan_frac_B = (
        sum(1 for s in pair01B_obs0 if all_nan_fields(s)) / len(pair01B_obs0)
        if pair01B_obs0
        else 0.0
    )

    rel_errs = rel_pos_errors(recorder, pair01A)
    rel_pos_mean = sum(rel_errs) / len(rel_errs) if rel_errs else float("inf")

    el_errs = []
    for s in sampC:
        if s.observer_id == 0 and s.peer_id == 1 and (s.flags & FLAG_BEARING_VALID):
            tr = recorder.nearest_truth(s)
            if tr and not math.isnan(tr.elevation_rad):
                el_errs.append(abs(s.elevation_rad - tr.elevation_rad))
    el_err_deg = math.degrees(sum(el_errs) / len(el_errs)) if el_errs else float("inf")

    entrance_rows = [
        s
        for s in sampA
        if s.peer_id == ENTRANCE_PEER_ID and s.observer_id in (0, 1)
    ]
    entrance_seen = len(entrance_rows) > 0
    entrance_surveyed = all(s.flags & FLAG_PEER_IS_SURVEYED for s in entrance_rows) if entrance_rows else False
    entrance_range_errs = []
    for s in entrance_rows:
        tr = recorder.nearest_truth(s)
        if tr:
            entrance_range_errs.append(abs(s.range_m - tr.range_m))
    entrance_range_err_max = max(entrance_range_errs) if entrance_range_errs else float("inf")
    entrance_range_mean = (
        sum(entrance_range_errs) / len(entrance_range_errs) if entrance_range_errs else float("inf")
    )
    entrance_range_ok = entrance_range_mean <= RANGE_ERR_MEAN_MAX_M

    checks = {
        "A1_edge_rate": rate_ok,
        "A2_range_mean": range_mean <= RANGE_ERR_MEAN_MAX_M,
        "A3_range_std": range_std <= RANGE_ERR_STD_MULT * sigma_r,
        "B_bidir_identical": bidir_frac >= 1.0,
        "C1_bearing_A": bearing_valid_A >= BEARING_VALID_FRAC_MIN,
        "C2_az_rms": az_rms_deg <= AZ_RMS_MULT * sigma_b_deg,
        "C3_degrade_B": (
            bearing_valid_B <= (1.0 - BEARING_VALID_FRAC_MIN)
            and in_cone_B <= (1.0 - BEARING_VALID_FRAC_MIN)
            and range_valid_B >= BEARING_VALID_FRAC_MIN
        ),
        "C4_nan_B": nan_frac_B >= NAN_FRAC_MIN,
        "D_rel_pos": rel_pos_mean <= REL_POS_ERR_MEAN_MAX_M,
        "E1_elevation": el_err_deg <= AZ_RMS_MULT * sigma_b_deg,
        "F_entrance": entrance_seen and entrance_surveyed and entrance_range_ok,
        "G_flight": not diverged,
    }
    gate_pass = all(checks.values())

    print("\n[uwb_gate] results:")
    print(f"  edge_rate_hz={edge_rate} (target {effective_rate}, window_A={window_a_s:.1f}s, rows={len(pair01A)})")
    print(f"  range_err_mean_m={range_mean:.3f}  std={range_std:.3f}")
    print(f"  bidir_identical_frac={bidir_frac:.3f}")
    print(f"  bearing_valid_frac_A={bearing_valid_A:.3f}")
    print(f"  az_rms_deg={az_rms_deg:.2f}")
    print(f"  bearing_valid_frac_B={bearing_valid_B:.3f}  nan_frac_B={nan_frac_B:.3f}")
    print(f"  rel_pos_err_mean_m={rel_pos_mean:.3f}")
    print(f"  el_err_deg={el_err_deg:.2f}")
    print(
        f"  entrance_surveyed={entrance_surveyed}  "
        f"range_err_mean={entrance_range_mean:.3f}  max={entrance_range_err_max:.3f}"
    )
    for k, v in checks.items():
        print(f"  {'PASS' if v else 'FAIL'} {k}")
    print(f"\n[uwb_gate] {'PASS' if gate_pass else 'FAIL'}")

    if not args.no_mlflow:
        try:
            mlflow.set_tracking_uri("sqlite:///mlflow.db")
            mlflow.set_experiment(cfg.get("mlflow_experiment", "phase1_uwb_pdoa"))
            with mlflow.start_run(run_name="uwb_gate"):
                mlflow.log_param("seed", cfg.get("seed"))
                mlflow.log_param("num_drones", 2)
                mlflow.log_param("effective_rate", effective_rate)
                mlflow.log_param("sigma_range_los_m", sigma_r)
                mlflow.log_param("sigma_boresight_deg", sigma_b_deg)
                if edge_rate is not None:
                    mlflow.log_metric("edge_rate_hz", edge_rate)
                mlflow.log_metric("range_err_mean_m", range_mean)
                mlflow.log_metric("range_err_std_m", range_std)
                mlflow.log_metric("bidir_identical_frac", bidir_frac)
                mlflow.log_metric("bearing_valid_frac_A", bearing_valid_A)
                mlflow.log_metric("az_rms_deg", az_rms_deg)
                mlflow.log_metric("bearing_valid_frac_B", bearing_valid_B)
                mlflow.log_metric("nan_frac_B", nan_frac_B)
                mlflow.log_metric("rel_pos_err_mean_m", rel_pos_mean)
                mlflow.log_metric("el_err_deg", el_err_deg)
                mlflow.log_metric("entrance_seen", int(entrance_seen))
                mlflow.log_metric("gate_pass", int(gate_pass))
        except Exception as e:
            print(f"[uwb_gate] MLflow skipped: {e}", file=sys.stderr)

    sys.exit(0 if gate_pass else 1)


if __name__ == "__main__":
    main()
