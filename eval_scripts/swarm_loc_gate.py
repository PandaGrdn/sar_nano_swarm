#!/usr/bin/env python3
"""swarm_loc_gate.py — P2-5 live ROS gate for the distributed swarm-loc filter.

Prereq: sim from phase0_gate.sh, e.g.
    ./eval_scripts/phase0_gate.sh -w phase0_tunnel_gate -n 3 --spacing 1.5 \\
        --headless --no-rviz

Usage (setup_env.sh sourced):
    python3 -u eval_scripts/swarm_loc_gate.py --num-drones 3 --duration 300
    python3 -u eval_scripts/swarm_loc_gate.py --num-drones 3 --eval-dir /tmp/swarm_loc_eval
    python3 eval_scripts/swarm_loc_gate.py --selftest
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
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import yaml

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SWARM = os.path.join(_REPO_ROOT, "perception", "swarm_loc")
if _SWARM not in sys.path:
    sys.path.insert(0, _SWARM)
if os.path.join(_REPO_ROOT, "eval_scripts") not in sys.path:
    sys.path.insert(0, os.path.join(_REPO_ROOT, "eval_scripts"))

from swarm_loc_node import topics_contain_truth  # noqa: E402
from swarm_msgs import STATE_DTYPE, unpack_state  # noqa: E402

_RESTART_MSG = (
    "[swarm_loc_gate] Restart sim before retrying:\n"
    "  pkill -9 -f 'gz sim'; pkill -9 -x cf2; pkill -9 -f swarm_loc_node\n"
    "  ./eval_scripts/phase0_gate.sh -w phase0_tunnel_gate -n 3 --spacing 1.5 "
    "--headless --no-rviz\n"
    "  python3 -u eval_scripts/swarm_loc_gate.py"
)

RATE_FRAC_MIN = 0.80


class _Timeout(Exception):
    pass


def _alarm(sig, frame):
    raise _Timeout()


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _cflib_cache_dir() -> str:
    # Do not use the repo cache on /mnt/d — DrvFs makes the first TOC
    # download exceed the connect timeout and looks like a hang.
    cache = os.path.join(os.environ.get("TMPDIR", "/tmp"), "cflib_cache")
    os.makedirs(cache, exist_ok=True)
    return cache


def _port_bound(port: int) -> bool:
    try:
        out = subprocess.run(
            ["ss", "-ulnp"], capture_output=True, text=True, check=False
        )
    except FileNotFoundError:
        return False
    return f":{port}" in out.stdout


def _count_cf2() -> int:
    try:
        out = subprocess.run(
            ["pgrep", "-x", "cf2"], capture_output=True, text=True, check=False
        )
    except FileNotFoundError:
        return -1
    return len([ln for ln in out.stdout.splitlines() if ln.strip()])


def _dump_sitl_log(cf_id: int) -> None:
    log = os.path.join(
        _REPO_ROOT,
        "firmware_mods",
        "CrazySim",
        "crazyflie-firmware",
        "sitl_make",
        "build",
        str(cf_id),
        "error.log",
    )
    if not os.path.isfile(log):
        print(f"[swarm_loc_gate] no SITL error.log at {log}", file=sys.stderr)
        return
    try:
        with open(log, "r", errors="replace") as f:
            tail = f.readlines()[-20:]
        print(f"[swarm_loc_gate] sitl {cf_id} error.log tail:\n{''.join(tail)}", file=sys.stderr)
    except OSError as e:
        print(f"[swarm_loc_gate] could not read {log}: {e}", file=sys.stderr)


def _wait_for_sitl(n: int, connect_wait: float) -> None:
    ports = [19850 + i for i in range(n)]
    deadline = time.time() + connect_wait
    print(f"[swarm_loc_gate] waiting for SITL UDP {ports} …")
    while time.time() < deadline:
        if all(_port_bound(p) for p in ports):
            n_cf2 = _count_cf2()
            print(f"[swarm_loc_gate] SITL UDP ports up (cf2={n_cf2}) — settle 8 s …")
            if n_cf2 >= 0 and n_cf2 < n:
                print(
                    f"[swarm_loc_gate] WARN: only {n_cf2}/{n} cf2 processes",
                    file=sys.stderr,
                )
            time.sleep(8.0)
            return
        time.sleep(0.5)
    raise SystemExit(
        f"[swarm_loc_gate] TIMEOUT: cflib ports not ready.\n{_RESTART_MSG}"
    )


def _open_sync_crazyflie(uri: str, cache: str, label: str, timeout_s: float):
    """One blocking connect (same as uwb_gate). Do not retry — a timed-out
    cflib thread keeps the UDP port and makes later tries fail."""
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    print(f"[swarm_loc_gate] connecting {label} at {uri} ({timeout_s:.0f}s) …")
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
    if holder["scf"] is not None:
        print(f"[swarm_loc_gate] {label} connected")
        return holder["scf"]
    why = "timeout (cflib still blocked — restart sim)" if thread.is_alive() else repr(holder.get("err"))
    cf_id = int(uri.rsplit(":", 1)[-1]) - 19850
    _dump_sitl_log(cf_id)
    raise SystemExit(f"[swarm_loc_gate] FAIL connecting {label}: {why}\n{_RESTART_MSG}")


def parse_ros2_node_info_subscribers(text: str) -> List[str]:
    """Extract subscriber topic names from `ros2 node info` output."""
    lines = text.splitlines()
    topics: List[str] = []
    in_subs = False
    for ln in lines:
        s = ln.strip()
        if s.startswith("Subscribers"):
            in_subs = True
            continue
        if in_subs:
            if s.startswith("Publishers") or s.startswith("Service") or s.startswith("Action"):
                break
            if s.endswith(":") and not s.startswith("/"):
                break
            if s.startswith("/"):
                name = s.split(":", 1)[0].split("[", 1)[0].strip()
                if name:
                    topics.append(name)
    return topics


def node_info_subscribers(node_name: str, quiet: bool = False) -> List[str]:
    names = [node_name]
    if node_name.startswith("/"):
        names.append(node_name[1:])
    else:
        names.append("/" + node_name)
    last = ""
    for name in names:
        try:
            out = subprocess.run(
                ["ros2", "node", "info", name],
                capture_output=True,
                text=True,
                check=False,
                timeout=15,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as e:
            print(f"[swarm_loc_gate] ros2 node info failed: {e}", file=sys.stderr)
            return []
        last = (out.stdout or "") + "\n" + (out.stderr or "")
        topics = parse_ros2_node_info_subscribers(last)
        if topics:
            return topics
    if last.strip() and not quiet:
        print(f"[swarm_loc_gate] ros2 node info raw ({node_name}):\n{last[:800]}", file=sys.stderr)
    return []


def inspect_estimator_subs(num_drones: int, attempts: int = 20) -> Tuple[bool, Dict[int, list], Dict[int, list]]:
    """Retry until each /swarm_loc_i lists subscribers. Fail only on truth topics."""
    truth_hits: Dict[int, list] = {}
    all_subs: Dict[int, list] = {}
    readable = False
    leaked_any = False
    for attempt in range(attempts):
        leaked_any = False
        readable = True
        last = attempt == attempts - 1
        for i in range(num_drones):
            subs = node_info_subscribers(f"/swarm_loc_{i}", quiet=not last)
            all_subs[i] = subs
            leaked = topics_contain_truth(subs)
            truth_hits[i] = leaked
            if leaked:
                leaked_any = True
            if not any(t.endswith("/rio/delta") for t in subs):
                readable = False
        if readable or leaked_any:
            break
        time.sleep(1.0)
    return (readable and not leaked_any), truth_hits, all_subs


class EstimateRecorder:
    def __init__(self, num_drones: int):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2

        self.rows: Dict[int, list] = {i: [] for i in range(num_drones)}
        self.truth: Dict[int, list] = {i: [] for i in range(num_drones)}

        class _Node(Node):
            def __init__(self_inner):
                super().__init__("swarm_loc_gate_recorder")
                qos = qos_profile_sensor_data
                for i in range(num_drones):
                    self_inner.create_subscription(
                        PointCloud2,
                        f"/cf_{i}/swarm_loc/estimate",
                        lambda msg, idx=i: self._on_est(idx, msg),
                        qos,
                    )
                    self_inner.create_subscription(
                        Odometry,
                        f"/cf_{i}/odom",
                        lambda msg, idx=i: self._on_odom(idx, msg),
                        qos,
                    )

        rclpy.init()
        self._node = _Node()
        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self._node)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()

    def _on_est(self, idx: int, msg) -> None:
        arr = unpack_state(msg)
        if arr.size:
            self.rows[idx].append((time.time(), arr[0]))

    def _on_odom(self, idx: int, msg) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.truth[idx].append((stamp, float(p.x), float(p.y), float(p.z), float(yaw)))

    def dump_eval(self, out_dir: str) -> None:
        from eval_6_1 import TRUTH_DTYPE, write_eval_bundle

        estimates = {}
        truth = {}
        for i, rec in self.rows.items():
            if rec:
                estimates[i] = np.array([row for _, row in rec], dtype=STATE_DTYPE)
            else:
                estimates[i] = np.zeros(0, dtype=STATE_DTYPE)
        for i, rec in self.truth.items():
            arr = np.zeros(len(rec), dtype=TRUTH_DTYPE)
            for k, (stamp, x, y, z, psi) in enumerate(rec):
                arr[k]["stamp"] = stamp
                arr[k]["p_x"], arr[k]["p_y"], arr[k]["p_z"] = x, y, z
                arr[k]["psi"] = psi
            truth[i] = arr
        write_eval_bundle(Path(out_dir), estimates, truth)

    def shutdown(self) -> None:
        try:
            self._exec.shutdown()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            import rclpy

            rclpy.shutdown()
        except Exception:
            pass


class _Link:
    """Holds a Crazyflie plus optional Swarm so links stay open."""

    def __init__(self, cf, swarm=None):
        self.cf = cf
        self._swarm = swarm

    def __exit__(self, *args):
        if self._swarm is not None:
            try:
                self._swarm.close_links()
            except Exception:
                pass
            self._swarm = None
        else:
            try:
                self.cf.close_link()
            except Exception:
                pass


def connect_drones(args) -> list:
    """Open every SITL radio at once. Sequential connect hangs on drone 1
    once drone 0 already owns a UDP link (cflib/CrazySim)."""
    import cflib.crtp

    n = int(args.num_drones)
    _wait_for_sitl(n, args.connect_wait)
    cache = _cflib_cache_dir()
    cflib.crtp.init_drivers()
    uris = [f"udp://127.0.0.1:{19850 + i}" for i in range(n)]
    timeout_s = float(args.connect_timeout)

    try:
        from cflib.crazyflie.swarm import CachedCfFactory, Swarm

        factory = CachedCfFactory(rw_cache=cache)
        swarm = Swarm(uris, factory=factory)
        print(f"[swarm_loc_gate] opening {n} SITL links in parallel ({timeout_s:.0f}s) …")
        done = threading.Event()
        err: list = []

        def _open():
            try:
                swarm.open_links()
            except Exception as exc:
                err.append(exc)
            done.set()

        t = threading.Thread(target=_open, daemon=True)
        t.start()
        if not done.wait(timeout_s):
            print(
                "[swarm_loc_gate] cflib parallel open timed out. "
                "Do not treat this as a flight. Restart sim (pkill gz sim/cf2) and retry. "
                "A hung cflib thread may still hold UDP — --no-fly fallback is not a fly.",
                file=sys.stderr,
            )
            return None
        if err:
            print(f"[swarm_loc_gate] cflib Swarm failed: {err[0]}. Falling back to --no-fly.", file=sys.stderr)
            return None
        links = []
        for i, uri in enumerate(uris):
            cf = swarm._cfs[uri]
            print(f"[swarm_loc_gate] drone {i} connected ({uri})")
            links.append(_Link(cf, swarm if i == 0 else None))
        return links
    except ImportError:
        pass

    # Fallback: start every SyncCrazyflie thread at t=0 (still parallel).
    print(f"[swarm_loc_gate] Swarm API missing — parallel SyncCrazyflie …")
    holder: dict = {}

    def _one(i, uri):
        holder[i] = _open_sync_crazyflie(uri, cache, f"drone {i}", timeout_s)

    threads = [
        threading.Thread(target=_one, args=(i, uris[i]), daemon=True) for i in range(n)
    ]
    for th in threads:
        th.start()
    for th in threads:
        th.join(timeout_s + 2.0)
    missing = [i for i in range(n) if i not in holder]
    if missing:
        raise SystemExit(
            f"[swarm_loc_gate] FAIL connecting drones {missing}. Use --no-fly.\n{_RESTART_MSG}"
        )
    return [holder[i] for i in range(n)]


def _as_cf(obj):
    """Unwrap _Link / SyncCrazyflie down to Crazyflie (has .param)."""
    x = obj
    for _ in range(4):
        if x is None:
            break
        if hasattr(x, "param") and hasattr(x, "platform"):
            return x
        x = getattr(x, "cf", None)
    raise TypeError(f"cannot unwrap Crazyflie from {type(obj)}")


def run_flight(args, scfs: list) -> bool:
    from pid_gains import apply_gains, load_gains, reset_estimator, reset_pose
    from cflib.positioning.motion_commander import MotionCommander

    n = int(args.num_drones)
    gains = load_gains(args.gains)
    signal.signal(signal.SIGALRM, _alarm)
    diverged = False
    try:
        for i, scf in enumerate(scfs):
            cf = _as_cf(scf)
            apply_gains(cf, gains)
            try:
                reset_pose(
                    args.world,
                    f"{args.model_prefix}_{i}",
                    xyz=(float(i * args.spacing), 0.0, args.hover_height),
                )
            except Exception as e:
                print(f"[swarm_loc_gate] reset_pose {i} skipped: {e}", file=sys.stderr)
            reset_estimator(cf, "kalman")
        time.sleep(2.0)
        for scf in scfs:
            try:
                _as_cf(scf).platform.send_arming_request(True)
            except Exception:
                pass
        time.sleep(0.5)
        signal.setitimer(signal.ITIMER_REAL, float(args.duration) + 60.0)
        mcs = [MotionCommander(_as_cf(s), default_height=args.hover_height) for s in scfs]
        # MotionCommander is a context manager; nest via ExitStack
        from contextlib import ExitStack

        with ExitStack() as stack:
            for mc in mcs:
                stack.enter_context(mc)
            print("[swarm_loc_gate] takeoff/settle 4 s …")
            time.sleep(4.0)
            t_end = time.time() + float(args.duration)
            print(f"[swarm_loc_gate] scripted path for {args.duration:.0f} s …")
            # Slow corridor shuttle, then hold the remainder.
            try:
                for mc in mcs:
                    mc.forward(0.4)
                time.sleep(3.0)
                for mc in mcs:
                    mc.back(0.4)
                time.sleep(3.0)
            except Exception as e:
                print(f"[swarm_loc_gate] motion warning: {e}", file=sys.stderr)
            remain = t_end - time.time()
            if remain > 0:
                time.sleep(remain)
            for mc in mcs:
                try:
                    mc.stop()
                except Exception:
                    pass
    except _Timeout:
        diverged = True
        print("[swarm_loc_gate] TIMEOUT during flight", file=sys.stderr)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        for scf in reversed(scfs):
            try:
                scf.__exit__(None, None, None)
            except Exception:
                pass
    return diverged


def _rate_ok(stamps: List[float], target_hz: float, window_s: float) -> Tuple[bool, float]:
    if len(stamps) < 2 or window_s <= 0:
        return False, 0.0
    hz = (len(stamps) - 1) / window_s
    return hz >= RATE_FRAC_MIN * target_hz, hz


def run_selftest() -> int:
    ok = True

    def check(name: str, cond: bool, detail: str = ""):
        nonlocal ok
        if cond:
            print(f"[selftest] PASS {name}")
        else:
            ok = False
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))

    sample = """
/swarm_loc_0
  Subscribers:
    /cf_0/rio/delta: sensor_msgs/msg/PointCloud2
    /cf_0/uwb/edges: sensor_msgs/msg/PointCloud2
    /cf_1/swarm_loc/broadcast: sensor_msgs/msg/PointCloud2
  Publishers:
    /cf_0/swarm_loc/estimate: sensor_msgs/msg/PointCloud2
"""
    subs = parse_ros2_node_info_subscribers(sample)
    check("1 parse rio", "/cf_0/rio/delta" in subs)
    check("1b parse no publishers", "/cf_0/swarm_loc/estimate" not in subs)
    alt = parse_ros2_node_info_subscribers(
        "Node: /swarm_loc_0\n  Subscribers (3):\n    /cf_0/rio/delta [sensor_msgs/msg/PointCloud2]\n  Publishers:\n"
    )
    check("1c alt format", "/cf_0/rio/delta" in alt)
    check("2 truth catch odom", bool(topics_contain_truth(["/cf_0/odom"])))
    check("2b estimate not truth", not topics_contain_truth(["/cf_0/swarm_loc/estimate"]))
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--config", default="configs/estimation/swarm_loc.yaml")
    parser.add_argument("--gains", default="configs/airframe/pid_gains_loaded.yaml")
    parser.add_argument("--world", default="phase0_tunnel_gate")
    parser.add_argument("--model-prefix", default="crazyflie")
    parser.add_argument("--num-drones", type=int, default=3)
    parser.add_argument("--spacing", type=float, default=1.5)
    parser.add_argument("--hover-height", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--connect-wait", type=float, default=90.0)
    parser.add_argument("--connect-timeout", type=float, default=90.0)
    parser.add_argument(
        "--no-fly",
        action="store_true",
        help="Skip cflib (drones stay on the ground). Still checks estimate rate / no-truth / diverge.",
    )
    parser.add_argument("--no-mlflow", action="store_true")
    parser.add_argument(
        "--eval-dir",
        default="",
        help="Write truth.npz + estimates.npz (odom subscribed here only) and run §6.1 metrics.",
    )
    parser.add_argument(
        "--logs",
        default="",
        help="Measurement log dir from phase0_gate.sh --swarm-loc-log-dir (UWB mix / hops / CPU).",
    )
    args = parser.parse_args()
    if args.selftest:
        sys.exit(run_selftest())

    cfg_path = args.config
    if not os.path.isabs(cfg_path):
        cfg_path = os.path.join(_REPO_ROOT, cfg_path)
    cfg = load_config(cfg_path)
    target_hz = float(cfg["estimator"]["rate_hz"])
    n = int(args.num_drones)

    flight_fail = False
    fly_fallback = False
    explicit_no_fly = bool(args.no_fly)
    recorder = None
    try:
        if args.no_fly:
            recorder = EstimateRecorder(n)
            print(f"[swarm_loc_gate] --no-fly: recording estimates for {args.duration:.0f} s …")
            time.sleep(float(args.duration))
        else:
            scfs = connect_drones(args)
            if scfs is None:
                fly_fallback = True
                recorder = EstimateRecorder(n)
                print(
                    "[swarm_loc_gate] radios did not connect — recording estimates on the ground. "
                    "flight check will FAIL. Kill sim leftovers and rerun for a real fly.",
                    flush=True,
                )
                time.sleep(float(args.duration))
            else:
                recorder = EstimateRecorder(n)
                flight_fail = run_flight(args, scfs)
        no_truth, truth_hits, all_subs = inspect_estimator_subs(n)
    finally:
        if recorder is not None:
            recorder.shutdown()

    checks = {}
    rates = {}
    finite_ok = True
    diverged = False
    for i in range(n):
        rec = recorder.rows[i]
        stamps = [t for t, _ in rec]
        t0 = stamps[0] if stamps else 0.0
        t1 = stamps[-1] if stamps else 0.0
        ok_r, hz = _rate_ok(stamps, target_hz, t1 - t0)
        rates[i] = hz
        checks[f"rate_hz_cf_{i}"] = ok_r
        for _, row in rec:
            p = [float(row["p_x"]), float(row["p_y"]), float(row["p_z"])]
            if not all(math.isfinite(v) for v in p):
                finite_ok = False
            if int(row["status"]) != 0:
                diverged = True
        checks[f"samples_cf_{i}"] = len(rec) > 10
    checks["finite"] = finite_ok
    checks["non_diverged"] = (not diverged) and (not flight_fail)
    checks["no_truth_subs"] = no_truth
    if explicit_no_fly:
        checks["flight"] = True
    else:
        checks["flight"] = (not flight_fail) and (not fly_fallback)
    gate_pass = all(checks.values())

    print("\n[swarm_loc_gate] results:")
    for i in range(n):
        print(f"  cf_{i} estimate_hz={rates.get(i, 0):.2f}  n={len(recorder.rows[i])}")
        print(f"  cf_{i} n_subs={len(all_subs.get(i, []))} truth_subs={truth_hits.get(i, [])}")
    for k, v in checks.items():
        print(f"  {'PASS' if v else 'FAIL'} {k}")
    print(f"\n[swarm_loc_gate] {'PASS' if gate_pass else 'FAIL'}")

    eval_dir = args.eval_dir.strip()
    if eval_dir and recorder is not None:
        out = Path(eval_dir)
        recorder.dump_eval(out)
        n_truth = sum(len(recorder.truth[i]) for i in range(n))
        print(f"[swarm_loc_gate] wrote {out / 'truth.npz'} and estimates.npz (odom samples={n_truth})")
        logs = args.logs.strip() or (str(out) if list(out.glob("cf_*.npz")) else "")
        try:
            from eval_6_1 import evaluate, load_run, load_structured_npz, print_report, TRUTH_DTYPE

            run = load_run(logs) if logs and Path(logs).exists() else None
            truth = load_structured_npz(out / "truth.npz", TRUTH_DTYPE)
            estimates = load_structured_npz(out / "estimates.npz")
            report = evaluate(run, truth, estimates, out_dir=out)
            print_report(report)
        except Exception as e:
            import traceback

            traceback.print_exc()
            print(f"[swarm_loc_gate] eval_6_1 skipped: {e}", file=sys.stderr)

    if not args.no_mlflow:
        try:
            import mlflow

            mlflow.set_tracking_uri("sqlite:///mlflow.db")
            mlflow.set_experiment(cfg.get("mlflow_experiment", "phase2_swarm_loc"))
            with mlflow.start_run(run_name="swarm_loc_gate"):
                mlflow.log_param("num_drones", n)
                mlflow.log_param("duration_s", args.duration)
                mlflow.log_param("rate_hz", target_hz)
                for i, hz in rates.items():
                    mlflow.log_metric(f"estimate_hz_cf_{i}", hz)
                mlflow.log_metric("gate_pass", int(gate_pass))
        except Exception as e:
            print(f"[swarm_loc_gate] MLflow skipped: {e}", file=sys.stderr)

    sys.exit(0 if gate_pass else 1)


if __name__ == "__main__":
    main()
