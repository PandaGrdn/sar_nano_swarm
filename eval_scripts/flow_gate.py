#!/usr/bin/env python3
"""flow_gate.py — Phase 1 M3b exit gate for the PMW3901 optical-flow sensor.

Validates the analytic flow_node stream against a running Gazebo + SITL instance
(launched by phase0_gate.sh with USE_FLOW=true — see §6 of the M3b plan):

  1. Publish rate on /cf_<id>/flow >= 0.8 * update_rate_hz.
  2. Hover window: valid fraction, RMS horizontal velocity.
  3. Forward-flight window: RMSE vs debug_truth, vx bias/sign.
  4. Smooth-patch dropout window: valid fraction near zero, invalid sentinel cov.

Prereq: a running SITL + Gazebo instance, e.g.
    ./eval_scripts/phase0_gate.sh -w phase1_pid_tune --no-radar --headless
NOTE (same CRTP v7 quirk as hover_gate.py / tof_gate.py): run this as the
FIRST cflib connection against a freshly launched sim.

Usage (setup_env.sh sourced):
    python3 -u eval_scripts/flow_gate.py --config configs/sensors/optical_flow.yaml
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
import warnings
from dataclasses import dataclass
from typing import List, Optional, Tuple

import mlflow
import rclpy
import yaml
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

warnings.filterwarnings("ignore", category=DeprecationWarning, module="cflib.*")
warnings.filterwarnings("ignore", category=UserWarning, module="cflib.*")

MIN_RATE_FRAC = 0.8
HOVER_VALID_FRAC_MIN = 0.9
PATCH_VALID_FRAC_MAX = 0.1
STAMP_PAIR_MAX_S = 0.015


def velocity_noise_rms_mps(cfg: dict, height_m: float) -> float:
    """2D RSS of per-axis velocity noise from pixel std (plan §2.3–2.4)."""
    dt = 1.0 / float(cfg["update_rate_hz"])
    k = float(cfg["npix"]) / float(cfg["thetapix_rad"])
    sigma_v = (
        float(cfg["flow_std_px"])
        * float(cfg["flow_resolution"])
        / (dt * k)
        * height_m
    )
    return math.sqrt(2.0) * sigma_v


class _Timeout(Exception):
    pass


def _alarm(sig, frame):
    raise _Timeout()


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


@dataclass
class FlowSample:
    t: float          # message header stamp (sim time)
    recv_t: float     # wall clock at receive — used for flight-window scoring
    vx: float
    vy: float
    valid: bool
    cov0: float


@dataclass
class TruthSample:
    t: float
    recv_t: float
    vx: float
    vy: float


class FlowRecorder(Node):
    def __init__(self, cf_id: str, invalid_variance: float):
        super().__init__("flow_gate_recorder")
        self.cf_id = cf_id
        self.invalid_variance = invalid_variance
        self.flow_topic = f"/cf_{cf_id}/flow"
        self.truth_topic = f"/cf_{cf_id}/flow/debug_truth"
        self.flow_samples: List[FlowSample] = []
        self.truth_samples: List[TruthSample] = []
        self._rate_t0: Optional[float] = None
        self._rate_count = 0
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(
            TwistWithCovarianceStamped, self.flow_topic, self._on_flow, qos
        )
        self.create_subscription(TwistStamped, self.truth_topic, self._on_truth, qos)

    def _on_flow(self, msg: TwistWithCovarianceStamped):
        t = stamp_to_sec(msg.header.stamp)
        recv_t = time.time()
        cov0 = float(msg.twist.covariance[0])
        valid = cov0 < self.invalid_variance * 0.5
        self.flow_samples.append(
            FlowSample(
                t=t,
                recv_t=recv_t,
                vx=float(msg.twist.twist.linear.x),
                vy=float(msg.twist.twist.linear.y),
                valid=valid,
                cov0=cov0,
            )
        )
        if self._rate_t0 is None:
            self._rate_t0 = recv_t
        self._rate_count += 1

    def _on_truth(self, msg: TwistStamped):
        self.truth_samples.append(
            TruthSample(
                t=stamp_to_sec(msg.header.stamp),
                recv_t=time.time(),
                vx=float(msg.twist.linear.x),
                vy=float(msg.twist.linear.y),
            )
        )

    def measured_rate_hz(self) -> Optional[float]:
        if self._rate_t0 is None or self._rate_count < 2:
            return None
        t_last = self.flow_samples[-1].recv_t
        dt = t_last - self._rate_t0
        return (self._rate_count - 1) / dt if dt > 0 else None

    def rate_over_window(self, t0: float, t1: float) -> Optional[float]:
        pts = [s for s in self.flow_samples if t0 <= s.recv_t <= t1]
        if len(pts) < 2:
            return None
        return (len(pts) - 1) / (pts[-1].recv_t - pts[0].recv_t)

    def samples_in(self, t0: float, t1: float) -> List[FlowSample]:
        return [s for s in self.flow_samples if t0 <= s.recv_t <= t1]

    def nearest_truth(self, flow: FlowSample) -> Optional[TruthSample]:
        if not self.truth_samples:
            return None
        best = min(self.truth_samples, key=lambda s: abs(s.t - flow.t))
        return best if abs(best.t - flow.t) <= STAMP_PAIR_MAX_S else None


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def valid_fraction(samples: List[FlowSample]) -> float:
    if not samples:
        return 0.0
    return sum(1 for s in samples if s.valid) / len(samples)


def hover_rms_mps(samples: List[FlowSample]) -> Optional[float]:
    valid = [s for s in samples if s.valid]
    if not valid:
        return None
    return math.sqrt(sum(s.vx * s.vx + s.vy * s.vy for s in valid) / len(valid))


def forward_metrics(
    recorder: FlowRecorder, samples: List[FlowSample]
) -> Tuple[Optional[float], Optional[float], bool]:
    """Return (rmse_mps, vx_bias_mps, sign_ok) over valid samples paired to truth."""
    errs = []
    vx_meas = []
    vx_true = []
    for s in samples:
        if not s.valid:
            continue
        tr = recorder.nearest_truth(s)
        if tr is None:
            continue
        errs.append((s.vx - tr.vx) ** 2 + (s.vy - tr.vy) ** 2)
        vx_meas.append(s.vx)
        vx_true.append(tr.vx)
    if not errs:
        return None, None, True
    rmse = math.sqrt(sum(errs) / len(errs))
    bias = sum(m - t for m, t in zip(vx_meas, vx_true)) / len(vx_meas)
    mean_true = sum(vx_true) / len(vx_true)
    mean_meas = sum(vx_meas) / len(vx_meas)
    sign_ok = (mean_meas * mean_true) >= 0 if abs(mean_true) > 0.05 else True
    return rmse, bias, sign_ok


def rmse_vs_truth(
    recorder: FlowRecorder, samples: List[FlowSample]
) -> Optional[float]:
    errs = []
    for s in samples:
        if not s.valid:
            continue
        tr = recorder.nearest_truth(s)
        if tr is None:
            continue
        errs.append((s.vx - tr.vx) ** 2 + (s.vy - tr.vy) ** 2)
    return math.sqrt(sum(errs) / len(errs)) if errs else None


def invalid_sentinel_ok(samples: List[FlowSample], invalid_variance: float) -> bool:
    invalid = [s for s in samples if not s.valid]
    if not invalid:
        return True
    for s in invalid:
        if abs(s.cov0 - invalid_variance) > 1.0:
            return False
        if abs(s.vx) > 1e-9 or abs(s.vy) > 1e-9:
            return False
    return True


def run_flight_sequence(
    args, recorder: FlowRecorder
) -> Tuple[float, float, float, float, float, float, bool]:
    """Returns window timestamps and diverged flag."""
    from pid_gains import load_gains, apply_gains, reset_estimator, reset_pose
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander

    gains = load_gains(args.gains)
    cflib.crtp.init_drivers()
    signal.signal(signal.SIGALRM, _alarm)

    hover_h = args.hover_height
    diverged = False
    t_hover0 = t_hover1 = t_fwd0 = t_fwd1 = t_patch0 = t_patch1 = 0.0

    print(f"[flow_gate] connecting {args.uri} (gains: {args.gains}) …")
    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
            cf = scf.cf
            apply_gains(cf, gains)
            try:
                reset_pose(args.world, args.model, xyz=(0.0, 0.0, hover_h))
            except Exception as e:
                print(f"[flow_gate] reset_pose skipped: {e}", file=sys.stderr)
            reset_estimator(cf, "kalman")
            time.sleep(2.0)
            try:
                cf.platform.send_arming_request(True)
                time.sleep(0.5)
            except Exception as e:
                print(f"[flow_gate] arming request skipped: {e}", file=sys.stderr)

            signal.setitimer(signal.ITIMER_REAL, 75.0)
            with MotionCommander(scf, default_height=hover_h) as mc:
                print(f"[flow_gate] leg 0: takeoff to {hover_h} m, settle 2.5 s …")
                time.sleep(2.5)

                print("[flow_gate] leg 1: hover 5 s (window A) …")
                t_hover0 = time.time()
                time.sleep(5.0)
                t_hover1 = time.time()

                print("[flow_gate] leg 2: forward 0.8 m @ 0.3 m/s (window B) …")
                t_fwd0 = time.time()
                mc.forward(0.8, velocity=0.3)
                time.sleep(1.0)
                t_fwd1 = time.time()

                print("[flow_gate] leg 3: forward 1.6 m into smooth patch (window C) …")
                t_patch0 = time.time()
                mc.forward(1.6, velocity=0.3)
                time.sleep(2.0)
                t_patch1 = time.time()

                print("[flow_gate] leg 4: return and stop …")
                mc.back(2.4)
                mc.stop()
    except _Timeout:
        diverged = True
        print("[flow_gate] TIMEOUT — firmware likely crashed", file=sys.stderr)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)

    return t_hover0, t_hover1, t_fwd0, t_fwd1, t_patch0, t_patch1, diverged


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="phase1_pid_tune")
    parser.add_argument("--model", default="crazyflie_0")
    parser.add_argument("--cf-id", default="0")
    parser.add_argument("--uri", default="udp://127.0.0.1:19850")
    parser.add_argument("--gains", default="configs/airframe/pid_gains_loaded.yaml")
    parser.add_argument("--config", default="configs/sensors/optical_flow.yaml")
    parser.add_argument("--hover-height", type=float, default=0.5)
    args = parser.parse_args()

    cfg = load_config(args.config)
    update_rate_hz = float(cfg["update_rate_hz"])
    invalid_variance = float(cfg["invalid_variance"])
    min_rate_hz = update_rate_hz * MIN_RATE_FRAC
    # Single-frame derived-vx noise floor at hover height (ToF mount −0.02 m).
    hover_h_eff = max(args.hover_height - 0.02, float(cfg["good_height_min_m"]))
    noise_rms = velocity_noise_rms_mps(cfg, hover_h_eff)
    hover_rms_max = noise_rms * 1.15
    fwd_rmse_max = noise_rms * 1.6
    fwd_vx_bias_max = 0.08

    rclpy.init()
    recorder = FlowRecorder(cf_id=args.cf_id, invalid_variance=invalid_variance)
    executor = MultiThreadedExecutor()
    executor.add_node(recorder)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(
        f"[flow_gate] topics /cf_{args.cf_id}/flow + debug_truth  "
        f"min_rate>={min_rate_hz:.1f} Hz"
    )

    # Brief wait for flow_node to start publishing before arming.
    t_wait0 = time.time()
    while time.time() - t_wait0 < 5.0 and len(recorder.flow_samples) < 5:
        time.sleep(0.2)
    if len(recorder.flow_samples) < 10:
        print("[flow_gate] WARNING: few flow samples before flight — is flow_node running?")

    t_hover0, t_hover1, t_fwd0, t_fwd1, t_patch0, t_patch1, diverged = run_flight_sequence(
        args, recorder
    )

    time.sleep(0.5)
    executor.shutdown()
    recorder.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

    hover_samples = recorder.samples_in(t_hover0, t_hover1)
    fwd_samples = recorder.samples_in(t_fwd0, t_fwd1)
    patch_samples = recorder.samples_in(t_patch0, t_patch1)

    measured_rate = recorder.rate_over_window(t_hover0, t_patch1)
    if measured_rate is None:
        measured_rate = recorder.measured_rate_hz()

    rate_pass = measured_rate is not None and measured_rate >= min_rate_hz
    hover_valid_frac = valid_fraction(hover_samples)
    hover_rms = hover_rms_mps(hover_samples)
    hover_rmse_truth = rmse_vs_truth(recorder, hover_samples)
    hover_valid_pass = hover_valid_frac >= HOVER_VALID_FRAC_MIN
    hover_rms_pass = hover_rms is not None and hover_rms <= hover_rms_max

    fwd_rmse, fwd_vx_bias, fwd_sign_ok = forward_metrics(recorder, fwd_samples)
    fwd_rmse_pass = fwd_rmse is not None and fwd_rmse <= fwd_rmse_max
    fwd_bias_pass = (
        fwd_vx_bias is not None
        and fwd_sign_ok
        and abs(fwd_vx_bias) <= fwd_vx_bias_max
    )

    patch_valid_frac = valid_fraction(patch_samples)
    patch_valid_pass = patch_valid_frac <= PATCH_VALID_FRAC_MAX
    patch_sentinel_pass = invalid_sentinel_ok(patch_samples, invalid_variance)

    diverged_pass = not diverged
    gate_pass = all(
        [
            rate_pass,
            hover_valid_pass,
            hover_rms_pass,
            fwd_rmse_pass,
            fwd_bias_pass,
            patch_valid_pass,
            patch_sentinel_pass,
            diverged_pass,
        ]
    )

    print("")
    print(f"[flow_gate] measured_rate_hz={measured_rate} ({'PASS' if rate_pass else 'FAIL'})")
    print(
        f"[flow_gate] hover valid_frac={hover_valid_frac:.3f} "
        f"rms={hover_rms} rmse_vs_truth={hover_rmse_truth} "
        f"({'PASS' if hover_valid_pass and hover_rms_pass else 'FAIL'})"
    )
    print(
        f"[flow_gate] forward rmse={fwd_rmse} vx_bias={fwd_vx_bias} sign_ok={fwd_sign_ok} "
        f"({'PASS' if fwd_rmse_pass and fwd_bias_pass else 'FAIL'})"
    )
    print(
        f"[flow_gate] patch valid_frac={patch_valid_frac:.3f} sentinel_ok={patch_sentinel_pass} "
        f"({'PASS' if patch_valid_pass and patch_sentinel_pass else 'FAIL'})"
    )
    print(f"[flow_gate] diverged={diverged} ({'PASS' if diverged_pass else 'FAIL'})")

    try:
        mlflow.set_tracking_uri("sqlite:///mlflow.db")
        mlflow.set_experiment(cfg.get("mlflow_experiment", "phase1_optical_flow"))
        with mlflow.start_run(run_name="flow_gate"):
            mlflow.log_param("seed", cfg.get("seed", 0))
            mlflow.log_param("update_rate_hz", update_rate_hz)
            mlflow.log_param("flow_std_px", cfg.get("flow_std_px"))
            mlflow.log_param("height_source", cfg.get("height_source"))
            mlflow.log_param("min_rate_hz", min_rate_hz)
            mlflow.log_param("hover_valid_frac_min", HOVER_VALID_FRAC_MIN)
            mlflow.log_param("hover_rms_max_mps", hover_rms_max)
            mlflow.log_param("fwd_rmse_max_mps", fwd_rmse_max)
            mlflow.log_param("fwd_vx_bias_max_mps", fwd_vx_bias_max)
            mlflow.log_param("noise_rms_floor_mps", noise_rms)
            mlflow.log_param("patch_valid_frac_max", PATCH_VALID_FRAC_MAX)
            if measured_rate is not None:
                mlflow.log_metric("measured_rate_hz", measured_rate)
            if hover_rms is not None:
                mlflow.log_metric("hover_rms_mps", hover_rms)
            if fwd_rmse is not None:
                mlflow.log_metric("fwd_rmse_mps", fwd_rmse)
            if fwd_vx_bias is not None:
                mlflow.log_metric("fwd_vx_bias_mps", fwd_vx_bias)
            mlflow.log_metric("hover_valid_frac", hover_valid_frac)
            mlflow.log_metric("patch_valid_frac", patch_valid_frac)
            mlflow.log_metric("diverged", int(diverged))
            mlflow.log_metric("gate_pass", int(gate_pass))
    except Exception as exc:
        print(f"[flow_gate] WARNING: MLflow logging failed: {exc}", file=sys.stderr)

    print("")
    if gate_pass:
        print("[GATE] PASS — optical flow M3b gate")
    else:
        print("[GATE] FAIL — see checks above")
    sys.exit(0 if gate_pass else 1)


if __name__ == "__main__":
    main()
