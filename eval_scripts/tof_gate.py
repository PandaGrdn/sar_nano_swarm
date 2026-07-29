#!/usr/bin/env python3
"""tof_gate.py — Phase 1 M3 exit gate for the IR ToF rangefinder.

Validates two things against a running Gazebo + SITL instance (spawned by
phase0_gate.sh with the ToF sensor injected — see apply_tof_sensor.py):

  1. Rate:  /cf_<id>/tof_down publishes at >= --min-rate-hz. Pure gz-transport
     check, no cflib/firmware involved.
  2. Altitude tracking: takes off via cflib/MotionCommander (reusing
     hover_gate.py's proven connect/arm/hover pattern from M2) and climbs
     through a sequence of PID-held hover plateaus, sampling the down-beam
     range at each and comparing it to the EKF's stateEstimate.z.

Design note: an earlier version of this gate tried to validate altitude
tracking by teleporting the (unpowered, gravity-on) drone to test heights via
`gz service .../set_pose` and reading the sensor immediately after. That
doesn't work: each `gz service`/`gz topic` CLI round-trip costs ~0.3-0.5 s,
during which the drone free-falls ~0.5-1 m — verified live (a teleport to
z=2.0 m read back as 0.895 m by the time the sample was taken). Pausing the
world and using WorldControl's `multi_step` to advance a bounded, deterministic
amount of sim time was also tried live and reliably returned "Infinity"
(no valid return) — the gz-sim-sensors-system's render/update loop did not
appear to produce a fresh scan from manual stepping in this Gazebo Harmonic
8.14 build (unconfirmed why; not chased further; don't rely on multi_step for
sensor cadence here). Hovering under the flight controller's own PID hold
(same approach as hover_gate.py) sidesteps the free-fall problem entirely: the
drone isn't moving, so subprocess latency doesn't matter.

Prereq: a running SITL + Gazebo instance, e.g.
    ./eval_scripts/phase0_gate.sh -w phase1_pid_tune --no-radar --headless
NOTE (same CRTP v7 quirk as hover_gate.py): run this as the FIRST cflib
connection against a freshly launched sim.

Usage (setup_env.sh sourced):
    python3 eval_scripts/tof_gate.py --config configs/sensors/tof.yaml
"""
import argparse
import json
import re
import signal
import subprocess
import sys
import time
import warnings

import mlflow
import yaml

# cflib's legacy CRTP v7 hover-commander / arming-request deprecation
# warnings otherwise spam every ~50 ms log tick (see hover_gate.py's docstring
# for the same CRTP v7 firmware quirk).
warnings.filterwarnings("ignore", category=DeprecationWarning, module="cflib.*")
warnings.filterwarnings("ignore", category=UserWarning, module="cflib.*")

SENSOR_OFFSET_M_DEFAULT = 0.02  # matches tof.yaml's down.pose_xyz_rpy z offset


class _Timeout(Exception):
    pass


def _alarm(sig, frame):
    raise _Timeout()


def load_config(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _parse_gz_float(value):
    """gz-transport's --json-output serializes non-finite doubles (inf/nan) as
    JSON *strings* ("Infinity", "-Infinity", "NaN") since JSON has no literal
    for them (verified live: `gz topic -e --json-output` on a gpu_lidar with
    no valid return emits `"ranges":["Infinity"]`). float() handles these
    strings natively.
    """
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def sample_range(topic, timeout_s=3.0):
    """Echo one gz.msgs.LaserScan message as JSON and return ranges[0], or None."""
    cmd = ["gz", "topic", "-e", "-t", topic, "-n", "1", "--json-output"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return None
    if result.returncode != 0 or not result.stdout.strip():
        return None
    try:
        msg = json.loads(result.stdout.strip().splitlines()[0])
    except (json.JSONDecodeError, IndexError):
        return None
    ranges = msg.get("ranges")
    if not ranges:
        return None
    return _parse_gz_float(ranges[0])


def sample_range_stable(topic, n=6, per_sample_timeout=3.0):
    """Average n range reads (used while the drone is holding a hover — no
    free-fall risk, so it's fine to take a moment)."""
    vals = []
    for _ in range(n):
        r = sample_range(topic, timeout_s=per_sample_timeout)
        if r is not None and r == r and abs(r) != float("inf"):
            vals.append(r)
    return (sum(vals) / len(vals)) if vals else None


def measure_rate(topic, duration_s=5.0):
    """Sample `gz topic -f` for duration_s and parse the average Hz.

    NOTE: verified live that `gz topic -f --duration N` does NOT actually
    self-terminate after N seconds — it streams "average rate:" blocks
    forever (every ~10 samples) until killed. So this runs it under Popen and
    explicitly terminates it after duration_s, then parses the LAST block
    captured (freshest window).
    """
    cmd = ["gz", "topic", "-f", "-t", topic]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        out, _ = proc.communicate(timeout=duration_s)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            out, _ = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate()
    matches = re.findall(r"average rate:\s*([0-9.]+)", out, re.IGNORECASE)
    if matches:
        return float(matches[-1])
    return None


def run_rate_check(topic, min_rate):
    print("[tof_gate] Measuring publish rate over 5 s …")
    rate_hz = measure_rate(topic, duration_s=5.0)
    rate_pass = rate_hz is not None and rate_hz >= min_rate
    if rate_hz is None:
        print(f"[tof_gate] Rate measurement FAILED — topic '{topic}' may not exist (check `gz topic -l`).")
    else:
        print(f"[tof_gate] Measured rate: {rate_hz:.2f} Hz ({'PASS' if rate_pass else 'FAIL'})")
    return rate_hz, rate_pass


def run_altitude_check(args, topic, sensor_z_offset):
    """Climb through hover plateaus (PID-held, per hover_gate.py's proven
    connect/arm/MotionCommander pattern) and compare the down-beam range to
    the EKF's stateEstimate.z at each plateau."""
    from pid_gains import load_gains, apply_gains, reset_estimator, reset_pose
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.crazyflie.syncLogger import SyncLogger
    from cflib.crazyflie.log import LogConfig
    from cflib.positioning.motion_commander import MotionCommander

    gains = load_gains(args.gains)
    cflib.crtp.init_drivers()
    signal.signal(signal.SIGALRM, _alarm)

    base_height = args.heights[0]
    results = []
    diverged = False

    print(f"[tof_gate] connecting {args.uri} (gains: {args.gains}) …")
    signal.setitimer(signal.ITIMER_REAL, 20.0 + 8.0 * len(args.heights))
    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
            cf = scf.cf
            apply_gains(cf, gains)
            try:
                reset_pose(args.world, args.model, xyz=(0.0, 0.0, base_height))
            except Exception as e:
                print(f"[tof_gate] reset_pose skipped: {e}", file=sys.stderr)
            reset_estimator(cf, "kalman")
            time.sleep(2.0)
            try:
                cf.platform.send_arming_request(True)
                time.sleep(0.5)
            except Exception as e:
                print(f"[tof_gate] arming request skipped: {e}", file=sys.stderr)

            lg = LogConfig(name="tof_alt", period_in_ms=50)
            for v in ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z"]:
                lg.add_variable(v, "float")

            with MotionCommander(scf, default_height=base_height) as mc:
                time.sleep(2.5)  # settle after takeoff
                prev_h = base_height
                for h in args.heights:
                    delta = h - prev_h
                    if abs(delta) > 1e-3:
                        if delta > 0:
                            mc.up(delta)
                        else:
                            mc.down(-delta)
                    prev_h = h
                    time.sleep(1.5)  # settle at new plateau

                    zs = []
                    with SyncLogger(scf, lg) as logger:
                        t0 = time.time()
                        for entry in logger:
                            d = entry[1]
                            z = d["stateEstimate.z"]
                            zs.append(z)
                            if abs(z - h) > 1.0:
                                diverged = True
                                break
                            if time.time() - t0 >= 1.0:
                                break
                    mean_z = sum(zs) / len(zs) if zs else h

                    measured = sample_range_stable(topic, n=args.samples_per_height)
                    expected = max(mean_z - sensor_z_offset, 0.0)
                    if measured is None:
                        print(f"[tof_gate] target={h:.2f} m actual_z={mean_z:.3f} m: no valid ToF samples — FAIL")
                        results.append({"height": h, "z": mean_z, "measured": None, "expected": expected,
                                         "error": None, "passed": False})
                        continue
                    error = abs(measured - expected)
                    passed = error <= args.tolerance_m
                    print(f"[tof_gate] target={h:.2f} m actual_z={mean_z:.3f} m: measured={measured:.3f} m "
                          f"expected={expected:.3f} m error={error:.3f} m ({'PASS' if passed else 'FAIL'})")
                    results.append({"height": h, "z": mean_z, "measured": measured, "expected": expected,
                                     "error": error, "passed": passed})

                    if diverged:
                        break
                mc.stop()
    except _Timeout:
        diverged = True
        print("[tof_gate] TIMEOUT — firmware likely crashed", file=sys.stderr)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)

    return results, diverged


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="phase1_pid_tune")
    parser.add_argument("--model", default="crazyflie_0")
    parser.add_argument("--cf-id", default="0")
    parser.add_argument("--uri", default="udp://127.0.0.1:19850")
    parser.add_argument("--gains", default="configs/airframe/pid_gains_loaded.yaml")
    parser.add_argument("--config", default="configs/sensors/tof.yaml")
    parser.add_argument("--min-rate-hz", type=float, default=None,
                         help="Overrides config's update_rate_hz * 0.8 default floor")
    parser.add_argument("--tolerance-m", type=float, default=0.12,
                         help="Max allowed |measured - EKF z| range error (looser than the "
                              "sensor's 1 cm noise floor to absorb EKF z error + attitude tilt)")
    parser.add_argument("--heights", type=float, nargs="+", default=[0.3, 0.6, 1.0])
    parser.add_argument("--samples-per-height", type=int, default=6)
    parser.add_argument("--rate-only", action="store_true", help="Skip the cflib altitude-hover check")
    args = parser.parse_args()

    cfg = load_config(args.config)
    topic = f"/cf_{args.cf_id}/tof_down"
    sensor_z_offset = abs(cfg.get("down", {}).get("pose_xyz_rpy", [0, 0, -SENSOR_OFFSET_M_DEFAULT])[2])
    min_rate = args.min_rate_hz if args.min_rate_hz is not None else cfg["update_rate_hz"] * 0.8

    print(f"[tof_gate] topic={topic}  expected rate>={min_rate:.1f} Hz  "
          f"tolerance=±{args.tolerance_m} m  sensor_z_offset={sensor_z_offset} m")

    rate_hz, rate_pass = run_rate_check(topic, min_rate)

    if args.rate_only:
        overall_pass = rate_pass
        height_results, diverged = [], False
    else:
        height_results, diverged = run_altitude_check(args, topic, sensor_z_offset)
        alt_pass = bool(height_results) and (not diverged) and all(r["passed"] for r in height_results)
        overall_pass = rate_pass and alt_pass

    try:
        mlflow.set_tracking_uri("sqlite:///mlflow.db")
        mlflow.set_experiment(cfg.get("mlflow_experiment", "phase1_tof_sensor"))
        with mlflow.start_run(run_name="tof_gate"):
            mlflow.log_param("topic", topic)
            mlflow.log_param("min_rate_hz", min_rate)
            mlflow.log_param("tolerance_m", args.tolerance_m)
            if rate_hz is not None:
                mlflow.log_metric("measured_rate_hz", rate_hz)
            for r in height_results:
                if r["measured"] is not None:
                    mlflow.log_metric(f"range_error_m_z{r['height']}", r["error"])
            mlflow.log_metric("rate_pass", int(rate_pass))
            mlflow.log_metric("diverged", int(diverged))
            mlflow.log_metric("gate_pass", int(overall_pass))
    except Exception as exc:  # pragma: no cover - MLflow issues shouldn't fail the gate
        print(f"[tof_gate] WARNING: MLflow logging failed: {exc}", file=sys.stderr)

    print("")
    if overall_pass:
        print(f"[GATE] PASS — {topic} @ {rate_hz:.2f} Hz, altitude tracking within ±{args.tolerance_m} m")
    else:
        print(f"[GATE] FAIL — rate_pass={rate_pass} diverged={diverged}")
    sys.exit(0 if overall_pass else 1)


if __name__ == "__main__":
    main()
