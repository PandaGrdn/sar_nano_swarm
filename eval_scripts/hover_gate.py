#!/usr/bin/env python3
"""hover_gate.py — Phase 1 M2 gate: does the loaded-mass Crazyflie hold a
stable hover with the re-tuned PID gains?

Applies a gain set (default: configs/airframe/pid_gains_loaded.yaml, the output
of tune_pid.py), takes off to a hover height, holds for a fixed window, and
measures position-hold quality:
  - mean / abs Z error vs the target height
  - sustained RMS horizontal drift  sqrt(x^2 + y^2)
  - peak horizontal drift (reported for info only)

Pass criterion (M2 "hover holds"): took off, did not diverge, sustained RMS
horizontal drift < RMS_MAX, and mean |Z error| < ZERR_MAX. RMS (sustained) is
the right measure for "does it hold a hover"; a single settle transient does
not gate. The tighter +/-10 cm envelope under sensor-noise + turbulence is the
Phase-1 EXIT gate (M5), not this one.

Prereq: a running SITL + Gazebo instance, e.g.
    ./eval_scripts/phase0_gate.sh -w phase1_pid_tune --no-radar --headless
Then (setup_env.sh sourced):
    python3 eval_scripts/hover_gate.py                 # tuned gains
    python3 eval_scripts/hover_gate.py --stock         # stock baseline

NOTE (firmware quirk): this CrazySim build is CRTP v7 and auto-arms at sim
boot but cannot be reliably re-armed by cflib after a disconnect. Run this as
the FIRST cflib connection against a freshly launched sim; a second run in the
same sim session may not take off. Every run logs to MLflow (sqlite:///mlflow.db).
"""
import argparse
import math
import signal
import sys
import time

RMS_MAX = 0.10      # m, max sustained RMS horizontal drift for M2 pass
ZERR_MAX = 0.05     # m, max mean abs altitude error for M2 pass


class _Timeout(Exception):
    pass


def _alarm(sig, frame):
    raise _Timeout()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gains", default="configs/airframe/pid_gains_loaded.yaml")
    ap.add_argument("--stock", action="store_true",
                    help="Use stock gains instead (baseline comparison)")
    ap.add_argument("--uri", default="udp://127.0.0.1:19850")
    ap.add_argument("--height", type=float, default=0.5)
    ap.add_argument("--hold", type=float, default=12.0)
    ap.add_argument("--world", default="phase1_pid_tune")
    ap.add_argument("--model", default="crazyflie_0")
    ap.add_argument("--no-mlflow", action="store_true")
    args = ap.parse_args()

    from pid_gains import load_gains, apply_gains, reset_estimator, reset_pose
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.crazyflie.syncLogger import SyncLogger
    from cflib.crazyflie.log import LogConfig
    from cflib.positioning.motion_commander import MotionCommander

    gains_path = "configs/airframe/pid_gains_stock.yaml" if args.stock else args.gains
    gains = load_gains(gains_path)
    label = "stock" if args.stock else "loaded"

    cflib.crtp.init_drivers()
    signal.signal(signal.SIGALRM, _alarm)
    print(f"[hover] connecting {args.uri} (gains: {gains_path}) …")

    xs, ys, zs = [], [], []
    diverged = False
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        apply_gains(cf, gains)
        # Teleport to a known spawn pose (in the air) + reset the estimator so
        # the drone launches cleanly, mirroring tune_pid.py's per-trial reset.
        try:
            reset_pose(args.world, args.model, xyz=(0.0, 0.0, args.height))
        except Exception as e:
            print(f"[hover] reset_pose skipped: {e}", file=sys.stderr)
        reset_estimator(cf, "kalman")
        time.sleep(2.0)
        try:
            cf.platform.send_arming_request(True)
            time.sleep(0.5)
        except Exception as e:
            print(f"[hover] arming request skipped: {e}", file=sys.stderr)

        lg = LogConfig(name="hover", period_in_ms=20)
        for v in ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z"]:
            lg.add_variable(v, "float")

        signal.setitimer(signal.ITIMER_REAL, args.hold + 25.0)
        try:
            with MotionCommander(scf, default_height=args.height) as mc:
                time.sleep(2.5)  # settle after takeoff
                with SyncLogger(scf, lg) as logger:
                    t0 = time.time()
                    for entry in logger:
                        d = entry[1]
                        x, y, z = d["stateEstimate.x"], d["stateEstimate.y"], d["stateEstimate.z"]
                        xs.append(x); ys.append(y); zs.append(z)
                        if abs(z - args.height) > 1.0:
                            diverged = True
                            break
                        if time.time() - t0 >= args.hold:
                            break
                mc.stop()
        except _Timeout:
            diverged = True
            print("[hover] TIMEOUT — firmware likely crashed", file=sys.stderr)
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0)

    if not xs:
        print("[hover] FAIL — no samples collected (connection/firmware issue)")
        sys.exit(1)

    n = len(xs)
    radii = [math.hypot(x, y) for x, y in zip(xs, ys)]
    max_drift = max(radii)
    rms_drift = math.sqrt(sum(r * r for r in radii) / n)
    mean_z = sum(zs) / n
    z_err_mean = sum(abs(z - args.height) for z in zs) / n
    z_max_dev = max(abs(z - args.height) for z in zs)
    took_off = mean_z > 0.3 * args.height

    passed = took_off and (not diverged) and (rms_drift < RMS_MAX) and (z_err_mean < ZERR_MAX)

    print(f"[hover] gains={label}  samples={n}  took_off={took_off}  diverged={diverged}")
    print(f"[hover]   target height   = {args.height:.3f} m")
    print(f"[hover]   mean Z          = {mean_z:.4f} m   (mean |Zerr| = {z_err_mean*100:.2f} cm, max dev {z_max_dev*100:.2f} cm)")
    print(f"[hover]   horiz drift RMS = {rms_drift*100:.2f} cm   (peak {max_drift*100:.2f} cm)")
    print(f"[hover]   thresholds: RMS horiz<{RMS_MAX*100:.0f}cm, mean|Zerr|<{ZERR_MAX*100:.0f}cm")

    if not args.no_mlflow:
        try:
            import mlflow
            mlflow.set_tracking_uri("sqlite:///mlflow.db")
            mlflow.set_experiment("phase1_pid_tune")
            with mlflow.start_run(run_name=f"hover_gate_{label}"):
                mlflow.log_param("gains", gains_path)
                mlflow.log_param("hold_s", args.hold)
                mlflow.log_metric("max_horiz_drift_m", max_drift)
                mlflow.log_metric("rms_horiz_drift_m", rms_drift)
                mlflow.log_metric("z_err_mean_m", z_err_mean)
                mlflow.log_metric("z_max_dev_m", z_max_dev)
                mlflow.log_metric("took_off", 1.0 if took_off else 0.0)
                mlflow.log_metric("diverged", 1.0 if diverged else 0.0)
                mlflow.log_metric("gate_passed", 1.0 if passed else 0.0)
        except Exception as e:
            print(f"[hover] mlflow log skipped: {e}", file=sys.stderr)

    print("[hover] PASS" if passed else "[hover] FAIL")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
