#!/usr/bin/env python3
"""push_pid_gains.py — connect to a running SITL instance and push a saved
PID gain set (e.g. configs/airframe/pid_gains_loaded.yaml, the output of
tune_pid.py) via the runtime CFLib PARAM interface. No firmware rebuild.

Useful for: re-verifying a tuned gain set after a hover test, or as the
last manual step before eventually flashing the same numbers to real
hardware (roadmap Ph1.2: "Save the gain set — this is what you'll flash to
real hardware later").

Usage:
    python3 eval_scripts/push_pid_gains.py \
        --gains configs/airframe/pid_gains_loaded.yaml \
        --uri udp://127.0.0.1:19850
"""
import argparse
import time

from pid_gains import load_gains, apply_gains


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gains", default="configs/airframe/pid_gains_loaded.yaml")
    parser.add_argument("--uri", default="udp://127.0.0.1:19850")
    args = parser.parse_args()

    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    gains = load_gains(args.gains)
    print(f"[push_pid_gains] Loaded {sum(len(f) for f in gains.values())} params from {args.gains}")

    cflib.crtp.init_drivers()
    print(f"[push_pid_gains] Connecting to {args.uri} …")
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        apply_gains(scf.cf, gains)
        time.sleep(0.5)  # let the last few param-set packets land
        print("[push_pid_gains] Done — gains applied to the running SITL instance.")


if __name__ == "__main__":
    main()
