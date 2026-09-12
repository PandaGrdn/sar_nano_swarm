#!/usr/bin/env python3
"""Block until /cf_<id>/odom is actually delivering ROS messages."""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO / "eval_scripts") not in sys.path:
    sys.path.insert(0, str(_REPO / "eval_scripts"))

from ros_gz_qos import subscribe_gz  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--num-drones", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=90.0)
    args = ap.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    got = {i: 0 for i in range(args.num_drones)}

    class Waiter(Node):
        def __init__(self):
            super().__init__("wait_ros_odom")
            for i in range(args.num_drones):
                subscribe_gz(
                    self,
                    Odometry,
                    f"/cf_{i}/odom",
                    lambda msg, idx=i: self._hit(idx, msg),
                )

        def _hit(self, idx: int, _msg) -> None:
            got[idx] += 1

    rclpy.init()
    node = Waiter()
    t0 = time.time()
    try:
        while time.time() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.25)
            if all(got[i] > 0 for i in got):
                print(
                    "[wait_ros_odom] ROS /cf_*/odom live "
                    + " ".join(f"cf_{i}={got[i]}" for i in got),
                    flush=True,
                )
                return 0
        missing = [f"cf_{i}" for i, n in got.items() if n == 0]
        print(
            "[wait_ros_odom] TIMEOUT — no ROS odom for " + ", ".join(missing),
            file=sys.stderr,
            flush=True,
        )
        import subprocess

        for cmd in (
            ["gz", "topic", "-l"],
            ["gz", "topic", "-i", "-t", "/cf_0/odom"],
            ["timeout", "4", "gz", "topic", "-e", "-t", "/cf_0/odom", "-n", "1"],
            ["timeout", "4", "gz", "topic", "-e", "-t", "/world/phase0_tunnel_gate/dynamic_pose/info", "-n", "1"],
        ):
            print("[wait_ros_odom] diag:", " ".join(cmd), flush=True)
            try:
                out = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
                print((out.stdout or "")[-1500:], flush=True)
                print((out.stderr or "")[-500:], flush=True)
            except Exception as exc:
                print(f"[wait_ros_odom] diag failed: {exc}", flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
