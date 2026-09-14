#!/usr/bin/env python3
"""radar_noise_node.py — simulator-side radar noise layer (one process, all drones).

For each drone i: subscribes /cf_<i>/radar/points_ideal (radarays_gz2 ideal
ray-cast cloud) and republishes /cf_<i>/radar/points as a realistic TI
IWR6843AOP detection cloud (math in radar_noise_model.py, parameters and
citations in configs/sensors/radar_noise.yaml). Header (sim stamp, frame_id)
is preserved exactly: rio_bridge takes dt from consecutive radar stamps.

⚠ AGENTS.md §1 Tier A: this node only transforms plugin output. It never reads
truth topics (no odom), and the estimator never reads its config.

Usage (setup_env.sh sourced):
    python3 -u perception/radar_sim/radar_noise_node.py [--config configs/sensors/radar_noise.yaml] --num-drones N
    python3 perception/radar_sim/radar_noise_node.py --selftest
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import yaml

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT / "perception" / "radar_sim") not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT / "perception" / "radar_sim"))

from radar_noise_model import RadarNoiseModel  # noqa: E402

FIELDS = ("x", "y", "z", "intensity", "doppler")


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def resolve_config_path(path: str) -> str:
    if os.path.isabs(path):
        return path
    root = os.environ.get("SAR_NANO_SWARM_ROOT", str(_REPO_ROOT))
    return os.path.join(root, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/sensors/radar_noise.yaml")
    parser.add_argument("--num-drones", type=int, default=1)
    parser.add_argument("--seed", type=int, default=None, help="override config seed (drone i uses seed + i)")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    cfg_path = resolve_config_path(args.config)

    if args.selftest:
        from radar_noise_model import run_selftest

        cfg = load_config(cfg_path) if os.path.isfile(cfg_path) else None
        sys.exit(run_selftest(cfg))

    cfg = load_config(cfg_path)
    if args.seed is not None:
        cfg["seed"] = args.seed
    base_seed = int(cfg.get("seed", 0))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2

    out_fields = [
        PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
        for k, n in enumerate(FIELDS)
    ]

    class RadarNoiseNode(Node):
        def __init__(self):
            super().__init__("radar_noise_sim")
            self.num_drones = args.num_drones
            in_fmt = cfg.get("input_topic_fmt", "/cf_{i}/radar/points_ideal")
            out_fmt = cfg.get("output_topic_fmt", "/cf_{i}/radar/points")
            self.models = {}
            self.pubs = {}
            self.stats = {}
            for i in range(self.num_drones):
                self.models[i] = RadarNoiseModel.from_config(cfg, seed=base_seed + i)
                self.pubs[i] = self.create_publisher(PointCloud2, out_fmt.format(i=i), qos_profile_sensor_data)
                self.create_subscription(
                    PointCloud2, in_fmt.format(i=i),
                    lambda msg, cf_id=i: self._on_cloud(cf_id, msg),
                    qos_profile_sensor_data,
                )
                self.stats[i] = self._zero_stats()
            self._t_last = time.monotonic()
            self.create_timer(float(cfg.get("diag_period_s", 10.0)), self._on_diag)
            m0 = self.models[0] if self.models else None
            self.get_logger().info(
                f"radar noise node config={cfg_path} seed={base_seed}(+i) N_drones={self.num_drones} "
                f"in={in_fmt} out={out_fmt} enable={cfg.get('enable')} "
                + (f"sigma_residual={m0.sigma_residual:.5f} m/s" if m0 else "")
            )

        @staticmethod
        def _zero_stats():
            return {"msgs": 0, "n_in": 0, "n_out": 0, "n_outliers": 0, "fail": 0}

        def _on_cloud(self, cf_id: int, msg):
            try:
                st = point_cloud2.read_points(msg, field_names=FIELDS, skip_nans=False)
                pts = np.stack([np.asarray(st[n], dtype=np.float64) for n in FIELDS], axis=1) \
                    if len(st) else np.zeros((0, 5))
            except Exception as exc:  # field layout mismatch
                self.stats[cf_id]["fail"] += 1
                self.get_logger().error(f"cf_{cf_id}: cannot parse radar cloud ({exc})",
                                        throttle_duration_sec=5.0)
                return
            out, diag = self.models[cf_id].apply(pts)
            data = np.ascontiguousarray(out, dtype="<f4")
            cloud = PointCloud2()
            cloud.header = msg.header
            cloud.height = 1
            cloud.width = int(data.shape[0])
            cloud.fields = out_fields
            cloud.is_bigendian = False
            cloud.point_step = 4 * len(FIELDS)
            cloud.row_step = cloud.point_step * cloud.width
            cloud.data = data.tobytes()
            cloud.is_dense = True
            self.pubs[cf_id].publish(cloud)
            s = self.stats[cf_id]
            s["msgs"] += 1
            s["n_in"] += diag["n_in"]
            s["n_out"] += diag["n_out"]
            s["n_outliers"] += diag["n_outliers"]

        def _on_diag(self):
            now = time.monotonic()
            dt = max(now - self._t_last, 1e-6)
            self._t_last = now
            for i in range(self.num_drones):
                s = self.stats[i]
                m = max(s["msgs"], 1)
                self.get_logger().info(
                    f"cf_{i}: {s['msgs'] / dt:.1f} Hz, pts/scan in {s['n_in'] / m:.0f} "
                    f"out {s['n_out'] / m:.0f}, outliers/scan {s['n_outliers'] / m:.1f}"
                    + (f", parse failures {s['fail']}" if s["fail"] else "")
                )
                self.stats[i] = self._zero_stats()

    rclpy.init()
    node = RadarNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
