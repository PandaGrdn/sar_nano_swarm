#!/usr/bin/env python3
"""uwb_node.py — inter-drone UWB PDoA sensor node (Phase 1 M4).

Single rclpy process subscribes to every drone's /cf_<id>/odom and publishes
/cf_<id>/uwb/edges plus optional /uwb/edges_all and /uwb/edges_truth.

⚠ AGENTS.md §1 Tier A: /uwb/edges_truth is a SIM ORACLE for gates only.
The Phase-2 estimator / RIO front-end must NEVER subscribe to it.

Usage (setup_env.sh sourced):
    python3 -u perception/uwb_sim/uwb_node.py [--config configs/sensors/uwb_pdoa.yaml]
    python3 perception/uwb_sim/uwb_node.py --selftest
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import yaml

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT / "perception" / "uwb_sim") not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT / "perception" / "uwb_sim"))

from uwb_edges import pack_edges  # noqa: E402
from uwb_model import (  # noqa: E402
    C_LIGHT,
    DeviceState,
    UwbModel,
    quat_to_rot_matrix,
    yaw_to_rot_matrix,
)


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def resolve_config_path(path: str) -> str:
    if os.path.isabs(path):
        return path
    root = os.environ.get("SAR_NANO_SWARM_ROOT", str(_REPO_ROOT))
    return os.path.join(root, path)


def odom_stamp_sec(msg) -> float:
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9


def build_static_devices(cfg: dict) -> Dict[int, DeviceState]:
    devices: Dict[int, DeviceState] = {}
    for peer in cfg.get("static_peers", []):
        did = int(peer["id"])
        pos = np.array(peer["position_xyz_m"], dtype=np.float64)
        yaw = math.radians(float(peer.get("yaw_deg", 0.0)))
        devices[did] = DeviceState(
            device_id=did,
            position=pos,
            R_bw=yaw_to_rot_matrix(yaw),
            is_static=True,
            peer_type=str(peer.get("type", "landed")),
        )
    return devices


def topic_for_observer(device_id: int, num_drones: int) -> str:
    if device_id < num_drones:
        return f"/cf_{device_id}/uwb/edges"
    # ROS 2 topic tokens cannot start with a digit (plan §3.3 id embedded in name).
    return f"/uwb/peer_{device_id}/edges"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/sensors/uwb_pdoa.yaml")
    parser.add_argument("--num-drones", type=int, default=2)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--no-truth", action="store_true")
    parser.add_argument("--no-mlflow", action="store_true")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        from uwb_model import run_selftest

        sys.exit(run_selftest())

    cfg_path = resolve_config_path(args.config)
    cfg = load_config(cfg_path)
    if args.seed is not None:
        cfg["seed"] = args.seed
    cfg["_num_drones"] = args.num_drones
    root = os.environ.get("SAR_NANO_SWARM_ROOT", str(_REPO_ROOT))
    cfg["_swarm_root"] = root

    publish_truth = cfg.get("publish_ground_truth", True) and not args.no_truth
    publish_agg = cfg.get("publish_aggregate", True)

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    class UwbNode(Node):
        def __init__(self):
            super().__init__("uwb_sim")
            self.cfg = cfg
            self.num_drones = args.num_drones
            self.model = UwbModel.from_config(cfg, seed=cfg.get("seed"))
            self.static_devices = build_static_devices(cfg)
            self.devices: Dict[int, DeviceState] = dict(self.static_devices)
            self.odom_stamp: Dict[int, Tuple[int, int]] = {}
            self.odom_recv_wall: Dict[int, float] = {}
            self._stale_warned: set = set()
            self._tick_times: list = []
            self._last_n_pairs = -1

            lam = C_LIGHT / float(cfg["channel_centre_hz"])
            spacing = float(cfg["antenna_spacing_m"])
            if spacing > lam / 2.0 + 1e-9:
                self.get_logger().warn(
                    f"antenna_spacing_m={spacing} > lambda/2={lam/2:.4f} — phase ambiguous (plan §2.3.1)"
                )

            if cfg.get("los_model") == "always_los":
                self.get_logger().warn("los_model=always_los — NLOS disabled")

            from sensor_msgs.msg import PointCloud2

            qos = qos_profile_sensor_data
            for i in range(self.num_drones):
                self.create_subscription(
                    Odometry,
                    f"/cf_{i}/odom",
                    lambda msg, cf_id=i: self._on_odom(cf_id, msg),
                    qos,
                )

            self.pubs: Dict[int, object] = {}
            for i in range(self.num_drones):
                t = topic_for_observer(i, self.num_drones)
                self.pubs[i] = self.create_publisher(PointCloud2, t, qos)
            for sid in self.static_devices:
                t = topic_for_observer(sid, self.num_drones)
                self.pubs[sid] = self.create_publisher(PointCloud2, t, qos)

            if publish_agg:
                self.pub_all = self.create_publisher(PointCloud2, "/uwb/edges_all", qos)
            if publish_truth:
                self.pub_truth = self.create_publisher(PointCloud2, "/uwb/edges_truth", qos)
                self.get_logger().warn(
                    "publish_ground_truth=True: /uwb/edges_truth is a SIM ORACLE — "
                    "AGENTS.md §1 Tier A: the estimator must NEVER subscribe to it."
                )

            period = 1.0 / float(cfg["scheduler_tick_hz"])
            self.create_timer(period, self._on_timer)

            self._log_startup(lam)

            if not args.no_mlflow:
                self._try_mlflow_log_params()

        def _log_startup(self, lam: float):
            peers = cfg.get("static_peers", [])
            self.get_logger().info(
                f"UWB node config={cfg_path} seed={cfg.get('seed')} "
                f"N_drones={self.num_drones} static_peers={peers} "
                f"lambda_m={lam:.4f} spacing/lambda/2="
                f"{float(cfg['antenna_spacing_m']) / (lam / 2):.3f} "
                f"boresight={cfg['boresight_axis']} aoa_fov_deg={cfg['aoa_fov_deg']} "
                f"n_pairs={self.model.n_pairs} effective_rate={self.model.effective_rate:.3f} Hz "
                f"antenna_delay_bias={self.model.antenna_delay_bias}"
            )

        def _try_mlflow_log_params(self):
            try:
                import mlflow

                mlflow.set_tracking_uri("sqlite:///mlflow.db")
                mlflow.set_experiment(cfg.get("mlflow_experiment", "phase1_uwb_pdoa"))
                with mlflow.start_run(run_name="uwb_node_startup"):
                    mlflow.log_param("seed", cfg.get("seed"))
                    mlflow.log_param("num_drones", self.num_drones)
                    mlflow.log_param("effective_rate", self.model.effective_rate)
                    mlflow.log_param("n_pairs", self.model.n_pairs)
            except Exception as exc:
                self.get_logger().warn(f"MLflow startup log skipped: {exc}")

        def _on_odom(self, cf_id: int, msg: Odometry):
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            q = (ori.x, ori.y, ori.z, ori.w)
            R = quat_to_rot_matrix(q)
            p = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
            stale_s = float(cfg.get("odom_stale_s", 0.5))
            now = time.time()
            self.odom_recv_wall[cf_id] = now
            self.odom_stamp[cf_id] = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if cf_id in self.devices and self.devices[cf_id].is_static:
                return
            was_inactive = cf_id in self.devices and not self.devices[cf_id].active
            self.devices[cf_id] = DeviceState(
                device_id=cf_id,
                position=p,
                R_bw=R,
                is_static=False,
                peer_type="drone",
                active=True,
            )
            if was_inactive and cf_id in self._stale_warned:
                self._stale_warned.discard(cf_id)

        def _refresh_active(self):
            stale_s = float(cfg.get("odom_stale_s", 0.5))
            now = time.time()
            for i in range(self.num_drones):
                last = self.odom_recv_wall.get(i)
                if last is None or (now - last) > stale_s:
                    if i in self.devices:
                        self.devices[i].active = False
                    if i not in self._stale_warned:
                        self.get_logger().warn(f"Drone {i} odom stale — removed from mesh")
                        self._stale_warned.add(i)

        def _newest_stamp(self) -> Tuple[int, int]:
            if not self.odom_stamp:
                now = self.get_clock().now().to_msg()
                return now.sec, now.nanosec
            return max(self.odom_stamp.values())

        def _on_timer(self):
            t0 = time.perf_counter()
            self._refresh_active()
            per_obs, _ = self.model.tick(self.devices)
            stamp = self._newest_stamp()

            if self.model.n_pairs != self._last_n_pairs:
                self.get_logger().info(
                    f"Scheduled pairs={self.model.n_pairs} effective_rate={self.model.effective_rate:.3f} Hz"
                )
                self._last_n_pairs = self.model.n_pairs

            all_rows = []
            for obs_id, rows in per_obs.items():
                frame = (
                    f"cf_{obs_id}/base_link"
                    if obs_id < self.num_drones
                    else f"uwb_{obs_id}/base_link"
                )
                msg = pack_edges(rows, stamp, frame)
                if obs_id in self.pubs:
                    self.pubs[obs_id].publish(msg)
                all_rows.extend(rows)

            # Heartbeat: publish empty messages for observers with no edges
            for obs_id in list(self.devices.keys()):
                if self.devices[obs_id].active and obs_id not in per_obs:
                    frame = (
                        f"cf_{obs_id}/base_link"
                        if obs_id < self.num_drones
                        else f"uwb_{obs_id}/base_link"
                    )
                    if obs_id in self.pubs:
                        self.pubs[obs_id].publish(pack_edges([], stamp, frame))

            if publish_agg:
                self.pub_all.publish(pack_edges(all_rows, stamp, "world"))

            if publish_truth:
                truth = self.model.compute_truth_edges(self.devices)
                self.pub_truth.publish(pack_edges(truth, stamp, "world"))

            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            self._tick_times.append(elapsed_ms)
            if len(self._tick_times) > 100:
                self._tick_times.pop(0)

    rclpy.init()
    node = UwbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
