#!/usr/bin/env python3
"""Republish Gazebo Pose_V (world dynamic_pose) as /cf_<id>/odom.

OdometryPublisher can advertise /cf_*/odom and never emit while the sim is
loaded. SceneBroadcaster pose/info still ticks; this keeps UWB and ATE alive.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO / "eval_scripts") not in sys.path:
    sys.path.insert(0, str(_REPO / "eval_scripts"))

from ros_gz_qos import subscribe_gz  # noqa: E402


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--num-drones", type=int, default=3)
    args = ap.parse_args()

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage

    topic = f"/world/{args.world}/dynamic_pose/info"
    names = {f"crazyflie_{i}": i for i in range(args.num_drones)}

    class PoseOdom(Node):
        def __init__(self):
            super().__init__("gz_pose_to_odom")
            self.pubs = {
                i: self.create_publisher(
                    Odometry, f"/cf_{i}/odom", qos_profile_sensor_data
                )
                for i in range(args.num_drones)
            }
            subscribe_gz(self, TFMessage, topic, self._on_poses)
            self.get_logger().info(f"Pose_V {topic} → /cf_*/odom")

        def _on_poses(self, msg: TFMessage) -> None:
            for t in msg.transforms:
                self._one(t)

        def _one(self, t: TransformStamped) -> None:
            name = t.child_frame_id or t.header.frame_id
            name = name.split("/")[-1]
            if name not in names:
                return
            i = names[name]
            p = t.transform.translation
            q = t.transform.rotation
            odom = Odometry()
            odom.header = t.header
            odom.header.frame_id = "world"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = p.x
            odom.pose.pose.position.y = p.y
            odom.pose.pose.position.z = p.z
            odom.pose.pose.orientation = q
            _ = yaw_from_quat(q.x, q.y, q.z, q.w)
            self.pubs[i].publish(odom)

    rclpy.init()
    node = PoseOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
