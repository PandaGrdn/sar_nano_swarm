"""QoS helpers for ros_gz_bridge / in-process rclcpp publishers.

Gazebo Harmonic's parameter_bridge and radarays `QoS(10)` publish RELIABLE.
Several nodes historically used `qos_profile_sensor_data` (BEST_EFFORT). Those
profiles do not match, so /cf_*/odom, IMU, and /radar/points can look "up"
in `ros2 topic list` while every subscriber callback stays empty.
"""
from __future__ import annotations


def gz_bridge_qos_profiles():
    from rclpy.qos import (
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )

    reliable = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )
    return (qos_profile_sensor_data, reliable)


def subscribe_gz(node, msg_type, topic, callback):
    """Subscribe twice so either bridge reliability matches."""
    return [
        node.create_subscription(msg_type, topic, callback, qos)
        for qos in gz_bridge_qos_profiles()
    ]
