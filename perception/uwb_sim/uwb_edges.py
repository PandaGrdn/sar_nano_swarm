#!/usr/bin/env python3
"""uwb_edges.py — pack/unpack helper for UWB edge PointCloud2 messages (Phase 1 M4).

Each PointCloud2 row is one UWB edge with twelve 4-byte fields (point_step=48).
Use this module instead of hand-rolling PointField layouts.

Consumer example::

    from uwb_edges import unpack_edges, FLAG_BEARING_VALID, describe
    import rclpy
    from rclpy.node import Node

    class MyNode(Node):
        def __init__(self):
            super().__init__('my_uwb_consumer')
            self.create_subscription(
                PointCloud2, '/uwb/edges_all', self._on_edges, 10)

        def _on_edges(self, msg):
            rows = unpack_edges(msg)
            for row in rows:
                if row['flags'] & FLAG_BEARING_VALID:
                    print(row['peer_id'], row['range_m'], row['azimuth_rad'])
"""
from __future__ import annotations

import math
from typing import Iterable, List, Union

import numpy as np

try:
    from sensor_msgs.msg import PointCloud2, PointField
except ImportError:  # selftest / no ROS
    PointCloud2 = None  # type: ignore
    PointField = None  # type: ignore

# Flags bitfield (plan §3.4)
FLAG_RANGE_VALID = 0x01
FLAG_BEARING_VALID = 0x02
FLAG_LOS = 0x04
FLAG_IN_AOA_CONE = 0x08
FLAG_PEER_IS_SURVEYED = 0x10
FLAG_PEER_IS_STATIC = 0x20

EDGE_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("observer_id", "<u4"),
        ("peer_id", "<u4"),
        ("range_m", "<f4"),
        ("azimuth_rad", "<f4"),
        ("elevation_rad", "<f4"),
        ("sigma_range_m", "<f4"),
        ("sigma_az_rad", "<f4"),
        ("sigma_el_rad", "<f4"),
        ("flags", "<u4"),
    ]
)

assert EDGE_DTYPE.itemsize == 48

_POINT_FIELDS = [
    ("x", PointField.FLOAT32 if PointField else 7, 0),
    ("y", PointField.FLOAT32 if PointField else 7, 4),
    ("z", PointField.FLOAT32 if PointField else 7, 8),
    ("observer_id", PointField.UINT32 if PointField else 6, 12),
    ("peer_id", PointField.UINT32 if PointField else 6, 16),
    ("range_m", PointField.FLOAT32 if PointField else 7, 20),
    ("azimuth_rad", PointField.FLOAT32 if PointField else 7, 24),
    ("elevation_rad", PointField.FLOAT32 if PointField else 7, 28),
    ("sigma_range_m", PointField.FLOAT32 if PointField else 7, 32),
    ("sigma_az_rad", PointField.FLOAT32 if PointField else 7, 36),
    ("sigma_el_rad", PointField.FLOAT32 if PointField else 7, 40),
    ("flags", PointField.UINT32 if PointField else 6, 44),
]

if PointField is not None:
    for name, _, offset in _POINT_FIELDS:
        assert EDGE_DTYPE.fields[name][1] == offset


def _stamp_to_sec_nsec(stamp) -> tuple:
    if hasattr(stamp, "sec"):
        return int(stamp.sec), int(stamp.nanosec)
    if isinstance(stamp, (tuple, list)) and len(stamp) == 2:
        return int(stamp[0]), int(stamp[1])
    return 0, 0


def pack_edges(edges: Union[Iterable[dict], np.ndarray], stamp, frame_id: str):
    """Pack edge rows into a sensor_msgs/PointCloud2 message."""
    if PointCloud2 is None:
        raise ImportError("sensor_msgs required for pack_edges")

    if isinstance(edges, np.ndarray):
        arr = edges.astype(EDGE_DTYPE, copy=False)
    else:
        rows = list(edges)
        if not rows:
            arr = np.zeros(0, dtype=EDGE_DTYPE)
        else:
            arr = np.zeros(len(rows), dtype=EDGE_DTYPE)
            for i, e in enumerate(rows):
                for name in EDGE_DTYPE.names:
                    arr[i][name] = e[name]

    msg = PointCloud2()
    sec, nsec = _stamp_to_sec_nsec(stamp)
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nsec
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(arr.size)
    msg.is_bigendian = False
    msg.is_dense = False
    msg.point_step = 48
    msg.row_step = 48 * msg.width
    msg.fields = []
    for name, dtype, offset in _POINT_FIELDS:
        pf = PointField()
        pf.name = name
        pf.offset = offset
        pf.datatype = dtype
        pf.count = 1
        msg.fields.append(pf)
    msg.data = arr.tobytes()
    return msg


def unpack_edges(msg) -> np.ndarray:
    """Unpack a PointCloud2 message into a structured numpy array."""
    n = msg.width * msg.height
    if n == 0:
        return np.zeros(0, dtype=EDGE_DTYPE)
    return np.frombuffer(msg.data, dtype=EDGE_DTYPE, count=n).copy()


def describe(msg) -> str:
    """Pretty-print edge rows for debugging."""
    rows = unpack_edges(msg)
    lines = [f"UWB edges ({len(rows)} rows):"]
    flag_names = {
        FLAG_RANGE_VALID: "RANGE",
        FLAG_BEARING_VALID: "BEARING",
        FLAG_LOS: "LOS",
        FLAG_IN_AOA_CONE: "AOA",
        FLAG_PEER_IS_SURVEYED: "SURVEYED",
        FLAG_PEER_IS_STATIC: "STATIC",
    }
    for i, r in enumerate(rows):
        flags = int(r["flags"])
        fstr = "|".join(n for bit, n in flag_names.items() if flags & bit) or "none"
        lines.append(
            f"  [{i}] obs={r['observer_id']} peer={r['peer_id']} "
            f"r={r['range_m']:.3f} az={_fmt(r['azimuth_rad'])} "
            f"el={_fmt(r['elevation_rad'])} flags={fstr}"
        )
    return "\n".join(lines)


def _fmt(v: float) -> str:
    if isinstance(v, (float, np.floating)) and (math.isnan(v) or math.isinf(v)):
        return "NaN"
    return f"{float(v):.3f}"
