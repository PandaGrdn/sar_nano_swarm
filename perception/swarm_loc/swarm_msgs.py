#!/usr/bin/env python3
"""swarm_msgs.py — PointCloud2 pack/unpack for swarm-loc wires (P2-5).

No custom .msg files. Mirror uwb_edges.py: module-level dtype, PointField
list, pack_*/unpack_*, assert itemsize.

Topics:
  /cf_<id>/swarm_loc/broadcast          STATE_DTYPE  (1 row)
  /cf_<id>/swarm_loc/bearing_rebroadcast BEARING_DTYPE (0..N rows)
  /cf_<id>/rio/delta                    RIO_DTYPE    (1 row)
  /cf_<id>/swarm_loc/estimate           STATE_DTYPE  (1 row, at rate_hz)

Usage:
    python3 perception/swarm_loc/swarm_msgs.py --selftest
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Iterable, List, Optional, Tuple, Union

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[2]
for _p in ("perception/swarm_loc", "perception/uwb_sim"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

try:
    from sensor_msgs.msg import PointCloud2, PointField
except ImportError:
    PointCloud2 = None  # type: ignore
    PointField = None  # type: ignore

_PF_F32 = PointField.FLOAT32 if PointField else 7
_PF_F64 = PointField.FLOAT64 if PointField else 8
_PF_U32 = PointField.UINT32 if PointField else 6

# ---------------------------------------------------------------------------
# dtypes
# ---------------------------------------------------------------------------

STATE_DTYPE = np.dtype(
    [
        ("stamp", "<f8"),
        ("drone_id", "<u4"),
        ("p_x", "<f4"),
        ("p_y", "<f4"),
        ("p_z", "<f4"),
        ("v_x", "<f4"),
        ("v_y", "<f4"),
        ("v_z", "<f4"),
        ("psi", "<f4"),
        ("cov_p_0", "<f4"),
        ("cov_p_1", "<f4"),
        ("cov_p_2", "<f4"),
        ("cov_p_3", "<f4"),
        ("cov_p_4", "<f4"),
        ("cov_p_5", "<f4"),
        ("cov_psi", "<f4"),
        ("roll", "<f4"),
        ("pitch", "<f4"),
        ("seq", "<u4"),
        ("status", "<u4"),
        ("n_bearing_edges", "<u4"),
    ]
)
assert STATE_DTYPE.itemsize == 88

BEARING_DTYPE = np.dtype(
    [
        ("stamp", "<f8"),
        ("observer_id", "<u4"),
        ("peer_id", "<u4"),
        ("range_m", "<f4"),
        ("azimuth_rad", "<f4"),
        ("elevation_rad", "<f4"),
        ("sigma_range_m", "<f4"),
        ("sigma_az_rad", "<f4"),
        ("sigma_el_rad", "<f4"),
        ("psi_observer", "<f4"),
        ("roll_observer", "<f4"),
        ("pitch_observer", "<f4"),
    ]
)
assert BEARING_DTYPE.itemsize == 52

RIO_DTYPE = np.dtype(
    [
        ("stamp", "<f8"),
        ("dt", "<f4"),
        ("dp_x", "<f4"),
        ("dp_y", "<f4"),
        ("dp_z", "<f4"),
        ("dpsi", "<f4"),
        ("roll", "<f4"),
        ("pitch", "<f4"),
        ("cov_0", "<f4"),
        ("cov_1", "<f4"),
        ("cov_2", "<f4"),
        ("cov_3", "<f4"),
        ("cov_4", "<f4"),
        ("cov_5", "<f4"),
        ("cov_6", "<f4"),
        ("cov_7", "<f4"),
        ("cov_8", "<f4"),
        ("cov_9", "<f4"),
        ("cov_10", "<f4"),
        ("cov_11", "<f4"),
        ("cov_12", "<f4"),
        ("cov_13", "<f4"),
        ("cov_14", "<f4"),
        ("valid", "<u4"),
    ]
)
assert RIO_DTYPE.itemsize == 100


def _fields_for(dtype: np.dtype, type_map: dict) -> list:
    out = []
    for name in dtype.names:
        offset = int(dtype.fields[name][1])
        np_t = dtype.fields[name][0]
        out.append((name, type_map[np_t.str], offset))
    return out


_TYPE_MAP = {"<f8": _PF_F64, "<f4": _PF_F32, "<u4": _PF_U32}
STATE_FIELDS = _fields_for(STATE_DTYPE, _TYPE_MAP)
BEARING_FIELDS = _fields_for(BEARING_DTYPE, _TYPE_MAP)
RIO_FIELDS = _fields_for(RIO_DTYPE, _TYPE_MAP)

if PointField is not None:
    for name, _, offset in STATE_FIELDS:
        assert STATE_DTYPE.fields[name][1] == offset
    for name, _, offset in BEARING_FIELDS:
        assert BEARING_DTYPE.fields[name][1] == offset
    for name, _, offset in RIO_FIELDS:
        assert RIO_DTYPE.fields[name][1] == offset


# ---------------------------------------------------------------------------
# triangles
# ---------------------------------------------------------------------------

def triu_n(n: int) -> List[Tuple[int, int]]:
    return [(i, j) for i in range(n) for j in range(i, n)]


def pack_triu(M: np.ndarray) -> np.ndarray:
    n = M.shape[0]
    return np.array([M[i, j] for i, j in triu_n(n)], dtype=np.float64)


def unpack_triu(vals, n: int) -> np.ndarray:
    M = np.zeros((n, n), dtype=np.float64)
    for k, (i, j) in enumerate(triu_n(n)):
        M[i, j] = M[j, i] = float(vals[k])
    return M


# ---------------------------------------------------------------------------
# pack / unpack
# ---------------------------------------------------------------------------

def _stamp_to_sec_nsec(stamp) -> tuple:
    if hasattr(stamp, "sec"):
        return int(stamp.sec), int(stamp.nanosec)
    if isinstance(stamp, (tuple, list)) and len(stamp) == 2:
        return int(stamp[0]), int(stamp[1])
    if isinstance(stamp, (float, np.floating)):
        sec = int(math.floor(float(stamp)))
        nsec = int(round((float(stamp) - sec) * 1e9))
        if nsec >= 1_000_000_000:
            sec += 1
            nsec -= 1_000_000_000
        return sec, nsec
    return 0, 0


def _to_array(rows, dtype: np.dtype) -> np.ndarray:
    if isinstance(rows, np.ndarray):
        return np.asarray(rows, dtype=dtype)
    rows = list(rows)
    if not rows:
        return np.zeros(0, dtype=dtype)
    arr = np.zeros(len(rows), dtype=dtype)
    for i, e in enumerate(rows):
        if isinstance(e, dict):
            for name in dtype.names:
                if name in e:
                    arr[i][name] = e[name]
        else:
            arr[i] = e
    return arr


def _pack(rows, stamp, frame_id: str, dtype: np.dtype, fields: list):
    if PointCloud2 is None:
        raise ImportError("sensor_msgs required for pack")
    arr = _to_array(rows, dtype)
    msg = PointCloud2()
    sec, nsec = _stamp_to_sec_nsec(stamp)
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nsec
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(arr.size)
    msg.is_bigendian = False
    msg.is_dense = False
    msg.point_step = int(dtype.itemsize)
    msg.row_step = int(dtype.itemsize) * msg.width
    msg.fields = []
    for name, pf_type, offset in fields:
        pf = PointField()
        pf.name = name
        pf.offset = offset
        pf.datatype = pf_type
        pf.count = 1
        msg.fields.append(pf)
    msg.data = arr.tobytes()
    return msg


def _unpack(msg, dtype: np.dtype) -> np.ndarray:
    n = int(msg.width) * int(msg.height)
    if n == 0:
        return np.zeros(0, dtype=dtype)
    return np.frombuffer(bytes(msg.data), dtype=dtype, count=n).copy()


def pack_state(rows, stamp, frame_id: str = "world"):
    return _pack(rows, stamp, frame_id, STATE_DTYPE, STATE_FIELDS)


def unpack_state(msg) -> np.ndarray:
    return _unpack(msg, STATE_DTYPE)


def pack_bearing(rows, stamp, frame_id: str = "world"):
    return _pack(rows, stamp, frame_id, BEARING_DTYPE, BEARING_FIELDS)


def unpack_bearing(msg) -> np.ndarray:
    return _unpack(msg, BEARING_DTYPE)


def pack_rio(rows, stamp, frame_id: str = "body"):
    return _pack(rows, stamp, frame_id, RIO_DTYPE, RIO_FIELDS)


def unpack_rio(msg) -> np.ndarray:
    return _unpack(msg, RIO_DTYPE)


def neighbor_P_from_state_row(row, vel_sigma: float = 5.0) -> np.ndarray:
    """9×9 cov from a broadcast row (position triangle + yaw; rest large)."""
    from state import IDX_PSI, N_STATE

    P = np.eye(N_STATE, dtype=np.float64) * (vel_sigma**2)
    P[0:3, 0:3] = unpack_triu(
        [row["cov_p_0"], row["cov_p_1"], row["cov_p_2"], row["cov_p_3"], row["cov_p_4"], row["cov_p_5"]],
        3,
    )
    P[IDX_PSI, IDX_PSI] = float(row["cov_psi"])
    return P


def state_row_from_filter(st, seq: int, n_bearing_edges: int) -> dict:
    tri = pack_triu(st.P[0:3, 0:3])
    return {
        "stamp": float(st.stamp),
        "drone_id": int(st.drone_id),
        "p_x": float(st.p[0]),
        "p_y": float(st.p[1]),
        "p_z": float(st.p[2]),
        "v_x": float(st.v[0]),
        "v_y": float(st.v[1]),
        "v_z": float(st.v[2]),
        "psi": float(st.psi),
        "cov_p_0": float(tri[0]),
        "cov_p_1": float(tri[1]),
        "cov_p_2": float(tri[2]),
        "cov_p_3": float(tri[3]),
        "cov_p_4": float(tri[4]),
        "cov_p_5": float(tri[5]),
        "cov_psi": float(st.P[6, 6]),
        "roll": float(st.roll),
        "pitch": float(st.pitch),
        "seq": int(seq),
        "status": int(st.status),
        "n_bearing_edges": int(n_bearing_edges),
    }


def rio_row_from_delta(delta) -> dict:
    tri = pack_triu(np.asarray(delta.cov, dtype=np.float64))
    return {
        "stamp": float(delta.stamp),
        "dt": float(delta.dt),
        "dp_x": float(delta.delta_p_body[0]),
        "dp_y": float(delta.delta_p_body[1]),
        "dp_z": float(delta.delta_p_body[2]),
        "dpsi": float(delta.delta_psi),
        "roll": float(delta.roll),
        "pitch": float(delta.pitch),
        **{f"cov_{k}": float(tri[k]) for k in range(15)},
        "valid": 1 if delta.valid else 0,
    }


def rio_delta_from_row(row):
    from rio_stub import RioDelta

    cov = unpack_triu([row[f"cov_{k}"] for k in range(15)], 5)
    return RioDelta(
        stamp=float(row["stamp"]),
        dt=float(row["dt"]),
        delta_p_body=np.array([row["dp_x"], row["dp_y"], row["dp_z"]], dtype=np.float64),
        delta_psi=float(row["dpsi"]),
        roll=float(row["roll"]),
        pitch=float(row["pitch"]),
        cov=cov,
        valid=bool(int(row["valid"])),
    )


# ---------------------------------------------------------------------------
# D17 — latency + loss (wall time). Applied at the sender.
# ---------------------------------------------------------------------------

class DelayedDropQueue:
    """Hold items for latency_s; drop each push with probability packet_loss."""

    def __init__(self, latency_s: float, packet_loss: float, rng: np.random.Generator):
        self.latency_s = float(max(latency_s, 0.0))
        self.packet_loss = float(np.clip(packet_loss, 0.0, 1.0))
        self.rng = rng
        self._q: List[Tuple[float, object]] = []

    def push(self, now_wall: float, item) -> bool:
        """Return False if dropped."""
        if self.rng.random() < self.packet_loss:
            return False
        self._q.append((float(now_wall) + self.latency_s, item))
        return True

    def pop_ready(self, now_wall: float) -> list:
        ready, keep = [], []
        t = float(now_wall)
        for rel, item in self._q:
            if rel <= t:
                ready.append(item)
            else:
                keep.append((rel, item))
        self._q = keep
        return ready


# ---------------------------------------------------------------------------
# selftest
# ---------------------------------------------------------------------------

class _FakeCloud:
    def __init__(self):
        self.width = 0
        self.height = 1
        self.data = b""


def run_selftest() -> int:
    ok = True
    n_pass = 0
    n_fail = 0

    def check(name: str, cond: bool, detail: str = ""):
        nonlocal ok, n_pass, n_fail
        if cond:
            n_pass += 1
            print(f"[selftest] PASS {name}")
        else:
            ok = False
            n_fail += 1
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))

    check("1 STATE itemsize 88", STATE_DTYPE.itemsize == 88)
    check("1b BEARING itemsize 52", BEARING_DTYPE.itemsize == 52)
    check("1c RIO itemsize 100", RIO_DTYPE.itemsize == 100)

    M = np.array([[1.0, 0.2, 0.3], [0.2, 4.0, 0.5], [0.3, 0.5, 9.0]])
    check("2 triu roundtrip", np.allclose(unpack_triu(pack_triu(M), 3), M))
    M5 = np.eye(5) * 2.0 + 0.1
    M5 = 0.5 * (M5 + M5.T)
    check("2b 5x5 triu", np.allclose(unpack_triu(pack_triu(M5), 5), M5, atol=1e-6))

    row = {
        "stamp": 1.5,
        "drone_id": 2,
        "p_x": 1.0,
        "p_y": 2.0,
        "p_z": 0.5,
        "v_x": 0.1,
        "v_y": 0.0,
        "v_z": 0.0,
        "psi": 0.2,
        "cov_p_0": 0.01,
        "cov_p_1": 0.0,
        "cov_p_2": 0.0,
        "cov_p_3": 0.02,
        "cov_p_4": 0.0,
        "cov_p_5": 0.03,
        "cov_psi": 0.05,
        "roll": 0.0,
        "pitch": 0.0,
        "seq": 7,
        "status": 0,
        "n_bearing_edges": 3,
    }
    arr = _to_array([row], STATE_DTYPE)
    fake = _FakeCloud()
    fake.width = 1
    fake.data = arr.tobytes()
    back = unpack_state(fake)
    check("3 unpack state row", len(back) == 1 and int(back[0]["drone_id"]) == 2)
    check("3b stamp survives", abs(float(back[0]["stamp"]) - 1.5) < 1e-12)

    P = neighbor_P_from_state_row(back[0])
    check(
        "4 neighbor P 9x9",
        P.shape == (9, 9) and abs(float(P[0, 0]) - 0.01) < 1e-6,
        f"shape={P.shape} P00={P[0,0]}",
    )
    check("4b neighbor P yaw", abs(P[6, 6] - 0.05) < 1e-9)

    rng = np.random.default_rng(0)
    q = DelayedDropQueue(0.02, 0.0, rng)
    q.push(1.0, "a")
    check("5 latency hold", q.pop_ready(1.01) == [])
    check("5b latency release", q.pop_ready(1.03) == ["a"])

    q2 = DelayedDropQueue(0.0, 1.0, np.random.default_rng(1))
    kept = sum(1 for _ in range(20) if q2.push(0.0, 1))
    check("6 total loss drops all", kept == 0)

    if PointCloud2 is not None:
        msg = pack_state([row], 1.5, "world")
        u = unpack_state(msg)
        check("7 ROS pack/unpack", int(u[0]["seq"]) == 7 and msg.point_step == 88)
    else:
        check("7 ROS pack skipped (no sensor_msgs)", True)

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    parser.print_help()
    sys.exit(2)


if __name__ == "__main__":
    main()
