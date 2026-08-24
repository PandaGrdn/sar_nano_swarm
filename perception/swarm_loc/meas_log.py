#!/usr/bin/env python3
"""meas_log.py — on-disk measurement log for the P2-7 centralized solver.

One ``.npz`` per drone. Written by ``swarm_loc_node.py --log-measurements``.
Do not reconstruct measurements from a rosbag.

Usage:
    python3 perception/swarm_loc/meas_log.py --selftest
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Union

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[2]
for _p in ("perception/swarm_loc", "perception/uwb_sim"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

from uwb_edges import FLAG_BEARING_VALID, FLAG_PEER_IS_SURVEYED  # noqa: E402

KIND_RANGE = 0
KIND_RELPOS = 1
KIND_RECIPROCAL = 2
KIND_ENTRANCE_RANGE = 3
KIND_ENTRANCE_RELPOS = 4
KIND_RANGE_RATE = 5
KIND_MUTUAL_YAW = 6

KIND_NAME = {
    KIND_RANGE: "range",
    KIND_RELPOS: "relpos",
    KIND_RECIPROCAL: "reciprocal",
    KIND_ENTRANCE_RANGE: "entrance_range",
    KIND_ENTRANCE_RELPOS: "entrance_relpos",
    KIND_RANGE_RATE: "range_rate",
    KIND_MUTUAL_YAW: "mutual_yaw",
}

UWB_DTYPE = np.dtype(
    [
        ("stamp", "<f8"),
        ("kind", "<u4"),
        ("observer_id", "<u4"),
        ("peer_id", "<u4"),
        ("range_m", "<f4"),
        ("azimuth_rad", "<f4"),
        ("elevation_rad", "<f4"),
        ("sigma_range_m", "<f4"),
        ("sigma_az_rad", "<f4"),
        ("sigma_el_rad", "<f4"),
        ("psi_obs", "<f4"),
        ("roll_obs", "<f4"),
        ("pitch_obs", "<f4"),
        ("z0", "<f4"),
        ("z1", "<f4"),
        ("z2", "<f4"),
    ]
)

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
        ("valid", "<u4"),
    ]
)

EST_DTYPE = np.dtype(
    [
        ("stamp", "<f8"),
        ("p_x", "<f4"),
        ("p_y", "<f4"),
        ("p_z", "<f4"),
        ("v_x", "<f4"),
        ("v_y", "<f4"),
        ("v_z", "<f4"),
        ("psi", "<f4"),
        ("status", "<u4"),
    ]
)


def kind_from_edge(edge, use_bearing: bool = True) -> int:
    flags = int(edge["flags"] if not isinstance(edge, dict) else edge.get("flags", 0))
    surveyed = bool(flags & FLAG_PEER_IS_SURVEYED)
    z = float(edge["z"] if not isinstance(edge, dict) else edge.get("z", float("nan")))
    full = bool(use_bearing and (flags & FLAG_BEARING_VALID) and math.isfinite(z))
    if surveyed and full:
        return KIND_ENTRANCE_RELPOS
    if surveyed:
        return KIND_ENTRANCE_RANGE
    if full:
        return KIND_RELPOS
    return KIND_RANGE


def resolve_log_path(path: str, drone_id: int) -> Path:
    p = Path(path)
    if p.suffix.lower() == ".npz":
        return p
    return p / f"cf_{int(drone_id)}.npz"


class MeasurementLogger:
    """Append-only buffers, flushed with save()."""

    def __init__(self, drone_id: int, path: Optional[Union[str, Path]] = None):
        self.drone_id = int(drone_id)
        self.path = Path(path) if path is not None else None
        self.rio: List[np.void] = []
        self.uwb: List[np.void] = []
        self.est: List[np.void] = []

    def add_rio(self, stamp, dt, dp, dpsi, roll, pitch, valid) -> None:
        row = np.zeros(1, dtype=RIO_DTYPE)[0]
        row["stamp"] = float(stamp)
        row["dt"] = float(dt)
        row["dp_x"], row["dp_y"], row["dp_z"] = (float(dp[0]), float(dp[1]), float(dp[2]))
        row["dpsi"] = float(dpsi)
        row["roll"] = float(roll)
        row["pitch"] = float(pitch)
        row["valid"] = 1 if valid else 0
        self.rio.append(row)

    def add_uwb(
        self,
        stamp: float,
        kind: int,
        observer_id: int,
        peer_id: int,
        range_m: float,
        azimuth_rad: float,
        elevation_rad: float,
        sigma_range_m: float,
        sigma_az_rad: float,
        sigma_el_rad: float,
        psi_obs: float,
        roll_obs: float,
        pitch_obs: float,
        z_body=None,
    ) -> None:
        row = np.zeros(1, dtype=UWB_DTYPE)[0]
        row["stamp"] = float(stamp)
        row["kind"] = int(kind)
        row["observer_id"] = int(observer_id)
        row["peer_id"] = int(peer_id)
        row["range_m"] = float(range_m)
        row["azimuth_rad"] = float(azimuth_rad)
        row["elevation_rad"] = float(elevation_rad)
        row["sigma_range_m"] = float(sigma_range_m)
        row["sigma_az_rad"] = float(sigma_az_rad)
        row["sigma_el_rad"] = float(sigma_el_rad)
        row["psi_obs"] = float(psi_obs)
        row["roll_obs"] = float(roll_obs)
        row["pitch_obs"] = float(pitch_obs)
        if z_body is None:
            row["z0"] = row["z1"] = row["z2"] = float("nan")
        else:
            z = np.asarray(z_body, dtype=np.float64).reshape(-1)
            row["z0"] = float(z[0]) if z.size > 0 else float("nan")
            row["z1"] = float(z[1]) if z.size > 1 else 0.0
            row["z2"] = float(z[2]) if z.size > 2 else 0.0
        self.uwb.append(row)

    def add_est(self, stamp, p, v, psi, status: int = 0) -> None:
        row = np.zeros(1, dtype=EST_DTYPE)[0]
        row["stamp"] = float(stamp)
        row["p_x"], row["p_y"], row["p_z"] = (float(p[0]), float(p[1]), float(p[2]))
        row["v_x"], row["v_y"], row["v_z"] = (float(v[0]), float(v[1]), float(v[2]))
        row["psi"] = float(psi)
        row["status"] = int(status)
        self.est.append(row)

    def save(self, path: Optional[Union[str, Path]] = None) -> Path:
        out = Path(path) if path is not None else self.path
        if out is None:
            raise ValueError("no log path")
        out = resolve_log_path(str(out), self.drone_id)
        out.parent.mkdir(parents=True, exist_ok=True)
        rio = np.array(self.rio, dtype=RIO_DTYPE) if self.rio else np.zeros(0, dtype=RIO_DTYPE)
        uwb = np.array(self.uwb, dtype=UWB_DTYPE) if self.uwb else np.zeros(0, dtype=UWB_DTYPE)
        est = np.array(self.est, dtype=EST_DTYPE) if self.est else np.zeros(0, dtype=EST_DTYPE)
        np.savez(
            out,
            drone_id=np.int32(self.drone_id),
            rio=rio,
            uwb=uwb,
            estimate=est,
        )
        return out


def load_drone_log(path: Union[str, Path]) -> dict:
    z = np.load(path, allow_pickle=False)
    return {
        "drone_id": int(z["drone_id"]),
        "rio": np.array(z["rio"], dtype=RIO_DTYPE),
        "uwb": np.array(z["uwb"], dtype=UWB_DTYPE),
        "estimate": np.array(z["estimate"], dtype=EST_DTYPE),
        "path": str(path),
    }


def load_run(path: Union[str, Path]) -> Dict[int, dict]:
    """Load one npz or a directory of cf_*.npz / *.npz."""
    p = Path(path)
    files: List[Path]
    if p.is_dir():
        files = sorted(p.glob("cf_*.npz"))
        if not files:
            files = sorted(p.glob("*.npz"))
    else:
        files = [p]
    out: Dict[int, dict] = {}
    for f in files:
        rec = load_drone_log(f)
        out[int(rec["drone_id"])] = rec
    return out


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

    check("1 resolve file", resolve_log_path("out/cf_2.npz", 9).name == "cf_2.npz")
    check("1b resolve dir", resolve_log_path("out/logs", 3).name == "cf_3.npz")
    edge = {"flags": FLAG_BEARING_VALID | FLAG_PEER_IS_SURVEYED, "z": 0.1}
    check("2 kind entrance relpos", kind_from_edge(edge) == KIND_ENTRANCE_RELPOS)
    check("2b kind range if no bearing", kind_from_edge(edge, use_bearing=False) == KIND_ENTRANCE_RANGE)

    import tempfile

    log = MeasurementLogger(1)
    log.add_rio(1.0, 0.02, [0.01, 0.0, 0.0], 0.001, 0.0, 0.0, True)
    log.add_uwb(
        1.0, KIND_RELPOS, 1, 0, 1.5, 0.0, 0.0, 0.08, 0.08, 0.08, 0.1, 0.0, 0.0, [1.5, 0.0, 0.0]
    )
    log.add_est(1.0, [1.5, 0.0, 0.5], [0.0, 0.0, 0.0], 0.1, 0)
    with tempfile.TemporaryDirectory() as td:
        path = log.save(Path(td) / "cf_1.npz")
        run = load_run(td)
        check("3 load by dir", 1 in run)
        check("3b rio rows", run[1]["rio"].shape[0] == 1)
        check("3c uwb peer", int(run[1]["uwb"][0]["peer_id"]) == 0)
        check("3d est psi", abs(float(run[1]["estimate"][0]["psi"]) - 0.1) < 1e-6)
        one = load_run(path)
        check("3e load by file", 1 in one and one[1]["uwb"].shape[0] == 1)

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
