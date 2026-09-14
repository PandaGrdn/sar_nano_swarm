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
import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Union

# Final log names only. `cf_1.tmp.npz` (atomic-save sibling) must not match.
_DRONE_NPZ_NAME = re.compile(r"^cf_\d+\.npz$")

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

MEAS_NAME_ID = {
    "range": 0,
    "relpos": 1,
    "range_rate": 2,
    "reciprocal_relpos": 3,
    "entrance_range": 4,
    "entrance_relpos": 5,
    "mutual_yaw": 6,
    "az_only": 7,
    "entrance_obs_relpos": 8,   # anchor-observed relpos (entrance is observer, §4.3e)
    "entrance_mutual_yaw": 9,   # D11 pair against the fixed-yaw entrance observer
}
ID_MEAS_NAME = {v: k for k, v in MEAS_NAME_ID.items()}

NIS_DTYPE = np.dtype(
    [
        ("stamp", "<f8"),
        ("name_id", "<u4"),
        ("nis", "<f4"),
        ("accepted", "<u4"),
    ]
)

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
        # Advertised 5x5 covariance of [dp_body(3), dpsi, scale] as received on
        # /cf_<i>/rio/delta: upper triangle, row-major, same order as the wire
        # RIO_DTYPE in swarm_msgs.py. NaN when the writer had no covariance and
        # for logs written before these fields existed (see load_drone_log).
        *[(f"cov_{k}", "<f4") for k in range(15)],
        ("valid", "<u4"),
    ]
)
RIO_COV_FIELDS = tuple(f"cov_{k}" for k in range(15))
RIO_COV_N = 5
# (i, j) of each cov_k — identical to swarm_msgs.triu_n(5).
RIO_COV_TRIU = tuple((i, j) for i in range(RIO_COV_N) for j in range(i, RIO_COV_N))


def rio_cov_triu(cov) -> np.ndarray:
    """15 upper-triangle entries from a 5x5 matrix or a 15-vector; None -> NaN."""
    if cov is None:
        return np.full(15, np.nan, dtype=np.float64)
    c = np.asarray(cov, dtype=np.float64)
    if c.shape == (RIO_COV_N, RIO_COV_N):
        return np.array([c[i, j] for i, j in RIO_COV_TRIU], dtype=np.float64)
    c = c.reshape(-1)
    if c.size != 15:
        raise ValueError(f"rio cov must be 5x5 or 15 entries, got shape {np.shape(cov)}")
    return c


def rio_cov_matrix(row) -> np.ndarray:
    """5x5 covariance from one logged rio row (NaN matrix for old logs)."""
    M = np.zeros((RIO_COV_N, RIO_COV_N), dtype=np.float64)
    for k, (i, j) in enumerate(RIO_COV_TRIU):
        M[i, j] = M[j, i] = float(row[f"cov_{k}"])
    return M


def cast_struct_by_name(arr: np.ndarray, dtype: np.dtype, fill: float = float("nan")) -> np.ndarray:
    """Copy a structured array into `dtype` field BY NAME.

    Fields missing from `arr` are filled with `fill` (floats) or 0 (ints).
    Needed because numpy casts structured arrays by field POSITION, so casting
    an old 9-field rio log into the 24-field dtype would misplace `valid`.
    """
    arr = np.asarray(arr)
    out = np.zeros(arr.shape, dtype=dtype)
    src = set(arr.dtype.names or ())
    for name in dtype.names:
        if name in src:
            out[name] = arr[name]
        elif out.dtype.fields[name][0].kind == "f":
            out[name] = fill
    return out


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
        self.nis: List[np.void] = []
        self.stats: dict = {}

    def add_rio(self, stamp, dt, dp, dpsi, roll, pitch, valid, cov=None) -> None:
        """`cov`: advertised 5x5 (or 15 upper-triangle entries); None -> NaN."""
        row = np.zeros(1, dtype=RIO_DTYPE)[0]
        row["stamp"] = float(stamp)
        row["dt"] = float(dt)
        row["dp_x"], row["dp_y"], row["dp_z"] = (float(dp[0]), float(dp[1]), float(dp[2]))
        row["dpsi"] = float(dpsi)
        row["roll"] = float(roll)
        row["pitch"] = float(pitch)
        for k, v in enumerate(rio_cov_triu(cov)):
            row[f"cov_{k}"] = float(v)
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

    def add_nis(self, stamp: float, name: str, nis: float, accepted: bool) -> None:
        row = np.zeros(1, dtype=NIS_DTYPE)[0]
        row["stamp"] = float(stamp)
        row["name_id"] = int(MEAS_NAME_ID.get(str(name), 99))
        row["nis"] = float(nis) if math.isfinite(float(nis)) else float("nan")
        row["accepted"] = 1 if accepted else 0
        self.nis.append(row)

    def set_stats(self, **kwargs) -> None:
        if not hasattr(self, "stats"):
            self.stats = {}
        self.stats.update({k: float(v) for k, v in kwargs.items()})

    def save(self, path: Optional[Union[str, Path]] = None) -> Path:
        out = Path(path) if path is not None else self.path
        if out is None:
            raise ValueError("no log path")
        out = resolve_log_path(str(out), self.drone_id)
        out.parent.mkdir(parents=True, exist_ok=True)
        rio = np.array(self.rio, dtype=RIO_DTYPE) if self.rio else np.zeros(0, dtype=RIO_DTYPE)
        uwb = np.array(self.uwb, dtype=UWB_DTYPE) if self.uwb else np.zeros(0, dtype=UWB_DTYPE)
        est = np.array(self.est, dtype=EST_DTYPE) if self.est else np.zeros(0, dtype=EST_DTYPE)
        nis = np.array(self.nis, dtype=NIS_DTYPE) if self.nis else np.zeros(0, dtype=NIS_DTYPE)
        payload = {
            "drone_id": np.int32(self.drone_id),
            "rio": rio,
            "uwb": uwb,
            "estimate": est,
            "nis": nis,
        }
        for k, v in self.stats.items():
            payload[f"stat_{k}"] = np.float64(v)
        # Write to a hidden sibling then os.replace so a concurrent reader
        # (swarm_loc_gate logs_intact) never opens a half-written zip.
        # Name must NOT match cf_<id>.npz — glob("cf_*.npz") would otherwise
        # pick up the in-flight temp (cf_1.tmp.npz raced the 2026-09-14 gate).
        tmp = out.with_name("." + out.stem + ".partial.npz")
        try:
            np.savez(tmp, **payload)
            os.replace(tmp, out)
        finally:
            if tmp.exists():
                try:
                    tmp.unlink()
                except OSError:
                    pass
        return out


def load_drone_log(path: Union[str, Path]) -> dict:
    z = np.load(path, allow_pickle=False)
    stats = {k[5:]: float(z[k]) for k in z.files if k.startswith("stat_")}
    nis = np.array(z["nis"], dtype=NIS_DTYPE) if "nis" in z.files else np.zeros(0, dtype=NIS_DTYPE)
    return {
        "drone_id": int(z["drone_id"]),
        # By-name cast: old logs (no cov_* fields) load with NaN covariance.
        "rio": cast_struct_by_name(z["rio"], RIO_DTYPE),
        "rio_has_cov": bool(set(RIO_COV_FIELDS) <= set(z["rio"].dtype.names or ())),
        "uwb": np.array(z["uwb"], dtype=UWB_DTYPE),
        "estimate": np.array(z["estimate"], dtype=EST_DTYPE),
        "nis": nis,
        "stats": stats,
        "path": str(path),
    }


def drone_npz_files(directory: Union[str, Path]) -> List[Path]:
    """``cf_<id>.npz`` only — ignores ``cf_1.tmp.npz`` / hidden partials."""
    d = Path(directory)
    return sorted(p for p in d.glob("cf_*.npz") if _DRONE_NPZ_NAME.match(p.name))


def load_run(path: Union[str, Path]) -> Dict[int, dict]:
    """Load one npz or a directory of cf_<id>.npz / *.npz."""
    p = Path(path)
    files: List[Path]
    if p.is_dir():
        files = drone_npz_files(p)
        if not files:
            files = sorted(
                q for q in p.glob("*.npz")
                if q.name != "truth.npz" and not q.name.startswith(".")
            )
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
    log.add_nis(1.0, "relpos", 1.2, True)
    log.set_stats(n_nis_reject=3, cpu_update_s=0.01)
    with tempfile.TemporaryDirectory() as td:
        path = log.save(Path(td) / "cf_1.npz")
        run = load_run(td)
        check("3 load by dir", 1 in run)
        check("3b rio rows", run[1]["rio"].shape[0] == 1)
        check("3c uwb peer", int(run[1]["uwb"][0]["peer_id"]) == 0)
        check("3d est psi", abs(float(run[1]["estimate"][0]["psi"]) - 0.1) < 1e-6)
        one = load_run(path)
        check("3e load by file", 1 in one and one[1]["uwb"].shape[0] == 1)
        check("3f stats roundtrip", abs(run[1]["stats"].get("n_nis_reject", 0) - 3) < 1e-9)
        check("3g nis rows", run[1]["nis"].shape[0] == 1)
        check("3h rio cov NaN when not given",
              bool(np.all(np.isnan(rio_cov_matrix(run[1]["rio"][0])))) and run[1]["rio_has_cov"])

    # 4 — advertised covariance round trip (5x5 in, same upper triangle out)
    C = np.array(
        [[4e-4, 1e-5, 0.0, 0.0, 0.0],
         [1e-5, 5e-4, 0.0, 0.0, 0.0],
         [0.0, 0.0, 9e-4, 0.0, 0.0],
         [0.0, 0.0, 0.0, 2e-6, 0.0],
         [0.0, 0.0, 0.0, 0.0, 1e-8]]
    )
    log4 = MeasurementLogger(2)
    log4.add_rio(2.0, 0.05, [0.02, -0.01, 0.003], 0.002, 0.01, -0.02, True, cov=C)
    log4.add_rio(2.05, 0.05, [0.02, -0.01, 0.003], 0.002, 0.01, -0.02, False, cov=rio_cov_triu(2.0 * C))
    with tempfile.TemporaryDirectory() as td:
        log4.save(Path(td) / "cf_2.npz")
        r4 = load_run(td)[2]
        rio4 = r4["rio"]
        check("4 cov 5x5 roundtrip", np.allclose(rio_cov_matrix(rio4[0]), C, rtol=1e-6, atol=1e-12),
              str(rio_cov_matrix(rio4[0])))
        check("4b cov 15-vector roundtrip", np.allclose(rio_cov_matrix(rio4[1]), 2.0 * C, rtol=1e-6, atol=1e-12))
        check("4c triu order matches wire (cov_1 is [0,1], cov_5 is [1,1])",
              abs(float(rio4[0]["cov_1"]) - 1e-5) < 1e-10 and abs(float(rio4[0]["cov_5"]) - 5e-4) < 1e-9)
        check("4d existing fields unchanged",
              abs(float(rio4[0]["stamp"]) - 2.0) < 1e-12
              and abs(float(rio4[0]["dt"]) - 0.05) < 1e-7
              and abs(float(rio4[0]["dp_x"]) - 0.02) < 1e-7
              and abs(float(rio4[0]["dp_y"]) + 0.01) < 1e-7
              and abs(float(rio4[0]["dp_z"]) - 0.003) < 1e-7
              and abs(float(rio4[0]["dpsi"]) - 0.002) < 1e-7
              and abs(float(rio4[0]["roll"]) - 0.01) < 1e-7
              and abs(float(rio4[0]["pitch"]) + 0.02) < 1e-7
              and int(rio4[0]["valid"]) == 1 and int(rio4[1]["valid"]) == 0)
        check("4e has_cov flag", r4["rio_has_cov"] is True)
        try:
            rio_cov_triu(np.zeros(7))
            bad_raised = False
        except ValueError:
            bad_raised = True
        check("4f malformed cov rejected", bad_raised)

    # 5 — an OLD-format log (rio without cov_*) still loads, cov = NaN
    OLD_RIO = np.dtype([("stamp", "<f8"), ("dt", "<f4"), ("dp_x", "<f4"), ("dp_y", "<f4"),
                        ("dp_z", "<f4"), ("dpsi", "<f4"), ("roll", "<f4"), ("pitch", "<f4"),
                        ("valid", "<u4")])
    old = np.zeros(3, dtype=OLD_RIO)
    old["stamp"] = [1.0, 1.02, 1.04]
    old["dt"] = 0.02
    old["dp_x"] = [0.1, 0.2, 0.3]
    old["dpsi"] = 0.004
    old["pitch"] = 0.05
    old["valid"] = [1, 0, 1]
    with tempfile.TemporaryDirectory() as td:
        np.savez(Path(td) / "cf_4.npz", drone_id=np.int32(4), rio=old,
                 uwb=np.zeros(0, dtype=UWB_DTYPE), estimate=np.zeros(0, dtype=EST_DTYPE))
        r5 = load_run(td)[4]
        rio5 = r5["rio"]
        check("5 old log loads into new dtype", rio5.dtype == RIO_DTYPE and rio5.shape[0] == 3)
        check("5b old log cov all NaN", all(bool(np.all(np.isnan(rio5[f]))) for f in RIO_COV_FIELDS))
        check("5c old log fields by name (valid not misplaced)",
              list(rio5["valid"]) == [1, 0, 1]
              and np.allclose(rio5["dp_x"], [0.1, 0.2, 0.3])
              and np.allclose(rio5["pitch"], 0.05)
              and np.allclose(rio5["stamp"], [1.0, 1.02, 1.04]))
        check("5d old log has_cov False, nis empty", r5["rio_has_cov"] is False and r5["nis"].shape[0] == 0)

    # 6 — atomic replace: a failed np.savez must leave the previous file loadable
    log6 = MeasurementLogger(0)
    log6.add_est(1.0, [0.0, 0.0, 0.5], [0.0, 0.0, 0.0], 0.0, 0)
    with tempfile.TemporaryDirectory() as td:
        dest = Path(td) / "cf_0.npz"
        log6.save(dest)
        load_drone_log(dest)
        real_savez = np.savez

        def boom(path, **kwargs):
            Path(path).write_bytes(b"not a zip file")
            raise RuntimeError("simulated crash mid-write")

        np.savez = boom
        try:
            try:
                log6.save(dest)
                check("6 save raised", False)
            except RuntimeError:
                check("6 save raised", True)
        finally:
            np.savez = real_savez
        try:
            load_drone_log(dest)
            check("6b dest still a valid zip after failed save", True)
        except Exception as exc:
            check("6b dest still a valid zip after failed save", False, repr(exc))
        leftovers = list(Path(td).glob(".*.partial.npz")) + list(Path(td).glob("*.tmp.npz"))
        check("6c no tmp leftover after failed save", leftovers == [], str(leftovers))
        log6.save(dest)
        leftovers = list(Path(td).glob(".*.partial.npz")) + list(Path(td).glob("*.tmp.npz"))
        check("6d tmp gone after successful replace", leftovers == [], str(leftovers))

        # 6e — a stale cf_1.tmp.npz (old temp name) must not be opened by load_run
        decoy = Path(td) / "cf_1.tmp.npz"
        decoy.write_bytes(b"not a zip file")
        run6 = load_run(td)
        check("6e load_run ignores cf_*.tmp.npz",
              list(run6) == [0] and decoy.is_file())
        check("6e2 drone_npz_files skips tmp",
              [p.name for p in drone_npz_files(td)] == ["cf_0.npz"])

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
