#!/usr/bin/env python3
"""rio_stub.py — RIO delta-pose interface + drift-injecting stub (P2-1).

⚠ AGENTS.md §1 Tier A — SIM ORACLE WALL.
This file is the ONLY module in perception/swarm_loc/ allowed to touch Gazebo
truth (`/cf_<id>/odom`). The estimator must subscribe to `/cf_<id>/rio/delta`
and nothing else for odometry.

Pure-math corruption (`RioStubEngine`) has no rclpy and is covered by
`python3 perception/swarm_loc/ekf.py --selftest` (and `--selftest` here).
The rclpy wrapper is a separate process, one per drone:

    python3 -u perception/swarm_loc/rio_stub.py --cf-id 0

Refuses to start unless `rio.source: stub` so a future real RIO cannot
silently fall back to this oracle.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np
import yaml

_REPO_ROOT = Path(__file__).resolve().parents[2]
for _p in ("perception/swarm_loc", "perception/uwb_sim"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

from state import rpy_to_R, rot_to_rpy, wrap_psi  # noqa: E402

EPS = 1e-12


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def resolve_config_path(path: str) -> str:
    if os.path.isabs(path):
        return path
    root = os.environ.get("SAR_NANO_SWARM_ROOT", str(_REPO_ROOT))
    return os.path.join(root, path)


@dataclass
class RioDelta:
    stamp: float
    dt: float
    delta_p_body: np.ndarray  # (3,) body-frame position increment
    delta_psi: float  # yaw increment (rad)
    roll: float  # absolute, from IMU
    pitch: float  # absolute, from IMU
    cov: np.ndarray  # (5,5) of [delta_p_body(3), delta_psi, scale]
    valid: bool  # False when radar returns too sparse to solve


def rio_measurement_cov(cfg: dict) -> np.ndarray:
    rio = cfg["rio"]
    sp = float(rio["sigma_p"]) ** 2
    spsi = math.radians(float(rio["sigma_psi_deg"])) ** 2
    ss = (float(rio["scale_error"]) - 1.0) ** 2
    cov = np.zeros((5, 5), dtype=np.float64)
    cov[0, 0] = cov[1, 1] = cov[2, 2] = sp
    cov[3, 3] = spsi
    cov[4, 4] = max(ss, 1.0e-8)
    return cov


def true_delta_from_poses(
    p0: np.ndarray,
    R0: np.ndarray,
    p1: np.ndarray,
    R1: np.ndarray,
    t0: float,
    t1: float,
) -> Tuple[np.ndarray, float, float, float, float]:
    """Truth body increment over [t0, t1]. Returns (dp_body, dpsi, roll, pitch, dt)."""
    dt = float(t1 - t0)
    if dt < EPS:
        yaw, pitch, roll = rot_to_rpy(R1)
        return np.zeros(3, dtype=np.float64), 0.0, roll, pitch, 0.0
    dp_world = np.asarray(p1, dtype=np.float64) - np.asarray(p0, dtype=np.float64)
    dp_body = R0.T @ dp_world
    yaw0, _, _ = rot_to_rpy(R0)
    yaw1, pitch, roll = rot_to_rpy(R1)
    dpsi = wrap_psi(yaw1 - yaw0)
    return dp_body, dpsi, roll, pitch, dt


class RioStubEngine:
    """Persistent drift state + per-step corruption. No rclpy, no topics."""

    def __init__(self, cfg: dict, rng: np.random.Generator):
        self.cfg = cfg
        self.rng = rng
        rio = cfg["rio"]
        self.vel_bias = np.zeros(3, dtype=np.float64)  # m/s, body
        self.dropout_until = -1.0
        self.vel_bias_walk = float(rio["vel_bias_walk"])
        self.yaw_walk_deg_per_min = float(rio["yaw_walk_deg_per_min"])
        self.scale_error = float(rio["scale_error"])
        self.sigma_p = float(rio["sigma_p"])
        self.sigma_psi = math.radians(float(rio["sigma_psi_deg"]))
        self.dropout_rate = float(rio["dropout_rate"])
        self.dropout_duration_s = float(rio["dropout_duration_s"])
        self.cov = rio_measurement_cov(cfg)

    def corrupt(
        self,
        stamp: float,
        dt: float,
        delta_p_body_true: np.ndarray,
        delta_psi_true: float,
        roll: float,
        pitch: float,
    ) -> RioDelta:
        dt = float(max(dt, 0.0))
        sqrt_dt = math.sqrt(dt) if dt > EPS else 0.0

        self.vel_bias = self.vel_bias + self.rng.normal(
            0.0, self.vel_bias_walk * sqrt_dt, size=3
        )
        yaw_rw_std = math.radians(self.yaw_walk_deg_per_min) / math.sqrt(60.0)
        dpsi_rw = float(self.rng.normal(0.0, yaw_rw_std * sqrt_dt))

        dp = np.asarray(delta_p_body_true, dtype=np.float64).copy()
        dp = self.scale_error * dp + self.vel_bias * dt
        dp = dp + self.rng.normal(0.0, self.sigma_p, size=3)
        dpsi = float(delta_psi_true) + dpsi_rw + float(self.rng.normal(0.0, self.sigma_psi))

        valid = True
        if stamp < self.dropout_until:
            valid = False
        elif dt > EPS and self.rng.random() < self.dropout_rate * dt:
            self.dropout_until = stamp + self.dropout_duration_s
            valid = False

        return RioDelta(
            stamp=float(stamp),
            dt=dt,
            delta_p_body=dp,
            delta_psi=dpsi,
            roll=float(roll),
            pitch=float(pitch),
            cov=self.cov.copy(),
            valid=valid,
        )


def make_engine(cfg: dict, cf_id: int) -> RioStubEngine:
    seed = int(cfg.get("seed", 0)) + int(cf_id)
    rng = np.random.default_rng(seed)
    return RioStubEngine(cfg, rng)


def run_selftest() -> int:
    ok = True

    def check(name: str, cond: bool, detail: str = ""):
        nonlocal ok
        if not cond:
            ok = False
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))
        else:
            print(f"[selftest] PASS {name}")

    cfg_path = resolve_config_path("configs/estimation/swarm_loc.yaml")
    cfg = load_config(cfg_path)
    check("config loads", isinstance(cfg, dict) and "rio" in cfg)
    check("rio.source is stub", cfg["rio"]["source"] == "stub")

    R0 = rpy_to_R(0.0, 0.0, 0.0)
    p0 = np.zeros(3)
    p1 = np.array([0.1, 0.0, 0.0])
    dp, dpsi, roll, pitch, dt = true_delta_from_poses(p0, R0, p1, R0, 0.0, 0.2)
    check("true_delta dt", abs(dt - 0.2) < 1e-12)
    check("true_delta body x", abs(dp[0] - 0.1) < 1e-12 and abs(dp[1]) < 1e-12)
    check("true_delta dpsi 0", abs(dpsi) < 1e-12)

    R90 = rpy_to_R(math.pi / 2, 0.0, 0.0)
    p2 = np.array([0.0, 0.1, 0.0])
    dp2, _, _, _, _ = true_delta_from_poses(p0, R90, p2, R90, 0.0, 0.1)
    # world +Y is body +X at yaw=+90
    check("true_delta yawed", abs(dp2[0] - 0.1) < 1e-9 and abs(dp2[1]) < 1e-9)

    silent = dict(cfg)
    silent["rio"] = dict(cfg["rio"])
    silent["rio"].update(
        {
            "vel_bias_walk": 0.0,
            "yaw_walk_deg_per_min": 0.0,
            "scale_error": 1.0,
            "sigma_p": 0.0,
            "sigma_psi_deg": 0.0,
            "dropout_rate": 0.0,
        }
    )
    eng = RioStubEngine(silent, np.random.default_rng(0))
    d = eng.corrupt(0.2, 0.2, dp, dpsi, 0.0, 0.0)
    check("zero-noise identity p", np.allclose(d.delta_p_body, dp, atol=1e-15))
    check("zero-noise identity psi", abs(d.delta_psi - dpsi) < 1e-15)
    check("zero-noise valid", d.valid is True)
    check("cov 5x5", d.cov.shape == (5, 5) and np.all(np.isfinite(d.cov)))

    eng_a = make_engine(cfg, 0)
    eng_b = make_engine(cfg, 0)
    da = eng_a.corrupt(0.02, 0.02, np.array([0.02, 0.0, 0.0]), 0.0, 0.0, 0.0)
    db = eng_b.corrupt(0.02, 0.02, np.array([0.02, 0.0, 0.0]), 0.0, 0.0, 0.0)
    check("seed reproducibility", np.allclose(da.delta_p_body, db.delta_p_body))

    eng0 = make_engine(cfg, 0)
    eng1 = make_engine(cfg, 1)
    d0 = eng0.corrupt(0.02, 0.02, np.array([0.02, 0.0, 0.0]), 0.0, 0.0, 0.0)
    d1 = eng1.corrupt(0.02, 0.02, np.array([0.02, 0.0, 0.0]), 0.0, 0.0, 0.0)
    check(
        "per-drone seed differs",
        not np.allclose(d0.delta_p_body, d1.delta_p_body),
    )

    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def _run_node(args) -> int:
    cfg = load_config(resolve_config_path(args.config))
    source = str(cfg["rio"]["source"])
    if source != "stub":
        print(
            f"[rio_stub] refuse to start: rio.source={source!r} (must be 'stub')",
            file=sys.stderr,
        )
        return 2

    from uwb_model import quat_to_rot_matrix  # noqa: E402

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    engine = make_engine(cfg, args.cf_id)

    class RioStubNode(Node):
        def __init__(self):
            super().__init__(f"rio_stub_{args.cf_id}")
            self._prev_p = None
            self._prev_R = None
            self._prev_t = None
            self._prev_wall = None
            self.create_subscription(
                Odometry,
                f"/cf_{args.cf_id}/odom",
                self._on_odom,
                qos_profile_sensor_data,
            )
            # Publishing `/rio/delta` PointCloud2 is P2-5 (swarm_msgs.py).
            # P2-1 only needs the corruption engine + this truth subscription.
            self.get_logger().warn(
                "⚠ AGENTS.md §1 Tier A: this node reads /cf_"
                f"{args.cf_id}/odom (Gazebo truth) and must never be imported "
                "by swarm_loc_node."
            )

        def _on_odom(self, msg: Odometry):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            wall = time.time()
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            p = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
            R = quat_to_rot_matrix((ori.x, ori.y, ori.z, ori.w))
            if self._prev_p is None:
                self._prev_p, self._prev_R, self._prev_t = p, R, stamp
                self._prev_wall = wall
                return
            dt = stamp - self._prev_t
            stale_s = float(cfg.get("measurements", {}).get("max_measurement_age_s", 0.5))
            if wall - self._prev_wall > stale_s:
                self._prev_p, self._prev_R, self._prev_t = p, R, stamp
                self._prev_wall = wall
                return
            dp, dpsi, roll, pitch, dt = true_delta_from_poses(
                self._prev_p, self._prev_R, p, R, self._prev_t, stamp
            )
            _delta = engine.corrupt(stamp, dt, dp, dpsi, roll, pitch)
            self._prev_p, self._prev_R, self._prev_t = p, R, stamp
            self._prev_wall = wall

    rclpy.init()
    node = RioStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--config", default="configs/estimation/swarm_loc.yaml")
    parser.add_argument("--cf-id", type=int, default=0)
    args = parser.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    sys.exit(_run_node(args))


if __name__ == "__main__":
    main()
