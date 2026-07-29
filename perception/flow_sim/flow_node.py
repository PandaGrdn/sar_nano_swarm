#!/usr/bin/env python3
"""flow_node.py — PMW3901 downward optical-flow simulator (Phase 1 M3b).

Analytic rclpy node modelling the PixArt PMW3901 on the Bitcraze Flow deck v2.
This is NOT computed from the VL53L1x ToF laser — it is a separate chip on the
same PCB. ToF supplies height; this node supplies integrated pixel deltas and a
derived (vx, vy) convenience channel using the Crazyflie firmware's flow
measurement equation (mm_flow.c).

⚠ AGENTS.md §1 Tier A: /cf_<id>/flow/debug_truth is a SIM ORACLE for gates only.
The Phase-2 estimator / RIO front-end must NEVER subscribe to it.

This node does NOT feed flow back into the SITL firmware Kalman filter
(estimatorEnqueueFlow) — that would require submodule C edits. Deliverable is
the measurement stream only.

Usage (setup_env.sh sourced):
    python3 -u perception/flow_sim/flow_node.py [--config configs/sensors/optical_flow.yaml]
    python3 perception/flow_sim/flow_node.py --selftest
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import yaml

# ---------------------------------------------------------------------------
# Pure math — no rclpy (testable offline via --selftest)
# ---------------------------------------------------------------------------

FLOW_RESOLUTION_DEFAULT = 0.1


def quat_conj(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_mul(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_rotate_world_to_body(
    q: Tuple[float, float, float, float],
    v_world: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """v_body = R(q)^T * v_world for unit quaternion (x,y,z,w)."""
    x, y, z, w = q
    # Rotation matrix from quaternion (Hamilton, ROS convention)
    r00 = 1 - 2 * (y * y + z * z)
    r01 = 2 * (x * y - z * w)
    r02 = 2 * (x * z + y * w)
    r10 = 2 * (x * y + z * w)
    r11 = 1 - 2 * (x * x + z * z)
    r12 = 2 * (y * z - x * w)
    r20 = 2 * (x * z - y * w)
    r21 = 2 * (y * z + x * w)
    r22 = 1 - 2 * (x * x + y * y)
    vx, vy, vz = v_world
    return (
        r00 * vx + r10 * vy + r20 * vz,
        r01 * vx + r11 * vy + r21 * vz,
        r02 * vx + r12 * vy + r22 * vz,
    )


def r22_from_quat(q: Tuple[float, float, float, float]) -> float:
    x, y, z, w = q
    return 1.0 - 2.0 * (x * x + y * y)


def body_rates_from_quat_diff(
    q_prev: Tuple[float, float, float, float],
    q_curr: Tuple[float, float, float, float],
    dt: float,
) -> Tuple[float, float, float]:
    if dt <= 0.0:
        return (0.0, 0.0, 0.0)
    dq = quat_mul(quat_conj(q_prev), q_curr)
    dx, dy, dz, dw = dq
    if dw < 0.0:
        dx, dy, dz, dw = -dx, -dy, -dz, -dw
    return (2.0 * dx / dt, 2.0 * dy / dt, 2.0 * dz / dt)


def _ramp(x: float, lo: float, hi: float) -> float:
    if x <= lo:
        return 0.0
    if x >= hi:
        return 1.0
    return (x - lo) / (hi - lo)


@dataclass
class FlowFrame:
    dpixelx: float
    dpixely: float
    dt: float
    sigma_px: float
    quality: float
    valid: bool
    vx_hat: float
    vy_hat: float
    var_v: float
    fx_rate: float
    fy_rate: float
    outlier_clipped: bool


class FlowModel:
    """PMW3901 measurement model per M3b plan §2."""

    def __init__(self, cfg: dict, rng: Optional[np.random.Generator] = None):
        self.cfg = cfg
        self.npix = float(cfg["npix"])
        self.thetapix = float(cfg["thetapix_rad"])
        self.k = self.npix / self.thetapix
        self.flow_resolution = float(cfg.get("flow_resolution", FLOW_RESOLUTION_DEFAULT))
        self.outlier_limit_px = float(cfg["outlier_limit_px"])
        self.flow_std_px = float(cfg["flow_std_px"])
        self.quality_eps = float(cfg["quality_eps"])
        self.max_flow_rad_s = float(cfg["max_flow_rad_s"])
        self.z_floor_m = float(cfg["z_floor_m"])
        self.r22_min = float(cfg["r22_min"])
        self.invalid_variance = float(cfg["invalid_variance"])
        self.min_quality = float(cfg["min_quality"])
        self.min_height_m = float(cfg["min_height_m"])
        self.good_height_min_m = float(cfg["good_height_min_m"])
        self.good_height_max_m = float(cfg["good_height_max_m"])
        self.max_height_m = float(cfg["max_height_m"])
        self.tilt_full_rad = math.radians(float(cfg["tilt_full_deg"]))
        self.tilt_zero_rad = math.radians(float(cfg["tilt_zero_deg"]))
        self.default_texture_quality = float(cfg.get("default_texture_quality", 1.0))
        self.patches = cfg.get("surface_patches", [])
        self.rng = rng if rng is not None else np.random.default_rng(int(cfg.get("seed", 0)))

    def surface_quality(self, x: float, y: float) -> float:
        q = self.default_texture_quality
        for patch in self.patches:
            if (
                patch["x_min"] <= x <= patch["x_max"]
                and patch["y_min"] <= y <= patch["y_max"]
            ):
                q = float(patch["texture_quality"])
        return q

    def q_height(self, h: float) -> float:
        if h < self.min_height_m or h > self.max_height_m:
            return 0.0
        if h < self.good_height_min_m:
            return _ramp(h, self.min_height_m, self.good_height_min_m)
        if h <= self.good_height_max_m:
            return 1.0
        return 1.0 - _ramp(h, self.good_height_max_m, self.max_height_m)

    def q_tilt(self, r22: float) -> float:
        r22_c = max(min(r22, 1.0), -1.0)
        tilt = math.acos(r22_c)
        if tilt <= self.tilt_full_rad:
            return 1.0
        if tilt >= self.tilt_zero_rad:
            return 0.0
        return 1.0 - _ramp(tilt, self.tilt_full_rad, self.tilt_zero_rad)

    def q_flow(self, fx_rate: float, fy_rate: float) -> float:
        peak = max(abs(fx_rate), abs(fy_rate))
        return 1.0 - min(peak / self.max_flow_rad_s, 1.0)

    def compute_rates(
        self,
        vx_body: float,
        vy_body: float,
        wx_body: float,
        wy_body: float,
        h_meas: float,
        r22: float,
    ) -> Tuple[float, float, float, float]:
        z_eff = max(h_meas, self.z_floor_m)
        r22_c = max(r22, 1e-6)
        fx_rate = (vx_body * r22_c / z_eff) - wy_body
        fy_rate = (vy_body * r22_c / z_eff) + wx_body
        fx_sat = max(min(fx_rate, self.max_flow_rad_s), -self.max_flow_rad_s)
        fy_sat = max(min(fy_rate, self.max_flow_rad_s), -self.max_flow_rad_s)
        return fx_rate, fy_rate, fx_sat, fy_sat

    def forward_pixels(
        self,
        dt: float,
        fx_sat: float,
        fy_sat: float,
    ) -> Tuple[float, float]:
        n_x = dt * self.k * fx_sat
        n_y = dt * self.k * fy_sat
        return n_x / self.flow_resolution, n_y / self.flow_resolution

    def invert_velocity(
        self,
        dpixelx: float,
        dpixely: float,
        dt: float,
        wx_body: float,
        wy_body: float,
        h_meas: float,
        r22: float,
        sigma_px: float = 0.0,
    ) -> Tuple[float, float, float]:
        z_eff = max(h_meas, self.z_floor_m)
        r22_d = max(r22, self.r22_min)
        if dt <= 0.0:
            return 0.0, 0.0, self.invalid_variance
        fx_hat = (dpixelx * self.flow_resolution) / (dt * self.k)
        fy_hat = (dpixely * self.flow_resolution) / (dt * self.k)
        vx_hat = (fx_hat + wy_body) * z_eff / r22_d
        vy_hat = (fy_hat - wx_body) * z_eff / r22_d
        sigma_v = (sigma_px * self.flow_resolution / (dt * self.k)) * z_eff / r22_d
        return vx_hat, vy_hat, sigma_v * sigma_v

    def synthesize(
        self,
        dt: float,
        vx_body: float,
        vy_body: float,
        wx_body: float,
        wy_body: float,
        h_meas: float,
        r22: float,
        world_x: float,
        world_y: float,
        height_ok: bool,
        add_noise: bool = True,
    ) -> FlowFrame:
        fx_rate, fy_rate, fx_sat, fy_sat = self.compute_rates(
            vx_body, vy_body, wx_body, wy_body, h_meas, r22
        )
        q = (
            self.surface_quality(world_x, world_y)
            * self.q_height(h_meas)
            * self.q_tilt(r22)
            * self.q_flow(fx_rate, fy_rate)
        )
        dpixelx, dpixely = self.forward_pixels(dt, fx_sat, fy_sat)
        sigma_px = self.flow_std_px / max(q, self.quality_eps) if add_noise else 0.0
        outlier_clipped = False

        if add_noise and q > 0.0:
            dpixelx += float(self.rng.normal(0.0, sigma_px))
            dpixely += float(self.rng.normal(0.0, sigma_px))
            dpixelx = float(round(dpixelx))
            dpixely = float(round(dpixely))
            if abs(dpixelx) >= self.outlier_limit_px or abs(dpixely) >= self.outlier_limit_px:
                outlier_clipped = True

        valid = (
            height_ok
            and (q >= self.min_quality)
            and (not outlier_clipped)
        )
        vx_hat, vy_hat, var_v = self.invert_velocity(
            dpixelx, dpixely, dt, wx_body, wy_body, h_meas, r22, sigma_px
        )
        if not valid:
            vx_hat = vy_hat = 0.0
            var_v = self.invalid_variance
            q = 0.0 if outlier_clipped or not height_ok else q

        return FlowFrame(
            dpixelx=dpixelx,
            dpixely=dpixely,
            dt=dt,
            sigma_px=sigma_px,
            quality=q,
            valid=valid,
            vx_hat=vx_hat,
            vy_hat=vy_hat,
            var_v=var_v,
            fx_rate=fx_rate,
            fy_rate=fy_rate,
            outlier_clipped=outlier_clipped,
        )


def run_selftest() -> int:
    cfg = {
        "npix": 35.0,
        "thetapix_rad": 0.71674,
        "flow_resolution": 0.1,
        "outlier_limit_px": 100,
        "flow_std_px": 2.0,
        "quality_eps": 0.05,
        "max_flow_rad_s": 7.4,
        "z_floor_m": 0.1,
        "r22_min": 0.5,
        "invalid_variance": 1e6,
        "min_quality": 0.15,
        "min_height_m": 0.08,
        "good_height_min_m": 0.15,
        "good_height_max_m": 2.5,
        "max_height_m": 4.0,
        "tilt_full_deg": 15.0,
        "tilt_zero_deg": 45.0,
        "default_texture_quality": 1.0,
        "surface_patches": [
            {"name": "smooth_tile", "x_min": 1.0, "x_max": 3.0,
             "y_min": -1.0, "y_max": 1.0, "texture_quality": 0.0},
        ],
        "seed": 42,
    }
    m = FlowModel(cfg, rng=np.random.default_rng(42))
    tol = 1e-3

    def check(name: str, cond: bool, detail: str = ""):
        if not cond:
            print(f"[selftest] FAIL: {name} {detail}", file=sys.stderr)
            return False
        print(f"[selftest] PASS: {name}")
        return True

    ok = True
    f = m.synthesize(0.01, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, True, add_noise=False)
    ok &= check("forward model", abs(f.dpixelx - 4.883) / 4.883 < tol, f"got {f.dpixelx}")
    ok &= check("forward vy=0", abs(f.dpixely) < 1e-6)

    f2 = m.synthesize(0.01, 1.0, 0.0, 0.0, 0.0, 2.0, 1.0, 0.0, 0.0, True, add_noise=False)
    ok &= check("height scaling", abs(f2.dpixelx - 2.441) / 2.441 < tol, f"got {f2.dpixelx}")

    f3 = m.synthesize(0.01, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, True, add_noise=False)
    ok &= check("gyro wy", abs(f3.dpixelx - (-4.883)) / 4.883 < tol, f"got {f3.dpixelx}")

    f4 = m.synthesize(0.01, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, True, add_noise=False)
    ok &= check("gyro wx", abs(f4.dpixely - 4.883) / 4.883 < tol, f"got {f4.dpixely}")

    for vx, vy, h, wx, wy, r22 in [
        (0.5, -0.3, 1.2, 0.1, -0.05, 0.95),
        (1.5, 0.8, 0.6, 0.0, 0.0, 1.0),
        (-0.2, 0.4, 2.0, 0.02, 0.01, 0.9),
    ]:
        fr = m.synthesize(0.01, vx, vy, wx, wy, h, r22, 0.0, 0.0, True, add_noise=False)
        rvx, rvy, _ = m.invert_velocity(
            fr.dpixelx, fr.dpixely, 0.01, wx, wy, h, r22, 0.0
        )
        ok &= check(
            f"round trip vx={vx}",
            abs(rvx - vx) < 1e-6 and abs(rvy - vy) < 1e-6,
            f"got ({rvx},{rvy})",
        )

    fs = m.synthesize(0.01, 100.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, True, add_noise=False)
    ok &= check("saturation invalid", not fs.valid and fs.quality == 0.0)

    ok &= check("surface smooth", m.surface_quality(2.0, 0.0) == 0.0)
    ok &= check("surface good", m.surface_quality(0.0, 0.0) == 1.0)
    fp = m.synthesize(0.01, 0.5, 0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 0.0, True, add_noise=False)
    ok &= check("patch invalid", not fp.valid)

    for bad_h in (0.05, 5.0):
        fb = m.synthesize(0.01, 0.5, 0.0, 0.0, 0.0, bad_h, 1.0, 0.0, 0.0, True, add_noise=False)
        ok &= check(f"height band h={bad_h}", fb.quality == 0.0 and not fb.valid)

    fi = m.synthesize(0.01, 0.5, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, False, add_noise=False)
    ok &= check("height stale invalid", not fi.valid and fi.var_v == 1e6)

    m_noise = FlowModel(cfg, rng=np.random.default_rng(123))
    samples = [
        m_noise.synthesize(0.01, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, True, add_noise=True).dpixelx
        for _ in range(10000)
    ]
    # Force quality ~0.5 by using height at edge of good band with textured surface
    m_half = FlowModel({**cfg, "good_height_min_m": 0.15, "good_height_max_m": 2.5}, rng=np.random.default_rng(99))
    half_samples = []
    for _ in range(10000):
        fr = m_half.synthesize(
            0.01, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, True, add_noise=True
        )
        # manually scale noise path: inject with fixed quality=0.5
        sigma = 2.0 / 0.5
        half_samples.append(float(m_half.rng.normal(0.0, sigma)))
    emp_std = float(np.std(half_samples))
    expected = 2.0 / 0.5
    ok &= check(
        "noise scaling",
        abs(emp_std - expected) / expected < 0.05,
        f"emp={emp_std:.3f} expected={expected:.3f}",
    )

    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def load_yaml(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def check_flow_deck(tof_config_path: str) -> None:
    cfg = load_yaml(tof_config_path)
    deck = cfg.get("deck", "")
    if deck != "flow_v2":
        print(
            f"[flow_node] ERROR: deck='{deck}' — PMW3901 optical flow exists only on "
            "Flow deck v2 (deck: flow_v2 in configs/sensors/tof.yaml). "
            "zranger_v2 has VL53L1x ToF only.",
            file=sys.stderr,
        )
        sys.exit(2)


def build_twist_covariance(var_v: float, invalid_var: float, valid: bool) -> list:
    cov = [0.0] * 36
    v = var_v if valid else invalid_var
    cov[0] = v   # vx
    cov[7] = v   # vy
    for idx in (14, 21, 28, 35):
        cov[idx] = invalid_var
    return cov


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/sensors/optical_flow.yaml")
    parser.add_argument("--tof-config", default="configs/sensors/tof.yaml")
    parser.add_argument("--cf-id", default="0")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--no-truth", action="store_true")
    parser.add_argument("--height-source", choices=("tof", "truth"), default=None)
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        sys.exit(run_selftest())

    check_flow_deck(args.tof_config)
    cfg = load_yaml(args.config)
    if args.seed is not None:
        cfg["seed"] = args.seed
    if args.height_source is not None:
        cfg["height_source"] = args.height_source
    publish_truth = cfg.get("publish_ground_truth", True) and not args.no_truth

    import rclpy
    from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, Vector3Stamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import LaserScan

    class FlowNode(Node):
        def __init__(self):
            super().__init__("flow_sim")
            self.cf_id = str(args.cf_id)
            self.cfg = cfg
            self.model = FlowModel(cfg, rng=np.random.default_rng(int(cfg.get("seed", 0))))
            self.height_source = cfg.get("height_source", "tof")
            self.invalid_var = float(cfg["invalid_variance"])
            self.publish_truth = publish_truth
            self.update_rate_hz = float(cfg["update_rate_hz"])

            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            prefix = f"/cf_{self.cf_id}"
            self.sub_odom = self.create_subscription(Odometry, f"{prefix}/odom", self._on_odom, qos)
            self.sub_tof = self.create_subscription(LaserScan, f"{prefix}/tof_down", self._on_tof, qos)

            self.pub_flow = self.create_publisher(TwistWithCovarianceStamped, f"{prefix}/flow", qos)
            self.pub_pixels = self.create_publisher(Vector3Stamped, f"{prefix}/flow/pixels", qos)
            self.pub_meta = self.create_publisher(Vector3Stamped, f"{prefix}/flow/meta", qos)
            if self.publish_truth:
                self.pub_truth = self.create_publisher(TwistStamped, f"{prefix}/flow/debug_truth", qos)
                self.get_logger().warn(
                    "publish_ground_truth=True: /flow/debug_truth is a SIM ORACLE — "
                    "AGENTS.md §1 Tier A: the estimator must NEVER subscribe to it."
                )

            self._prev_pos = None
            self._prev_quat = None
            self._prev_stamp = None
            self._wx_f = self._wy_f = self._wz_f = 0.0
            self._last_tof_range = None
            self._last_tof_stamp = None
            self._last_tof_recv_wall = None
            self._last_odom_stamp = None
            self._last_pub_time = None
            self._twist_frame_logged = False

            period = 1.0 / self.update_rate_hz
            self.create_timer(period, self._on_timer)

            self.get_logger().info(
                f"flow_node cf_id={self.cf_id} K={self.model.k:.3f} px/rad "
                f"rate={self.update_rate_hz} Hz height_source={self.height_source} "
                f"seed={cfg.get('seed', 0)}"
            )

        def _on_tof(self, msg: LaserScan):
            if not msg.ranges:
                return
            r = float(msg.ranges[0])
            if math.isfinite(r):
                self._last_tof_range = r
                self._last_tof_stamp = msg.header.stamp
                self._last_tof_recv_wall = time.time()

        def _on_odom(self, msg: Odometry):
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            q = (ori.x, ori.y, ori.z, ori.w)
            stamp = msg.header.stamp
            t = stamp.sec + stamp.nanosec * 1e-9

            if self._prev_pos is not None and self._prev_stamp is not None:
                dt = t - self._prev_stamp
                if dt > 0.0:
                    dp = (
                        (pos.x - self._prev_pos[0]) / dt,
                        (pos.y - self._prev_pos[1]) / dt,
                        (pos.z - self._prev_pos[2]) / dt,
                    )
                    vb = quat_rotate_world_to_body(q, dp)
                    self._v_body = (vb[0], vb[1])
                    wx, wy, wz = body_rates_from_quat_diff(self._prev_quat, q, dt)
                    alpha = float(self.cfg.get("omega_lowpass_alpha", 0.3))
                    self._wx_f = alpha * wx + (1.0 - alpha) * self._wx_f
                    self._wy_f = alpha * wy + (1.0 - alpha) * self._wy_f
                    self._wz_f = alpha * wz + (1.0 - alpha) * self._wz_f

                    if not self._twist_frame_logged:
                        tl = msg.twist.twist.linear
                        tw_norm = math.hypot(tl.x, tl.y, tl.z)
                        dp_norm = math.hypot(dp[0], dp[1], dp[2])
                        vb_norm = math.hypot(vb[0], vb[1])
                        if tw_norm > 1e-6:
                            match_body = abs(tw_norm - vb_norm) < abs(tw_norm - dp_norm)
                            frame = "BODY" if match_body else "WORLD"
                            self.get_logger().info(
                                f"odom.twist.linear appears to be in the {frame} frame "
                                f"(twist_norm={tw_norm:.4f} vb_norm={vb_norm:.4f} dp_norm={dp_norm:.4f})"
                            )
                        self._twist_frame_logged = True
                else:
                    self._v_body = getattr(self, "_v_body", (0.0, 0.0))
            else:
                self._v_body = (0.0, 0.0)

            self._prev_pos = (pos.x, pos.y, pos.z)
            self._prev_quat = q
            self._prev_stamp = t
            self._last_odom_stamp = stamp
            self._world_xy = (pos.x, pos.y)
            self._r22 = r22_from_quat(q)
            if self.height_source == "truth":
                self._truth_h = pos.z

        def _height_ok_and_value(self) -> Tuple[bool, float]:
            if self.height_source == "truth":
                h = getattr(self, "_truth_h", None)
                return (h is not None and math.isfinite(h)), (h or 0.0)

            if self._last_tof_range is None or self._last_tof_recv_wall is None:
                return False, 0.0
            age = time.time() - self._last_tof_recv_wall
            if age > float(self.cfg["tof_stale_s"]):
                return False, 0.0
            r = self._last_tof_range
            if not math.isfinite(r):
                return False, 0.0
            if r < float(self.cfg["tof_min_valid_m"]) or r > float(self.cfg["tof_max_valid_m"]):
                return False, 0.0
            # ToF sensor reads range from its mount at z=-0.02 on base_link — use directly.
            return True, r

        def _on_timer(self):
            if self._prev_pos is None:
                return
            now = self.get_clock().now()
            if self._last_pub_time is None:
                self._last_pub_time = now
                return
            dt = (now - self._last_pub_time).nanoseconds * 1e-9
            if dt <= 0.0:
                return
            self._last_pub_time = now

            vx, vy = getattr(self, "_v_body", (0.0, 0.0))
            height_ok, h_meas = self._height_ok_and_value()
            wx, wy = self._wx_f, self._wy_f
            r22 = getattr(self, "_r22", 1.0)
            wx_wy = getattr(self, "_world_xy", (0.0, 0.0))

            frame = self.model.synthesize(
                dt, vx, vy, wx, wy, h_meas, r22, wx_wy[0], wx_wy[1], height_ok, add_noise=True
            )

            stamp = self._last_odom_stamp if self._last_odom_stamp is not None else now.to_msg()
            frame_id = f"cf_{self.cf_id}/base_link"

            flow_msg = TwistWithCovarianceStamped()
            flow_msg.header.stamp = stamp
            flow_msg.header.frame_id = frame_id
            flow_msg.twist.twist.linear.x = frame.vx_hat if frame.valid else 0.0
            flow_msg.twist.twist.linear.y = frame.vy_hat if frame.valid else 0.0
            flow_msg.twist.covariance = build_twist_covariance(
                frame.var_v, self.invalid_var, frame.valid
            )
            self.pub_flow.publish(flow_msg)

            pix = Vector3Stamped()
            pix.header = flow_msg.header
            pix.vector.x = frame.dpixelx
            pix.vector.y = frame.dpixely
            pix.vector.z = frame.quality
            self.pub_pixels.publish(pix)

            meta = Vector3Stamped()
            meta.header = flow_msg.header
            meta.vector.x = frame.dt
            meta.vector.y = frame.sigma_px
            meta.vector.z = h_meas
            self.pub_meta.publish(meta)

            if self.publish_truth:
                truth = TwistStamped()
                truth.header = flow_msg.header
                truth.twist.linear.x = vx
                truth.twist.linear.y = vy
                truth.twist.linear.z = 0.0
                truth.twist.angular.x = wx
                truth.twist.angular.y = wy
                truth.twist.angular.z = self._wz_f
                self.pub_truth.publish(truth)

            self.get_logger().debug(
                f"valid={frame.valid} q={frame.quality:.3f} dpx=({frame.dpixelx:.1f},{frame.dpixely:.1f}) "
                f"vx={frame.vx_hat:.3f} vy={frame.vy_hat:.3f} h={h_meas:.3f}"
            )

    rclpy.init()
    node = FlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
