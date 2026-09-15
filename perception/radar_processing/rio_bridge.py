#!/usr/bin/env python3
"""rio_bridge.py — wrap radar_processing/RIO.py onto swarm-loc /cf_<id>/rio/delta.

Real RIO (rio.source: real): 2D IMU+Doppler ego-velocity KF — no landmarks,
no absolute pose, no truth reads. This bridge integrates RIO's fused
velocity + IMU yaw into the RioDelta wire rows the EKF consumes (plan §3.2):

  in : /cf_<id>/radar/points  (radarays_gz2, per-drone, x/y/z/intensity/doppler,
                               SIM-time stamps, BEST_EFFORT)
       /cf_<id>/imu           (ros_gz_bridge; ONLY angular_velocity, linear_acceleration
                               and header.stamp are read — attitude is ESTIMATED by
                               attitude_filter.py (Madgwick IMU), never taken from
                               Gazebo's ground-truth Imu.orientation, unless
                               rio.attitude.source: gazebo_truth (SIM-ORACLE ablation))
  out: /cf_<id>/rio/delta     (PointCloud2 RIO_DTYPE, 1 row per radar scan)

Contract kept exactly:
  stamp   <f8  sim seconds of the radar scan (advancing)
  dt      <f4  from consecutive radar header stamps
  dp_*    <f4  body-frame position increment (v_body * dt)
  dpsi    <f4  yaw increment over dt of the (estimated) attitude
  roll/pitch   absolute, from the (estimated) attitude
  cov_0..14    upper triangle of the 5×5 [dp(3), dpsi, scale] covariance —
               dp x/y from RIO's own KF covariance (rotated world→body),
               NOT from config constants
  valid   <u4  1 only when the Doppler solve succeeded and was well-conditioned

Staleness is judged by WALL clock (plan §9): if no radar scan arrived for
longer than max_measurement_age_s of wall time, the integration re-anchors
instead of emitting a huge dt row.

    python3 -u perception/radar_processing/rio_bridge.py --cf-id 0
    python3 perception/radar_processing/rio_bridge.py --selftest
"""
from __future__ import annotations

import argparse
import math
import re
import sys
import time
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[2]
_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _REPO / "perception/swarm_loc", _REPO / "perception/uwb_sim"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from attitude_filter import (  # noqa: E402
    AttitudeConfigError,
    MadgwickImuFilter,
    parse_attitude_config,
    quat_from_rpy,
)
from rio_stub import RioDelta, load_config, resolve_config_path  # noqa: E402
from state import rot_to_rpy, wrap_psi  # noqa: E402
from swarm_msgs import RIO_DTYPE, _to_array, pack_rio, rio_delta_from_row, rio_row_from_delta  # noqa: E402

# 2D RIO gives NO vertical odometry. Advertise the z increment (always 0)
# with an honest uncertainty: the drone's unmodelled vertical velocity. This
# is a bridge model constant (the only axis RIO itself has no covariance
# for), not an estimator tunable.
#
# CALIBRATED 2026-09-14 on OPEN-FIELD collinear_shuttle (drones were ~50 m
# outside lava_tube.obj). Redo after the tunnel site is flown; these floors
# make RIO nearly worthless to the EKF and must not back a "swarm vs RIO-only"
# claim. Protocol: eval_scripts/calibrate_rio_covariance.py
# --calibrate out/swarm_loc_logs/tunnel/collinear_shuttle
# --out out/rio_cov_calibration/collinear_shuttle.json --min-valid-rows 150
# (default 200 refused: 10 Hz RIO × ~18.5 s score window yields 183/188/186
# valid rows; windows@1s = 20/21/20 ≥ min 10). Rule = 1 s window, max over
# drones and axes of debiased sigma_eff. Report path above. Bias is not
# absorbed (cf_1 body y −0.95 m/s, cf_1 x +0.46, cf_2 x +0.52).
SIGMA_VZ_MPS = 0.7728129088797581  # cf_1 body z @ 1 s
# RIO's own 2x2 velocity KF covariance is the covariance of its ESTIMATE
# under its own tuned noise model — it knows nothing about Doppler solve
# conditioning, scan-to-scan bias, or geometry. Floor the body-frame
# horizontal velocity variance at the calibrated 1 s sigma_eff. Adding a
# non-negative diagonal to a PSD matrix keeps it PSD; the floor only ever
# widens what RIO claims — it never shrinks it.
SIGMA_V_XY_FLOOR_MPS = 1.2670450592233324  # cf_1 body y @ 1 s
# Per-second yaw-increment noise of the IMU-orientation-differenced dpsi.
# Same 1 s max-over-drones rule as the translation floors (cf_2 heading).
SIGMA_DPSI_RAD_PER_SQRT_S = 0.003411593210792272  # 0.1955 deg/sqrt(s)
# Doppler velocity is metric; scale error is negligible by construction.
SCALE_VAR = 1e-8
_COV_FLOOR = 1e-8


# --- RIO ROS parameters from cfg["rio"]["ros_params"] -----------------------
# Whitelist + declared type. MIRRORS the declare_parameter calls in
# RIO.ImuRadarFusionNode.__init__ (RIO.py) — the type is that of RIO.py's
# declared default, because ROS 2 rejects an override whose type differs
# (a double param refuses `1`, an int param refuses `8.0`). Update this table
# if RIO.py adds or retypes a parameter.
RIO_PARAM_TYPES: dict[str, str] = {
    "world_frame_id": "string",
    "intensity_field": "string",
    "doppler_field": "string",
    "radar_extrinsic_quat_xyzw": "double_array",
    "radar_intensity_threshold": "double",
    "radar_denoise": "bool",
    "radar_denoise_radius": "double",
    "radar_denoise_min_neighbors": "int",
    "max_range": "double",
    "doppler_sign": "double",
    "doppler_min_points": "int",
    "doppler_max_condition_number": "double",
    "doppler_residual_threshold_scale": "double",
    "doppler_intensity_weighting": "bool",
    "doppler_outlier_reject": "bool",
    "imu_process_noise_std": "double",
    "imu_measurement_noise_std": "double",
    "imu_use_residual_as_noise": "bool",
    "imu_residual_noise_scale": "double",
    "imu_residual_noise_floor": "double",
    "imu_remove_gravity": "bool",
    "imu_gravity_mag": "double",
    "imu_gravity_sign": "double",
}
# Declared by RIO.py but never settable from rio.ros_params:
#   odom_topic  — RIO's ground-truth benchmarking input (truth wall, plan §3.3)
#   imu_topic / radar_topic — owned by this bridge (per-drone /cf_<id>/...)
RIO_PARAM_FORBIDDEN: dict[str, str] = {
    "odom_topic": "ground-truth benchmarking input; the estimator must never read truth",
    "imu_topic": "set by rio_bridge per drone (/cf_<id>/imu)",
    "radar_topic": "set by rio_bridge per drone (/cf_<id>/radar/points)",
}
_STRING_PARAM_OK = re.compile(r"^[A-Za-z_/][A-Za-z0-9_/.\-]*$")
_YAML_BOOLISH = {"true", "false", "yes", "no", "on", "off", "null", "~"}


class RioParamError(ValueError):
    """Bad rio.ros_params entry (unknown key, forbidden key, wrong type)."""


def _fmt_double(name: str, v) -> str:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise RioParamError(
            f"rio.ros_params.{name}: expected a number (double), got {v!r} "
            f"({type(v).__name__}); note PyYAML reads '1e-3' as a string — write 1.0e-3")
    f = float(v)
    if not math.isfinite(f):
        raise RioParamError(f"rio.ros_params.{name}: non-finite value {v!r}")
    s = repr(f)  # shortest round-trip: '1.0', '0.0404', '1e-05'
    mant, sep, exp = s.partition("e")
    if "." not in mant:
        mant += ".0"  # '1e-05' -> '1.0e-05' (never mistakable for an int)
    return mant + sep + exp


def _fmt_rio_param(name: str, v) -> str:
    kind = RIO_PARAM_TYPES[name]
    if kind == "double":
        return _fmt_double(name, v)
    if kind == "int":
        if isinstance(v, bool) or not isinstance(v, int):
            raise RioParamError(
                f"rio.ros_params.{name}: RIO.py declares an int; got {v!r} "
                f"({type(v).__name__}) — ROS 2 rejects a double for an int parameter")
        return str(int(v))
    if kind == "bool":
        if not isinstance(v, bool):
            raise RioParamError(
                f"rio.ros_params.{name}: RIO.py declares a bool; got {v!r} ({type(v).__name__})")
        return "true" if v else "false"
    if kind == "double_array":
        if not isinstance(v, (list, tuple)) or not v:
            raise RioParamError(f"rio.ros_params.{name}: expected a non-empty list of numbers, got {v!r}")
        return "[" + ", ".join(_fmt_double(name, x) for x in v) + "]"
    if kind == "string":
        if (not isinstance(v, str) or not _STRING_PARAM_OK.match(v)
                or v.lower() in _YAML_BOOLISH):
            raise RioParamError(
                f"rio.ros_params.{name}: expected a plain identifier-like string, got {v!r}")
        return v
    raise RioParamError(f"rio.ros_params.{name}: unhandled declared type {kind!r}")  # pragma: no cover


def rio_ros_param_args(cfg: dict, cf_id: int) -> tuple[list[str], dict[str, str]]:
    """Pure (no rclpy): build RIO's `--ros-args` list from the config.

    Always the two per-drone topic params; then one `-p name:=value` per entry
    of cfg["rio"]["ros_params"] (sorted by name), formatted to RIO.py's
    declared type. Absent/empty block -> topics only (RIO.py defaults).
    Raises RioParamError on an unknown, forbidden or mistyped key.
    Returns (ros_args, applied {name: formatted value}).
    """
    args = [
        "--ros-args",
        "-p", f"imu_topic:=/cf_{cf_id}/imu",
        "-p", f"radar_topic:=/cf_{cf_id}/radar/points",
    ]
    block = ((cfg or {}).get("rio") or {}).get("ros_params")
    if block is None:
        return args, {}
    if not isinstance(block, dict):
        raise RioParamError(f"rio.ros_params must be a mapping, got {type(block).__name__}")
    applied: dict[str, str] = {}
    for name in sorted(block):
        if name in RIO_PARAM_FORBIDDEN:
            raise RioParamError(
                f"rio.ros_params.{name} is not allowed: {RIO_PARAM_FORBIDDEN[name]}")
        if name not in RIO_PARAM_TYPES:
            raise RioParamError(
                f"rio.ros_params.{name}: unknown RIO parameter (typo?). "
                f"Allowed: {', '.join(sorted(RIO_PARAM_TYPES))}")
        applied[name] = _fmt_rio_param(name, block[name])
        args += ["-p", f"{name}:={applied[name]}"]
    return args, applied


def _yaw_from_R(R: np.ndarray) -> float:
    yaw, _, _ = rot_to_rpy(np.asarray(R, dtype=np.float64))
    return float(yaw)


def feed_attitude_filter(filt: MadgwickImuFilter, msg) -> bool:
    """Feed one sensor_msgs/Imu (or duck-typed equivalent) to the filter.

    Reads ONLY msg.header.stamp, msg.angular_velocity and
    msg.linear_acceleration — never msg.orientation (ground truth in Gazebo)
    nor msg.orientation_covariance. Returns filt.ready afterwards.
    """
    w = msg.angular_velocity
    a = msg.linear_acceleration
    filt.update(msg.header.stamp, (w.x, w.y, w.z), (a.x, a.y, a.z))
    return bool(filt.ready)


def imu_msg_with_orientation(msg, q_xyzw, orientation_cov_rad2: float):
    """Pure helper: a NEW message of type(msg) carrying msg's header, gyro and
    accel (+ their covariances) and the given body->world quaternion.

    msg.orientation / msg.orientation_covariance are never read: the output is
    built from a fresh type(msg)() and only the non-attitude fields are copied,
    so no ground-truth value can leak through. orientation_covariance is set
    to diag(orientation_cov_rad2) (>= 0, so RIO.py's `== -1` "not available"
    check passes).
    """
    import copy

    c = float(orientation_cov_rad2)
    if not (math.isfinite(c) and c >= 0.0):
        raise ValueError(f"orientation_cov_rad2 must be finite and >= 0, got {orientation_cov_rad2!r}")
    q = [float(v) for v in q_xyzw]
    out = type(msg)()
    out.header = copy.deepcopy(msg.header)
    out.angular_velocity = copy.deepcopy(msg.angular_velocity)
    out.linear_acceleration = copy.deepcopy(msg.linear_acceleration)
    out.angular_velocity_covariance = copy.deepcopy(msg.angular_velocity_covariance)
    out.linear_acceleration_covariance = copy.deepcopy(msg.linear_acceleration_covariance)
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q
    cov = [c, 0.0, 0.0, 0.0, c, 0.0, 0.0, 0.0, c]
    for i in range(9):
        out.orientation_covariance[i] = cov[i]
    return out


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    from attitude_filter import quat_to_matrix
    return quat_to_matrix(quat_from_rpy(roll, pitch, yaw))


def attitude_mode(cfg: dict):
    """cfg -> AttitudeConfig. Raises AttitudeConfigError on a missing
    rio.attitude block, an unknown source or an incomplete estimated block."""
    return parse_attitude_config(cfg)


def imu_for_rio(att_cfg, filt, msg):
    """Pure decision (no rclpy): what RIO's _imu_callback should receive.

    gazebo_truth -> msg unchanged (the same object; SIM-ORACLE ablation).
    estimated    -> feed the filter (gyro/accel/stamp only); None while not
                    ready (RIO waits), else a copy carrying the filter attitude.
    """
    if att_cfg.source == "gazebo_truth":
        return msg
    if att_cfg.source != "estimated":
        raise AttitudeConfigError(f"rio.attitude.source={att_cfg.source!r} unknown")
    if not feed_attitude_filter(filt, msg):
        return None
    return imu_msg_with_orientation(msg, filt.quaternion_xyzw, att_cfg.orientation_cov_rad2)


def is_stale(now_wall: float, prev_wall: float, stale_s: float) -> bool:
    """Wall-clock gap check (plan §9: never judge staleness on sim stamps)."""
    return (float(now_wall) - float(prev_wall)) > float(stale_s)


def delta_from_velocity(
    stamp: float,
    dt: float,
    vel_world_xy: np.ndarray,
    R_world_from_body: np.ndarray,
    yaw_now: float,
    yaw_prev: float,
    P_vel: np.ndarray,
    doppler_ok: bool,
) -> RioDelta:
    """v_world (vx,vy) → body Δp; Δψ from IMU yaw; vz unobserved (2D RIO).

    P_vel is RIO's own 2×2 [vx,vy] world-frame KF covariance — passed
    through (rotated into the body frame, scaled by dt²), never replaced by
    config constants, but FLOORED at SIGMA_V_XY_FLOOR_MPS² so the advertised
    number cannot fall below the error RIO actually makes (see the constant).
    """
    dt = float(max(dt, 0.0))
    R = np.asarray(R_world_from_body, dtype=np.float64)
    v_w = np.array([float(vel_world_xy[0]), float(vel_world_xy[1]), 0.0], dtype=np.float64)
    v_body = R.T @ v_w
    dp = v_body * dt
    dpsi = wrap_psi(float(yaw_now) - float(yaw_prev)) if dt > 0 else 0.0
    _, pitch, roll = rot_to_rpy(R)

    # world-frame 3×3 velocity covariance → body frame → position increment
    P2 = np.asarray(P_vel, dtype=np.float64)[:2, :2]
    C_w = np.zeros((3, 3), dtype=np.float64)
    C_w[:2, :2] = 0.5 * (P2 + P2.T)
    # Honesty floor on the horizontal channel. An isotropic diagonal floor is
    # rotation-invariant, so flooring here (world) or after the rotation
    # (body) is the same matrix; doing it here keeps one code path. max() on
    # the diagonal of a PSD matrix adds a non-negative diagonal, so the result
    # stays PSD and is never smaller than what RIO advertised.
    v_floor = SIGMA_V_XY_FLOOR_MPS**2
    C_w[0, 0] = max(C_w[0, 0], v_floor)
    C_w[1, 1] = max(C_w[1, 1], v_floor)
    C_w[2, 2] = SIGMA_VZ_MPS**2
    C_b = R.T @ C_w @ R

    cov = np.zeros((5, 5), dtype=np.float64)
    cov[0:3, 0:3] = C_b * dt * dt
    cov[3, 3] = (SIGMA_DPSI_RAD_PER_SQRT_S**2) * max(dt, 1e-3)
    cov[4, 4] = SCALE_VAR
    for k in range(5):
        cov[k, k] = max(cov[k, k], _COV_FLOOR)

    return RioDelta(
        stamp=float(stamp),
        dt=dt,
        delta_p_body=dp,
        delta_psi=dpsi,
        roll=float(roll),
        pitch=float(pitch),
        cov=cov,
        valid=bool(doppler_ok and dt > 0),
    )


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

    # 1 — identity attitude: body == world
    R = np.eye(3)
    d = delta_from_velocity(
        1.0, 0.2, np.array([0.5, 0.0]), R, 0.1, 0.0, np.eye(2) * 0.04, True
    )
    check("1 dp x = vx*dt", abs(d.delta_p_body[0] - 0.1) < 1e-9, str(d.delta_p_body))
    check("1b dp y ~0", abs(d.delta_p_body[1]) < 1e-9)
    check("1c dp z 0", abs(d.delta_p_body[2]) < 1e-12)
    check("1d dpsi", abs(d.delta_psi - 0.1) < 1e-9)
    check("1e valid", d.valid)
    check("1f stamp passthrough", abs(d.stamp - 1.0) < 1e-12)
    check("1g dt passthrough", abs(d.dt - 0.2) < 1e-12)

    # 2 — solve failure → valid=False (row still packable / publishable)
    d2 = delta_from_velocity(
        1.0, 0.2, np.array([0.5, 0.0]), R, 0.0, 0.0, np.eye(2), False
    )
    check("2 invalid without doppler", not d2.valid)
    check("2b invalid at dt=0", not delta_from_velocity(
        1.0, 0.0, np.array([0.5, 0.0]), R, 0.0, 0.0, np.eye(2), True).valid)

    # 3 — yawed frame: world +Y velocity is body +X at yaw=+90°
    cy, sy = math.cos(math.pi / 2), math.sin(math.pi / 2)
    R90 = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    # P_vel here is deliberately ABOVE SIGMA_V_XY_FLOOR_MPS² on both axes, so
    # 3c/3c2 test the frame rotation and not the floor (8c/8d cover the floor).
    var_wx = 4.0 * SIGMA_V_XY_FLOOR_MPS ** 2  # world x → body y at +90°
    var_wy = 2.0 * SIGMA_V_XY_FLOOR_MPS ** 2  # world y → body x at +90°
    d3 = delta_from_velocity(
        2.0, 0.1, np.array([0.0, 0.4]), R90, 0.0, 0.0, np.diag([var_wx, var_wy]), True
    )
    check("3 yawed dp body x", abs(d3.delta_p_body[0] - 0.04) < 1e-9, str(d3.delta_p_body))
    check("3b yawed dp body y ~0", abs(d3.delta_p_body[1]) < 1e-9)

    # 3c — covariance rotates with the frame: world var_x lands on body y
    # (body x picks up world var_y). Scaled by dt².
    check(
        "3c cov rotated w->b",
        abs(d3.cov[0, 0] - var_wy * 0.1**2) < 1e-12
        and abs(d3.cov[1, 1] - var_wx * 0.1**2) < 1e-12,
        f"c00={d3.cov[0,0]:.3e} c11={d3.cov[1,1]:.3e}",
    )
    check("3d cov z honest (no z odometry)",
          abs(d3.cov[2, 2] - (SIGMA_VZ_MPS * 0.1) ** 2) < 1e-12)
    check("3e cov symmetric finite",
          np.allclose(d3.cov, d3.cov.T) and np.all(np.isfinite(d3.cov)))
    check("3f cov PSD", np.all(np.linalg.eigvalsh(d3.cov) >= 0.0))

    # 4 — KF covariance is passed through, not a config constant: doubling
    # P_vel doubles the advertised dp covariance (both values above the floor).
    d4a = delta_from_velocity(
        1.0, 0.1, np.zeros(2), R, 0.0, 0.0, np.eye(2) * (2.0 * SIGMA_V_XY_FLOOR_MPS**2), True)
    d4b = delta_from_velocity(
        1.0, 0.1, np.zeros(2), R, 0.0, 0.0, np.eye(2) * (4.0 * SIGMA_V_XY_FLOOR_MPS**2), True)
    check("4 cov tracks RIO P", abs(d4b.cov[0, 0] / d4a.cov[0, 0] - 2.0) < 1e-9)

    # 5 — dpsi wraps
    d5 = delta_from_velocity(1.0, 0.1, np.zeros(2), R, math.pi - 0.05, -math.pi + 0.05, np.eye(2), True)
    check("5 dpsi wrapped", abs(d5.delta_psi + 0.1) < 1e-9, f"dpsi={d5.delta_psi}")

    # 6 — RIO_DTYPE wire roundtrip (no rclpy: numpy pack only)
    arr = _to_array([rio_row_from_delta(d)], RIO_DTYPE)
    check("6 wire itemsize 100", arr.dtype.itemsize == 100)
    back = rio_delta_from_row(arr[0])
    check("6b roundtrip stamp/dt", abs(back.stamp - d.stamp) < 1e-9 and abs(back.dt - d.dt) < 1e-6)
    check("6c roundtrip dp", np.allclose(back.delta_p_body, d.delta_p_body, atol=1e-6))
    check("6d roundtrip cov", np.allclose(back.cov, d.cov, atol=1e-6))
    check("6e roundtrip valid", back.valid is True)
    arr2 = _to_array([rio_row_from_delta(d2)], RIO_DTYPE)
    check("6f invalid row valid=0", int(arr2[0]["valid"]) == 0)

    # 7 — wall-clock staleness helper
    check("7 fresh not stale", not is_stale(10.2, 10.0, 0.5))
    check("7b gap stale", is_stale(10.6, 10.0, 0.5))
    check("7c boundary not stale", not is_stale(10.5, 10.0, 0.5))

    # 8 — honesty floor on the horizontal channel (BUG B5). RIO's steady-state
    # KF P is ~2.5e-4 (sigma_v ~0.016 m/s); the measured per-axis velocity
    # error is 0.02–1.30 m/s. The floor makes the advertised number the larger
    # of the two and must never shrink what RIO claims.
    dt8 = 0.101
    tiny = np.eye(2) * 2.5e-4                      # RIO's real steady-state P
    d8 = delta_from_velocity(1.0, dt8, np.zeros(2), R, 0.0, 0.0, tiny, True)
    want = (SIGMA_V_XY_FLOOR_MPS * dt8) ** 2
    check("8 tiny RIO P floored", abs(d8.cov[0, 0] - want) < 1e-15 and abs(d8.cov[1, 1] - want) < 1e-15,
          f"c00={d8.cov[0,0]:.4e} want={want:.4e}")
    sig_unfloored = math.sqrt(2.5e-4) * dt8
    check("8a floor is the documented overconfidence factor",
          abs(math.sqrt(d8.cov[0, 0]) / sig_unfloored - SIGMA_V_XY_FLOOR_MPS / math.sqrt(2.5e-4)) < 1e-6,
          f"ratio={math.sqrt(d8.cov[0,0])/sig_unfloored:.1f}")
    big = np.eye(2) * (4.0 * SIGMA_V_XY_FLOOR_MPS**2)
    d8b = delta_from_velocity(1.0, dt8, np.zeros(2), R, 0.0, 0.0, big, True)
    check("8b floor never shrinks a larger RIO P",
          abs(d8b.cov[0, 0] - 4.0 * want) < 1e-15, f"c00={d8b.cov[0,0]:.4e}")
    # one axis under, one over: only the under-confident axis moves
    mixed = np.diag([2.5e-4, 4.0 * SIGMA_V_XY_FLOOR_MPS**2])
    d8c = delta_from_velocity(1.0, dt8, np.zeros(2), R, 0.0, 0.0, mixed, True)
    check("8c per-axis floor", abs(d8c.cov[0, 0] - want) < 1e-15 and abs(d8c.cov[1, 1] - 4.0 * want) < 1e-15,
          f"c00={d8c.cov[0,0]:.4e} c11={d8c.cov[1,1]:.4e}")
    # floored covariance stays symmetric / PSD even with a correlated P
    corr = np.array([[2.5e-4, 2.0e-4], [2.0e-4, 2.5e-4]])
    d8d = delta_from_velocity(1.0, dt8, np.zeros(2), R90, 0.0, 0.0, corr, True)
    check("8d floored cov symmetric PSD",
          np.allclose(d8d.cov, d8d.cov.T) and np.all(np.linalg.eigvalsh(d8d.cov) >= -1e-15)
          and np.all(np.isfinite(d8d.cov)))
    check("8e floor is rotation invariant",
          abs(d8d.cov[0, 0] - d8.cov[0, 0]) < 1e-12 or d8d.cov[0, 0] >= want - 1e-15,
          f"c00={d8d.cov[0,0]:.4e}")
    # 8f — the floor must not touch the z or yaw channels
    check("8f floor leaves z channel to SIGMA_VZ_MPS",
          abs(d8.cov[2, 2] - (SIGMA_VZ_MPS * dt8) ** 2) < 1e-15)
    check("8g floor leaves yaw channel alone",
          abs(d8.cov[3, 3] - SIGMA_DPSI_RAD_PER_SQRT_S**2 * dt8) < 1e-15)

    # 9 — pin the 2026-09-14 collinear_shuttle 1 s calibration (report:
    # out/rio_cov_calibration/collinear_shuttle.json). A silent revert to the
    # 2026-09-11 0.60 / 0.50 / 0.5°/√s floors fails here.
    check("9 SIGMA_VZ_MPS is the calibrated 1s max",
          abs(SIGMA_VZ_MPS - 0.7728129088797581) < 1e-12)
    sig_dz = SIGMA_VZ_MPS * dt8
    check("9a advertised z sigma covers calibrated 1s seff",
          0.070 <= sig_dz <= 0.090, f"sigma_dz={sig_dz:.4f}")
    check("9b SIGMA_V_XY_FLOOR_MPS is the calibrated 1s max",
          abs(SIGMA_V_XY_FLOOR_MPS - 1.2670450592233324) < 1e-12
          and 0.10 <= SIGMA_V_XY_FLOOR_MPS * dt8 <= 0.16,
          f"sigma_dxy={SIGMA_V_XY_FLOOR_MPS*dt8:.4f}")
    n_steps = int(round(20.0 / dt8))
    rw = SIGMA_V_XY_FLOOR_MPS * dt8 * math.sqrt(n_steps)
    check("9c 20 s advertised drift is the calibrated order",
          1.5 <= rw <= 2.5, f"rw={rw:.3f} m")
    check("9d SIGMA_DPSI_RAD_PER_SQRT_S is the calibrated 1s max",
          abs(SIGMA_DPSI_RAD_PER_SQRT_S - 0.003411593210792272) < 1e-12)

    # 10 — rio.ros_params -> RIO `--ros-args` (pure, no rclpy)
    topics = ["--ros-args", "-p", "imu_topic:=/cf_2/imu", "-p", "radar_topic:=/cf_2/radar/points"]

    def p_args(a):
        return [a[i + 1] for i in range(len(a) - 1) if a[i] == "-p"]

    def raises(block) -> bool:
        try:
            rio_ros_param_args({"rio": {"ros_params": block}}, 2)
        except RioParamError:
            return True
        return False

    rec = {"imu_process_noise_std": 0.0012964481834306532,
           "imu_residual_noise_floor": 0.0404,
           "imu_measurement_noise_std": 0.4877}
    a10, ap10 = rio_ros_param_args({"rio": {"ros_params": rec}}, 2)
    check("10 recommended values -> -p args",
          p_args(a10)[2:] == ["imu_measurement_noise_std:=0.4877",
                              "imu_process_noise_std:=0.0012964481834306532",
                              "imu_residual_noise_floor:=0.0404"], str(a10))
    check("10a topics still first", a10[:5] == topics, str(a10[:5]))
    check("10b doubles round-trip exactly",
          all(float(ap10[k]) == v for k, v in rec.items()), str(ap10))
    _, ap10c = rio_ros_param_args({"rio": {"ros_params": {
        "imu_gravity_mag": 1, "imu_residual_noise_scale": 1.0, "doppler_sign": -1,
        "imu_process_noise_std": 1e-5}}}, 0)
    check("10c double literals never int-looking",
          ap10c["imu_gravity_mag"] == "1.0" and ap10c["imu_residual_noise_scale"] == "1.0"
          and ap10c["doppler_sign"] == "-1.0" and ap10c["imu_process_noise_std"] == "1.0e-05"
          and float(ap10c["imu_process_noise_std"]) == 1e-5, str(ap10c))
    _, ap10d = rio_ros_param_args({"rio": {"ros_params": {
        "imu_use_residual_as_noise": True, "radar_denoise": False}}}, 0)
    check("10d bool formatting",
          ap10d == {"imu_use_residual_as_noise": "true", "radar_denoise": "false"}, str(ap10d))
    _, ap10e = rio_ros_param_args({"rio": {"ros_params": {"doppler_min_points": 8}}}, 0)
    check("10e int formatting", ap10e == {"doppler_min_points": "8"}, str(ap10e))
    check("10f int param rejects 8.0", raises({"doppler_min_points": 8.0}))
    check("10g double param rejects bool / string",
          raises({"imu_process_noise_std": True}) and raises({"imu_process_noise_std": "1e-3"}))
    check("10h bool param rejects 1", raises({"imu_use_residual_as_noise": 1}))
    _, ap10i = rio_ros_param_args({"rio": {"ros_params": {
        "radar_extrinsic_quat_xyzw": [0, 0.0, 0.5, 1]}}}, 0)
    check("10i float list formatting",
          ap10i == {"radar_extrinsic_quat_xyzw": "[0.0, 0.0, 0.5, 1.0]"}, str(ap10i))
    check("10j unknown key rejected", raises({"imu_process_noise": 0.01}))
    check("10k odom_topic rejected (truth wall)", raises({"odom_topic": "/cf_2/odom"}))
    check("10l topic override rejected",
          raises({"imu_topic": "/imu/data"}) and raises({"radar_topic": "/radar/points"}))
    check("10m non-finite rejected", raises({"imu_process_noise_std": float("nan")}))
    check("10n absent block -> topics only",
          rio_ros_param_args({"rio": {"source": "real"}}, 2) == (topics, {})
          and rio_ros_param_args({}, 2) == (topics, {}))
    check("10o empty / null block -> topics only",
          rio_ros_param_args({"rio": {"ros_params": {}}}, 2) == (topics, {})
          and rio_ros_param_args({"rio": {"ros_params": None}}, 2) == (topics, {}))
    check("10p whitelist excludes the forbidden names",
          not (set(RIO_PARAM_FORBIDDEN) & set(RIO_PARAM_TYPES)))

    # 11 — the shipped estimator config yields exactly the documented values
    try:
        shipped = load_config(str(_REPO / "configs/estimation/swarm_loc.yaml"))
        a11, ap11 = rio_ros_param_args(shipped, 0)
        err11 = ""
    except Exception as e:  # noqa: BLE001
        a11, ap11, err11 = [], {}, repr(e)
    check("11 shipped swarm_loc.yaml rio.ros_params",
          ap11 == {"imu_measurement_noise_std": "0.4877",
                   "imu_process_noise_std": "0.0012964481834306532",
                   "imu_residual_noise_floor": "0.0404",
                   "imu_use_residual_as_noise": "true"}, err11 or str(ap11))
    check("11a shipped config: 4 params after the 2 topics", len(p_args(a11)) == 6, str(a11))

    # 12 — estimated attitude: the Imu.orientation field (Gazebo ground truth)
    # is never read, and RIO receives the filter's quaternion.
    from types import SimpleNamespace as NS

    class FakeImu:
        """Duck-typed sensor_msgs/Imu. poison=True makes any access to the
        orientation fields raise, proving the code path never reads them."""

        def __init__(self, poison: bool = False):
            self._poison = poison
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id="cf_0/imu")
            self.angular_velocity = NS(x=0.0, y=0.0, z=0.0)
            self.linear_acceleration = NS(x=0.0, y=0.0, z=9.81)
            self.angular_velocity_covariance = [0.0] * 9
            self.linear_acceleration_covariance = [0.0] * 9
            self._orientation = NS(x=0.0, y=0.0, z=0.0, w=1.0)
            self._orientation_covariance = [0.0] * 9

        @property
        def orientation(self):
            if self._poison:
                raise AssertionError("orientation (ground truth) was read")
            return self._orientation

        @orientation.setter
        def orientation(self, v):
            self._orientation = v

        @property
        def orientation_covariance(self):
            if self._poison:
                raise AssertionError("orientation_covariance (ground truth) was read")
            return self._orientation_covariance

        @orientation_covariance.setter
        def orientation_covariance(self, v):
            self._orientation_covariance = v

    def fake_msg(t, gyro, acc, wrong_q=None, poison=False):
        m = FakeImu(poison=False)
        m.header.stamp = NS(sec=int(math.floor(t)), nanosec=int(round((t - math.floor(t)) * 1e9)))
        m.angular_velocity = NS(x=gyro[0], y=gyro[1], z=gyro[2])
        m.linear_acceleration = NS(x=acc[0], y=acc[1], z=acc[2])
        if wrong_q is not None:
            m.orientation = NS(x=wrong_q[0], y=wrong_q[1], z=wrong_q[2], w=wrong_q[3])
            m.orientation_covariance = [0.123] * 9
        m._poison = poison
        return m

    try:
        att_cfg = parse_attitude_config(load_config(str(_REPO / "configs/estimation/swarm_loc.yaml")))
        err12 = ""
    except Exception as e:  # noqa: BLE001
        att_cfg, err12 = None, repr(e)
    check("12 shipped rio.attitude parses, source estimated",
          att_cfg is not None and att_cfg.source == "estimated", err12)
    if att_cfg is not None:
        filt = MadgwickImuFilter(att_cfg)
        rng12 = np.random.default_rng(3)
        # roll 4 deg, pitch -2 deg on the ground; poisoned messages (any read
        # of orientation raises) during the whole init window
        R12 = rotation_from_rpy(math.radians(4.0), math.radians(-2.0), 0.0)
        fb = R12.T @ np.array([0.0, 0.0, 9.81])
        rate12 = 300.0
        raised = ""
        ready_seen_early = False
        n_init = int(round((att_cfg.init_window_s + 0.2) * rate12))
        try:
            for k in range(n_init):
                m = fake_msg(10.0 + k / rate12, rng12.normal(0, 0.005, 3) + [0.01, 0.0, -0.01],
                             fb + rng12.normal(0, 0.03, 3), poison=True)
                ready = feed_attitude_filter(filt, m)
                if ready and k < int(att_cfg.init_window_s * rate12) - 1:
                    ready_seen_early = True
        except AssertionError as e:
            raised = str(e)
        check("12a feed path never reads orientation (poisoned msg)", raised == "", raised)
        check("12b not ready before init window", not ready_seen_early)
        check("12c ready after init window", filt.ready)

        # a deliberately WRONG truth quaternion (yaw 180 deg, roll 30 deg)
        wrong = quat_from_rpy(math.radians(30.0), 0.0, math.pi)
        wrong_xyzw = [wrong[1], wrong[2], wrong[3], wrong[0]]
        m_wrong = fake_msg(10.0 + n_init / rate12, (0.01, 0.0, -0.01), fb, wrong_q=wrong_xyzw)
        feed_attitude_filter(filt, m_wrong)
        out = imu_msg_with_orientation(m_wrong, filt.quaternion_xyzw, att_cfg.orientation_cov_rad2)
        q_out = np.array([out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w])
        check("12d output orientation == filter quaternion, not the wrong truth field",
              np.allclose(q_out, filt.quaternion_xyzw, atol=0.0)
              and not np.allclose(np.abs(q_out), np.abs(wrong_xyzw), atol=1e-2), f"{q_out} vs {wrong_xyzw}")
        try:
            from scipy.spatial.transform import Rotation
            R_out = Rotation.from_quat(q_out.tolist()).as_matrix()   # exactly how RIO.py reads it
            _, p_out, r_out = rot_to_rpy(R_out)
            check("12e RIO would see estimated roll/pitch (~4/-2 deg)",
                  abs(math.degrees(r_out) - 4.0) < 0.3 and abs(math.degrees(p_out) + 2.0) < 0.3,
                  f"roll={math.degrees(r_out):.3f} pitch={math.degrees(p_out):.3f}")
        except ImportError:
            check("12e scipy available", False)
        check("12f orientation_covariance valid (not -1), diag from config, truth cov not copied",
              out.orientation_covariance[0] == att_cfg.orientation_cov_rad2
              and out.orientation_covariance[0] >= 0.0
              and list(out.orientation_covariance) == [att_cfg.orientation_cov_rad2, 0, 0, 0,
                                                       att_cfg.orientation_cov_rad2, 0, 0, 0,
                                                       att_cfg.orientation_cov_rad2])
        check("12g gyro/accel/header copied unchanged",
              out.angular_velocity.x == 0.01 and out.linear_acceleration.z == fb[2]
              and out.header.stamp.sec == m_wrong.header.stamp.sec
              and out.header.stamp.nanosec == m_wrong.header.stamp.nanosec
              and out.header.frame_id == "cf_0/imu")
        check("12h input message not mutated",
              m_wrong.orientation.w == wrong_xyzw[3] and m_wrong.orientation_covariance[0] == 0.123)
        m_p = fake_msg(20.0, (0.0, 0.0, 0.0), (0.0, 0.0, 9.81), wrong_q=wrong_xyzw, poison=True)
        try:
            out_p = imu_msg_with_orientation(m_p, filt.quaternion_xyzw, att_cfg.orientation_cov_rad2)
            err12i = "" if np.allclose([out_p.orientation.x, out_p.orientation.y, out_p.orientation.z,
                                        out_p.orientation.w], filt.quaternion_xyzw) else "wrong q"
        except AssertionError as e:
            err12i = str(e)
        check("12i copy helper never reads orientation (poisoned msg)", err12i == "", err12i)
        bad_cov = False
        try:
            imu_msg_with_orientation(m_wrong, filt.quaternion_xyzw, -1.0)
        except ValueError:
            bad_cov = True
        check("12j negative orientation covariance refused", bad_cov)

    # 13 — source selection
    base13 = load_config(str(_REPO / "configs/estimation/swarm_loc.yaml"))

    def att_decision(cfg):
        try:
            return attitude_mode(cfg), ""
        except AttitudeConfigError as e:
            return None, str(e)

    import copy as _copy
    c_truth = _copy.deepcopy(base13)
    c_truth["rio"]["attitude"] = {"source": "gazebo_truth"}
    mode_t, _ = att_decision(c_truth)
    check("13 gazebo_truth selects passthrough", mode_t is not None and mode_t.source == "gazebo_truth")
    m_t = fake_msg(5.0, (0.1, 0.2, 0.3), (0.0, 0.0, 9.81), wrong_q=wrong_xyzw if att_cfg is not None else None)
    check("13a gazebo_truth passthrough forwards the SAME message object",
          imu_for_rio(mode_t, None, m_t) is m_t)
    c_unk = _copy.deepcopy(base13)
    c_unk["rio"]["attitude"]["source"] = "truth"
    mode_u, err_u = att_decision(c_unk)
    check("13b unknown source refused", mode_u is None and "unknown" in err_u, err_u)
    c_miss = _copy.deepcopy(base13)
    del c_miss["rio"]["attitude"]
    mode_m, err_m = att_decision(c_miss)
    check("13c missing rio.attitude refused (no silent truth fallback)",
          mode_m is None and "missing" in err_m, err_m)
    if att_cfg is not None:
        f13 = MadgwickImuFilter(att_cfg)
        check("13d estimated: nothing forwarded to RIO while not ready",
              imu_for_rio(att_cfg, f13, fake_msg(1.0, (0, 0, 0), (0, 0, 9.81), poison=True)) is None)
        fwd = imu_for_rio(att_cfg, filt, fake_msg(10.0 + (n_init + 1) / rate12, (0.01, 0.0, -0.01), fb,
                                                  wrong_q=wrong_xyzw))
        check("13e estimated: forwarded copy carries the filter quaternion",
              fwd is not None and np.allclose([fwd.orientation.x, fwd.orientation.y, fwd.orientation.z,
                                               fwd.orientation.w], filt.quaternion_xyzw))

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def _run_node(cf_id: int, cfg_path: str) -> int:
    cfg = load_config(resolve_config_path(cfg_path))
    if str(cfg["rio"]["source"]) != "real":
        print(
            f"[rio_bridge] refuse: rio.source={cfg['rio']['source']!r} (must be 'real')",
            file=sys.stderr,
        )
        return 2
    stale_s = float(cfg.get("measurements", {}).get("max_measurement_age_s", 0.5))
    # Build (and validate) RIO's parameters BEFORE touching rclpy, so a typo
    # in rio.ros_params exits non-zero instead of silently using a default.
    # odom_topic stays at RIO.py's default '' — this bridge never reads truth.
    try:
        ros_args, applied = rio_ros_param_args(cfg, cf_id)
    except RioParamError as e:
        print(f"[rio_bridge] refuse: {e}", file=sys.stderr)
        return 2
    if applied:
        print("[rio_bridge] RIO ros_params applied: "
              + ", ".join(f"{k}={v}" for k, v in applied.items()), flush=True)
    else:
        print("[rio_bridge] RIO ros_params: none (RIO.py defaults)", flush=True)

    # Attitude source (rio.attitude) — validated before rclpy too. A missing
    # block or unknown source refuses to start: never a silent truth fallback.
    try:
        att_cfg = attitude_mode(cfg)
    except AttitudeConfigError as e:
        print(f"[rio_bridge] refuse: {e}", file=sys.stderr)
        return 2
    if att_cfg.source == "gazebo_truth":
        print("[rio_bridge] " + "!" * 72 + "\n"
              "[rio_bridge] WARNING: rio.attitude.source=gazebo_truth — RIO attitude, dpsi and\n"
              "[rio_bridge] roll/pitch are GROUND TRUTH (Gazebo Imu.orientation). SIM-ORACLE\n"
              "[rio_bridge] ABLATION ONLY — never use for reported results.\n"
              "[rio_bridge] " + "!" * 72, file=sys.stderr, flush=True)
    else:
        print(f"[rio_bridge] attitude: estimated ({att_cfg.algorithm}, beta={att_cfg.beta}, "
              f"init_window_s={att_cfg.init_window_s}, init_yaw={math.degrees(att_cfg.init_yaw_rad)} deg); "
              "Imu.orientation is ignored", flush=True)

    import rclpy
    import RIO

    class SwarmRioNode(RIO.ImuRadarFusionNode):
        def __init__(self, drone_id: int):
            super().__init__(node_name=f"imu_radar_fusion_{drone_id}")
            # set before spin, so before any _imu_callback can fire
            self._att_cfg = att_cfg
            self._att = MadgwickImuFilter(att_cfg) if att_cfg.source == "estimated" else None
            self._att_ready_logged = False
            if att_cfg.source == "gazebo_truth":
                self.get_logger().warning(
                    "rio.attitude.source=gazebo_truth: attitude is GROUND TRUTH "
                    "(SIM-ORACLE ablation, not for reported results)")
            self._prev_stamp = None
            self._prev_yaw = None
            self._prev_wall = None
            from sensor_msgs.msg import PointCloud2
            from rclpy.qos import qos_profile_sensor_data

            self._delta_pub = self.create_publisher(
                PointCloud2, f"/cf_{drone_id}/rio/delta", qos_profile_sensor_data
            )
            self.get_logger().info(
                f"RIO real: /cf_{drone_id}/radar/points + /cf_{drone_id}/imu → "
                f"/cf_{drone_id}/rio/delta (no landmarks, no odom, no truth)"
            )

        def _imu_callback(self, msg):
            fwd = imu_for_rio(self._att_cfg, self._att, msg)
            if fwd is None:
                self.get_logger().info(
                    f"attitude initializing: hold still for {self._att_cfg.init_window_s} s of sim time "
                    f"(restarts={self._att.n_init_restarts}, tilt_rejects={self._att.n_init_tilt_rejects}, gaps={self._att.n_gaps}) — RIO waits",
                    throttle_duration_sec=self._att_cfg.log_throttle_s)
                return
            if self._att is not None and not self._att_ready_logged:
                self._att_ready_logged = True
                from attitude_filter import matrix_to_rpy
                r, p, y = matrix_to_rpy(self._att.rotation_matrix)
                self.get_logger().info(
                    f"attitude initialized: gyro bias={np.round(self._att.gyro_bias, 6).tolist()} rad/s, "
                    f"roll={math.degrees(r):.2f} pitch={math.degrees(p):.2f} yaw={math.degrees(y):.2f} deg, "
                    f"init restarts={self._att.n_init_restarts}, tilt_rejects={self._att.n_init_tilt_rejects}")
            super()._imu_callback(fwd)

        def _publish_twist(self, publisher, vel_xy, stamp):
            super()._publish_twist(publisher, vel_xy, stamp)
            if publisher is not self.fused_pub:
                return
            # fused_pub fires exactly once per radar scan, stamped with the
            # radar header (sim time) — dt below is therefore the interval
            # between consecutive radar scans.
            t = RIO._stamp_to_sec(stamp)
            wall = time.time()
            yaw = _yaw_from_R(self._last_orientation_R)
            if self._prev_stamp is None or is_stale(wall, self._prev_wall, stale_s):
                # first scan, or a wall-clock gap (paused sim / dropped
                # stream): re-anchor instead of integrating across the hole.
                self._prev_stamp = t
                self._prev_yaw = yaw
                self._prev_wall = wall
                return
            dt = t - self._prev_stamp
            if dt <= 0:
                return  # duplicate / non-advancing stamp
            delta = delta_from_velocity(
                t,
                dt,
                vel_xy,
                self._last_orientation_R,
                yaw,
                self._prev_yaw,
                self.kf.covariance,
                doppler_ok=bool(getattr(self, "_last_doppler_ok", False)),
            )
            self._prev_stamp = t
            self._prev_yaw = yaw
            self._prev_wall = wall
            self._delta_pub.publish(pack_rio([rio_row_from_delta(delta)], t, "body"))

    rclpy.init(args=ros_args)
    node = SwarmRioNode(cf_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--cf-id", type=int, default=0)
    ap.add_argument("--config", default="configs/estimation/swarm_loc.yaml")
    args = ap.parse_args()
    if args.selftest:
        raise SystemExit(run_selftest())
    raise SystemExit(_run_node(args.cf_id, args.config))


if __name__ == "__main__":
    main()
