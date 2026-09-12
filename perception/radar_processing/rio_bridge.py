#!/usr/bin/env python3
"""rio_bridge.py — wrap radar_processing/RIO.py onto swarm-loc /cf_<id>/rio/delta.

Real RIO (rio.source: real): 2D IMU+Doppler ego-velocity KF — no landmarks,
no absolute pose, no truth reads. This bridge integrates RIO's fused
velocity + IMU yaw into the RioDelta wire rows the EKF consumes (plan §3.2):

  in : /cf_<id>/radar/points  (radarays_gz2, per-drone, x/y/z/intensity/doppler,
                               SIM-time stamps, BEST_EFFORT)
       /cf_<id>/imu           (ros_gz_bridge, orientation for roll/pitch/yaw)
  out: /cf_<id>/rio/delta     (PointCloud2 RIO_DTYPE, 1 row per radar scan)

Contract kept exactly:
  stamp   <f8  sim seconds of the radar scan (advancing)
  dt      <f4  from consecutive radar header stamps
  dp_*    <f4  body-frame position increment (v_body * dt)
  dpsi    <f4  IMU yaw increment over dt
  roll/pitch   absolute, from the IMU orientation
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
import sys
import time
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[2]
_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _REPO / "perception/swarm_loc", _REPO / "perception/uwb_sim"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rio_stub import RioDelta, load_config, resolve_config_path  # noqa: E402
from state import rot_to_rpy, wrap_psi  # noqa: E402
from swarm_msgs import RIO_DTYPE, _to_array, pack_rio, rio_delta_from_row, rio_row_from_delta  # noqa: E402

# 2D RIO gives NO vertical odometry. Advertise the z increment (always 0)
# with an honest uncertainty: the drone's unmodelled vertical velocity. This
# is a bridge model constant (the only axis RIO itself has no covariance
# for), not an estimator tunable.
#
# MEASURED 2026-09-11 (tunnel/triangle_forward, two runs, truth offline —
# never at runtime). The old 0.15 "near-hover nano quad" guess was wrong for
# this platform: these drones take off, bounce and land inside the scored
# window (z sweeps 0.01 -> 1.7 m in ~20 s). Truth |vz| RMS per drone was
# 0.60/0.65/0.66 and 0.38/0.68/0.63 m/s, with 41-57% of 0.1 s steps above
# 0.15 m/s; differencing the RIO increment against truth gives the same
# answer independently (vz error std 0.38-0.69 m/s). So 0.15 understated the
# z channel by ~4x in sigma (~16x in variance) and the EKF's z covariance was
# correspondingly ~14x too small. 0.60 is the measurement, not a tuning knob.
SIGMA_VZ_MPS = 0.60
# RIO's own 2x2 velocity KF covariance is the covariance of its ESTIMATE
# under its own tuned noise model — it knows nothing about Doppler solve
# conditioning, scan-to-scan bias, or the tunnel geometry that makes the
# along-track component nearly unobservable. In steady state it advertises
# sigma_v ~ 0.016 m/s, i.e. ~1.6 mm per 0.1 s step, while the MEASURED
# per-axis horizontal velocity error over the same runs was 0.02-1.30 m/s
# (pooled RMS 0.56 m/s) and the dead-reckoned horizontal drift was 0.8-1.4 m
# over ~21 s against an advertised random walk of ~0.02 m. Passing that P
# straight through is what made the filter overconfident by ~35-60x in sigma.
# Floor the body-frame horizontal velocity variance at the pooled measured
# level: adding a non-negative diagonal to a PSD matrix keeps it PSD, and the
# floor only ever widens what RIO claims — it never shrinks it.
SIGMA_V_XY_FLOOR_MPS = 0.50
# Per-second yaw-increment noise of the IMU-orientation-differenced dpsi.
# Measured per-step dpsi error / advertised sigma was 0.1-1.6 (run A) and
# 0.0-8.1 (run B); the median drone matches 0.5 deg/sqrt(s) but the worst
# drone needs ~4 deg/sqrt(s). Left unchanged deliberately: the spread is
# drone-specific (see the cf_2 yaw investigation in §7) and picking a number
# in the middle would be a guess, not a measurement. Position NEES does not
# depend on it.
SIGMA_DPSI_RAD_PER_SQRT_S = math.radians(0.5)
# Doppler velocity is metric; scale error is negligible by construction.
SCALE_VAR = 1e-8
_COV_FLOOR = 1e-8


def _yaw_from_R(R: np.ndarray) -> float:
    yaw, _, _ = rot_to_rpy(np.asarray(R, dtype=np.float64))
    return float(yaw)


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
    d3 = delta_from_velocity(
        2.0, 0.1, np.array([0.0, 0.4]), R90, 0.0, 0.0, np.diag([0.90, 0.36]), True
    )
    check("3 yawed dp body x", abs(d3.delta_p_body[0] - 0.04) < 1e-9, str(d3.delta_p_body))
    check("3b yawed dp body y ~0", abs(d3.delta_p_body[1]) < 1e-9)

    # 3c — covariance rotates with the frame: world var_x=0.90 lands on body y
    # (body x picks up world var_y=0.36). Scaled by dt².
    check(
        "3c cov rotated w->b",
        abs(d3.cov[0, 0] - 0.36 * 0.1**2) < 1e-12
        and abs(d3.cov[1, 1] - 0.90 * 0.1**2) < 1e-12,
        f"c00={d3.cov[0,0]:.3e} c11={d3.cov[1,1]:.3e}",
    )
    check("3d cov z honest (no z odometry)",
          abs(d3.cov[2, 2] - (SIGMA_VZ_MPS * 0.1) ** 2) < 1e-12)
    check("3e cov symmetric finite",
          np.allclose(d3.cov, d3.cov.T) and np.all(np.isfinite(d3.cov)))
    check("3f cov PSD", np.all(np.linalg.eigvalsh(d3.cov) >= 0.0))

    # 4 — KF covariance is passed through, not a config constant: doubling
    # P_vel doubles the advertised dp covariance (both values above the floor).
    d4a = delta_from_velocity(1.0, 0.1, np.zeros(2), R, 0.0, 0.0, np.eye(2) * 0.36, True)
    d4b = delta_from_velocity(1.0, 0.1, np.zeros(2), R, 0.0, 0.0, np.eye(2) * 0.72, True)
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

    # 9 — the measured z sigma. Pin the constant itself so a silent revert to
    # the old 0.15 "near-hover" guess fails here, and pin that the advertised
    # per-step z sigma now brackets the measured 0.038–0.069 m increment error.
    check("9 SIGMA_VZ_MPS is the measured value", abs(SIGMA_VZ_MPS - 0.60) < 1e-12)
    sig_dz = SIGMA_VZ_MPS * dt8
    check("9a advertised z sigma covers measured per-step error",
          0.060 <= sig_dz <= 0.12, f"sigma_dz={sig_dz:.4f}")
    check("9b advertised xy sigma covers measured per-step error",
          0.030 <= SIGMA_V_XY_FLOOR_MPS * dt8 <= 0.12,
          f"sigma_dxy={SIGMA_V_XY_FLOOR_MPS*dt8:.4f}")
    # a 20 s dead-reckoning window: advertised random walk must be the same
    # order as the 0.8–1.4 m horizontal drift actually measured, not 0.02 m.
    n_steps = int(round(20.0 / dt8))
    rw = SIGMA_V_XY_FLOOR_MPS * dt8 * math.sqrt(n_steps)
    check("9c 20 s advertised drift is the measured order", 0.4 <= rw <= 2.0, f"rw={rw:.3f} m")

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

    import rclpy
    import RIO

    class SwarmRioNode(RIO.ImuRadarFusionNode):
        def __init__(self, drone_id: int):
            super().__init__(node_name=f"imu_radar_fusion_{drone_id}")
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

    # Leave odom_topic at RIO.py default '' — this bridge must never read
    # Gazebo truth (it replaces the truth-reading stub).
    ros_args = [
        "--ros-args",
        "-p",
        f"imu_topic:=/cf_{cf_id}/imu",
        "-p",
        f"radar_topic:=/cf_{cf_id}/radar/points",
    ]
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
