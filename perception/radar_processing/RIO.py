#!/usr/bin/env python3
"""
RIO.py

ROS 2 node for real-time IMU + radar-Doppler ego-velocity fusion in your
Gazebo sim, built on the same linear Kalman filter and clustering-free
Doppler ego-velocity estimator.

-----------------------------------------------------------------------
SUBSCRIBES
-----------------------------------------------------------------------
  <imu_topic>    sensor_msgs/Imu           required
  <radar_topic>  sensor_msgs/PointCloud2   required -- see NOTE below
  <odom_topic>   nav_msgs/Odometry         OPTIONAL, ground truth.
                 Used ONLY to log |err| for benchmarking -- never read
                 inside predict()/update(). Leave the odom_topic
                 parameter empty to disable this entirely.

NOTE ON THE RADAR MESSAGE TYPE
This assumes your radar plugin publishes a PointCloud2 with named
fields 'x', 'y', 'z', plus an intensity field and a doppler field
(names configurable via the intensity_field / doppler_field
parameters, in case yours differ). If your plugin instead publishes a
genuinely custom message type, everything downstream of
parse_radar_pointcloud() is unaffected -- swap that one function out
for a parser matching your .msg and the rest of the node is unchanged
(paste me the .msg definition and I can write that parser directly).

-----------------------------------------------------------------------
PUBLISHES
-----------------------------------------------------------------------
  ~/fused_velocity    geometry_msgs/TwistStamped  IMU+Doppler fused
                                                    ego velocity (world-
                                                    oriented frame, xy)
  ~/doppler_velocity  geometry_msgs/TwistStamped  Doppler-only estimate,
                                                    for comparison

-----------------------------------------------------------------------
WORLD-FRAME ORIENTATION
-----------------------------------------------------------------------
Both the IMU's linear acceleration and the radar points' bearings must
be rotated into a common orientation before fusion (see
doppler_ego_velocity_sim.py's module docstring for why). This node
gets that rotation from the IMU message's own `orientation` field --
i.e. the IMU's onboard attitude estimate, the same way a real AHRS-
equipped IMU would be used. This is intentionally NOT sourced from
ground-truth odometry: that topic (if configured) is read only for
logging comparison error, and is never touched inside predict()/
update().

If your simulated IMU doesn't populate `orientation` (covariance[0]
== -1.0 is the standard ROS "not available" flag), the last known
orientation is reused and a throttled warning is logged -- in that
case you need your own attitude filter (e.g. Madgwick/Mahony)
publishing orientation on the same topic, since this node has no other
way to know attitude.

-----------------------------------------------------------------------
RADAR MOUNTING EXTRINSIC
-----------------------------------------------------------------------
If the radar is not mounted aligned with the IMU/body axes, set
radar_extrinsic_quat_xyzw to the radar->body rotation (e.g. from your
URDF) so radar points get rotated into the body frame before being
rotated into world orientation by the IMU attitude. Defaults to
identity (radar assumed aligned with body axes).

-----------------------------------------------------------------------
EXAMPLE
-----------------------------------------------------------------------
  ros2 run <your_package> RIO.py --ros-args \\
      -p imu_topic:=/drone/imu \\
      -p radar_topic:=/drone/radar/points \\
      -p odom_topic:=/drone/odom_ground_truth \\
      -p imu_process_noise_std:=0.02 \\
      -p imu_residual_noise_scale:=1.0
"""

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


# ---------------------------------------------------------------------------
# Doppler velocity estimator -- unchanged from doppler_ego_velocity_sim.py,
# inlined here so this node has no dependency on that file.
# ---------------------------------------------------------------------------
@dataclass
class DopplerVelocityResult:
    velocity: np.ndarray
    resolved: bool
    well_conditioned: bool
    condition_number: float
    num_points_total: int
    num_points_used: int
    residual_rms: float
    outlier_indices: np.ndarray


def estimate_velocity_doppler(
    points,
    ego_vel_xy=(0.0, 0.0),
    ego_yaw_rate=0.0,
    doppler_col=2,
    intensity_col=None,
    doppler_sign=1.0,
    use_intensity_weighting=True,
    outlier_reject=True,
    max_iterations=5,
    residual_threshold_scale=3.0,
    min_points=2,
    max_condition_number=20.0,
) -> DopplerVelocityResult:
    """Fit one rigid-body 2D velocity (vx, vy) to a set of radar points'
    Doppler readings from a SINGLE scan. Static-scene assumption: with
    ego_vel_xy=(0,0), this solves for the SENSOR's own velocity."""
    pts = np.asarray(points, dtype=np.float64)
    n_total = pts.shape[0]

    if n_total < min_points:
        return DopplerVelocityResult(
            velocity=np.zeros(2), resolved=False, well_conditioned=False,
            condition_number=np.inf, num_points_total=n_total,
            num_points_used=0, residual_rms=np.nan,
            outlier_indices=np.arange(n_total),
        )

    xy = pts[:, :2]
    doppler = doppler_sign * pts[:, doppler_col]

    r = np.linalg.norm(xy, axis=1)
    r_safe = np.where(r > 1e-6, r, 1e-6)
    r_hat = xy / r_safe[:, None]

    ego_vel_xy = np.asarray(ego_vel_xy, dtype=np.float64)
    v_sensor_at_point = ego_vel_xy[None, :] + ego_yaw_rate * np.stack(
        [-xy[:, 1], xy[:, 0]], axis=1)
    sensor_radial = np.sum(v_sensor_at_point * r_hat, axis=1)

    target_radial = doppler + sensor_radial

    theta = np.arctan2(xy[:, 1], xy[:, 0])
    A_full = np.stack([np.cos(theta), np.sin(theta)], axis=1)
    b_full = target_radial

    if use_intensity_weighting and intensity_col is not None:
        intensity = pts[:, intensity_col].astype(np.float64)
        span = intensity.max() - intensity.min()
        weights_full = ((intensity - intensity.min()) / span) if span > 1e-9 \
            else np.ones(n_total)
        weights_full = 0.1 + 0.9 * weights_full
    else:
        weights_full = np.ones(n_total)

    active = np.ones(n_total, dtype=bool)

    def _weighted_fit(mask):
        A = A_full[mask]
        b = b_full[mask]
        w = weights_full[mask]
        sw = np.sqrt(w)
        A_w = A * sw[:, None]
        b_w = b * sw
        v_xy, *_ = np.linalg.lstsq(A_w, b_w, rcond=None)
        singular_values = np.linalg.svd(A_w, compute_uv=False)
        cond = (singular_values[0] / singular_values[-1]
                if singular_values[-1] > 1e-9 else np.inf)
        residuals = A_full @ v_xy - b_full
        return v_xy, cond, residuals

    v_xy, cond, residuals = _weighted_fit(active)

    if outlier_reject:
        for _ in range(max_iterations):
            active_residuals = residuals[active]
            med = np.median(active_residuals)
            mad = np.median(np.abs(active_residuals - med))
            robust_std = 1.4826 * mad if mad > 1e-9 else np.std(active_residuals) + 1e-9
            new_active = np.abs(residuals - med) <= residual_threshold_scale * robust_std
            if new_active.sum() < min_points:
                break
            if np.array_equal(new_active, active):
                break
            active = new_active
            v_xy, cond, residuals = _weighted_fit(active)

    used_residuals = residuals[active]
    residual_rms = float(np.sqrt(np.mean(used_residuals ** 2))) if active.any() else np.nan
    outlier_indices = np.where(~active)[0]

    return DopplerVelocityResult(
        velocity=v_xy, resolved=True,
        well_conditioned=bool(cond <= max_condition_number),
        condition_number=float(cond),
        num_points_total=n_total, num_points_used=int(active.sum()),
        residual_rms=residual_rms, outlier_indices=outlier_indices,
    )


def denoise_scattered_points(pcl, radius=0.5, min_neighbors=3):
    """Radius outlier removal: a point survives only if at least
    min_neighbors OTHER points fall within radius (meters, XY) of it."""
    if pcl.shape[0] <= 1:
        return pcl
    xy = pcl[:, :2]
    tree = cKDTree(xy)
    counts = tree.query_ball_point(xy, r=radius, return_length=True)
    keep = (counts - 1) >= min_neighbors
    return pcl[keep]


# ---------------------------------------------------------------------------
# Linear Kalman filter -- unchanged from doppler_ego_velocity_sim.py.
# ---------------------------------------------------------------------------
class ImuRadarVelocityKF:
    """Linear Kalman filter fusing IMU acceleration with radar Doppler
    ego-velocity measurements to estimate 2D ego-velocity [vx, vy]."""

    def __init__(self, initial_velocity=(0.0, 0.0), initial_covariance=None,
                 process_noise_std=0.5, measurement_noise_std=0.2):
        self.x = np.asarray(initial_velocity, dtype=np.float64).reshape(2, 1)
        self.P = (np.eye(2, dtype=np.float64) * 1.0 if initial_covariance is None
                  else np.asarray(initial_covariance, dtype=np.float64).copy())
        self.F = np.eye(2, dtype=np.float64)
        self.H = np.eye(2, dtype=np.float64)
        self.process_noise_std = process_noise_std
        self.measurement_noise_std = measurement_noise_std
        self.last_innovation = np.zeros(2)
        self.last_kalman_gain = np.zeros((2, 2))

    def predict(self, accel_xy, dt):
        accel_xy = np.asarray(accel_xy, dtype=np.float64).reshape(2, 1)
        if dt <= 0:
            return self.x.flatten()
        B = np.eye(2, dtype=np.float64) * dt
        self.x = self.F @ self.x + B @ accel_xy
        q = (self.process_noise_std ** 2) * dt
        Q = np.eye(2, dtype=np.float64) * q
        self.P = self.F @ self.P @ self.F.T + Q
        return self.x.flatten()

    def update(self, radar_vel_xy, measurement_noise_std=None):
        z = np.asarray(radar_vel_xy, dtype=np.float64).reshape(2, 1)
        r_std = (measurement_noise_std if measurement_noise_std is not None
                 else self.measurement_noise_std)
        R = np.eye(2, dtype=np.float64) * (r_std ** 2)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(2, dtype=np.float64)
        self.P = (I - K @ self.H) @ self.P @ (I - K @ self.H).T + K @ R @ K.T
        self.last_innovation = y.flatten()
        self.last_kalman_gain = K
        return self.x.flatten()

    @property
    def velocity(self):
        return self.x.flatten()

    @property
    def covariance(self):
        return self.P.copy()


def parse_radar_pointcloud(msg: PointCloud2, intensity_field: str, doppler_field: str):
    """Reads a PointCloud2 into a plain (N,5) float array [x,y,z,intensity,
    doppler]. Returns None on parse failure (e.g. field names don't
    match) so the caller can skip the scan and log once, rather than
    crashing the node on a single bad message."""
    field_names = ('x', 'y', 'z', intensity_field, doppler_field)
    try:
        pts_struct = point_cloud2.read_points(msg, field_names=field_names, skip_nans=True)
    except Exception:
        return None
    if pts_struct.size == 0:
        return np.zeros((0, 5), dtype=np.float64)
    # read_points returns a structured array; stack named fields into a
    # plain float array so downstream code (shared with the offline sim)
    # can index by column position.
    return np.stack([pts_struct[name].astype(np.float64) for name in field_names], axis=1)


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class ImuRadarFusionNode(Node):
    def __init__(self, node_name: str = "imu_radar_fusion_node"):
        super().__init__(node_name)

        # ---- topics ----
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('radar_topic', '/radar/points')
        self.declare_parameter('odom_topic', '')  # empty = benchmarking disabled
        self.declare_parameter('world_frame_id', 'odom')

        # ---- radar message parsing ----
        self.declare_parameter('intensity_field', 'intensity')
        self.declare_parameter('doppler_field', 'doppler')
        # radar->body extrinsic rotation, e.g. from your URDF (identity default)
        self.declare_parameter('radar_extrinsic_quat_xyzw', [0.0, 0.0, 0.0, 1.0])

        # ---- radar cleanup (same knobs as doppler_ego_velocity_sim.py) ----
        self.declare_parameter('radar_intensity_threshold', 0.0)
        self.declare_parameter('radar_denoise', False)
        self.declare_parameter('radar_denoise_radius', 0.5)
        self.declare_parameter('radar_denoise_min_neighbors', 3)
        self.declare_parameter('max_range', 20.0)

        # ---- Doppler fit ----
        self.declare_parameter('doppler_sign', 1.0)
        self.declare_parameter('doppler_min_points', 8)
        self.declare_parameter('doppler_max_condition_number', 20.0)
        self.declare_parameter('doppler_residual_threshold_scale', 3.0)
        self.declare_parameter('doppler_intensity_weighting', True)
        self.declare_parameter('doppler_outlier_reject', True)

        # ---- Kalman filter ----
        self.declare_parameter('imu_process_noise_std', 0.05)
        self.declare_parameter('imu_measurement_noise_std', 0.2)
        self.declare_parameter('imu_use_residual_as_noise', True)
        self.declare_parameter('imu_residual_noise_scale', 1.0)
        self.declare_parameter('imu_residual_noise_floor', 0.02)
        self.declare_parameter('imu_remove_gravity', True)
        self.declare_parameter('imu_gravity_mag', 9.81)
        self.declare_parameter('imu_gravity_sign', 1.0)

        g = lambda name: self.get_parameter(name).value
        self.intensity_field = g('intensity_field')
        self.doppler_field = g('doppler_field')
        self.R_radar_to_body = Rotation.from_quat(g('radar_extrinsic_quat_xyzw')).as_matrix()
        self.radar_denoise = g('radar_denoise')
        self.radar_denoise_radius = g('radar_denoise_radius')
        self.radar_denoise_min_neighbors = g('radar_denoise_min_neighbors')
        self.max_range = g('max_range')

        self.doppler_sign = g('doppler_sign')
        self.doppler_min_points = g('doppler_min_points')
        self.doppler_max_condition_number = g('doppler_max_condition_number')
        self.doppler_residual_threshold_scale = g('doppler_residual_threshold_scale')
        self.doppler_intensity_weighting = g('doppler_intensity_weighting')
        self.doppler_outlier_reject = g('doppler_outlier_reject')

        self.imu_measurement_noise_std = g('imu_measurement_noise_std')
        self.imu_use_residual_as_noise = g('imu_use_residual_as_noise')
        self.imu_residual_noise_scale = g('imu_residual_noise_scale')
        self.imu_residual_noise_floor = g('imu_residual_noise_floor')
        self.imu_remove_gravity = g('imu_remove_gravity')
        self.imu_gravity_mag = g('imu_gravity_mag')
        self.imu_gravity_sign = g('imu_gravity_sign')
        self.world_frame_id = g('world_frame_id')

        self.kf = ImuRadarVelocityKF(
            process_noise_std=g('imu_process_noise_std'),
            measurement_noise_std=self.imu_measurement_noise_std,
        )

        # ---- runtime state ----
        self._last_imu_stamp = None
        self._last_orientation_R = np.eye(3)
        self._have_orientation = False
        self._last_gt_vel_xy = None  # BENCHMARKING ONLY -- never read by predict()/update()
        # Set by _radar_callback for subclasses (rio_bridge): did the last
        # scan's Doppler solve succeed AND come out well-conditioned?
        self._last_doppler_ok = False
        self._last_doppler_result = None

        # ---- pub/sub ----
        self.fused_pub = self.create_publisher(TwistStamped, '~/fused_velocity', 10)
        self.doppler_pub = self.create_publisher(TwistStamped, '~/doppler_velocity', 10)

        # Single BEST_EFFORT (sensor-data) subscription per topic. A
        # BEST_EFFORT request matches BOTH publisher reliabilities (RELIABLE
        # ros_gz_bridge IMU, BEST_EFFORT SensorDataQoS radarays cloud), while
        # a dual best-effort+reliable subscription pair would deliver every
        # message from a RELIABLE publisher twice — double KF updates.
        self.create_subscription(
            Imu, g('imu_topic'), self._imu_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, g('radar_topic'), self._radar_callback,
            qos_profile_sensor_data
        )

        odom_topic = g('odom_topic')
        if odom_topic:
            self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
            self.get_logger().info(
                f"Benchmarking against ground truth on '{odom_topic}' "
                f"(logging only -- NOT used by the filter).")
        else:
            self.get_logger().info("No odom_topic set -- running without ground-truth "
                                    "benchmarking (this is normal for real deployment).")

    # -----------------------------------------------------------------
    def _imu_callback(self, msg: Imu):
        stamp = _stamp_to_sec(msg.header.stamp)

        if msg.orientation_covariance[0] == -1.0:
            self.get_logger().warning(
                "IMU orientation not available (covariance[0] == -1) -- reusing last "
                "known orientation. Fusion will be inaccurate until orientation is "
                "published (add an attitude filter, e.g. Madgwick/Mahony, upstream).",
                throttle_duration_sec=5.0)
        else:
            q = msg.orientation
            self._last_orientation_R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            self._have_orientation = True

        accel_body = np.array([msg.linear_acceleration.x,
                                msg.linear_acceleration.y,
                                msg.linear_acceleration.z], dtype=np.float64)
        accel_world = self._last_orientation_R @ accel_body
        if self.imu_remove_gravity:
            accel_world[2] -= self.imu_gravity_sign * self.imu_gravity_mag

        if self._last_imu_stamp is not None:
            dt = stamp - self._last_imu_stamp
            if dt > 0:
                self.kf.predict(accel_world[:2], dt)
        self._last_imu_stamp = stamp

    # -----------------------------------------------------------------
    def _radar_callback(self, msg: PointCloud2):
        if not self._have_orientation:
            self.get_logger().warning(
                "No IMU orientation received yet -- skipping radar scan until attitude "
                "is available.", throttle_duration_sec=5.0)
            return

        pts = parse_radar_pointcloud(msg, self.intensity_field, self.doppler_field)
        if pts is None:
            self.get_logger().error(
                f"Failed to parse radar PointCloud2 with fields "
                f"('x','y','z','{self.intensity_field}','{self.doppler_field}') -- "
                f"check the intensity_field/doppler_field parameters match your "
                f"radar plugin's actual field names.", throttle_duration_sec=5.0)
            return
        if pts.shape[0] == 0:
            return

        # radar sensor frame -> body frame (static extrinsic) -> world
        # orientation (from the latest IMU attitude). Rotation only, no
        # translation -- this stays an ego-centered frame, matching the
        # convention used for the Doppler bearing fit.
        xyz_body = pts[:, :3] @ self.R_radar_to_body.T
        xyz_world = xyz_body @ self._last_orientation_R.T
        radar_pc_2d = np.concatenate([xyz_world[:, :2], pts[:, 3:]], axis=1)  # x,y,intensity,doppler

        if self.radar_denoise:
            radar_pc_2d = denoise_scattered_points(radar_pc_2d,
                                                        radius=self.radar_denoise_radius,
                                                        min_neighbors=self.radar_denoise_min_neighbors)

        if radar_pc_2d.shape[0] > 0:
            dists = np.linalg.norm(radar_pc_2d[:, :2], axis=1)
            pts_fit = radar_pc_2d[dists <= self.max_range]
        else:
            pts_fit = radar_pc_2d

        doppler_vel_est = None
        result = None
        if pts_fit.shape[0] >= self.doppler_min_points:
            result = estimate_velocity_doppler(
                pts_fit,
                ego_vel_xy=(0.0, 0.0), ego_yaw_rate=0.0,
                doppler_col=3, intensity_col=2,
                doppler_sign=self.doppler_sign,
                use_intensity_weighting=self.doppler_intensity_weighting,
                outlier_reject=self.doppler_outlier_reject,
                residual_threshold_scale=self.doppler_residual_threshold_scale,
                min_points=self.doppler_min_points,
                max_condition_number=self.doppler_max_condition_number,
            )
            doppler_vel_est = -result.velocity  # static-scene assumption: fit recovers -v_ego

        self._last_doppler_result = result
        self._last_doppler_ok = bool(
            doppler_vel_est is not None
            and result is not None
            and result.resolved
            and result.well_conditioned
        )

        if doppler_vel_est is not None:
            self._publish_twist(self.doppler_pub, doppler_vel_est, msg.header.stamp)
            meas_std = self.imu_measurement_noise_std
            if self.imu_use_residual_as_noise and np.isfinite(result.residual_rms):
                meas_std = max(result.residual_rms * self.imu_residual_noise_scale,
                                self.imu_residual_noise_floor)
            self.kf.update(doppler_vel_est, measurement_noise_std=meas_std)
        else:
            self.get_logger().debug(
                f"Only {pts_fit.shape[0]} usable radar points this scan (need "
                f"{self.doppler_min_points}+) -- skipping Doppler update.")

        fused_vel = self.kf.velocity
        self._publish_twist(self.fused_pub, fused_vel, msg.header.stamp)

        # ---- benchmarking-only comparison log ----
        if self._last_gt_vel_xy is not None:
            fused_err = np.linalg.norm(fused_vel - self._last_gt_vel_xy)
            line = f"GT=({self._last_gt_vel_xy[0]:.3f},{self._last_gt_vel_xy[1]:.3f})"
            if doppler_vel_est is not None:
                doppler_err = np.linalg.norm(doppler_vel_est - self._last_gt_vel_xy)
                line += (f"  Doppler=({doppler_vel_est[0]:.3f},{doppler_vel_est[1]:.3f}) "
                         f"err={doppler_err:.3f}")
            line += (f"  Fused=({fused_vel[0]:.3f},{fused_vel[1]:.3f}) "
                     f"err={fused_err:.3f}")
            self.get_logger().info(line, throttle_duration_sec=1.0)

    # -----------------------------------------------------------------
    def _odom_callback(self, msg: Odometry):
        """BENCHMARKING ONLY. This value is read only inside the console
        comparison log above -- it is never passed to self.kf.predict()
        or self.kf.update(). Assumes msg.twist.twist is expressed in the
        world/odom frame; if your odom plugin publishes it in the body
        frame instead, rotate it here before storing."""
        self._last_gt_vel_xy = np.array([msg.twist.twist.linear.x,
                                          msg.twist.twist.linear.y], dtype=np.float64)

    # -----------------------------------------------------------------
    def _publish_twist(self, publisher, vel_xy, stamp):
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame_id
        msg.twist.linear.x = float(vel_xy[0])
        msg.twist.linear.y = float(vel_xy[1])
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRadarFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()