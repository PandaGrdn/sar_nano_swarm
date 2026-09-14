#!/usr/bin/env python3
"""attitude_filter.py — 6-DoF (gyro + accelerometer) attitude estimate for RIO.

WHY THIS EXISTS
Gazebo's sensor_msgs/Imu `orientation` field is computed from the simulated
body pose — it is ground truth, and gz-sensors cannot add noise to it. A real
Crazyflie has no attitude sensor: attitude is ESTIMATED from the (noisy,
biased) gyro and accelerometer, and heading drifts because there is no
magnetometer. rio_bridge.py feeds this filter only `angular_velocity`,
`linear_acceleration` and `header.stamp`, and hands RIO the filter's
quaternion instead of Gazebo's.

ALGORITHM — Madgwick IMU (gyro + accel) gradient-descent filter:
  S. O. H. Madgwick, A. J. L. Harrison, R. Vaidyanathan, "Estimation of IMU
  and MARG orientation using a gradient descent algorithm", IEEE Int. Conf.
  Rehabilitation Robotics (ICORR), 2011, doi:10.1109/ICORR.2011.5975346.

  q_dot = 1/2 q (x) [0, w - b_g]  -  beta * grad(f) / |grad(f)|
  f(q, a) = R(q)^T e_z - a/|a|,   grad(f) = J(q)^T f   (paper eqs. 25-26, 33)

  Why Madgwick over Mahony (Mahony, Hamel, Pflimlin, IEEE TAC 2008,
  doi:10.1109/TAC.2008.923738): the paper gives a single gain with a direct,
  documented mapping from the gyro measurement error (beta = sqrt(3/4) *
  omega_beta, paper eq. 50), so the gain is DERIVED from the calibrated gyro
  noise instead of hand-tuned Kp/Ki. The normalised gradient step bounds the
  correction rate (<= beta in quaternion-rate units, i.e. ~2*beta rad/s of
  tilt), which limits how far transient translational acceleration can pull
  the tilt. Mahony's integral term (gyro bias tracking) is not needed here:
  the constant bias is removed by the stationary start-up calibration below
  (the Crazyflie firmware's own sensfusion6.c ships both filters, and does a
  startup gyro-bias calibration in the same way).

  Quaternion convention: body->world (R(q) maps a body vector into world),
  ROS/scipy (x, y, z, w) order at the interface — identical to how RIO.py
  consumes `msg.orientation` via Rotation.from_quat([x, y, z, w]).as_matrix().

  Accelerometer convention: SPECIFIC FORCE, i.e. ~(0, 0, +g) in body z when
  level and at rest (gz-sensors reports a_world - g_world rotated into body;
  RIO.py subtracts +imu_gravity_mag from world z, which is consistent). The
  correction therefore aligns R^T e_z with a/|a|.

  Yaw is not observable from gravity (the accel gradient has no component
  about world z), so yaw is pure gyro integration and drifts with the residual
  gyro bias — like a real magnetometer-less Crazyflie.

TIMESTAMPS: dt from consecutive message header stamps (SIM time). Non-advancing
or duplicate stamps are ignored (counted). A forward gap > max_dt_gap_s, or a
backward jump larger than that (sim reset), is NOT integrated across: the
filter re-anchors its time base and counts it.

INITIALISATION (mirrors EKF-RIO `calib_gyro: true` / `T_init`,
github.com/christopherdoer/rio ekf_rio_default.yaml, and the Crazyflie
startup gyro calibration): for init_window_s of sim time the platform must be
stationary; the mean gyro is the bias estimate, the mean accel gives roll and
pitch from gravity, yaw = the surveyed common launch heading
(launch.init_yaw_deg, plan D14). A sample with |w| > init_max_gyro_norm_rad_s
or ||a| - g| > init_max_accel_dev_mps2 restarts the window (counted). Until
initialised the filter is "not ready".

Pure numpy (no rclpy).

    python perception/radar_processing/attitude_filter.py --selftest
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass

import numpy as np

ATTITUDE_SOURCES = ("estimated", "gazebo_truth")
ATTITUDE_ALGORITHMS = ("madgwick_imu",)
# Keys of rio.attitude that must be present for source: estimated. No defaults:
# a missing key must fail loudly rather than silently pick a number.
_REQUIRED_ESTIMATED_KEYS = (
    "algorithm",
    "madgwick_beta",
    "init_window_s",
    "init_min_samples",
    "init_max_gyro_norm_rad_s",
    "init_max_accel_dev_mps2",
    "max_dt_gap_s",
    "gravity_mps2",
    "orientation_cov_rad2",
    "log_throttle_s",
)


class AttitudeConfigError(ValueError):
    """Missing / invalid rio.attitude block."""


@dataclass
class AttitudeConfig:
    source: str
    algorithm: str = "madgwick_imu"
    beta: float = float("nan")
    init_window_s: float = float("nan")
    init_min_samples: int = 0
    init_max_gyro_norm_rad_s: float = float("nan")
    init_max_accel_dev_mps2: float = float("nan")
    max_dt_gap_s: float = float("nan")
    gravity_mps2: float = float("nan")
    orientation_cov_rad2: float = float("nan")
    log_throttle_s: float = float("nan")
    init_yaw_rad: float = 0.0


def parse_attitude_config(cfg: dict) -> AttitudeConfig:
    """cfg["rio"]["attitude"] (+ launch.init_yaw_deg) -> AttitudeConfig.

    Raises AttitudeConfigError when the block is absent, the source is
    unknown, or (for source: estimated) any key is missing / invalid. An
    absent block never falls back to anything — least of all ground truth.
    """
    rio = (cfg or {}).get("rio")
    block = rio.get("attitude") if isinstance(rio, dict) else None
    if not isinstance(block, dict):
        raise AttitudeConfigError(
            "rio.attitude block missing from the estimator config (required; set "
            "source: estimated — refusing to guess, and never defaulting to ground truth)")
    source = block.get("source")
    if source not in ATTITUDE_SOURCES:
        raise AttitudeConfigError(
            f"rio.attitude.source={source!r} unknown; expected one of {ATTITUDE_SOURCES}")
    if source == "gazebo_truth":
        return AttitudeConfig(source="gazebo_truth")

    missing = [k for k in _REQUIRED_ESTIMATED_KEYS if k not in block]
    if missing:
        raise AttitudeConfigError(f"rio.attitude missing required key(s): {', '.join(missing)}")
    algo = block["algorithm"]
    if algo not in ATTITUDE_ALGORITHMS:
        raise AttitudeConfigError(
            f"rio.attitude.algorithm={algo!r} unknown; expected one of {ATTITUDE_ALGORITHMS}")

    def num(key, positive=True, allow_zero=False):
        v = block[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)):
            raise AttitudeConfigError(f"rio.attitude.{key}: expected a finite number, got {v!r}")
        v = float(v)
        if positive and (v < 0.0 or (v == 0.0 and not allow_zero)):
            raise AttitudeConfigError(f"rio.attitude.{key}: must be > 0, got {v!r}")
        return v

    n_min = block["init_min_samples"]
    if isinstance(n_min, bool) or not isinstance(n_min, int) or n_min < 1:
        raise AttitudeConfigError(f"rio.attitude.init_min_samples: expected an int >= 1, got {n_min!r}")

    launch = (cfg or {}).get("launch") or {}
    if "init_yaw_deg" not in launch:
        raise AttitudeConfigError(
            "launch.init_yaw_deg missing: the attitude filter's initial yaw is the "
            "surveyed common launch heading (D14, no magnetometer)")
    yaw_deg = launch["init_yaw_deg"]
    if isinstance(yaw_deg, bool) or not isinstance(yaw_deg, (int, float)) or not math.isfinite(float(yaw_deg)):
        raise AttitudeConfigError(f"launch.init_yaw_deg: expected a finite number, got {yaw_deg!r}")

    return AttitudeConfig(
        source="estimated",
        algorithm=str(algo),
        beta=num("madgwick_beta"),
        init_window_s=num("init_window_s"),
        init_min_samples=int(n_min),
        init_max_gyro_norm_rad_s=num("init_max_gyro_norm_rad_s"),
        init_max_accel_dev_mps2=num("init_max_accel_dev_mps2"),
        max_dt_gap_s=num("max_dt_gap_s"),
        gravity_mps2=num("gravity_mps2"),
        orientation_cov_rad2=num("orientation_cov_rad2", allow_zero=True),
        log_throttle_s=num("log_throttle_s"),
        init_yaw_rad=math.radians(float(yaw_deg)),
    )


# --- quaternion helpers (internal order w, x, y, z) --------------------------
def quat_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX: q = qz(yaw) (x) qy(pitch) (x) qx(roll), body->world, (w,x,y,z)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    ], dtype=np.float64)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """(roll, pitch, yaw) of body->world R = Rz Ry Rx."""
    pitch = math.asin(float(np.clip(-R[2, 0], -1.0, 1.0)))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def stamp_to_sec(stamp) -> float:
    """builtin_interfaces/Time (sec, nanosec) or a plain float -> seconds."""
    if isinstance(stamp, (int, float)):
        return float(stamp)
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class MadgwickImuFilter:
    """Madgwick (2011) IMU filter with stationary start-up calibration."""

    def __init__(self, cfg: AttitudeConfig):
        if cfg.source != "estimated":
            raise AttitudeConfigError(f"MadgwickImuFilter needs source: estimated, got {cfg.source!r}")
        self.cfg = cfg
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.gyro_bias = np.zeros(3)
        self.ready = False
        self._last_stamp: float | None = None
        # init window accumulators
        self._win_start: float | None = None
        self._win_n = 0
        self._win_gyro = np.zeros(3)
        self._win_accel = np.zeros(3)
        # counters
        self.n_duplicate = 0
        self.n_gaps = 0
        self.n_init_restarts = 0
        self.n_nonfinite = 0
        self.n_integrated = 0

    # ---------------------------------------------------------------------
    @property
    def quaternion_xyzw(self) -> np.ndarray:
        w, x, y, z = self.q
        return np.array([x, y, z, w], dtype=np.float64)

    @property
    def rotation_matrix(self) -> np.ndarray:
        return quat_to_matrix(self.q)

    def _restart_window(self):
        self._win_start = None
        self._win_n = 0
        self._win_gyro[:] = 0.0
        self._win_accel[:] = 0.0

    # ---------------------------------------------------------------------
    def update(self, stamp, gyro, accel) -> bool:
        """Feed one IMU sample. Returns True iff the state changed (an init
        sample was accepted into the window, init completed, or the attitude
        was propagated)."""
        t = stamp_to_sec(stamp)
        w = np.asarray(gyro, dtype=np.float64).reshape(3)
        a = np.asarray(accel, dtype=np.float64).reshape(3)
        if not (math.isfinite(t) and np.all(np.isfinite(w)) and np.all(np.isfinite(a))):
            self.n_nonfinite += 1
            return False

        if self._last_stamp is not None:
            dt = t - self._last_stamp
            if dt <= 0.0 and dt >= -self.cfg.max_dt_gap_s:
                self.n_duplicate += 1  # duplicate / slightly out-of-order
                return False
            if dt > self.cfg.max_dt_gap_s or dt < 0.0:
                # gap or sim-time reset: never integrate across it
                self.n_gaps += 1
                self._last_stamp = t
                if not self.ready:
                    self._restart_window()
                return False
        else:
            dt = 0.0

        if not self.ready:
            self._last_stamp = t
            return self._init_sample(t, w, a)

        self._last_stamp = t
        self._propagate(w - self.gyro_bias, a, dt)
        self.n_integrated += 1
        return True

    def _init_sample(self, t, w, a) -> bool:
        c = self.cfg
        if (np.linalg.norm(w) > c.init_max_gyro_norm_rad_s
                or abs(np.linalg.norm(a) - c.gravity_mps2) > c.init_max_accel_dev_mps2):
            if self._win_n > 0:
                self.n_init_restarts += 1
            self._restart_window()
            return False
        if self._win_start is None:
            self._win_start = t
        self._win_n += 1
        self._win_gyro += w
        self._win_accel += a
        if (t - self._win_start) >= c.init_window_s and self._win_n >= c.init_min_samples:
            self.gyro_bias = self._win_gyro / self._win_n
            am = self._win_accel / self._win_n
            roll = math.atan2(am[1], am[2])
            pitch = math.atan2(-am[0], math.hypot(am[1], am[2]))
            self.q = quat_from_rpy(roll, pitch, c.init_yaw_rad)
            self.ready = True
        return True

    def _propagate(self, w, a, dt):
        q0, q1, q2, q3 = self.q
        # gyro term: 1/2 q (x) [0, w]
        qdot = 0.5 * np.array([
            -q1 * w[0] - q2 * w[1] - q3 * w[2],
            q0 * w[0] + q2 * w[2] - q3 * w[1],
            q0 * w[1] - q1 * w[2] + q3 * w[0],
            q0 * w[2] + q1 * w[1] - q2 * w[0],
        ])
        an = float(np.linalg.norm(a))
        if an > 0.0:
            ax, ay, az = a / an
            f = np.array([
                2.0 * (q1 * q3 - q0 * q2) - ax,
                2.0 * (q0 * q1 + q2 * q3) - ay,
                2.0 * (0.5 - q1 * q1 - q2 * q2) - az,
            ])
            J = np.array([
                [-2.0 * q2, 2.0 * q3, -2.0 * q0, 2.0 * q1],
                [2.0 * q1, 2.0 * q0, 2.0 * q3, 2.0 * q2],
                [0.0, -4.0 * q1, -4.0 * q2, 0.0],
            ])
            grad = J.T @ f
            gn = float(np.linalg.norm(grad))
            if gn > 0.0:
                qdot = qdot - self.cfg.beta * grad / gn
        q = self.q + qdot * dt
        self.q = q / np.linalg.norm(q)


# =============================================================================
# selftest
# =============================================================================
# SIMULATOR-side noise used ONLY to synthesise test data (per-sample stddev at
# the 1000 Hz Gazebo IMU = imu_calib_results.yaml random_walk_coeff*sqrt(1000)).
_SIM_GYRO_STD = np.array([0.003962, 0.006329, 0.002489])       # rad/s
_SIM_ACCEL_STD = np.array([0.08448, 0.028579, 0.009932])      # m/s^2
_G = 9.81


def _test_cfg(**over) -> AttitudeConfig:
    base = {"rio": {"attitude": {
        "source": "estimated", "algorithm": "madgwick_imu",
        "madgwick_beta": 0.003935, "init_window_s": 2.0, "init_min_samples": 50,
        "init_max_gyro_norm_rad_s": 0.05, "init_max_accel_dev_mps2": 0.5,
        "max_dt_gap_s": 0.1, "gravity_mps2": 9.81, "orientation_cov_rad2": 3.0e-4,
        "log_throttle_s": 5.0}}, "launch": {"init_yaw_deg": 0.0}}
    for k, v in over.items():
        if k == "init_yaw_deg":
            base["launch"][k] = v
        else:
            base["rio"]["attitude"][k] = v
    return parse_attitude_config(base)


def _rot_err_deg(Ra, Rb) -> float:
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


_HT_TURN_ON_BIAS = np.array([0.003, -0.002, 0.001])      # rad/s, recovered by init


def _hover_translate(rng, c: AttitudeConfig, noise: bool, rate: float = 300.0, T: float = 120.0):
    """Synthetic 5 s on the ground, then take-off ramp into hover with
    coordinated translation (x 0.8 m/s^2 @8 s, y 0.5 @13 s, z 0.3 @5 s),
    small attitude jitter and a +-0.6 rad yaw sweep. Gyro = body rate
    (differentiated from the analytic attitude) + turn-on bias + Gauss-Markov
    bias (calib bias_instability, tau 4.57 s) + white noise; accel = specific
    force + white noise. Returns (N x 3 [roll,pitch,yaw] error deg, filter)."""
    dt = 1.0 / rate
    t_takeoff = 5.0
    f = MadgwickImuFilter(c)
    gm_q, gm_T = np.array([7.5e-5, 1.19e-4, 5.3e-5]), 4.57
    gm = np.zeros(3)

    def att(tt):
        s = max(0.0, tt - t_takeoff)
        ramp = min(1.0, s / 5.0)
        ax_w = ramp * 0.8 * math.sin(2 * math.pi * s / 8.0)
        ay_w = ramp * 0.5 * math.sin(2 * math.pi * s / 13.0)
        az_w = ramp * 0.3 * math.sin(2 * math.pi * s / 5.0)
        yaw_t = ramp * 0.6 * math.sin(2 * math.pi * s / 30.0)
        roll_t = ramp * (-math.atan2(ay_w, _G) + 0.02 * math.sin(2 * math.pi * s / 1.7))
        pitch_t = ramp * (math.atan2(ax_w, _G) + 0.02 * math.sin(2 * math.pi * s / 2.3))
        return quat_to_matrix(quat_from_rpy(roll_t, pitch_t, yaw_t)), np.array([ax_w, ay_w, az_w])

    h = 1e-4
    errs = []
    for k in range(int(round(T * rate)) + 1):
        tt = k * dt
        R_t, a_w = att(tt)
        Rp, _ = att(tt + h)
        Rm, _ = att(tt - h)
        W = R_t.T @ (Rp - Rm) / (2 * h)
        omega = np.array([W[2, 1], W[0, 2], W[1, 0]])
        fb = R_t.T @ (a_w + np.array([0.0, 0.0, _G]))
        if noise:
            gm = math.exp(-dt / gm_T) * gm + rng.normal(0.0, gm_q * math.sqrt(1 - math.exp(-2 * dt / gm_T)))
            gyro = omega + _HT_TURN_ON_BIAS + gm + rng.normal(0, _SIM_GYRO_STD)
            acc = fb + rng.normal(0, _SIM_ACCEL_STD)
        else:
            gyro, acc = omega, fb
        f.update(tt, gyro, acc)
        if f.ready and k % 30 == 0:
            re, pe, ye = matrix_to_rpy(f.rotation_matrix)
            rt, pt, yt = matrix_to_rpy(R_t)
            errs.append((math.degrees(re - rt), math.degrees(pe - pt), math.degrees(_wrap(ye - yt))))
    return np.array(errs), f


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

    rng = np.random.default_rng(0)
    rate = 300.0
    dt = 1.0 / rate

    def static_run(f, R_true, T, bias, t0=0.0, noise=True):
        """Feed a stationary IMU with attitude R_true for T seconds."""
        n = int(round(T * rate))
        fb = R_true.T @ np.array([0.0, 0.0, _G])
        for k in range(n):
            wn = rng.normal(0.0, _SIM_GYRO_STD) if noise else np.zeros(3)
            an = rng.normal(0.0, _SIM_ACCEL_STD) if noise else np.zeros(3)
            f.update(t0 + k * dt, bias + wn, fb + an)
        return t0 + n * dt

    # ---- 1 config -------------------------------------------------------------
    c = _test_cfg()
    check("1 parse estimated", c.source == "estimated" and c.algorithm == "madgwick_imu")
    rms = math.sqrt(float(np.mean(_SIM_GYRO_STD ** 2)))
    check("1a documented beta = sqrt(3/4)*rms(gyro std)",
          abs(math.sqrt(0.75) * rms - 0.003935) < 5e-6, f"{math.sqrt(0.75)*rms:.6f}")

    def raises(cfg):
        try:
            parse_attitude_config(cfg)
        except AttitudeConfigError:
            return True
        return False

    check("1b missing block refused", raises({"rio": {"source": "real"}}) and raises({}))
    check("1c unknown source refused", raises({"rio": {"attitude": {"source": "truth"}}}))
    check("1d gazebo_truth parses", parse_attitude_config(
        {"rio": {"attitude": {"source": "gazebo_truth"}}}).source == "gazebo_truth")
    good = {"rio": {"attitude": dict(source="estimated", algorithm="madgwick_imu", madgwick_beta=0.004,
                                     init_window_s=2.0, init_min_samples=50, init_max_gyro_norm_rad_s=0.05,
                                     init_max_accel_dev_mps2=0.5, max_dt_gap_s=0.1, gravity_mps2=9.81,
                                     orientation_cov_rad2=3e-4, log_throttle_s=5.0)},
            "launch": {"init_yaw_deg": 0.0}}
    check("1e complete dict parses", not raises(good))
    bad = {"rio": {"attitude": dict(good["rio"]["attitude"])}, "launch": {"init_yaw_deg": 0.0}}
    del bad["rio"]["attitude"]["madgwick_beta"]
    check("1f missing key refused", raises(bad))
    bad2 = {"rio": {"attitude": dict(good["rio"]["attitude"], algorithm="ekf")}, "launch": {"init_yaw_deg": 0.0}}
    check("1g unknown algorithm refused", raises(bad2))
    check("1h missing launch.init_yaw_deg refused", raises({"rio": good["rio"]}))
    bad3 = {"rio": {"attitude": dict(good["rio"]["attitude"], madgwick_beta="0.004")}, "launch": {"init_yaw_deg": 0.0}}
    check("1i string gain refused", raises(bad3))
    try:
        from pathlib import Path
        import yaml
        shipped = yaml.safe_load(open(Path(__file__).resolve().parents[2]
                                      / "configs/estimation/swarm_loc.yaml", encoding="utf-8"))
        sc = parse_attitude_config(shipped)
        err = ""
    except Exception as e:  # noqa: BLE001
        sc, err = None, repr(e)
    check("1j shipped swarm_loc.yaml rio.attitude parses as estimated",
          sc is not None and sc.source == "estimated" and abs(sc.beta - math.sqrt(0.75) * rms) < 5e-6,
          err or str(sc))

    # ---- 2 quaternion convention vs scipy --------------------------------------
    roll, pitch, yaw = 0.3, -0.2, 2.5
    q = quat_from_rpy(roll, pitch, yaw)
    Rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    Rzyx = Rz @ Ry @ Rx
    check("2 quat unit norm", abs(np.linalg.norm(q) - 1.0) < 1e-12)
    check("2a quat_to_matrix == Rz Ry Rx", np.allclose(quat_to_matrix(q), Rzyx, atol=1e-12))
    f2 = MadgwickImuFilter(c)
    f2.q = q
    try:
        from scipy.spatial.transform import Rotation
        Rs = Rotation.from_quat(f2.quaternion_xyzw.tolist()).as_matrix()
        v_body = np.array([1.0, 0.0, 0.0])
        check("2b scipy from_quat([x,y,z,w]) matches (body->world)",
              np.allclose(Rs, Rzyx, atol=1e-12)
              and np.allclose(Rs @ v_body, Rzyx @ v_body, atol=1e-12)
              and np.allclose(Rs @ v_body, [math.cos(yaw) * math.cos(pitch),
                                            math.sin(yaw) * math.cos(pitch), -math.sin(pitch)], atol=1e-12))
    except ImportError:
        check("2b scipy available", False, "scipy not importable")
    check("2c matrix_to_rpy roundtrip",
          np.allclose(matrix_to_rpy(Rzyx), (roll, pitch, yaw), atol=1e-12))

    # ---- 3 not ready before the window; bias & tilt from init ----------------
    R_tilt = quat_to_matrix(quat_from_rpy(math.radians(5.0), math.radians(-3.0), 0.0))
    bias = np.array([0.010, -0.020, 0.005])
    f3 = MadgwickImuFilter(c)
    t = static_run(f3, R_tilt, 1.9, bias)
    check("3 not ready before init window", not f3.ready)
    t = static_run(f3, R_tilt, 0.2, bias, t0=t)
    check("3a ready after init window", f3.ready)
    n_win = int(round(2.0 * rate)) + 1
    bias_tol = 4.0 * _SIM_GYRO_STD / math.sqrt(n_win)
    check("3b injected constant gyro bias recovered",
          np.all(np.abs(f3.gyro_bias - bias) < bias_tol),
          f"err={f3.gyro_bias - bias} tol={bias_tol}")
    r0, p0, y0 = matrix_to_rpy(f3.rotation_matrix)
    check("3c initial roll/pitch from gravity (<0.3 deg)",
          abs(math.degrees(r0) - 5.0) < 0.3 and abs(math.degrees(p0) + 3.0) < 0.3 and abs(y0) < 1e-3,
          f"roll={math.degrees(r0):.3f} pitch={math.degrees(p0):.3f} yaw={y0}")

    # ---- 4 static 60 s with injected noise ---------------------------------------
    bias_res = bias - f3.gyro_bias
    T_static = 60.0
    t_start = t
    max_tilt = 0.0
    n = int(round(T_static * rate))
    fb = R_tilt.T @ np.array([0.0, 0.0, _G])
    for k in range(n):
        f3.update(t_start + k * dt, bias + rng.normal(0, _SIM_GYRO_STD), fb + rng.normal(0, _SIM_ACCEL_STD))
        if k % 30 == 0:
            rr, pp, _ = matrix_to_rpy(f3.rotation_matrix)
            max_tilt = max(max_tilt, abs(math.degrees(rr) - 5.0), abs(math.degrees(pp) + 3.0))
    rr, pp, yy = matrix_to_rpy(f3.rotation_matrix)
    e_roll, e_pitch = math.degrees(rr) - 5.0, math.degrees(pp) + 3.0
    print(f"[selftest] info static 60 s @300 Hz: roll err {e_roll:+.4f} deg, pitch err {e_pitch:+.4f} deg, "
          f"max |tilt err| {max_tilt:.4f} deg, yaw drift {math.degrees(yy):+.4f} deg")
    check("4 static roll/pitch error bounded (<0.5 deg max over 60 s)", max_tilt < 0.5, f"max={max_tilt:.4f}")
    # yaw drift = residual bias projected on world z, integrated; + gyro ARW
    wz_world = float((R_tilt @ bias_res)[2])
    pred = math.degrees(wz_world * T_static)
    arw = math.degrees(float(np.linalg.norm(_SIM_GYRO_STD)) * math.sqrt(dt * T_static))
    check("4a yaw drift consistent with residual gyro bias",
          abs(math.degrees(yy) - pred) < 4.0 * arw + 0.05,
          f"yaw={math.degrees(yy):.4f} pred={pred:.4f} arw_1sigma={arw:.4f}")
    check("4b quaternion stays normalised", abs(np.linalg.norm(f3.q) - 1.0) < 1e-12)

    # ---- 5 constant yaw rate tracked --------------------------------------------
    f5 = MadgwickImuFilter(c)
    t = static_run(f5, np.eye(3), 2.1, np.zeros(3), noise=False)
    rate_z = 0.5
    T5 = 10.0
    n5 = int(round(T5 * rate))
    for k in range(n5):   # static_run's last stamp was t - dt
        f5.update(t + k * dt, np.array([0.0, 0.0, rate_z]), np.array([0.0, 0.0, _G]))
    _, _, y5 = matrix_to_rpy(f5.rotation_matrix)
    e5 = math.degrees(_wrap(y5 - rate_z * n5 * dt))
    check("5 constant yaw rate tracked (<0.05 deg after 5 rad)", abs(e5) < 0.05, f"err={e5:.5f} deg")

    # ---- 6 roll step converges at the gain-implied rate -------------------------
    f6 = MadgwickImuFilter(c)
    t = static_run(f6, np.eye(3), 2.1, np.zeros(3), noise=False)
    step = math.radians(10.0)
    fb6 = quat_to_matrix(quat_from_rpy(step, 0.0, 0.0)).T @ np.array([0.0, 0.0, _G])
    T6 = 10.0
    for k in range(int(round(T6 * rate))):
        f6.update(t + k * dt, np.zeros(3), fb6)
    r6, _, _ = matrix_to_rpy(f6.rotation_matrix)
    rate_meas = r6 / T6                        # rad/s while error is still large
    rate_theory = 2.0 * c.beta                 # |q_dot| = beta <=> |omega| = 2 beta
    print(f"[selftest] info roll step: measured correction {math.degrees(rate_meas):.4f} deg/s, "
          f"2*beta = {math.degrees(rate_theory):.4f} deg/s")
    check("6 tilt correction rate = 2*beta (+-5%)", abs(rate_meas / rate_theory - 1.0) < 0.05,
          f"ratio={rate_meas/rate_theory:.4f}")
    for k in range(int(round(T6 * rate)), int(round(60.0 * rate))):
        f6.update(t + k * dt, np.zeros(3), fb6)
    r6b, _, _ = matrix_to_rpy(f6.rotation_matrix)
    check("6a roll step fully converged after 60 s (<0.05 deg)",
          abs(math.degrees(r6b - step)) < 0.05, f"err={math.degrees(r6b - step):.4f} deg")

    # ---- 7 init restarts on motion ----------------------------------------------
    f7 = MadgwickImuFilter(c)
    t = static_run(f7, np.eye(3), 1.0, np.zeros(3))
    f7.update(t, np.array([0.2, 0.0, 0.0]), np.array([0.0, 0.0, _G]))      # gyro motion
    check("7 gyro motion restarts window", f7.n_init_restarts == 1 and not f7.ready, str(f7.n_init_restarts))
    t2 = static_run(f7, np.eye(3), 1.5, np.zeros(3), t0=t + dt)
    check("7a not ready 1.5 s after restart (window 2 s)", not f7.ready)
    f7.update(t2, np.zeros(3), np.array([0.0, 0.0, _G + 2.0]))           # accel motion
    check("7b accel motion restarts window", f7.n_init_restarts == 2 and not f7.ready)
    static_run(f7, np.eye(3), 2.1, np.zeros(3), t0=t2 + dt)
    check("7c ready after a full quiet window", f7.ready)

    # ---- 8 duplicate / non-advancing stamps ---------------------------------------
    f8 = MadgwickImuFilter(c)
    t = static_run(f8, np.eye(3), 2.1, np.zeros(3), noise=False)
    q_before = f8.q.copy()
    t_last = f8._last_stamp
    changed = f8.update(t_last, np.array([5.0, 5.0, 5.0]), np.array([0.0, 0.0, _G]))
    changed2 = f8.update(t_last - 0.001, np.array([5.0, 5.0, 5.0]), np.array([0.0, 0.0, _G]))
    check("8 duplicate / non-advancing stamps ignored",
          not changed and not changed2 and np.array_equal(q_before, f8.q) and f8.n_duplicate == 2,
          f"dup={f8.n_duplicate}")

    # ---- 9 large gap not integrated --------------------------------------------
    changed = f8.update(t_last + 1.0, np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, _G]))
    check("9 gap > max_dt_gap_s not integrated",
          not changed and np.array_equal(q_before, f8.q) and f8.n_gaps == 1, f"gaps={f8.n_gaps}")
    changed = f8.update(t_last + 1.0 + dt, np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, _G]))
    _, _, y9 = matrix_to_rpy(f8.rotation_matrix)
    check("9a integration resumes after the gap", changed and abs(y9 - dt) < 1e-7, f"yaw={y9}")
    changed = f8.update(t_last - 5.0, np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, _G]))
    check("9b sim-time reset (backward jump) re-anchors, not integrated",
          not changed and f8.n_gaps == 2 and f8._last_stamp == t_last - 5.0)
    check("9c non-finite sample ignored",
          not f8.update(t_last, np.array([np.nan, 0, 0]), np.zeros(3)) and f8.n_nonfinite == 1)

    # ---- 10 hover-then-translate, 120 s @ 300 Hz -----------------------------------
    E, f10 = _hover_translate(np.random.default_rng(1), c, noise=True)
    fr, fp, fy = E[-1]
    rms10 = np.sqrt(np.mean(E ** 2, axis=0))
    mx10 = np.max(np.abs(E), axis=0)
    print(f"[selftest] info hover-translate 120 s @300 Hz (noise+bias): final err roll {fr:+.3f} pitch {fp:+.3f} "
          f"yaw {fy:+.3f} deg; RMS r/p/y {rms10[0]:.3f}/{rms10[1]:.3f}/{rms10[2]:.3f}; "
          f"max r/p/y {mx10[0]:.3f}/{mx10[1]:.3f}/{mx10[2]:.3f} deg; restarts={f10.n_init_restarts} "
          f"bias err={np.round(f10.gyro_bias - _HT_TURN_ON_BIAS, 6)}")
    check("10 hover-translate initialised during the ground window", f10.ready and E.shape[0] > 1100,
          str(E.shape))
    check("10a hover-translate roll/pitch max error < 1.5 deg", mx10[0] < 1.5 and mx10[1] < 1.5, str(mx10))
    check("10b hover-translate yaw max error < 2 deg over 120 s", mx10[2] < 2.0, str(mx10))
    E0, _ = _hover_translate(np.random.default_rng(1), c, noise=False)
    rms0 = np.sqrt(np.mean(E0 ** 2, axis=0))
    mx0 = np.max(np.abs(E0), axis=0)
    print(f"[selftest] info hover-translate noise-free (algorithmic error only): final r/p/y "
          f"{E0[-1][0]:+.3f}/{E0[-1][1]:+.3f}/{E0[-1][2]:+.3f} deg; RMS {rms0[0]:.3f}/{rms0[1]:.3f}/{rms0[2]:.3f}; "
          f"max {mx0[0]:.3f}/{mx0[1]:.3f}/{mx0[2]:.3f} deg")
    # Noise-free error is algorithmic: Madgwick pulls tilt toward the apparent
    # gravity (translational accel), bounded by the 2*beta correction rate;
    # yaw-angle error is only the ZYX coupling of that tilt error.
    check("10c noise-free hover-translate: tilt error from translational accel < 1.5 deg, yaw < 0.25 deg",
          mx0[0] < 1.5 and mx0[1] < 1.5 and mx0[2] < 0.25, str(mx0))

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    ap.print_help()
    sys.exit(0)


if __name__ == "__main__":
    main()
