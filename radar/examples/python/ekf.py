import numpy as np

# class VelocityEKF:
#     """
#     Extended Kalman Filter estimating 3D linear velocity AND 3D angular
#     velocity (omega), fusing:
#       - IMU acceleration + gyro as the process model input (prediction step)
#       - Landmark-shift-derived linear velocity as a measurement (update step)
#       - Optionally, landmark/vision-derived angular velocity as a measurement

#     State vector (12,): [vx, vy, vz, wx, wy, wz, bias_ax, bias_ay, bias_az, bias_wx, bias_wy, bias_wz]
#       - vx, vy, vz: linear velocity in the world/navigation frame (m/s)
#       - wx, wy, wz: angular velocity in the body frame (rad/s)
#       - bias_ax/ay/az: slowly-varying accelerometer bias (m/s^2)
#       - bias_wx/wy/wz: slowly-varying gyro bias (rad/s)

#     Process model:
#       v_k     = v_{k-1} + (a_meas - bias_a_{k-1}) * dt
#       w_k     = gyro_meas - bias_w_{k-1}     (gyro directly measures angular velocity)
#       bias_a_k = bias_a_{k-1}  (random walk)
#       bias_w_k = bias_w_{k-1}  (random walk)

#     Measurement model:
#       z_vel   = v_k     (landmark-based velocity directly observes linear velocity)
#       z_omega = w_k     (landmark/vision-based rotation-rate directly observes angular velocity)
#     """

#     def __init__(self, initial_velocity=None, initial_omega=None,
#                  initial_bias=None, initial_gyro_bias=None,
#                  process_noise_vel=0.05, process_noise_omega=0.05,
#                  process_noise_bias=0.001, process_noise_gyro_bias=0.001,
#                  measurement_noise=0.3, omega_measurement_noise=0.1):
#         """
#         initial_velocity: (3,) array, starting velocity guess (default zeros)
#         initial_omega: (3,) array, starting angular velocity guess (default zeros)
#         initial_bias: (3,) array, starting accel bias guess (default zeros)
#         initial_gyro_bias: (3,) array, starting gyro bias guess (default zeros)
#         process_noise_vel: process noise std-dev for velocity (m/s^2), scaled by dt in Q
#         process_noise_omega: process noise std-dev for angular velocity (rad/s), scaled by dt in Q
#         process_noise_bias: process noise std-dev for accel bias random walk (m/s^2 per sqrt(s))
#         process_noise_gyro_bias: process noise std-dev for gyro bias random walk (rad/s per sqrt(s))
#         measurement_noise: std-dev of landmark-derived velocity measurement (m/s)
#         omega_measurement_noise: std-dev of landmark/vision-derived angular velocity measurement (rad/s)
#         """
#         v0 = np.zeros(3) if initial_velocity is None else np.asarray(initial_velocity, dtype=float)
#         w0 = np.zeros(3) if initial_omega is None else np.asarray(initial_omega, dtype=float)
#         ba0 = np.zeros(3) if initial_bias is None else np.asarray(initial_bias, dtype=float)
#         bw0 = np.zeros(3) if initial_gyro_bias is None else np.asarray(initial_gyro_bias, dtype=float)

#         # state: [vx,vy,vz, wx,wy,wz, bax,bay,baz, bwx,bwy,bwz]
#         self.x = np.concatenate([v0, w0, ba0, bw0])
#         self.P = np.eye(12) * 1.0  # initial state covariance (fairly uncertain)

#         self.q_vel = process_noise_vel
#         self.q_omega = process_noise_omega
#         self.q_bias = process_noise_bias
#         self.q_gyro_bias = process_noise_gyro_bias

#         self.r_meas = measurement_noise
#         self.r_omega_meas = omega_measurement_noise

#         # measurement models: directly observe velocity or omega, not the biases
#         self.H_vel = np.hstack([np.eye(3), np.zeros((3, 9))])
#         self.H_omega = np.hstack([np.zeros((3, 3)), np.eye(3), np.zeros((3, 6))])

#     def predict(self, accel_meas, gyro_meas, dt):
#         """
#         Prediction step using IMU acceleration and gyro.

#         accel_meas: (3,) array, measured linear acceleration in the world/nav frame
#                     (already gravity-compensated — see note below).
#         gyro_meas: (3,) array, measured angular velocity (rad/s), body frame.
#         dt: time elapsed since the last predict/update, in seconds.
#         """
#         accel_meas = np.asarray(accel_meas, dtype=float)
#         gyro_meas = np.asarray(gyro_meas, dtype=float)

#         v = self.x[0:3]
#         w = self.x[3:6]
#         ba = self.x[6:9]
#         bw = self.x[9:12]

#         # process model
#         v_new = v + (accel_meas - ba) * dt
#         w_new = gyro_meas - bw   # gyro directly gives angular velocity, bias-corrected
#         ba_new = ba              # random walk, no change in the mean
#         bw_new = bw              # random walk, no change in the mean

#         self.x = np.concatenate([v_new, w_new, ba_new, bw_new])

#         # state transition Jacobian F (12x12)
#         F = np.eye(12)
#         F[0:3, 6:9] = -np.eye(3) * dt   # d(v_new)/d(bias_a) = -dt
#         F[3:6, 3:6] = np.zeros((3, 3))  # w_new does NOT depend on previous w
#         F[3:6, 9:12] = -np.eye(3)       # d(w_new)/d(bias_w) = -1

#         # process noise covariance Q (12x12)
#         Q = np.zeros((12, 12))
#         Q[0:3, 0:3] = np.eye(3) * (self.q_vel ** 2) * dt
#         Q[3:6, 3:6] = np.eye(3) * (self.q_omega ** 2) * dt
#         Q[6:9, 6:9] = np.eye(3) * (self.q_bias ** 2) * dt
#         Q[9:12, 9:12] = np.eye(3) * (self.q_gyro_bias ** 2) * dt

#         self.P = F @ self.P @ F.T + Q

#     def update(self, velocity_measurement, measurement_noise=None):
#         """
#         Update step using a landmark-shift-derived linear velocity measurement.

#         velocity_measurement: (3,) array, e.g. vel_vec from estimate_velocity()
#         measurement_noise: optional override of the default measurement noise std-dev
#         """
#         z = np.asarray(velocity_measurement, dtype=float)
#         r = self.r_meas if measurement_noise is None else measurement_noise
#         R = np.eye(3) * (r ** 2)

#         y = z - self.H_vel @ self.x
#         S = self.H_vel @ self.P @ self.H_vel.T + R
#         K = self.P @ self.H_vel.T @ np.linalg.inv(S)

#         self.x = self.x + K @ y
#         self.P = (np.eye(12) - K @ self.H_vel) @ self.P

#     def update_omega(self, omega_measurement, measurement_noise=None):
#         """
#         Update step using a landmark/vision-derived angular velocity measurement.

#         omega_measurement: (3,) array, angular velocity estimate (rad/s)
#         measurement_noise: optional override of the default omega measurement noise std-dev
#         """
#         z = np.asarray(omega_measurement, dtype=float)
#         r = self.r_omega_meas if measurement_noise is None else measurement_noise
#         R = np.eye(3) * (r ** 2)

#         y = z - self.H_omega @ self.x
#         S = self.H_omega @ self.P @ self.H_omega.T + R
#         K = self.P @ self.H_omega.T @ np.linalg.inv(S)

#         self.x = self.x + K @ y
#         self.P = (np.eye(12) - K @ self.H_omega) @ self.P

#     def get_velocity(self):
#         return self.x[0:3].copy()

#     def get_omega(self):
#         return self.x[3:6].copy()

#     def get_bias(self):
#         return self.x[6:9].copy()

#     def get_gyro_bias(self):
#         return self.x[9:12].copy()# import numpy as np

class VelocityEKF:
    """
    Extended Kalman Filter estimating 3D velocity, fusing:
      - IMU acceleration as the process model input (prediction step)
      - Landmark-shift-derived velocity as the measurement (update step)

    State vector (6,): [vx, vy, vz, bias_ax, bias_ay, bias_az]
      - vx, vy, vz: velocity in the world/navigation frame (m/s)
      - bias_ax/ay/az: slowly-varying accelerometer bias (m/s^2)

    Process model: v_k = v_{k-1} + (a_meas - bias_{k-1}) * dt
                   bias_k = bias_{k-1}  (random walk)
    Measurement model: z_k = v_k  (landmark-based velocity directly observes velocity)
    """

    def __init__(self, initial_velocity=None, initial_bias=None,
                 process_noise_vel=0.05, process_noise_bias=0.001,
                 measurement_noise=0.3):
        """
        initial_velocity: (3,) array, starting velocity guess (default zeros)
        initial_bias: (3,) array, starting accel bias guess (default zeros)
        process_noise_vel: process noise std-dev for velocity (m/s^2), scaled by dt in Q
        process_noise_bias: process noise std-dev for bias random walk (m/s^2 per sqrt(s))
        measurement_noise: std-dev of landmark-derived velocity measurement (m/s)
        """
        v0 = np.zeros(3) if initial_velocity is None else np.asarray(initial_velocity, dtype=float)
        b0 = np.zeros(3) if initial_bias is None else np.asarray(initial_bias, dtype=float)

        self.x = np.concatenate([v0, b0])  # state: [vx,vy,vz,bx,by,bz]
        self.P = np.eye(6) * 1.0            # initial state covariance (fairly uncertain)

        self.q_vel = process_noise_vel
        self.q_bias = process_noise_bias
        self.r_meas = measurement_noise

        # measurement model: z = H x  (we directly observe velocity, not bias)
        self.H = np.hstack([np.eye(3), np.zeros((3, 3))])

    def predict(self, accel_meas, dt):
        """
        Prediction step using IMU acceleration.

        accel_meas: (3,) array, measured linear acceleration in the world/nav frame
                    (already gravity-compensated — see note below).
        dt: time elapsed since the last predict/update, in seconds.
        """
        accel_meas = np.asarray(accel_meas, dtype=float)

        v = self.x[:3]
        b = self.x[3:]

        # process model (linear, so F is just the Jacobian of this same model)
        v_new = v + (accel_meas - b) * dt
        b_new = b  # random walk, no change in the mean

        self.x = np.concatenate([v_new, b_new])

        # state transition Jacobian F (6x6)
        F = np.eye(6)
        F[0:3, 3:6] = -np.eye(3) * dt  # d(v_new)/d(bias) = -dt

        # process noise covariance Q (6x6)
        Q = np.zeros((6, 6))
        Q[0:3, 0:3] = np.eye(3) * (self.q_vel ** 2) * dt
        Q[3:6, 3:6] = np.eye(3) * (self.q_bias ** 2) * dt

        self.P = F @ self.P @ F.T + Q

    def update(self, velocity_measurement, measurement_noise=None):
        """
        Update step using a landmark-shift-derived velocity measurement.

        velocity_measurement: (3,) array, e.g. vel_vec from estimate_velocity()
        measurement_noise: optional override of the default measurement noise std-dev
        """
        z = np.asarray(velocity_measurement, dtype=float)
        r = self.r_meas if measurement_noise is None else measurement_noise
        R = np.eye(3) * (r ** 2)

        y = z - self.H @ self.x                      # innovation
        S = self.H @ self.P @ self.H.T + R            # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)      # Kalman gain

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def get_velocity(self):
        return self.x[:3].copy()

    def get_bias(self):
        return self.x[3:].copy()