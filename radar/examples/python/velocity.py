import numpy as np

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