#!/usr/bin/env python3
"""state.py — 9-state error-state helpers for the swarm-loc EKF (P2-1).

No rclpy. State vector (plan D3):
    x = [p (3), v (3), ψ (1), b_ψ (1), s (1)]
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

N_STATE = 9
IDX_P = slice(0, 3)
IDX_V = slice(3, 6)
IDX_PSI = 6
IDX_BPSI = 7
IDX_S = 8

STATUS_OK = 0
STATUS_DIVERGED = 1

EPS = 1e-12


def wrap_psi(psi: float) -> float:
    """Wrap yaw to (-pi, pi]."""
    return math.atan2(math.sin(psi), math.cos(psi))


def rpy_to_R(psi: float, pitch: float, roll: float) -> np.ndarray:
    """Body→world rotation R = Rz(ψ) · Ry(pitch) · Rx(roll). Plan §4.1."""
    cψ, sψ = math.cos(psi), math.sin(psi)
    cθ, sθ = math.cos(pitch), math.sin(pitch)
    cφ, sφ = math.cos(roll), math.sin(roll)
    rz = np.array([[cψ, -sψ, 0.0], [sψ, cψ, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    ry = np.array([[cθ, 0.0, sθ], [0.0, 1.0, 0.0], [-sθ, 0.0, cθ]], dtype=np.float64)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cφ, -sφ], [0.0, sφ, cφ]], dtype=np.float64)
    return rz @ ry @ rx


def dR_dpsi(psi: float, pitch: float, roll: float) -> np.ndarray:
    """∂R/∂ψ with R = Rz(ψ) Ry(pitch) Rx(roll)."""
    cψ, sψ = math.cos(psi), math.sin(psi)
    cθ, sθ = math.cos(pitch), math.sin(pitch)
    cφ, sφ = math.cos(roll), math.sin(roll)
    drz = np.array([[-sψ, -cψ, 0.0], [cψ, -sψ, 0.0], [0.0, 0.0, 0.0]], dtype=np.float64)
    ry = np.array([[cθ, 0.0, sθ], [0.0, 1.0, 0.0], [-sθ, 0.0, cθ]], dtype=np.float64)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cφ, -sφ], [0.0, sφ, cφ]], dtype=np.float64)
    return drz @ ry @ rx


def rot_to_rpy(R: np.ndarray) -> tuple:
    """Extract (yaw, pitch, roll) from body→world R = Rz Ry Rx (ZYX)."""
    pitch = math.asin(float(np.clip(-R[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) < 1e-8:
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return yaw, pitch, roll


def launch_position(cfg: dict, drone_id: int) -> np.ndarray:
    launch = cfg["launch"]
    return np.array(
        [
            float(launch["spawn_x0_m"]) + int(drone_id) * float(launch["spacing_m"]),
            float(launch["spawn_y_m"]),
            float(launch["spawn_z_m"]),
        ],
        dtype=np.float64,
    )


def init_covariance(cfg: dict) -> np.ndarray:
    est = cfg["estimator"]
    P = np.zeros((N_STATE, N_STATE), dtype=np.float64)
    sp = float(est["state_init_sigma_p_m"]) ** 2
    sv = float(est["state_init_sigma_v_mps"]) ** 2
    spsi = math.radians(float(est["state_init_sigma_psi_deg"])) ** 2
    ss = float(est["scale_init_sigma"]) ** 2
    P[0, 0] = P[1, 1] = P[2, 2] = sp
    P[3, 3] = P[4, 4] = P[5, 5] = sv
    P[IDX_PSI, IDX_PSI] = spsi
    P[IDX_BPSI, IDX_BPSI] = float(est["yaw_bias_walk_sigma"]) ** 2
    P[IDX_S, IDX_S] = ss
    return P


def symmetrize(P: np.ndarray) -> np.ndarray:
    return 0.5 * (P + P.T)


def project_psd(P: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    w, V = np.linalg.eigh(symmetrize(P))
    return symmetrize(V @ np.diag(np.maximum(w, eps)) @ V.T)


def clamp_position_cov(P: np.ndarray, cfg: dict) -> np.ndarray:
    est = cfg["estimator"]
    lo = float(est["cov_floor_p_m"]) ** 2
    hi = float(est["max_cov_p_m"]) ** 2
    out = P.copy()
    Pp = 0.5 * (out[0:3, 0:3] + out[0:3, 0:3].T)
    w, V = np.linalg.eigh(Pp)
    w = np.clip(w, lo, hi)
    out[0:3, 0:3] = V @ np.diag(w) @ V.T
    return symmetrize(out)


@dataclass
class SwarmState:
    x: np.ndarray
    P: np.ndarray
    roll: float = 0.0
    pitch: float = 0.0
    stamp: float = 0.0
    status: int = STATUS_OK
    drone_id: int = 0
    t_last_gauge_s: float = float("nan")  # last accepted entrance update stamp

    @property
    def p(self) -> np.ndarray:
        return self.x[IDX_P]

    @property
    def v(self) -> np.ndarray:
        return self.x[IDX_V]

    @property
    def psi(self) -> float:
        return float(self.x[IDX_PSI])

    @property
    def b_psi(self) -> float:
        return float(self.x[IDX_BPSI])

    @property
    def s(self) -> float:
        return float(self.x[IDX_S])

    def R(self) -> np.ndarray:
        return rpy_to_R(self.psi, self.pitch, self.roll)

    def copy(self) -> "SwarmState":
        return SwarmState(
            x=self.x.copy(),
            P=self.P.copy(),
            roll=self.roll,
            pitch=self.pitch,
            stamp=self.stamp,
            status=self.status,
            drone_id=self.drone_id,
            t_last_gauge_s=self.t_last_gauge_s,
        )

    @classmethod
    def from_launch(
        cls, cfg: dict, drone_id: int, stamp: float = 0.0
    ) -> "SwarmState":
        x = np.zeros(N_STATE, dtype=np.float64)
        x[IDX_P] = launch_position(cfg, drone_id)
        x[IDX_PSI] = math.radians(float(cfg["launch"]["init_yaw_deg"]))
        x[IDX_S] = float(cfg["estimator"]["scale_init"])
        return cls(
            x=x,
            P=init_covariance(cfg),
            stamp=stamp,
            drone_id=int(drone_id),
        )
