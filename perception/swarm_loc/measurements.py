#!/usr/bin/env python3
"""measurements.py — UWB measurement models + analytic Jacobians (P2-2).

No rclpy. Models (plan §4.3):
    (a) full 3D relative position (range + az + el)
    (b) range-only
    (c) range-rate (OLS slope vs actual timestamps, not sample index)
    (d) mutual yaw (antiparallel bearing vectors)
    (e) entrance anchor — (a) or (b) against the surveyed position

Usage:
    python3 perception/swarm_loc/measurements.py --selftest
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[2]
for _p in ("perception/swarm_loc", "perception/uwb_sim"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

from state import (  # noqa: E402
    EPS,
    IDX_P,
    IDX_PSI,
    IDX_V,
    N_STATE,
    SwarmState,
    dR_dpsi,
    rpy_to_R,
)
from uwb_edges import FLAG_BEARING_VALID, FLAG_PEER_IS_SURVEYED  # noqa: E402
from uwb_model import bearing_xyz  # noqa: E402

# Range-rate constrains only the component along the line of sight, and
# degenerates when the pair is collinear with relative velocity. That is
# expected and is exactly why model (a) exists (plan §4.3c).


@dataclass
class Measurement:
    """EKF innovation pieces: residual = z - h(x), H = dh/dx."""

    name: str
    z: np.ndarray
    h: np.ndarray
    residual: np.ndarray
    H_i: np.ndarray  # (dim, 9) wrt own state
    H_j: np.ndarray  # (dim, 9) wrt neighbor state (zeros if unused)
    R: np.ndarray  # (dim, dim) measurement covariance

    def finite(self) -> bool:
        return bool(
            np.all(np.isfinite(self.residual))
            and np.all(np.isfinite(self.H_i))
            and np.all(np.isfinite(self.H_j))
            and np.all(np.isfinite(self.R))
        )


def _as_vec3(x) -> np.ndarray:
    return np.asarray(x, dtype=np.float64).reshape(3)


def _edge_get(edge, key, default=None):
    if isinstance(edge, dict):
        return edge.get(key, default)
    return edge[key]


def has_full_bearing(edge) -> bool:
    """True iff FLAG_BEARING_VALID and elevation (z) is finite. Plan §4.3a."""
    flags = int(_edge_get(edge, "flags", 0))
    z = float(_edge_get(edge, "z", float("nan")))
    return bool(flags & FLAG_BEARING_VALID) and math.isfinite(z)


def is_surveyed_peer(edge) -> bool:
    return bool(int(_edge_get(edge, "flags", 0)) & FLAG_PEER_IS_SURVEYED)


def spherical_to_cartesian_jacobian(d: float, az: float, el: float) -> np.ndarray:
    """J = dz_body / d[d, az, el], z = bearing_xyz(d, az, el)."""
    ce, se = math.cos(el), math.sin(el)
    ca, sa = math.cos(az), math.sin(az)
    # z = [d ce ca, d ce sa, d se]
    return np.array(
        [
            [ce * ca, -d * ce * sa, -d * se * ca],
            [ce * sa, d * ce * ca, -d * se * sa],
            [se, 0.0, d * ce],
        ],
        dtype=np.float64,
    )


def cartesian_cov_from_spherical(
    d: float, az: float, el: float, sigma_d: float, sigma_az: float, sigma_el: float
) -> np.ndarray:
    """R_cart = J diag(σ_d², σ_az², σ_el²) Jᵀ. Not isotropic."""
    J = spherical_to_cartesian_jacobian(d, az, el)
    S = np.diag([sigma_d**2, sigma_az**2, sigma_el**2])
    return J @ S @ J.T


def z_body_from_edge(edge) -> np.ndarray:
    return np.array(
        [
            float(_edge_get(edge, "x")),
            float(_edge_get(edge, "y")),
            float(_edge_get(edge, "z")),
        ],
        dtype=np.float64,
    )


# ---------------------------------------------------------------------------
# (a) full relative position
# ---------------------------------------------------------------------------

def relpos(
    p_i,
    p_j,
    psi_i: float,
    pitch_i: float,
    roll_i: float,
    z_body,
    sigma_d: float,
    sigma_az: float,
    sigma_el: float,
    d: Optional[float] = None,
    az: Optional[float] = None,
    el: Optional[float] = None,
) -> Measurement:
    """h = R_iᵀ (p_j - p_i), residual = z_body - h. Plan §4.3a."""
    p_i = _as_vec3(p_i)
    p_j = _as_vec3(p_j)
    z = _as_vec3(z_body)
    R = rpy_to_R(psi_i, pitch_i, roll_i)
    dp = p_j - p_i
    h = R.T @ dp
    residual = z - h

    H_i = np.zeros((3, N_STATE), dtype=np.float64)
    H_j = np.zeros((3, N_STATE), dtype=np.float64)
    Rt = R.T
    H_i[:, IDX_P] = -Rt
    H_j[:, IDX_P] = Rt
    # ∂h/∂ψ_i = (∂Rᵀ/∂ψ) dp = (∂R/∂ψ)ᵀ dp
    H_i[:, IDX_PSI] = dR_dpsi(psi_i, pitch_i, roll_i).T @ dp

    if d is None:
        d = float(np.linalg.norm(z))
    if az is None:
        az = math.atan2(float(z[1]), float(z[0]))
    if el is None:
        rxy = math.hypot(float(z[0]), float(z[1]))
        el = math.atan2(float(z[2]), rxy)
    R_cart = cartesian_cov_from_spherical(
        float(d), float(az), float(el), float(sigma_d), float(sigma_az), float(sigma_el)
    )
    return Measurement("relpos", z, h, residual, H_i, H_j, R_cart)


def relpos_from_edge(
    st: SwarmState, p_j, edge, pitch_i: Optional[float] = None, roll_i: Optional[float] = None
) -> Optional[Measurement]:
    if not has_full_bearing(edge):
        return None
    pitch = st.pitch if pitch_i is None else pitch_i
    roll = st.roll if roll_i is None else roll_i
    return relpos(
        st.p,
        p_j,
        st.psi,
        pitch,
        roll,
        z_body_from_edge(edge),
        float(_edge_get(edge, "sigma_range_m")),
        float(_edge_get(edge, "sigma_az_rad")),
        float(_edge_get(edge, "sigma_el_rad")),
        d=float(_edge_get(edge, "range_m")),
        az=float(_edge_get(edge, "azimuth_rad")),
        el=float(_edge_get(edge, "elevation_rad")),
    )


def reciprocal_relpos(
    p_i,
    p_j,
    psi_j: float,
    pitch_j: float,
    roll_j: float,
    z_body_ji,
    sigma_d: float,
    sigma_az: float,
    sigma_el: float,
    d: Optional[float] = None,
    az: Optional[float] = None,
    el: Optional[float] = None,
) -> Optional[Measurement]:
    """D8: peer j measured bearing to us. h = R_jᵀ (p_i - p_j).

    Built from relpos with observer=j, target=i, then H blocks swapped so
    H_i is ours and H_j is the neighbor's.
    """
    m = relpos(p_j, p_i, psi_j, pitch_j, roll_j, z_body_ji, sigma_d, sigma_az, sigma_el, d, az, el)
    m.H_i, m.H_j = m.H_j, m.H_i
    m.name = "reciprocal_relpos"
    return m


# ---------------------------------------------------------------------------
# (b) range-only
# ---------------------------------------------------------------------------

def range_only(p_i, p_j, range_m: float, sigma_range_m: float) -> Optional[Measurement]:
    """h = ‖p_j - p_i‖. Antenna-delay bias is unmodeled (D19)."""
    p_i = _as_vec3(p_i)
    p_j = _as_vec3(p_j)
    dp = p_j - p_i
    r = float(np.linalg.norm(dp))
    if r < EPS:
        return None
    u = dp / r
    h = np.array([r], dtype=np.float64)
    z = np.array([float(range_m)], dtype=np.float64)
    H_i = np.zeros((1, N_STATE), dtype=np.float64)
    H_j = np.zeros((1, N_STATE), dtype=np.float64)
    H_i[:, IDX_P] = -u
    H_j[:, IDX_P] = u
    R = np.array([[float(sigma_range_m) ** 2]], dtype=np.float64)
    return Measurement("range", z, h, z - h, H_i, H_j, R)


# ---------------------------------------------------------------------------
# (c) range-rate
# ---------------------------------------------------------------------------

def range_rate(
    p_i, v_i, p_j, v_j, dprime: float, sigma_dprime: float
) -> Optional[Measurement]:
    """h = (p_j-p_i)·(v_j-v_i) / ‖p_j-p_i‖. Line-of-sight relative speed."""
    p_i = _as_vec3(p_i)
    p_j = _as_vec3(p_j)
    v_i = _as_vec3(v_i)
    v_j = _as_vec3(v_j)
    dp = p_j - p_i
    dv = v_j - v_i
    r = float(np.linalg.norm(dp))
    if r < EPS:
        return None
    u = dp / r
    h_s = float(np.dot(u, dv))
    h = np.array([h_s], dtype=np.float64)
    z = np.array([float(dprime)], dtype=np.float64)
    # ∂h/∂dp = dv/r - (dp·dv) dp / r^3
    dh_ddp = dv / r - (np.dot(dp, dv) * dp) / (r**3)
    H_i = np.zeros((1, N_STATE), dtype=np.float64)
    H_j = np.zeros((1, N_STATE), dtype=np.float64)
    H_i[:, IDX_P] = -dh_ddp
    H_j[:, IDX_P] = dh_ddp
    H_i[:, IDX_V] = -u
    H_j[:, IDX_V] = u
    R = np.array([[float(sigma_dprime) ** 2]], dtype=np.float64)
    return Measurement("range_rate", z, h, z - h, H_i, H_j, R)


def range_rate_regression(
    stamps: Sequence[float],
    ranges: Sequence[float],
    sigmas: Optional[Sequence[float]] = None,
    window: int = 5,
    max_age_s: float = 0.5,
) -> Optional[Tuple[float, float]]:
    """OLS slope of range vs actual timestamps. Returns (dprime, sigma_dprime).

    Uses the last `window` samples. Skips if fewer than `window` points or if
    those points span more than `max_age_s` (plan §4.3c).
    """
    t = np.asarray(stamps, dtype=np.float64)
    d = np.asarray(ranges, dtype=np.float64)
    if t.size != d.size or t.size < int(window):
        return None
    t = t[-int(window) :]
    d = d[-int(window) :]
    if not np.all(np.isfinite(t)) or not np.all(np.isfinite(d)):
        return None
    span = float(t[-1] - t[0])
    if span < EPS or span > float(max_age_s):
        return None
    t0 = float(np.mean(t))
    tc = t - t0
    sxx = float(np.dot(tc, tc))
    if sxx < EPS:
        return None
    dc = d - float(np.mean(d))
    slope = float(np.dot(tc, dc) / sxx)
    if sigmas is None:
        resid = dc - slope * tc
        dof = max(t.size - 2, 1)
        sigma2 = float(np.dot(resid, resid) / dof)
    else:
        s = np.asarray(sigmas, dtype=np.float64)[-int(window) :]
        sigma2 = float(np.mean(s**2))
    sigma_slope = math.sqrt(max(sigma2 / sxx, EPS))
    return slope, sigma_slope


# ---------------------------------------------------------------------------
# (d) mutual yaw
# ---------------------------------------------------------------------------

def _unit(v: np.ndarray) -> Optional[np.ndarray]:
    n = float(np.linalg.norm(v))
    if n < EPS:
        return None
    return v / n


def _unit_jacobian(z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    nrm = float(np.linalg.norm(z))
    u = z / nrm
    Jn = (np.eye(3) - np.outer(u, u)) / nrm
    return u, Jn


def mutual_yaw(
    psi_i: float,
    pitch_i: float,
    roll_i: float,
    z_body_ij,
    psi_j: float,
    pitch_j: float,
    roll_j: float,
    z_body_ji,
    R_cart_ij: Optional[np.ndarray] = None,
    R_cart_ji: Optional[np.ndarray] = None,
) -> Optional[Measurement]:
    """r = u_ij + u_ji, u = R · unit(z_body). Sensitive to (ψ_i - ψ_j)."""
    zij = _as_vec3(z_body_ij)
    zji = _as_vec3(z_body_ji)
    nij = _unit(zij)
    nji = _unit(zji)
    if nij is None or nji is None:
        return None
    Ri = rpy_to_R(psi_i, pitch_i, roll_i)
    Rj = rpy_to_R(psi_j, pitch_j, roll_j)
    uij = Ri @ nij
    uji = Rj @ nji
    # z = 0, h = -(u_ij + u_ji)  →  residual z-h = u_ij + u_ji (plan residual)
    h = -(uij + uji)
    z = np.zeros(3, dtype=np.float64)
    residual = z - h

    H_i = np.zeros((3, N_STATE), dtype=np.float64)
    H_j = np.zeros((3, N_STATE), dtype=np.float64)
    H_i[:, IDX_PSI] = -(dR_dpsi(psi_i, pitch_i, roll_i) @ nij)
    H_j[:, IDX_PSI] = -(dR_dpsi(psi_j, pitch_j, roll_j) @ nji)

    if R_cart_ij is None:
        R_cart_ij = np.eye(3) * 1e-4
    if R_cart_ji is None:
        R_cart_ji = np.eye(3) * 1e-4
    _, Jn_ij = _unit_jacobian(zij)
    _, Jn_ji = _unit_jacobian(zji)
    Pu_ij = Ri @ (Jn_ij @ R_cart_ij @ Jn_ij.T) @ Ri.T
    Pu_ji = Rj @ (Jn_ji @ R_cart_ji @ Jn_ji.T) @ Rj.T
    Rm = Pu_ij + Pu_ji
    Rm = 0.5 * (Rm + Rm.T)
    return Measurement("mutual_yaw", z, h, residual, H_i, H_j, Rm)


# ---------------------------------------------------------------------------
# (e) entrance anchor
# ---------------------------------------------------------------------------

def entrance_from_edge(st: SwarmState, edge, cfg: dict) -> Optional[Measurement]:
    """(a) or (b) against entrance.position_xyz_m. Identify by FLAG_PEER_IS_SURVEYED."""
    if not is_surveyed_peer(edge):
        return None
    p_j = np.array(cfg["entrance"]["position_xyz_m"], dtype=np.float64)
    if has_full_bearing(edge):
        m = relpos_from_edge(st, p_j, edge)
        if m is not None:
            m.name = "entrance_relpos"
        return m
    rng = _edge_get(edge, "range_m", None)
    sig = _edge_get(edge, "sigma_range_m", None)
    if rng is None or sig is None or not math.isfinite(float(rng)):
        return None
    m = range_only(st.p, p_j, float(rng), float(sig))
    if m is not None:
        m.name = "entrance_range"
    return m


def from_edge(
    st: SwarmState,
    p_j,
    edge,
    cfg: dict,
    v_j=None,
    dprime: Optional[Tuple[float, float]] = None,
) -> Optional[Measurement]:
    """Pick (e), else (a), else (b). Range-rate is separate (needs a window)."""
    if is_surveyed_peer(edge):
        return entrance_from_edge(st, edge, cfg)
    if has_full_bearing(edge):
        return relpos_from_edge(st, p_j, edge)
    rng = _edge_get(edge, "range_m", None)
    sig = _edge_get(edge, "sigma_range_m", None)
    if rng is None or not math.isfinite(float(rng)):
        return None
    return range_only(st.p, p_j, float(rng), float(sig if sig is not None else 0.1))


# ---------------------------------------------------------------------------
# selftest
# ---------------------------------------------------------------------------

def _numdiff_h(fn, x0: np.ndarray, eps: float = 1e-7) -> np.ndarray:
    h0 = np.asarray(fn(x0), dtype=np.float64).reshape(-1)
    H = np.zeros((h0.size, x0.size), dtype=np.float64)
    for j in range(x0.size):
        xp = x0.copy()
        xm = x0.copy()
        xp[j] += eps
        xm[j] -= eps
        H[:, j] = (np.asarray(fn(xp)).reshape(-1) - np.asarray(fn(xm)).reshape(-1)) / (
            2.0 * eps
        )
    return H


def _rand_geom(rng: np.random.Generator):
    p_i = rng.normal(0.0, 3.0, size=3)
    # keep range in [0.8, 12] m so models stay well-conditioned
    direction = rng.normal(0.0, 1.0, size=3)
    direction /= max(float(np.linalg.norm(direction)), EPS)
    dist = float(rng.uniform(0.8, 12.0))
    p_j = p_i + direction * dist
    v_i = rng.normal(0.0, 1.5, size=3)
    v_j = rng.normal(0.0, 1.5, size=3)
    psi_i = float(rng.uniform(-math.pi, math.pi))
    psi_j = float(rng.uniform(-math.pi, math.pi))
    pitch_i = float(rng.uniform(-0.4, 0.4))
    pitch_j = float(rng.uniform(-0.4, 0.4))
    roll_i = float(rng.uniform(-0.4, 0.4))
    roll_j = float(rng.uniform(-0.4, 0.4))
    return {
        "p_i": p_i,
        "p_j": p_j,
        "v_i": v_i,
        "v_j": v_j,
        "psi_i": psi_i,
        "psi_j": psi_j,
        "pitch_i": pitch_i,
        "pitch_j": pitch_j,
        "roll_i": roll_i,
        "roll_j": roll_j,
    }


def run_selftest() -> int:
    ok = True
    n_pass = 0
    n_fail = 0

    def check(name: str, cond: bool, detail: str = ""):
        nonlocal ok, n_pass, n_fail
        if not cond:
            ok = False
            n_fail += 1
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))
        else:
            n_pass += 1
            print(f"[selftest] PASS {name}")

    # --- spherical -> cartesian, hand-computed axis-aligned case ---
    d, az, el = 1.0, 0.0, 0.0
    sd, saz, sel = 0.1, 0.01, 0.02
    Rc = cartesian_cov_from_spherical(d, az, el, sd, saz, sel)
    # J = I at this point → R = diag(sd^2, saz^2, sel^2)
    R_hand = np.diag([sd**2, saz**2, sel**2])
    check(
        "1 cart cov dead-ahead",
        np.allclose(Rc, R_hand, atol=1e-12),
        f"Rc=\n{Rc}\nhand=\n{R_hand}",
    )
    z = np.array(bearing_xyz(d, az, el))
    check("1b bearing_xyz dead-ahead", np.allclose(z, [1.0, 0.0, 0.0]))

    d, az, el = 2.0, math.pi / 2, 0.0
    Rc = cartesian_cov_from_spherical(d, az, el, sd, saz, sel)
    # J = [[0,-2,0],[1,0,0],[0,0,2]] → R = diag(4 saz^2, sd^2, 4 sel^2)
    R_hand = np.diag([4.0 * saz**2, sd**2, 4.0 * sel**2])
    check(
        "2 cart cov az=+pi/2",
        np.allclose(Rc, R_hand, atol=1e-12),
        f"diag={np.diag(Rc)} hand={np.diag(R_hand)}",
    )
    # Realistic UWB-like sigmas: short along range, long transversely.
    d, az, el = 5.0, 0.0, 0.0
    sd, saz, sel = 0.087, math.radians(4.8), math.radians(4.8)
    Rc = cartesian_cov_from_spherical(d, az, el, sd, saz, sel)
    check(
        "2b ellipsoid long in transverse",
        float(Rc[1, 1]) > float(Rc[0, 0]) and float(Rc[2, 2]) > float(Rc[0, 0]),
        f"diag={np.diag(Rc)}",
    )

    # --- (a) residual identity at matching geometry ---
    p_i = np.zeros(3)
    p_j = np.array([2.0, 0.5, -0.3])
    psi, pitch, roll = 0.2, 0.05, -0.04
    R = rpy_to_R(psi, pitch, roll)
    z_true = R.T @ (p_j - p_i)
    m = relpos(p_i, p_j, psi, pitch, roll, z_true, 0.05, 0.02, 0.02)
    check("3 relpos residual zero", float(np.linalg.norm(m.residual)) < 1e-12)
    check("3b relpos finite", m.finite())
    z_ji_true = rpy_to_R(0.3, -0.05, 0.02).T @ (p_i - p_j)
    mr = reciprocal_relpos(p_i, p_j, 0.3, -0.05, 0.02, z_ji_true, 0.05, 0.02, 0.02)
    check("3c reciprocal residual zero", float(np.linalg.norm(mr.residual)) < 1e-12)
    check("3d reciprocal name", mr.name == "reciprocal_relpos")

    # --- 100 randomized Jacobian checks ---
    rng = np.random.default_rng(2)
    max_a = max_b = max_c = max_d = max_r = 0.0
    n_geom = 100
    failed = []
    for k in range(n_geom):
        g = _rand_geom(rng)
        # (a)
        R = rpy_to_R(g["psi_i"], g["pitch_i"], g["roll_i"])
        z_b = R.T @ (g["p_j"] - g["p_i"])
        ma = relpos(
            g["p_i"], g["p_j"], g["psi_i"], g["pitch_i"], g["roll_i"], z_b, 0.08, 0.03, 0.03
        )
        x_i = np.zeros(N_STATE)
        x_i[IDX_P] = g["p_i"]
        x_i[IDX_PSI] = g["psi_i"]
        x_j = np.zeros(N_STATE)
        x_j[IDX_P] = g["p_j"]

        def h_a_i(x, g=g, z_b=z_b):
            return relpos(
                x[IDX_P], g["p_j"], float(x[IDX_PSI]), g["pitch_i"], g["roll_i"], z_b, 0.08, 0.03, 0.03
            ).h

        def h_a_j(x, g=g, z_b=z_b):
            return relpos(
                g["p_i"], x[IDX_P], g["psi_i"], g["pitch_i"], g["roll_i"], z_b, 0.08, 0.03, 0.03
            ).h

        Ha_i = _numdiff_h(h_a_i, x_i)
        Ha_j = _numdiff_h(h_a_j, x_j)
        ea_i = float(np.max(np.abs(ma.H_i - Ha_i)))
        ea_j = float(np.max(np.abs(ma.H_j - Ha_j)))
        max_a = max(max_a, ea_i, ea_j)
        if ea_i >= 1e-6 or ea_j >= 1e-6:
            failed.append(("a", k, ea_i, ea_j))

        # (b)
        mb = range_only(g["p_i"], g["p_j"], float(np.linalg.norm(g["p_j"] - g["p_i"])), 0.08)
        assert mb is not None

        def h_b_i(x, g=g):
            return range_only(x[IDX_P], g["p_j"], 0.0, 0.08).h

        def h_b_j(x, g=g):
            return range_only(g["p_i"], x[IDX_P], 0.0, 0.08).h

        Hb_i = _numdiff_h(h_b_i, x_i)
        Hb_j = _numdiff_h(h_b_j, x_j)
        eb_i = float(np.max(np.abs(mb.H_i - Hb_i)))
        eb_j = float(np.max(np.abs(mb.H_j - Hb_j)))
        max_b = max(max_b, eb_i, eb_j)
        if eb_i >= 1e-6 or eb_j >= 1e-6:
            failed.append(("b", k, eb_i, eb_j))

        # (c)
        x_i[IDX_V] = g["v_i"]
        x_j[IDX_V] = g["v_j"]
        mc = range_rate(g["p_i"], g["v_i"], g["p_j"], g["v_j"], 0.0, 0.1)
        assert mc is not None

        def h_c_i(x, g=g):
            return range_rate(x[IDX_P], x[IDX_V], g["p_j"], g["v_j"], 0.0, 0.1).h

        def h_c_j(x, g=g):
            return range_rate(g["p_i"], g["v_i"], x[IDX_P], x[IDX_V], 0.0, 0.1).h

        Hc_i = _numdiff_h(h_c_i, x_i)
        Hc_j = _numdiff_h(h_c_j, x_j)
        ec_i = float(np.max(np.abs(mc.H_i - Hc_i)))
        ec_j = float(np.max(np.abs(mc.H_j - Hc_j)))
        max_c = max(max_c, ec_i, ec_j)
        if ec_i >= 1e-6 or ec_j >= 1e-6:
            failed.append(("c", k, ec_i, ec_j))

        # (d)
        nij = _unit(z_b)
        # facing pair: reverse body vector expressed in j's frame
        Rj = rpy_to_R(g["psi_j"], g["pitch_j"], g["roll_j"])
        z_ji = Rj.T @ (g["p_i"] - g["p_j"])
        md = mutual_yaw(
            g["psi_i"],
            g["pitch_i"],
            g["roll_i"],
            z_b,
            g["psi_j"],
            g["pitch_j"],
            g["roll_j"],
            z_ji,
        )
        assert md is not None and nij is not None
        x_i2 = np.zeros(N_STATE)
        x_i2[IDX_PSI] = g["psi_i"]
        x_j2 = np.zeros(N_STATE)
        x_j2[IDX_PSI] = g["psi_j"]

        def h_d_i(x, g=g, z_b=z_b, z_ji=z_ji):
            return mutual_yaw(
                float(x[IDX_PSI]),
                g["pitch_i"],
                g["roll_i"],
                z_b,
                g["psi_j"],
                g["pitch_j"],
                g["roll_j"],
                z_ji,
            ).h

        def h_d_j(x, g=g, z_b=z_b, z_ji=z_ji):
            return mutual_yaw(
                g["psi_i"],
                g["pitch_i"],
                g["roll_i"],
                z_b,
                float(x[IDX_PSI]),
                g["pitch_j"],
                g["roll_j"],
                z_ji,
            ).h

        Hd_i = _numdiff_h(h_d_i, x_i2)
        Hd_j = _numdiff_h(h_d_j, x_j2)
        ed_i = float(np.max(np.abs(md.H_i - Hd_i)))
        ed_j = float(np.max(np.abs(md.H_j - Hd_j)))
        max_d = max(max_d, ed_i, ed_j)
        if ed_i >= 1e-6 or ed_j >= 1e-6:
            failed.append(("d", k, ed_i, ed_j))

        # D8 reciprocal: same geometry as (a) but observer is j
        z_ji = rpy_to_R(g["psi_j"], g["pitch_j"], g["roll_j"]).T @ (g["p_i"] - g["p_j"])
        mrec = reciprocal_relpos(
            g["p_i"],
            g["p_j"],
            g["psi_j"],
            g["pitch_j"],
            g["roll_j"],
            z_ji,
            0.08,
            0.03,
            0.03,
        )
        x_i_r = np.zeros(N_STATE)
        x_i_r[IDX_P] = g["p_i"]
        x_j_r = np.zeros(N_STATE)
        x_j_r[IDX_P] = g["p_j"]
        x_j_r[IDX_PSI] = g["psi_j"]

        def h_r_i(x, g=g, z_ji=z_ji):
            return reciprocal_relpos(
                x[IDX_P],
                g["p_j"],
                g["psi_j"],
                g["pitch_j"],
                g["roll_j"],
                z_ji,
                0.08,
                0.03,
                0.03,
            ).h

        def h_r_j(x, g=g, z_ji=z_ji):
            return reciprocal_relpos(
                g["p_i"],
                x[IDX_P],
                float(x[IDX_PSI]),
                g["pitch_j"],
                g["roll_j"],
                z_ji,
                0.08,
                0.03,
                0.03,
            ).h

        Hr_i = _numdiff_h(h_r_i, x_i_r)
        Hr_j = _numdiff_h(h_r_j, x_j_r)
        er_i = float(np.max(np.abs(mrec.H_i - Hr_i)))
        er_j = float(np.max(np.abs(mrec.H_j - Hr_j)))
        max_r = max(max_r, er_i, er_j)
        if er_i >= 1e-6 or er_j >= 1e-6:
            failed.append(("r", k, er_i, er_j))

    check(
        "4 Jac (a) 100 geom <1e-6",
        max_a < 1e-6 and not any(f[0] == "a" for f in failed),
        f"max={max_a:.3e} nfail={sum(1 for f in failed if f[0]=='a')}",
    )
    check(
        "5 Jac (b) 100 geom <1e-6",
        max_b < 1e-6 and not any(f[0] == "b" for f in failed),
        f"max={max_b:.3e}",
    )
    check(
        "6 Jac (c) 100 geom <1e-6",
        max_c < 1e-6 and not any(f[0] == "c" for f in failed),
        f"max={max_c:.3e}",
    )
    check(
        "7 Jac (d) 100 geom <1e-6",
        max_d < 1e-6 and not any(f[0] == "d" for f in failed),
        f"max={max_d:.3e}",
    )
    check(
        "7b Jac reciprocal 100 geom <1e-6",
        max_r < 1e-6 and not any(f[0] == "r" for f in failed),
        f"max={max_r:.3e} nfail={sum(1 for f in failed if f[0]=='r')}",
    )

    # facing pair: residual ~ 0 when both attitudes consistent with geometry
    g = _rand_geom(np.random.default_rng(7))
    Ri = rpy_to_R(g["psi_i"], g["pitch_i"], g["roll_i"])
    Rj = rpy_to_R(g["psi_j"], g["pitch_j"], g["roll_j"])
    z_ij = Ri.T @ (g["p_j"] - g["p_i"])
    z_ji = Rj.T @ (g["p_i"] - g["p_j"])
    md = mutual_yaw(
        g["psi_i"], g["pitch_i"], g["roll_i"], z_ij,
        g["psi_j"], g["pitch_j"], g["roll_j"], z_ji,
    )
    check(
        "8 mutual-yaw residual ~0 when consistent",
        md is not None and float(np.linalg.norm(md.residual)) < 1e-10,
        f"||r||={None if md is None else np.linalg.norm(md.residual)}",
    )

    # --- range-rate regression recovers known d' ---
    rng = np.random.default_rng(11)
    t = np.array([0.00, 0.08, 0.17, 0.29, 0.40])  # irregular, as the scheduler is
    dprime_true = 0.37
    d0 = 4.2
    sigma_r = 0.002
    d_meas = d0 + dprime_true * t + rng.normal(0.0, sigma_r, size=t.size)
    got = range_rate_regression(t, d_meas, sigmas=[sigma_r] * 5, window=5, max_age_s=0.5)
    check("9 regression returned", got is not None)
    if got is not None:
        slope, sig = got
        check(
            "9b recovered dprime",
            abs(slope - dprime_true) < 0.02,
            f"slope={slope:.4f} true={dprime_true} sig={sig:.4f}",
        )

    check("10 skip too few", range_rate_regression([0.0, 0.1], [1.0, 1.1], window=5) is None)
    t_old = np.linspace(0.0, 2.0, 5)
    check(
        "10b skip span>max_age",
        range_rate_regression(t_old, np.ones(5), window=5, max_age_s=0.5) is None,
    )

    # --- (e) + flag routing ---
    cfg = {
        "entrance": {"position_xyz_m": [-2.0, 0.0, 0.30], "sigma_m": 0.01},
        "estimator": {
            "state_init_sigma_p_m": 0.05,
            "state_init_sigma_v_mps": 0.05,
            "state_init_sigma_psi_deg": 5.0,
            "yaw_bias_walk_sigma": 1e-4,
            "scale_init": 1.0,
            "scale_init_sigma": 0.02,
            "cov_floor_p_m": 0.01,
            "max_cov_p_m": 50.0,
        },
        "launch": {
            "spawn_x0_m": 0.0,
            "spawn_y_m": 0.0,
            "spawn_z_m": 0.5,
            "spacing_m": 1.5,
            "init_yaw_deg": 0.0,
        },
    }
    st = SwarmState.from_launch(cfg, 0)
    p_ent = np.array([-2.0, 0.0, 0.30])
    dp = p_ent - st.p
    rng_true = float(np.linalg.norm(dp))
    edge_range = {
        "flags": FLAG_PEER_IS_SURVEYED,
        "range_m": rng_true,
        "sigma_range_m": 0.09,
        "x": float("nan"),
        "y": float("nan"),
        "z": float("nan"),
        "azimuth_rad": float("nan"),
        "elevation_rad": float("nan"),
        "sigma_az_rad": float("nan"),
        "sigma_el_rad": float("nan"),
    }
    me = entrance_from_edge(st, edge_range, cfg)
    check("11 entrance range-only", me is not None and me.name == "entrance_range")
    check(
        "11b uses config position not a fake peer",
        me is not None and abs(float(me.h[0]) - rng_true) < 1e-12,
    )
    check("11c ignores non-surveyed", entrance_from_edge(st, {"flags": 0, "range_m": 1.0, "sigma_range_m": 0.1, "z": float("nan")}, cfg) is None)

    z_ent = st.R().T @ dp
    xz, yz, zz = z_ent
    edge_brg = {
        "flags": FLAG_PEER_IS_SURVEYED | FLAG_BEARING_VALID,
        "x": xz,
        "y": yz,
        "z": zz,
        "range_m": rng_true,
        "azimuth_rad": math.atan2(yz, xz),
        "elevation_rad": math.atan2(zz, math.hypot(xz, yz)),
        "sigma_range_m": 0.09,
        "sigma_az_rad": 0.08,
        "sigma_el_rad": 0.08,
    }
    meb = entrance_from_edge(st, edge_brg, cfg)
    check("12 entrance relpos", meb is not None and meb.name == "entrance_relpos")
    check(
        "12b entrance relpos residual ~0",
        meb is not None and float(np.linalg.norm(meb.residual)) < 1e-6,
    )

    # azimuth-only (z NaN) must NOT use (a)
    edge_az = dict(edge_brg)
    edge_az["flags"] = FLAG_BEARING_VALID  # not surveyed
    edge_az["z"] = float("nan")
    edge_az["peer_pos_trap"] = np.array([99.0, 99.0, 99.0])
    check("13 az-only not full bearing", not has_full_bearing(edge_az))
    ma_skip = relpos_from_edge(st, p_ent, edge_az)
    check("13b relpos_from_edge skips az-only", ma_skip is None)
    mb2 = from_edge(st, p_ent, edge_az, cfg)
    check("13c from_edge falls back to range", mb2 is not None and mb2.name == "range")

    # NaN guard
    mnan = relpos(p_i, p_j, 0.0, 0.0, 0.0, np.array([1.0, float("nan"), 0.0]), 0.1, 0.1, 0.1)
    check("14 NaN residual fails finite()", not mnan.finite())

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
