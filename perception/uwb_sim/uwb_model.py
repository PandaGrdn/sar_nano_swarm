#!/usr/bin/env python3
"""uwb_model.py — pure UWB PDoA measurement model + scheduler (Phase 1 M4).

No rclpy import. Fully testable offline via --selftest.

See .cursor/docs/M4_UWB_Relative_Positioning_Implementation_Plan.md
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

from uwb_edges import (
    EDGE_DTYPE,
    FLAG_BEARING_VALID,
    FLAG_IN_AOA_CONE,
    FLAG_LOS,
    FLAG_PEER_IS_STATIC,
    FLAG_PEER_IS_SURVEYED,
    FLAG_RANGE_VALID,
)

C_LIGHT = 299792458.0
EPS = 1e-12


def quat_to_rot_matrix(q: Tuple[float, float, float, float]) -> np.ndarray:
    """Body -> world rotation matrix from unit quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def yaw_to_rot_matrix(yaw_rad: float) -> np.ndarray:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def world_to_body(R_bw: np.ndarray, d_w: np.ndarray) -> np.ndarray:
    """d_b = R_bw^T @ d_w"""
    return R_bw.T @ d_w


def compute_peer_geometry(
    p_observer: np.ndarray,
    R_observer_bw: np.ndarray,
    p_peer: np.ndarray,
    boresight_axis: np.ndarray,
    aoa_fov_deg: float,
) -> dict:
    """Compute true range/az/el and cone membership in observer body frame."""
    d_w = p_peer - p_observer
    d_b = world_to_body(R_observer_bw, d_w)
    r = float(np.linalg.norm(d_b))
    if r < EPS:
        az = el = theta = 0.0
        in_cone = True
    else:
        az = math.atan2(d_b[1], d_b[0])
        el = math.asin(float(np.clip(d_b[2] / r, -1.0, 1.0)))
        b_unit = boresight_axis / max(np.linalg.norm(boresight_axis), EPS)
        cos_theta = float(np.clip(np.dot(d_b / r, b_unit), -1.0, 1.0))
        theta = math.acos(cos_theta)
        in_cone = theta <= math.radians(aoa_fov_deg) / 2.0
    return {
        "d_b": d_b,
        "range_m": r,
        "azimuth_rad": az,
        "elevation_rad": el,
        "theta_rad": theta,
        "in_cone": in_cone,
    }


def sigma_ang(theta_rad: float, cfg: dict) -> float:
    """Angular measurement sigma (rad) as a function of off-boresight angle."""
    sigma_b = math.radians(float(cfg["sigma_boresight_deg"]))
    model = cfg.get("angle_error_model", "inv_cos")
    aoa_fov = float(cfg["aoa_fov_deg"])
    cos_floor = math.cos(math.radians(aoa_fov) / 2.0)
    if model == "constant":
        return sigma_b
    if model == "linear":
        return sigma_b * (1.0 + theta_rad / max(math.radians(aoa_fov) / 2.0, EPS))
    # inv_cos (default)
    cos_t = max(math.cos(theta_rad), cos_floor)
    return sigma_b / cos_t


def segment_intersects_box(
    a: np.ndarray, b: np.ndarray, box: dict
) -> bool:
    """Slab method: segment a->b vs axis-aligned box."""
    d = b - a
    t_enter, t_exit = 0.0, 1.0
    mins = [box["x_min"], box["y_min"], box["z_min"]]
    maxs = [box["x_max"], box["y_max"], box["z_max"]]
    for k in range(3):
        if abs(d[k]) < EPS:
            if a[k] < mins[k] or a[k] > maxs[k]:
                return False
            continue
        t0 = (mins[k] - a[k]) / d[k]
        t1 = (maxs[k] - a[k]) / d[k]
        if t0 > t1:
            t0, t1 = t1, t0
        t_enter = max(t_enter, t0)
        t_exit = min(t_exit, t1)
    return t_enter <= t_exit and t_exit >= 0.0 and t_enter <= 1.0


def los_check_boxes(p_a: np.ndarray, p_b: np.ndarray, boxes: List[dict]) -> bool:
    for box in boxes:
        if segment_intersects_box(p_a, p_b, box):
            return False
    return True


_mesh_cache: dict = {}


def los_check_mesh(p_a: np.ndarray, p_b: np.ndarray, cfg: dict, logger=None) -> bool:
    mesh_path = cfg.get("mesh_path", "")
    root = cfg.get("_swarm_root", "")
    if not mesh_path:
        return True
    key = (mesh_path, root)
    if key not in _mesh_cache:
        try:
            import trimesh
            from trimesh.ray.ray_pyembree import RayMeshIntersector
        except ImportError:
            if logger:
                logger("WARNING: trimesh/embreex unavailable — falling back to always_los")
            _mesh_cache[key] = None
            return True
        import os

        full = mesh_path if os.path.isabs(mesh_path) else os.path.join(root, mesh_path)
        try:
            mesh = trimesh.load(full, force="mesh")
            intersector = RayMeshIntersector(mesh)
            _mesh_cache[key] = intersector
        except Exception as exc:
            if logger:
                logger(f"WARNING: mesh load failed ({full}): {exc} — falling back to always_los")
            _mesh_cache[key] = None
            return True

    intersector = _mesh_cache[key]
    if intersector is None:
        return True
    direction = p_b - p_a
    dist = float(np.linalg.norm(direction))
    if dist < EPS:
        return True
    direction = direction / dist
    origins = np.array([p_a])
    directions = np.array([direction])
    try:
        loc, _, _ = intersector.intersects_location(origins, directions, multiple_hits=False)
    except TypeError:
        loc, _, _ = intersector.intersects_location(origins, directions)
    if len(loc) == 0:
        return True
    hit_dist = float(np.linalg.norm(loc[0] - p_a))
    eps = float(cfg.get("surface_eps_m", 0.05))
    return hit_dist >= dist - eps


def los_check(p_a: np.ndarray, p_b: np.ndarray, cfg: dict, logger=None) -> bool:
    model = cfg.get("los_model", "boxes")
    if model == "always_los":
        return True
    if model == "mesh":
        return los_check_mesh(p_a, p_b, cfg, logger=logger)
    return los_check_boxes(p_a, p_b, cfg.get("occluder_boxes", []))


def pair_key(i: int, j: int) -> Tuple[int, int]:
    return (min(i, j), max(i, j))


def bearing_xyz(range_m: float, az: float, el: float) -> Tuple[float, float, float]:
    ce = math.cos(el)
    return (
        range_m * ce * math.cos(az),
        range_m * ce * math.sin(az),
        range_m * math.sin(el),
    )


def nan_edge_fields() -> dict:
    nan = float("nan")
    return {
        "x": nan,
        "y": nan,
        "z": nan,
        "azimuth_rad": nan,
        "elevation_rad": nan,
        "sigma_az_rad": nan,
        "sigma_el_rad": nan,
    }


@dataclass
class DeviceState:
    device_id: int
    position: np.ndarray
    R_bw: np.ndarray
    is_static: bool
    peer_type: str  # drone | entrance | landed
    active: bool = True


@dataclass
class UwbModel:
    cfg: dict
    rng: np.random.Generator
    antenna_delay_bias: Dict[int, float] = field(default_factory=dict)
    scheduled_pairs: Set[Tuple[int, int]] = field(default_factory=set)
    next_due: Dict[Tuple[int, int], float] = field(default_factory=dict)
    sim_time: float = 0.0
    n_pairs: int = 0
    effective_rate: float = 0.0
    lambda_m: float = 0.0
    phase_ambiguity_warned: bool = False
    _los_warned: bool = False

    def __post_init__(self):
        hz = float(self.cfg["channel_centre_hz"])
        self.lambda_m = C_LIGHT / hz
        spacing = float(self.cfg["antenna_spacing_m"])
        if spacing > self.lambda_m / 2.0 + 1e-9 and not self.phase_ambiguity_warned:
            self.phase_ambiguity_warned = True
        if self.cfg.get("los_model") == "always_los" and not self._los_warned:
            self._los_warned = True

    @classmethod
    def from_config(cls, cfg: dict, seed: Optional[int] = None) -> "UwbModel":
        s = int(cfg.get("seed", 0) if seed is None else seed)
        rng = np.random.default_rng(s)
        model = cls(cfg=cfg, rng=rng)
        sigma_bias = float(cfg.get("antenna_delay_bias_sigma_m", 0.0))
        device_ids: List[int] = []
        n_drones = int(cfg.get("_num_drones", 0))
        device_ids.extend(range(n_drones))
        for peer in cfg.get("static_peers", []):
            device_ids.append(int(peer["id"]))
        for did in device_ids:
            if sigma_bias > 0.0:
                model.antenna_delay_bias[did] = float(rng.normal(0.0, sigma_bias))
            else:
                model.antenna_delay_bias[did] = 0.0
        return model

    def boresight_unit(self) -> np.ndarray:
        b = np.array(self.cfg["boresight_axis"], dtype=np.float64)
        n = np.linalg.norm(b)
        return b / max(n, EPS)

    def link_budget_dropout_prob(self, r: float) -> float:
        max_r = float(self.cfg["max_range_m"])
        p_max = float(self.cfg["p_dropout_at_max_range"])
        if max_r <= 0.0:
            return 0.0
        p = p_max * (r / max_r) ** 2
        return float(np.clip(p, 0.0, 1.0))

    def update_scheduled_pairs(self, devices: Dict[int, DeviceState]) -> None:
        active = {did: d for did, d in devices.items() if d.active}
        ids = sorted(active.keys())
        max_r = float(self.cfg["max_range_m"])
        k_cap = int(self.cfg["max_neighbors_per_drone"])

        neighbours: Dict[int, List[Tuple[float, int]]] = {did: [] for did in ids}
        candidates: Set[Tuple[int, int]] = set()
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a, b = ids[i], ids[j]
                pa, pb = active[a].position, active[b].position
                r = float(np.linalg.norm(pb - pa))
                if r < max_r:
                    candidates.add((a, b))
                    neighbours[a].append((r, b))
                    neighbours[b].append((r, a))

        for did in neighbours:
            neighbours[did].sort(key=lambda x: x[0])

        kept: Set[Tuple[int, int]] = set()
        for a, b in candidates:
            a_nei = [p for _, p in neighbours[a][:k_cap]]
            b_nei = [p for _, p in neighbours[b][:k_cap]]
            if b in a_nei and a in b_nei:
                kept.add(pair_key(a, b))

        self.scheduled_pairs = kept
        self.n_pairs = len(kept)
        ranging_rate = float(self.cfg["ranging_rate_hz"])
        budget = float(self.cfg["max_exchanges_per_s"])
        self.effective_rate = min(
            ranging_rate, budget / max(self.n_pairs, 1)
        )

        for pk in kept:
            if pk not in self.next_due:
                self.next_due[pk] = self.sim_time

        stale = [pk for pk in self.next_due if pk not in kept]
        for pk in stale:
            del self.next_due[pk]

    def _peer_flags(self, peer: DeviceState) -> int:
        flags = FLAG_RANGE_VALID
        if peer.peer_type == "entrance":
            flags |= FLAG_PEER_IS_SURVEYED
        if peer.is_static:
            flags |= FLAG_PEER_IS_STATIC
        return flags

    def _apply_phase_wrap(self, az: float, el: float, theta: float) -> Tuple[float, float]:
        if not self.cfg.get("model_phase_wrap", False):
            return az, el
        spacing = float(self.cfg["antenna_spacing_m"])
        lam = self.lambda_m
        if spacing <= lam / 2.0:
            return az, el
        # Simplified alias: fold azimuth by half-wavelength ambiguity
        period = math.asin(min(1.0, lam / (2.0 * spacing)))
        if period > EPS:
            az = ((az + period) % (2.0 * period)) - period
        return az, el

    def execute_exchange(
        self,
        obs: DeviceState,
        peer: DeviceState,
        shared: dict,
    ) -> Optional[dict]:
        """Build one edge row for observer obs measuring peer."""
        geom = compute_peer_geometry(
            obs.position,
            obs.R_bw,
            peer.position,
            self.boresight_unit(),
            float(self.cfg["aoa_fov_deg"]),
        )
        flags = self._peer_flags(peer)
        row = {
            "observer_id": obs.device_id,
            "peer_id": peer.device_id,
            "range_m": shared["range_m"],
            "sigma_range_m": shared["sigma_range_m"],
            "flags": flags | FLAG_RANGE_VALID,
        }

        bearing_ok = (
            geom["in_cone"]
            and shared["los"]
            and not shared.get("bearing_invalidated", False)
            and int(self.cfg.get("n_antennas", 3)) >= 2
        )
        elev_ok = bearing_ok and int(self.cfg.get("n_antennas", 3)) >= 3

        if bearing_ok:
            az = shared["az_meas"][obs.device_id]
            el = shared["el_meas"].get(obs.device_id, float("nan"))
            if not elev_ok:
                el = float("nan")
            sigma_a = shared["sigma_az"][obs.device_id]
            sigma_e = shared["sigma_el"].get(obs.device_id, float("nan"))
            if elev_ok and not math.isnan(el):
                x, y, z = bearing_xyz(shared["range_m"], az, el)
            elif not math.isnan(az):
                x, y, z = bearing_xyz(shared["range_m"], az, 0.0)
                z = float("nan")
            else:
                x = y = z = float("nan")
            row.update(
                {
                    "x": x,
                    "y": y,
                    "z": z if elev_ok else float("nan"),
                    "azimuth_rad": az,
                    "elevation_rad": el if elev_ok else float("nan"),
                    "sigma_az_rad": sigma_a,
                    "sigma_el_rad": sigma_e if elev_ok else float("nan"),
                    "flags": flags
                    | FLAG_RANGE_VALID
                    | FLAG_BEARING_VALID
                    | (FLAG_LOS if shared["los"] else 0)
                    | FLAG_IN_AOA_CONE,
                }
            )
        else:
            row.update(nan_edge_fields())
            row["flags"] = (
                flags
                | FLAG_RANGE_VALID
                | (FLAG_LOS if shared["los"] else 0)
                | (FLAG_IN_AOA_CONE if geom["in_cone"] else 0)
            )
        return row

    def _draw_exchange(self, dev_a: DeviceState, dev_b: DeviceState) -> Optional[dict]:
        pa, pb = dev_a.position, dev_b.position
        r_true = float(np.linalg.norm(pb - pa))
        los = los_check(pa, pb, self.cfg)

        if self.rng.random() < self.link_budget_dropout_prob(r_true):
            return None

        nlos_bias = 0.0
        sigma_r = float(self.cfg["sigma_range_los_m"])
        bearing_invalidated = False

        if not los:
            nlos_bias = abs(
                float(
                    self.rng.normal(
                        float(self.cfg["nlos_bias_mean_m"]),
                        float(self.cfg["nlos_bias_sigma_m"]),
                    )
                )
            )
            sigma_r *= float(self.cfg["nlos_sigma_mult"])
            if self.rng.random() < float(self.cfg["p_dropout_nlos"]):
                return None
            if self.cfg.get("nlos_invalidates_bearing", True):
                bearing_invalidated = True

        bias_ij = self.antenna_delay_bias.get(dev_a.device_id, 0.0) + self.antenna_delay_bias.get(
            dev_b.device_id, 0.0
        )
        range_noise = float(self.rng.normal(0.0, sigma_r))
        r_meas = r_true + bias_ij + nlos_bias + range_noise

        geom_a = compute_peer_geometry(
            pa, dev_a.R_bw, pb, self.boresight_unit(), float(self.cfg["aoa_fov_deg"])
        )
        geom_b = compute_peer_geometry(
            pb, dev_b.R_bw, pa, self.boresight_unit(), float(self.cfg["aoa_fov_deg"])
        )

        az_meas: Dict[int, float] = {}
        el_meas: Dict[int, float] = {}
        sigma_az: Dict[int, float] = {}
        sigma_el: Dict[int, float] = {}

        for obs, geom in ((dev_a, geom_a), (dev_b, geom_b)):
            if geom["in_cone"] and not bearing_invalidated:
                theta = geom["theta_rad"]
                sa = sigma_ang(theta, self.cfg)
                az = geom["azimuth_rad"] + float(self.rng.normal(0.0, sa))
                az, el_true = self._apply_phase_wrap(az, geom["elevation_rad"], theta)
                az_meas[obs.device_id] = az
                sigma_az[obs.device_id] = sa
                if int(self.cfg.get("n_antennas", 3)) >= 3:
                    se = sigma_ang(theta, self.cfg)
                    el = geom["elevation_rad"] + float(self.rng.normal(0.0, se))
                    if float(self.cfg.get("elevation_mirror_prob", 0.0)) > 0.0:
                        if self.rng.random() < float(self.cfg["elevation_mirror_prob"]):
                            el = -el
                    el_meas[obs.device_id] = el
                    sigma_el[obs.device_id] = se

        return {
            "range_m": r_meas,
            "sigma_range_m": sigma_r,
            "los": los,
            "bearing_invalidated": bearing_invalidated,
            "az_meas": az_meas,
            "el_meas": el_meas,
            "sigma_az": sigma_az,
            "sigma_el": sigma_el,
        }

    def tick(
        self, devices: Dict[int, DeviceState], dt: Optional[float] = None
    ) -> Tuple[Dict[int, List[dict]], List[dict]]:
        """Advance scheduler one tick. Returns per-observer edges and truth rows."""
        if dt is None:
            dt = 1.0 / float(self.cfg["scheduler_tick_hz"])
        self.sim_time += dt
        self.update_scheduled_pairs(devices)

        per_observer: Dict[int, List[dict]] = {did: [] for did in devices if devices[did].active}
        truth_rows: List[dict] = []

        max_per_tick = int(
            math.ceil(float(self.cfg["max_exchanges_per_s"]) / float(self.cfg["scheduler_tick_hz"]))
        )
        fired = 0

        for pk in sorted(self.scheduled_pairs):
            if fired >= max_per_tick:
                break
            if self.sim_time < self.next_due.get(pk, 0.0):
                continue
            a_id, b_id = pk
            if a_id not in devices or b_id not in devices:
                continue
            da, db = devices[a_id], devices[b_id]
            if not da.active or not db.active:
                continue

            shared = self._draw_exchange(da, db)
            period = 1.0 / max(self.effective_rate, EPS)
            self.next_due[pk] = self.sim_time + period
            fired += 1

            if shared is None:
                continue

            for obs, peer in ((da, db), (db, da)):
                row = self.execute_exchange(obs, peer, shared)
                if row is not None:
                    per_observer.setdefault(obs.device_id, []).append(row)

        return per_observer, truth_rows

    def compute_truth_edges(self, devices: Dict[int, DeviceState]) -> List[dict]:
        """Noise-free truth for all candidate pairs within range."""
        active = {did: d for did, d in devices.items() if d.active}
        ids = sorted(active.keys())
        max_r = float(self.cfg["max_range_m"])
        rows: List[dict] = []
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a, b = ids[i], ids[j]
                pa, pb = active[a].position, active[b].position
                r = float(np.linalg.norm(pb - pa))
                if r >= max_r:
                    continue
                los = los_check(pa, pb, self.cfg)
                for obs_id, peer_id in ((a, b), (b, a)):
                    obs, peer = active[obs_id], active[peer_id]
                    geom = compute_peer_geometry(
                        obs.position,
                        obs.R_bw,
                        peer.position,
                        self.boresight_unit(),
                        float(self.cfg["aoa_fov_deg"]),
                    )
                    flags = self._peer_flags(peer) | FLAG_RANGE_VALID
                    if los:
                        flags |= FLAG_LOS
                    if geom["in_cone"]:
                        flags |= FLAG_IN_AOA_CONE
                    row = {
                        "observer_id": obs_id,
                        "peer_id": peer_id,
                        "range_m": geom["range_m"],
                        "sigma_range_m": 0.0,
                        "azimuth_rad": geom["azimuth_rad"],
                        "elevation_rad": geom["elevation_rad"],
                        "sigma_az_rad": 0.0,
                        "sigma_el_rad": 0.0,
                        "flags": flags,
                    }
                    if geom["in_cone"]:
                        x, y, z = bearing_xyz(
                            geom["range_m"],
                            geom["azimuth_rad"],
                            geom["elevation_rad"],
                        )
                        row["x"] = x
                        row["y"] = y
                        row["z"] = z
                        row["flags"] |= FLAG_BEARING_VALID
                    else:
                        row.update(nan_edge_fields())
                    rows.append(row)
        return rows


def default_selftest_config() -> dict:
    return {
        "channel_centre_hz": 6.5e9,
        "antenna_spacing_m": 0.023,
        "n_antennas": 3,
        "boresight_axis": [1.0, 0.0, 0.0],
        "aoa_fov_deg": 100.0,
        "sigma_range_los_m": 0.10,
        "antenna_delay_bias_sigma_m": 0.05,
        "max_range_m": 30.0,
        "p_dropout_at_max_range": 0.5,
        "angle_error_model": "inv_cos",
        "sigma_boresight_deg": 8.0,
        "nlos_bias_mean_m": 0.35,
        "nlos_bias_sigma_m": 0.25,
        "nlos_sigma_mult": 3.0,
        "p_dropout_nlos": 0.35,
        "nlos_invalidates_bearing": True,
        "los_model": "boxes",
        "occluder_boxes": [],
        "ranging_rate_hz": 10.0,
        "max_exchanges_per_s": 400.0,
        "max_neighbors_per_drone": 6,
        "scheduler_tick_hz": 50.0,
        "static_peers": [],
        "model_phase_wrap": False,
        "elevation_mirror_prob": 0.0,
        "seed": 42,
        "_num_drones": 2,
    }


def run_selftest() -> int:
    from uwb_edges import pack_edges, unpack_edges

    ok = True
    cfg = default_selftest_config()
    boresight = np.array([1.0, 0.0, 0.0])
    R0 = np.eye(3)

    def check(name: str, cond: bool, detail: str = ""):
        nonlocal ok
        if not cond:
            ok = False
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))
        else:
            print(f"[selftest] PASS {name}")

    # 1 geometry dead ahead
    g = compute_peer_geometry(np.zeros(3), R0, np.array([2.0, 0.0, 0.0]), boresight, 100.0)
    check("1 dead ahead", abs(g["range_m"] - 2.0) < 1e-9 and abs(g["azimuth_rad"]) < 1e-9)
    check("1 el=0", abs(g["elevation_rad"]) < 1e-9 and g["in_cone"])

    # 2 azimuth sign
    g_left = compute_peer_geometry(np.zeros(3), R0, np.array([0.0, 2.0, 0.0]), boresight, 100.0)
    g_right = compute_peer_geometry(np.zeros(3), R0, np.array([0.0, -2.0, 0.0]), boresight, 100.0)
    check("2 az left +pi/2", abs(g_left["azimuth_rad"] - math.pi / 2) < 1e-6)
    check("2 az right -pi/2", abs(g_right["azimuth_rad"] + math.pi / 2) < 1e-6)

    # 3 elevation sign
    g_up = compute_peer_geometry(np.zeros(3), R0, np.array([0.0, 0.0, 2.0]), boresight, 100.0)
    check("3 el up +pi/2", abs(g_up["elevation_rad"] - math.pi / 2) < 1e-6)

    # 4 observer yaw
    R_yaw = yaw_to_rot_matrix(math.pi / 2)
    g_yaw = compute_peer_geometry(np.zeros(3), R_yaw, np.array([2.0, 0.0, 0.0]), boresight, 100.0)
    check("4 yawed az", abs(g_yaw["azimuth_rad"] + math.pi / 2) < 1e-5)
    check("4 yawed r", abs(g_yaw["range_m"] - 2.0) < 1e-6)

    # 5 cone fallback via model exchange
    cfg5 = dict(cfg)
    cfg5["sigma_range_los_m"] = 0.0
    cfg5["antenna_delay_bias_sigma_m"] = 0.0
    cfg5["p_dropout_at_max_range"] = 0.0
    cfg5["p_dropout_nlos"] = 0.0
    m5 = UwbModel.from_config(cfg5, seed=1)
    dev0 = DeviceState(0, np.zeros(3), R0, False, "drone")
    dev1 = DeviceState(1, np.array([-2.0, 0.0, 0.0]), R0, False, "drone")
    shared = None
    for _ in range(50):
        shared = m5._draw_exchange(dev0, dev1)
        if shared:
            break
    check("5 shared drawn", shared is not None)
    if shared:
        row = m5.execute_exchange(dev0, dev1, shared)
        check("5 range valid", row and (row["flags"] & FLAG_RANGE_VALID))
        check("5 no bearing", row and not (row["flags"] & FLAG_BEARING_VALID))
        check("5 az nan", row and math.isnan(row["azimuth_rad"]))
        check("5 range finite", row and math.isfinite(row["range_m"]))

    # 6 bidirectional range identity
    cfg6 = dict(cfg)
    cfg6["p_dropout_at_max_range"] = 0.0
    cfg6["p_dropout_nlos"] = 0.0
    m6 = UwbModel.from_config(cfg6, seed=6)
    d0 = DeviceState(0, np.zeros(3), R0, False, "drone")
    d1 = DeviceState(1, np.array([2.0, 0.0, 0.0]), R0, False, "drone")
    devices = {0: d0, 1: d1}
    m6.update_scheduled_pairs(devices)
    m6.next_due[pair_key(0, 1)] = 0.0
    m6.sim_time = 0.0
    edges, _ = m6.tick(devices)
    r01 = r10 = None
    az0 = az1 = None
    for row in edges.get(0, []):
        if row["peer_id"] == 1:
            r01 = row["range_m"]
            az0 = row["azimuth_rad"]
    for row in edges.get(1, []):
        if row["peer_id"] == 0:
            r10 = row["range_m"]
            az1 = row["azimuth_rad"]
    check("6 range identical", r01 is not None and r01 == r10)
    if az0 is not None and az1 is not None and not (math.isnan(az0) or math.isnan(az1)):
        check("6 az differ", az0 != az1)

    # 7 antenna bias constant
    cfg7 = dict(cfg)
    cfg7["sigma_range_los_m"] = 0.0
    cfg7["p_dropout_at_max_range"] = 0.0
    cfg7["p_dropout_nlos"] = 0.0
    m7 = UwbModel.from_config(cfg7, seed=7)
    m7.antenna_delay_bias = {0: 0.03, 1: 0.04}
    d0 = DeviceState(0, np.zeros(3), R0, False, "drone")
    d1 = DeviceState(1, np.array([2.0, 0.0, 0.0]), R0, False, "drone")
    ranges = []
    for _ in range(100):
        s = m7._draw_exchange(d0, d1)
        if s:
            ranges.append(s["range_m"])
    check("7 std zero", len(ranges) == 100 and np.std(ranges) == 0.0)
    check("7 mean bias", abs(np.mean(ranges) - (2.0 + 0.07)) < 1e-9)
    cfg7b = dict(cfg7)
    cfg7b["antenna_delay_bias_sigma_m"] = 0.0
    m7b = UwbModel.from_config(cfg7b, seed=7)
    m7b.antenna_delay_bias = {0: 0.0, 1: 0.0}
    ranges2 = [m7b._draw_exchange(d0, d1)["range_m"] for _ in range(100)]
    check("7 zero bias mean", abs(np.mean(ranges2) - 2.0) < 0.05)

    # 8 occlusion boxes
    wall = {"name": "wall", "x_min": 0.9, "x_max": 1.1, "y_min": -5.0, "y_max": 5.0, "z_min": 0.0, "z_max": 3.0}
    check("8 occluded", not los_check_boxes(np.array([0, 0, 1.0]), np.array([2, 0, 1.0]), [wall]))
    check("8 clear side", los_check_boxes(np.array([0, 0, 1.0]), np.array([0, 2, 1.0]), [wall]))
    check("8 over top", los_check_boxes(np.array([0, 0, 5.0]), np.array([2, 0, 5.0]), [wall]))

    # 9 NLOS effects
    cfg9 = dict(cfg)
    cfg9["occluder_boxes"] = [wall]
    cfg9["p_dropout_nlos"] = 0.0
    cfg9["p_dropout_at_max_range"] = 0.0
    cfg9["antenna_delay_bias_sigma_m"] = 0.0
    m9 = UwbModel.from_config(cfg9, seed=9)
    d0 = DeviceState(0, np.zeros(3), R0, False, "drone")
    d1 = DeviceState(1, np.array([2.0, 0.0, 1.0]), R0, False, "drone")
    los_vals, ranges, sigmas, bearings = [], [], [], []
    for _ in range(200):
        s = m9._draw_exchange(d0, d1)
        if not s:
            continue
        los_vals.append(s["los"])
        ranges.append(s["range_m"])
        sigmas.append(s["sigma_range_m"])
        row = m9.execute_exchange(d0, d1, s)
        bearings.append(row["flags"] & FLAG_BEARING_VALID)
    check("9 nlos flagged", los_vals and not all(los_vals))
    check("9 positive bias", np.mean(ranges) > 2.0)
    check("9 sigma mult", sigmas and abs(sigmas[0] - cfg9["sigma_range_los_m"] * cfg9["nlos_sigma_mult"]) < 1e-9)
    check("9 bearing cleared", not any(bearings))

    # 10 NLOS dropout
    cfg10 = dict(cfg9)
    cfg10["p_dropout_nlos"] = 1.0
    m10 = UwbModel.from_config(cfg10, seed=10)
    drops = [m10._draw_exchange(d0, d1) for _ in range(50)]
    check("10 all dropped", all(d is None for d in drops))

    # 11 off-boresight sigma (use wide FOV so 60° is not cos_floor-capped)
    cfg11 = dict(cfg)
    cfg11["aoa_fov_deg"] = 140.0
    s0 = sigma_ang(0.0, cfg11)
    s60 = sigma_ang(math.radians(60.0), cfg11)
    s_edge = sigma_ang(math.radians(100.0), cfg)
    check("11 boresight", abs(s0 - math.radians(8.0)) < 1e-9)
    check("11 60deg ~2x", abs(s60 / s0 - 2.0) < 0.15)
    check("11 capped", s_edge < 1e6 and s_edge <= sigma_ang(math.radians(45.0), cfg) * 1.5)

    # 12 airtime
    cfg12 = dict(cfg)
    cfg12["max_neighbors_per_drone"] = 9
    cfg12["ranging_rate_hz"] = 100.0
    cfg12["max_exchanges_per_s"] = 90.0
    cfg12["scheduler_tick_hz"] = 50.0
    cfg12["p_dropout_at_max_range"] = 0.0
    cfg12["p_dropout_nlos"] = 0.0
    cfg12["_num_drones"] = 10
    m12 = UwbModel.from_config(cfg12, seed=12)
    line = np.linspace(0, 9, 10)
    devices12 = {
        i: DeviceState(i, np.array([float(i), 0.0, 0.0]), R0, False, "drone")
        for i in range(10)
    }
    m12.update_scheduled_pairs(devices12)
    check("12 effective rate", abs(m12.effective_rate - 2.0) < 1e-9)
    dt = 1.0 / 50.0
    total_rows = 0
    for _ in range(int(10.0 / dt)):
        edges, _ = m12.tick(devices12, dt)
        for rows in edges.values():
            total_rows += len(rows)
    total_exchanges = total_rows // 2
    expected = 45 * 2.0 * 10.0
    check(
        "12 exchange count",
        abs(total_exchanges - expected) / expected <= 0.05,
        f"got {total_exchanges} expected {expected}",
    )

    # 13 neighbour cap
    cfg13 = dict(cfg)
    cfg13["max_neighbors_per_drone"] = 2
    cfg13["_num_drones"] = 10
    m13 = UwbModel.from_config(cfg13, seed=13)
    devices13 = {
        i: DeviceState(i, np.array([float(i), 0.0, 0.0]), R0, False, "drone")
        for i in range(10)
    }
    m13.update_scheduled_pairs(devices13)
    for a, b in m13.scheduled_pairs:
        if abs(a - b) > 2:
            ok = False
            print(f"[selftest] FAIL 13 cap: pair ({a},{b}) distance {abs(a-b)} > 2")
    check("13 neighbour cap", all(abs(a - b) <= 2 for a, b in m13.scheduled_pairs))

    # 14 link budget dropout
    cfg14 = dict(cfg)
    cfg14["p_dropout_at_max_range"] = 0.5
    cfg14["p_dropout_nlos"] = 1.0
    cfg14["nlos_invalidates_bearing"] = False
    m14 = UwbModel.from_config(cfg14, seed=14)
    max_r = float(cfg14["max_range_m"])
    d0 = DeviceState(0, np.zeros(3), R0, False, "drone")
    d_max = DeviceState(1, np.array([max_r, 0.0, 0.0]), R0, False, "drone")
    d_half = DeviceState(2, np.array([max_r / 2, 0.0, 0.0]), R0, False, "drone")
    drops_max = sum(
        1
        for _ in range(2000)
        if m14._draw_exchange(d0, d_max) is None
    )
    drops_half = sum(
        1
        for _ in range(2000)
        if m14._draw_exchange(d0, d_half) is None
    )
    check("14 max range drop", abs(drops_max / 2000 - 0.5) <= 0.05, f"frac={drops_max/2000}")
    check("14 half range drop", abs(drops_half / 2000 - 0.125) <= 0.05, f"frac={drops_half/2000}")

    # 15 pack/unpack
    edge = {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0,
        "observer_id": 0,
        "peer_id": 1,
        "range_m": 2.5,
        "azimuth_rad": 0.1,
        "elevation_rad": 0.2,
        "sigma_range_m": 0.1,
        "sigma_az_rad": 0.05,
        "sigma_el_rad": 0.05,
        "flags": FLAG_RANGE_VALID | FLAG_BEARING_VALID,
    }
    try:
        from sensor_msgs.msg import PointCloud2  # noqa: F401
        has_pc2 = True
    except ImportError:
        has_pc2 = False
    if has_pc2:
        msg = pack_edges([edge], (0, 0), "cf_0/base_link")
        back = unpack_edges(msg)[0]
        for name in EDGE_DTYPE.names:
            av, bv = edge[name], back[name]
            if isinstance(av, float) and math.isnan(av):
                check(f"15 {name} nan", math.isnan(bv))
            else:
                check(f"15 {name}", av == bv)
        check("15 itemsize", EDGE_DTYPE.itemsize == 48)
    else:
        print("[selftest] SKIP 15 pack/unpack (no sensor_msgs)")

    # 16 lambda/2 warning flag
    lam = C_LIGHT / 6.5e9
    check("16 ambiguous spacing", 0.05 > lam / 2)
    check("16 ok spacing", 0.023 <= lam / 2 + 1e-9)

    # 17 determinism
    def run_det(seed):
        c = dict(cfg)
        c["p_dropout_at_max_range"] = 0.0
        c["p_dropout_nlos"] = 0.0
        m = UwbModel.from_config(c, seed=seed)
        d0 = DeviceState(0, np.zeros(3), R0, False, "drone")
        d1 = DeviceState(1, np.array([2.0, 0.0, 0.0]), R0, False, "drone")
        devs = {0: d0, 1: d1}
        m.next_due[pair_key(0, 1)] = 0.0
        out = []
        for _ in range(5):
            e, _ = m.tick(devs)
            for rows in e.values():
                for r in rows:
                    out.append(r["range_m"])
        return out

    a = run_det(99)
    b = run_det(99)
    c = run_det(100)
    check("17 same seed", a == b)
    check("17 diff seed", a != c)

    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    print("Use uwb_node.py for the ROS wrapper, or --selftest here.", file=sys.stderr)
    sys.exit(2)


if __name__ == "__main__":
    main()
