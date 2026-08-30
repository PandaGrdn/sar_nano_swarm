#!/usr/bin/env python3
"""swarm_loc_node.py — per-drone rclpy wrapper for the swarm-loc EKF (P2-5/P2-6).

One process per drone. Odometry enters only via /cf_<id>/rio/delta.
Must never subscribe to /cf_*/odom, /uwb/edges_truth, or /cf_*/flow/debug_truth.

Usage (setup_env.sh sourced):
    python3 -u perception/swarm_loc/swarm_loc_node.py --cf-id 0 --num-drones 3
    python3 -u perception/swarm_loc/swarm_loc_node.py --cf-id 0 --log-measurements logs/
    python3 perception/swarm_loc/swarm_loc_node.py --selftest
"""
from __future__ import annotations

import argparse
import heapq
import math
import sys
import time
from collections import defaultdict, deque
from pathlib import Path
from typing import Deque, Dict, List

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[2]
for _p in ("perception/swarm_loc", "perception/uwb_sim"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

from ekf import YawModeGuard, propagate, update  # noqa: E402
from meas_log import (  # noqa: E402
    KIND_ENTRANCE_RELPOS,
    KIND_RELPOS,
    MeasurementLogger,
    kind_from_edge,
    resolve_log_path,
)
from measurements import (  # noqa: E402
    cartesian_cov_from_spherical,
    entrance_observed_from_edge,
    from_edge,
    mutual_yaw,
    range_rate,
    range_rate_regression,
    reciprocal_relpos,
)
from rio_stub import load_config, resolve_config_path  # noqa: E402
from state import STATUS_DIVERGED, SwarmState  # noqa: E402
from swarm_msgs import (  # noqa: E402
    DelayedDropQueue,
    neighbor_P_from_state_row,
    pack_bearing,
    pack_state,
    rio_delta_from_row,
    state_row_from_filter,
    unpack_bearing,
    unpack_rio,
    unpack_state,
)
from uwb_model import bearing_xyz  # noqa: E402
from uwb_edges import (  # noqa: E402
    FLAG_BEARING_VALID,
    FLAG_PEER_IS_SURVEYED,
    FLAG_RANGE_VALID,
    unpack_edges,
)


def entrance_device_id(cfg: dict) -> int:
    """Surveyed observer id from the ESTIMATOR'S config (never uwb_pdoa.yaml)."""
    return int(cfg.get("entrance", {}).get("device_id", 1000))


def classify_uwb_row(observer_id: int, peer_id: int, cf_id: int, entrance_id: int) -> str:
    """Route one UWB edge row (R1 fix — observer check comes FIRST).

    'own'               — this drone observed the edge; may cache its own
                          bearing, rebroadcast it, and update against the peer.
    'entrance_observed' — the surveyed entrance observed THIS drone (§4.3e).
    'drop'              — everything else. In particular an entrance-observed
                          row about another drone must never enter the own-
                          bearing cache or the rebroadcast queue (the pre-fix
                          bug behind the hot mutual_yaw / reciprocal_relpos NIS).
    """
    if int(observer_id) == int(cf_id):
        return "own"
    if int(observer_id) == int(entrance_id) and int(peer_id) == int(cf_id):
        return "entrance_observed"
    return "drop"


def estimator_subscription_topics(cf_id: int, num_drones: int, cfg: dict) -> List[str]:
    """Topics this node is allowed to subscribe to. Used by --selftest and the gate."""
    n = int(num_drones)
    k = min(int(cfg["comms"]["max_neighbors"]), max(n, 1))
    topics = [
        f"/cf_{cf_id}/rio/delta",
        f"/cf_{cf_id}/uwb/edges",
        f"/uwb/peer_{entrance_device_id(cfg)}/edges",
    ]
    added = 0
    for j in range(n):
        if j == cf_id:
            continue
        if added >= k:
            break
        topics.append(f"/cf_{j}/swarm_loc/broadcast")
        topics.append(f"/cf_{j}/swarm_loc/bearing_rebroadcast")
        added += 1
    return topics


def topics_contain_truth(topics: List[str]) -> List[str]:
    bad = []
    for t in topics:
        if t.endswith("/uwb/edges_truth") or t == "/uwb/edges_truth":
            bad.append(t)
        if t.endswith("/odom") and "/swarm_loc/" not in t:
            bad.append(t)
        if t.endswith("/flow/debug_truth"):
            bad.append(t)
    return bad


def _stamp_msg(msg) -> float:
    s = msg.header.stamp
    return float(s.sec) + float(s.nanosec) * 1e-9


class SwarmLocNode:
    """Filter loop. Instantiated only after rclpy import (see _run_node)."""

    def __init__(self, node, cf_id: int, num_drones: int, cfg: dict, log_path=None):
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2

        self._PoseStamped = PoseStamped

        self.node = node
        self.cf_id = int(cf_id)
        self.num_drones = int(num_drones)
        self.cfg = cfg
        self.abl = cfg.get("ablation", {})
        self.st = SwarmState.from_launch(cfg, self.cf_id)
        self._seq = 0
        self._n_bearing_period = 0
        self._buf: List[Tuple[float, int, str, object]] = []
        self._buf_seq = 0
        self._neighbors: Dict[int, dict] = {}
        self._range_win: Dict[int, Deque] = defaultdict(lambda: deque(maxlen=32))
        self._pending_rebroadcast: List[dict] = []
        self._own_bearing: Dict[int, dict] = {}
        self._peer_bearing_to_us: Dict[int, dict] = {}
        self._entrance_id = entrance_device_id(cfg)
        self._entrance_bearing_to_us: dict | None = None
        self._yaw_guard = YawModeGuard(cfg)
        self._n_mutual_yaw = 0
        self._n_entrance_mutual_yaw = 0
        self._n_entrance_obs = 0
        self._n_entrance_az_only = 0
        self._n_reciprocal = 0
        self._n_az_only = 0
        self._n_update = 0
        self._n_accept = 0
        self._n_nis = 0
        self._n_prop = 0
        self._cpu_prop = 0.0
        self._cpu_upd = 0.0
        self._n_tx_bytes = 0
        self._sum_nis = 0.0
        self._n_nis_samples = 0
        self._nis_by = defaultdict(lambda: {"n": 0, "sum": 0.0, "rej": 0})
        self._t0_wall = time.time()
        self._last_metric_wall = self._t0_wall
        self._log = None
        if log_path:
            self._log = MeasurementLogger(self.cf_id, resolve_log_path(str(log_path), self.cf_id))
        rng = np.random.default_rng(int(cfg.get("seed", 0)) + self.cf_id)
        comms = cfg["comms"]
        self._tx = DelayedDropQueue(
            float(comms["latency_ms"]) * 1e-3,
            float(comms["packet_loss"]),
            rng,
        )
        qos = qos_profile_sensor_data
        self._pub_est = node.create_publisher(
            PointCloud2, f"/cf_{self.cf_id}/swarm_loc/estimate", qos
        )
        self._pub_bc = node.create_publisher(
            PointCloud2, f"/cf_{self.cf_id}/swarm_loc/broadcast", qos
        )
        self._pub_brg = node.create_publisher(
            PointCloud2, f"/cf_{self.cf_id}/swarm_loc/bearing_rebroadcast", qos
        )
        self._pub_pose = node.create_publisher(
            PoseStamped, f"/cf_{self.cf_id}/swarm_loc/pose", qos
        )
        for topic in estimator_subscription_topics(self.cf_id, self.num_drones, cfg):
            if topic.endswith("/rio/delta"):
                node.create_subscription(PointCloud2, topic, self._on_rio, qos)
            elif topic.endswith("/uwb/edges") or "/uwb/peer_" in topic:
                node.create_subscription(PointCloud2, topic, self._on_uwb, qos)
            elif topic.endswith("/bearing_rebroadcast"):
                node.create_subscription(PointCloud2, topic, self._on_rebroadcast, qos)
            elif topic.endswith("/broadcast"):
                node.create_subscription(PointCloud2, topic, self._on_neighbor, qos)
        rate = float(cfg["estimator"]["rate_hz"])
        bc_hz = float(comms["broadcast_rate_hz"])
        node.create_timer(1.0 / max(rate, 1.0), self._on_tick)
        node.create_timer(1.0 / max(bc_hz, 1.0), self._on_broadcast_tick)
        node.get_logger().info(
            f"swarm_loc_{self.cf_id} up — estimate @ {rate:.0f} Hz, "
            f"broadcast @ {bc_hz:.0f} Hz"
        )

    def _enqueue(self, stamp: float, kind: str, payload) -> None:
        heapq.heappush(self._buf, (float(stamp), self._buf_seq, kind, payload))
        self._buf_seq += 1

    def _apply_update(self, meas, P_j=None, fusion=None):
        """Timed EKF update; counts NIS rejects (D16) for §6.1."""
        if meas is None:
            return {}
        t0 = time.perf_counter()
        self.st, info = update(self.st, meas, self.cfg, P_j=P_j, fusion=fusion)
        self._cpu_upd += time.perf_counter() - t0
        self._n_update += 1
        if info.get("accepted"):
            self._n_accept += 1
        if info.get("reason") == "nis_gate":
            self._n_nis += 1
        nis = info.get("nis", float("nan"))
        name = getattr(meas, "name", "unknown")
        rec = self._nis_by[name]
        rec["n"] += 1
        if isinstance(nis, (int, float)) and math.isfinite(float(nis)):
            self._sum_nis += float(nis)
            self._n_nis_samples += 1
            rec["sum"] += float(nis)
        if info.get("reason") == "nis_gate":
            rec["rej"] += 1
        if self._log is not None:
            self._log.add_nis(
                float(self.st.stamp),
                name,
                float(nis) if isinstance(nis, (int, float)) else float("nan"),
                bool(info.get("accepted")),
            )
        # R3 π-flip guard: covariance-only recovery when bearing-family
        # measurements are persistently rejected under a confident yaw.
        self.st, yaw_trig = self._yaw_guard.observe(
            name, bool(info.get("accepted")), float(self.st.stamp), self.st
        )
        if yaw_trig:
            self.node.get_logger().warning(
                f"YAW_MODE_SUSPECT cf={self.cf_id}: bearing accept rate low with "
                f"confident yaw — inflated yaw sigma (trigger #{self._yaw_guard.n_triggered})"
            )
        return info

    def _on_rio(self, msg) -> None:
        if self.abl.get("disable_rio"):
            return
        rows = unpack_rio(msg)
        if rows.size == 0:
            return
        delta = rio_delta_from_row(rows[0])
        if self.st.status != STATUS_DIVERGED:
            t0 = time.perf_counter()
            self.st = propagate(self.st, delta, self.cfg)
            self._cpu_prop += time.perf_counter() - t0
            self._n_prop += 1
        if self._log is not None:
            self._log.add_rio(
                delta.stamp,
                delta.dt,
                delta.delta_p_body,
                delta.delta_psi,
                delta.roll,
                delta.pitch,
                delta.valid,
            )
        self._drain()

    def _on_uwb(self, msg) -> None:
        if self.abl.get("disable_uwb"):
            return
        stamp = _stamp_msg(msg)
        rows = unpack_edges(msg)
        self._enqueue(stamp, "uwb", rows)

    def _on_neighbor(self, msg) -> None:
        rows = unpack_state(msg)
        if rows.size == 0:
            return
        self._enqueue(float(rows[0]["stamp"]), "nb", rows[0])

    def _on_rebroadcast(self, msg) -> None:
        rows = unpack_bearing(msg)
        if rows.size == 0:
            return
        self._enqueue(float(rows[0]["stamp"]), "rb", rows)

    def _drain(self) -> None:
        max_age = float(self.cfg["measurements"]["max_measurement_age_s"])
        now_sim = float(self.st.stamp)
        if now_sim <= 0.0:
            return
        while self._buf and self._buf[0][0] <= now_sim + 1e-9:
            stamp, _, kind, payload = heapq.heappop(self._buf)
            if now_sim - stamp > max_age:
                continue
            if kind == "nb":
                self._store_neighbor(payload)
            elif kind == "uwb":
                self._apply_uwb_rows(payload)
            elif kind == "rb":
                self._apply_rebroadcast(payload)

    def _store_neighbor(self, row) -> None:
        j = int(row["drone_id"])
        if int(row["status"]) == STATUS_DIVERGED:
            self._neighbors.pop(j, None)
            return
        self._neighbors[j] = {
            "p": np.array([row["p_x"], row["p_y"], row["p_z"]], dtype=np.float64),
            "v": np.array([row["v_x"], row["v_y"], row["v_z"]], dtype=np.float64),
            "psi": float(row["psi"]),
            "P": neighbor_P_from_state_row(row),
            "roll": float(row["roll"]),
            "pitch": float(row["pitch"]),
            "stamp": float(row["stamp"]),
            "status": int(row["status"]),
        }

    def _apply_uwb_rows(self, rows) -> None:
        if self.st.status == STATUS_DIVERGED:
            return
        use_bearing = bool(self.cfg["measurements"].get("use_bearing", True))
        if self.abl.get("disable_bearing"):
            use_bearing = False
        use_rr = bool(self.cfg["measurements"].get("use_range_rate", True))
        win_n = int(self.cfg["measurements"]["range_rate_window"])
        max_age = float(self.cfg["measurements"]["max_measurement_age_s"])
        for row in rows:
            flags = int(row["flags"])
            peer = int(row["peer_id"])
            # R1 fix: route on the OBSERVER first. Only rows this drone itself
            # observed may feed the own-bearing cache, the rebroadcast queue,
            # or the bearing/az-only counters. Entrance-observed rows about us
            # go to the §4.3e anchor path; everything else is dropped.
            route = classify_uwb_row(int(row["observer_id"]), peer, self.cf_id, self._entrance_id)
            if route == "drop":
                continue
            if route == "entrance_observed":
                self._apply_entrance_observed(row)
                continue
            if (flags & FLAG_BEARING_VALID) and not math.isfinite(float(row["z"])):
                self._n_az_only += 1
            if flags & FLAG_BEARING_VALID:
                self._n_bearing_period += 1
                if use_bearing and math.isfinite(float(row["z"])):
                    self._own_bearing[peer] = {
                        "stamp": float(self.st.stamp),
                        "range_m": float(row["range_m"]),
                        "azimuth_rad": float(row["azimuth_rad"]),
                        "elevation_rad": float(row["elevation_rad"]),
                        "sigma_range_m": float(row["sigma_range_m"]),
                        "sigma_az_rad": float(row["sigma_az_rad"]),
                        "sigma_el_rad": float(row["sigma_el_rad"]),
                    }
                self._pending_rebroadcast.append(
                    {
                        "stamp": float(self.st.stamp),
                        "observer_id": self.cf_id,
                        "peer_id": peer,
                        "range_m": float(row["range_m"]),
                        "azimuth_rad": float(row["azimuth_rad"]),
                        "elevation_rad": float(row["elevation_rad"]),
                        "sigma_range_m": float(row["sigma_range_m"]),
                        "sigma_az_rad": float(row["sigma_az_rad"]),
                        "sigma_el_rad": float(row["sigma_el_rad"]),
                        "psi_observer": float(self.st.psi),
                        "roll_observer": float(self.st.roll),
                        "pitch_observer": float(self.st.pitch),
                    }
                )
            surveyed = bool(flags & FLAG_PEER_IS_SURVEYED)
            if surveyed and self.abl.get("disable_entrance"):
                continue
            edge = row
            if not use_bearing:
                # force range-only path by clearing bearing
                edge = {n: row[n] for n in row.dtype.names}
                edge["flags"] = flags & ~FLAG_BEARING_VALID
                edge["z"] = float("nan")
            if self._log is not None:
                self._log_uwb_edge(edge, peer)
            if surveyed:
                self._apply_update(from_edge(self.st, None, edge, self.cfg))
                continue
            nb = self._neighbors.get(peer)
            if nb is None:
                continue
            if abs(self.st.stamp - nb["stamp"]) > max_age:
                continue
            self._apply_update(from_edge(self.st, nb["p"], edge, self.cfg), P_j=nb["P"], fusion="ci")
            if use_bearing:
                self._try_mutual_yaw(peer)
            if use_rr and (flags & FLAG_RANGE_VALID):
                w = self._range_win[peer]
                w.append((float(self.st.stamp), float(row["range_m"]), float(row["sigma_range_m"])))
                rr = range_rate_regression(
                    [t for t, _, _ in w],
                    [r for _, r, _ in w],
                    [s for _, _, s in w],
                    window=win_n,
                    max_age_s=max_age,
                )
                if rr is not None:
                    dprime, sig = rr
                    self._apply_update(
                        range_rate(self.st.p, self.st.v, nb["p"], nb["v"], dprime, sig),
                        P_j=nb["P"],
                        fusion="ci",
                    )

    def _apply_entrance_observed(self, row) -> None:
        """§4.3e — the surveyed entrance observed THIS drone.

        Full-bearing rows become a direct entrance_obs_relpos update (known
        landmark, no CI — the entrance broadcasts no state) plus a mutual-yaw
        attempt against the entrance's surveyed yaw. Azimuth-only rows are
        counted and skipped. Range-only rows are skipped: the drone's own
        entrance_range already carries that physical exchange (P2_DEVIATIONS.md).
        A one-sided entrance bearing never invents a yaw measurement.
        """
        if self.abl.get("disable_entrance"):
            return
        use_bearing = bool(self.cfg["measurements"].get("use_bearing", True))
        if self.abl.get("disable_bearing"):
            use_bearing = False
        flags = int(row["flags"])
        if not (flags & FLAG_BEARING_VALID):
            return  # range-only: own entrance_range covers it
        if not math.isfinite(float(row["z"])):
            self._n_entrance_az_only += 1
            return
        if not use_bearing:
            return
        self._entrance_bearing_to_us = {
            "stamp": float(self.st.stamp),
            "range_m": float(row["range_m"]),
            "azimuth_rad": float(row["azimuth_rad"]),
            "elevation_rad": float(row["elevation_rad"]),
            "sigma_range_m": float(row["sigma_range_m"]),
            "sigma_az_rad": float(row["sigma_az_rad"]),
            "sigma_el_rad": float(row["sigma_el_rad"]),
        }
        info = self._apply_update(entrance_observed_from_edge(self.st, row, self.cfg))
        if info.get("accepted"):
            self._n_entrance_obs += 1
        self._try_entrance_mutual_yaw()

    def _try_entrance_mutual_yaw(self) -> None:
        """D11 against the fixed-yaw entrance: pins ABSOLUTE yaw (§4.3e).

        Needs our own bearing to the entrance (rare, rear cone) AND an
        entrance-observed bearing to us within mutual_yaw_max_dt_s. The
        entrance side has zero attitude uncertainty, so this is a direct EKF
        update (no P_j / CI).
        """
        if not bool(self.cfg["measurements"].get("use_mutual_yaw", True)):
            return
        if self.abl.get("disable_bearing") or self.abl.get("disable_entrance"):
            return
        own = self._own_bearing.get(self._entrance_id)
        rec = self._entrance_bearing_to_us
        if own is None or rec is None:
            return
        pair_dt = float(self.cfg["measurements"]["mutual_yaw_max_dt_s"])
        if abs(float(own["stamp"]) - float(rec["stamp"])) > pair_dt:
            return
        z_ij = bearing_xyz(own["range_m"], own["azimuth_rad"], own["elevation_rad"])
        z_ji = bearing_xyz(rec["range_m"], rec["azimuth_rad"], rec["elevation_rad"])
        Rij = cartesian_cov_from_spherical(
            own["range_m"],
            own["azimuth_rad"],
            own["elevation_rad"],
            own["sigma_range_m"],
            own["sigma_az_rad"],
            own["sigma_el_rad"],
        )
        Rji = cartesian_cov_from_spherical(
            rec["range_m"],
            rec["azimuth_rad"],
            rec["elevation_rad"],
            rec["sigma_range_m"],
            rec["sigma_az_rad"],
            rec["sigma_el_rad"],
        )
        psi_ent = math.radians(float(self.cfg["entrance"].get("yaw_deg", 0.0)))
        meas = mutual_yaw(
            self.st.psi,
            self.st.pitch,
            self.st.roll,
            z_ij,
            psi_ent,
            0.0,
            0.0,
            z_ji,
            Rij,
            Rji,
        )
        if meas is None:
            return
        # Partner yaw is a surveyed constant → its Jacobian block carries no
        # uncertainty; direct update, distinct name for NIS-by-type.
        meas.H_j = np.zeros_like(meas.H_j)
        meas.name = "entrance_mutual_yaw"
        info = self._apply_update(meas)
        if info.get("accepted"):
            self._n_entrance_mutual_yaw += 1
            self._own_bearing.pop(self._entrance_id, None)
            self._entrance_bearing_to_us = None

    def _apply_rebroadcast(self, rows) -> None:
        if self.st.status == STATUS_DIVERGED:
            return
        use_rec = bool(self.cfg["measurements"].get("use_reciprocal_bearing", True))
        use_my = bool(self.cfg["measurements"].get("use_mutual_yaw", True))
        if self.abl.get("disable_bearing"):
            use_rec = False
            use_my = False
        max_age = float(self.cfg["measurements"]["max_measurement_age_s"])
        pair_dt = float(self.cfg["measurements"]["mutual_yaw_max_dt_s"])
        for row in rows:
            obs = int(row["observer_id"])
            if obs == self.cf_id or int(row["peer_id"]) != self.cf_id:
                continue
            rec = {
                "stamp": float(row["stamp"]),
                "range_m": float(row["range_m"]),
                "azimuth_rad": float(row["azimuth_rad"]),
                "elevation_rad": float(row["elevation_rad"]),
                "sigma_range_m": float(row["sigma_range_m"]),
                "sigma_az_rad": float(row["sigma_az_rad"]),
                "sigma_el_rad": float(row["sigma_el_rad"]),
                "psi_observer": float(row["psi_observer"]),
                "roll_observer": float(row["roll_observer"]),
                "pitch_observer": float(row["pitch_observer"]),
            }
            self._peer_bearing_to_us[obs] = rec
            nb = self._neighbors.get(obs)
            if nb is None or abs(self.st.stamp - nb["stamp"]) > max_age:
                continue
            own = self._own_bearing.get(obs)
            have_own = own is not None and abs(self.st.stamp - float(own["stamp"])) <= pair_dt
            if use_rec and not have_own:
                z_ji = bearing_xyz(rec["range_m"], rec["azimuth_rad"], rec["elevation_rad"])
                meas = reciprocal_relpos(
                    self.st.p,
                    nb["p"],
                    rec["psi_observer"],
                    rec["pitch_observer"],
                    rec["roll_observer"],
                    z_ji,
                    rec["sigma_range_m"],
                    rec["sigma_az_rad"],
                    rec["sigma_el_rad"],
                    rec["range_m"],
                    rec["azimuth_rad"],
                    rec["elevation_rad"],
                )
                if meas is not None:
                    info = self._apply_update(meas, P_j=nb["P"], fusion="ci")
                    if info.get("accepted"):
                        self._n_reciprocal += 1
            if use_my:
                self._try_mutual_yaw(obs)

    def _try_mutual_yaw(self, peer: int) -> None:
        if not bool(self.cfg["measurements"].get("use_mutual_yaw", True)):
            return
        if self.abl.get("disable_bearing"):
            return
        own = self._own_bearing.get(peer)
        rec = self._peer_bearing_to_us.get(peer)
        nb = self._neighbors.get(peer)
        if own is None or rec is None or nb is None:
            return
        pair_dt = float(self.cfg["measurements"]["mutual_yaw_max_dt_s"])
        if abs(float(own["stamp"]) - float(rec["stamp"])) > pair_dt:
            return
        z_ij = bearing_xyz(own["range_m"], own["azimuth_rad"], own["elevation_rad"])
        z_ji = bearing_xyz(rec["range_m"], rec["azimuth_rad"], rec["elevation_rad"])
        Rij = cartesian_cov_from_spherical(
            own["range_m"],
            own["azimuth_rad"],
            own["elevation_rad"],
            own["sigma_range_m"],
            own["sigma_az_rad"],
            own["sigma_el_rad"],
        )
        Rji = cartesian_cov_from_spherical(
            rec["range_m"],
            rec["azimuth_rad"],
            rec["elevation_rad"],
            rec["sigma_range_m"],
            rec["sigma_az_rad"],
            rec["sigma_el_rad"],
        )
        meas = mutual_yaw(
            self.st.psi,
            self.st.pitch,
            self.st.roll,
            z_ij,
            rec["psi_observer"],
            rec["pitch_observer"],
            rec["roll_observer"],
            z_ji,
            Rij,
            Rji,
        )
        if meas is None:
            return
        info = self._apply_update(meas, P_j=nb["P"], fusion="ci")
        if info.get("accepted"):
            self._n_mutual_yaw += 1
            self._own_bearing.pop(peer, None)
            self._peer_bearing_to_us.pop(peer, None)

    def _on_tick(self) -> None:
        now_wall = time.time()
        self._drain()
        row = state_row_from_filter(self.st, self._seq, self._n_bearing_period)
        msg = pack_state([row], self.st.stamp, "world")
        self._pub_est.publish(msg)
        if self._log is not None and self.st.stamp > 0.0:
            self._log.add_est(self.st.stamp, self.st.p, self.st.v, self.st.psi, self.st.status)
        pose = self._PoseStamped()
        pose.header.frame_id = "world"
        sec = int(self.st.stamp)
        pose.header.stamp.sec = sec
        pose.header.stamp.nanosec = int((self.st.stamp - sec) * 1e9)
        pose.pose.position.x = float(self.st.p[0])
        pose.pose.position.y = float(self.st.p[1])
        pose.pose.position.z = float(self.st.p[2])
        hy = 0.5 * float(self.st.psi)
        pose.pose.orientation.z = math.sin(hy)
        pose.pose.orientation.w = math.cos(hy)
        self._pub_pose.publish(pose)
        for item in self._tx.pop_ready(now_wall):
            kind, payload = item
            data = getattr(payload, "data", b"")
            self._n_tx_bytes += len(data)
            if kind == "bc":
                self._pub_bc.publish(payload)
            else:
                self._pub_brg.publish(payload)

    def _on_broadcast_tick(self) -> None:
        now_wall = time.time()
        self._seq += 1
        row = state_row_from_filter(self.st, self._seq, self._n_bearing_period)
        bc = pack_state([row], self.st.stamp, "world")
        brg = pack_bearing(self._pending_rebroadcast, self.st.stamp, "world")
        self._tx.push(now_wall, ("bc", bc))
        self._tx.push(now_wall, ("brg", brg))
        self._pending_rebroadcast = []
        self._n_bearing_period = 0
        if now_wall - self._last_metric_wall >= 5.0:
            elapsed = max(now_wall - self._t0_wall, 1e-6)
            rate = self._n_mutual_yaw / elapsed
            print(
                f"[swarm_loc cf={self.cf_id}] n_mutual_yaw_pairs_per_s={rate:.4f} "
                f"n_mutual_yaw={self._n_mutual_yaw} n_reciprocal={self._n_reciprocal}",
                flush=True,
            )
            self._last_metric_wall = now_wall
            if self._log is not None:
                self.flush_log(quiet=True)

    def _log_uwb_edge(self, edge, peer: int) -> None:
        use_bearing = bool(self.cfg["measurements"].get("use_bearing", True))
        if self.abl.get("disable_bearing"):
            use_bearing = False
        kind = kind_from_edge(edge, use_bearing)
        z = None
        if kind in (KIND_RELPOS, KIND_ENTRANCE_RELPOS):
            z = [
                float(edge["x"] if not isinstance(edge, dict) else edge.get("x", float("nan"))),
                float(edge["y"] if not isinstance(edge, dict) else edge.get("y", float("nan"))),
                float(edge["z"] if not isinstance(edge, dict) else edge.get("z", float("nan"))),
            ]
        self._log.add_uwb(
            float(self.st.stamp),
            kind,
            self.cf_id,
            int(peer),
            float(edge["range_m"]),
            float(edge["azimuth_rad"]),
            float(edge["elevation_rad"]),
            float(edge["sigma_range_m"]),
            float(edge["sigma_az_rad"]),
            float(edge["sigma_el_rad"]),
            float(self.st.psi),
            float(self.st.roll),
            float(self.st.pitch),
            z,
        )

    def flush_log(self, quiet: bool = False) -> None:
        if self._log is None:
            return
        elapsed = max(time.time() - self._t0_wall, 1e-6)
        n_upd = max(self._n_update, 1)
        n_prop = max(self._n_prop, 1)
        extra = {}
        for name, rec in self._nis_by.items():
            key = "".join(c if c.isalnum() else "_" for c in name)
            extra[f"nis_n_{key}"] = rec["n"]
            extra[f"nis_mean_{key}"] = rec["sum"] / max(rec["n"], 1)
            extra[f"nis_rej_{key}"] = rec["rej"]
        self._log.set_stats(
            n_nis_reject=self._n_nis,
            n_update=self._n_update,
            n_accept=self._n_accept,
            n_az_only=self._n_az_only,
            n_mutual_yaw=self._n_mutual_yaw,
            n_entrance_mutual_yaw=self._n_entrance_mutual_yaw,
            n_entrance_obs=self._n_entrance_obs,
            n_entrance_az_only=self._n_entrance_az_only,
            n_yaw_mode_triggers=self._yaw_guard.n_triggered,
            n_reciprocal=self._n_reciprocal,
            n_mutual_yaw_pairs_per_s=self._n_mutual_yaw / elapsed,
            nis_reject_rate=self._n_nis / n_upd if self._n_update else 0.0,
            mean_nis=self._sum_nis / max(self._n_nis_samples, 1) if self._n_nis_samples else float("nan"),
            cpu_prop_s=self._cpu_prop,
            cpu_update_s=self._cpu_upd,
            n_prop=self._n_prop,
            cpu_per_step_s=(self._cpu_prop / n_prop) + (self._cpu_upd / n_upd),
            n_tx_bytes=self._n_tx_bytes,
            comms_bytes_per_s=self._n_tx_bytes / elapsed,
            wall_s=elapsed,
            n_diverged=1.0 if self.st.status == STATUS_DIVERGED else 0.0,
            **extra,
        )
        path = self._log.save()
        if not quiet:
            print(f"[swarm_loc cf={self.cf_id}] wrote measurements {path}", flush=True)


def _run_node(args) -> int:
    cfg = load_config(resolve_config_path(args.config))
    topics = estimator_subscription_topics(args.cf_id, args.num_drones, cfg)
    leaked = topics_contain_truth(topics)
    if leaked:
        print(f"[swarm_loc] refuse to start: truth topics in plan: {leaked}", file=sys.stderr)
        return 2

    import rclpy
    from rclpy.node import Node

    class _Node(Node):
        def __init__(self):
            super().__init__(f"swarm_loc_{args.cf_id}")
            self.impl = SwarmLocNode(
                self,
                args.cf_id,
                args.num_drones,
                cfg,
                log_path=args.log_measurements or None,
            )

    rclpy.init()
    node = _Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.impl.flush_log()
        node.destroy_node()
        rclpy.shutdown()
    return 0


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

    cfg = load_config(resolve_config_path("configs/estimation/swarm_loc.yaml"))
    topics = estimator_subscription_topics(1, 3, cfg)
    check("1 own rio", "/cf_1/rio/delta" in topics)
    check("1b own uwb", "/cf_1/uwb/edges" in topics)
    check("1c neighbor broadcast", "/cf_0/swarm_loc/broadcast" in topics)
    check("1d no self broadcast sub", "/cf_1/swarm_loc/broadcast" not in topics)
    check("1f neighbor rebroadcast", "/cf_0/swarm_loc/bearing_rebroadcast" in topics)
    check("1g no self rebroadcast sub", "/cf_1/swarm_loc/bearing_rebroadcast" not in topics)
    check("1e entrance peer edges", "/uwb/peer_1000/edges" in topics)
    leaked = topics_contain_truth(topics)
    check("2 no truth topics", leaked == [], str(leaked))
    check(
        "2b odom would be caught",
        "/cf_0/odom" in topics_contain_truth(["/cf_0/odom"]),
    )
    check(
        "2c edges_truth caught",
        "/uwb/edges_truth" in topics_contain_truth(["/uwb/edges_truth"]),
    )
    check("3 log path file", resolve_log_path("run/cf_0.npz", 2).name == "cf_0.npz")
    check("3b log path dir", str(resolve_log_path("run", 2)).replace("\\", "/").endswith("run/cf_2.npz"))

    # R1 regression: observer routing. An entrance-observed row about ANOTHER
    # drone must be dropped by drone 0 — it must never reach the own-bearing
    # cache / rebroadcast queue (only the 'own' branch populates those).
    ent = entrance_device_id(cfg)
    check("4 entrance id from estimator config", ent == 1000)
    check(
        "4b topic uses config device id",
        f"/uwb/peer_{ent}/edges" in topics,
    )
    check("5 entrance row about peer j dropped", classify_uwb_row(ent, 2, 0, ent) == "drop")
    check("5b entrance row about self routed", classify_uwb_row(ent, 0, 0, ent) == "entrance_observed")
    check("5c own row is own", classify_uwb_row(0, 2, 0, ent) == "own")
    check("5d own row to entrance peer is own", classify_uwb_row(0, ent, 0, ent) == "own")
    check("5e other drone's row dropped", classify_uwb_row(2, 0, 0, ent) == "drop")
    check(
        "5f entrance never classifies as own",
        all(classify_uwb_row(ent, pid, 0, ent) != "own" for pid in (0, 1, 2, ent)),
    )
    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--config", default="configs/estimation/swarm_loc.yaml")
    parser.add_argument("--cf-id", type=int, default=0)
    parser.add_argument("--num-drones", type=int, default=3)
    parser.add_argument(
        "--log-measurements",
        default="",
        help="directory or .npz path; one file per drone for central_reference.py",
    )
    args = parser.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    sys.exit(_run_node(args))


if __name__ == "__main__":
    main()
