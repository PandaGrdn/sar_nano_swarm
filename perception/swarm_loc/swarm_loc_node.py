#!/usr/bin/env python3
"""swarm_loc_node.py — per-drone rclpy wrapper for the swarm-loc EKF (P2-5/P2-6).

One process per drone. Odometry enters only via /cf_<id>/rio/delta.
Must never subscribe to /cf_*/odom, /uwb/edges_truth, or /cf_*/flow/debug_truth.

Usage (setup_env.sh sourced):
    python3 -u perception/swarm_loc/swarm_loc_node.py --cf-id 0 --num-drones 3
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

from ekf import propagate, update  # noqa: E402
from measurements import (  # noqa: E402
    cartesian_cov_from_spherical,
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


def estimator_subscription_topics(cf_id: int, num_drones: int, cfg: dict) -> List[str]:
    """Topics this node is allowed to subscribe to. Used by --selftest and the gate."""
    n = int(num_drones)
    k = min(int(cfg["comms"]["max_neighbors"]), max(n, 1))
    topics = [
        f"/cf_{cf_id}/rio/delta",
        f"/cf_{cf_id}/uwb/edges",
        f"/uwb/peer_1000/edges",
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

    def __init__(self, node, cf_id: int, num_drones: int, cfg: dict):
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2

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
        self._n_mutual_yaw = 0
        self._n_reciprocal = 0
        self._t0_wall = time.time()
        self._last_metric_wall = self._t0_wall
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

    def _on_rio(self, msg) -> None:
        if self.abl.get("disable_rio"):
            return
        rows = unpack_rio(msg)
        if rows.size == 0:
            return
        delta = rio_delta_from_row(rows[0])
        if self.st.status != STATUS_DIVERGED:
            self.st = propagate(self.st, delta, self.cfg)
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
            if int(row["observer_id"]) != self.cf_id:
                continue
            surveyed = bool(flags & FLAG_PEER_IS_SURVEYED)
            if surveyed and self.abl.get("disable_entrance"):
                continue
            edge = row
            if not use_bearing:
                # force range-only path by clearing bearing
                edge = {n: row[n] for n in row.dtype.names}
                edge["flags"] = flags & ~FLAG_BEARING_VALID
                edge["z"] = float("nan")
            if surveyed:
                meas = from_edge(self.st, None, edge, self.cfg)
                if meas is not None:
                    self.st, _ = update(self.st, meas, self.cfg)
                continue
            nb = self._neighbors.get(peer)
            if nb is None:
                continue
            if abs(self.st.stamp - nb["stamp"]) > max_age:
                continue
            meas = from_edge(self.st, nb["p"], edge, self.cfg)
            if meas is not None:
                self.st, _ = update(self.st, meas, self.cfg, P_j=nb["P"], fusion="ci")
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
                    mrr = range_rate(self.st.p, self.st.v, nb["p"], nb["v"], dprime, sig)
                    if mrr is not None:
                        self.st, _ = update(self.st, mrr, self.cfg, P_j=nb["P"], fusion="ci")

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
                    self.st, info = update(self.st, meas, self.cfg, P_j=nb["P"], fusion="ci")
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
        self.st, info = update(self.st, meas, self.cfg, P_j=nb["P"], fusion="ci")
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
        pose = PoseStamped()
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
            self.impl = SwarmLocNode(self, args.cf_id, args.num_drones, cfg)

    rclpy.init()
    node = _Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--config", default="configs/estimation/swarm_loc.yaml")
    parser.add_argument("--cf-id", type=int, default=0)
    parser.add_argument("--num-drones", type=int, default=3)
    args = parser.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    sys.exit(_run_node(args))


if __name__ == "__main__":
    main()
