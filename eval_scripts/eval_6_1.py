#!/usr/bin/env python3
"""eval_6_1.py — §6.1 metrics vs Gazebo truth (eval_scripts only).

ATE/RPE, error vs hops from the entrance, yaw(t), NEES, UWB mix, mutual-yaw
rate, NIS reject rate, comms bytes/s, laptop CPU vs the GAP9 20 ms budget,
divergence counts.

Truth is `/cf_<id>/odom` recorded by `swarm_loc_gate.py --eval-dir`, never by
the estimator. No SE(3) alignment: the entrance is the gauge.

Usage:
    python3 eval_scripts/eval_6_1.py --selftest
    python3 eval_scripts/eval_6_1.py --eval-dir DIR
    python3 eval_scripts/eval_6_1.py --logs DIR --truth FILE --estimates FILE --out DIR
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict, deque
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in ("perception/swarm_loc", "perception/uwb_sim", "eval_scripts"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

from meas_log import (  # noqa: E402
    ID_MEAS_NAME,
    KIND_ENTRANCE_RANGE,
    KIND_ENTRANCE_RELPOS,
    KIND_RANGE,
    KIND_RELPOS,
    KIND_RECIPROCAL,
    MeasurementLogger,
    load_run,
)
from state import wrap_psi  # noqa: E402
from swarm_msgs import STATE_DTYPE, unpack_triu  # noqa: E402
from eval_6_1_plots import SERIES_KEYS, write_plots  # noqa: E402

ENTRANCE_ID = 1000
RPE_DT_S = 1.0
# Window for the time-resolved hop assignment (same 1 s binning as
# entrance_edges_vs_time). A hop is the BFS distance in the graph of UWB
# edges active within the sample's window, so a drone can be hop 1 early
# and hop 2+ deep in the corridor (plan §5 P2-8, §6.1 headline plot).
HOP_WINDOW_S = 1.0
# AGENTS.md §6.8 / plan §6.1: ~150 int-GOp/s. At 50 Hz the step budget is 20 ms.
# Numbers here are laptop `perf_counter` times, not GAP9 measurements.
GAP9_BUDGET_S = 0.020
CHI2_95_3 = 7.814727903251179

TRUTH_DTYPE = np.dtype(
    [
        ("stamp", "<f8"),
        ("p_x", "<f4"),
        ("p_y", "<f4"),
        ("p_z", "<f4"),
        ("psi", "<f4"),
    ]
)


# A truth row whose header stamp is at/below this is UNSET, not "t = 0".
# Same constant and same reason as swarm_loc_gate.ODOM_SIM_STAMP_MIN_S: the
# /cf_*/odom stream interleaves zero-stamped (pre-clock / unstamped bridge)
# messages with properly sim-stamped ones, so the recorded truth array is
# neither sorted nor uniformly stamped. See P2_DEVIATIONS BUG B4.
TRUTH_SIM_STAMP_MIN_S = 1e-3

# Metrics are scored over [t0, t1] (sim seconds). The gate sets t0 to the start
# of the scripted path: before it, reset_pose teleports every drone to its layout
# slot and hover height, a truth discontinuity no estimator can follow. The
# window is saved beside the bundle so offline re-evaluation reproduces the score.
SCORE_WINDOW_FILE = "score_window.json"


def clip_to_window(
    est: Optional[np.ndarray], t0: Optional[float], t1: Optional[float]
) -> Optional[np.ndarray]:
    """Estimate rows with t0 <= stamp <= t1; a None bound is open."""
    if est is None or getattr(est, "size", 0) == 0:
        return est
    stamps = np.asarray(est["stamp"], dtype=np.float64)
    keep = np.ones(stamps.size, dtype=bool)
    if t0 is not None:
        keep &= stamps >= float(t0)
    if t1 is not None:
        keep &= stamps <= float(t1)
    return est[keep]


def write_score_window(
    out_dir: Path, t0: Optional[float], t1: Optional[float], source: str
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    with (out_dir / SCORE_WINDOW_FILE).open("w", encoding="utf-8") as f:
        json.dump({"t0": t0, "t1": t1, "source": source}, f, indent=2)


def load_score_window(
    eval_dir: Optional[Path],
) -> Tuple[Optional[float], Optional[float], str]:
    if eval_dir is None or not (eval_dir / SCORE_WINDOW_FILE).is_file():
        return None, None, ""
    with (eval_dir / SCORE_WINDOW_FILE).open("r", encoding="utf-8") as f:
        w = json.load(f)
    return w.get("t0"), w.get("t1"), str(w.get("source", ""))


def sanitize_truth(truth: np.ndarray) -> np.ndarray:
    """Drop unset/non-finite stamps, sort by stamp, drop duplicate stamps.

    `interp_pose` below is a binary search (`np.searchsorted`) and its span
    guard reads `ts[0]` / `ts[-1]` — both are only meaningful on a sorted,
    real-stamped array. The recorder hands us neither (BUG B4): roughly half
    the rows carry stamp 0 and they are interleaved with the real ones, so the
    raw array has thousands of descending steps. Feeding that to interp_pose
    pairs each estimate with an essentially arbitrary truth pose (ATE and NEES
    inflated by ~6x and ~90x on the 2026-09-11 triangle_forward run), or — when
    the LAST row happens to be zero-stamped — makes `t > ts[-1]` reject every
    estimate and the metrics come out NaN. Sanitizing is not a threshold
    change: it only removes rows that were never valid truth samples.
    """
    if truth is None or getattr(truth, "size", 0) == 0:
        return truth
    ts = truth["stamp"].astype(np.float64)
    keep = np.isfinite(ts) & (ts > TRUTH_SIM_STAMP_MIN_S)
    out = truth[keep]
    if out.size == 0:
        return out
    order = np.argsort(out["stamp"].astype(np.float64), kind="stable")
    out = out[order]
    _, first = np.unique(out["stamp"].astype(np.float64), return_index=True)
    return out[np.sort(first)]


def interp_pose(truth: np.ndarray, t: float) -> Optional[np.ndarray]:
    """Linear p, wrap-aware yaw. None if t is outside the truth span.

    Assumes `truth` is already sanitized (sorted, real stamps) — callers go
    through `sanitize_truth` once per drone rather than per row.
    """
    if truth.size < 2:
        return None
    ts = truth["stamp"].astype(np.float64)
    if t < ts[0] or t > ts[-1]:
        return None
    i = int(np.searchsorted(ts, t, side="right") - 1)
    i = max(0, min(i, ts.size - 2))
    t0, t1 = ts[i], ts[i + 1]
    a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
    p0 = np.array([truth[i]["p_x"], truth[i]["p_y"], truth[i]["p_z"]], dtype=np.float64)
    p1 = np.array(
        [truth[i + 1]["p_x"], truth[i + 1]["p_y"], truth[i + 1]["p_z"]], dtype=np.float64
    )
    p = p0 + a * (p1 - p0)
    yaw = wrap_psi(float(truth[i]["psi"]) + a * wrap_psi(float(truth[i + 1]["psi"]) - float(truth[i]["psi"])))
    return np.array([p[0], p[1], p[2], yaw], dtype=np.float64)


def est_pose_from_row(row) -> Tuple[float, np.ndarray]:
    names = row.dtype.names if hasattr(row, "dtype") else None
    if names and "p_x" in names:
        t = float(row["stamp"])
        p = np.array([row["p_x"], row["p_y"], row["p_z"]], dtype=np.float64)
        psi = float(row["psi"])
        return t, np.array([p[0], p[1], p[2], psi], dtype=np.float64)
    t = float(row["stamp"])
    p = np.array([row["p_x"], row["p_y"], row["p_z"]], dtype=np.float64)
    return t, np.array([p[0], p[1], p[2], float(row["psi"])], dtype=np.float64)


def _cov_p(row) -> Optional[np.ndarray]:
    names = getattr(row.dtype, "names", None) or ()
    if "cov_p_0" not in names:
        return None
    return unpack_triu(
        [row["cov_p_0"], row["cov_p_1"], row["cov_p_2"], row["cov_p_3"], row["cov_p_4"], row["cov_p_5"]],
        3,
    )


def paired_errors(est: np.ndarray, truth: np.ndarray) -> dict:
    """ATE RMSE (no alignment), RPE at 1 s, yaw RMSE, position NEES."""
    truth = sanitize_truth(truth)
    if truth is None or truth.size < 2:
        truth = np.zeros(0, dtype=TRUTH_DTYPE)
    err = []
    err_xyz = []
    yaw = []
    nees = []
    sig_p = []
    eig_min = []
    eig_max = []
    stamps = []
    poses = []
    gts = []
    for row in est:
        t, pose = est_pose_from_row(row)
        gt = interp_pose(truth, t)
        if gt is None:
            continue
        e = pose[0:3] - gt[0:3]
        err.append(float(np.linalg.norm(e)))
        err_xyz.append(e.copy())
        yaw.append(abs(wrap_psi(pose[3] - gt[3])))
        stamps.append(t)
        poses.append(pose)
        gts.append(gt.copy())
        P = _cov_p(row)
        if P is not None:
            P = 0.5 * (P + P.T) + 1e-12 * np.eye(3)
            sig_p.append(float(np.sqrt(max(np.trace(P) / 3.0, 0.0))))
            w = np.linalg.eigvalsh(P)
            eig_min.append(float(w[0]))
            eig_max.append(float(w[-1]))
            try:
                nees.append(float(e @ np.linalg.solve(P, e)))
            except np.linalg.LinAlgError:
                nees.append(float("inf"))
        else:
            sig_p.append(float("nan"))
            eig_min.append(float("nan"))
            eig_max.append(float("nan"))
            nees.append(float("nan"))
    err = np.asarray(err, dtype=np.float64)
    yaw = np.asarray(yaw, dtype=np.float64)
    ate = float(np.sqrt(np.mean(err**2))) if err.size else float("nan")
    rpe_vals = []
    rpe_t = []
    if len(stamps) >= 2:
        ts = np.asarray(stamps, dtype=np.float64)
        ps = np.asarray(poses, dtype=np.float64)
        for k, t in enumerate(ts):
            j = int(np.searchsorted(ts, t + RPE_DT_S, side="left"))
            if j >= ts.size:
                continue
            if abs(ts[j] - (t + RPE_DT_S)) > 0.25:
                continue
            d_est = ps[j, 0:3] - ps[k, 0:3]
            gt0 = interp_pose(truth, t)
            gt1 = interp_pose(truth, ts[j])
            if gt0 is None or gt1 is None:
                continue
            d_gt = gt1[0:3] - gt0[0:3]
            rpe_vals.append(float(np.linalg.norm(d_est - d_gt)))
            rpe_t.append(float(t))
    rpe = np.asarray(rpe_vals, dtype=np.float64)
    nees = np.asarray(nees, dtype=np.float64)
    xyz = np.asarray(err_xyz, dtype=np.float64) if err_xyz else np.zeros((0, 3))
    gts_a = np.asarray(gts, dtype=np.float64) if gts else np.zeros((0, 4))
    pos_a = np.asarray(poses, dtype=np.float64) if poses else np.zeros((0, 4))
    finite_nees = nees[np.isfinite(nees)]
    return {
        "n": int(err.size),
        "ate_rmse_m": ate,
        "ate_p50_m": float(np.median(err)) if err.size else float("nan"),
        "ate_p95_m": float(np.percentile(err, 95)) if err.size else float("nan"),
        "rpe_rmse_m": float(np.sqrt(np.mean(rpe**2))) if rpe.size else float("nan"),
        "yaw_rmse_rad": float(np.sqrt(np.mean(yaw**2))) if yaw.size else float("nan"),
        "err_t": stamps,
        "err_m": err.tolist(),
        "err_xyz": xyz.tolist(),
        "yaw_rad": yaw.tolist(),
        "rpe_t": rpe_t,
        "rpe_m": rpe.tolist(),
        "nees": nees.tolist(),
        "sig_p": sig_p,
        "eig_min": eig_min,
        "eig_max": eig_max,
        "est_pose": pos_a.tolist(),
        "gt_pose": gts_a.tolist(),
        "mean_xyz_m": (
            [float(np.mean(xyz[:, 0])), float(np.mean(xyz[:, 1])), float(np.mean(xyz[:, 2]))]
            if xyz.size
            else [float("nan"), float("nan"), float("nan")]
        ),
        "mean_z_m": float(np.mean(xyz[:, 2])) if xyz.size else float("nan"),
        "mean_nees": float(np.mean(finite_nees)) if finite_nees.size else float("nan"),
        "frac_nees_in_95": float(np.mean(finite_nees <= CHI2_95_3)) if finite_nees.size else float("nan"),
        "n_nees": int(finite_nees.size),
        "n_rpe": int(rpe.size),
    }


def hops_from_uwb(run: Dict[int, dict]) -> Dict[int, int]:
    """Shortest undirected hop count from entrance id 1000."""
    adj: Dict[int, set] = defaultdict(set)
    ids = set(run)
    for i in ids:
        adj[i]
    adj[ENTRANCE_ID]
    for rec in run.values():
        for row in rec["uwb"]:
            a = int(row["observer_id"])
            b = int(row["peer_id"])
            adj[a].add(b)
            adj[b].add(a)
    hop = _bfs_hops(adj)
    return {i: int(hop.get(i, -1)) for i in ids}


def _bfs_hops(adj: Dict[int, set]) -> Dict[int, int]:
    hop = {ENTRANCE_ID: 0}
    q = deque([ENTRANCE_ID])
    while q:
        u = q.popleft()
        for v in adj.get(u, ()):
            if v not in hop:
                hop[v] = hop[u] + 1
                q.append(v)
    return hop


def hops_vs_time(
    run: Dict[int, dict], window_s: float = HOP_WINDOW_S
) -> Dict[int, Dict[int, int]]:
    """Per-window BFS hop count from the entrance (id 1000).

    Returns {window_index: {drone_id: hop}} where a window covers
    [w*window_s, (w+1)*window_s). A drone with no path to the entrance in a
    window is absent from that window's dict (hop unknown, not hop -1: with
    5 s log flushes a silent window means no data, not disconnection proof).
    """
    edges_by_win: Dict[int, set] = defaultdict(set)
    for rec in run.values():
        uwb = rec.get("uwb")
        if uwb is None or getattr(uwb, "size", 0) == 0:
            continue
        wins = np.floor(np.asarray(uwb["stamp"], dtype=np.float64) / window_s).astype(int)
        for w, a, b in zip(wins, uwb["observer_id"], uwb["peer_id"]):
            edges_by_win[int(w)].add((int(a), int(b)))
    out: Dict[int, Dict[int, int]] = {}
    for w, edges in edges_by_win.items():
        adj: Dict[int, set] = defaultdict(set)
        for a, b in edges:
            adj[a].add(b)
            adj[b].add(a)
        hop = _bfs_hops(adj)
        out[w] = {i: h for i, h in hop.items() if i != ENTRANCE_ID}
    return out


def error_vs_hops_time(
    per_drone: Dict[int, dict],
    hops_t: Dict[int, Dict[int, int]],
    window_s: float = HOP_WINDOW_S,
) -> dict:
    """Bucket per-sample position errors by their instantaneous hop count.

    Each (t, |err|) sample of each drone is assigned the BFS hop of that
    drone in the UWB graph of its window. Samples in windows where the drone
    has no path to the entrance are counted as unassigned, not bucketed.
    """
    buckets: Dict[int, List[float]] = defaultdict(list)
    n_unassigned = 0
    for i, m in per_drone.items():
        ts = m.get("err_t") or []
        es = m.get("err_m") or []
        for t, e in zip(ts, es):
            h = hops_t.get(int(math.floor(float(t) / window_s)), {}).get(i, -1)
            if h > 0 and math.isfinite(e):
                buckets[h].append(float(e))
            else:
                n_unassigned += 1
    out = {}
    for h in sorted(buckets):
        a = np.asarray(buckets[h], dtype=np.float64)
        out[str(h)] = {
            "rmse_m": float(np.sqrt(np.mean(a**2))),
            "p50_m": float(np.median(a)),
            "p95_m": float(np.percentile(a, 95)),
            "n": int(a.size),
        }
    return {"window_s": float(window_s), "buckets": out, "n_unassigned": int(n_unassigned)}


def uwb_mix(run: Dict[int, dict]) -> dict:
    n = 0
    n_bearing = 0
    n_range = 0
    n_reciprocal_kind = 0
    n_az = 0
    n_my = 0
    n_ent_obs = 0
    n_ent_my = 0
    n_ent_az = 0
    n_rec_stat = 0
    n_nis = 0
    n_upd = 0
    n_div = 0
    bytes_s = []
    cpu_step = []
    my_rate = []
    for rec in run.values():
        uwb = rec["uwb"]
        n += int(uwb.shape[0])
        for row in uwb:
            k = int(row["kind"])
            if k in (KIND_RELPOS, KIND_ENTRANCE_RELPOS):
                n_bearing += 1
            elif k in (KIND_RANGE, KIND_ENTRANCE_RANGE):
                n_range += 1
            elif k == KIND_RECIPROCAL:
                n_reciprocal_kind += 1
        st = rec.get("stats") or {}
        n_az += int(st.get("n_az_only", 0))
        n_my += int(st.get("n_mutual_yaw", 0))
        n_ent_obs += int(st.get("n_entrance_obs", 0))
        n_ent_my += int(st.get("n_entrance_mutual_yaw", 0))
        n_ent_az += int(st.get("n_entrance_az_only", 0))
        n_rec_stat += int(st.get("n_reciprocal", 0))
        n_nis += int(st.get("n_nis_reject", 0))
        n_upd += int(st.get("n_update", 0))
        n_div += int(st.get("n_diverged", 0))
        if "comms_bytes_per_s" in st:
            bytes_s.append(float(st["comms_bytes_per_s"]))
        if "cpu_per_step_s" in st:
            cpu_step.append(float(st["cpu_per_step_s"]))
        if "n_mutual_yaw_pairs_per_s" in st:
            my_rate.append(float(st["n_mutual_yaw_pairs_per_s"]))
    den = max(n, 1)
    mean_cpu = float(np.mean(cpu_step)) if cpu_step else float("nan")
    return {
        "n_uwb": n,
        "frac_bearing": n_bearing / den,
        "frac_range_only": n_range / den,
        "frac_reciprocal_kind": n_reciprocal_kind / den,
        "n_reciprocal_updates": n_rec_stat,
        "n_az_only": n_az,
        "frac_az_only": n_az / den,
        "n_mutual_yaw": n_my,
        "n_entrance_obs": n_ent_obs,
        "n_entrance_mutual_yaw": n_ent_my,
        "n_entrance_az_only": n_ent_az,
        "n_mutual_yaw_pairs_per_s": float(np.mean(my_rate)) if my_rate else 0.0,
        "nis_reject_rate": n_nis / max(n_upd, 1),
        "n_update": n_upd,
        "n_nis_reject": n_nis,
        "n_diverged": n_div,
        "comms_bytes_per_s_mean": float(np.mean(bytes_s)) if bytes_s else float("nan"),
        "cpu_per_step_s": mean_cpu,
        "cpu_per_step_us": mean_cpu * 1e6 if math.isfinite(mean_cpu) else float("nan"),
        "gap9_budget_s": GAP9_BUDGET_S,
        "cpu_fits_20ms_on_laptop": bool(math.isfinite(mean_cpu) and mean_cpu < GAP9_BUDGET_S),
        "cpu_note": (
            "laptop perf_counter, not GAP9. Budget 20 ms at 50 Hz vs ~150 int-GOp/s. "
            "A step that only fits a laptop is a failed design, not a passing metric."
        ),
    }


def entrance_edges_vs_time(run: Dict[int, dict], bin_s: float = 1.0) -> dict:
    """Per-second count of edges to peer 1000, split bearing vs range."""
    by_drone: Dict[int, dict] = {}
    for i, rec in run.items():
        buckets: Dict[int, List[int]] = defaultdict(lambda: [0, 0])  # bearing, range
        uwb = rec.get("uwb")
        if uwb is None or uwb.size == 0:
            by_drone[i] = {"t": [], "bearing": [], "range": []}
            continue
        for row in uwb:
            if int(row["peer_id"]) != ENTRANCE_ID:
                continue
            sec = int(math.floor(float(row["stamp"]) / bin_s))
            k = int(row["kind"])
            if k == KIND_ENTRANCE_RELPOS:
                buckets[sec][0] += 1
            else:
                buckets[sec][1] += 1
        secs = sorted(buckets)
        by_drone[i] = {
            "t": [s * bin_s for s in secs],
            "bearing": [buckets[s][0] for s in secs],
            "range": [buckets[s][1] for s in secs],
        }
    return {str(i): v for i, v in by_drone.items()}


def nis_by_type(run: Dict[int, dict]) -> dict:
    acc: Dict[str, List[float]] = defaultdict(list)
    n_rej: Dict[str, int] = defaultdict(int)
    for rec in run.values():
        nis = rec.get("nis")
        if nis is None or getattr(nis, "size", 0) == 0:
            continue
        for row in nis:
            name = ID_MEAS_NAME.get(int(row["name_id"]), f"id_{int(row['name_id'])}")
            v = float(row["nis"])
            if math.isfinite(v):
                acc[name].append(v)
            if int(row["accepted"]) == 0:
                n_rej[name] += 1
    out = {}
    for name, vals in acc.items():
        a = np.asarray(vals, dtype=np.float64)
        out[name] = {
            "n": int(a.size),
            "mean": float(np.mean(a)),
            "p50": float(np.median(a)),
            "p95": float(np.percentile(a, 95)),
            "n_reject": int(n_rej.get(name, 0)),
            "samples": a.tolist()[:4000],
        }
    return out


def centroid_vs_shape(
    estimates: Dict[int, np.ndarray], truth: Dict[int, np.ndarray]
) -> dict:
    """Formation centroid error vs shape error (truth-aligned, no SE3)."""
    ids = sorted(set(estimates) & set(truth))
    if len(ids) < 2:
        return {"t": [], "centroid_m": [], "shape_m": [], "mean_ate_m": []}
    truth = {i: sanitize_truth(truth[i]) for i in ids}  # BUG B4 — see sanitize_truth
    t_ref = None
    for i in ids:
        est = estimates[i]
        if est is not None and getattr(est, "size", 0):
            t_ref = np.asarray(est["stamp"], dtype=np.float64)
            break
    if t_ref is None or t_ref.size == 0:
        return {"t": [], "centroid_m": [], "shape_m": [], "mean_ate_m": []}
    # subsample to ~5 Hz
    step = max(int(round(0.2 / max(float(np.median(np.diff(t_ref))), 1e-3))), 1)
    t_use = t_ref[::step]
    cent = []
    shape = []
    ate = []
    t_out = []
    for t in t_use:
        pest = []
        pgt = []
        ok = True
        for i in ids:
            row = estimates[i]
            ts = np.asarray(row["stamp"], dtype=np.float64)
            k = int(np.argmin(np.abs(ts - t)))
            if abs(ts[k] - t) > 0.25:
                ok = False
                break
            gt = interp_pose(truth[i], float(ts[k]))
            if gt is None:
                ok = False
                break
            pest.append(np.array([row[k]["p_x"], row[k]["p_y"], row[k]["p_z"]], dtype=np.float64))
            pgt.append(gt[0:3])
        if not ok or len(pest) < 2:
            continue
        pe = np.stack(pest)
        pg = np.stack(pgt)
        ce = pe.mean(axis=0)
        cg = pg.mean(axis=0)
        se = pe - ce
        sg = pg - cg
        cent.append(float(np.linalg.norm(ce - cg)))
        shape.append(float(np.sqrt(np.mean(np.sum((se - sg) ** 2, axis=1)))))
        ate.append(float(np.mean(np.linalg.norm(pe - pg, axis=1))))
        t_out.append(float(t))
    return {
        "t": t_out,
        "centroid_m": cent,
        "shape_m": shape,
        "mean_ate_m": ate,
        "mean_centroid_m": float(np.mean(cent)) if cent else float("nan"),
        "mean_shape_m": float(np.mean(shape)) if shape else float("nan"),
        "centroid_explains_ate": bool(
            cent
            and ate
            and float(np.mean(cent)) > 0.7 * float(np.mean(ate))
            and float(np.mean(shape)) < 0.5 * float(np.mean(ate))
        ),
    }


def error_vs_hops(per_drone: Dict[int, dict], hops: Dict[int, int]) -> Dict[int, float]:
    buckets: Dict[int, List[float]] = defaultdict(list)
    for i, m in per_drone.items():
        h = hops.get(i, -1)
        ate = m.get("ate_rmse_m", float("nan"))
        if h > 0 and math.isfinite(ate):
            buckets[h].append(ate)
    return {h: float(np.mean(v)) for h, v in sorted(buckets.items())}


def write_eval_bundle(out_dir: Path, estimates: Dict[int, np.ndarray], truth: Dict[int, np.ndarray]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    est_kw = {f"cf_{i}": np.asarray(a) for i, a in estimates.items()}
    # BUG B4: persist truth already sanitized so the bundle on disk is usable
    # by any offline consumer that assumes a sorted, real-stamped series.
    tru_kw = {
        f"cf_{i}": sanitize_truth(np.asarray(a, dtype=TRUTH_DTYPE))
        for i, a in truth.items()
    }
    np.savez(out_dir / "estimates.npz", **est_kw)
    np.savez(out_dir / "truth.npz", **tru_kw)


def load_structured_npz(path: Path, dtype=None) -> Dict[int, np.ndarray]:
    if not path.is_file():
        return {}
    z = np.load(path, allow_pickle=False)
    out: Dict[int, np.ndarray] = {}
    for k in z.files:
        if not k.startswith("cf_"):
            continue
        i = int(k.split("_", 1)[1])
        arr = z[k]
        if dtype is not None and arr.size:
            arr = np.array(arr, dtype=dtype)
        out[i] = arr
    return out


def load_estimates_from_logs(run: Dict[int, dict]) -> Dict[int, np.ndarray]:
    return {i: rec["estimate"] for i, rec in run.items()}


def evaluate(
    logs: Optional[Dict[int, dict]],
    truth: Dict[int, np.ndarray],
    estimates: Dict[int, np.ndarray],
    out_dir: Optional[Path] = None,
    write_png: bool = True,
    show: bool = False,
    score_t0: Optional[float] = None,
    score_t1: Optional[float] = None,
    score_source: str = "",
) -> dict:
    ids = sorted(set(estimates) | set(truth) | (set(logs) if logs else set()))
    # Divergence counts the whole recording: a filter that diverged during the
    # teleport still diverged, even though those rows are not scored.
    n_div_est = 0
    for est in estimates.values():
        if hasattr(est, "dtype") and est.size and est.dtype.names and "status" in est.dtype.names:
            n_div_est += int(np.sum(est["status"] != 0))
    n_outside: Dict[int, int] = {}
    if score_t0 is not None or score_t1 is not None:
        clipped = {}
        for i, est in estimates.items():
            clipped[i] = clip_to_window(est, score_t0, score_t1)
            n_outside[i] = int(est.size) - int(clipped[i].size) if est is not None else 0
        estimates = clipped
    per: Dict[int, dict] = {}
    for i in ids:
        est = estimates.get(i)
        tru = truth.get(i)
        if est is None or tru is None or est.size == 0 or tru.size == 0:
            per[i] = {"n": 0, "ate_rmse_m": float("nan")}
            continue
        per[i] = paired_errors(est, tru)
        per[i]["yaw_rmse_deg"] = (
            math.degrees(per[i]["yaw_rmse_rad"]) if math.isfinite(per[i]["yaw_rmse_rad"]) else float("nan")
        )
    hops = hops_from_uwb(logs) if logs else {i: -1 for i in ids}
    hops_ate = error_vs_hops(per, hops)
    hops_t = hops_vs_time(logs) if logs else {}
    hops_ate_time = error_vs_hops_time(per, hops_t)
    mix = uwb_mix(logs) if logs else {}
    if n_div_est:
        mix["n_diverged_estimate_rows"] = n_div_est
    nis_raw = nis_by_type(logs) if logs else {}
    diag = {
        "ate_vs_hops_time": hops_ate_time,
        "entrance_edges": entrance_edges_vs_time(logs) if logs else {},
        "centroid_shape": centroid_vs_shape(estimates, truth),
        "nis_by_type": {
            k: {kk: vv for kk, vv in v.items() if kk != "samples"} for k, v in nis_raw.items()
        },
        "nis_samples": {k: v.get("samples", []) for k, v in nis_raw.items()},
    }
    report = {
        "per_drone": {
            str(i): {k: v for k, v in per[i].items() if k not in SERIES_KEYS}
            for i in per
        },
        "hops": {str(i): hops.get(i, -1) for i in ids},
        "ate_vs_hops_m": {str(h): hops_ate[h] for h in hops_ate},
        # Time-resolved headline metric: samples bucketed by instantaneous
        # BFS hop in each HOP_WINDOW_S window (run-aggregate kept above).
        "ate_vs_hops_time": hops_ate_time,
        "mix": mix,
        "rpe_dt_s": RPE_DT_S,
        "score_window": {
            "t0": score_t0,
            "t1": score_t1,
            "source": score_source,
            "n_estimates_outside": {str(i): n for i, n in n_outside.items()},
        },
        "aoa_fov_note": "bearing cone ±45° (aoa_fov_deg 90); live mutual-yaw often ~0",
        "diag": {
            "entrance_edges": diag["entrance_edges"],
            "centroid_shape": {
                k: v
                for k, v in diag["centroid_shape"].items()
                if k not in ("t", "centroid_m", "shape_m", "mean_ate_m")
            },
            "centroid_shape_series": {
                k: diag["centroid_shape"].get(k, [])
                for k in ("t", "centroid_m", "shape_m", "mean_ate_m")
            },
            "nis_by_type": diag["nis_by_type"],
        },
    }
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        yaw_series = {str(i): {"t": per[i].get("err_t", []), "yaw_err_rad": per[i].get("yaw_rad", [])} for i in per}
        with (out_dir / "metrics_6_1.json").open("w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
        tb = hops_ate_time["buckets"]
        np.savez(
            out_dir / "metrics_6_1.npz",
            hops=np.array(list(hops_ate.keys()), dtype=np.int32),
            ate_vs_hops=np.array(list(hops_ate.values()), dtype=np.float64),
            hops_time=np.array([int(h) for h in tb], dtype=np.int32),
            ate_vs_hops_time_rmse=np.array([tb[h]["rmse_m"] for h in tb], dtype=np.float64),
            ate_vs_hops_time_n=np.array([tb[h]["n"] for h in tb], dtype=np.int64),
            hop_window_s=np.float64(hops_ate_time["window_s"]),
        )
        with (out_dir / "yaw_error.json").open("w", encoding="utf-8") as f:
            json.dump(yaw_series, f)
        if write_png:
            report["plot_files"] = write_plots(
                per, hops_ate, mix, estimates, truth, out_dir, show=show, diag=diag
            )
            with (out_dir / "metrics_6_1.json").open("w", encoding="utf-8") as f:
                json.dump(report, f, indent=2, default=str)
    return report


def print_report(report: dict) -> None:
    sw = report.get("score_window") or {}
    if sw.get("t0") is not None or sw.get("t1") is not None:
        print(
            f"[eval_6_1] scoring window t0={sw.get('t0')} t1={sw.get('t1')} "
            f"({sw.get('source', '')}); estimate rows outside={sw.get('n_estimates_outside', {})}"
        )
    else:
        print("[eval_6_1] scoring window: full recording (none set)")
    print("[eval_6_1] per drone:")
    for i, m in report["per_drone"].items():
        print(
            f"  cf_{i}  ATE_rmse={m.get('ate_rmse_m', float('nan')):.3f} m  "
            f"RPE_rmse={m.get('rpe_rmse_m', float('nan')):.3f} m  "
            f"yaw_rmse={m.get('yaw_rmse_deg', float('nan')):.2f} deg  "
            f"mean_z={m.get('mean_z_m', float('nan')):+.3f} m  "
            f"NEES={m.get('mean_nees', float('nan')):.2f}  "
            f"NEES_in95={m.get('frac_nees_in_95', float('nan')):.3f}  n={m.get('n', 0)}"
        )
    print(f"  hops={report['hops']}")
    print(f"  ATE vs hops (m)={report['ate_vs_hops_m']}")
    ht = report.get("ate_vs_hops_time") or {}
    hb = ht.get("buckets") or {}
    if hb:
        bits = [f"hop {h}: rmse={v['rmse_m']:.3f} m (n={v['n']})" for h, v in sorted(hb.items(), key=lambda kv: int(kv[0]))]
        print(
            f"  err vs hops (time-resolved, {ht.get('window_s', HOP_WINDOW_S):.0f} s windows): "
            + "; ".join(bits)
            + f"; unassigned={ht.get('n_unassigned', 0)}"
        )
    mix = report.get("mix") or {}
    if mix:
        print(
            f"  mix bearing={mix.get('frac_bearing', float('nan')):.3f} "
            f"range={mix.get('frac_range_only', float('nan')):.3f} "
            f"az_only={mix.get('frac_az_only', float('nan')):.3f} "
            f"reciprocal_updates={mix.get('n_reciprocal_updates', 0)}"
        )
        print(
            f"  n_mutual_yaw_pairs_per_s={mix.get('n_mutual_yaw_pairs_per_s', 0):.4f}  "
            f"n_entrance_obs={mix.get('n_entrance_obs', 0)}  "
            f"NIS reject={mix.get('nis_reject_rate', float('nan')):.4f}  "
            f"comms_B/s={mix.get('comms_bytes_per_s_mean', float('nan')):.1f}  "
            f"CPU={mix.get('cpu_per_step_us', float('nan')):.1f} us/step  "
            f"GAP9 budget={1e3 * GAP9_BUDGET_S:.0f} ms  "
            f"diverged={mix.get('n_diverged', 0)}"
        )
        print(f"  {mix.get('cpu_note', '')}")
    cs = (report.get("diag") or {}).get("centroid_shape") or {}
    if cs:
        print(
            f"  centroid_mean={cs.get('mean_centroid_m', float('nan')):.3f} m  "
            f"shape_mean={cs.get('mean_shape_m', float('nan')):.3f} m  "
            f"centroid_explains_ATE={cs.get('centroid_explains_ate', False)}"
        )
    nis_t = (report.get("diag") or {}).get("nis_by_type") or {}
    if nis_t:
        bits = [
            f"{k}:mean={v.get('mean', float('nan')):.2f}/n={v.get('n', 0)}/rej={v.get('n_reject', 0)}"
            for k, v in sorted(nis_t.items())
        ]
        print("  NIS by type: " + "; ".join(bits))
    print(f"  {report.get('aoa_fov_note', '')}")
    files = report.get("plot_files") or []
    if files:
        print("[eval_6_1] matplotlib PNGs:")
        for p in files:
            print(f"  {p}")


def _synth_state_row(t, i, p, psi, sig=0.05, status=0):
    v = np.zeros(1, dtype=STATE_DTYPE)[0]
    v["stamp"] = t
    v["drone_id"] = i
    v["p_x"], v["p_y"], v["p_z"] = p
    v["psi"] = psi
    s2 = float(sig) ** 2
    v["cov_p_0"] = s2
    v["cov_p_3"] = s2
    v["cov_p_5"] = s2
    v["cov_psi"] = (math.radians(5.0)) ** 2
    v["status"] = status
    return v


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

    t = np.linspace(0.0, 10.0, 501)
    truth = {}
    estimates = {}
    offset = np.array([0.05, 0.05, 0.05])
    sig = 0.05
    for i, x0 in ((0, 0.0), (1, 1.5)):
        tru = np.zeros(t.size, dtype=TRUTH_DTYPE)
        est = np.zeros(t.size, dtype=STATE_DTYPE)
        for k, tk in enumerate(t):
            p = np.array([x0 + 0.2 * math.sin(0.3 * tk), 0.05 * tk, 0.5])
            psi = 0.1 * math.sin(0.2 * tk)
            tru[k]["stamp"] = tk
            tru[k]["p_x"], tru[k]["p_y"], tru[k]["p_z"] = p
            tru[k]["psi"] = psi
            est[k] = _synth_state_row(tk, i, p + offset, psi + 0.02, sig=sig)
        truth[i] = tru
        estimates[i] = est

    log0 = MeasurementLogger(0)
    log1 = MeasurementLogger(1)
    log0.add_uwb(1.0, KIND_ENTRANCE_RELPOS, 0, ENTRANCE_ID, 2.0, 0.0, 0.0, 0.08, 0.08, 0.08, 0.0, 0.0, 0.0, [2, 0, 0])
    log0.add_uwb(1.0, KIND_RANGE, 0, 1, 1.5, float("nan"), float("nan"), 0.08, float("nan"), float("nan"), 0.0, 0.0, 0.0)
    log1.add_uwb(1.0, KIND_RANGE, 1, 0, 1.5, float("nan"), float("nan"), 0.08, float("nan"), float("nan"), 0.0, 0.0, 0.0)
    log0.set_stats(
        n_nis_reject=2,
        n_update=100,
        n_az_only=5,
        n_mutual_yaw=0,
        n_reciprocal=7,
        n_mutual_yaw_pairs_per_s=0.0,
        cpu_per_step_s=0.0004,
        comms_bytes_per_s=800.0,
        n_diverged=0,
    )
    log1.set_stats(
        n_nis_reject=0,
        n_update=80,
        n_az_only=1,
        n_mutual_yaw=0,
        n_reciprocal=3,
        n_mutual_yaw_pairs_per_s=0.0,
        cpu_per_step_s=0.0005,
        comms_bytes_per_s=700.0,
        n_diverged=0,
    )
    import tempfile

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        log0.save(td / "cf_0.npz")
        log1.save(td / "cf_1.npz")
        write_eval_bundle(td, estimates, truth)
        run = load_run(td)
        hops = hops_from_uwb(run)
        check("1 hop drone 0 is 1", hops.get(0) == 1, str(hops))
        check("1b hop drone 1 is 2", hops.get(1) == 2, str(hops))
        loaded_t = load_structured_npz(td / "truth.npz", TRUTH_DTYPE)
        loaded_e = load_structured_npz(td / "estimates.npz")
        check("2 bundle roundtrip", 0 in loaded_t and loaded_e[0].shape[0] == t.size)
        report = evaluate(
            run, loaded_t, loaded_e, out_dir=td / "out", write_png=True
        )
        ate0 = report["per_drone"]["0"]["ate_rmse_m"]
        expect = float(np.linalg.norm(offset))
        check("3 ATE is the planted offset", abs(ate0 - expect) < 0.002, f"ate={ate0:.4f} expect={expect:.4f}")
        rpe0 = report["per_drone"]["0"]["rpe_rmse_m"]
        check("4 RPE ~0 for constant offset", rpe0 < 0.01, f"rpe={rpe0:.4f}")
        nees = report["per_drone"]["0"]["mean_nees"]
        check("5 NEES ~3 for matched P", abs(nees - 3.0) < 0.2, f"nees={nees:.3f}")
        check("6 hops plot or json", (td / "out" / "metrics_6_1.json").is_file())
        pngs = list((td / "out" / "plots").glob("*.png")) if (td / "out" / "plots").is_dir() else []
        check("6b matplotlib pngs", len(pngs) >= 8, f"n={len(pngs)}")
        check("6c dashboard", (td / "out" / "plots" / "00_dashboard.png").is_file())
        check("6d entrance-edge plot", (td / "out" / "plots" / "11_entrance_edges.png").is_file())
        cs = report.get("diag", {}).get("centroid_shape") or {}
        check("6e centroid vs shape keys", "mean_centroid_m" in cs)
        mix = report["mix"]
        check("6f n_entrance_obs in mix", mix.get("n_entrance_obs") == 0, str(mix.get("n_entrance_obs")))
        mz = report["per_drone"]["0"].get("mean_z_m", float("nan"))
        check("6g mean_z planted", abs(mz - 0.05) < 0.002, f"mean_z={mz}")
        check("7 bearing fraction", mix["frac_bearing"] > 0.2)
        check("7b NIS reject rate", abs(mix["nis_reject_rate"] - 2 / 180) < 1e-9, str(mix["nis_reject_rate"]))
        check("8 CPU under 20 ms (laptop)", mix["cpu_fits_20ms_on_laptop"])
        check("9 ATE vs hops has hop 1 and 2", set(report["ate_vs_hops_m"]) >= {"1", "2"})
        ht = report.get("ate_vs_hops_time") or {}
        check(
            "9b time-resolved buckets 1 and 2 in report",
            set(ht.get("buckets", {})) >= {"1", "2"},
            str(ht),
        )
        check("10 no SE3 needed: hop-1 ATE == offset", abs(float(report["ate_vs_hops_m"]["1"]) - expect) < 0.002)

    # Time-resolved hops: one drone that is hop 1 early and hop 2 late.
    # Direct entrance edge only at t=1; at t=6 drone 0 reaches the entrance
    # only through drone 1 (relay), so its hop changes from 1 to 2.
    log_a = MeasurementLogger(0)
    log_b = MeasurementLogger(1)
    nanv = float("nan")
    log_a.add_uwb(1.0, KIND_ENTRANCE_RANGE, 0, ENTRANCE_ID, 2.0, nanv, nanv, 0.08, nanv, nanv, 0.0, 0.0, 0.0)
    log_b.add_uwb(6.0, KIND_ENTRANCE_RANGE, 1, ENTRANCE_ID, 2.0, nanv, nanv, 0.08, nanv, nanv, 0.0, 0.0, 0.0)
    log_a.add_uwb(6.0, KIND_RANGE, 0, 1, 1.5, nanv, nanv, 0.08, nanv, nanv, 0.0, 0.0, 0.0)
    with tempfile.TemporaryDirectory() as td2:
        td2 = Path(td2)
        log_a.save(td2 / "cf_0.npz")
        log_b.save(td2 / "cf_1.npz")
        run2 = load_run(td2)
        check("11 aggregate hop still 1", hops_from_uwb(run2).get(0) == 1)
        ht2 = hops_vs_time(run2)
        check("11b hop 1 early", ht2.get(1, {}).get(0) == 1, str(ht2))
        check("11c hop 2 late", ht2.get(6, {}).get(0) == 2, str(ht2))
        per2 = {0: {"err_t": [1.5, 3.5, 6.5], "err_m": [0.1, 0.2, 0.3]}}
        evh = error_vs_hops_time(per2, ht2)
        b = evh["buckets"]
        check("11d both hop buckets populated", set(b) == {"1", "2"}, str(evh))
        check(
            "11e bucketed by instantaneous hop",
            b.get("1", {}).get("n") == 1
            and abs(b.get("1", {}).get("rmse_m", 0.0) - 0.1) < 1e-9
            and b.get("2", {}).get("n") == 1
            and abs(b.get("2", {}).get("rmse_m", 0.0) - 0.3) < 1e-9,
            str(b),
        )
        check("11f no-graph window unassigned", evh["n_unassigned"] == 1, str(evh))

    # ---- 12: BUG B4 — unsorted / zero-stamped truth (the real recorder shape).
    # The recorder interleaves zero-stamped odom rows with sim-stamped ones, so
    # the truth array is ~half junk and thousands of steps descend. Build that
    # shape from the clean series above and pin both failure modes.
    tru_c = truth[0]
    est_c = estimates[0]
    ref = paired_errors(est_c, tru_c)
    check("12 clean truth still pairs", ref["n"] > 400 and math.isfinite(ref["ate_rmse_m"]), str(ref["n"]))
    check("12a clean truth ATE == offset", abs(ref["ate_rmse_m"] - float(np.linalg.norm(offset))) < 2e-3,
          f"{ref['ate_rmse_m']:.4f}")
    check("12b clean truth NEES ~ 3", abs(ref["mean_nees"] - 3.0) < 0.3, f"{ref['mean_nees']:.3f}")

    zero = np.zeros(tru_c.size, dtype=TRUTH_DTYPE)  # stamp 0, pose 0 — the intruders
    inter = np.empty(tru_c.size * 2, dtype=TRUTH_DTYPE)
    inter[0::2] = tru_c
    inter[1::2] = zero          # ends on a zero-stamped row, like cf_0/cf_1 live
    check("12c interleaved truth is unsorted",
          int(np.sum(np.diff(inter["stamp"]) < 0)) > 100 and inter["stamp"][-1] == 0.0)
    san = sanitize_truth(inter)
    check("12d sanitize drops zero stamps", int(np.sum(san["stamp"] <= TRUTH_SIM_STAMP_MIN_S)) == 0)
    check("12e sanitize sorts", bool(np.all(np.diff(san["stamp"]) > 0)))
    check("12f sanitize keeps every real row", san.size == int(np.sum(tru_c["stamp"] > TRUTH_SIM_STAMP_MIN_S)),
          f"{san.size} vs {int(np.sum(tru_c['stamp'] > TRUTH_SIM_STAMP_MIN_S))}")
    check("12g sanitize preserves poses",
          bool(np.allclose(san["p_x"], tru_c["p_x"][tru_c["stamp"] > TRUTH_SIM_STAMP_MIN_S])))

    bad = paired_errors(est_c, inter)
    check("12h interleaved truth now pairs like clean", bad["n"] == ref["n"], f"{bad['n']} vs {ref['n']}")
    check("12i interleaved truth ATE unchanged", abs(bad["ate_rmse_m"] - ref["ate_rmse_m"]) < 1e-9,
          f"{bad['ate_rmse_m']:.4f}")
    check("12j interleaved truth NEES unchanged", abs(bad["mean_nees"] - ref["mean_nees"]) < 1e-9)

    # Pin what the OLD code did, so a regression cannot pass silently: the raw
    # interleaved array ending on a zero stamp rejects every estimate (NaN
    # metrics, the 2026-09-11 re-run), and an array ending on a real stamp
    # pairs garbage (inflated ATE/NEES, the 2026-09-11 scored run).
    def _old_paired_ate_n(est, tru):
        n, se = 0, 0.0
        ts = tru["stamp"].astype(np.float64)
        for row in est:
            t = float(row["stamp"])
            if t < ts[0] or t > ts[-1]:
                continue
            k = max(0, min(int(np.searchsorted(ts, t, side="right") - 1), ts.size - 2))
            e = np.array([row["p_x"] - tru[k]["p_x"], row["p_y"] - tru[k]["p_y"],
                          row["p_z"] - tru[k]["p_z"]], dtype=np.float64)
            n += 1
            se += float(e @ e)
        return n, (math.sqrt(se / n) if n else float("nan"))

    n_old, ate_old = _old_paired_ate_n(est_c, inter)
    # (a t == 0.0 estimate can still slip through the degenerate ts[0]==ts[-1]==0
    # guard; everything else is rejected, which is what NaNs the metrics)
    check("12k old pairing rejected ~everything (NaN metrics)",
          n_old < 0.01 * ref["n"], f"n={n_old} of {ref['n']}")
    inter2 = inter[:-1]  # now ends on a real stamp, like cf_2 live
    n_old2, ate_old2 = _old_paired_ate_n(est_c, inter2)
    check("12l old pairing inflated ATE", n_old2 > 0 and ate_old2 > 5.0 * ref["ate_rmse_m"],
          f"n={n_old2} ate_old={ate_old2:.3f} vs {ref['ate_rmse_m']:.3f}")
    check("12m fixed pairing unaffected by trailing row",
          abs(paired_errors(est_c, inter2)["ate_rmse_m"] - ref["ate_rmse_m"]) < 1e-9)

    # centroid/shape goes through the same guard
    cs_ref = centroid_vs_shape(estimates, truth)
    cs_bad = centroid_vs_shape(estimates, {i: np.concatenate([truth[i], np.zeros(1, dtype=TRUTH_DTYPE)])
                                           for i in truth})
    check("12n centroid_vs_shape survives a trailing zero stamp",
          len(cs_bad["t"]) == len(cs_ref["t"]) and len(cs_ref["t"]) > 0,
          f"{len(cs_bad['t'])} vs {len(cs_ref['t'])}")

    # bundle written to disk is sanitized
    with tempfile.TemporaryDirectory() as td3:
        td3 = Path(td3)
        write_eval_bundle(td3, {0: est_c}, {0: inter})
        got = np.load(td3 / "truth.npz")["cf_0"]
        check("12o written bundle truth is sorted and real-stamped",
              got.size == san.size and bool(np.all(np.diff(got["stamp"]) > 0)))

    # ---- 13: scoring window. The line-spawn -> layout teleport before the
    # scripted path must not enter the score, and the window must survive an
    # offline re-evaluation.
    tt = np.linspace(40.0, 60.0, 1001)
    tru_w = np.zeros(tt.size, dtype=TRUTH_DTYPE)
    est_w = np.zeros(tt.size, dtype=STATE_DTYPE)
    for k, tk in enumerate(tt):
        p = np.array([1.8, 0.0, 0.015]) if tk < 46.3 else np.array([0.0, 0.0, 0.5])
        tru_w[k]["stamp"] = tk
        tru_w[k]["p_x"], tru_w[k]["p_y"], tru_w[k]["p_z"] = p
        # the estimate lags the teleport, reconverging over 3 s to a 5 cm offset
        lag = 1.8 * max(0.0, 1.0 - (tk - 46.3) / 3.0) if tk >= 46.3 else 0.0
        est_w[k] = _synth_state_row(tk, 0, p + np.array([lag + 0.05, 0.0, 0.0]), 0.0)
    full = evaluate(None, {0: tru_w}, {0: est_w}, out_dir=None, write_png=False)
    win = evaluate(None, {0: tru_w}, {0: est_w}, out_dir=None, write_png=False,
                   score_t0=52.9, score_source="flight_sim_t0")
    ate_full = full["per_drone"]["0"]["ate_rmse_m"]
    ate_win = win["per_drone"]["0"]["ate_rmse_m"]
    check("13 teleport transient inflates full-recording ATE", ate_full > 0.2, f"{ate_full:.3f}")
    check("13a windowed ATE is the steady tracking error", abs(ate_win - 0.05) < 2e-3, f"{ate_win:.4f}")
    check("13b rows before t0 excluded and counted",
          win["score_window"]["n_estimates_outside"].get("0") == int(np.sum(tt < 52.9)),
          str(win["score_window"]))
    check("13c full recording reports no window",
          full["score_window"]["t0"] is None and not full["score_window"]["n_estimates_outside"])
    end = evaluate(None, {0: tru_w}, {0: est_w}, out_dir=None, write_png=False,
                   score_t0=52.9, score_t1=55.0)
    check("13d end bound clips too",
          end["per_drone"]["0"]["n"] == int(np.sum((tt >= 52.9) & (tt <= 55.0))),
          str(end["per_drone"]["0"]["n"]))
    past = evaluate(None, {0: tru_w}, {0: est_w}, out_dir=None, write_png=False, score_t0=70.0)
    check("13e window past the run scores nothing, not a silent full score",
          past["per_drone"]["0"]["n"] == 0)
    with tempfile.TemporaryDirectory() as td4:
        td4 = Path(td4)
        write_score_window(td4, 52.9, None, "flight_sim_t0")
        check("13f score window file roundtrip", load_score_window(td4) == (52.9, None, "flight_sim_t0"))
        check("13g missing score window file -> full recording",
              load_score_window(td4 / "absent") == (None, None, ""))

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def _load_logs(path: Optional[Path]):
    if path is None or not path.exists():
        return None
    if path.is_file():
        return load_run(path)
    if list(path.glob("cf_*.npz")):
        return load_run(path)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--eval-dir", default="", help="dir with cf_*.npz, truth.npz, estimates.npz")
    parser.add_argument("--logs", default="", help="directory of cf_*.npz")
    parser.add_argument("--truth", default="", help="truth.npz from swarm_loc_gate --eval-dir")
    parser.add_argument("--estimates", default="", help="estimates.npz (STATE_DTYPE, has cov for NEES)")
    parser.add_argument("--out", default="", help="write metrics_6_1.json and plots/")
    parser.add_argument("--show", action="store_true", help="open matplotlib windows (needs a display)")
    parser.add_argument("--no-png", action="store_true", help="skip PNG files")
    parser.add_argument(
        "--score-start",
        type=float,
        default=None,
        help=f"sim time to score from (default: {SCORE_WINDOW_FILE} in --eval-dir, else full recording)",
    )
    parser.add_argument(
        "--score-end", type=float, default=None, help="sim time to score until (default: end of recording)"
    )
    args = parser.parse_args()
    if args.selftest:
        sys.exit(run_selftest())

    eval_dir = Path(args.eval_dir) if args.eval_dir else None
    logs_path = Path(args.logs) if args.logs else eval_dir
    truth_path = Path(args.truth) if args.truth else (eval_dir / "truth.npz" if eval_dir else None)
    est_path = Path(args.estimates) if args.estimates else (eval_dir / "estimates.npz" if eval_dir else None)
    out_dir = Path(args.out) if args.out else eval_dir
    if logs_path is None and (truth_path is None or not Path(truth_path).is_file()):
        parser.print_help()
        sys.exit(2)

    logs = _load_logs(logs_path)
    truth = load_structured_npz(Path(truth_path), TRUTH_DTYPE) if truth_path else {}
    estimates = load_structured_npz(Path(est_path)) if est_path else {}
    if not estimates and logs:
        estimates = load_estimates_from_logs(logs)
        print("[eval_6_1] no estimates.npz — NEES skipped (meas_log has no covariance)", flush=True)
    if not truth:
        print("[eval_6_1] no truth.npz — ATE/RPE/NEES skipped (estimator must not subscribe to odom)", flush=True)
    score_t0, score_t1, score_source = load_score_window(eval_dir)
    if args.score_start is not None:
        score_t0, score_source = args.score_start, "--score-start"
    if args.score_end is not None:
        score_t1 = args.score_end
    report = evaluate(
        logs,
        truth,
        estimates,
        out_dir=out_dir,
        write_png=not args.no_png,
        show=args.show,
        score_t0=score_t0,
        score_t1=score_t1,
        score_source=score_source,
    )
    print_report(report)
    if out_dir is not None:
        print(f"[eval_6_1] wrote {out_dir}")
    sys.exit(0)


if __name__ == "__main__":
    main()
