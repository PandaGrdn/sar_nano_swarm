#!/usr/bin/env python3
"""UTIL (Zhao et al.) TDOA range-error fitter.

UTIL is TDOA-only (d12 = d2 - d1), DWM1000, identification + flight.
It has no AoA. Convert TDOA residual sigma to a TWR-like pairwise range
sigma with 1/sqrt(2) (two independent links in the difference).

Identification CSV columns (paper Table 2 / los_visual.py):
  tdoa12, tdoa21, ... plus a sibling .txt with anchor/tag poses.

Flight CSV columns (extract_tdoa / extract_gt):
  t_tdoa, idA, idB, tdoa_meas, t_pose, pose_x/y/z, pose_q*.
Anchors: .npz with an_pos (N,3), as in visual_tdoa3_csv.py.
"""
from __future__ import annotations

import math
import os
import re
from glob import glob
from typing import Dict, List, Optional, Tuple

import numpy as np

# Crazyflie body → UWB antenna (UTIL visual_tdoa3_csv.py)
T_UV = np.array([-0.01245, 0.00127, 0.0908], dtype=np.float64)
TWR_FROM_TDOA = 1.0 / math.sqrt(2.0)
MATERIALS = ("cardboard", "metal", "wood", "plastic", "foam")
PAPER_LOS_POSITIONING_RMSE_M = 0.10  # ESKF/batch obstacle-free, Table 4 ~10 cm
# Hard caps so spiked flight residuals cannot land in the sim.
MAX_TWR_RANGE_SIGMA_M = 0.20
MAX_FLIGHT_TDOA_SIGMA_M = 0.35
DEFAULT_FLIGHT_INFLATION = 1.25
MAX_FLIGHT_INFLATION = 2.0
MAX_NLOS_SIGMA_MULT = 4.0
MAX_NLOS_BIAS_M = 0.50
MAX_NLOS_BIAS_SIGMA_M = 0.40
MAX_NLOS_OUTLIER = 0.40


def _delete_nan(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=np.float64).reshape(-1)
    return a[np.isfinite(a)]


def _residual_summary(e: np.ndarray) -> dict:
    """Bias/sigma that ignore TDOA spikes (flight raw std was ~4 m; p95 ~0.3 m)."""
    e = _delete_nan(e)
    if e.size == 0:
        return {
            "bias_m": float("nan"),
            "sigma_m": float("nan"),
            "sigma_raw_m": float("nan"),
            "sigma_mad_m": float("nan"),
            "mae_m": float("nan"),
            "n": 0,
            "n_inlier": 0,
            "p95_abs_m": float("nan"),
        }
    med = float(np.median(e))
    mad = float(np.median(np.abs(e - med)))
    sigma_mad = 1.4826 * mad
    sigma_raw = float(np.std(e))
    thr = max(1.0, 5.0 * sigma_mad) if sigma_mad > 1e-9 else 1.0
    inliers = e[np.abs(e - med) <= thr]
    sigma = float(np.std(inliers)) if inliers.size >= 20 else sigma_mad
    return {
        "bias_m": float(np.mean(inliers)) if inliers.size else med,
        "sigma_m": float(sigma),
        "sigma_raw_m": sigma_raw,
        "sigma_mad_m": float(sigma_mad),
        "mae_m": float(np.mean(np.abs(e))),
        "n": int(e.size),
        "n_inlier": int(inliers.size),
        "p95_abs_m": float(np.percentile(np.abs(e), 95)),
    }


def parse_pose_txt(path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (an1, an2, tag) xyz from a UTIL identification pose txt."""
    pos: List[np.ndarray] = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = [p.strip() for p in line.strip().split(",") if p.strip() != ""]
            if len(parts) == 4:
                pos.append(np.array([float(parts[1]), float(parts[2]), float(parts[3])], dtype=np.float64))
            elif len(parts) == 3:
                pos.append(np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=np.float64))
    if len(pos) < 3:
        raise ValueError(f"need ≥3 xyz rows in {path}, got {len(pos)}")
    return pos[0], pos[1], pos[2]


def classify_ident_folder(path: str) -> str:
    """Official tree is .../identification-dataset/los/distTest/distT1 (los is an ancestor)."""
    blob = path.replace("\\", "/").lower()
    if "nlos" in blob:
        for mat in MATERIALS:
            if mat in blob:
                return f"nlos_{mat}"
        return "nlos"
    if "/los/" in blob or blob.rstrip("/").endswith("/los"):
        return "los"
    if "los" in os.path.basename(path).lower():
        return "los"
    return "unknown"


def _ident_errors(folder: str) -> np.ndarray:
    txts = glob(os.path.join(folder, "*.txt"))
    csvs = glob(os.path.join(folder, "*.csv"))
    if not txts or not csvs:
        return np.zeros(0)
    an1, an2, tag = parse_pose_txt(txts[0])
    gt = float(np.linalg.norm(an2 - tag) - np.linalg.norm(an1 - tag))
    try:
        import pandas as pd
    except ImportError as exc:
        raise RuntimeError("pandas required to read UTIL CSVs") from exc
    df = pd.read_csv(csvs[0])
    col = "tdoa12" if "tdoa12" in df.columns else df.columns[0]
    meas = _delete_nan(np.array(df[col], dtype=np.float64))
    return meas - gt


def fit_identification(root: str) -> dict:
    """Walk identification (or a tree containing it) for LOS/NLOS TDOA residuals."""
    by: Dict[str, List[np.ndarray]] = {}
    for dirpath, _, files in os.walk(root):
        if not any(f.endswith(".csv") for f in files):
            continue
        if not any(f.endswith(".txt") for f in files):
            continue
        kind = classify_ident_folder(dirpath)
        if kind == "unknown":
            continue
        err = _ident_errors(dirpath)
        if err.size == 0:
            continue
        by.setdefault(kind, []).append(err)

    def _pack(kind: str) -> Optional[dict]:
        chunks = by.get(kind)
        if not chunks:
            return None
        return _residual_summary(np.concatenate(chunks))

    los = _pack("los")
    nlos_kinds = {k: _pack(k) for k in by if k.startswith("nlos")}
    nlos_all = None
    outlier = float("nan")
    nlos_chunks = [np.concatenate(v) for k, v in by.items() if k.startswith("nlos") and v]
    if nlos_chunks:
        e = np.concatenate(nlos_chunks)
        nlos_all = _residual_summary(e)
        if los is not None and math.isfinite(float(los["sigma_m"])):
            thr = 3.0 * float(los["sigma_m"])
            outlier = float(np.mean(np.abs(e) > thr))
            nlos_all["outlier_rate"] = outlier
    return {
        "los": los,
        "nlos": nlos_all,
        "nlos_by_material": nlos_kinds,
        "n_folders": sum(len(v) for v in by.values()),
        "nlos_outlier_rate": outlier,
    }


def _quat_xyzw_to_R(qx, qy, qz, qw) -> np.ndarray:
    x, y, z, w = float(qx), float(qy), float(qz), float(qw)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _interp_pose(gt: np.ndarray, t: np.ndarray) -> np.ndarray:
    """gt: (N,8) t,x,y,z,qx,qy,qz,qw. Return (len(t), 3) tag antenna positions."""
    ts = gt[:, 0]
    out = np.zeros((t.size, 3), dtype=np.float64)
    for k, tk in enumerate(t):
        if ts.size == 1:
            i = 0
        else:
            i = int(np.searchsorted(ts, tk, side="right") - 1)
            i = max(0, min(i, ts.size - 1))
        p = gt[i, 1:4]
        R = _quat_xyzw_to_R(gt[i, 4], gt[i, 5], gt[i, 6], gt[i, 7])
        out[k] = p + R @ T_UV
    return out


def _load_anchors(npz_path: str) -> np.ndarray:
    z = np.load(npz_path)
    key = "an_pos" if "an_pos" in z.files else z.files[0]
    return np.asarray(z[key], dtype=np.float64)


def _flight_csv_errors(csv_path: str, anchors: np.ndarray) -> np.ndarray:
    import pandas as pd

    df = pd.read_csv(csv_path)
    need = ("t_tdoa", "idA", "idB", "tdoa_meas", "t_pose", "pose_x", "pose_y", "pose_z")
    if not all(c in df.columns for c in need):
        return np.zeros(0)
    tdoa = df[["t_tdoa", "idA", "idB", "tdoa_meas"]].dropna().to_numpy(dtype=np.float64)
    pose_cols = ["t_pose", "pose_x", "pose_y", "pose_z", "pose_qx", "pose_qy", "pose_qz", "pose_qw"]
    if not all(c in df.columns for c in pose_cols):
        return np.zeros(0)
    gt = df[pose_cols].dropna().to_numpy(dtype=np.float64)
    if tdoa.size == 0 or gt.size == 0:
        return np.zeros(0)
    tag = _interp_pose(gt, tdoa[:, 0])
    ids_a = tdoa[:, 1].astype(int)
    ids_b = tdoa[:, 2].astype(int)
    n_an = int(anchors.shape[0])
    ok = (ids_a >= 0) & (ids_a < n_an) & (ids_b >= 0) & (ids_b < n_an)
    tdoa, tag, ids_a, ids_b = tdoa[ok], tag[ok], ids_a[ok], ids_b[ok]
    pa = anchors[ids_a]
    pb = anchors[ids_b]
    d_i = np.linalg.norm(pa - tag, axis=1)
    d_j = np.linalg.norm(pb - tag, axis=1)
    gt_tdoa = d_j - d_i
    return tdoa[:, 3] - gt_tdoa


def fit_flight(root: str) -> dict:
    """Flight TDOA residuals vs Vicon, split obstacle-free vs cluttered by path name."""
    npzs = glob(os.path.join(root, "**", "*.npz"), recursive=True)
    csvs = glob(os.path.join(root, "**", "*.csv"), recursive=True)
    csvs = [c for c in csvs if _csv_looks_flight(c)]
    if not csvs:
        return {"los": None, "cluttered": None, "n_csv": 0}

    def _nearest_npz(csv_path: str) -> Optional[str]:
        blob = csv_path.replace("\\", "/").lower()
        m = re.search(r"const(\d+)", blob)
        if m:
            needle = f"const{m.group(1)}"
            ranked = [p for p in npzs if needle in os.path.basename(p).lower()]
            if ranked:
                return ranked[0]
        d = os.path.dirname(csv_path)
        for _ in range(6):
            hits = glob(os.path.join(d, "*.npz"))
            if hits:
                return hits[0]
            d = os.path.dirname(d)
        return npzs[0] if npzs else None

    los_e: List[np.ndarray] = []
    clut_e: List[np.ndarray] = []
    for csv_path in csvs:
        npz = _nearest_npz(csv_path)
        if npz is None:
            continue
        try:
            anchors = _load_anchors(npz)
            err = _flight_csv_errors(csv_path, anchors)
        except Exception:
            continue
        if err.size < 20:
            continue
        blob = csv_path.lower()
        cluttered = any(k in blob for k in ("clutter", "nlos", "const4", "const_4", "constellation4", "obstacle"))
        (clut_e if cluttered else los_e).append(err)

    def _summ(chunks: List[np.ndarray]) -> Optional[dict]:
        if not chunks:
            return None
        return _residual_summary(np.concatenate(chunks))

    return {"los": _summ(los_e), "cluttered": _summ(clut_e), "n_csv": len(los_e) + len(clut_e)}


def _csv_looks_flight(path: str) -> bool:
    try:
        with open(path, encoding="utf-8") as f:
            hdr = f.readline().lower()
        return "t_tdoa" in hdr and "tdoa_meas" in hdr
    except OSError:
        return False


def combine_util(ident: dict, flight: dict, eth_static_range_sigma_m: Optional[float]) -> dict:
    """Map UTIL TDOA stats onto uwb_pdoa.yaml knobs. Never emit a sim-breaking σ."""
    ident_los = ident.get("los") or {}
    ident_nlos = ident.get("nlos") or {}
    flight_los = (flight or {}).get("los") or {}
    notes: List[str] = [
        "UTIL measures TDOA d12=d2-d1, not TWR. range_sigma_m = sigma_tdoa/sqrt(2).",
    ]
    ident_tdoa = float(ident_los["sigma_m"]) if ident_los and ident_los.get("sigma_m") is not None else float("nan")
    flight_tdoa = float(flight_los["sigma_m"]) if flight_los and flight_los.get("sigma_m") is not None else float("nan")
    flight_raw = float(flight_los["sigma_raw_m"]) if flight_los and flight_los.get("sigma_raw_m") is not None else float("nan")
    used_flight = False
    if math.isfinite(flight_tdoa) and flight_tdoa <= MAX_FLIGHT_TDOA_SIGMA_M:
        tdoa_los = flight_tdoa
        used_flight = True
        notes.append(f"LOS sigma from robust flight TDOA ({flight_tdoa:.4f} m).")
    elif math.isfinite(ident_tdoa):
        tdoa_los = ident_tdoa * DEFAULT_FLIGHT_INFLATION
        notes.append(
            f"Flight TDOA sigma discarded ({flight_tdoa:.4f} m robust"
            + (f", {flight_raw:.2f} m raw" if math.isfinite(flight_raw) else "")
            + f"); used ident LOS × {DEFAULT_FLIGHT_INFLATION:.2f}."
        )
    else:
        raise ValueError("UTIL fit found no LOS TDOA residuals")

    range_sigma = float(tdoa_los) * TWR_FROM_TDOA
    if range_sigma > MAX_TWR_RANGE_SIGMA_M:
        notes.append(f"Clipped TWR-proxy sigma {range_sigma:.4f} -> {MAX_TWR_RANGE_SIGMA_M:.4f} m.")
        range_sigma = MAX_TWR_RANGE_SIGMA_M

    ident_los_s = ident_tdoa * TWR_FROM_TDOA if math.isfinite(ident_tdoa) else float("nan")
    flight_s = flight_tdoa * TWR_FROM_TDOA if math.isfinite(flight_tdoa) else float("nan")
    infl = (
        float(flight_s / ident_los_s)
        if math.isfinite(flight_s) and math.isfinite(ident_los_s) and ident_los_s > 1e-9
        else float("nan")
    )
    if math.isfinite(infl) and infl > MAX_FLIGHT_INFLATION and not used_flight:
        infl = float(DEFAULT_FLIGHT_INFLATION)
    eth_infl = (
        float(range_sigma / eth_static_range_sigma_m)
        if eth_static_range_sigma_m and eth_static_range_sigma_m > 1e-9
        else float("nan")
    )
    nlos_sigma_tdoa = float(ident_nlos["sigma_m"]) if ident_nlos and ident_nlos.get("sigma_m") is not None else float("nan")
    nlos_bias = abs(float(ident_nlos["bias_m"])) if ident_nlos and ident_nlos.get("bias_m") is not None else float("nan")
    if ident_nlos and ident_nlos.get("mae_m") is not None:
        nlos_bias = max(nlos_bias if math.isfinite(nlos_bias) else 0.0, float(ident_nlos["mae_m"]))
    los_tdoa_s = ident_tdoa if math.isfinite(ident_tdoa) else float(tdoa_los)
    nlos_mult = (
        float(nlos_sigma_tdoa / los_tdoa_s)
        if math.isfinite(nlos_sigma_tdoa) and los_tdoa_s > 1e-9
        else 3.0
    )
    nlos_mult = float(min(max(nlos_mult, 1.0), MAX_NLOS_SIGMA_MULT))
    outlier = 0.35
    raw_out = ident.get("nlos_outlier_rate")
    if raw_out is not None and math.isfinite(float(raw_out)):
        outlier = float(raw_out)
    elif ident_nlos and ident_nlos.get("outlier_rate") is not None:
        outlier = float(ident_nlos["outlier_rate"])
    outlier = float(min(max(outlier, 0.0), MAX_NLOS_OUTLIER))
    if math.isfinite(nlos_bias):
        nlos_bias = float(min(nlos_bias, MAX_NLOS_BIAS_M))
    nlos_bias_sigma = nlos_sigma_tdoa if math.isfinite(nlos_sigma_tdoa) else 0.25
    nlos_bias_sigma = float(min(nlos_bias_sigma, MAX_NLOS_BIAS_SIGMA_M))
    bias_src = flight_los if used_flight and flight_los.get("bias_m") is not None else ident_los
    range_bias = float(bias_src.get("bias_m", 0.0) or 0.0) * TWR_FROM_TDOA

    return {
        "tdoa_note": " ".join(notes),
        "range_sigma_m": float(range_sigma),
        "range_bias_m": float(range_bias),
        "ident_los_tdoa_sigma_m": ident_tdoa,
        "flight_los_tdoa_sigma_m": flight_tdoa,
        "used_flight_los_sigma": used_flight,
        "flight_inflation_vs_ident": infl,
        "flight_inflation_vs_eth_static": eth_infl,
        "nlos_bias_mean_m": float(nlos_bias) if math.isfinite(nlos_bias) else 0.35,
        "nlos_bias_sigma_m": nlos_bias_sigma,
        "nlos_sigma_mult": nlos_mult,
        "range_nlos_outlier_rate": outlier,
        "nlos_by_material": ident.get("nlos_by_material") or {},
        "paper_los_positioning_rmse_m": PAPER_LOS_POSITIONING_RMSE_M,
        "source": "UTIAS UTIL (Zhao et al. 2024 IJRR), DWM1000 TDOA, identification+flight",
    }


def find_util_root(path: str) -> str:
    if os.path.isdir(os.path.join(path, "identification-dataset")) or os.path.isdir(
        os.path.join(path, "flight-dataset")
    ):
        return path
    nested = os.path.join(path, "dataset")
    if os.path.isdir(os.path.join(nested, "identification-dataset")) or os.path.isdir(
        os.path.join(nested, "flight-dataset")
    ):
        return nested
    for name in ("identification-dataset", "flight-dataset", "csv-data"):
        hits = glob(os.path.join(path, "**", name), recursive=True)
        if hits:
            return os.path.dirname(hits[0])
    return path
