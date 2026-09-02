#!/usr/bin/env python3
"""fit_uwb_noise.py — UWB noise from ETH-PBL (angle) + UTIAS UTIL (range).

  python3 perception/uwb_sim/uwb_noise_model/fit_uwb_noise.py --selftest
  python3 perception/uwb_sim/uwb_noise_model/fit_uwb_noise.py \\
      --dataset <eth-pbl-rotation-logs> --util <util-uwb-dataset-root> --write-config

ETH-PBL UWB_DualAntenna_AoA: azimuth sigma vs angle, static LOS only.
UTIAS UTIL (Zhao et al.): TDOA range-difference in identification LOS/NLOS
and flight. No AoA. See docs/uwb_noise_provenance.md.
"""
from __future__ import annotations

import argparse
import datetime as dt
import glob
import math
import os
import re
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_UWB_SIM = os.path.dirname(_SCRIPT_DIR)
_REPO_ROOT = os.path.dirname(os.path.dirname(_UWB_SIM))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from eth_pbl.serial_parser import parse_log_file  # noqa: E402
from util_range import (  # noqa: E402
    T_UV,
    TWR_FROM_TDOA,
    combine_util,
    find_util_root,
    fit_flight,
    fit_identification,
)

# ETH-PBL MLP (paper).ipynb / DW3220 channel 5 setup
C_LIGHT = 299_792_458.0
CHANNEL_HZ = 6489.6e6
LAMBDA_M = C_LIGHT / CHANNEL_HZ
# Antenna spacing on UWB_AoA_module (paper notebook); must satisfy l/(2d) <= 1 for arcsin
DEFAULT_ETH_ANTENNA_SPACING_M = 0.0231
# PDoA sign: dataset body frame vs. DW3220 STS convention (negate raw phase for AoA)
DEFAULT_PDOA_SIGN = -1.0
TDOA_UNIT_S = 1.0 / (128.0 * 499.2e6)
TDOA_NS_LIMIT = 1e-9 / TDOA_UNIT_S
PAPER_REPORTED_AZ_RMS_DEG = 2.4
PAPER_REPORTED_AZ_FOV_HALF_DEG = 45.0
DIST_MM_MIN_VALID = 1000  # ETH-PBL outlier filter (failed TWR reports ~26 mm)


def _default_dataset_dir() -> str:
    candidates = [
        os.path.join(_SCRIPT_DIR, "dataset", "Raw Measurement Logs", "Raw Measurement Logs"),
        os.path.join(_SCRIPT_DIR, "dataset", "Raw%20Measurement%20Logs", "Raw%20Measurement%20Logs"),
        os.path.join(_SCRIPT_DIR, "dataset", "Raw Measurement Logs"),
    ]
    for path in candidates:
        if os.path.isdir(path) and glob.glob(os.path.join(path, "rotation_*.log.gz")):
            return path
    return candidates[0]


def _wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def _true_angle_deg(rotation: float) -> float:
    """Turntable azimuth in [-180, 180] deg (0 = boresight)."""
    r = float(rotation) % 360.0
    return r if r <= 180.0 else r - 360.0


def _pdoa_unwrap_rad(pdoa_rad: float, rotation_deg: float) -> float:
    """Match ETH-PBL df_add_unwrapped_pdoa (rotation dataset)."""
    p = float(pdoa_rad)
    rot = float(rotation_deg) % 360.0
    if p > 1.0 and 45.0 < rot < 155.0:
        p -= 2.0 * math.pi
    if p < -1.0 and 200.0 < rot < 361.0:
        p += 2.0 * math.pi
    return p


def _pdoa_counts_to_rad(pdoa_counts: int) -> float:
    """PDoA int16 scaled [1:-11] -> phase difference (rad)."""
    return float(pdoa_counts) / 2048.0


def _pdoa_to_angle_deg(
    pdoa_counts: int,
    rotation_deg: float,
    antenna_spacing_m: float,
    pdoa_sign: float = DEFAULT_PDOA_SIGN,
) -> float:
    p = _pdoa_unwrap_rad(_pdoa_counts_to_rad(pdoa_counts), rotation_deg)
    p *= pdoa_sign
    x = p * LAMBDA_M / (2.0 * math.pi * antenna_spacing_m)
    x = float(np.clip(x, -1.0, 1.0))
    return math.degrees(math.asin(x))


def _parse_true_distance_m(path: str) -> float:
    m = re.search(r"rotation_(\d+)cm", os.path.basename(path))
    if not m:
        raise ValueError(f"cannot parse distance from {path}")
    return int(m.group(1)) / 100.0


def _frame_ok(frame, require_cir: bool = True) -> bool:
    if frame.twr_data is None or frame.toa_data is None:
        return False
    if frame.twr_data.dist_mm is None or frame.twr_data.dist_mm <= DIST_MM_MIN_VALID:
        return False
    tdoa = frame.toa_data.tdoa
    if not (-TDOA_NS_LIMIT < tdoa < TDOA_NS_LIMIT):
        return False
    if require_cir and frame.cir_analysis_sts1 is None:
        return False
    return True


def _load_rotation_logs(
    dataset_dir: str,
    antenna_spacing_m: float,
    pdoa_sign: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    paths = sorted(glob.glob(os.path.join(dataset_dir, "rotation_*.log.gz")))
    if not paths:
        raise FileNotFoundError(f"no rotation_*.log.gz under {dataset_dir}")

    true_dists: List[float] = []
    true_angles: List[float] = []
    meas_dists: List[float] = []
    meas_angles: List[float] = []

    for path in paths:
        true_d = _parse_true_distance_m(path)
        print(f"[fit] parsing {os.path.basename(path)} (true distance {true_d:.2f} m) ...")
        frames = parse_log_file(path)
        dist_mm_vals = [
            float(f.twr_data.dist_mm)
            for f in frames
            if _frame_ok(f, require_cir=False) and f.twr_data is not None
        ]
        if not dist_mm_vals:
            print("      0 usable TWR frames")
            continue
        dist_mm_median = float(np.median(dist_mm_vals))

        n = 0
        for frame in frames:
            if not _frame_ok(frame):
                continue
            rot = float(frame.twr_data.rotation % 360.0)
            az_meas = _pdoa_to_angle_deg(
                frame.toa_data.pdoa, rot, antenna_spacing_m, pdoa_sign
            )
            # Raw dist_mm has a large firmware offset (~7130 m) and weak scale;
            # use per-run demeaned residual (mm) + true distance for noise fitting.
            dist_residual_m = (float(frame.twr_data.dist_mm) - dist_mm_median) / 1000.0
            true_dists.append(true_d)
            true_angles.append(_true_angle_deg(rot))
            meas_dists.append(true_d + dist_residual_m)
            meas_angles.append(az_meas)
            n += 1
        print(f"      {n} TWR frames (dist_mm median {dist_mm_median:.0f} mm)")

    return (
        np.asarray(true_dists),
        np.asarray(true_angles),
        np.asarray(meas_dists),
        np.asarray(meas_angles),
    )


def _try_load_h5(
    h5_path: str,
    antenna_spacing_m: float,
    pdoa_sign: float,
):
    import pandas as pd

    store = pd.HDFStore(h5_path, mode="r")
    keys = [k for k in store.keys() if k.endswith("/df")]
    true_dists, true_angles, meas_dists, meas_angles = [], [], [], []
    for key in keys:
        title = key.split("/")[1].replace("cache_", "")
        true_d = int(re.sub(r"\D", "", title)) / 100.0
        df = store.get(key)
        dist_col = df[("dist_mm", "")]
        dist_mm_median = float(np.median(dist_col[dist_col > DIST_MM_MIN_VALID]))
        rot = df[("rotation", "")].to_numpy()
        pdoa = df[("pdoa", "")].to_numpy()
        tdoa = df[("tdoa", "")].to_numpy()
        dist = dist_col.to_numpy()
        for r, p, t, d in zip(rot, pdoa, tdoa, dist):
            if not np.isfinite(d) or d <= DIST_MM_MIN_VALID:
                continue
            if not (-TDOA_NS_LIMIT < t < TDOA_NS_LIMIT):
                continue
            rot_f = float(r % 360.0)
            pdoa_counts = int(round(float(p) * 2048.0))
            true_dists.append(true_d)
            true_angles.append(_true_angle_deg(rot_f))
            meas_dists.append(true_d + (float(d) - dist_mm_median) / 1000.0)
            meas_angles.append(
                _pdoa_to_angle_deg(pdoa_counts, rot_f, antenna_spacing_m, pdoa_sign)
            )
    store.close()
    return (
        np.asarray(true_dists),
        np.asarray(true_angles),
        np.asarray(meas_dists),
        np.asarray(meas_angles),
    )


def fit_range(true_d: np.ndarray, meas_d: np.ndarray) -> dict:
    err = meas_d - true_d
    overall = {"bias_m": float(np.mean(err)), "sigma_m": float(np.std(err))}
    by_dist: Dict[str, dict] = {}
    for d in sorted(set(true_d.tolist())):
        mask = true_d == d
        e = err[mask]
        by_dist[f"{d:.1f}"] = {
            "bias_m": float(np.mean(e)),
            "sigma_m": float(np.std(e)),
            "n": int(mask.sum()),
        }
    sigmas = [v["sigma_m"] for v in by_dist.values()]
    return {
        "overall": overall,
        "by_distance_m": by_dist,
        "range_bias_m": overall["bias_m"],
        "range_sigma_m": float(np.median(sigmas)),
        "range_method": "per_run_demeaned_dist_mm",
    }


def fit_angle(true_ang: np.ndarray, meas_ang: np.ndarray, bin_deg: float = 5.0) -> dict:
    err = np.array([_wrap_deg(m - t) for m, t in zip(meas_ang, true_ang)])
    abs_true = np.abs(true_ang)
    # Direct arcsin AoA is meaningful on the front hemisphere only (|az| <= 90).
    front_mask = abs_true <= 90.0

    boresight_mask = front_mask & (abs_true <= 10.0)
    if boresight_mask.sum() < 50:
        boresight_mask = front_mask & (abs_true <= 15.0)
    paper_fov_mask = front_mask & (abs_true <= PAPER_REPORTED_AZ_FOV_HALF_DEG)

    az_bias_deg = float(np.mean(err[boresight_mask])) if boresight_mask.any() else float(np.mean(err))
    az_sigma_boresight = (
        float(np.std(err[boresight_mask])) if boresight_mask.any() else float(np.std(err))
    )
    paper_sigma = float(np.std(err[paper_fov_mask])) if paper_fov_mask.any() else az_sigma_boresight
    paper_mae = (
        float(np.mean(np.abs(err[paper_fov_mask]))) if paper_fov_mask.any() else float("nan")
    )

    max_abs = float(min(90.0, np.percentile(abs_true[front_mask], 99) if front_mask.any() else 90.0))
    edges = np.arange(0.0, max_abs + bin_deg, bin_deg)
    bin_centers, bin_sigmas, bin_ns = [], [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        mask = front_mask & (abs_true >= lo) & (abs_true < hi)
        if mask.sum() < 20:
            continue
        bin_centers.append(0.5 * (lo + hi))
        bin_sigmas.append(float(np.std(err[mask])))
        bin_ns.append(int(mask.sum()))

    if len(bin_centers) >= 2:
        coef = np.polyfit(bin_centers, bin_sigmas, 1)
        slope_deg_per_deg = float(max(coef[0], 0.0))
        intercept = float(coef[1])
    else:
        slope_deg_per_deg = 0.0
        intercept = az_sigma_boresight

    fov_half = float(PAPER_REPORTED_AZ_FOV_HALF_DEG)
    threshold = max(2.0 * az_sigma_boresight, az_sigma_boresight + 1.0)
    for center, sigma, n in zip(bin_centers, bin_sigmas, bin_ns):
        if center > 90.0:
            break
        if n >= 50 and sigma <= threshold:
            fov_half = max(fov_half, center + bin_deg / 2.0)
        elif center <= 45.0:
            # Stop at first degraded bin inside the paper-validated hemisphere.
            break
    fov_half = float(np.clip(fov_half, PAPER_REPORTED_AZ_FOV_HALF_DEG, 90.0))
    aoa_fov_deg = float(min(180.0, 2.0 * fov_half))

    lookup = {
        f"{c:.0f}": {"sigma_deg": s, "n": n}
        for c, s, n in zip(bin_centers, bin_sigmas, bin_ns)
    }

    return {
        "angle_bias_deg": az_bias_deg,
        "sigma_boresight_deg": max(intercept, az_sigma_boresight),
        "piecewise_linear": {
            "a_deg": max(intercept, az_sigma_boresight),
            "b_deg_per_deg": slope_deg_per_deg,
        },
        "lookup_abs_angle_deg": lookup,
        "aoa_fov_deg": aoa_fov_deg,
        "paper_fov_sigma_deg": paper_sigma,
        "paper_fov_mae_deg": paper_mae,
        "err_mean_deg": float(np.mean(np.abs(err))),
        "err_std_deg": float(np.std(err)),
        "err_rms_deg": float(np.sqrt(np.mean(err**2))),
        "n": int(len(err)),
    }


def sanity_table(angle: dict, range_fit: dict) -> str:
    pl = angle["piecewise_linear"]
    lines = [
        "Sanity check vs ETH-PBL paper (Margiani et al. 2023, TIM):",
        f"  Paper reported mean angular accuracy: ~{PAPER_REPORTED_AZ_RMS_DEG} deg (within +/-{PAPER_REPORTED_AZ_FOV_HALF_DEG:.0f} deg)",
        f"  Fitted sigma |theta|<={PAPER_REPORTED_AZ_FOV_HALF_DEG:.0f} deg: {angle['paper_fov_sigma_deg']:.2f} deg",
        f"  Fitted MAE  |theta|<={PAPER_REPORTED_AZ_FOV_HALF_DEG:.0f} deg: {angle['paper_fov_mae_deg']:.2f} deg",
        f"  Fitted sigma at boresight (|theta|<=10 deg): {angle['sigma_boresight_deg']:.2f} deg",
        f"  Fitted angle bias (boresight): {angle['angle_bias_deg']:+.2f} deg",
        f"  Fitted range sigma (demeaned dist_mm): {range_fit['range_sigma_m']:.4f} m",
    ]
    ratio = angle["paper_fov_sigma_deg"] / PAPER_REPORTED_AZ_RMS_DEG
    boresight_ratio = angle["sigma_boresight_deg"] / PAPER_REPORTED_AZ_RMS_DEG
    if 0.5 <= boresight_ratio <= 3.0:
        lines.append(
            f"  -> Boresight sigma / paper: {boresight_ratio:.2f} (reasonable; paper FOV ratio {ratio:.2f})"
        )
    else:
        lines.append(
            f"  -> WARNING: boresight/paper sigma = {boresight_ratio:.2f} -- check antenna_spacing_m / pdoa sign"
        )
    lines.extend(
        [
            "",
            "Range note: raw dist_mm carries a ~7.1 km firmware offset (TWR timestamp truncation);",
            "  range sigmas use per-run demeaned dist_mm residuals, not absolute TWR distance.",
            "",
            "WARNING: Dataset is STATIC / LOS / tripod-mounted -- not flight or NLOS.",
            "  Keep 2x-noise eval discipline until real Crazyflie hardware data exists.",
        ]
    )
    return "\n".join(lines)


def emit_yaml(
    range_fit: dict,
    angle_fit: dict,
    antenna_spacing_m: float,
    pdoa_sign: float,
    fitted_at: str,
    util: Optional[dict] = None,
) -> str:
    pl = angle_fit["piecewise_linear"]
    lines = [
        "# -- noise calibration (auto-generated -- re-run fit_uwb_noise.py) --",
        "noise_calibration:",
        f'  source: "ETH-PBL UWB_DualAntenna_AoA (Margiani et al. 2023), fitted {fitted_at}"',
        '  conditions: "STATIC / LOS / tripod-mounted lab rotation sweep; not flight/NLOS"',
        f"  dataset_channel_hz: {CHANNEL_HZ:.1f}",
        f"  dataset_antenna_spacing_m: {antenna_spacing_m:.4f}",
        f"  dataset_pdoa_sign: {pdoa_sign:.1f}",
        f"  range_method: {range_fit['range_method']}",
        f"  range_bias_m: {range_fit['range_bias_m']:.4f}",
        f"  range_sigma_m: {range_fit['range_sigma_m']:.4f}  # ETH-PBL static TWR residuals; not used as sim LOS if UTIL present",
        "  range_by_distance_m:",
    ]
    for dist, stats in sorted(range_fit["by_distance_m"].items(), key=lambda x: float(x[0])):
        lines.append(
            f"    {dist}: {{bias_m: {stats['bias_m']:.4f}, sigma_m: {stats['sigma_m']:.4f}, n: {stats['n']}}}"
        )
    lines.extend(
        [
            f"  angle_bias_deg: {angle_fit['angle_bias_deg']:.3f}",
            "  angle_sigma_deg:",
            "    model: piecewise_linear",
            "    params:",
            f"      a_deg: {pl['a_deg']:.3f}      # sigma at boresight",
            f"      b_deg_per_deg: {pl['b_deg_per_deg']:.4f}  # sigma grows with |off-boresight angle|",
            f"  aoa_fov_deg: {angle_fit['aoa_fov_deg']:.1f}",
        ]
    )
    if util:
        lines.extend(
            [
                "  util:",
                f'    source: "{util["source"]}"',
                f'    tdoa_note: "{util["tdoa_note"]}"',
                f"    range_sigma_m: {util['range_sigma_m']:.4f}",
                f"    range_bias_m: {util['range_bias_m']:.4f}",
                f"    ident_los_tdoa_sigma_m: {util['ident_los_tdoa_sigma_m']:.4f}",
                f"    flight_los_tdoa_sigma_m: {util['flight_los_tdoa_sigma_m']:.4f}",
                f"    flight_inflation_vs_ident: {util['flight_inflation_vs_ident']:.3f}",
                f"    flight_inflation_vs_eth_static: {util['flight_inflation_vs_eth_static']:.3f}",
                f"    nlos_bias_mean_m: {util['nlos_bias_mean_m']:.4f}",
                f"    nlos_bias_sigma_m: {util['nlos_bias_sigma_m']:.4f}",
                f"    nlos_sigma_mult: {util['nlos_sigma_mult']:.3f}",
                f"    range_nlos_outlier_rate: {util['range_nlos_outlier_rate']:.4f}",
            ]
        )
    sim_range = util["range_sigma_m"] if util else range_fit["range_sigma_m"]
    sim_bias = util["range_bias_m"] if util else range_fit["range_bias_m"]
    lines.extend(
        [
            "",
            "# Top-level sim knobs (consumed by uwb_model.py):",
            f"sigma_range_los_m: {sim_range:.4f}",
            f"range_bias_m: {sim_bias:.4f}",
            "angle_error_model: piecewise_linear",
            f"sigma_boresight_deg: {pl['a_deg']:.3f}",
            f"angle_sigma_slope_deg_per_deg: {pl['b_deg_per_deg']:.4f}",
            f"aoa_fov_deg: {angle_fit['aoa_fov_deg']:.1f}",
        ]
    )
    if util:
        lines.extend(
            [
                f"nlos_bias_mean_m: {util['nlos_bias_mean_m']:.4f}",
                f"nlos_bias_sigma_m: {util['nlos_bias_sigma_m']:.4f}",
                f"nlos_sigma_mult: {util['nlos_sigma_mult']:.3f}",
                f"p_dropout_nlos: {util['range_nlos_outlier_rate']:.4f}",
            ]
        )
    nlos_bias = util["nlos_bias_mean_m"] if util else float("nan")
    nlos_out = util["range_nlos_outlier_rate"] if util else float("nan")
    lines.extend(
        [
            "",
            "noise:",
            f"  range_sigma_m: {sim_range:.4f}",
            f"  range_nlos_bias_m: {nlos_bias:.4f}"
            if util
            else "  range_nlos_bias_m: null  # fit UTIL identification for this",
            f"  range_nlos_outlier_rate: {nlos_out:.4f}"
            if util
            else "  range_nlos_outlier_rate: null",
            "  angle_sigma_deg:",
            "    model: piecewise_linear",
            "    params:",
            f"      a_deg: {pl['a_deg']:.3f}",
            f"      b_deg_per_deg: {pl['b_deg_per_deg']:.4f}",
            f"  aoa_fov_deg: {angle_fit['aoa_fov_deg']:.1f}",
            "source:",
            f'  range: "{util["source"] if util else "ETH-PBL UWB_DualAntenna_AoA static TWR (UTIL not fitted)"}"',
            '  angle: "ETH-PBL UWB_DualAntenna_AoA, static/LOS ONLY — not flight-validated"',
        ]
    )
    return "\n".join(lines)


def write_provenance(
    path: str,
    angle_fit: dict,
    range_fit: dict,
    util: Optional[dict],
    fitted_at: str,
) -> None:
    pl = angle_fit["piecewise_linear"]
    lines = [
        "# UWB noise provenance",
        "",
        f"Fitted `{fitted_at}` by `perception/uwb_sim/uwb_noise_model/fit_uwb_noise.py`.",
        "",
        "| Parameter | Value | Dataset | Condition |",
        "|---|---|---|---|",
        f"| `sigma_boresight_deg` | {pl['a_deg']:.3f} | ETH-PBL DualAntenna AoA | static LOS lab rotation |",
        f"| `angle_sigma_slope_deg_per_deg` | {pl['b_deg_per_deg']:.4f} | ETH-PBL | static LOS, binned |θ| |",
        f"| `aoa_fov_deg` | {angle_fit['aoa_fov_deg']:.1f} | ETH-PBL | max reliable front hemisphere (paper ±45°) |",
        f"| ETH static `range_sigma_m` (not sim LOS if UTIL present) | {range_fit['range_sigma_m']:.4f} | ETH-PBL TWR dist_mm demeaned | static LOS |",
    ]
    if util:
        lines.extend(
            [
                f"| `sigma_range_los_m` | {util['range_sigma_m']:.4f} | UTIAS UTIL TDOA /√2 | flight LOS if present else ident LOS |",
                f"| `range_bias_m` | {util['range_bias_m']:.4f} | UTIL | same |",
                f"| `nlos_bias_mean_m` | {util['nlos_bias_mean_m']:.4f} | UTIL identification | NLOS materials |",
                f"| `nlos_bias_sigma_m` | {util['nlos_bias_sigma_m']:.4f} | UTIL identification | NLOS |",
                f"| `nlos_sigma_mult` | {util['nlos_sigma_mult']:.3f} | UTIL | σ_NLOS / σ_LOS (TDOA) |",
                f"| `p_dropout_nlos` | {util['range_nlos_outlier_rate']:.4f} | UTIL | frac \\|e\\| > 3 σ_LOS |",
                f"| flight inflation vs ident | {util['flight_inflation_vs_ident']:.3f} | UTIL | motion vs static TDOA |",
                f"| flight inflation vs ETH static | {util['flight_inflation_vs_eth_static']:.3f} | UTIL vs ETH-PBL | different radios |",
                f"| flight LOS σ used? | {util.get('used_flight_los_sigma')} | UTIL | {util.get('tdoa_note','')} |",
            ]
        )
    else:
        lines.append("| `sigma_range_los_m` | (ETH static; UTIL not fitted) | ETH-PBL | static LOS |")
    lines.extend(
        [
            "",
            "## Sanity vs papers",
            "",
            f"- ETH-PBL (Margiani et al. 2023 TIM): ~{PAPER_REPORTED_AZ_RMS_DEG}° mean angular accuracy within ±{PAPER_REPORTED_AZ_FOV_HALF_DEG:.0f}°. "
            f"This fit: σ(|θ|≤45°)={angle_fit['paper_fov_sigma_deg']:.2f}°, MAE={angle_fit['paper_fov_mae_deg']:.2f}°, "
            f"boresight σ={angle_fit['sigma_boresight_deg']:.2f}°.",
            "- UTIL (Zhao et al. 2024 IJRR): TDOA dataset, DWM1000, identification LOS/NLOS + ~150 min flight. "
            "Paper Table 4 obstacle-free positioning RMSE ~10 cm (ESKF/batch) — that is **localization** RMSE, not raw TDOA σ. "
            "Raw TDOA σ from this fitter is the quantity mapped to `sigma_range_los_m` via 1/√2.",
            "",
            "## Do not use",
            "",
            "- ETH-PBL for flight dynamics, NLOS, or tunnel multipath.",
            "- UTIL for angle/AoA (TDOA-only, no PDoA).",
            "- Either dataset as Crazyflie-nano Phase-11 bench of *this* airframe.",
            "",
            "**Angle noise has no flight validation.** Carry that into any paper limitations section.",
            "",
            "## Re-run",
            "",
            "```",
            "python3 perception/uwb_sim/uwb_noise_model/fit_uwb_noise.py \\",
            "  --dataset <ETH-PBL rotation_*.log.gz dir> \\",
            "  --util <UTIL tree with identification-dataset/ and flight-dataset/> \\",
            "  --write-config",
            "```",
            "",
            "Download UTIL from [utiasdsl.github.io/util-uwb-dataset](https://utiasdsl.github.io/util-uwb-dataset/) "
            "([github.com/learnsyslab/util-uwb-dataset](https://github.com/learnsyslab/util-uwb-dataset)). "
            "Place extracted CSVs under `perception/uwb_sim/uwb_noise_model/dataset/util-uwb-dataset/` (gitignored). "
            "The GitHub repo is parsers only; the CSVs are downloaded from the dataset site.",
            "",
        ]
    )
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def _load_eth(args) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if args.h5 and os.path.isfile(args.h5):
        print(f"[fit] loading HDF5 {args.h5}")
        return _try_load_h5(args.h5, args.antenna_spacing_m, args.pdoa_sign)
    h5_guess = os.path.join(args.dataset, "processed_data_cache_rotation.h5")
    if os.path.isfile(h5_guess):
        print(f"[fit] loading HDF5 {h5_guess}")
        return _try_load_h5(h5_guess, args.antenna_spacing_m, args.pdoa_sign)
    return _load_rotation_logs(args.dataset, args.antenna_spacing_m, args.pdoa_sign)


def _eth_logs_present(args) -> bool:
    if args.h5 and os.path.isfile(args.h5):
        return True
    if os.path.isfile(os.path.join(args.dataset, "processed_data_cache_rotation.h5")):
        return True
    return bool(glob.glob(os.path.join(args.dataset, "rotation_*.log.gz")))


def _cached_eth_from_config() -> Tuple[dict, dict]:
    """Prior ETH-PBL fit already in uwb_pdoa.yaml (angle stays this; logs not required)."""
    range_fit = {
        "overall": {"bias_m": 0.0053, "sigma_m": 0.0873},
        "by_distance_m": {},
        "range_bias_m": 0.0053,
        "range_sigma_m": 0.0873,
        "range_method": "cached_uwb_pdoa.yaml_eth_pbl_static",
    }
    angle_fit = {
        "angle_bias_deg": 0.0,
        "sigma_boresight_deg": 4.795,
        "piecewise_linear": {"a_deg": 4.795, "b_deg_per_deg": 0.1649},
        "lookup_abs_angle_deg": {},
        "aoa_fov_deg": 90.0,
        "paper_fov_sigma_deg": 4.795,
        "paper_fov_mae_deg": 2.4,
        "err_mean_deg": float("nan"),
        "err_std_deg": 4.795,
        "err_rms_deg": 4.795,
        "n": 0,
    }
    return range_fit, angle_fit


def _maybe_util(util_root: str, eth_range_sigma: float, require: bool) -> Optional[dict]:
    if not util_root or not os.path.isdir(util_root):
        if require:
            raise FileNotFoundError(f"UTIL root missing: {util_root}")
        print(f"[fit] UTIL root not found ({util_root}); range knobs stay ETH-PBL static")
        return None
    root = find_util_root(util_root)
    ident = fit_identification(root)
    flight = fit_flight(root)
    if ident.get("los") is None and not (flight or {}).get("los"):
        if require:
            raise RuntimeError(f"no UTIL LOS CSVs under {root}")
        print(f"[fit] no UTIL LOS CSVs under {root}; skipping UTIL")
        return None
    combined = combine_util(ident, flight, eth_range_sigma)
    print("\n--- UTIL TDOA (Zhao et al.) ---")
    print(f"  {combined['tdoa_note']}")
    print(
        f"  ident LOS tdoa sigma={combined['ident_los_tdoa_sigma_m']:.4f} m  "
        f"flight LOS tdoa sigma={combined['flight_los_tdoa_sigma_m']:.4f} m"
    )
    print(
        f"  TWR-proxy range_sigma_m={combined['range_sigma_m']:.4f}  "
        f"inflation vs ident={combined['flight_inflation_vs_ident']:.3f}  "
        f"vs ETH={combined['flight_inflation_vs_eth_static']:.3f}"
    )
    print(
        f"  NLOS bias={combined['nlos_bias_mean_m']:.4f}  "
        f"nlos_sigma_mult={combined['nlos_sigma_mult']:.3f}  "
        f"outlier={combined['range_nlos_outlier_rate']:.3f}"
    )
    print(
        f"  Paper obstacle-free positioning RMSE ~{combined['paper_los_positioning_rmse_m']:.2f} m "
        "(not raw TDOA sigma)"
    )
    return combined


def run_selftest() -> int:
    import tempfile

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
    true_a = rng.uniform(-40.0, 40.0, size=800)
    meas_a = true_a + rng.normal(0.0, 2.5, size=800)
    true_d = np.full(800, 2.0)
    meas_d = true_d + rng.normal(0.0, 0.09, size=800)
    angle = fit_angle(true_a, meas_a, bin_deg=10.0)
    rang = fit_range(true_d, meas_d)
    check("eth angle sigma in ballpark", 1.5 < angle["sigma_boresight_deg"] < 4.0, str(angle["sigma_boresight_deg"]))
    check("eth range sigma ~0.09", abs(rang["range_sigma_m"] - 0.09) < 0.03, str(rang["range_sigma_m"]))

    with tempfile.TemporaryDirectory() as td:
        los_dir = os.path.join(td, "identification-dataset", "los_dT01")
        nlos_dir = os.path.join(td, "identification-dataset", "nlos_metal_01")
        flt_dir = os.path.join(td, "flight-dataset", "constellation1")
        os.makedirs(los_dir)
        os.makedirs(nlos_dir)
        os.makedirs(flt_dir)
        for folder, an1, tag in (
            (los_dir, np.array([0.0, 0.0, 1.0]), np.array([2.0, 0.0, 1.0])),
            (nlos_dir, np.array([0.0, 0.0, 1.0]), np.array([2.0, 0.0, 1.0])),
        ):
            an2 = np.array([0.0, 2.0, 1.0])
            with open(os.path.join(folder, "pose.txt"), "w", encoding="utf-8") as f:
                f.write(f"an1,{an1[0]},{an1[1]},{an1[2]}\n")
                f.write(f"an2,{an2[0]},{an2[1]},{an2[2]}\n")
                f.write(f"tag,{tag[0]},{tag[1]},{tag[2]}\n")
            gt = float(np.linalg.norm(an2 - tag) - np.linalg.norm(an1 - tag))
            if "nlos" in folder:
                err = rng.normal(0.40, 0.20, size=400)
                err[:80] += 1.5
            else:
                err = rng.normal(0.0, 0.08, size=400)
            with open(os.path.join(folder, "data.csv"), "w", encoding="utf-8") as f:
                f.write("tdoa12,tdoa21\n")
                for e in err:
                    f.write(f"{gt + e},{-(gt + e)}\n")
        anchors = np.array([[0.0, 0.0, 1.5], [4.0, 0.0, 1.5]], dtype=np.float64)
        np.savez(os.path.join(flt_dir, "anchors.npz"), an_pos=anchors)
        n = 300
        t = np.linspace(0, 10, n)
        x = 1.0 + 0.05 * t
        pose = np.column_stack([x, np.zeros(n), np.ones(n)])
        tdoa_t = t
        tag_ant = pose + T_UV
        di = np.linalg.norm(anchors[0] - tag_ant, axis=1)
        dj = np.linalg.norm(anchors[1] - tag_ant, axis=1)
        meas = (dj - di) + rng.normal(0.0, 0.12, size=n)
        with open(os.path.join(flt_dir, "flight.csv"), "w", encoding="utf-8") as f:
            f.write("t_tdoa,idA,idB,tdoa_meas,t_pose,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw\n")
            for i in range(n):
                f.write(
                    f"{tdoa_t[i]},0,1,{meas[i]},{t[i]},{pose[i,0]},{pose[i,1]},{pose[i,2]},0,0,0,1\n"
                )
        ident = fit_identification(td)
        flight = fit_flight(td)
        check("ident los present", ident.get("los") is not None and ident["los"]["n"] > 100)
        check("ident nlos present", ident.get("nlos") is not None)
        check("flight los present", flight.get("los") is not None and flight["los"]["n"] > 50, str(flight))
        comb = combine_util(ident, flight, 0.0873)
        check(
            "flight tdoa sigma ~0.12",
            abs(comb["flight_los_tdoa_sigma_m"] - 0.12) < 0.04,
            str(comb["flight_los_tdoa_sigma_m"]),
        )
        check(
            "range_sigma is tdoa/sqrt2",
            abs(comb["range_sigma_m"] - comb["flight_los_tdoa_sigma_m"] * TWR_FROM_TDOA) < 1e-9,
        )
        check("nlos bias positive", comb["nlos_bias_mean_m"] > 0.2, str(comb["nlos_bias_mean_m"]))
        check("nlos outlier > 0", comb["range_nlos_outlier_rate"] > 0.05, str(comb["range_nlos_outlier_rate"]))
        y = emit_yaml(rang, angle, 0.0231, -1.0, "selftest", comb)
        check("yaml has util range", "noise_calibration:" in y and "util:" in y)
        prov = os.path.join(td, "prov.md")
        write_provenance(prov, angle, rang, comb, "selftest")
        prov_txt = open(prov, encoding="utf-8").read()
        check(
            "provenance mentions UTIL and no flight angle",
            "no flight validation" in prov_txt.lower() and "UTIL" in prov_txt,
        )
        spiked = combine_util(
            {"los": {"sigma_m": 0.08, "bias_m": 0.0}, "nlos": {"sigma_m": 0.16, "bias_m": 0.1, "mae_m": 0.1}, "nlos_outlier_rate": 0.1},
            {"los": {"sigma_m": 4.0, "sigma_raw_m": 4.0, "bias_m": 0.0}},
            0.0873,
        )
        check(
            "4m flight sigma does not enter sim knob",
            spiked["range_sigma_m"] <= 0.20 and not spiked["used_flight_los_sigma"],
            str(spiked["range_sigma_m"]),
        )

    print("[selftest] " + ("ALL PASS" if ok else "FAILED") + f" ({n_pass} passed, {n_fail} failed)")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument(
        "--dataset",
        default=_default_dataset_dir(),
        help="Directory with rotation_*.log.gz (or parent containing preprocessed .h5)",
    )
    ap.add_argument("--h5", default="", help="Optional preprocessed HDF5")
    ap.add_argument("--antenna-spacing-m", type=float, default=DEFAULT_ETH_ANTENNA_SPACING_M)
    ap.add_argument("--pdoa-sign", type=float, default=DEFAULT_PDOA_SIGN)
    ap.add_argument("--bin-deg", type=float, default=5.0)
    ap.add_argument(
        "--util",
        default="",
        help="UTIL root (folder that contains identification-dataset/ and/or flight-dataset/, or their parent)",
    )
    ap.add_argument(
        "--require-util",
        action="store_true",
        help="Fail if UTIL CSVs are missing (default: ETH-only range with a warning)",
    )
    ap.add_argument(
        "--output",
        default=os.path.join(_SCRIPT_DIR, "fitted_uwb_noise.yaml"),
    )
    ap.add_argument(
        "--provenance",
        default=os.path.join(_REPO_ROOT, "docs", "uwb_noise_provenance.md"),
    )
    ap.add_argument("--write-config", action="store_true")
    ap.add_argument(
        "--skip-eth",
        action="store_true",
        help="Keep angle/range ETH numbers from uwb_pdoa.yaml (no rotation logs required)",
    )
    args = ap.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    if not args.util:
        for cand in (
            os.path.join(_SCRIPT_DIR, "util-uwb-dataset"),
            os.path.join(_SCRIPT_DIR, "dataset", "util-uwb-dataset"),
        ):
            if os.path.isdir(cand):
                args.util = cand
                break
        else:
            args.util = os.path.join(_SCRIPT_DIR, "util-uwb-dataset")

    eth_ok = (not args.skip_eth) and _eth_logs_present(args)
    if eth_ok:
        true_d, true_a, meas_d, meas_a = _load_eth(args)
        print(f"[fit] ETH-PBL samples: {len(true_d)}")
        range_fit = fit_range(true_d, meas_d)
        angle_fit = fit_angle(true_a, meas_a, bin_deg=args.bin_deg)
    else:
        print("[fit] ETH-PBL logs not loaded — using cached angle/static range from uwb_pdoa.yaml")
        range_fit, angle_fit = _cached_eth_from_config()
    util = _maybe_util(args.util, range_fit["range_sigma_m"], bool(args.require_util))

    fitted_at = dt.date.today().isoformat()
    yaml_text = emit_yaml(
        range_fit, angle_fit, args.antenna_spacing_m, args.pdoa_sign, fitted_at, util
    )

    print("\n--- ETH-PBL range (demeaned dist_mm, static) ---")
    print(f"  bias={range_fit['range_bias_m']:.4f} m  sigma={range_fit['range_sigma_m']:.4f} m")
    print("\n--- ETH-PBL angle ---")
    pl = angle_fit["piecewise_linear"]
    print(f"  sigma(theta) ~ {pl['a_deg']:.2f} + {pl['b_deg_per_deg']:.4f} |theta|  deg")
    print(f"  aoa_fov_deg={angle_fit['aoa_fov_deg']:.1f}")
    print("\n" + sanity_table(angle_fit, range_fit))

    with open(args.output, "w", encoding="utf-8") as f:
        f.write(yaml_text + "\n")
    print(f"\n[fit] wrote {args.output}")
    write_provenance(args.provenance, angle_fit, range_fit, util, fitted_at)
    print(f"[fit] wrote {args.provenance}")

    if args.write_config:
        cfg_path = os.path.join(_REPO_ROOT, "configs", "sensors", "uwb_pdoa.yaml")
        _patch_config(cfg_path, range_fit, angle_fit, fitted_at, args.antenna_spacing_m, util)
        print(f"[fit] patched {cfg_path}")


def _patch_config(cfg_path, range_fit, angle_fit, fitted_at, antenna_spacing_m, util=None):
    with open(cfg_path, encoding="utf-8") as f:
        lines = f.readlines()

    pl = angle_fit["piecewise_linear"]
    sim_range = util["range_sigma_m"] if util else range_fit["range_sigma_m"]
    sim_bias = util["range_bias_m"] if util else range_fit["range_bias_m"]
    replacements = {
        "sigma_range_los_m:": f"sigma_range_los_m: {sim_range:.4f}",
        "angle_error_model:": "angle_error_model: piecewise_linear",
        "sigma_boresight_deg:": f"sigma_boresight_deg: {pl['a_deg']:.3f}",
        "aoa_fov_deg:": f"aoa_fov_deg: {angle_fit['aoa_fov_deg']:.1f}",
        "range_bias_m:": f"range_bias_m: {sim_bias:.4f}",
    }
    if util:
        replacements.update(
            {
                "nlos_bias_mean_m:": f"nlos_bias_mean_m: {util['nlos_bias_mean_m']:.4f}",
                "nlos_bias_sigma_m:": f"nlos_bias_sigma_m: {util['nlos_bias_sigma_m']:.4f}",
                "nlos_sigma_mult:": f"nlos_sigma_mult: {util['nlos_sigma_mult']:.3f}",
                "p_dropout_nlos:": f"p_dropout_nlos: {util['range_nlos_outlier_rate']:.4f}",
            }
        )
    out = []
    have_slope = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("angle_sigma_slope_deg_per_deg:"):
            out.append(f"angle_sigma_slope_deg_per_deg: {pl['b_deg_per_deg']:.4f}\n")
            have_slope = True
            continue
        replaced = False
        for key, val in replacements.items():
            if stripped.startswith(key):
                out.append(val + "\n")
                replaced = True
                break
        if not replaced:
            out.append(line)

    if not have_slope:
        idx = next(i for i, l in enumerate(out) if l.strip().startswith("sigma_boresight_deg:"))
        out.insert(idx + 1, f"angle_sigma_slope_deg_per_deg: {pl['b_deg_per_deg']:.4f}\n")

    with open(cfg_path, "w", encoding="utf-8") as f:
        f.writelines(out)


if __name__ == "__main__":
    main()
