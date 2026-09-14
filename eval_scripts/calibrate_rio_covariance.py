#!/usr/bin/env python3
"""calibrate_rio_covariance.py — audit and re-derive the RIO increment covariance.

`perception/radar_processing/rio_bridge.py` publishes per-radar-scan body-frame
increments [dp(3), dpsi] with a 5x5 covariance whose diagonal is FLOORED by
SIGMA_V_XY_FLOOR_MPS, SIGMA_VZ_MPS (dp var = sigma_v^2 * dt^2) and
SIGMA_DPSI_RAD_PER_SQRT_S (dpsi var = sigma^2 * dt). The swarm EKF consumes the
rows as WHITE process noise. RIO errors are strongly correlated step to step,
so a per-step fit under-predicts the drift accumulated between corrections;
this tool measures that directly and fits the floors at the correction scale.

PROTOCOL (calibrate-then-evaluate; never tune on the scenario you report):
  1. --calibrate RUN...  designated calibration runs. Per-step and windowed error
     statistics vs truth, then a RECOMMENDATION for the three constants using the
     rule below. Refuses (non-zero exit) when the logs lack the advertised
     covariance (old meas_log format), too few valid rows remain, or too few
     windows exist at the correction interval.
  2. --check RUN...  (alias --eval) held-out runs of DIFFERENT scenarios. Reports
     the consistency of a given set of constants (--sigma-* flags, --constants
     REPORT.json, or the recommendation from step 1 in the same invocation) and of
     the as-flown logged covariance. Never recommends.
  A run passed to both modes, or two runs sharing a scenario key
  (scenario.json "key"), is refused (exit 5) unless --allow-same-scenario.

RUN spec: `LOGS_DIR,EVAL_DIR`, or one directory. A single directory holding both
cf_*.npz and truth.npz is used as-is; `.../swarm_loc_logs/X` pairs with
`.../swarm_loc_eval/X` and vice versa.

STATISTICS (per run, per drone; valid rows inside the score window and truth span):
  * truth increment over [stamp-dt, stamp] by linear interpolation of the sanitized
    truth (eval_6_1.sanitize_truth, BUG B4), yaw unwrapped.
  * BODY frame: truth world increment rotated by Rz(truth psi at the interval
    midpoint)^T. Truth roll/pitch are not recorded (TRUTH_DTYPE) and are small in
    flight, so they are taken as 0. Using TRUTH attitude isolates RIO translation
    error from the attitude error of the IMU used inside the bridge. WORLD frame:
    RIO body increment rotated by the same truth yaw minus the truth increment.
  * per-step: bias (m/s, = sum e / sum dt), residual r = e - bias*dt, sigma_v_step =
    sqrt(sum r^2 / sum dt^2), rmse, autocorrelation of r/dt at lags 1..K, valid
    fraction. Heading: bias rad/s, sigma per sqrt(s) = sqrt(sum r^2 / sum dt).
  * windows of W seconds of sim time (non-overlapping, >= 2 rows, >= W/2 covered):
    sigma_eff(W) = sqrt(sum_w (sum r)^2 / sum_w sum dt^2) — the per-step velocity
    sigma that makes the summed-window variance consistent if increments are
    treated as white (bias removed; sigma_eff_raw keeps it). Heading:
    sqrt(sum_w (sum r)^2 / sum_w sum dt).
  * consistency per covariance source (as_flown = logged cov, constants = floor-only
    model diag(sxy^2, sxy^2, sz^2)*dt^2, dpsi sdpsi^2*dt; the model ignores RIO's own
    P above the floor): ratio(W) = sum_w (sum e)^2 / sum_w sum advertised var (raw
    and debiased), and NEES of the RAW error per step and per window: horizontal
    2-DoF (body xy block), vertical 1-DoF, heading 1-DoF, vs chi-square mean/95%.

RECOMMENDATION RULE (calibration runs only): Wc = --correction-interval-s (default
1.0 s), documented as the swarm EKF's expected interval between absolute
corrections — NOT read from the simulator's UWB rate. Then
  SIGMA_V_XY_FLOOR_MPS      = max over (run, drone, body axis x|y) of sigma_eff(Wc)
  SIGMA_VZ_MPS              = max over (run, drone) of body-z sigma_eff(Wc)
  SIGMA_DPSI_RAD_PER_SQRT_S = max over (run, drone) of heading sigma_eff(Wc)
Body axes are used because the floor is isotropic in xy (frame-invariant) while
body axes keep physically body-fixed biases identifiable. Bias is reported, not
folded into sigma; a warning is printed when sigma_eff_raw exceeds sigma_eff by >10%.

Usage:
    python3 eval_scripts/calibrate_rio_covariance.py --selftest
    python3 eval_scripts/calibrate_rio_covariance.py --calibrate RUN_A RUN_B --check RUN_C --out rep.json
    python3 eval_scripts/calibrate_rio_covariance.py --check RUN_C --sigma-v-xy 0.5 --sigma-vz 0.6 --sigma-dpsi-deg 0.5
"""
from __future__ import annotations

import argparse
import contextlib
import datetime
import io
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in ("perception/swarm_loc", "perception/uwb_sim", "eval_scripts"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

from meas_log import RIO_COV_TRIU, RIO_DTYPE, load_run  # noqa: E402
from eval_6_1 import SCORE_WINDOW_FILE, TRUTH_DTYPE, load_score_window, sanitize_truth  # noqa: E402

TAG = "[calibrate_rio_covariance]"
DEFAULT_WINDOWS_S = (0.5, 1.0, 2.0, 5.0)
# Expected interval between absolute (UWB/entrance) corrections in the swarm EKF.
# An estimator-side assumption; the sim's UWB ranging rate must not be read.
DEFAULT_CORRECTION_INTERVAL_S = 1.0
DEFAULT_LAGS = 10
DEFAULT_MIN_VALID_ROWS = 200
DEFAULT_MIN_WINDOWS = 10
CHI2_95 = {1: 3.841458820694124, 2: 5.991464547107979}
CONST_KEYS = ("SIGMA_V_XY_FLOOR_MPS", "SIGMA_VZ_MPS", "SIGMA_DPSI_RAD_PER_SQRT_S")

EXIT_OK = 0
EXIT_USAGE = 2
EXIT_NO_COV = 3
EXIT_TOO_FEW = 4
EXIT_OVERLAP = 5


class Refusal(Exception):
    def __init__(self, code: int, message: str):
        super().__init__(message)
        self.code = int(code)
        self.message = message


# ---------------------------------------------------------------------------
# geometry / truth
# ---------------------------------------------------------------------------

def wrap(a):
    return (np.asarray(a, dtype=np.float64) + np.pi) % (2.0 * np.pi) - np.pi


def world_from_body(psi: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rz(psi) v, row-wise; z untouched (roll/pitch taken as 0)."""
    c, s = np.cos(psi), np.sin(psi)
    return np.column_stack([c * v[:, 0] - s * v[:, 1], s * v[:, 0] + c * v[:, 1], v[:, 2]])


def body_from_world(psi: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rz(psi)^T v, row-wise."""
    c, s = np.cos(psi), np.sin(psi)
    return np.column_stack([c * v[:, 0] + s * v[:, 1], -s * v[:, 0] + c * v[:, 1], v[:, 2]])


def interp_truth(truth: np.ndarray, t: np.ndarray):
    """Positions (n,3), unwrapped yaw (n,), inside-span mask. truth must be sanitized."""
    ts = truth["stamp"].astype(np.float64)
    P = np.column_stack([truth[k].astype(np.float64) for k in ("p_x", "p_y", "p_z")])
    psi_u = np.unwrap(truth["psi"].astype(np.float64))
    inside = (t >= ts[0]) & (t <= ts[-1])
    p = np.column_stack([np.interp(t, ts, P[:, k]) for k in range(3)])
    return p, np.interp(t, ts, psi_u), inside


def build_increments(rio: np.ndarray, truth: np.ndarray, t0=None, t1=None) -> dict:
    rio = np.asarray(rio)
    if rio.size:
        rio = rio[np.argsort(rio["stamp"].astype(np.float64), kind="stable")]
    st = rio["stamp"].astype(np.float64) if rio.size else np.zeros(0)
    dt = rio["dt"].astype(np.float64) if rio.size else np.zeros(0)
    in_win = np.isfinite(st) & np.isfinite(dt)
    if t0 is not None:
        in_win &= (st - dt) >= float(t0)
    if t1 is not None:
        in_win &= st <= float(t1)
    n_total = int(in_win.sum())
    finite = np.ones(st.size, dtype=bool)
    for f in ("dp_x", "dp_y", "dp_z", "dpsi"):
        if rio.size:
            finite &= np.isfinite(rio[f].astype(np.float64))
    valid = in_win & (dt > 0) & finite
    if rio.size:
        valid &= rio["valid"] != 0
    n_valid = int(valid.sum())
    use = valid.copy()
    if truth is not None and truth.size >= 2 and st.size:
        pa, psia, ina = interp_truth(truth, st - dt)
        pb, psib, inb = interp_truth(truth, st)
        use &= ina & inb
    else:
        use[:] = False
        pa = pb = np.zeros((st.size, 3))
        psia = psib = np.zeros(st.size)
    r = rio[use]
    dt_u = dt[use]
    psi_mid = 0.5 * (psia[use] + psib[use])
    dpw_true = pb[use] - pa[use]
    dpb_true = body_from_world(psi_mid, dpw_true) if r.size else np.zeros((0, 3))
    dp_rio = (np.column_stack([r[f].astype(np.float64) for f in ("dp_x", "dp_y", "dp_z")])
              if r.size else np.zeros((0, 3)))
    e_body = dp_rio - dpb_true
    e_world = (world_from_body(psi_mid, dp_rio) - dpw_true) if r.size else np.zeros((0, 3))
    e_psi = wrap(r["dpsi"].astype(np.float64) - (psib[use] - psia[use])) if r.size else np.zeros(0)
    n = int(r.size)
    C = np.full((n, 5, 5), np.nan)
    names = set(r.dtype.names or ()) if r.size else set()
    for k, (i, j) in enumerate(RIO_COV_TRIU):
        if f"cov_{k}" in names:
            C[:, i, j] = C[:, j, i] = r[f"cov_{k}"].astype(np.float64)
    cov_finite = np.all(np.isfinite(C[:, :4, :4].reshape(n, -1)), axis=1) if n else np.zeros(0, bool)
    return {
        "t": st[use], "dt": dt_u, "e_body": e_body, "e_world": e_world, "e_psi": e_psi,
        "C_body": C[:, :3, :3], "var_psi": C[:, 3, 3], "cov_finite": cov_finite,
        "counts": {
            "n_rows_in_window": n_total,
            "n_valid": n_valid,
            "n_used": n,
            "valid_fraction": (n_valid / n_total) if n_total else float("nan"),
            "duration_s": float(dt_u.sum()),
            "mean_dt_s": float(dt_u.mean()) if n else float("nan"),
        },
    }


# ---------------------------------------------------------------------------
# statistics
# ---------------------------------------------------------------------------

def autocorr(x: np.ndarray, lags: int) -> List[float]:
    if x.size < 3:
        return [float("nan")] * lags
    x = x - x.mean()
    den = float(np.dot(x, x))
    out = []
    for k in range(1, lags + 1):
        out.append(float(np.dot(x[:-k], x[k:]) / den) if (k < x.size and den > 0) else float("nan"))
    return out


def axis_step_stats(e: np.ndarray, dt: np.ndarray, lags: int) -> dict:
    n = int(e.size)
    if n < 3:
        nan = float("nan")
        return {"n": n, "bias_mps": nan, "bias_step_m": nan, "rmse_step_m": nan,
                "sigma_step_m": nan, "sigma_v_step_mps": nan, "acf": [nan] * lags}
    b = float(e.sum() / dt.sum())
    r = e - b * dt
    return {
        "n": n,
        "bias_mps": b,
        "bias_step_m": float(e.mean()),
        "rmse_step_m": float(math.sqrt(np.mean(e * e))),
        "sigma_step_m": float(np.std(r)),
        "sigma_v_step_mps": float(math.sqrt(np.sum(r * r) / np.sum(dt * dt))),
        "acf": autocorr(r / dt, lags),
    }


def heading_step_stats(e: np.ndarray, dt: np.ndarray, lags: int) -> dict:
    n = int(e.size)
    if n < 3:
        nan = float("nan")
        return {"n": n, "bias_rad_per_s": nan, "rmse_step_rad": nan,
                "sigma_rad_per_sqrt_s": nan, "acf": [nan] * lags}
    b = float(e.sum() / dt.sum())
    r = e - b * dt
    return {
        "n": n,
        "bias_rad_per_s": b,
        "rmse_step_rad": float(math.sqrt(np.mean(e * e))),
        "sigma_rad_per_sqrt_s": float(math.sqrt(np.sum(r * r) / np.sum(dt))),
        "acf": autocorr(r / np.sqrt(dt), lags),
    }


def window_groups(t: np.ndarray, dt: np.ndarray, W: float) -> Optional[dict]:
    if t.size == 0:
        return None
    ids = np.floor((t - t[0]) / float(W)).astype(np.int64)
    nb = int(ids.max()) + 1
    cover = np.bincount(ids, weights=dt, minlength=nb)
    cnt = np.bincount(ids, minlength=nb)
    good = (cover >= 0.5 * float(W)) & (cnt >= 2)
    return {"ids": ids, "nb": nb, "good": good, "n_windows": int(good.sum()),
            "mean_rows": float(cnt[good].mean()) if good.any() else float("nan")}


def wsum(g: Optional[dict], x: np.ndarray) -> np.ndarray:
    if g is None:
        return np.zeros(0)
    return np.bincount(g["ids"], weights=x, minlength=g["nb"])[g["good"]]


def _ratio(num: np.ndarray, den: np.ndarray) -> float:
    d = float(np.sum(den))
    return float(np.sum(num) / d) if (num.size and d > 0 and math.isfinite(d)) else float("nan")


def axis_window_stats(e, dt, g, bias_mps, sigma_v_step) -> dict:
    n_w = 0 if g is None else g["n_windows"]
    if n_w < 2 or not math.isfinite(bias_mps):
        nan = float("nan")
        return {"n_windows": n_w, "sigma_eff_mps": nan, "sigma_eff_raw_mps": nan, "inflation_vs_step": nan}
    D2 = wsum(g, dt * dt)
    Sr = wsum(g, e - bias_mps * dt)
    Se = wsum(g, e)
    s_eff = math.sqrt(_ratio(Sr * Sr, D2))
    return {
        "n_windows": n_w,
        "mean_rows_per_window": g["mean_rows"],
        "sigma_eff_mps": s_eff,
        "sigma_eff_raw_mps": math.sqrt(_ratio(Se * Se, D2)),
        "inflation_vs_step": s_eff / sigma_v_step if sigma_v_step > 0 else float("nan"),
    }


def heading_window_stats(e, dt, g, bias, sigma_step) -> dict:
    n_w = 0 if g is None else g["n_windows"]
    if n_w < 2 or not math.isfinite(bias):
        nan = float("nan")
        return {"n_windows": n_w, "sigma_eff_rad_per_sqrt_s": nan, "sigma_eff_raw_rad_per_sqrt_s": nan,
                "inflation_vs_step": nan}
    D1 = wsum(g, dt)
    Sr = wsum(g, e - bias * dt)
    Se = wsum(g, e)
    s_eff = math.sqrt(_ratio(Sr * Sr, D1))
    return {
        "n_windows": n_w,
        "sigma_eff_rad_per_sqrt_s": s_eff,
        "sigma_eff_raw_rad_per_sqrt_s": math.sqrt(_ratio(Se * Se, D1)),
        "inflation_vs_step": s_eff / sigma_step if sigma_step > 0 else float("nan"),
    }


def nees2(ex, ey, cxx, cxy, cyy) -> np.ndarray:
    det = cxx * cyy - cxy * cxy
    ok = (det > 0) & (cxx > 0)
    out = np.full(ex.shape, np.nan)
    out[ok] = (cyy[ok] * ex[ok] ** 2 - 2.0 * cxy[ok] * ex[ok] * ey[ok] + cxx[ok] * ey[ok] ** 2) / det[ok]
    return out


def nees1(e, var) -> np.ndarray:
    out = np.full(e.shape, np.nan)
    ok = var > 0
    out[ok] = e[ok] ** 2 / var[ok]
    return out


def nees_summary(vals: np.ndarray, dof: int) -> dict:
    v = vals[np.isfinite(vals)]
    return {
        "dof": dof,
        "n": int(v.size),
        "mean": float(v.mean()) if v.size else float("nan"),
        "median": float(np.median(v)) if v.size else float("nan"),
        "frac_above_chi2_95": float(np.mean(v > CHI2_95[dof])) if v.size else float("nan"),
        "chi2_mean": float(dof),
        "chi2_95": CHI2_95[dof],
        "expected_frac_above": 0.05,
    }


def model_cov(dt: np.ndarray, const: dict) -> Tuple[np.ndarray, np.ndarray]:
    n = dt.size
    C = np.zeros((n, 3, 3))
    C[:, 0, 0] = C[:, 1, 1] = (float(const["SIGMA_V_XY_FLOOR_MPS"]) * dt) ** 2
    C[:, 2, 2] = (float(const["SIGMA_VZ_MPS"]) * dt) ** 2
    vpsi = float(const["SIGMA_DPSI_RAD_PER_SQRT_S"]) ** 2 * np.maximum(dt, 1e-3)
    return C, vpsi


def consistency(inc: dict, groups: dict, C: np.ndarray, vpsi: np.ndarray, step: dict) -> dict:
    e, ep, dt = inc["e_body"], inc["e_psi"], inc["dt"]
    out = {
        "step": {
            "horiz_2dof": nees_summary(nees2(e[:, 0], e[:, 1], C[:, 0, 0], C[:, 0, 1], C[:, 1, 1]), 2),
            "vert_1dof": nees_summary(nees1(e[:, 2], C[:, 2, 2]), 1),
            "psi_1dof": nees_summary(nees1(ep, vpsi), 1),
        },
        "windows": {},
    }
    for key, g in groups.items():
        if g is None or g["n_windows"] < 2:
            out["windows"][key] = {"n_windows": 0 if g is None else g["n_windows"]}
            continue
        S = [wsum(g, e[:, k]) for k in range(3)]
        Sp = wsum(g, ep)
        Cs = {n: wsum(g, C[:, i, j]) for n, (i, j) in
              {"xx": (0, 0), "xy": (0, 1), "yy": (1, 1), "zz": (2, 2)}.items()}
        Vp = wsum(g, vpsi)
        rd = {}
        for k, ax in enumerate("xyz"):
            b = step["body"][ax]["bias_mps"]
            Sr = wsum(g, e[:, k] - b * dt)
            rd[ax] = _ratio(Sr * Sr, Cs[ax + ax])
        Srp = wsum(g, ep - step["psi"]["bias_rad_per_s"] * dt)
        rd["psi"] = _ratio(Srp * Srp, Vp)
        out["windows"][key] = {
            "n_windows": g["n_windows"],
            "ratio_raw": {"x": _ratio(S[0] ** 2, Cs["xx"]), "y": _ratio(S[1] ** 2, Cs["yy"]),
                          "z": _ratio(S[2] ** 2, Cs["zz"]), "psi": _ratio(Sp ** 2, Vp)},
            "ratio_debiased": rd,
            "horiz_2dof": nees_summary(nees2(S[0], S[1], Cs["xx"], Cs["xy"], Cs["yy"]), 2),
            "vert_1dof": nees_summary(nees1(S[2], Cs["zz"]), 1),
            "psi_1dof": nees_summary(nees1(Sp, Vp), 1),
        }
    return out


def wkey(W: float) -> str:
    return f"{float(W):g}"


def analyze_drone(rio, truth, windows: Sequence[float], lags: int = DEFAULT_LAGS, t0=None, t1=None,
                  constants: Optional[dict] = None, rio_has_cov: bool = True) -> dict:
    inc = build_increments(rio, truth, t0, t1)
    rec: dict = dict(inc["counts"])
    n = rec["n_used"]
    rec["has_cov"] = bool(rio_has_cov and n > 0 and bool(np.all(inc["cov_finite"])))
    dt = inc["dt"]
    step: dict = {"body": {}, "world": {}}
    for frame, E in (("body", inc["e_body"]), ("world", inc["e_world"])):
        for k, ax in enumerate("xyz"):
            step[frame][ax] = axis_step_stats(E[:, k], dt, lags)
    step["psi"] = heading_step_stats(inc["e_psi"], dt, lags)
    groups = {wkey(W): window_groups(inc["t"], dt, W) for W in windows}
    win: dict = {"body": {a: {} for a in "xyz"}, "world": {a: {} for a in "xyz"}, "psi": {}}
    for key, g in groups.items():
        for frame, E in (("body", inc["e_body"]), ("world", inc["e_world"])):
            for k, ax in enumerate("xyz"):
                s = step[frame][ax]
                win[frame][ax][key] = axis_window_stats(E[:, k], dt, g, s["bias_mps"], s["sigma_v_step_mps"])
        sp = step["psi"]
        win["psi"][key] = heading_window_stats(inc["e_psi"], dt, g, sp["bias_rad_per_s"],
                                               sp["sigma_rad_per_sqrt_s"])
    rec["step"] = step
    rec["windows"] = win
    cons = {}
    if rec["has_cov"]:
        cons["as_flown"] = consistency(inc, groups, inc["C_body"], inc["var_psi"], step)
    if constants is not None and n > 0:
        Cm, vm = model_cov(dt, constants)
        cons["constants"] = consistency(inc, groups, Cm, vm, step)
    rec["consistency"] = cons
    return rec


# ---------------------------------------------------------------------------
# runs
# ---------------------------------------------------------------------------

def resolve_run(spec: str) -> Tuple[Path, Path]:
    if "," in spec:
        a, b = spec.split(",", 1)
        logs, ev = Path(a.strip()), Path(b.strip())
    else:
        d = Path(spec)
        parts = list(d.parts)
        if (d / "truth.npz").is_file() and any(d.glob("cf_*.npz")):
            logs = ev = d
        elif "swarm_loc_logs" in parts:
            logs = d
            ev = Path(*[("swarm_loc_eval" if p == "swarm_loc_logs" else p) for p in parts])
        elif "swarm_loc_eval" in parts:
            ev = d
            logs = Path(*[("swarm_loc_logs" if p == "swarm_loc_eval" else p) for p in parts])
        else:
            logs = ev = d
    if not logs.is_dir() or not any(logs.glob("cf_*.npz")):
        raise Refusal(EXIT_USAGE, f"run {spec!r}: no cf_*.npz in {logs}")
    if not (ev / "truth.npz").is_file():
        raise Refusal(EXIT_USAGE, f"run {spec!r}: no truth.npz in {ev}")
    return logs.resolve(), ev.resolve()


def load_scenario(eval_dir: Path) -> Tuple[Optional[str], Optional[dict]]:
    f = eval_dir / "scenario.json"
    if not f.is_file():
        return None, None
    try:
        d = json.loads(f.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None, None
    key = d.get("key")
    if not key and d.get("env") and d.get("situation"):
        key = f"{d['env']}/{d['situation']}"
    return (str(key) if key else None), d


def analyze_run(logs_dir: Path, eval_dir: Path, windows, lags, use_score_window: bool,
                constants: Optional[dict]) -> dict:
    t0 = t1 = None
    src = ""
    if use_score_window:
        t0, t1, src = load_score_window(eval_dir)
    key, scen = load_scenario(eval_dir)
    tz = np.load(eval_dir / "truth.npz", allow_pickle=False)
    logs = load_run(logs_dir)
    drones = {}
    for i in sorted(logs):
        rec = logs[i]
        tk = f"cf_{int(i)}"
        truth = sanitize_truth(np.asarray(tz[tk])) if tk in tz.files else None
        d = analyze_drone(rec["rio"], truth, windows, lags, t0, t1, constants,
                          bool(rec.get("rio_has_cov", False)))
        d["truth_rows_sanitized"] = 0 if truth is None else int(truth.size)
        d["log_has_cov_fields"] = bool(rec.get("rio_has_cov", False))
        drones[tk] = d
    return {
        "logs_dir": str(logs_dir),
        "eval_dir": str(eval_dir),
        "scenario": key,
        "scenario_json": scen,
        "score_window": {"t0": t0, "t1": t1, "source": src, "applied": bool(use_score_window),
                         "file_present": (eval_dir / SCORE_WINDOW_FILE).is_file()},
        "drones": drones,
    }


def recommend(runs: List[dict], Wc: float, min_rows: int, min_windows: int) -> dict:
    key = wkey(Wc)
    no_cov, few = [], []
    for run in runs:
        for dk, d in run["drones"].items():
            tag = f"{run['logs_dir']}:{dk}"
            if not d["has_cov"]:
                no_cov.append(tag + (" (old-format log: no cov_* fields)" if not d["log_has_cov_fields"]
                                     else " (non-finite covariance rows)"))
            nw = d["windows"]["body"]["x"][key]["n_windows"]
            if d["n_used"] < min_rows or nw < min_windows:
                few.append(f"{tag} n_used={d['n_used']} (min {min_rows}), windows@{key}s={nw} (min {min_windows})")
    if not runs or not any(r["drones"] for r in runs):
        raise Refusal(EXIT_TOO_FEW, "no drones in the calibration runs")
    if no_cov:
        raise Refusal(EXIT_NO_COV, "logs lack the advertised RIO covariance — re-fly with the current "
                      "swarm_loc_node (meas_log cov_* fields): " + "; ".join(no_cov)
                      + ("" if not few else " | also too few rows: " + "; ".join(few)))
    if few:
        raise Refusal(EXIT_TOO_FEW, "too few valid rows/windows: " + "; ".join(few))

    def pick(cands):
        best = max(cands, key=lambda c: (c[0] if math.isfinite(c[0]) else -1.0))
        return best

    xy, z, psi, warnings = [], [], [], []
    for run in runs:
        for dk, d in run["drones"].items():
            for ax in ("x", "y"):
                w = d["windows"]["body"][ax][key]
                xy.append((w["sigma_eff_mps"], run["logs_dir"], dk, f"body {ax}"))
                if w["sigma_eff_raw_mps"] > 1.1 * w["sigma_eff_mps"]:
                    warnings.append(f"{dk} body {ax}: bias inflates sigma_eff_raw to {w['sigma_eff_raw_mps']:.3f} "
                                    f"(bias {d['step']['body'][ax]['bias_mps']:+.3f} m/s) — not absorbed")
            wz = d["windows"]["body"]["z"][key]
            z.append((wz["sigma_eff_mps"], run["logs_dir"], dk, "body z"))
            wp = d["windows"]["psi"][key]
            psi.append((wp["sigma_eff_rad_per_sqrt_s"], run["logs_dir"], dk, "heading"))
    out = {"window_s": float(Wc), "rule": "max over calibration (run, drone) of sigma_eff(Wc); xy also max over body x|y",
           "warnings": warnings}
    for name, cands in zip(CONST_KEYS, (xy, z, psi)):
        v, run_dir, dk, ax = pick(cands)
        out[name] = {"value": float(v), "run": run_dir, "drone": dk, "axis": ax, "window_s": float(Wc)}
    out["SIGMA_DPSI_RAD_PER_SQRT_S"]["value_deg"] = math.degrees(out["SIGMA_DPSI_RAD_PER_SQRT_S"]["value"])
    return out


# ---------------------------------------------------------------------------
# printing / json
# ---------------------------------------------------------------------------

def _f(x, spec="8.4f") -> str:
    try:
        xf = float(x)
    except (TypeError, ValueError):
        return f"{'-':>{len(format(0.0, spec))}}"
    if not math.isfinite(xf):
        return f"{'nan':>{len(format(0.0, spec))}}"
    return format(xf, spec)


def print_run(label: str, run: dict, windows: Sequence[float]) -> None:
    sw = run["score_window"]
    print(f"{TAG} [{label}] logs={run['logs_dir']}")
    print(f"{TAG} [{label}] eval={run['eval_dir']} scenario={run['scenario']} "
          f"score_window=[{sw['t0']}, {sw['t1']}] applied={sw['applied']} file={sw['file_present']}")
    keys = [wkey(W) for W in windows]
    for dk, d in run["drones"].items():
        print(f"  {dk}: rows_in_window={d['n_rows_in_window']} valid={_f(100 * d['valid_fraction'], '5.1f')}% "
              f"used={d['n_used']} ({_f(d['duration_s'], '.1f')} s, dt~{_f(d['mean_dt_s'], '.4f')}) "
              f"truth_rows={d['truth_rows_sanitized']} logged_cov={'yes' if d['has_cov'] else 'NO'}")
        hdr = "    frame axis  bias[m/s] sigv_step  rmse[m]  acf1   acf5  " + " ".join(f"seff@{k:>4}" for k in keys)
        print(hdr)
        for frame in ("body", "world"):
            for ax in "xyz":
                s = d["step"][frame][ax]
                acf = s["acf"]
                row = (f"    {frame:5s} {ax:4s} {_f(s['bias_mps'], '+9.4f')} {_f(s['sigma_v_step_mps'], '9.4f')} "
                       f"{_f(s['rmse_step_m'], '8.5f')} {_f(acf[0] if acf else None, '5.2f')} "
                       f"{_f(acf[4] if len(acf) > 4 else None, '6.2f')}  ")
                row += " ".join(_f(d["windows"][frame][ax][k]["sigma_eff_mps"], "9.4f") for k in keys)
                print(row)
        p = d["step"]["psi"]
        row = (f"    psi  [rad] bias/s={_f(p['bias_rad_per_s'], '+.5f')} sig/sqrt(s)={_f(p['sigma_rad_per_sqrt_s'], '.5f')} "
               f"acf1={_f(p['acf'][0] if p['acf'] else None, '.2f')}  seff: ")
        row += " ".join(f"{k}s={_f(d['windows']['psi'][k]['sigma_eff_rad_per_sqrt_s'], '.5f')}" for k in keys)
        print(row)
        for src, c in d["consistency"].items():
            st = c["step"]
            print(f"    [{src}] step NEES h2 mean={_f(st['horiz_2dof']['mean'], '.2f')} "
                  f">95%={_f(st['horiz_2dof']['frac_above_chi2_95'], '.3f')} | v1 mean={_f(st['vert_1dof']['mean'], '.2f')} "
                  f">95%={_f(st['vert_1dof']['frac_above_chi2_95'], '.3f')} | psi mean={_f(st['psi_1dof']['mean'], '.2f')} "
                  f"(chi2: 2 / 1, 5% above)")
            for k in keys:
                w = c["windows"].get(k, {})
                if "ratio_raw" not in w:
                    print(f"    [{src}] W={k}s: windows={w.get('n_windows', 0)} (too few)")
                    continue
                rr, rd = w["ratio_raw"], w["ratio_debiased"]
                print(f"    [{src}] W={k:>4}s n={w['n_windows']:4d} ratio raw x/y/z/psi="
                      f"{_f(rr['x'], '.2f')}/{_f(rr['y'], '.2f')}/{_f(rr['z'], '.2f')}/{_f(rr['psi'], '.2f')} "
                      f"debiased={_f(rd['x'], '.2f')}/{_f(rd['y'], '.2f')}/{_f(rd['z'], '.2f')}/{_f(rd['psi'], '.2f')} "
                      f"NEES h2={_f(w['horiz_2dof']['mean'], '.2f')} v1={_f(w['vert_1dof']['mean'], '.2f')} "
                      f"psi={_f(w['psi_1dof']['mean'], '.2f')}")
        if not d["consistency"]:
            print("    consistency: UNAVAILABLE (no logged covariance and no constants given)")


def to_json_safe(x):
    if isinstance(x, dict):
        return {str(k): to_json_safe(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [to_json_safe(v) for v in x]
    if isinstance(x, (np.floating, float)):
        return float(x) if math.isfinite(float(x)) else None
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, np.bool_):
        return bool(x)
    return x


def git_commit() -> Optional[str]:
    try:
        r = subprocess.run(["git", "rev-parse", "HEAD"], cwd=str(_REPO_ROOT), capture_output=True,
                           text=True, timeout=10)
        return r.stdout.strip() or None if r.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def _parse_windows(s: str) -> List[float]:
    return [float(v) for v in str(s).split(",") if v.strip()]


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--calibrate", nargs="+", default=[], metavar="RUN", help="calibration runs")
    parser.add_argument("--check", "--eval", dest="check", nargs="+", default=[], metavar="RUN",
                        help="held-out evaluation runs (never recommends)")
    parser.add_argument("--windows", default=",".join(f"{w:g}" for w in DEFAULT_WINDOWS_S),
                        help="window lengths [s sim time], comma separated")
    parser.add_argument("--correction-interval-s", type=float, default=DEFAULT_CORRECTION_INTERVAL_S,
                        help="swarm EKF expected interval between absolute corrections (recommendation window)")
    parser.add_argument("--lags", type=int, default=DEFAULT_LAGS)
    parser.add_argument("--min-valid-rows", type=int, default=DEFAULT_MIN_VALID_ROWS)
    parser.add_argument("--min-windows", type=int, default=DEFAULT_MIN_WINDOWS)
    parser.add_argument("--no-score-window", action="store_true", help="ignore score_window.json")
    parser.add_argument("--sigma-v-xy", type=float, default=None, help="check: SIGMA_V_XY_FLOOR_MPS")
    parser.add_argument("--sigma-vz", type=float, default=None, help="check: SIGMA_VZ_MPS")
    parser.add_argument("--sigma-dpsi-deg", type=float, default=None, help="check: SIGMA_DPSI in deg/sqrt(s)")
    parser.add_argument("--constants", default="", help="check: JSON report whose recommendation to check")
    parser.add_argument("--allow-same-scenario", action="store_true")
    parser.add_argument("--out", default="", help="JSON report path")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args(argv)
    if args.selftest:
        return run_selftest()
    if not args.calibrate and not args.check:
        parser.print_help()
        return EXIT_USAGE

    windows = sorted(set(_parse_windows(args.windows)) | {float(args.correction_interval_s)})
    report: dict = {
        "tool": "calibrate_rio_covariance",
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "git_commit": git_commit(),
        "argv": list(sys.argv[1:] if argv is None else argv),
        "protocol": "calibrate on --calibrate runs; evaluate constants on held-out --check runs of other scenarios",
        "windows_s": windows,
        "correction_interval_s": float(args.correction_interval_s),
        "lags": int(args.lags),
        "min_valid_rows": int(args.min_valid_rows),
        "min_windows": int(args.min_windows),
        "frames": "body = truth increment rotated by Rz(truth psi at interval midpoint)^T, roll/pitch=0; "
                  "world = RIO increment rotated by the same truth yaw",
    }
    rc = EXIT_OK

    def finish(code: int) -> int:
        if args.out:
            out = Path(args.out)
            out.parent.mkdir(parents=True, exist_ok=True)
            report["exit_code"] = int(code)
            out.write_text(json.dumps(to_json_safe(report), indent=2, allow_nan=False), encoding="utf-8")
            print(f"{TAG} wrote {out}")
        return code

    try:
        cal_pairs = [resolve_run(s) for s in args.calibrate]
        chk_pairs = [resolve_run(s) for s in args.check]
    except Refusal as e:
        print(f"{TAG} ERROR: {e.message}")
        report["error"] = e.message
        return finish(e.code)

    cal_dirs = {p for pair in cal_pairs for p in pair}
    overlap = sorted(str(p) for pair in chk_pairs for p in pair if p in cal_dirs)
    if overlap:
        msg = f"REFUSE: run directories passed to both --calibrate and --check: {overlap}"
        print(f"{TAG} {msg}")
        report["refusal"] = {"code": EXIT_OVERLAP, "message": msg}
        return finish(EXIT_OVERLAP)
    if not args.allow_same_scenario:
        ck = {load_scenario(e)[0] for _, e in cal_pairs} - {None}
        hk = {load_scenario(e)[0] for _, e in chk_pairs} - {None}
        if ck & hk:
            msg = (f"REFUSE: scenario(s) {sorted(ck & hk)} used for both calibration and check "
                   f"(never tune on the scenario you report; --allow-same-scenario to override)")
            print(f"{TAG} {msg}")
            report["refusal"] = {"code": EXIT_OVERLAP, "message": msg}
            return finish(EXIT_OVERLAP)

    cli_consts = [args.sigma_v_xy, args.sigma_vz, args.sigma_dpsi_deg]
    if any(v is not None for v in cli_consts) and not all(v is not None for v in cli_consts):
        print(f"{TAG} ERROR: give all of --sigma-v-xy --sigma-vz --sigma-dpsi-deg, or none")
        return finish(EXIT_USAGE)

    rec = None
    if cal_pairs:
        print(f"{TAG} === CALIBRATE ({len(cal_pairs)} run(s)) windows={windows} Wc={args.correction_interval_s:g}s ===")
        runs = [analyze_run(l, e, windows, args.lags, not args.no_score_window, None) for l, e in cal_pairs]
        for r in runs:
            print_run("calibrate", r, windows)
        report["calibrate"] = {"runs": runs}
        try:
            rec = recommend(runs, args.correction_interval_s, args.min_valid_rows, args.min_windows)
            report["recommendation"] = rec
            print(f"{TAG} RECOMMENDATION (rule: {rec['rule']}; W = {rec['window_s']:g} s = --correction-interval-s)")
            print(f"{TAG}   calibration runs: {[r['logs_dir'] for r in runs]} scenarios={[r['scenario'] for r in runs]}")
            for k in CONST_KEYS:
                v = rec[k]
                extra = f" ({v['value_deg']:.4f} deg/sqrt(s))" if "value_deg" in v else ""
                print(f"{TAG}   {k} = {v['value']:.4f}{extra}  <- {v['drone']} {v['axis']} run={v['run']}")
            for w in rec["warnings"]:
                print(f"{TAG}   WARNING {w}")
        except Refusal as e:
            print(f"{TAG} REFUSE to recommend (exit {e.code}): {e.message}")
            report["refusal"] = {"code": e.code, "message": e.message}
            rc = e.code

    if chk_pairs:
        consts = None
        source = None
        if all(v is not None for v in cli_consts):
            consts = {"SIGMA_V_XY_FLOOR_MPS": args.sigma_v_xy, "SIGMA_VZ_MPS": args.sigma_vz,
                      "SIGMA_DPSI_RAD_PER_SQRT_S": math.radians(args.sigma_dpsi_deg)}
            source = "cli"
        elif args.constants:
            try:
                j = json.loads(Path(args.constants).read_text(encoding="utf-8"))
                consts = {k: float(j["recommendation"][k]["value"]) for k in CONST_KEYS}
                source = f"report:{args.constants}"
            except (OSError, ValueError, KeyError, TypeError) as e:
                print(f"{TAG} ERROR: cannot read recommendation from {args.constants}: {e}")
                return finish(EXIT_USAGE)
        elif rec is not None:
            consts = {k: rec[k]["value"] for k in CONST_KEYS}
            source = "this invocation's calibration"
        print(f"{TAG} === CHECK ({len(chk_pairs)} held-out run(s)) constants={consts} source={source} ===")
        runs = [analyze_run(l, e, windows, args.lags, not args.no_score_window, consts) for l, e in chk_pairs]
        for r in runs:
            print_run("check", r, windows)
        report["check"] = {"runs": runs, "constants": consts, "constants_source": source,
                           "recommendation": None}
        print(f"{TAG} check mode: no recommendation (held-out runs are never used for tuning)")
        any_cov = any(d["has_cov"] for r in runs for d in r["drones"].values())
        if consts is None and not any_cov:
            msg = ("nothing to audit: held-out logs lack the advertised covariance (old meas_log format) "
                   "and no constants were given; error statistics above are still valid")
            print(f"{TAG} REFUSE (exit {EXIT_NO_COV}): {msg}")
            report["check"]["refusal"] = {"code": EXIT_NO_COV, "message": msg}
            rc = rc or EXIT_NO_COV
    return finish(rc)


# ---------------------------------------------------------------------------
# selftest
# ---------------------------------------------------------------------------

_T_OFF = 10.0
_YAW_RATE = 0.3


def _p_true(t: np.ndarray) -> np.ndarray:
    tau = t - _T_OFF
    return np.column_stack([5.0 * np.sin(0.1 * tau), 5.0 * (1.0 - np.cos(0.1 * tau)),
                            1.0 - 0.25 * np.cos(0.2 * tau)])


def _psi_true(t: np.ndarray) -> np.ndarray:
    return _YAW_RATE * (t - _T_OFF)


def _synth_truth(T: float, rate: float = 100.0, junk: bool = False, seed: int = 0) -> np.ndarray:
    ts = _T_OFF + np.arange(int(round(T * rate)) + 1) / rate
    tr = np.zeros(ts.size, dtype=TRUTH_DTYPE)
    tr["stamp"] = ts
    P = _p_true(ts)
    tr["p_x"], tr["p_y"], tr["p_z"] = P[:, 0], P[:, 1], P[:, 2]
    tr["psi"] = wrap(_psi_true(ts))
    if junk:
        rng = np.random.default_rng(seed)
        bad = np.zeros(ts.size, dtype=TRUTH_DTYPE)
        bad["p_x"] = rng.normal(0, 50, ts.size)
        bad["psi"] = rng.uniform(-3, 3, ts.size)
        tr = np.concatenate([tr, bad])
        tr = tr[rng.permutation(tr.size)]
    return tr


def _synth_rio(T: float, dt: float, noise_v_body: np.ndarray, dpsi_noise: np.ndarray,
               cov_sigmas: Optional[Tuple[float, float, float]]) -> np.ndarray:
    """noise_v_body: (n,3) velocity error [m/s]; dp error = noise*dt. dpsi_noise in rad."""
    n = int(round(T / dt))
    tb = _T_OFF + dt * np.arange(1, n + 1)
    ta = tb - dt
    psi_mid = 0.5 * (_psi_true(ta) + _psi_true(tb))
    dpb = body_from_world(psi_mid, _p_true(tb) - _p_true(ta)) + noise_v_body[:n] * dt
    r = np.zeros(n, dtype=RIO_DTYPE)
    r["stamp"], r["dt"] = tb, dt
    r["dp_x"], r["dp_y"], r["dp_z"] = dpb[:, 0], dpb[:, 1], dpb[:, 2]
    r["dpsi"] = _YAW_RATE * dt + dpsi_noise[:n]
    r["valid"] = 1
    for k in range(15):
        r[f"cov_{k}"] = 0.0 if cov_sigmas is not None else np.nan
    if cov_sigmas is not None:
        sxy, sz, spsi = cov_sigmas
        r["cov_0"] = r["cov_5"] = (sxy * dt) ** 2  # (0,0), (1,1)
        r["cov_9"] = (sz * dt) ** 2               # (2,2)
        r["cov_12"] = spsi ** 2 * dt              # (3,3)
        r["cov_14"] = 1e-8
    return r


def _ar1(n: int, phi: float, sigma: float, rng, dim: int = 3) -> np.ndarray:
    w = rng.normal(0.0, sigma * math.sqrt(1.0 - phi * phi), size=(n, dim))
    x = np.empty((n, dim))
    x[0] = rng.normal(0.0, sigma, size=dim)
    for k in range(1, n):
        x[k] = phi * x[k - 1] + w[k]
    return x


def _write_run(d: Path, key: str, drones: Dict[int, np.ndarray], truth: Dict[int, np.ndarray],
               old_format: bool = False) -> None:
    from meas_log import EST_DTYPE, NIS_DTYPE, UWB_DTYPE
    d.mkdir(parents=True, exist_ok=True)
    for i, rio in drones.items():
        if old_format:
            names = [n for n in RIO_DTYPE.names if not n.startswith("cov_")]
            old = np.zeros(rio.size, dtype=np.dtype([(n, RIO_DTYPE.fields[n][0]) for n in names]))
            for n in names:
                old[n] = rio[n]
            rio = old
        np.savez(d / f"cf_{i}.npz", drone_id=np.int32(i), rio=rio, uwb=np.zeros(0, dtype=UWB_DTYPE),
                 estimate=np.zeros(0, dtype=EST_DTYPE), nis=np.zeros(0, dtype=NIS_DTYPE))
    np.savez(d / "truth.npz", **{f"cf_{i}": tr for i, tr in truth.items()})
    (d / "scenario.json").write_text(json.dumps({"key": key}), encoding="utf-8")


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

    def rel(a, b):
        return abs(float(a) / float(b) - 1.0) if b else float("inf")

    W = [0.5, 1.0, 2.0, 5.0]
    dt = 0.02
    T = 2000.0
    n = int(round(T / dt))
    rng = np.random.default_rng(1)
    truth = sanitize_truth(_synth_truth(T))

    # 0 — rotation helpers
    v = np.array([[1.0, 0.0, 0.5]])
    b = body_from_world(np.array([math.pi / 2]), v)
    check("0 body_from_world yaw 90", np.allclose(b, [[0.0, -1.0, 0.5]], atol=1e-12), str(b))
    check("0b world_from_body inverse", np.allclose(world_from_body(np.array([0.7]), body_from_world(np.array([0.7]), v)), v))

    # 1 — white noise: sigma_v_step and sigma_eff independent of W; consistent cov
    sv, sz, sp = 0.3, 0.2, 0.01
    nz = np.column_stack([rng.normal(0, sv, n), rng.normal(0, sv, n), rng.normal(0, sz, n)])
    npsi = rng.normal(0, sp * math.sqrt(dt), n)
    rw = _synth_rio(T, dt, nz, npsi, (sv, sz, sp))
    d1 = analyze_drone(rw, truth, W)
    s1 = d1["step"]["body"]["x"]
    check("1 white used rows", d1["n_used"] >= n - 2 and d1["has_cov"], str(d1["n_used"]))
    check("1b white sigma_v_step", rel(s1["sigma_v_step_mps"], sv) < 0.02, f"{s1['sigma_v_step_mps']:.4f}")
    seffs = [d1["windows"]["body"]["x"][wkey(w)]["sigma_eff_mps"] for w in W]
    check("1c white sigma_eff flat in W", all(rel(s, sv) < 0.08 for s in seffs), str([round(s, 4) for s in seffs]))
    check("1d white acf1 ~ 0", abs(s1["acf"][0]) < 0.03, f"{s1['acf'][0]:.3f}")
    af = d1["consistency"]["as_flown"]
    check("1e white step NEES h2 ~ 2, v1 ~ 1",
          rel(af["step"]["horiz_2dof"]["mean"], 2.0) < 0.05 and rel(af["step"]["vert_1dof"]["mean"], 1.0) < 0.05
          and abs(af["step"]["horiz_2dof"]["frac_above_chi2_95"] - 0.05) < 0.01,
          f"{af['step']['horiz_2dof']['mean']:.3f} {af['step']['vert_1dof']['mean']:.3f}")
    r5 = af["windows"]["5"]["ratio_raw"]
    check("1f white window ratio ~ 1 at 5 s", all(rel(r5[a], 1.0) < 0.2 for a in "xyz"), str(r5))
    ps = d1["step"]["psi"]
    pe = [d1["windows"]["psi"][wkey(w)]["sigma_eff_rad_per_sqrt_s"] for w in W]
    check("1g heading sigma per sqrt(s) and flat in W",
          rel(ps["sigma_rad_per_sqrt_s"], sp) < 0.03 and all(rel(x, sp) < 0.1 for x in pe),
          f"{ps['sigma_rad_per_sqrt_s']:.5f} {pe}")
    check("1h heading NEES ~ 1", rel(af["step"]["psi_1dof"]["mean"], 1.0) < 0.05)

    # 2 — AR(1) correlated noise
    phi = 0.8
    ar = _ar1(n, phi, sv, np.random.default_rng(2))
    ra = _synth_rio(T, dt, ar, np.zeros(n), (sv, sz, sp))
    d2 = analyze_drone(ra, truth, W)
    s2 = d2["step"]["body"]["x"]
    check("2 AR1 per-step sigma", rel(s2["sigma_v_step_mps"], sv) < 0.05, f"{s2['sigma_v_step_mps']:.4f}")
    check("2b AR1 acf1 = phi", abs(s2["acf"][0] - phi) < 0.02, f"{s2['acf'][0]:.3f}")
    se2 = [d2["windows"]["body"]["x"][wkey(w)]["sigma_eff_mps"] for w in W]
    check("2c AR1 sigma_eff grows with W", all(se2[i + 1] > se2[i] for i in range(len(se2) - 1)), str(se2))

    def closed(nstep):
        return math.sqrt((1 + phi) / (1 - phi) - 2 * phi * (1 - phi ** nstep) / (nstep * (1 - phi) ** 2))

    infl5 = se2[-1] / s2["sigma_v_step_mps"]
    check("2d AR1 inflation@5s matches finite-n closed form",
          rel(infl5, closed(5.0 / dt)) < 0.08, f"{infl5:.3f} vs {closed(5.0 / dt):.3f}")
    check("2e AR1 inflation -> sqrt((1+phi)/(1-phi))",
          rel(infl5, math.sqrt((1 + phi) / (1 - phi))) < 0.1 and rel(se2[0] / s2["sigma_v_step_mps"], closed(0.5 / dt)) < 0.08,
          f"{infl5:.3f} vs {math.sqrt((1 + phi) / (1 - phi)):.3f}")
    check("2f AR1 as-flown cov under-predicts windows",
          d2["consistency"]["as_flown"]["windows"]["5"]["ratio_raw"]["x"] > 5.0)

    # 3 — bias reported as bias
    bias = 0.2
    nb = np.column_stack([bias + rng.normal(0, 0.1, n), rng.normal(0, 0.1, n), rng.normal(0, 0.1, n)])
    d3 = analyze_drone(_synth_rio(T, dt, nb, np.zeros(n), (0.1, 0.1, sp)), truth, W)
    s3 = d3["step"]["body"]["x"]
    w3 = d3["windows"]["body"]["x"]["5"]
    check("3 bias recovered", abs(s3["bias_mps"] - bias) < 0.005, f"{s3['bias_mps']:.4f}")
    check("3b bias not in sigma", rel(s3["sigma_v_step_mps"], 0.1) < 0.03 and rel(w3["sigma_eff_mps"], 0.1) < 0.1,
          f"{s3['sigma_v_step_mps']:.4f} {w3['sigma_eff_mps']:.4f}")
    check("3c raw sigma_eff keeps bias", w3["sigma_eff_raw_mps"] > 1.5 * w3["sigma_eff_mps"])

    # 4 — body/world frames on a yawed trajectory
    nbx = np.column_stack([rng.normal(0, sv, n), np.zeros(n), np.zeros(n)])
    d4 = analyze_drone(_synth_rio(T, dt, nbx, np.zeros(n), (sv, sz, sp)), truth, W)
    sb, swd = d4["step"]["body"], d4["step"]["world"]
    check("4 body y error ~ 0 for body-x noise", sb["y"]["sigma_v_step_mps"] < 1e-3 and sb["z"]["sigma_v_step_mps"] < 1e-3,
          f"{sb['y']['sigma_v_step_mps']:.2e}")
    check("4b world splits body-x noise over x,y",
          rel(swd["x"]["sigma_v_step_mps"], sv / math.sqrt(2)) < 0.05
          and rel(swd["y"]["sigma_v_step_mps"], sv / math.sqrt(2)) < 0.05,
          f"{swd['x']['sigma_v_step_mps']:.4f} {swd['y']['sigma_v_step_mps']:.4f}")
    check("4c body x exact", rel(sb["x"]["sigma_v_step_mps"], sv) < 0.02)

    # 5 — unsorted, zero-stamped truth sanitized
    junk = sanitize_truth(_synth_truth(T, junk=True, seed=5))
    d5 = analyze_drone(rw, junk, W)
    check("5 unsorted zero-stamped truth == clean",
          d5["n_used"] == d1["n_used"]
          and abs(d5["step"]["body"]["x"]["sigma_v_step_mps"] - s1["sigma_v_step_mps"]) < 1e-9,
          f"{d5['n_used']} {d5['step']['body']['x']['sigma_v_step_mps']}")
    raw_junk = _synth_truth(T, junk=True, seed=5)
    check("5b raw junk truth is unsorted with zeros",
          bool(np.any(np.diff(raw_junk["stamp"]) < 0)) and bool(np.any(raw_junk["stamp"] == 0)))

    # 6 — score window and invalid rows
    d6 = analyze_drone(rw, truth, W, t0=_T_OFF + 1000.0, t1=_T_OFF + 1500.0)
    check("6 score window clips rows", abs(d6["n_used"] - 500.0 / dt) <= 2, str(d6["n_used"]))
    rv = rw.copy()
    rv["valid"][::4] = 0
    d6b = analyze_drone(rv, truth, W)
    check("6b valid fraction", abs(d6b["valid_fraction"] - 0.75) < 1e-3 and d6b["n_used"] == int(0.75 * n) + (n % 4 > 0) * 0
          or abs(d6b["valid_fraction"] - 0.75) < 1e-3, f"{d6b['valid_fraction']:.4f}")
    d6c = analyze_drone(rw[:0], truth, W)
    check("6c empty rio no crash", d6c["n_used"] == 0 and not d6c["has_cov"])

    # 7 — pipeline through files and main()
    Tp, dtp = 300.0, 0.02
    npn = int(round(Tp / dtp))
    tr_p = _synth_truth(Tp, junk=True, seed=7)

    def drone(sig, seed, cov=True):
        g = np.random.default_rng(seed)
        nzp = np.column_stack([g.normal(0, sig, npn), g.normal(0, sig, npn), g.normal(0, 0.3, npn)])
        return _synth_rio(Tp, dtp, nzp, g.normal(0, 0.01 * math.sqrt(dtp), npn), (0.5, 0.6, math.radians(0.5)))

    quiet = io.StringIO()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        _write_run(td / "A", "synth/A", {0: drone(0.2, 10), 1: drone(0.4, 11)}, {0: tr_p, 1: tr_p})
        _write_run(td / "B", "synth/B", {0: drone(0.3, 12)}, {0: tr_p})
        _write_run(td / "C", "synth/A", {0: drone(0.3, 13)}, {0: tr_p})
        _write_run(td / "OLD", "synth/old", {0: drone(0.3, 14)}, {0: tr_p}, old_format=True)
        rep = td / "rep.json"
        with contextlib.redirect_stdout(quiet):
            rc = main(["--calibrate", str(td / "A"), "--out", str(rep)])
        j = json.loads(rep.read_text(encoding="utf-8"))
        rxy = j.get("recommendation", {}).get("SIGMA_V_XY_FLOOR_MPS", {})
        check("7 calibrate new logs rc 0", rc == EXIT_OK, str(rc))
        check("7b rule = max over drones/axes at Wc",
              rxy.get("drone") == "cf_1" and rel(rxy.get("value", 0), 0.4) < 0.12 and rxy.get("window_s") == 1.0,
              str(rxy))
        rz = j["recommendation"]["SIGMA_VZ_MPS"]["value"]
        rpsi = j["recommendation"]["SIGMA_DPSI_RAD_PER_SQRT_S"]["value"]
        check("7c z and heading recommendation", rel(rz, 0.3) < 0.12 and rel(rpsi, 0.01) < 0.15, f"{rz} {rpsi}")
        check("7d report provenance",
              "git_commit" in j and j["calibrate"]["runs"][0]["scenario"] == "synth/A"
              and j["calibrate"]["runs"][0]["logs_dir"].endswith("A"))
        rep2 = td / "rep2.json"
        with contextlib.redirect_stdout(quiet):
            rc2 = main(["--calibrate", str(td / "A"), "--check", str(td / "B"), "--out", str(rep2)])
        j2 = json.loads(rep2.read_text(encoding="utf-8"))
        cons_b = j2["check"]["runs"][0]["drones"]["cf_0"]["consistency"]
        check("7e calibrate+check rc 0, check never recommends",
              rc2 == EXIT_OK and j2["check"]["recommendation"] is None and "constants" in cons_b
              and "as_flown" in cons_b, str(rc2))
        with contextlib.redirect_stdout(quiet):
            rc3 = main(["--calibrate", str(td / "A"), "--check", str(td / "A")])
        check("7f same dir calibrate/check refused", rc3 == EXIT_OVERLAP, str(rc3))
        with contextlib.redirect_stdout(quiet):
            rc3b = main(["--calibrate", str(td / "A"), "--eval", str(td / "C")])
        check("7g same scenario key refused", rc3b == EXIT_OVERLAP, str(rc3b))
        rep4 = td / "rep4.json"
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc4 = main(["--calibrate", str(td / "OLD"), "--out", str(rep4)])
        j4 = json.loads(rep4.read_text(encoding="utf-8"))
        check("7h old-format logs refused for recommendation",
              rc4 == EXIT_NO_COV and "recommendation" not in j4 and j4["refusal"]["code"] == EXIT_NO_COV, str(rc4))
        check("7i old-format logs still yield error stats",
              (j4["calibrate"]["runs"][0]["drones"]["cf_0"]["step"]["body"]["z"]["sigma_v_step_mps"] or 0) > 0.25
              and "REFUSE" in buf.getvalue())
        with contextlib.redirect_stdout(quiet):
            rc5 = main(["--calibrate", str(td / "A"), "--min-valid-rows", "100000000"])
        check("7j too few rows refused", rc5 == EXIT_TOO_FEW, str(rc5))
        with contextlib.redirect_stdout(quiet):
            rc6 = main(["--check", str(td / "OLD")])
            rc6b = main(["--check", str(td / "OLD"), "--sigma-v-xy", "0.5", "--sigma-vz", "0.6",
                         "--sigma-dpsi-deg", "0.5"])
            rc6c = main(["--check", str(td / "B"), "--constants", str(rep)])
            rc6d = main(["--check", str(td / "B"), "--sigma-v-xy", "0.5"])
        check("7k check old logs: nothing to audit w/o constants, ok with constants",
              rc6 == EXIT_NO_COV and rc6b == EXIT_OK, f"{rc6} {rc6b}")
        check("7l check with --constants report, partial flags rejected",
              rc6c == EXIT_OK and rc6d == EXIT_USAGE, f"{rc6c} {rc6d}")
        (td / "A" / SCORE_WINDOW_FILE).write_text(json.dumps({"t0": _T_OFF + 100.0, "t1": _T_OFF + 200.0}),
                                                  encoding="utf-8")
        rep7 = td / "rep7.json"
        with contextlib.redirect_stdout(quiet):
            main(["--calibrate", str(td / "A"), "--out", str(rep7)])
            rep8 = td / "rep8.json"
            main(["--calibrate", str(td / "A"), "--no-score-window", "--out", str(rep8)])
        nu7 = json.loads(rep7.read_text(encoding="utf-8"))["calibrate"]["runs"][0]["drones"]["cf_0"]["n_used"]
        nu8 = json.loads(rep8.read_text(encoding="utf-8"))["calibrate"]["runs"][0]["drones"]["cf_0"]["n_used"]
        check("7m score_window.json honored / --no-score-window", abs(nu7 - 100.0 / dtp) <= 2 and nu8 >= npn - 2,
              f"{nu7} {nu8}")

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
