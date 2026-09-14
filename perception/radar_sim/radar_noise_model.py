#!/usr/bin/env python3
"""radar_noise_model.py — TI IWR6843AOP detection-cloud noise model (pure numpy).

Turns the ideal radarays_gz2 ray-cast cloud [x, y, z, intensity, doppler]
(radar_link frame, boresight +x) into a realistic FMCW detection cloud:

  1. FOV / range gate on the IDEAL point (|az|, |el| <= 60 deg, 0.25-20 m)
  2. bearing noise (reve / EKF-RIO model: sigma_k = off + scale*(1-|u_k|))
  3. range quantization (0.07 m lattice)
  4. Doppler quantization (0.13 m/s) + Gaussian residual (total std 0.040 m/s)
  5. Doppler outliers (p = 0.24, Uniform(-v_max, v_max), quantized)
  6. Doppler ambiguity wrap into [-v_max, v_max)

Every value and its citation lives in configs/sensors/radar_noise.yaml.
⚠ AGENTS.md §1 Tier A: simulator-side only — the estimator must never import
this module or read its config.

No rclpy here; the ROS wrapper is radar_noise_node.py.

Usage:
    python perception/radar_sim/radar_noise_model.py --selftest
"""
from __future__ import annotations

import argparse
import math
import sys
from typing import Dict, Optional, Tuple

import numpy as np

SWITCHES = (
    "gating",
    "bearing_noise",
    "range_quantization",
    "doppler_quantization",
    "doppler_residual",
    "outliers",
    "wrap",
)


def doppler_residual_sigma(total_std: float, v_res: float) -> float:
    """sigma_residual = sqrt(total^2 - v_res^2/12). Raises if negative."""
    q_var = v_res * v_res / 12.0
    if total_std * total_std < q_var:
        raise ValueError(
            f"doppler_total_std_mps={total_std} is below the quantization std "
            f"v_res/sqrt(12)={math.sqrt(q_var):.5f} (v_res={v_res}) — no residual "
            f"variance is left; fix configs/sensors/radar_noise.yaml"
        )
    return math.sqrt(total_std * total_std - q_var)


def wrap_symmetric(v: np.ndarray, v_max: float) -> np.ndarray:
    """Wrap into [-v_max, v_max)."""
    return np.mod(v + v_max, 2.0 * v_max) - v_max


def quantize(v: np.ndarray, res: float) -> np.ndarray:
    return res * np.round(v / res)


def bearing_sigma_rad(u: np.ndarray, offset_deg: float, scale_deg: float) -> np.ndarray:
    """Per-component bearing sigma (reve radar_ego_velocity_estimator.cpp)."""
    return math.radians(offset_deg) + math.radians(scale_deg) * (1.0 - np.abs(u))


class RadarNoiseModel:
    def __init__(
        self,
        *,
        fov_az_half_rad: float,
        fov_el_half_rad: float,
        range_min_m: float,
        range_max_m: float,
        offset_deg: float,
        scale_deg: float,
        d_res: float,
        v_res: float,
        doppler_total_std: float,
        p_outlier: float,
        v_max: float,
        enable: Optional[Dict[str, bool]] = None,
        seed: Optional[int] = None,
    ):
        self.fov_az = float(fov_az_half_rad)
        self.fov_el = float(fov_el_half_rad)
        self.r_min = float(range_min_m)
        self.r_max = float(range_max_m)
        self.offset_deg = float(offset_deg)
        self.scale_deg = float(scale_deg)
        self.d_res = float(d_res)
        self.v_res = float(v_res)
        self.doppler_total_std = float(doppler_total_std)
        self.sigma_residual = doppler_residual_sigma(self.doppler_total_std, self.v_res)
        self.p_outlier = float(p_outlier)
        self.v_max = float(v_max)
        en = {k: True for k in SWITCHES}
        if enable:
            unknown = set(enable) - set(SWITCHES)
            if unknown:
                raise ValueError(f"unknown radar-noise switches: {sorted(unknown)}")
            en.update({k: bool(v) for k, v in enable.items()})
        self.enable = en
        self.seed = seed
        self.rng = np.random.default_rng(seed)

    @classmethod
    def from_config(cls, cfg: dict, seed: Optional[int] = None) -> "RadarNoiseModel":
        return cls(
            fov_az_half_rad=math.radians(float(cfg["fov_azimuth_half_deg"])),
            fov_el_half_rad=math.radians(float(cfg["fov_elevation_half_deg"])),
            range_min_m=float(cfg["range_min_m"]),
            range_max_m=float(cfg["range_max_m"]),
            offset_deg=float(cfg["model_noise_offset_deg"]),
            scale_deg=float(cfg["model_noise_scale_deg"]),
            d_res=float(cfg["range_resolution_m"]),
            v_res=float(cfg["velocity_resolution_mps"]),
            doppler_total_std=float(cfg["doppler_total_std_mps"]),
            p_outlier=float(cfg["p_outlier"]),
            v_max=float(cfg["velocity_max_mps"]),
            enable=cfg.get("enable"),
            seed=cfg.get("seed") if seed is None else seed,
        )

    # ── individual stages ───────────────────────────────────────────────────
    def perturb_directions(self, u: np.ndarray, renormalize: bool = True) -> np.ndarray:
        sig = bearing_sigma_rad(u, self.offset_deg, self.scale_deg)
        un = u + self.rng.standard_normal(u.shape) * sig
        if renormalize:
            un = un / np.linalg.norm(un, axis=1, keepdims=True)
        return un

    def apply(self, points: np.ndarray) -> Tuple[np.ndarray, dict]:
        """points: (N,5) [x,y,z,intensity,doppler] -> ((M,5) float64, diagnostics)."""
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 5)
        en = self.enable
        diag = {"n_in": int(pts.shape[0])}

        xyz = pts[:, :3]
        r = np.linalg.norm(xyz, axis=1)
        keep = np.all(np.isfinite(pts), axis=1) & (r > 1e-9)
        diag["n_finite"] = int(keep.sum())

        if en["gating"]:
            with np.errstate(invalid="ignore"):
                az = np.arctan2(xyz[:, 1], xyz[:, 0])
                el = np.arctan2(xyz[:, 2], np.hypot(xyz[:, 0], xyz[:, 1]))
                keep &= (np.abs(az) <= self.fov_az) & (np.abs(el) <= self.fov_el)
                keep &= (r >= self.r_min) & (r <= self.r_max)
        diag["n_after_gate"] = int(keep.sum())
        diag["kept_idx"] = np.nonzero(keep)[0]

        out = pts[keep].copy()
        n = out.shape[0]
        r = r[keep]

        if en["bearing_noise"] or en["range_quantization"]:
            u = out[:, :3] / r[:, None]
            if en["bearing_noise"]:
                u = self.perturb_directions(u)
            r_meas = r
            n_clamped = 0
            if en["range_quantization"]:
                r_meas = quantize(r, self.d_res)
                n_clamped = int(np.sum(r_meas < self.d_res))
                r_meas = np.maximum(r_meas, self.d_res)
            diag["n_range_clamped"] = n_clamped
            out[:, :3] = u * r_meas[:, None]
        else:
            diag["n_range_clamped"] = 0

        dop = out[:, 4]
        if en["doppler_quantization"]:
            dop = quantize(dop, self.v_res)
        if en["doppler_residual"]:
            dop = dop + self.rng.normal(0.0, self.sigma_residual, n)

        n_out = 0
        is_out = np.zeros(n, dtype=bool)
        if en["outliers"]:
            is_out = self.rng.random(n) < self.p_outlier
            n_out = int(is_out.sum())
            vals = self.rng.uniform(-self.v_max, self.v_max, n_out)
            if en["doppler_quantization"]:
                vals = quantize(vals, self.v_res)
            dop = dop.copy()
            dop[is_out] = vals
        diag["n_outliers"] = n_out
        diag["outlier_mask"] = is_out

        if en["wrap"]:
            wrapped = wrap_symmetric(dop, self.v_max)
            diag["n_wrapped"] = int(np.sum(np.abs(wrapped - dop) > 1e-12))
            dop = wrapped
        else:
            diag["n_wrapped"] = 0

        out[:, 4] = dop
        diag["n_out"] = int(n)
        return out, diag


# ── synthetic tunnel scan (plugin ray pattern, straight corridor) ────────────
def plugin_ray_dirs() -> np.ndarray:
    """radarays_gz2 SphericalModel: theta -pi + k*4deg (90), phi +/-20 deg (8 rows).
    rmagine getDirection = [cos(phi)cos(theta), cos(phi)sin(theta), sin(phi)]."""
    theta = -math.pi + np.arange(90) * (2.0 * math.pi / 90.0)
    phi = math.radians(-20.0) + np.arange(8) * (math.radians(40.0) / 7.0)
    P, T = np.meshgrid(phi, theta, indexing="ij")
    return np.stack([np.cos(P) * np.cos(T), np.cos(P) * np.sin(T), np.sin(P)], axis=-1).reshape(-1, 3)


def synthetic_tunnel_scan(v_sensor=(0.3, 0.0, 0.0), half_width=1.5, half_height=1.0,
                          r_max=30.0) -> np.ndarray:
    d = plugin_ray_dirs()
    t = np.full(d.shape[0], np.inf)
    for axis, bound in ((1, half_width), (2, half_height)):
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = np.where(np.abs(d[:, axis]) > 1e-12, bound / np.abs(d[:, axis]), np.inf)
        t = np.minimum(t, tt)
    t = np.where(t <= r_max, t, np.inf)  # miss -> inf (plugin: non-finite)
    xyz = d * t[:, None]
    v = np.asarray(v_sensor, dtype=np.float64)
    with np.errstate(invalid="ignore"):
        dop = -(d @ v)
        inten = 1.0 / t
    return np.column_stack([xyz, inten, dop])


def ls_ego_velocity(pts: np.ndarray, dims: int, robust: bool) -> np.ndarray:
    """Static-scene LS: u . v = -doppler. dims=2 uses azimuth only (as RIO)."""
    xyz = pts[:, :3]
    if dims == 3:
        A = xyz / np.linalg.norm(xyz, axis=1, keepdims=True)
    else:
        th = np.arctan2(xyz[:, 1], xyz[:, 0])
        A = np.column_stack([np.cos(th), np.sin(th)])
    b = -pts[:, 4]
    active = np.ones(len(b), dtype=bool)
    v = np.linalg.lstsq(A, b, rcond=None)[0]
    if robust:
        for _ in range(10):
            res = A @ v - b
            med = np.median(res[active])
            mad = np.median(np.abs(res[active] - med))
            s = 1.4826 * mad if mad > 1e-9 else np.std(res[active]) + 1e-9
            new = np.abs(res - med) <= 3.0 * s
            if new.sum() < dims + 1 or np.array_equal(new, active):
                break
            active = new
            v = np.linalg.lstsq(A[active], b[active], rcond=None)[0]
    return v


def tunnel_report(cfg: dict, seed: int = 0) -> dict:
    v_true = np.array([0.3, 0.0, 0.0])
    scan = synthetic_tunnel_scan(tuple(v_true))
    m = RadarNoiseModel.from_config(cfg, seed=seed)
    noisy, diag = m.apply(scan)
    ideal_gated = scan[diag["kept_idx"]]
    derr = noisy[:, 4] - ideal_gated[:, 4]
    inl = ~diag["outlier_mask"]
    rep = {
        "points_in": diag["n_in"],
        "points_finite": diag["n_finite"],
        "points_out": diag["n_out"],
        "outliers": diag["n_outliers"],
        "doppler_err_std_all": float(np.std(derr)),
        "doppler_err_std_non_outliers": float(np.std(derr[inl])),
    }
    for dims in (2, 3):
        vt = v_true[:dims]
        e_ideal = np.linalg.norm(ls_ego_velocity(ideal_gated, dims, robust=False) - vt)
        e_plain = np.linalg.norm(ls_ego_velocity(noisy, dims, robust=False) - vt)
        e_rob = np.linalg.norm(ls_ego_velocity(noisy, dims, robust=True) - vt)
        rep[f"ls{dims}d_err_ideal"] = float(e_ideal)
        rep[f"ls{dims}d_err_noisy_plain"] = float(e_plain)
        rep[f"ls{dims}d_err_noisy_robust"] = float(e_rob)
    # Monte-Carlo over seeds for the robust fits (one scan is a single draw)
    errs2, errs3 = [], []
    for s in range(200):
        mm = RadarNoiseModel.from_config(cfg, seed=1000 + s)
        nz, _ = mm.apply(scan)
        errs2.append(np.linalg.norm(ls_ego_velocity(nz, 2, True) - v_true[:2]))
        errs3.append(np.linalg.norm(ls_ego_velocity(nz, 3, True) - v_true))
    rep["ls2d_err_noisy_robust_median_200"] = float(np.median(errs2))
    rep["ls3d_err_noisy_robust_median_200"] = float(np.median(errs3))
    return rep


# ── selftest ────────────────────────────────────────────────────────────────
def default_test_cfg() -> dict:
    return {
        "seed": 7,
        "enable": {k: True for k in SWITCHES},
        "fov_azimuth_half_deg": 60.0,
        "fov_elevation_half_deg": 60.0,
        "range_min_m": 0.25,
        "range_max_m": 20.0,
        "model_noise_offset_deg": 2.0,
        "model_noise_scale_deg": 10.0,
        "range_resolution_m": 0.07,
        "velocity_resolution_mps": 0.13,
        "doppler_total_std_mps": 0.040,
        "p_outlier": 0.24,
        "velocity_max_mps": 4.0,
    }


def _cfg_with(base: dict, **switches) -> dict:
    c = dict(base)
    en = {k: False for k in SWITCHES}
    en.update(switches)
    c["enable"] = en
    return c


def run_selftest(cfg: Optional[dict] = None) -> int:
    n_pass = 0
    n_fail = 0

    def check(name: str, cond: bool, detail: str = ""):
        nonlocal n_pass, n_fail
        if cond:
            n_pass += 1
            print(f"[selftest] PASS {name}")
        else:
            n_fail += 1
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))

    base = default_test_cfg()
    if cfg is not None:
        for k in base:
            if k in cfg:
                base[k] = cfg[k]
        check("00 yaml values match literature set",
              all(abs(float(base[k]) - float(default_test_cfg()[k])) < 1e-12
                  for k in base if k not in ("seed", "enable")),
              str({k: base[k] for k in base if k not in ("enable",)}))

    def pt(x, y, z, dop=0.0):
        return [x, y, z, 1.0, dop]

    def sph(r, az_deg, el_deg, dop=0.0):
        a, e = math.radians(az_deg), math.radians(el_deg)
        return pt(r * math.cos(e) * math.cos(a), r * math.cos(e) * math.sin(a), r * math.sin(e), dop)

    gate_only = RadarNoiseModel.from_config(_cfg_with(base, gating=True), seed=1)

    # 1 FOV gate
    rows = [sph(5, 59.9, 0), sph(5, 60.1, 0), sph(5, -59.9, 0), sph(5, -60.1, 0),
            sph(5, 0, 59.9), sph(5, 0, 60.1), sph(5, 0, -59.9), sph(5, 0, -60.1),
            pt(-5, 0, 0), pt(-5, 0.5, 0), sph(5, 180, 0), sph(5, 120, 0)]
    exp = [True, False, True, False, True, False, True, False, False, False, False, False]
    _, d = gate_only.apply(np.array(rows))
    got = [i in set(d["kept_idx"].tolist()) for i in range(len(rows))]
    check("01 FOV gate az/el edges and behind sensor", got == exp, f"got {got}")

    # 2 range gate
    rows = [pt(0.24, 0, 0), pt(0.26, 0, 0), pt(19.99, 0, 0), pt(20.01, 0, 0), pt(0.25, 0, 0), pt(20.0, 0, 0)]
    _, d = gate_only.apply(np.array(rows))
    got = [i in set(d["kept_idx"].tolist()) for i in range(len(rows))]
    check("02 range gate 0.25..20 m", got == [False, True, True, False, True, True], f"got {got}")

    # 3 non-finite / zero range dropped (even with gating off)
    rows = np.array([pt(np.inf, 0, 0), pt(np.nan, 1, 0), pt(1, 0, 0, np.nan),
                     pt(0, 0, 0), pt(1, 0, 0), [1, 0, 0, np.inf, 0.0]])
    none_m = RadarNoiseModel.from_config(_cfg_with(base), seed=1)
    o, d = none_m.apply(rows)
    check("03 non-finite and zero-range dropped", o.shape[0] == 1 and d["n_finite"] == 1, str(d))

    # 4 range lattice (full model)
    rng = np.random.default_rng(3)
    N = 20000
    dirs = rng.normal(size=(N, 3))
    dirs[:, 0] = np.abs(dirs[:, 0]) + 1.0
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    rr = rng.uniform(0.25, 20.0, N)
    vv = rng.uniform(-3.0, 3.0, N)
    cloud = np.column_stack([dirs * rr[:, None], 1.0 / rr, vv])
    full = RadarNoiseModel.from_config(base, seed=11)
    o, d = full.apply(cloud)
    rm_ = np.linalg.norm(o[:, :3], axis=1) / 0.07
    check("04 range on 0.07 m lattice", np.max(np.abs(rm_ - np.round(rm_))) < 1e-9 and d["n_out"] > 0,
          f"max dev {np.max(np.abs(rm_ - np.round(rm_)))}")
    kept = cloud[d["kept_idx"]]
    check("04b range quantization error <= d_res/2",
          np.max(np.abs(np.linalg.norm(o[:, :3], axis=1) - np.linalg.norm(kept[:, :3], axis=1))) <= 0.035 + 1e-9)

    # 5 bearing off -> directions unchanged
    m5 = RadarNoiseModel.from_config(_cfg_with(base, gating=True, range_quantization=True), seed=2)
    o, d = m5.apply(cloud)
    k = cloud[d["kept_idx"]]
    u0 = k[:, :3] / np.linalg.norm(k[:, :3], axis=1, keepdims=True)
    u1 = o[:, :3] / np.linalg.norm(o[:, :3], axis=1, keepdims=True)
    check("05 bearing off -> directions unchanged", np.max(np.abs(u0 - u1)) < 1e-12)

    # 6 bearing statistics
    mb = RadarNoiseModel.from_config(_cfg_with(base, bearing_noise=True), seed=5)
    NB = 200000
    for label, u in (("boresight", np.array([1.0, 0.0, 0.0])),
                     ("off-axis (1,1,1)/sqrt3", np.ones(3) / math.sqrt(3.0))):
        U = np.tile(u, (NB, 1))
        sig_exp = np.degrees(bearing_sigma_rad(u, 2.0, 10.0))
        raw = mb.perturb_directions(U, renormalize=False) - U
        sig_raw = np.degrees(np.std(raw, axis=0))
        ok = np.all(np.abs(sig_raw - sig_exp) / sig_exp < 0.01)
        check(f"06 bearing sigma formula {label}", bool(ok),
              f"raw std deg {np.round(sig_raw, 3)} vs formula {np.round(sig_exp, 3)}")
        nrm = np.degrees(np.std(mb.perturb_directions(U) - U, axis=0))
        print(f"[selftest]   {label}: formula sigma deg {np.round(sig_exp, 3)}, "
              f"sampled pre-renorm {np.round(sig_raw, 3)}, post-renorm {np.round(nrm, 3)}")
    check("06c boresight lateral sigma 12 deg / boresight 2 deg",
          np.allclose(np.degrees(bearing_sigma_rad(np.array([1.0, 0, 0]), 2.0, 10.0)), [2.0, 12.0, 12.0]))
    # through apply(): angular error at boresight
    Ub = np.tile([10.0, 0.0, 0.0, 0.1, 0.0], (50000, 1))
    ob, _ = mb.apply(Ub)
    lat = np.degrees(np.std(ob[:, 1] / np.linalg.norm(ob[:, :3], axis=1)))
    check("06d apply() boresight lateral std ~11-12 deg after renorm", 10.5 < lat < 12.5, f"{lat:.3f}")

    # 7 Doppler lattice with residual/outliers off
    mq = RadarNoiseModel.from_config(_cfg_with(base, doppler_quantization=True), seed=1)
    o, d = mq.apply(cloud)
    q = o[:, 4] / 0.13
    check("07 Doppler on 0.13 m/s lattice", np.max(np.abs(q - np.round(q))) < 1e-9)
    check("07b Doppler == round(v/v_res)*v_res",
          np.allclose(o[:, 4], np.round(cloud[:, 4] / 0.13) * 0.13, atol=1e-12))

    # 8 total Doppler std
    sr = doppler_residual_sigma(0.040, 0.13)
    check("08a sigma_residual computed = 0.01385", abs(sr - 0.013853) < 2e-5, f"{sr:.6f}")
    ND = 400000
    vt = rng.uniform(-3.0, 3.0, ND)
    dc = np.column_stack([np.full(ND, 5.0), np.zeros(ND), np.zeros(ND), np.full(ND, 0.2), vt])
    mt = RadarNoiseModel.from_config(_cfg_with(base, doppler_quantization=True, doppler_residual=True), seed=9)
    o, _ = mt.apply(dc)
    tot = float(np.std(o[:, 4] - vt))
    print(f"[selftest]   total Doppler error std = {tot:.5f} m/s (target 0.040); "
          f"quantization-only std = {np.std(np.round(vt / 0.13) * 0.13 - vt):.5f}")
    check("08 total Doppler std ~0.040", abs(tot - 0.040) < 0.001, f"{tot:.5f}")

    # 9 outliers
    mo = RadarNoiseModel.from_config(_cfg_with(base, doppler_quantization=True, outliers=True), seed=13)
    dc0 = dc.copy()
    dc0[:, 4] = 0.0
    o, d = mo.apply(dc0)
    frac = d["n_outliers"] / d["n_out"]
    print(f"[selftest]   outlier fraction = {frac:.4f} (target 0.24)")
    check("09 outlier fraction ~0.24", abs(frac - 0.24) < 0.005, f"{frac:.4f}")
    mo2 = RadarNoiseModel.from_config(_cfg_with(base, outliers=True, wrap=True), seed=13)
    o2, _ = mo2.apply(dc0)
    check("09b outlier values in [-v_max, v_max)",
          float(o2[:, 4].min()) >= -4.0 and float(o2[:, 4].max()) < 4.0)
    nz = o[:, 4] != 0.0
    qq = o[nz, 4] / 0.13
    check("09c outliers quantized", np.max(np.abs(qq - np.round(qq))) < 1e-9)

    # 10 wrap
    w = wrap_symmetric(np.array([4.0, -4.0, 3.99, 4.1, -4.1, 0.0, 8.5]), 4.0)
    check("10 wrap at +/-4.0 boundaries", np.allclose(w, [-4.0, -4.0, 3.99, -3.9, 3.9, 0.0, 0.5]), str(w))
    mw = RadarNoiseModel.from_config(_cfg_with(base, wrap=True), seed=1)
    o, d = mw.apply(np.array([pt(3, 0, 0, 4.1), pt(3, 0, 0, -4.0), pt(3, 0, 0, 1.0)]))
    check("10b wrap via apply()", np.allclose(o[:, 4], [-3.9, -4.0, 1.0]) and d["n_wrapped"] == 1, str(o[:, 4]))

    # 11 residual raises
    bad = dict(base)
    bad["doppler_total_std_mps"] = 0.030
    try:
        RadarNoiseModel.from_config(bad)
        raised = False
    except ValueError:
        raised = True
    check("11 raises when total^2 < v_res^2/12", raised)

    # 12 determinism
    a1, _ = RadarNoiseModel.from_config(base, seed=42).apply(cloud)
    a2, _ = RadarNoiseModel.from_config(base, seed=42).apply(cloud)
    a3, _ = RadarNoiseModel.from_config(base, seed=43).apply(cloud)
    check("12 same seed -> identical output", np.array_equal(a1, a2))
    check("12b different seed -> different output", not np.array_equal(a1, a3))

    # 13 all switches off
    mixed = np.vstack([cloud[:100], [[np.nan, 0, 0, 0, 0]], [pt(-5, 0, 0, 0.3)], [pt(50, 0, 0, 9.0)]])
    o, _ = RadarNoiseModel.from_config(_cfg_with(base), seed=1).apply(mixed)
    fin = mixed[np.all(np.isfinite(mixed), axis=1)]
    check("13 all off -> identical finite input", np.array_equal(o, fin))
    o, d = RadarNoiseModel.from_config(_cfg_with(base, gating=True), seed=1).apply(mixed)
    check("13b only gating -> equals gated input", np.array_equal(o, mixed[d["kept_idx"]]))

    # 14 intensity passes through, unknown switch rejected
    o, d = full.apply(cloud)
    check("14 intensity unchanged", np.array_equal(o[:, 3], cloud[d["kept_idx"], 3]))
    try:
        RadarNoiseModel.from_config(dict(base, enable={"bogus": True}))
        raised = False
    except ValueError:
        raised = True
    check("14b unknown switch rejected", raised)

    # 15 plugin frame convention: theta=0 row points along +x
    dirs_p = plugin_ray_dirs().reshape(8, 90, 3)
    check("15 rmagine theta=0 is +x (boresight)", np.allclose(dirs_p[:, 45, 1], 0.0, atol=1e-12)
          and np.all(dirs_p[:, 45, 0] > 0.9))

    # 16 synthetic tunnel scan
    rep = tunnel_report(base, seed=0)
    print("[selftest]   tunnel scan (corridor 3 m x 2 m, v=+0.3 m/s x):")
    for kk, vv_ in rep.items():
        print(f"[selftest]     {kk}: {vv_:.4f}" if isinstance(vv_, float) else f"[selftest]     {kk}: {vv_}")
    check("16 tunnel scan points survive and robust 3D LS < 0.1 m/s",
          rep["points_out"] >= 8 and rep["ls3d_err_noisy_robust_median_200"] < 0.1, str(rep))

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if n_fail == 0 else "FAILED"))
    return 0 if n_fail == 0 else 1


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--config", default=None, help="optionally validate this yaml's values in the selftest")
    args = p.parse_args()
    if args.selftest:
        cfg = None
        if args.config:
            import yaml

            with open(args.config, "r") as f:
                cfg = yaml.safe_load(f)
        sys.exit(run_selftest(cfg))
    p.print_help()


if __name__ == "__main__":
    main()
