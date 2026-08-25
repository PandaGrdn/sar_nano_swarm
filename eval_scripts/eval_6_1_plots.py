#!/usr/bin/env python3
"""Matplotlib PNG plots for eval_6_1.py (§6.1).

Ubuntu apt matplotlib is built against NumPy 1.x. ROS/pip often ships NumPy 2.
Importing the apt wheel then crashes with `_ARRAY_API not found`. This module
drops `/usr/lib/python3*/dist-packages` from sys.path, unloads a failed
matplotlib, and pip-installs a user wheel if needed.
"""
from __future__ import annotations

import math
import os
import pickle
import site
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, List

import numpy as np

CHI2_95_3 = 7.814727903251179
RPE_DT_S = 1.0

SERIES_KEYS = (
    "err_t",
    "err_m",
    "err_xyz",
    "yaw_rad",
    "rpe_t",
    "rpe_m",
    "nees",
    "sig_p",
    "est_pose",
    "gt_pose",
)


def _is_apt_dist_packages(path: str) -> bool:
    p = path.replace("\\", "/")
    if "dist-packages" not in p:
        return False
    return p.startswith("/usr/lib/python")


def _is_apt_matplotlib(path: str) -> bool:
    p = (path or "").replace("\\", "/")
    return "/usr/lib/python" in p and "dist-packages" in p and "matplotlib" in p


def _user_site_dir() -> str:
    try:
        usp = site.getusersitepackages()
        if usp:
            return usp
    except Exception:
        pass
    ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    return str(Path.home() / ".local" / "lib" / ver / "site-packages")


def _purge_matplotlib() -> None:
    for k in list(sys.modules):
        if k == "matplotlib" or k.startswith("matplotlib."):
            del sys.modules[k]


def _prefer_pip_matplotlib() -> None:
    """Put ~/.local first. Do not prepend Debian dist-packages."""
    user = _user_site_dir()
    Path(user).mkdir(parents=True, exist_ok=True)
    rest = [p for p in sys.path if p and p != user]
    sys.path[:] = [user] + rest


def _pip_install_matplotlib() -> bool:
    print("[eval_6_1] installing matplotlib into user site (NumPy 2-safe) …", flush=True)
    cmd = [
        sys.executable,
        "-m",
        "pip",
        "install",
        "--user",
        "--upgrade",
        "--force-reinstall",
        "matplotlib>=3.8",
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=240)
    except (OSError, subprocess.TimeoutExpired) as e:
        print(f"[eval_6_1] pip install matplotlib failed: {e}", flush=True)
        return False
    tail = ((r.stdout or "") + (r.stderr or ""))[-800:]
    if r.returncode != 0:
        print(f"[eval_6_1] pip install matplotlib rc={r.returncode}: {tail}", flush=True)
        return False
    print("[eval_6_1] pip matplotlib install finished", flush=True)
    return True


def _pyplot(interactive: bool):
    os.environ["MPLBACKEND"] = "Agg" if not interactive else os.environ.get("MPLBACKEND", "Agg")
    _prefer_pip_matplotlib()
    _purge_matplotlib()
    last = None
    for attempt in range(2):
        try:
            import matplotlib

            loc = getattr(matplotlib, "__file__", "") or ""
            if _is_apt_matplotlib(loc):
                raise ImportError(f"refusing Debian matplotlib at {loc}")
            matplotlib.use("Agg", force=True)
            import matplotlib.pyplot as plt

            print(f"[eval_6_1] matplotlib {matplotlib.__version__} from {loc}", flush=True)
            return plt
        except Exception as exc:
            last = exc
            print(f"[eval_6_1] matplotlib import attempt {attempt + 1} failed: {exc}", flush=True)
            _purge_matplotlib()
            if attempt == 0:
                _pip_install_matplotlib()
                _prefer_pip_matplotlib()
    raise RuntimeError(
        f"matplotlib import failed ({last}). "
        f"Fix: {sys.executable} -m pip install --user --force-reinstall 'matplotlib>=3.8'"
    ) from last


def _child_env() -> dict:
    env = os.environ.copy()
    env["EVAL_6_1_PLOT_WORKER"] = "1"
    env["MPLBACKEND"] = "Agg"
    user = _user_site_dir()
    parts = [user]
    for p in (env.get("PYTHONPATH") or "").split(os.pathsep):
        if p and not _is_apt_dist_packages(p):
            parts.append(p)
    here = str(Path(__file__).resolve().parent)
    root = Path(__file__).resolve().parents[1]
    for extra in (here, str(root / "perception" / "swarm_loc"), str(root / "perception" / "uwb_sim")):
        if extra not in parts:
            parts.append(extra)
    env["PYTHONPATH"] = os.pathsep.join(parts)
    return env


def _maybe_legend(ax, **kwargs) -> None:
    h, _ = ax.get_legend_handles_labels()
    if h:
        ax.legend(**kwargs)


def write_plots(
    per: Dict[int, dict],
    hops_ate: Dict[int, float],
    mix: dict,
    estimates: Dict[int, np.ndarray],
    truth: Dict[int, np.ndarray],
    out_dir: Path,
    show: bool = False,
) -> List[str]:
    if os.environ.get("EVAL_6_1_PLOT_WORKER") == "1":
        return _write_plots_impl(per, hops_ate, mix, estimates, truth, out_dir, show)
    payload = {
        "per": per,
        "hops_ate": hops_ate,
        "mix": mix,
        "estimates": estimates,
        "truth": truth,
        "out_dir": str(out_dir),
        "show": bool(show),
    }
    tmp = tempfile.NamedTemporaryFile(suffix=".pkl", delete=False)
    tmp.close()
    try:
        with open(tmp.name, "wb") as f:
            pickle.dump(payload, f, protocol=4)
        cmd = [sys.executable, str(Path(__file__).resolve()), "--worker", tmp.name]
        print("[eval_6_1] plotting in a child process (avoids apt matplotlib) ...", flush=True)
        r = subprocess.run(cmd, env=_child_env(), cwd=str(Path(__file__).resolve().parents[1]))
        if r.returncode != 0:
            print("[eval_6_1] child plot failed, trying in-process …", flush=True)
            return _write_plots_impl(per, hops_ate, mix, estimates, truth, out_dir, show)
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass
    plot_dir = Path(out_dir) / "plots"
    saved = sorted(str(p) for p in plot_dir.glob("*.png")) if plot_dir.is_dir() else []
    return saved


def _write_plots_impl(
    per: Dict[int, dict],
    hops_ate: Dict[int, float],
    mix: dict,
    estimates: Dict[int, np.ndarray],
    truth: Dict[int, np.ndarray],
    out_dir: Path,
    show: bool = False,
) -> List[str]:
    plt = _pyplot(interactive=show)
    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except Exception:
        try:
            plt.style.use("ggplot")
        except Exception:
            pass

    plot_dir = Path(out_dir) / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    saved: List[str] = []
    ids = sorted(per)

    def save(fig, name: str):
        path = plot_dir / name
        fig.tight_layout()
        fig.savefig(path, dpi=140)
        saved.append(str(path))
        if not show:
            plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    if hops_ate:
        hs = list(hops_ate)
        ax.plot(hs, [hops_ate[h] for h in hs], marker="o", lw=2)
        ax.set_xticks(hs)
    ax.set_xlabel("hops from entrance")
    ax.set_ylabel("ATE RMSE (m)")
    ax.set_title("Error vs hops from entrance")
    save(fig, "01_error_vs_hops.png")

    fig, ax = plt.subplots(figsize=(7.4, 4.2))
    for i in ids:
        t = per[i].get("err_t") or []
        if t:
            ax.plot(t, per[i].get("err_m") or [], label=f"cf_{i}", lw=1.2)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("|p_est - p_truth| (m)")
    ax.set_title("Position error vs time")
    _maybe_legend(ax)
    save(fig, "02_ate_vs_time.png")

    fig, axes = plt.subplots(3, 1, figsize=(7.4, 7.2), sharex=True)
    labels = ("x error (m)", "y error (m)", "z error (m)")
    for i in ids:
        t = per[i].get("err_t") or []
        xyz = np.asarray(per[i].get("err_xyz") or [], dtype=np.float64)
        if xyz.size == 0:
            continue
        for k, axk in enumerate(axes):
            axk.plot(t, xyz[:, k], label=f"cf_{i}", lw=1.1)
    for axk, lab in zip(axes, labels):
        axk.set_ylabel(lab)
        _maybe_legend(axk)
    axes[-1].set_xlabel("t (s)")
    axes[0].set_title("Component position error vs time")
    save(fig, "03_xyz_error_vs_time.png")

    fig, ax = plt.subplots(figsize=(7.4, 4.2))
    for i in ids:
        t = per[i].get("err_t") or []
        if t:
            ax.plot(t, [math.degrees(v) for v in (per[i].get("yaw_rad") or [])], label=f"cf_{i}", lw=1.2)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("|yaw error| (deg)")
    ax.set_title("Yaw error vs time")
    _maybe_legend(ax)
    save(fig, "04_yaw_error_vs_time.png")

    fig, ax = plt.subplots(figsize=(7.4, 4.2))
    for i in ids:
        t = per[i].get("err_t") or []
        nees = np.asarray(per[i].get("nees") or [], dtype=np.float64)
        if t and nees.size:
            ax.plot(t, nees, label=f"cf_{i}", lw=1.1)
    ax.axhline(CHI2_95_3, color="k", ls="--", lw=1, label="χ² 0.95 (3 DoF)")
    ax.axhline(3.0, color="0.4", ls=":", lw=1, label="E[NEES]=3")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("position NEES")
    ax.set_title("Filter honesty (NEES)")
    _maybe_legend(ax)
    save(fig, "05_nees_vs_time.png")

    fig, ax = plt.subplots(figsize=(6.6, 6.2))
    for i in ids:
        tru = truth.get(i)
        est = estimates.get(i)
        if tru is not None and getattr(tru, "size", 0):
            ax.plot(tru["p_x"], tru["p_y"], ls="--", lw=1.2, label=f"cf_{i} truth")
        if est is not None and getattr(est, "size", 0):
            ax.plot(est["p_x"], est["p_y"], lw=1.2, label=f"cf_{i} est")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Top-down trajectory (no SE(3) align)")
    ax.set_aspect("equal", adjustable="datalim")
    _maybe_legend(ax, fontsize=8)
    save(fig, "06_xy_topdown.png")

    fig, ax = plt.subplots(figsize=(7.4, 4.2))
    for i in ids:
        tru = truth.get(i)
        est = estimates.get(i)
        if tru is not None and getattr(tru, "size", 0):
            ax.plot(tru["stamp"], tru["p_z"], ls="--", lw=1.2, label=f"cf_{i} truth z")
        if est is not None and getattr(est, "size", 0):
            ax.plot(est["stamp"], est["p_z"], lw=1.2, label=f"cf_{i} est z")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("z (m)")
    ax.set_title("Altitude (no ToF/baro aiding)")
    _maybe_legend(ax, fontsize=8)
    save(fig, "07_z_vs_time.png")

    fig, ax = plt.subplots(figsize=(7.4, 4.2))
    for i in ids:
        t = per[i].get("rpe_t") or []
        if t:
            ax.plot(t, per[i].get("rpe_m") or [], label=f"cf_{i}", lw=1.1)
    ax.set_xlabel("t (s)")
    ax.set_ylabel(f"RPE (m)  Δt={RPE_DT_S:.0f}s")
    ax.set_title("Relative translation error")
    _maybe_legend(ax)
    save(fig, "08_rpe_vs_time.png")

    fig, ax = plt.subplots(figsize=(6.6, 4.0))
    if mix:
        n_uwb = float(mix.get("n_uwb", 0) or 0)
        labs = ["bearing", "range-only", "az-only", "reciprocal", "mutual yaw"]
        vals = [
            mix.get("frac_bearing", 0) * n_uwb,
            mix.get("frac_range_only", 0) * n_uwb,
            mix.get("n_az_only", 0),
            mix.get("n_reciprocal_updates", 0),
            mix.get("n_mutual_yaw", 0),
        ]
        ax.bar(labs, vals)
    ax.set_ylabel("count")
    ax.set_title("UWB mix")
    ax.tick_params(axis="x", rotation=20)
    save(fig, "09_uwb_mix.png")

    fig, ax = plt.subplots(figsize=(7.4, 4.2))
    for i in ids:
        t = per[i].get("err_t") or []
        sig = per[i].get("sig_p") or []
        if t and sig:
            ax.plot(t, sig, label=f"cf_{i} σ_p", lw=1.1)
    ax.axhline(2.0, color="k", ls="--", lw=1, label="diverge trip 2 m")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("rms position σ (m)")
    ax.set_title("Claimed uncertainty vs time")
    _maybe_legend(ax)
    save(fig, "10_sigma_p_vs_time.png")

    fig, axes = plt.subplots(2, 2, figsize=(10.8, 7.8))
    ax = axes[0, 0]
    if hops_ate:
        hs = list(hops_ate)
        ax.plot(hs, [hops_ate[h] for h in hs], marker="o", lw=2)
    ax.set_title("ATE vs hops")
    ax.set_xlabel("hops")
    ax.set_ylabel("m")
    ax = axes[0, 1]
    for i in ids:
        t = per[i].get("err_t") or []
        if t:
            ax.plot(t, per[i].get("err_m") or [], label=f"cf_{i}", lw=1.1)
    ax.set_title("Position error")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("m")
    _maybe_legend(ax, fontsize=7)
    ax = axes[1, 0]
    for i in ids:
        t = per[i].get("err_t") or []
        if t:
            ax.plot(t, [math.degrees(v) for v in (per[i].get("yaw_rad") or [])], label=f"cf_{i}", lw=1.1)
    ax.set_title("Yaw error")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("deg")
    ax = axes[1, 1]
    for i in ids:
        tru = truth.get(i)
        est = estimates.get(i)
        if tru is not None and getattr(tru, "size", 0):
            ax.plot(tru["p_x"], tru["p_y"], ls="--", lw=1.0)
        if est is not None and getattr(est, "size", 0):
            ax.plot(est["p_x"], est["p_y"], lw=1.0, label=f"cf_{i}")
    ax.set_title("XY")
    ax.set_aspect("equal", adjustable="datalim")
    _maybe_legend(ax, fontsize=7)
    save(fig, "00_dashboard.png")

    if show:
        plt.show()
        plt.close("all")
    print(f"[eval_6_1] wrote {len(saved)} matplotlib PNGs -> {plot_dir}", flush=True)
    for p in saved:
        print(f"  {p}", flush=True)
    return saved


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "--worker":
        _prefer_pip_matplotlib()
        with open(sys.argv[2], "rb") as f:
            payload = pickle.load(f)
        _write_plots_impl(
            payload["per"],
            payload["hops_ate"],
            payload["mix"],
            payload["estimates"],
            payload["truth"],
            Path(payload["out_dir"]),
            show=bool(payload.get("show")),
        )
        sys.exit(0)
    print("eval_6_1_plots.py is imported by eval_6_1.py", file=sys.stderr)
    sys.exit(2)
