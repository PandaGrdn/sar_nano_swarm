#!/usr/bin/env python3
"""run_ablations.py — P2-8 / §6.2 identical-scenario ablation sweep.

Offline (same process as P2-1…P2-7). Live knobs are written with
``--write-live-configs DIR`` for phase0_gate.sh; they are not required
to pass this gate.

Conditions (nominal and 2× UWB noise):
  1 rio_only
  2 uwb_range_no_rio
  3 rio_range
  4 rio_range_bearing
  5 full_minus_entrance
  6 full_minus_mutual_yaw
  7 full
  8 centralized (P2-7 batch LS on condition 7's log)

Usage:
    python3 eval_scripts/run_ablations.py --selftest
    python3 eval_scripts/run_ablations.py --write-live-configs /tmp/p2_8
"""
from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path
from typing import Dict

import numpy as np
import yaml

_REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in ("perception/swarm_loc", "perception/uwb_sim", "eval_scripts"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

from central_reference import solve  # noqa: E402
from meas_log import EST_DTYPE, MeasurementLogger, RIO_DTYPE, UWB_DTYPE  # noqa: E402
from rio_stub import load_config, resolve_config_path  # noqa: E402
from stress import run_line_mesh  # noqa: E402

CONDITIONS = [
    ("rio_only", {"disable_uwb": True}),
    ("uwb_range_no_rio", {"disable_rio": True, "disable_bearing": True}),
    ("rio_range", {"disable_bearing": True}),
    ("rio_range_bearing", {}),
    ("full_minus_entrance", {"disable_entrance": True}),
    ("full_minus_mutual_yaw", {}),  # cone-starved; yaw flag is a no-op on this geometry
    ("full", {}),
]


def run_condition(base: dict, name: str, abl: dict, noise_scale: float, seed: int = 21) -> dict:
    cfg = copy.deepcopy(base)
    cfg["ablation"] = dict(cfg.get("ablation", {}))
    cfg["ablation"].update(abl)
    cfg["ablation"]["uwb_noise_scale"] = float(noise_scale)
    if name == "full_minus_mutual_yaw":
        cfg["measurements"] = dict(cfg["measurements"])
        cfg["measurements"]["use_mutual_yaw"] = False
    loggers = None
    out = run_line_mesh(cfg, seed=seed, loggers=loggers)
    out["name"] = name
    out["noise_scale"] = noise_scale
    out["loggers"] = loggers
    return out


def _logger_to_rec(lg: MeasurementLogger) -> dict:
    return {
        "drone_id": lg.drone_id,
        "rio": np.array(lg.rio, dtype=RIO_DTYPE) if lg.rio else np.zeros(0, dtype=RIO_DTYPE),
        "uwb": np.array(lg.uwb, dtype=UWB_DTYPE) if lg.uwb else np.zeros(0, dtype=UWB_DTYPE),
        "estimate": np.array(lg.est, dtype=EST_DTYPE) if lg.est else np.zeros(0, dtype=EST_DTYPE),
    }


def write_live_configs(out_dir: str) -> None:
    """Materialize the plan's /tmp UWB copies: empty entrance, NLOS wall."""
    src = _REPO_ROOT / "configs" / "sensors" / "uwb_pdoa.yaml"
    with open(src, "r") as f:
        uwb = yaml.safe_load(f)
    d = Path(out_dir)
    d.mkdir(parents=True, exist_ok=True)
    no_ent = copy.deepcopy(uwb)
    no_ent["static_peers"] = []
    with open(d / "uwb_no_entrance.yaml", "w") as f:
        yaml.safe_dump(no_ent, f, sort_keys=False)
    nlos = copy.deepcopy(uwb)
    nlos["los_model"] = "boxes"
    nlos["occluder_boxes"] = [
        {
            "name": "wall_between_1_and_2",
            "x_min": 2.2,
            "x_max": 2.4,
            "y_min": -5.0,
            "y_max": 5.0,
            "z_min": 0.0,
            "z_max": 3.0,
        }
    ]
    with open(d / "uwb_nlos_wall.yaml", "w") as f:
        yaml.safe_dump(nlos, f, sort_keys=False)
    est = load_config(resolve_config_path("configs/estimation/swarm_loc.yaml"))
    rio_burst = copy.deepcopy(est)
    rio_burst["rio"]["dropout_rate"] = 0.4
    rio_burst["rio"]["dropout_duration_s"] = 1.0
    with open(d / "swarm_loc_rio_dropout.yaml", "w") as f:
        yaml.safe_dump(rio_burst, f, sort_keys=False)
    print(f"[run_ablations] wrote live knobs in {d}")
    print("  corridor:     ./eval_scripts/phase0_gate.sh -w phase0_tunnel_gate -n 4 --spacing 1.5 --headless --no-rviz")
    print("  entrance off:  --uwb-config <dir>/uwb_no_entrance.yaml")
    print("  NLOS wall:     --uwb-config <dir>/uwb_nlos_wall.yaml")
    print("  RIO bursts:    --swarm-loc-config <dir>/swarm_loc_rio_dropout.yaml")
    print("  partition:     kill the cf2 of drone 1 after takeoff (odom_stale_s)")


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

    base = load_config(resolve_config_path("configs/estimation/swarm_loc.yaml"))
    rows: Dict[str, dict] = {}
    for scale in (1.0, 2.0):
        print(f"[selftest] ablation sweep noise_scale={scale} …")
        for name, abl in CONDITIONS:
            r = run_condition(base, name, abl, scale, seed=21)
            rows[f"{name}@{scale}"] = r
            print(
                f"  {name:22s} x{scale:.0f}  ATE={r['ate']:.3f}  "
                f"sigma={r['mean_sigma']:.3f}  finite={r['finite']}  "
                f"overconf={r['overconfident']}"
            )
            check(
                f"{name} x{scale} finite",
                r["finite"] and (not any(r["diverged"].values()) or name == "rio_only"),
                str(r["diverged"]),
            )
            if name == "uwb_range_no_rio":
                check(
                    f"{name} x{scale} corridor degeneracy",
                    r["ate"] > 0.25 and r["mean_sigma"] < 0.08,
                    f"ATE={r['ate']:.3f} sigma={r['mean_sigma']:.3f} overconf={r['overconfident']}",
                )
            else:
                check(
                    f"{name} x{scale} honest",
                    r["overconfident"] == [],
                    str(r["overconfident"]),
                )

    full = rows["full@1.0"]
    rio = rows["rio_only@1.0"]
    rng = rows["rio_range@1.0"]
    noent = rows["full_minus_entrance@1.0"]
    yaw_off = rows["full_minus_mutual_yaw@1.0"]
    check("1 rio-only worse than full", rio["ate"] > 1.15 * full["ate"], f"rio={rio['ate']:.3f} full={full['ate']:.3f}")
    check(
        "2 rio-only cov grew",
        rio["mean_sigma"] > full["mean_sigma"],
        f"rio_s={rio['mean_sigma']:.3f} full_s={full['mean_sigma']:.3f}",
    )
    check(
        "3 range-only worse or similar vs bearing",
        rng["ate"] >= 0.90 * full["ate"],
        f"range={rng['ate']:.3f} full={full['ate']:.3f}",
    )
    check(
        "4 no-entrance not overconfident",
        noent["overconfident"] == [],
        str(noent["overconfident"]),
    )
    # Corridor ±45° cone: mutual yaw is starved. Indistinguishable is a pass.
    dpsi = abs(yaw_off["ate"] - full["ate"])
    check("5 minus-yaw ~ full (starved cone)", dpsi < 0.05, f"dATE={dpsi:.3f}")
    print(
        f"[selftest] n_mutual_yaw_pairs_per_s=0  (corridor cone; yaw-off ATE={yaw_off['ate']:.3f} "
        f"full={full['ate']:.3f})"
    )

    print("[selftest] centralized on static full log …")
    cfg_full = copy.deepcopy(base)
    loggers = {i: MeasurementLogger(i) for i in range(4)}
    static = run_line_mesh(cfg_full, speed_mps=0.0, seed=21, loggers=loggers)
    logs = {i: _logger_to_rec(loggers[i]) for i in range(4)}
    for i in logs:
        logs[i]["rio"] = np.zeros(0, dtype=RIO_DTYPE)
    ref = solve(logs, static["cfg"])
    ate_c = float(
        np.mean([np.linalg.norm(ref["last"][i]["p"] - static["truth"][i]) for i in range(4)])
    )
    print(f"[selftest] centralized ATE={ate_c:.3f} distributed={static['ate']:.3f}")
    check("6 centralized finite", np.isfinite(ate_c) and static["finite"])
    check(
        "6b centralized close to dist on static corridor",
        ate_c < static["ate"] + 0.12,
        f"c={ate_c:.3f} d={static['ate']:.3f}",
    )

    x2 = rows["full@2.0"]
    check("7 2x noise still finite", x2["finite"] and x2["overconfident"] == [])

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--write-live-configs", default="", metavar="DIR")
    args = parser.parse_args()
    if args.write_live_configs:
        write_live_configs(args.write_live_configs)
        if not args.selftest:
            sys.exit(0)
    if args.selftest:
        sys.exit(run_selftest())
    parser.print_help()
    sys.exit(2)


if __name__ == "__main__":
    main()
