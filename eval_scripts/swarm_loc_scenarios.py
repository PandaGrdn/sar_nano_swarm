#!/usr/bin/env python3
"""Named swarm-loc live scenarios: environment (world) × situation (what they fly).

Eval artifacts go to:
    out/swarm_loc_eval/<env>/<situation>/
Logs (phase0 --swarm-loc-log-dir):
    out/swarm_loc_logs/<env>/<situation>/

Usage:
    python3 eval_scripts/swarm_loc_scenarios.py --list
    python3 -u eval_scripts/swarm_loc_gate.py --scenario tunnel/triangle_forward
    ./eval_scripts/run_swarm_loc_scenario.sh tunnel/triangle_forward
"""
from __future__ import annotations

import argparse
import math
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

XY = Tuple[float, float]
MotionFn = Callable  # (mcs, t_end, spec) -> None

# Short env slug → phase0_gate.sh -w
ENV_WORLD = {
    "tunnel": "phase0_tunnel_gate",
    "open": "phase1_pid_tune",
}

REPO_EVAL_ROOT = "out/swarm_loc_eval"
REPO_LOG_ROOT = "out/swarm_loc_logs"


def line_x_xy(n: int, spacing: float) -> List[XY]:
    return [(float(i) * float(spacing), 0.0) for i in range(int(n))]


def triangle_xy(n: int, spacing: float) -> List[XY]:
    """Equilateral triangle in XY, apex toward +x. n must be 3."""
    if int(n) != 3:
        raise ValueError("triangle layout requires num_drones=3")
    s = float(spacing)
    h = s * math.sqrt(3.0) / 2.0
    # cf_0 left-rear, cf_1 apex, cf_2 right-rear
    return [(0.0, 0.5 * s), (h, 0.0), (0.0, -0.5 * s)]


def _sleep_until(t_end: float, seconds: float) -> None:
    remain = t_end - time.time()
    if remain <= 0:
        return
    time.sleep(min(float(seconds), remain))


def _stop_all(mcs) -> None:
    for mc in mcs:
        try:
            mc.stop()
        except Exception:
            pass


def _all_forward(mcs, distance_m: float, velocity: float = 0.2) -> None:
    """Start every vehicle at once so a formation translates together."""
    d = abs(float(distance_m))
    v = max(0.05, float(velocity))
    if d < 1e-3:
        return
    for mc in mcs:
        if distance_m >= 0:
            mc.start_forward(v)
        else:
            mc.start_back(v)
    time.sleep(d / v)
    _stop_all(mcs)


def motion_shuttle_then_hover(mcs, t_end: float, spec: dict) -> None:
    """Current gate: 0.4 m forward, 0.4 m back, hover the rest of duration."""
    _all_forward(mcs, 0.4)
    _sleep_until(t_end, 3.0)
    _all_forward(mcs, -0.4)
    _sleep_until(t_end, 3.0)
    remain = t_end - time.time()
    if remain > 0:
        time.sleep(remain)


def motion_repeated_shuttle(mcs, t_end: float, spec: dict) -> None:
    """Keep translating along body +x/−x so RPE / heading see real motion."""
    leg = float(spec.get("leg_m", 0.5))
    while time.time() + 2.0 < t_end:
        _all_forward(mcs, leg)
        _sleep_until(t_end, 1.5)
        if time.time() + 2.0 >= t_end:
            break
        _all_forward(mcs, -leg)
        _sleep_until(t_end, 1.5)
    remain = t_end - time.time()
    if remain > 0:
        time.sleep(remain)


def motion_formation_forward(mcs, t_end: float, spec: dict) -> None:
    """Several simultaneous forward legs, then hover leftover time.

    ``hover_end_s`` is a *minimum* hover after the last leg, not a cap.
    The 2026-09-14 triangle_forward gate returned after ~4 legs + 5 s and
    the sim-time flight window was 4.3 s (under the 5 s liveness floor).
    Always consume remaining wall time until ``t_end`` (the scenario duration).
    """
    n_legs = int(spec.get("n_legs", 4))
    leg = float(spec.get("leg_m", 0.5))
    pause = float(spec.get("pause_s", 2.0))
    for _ in range(n_legs):
        if time.time() + 1.0 >= t_end:
            break
        _all_forward(mcs, leg)
        _sleep_until(t_end, pause)
    hover_end = spec.get("hover_end_s")
    if hover_end is not None:
        _sleep_until(t_end, float(hover_end))
    remain = t_end - time.time()
    if remain > 0:
        time.sleep(remain)


def motion_staggered_advance(mcs, t_end: float, spec: dict) -> None:
    """Rear-to-front one-at-a-time so hop geometry and UWB graph change."""
    leg = float(spec.get("leg_m", 0.4))
    pause = float(spec.get("pause_s", 3.0))
    for mc in reversed(list(mcs)):
        if time.time() + 1.0 >= t_end:
            break
        mc.forward(leg)
        _sleep_until(t_end, pause)
    remain = t_end - time.time()
    if remain > 0:
        time.sleep(remain)


LAYOUTS = {
    "line_x": line_x_xy,
    "triangle": triangle_xy,
}

MOTIONS = {
    "shuttle_then_hover": motion_shuttle_then_hover,
    "repeated_shuttle": motion_repeated_shuttle,
    "formation_forward": motion_formation_forward,
    "staggered_advance": motion_staggered_advance,
}

# env / situation → spec
SCENARIOS: Dict[str, dict] = {
    "tunnel/collinear_hover": {
        "env": "tunnel",
        "situation": "collinear_hover",
        "world": "phase0_tunnel_gate",
        "num_drones": 3,
        "spacing": 1.5,
        "layout": "line_x",
        "motion": "shuttle_then_hover",
        "duration": 90.0,
        "why": "Baseline: line along the tube, tiny shuttle, then hover. Gauge + collinear UWB.",
    },
    "tunnel/collinear_shuttle": {
        "env": "tunnel",
        "situation": "collinear_shuttle",
        "world": "phase0_tunnel_gate",
        "num_drones": 3,
        "spacing": 1.5,
        "layout": "line_x",
        "motion": "repeated_shuttle",
        "leg_m": 0.5,
        "duration": 90.0,
        "why": "Same line, keep moving so ATE/RPE are not a 5 min hover blob.",
    },
    "tunnel/collinear_forward": {
        "env": "tunnel",
        "situation": "collinear_forward",
        "world": "phase0_tunnel_gate",
        "num_drones": 3,
        "spacing": 1.5,
        "layout": "line_x",
        "motion": "formation_forward",
        "n_legs": 5,
        "leg_m": 0.4,
        "pause_s": 2.0,
        "duration": 90.0,
        "why": "Line formation translates down-tunnel (body +x). Tests along-track error.",
    },
    "tunnel/triangle_forward": {
        "env": "tunnel",
        "situation": "triangle_forward",
        "world": "phase0_tunnel_gate",
        "num_drones": 3,
        "spacing": 0.9,
        "layout": "triangle",
        "motion": "formation_forward",
        "n_legs": 4,
        "leg_m": 0.5,
        "pause_s": 2.0,
        "hover_end_s": 5.0,
        "duration": 45.0,
        "why": "Non-collinear mesh: equilateral triangle, all translate +x together, 5 s hover then land.",
    },
    "tunnel/staggered_advance": {
        "env": "tunnel",
        "situation": "staggered_advance",
        "world": "phase0_tunnel_gate",
        "num_drones": 3,
        "spacing": 1.5,
        "layout": "line_x",
        "motion": "staggered_advance",
        "leg_m": 0.4,
        "pause_s": 3.0,
        "duration": 90.0,
        "why": "Hop/graph change: drones advance one at a time (deepest first).",
    },
    "open/triangle_hover": {
        "env": "open",
        "situation": "triangle_hover",
        "world": "phase1_pid_tune",
        "num_drones": 3,
        "spacing": 1.2,
        "layout": "triangle",
        "motion": "shuttle_then_hover",
        "duration": 90.0,
        "why": "Same triangle, no cave walls — isolate geometry vs tunnel degeneracy.",
    },
}

ALIASES = {spec["situation"]: key for key, spec in SCENARIOS.items()}
# last alias wins if situation names collide; they don't today.


def parse_scenario_key(raw: str) -> str:
    s = (raw or "").strip().strip("/")
    if not s:
        raise KeyError("empty scenario")
    if s in SCENARIOS:
        return s
    if s in ALIASES:
        return ALIASES[s]
    if "/" not in s:
        matches = [k for k in SCENARIOS if k.endswith("/" + s)]
        if len(matches) == 1:
            return matches[0]
    raise KeyError(
        f"unknown scenario {raw!r}. Try --list-scenarios. Known: {', '.join(SCENARIOS)}"
    )


def get_scenario(raw: str) -> dict:
    key = parse_scenario_key(raw)
    spec = dict(SCENARIOS[key])
    spec["key"] = key
    return spec


def spawn_xy(spec: dict) -> List[XY]:
    layout = LAYOUTS[spec["layout"]]
    return layout(int(spec["num_drones"]), float(spec["spacing"]))


def eval_dir_for(spec: dict) -> str:
    return f"{REPO_EVAL_ROOT}/{spec['env']}/{spec['situation']}"


def log_dir_for(spec: dict) -> str:
    return f"{REPO_LOG_ROOT}/{spec['env']}/{spec['situation']}"


def derived_estimator_config(
    spec: dict, base_cfg: dict, hover_height: float = 0.5
) -> dict:
    """Estimator config with launch geometry matching THIS scenario (D13).

    The gate reset_poses drone i to (spawn_xy[i], hover_height); the stock
    swarm_loc.yaml `launch:` block assumes a 1.5 m line. Write the actual
    per-drone positions into `launch.positions_xyz_m` so the estimators
    initialize where the drones really are. Purely surveyed deployment
    geometry — never a truth read, never uwb_pdoa.yaml.
    """
    import copy

    cfg = copy.deepcopy(base_cfg)
    xy = spawn_xy(spec)
    cfg.setdefault("launch", {})
    cfg["launch"]["positions_xyz_m"] = [
        [float(x), float(y), float(hover_height)] for (x, y) in xy
    ]
    # Keep the line parameters coherent for anything still reading them.
    cfg["launch"]["spacing_m"] = float(spec["spacing"])
    return cfg


def write_derived_estimator_config(
    scenario: str, base_path: str, out_path: str, hover_height: float = 0.5
) -> str:
    import yaml

    spec = get_scenario(scenario)
    with open(base_path, "r") as f:
        base_cfg = yaml.safe_load(f)
    cfg = derived_estimator_config(spec, base_cfg, hover_height)
    header = (
        "# DERIVED estimator config — generated by swarm_loc_scenarios.py for\n"
        f"# scenario {spec['key']} (launch.positions_xyz_m = actual spawn/reset\n"
        f"# geometry). Base: {base_path}. Do not edit; regenerate instead.\n"
    )
    with open(out_path, "w") as f:
        f.write(header)
        yaml.safe_dump(cfg, f, sort_keys=False)
    return out_path


def apply_motion(mcs, t_end: float, spec: dict) -> None:
    fn = MOTIONS[spec["motion"]]
    fn(mcs, t_end, spec)


def phase0_cmd(spec: dict, extra: Optional[Sequence[str]] = None) -> List[str]:
    cmd = [
        "./eval_scripts/phase0_gate.sh",
        "-w",
        str(spec["world"]),
        "-n",
        str(int(spec["num_drones"])),
        "--spacing",
        str(float(spec["spacing"])),
        "--swarm-loc-log-dir",
        log_dir_for(spec),
    ]
    if extra:
        cmd.extend(extra)
    return cmd


def run_selftest() -> int:
    ok = True

    def check(name: str, cond: bool, detail: str = ""):
        nonlocal ok
        if not cond:
            ok = False
            print(f"  FAIL {name} {detail}")
        else:
            print(f"  PASS {name}")

    xy = triangle_xy(3, 0.9)
    check("triangle n=3", len(xy) == 3)
    check("triangle apex +x", xy[1][0] > xy[0][0] and abs(xy[1][1]) < 1e-9)
    check("triangle symmetric y", abs(xy[0][1] + xy[2][1]) < 1e-9)
    try:
        triangle_xy(2, 1.0)
        check("triangle rejects n!=3", False)
    except ValueError:
        check("triangle rejects n!=3", True)
    check("line", line_x_xy(3, 1.5) == [(0.0, 0.0), (1.5, 0.0), (3.0, 0.0)])
    spec = get_scenario("triangle_forward")
    check("alias triangle_forward", spec["key"] == "tunnel/triangle_forward")
    check("eval path", eval_dir_for(spec) == "out/swarm_loc_eval/tunnel/triangle_forward")
    check("every motion exists", all(s["motion"] in MOTIONS for s in SCENARIOS.values()))
    check("every layout exists", all(s["layout"] in LAYOUTS for s in SCENARIOS.values()))

    # derived estimator config (D13 launch geometry per scenario)
    base = {"launch": {"spawn_x0_m": 0.0, "spawn_y_m": 0.0, "spawn_z_m": 0.5,
                       "spacing_m": 1.5, "init_yaw_deg": 0.0},
            "seed": 0}
    tri = derived_estimator_config(get_scenario("triangle_forward"), base, 0.5)
    pos = tri["launch"]["positions_xyz_m"]
    check("derived tri 3 positions", len(pos) == 3)
    check(
        "derived tri apex",
        abs(pos[1][0] - 0.9 * math.sqrt(3.0) / 2.0) < 1e-9 and abs(pos[1][1]) < 1e-9,
        str(pos),
    )
    check("derived tri z = hover", all(abs(p[2] - 0.5) < 1e-12 for p in pos))
    check("derived keeps base keys", tri["launch"]["init_yaw_deg"] == 0.0 and "seed" in tri)
    check("derived does not mutate base", "positions_xyz_m" not in base["launch"])

    # formation_forward must consume t_end, not return after hover_end_s.
    slept: list = []
    real_sleep = time.sleep
    real_time = time.time
    t0 = {"now": 1000.0}

    def fake_time():
        return t0["now"]

    def fake_sleep(s):
        slept.append(float(s))
        t0["now"] += float(s)

    class _Mc:
        def start_forward(self, _v):
            return None
        def start_back(self, _v):
            return None
        def stop(self):
            return None

    time.time = fake_time  # type: ignore[assignment]
    time.sleep = fake_sleep  # type: ignore[assignment]
    try:
        motion_formation_forward(
            [_Mc(), _Mc(), _Mc()],
            t0["now"] + 45.0,
            {"n_legs": 4, "leg_m": 0.5, "pause_s": 2.0, "hover_end_s": 5.0},
        )
    finally:
        time.time = real_time
        time.sleep = real_sleep
    # 4*(0.5/0.2 travel + 2s pause) + 5s hover_end + remainder → 45 s total
    check("formation_forward consumes duration",
          abs(sum(slept) - 45.0) < 1e-9, f"slept={sum(slept):.3f} {slept}")
    line = derived_estimator_config(get_scenario("tunnel/collinear_hover"), base, 0.5)
    check(
        "derived line matches x0+i*spacing",
        all(
            abs(line["launch"]["positions_xyz_m"][i][0] - i * 1.5) < 1e-9
            and abs(line["launch"]["positions_xyz_m"][i][1]) < 1e-9
            for i in range(3)
        ),
    )
    print("[swarm_loc_scenarios] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument(
        "--write-config",
        metavar="SCENARIO",
        default="",
        help="Write a derived estimator config (launch.positions_xyz_m for this scenario) and exit.",
    )
    ap.add_argument("--base", default="configs/estimation/swarm_loc.yaml")
    ap.add_argument("--out", default="")
    ap.add_argument("--hover-height", type=float, default=0.5)
    args = ap.parse_args()
    if args.selftest:
        raise SystemExit(run_selftest())
    if args.write_config:
        if not args.out:
            ap.error("--write-config requires --out PATH")
        p = write_derived_estimator_config(
            args.write_config, args.base, args.out, args.hover_height
        )
        print(p)
        return
    print("env/situation                  world                 n  layout    motion")
    for key, spec in SCENARIOS.items():
        print(
            f"  {key:<30} {spec['world']:<20} {spec['num_drones']}  "
            f"{spec['layout']:<9} {spec['motion']}"
        )
        print(f"      {spec['why']}")
        print(f"      eval → {eval_dir_for(spec)}")


if __name__ == "__main__":
    main()
