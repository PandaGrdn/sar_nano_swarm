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


def sim_sleep(recorder, duration_s: float, wall_timeout_s: float = 60.0) -> bool:
    """Sleep for `duration_s` of SIM time, or fall back to wall-clock if no recorder.

    Polls recorder.sim_now() every ~20 ms. Returns True if duration elapsed,
    False if wall timeout fired (sim diverged/stalled).
    When recorder is None, does wall-clock sleep and returns True.
    """
    if recorder is None:
        time.sleep(float(duration_s))
        return True

    import math
    t_sim0 = recorder.sim_now()
    t_wall0 = time.time()
    wall_timeout = float(max(60.0, duration_s * 20.0))  # Generous: at least 60s or 20× duration
    while True:
        t_sim = recorder.sim_now()
        if (math.isfinite(t_sim) and math.isfinite(t_sim0) and
                (t_sim - t_sim0) >= float(duration_s)):
            return True
        if time.time() - t_wall0 >= wall_timeout:
            return False
        time.sleep(0.02)  # Poll every 20 ms


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


def motion_shuttle_then_hover(mcs, t_end: float, spec: dict,
                              recorder: Optional[object] = None) -> None:
    """Current gate: 0.4 m forward, 0.4 m back, hover the rest of duration.

    When recorder is provided, t_end is a sim-time deadline (sim sec from flight start).
    Without recorder, t_end is a wall-clock deadline (wall sec since epoch).
    """
    import math
    t_sim_start = recorder.sim_now() if recorder else None

    _all_forward(mcs, 0.4)
    if recorder is not None:
        sim_sleep(recorder, 3.0)
    else:
        _sleep_until(t_end, 3.0)
    _all_forward(mcs, -0.4)
    if recorder is not None:
        sim_sleep(recorder, 3.0)
    else:
        _sleep_until(t_end, 3.0)

    # Hover for the remainder
    if recorder is not None:
        t_now_sim = recorder.sim_now()
        if math.isfinite(t_sim_start) and math.isfinite(t_now_sim):
            remain_sim = t_end - (t_now_sim - t_sim_start)
            if remain_sim > 0:
                sim_sleep(recorder, remain_sim)
    else:
        remain = t_end - time.time()
        if remain > 0:
            time.sleep(remain)


def motion_repeated_shuttle(mcs, t_end: float, spec: dict,
                             recorder: Optional[object] = None) -> None:
    """Keep translating along body +x/−x so RPE / heading see real motion.

    When recorder is provided, t_end is a sim-time deadline.
    Without recorder, t_end is a wall-clock deadline.
    """
    import math
    leg = float(spec.get("leg_m", 0.5))
    t_sim_start = recorder.sim_now() if recorder else None

    if recorder is not None:
        # Sim-time mode: check elapsed sim time
        while True:
            t_now_sim = recorder.sim_now()
            if math.isfinite(t_sim_start) and math.isfinite(t_now_sim):
                elapsed_sim = t_now_sim - t_sim_start
                if elapsed_sim + 2.0 >= t_end:
                    break
            _all_forward(mcs, leg)
            sim_sleep(recorder, 1.5)
            t_now_sim = recorder.sim_now()
            if math.isfinite(t_sim_start) and math.isfinite(t_now_sim):
                elapsed_sim = t_now_sim - t_sim_start
                if elapsed_sim + 2.0 >= t_end:
                    break
            _all_forward(mcs, -leg)
            sim_sleep(recorder, 1.5)
        # Consume remainder
        t_now_sim = recorder.sim_now()
        if math.isfinite(t_sim_start) and math.isfinite(t_now_sim):
            remain_sim = t_end - (t_now_sim - t_sim_start)
            if remain_sim > 0:
                sim_sleep(recorder, remain_sim)
    else:
        # Wall-clock mode: old behavior
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


def motion_formation_forward(mcs, t_end: float, spec: dict,
                             recorder: Optional[object] = None) -> None:
    """Several simultaneous forward legs, then hover leftover time.

    ``hover_end_s`` is a *minimum* hover after the last leg, not a cap.
    The 2026-09-14 triangle_forward gate returned after ~4 legs + 5 s and
    the sim-time flight window was 4.3 s (under the 5 s liveness floor).
    Always consume remaining time until ``t_end`` (the scenario duration).
    When recorder is provided, t_end is a sim-time deadline.
    """
    import math
    n_legs = int(spec.get("n_legs", 4))
    leg = float(spec.get("leg_m", 0.5))
    pause = float(spec.get("pause_s", 2.0))
    t_sim_start = recorder.sim_now() if recorder else None

    for _ in range(n_legs):
        if recorder is not None:
            t_now_sim = recorder.sim_now()
            if math.isfinite(t_sim_start) and math.isfinite(t_now_sim):
                elapsed_sim = t_now_sim - t_sim_start
                if elapsed_sim + 1.0 >= t_end:
                    break
        else:
            if time.time() + 1.0 >= t_end:
                break
        _all_forward(mcs, leg)
        if recorder is not None:
            sim_sleep(recorder, pause)
        else:
            _sleep_until(t_end, pause)

    hover_end = spec.get("hover_end_s")
    if hover_end is not None:
        if recorder is not None:
            sim_sleep(recorder, float(hover_end))
        else:
            _sleep_until(t_end, float(hover_end))

    # Consume remainder
    if recorder is not None:
        t_now_sim = recorder.sim_now()
        if math.isfinite(t_sim_start) and math.isfinite(t_now_sim):
            remain_sim = t_end - (t_now_sim - t_sim_start)
            if remain_sim > 0:
                sim_sleep(recorder, remain_sim)
    else:
        remain = t_end - time.time()
        if remain > 0:
            time.sleep(remain)


def motion_staggered_advance(mcs, t_end: float, spec: dict,
                             recorder: Optional[object] = None) -> None:
    """Rear-to-front one-at-a-time so hop geometry and UWB graph change.

    When recorder is provided, t_end is a sim-time deadline.
    Without recorder, t_end is a wall-clock deadline.
    """
    import math
    leg = float(spec.get("leg_m", 0.4))
    pause = float(spec.get("pause_s", 3.0))
    t_sim_start = recorder.sim_now() if recorder else None

    for mc in reversed(list(mcs)):
        if recorder is not None:
            t_now_sim = recorder.sim_now()
            if math.isfinite(t_sim_start) and math.isfinite(t_now_sim):
                elapsed_sim = t_now_sim - t_sim_start
                if elapsed_sim + 1.0 >= t_end:
                    break
        else:
            if time.time() + 1.0 >= t_end:
                break
        mc.forward(leg)
        if recorder is not None:
            sim_sleep(recorder, pause)
        else:
            _sleep_until(t_end, pause)

    # Consume remainder
    if recorder is not None:
        t_now_sim = recorder.sim_now()
        if math.isfinite(t_sim_start) and math.isfinite(t_now_sim):
            remain_sim = t_end - (t_now_sim - t_sim_start)
            if remain_sim > 0:
                sim_sleep(recorder, remain_sim)
    else:
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
    spec: dict, base_cfg: dict, hover_height: float = 0.5, site: Optional[dict] = None
) -> dict:
    """Estimator config with launch geometry matching THIS scenario (D13).

    The gate reset_poses drone i to (spawn_xy[i], hover_height); the stock
    swarm_loc.yaml `launch:` block assumes a 1.5 m line. Write the actual
    per-drone positions into `launch.positions_xyz_m` so the estimators
    initialize where the drones really are. Tunnel scenarios also write
    `entrance` from configs/sim/tunnel_site.yaml. Purely surveyed deployment
    geometry — never a truth read, never uwb_pdoa.yaml.
    """
    import copy

    from tunnel_site import apply_pad_to_estimator, apply_site_to_estimator, try_load_site

    cfg = copy.deepcopy(base_cfg)
    xy = spawn_xy(spec)
    cfg.setdefault("launch", {})
    cfg["launch"]["positions_xyz_m"] = [
        [float(x), float(y), float(hover_height)] for (x, y) in xy
    ]
    cfg["launch"]["spacing_m"] = float(spec["spacing"])
    if site is None and spec.get("env") == "tunnel" and spec.get("world") == "phase0_tunnel_gate":
        site = try_load_site()
    if site:
        cfg = apply_site_to_estimator(cfg, xy, site, hover=hover_height)
        cfg["launch"]["spacing_m"] = float(spec["spacing"])
        if site.get("launch_pad"):
            cfg = apply_pad_to_estimator(cfg, xy, site)
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
        f"# scenario {spec['key']} (launch.positions_xyz_m + entrance from\n"
        f"# tunnel_site.yaml). Base: {base_path}. Do not edit; regenerate.\n"
    )
    with open(out_path, "w") as f:
        f.write(header)
        yaml.safe_dump(cfg, f, sort_keys=False)
    return out_path


def apply_motion(mcs, t_end: float, spec: dict, recorder: Optional[object] = None) -> None:
    """Execute scenario motion. When recorder is provided, t_end is sim-time seconds."""
    fn = MOTIONS[spec["motion"]]
    fn(mcs, t_end, spec, recorder)


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
    if spec.get("env") == "tunnel" and spec.get("world") == "phase0_tunnel_gate":
        from tunnel_site import pad_rest_positions, spawn_positions_arg, try_load_site
        site = try_load_site()
        if site:
            xyz = site["spawn"]["xyz_m"]
            cmd.extend(["-x", str(float(xyz[0])), "-y", str(float(xyz[1])),
                        "-z", str(float(site["spawn"].get("hover_height_m", xyz[2])))])
            if site.get("launch_pad"):
                positions = pad_rest_positions(site, spawn_xy(spec))
                cmd.extend(["--spawn-positions", spawn_positions_arg(positions)])
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
    # Explicit site (no launch_pad) so this check is independent of whatever
    # the ambient configs/sim/tunnel_site.yaml on disk happens to contain
    # (with a launch_pad, positions_xyz_m z is pad_top_z_m + hover_above_pad_m,
    # covered separately below).
    neutral_site = {
        "spawn": {"xyz_m": [0.0, 0.0, 0.5], "yaw_deg": 0.0, "hover_height_m": 0.5},
        "entrance": {"xyz_m": [-2.0, 0.0, 0.30], "yaw_deg": 0.0, "device_id": 1000},
        "mesh": {"pose_xyz_m": [0.0, 0.0, 0.0], "pose_rpy_rad": [0.0, 0.0, 0.0]},
    }
    tri = derived_estimator_config(get_scenario("triangle_forward"), base, 0.5, site=neutral_site)
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
    fake_site = {
        "spawn": {"xyz_m": [10.0, -3.0, 0.6], "yaw_deg": 15.0, "hover_height_m": 0.6},
        "entrance": {"xyz_m": [8.0, -3.0, 0.30], "yaw_deg": 15.0, "device_id": 1000},
        "mesh": {"pose_xyz_m": [0, 0, 0], "pose_rpy_rad": [0, 0, 0]},
    }
    lined = derived_estimator_config(
        get_scenario("tunnel/collinear_hover"), base, 0.6, site=fake_site
    )
    check("derived site offsets line",
          abs(lined["launch"]["positions_xyz_m"][1][0] - 11.5) < 1e-9
          and abs(lined["launch"]["positions_xyz_m"][1][1] + 3.0) < 1e-9)
    check("derived site writes entrance",
          lined["entrance"]["position_xyz_m"] == [8.0, -3.0, 0.30]
          and abs(float(lined["entrance"]["yaw_deg"]) - 15.0) < 1e-12)
    check("derived site does not mutate base entrance",
          "entrance" not in base or base.get("entrance", {}).get("position_xyz_m") != [8.0, -3.0, 0.30])

    # ── launch pad: site with launch_pad -> D-tunnel-pad keys; without -> unchanged
    fake_site_pad = dict(fake_site)
    fake_site_pad["launch_pad"] = {
        "center_xy_m": [10.0, -3.0], "size_xy_m": [3.0, 0.9], "yaw_deg": 15.0,
        "top_z_m": 0.05, "thickness_m": 0.1, "hover_above_pad_m": 0.6,
    }
    tri_pad = derived_estimator_config(
        get_scenario("triangle_forward"), base, 0.6, site=fake_site_pad
    )
    check("pad present: spawned_at_layout written",
          tri_pad["launch"]["spawned_at_layout"] is True)
    check("pad present: pad_top_z_m written",
          abs(tri_pad["launch"]["pad_top_z_m"] - 0.05) < 1e-12)
    check("pad present: hover_above_pad_m written",
          abs(tri_pad["launch"]["hover_above_pad_m"] - 0.6) < 1e-12)
    check("pad present: positions z = top + hover_above",
          all(abs(p[2] - 0.65) < 1e-9 for p in tri_pad["launch"]["positions_xyz_m"]),
          str(tri_pad["launch"]["positions_xyz_m"]))

    tri_no_pad = derived_estimator_config(
        get_scenario("triangle_forward"), base, 0.6, site=fake_site
    )
    check("pad absent: no new keys, unchanged behavior",
          "spawned_at_layout" not in tri_no_pad["launch"]
          and "pad_top_z_m" not in tri_no_pad["launch"]
          and "hover_above_pad_m" not in tri_no_pad["launch"])

    from tunnel_site import CF_COLLISION_HALF_HEIGHT_M, PAD_REST_CLEARANCE_M, pad_rest_positions

    tri_spec = get_scenario("triangle_forward")
    rest = pad_rest_positions(fake_site_pad, spawn_xy(tri_spec))
    expect_z = 0.05 + CF_COLLISION_HALF_HEIGHT_M + PAD_REST_CLEARANCE_M
    check("pad rest positions z = pad top + collision offset + clearance",
          all(abs(p[2] - expect_z) < 1e-9 for p in rest), str(rest))

    cmd_with_pad = phase0_cmd(tri_spec)

    def _find(cmd, flag):
        i = cmd.index(flag)
        return cmd[i + 1]

    # phase0_cmd loads the real (non-fake) site off disk via try_load_site();
    # only assert --spawn-positions appears/is absent consistent with that site.
    from tunnel_site import try_load_site
    real_site = try_load_site()
    if real_site and real_site.get("launch_pad"):
        check("phase0_cmd includes --spawn-positions when the real site has a pad",
              "--spawn-positions" in cmd_with_pad)
        pos_str = _find(cmd_with_pad, "--spawn-positions")
        check("phase0_cmd --spawn-positions has one entry per drone",
              len(pos_str.split(";")) == int(tri_spec["num_drones"]), pos_str)
    else:
        check("phase0_cmd omits --spawn-positions when the real site has no pad",
              "--spawn-positions" not in cmd_with_pad)

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
