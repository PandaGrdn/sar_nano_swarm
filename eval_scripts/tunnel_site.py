#!/usr/bin/env python3
"""tunnel_site.py — one surveyed site frame for the phase0 tunnel world.

The estimator must NEVER read this file (AGENTS.md §1). It consumes launch
and entrance only via the derived swarm_loc yaml. The UWB simulator may
read it (or a derived uwb yaml) because that is how measurements are made.

Usage:
    python3 eval_scripts/tunnel_site.py --selftest
    python3 eval_scripts/tunnel_site.py --derive-uwb --mesh-path MAP.obj --out PATH
    python3 eval_scripts/tunnel_site.py --apply-sdf
"""
from __future__ import annotations

import argparse
import copy
import math
import os
import re
import sys
from pathlib import Path
from typing import Any, Optional, Tuple

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SITE = REPO_ROOT / "configs" / "sim" / "tunnel_site.yaml"
DEFAULT_WORLD_SDF = REPO_ROOT / "sim_worlds" / "phase0_tunnel_gate.sdf"
DEFAULT_UWB_BASE = REPO_ROOT / "configs" / "sensors" / "uwb_pdoa.yaml"
SITE_WORLD = "phase0_tunnel_gate"


def load_site(path: Optional[os.PathLike] = None) -> dict:
    p = Path(path) if path is not None else DEFAULT_SITE
    if not p.is_file():
        raise FileNotFoundError(f"tunnel site file not found: {p}")
    with p.open("r", encoding="utf-8") as f:
        site = yaml.safe_load(f)
    if not isinstance(site, dict):
        raise ValueError(f"{p}: not a mapping")
    for key in ("spawn", "entrance", "mesh"):
        if key not in site:
            raise ValueError(f"{p}: missing {key}")
    return site


def try_load_site(path: Optional[os.PathLike] = None) -> Optional[dict]:
    p = Path(path) if path is not None else DEFAULT_SITE
    if not p.is_file():
        return None
    return load_site(p)


def spawn_xyz(site: dict) -> list:
    return [float(v) for v in site["spawn"]["xyz_m"]]


def spawn_yaw_deg(site: dict) -> float:
    return float(site["spawn"].get("yaw_deg", 0.0))


def hover_height(site: dict, default: float = 0.5) -> float:
    sp = site.get("spawn") or {}
    if "hover_height_m" in sp:
        return float(sp["hover_height_m"])
    xyz = sp.get("xyz_m")
    if xyz is not None:
        return float(xyz[2])
    return float(default)


def entrance_xyz(site: dict) -> list:
    return [float(v) for v in site["entrance"]["position_xyz_m"]
            ] if "position_xyz_m" in site["entrance"] else [
        float(v) for v in site["entrance"]["xyz_m"]
    ]


def entrance_yaw_deg(site: dict) -> float:
    return float(site["entrance"].get("yaw_deg", 0.0))


# ── launch pad (deployment assumption) ───────────────────────────────────────
# Real SAR deployments launch from a staging pad, and a real Crazyflie
# calibrates its gyro sitting still on the ground; the tunnel site models
# that instead of dropping drones onto bumpy lava rock from a hover.
PAD_MARGIN_M = 0.5
PAD_CEILING_CLEARANCE_M = 2.0
PAD_BOTTOM_MARGIN_M = 0.05
PAD_GRID_SPACING_M = 0.1
# body_collision box <size>0.10 0.10 0.03</size> centered at the model origin
# in firmware_mods/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/
# simulator_files/gazebo/models/crazyflie/model.sdf.jinja (base_link, no link
# <pose> so it sits at the model/spawn origin) -> the collision bottom is
# 0.03/2 = 0.015 m below the spawn z used in the gz spawn request.
CF_COLLISION_HALF_HEIGHT_M = 0.015
PAD_REST_CLEARANCE_M = 0.005


def _rotate_xy(x: float, y: float, yaw_rad: float) -> Tuple[float, float]:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return x * c - y * s, x * s + y * c


def site_frame_to_world_xy(x: float, y: float, site: dict) -> Tuple[float, float]:
    ox, oy, _ = spawn_xyz(site)
    yaw = math.radians(spawn_yaw_deg(site))
    rx, ry = _rotate_xy(float(x), float(y), yaw)
    return ox + rx, oy + ry


def pad_footprint_local_bounds(margin_m: float = PAD_MARGIN_M) -> Tuple[float, float, float, float]:
    """(xmin, xmax, ymin, ymax) union of every tunnel scenario layout (site
    frame, spawn origin, no rotation applied yet), expanded by margin_m.
    """
    from swarm_loc_scenarios import SCENARIOS, spawn_xy as _spawn_xy

    xs: list = []
    ys: list = []
    for spec in SCENARIOS.values():
        if spec.get("env") != "tunnel":
            continue
        for x, y in _spawn_xy(spec):
            xs.append(float(x))
            ys.append(float(y))
    if not xs:
        raise ValueError("no tunnel scenario layouts found to derive a launch pad footprint")
    m = float(margin_m)
    return (min(xs) - m, max(xs) + m, min(ys) - m, max(ys) + m)


def pad_footprint_world(site: dict, margin_m: float = PAD_MARGIN_M) -> dict:
    """Footprint box in world XY: site spawn xy offset, site spawn yaw rotated."""
    xmin, xmax, ymin, ymax = pad_footprint_local_bounds(margin_m)
    cx_l, cy_l = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
    sx, sy = xmax - xmin, ymax - ymin
    ox, oy, _ = spawn_xyz(site)
    yaw = math.radians(spawn_yaw_deg(site))
    rx, ry = _rotate_xy(cx_l, cy_l, yaw)
    return {
        "center_xy_m": [float(ox + rx), float(oy + ry)],
        "size_xy_m": [float(sx), float(sy)],
        "yaw_deg": float(math.degrees(yaw)),
        "margin_m": float(margin_m),
    }


def pad_rest_positions(site: dict, layout_xy) -> list:
    """Per-drone rest pose on the pad: [x, y, top_z + collision-bottom-offset
    + clearance], resting (not dropped). Raises if site has no launch_pad.
    """
    pad = site.get("launch_pad")
    if not pad:
        raise ValueError("site has no launch_pad — survey with find_tunnel_site.py first")
    z = float(pad["top_z_m"]) + CF_COLLISION_HALF_HEIGHT_M + PAD_REST_CLEARANCE_M
    out = []
    for x, y in layout_xy:
        wx, wy = site_frame_to_world_xy(x, y, site)
        out.append([float(wx), float(wy), float(z)])
    return out


def spawn_positions_arg(positions) -> str:
    """Format [[x,y,z], ...] as the phase0_gate.sh --spawn-positions string."""
    return ";".join(f"{x:.6f},{y:.6f},{z:.6f}" for x, y, z in positions)


def apply_pad_to_estimator(cfg: dict, layout_xy, site: dict) -> dict:
    """Write the SHARED INTERFACE launch pad keys (D-tunnel-pad).

    Only called when site has a launch_pad. Overrides launch.positions_xyz_m
    with the surveyed takeoff point directly above each pad spot (world
    frame); the swarm EKF initializes there. 2D RIO has no vertical odometry,
    so takeoff from the pad to hover is not observed by RIO and is outside
    the scored window.
    """
    pad = site.get("launch_pad")
    if not pad:
        return cfg
    cfg = copy.deepcopy(cfg)
    top_z = float(pad["top_z_m"])
    hover_above = float(pad.get("hover_above_pad_m", hover_height(site)))
    z = top_z + hover_above
    positions = []
    for x, y in layout_xy:
        wx, wy = site_frame_to_world_xy(x, y, site)
        positions.append([float(wx), float(wy), float(z)])
    cfg.setdefault("launch", {})
    cfg["launch"]["spawned_at_layout"] = True
    cfg["launch"]["pad_top_z_m"] = top_z
    cfg["launch"]["hover_above_pad_m"] = hover_above
    cfg["launch"]["positions_xyz_m"] = positions
    return cfg


def apply_site_to_estimator(cfg: dict, layout_xy, site: dict, hover: Optional[float] = None) -> dict:
    """Offset a scenario layout by the site spawn and write the entrance gauge.

    layout_xy is a list of (x, y) in the site frame (formation about the
    spawn origin). Never reads uwb_pdoa.yaml.
    """
    cfg = copy.deepcopy(cfg)
    ox, oy, oz = spawn_xyz(site)
    z = float(hover) if hover is not None else hover_height(site, oz)
    yaw = spawn_yaw_deg(site)
    cfg.setdefault("launch", {})
    cfg["launch"]["spawn_x0_m"] = ox
    cfg["launch"]["spawn_y_m"] = oy
    cfg["launch"]["spawn_z_m"] = z
    cfg["launch"]["init_yaw_deg"] = yaw
    cfg["launch"]["positions_xyz_m"] = [
        [float(x) + ox, float(y) + oy, z] for (x, y) in layout_xy
    ]
    cfg.setdefault("entrance", {})
    cfg["entrance"]["position_xyz_m"] = entrance_xyz(site)
    cfg["entrance"]["yaw_deg"] = entrance_yaw_deg(site)
    if "device_id" in site["entrance"]:
        cfg["entrance"]["device_id"] = int(site["entrance"]["device_id"])
    return cfg


def apply_site_to_uwb(cfg: dict, site: dict, mesh_path: Optional[str] = None) -> dict:
    """Surveyed entrance + mesh LOS. Raises if mesh requested without a path."""
    cfg = copy.deepcopy(cfg)
    peer = {
        "id": int(site["entrance"].get("device_id", 1000)),
        "type": "entrance",
        "position_xyz_m": entrance_xyz(site),
        "yaw_deg": entrance_yaw_deg(site),
    }
    others = [p for p in cfg.get("static_peers", []) if int(p.get("id", -1)) != peer["id"]]
    cfg["static_peers"] = [peer] + others
    if mesh_path:
        cfg["los_model"] = "mesh"
        cfg["mesh_path"] = str(mesh_path)
        cfg["occluder_boxes"] = []
    return cfg


def write_derived_uwb(
    base_path: os.PathLike,
    out_path: os.PathLike,
    site: dict,
    mesh_path: Optional[str],
) -> Path:
    with open(base_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg = apply_site_to_uwb(cfg, site, mesh_path=mesh_path)
    if cfg.get("los_model") == "mesh" and not cfg.get("mesh_path"):
        raise ValueError("los_model is mesh but mesh_path is empty — refusing silent LOS")
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# DERIVED UWB config — generated by tunnel_site.py.\n"
        "# Entrance pose from configs/sim/tunnel_site.yaml; mesh_path is the\n"
        "# world-frame radar map (same geometry the plugin raycasts).\n"
    )
    with out.open("w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(cfg, f, sort_keys=False)
    return out


def mesh_pose_string(site: dict) -> str:
    xyz = [float(v) for v in site["mesh"]["pose_xyz_m"]]
    rpy = site["mesh"].get("pose_rpy_rad")
    if rpy is None:
        d = [float(v) for v in site["mesh"]["pose_rpy_deg"]]
        rpy = [math.radians(v) for v in d]
    else:
        rpy = [float(v) for v in rpy]
    return f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f} {rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"


def pad_model_sdf(pad: dict) -> str:
    """<model name="launch_pad"> static box, top surface at pad['top_z_m']."""
    cx, cy = [float(v) for v in pad["center_xy_m"]]
    sx, sy = [float(v) for v in pad["size_xy_m"]]
    yaw = math.radians(float(pad["yaw_deg"]))
    thickness = float(pad["thickness_m"])
    top_z = float(pad["top_z_m"])
    cz = top_z - thickness / 2.0
    return (
        '\n    <model name="launch_pad">\n'
        "      <static>true</static>\n"
        f"      <pose>{cx:.6f} {cy:.6f} {cz:.6f} 0 0 {yaw:.6f}</pose>\n"
        '      <link name="link">\n'
        '        <collision name="collision">\n'
        "          <geometry>\n"
        f"            <box><size>{sx:.6f} {sy:.6f} {thickness:.6f}</size></box>\n"
        "          </geometry>\n"
        "        </collision>\n"
        '        <visual name="visual">\n'
        "          <geometry>\n"
        f"            <box><size>{sx:.6f} {sy:.6f} {thickness:.6f}</size></box>\n"
        "          </geometry>\n"
        "          <material>\n"
        "            <ambient>0.5 0.5 0.55 1</ambient>\n"
        "            <diffuse>0.6 0.6 0.65 1</diffuse>\n"
        "          </material>\n"
        "        </visual>\n"
        "      </link>\n"
        "    </model>\n"
    )


def apply_site_to_world_sdf(sdf_path: os.PathLike, site: dict) -> str:
    """Patch tunnel_segment pose; drop the ground plane when site says so;
    add/update the static launch_pad box model (idempotent) when the site has
    a launch_pad block."""
    path = Path(sdf_path)
    text = path.read_text(encoding="utf-8")
    pose = mesh_pose_string(site)
    new_text, n = re.subn(
        r'(<model name="tunnel_segment">\s*<static>true</static>\s*<pose>)[^<]+(</pose>)',
        rf"\g<1>{pose}\2",
        text,
        count=1,
        flags=re.S,
    )
    if n != 1:
        raise ValueError(f"{path}: could not patch tunnel_segment <pose>")
    if not site.get("ground_plane", True) and '<model name="ground_plane">' in new_text:
        new_text, n2 = re.subn(
            r"\n    <model name=\"ground_plane\">.*?</model>\n",
            "\n    <!-- ground_plane removed (tunnel_site.ground_plane=false); "
            "the tube floor is the floor. -->\n",
            new_text,
            count=1,
            flags=re.S,
        )
        if n2 != 1:
            raise ValueError(f"{path}: could not remove ground_plane")
    pad = site.get("launch_pad")
    if pad:
        pad_sdf = pad_model_sdf(pad)
        pad_pattern = re.compile(r'\n    <model name="launch_pad">.*?</model>\n', re.S)
        if pad_pattern.search(new_text):
            new_text = pad_pattern.sub(pad_sdf, new_text, count=1)
        else:
            n3 = new_text.count("</world>")
            if n3 != 1:
                raise ValueError(f"{path}: expected exactly one </world>, found {n3}")
            new_text = new_text.replace("</world>", pad_sdf + "  </world>")
    path.write_text(new_text, encoding="utf-8")
    return pose


def write_site(path: os.PathLike, site: dict) -> Path:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# Surveyed tunnel site. Generated by eval_scripts/find_tunnel_site.py.\n"
        "# The estimator must NEVER read this file (AGENTS.md §1). Derived\n"
        "# swarm_loc yaml carries launch + entrance; derived UWB yaml carries\n"
        "# the simulator entrance peer and mesh LOS path.\n"
    )
    with p.open("w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(site, f, sort_keys=False)
    return p


def run_selftest() -> int:
    ok = True
    n_pass = n_fail = 0

    def check(name: str, cond: bool, detail: str = ""):
        nonlocal ok, n_pass, n_fail
        if cond:
            n_pass += 1
            print(f"[selftest] PASS {name}")
        else:
            ok = False
            n_fail += 1
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))

    site = {
        "world": SITE_WORLD,
        "ground_plane": False,
        "mesh": {
            "uri": "model://cave_world/meshes/lava_tube.obj",
            "pose_xyz_m": [1.25, -2.5, 3.0],
            "pose_rpy_rad": [1.570796, 0.0, 0.3],
        },
        "spawn": {"xyz_m": [0.0, 0.0, 0.5], "yaw_deg": 0.0, "hover_height_m": 0.5},
        "entrance": {
            "xyz_m": [-2.0, 0.0, 0.30],
            "yaw_deg": 0.0,
            "device_id": 1000,
        },
    }
    check("spawn_xyz", spawn_xyz(site) == [0.0, 0.0, 0.5])
    check("hover", abs(hover_height(site) - 0.5) < 1e-12)
    check("entrance", entrance_xyz(site) == [-2.0, 0.0, 0.30])
    pose = mesh_pose_string(site)
    check("pose string has translation", pose.startswith("1.250000 -2.500000 3.000000"))

    est = apply_site_to_estimator(
        {"launch": {"spacing_m": 1.5}, "entrance": {"position_xyz_m": [0, 0, 0], "yaw_deg": 90.0}},
        [(0.0, 0.45), (0.8, 0.0), (0.0, -0.45)],
        site,
    )
    check("est positions offset by spawn (origin)",
          est["launch"]["positions_xyz_m"][0] == [0.0, 0.45, 0.5])
    check("est entrance from site not stock",
          est["entrance"]["position_xyz_m"] == [-2.0, 0.0, 0.30]
          and abs(float(est["entrance"]["yaw_deg"])) < 1e-12)
    site_off = copy.deepcopy(site)
    site_off["spawn"]["xyz_m"] = [10.0, -4.0, 0.6]
    site_off["spawn"]["hover_height_m"] = 0.6
    est2 = apply_site_to_estimator({"launch": {}, "entrance": {}}, [(0.0, 0.0)], site_off)
    check("est spawn offset applied",
          est2["launch"]["positions_xyz_m"][0] == [10.0, -4.0, 0.6]
          and abs(est2["launch"]["spawn_x0_m"] - 10.0) < 1e-12)

    uwb = apply_site_to_uwb(
        {"static_peers": [{"id": 1000, "type": "entrance", "position_xyz_m": [0, 0, 0], "yaw_deg": 0}],
         "los_model": "boxes", "occluder_boxes": [{"name": "old"}]},
        site,
        mesh_path="/tmp/world_frame.obj",
    )
    check("uwb entrance replaced", uwb["static_peers"][0]["position_xyz_m"] == [-2.0, 0.0, 0.30])
    check("uwb mesh los", uwb["los_model"] == "mesh" and uwb["mesh_path"] == "/tmp/world_frame.obj")
    check("uwb boxes cleared", uwb["occluder_boxes"] == [])

    # ── launch pad (deployment assumption) ──────────────────────────────────
    site_pad = copy.deepcopy(site)
    site_pad["launch_pad"] = {
        "center_xy_m": [0.5, 0.0],
        "size_xy_m": [2.0, 1.5],
        "yaw_deg": 0.0,
        "top_z_m": 0.05,
        "thickness_m": 0.1,
        "hover_above_pad_m": 0.5,
    }
    pad_est = apply_pad_to_estimator({"launch": {}}, [(0.0, 0.45), (0.8, 0.0)], site_pad)
    check("pad estimator spawned_at_layout", pad_est["launch"]["spawned_at_layout"] is True)
    check("pad estimator pad_top_z_m", abs(pad_est["launch"]["pad_top_z_m"] - 0.05) < 1e-12)
    check("pad estimator z = top + hover_above", abs(pad_est["launch"]["positions_xyz_m"][0][2] - 0.55) < 1e-9,
          str(pad_est["launch"]["positions_xyz_m"]))
    check("pad estimator xy unrotated at yaw0",
          pad_est["launch"]["positions_xyz_m"][0][:2] == [0.0, 0.45])

    no_pad = apply_pad_to_estimator({"launch": {"positions_xyz_m": [[9, 9, 9]]}}, [(0.0, 0.0)], site)
    check("no launch_pad -> cfg unchanged",
          no_pad["launch"]["positions_xyz_m"] == [[9, 9, 9]] and "spawned_at_layout" not in no_pad["launch"])

    rest = pad_rest_positions(site_pad, [(0.0, 0.45), (0.8, 0.0)])
    expect_rest_z = 0.05 + CF_COLLISION_HALF_HEIGHT_M + PAD_REST_CLEARANCE_M
    check("pad rest z = top + collision-bottom-offset + clearance",
          abs(rest[0][2] - expect_rest_z) < 1e-9, str(rest))
    check("spawn_positions_arg formats x,y,z;x,y,z",
          spawn_positions_arg(rest) ==
          f"0.000000,0.450000,{expect_rest_z:.6f};0.800000,0.000000,{expect_rest_z:.6f}",
          spawn_positions_arg(rest))
    try:
        pad_rest_positions(site, [(0.0, 0.0)])
        check("pad_rest_positions refuses without launch_pad", False)
    except ValueError:
        check("pad_rest_positions refuses without launch_pad", True)

    site_yaw = copy.deepcopy(site)
    site_yaw["spawn"] = {"xyz_m": [0.0, 0.0, 0.5], "yaw_deg": 90.0, "hover_height_m": 0.5}
    wx, wy = site_frame_to_world_xy(1.0, 0.0, site_yaw)
    check("site_frame_to_world_xy rotates by spawn yaw",
          abs(wx) < 1e-9 and abs(wy - 1.0) < 1e-9, f"({wx}, {wy})")

    fp = pad_footprint_world(site)
    check("pad footprint from real tunnel scenarios has positive size",
          fp["size_xy_m"][0] > 0 and fp["size_xy_m"][1] > 0, str(fp))
    check("pad footprint margin default 0.5",
          abs(fp.get("margin_m", PAD_MARGIN_M) - PAD_MARGIN_M) < 1e-12)

    import tempfile
    sdf = (
        '<?xml version="1.0" ?>\n<sdf version="1.9">\n  <world name="phase0_tunnel_gate">\n'
        '    <model name="ground_plane">\n      <static>true</static>\n'
        '      <link name="link"></link>\n    </model>\n'
        '    <model name="tunnel_segment">\n      <static>true</static>\n'
        '      <pose>0 0 0 1.570796 0 0</pose>\n    </model>\n  </world>\n</sdf>\n'
    )
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "w.sdf"
        p.write_text(sdf, encoding="utf-8")
        apply_site_to_world_sdf(p, site)
        out = p.read_text(encoding="utf-8")
        check("sdf pose patched", pose in out)
        check("sdf ground plane removed", "ground_plane" not in out or "removed" in out)
        check("sdf tunnel_segment kept", "tunnel_segment" in out)
        check("sdf no launch_pad without site.launch_pad", "launch_pad" not in out)

        p2 = Path(td) / "w_pad.sdf"
        p2.write_text(sdf, encoding="utf-8")
        apply_site_to_world_sdf(p2, site_pad)
        out_pad1 = p2.read_text(encoding="utf-8")
        check("launch_pad model inserted once", out_pad1.count('<model name="launch_pad">') == 1, out_pad1)
        check("launch_pad box size matches pad size_xy_m/thickness",
              "2.000000 1.500000 0.100000" in out_pad1, out_pad1)
        apply_site_to_world_sdf(p2, site_pad)
        out_pad2 = p2.read_text(encoding="utf-8")
        check("launch_pad apply is idempotent (re-apply leaves exactly one model)",
              out_pad2.count('<model name="launch_pad">') == 1, out_pad2)
        check("re-apply keeps tunnel_segment and ground-plane removal intact",
              out_pad2.count("tunnel_segment") == 1 and "removed" in out_pad2)

        base = {
            "los_model": "boxes",
            "occluder_boxes": [],
            "mesh_path": "",
            "static_peers": [{"id": 1000, "type": "entrance", "position_xyz_m": [9, 9, 9], "yaw_deg": 1}],
            "seed": 0,
        }
        bp = Path(td) / "uwb.yaml"
        bp.write_text(yaml.safe_dump(base), encoding="utf-8")
        outp = Path(td) / "uwb_derived.yaml"
        write_derived_uwb(bp, outp, site, mesh_path="/abs/map.obj")
        d = yaml.safe_load(outp.read_text(encoding="utf-8"))
        check("derived uwb mesh", d["los_model"] == "mesh" and d["mesh_path"] == "/abs/map.obj")
        try:
            write_derived_uwb(bp, Path(td) / "bad.yaml", site, mesh_path="")
            # apply_site_to_uwb without mesh_path leaves boxes — write allows that
            check("derive without mesh_path stays boxes", True)
        except ValueError:
            check("derive without mesh_path stays boxes", False)

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--site", default=str(DEFAULT_SITE))
    ap.add_argument("--derive-uwb", action="store_true")
    ap.add_argument("--base", default=str(DEFAULT_UWB_BASE))
    ap.add_argument("--mesh-path", default="")
    ap.add_argument("--out", default="")
    ap.add_argument("--apply-sdf", action="store_true")
    ap.add_argument("--sdf", default=str(DEFAULT_WORLD_SDF))
    args = ap.parse_args()
    if args.selftest:
        raise SystemExit(run_selftest())
    site = load_site(args.site)
    if args.derive_uwb:
        if not args.out:
            ap.error("--derive-uwb requires --out")
        p = write_derived_uwb(args.base, args.out, site, args.mesh_path or None)
        print(p)
        return
    if args.apply_sdf:
        pose = apply_site_to_world_sdf(args.sdf, site)
        print(pose)
        return
    ap.print_help()
    raise SystemExit(2)


if __name__ == "__main__":
    main()
