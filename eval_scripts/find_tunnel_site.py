#!/usr/bin/env python3
"""find_tunnel_site.py — find a flyable cell in lava_tube.obj and write tunnel_site.yaml.

A cell is accepted only if a downward ray hits a floor within --floor-max-m,
upward clearance is at least --clearance-m, and a forward ray hits a wall
within --forward-max-m (lateral Doppler will exist). The recommended mesh
pose translates/yaws that cell to the world origin with +x down-tunnel so
existing formation_forward motions stay body +x.

Usage:
    python3 eval_scripts/find_tunnel_site.py --selftest
    python3 eval_scripts/find_tunnel_site.py --write --apply-sdf
"""
from __future__ import annotations

import argparse
import datetime
import math
import os
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT / "eval_scripts") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "eval_scripts"))

from build_radar_map import load_obj, pose_matrix, transform_points  # noqa: E402
from tunnel_site import (  # noqa: E402
    DEFAULT_SITE,
    DEFAULT_WORLD_SDF,
    PAD_BOTTOM_MARGIN_M,
    PAD_CEILING_CLEARANCE_M,
    PAD_GRID_SPACING_M,
    PAD_MARGIN_M,
    apply_site_to_world_sdf,
    entrance_xyz,
    hover_height,
    pad_footprint_world,
    write_site,
)

def _repo_rel(path) -> str:
    p = Path(path).resolve()
    try:
        return str(p.relative_to(REPO_ROOT)).replace("\\", "/")
    except ValueError:
        return str(p)


DEFAULT_MESH = (
    REPO_ROOT / "sim_worlds" / "darpa_subt_worlds" / "worlds" / "models"
    / "cave_world" / "meshes" / "lava_tube.obj"
)
# Current phase0_tunnel_gate roll (no translation).
BASE_ROLL = math.pi / 2


def _try_trimesh_intersector(vertices: np.ndarray, faces: np.ndarray):
    try:
        import trimesh
        from trimesh.ray.ray_pyembree import RayMeshIntersector
    except ImportError as exc:
        raise RuntimeError(
            "trimesh/embreex required to survey the lava_tube mesh. "
            "pip install trimesh embreex"
        ) from exc
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    return RayMeshIntersector(mesh)


def ray_hits(intersector, origins, directions, max_dist: float) -> np.ndarray:
    """Nearest hit distance per ray, or +inf. origins/directions (N,3)."""
    origins = np.asarray(origins, dtype=np.float64).reshape(-1, 3)
    directions = np.asarray(directions, dtype=np.float64).reshape(-1, 3)
    n = origins.shape[0]
    out = np.full(n, np.inf)
    loc, idx_ray, _ = intersector.intersects_location(
        origins, directions, multiple_hits=False
    )
    if len(idx_ray) == 0:
        return out
    d = np.linalg.norm(loc - origins[np.asarray(idx_ray)], axis=1)
    for i, dist in zip(idx_ray, d):
        if dist <= max_dist and dist < out[int(i)]:
            out[int(i)] = float(dist)
    return out


def heading_yaw_from_xy(dx: float, dy: float) -> float:
    return math.atan2(dy, dx)


def site_from_cell(cell_xyz, heading_xy, hover_m: float, entrance_back_m: float,
                   mesh_uri: str, mesh_path: str, extra: dict) -> dict:
    """Mesh pose that maps cell floor to the origin, +x = heading."""
    c = np.asarray(cell_xyz, dtype=float)
    h = np.asarray(heading_xy, dtype=float)
    h = h / (np.linalg.norm(h) + 1e-15)
    alpha = -heading_yaw_from_xy(float(h[0]), float(h[1]))
    rz = np.array([
        [math.cos(alpha), -math.sin(alpha), 0.0],
        [math.sin(alpha), math.cos(alpha), 0.0],
        [0.0, 0.0, 1.0],
    ])
    t = -rz @ c
    return {
        "world": "phase0_tunnel_gate",
        "ground_plane": False,
        "mesh": {
            "uri": mesh_uri,
            "src": _repo_rel(mesh_path),
            "pose_xyz_m": [float(t[0]), float(t[1]), float(t[2])],
            "pose_rpy_rad": [BASE_ROLL, 0.0, float(alpha)],
            "pose_rpy_deg": [90.0, 0.0, float(math.degrees(alpha))],
        },
        "spawn": {
            "xyz_m": [0.0, 0.0, float(hover_m)],
            "yaw_deg": 0.0,
            "hover_height_m": float(hover_m),
        },
        "entrance": {
            "xyz_m": [-float(entrance_back_m), 0.0, 0.30],
            "yaw_deg": 0.0,
            "device_id": 1000,
        },
        "survey": extra,
    }


def survey(
    mesh_path: Path,
    grid_m: float = 4.0,
    hover_m: float = 0.5,
    floor_max_m: float = 2.0,
    clearance_m: float = 2.0,
    forward_min_m: float = 3.0,
    forward_max_m: float = 25.0,
    side_min_m: float = 1.2,
    side_max_m: float = 8.0,
    entrance_back_m: float = 2.0,
) -> dict:
    v, f = load_obj(str(mesh_path))
    T = pose_matrix(0.0, 0.0, 0.0, BASE_ROLL, 0.0, 0.0)
    vw = transform_points(T, v)
    intersector = _try_trimesh_intersector(vw, f)
    bmin = vw.min(axis=0)
    bmax = vw.max(axis=0)
    inset = max(2.0, grid_m)
    xs = np.arange(bmin[0] + inset, bmax[0] - inset, grid_m)
    ys = np.arange(bmin[1] + inset, bmax[1] - inset, grid_m)
    if xs.size == 0 or ys.size == 0:
        raise RuntimeError(
            f"mesh AABB too small for inset={inset} m "
            f"(span x={bmax[0]-bmin[0]:.1f} y={bmax[1]-bmin[1]:.1f})"
        )
    xx, yy = np.meshgrid(xs, ys, indexing="xy")
    cols = np.column_stack([xx.ravel(), yy.ravel()])
    z_top = float(bmax[2]) - 1.0
    origins = np.column_stack([cols, np.full(len(cols), z_top)])
    down = np.tile(np.array([[0.0, 0.0, -1.0]]), (len(cols), 1))
    hit = ray_hits(intersector, origins, down, float(bmax[2] - bmin[2]) + 4.0)
    finite = np.isfinite(hit)
    if not finite.any():
        raise RuntimeError("no downward hits — mesh may be empty after the roll")

    headings = [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)]
    best = None
    for (x, y), dist in zip(cols[finite], hit[finite]):
        floor_z = z_top - float(dist)
        probe = np.array([x, y, floor_z + hover_m])
        up = ray_hits(intersector, probe, [[0.0, 0.0, 1.0]], 20.0)[0]
        dn = ray_hits(intersector, probe, [[0.0, 0.0, -1.0]], floor_max_m + 0.2)[0]
        if not math.isfinite(dn) or dn > floor_max_m:
            continue
        if not math.isfinite(up) or up < clearance_m:
            continue
        walls = []
        for hx, hy in headings:
            d = ray_hits(
                intersector, probe, [[hx, hy, 0.0]], forward_max_m
            )[0]
            walls.append((d, hx, hy))
        walls.sort(key=lambda w: w[0])
        # Need at least one corridor-ish hit in [forward_min, forward_max]
        # and two sides in [side_min, side_max].
        sides = [w for w in walls if side_min_m <= w[0] <= side_max_m]
        forwards = [w for w in walls if forward_min_m <= w[0] <= forward_max_m]
        if len(sides) < 2 or not forwards:
            continue
        # Longest finite wall among the four is the "down-tunnel" guess if
        # it is also a forward; else the farthest forward.
        heading = max(forwards, key=lambda w: w[0])
        # Prefer a heading whose opposite side is also a wall (tube, not alcove).
        opp = (-heading[1], -heading[2])
        opp_d = next((w[0] for w in walls if (w[1], w[2]) == opp), np.inf)
        score = (len(sides), heading[0], -abs(opp_d - heading[0]) if math.isfinite(opp_d) else -1e3)
        rec = {
            "xyz": (float(x), float(y), float(floor_z)),
            "heading": (heading[1], heading[2]),
            "score": score,
            "floor_hit_m": float(dn),
            "ceiling_m": float(up),
            "walls_m": {f"{hx:+.0f},{hy:+.0f}": float(d) for d, hx, hy in walls},
        }
        if best is None or rec["score"] > best["score"]:
            best = rec
    if best is None:
        raise RuntimeError(
            "no flyable cell (floor + clearance + two side walls + a forward wall). "
            "The mesh may not contain a corridor at this roll."
        )
    extra = {
        "mesh_path": _repo_rel(mesh_path),
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "cell_in_rolled_world_m": list(best["xyz"]),
        "heading_xy": list(best["heading"]),
        "floor_hit_m": best["floor_hit_m"],
        "ceiling_m": best["ceiling_m"],
        "walls_m": best["walls_m"],
        "grid_m": float(grid_m),
        "rule": (
            "down hits floor within floor_max_m; up clearance >= clearance_m; "
            "two side walls in [side_min, side_max]; one forward wall in "
            "[forward_min, forward_max]; mesh translated/yawed so the cell "
            "floor is the origin and +x is down-tunnel"
        ),
    }
    return site_from_cell(
        best["xyz"], best["heading"], hover_m, entrance_back_m,
        "model://cave_world/meshes/lava_tube.obj", str(mesh_path), extra,
    )


def _mesh_pose_matrix(site: dict) -> np.ndarray:
    xyz = [float(v) for v in site["mesh"]["pose_xyz_m"]]
    rpy = [float(v) for v in site["mesh"]["pose_rpy_rad"]]
    return pose_matrix(xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2])


def survey_launch_pad(
    mesh_path: Path,
    site: dict,
    margin_m: float = PAD_MARGIN_M,
    grid_m: float = PAD_GRID_SPACING_M,
    ceiling_clearance_m: float = PAD_CEILING_CLEARANCE_M,
    bottom_margin_m: float = PAD_BOTTOM_MARGIN_M,
    floor_max_m: float = 2.0,
    probe_above_floor_m: float = 0.3,
    punch_through_max_m: float = 30.0,
) -> dict:
    """Raycast the lava_tube mesh AS PLACED in the world (site['mesh'] pose) to
    find a flat launch pad under the union footprint of every tunnel scenario
    layout. Refuses (raises RuntimeError) if any grid ray misses the floor, if
    the pad would intersect a wall inside the footprint, or if ceiling
    clearance above the pad top is below ceiling_clearance_m.

    lava_tube.obj is a whole cave complex: a single downward ray from near the
    mesh's overall bounding-box top can land on the OUTSIDE of the local
    ceiling shell (the nearest solid surface from way above) rather than the
    floor below it — confirmed on the real site, where grid points at the
    footprint's edge returned the (correct) local ceiling height as "floor".
    So each grid point is "punched through": the first downward hit is
    treated as provisional; a second downward ray from just below it either
    finds the true floor further down (first hit was the ceiling) or nothing
    within punch_through_max_m (first hit already was the floor). Only then
    is the existing refined probe (+/- floor_max_m / ceiling_clearance_m)
    applied, anchored just above the resolved floor.
    """
    footprint = pad_footprint_world(site, margin_m)
    cx, cy = footprint["center_xy_m"]
    sx, sy = footprint["size_xy_m"]
    yaw = math.radians(footprint["yaw_deg"])
    c, s = math.cos(yaw), math.sin(yaw)

    v, f = load_obj(str(mesh_path))
    vw = transform_points(_mesh_pose_matrix(site), v)
    intersector = _try_trimesh_intersector(vw, f)

    nx = max(2, int(math.ceil(sx / grid_m)) + 1)
    ny = max(2, int(math.ceil(sy / grid_m)) + 1)
    xs = np.linspace(-sx / 2.0, sx / 2.0, nx)
    ys = np.linspace(-sy / 2.0, sy / 2.0, ny)
    xx, yy = np.meshgrid(xs, ys, indexing="xy")
    local = np.column_stack([xx.ravel(), yy.ravel()])
    wx = cx + local[:, 0] * c - local[:, 1] * s
    wy = cy + local[:, 0] * s + local[:, 1] * c
    n_grid = len(wx)

    bmin = vw.min(axis=0)
    bmax = vw.max(axis=0)
    z_top = float(bmax[2]) + 1.0
    down = np.array([0.0, 0.0, -1.0])
    up = np.array([0.0, 0.0, 1.0])
    full_span = float(bmax[2] - bmin[2]) + 4.0

    origins = np.column_stack([wx, wy, np.full(n_grid, z_top)])
    first_hit = ray_hits(intersector, origins, np.tile(down, (n_grid, 1)), full_span)
    n_miss = int(np.sum(~np.isfinite(first_hit)))
    if n_miss:
        raise RuntimeError(
            f"launch pad survey: {n_miss}/{n_grid} downward grid rays missed the "
            f"floor over footprint center=({cx:.2f},{cy:.2f}) size=({sx:.2f}x{sy:.2f})"
        )
    first_z = z_top - first_hit

    punch_origins = np.column_stack([wx, wy, first_z - 0.05])
    punch_hit = ray_hits(intersector, punch_origins, np.tile(down, (n_grid, 1)), punch_through_max_m)
    approx_floor = np.where(np.isfinite(punch_hit), (first_z - 0.05) - punch_hit, first_z)

    probe_z = approx_floor + probe_above_floor_m
    probe = np.column_stack([wx, wy, probe_z])
    dn = ray_hits(intersector, probe, np.tile(down, (n_grid, 1)), floor_max_m + 0.2)
    n_dn_miss = int(np.sum(~np.isfinite(dn)))
    if n_dn_miss:
        raise RuntimeError(
            f"launch pad survey: {n_dn_miss}/{n_grid} refined floor rays missed "
            f"within {floor_max_m:.2f} m of the approximate floor"
        )
    floor_z = probe_z - dn
    floor_min = float(floor_z.min())
    floor_max = float(floor_z.max())
    floor_median = float(np.median(floor_z))
    top_z = floor_max + 0.01
    thickness = (top_z - floor_min) + bottom_margin_m
    bottom_z = top_z - thickness

    up_hit = ray_hits(intersector, probe, np.tile(up, (n_grid, 1)), ceiling_clearance_m + 40.0)
    n_up_miss = int(np.sum(~np.isfinite(up_hit)))
    if n_up_miss:
        raise RuntimeError(
            f"launch pad survey: {n_up_miss}/{n_grid} ceiling rays found nothing "
            f"within {ceiling_clearance_m + 40.0:.1f} m above the pad"
        )
    ceiling_z = probe_z + up_hit
    clearance = float((ceiling_z - top_z).min())
    if clearance < ceiling_clearance_m:
        raise RuntimeError(
            f"launch pad ceiling clearance {clearance:.2f} m < required "
            f"{ceiling_clearance_m:.2f} m"
        )

    half_sx, half_sy = sx / 2.0, sy / 2.0
    mid_z = 0.5 * (top_z + floor_median)
    centers = np.tile(np.array([[cx, cy, mid_z]]), (4, 1))
    dirs = np.array([
        [c, s, 0.0], [-c, -s, 0.0], [-s, c, 0.0], [s, -c, 0.0],
    ])
    needed = [half_sx, half_sx, half_sy, half_sy]
    wall_hit = ray_hits(intersector, centers, dirs, max(half_sx, half_sy) + 1.0)
    for need, got, label in zip(needed, wall_hit, ("+x", "-x", "+y", "-y")):
        if math.isfinite(got) and got < need - 1e-6:
            raise RuntimeError(
                f"launch pad footprint intersects a wall (local {label}: "
                f"wall at {got:.2f} m < half-extent {need:.2f} m)"
            )

    ent = entrance_xyz(site)
    edx, edy = ent[0] - cx, ent[1] - cy
    lx = edx * c + edy * s
    ly = -edx * s + edy * c
    if abs(lx) <= half_sx and abs(ly) <= half_sy:
        raise RuntimeError(
            f"entrance node ({ent[0]:.2f},{ent[1]:.2f}) lies inside the launch "
            f"pad footprint (half-extents {half_sx:.2f}x{half_sy:.2f})"
        )
    entrance_height_above_pad = float(ent[2]) - top_z
    if entrance_height_above_pad <= 0:
        raise RuntimeError(
            f"entrance node z={ent[2]:.2f} m is not above pad top {top_z:.2f} m"
        )

    return {
        "center_xy_m": [float(cx), float(cy)],
        "size_xy_m": [float(sx), float(sy)],
        "yaw_deg": float(math.degrees(yaw)),
        "margin_m": float(margin_m),
        "floor_min_m": floor_min,
        "floor_max_m": floor_max,
        "floor_median_m": floor_median,
        "top_z_m": float(top_z),
        "thickness_m": float(thickness),
        "bottom_z_m": float(bottom_z),
        "ceiling_clearance_m": clearance,
        "entrance_height_above_pad_m": entrance_height_above_pad,
        "grid_m": float(grid_m),
        "hover_above_pad_m": hover_height(site),
    }


def _corridor_obj(path: Path) -> None:
    """Axis-aligned corridor: floor z=0, walls y=±2.5, ceiling z=3, x=0..20."""
    # Two triangles per face.
    faces = []
    verts = []

    def quad(a, b, c, d):
        i = len(verts)
        verts.extend((a, b, c, d))
        faces.append((i, i + 1, i + 2))
        faces.append((i, i + 2, i + 3))

    quad((0, -3, 0), (20, -3, 0), (20, 3, 0), (0, 3, 0))          # floor
    quad((0, -3, 3), (0, 3, 3), (20, 3, 3), (20, -3, 3))          # ceiling
    quad((0, -2.5, 0), (20, -2.5, 0), (20, -2.5, 3), (0, -2.5, 3))  # -y wall
    quad((0, 2.5, 0), (0, 2.5, 3), (20, 2.5, 3), (20, 2.5, 0))     # +y wall
    # End wall at x=20 so forward +x hits.
    quad((20, -2.5, 0), (20, 2.5, 0), (20, 2.5, 3), (20, -2.5, 3))
    lines = [f"v {p[0]} {p[1]} {p[2]}" for p in verts]
    for a, b, c in faces:
        lines.append(f"f {a + 1} {b + 1} {c + 1}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _flat_pad_obj(path: Path, x_max: float = 3.5, y_half: float = 1.0,
                   ceiling_z: float = 3.0, wall_x: Optional[float] = None) -> None:
    """Flat floor (z=0) + flat ceiling, x in [-1, x_max], y in [-y_half, y_half].
    Optional vertical wall at x=wall_x spanning the full y/low-z range."""
    faces = []
    verts = []

    def quad(a, b, c, d):
        i = len(verts)
        verts.extend((a, b, c, d))
        faces.append((i, i + 1, i + 2))
        faces.append((i, i + 2, i + 3))

    quad((-1, -y_half, 0), (x_max, -y_half, 0), (x_max, y_half, 0), (-1, y_half, 0))
    quad((-1, -y_half, ceiling_z), (-1, y_half, ceiling_z),
         (x_max, y_half, ceiling_z), (x_max, -y_half, ceiling_z))
    if wall_x is not None:
        quad((wall_x, -y_half, 0), (wall_x, y_half, 0),
             (wall_x, y_half, 2.0), (wall_x, -y_half, 2.0))
    lines = [f"v {p[0]} {p[1]} {p[2]}" for p in verts]
    for a, b, c in faces:
        lines.append(f"f {a + 1} {b + 1} {c + 1}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


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

    import tempfile
    # site_from_cell: heading +x, cell at (8, -4, 1) → t = (-8, 4, -1), yaw 0
    s = site_from_cell((8.0, -4.0, 1.0), (1.0, 0.0), 0.5, 2.0, "model://x", "/tmp/x.obj", {})
    check("translate x", abs(s["mesh"]["pose_xyz_m"][0] + 8.0) < 1e-9)
    check("translate y", abs(s["mesh"]["pose_xyz_m"][1] - 4.0) < 1e-9)
    check("translate z", abs(s["mesh"]["pose_xyz_m"][2] + 1.0) < 1e-9)
    check("yaw 0 for +x heading", abs(s["mesh"]["pose_rpy_rad"][2]) < 1e-9)
    check("spawn at origin hover", s["spawn"]["xyz_m"] == [0.0, 0.0, 0.5])
    check("entrance 2 m behind", s["entrance"]["xyz_m"] == [-2.0, 0.0, 0.30])
    check("no ground plane", s["ground_plane"] is False)
    # heading +y → yaw = -90 deg so +x becomes old +y
    s2 = site_from_cell((0.0, 0.0, 0.0), (0.0, 1.0), 0.5, 2.0, "model://x", "/tmp/x.obj", {})
    check("yaw -90 for +y heading", abs(s2["mesh"]["pose_rpy_rad"][2] + math.pi / 2) < 1e-9)

    try:
        import trimesh  # noqa: F401
        from trimesh.ray.ray_pyembree import RayMeshIntersector  # noqa: F401
        have = True
    except ImportError:
        have = False
    check("trimesh documented as required for live survey", True)
    if have:
        with tempfile.TemporaryDirectory() as td:
            obj = Path(td) / "corridor.obj"
            _corridor_obj(obj)
            # Survey applies BASE_ROLL to the file. Build the corridor already
            # in the rolled frame by writing vertices as world points and
            # inverting the roll before save... easier: call survey pieces
            # with a pre-rolled mesh via a tiny helper.
            v, f = load_obj(str(obj))
            # The corridor is written in world frame with z-up. survey() will
            # apply Rx(90), which maps (x,y,z) -> (x,-z,y) and destroys it.
            # Invert that roll so after survey's Rx(90) we get the corridor back.
            Tinv = pose_matrix(0.0, 0.0, 0.0, -BASE_ROLL, 0.0, 0.0)
            v_local = transform_points(Tinv, v)
            tmp = Path(td) / "local.obj"
            lines = [f"v {p[0]} {p[1]} {p[2]}" for p in v_local]
            for a, b, c in f:
                lines.append(f"f {a + 1} {b + 1} {c + 1}")
            tmp.write_text("\n".join(lines) + "\n", encoding="utf-8")
            found = survey(tmp, grid_m=2.0, hover_m=0.5, floor_max_m=2.0,
                           clearance_m=1.5, forward_min_m=2.0, forward_max_m=25.0,
                           side_min_m=1.0, side_max_m=8.0)
            check("survey found a cell", "cell_in_rolled_world_m" in found["survey"])
            cell = found["survey"]["cell_in_rolled_world_m"]
            check("survey cell inside corridor x", 0.0 < cell[0] < 20.0, str(cell))
            check("survey cell near y=0", abs(cell[1]) < 2.0, str(cell))
            check("survey floor near z=0", abs(cell[2]) < 0.4, str(cell))
            check("survey spawn origin", found["spawn"]["xyz_m"][0] == 0.0)

            # ── launch pad survey (flat/bumpy synthetic meshes) ─────────────
            pad_site = {
                "mesh": {"pose_xyz_m": [0.0, 0.0, 0.0], "pose_rpy_rad": [0.0, 0.0, 0.0]},
                "spawn": {"xyz_m": [0.0, 0.0, 0.5], "yaw_deg": 0.0, "hover_height_m": 0.5},
                "entrance": {"xyz_m": [-2.0, 0.0, 0.30], "yaw_deg": 0.0, "device_id": 1000},
            }
            flat = Path(td) / "pad_flat.obj"
            _flat_pad_obj(flat, x_max=3.5, y_half=1.0, ceiling_z=3.0)
            pad = survey_launch_pad(flat, pad_site, margin_m=0.0, grid_m=0.5)
            check("pad survey (flat mesh) top_z near floor 0",
                  abs(pad["top_z_m"]) < 0.2, str(pad))
            check("pad survey (flat mesh) ceiling clearance >= 2.0",
                  pad["ceiling_clearance_m"] >= 2.0, str(pad))
            check("pad survey (flat mesh) entrance above pad",
                  pad["entrance_height_above_pad_m"] > 0, str(pad))
            check("pad survey (flat mesh) size matches real tunnel scenarios",
                  abs(pad["size_xy_m"][0] - 3.0) < 1e-6 and abs(pad["size_xy_m"][1] - 0.9) < 1e-6,
                  str(pad))

            bumpy = Path(td) / "pad_bumpy.obj"
            _flat_pad_obj(bumpy, x_max=1.4, y_half=1.0, ceiling_z=3.0)
            try:
                survey_launch_pad(bumpy, pad_site, margin_m=0.0, grid_m=0.5)
                check("pad survey refuses when grid rays miss the floor", False)
            except RuntimeError as e:
                check("pad survey refuses when grid rays miss the floor",
                      "missed" in str(e), str(e))

            low_ceiling = Path(td) / "pad_low_ceiling.obj"
            _flat_pad_obj(low_ceiling, x_max=3.5, y_half=1.0, ceiling_z=1.5)
            try:
                survey_launch_pad(low_ceiling, pad_site, margin_m=0.0, grid_m=0.5)
                check("pad survey refuses on low ceiling clearance", False)
            except RuntimeError as e:
                check("pad survey refuses on low ceiling clearance",
                      "clearance" in str(e), str(e))

            walled = Path(td) / "pad_walled.obj"
            _flat_pad_obj(walled, x_max=3.5, y_half=1.0, ceiling_z=3.0, wall_x=2.1)
            try:
                survey_launch_pad(walled, pad_site, margin_m=0.0, grid_m=0.5)
                check("pad survey refuses when a wall crosses the footprint", False)
            except RuntimeError as e:
                check("pad survey refuses when a wall crosses the footprint",
                      "wall" in str(e), str(e))
    else:
        print("[selftest] SKIP live corridor survey (trimesh/embreex not installed)")

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--mesh", default=str(DEFAULT_MESH))
    ap.add_argument("--write", nargs="?", const=str(DEFAULT_SITE), default="")
    ap.add_argument("--apply-sdf", action="store_true")
    ap.add_argument("--sdf", default=str(DEFAULT_WORLD_SDF))
    ap.add_argument("--grid-m", type=float, default=4.0)
    ap.add_argument("--hover-m", type=float, default=0.5)
    ap.add_argument("--no-pad", action="store_true",
                    help="Skip the launch-pad survey (site has no launch_pad block)")
    ap.add_argument("--pad-grid-m", type=float, default=PAD_GRID_SPACING_M)
    ap.add_argument("--pad-margin-m", type=float, default=PAD_MARGIN_M)
    args = ap.parse_args()
    if args.selftest:
        raise SystemExit(run_selftest())
    site = survey(Path(args.mesh), grid_m=args.grid_m, hover_m=args.hover_m)
    if not args.no_pad:
        try:
            pad = survey_launch_pad(
                Path(args.mesh), site, margin_m=args.pad_margin_m, grid_m=args.pad_grid_m
            )
        except RuntimeError as e:
            print(f"[find_tunnel_site] ERROR: launch pad survey failed: {e}", file=sys.stderr, flush=True)
            return 1
        site["launch_pad"] = pad
    import yaml
    print(yaml.safe_dump(site, sort_keys=False))
    if args.write:
        p = write_site(args.write, site)
        print(f"[find_tunnel_site] wrote {p}", flush=True)
    if args.apply_sdf:
        pose = apply_site_to_world_sdf(args.sdf, site)
        print(f"[find_tunnel_site] patched {args.sdf} pose={pose}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
