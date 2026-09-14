#!/usr/bin/env python3
"""build_radar_map.py — build the radarays_gz2 raycast map from a world SDF.

Why: the radar plugin (perception/radarays_gz2/src/RadarSensorSystem.cpp)
imports `<mesh_path>` with rm::import_embree_map at IDENTITY and raycasts in
WORLD frame (buildCloud uses the sensor's world pose as map->sensor). It never
sees the world SDF, so model poses, scales and primitive shapes (the ground
<plane>) are lost unless the map mesh is already the world's physical geometry
expressed in world coordinates. This script produces exactly that mesh.

What it does:
  1. Parse the world SDF. For every top-level <model> (recursing into nested
     <model>) and <include> (model:// resolved via GZ_SIM_RESOURCE_PATH, its
     <pose> overriding the included model's pose), walk links and take the
     COLLISION geometry (radar reflects off physical surfaces):
       <plane>  finite quad from <size>/<normal>
       <box>    12 triangles
       <mesh>   OBJ (<uri> model://, file:// or path; optional <scale>);
                polygons fan-triangulated, normals/uvs ignored
     Anything else (sphere, cylinder, capsule, heightmap, non-OBJ mesh) is
     warned and skipped by name — never silently dropped.
  2. Poses compose model * link * collision, each SDF `x y z roll pitch yaw`
     with fixed-axis R = Rz(yaw) * Ry(pitch) * Rx(roll) (Gazebo convention;
     roll +90 deg maps (x, y, z) -> (x, -z, y)). `relative_to` is honoured only
     when absent or naming the parent frame; otherwise a warning is printed and
     the pose is still treated as parent-relative.
  3. Crop: flight region box expanded by radar range + margin on every axis.
     A triangle is kept iff its axis-aligned bounds intersect the crop box
     (conservative: no reachable surface is dropped, including big triangles
     whose vertices all lie outside). Planes are clipped exactly to the box
     (Sutherland-Hodgman) because a 200 x 200 m quad would be mostly useless.
  4. Write one world-frame OBJ to <cache-dir>/<world_name>_<hash12>.obj; the
     hash covers the world SDF bytes, every referenced file's path/size/mtime
     (meshes and included model.sdf), the crop box and BUILDER_VERSION. A cached
     file with the same hash is reused without parsing any mesh.

The LAST stdout line is the absolute output path (phase0_gate.sh captures it).
Warnings go to stderr. Exit 0 on success, 2 if the cropped map is empty, 1 on
any other error.

Usage:
    build_radar_map.py WORLD_SDF --region XMIN XMAX YMIN YMAX ZMIN ZMAX
                       [--range-m R] [--margin-m 2.0] [--cache-dir out/radar_maps]
                       [--radar-config configs/sensors/radar_noise.yaml]
                       [--resource-path DIR ...] [--force]
    build_radar_map.py --selftest
"""
import argparse
import hashlib
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

BUILDER_VERSION = "1"
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_RADAR_CONFIG = os.path.join(REPO_ROOT, "configs", "sensors", "radar_noise.yaml")
DEFAULT_CACHE_DIR = os.path.join(REPO_ROOT, "out", "radar_maps")
DEFAULT_MARGIN_M = 2.0
# Same dirs setup_env.sh puts on GZ_SIM_RESOURCE_PATH; used as a fallback so the
# builder resolves model:// even when setup_env.sh was not sourced.
REPO_RESOURCE_PATHS = [
    os.path.join(REPO_ROOT, "sim_worlds"),
    os.path.join(REPO_ROOT, "sim_worlds", "darpa_subt_worlds"),
    os.path.join(REPO_ROOT, "sim_worlds", "darpa_subt_worlds", "worlds"),
    os.path.join(REPO_ROOT, "sim_worlds", "darpa_subt_worlds", "worlds", "models"),
]
UNSUPPORTED_GEOMETRY = ("sphere", "cylinder", "capsule", "ellipsoid", "cone",
                        "heightmap", "polyline", "image", "empty")


def log(msg):
    print(f"[build_radar_map] {msg}", flush=True)


class EmptyMapError(RuntimeError):
    pass


# ── poses ────────────────────────────────────────────────────────────────────
def pose_matrix(x, y, z, roll, pitch, yaw):
    """4x4 homogeneous transform; R = Rz(yaw) @ Ry(pitch) @ Rx(roll) (Gazebo)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    t = np.eye(4)
    t[:3, :3] = rz @ ry @ rx
    t[:3, 3] = (x, y, z)
    return t


def transform_points(t, pts):
    pts = np.asarray(pts, dtype=float)
    return pts @ t[:3, :3].T + t[:3, 3]


def _floats(text, n, what, warnings):
    vals = [float(v) for v in (text or "").split()]
    if len(vals) != n:
        raise ValueError(f"{what}: expected {n} numbers, got {text!r}")
    return vals


class _Ctx:
    def __init__(self, resource_paths):
        self.resource_paths = list(resource_paths)
        self.sources = []
        self.deps = []        # files whose path/size/mtime enter the cache hash
        self.warnings = []

    def warn(self, msg):
        self.warnings.append(msg)
        print(f"[build_radar_map] WARN: {msg}", file=sys.stderr, flush=True)


def _pose_of(elem, parent_names, scope, ctx):
    """Parse elem's <pose> child (parent-relative). Identity if absent."""
    p = elem.find("pose")
    if p is None:
        return np.eye(4)
    rel = p.get("relative_to")
    if rel not in (None, "") and rel not in parent_names:
        ctx.warn(f"{scope}: pose relative_to='{rel}' is not the parent frame "
                 f"({parent_names[0]}); treating it as parent-relative anyway")
    if p.get("rotation_format", "euler_rpy") != "euler_rpy":
        raise ValueError(f"{scope}: pose rotation_format={p.get('rotation_format')} unsupported")
    v = _floats(p.text, 6, f"{scope} <pose>", ctx.warnings)
    if str(p.get("degrees", "false")).lower() in ("true", "1"):
        v[3:] = [math.radians(a) for a in v[3:]]
    return pose_matrix(*v)


# ── uri resolution ───────────────────────────────────────────────────────────
def resolve_uri(uri, base_dir, resource_paths):
    """model://name/rest -> first existing <res>/name/rest; file:// / abs / rel paths."""
    uri = (uri or "").strip()
    if not uri:
        return None
    if uri.startswith("model://"):
        rest = uri[len("model://"):]
        for res in resource_paths:
            if not res:
                continue
            cand = os.path.join(res, *rest.split("/"))
            if os.path.exists(cand):
                return os.path.abspath(cand)
        return None
    if uri.startswith("file://"):
        uri = uri[len("file://"):]
    if "://" in uri:
        return None       # fuel / http etc. are not resolvable offline
    cand = uri if os.path.isabs(uri) else os.path.join(base_dir, uri)
    return os.path.abspath(cand) if os.path.exists(cand) else None


def default_resource_paths():
    env = []
    for var in ("GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH", "SDF_PATH"):
        env += [p for p in os.environ.get(var, "").split(os.pathsep) if p]
    return env + REPO_RESOURCE_PATHS


# ── SDF walk ─────────────────────────────────────────────────────────────────
def _walk_geometry(geom, t_col, scope, base_dir, ctx):
    if geom is None:
        ctx.warn(f"{scope}: collision without <geometry>; skipped")
        return
    for g in geom:
        tag = g.tag
        if tag == "plane":
            normal = _floats(g.findtext("normal", "0 0 1"), 3, f"{scope} plane normal", ctx.warnings)
            size = _floats(g.findtext("size", "1 1"), 2, f"{scope} plane size", ctx.warnings)
            ctx.sources.append({"name": scope, "kind": "plane", "T": t_col,
                                "normal": normal, "size": size})
        elif tag == "box":
            size = _floats(g.findtext("size", "1 1 1"), 3, f"{scope} box size", ctx.warnings)
            ctx.sources.append({"name": scope, "kind": "box", "T": t_col, "size": size})
        elif tag == "mesh":
            uri = g.findtext("uri", "")
            path = resolve_uri(uri, base_dir, ctx.resource_paths)
            if path is None:
                ctx.warn(f"{scope}: mesh uri '{uri}' not resolvable (resource paths: "
                         f"{ctx.resource_paths}); skipped")
                continue
            if not path.lower().endswith(".obj"):
                ctx.warn(f"{scope}: mesh '{path}' is not an OBJ (only OBJ supported); skipped")
                continue
            if g.find("submesh") is not None:
                ctx.warn(f"{scope}: <submesh> ignored; the whole mesh is used")
            scale = _floats(g.findtext("scale", "1 1 1"), 3, f"{scope} mesh scale", ctx.warnings)
            ctx.sources.append({"name": scope, "kind": "mesh", "T": t_col,
                                "path": path, "scale": scale, "uri": uri})
            ctx.deps.append(path)
        elif tag in UNSUPPORTED_GEOMETRY:
            ctx.warn(f"{scope}: unsupported geometry <{tag}> skipped — the radar will NOT see it")
        else:
            ctx.warn(f"{scope}: unknown geometry <{tag}> skipped — the radar will NOT see it")


def _walk_model(model, t_parent, parent_names, base_dir, prefix, ctx, pose_override=None):
    mname = model.get("name", "unnamed")
    scope_m = f"{prefix}{mname}"
    t_model = t_parent @ (pose_override if pose_override is not None
                          else _pose_of(model, parent_names, scope_m, ctx))
    for link in model.findall("link"):
        lname = link.get("name", "link")
        scope_l = f"{scope_m}/{lname}"
        t_link = t_model @ _pose_of(link, [mname, "__model__"], scope_l, ctx)
        for col in link.findall("collision"):
            scope_c = f"{scope_l}/{col.get('name', 'collision')}"
            t_col = t_link @ _pose_of(col, [lname], scope_c, ctx)
            _walk_geometry(col.find("geometry"), t_col, scope_c, base_dir, ctx)
    for nested in model.findall("model"):
        _walk_model(nested, t_model, [mname, "__model__"], base_dir, scope_m + "/", ctx)
    for inc in model.findall("include"):
        _walk_include(inc, t_model, [mname, "__model__"], base_dir, scope_m + "/", ctx)


def _walk_include(inc, t_parent, parent_names, base_dir, prefix, ctx):
    uri = inc.findtext("uri", "")
    target = resolve_uri(uri, base_dir, ctx.resource_paths)
    label = f"{prefix}include({uri})"
    if target is None:
        ctx.warn(f"{label}: uri not resolvable; skipped")
        return
    sdf_file = target
    if os.path.isdir(target):
        sdf_file = os.path.join(target, "model.sdf")
        cfg = os.path.join(target, "model.config")
        if os.path.isfile(cfg):
            try:
                ctx.deps.append(cfg)
                txt = ET.parse(cfg).getroot().findtext("sdf")
                if txt and os.path.isfile(os.path.join(target, txt.strip())):
                    sdf_file = os.path.join(target, txt.strip())
            except ET.ParseError:
                pass
    if not os.path.isfile(sdf_file):
        ctx.warn(f"{label}: no model SDF at {sdf_file}; skipped")
        return
    ctx.deps.append(sdf_file)
    model = ET.parse(sdf_file).getroot().find("model")
    if model is None:
        ctx.warn(f"{label}: {sdf_file} has no <model>; skipped")
        return
    override = _pose_of(inc, parent_names, label, ctx) if inc.find("pose") is not None else None
    name = inc.findtext("name")
    if name:
        model.set("name", name.strip())
    _walk_model(model, t_parent, parent_names, os.path.dirname(sdf_file), prefix, ctx,
                pose_override=override)


def collect_sources(world_sdf, resource_paths):
    """Parse the world SDF into geometry descriptors (no mesh loading)."""
    ctx = _Ctx(resource_paths)
    root = ET.parse(world_sdf).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"{world_sdf}: no <world> element")
    base_dir = os.path.dirname(os.path.abspath(world_sdf))
    for child in world:
        if child.tag == "model":
            _walk_model(child, np.eye(4), ["world"], base_dir, "", ctx)
        elif child.tag == "include":
            _walk_include(child, np.eye(4), ["world"], base_dir, "", ctx)
    return world.get("name", "unknown"), ctx


# ── geometry ─────────────────────────────────────────────────────────────────
def load_obj(path):
    """Return (V (n,3) float64, F (m,3) int64). Polygons fan-triangulated."""
    with open(path, "rb") as f:
        data = f.read()
    vtok = []
    faces = []
    nv = 0
    for line in data.splitlines():
        if line[:2] == b"v " or line[:2] == b"v\t":
            parts = line.split()
            if len(parts) < 4:
                raise ValueError(f"{path}: bad vertex line {line[:60]!r}")
            vtok.extend(parts[1:4])
            nv += 1
        elif line[:2] == b"f " or line[:2] == b"f\t":
            idx = []
            for tok in line.split()[1:]:
                i = int(tok.split(b"/", 1)[0])
                idx.append(i - 1 if i > 0 else nv + i)
            for k in range(1, len(idx) - 1):
                faces.extend((idx[0], idx[k], idx[k + 1]))
    v = np.fromiter(map(float, vtok), dtype=np.float64, count=len(vtok)).reshape(-1, 3)
    f = np.fromiter(faces, dtype=np.int64, count=len(faces)).reshape(-1, 3)
    if f.size and (f.min() < 0 or f.max() >= len(v)):
        raise ValueError(f"{path}: face index out of range (nv={len(v)})")
    return v, f


def _rot_z_to(n):
    n = np.asarray(n, dtype=float)
    n = n / np.linalg.norm(n)
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(z, n))
    if c > 1 - 1e-12:
        return np.eye(3)
    if c < -1 + 1e-12:
        return np.diag([1.0, -1.0, -1.0])      # 180 deg about x
    axis = np.cross(z, n)
    s = np.linalg.norm(axis)
    k = axis / s
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + s * kx + (1 - c) * (kx @ kx)


def plane_polygon(src):
    """World-frame quad (4,3) of a finite SDF plane."""
    sx, sy = src["size"]
    local = np.array([[-sx / 2, -sy / 2, 0], [sx / 2, -sy / 2, 0],
                      [sx / 2, sy / 2, 0], [-sx / 2, sy / 2, 0]], dtype=float)
    local = local @ _rot_z_to(src["normal"]).T
    return transform_points(src["T"], local)


def box_triangles(src):
    sx, sy, sz = src["size"]
    c = np.array([[x, y, z] for x in (-sx / 2, sx / 2) for y in (-sy / 2, sy / 2)
                  for z in (-sz / 2, sz / 2)], dtype=float)   # index = 4*ix + 2*iy + iz
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5),      # -x, +x
             (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6),      # -y, +y
             (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]      # -z, +z
    w = transform_points(src["T"], c)
    return w[np.array(faces)]


def mesh_triangles(src):
    v, f = load_obj(src["path"])
    v = v * np.asarray(src["scale"], dtype=float)
    return transform_points(src["T"], v)[f]


def clip_polygon_to_box(poly, bmin, bmax):
    """Sutherland-Hodgman clip of a planar polygon against an AABB."""
    out = [np.asarray(p, dtype=float) for p in poly]
    for axis in range(3):
        for sign, bound in ((1.0, bmin[axis]), (-1.0, bmax[axis])):
            if not out:
                return np.zeros((0, 3))
            inp, out = out, []
            for i in range(len(inp)):
                cur, prev = inp[i], inp[i - 1]
                dc = sign * (cur[axis] - bound)
                dp = sign * (prev[axis] - bound)
                if dc >= 0:
                    if dp < 0:
                        out.append(prev + (cur - prev) * (dp / (dp - dc)))
                    out.append(cur)
                elif dp >= 0:
                    out.append(prev + (cur - prev) * (dp / (dp - dc)))
    return np.array(out) if out else np.zeros((0, 3))


def fan(poly):
    if len(poly) < 3:
        return np.zeros((0, 3, 3))
    return np.array([[poly[0], poly[k], poly[k + 1]] for k in range(1, len(poly) - 1)])


def crop_triangles(tris, bmin, bmax):
    """Keep triangles whose AABB intersects [bmin, bmax]."""
    if len(tris) == 0:
        return tris
    lo = tris.min(axis=1)
    hi = tris.max(axis=1)
    keep = np.all(hi >= np.asarray(bmin), axis=1) & np.all(lo <= np.asarray(bmax), axis=1)
    return tris[keep]


def crop_box(region, range_m, margin_m):
    r = [float(v) for v in region]
    if r[0] > r[1] or r[2] > r[3] or r[4] > r[5]:
        raise ValueError(f"region min > max: {r}")
    e = float(range_m) + float(margin_m)
    return (np.array([r[0] - e, r[2] - e, r[4] - e]),
            np.array([r[1] + e, r[3] + e, r[5] + e]))


# ── cache / io ───────────────────────────────────────────────────────────────
def cache_hash(world_sdf, deps, bmin, bmax):
    h = hashlib.sha256()
    h.update(f"builder={BUILDER_VERSION}\n".encode())
    with open(world_sdf, "rb") as f:
        h.update(f.read())
    for d in deps:
        st = os.stat(d)
        h.update(f"\ndep={os.path.abspath(d)}|{st.st_size}|{st.st_mtime_ns}".encode())
    h.update(("\nbox=" + " ".join(f"{v:.6f}" for v in list(bmin) + list(bmax))).encode())
    return h.hexdigest()[:12]


def write_obj(path, tris, header_lines):
    pts = tris.reshape(-1, 3)
    uniq, inv = np.unique(pts, axis=0, return_inverse=True)
    faces = inv.reshape(-1, 3) + 1
    tmp = f"{path}.tmp{os.getpid()}"
    with open(tmp, "w", newline="\n") as f:
        for h in header_lines:
            f.write(f"# {h}\n")
        np.savetxt(f, uniq, fmt="v %.6f %.6f %.6f")
        np.savetxt(f, faces, fmt="f %d %d %d")
    os.replace(tmp, path)


def build_map(world_sdf, region, range_m, margin_m, cache_dir, resource_paths=None, force=False):
    """Build (or reuse) the cropped world-frame OBJ. Returns an info dict."""
    t0 = time.time()
    world_sdf = os.path.abspath(world_sdf)
    if resource_paths is None:
        resource_paths = default_resource_paths()
    world_name, ctx = collect_sources(world_sdf, resource_paths)
    bmin, bmax = crop_box(region, range_m, margin_m)
    log(f"world '{world_name}': {len(ctx.sources)} collision source(s), {len(ctx.warnings)} warning(s)")
    log(f"crop box: x [{bmin[0]:.2f}, {bmax[0]:.2f}]  y [{bmin[1]:.2f}, {bmax[1]:.2f}]  "
        f"z [{bmin[2]:.2f}, {bmax[2]:.2f}]  (region {list(map(float, region))} "
        f"+ range {float(range_m):.2f} m + margin {float(margin_m):.2f} m)")
    digest = cache_hash(world_sdf, ctx.deps, bmin, bmax)
    out = os.path.abspath(os.path.join(cache_dir, f"{world_name}_{digest}.obj"))
    info = {"path": out, "world_name": world_name, "warnings": ctx.warnings,
            "bmin": bmin, "bmax": bmax, "counts": [], "rebuilt": False}
    if os.path.isfile(out) and not force:
        with open(out, "rb") as f:
            n = sum(1 for line in f if line[:2] == b"f ")
        info["n_tris"] = n
        log(f"cache hit: {out} ({n} triangles, {os.path.getsize(out)} bytes, "
            f"{time.time() - t0:.2f} s)")
        return info

    parts = []
    for src in ctx.sources:
        ts = time.time()
        if src["kind"] == "plane":
            poly = plane_polygon(src)
            before = 2
            tri = fan(clip_polygon_to_box(poly, bmin, bmax))
        else:
            tri_all = box_triangles(src) if src["kind"] == "box" else mesh_triangles(src)
            before = len(tri_all)
            tri = crop_triangles(tri_all, bmin, bmax)
        tri = tri.reshape(-1, 3, 3)
        label = src["name"] + (f" ({src['uri']})" if src["kind"] == "mesh" else "")
        log(f"  {src['kind']:5s} {label}: {before} -> {len(tri)} triangles "
            f"({time.time() - ts:.2f} s)")
        info["counts"].append((src["name"], src["kind"], before, len(tri)))
        if len(tri):
            parts.append(tri)
    tris = np.concatenate(parts) if parts else np.zeros((0, 3, 3))
    info["n_tris"] = len(tris)
    if len(tris) == 0:
        raise EmptyMapError(
            f"cropped radar map for world '{world_name}' has ZERO triangles — nothing the "
            f"radar can hit inside the crop box; refusing to write an empty map")
    os.makedirs(cache_dir, exist_ok=True)
    write_obj(out, tris, [
        f"radar raycast map generated by eval_scripts/build_radar_map.py v{BUILDER_VERSION}",
        f"world: {world_sdf} ({world_name})",
        "frame: WORLD (collision geometry, poses applied), cropped to AABB",
        "crop box min: " + " ".join(f"{v:.6f}" for v in bmin),
        "crop box max: " + " ".join(f"{v:.6f}" for v in bmax),
        f"triangles: {len(tris)}"])
    info["rebuilt"] = True
    log(f"wrote {out} ({len(tris)} triangles, {os.path.getsize(out)} bytes, "
        f"{time.time() - t0:.2f} s total)")
    return info


def radar_range_from_config(path):
    import yaml
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return float(cfg["range_max_m"])


# ── selftest ─────────────────────────────────────────────────────────────────
def run_selftest():
    import contextlib
    import io
    import shutil
    import subprocess
    import tempfile

    results = {"pass": 0, "fail": 0}

    def check(name, cond, detail=""):
        if cond:
            results["pass"] += 1
            print(f"[selftest] PASS {name}")
        else:
            results["fail"] += 1
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))

    # conventions
    p = transform_points(pose_matrix(0, 0, 0, math.pi / 2, 0, 0), [[1, 2, 3]])[0]
    check("roll +90 maps (x,y,z) -> (x,-z,y)", np.allclose(p, [1, -3, 2]), str(p))
    p = transform_points(pose_matrix(0, 0, 0, 0, 0, math.pi / 2), [[1, 0, 0]])[0]
    check("yaw +90 maps +x -> +y", np.allclose(p, [0, 1, 0]), str(p))
    p = transform_points(pose_matrix(0, 0, 0, math.pi / 2, math.pi / 2, 0), [[0, 1, 0]])[0]
    check("fixed-axis order Ry(pitch)*Rx(roll): (0,1,0) -> (1,0,0)", np.allclose(p, [1, 0, 0]), str(p))

    td = tempfile.mkdtemp(prefix="radar_map_selftest_")
    try:
        res = os.path.join(td, "res")
        os.makedirs(os.path.join(res, "tmesh", "meshes"))
        os.makedirs(os.path.join(res, "incbox"))
        with open(os.path.join(res, "tmesh", "meshes", "quad.obj"), "w") as f:
            f.write("# quad + tri\no q\nv 1 2 3\nv 2 2 3\nv 2 3 3\nv 1 3 3\n"
                    "vn 0 0 1\nvt 0 0\nf 1/1/1 2/1/1 3/1/1 4/1/1\nf -4//1 -3//1 -1//1\n")
        with open(os.path.join(res, "incbox", "model.sdf"), "w") as f:
            f.write('<sdf version="1.9"><model name="incbox"><pose>99 99 99 0 0 0</pose>'
                    '<link name="l"><collision name="c"><geometry><box><size>2 2 2</size>'
                    '</box></geometry></collision></link></model></sdf>')

        check("model:// resolves against temp resource path",
              resolve_uri("model://tmesh/meshes/quad.obj", td, [os.path.join(td, "nope"), res])
              == os.path.abspath(os.path.join(res, "tmesh", "meshes", "quad.obj")))
        check("missing model:// returns None",
              resolve_uri("model://tmesh/meshes/missing.obj", td, [res]) is None)

        world = os.path.join(td, "synth.sdf")
        world_xml = """<?xml version="1.0"?>
<sdf version="1.9"><world name="synth">
  <model name="ground"><static>true</static><link name="link">
    <collision name="c"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry></collision>
    <visual name="v"><geometry><sphere><radius>9</radius></sphere></geometry></visual>
  </link></model>
  <model name="crate"><pose>5 0 0.5 0 0 0</pose><link name="link">
    <collision name="c"><geometry><box><size>1 2 3</size></box></geometry></collision>
  </link></model>
  <model name="rock"><pose>10 20 30 1.5707963267948966 0 0</pose><link name="link">
    <collision name="c"><geometry><mesh><uri>model://tmesh/meshes/quad.obj</uri><scale>2 2 2</scale></mesh></geometry></collision>
    <collision name="ball"><geometry><sphere><radius>1</radius></sphere></geometry></collision>
  </link></model>
  <model name="odd"><link name="link"><pose relative_to="crate">0 0 0 0 0 0</pose>
    <collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision>
  </link></model>
  <include><uri>model://incbox</uri><name>inc1</name><pose>0 3 1 0 0 1.5707963267948966</pose></include>
</world></sdf>
"""
        with open(world, "w") as f:
            f.write(world_xml)

        with contextlib.redirect_stderr(io.StringIO()):
            wname, ctx = collect_sources(world, [res])
        by = {s["name"]: s for s in ctx.sources}
        check("world name parsed", wname == "synth", wname)
        check("collision-only sources (visual sphere ignored)",
              sorted(by) == sorted(["ground/link/c", "crate/link/c", "rock/link/c",
                                    "odd/link/c", "inc1/l/c"]), str(sorted(by)))
        check("unsupported sphere collision warned by name",
              any("rock/link/ball" in w and "sphere" in w for w in ctx.warnings), str(ctx.warnings))
        check("foreign relative_to warned",
              any("odd/link" in w and "relative_to" in w for w in ctx.warnings), str(ctx.warnings))
        check("mesh file recorded as cache dependency",
              any(d.endswith("quad.obj") for d in ctx.deps) and any(d.endswith("model.sdf") for d in ctx.deps),
              str(ctx.deps))

        mt = mesh_triangles(by["rock/link/c"])
        check("OBJ polygon fan + negative index -> 3 triangles", mt.shape == (3, 3, 3), str(mt.shape))
        # (1,2,3)*2 = (2,4,6) -> roll90 (2,-6,4) -> +(10,20,30) = (12,14,34)
        # (2,3,3)*2 = (4,6,6) -> roll90 (4,-6,6) -> +(10,20,30) = (14,14,36)
        check("mesh vertex (1,2,3) scale 2 roll 90 + t -> (12,14,34)", np.allclose(mt[0][0], [12, 14, 34]), str(mt[0][0]))
        check("mesh vertex (2,3,3) -> (14,14,36)", np.allclose(mt[0][2], [14, 14, 36]), str(mt[0][2]))

        bt = box_triangles(by["crate/link/c"])
        check("box -> 12 triangles", bt.shape == (12, 3, 3), str(bt.shape))
        check("box extents = pose +/- size/2",
              np.allclose(bt.reshape(-1, 3).min(0), [4.5, -1, -1]) and np.allclose(bt.reshape(-1, 3).max(0), [5.5, 1, 2]))
        areas = 0.5 * np.linalg.norm(np.cross(bt[:, 1] - bt[:, 0], bt[:, 2] - bt[:, 0]), axis=1)
        check("box surface area 2(ab+bc+ca)=22", abs(areas.sum() - 22.0) < 1e-9, str(areas.sum()))
        it = box_triangles(by["inc1/l/c"]).reshape(-1, 3)
        check("include pose overrides model pose (center 0,3,1)",
              np.allclose((it.min(0) + it.max(0)) / 2, [0, 3, 1]), str((it.min(0) + it.max(0)) / 2))

        bmin, bmax = np.array([-1.0, -1, -1]), np.array([1.0, 1, 1])
        near = np.array([[0, 0, 0], [0.5, 0, 0], [0, 0.5, 0]], dtype=float)
        far = near + 100
        big = np.array([[-50, -50, 0], [50, -50, 0], [0, 50, 0]], dtype=float)
        kept = crop_triangles(np.stack([near, far, big]), bmin, bmax)
        check("crop keeps near, drops far, keeps box-crossing big triangle",
              len(kept) == 2 and np.allclose(kept[0], near) and np.allclose(kept[1], big), str(kept))

        cb_min, cb_max = np.array([-5.0, -3, -1]), np.array([5.0, 3, 1])
        clipped = fan(clip_polygon_to_box(plane_polygon(by["ground/link/c"]), cb_min, cb_max))
        cpts = clipped.reshape(-1, 3)
        carea = 0.5 * np.linalg.norm(np.cross(clipped[:, 1] - clipped[:, 0], clipped[:, 2] - clipped[:, 0]), axis=1).sum()
        check("plane quad clipped to crop box (area 10x6)",
              abs(carea - 60.0) < 1e-9 and np.all(cpts >= cb_min - 1e-9) and np.all(cpts <= cb_max + 1e-9),
              f"area={carea}")
        tilted = {"T": pose_matrix(0, 0, 0, 0, 0, 0), "normal": [0, -1, 0], "size": [4, 6]}
        tq = plane_polygon(tilted)
        check("plane normal (0,-1,0) gives a quad in the y=0 plane", np.allclose(tq[:, 1], 0), str(tq))

        cache = os.path.join(td, "cache")
        region = [0, 1, 0, 0, 0, 3]
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            i1 = build_map(world, region, 20.0, 2.0, cache, [res])
            mtime1 = os.stat(i1["path"]).st_mtime_ns
            i2 = build_map(world, region, 20.0, 2.0, cache, [res])
        check("first build writes a file", i1["rebuilt"] and os.path.isfile(i1["path"]))
        check("second build is a cache hit (no rebuild, file untouched)",
              not i2["rebuilt"] and i2["path"] == i1["path"] and os.stat(i2["path"]).st_mtime_ns == mtime1)
        check("output name is <world>_<hash12>.obj",
              os.path.basename(i1["path"]).startswith("synth_") and len(os.path.basename(i1["path"])) == len("synth_") + 12 + 4)
        # expected: plane clipped (2) + crate 12 + inc box 12 + odd box 12; rock at z~34 is out of box z<=25
        check("crop drops out-of-range mesh, keeps plane/boxes",
              i1["n_tris"] == 2 + 12 + 12 + 12 and dict((c[0], c[3]) for c in i1["counts"])["rock/link/c"] == 0,
              str(i1["counts"]))
        v_back, f_back = load_obj(i1["path"])
        check("written OBJ parses back with same triangle count", len(f_back) == i1["n_tris"],
              f"{len(f_back)} vs {i1['n_tris']}")
        check("parsed-back vertices lie inside crop box",
              np.all(v_back >= i1["bmin"] - 1e-5) and np.all(v_back <= i1["bmax"] + 1e-5))
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            i3 = build_map(world, [0, 1, 0, 0, 0, 20], 20.0, 2.0, cache, [res])
        check("larger region includes the mesh", dict((c[0], c[3]) for c in i3["counts"])["rock/link/c"] == 3,
              str(i3["counts"]))
        with open(world, "a") as f:
            f.write("<!-- changed -->\n")
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            i4 = build_map(world, region, 20.0, 2.0, cache, [res])
        check("hash changes when world file changes", i4["rebuilt"] and i4["path"] != i1["path"])

        env = dict(os.environ, GZ_SIM_RESOURCE_PATH=res, PYTHONIOENCODING="utf-8")
        cp = subprocess.run([sys.executable, os.path.abspath(__file__), world, "--region", "0", "1", "0", "0", "0", "3",
                             "--range-m", "20", "--cache-dir", os.path.join(td, "cache_cli"), "--resource-path", res],
                            capture_output=True, text=True, env=env)
        lines = cp.stdout.strip().splitlines()
        check("CLI exits 0 and last stdout line is the absolute output path",
              cp.returncode == 0 and lines and os.path.isabs(lines[-1]) and os.path.isfile(lines[-1]),
              f"rc={cp.returncode} out={cp.stdout[-300:]} err={cp.stderr[-300:]}")

        far_world = os.path.join(td, "far.sdf")
        with open(far_world, "w") as f:
            f.write('<sdf version="1.9"><world name="far"><model name="b"><pose>1000 0 0 0 0 0</pose>'
                    '<link name="l"><collision name="c"><geometry><box><size>1 1 1</size></box>'
                    '</geometry></collision></link></model></world></sdf>')
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            rc = main([far_world, "--region", "0", "1", "0", "0", "0", "3", "--range-m", "20",
                       "--cache-dir", os.path.join(td, "cache_far")])
        far_cache = os.path.join(td, "cache_far")
        check("zero-triangle result exits non-zero and writes nothing",
              rc != 0 and (not os.path.isdir(far_cache) or not os.listdir(far_cache)), f"rc={rc}")
        if os.path.isfile(DEFAULT_RADAR_CONFIG):
            check("range_max_m read from radar_noise.yaml", radar_range_from_config(DEFAULT_RADAR_CONFIG) > 0)
    finally:
        shutil.rmtree(td, ignore_errors=True)

    ok = results["fail"] == 0
    print(f"[selftest] {results['pass']} passed, {results['fail']} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


# ── CLI ──────────────────────────────────────────────────────────────────────
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("world_sdf", nargs="?", help="World SDF to convert")
    parser.add_argument("--region", nargs=6, type=float,
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
                        help="Flight region (world frame, metres)")
    parser.add_argument("--range-m", type=float, default=None,
                        help="Radar max range [default: range_max_m from --radar-config]")
    parser.add_argument("--radar-config", default=DEFAULT_RADAR_CONFIG,
                        help="radar_noise.yaml for the default range [default: %(default)s]")
    parser.add_argument("--margin-m", type=float, default=DEFAULT_MARGIN_M,
                        help="Extra crop margin beyond the range [default: %(default)s]")
    parser.add_argument("--cache-dir", default=DEFAULT_CACHE_DIR, help="[default: %(default)s]")
    parser.add_argument("--resource-path", action="append", default=None,
                        help="Extra model:// search dir (repeatable; searched before GZ_SIM_RESOURCE_PATH)")
    parser.add_argument("--force", action="store_true", help="Rebuild even if a cached map exists")
    parser.add_argument("--selftest", action="store_true", help="Run the offline selftest and exit")
    args = parser.parse_args(argv)
    if args.selftest:
        return run_selftest()
    if not args.world_sdf or args.region is None:
        parser.error("world_sdf and --region are required (or pass --selftest)")
    try:
        if args.range_m is None:
            args.range_m = radar_range_from_config(args.radar_config)
            log(f"radar range {args.range_m:.2f} m from {args.radar_config}")
        paths = (args.resource_path or []) + default_resource_paths()
        info = build_map(args.world_sdf, args.region, args.range_m, args.margin_m,
                         args.cache_dir, paths, force=args.force)
    except EmptyMapError as e:
        print(f"[build_radar_map] ERROR: {e}", file=sys.stderr, flush=True)
        return 2
    except Exception as e:     # noqa: BLE001 — surface any failure as a clean exit code
        print(f"[build_radar_map] ERROR: {type(e).__name__}: {e}", file=sys.stderr, flush=True)
        return 1
    print(info["path"], flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
