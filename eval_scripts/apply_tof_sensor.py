#!/usr/bin/env python3
"""apply_tof_sensor.py — inject IR ToF rangefinder sensor(s) (Gazebo
`gpu_lidar`, single ray per beam = pure raycast geometry) into a generated
Crazyflie SDF's `base_link`, per configs/sensors/tof.yaml.

Phase 1 M3 (Workstream C, §4.1 of
.cursor/docs/Phase1_Physical_Fidelity_and_Sensor_Implementation_Plan.md).

Like apply_payload.py and the radar-plugin injection, this is a launch-time
post-processing step on the temporary generated SDF (e.g. /tmp/<model>_<id>.sdf)
— it never touches the firmware_mods/CrazySim submodule. Called from
phase0_gate.sh right after apply_payload.py's mass/inertia rewrite.

Kept as a separate script from apply_payload.py (which only rewrites mass/CoM/
inertia) rather than merging sensor injection into it, for single-responsibility/
testability — each script reads one config and does one job.

Requires gz-sim-sensors-system (ogre2) at the WORLD level — already present in
sim_worlds/phase0_tunnel_gate.sdf and sim_worlds/phase1_pid_tune.sdf.

Usage:
    apply_tof_sensor.py SDF_PATH [--config configs/sensors/tof.yaml]
                                 [--link base_link] [--cf-id 0]
"""
import argparse
import sys
import xml.etree.ElementTree as ET

import yaml

SENSOR_TYPE = "gpu_lidar"


def load_config(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _single_ray_lidar_elem(cfg):
    """Build the <lidar> block shared by every beam: samples=1 in H and V
    (a single ray — exactly what a ToF laser measures), range/noise from cfg.
    """
    lidar = ET.Element("lidar")
    scan = ET.SubElement(lidar, "scan")
    horizontal = ET.SubElement(scan, "horizontal")
    ET.SubElement(horizontal, "samples").text = "1"
    ET.SubElement(horizontal, "min_angle").text = "0"
    ET.SubElement(horizontal, "max_angle").text = "0"
    vertical = ET.SubElement(scan, "vertical")
    ET.SubElement(vertical, "samples").text = "1"
    ET.SubElement(vertical, "min_angle").text = "0"
    ET.SubElement(vertical, "max_angle").text = "0"

    rng = ET.SubElement(lidar, "range")
    ET.SubElement(rng, "min").text = str(cfg["range_min_m"])
    ET.SubElement(rng, "max").text = str(cfg["range_max_m"])
    ET.SubElement(rng, "resolution").text = str(cfg["range_resolution_m"])

    # NOTE: unlike <air_pressure><pressure><noise type="gaussian"> (an
    # attribute) in model.sdf.jinja, sdformat's <lidar><noise> schema wants
    # "type" as a child element (verified against
    # /usr/share/sdformat12/1.9/lidar.sdf) — using an attribute here logs an
    # "XML Attribute[type] ... not defined in SDF" warning and silently drops
    # the noise model.
    noise = ET.SubElement(lidar, "noise")
    ET.SubElement(noise, "type").text = "gaussian"
    ET.SubElement(noise, "mean").text = "0"
    ET.SubElement(noise, "stddev").text = str(cfg["noise_stddev_m"])
    return lidar


def build_beam_sensor(name, topic, pose_xyz_rpy, cfg):
    sensor = ET.Element("sensor")
    sensor.set("name", f"tof_{name}")
    sensor.set("type", SENSOR_TYPE)
    ET.SubElement(sensor, "topic").text = topic
    ET.SubElement(sensor, "always_on").text = "1"
    ET.SubElement(sensor, "update_rate").text = str(cfg["update_rate_hz"])
    ET.SubElement(sensor, "pose").text = " ".join(str(v) for v in pose_xyz_rpy)
    sensor.append(_single_ray_lidar_elem(cfg))
    return sensor


def collect_beams(cfg):
    """Return [(beam_name, pose_xyz_rpy), ...] for every enabled beam."""
    beams = []
    down = cfg.get("down", {})
    if down.get("enabled", True):
        beams.append(("down", down["pose_xyz_rpy"]))

    mr = cfg.get("multi_ranger", {})
    if mr.get("enabled", False):
        for direction in ("front", "back", "left", "right", "up"):
            entry = mr.get(direction)
            if entry:
                beams.append((direction, entry["pose_xyz_rpy"]))
    return beams


def inject(sdf_path, link_name, cfg, cf_id):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        print(f"[apply_tof_sensor] ERROR: no <model> element in {sdf_path}", file=sys.stderr)
        sys.exit(1)

    link = None
    for candidate in model.findall("link"):
        if candidate.get("name") == link_name:
            link = candidate
            break
    if link is None:
        print(f"[apply_tof_sensor] ERROR: no <link name=\"{link_name}\"> in {sdf_path}", file=sys.stderr)
        sys.exit(1)

    beams = collect_beams(cfg)
    if not beams:
        print("[apply_tof_sensor] WARNING: no beams enabled in config — nothing injected", file=sys.stderr)
        return []

    injected = []
    for name, pose in beams:
        topic = f"/cf_{cf_id}/tof_{name}"
        sensor_elem = build_beam_sensor(name, topic, pose, cfg)
        link.append(sensor_elem)
        injected.append((name, topic))

    tree.write(sdf_path, encoding="unicode")
    return injected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdf_path", help="Path to the generated SDF (edited in place)")
    parser.add_argument(
        "--config",
        default="configs/sensors/tof.yaml",
        help="Path to tof.yaml (relative to SAR_NANO_SWARM_ROOT or absolute)",
    )
    parser.add_argument("--link", default="base_link", help="Link to inject sensors into")
    parser.add_argument("--cf-id", default="0", help="Crazyflie instance id (matches phase0_gate.sh's CF_ID, used to build the /cf_<id>/tof_* topic name)")
    args = parser.parse_args()

    cfg = load_config(args.config)
    injected = inject(args.sdf_path, args.link, cfg, args.cf_id)

    print(f"[apply_tof_sensor] {args.sdf_path}: link '{args.link}' updated")
    print(f"[apply_tof_sensor]   deck   = {cfg.get('deck', '?')}")
    print(f"[apply_tof_sensor]   range  = {cfg['range_min_m']}-{cfg['range_max_m']} m, "
          f"noise stddev {cfg['noise_stddev_m']} m, rate {cfg['update_rate_hz']} Hz")
    for name, topic in injected:
        print(f"[apply_tof_sensor]   beam '{name}' -> {topic}")


if __name__ == "__main__":
    main()
