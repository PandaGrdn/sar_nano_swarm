#!/usr/bin/env python3
"""apply_payload.py — rewrite a generated Crazyflie SDF's base_link
<inertial> block to reflect the real loaded airframe (base + every
sensor/compute payload deck), computed from configs/airframe/payload.yaml.

This is a launch-time post-processing step, mirroring the radar-plugin
injection already done in eval_scripts/phase0_gate.sh. It never touches the
firmware_mods/CrazySim submodule; it only edits the temporary generated SDF
(e.g. /tmp/<model>_<id>.sdf) produced by CrazySim's jinja_gen.py.

Roadmap: Phase 1 step 1 (mass/inertia model for the loaded airframe).
See .cursor/docs/Phase1_Physical_Fidelity_and_Sensor_Implementation_Plan.md.

Usage:
    apply_payload.py SDF_PATH [--payload configs/airframe/payload.yaml]
                              [--link base_link]
                              [--report /path/to/report.json]

Math: every component (base + point-mass payloads) is combined via the
parallel-axis theorem about the composite centre of mass:

    M      = sum(m_i)
    c      = sum(m_i * r_i) / M
    I_c    = sum( I_i_own + m_i * (|d_i|^2 * E3 - outer(d_i, d_i)) )
             where d_i = r_i - c

`base` supplies I_i_own (its own rotational inertia about its own CoM,
r_base); every other component is a point mass (I_i_own = 0).
"""
import argparse
import json
import sys
import xml.etree.ElementTree as ET

import yaml

G_TO_KG = 1.0e-3


def load_payload(payload_path):
    with open(payload_path, "r") as f:
        data = yaml.safe_load(f)
    return data


def component_mass_kg(comp):
    mass_g = comp.get("mass_g", 0.0)
    count = comp.get("count", 1)
    return mass_g * count * G_TO_KG


def compute_composite(payload):
    base = payload["base"]
    base_mass_kg = base["mass_g"] * G_TO_KG
    base_pos = tuple(base.get("com_xyz_m", [0.0, 0.0, 0.0]))
    base_I = base["inertia"]

    bodies = [
        {
            "name": "base",
            "mass_kg": base_mass_kg,
            "pos": base_pos,
            "own_I": (
                (base_I["ixx"], base_I["ixy"], base_I["ixz"]),
                (base_I["ixy"], base_I["iyy"], base_I["iyz"]),
                (base_I["ixz"], base_I["iyz"], base_I["izz"]),
            ),
            "enabled": True,
        }
    ]

    for comp in payload.get("components", []):
        if not comp.get("enabled", True):
            continue
        mass_kg = component_mass_kg(comp)
        if mass_kg <= 0.0:
            continue
        bodies.append(
            {
                "name": comp.get("name", "?"),
                "mass_kg": mass_kg,
                "pos": tuple(comp.get("pose_xyz_m", [0.0, 0.0, 0.0])),
                "own_I": ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
                "enabled": True,
            }
        )

    total_mass = sum(b["mass_kg"] for b in bodies)
    if total_mass <= 0.0:
        raise ValueError("Composite mass is zero or negative — check payload.yaml")

    com = [0.0, 0.0, 0.0]
    for b in bodies:
        for k in range(3):
            com[k] += b["mass_kg"] * b["pos"][k]
    com = tuple(c / total_mass for c in com)

    # Parallel-axis theorem, accumulated as a plain 3x3 nested list.
    I_total = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    for b in bodies:
        d = tuple(b["pos"][k] - com[k] for k in range(3))
        d2 = sum(x * x for x in d)
        m = b["mass_kg"]
        for i in range(3):
            for j in range(3):
                shift = m * ((d2 if i == j else 0.0) - d[i] * d[j])
                I_total[i][j] += b["own_I"][i][j] + shift

    return {
        "total_mass_kg": total_mass,
        "com_xyz_m": com,
        "inertia": I_total,
        "bodies": [
            {"name": b["name"], "mass_kg": b["mass_kg"], "pos": b["pos"]}
            for b in bodies
        ],
    }


def rewrite_sdf(sdf_path, link_name, composite):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        print(f"[apply_payload] ERROR: no <model> element in {sdf_path}", file=sys.stderr)
        sys.exit(1)

    link = None
    for candidate in model.findall("link"):
        if candidate.get("name") == link_name:
            link = candidate
            break
    if link is None:
        print(f"[apply_payload] ERROR: no <link name=\"{link_name}\"> in {sdf_path}", file=sys.stderr)
        sys.exit(1)

    inertial = link.find("inertial")
    if inertial is None:
        print(f"[apply_payload] ERROR: <link name=\"{link_name}\"> has no <inertial>", file=sys.stderr)
        sys.exit(1)

    # <pose> — CoM offset from the link origin.
    pose_elem = inertial.find("pose")
    if pose_elem is None:
        pose_elem = ET.SubElement(inertial, "pose")
    cx, cy, cz = composite["com_xyz_m"]
    pose_elem.text = f"{cx:.8f} {cy:.8f} {cz:.8f} 0 0 0"

    # <mass>
    mass_elem = inertial.find("mass")
    if mass_elem is None:
        mass_elem = ET.SubElement(inertial, "mass")
    mass_elem.text = f"{composite['total_mass_kg']:.8f}"

    # <inertia>
    inertia_elem = inertial.find("inertia")
    if inertia_elem is None:
        inertia_elem = ET.SubElement(inertial, "inertia")
    I = composite["inertia"]
    field_map = {
        "ixx": I[0][0], "iyy": I[1][1], "izz": I[2][2],
        "ixy": I[0][1], "ixz": I[0][2], "iyz": I[1][2],
    }
    for tag, value in field_map.items():
        elem = inertia_elem.find(tag)
        if elem is None:
            elem = ET.SubElement(inertia_elem, tag)
        elem.text = f"{value:.10e}"

    tree.write(sdf_path, encoding="unicode")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdf_path", help="Path to the generated SDF (edited in place)")
    parser.add_argument(
        "--payload",
        default="configs/airframe/payload.yaml",
        help="Path to payload.yaml (relative to SAR_NANO_SWARM_ROOT or absolute)",
    )
    parser.add_argument("--link", default="base_link", help="Link to rewrite <inertial> for")
    parser.add_argument("--report", default=None, help="Optional path to write a JSON summary")
    args = parser.parse_args()

    payload = load_payload(args.payload)
    composite = compute_composite(payload)
    rewrite_sdf(args.sdf_path, args.link, composite)

    total_g = composite["total_mass_kg"] / G_TO_KG
    cx, cy, cz = composite["com_xyz_m"]
    print(f"[apply_payload] {args.sdf_path}: link '{args.link}' updated")
    print(f"[apply_payload]   total mass   = {total_g:.3f} g ({composite['total_mass_kg']:.6f} kg)")
    print(f"[apply_payload]   CoM offset   = ({cx:.5f}, {cy:.5f}, {cz:.5f}) m")
    print("[apply_payload]   contributing bodies:")
    for b in composite["bodies"]:
        print(f"[apply_payload]     - {b['name']:<22s} {b['mass_kg'] / G_TO_KG:7.3f} g  @ {b['pos']}")

    if args.report:
        with open(args.report, "w") as f:
            json.dump(
                {
                    "sdf_path": args.sdf_path,
                    "link": args.link,
                    "total_mass_kg": composite["total_mass_kg"],
                    "com_xyz_m": list(composite["com_xyz_m"]),
                    "inertia": composite["inertia"],
                    "bodies": composite["bodies"],
                },
                f,
                indent=2,
            )
        print(f"[apply_payload]   report written: {args.report}")


if __name__ == "__main__":
    main()
