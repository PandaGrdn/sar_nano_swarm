#!/usr/bin/env python3
"""thrust_margin_check.py — compute thrust-to-weight ratio for a generated
Crazyflie SDF and gate against a configurable floor.

Reads:
  - motor thrust model from every gz-sim-multicopter-motor-model-system
    plugin in the SDF (motorConstant, maxRotVelocity): F_i = k_i * omega_i^2
  - loaded mass from the base_link <inertial><mass> (run apply_payload.py
    first so this reflects the full payload, not the stock 27 g default)

Logs params/metrics to MLflow (sqlite:///mlflow.db, per AGENTS.md §6.5) and
exits 0/1 for use as a CI-style gate.

Usage:
    thrust_margin_check.py SDF_PATH [--config configs/airframe/thrust_margin.yaml]
"""
import argparse
import sys
import xml.etree.ElementTree as ET

import yaml

MOTOR_PLUGIN_NAME = "gz::sim::systems::MulticopterMotorModel"


def load_config(config_path):
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def parse_motors(model):
    motors = []
    for plugin in model.findall("plugin"):
        if plugin.get("name") != MOTOR_PLUGIN_NAME:
            continue
        k_elem = plugin.find("motorConstant")
        w_elem = plugin.find("maxRotVelocity")
        joint_elem = plugin.find("jointName")
        if k_elem is None or w_elem is None:
            continue
        motors.append(
            {
                "joint": joint_elem.text if joint_elem is not None else "?",
                "motor_constant": float(k_elem.text),
                "max_rot_velocity": float(w_elem.text),
            }
        )
    return motors


def parse_mass_kg(model, link_name):
    for link in model.findall("link"):
        if link.get("name") != link_name:
            continue
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass_elem = inertial.find("mass")
        if mass_elem is None:
            continue
        return float(mass_elem.text)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdf_path", help="Path to the generated SDF")
    parser.add_argument(
        "--config",
        default="configs/airframe/thrust_margin.yaml",
        help="Path to thrust_margin.yaml (relative to SAR_NANO_SWARM_ROOT or absolute)",
    )
    parser.add_argument("--link", default="base_link", help="Link the mass is read from")
    parser.add_argument("--no-mlflow", action="store_true", help="Skip MLflow logging")
    args = parser.parse_args()

    cfg = load_config(args.config)
    gravity = cfg.get("gravity_mps2", 9.80665)
    floor = cfg.get("min_thrust_to_weight", 1.5)

    tree = ET.parse(args.sdf_path)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        print(f"[thrust_margin] ERROR: no <model> element in {args.sdf_path}", file=sys.stderr)
        sys.exit(1)

    motors = parse_motors(model)
    if not motors:
        print(f"[thrust_margin] ERROR: no '{MOTOR_PLUGIN_NAME}' plugins found", file=sys.stderr)
        sys.exit(1)

    mass_kg = parse_mass_kg(model, args.link)
    if mass_kg is None:
        print(f"[thrust_margin] ERROR: could not read <mass> from link '{args.link}'", file=sys.stderr)
        sys.exit(1)

    thrust_max_n = sum(m["motor_constant"] * m["max_rot_velocity"] ** 2 for m in motors)
    weight_n = mass_kg * gravity
    twr = thrust_max_n / weight_n if weight_n > 0 else float("inf")
    passed = twr >= floor

    print(f"[thrust_margin] {args.sdf_path}")
    print(f"[thrust_margin]   motors           = {len(motors)}")
    for m in motors:
        print(
            f"[thrust_margin]     - {m['joint']:<10s} k={m['motor_constant']:.4e}"
            f"  max_omega={m['max_rot_velocity']:.1f} rad/s"
        )
    print(f"[thrust_margin]   loaded mass      = {mass_kg * 1000:.3f} g")
    print(f"[thrust_margin]   max static thrust= {thrust_max_n * 1000:.2f} mN")
    print(f"[thrust_margin]   weight           = {weight_n * 1000:.2f} mN")
    print(f"[thrust_margin]   thrust/weight    = {twr:.3f}  (floor: {floor})")

    if not args.no_mlflow:
        try:
            import mlflow

            mlflow.set_tracking_uri("sqlite:///mlflow.db")
            mlflow.set_experiment(cfg.get("mlflow_experiment", "phase1_airframe"))
            with mlflow.start_run(run_name="thrust_margin_check"):
                mlflow.log_param("sdf_path", args.sdf_path)
                mlflow.log_param("num_motors", len(motors))
                mlflow.log_param("min_thrust_to_weight_floor", floor)
                mlflow.log_metric("mass_kg", mass_kg)
                mlflow.log_metric("thrust_max_n", thrust_max_n)
                mlflow.log_metric("weight_n", weight_n)
                mlflow.log_metric("thrust_to_weight", twr)
                mlflow.log_metric("gate_passed", 1.0 if passed else 0.0)
        except ImportError:
            print("[thrust_margin] WARNING: mlflow not installed, skipping logging", file=sys.stderr)

    if passed:
        print("[thrust_margin] PASS")
        sys.exit(0)
    else:
        print(
            "[thrust_margin] FAIL — thrust-to-weight below floor. "
            "Reduce payload mass or evaluate the Crazyflie 2.1 Brushless (roadmap Ph1.7)."
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
