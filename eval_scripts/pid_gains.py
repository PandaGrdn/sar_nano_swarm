#!/usr/bin/env python3
"""pid_gains.py — shared helpers for reading/writing/applying Crazyflie PID
gain sets (Phase 1 M2). Used by tune_pid.py and push_pid_gains.py.

All group/field names below are verified runtime CFLib parameters, taken
directly from the firmware's PARAM_ADD() calls:
  - src/modules/src/controller/attitude_pid_controller.c
      -> PARAM_GROUP_START(pid_rate), PARAM_GROUP_START(pid_attitude)
  - src/modules/src/controller/position_controller_pid.c
      -> PARAM_GROUP_START(velCtlPid), PARAM_GROUP_START(posCtlPid)
If you bump the pinned CrazySim/crazyflie-firmware submodule commit, re-grep
those files before trusting this list (AGENTS.md §6.4: never fabricate
firmware API details).
"""
import subprocess

import yaml

PARAM_GROUPS = {
    "pid_rate": ["roll_kp", "roll_ki", "roll_kd", "roll_kff",
                 "pitch_kp", "pitch_ki", "pitch_kd", "pitch_kff",
                 "yaw_kp", "yaw_ki", "yaw_kd", "yaw_kff"],
    "pid_attitude": ["roll_kp", "roll_ki", "roll_kd", "roll_kff",
                      "pitch_kp", "pitch_ki", "pitch_kd", "pitch_kff",
                      "yaw_kp", "yaw_ki", "yaw_kd", "yaw_kff"],
    "velCtlPid": ["vxKp", "vxKi", "vxKd", "vxKFF",
                  "vyKp", "vyKi", "vyKd", "vyKFF",
                  "vzKp", "vzKi", "vzKd", "vzKFF"],
    "posCtlPid": ["xKp", "xKi", "xKd", "xKff",
                  "yKp", "yKi", "yKd", "yKff",
                  "zKp", "zKi", "zKd", "zKff"],
}


def load_gains(path):
    """Load a nested {group: {field: value}} gain dict from YAML."""
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    for group, fields in data.items():
        for field in fields:
            if group in PARAM_GROUPS and field not in PARAM_GROUPS[group]:
                raise ValueError(f"Unknown param '{group}.{field}' — check for a typo "
                                  f"or a firmware param rename.")
    return data


def save_gains(gains, path):
    with open(path, "w") as f:
        yaml.safe_dump(gains, f, default_flow_style=False, sort_keys=False)


def flatten(gains):
    """{group: {field: value}} -> {"group.field": value}"""
    return {f"{g}.{k}": v for g, fields in gains.items() for k, v in fields.items()}


def unflatten(flat):
    """{"group.field": value} -> {group: {field: value}}"""
    nested = {}
    for key, value in flat.items():
        group, field = key.split(".", 1)
        nested.setdefault(group, {})[field] = value
    return nested


def apply_gains(cf, gains):
    """Push every gain in a nested {group: {field: value}} dict to a
    connected cflib Crazyflie instance via the runtime PARAM interface
    (no firmware rebuild/reflash needed).
    """
    for group, fields in gains.items():
        for field, value in fields.items():
            cf.param.set_value(f"{group}.{field}", str(value))


def reset_estimator(cf, estimator_group="kalman"):
    """Fire the estimator's runtime reset param (e.g. kalman.resetEstimation).
    Group name depends on which estimator build is active — see
    configs/airframe/pid_tune.yaml `connection.estimator_reset_group`.
    """
    cf.param.set_value(f"{estimator_group}.resetEstimation", "1")


def reset_pose(world_name, model_name, xyz=(0.0, 0.0, 0.5), gz_bin="gz", timeout_ms=2000):
    """Teleport the gz-sim model entity back to a spawn pose between tuning
    trials, via the UserCommands 'set_pose' service.

    NOTE (AGENTS.md §6.4): this service name/message shape is the standard
    gz-sim UserCommands convention but is NOT yet exercised anywhere else in
    this repo. Verify once with:
        gz service -l | grep set_pose
    before trusting this in an unattended sweep; if the name/fields differ
    on your installed gz-sim Harmonic version, paste `gz service -i -s
    /world/<world>/set_pose` output and this will get corrected.
    """
    x, y, z = xyz
    req = f"name: \"{model_name}\", position: {{x: {x}, y: {y}, z: {z}}}"
    cmd = [
        gz_bin, "service",
        "-s", f"/world/{world_name}/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", str(timeout_ms),
        "--req", req,
    ]
    subprocess.run(cmd, check=True, capture_output=True, text=True)
