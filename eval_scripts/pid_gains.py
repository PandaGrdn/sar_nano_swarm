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
import math
import subprocess
import sys

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


def pose_request(model_name, xyz=(0.0, 0.0, 0.5), yaw_rad=0.0) -> str:
    """gz.msgs.Pose protobuf text for UserCommands set_pose.

    Always includes an identity-tilt quaternion (yaw only). Omitting
    orientation leaves whatever tumble the model already has — on lava_tube
    that flipped cf_0 (roll ≈ −177°) so it never took off.
    """
    x, y, z = (float(v) for v in xyz)
    yaw = float(yaw_rad)
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (
        f'name: "{model_name}", '
        f"position: {{x: {x}, y: {y}, z: {z}}}, "
        f"orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}"
    )


def reset_pose(world_name, model_name, xyz=(0.0, 0.0, 0.5), yaw_rad=0.0,
               gz_bin="gz", timeout_ms=2000):
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
    req = pose_request(model_name, xyz, yaw_rad)
    cmd = [
        gz_bin, "service",
        "-s", f"/world/{world_name}/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", str(timeout_ms),
        "--req", req,
    ]
    subprocess.run(cmd, check=True, capture_output=True, text=True)


def run_selftest() -> int:
    ok = True
    n_pass = n_fail = 0

    def check(name, cond, detail=""):
        nonlocal ok, n_pass, n_fail
        if cond:
            n_pass += 1
            print(f"[selftest] PASS {name}")
        else:
            ok = False
            n_fail += 1
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))

    r = pose_request("crazyflie_0", (0.0, 0.45, 0.5), 0.0)
    check("1 identity tilt at yaw 0", "orientation: {x: 0, y: 0, z: 0.0, w: 1.0}" in r, r)
    check("1b position", "position: {x: 0.0, y: 0.45, z: 0.5}" in r, r)
    check("1c name", 'name: "crazyflie_0"' in r, r)
    r90 = pose_request("crazyflie_1", (1.0, 0.0, 0.5), math.pi / 2)
    qz, qw = math.sin(math.pi / 4), math.cos(math.pi / 4)
    check("2 yaw 90 deg is z-w equal",
          f"z: {qz}" in r90 and f"w: {qw}" in r90 and "x: 0, y: 0" in r90, r90)
    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        raise SystemExit(run_selftest())
    raise SystemExit("pid_gains.py is a library; use --selftest")
