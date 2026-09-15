#!/usr/bin/env python3
"""Start phase0 for tunnel/triangle_forward (pad spawn + fresh derived yaml)."""
import os
import shlex
import subprocess
import sys
from pathlib import Path

root = Path("/mnt/d/GitHub/gps_denied_drones")
os.chdir(root)
sys.path.insert(0, str(root / "eval_scripts"))
from swarm_loc_scenarios import (  # noqa: E402
    eval_dir_for,
    get_scenario,
    phase0_cmd,
    write_derived_estimator_config,
)

spec = get_scenario("tunnel/triangle_forward")
eval_dir = eval_dir_for(spec)
derived = f"{eval_dir}/swarm_loc_derived.yaml"
write_derived_estimator_config(
    "tunnel/triangle_forward",
    "configs/estimation/swarm_loc.yaml",
    derived,
)
argv = phase0_cmd(
    spec,
    extra=["--headless", "--no-rviz", "--swarm-loc-config", derived],
)
log = Path("/tmp/tri_fwd_phase0.log")
log.write_text("LAUNCH_START\n", encoding="utf-8")
inner = "source setup_env.sh && exec " + shlex.join(argv)
cmd = ["bash", "-lc", inner]
with log.open("a", encoding="utf-8") as fh:
    proc = subprocess.Popen(
        cmd,
        stdout=fh,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
        cwd=str(root),
    )
print(f"LAUNCH_PID={proc.pid}")
print("PHASE0_CMD=" + shlex.join(argv))
