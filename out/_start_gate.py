#!/usr/bin/env python3
"""Start swarm_loc_gate in the background. Filename avoids pkill patterns."""
import os
import subprocess
from pathlib import Path

root = Path("/mnt/d/GitHub/gps_denied_drones")
os.chdir(root)
log = Path("/tmp/tri_fwd_gate.log")
log.write_text("GATE_START\n", encoding="utf-8")
cmd = [
    "bash", "-lc",
    "source /mnt/d/GitHub/gps_denied_drones/setup_env.sh && "
    "cd /mnt/d/GitHub/gps_denied_drones && "
    "exec python3 -u eval_scripts/swarm_loc_gate.py "
    "--scenario tunnel/triangle_forward "
    "--config out/swarm_loc_eval/tunnel/triangle_forward/swarm_loc_derived.yaml "
    "--eval-dir out/swarm_loc_eval/tunnel/triangle_forward "
    "--logs out/swarm_loc_logs/tunnel/triangle_forward "
    "--connect-timeout 180 --connect-wait 180",
]
with log.open("a", encoding="utf-8") as fh:
    proc = subprocess.Popen(
        cmd,
        stdout=fh,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
        cwd=str(root),
    )
print(f"FLY_PID={proc.pid}")
