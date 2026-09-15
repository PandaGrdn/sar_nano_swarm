#!/usr/bin/env python3
from pathlib import Path

g = Path("/tmp/tri_fwd_gate.log").read_text(errors="replace")
p = Path("/tmp/tri_fwd_phase0.log").read_text(errors="replace")
print("===== gate =====")
for line in g.splitlines():
    if any(k in line for k in (
        "hover until", "RIO ready", "RIO not ready", "hovering for Madgwick",
        "rio_alive", "ekf_alive_cf_0", "PASS ", "FAIL ",
        "[swarm_loc_gate] PASS", "[swarm_loc_gate] FAIL",
        "flight_window",
    )):
        if "Deprecation" in line or "warnings.warn" in line:
            continue
        print(line[:260])
print("===== attitude init =====")
for line in p.splitlines():
    if "attitude initialized" in line:
        print(line[:280])
