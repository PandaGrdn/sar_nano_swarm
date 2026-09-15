#!/usr/bin/env python3
"""Extract phase0 live-check lines. Filename avoids pkill patterns."""
import glob
import os
from pathlib import Path

log = Path("/tmp/tri_fwd_phase0.log")
text = log.read_text(errors="replace") if log.is_file() else ""
needles = (
    "tunnel_site",
    "Building radar map",
    "cache hit",
    "triangles",
    "Radar mesh:",
    "radar_maps",
    "radar_noise",
    "IMU noise",
    "apply_imu_noise",
    "attitude initialized",
    "init restarts",
    "Simulation ready",
    "derived UWB",
    "mesh LOS",
    "runtime_uwb",
    "Spawning",
)
print("=== log hits ===")
for line in text.splitlines():
    if any(n.lower() in line.lower() for n in needles):
        print(line[:260])

print("=== radar maps ===")
for p in sorted(glob.glob("/mnt/d/GitHub/gps_denied_drones/out/radar_maps/*")):
    st = os.stat(p)
    print(f"{st.st_size:10d}  {p}")

uwb = Path("/mnt/d/GitHub/gps_denied_drones/out/runtime_uwb.yaml")
print("=== runtime uwb ===")
if uwb.is_file():
    print(uwb.read_text(encoding="utf-8", errors="replace")[:1200])
else:
    print("MISSING")
