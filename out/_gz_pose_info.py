#!/usr/bin/env python3
"""Dump gz set_pose service info. Filename avoids pkill patterns."""
import subprocess
print("=== service -i ===")
r = subprocess.run(
    ["gz", "service", "-i", "-s", "/world/phase0_tunnel_gate/set_pose"],
    capture_output=True, text=True, timeout=8,
)
print((r.stdout or "")[:2500])
print((r.stderr or "")[:500])
