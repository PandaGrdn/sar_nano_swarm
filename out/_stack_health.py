#!/usr/bin/env python3
"""Process + last attitude lines. Filename avoids pkill patterns."""
import subprocess
from pathlib import Path

def pg(pat: str) -> str:
    r = subprocess.run(["pgrep", "-af", pat], capture_output=True, text=True)
    out = (r.stdout or "").strip()
    return out if out else "(none)"

print("=== processes ===")
for p in ("[c]f2", "[g]z sim", "[p]hase0_gate", "[r]io_bridge", "[u]wb_node",
          "[s]warm_loc_node", "[r]adar_noise"):
    print(p, "->")
    print(pg(p)[:400])

log = Path("/tmp/tri_fwd_phase0.log")
text = log.read_text(errors="replace") if log.is_file() else ""
print("=== last attitude / ready / error ===")
hits = []
for line in text.splitlines():
    if any(k in line.lower() for k in (
        "attitude initialized", "attitude initializing", "error",
        "traceback", "died", "mesh los", "simulation ready",
    )):
        hits.append(line[:260])
for line in hits[-18:]:
    print(line)
print("n_hits", len(hits), "log_bytes", log.stat().st_size if log.is_file() else 0)
