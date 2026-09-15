#!/usr/bin/env python3
import sys
import time
from pathlib import Path

log = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/tri_fwd_phase0.log")
max_s = int(sys.argv[2] if len(sys.argv) > 2 else 420)
needle = "Simulation ready"
for i in range(1, max_s + 1):
    try:
        text = log.read_text(errors="replace")
    except OSError:
        text = ""
    if needle in text:
        print(f"READY after {i}s")
        sys.exit(0)
    time.sleep(1)
print(f"NOT_READY after {max_s}s")
if log.is_file():
    print("\n".join(log.read_text(errors="replace").splitlines()[-40:]))
sys.exit(1)
