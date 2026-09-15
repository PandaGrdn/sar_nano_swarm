#!/usr/bin/env python3
"""Poll gate log until done. Filename avoids pkill patterns."""
import time
from pathlib import Path

log = Path("/tmp/tri_fwd_gate.log")
done = (
    "traceback (most recent",
    "all checks passed",
    "checks failed",
    "mlflow run",
    "[gate] pass",
    "[gate] fail",
    "gate: pass",
    "gate: fail",
    "[swarm_loc_gate] pass",
    "[swarm_loc_gate] fail",
)
deadline = time.time() + 900
last_n = 0
while time.time() < deadline:
    text = log.read_text(errors="replace") if log.is_file() else ""
    lines = text.splitlines()
    if len(lines) != last_n:
        print(f"--- {len(lines)} lines ---")
        shown = [
            ln for ln in lines
            if "DeprecationWarning" not in ln
            and "legacy TYPE_HOVER" not in ln
            and "warnings.warn" not in ln
        ]
        for ln in shown[-12:]:
            print(ln[:240])
        last_n = len(lines)
    low = text.lower()
    if any(k in low for k in done):
        print("DONE_MARKER")
        break
    time.sleep(5)
else:
    print("WAIT_TIMEOUT")
print("bytes", log.stat().st_size if log.is_file() else 0)
