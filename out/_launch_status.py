from pathlib import Path
import os, time, subprocess
p = Path("/tmp/tri_fwd_phase0.log")
print("exists", p.is_file(), "bytes", p.stat().st_size if p.is_file() else 0)
if p.is_file():
    st = p.stat()
    print("mtime_age_s", round(time.time() - st.st_mtime, 1))
    text = p.read_text(errors="replace")
    print("has_ready", "Simulation ready" in text)
    lines = text.splitlines()
    print("nlines", len(lines))
    for ln in lines[:8]:
        print("H", ln[:200])
    print("--- tail ---")
    for ln in lines[-8:]:
        print("T", ln[:200])
r = subprocess.run(["pgrep", "-af", "phase0_gate.sh"], capture_output=True, text=True)
print("phase0", (r.stdout or "").strip()[:400] or "NONE")
r = subprocess.run(["pgrep", "-af", "gz sim"], capture_output=True, text=True)
print("gz", (r.stdout or "").strip()[:300] or "NONE")
