from pathlib import Path
import time, subprocess
p = Path("/tmp/tri_fwd_gate.log")
print("bytes", p.stat().st_size if p.is_file() else 0)
print("age_s", round(time.time() - p.stat().st_mtime, 1) if p.is_file() else None)
r = subprocess.run(["pgrep", "-af", "swarm_loc_gate.py"], capture_output=True, text=True)
keep = [ln for ln in (r.stdout or "").splitlines() if "pgrep" not in ln]
print("gate", "\n".join(keep)[:400] or "NONE")
text = p.read_text(errors="replace") if p.is_file() else ""
for needle in ("hover until", "RIO ready", "RIO not ready", "drone 0 connected", "relevel"):
    print(needle, text.count(needle))
