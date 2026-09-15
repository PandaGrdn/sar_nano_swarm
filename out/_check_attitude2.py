from pathlib import Path
text = Path("/tmp/tri_fwd_phase0.log").read_text(errors="replace")
print("=== attitude / mesh LOS ===")
for line in text.splitlines():
    if "attitude initialized" in line or "mesh LOS" in line or "MeshLos" in line:
        print(line[:300])
