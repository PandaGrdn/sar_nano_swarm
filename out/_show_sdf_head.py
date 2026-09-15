from pathlib import Path
p = Path("/mnt/d/GitHub/gps_denied_drones/sim_worlds/phase0_tunnel_gate.sdf")
text = p.read_text(encoding="utf-8")
print(repr(text[:500]))
print("---")
for i, line in enumerate(text.splitlines(), 1):
    if i <= 6:
        print(f"{i:2d} ({len(line):3d}) {line}")
        if i == 4:
            print("    col70:", repr(line[65:80] if len(line) > 65 else line))
