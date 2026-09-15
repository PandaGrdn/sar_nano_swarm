from pathlib import Path
text = Path("/tmp/tri_fwd_phase0.log").read_text(errors="replace")
keys = ("attitude", "imu_radar", "RIO", "rio_bridge", "ERROR", "Traceback", "died", "init restart")
print("=== rio/fusion ===")
n = 0
for line in text.splitlines():
    if any(k.lower() in line.lower() for k in keys):
        print(line[:280])
        n += 1
print("lines", n)
