from pathlib import Path
p = Path("/mnt/d/GitHub/gps_denied_drones/out/_launch_phase0.sh")
p.write_bytes(p.read_bytes().replace(b"\r\n", b"\n"))
print("lf_ok")
