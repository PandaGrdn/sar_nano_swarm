#!/usr/bin/env python3
import subprocess

pats = (
    "gz sim", "gz-sim", "cf2", "phase0_gate", "swarm_loc_gate",
    "swarm_loc_node", "uwb_node", "radar_noise", "rio_bridge",
    "rio_stub", "cflib", "crazyflie", "parameter_bridge",
    "ros_gz_bridge", "gz_pose_to_odom",
)
print("=== leftover procs ===")
any_hit = False
for pat in pats:
    r = subprocess.run(["pgrep", "-af", pat], capture_output=True, text=True)
    lines = [ln for ln in (r.stdout or "").splitlines()
             if "pgrep" not in ln and "_leftover" not in ln and pat.replace(" ", "") in ln.replace(" ", "")
             or (pat in ln and "_leftover" not in ln and "pgrep" not in ln)]
    # simpler: print raw if any
    raw = (r.stdout or "").strip()
    if raw:
        # filter the checker itself
        keep = [ln for ln in raw.splitlines() if "_leftover.py" not in ln]
        if keep:
            any_hit = True
            print(f"-- {pat}")
            for ln in keep:
                print(ln[:200])
if not any_hit:
    print("(none)")

r = subprocess.run(["ss", "-ulnp"], capture_output=True, text=True)
print("=== udp 1985x/1995x/2095x ===")
hits = [ln for ln in (r.stdout or "").splitlines()
        if ":1985" in ln or ":1995" in ln or ":2095" in ln]
print("\n".join(hits) if hits else "(free)")
