#!/usr/bin/env bash
# Hard-stop sim / SITL / cflib / ROS nodes. Do not use fuser (hangs on this host).
set -uo pipefail
kill_pat() { pkill -9 -f "$1" 2>/dev/null || true; }
echo "[kill] stale sim / cflib / ROS …"
kill_pat '[g]z sim'
kill_pat '[g]z-sim'
killall -9 gz 2>/dev/null || true
killall -9 cf2 2>/dev/null || true
pkill -9 -x cf2 2>/dev/null || true
kill_pat 'phase0_gate.sh'
kill_pat 'swarm_loc_gate.py'
kill_pat 'swarm_loc_node.py'
kill_pat 'swarm_loc_logger'
kill_pat 'uwb_node.py'
kill_pat 'rio_stub'
kill_pat 'flow_node.py'
kill_pat 'ros_gz_bridge'
kill_pat 'parameter_bridge'
kill_pat 'static_transform_publisher'
kill_pat 'robot_state_publisher'
kill_pat 'crazyflie_ros2'
kill_pat 'rviz2'
kill_pat 'collect_data.py'
python3 - <<'PY'
import os, re, subprocess
try:
    out = subprocess.check_output(["ss", "-ulnp"], text=True, stderr=subprocess.DEVNULL)
except Exception:
    raise SystemExit(0)
for line in out.splitlines():
    if any(p in line for p in (":19850", ":19851", ":19852", ":19950", ":19951", ":19952")):
        for pid in re.findall(r"pid=(\d+)", line):
            print("[kill] udp holder", pid, line.strip()[:100])
            try:
                os.kill(int(pid), 9)
            except ProcessLookupError:
                pass
PY
sleep 2
echo "[kill] leftover:"
pgrep -af 'gz sim|cf2|swarm_loc|uwb_node|rio_stub|phase0_gate|flow_node' || echo "  (none)"
ss -ulnp 2>/dev/null | grep -E ':1985|:1995' || echo "[kill] cflib/CfFirm ports free"
echo "[kill] done"
