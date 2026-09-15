#!/usr/bin/env bash
# Launch helper. Filename avoids pkill patterns.
set -euo pipefail
cd /mnt/d/GitHub/gps_denied_drones
source setup_env.sh
python3 -c "import xml.etree.ElementTree as ET; ET.parse('sim_worlds/phase0_tunnel_gate.sdf'); print('SDF_OK')"
rm -f /tmp/tri_fwd_phase0.log
setsid nohup ./eval_scripts/phase0_gate.sh \
  -w phase0_tunnel_gate -n 3 --spacing 0.9 \
  --headless --no-rviz \
  --swarm-loc-log-dir out/swarm_loc_logs/tunnel/triangle_forward \
  --swarm-loc-config out/swarm_loc_eval/tunnel/triangle_forward/swarm_loc_derived.yaml \
  > /tmp/tri_fwd_phase0.log 2>&1 < /dev/null &
echo LAUNCH_PID=$!
sleep 3
wc -l /tmp/tri_fwd_phase0.log
pgrep -af "[p]hase0_gate.sh" || echo NO_PHASE0
