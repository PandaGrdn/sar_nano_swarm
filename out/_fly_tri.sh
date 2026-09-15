#!/usr/bin/env bash
# Fly helper. Filename avoids pkill patterns.
set -euo pipefail
cd /mnt/d/GitHub/gps_denied_drones
source setup_env.sh
rm -f /tmp/tri_fwd_gate.log
exec python3 -u eval_scripts/swarm_loc_gate.py --scenario tunnel/triangle_forward \
  --config out/swarm_loc_eval/tunnel/triangle_forward/swarm_loc_derived.yaml \
  --eval-dir out/swarm_loc_eval/tunnel/triangle_forward \
  --logs out/swarm_loc_logs/tunnel/triangle_forward \
  --connect-timeout 180 --connect-wait 180
