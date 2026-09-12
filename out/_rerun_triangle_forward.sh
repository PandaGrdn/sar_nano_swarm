#!/usr/bin/env bash
set -euo pipefail
cd /mnt/d/GitHub/gps_denied_drones
# shellcheck disable=SC1091
source setup_env.sh
bash out/_kill_sim_stack.sh
rm -rf /tmp/cflib_cache
mkdir -p /tmp/cflib_cache
mkdir -p out/swarm_loc_logs/tunnel/triangle_forward \
         out/swarm_loc_eval/tunnel/triangle_forward
# Estimator config with the triangle's actual launch positions (D13).
DERIVED_CFG=out/swarm_loc_eval/tunnel/triangle_forward/swarm_loc_derived.yaml
python3 eval_scripts/swarm_loc_scenarios.py \
  --write-config tunnel/triangle_forward \
  --base configs/estimation/swarm_loc.yaml \
  --out "$DERIVED_CFG" >/dev/null
echo "[retry] derived estimator config → $DERIVED_CFG"
LOG=/tmp/tri_fwd_phase0.log
rm -f "$LOG"
nohup ./eval_scripts/phase0_gate.sh \
  -w phase0_tunnel_gate -n 3 --spacing 0.9 \
  --headless --no-rviz \
  --swarm-loc-log-dir out/swarm_loc_logs/tunnel/triangle_forward \
  --swarm-loc-config "$DERIVED_CFG" \
  > "$LOG" 2>&1 &
echo $! > /tmp/tri_fwd_phase0.pid
ready=0
for i in $(seq 1 420); do
  if grep -q "Simulation ready" "$LOG" 2>/dev/null; then
    echo "[retry] Simulation ready after ${i}s"
    ready=1
    break
  fi
  sleep 1
done
if [[ "$ready" != 1 ]]; then
  echo "[retry] SIM_FAIL"
  tail -40 "$LOG"
  bash out/_kill_sim_stack.sh
  exit 1
fi
echo "[retry] extra SITL settle 20 s"
sleep 20
echo "[retry] probe cf0 45s"
set +e
timeout 45 python3 -u - <<'PY'
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
cflib.crtp.init_drivers()
print("probe connect udp://127.0.0.1:19850", flush=True)
with SyncCrazyflie("udp://127.0.0.1:19850", cf=Crazyflie(rw_cache="/tmp/cflib_cache")) as scf:
    print("probe cf0 OK", flush=True)
PY
probe=$?
set -e
echo "[retry] probe_rc=$probe"
if [[ "$probe" != 0 ]]; then
  echo "[retry] cf0 probe failed — not flying"
  bash out/_kill_sim_stack.sh
  exit 2
fi
set +e
python3 -u eval_scripts/swarm_loc_gate.py \
  --scenario tunnel/triangle_forward \
  --config "$DERIVED_CFG" \
  --connect-timeout 180 \
  --eval-dir out/swarm_loc_eval/tunnel/triangle_forward \
  --logs out/swarm_loc_logs/tunnel/triangle_forward
rc=$?
set -e
echo "[retry] GATE_RC=$rc"
bash out/_kill_sim_stack.sh
exit "$rc"
