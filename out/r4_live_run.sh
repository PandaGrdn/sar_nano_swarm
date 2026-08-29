#!/usr/bin/env bash
# Headless live fly + §6.1 metrics. Kill leftovers first so cflib UDP is free.
set -euo pipefail
cd /mnt/d/GitHub/gps_denied_drones
# shellcheck disable=SC1091
source setup_env.sh

kill_pat() { pkill -9 -f "$1" 2>/dev/null || true; }

echo "[r4] killing stale sim / cflib / ROS nodes …"
kill_pat '[g]z sim'
kill_pat '[g]z-sim-server'
kill_pat '[g]z-sim'
killall -9 gz 2>/dev/null || true
killall -9 cf2 2>/dev/null || true
kill_pat '[s]warm_loc_gate.py'
kill_pat '[s]warm_loc_node.py'
kill_pat '[s]warm_loc_logger'
kill_pat '[u]wb_node.py'
kill_pat '[r]io_stub'
kill_pat '[f]low_node.py'
kill_pat '[c]razyflie_ros2'
kill_pat '[r]obot_state_publisher'
kill_pat '[p]hase0_gate.sh'
# cflib / SITL UDP
for p in 19850 19851 19852 19853 11311; do
  fuser -k "${p}/tcp" 2>/dev/null || true
  fuser -k "${p}/udp" 2>/dev/null || true
done
sleep 2
echo "[r4] leftover matching processes:"
pgrep -af 'gz sim|cf2|swarm_loc|uwb_node|rio_stub' || echo "  (none)"

rm -rf out/swarm_loc_logs
mkdir -p out/swarm_loc_logs out/swarm_loc_eval
LOG=/tmp/r4_phase0.log
rm -f "$LOG"
echo "[r4] starting headless phase0 (server only, no GUI, no RViz) …"
nohup ./eval_scripts/phase0_gate.sh \
  -w phase0_tunnel_gate -n 3 --spacing 1.5 \
  --headless --no-rviz --no-radar \
  --swarm-loc-log-dir out/swarm_loc_logs \
  > "$LOG" 2>&1 &
echo $! > /tmp/r4_phase0.pid

ready=0
for i in $(seq 1 120); do
  if grep -q "Simulation ready" "$LOG" 2>/dev/null; then
    echo "[r4] Simulation ready after ${i}s"
    ready=1
    break
  fi
  sleep 1
done
if [[ "$ready" != 1 ]]; then
  echo "[r4] FAIL: sim did not become ready"
  tail -100 "$LOG"
  exit 1
fi

GATE_LOG=/tmp/r4_gate.log
rm -f "$GATE_LOG"
set +e
python3 -u eval_scripts/swarm_loc_gate.py \
  --num-drones 3 --duration 90 --connect-timeout 90 \
  --eval-dir out/swarm_loc_eval --logs out/swarm_loc_logs \
  | tee "$GATE_LOG"
rc=${PIPESTATUS[0]}
set -e

if grep -qE "timed out|Falling back to --no-fly|cflib parallel open timed out" "$GATE_LOG"; then
  echo "[r4] FAIL: cflib did not fly (timeout or no-fly fallback). Not a valid R4 live run."
  exit 2
fi
if [[ $rc -ne 0 ]]; then
  echo "[r4] gate exit $rc"
  exit $rc
fi
echo "[r4] gate finished OK"
python3 - <<'PY'
import json
from pathlib import Path
p = Path("out/swarm_loc_eval/metrics_6_1.json")
r = json.loads(p.read_text())
print("=== live metrics (max_cov_p_m from yaml at node start) ===")
for i, m in r["per_drone"].items():
    print(
        f"  cf_{i}: ATE={m.get('ate_rmse_m'):.3f}  yaw={m.get('yaw_rmse_deg'):.2f} deg  "
        f"mean_z={m.get('mean_z_m', float('nan')):+.3f} m  NEES={m.get('mean_nees'):.2f}"
    )
mix = r.get("mix") or {}
print(
    f"  n_entrance_obs={mix.get('n_entrance_obs')}  "
    f"n_entrance_mutual_yaw={mix.get('n_entrance_mutual_yaw')}  "
    f"NIS reject={mix.get('nis_reject_rate'):.4f}"
)
nis = (r.get("diag") or {}).get("nis_by_type") or {}
for k in ("mutual_yaw", "reciprocal_relpos", "entrance_obs_relpos", "entrance_mutual_yaw", "relpos"):
    v = nis.get(k)
    if not v:
        print(f"  NIS {k}: (absent)")
        continue
    print(
        f"  NIS {k}: n={v.get('n')} mean={v.get('mean'):.3g} p50={v.get('p50'):.3g} "
        f"rej={v.get('n_reject')}"
    )
PY
