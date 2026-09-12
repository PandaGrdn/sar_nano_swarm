#!/usr/bin/env bash
# Launch (optional) + fly a named swarm-loc scenario.
# Artifacts: out/swarm_loc_eval/<env>/<situation>/
#
#   ./eval_scripts/run_swarm_loc_scenario.sh --list
#   ./eval_scripts/run_swarm_loc_scenario.sh tunnel/triangle_forward
#   ./eval_scripts/run_swarm_loc_scenario.sh triangle_forward --launch-sim --headless
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
# shellcheck disable=SC1091
source setup_env.sh 2>/dev/null || true

LAUNCH=false
HEADLESS=("--headless" "--no-rviz")
EXTRA_PHASE0=()
SCENARIO=""

usage() {
  cat <<'EOF'
Usage: ./eval_scripts/run_swarm_loc_scenario.sh [--launch-sim] [--gui] SCENARIO

  --launch-sim   start phase0_gate.sh for this scenario's world (blocks until ready)
  --gui          with --launch-sim, keep Gazebo GUI / RViz
  --list         print catalog
  --no-radar     only if rio.source is stub (real RIO needs /radar/points)

Examples:
  ./eval_scripts/run_swarm_loc_scenario.sh --list
  ./eval_scripts/run_swarm_loc_scenario.sh tunnel/collinear_hover
  ./eval_scripts/run_swarm_loc_scenario.sh tunnel/triangle_forward --launch-sim
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --launch-sim) LAUNCH=true; shift ;;
    --gui)        HEADLESS=(); shift ;;
    --list)       python3 eval_scripts/swarm_loc_scenarios.py --list; exit 0 ;;
    --no-radar)   EXTRA_PHASE0+=(--no-radar); shift ;;
    -h|--help)    usage; exit 0 ;;
    -*)           echo "unknown flag $1" >&2; usage; exit 2 ;;
    *)            SCENARIO="$1"; shift ;;
  esac
done

if [[ -z "$SCENARIO" ]]; then
  usage
  exit 2
fi

META="$(python3 -c "
import sys
sys.path.insert(0, 'eval_scripts')
from swarm_loc_scenarios import get_scenario, log_dir_for, eval_dir_for
s = get_scenario(sys.argv[1])
print(s['world'], s['num_drones'], s['spacing'], log_dir_for(s), eval_dir_for(s))
" "$SCENARIO")"
read -r WORLD N SPACING LOGDIR EVALDIR <<<"$META"

mkdir -p "$LOGDIR" "$EVALDIR"

# Derived estimator config: launch.positions_xyz_m = this scenario's actual
# spawn/reset geometry (triangle layouts do NOT match the stock line launch:
# block). Passed to phase0 (estimators + RIO) and to the gate below.
DERIVED_CFG="$EVALDIR/swarm_loc_derived.yaml"
python3 eval_scripts/swarm_loc_scenarios.py \
  --write-config "$SCENARIO" \
  --base configs/estimation/swarm_loc.yaml \
  --out "$DERIVED_CFG" >/dev/null
echo "[scenario] derived estimator config → $DERIVED_CFG"

if [[ "$LAUNCH" == true ]]; then
  LOG=/tmp/swarm_loc_scenario_phase0.log
  rm -f "$LOG"
  echo "[scenario] starting sim world=$WORLD n=$N …"
  nohup ./eval_scripts/phase0_gate.sh \
    -w "$WORLD" -n "$N" --spacing "$SPACING" \
    --swarm-loc-log-dir "$LOGDIR" \
    --swarm-loc-config "$DERIVED_CFG" \
    "${HEADLESS[@]}" "${EXTRA_PHASE0[@]}" \
    > "$LOG" 2>&1 &
  echo $! > /tmp/swarm_loc_scenario_phase0.pid
  ready=0
  for i in $(seq 1 180); do
    if grep -q "Simulation ready" "$LOG" 2>/dev/null; then
      echo "[scenario] Simulation ready after ${i}s"
      ready=1
      break
    fi
    sleep 1
  done
  if [[ "$ready" != 1 ]]; then
    echo "[scenario] FAIL: sim not ready"
    tail -80 "$LOG"
    exit 1
  fi
fi

python3 -u eval_scripts/swarm_loc_gate.py --scenario "$SCENARIO" \
  --config "$DERIVED_CFG" \
  --eval-dir "$EVALDIR" --logs "$LOGDIR"
echo "[scenario] eval → $EVALDIR"
