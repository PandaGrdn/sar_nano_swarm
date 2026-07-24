#!/usr/bin/env bash
# eval_scripts/phase0_gate.sh
#
# Main simulation launcher — and Phase-0 exit gate.
#
# Starts Crazyflie SITL + radarays_gz2 radar plugin inside a Gazebo Harmonic
# world of your choosing, opens the Gazebo GUI and RViz for full visualisation,
# and prints the cfclient connection URI so you can fly manually.
#
# Usage:
#   cd <repo_root>
#   ./eval_scripts/phase0_gate.sh [OPTIONS]
#
# Options:
#   -w, --world  WORLD   World name (no .sdf) searched in sim_worlds/, OR an
#                        absolute path to any .sdf file.
#                        Built-in: phase0_tunnel_gate (default), crazysim_default
#   -m, --model  MODEL   crazyflie | crazyflie_thrust_upgrade  (default: crazyflie)
#   -x X                 Spawn X position in metres (default: 0)
#   -y Y                 Spawn Y position in metres (default: 0)
#       --mesh PATH      Mesh file for radar raycasting.
#                        Relative paths are resolved against $SAR_NANO_SWARM_ROOT.
#                        Defaults are auto-detected for built-in worlds; for custom
#                        worlds you must provide this or pass --no-radar.
#       --no-radar       Skip radar plugin injection entirely.
#       --no-payload      Skip mass/inertia payload rewrite (apply_payload.py).
#       --payload-config PATH
#                         Path to payload.yaml [default: configs/airframe/payload.yaml]
#       --no-tof         Skip IR ToF rangefinder sensor injection (apply_tof_sensor.py).
#       --tof-config PATH
#                         Path to tof.yaml [default: configs/sensors/tof.yaml]
#       --no-rviz        Skip RViz launch.
#       --headless       Skip Gazebo GUI (server + SITL only, useful for CI).
#       --check          Gate-check mode: start headless, wait 15 s, verify
#                        /radar/points publishes ≥ 8 Hz, then exit 0/1.
#                        Implies --no-rviz --headless.
#   -h, --help           Show this help and exit.
#
# Environment overrides:
#   CRAZYSIM_FW        Full path to the cf2 binary.
#                      Default: <repo>/firmware_mods/CrazySim/crazyflie-firmware/sitl_make/build/cf2
#   RADAR_PLUGIN_DIR   Dir containing libradar_sensor_system.so.
#                      Default: <repo>/install/radarays_gz2/lib
#
# cfclient connection URI printed at startup:
#   udp://127.0.0.1:19850   (drone ID 0)
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

# ── helpers ──────────────────────────────────────────────────────────────────
info()  { echo -e "\033[1;34m[sim]\033[0m $*"; }
warn()  { echo -e "\033[1;33m[sim]\033[0m $*" >&2; }
die()   { echo -e "\033[1;31m[sim] ERROR:\033[0m $*" >&2; exit 1; }

usage() {
cat <<'EOF'
Usage: ./eval_scripts/phase0_gate.sh [OPTIONS]

  -w, --world  WORLD   World name (no .sdf) or absolute .sdf path
                       [default: phase0_tunnel_gate]
  -m, --model  MODEL   crazyflie | crazyflie_thrust_upgrade  [default: crazyflie]
  -x X                 Spawn X  [default: 0]
  -y Y                 Spawn Y  [default: 0]
      --mesh PATH      Mesh for radar raycasting (rel to SAR_NANO_SWARM_ROOT)
      --no-radar       Skip radar plugin
      --no-payload     Skip mass/inertia payload rewrite
      --payload-config PATH  payload.yaml to use [default: configs/airframe/payload.yaml]
      --no-tof         Skip IR ToF rangefinder sensor injection
      --tof-config PATH  tof.yaml to use [default: configs/sensors/tof.yaml]
      --no-rviz        Skip RViz
      --headless       Skip Gazebo GUI
      --check          Headless gate-check (prints PASS/FAIL)
  -h, --help           This message
EOF
}

# ── argument defaults ─────────────────────────────────────────────────────────
WORLD="phase0_tunnel_gate"
MODEL="crazyflie"
SPAWN_X=0
SPAWN_Y=0
MESH_ARG=""
USE_RADAR=true
USE_PAYLOAD=true
PAYLOAD_CONFIG=""
USE_TOF=true
TOF_CONFIG=""
USE_RVIZ=true
USE_GUI=true
GATE_CHECK=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    -w|--world)   WORLD="$2";       shift 2 ;;
    -m|--model)   MODEL="$2";       shift 2 ;;
    -x)           SPAWN_X="$2";     shift 2 ;;
    -y)           SPAWN_Y="$2";     shift 2 ;;
    --mesh)       MESH_ARG="$2";    shift 2 ;;
    --no-radar)   USE_RADAR=false;  shift   ;;
    --no-payload) USE_PAYLOAD=false; shift  ;;
    --payload-config) PAYLOAD_CONFIG="$2"; shift 2 ;;
    --no-tof)     USE_TOF=false;   shift   ;;
    --tof-config) TOF_CONFIG="$2"; shift 2 ;;
    --no-rviz)    USE_RVIZ=false;   shift   ;;
    --headless)   USE_GUI=false;    shift   ;;
    --check)      GATE_CHECK=true; USE_RVIZ=false; USE_GUI=false; shift ;;
    -h|--help)    usage; exit 0 ;;
    *) die "Unknown option: $1" ;;
  esac
done

# ── locate repo root ──────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Allow override via env (useful when running from a different working dir)
export SAR_NANO_SWARM_ROOT="${SAR_NANO_SWARM_ROOT:-$REPO_ROOT}"

# ── source environment ────────────────────────────────────────────────────────
info "Sourcing setup_env.sh …"
# shellcheck disable=SC1091
source "$SAR_NANO_SWARM_ROOT/setup_env.sh"

# ── locate CrazySim paths ─────────────────────────────────────────────────────
CRAZYSIM_DIR="$SAR_NANO_SWARM_ROOT/firmware_mods/CrazySim/crazyflie-firmware"
[[ -d "$CRAZYSIM_DIR" ]] || die "CrazySim not found at $CRAZYSIM_DIR"

BUILD_DIR="$CRAZYSIM_DIR/sitl_make/build"
JINJA_GEN="$CRAZYSIM_DIR/tools/crazyflie-simulation/simulator_files/gazebo/launch/jinja_gen.py"
SETUP_GZ="$CRAZYSIM_DIR/tools/crazyflie-simulation/simulator_files/gazebo/launch/setup_gz.bash"
MODELS_DIR="$CRAZYSIM_DIR/tools/crazyflie-simulation/simulator_files/gazebo/models"
WORLDS_DIR="$CRAZYSIM_DIR/tools/crazyflie-simulation/simulator_files/gazebo/worlds"

[[ -f "$JINJA_GEN" ]]  || die "jinja_gen.py not found: $JINJA_GEN"
[[ -f "$SETUP_GZ" ]]   || die "setup_gz.bash not found: $SETUP_GZ"

# shellcheck disable=SC1090
source "$SETUP_GZ" "$CRAZYSIM_DIR" "$BUILD_DIR"

# ── locate cf2 binary ─────────────────────────────────────────────────────────
CF2_BIN="${CRAZYSIM_FW:-$BUILD_DIR/cf2}"
[[ -f "$CF2_BIN" ]] || die "cf2 binary not found: $CF2_BIN
  Build with:  cd $CRAZYSIM_DIR/sitl_make && make
  Or set:      export CRAZYSIM_FW=/path/to/cf2"

# ── optional: override radar plugin dir ──────────────────────────────────────
if [[ -n "${RADAR_PLUGIN_DIR:-}" ]]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$RADAR_PLUGIN_DIR:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
  export LD_LIBRARY_PATH="$RADAR_PLUGIN_DIR:${LD_LIBRARY_PATH:-}"
fi

# ── resolve world SDF ─────────────────────────────────────────────────────────
if [[ "$WORLD" == /* ]]; then
  WORLD_SDF="$WORLD"
elif [[ -f "$SAR_NANO_SWARM_ROOT/sim_worlds/${WORLD}.sdf" ]]; then
  WORLD_SDF="$SAR_NANO_SWARM_ROOT/sim_worlds/${WORLD}.sdf"
elif [[ -f "$SAR_NANO_SWARM_ROOT/sim_worlds/${WORLD}.world" ]]; then
  WORLD_SDF="$SAR_NANO_SWARM_ROOT/sim_worlds/${WORLD}.world"
elif [[ -f "$WORLDS_DIR/${WORLD}.sdf" ]]; then
  WORLD_SDF="$WORLDS_DIR/${WORLD}.sdf"
elif [[ -f "$WORLDS_DIR/${WORLD}.world" ]]; then
  WORLD_SDF="$WORLDS_DIR/${WORLD}.world"
else
  die "World file not found for '${WORLD}'.
  Searched (.sdf and .world) in:
    $SAR_NANO_SWARM_ROOT/sim_worlds/
    $WORLDS_DIR/
  Pass an absolute path with -w /path/to/world.sdf|.world"
fi

info "World SDF: $WORLD_SDF"

# Extract the world name attribute from the SDF (used in gz service path).
WORLD_NAME=$(python3 - "$WORLD_SDF" <<'PYEOF'
import sys, xml.etree.ElementTree as ET
tree = ET.parse(sys.argv[1])
root = tree.getroot()
w = root.find('world')
print(w.get('name') if w is not None else 'unknown')
PYEOF
)
info "World name: $WORLD_NAME"

# ── resolve radar mesh ────────────────────────────────────────────────────────
# Default meshes keyed by world name (relative to SAR_NANO_SWARM_ROOT).
declare -A _DEFAULT_MESHES=(
  ["phase0_tunnel_gate"]="sim_worlds/darpa_subt_worlds/worlds/models/cave_world/meshes/cave_world.obj"
  ["phase1_pid_tune"]=""    # flat/open world, no geometry to raycast against — use --no-radar
  ["crazysim_default"]=""
)

if [[ "$USE_RADAR" == true ]]; then
  MESH_PATH="${MESH_ARG:-${_DEFAULT_MESHES[$WORLD_NAME]:-}}"

  if [[ -z "$MESH_PATH" ]]; then
    warn "No mesh path for world '$WORLD_NAME'. Disabling radar."
    warn "Pass --mesh <path> to enable it, or --no-radar to suppress this warning."
    USE_RADAR=false
  else
    # Verify the mesh file is reachable
    _resolved_mesh="$MESH_PATH"
    [[ "$MESH_PATH" != /* ]] && _resolved_mesh="$SAR_NANO_SWARM_ROOT/$MESH_PATH"
    if [[ ! -f "$_resolved_mesh" ]]; then
      warn "Mesh file not found: $_resolved_mesh"
      warn "Disabling radar. Check GZ_SIM_RESOURCE_PATH or the submodule checkout."
      USE_RADAR=false
    else
      info "Radar mesh: $MESH_PATH"
    fi
  fi
fi

# ── cleanup trap ──────────────────────────────────────────────────────────────
_PIDS=()
cleanup() {
  info "Shutting down …"
  for pid in "${_PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  pkill -x cf2 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "rviz2.*radar" 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM EXIT

# ── kill stale instances ──────────────────────────────────────────────────────
info "Stopping any running cf2 instances …"
pkill -x cf2 2>/dev/null || true
sleep 1

# ── start Gazebo server ───────────────────────────────────────────────────────
info "Starting Gazebo server (world: $WORLD_NAME) …"
gz sim -s -r "$WORLD_SDF" -v 3 &
_PIDS+=($!)
GZ_SERVER_PID=${_PIDS[-1]}

# Wait until gz is responsive: poll topic list until the world clock appears.
info "Waiting for Gazebo to initialise …"
_gz_ready=false
for _i in $(seq 1 30); do
  if gz topic -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/clock"; then
    _gz_ready=true
    break
  fi
  sleep 1
done
[[ "$_gz_ready" == true ]] || warn "Gazebo did not respond after 30 s — continuing anyway."

# ── generate Crazyflie SDF ────────────────────────────────────────────────────
CF_ID=0
CFLIB_PORT=$((19850 + CF_ID))
CFFIRM_PORT=$((19950 + CF_ID))
SDF_TMP="/tmp/${MODEL}_${CF_ID}.sdf"

rm -f "$SDF_TMP"
mkdir -p "$BUILD_DIR/$CF_ID"
info "Generating Crazyflie SDF …"
python3 "$JINJA_GEN" \
  "$MODELS_DIR/${MODEL}/model.sdf.jinja" \
  "$MODELS_DIR" \
  --cffirm_udp_port "$CFFIRM_PORT" \
  --cflib_udp_port  "$CFLIB_PORT" \
  --cf_id           "$CF_ID" \
  --cf_name         "cf" \
  --output-file     "$SDF_TMP"

# ── inject radar plugin ───────────────────────────────────────────────────────
if [[ "$USE_RADAR" == true ]]; then
  info "Injecting radarays_gz2 plugin (mesh: $MESH_PATH) …"
  python3 - "$SDF_TMP" "$MESH_PATH" <<'PYEOF'
import sys, xml.etree.ElementTree as ET

ET.register_namespace('', 'http://sdformat.org/schemas/root.xsd')
tree = ET.parse(sys.argv[1])
root = tree.getroot()
model = root.find('model')
if model is None:
    print("[radar-inject] ERROR: no <model> element found", file=sys.stderr)
    sys.exit(1)

plugin = ET.SubElement(model, 'plugin')
plugin.set('filename', 'radar_sensor_system')
plugin.set('name', 'radarays_gz2::RadarSensorSystem')
mesh_elem = ET.SubElement(plugin, 'mesh_path')
mesh_elem.text = sys.argv[2]

tree.write(sys.argv[1], encoding='unicode')
print(f"[radar-inject] Plugin injected into {sys.argv[1]}")
PYEOF
fi

# ── rewrite mass/CoM/inertia for the loaded payload (Phase 1 step 1) ─────────
# Recomputes base_link's <inertial> from configs/airframe/payload.yaml (base
# airframe + every enabled sensor/compute deck) so downstream flight dynamics,
# PID tuning, and the thrust-margin check reflect the real loaded drone, not
# the stock 27 g default. See
# .cursor/docs/Phase1_Physical_Fidelity_and_Sensor_Implementation_Plan.md
if [[ "$USE_PAYLOAD" == true ]]; then
  _payload_cfg="${PAYLOAD_CONFIG:-$SAR_NANO_SWARM_ROOT/configs/airframe/payload.yaml}"
  [[ "$_payload_cfg" != /* ]] && _payload_cfg="$SAR_NANO_SWARM_ROOT/$_payload_cfg"
  if [[ ! -f "$_payload_cfg" ]]; then
    warn "Payload config not found: $_payload_cfg — skipping mass/inertia rewrite."
  else
    info "Applying payload mass/inertia model ($_payload_cfg) …"
    python3 "$SAR_NANO_SWARM_ROOT/eval_scripts/apply_payload.py" "$SDF_TMP" --payload "$_payload_cfg"

    info "Checking thrust margin …"
    python3 "$SAR_NANO_SWARM_ROOT/eval_scripts/thrust_margin_check.py" "$SDF_TMP" \
      --config "$SAR_NANO_SWARM_ROOT/configs/airframe/thrust_margin.yaml" \
      || warn "Thrust-margin check failed — drone may be under-thrusted for this payload."
  fi
fi

# ── inject IR ToF rangefinder sensor(s) (Phase 1 M3, §4.1) ───────────────────
# Pure-geometry gpu_lidar beam(s) into base_link. Mass already accounted for
# in payload.yaml (flow_deck_v2 component) — this only adds the sensor element.
if [[ "$USE_TOF" == true ]]; then
  _tof_cfg="${TOF_CONFIG:-$SAR_NANO_SWARM_ROOT/configs/sensors/tof.yaml}"
  [[ "$_tof_cfg" != /* ]] && _tof_cfg="$SAR_NANO_SWARM_ROOT/$_tof_cfg"
  if [[ ! -f "$_tof_cfg" ]]; then
    warn "ToF config not found: $_tof_cfg — skipping sensor injection."
  else
    info "Injecting IR ToF sensor(s) ($_tof_cfg) …"
    python3 "$SAR_NANO_SWARM_ROOT/eval_scripts/apply_tof_sensor.py" "$SDF_TMP" \
      --config "$_tof_cfg" --cf-id "$CF_ID"
  fi
fi

# ── spawn Crazyflie in Gazebo ─────────────────────────────────────────────────
info "Spawning ${MODEL}_${CF_ID} at (${SPAWN_X}, ${SPAWN_Y}) …"
gz service \
  -s "/world/${WORLD_NAME}/create" \
  --reqtype  gz.msgs.EntityFactory \
  --reptype  gz.msgs.Boolean \
  --timeout  5000 \
  --req "sdf_filename: \"${SDF_TMP}\",
         pose: {position: {x: ${SPAWN_X}, y: ${SPAWN_Y}, z: 0.5}},
         name: \"${MODEL}_${CF_ID}\",
         allow_renaming: 1"

# ── wait for drone sensors to come online before starting firmware ───────────
# In heavy worlds (e.g. cave), the drone's gz sensor plugins take time to init.
# Starting cf2 too early drops the first IMU packets and the estimator never
# recovers (canfly stays 0, drone won't arm). Wait until /cf_0/odom publishes.
info "Waiting for drone sensors (/cf_${CF_ID}/odom) to come online …"
_drone_ready=false
for _i in $(seq 1 30); do
  if timeout 2 gz topic -e -t "/cf_${CF_ID}/odom" -n 1 >/dev/null 2>&1; then
    _drone_ready=true
    break
  fi
  sleep 1
done
if [[ "$_drone_ready" == true ]]; then
  info "Drone sensors publishing. Giving them 2s to stabilise …"
  sleep 2
else
  warn "Drone odom not detected after 30s — starting firmware anyway."
fi

# ── CRITICAL: cf2 needs CF2_SIM_MODEL to bind its SITL sensor interface to the
# gz drone. Without it, IMU never reaches the firmware and the estimator locks.
export CF2_SIM_MODEL="gz_${MODEL}"

# ── start SITL firmware ───────────────────────────────────────────────────────
info "Starting SITL firmware (instance ${CF_ID}) …"
pushd "$BUILD_DIR/$CF_ID" > /dev/null
"$CF2_BIN" "$CFFIRM_PORT" > out.log 2> error.log &
_PIDS+=($!)
popd > /dev/null

# ── optional: bridge ToF gz topic(s) to ROS 2 for Phase-2 EKF consumption ────
# ros_gz_bridge lives in the separate source-built workspace sourced by
# setup_env.sh (${ROS_GZ_WS:-$HOME/ros2_ws}/install) — NOT the apt package, so
# `apt-cache policy ros-humble-ros-gz-bridge` alone will look like it's
# missing even when it's actually available here. Verified live (2026-07-21):
# bridges /cf_${CF_ID}/tof_down (gz.msgs.LaserScan) -> ROS
# sensor_msgs/msg/LaserScan, confirmed publishing at ~27-28 Hz via
# `ros2 topic hz`.
if [[ "$USE_TOF" == true ]] && command -v ros2 &>/dev/null && ros2 pkg prefix ros_gz_bridge &>/dev/null; then
  info "Bridging /cf_${CF_ID}/tof_down to ROS 2 (sensor_msgs/msg/LaserScan) …"
  ros2 run ros_gz_bridge parameter_bridge \
    "/cf_${CF_ID}/tof_down@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" &
  _PIDS+=($!)
elif [[ "$USE_TOF" == true ]]; then
  warn "ros_gz_bridge not installed — /cf_${CF_ID}/tof_down is gz-native only (no ROS topic)."
  warn "Install with: sudo apt install ros-humble-ros-gz-bridge"
fi

# ── launch RViz ───────────────────────────────────────────────────────────────
RVIZ_CFG="$SAR_NANO_SWARM_ROOT/configs/rviz/radar.rviz"

if [[ "$USE_RVIZ" == true ]]; then
  if ! command -v rviz2 &>/dev/null; then
    warn "rviz2 not found — skipping RViz launch."
  else
    info "Launching RViz (config: $RVIZ_CFG) …"
    if [[ -f "$RVIZ_CFG" ]]; then
      rviz2 -d "$RVIZ_CFG" &
    else
      warn "RViz config not found ($RVIZ_CFG), launching with defaults."
      rviz2 &
    fi
    _PIDS+=($!)
  fi
fi

# ── static TF: world → odom → base_link → radar_link ─────────────────────────
# The radar plugin publishes in 'radar_link'. Until a full TF tree is wired up
# in Phase 2, broadcast a static transform so RViz can display the cloud.
if command -v ros2 &>/dev/null; then
  ros2 run tf2_ros static_transform_publisher \
    --frame-id base_link --child-frame-id radar_link \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 &
  _PIDS+=($!)

  ros2 run tf2_ros static_transform_publisher \
    --frame-id world --child-frame-id base_link \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 &
  _PIDS+=($!)
fi

# ── print connection info ─────────────────────────────────────────────────────
echo ""
echo "  ╔═══════════════════════════════════════════════╗"
echo "  ║          Simulation ready                     ║"
echo "  ╠═══════════════════════════════════════════════╣"
echo "  ║  cfclient URI : udp://127.0.0.1:${CFLIB_PORT}      ║"
echo "  ║  World        : ${WORLD_NAME}"
echo "  ║  Model        : ${MODEL}_${CF_ID}"
echo "  ║  Radar        : ${USE_RADAR}"
echo "  ║  Payload model: ${USE_PAYLOAD}"
echo "  ║  ToF sensor   : ${USE_TOF}"
echo "  ║  Radar topic  : /radar/points  (~10 Hz)"
echo "  ║  ToF topic    : /cf_${CF_ID}/tof_down  (gz-native, ~30 Hz)"
echo "  ╚═══════════════════════════════════════════════╝"
echo ""

# ── launch Gazebo GUI ─────────────────────────────────────────────────────────
if [[ "$USE_GUI" == true ]]; then
  info "Starting Gazebo GUI …"
  gz sim -g &
  _PIDS+=($!)
fi

# ── gate-check mode ───────────────────────────────────────────────────────────
if [[ "$GATE_CHECK" == true ]]; then
  info "Gate-check mode: waiting 15 s for /radar/points to stabilise …"
  sleep 15

  if ! command -v ros2 &>/dev/null; then
    die "--check requires ros2 on PATH (source setup_env.sh first)."
  fi

  info "Sampling /radar/points for 5 s …"
  HZ_OUTPUT=$(ros2 topic hz /radar/points --window 10 2>&1 &
              HZ_PID=$!
              sleep 5
              kill $HZ_PID 2>/dev/null || true
              wait $HZ_PID 2>/dev/null || true)

  MEASURED_HZ=$(echo "$HZ_OUTPUT" | grep -oP 'average rate: \K[0-9.]+' | tail -1)

  if [[ -z "$MEASURED_HZ" ]]; then
    echo ""
    echo "  [GATE] FAIL — /radar/points not detected (check plugin build and mesh path)"
    exit 1
  fi

  # Pass if measured rate >= 8 Hz (allows some jitter below the 10 Hz target).
  if python3 -c "import sys; sys.exit(0 if float('${MEASURED_HZ}') >= 8.0 else 1)"; then
    echo ""
    echo "  [GATE] PASS — /radar/points @ ${MEASURED_HZ} Hz  (target ≥ 8 Hz)"
    exit 0
  else
    echo ""
    echo "  [GATE] FAIL — /radar/points @ ${MEASURED_HZ} Hz  (target ≥ 8 Hz)"
    exit 1
  fi
fi

# ── interactive: wait for Ctrl-C ─────────────────────────────────────────────
info "Simulation running. Press Ctrl-C to stop all processes."
wait
