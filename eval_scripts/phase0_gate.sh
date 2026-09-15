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
#   -x X                 Spawn X position in metres (default: site or 0)
#   -y Y                 Spawn Y position in metres (default: site or 0)
#   -z Z                 Spawn Z position in metres (default: site or 0.5)
#   -n, --num-drones N    Number of Crazyflie drones to spawn (default: 1)
#       --spacing M      Metres between drone spawn points on X axis (default: 1.5)
#       --spawn-positions "x,y,z;x,y,z;..."
#                        Explicit per-drone spawn poses (one x,y,z entry per
#                        drone, world frame), used verbatim instead of the
#                        SPAWN_X + i*SPACING line. Entry count must equal -n.
#                        Spawned with identity tilt and the tunnel site's yaw
#                        (configs/sim/tunnel_site.yaml spawn.yaw_deg) when the
#                        site file is present. Used for tunnel launch-pad rest
#                        poses (eval_scripts/swarm_loc_scenarios.py) so drones
#                        sit on the pad instead of dropping onto bumpy rock.
#       --mesh PATH      Mesh file for radar raycasting.
#                        Relative paths are resolved against $SAR_NANO_SWARM_ROOT.
#                        Defaults are auto-detected for built-in worlds; for custom
#                        worlds you must provide this or pass --no-radar.
#       --no-radar       Skip radar plugin injection entirely.
#       --no-radar-noise Publish the ideal radarays cloud directly on
#                        /cf_<i>/radar/points (skip perception/radar_sim/radar_noise_node.py).
#       --radar-noise-config PATH
#                         Path to radar_noise.yaml [default: configs/sensors/radar_noise.yaml]
#       --no-payload      Skip mass/inertia payload rewrite (apply_payload.py).
#       --payload-config PATH
#                         Path to payload.yaml [default: configs/airframe/payload.yaml]
#       --no-tof         Skip IR ToF rangefinder sensor injection (apply_tof_sensor.py).
#       --tof-config PATH
#                         Path to tof.yaml [default: configs/sensors/tof.yaml]
#       --no-imu-noise   Skip Gazebo IMU noise-model injection (apply_imu_noise.py).
#       --imu-noise-config PATH
#                         Path to imu_noise.yaml [default: configs/sensors/imu_noise.yaml]
#       --no-flow        Skip PMW3901 optical-flow node (perception/flow_sim/flow_node.py).
#       --flow-config PATH
#                         Path to optical_flow.yaml [default: configs/sensors/optical_flow.yaml]
#       --no-uwb         Skip inter-drone UWB PDoA node (perception/uwb_sim/uwb_node.py).
#       --uwb-config PATH
#                         Path to uwb_pdoa.yaml [default: configs/sensors/uwb_pdoa.yaml]
#       --no-swarm-loc   Skip per-drone swarm-loc estimator + RIO stub (P2-5).
#       --swarm-loc-config PATH
#                         Path to swarm_loc.yaml [default: configs/estimation/swarm_loc.yaml]
#       --swarm-loc-log-dir PATH
#                         Per-drone measurement .npz for central_reference.py (P2-7).
#       --no-rviz        Skip RViz launch.
#       --headless       Skip Gazebo GUI (server + SITL only, useful for CI).
#       --check          Gate-check mode: start headless, wait 15 s, verify
#                        /cf_0/radar/points publishes ≥ 8 Hz, then exit 0/1.
#                        Implies --no-rviz --headless.
#   -h, --help           Show this help and exit.
#
# Environment overrides:
#   CRAZYSIM_FW        Full path to the cf2 binary.
#                      Default: <repo>/firmware_mods/CrazySim/crazyflie-firmware/sitl_make/build/cf2
#   RADAR_PLUGIN_DIR   Dir containing libradar_sensor_system.so.
#                      Default: <repo>/install/radarays_gz2/lib
#   CRAZYSIM_LOCKSTEP  1 (default) = Gazebo drives each cf2 1 kHz tick from
#                      sim time (plugin UDP port = firmware port + 1000).
#                      Required when 3-drone physics runs below RTF 1; without
#                      it the controller ticks on the wall clock and climbs.
#                      Set 0 to restore wall-clock FreeRTOS ticks.
#
# cfclient connection URI printed at startup:
#   udp://127.0.0.1:19850+N   (drone ID N)
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
  -x X                 Spawn X  [default: tunnel_site or 0]
  -y Y                 Spawn Y  [default: tunnel_site or 0]
  -z Z                 Spawn Z  [default: tunnel_site or 0.5]
  -n, --num-drones N    Number of drones  [default: 1]
      --spacing M      Spawn spacing on X axis (m)  [default: 1.5]
      --spawn-positions "x,y,z;x,y,z;..."  Explicit per-drone spawn poses
                       (world frame, one entry per drone; count must equal -n)
      --mesh PATH      Mesh for radar raycasting (rel to SAR_NANO_SWARM_ROOT)
      --no-radar       Skip radar plugin
      --no-radar-noise Skip radar noise layer (ideal cloud on /cf_<i>/radar/points)
      --radar-noise-config PATH  radar_noise.yaml [default: configs/sensors/radar_noise.yaml]
      --no-payload     Skip mass/inertia payload rewrite
      --payload-config PATH  payload.yaml to use [default: configs/airframe/payload.yaml]
      --no-tof         Skip IR ToF rangefinder sensor injection
      --tof-config PATH  tof.yaml to use [default: configs/sensors/tof.yaml]
      --no-imu-noise   Skip Gazebo IMU noise-model injection
      --imu-noise-config PATH  imu_noise.yaml [default: configs/sensors/imu_noise.yaml]
      --no-flow        Skip PMW3901 optical-flow node
      --flow-config PATH  optical_flow.yaml [default: configs/sensors/optical_flow.yaml]
      --no-uwb         Skip UWB PDoA node
      --uwb-config PATH  uwb_pdoa.yaml [default: configs/sensors/uwb_pdoa.yaml]
      --no-swarm-loc   Skip swarm-loc estimator + RIO stub
      --swarm-loc-config PATH  swarm_loc.yaml [default: configs/estimation/swarm_loc.yaml]
      --swarm-loc-log-dir PATH  write cf_<i>.npz measurement logs (P2-7)
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
SPAWN_Z=0.5
SPAWN_X_CLI=0
SPAWN_Y_CLI=0
SPAWN_Z_CLI=0
NUM_DRONES=1
SPACING=1.5
SPAWN_POSITIONS=""
MESH_ARG=""
USE_RADAR=true
USE_RADAR_NOISE=true
RADAR_NOISE_CONFIG=""
USE_PAYLOAD=true
PAYLOAD_CONFIG=""
USE_TOF=true
TOF_CONFIG=""
USE_IMU_NOISE=true
IMU_NOISE_CONFIG=""
USE_FLOW=true
FLOW_CONFIG=""
USE_UWB=true
UWB_CONFIG=""
USE_SWARM_LOC=true
SWARM_LOC_CONFIG=""
SWARM_LOC_LOG_DIR=""
USE_RVIZ=true
USE_GUI=true
GATE_CHECK=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    -w|--world)   WORLD="$2";       shift 2 ;;
    -m|--model)   MODEL="$2";       shift 2 ;;
    -x)           SPAWN_X="$2"; SPAWN_X_CLI=1; shift 2 ;;
    -y)           SPAWN_Y="$2"; SPAWN_Y_CLI=1; shift 2 ;;
    -z)           SPAWN_Z="$2"; SPAWN_Z_CLI=1; shift 2 ;;
    -n|--num-drones) NUM_DRONES="$2"; shift 2 ;;
    --spacing)    SPACING="$2";    shift 2 ;;
    --spawn-positions) SPAWN_POSITIONS="$2"; shift 2 ;;
    --mesh)       MESH_ARG="$2";    shift 2 ;;
    --no-radar)   USE_RADAR=false;  shift   ;;
    --no-radar-noise) USE_RADAR_NOISE=false; shift ;;
    --radar-noise-config) RADAR_NOISE_CONFIG="$2"; shift 2 ;;
    --no-payload) USE_PAYLOAD=false; shift  ;;
    --payload-config) PAYLOAD_CONFIG="$2"; shift 2 ;;
    --no-tof)     USE_TOF=false;   shift   ;;
    --tof-config) TOF_CONFIG="$2"; shift 2 ;;
    --no-imu-noise) USE_IMU_NOISE=false; shift ;;
    --imu-noise-config) IMU_NOISE_CONFIG="$2"; shift 2 ;;
    --no-flow)    USE_FLOW=false;  shift   ;;
    --flow-config) FLOW_CONFIG="$2"; shift 2 ;;
    --no-uwb)     USE_UWB=false;   shift   ;;
    --uwb-config) UWB_CONFIG="$2"; shift 2 ;;
    --no-swarm-loc) USE_SWARM_LOC=false; shift ;;
    --swarm-loc-config) SWARM_LOC_CONFIG="$2"; shift 2 ;;
    --swarm-loc-log-dir) SWARM_LOC_LOG_DIR="$2"; shift 2 ;;
    --no-rviz)    USE_RVIZ=false;   shift   ;;
    --headless)   USE_GUI=false;    shift   ;;
    --check)      GATE_CHECK=true; USE_RVIZ=false; USE_GUI=false; shift ;;
    -h|--help)    usage; exit 0 ;;
    *) die "Unknown option: $1" ;;
  esac
done

# ── validate --spawn-positions ────────────────────────────────────────────────
SPAWN_POS_ARR=()
if [[ -n "$SPAWN_POSITIONS" ]]; then
  IFS=';' read -ra SPAWN_POS_ARR <<< "$SPAWN_POSITIONS"
  [[ "${#SPAWN_POS_ARR[@]}" -eq "$NUM_DRONES" ]] || die \
    "--spawn-positions has ${#SPAWN_POS_ARR[@]} entries but -n/--num-drones is $NUM_DRONES"
fi

# ── locate repo root ──────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Allow override via env (useful when running from a different working dir)
export SAR_NANO_SWARM_ROOT="${SAR_NANO_SWARM_ROOT:-$REPO_ROOT}"

# Tunnel site: spawn + (later) derived UWB entrance/mesh LOS. CLI -x/-y/-z win.
_SITE_YAML="$SAR_NANO_SWARM_ROOT/configs/sim/tunnel_site.yaml"
_SITE_WORLD=""
if [[ -f "$_SITE_YAML" ]]; then
  _SITE_WORLD="$(python3 -c "import yaml; print(yaml.safe_load(open('$_SITE_YAML')).get('world',''))" 2>/dev/null || true)"
fi
if [[ -f "$_SITE_YAML" && "$WORLD" == "phase0_tunnel_gate" && "$_SITE_WORLD" == "phase0_tunnel_gate" ]]; then
  _site_xyz="$(python3 -c "
import sys
sys.path.insert(0, '$SAR_NANO_SWARM_ROOT/eval_scripts')
from tunnel_site import load_site, hover_height
s = load_site('$_SITE_YAML')
x,y,z = s['spawn']['xyz_m']
print(float(x), float(y), float(hover_height(s)))
")"
  read -r _SX _SY _SZ <<< "$_site_xyz"
  [[ "$SPAWN_X_CLI" != 1 ]] && SPAWN_X="$_SX"
  [[ "$SPAWN_Y_CLI" != 1 ]] && SPAWN_Y="$_SY"
  [[ "$SPAWN_Z_CLI" != 1 ]] && SPAWN_Z="$_SZ"
  info "tunnel_site spawn (${SPAWN_X}, ${SPAWN_Y}, ${SPAWN_Z}) from $_SITE_YAML"
fi

# --spawn-positions: identity tilt + the site's yaw (a real Crazyflie
# calibrates its gyro sitting still and level; the launch pad places drones
# resting at their scenario layout instead of dropping them from a hover).
_SITE_YAW_DEG=0
_SPAWN_ORIENT_REQ=""
if [[ -n "$SPAWN_POSITIONS" ]]; then
  if [[ -f "$_SITE_YAML" ]]; then
    _SITE_YAW_DEG="$(python3 -c "
import sys
sys.path.insert(0, '$SAR_NANO_SWARM_ROOT/eval_scripts')
from tunnel_site import load_site, spawn_yaw_deg
print(spawn_yaw_deg(load_site('$_SITE_YAML')))
")"
  fi
  read -r _SPAWN_QZ _SPAWN_QW <<< "$(python3 -c "
import math
yaw = math.radians(${_SITE_YAW_DEG})
print(math.sin(yaw / 2.0), math.cos(yaw / 2.0))
")"
  _SPAWN_ORIENT_REQ=", orientation: {z: ${_SPAWN_QZ}, w: ${_SPAWN_QW}}"
  info "spawn-positions: ${#SPAWN_POS_ARR[@]} explicit poses, yaw ${_SITE_YAW_DEG} deg from $_SITE_YAML"
fi

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

# ── radar plugin path (repo install wins so Doppler fields load) ─────────────
_REPO_RADAR="$SAR_NANO_SWARM_ROOT/install/radarays_gz2/lib"
if [[ -f "$_REPO_RADAR/libradar_sensor_system.so" ]]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$_REPO_RADAR:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
  export LD_LIBRARY_PATH="$_REPO_RADAR:${LD_LIBRARY_PATH:-}"
fi
if [[ -n "${RADAR_PLUGIN_DIR:-}" && -d "$RADAR_PLUGIN_DIR" ]]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:$RADAR_PLUGIN_DIR"
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$RADAR_PLUGIN_DIR"
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
# radarays_gz2 imports <mesh_path> at IDENTITY and raycasts in WORLD frame, and
# never reads the world SDF. So the map must be the launched world's COLLISION
# geometry (ground <plane> included) already transformed into world
# coordinates. eval_scripts/build_radar_map.py builds exactly that from
# $WORLD_SDF, cropped to the flight region + radar range, cached in
# out/radar_maps/<world>_<hash>.obj (a relaunch reuses it without re-parsing
# meshes). No per-world mesh table: every world with physical geometry gets
# radar (phase1_pid_tune / crazysim_default via their ground planes).
#
# --mesh PATH stays an explicit override (used verbatim, as before).
# If the builder fails (or the cropped map is empty) radar is DISABLED with a
# loud warning — never a silent fallback to a mesh that does not match the world.
#
# Flight region: spawn line x in [SPAWN_X, SPAWN_X+(N-1)*SPACING], y = SPAWN_Y,
# z in [0, RADAR_FLIGHT_ZMAX_M], widened by RADAR_FLIGHT_MARGIN_M on each
# horizontal side. 5 m covers everything the launcher's users do today:
# eval_scripts/swarm_loc_scenarios.py motions translate at most ~2-3 m total
# (0.4-0.5 m legs, a few of them) and the gate reset_poses drones into
# line/triangle layouts within a few metres of the spawn line. The builder adds
# the radar range (+2 m margin) on top of this on every axis.
RADAR_FLIGHT_MARGIN_M=5.0
RADAR_FLIGHT_ZMAX_M=3.0
# With --no-radar-noise the ideal plugin cloud is not gated to radar_noise.yaml
# range_max_m; the plugin itself casts to radarModel_.range.max = 30 m
# (perception/radarays_gz2/src/RadarSensorSystem.cpp), so crop to that instead.
RADAR_PLUGIN_RANGE_MAX_M=30.0

if [[ "$USE_RADAR" == true ]]; then
  MESH_PATH="$MESH_ARG"

  if [[ -z "$MESH_PATH" ]]; then
    _map_region=$(SPAWN_POSITIONS="$SPAWN_POSITIONS" python3 -c "
import os
m = float('${RADAR_FLIGHT_MARGIN_M}')
sp = os.environ.get('SPAWN_POSITIONS', '')
if sp:
    xs = []; ys = []
    for entry in sp.split(';'):
        x, y, z = [float(v) for v in entry.split(',')]
        xs.append(x); ys.append(y)
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
else:
    x0 = float('${SPAWN_X}'); x1 = x0 + (int('${NUM_DRONES}') - 1) * float('${SPACING}')
    y0 = y1 = float('${SPAWN_Y}')
print(x0 - m, x1 + m, y0 - m, y1 + m, 0.0, float('${RADAR_FLIGHT_ZMAX_M}'))")
    _map_args=("$WORLD_SDF" --region $_map_region
               --cache-dir "$SAR_NANO_SWARM_ROOT/out/radar_maps")
    if [[ "$USE_RADAR_NOISE" != true ]]; then
      _map_args+=(--range-m "$RADAR_PLUGIN_RANGE_MAX_M")
    elif [[ -n "$RADAR_NOISE_CONFIG" ]]; then
      _rn_cfg="$RADAR_NOISE_CONFIG"
      [[ "$_rn_cfg" != /* ]] && _rn_cfg="$SAR_NANO_SWARM_ROOT/$_rn_cfg"
      _map_args+=(--radar-config "$_rn_cfg")
    fi
    info "Building radar map from world collision geometry (region: $_map_region) …"
    if _map_out=$(python3 "$SAR_NANO_SWARM_ROOT/eval_scripts/build_radar_map.py" "${_map_args[@]}"); then
      echo "$_map_out" | sed '$d' | sed 's/^/    /'
      MESH_PATH="$(echo "$_map_out" | tail -n 1)"
    else
      warn "════════════════════════════════════════════════════════════════════"
      warn "build_radar_map.py FAILED for $WORLD_SDF — RADAR DISABLED."
      warn "The radar map must match the world; not falling back to another mesh."
      warn "Fix the error above, pass --mesh <path>, or --no-radar."
      warn "════════════════════════════════════════════════════════════════════"
      USE_RADAR=false
      MESH_PATH=""
    fi
  fi

  if [[ "$USE_RADAR" == true ]]; then
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

# ── radar noise layer (sim-side) — resolve BEFORE injection ──────────────────
# Enabled: plugin publishes the ideal cloud on /cf_<i>/radar/points_ideal and
# perception/radar_sim/radar_noise_node.py republishes /cf_<i>/radar/points.
# If the node cannot start, fall back to the plugin publishing /radar/points
# directly so RIO is never left without a cloud.
RADAR_TOPIC_SUFFIX="radar/points"
if [[ "$USE_RADAR" == true && "$USE_RADAR_NOISE" == true ]]; then
  _radar_noise_cfg="${RADAR_NOISE_CONFIG:-$SAR_NANO_SWARM_ROOT/configs/sensors/radar_noise.yaml}"
  [[ "$_radar_noise_cfg" != /* ]] && _radar_noise_cfg="$SAR_NANO_SWARM_ROOT/$_radar_noise_cfg"
  if [[ ! -f "$_radar_noise_cfg" ]]; then
    warn "Radar noise config not found: $_radar_noise_cfg — radar cloud will be IDEAL."
    USE_RADAR_NOISE=false
  elif ! command -v ros2 &>/dev/null; then
    warn "ros2 not on PATH — skipping radar noise node; radar cloud will be IDEAL."
    USE_RADAR_NOISE=false
  else
    RADAR_TOPIC_SUFFIX="radar/points_ideal"
  fi
else
  USE_RADAR_NOISE=false
fi

# Real RIO needs radarays Doppler clouds. Stub is the only no-radar odom path.
if [[ "$USE_SWARM_LOC" == true ]]; then
  _rio_cfg_early="${SWARM_LOC_CONFIG:-$SAR_NANO_SWARM_ROOT/configs/estimation/swarm_loc.yaml}"
  [[ "$_rio_cfg_early" != /* ]] && _rio_cfg_early="$SAR_NANO_SWARM_ROOT/$_rio_cfg_early"
  _rio_src_early="$(python3 -c "import yaml; print(yaml.safe_load(open('$_rio_cfg_early'))['rio']['source'])" 2>/dev/null || echo stub)"
  if [[ "$_rio_src_early" == "real" && "$USE_RADAR" != true ]]; then
    die "rio.source=real requires the radar plugin (Doppler on /cf_<i>/radar/points).
  Drop --no-radar, or set rio.source: stub in configs/estimation/swarm_loc.yaml."
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
info "Stopping any running cf2 / UWB / swarm-loc nodes …"
pkill -x cf2 2>/dev/null || true
pkill -f "uwb_node.py" 2>/dev/null || true
pkill -f "uwb_sim" 2>/dev/null || true
pkill -f "radar_noise_node.py" 2>/dev/null || true
pkill -f "swarm_loc_node.py" 2>/dev/null || true
pkill -f "rio_stub.py" 2>/dev/null || true
pkill -f "rio_bridge.py" 2>/dev/null || true
sleep 1

# ── start Gazebo server ───────────────────────────────────────────────────────
# Plugin reads CRAZYSIM_LOCKSTEP at Configure; gz must see it. Each cf2 gets
# CF2_LOCKSTEP_PORT=$((cffirm + 1000)) so ticks follow sim dt, not wall time.
: "${CRAZYSIM_LOCKSTEP:=1}"
export CRAZYSIM_LOCKSTEP
info "Starting Gazebo server (world: $WORLD_NAME, CRAZYSIM_LOCKSTEP=${CRAZYSIM_LOCKSTEP}) …"
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

# ── per-drone: generate SDF, inject sensors, spawn, wait, start SITL ─────────
CFLIB_PORTS=()
for CF_ID in $(seq 0 $((NUM_DRONES - 1))); do
  CFLIB_PORT=$((19850 + CF_ID))
  CFFIRM_PORT=$((19950 + CF_ID))
  CFLIB_PORTS+=("$CFLIB_PORT")
  SDF_TMP="/tmp/${MODEL}_${CF_ID}.sdf"
  if [[ -n "$SPAWN_POSITIONS" ]]; then
    IFS=',' read -r SPAWN_XI SPAWN_YI SPAWN_ZI <<< "${SPAWN_POS_ARR[$CF_ID]}"
  else
    SPAWN_XI=$(python3 -c "print(${SPAWN_X} + ${CF_ID} * ${SPACING})")
    SPAWN_YI="$SPAWN_Y"
    SPAWN_ZI="$SPAWN_Z"
  fi

  rm -f "$SDF_TMP"
  mkdir -p "$BUILD_DIR/$CF_ID"
  info "Generating Crazyflie SDF for drone ${CF_ID} …"
  python3 "$JINJA_GEN" \
    "$MODELS_DIR/${MODEL}/model.sdf.jinja" \
    "$MODELS_DIR" \
    --cffirm_udp_port "$CFFIRM_PORT" \
    --cflib_udp_port  "$CFLIB_PORT" \
    --cf_id           "$CF_ID" \
    --cf_name         "cf" \
    --output-file     "$SDF_TMP"

  if [[ "$USE_RADAR" == true ]]; then
    info "Injecting radarays_gz2 plugin on drone ${CF_ID} (mesh: $MESH_PATH, topic: /cf_${CF_ID}/${RADAR_TOPIC_SUFFIX}) …"
    python3 - "$SDF_TMP" "$MESH_PATH" "$CF_ID" "/cf_${CF_ID}/${RADAR_TOPIC_SUFFIX}" <<'PYEOF'
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
# Per-drone radar topic so rio_bridge <i> sees only its own Doppler cloud.
# /cf_<i>/radar/points_ideal when the radar noise node is enabled, else /cf_<i>/radar/points.
topic_elem = ET.SubElement(plugin, 'topic')
topic_elem.text = sys.argv[4]

tree.write(sys.argv[1], encoding='unicode')
print(f"[radar-inject] Plugin injected into {sys.argv[1]}")
PYEOF
  fi

  if [[ "$USE_PAYLOAD" == true ]]; then
    _payload_cfg="${PAYLOAD_CONFIG:-$SAR_NANO_SWARM_ROOT/configs/airframe/payload.yaml}"
    [[ "$_payload_cfg" != /* ]] && _payload_cfg="$SAR_NANO_SWARM_ROOT/$_payload_cfg"
    if [[ ! -f "$_payload_cfg" ]]; then
      warn "Payload config not found: $_payload_cfg — skipping mass/inertia rewrite."
    else
      info "Applying payload mass/inertia model on drone ${CF_ID} ($_payload_cfg) …"
      python3 "$SAR_NANO_SWARM_ROOT/eval_scripts/apply_payload.py" "$SDF_TMP" --payload "$_payload_cfg"

      if [[ "$CF_ID" -eq 0 ]]; then
        info "Checking thrust margin …"
        python3 "$SAR_NANO_SWARM_ROOT/eval_scripts/thrust_margin_check.py" "$SDF_TMP" \
          --config "$SAR_NANO_SWARM_ROOT/configs/airframe/thrust_margin.yaml" \
          || warn "Thrust-margin check failed — drone may be under-thrusted for this payload."
      fi
    fi
  fi

  if [[ "$USE_TOF" == true ]]; then
    _tof_cfg="${TOF_CONFIG:-$SAR_NANO_SWARM_ROOT/configs/sensors/tof.yaml}"
    [[ "$_tof_cfg" != /* ]] && _tof_cfg="$SAR_NANO_SWARM_ROOT/$_tof_cfg"
    if [[ ! -f "$_tof_cfg" ]]; then
      warn "ToF config not found: $_tof_cfg — skipping sensor injection."
    else
      info "Injecting IR ToF sensor(s) on drone ${CF_ID} ($_tof_cfg) …"
      python3 "$SAR_NANO_SWARM_ROOT/eval_scripts/apply_tof_sensor.py" "$SDF_TMP" \
        --config "$_tof_cfg" --cf-id "$CF_ID"
    fi
  fi

  if [[ "$USE_IMU_NOISE" == true ]]; then
    _imu_cfg="${IMU_NOISE_CONFIG:-$SAR_NANO_SWARM_ROOT/configs/sensors/imu_noise.yaml}"
    [[ "$_imu_cfg" != /* ]] && _imu_cfg="$SAR_NANO_SWARM_ROOT/$_imu_cfg"
    if [[ ! -f "$_imu_cfg" ]]; then
      warn "IMU noise config not found: $_imu_cfg — skipping IMU noise injection."
    else
      info "Injecting IMU noise model on drone ${CF_ID} ($_imu_cfg) …"
      python3 "$SAR_NANO_SWARM_ROOT/eval_scripts/apply_imu_noise.py" "$SDF_TMP" \
        --config "$_imu_cfg" --cf-id "$CF_ID"
    fi
  fi

  info "Spawning ${MODEL}_${CF_ID} at (${SPAWN_XI}, ${SPAWN_YI}, ${SPAWN_ZI}) …"
  gz service \
    -s "/world/${WORLD_NAME}/create" \
    --reqtype  gz.msgs.EntityFactory \
    --reptype  gz.msgs.Boolean \
    --timeout  5000 \
    --req "sdf_filename: \"${SDF_TMP}\",
           pose: {position: {x: ${SPAWN_XI}, y: ${SPAWN_YI}, z: ${SPAWN_ZI}}${_SPAWN_ORIENT_REQ}},
           name: \"${MODEL}_${CF_ID}\",
           allow_renaming: 1"

  # Start SITL immediately after spawn so the lockstep UDP bind is up before
  # many physics steps elapse. Plugin sends to cffirm_port+1000; firmware
  # binds that port when CF2_LOCKSTEP_PORT is set.
  export CF2_SIM_MODEL="gz_${MODEL}"
  pushd "$BUILD_DIR/$CF_ID" > /dev/null
  if [[ "$CRAZYSIM_LOCKSTEP" == "1" ]]; then
    _ls_port=$((CFFIRM_PORT + 1000))
    info "Starting SITL firmware (instance ${CF_ID}, lockstep udp ${_ls_port}) …"
    env CF2_LOCKSTEP_PORT="${_ls_port}" "$CF2_BIN" "$CFFIRM_PORT" > out.log 2> error.log &
  else
    info "Starting SITL firmware (instance ${CF_ID}) …"
    env -u CF2_LOCKSTEP_PORT "$CF2_BIN" "$CFFIRM_PORT" > out.log 2> error.log &
  fi
  _PIDS+=($!)
  popd > /dev/null

  info "Waiting for drone ${CF_ID} sensors (/cf_${CF_ID}/odom) to come online …"
  _drone_ready=false
  for _i in $(seq 1 20); do
    if timeout 3 gz topic -e -t "/cf_${CF_ID}/odom" -n 1 >/dev/null 2>&1; then
      _drone_ready=true
      break
    fi
    sleep 1
  done
  if [[ "$_drone_ready" == true ]]; then
    info "Drone ${CF_ID} sensors publishing. Giving them 2s to stabilise …"
    sleep 2
  else
    warn "Drone ${CF_ID} gz odom not detected after ~20s — ROS wait is later."
  fi
done

# ── wait for cflib UDP ports + firmware settle (uwb_gate / cflib connect) ─────
info "Waiting for cflib UDP ports (${CFLIB_PORTS[*]}) …"
for _port in "${CFLIB_PORTS[@]}"; do
  _port_ok=false
  for _i in $(seq 1 45); do
    if ss -ulnp 2>/dev/null | grep -q ":${_port} "; then
      _port_ok=true
      break
    fi
    sleep 1
  done
  [[ "$_port_ok" == true ]] || warn "cflib port ${_port} not bound after 45s — gate scripts may fail."
done
_cf2_n="$(pgrep -x cf2 2>/dev/null | wc -l | tr -d ' ')"
[[ "${_cf2_n:-0}" -ge "$NUM_DRONES" ]] || warn "Only ${_cf2_n:-0}/${NUM_DRONES} cf2 process(es) — check sitl_make/build/*/error.log"
if [[ "$CRAZYSIM_LOCKSTEP" == "1" ]]; then
  for CF_ID in $(seq 0 $((NUM_DRONES - 1))); do
    if grep -q "LOCKSTEP: firmware ticks" "$BUILD_DIR/$CF_ID/out.log" 2>/dev/null; then
      info "cf2 ${CF_ID} lockstep bound"
    else
      warn "cf2 ${CF_ID} did not log lockstep bind — check $BUILD_DIR/$CF_ID/out.log (wall-clock ticks?)"
    fi
  done
fi
info "SITL settle (3 s for CfFirm handshake) …"
sleep 3

# ── firmware liveness check: CRTP link-echo per port ─────────────────────────
for _port in "${CFLIB_PORTS[@]}"; do
  if python3 - "$_port" <<'PYEOF'
import sys, socket

port = int(sys.argv[1])
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)
echo_packet = b"\xf0\x01\x02\x03"
found_reply = False

for attempt in range(20):
  try:
    sock.sendto(echo_packet, ('127.0.0.1', port))
    reply, _ = sock.recvfrom(64)
    if reply and reply[0] == 0xf0 and reply != b"\xff":
      found_reply = True
      break
  except socket.timeout:
    pass
  except Exception:
    pass
sock.close()
sys.exit(0 if found_reply else 1)
PYEOF
  then
    info "Firmware on port ${_port} answers CRTP link echo."
  else
    _cf_id=$(((_port - 19850)))
    warn "FIRMWARE NOT ANSWERING on cflib port ${_port} (wedged or dead cf2) — check $BUILD_DIR/${_cf_id}/out.log; cflib connect will time out."
  fi
done

# ── bridge gz topics to ROS 2 ────────────────────────────────────────────────
if [[ "$USE_TOF" == true || "$USE_FLOW" == true || "$USE_UWB" == true || "$USE_SWARM_LOC" == true ]] && command -v ros2 &>/dev/null && ros2 pkg prefix ros_gz_bridge &>/dev/null; then
  _bridge_args=()
  for i in $(seq 0 $((NUM_DRONES - 1))); do
    if [[ "$USE_TOF" == true ]]; then
      _bridge_args+=("/cf_${i}/tof_down@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan")
    fi
    if [[ "$USE_FLOW" == true || "$USE_UWB" == true || "$USE_SWARM_LOC" == true ]]; then
      _bridge_args+=("/cf_${i}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry")
    fi
    if [[ "$USE_SWARM_LOC" == true ]]; then
      _bridge_args+=("/cf_${i}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU")
    fi
  done
  if [[ "$USE_UWB" == true || "$USE_SWARM_LOC" == true ]]; then
    _bridge_args+=("/world/${WORLD_NAME}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V")
  fi
  info "Bridging gz topics to ROS 2 (${#_bridge_args[@]} mappings) …"
  ros2 run ros_gz_bridge parameter_bridge "${_bridge_args[@]}" &
  _PIDS+=($!)
  if [[ "$USE_UWB" == true || "$USE_SWARM_LOC" == true ]]; then
    python3 -u "$SAR_NANO_SWARM_ROOT/eval_scripts/gz_pose_to_odom.py" \
      --world "$WORLD_NAME" --num-drones "$NUM_DRONES" &
    _PIDS+=($!)
  fi
  if [[ "$USE_UWB" == true || "$USE_SWARM_LOC" == true ]]; then
    info "Waiting for ROS /cf_*/odom (both QoS profiles) …"
    if ! python3 -u "$SAR_NANO_SWARM_ROOT/eval_scripts/wait_ros_odom.py" \
        --num-drones "$NUM_DRONES" --timeout 120; then
      die "ROS /cf_*/odom never delivered. UWB and ATE would be empty."
    fi
  fi
elif [[ "$USE_TOF" == true || "$USE_FLOW" == true || "$USE_UWB" == true || "$USE_SWARM_LOC" == true ]]; then
  warn "ros_gz_bridge not installed — gz-native topics not bridged to ROS."
  warn "Install with: sudo apt install ros-humble-ros-gz-bridge (or source setup_env.sh)."
fi

# ── launch PMW3901 optical-flow node (Phase 1 M3b) — drone 0 only ───────────
if [[ "$USE_FLOW" == true ]]; then
  _flow_cfg="${FLOW_CONFIG:-$SAR_NANO_SWARM_ROOT/configs/sensors/optical_flow.yaml}"
  [[ "$_flow_cfg" != /* ]] && _flow_cfg="$SAR_NANO_SWARM_ROOT/$_flow_cfg"
  if [[ ! -f "$_flow_cfg" ]]; then
    warn "Optical-flow config not found: $_flow_cfg — skipping flow node."
  elif ! command -v ros2 &>/dev/null; then
    warn "ros2 not on PATH — skipping optical-flow node (source setup_env.sh)."
  else
    info "Starting PMW3901 optical-flow node ($_flow_cfg, cf_0) …"
    python3 -u "$SAR_NANO_SWARM_ROOT/perception/flow_sim/flow_node.py" \
      --config "$_flow_cfg" --cf-id 0 &
    _PIDS+=($!)
  fi
fi

# ── launch UWB PDoA node (Phase 1 M4) — one node for the whole swarm ─────────
if [[ "$USE_UWB" == true ]]; then
  _uwb_cfg="${UWB_CONFIG:-$SAR_NANO_SWARM_ROOT/configs/sensors/uwb_pdoa.yaml}"
  [[ "$_uwb_cfg" != /* ]] && _uwb_cfg="$SAR_NANO_SWARM_ROOT/$_uwb_cfg"
  # Tunnel site: pin the entrance peer and point mesh LOS at the world-frame
  # radar map. Skip if the caller passed --uwb-config (they own the file).
  if [[ -z "$UWB_CONFIG" && -f "$_SITE_YAML" && "$WORLD" == "phase0_tunnel_gate" && "$USE_RADAR" == true && -n "${MESH_PATH:-}" ]]; then
    _uwb_derived="$SAR_NANO_SWARM_ROOT/out/runtime_uwb.yaml"
    if python3 "$SAR_NANO_SWARM_ROOT/eval_scripts/tunnel_site.py" \
        --site "$_SITE_YAML" --derive-uwb --base "$_uwb_cfg" \
        --mesh-path "$MESH_PATH" --out "$_uwb_derived"; then
      _uwb_cfg="$_uwb_derived"
      info "derived UWB config (mesh LOS + site entrance) → $_uwb_cfg"
    else
      die "failed to derive UWB config with mesh LOS — refusing silent always-LOS"
    fi
  fi
  if [[ ! -f "$_uwb_cfg" ]]; then
    warn "UWB config not found: $_uwb_cfg — skipping UWB node."
  elif ! command -v ros2 &>/dev/null; then
    warn "ros2 not on PATH — skipping UWB node (source setup_env.sh)."
  else
    info "Starting UWB PDoA node ($_uwb_cfg, ${NUM_DRONES} drones) …"
    python3 -u "$SAR_NANO_SWARM_ROOT/perception/uwb_sim/uwb_node.py" \
      --config "$_uwb_cfg" --num-drones "$NUM_DRONES" &
    _PIDS+=($!)
  fi
fi

# ── launch radar noise node — one node for the whole swarm, before RIO ───────
# (config / ros2 availability already resolved before plugin injection)
if [[ "$USE_RADAR" == true && "$USE_RADAR_NOISE" == true ]]; then
  info "Starting radar noise node ($_radar_noise_cfg, ${NUM_DRONES} drones: /cf_<i>/radar/points_ideal → /cf_<i>/radar/points) …"
  python3 -u "$SAR_NANO_SWARM_ROOT/perception/radar_sim/radar_noise_node.py" \
    --config "$_radar_noise_cfg" --num-drones "$NUM_DRONES" &
  _PIDS+=($!)
fi

# ── launch RIO stub + swarm-loc estimator (Phase 2 P2-5) — one pair per drone ─
if [[ "$USE_SWARM_LOC" == true ]]; then
  _cfg="${SWARM_LOC_CONFIG:-$SAR_NANO_SWARM_ROOT/configs/estimation/swarm_loc.yaml}"
  [[ "$_cfg" != /* ]] && _cfg="$SAR_NANO_SWARM_ROOT/$_cfg"
  if [[ ! -f "$_cfg" ]]; then
    warn "swarm-loc config not found: $_cfg — skipping estimator."
  elif ! command -v ros2 &>/dev/null; then
    warn "ros2 not on PATH — skipping swarm-loc (source setup_env.sh)."
  else
    info "Starting RIO + swarm-loc ($_cfg, ${NUM_DRONES} drones) …"
    _rio_src="$(python3 -c "import yaml; print(yaml.safe_load(open('$_cfg'))['rio']['source'])" 2>/dev/null || echo stub)"
    _log_dir=""
    if [[ -n "$SWARM_LOC_LOG_DIR" ]]; then
      _log_dir="$SWARM_LOC_LOG_DIR"
      [[ "$_log_dir" != /* ]] && _log_dir="$SAR_NANO_SWARM_ROOT/$_log_dir"
      mkdir -p "$_log_dir"
      info "swarm-loc measurement logs → $_log_dir"
    fi
    for i in $(seq 0 $((NUM_DRONES - 1))); do
      if [[ "$_rio_src" == "real" ]]; then
        info "RIO source=real (radar_processing/rio_bridge.py) cf_${i}"
        python3 -u "$SAR_NANO_SWARM_ROOT/perception/radar_processing/rio_bridge.py" \
          --cf-id "$i" --config "$_cfg" &
      else
        info "RIO source=stub (rio_stub.py) cf_${i}"
        python3 -u "$SAR_NANO_SWARM_ROOT/perception/swarm_loc/rio_stub.py" \
          --cf-id "$i" --config "$_cfg" &
      fi
      _PIDS+=($!)
      if [[ -n "$_log_dir" ]]; then
        python3 -u "$SAR_NANO_SWARM_ROOT/perception/swarm_loc/swarm_loc_node.py" \
          --cf-id "$i" --num-drones "$NUM_DRONES" --config "$_cfg" \
          --log-measurements "$_log_dir" &
      else
        python3 -u "$SAR_NANO_SWARM_ROOT/perception/swarm_loc/swarm_loc_node.py" \
          --cf-id "$i" --num-drones "$NUM_DRONES" --config "$_cfg" &
      fi
      _PIDS+=($!)
    done
  fi
fi

# ── launch RViz ───────────────────────────────────────────────────────────────
if [[ "$USE_SWARM_LOC" == true ]]; then
  RVIZ_CFG="$SAR_NANO_SWARM_ROOT/configs/rviz/swarm_loc.rviz"
else
  RVIZ_CFG="$SAR_NANO_SWARM_ROOT/configs/rviz/radar.rviz"
fi

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
echo "  ║  Drones       : ${NUM_DRONES}"
echo "  ║  cfclient URIs:"
for _p in "${CFLIB_PORTS[@]}"; do
  echo "  ║    udp://127.0.0.1:${_p}"
done
echo "  ║  World        : ${WORLD_NAME}"
echo "  ║  Model        : ${MODEL}_0..$((NUM_DRONES - 1))"
echo "  ║  Radar        : ${USE_RADAR}"
echo "  ║  Radar noise  : ${USE_RADAR_NOISE}"
echo "  ║  Payload model: ${USE_PAYLOAD}"
echo "  ║  ToF sensor   : ${USE_TOF}"
echo "  ║  IMU noise    : ${USE_IMU_NOISE}"
echo "  ║  Optical flow : ${USE_FLOW}"
echo "  ║  UWB          : ${USE_UWB}"
echo "  ║  Swarm-loc    : ${USE_SWARM_LOC}"
echo "  ║  Radar topic  : /cf_<id>/radar/points  (~10 Hz, per drone)"
echo "  ║  ToF topic    : /cf_<id>/tof_down  (gz-native, ~30 Hz)"
echo "  ║  Flow topic   : /cf_0/flow  (ROS, ~100 Hz)"
echo "  ║  UWB topic    : /cf_<id>/uwb/edges  (~scheduler tick Hz)"
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
  info "Gate-check mode: waiting 15 s for /cf_0/radar/points to stabilise …"
  sleep 15

  if ! command -v ros2 &>/dev/null; then
    die "--check requires ros2 on PATH (source setup_env.sh first)."
  fi

  info "Sampling /cf_0/radar/points for 5 s …"
  HZ_OUTPUT=$(ros2 topic hz /cf_0/radar/points --window 10 2>&1 &
              HZ_PID=$!
              sleep 5
              kill $HZ_PID 2>/dev/null || true
              wait $HZ_PID 2>/dev/null || true)

  MEASURED_HZ=$(echo "$HZ_OUTPUT" | grep -oP 'average rate: \K[0-9.]+' | tail -1)

  if [[ -z "$MEASURED_HZ" ]]; then
    echo ""
    echo "  [GATE] FAIL — /cf_0/radar/points not detected (check plugin build and mesh path)"
    exit 1
  fi

  # Pass if measured rate >= 8 Hz (allows some jitter below the 10 Hz target).
  if python3 -c "import sys; sys.exit(0 if float('${MEASURED_HZ}') >= 8.0 else 1)"; then
    echo ""
    echo "  [GATE] PASS — /cf_0/radar/points @ ${MEASURED_HZ} Hz  (target ≥ 8 Hz)"
    exit 0
  else
    echo ""
    echo "  [GATE] FAIL — /cf_0/radar/points @ ${MEASURED_HZ} Hz  (target ≥ 8 Hz)"
    exit 1
  fi
fi

# ── interactive: wait for Ctrl-C ─────────────────────────────────────────────
info "Simulation running. Press Ctrl-C to stop all processes."
wait
