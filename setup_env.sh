#!/usr/bin/env bash
# Source from the repository root:
#   source setup_env.sh
#
# Sets environment variables for ROS 2 Humble, Gazebo Harmonic, rmagine,
# radarays_gz2, and DARPA SubT world assets.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Source this script instead of executing it:" >&2
  echo "  source setup_env.sh" >&2
  exit 1
fi

export SAR_NANO_SWARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS 2 Humble
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "warning: /opt/ros/humble/setup.bash not found" >&2
fi

# Gazebo Harmonic
export GZ_VERSION=harmonic

# ros_gz bridge workspace (built separately; see README)
if [[ -f "${ROS_GZ_WS:-$HOME/ros2_ws}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROS_GZ_WS:-$HOME/ros2_ws}/install/setup.bash"
else
  echo "warning: ros_gz workspace not found at \${ROS_GZ_WS:-$HOME/ros2_ws}" >&2
fi

# rmagine (Embree backend)
export CMAKE_PREFIX_PATH="${SAR_NANO_SWARM_ROOT}/perception/rmagine/build:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${SAR_NANO_SWARM_ROOT}/perception/rmagine/build/src:${LD_LIBRARY_PATH:-}"

# Project colcon overlay (radarays_gz2)
if [[ -f "${SAR_NANO_SWARM_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${SAR_NANO_SWARM_ROOT}/install/setup.bash"
fi

# CrazySim / Crazyswarm2 overlay
if [[ -f "${SAR_NANO_SWARM_ROOT}/firmware_mods/CrazySim/crazyswarm2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${SAR_NANO_SWARM_ROOT}/firmware_mods/CrazySim/crazyswarm2_ws/install/setup.bash"
fi

# Gazebo model/world search path for DARPA SubT assets
export GZ_SIM_RESOURCE_PATH="${SAR_NANO_SWARM_ROOT}/sim_worlds/darpa_subt_worlds:${GZ_SIM_RESOURCE_PATH:-}"

# gz-sim System plugin path for radarays_gz2
export GZ_SIM_SYSTEM_PLUGIN_PATH="${SAR_NANO_SWARM_ROOT}/install/radarays_gz2/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

# Intel OpenMP runtime required by Embree/rmagine
for _iomp_dir in \
  "${IOMP_LIB_DIR:-}" \
  "${HOME}/.local/lib" \
  /opt/intel/oneapi/compiler/latest/lib \
  /usr/lib/x86_64-linux-gnu; do
  if [[ -n "${_iomp_dir}" && -f "${_iomp_dir}/libiomp5.so" ]]; then
    export LD_LIBRARY_PATH="${_iomp_dir}:${LD_LIBRARY_PATH:-}"
    break
  fi
done
unset _iomp_dir
