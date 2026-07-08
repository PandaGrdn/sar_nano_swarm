# sar_nano_swarm

ROS 2 Humble + Gazebo Harmonic (gz-sim 8) simulation stack for a nano-drone swarm with custom FMCW radar perception, CrazySim SITL firmware, and DARPA SubT worlds.

## Repository layout

| Path | Purpose |
|------|---------|
| `firmware_mods/CrazySim` | CrazySim SITL firmware + Crazyswarm2 workspace (git submodule) |
| `perception/rmagine` | rmagine raycasting library, Embree backend (git submodule) |
| `perception/radarays_gz2` | Custom gz-sim System plugin publishing `sensor_msgs/PointCloud2` on `/radar/points` |
| `sim_worlds/darpa_subt_worlds` | DARPA SubT Tunnel/Urban/Cave Gazebo worlds (git submodule) |
| `sim_worlds/test_radar.world` | Minimal test world for the radar plugin |
| `coordination/` | Swarm coordination nodes (placeholder) |
| `eval_scripts/` | Evaluation and smoke-test scripts |
| `configs/` | Launch/config YAML (placeholder) |
| `setup_env.sh` | Project environment setup (source, do not execute) |

### About `radarays_gz2` vs RadaRays

`perception/radarays_gz2` is a **custom ROS 2 / gz-sim Harmonic replacement** for the original [RadaRays](https://github.com/robotics-upo/radarays_gazebo_plugins) package, which targets ROS 1 and Gazebo Classic. This repo does **not** vendor RadaRays itself.

## Prerequisites

- Ubuntu 22.04
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) (`gz-sim8`)
- Build tools: `cmake`, `build-essential`, `git`, `python3-colcon-common-extensions`
- Embree + Intel OpenMP (`libiomp5.so`) for rmagine
- Separate `ros_gz` workspace (default: `~/ros2_ws`) built from [ros_gz](https://github.com/gazebosim/ros_gz) for Harmonic

### System packages (example)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  python3-colcon-common-extensions \
  libembree4-dev \
  gz-harmonic
```

If `libiomp5.so` is missing, install Intel OpenMP or place the library where `setup_env.sh` can find it (e.g. `~/.local/lib`). You can override the search path with:

```bash
export IOMP_LIB_DIR=/path/to/dir/containing/libiomp5.so
```

## Clone and initialize submodules

```bash
git clone git@github.com:PandaGrdn/sar_nano_swarm.git
cd sar_nano_swarm
git submodule update --init --recursive
```

Pinned submodule commits (as of consolidation):

| Submodule | Remote | Commit |
|-----------|--------|--------|
| `firmware_mods/CrazySim` | `https://github.com/gtfactslab/CrazySim.git` | `3ec8b55` |
| `firmware_mods/CrazySim/crazyflie-firmware` | `https://github.com/llanesc/crazyflie-firmware.git` (`crazysim` branch) | `aa6571d` |
| `firmware_mods/CrazySim/crazyswarm2_ws/src/crazyswarm2` | `https://github.com/llanesc/crazyswarm2.git` (`crazysim` branch) | `2334b7a` |
| `perception/rmagine` | `https://github.com/uos/rmagine.git` | `8ed69e4` |
| `sim_worlds/darpa_subt_worlds` | `https://github.com/LTU-RAI/darpa_subt_worlds.git` | `a15a110` |

If `sim_worlds/darpa_subt_worlds` is missing after submodule init (large mesh assets), clone it manually:

```bash
git clone https://github.com/LTU-RAI/darpa_subt_worlds.git sim_worlds/darpa_subt_worlds
git -C sim_worlds/darpa_subt_worlds checkout a15a1107e638eae090335d2d6f36c623439aaa7f
```

## Python dependencies

```bash
python3 -m pip install -r requirements.txt
```

## Build order

Run these from the repository root after `source setup_env.sh` (once `setup_env.sh` exists post-build paths are optional on first pass).

### 1. ros_gz (one-time, outside this repo)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/gazebosim/ros_gz -b humble
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### 2. rmagine (Embree backend)

```bash
cd perception/rmagine
mkdir -p build && cd build
cmake .. -DRMGINE_BUILD_EMBREE=ON -DRMGINE_BUILD_EXAMPLES=OFF -DRMGINE_BUILD_TESTS=OFF
cmake --build . -j"$(nproc)"
cd ../../..
```

### 3. radarays_gz2 (this repo)

```bash
source setup_env.sh
colcon build --paths perception/radarays_gz2
source install/setup.bash
```

### 4. CrazySim firmware (SITL)

```bash
cd firmware_mods/CrazySim/crazyflie-firmware
mkdir -p sitl_make/build && cd sitl_make/build
cmake ..
make all
cd ../../../../..
```

### 5. Crazyswarm2 workspace

```bash
cd firmware_mods/CrazySim/crazyswarm2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
cd ../../..
```

## Environment setup

After building, source the project environment in every new terminal:

```bash
cd /path/to/sar_nano_swarm
source setup_env.sh
```

`setup_env.sh` exports:

- `SAR_NANO_SWARM_ROOT` — repository root (used to resolve relative `mesh_path` values)
- `GZ_VERSION=harmonic`
- `GZ_SIM_RESOURCE_PATH` — includes `sim_worlds/darpa_subt_worlds`
- `GZ_SIM_SYSTEM_PLUGIN_PATH` — includes `install/radarays_gz2/lib`
- `CMAKE_PREFIX_PATH` / `LD_LIBRARY_PATH` for rmagine and `libiomp5.so`
- ROS overlays for `ros_gz`, `radarays_gz2`, and Crazyswarm2 (when built)

Override the ros_gz workspace location if needed:

```bash
export ROS_GZ_WS=/path/to/your/ros2_ws
source setup_env.sh
```

## Run the radar smoke test

Terminal 1 — Gazebo:

```bash
source setup_env.sh
gz sim -r sim_worlds/test_radar.world
```

Terminal 2 — verify `/radar/points`:

```bash
source setup_env.sh
ros2 topic list | grep radar
ros2 topic echo /radar/points --once
# or:
./eval_scripts/check_radar_topic.sh /radar/points 15
```

You should see a `sensor_msgs/PointCloud2` stream from the `radarays_gz2::RadarSensorSystem` plugin.

## Path portability

World files and plugin SDF use **paths relative to `SAR_NANO_SWARM_ROOT`**, not user-specific absolute paths. Example from `sim_worlds/test_radar.world`:

```xml
<mesh_path>sim_worlds/darpa_subt_worlds/worlds/models/jersey_barrier/meshes/jersey_barrier.dae</mesh_path>
```

The plugin resolves non-absolute `mesh_path` values against `SAR_NANO_SWARM_ROOT` at runtime.

## MLflow

Experiment tracking artifacts (`mlruns/`, `mlflow.db`) are gitignored. Start a local UI after installing `requirements.txt`:

```bash
mlflow ui
```

## Further reading

- CrazySim setup and usage: `firmware_mods/CrazySim/README.md`
- rmagine: `perception/rmagine/README.md`
