# sar_nano_swarm

A SAR (search-and-rescue) nano-drone swarm simulation project: GPS-denied navigation, radar-based perception, and swarm coordination for locating victims in collapsed structures. Built on ROS 2 Humble, Gazebo Harmonic, and CrazySim SITL firmware.

See `Simulation_Training_Optimization_Roadmap_v2.md` for the phased development plan and `CLAUDE.md` for architecture/design notes.

## Repository layout

| Path | Purpose |
|---|---|
| `firmware_mods/CrazySim` | CrazySim SITL firmware + Crazyswarm2 workspace (git submodule) |
| `perception/rmagine` | Raycasting library used for radar simulation (git submodule) |
| `perception/radarays_gz2` | Custom radar sensor plugin for Gazebo, publishing point clouds over ROS 2 |
| `sim_worlds/darpa_subt_worlds` | DARPA SubT simulation worlds (git submodule) |
| `sim_worlds/patched_worlds/` | Locally fixed copies of specific worlds, used in place of the submodule versions |
| `coordination/` | Swarm coordination code (in progress) |
| `eval_scripts/` | Launch scripts and evaluation/test tools |
| `configs/` | Configuration and tuning files |
| `setup_env.sh` | Environment setup — source this in every terminal before building or running anything |

## Dependencies

- Ubuntu 22.04
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- Build tools: `cmake`, `build-essential`, `git`, `python3-colcon-common-extensions`
- Embree and Intel OpenMP (for the radar raycasting library)
- Python packages in `requirements.txt`

## Setup

Clone the repo and pull in submodules:

```bash
git clone git@github.com:PandaGrdn/sar_nano_swarm.git
cd sar_nano_swarm
git submodule update --init --recursive
```

Install Python dependencies:

```bash
python3 -m pip install -r requirements.txt
```

Build components in order — rmagine, the radar plugin, the CrazySim firmware, then Crazyswarm2. Each has its own build instructions in its directory (or see `CLAUDE.md` for the full sequence).

Source the environment before building or running anything:

```bash
source setup_env.sh
```

## Running a simulation

```bash
./eval_scripts/phase0_gate.sh -w sim_worlds/patched_worlds/cave_world.world
```

This launches Gazebo, spawns a simulated drone with the radar plugin attached, starts the firmware, and opens RViz.

To fly the drone (scripted, since manual flying requires a joystick):

```bash
python3 eval_scripts/quick_fly_test.py
```

## Experiment tracking

This project uses MLflow for tracking simulation runs:

```bash
mlflow ui --backend-store-uri sqlite:///mlflow.db
```