# CLAUDE.md — sar_nano_swarm

SAR nano-drone swarm simulation: ROS 2 Humble + Gazebo Harmonic (gz-sim8) + CrazySim SITL + custom FMCW radar perception, targeting GPS-denied victim search in collapsed structures. Solo developer, WSL2 on Windows.

**Source-of-truth docs** (read when task touches their domain; do not duplicate their content here):
- `Simulation_Training_Optimization_Roadmap_v2.md` — the 12-phase sim/validation plan (Phase 0–11). Phase gates live there.
- Technical plan (ethan_technical_plan) — system architecture: 3-tier fleet, anchor-SLAM, auctions, victim detection. ⚠ Its own "Phase 1–5" numbering is a *different* scheme from the roadmap's Phase 0–11. When the user says "Phase N," assume the **roadmap** numbering unless context says otherwise.

---

## 1. Decision stability — read this first

Nothing in this project is frozen. Treat decisions in three tiers:

**Tier A — settled unless the user explicitly reopens them:**
- ROS 2 Humble, Ubuntu 22.04, Gazebo Harmonic/gz-sim8 (not ROS 1, not Gazebo Classic — chosen for CrazySim compatibility).
- Custom `radarays_gz2` plugin instead of upstream RadaRays (which is ROS 1/Classic-only, incompatible; do not suggest vendoring it).
- Classical/inspectable coordination (auctions + response-threshold fallback). **Never propose MARL or learned coordination/control policies.** Learned components are scoped to: place-recognition encoder, possibly vitals classifier. Hyperparameter tuning = Optuna/CMA-ES, not RL.
- Anchor positions are **state variables, never known constants** — every anchor is instantiated at the dropping drone's *drifted* estimated pose and refined (multi-pose ranging, anchor-to-anchor ranging, loop closures). Any code/eval that treats dropped-anchor positions as ground truth is wrong by design (ground-truth anchors are allowed only as the Phase 2(a) baseline sanity check).
- Data outlives the drone: Class-A (tiny, instant: victim alerts, drone-down, claims) vs Class-B (bulk, deferred) split.
- C++ only where forced (gz-sim System plugins, firmware); **all algorithmic/ML/eval work in Python**. User does not know C++ — explain any unavoidable C++ change plainly and keep it minimal.

**Tier B — current thinking, likely to evolve (implement as configurable, don't hard-code):**
- Fleet composition (50 drones, 5 squads, ~6 GAP9 mappers), anchor spacing, bingo-fuel thresholds, redundant-coverage tolerance (<15%), radar band (24 vs 60 GHz vs 70–80 GHz 4D), Crazyflie 2.1 vs 2.1 Brushless, specific EKF vs fixed-lag smoother. Put such numbers in `configs/`, not in code.

**Tier C — placeholder, expect replacement:**
- Radar model in `radarays_gz2` is a stand-in: 360 × 1 planar scan, 0.1–30 m, single elevation row. Real target: multi-elevation 4D mmWave geometry + Doppler + measured degradation model layered on top. Don't build downstream code that assumes the placeholder's shape.
- `coordination/` and `configs/` are empty scaffolds.

**Maintenance rule:** when the user changes a decision, update this file in the same session (move items between tiers, delete contradicted lines). Stale instructions are worse than missing ones.

---

## 2. Repo map — where things live and go

| Path | Contents | New code goes here when… |
|---|---|---|
| `perception/radarays_gz2/` | Custom gz-sim System plugin (C++): rmagine Embree raycast → `sensor_msgs/PointCloud2` on `/radar/points` | Sensor geometry/plugin changes only |
| `perception/rmagine/` | Submodule (uos/rmagine, pinned). Never edit in place | — |
| `perception/` (new subdirs) | — | Radar degradation model, EKF/RIO, place-recognition encoder, vitals pipeline (roadmap Ph 1–5) |
| `coordination/` | Placeholder | Auctions, leader election, RTH, fracture logic (Ph 7–8) |
| `firmware_mods/CrazySim/` | Submodule: CrazySim + crazyflie-firmware (`crazysim` branch) + crazyswarm2_ws | PID gains, mass model, battery model (Ph 1) — prefer config/SDF edits over firmware C edits |
| `sim_worlds/` | `darpa_subt_worlds` submodule + `test_radar.world` | New/procedural worlds (Ph 4 step 1) |
| `eval_scripts/` | `check_radar_topic.sh` smoke test | All metrics, sweeps, regression suites; every run logs to MLflow |
| `configs/` | Placeholder | All tunable parameters (Tier B numbers) |
| `setup_env.sh` | Exports `SAR_NANO_SWARM_ROOT`, `GZ_SIM_RESOURCE_PATH`, `GZ_SIM_SYSTEM_PLUGIN_PATH`, `LD_LIBRARY_PATH` (incl. libiomp5), ROS overlays. **Source in every terminal**; assume the user forgot it when "topic not found"/"plugin not found" errors appear | — |

Build order & pinned submodule commits: see README. Path portability rule: world/plugin SDF paths are **relative to `SAR_NANO_SWARM_ROOT`** — never write `/home/<user>/…` absolute paths into tracked files.

---

## 3. Current status (update as phases close)

- **Phase 0: COMPLETE (exit gate passed 2026-07-09).** Crazyflie SITL in SubT tunnel world (`sim_worlds/phase0_tunnel_gate.sdf`) with `radarays_gz2` on the drone; `/radar/points` publishing ~10 Hz. Re-run: `./eval_scripts/phase0_gate.sh` (uses repo builds, or set `CRAZYSIM_FW` / `RADAR_PLUGIN_DIR` to `~/crazyflie_ws` artifacts).
- Verified working: SITL build (`sitl_make/build`, native cmake — **not** `make defconfig`/ARM kbuild), Crazyswarm2 build, radar plugin on flying model in tunnel gate world, MLflow (sqlite backend: `sqlite:///mlflow.db`, filesystem `./mlruns` backend is deprecated and errors).
- cfclient connects to SITL at `udp://127.0.0.1:19850`.
- **Phase 1 M1 (mass/inertia model): DONE.** `configs/airframe/payload.yaml` (base + per-component point masses, real datasheet weights for UWB Loco deck 3.3g / Flow deck v2 1.6g; radar/GAP9/pucks present but `enabled: false` PLACEHOLDER until weighed) + `eval_scripts/apply_payload.py` (parallel-axis mass/CoM/inertia rewrite of the generated SDF's `base_link`, called from `phase0_gate.sh` after radar injection — submodule untouched) + `eval_scripts/thrust_margin_check.py` (T/W gate, floor 1.5 in `configs/airframe/thrust_margin.yaml`, logs to MLflow). Verified: current sensor-only payload (33.1g) passes at T/W≈2.06; enabling the placeholder radar+GAP9+4 pucks (49.1g) drops it to T/W≈1.39 — **fails the floor**, confirming the Brushless-motor question is real, not hypothetical. See `.cursor/docs/Phase1_Physical_Fidelity_and_Sensor_Implementation_Plan.md`.
- **Phase 1 M2 (PID retune, cascade Optuna search): DONE, run live.** `sim_worlds/phase1_pid_tune.sdf` (flat/open world), `configs/airframe/{pid_gains_stock,pid_tune}.yaml`, `eval_scripts/{pid_gains,tune_pid,push_pid_gains,hover_gate}.py`. `tune_pid.py` got a `SIGALRM` watchdog + consecutive-timeout abort partway through (the `cf2` SITL binary crashes/hangs intermittently under long Optuna runs — teardown must kill `gz sim` too, since it holds the UDP ports `cf2` needs). Best Stage-1+2 gains committed to `configs/airframe/pid_gains_loaded.yaml`; `hover_gate.py` (M2 exit gate: RMS horizontal drift + mean |Z error| over a hold window) passes on the loaded-mass model with these gains.
- **Phase 1 M3 (IR ToF `gpu_lidar` sensor): DONE, gated live.** `configs/sensors/tof.yaml`, `eval_scripts/apply_tof_sensor.py` (injects the sensor into `base_link`, called from `phase0_gate.sh` after `apply_payload.py` — kept as a separate script, not folded into `apply_payload.py`), `eval_scripts/tof_gate.py` (rate + hover-plateau altitude-tracking gate). Live result: `/cf_0/tof_down` at ~29.9 Hz (floor 24 Hz), altitude error 0.8–1.8 cm at 0.3/0.6/1.0 m PID-held plateaus. `phase0_gate.sh` also auto-launches `ros_gz_bridge` for this topic when available (see §7 — it's provided by a separately source-built workspace, not the apt package). See `.cursor/docs/Phase1_Physical_Fidelity_and_Sensor_Implementation_Plan.md` §4.1 for live-run gotchas (sdformat's `<lidar><noise><type>` must be a child element, not an attribute like `<air_pressure>` uses; `gz topic -e --json-output` serializes `inf`/`nan` as JSON strings; `gz topic -f --duration` doesn't self-terminate).
- Next major work: M4 (UWB analytic range node), then M5 (Phase-1 exit gate: ±10 cm hold under noise+turbulence), then the non-linear battery model (**never the default linear drain**) and radar degradation model from real bench data.

---

## 4. Known pitfalls — check here BEFORE debugging builds

- **WSL Windows-PATH poisoning:** Windows PATH (Anaconda etc.) leaks into WSL; CMake then finds Anaconda's protobuf and gz-msgs10 fails with `ArenaStringPtr` errors. Fix already applied: `appendWindowsPath = false` under `[interop]` in `/etc/wsl.conf` + `wsl --shutdown`. If protobuf errors reappear, check `echo $PATH | grep -i anaconda` first.
- **`libiomp5.so` missing at plugin load:** comes from `pip install intel-openmp` → `~/.local/lib`. `setup_env.sh` handles `LD_LIBRARY_PATH`; `IOMP_LIB_DIR` overrides.
- **rmagine CMake:** must request `COMPONENTS core embree` and link both `rmagine::core` and `rmagine::embree`; `embree` alone → "missing rmagine::core".
- **rmagine has no prefab sensor models** in the pinned version (no `vlp16_900()` etc.) — construct `SphericalModel` field-by-field (`theta/phi: {min, inc, size}`, `range: {min, max}`). Don't invent rmagine API; if unsure of a struct/field, ask the user to grep the header and paste it.
- **gz-sim custom plugins attach at `<model>`/`<world>` level**, not inside `<sensor type="custom">` — nested form silently never runs (topic never appears).
- **gz-sim starts paused** — `PreUpdate` doesn't run until play (or `gz sim -r`). "Topic not published" is often just this or an unsourced `setup_env.sh`.
- **Offline worlds:** Fuel `model://sun`, `model://ground_plane` URIs fail without cache — use inline SDF lights/planes in test worlds.
- **`python` vs `python3`:** SITL cmake needs `python` (`python-is-python3` installed). WSL DNS previously fixed via static `/etc/resolv.conf` + `generateResolvConf = false`.
- Harmless noise to ignore: `librotors_gazebo_ros_interface_plugin.so` load failure; TBB dual-location CMake warning; WSLg `QStandardPaths` permission warning.
- **`gz sim` holds the UDP ports `cf2` needs** (19850/19950-range) — a stale `gz sim` process from a previous run silently prevents the next `cf2` from binding, even though `phase0_gate.sh` prints "Simulation ready." Always kill `gz sim` (not just `cf2`) between runs; if hangs persist, `wsl --shutdown` clears leftover VM state.
- **`mlflow.db` (sqlite) can hang indefinitely under concurrent/unclean access on the `/mnt/d` DrvFs mount** — SQLite's file-locking doesn't play well with 9p/DrvFs, and repeatedly `kill -9`ing scripts mid-run without letting their sqlite connection close leaves it wedged (`D` process state, uninterruptible). If a script hangs specifically at an MLflow `start_run()`/log call with no other explanation, check for orphaned processes with an open handle on `mlflow.db` first.
- **sdformat's `<lidar><noise>` wants `<type>gaussian</type>` as a child element**, not an attribute like `<air_pressure><pressure><noise type="gaussian">` uses — the attribute form is silently dropped (logs "XML Attribute[type] ... not defined in SDF"). Different sensor types, different noise schemas; don't assume they match.
- **`gz topic -f --duration N` does not self-terminate** — it streams "average rate:" blocks forever until killed, regardless of `--duration`. **`gz topic -e --json-output`** serializes non-finite doubles (`inf`/`nan`) as JSON *strings* (`"Infinity"`), not JSON numbers.
- **Teleporting a gravity-on, unpowered body via `gz service .../set_pose` and reading a sensor right after doesn't work for tight height control** — each `gz service`/`gz topic` CLI round-trip costs ~0.3–0.5 s, long enough to free-fall past the target height. Pausing the world + `WorldControl.multi_step` was tried as a fix and reliably returned "no valid return" for a `gpu_lidar` sensor (root cause not chased down). If you need a drone to sit still at a precise height for sensor validation, take off and hover it under the flight controller's own PID hold (per `hover_gate.py`'s pattern) instead of fighting gravity with teleports.

---

## 5. Simulation fidelity boundaries — never claim sim validates these

- **Raycasting is geometry only.** rmagine/radarays_gz2 knows nothing of frequency: no 70–80 GHz material penetration, band-specific multipath, or RCS. Realism comes from the **Phase-1 degradation model fitted to real bench data** (sparsity, dropout, multipath ghosts) applied on top. Downstream stages consuming raw raycast output are being flattered — say so.
- **No biological radar physics:** vital-signs work (Ph 5) validates against real public datasets only; in mission sims the vitals trigger is **sampled from the Phase-5 measured detection/false-positive distribution**, never a scripted guaranteed hit.
- **Thermal-inertial odometry: real-hardware-only.** Gazebo low-contrast tunnel thermal modeling is too weak. Thermal, acoustic, CO₂ cues are all mocked in sim.
- **Contact physics (perching, Ph 6) requires the MuJoCo backend**, not Gazebo.
- **Anchor geometry in evals: floor-only (coplanar)** — dropped pucks land on floors → bad vertical DOP. Idealized wall/ceiling placements flatter results; flag any eval using them.
- **Comms:** CrazySim delay sim ≠ tunnel RF. Ph 7 needs the distance+bend attenuation model for the drop-on-link-margin policy.
- **Battery:** default linear drain is banned for anything feeding bingo-fuel logic.

---

## 6. Working rules for the agent

1. Prefer terse, command-first replies; the user asks for commands and pastes errors back. Explain only what's notable.
2. Before writing code: confirm which roadmap phase it serves and put it in the mapped directory; put tunables in `configs/`.
3. Python (`rclpy`, numpy/scipy, PyTorch) by default. Touch C++ only for the plugin/firmware layer, minimally, with plain-language explanation.
4. Never fabricate rmagine/gz-sim/CrazySim API details. If a symbol is unverified, say so and request a grep/paste rather than guessing.
5. Every experiment/eval logs params, seed, metrics to MLflow (sqlite URI). No new tracking tools.
6. Don't edit submodules in place; changes to them go via fork + pinned-commit bump, or live in this repo's own dirs.
7. Respect phase gates: don't build Phase-N features on unvalidated Phase-(N−1) assumptions without flagging it. Metrics that matter are the gate metrics (e.g., error at deepest traverse point, Z error, not just mean ATE).
8. On-drone compute targets **GAP9** (150 int-GOp/s, ~128 KB L1, ~1.5 MB L2, ~32 MB L3), not the dev machine — reject designs that only fit a laptop.
9. When a decision changes, update this file (Section 1 tiers, Section 3 status) in the same session.

---

## 7. Known doc↔repo discrepancies (as of 2026-07-08)

- Roadmap says "clone and build `radarays_gazebo_plugins`" (Phase 0 step 8) — superseded: that package is ROS 1-only; the custom `radarays_gz2` fulfills that step. Roadmap text not yet amended.
- README's RadaRays link points to `robotics-upo`; actual upstream is `uos/radarays_gazebo_plugins`. Cosmetic.
- Roadmap assumes RadaRays' radar *physics* (ray-traced FMCW reflections); current plugin is plain first-hit raycasting — a step below even RadaRays until the degradation model and/or reflection model is added. Factor this into any fidelity claims.
- Technical plan's fleet/payload specifics (GAP9 counts, puck mass) are design intent, not represented anywhere in code yet.
