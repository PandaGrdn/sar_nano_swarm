# Phase 1 Implementation Plan — Physically-Accurate Airframe + IR & UWB Sensors

**Roadmap mapping:** This expands **Phase 1 steps 1, 3, 6** of `Simulation_Training_Optimization_Roadmap_v2.md` (mass/inertia model, sensor noise, `--ground-effect/--turbulence` flags) and pre-builds the sensor plumbing that **Phase 2** (EKF: IMU + radar Doppler + **downward ToF** + **UWB range** updates) consumes. It does **not** implement the EKF itself (that is Phase 2) — it produces the *physically accurate airframe* and the *IR + UWB measurement streams* the EKF will fuse.

**Prereq status:** Phase 0 is COMPLETE (SITL Crazyflie + `radarays_gz2` in `sim_worlds/phase0_tunnel_gate.sdf`, `/radar/points` @ ~10 Hz).

> **⚠ SUPERSEDED BY v3 (read before touching §5/UWB or M4):** `Simulation_Training_Optimization_Roadmap_v3_MOONSHOT.md` (2026-07-27) retires the dropped-anchor-puck architecture entirely — no infrastructure gets deployed inside the structure anymore. UWB's role becomes an **inter-drone range+bearing (PDoA) mesh** gauged only at the entrance, not a drone↔anchor ranging system. This doc's mass model (§2/M1) and ToF sensor (§4/M3) are **unaffected** — they don't depend on the anchor architecture. §5/M4 (UWB) is **substantially rewritten below**: the original drone↔anchor analytic node is kept only as a one-time baseline control-arm run (v3 Phase 2 item 1), and the real M4 deliverable is now the inter-drone PDoA plugin described in §5.5. M1's mass budget also needs one correction: **anchor pucks are retired from the budget** (v3 Phase 6: "Remove: puck payload handling and drop-mechanism behaviors"), and the UWB deck entry needs relabeling from a single-antenna Loco deck to a dual-antenna PDoA module (§2).

---

## 0. Read this first — simulation fidelity boundaries

These are hard constraints from `AGENTS.md §5`. The plan is designed *around* them; do not let the implementation quietly cross them.

| Sensor | Sim fidelity | What we may claim | What we must NOT claim |
|---|---|---|---|
| **UWB ranging** (drone↔anchor, v2 control arm — §5.1–5.4) | High (geometry + analytic noise) | Range accuracy, NLOS/multipath dropout, DOP behaviour | — (this one is genuinely good in sim) |
| **UWB PDoA ranging+bearing** (inter-drone mesh, v3 primary — §5.5) | High for range; angle fidelity only as good as the off-boresight/AoA-cone model, which is placeholder until Phase 1's external-validation gate (v3 roadmap Phase 1 item 1) | Range accuracy, angle accuracy within a validated envelope, AoA-cone range-only fallback, NLOS/multipath dropout | Angle accuracy outside the externally-validated range; anything about a *specific* unbuilt PDoA module's real aperture until Phase 11 bench data exists |
| **IR ToF rangefinder** (downward/obstacle) | High (raycast geometry + noise) | Altitude/obstacle ranging under geometry | Material-specific reflectivity dropout (approximate only) |
| **Thermal / LWIR camera** (victim IR) | **Low — mocked** | *Out of scope this phase (see §4.2)* | ❌ That the sim validates thermal victim detection or **thermal-inertial odometry** (real-hardware-only per `AGENTS.md §5`) |

**Consequence for this plan:** "IR sensor" = **IR ToF rangefinder** (decided). Both the **IR ToF** and **UWB** streams are built as *real, EKF-grade measurement sources*. A thermal/LWIR camera is **deferred** (§4.2) — not part of this phase.

---

## 1. Objectives (definition of done)

1. **Physically accurate drone**: SDF mass, centre-of-mass (CoM), and inertia tensor reflect the *real loaded airframe* (base + battery + every payload deck), not the default 27 g. PID gains re-tuned for that mass. Thrust margin sanity-checked.
2. **IR sensor in sim (= IR ToF rangefinder, decided)**: a downward + (optional) 5-direction **IR ToF** ranging stream published as ROS 2 topics with datasheet-matched noise. (Thermal/LWIR camera is deferred — §4.2.)
3. **UWB sensor in sim (revised for v3):** two deliverables, not one —
   - **(retained, control-arm only)** the original analytic drone↔anchor + anchor↔anchor range generator with ±10 cm Gaussian noise + NLOS/multipath outliers, run **once** as v3's frozen baseline comparison (v3 Phase 2 item 1). Anchors here are still **configurable entities, positions treated as state downstream — never ground truth** (`AGENTS.md §1 Tier A`).
   - **(new, primary — v3 W1)** an **inter-drone UWB PDoA plugin**: per drone-pair range + azimuth + elevation in the observer's body frame, with an AoA field-of-view cone (~90–120°, range-only fallback outside it), off-boresight angle-error growth, NLOS/multipath outliers, and a fixed entrance/base-station node as the localization gauge. No anchor pucks are dropped; nothing is deployed inside the structure. See §5.5.
4. **Sensor weights folded into the mass model**: every added sensor's real mass increases total mass, shifts CoM, and updates inertia — automatically, from one config file.
5. Everything **config-driven** (`configs/`), **portable** (paths relative to `SAR_NANO_SWARM_ROOT`), submodule left unedited, and every eval logged to **MLflow** (`sqlite:///mlflow.db`).

---

## 2. Sensor weights (real datasheet values) → mass budget

All weights below are **measured datasheet values** except those marked ⚠ **PLACEHOLDER** (custom hardware with no public spec — you must replace with a scale measurement). These live in `configs/`, not code (`AGENTS.md §1 Tier B`).

| Component | Mass (g) | Source | Tier |
|---|---|---|---|
| Crazyflie 2.1 (brushed) incl. battery | ~27.0 | Bitcraze | A (base) |
| Crazyflie 2.1 **Brushless** (2024) | ⚠ measure (~heavier) | roadmap Ph1.7 recommends this | B |
| **UWB PDoA module (dual-antenna, inter-drone range+bearing)** — v3 replaces the single-antenna Loco deck below | ⚠ **PLACEHOLDER ~4.0** (budgeted slightly above the Loco deck's 3.3 g for the 2nd antenna + RF switch; no committed part yet — candidates are DW3xxx-class dual-antenna boards, e.g. the ETH-PBL reference design cited in v3 roadmap Phase 1 item 1) | v3 architecture requirement; mass not yet datasheet-sourced | B |
| ~~Loco Positioning deck (UWB, DWM1000)~~ — **RETIRED, v3**: single-antenna, range-only, can't do the PDoA bearing v3 requires. Superseded by the row above. | ~~3.3~~ | [Bitcraze datasheet](https://www.bitcraze.io/products/loco-positioning-deck/) | — |
| **Flow deck v2** (VL53L1x ToF **down** + PMW3901 flow) | **1.6** | [Bitcraze](https://www.bitcraze.io/products/flow-deck-v2/) | A |
| Z-ranger deck v2 (single **down** VL53L1x only) | 1.3 | Bitcraze | A (alt to Flow) |
| **Multi-ranger deck** (5× VL53L1x, F/B/L/R/up) | **2.3** | [Bitcraze](https://www.bitcraze.io/products/multi-ranger-deck/) | A |
| **FLIR Lepton 3.5** LWIR module (bare) | **0.90** | FLIR datasheet | A |
| FLIR Lepton 3.5 as a flyable **thermal deck** | ~2.1 | Bitcraze fwd-connector prototype | B |
| MLX90640 thermal deck (alt) | ~2.0 | Bitcraze prototype | B |
| AI deck (GAP8; GAP9 proxy for mass) | 4.4 | Bitcraze | B |
| **GAP9 compute shield** | ⚠ **PLACEHOLDER 3–5** | custom | B |
| **mmWave/60GHz radar module** (odometry/mapping/obstacle/vitals role only, v3 — no longer inter-drone) | ⚠ **PLACEHOLDER 5–8** | custom | B |
| ~~Anchor puck ×N carried for dropping~~ — **RETIRED, v3**: no anchors are dropped inside the structure anymore (v3 Phase 6: "Remove: puck payload handling and drop-mechanism behaviors"). Payload margin this frees up is noted by v3 as partially relieving the sortie-duration floor. | ~~⚠ PLACEHOLDER~~ | — | — |

**Reality check — thrust margin (feeds the Brushless decision, roadmap Ph1.7):**
A representative "mapper" load = 27 (base) + 4.0 (UWB PDoA) + 1.6 (ToF/flow) + 2.1 (thermal) ≈ **34.7 g before radar and compute**. Add ⚠ radar+GAP9 and you are near/over the brushed 2.1's usable thrust ceiling → poor control margin. This is exactly why the roadmap flags the **2.1 Brushless**. The mass model must make this visible (see §3.3), not hide it. (Retiring the anchor pucks from the budget *helps* this margin slightly vs. the v2 numbers — one of the few places the moonshot architecture is cheaper.)

> Note: the current stock SDF (`.../models/crazyflie/model.sdf.jinja`) models **base_link 0.025 kg + 4×0.0008 kg props = 0.0282 kg**. That is the number we are replacing.

---

## 3. Workstream A — Physically accurate airframe

### 3.1 Where the edit goes (do NOT edit the submodule in place)
`model.sdf.jinja` lives under the `firmware_mods/CrazySim` submodule (`AGENTS.md §2/§6.6`: no in-place submodule edits). `phase0_gate.sh` already **generates** the model from Jinja and **injects** the radar plugin via a Python XML step at launch. We extend that same pattern:

- **New script** `eval_scripts/apply_payload.py` — takes the generated `/tmp/<model>_<id>.sdf`, reads `configs/airframe/payload.yaml`, and **rewrites the `base_link` `<inertial>` block** (mass, CoM `<pose>`, inertia tensor) + **injects sensor elements** (§4, §5). Called from `phase0_gate.sh` right after the radar-inject step.
- Result: submodule stays pristine; the airframe is fully described by `configs/`, portable, and versioned.

**Status: DONE (M1), revised for v3.** `configs/airframe/payload.yaml` retired the `anchor_pucks` placeholder entry and relabeled `uwb_loco_deck` → `uwb_pdoa_module` (see §2's mass-table revision). `apply_tof_sensor.py` (§4) ended up as a **separate** script from `apply_payload.py` rather than folded in as sketched above — kept for single-responsibility. Re-ran `apply_payload.py` + `thrust_margin_check.py` against a fresh jinja-generated SDF: total mass 33.1 g → **33.8 g** (the +0.7 g uwb_loco_deck→uwb_pdoa_module bump; pucks were already `enabled: false` so their removal didn't change anything), T/W 2.062 → **2.019**, still comfortably **PASS** against the 1.5 floor.

### 3.2 Composite mass / CoM / inertia (the math, done once in Python)
Treat each deck as a point mass (nano-decks are thin; a small-box refinement is optional) at offset `r_i=(x,y,z)` from `base_link` origin.

- Total mass: `M = Σ m_i`
- CoM: `c = (Σ m_i r_i) / M`
- Inertia about CoM (parallel-axis, point-mass form): for each component `d_i = r_i − c`,
  `I += m_i (‖d_i‖² · E₃ − d_i d_iᵀ)`, plus the base's own tensor about its own CoM.

`apply_payload.py` computes `M`, `c`, `I` and writes:
```xml
<inertial>
  <pose>cx cy cz 0 0 0</pose>   <!-- CoM offset -->
  <mass>M</mass>
  <inertia><ixx/><ixy/><ixz/><iyy/><iyz/><izz/></inertia>
</inertial>
```
Keep the base tensor from the stock SDF as the `base` entry in the YAML so nothing is lost.

### 3.3 Thrust-margin check (cheap, high-value)
Add `eval_scripts/thrust_margin_check.py`: from the motor model in the SDF (`maxRotVelocity`, `motorConstant`; 4 rotors) compute max static thrust `T_max = 4 · motorConstant · maxRotVel²`, compare to `M·g`. Emit **thrust-to-weight ratio** and fail loudly if T/W < a configurable floor (e.g. 1.5). Log to MLflow. This is the quantitative form of the Brushless argument.

### 3.4 PID retune for the loaded mass (roadmap Ph1.2)
- After the mass model is in, hover will be sluggish/unstable on the old gains. Re-tune via the CrazySim PID-tuning workflow; **save the gain set** to `configs/airframe/pid_gains_loaded.yaml` (this is what eventually flashes to hardware).
- Gains belong in `configs/`, not firmware C. Prefer parameter/config over editing firmware (`AGENTS.md §2`).

### 3.5 Enable realistic disturbances (roadmap Ph1.6)
Run hover + waypoint sweeps with CrazySim's `--sensor-noise --ground-effect --wind-speed --turbulence`. These are launch flags; expose them through `phase0_gate.sh` pass-through args so evals can sweep them.

---

## 4. Workstream C — IR sensor (IR ToF rangefinder)

**Decision: "IR sensor" = IR ToF laser rangefinder** (VL53L1x-class, 940 nm) — *ranging*. High sim fidelity, directly needed by the Phase-2 EKF ("downward ToF for altitude"). This is the sole IR deliverable for this phase; the thermal/LWIR camera is **deferred** (§4.2).

### 4.1 IR ToF rangefinder — real, EKF-grade
- **Sim mechanism:** Gazebo `gpu_lidar` sensor (single ray = `samples: 1` in H & V), one per beam direction. This is pure geometry (raycast), which is exactly what a ToF laser measures.
  - **Downward** beam (required by EKF for altitude) → models Flow deck v2 / Z-ranger.
  - Optional **5-direction** set (front/back/left/right/up) → models Multi-ranger; useful early for Phase-3 obstacle avoidance.
- **Requires** the `gz-sim-sensors-system` (already present with `ogre2` in `phase0_tunnel_gate.sdf`).
- **SDF (injected by `apply_payload.py`)**, per beam:
```xml
<sensor name="tof_down" type="gpu_lidar">
  <topic>/cf_0/tof_down</topic>
  <update_rate>30</update_rate>          <!-- VL53L1x ~ up to 50 Hz; use configs -->
  <pose>0 0 -0.02 0 1.5708 0</pose>       <!-- pointing -Z -->
  <lidar>
    <scan><horizontal><samples>1</samples><min_angle>0</min_angle><max_angle>0</max_angle></horizontal>
          <vertical><samples>1</samples><min_angle>0</min_angle><max_angle>0</max_angle></vertical></scan>
    <range><min>0.04</min><max>4.0</max><resolution>0.001</resolution></range>  <!-- VL53L1x: 4 cm–4 m -->
    <noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise>            <!-- ~1 cm, from configs -->
  </lidar>
</sensor>
```
- **Datasheet params → `configs/sensors/tof.yaml`** (Tier B): range 0.04–4.0 m, noise ~1 cm (surface/light dependent), rate. VL53L1x-based (Flow/Z-ranger/Multi-ranger).
- **ROS 2 bridge:** map the gz topic to ROS via the CrazySim/`ros_gz_bridge` mechanism already used for other topics (verify the bridge line; add if missing). The EKF (Phase 2) subscribes to `/cf_0/tof_down`.
- **Mass:** adds Flow deck v2 **1.6 g** (or Z-ranger **1.3 g**, or Multi-ranger **2.3 g**) — folded in via §3.2.

**Status: DONE, gated live.** `configs/sensors/tof.yaml` + `eval_scripts/apply_tof_sensor.py` (kept as a **separate** script from `apply_payload.py` rather than folded in as originally sketched in §3.1/§8 — single-responsibility: `apply_payload.py` stays mass/inertia-only, `apply_tof_sensor.py` injects the sensor element(s); both are called back-to-back from `phase0_gate.sh`). Wired into `phase0_gate.sh` via `--no-tof`/`--tof-config`. `eval_scripts/tof_gate.py` is the M3 exit gate. Live-run findings, all fixed:
  - The exact `<noise type="gaussian">...</noise>` attribute form shown above (copied from the `<air_pressure>` sensor's noise syntax) is **invalid for `<lidar><noise>`** — sdformat logs "XML Attribute[type] ... not defined in SDF" and silently drops the noise model. `<lidar><noise>` wants `<type>gaussian</type>` as a **child element** (verified against `/usr/share/sdformat12/1.9/lidar.sdf`). Fixed in `apply_tof_sensor.py`.
  - **ROS bridge — NOT actually missing**, contrary to how it looked at first: `ros_gz_bridge` isn't in the system apt tree, but setup_env.sh sources a separately source-built workspace (`${ROS_GZ_WS:-$HOME/ros2_ws}/install`) that has it. `phase0_gate.sh` now auto-launches `ros2 run ros_gz_bridge parameter_bridge "/cf_<id>/tof_down@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"` when `ros_gz_bridge` resolves, and warns (non-fatal) otherwise. Verified live: bridged topic publishes at ~27–28 Hz measured via `ros2 topic hz`.
  - `tof_gate.py`'s altitude-tracking check does **not** use `gz service .../set_pose` teleports — teleporting a gravity-on, unpowered body and then reading the sensor immediately still costs ~0.3–0.5 s per `gz service`/`gz topic` CLI round-trip, in which the drone free-falls past the target height (verified: a teleport to 2.0 m read back as 0.895 m by read time). Pausing the world and stepping via `WorldControl.multi_step` was also tried live and reliably returned "no valid return" (unclear why; not chased further). The gate instead reuses `hover_gate.py`'s (M2) proven cflib connect/arm/`MotionCommander` hover pattern and samples the down-beam range while the drone is genuinely PID-held at each plateau — no free-fall, so CLI latency doesn't matter. Live result: 0.3/0.6/1.0 m plateaus tracked within 0.8–1.8 cm of the EKF's `stateEstimate.z`.
  - `gz topic -e --json-output` serializes non-finite doubles (`inf`/`nan`, e.g. "no return") as JSON **strings** (`"Infinity"`), not JSON numbers — `float()` handles this fine, but naive numeric summing does not.
  - `gz topic -f --duration N` does **not** self-terminate after N seconds; it streams average-rate blocks forever until killed.

### 4.2 Thermal / LWIR camera — DEFERRED (out of scope this phase)
Not implemented now (you selected IR ToF). Recorded here so the option is documented if revisited:
- Gazebo Harmonic *can* render one (`<sensor type="thermal">` + `gz-sim-thermal-sensor-system` sensor plugin + `gz-sim-thermal-system` visual `<temperature>` on victim models like `rescue_randy`), but it is **mocked/non-validating** (no LWIR material physics, no thermal-inertial odometry validation — `AGENTS.md §5`).
- If added later: implement its **mass only** (~2.1 g Lepton 3.5 deck) for physics accuracy behind a `configs` flag, and **never** feed the thermal image into Phase-2+ logic.
- **For now: exclude the thermal deck mass from `payload.yaml`** unless the real airframe actually carries one.

---

## 5. Workstream D — UWB sensor (REVISED FOR v3 — read the banner at the top of this doc)

**There is no native Gazebo UWB sensor.** UWB is modelled **analytically** — this is the correct and standard approach, and it fits `AGENTS.md §1` (anchors are state, positions never known constants).

**v3 changes the primary architecture.** §5.1–5.4 below (drone↔anchor + anchor↔anchor ranging to dropped pucks) is the **v2 design**. It is **not deleted** — v3 Phase 2 item 1 explicitly keeps it as a **frozen, one-time baseline control arm** ("run it once, properly, and freeze the results. Every W1 claim is reported against this."). But it is no longer what M4 is *for*. The new primary deliverable is **§5.5: an inter-drone UWB PDoA plugin** — a gz-sim System plugin, not a simple analytic ROS node, because it needs efficient per-pair geometry in each drone's own body frame every sim step (same reason `radarays_gz2` is a C++ System plugin and not a Python node).

**M4 is therefore split:**
- **M4a** (small, lower priority): keep/finish the v2 analytic range node below, run it once for the control-arm dataset, freeze results.
- **M4b** (the real work, and gated earlier than "M4" implied — v3 wants a basic version working to *exit Phase 0*): the inter-drone PDoA plugin, §5.5.

### 5.1 Architecture (v2 design — M4a, control-arm only)
A ROS 2 node `perception/uwb_sim/uwb_range_node.py` (rclpy, per `AGENTS.md §6.3`):
- **Inputs:** drone ground-truth pose (from gz `/cf_*/odom` or a gz→ROS pose bridge) and a set of **anchor world positions** from `configs/sensors/uwb_anchors.yaml`.
- **Per range measurement** (drone↔anchor, and **anchor↔anchor** — cheap, stiffens the chain per roadmap Ph2.3):
  1. True Euclidean distance `d`.
  2. Add **Gaussian noise** σ ≈ **0.10 m** (DWM1000 ±10 cm datasheet spec).
  3. **NLOS / multipath outliers:** raycast between the two endpoints against the world mesh (reuse the `rmagine`/Embree map already loaded by the radar plugin, or a lightweight Python occlusion check). If occluded → add a **positive NLOS bias** + inflate σ, and with probability `p_dropout` **drop** the measurement. This is what makes tunnel UWB realistic.
  4. Respect **max range** (~10 m tested for the Loco deck) → beyond it, drop.
- **Output:** a custom/`std_msgs`-style range array or `sensor_msgs`-compatible message on `/cf_0/uwb/ranges`, timestamped, with `(anchor_id, range, sigma, los_flag)`.

### 5.2 Anchors are configurable entities, positions are NOT truth (M4a)
- `configs/sensors/uwb_anchors.yaml` holds anchor **world** positions **for the simulator to generate ranges from** — this is the sim oracle, analogous to Gazebo ground truth.
- **Critical (`AGENTS.md §1 Tier A`):** downstream (Phase 2) the EKF must **not** read these YAML positions. Anchors are instantiated at the dropping drone's *drifted estimated* pose and refined as state. Keep a hard wall: the sim range generator knows true anchor positions; the estimator never does. Document this at the top of both files.
- **Geometry for evals:** use **floor-only / coplanar** anchor layouts (`AGENTS.md §5`) — dropped pucks land on the floor → bad vertical DOP. Do not use idealized floor/wall/ceiling placements.

### 5.3 Noise params → `configs/sensors/uwb.yaml` (Tier B) (M4a)
`sigma_los: 0.10`, `nlos_bias_mean`, `nlos_sigma`, `p_dropout_nlos`, `max_range: 10.0`, `ranging_rate` (Loco: ~80 Hz/anchor with 6 anchors), `anchor_to_anchor: true`.

### 5.4 Mass (M4a — RETIRED from the live payload budget, v3)
Was: UWB Loco deck **3.3 g** + carried anchor pucks (⚠ placeholder). **Both retired from `payload.yaml`'s live budget** — see §2. The Loco deck's *analytic ranging math* (§5.1–5.3) is still used for the M4a control-arm run; its *mass line* just isn't part of the airframe anymore because the real hardware being flown for v3 is the PDoA module in §5.5, not this deck.

### 5.5 Inter-drone UWB PDoA plugin (M4b — v3 primary deliverable)

**Decision:** a gz-sim **System plugin** (C++, sibling to `radarays_gz2` — same reasoning: needs per-entity-pair geometry computed against live sim state every step, which is what System plugins are for, not a decision to prefer C++ generally per `AGENTS.md §1`/`§6.5`).

- **Per drone pair, per update tick, emit:** range (m), azimuth (rad), elevation (rad) — **in the observing drone's own body frame** (this is what makes mutual-bearing yaw-constraint factors possible in Phase 2's mesh estimator: two drones each report a bearing to the other, and disagreement/consistency between those two bearings is an observable that constrains their relative yaw).
- **Identity is always known** — UWB ranging is a two-way physical exchange (TWR/PDoA), not a passive detection, so there's no data-association ambiguity to model (unlike radar returns). Emit `(peer_id, range, azimuth, elevation, in_aoa_cone: bool)`.
- **AoA field-of-view cone (~90–120°, configurable):** outside the cone, still emit the **range-only** edge (`azimuth`/`elevation` null or flagged invalid) rather than dropping the measurement entirely — per the v3 roadmap, range-only edges still contribute to the pose graph, just without the rigidity that range+bearing edges give it.
- **Noise model (placeholder until Phase 1's external-validation gate):** range noise is roughly omnidirectional (TWR ranging doesn't care about orientation); angle noise **grows off-boresight** and must reproduce the ETH-PBL dual-antenna characterization the v3 roadmap cites (~2.4° mean accuracy within ±45° of boresight, for a *larger, tuned* module — budget conservatively, 5–15°, for the ~4–5 cm antenna spacing a nano-drone can actually fit, until real bench data says otherwise). NLOS/multipath outliers via the same occlusion-raycast approach as §5.1.3.
- **Entrance/base-station node:** a fixed, non-drone entity with its own PDoA unit — this is the "gauge" the whole mesh's absolute position is hung off of. Model it as a static world entity the plugin treats like any other peer (so the same range+bearing math applies uniformly), not a special case.
- **Config → `configs/sensors/uwb_pdoa.yaml`** (Tier B, new — see §6): `sigma_range_m`, `aoa_fov_deg`, `angle_error_deg_at_boresight`, `angle_error_growth_model` (placeholder functional form until Phase 1 validation replaces it), `nlos_bias_mean`, `nlos_sigma`, `p_dropout_nlos`, `max_range_m`, `update_rate_hz`, `entrance_node_pose` (this one *is* allowed to be a known constant — it's a real fixed base station, not a dropped/drifted anchor).
- **Gate (part of v3's Phase 0 exit criterion, i.e. wanted before this plan's own M4b is "done"):** two SITL drones exchanging simulated UWB range+bearing measurements on a ROS topic, including correct range-only fallback when a neighbor is outside the AoA cone.
- **Status: NOT STARTED.** This is the next real implementation task after this doc revision.

---

## 6. Config files to create (all Tier B tunables here, not in code)

```
configs/
  airframe/
    payload.yaml            # base inertial + component list {name, mass_g, pose xyz, optional box dims}
    pid_gains_loaded.yaml   # re-tuned gains for the loaded mass (from §3.4)
  sensors/
    tof.yaml                # IR ToF: range, noise, rate, which deck (flow/zranger/multiranger). DONE.
    uwb.yaml                # M4a (control-arm only): UWB noise/NLOS/dropout/range/rate params
    uwb_anchors.yaml        # M4a (control-arm only): SIM-ORACLE anchor world positions (floor-only); NEVER read by the estimator
    uwb_pdoa.yaml           # M4b (v3 primary, NEW): inter-drone PDoA noise/AoA-cone/entrance-node params — NOT YET CREATED
```

`payload.yaml` sketch (reflects the v3-revised, **live** budget — actual file has real datasheet sources and per-component `enabled` flags, see §2):
```yaml
base:
  mass_g: 25.0    # stock airframe+battery; sensor mass is additive below (see actual payload.yaml for exact base figure)
  com_xyz_m: [0.0, 0.0, 0.0]
  inertia: {ixx: 1.6572e-5, iyy: 1.6656e-5, izz: 2.9262e-5, ixy: 0, ixz: 0, iyz: 0}
components:
  - {name: uwb_pdoa_module, mass_g: 4.0, pose_xyz_m: [0.0,  0.0,  0.010]}   # v3: PLACEHOLDER dual-antenna PDoA module, replaces uwb_loco_deck
  - {name: flow_deck_v2,    mass_g: 1.6, pose_xyz_m: [0.0,  0.0, -0.010]}   # deck below (down ToF+flow)
  # thermal deck deferred (§4.2) — add only if the real airframe carries one
  - {name: gap9_compute_shield, mass_g: 6.0, pose_xyz_m: [0.0, 0.0,  0.008], enabled: false}   # PLACEHOLDER — measure
  - {name: mmwave_radar_module, mass_g: 4.5, pose_xyz_m: [0.02, 0.0, 0.000], enabled: false}   # PLACEHOLDER — measure
  # anchor_pucks component RETIRED in v3 — no longer part of the mission architecture, removed from the file entirely
```

---

## 7. Validation & gates (every run → MLflow: params, seed, metrics)

| Check | Script | Pass criterion |
|---|---|---|
| Mass model applied | inspect generated SDF | `<mass>` = Σ config, CoM offset present, inertia recomputed |
| Thrust margin | `thrust_margin_check.py` | T/W ≥ configurable floor; else flag "needs Brushless" |
| IR ToF stream | `tof_gate.py` | `/cf_0/tof_down` publishes at configured rate; ranges track altitude — **DONE, PASSED** |
| UWB stream (M4a, control-arm only) | `uwb_range_node` self-test | ranges = true + noise; NLOS drops when wall occludes; anchor↔anchor present |
| UWB PDoA mesh edge (M4b, v3 primary) | new gate TBD when M4b is implemented | two drones exchange range+bearing on a ROS topic; range-only fallback fires correctly outside the AoA cone |
| **Phase-1 gate** (roadmap) | hover/waypoint sweep | **position hold within tolerance (e.g. ±10 cm) under sensor noise + mild turbulence, in an empty world**, on the *loaded-mass* model with re-tuned PID |

The Phase-1 exit gate is the roadmap's: stable loaded-mass flight under realistic noise **before** any radar/UWB fusion (that's Phase 2).

---

## 8. File-by-file change list

**New:**
- `eval_scripts/apply_payload.py` — rewrite inertial + inject ToF/thermal sensors from configs. **(done)**
- `eval_scripts/thrust_margin_check.py` — T/W check → MLflow. **(done)**
- `eval_scripts/pid_gains.py`, `eval_scripts/tune_pid.py`, `eval_scripts/push_pid_gains.py` — cascade PID retune (§3.4). **(code complete, not yet run live)**
- `sim_worlds/phase1_pid_tune.sdf` — empty/open world for hover/PID testing. **(done)**
- `perception/uwb_sim/uwb_range_node.py` (+ `package.xml`/setup if a ROS pkg) — analytic UWB ranges. **(M4a, control-arm only — not started)**
- `configs/airframe/payload.yaml` **(done, revised for v3 — see §2)**, `configs/airframe/thrust_margin.yaml` **(done)**, `configs/airframe/pid_gains_stock.yaml` **(done)**, `configs/airframe/pid_tune.yaml` **(done)**, `configs/airframe/pid_gains_loaded.yaml` **(done, produced by tune_pid.py, M2 ran live)**
- `eval_scripts/apply_tof_sensor.py` — inject `gpu_lidar` ToF beam(s) into `base_link` from `configs/sensors/tof.yaml`. **(done)**
- `eval_scripts/tof_gate.py` — M3 exit gate (rate + hover-plateau altitude tracking). **(done)**
- `eval_scripts/hover_gate.py` — M2 exit gate (RMS horizontal drift + mean |Z error|). **(done)**
- `configs/sensors/tof.yaml` **(done)**, `configs/sensors/{uwb,uwb_anchors}.yaml` (M4a, control-arm — not started; thermal deferred, §4.2)
- **NEW for v3 M4b:** a gz-sim System plugin (C++, name TBD e.g. `uwb_pdoa_gz`, sibling to `perception/radarays_gz2/`) — inter-drone range+bearing. `configs/sensors/uwb_pdoa.yaml` (new). **Not started.**

**Edited:**
- `eval_scripts/phase0_gate.sh` — after the radar-inject block, call `apply_payload.py`; add pass-through flags for `--sensor-noise/--ground-effect/--wind-speed/--turbulence`; optionally launch `uwb_range_node`.
- `configs/rviz/radar.rviz` — add ToF range + (optional) thermal image + UWB range displays.
- README / `AGENTS.md §3` status — update when the Phase-1 gate passes.

**Untouched:** the CrazySim submodule (`model.sdf.jinja` etc.) — all changes via launch-time injection + configs.

---

## 9. Suggested sequencing (milestones)

1. **M1 — Mass model** (§2, §3.1–3.3): `payload.yaml` + `apply_payload.py` inertial rewrite + thrust check. *Gate: SDF shows correct M/CoM/I; T/W reported.*
   **Status: DONE, revised for v3.** Original gate: 33.1 g total, T/W=2.062, PASS. **v3 revision (this session):** retired the `anchor_pucks` placeholder component and relabeled `uwb_loco_deck` → `uwb_pdoa_module` (mass bumped 3.3→4.0 g placeholder, pending a real PDoA part choice — see §2). Re-ran `apply_payload.py` + `thrust_margin_check.py` live against a fresh jinja-generated SDF: **33.8 g total, T/W=2.019, PASS.**
2. **M2 — PID retune + disturbance flags** (§3.4–3.5): stable hover on loaded mass. *Gate: hover holds.*
   **Status: DONE, run live.** Two-stage cascade Optuna search (Stage 1 = pid_rate+pid_attitude, Stage 2 = velCtlPid+posCtlPid), `configs/airframe/pid_gains_loaded.yaml` committed, `hover_gate.py` passing (RMS horizontal drift + mean |Z error| within thresholds). `tune_pid.py` needed a `SIGALRM` watchdog + consecutive-timeout abort added mid-run for `cf2` SITL crashes (see `AGENTS.md` §3/§4). **v3 revision (this session):** since M1's mass only moved 33.1→33.8 g (+0.7 g), re-ran `hover_gate.py` only (no re-tune) as a cheap sanity check against the updated mass model, against a freshly launched sim (existing gains still applicable at this small a mass delta — no need to burn another multi-hour Optuna run). **Result: PASS** — mean |Z error| = 1.18 cm (threshold 5 cm), RMS horizontal drift = 8.03 cm (threshold 10 cm), took off cleanly, no divergence. Existing `pid_gains_loaded.yaml` gains remain valid; no re-tune needed for the v3 mass revision. The `--sensor-noise/--ground-effect/--wind-speed/--turbulence` sweep from §3.5 is still open (not yet run).
3. **M3 — IR ToF** (§4.1): downward ToF publishing, tracks altitude. (Optional multi-ranger for Phase 3.)
   **Status: DONE, gated live, unaffected by v3.** `configs/sensors/tof.yaml`, `eval_scripts/apply_tof_sensor.py`, `eval_scripts/tof_gate.py`. Gate result: rate ~29.9 Hz (floor 24 Hz), altitude error 0.8–1.8 cm at 0.3/0.6/1.0 m hover plateaus (tolerance ±12 cm). See §4.1 for live-run findings/fixes.
4. **M4 — UWB, REDEFINED by v3** (§5): split into **M4a** (v2 drone↔anchor range node, kept only as a frozen baseline control-arm run — not started) and **M4b** (v3 primary: inter-drone UWB PDoA gz-sim plugin, range+bearing+AoA-cone — not started, next real implementation task). *M4b gate: two drones exchange range+bearing on a ROS topic; correct range-only fallback outside the AoA cone.*
5. **M5 — Phase-1 exit gate** (§7): ±10 cm hold under noise+turbulence on the full loaded airframe.

M3 and M4 are independent and can be parallelised; M1→M2 are the critical path. M4b is now gated earlier than "M4" implied — v3 wants a basic inter-drone PDoA exchange working to exit its own Phase 0, so don't treat it as low-priority just because of the M-number.

---

## 10. Risks / open decisions

- **"IR sensor" meaning** — RESOLVED: IR ToF rangefinder (§4). Thermal camera deferred (§4.2).
- **Airframe choice** — brushed 2.1 likely fails T/W once radar+GAP9 are added (pucks retired, v3 — one less item pushing toward Brushless); the Brushless is Tier B but the mass model will force this decision with data (§3.3).
- **⚠ Placeholder masses** (radar, GAP9 shield, UWB PDoA module) — the physics is only as accurate as these; get a scale on the real parts once chosen (v3 defers all hardware purchase to Phase 11 — see roadmap). Until then, clearly label sim results as provisional. (Anchor pucks removed from this list — retired entirely, v3.)
- **UWB architecture superseded** — v2's drone↔anchor puck design is retained *only* as a frozen baseline control arm (§5.1–5.4, M4a); it is not the thing M4 is building toward anymore. Don't let new work quietly assume anchors are still the primary inter-drone sensing mechanism — see §5.5/M4b.
- **ROS↔gz bridging** — RESOLVED for ToF: `phase0_gate.sh` auto-launches `ros_gz_bridge` for `/cf_0/tof_down` when the workspace that provides it (`${ROS_GZ_WS:-$HOME/ros2_ws}`, sourced by `setup_env.sh`) is present; verified live at ~27–28 Hz. If that workspace isn't set up on a given machine, the script warns and continues with the gz-native topic only (same situation `/cf_0/{imu,baro,odom}` are already in).
- **Known repo nit (not this phase):** `RadarSensorSystem.cpp` has a hard-coded default `/home/ethan/...` mesh path (overridden by SDF `mesh_path`, so functional). Flagging per the path-portability rule; fix opportunistically.
- **MuJoCo — new dependency flagged by v3, not yet installed.** v3 lists MuJoCo (for the Phase 6 perching gate) as a direct Phase 0 checklist item ("Install and validate MuJoCo, confirm it can simulate at minimum a basic gripping/perching mechanism"). Out of scope for *this* Phase 1 plan (perching is a later phase), but tracked here since it's an environment-setup item the same team will hit soon — not addressed by this revision.
