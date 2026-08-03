# Phase 1 Implementation Plan — Physically-Accurate Airframe + IR & UWB Sensors

**Roadmap mapping:** This expands **Phase 1 steps 1, 3, 6** of `Simulation_Training_Optimization_Roadmap_v2.md` (mass/inertia model, sensor noise, `--ground-effect/--turbulence` flags) and pre-builds the sensor plumbing that **Phase 2** (EKF: IMU + radar Doppler + **downward ToF** + **UWB range** updates) consumes. It does **not** implement the EKF itself (that is Phase 2) — it produces the *physically accurate airframe* and the *IR + UWB measurement streams* the EKF will fuse.

**Prereq status:** Phase 0 is COMPLETE (SITL Crazyflie + `radarays_gz2` in `sim_worlds/phase0_tunnel_gate.sdf`, `/radar/points` @ ~10 Hz).

> **⚠ SUPERSEDED BY v3 (read before touching §5/UWB or M4):** `Simulation_Training_Optimization_Roadmap_v3_MOONSHOT.md` (2026-07-27) retires dropped **anchor pucks** entirely. UWB's role is an **inter-drone range+bearing (PDoA) mesh** gauged at a fixed entrance node. **M4a cancelled.** **Build authority for M4:** `.cursor/docs/M4_UWB_Relative_Positioning_Implementation_Plan.md` (Python analytic node, not the C++ plugin sketch in §5 below).

---

## Status at a glance (updated 2026-07-30)

| Milestone | Scope | Status | Gate / notes |
|---|---|---|---|
| **M1** | Mass / CoM / inertia model + thrust margin | ✅ **DONE** (v3-revised) | 33.8 g, T/W = 2.019 (floor 1.5) |
| **M2** | PID retune + hover hold on loaded mass | ✅ **DONE** | `hover_gate.py` PASS; mean \|Z err\| = 1.18 cm, RMS drift = 8.03 cm |
| **M2b** | Disturbance flags (`--sensor-noise`, wind, turbulence) | ❌ **NOT STARTED** | §3.5 — no sweep run yet |
| **M3a** | VL53L1x down-ToF (`gpu_lidar`) | ✅ **DONE** | `tof_gate.py` PASS; ~29.9 Hz, altitude error 0.8–1.8 cm |
| **M3b** | PMW3901 optical flow (Flow deck v2 only) | ✅ **DONE** | `flow_gate.py` PASS; ~100 Hz, dropout on smooth patch works |
| **M4** | Inter-drone UWB PDoA mesh (Python analytic node) | ✅ **DONE** | `uwb_gate.py` PASS 12/12; see §5. M4a **cancelled** |
| **M4-4** | Landed peers + mesh LOS control arm | ⏸ **NOT RUN** | Config-only; low priority |
| **M5** | Phase-1 exit gate (±10 cm hold under noise + turbulence) | ❌ **NOT STARTED** | Roadmap Ph1 exit; stricter than M2 |
| **§4.3** | Thermal / LWIR camera | ⏸ **DEFERRED** | Out of scope Phase 1 |
| **Multi-ranger** | 5-direction ToF beams | ⏸ **DISABLED** | Config stub only; optional for Phase 3 |

**Phase 1 progress:** airframe physics (M1–M2), Flow-deck sensors (M3a + M3b), and UWB PDoA mesh (M4) are complete and gated. **M5** exit gate and **M2b** disturbance flags remain.

---

## 0. Read this first — simulation fidelity boundaries

These are hard constraints from `AGENTS.md §5`. The plan is designed *around* them; do not let the implementation quietly cross them.

| Sensor | Sim fidelity | What we may claim | What we must NOT claim |
|---|---|---|---|
| **UWB PDoA ranging+bearing** (inter-drone mesh, v3 — §5.1) | High for range; angle fidelity only as good as the off-boresight/AoA model, which is placeholder until Phase 1's external-validation gate (v3 roadmap Phase 1 item 1) | Range accuracy, angle accuracy within a validated envelope, AoA-cone range-only fallback, NLOS/multipath dropout | Angle accuracy outside the externally-validated range; anything about a *specific* unbuilt PDoA module's real aperture until Phase 11 bench data exists |
| **IR ToF rangefinder** (downward/obstacle) | High (raycast geometry + noise) | Altitude/obstacle ranging under geometry | Material-specific reflectivity dropout (approximate only) |
| **Thermal / LWIR camera** (victim IR) | **Low — mocked** | *Out of scope this phase (see §4.3)* | ❌ That the sim validates thermal victim detection or **thermal-inertial odometry** (real-hardware-only per `AGENTS.md §5`) |
| **PMW3901 optical flow** (Flow deck) | High when analytic sim uses true velocity + noise; texture dropout must be modeled | Body-frame `(vx, vy)` ground-relative velocity on textured floor | ❌ That flow works on untextured/smooth floors without invalid flag; ❌ that PMW3901 measures range/altitude |

**Consequence for this plan:** default **Flow deck v2** = two sensors — **VL53L1x ToF** (M3a, ranging) and **PMW3901 optical flow** (M3b, horizontal velocity). **Both are built and gated (M3 complete).** UWB PDoA (M4) is **built and gated** — see §5. Thermal/LWIR is **deferred** (§4.3).

---

## 1. Objectives (definition of done)

| # | Objective | Status |
|---|---|---|
| 1 | Physically accurate drone (mass, CoM, inertia, PID, T/W) | ✅ M1 + M2 done |
| 2 | IR sensors in sim (Flow deck: ToF + optical flow) | ✅ M3a + M3b done |
| 3 | UWB sensor in sim (inter-drone PDoA mesh only; M4a cancelled) | ✅ M4 done |
| 4 | Sensor weights in mass model (config-driven) | ✅ Done |
| 5 | Config-driven, portable, MLflow-logged evals | ✅ Done for M1–M4 gates |

**Still open:** disturbance flags (§3.5), Phase-1 exit gate M5 (±10 cm under noise+turbulence). Thermal LWIR deferred (§4.3).

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

**Status: DONE (M1).** Live PASS at T/W = 2.019 on v3-revised 33.8 g payload.

### 3.4 PID retune for the loaded mass (roadmap Ph1.2)
- After the mass model is in, hover will be sluggish/unstable on the old gains. Re-tune via the CrazySim PID-tuning workflow; **save the gain set** to `configs/airframe/pid_gains_loaded.yaml` (this is what eventually flashes to hardware).
- Gains belong in `configs/`, not firmware C. Prefer parameter/config over editing firmware (`AGENTS.md §2`).

**Status: DONE (M2).** Two-stage Optuna cascade; `pid_gains_loaded.yaml` committed; `hover_gate.py` PASS on loaded mass.

### 3.5 Enable realistic disturbances (roadmap Ph1.6)
Run hover + waypoint sweeps with CrazySim's `--sensor-noise --ground-effect --wind-speed --turbulence`. These are launch flags; expose them through `phase0_gate.sh` pass-through args so evals can sweep them.

**Status: NOT STARTED.** Flags not yet wired through `phase0_gate.sh`; no disturbance sweep logged to MLflow.

---

## 4. Workstream C — Flow deck sensors (IR ToF + optical flow)

**Decision:** the default payload deck is **Bitcraze Flow deck v2** (`deck: flow_v2` in `configs/sensors/tof.yaml`). It carries **two separate downward sensors** on one PCB — do not conflate them:

| Chip | Type | What it measures | Sim in M3 | ROS topic (planned/built) |
|---|---|---|---|---|
| **STMicro VL53L1x** | IR time-of-flight laser rangefinder (940 nm) | Single-point range down (altitude over floor) | **M3a — DONE** (`apply_tof_sensor.py`, `gpu_lidar` single ray) | `/cf_<id>/tof_down` → `sensor_msgs/LaserScan` (1 range) |
| **PixArt PMW3901** | Optical-flow motion sensor (downward low-res camera + DSP) | Ground-relative horizontal velocity `(vx, vy)` in body frame by tracking floor texture | **M3b — DONE** (analytic node) | `/cf_<id>/flow` → `TwistWithCovarianceStamped`; `/cf_<id>/flow/pixels` |

The VL53L1x is **not** an optical-flow sensor — it fires a laser pulse and times the return. The PMW3901 is **not** a rangefinder — it watches how the floor pattern moves between frames. On real hardware both point down from the same deck; in sim they are separate injectors/nodes with separate noise models.

**Alternate decks (no PMW3901):**
- `zranger_v2` — VL53L1x down ToF only (1.3 g); skip M3b entirely.
- `multi_ranger` — 5× VL53L1x (F/B/L/R/up, 2.3 g); optional Phase-3 obstacle ranging; no optical flow.

Mass for Flow deck v2 (**1.6 g**, both chips) is already in `payload.yaml`. Both chips are simmed when `deck: flow_v2` (M3 complete).

### 4.1 M3a — VL53L1x IR ToF rangefinder (altitude)

- **Hardware:** STMicro **VL53L1x** on the Flow/Z-ranger/Multi-ranger deck — 940 nm IR laser time-of-flight, **one range value per frame**, 4 cm–4 m, ~30 Hz in our config (up to ~50 Hz on chip).
- **Sim mechanism:** Gazebo `gpu_lidar` with `samples: 1` in H & V (single ray = pencil beam). Pure geometry raycast — high fidelity per `AGENTS.md §5`.
  - **Downward** beam (required by Phase-2 EKF for altitude) → `/cf_<id>/tof_down`.
  - Optional **5-direction** Multi-ranger beams (disabled by default) → Phase-3 near-field obstacle check.
- **Injector:** `eval_scripts/apply_tof_sensor.py` (not `apply_payload.py`) — called from `phase0_gate.sh` after mass rewrite.
- **Requires** `gz-sim-sensors-system` (ogre2) at world level — already in `phase0_tunnel_gate.sdf` / `phase1_pid_tune.sdf`.
- **SDF sketch**, per beam:
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
- **Config:** `configs/sensors/tof.yaml` → `down`, `multi_ranger`, range/noise/rate params.
- **ROS 2 bridge:** `/cf_<id>/tof_down` → `sensor_msgs/LaserScan` via `ros_gz_bridge` (auto-launched from `phase0_gate.sh` when available).
- **Gate:** `eval_scripts/tof_gate.py` — publish rate + hover-plateau altitude tracking vs EKF `stateEstimate.z`.

**Status: DONE, gated live.** Gate result: ~29.9 Hz (floor 24 Hz); altitude error 0.8–1.8 cm at 0.3/0.6/1.0 m plateaus. Live-run gotchas (all fixed): invalid `<lidar><noise type=...>` attribute syntax (use child `<type>` element); `gz topic -f` never self-terminates; `gz topic -e --json-output` serializes `inf` as `"Infinity"` string; altitude gate uses cflib hover not `set_pose` teleport (free-fall during CLI latency).

### 4.2 M3b — PMW3901 downward optical flow (horizontal velocity)

**Meeting decision (2026-07-28):** extend M3 to sim the **second chip on the Flow deck** — the PixArt **PMW3901** optical-flow sensor — alongside the VL53L1x (M3a). This is **M3b**, not a separate phase; M3a's gate already passed independently so M3b can land without re-gating altitude.

**Hardware (real):** PMW3901 is a small **downward-facing optical-flow ASIC** (camera + motion DSP on one die). It compares consecutive floor-texture frames and outputs **integrated motion in pixels**, which the deck firmware converts to **ground-relative horizontal velocity** `(vx, vy)` in the drone body frame. It does **not** measure height — that is the VL53L1x's job. Typical specs (Tier B, tune from [PixArt PMW3901 datasheet](https://www.pixart.com/) / Bitcraze Flow deck docs):
- Update rate: ~100–200 Hz (firmware-dependent)
- Max measurable flow: ~7.4 m/s (saturates above)
- Works over **textured** surfaces; reports invalid / near-zero quality on smooth glass, glossy tile, or featureless concrete
- Height band: roughly **0.1–4 m** over floor (same band as down-ToF; too low = out of focus, too high = texture too small)

**What it is NOT:**
- **Not IR ToF** — no laser, no time-of-flight, no range output.
- **Not derived from ToF data** — you cannot compute optical flow from `/cf_<id>/tof_down`'s single range sample.
- **Not a point cloud** — two velocity scalars `(vx, vy)`, not a depth image.

**Built sim mechanism (M3b — DONE, 2026-07-28):**
- **Config:** `configs/sensors/optical_flow.yaml` (own file — not inside `tof.yaml`). Active only when `deck: flow_v2`; node refuses to start on `zranger_v2`.
- **Implementation:** `perception/flow_sim/flow_node.py` — analytic rclpy node using the Crazyflie firmware flow measurement equation (`mm_flow.c`), PMW3901 pixel noise, hand-authored surface texture map, ToF height scale from bridged `/cf_<id>/tof_down`. No SDF injection (`apply_flow_sensor.py` not used).
- **Launch wiring:** `phase0_gate.sh` `--no-flow` / `--flow-config`; bridges `/cf_<id>/odom` alongside ToF when flow enabled; launches node with `python3 -u`.
- **ROS topics:** `/cf_<id>/flow` → `geometry_msgs/TwistWithCovarianceStamped` (derived vx/vy + covariance sentinel when invalid); `/cf_<id>/flow/pixels` → driver-unit dpixel + quality; `/cf_<id>/flow/meta` → dt, σ_px, h_meas; `/cf_<id>/flow/debug_truth` → sim oracle (Tier A — estimator must never subscribe).
- **Downstream (Phase 2):** EKF velocity update; breadcrumb path integration `(v, Δψ)`; short-horizon hover hold when radar is sparse.

**Gate (`eval_scripts/flow_gate.py`):** live PASS on `phase1_pid_tune --no-radar --headless` — `/cf_0/flow` ~100 Hz; hover valid fraction 100%, hover RMS ~0.31 m/s (single-frame noise floor at `flow_std_px=2.0`); forward RMSE vs debug_truth ~0.32 m/s, mean vx bias ~−0.02 m/s; smooth-patch dropout valid fraction ~8%. See `.cursor/docs/M3b_Optical_Flow_Implementation_Plan.md`.

**Status: DONE, gated live.** Mass already counted in Flow deck v2 line item. M4 (UWB) also complete.

### 4.3 Thermal / LWIR camera — DEFERRED (out of scope Phase 1)
Not implemented now (you selected IR ToF). Recorded here so the option is documented if revisited:
- Gazebo Harmonic *can* render one (`<sensor type="thermal">` + `gz-sim-thermal-sensor-system` sensor plugin + `gz-sim-thermal-system` visual `<temperature>` on victim models like `rescue_randy`), but it is **mocked/non-validating** (no LWIR material physics, no thermal-inertial odometry validation — `AGENTS.md §5`).
- If added later: implement its **mass only** (~2.1 g Lepton 3.5 deck) for physics accuracy behind a `configs` flag, and **never** feed the thermal image into Phase-2+ logic.
- **For now: exclude the thermal deck mass from `payload.yaml`** unless the real airframe actually carries one.

---

## 5. Workstream D — UWB sensor (M4 — inter-drone PDoA mesh)

**Build authority:** `.cursor/docs/M4_UWB_Relative_Positioning_Implementation_Plan.md` — supersedes the C++ plugin sketch below wherever they disagree.

**There is no native Gazebo UWB sensor.** UWB is modelled **analytically** by a **Python `rclpy` node** (`perception/uwb_sim/uwb_node.py`), not a gz-sim System plugin — see M4 plan §1.1 reversal (compute budget does not justify C++).

**M4a cancelled.** The v2 drone↔anchor analytic node (`uwb_range_node.py`, `uwb.yaml`, `uwb_anchors.yaml`) and **physical anchor pucks** are **not** being implemented.

**Reference nodes without pucks:** if the mission needs fixed physical reference points later, **other drones** serve that purpose — e.g. a drone lands/holds and acts as a mesh peer (same PDoA range+bearing as any flying neighbor). The **entrance/base-station** node is a static world peer (same maths; pose surveyed).

### 5.1 Inter-drone UWB PDoA node (M4 — sole deliverable)

**Status: DONE, gated live (2026-07-30).** One swarm-wide node with airtime budget + round-robin scheduler (M4 plan §1.3). Offline: `uwb_model.py --selftest` (17/17 PASS). Live: `uwb_gate.py` **PASS 12/12** on `phase1_pid_tune -n 2 --spacing 2.0 --no-radar --headless --no-flow`.

**Implementation:**

| Component | Path | Role |
|---|---|---|
| Config | `configs/sensors/uwb_pdoa.yaml` | All Tier B knobs (noise, AoA cone, airtime, static peers) |
| Edge codec | `perception/uwb_sim/uwb_edges.py` | 48-byte `PointCloud2` record; `pack_edges()` / `unpack_edges()` |
| Pure model | `perception/uwb_sim/uwb_model.py` | Geometry, noise, LOS, scheduler — no `rclpy` |
| ROS wrapper | `perception/uwb_sim/uwb_node.py` | Subscribes `/cf_<id>/odom`; publishes edge topics |
| Exit gate | `eval_scripts/uwb_gate.py` | Checks A1–G (range, bidirectional identity, AoA cone, rel-pos, entrance node) |
| Launcher | `eval_scripts/phase0_gate.sh` | `-n N`, `--spacing`, `--no-uwb`, `--uwb-config`; kills stale `uwb_node` |

**ROS topics:**

| Topic | Purpose |
|---|---|
| `/cf_<id>/uwb/edges` | Per-drone measured edges (flying drones) |
| `/uwb/peer_<id>/edges` | Static peers (entrance node) — not `/uwb/1000/edges` (invalid ROS 2 name) |
| `/uwb/edges_all` | Aggregate mesh (gate + RViz) |
| `/uwb/edges_truth` | Sim oracle (Tier A) — estimator must **never** subscribe |

**Per drone pair, per scheduled exchange, emit:** range (m), azimuth (rad), elevation (rad) in the **observer's body frame**, plus validity flags. **Shared range noise** per TWR exchange (both observers see the same `r_meas`). Outside the **180° AoA cone**, emit **range-only** edges (bearing flagged invalid) — gate check C validates this via spawn asymmetry (drone 0 ahead, drone 1 behind).

**Live gate metrics (representative PASS run):**

| Check | Result |
|---|---|
| A1 edge rate | PASS (~10 Hz target after dedupe) |
| A2 range error mean | ~0.08 m |
| B bidirectional range identity | 100% |
| C bearing-valid fraction (leg A) | 100% |
| D azimuth RMS | ~8.5° |
| E relative-position error mean | ~0.38 m |
| F entrance node range error mean | ~0.08 m |
| G aggregate | **12/12 PASS** |

NLOS live behaviour is covered by offline selftest checks 8–10; not a separate live gate leg.

#### §1.5 deltas from this doc's original §5 sketch (resolved — do not revert)

| Original §5 sketch | Built implementation | Why |
|---|---|---|
| C++ gz-sim System plugin | **Python `rclpy` node** | M4 plan §1.1 — compute does not justify C++ |
| flat `update_rate_hz` | **airtime budget + neighbour cap + round-robin scheduler** | §1.3 — flat rate hides mesh scaling limit |
| separate M4a + M4b codebases | **one node, one model** | Range-only anchor = static peer with bearing disabled |
| three config files | **one `uwb_pdoa.yaml`** | Peer list + noise model in one place |
| antenna spacing "~4–5 cm" | **`antenna_spacing_m: 0.023` (≈ λ/2 at 6.5 GHz)** | >λ/2 is phase-ambiguous — physics error |
| NLOS range bias only | **NLOS also invalidates bearing by default** | Multipath AoA points at reflector, not peer |
| — | **antenna-delay bias sampled once at startup** | Persistent 10–30 cm offset if omitted |
| — | **`--selftest` (17 checks, no ROS/sim)** | Retires correctness risk before Gazebo |

#### Mid-level decision 8 — AoA cone width

**Target: 180°** as the practical maximum while maintaining usable bearing accuracy. A dual-antenna PDoA module on a small airframe is fundamentally limited by **front vs. back ambiguity** on the azimuth axis — widening the cone beyond ~180° reintroduces that ambiguity without extra motion or hardware.

> **Side note (360° azimuth):** full **360° bearing** is possible if the drone performs a **slight yaw twitch** — a small deliberate yaw motion and comparison of bearing readings before/after disambiguates whether the peer is in the forward vs. rear half-plane. Not in M4 scope; document as a future estimator/motion primitive if needed.

#### Future work (not M4 scope)

> **Swarm measurement reinforcement:** inter-drone **relative range and bearing** can **reinforce or correct local positioning drift** — e.g. if one drone's IMU/flow odometry drifts slightly, consistent range+bearing to neighbors (and mutual bearings) provide cross-checks that tighten the local state without requiring anchor pucks. Lands in Phase 2 mesh fusion / later swarm positioning work.

### 5.2 Cancelled — M4a drone↔anchor control arm (do not implement)

The following v2 design is **retained in git history / v3 roadmap references only** — **do not build**:

- `perception/uwb_sim/uwb_range_node.py`, `configs/sensors/uwb.yaml`, `configs/sensors/uwb_anchors.yaml`
- Dropped puck anchors, anchor↔anchor ranging, Loco-deck mass line

If v3 Phase 2 ever needs a range-only baseline comparison, that can be a one-off script — not a Phase 1 milestone.

### 5.3 M4 milestone breakdown (M4 plan §7.1)

| Sub-milestone | Scope | Status |
|---|---|---|
| **M4-0** | Multi-drone launch (`phase0_gate.sh -n N`) | ✅ DONE |
| **M4-1** | `uwb_model.py` + `uwb_edges.py` + `--selftest` | ✅ DONE (17/17) |
| **M4-2** | `uwb_node.py` live; edge topics + RViz mesh | ✅ DONE |
| **M4-3** | `uwb_gate.py` passes A–G | ✅ DONE (12/12) |
| **M4-4** | Landed peers + mesh LOS in cave world (control arm) | ⏸ NOT RUN — config-only, low priority |

---

## 6. Config files to create (all Tier B tunables here, not in code)

```
configs/
  airframe/
    payload.yaml            # base inertial + component list — DONE (v3-revised)
    pid_gains_loaded.yaml   # re-tuned gains for loaded mass — DONE (M2)
    pid_gains_stock.yaml    # stock baseline — DONE
    pid_tune.yaml           # Optuna search space — DONE
    thrust_margin.yaml      # T/W floor — DONE
  sensors/
    tof.yaml                # IR ToF (M3a) — DONE
    optical_flow.yaml       # PMW3901 flow (M3b) — DONE (separate file, not inside tof.yaml)
    uwb_pdoa.yaml           # M4 inter-drone PDoA params — DONE
    # M4a cancelled: no uwb.yaml, uwb_anchors.yaml, or uwb_range_node.py
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
  # thermal deck deferred (§4.3) — add only if the real airframe carries one
  - {name: gap9_compute_shield, mass_g: 6.0, pose_xyz_m: [0.0, 0.0,  0.008], enabled: false}   # PLACEHOLDER — measure
  - {name: mmwave_radar_module, mass_g: 4.5, pose_xyz_m: [0.02, 0.0, 0.000], enabled: false}   # PLACEHOLDER — measure
  # anchor_pucks component RETIRED in v3 — no longer part of the mission architecture, removed from the file entirely
```

---

## 7. Validation & gates (every run → MLflow: params, seed, metrics)

| Mass model applied | inspect generated SDF / `thrust_margin_check.py` | T/W ≥ floor — **DONE, PASS** |
| Thrust margin | `thrust_margin_check.py` | T/W ≥ 1.5 — **DONE, PASS (2.019)** |
| Loaded-mass hover (M2) | `hover_gate.py` | RMS drift + \|Z err\| within thresholds — **DONE, PASS** |
| IR ToF stream (M3a) | `tof_gate.py` | rate + altitude tracking — **DONE, PASS** |
| Optical flow stream (M3b) | `flow_gate.py` | ~100 Hz; texture valid; smooth-patch dropout — **DONE, PASS (2026-07-28)** |
| Disturbance sweep (M2b) | hover/waypoint under `--sensor-noise` etc. | **NOT STARTED** |
| UWB PDoA mesh edge (M4) | `uwb_gate.py` | **DONE, PASS 12/12** — range err mean ~0.08 m, bidir identity 100%, bearing-valid A 100%, az RMS ~8.5°, rel-pos err mean ~0.38 m, entrance range err mean ~0.08 m |
| **Phase-1 exit gate (M5)** | hover/waypoint sweep | **±10 cm hold under noise + turbulence — NOT STARTED** |

The Phase-1 exit gate is the roadmap's: stable loaded-mass flight under realistic noise **before** any radar/UWB fusion (that's Phase 2).

---

## 8. File-by-file change list

### Done ✅

| File | Milestone |
|---|---|
| `eval_scripts/apply_payload.py` | M1 — mass/CoM/inertia rewrite |
| `eval_scripts/thrust_margin_check.py` | M1 — T/W gate |
| `configs/airframe/payload.yaml` | M1 — v3-revised (no pucks, `uwb_pdoa_module`) |
| `configs/airframe/thrust_margin.yaml` | M1 |
| `configs/airframe/pid_gains_stock.yaml`, `pid_tune.yaml` | M2 |
| `configs/airframe/pid_gains_loaded.yaml` | M2 — Optuna output, live-validated |
| `eval_scripts/pid_gains.py`, `tune_pid.py`, `push_pid_gains.py` | M2 |
| `eval_scripts/hover_gate.py` | M2 exit gate |
| `sim_worlds/phase1_pid_tune.sdf` | M2 test world |
| `eval_scripts/apply_tof_sensor.py` | M3a — ToF injection |
| `eval_scripts/tof_gate.py` | M3a exit gate |
| `configs/sensors/tof.yaml` | M3a (+ deck selector for M3b) |
| `perception/flow_sim/flow_node.py` | M3b — analytic PMW3901 sim |
| `eval_scripts/flow_gate.py` | M3b exit gate |
| `configs/sensors/optical_flow.yaml` | M3b config |
| `eval_scripts/phase0_gate.sh` | M3a/M3b/M4 wiring (ToF, odom bridge, flow, `-n N`, UWB node launch) |
| `configs/sensors/uwb_pdoa.yaml` | M4 config |
| `perception/uwb_sim/uwb_edges.py` | M4 PointCloud2 edge codec |
| `perception/uwb_sim/uwb_model.py` | M4 pure model + `--selftest` |
| `perception/uwb_sim/uwb_node.py` | M4 rclpy wrapper |
| `eval_scripts/uwb_gate.py` | M4 exit gate |

### Not started ❌

| File | Milestone |
|---|---|
| M5 exit gate script (noise + turbulence sweep) | M5 |
| `phase0_gate.sh` pass-through for `--sensor-noise` / wind / turbulence | M2b / §3.5 |

### Partial / deferred ⏸

| Item | Notes |
|---|---|
| `configs/rviz/radar.rviz` | UWB PointCloud2 on `/uwb/edges_all` wired; ToF/flow displays partial |
| `configs/sensors/tof.yaml` → `multi_ranger.enabled` | Disabled; optional Phase 3 |
| Thermal deck mass + sim | Deferred §4.3 |
| README / `AGENTS.md §3` | Updated incrementally; M3b added 2026-07-28; M4 gated 2026-07-30 |

**Untouched:** the CrazySim submodule (`model.sdf.jinja` etc.) — all changes via launch-time injection + configs.
---

## 9. Suggested sequencing (milestones)

1. **M1 — Mass model** (§2, §3.1–3.3): `payload.yaml` + `apply_payload.py` inertial rewrite + thrust check. *Gate: SDF shows correct M/CoM/I; T/W reported.*
   **Status: DONE, revised for v3.** Original gate: 33.1 g total, T/W=2.062, PASS. **v3 revision (this session):** retired the `anchor_pucks` placeholder component and relabeled `uwb_loco_deck` → `uwb_pdoa_module` (mass bumped 3.3→4.0 g placeholder, pending a real PDoA part choice — see §2). Re-ran `apply_payload.py` + `thrust_margin_check.py` live against a fresh jinja-generated SDF: **33.8 g total, T/W=2.019, PASS.**
2. **M2 — PID retune + disturbance flags** (§3.4–3.5): stable hover on loaded mass. *Gate: hover holds.*
   **Status: DONE, run live.** Two-stage cascade Optuna search (Stage 1 = pid_rate+pid_attitude, Stage 2 = velCtlPid+posCtlPid), `configs/airframe/pid_gains_loaded.yaml` committed, `hover_gate.py` passing (RMS horizontal drift + mean |Z error| within thresholds). `tune_pid.py` needed a `SIGALRM` watchdog + consecutive-timeout abort added mid-run for `cf2` SITL crashes (see `AGENTS.md` §3/§4). **v3 revision (this session):** since M1's mass only moved 33.1→33.8 g (+0.7 g), re-ran `hover_gate.py` only (no re-tune) as a cheap sanity check against the updated mass model, against a freshly launched sim (existing gains still applicable at this small a mass delta — no need to burn another multi-hour Optuna run). **Result: PASS** — mean |Z error| = 1.18 cm (threshold 5 cm), RMS horizontal drift = 8.03 cm (threshold 10 cm), took off cleanly, no divergence. Existing `pid_gains_loaded.yaml` gains remain valid; no re-tune needed for the v3 mass revision. The `--sensor-noise/--ground-effect/--wind-speed/--turbulence` sweep from §3.5 is still open (not yet run).
3. **M3 — Flow deck sensors** (§4): **COMPLETE (2026-07-28).**
   - **M3a — IR ToF: DONE, gated live.** Gate: rate ~29.9 Hz, altitude error 0.8–1.8 cm at hover plateaus. See §4.1.
   - **M3b — PMW3901 optical flow: DONE, gated live.** Gate: ~100 Hz, smooth-patch dropout ~8% valid. See §4.2.
4. **M4 — UWB inter-drone PDoA mesh** (§5): **DONE, gated live (2026-07-30).** M4a (anchor pucks / analytic range node) **cancelled**. Built as Python analytic node per M4 plan (not C++ plugin). Sub-milestones: M4-0…M4-3 ✅; M4-4 (landed peers + mesh LOS) not run.
5. **M5 — Phase-1 exit gate** (§7): **NOT STARTED.** ±10 cm hold under noise+turbulence on the full loaded airframe.

**What's left for Phase 1:** M2b disturbance flags, M5 exit gate. M1→M2→M3→M4 critical path is done.

---

## 10. Risks / open decisions

- **"IR sensor" meaning** — RESOLVED: Flow deck = VL53L1x ToF (M3a, §4.1) + PMW3901 optical flow (M3b, §4.2) as separate chips. Thermal LWIR deferred (§4.3).
- **Airframe choice** — brushed 2.1 likely fails T/W once radar+GAP9 are added (pucks retired, v3 — one less item pushing toward Brushless); the Brushless is Tier B but the mass model will force this decision with data (§3.3).
- **⚠ Placeholder masses** (radar, GAP9 shield, UWB PDoA module) — the physics is only as accurate as these; get a scale on the real parts once chosen (v3 defers all hardware purchase to Phase 11 — see roadmap). Until then, clearly label sim results as provisional. (Anchor pucks removed from this list — retired entirely, v3.)
- **UWB architecture** — **RESOLVED (2026-07-30).** M4a cancelled (no pucks, no `uwb_range_node`). M4 built as inter-drone PDoA Python node (§5.1, M4 plan). Fixed reference points → **landed drones** as mesh peers or static `static_peers` in config. AoA default **180°** (decision 8). Antenna spacing corrected to **λ/2 (0.023 m)** — parent doc's "~4–5 cm" was phase-ambiguous. NLOS invalidates bearing by default. Airtime budget models mesh scaling honestly.
- **ROS↔gz bridging** — RESOLVED for ToF + flow: `phase0_gate.sh` auto-launches `ros_gz_bridge` for `/cf_0/tof_down` and `/cf_0/odom` (flow node) when the workspace that provides it (`${ROS_GZ_WS:-$HOME/ros2_ws}`, sourced by `setup_env.sh`) is present; ToF verified ~27–28 Hz, flow ~100 Hz. If that workspace isn't set up on a given machine, the script warns and continues with gz-native topics only.
- **Known repo nit (not this phase):** `RadarSensorSystem.cpp` has a hard-coded default `/home/ethan/...` mesh path (overridden by SDF `mesh_path`, so functional). Flagging per the path-portability rule; fix opportunistically.
- **MuJoCo — new dependency flagged by v3, not yet installed.** v3 lists MuJoCo (for the Phase 6 perching gate) as a direct Phase 0 checklist item ("Install and validate MuJoCo, confirm it can simulate at minimum a basic gripping/perching mechanism"). Out of scope for *this* Phase 1 plan (perching is a later phase), but tracked here since it's an environment-setup item the same team will hit soon — not addressed by this revision.

---

## 11. Downstream mission architecture (meeting 2026-07-28)

Phase 1 ends at sensor plumbing + stable loaded-mass flight. The meeting clarified what those sensors *feed* in later phases — captured in full in `Simulation_Training_Optimization_Roadmap_v3_MOONSHOT.md` (new "Meeting decisions" section). Summary for traceability:

| Meeting topic | Phase 1 status | Where it lands |
|---|---|---|
| Obstacle avoidance (mmWave + hard-coded rules) | Radar exists (`/radar/points`); avoidance logic **not built** | Phase 3 |
| Auction floor exploration (ToF grid) | Down-ToF **built** (M3); Multi-ranger **disabled**; auction logic **not built** | Phase 7 |
| Local / global / swarm positioning | ToF + flow **built** (M3); UWB PDoA **built & gated** (M4); estimator **not built** | Phase 2 |
| Path tracing `(v, Δψ)` breadcrumbs | **Not built** | Phase 2 log + Phase 7 RTH |
| Human detection — IR ToF | **Misnomer in meeting** — ToF = altitude/floor only | M3 done; no change |
| Human detection — thermal IR | Deferred (§4.3) | Phase 5/6 |
| Human detection — mmWave heartbeat | Radar plugin exists; vitals mode **not built** | Phase 5 |
| Human detection — audio | **Not built** | Phase 5/6 |
| Downward optical flow (PMW3901 on Flow deck) | M3a ToF **built**; M3b flow **built & gated** | Done (`§4.2`, `M3b_Optical_Flow_Implementation_Plan.md`) |

**One actionable Phase 1 follow-on (optional, not gated):** flip `multi_ranger.enabled: true` in `configs/sensors/tof.yaml` when Phase 3 obstacle avoidance wants near-field beams — requires updating `payload.yaml` mass to Multi-ranger deck (2.3 g) if enabled.
