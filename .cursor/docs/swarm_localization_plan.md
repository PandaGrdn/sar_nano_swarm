# P2 — Swarm Localization Implementation Plan
## RIO + UWB Range/Bearing Distributed Estimator (`sar_nano_swarm`)

**Audience:** implementing model/engineer. **Every design decision in this document is already made.** Do not re-litigate them. If you hit a case this document does not cover, pick the option that is *more physically realistic*, even if it makes results look worse, and write the choice in `.cursor/docs/P2_DEVIATIONS.md`.

> **Naming (read first).** An earlier draft called this "M5". **That label is already taken:** in `Simulation_Training_Optimization_Roadmap_v3_MOONSHOT.md` / `AGENTS.md §3`, `M5` is the *Phase-1* exit gate (±10 cm hover hold under noise + turbulence) — an unrelated single-drone milestone. This document is **roadmap Phase 2 — "Entrance-Gauged Relative-Pose Mesh"** (roadmap §PHASE 2). Milestones here are numbered `P2-1 … P2-8`. Never write `M5` in a filename, MLflow run name, or commit message for this work.

**Prerequisite state (already done, Phase-1 M4 — verified against the repo):**

`perception/uwb_sim/uwb_node.py` is a single rclpy process for the whole swarm. It subscribes to `/cf_<id>/odom` (`nav_msgs/msg/Odometry`, bridged from Gazebo by `eval_scripts/phase0_gate.sh`) and publishes simulated UWB edges as `sensor_msgs/msg/PointCloud2` on:

| Topic | Contents |
|---|---|
| `/cf_<id>/uwb/edges` | edges observed by drone `<id>` (`<id> < num_drones`) |
| `/uwb/peer_<id>/edges` | edges observed by a static peer (ROS 2 rejects topic tokens starting with a digit, hence the `peer_` prefix) |
| `/uwb/edges_all` | aggregate of all edges (`publish_aggregate: true`) |
| `/uwb/edges_truth` | **sim oracle — AGENTS.md Tier A. Never subscribe from `perception/swarm_loc/`.** |

**Edge row schema — this is fixed; do not redefine it.** `perception/uwb_sim/uwb_edges.py` defines `EDGE_DTYPE` (12 fields, `itemsize == 48`, little-endian) and `pack_edges()` / `unpack_edges(msg) -> np.ndarray`:

```python
("x","<f4"), ("y","<f4"), ("z","<f4"),          # body-frame Cartesian bearing vector (see below)
("observer_id","<u4"), ("peer_id","<u4"),
("range_m","<f4"), ("azimuth_rad","<f4"), ("elevation_rad","<f4"),
("sigma_range_m","<f4"), ("sigma_az_rad","<f4"), ("sigma_el_rad","<f4"),
("flags","<u4")
```

Flag bits (also in `uwb_edges.py`): `FLAG_RANGE_VALID=0x01`, `FLAG_BEARING_VALID=0x02`, `FLAG_LOS=0x04`, `FLAG_IN_AOA_CONE=0x08`, `FLAG_PEER_IS_SURVEYED=0x10`, `FLAG_PEER_IS_STATIC=0x20`.

Three facts about this schema that change what you have to write:

1. **`x, y, z` already are `range·[cos el·cos az, cos el·sin az, sin el]`** in the observer's body frame (`uwb_model.bearing_xyz()`). §4.3(a) does **not** require you to recompute them from `range/az/el` — read them directly. You still need the spherical→Cartesian Jacobian for the *covariance*.
2. **Invalid fields are `NaN`, not zero and not absent.** When bearing is unavailable, `x, y, z, azimuth_rad, elevation_rad, sigma_az_rad, sigma_el_rad` are all `NaN`. When only azimuth is available (`n_antennas == 2`), `z`, `elevation_rad`, `sigma_el_rad` are `NaN` while `x, y` are computed with `el = 0`. Gate on the **flag bits**, then assert with `np.isfinite`, never on `== 0`.
3. **`sigma_range_m`, `sigma_az_rad`, `sigma_el_rad` are per-edge**, already reflecting off-boresight degradation and NLOS inflation. Use them; do not substitute a constant from config.

The current `configs/sensors/uwb_pdoa.yaml` runs `n_antennas: 3` (azimuth **and** elevation), `boresight_axis: [1,0,0]` body-FLU, `aoa_fov_deg: 90.0` (i.e. a ±45° bearing cone with a large rear/side blind region — **not** the 180° the roadmap prose mentions as a future default), `antenna_delay_bias_sigma_m: 0.05`, `max_neighbors_per_drone: 6`, `max_exchanges_per_s: 400`.

**What Phase 2 adds:** the estimator that consumes those edges plus (stubbed) RIO odometry and produces each drone's corrected global pose.

---

## 0. Scope

**In scope:** per-drone distributed state estimator, inter-drone state broadcast, entrance-anchor gauge, reciprocal-bearing handling, range-only fallback, an offline centralized reference solver, evaluation harness and ablations.

**Out of scope (do not build):** mapping, frontier exploration, obstacle avoidance, victim detection, loop closure / place recognition, GAP9 porting, real hardware. Do not add these even if they seem helpful.

**Hard rule:** no node in `perception/swarm_loc/` may subscribe to a truth source. In this repo the truth sources are, exactly:

- `/uwb/edges_truth` — noise-free UWB oracle
- **`/cf_<id>/odom`** — this is **Gazebo ground truth**, not an onboard estimate. It is the single most likely accidental violation, because every other sim node in the repo subscribes to it.
- `/cf_<id>/flow/debug_truth` — noise-free body velocity oracle
- `configs/sensors/uwb_pdoa.yaml` — the file the *simulator* uses to generate measurements, including `static_peers` positions. The estimator reads its own `configs/estimation/swarm_loc.yaml` and nothing else.

Truth is used **only** inside `eval_scripts/` and inside the RIO stub's internal error-injection (§3.2), never read by the estimator itself. Violating this invalidates every result the project will publish.

---

## 1. Locked design decisions

| # | Decision | Chosen | Rationale (do not revisit) |
|---|---|---|---|
| D1 | Estimator topology | **One estimator node per drone**, separate process, communicating only via ROS topics | Forces honest message passing; no shared memory shortcuts; matches deployment |
| D2 | Filter type | **Error-state EKF** per drone, 9-state | Tractable on nano-hardware; well understood; avoids particle-filter cost |
| D3 | State vector | `x_i = [p (3), v (3), ψ (1), b_ψ (1), s (1)]` — global position, global velocity, yaw, yaw-rate bias, RIO scale factor | Yaw is the poison (§4.1); scale error is a real RIO failure mode |
| D4 | Roll/pitch | **Not estimated.** Taken from RIO/IMU as known, with noise | Gravity makes roll/pitch observable and driftless; only yaw drifts |
| D5 | RIO input | Consumed as **incremental delta-pose** (body-frame), not absolute position | Absolute RIO position would double-integrate drift into the filter |
| D6 | Neighbor state fusion | **Covariance Intersection (CI)** for any update using a neighbor's broadcast state | Naive EKF fusion double-counts shared information and produces overconfident, diverging estimates. Non-negotiable. |
| D7 | Bearing usage | When azimuth+elevation present: use **full 3D relative-position measurement** (range+az+el → Cartesian). Do not use `d·d'` on that link | Range+bearing fully determines relative position; `d·d'` is redundant and strictly worse |
| D8 | Reciprocal bearing | If either side of a pair has bearing, **both** sides use it (peer rebroadcasts the measurement). `d·d'` only when *neither* side has bearing | Corridor FOV asymmetry means one side usually sees the other |
| D9 | Range-only fallback | Implement as **two EKF measurement models** (range, and range-rate), not as a port of Guo et al.'s closed-form consensus solution | Guo's closed form is tied to their 2D consensus architecture; the EKF form is equivalent in information content, 3D-native, and far less error-prone to implement. Cite Guo for observability rationale only. |
| D10 | Range-rate `d'` | Sliding-window **linear regression over the last N range samples** (N from config, default 5), not raw finite difference | Finite differencing amplifies UWB noise unusably |
| D11 | Mutual-yaw constraint | Reciprocal bearing pairs generate an explicit **relative-yaw measurement** | This is the only magnetometer-free mechanism that makes yaw observable. Core to the research claim. |
| D12 | Global gauge | The **entrance node** (M4 device id `1000`, `type: entrance`) is the gauge, covariance fixed at ~0 (config `entrance.sigma_m`, default 0.01). Identify its edges by the `FLAG_PEER_IS_SURVEYED` bit, **not** by hard-coding the id. Its surveyed position is a constant in the estimator's own config | Without it the swarm is internally consistent but globally free (gauge freedom). ⚠ It is **not at the origin** — `uwb_pdoa.yaml` places it at `[-2.0, 0.0, 0.30]`. See §4.1 |
| D13 | Initialization | Drones launch from a **surveyed launch line**, not from above the entrance. `phase0_gate.sh` spawns drone `i` at `(spawn_x + i·spacing, spawn_y, 0.5)`; at arm time each drone's state is initialized to its own surveyed launch position with small covariance. These four numbers live in the estimator config (`launch:` block, §3.3) and **must be kept equal to the launcher's `-x`, `-y`, `--spacing`** | Establishes a shared frame at t=0 without changing the launcher and without reading any truth topic. This is an initial condition, not a substitute for ongoing correction |
| D14 | Yaw initialization | All drones initialized to a **common yaw reference** at launch, with configurable initial yaw uncertainty (default 5°) | Do not assume magnetometer. Common launch heading is the realistic assumption |
| D15 | Reference solver | An **offline centralized batch least-squares** solver (`scipy.optimize.least_squares`, sparse Jacobian) run post-hoc in `eval_scripts/` | Gives the "best achievable given these measurements" upper bound to compare the distributed filter against. Not GTSAM — avoid the build dependency |
| D16 | Outlier handling | **Chi-squared (NIS) gate** on every measurement before update; rejected measurements logged, never silently dropped | NLOS/multipath outliers will otherwise corrupt the graph |
| D17 | Comms model | State broadcast at configurable rate with configurable **latency and packet-loss**; defaults 10 Hz, 20 ms, 2% loss | Perfect comms is a fake result |
| D18 | Clock sync | Assume common clock (sim Tier A); every message carries a timestamp and the filter **buffers and applies measurements in timestamp order** | Clock drift is deferred, but the plumbing must exist. Document as a limitation |
| D19 | Antenna-delay bias | Estimator must **not** be given the per-device bias; it must survive it as unmodeled error | M4 deliberately excluded it from reported σ. Keep it honest |
| D20 | Landed peers | `type: landed` peers are treated as **normal drones whose velocity is zero**, with their own uncertainty — *not* as zero-uncertainty anchors | A landed drone's position is only as good as its estimate when it landed. Only the entrance node is ground truth |

### 1.1 Precedence over the roadmap

**Where this document and `Simulation_Training_Optimization_Roadmap_v3_MOONSHOT.md` disagree, this document wins.** The roadmap's Phase 2 section is a scoping sketch written before the M4 sensor model existed; D1–D20 above are the decisions actually being implemented. Do not "reconcile" the two by changing anything here, and do not stop to ask which is right. Note the divergence in `.cursor/docs/P2_DEVIATIONS.md` and keep going.

The two divergences that already exist, so you recognise them rather than treating them as mistakes:

- Roadmap §PHASE 2 item 6 says "implement centralized graph optimization first … defer distributed implementation." **D1 supersedes it**: the distributed per-drone filter is built first, and the centralized solver is kept as the offline reference bound (D15, P2-7). The message-budget number the roadmap wanted comes out of the comms-volume metric in §6.1 anyway.
- Roadmap §PHASE 2 items 1 and 8 want **altitude factors** (baro + `/cf_<id>/tof_down`) and **optical flow** (`/cf_<id>/flow`) fused into the local estimator. Both sim nodes already exist and are gated (`tof_gate.py`, `flow_gate.py`). **They are deliberately out of scope here** — in this milestone all odometry enters through the single `RioDelta` interface (D5), so the real RIO front end can be swapped in without touching the filter. Fusing flow/ToF/baro is a later milestone, not an omission.

---

## 2. Repository layout (new files only)

```
configs/estimation/swarm_loc.yaml          # all P2 tunables (§3.3). configs/estimation/ does not exist yet — create it
perception/swarm_loc/state.py              # state vector, covariance, error-state helpers
perception/swarm_loc/rio_stub.py           # RIO interface + drift-injecting stub (§3.2)
perception/swarm_loc/measurements.py       # measurement models + Jacobians (§4.3)
perception/swarm_loc/ci_fusion.py          # covariance intersection (§4.4)
perception/swarm_loc/ekf.py                # pure math, no rclpy, --selftest
perception/swarm_loc/swarm_loc_node.py     # rclpy wrapper, one instance per drone
perception/swarm_loc/swarm_msgs.py         # broadcast packing/unpacking (§3.1)
eval_scripts/central_reference.py          # offline batch solver (D15)
eval_scripts/swarm_loc_gate.py             # P2 exit gate
eval_scripts/run_ablations.py              # ablation sweep driver (§6.2)
.cursor/docs/P2_DEVIATIONS.md              # any deviation from this plan, with reason
```

### 2.1 Conventions — copy these from M4, they are not negotiable

**No `__init__.py`, no package-relative imports.** `perception/uwb_sim/` has no `__init__.py`; every module is run directly as a script (`python3 perception/swarm_loc/ekf.py --selftest`), which makes `from .state import …` fail. Bootstrap the path exactly as `uwb_node.py` does and use flat absolute imports:

```python
_REPO_ROOT = Path(__file__).resolve().parents[2]
for _p in ("perception/swarm_loc", "perception/uwb_sim"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

from state import SwarmState            # noqa: E402
from uwb_edges import unpack_edges, FLAG_BEARING_VALID, FLAG_PEER_IS_SURVEYED  # noqa: E402
```

**Reuse, do not reimplement.** These already exist and are selftested:

| Symbol | Module | Use for |
|---|---|---|
| `unpack_edges`, `FLAG_*` | `perception/uwb_sim/uwb_edges.py` | decoding every UWB edge message |
| `quat_to_rot_matrix(q)` | `perception/uwb_sim/uwb_model.py` | quaternion `(x,y,z,w)` → **body→world** rotation |
| `yaw_to_rot_matrix(yaw)` | `perception/uwb_sim/uwb_model.py` | `Rz(ψ)` |
| `world_to_body(R_bw, d_w)` | `perception/uwb_sim/uwb_model.py` | `R_bw.T @ d_w` — the doc's `R_iᵀ(p_j−p_i)` |
| `load_gains`, `apply_gains`, `reset_estimator`, `reset_pose` | `eval_scripts/pid_gains.py` | any gate script that flies drones via cflib |

**Selftest contract.** Pure-math modules import **no `rclpy`** and expose `--selftest`; ROS wrappers contain no math. Match `uwb_model.run_selftest()` exactly: a nested `check(name, cond, detail="")` closure printing `[selftest] PASS <name>` / `[selftest] FAIL <name>: <detail>`, a final `[selftest] ALL PASS` or `[selftest] FAILED`, and `sys.exit(0 if ok else 1)`.

**MLflow.** `mlflow.set_tracking_uri("sqlite:///mlflow.db")` (relative — run from repo root), experiment from `cfg.get("mlflow_experiment", "phase2_swarm_loc")`, run name = the script's own name (`"swarm_loc_gate"`, `"central_reference"`, `"run_ablations"`). Wrap the whole MLflow block in `try/except` and print `MLflow skipped: …` on failure, like `uwb_gate.py` does — see the `mlflow.db` hang pitfall in §9.

**Config loading.** No shared loader exists. Copy `resolve_config_path()` + `load_config()` verbatim from `uwb_node.py` (absolute passes through; relative resolves against `$SAR_NANO_SWARM_ROOT`, falling back to the repo root).

---

## 3. Interfaces and data contracts

### 3.1 Inter-drone broadcast messages

Two topics, both `sensor_msgs/msg/PointCloud2`, both published at `broadcast_rate_hz`. Do not introduce custom `.msg` files (the repo has no ROS package/`CMakeLists` for message generation, and adding one is out of scope).

**You cannot call `pack_edges()` for these.** It is hard-wired to the 12-field, 48-byte `EDGE_DTYPE`. **Mirror its pattern** in `swarm_msgs.py`: a module-level `np.dtype`, a matching `PointField` list, `pack_*` / `unpack_*` functions, and an `assert DTYPE.itemsize == <N>` at import. Keep `height=1`, `width=len(rows)`, `is_bigendian=False`, `is_dense=False`, `point_step=itemsize`, `row_step=itemsize*width`, and the timestamp in `header.stamp` **as well as** in the row (the row copy is what survives the latency buffer).

**(1) `/cf_<id>/swarm_loc/broadcast`** — exactly one row per message, `STATE_DTYPE`:

| Field | numpy dtype | Note |
|---|---|---|
| `stamp` | `<f8` | seconds; the estimate's valid time, not the publish time |
| `drone_id` | `<u4` | (`u4` not `u16` — `PointField` has no 16-bit type) |
| `p_x, p_y, p_z` | `<f4` ×3 | global position estimate |
| `v_x, v_y, v_z` | `<f4` ×3 | global velocity estimate |
| `psi` | `<f4` | global yaw estimate (rad) |
| `cov_p_0 … cov_p_5` | `<f4` ×6 | upper triangle of the 3×3 position covariance, row-major: `(00,01,02,11,12,22)` |
| `cov_psi` | `<f4` | yaw variance |
| `roll, pitch` | `<f4` ×2 | from IMU, treated as known |
| `seq` | `<u4` | monotonic, for loss detection |
| `status` | `<u4` | `0 = OK`, `1 = DIVERGED` (§4.5) |
| `n_bearing_edges` | `<u4` | diagnostics |

**(2) `/cf_<id>/swarm_loc/bearing_rebroadcast`** — zero or more rows per message, `BEARING_DTYPE`. This is D8: every bearing measurement this drone made in the last broadcast period, so the peer can use it instead of its own `d·d'` fallback.

| Field | numpy dtype |
|---|---|
| `stamp` | `<f8` |
| `observer_id`, `peer_id` | `<u4` ×2 |
| `range_m`, `azimuth_rad`, `elevation_rad` | `<f4` ×3 |
| `sigma_range_m`, `sigma_az_rad`, `sigma_el_rad` | `<f4` ×3 |
| `psi_observer` | `<f4` — the observer's yaw estimate at `stamp`, needed for the mutual-yaw residual (§4.3d) |
| `roll_observer`, `pitch_observer` | `<f4` ×2 |

Copy `range/az/el` and the three sigmas **straight from the M4 edge row** — do not re-derive or re-noise them. Rows whose `FLAG_BEARING_VALID` bit is clear are simply not rebroadcast.

Each drone subscribes to `/cf_<j>/swarm_loc/broadcast` and `/cf_<j>/swarm_loc/bearing_rebroadcast` for every `j != i` up to `comms.max_neighbors`.

### 3.2 RIO interface (NOT YET IMPLEMENTED — stub now, swap later)

**Critical:** the real radar-inertial odometry front end does not exist yet and will change. Define the interface now so the estimator never needs modification when RIO lands.

```python
# perception/swarm_loc/rio_stub.py
@dataclass
class RioDelta:
    stamp: float
    dt: float
    delta_p_body: np.ndarray   # (3,) body-frame position increment
    delta_psi: float           # yaw increment (rad)
    roll: float                # absolute, from IMU
    pitch: float               # absolute, from IMU
    cov: np.ndarray            # (5,5) covariance of [delta_p_body(3), delta_psi, scale]
    valid: bool                # False when radar returns too sparse to solve
```

Publish on `/cf_<id>/rio/delta`. The estimator subscribes to this and **nothing else** for odometry.

Wire format for `/cf_<id>/rio/delta`: same rule as §3.1 — one `PointCloud2` row, `RIO_DTYPE` in `swarm_msgs.py`, fields `stamp <f8`, `dt <f4`, `dp_x/dp_y/dp_z <f4`, `dpsi <f4`, `roll <f4`, `pitch <f4`, `cov_0 … cov_14 <f4` (upper triangle of the 5×5, row-major), `valid <u4`.

**The stub** (`rio.source: stub`) is a separate rclpy process, `rio_stub.py`, run **one per drone** with `--cf-id <i>`. It subscribes to **`/cf_<i>/odom`** (`nav_msgs/msg/Odometry` — this is the Gazebo truth pose, bridged by `phase0_gate.sh`), differences consecutive poses to get the true body-frame delta, takes `roll`/`pitch` from the odom quaternion via `quat_to_rot_matrix`, then corrupts the delta with:
- velocity random walk (bias on `delta_p_body`, config `rio.vel_bias_walk`)
- yaw random walk (config `rio.yaw_walk_deg_per_min`, default 3.0)
- multiplicative scale error (config `rio.scale_error`, default 1.02)
- white noise per axis (config `rio.sigma_p`, `rio.sigma_psi_deg`)
- `valid=False` bursts at `rio.dropout_rate` for `rio.dropout_duration_s` (radar sparsity)

Seed the stub's RNG from `seed` + `cf_id` so the per-drone drift realizations differ but the run is reproducible, matching `UwbModel.from_config`'s seeding discipline.

Defaults must be **pessimistic**, matching published RIO drift (~2%/distance) rather than optimistic. `rio_stub.py` is the only file in `perception/swarm_loc/` allowed to touch truth; put the same `⚠ AGENTS.md §1 Tier A` banner at the top of the file that `uwb_node.py` carries, and refuse to start unless `rio.source: stub` — so a future real RIO cannot silently fall back to it.

Two launcher facts you must handle:

- `phase0_gate.sh` only adds the `/cf_<i>/odom` bridge mapping when flow or UWB is enabled. The stub needs it unconditionally, so include odom in the bridge condition when you add the `--swarm-loc` flag (§5, P2-5).
- The odom stamp is **sim time**, and sim time diverges from ROS wall time in this repo. Compute `dt` from consecutive odom header stamps, and judge staleness against wall time — the mistake that broke the optical-flow node (§9).

### 3.3 Config: `configs/estimation/swarm_loc.yaml`

Every number the estimator uses lives here. No magic numbers in code. Follow the house YAML style seen in `configs/sensors/uwb_pdoa.yaml`: a `#` header block naming the phase and the AGENTS.md fidelity tier, `# ── section ──` dividers, units baked into key names, ≤3 levels of nesting. This is the **complete** required key set — every key referenced anywhere in this document appears below:

```yaml
# Swarm relative localization estimator — roadmap Phase 2.
# See .cursor/docs/swarm_localization_plan.md
# ⚠ AGENTS.md §1 Tier A: the estimator reads THIS file only. It must never read
# configs/sensors/uwb_pdoa.yaml (that is how the SIMULATOR generates measurements).

estimator:
  rate_hz: 50                      # filter propagation rate
  state_init_sigma_p_m: 0.05
  state_init_sigma_v_mps: 0.05
  state_init_sigma_psi_deg: 5.0    # D14
  yaw_bias_walk_sigma: 1.0e-4
  scale_init: 1.0
  scale_init_sigma: 0.02
  cov_floor_p_m: 0.01              # prevents over-confidence collapse
  max_cov_p_m: 50.0                # divergence detector threshold

# ── launch geometry (D13) ─────────────────────────────────────────────────────
# MUST mirror phase0_gate.sh -x / -y / --spacing. Surveyed deployment geometry,
# not a truth read: drone i initializes at (spawn_x0_m + i*spacing_m, spawn_y_m, spawn_z_m).
launch:
  spawn_x0_m: 0.0
  spawn_y_m: 0.0
  spawn_z_m: 0.5
  spacing_m: 1.5
  init_yaw_deg: 0.0                # D14 — common launch heading, no magnetometer

# ── entrance gauge (D12) ──────────────────────────────────────────────────────
entrance:
  position_xyz_m: [-2.0, 0.0, 0.30]   # surveyed constant; matches the M4 entrance peer
  sigma_m: 0.01

measurements:
  use_bearing: true
  use_reciprocal_bearing: true     # D8
  use_range_only_fallback: true    # D9
  use_range_rate: true
  use_mutual_yaw: true             # D11
  range_rate_window: 5             # D10 — samples, NOT a fixed dt (§4.3c)
  nis_gate_chi2_p: 0.99            # D16
  mutual_yaw_max_dt_s: 0.2         # D11 / §4.3d pairing window
  max_measurement_age_s: 0.5       # D18 / §4.5 — discard older than this

comms:
  broadcast_rate_hz: 10            # D17
  latency_ms: 20
  packet_loss: 0.02
  max_neighbors: 6                 # keep equal to uwb_pdoa.yaml max_neighbors_per_drone;
                                   # a larger value here buys nothing — M4 will not
                                   # produce edges to peers outside its own k-neighbour graph

fusion:
  method: covariance_intersection  # D6 — do not change
  ci_omega_search: fast            # 'fast' = closed-form trace-min; 'grid' = 21-point sweep

rio:
  source: stub                     # stub | real
  vel_bias_walk: 1.0e-3
  yaw_walk_deg_per_min: 3.0
  scale_error: 1.02
  sigma_p: 0.02
  sigma_psi_deg: 0.5
  dropout_rate: 0.02
  dropout_duration_s: 0.5
  dropout_q_scale: 25.0            # §4.2 — Q inflation while valid == False

ablation:
  disable_uwb: false
  disable_bearing: false
  disable_entrance: false
  disable_rio: false
  uwb_noise_scale: 1.0             # §6.2 — 2.0 for the pessimistic arm

seed: 0
mlflow_experiment: "phase2_swarm_loc"
```

**The 2× noise arm (§6.2) is produced by the simulator, not the estimator.** `ablation.uwb_noise_scale` is a *label* the estimator logs so runs are identifiable; the actual doubling is done by passing `phase0_gate.sh --uwb-config` a copy of `uwb_pdoa.yaml` with `sigma_range_los_m`, `sigma_boresight_deg`, and `angle_sigma_slope_deg_per_deg` doubled. `run_ablations.py` generates that copy into `/tmp`; do not edit the tracked config.

---

## 4. Math specification

Implement exactly this. Do not substitute alternative formulations.

### 4.1 Frames and conventions

- Global frame `G`: **the Gazebo world frame** — right-handed, z-up, and *not* centred on the entrance node. Do not invent an entrance-centred frame: `/cf_<id>/odom` (the only truth the eval scripts have) is expressed in the world frame, so an offset frame would silently corrupt every ATE number. The entrance node sits at `entrance.position_xyz_m = [-2.0, 0.0, 0.30]` in `G`; it gauges the mesh by being a *known point*, which it can be anywhere.
- Body frame `B_i`: drone `i`'s frame, FLU (x forward, y left, z up) — the same convention the UWB `boresight_axis: [1,0,0]` uses. Rotation `R_i = Rz(ψ_i)·Ry(pitch_i)·Rx(roll_i)` maps **body→world**, matching `uwb_model.quat_to_rot_matrix`. Consequently `R_iᵀ` maps world→body and is exactly `uwb_model.world_to_body(R_i, ·)`. Getting this transposed is the single most common way to make every Jacobian in §4.3 subtly wrong; the P2-2 numerical-Jacobian gate is what catches it.
- Roll/pitch are inputs (D4). **Only `ψ` is estimated**, because only yaw drifts without an absolute reference. A yaw error of `δψ` rotates every bearing measurement by `δψ`, producing lateral error `≈ range·δψ` — this is why D11 exists.

### 4.2 Propagation (at `rate_hz`, driven by `RioDelta`)

```
p⁺ = p + s · R(ψ, roll, pitch) · delta_p_body
ψ⁺ = ψ + delta_psi - b_ψ · dt
v⁺ = s · R · delta_p_body / dt
s⁺ = s                    # random walk, driven by process noise only
b_ψ⁺ = b_ψ                # random walk
```

Covariance: `P⁺ = F P Fᵀ + G Q Gᵀ`, with `F` the Jacobian of the above w.r.t. `[p, v, ψ, b_ψ, s]` and `Q` built from `RioDelta.cov` plus the random-walk terms from config. Derive `F` analytically; the only non-trivial block is `∂p⁺/∂ψ = s · (∂R/∂ψ) · delta_p_body`.

If `RioDelta.valid == False`: propagate with **inflated** `Q` (multiply by `rio.dropout_q_scale`, default 25) and hold velocity — do not skip propagation.

### 4.3 Measurement models

Let `i` = self, `j` = neighbor. `p̂_j`, `P_j` come from the neighbor's broadcast.

**(a) Full relative position (bearing available — D7).** Applies when the edge row has `FLAG_BEARING_VALID` set **and** `np.isfinite(row["z"])` (elevation present). The body-frame Cartesian reading is **already in the edge row** — take `z_body = [row["x"], row["y"], row["z"]]`. It equals `[d·cos(el)·cos(az), d·cos(el)·sin(az), d·sin(el)]` by construction (`uwb_model.bearing_xyz`); recomputing it from `range_m/azimuth_rad/elevation_rad` is redundant but harmless if you prefer to assert the two agree in the selftest.

If `FLAG_BEARING_VALID` is set but `z` is `NaN` (azimuth-only, `n_antennas == 2`), **do not** use this model — `x, y` were computed with `el = 0` and are wrong in 3D. Fall back to model (b) plus a scalar azimuth measurement `h = atan2(·)` if you implement one; otherwise (b) alone. The shipped config has `n_antennas: 3`, so this branch should be rare — count it as a metric (§6.1) rather than assuming it never fires.

Measurement model: `h = R_iᵀ (p_j - p_i)`, residual `r = z_body - h`. Jacobians:

```
∂h/∂p_i = -R_iᵀ
∂h/∂p_j = +R_iᵀ
∂h/∂ψ_i = (∂R_iᵀ/∂ψ)(p_j - p_i)
```

Measurement covariance: propagate the per-edge `(sigma_range_m, sigma_az_rad, sigma_el_rad)` from the M4 edge row through the spherical→Cartesian Jacobian, `R_cart = J diag(σ_d², σ_az², σ_el²) Jᵀ`. **Do not** use an isotropic covariance and **do not** substitute `sigma_boresight_deg` from config — the per-edge sigma already carries the off-boresight growth (`piecewise_linear`, ~4.8° + 0.165°/° of off-boresight angle) and the ×3 NLOS inflation. Bearing error produces a covariance ellipsoid that is long in the transverse directions and short in range; faking it isotropic is a fidelity failure.

**(b) Range-only (D9, no bearing on either side).** `h = ‖p_j - p_i‖`, `∂h/∂p_i = -(p_j-p_i)ᵀ/‖·‖`. Covariance `sigma_range_m²` from the edge row. Note that `range_m` still carries the unmodelled per-device antenna-delay bias (D19, `antenna_delay_bias_sigma_m: 0.05` in M4 — a pair bias of up to ~±0.1 m that is **not** in `sigma_range_m`). This is deliberate; do not add a bias state to absorb it.

**(c) Range-rate (D9/D10).** `h = (p_j - p_i)·(v_j - v_i) / ‖p_j - p_i‖`, computed `d'` from the regression window. Note in code comments: this constrains only the component along relative velocity, and degenerates when the pair is collinear with the velocity — that is expected and is exactly why (a) exists.

⚠ **The regression window spans a variable, non-uniform time interval.** M4's scheduler fires each pair at `effective_rate = min(ranging_rate_hz, max_exchanges_per_s / n_pairs)` — with the shipped `ranging_rate_hz: 10.0` and `max_exchanges_per_s: 400`, that is 10 Hz for a handful of drones but drops sharply as the mesh grows, and individual exchanges are dropped by the link-budget and NLOS models. Regress `range_m` against the **actual edge timestamps**, never against a sample index times an assumed `dt`. Require at least `range_rate_window` samples spanning ≤ `measurements.max_measurement_age_s`, otherwise skip the update.

**(d) Mutual yaw (D11).** When `i` measures bearing to `j` **and** `j` has broadcast its bearing to `i` (on `/cf_<j>/swarm_loc/bearing_rebroadcast`) within `measurements.mutual_yaw_max_dt_s` (default 0.2 s): the two bearing vectors, expressed in the global frame, must be antiparallel. Residual:

```
u_ij = R_i · unit(z_body_i→j)        # direction i→j in global frame
u_ji = R_j · unit(z_body_j→i)        # direction j→i in global frame
r = u_ij + u_ji                       # zero when frames are consistent
```

This is a 3-vector residual sensitive to `(ψ_i - ψ_j)`. Jacobians w.r.t. `ψ_i` and `ψ_j` follow from `∂R/∂ψ`. Use `psi_observer`, `roll_observer`, `pitch_observer` from the rebroadcast row for `R_j`, not `j`'s state-broadcast values — they must be the attitude at the instant of *that* measurement. This is the mechanism that makes relative yaw observable without a magnetometer — do not omit it.

⚠ **Mutual-yaw pairs will be scarce with the shipped UWB config. This is a known, accepted limitation — measure it, do not engineer around it.** `aoa_fov_deg: 90.0` with `boresight_axis: [1,0,0]` gives a ±45° forward bearing cone. In a single-file corridor, the trailing drone sees the leader but the leader's rear blind spot makes the reverse edge range-only — so D8 (one-sided rebroadcast) fires often while D11 (needs *both* directions inside the same `mutual_yaw_max_dt_s` window) may fire rarely or never.

Still implement D11 in full: the entrance node is a fixed-yaw observer in the mesh (§4.3e), junctions and turns do produce facing pairs, and a wider cone later makes the mechanism pay off without a rewrite.

**Do not** widen `aoa_fov_deg`, choreograph flight paths to manufacture facing pairs, add yaw-glance manoeuvres, or relax the pairing window to raise the count. Log `n_mutual_yaw_pairs_per_s` as a first-class metric from P2-6 onward and report whatever it is. A near-zero count is a **result about this sensor geometry**, and reporting it honestly alongside the yaw-drift curve is more valuable than a rate propped up by a scenario built to flatter it.

**(e) Entrance anchor (D12).** Identify entrance edges by the `FLAG_PEER_IS_SURVEYED` flag bit on the edge row; take the position from `entrance.position_xyz_m` in the estimator's own config (never from `uwb_pdoa.yaml`). Treat it as measurement (a) or (b) against a neighbor whose covariance is `entrance.sigma_m²·I`. Because its covariance is ~0, CI will assign nearly all correction to the drone — which is correct.

The entrance is also an *observer*: it publishes its own edges on `/uwb/peer_1000/edges` with a fixed yaw of 0. Those rows are a legitimate extra source of bearing-to-drone measurements and of D11 mutual-yaw pairs against a known-yaw partner (which pins absolute yaw, not just relative yaw, for any drone in the entrance's cone). Subscribe to `/uwb/peer_<id>/edges` for every surveyed peer.

### 4.4 Update with Covariance Intersection (D6)

For any measurement involving a neighbor's broadcast state, the two estimates share unknown correlation (both have been corrected by common information already). Naive EKF fusion will be overconfident and can diverge in a mesh. Use CI:

```
P_fused⁻¹ = ω·P_i⁻¹ + (1-ω)·P_j⁻¹
x_fused   = P_fused (ω·P_i⁻¹ x_i + (1-ω)·P_j⁻¹ x_j)
```

Choose `ω ∈ [0,1]` minimizing `trace(P_fused)`. `ci_omega_search: fast` = closed-form approximation `ω = trace(P_j)/(trace(P_i)+trace(P_j))`; `grid` = evaluate 21 evenly spaced `ω` and take the argmin. Default `fast`; use `grid` when validating.

**This directly implements "split the residual by uncertainty":** the drone with larger covariance receives the larger share of the correction, automatically, with no hand-tuned weighting.

### 4.5 Gating and guardrails (D16)

Before every update compute NIS = `rᵀ S⁻¹ r`, `S = H P Hᵀ + R`. Reject if NIS exceeds the chi-squared threshold at `nis_gate_chi2_p` for the residual dimension. Log all rejections with reason. Additionally:

- Clamp diagonal of `P` to `[cov_floor_p_m², max_cov_p_m²]`.
- Symmetrize `P = (P + Pᵀ)/2` after every update.
- If any position variance exceeds `max_cov_p_m²`, mark the drone `DIVERGED`, publish that status, and stop applying updates from it to neighbors (a diverged drone must not poison the mesh).
- Apply buffered measurements in timestamp order (D18); discard anything older than `measurements.max_measurement_age_s` (default 0.5). All timestamps in this system are **sim time** taken from message header stamps — never `Node.get_clock().now()`, which is wall time here (§9).
- Guard every model against `NaN`: the M4 edge row uses `NaN` for unavailable bearing fields, and a single `NaN` propagated into `P` silently kills the filter with no exception. Assert `np.all(np.isfinite(...))` on `r`, `H`, and `R` before the update and log-and-drop otherwise.

---

## 5. Milestones and gates

Each milestone has an exit gate. Do not start the next until the gate passes. Every gate result goes to MLflow.

### P2-1 — State, propagation, RIO stub (offline)
Build `state.py`, `rio_stub.py`, propagation half of `ekf.py`. No UWB yet. Split `rio_stub.py` into a pure-math corruption function (no `rclpy`, covered by `--selftest`) and a thin rclpy wrapper, so this gate can run with no simulator.
**Gate:** `python3 perception/swarm_loc/ekf.py --selftest` passes ≥15 checks including: analytic Jacobian `F` matches numerical differentiation to 1e-6; covariance stays symmetric positive-definite over 10k steps; with zero injected noise the propagated trajectory matches truth to <1e-6; with default stub noise, drift grows approximately linearly and is within 2× of the configured drift rate.

### P2-2 — Measurement models (offline)
Build `measurements.py`: models (a)–(e) with analytic Jacobians.
**Gate:** every Jacobian matches numerical differentiation to 1e-6 across 100 randomized geometries; spherical→Cartesian covariance propagation reproduces a hand-computed reference case; range-rate regression recovers a known `d'` from synthetic noisy ranges within tolerance.

### P2-3 — Single-pair EKF, static, entrance only (offline, no ROS)
One drone + entrance node, synthetic measurements.
**Gate:** position error bounded and NEES within the 95% consistency band over 1000 Monte Carlo runs. **If NEES is persistently above the band the filter is overconfident — fix it here, not later.**

### P2-4 — CI fusion + two-drone pair (offline)
Add `ci_fusion.py`, two drones exchanging broadcasts.
**Gate:** with bearing enabled, relative position error < range-only error by a clear margin; NEES consistent; a deliberate test where naive EKF fusion is substituted must demonstrably become overconfident (this proves CI is doing work — keep it as a regression test).

### P2-5 — ROS integration, N drones live
Build `swarm_loc_node.py` and `swarm_msgs.py`, and the rclpy half of `rio_stub.py`. Apply comms latency/loss (D17). Extend `eval_scripts/phase0_gate.sh`, copying the existing UWB block verbatim as the template:

- Add `--no-swarm-loc` and `--swarm-loc-config <path>` to the argument parser (~line 113), defaulting `USE_SWARM_LOC=true`.
- Add `pkill -f "swarm_loc_node.py"` and `pkill -f "rio_stub.py"` to the stale-process block (~line 253) next to the existing `uwb_node.py` kill.
- Include `/cf_<i>/odom` in the `ros_gz_bridge` condition (~line 407) whenever swarm-loc is enabled, since `rio_stub.py` needs it.
- After the `uwb_node.py` block (~line 455), launch **one `rio_stub.py --cf-id i` and one `swarm_loc_node.py --cf-id i` per drone** in a `for i in $(seq 0 $((NUM_DRONES - 1)))` loop, each with `python3 -u`, each appended to `_PIDS`. Resolve the config path with the same `[[ "$_cfg" != /* ]] && _cfg="$SAR_NANO_SWARM_ROOT/$_cfg"` idiom.

Build `eval_scripts/swarm_loc_gate.py` following `uwb_gate.py`'s structure: `sys.path` bootstrap, local `load_config`, a recorder `Node` on a `MultiThreadedExecutor` in a daemon thread, `pid_gains` for arming/flight, a `checks` dict, `gate_pass = all(checks.values())`, `PASS`/`FAIL` per line, then the MLflow block.

**Gate:** 3 drones fly a scripted path in a SubT world (`./eval_scripts/phase0_gate.sh -w phase0_tunnel_gate -n 3 --spacing 1.5 --headless --no-rviz`); every estimator publishes at `rate_hz`; no truth topic (`/uwb/edges_truth`, `/cf_*/odom`, `/cf_*/flow/debug_truth`) appears in any estimator's subscription list — assert programmatically by parsing `ros2 node info /swarm_loc_<i>`, and note that `rio_stub` is a *separate* node so its legitimate odom subscription does not contaminate the check; estimates remain finite and non-diverged for a full 5-minute run.

### P2-6 — Reciprocal bearing + mutual yaw
Implement D8 and D11 end-to-end.
**Gate — D8 half (blocking):** in a two-drone corridor test where drone 0 sees drone 1 (bearing) and drone 1 sees drone 0 only in its rear cone (range-only), drone 1's position error using the rebroadcast bearing is materially lower than with `d·d'` alone.

**D11 half (reporting, not blocking):** report `n_mutual_yaw_pairs_per_s` and the yaw-error curve with `use_mutual_yaw` true vs false. Per §4.3(d), the ±45° cone means the mutual-yaw pair rate may be near zero on this scenario, in which case the two yaw curves will look alike. **That outcome passes the gate** — record the pair rate and the curves in `.cursor/docs/P2_DEVIATIONS.md` and move on to P2-7. Do not alter the scenario, the cone, or the pairing window to make yaw drift appear.

Correctness of the D11 implementation is established offline instead, in the P2-2 Jacobian checks and a P2-3-style synthetic case with facing observers — not by the live pair rate.

### P2-7 — Centralized reference solver
`eval_scripts/central_reference.py`: read a full run's logged measurements, solve one batch least-squares over all drone poses at all keyframes, output the reference trajectory. Log measurements to disk during the run (`swarm_loc_node.py --log-measurements <path>`, one `.npz` per drone) — do not try to reconstruct them from a rosbag.
**Gate:** on a run where the distributed filter performs well, centralized and distributed agree closely; on a hard run, centralized is better (this quantifies the price of decentralization — a headline number for the paper).

### P2-8 — Corridor / degeneracy stress
Run the geometries that break things. Each maps to an existing knob — use it, do not invent a new mechanism:

| Scenario | How to produce it |
|---|---|
| Single-file collinear corridor | `phase0_gate.sh -n 4 --spacing 1.5` in a tunnel world; drones hold a line |
| Mesh partition | `kill` the middle drone's `cf2` process mid-run; M4 drops it from the mesh after `odom_stale_s` |
| Entrance-node dropout | run with `--uwb-config` pointing at a `/tmp` copy with an empty `static_peers: []` |
| Sustained NLOS on one link | `/tmp` UWB config copy with an `occluder_boxes` wall between two drones (`los_model: boxes`) |
| `RioDelta.valid=False` bursts | raise `rio.dropout_rate` / `rio.dropout_duration_s` in the estimator config |

**Gate:** no divergence in any case; degradation is graceful and **covariance grows to reflect it** (a filter that stays confident while wrong is a failure, even if its error is small). Produce error-vs-hops-from-entrance curves.

---

## 6. Evaluation

### 6.1 Metrics (log every run)
- ATE and RPE per drone vs. Gazebo truth (`/cf_<id>/odom`, read **only** in `eval_scripts/`)
- **Error vs. hop count from the entrance node** — the project's headline plot
- Yaw error over time, per drone
- NEES / NIS consistency (filter honesty)
- Fraction of edges that were bearing / reciprocal-bearing / range-only, plus the azimuth-only (`z == NaN`) fraction
- `n_mutual_yaw_pairs_per_s` (§4.3d)
- Measurement rejection rate (gate)
- Comms volume (bytes/s per drone) and CPU time per filter step. **Report CPU against GAP9, not the dev machine** (AGENTS.md §6.8: ~150 int-GOp/s, ~128 KB L1, ~1.5 MB L2). A per-step cost that only fits a laptop is a failed design, not a passing metric.
- Divergence events

### 6.2 Required ablations (`eval_scripts/run_ablations.py`)
Run identical scenarios under each condition:

1. **RIO only** (`disable_uwb: true`) — expect unbounded drift
2. **UWB range-only, no RIO** (`disable_rio: true`, `disable_bearing: true`) — expect degeneracy in corridors
3. **RIO + range-only UWB** — the Guo et al. comparison arm
4. **RIO + range + bearing** — the proposed system
5. **Full, minus entrance anchor** (`disable_entrance: true`) — expect internally consistent but globally drifting mesh
6. **Full, minus mutual yaw** — isolates D11. Expect yaw drift *only where mutual-yaw pairs actually occur*; on scenarios where the ±45° cone yields no facing pairs this arm will be indistinguishable from condition 7, which is the honest answer and not a failed run. Always report this arm next to `n_mutual_yaw_pairs_per_s`, or the comparison is uninterpretable.
7. **Full** — proposed system
8. **Centralized reference** (P2-7) — upper bound

Conditions 1, 2, 3 are the "either sensor alone fails" evidence. Conditions 5 and 6 isolate the two mechanisms this project claims are necessary.

Run every condition at **nominal and 2× UWB noise** (see §3.3 for how to generate the 2× arm). The M4 noise model *is* fitted to a real dataset now, but to **static, LOS, tripod-mounted lab data from a different, larger, tuned module** — not to a flying nano-airframe in a tunnel. Roadmap Phase 1 item 4 makes the 2× sweep mandatory for exactly this reason: any conclusion that flips between the two conditions is flagged model-sensitive and not claimed.

---

## 7. Known gaps — state these, do not paper over them

Record in `.cursor/docs/P2_DEVIATIONS.md` and in every results write-up:

1. **RIO is a stub.** All drift characteristics are assumed, not measured. Every number changes when real RIO lands.
2. **UWB noise is fitted, but to the wrong conditions.** *(Corrected — an earlier draft said the model was an unfitted `inv_cos`/8° placeholder. That is no longer true: `fit_uwb_noise.py` fitted it on 2026-08-06 against the ETH-PBL `UWB_DualAntenna_AoA` rotation dataset, and `uwb_pdoa.yaml` now runs `angle_error_model: piecewise_linear`, `sigma_boresight_deg: 4.795`, `angle_sigma_slope_deg_per_deg: 0.1649`, `sigma_range_los_m: 0.0873`.)* The remaining gap is the **conditions**: that dataset is static, LOS, tripod-mounted, 23 mm antenna spacing, on a larger tuned module. Flight vibration, downwash, tunnel multipath, and nano-airframe aperture limits are all unvalidated until Phase 11 bench data. Hence §6.2's mandatory 2× arm.
3. **NLOS and link-budget parameters are *not* fitted** — `nlos_bias_mean_m`, `nlos_sigma_mult`, `p_dropout_nlos`, `p_dropout_at_max_range` are Tier-B guesses, and NLOS is precisely the regime the corridor stress tests (P2-8) lean on.
4. **No clock drift modeled** (D18).
5. **Roll/pitch assumed accurate** (D4) — true in normal flight, less so under aggressive maneuvers or downwash.
6. **Bearing cone is ±45°, not 180°, and mutual yaw is starved by it** — `aoa_fov_deg: 90.0`. D11 requires bearing in *both* directions within one short window, which a forward-only ±45° cone rarely produces in corridor formations. **This is accepted as-is for this milestone; no workaround is in scope.** The consequence is that the yaw-observability claim (D11) is demonstrated analytically and in synthetic facing-pair cases, but may be weakly or not at all exercised in the live runs — say so explicitly rather than letting the reader assume it carried the results. The roadmap's 180° cone and yaw-glance manoeuvre are the eventual remedies; neither is implemented. State the cone next to every bearing-availability and mutual-yaw number.
7. **Landed peers are not anchors** (D20) — if a future milestone wants surveyed landed anchors, that is a new decision, not an implicit one.
8. **CI is conservative by construction** — it will underclaim accuracy relative to an optimal correlated fusion. This is intentional; do not "fix" it by switching to naive fusion.
9. **Altitude aiding is absent.** Baro, `/cf_<id>/tof_down`, and `/cf_<id>/flow` are simulated and gated but deliberately unused (see §1). A coplanar corridor mesh gives the vertical channel almost nothing, so Z error here is a floor, not a final number.
10. **Post-Run-B honesty patch is the min-floor, not Schmidt-Kalman** — see **§10**. Live NEES is now in-band; cf_2 yaw ~180° and mutual-yaw NIS rejects are remaining, not solved by that patch.

---

## 8. Implementation order

1. `configs/estimation/swarm_loc.yaml` → `state.py` → `rio_stub.py` → propagation in `ekf.py` → **P2-1 gate**
2. `measurements.py` → **P2-2 gate**
3. Update path in `ekf.py`, entrance anchor → **P2-3 gate**
4. `ci_fusion.py` → **P2-4 gate**
5. `swarm_msgs.py` → `swarm_loc_node.py` → `phase0_gate.sh` changes → `swarm_loc_gate.py` → **P2-5 gate**
6. Reciprocal bearing + mutual yaw → **P2-6 gate**
7. `central_reference.py` → **P2-7 gate**
8. Stress scenarios + `run_ablations.py` → **P2-8 gate**

Write the selftest for each module in the same commit as the module. Do not batch tests at the end.

---

## 9. Environment pitfalls that will cost you a day each

These are already documented in `AGENTS.md §4`; they bite this milestone specifically.

- **Source `setup_env.sh` in every terminal.** "Topic not found" is almost always this.
- **Kill stale `gz sim` *and* `cf2` between runs.** A leftover `gz sim` holds the UDP ports (19850+, 19950+) that `cf2` needs and the next run silently fails after printing "Simulation ready."
- **Use `python3 -u` whenever output is redirected to a log.** Python fully buffers stdout to a pipe; a script killed by `timeout` loses 100% of its output even though it ran and would have passed. Every node and gate in this milestone runs under redirection.
- **`mlflow.db` (sqlite) can wedge on the `/mnt/d` DrvFs mount** under concurrent or `kill -9`'d access. With one estimator per drone all logging to MLflow, this is a live risk: **only the gate script logs to MLflow — the per-drone nodes must not.** If a script hangs at `start_run()`, look for orphaned processes holding the DB before debugging anything else.
- **Sim time ≠ ROS wall time.** Compare receive wall time for staleness, use header stamps for `dt` and ordering. Mixing them is what made the optical-flow node report every sample stale.
- **The dev shell is PowerShell.** When invoking through `wsl -e bash -lc '...'`, single-quote the outer argument or PowerShell interpolates `$?`/`$var` before bash sees them.
- **`wsl -e bash -lc` from Cursor often cannot open cflib SITL links** (90–180 s timeout, hung UDP thread). Run `swarm_loc_gate.py` in an interactive WSL shell with `setup_env.sh` sourced. Kill `swarm_loc_gate.py` after a timeout before retrying — the daemon `open_links` thread keeps 19850+.

---

## 10. Post-Run-B honesty patch (implemented)

First live fly (Run B) was meter-class and overconfident: ATE ~1.3–1.6 m, mean NEES ~200–290, claimed σ ~0.05–0.25 m, all three drones blowing up together near t≈85 s, Z sinking, cf_2 yaw ~180°. Offline MC had been consistent (~cm, NEES~3). Diagnosis and the **minimum** (not Schmidt-Kalman) fix are below. Details also in `.cursor/docs/P2_DEVIATIONS.md`.

### 10.1 What was wrong

1. **Gauge starvation.** Entrance behind the ±45° cone → range-only (or dropped by `max_neighbors_per_drone`). Relative UWB holds *shape*; the whole formation drifts as a rigid body. Z is almost unconstrained once elevation to the entrance is gone.
2. **Overconfidence loop.** Relative updates cannot observe a common-mode translation. Neighbors were shrinking each other's absolute `P` down to measurement noise. CI on the pairwise update does not block that. Offline MC hid it because the entrance stayed in every graph.
3. **Yaw wrap.** An unwrapped az/ψ residual near ±π is ~2π and one huge update. NIS reject count 0 on Run B was a symptom.

### 10.2 What was implemented (files)

| Item | Where | Behavior |
|---|---|---|
| Entrance-edge series, centroid vs shape, NIS-by-type, `P_p` eigenvalues | `eval_scripts/eval_6_1.py`, `eval_6_1_plots.py` (plots 11–14) | Confirms #1/#2 on every `--eval-dir` run |
| `nis` array + 5 s `flush_log` | `meas_log.py`, `swarm_loc_node.py` | Hops/mix survive `timeout`; per-type NIS is on disk |
| Angular wrap | `measurements.wrap_measurement_residual`, used in `ekf.update` | 1-D az/el/ψ residuals wrap to (−π, π]. Cartesian relpos stays `z−h` unless the spherical az/el difference actually wraps. `mutual_yaw` wraps `ψ_i`, `ψ_j` before `R` |
| Force-include entrance | `uwb_model.update_scheduled_pairs` | Any in-range pair with `peer_type == "entrance"` is kept even if k-cap would drop it. Drone–drone k-cap **unchanged** (selftest 13 / 13b–c) |
| Gauge-age Q | `estimator.gauge_age_q_m2_per_s: 0.05` in `swarm_loc.yaml`, added in `process_Q` | Common-mode position random walk (m²/s). Relative UWB must not cancel it |
| Relative absolute floor | `ekf.apply_relative_abs_floor` after CI Joseph | On a **CI** neighbor update, each position variance is at least `min(P_i[k,k], P_j[k,k])`. Naive fusion is **not** floored (P2-4 regression: naive must stay overconfident) |
| PSD hygiene | `state.clamp_position_cov` eig-clips the 3×3; `project_psd` after propagate/update | Diagonal-only clamp at `max_cov_p_m: 2` was making `P` indefinite |

Gates: `measurements.py --selftest` (wrap 15), `ekf.py --selftest` (17c–e no-anchor σ grows), `uwb_model.py --selftest` (13b), `stress.py --selftest` (3d), `eval_6_1.py --selftest`.

### 10.3 Deviations from the diagnosis note / locked plan

- **Not implemented:** Schmidt-Kalman / considered-state neighbor pose, or a proper common-mode vs relative-mode covariance split. The note called those “correct”; the floor is the minimum version. Do not “fix” remaining conservatism by switching CI to naive fusion (D6, §7.8).
- **`max_cov_p_m` stays 2 m**, not the plan’s 50 m. Unbounded no-anchor growth trips diverge at 2 m; that is the abort, not a cm-σ plateau.
- **Force-include is entrance-only** (`peer_type == "entrance"`), not landed peers (D20).
- **Did not widen `aoa_fov_deg`**, add ToF/baro, or subscribe to odom in `perception/swarm_loc/`.
- **Cartesian relpos residual is still `z−h`** in the usual (non-wrapping) case so P2-2 Jacobians and the NIS-outlier selftest stay honest. Wrap is applied when the implied az/el difference crosses ±π, and always for 1-D angular names.
- **`t_last_gauge` is not on the wire** (`STATE_DTYPE` itemsize 88 unchanged). Gauge age is process noise + the min-prior floor, not a broadcast timestamp.
- **P2-3 NEES check** allows mean NEES ≤ 1.03× the χ² band upper edge (finite-MC / Wilson–Hilferty slack). Overconfidence is still a fail if it is persistent.

### 10.4 Live re-run (post-patch) — `out/swarm_loc_eval/metrics_6_1.json`

3-drone tunnel, hop count 1 for all (entrance force-included). No diverged rows.

| drone | ATE RMSE | RPE 1 s | yaw RMSE | mean NEES | NEES in χ²95 |
|---|---|---|---|---|---|
| cf_0 | 0.32 m | 0.24 m | 7.8° | 1.64 | 0.993 |
| cf_1 | 0.29 m | 0.16 m | 9.2° | 1.09 | 0.999 |
| cf_2 | 0.38 m | 0.17 m | **173°** | 2.76 | 0.960 |

Vs Run B (~1.3–1.6 m ATE, NEES ~200, σ dishonest): position and honesty are in the intended regime. **σ growing down-tunnel is success**, not a failure.

**Still true on this run (do not paper over):**

- **Entrance bearing count is 0** for all three; only range-only to peer 1000 (~15–30 edges/s). The cone is doing what §7.6 said. Force-include keeps the *range* gauge; it does not create elevation.
- **Centroid vs shape:** mean centroid error 0.26 m, shape 0.16 m, `centroid_explains_ate: false`. Not a pure rigid-body drift anymore.
- **cf_2 yaw ~180° is not fixed.** Wrap stopped the 2π innovation blow-up; a π flip can still sit in a consistent local minimum under relpos. D11 is not carrying live yaw: `nis_by_type.mutual_yaw` mean NIS ~4.6e4, **63520 / 87119 rejected**. Reciprocal relpos is also hot (14267 / 23977 rejected). Aggregate NIS reject rate **0.327** is almost entirely those two, not range/relpos/entrance_range (those means are O(1)).
- Logged `n_mutual_yaw_pairs_per_s ≈ 7.5` is **pair attempts**, not accepted D11 updates. The `aoa_fov_note` in eval still says live mutual-yaw is often ~0 in the *accepted* sense; do not cite 7.5 as D11 working in the corridor.
- Mix: frac_bearing 0.33, frac_range_only 0.67 (peer–peer bearing still exists; entrance is range-only).

### 10.5 What to do next (not this patch)

Z honesty is the floor + entrance range; Z *accuracy* still needs in-cone elevation or scoped-out altitude aiding. cf_2 yaw needs a separate investigation (sign of D8/D11, init, or a π mode), not another common-mode floor. Schmidt-Kalman remains the structural upgrade if the min-floor is too conservative for a paper claim.