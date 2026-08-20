# M5 — Swarm Localization Implementation Plan
## RIO + UWB Range/Bearing Distributed Estimator (`sar_nano_swarm`)

**Audience:** implementing model/engineer. **Every design decision in this document is already made.** Do not re-litigate them. If you hit a case this document does not cover, pick the option that is *more physically realistic*, even if it makes results look worse, and write the choice in `docs/M5_DEVIATIONS.md`.

**Prerequisite state (already done, M4):** `perception/uwb_sim/` publishes simulated UWB edges (range + optional azimuth/elevation in observer body frame) on `/cf_<id>/uwb/edges`, `/uwb/peer_<id>/edges`, `/uwb/edges_all`, with `/uwb/edges_truth` as a sim oracle. Edge rows are packed `PointCloud2` via `pack_edges()`/`unpack_edges()` in `perception/uwb_sim/uwb_edges.py`.

**What M5 adds:** the estimator that consumes those edges plus (stubbed) RIO odometry and produces each drone's corrected global pose.

---

## 0. Scope

**In scope:** per-drone distributed state estimator, inter-drone state broadcast, entrance-anchor gauge, reciprocal-bearing handling, range-only fallback, an offline centralized reference solver, evaluation harness and ablations.

**Out of scope (do not build):** mapping, frontier exploration, obstacle avoidance, victim detection, loop closure / place recognition, GAP9 porting, real hardware. Do not add these even if they seem helpful.

**Hard rule:** no node in `perception/swarm_loc/` may subscribe to `/uwb/edges_truth` or to any Gazebo ground-truth pose topic. Truth is used **only** inside `eval_scripts/` and inside the RIO stub's internal error-injection (§3.2), never read by the estimator itself. Violating this invalidates every result the project will publish.

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
| D12 | Global gauge | One **entrance node** at origin, `type: entrance`, covariance fixed at ~0 (config `entrance_sigma_m`, default 0.01) | Without it the swarm is internally consistent but globally free (gauge freedom) |
| D13 | Initialization | All drones **launch from directly above the entrance node**, sequentially; at arm time each drone's state is initialized to the surveyed entrance position with small covariance | Establishes a shared frame at t=0. Note: this is an initial condition, not a substitute for ongoing correction |
| D14 | Yaw initialization | All drones initialized to a **common yaw reference** at launch, with configurable initial yaw uncertainty (default 5°) | Do not assume magnetometer. Common launch heading is the realistic assumption |
| D15 | Reference solver | An **offline centralized batch least-squares** solver (`scipy.optimize.least_squares`, sparse Jacobian) run post-hoc in `eval_scripts/` | Gives the "best achievable given these measurements" upper bound to compare the distributed filter against. Not GTSAM — avoid the build dependency |
| D16 | Outlier handling | **Chi-squared (NIS) gate** on every measurement before update; rejected measurements logged, never silently dropped | NLOS/multipath outliers will otherwise corrupt the graph |
| D17 | Comms model | State broadcast at configurable rate with configurable **latency and packet-loss**; defaults 10 Hz, 20 ms, 2% loss | Perfect comms is a fake result |
| D18 | Clock sync | Assume common clock (sim Tier A); every message carries a timestamp and the filter **buffers and applies measurements in timestamp order** | Clock drift is deferred, but the plumbing must exist. Document as a limitation |
| D19 | Antenna-delay bias | Estimator must **not** be given the per-device bias; it must survive it as unmodeled error | M4 deliberately excluded it from reported σ. Keep it honest |
| D20 | Landed peers | `type: landed` peers are treated as **normal drones whose velocity is zero**, with their own uncertainty — *not* as zero-uncertainty anchors | A landed drone's position is only as good as its estimate when it landed. Only the entrance node is ground truth |

---

## 2. Repository layout (new files only)

```
configs/estimation/swarm_loc.yaml          # all M5 tunables (§3.3)
perception/swarm_loc/__init__.py
perception/swarm_loc/state.py              # state vector, covariance, error-state helpers
perception/swarm_loc/rio_stub.py           # RIO interface + drift-injecting stub (§3.2)
perception/swarm_loc/measurements.py       # measurement models + Jacobians (§4.3)
perception/swarm_loc/ci_fusion.py          # covariance intersection (§4.4)
perception/swarm_loc/ekf.py                # pure math, no rclpy, --selftest
perception/swarm_loc/swarm_loc_node.py     # rclpy wrapper, one instance per drone
perception/swarm_loc/msg_defs.py           # broadcast packing/unpacking (§3.1)
eval_scripts/central_reference.py          # offline batch solver (D15)
eval_scripts/swarm_loc_gate.py             # M5 exit gate
eval_scripts/run_ablations.py              # ablation sweep driver (§6.2)
docs/M5_DEVIATIONS.md                      # any deviation from this plan, with reason
```

Follow M4 conventions exactly: pure-math modules have **no `rclpy` import** and expose `--selftest`; ROS wrappers contain no math. Log every run to MLflow using the existing experiment-naming pattern in `configs/sensors/uwb_pdoa.yaml`.

---

## 3. Interfaces and data contracts

### 3.1 Inter-drone broadcast message

Each drone broadcasts on `/cf_<id>/swarm_loc/broadcast` at `broadcast_rate_hz`. Pack as `PointCloud2` rows using the same helper pattern as `uwb_edges.py` (keeps tooling consistent; do not introduce custom `.msg` files).

Fields per broadcast:

| Field | Type | Note |
|---|---|---|
| `stamp` | float64 | seconds |
| `drone_id` | uint16 | |
| `p_x, p_y, p_z` | float32 ×3 | global position estimate |
| `v_x, v_y, v_z` | float32 ×3 | global velocity estimate |
| `psi` | float32 | global yaw estimate (rad) |
| `cov_p` | float32 ×6 | upper-triangular 3×3 position covariance |
| `cov_psi` | float32 | yaw variance |
| `roll, pitch` | float32 ×2 | from IMU, treated as known |
| `seq` | uint32 | for loss detection |
| `n_bearing_edges` | uint8 | diagnostics |

Also rebroadcast, on the same topic family, any **bearing measurement this drone made** (D8): `peer_id`, `range`, `az`, `el`, `sigma_range`, `sigma_az`, `sigma_el`, `stamp`. The peer uses this instead of its own `d·d'` fallback.

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

**The stub** (`--rio-source=stub`) reads Gazebo truth internally, computes the true delta, then corrupts it with:
- velocity random walk (bias on `delta_p_body`, config `rio_vel_bias_walk`)
- yaw random walk (config `rio_yaw_walk_deg_per_min`, default 3.0)
- multiplicative scale error (config `rio_scale_error`, default 1.02)
- white noise per axis (config `rio_sigma_p`, `rio_sigma_psi`)
- `valid=False` bursts at `rio_dropout_rate` for `rio_dropout_duration_s` (radar sparsity)

Defaults must be **pessimistic**, matching published RIO drift (~2%/distance) rather than optimistic. The stub is the only file in `perception/` allowed to touch truth; mark it clearly and gate it behind the `--rio-source` flag so a future real RIO cannot silently fall back to it.

### 3.3 Config: `configs/estimation/swarm_loc.yaml`

Every number the estimator uses lives here. No magic numbers in code. Required sections:

```yaml
estimator:
  rate_hz: 50                      # filter propagation rate
  state_init_sigma_p_m: 0.05
  state_init_sigma_v_mps: 0.05
  state_init_sigma_psi_deg: 5.0
  yaw_bias_walk_sigma: 1.0e-4
  scale_init: 1.0
  scale_init_sigma: 0.02
  cov_floor_p_m: 0.01              # prevents over-confidence collapse
  max_cov_p_m: 50.0                # divergence detector threshold

measurements:
  use_bearing: true
  use_reciprocal_bearing: true     # D8
  use_range_only_fallback: true    # D9
  use_range_rate: true
  use_mutual_yaw: true             # D11
  range_rate_window: 5             # D10
  nis_gate_chi2_p: 0.99            # D16
  entrance_sigma_m: 0.01           # D12

comms:
  broadcast_rate_hz: 10            # D17
  latency_ms: 20
  packet_loss: 0.02
  max_neighbors: 6

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

ablation:
  disable_uwb: false
  disable_bearing: false
  disable_entrance: false
  disable_rio: false
```

---

## 4. Math specification

Implement exactly this. Do not substitute alternative formulations.

### 4.1 Frames and conventions

- Global frame `G`: right-handed, origin at the entrance node, x-forward into the structure, z-up. Set at launch (D13).
- Body frame `B_i`: drone `i`'s frame. Rotation `R_i = Rz(ψ_i)·Ry(pitch_i)·Rx(roll_i)`.
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

If `RioDelta.valid == False`: propagate with **inflated** `Q` (multiply by `rio_dropout_q_scale`, default 25) and hold velocity — do not skip propagation.

### 4.3 Measurement models

Let `i` = self, `j` = neighbor. `p̂_j`, `P_j` come from the neighbor's broadcast.

**(a) Full relative position (bearing available — D7).** Convert the UWB spherical reading to body-frame Cartesian:

```
z_body = [ d·cos(el)·cos(az),
           d·cos(el)·sin(az),
           d·sin(el) ]
```

Measurement model: `h = R_iᵀ (p_j - p_i)`, residual `r = z_body - h`. Jacobians:

```
∂h/∂p_i = -R_iᵀ
∂h/∂p_j = +R_iᵀ
∂h/∂ψ_i = (∂R_iᵀ/∂ψ)(p_j - p_i)
```

Measurement covariance: propagate `(σ_d, σ_az, σ_el)` from the M4 edge row through the spherical→Cartesian Jacobian. **Do not** use an isotropic covariance — bearing error produces a covariance ellipsoid that is long in the transverse directions and short in range; faking it isotropic is a fidelity failure.

**(b) Range-only (D9, no bearing on either side).** `h = ‖p_j - p_i‖`, `∂h/∂p_i = -(p_j-p_i)ᵀ/‖·‖`. Covariance `σ_d²`.

**(c) Range-rate (D9/D10).** `h = (p_j - p_i)·(v_j - v_i) / ‖p_j - p_i‖`, computed `d'` from the regression window. Note in code comments: this constrains only the component along relative velocity, and degenerates when the pair is collinear with the velocity — that is expected and is exactly why (a) exists.

**(d) Mutual yaw (D11).** When `i` measures bearing to `j` **and** `j` has broadcast its bearing to `i` within `mutual_yaw_max_dt` (default 0.2 s): the two bearing vectors, expressed in the global frame, must be antiparallel. Residual:

```
u_ij = R_i · unit(z_body_i→j)        # direction i→j in global frame
u_ji = R_j · unit(z_body_j→i)        # direction j→i in global frame
r = u_ij + u_ji                       # zero when frames are consistent
```

This is a 3-vector residual sensitive to `(ψ_i - ψ_j)`. Jacobians w.r.t. `ψ_i` and `ψ_j` follow from `∂R/∂ψ`. This is the mechanism that makes relative yaw observable without a magnetometer — do not omit it.

**(e) Entrance anchor (D12).** The entrance peer is a static edge with a surveyed position. Treat it as measurement (a) or (b) against a neighbor whose covariance is `entrance_sigma_m²·I`. Because its covariance is ~0, CI will assign nearly all correction to the drone — which is correct.

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
- Apply buffered measurements in timestamp order (D18); discard anything older than `max_measurement_age_s` (default 0.5).

---

## 5. Milestones and gates

Each milestone has an exit gate. Do not start the next until the gate passes. Every gate result goes to MLflow.

### M5-1 — State, propagation, RIO stub (offline)
Build `state.py`, `rio_stub.py`, propagation half of `ekf.py`. No UWB yet.
**Gate:** `python3 perception/swarm_loc/ekf.py --selftest` passes ≥15 checks including: analytic Jacobian `F` matches numerical differentiation to 1e-6; covariance stays symmetric positive-definite over 10k steps; with zero injected noise the propagated trajectory matches truth to <1e-6; with default stub noise, drift grows approximately linearly and is within 2× of the configured drift rate.

### M5-2 — Measurement models (offline)
Build `measurements.py`: models (a)–(e) with analytic Jacobians.
**Gate:** every Jacobian matches numerical differentiation to 1e-6 across 100 randomized geometries; spherical→Cartesian covariance propagation reproduces a hand-computed reference case; range-rate regression recovers a known `d'` from synthetic noisy ranges within tolerance.

### M5-3 — Single-pair EKF, static, entrance only (offline, no ROS)
One drone + entrance node, synthetic measurements.
**Gate:** position error bounded and NEES within the 95% consistency band over 1000 Monte Carlo runs. **If NEES is persistently above the band the filter is overconfident — fix it here, not later.**

### M5-4 — CI fusion + two-drone pair (offline)
Add `ci_fusion.py`, two drones exchanging broadcasts.
**Gate:** with bearing enabled, relative position error < range-only error by a clear margin; NEES consistent; a deliberate test where naive EKF fusion is substituted must demonstrably become overconfident (this proves CI is doing work — keep it as a regression test).

### M5-5 — ROS integration, N drones live
Build `swarm_loc_node.py` and `msg_defs.py`; extend `eval_scripts/phase0_gate.sh` to launch one estimator node per drone. Apply comms latency/loss (D17).
**Gate:** 3 drones fly a scripted path in a SubT world; every estimator publishes at `rate_hz`; no truth topic appears in any estimator subscription list (assert programmatically via `ros2 node info`); estimates remain finite and non-diverged for a full 5-minute run.

### M5-6 — Reciprocal bearing + mutual yaw
Implement D8 and D11 end-to-end.
**Gate:** in a two-drone corridor test where drone 0 sees drone 1 (bearing) and drone 1 sees drone 0 only in its rear cone (range-only), drone 1's position error using the rebroadcast bearing is materially lower than with `d·d'` alone; yaw error stays bounded over 5 minutes with no magnetometer, whereas with `use_mutual_yaw: false` it drifts.

### M5-7 — Centralized reference solver
`eval_scripts/central_reference.py`: read a full run's logged measurements, solve one batch least-squares over all drone poses at all keyframes, output the reference trajectory.
**Gate:** on a run where the distributed filter performs well, centralized and distributed agree closely; on a hard run, centralized is better (this quantifies the price of decentralization — a headline number for the paper).

### M5-8 — Corridor / degeneracy stress
Run the geometries that break things: single-file collinear corridor, mesh partition (kill a middle drone), entrance-node dropout, sustained NLOS on one link, `RioDelta.valid=False` bursts.
**Gate:** no divergence in any case; degradation is graceful and **covariance grows to reflect it** (a filter that stays confident while wrong is a failure, even if its error is small). Produce error-vs-hops-from-entrance curves.

---

## 6. Evaluation

### 6.1 Metrics (log every run)
- ATE and RPE per drone vs. Gazebo truth
- **Error vs. hop count from the entrance node** — the project's headline plot
- Yaw error over time, per drone
- NEES / NIS consistency (filter honesty)
- Fraction of edges that were bearing / reciprocal-bearing / range-only
- Measurement rejection rate (gate)
- Comms volume (bytes/s per drone) and CPU time per filter step
- Divergence events

### 6.2 Required ablations (`eval_scripts/run_ablations.py`)
Run identical scenarios under each condition:

1. **RIO only** (`disable_uwb: true`) — expect unbounded drift
2. **UWB range-only, no RIO** (`disable_rio: true`, `disable_bearing: true`) — expect degeneracy in corridors
3. **RIO + range-only UWB** — the Guo et al. comparison arm
4. **RIO + range + bearing** — the proposed system
5. **Full, minus entrance anchor** (`disable_entrance: true`) — expect internally consistent but globally drifting mesh
6. **Full, minus mutual yaw** — expect yaw drift
7. **Full** — proposed system
8. **Centralized reference** (M5-7) — upper bound

Conditions 1, 2, 3 are the "either sensor alone fails" evidence. Conditions 5 and 6 isolate the two mechanisms this project claims are necessary. Run every condition at **nominal and 2× UWB noise** — the noise model is not yet validated against hardware, so no single-noise-level claim is trustworthy.

---

## 7. Known gaps — state these, do not paper over them

Record in `docs/M5_DEVIATIONS.md` and in every results write-up:

1. **RIO is a stub.** All drift characteristics are assumed, not measured. Every number changes when real RIO lands.
2. **UWB angle noise is unvalidated** (M4 used an `inv_cos`/8°-boresight placeholder). Phase 1 dataset fitting must precede any published claim.
3. **No clock drift modeled** (D18).
4. **Roll/pitch assumed accurate** (D4) — true in normal flight, less so under aggressive maneuvers or downwash.
5. **Landed peers are not anchors** (D20) — if a future milestone wants surveyed landed anchors, that is a new decision, not an implicit one.
6. **CI is conservative by construction** — it will underclaim accuracy relative to an optimal correlated fusion. This is intentional; do not "fix" it by switching to naive fusion.

---

## 8. Implementation order

1. `state.py` → `rio_stub.py` → propagation in `ekf.py` → **M5-1 gate**
2. `measurements.py` → **M5-2 gate**
3. Update path in `ekf.py`, entrance anchor → **M5-3 gate**
4. `ci_fusion.py` → **M5-4 gate**
5. `msg_defs.py` → `swarm_loc_node.py` → launcher changes → **M5-5 gate**
6. Reciprocal bearing + mutual yaw → **M5-6 gate**
7. `central_reference.py` → **M5-7 gate**
8. Stress scenarios + `run_ablations.py` → **M5-8 gate**

Write the selftest for each module in the same commit as the module. Do not batch tests at the end.