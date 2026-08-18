# M4 Implementation Plan — Inter-Drone UWB Ranging + PDoA Bearing (Swarm Relative Positioning)

**Status of this doc:** **IMPLEMENTED & GATED (2026-07-30).** Live gate: `uwb_gate.py` PASS 12/12; offline selftest 17/17. Expands §5 (M4a/M4b) of
`Phase1_Physical_Fidelity_and_Sensor_Implementation_Plan.md`. Where it disagrees with that
doc, **this doc wins** — every difference is listed in §1.5 with a reason, and the two
substantive reversals (§1.1 language choice, §1.3 airtime model) are called out explicitly
so they can be argued with.

**Audience:** an implementer who has *not* read the rest of the repo. Everything needed to
write the code is here. Do not invent APIs: `AGENTS.md §6.4` — if a symbol is unverified,
print its docstring or grep the header rather than guessing. Constants quoted from the
submodule carry their `file:line`.

---

## 0. What M4 is and is not

**Is:** a *sensor model*. It produces, for every drone, a stream of measured **edges** to
its neighbours — `(range, azimuth, elevation)` in that drone's own body frame, with
per-measurement uncertainty and validity flags, degraded by NLOS, antenna geometry, and
radio airtime. Plus a directly-derivable **relative position** of each neighbour in the
observer's body frame, which is "relative positioning" in the usable sense.

**Is not:** the mesh estimator. The multi-drone pose graph / fixed-lag smoother that fuses
these edges with IMU, radar-inertial odometry, flow, and loop closures is **Phase 2** and
belongs to the RIO/estimator workstream. M4's job is to hand Phase 2 a measurement stream
that is *honest* — one that punishes an estimator making bad assumptions, rather than
flattering it.

The line matters because of `AGENTS.md §1 Tier A`: **no ground-truth neighbour position
ever enters the estimator.** The simulator knows where everyone is; the estimator only
ever sees the noisy edges on the topic. §3.6 specifies the one debug topic that carries
truth and the three places it must be labelled as a sim oracle.

### 0.1 Architecture context — read this so you don't build the retired design

The v2 design had drones **dropping UWB anchor pucks** inside the structure. **That is
retired.** The reason is not technical, it is programmatic: *we are not going to
manufacture pucks.* Two replacements are in scope conceptually, and the model in this doc
supports both without extra code (§7):

1. **Landed drones as reference points.** A drone that lands and stays put is a static
   node. It is still a *drone* — same radio, same PDoA array, same measurement model —
   it just stops moving. Its position is **still a state variable, never a known
   constant** (`AGENTS.md §1 Tier A`): it landed at a drifted estimated pose.
2. **MiFly-style passive retroreflective tags** — see §12, an aside for later, not part of
   this build.

The **one** node whose pose *is* a legitimate known constant is the fixed **entrance /
base-station** node outside the structure. It is the gauge the whole mesh hangs off. Model
it as an ordinary peer so the same maths applies uniformly; the only difference is a flag
telling downstream "this one's pose is surveyed."

---

## 1. Design decisions — do not re-litigate these while implementing

### 1.1 REVERSAL: Python `rclpy` node, **not** a C++ gz-sim System plugin

The parent plan (§5.5) specifies a C++ gz-sim System plugin, "sibling to `radarays_gz2` —
same reasoning: needs per-entity-pair geometry computed against live sim state every step."
**That reasoning does not survive contact with the numbers, and the decision is reversed.**

- **The radar genuinely needs C++.** It fires thousands of rays per scan through an Embree
  BVH at 10 Hz. That is a real compute budget.
- **UWB is not that.** `N` drones give `N(N−1)/2` pairs. At the roadmap's headline `N=50`
  that is 1 225 pairs; at the actual gate size it is **1**. Each pair is a subtract, a
  rotate, an `atan2`, and a few Gaussian draws — call it 30 flops. Vectorised in numpy the
  entire 50-drone mesh is a handful of `(50, 50, 3)` array ops. At 10 Hz that is
  **~0.4 MFLOP/s.** This is not a C++ problem. Building it in C++ costs a CMake target, a
  colcon build step, and a maintenance burden in a language `AGENTS.md §1 Tier A` says the
  user does not read — for zero measurable benefit.
- **The one expensive part is the NLOS occlusion raycast**, and §5 solves that in Python
  three different ways, the default of which needs no mesh and no dependencies at all.
- `AGENTS.md §1 Tier A` is explicit: *"C++ only where forced; all algorithmic/ML/eval work
  in Python."* UWB is algorithmic work. It is not forced.

**Trigger to revisit:** if a profiled run with `N ≥ 30` drones shows the node's per-tick
cost exceeding ~20 % of the tick period, revisit. Log per-tick wall time to MLflow (§3.7)
so that number exists rather than being argued about.

**What we lose:** the plugin would read poses straight out of the gz ECM. The node instead
consumes the bridged `/cf_<id>/odom` topics (§3.2), which adds bridge latency (sub-ms,
irrelevant at 10 Hz ranging) and requires the odom bridge to be running. Acceptable.

### 1.2 One node for the whole swarm, not one per drone

A single `uwb_node.py` process subscribes to every drone's odom and publishes every
drone's edge topic. Reason: UWB is inherently a **shared-medium, pairwise** phenomenon —
one TWR exchange produces one range that *both* participants observe. Modelling that
correctly (§2.4) requires one place that owns the pair. N independent processes would each
draw their own noise and silently fabricate independent measurements (§2.4 explains why
that is a serious, subtle error). The airtime scheduler (§1.3) has the same requirement.

### 1.3 ADDITION: model radio airtime, because the mesh does not scale for free

The parent plan gives UWB a flat `update_rate_hz`. **That silently assumes infinite radio
airtime, and it is the single most flattering assumption available to this workstream.**

UWB TWR is a shared medium. Every exchange is a real over-the-air transaction costing real
milliseconds, and every drone in earshot is competing for the same channel. A full mesh
needs `N(N−1)/2` exchanges per round. At `N = 50` that is 1 225 exchanges; a generous
400 exchanges/s budget gives each pair **0.33 Hz**. That is a load-bearing fact about
whether the moonshot's mesh is viable, and a sensor model that hides it will produce a
Phase-2 estimator tuned against a fantasy.

So the model carries three coupled knobs (§4):

- `ranging_rate_hz` — the rate a pair *would* get with the channel to itself.
- `max_exchanges_per_s` — total channel budget shared by the whole swarm.
- `max_neighbors_per_drone` — a drone only ranges with its K best-link peers, which is
  what real systems do and what makes the mesh scale at all.

Effective per-pair rate is `min(ranging_rate_hz, max_exchanges_per_s / n_scheduled_pairs)`,
and the scheduler round-robins (§2.6). At the 2-drone gate size all three are slack and the
gate sees the full `ranging_rate_hz` — so this costs the gate nothing while making the
50-drone claim honest.

### 1.4 Emit a structured edge record, and ship the pack/unpack helper with it

Same argument as M3b §1.3: a custom ROS message needs a `rosidl` package, CMake, and a
colcon build. Instead, use **`sensor_msgs/msg/PointCloud2` as a generic typed record
array** — which is what it actually is. One "point" per edge, twelve fields (§3.4),
including `x,y,z` = the peer's position in the observer's body frame so **RViz renders the
live mesh for free**.

Because hand-rolling `PointField` layouts is error-prone, **ship
`perception/uwb_sim/uwb_edges.py` with `pack_edges()` / `unpack_edges()`** in the same
commit. Neither we nor the teammate's estimator ever touches a byte offset; consuming an
edge stream is a one-line import. This is non-negotiable — the message design is only
acceptable *because* the helper exists.

### 1.5 Deltas from the parent plan §5, with reasons

| Parent plan §5 said | This plan does | Why |
|---|---|---|
| C++ gz-sim System plugin | **Python `rclpy` node** | §1.1 — the compute argument doesn't hold; Tier A says Python. |
| flat `update_rate_hz` | **airtime budget + neighbour cap + round-robin scheduler** | §1.3 — a flat rate hides the mesh's scaling limit. |
| separate `uwb_range_node.py` (M4a) and PDoA plugin (M4b) | **one node, one model**; M4a is a config with static peers | The maths is identical; a range-only anchor is a PDoA peer with bearing disabled. Two codebases for one model is pure duplication. |
| `uwb.yaml` + `uwb_anchors.yaml` + `uwb_pdoa.yaml` (three files) | **one `configs/sensors/uwb_pdoa.yaml`** | Same reason. Peer list lives in the same file as the noise model that applies to it. |
| antenna spacing "~4–5 cm a nano-drone can fit" | **`antenna_spacing_m: 0.023` (≈ λ/2 at 6.5 GHz)** | §2.3 — 4–5 cm is **greater than λ/2 and is phase-ambiguous**. This is a physics error in the parent doc, not a tuning choice. |
| az + el, no mention of array geometry | **explicit 3-antenna L-array, configurable boresight axis, documented rear blind spot** | §2.3 — you cannot get two angles from two antennas, and boresight direction is a real design decision with a real consequence (§2.3.4). |
| NLOS adds "positive bias + inflated σ" to range | **NLOS additionally invalidates bearing by default** | §2.5 — a multipath arrival's angle of arrival is the *reflector's* direction, not the peer's. Keeping the bearing would be worse than dropping it. |
| — (absent) | **per-device antenna-delay bias, sampled once at startup** | §2.2 — uncalibrated antenna delay is a persistent 10–30 cm *constant* offset. An estimator assuming zero-mean range noise will quietly diverge on it. Real, cheap to model, and the most commonly forgotten UWB error term. |
| — (absent) | **`--selftest`, no ROS, no sim** | Mirrors M3b. ~70 % of the correctness risk retired before Gazebo ever launches. |

### 1.6 Explicitly out of scope

- **The mesh estimator / pose graph.** Phase 2. §0.
- **Neighbour velocity broadcast** (meeting 2026-07-28: each drone broadcasts its own
  estimated `(vx,vy,vz)` + yaw rate with covariance). That is a *comms + estimator*
  product, not a sensor: the number broadcast is the drone's own **estimate**, which does
  not exist until Phase 2. Do not fake it from ground truth.
- **Clock-drift / carrier-frequency-offset modelling** in TWR. Real, second-order next to
  antenna delay and NLOS. Note it as unmodelled in the config header.
- **Chirp/slot interference between radios** — Phase 7 item 5.
- **MiFly passive tags** — §12, aside only.

---

## 2. The measurement model

### 2.1 Frames and definitions — fix these before writing a line

- World frame: gz/ROS standard, **Z up**.
- Body frame: **FLU** — `+x` forward, `+y` left, `+z` up. This is the gz-sim model
  convention the Crazyflie SDF uses.
- For observer `i` and peer `j`, with world positions `p_i, p_j` and observer body→world
  rotation `R_i` (from `odom.pose.orientation`):

```
d_w   = p_j - p_i                       # world-frame displacement
d_b   = R_i^T · d_w                     # peer position in OBSERVER body frame
r     = ||d_b||                         # true range, m   (== ||d_w||)
az    = atan2(d_b.y, d_b.x)             # rad, (-pi, pi],  0 = dead ahead, +ve = to the LEFT
el    = asin(clamp(d_b.z / r, -1, 1))   # rad, [-pi/2, pi/2], +ve = ABOVE
```

- **Boresight** is the antenna-array normal, config `boresight_axis`, default
  `[1, 0, 0]` (forward). Off-boresight angle:

```
theta = acos(clamp(dot(d_b / r, boresight_axis_unit), -1, 1))    # rad, [0, pi]
in_cone = theta <= radians(aoa_fov_deg) / 2
```

Write these five lines as a single tested function. Every sign error in this workstream
will originate here; §8.1 asserts each one numerically.

### 2.2 Range measurement (TWR)

```
b_ij      = antenna_delay_bias[i] + antenna_delay_bias[j]     # CONSTANT per pair, §2.2.1
r_meas    = r + b_ij + nlos_bias + gauss(0, sigma_r)
sigma_r   = sigma_range_los_m                      if LOS
          = sigma_range_los_m * nlos_sigma_mult    if NLOS
```

**Publish `sigma_r` as the reported uncertainty — but do NOT include `b_ij` in it.** The
bias is unmodelled-by-the-estimator on purpose; that is the whole point of §1.5's row. An
estimator that wants to survive it must estimate a per-pair bias state or use a robust
kernel, which is exactly the Phase-2 design pressure we want to create.

#### 2.2.1 Antenna-delay bias

Sample **once, at node startup**, per device (including the entrance node), from
`gauss(0, antenna_delay_bias_sigma_m)` using the config seed. Store in a dict keyed by
device id. It never changes during a run. Default `antenna_delay_bias_sigma_m: 0.05`
(→ typical pair bias ~7 cm, tail to ~20 cm — consistent with what uncalibrated DW-class
radios actually do). Set it to `0.0` to disable and recover the naive model, and log the
sampled table at startup so gate failures are explicable.

### 2.3 Bearing measurement (PDoA)

#### 2.3.1 The physics you must not get wrong

PDoA measures the **phase difference** of the same arriving wavefront at two antennas
separated by baseline `d`. That phase difference maps to an angle:

```
Δφ = (2π d / λ) · sin(angle from the baseline's normal)
```

`Δφ` is only observable modulo `2π`. So the mapping is unique **only if `d ≤ λ/2`.** At
UWB channel 5 (≈6.5 GHz), `λ ≈ 4.6 cm`, so `λ/2 ≈ 2.3 cm`.

**The parent plan's "~4–5 cm antenna spacing a nano-drone can actually fit" is therefore
ambiguous by roughly a factor of two** — it would alias two distinct arrival angles onto
the same phase reading. Real DW3000-class PDoA reference designs use ≈ λ/2 for exactly
this reason. Config default is `antenna_spacing_m: 0.023`, and the node **warns loudly at
startup if `antenna_spacing_m > lambda_m / 2`**, then models the resulting wrap (§2.3.5) so
that if someone deliberately chooses a wider baseline for accuracy, they see the cost.

#### 2.3.2 Array geometry

Two antennas give **one** angle. Two angles need **three**: an L-shaped planar array —
one pair on the body `y` baseline (→ azimuth), one on the body `z` baseline (→ elevation),
sharing a common antenna. Config `n_antennas: 3` (default, az+el) or `2` (az only,
elevation reported invalid). Say in the config comment that `3` implies a slightly larger
board than the retired 3.3 g Loco deck, which is already why `payload.yaml` carries
`uwb_pdoa_module` at a ⚠ placeholder 4.0 g.

#### 2.3.2b Coverage mode — single forward array vs. 360° (config toggle)

**Decision: build both behind one config switch, `array_mode: single | multi_array`,
default `single`.** This was raised as "just simulate different PDoA hardware for 360°
coverage" — that's the right instinct, and it's cheap enough in sim that there's no reason
not to make it a toggle you can A/B in Phase 2 rather than a one-way decision made now.

**`single` (default, matches §2.3.1–§2.3.3 above unchanged):** one array on `boresight_axis`,
forward cone of `aoa_fov_deg`, rear blind spot per §2.3.4. This is the physically
conservative case — it's what a single L-array of 3 antennas actually is.

**`multi_array`:** `n_arrays` (config, default 3) arrays spaced evenly around the yaw axis,
each an independent L-array with its own boresight `k`:

```
boresight_k = rotate(boresight_axis, yaw = k * 360/n_arrays, about body +z)   for k in 0..n_arrays-1
theta_k     = acos(clamp(dot(d_b / r, boresight_k), -1, 1))
k_best      = argmin_k theta_k              # peer is served by its nearest array
theta       = theta_k[k_best]
in_cone     = theta <= radians(aoa_fov_deg) / 2     # per-array cone, same aoa_fov_deg
```

Everything downstream (§2.3.3's `sigma_ang(theta)`, the `FLAG_IN_AOA_CONE` flag, the NaN
fallback outside cone) is **unchanged** — `multi_array` only changes which `theta` feeds
it, by picking whichever array is closest to boresight for that peer. With
`n_arrays * aoa_fov_deg ≥ 360°` (e.g. 3 × 130°, or 4 × 100°) every azimuth has bearing
coverage and the rear-blind-spot behaviour of §2.3.4 disappears; elevation is still capped
by each array's own baseline geometry, same as `single`.

**Cost this makes visible, not hides:** each array is its own antenna set — `multi_array`
scales `n_antennas` to `n_arrays * n_antennas_per_array` (config, default
`n_antennas_per_array: 3`, i.e. 9 antennas at `n_arrays: 3`), each with its own RF switch
leg and its own sampled antenna-delay bias per §2.2.1 extended to azimuth (§2.3.2c). This
is real mass and real complexity on a nano-drone — the config's `mass_g` comment must scale
accordingly (§4), and the node logs a WARNING at startup naming the effective antenna count
so nobody runs a `multi_array` sweep without seeing the hardware cost it implies.

**This is exactly the A/B you asked for:** run the M4 gate and any Phase-2 mesh-rigidity
eval once with `array_mode: single`, once with `multi_array`, same seed, same flight —
compare `rel_pos_err_mean_m` (§7.3 check D) and bearing-valid fraction directly. If
`multi_array` doesn't move the mesh-rigidity number enough to justify the antenna count,
that's the data-backed answer to "does 360° actually matter", and it's cheap to get because
nothing else in the model changes.

#### 2.3.2c Per-array antenna-delay bias (multi_array only)

Extend §2.2.1: with `array_mode: multi_array`, sample one **azimuth-delay bias**
`gauss(0, array_delay_bias_sigma_deg)` per `(device, array_index)` pair at startup, added to
`az_meas` before noise. Default `array_delay_bias_sigma_deg: 1.5`. Reason: physically,
inter-array phase-reference mismatch (different RF paths to `n_arrays` switch legs) is a
real, per-array constant offset — same category of error as §2.2.1's range bias, and
skipping it would make `multi_array` look artificially cleaner than `single` for reasons
that have nothing to do with 360° coverage. Set to `0.0` to disable.

#### 2.3.3 Angle noise — the off-boresight growth model

The effective aperture projects as `cos(θ)`, so angular resolution degrades as `1/cos(θ)`.
That is the physically motivated one-parameter form, and it is the default:

```
sigma_ang(theta) = radians(sigma_boresight_deg) / max(cos(theta), cos_floor)
cos_floor        = cos(radians(aoa_fov_deg) / 2)         # caps growth at the cone edge
az_meas = az + gauss(0, sigma_ang(theta))
el_meas = el + gauss(0, sigma_ang(theta))
```

`sigma_boresight_deg` default **8.0** — deliberately conservative. The ETH-PBL dual-antenna
characterisation the v3 roadmap cites reports ~2.4° mean within ±45° of boresight, but for
a **larger, tuned** module; budgeting 5–15° for what a nano-drone can carry is the parent
plan's own guidance and it is right. ⚠ **Tier B placeholder** until v3 Phase 1 item 1's
external-validation gate replaces it with a fitted form. Provide
`angle_error_model: inv_cos | linear | constant` so swapping in the validated form is a
config change, not a rewrite.

#### 2.3.4 The rear blind spot is a real design consequence — document it

With `boresight_axis: [1,0,0]` and `aoa_fov_deg: 100`, a peer more than 50° off the nose
gets **range-only**. That is not a modelling shortcut; it is what a single forward-facing
array does. It is also *why* the v3 roadmap has "yaw-glance / mount-orientation
coordination" as a Phase 7 squad behaviour. Two consequences to write down in the config
header so nobody is surprised in Phase 2:

- A drone flying down a corridor gets bearing to the peer ahead and range-only to the peer
  behind. The mesh is **anisotropic**.
- If Phase 2 finds the mesh under-constrained, the hardware answer is a second array
  (mass!) or the behavioural answer is scheduled yaw glances. Both are Phase-2+ decisions;
  M4's job is to make the asymmetry visible.

#### 2.3.5 Optional degradations — build the hooks, default them off

These are real and the v3 validation gate will want them. Implement behind config flags,
**default `false`**, and assert their behaviour in `--selftest` so they aren't rotted code:

- `model_phase_wrap` — when `antenna_spacing_m > lambda_m/2`, wrap the implied phase and
  report the aliased angle instead of the true one.
- `elevation_mirror_prob` — a planar array cannot distinguish `+el` from `−el` without a
  ground plane or a third baseline. With this probability, flip the sign of `el_meas` and
  clear nothing (the estimator must survive it via robust kernels). Default `0.0`.

### 2.4 CRITICAL: one exchange, one range — shared between both directions

A TWR exchange between `i` and `j` is **one physical transaction** producing **one**
round-trip time. Both drones learn the same distance.

**Therefore: draw the range noise ONCE per pair per scheduled exchange, and publish the
identical `r_meas` on both `/cf_i/uwb/edges` and `/cf_j/uwb/edges`.**

If you instead draw independent noise per direction, you have handed the estimator two
independent observations of the same quantity. Averaging them improves range variance by
√2 — a free accuracy gain that does not exist in reality, invisible in any plot, and
directly inflating the moonshot's headline result. This is the most damaging bug available
in this workstream. §8.1 check 6 asserts it, and §7.3 check B re-asserts it live.

**Bearings are the opposite:** `i`'s bearing to `j` and `j`'s bearing to `i` come from
*different antenna arrays* observing *different geometry*, so they get **independent** noise
draws. That independence is precisely what makes mutual-bearing yaw constraints observable
in Phase 2 — two drones each reporting a bearing to the other, and the consistency between
those two bearings constraining their relative yaw.

Likewise, LOS/NLOS status is a property of the **path**, so it is shared. Dropout is
shared: if the exchange fails, neither side gets an edge.

### 2.5 NLOS / multipath

```
occluded = los_check(p_i, p_j)                     # §5
if occluded:
    nlos_bias = abs(gauss(nlos_bias_mean_m, nlos_bias_sigma_m))    # always positive
    sigma_r  *= nlos_sigma_mult
    if uniform() < p_dropout_nlos:  drop the exchange entirely (no edge either side)
    if nlos_invalidates_bearing:    bearing flags cleared, az/el = NaN
```

**Bias is always positive.** A blocked direct path means the signal arrived via a longer
reflected route or a slower-through-material path; it can never arrive early. Take `abs()`
of the draw — do not use a signed Gaussian.

**`nlos_invalidates_bearing: true` by default.** Under NLOS the strongest arrival is a
reflection, so the measured angle of arrival points at the **reflector**, not the peer. A
bearing that is confidently wrong is far more corrosive to a pose graph than a missing
bearing. If you ever want it on, it must come with an angle error large enough to be
meaningless — at which point dropping it is equivalent and cheaper.

### 2.6 Link budget, neighbour selection, and the airtime scheduler

**Per tick** (`scheduler_tick_hz`, default 50):

1. **Candidate pairs.** All unordered pairs of *active devices* (drones with fresh odom,
   plus static peers) whose true range `< max_range_m`.
2. **Range-dependent dropout** (link budget) — independent of NLOS:
   `p_drop = p_dropout_at_max_range * (r / max_range_m)**2`, clipped to `[0,1]`. Inverse-
   square is the right shape for received power. Default `p_dropout_at_max_range: 0.5`.
3. **Neighbour cap.** Each device keeps only its `max_neighbors_per_drone` nearest
   candidates; a pair survives only if **both** ends keep it. (Real systems select on link
   quality, which correlates with range — note in the config that using true range here is
   a sim-oracle shortcut, acceptable because the alternative is modelling RSSI we have no
   data for.)
4. **Airtime.** `n_pairs = len(scheduled_pairs)`;
   `effective_rate = min(ranging_rate_hz, max_exchanges_per_s / max(n_pairs, 1))`.
   Each pair maintains a `next_due_time`; a pair fires when `t >= next_due`, then
   `next_due += 1 / effective_rate`. Round-robin falls out naturally; cap the number of
   exchanges executed in one tick at `ceil(max_exchanges_per_s / scheduler_tick_hz)` so a
   backlog cannot burst.
5. **Execute** each fired exchange per §2.2–2.5, appending one edge row to *each*
   participant's outgoing buffer.
6. **Publish** every observer's buffer, even if empty (heartbeat — a consumer must be able
   to distinguish "no neighbours in range" from "node died").

Log `n_pairs` and `effective_rate` to MLflow and print them once at startup and whenever
`n_pairs` changes. When someone later asks "why is my mesh only updating at 0.4 Hz", the
answer must already be in the log.

---

## 3. Node spec

### 3.1 Files

```
perception/uwb_sim/uwb_model.py   # pure maths + scheduler + LOS. NO rclpy import. --selftest lives here.
perception/uwb_sim/uwb_edges.py   # pack_edges() / unpack_edges() PointCloud2 helper. NO rclpy-specific logic beyond msg types.
perception/uwb_sim/uwb_node.py    # rclpy wrapper: subscriptions, timer, publishers, MLflow
eval_scripts/uwb_gate.py          # M4 exit gate
configs/sensors/uwb_pdoa.yaml     # all Tier B knobs
```

Plain scripts, **no `package.xml` / colcon package** — run with `python3` after sourcing
`setup_env.sh`, exactly like `eval_scripts/*.py` and (per the M3b plan)
`perception/flow_sim/flow_node.py`. `perception/` is the mapped home for sensor code
(`AGENTS.md §2`); `eval_scripts/` is for gates.

The split is load-bearing: `uwb_model.py` must be importable and fully testable with **no
ROS and no Gazebo**. Write it first (§10).

### 3.2 Inputs

| Topic | Type | Source | Use |
|---|---|---|---|
| `/cf_<id>/odom` for each `id` in `0..N-1` | `nav_msgs/msg/Odometry` | bridged from `gz.msgs.Odometry` (`gz-sim-odometry-publisher-system`, `model.sdf.jinja:302-308`, 200 Hz) | ground-truth position + orientation per drone |

Use `odom.pose` only. **Do not use `odom.twist`** — its frame convention (body vs world) is
not documented anywhere we can verify and `AGENTS.md §6.4` bans guessing. M4 needs no
velocity anyway (§1.6). If a future version does, copy M3b's pose-differencing approach.

A drone whose odom is older than `odom_stale_s` (default 0.5 s) is dropped from the active
set for that tick, and a WARN is logged once per transition. Do **not** coast on a stale
pose — a dead drone must disappear from the mesh, which is exactly the "drone-down"
condition the coordination layer cares about later.

### 3.3 Static peers

From config (§4). Each entry: `id`, `type` (`entrance` | `landed`), `position_xyz_m`,
`yaw_deg`. They participate in the model identically to drones — same range noise, same
antenna-delay bias, same NLOS check, same scheduler slot — and they publish their own
edge topic at `/uwb/<id>/edges` (they observe too).

**The distinction that matters is a flag, not different maths:**

- `type: entrance` → sets `FLAG_PEER_IS_SURVEYED`. Its pose **is** a legitimate known
  constant; downstream may treat it as the gauge.
- `type: landed` → **does not** set that flag. It is a drone that landed at a drifted
  estimated pose. `AGENTS.md §1 Tier A`: its position is a state variable. The config
  position is a **sim oracle** for generating measurements, and the estimator must never
  read this file. Put that warning in the config header in those words.

### 3.4 Output: `/cf_<id>/uwb/edges` — `sensor_msgs/msg/PointCloud2`

One message per observer per scheduler tick. One "point" per edge. `height = 1`,
`width = n_edges`, `is_dense = false`, `is_bigendian = false`, `point_step = 48`,
`row_step = 48 * width`. `header.stamp` = the newest odom stamp consumed this tick;
`header.frame_id` = `cf_<id>/base_link`.

Fields, in order (all offsets are `4 * index`; every field is 4 bytes):

| # | name | datatype | meaning |
|---|---|---|---|
| 0 | `x` | `FLOAT32` (7) | peer position in observer body frame, `r_meas * cos(el)*cos(az)`. **NaN** if bearing invalid |
| 1 | `y` | `FLOAT32` | `r_meas * cos(el)*sin(az)`. NaN if bearing invalid |
| 2 | `z` | `FLOAT32` | `r_meas * sin(el)`. NaN if bearing invalid |
| 3 | `observer_id` | `UINT32` (6) | who measured it (redundant with topic, needed for `/uwb/edges_all`) |
| 4 | `peer_id` | `UINT32` | who was measured |
| 5 | `range_m` | `FLOAT32` | `r_meas`, §2.2. **Always valid** if the row exists |
| 6 | `azimuth_rad` | `FLOAT32` | §2.3, or NaN |
| 7 | `elevation_rad` | `FLOAT32` | §2.3, or NaN |
| 8 | `sigma_range_m` | `FLOAT32` | reported σ, §2.2 (excludes the antenna bias — deliberate) |
| 9 | `sigma_az_rad` | `FLOAT32` | `sigma_ang(theta)`, or NaN |
| 10 | `sigma_el_rad` | `FLOAT32` | as above, or NaN |
| 11 | `flags` | `UINT32` | bitfield below |

`x,y,z` first and named exactly that so **RViz's PointCloud2 display renders the mesh with
no extra work**. Rows with NaN xyz are skipped by RViz — i.e. range-only edges are
invisible in RViz. That is a known, acceptable limitation; note it in the docstring.

Flags bitfield (define as module constants in `uwb_edges.py`):

```
FLAG_RANGE_VALID     = 0x01   # set on every published row (a row without range is not published)
FLAG_BEARING_VALID   = 0x02   # az/el present and meaningful
FLAG_LOS             = 0x04   # set = line of sight; clear = NLOS
FLAG_IN_AOA_CONE     = 0x08   # peer within aoa_fov_deg/2 of boresight
FLAG_PEER_IS_SURVEYED= 0x10   # peer is the fixed entrance node (pose is a known constant)
FLAG_PEER_IS_STATIC  = 0x20   # peer is not moving (entrance OR landed drone)
```

**Dropouts produce no row at all** — that is the difference between "measured badly"
(row present, flags/σ tell you how badly) and "not measured" (absent). Do not publish a
row with `FLAG_RANGE_VALID` clear; there is no such thing.

Also publish **`/uwb/edges_all`** (same layout, all observers concatenated) when
`publish_aggregate: true` (default). It is what `uwb_gate.py` and any swarm-wide
visualiser subscribe to, and it costs one extra `pack_edges` call.

QoS: `SensorDataQoS()` (best-effort, depth 10) on all publishers and subscriptions — the
odom bridge publishes best-effort.

### 3.5 `uwb_edges.py` contract

```python
pack_edges(edges, stamp, frame_id) -> PointCloud2
    # edges: list of dicts or a numpy structured array with the 12 fields above

unpack_edges(msg) -> numpy structured array with the same 12 named fields
    # dtype: [('x','<f4'), ..., ('flags','<u4')]  -- one np.frombuffer, no Python loop
```

Implement `unpack_edges` as a single `np.frombuffer(msg.data, dtype=EDGE_DTYPE)` against a
module-level `EDGE_DTYPE` built to match `point_step = 48`. Then **assert** at import time
that `EDGE_DTYPE.itemsize == 48` and that the generated `PointField` offsets match
`EDGE_DTYPE.fields` — a two-line guard that makes a layout mistake impossible to ship.

Provide `EDGE_DTYPE`, the six `FLAG_*` constants, and a `describe(msg) -> str` pretty-printer
for debugging. Add a module docstring showing a complete five-line consumer example; that
docstring is the API document the teammate will actually read.

### 3.6 `/uwb/edges_truth` — the sim oracle

Published only when `publish_ground_truth: true` (default `true`). Same PointCloud2 layout,
same rows, but populated with **noise-free, dropout-free** `range/az/el` for every candidate
pair, and `sigma_* = 0`.

⚠ **`AGENTS.md §1 Tier A`: this is a sim oracle. Nothing in the estimator, the RIO
front-end, or any Phase-2+ code may subscribe to it — it exists so `uwb_gate.py` can score
without re-deriving geometry.** That warning, in those words, must appear in **three
places**: the node docstring, the config comment on the flag, and a
`get_logger().warn()` fired once at startup when the flag is on.

### 3.7 CLI and startup

```
python3 perception/uwb_sim/uwb_node.py \
    [--config configs/sensors/uwb_pdoa.yaml] \
    [--num-drones 2] [--seed 0] [--no-truth] [--no-mlflow] [--selftest]
```

`--selftest` delegates to `uwb_model.py` (§8.1): **no `rclpy.init()` anywhere on that
path**, exit 0/1.

At startup log exactly one INFO block: resolved config path, seed, `N` drones, static peer
list with types, `lambda_m` and the λ/2 check verdict, `boresight_axis`, `aoa_fov_deg`,
`n_pairs`, `effective_rate`, and the sampled antenna-delay bias table. Every one of these
has caused a confusing gate failure in some project; printing them costs nothing.

MLflow (`sqlite:///mlflow.db`, experiment from config): log the params above plus periodic
`per_tick_ms` (mean and p95 over the last 100 ticks) — that number is what settles §1.1's
revisit trigger. Guard the whole MLflow block in `try/except` and never let it fail the
node (`AGENTS.md §4`: `mlflow.db` on `/mnt/d` can wedge).

---

## 4. `configs/sensors/uwb_pdoa.yaml`

```yaml
# Inter-drone UWB ranging + PDoA bearing — Phase 1 M4.
# See .cursor/docs/M4_UWB_Relative_Positioning_Implementation_Plan.md
#
# Simulated analytically by perception/uwb_sim/uwb_node.py. There is no native
# Gazebo UWB sensor; analytic modelling is the correct and standard approach.
#
# ARCHITECTURE (v3): no anchor pucks are dropped inside the structure — we are
# not manufacturing pucks. Physical reference points, if we add them, are either
# (a) drones that LAND and act as static nodes, or (b) MiFly-style passive
# retroreflective tags (plan doc §12, not built). The ONLY node whose pose is a
# legitimate known constant is the fixed entrance/base-station gauge.
#
# ⚠ AGENTS.md §1 Tier A — SIM ORACLE WALL: the `static_peers` positions below are
# how the SIMULATOR generates measurements. The estimator must NEVER read this
# file. A `landed` peer's position is a STATE VARIABLE: it landed at a drifted
# estimated pose. Only `type: entrance` is a surveyed constant.
#
# FIDELITY (AGENTS.md §5): range geometry + noise are high fidelity. Angle
# fidelity is only as good as the placeholder off-boresight model below, which
# stands until v3 roadmap Phase 1 item 1's external-validation gate replaces it.
# Do not claim angle accuracy outside a validated envelope, and do not claim
# anything about a specific unbuilt PDoA module until Phase 11 bench data exists.
# UNMODELLED: TWR clock drift / carrier frequency offset; inter-radio chirp
# interference (Phase 7); material-specific attenuation.

# ── radio / array geometry ────────────────────────────────────────────────────
channel_centre_hz: 6.5e9      # UWB ch5. lambda = c / this = ~0.0461 m
antenna_spacing_m: 0.023      # ~lambda/2. > lambda/2 is PHASE AMBIGUOUS — plan §2.3.1.
                              # The node WARNS at startup if this exceeds lambda/2.
n_antennas: 3                 # 3 = L-array -> azimuth + elevation. 2 = azimuth only.
                              # (single mode only — multi_array derives antenna count from
                              # n_arrays * n_antennas_per_array below.)
boresight_axis: [1.0, 0.0, 0.0]   # body FLU. Forward-facing => REAR BLIND SPOT for
                              # bearing (range still works). Plan §2.3.4 — this is a
                              # real design consequence, not a modelling shortcut.
aoa_fov_deg: 100.0            # full cone angle (per array); outside it -> range-only fallback

# ── coverage mode (plan §2.3.2b) — A/B this before committing hardware ─────────
array_mode: single            # single | multi_array
                              # single:      one forward L-array, rear blind spot (§2.3.4).
                              # multi_array: n_arrays L-arrays spaced evenly around yaw,
                              #              360 deg bearing coverage if n_arrays*aoa_fov_deg
                              #              >= 360. Real antenna/mass/RF-switch cost scales
                              #              with n_arrays — see n_antennas_per_array below.
n_arrays: 3                   # multi_array only. 3x130deg or 4x100deg both give full 360.
n_antennas_per_array: 3       # multi_array only. Effective antenna count = n_arrays * this.
array_delay_bias_sigma_deg: 1.5   # multi_array only. Per-(device,array) constant azimuth
                              # bias from inter-array RF-path mismatch (plan §2.3.2c).
                              # Set 0.0 to disable. Does NOT apply to single mode.

# ── range noise (Tier B) ──────────────────────────────────────────────────────
sigma_range_los_m: 0.10       # DW-class TWR, ~+/-10 cm
antenna_delay_bias_sigma_m: 0.05   # PER-DEVICE CONSTANT offset, sampled once at startup.
                              # Pair bias = bias_i + bias_j. NOT included in the reported
                              # sigma — the estimator is meant to have to deal with it
                              # (plan §2.2.1). Set 0.0 to disable.
max_range_m: 30.0
p_dropout_at_max_range: 0.5   # link-budget dropout, scales as (r/max_range)^2

# ── bearing noise (Tier B, ⚠ PLACEHOLDER until externally validated) ──────────
angle_error_model: inv_cos    # inv_cos | linear | constant
sigma_boresight_deg: 8.0      # conservative for a nano-scale array. ETH-PBL report ~2.4 deg
                              # within +/-45 deg for a LARGER tuned module — do not adopt
                              # that number for a Crazyflie-sized board without bench data.

# ── NLOS / multipath ──────────────────────────────────────────────────────────
nlos_bias_mean_m: 0.35        # always POSITIVE (abs of the draw) — a blocked path is longer
nlos_bias_sigma_m: 0.25
nlos_sigma_mult: 3.0          # sigma_range inflation under NLOS
p_dropout_nlos: 0.35
nlos_invalidates_bearing: true  # plan §2.5 — a multipath AoA points at the REFLECTOR

# ── occlusion / LOS test (plan §5) ────────────────────────────────────────────
los_model: boxes              # boxes | mesh | always_los
occluder_boxes: []            # list of {name, x_min, x_max, y_min, y_max, z_min, z_max}
                              # e.g. a wall for the NLOS selftest:
                              # - {name: wall, x_min: 0.9, x_max: 1.1, y_min: -5.0,
                              #    y_max: 5.0, z_min: 0.0, z_max: 3.0}
mesh_path: ""                 # relative to SAR_NANO_SWARM_ROOT; used when los_model: mesh.
                              # Same .obj the radar plugin raycasts, e.g.
                              # sim_worlds/darpa_subt_worlds/worlds/models/cave_world/meshes/cave_world.obj

# ── schedule / airtime (plan §1.3, §2.6) ──────────────────────────────────────
ranging_rate_hz: 10.0         # per-pair rate with the channel to itself
max_exchanges_per_s: 400      # TOTAL channel budget shared by the whole swarm.
                              # At N=50 (1225 pairs) this is ~0.33 Hz/pair — that limit is
                              # REAL and must stay visible. Do not raise it to make a gate pass.
max_neighbors_per_drone: 6
scheduler_tick_hz: 50.0

# ── static peers ──────────────────────────────────────────────────────────────
# type: entrance -> surveyed constant, the mesh gauge (FLAG_PEER_IS_SURVEYED)
# type: landed   -> a drone that landed. Position here is SIM ORACLE ONLY.
static_peers:
  - {id: 1000, type: entrance, position_xyz_m: [-2.0, 0.0, 0.30], yaw_deg: 0.0}

# ── misc ──────────────────────────────────────────────────────────────────────
odom_stale_s: 0.5
publish_aggregate: true
publish_ground_truth: true    # /uwb/edges_truth — SIM ORACLE, AGENTS.md §1 Tier A.
                              # The estimator must NEVER subscribe to it.
model_phase_wrap: false       # plan §2.3.5 — hook built, off by default
elevation_mirror_prob: 0.0    # plan §2.3.5 — hook built, off by default
seed: 0

mlflow_experiment: "phase1_uwb_pdoa"
```

---

## 5. LOS / occlusion — three tiers, and the default needs no dependencies

`los_check(p_a, p_b) -> bool` (True = clear line of sight). Selected by `los_model`:

### 5.1 `boxes` (DEFAULT)

Segment vs axis-aligned box, **slab method**, pure numpy, zero dependencies, fully
deterministic, works in any world including the geometry-free `phase1_pid_tune`. Occluded
if the segment intersects **any** configured box.

```
d = b - a
for each slab axis k in (x,y,z):
    if |d[k]| < eps:  if a[k] < min[k] or a[k] > max[k]: no intersection; else skip axis
    else: t0 = (min[k]-a[k])/d[k]; t1 = (max[k]-a[k])/d[k]; if t0>t1: swap
          t_enter = max(t_enter, t0); t_exit = min(t_exit, t1)
intersects = (t_enter <= t_exit) and (t_exit >= 0.0) and (t_enter <= 1.0)
```

Initialise `t_enter = 0.0`, `t_exit = 1.0` so `t` is already normalised to the segment.
Vectorise over pairs × boxes; at realistic sizes it is microseconds.

**This is the tier the gate uses** — deterministic, world-independent, and it makes the
NLOS behaviour testable entirely offline (§8.1 checks 8–10).

### 5.2 `mesh` (upgrade path, optional dependency)

Load the **same `.obj` the radar plugin raycasts** (`RadarSensorSystem.cpp:25`,
`rm::import_embree_map(meshPath)`) with `trimesh`, backed by `embreex` for speed. Cast from
`a` toward `b`, take the first hit, occluded if `hit_distance < ||b − a|| − surface_eps_m`
(the epsilon prevents a drone sitting on the floor from occluding itself).

⚠ `trimesh` is **not** in `requirements.txt` and was not importable when this plan was
written. So:
- Add `trimesh` and `embreex` to `requirements.txt` under an `# Optional:` comment, like
  the existing `# mujoco` line.
- Import them **lazily, inside the `mesh` branch only**, wrapped in `try/except ImportError`.
- On failure or a missing/unreadable `mesh_path`: log a **WARNING naming the fallback** and
  degrade to `always_los`. Never crash, and never silently pretend the mesh loaded.
- `AGENTS.md §6.4`: verify the intersector API against the installed version before use —
  `python3 -c "import trimesh; help(trimesh.ray.ray_pyembree.RayMeshIntersector.intersects_location)"`
  — do not assume the signature from memory.
- Cache the loaded mesh and intersector at first use; never reload per tick.

### 5.3 `always_los`

Everything is LOS. Ablation baseline and the degradation target. Log a WARNING at startup
that NLOS is disabled, so no result is ever reported under it by accident.

---

## 6. Multi-drone launch — **this is the real first task, and the parent plan omits it**

`phase0_gate.sh` hard-codes `CF_ID=0` (line ~251) and spawns exactly one drone. **M4's gate
needs two.** Nothing else in M4 can be tested until this exists, so build it first.

CrazySim already ships the pattern — read
`firmware_mods/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_text.sh`
and mirror it rather than inventing conventions. Verified from that file:

- per-drone SDF at `/tmp/<MODEL>_<N>.sdf`
- `--cflib_udp_port $((19850+N))`, `--cffirm_udp_port $((19950+N))`, `--cf_id N`
- per-drone working dir `$BUILD_DIR/$N` (must exist before `pushd`)
- one `cf2 $((19950+N))` process per drone, each `> out.log 2> error.log`

### 6.1 Changes to `phase0_gate.sh`

**(a) Flags.** Add `-n | --num-drones N` (default 1), `--spacing M` (default 1.5, metres
between spawn points), `--no-uwb`, `--uwb-config PATH`. Update the header comment block
(~lines 14–37) and `usage()` (~lines 57–76) — both, they are separate text.

**(b) Loop the per-drone block.** Everything from "generate Crazyflie SDF" through "start
SITL firmware" (lines ~250–371) becomes a `for CF_ID in $(seq 0 $((NUM_DRONES-1)))` loop.
Inside, unchanged except:

- `SDF_TMP="/tmp/${MODEL}_${CF_ID}.sdf"`, ports as above (already `CF_ID`-derived — good).
- Spawn position: lay drones out on a line, `x = SPAWN_X + CF_ID * SPACING`, `y = SPAWN_Y`.
  **They must not overlap** or the physics engine will fight and no gate will ever pass.
- `apply_payload.py` / `apply_tof_sensor.py` already take `--cf-id`; pass `$CF_ID`.
- Run `thrust_margin_check.py` **only for `CF_ID == 0`** — it is identical for every drone
  and would otherwise write N duplicate MLflow runs per launch.
- Keep the existing "wait for `/cf_<id>/odom`" readiness poll **inside** the loop, before
  starting that drone's `cf2`. The comment at line ~343 explains why (starting `cf2` early
  drops the first IMU packets and the estimator never recovers). That hazard is per-drone.

**(c) Bridge N drones.** Extend the single `parameter_bridge` invocation (lines ~381–389) to
build its argument list in a loop — one process, `2N` topic mappings:

```bash
_bridge_args=()
for i in $(seq 0 $((NUM_DRONES-1))); do
  _bridge_args+=( "/cf_${i}/tof_down@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" )
  _bridge_args+=( "/cf_${i}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry" )
done
ros2 run ros_gz_bridge parameter_bridge "${_bridge_args[@]}" &
```

(The M3b plan adds the odom mapping for `cf_0`; if M3b landed first, this generalises it —
do not create a second bridge process.)

**(d) Launch the UWB node** once, after the bridge, before RViz:

```bash
if [[ "$USE_UWB" == true ]]; then
  _uwb_cfg="${UWB_CONFIG:-$SAR_NANO_SWARM_ROOT/configs/sensors/uwb_pdoa.yaml}"
  [[ "$_uwb_cfg" != /* ]] && _uwb_cfg="$SAR_NANO_SWARM_ROOT/$_uwb_cfg"
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
```

`python3 -u` is **required** — `AGENTS.md §4`: buffered stdout is lost when the trap kills
the process, and this repo has already been bitten by it.

**(e) Banner.** Add `Drones : ${NUM_DRONES}`, `UWB : ${USE_UWB}`,
`UWB topic : /cf_<id>/uwb/edges`, and print every drone's cfclient URI, not just 19850.

### 6.2 Multi-drone hazards to expect

- **`gz sim` holds the UDP ports `cf2` needs** (`AGENTS.md §4`). With N drones there are 2N
  ports; a single stale `gz sim` blocks all of them. Always kill `gz sim`, not just `cf2`,
  between runs.
- **CRTP v7 one-shot arming** (`hover_gate.py` docstring): this build auto-arms at sim boot
  but cannot be reliably re-armed by cflib after a disconnect. With two drones this means
  **connect both, in the same script, as the first cflib connections against a freshly
  launched sim.** `uwb_gate.py` must open both `SyncCrazyflie` contexts before flying
  either (§7.2).
- **Startup cost.** Two SITL instances plus two sensor sets in a heavy world is slow;
  prefer `phase1_pid_tune` + `--no-radar --headless` for the gate.
- If drone 1 never arms, check `$BUILD_DIR/1/error.log` before anything else.

---

## 7. Milestone breakdown

**Status (2026-07-30):** M4-0…M4-3 ✅ complete; M4-4 not run.

| Milestone | Content | Status |
|---|---|---|
| **M4-0** | Multi-drone launch (§6). Two drones hover simultaneously; both `/cf_<id>/odom` publish. | ✅ DONE |
| **M4-1** | `uwb_model.py` + `uwb_edges.py` + `--selftest` all green (§8.1). No ROS, no sim. | ✅ DONE (17/17) |
| **M4-2** | `uwb_node.py` live; `/cf_0/uwb/edges` + `/cf_1/uwb/edges` + `/uwb/edges_all` publishing; RViz shows the mesh. | ✅ DONE |
| **M4-3** | `uwb_gate.py` passes (§7.3 checks A–G). **v3 Phase-0 UWB exit criterion.** | ✅ DONE (12/12) |
| **M4-4** | *(control arm, low priority)* Landed peers + mesh LOS in cave world. | ⏸ NOT RUN |
| **M4-5** | *(optional, informs a hardware decision, not gated)* `array_mode: single` vs `multi_array` A/B — same seed, same flight, compare `rel_pos_err_mean_m` (§7.3 check D) and bearing-valid fraction. | ⏸ NOT RUN |

M4-4 is **not** a separate codebase — it is `uwb_pdoa.yaml` with extra `static_peers` and a
different `los_model`. That is the payoff of §1.5's "one node, one model" decision. Use
**floor-only / coplanar** peer layouts (`AGENTS.md §5`): landed drones are on the floor, so
vertical DOP is genuinely bad, and idealised wall/ceiling placements flatter the result.

M4-5 is likewise just a config swap (§2.3.2b) — no code branch beyond the theta selection.
Run it once M4-3 is green; the result answers "does 360-degree coverage move the needle
enough to justify the antenna/mass cost", with a number to decide it rather than an
intuition.

### 7.1 Offline selftest — see §8.1. Write it first.

### 7.2 Live smoke test

```bash
# terminal 1
./eval_scripts/phase0_gate.sh -w phase1_pid_tune -n 2 --spacing 2.0 --no-radar --headless
# terminal 2 (setup_env.sh sourced)
ros2 topic hz /cf_0/uwb/edges          # expect ~scheduler_tick_hz (50), most ticks empty
ros2 topic echo /uwb/edges_all --once
python3 -c "
import rclpy, sys; sys.path.insert(0,'perception/uwb_sim')
from uwb_edges import unpack_edges, describe
"                                       # helper imports cleanly (asserts itemsize==48)
```

On the ground, before takeoff: both drones sit at z≈0.5 spawn, 2 m apart. Expect edges
with `range_m ≈ 2.0`, `FLAG_LOS` set (no occluders configured), bearing valid for whichever
drone has the other within its forward cone. **Drones spawn facing +x, so drone 0 sees
drone 1 dead ahead (az≈0) and drone 1 sees drone 0 directly behind (az≈±π) → outside a 100°
cone → range-only.** That asymmetry is correct (§2.3.4) and is the first thing to confirm
rather than debug.

### 7.3 `eval_scripts/uwb_gate.py` — the M4 exit gate

An **rclpy node** (the topics are ROS-native), spun on a background
`MultiThreadedExecutor` thread while the cflib flight sequence runs on the main thread.
Reuse the proven patterns verbatim: `pid_gains.load_gains/apply_gains/reset_estimator/
reset_pose`, the `SyncCrazyflie` + `MotionCommander` structure from `tof_gate.py:151-243`,
the `SIGALRM` watchdog (`_Timeout`/`_alarm`), and the `warnings.filterwarnings` calls that
silence cflib's CRTP-v7 deprecation spam.

Subscribe to `/uwb/edges_all` and `/uwb/edges_truth`; buffer `(stamp, unpack_edges(msg))`
from each; pair measured and truth rows by `(observer_id, peer_id)` within a 50 ms stamp
window.

**Flight sequence** (`phase1_pid_tune`, `-n 2 --spacing 2.0 --no-radar --headless`).
Open **both** `SyncCrazyflie` contexts first (§6.2), then:

| Leg | Action | Window |
|---|---|---|
| 0 | both drones take off to 0.5 m, settle 3 s | — |
| 1 | both hold 6 s | **A: static LOS** |
| 2 | drone 0 `mc.turn_left(150)`, settle 2 s, hold 5 s | **B: out-of-cone** |
| 3 | drone 0 `mc.turn_right(150)` back, settle 2 s; drone 1 `mc.up(0.6)`, hold 5 s | **C: elevation** |
| 4 | both `mc.stop()` | — |

Leg 2 tests the AoA-cone fallback with **yaw only** — no translation, so nothing can drift
into a wall and the geometry stays exactly known. Leg 3 puts a real non-zero elevation on
the edge so `el` is actually exercised rather than sitting at 0.

**Pass criteria** (thresholds as module constants, like `hover_gate.py:35-36`):

| # | Check | Threshold |
|---|---|---|
| A1 | edge rate for pair (0,1) in window A | within ±20 % of `effective_rate` from §2.6 |
| A2 | range error `\|range_m − truth_range\|`, mean over window A | ≤ 0.25 m (covers 1σ noise + the antenna-delay bias, which is deliberately unreported) |
| A3 | range error **std** over window A | ≤ 2 × `sigma_range_los_m` (catches a bias masquerading as noise, and vice versa) |
| **B** | **bidirectional range identity**: for exchanges present on both `0→1` and `1→0` in the same tick, `range_m` is **bit-identical** | 100 %. §2.4 — this is the check that catches the most damaging bug in the workstream |
| C1 | window A, drone 0 → drone 1: `FLAG_BEARING_VALID` set | ≥ 90 % of rows |
| C2 | window A, azimuth error vs truth (valid rows) | RMS ≤ 3 × `sigma_boresight_deg` |
| C3 | window B (drone 0 yawed 150°): `FLAG_BEARING_VALID` **clear** and `FLAG_IN_AOA_CONE` **clear**, while `FLAG_RANGE_VALID` stays set and `range_m` remains accurate | ≥ 90 % of rows — *the edge degrades, it does not disappear* |
| C4 | window B: `azimuth_rad`, `elevation_rad`, `sigma_az_rad`, `sigma_el_rad`, `x`, `y`, `z` are all **NaN** | 100 % — no silent truth pass-through |
| **D** | **relative position**: reconstruct `p̂_j = p_i_true + R_i_true · (x,y,z)` from bearing-valid rows; error vs `p_j_true` | mean ≤ 0.45 m in window A. *This is the deliverable — "relative positioning works" is this number.* |
| E1 | window C: mean `elevation_rad` matches truth sign and magnitude | error ≤ 3 × `sigma_boresight_deg` |
| F | entrance node (`id: 1000`) observed by both drones, `FLAG_PEER_IS_SURVEYED` set, range error within A2's bound | — |
| G | no drone diverged, no `SIGALRM` timeout, both took off | — |

NLOS is **not** in the live gate — it is fully deterministic geometry and is covered
offline (§8.1 checks 8–10), which is faster, more reproducible, and does not need a wall
in the flight world. Say so in the gate docstring so its absence reads as a decision.

**MLflow** (`sqlite:///mlflow.db`, experiment from config, run name `uwb_gate`): params
`seed`, `num_drones`, `ranging_rate_hz`, `max_exchanges_per_s`, `sigma_range_los_m`,
`sigma_boresight_deg`, `aoa_fov_deg`, `los_model`, every threshold; metrics
`edge_rate_hz`, `range_err_mean_m`, `range_err_std_m`, `bidir_identical_frac`,
`bearing_valid_frac_A`, `az_rms_deg`, `bearing_valid_frac_B`, `nan_frac_B`,
`rel_pos_err_mean_m`, `el_err_deg`, `entrance_seen`, `gate_pass`. Exit `0`/`1`.

---

## 8. Verification

### 8.1 `--selftest` — no ROS, no Gazebo. Write this before the node.

All against a fixed seed with an explicit config dict, so every number is reproducible.

1. **Geometry, dead ahead.** Observer at origin, identity attitude, peer at `(2,0,0)`
   ⇒ `r=2.0`, `az=0`, `el=0`, `theta=0`, `in_cone=True`.
2. **Azimuth sign.** Peer at `(0,2,0)` ⇒ `az = +π/2` (**left is positive** — FLU). Peer at
   `(0,-2,0)` ⇒ `az = −π/2`. Getting this backwards silently mirrors the entire mesh.
3. **Elevation sign.** Peer at `(0,0,2)` ⇒ `el = +π/2` (**up is positive**).
4. **Observer yaw.** Observer yawed +90° (facing `+y`), peer at world `(2,0,0)`
   ⇒ `az = −π/2`, `r = 2.0`. Confirms `R^T`, not `R`.
5. **Cone fallback.** `aoa_fov_deg=100`, peer at `(-2,0,0)` (θ=180°) ⇒ `in_cone=False`,
   `FLAG_BEARING_VALID` clear, `FLAG_RANGE_VALID` **set**, `az/el/sigma_az/sigma_el/x/y/z`
   all NaN, `range_m` finite and near truth.
6. **Bidirectional range identity.** Run one scheduler tick with 2 drones; assert the row
   on observer 0 and the row on observer 1 carry **exactly equal** `range_m` (compare with
   `==`, not `isclose`), and that their `azimuth_rad` values are **not** equal (independent
   draws, §2.4).
7. **Antenna bias is a constant.** With `sigma_range_los_m = 0`, run 100 exchanges for one
   pair ⇒ `std(range_m) == 0` and `mean(range_m) − truth == bias[i] + bias[j]` exactly.
   Then with `antenna_delay_bias_sigma_m = 0` ⇒ `mean(range_m) − truth ≈ 0` within 3 σ/√100.
8. **Occlusion, boxes.** Wall box spanning `x∈[0.9,1.1]`, endpoints `(0,0,1)` and `(2,0,1)`
   ⇒ occluded. Endpoints `(0,0,1)` and `(0,2,1)` (both on the near side) ⇒ clear. Endpoints
   `(0,0,5)` and `(2,0,5)` with the box capped at `z=3` ⇒ clear (over the top).
9. **NLOS effects.** With that wall and `p_dropout_nlos=0`: `FLAG_LOS` clear,
   `mean(range_m) > truth` over 200 draws (positive bias), `sigma_range_m ==
   sigma_range_los_m * nlos_sigma_mult`, and `FLAG_BEARING_VALID` clear (default
   `nlos_invalidates_bearing`).
10. **NLOS dropout.** `p_dropout_nlos=1.0` ⇒ **zero** rows for that pair on **both**
    observers (dropout is shared, §2.4).
11. **Off-boresight σ growth.** `sigma_ang(0) == radians(sigma_boresight_deg)`;
    `sigma_ang(60°) ≈ 2 ×` that; `sigma_ang(θ > fov/2)` is capped at the `cos_floor` value,
    not infinite.
12. **Airtime.** 10 devices, `max_neighbors_per_drone=9`, `ranging_rate_hz=100`,
    `max_exchanges_per_s=90` ⇒ 45 pairs ⇒ `effective_rate == 2.0 Hz`. Run 10 simulated
    seconds of ticks and assert total exchanges is within 5 % of `45 × 2 × 10 = 900`.
13. **Neighbour cap.** 10 devices in a line, `max_neighbors_per_drone=2` ⇒ the scheduled
    pair set contains only pairs where each end is within the other's 2 nearest.
14. **Link-budget dropout.** At `r = max_range_m`, empirical drop fraction over 2 000 draws
    ≈ `p_dropout_at_max_range` within 5 %; at `r = max_range_m/2`, ≈ a quarter of it.
15. **Pack/unpack round trip.** `unpack_edges(pack_edges(edges,...))` reproduces every
    field exactly (NaNs compare via `np.isnan`), and `EDGE_DTYPE.itemsize == 48`.
16. **λ/2 warning.** `antenna_spacing_m = 0.05` at 6.5 GHz triggers the ambiguity warning;
    `0.023` does not.
17. **Determinism.** Two full runs with the same seed produce byte-identical edge arrays;
    different seeds do not.
18. **`multi_array` coverage.** `array_mode: multi_array`, `n_arrays=3`, `aoa_fov_deg=130`
    (>360/3, so full coverage). Place a peer at 12 evenly-spaced azimuths around the
    observer (every 30°, including azimuths a `single`-mode rear cone would drop, e.g.
    180°) ⇒ `FLAG_BEARING_VALID` set at **every** azimuth, and `k_best` (or the resulting
    `theta`) matches whichever configured array boresight is nearest. Re-run the same 12
    points with `array_mode: single` ⇒ only the ones within `aoa_fov_deg/2` of
    `boresight_axis` keep `FLAG_BEARING_VALID`; confirms the toggle actually changes
    behaviour and `single` still reproduces §8.1 check 5's cone fallback unchanged.
19. **`multi_array` antenna count logged.** With `n_arrays=3, n_antennas_per_array=3`,
    startup log / returned config summary reports effective antenna count `9`, and the
    per-`(device, array)` delay-bias table has `n_arrays` entries per device (vs. one
    range-bias scalar per device from check 7 — these are independent tables).

### 8.2 Live smoke test — §7.2.
### 8.3 Live gate — §7.3.

---

## 9. Gotchas

1. **§2.4 — one exchange, one range.** Read it again before writing the scheduler. Shared
   range, shared LOS status, shared dropout; **independent** bearings.
2. **`az` is positive to the LEFT, `el` positive UP** (FLU). Assert it (§8.1 checks 2–3).
3. **`R^T`, not `R`.** The peer's displacement is rotated *into* the observer's body frame.
   §8.1 check 4 catches the transpose.
4. **NLOS range bias is `abs()`** of the draw — never signed. A blocked path is longer.
5. **The reported σ excludes the antenna-delay bias.** Deliberate (§2.2.1). Do not "fix" it.
6. **Dropouts publish no row**; degraded measurements publish a row with flags cleared.
   There is no such thing as a row with `FLAG_RANGE_VALID` clear.
7. **`odom.twist` frame is unverified** — use `odom.pose` only (`AGENTS.md §6.4`).
8. **Stale odom removes a drone from the mesh.** Do not coast on the last pose.
9. **`python3 -u`** whenever output is redirected (`AGENTS.md §4`).
10. **Kill `gz sim`, not just `cf2`**, between runs — it holds all 2N UDP ports
    (`AGENTS.md §4`).
11. **CRTP v7 one-shot arming** — `uwb_gate.py` must open **both** cflib connections as the
    first connections against a freshly launched sim (§6.2).
12. **`mlflow.db` on `/mnt/d` wedges under unclean kills.** A hang at `start_run()` is
    almost always an orphaned process holding the sqlite file, not your code.
13. **Do not raise `max_exchanges_per_s` to make a gate pass.** It is a physical constraint
    (§1.3). If a gate needs more airtime than the radio has, the gate is testing a
    configuration that cannot fly.
14. **`gz-sim` starts paused** unless `-r` (`AGENTS.md §4`) — "no edges published" is often
    just this, or an unsourced `setup_env.sh`.

---

## 10. Build order

1. **§6 — multi-drone launch.** Nothing else is testable without it. Prove it with
   `-n 2` and two hovering drones before writing any UWB code.
2. `configs/sensors/uwb_pdoa.yaml` (§4).
3. `uwb_edges.py` + §8.1 check 15. Small, self-contained, and everything downstream
   depends on the layout being right.
4. `uwb_model.py`: geometry → noise → LOS → scheduler, in that order, adding the matching
   §8.1 assertions as you go. **Do not proceed until all 17 are green.** This retires the
   bulk of the correctness risk without launching Gazebo once.
5. `uwb_node.py` (§3), then the live smoke test (§7.2). Confirm the spawn-asymmetry
   prediction in §7.2 before anything else.
6. `eval_scripts/uwb_gate.py` (§7.3). If a check fails, suspect §9.1–9.3 (shared range,
   sign conventions, transpose) before touching a threshold.
7. Doc updates (§11). Report the real numbers; if something did not pass, say so with the
   output rather than loosening a threshold to make it green.

---

## 11. Doc / status updates to make in the same session (`AGENTS.md §6.9`)

**Completed 2026-07-30** — all items below were applied:

- **`.cursor/AGENTS.md §3`**: M4 status → DONE, with live gate numbers. Note the §1.1
  reversal (Python node, not a C++ plugin) and the §1.3 addition (airtime budget) so the
  next session does not "fix" them back. Also record that `phase0_gate.sh` now supports
  `-n N` — that is a repo-wide capability, not a UWB detail.
- **`AGENTS.md §2` repo map**: add `perception/uwb_sim/`.
- **`Phase1_..._Implementation_Plan.md`**:
  - §5 banner → point at this doc as the authority for M4.
  - §5.5 → status, and the §1.5 delta table (Python not C++; one node not two; one config
    not three; λ/2 antenna spacing correction; airtime model; antenna-delay bias; NLOS
    invalidates bearing).
  - §6 config list: `uwb.yaml` / `uwb_anchors.yaml` / `uwb_pdoa.yaml` → **one**
    `uwb_pdoa.yaml`.
  - §7 gate table: M4a and M4b rows → the single `uwb_gate.py` row with results.
  - §8 file list and §9 milestone 4 → per §7's M4-0…M4-4.
  - §10: strike the "no native Gazebo UWB sensor" open item (resolved), and add the λ/2
    antenna-spacing correction as a resolved risk.
- **`configs/rviz/radar.rviz`**: add a PointCloud2 display on `/uwb/edges_all` so the mesh
  is visible next to the radar cloud.
- **`requirements.txt`**: `trimesh` / `embreex` under an `# Optional:` comment (§5.2).

---

## 12. Aside — MiFly-style passive tags (NOT part of this build)

Recorded because it is the fallback if the mesh turns out to need more physical reference
points than landed drones can supply, and because retiring the pucks left an architectural
hole worth naming.

**The idea.** MIT (Fadel Adib's Signal Kinetics group) published *MiFly*, a
self-localization approach in which a drone carries the active radio and the environment
carries **passive, battery-free UWB backscatter tags** — small "stickers" that
retroreflect the drone's own interrogation signal. The drone localizes itself against a tag
without the tag needing power, provisioning, or a network.

**Why it fits this mission.** It replaces "manufacture and deploy powered pucks" — which we
have declined to do — with "stick a tag on a wall". Tags are cheap, essentially massless,
and could plausibly be applied by the first drone through a corridor or pre-placed by a
human at the entrance.

**What would change in the model** (if it is ever built):

- A tag is a **passive** peer: it never observes. It produces edges in exactly one
  direction, drone → tag. In this code that is a `static_peer` with an `observes: false`
  flag — a small addition to §3.3, not a new node.
- Bearing to a tag would come from the **drone's own** array, so the existing PDoA model
  applies unchanged. There is no reciprocal bearing, so the mutual-bearing yaw constraint
  of §2.4 does **not** exist for tag edges. That is a real reduction in mesh rigidity and
  the estimator must not assume otherwise.
- Backscatter link budget is far tighter than active TWR (the signal makes a round trip and
  is reflected by an unpowered device), so `max_range_m` and
  `p_dropout_at_max_range` would need their own values, and its NLOS behaviour is likely
  worse. Do not reuse the active-radio numbers.
- A tag's position is a **state variable** unless a human surveyed it. Same `AGENTS.md §1
  Tier A` wall as a landed drone.

⚠ **Everything above about MiFly's specifics — accuracy figures, range, tag construction,
whether it provides bearing at all — is from recollection and is NOT verified.** Before any
of it enters a config or a claim, read the paper and record the actual numbers, exactly as
the v3 roadmap's Phase 1 item 1 requires of every other noise model. Treat this section as a
pointer to a literature task, not as a specification.
