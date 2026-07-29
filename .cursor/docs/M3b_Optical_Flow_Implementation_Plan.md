# M3b Implementation Plan — PMW3901 Downward Optical Flow (Phase 1, Workstream C)

**Status of this doc:** build-ready spec. Expands §4.2 of
`Phase1_Physical_Fidelity_and_Sensor_Implementation_Plan.md`. Where it disagrees with
that doc, **this doc wins** — the differences are listed in §1.3 and each has a reason.

**Audience:** an implementer who has *not* read the rest of the repo. Everything needed
to write the code is in here. Do not invent APIs; every firmware constant quoted below
was read out of the pinned submodule and the file:line is given so you can re-check it.

---

## 0. Read this before writing anything — the naming trap

The task was described as "IR ToF optical flow". **There is no such thing, and building it
that way would be wrong.** The Bitcraze **Flow deck v2** is one PCB carrying **two
physically different chips** that both point down:

| Chip | Physics | Output | Repo status |
|---|---|---|---|
| **STMicro VL53L1x** | 940 nm IR laser, times the pulse return | **one range** (height above floor) | **DONE** (M3a). `gpu_lidar`, 1 ray, `/cf_0/tof_down`, gated at ~29.9 Hz |
| **PixArt PMW3901** | tiny downward camera + motion DSP, correlates consecutive floor-texture frames | **integrated pixel displacement** → horizontal velocity | **this task (M3b)** |

You **cannot** compute optical flow from `/cf_0/tof_down` — it is a single scalar range,
there is no image and no lateral information in it. M3b is a **new, separate sensor
model**. The ToF *is* used by M3b, but only as the **height input** the flow model needs
to convert angular flow into metric velocity — exactly as on real hardware, where the
Flow deck firmware divides pixel motion by the VL53L1x range.

`AGENTS.md §5` explicitly forbids claiming "PMW3901 measures range" or "flow works on
untextured floors without an invalid flag". §4 of this plan is what keeps us honest on
the second one.

---

## 1. Design decisions (and why) — do not re-litigate these while implementing

### 1.1 Analytic ROS 2 node, not a rendered camera

Two options were on the table (plan §4.2). **Pick option 1: an analytic `rclpy` node.**

- A real PMW3901 is 35×35 px at ~100 Hz. Rendering that in Gazebo and running real
  block-matching on it would burn GPU for an image nobody looks at, and the *texture*
  realism you'd buy is fake anyway — our worlds have no photometrically-calibrated floor
  materials, so the "texture" driving the correlation would be an arbitrary choice of
  visual material. You'd pay a lot and get modelling error dressed up as physics.
- The analytic path takes ground-truth motion, pushes it through the **exact measurement
  equation the Crazyflie firmware already uses** (§2), and adds a noise + validity model
  that is explicitly config-driven and labelled Tier B. That is the same fidelity posture
  as M3a's ToF and the UWB plan, and it is honest.
- Upgrade path is preserved: if Phase 5+ ever wants photometric texture dropout, the
  camera pipeline can replace the node behind the same topic contract.

**Consequence for fidelity claims:** we may claim body-frame `(vx, vy)` on a textured
floor with a modelled invalid flag. We may **not** claim the sim validates the *texture
threshold itself* — the surface-quality map (§4) is a hand-authored Tier B input, not a
measurement. Say so in the docstring and in the doc updates (§9).

### 1.2 Emit the chip's native quantity, not just a convenience velocity

Publish **integrated pixel deltas** (`dpixelx`, `dpixely`, `dt`, `stdDev`, `quality`,
`valid`) as the primary product, plus a **derived** `(vx, vy)` for convenience.

Reason: the teammate's RIO front-end will fuse flow with IMU and height. Pixel flow is
**coupled** to height and body angular rate (§2) — a bare `(vx, vy)` topic silently bakes
in a height estimate and a gyro-compensation the estimator should be doing itself, and
hides the fact that flow noise blows up as height grows. Handing her a pre-solved `(vx,
vy)` would make her filter look better than it is. Give her the raw measurement *and* the
convenience channel, clearly labelled.

### 1.3 Deltas from the parent plan (§4.2), with reasons

| Parent plan said | This plan does | Why |
|---|---|---|
| topic `geometry_msgs/TwistStamped` "or a custom msg with valid/quality" | **`geometry_msgs/TwistWithCovarianceStamped`** on `/cf_<id>/flow`, plus `geometry_msgs/Vector3Stamped` on `/cf_<id>/flow/pixels` | A custom msg needs a `rosidl` package + CMake + colcon build — friction, and C++ build files the user doesn't want to maintain (`AGENTS.md §1`: C++ only where forced). `TwistWithCovarianceStamped` is a stock msg that carries the header, the velocity, **and** the per-axis variance an EKF actually wants. Invalid is signalled by a sentinel variance (§3.3). Zero new build steps. |
| `update_rate_hz: 200` in the `tof.yaml` stub | **100 Hz** | Verified: the real driver task is `vTaskDelay(10)` = 10 ms = 100 Hz (`firmware_mods/CrazySim/crazyflie-firmware/src/deck/drivers/src/flowdeck_v1v2.c:88`). Use the measured firmware rate, not a guess. |
| `noise_stddev_mps: 0.05` | noise lives in the **pixel domain**, `flow_std_px: 2.0` | The firmware's own default is `flowStdFixed = 2.0f` pixels (`flowdeck_v1v2.c:77`). Velocity-domain noise is wrong physics: the same pixel noise means a much larger velocity error at 3 m than at 0.3 m. Modelling in pixels reproduces that for free. |
| `max_flow_mps: 7.4` | **`max_flow_rad_s: 7.4`** (angular flow rate `v/h`) | The 7.4 figure is an *angular* flow-rate limit; quoting it as m/s is only correct at exactly 1 m height. Saturating on `v/h` is the correct behaviour and reproduces "fast + low = saturates". |
| config block inside `configs/sensors/tof.yaml` | **new file `configs/sensors/optical_flow.yaml`** | One config per sensor, matching how `apply_tof_sensor.py` was deliberately split out of `apply_payload.py` for single responsibility. `tof.yaml`'s commented stub gets deleted and replaced by a pointer comment. |
| "`apply_flow_sensor.py` **or** `flow_node.py`" | **`flow_node.py` only** | There is no SDF element to inject — the model is entirely in the node. An `apply_flow_sensor.py` would have nothing to write. |

### 1.4 Explicitly out of scope

- **Feeding flow back into the SITL firmware's Kalman filter** (`estimatorEnqueueFlow`).
  That would need C edits inside the pinned `CrazySim` submodule + a new UDP sensor
  channel in `gz_crazysim_plugin`, which `AGENTS.md §6.6` forbids doing in place. The
  deliverable is the **measurement stream**; the estimator that consumes it is Phase 2.
  Mention this in the node docstring so nobody assumes the drone is flying on flow.
- Photometric/rendered texture (see §1.1).
- Multi-drone. Node is per-`cf_id`, launched once.

---

## 2. The measurement model — this is the core, get it exactly right

### 2.1 Source of truth

The Crazyflie EKF's flow update lives at
`firmware_mods/CrazySim/crazyflie-firmware/src/modules/src/kalman_core/mm_flow.c`. Read
lines 40–100. The predicted measurement is (verbatim, lines 79 and 92):

```c
predictedNX = (flow->dt * Npix / thetapix) * ((dx_g * this->R[2][2] / z_g) - omegay_b);
predictedNY = (flow->dt * Npix / thetapix) * ((dy_g * this->R[2][2] / z_g) + omegax_b);
```

with, from the same file:

| Symbol | Value / meaning | Line |
|---|---|---|
| `Npix` | `35.0` pixels | 43 |
| `thetapix` | `0.71674` rad (≈41.1° total aperture) | 45 |
| `Npix / thetapix` | **48.833 px/rad** — precompute this once | — |
| `dx_g`, `dy_g` | **body-frame** horizontal velocity, m/s (EKF states `PX`,`PY`) | 64–65 |
| `z_g` | height, m, **clamped to a 0.1 m floor** to avoid the singularity | 67–72 |
| `R[2][2]` | world-Z component of the body-Z axis = `cos(tilt)` | 79 |
| `omegax_b`, `omegay_b` | body angular rate about X and Y, **rad/s** | 47–48 |
| `flow->dt` | integration interval, s | — |

And crucially, line 80 / 93:

```c
measuredNX = flow->dpixelx * FLOW_RESOLUTION;   // FLOW_RESOLUTION 0.1f, line 29
```

So **the driver reports 10× the motion pixels.** Our simulated `dpixelx` must be in that
same inflated driver unit, i.e. `dpixelx = N_X / 0.1 = 10 * N_X`, so that a consumer
applying `FLOW_RESOLUTION` recovers `N_X`. Do not skip this factor — it is the difference
between the teammate's filter working and being 10× off.

### 2.2 The forward (simulation) model

Per output frame, given ground truth and the measured height:

```
K       = Npix / thetapix                       # 48.833 px/rad, precompute
z_eff   = max(h_meas, z_floor_m)                # z_floor_m = 0.1, mirrors mm_flow.c:67-72
R22     = 1 - 2*(qx^2 + qy^2)                   # from the body->world quaternion (x,y,z,w)

# angular flow rate about each axis, rad/s  (this is what saturates)
fx_rate = (vx_body * R22 / z_eff) - wy_body
fy_rate = (vy_body * R22 / z_eff) + wx_body

# saturation (PMW3901 loses lock above ~7.4 rad/s)
fx_sat  = clamp(fx_rate, -max_flow_rad_s, +max_flow_rad_s)
fy_sat  = clamp(fy_rate, -max_flow_rad_s, +max_flow_rad_s)

N_X     = dt * K * fx_sat                       # motion pixels this frame
N_Y     = dt * K * fy_sat

dpixelx = N_X / FLOW_RESOLUTION                 # = 10 * N_X, driver units
dpixely = N_Y / FLOW_RESOLUTION
```

**Sanity number to hard-code as a unit test (§7.1):** `vx=1.0 m/s, h=1.0 m, level, ω=0,
dt=0.01` ⇒ `fx_rate = 1.0 rad/s`, `N_X = 0.4883 px`, `dpixelx = 4.883`.

### 2.3 Noise, quantisation, outlier clip — applied in this order

```
sigma_px = flow_std_px / max(quality, quality_eps)      # quality from §4, in (0,1]
dpixelx += gauss(0, sigma_px)                            # driver units
dpixely += gauss(0, sigma_px)
dpixelx  = round(dpixelx);  dpixely = round(dpixely)     # chip returns integers
# driver drops frames beyond OULIER_LIMIT=100 (flowdeck_v1v2.c:45,98)
if abs(dpixelx) >= outlier_limit_px or abs(dpixely) >= outlier_limit_px:
    valid = False
```

Dividing σ by quality is the mechanism by which a poorly-textured floor degrades
*gracefully* before it drops out — this mirrors the real driver's adaptive-σ path
(`flowdeck_v1v2.c:99-111`, σ grows with shutter time on low-texture floors), without
pretending we've modelled shutter.

Use a **seeded** `numpy.random.default_rng(seed)` from config; log the seed to MLflow
(`AGENTS.md §6.5`).

### 2.4 The derived `(vx, vy)` — invert §2.2

```
fx_hat  = (dpixelx * FLOW_RESOLUTION) / (dt * K)         # rad/s, back out of driver units
vx_hat  = (fx_hat + wy_body) * z_eff / max(R22, r22_min) # r22_min = 0.5, guards extreme tilt
vy_hat  = (fy_hat - wx_body) * z_eff / max(R22, r22_min)
```

Note this uses the **measured** height and the **true** body rates. That is a deliberate,
documented convenience: the gyro-compensation term is what a real flow deck's host does
with its own gyro, and we do not model gyro noise here (the IMU is already simulated
separately by `gz-sim-imu-system`). **Put this caveat in the node docstring and in the
`/cf_<id>/flow` topic's own doc comment** — the derived channel is *slightly* flattered
relative to what a real host would get, and the teammate must know that the `pixels`
topic is the un-flattered one.

Variance published on the twist: propagate the pixel σ through the same inversion,

```
sigma_v = (sigma_px * FLOW_RESOLUTION / (dt * K)) * z_eff / max(R22, r22_min)
var_v   = sigma_v ** 2
```

This automatically grows with height, which is the whole point of §1.2.

---

## 3. Node spec — `perception/flow_sim/flow_node.py`

Plain `rclpy` script, **no** `package.xml` / colcon package. Run it with `python3` after
sourcing `setup_env.sh`, exactly like the `eval_scripts/*.py` gates. (`perception/` is the
mapped home for sensor/perception code per `AGENTS.md §2`; `eval_scripts/` is for gates.)

### 3.1 Inputs

| Topic | Type | Source | Use |
|---|---|---|---|
| `/cf_<id>/odom` | `nav_msgs/msg/Odometry` | bridged from `gz.msgs.Odometry` (the `gz-sim-odometry-publisher-system` in `model.sdf.jinja:302-308`, 200 Hz) | ground-truth pose → body velocity, body angular rate, orientation |
| `/cf_<id>/tof_down` | `sensor_msgs/msg/LaserScan` | already bridged by `phase0_gate.sh:381-389` | measured height (`ranges[0]`) |

**Do not use `odom.twist`.** Its frame convention (body vs world) is not documented
anywhere we can verify, and `AGENTS.md §6.4` bans guessing. Instead **derive everything
from `odom.pose`**, which is unambiguous:

```
# body-frame linear velocity, from consecutive world poses
dp_world = (p_k - p_{k-1}) / dt_odom
v_body   = R(q_k)^T · dp_world                # take x,y components

# body angular rate, from consecutive quaternions
dq       = q_{k-1}^{-1} ⊗ q_k                 # Hamilton product, (x,y,z,w) convention
w_body   = 2 * dq.vec / dt_odom               # sign-flip dq if dq.w < 0 (shortest arc)
```

Ground-truth pose at 200 Hz over 1 kHz physics is exact, so finite differencing is clean.
Guard `dt_odom <= 0` (duplicate stamps) by skipping the frame.

Light low-pass (single-pole, `alpha` from config, default 0.3 on `w_body` only) to take
the edge off quaternion differencing; leave `v_body` unfiltered.

**Cross-check, not dependency:** on the first 200 samples, also compute
`||odom.twist.linear||` and compare against both `||dp_world||` and `||v_body||`, then log
once at INFO which one it matches (`"odom.twist.linear appears to be in the BODY/WORLD
frame"`). This costs nothing and finally documents the convention for the repo. Never
branch behaviour on it.

**Height sourcing** (config `height_source`):
- `tof` (default): use `ranges[0]` from the latest `/cf_<id>/tof_down`. Reject
  non-finite (the bridge passes `inf` through for no-return) and out-of-band values;
  if the last good ToF is older than `tof_stale_s` (default 0.2 s), treat height as
  unknown → `valid=False`, `quality=0`. This is correct behaviour: a real Flow deck with
  a dead ranger cannot produce metric velocity.
  Add the ToF's own mount offset back: the sensor sits at `z = -0.02` on `base_link`
  (`configs/sensors/tof.yaml`), so it already reads height-above-floor from its own
  position; use `ranges[0]` directly as `h_meas` and note that in a comment.
- `truth`: use `odom.pose.position.z`. Debug/ablation only; log a WARNING at startup.

### 3.2 Timing

Run the publish loop on a `create_timer(1.0 / update_rate_hz)`. `dt` published in each
frame is the **actual wall/ROS time since the previous publish**, not the nominal period —
the real driver does the same (`flowdeck_v1v2.c:120-122` uses a measured `usecTimestamp()`
delta, with the comment explaining why). Use the node clock (`self.get_clock().now()`),
which follows `/clock` if `use_sim_time` is set — set `use_sim_time` from config, default
`false` (matches how the ToF gate works today; revisit only if the sim runs off real time).

Stamp every message with the **ToF/odom message stamp** you most recently consumed, not
`now()`, so downstream time-sync is meaningful.

### 3.3 Outputs

**`/cf_<id>/flow` — `geometry_msgs/msg/TwistWithCovarianceStamped`** (primary consumable)

- `header.frame_id`: `cf_<id>/base_link`
- `twist.twist.linear.x/y` = `vx_hat`, `vy_hat` (§2.4); `linear.z = 0.0`
- `twist.twist.angular` = all zeros (PMW3901 measures no rotation)
- `twist.covariance` — 6×6 row-major, 36 floats:
  - `[0]` (vx variance) and `[7]` (vy variance) = `var_v` from §2.4
  - `[14]` (vz) and `[21],[28],[35]` = `INVALID_VAR` (unmeasured axes)
  - everything else `0.0`
  - **when `valid == False`: set `[0]` and `[7]` to `INVALID_VAR` too**, and publish
    `linear.x = linear.y = 0.0`. Define `INVALID_VAR = 1e6`. Document it in the node
    docstring and in `optical_flow.yaml`. This is the invalid signal — a consumer that
    ignores covariance still gets zeros rather than a fabricated velocity, and a
    consumer that respects it correctly assigns the measurement no weight.
    (Rationale for a sentinel over dropping the message: the RIO front-end wants a
    steady heartbeat so it can tell "flow says nothing" from "flow node died".)

**`/cf_<id>/flow/pixels` — `geometry_msgs/msg/Vector3Stamped`** (chip-native)

- `vector.x` = `dpixelx`, `vector.y` = `dpixely` (driver units, i.e. 10× motion pixels)
- `vector.z` = `quality` in `[0.0, 1.0]`
- same `header` as above. `valid` is recoverable as `quality >= min_quality`; also encode
  hard-invalid as `quality = 0.0`.
- The measured `dt` and σ are **not** in this message. Publish them on
  `/cf_<id>/flow/meta` — `geometry_msgs/msg/Vector3Stamped` with `x = dt_s`,
  `y = sigma_px` (driver units, the `flowData.stdDev*` equivalent), `z = h_meas`. Three
  stock messages beats one custom package.

**`/cf_<id>/flow/debug_truth` — `geometry_msgs/msg/TwistStamped`**, published only when
`publish_ground_truth: true` (default `true`).

- `twist.linear.x/y/z` = true `v_body`; `twist.angular.x/y/z` = true `w_body`.
- ⚠ **`AGENTS.md §1 Tier A`: this is a sim oracle. Nothing in the estimator, the RIO
  front-end, or any Phase-2+ code may subscribe to it — it exists so `flow_gate.py` can
  score without re-deriving.** Put that warning, in those words, in the node docstring,
  in the config comment, and in a `get_logger().warn()` fired once at startup when the
  flag is on.

QoS: `SensorDataQoS()` (best-effort, depth 10) on all publishers and on the two
subscriptions — the ToF bridge and odom bridge publish best-effort.

### 3.4 CLI

```
python3 perception/flow_sim/flow_node.py \
    [--config configs/sensors/optical_flow.yaml] \
    [--cf-id 0] [--seed 0] [--no-truth] [--height-source tof|truth] [--selftest]
```

`--selftest` runs §7.1 with **no ROS init at all** and exits 0/1. It must be importable
and runnable without a live sim — factor the math into a plain
`class FlowModel` (no rclpy references) that the node wraps. Do this from the start; it
makes the whole thing testable in a second instead of a five-minute sim launch.

### 3.5 Startup behaviour

- If `deck` in `configs/sensors/tof.yaml` is **not** `flow_v2`, log an ERROR explaining
  that only Flow deck v2 carries a PMW3901 (`zranger_v2` does not) and exit 2. Read that
  file for the check; don't duplicate the deck selector into `optical_flow.yaml`.
- Log the resolved config, the seed, `Npix/thetapix`, and the height source on one line.

---

## 4. Surface / validity model — the part that keeps us honest

`AGENTS.md §5` and the parent plan's §0 table both require that texture dropout be
modelled and that we never claim flow works on a smooth floor. This is how.

### 4.1 Surface-quality map

A **config-driven list of axis-aligned world-frame rectangles**, each with a
`texture_quality` in `[0, 1]`, over a `default_texture_quality` background. The node looks
up the drone's world `(x, y)` from odom; last matching rectangle in list order wins.

Why config rectangles and not world SDF materials: it needs zero world edits (so it works
in `phase1_pid_tune`, `phase0_tunnel_gate`, and any future world), it's deterministic and
sweepable, and it's honestly labelled Tier B rather than pretending a Gazebo material
encodes photometric texture. Cost: it does not follow the visual world. Accept that; note
it in the config header.

### 4.2 Quality factors (all in `[0,1]`, multiplied together)

```
q_surface = lookup(x, y)                        # §4.1

q_height:                                       # focus + feature-size band
    0                              h < min_height_m  or  h > max_height_m
    linear ramp 0->1               over [min_height_m,  good_height_min_m]
    1                              over [good_height_min_m, good_height_max_m]
    linear ramp 1->0               over [good_height_max_m, max_height_m]

q_tilt:                                         # beam leaves the floor patch
    1                              tilt <= tilt_full_deg
    linear ramp 1->0               over [tilt_full_deg, tilt_zero_deg]
    0                              tilt >= tilt_zero_deg
    where tilt = acos(clamp(R22, -1, 1))

q_flow:                                         # correlation fails near saturation
    1 - clamp(max(|fx_rate|, |fy_rate|) / max_flow_rad_s, 0, 1)

quality = q_surface * q_height * q_tilt * q_flow
valid   = (quality >= min_quality) and height_is_fresh and (not outlier_clipped)
```

Every threshold above is a config key (§5). Publish `quality`; do **not** publish the
individual factors (keep the message contract small) but **do** log them at DEBUG.

---

## 5. `configs/sensors/optical_flow.yaml` — create this file verbatim-ish

```yaml
# PMW3901 downward optical flow — Phase 1 M3b.
# See .cursor/docs/M3b_Optical_Flow_Implementation_Plan.md
#
# The PMW3901 is the SECOND chip on the Bitcraze Flow deck v2. The first is the
# VL53L1x IR ToF rangefinder (M3a, configs/sensors/tof.yaml) — a different chip
# measuring a different quantity. Flow is NOT computed from ToF; ToF only
# supplies the height this model divides by.
#
# Simulated analytically by perception/flow_sim/flow_node.py using the exact
# measurement equation the Crazyflie firmware's EKF uses
# (firmware_mods/CrazySim/crazyflie-firmware/src/modules/src/kalman_core/mm_flow.c).
# Active only when configs/sensors/tof.yaml has `deck: flow_v2`.
#
# FIDELITY (AGENTS.md §5): geometry + noise are high-fidelity. The surface map
# below is a HAND-AUTHORED Tier B input, not a measurement — we may claim
# "dropout on low-texture floors is modelled", NOT "the sim validates where the
# real texture threshold is".

# --- chip constants (verified against mm_flow.c / flowdeck_v1v2.c — see the
# --- plan doc §2.1 for file:line. Do not change without re-reading them.)
npix: 35.0                  # mm_flow.c:43
thetapix_rad: 0.71674       # mm_flow.c:45  (~41.1 deg aperture)
flow_resolution: 0.1        # mm_flow.c:29  driver reports 10x motion pixels
outlier_limit_px: 100       # flowdeck_v1v2.c:45  (driver units)

# --- rates & noise (Tier B)
update_rate_hz: 100         # flowdeck_v1v2.c:88 vTaskDelay(10) == 10 ms
flow_std_px: 2.0            # flowdeck_v1v2.c:77 flowStdFixed, driver units
quality_eps: 0.05           # floor on the quality divisor in sigma_px
max_flow_rad_s: 7.4         # angular flow-rate saturation (v/h), NOT m/s
z_floor_m: 0.1              # mm_flow.c:67-72 singularity clamp
r22_min: 0.5                # guard on the 1/cos(tilt) inversion in the derived vx/vy
invalid_variance: 1.0e6     # sentinel in twist.covariance when valid == false

# --- height input
height_source: tof          # tof | truth   (truth = debug/ablation only)
tof_stale_s: 0.2            # older than this -> height unknown -> invalid
tof_min_valid_m: 0.04       # mirrors tof.yaml range_min_m
tof_max_valid_m: 4.0        # mirrors tof.yaml range_max_m

# --- validity / quality model (plan §4.2)
min_quality: 0.15
min_height_m: 0.08
good_height_min_m: 0.15
good_height_max_m: 2.5
max_height_m: 4.0
tilt_full_deg: 15.0
tilt_zero_deg: 45.0
omega_lowpass_alpha: 0.3    # single-pole on body angular rate from quaternion diff

# --- surface texture map (world frame, axis-aligned rects; last match wins)
# quality 1.0 = richly textured rubble/concrete; 0.0 = polished glass.
default_texture_quality: 1.0
surface_patches:
  # Low-texture patch used by flow_gate.py's dropout test. Keep it clear of the
  # (0,0) spawn so takeoff and the hover leg happen over good texture.
  - {name: smooth_tile, x_min: 1.0, x_max: 3.0, y_min: -1.0, y_max: 1.0, texture_quality: 0.0}

# --- debug
publish_ground_truth: true  # /cf_<id>/flow/debug_truth — SIM ORACLE, AGENTS.md §1
                            # Tier A: the estimator must NEVER subscribe to it.
seed: 0

mlflow_experiment: "phase1_optical_flow"
```

Also **edit `configs/sensors/tof.yaml`**: delete the commented-out `optical_flow:` stub at
lines 51–61 and replace it with a three-line pointer to `optical_flow.yaml` + the plan
doc, so there is exactly one place flow is configured.

---

## 6. Launch wiring — `eval_scripts/phase0_gate.sh`

Three edits, mirroring the existing ToF blocks exactly.

**(a) Flags.** Add `--no-flow` and `--flow-config PATH` next to `--no-tof` /
`--tof-config`: defaults `USE_FLOW=true`, `FLOW_CONFIG=""`; add to the `case` block, to
the two usage blocks (the header comment at lines ~29-31 and `usage()` at ~69-70).

**(b) Bridge odom.** The flow node needs `/cf_<id>/odom` in ROS. The bridge invocation at
`phase0_gate.sh:381-389` currently bridges only `tof_down`. Extend that same
`parameter_bridge` call with a second mapping:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/cf_${CF_ID}/tof_down@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
  "/cf_${CF_ID}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry" &
```

Keep it one process (one bridge, two topics). Update the guard condition so the bridge
runs when **either** `USE_TOF` or `USE_FLOW` is true, and update the two `warn` lines to
mention odom.

**(c) Launch the node**, after the bridge block and before RViz:

```bash
if [[ "$USE_FLOW" == true ]]; then
  _flow_cfg="${FLOW_CONFIG:-$SAR_NANO_SWARM_ROOT/configs/sensors/optical_flow.yaml}"
  [[ "$_flow_cfg" != /* ]] && _flow_cfg="$SAR_NANO_SWARM_ROOT/$_flow_cfg"
  if [[ ! -f "$_flow_cfg" ]]; then
    warn "Optical-flow config not found: $_flow_cfg — skipping flow node."
  elif ! command -v ros2 &>/dev/null; then
    warn "ros2 not on PATH — skipping optical-flow node (source setup_env.sh)."
  else
    info "Starting PMW3901 optical-flow node ($_flow_cfg) …"
    python3 -u "$SAR_NANO_SWARM_ROOT/perception/flow_sim/flow_node.py" \
      --config "$_flow_cfg" --cf-id "$CF_ID" &
    _PIDS+=($!)
  fi
fi
```

`python3 -u` is **required**, not optional — `AGENTS.md §4`: buffered stdout is lost when
the trap kills the process, and this repo has already been bitten by it.

**(d)** Add a `║  Flow topic   : /cf_${CF_ID}/flow  (ROS, ~100 Hz)` line to the
"Simulation ready" banner (~line 436) and a `║  Optical flow : ${USE_FLOW}` status line.

---

## 7. Verification

### 7.1 Offline self-test (`--selftest`) — write this first, run it constantly

No ROS, no sim. Assert, each within 1e-3 relative:

1. **Forward model.** `vx=1.0, vy=0, h=1.0, R22=1, w=0, dt=0.01, noise off`
   ⇒ `dpixelx ≈ 4.883`, `dpixely ≈ 0`.
2. **Height scaling.** Same but `h=2.0` ⇒ `dpixelx ≈ 2.441` (halves — flow is angular).
3. **Gyro coupling.** `vx=0, wy=1.0 rad/s` ⇒ `dpixelx ≈ -4.883` (note the **minus**: the
   `-omegay_b` sign in `mm_flow.c:79`). Getting this sign wrong is the single most likely
   bug; assert it explicitly. Correspondingly `vy=0, wx=1.0` ⇒ `dpixely ≈ +4.883`.
4. **Round trip.** Feed the noiseless `dpixel*` back through §2.4 ⇒ recover `vx, vy` to 1e-6
   for a few random `(v, h, ω, tilt)` tuples.
5. **Saturation.** `vx=100, h=1.0` ⇒ `|fx_rate|` clamps at `7.4` and `quality == 0`
   (q_flow drives it to 0), `valid == False`.
6. **Surface map.** `(x,y) = (2.0, 0.0)` ⇒ `q_surface == 0` ⇒ `valid == False`;
   `(0,0)` ⇒ `q_surface == 1`.
7. **Height band.** `h = 0.05` and `h = 5.0` ⇒ `quality == 0`, `valid == False`.
8. **Variance sentinel.** invalid frame ⇒ `covariance[0] == covariance[7] == 1e6` and
   `linear.x == linear.y == 0.0`.
9. **Noise scaling.** With `quality=0.5`, empirical σ of 10 000 sampled `dpixelx` ≈
   `flow_std_px / 0.5 = 4.0` within 5 %.

### 7.2 Live smoke test

```bash
# terminal 1
./eval_scripts/phase0_gate.sh -w phase1_pid_tune --no-radar --headless
# terminal 2 (setup_env.sh sourced)
ros2 topic hz  /cf_0/flow                     # expect ~100 Hz
ros2 topic echo /cf_0/flow --once
ros2 topic echo /cf_0/flow/pixels --once      # z (quality) should be ~1.0 on the ground plane
ros2 topic list | grep flow
```

On the ground, unarmed: `dpixel*` ≈ 0, `quality` low or 0 (height 0.02 m is below
`min_height_m`) → `valid=False`. **That is correct behaviour**, not a bug — confirm before
concluding anything is broken.

### 7.3 `eval_scripts/flow_gate.py` — the M3b exit gate

Same shape as `tof_gate.py`. Reuse `pid_gains.load_gains/apply_gains/reset_estimator/
reset_pose` and the `SyncCrazyflie` + `MotionCommander` pattern from `tof_gate.py:151-243`
verbatim — including the `SIGALRM` watchdog (`_Timeout`/`_alarm`), the
`warnings.filterwarnings` for cflib's CRTP-v7 deprecation spam, and the "run this as the
FIRST cflib connection against a freshly launched sim" note in the docstring.

It is an **rclpy node** (unlike `tof_gate.py`, which shells out to `gz topic`) because the
flow topics are ROS-native. Spin it on a background `MultiThreadedExecutor` thread while
the cflib flight sequence runs on the main thread; buffer `(stamp, vx, vy, quality, valid)`
from `/cf_0/flow` and `(stamp, vx_true, vy_true)` from `/cf_0/flow/debug_truth`, and pair
them by nearest stamp within 15 ms.

**Flight sequence** (all in `phase1_pid_tune`, `--no-radar --headless`):

| Leg | Action | Window scored |
|---|---|---|
| 0 | takeoff to `0.5 m`, settle 2.5 s | — |
| 1 | hold hover 5 s at (0,0) | **A: hover** |
| 2 | `mc.forward(0.8, velocity=0.3)` → sit 1 s | **B: forward flight** |
| 3 | `mc.forward(1.6, velocity=0.3)` (now at x≈2.4, inside the smooth patch), sit 2 s | **C: dropout** |
| 4 | `mc.back(2.4)`, `mc.stop()` | — |

**Pass criteria** (all must hold; thresholds as module constants like `hover_gate.py:35-36`):

| # | Check | Threshold |
|---|---|---|
| 1 | publish rate on `/cf_0/flow` | ≥ `0.8 * update_rate_hz` (≥ 80 Hz) |
| 2 | window A: `valid` fraction | ≥ 0.9 |
| 3 | window A: RMS of `hypot(vx, vy)` | ≤ 0.10 m/s |
| 4 | window B: RMSE of `(vx,vy)` vs `debug_truth` over valid samples | ≤ 0.10 m/s |
| 5 | window B: mean `vx` sign matches truth and \|bias\| | ≤ 0.05 m/s (catches sign/axis flips) |
| 6 | window C: `valid` fraction | ≤ 0.1 |
| 7 | window C: for invalid samples, `covariance[0] == invalid_variance` and `vx==vy==0` | 100 % (no silent truth pass-through) |
| 8 | did not diverge / did not time out | — |

Check 5 exists because checks 3–4 are both satisfied by a sign-flipped `vy`; the axis and
sign convention is the second most likely bug after §7.1.3.

**MLflow** (`sqlite:///mlflow.db`, experiment from config's `mlflow_experiment`, run name
`flow_gate`): params `seed`, `update_rate_hz`, `flow_std_px`, `height_source`, all
thresholds; metrics `measured_rate_hz`, `hover_rms_mps`, `fwd_rmse_mps`, `fwd_vx_bias_mps`,
`hover_valid_frac`, `patch_valid_frac`, `diverged`, `gate_pass`. Exit `0`/`1`.

---

## 8. Gotchas — read `AGENTS.md §4` too, these are the ones specific to M3b

1. **`FLOW_RESOLUTION = 0.1`.** Forget it and you are 10× off. `dpixel*` is in driver
   units (10× motion pixels), because the firmware multiplies by 0.1 on receipt.
2. **Gyro sign.** X uses `− omegay_b`, Y uses `+ omegax_b`. Asymmetric, and it is right —
   `mm_flow.c:79` and `:92`.
3. **`odom.twist` frame is unverified.** Derive from `odom.pose` (§3.1). Do not guess.
4. **`gz.msgs.LaserScan` no-return is `inf`**, and via the bridge it arrives as float
   `inf` in ROS. Filter with `math.isfinite` before using it as height. (The gz *CLI*
   quirk of serialising it as the JSON **string** `"Infinity"` is `tof_gate.py`'s problem,
   not yours — you're on the ROS side.)
5. **Stale ToF must invalidate flow.** If ToF stops, flow has no scale. Do not silently
   coast on the last height.
6. **`python3 -u`** whenever output is redirected (`AGENTS.md §4`).
7. **CRTP v7 / one-shot arming.** `flow_gate.py` must be the *first* cflib connection to a
   freshly launched sim — same constraint as `hover_gate.py` and `tof_gate.py`. A second
   run in the same sim session may not take off. Say it in the docstring.
8. **Kill `gz sim`, not just `cf2`, between runs** — `gz sim` holds the UDP ports `cf2`
   needs (`AGENTS.md §4`).
9. **`mlflow.db` on `/mnt/d` wedges under unclean kills.** If a gate hangs at
   `start_run()`, look for orphaned processes holding the sqlite file before debugging
   anything else.
10. **Deck check.** With `deck: zranger_v2` there is no PMW3901 — the node must refuse to
    run, not publish zeros. Silently publishing a sensor the airframe doesn't carry is
    exactly the kind of thing that flatters a downstream result.

---

## 9. Doc / status updates to make in the same session (`AGENTS.md §6.9`)

- **`.cursor/AGENTS.md §3`**: add an M3b bullet next to the M3a one — files created,
  live gate numbers, and the fidelity caveat that the surface map is hand-authored.
- **`Phase1_..._Implementation_Plan.md`**:
  - §4.2 → status `DONE`, and note the §1.3 deltas (topic types, 100 Hz, pixel-domain
    noise, own config file, no `apply_flow_sensor.py`).
  - §7 table row "Optical flow stream (M3b)" → result.
  - §8 file list: `perception/flow_sim/flow_node.py`, `eval_scripts/flow_gate.py`,
    `configs/sensors/optical_flow.yaml` → done; drop `apply_flow_sensor.py`.
  - §9 milestone 3 → M3 complete.
  - §11 last row → built.
- **`configs/sensors/tof.yaml`** header: replace the "optical flow NOT simulated" note.

---

## 10. Build order — do it in this sequence

1. `configs/sensors/optical_flow.yaml` (§5).
2. `FlowModel` class (pure math + quality, no rclpy) inside `flow_node.py`, **plus
   `--selftest`** (§7.1). Get all 9 assertions green before touching ROS. This is ~70 %
   of the correctness risk retired without ever launching Gazebo.
3. The rclpy wrapper: subscriptions, pose-differencing, timer, three publishers (§3).
4. `phase0_gate.sh` wiring (§6), then the live smoke test (§7.2).
5. `eval_scripts/flow_gate.py` (§7.3). Run it. Iterate on thresholds only if the *physics*
   is demonstrably right — if a check fails, first suspect §8.1 (10×), §8.2 (gyro sign),
   or an axis swap, not the threshold.
6. Doc updates (§9). Report the real gate numbers; if something didn't pass, say so with
   the output rather than loosening a threshold to make it green.

---

## 11. Definition of done

- [x] `python3 perception/flow_sim/flow_node.py --selftest` exits 0 (all 9 assertions).
- [x] `./eval_scripts/phase0_gate.sh -w phase1_pid_tune --no-radar --headless` brings up
      `/cf_0/flow`, `/cf_0/flow/pixels`, `/cf_0/flow/meta` at ~100 Hz.
- [x] `python3 -u eval_scripts/flow_gate.py` exits 0 with all 8 checks in §7.3 passing,
      logged to MLflow.
- [x] Node refuses to start under `deck: zranger_v2`.
- [x] `/cf_0/flow/debug_truth` is documented as a sim oracle in three places (§3.3).
- [x] Docs in §9 updated with the actual measured numbers.
