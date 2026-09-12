# P2 deviations from `swarm_localization_plan.md`

## `estimator.max_cov_p_m`: 50 m → 2 m

The plan’s 50 m threshold is when a **position standard deviation** (sqrt of `P` diagonal) is treated as `DIVERGED`. It is not a 50 m position-error allowance.

A 50 m uncertainty is not useful in a SubT corridor: by then the drone would already have hit a wall. Set to **2 m** so a filter that has lost the mesh stops publishing into neighbors on a tunnel-relevant scale.

Restart `rio_stub` / `swarm_loc_node` (or relaunch `phase0_gate.sh`) to pick up the YAML change.

## P2-6 / D8 — corridor MC (offline gate)

`python3 perception/swarm_loc/d8_d11.py --selftest`: drone 0 has bearing, drone 1 does not. Drone-1 position p50 is lower with the rebroadcast bearing than with range-only or `d·d'`. Relative error can stay similar (common-mode + CI); the gate is drone 1's absolute error.

## P2-6 / D11 — live mutual-yaw pair rate is not a gate

The ±45° cone (`aoa_fov_deg: 90`) means a corridor formation almost never produces facing pairs. `swarm_loc_node` logs `n_mutual_yaw_pairs_per_s`; a near-zero live rate **passes**. D11 correctness is the offline facing-pair Monte Carlo in `perception/swarm_loc/d8_d11.py --selftest`. Do not widen the cone or choreograph glances to inflate the live rate.

## P2-7 — centralized vs distributed gate is offline

The plan’s “easy run vs hard run” comparison is `eval_scripts/central_reference.py --selftest`: the same log schema the live node writes (`--log-measurements` / `phase0_gate.sh --swarm-loc-log-dir`). No rosbag reconstruction. Live ATE vs Gazebo is still eval-only and is not this milestone’s blocking gate.

Offline gate numbers (mean ATE over drones): easy bearing+entrance `cent=0.066` `dist=0.045` `|c-d|=0.090`; hard range-only mesh `cent=0.039` `dist=0.186`. The hard-run gap is the price of sequential CI vs a joint batch solve.

## §6.1 — live ATE uses gate odom, not the estimator

`eval_scripts/eval_6_1.py` is the §6.1 metric harness. The estimator still must not subscribe to `/cf_*/odom`. The gate records Gazebo odom into `truth.npz` (`swarm_loc_gate.py --eval-dir`) plus full `STATE_DTYPE` estimates (covariance for NEES). Measurement `cf_*.npz` files have no `P`; without `estimates.npz`, ATE still runs from the log means but NEES is skipped.

CPU is laptop `perf_counter` against the GAP9 **20 ms** budget at 50 Hz (~150 int-GOp/s). It is not a GAP9 measurement. A design that only fits a laptop fails the plan's intent even if the number looks small here.

RPE uses Δt = 1 s translation error. ATE is RMSE with **no SE(3) alignment** (entrance gauge). Hop count is BFS on the logged UWB graph from entrance id 1000.

## Gauge starvation / overconfidence (post-Run-B)

Instrumentation: `eval_6_1` logs entrance-edge counts to peer 1000, centroid vs shape, NIS by measurement type, and min/max eigenvalues of `P_p`. `cf_*.npz` now includes a `nis` array and is flushed every 5 s so hops/mix survive a `timeout` kill.

Angle wrap: azimuth/elevation innovations are wrapped to [-π, π] at the residual site (`measurements.wrap_measurement_residual`); 1-D angular residuals wrap the same way.

Force-include: UWB `update_scheduled_pairs` always keeps in-range pairs involving `peer_type == "entrance"` even when `max_neighbors_per_drone` would drop them. Drone–drone k-cap is unchanged.

Gauge-age floor: `estimator.gauge_age_q_m2_per_s: 0.05` is common-mode process noise. Relative **CI** updates cannot reduce a position variance below `min(P_i, P_j)` on that axis (naive fusion is left unfloored for the P2-4 overconfidence test). Gate: `disable_entrance` → absolute σ grows (`stress.py` 3d, `ekf.py` 17e). `max_cov_p_m: 2` still trips diverge; that is the abort, not a plateau at centimetre σ.

Live re-run (`out/swarm_loc_eval/metrics_6_1.json`): ATE RMSE 0.32 / 0.29 / 0.38 m, mean NEES 1.64 / 1.09 / 2.76, NEES-in-95 0.99 / 1.00 / 0.96, no diverge. Entrance edges are **range-only** (bearing count 0). cf_2 yaw RMSE 173° remains. Mutual-yaw NIS is huge (most of the 0.33 reject rate). Full write-up: plan **§10**.

## P2-8 — stress + ablations are offline; live knobs are files

Blocking gate: `python3 perception/swarm_loc/stress.py --selftest` and `python3 eval_scripts/run_ablations.py --selftest`.

Same knobs as the plan (empty `static_peers`, `occluder_boxes`, RIO `dropout_rate`, drop-middle inactive). Live copies: `python3 eval_scripts/run_ablations.py --write-live-configs DIR`.

Error vs hops (4-drone line, hop 1 = nearest the entrance): `{1: 0.076, 2: 0.142, 3: 0.113, 4: 0.151}` m. Not strictly monotonic; hop 1 is the best. Condition 2 (`uwb_range_no_rio`) is *supposed* to be overconfident — that is the degeneracy evidence, not a filter bug. Condition 6 (minus mutual yaw) matches full on this corridor (`n_mutual_yaw_pairs_per_s=0`). 2× UWB noise did not flip which ablation wins.

## 2026-08-27 — R1/R2/R3: observer misattribution, entrance-observed bearings, π-flip guard

### R1 — observer-misattribution bug in `swarm_loc_node._apply_uwb_rows` (fixed)

The bearing-cache / rebroadcast block ran for EVERY row **before** the
`observer_id != self.cf_id` check. Every drone subscribes to
`/uwb/peer_1000/edges`, so drone *i* cached the ENTRANCE's bearing to drone
*j* as its own bearing to *j* (wrong frame, wrong observer yaw) and
rebroadcast it labelled `observer_id=i` with its own ψ. Peers then consumed
garbage `reciprocal_relpos` / `mutual_yaw` measurements — the source of the
hot NIS types in `metrics_6_1.json` (mutual_yaw 63520/87119 rejected,
reciprocal_relpos 14267/23977). Fix: rows are routed first by
`classify_uwb_row(observer, peer, cf_id, entrance_id)` →
`own | entrance_observed | drop`; only `own` rows may touch `_own_bearing`,
`_pending_rebroadcast`, and the bearing/az-only counters. Regression checks
4–5f in `swarm_loc_node.py --selftest`.

### R2 — entrance-observed bearing edges are now consumed (plan §4.3e)

Previously all `/uwb/peer_1000/edges` rows were dropped (observer != self).
Now rows with `observer_id == entrance.device_id` and `peer_id == self`:

- **Full bearing (az+el finite)** → new measurement
  `measurements.anchor_observed_relpos`: h = R_aᵀ(p_i − p_a), ∂h/∂p_i = **+R_aᵀ**
  (sign opposite to model (a)), zero wrt ψ_i. Direct EKF update against a
  known landmark, **not** CI (the entrance broadcasts no state). R = per-edge
  spherical→Cartesian propagation + `entrance.sigma_m²·I` (added in
  `ekf._measurement_R`, isotropic so rotation-invariant). NIS type name
  `entrance_obs_relpos` — the plan-suggested `entrance_relpos` was already
  taken by the drone-observed model (a) to the entrance, so a distinct name
  keeps NIS-by-type interpretable. Also paired with the drone's own bearing
  to the entrance (if any, within `mutual_yaw_max_dt_s`) into a D11 mutual-yaw
  update with ψ_partner = `entrance.yaw_deg` (config, default 0), roll=pitch=0,
  zero partner uncertainty, direct update, name `entrance_mutual_yaw`. This
  pins ABSOLUTE yaw for drones in the entrance cone. A one-sided
  entrance-observed bearing constrains position only — no yaw measurement is
  invented from it.
- **Azimuth-only (z NaN)** → skipped, counted (`n_entrance_az_only`).
- **Range-only** → skipped: the drone's own `entrance_range` already carries
  that physical exchange; consuming both would double-count it.

Config: `entrance.device_id: 1000` and `entrance.yaw_deg: 0.0` added to
`configs/estimation/swarm_loc.yaml`. The device id is now **estimator**
config (not read from `uwb_pdoa.yaml`): §4.3e requires subscribing to
`/uwb/peer_<id>/edges`, whose id `estimator_subscription_topics` previously
hardcoded; it now reads the same config value. Entrance-observed rows cannot
be identified by `FLAG_PEER_IS_SURVEYED` (that flag describes the row's PEER,
which here is the drone), hence the observer-id mechanism.
`ablation.disable_entrance` disables the whole path; the D16 NIS gate applies
to every new measurement. Jacobian + hand-case selftests: `measurements.py`
checks 16–16k (100 randomized geometries vs numdiff to 1e-6).

### R3 — π-flip guard (`ekf.YawModeGuard`)

Truth-free detector + covariance-only recovery for the cf_2 ~175° yaw mode:
a windowed acceptance rate of bearing-family measurements (`relpos`,
`reciprocal_relpos`, `mutual_yaw`, `entrance_relpos`, `entrance_obs_relpos`,
`entrance_mutual_yaw`). If ≥ `yaw_mode_min_attempts` (20) attempts fall in a
`yaw_mode_window_s` (10 s) window with accepted fraction <
`yaw_mode_accept_frac` (0.2) while yaw σ < `yaw_mode_sigma_deg` (10°), the
node logs `YAW_MODE_SUSPECT` and inflates P[ψ,ψ] to
`yaw_mode_reset_sigma_deg`² (60°; diagonal raise keeps P PSD, then
`project_psd`). No hard yaw reset, no truth read; a
`yaw_mode_cooldown_s` (30 s) cooldown prevents thrash. All six knobs live in
`configs/estimation/swarm_loc.yaml` under `estimator:`. Deterministic
recovery selftest: `ekf.py` checks 25–26b (yaw off by 0.97π with 2° σ →
measurements NIS-rejected → guard fires → post-inflation acceptances → yaw
error < 20°; trigger count stays 1 under cooldown).

Note: entrance-observed rows are logged to the NIS log (per-type stats) but
not to the `uwb` array of the measurement `.npz` — `central_reference.py`'s
schema assumes drone-observed rows; extending it is out of this patch's scope.

## 2026-09-09 — cf_2 yaw excursions: lead-drone yaw is bearing-starved; RIO stub white noise was publish-rate-dependent (fixed)

Post-R1/R2/R3 run (`out/swarm_loc_eval/`, ATE 0.13–0.17 m): cf_2 yaw RMSE
30.5° with excursions to ~60° (t≈58, t≈79–84). Diagnosis from
`out/swarm_loc_logs/cf_*.npz`:

- **Not the guard, not bad acceptances.** `n_yaw_mode_triggers = 0` on all
  three drones; NIS reject rate 0.3% overall; no π mode.
- **cf_2 is the LEAD drone** (spawn x = 3.0 m, flies furthest in). Truth
  geometry puts both peers at az ≈ ±180° in its body frame for the whole run
  — outside the ±45° AoA cone — so cf_2 logs **zero** own-bearing rows
  (`relpos` counts: cf_0 3185, cf_1 1470, cf_2 0) and therefore zero
  `mutual_yaw` pairs. Its 3042 `reciprocal_relpos` rows (peers' bearings TO
  it) and 1620 `entrance_obs_relpos` rows have ∂h/∂ψ_i = 0: no direct yaw
  information at all. `entrance_mutual_yaw = 0` is legitimate, not a pairing
  bug: every own entrance edge in every log is range-only (the entrance is
  astern of all drones for the whole run), so the §4.3e "own bearing to the
  entrance" precondition never holds. cf_2's only yaw feedback is the p–ψ
  cross-covariance built by body-frame odometry while translating; it does
  work (t=66→69 the estimate recovers −30°→−2.5° while integrated RIO yaw
  stays at −48°) but is weak. This is fundamental observability of the
  scenario, not an estimator bug.
- **Magnitude driver (fixed): `RioStubEngine.corrupt` injected `sigma_p` /
  `sigma_psi_deg` white noise once per odom message with no √dt scaling.**
  Gazebo odom runs at ~182 Hz (nominal filter rate is 50), so the integrated
  yaw walk was 0.5°·√182 ≈ 45 deg²/s — matching the observed σ_ψ growth
  (5.2°→12.1° over t=48–51 ⇒ ~40 deg²/s) and cf_2's error (est −43.3° at
  t=57 vs RIO-integrated −45.2°). Integrated RIO yaw drifts −45…−77° for ALL
  drones; cf_0/cf_1 correct it through their own bearings, cf_2 cannot. The
  filter itself was honest (Q matched the injection; cf_2 mean NEES 3.34,
  |err|/σ_ψ ≈ 2 at the peaks).

Fix: `rio.noise_ref_rate_hz: 50.0` (new key in
`configs/estimation/swarm_loc.yaml`); `corrupt()` scales each sample's white
p/ψ noise by √(dt·ref_rate) and the advertised per-delta cov axes 0–3 by
dt·ref_rate, so the walk per unit time is publish-rate independent. At
dt = 1/ref_rate behavior is bit-identical to before (all offline selftests use
dt = 0.02), and the cov travels inside the RIO message so estimator Q stays
matched. At 182 Hz this cuts the injected yaw-walk variance 3.64× (45 →
12.5 deg²/s, i.e. the configured 0.5°-per-50 Hz-frame spec); expected cf_2
yaw excursions shrink ~1.9×. New checks in `rio_stub.py --selftest`
(cov-at-ref-rate, cov scaling, 50-vs-200 Hz walk equality).

Residual: even at 12.5 deg²/s the white per-frame jitter dominates
`yaw_walk_deg_per_min: 3.0` (0.0025 deg²/s) as the yaw drift driver, and the
lead drone's yaw remains unobservable through the bearing family. In-scope
remedy candidates (not implemented — scenario/hardware changes): a rear-facing
AoA antenna or wider cone (explicitly out of bounds per D11 note), periodic
yaw glances by the lead drone, or a formation where the leader occasionally
sees a peer. Estimator-side there is nothing left to correct: the filter's
reported σ_ψ tracks the truth error honestly.

### 2026-09-09 — Time-resolved error-vs-hops (`eval_scripts/eval_6_1.py`)

The §6.1 headline metric was run-aggregate: one BFS over every UWB edge in
the whole run (`hops_from_uwb`), so a drone that ranged the entrance even
once was hop 1 for the entire run. On the 3-drone tunnel run this was not a
bug — every drone held a direct entrance link at ~28 edges/s for the whole
flight (entrance range 1.5–6.0 m vs `max_range_m: 30`, only a 2.5 s gap at
t≈46–48 s with no estimate samples), so hop ≥ 2 was physically impossible —
but the aggregate would hide the interesting regime on P2-8 corridor runs.

Added, keeping the old aggregate keys unchanged (`hops`, `ate_vs_hops_m`):
`hops_vs_time` (per-1 s-window BFS over the edges active in that window;
window matches the existing `entrance_edges_vs_time` binning) and
`error_vs_hops_time` (each error sample bucketed by its instantaneous hop;
RMSE/p50/p95/n per bucket, unassigned counted). Report key
`ate_vs_hops_time`, npz keys `hops_time` / `ate_vs_hops_time_rmse` /
`ate_vs_hops_time_n` / `hop_window_s`. Plot `01_error_vs_hops.png` (and the
dashboard panel) now draws the time-resolved curve with n per bucket and the
aggregate as a dashed overlay. Selftest checks 9b and 11–11f cover a drone
that is hop 1 early and hop 2 late. Exercising hop ≥ 2 for real needs a
longer corridor / more drones / smaller entrance range (scenario change —
not made here).

### 2026-09-11 — Hollow-run tripwires in the P2-5 gate (`eval_scripts/swarm_loc_gate.py`)

The 2026-09-11 `tunnel/triangle_forward` scored run PASSED the gate while the
EKF consumed nothing: `n_update=0`, `n_uwb=0`, `cpu_per_step=0.0`, empty
`rio`/`uwb`/`estimate` buffers in every `cf_*.npz`, EKF publish stamps frozen
at 0, `hops=-1` for all drones, mix fractions all 0, and mean NEES 170–2360
with `frac_nees_in_95=0.0`. ATE looked plausible only because the estimates
were pose-initialization plus nothing. The gate's checks (estimate rate,
finiteness, no-truth-subs, flight) could not see any of that.

Added blocking checks (each a named entry in the `checks` dict; all must
pass; thresholds are commented module constants — the gate has no config/CLI
threshold pattern):

- `rio_alive_cf_<i>` — the recorder now also subscribes `/cf_<i>/rio/delta`:
  rows received, `valid==1` fraction ≥ 0.5, stamps strictly advancing
  (≥ 90% of consecutive diffs > 0), stamp span ≥ 80% of the scored duration,
  mean rate ≥ 5 Hz. Applies equally to stub-RIO runs (the stub publishes
  valid deltas at ~50 Hz).
- `ekf_alive_cf_<i>` — RAW `/cf_<i>/swarm_loc/estimate` stamps (captured
  before the recorder's receive-time substitution) span ≥ 80% of the scored
  duration, and `seq` increments on ≥ half the rows.
- `logs_intact` — every `cf_<i>.npz` in the configured log dir exists,
  `np.load`s, and has non-empty `rio` and `estimate` arrays. The gate does
  not control the `--log-measurements` path (phase0_gate.sh does), so it
  verifies the files at scoring time.
- `uwb_consumed` — total `n_update` > 0 AND total UWB edge rows > 0 AND each
  drone's `n_update` ≥ 1 × scored seconds (edges arrive ~10+/s per pair, so
  1 Hz is an order of magnitude of slack; 0 never passes).
- `hops_valid` — computed hops ≠ -1 for every drone (measurement graph
  actually connects each drone to the entrance).
- `nees_sane` — mean NEES per drone finite and < 50. Purely an anti-garbage
  tripwire, not a consistency claim; real consistency reporting stays in the
  eval_6_1 report unchanged.
- `mix_nonzero` — `n_uwb` > 0 and `frac_bearing + frac_range_only` > 0.

The log-based checks run whenever `--logs`/`--eval-dir` is configured (every
scored `--scenario` run sets both); a run without either prints a loud
NOT-scoreable warning. `main()` was restructured so eval_6_1 runs before the
PASS/FAIL block and its report feeds `hops_valid`/`mix_nonzero`/`nees_sane`;
a missing/failed eval now FAILS those checks instead of being silently
skipped. The checks are pure functions over collected arrays/dicts (no
rclpy); `--selftest` (now 28 checks) feeds a synthetic replica of the
2026-09-11 hollow state and asserts each new check individually fails, plus a
healthy synthetic state where all pass. All seven were also verified to fail
against the real 2026-09-11 artifacts on disk.

## 2026-09-11 (later) — first live real-RIO run: radar was silent; two plugin bugs

The hardened gate's tripwires did their job: the previous `tunnel/triangle_forward`
run was hollow because **`/cf_*/radar/points` never published a single message**,
so `rio_bridge` had nothing to fuse, the EKF never propagated
(`n_prop=0`), and every downstream count was zero. The plugin was loaded and
the topics were advertised — `ros2 topic list` showed all three, and the three
`radarays_gz2_node_<entity>` ROS nodes existed — which is exactly why this was
invisible without instrumentation.

### Instrumentation added (`RadarSensorSystem.cpp/.hpp`)

`gzmsg`/`gzerr` liveness counters: `map ready (…, imported|shared)` once per
instance, `PreUpdate live on <topic>` on the first physics callback, and
`<topic> scans=N published=N jobs=N` on the first scan and every 200th. Map
init and `simulate()` are now wrapped in try/catch — an exception in the worker
previously took the whole sim down via `std::terminate`. These three counters
separate "PreUpdate never ran" from "worker never produced" from "produced but
never published", which is what made both root causes findable.

### Bug 1 — Embree map built on the worker thread never returns

The off-thread refactor had moved `rm::import_embree_map()` into the worker.
Live evidence: all three workers parsed the whole mesh (2970 `make_embree_scene`
lines, i.e. 990 × 3) and then hung forever — no `map ready`, ever — while
`PreUpdate live` printed normally on all three topics. Embree's BVH build
(`rtcCommitScene`) runs on TBB, and this host exposes two `libtbb.so.12`
(Embree 4 links `/usr/local/lib`'s oneTBB 2021.12; the system has 2021.5 — the
build even warns that one may hide the other). Committing from a non-main
thread deadlocks.

Fix: build the map in `Configure()`, on the Gazebo main thread, once. The
per-step raycasting — the freeze this design exists to prevent — **stays on the
worker**; `rtcIntersect1` is TBB-free and thread-safe. Do not move the import
back.

### Bug 2 — one Embree map per drone does not fit in RAM

With the import back on the main thread it still never finished: a single
`cave_world.obj` map costs **~5.3 GB RSS**, and each plugin instance was
importing its own copy, so three drones needed ~16 GB on a 7.9 GB WSL host.

Fix: a process-wide `mesh path → EmbreeMapPtr` cache (`acquire_map`, guarded by
a mutex) in `RadarSensorSystem.cpp`. The map is immutable after commit and
Embree ray queries are thread-safe, so one copy serves every drone. The log
says which instance paid: `imported` for the first, `shared` afterwards.

Cost: gz load time is now dominated by that one import (~3 min for
`cave_world.obj` off the `/mnt/d` 9p mount); `Simulation ready` lands at
~170 s instead of ~60 s. The `gz service …/create` call for drone 0 times out
at its 5 s budget while the import runs; that is harmless — the entity is still
created and the later spawns queue behind it.

### Live numbers after the fix (phase0 stack, headless, 3 drones)

Per drone, 20 s window: `/cf_<i>/radar/points` **3.6 Hz wall / ~10 Hz sim**,
fields `['x','y','z','intensity','doppler']`, 33–43 usable points per scan
(vs `doppler_min_points: 8`); `/cf_<i>/imu` ~299 Hz with a valid orientation
quaternion (`orientation_covariance[0] != -1`), so RIO needs no attitude
filter added; `/cf_<i>/rio/delta` **valid_frac = 1.000**, stamps strictly
advancing, ~9.9 Hz in sim time. No RIO tuning parameter was changed — the
Doppler solve accepts every scan at the stock `doppler_min_points: 8` /
`doppler_max_condition_number: 20`.

`doppler ≡ 0` and `|dp| ≡ 0` while the drones sit on the ground is correct, not
a fault: the plugin's Doppler is the projection of the sensor's own velocity,
which is zero when parked.

Scored run (`cf_*.npz`, was all-zero before): rio rows 852/855/856, uwb rows
4505/4504/4514, estimate rows 11401/11337/11451, `n_update`
8966/10443/11928, `n_prop` = rio rows, `n_diverged=0`. `metrics_6_1.json`:
`hops={0:1,1:1,2:1}`, mix `frac_bearing=0.333` / `frac_range_only=0.667`,
`n_uwb=11680`, NIS reject rate 0.0023. Gate: `logs_intact`, `uwb_consumed`,
`hops_valid`, `mix_nonzero`, `no_truth_subs`, `finite`, `non_diverged` and all
per-drone `rate_hz`/`samples` **PASS** (estimate ~45 Hz).

### Two blockers remain — neither is the radar/RIO chain

1. **cflib cannot open 3 SITL links in parallel from a non-interactive
   `wsl -e bash -lc` shell.** A single-link probe to `udp://127.0.0.1:19850`
   connects fine; `Swarm.open_links()` for all three times out at 120 s, with
   or without a preceding probe. So `flight` FAILs and the gate falls back to
   recording on the ground, which makes ATE/RPE/yaw/NEES `nan` (n=0 paired
   samples) and trips `nees_sane`. This is an environment limitation, not a
   pipeline defect — the fly must be launched from an interactive WSL shell.

2. **`rio_alive`/`ekf_alive` compare a SIM-time span against a WALL-clock
   duration.** Every quality criterion passes (`valid_frac=1.00`,
   `adv_frac=1.00`, `rate=9.9 Hz ≥ 5`); only `span` fails: 16.2 s of sim time
   vs `need >= 36.0 s` (0.8 × the 45 s scenario duration). RIO stamps are radar
   header stamps in sim seconds, `duration_s` is wall seconds, and this host
   runs at **RTF ≈ 0.36**, so the check cannot pass at any pipeline quality.
   `rio_stub` stamped from `/cf_*/odom` headers — also sim time — so this
   predates real RIO; it was simply never reachable while the run was hollow.
   The gate was left **unmodified**: this is a unit bug in the check, not
   strictness, and the fix is a judgement call for the owner. Suggested
   unit-correct form, preserving the intent exactly ("RIO was alive for ≥80% of
   the run"): compare the RIO stamp span against the sim-time span the recorder
   actually observed — the `/cf_*/odom` header-stamp span it already collects —
   instead of against `duration_s`. Do not lower `RIO_SPAN_FRAC_MIN`.

## 2026-09-11 — `rio_alive`/`ekf_alive` unit bug fixed (sim-time reference span)

Fixes blocker 2 above. `eval_scripts/swarm_loc_gate.py` only; nothing under
`perception/` touched.

**The bug.** `/cf_i/rio/delta` stamps and the raw EKF estimate stamps are SIM
seconds (message headers; plan §9 "sim time ≠ ROS wall time"), while
`args.duration` is WALL seconds. Both checks did
`span >= FRAC_MIN * duration_s`, mixing the two. On a host at RTF ≈ 0.36 a
healthy 45 s wall run advances the sim clock only ~16 s, so the check demanded
≥ 36 s of sim span and could not pass at any pipeline quality — exactly what the
verified-healthy live run hit (`valid_frac=1.000`, `adv_frac=1.00`, ~9.9 sim-Hz,
`n_update` 9k–12k per drone, yet `span=16.2s (need>=36.0s)`).

**The fix.** `RIO_SPAN_FRAC_MIN`/`EKF_SPAN_FRAC_MIN` stay at 0.8 — the intent
("alive for essentially the whole run, not a burst") was correct. Only the
reference changed:

    before:  stamp_span(sim) >= FRAC_MIN * duration_s(wall)
    after:   stamp_span(sim) >= FRAC_MIN * sim_reference_span(sim)

`sim_reference_span()` is built from the `/cf_*/odom` **header** stamps (Gazebo
publishes odom with sim-time headers). The recorder previously discarded them —
`_on_odom` overwrote the stamp with `time.time()` for receive-time pairing — so
it now keeps them in a separate `odom_sim_stamps` list; `self.truth` and the
eval bundle are unchanged. Per drone we take max−min and the reference is the
**max across drones**, so one drone's odom dropping out early cannot shrink the
bar and excuse a dead RIO elsewhere. `main()` prints the reference span and the
implied RTF next to the wall duration.

**Vacuity guards.** A sim reference of ~0 would make `span >= 0.8 * 0` trivially
true, so a hollow run would start passing. `_sim_ref_guard()` runs first in both
checks: if the reference is non-finite or below `SIM_REF_SPAN_FLOOR_S = 5.0`
(missing odom, frozen sim clock, or a run so short liveness is unverifiable) the
check FAILS with an explicit message instead of comparing against ~0. 5 s is far
below the shortest scored scenario (45 s wall ≈ 16 s sim at the worst observed
RTF) and far above startup jitter. Every other criterion is unchanged:
`RIO_VALID_FRAC_MIN=0.5`, `RIO_ADV_FRAC_MIN=0.9`, `RIO_RATE_MIN_HZ=5.0`,
`EKF_SEQ_FRAC_MIN=0.5` and the seq-must-increase test.

**Rate units.** `hz = (n-1) / stamp_span` was already per SIM second (both
operands are sim-time), so no numeric change — but it was undocumented and one
symbol away from the same bug. This is now stated explicitly in a UNITS block:
sim-Hz is the physically meaningful rate for a sensor-driven pipeline, because
RIO fires once per radar scan and scans are scheduled in sim time, making the
rate invariant to host speed. Real RIO is ~9.9 sim-Hz (~3.6 wall-Hz at RTF
0.36), so the 5.0 floor keeps a comfortable margin; a genuinely slow 2 sim-Hz
RIO still fails (selftest 9o).

**Selftest: 44 checks, ALL PASS** (was 28; all 28 retained unchanged). New:
`9/9d/9g` sim-reference-span construction; `9a/9b` the healthy slow-RTF run
(16.2 s sim ref, 45 s wall, 9.9 sim-Hz, valid_frac 1.0) now PASSES both checks;
`9c` asserts the *same data* still FAILS when compared against the wall
duration, pinning the regression; `9e/9f` zero sim reference FAILS;
`9h/9i` the floor boundary; `9j/9k` RIO/EKF dying at 30 % of the sim reference
FAILS; `9l/9m/9n` the 2026-09-11 hollow run (stamps frozen at 0, zero rows)
still FAILS against a healthy sim reference.

## 2026-09-11 (later) — one clock for ATE, flight-window liveness, config-derived seq bar

The ~18:26 `tunnel/triangle_forward` fly succeeded mechanically (3 drones armed
and flew, UWB + real RIO in the filter, estimates ~44 Hz, hops = 1, mix ~20 %
bearing / 80 % range-only, 1498 entrance-observed updates, 864 reciprocal, logs
intact, RIO `valid_frac=1.00` at ~9.4 sim-Hz with `adv_frac=1.00`) yet the gate
FAILed with NaN ATE/RPE/NEES. Three independent bugs, all in
`eval_scripts/swarm_loc_gate.py`; nothing under `perception/` touched and
`eval_scripts/eval_6_1.py` needed no change (the pairing bug was upstream of it,
in the stamps the recorder produced).

### A — truth and estimates were on DIFFERENT clocks (ATE n = 0)

**Before.** `EstimateRecorder._on_odom` overwrote the truth stamp with
`time.time()` (WALL, ~1.7e9 s) — the deliberate "receive-time pairing" added
when EKF stamps were stuck at 0 — and `_on_est` mirrored it with
`if stamp <= 1e-3: stamp = time.time()`. With real RIO working, the EKF rows now
carry proper SIM time (~0–20 s), so `eval_6_1.interp_pose` rejected every row
(`t < ts[0] or t > ts[-1]`), 0 samples paired, ATE/RPE/NEES came out NaN and
`nees_sane` FAILed on the NaN.

**After.** One clock, the odom/EKF **header sim time**:

    _on_odom:  stamp = header.sec + header.nanosec*1e-9   (was time.time())
    _on_est:   row["stamp"] left exactly as published      (substitution deleted)

Wall clock survives only as metadata (`est_meta`/`rio_meta` receive times, the
per-drone `estimate_hz` rate check). Odom messages with no header stamp are
counted in `odom_header_missing`, stamped NaN, and dropped in `dump_eval` so a
NaN can never corrupt `interp_pose`'s `searchsorted`. The eval-bundle schema
(`TRUTH_DTYPE`, `STATE_DTYPE`, `write_eval_bundle`) is unchanged — only the
values on the `stamp` field changed clock.

**The fallback is not silently gone, it is a FAILURE.** The receive-time
substitution existed for dead/frozen EKF stamps; deleting it without a guard
would just move the silence. New pure check `pairing_clock_check(truth_stamps,
est_stamps)`, wired as `pairing_clock_cf_<i>`, refuses three distinct ways:

* estimate stamps frozen or `<= EST_STAMP_LIVE_MIN_S` (the hollow-run case) →
  FAIL, message states outright that it is *refusing* the wall-clock
  substitution because it "would pair sim truth against wall estimates and emit
  a meaningless ATE";
* truth stamps frozen/absent → FAIL;
* ranges disjoint or overlapping less than `CLOCK_OVERLAP_FRAC_MIN = 0.5` of the
  estimate span → FAIL, message says `DIFFERENT CLOCKS (wall vs sim)`.

A second, symptom-level tripwire `ate_paired` asserts every drone's
`per_drone[i]["n"] > 0` in the eval report, so an n = 0 ATE can never again be
reported as a number-shaped result.

### B — `rio_alive`/`ekf_alive` compared against the WRONG window

**Before.** `sim_reference_span()` = max `/cf_*/odom` header-stamp span = 53.5 s,
because Gazebo publishes odom for the recorder's *entire life* (wait-for-truth,
pre-arm hover, takeoff, flight, teardown). RIO/EKF only come up for the flight:
spans 19.6 / 19.9 s. `0.8 x 53.5 = 42.8 s` → FAIL on a perfectly healthy RIO.

**After.** The reference is the **flight window**. `EstimateRecorder` tracks
`_last_odom_sim` (most recent odom header stamp on any drone) and captures
`flight_sim_t0` / `flight_sim_t1` at the two instants the gate starts and
finishes the scripted path inside `run_flight` (the `--no-fly` and
radios-did-not-connect recording paths mark their own record window the same
way). `flight_window_span(t0, t1)` returns the span, or **0.0** when a marker is
missing, non-finite, or the window is zero/backwards.

    before:  rio_span >= 0.8 * whole_odom_span      (53.5 s — hover included)
    after:   rio_span >= 0.8 * flight_window_span   (20 s — scripted path only)

**Nothing was weakened.** `RIO_SPAN_FRAC_MIN`/`EKF_SPAN_FRAC_MIN` stay 0.8,
`RIO_VALID_FRAC_MIN=0.5`, `RIO_ADV_FRAC_MIN=0.9`, `RIO_RATE_MIN_HZ=5.0`
(sim-Hz) are untouched, and `SIM_REF_SPAN_FLOOR_S = 5.0` still runs first in
`_sim_ref_guard` — now applied to the flight window, so an unmarked, zero or
sub-5 s flight window FAILS outright instead of making `span >= 0.8 * 0`
vacuously true. `sim_reference_span()` is kept and still printed, as diagnostic
context (whole-run odom span and the implied RTF) only.

### B2 — `ekf` `seq_inc_frac = 0.22` vs a fixed `EKF_SEQ_FRAC_MIN = 0.5`

**Investigated first; the hypothesis is confirmed in the code.**
`perception/swarm_loc/swarm_loc_node.py` (read-only) publishes the estimate on
the estimator tick but bumps `seq` only on the broadcast tick:

    node.create_timer(1.0 / max(rate, 1.0), self._on_tick)             # rate_hz = 50
    node.create_timer(1.0 / max(bc_hz, 1.0), self._on_broadcast_tick)  # 10 Hz

    def _on_tick(self):            row = state_row_from_filter(self.st, self._seq, ...)
                                   self._pub_est.publish(pack_state([row], ...))
    def _on_broadcast_tick(self):  self._seq += 1

So a *healthy* run shows a new seq on only `broadcast_rate_hz / rate_hz =
10 / 50 = 0.20` of consecutive `/cf_i/swarm_loc/estimate` rows — the observed
0.22 exactly. The 0.5 bar was unpassable by construction, not a symptom.

**After.** `EKF_SEQ_FRAC_MIN` is replaced by a config-derived expectation with
margin. `expected_seq_inc_frac(cfg)` reads the same `swarm_loc.yaml` the gate
already loads and returns `min(1.0, bc_hz / rate_hz)`, falling back to **1.0**
(the old, strictest behaviour — never a weaker bar) when the config is
unreadable or non-positive. `ekf_alive_check` then requires

    seq_inc_frac >= max(EKF_SEQ_FRAC_FLOOR 0.02, EKF_SEQ_FRAC_MARGIN 0.5 * expected)

i.e. 0.10 at the shipped 50/10 config, plus the two unchanged conditions
`seq.max() > seq.min()` and, new, `seq_dec_frac <= EKF_SEQ_DECREASE_FRAC_MAX
= 0.01` (seq must be non-decreasing; ~1 % tolerated for reordered arrivals).
A frozen seq gives frac 0.0 and `max == min` → still FAILs on both counts, at
any expectation, including a deliberately tiny one.

### Selftest

`python eval_scripts/swarm_loc_gate.py --selftest` → **87 checks, ALL PASS**
(was 44; all 44 retained unchanged). `eval_6_1.py --selftest` still 25/25 —
it was not modified.

New cases, all pinning failing-before / passing-after in the suite the way 9c
does:

* `10`–`10b2` `flight_window_span` construction: healthy, unmarked, half-marked,
  backwards, zero-length.
* `10c2`/`10c3` the identical healthy live arrays (odom span 53.5 s, RIO 19.6 s
  at 9.4 sim-Hz, EKF 19.9 s at 44 sim-Hz) **FAIL** against the old whole-odom
  reference; `10d`/`10e` the same arrays **PASS** against the 20 s flight window.
* `10f`/`10f2` RIO/EKF dying halfway through the flight window → FAIL.
* `10g`–`10h2` flight window unmarked (0 s) or under the 5 s floor → FAIL, not a
  vacuous pass.
* `10i`–`10i3` the 2026-09-11 hollow run (stamps frozen at 0, zero rows) still
  FAILs every liveness check against a good flight window.
* `11`–`11b2` `expected_seq_inc_frac` = bc/rate, clamped at 1.0, falling back to
  1.0 on bad config; `11c` the synthetic 10 Hz-seq-on-50 Hz-estimates series
  reproduces frac ≈ 0.2; `11d` it **FAILS** under the old fixed 0.5 bar;
  `11e` it **PASSES** under the config-derived bar; `11f`/`11g` frozen seq FAILs
  at any expectation; `11h` decreasing seq FAILs.
* `12`/`12a` sim/sim series pair in `eval_6_1.paired_errors` (n > 0) and
  `pairing_clock_check` passes; `12b`/`12b2` wall truth vs sim estimates pairs
  **0** rows and yields the observed NaN ATE; `12c`/`12c2` `pairing_clock_check`
  FAILs it with `DIFFERENT CLOCKS`; `12d`/`12d2` frozen/zero estimate stamps FAIL
  and the message explicitly refuses the wall-clock fallback; `12e`–`12j` frozen
  truth, empty series, sub-threshold partial overlap FAIL, wider-than-estimate
  truth passes.


---

## 2026-09-11 — BUG B3: flight-window markers captured 0.0 on a healthy run (`eval_scripts/swarm_loc_gate.py`)

**Symptom.** The scored `tunnel/triangle_forward` run printed
`flight window start (sim) = 0.0` / `end (sim) = 0.0`, so
`flight_window_span()` returned 0.0 and `_sim_ref_guard` failed six checks
(`rio_alive_cf_*`, `ekf_alive_cf_*`) while `flight`, `pairing_clock_cf_*`,
`ate_paired`, `logs_intact`, `uwb_consumed`, `hops_valid`, `mix_nonzero` and
every `rate_hz`/`samples` check PASSED. The drones really flew.

**Root cause — marker plumbing, not a threshold.** `EstimateRecorder._on_odom`
held the marker as a plain last-write-wins scalar:

    self._last_odom_sim = stamp          # every odom callback, any stamp

`/cf_*/odom` is not uniformly sim-stamped: the stream interleaves messages whose
header stamp is 0 (which is why every drone's whole-run odom span reads
`min = 0.000`, truth `[0.000, 111.734]s`). Whichever message happened to arrive
last before `run_flight` called `mark_flight_start()` / `mark_flight_end()`
defined the window, and that was usually a zero-stamped one — so both markers
read 0.0 even though the sim clock had reached ~92 s / ~111.6 s. The
`MultiThreadedExecutor` daemon thread makes the race the normal case, not the
exception.

**Fix.** The marker is now a locked HIGH-WATER MARK over odom header stamps that
are actually set (`ODOM_SIM_STAMP_MIN_S = 1e-3`): `_note_odom_sim()` only
advances it, `sim_now()` reads it under the same lock the callbacks write under,
and `mark_flight_start/end` sample `sim_now()`. Zero/unset/non-finite/out-of-
order stamps can no longer clobber it. Recorder state setup was split into
`EstimateRecorder._init_state()` so the capture path is constructible without
rclpy.

**No threshold or guard changed.** `RIO_SPAN_FRAC_MIN`/`EKF_SPAN_FRAC_MIN` 0.8,
`SIM_REF_SPAN_FLOOR_S` 5.0, `RIO_VALID_FRAC_MIN` 0.5, `RIO_ADV_FRAC_MIN` 0.9,
`RIO_RATE_MIN_HZ` 5.0, the config-derived `expected_seq_inc_frac` and the
hollow-run FAIL behaviour are byte-identical. An unset or frozen sim clock still
yields a 0.0 window and still FAILS. Expected on the next flight: window
~19.4 s sim (~[92.2, 111.6]), RIO/EKF spans clearing the 0.8x bar.

**Selftest hole closed.** The 87-case suite passed while this shipped because it
only exercised the pure helper `flight_window_span(t0, t1)` with hand-passed
values — the capture path was never run. New cases `10j`–`10m3` build a real
`EstimateRecorder` (via `__new__` + `_init_state`, no rclpy), feed synthetic
`nav_msgs/Odometry` stand-ins with advancing header stamps *plus a zero-stamped
intruder immediately before each mark*, invoke `mark_flight_start/end` the way
`run_flight` does, and assert the markers are non-zero, bracket the fed stamps,
and produce the ~19.4 s window; `10k5` pins that last-write-wins would have
captured 0.0; `10j2`/`10m` pin that an unmarked and a frozen-clock run still
FAIL; `10m2` header-less odom is counted and leaves the marker unset; `10m3`
pins the high-water (non-regressing) behaviour.

`python eval_scripts/swarm_loc_gate.py --selftest` -> **99 checks, ALL PASS**
(was 87; all 87 retained unchanged). Scope: `eval_scripts/swarm_loc_gate.py`
only; nothing under `perception/` or `out/` touched; not committed.


---

## 2026-09-11 — BUG B4: truth pairing read an unsorted, half-zero-stamped array (`eval_scripts/eval_6_1.py`)

**Symptom.** The scored `tunnel/triangle_forward` run on real RIO reported
ATE 0.99 m (hop-1 rmse 1.007, n=8348) and mean NEES 344 / 492 / 135 against an
expected ~3, while every *online* diagnostic said the filter was healthy:
`nis_by_type` means were O(1) for relpos (1.54), entrance_range (1.19),
entrance_obs_relpos (3.06), range (0.68), range_rate (0.01), reciprocal_relpos
(1.44); aggregate NIS reject rate 0.0066; `n_diverged` 0. A filter overconfident
by 100x in position cannot have healthy innovations — that contradiction is the
tell, and it points at the evaluator rather than the estimator.

**Root cause.** `interp_pose` is a binary search (`np.searchsorted`) with a span
guard on `ts[0]` / `ts[-1]`. Both are meaningful only on a sorted array. The
recorded truth array is neither sorted nor uniformly stamped: `/cf_*/odom`
interleaves zero-stamped (pre-clock / unstamped bridge) messages with
sim-stamped ones — the same stream shape BUG B3 diagnosed for the flight-window
markers, but the *array itself* was never cleaned. On the scored run `truth.npz`
held 11721 / 11643 / 11462 rows per drone, of which 6464 / 6440 / 6281 carried
stamp 0.0, with 4479 / 4451 / 4388 descending steps and only 3894 real samples.
Two failure modes follow:

* array ends on a real stamp (cf_2) -> `searchsorted` returns an arbitrary
  index, every estimate is paired with an essentially random truth pose, and the
  metrics inflate;
* array ends on a zero-stamped row (cf_0 / cf_1 on the 2026-09-11 re-run) ->
  `t > ts[-1]` rejects **every** estimate: n=0, ATE/NEES **NaN**.

**Measured offline on the recorded bundle** (`eval_6_1.paired_errors`, identical
code, only the truth array sanitized):

| drone | ATE as shipped | ATE fixed | NEES as shipped | NEES fixed | frac_nees_in_95 fixed | yaw fixed |
|---|---|---|---|---|---|---|
| cf_0 | 1.145 | **0.163** | 344.2 | **3.7** | 0.861 | 0.38 deg |
| cf_1 | 1.110 | **0.172** | 491.7 | **4.0** | 0.886 | 0.98 deg |
| cf_2 | 0.710 | **0.438** | 134.9 | **23.4** | 0.750 | 3.64 deg |

The "as shipped" column reproduces `metrics_6_1.json` to three decimals, so this
is the path the gate actually ran. The 2026-09-11 re-run (truth window
[282.8, 304.6] s) shows both modes: cf_0 / cf_1 NaN as shipped vs ATE 0.234 /
0.497 and NEES 6.4 / 23.8 fixed; cf_2 ATE 1.516 -> **0.418**, NEES 578 ->
**16.6**. The 0.13-0.17 m stub-era numbers were therefore never lost — cf_0 /
cf_1 on real RIO sit at 0.16-0.23 m.

**Fix.** New `sanitize_truth()` — drop non-finite and `<=
TRUTH_SIM_STAMP_MIN_S = 1e-3` stamps, stable-sort, keep the first of each
duplicate stamp — applied once per drone in `paired_errors`, in
`centroid_vs_shape`, and in `write_eval_bundle` so the persisted `truth.npz` is
usable by any offline consumer. It removes only rows that were never valid truth
samples; no threshold, no alignment, no metric definition changed. `interp_pose`
now documents that it requires a sanitized series.

**Selftest.** `python eval_scripts/eval_6_1.py --selftest` -> **41 checks, ALL
PASS** (was 25; all 25 retained unchanged). New `12`-`12o` rebuild the real
recorder shape by interleaving zero-stamped rows into the synthetic truth series:
`12`-`12b` the clean series still pairs, ATE == the injected offset and NEES ~ 3;
`12c` the interleaved array really is unsorted and ends on a zero stamp;
`12d`-`12g` sanitation drops / sorts / dedups and preserves every real pose;
`12h`-`12j` `paired_errors` on the interleaved array now matches the clean result
exactly; `12k` reproduces the old code rejecting ~everything (the NaN mode) and
`12l` the old code inflating ATE by >5x (the garbage-pairing mode); `12m`-`12o`
cover the trailing row, `centroid_vs_shape` and the written bundle.


---

## 2026-09-11 — BUG B5: `rio_bridge` advertised a covariance real RIO cannot honour (`perception/radar_processing/rio_bridge.py`)

**What B4 left behind.** With truth pairing fixed, NEES is 3.7 / 4.0 / 23.4 on
the scored run and 6.4 / 23.8 / 16.6 on the re-run against an ideal 3.0, and
state-level `err/sigma` is 1.2-8.4. That residual is real, and it is the
advertised odometry covariance — so it is fixed by telling the filter the truth
about RIO, not by tuning the estimator. Truth was used **offline only**; no
truth topic is read by `rio_bridge.py` or anything under `perception/swarm_loc/`.

**Measured** (logged `rio` increments differenced against offline truth at the
RIO stamps, both runs, dt = 0.101 s):

* **z.** `delta_p_body.z` is structurally 0 (2D RIO). Per-step z increment error
  std was 0.038-0.069 m against an advertised `SIGMA_VZ_MPS * dt` = 0.0151 m —
  a **2.5-4.6x sigma (6-21x variance) understatement**. Independently, truth
  |vz| RMS was 0.60 / 0.65 / 0.66 and 0.38 / 0.68 / 0.63 m/s with 41-57% of
  steps above 0.15 m/s: these drones take off, bounce and land inside the scored
  window (z sweeps 0.012 -> 1.72 m in ~20 s), so the old 0.15 "near-hover nano
  quad" figure described a flight regime that does not occur here.
* **horizontal.** The bridge passes RIO's own 2x2 velocity-KF covariance through,
  correctly rotated world->body and correctly scaled by dt² — **no missing dt²,
  no frame error and no sign error was found**. But that P is the covariance of
  RIO's estimate under RIO's own tuned noise model: with `imu_process_noise_std`
  0.05 and a 10 Hz Doppler update it settles at sigma_v ~ 0.016 m/s, i.e. ~1.6 mm
  per step. The **measured** per-axis velocity error was 0.02-1.30 m/s (pooled
  RMS 0.56 m/s) and the dead-reckoned horizontal drift 0.8-1.4 m over ~21 s
  against an advertised random walk of ~0.02 m. That is a **~35-60x sigma
  (~1300x variance) understatement**, and it is structural: RIO's KF knows
  nothing about Doppler solve conditioning or the tunnel geometry that makes the
  along-track component nearly unobservable, and the error it makes is
  correlated across steps (lag-1 autocorrelation up to 0.89) while the EKF
  consumes it as white.
* **yaw.** Per-step `dpsi` error / advertised sigma was 0.1-1.6 (scored run) and
  0.0-8.1 (re-run); cumulative yaw error ~0 (IMU-absolute, no drift).
* **scale.** No consistent scale factor found; `SCALE_VAR` left at 1e-8.

**Fix — make the advertised number honest.** `SIGMA_VZ_MPS` 0.15 -> **0.60**
(the measurement). New `SIGMA_V_XY_FLOOR_MPS = 0.50` applied as a per-axis
variance floor on the world-frame horizontal velocity covariance before the
rotation (an isotropic diagonal floor is rotation-invariant; `max()` on the
diagonal of a PSD matrix adds a non-negative diagonal, so the result stays PSD
and is never smaller than what RIO claimed). `SIGMA_DPSI_RAD_PER_SQRT_S` left at
0.5 deg/sqrt(s) **deliberately**: the median drone matches it and the worst needs
~4 deg/sqrt(s), so any middle value would be a guess rather than a measurement,
and position NEES does not depend on it. Nothing in the estimator changed; no
gate threshold, no `aoa_fov_deg`, no CI/fusion change, and no ToF/baro/flow
aiding (still out of scope per plan §1 and gap 9 — an honest sigma is the
correct placeholder for the missing vertical channel, not a reason to add a
sensor).

**Selftest.** `python perception/radar_processing/rio_bridge.py --selftest` ->
**38 checks, ALL PASS** (was 26; all 26 retained, two of them re-parameterised —
`3c` and `4` now use P_vel values above the floor so they keep testing the frame
rotation and the KF pass-through rather than the floor). New `8`-`8g` pin the
floor: RIO's real steady-state P is raised to it, a larger P is never shrunk, the
floor is per-axis, the floored covariance stays symmetric/PSD under a correlated
P and a yawed frame, and the z and yaw channels are untouched. New `9`-`9c` pin
the measured constants so a silent revert to 0.15 fails, and pin that the
advertised 20 s dead-reckoning drift is now the same order (0.4-2.0 m) as the
0.8-1.4 m actually measured instead of 0.02 m.

**Expected on the next live run** (`tunnel/triangle_forward`, 3 drones): hop-1
ATE **0.2-0.5 m** (down from the reported 0.99 m, which was overwhelmingly B4),
mean NEES **2-6 per drone** with `frac_nees_in_95` **0.85-0.95** (down from
344 / 492 / 135), `nis_by_type` means unchanged and still O(1),
`nis_reject_rate` unchanged or slightly lower, `n_diverged` 0. If ATE comes back
near 1 m with healthy NIS again, the truth array is still reaching the evaluator
unsanitized — check `truth.npz` for stamp 0.0 rows first. Scope:
`eval_scripts/eval_6_1.py` and `perception/radar_processing/rio_bridge.py` only;
`eval_scripts/swarm_loc_gate.py` untouched; nothing under `out/` written; not
committed.
