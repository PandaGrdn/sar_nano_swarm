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
