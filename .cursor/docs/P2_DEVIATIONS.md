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

Gauge-age floor: `estimator.gauge_age_q_m2_per_s: 0.05` is common-mode process noise. Relative updates cannot reduce a position variance below `min(P_i, P_j)` on that axis. Gate: `disable_entrance` → absolute σ grows (`stress.py` 3d, `ekf.py` 17e). `max_cov_p_m: 2` still trips diverge; that is the abort, not a plateau at centimetre σ.

## P2-8 — stress + ablations are offline; live knobs are files

Blocking gate: `python3 perception/swarm_loc/stress.py --selftest` and `python3 eval_scripts/run_ablations.py --selftest`.

Same knobs as the plan (empty `static_peers`, `occluder_boxes`, RIO `dropout_rate`, drop-middle inactive). Live copies: `python3 eval_scripts/run_ablations.py --write-live-configs DIR`.

Error vs hops (4-drone line, hop 1 = nearest the entrance): `{1: 0.076, 2: 0.142, 3: 0.113, 4: 0.151}` m. Not strictly monotonic; hop 1 is the best. Condition 2 (`uwb_range_no_rio`) is *supposed* to be overconfident — that is the degeneracy evidence, not a filter bug. Condition 6 (minus mutual yaw) matches full on this corridor (`n_mutual_yaw_pairs_per_s=0`). 2× UWB noise did not flip which ablation wins.
