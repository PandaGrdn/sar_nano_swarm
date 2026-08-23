# P2 deviations from `swarm_localization_plan.md`

## `estimator.max_cov_p_m`: 50 m → 2 m

The plan’s 50 m threshold is when a **position standard deviation** (sqrt of `P` diagonal) is treated as `DIVERGED`. It is not a 50 m position-error allowance.

A 50 m uncertainty is not useful in a SubT corridor: by then the drone would already have hit a wall. Set to **2 m** so a filter that has lost the mesh stops publishing into neighbors on a tunnel-relevant scale.

Restart `rio_stub` / `swarm_loc_node` (or relaunch `phase0_gate.sh`) to pick up the YAML change.

## P2-6 / D8 — corridor MC (offline gate)

`python3 perception/swarm_loc/d8_d11.py --selftest`: drone 0 has bearing, drone 1 does not. Drone-1 position p50 is lower with the rebroadcast bearing than with range-only or `d·d'`. Relative error can stay similar (common-mode + CI); the gate is drone 1's absolute error.

## P2-6 / D11 — live mutual-yaw pair rate is not a gate

The ±45° cone (`aoa_fov_deg: 90`) means a corridor formation almost never produces facing pairs. `swarm_loc_node` logs `n_mutual_yaw_pairs_per_s`; a near-zero live rate **passes**. D11 correctness is the offline facing-pair Monte Carlo in `perception/swarm_loc/d8_d11.py --selftest`. Do not widen the cone or choreograph glances to inflate the live rate.
