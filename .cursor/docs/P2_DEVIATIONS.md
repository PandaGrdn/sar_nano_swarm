# P2 deviations from `swarm_localization_plan.md`

## `estimator.max_cov_p_m`: 50 m → 2 m

The plan’s 50 m threshold is when a **position standard deviation** (sqrt of `P` diagonal) is treated as `DIVERGED`. It is not a 50 m position-error allowance.

A 50 m uncertainty is not useful in a SubT corridor: by then the drone would already have hit a wall. Set to **2 m** so a filter that has lost the mesh stops publishing into neighbors on a tunnel-relevant scale.

Restart `rio_stub` / `swarm_loc_node` (or relaunch `phase0_gate.sh`) to pick up the YAML change.
