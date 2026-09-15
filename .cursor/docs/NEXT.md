# What is next (after lockstep PASS)

Lockstep is on: firmware 1 kHz ticks follow Gazebo **sim** time, not the wall
clock. `tunnel/triangle_forward` on 2026-09-14 is a real airborne in-tube
flight (hover ~0.5 m, `airborne_cf_*` / `rio_alive_cf_*` PASS). That dump is
valid as a **first airborne set**, not a frozen estimator spec.

## Time units (the “one timestep” confusion)

Nothing here is a one-step flight.

| Thing | What it is |
| --- | --- |
| Lockstep tick | **1 ms of sim time** = one FreeRTOS tick = one physics step. That is the controller dt. |
| Scenario `duration` | **45 s of sim time** of scripted path (`triangle_forward`). |
| Wall time | Gazebo still runs at RTF ~0.3, so 45 s sim is ~2–3 min of wall flying. Pad + RIO + takeoff + plots made the gate process ~8 min. |
| Dashboard `t` (s) | Sim seconds (scored window ~46–138 s on that run). |

A 1 ms lockstep tick is *how* the PID stays in sync with physics. The flight
you looked at lasted tens of sim seconds.

## The XY plot is not a 2 m formation translate

`triangle_forward` is supposed to be **4 × 0.5 m** simultaneous +x legs
(2 m net), then hover/land.

Truth in the scored window (`out/swarm_loc_eval/tunnel/triangle_forward/truth.npz`):

| drone | net Δx | x span | y span |
| --- | --- | --- | --- |
| cf_0 | +0.36 m | 0.43 m | 0.45 m |
| cf_1 | +0.38 m | 0.45 m | 0.33 m |
| cf_2 | +0.63 m | 0.72 m | 0.53 m |

That is hover jitter plus a small +x drift, which is why `06_xy_topdown.png`
looks like three blobs. **Cause:** `_all_forward()` in
`eval_scripts/swarm_loc_scenarios.py` does `time.sleep(distance/velocity)`
on the **wall** clock. At RTF ~0.3 a “0.5 m @ 0.2 m/s” leg is only ~0.7 s of
sim (~15 cm). Four legs ≈ 0.6 m, which matches the truth. Pauses after each
leg already use `sim_sleep`; the move itself does not.

`staggered_advance` (`mc.forward(leg)`) has the same wall-clock trap.

## Do next, in order

1. **Drive motion on sim time.** `_all_forward` (and `mc.forward` / shuttle
   sleeps that are still wall-based) must hold the velocity setpoint for
   `distance/velocity` **sim** seconds, then stop. Selftest that a mocked
   RTF&lt;1 still yields the commanded displacement.
2. **Re-fly `tunnel/triangle_forward`** with lockstep still on. Truth XY
   should show ~2 m of +x as a formation, not 0.4 m blobs. Kill via
   `out/_kill_sim_stack.sh` before and after. Keep using
   `out/_start_phase0.py` (pad spawn + regenerated derived yaml).
3. **Only then** treat ATE/RPE/NEES as an airborne localization number, and
   refit RIO process/measurement noise from that in-tube hover+translate
   (previous RIO floors were grounded). Do not freeze 0.13 m ATE from the
   blob run.
4. **UWB mix.** This run was ~86% range-only, 0 mutual-yaw; cf_1 yaw walked
   ~6°. Revisit bearing / mutual-yaw after the drones actually translate.
5. **Score window.** `score_window.json` has `t1: null` (score from path
   start through end of log). Mark a real `t1` at `mark_flight_end`.
6. **Other scenarios** (`collinear_shuttle`, `staggered_advance`) only after
   (1)–(2). Old collinear plots are grounded/crash data.

## Where the current (blob) PASS lives

Use this dump only as “lockstep hover works, RIO inits on the pad”:

- `out/swarm_loc_eval/tunnel/triangle_forward/plots/00_dashboard.png`
- `out/swarm_loc_eval/tunnel/triangle_forward/plots/06_xy_topdown.png`
- `out/swarm_loc_eval/tunnel/triangle_forward/metrics_6_1.json`
- `out/swarm_loc_eval/tunnel/triangle_forward/truth.npz`
- `out/swarm_loc_logs/tunnel/triangle_forward/cf_{0,1,2}.npz`

Do not cite `out/swarm_loc_eval/tunnel/collinear_shuttle/`.
