#!/usr/bin/env python3
"""tune_pid.py — Optuna-based cascade PID retune for the loaded-mass
Crazyflie (Phase 1 M2). Classical/optimizer-based tuning, per AGENTS.md
Tier A ("Hyperparameter tuning = Optuna/CMA-ES, not RL") — the same rule
stated there for auction weights is applied here to PID gains too, rather
than hand-guessing.

Two cascaded stages (inner loop before outer loop, standard cascade PID
practice — see configs/airframe/pid_tune.yaml header):
  Stage 1: pid_rate + pid_attitude, scored on a lateral velocity "kick" and
           how fast/cleanly stabilizer.roll/pitch return to level.
  Stage 2: velCtlPid + posCtlPid (with Stage-1 gains applied), scored on a
           forward position-step test using stateEstimate.x tracking.

PREREQUISITE: run this against an ALREADY-RUNNING SITL + Gazebo instance —
this script does not manage the sim lifecycle (too slow to restart Gazebo
per Optuna trial; only pose + estimator are reset between trials). Launch
with the empty Phase-1 world and the current loaded-mass model, e.g.:

    ./eval_scripts/phase0_gate.sh -w phase1_pid_tune --no-radar --headless

Then, in another terminal (with setup_env.sh sourced):

    python3 eval_scripts/tune_pid.py

⚠ This has NOT been exercised against a live Gazebo/SITL instance from this
environment (no ROS/Gazebo available here) — the cflib Commander/
MotionCommander call signatures (`start_linear_motion`, `send_hover_setpoint`)
and the gz `set_pose` service are standard, stable, long-documented APIs but
are unverified against YOUR installed cflib/gz-sim versions per AGENTS.md
§6.4. Run a short --trials 2 --stage 1 smoke test first; if a call fails,
paste the traceback/`help()` output and it'll get corrected quickly.
"""
import argparse
import copy
import json
import signal
import time

import optuna
import yaml


class TrialTimeout(Exception):
    """Raised by the SIGALRM watchdog when a single trial exceeds its wall-clock
    budget. In practice this almost always means the SITL firmware (cf2) died
    mid-run (CrazySim SITL is known to segfault on long/aggressive sweeps), so
    cflib's SyncLogger blocks forever waiting for log packets that never come.
    Without this watchdog, tune_pid.py hangs indefinitely instead of failing.
    """


def _alarm_handler(signum, frame):
    raise TrialTimeout()

from pid_gains import (
    apply_gains,
    load_gains,
    reset_estimator,
    reset_pose,
    save_gains,
    unflatten,
)

# ── pure cost math (unit-testable offline, no sim dependency) ───────────────


def itae(samples, target, key):
    """Integral of time-weighted |error| dt, trapezoidal.
    samples: list of (t, {key: value}) tuples, t in seconds, t=0 at the start
    of the scored window (NOT the start of the whole trial).
    """
    if len(samples) < 2:
        return 0.0
    total = 0.0
    for (t0, s0), (t1, s1) in zip(samples, samples[1:]):
        dt = t1 - t0
        e0 = abs(s0[key] - target)
        e1 = abs(s1[key] - target)
        total += dt * ((t0 * e0 + t1 * e1) / 2.0)
    return total


def max_overshoot(samples, target, key):
    if not samples:
        return 0.0
    return max(abs(s[key] - target) for _, s in samples)


def steady_state_error(samples, target, key, tail_frac=0.2):
    if not samples:
        return 0.0
    n_tail = max(1, int(len(samples) * tail_frac))
    tail = samples[-n_tail:]
    return abs(sum(s[key] for _, s in tail) / len(tail) - target)


def combined_cost(samples, target, key, cost_cfg, diverged=False):
    if diverged:
        return cost_cfg["divergence_penalty"]
    return (
        cost_cfg["w_itae"] * itae(samples, target, key)
        + cost_cfg["w_overshoot"] * max_overshoot(samples, target, key)
        + cost_cfg["w_steady_state_error"] * steady_state_error(samples, target, key)
    )


# ── config / gain-vector helpers ─────────────────────────────────────────────


def build_gains(stock, base_overrides, trial, bounds, additive_params):
    """Start from `stock` (deep-copied), apply `base_overrides` (e.g. Stage-1's
    winning gains, held fixed while Stage 2 searches), then let `trial`
    suggest values for every key in `bounds`.
    """
    gains = copy.deepcopy(stock)
    flat = {}
    for group, fields in gains.items():
        for field, value in fields.items():
            flat[f"{group}.{field}"] = value
    flat.update(base_overrides)

    for key, (lo, hi) in bounds.items():
        stock_val = flat[key]
        if key in additive_params:
            flat[key] = trial.suggest_float(key, lo, hi)
        else:
            mult = trial.suggest_float(key, lo, hi)
            flat[key] = stock_val * mult

    return unflatten(flat)


# ── live sim interaction ─────────────────────────────────────────────────────


def run_trial(scf, mc_cls, sync_logger_cls, log_config_cls, cfg, gains, stage):
    """Apply `gains`, run the Stage-1 kick or Stage-2 step maneuver, and
    return (samples, target, key, diverged) for cost scoring.

    `mc_cls`/`sync_logger_cls`/`log_config_cls` are injected (rather than
    imported at module scope) so this function can be unit-exercised with
    fakes if needed; in normal use they are cflib's MotionCommander,
    SyncLogger, LogConfig.
    """
    apply_gains(scf.cf, gains)
    time.sleep(0.05)

    man = cfg["maneuver"]
    bound = cfg["cost"]["divergence_bound_m"]
    diverged = False
    samples = []

    if stage == 1:
        log_config = log_config_cls(name="pid_tune_s1", period_in_ms=man["log_period_ms"])
        log_config.add_variable("stabilizer.roll", "float")
        log_config.add_variable("stabilizer.pitch", "float")
        log_config.add_variable("stateEstimate.z", "float")
        target, key = 0.0, "stabilizer.roll"  # roll is scored; pitch logged for inspection
    else:
        log_config = log_config_cls(name="pid_tune_s2", period_in_ms=man["log_period_ms"])
        log_config.add_variable("stateEstimate.x", "float")
        log_config.add_variable("stateEstimate.y", "float")
        log_config.add_variable("stateEstimate.z", "float")
        target, key = man["stage2_step_dx_m"], "stateEstimate.x"

    with mc_cls(scf, default_height=man["hover_height_m"]) as mc:
        time.sleep(man["settle_time_s"])

        with sync_logger_cls(scf, log_config) as logger:
            t0 = time.time()

            if stage == 1:
                mc.start_linear_motion(man["stage1_kick_vx_mps"], 0, 0)
                kick_end = man["stage1_kick_duration_s"]
                window_end = kick_end + man["stage1_measure_window_s"]
            else:
                step_velocity = 0.3
                step_duration = man["stage2_step_dx_m"] / step_velocity
                mc.start_linear_motion(step_velocity, 0, 0)
                kick_end = step_duration
                window_end = step_duration + man["stage2_measure_window_s"]

            returned = False
            for entry in logger:
                t = time.time() - t0
                data = entry[1]
                samples.append((t, data))

                if abs(data.get("stateEstimate.z", man["hover_height_m"]) - man["hover_height_m"]) > bound:
                    diverged = True
                    break

                if not returned and t >= kick_end:
                    mc.start_linear_motion(0, 0, 0)
                    returned = True

                if t >= window_end:
                    break

        mc.start_linear_motion(0, 0, 0)
    # mc.__exit__ lands the drone here.

    scored = [(t, s) for t, s in samples if t >= kick_end] if not diverged else samples
    return scored, target, key, diverged


def run_stage(scf, cfg, stock, base_overrides, stage, mc_cls, sync_logger_cls, log_config_cls,
              world_name, model_name, estimator_group):
    """Run one Optuna study (a full stage) against the live SITL connection.
    Must be called from inside an active `mlflow.start_run(...)` context
    (main() opens one); each trial logs as a nested child run.
    """
    import mlflow

    bounds = cfg["search_bounds"][f"stage{stage}"]
    additive = cfg["additive_params"]
    n_trials = cfg["optuna"][f"n_trials_stage{stage}"]

    sampler = optuna.samplers.TPESampler(seed=cfg["optuna"]["seed"])
    study = optuna.create_study(direction="minimize", sampler=sampler)

    # Per-trial wall-clock watchdog. Normal trials run ~15-20 s; a trial that
    # blows well past that has almost certainly lost the firmware. Default to a
    # generous multiple so a merely-slow (real-time-factor < 1) sim isn't
    # falsely flagged.
    trial_timeout_s = float(cfg["maneuver"].get("trial_timeout_s", 45.0))
    max_consec_timeouts = int(cfg["optuna"].get("max_consecutive_timeouts", 3))
    signal.signal(signal.SIGALRM, _alarm_handler)
    consec = {"timeouts": 0}

    def objective(trial):
        gains = build_gains(stock, base_overrides, trial, bounds, additive)

        reset_pose(world_name, model_name, xyz=(0.0, 0.0, 0.5))
        reset_estimator(scf.cf, estimator_group)
        time.sleep(cfg["maneuver"]["inter_trial_idle_s"])

        timed_out = False
        signal.setitimer(signal.ITIMER_REAL, trial_timeout_s)
        try:
            samples, target, key, diverged = run_trial(
                scf, mc_cls, sync_logger_cls, log_config_cls, cfg, gains, stage
            )
        except TrialTimeout:
            timed_out = True
            diverged = True
            samples, target, key = [], 0.0, "stabilizer.roll"
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0)

        if timed_out:
            consec["timeouts"] += 1
            print(
                f"[tune_pid] Trial {trial.number} TIMED OUT after {trial_timeout_s:.0f}s "
                f"({consec['timeouts']}/{max_consec_timeouts} consecutive) — "
                f"firmware likely crashed."
            )
            if consec["timeouts"] >= max_consec_timeouts:
                raise RuntimeError(
                    f"{max_consec_timeouts} consecutive trial timeouts — the SITL "
                    f"firmware (cf2) has almost certainly segfaulted. Aborting this "
                    f"study. Restart the sim (phase0_gate.sh) and re-run; results so "
                    f"far are logged to MLflow."
                )
        else:
            consec["timeouts"] = 0

        cost = combined_cost(samples, target, key, cfg["cost"], diverged)

        with mlflow.start_run(nested=True, run_name=f"stage{stage}_trial{trial.number}"):
            for k, v in trial.params.items():
                mlflow.log_param(k, v)
            mlflow.log_metric("cost", cost)
            mlflow.log_metric("diverged", 1.0 if diverged else 0.0)
            mlflow.log_metric("timed_out", 1.0 if timed_out else 0.0)

        return cost

    # RuntimeError is raised by the objective when the firmware dies (repeated
    # timeouts). Catch it so the best gains found BEFORE the crash are still
    # returned/saved instead of being lost with the exception.
    try:
        study.optimize(objective, n_trials=n_trials)
    except RuntimeError as e:
        completed = [t for t in study.trials
                     if t.state == optuna.trial.TrialState.COMPLETE]
        if not completed:
            raise
        print(f"[tune_pid] Stage {stage} aborted early ({e}); keeping best of "
              f"{len(completed)} completed trials.")

    best_flat = {}
    gains_at_best = build_gains(stock, base_overrides, study.best_trial, bounds, additive)
    for group, fields in gains_at_best.items():
        for field, value in fields.items():
            best_flat[f"{group}.{field}"] = value
    return study.best_value, best_flat


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/airframe/pid_tune.yaml")
    parser.add_argument("--stock", default="configs/airframe/pid_gains_stock.yaml")
    parser.add_argument("--stage", choices=["1", "2", "both"], default="both")
    parser.add_argument(
        "--trials", type=int, default=None,
        help="Override n_trials for whichever stage(s) run (quick smoke test, e.g. --trials 3)",
    )
    args = parser.parse_args()

    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)
    stock = load_gains(args.stock)

    if args.trials is not None:
        cfg["optuna"]["n_trials_stage1"] = args.trials
        cfg["optuna"]["n_trials_stage2"] = args.trials

    import mlflow
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.crazyflie.syncLogger import SyncLogger
    from cflib.crazyflie.log import LogConfig
    from cflib.positioning.motion_commander import MotionCommander

    mlflow.set_tracking_uri("sqlite:///mlflow.db")
    mlflow.set_experiment(cfg["optuna"]["mlflow_experiment"])

    cflib.crtp.init_drivers()
    uri = cfg["connection"]["uri"]
    world_name = cfg["connection"]["world_name"]
    model_name = cfg["connection"]["model_name"]
    estimator_group = cfg["connection"]["estimator_reset_group"]

    print(f"[tune_pid] Connecting to {uri} …")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        with mlflow.start_run(run_name="pid_cascade_tune"):
            best_overrides = {}

            if args.stage in ("1", "both"):
                print("[tune_pid] Stage 1: pid_rate + pid_attitude …")
                cost1, best_overrides = run_stage(
                    scf, cfg, stock, {}, 1,
                    MotionCommander, SyncLogger, LogConfig,
                    world_name, model_name, estimator_group,
                )
                print(f"[tune_pid] Stage 1 best cost: {cost1:.4f}")
                mlflow.log_metric("stage1_best_cost", cost1)

            if args.stage in ("2", "both"):
                print("[tune_pid] Stage 2: velCtlPid + posCtlPid …")
                cost2, stage2_overrides = run_stage(
                    scf, cfg, stock, best_overrides, 2,
                    MotionCommander, SyncLogger, LogConfig,
                    world_name, model_name, estimator_group,
                )
                print(f"[tune_pid] Stage 2 best cost: {cost2:.4f}")
                mlflow.log_metric("stage2_best_cost", cost2)
                best_overrides.update(stage2_overrides)

            final_gains = unflatten(best_overrides) if best_overrides else stock
            # Fill in any groups/fields untouched by the search from stock.
            merged = copy.deepcopy(stock)
            for group, fields in final_gains.items():
                merged.setdefault(group, {}).update(fields)

            out_path = cfg["output"]["gains_path"]
            save_gains(merged, out_path)
            print(f"[tune_pid] Wrote tuned gains: {out_path}")

            with open(cfg["output"]["report_path"], "w") as f:
                json.dump({"best_overrides": best_overrides}, f, indent=2)


if __name__ == "__main__":
    main()
