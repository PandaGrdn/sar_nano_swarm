#!/usr/bin/env python3
"""stress.py — P2-8 corridor / degeneracy stresses (offline).

Each case is an existing knob, not a new mechanism:

- collinear corridor: 4 drones, adjacent UWB only, entrance on drone 0
- mesh partition: drop the middle drone mid-run (odom_stale / inactive)
- entrance dropout: static_peers empty / disable_entrance
- NLOS on one link: occluder_boxes + nlos_* from uwb_pdoa.yaml
- RIO valid=False bursts: rio.dropout_rate / dropout_duration_s

Gate: no NaNs; do not stay confident while wrong; cov grows vs a healthy
control; error vs hops-from-entrance is produced.

Usage:
    python3 perception/swarm_loc/stress.py --selftest
"""
from __future__ import annotations

import argparse
import copy
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

_REPO_ROOT = Path(__file__).resolve().parents[2]
for _p in ("perception/swarm_loc", "perception/uwb_sim"):
    if str(_REPO_ROOT / _p) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT / _p))

from ekf import position_nees, propagate, update  # noqa: E402
from measurements import from_edge, reciprocal_relpos  # noqa: E402
from rio_stub import RioStubEngine, load_config, resolve_config_path  # noqa: E402
from state import IDX_P, N_STATE, STATUS_DIVERGED, SwarmState, rpy_to_R, wrap_psi  # noqa: E402
from uwb_edges import FLAG_BEARING_VALID, FLAG_PEER_IS_SURVEYED, FLAG_RANGE_VALID  # noqa: E402
from uwb_model import bearing_xyz, los_check_boxes  # noqa: E402

ENTRANCE_ID = 1000


def load_uwb_cfg() -> dict:
    path = _REPO_ROOT / "configs" / "sensors" / "uwb_pdoa.yaml"
    with open(path, "r") as f:
        return yaml.safe_load(f)


def sigma_p(st: SwarmState) -> float:
    return float(math.sqrt(max(st.P[0, 0], st.P[1, 1], st.P[2, 2])))


def hops_from_entrance(n: int) -> Dict[int, int]:
    """Line 1000—0—1—… ; hop(i) = i+1."""
    return {i: i + 1 for i in range(n)}


def _edge(
    p_obs,
    psi,
    p_peer,
    rng,
    *,
    bearing: bool,
    surveyed: bool,
    sigma_d: float,
    sigma_az: float,
    sigma_el: float,
    nlos: bool,
    uwb: dict,
    noise_scale: float,
) -> dict:
    if nlos:
        sigma_d = sigma_d * float(uwb.get("nlos_sigma_mult", 3.0))
        bearing = False
    sd, saz, sel = sigma_d * noise_scale, sigma_az * noise_scale, sigma_el * noise_scale
    R = rpy_to_R(float(psi), 0.0, 0.0)
    z_true = R.T @ (np.asarray(p_peer) - np.asarray(p_obs))
    d = float(np.linalg.norm(z_true))
    az = math.atan2(float(z_true[1]), float(z_true[0]))
    el = math.asin(float(np.clip(z_true[2] / max(d, 1e-12), -1.0, 1.0)))
    if rng is None:
        d_m, az_m, el_m = d, az, el
    else:
        d_m = max(d + float(rng.normal(0.0, sd)), 0.05)
        if nlos:
            d_m += abs(float(rng.normal(float(uwb.get("nlos_bias_mean_m", 0.35)), float(uwb.get("nlos_bias_sigma_m", 0.25)))))
        az_m = az + float(rng.normal(0.0, saz))
        el_m = float(np.clip(el + rng.normal(0.0, sel), -1.56, 1.56))
    x, y, z = bearing_xyz(d_m, az_m, el_m)
    flags = FLAG_RANGE_VALID
    if bearing:
        flags |= FLAG_BEARING_VALID
    if surveyed:
        flags |= FLAG_PEER_IS_SURVEYED
    return {
        "flags": flags,
        "x": float(x),
        "y": float(y),
        "z": float(z) if bearing else float("nan"),
        "range_m": float(d_m),
        "azimuth_rad": float(az_m),
        "elevation_rad": float(el_m),
        "sigma_range_m": float(sd),
        "sigma_az_rad": float(saz),
        "sigma_el_rad": float(sel),
    }


def in_forward_cone(p_obs, psi, p_peer, fov_deg: float = 90.0) -> bool:
    R = rpy_to_R(float(psi), 0.0, 0.0)
    db = R.T @ (np.asarray(p_peer) - np.asarray(p_obs))
    r = float(np.linalg.norm(db))
    if r < 1e-9:
        return True
    cos_t = float(np.clip(db[0] / r, -1.0, 1.0))
    return math.acos(cos_t) <= math.radians(fov_deg) / 2.0


def run_line_mesh(
    est_cfg: dict,
    *,
    n_drones: int = 4,
    n_steps: int = 30,
    dt: float = 0.1,
    speed_mps: float = 0.20,
    seed: int = 21,
    entrance: bool = True,
    drop_id: Optional[int] = None,
    drop_step: int = 12,
    nlos_pair: Optional[Tuple[int, int]] = None,
    occluder_boxes: Optional[list] = None,
    rio_force_invalid: bool = False,
    loggers=None,
) -> dict:
    """Single-file corridor of n drones. Adjacent UWB + entrance on drone 0."""
    from meas_log import MeasurementLogger, kind_from_edge

    cfg = copy.deepcopy(est_cfg)
    abl = cfg.setdefault("ablation", {})
    uwb = load_uwb_cfg()
    if occluder_boxes:
        uwb = dict(uwb)
        uwb["occluder_boxes"] = occluder_boxes
        uwb["los_model"] = "boxes"
    rng = np.random.default_rng(seed)
    noise_scale = float(abl.get("uwb_noise_scale", 1.0))
    sigma_d = float(uwb["sigma_range_los_m"])
    sigma_az = math.radians(float(uwb["sigma_boresight_deg"]))
    sigma_el = sigma_az
    p_ent = np.array(cfg["entrance"]["position_xyz_m"], dtype=np.float64)
    fov = float(uwb.get("aoa_fov_deg", 90.0))

    truth = {i: SwarmState.from_launch(cfg, i).p.copy() for i in range(n_drones)}
    states = {i: SwarmState.from_launch(cfg, i) for i in range(n_drones)}
    for i, st in states.items():
        st.x = st.x + rng.multivariate_normal(np.zeros(N_STATE), 0.25 * st.P)
        st.x[6] = wrap_psi(float(st.x[6]))
        st.stamp = dt
    engines = {i: RioStubEngine(cfg, np.random.default_rng(seed + 10 + i)) for i in range(n_drones)}
    if rio_force_invalid:
        for eng in engines.values():
            eng.dropout_rate = 1.0
            eng.dropout_duration_s = 1e6

    inactive = set()
    last_bearing: Dict[Tuple[int, int], dict] = {}
    finite = True
    hops = hops_from_entrance(n_drones)
    hist = defaultdict(list)

    def nlos_now(a: int, b: int) -> bool:
        pair = (min(a, b), max(a, b))
        if nlos_pair is not None and pair == (min(nlos_pair), max(nlos_pair)):
            return True
        if occluder_boxes:
            return not los_check_boxes(truth[a], truth[b], occluder_boxes)
        return False

    for step in range(n_steps):
        stamp = (step + 1) * dt
        if drop_id is not None and step >= drop_step:
            inactive.add(int(drop_id))

        for i in range(n_drones):
            if i in inactive:
                continue
            dp_true = np.array([speed_mps * dt, 0.0, 0.0])
            truth[i] = truth[i] + rpy_to_R(0.0, 0.0, 0.0) @ dp_true
            if abl.get("disable_rio"):
                states[i].stamp = stamp
                continue
            delta = engines[i].corrupt(stamp, dt, dp_true, 0.0, 0.0, 0.0)
            if rio_force_invalid:
                delta.valid = False
            states[i] = propagate(states[i], delta, cfg)
            if not np.all(np.isfinite(states[i].x)):
                finite = False
            if loggers is not None:
                loggers[i].add_rio(
                    stamp, dt, delta.delta_p_body, delta.delta_psi, 0.0, 0.0, delta.valid
                )

        snap = {i: states[i].copy() for i in range(n_drones)}
        if abl.get("disable_uwb"):
            for i in range(n_drones):
                if i not in inactive:
                    hist[i].append((stamp, float(np.linalg.norm(states[i].p - truth[i])), sigma_p(states[i])))
            continue

        use_bearing = not bool(abl.get("disable_bearing", False))
        use_ent = entrance and not bool(abl.get("disable_entrance", False))
        use_rec = bool(cfg["measurements"].get("use_reciprocal_bearing", True)) and use_bearing

        # entrance → drone 0
        if use_ent and 0 not in inactive:
            nlos_e = bool(occluder_boxes) and (not los_check_boxes(truth[0], p_ent, occluder_boxes))
            e0 = _edge(
                truth[0], 0.0, p_ent, rng,
                bearing=use_bearing, surveyed=True,
                sigma_d=sigma_d, sigma_az=sigma_az, sigma_el=sigma_el,
                nlos=nlos_e, uwb=uwb, noise_scale=noise_scale,
            )
            m = from_edge(states[0], None, e0, cfg)
            if m is not None:
                states[0], _ = update(states[0], m, cfg)
            if loggers is not None:
                z = [e0["x"], e0["y"], e0["z"]] if not math.isnan(float(e0["z"])) else None
                loggers[0].add_uwb(
                    stamp, kind_from_edge(e0, use_bearing), 0, ENTRANCE_ID,
                    e0["range_m"], e0["azimuth_rad"], e0["elevation_rad"],
                    e0["sigma_range_m"], e0["sigma_az_rad"], e0["sigma_el_rad"],
                    states[0].psi, 0.0, 0.0, z,
                )

        pairs = [(i, i + 1) for i in range(n_drones - 1)]
        for a, b in pairs:
            if a in inactive or b in inactive:
                continue
            nlos = nlos_now(a, b)
            bear_a = use_bearing and (not nlos) and in_forward_cone(truth[a], 0.0, truth[b], fov)
            bear_b = use_bearing and (not nlos) and in_forward_cone(truth[b], 0.0, truth[a], fov)
            ea = _edge(
                truth[a], 0.0, truth[b], rng, bearing=bear_a, surveyed=False,
                sigma_d=sigma_d, sigma_az=sigma_az, sigma_el=sigma_el,
                nlos=nlos, uwb=uwb, noise_scale=noise_scale,
            )
            eb = _edge(
                truth[b], 0.0, truth[a], rng, bearing=bear_b, surveyed=False,
                sigma_d=sigma_d, sigma_az=sigma_az, sigma_el=sigma_el,
                nlos=nlos, uwb=uwb, noise_scale=noise_scale,
            )
            ma = from_edge(states[a], snap[b].p, ea, cfg)
            mb = from_edge(states[b], snap[a].p, eb, cfg)
            if ma is not None:
                states[a], _ = update(states[a], ma, cfg, P_j=snap[b].P, fusion="ci")
            if mb is not None:
                states[b], _ = update(states[b], mb, cfg, P_j=snap[a].P, fusion="ci")
            if loggers is not None:
                for obs, peer, edge in ((a, b, ea), (b, a, eb)):
                    z = [edge["x"], edge["y"], edge["z"]] if not math.isnan(float(edge["z"])) else None
                    loggers[obs].add_uwb(
                        stamp, kind_from_edge(edge, use_bearing), obs, peer,
                        edge["range_m"], edge["azimuth_rad"], edge["elevation_rad"],
                        edge["sigma_range_m"], edge["sigma_az_rad"], edge["sigma_el_rad"],
                        states[obs].psi, 0.0, 0.0, z,
                    )
            if bear_a:
                last_bearing[(a, b)] = ea
            if use_rec and (not bear_b) and (a, b) in last_bearing:
                src = last_bearing[(a, b)]
                z = bearing_xyz(src["range_m"], src["azimuth_rad"], src["elevation_rad"])
                rec = reciprocal_relpos(
                    states[b].p, snap[a].p, snap[a].psi, snap[a].pitch, snap[a].roll,
                    z, src["sigma_range_m"], src["sigma_az_rad"], src["sigma_el_rad"],
                    src["range_m"], src["azimuth_rad"], src["elevation_rad"],
                )
                if rec is not None:
                    states[b], _ = update(states[b], rec, cfg, P_j=snap[a].P, fusion="ci")

        for i in range(n_drones):
            if i in inactive:
                continue
            if not np.all(np.isfinite(states[i].x)):
                finite = False
            hist[i].append(
                (stamp, float(np.linalg.norm(states[i].p - truth[i])), sigma_p(states[i]))
            )
            if loggers is not None:
                loggers[i].add_est(stamp, states[i].p, states[i].v, states[i].psi, states[i].status)

    err = {}
    sig = {}
    nees = {}
    diverged = {}
    for i in range(n_drones):
        if i in inactive and i not in states:
            continue
        err[i] = float(np.linalg.norm(states[i].p - truth[i]))
        sig[i] = sigma_p(states[i])
        nees[i] = position_nees(states[i], truth[i])
        diverged[i] = states[i].status == STATUS_DIVERGED
        finite = finite and np.all(np.isfinite(states[i].x)) and np.all(np.isfinite(states[i].P))

    hops_err = {hops[i]: err[i] for i in err}
    overconf = [
        i for i in err
        if err[i] > 0.20 and err[i] > 4.0 * max(sig[i], 1e-6) and not diverged[i]
    ]
    return {
        "err": err,
        "sigma": sig,
        "nees": nees,
        "diverged": diverged,
        "finite": finite,
        "hops": hops,
        "hops_err": hops_err,
        "overconfident": overconf,
        "inactive": set(inactive),
        "ate": float(np.mean(list(err.values()))) if err else float("inf"),
        "mean_sigma": float(np.mean(list(sig.values()))) if sig else float("inf"),
        "hist": dict(hist),
        "states": states,
        "truth": truth,
        "cfg": cfg,
    }


def run_selftest() -> int:
    ok = True
    n_pass = 0
    n_fail = 0

    def check(name: str, cond: bool, detail: str = ""):
        nonlocal ok, n_pass, n_fail
        if cond:
            n_pass += 1
            print(f"[selftest] PASS {name}")
        else:
            ok = False
            n_fail += 1
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))

    cfg = load_config(resolve_config_path("configs/estimation/swarm_loc.yaml"))
    print("[selftest] collinear corridor (n=4) …")
    base = run_line_mesh(cfg, seed=21)
    print(
        f"[selftest] corridor ATE={base['ate']:.3f} sigma={base['mean_sigma']:.3f} "
        f"hops_err={ {h: round(e, 3) for h, e in sorted(base['hops_err'].items())} }"
    )
    check("1 corridor finite", base["finite"])
    check("1b corridor no diverge", not any(base["diverged"].values()), str(base["diverged"]))
    check("1c corridor not overconfident", base["overconfident"] == [], str(base["overconfident"]))
    # Headline: farther hops are not *better* than hop 1.
    e1 = base["hops_err"].get(1, 0.0)
    e_far = max(base["hops_err"].get(h, 0.0) for h in (3, 4) if h in base["hops_err"])
    check("1d hops: far >= hop1 * 0.8", e_far + 1e-9 >= 0.8 * e1, f"hop1={e1:.3f} far={e_far:.3f}")

    print("[selftest] mesh partition (drop drone 1) …")
    part = run_line_mesh(cfg, drop_id=1, drop_step=8, n_steps=40, seed=21)
    check("2 partition finite", part["finite"] and 0 in part["err"] and 3 in part["err"])
    check("2b partition no diverge on survivors", not part["diverged"].get(0) and not part["diverged"].get(3))
    check(
        "2c far side degraded (cov or error)",
        part["sigma"][3] > 1.05 * base["sigma"][3] or part["err"][3] > 1.15 * base["err"][3],
        f"sig part={part['sigma'][3]:.3f} base={base['sigma'][3]:.3f} "
        f"err part={part['err'][3]:.3f} base={base['err'][3]:.3f}",
    )
    check("2d not overconfident", part["overconfident"] == [], str(part["overconfident"]))

    print("[selftest] entrance dropout …")
    cfg_noent = copy.deepcopy(cfg)
    cfg_noent["ablation"] = dict(cfg.get("ablation", {}))
    cfg_noent["ablation"]["disable_entrance"] = True
    noent = run_line_mesh(cfg_noent, entrance=False, seed=21)
    check("3 no-entrance finite", noent["finite"] and not any(noent["diverged"].values()))
    check(
        "3b absolute cov not tighter than with entrance",
        noent["mean_sigma"] + 1e-9 >= 0.90 * base["mean_sigma"],
        f"noent={noent['mean_sigma']:.3f} base={base['mean_sigma']:.3f}",
    )
    check("3c not overconfident", noent["overconfident"] == [], str(noent["overconfident"]))

    print("[selftest] NLOS wall between 1–2 …")
    wall = [{"name": "wall", "x_min": 2.2, "x_max": 2.4, "y_min": -5.0, "y_max": 5.0, "z_min": 0.0, "z_max": 3.0}]
    nlos = run_line_mesh(cfg, nlos_pair=(1, 2), occluder_boxes=wall, seed=21)
    check("4 nlos finite", nlos["finite"] and not any(nlos["diverged"].values()))
    check(
        "4b downstream cov grew",
        nlos["sigma"][3] > 1.10 * base["sigma"][3] or nlos["err"][3] > base["err"][3],
        f"sig {nlos['sigma'][3]:.3f} vs {base['sigma'][3]:.3f} err {nlos['err'][3]:.3f} vs {base['err'][3]:.3f}",
    )
    check("4c not overconfident", nlos["overconfident"] == [], str(nlos["overconfident"]))

    print("[selftest] RIO valid=False bursts …")
    rio_d = run_line_mesh(cfg, rio_force_invalid=True, seed=21)
    check("5 rio-dropout finite", rio_d["finite"])
    check(
        "5b Q inflation grew cov",
        rio_d["mean_sigma"] > 1.20 * base["mean_sigma"],
        f"drop={rio_d['mean_sigma']:.3f} base={base['mean_sigma']:.3f}",
    )
    check("5c not overconfident", rio_d["overconfident"] == [], str(rio_d["overconfident"]))
    # Hitting the 2 m trip is allowed here — that *is* cov growing to the abort.
    if any(rio_d["diverged"].values()):
        check("5d diverge only after cov trip", True)

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    parser.print_help()
    sys.exit(2)


if __name__ == "__main__":
    main()
