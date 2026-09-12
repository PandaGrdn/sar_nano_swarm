#!/usr/bin/env python3
"""swarm_loc_gate.py — P2-5 live ROS gate for the distributed swarm-loc filter.

Prereq: sim from phase0_gate.sh, e.g.
    ./eval_scripts/phase0_gate.sh -w phase0_tunnel_gate -n 3 --spacing 1.5 \\
        --headless --no-rviz

Usage (setup_env.sh sourced):
    python3 -u eval_scripts/swarm_loc_gate.py --num-drones 3 --duration 300
    python3 -u eval_scripts/swarm_loc_gate.py --scenario tunnel/triangle_forward
    python3 eval_scripts/swarm_loc_gate.py --list-scenarios
    python3 eval_scripts/swarm_loc_gate.py --selftest
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SWARM = os.path.join(_REPO_ROOT, "perception", "swarm_loc")
if _SWARM not in sys.path:
    sys.path.insert(0, _SWARM)
if os.path.join(_REPO_ROOT, "eval_scripts") not in sys.path:
    sys.path.insert(0, os.path.join(_REPO_ROOT, "eval_scripts"))

from swarm_loc_node import topics_contain_truth  # noqa: E402
from swarm_msgs import STATE_DTYPE, unpack_rio, unpack_state  # noqa: E402
from ros_gz_qos import subscribe_gz  # noqa: E402
from swarm_loc_scenarios import (  # noqa: E402
    apply_motion,
    eval_dir_for,
    get_scenario,
    log_dir_for,
    spawn_xy,
)

_RESTART_MSG = (
    "[swarm_loc_gate] Restart sim before retrying:\n"
    "  pkill -9 -f 'gz sim'; pkill -9 -x cf2; pkill -9 -f swarm_loc_node\n"
    "  ./eval_scripts/phase0_gate.sh -w phase0_tunnel_gate -n 3 --spacing 1.5 "
    "--headless --no-rviz\n"
    "  python3 -u eval_scripts/swarm_loc_gate.py"
)

RATE_FRAC_MIN = 0.80

# ---------------------------------------------------------------------------
# Hollow-run tripwires (blocking). The 2026-09-11 triangle_forward run PASSED
# the old gate while the EKF consumed nothing: n_update=0, n_uwb=0, empty rio
# logs, EKF publish stamps frozen at 0, hops -1 for every drone, mean NEES
# 170–2360 with frac_nees_in_95=0.0. ATE only looked plausible because the
# estimates were pose-initialization + nothing. Each constant below backs a
# named check in the checks dict; all must PASS for the gate to pass. This
# gate has no config/CLI threshold pattern, so these are module constants —
# edit here, with a comment, if a threshold needs tuning.
# ---------------------------------------------------------------------------
RIO_VALID_FRAC_MIN = 0.5    # >= half of /cf_i/rio/delta rows must have valid==1
                            # (applies to stub RIO too — the stub publishes valid deltas)
RIO_SPAN_FRAC_MIN = 0.8     # RIO stamps must span >= 80% of the SIM-time reference span
RIO_ADV_FRAC_MIN = 0.9      # >= 90% of consecutive RIO stamps strictly advance
RIO_RATE_MIN_HZ = 5.0       # mean RIO delta rate sanity floor, in SIM-Hz (see below).
                            # Real RIO runs ~9.9 sim-Hz; the stub runs ~50 sim-Hz.
EKF_SPAN_FRAC_MIN = 0.8     # EKF publish stamps must advance across the run,
                            # not sit frozen at 0/constant
# --- seq increment rate (fixed 2026-09-11, BUG B2) --------------------------
# /cf_i/swarm_loc/estimate is published from swarm_loc_node._on_tick at
# estimator.rate_hz (50 Hz) using the CURRENT self._seq; self._seq is
# incremented only in _on_broadcast_tick at comms.broadcast_rate_hz (10 Hz):
#     _on_tick:            row = state_row_from_filter(self.st, self._seq, ...)
#     _on_broadcast_tick:  self._seq += 1
# So a healthy run shows a new seq on only ~bc_hz/rate_hz = 10/50 = 0.20 of
# consecutive estimate rows. The old fixed EKF_SEQ_FRAC_MIN = 0.5 was therefore
# unpassable by construction (the live run measured 0.22). The honest check is
# (a) seq is non-decreasing, (b) seq strictly increases across the run, and
# (c) the increment fraction is at least a margin below the ratio the gate's
# own config implies. A frozen seq still gives frac 0.0 and max==min → FAIL.
EKF_SEQ_FRAC_MARGIN = 0.5   # need >= 0.5 × expected(bc_hz / rate_hz)
EKF_SEQ_FRAC_FLOOR = 0.02   # absolute floor; no config may drive the bar to 0
EKF_SEQ_DECREASE_FRAC_MAX = 0.01  # tolerate ~1% reordered arrivals, no more
UPDATE_RATE_FLOOR_HZ = 1.0  # per-drone fused updates >= 1 per scored second.
                            # UWB edges arrive at ~10+/s per pair, so 1 Hz is
                            # an order of magnitude of slack — but 0 never passes.
NEES_MEAN_MAX = 50.0        # generous anti-garbage ceiling. NOT a consistency
                            # claim — real consistency reporting stays in eval_6_1.

# ---------------------------------------------------------------------------
# UNITS (fixed 2026-09-11). /cf_i/rio/delta stamps and the raw EKF estimate
# stamps are SIM seconds (taken from message headers; plan §9 "sim time ≠ ROS
# wall time"). args.duration is WALL seconds. Comparing one against the other
# made rio_alive/ekf_alive unpassable on any host with RTF < RIO_SPAN_FRAC_MIN:
# at RTF ≈ 0.36 a healthy 45 s wall run yields ~16 s of sim span while the check
# demanded >= 36 s. The reference span is therefore a SIM-time span the
# recorder itself observed — never duration_s. The intent ("alive for
# essentially the whole run, not a burst") is unchanged.
#
# WHICH sim span (fixed 2026-09-11, BUG B): NOT the whole /cf_*/odom header
# span. Gazebo publishes odom for the recorder's entire life — wait-for-truth,
# pre-arm hover, takeoff, flight, teardown — so on the 2026-09-11 run that span
# was 53.5 s while RIO/EKF (which only come up for the flight) spanned 19.6 /
# 19.9 s: 0.8 x 53.5 = 42.8 s, an unpassable bar for a perfectly healthy RIO.
# The reference is now the FLIGHT WINDOW: the sim time between the moment the
# scripted path starts and the moment it ends, sampled from the most recent
# /cf_*/odom header stamp at each of those two instants. "RIO was alive for
# essentially the whole flight" is exactly what the frac thresholds then mean.
# sim_reference_span() (whole-odom) is kept only as diagnostic context.
#
# Rates are likewise per SIM second: hz = (n-1) / sim_stamp_span. That is the
# physically meaningful rate for a sensor-driven pipeline — RIO fires once per
# radar scan, and radar scans are scheduled in sim time, so sim-Hz is invariant
# to how fast the host happens to run. RIO publishes ~9.9 sim-Hz (only ~3.6
# wall-Hz at RTF 0.36); RIO_RATE_MIN_HZ = 5.0 is a sim-Hz floor.
# ---------------------------------------------------------------------------
SIM_REF_SPAN_FLOOR_S = 5.0  # Vacuity guard, now applied to the FLIGHT WINDOW.
                            # A reference span shorter than this means the sim
                            # clock barely moved, /cf_*/odom never arrived, or
                            # no flight window was ever marked, so `span >= 0.8 * ref`
                            # would be trivially satisfiable by a burst — or by
                            # nothing at all when ref == 0. Below the floor the
                            # check FAILS outright rather than passing vacuously.
                            # 5 s is well under the shortest scored scenario
                            # (45 s wall ≈ 16 s sim at the worst observed RTF)
                            # yet far above any plausible startup jitter.

# MARKER PLUMBING (fixed 2026-09-11, BUG B3). The flight-window markers used to
# be a plain "last odom header stamp seen on any drone" scalar, written by every
# /cf_*/odom callback with last-write-wins semantics. The live stream is NOT
# uniformly sim-stamped: /cf_*/odom also carries messages whose header stamp is
# 0 (pre-clock / unstamped bridge traffic), which is exactly why every drone's
# whole-run odom span reads min=0.000 (truth [0.000, 111.734]s on the
# 2026-09-11 triangle_forward run). Whenever such a zero-stamped message was the
# most recent arrival at the instant run_flight marked the window — which is
# most of the time — the marker captured 0.0, and start==end==0.0 sank six
# liveness checks on a healthy run. The marker is therefore a HIGH-WATER MARK
# over stamps that are actually set: zero/unset/non-finite stamps never clobber
# it, and it never runs backwards. Thresholds are untouched; a run that truly
# never advances the sim clock still leaves the marker unset -> window 0.0 ->
# FAIL.
ODOM_SIM_STAMP_MIN_S = 1e-3  # an odom header stamp at/below this is "unset"
                             # and must not be taken as a flight-window marker

# ---------------------------------------------------------------------------
# ONE CLOCK (fixed 2026-09-11, BUG A). Truth (/cf_*/odom) and estimates
# (/cf_*/swarm_loc/estimate) must be stamped on the SAME clock or
# eval_6_1.interp_pose pairs nothing: the run of 2026-09-11 wrote truth at WALL
# time (~1.7e9 s, a deliberate "receive-time pairing" hack from when EKF stamps
# were stuck at 0) against estimates at SIM time (~0–20 s), so every estimate
# fell outside the truth span, n=0 paired, and ATE/RPE/NEES came out NaN.
# Both series now carry the odom/EKF HEADER sim time; wall clock is kept as
# metadata only. The receive-time substitution is GONE, not silently
# re-applied: if the EKF stamps are frozen/zero (the hollow-run case) the
# pairing_clock check FAILS loudly rather than manufacturing a plausible ATE.
# ---------------------------------------------------------------------------
EST_STAMP_LIVE_MIN_S = 1e-3      # an estimate stamp at/below this is "unset"
CLOCK_OVERLAP_FRAC_MIN = 0.5     # truth must cover >= half the estimate span


class _Timeout(Exception):
    pass


def _alarm(sig, frame):
    raise _Timeout()


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _cflib_cache_dir() -> str:
    # Do not use the repo cache on /mnt/d — DrvFs makes the first TOC
    # download exceed the connect timeout and looks like a hang.
    cache = os.path.join(os.environ.get("TMPDIR", "/tmp"), "cflib_cache")
    os.makedirs(cache, exist_ok=True)
    return cache


def _port_bound(port: int) -> bool:
    try:
        out = subprocess.run(
            ["ss", "-ulnp"], capture_output=True, text=True, check=False
        )
    except FileNotFoundError:
        return False
    return f":{port}" in out.stdout


def _count_cf2() -> int:
    try:
        out = subprocess.run(
            ["pgrep", "-x", "cf2"], capture_output=True, text=True, check=False
        )
    except FileNotFoundError:
        return -1
    return len([ln for ln in out.stdout.splitlines() if ln.strip()])


def _dump_sitl_log(cf_id: int) -> None:
    log = os.path.join(
        _REPO_ROOT,
        "firmware_mods",
        "CrazySim",
        "crazyflie-firmware",
        "sitl_make",
        "build",
        str(cf_id),
        "error.log",
    )
    if not os.path.isfile(log):
        print(f"[swarm_loc_gate] no SITL error.log at {log}", file=sys.stderr)
        return
    try:
        with open(log, "r", errors="replace") as f:
            tail = f.readlines()[-20:]
        print(f"[swarm_loc_gate] sitl {cf_id} error.log tail:\n{''.join(tail)}", file=sys.stderr)
    except OSError as e:
        print(f"[swarm_loc_gate] could not read {log}: {e}", file=sys.stderr)


def _wait_for_sitl(n: int, connect_wait: float) -> None:
    ports = [19850 + i for i in range(n)]
    deadline = time.time() + connect_wait
    print(f"[swarm_loc_gate] waiting for SITL UDP {ports} …")
    while time.time() < deadline:
        if all(_port_bound(p) for p in ports):
            n_cf2 = _count_cf2()
            print(f"[swarm_loc_gate] SITL UDP ports up (cf2={n_cf2}) — settle 8 s …")
            if n_cf2 >= 0 and n_cf2 < n:
                print(
                    f"[swarm_loc_gate] WARN: only {n_cf2}/{n} cf2 processes",
                    file=sys.stderr,
                )
            time.sleep(8.0)
            return
        time.sleep(0.5)
    raise SystemExit(
        f"[swarm_loc_gate] TIMEOUT: cflib ports not ready.\n{_RESTART_MSG}"
    )


def _open_sync_crazyflie(uri: str, cache: str, label: str, timeout_s: float):
    """One blocking connect (same as uwb_gate). Do not retry — a timed-out
    cflib thread keeps the UDP port and makes later tries fail."""
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    print(f"[swarm_loc_gate] connecting {label} at {uri} ({timeout_s:.0f}s) …")
    holder: dict = {"scf": None, "err": None}

    def _worker():
        try:
            scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache))
            scf.__enter__()
            holder["scf"] = scf
        except Exception as exc:
            holder["err"] = exc

    thread = threading.Thread(target=_worker, daemon=True)
    thread.start()
    thread.join(timeout_s)
    if holder["scf"] is not None:
        print(f"[swarm_loc_gate] {label} connected")
        return holder["scf"]
    why = "timeout (cflib still blocked — restart sim)" if thread.is_alive() else repr(holder.get("err"))
    cf_id = int(uri.rsplit(":", 1)[-1]) - 19850
    _dump_sitl_log(cf_id)
    raise SystemExit(f"[swarm_loc_gate] FAIL connecting {label}: {why}\n{_RESTART_MSG}")


def parse_ros2_node_info_subscribers(text: str) -> List[str]:
    """Extract subscriber topic names from `ros2 node info` output."""
    lines = text.splitlines()
    topics: List[str] = []
    in_subs = False
    for ln in lines:
        s = ln.strip()
        if s.startswith("Subscribers"):
            in_subs = True
            continue
        if in_subs:
            if s.startswith("Publishers") or s.startswith("Service") or s.startswith("Action"):
                break
            if s.endswith(":") and not s.startswith("/"):
                break
            if s.startswith("/"):
                name = s.split(":", 1)[0].split("[", 1)[0].strip()
                if name:
                    topics.append(name)
    return topics


def node_info_subscribers(node_name: str, quiet: bool = False) -> List[str]:
    names = [node_name]
    if node_name.startswith("/"):
        names.append(node_name[1:])
    else:
        names.append("/" + node_name)
    last = ""
    for name in names:
        try:
            out = subprocess.run(
                ["ros2", "node", "info", name],
                capture_output=True,
                text=True,
                check=False,
                timeout=15,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as e:
            print(f"[swarm_loc_gate] ros2 node info failed: {e}", file=sys.stderr)
            return []
        last = (out.stdout or "") + "\n" + (out.stderr or "")
        topics = parse_ros2_node_info_subscribers(last)
        if topics:
            return topics
    if last.strip() and not quiet:
        print(f"[swarm_loc_gate] ros2 node info raw ({node_name}):\n{last[:800]}", file=sys.stderr)
    return []


def inspect_estimator_subs(num_drones: int, attempts: int = 20) -> Tuple[bool, Dict[int, list], Dict[int, list]]:
    """Retry until each /swarm_loc_i lists subscribers. Fail only on truth topics."""
    truth_hits: Dict[int, list] = {}
    all_subs: Dict[int, list] = {}
    readable = False
    leaked_any = False
    for attempt in range(attempts):
        leaked_any = False
        readable = True
        last = attempt == attempts - 1
        for i in range(num_drones):
            subs = node_info_subscribers(f"/swarm_loc_{i}", quiet=not last)
            all_subs[i] = subs
            leaked = topics_contain_truth(subs)
            truth_hits[i] = leaked
            if leaked:
                leaked_any = True
            if not any(t.endswith("/rio/delta") for t in subs):
                readable = False
        if readable or leaked_any:
            break
        time.sleep(1.0)
    return (readable and not leaked_any), truth_hits, all_subs


# ---------------------------------------------------------------------------
# Pure hollow-run checks (no rclpy). Fed by the recorder's collected arrays
# and the on-disk measurement logs; unit-tested by --selftest against a
# synthetic replica of the 2026-09-11 hollow run.
# ---------------------------------------------------------------------------

def sim_reference_span(odom_stamps: Dict[int, List[float]]) -> Tuple[float, str]:
    """SIM-time reference span for the liveness checks.

    Built from the /cf_*/odom header stamps the recorder collects (Gazebo
    publishes odom with sim-time headers). Per drone we take max-min, then the
    MAX across drones: one drone's odom dropping out early must not shrink the
    reference and thereby excuse a dead RIO on another drone.

    Returns (span_s, detail). A span below SIM_REF_SPAN_FLOOR_S is returned as
    is; callers must treat it as a hard failure, not as a lowered bar.
    """
    spans: Dict[int, float] = {}
    for i, st in (odom_stamps or {}).items():
        if st and len(st) >= 2:
            spans[int(i)] = float(max(st) - min(st))
        else:
            spans[int(i)] = 0.0
    span = max(spans.values()) if spans else 0.0
    det = "sim_ref_span=" + (
        f"{span:.1f}s from odom headers " + "{" + ", ".join(
            f"cf_{i}:{spans[i]:.1f}" for i in sorted(spans)
        ) + "}"
        if spans
        else "0.0s (no /cf_*/odom collected)"
    )
    return span, det


def flight_window_span(
    t_start: Optional[float], t_end: Optional[float]
) -> Tuple[float, str]:
    """SIM-time span of the scripted flight, the liveness reference (BUG B).

    `t_start` / `t_end` are the most recent /cf_*/odom header stamps captured at
    the instants the gate began and finished the scripted path. A missing,
    non-finite, zero or backwards window returns 0.0 — callers pass that to
    _sim_ref_guard, which FAILS rather than lowering the bar.
    """
    a = float(t_start) if t_start is not None else float("nan")
    b = float(t_end) if t_end is not None else float("nan")
    if not (math.isfinite(a) and math.isfinite(b)):
        return 0.0, (
            "flight_window=0.0s (no sim-time marker at flight "
            f"{'start' if not math.isfinite(a) else 'end'} — "
            "no /cf_*/odom header had arrived, or the flight never ran)"
        )
    span = b - a
    if span <= 0.0:
        return 0.0, f"flight_window=0.0s (markers not advancing: start={a:.1f}s end={b:.1f}s)"
    return span, f"flight_window={span:.1f}s sim (start={a:.1f}s end={b:.1f}s from odom headers)"


def _sim_ref_guard(sim_ref_span_s: float) -> Optional[str]:
    """Vacuity guard shared by both liveness checks. Returns a failure message
    when the sim-time reference (the FLIGHT WINDOW) is missing/zero/implausibly
    short, else None."""
    ref = float(sim_ref_span_s) if sim_ref_span_s is not None else 0.0
    if not math.isfinite(ref) or ref < SIM_REF_SPAN_FLOOR_S:
        return (
            f"sim-time flight window {ref:.2f}s < floor {SIM_REF_SPAN_FLOOR_S:g}s — "
            "the sim clock barely advanced, /cf_*/odom never arrived, or no "
            "flight window was marked; liveness is unverifiable, so this FAILS "
            "rather than passing vacuously"
        )
    return None


def pairing_clock_check(
    truth_stamps: List[float], est_stamps: List[float]
) -> Tuple[bool, str]:
    """Truth and estimates are on ONE clock, so eval_6_1 can actually pair them.

    BUG A tripwire. Both series must be non-empty, advancing, and overlapping.
    Three distinct failures, each reported explicitly:
      * estimate stamps frozen / at zero  -> hollow run. We refuse to fall back
        to wall-clock pairing; a meaningless ATE must never look plausible.
      * truth stamps frozen / absent      -> nothing to pair against.
      * ranges disjoint (wall ~1.7e9 vs sim ~0-20 s) -> two clocks. FAIL.
    """
    t = np.asarray([v for v in (truth_stamps or []) if math.isfinite(v)], dtype=np.float64)
    e = np.asarray([v for v in (est_stamps or []) if math.isfinite(v)], dtype=np.float64)
    if e.size < 2:
        return False, f"only {e.size} finite estimate stamps — nothing to pair"
    if t.size < 2:
        return False, f"only {t.size} finite truth stamps — nothing to pair against"
    e_span = float(e.max() - e.min())
    t_span = float(t.max() - t.min())
    if float(e.max()) <= EST_STAMP_LIVE_MIN_S or e_span <= 0.0:
        return False, (
            f"estimate stamps frozen/unset (min={e.min():.3f} max={e.max():.3f} "
            f"span={e_span:.3f}s) — hollow run. Refusing the old receive-time "
            "(wall-clock) substitution: it would pair sim truth against wall "
            "estimates and emit a meaningless ATE. FAIL."
        )
    if t_span <= 0.0:
        return False, f"truth stamps frozen (all {t.min():.3f}s) — cannot pair"
    overlap = float(min(t.max(), e.max()) - max(t.min(), e.min()))
    frac = overlap / e_span if e_span > 0 else 0.0
    if overlap <= 0.0:
        return False, (
            f"truth [{t.min():.3f}, {t.max():.3f}] and estimates "
            f"[{e.min():.3f}, {e.max():.3f}] do not overlap — DIFFERENT CLOCKS "
            "(wall vs sim). eval_6_1.interp_pose would pair 0 rows and report "
            "NaN ATE/RPE/NEES. FAIL."
        )
    ok = frac >= CLOCK_OVERLAP_FRAC_MIN
    return ok, (
        f"truth [{t.min():.3f}, {t.max():.3f}]s vs est [{e.min():.3f}, "
        f"{e.max():.3f}]s overlap={overlap:.1f}s ({frac:.2f} of est span, "
        f"need>={CLOCK_OVERLAP_FRAC_MIN:g})"
    )


def rio_alive_check(stamps: List[float], valids: List[int], sim_ref_span_s: float) -> Tuple[bool, str]:
    """/cf_i/rio/delta liveness: rows seen, mostly valid, stamps advancing.

    `stamps` are SIM seconds and `sim_ref_span_s` is the SIM-time reference span
    (see UNITS above) — NOT the wall-clock scored duration. `rate` is sim-Hz.
    """
    bad_ref = _sim_ref_guard(sim_ref_span_s)
    if bad_ref is not None:
        return False, bad_ref
    n = len(stamps)
    if n < 2:
        return False, f"only {n} /rio/delta rows received"
    s = np.asarray(stamps, dtype=np.float64)
    v = np.asarray(valids, dtype=np.float64)
    valid_frac = float(np.mean(v >= 1))
    span = float(s.max() - s.min())
    diffs = np.diff(s)
    adv_frac = float(np.mean(diffs > 0)) if diffs.size else 0.0
    hz = (n - 1) / span if span > 0 else 0.0  # sim-Hz: rows per SIM second
    need_span = RIO_SPAN_FRAC_MIN * float(sim_ref_span_s)
    ok = (
        valid_frac >= RIO_VALID_FRAC_MIN
        and span >= need_span
        and adv_frac >= RIO_ADV_FRAC_MIN
        and hz >= RIO_RATE_MIN_HZ
    )
    return ok, (
        f"n={n} valid_frac={valid_frac:.2f} sim_span={span:.1f}s "
        f"(need>={need_span:.1f}s = {RIO_SPAN_FRAC_MIN:g}×sim_ref {float(sim_ref_span_s):.1f}s) "
        f"adv_frac={adv_frac:.2f} rate={hz:.1f} sim-Hz (need>={RIO_RATE_MIN_HZ:g})"
    )


def expected_seq_inc_frac(cfg: dict) -> float:
    """Fraction of consecutive /swarm_loc/estimate rows expected to show a new
    seq, derived from the config the gate already loads.

    swarm_loc_node publishes the estimate at estimator.rate_hz but bumps seq
    only at comms.broadcast_rate_hz, so the ratio is bc_hz / rate_hz (clamped
    to (0, 1]). Falls back to 1.0 when the config is unreadable — i.e. the old,
    strictest behaviour, never a weaker bar.
    """
    try:
        rate = float(((cfg or {}).get("estimator") or {}).get("rate_hz", 0.0))
        bc = float(((cfg or {}).get("comms") or {}).get("broadcast_rate_hz", 0.0))
    except (TypeError, ValueError):
        return 1.0
    if not (math.isfinite(rate) and math.isfinite(bc)) or rate <= 0.0 or bc <= 0.0:
        return 1.0
    return float(min(1.0, bc / rate))


def ekf_alive_check(
    stamps: List[float],
    seqs: List[int],
    sim_ref_span_s: float,
    seq_frac_expected: float = 1.0,
) -> Tuple[bool, str]:
    """EKF publish liveness: message stamps advance (not frozen at 0/constant)
    and seq increments. Uses the RAW stamp field, before any receive-time
    substitution the recorder does for pairing.

    `stamps` are SIM seconds and `sim_ref_span_s` is the SIM-time reference span
    (see UNITS above) — NOT the wall-clock scored duration.
    """
    bad_ref = _sim_ref_guard(sim_ref_span_s)
    if bad_ref is not None:
        return False, bad_ref
    n = len(stamps)
    if n < 2:
        return False, f"only {n} estimate rows received"
    s = np.asarray(stamps, dtype=np.float64)
    q = np.asarray(seqs, dtype=np.int64)
    span = float(s.max() - s.min())
    need_span = EKF_SPAN_FRAC_MIN * float(sim_ref_span_s)
    seq_inc = np.diff(q)
    seq_frac = float(np.mean(seq_inc > 0)) if seq_inc.size else 0.0
    dec_frac = float(np.mean(seq_inc < 0)) if seq_inc.size else 0.0
    exp = float(seq_frac_expected) if seq_frac_expected is not None else 1.0
    if not math.isfinite(exp) or exp <= 0.0:
        exp = 1.0
    need_seq = max(EKF_SEQ_FRAC_FLOOR, EKF_SEQ_FRAC_MARGIN * min(exp, 1.0))
    ok = (
        span >= need_span
        and int(q.max()) > int(q.min())
        and seq_frac >= need_seq
        and dec_frac <= EKF_SEQ_DECREASE_FRAC_MAX
    )
    return ok, (
        f"n={n} stamp_sim_span={span:.1f}s "
        f"(need>={need_span:.1f}s = {EKF_SPAN_FRAC_MIN:g}×sim_ref {float(sim_ref_span_s):.1f}s) "
        f"seq={int(q.min())}..{int(q.max())} seq_inc_frac={seq_frac:.2f} "
        f"(need>={need_seq:.2f} = max({EKF_SEQ_FRAC_FLOOR:g}, "
        f"{EKF_SEQ_FRAC_MARGIN:g}×expected {exp:.2f}=bc_hz/rate_hz)) "
        f"seq_dec_frac={dec_frac:.3f} (need<={EKF_SEQ_DECREASE_FRAC_MAX:g})"
    )


def logs_intact_check(log_dir: str, num_drones: int) -> Tuple[bool, str, Optional[Dict[int, dict]]]:
    """Every cf_i.npz exists, loads, and has non-empty rio + estimate arrays.
    (uwb totals are gated separately by uwb_consumed.) Returns the loaded run
    so later checks reuse it."""
    from meas_log import load_drone_log

    if not log_dir or not Path(log_dir).is_dir():
        return False, f"log dir missing: {log_dir!r}", None
    run: Dict[int, dict] = {}
    problems: List[str] = []
    for i in range(num_drones):
        p = Path(log_dir) / f"cf_{i}.npz"
        if not p.is_file():
            problems.append(f"cf_{i}.npz missing")
            continue
        try:
            rec = load_drone_log(p)
        except Exception as e:
            problems.append(f"cf_{i}.npz unloadable: {e}")
            continue
        if int(getattr(rec["rio"], "size", 0)) == 0:
            problems.append(f"cf_{i} rio log empty")
        if int(getattr(rec["estimate"], "size", 0)) == 0:
            problems.append(f"cf_{i} estimate log empty")
        run[int(rec["drone_id"])] = rec
    ok = not problems and len(run) == num_drones
    return ok, ("; ".join(problems) if problems else "all logs present and non-empty"), (run or None)


def uwb_consumed_check(run: Optional[Dict[int, dict]], num_drones: int, duration_s: float) -> Tuple[bool, str]:
    """The EKF actually fused something: total n_update > 0, total UWB edge
    rows > 0, and every drone cleared a rate-based per-drone update floor."""
    if not run:
        return False, "no measurement logs loaded"
    floor = max(1, int(math.ceil(UPDATE_RATE_FLOOR_HZ * float(duration_s))))
    total_upd = 0
    total_uwb = 0
    bad: List[str] = []
    for i in range(num_drones):
        rec = run.get(i)
        if rec is None:
            bad.append(f"cf_{i} log missing")
            continue
        n_upd = int(rec.get("stats", {}).get("n_update", 0))
        uwb = rec.get("uwb")
        n_uwb = int(getattr(uwb, "shape", (0,))[0]) if uwb is not None else 0
        total_upd += n_upd
        total_uwb += n_uwb
        if n_upd < floor:
            bad.append(f"cf_{i} n_update={n_upd} < floor {floor}")
    ok = total_upd > 0 and total_uwb > 0 and not bad
    det = f"total n_update={total_upd} total n_uwb={total_uwb} floor={floor}/drone"
    if bad:
        det += "; " + "; ".join(bad)
    return ok, det


def hops_valid_check(hops: Dict, num_drones: int) -> Tuple[bool, str]:
    """The measurement graph connects every drone to the entrance (hop != -1)."""
    if not hops:
        return False, "no hop data"
    h = {int(k): int(v) for k, v in hops.items()}
    bad = [i for i in range(num_drones) if h.get(i, -1) < 1]
    det = f"hops={h}"
    if bad:
        det += f" — drones {bad} have no path to the entrance"
    return (not bad), det


def nees_sane_check(per_drone: Dict) -> Tuple[bool, str]:
    """Anti-garbage tripwire only: mean NEES finite and under a generous
    ceiling. Real consistency reporting (frac_nees_in_95 etc.) stays in the
    eval_6_1 report unchanged."""
    if not per_drone:
        return False, "no per-drone metrics"
    bad: List[str] = []
    for k, m in per_drone.items():
        v = float(m.get("mean_nees", float("nan")))
        if not math.isfinite(v) or v >= NEES_MEAN_MAX:
            bad.append(f"cf_{k} mean_nees={v:.1f}")
    return (not bad), ("; ".join(bad) if bad else f"all mean NEES < {NEES_MEAN_MAX:g}")


def mix_nonzero_check(mix: Dict) -> Tuple[bool, str]:
    """UWB measurement mix is not all zeros."""
    if not mix:
        return False, "no mix stats"
    n_uwb = int(mix.get("n_uwb", 0))
    frac = float(mix.get("frac_bearing", 0.0)) + float(mix.get("frac_range_only", 0.0))
    ok = n_uwb > 0 and frac > 0.0
    return ok, f"n_uwb={n_uwb} frac_bearing+frac_range_only={frac:.3f}"


class EstimateRecorder:
    def _init_state(self, num_drones: int) -> None:
        """Pure-Python collection state (no rclpy).

        Split out of __init__ so --selftest can drive the REAL callback and
        marker-capture code paths against synthetic messages; the 87-case suite
        passed while BUG B3 shipped precisely because only the pure
        flight_window_span() helper was ever exercised, never the capture.
        """
        self.rows: Dict[int, list] = {i: [] for i in range(num_drones)}
        self.truth: Dict[int, list] = {i: [] for i in range(num_drones)}
        # Raw liveness meta, untouched by receive-time pairing:
        #   est_meta[i]: (recv_wall, raw_msg_stamp, seq)
        #   rio_meta[i]: (recv_wall, msg_stamp, valid)
        self.est_meta: Dict[int, list] = {i: [] for i in range(num_drones)}
        self.rio_meta: Dict[int, list] = {i: [] for i in range(num_drones)}
        # Odom HEADER stamps (sim seconds). These are ALSO what self.truth is
        # stamped with (BUG A: one clock) — kept separately because the
        # diagnostic whole-run odom span is still printed.
        self.odom_sim_stamps: Dict[int, list] = {i: [] for i in range(num_drones)}
        # Odom messages that carried no usable header stamp. Non-zero means the
        # truth series is not trustworthy sim time; surfaced as a check.
        self.odom_header_missing: Dict[int, int] = {i: 0 for i in range(num_drones)}
        # High-water mark of the odom header sim clock across ALL drones, and
        # the sim-time markers captured at flight start / end (BUG B / B3).
        # NaN = no /cf_*/odom message with a set header stamp has arrived yet;
        # that propagates to the markers and FAILS the liveness checks rather
        # than passing vacuously.
        self._sim_lock = threading.Lock()
        self._last_odom_sim: float = float("nan")
        self.flight_sim_t0: Optional[float] = None
        self.flight_sim_t1: Optional[float] = None

    def __init__(self, num_drones: int):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2

        self._init_state(num_drones)

        class _Node(Node):
            def __init__(self_inner):
                super().__init__("swarm_loc_gate_recorder")
                qos = qos_profile_sensor_data
                for i in range(num_drones):
                    self_inner.create_subscription(
                        PointCloud2,
                        f"/cf_{i}/swarm_loc/estimate",
                        lambda msg, idx=i: self._on_est(idx, msg),
                        qos,
                    )
                    self_inner.create_subscription(
                        PointCloud2,
                        f"/cf_{i}/rio/delta",
                        lambda msg, idx=i: self._on_rio(idx, msg),
                        qos,
                    )
                    subscribe_gz(
                        self_inner,
                        Odometry,
                        f"/cf_{i}/odom",
                        lambda msg, idx=i: self._on_odom(idx, msg),
                    )

        rclpy.init()
        self._node = _Node()
        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self._node)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()

    def wait_truth(self, timeout_s: float = 30.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if all(self.truth[i] for i in self.truth):
                n = {i: len(self.truth[i]) for i in self.truth}
                print(f"[swarm_loc_gate] ROS odom truth live {n}", flush=True)
                return True
            time.sleep(0.2)
        missing = [i for i in self.truth if not self.truth[i]]
        print(
            f"[swarm_loc_gate] no ROS /cf_*/odom after {timeout_s:.0f}s "
            f"(missing cf_{missing})",
            flush=True,
        )
        return False

    def sim_now(self) -> float:
        """Sim-clock high-water mark from /cf_*/odom headers, or NaN if unset.

        Read under the same lock the odom callbacks write under: the recorder
        spins on a MultiThreadedExecutor in a daemon thread, so the marker is
        sampled from another thread than the one updating it.
        """
        with self._sim_lock:
            return float(self._last_odom_sim)

    def _note_odom_sim(self, stamp: float) -> None:
        """Advance the odom sim-clock high-water mark (BUG B3).

        Only stamps that are actually SET count. /cf_*/odom carries interleaved
        zero-stamped messages; a plain last-write-wins scalar captured 0.0 at
        the flight markers and zeroed the liveness reference on a healthy run.
        """
        if not math.isfinite(stamp) or stamp <= ODOM_SIM_STAMP_MIN_S:
            return
        with self._sim_lock:
            prev = self._last_odom_sim
            if not math.isfinite(prev) or stamp > prev:
                self._last_odom_sim = float(stamp)

    def mark_flight_start(self) -> None:
        """Capture the sim-time marker for the start of the scored flight."""
        self.flight_sim_t0 = self.sim_now()
        print(f"[swarm_loc_gate] flight window start (sim) = {self.flight_sim_t0}", flush=True)

    def mark_flight_end(self) -> None:
        """Capture the sim-time marker for the end of the scored flight."""
        self.flight_sim_t1 = self.sim_now()
        print(f"[swarm_loc_gate] flight window end (sim) = {self.flight_sim_t1}", flush=True)

    def truth_sim_stamps(self, idx: int) -> List[float]:
        return [t for t, _, _, _, _ in self.truth[idx]]

    def _on_est(self, idx: int, msg) -> None:
        arr = unpack_state(msg)
        if arr.size:
            row = np.array(arr[0], copy=True)
            now = time.time()
            # Keep the raw stamp/seq for the ekf_alive check BEFORE any
            # receive-time substitution — frozen stamps must stay visible.
            self.est_meta[idx].append((now, float(row["stamp"]), int(row["seq"])))
            # BUG A: the stamp is left EXACTLY as published (sim seconds). The
            # old `if stamp <= 1e-3: stamp = time.time()` receive-time
            # substitution silently put estimates on the wall clock while truth
            # stayed on sim time, so interp_pose paired nothing. A frozen/zero
            # stamp is now a FAILURE (pairing_clock_check), not a fallback.
            self.rows[idx].append((now, row))

    def _on_rio(self, idx: int, msg) -> None:
        now = time.time()
        for row in unpack_rio(msg):
            self.rio_meta[idx].append((now, float(row["stamp"]), int(row["valid"])))

    def _on_odom(self, idx: int, msg) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # BUG A: truth is stamped with the odom HEADER sim time — the same
        # clock the EKF stamps its estimates with. Wall time is metadata only.
        try:
            h = msg.header.stamp
            stamp = float(h.sec) + float(h.nanosec) * 1e-9
            self.odom_sim_stamps[idx].append(stamp)
            self._note_odom_sim(stamp)
        except AttributeError:
            # No header stamp at all: record the gap loudly and store NaN so the
            # row can never masquerade as a valid sim-time sample.
            self.odom_header_missing[idx] += 1
            stamp = float("nan")
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.truth[idx].append((stamp, float(p.x), float(p.y), float(p.z), float(yaw)))

    def dump_eval(self, out_dir: str) -> None:
        from eval_6_1 import TRUTH_DTYPE, write_eval_bundle

        estimates = {}
        truth = {}
        for i, rec in self.rows.items():
            if rec:
                estimates[i] = np.array([row for _, row in rec], dtype=STATE_DTYPE)
            else:
                estimates[i] = np.zeros(0, dtype=STATE_DTYPE)
        for i, rec in self.truth.items():
            # Drop header-less rows (stamp NaN): interp_pose does a
            # searchsorted on this array, so a NaN stamp would corrupt pairing.
            rec = [r for r in rec if math.isfinite(r[0])]
            arr = np.zeros(len(rec), dtype=TRUTH_DTYPE)
            for k, (stamp, x, y, z, psi) in enumerate(rec):
                arr[k]["stamp"] = stamp
                arr[k]["p_x"], arr[k]["p_y"], arr[k]["p_z"] = x, y, z
                arr[k]["psi"] = psi
            truth[i] = arr
        write_eval_bundle(Path(out_dir), estimates, truth)
        from eval_6_1 import write_score_window

        write_score_window(
            Path(out_dir), self.score_start(), None, "flight_sim_t0 (start of scripted path)"
        )

    def score_start(self) -> Optional[float]:
        """Sim time scoring starts at, or None if the scripted path was never marked."""
        t0 = self.flight_sim_t0
        if t0 is None or not math.isfinite(t0) or t0 <= ODOM_SIM_STAMP_MIN_S:
            return None
        return float(t0)

    def shutdown(self) -> None:
        try:
            self._exec.shutdown()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            import rclpy

            rclpy.shutdown()
        except Exception:
            pass


class _Link:
    """Holds a Crazyflie plus optional Swarm so links stay open."""

    def __init__(self, cf, swarm=None):
        self.cf = cf
        self._swarm = swarm

    def __exit__(self, *args):
        if self._swarm is not None:
            try:
                self._swarm.close_links()
            except Exception:
                pass
            self._swarm = None
        else:
            try:
                self.cf.close_link()
            except Exception:
                pass


def connect_drones(args) -> list:
    """Open every SITL radio at once. Sequential connect hangs on drone 1
    once drone 0 already owns a UDP link (cflib/CrazySim)."""
    import cflib.crtp

    n = int(args.num_drones)
    _wait_for_sitl(n, args.connect_wait)
    cache = _cflib_cache_dir()
    cflib.crtp.init_drivers()
    uris = [f"udp://127.0.0.1:{19850 + i}" for i in range(n)]
    timeout_s = float(args.connect_timeout)

    try:
        from cflib.crazyflie.swarm import CachedCfFactory, Swarm

        factory = CachedCfFactory(rw_cache=cache)
        swarm = Swarm(uris, factory=factory)
        print(f"[swarm_loc_gate] opening {n} SITL links in parallel ({timeout_s:.0f}s) …")
        done = threading.Event()
        err: list = []

        def _open():
            try:
                swarm.open_links()
            except Exception as exc:
                err.append(exc)
            done.set()

        t = threading.Thread(target=_open, daemon=True)
        t.start()
        if not done.wait(timeout_s):
            print(
                "[swarm_loc_gate] cflib parallel open timed out. "
                "Do not treat this as a flight. Restart sim (pkill gz sim/cf2) and retry. "
                "A hung cflib thread may still hold UDP — --no-fly fallback is not a fly.",
                file=sys.stderr,
            )
            return None
        if err:
            print(f"[swarm_loc_gate] cflib Swarm failed: {err[0]}. Falling back to --no-fly.", file=sys.stderr)
            return None
        links = []
        for i, uri in enumerate(uris):
            cf = swarm._cfs[uri]
            print(f"[swarm_loc_gate] drone {i} connected ({uri})")
            links.append(_Link(cf, swarm if i == 0 else None))
        return links
    except ImportError:
        pass

    # Fallback: start every SyncCrazyflie thread at t=0 (still parallel).
    print(f"[swarm_loc_gate] Swarm API missing — parallel SyncCrazyflie …")
    holder: dict = {}

    def _one(i, uri):
        holder[i] = _open_sync_crazyflie(uri, cache, f"drone {i}", timeout_s)

    threads = [
        threading.Thread(target=_one, args=(i, uris[i]), daemon=True) for i in range(n)
    ]
    for th in threads:
        th.start()
    for th in threads:
        th.join(timeout_s + 2.0)
    missing = [i for i in range(n) if i not in holder]
    if missing:
        raise SystemExit(
            f"[swarm_loc_gate] FAIL connecting drones {missing}. Use --no-fly.\n{_RESTART_MSG}"
        )
    return [holder[i] for i in range(n)]


def _as_cf(obj):
    """Unwrap _Link / SyncCrazyflie down to Crazyflie (has .param)."""
    x = obj
    for _ in range(4):
        if x is None:
            break
        if hasattr(x, "param") and hasattr(x, "platform"):
            return x
        x = getattr(x, "cf", None)
    raise TypeError(f"cannot unwrap Crazyflie from {type(obj)}")


def run_flight(args, scfs: list, recorder=None) -> bool:
    from pid_gains import apply_gains, load_gains, reset_estimator, reset_pose
    from cflib.positioning.motion_commander import MotionCommander

    n = int(args.num_drones)
    gains = load_gains(args.gains)
    signal.signal(signal.SIGALRM, _alarm)
    diverged = False
    try:
        for i, scf in enumerate(scfs):
            cf = _as_cf(scf)
            apply_gains(cf, gains)
            try:
                xy = args._spawn_xy[i] if getattr(args, "_spawn_xy", None) else (
                    float(i * args.spacing),
                    0.0,
                )
                reset_pose(
                    args.world,
                    f"{args.model_prefix}_{i}",
                    xyz=(float(xy[0]), float(xy[1]), args.hover_height),
                )
            except Exception as e:
                print(f"[swarm_loc_gate] reset_pose {i} skipped: {e}", file=sys.stderr)
            reset_estimator(cf, "kalman")
        time.sleep(2.0)
        for scf in scfs:
            try:
                _as_cf(scf).platform.send_arming_request(True)
            except Exception:
                pass
        time.sleep(0.5)
        signal.setitimer(signal.ITIMER_REAL, float(args.duration) + 60.0)
        mcs = [MotionCommander(_as_cf(s), default_height=args.hover_height) for s in scfs]
        # MotionCommander is a context manager; nest via ExitStack
        from contextlib import ExitStack

        with ExitStack() as stack:
            for mc in mcs:
                stack.enter_context(mc)
            print("[swarm_loc_gate] takeoff/settle 4 s …")
            time.sleep(4.0)
            t_end = time.time() + float(args.duration)
            spec = getattr(args, "_scenario", None)
            label = spec["key"] if spec else "tunnel/collinear_hover (default motion)"
            print(f"[swarm_loc_gate] scripted path {label} for {args.duration:.0f} s …")
            # BUG B: the liveness reference is the FLIGHT window, not the whole
            # recorder life. Mark it around the scripted path only.
            if recorder is not None:
                recorder.mark_flight_start()
            try:
                if spec is not None:
                    apply_motion(mcs, t_end, spec)
                else:
                    apply_motion(mcs, t_end, get_scenario("tunnel/collinear_hover"))
            except Exception as e:
                print(f"[swarm_loc_gate] motion warning: {e}", file=sys.stderr)
            finally:
                if recorder is not None:
                    recorder.mark_flight_end()
            for mc in mcs:
                try:
                    mc.stop()
                except Exception:
                    pass
    except _Timeout:
        diverged = True
        print("[swarm_loc_gate] TIMEOUT during flight", file=sys.stderr)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        for scf in reversed(scfs):
            try:
                scf.__exit__(None, None, None)
            except Exception:
                pass
    return diverged


def _rate_ok(stamps: List[float], target_hz: float, window_s: float) -> Tuple[bool, float]:
    if len(stamps) < 2 or window_s <= 0:
        return False, 0.0
    hz = (len(stamps) - 1) / window_s
    return hz >= RATE_FRAC_MIN * target_hz, hz


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

    sample = """
/swarm_loc_0
  Subscribers:
    /cf_0/rio/delta: sensor_msgs/msg/PointCloud2
    /cf_0/uwb/edges: sensor_msgs/msg/PointCloud2
    /cf_1/swarm_loc/broadcast: sensor_msgs/msg/PointCloud2
  Publishers:
    /cf_0/swarm_loc/estimate: sensor_msgs/msg/PointCloud2
"""
    subs = parse_ros2_node_info_subscribers(sample)
    check("1 parse rio", "/cf_0/rio/delta" in subs)
    check("1b parse no publishers", "/cf_0/swarm_loc/estimate" not in subs)
    alt = parse_ros2_node_info_subscribers(
        "Node: /swarm_loc_0\n  Subscribers (3):\n    /cf_0/rio/delta [sensor_msgs/msg/PointCloud2]\n  Publishers:\n"
    )
    check("1c alt format", "/cf_0/rio/delta" in alt)
    check("2 truth catch odom", bool(topics_contain_truth(["/cf_0/odom"])))
    check("2b estimate not truth", not topics_contain_truth(["/cf_0/swarm_loc/estimate"]))
    spec = get_scenario("tunnel/triangle_forward")
    check("3 scenario key", spec["world"] == "phase0_tunnel_gate")
    check("3b eval subdir", eval_dir_for(spec) == "out/swarm_loc_eval/tunnel/triangle_forward")

    # ------------------------------------------------------------------
    # Hollow-run tripwires. State below replicates the 2026-09-11
    # triangle_forward artifacts (empty rio/uwb/estimate logs, n_update=0,
    # EKF stamps frozen at 0, hops -1, mean NEES 170–2360, mix all zero):
    # every new check must FAIL on it individually, and PASS on a healthy
    # synthetic run. No rclpy needed — the checks are pure functions.
    # ------------------------------------------------------------------
    import tempfile

    from meas_log import KIND_ENTRANCE_RANGE, MeasurementLogger

    entrance = 1000
    nanv = float("nan")
    dur = 180.0

    # rio_alive — hollow: no rows / frozen stamps / low rate / invalid deltas
    check("4 rio_alive fails: no rows", not rio_alive_check([], [], dur)[0])
    r_ok, r_det = rio_alive_check([0.0] * 900, [1] * 900, dur)
    check("4b rio_alive fails: stamps frozen at 0", not r_ok, r_det)
    r_ok, r_det = rio_alive_check(list(np.linspace(0, dur, 200)), [1] * 200, dur)
    check("4c rio_alive fails: rate under floor", not r_ok, r_det)
    r_ok, r_det = rio_alive_check(list(np.linspace(0, dur, 9001)), [0] * 9001, dur)
    check("4d rio_alive fails: valid==0 rows", not r_ok, r_det)

    # ekf_alive — hollow: publish stamps stuck at 0, or seq frozen
    e_ok, e_det = ekf_alive_check([0.0] * 3600, list(range(3600)), dur)
    check("5 ekf_alive fails: stamps stuck at 0", not e_ok, e_det)
    e_ok, e_det = ekf_alive_check(list(np.linspace(0, dur, 3600)), [7] * 3600, dur)
    check("5b ekf_alive fails: seq frozen", not e_ok, e_det)

    # logs_intact + uwb_consumed — hollow: files exist but every buffer empty,
    # n_update == 0 (exactly what the 2026-09-11 cf_*.npz contain)
    with tempfile.TemporaryDirectory() as td:
        tdp = Path(td)
        for i in range(3):
            lg = MeasurementLogger(i)
            lg.set_stats(n_update=0, cpu_per_step_s=0.0, comms_bytes_per_s=836.8)
            lg.save(tdp / f"cf_{i}.npz")
        l_ok, l_det, run_h = logs_intact_check(str(tdp), 3)
        check("6 logs_intact fails: empty buffers", not l_ok, l_det)
        u_ok, u_det = uwb_consumed_check(run_h, 3, dur)
        check("6b uwb_consumed fails: n_update=0, n_uwb=0", not u_ok, u_det)
    l_ok, l_det, _ = logs_intact_check(str(Path(tempfile.gettempdir()) / "no_such_swarm_loc_dir"), 3)
    check("6c logs_intact fails: dir missing", not l_ok, l_det)
    check("6d uwb_consumed fails: no run", not uwb_consumed_check(None, 3, dur)[0])

    # hops_valid / nees_sane / mix_nonzero — hollow values verbatim from the
    # 2026-09-11 metrics_6_1.json
    h_ok, h_det = hops_valid_check({0: -1, 1: -1, 2: -1}, 3)
    check("7 hops_valid fails: hops all -1", not h_ok, h_det)
    n_ok, n_det = nees_sane_check(
        {"0": {"mean_nees": 228.14}, "1": {"mean_nees": 171.81}, "2": {"mean_nees": 2362.25}}
    )
    check("7b nees_sane fails: 2026-09-11 NEES values", not n_ok, n_det)
    check("7c nees_sane fails: empty report", not nees_sane_check({})[0])
    m_ok, m_det = mix_nonzero_check({"n_uwb": 0, "frac_bearing": 0.0, "frac_range_only": 0.0})
    check("7d mix_nonzero fails: all-zero mix", not m_ok, m_det)

    # Healthy synthetic run — every new check passes
    r_ok, r_det = rio_alive_check(list(np.linspace(0.0, dur, 9001)), [1] * 9001, dur)
    check("8 rio_alive passes healthy", r_ok, r_det)
    e_ok, e_det = ekf_alive_check(list(np.linspace(0.0, dur, 3600)), list(range(3600)), dur)
    check("8b ekf_alive passes healthy", e_ok, e_det)
    with tempfile.TemporaryDirectory() as td:
        tdp = Path(td)
        for i in range(3):
            lg = MeasurementLogger(i)
            for tk in np.arange(0.0, dur, 0.5):
                lg.add_rio(tk, 0.02, [0.01, 0.0, 0.0], 0.001, 0.0, 0.0, True)
                lg.add_est(tk, [0.0, 0.0, 0.5], [0.0, 0.0, 0.0], 0.0, 0)
                lg.add_uwb(
                    tk, KIND_ENTRANCE_RANGE, i, entrance, 2.0, nanv, nanv,
                    0.08, nanv, nanv, 0.0, 0.0, 0.0,
                )
            lg.set_stats(n_update=400)
            lg.save(tdp / f"cf_{i}.npz")
        l_ok, l_det, run_ok = logs_intact_check(str(tdp), 3)
        check("8c logs_intact passes healthy", l_ok, l_det)
        u_ok, u_det = uwb_consumed_check(run_ok, 3, dur)
        check("8d uwb_consumed passes healthy", u_ok, u_det)
    h_ok, h_det = hops_valid_check({0: 1, 1: 1, 2: 2}, 3)
    check("8e hops_valid passes healthy", h_ok, h_det)
    n_ok, n_det = nees_sane_check({"0": {"mean_nees": 3.1}, "1": {"mean_nees": 2.8}})
    check("8f nees_sane passes healthy", n_ok, n_det)
    m_ok, m_det = mix_nonzero_check({"n_uwb": 5000, "frac_bearing": 0.4, "frac_range_only": 0.6})
    check("8g mix_nonzero passes healthy", m_ok, m_det)

    # ------------------------------------------------------------------
    # Sim-time vs wall-clock units (2026-09-11 regression). The live
    # triangle_forward run was healthy on every quality criterion
    # (valid_frac=1.00, adv_frac=1.00, 9.9 sim-Hz) yet rio_alive/ekf_alive
    # FAILED because 16.2 s of SIM span was compared against 0.8 × 45 s of
    # WALL duration. Case 9 is that run; it must PASS now. Cases 9c–9e keep
    # the check from becoming vacuous.
    # ------------------------------------------------------------------
    wall_dur = 45.0          # scenario duration, wall seconds
    sim_span = 16.2          # what the sim clock actually advanced (RTF ≈ 0.36)
    n_rio = int(round(9.9 * sim_span)) + 1     # ~9.9 sim-Hz
    rio_s = list(np.linspace(0.0, sim_span, n_rio))
    n_ekf = int(round(45.0 * sim_span)) + 1    # EKF ~45 sim-Hz
    ekf_s = list(np.linspace(0.0, sim_span, n_ekf))

    # sim reference span from odom header stamps
    odom = {i: list(np.linspace(0.0, sim_span, 1600)) for i in range(3)}
    ref, ref_det = sim_reference_span(odom)
    check("9 sim_reference_span from odom headers", abs(ref - sim_span) < 1e-6, ref_det)

    r_ok, r_det = rio_alive_check(rio_s, [1] * n_rio, ref)
    check("9a rio_alive passes healthy slow-RTF run", r_ok, r_det)
    e_ok, e_det = ekf_alive_check(ekf_s, list(range(n_ekf)), ref)
    check("9b ekf_alive passes healthy slow-RTF run", e_ok, e_det)
    # The bug being fixed: against the WALL duration the same healthy run fails.
    check(
        "9c same run would fail against wall duration (the bug)",
        not rio_alive_check(rio_s, [1] * n_rio, wall_dur)[0],
    )

    # Nothing published at all — sim reference span ~0 must FAIL, never pass
    # vacuously by comparing a burst against ~0.
    ref0, ref0_det = sim_reference_span({0: [], 1: [], 2: []})
    check("9d sim_reference_span 0 when no odom", ref0 == 0.0, ref0_det)
    r_ok, r_det = rio_alive_check(rio_s, [1] * n_rio, ref0)
    check("9e rio_alive fails: sim ref span ~0", not r_ok, r_det)
    e_ok, e_det = ekf_alive_check(ekf_s, list(range(n_ekf)), ref0)
    check("9f ekf_alive fails: sim ref span ~0", not e_ok, e_det)
    # A frozen sim clock (odom arriving, stamps identical) is equally vacuous.
    ref_frozen, _ = sim_reference_span({0: [12.5] * 900})
    check(
        "9g rio_alive fails: frozen sim clock",
        (not rio_alive_check([12.5] * 900, [1] * 900, ref_frozen)[0]) and ref_frozen == 0.0,
    )
    # Just under / just over the absolute floor.
    check(
        "9h rio_alive fails: sim ref just under floor",
        not rio_alive_check(
            list(np.linspace(0.0, 4.9, 50)), [1] * 50, SIM_REF_SPAN_FLOOR_S - 0.1
        )[0],
    )
    check(
        "9i rio_alive passes: sim ref just over floor",
        rio_alive_check(
            list(np.linspace(0.0, 6.0, 60)), [1] * 60, SIM_REF_SPAN_FLOOR_S + 1.0
        )[0],
    )

    # RIO dies partway: stamps span only ~30% of the sim reference.
    part = 0.30 * sim_span
    n_part = int(round(9.9 * part)) + 1
    r_ok, r_det = rio_alive_check(list(np.linspace(0.0, part, n_part)), [1] * n_part, ref)
    check("9j rio_alive fails: RIO dies partway (30% of sim ref)", not r_ok, r_det)
    n_pe = int(round(45.0 * part)) + 1
    e_ok, e_det = ekf_alive_check(
        list(np.linspace(0.0, part, n_pe)), list(range(n_pe)), ref
    )
    check("9k ekf_alive fails: EKF dies partway (30% of sim ref)", not e_ok, e_det)

    # Hollow run (stamps frozen at 0) still fails even with a healthy sim ref.
    check("9l rio_alive still fails hollow run vs sim ref", not rio_alive_check([0.0] * 900, [1] * 900, ref)[0])
    check("9m ekf_alive still fails hollow run vs sim ref", not ekf_alive_check([0.0] * 3600, list(range(3600)), ref)[0])
    check("9n rio_alive still fails zero rows vs sim ref", not rio_alive_check([], [], ref)[0])
    # Rate floor is sim-Hz: a real-but-slow 2 sim-Hz RIO still fails.
    n_slow = int(round(2.0 * sim_span)) + 1
    r_ok, r_det = rio_alive_check(list(np.linspace(0.0, sim_span, n_slow)), [1] * n_slow, ref)
    check("9o rio_alive fails: 2 sim-Hz under rate floor", not r_ok, r_det)

    # ------------------------------------------------------------------
    # BUG B (2026-09-11): the liveness reference must be the FLIGHT window,
    # not the whole /cf_*/odom span. On the live run odom spanned 53.5 s
    # (wait-for-truth + pre-arm hover + flight) while RIO/EKF — which only come
    # up for the flight — spanned 19.6 / 19.9 s. 0.8 × 53.5 = 42.8 s failed a
    # perfectly healthy RIO. Cases 10c/10d pin failing-before / passing-after.
    # ------------------------------------------------------------------
    fw, fw_det = flight_window_span(12.0, 32.0)
    check("10 flight_window_span healthy", abs(fw - 20.0) < 1e-9, fw_det)
    fw0, fw0_det = flight_window_span(None, None)
    check("10a flight_window_span 0 when unmarked", fw0 == 0.0, fw0_det)
    check("10a2 flight_window_span 0 when end marker missing", flight_window_span(12.0, None)[0] == 0.0)
    check("10b flight_window_span 0 when backwards", flight_window_span(32.0, 12.0)[0] == 0.0)
    check("10b2 flight_window_span 0 when zero-length", flight_window_span(32.0, 32.0)[0] == 0.0)

    odom_span_live = 53.5      # whole recorder life, sim seconds (the old ref)
    flight_win = 20.0          # scripted path only, sim seconds (the new ref)
    rio_span_live = 19.6
    ekf_span_live = 19.9
    n_rio_l = int(round(9.4 * rio_span_live)) + 1      # measured ~9.4 sim-Hz
    rio_l = list(np.linspace(12.2, 12.2 + rio_span_live, n_rio_l))
    n_ekf_l = int(round(44.0 * ekf_span_live)) + 1     # measured ~44 sim-Hz
    ekf_l = list(np.linspace(12.1, 12.1 + ekf_span_live, n_ekf_l))
    # seq bumped at 10 Hz while estimates publish at ~44 Hz (BUG B2 shape)
    seq_l = [7 + int(k * 10.0 / 44.0) for k in range(n_ekf_l)]
    odom_live = {i: list(np.linspace(0.0, odom_span_live, 5000)) for i in range(3)}
    ref_odom_live, _ = sim_reference_span(odom_live)
    check("10c whole-odom ref is the 53.5 s the bug used", abs(ref_odom_live - odom_span_live) < 1e-6)
    # BEFORE (pinned): the identical healthy arrays fail against the odom span.
    check(
        "10c2 rio_alive FAILED before: healthy RIO vs whole-odom span (the bug)",
        not rio_alive_check(rio_l, [1] * n_rio_l, ref_odom_live)[0],
    )
    check(
        "10c3 ekf_alive FAILED before: healthy EKF vs whole-odom span (the bug)",
        not ekf_alive_check(ekf_l, seq_l, ref_odom_live, seq_frac_expected=0.2)[0],
    )
    # AFTER: the same arrays pass against the flight window.
    r_ok, r_det = rio_alive_check(rio_l, [1] * n_rio_l, flight_win)
    check("10d rio_alive PASSES now: RIO covers the flight window", r_ok, r_det)
    e_ok, e_det = ekf_alive_check(ekf_l, seq_l, flight_win, seq_frac_expected=0.2)
    check("10e ekf_alive PASSES now: EKF covers the flight window", e_ok, e_det)

    # RIO dying partway through the FLIGHT window still FAILS.
    n_half = int(round(9.4 * 10.0)) + 1
    r_ok, r_det = rio_alive_check(
        list(np.linspace(12.2, 22.2, n_half)), [1] * n_half, flight_win
    )
    check("10f rio_alive fails: RIO dies halfway through the flight window", not r_ok, r_det)
    n_ehalf = int(round(44.0 * 10.0)) + 1
    e_ok, e_det = ekf_alive_check(
        list(np.linspace(12.1, 22.1, n_ehalf)),
        [7 + int(k * 10.0 / 44.0) for k in range(n_ehalf)],
        flight_win,
        seq_frac_expected=0.2,
    )
    check("10f2 ekf_alive fails: EKF dies halfway through the flight window", not e_ok, e_det)

    # ------------------------------------------------------------------
    # BUG B3 (2026-09-11): the marker CAPTURE PATH, not just the span helper.
    # The live run printed start=0.0 end=0.0 on a healthy flight because
    # `self._last_odom_sim = stamp` in _on_odom was last-write-wins over a
    # /cf_*/odom stream that interleaves zero-stamped messages (whole-run odom
    # min reads 0.000 on every drone), so whatever arrived last at the mark
    # instant — usually a zero — became the window. flight_window_span() alone
    # could never catch that: these cases drive the REAL recorder callbacks and
    # the REAL mark_flight_start/end, exactly as run_flight does.
    # ------------------------------------------------------------------
    class _FakeStamp:
        def __init__(self, t: float):
            self.sec = int(math.floor(t))
            self.nanosec = int(round((t - math.floor(t)) * 1e9))

    class _FakeHeader:
        def __init__(self, t: float):
            self.stamp = _FakeStamp(t)

    class _FakeVec:
        x = 0.0
        y = 0.0
        z = 0.0
        w = 1.0

    class _FakePose:
        def __init__(self):
            self.position = _FakeVec()
            self.orientation = _FakeVec()

    class _FakePoseWrap:
        def __init__(self):
            self.pose = _FakePose()

    class _FakeOdom:
        """Minimal nav_msgs/Odometry stand-in; omit `t` for an unstamped msg."""

        def __init__(self, t=None):
            self.pose = _FakePoseWrap()
            if t is not None:
                self.header = _FakeHeader(t)

    def _fresh_recorder(n: int = 3):
        rec = EstimateRecorder.__new__(EstimateRecorder)  # no rclpy / no spin
        rec._init_state(n)
        return rec

    rec = _fresh_recorder()
    check("10j capture: marker unset before any odom", not math.isfinite(rec.sim_now()))
    rec.mark_flight_start()
    check(
        "10j2 capture: unmarked clock -> window 0.0 (FAILS, not vacuous)",
        flight_window_span(rec.flight_sim_t0, rec.flight_sim_t1)[0] == 0.0,
    )

    # Healthy replica of the live run: odom advancing 0 -> 111.7 s sim, the
    # scripted path marked around sim 92.2 -> 111.6, and — the regression — a
    # zero-stamped odom message landing immediately before EACH mark.
    rec = _fresh_recorder()
    for t in np.linspace(0.0, 92.2, 400):
        for idx in range(3):
            rec._on_odom(idx, _FakeOdom(float(t)))
    rec._on_odom(1, _FakeOdom(0.0))          # zero-stamped intruder
    rec.mark_flight_start()
    for t in np.linspace(92.2, 111.7, 200):
        for idx in range(3):
            rec._on_odom(idx, _FakeOdom(float(t)))
    rec._on_odom(2, _FakeOdom(0.0))          # zero-stamped intruder
    rec.mark_flight_end()
    t0_cap, t1_cap = rec.flight_sim_t0, rec.flight_sim_t1
    check(
        "10k capture: start marker non-zero and at the fed start stamp",
        t0_cap is not None and math.isfinite(t0_cap) and abs(t0_cap - 92.2) < 0.5,
        f"start={t0_cap}",
    )
    check(
        "10k2 capture: end marker non-zero and at the fed end stamp",
        t1_cap is not None and math.isfinite(t1_cap) and abs(t1_cap - 111.7) < 0.5,
        f"end={t1_cap}",
    )
    fed = [s for i in rec.odom_sim_stamps for s in rec.odom_sim_stamps[i] if s > 0.0]
    check(
        "10k3 capture: markers bracket the stamps fed during the flight",
        t0_cap <= max(fed) and t1_cap >= min(s for s in fed if s >= 92.2) and t1_cap <= max(fed),
    )
    fw_cap, fw_cap_det = flight_window_span(t0_cap, t1_cap)
    check(
        "10k4 capture: window is the ~19.4 s scripted path, not 0.0",
        abs(fw_cap - 19.5) < 1.0 and fw_cap >= SIM_REF_SPAN_FLOOR_S,
        fw_cap_det,
    )
    # The bug reproduced: last-write-wins over the same stream captures 0.0.
    lww = [s for i in rec.odom_sim_stamps for s in rec.odom_sim_stamps[i]][-1]
    check("10k5 capture: last-write-wins would have captured 0.0 (the bug)", lww == 0.0)
    # And the fixed capture path makes RIO/EKF covering that flight clear 0.8×.
    n_rio_c = int(round(9.4 * fw_cap)) + 1
    check(
        "10k6 capture: healthy RIO over the captured window passes",
        rio_alive_check(list(np.linspace(t0_cap, t1_cap, n_rio_c)), [1] * n_rio_c, fw_cap)[0],
    )
    n_ekf_c = int(round(44.0 * fw_cap)) + 1
    check(
        "10k7 capture: healthy EKF over the captured window passes",
        ekf_alive_check(
            list(np.linspace(t0_cap, t1_cap, n_ekf_c)),
            [7 + int(k * 10.0 / 44.0) for k in range(n_ekf_c)],
            fw_cap,
            seq_frac_expected=0.2,
        )[0],
    )
    # A dead sim (odom present but clock frozen at 0) must still FAIL.
    rec = _fresh_recorder()
    for _ in range(200):
        for idx in range(3):
            rec._on_odom(idx, _FakeOdom(0.0))
    rec.mark_flight_start()
    rec.mark_flight_end()
    check(
        "10m capture: frozen sim clock -> window 0.0 -> FAIL",
        flight_window_span(rec.flight_sim_t0, rec.flight_sim_t1)[0] == 0.0
        and _sim_ref_guard(flight_window_span(rec.flight_sim_t0, rec.flight_sim_t1)[0]) is not None,
    )
    # Unstamped odom (no header at all) is counted, never taken as a marker.
    rec = _fresh_recorder()
    for idx in range(3):
        rec._on_odom(idx, _FakeOdom())
    check(
        "10m2 capture: header-less odom counted, marker stays unset",
        all(rec.odom_header_missing[i] == 1 for i in range(3))
        and not math.isfinite(rec.sim_now()),
    )
    # Markers never run backwards when a stale/out-of-order stamp arrives last.
    rec = _fresh_recorder()
    for t in (10.0, 20.0, 30.0, 15.0):
        rec._on_odom(0, _FakeOdom(t))
    rec.mark_flight_end()
    check("10m3 capture: marker is a high-water mark, not the last arrival",
          abs(float(rec.flight_sim_t1) - 30.0) < 1e-6, f"end={rec.flight_sim_t1}")

    # Missing / zero / too-short flight window must FAIL, never pass vacuously.
    check(
        "10g rio_alive fails: flight window unmarked (0 s)",
        not rio_alive_check(rio_l, [1] * n_rio_l, flight_window_span(None, None)[0])[0],
    )
    check(
        "10g2 ekf_alive fails: flight window unmarked (0 s)",
        not ekf_alive_check(ekf_l, seq_l, flight_window_span(None, None)[0], seq_frac_expected=0.2)[0],
    )
    check(
        "10h rio_alive fails: flight window under the 5 s floor",
        not rio_alive_check(rio_l, [1] * n_rio_l, flight_window_span(12.0, 16.0)[0])[0],
    )
    check(
        "10h2 ekf_alive fails: flight window under the 5 s floor",
        not ekf_alive_check(ekf_l, seq_l, flight_window_span(12.0, 16.0)[0], seq_frac_expected=0.2)[0],
    )
    # The 2026-09-11 hollow run (frozen stamps / no rows) fails every liveness
    # check even with a perfectly good flight window.
    check("10i rio_alive fails hollow run vs flight window", not rio_alive_check([0.0] * 900, [1] * 900, flight_win)[0])
    check("10i2 rio_alive fails zero rows vs flight window", not rio_alive_check([], [], flight_win)[0])
    check(
        "10i3 ekf_alive fails hollow run (stamps frozen at 0) vs flight window",
        not ekf_alive_check([0.0] * 3600, list(range(3600)), flight_win, seq_frac_expected=0.2)[0],
    )

    # ------------------------------------------------------------------
    # BUG B2 (2026-09-11): seq_inc_frac 0.22 vs a fixed 0.5 bar. Proof in
    # swarm_loc_node.py: _on_tick publishes the estimate at estimator.rate_hz
    # with the current self._seq, and only _on_broadcast_tick (at
    # comms.broadcast_rate_hz) does `self._seq += 1`. Expected frac is therefore
    # bc_hz/rate_hz = 10/50 = 0.20, not 0.5.
    # ------------------------------------------------------------------
    cfg_live = {"estimator": {"rate_hz": 50}, "comms": {"broadcast_rate_hz": 10}}
    check(
        "11 expected_seq_inc_frac = bc_hz/rate_hz",
        abs(expected_seq_inc_frac(cfg_live) - 0.2) < 1e-12,
        str(expected_seq_inc_frac(cfg_live)),
    )
    check("11a expected_seq_inc_frac clamps to 1.0", expected_seq_inc_frac({"estimator": {"rate_hz": 5}, "comms": {"broadcast_rate_hz": 10}}) == 1.0)
    check("11b expected_seq_inc_frac falls back to 1.0 (strictest) on bad cfg", expected_seq_inc_frac({}) == 1.0)
    check("11b2 expected_seq_inc_frac falls back to 1.0 on zero rate", expected_seq_inc_frac({"estimator": {"rate_hz": 0}, "comms": {"broadcast_rate_hz": 10}}) == 1.0)

    # 50 Hz estimates, seq bumped at 10 Hz → seq_inc_frac ≈ 0.20 (live: 0.22)
    n_s = int(round(50.0 * flight_win)) + 1
    stamps_s = list(np.linspace(12.0, 12.0 + flight_win, n_s))
    seq_s = [3 + int(k * 10.0 / 50.0) for k in range(n_s)]
    obs_frac = float(np.mean(np.diff(np.asarray(seq_s)) > 0))
    check("11c synthetic seq_inc_frac ≈ 0.2 like the live run", abs(obs_frac - 0.2) < 0.02, f"{obs_frac:.3f}")
    # BEFORE (pinned): under the old fixed 0.5 bar this healthy run failed.
    check(
        "11d ekf_alive FAILED before: 10 Hz seq on 50 Hz estimates vs fixed 0.5 (the bug)",
        not ekf_alive_check(stamps_s, seq_s, flight_win, seq_frac_expected=1.0)[0],
    )
    # AFTER: the config-derived expectation passes it.
    e_ok, e_det = ekf_alive_check(
        stamps_s, seq_s, flight_win, seq_frac_expected=expected_seq_inc_frac(cfg_live)
    )
    check("11e ekf_alive PASSES now: seq at the configured broadcast ratio", e_ok, e_det)
    # A frozen seq must still FAIL, at any expectation.
    check(
        "11f ekf_alive fails: seq frozen (hollow run)",
        not ekf_alive_check(stamps_s, [3] * n_s, flight_win, seq_frac_expected=0.2)[0],
    )
    check(
        "11g ekf_alive fails: seq frozen even at tiny expectation",
        not ekf_alive_check(stamps_s, [3] * n_s, flight_win, seq_frac_expected=0.001)[0],
    )
    # Seq must be non-decreasing: a scrambled/rolling-back seq FAILS.
    seq_bad = list(np.tile([5, 4, 6, 3, 7], n_s // 5 + 1))[:n_s]
    check(
        "11h ekf_alive fails: seq decreasing",
        not ekf_alive_check(stamps_s, seq_bad, flight_win, seq_frac_expected=0.2)[0],
    )

    # ------------------------------------------------------------------
    # BUG A (2026-09-11): truth on the WALL clock (~1.7e9 s) vs estimates on
    # SIM time (~0–20 s) → eval_6_1.interp_pose paired 0 rows → ATE/RPE/NEES
    # NaN. Both series now carry odom/EKF header sim time; a frozen/zero
    # estimate stamp FAILS loudly instead of falling back to wall time.
    # ------------------------------------------------------------------
    from eval_6_1 import TRUTH_DTYPE as _TD, paired_errors as _paired

    t_sim = np.linspace(0.0, 20.0, 401)
    tru_sim = np.zeros(t_sim.size, dtype=_TD)
    for k, tk in enumerate(t_sim):
        tru_sim[k]["stamp"] = tk
        tru_sim[k]["p_x"], tru_sim[k]["p_y"], tru_sim[k]["p_z"] = 0.1 * tk, 0.0, 0.5
        tru_sim[k]["psi"] = 0.0
    t_est = np.linspace(0.5, 19.5, 800)
    est_sim = np.zeros(t_est.size, dtype=STATE_DTYPE)
    for k, tk in enumerate(t_est):
        est_sim[k]["stamp"] = tk
        est_sim[k]["p_x"], est_sim[k]["p_y"], est_sim[k]["p_z"] = 0.1 * tk, 0.0, 0.5
        est_sim[k]["psi"] = 0.0
    pe = _paired(est_sim, tru_sim)
    check("12 sim/sim pairs in eval_6_1 (n>0)", pe["n"] == t_est.size, f"n={pe['n']}")
    p_ok, p_det = pairing_clock_check(list(tru_sim["stamp"]), list(est_sim["stamp"]))
    check("12a pairing_clock passes sim/sim", p_ok, p_det)

    # The bug: truth stamped at wall time against sim estimates.
    tru_wall = tru_sim.copy()
    tru_wall["stamp"] = tru_sim["stamp"].astype(np.float64) + 1.7e9
    pe_bad = _paired(est_sim, tru_wall)
    check("12b wall truth vs sim est pairs nothing in eval_6_1", pe_bad["n"] == 0, f"n={pe_bad['n']}")
    check("12b2 that yields NaN ATE (the observed symptom)", not math.isfinite(pe_bad["ate_rmse_m"]))
    p_ok, p_det = pairing_clock_check(list(tru_wall["stamp"]), list(est_sim["stamp"]))
    check("12c pairing_clock FAILS wall-vs-sim — never a silent pair", not p_ok, p_det)
    check("12c2 and says DIFFERENT CLOCKS", "DIFFERENT CLOCKS" in p_det, p_det)

    # Hollow run: EKF stamps frozen/zero. The old code substituted wall time
    # here; now it must FAIL rather than manufacture a plausible ATE.
    p_ok, p_det = pairing_clock_check(list(tru_sim["stamp"]), [0.0] * 900)
    check("12d pairing_clock FAILS frozen/zero estimate stamps", not p_ok, p_det)
    check("12d2 and refuses the wall-clock fallback explicitly", "Refusing" in p_det, p_det)
    p_ok, p_det = pairing_clock_check(list(tru_sim["stamp"]), [12.5] * 900)
    check("12e pairing_clock FAILS frozen (non-zero) estimate stamps", not p_ok, p_det)
    p_ok, p_det = pairing_clock_check([4.0] * 500, list(est_sim["stamp"]))
    check("12f pairing_clock FAILS frozen truth stamps", not p_ok, p_det)
    check("12g pairing_clock FAILS no estimate rows", not pairing_clock_check(list(tru_sim["stamp"]), [])[0])
    check("12h pairing_clock FAILS no truth rows", not pairing_clock_check([], list(est_sim["stamp"]))[0])
    # Partial overlap below the required fraction is also a FAIL.
    check(
        "12i pairing_clock FAILS partial overlap (truth covers 25% of est)",
        not pairing_clock_check(list(np.linspace(0.0, 5.0, 200)), list(est_sim["stamp"]))[0],
    )
    check(
        "12j pairing_clock passes truth wider than est",
        pairing_clock_check(list(np.linspace(-2.0, 25.0, 600)), list(est_sim["stamp"]))[0],
    )

    # Scoring window: drive the real recorder and dump_eval, so the marker ->
    # score_window.json plumbing itself is exercised, not a hand-passed value.
    import tempfile

    from eval_6_1 import load_score_window

    rec = _fresh_recorder(1)
    check("13 score_start None before the path is marked", rec.score_start() is None)
    for t in np.linspace(0.0, 52.9, 50):
        rec._on_odom(0, _FakeOdom(float(t)))
    rec.mark_flight_start()
    s0 = rec.score_start()
    check("13a score_start is the flight-start marker", s0 is not None and abs(s0 - 52.9) < 0.5, str(s0))
    with tempfile.TemporaryDirectory() as td:
        rec.dump_eval(td)
        t0, t1, src = load_score_window(Path(td))
        check("13b dump_eval persists the score window start", t0 is not None and abs(t0 - s0) < 1e-9, f"t0={t0}")
        check("13c score window end left open (landing stays scored)", t1 is None, f"t1={t1}")
        check("13d score window records its source", "flight_sim_t0" in src, src)

    rec = _fresh_recorder(1)
    rec._on_odom(0, _FakeOdom(0.0))
    rec.mark_flight_start()
    with tempfile.TemporaryDirectory() as td:
        rec.dump_eval(td)
        t0, _, _ = load_score_window(Path(td))
        check("13e unset marker persists no start, never a 0.0 window", t0 is None, f"t0={t0}")

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument(
        "--list-scenarios",
        action="store_true",
        help="Print env/situation catalog and exit.",
    )
    parser.add_argument(
        "--scenario",
        default="",
        help="env/situation (e.g. tunnel/triangle_forward). Sets world, n, "
        "spacing, motion, and default eval/log dirs under out/swarm_loc_eval/.",
    )
    parser.add_argument("--config", default="configs/estimation/swarm_loc.yaml")
    parser.add_argument("--gains", default="configs/airframe/pid_gains_loaded.yaml")
    parser.add_argument("--world", default="phase0_tunnel_gate")
    parser.add_argument("--model-prefix", default="crazyflie")
    parser.add_argument("--num-drones", type=int, default=3)
    parser.add_argument("--spacing", type=float, default=1.5)
    parser.add_argument("--hover-height", type=float, default=0.5)
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Flight/record seconds. Default 300, or the scenario's duration if --scenario is set.",
    )
    parser.add_argument("--connect-wait", type=float, default=90.0)
    parser.add_argument("--connect-timeout", type=float, default=90.0)
    parser.add_argument(
        "--no-fly",
        action="store_true",
        help="Skip cflib (drones stay on the ground). Still checks estimate rate / no-truth / diverge.",
    )
    parser.add_argument("--no-mlflow", action="store_true")
    parser.add_argument(
        "--eval-dir",
        default="",
        help="Write truth.npz + estimates.npz (odom subscribed here only) and run §6.1 metrics. "
        "Default with --scenario: out/swarm_loc_eval/<env>/<situation>/",
    )
    parser.add_argument(
        "--logs",
        default="",
        help="Measurement log dir from phase0_gate.sh --swarm-loc-log-dir (UWB mix / hops / CPU).",
    )
    args = parser.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    if args.list_scenarios:
        from swarm_loc_scenarios import SCENARIOS, eval_dir_for

        print("env/situation                  world                 n  layout    motion")
        for key, spec in SCENARIOS.items():
            print(
                f"  {key:<30} {spec['world']:<20} {spec['num_drones']}  "
                f"{spec['layout']:<9} {spec['motion']}"
            )
            print(f"      {spec['why']}")
            print(f"      eval → {eval_dir_for(spec)}")
        sys.exit(0)

    spec = None
    if args.scenario.strip():
        try:
            spec = get_scenario(args.scenario)
        except KeyError as e:
            print(f"[swarm_loc_gate] {e}", file=sys.stderr)
            sys.exit(2)
        args._scenario = spec
        args._spawn_xy = spawn_xy(spec)
        args.world = spec["world"]
        args.num_drones = int(spec["num_drones"])
        args.spacing = float(spec["spacing"])
        if args.duration is None:
            args.duration = float(spec["duration"])
        if not args.eval_dir.strip():
            args.eval_dir = eval_dir_for(spec)
        if not args.logs.strip():
            args.logs = log_dir_for(spec)
        print(
            f"[swarm_loc_gate] scenario {spec['key']}  world={args.world}  "
            f"n={args.num_drones}  eval-dir={args.eval_dir}",
            flush=True,
        )
    else:
        args._scenario = None
        args._spawn_xy = None
        if args.duration is None:
            args.duration = 300.0

    cfg_path = args.config
    if not os.path.isabs(cfg_path):
        cfg_path = os.path.join(_REPO_ROOT, cfg_path)
    cfg = load_config(cfg_path)
    target_hz = float(cfg["estimator"]["rate_hz"])
    n = int(args.num_drones)

    flight_fail = False
    fly_fallback = False
    explicit_no_fly = bool(args.no_fly)
    recorder = None
    try:
        if args.no_fly:
            recorder = EstimateRecorder(n)
            print(f"[swarm_loc_gate] --no-fly: recording estimates for {args.duration:.0f} s …")
            recorder.wait_truth(45.0)
            recorder.mark_flight_start()
            time.sleep(float(args.duration))
            recorder.mark_flight_end()
        else:
            scfs = connect_drones(args)
            if scfs is None:
                fly_fallback = True
                recorder = EstimateRecorder(n)
                print(
                    "[swarm_loc_gate] radios did not connect — recording estimates on the ground. "
                    "flight check will FAIL. Kill sim leftovers and rerun for a real fly.",
                    flush=True,
                )
                recorder.wait_truth(45.0)
                recorder.mark_flight_start()
                time.sleep(float(args.duration))
                recorder.mark_flight_end()
            else:
                recorder = EstimateRecorder(n)
                if not recorder.wait_truth(45.0):
                    print(
                        "[swarm_loc_gate] FAIL: Gazebo truth not on ROS. "
                        "Not flying (ATE would be empty).",
                        flush=True,
                    )
                    flight_fail = True
                else:
                    flight_fail = run_flight(args, scfs, recorder=recorder)
        no_truth, truth_hits, all_subs = inspect_estimator_subs(n)
    finally:
        if recorder is not None:
            recorder.shutdown()

    checks = {}
    details: Dict[str, str] = {}
    rates = {}
    finite_ok = True
    diverged = False
    for i in range(n):
        rec = recorder.rows[i]
        stamps = [t for t, _ in rec]
        t0 = stamps[0] if stamps else 0.0
        t1 = stamps[-1] if stamps else 0.0
        ok_r, hz = _rate_ok(stamps, target_hz, t1 - t0)
        rates[i] = hz
        checks[f"rate_hz_cf_{i}"] = ok_r
        for _, row in rec:
            p = [float(row["p_x"]), float(row["p_y"]), float(row["p_z"])]
            if not all(math.isfinite(v) for v in p):
                finite_ok = False
            if int(row["status"]) != 0:
                diverged = True
        checks[f"samples_cf_{i}"] = len(rec) > 10
    checks["finite"] = finite_ok
    checks["non_diverged"] = (not diverged) and (not flight_fail)
    checks["no_truth_subs"] = no_truth
    if explicit_no_fly:
        checks["flight"] = True
    else:
        checks["flight"] = (not flight_fail) and (not fly_fallback)

    # ---- hollow-run tripwires: RIO / EKF liveness from the recorder ----
    duration_s = float(args.duration)  # WALL seconds — used only by rate/floor
                                       # checks that count wall-clock arrivals.
    # SIM-time reference for the liveness checks. duration_s is wall time and
    # must never be compared against sim-time stamps (see UNITS).
    odom_span, odom_det = sim_reference_span(recorder.odom_sim_stamps)
    # BUG B: the reference is the FLIGHT window, not the whole odom span (which
    # also covers wait-for-truth + pre-arm hover and made the bar unpassable).
    sim_ref_span, flight_det = flight_window_span(
        recorder.flight_sim_t0, recorder.flight_sim_t1
    )
    print(f"[swarm_loc_gate] {flight_det}; context: {odom_det} "
          f"(wall duration={duration_s:.1f}s, "
          f"RTF≈{(odom_span / duration_s if duration_s > 0 else 0.0):.2f})")
    seq_frac_exp = expected_seq_inc_frac(cfg)
    print(
        f"[swarm_loc_gate] expected seq_inc_frac = broadcast_rate_hz/rate_hz = "
        f"{seq_frac_exp:.2f} (seq is bumped only on the broadcast tick)"
    )
    for i in range(n):
        rm = recorder.rio_meta[i]
        ok_c, det = rio_alive_check([s for _, s, _ in rm], [v for _, _, v in rm], sim_ref_span)
        checks[f"rio_alive_cf_{i}"] = ok_c
        details[f"rio_alive_cf_{i}"] = det
        em = recorder.est_meta[i]
        ok_c, det = ekf_alive_check(
            [s for _, s, _ in em],
            [q for _, _, q in em],
            sim_ref_span,
            seq_frac_expected=seq_frac_exp,
        )
        checks[f"ekf_alive_cf_{i}"] = ok_c
        details[f"ekf_alive_cf_{i}"] = det
        # BUG A: truth and estimates must be on ONE clock or ATE/RPE/NEES are NaN.
        ok_c, det = pairing_clock_check(
            recorder.truth_sim_stamps(i), [float(r["stamp"]) for _, r in recorder.rows[i]]
        )
        if recorder.odom_header_missing.get(i, 0):
            ok_c = False
            det += (
                f"; {recorder.odom_header_missing[i]} /cf_{i}/odom messages had no "
                "header stamp — truth sim time is not trustworthy"
            )
        checks[f"pairing_clock_cf_{i}"] = ok_c
        details[f"pairing_clock_cf_{i}"] = det

    eval_dir = args.eval_dir.strip()
    report = None
    if eval_dir and recorder is not None:
        out = Path(eval_dir)
        recorder.dump_eval(out)
        n_truth = sum(len(recorder.truth[i]) for i in range(n))
        spec = getattr(args, "_scenario", None)
        if spec is not None:
            meta = {k: spec[k] for k in spec if k != "key"}
            meta["key"] = spec["key"]
            with (out / "scenario.json").open("w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2)
        print(f"[swarm_loc_gate] wrote {out / 'truth.npz'} and estimates.npz (odom samples={n_truth})")
        logs = args.logs.strip() or (str(out) if list(out.glob("cf_*.npz")) else "")
        try:
            from eval_6_1 import evaluate, load_run, load_structured_npz, print_report, TRUTH_DTYPE

            run = load_run(logs) if logs and Path(logs).exists() else None
            truth = load_structured_npz(out / "truth.npz", TRUTH_DTYPE)
            estimates = load_structured_npz(out / "estimates.npz")
            report = evaluate(
                run,
                truth,
                estimates,
                out_dir=out,
                score_t0=recorder.score_start(),
                score_source="flight_sim_t0 (start of scripted path)",
            )
            print_report(report)
        except Exception as e:
            import traceback

            traceback.print_exc()
            print(f"[swarm_loc_gate] eval_6_1 skipped: {e}", file=sys.stderr)

    # ---- hollow-run tripwires: measurement-log checks ----
    # These apply whenever a log/eval location is configured (every scored
    # scenario run sets both). A run without them cannot be scored.
    log_dir = args.logs.strip() or eval_dir
    if log_dir:
        ok_c, det, run_logs = logs_intact_check(log_dir, n)
        checks["logs_intact"] = ok_c
        details["logs_intact"] = det
        ok_c, det = uwb_consumed_check(run_logs, n, duration_s)
        checks["uwb_consumed"] = ok_c
        details["uwb_consumed"] = det
        hops = (report or {}).get("hops")
        mix = (report or {}).get("mix")
        if run_logs and (not hops or not mix):
            try:
                from eval_6_1 import hops_from_uwb, uwb_mix

                hops = hops or hops_from_uwb(run_logs)
                mix = mix or uwb_mix(run_logs)
            except Exception as e:
                print(f"[swarm_loc_gate] hops/mix fallback failed: {e}", file=sys.stderr)
        ok_c, det = hops_valid_check(hops or {}, n)
        checks["hops_valid"] = ok_c
        details["hops_valid"] = det
        ok_c, det = mix_nonzero_check(mix or {})
        checks["mix_nonzero"] = ok_c
        details["mix_nonzero"] = det
        if eval_dir:
            ok_c, det = nees_sane_check((report or {}).get("per_drone") or {})
            checks["nees_sane"] = ok_c
            details["nees_sane"] = det
            # BUG A symptom guard: eval_6_1 must have actually paired rows.
            per = (report or {}).get("per_drone") or {}
            empty = [k for k, m in per.items() if int(m.get("n", 0)) <= 0]
            checks["ate_paired"] = bool(per) and not empty
            if per:
                counts = ", ".join(f"cf_{k}:{int(m.get('n', 0))}" for k, m in sorted(per.items()))
                det = f"paired samples {{{counts}}}"
                if empty:
                    det += (
                        f"; drones {empty} paired 0 rows — truth and estimate "
                        "stamps are on different clocks, ATE/RPE/NEES are NaN"
                    )
                details["ate_paired"] = det
            else:
                details["ate_paired"] = "no per-drone metrics (eval_6_1 produced nothing)"
    else:
        print(
            "[swarm_loc_gate] WARN: no --logs/--eval-dir configured — "
            "measurement-log hollow-run checks skipped; this run is NOT scoreable.",
            file=sys.stderr,
        )

    gate_pass = all(checks.values())

    print("\n[swarm_loc_gate] results:")
    for i in range(n):
        print(f"  cf_{i} estimate_hz={rates.get(i, 0):.2f}  n={len(recorder.rows[i])}")
        print(f"  cf_{i} n_subs={len(all_subs.get(i, []))} truth_subs={truth_hits.get(i, [])}")
    for k, v in checks.items():
        line = f"  {'PASS' if v else 'FAIL'} {k}"
        if k in details and (not v or k.startswith(("rio_alive", "ekf_alive", "pairing_clock"))):
            line += f" — {details[k]}"
        print(line)
    print(f"\n[swarm_loc_gate] {'PASS' if gate_pass else 'FAIL'}")

    if not args.no_mlflow:
        try:
            import mlflow

            mlflow.set_tracking_uri("sqlite:///mlflow.db")
            mlflow.set_experiment(cfg.get("mlflow_experiment", "phase2_swarm_loc"))
            with mlflow.start_run(run_name="swarm_loc_gate"):
                mlflow.log_param("num_drones", n)
                mlflow.log_param("duration_s", args.duration)
                mlflow.log_param("rate_hz", target_hz)
                for i, hz in rates.items():
                    mlflow.log_metric(f"estimate_hz_cf_{i}", hz)
                mlflow.log_metric("gate_pass", int(gate_pass))
        except Exception as e:
            print(f"[swarm_loc_gate] MLflow skipped: {e}", file=sys.stderr)

    sys.exit(0 if gate_pass else 1)


if __name__ == "__main__":
    main()
