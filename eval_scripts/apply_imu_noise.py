#!/usr/bin/env python3
"""apply_imu_noise.py — inject an Allan-variance-derived noise model into the
Gazebo IMU sensor (`<sensor type="imu">`) of a generated Crazyflie SDF, per
configs/sensors/imu_noise.yaml (which points at the calibration output
configs/sensors/imu_calib_results.yaml).

This is SIMULATOR-side sensor corruption: the gz IMU readings themselves become
noisy. The same gz IMU feeds the Crazyflie firmware through the CrazySim
bridge (crazysim_plugin.cpp ImuCallback), so the flight controller sees the
noise too. perception/radar_processing/RIO.py reads only orientation and
linear_acceleration, so gyro noise reaches the firmware only, not RIO.

Like apply_payload.py, apply_tof_sensor.py and the radar-plugin injection,
this is a launch-time post-processing step on the temporary generated SDF
(e.g. /tmp/<model>_<id>.sdf) — it never touches the firmware_mods/CrazySim
submodule. Called from phase0_gate.sh right after apply_tof_sensor.py.

Conversions (derived in comments below, reproduced by --selftest):
  white noise   stddev = N * sqrt(f)      N = random_walk_coeff [unit/sqrt(Hz)],
                                           f = the sensor's own <update_rate>
  bias instab.  first-order Gauss-Markov: steady-state stddev q, corr. time T
                  q = ADEV_min / GM_ADEV_PEAK_OVER_SIGMA
                  T = tau_min  / GM_TAU_PEAK_OVER_T
                gz <dynamic_bias_stddev> is the DRIVING density sigma_b, with
                steady-state stddev sigma_b*sqrt(T/2) -> sigma_b = q*sqrt(2/T)
  rate RW       skipped (gz has no unbounded random-walk bias term)
  turn-on bias  bias_mean = bias_stddev = 0 (not observable by Allan analysis)

Usage:
    apply_imu_noise.py SDF_PATH [--config configs/sensors/imu_noise.yaml]
                                [--cf-id 0]
    apply_imu_noise.py --selftest
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

SENSOR_TYPE = "imu"
TAG = "[apply_imu_noise]"

# calibration group -> sdformat <imu> child. gz-sensors8 exposes noise ONLY on
# these six channels; orientation cannot be noised (only <enable_orientation>,
# which this script never touches).
GROUPS = (("gyro", "angular_velocity"), ("accel", "linear_acceleration"))
AXES = ("x", "y", "z")

CONVENTIONS = ("allan_minimum", "ieee_952_B")

# IEEE Std 952 flicker (bias-instability) noise: the Allan-deviation plateau of
# flicker noise with coefficient B is B*sqrt(2*ln2/pi) (= 0.6643). So if the
# file's bias_instability is the coefficient B, the ADEV minimum is B * this.
IEEE952_FLICKER_FACTOR = math.sqrt(2.0 * math.log(2.0) / math.pi)

# First-order Gauss-Markov process (steady-state stddev q, correlation time T):
# its Allan deviation rises then falls, peaking at tau_peak with value
# adev_peak. These are the fitted ratios tau_peak/T and adev_peak/q. They are
# NOT remembered textbook constants: gm_allan_peak_numeric() derives them
# deterministically from the exact GM autocovariance, and --selftest both
# reproduces them to 0.5% and cross-checks them against a seeded simulation of
# gz-sensors' own discrete bias update.
GM_TAU_PEAK_OVER_T = 1.8926
GM_ADEV_PEAK_OVER_SIGMA = 0.61736
# (Beware: the often-quoted "0.437" belongs to a different parameterisation of
# the GM strength; with q = steady-state stddev the fitted value is 0.617.)

# sdformat 1.11 names written by this script (checked against the installed
# schema by --selftest when /usr/share/sdformat14/1.11 is present).
NOISE_CHILDREN_WRITTEN = ("mean", "stddev", "bias_mean", "bias_stddev",
                          "dynamic_bias_stddev", "dynamic_bias_correlation_time")
SCHEMA_DIRS = ("/usr/share/sdformat14/1.11",)


def load_config(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _repo_root():
    return os.environ.get("SAR_NANO_SWARM_ROOT") or os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))


def resolve_path(path):
    return path if os.path.isabs(path) else os.path.join(_repo_root(), path)


def _usable(v):
    """True for a finite, strictly positive number (never write NaN/inf/<=0)."""
    return (isinstance(v, (int, float)) and not isinstance(v, bool)
            and math.isfinite(v) and v > 0)


def _fmt(v):
    return format(float(v), ".10g")


# ── Gauss-Markov <-> Allan deviation ─────────────────────────────────────────
def gm_allan_peak_numeric(samples_per_T=1000, max_tau_over_T=5.0):
    """Exact Allan variance of a sampled unit-variance first-order GM process,
    from its autocovariance R(k) = phi^k, phi = exp(-dt/T), T = 1.

    For averaging windows of m samples: AVAR(m) = Var(ybar) - Cov(ybar1, ybar2)
      m^2 Var  = S(m) = m + 2*sum_{k=1}^{m-1} (m-k) phi^k
      m^2 Cov  = C(m) = sum_{d=1}^{m} d phi^d + sum_{d=m+1}^{2m-1} (2m-d) phi^d
    evaluated with prefix sums P0[n] = sum_{k<=n} phi^k, P1[n] = sum_{k<=n} k phi^k.
    Returns (tau_peak/T, adev_peak/q). Deterministic; discretisation error
    O(dt/T) = 0.1% at the default resolution.
    """
    import numpy as np
    phi = math.exp(-1.0 / samples_per_T)
    m_max = int(max_tau_over_T * samples_per_T)
    k = np.arange(2 * m_max + 1, dtype=float)
    pk = phi ** k
    pk[0] = 0.0
    P0 = np.cumsum(pk)
    P1 = np.cumsum(k * pk)
    m = np.arange(1, m_max + 1)
    S = m + 2.0 * (m * P0[m - 1] - P1[m - 1])
    C = P1[m] + 2.0 * m * (P0[2 * m - 1] - P0[m]) - (P1[2 * m - 1] - P1[m])
    avar = (S - C) / m.astype(float) ** 2
    i = int(np.argmax(avar))
    # parabolic refinement in ln(tau) around the discrete maximum
    if 0 < i < len(avar) - 1:
        x = np.log(m[i - 1:i + 2].astype(float))
        a, b, _ = np.polyfit(x, avar[i - 1:i + 2], 2)
        m_peak = math.exp(-b / (2 * a))
    else:
        m_peak = float(m[i])
    return m_peak / samples_per_T, math.sqrt(float(avar[i]))


def gm_from_allan(adev_min, tau_min_s, convention):
    """Map a bias-instability reading to (steady-state stddev q, corr. time T)."""
    if convention == "allan_minimum":
        peak = adev_min
    elif convention == "ieee_952_B":
        peak = adev_min * IEEE952_FLICKER_FACTOR   # B -> ADEV plateau value
    else:
        raise ValueError(f"unknown bias_instability_convention {convention!r}")
    return peak / GM_ADEV_PEAK_OVER_SIGMA, tau_min_s / GM_TAU_PEAK_OVER_T


def gz_dynamic_bias_stddev(q, T):
    """gz-sensors8 GaussianNoiseModel::ApplyImpl updates the bias as
         b <- exp(-dt/T) b + N(0, sqrt(-sigma_b^2 * T/2 * expm1(-2 dt/T)))
    whose steady-state variance is sigma_b^2 * T/2. So <dynamic_bias_stddev>
    is a driving density, and a GM with steady-state stddev q needs
    sigma_b = q * sqrt(2/T)."""
    return q * math.sqrt(2.0 / T)


# ── parameter computation ────────────────────────────────────────────────────
def compute_noise_params(calib, cfg, update_rate_hz):
    """Return (params, notes). params[sdf_group][axis] = {name: value};
    notes = list of human-readable skip/warning strings."""
    terms = cfg.get("terms", {})
    use_white = bool(terms.get("white_noise", True))
    use_dyn = bool(terms.get("dynamic_bias", True))
    convention = cfg.get("bias_instability_convention", "allan_minimum")
    if convention not in CONVENTIONS:
        raise ValueError(f"bias_instability_convention must be one of {CONVENTIONS}, got {convention!r}")
    if not _usable(update_rate_hz):
        raise ValueError(f"update rate must be a finite positive number, got {update_rate_hz!r}")

    params, notes = {}, []
    for cal_group, sdf_group in GROUPS:
        per_axis = (calib.get(cal_group) or {}).get("per_axis") or {}
        params[sdf_group] = {}
        for axis in AXES:
            a = per_axis.get(axis) or {}
            p = {}
            label = f"{cal_group}.{axis}"

            if use_white:
                n = a.get("random_walk_coeff")
                if _usable(n):
                    # Continuous white noise with density N sampled at f Hz
                    # gives per-sample variance N^2 * f.
                    p["stddev"] = n * math.sqrt(update_rate_hz)
                else:
                    notes.append(f"WARNING: {label} random_walk_coeff={n!r} not finite/positive — white noise skipped")

            if use_dyn:
                bi, tau = a.get("bias_instability"), a.get("bias_instability_tau_s")
                if _usable(bi) and _usable(tau):
                    q, T = gm_from_allan(bi, tau, convention)
                    p["dynamic_bias_stddev"] = gz_dynamic_bias_stddev(q, T)
                    p["dynamic_bias_correlation_time"] = T
                    p["_gm_steady_stddev"] = q
                else:
                    notes.append(f"WARNING: {label} bias_instability={bi!r} tau={tau!r} not finite/positive — dynamic bias skipped")

            rrw = a.get("rate_random_walk_coeff")
            notes.append(f"skipped: {label} rate_random_walk_coeff={rrw!r} — gz has no unbounded random-walk bias term"
                         + ("" if _usable(rrw) else " (value not finite/positive anyway)"))
            params[sdf_group][axis] = p
    return params, notes


def convention_floor_notes(calib, convention):
    """Diagnostic: the ADEV minimum can never lie below the white-noise
    contribution N/sqrt(tau) at the same tau. Report axes where the chosen
    convention implies it does (a sign the convention is wrong for this file)."""
    out = []
    for cal_group, _ in GROUPS:
        per_axis = (calib.get(cal_group) or {}).get("per_axis") or {}
        for axis in AXES:
            a = per_axis.get(axis) or {}
            n, bi, tau = a.get("random_walk_coeff"), a.get("bias_instability"), a.get("bias_instability_tau_s")
            if not (_usable(n) and _usable(bi) and _usable(tau)):
                continue
            adev_min = bi if convention == "allan_minimum" else bi * IEEE952_FLICKER_FACTOR
            floor = n / math.sqrt(tau)
            if adev_min < floor:
                out.append(f"{cal_group}.{axis}: implied ADEV min {adev_min:.3g} < white floor N/sqrt(tau) {floor:.3g}")
    return out


# ── SDF editing ──────────────────────────────────────────────────────────────
def find_imu_sensor(root, sdf_path):
    model = root.find("model")
    if model is None:
        print(f"{TAG} ERROR: no <model> element in {sdf_path}", file=sys.stderr)
        sys.exit(1)
    sensors = [s for s in model.iter("sensor") if s.get("type") == SENSOR_TYPE]
    if len(sensors) != 1:
        print(f"{TAG} ERROR: expected exactly one <sensor type=\"imu\"> in <model> of {sdf_path}, found {len(sensors)}",
              file=sys.stderr)
        sys.exit(1)
    return sensors[0]


def read_update_rate(sensor, sdf_path):
    el = sensor.find("update_rate")
    try:
        rate = float(el.text) if el is not None else None
    except (TypeError, ValueError):
        rate = None
    if not _usable(rate):
        print(f"{TAG} ERROR: IMU sensor '{sensor.get('name')}' in {sdf_path} has no valid <update_rate>", file=sys.stderr)
        sys.exit(1)
    return rate


def _child(parent, tag):
    el = parent.find(tag)
    return el if el is not None else ET.SubElement(parent, tag)


def write_noise(sensor, params):
    """Idempotent: reuse <imu>/<group>/<axis>, replace any existing <noise>.
    Other <imu> children (e.g. enable_orientation) are left untouched."""
    imu = _child(sensor, "imu")
    for _, sdf_group in GROUPS:
        group = _child(imu, sdf_group)
        for axis in AXES:
            ax = _child(group, axis)
            for old in ax.findall("noise"):
                ax.remove(old)
            p = params[sdf_group][axis]
            if "stddev" not in p and "dynamic_bias_stddev" not in p:
                continue
            # For <imu> axes sdformat 1.11 wants `type` as an ATTRIBUTE
            # (noise.sdf: <attribute name="type">), same as the barometer in
            # model.sdf.jinja — unlike the <lidar><noise> pitfall noted in
            # apply_tof_sensor.py.
            noise = ET.SubElement(ax, "noise")
            noise.set("type", "gaussian")
            ET.SubElement(noise, "mean").text = "0"
            ET.SubElement(noise, "stddev").text = _fmt(p.get("stddev", 0.0))
            # Turn-on bias: Allan analysis cannot observe an absolute bias and
            # the calibration file has none -> 0.
            ET.SubElement(noise, "bias_mean").text = "0"
            ET.SubElement(noise, "bias_stddev").text = "0"
            if "dynamic_bias_stddev" in p:
                ET.SubElement(noise, "dynamic_bias_stddev").text = _fmt(p["dynamic_bias_stddev"])
                ET.SubElement(noise, "dynamic_bias_correlation_time").text = _fmt(p["dynamic_bias_correlation_time"])
            # <precision> intentionally omitted (no source for one).


def inject(sdf_path, cfg, calib):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    sensor = find_imu_sensor(root, sdf_path)
    rate = read_update_rate(sensor, sdf_path)
    params, notes = compute_noise_params(calib, cfg, rate)
    write_noise(sensor, params)
    tree.write(sdf_path, encoding="unicode")
    return sensor.get("name"), rate, params, notes


# ── selftest ─────────────────────────────────────────────────────────────────
_TEST_SDF = """<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="crazyflie">
    <link name="base_link">
      <sensor name="air_pressure" type="air_pressure"><update_rate>50</update_rate></sensor>
      {sensors}
    </link>
  </model>
</sdf>
"""
_IMU = '<sensor name="imu_sensor" type="imu"><topic>/cf_0/imu</topic><always_on>1</always_on><update_rate>{rate}</update_rate>{extra}</sensor>'


def _simulate_gz_gm_adev(q, T, dt, n, seed):
    """Seeded simulation of gz-sensors' exact discrete bias update, then its
    overlapping Allan deviation. Returns (sample_std, tau_peak, adev_peak)."""
    import numpy as np
    rng = np.random.default_rng(seed)
    sigma_b = gz_dynamic_bias_stddev(q, T)
    sd = math.sqrt(-sigma_b * sigma_b * T / 2.0 * math.expm1(-2.0 * dt / T))
    phi = math.exp(-dt / T)
    x = np.empty(n)
    state = rng.normal(0.0, q)
    blk = 500                                   # phi^-blk stays ~e^5: no precision loss
    j = np.arange(blk)
    for s in range(0, n, blk):
        L = min(blk, n - s)
        w = rng.normal(0.0, sd, L)
        pw = phi ** j[:L]
        # x_k = phi^(k+1) state + sum_{i<=k} phi^(k-i) w_i
        x[s:s + L] = phi * pw * state + pw * np.cumsum(w / pw)
        state = x[s + L - 1]
    theta = np.concatenate(([0.0], np.cumsum(x) * dt))
    ms = np.unique(np.round(np.logspace(math.log10(0.3 * T / dt), math.log10(8 * T / dt), 120)).astype(int))
    adev = []
    for m in ms:
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        adev.append(math.sqrt(np.mean(d * d) / (2.0 * (m * dt) ** 2)))
    adev = np.array(adev)
    tau = ms * dt
    i = int(np.argmax(adev))
    sel = np.abs(np.log10(tau / tau[i])) < 0.35    # quadratic fit in log-tau over the broad peak
    a, b, c = np.polyfit(np.log(tau[sel]), adev[sel], 2)
    lt = -b / (2 * a)
    return float(np.std(x)), math.exp(lt), float(a * lt * lt + b * lt + c)


def run_selftest():
    import tempfile
    ok = True
    n_pass = 0
    n_fail = 0

    def check(name, cond, detail=""):
        nonlocal ok, n_pass, n_fail
        if cond:
            n_pass += 1
            print(f"[selftest] PASS {name}")
        else:
            ok = False
            n_fail += 1
            print(f"[selftest] FAIL {name}" + (f": {detail}" if detail else ""))

    def rel(a, b):
        return abs(a - b) / max(abs(b), 1e-300)

    cfg_path = resolve_path("configs/sensors/imu_noise.yaml")
    cfg = load_config(cfg_path)
    calib = load_config(resolve_path(cfg["calib_results_path"]))
    td = tempfile.mkdtemp(prefix="imu_noise_selftest_")

    def make_sdf(name, rate=1000, n_imu=1, extra=""):
        p = os.path.join(td, name)
        with open(p, "w") as f:
            f.write(_TEST_SDF.format(sensors="".join(_IMU.format(rate=rate, extra=extra) for _ in range(n_imu))))
        return p

    def noise_of(path, group, axis):
        s = [e for e in ET.parse(path).getroot().iter("sensor") if e.get("type") == "imu"][0]
        return s.find(f"imu/{group}/{axis}/noise")

    def val(noise, tag):
        el = noise.find(tag)
        return None if el is None else float(el.text)

    cfg_min = dict(cfg, bias_instability_convention="allan_minimum")

    # 1. white noise sigma = N * sqrt(1000), every axis, gyro and accel
    p1 = make_sdf("rate1000.sdf")
    _, rate, _, _ = inject(p1, cfg_min, calib)
    check("1a update rate read = 1000", rate == 1000.0, f"rate={rate}")
    for cal_group, sdf_group in GROUPS:
        for axis in AXES:
            n = calib[cal_group]["per_axis"][axis]["random_walk_coeff"]
            got = val(noise_of(p1, sdf_group, axis), "stddev")
            check(f"1b {cal_group}.{axis} stddev = N*sqrt(1000)", got is not None and rel(got, n * math.sqrt(1000)) < 1e-8,
                  f"got={got} want={n * math.sqrt(1000)}")

    # 2. wrong-rate pin: calibration's 100 Hz would under-noise by sqrt(10)
    n_gx = calib["gyro"]["per_axis"]["x"]["random_walk_coeff"]
    got = val(noise_of(p1, "angular_velocity", "x"), "stddev")
    check("2 1000Hz/100Hz stddev ratio = sqrt(10)", rel(got / (n_gx * math.sqrt(100.0)), math.sqrt(10.0)) < 1e-8,
          f"ratio={got / (n_gx * math.sqrt(100.0))}")

    # 3. rate comes from the SDF, not a constant
    p3 = make_sdf("rate250.sdf", rate=250)
    _, rate3, _, _ = inject(p3, cfg_min, calib)
    got3 = val(noise_of(p3, "linear_acceleration", "z"), "stddev")
    n_az = calib["accel"]["per_axis"]["z"]["random_walk_coeff"]
    check("3 update_rate 250 in SDF -> stddev = N*sqrt(250)", rate3 == 250.0 and rel(got3, n_az * math.sqrt(250)) < 1e-8,
          f"rate={rate3} got={got3}")

    # 4. NaN handling: gyro RRW is NaN (skipped + reported); forced NaN/neg terms never written
    _, _, _, notes = inject(make_sdf("notes.sdf"), cfg_min, calib)
    check("4a NaN gyro RRW reported as skipped",
          all(any(n.startswith(f"skipped: gyro.{a} rate_random_walk_coeff") for n in notes) for a in AXES))
    bad = {"gyro": {"per_axis": {"x": {"random_walk_coeff": float("nan"), "bias_instability": -1.0,
                                       "bias_instability_tau_s": 1.0, "rate_random_walk_coeff": float("nan")},
                                 "y": {"random_walk_coeff": float("inf"), "bias_instability": 1e-4,
                                       "bias_instability_tau_s": float("nan")},
                                 "z": {"random_walk_coeff": 1e-4, "bias_instability": 1e-4, "bias_instability_tau_s": 2.0}}},
           "accel": calib["accel"]}
    p4 = make_sdf("bad.sdf")
    _, _, _, notes4 = inject(p4, cfg_min, bad)
    txt = open(p4).read()
    nums = []
    for e in ET.parse(p4).getroot().iter():
        if e.tag in NOISE_CHILDREN_WRITTEN:
            nums.append(float(e.text))
    check("4b no nan/inf text in SDF", "nan" not in txt.lower() and "inf" not in txt.lower())
    check("4c all written noise numbers finite and >= 0", all(math.isfinite(v) and v >= 0 for v in nums), f"{nums}")
    check("4d fully-invalid gyro.x/y axes get no <noise>",
          noise_of(p4, "angular_velocity", "x") is None and noise_of(p4, "angular_velocity", "y") is None)
    check("4e invalid terms produce WARNING notes", sum(n.startswith("WARNING") for n in notes4) == 4, f"{notes4}")

    # 5. idempotency (also enable_orientation preserved)
    p5 = make_sdf("idem.sdf", extra="<imu><enable_orientation>0</enable_orientation></imu>")
    inject(p5, cfg_min, calib)
    first = ET.tostring(ET.parse(p5).getroot().find(".//sensor[@type='imu']/imu"), encoding="unicode")
    inject(p5, cfg_min, calib)
    s5 = ET.parse(p5).getroot().find(".//sensor[@type='imu']")
    second = ET.tostring(s5.find("imu"), encoding="unicode")
    check("5a twice -> exactly one <imu>", len(s5.findall("imu")) == 1)
    check("5b twice -> one <noise> per axis",
          all(len(s5.findall(f"imu/{g}/{a}/noise")) == 1 for _, g in GROUPS for a in AXES))
    check("5c twice -> identical values", first == second)
    check("5d enable_orientation untouched", s5.findtext("imu/enable_orientation") == "0")

    # 6. missing / duplicate IMU -> exit 1
    for label, n_imu in (("missing", 0), ("duplicate", 2)):
        code = None
        try:
            inject(make_sdf(f"{label}.sdf", n_imu=n_imu), cfg_min, calib)
        except SystemExit as e:
            code = e.code
        check(f"6 {label} imu sensor -> exit 1", code == 1, f"code={code}")

    # 7. Gauss-Markov <-> Allan relation, numerically
    tau_r, adev_r = gm_allan_peak_numeric()
    print(f"[selftest] GM exact-autocovariance fit: tau_peak/T={tau_r:.5f} adev_peak/q={adev_r:.5f}")
    check("7a committed GM_TAU_PEAK_OVER_T reproduced (0.5%)", rel(GM_TAU_PEAK_OVER_T, tau_r) < 5e-3, f"{tau_r}")
    check("7b committed GM_ADEV_PEAK_OVER_SIGMA reproduced (0.5%)", rel(GM_ADEV_PEAK_OVER_SIGMA, adev_r) < 5e-3, f"{adev_r}")
    tau_r2, adev_r2 = gm_allan_peak_numeric(samples_per_T=2000, max_tau_over_T=4.0)
    check("7c fit converged in resolution", rel(tau_r2, tau_r) < 2e-3 and rel(adev_r2, adev_r) < 2e-3,
          f"{tau_r2} {adev_r2}")
    q_s, T_s = 2.0, 0.5
    std_s, tau_s, adev_s = _simulate_gz_gm_adev(q_s, T_s, dt=T_s / 100.0, n=2_000_000, seed=7)
    print(f"[selftest] seeded gz-update simulation (q={q_s}, T={T_s}): std={std_s:.4f} "
          f"tau_peak/T={tau_s / T_s:.4f} adev_peak/q={adev_s / q_s:.4f}")
    check("7d gz dynamic_bias_stddev=q*sqrt(2/T) gives steady std q (3%)", rel(std_s, q_s) < 0.03, f"std={std_s}")
    check("7e simulated adev_peak/q matches constant (4%)", rel(adev_s / q_s, GM_ADEV_PEAK_OVER_SIGMA) < 0.04,
          f"{adev_s / q_s}")
    check("7f simulated tau_peak/T matches constant (15%)", rel(tau_s / T_s, GM_TAU_PEAK_OVER_T) < 0.15,
          f"{tau_s / T_s}")

    # 8. both conventions produce the documented values
    a = calib["gyro"]["per_axis"]["x"]
    bi, tau = a["bias_instability"], a["bias_instability_tau_s"]
    for conv, peak in (("allan_minimum", bi), ("ieee_952_B", bi * math.sqrt(2 * math.log(2) / math.pi))):
        pc = make_sdf(f"conv_{conv}.sdf")
        inject(pc, dict(cfg, bias_instability_convention=conv), calib)
        nz = noise_of(pc, "angular_velocity", "x")
        q = peak / GM_ADEV_PEAK_OVER_SIGMA
        T = tau / GM_TAU_PEAK_OVER_T
        check(f"8 {conv}: corr_time = tau_min/{GM_TAU_PEAK_OVER_T}",
              rel(val(nz, "dynamic_bias_correlation_time"), T) < 1e-8, f"{val(nz, 'dynamic_bias_correlation_time')}")
        check(f"8 {conv}: dynamic_bias_stddev = (peak/{GM_ADEV_PEAK_OVER_SIGMA})*sqrt(2/T)",
              rel(val(nz, "dynamic_bias_stddev"), q * math.sqrt(2 / T)) < 1e-8, f"{val(nz, 'dynamic_bias_stddev')}")
    code = None
    try:
        compute_noise_params(calib, dict(cfg, bias_instability_convention="bogus"), 1000.0)
    except ValueError:
        code = "ValueError"
    check("8 unknown convention rejected", code == "ValueError")

    # 9. output names match the installed schema
    schema = next((d for d in SCHEMA_DIRS if os.path.isfile(os.path.join(d, "noise.sdf"))), None)
    if schema is None:
        print(f"[selftest] SKIP 9 schema names (no sdformat schema at {SCHEMA_DIRS}; run under WSL)")
    else:
        nroot = ET.parse(os.path.join(schema, "noise.sdf")).getroot()
        n_elems = {e.get("name") for e in nroot.findall("element")}
        n_attrs = {e.get("name") for e in nroot.findall("attribute")}
        check("9a noise 'type' is an attribute in schema", "type" in n_attrs and "type" not in n_elems)
        check("9b noise children written are schema elements", set(NOISE_CHILDREN_WRITTEN) <= n_elems,
              f"{set(NOISE_CHILDREN_WRITTEN) - n_elems}")
        iroot = ET.parse(os.path.join(schema, "imu.sdf")).getroot()
        groups_ok = all(
            (g := iroot.find(f"element[@name='{sdf_group}']")) is not None
            and all(g.find(f"element[@name='{ax}']/include[@filename='noise.sdf']") is not None for ax in AXES)
            for _, sdf_group in GROUPS)
        check("9c imu/{angular_velocity,linear_acceleration}/{x,y,z} include noise.sdf", groups_ok)
        written = noise_of(p1, "angular_velocity", "x")
        check("9d written noise uses only schema names",
              set(written.attrib) <= n_attrs and {c.tag for c in written} <= n_elems)

    print(f"[selftest] {n_pass} passed, {n_fail} failed")
    print("[selftest] " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdf_path", nargs="?", help="Path to the generated SDF (edited in place)")
    parser.add_argument(
        "--config",
        default="configs/sensors/imu_noise.yaml",
        help="Path to imu_noise.yaml (relative to SAR_NANO_SWARM_ROOT or absolute)",
    )
    parser.add_argument("--cf-id", default="0", help="Crazyflie instance id (matches phase0_gate.sh's CF_ID; log labelling only)")
    parser.add_argument("--selftest", action="store_true", help="Run the offline selftest and exit")
    args = parser.parse_args()

    if args.selftest:
        sys.exit(run_selftest())
    if not args.sdf_path:
        parser.error("sdf_path is required (or pass --selftest)")

    cfg = load_config(resolve_path(args.config))
    calib_path = resolve_path(cfg["calib_results_path"])
    calib = load_config(calib_path)
    try:
        name, rate, params, notes = inject(args.sdf_path, cfg, calib)
    except ValueError as e:
        print(f"{TAG} ERROR: {e}", file=sys.stderr)
        sys.exit(1)

    conv = cfg.get("bias_instability_convention", "allan_minimum")
    print(f"{TAG} {args.sdf_path}: sensor '{name}' (cf_{args.cf_id}) updated")
    print(f"{TAG}   calib  = {calib_path}")
    print(f"{TAG}   rate   = {rate:g} Hz (from SDF <update_rate>)  convention = {conv}")
    for cal_group, sdf_group in GROUPS:
        for axis in AXES:
            p = params[sdf_group][axis]
            print(f"{TAG}   {cal_group}.{axis}: stddev={p.get('stddev', 0.0):.4g} "
                  f"dyn_bias_stddev={p.get('dynamic_bias_stddev', 0.0):.4g} "
                  f"(steady {p.get('_gm_steady_stddev', 0.0):.4g}) "
                  f"corr_time={p.get('dynamic_bias_correlation_time', 0.0):.4g} s")
    for n in notes:
        print(f"{TAG}   {n}", file=sys.stderr if n.startswith("WARNING") else sys.stdout)
    for n in convention_floor_notes(calib, conv):
        print(f"{TAG}   WARNING: convention check: {n}", file=sys.stderr)
    print(f"{TAG}   orientation: NOT noised (gz-sensors8 cannot); turn-on bias: 0 (not in calibration)")


if __name__ == "__main__":
    main()
