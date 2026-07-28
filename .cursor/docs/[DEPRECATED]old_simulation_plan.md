# Simulation, Training & Optimization Roadmap
## SAR Nano-Drone Swarm — Software-Only Track

Each phase has a **gate**: don't move to the next phase until the gate passes. Estimated durations assume one engineer, part-time hobbyist pace is 3-5x longer.

---

## PHASE 0 — Environment Setup (3-5 days)

1. Install Ubuntu 22.04 (native or WSL2 with GPU passthrough if on Windows).
2. Install ROS 2 Humble.
3. Install Gazebo — but first verify which version CrazySim currently requires. **Gazebo Garden reached end-of-life in late 2024; do not default to it.** Check CrazySim's current README and use Harmonic (or whatever it now supports) before building your whole stack on an EOL simulator. One hour of verification now vs. a painful migration at Phase 7.
4. Clone and build CrazySim:
   - `crazyflie-firmware` (SITL build via `sitl_make`)
   - Choose Gazebo backend first (MuJoCo backend is **required later for the Phase 6 perching gate** — contact dynamics on rubble — and for higher-fidelity downwash physics)
5. Clone and build `Crazyswarm2` (ROS 2 swarm interface).
6. Verify: launch `sitl_singleagent.sh`, connect with `cfclient` over the SITL UDP URI, fly manually. **Gate: single sim drone responds to manual control.**
7. Clone `LTU-RAI/darpa_subt_worlds`; load Tunnel, Urban, and Cave worlds into your Gazebo resource path.
8. Clone and build `radarays_gazebo_plugins`; run the example robot launch with Embree (CPU) backend first, OptiX (GPU) later if you have an NVIDIA card.
9. Set up experiment tracking: Weights & Biases or MLflow (free tiers) for every phase from here on — log every sim run's parameters, seeds, and metrics. Do this now, not later.
10. Set up a Python repo structure: `/sim_worlds`, `/firmware_mods`, `/perception`, `/coordination`, `/eval_scripts`, `/configs`. Version control everything, including world files and configs, not just code.

**Gate to exit Phase 0:** one Crazyflie SITL instance flying in a SubT Tunnel world, with RadaRays attached and publishing a point cloud topic you can see in RViz.

---

## PHASE 1 — Single-Drone Flight & Sensor Baseline (1 week)

1. Mass/inertia model: edit the Gazebo SDF for your Crazyflie 2.1 + RIO/UWB/GAP9Shield payload (real measured mass, not default 27g). Use CrazySim's thrust-upgrade bundle parameters as the starting point if your real motors are upgraded.
2. Re-tune PID gains in sim for the loaded mass via the CrazySim PID-tuning client workflow. Save the gain set — this is what you'll flash to real hardware later.
3. Add sensor noise models matching your real parts' datasheets: IMU noise/bias (from STM32 IMU spec), barometer noise, UWB ranging error (cm-level Gaussian + occasional multipath outliers), radar Doppler noise (~0.124 m/s std as a starting reference from published RIO work).
4. **Battery model — do NOT use the default linear drain.** Model voltage sag under load and payload-dependent hover current (full sensor stack), fit to a real bench discharge curve of your actual loaded airframe. Bingo-fuel (Phase 6/8) is a life-or-death policy and must be tuned against this, not a fantasy battery.
5. **Start real radar bench-data collection now — pulled forward from Phase 11.** Buy the actual 24/60 GHz dev kit and record static bench data (corridors, rubble mockups, a doorway). Use it to fit a **degradation model** (point sparsity, dropout rate, multipath ghost rate) that you apply on top of RadaRays output from Phase 2 on. RadaRays models rotating/scanning radar; single-chip mmWave returns are far sparser (tens of points/frame), so RadaRays alone flatters every downstream stage. This de-risks the single most fragile assumption in the pipeline.
6. Enable CrazySim's `--sensor-noise --ground-effect --wind-speed --turbulence` flags. Run hover and waypoint-tracking tests across a noise sweep.
7. **Hardware note:** evaluate the **Crazyflie 2.1 Brushless (2024)** over the brushed 2.1 — meaningfully better thrust margin and endurance, directly relieving your payload ceiling and 2–4 min sortie floor (both load-bearing constraints).
8. **Gate:** drone holds position within your target tolerance (e.g., ±10 cm) under realistic sensor noise and mild turbulence, in an empty world, before you add radar/UWB fusion.

---

## PHASE 2 — Radar-Inertial + UWB-Anchor State Estimation (3-4 weeks)

This is the highest-risk phase — validate it in isolation before anything else. **The single most important correction from review: anchor positions are NOT known.** A dropped anchor's position is only as good as the dropping drone's *drifted odometry at the moment of the drop*. The chain is a surveyor's traverse — error accumulates along it, so "resetting" against anchor #12 resets you to a point that already carries all drift up to anchor #12's placement. The backbone bounds *relative* drift locally, not absolute error deep in the structure — unless anchor positions are refined as state.

1. Consider adapting an existing open RIO front-end (EKF-RIO / x-RIO lineage) rather than writing radar-inertial from scratch. Spend your engineering hours on the anchor-SLAM and coordination layers — those are your actual novelty; RIO and auctions already exist in the literature.
2. Implement the EKF (or fixed-lag smoother) fusing: IMU propagation, radar Doppler ego-velocity updates (radar point cloud → RANSAC least-squares ego-velocity), barometer + downward ToF for altitude, and UWB range updates. **Apply your Phase-1 radar degradation model on top of RadaRays output** so the front-end sees realistic sparse/multipath returns, not idealized ones.
3. **Baseline, then the real test.** (a) Baseline: anchors at known coordinates to confirm the filter works. (b) **Mandatory — instantiate each anchor at the drone's *estimated* (drifted) pose at drop time, and include anchor positions as state variables to be refined:** via ranging from multiple drone poses, **anchor-to-anchor UWB ranging** (add this to the Tier-1 puck spec — it's cheap and dramatically stiffens the chain), and loop closures propagating corrections back through the anchor graph. The (b) results — not (a) — are your real drop-interval spec.
4. **Use floor-only anchor geometry in the sweep, not idealized floor/wall/ceiling placements.** A nano-drone dropping a puck puts it on the floor → anchors roughly coplanar → severe vertical dilution of precision, compounding your weakest axis. Idealized 3D placement flatters the results; test the geometry you'll actually get.
5. Run closed-loop flight through a multi-room SubT world section. Log estimated trajectory **and estimated anchor positions** vs Gazebo ground-truth.
6. **Metrics:** absolute position error over time AND **error at the deepest point of the traverse** (must be bounded, not growing with chain length); **vertical (Z) error** specifically; anchor-position estimation error along the chain. Plot vs anchor density.
7. Sweep anchor spacing (5m/10m/20m) **under the estimated-drop-pose condition** to find the minimum density that keeps drift bounded — this is the real spec.
8. Deliberately remove anchors mid-flight (squad overran its backbone) and confirm graceful degradation, not divergence.
9. **Gate:** with anchors placed at drifted estimated poses (not known coordinates), absolute error stays bounded to the deepest point of a 5+ minute traverse, Z error within target under floor-only geometry, and the degraded-anchor stress test doesn't diverge. *(Note: the actual drop behavior/mechanism — drop-on-link-margin policy, release dynamics — is implemented in **Phase 7**, not here; here anchors are instantiated programmatically.)*

---

## PHASE 3 — Local Mapping & Obstacle Avoidance (1-2 weeks)

1. Build a rolling local occupancy grid (or ESDF-lite) from the RadaRays point cloud.
2. Implement reactive obstacle avoidance (velocity-space or CBF-based) against this grid.
3. Test in increasingly cluttered SubT world sections (start in open Tunnel sections, move to Urban/Cave narrow passages and doorway-equivalents).
4. **Metric:** collision rate, near-miss rate, time-to-traverse vs a hand-flown baseline.
5. **Gate:** <5% collision rate across a fixed test suite of 20+ randomized cluttered approach angles in sim before moving on. Log every collision case — these become your real-world no-fly regression suite later.

---

## PHASE 4 — Tiny Place-Recognition Encoder (1-2 weeks, ML track)

**Caution:** radar place recognition in the literature is mostly on *dense spinning* radar; on sparse single-chip mmWave returns it's a much harder, less-proven problem. Treat encoder accuracy as an open risk and lean on the Phase-2 anchor-position constraints rather than expecting loop closure to carry the load alone.

1. Auto-generate a labeled dataset by flying scripted/randomized trajectories through the SubT worlds — **but don't train on only the three stock worlds; that invites overfitting.** Add procedural world generation (randomized tunnel/room/junction layouts) as cheap insurance; it also fattens your Phase 9 regression suite. Record (radar point cloud or rasterized radar image, ground-truth pose) pairs, **applying the Phase-1 radar degradation model** so the encoder trains on realistically sparse returns.
2. Define positive pairs (same place, different pose/time) and negative pairs (different places) using ground-truth pose distance as the label oracle.
3. Train a small CNN/embedding network (contrastive or triplet loss) in PyTorch on this synthetic dataset.
4. Validate loop-closure precision/recall on held-out trajectories (different random seed/world section than training).
5. Export to ONNX, then run through GreenWaves' GAP SDK / NNTool toolchain to quantize to int8 and check it actually fits GAP9's L1/L2 memory and meets your inference-time budget (target: under ~20ms per the published GAP9 CNN benchmarks).
6. **Gate:** quantized model retains acceptable loop-closure accuracy (compare pre/post quantization precision-recall) and fits the real memory/latency budget — not just the float32 sim version.

---

## PHASE 5 — Victim Vital-Signs Detection (Parallel track, 1-2 weeks; not flight-dependent)

This cannot be validated against the SubT sim (no biological radar physics) — use real datasets.

1. Download the radar human-breathing SAR dataset and the radar heart-sounds dataset (both free, cited earlier).
2. Implement the signal-processing pipeline: clutter suppression (e.g., MTI/SVD-based static clutter removal), phase extraction, breathing/heartbeat frequency estimation.
3. Validate against the datasets' ground-truth reference sensors (lidar reference / ECG reference). Track detection rate vs distance, vs body posture, vs added synthetic noise floor matching your actual radar's specs.
4. Determine your **minimum reliable integration window** (e.g., is 10s enough, or do you need 20-30s?) and your **false-positive rate** at your target confidence threshold — these numbers directly set the perch-and-stare timing parameter.
5. Port the (much lighter) signal-processing pipeline to run on GAP9 SDK; check it fits the power/latency budget for a perched, near-idle drone.
6. **Gate:** documented detection rate, false-positive rate, and required integration time at realistic SNR — these become hard parameters in Phase 6's behavior tree, not assumptions.

---

## PHASE 6 — Single-Drone Behavior Integration + Perching (1-2 weeks)

1. Combine Phases 2-5 into one drone's full behavior tree/state machine: explore → detect cue → perch → integrate vitals → alert or resume → bingo-fuel decision (RTH / become relay / hold on victim).
2. **Perching gate (new — perching carries huge load in the design and was previously untested).** Using the MuJoCo backend for contact physics, test perch + re-launch on N randomized rubble geometries: landing-site selection, contact dynamics, whether a ~30 g airframe reliably takes off again from debris, prop-damage risk. Implement a fallback "steadiest-hover" mode when no perch site is viable, with a **measured vital-signs SNR penalty** for hovering vs perched. *Sub-gate:* successful perch/re-launch on ≥80% of randomized rubble sites, or clean fallback to hover.
3. **Radar mode-sharing (new).** One radar does obstacle avoidance, ego-velocity, AND vital-signs — different chirp configs. Make explicit: chirp reconfiguration time, what the EKF does during the radar blackout on approach-to-perch (coast on IMU + last anchor fix), and whether GAP9 holds both processing pipelines in memory or swaps them (carry this to the Phase 10 memory budget).
4. Inject synthetic "victim" entities at SubT cave-world artifact survivor locations; since RadaRays won't generate biological signatures, **mock the vitals trigger** using your Phase-5 detection-rate/false-positive distribution (sample an outcome, don't hand-wave a guaranteed hit). **Thermal, acoustic, and CO₂ cues are all mocked here too.** Note explicitly that **thermal-inertial odometry (design §2.1) is NOT sim-validated** — Gazebo modeling of low-contrast tunnel thermal scenes is weak, so it stays a real-hardware-only validation item; don't let sim results imply the fused radar+thermal stack was tested.
5. Run full single-drone missions end-to-end; log outcome, **battery state via the Phase-1 non-linear model**, time-to-detect.
6. **Gate:** single drone completes explore→perch→detect→alert→return across 20+ randomized missions, perching sub-gate met, with battery and bingo-fuel logic firing correctly under both early and late detections.

---

## PHASE 7 — Multi-Drone Squad Coordination + Backbone Deployment (2-3 weeks)

1. Scale to 10 simulated drones (one squad) via CrazySim multi-agent launch.
2. **Anchor-drop behavior (its real home — previously orphaned).** Implement the actual dropping: a **drop-on-link-margin policy** (drop a node *before* the last link's margin falls below threshold) plus drop/release dynamics. This is where the Phase-2 programmatic anchors become an autonomous behavior.
3. **RF propagation model (new — the drop trigger needs something to fire against).** CrazySim's comms-delay sim won't capture tunnel RF, where one bend can drop 30+ dB. Add a distance + wall/bend attenuation model with a link-margin threshold. You don't need ns-3 fidelity — you need an honest signal for the drop policy and the connected/disconnected coordination switch to react to.
4. Implement and test Anchor/Shadow leader election: kill the Anchor mid-mission, confirm Shadow promotes, a new Shadow is elected, buffered pose graph intact.
5. Implement auction-based frontier allocation among connected agents; response-threshold dispersion fallback for disconnected agents (switch driven by the Phase-7 RF model).
6. Test both regimes under increasing latency/packet loss (CrazySim comm-delay + your RF model), not just the ideal case.
7. Implement and test elastic-breadcrumb RTH (inverted topological graph) under normal and degraded-anchor conditions (reuse Phase 2 stress tests).
8. **Metrics:** frontier coverage rate, redundant-coverage rate (overlap waste), leader-election recovery time, RTH success rate vs comms degradation.
9. **Gate:** squad completes coordinated exploration with **<15% redundant coverage** (was an undefined placeholder — set to your own tolerance, but define it), survives a forced Anchor kill without losing the map, drops backbone nodes correctly before losing links, and RTH succeeds even when comms drops to zero mid-mission.

---

## PHASE 8 — Squad Fracturing & Multi-Squad Map Merge (1-2 weeks)

1. Implement the auction-triggered squad-fracture behavior at branch points.
2. Run 2+ squads (20+ drones) simultaneously in a larger SubT world (Urban or Cave, which have multiple branches).
3. Implement the base-station merge: pose-graph optimization using anchor-coincidence constraints + place-recognition loop closures (from Phase 4's encoder) across squads.
4. **Metric:** merged-map consistency error (compare against full Gazebo ground truth of the whole world) and victim-marker placement error in the merged frame.
5. **Gate:** two independently-fractured squads' maps merge into one consistent global map with bounded error, and a victim found by one squad appears correctly placed for an operator who never saw that squad's raw data.

---

## PHASE 9 — Full-Scale Stress & Attrition Testing (2 weeks)

1. Scale to the full 50-drone, 5-squad fleet (note CrazySim's real-time-factor will drop — consider headless `gz sim -s` and a multi-core machine; budget for this being slow). **DDS plumbing (new):** at ~50 SITL instances, ROS 2 default DDS discovery traffic becomes a bottleneck *before* physics does — plan on a Fast-DDS discovery server or Zenoh bridging, and budget it as real work, not a config flag.
2. Run wave-based launches (Phase 7 of your technical plan) rather than single-shot deployment.
3. **Deliberate failure injection:** randomly kill drones at scripted rates (5%, 15%, 30% attrition) mid-mission; measure mission success (victims found, area covered) vs attrition rate. This produces your real attrition-budget numbers instead of guessing.
4. Stress comms: randomly sever relay-node links, simulate dropped-beacon failures, test rendezvous/Class-A-vs-Class-B data prioritization under bandwidth caps.
5. Run a full randomized scenario suite (different world, different victim placement, different seed) at least 30-50 times — treat this as your regression test suite going forward, run automatically (CI) every time you change an algorithm.
6. **Gate:** mission success rate holds within an acceptable band across the attrition sweep; no silent failure modes (e.g., orphaned drones that neither RTH nor relay nor report).

---

## PHASE 10 — Optimization Pass (1-2 weeks, ongoing)

1. Profile every on-drone algorithm's compute/memory footprint against actual GAP9 budgets (150 int-GOp/s, ~128KB L1, ~1.5MB L2, ~32MB L3) — not against your dev laptop. **Include the Phase-6 radar mode-sharing cost:** whether the obstacle/ego-velocity and vital-signs processing pipelines fit in memory simultaneously or must be swapped, and the swap latency.
2. For learned components (place-recognition encoder, vitals classifier if learned): quantize (int8), prune, and re-validate accuracy didn't collapse (Phase 4/5 gates re-run post-quantization).
3. For classical components (EKF, auction bid function, CBF avoidance): profile cycle counts, not just "it runs" — tune integration rates (e.g., does the EKF really need 200Hz or does 100Hz hold drift bounds at lower compute cost).
4. Hyperparameter tuning for the auction bid weights (battery/trajectory/squad-health weighting) and the bingo-fuel threshold: use Bayesian optimization (Optuna) or CMA-ES over the Phase 9 scenario suite as the objective, **not manual guessing, and not RL** — keep this classical/optimizer-based per your "avoid fragile learned policies" principle.
5. **Gate:** every on-drone model/algorithm fits its real hardware budget with margin, and bid/threshold parameters are tuned against the full scenario suite rather than a single scenario (avoids overfitting to one map).

---

## PHASE 11 — Hardware-in-the-Loop Bridge (before real flight)

1. Swap the SITL firmware build for a build that actually links against your real GAP9 inference binaries (run the quantized models in CrazySim's loop, not a Python mock).
2. Replace simulated radar/UWB noise models with playback of **real recorded sensor logs** from your actual hardware fed into the same EKF code path (you began this in Phase 1–2 with bench data and the degradation model; here you extend it to full logged flights/motion, not just static bench captures).
3. Re-run the Phase 2, 4, 5 gates against this hybrid sim+real-sensor-log pipeline. Any gate that fails here means your noise models in pure sim were wrong — fix the model, don't just patch the algorithm.
4. **Gate:** algorithm behaves consistently between pure-sim and real-sensor-log-replay before you risk real hardware in a real cluttered/smoke environment.

---

## Ongoing: Regression Discipline

From Phase 9 onward, every algorithm change must run against the full scenario suite before being considered "done." Track metrics over time in your experiment tracker (W&B/MLflow). A change that improves one metric but regresses mission-success-rate-under-attrition is a regression, not progress — this is the actual definition of "won't fail in real life" for this project: it doesn't get worse on the stress suite as you add features.

---

### Rough total timeline (revised up for the added anchor-SLAM, perching, battery/RF modeling, and radar-bench work)
Phases 0-6 (single-drone pipeline solid): **8-12 weeks**
Phases 7-9 (swarm coordination + scale + attrition): **5-7 weeks**
Phases 10-11 (optimization + HIL bridge): **3-4 weeks**
**Total: ~4-6 months** of focused work before real hardware field testing is responsible to attempt. Plus hardware lead time to start *now*: order the radar dev kit and a Crazyflie (ideally the 2.1 Brushless) in Phase 0 so bench data is ready for Phase 1–2.
