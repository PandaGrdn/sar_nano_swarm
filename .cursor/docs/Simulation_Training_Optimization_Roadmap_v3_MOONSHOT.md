# Simulation, Training & Optimization Roadmap — v3 (MOONSHOT)
## SAR Nano-Drone Swarm — Entrance-Gauged Collaborative SLAM, No Interior Infrastructure

**What changed from v2:** the localization architecture no longer relies on dropped Tier-1 UWB anchor pucks. The research objective is now:

> **Bounded-error collaborative localization for a nano-drone swarm in dark, GPS-denied structures, with no infrastructure deployed inside the structure** — gauged only at the entrance (base station), held together by an inter-drone UWB range+bearing (PDoA) mesh with mutual-bearing yaw constraints, and *bounded* (not merely slowed) by sparse-radar loop closures that the swarm actively plans to create.

Three honest framings this roadmap commits to:

1. **The math:** without any external reference, absolute error cannot be bounded (gauge freedom). Our references are (a) one fixed node at the entrance, and (b) the static environment itself, via loop closures. Everything in this plan exists to make those two references carry the load the pucks used to carry.
2. **Comms is not the moonshot.** Deleting anchors deletes the *localization* backbone, not the RF physics. Comms in v3 = landed drones as relays + rendezvous/data-mule for bulk data. Class-A alert latency from deep inside will be worse than the v2 backbone design; this is an accepted, stated trade.
3. **The control arm stays.** Static-anchor localization (v2's Phase 2) is retained as the experimental baseline. Every mesh/loop-closure result is reported against it. This is what makes the work publishable rather than merely ambitious.

**Four research workstreams → phase mapping:**

| Workstream | Phase home | Paper-shaped deliverable |
|---|---|---|
| W1: Relative-pose mesh (UWB PDoA range+bearing + mutual-yaw) | Phase 2 | Cooperative localization in degenerate corridor geometry (ICRA/IROS class) |
| W2: Sparse single-chip mmWave place recognition | Phase 4 | Submap-accumulated radar place recognition on real sparse returns (RA-L class) |
| W3: Active loop-closing as a swarm behavior | Phase 7 | Auction-based active SLAM at swarm level (ICRA/IROS/AURO class) |
| W4 (hedge): Passive mmWave backscatter tags | Track H (parallel) | Systems paper if integrated; otherwise de-risks the product |

Each phase has a **gate**. Durations assume one engineer; hobbyist pace is 3–5x longer.

---

## PHASE 0 — Environment Setup (LARGELY COMPLETE per sar_nano_swarm repo)

Done (verified in repo): Ubuntu 22.04 + ROS 2 Humble + **Gazebo Harmonic (gz-sim 8)** — Garden EOL correctly avoided; CrazySim SITL + Crazyswarm2 pinned as submodules; **custom `radarays_gz2` plugin** replacing ROS1-era RadaRays, publishing PointCloud2 on `/radar/points`; SubT worlds submodule; MLflow; path-portable world files; radar smoke test.

Remaining Phase 0 items:

1. **MuJoCo backend check** — confirm the CrazySim MuJoCo path builds in your pinned commit (`3ec8b55`). Required for the Phase 6 perching gate. If the pinned commit predates solid MuJoCo support, plan the submodule bump now, not at Phase 6.
2. **Inter-drone UWB PDoA measurement plugin (new, load-bearing for W1).** Sibling to `radarays_gz2`: a gz-sim System plugin that, for each drone pair, emits (range, azimuth, elevation) *in the observer's body frame*, modeling a dual/multi-antenna UWB PDoA module. Configurable: range noise (omnidirectional — TWR ranging works regardless of orientation), angle noise with off-boresight growth, **AoA field-of-view mask (~90–120° per published module characterizations — outside the AoA cone the plugin still emits range-only edges, which matters for graph rigidity)**, NLOS/multipath outlier injection, and update rate / channel-schedule slotting. Identity is known per measurement (UWB ranging is a two-way exchange — no association ambiguity to model). Treat noise parameters as placeholders until Phase 1 validation replaces them.
3. Full-stack smoke test: one SITL Crazyflie flying in a SubT Tunnel world with `/radar/points` live in RViz (your v2 Phase 0 exit gate — verify it passes end-to-end, not just the standalone radar world).

**Gate to exit Phase 0:** v2 gate (SITL drone + radar point cloud in SubT world) **plus** two SITL drones exchanging simulated UWB range+bearing measurements on a ROS topic, including correct range-only fallback when a neighbor is outside the AoA cone.

---

## PHASE 1 — Single-Drone Baseline + Hardware Bench Data (1–2 weeks)

Unchanged from v2 (mass/inertia model, PID retune, sensor noise from datasheets, **non-linear battery model fit to a real discharge curve**, CrazySim noise/turbulence flags, ±10 cm hover gate, evaluate **Crazyflie 2.1 Brushless**), with these edits:

*(Revision: no dev-kit purchase in Phase 1 — noise models are built in-house instead. The acceptance criterion below is what keeps that decision honest: the models must be validated against published empirical data of the same sensor class, not derived in a vacuum. Hardware purchase deferred to Phase 11, where it becomes mandatory.)*

1. **Noise models (in-house method, externally validated).** Whatever the internal modeling method, each model must reproduce published empirical measurements before it enters the sim. Sensor roles are now fixed: **UWB PDoA for all inter-drone sensing; TI IWR6843AOP radar for odometry, mapping, obstacle avoidance, and vital-signs only** (no inter-drone role):
   - **UWB PDoA measurement model (primary — the W1 channel):** validate against published DW3xxx-class characterizations, especially the ETH-PBL smart UWB node work (arXiv 2312.13672 + open dataset/firmware at `ETH-PBL/UWB_DualAntenna_AoA`: ~cm range precision, ~2.4° mean angular accuracy within ±45° of boresight) and DW3xxx app-note data. Model must include: angle-error growth off-boresight, the **~90–120° AoA FoV limit** (range-only beyond it), error vs range, NLOS bias/outlier tail near walls, and aperture limits at nano-drone antenna spacing (~4–5 cm — budget conservatively, 5–15°, until Phase 11 bench data says otherwise; the ETH-PBL 2.4° figure is a larger, tuned module).
   - **Radar degradation model (odometry/mapping returns):** unchanged — fit/validate against **ColoRadar** and the EKF-RIO/x-RIO public datasets for point sparsity, dropout, multipath ghost rates, and Doppler noise; validate against the IWR6843AOP datasheet/app notes since that's the committed part.
   - **Acceptance test (gate condition):** for each model, a short written comparison showing the model's output statistics against the published measurements it claims to match. A model that can't be checked against any external measurement doesn't pass, regardless of method.
2. **Load the validated UWB model into the Phase 0 plugin,** replacing placeholders; apply the radar degradation model on top of `radarays_gz2` output from Phase 2 onward.
3. **IMU yaw-drift parameters:** take from published Crazyflie/BMI088-class characterizations rather than datasheet random-walk figures alone; yaw error rotates every PDoA measurement, so this number is load-bearing for W1.
4. **Sensitivity discipline (promoted to mandatory):** because no in-house bench data grounds these models, every headline Phase 2+ result is reported at nominal **and** 2x noise. Any conclusion that flips between the two conditions is flagged as model-sensitive and not claimed.

**Gate:** v2 hover gate **plus** radar and PDoA models validated against external empirical data (acceptance tests written up), plugged into the sim, with the 2x-noise condition wired into the eval scripts.

---

## PHASE 2 — Entrance-Gauged Relative-Pose Mesh (W1 core; 4–6 weeks — the heart of the moonshot)

This replaces v2's anchor-SLAM phase. **Committed modality: UWB PDoA for all inter-drone measurements** (identity-aware, omnidirectional range, ~90–120° AoA cone). The estimation problem: a distributed pose graph over all airborne (and landed) drones, gauged by a single fixed node at the entrance (Tier-0 base with its own UWB PDoA unit), with edges from (a) per-drone odometry — RIO with the Phase-1 degradation model applied; adapt the **Michalczyk et al. (ICRA 2023) multi-state EKF with persistent radar landmarks** as the front-end baseline rather than plain Doppler-only RIO, since tracked high-RCS landmarks measurably cut drift between mesh updates — (b) inter-drone UWB range+bearing factors, (c) **mutual-bearing factors** (A measures bearing to B and B to A in the same window → their relative yaw becomes observable), and (d) altitude factors (baro + downward ToF — Z still gets no help from a coplanar mesh strung along a corridor; the vertical channel remains sensor-carried).

0. **AoA-cone coverage handling (replaces the retired modality trade study).** The UWB AoA cone (~90–120°) means not every link gets an angle — out-of-cone neighbors contribute range-only edges. Per the graph-rigidity literature (range+angle edges make a subgraph rigid; range-only edges leave rotational freedom), the estimator and the behaviors must together ensure enough angle edges exist: model antenna mounting orientation explicitly, evaluate whether corridor chains need alternating mounts or periodic yaw-glances to close mutual-bearing pairs, and have the graph track its own rigidity (flag floppy subgraphs) rather than silently absorbing them.

1. **Baseline condition (retained control arm):** v2's anchor experiments — anchors at drifted drop poses, positions as state, floor-only geometry, 5/10/20 m spacing sweep. Run it once, properly, and freeze the results. Every W1 claim is reported against this.
2. **Measurement model realism:** use the Phase-1 PDoA model including off-boresight degradation and NLOS outliers. Robust kernels (Huber/DCS) on all PDoA factors from day one — multipath outliers are a when, not an if.
3. **Yaw handling is the make-or-break sub-problem.** Implement and compare: (a) gyro-only yaw (control — expect divergence), (b) mutual-bearing pairwise yaw constraints, (c) mutual-bearing + entrance-node bearing observations for drones in view of base. Metric: yaw error vs time and vs hops from entrance.
4. **Chain/mesh geometry experiments:** single-file corridor chain (the degenerate case that killed range-only cooperative localization — verify range+bearing actually rescues it); staggered chain; chain with 1–2 drones deliberately held as static mesh nodes (landed — preview of Phase 7 behavior); branching mesh at a junction.
5. **Error-vs-hops characterization:** absolute position error at the frontier drone vs number of PDoA hops from the entrance, across the Phase-1 noise model and a pessimistic 2x-noise condition. This curve *is* the headline result: it tells you the depth at which "which room" accuracy (call it 2–3 m) is lost without loop closures.
6. **Distributed vs centralized:** implement centralized graph optimization first (base station solves, results broadcast); measure the message budget a truly distributed solver would need, and defer distributed implementation unless the budget clearly breaks the comms model. Don't gold-plate this in sim.
7. **Failure drills:** kill mid-chain drones (mesh partition — does the far side's covariance grow gracefully or does the filter lie?), entrance-node dropout (gauge loss — detect and flag, don't silently drift), sustained NLOS on a critical link.

**Gate:** in a multi-room SubT section with realistic PDoA + degraded-radar noise, (a) the corridor-chain configuration remains observable (no lateral blow-up) where range-only demonstrably fails, (b) frontier-drone error grows sub-linearly with hop count and stays within the "which room" budget out to a stated hop depth N — publish N, don't hide it, (c) yaw stays bounded via mutual-bearing constraints without magnetometer, (d) mesh-partition and gauge-loss drills degrade gracefully. **Paper 1 checkpoint:** baseline-vs-mesh comparison + error-vs-hops curves + degenerate-geometry result is a submittable unit even before loop closures exist.

---

## PHASE 3 — Local Mapping & Obstacle Avoidance (1–2 weeks; unchanged from v2)

Rolling occupancy/ESDF-lite from `radarays_gz2` (with degradation model applied), reactive avoidance (velocity-space or CBF), cluttered SubT test suite. **Gate:** <5% collision rate across 20+ randomized cluttered approaches; log every collision as future regression cases. No moonshot changes — but note the avoidance stack must run on Light Scouts without GAP9, so keep it classical and cheap.

---

## PHASE 4 — Sparse-Radar Place Recognition (W2; 3–5 weeks — the open research problem)

Upgraded from "train a small encoder" to a research workstream, because loop closures are now the *only* mechanism that bounds (rather than slows) error, and sparse single-chip place recognition is unproven. Three candidate representations, evaluated head-to-head:

1. **Submap accumulation → persistent-landmark pipeline (primary bet).** Accumulate degraded radar returns over a short trajectory window (1–3 s of flight, poses from the Phase-2 mesh estimate) into a denser local submap — motion as synthetic aperture, converting tens-of-points frames into hundreds-of-points submaps. Then run Michalczyk-style trail-tracking *on the accumulated points* (promote points seen consistently across successive submaps), so accumulation supplies the raw material and persistence supplies the trust. Sweep accumulation-window length vs localization error (longer windows = denser submaps but more within-window drift contamination).
2. **Scan-Context-style classical descriptor (zero-training baseline).** A hand-built polar/height signature of the submap, matched by correlation across rotations — no training data needed, cheap, interpretable. Run this *first*: it tells you quickly whether learned encoders are even necessary, and it's the loop-closure method 4D iRIOM used successfully on radar.
3. **Learned encoder on submaps:** small contrastive/triplet-trained network (PointNet-AE-style per MMGraphSLAM precedent) on accumulated submaps.
4. **Doppler-augmented descriptors:** environment Doppler signatures (static-world velocity field structure) as an extra channel.
5. **Raw-frame encoder (control):** the v2 approach, expected to underperform — kept to quantify how much submapping buys.

Dataset and validation edits from v2, all retained: procedural world generation (don't overfit three stock worlds), degradation model applied to all training data, held-out worlds for eval, ONNX → GAP9 int8 with memory/latency budget. New requirements:

4. **Wrong-closure rate is the gate metric, not just precision/recall.** A false loop closure corrupts the whole graph. Determine, empirically, the false-positive rate your Phase-2 pose graph rejects cleanly with robust kernels + consistency checks (e.g., pairwise-consistency/PCM-style vetting), then gate the encoder against *that* threshold at whatever recall it can manage. Low recall is survivable (fewer closures = slower bounding); bad precision is fatal.
5. **Real-data validation before the gate closes:** re-run the eval on bench-recorded and (once available) logged-flight radar data, not RadaRays output alone. If sim-trained descriptors collapse on real returns, that finding itself feeds Paper 2.

**Gate:** at least one representation achieves, on realistically degraded (and where possible real) sparse returns, a precision/recall operating point where wrong closures are below the graph's demonstrated rejection threshold, in int8 on GAP9's budget. **Paper 2 checkpoint:** the submap-accumulation method + real-sensor evaluation, positive or carefully-negative.

---

## PHASE 5 — Victim Vital-Signs Detection (parallel, 1–2 weeks; unchanged from v2)

Real datasets (breathing SAR + heart sounds), clutter suppression pipeline, detection rate / false-positive rate / minimum integration window quantified, GAP9 port. The mission is still SAR; the moonshot changes how drones know where they are, not why they're there. **Gate:** unchanged.

---

## PHASE 6 — Single-Drone Behavior Integration + Perching (1–2 weeks)

As v2 (behavior tree: explore → cue → perch → integrate vitals → alert/resume → bingo decision; MuJoCo perching gate with ≥80% perch/re-launch on randomized rubble; radar mode-sharing made explicit; mocked vitals/thermal/acoustic/CO₂ with Phase-5 statistics; non-linear battery), with these moonshot edits:

1. **Remove:** puck payload handling and drop-mechanism behaviors. (Payload margin freed — note it, it partially relieves the 2–4 min sortie floor.)
2. **Add: mesh-duty landing.** Landing-site selection gains a second objective besides vitals-stare: "become a static mesh node / relay here." Geometry-aware (prefer sites that de-collinearize the mesh; elevated debris preferred for Z diversity and RF line-of-sight), battery-aware. A landed drone keeps its ranging channel + low-rate radio alive for tens of minutes — and **if the Phase 2 trade study picked radar+reflector, a landed drone is a passive landmark even after its battery dies**, so mesh-duty sites should be scored partly as *permanent* landmark placements.
3. **Add: radar blackout accounting during mode switches** — now with a fourth mode if the trade study picked radar-based inter-drone sensing (odometry / obstacle / vitals / neighbor-tracking chirp configs share one 6843AOP). Make explicit the time-multiplexing schedule, what the EKF coasts on during each mode's blackout, and the perched case: a drone in vital-signs mode stops *measuring* neighbors but its reflector keeps *being measured* (vs UWB, where a busy/asleep node answers ranging on a separate radio — log whichever assumption applies).

**Gate:** v2 gate + perching sub-gate + at least one mission variant where the drone ends its sortie as a functioning static mesh/relay node and neighboring sim drones measurably benefit (localization covariance drops when ranging to it).

---

## PHASE 7 — Swarm Coordination: Mesh Maintenance + Active Loop Closing (W3; 3–5 weeks)

v2's backbone-deployment phase, rebuilt. Retained: 10-drone squad scale-up, Anchor/Shadow leader election with buffered pose graphs, auction-based frontier allocation + response-threshold fallback, latency/packet-loss stress, elastic-breadcrumb RTH (now over the mesh-estimated topological graph), **RF propagation model** (distance + wall/bend attenuation — still needed; the connected/disconnected coordination switch and relay decisions fire against it). Removed: anchor-drop policy and drop dynamics. Added — the W3 research content:

1. **Mesh-configuration planning.** The swarm must decide, continuously, who flies the frontier and who holds (hovers or lands) as a mesh node. Implement as auction terms: a drone bids on "mesh-node duty at site X" with battery state, position value (de-collinearization, hop-count reduction, RF relay value), and expendability class. This replaces the drop-on-link-margin policy: **the trigger condition is analogous (don't let the last link's margin/geometry collapse), but the resource spent is a drone-minutes budget, not a puck.**
2. **Active loop-closure planning.** Add an information-value term to frontier auctions: trajectories that revisit mapped regions, cross another drone's path at a junction, or return past known keyframes *purchase* loop closures with flight time. Start dead simple — a bonus for paths intersecting the existing keyframe graph within descriptor-match range — and let Phase 9's mission-level metrics judge whether sophistication pays.
3. **Inter-drone loop closures:** when two drones' descriptors match across the mesh, that's a cross-agent factor. Wire it through the same vetting as Phase 4.
4. **Radar-modality swarm mechanics (conditional on the Phase 2 trade study picking radar or hybrid):** (i) **chirp TDM/scheduling** — many 6843AOPs radiating the same band in one corridor will interfere; add a slot-scheduling scheme and model chirp-collision dropout in the plugin; (ii) **yaw-glance / mount-orientation coordination** — the FoV coverage plan from Phase 2 item 0 becomes a real squad behavior here (who glances back, when), budgeted as flight-time cost in the auctions.
5. **Comms honesty (the accepted v3 trade):** Class-A alerts route over landed-drone relays where they exist, else store-and-forward to the nearest connected node, else data-mule on RTH. **Measure and report alert latency vs depth** — expect it to be worse than v2's dedicated backbone; the mission-level question is whether it's still operationally acceptable (seconds-to-minutes, not never).

**Gate:** squad explores with <15% redundant coverage; mesh stays connected-enough that frontier-drone error meets the Phase-2 budget *with* loop closures now active (show the error-vs-hops curve bends flat where closures occur — this plot is the moonshot's thesis in one figure); Anchor kill survived; RTH succeeds with comms at zero; alert-latency-vs-depth curve produced and stated. **Paper 3 checkpoint:** active loop closing as an auction mechanism, evaluated at squad scale.

---

## PHASE 8 — Multi-Squad Map Merge (1–2 weeks)

v2's merge used anchor-coincidence constraints; those are gone. v3 merge rests on: (a) the **shared entrance gauge** — all squads hang off the same base node, so inter-squad error is bounded by each squad's error-to-entrance, (b) **cross-squad place-recognition closures** (Phase 4 encoder, same vetting), (c) **inter-squad encounter factors** — when squads meet at a junction, direct range+bearing measurements between members (whichever modality won the trade study) are gold-plated merge constraints; consider making brief rendezvous-at-junction a deliberate behavior. **Gate:** two independently-fractured squads' maps merge with bounded consistency error; a victim found by one squad is correctly placed for an operator who never saw that squad's data. Report merge error vs the v2 anchor-based baseline from Phase 2's control arm.

---

## PHASE 9 — Full-Scale Stress & Attrition (2–3 weeks)

As v2 (50 drones, DDS discovery-server/Zenoh plumbing budgeted as real work, wave launches, 5/15/30% attrition injection, comms severing, 30–50-run randomized regression suite in CI), plus moonshot-specific failure modes:

1. **Mesh-partition attrition:** killed drones now also remove mesh nodes — attrition attacks localization, not just coverage. Measure error inflation vs attrition rate.
2. **Mesh-duty vs coverage trade under attrition:** as drones die, does the auction correctly rebalance who holds mesh duty, or does coverage greedily starve the mesh (silent failure mode to hunt for)?
3. **Deliberate-expenditure accounting:** drone-minutes spent as static mesh nodes are now a budgeted mission cost; report victims-found and area-covered *per drone-minute*, so the moonshot architecture can be compared honestly against the v2 puck architecture (simulate the v2 system once at this scale as the comparison arm if the numbers are contested).

**Gate:** mission success holds within band across the attrition sweep; no orphaned drones; localization error at the frontier stays within budget at ≥15% attrition.

---

## PHASE 10 — Optimization Pass (1–2 weeks, ongoing; as v2)

GAP9 budgets, int8 re-validation, cycle-count profiling, Optuna/CMA-ES over the Phase 9 suite for bid weights and bingo thresholds (still no RL). Added: the graph-optimization message budget (Phase 2 item 6) and the mesh-duty auction terms join the tuned-parameter set.

---

## PHASE 11 — Hardware-in-the-Loop Bridge (as v2, extended)

**Hardware purchase happens here at the latest** — **two TI IWR6843AOP EVMs** (two, so inter-drone reflector tracking can be bench-tested drone-to-drone-analog), a handful of small trihedral corner reflectors, the Crazyflie 2.1 Brushless, and **UWB PDoA eval kits only if the Phase 2 trade study kept UWB in the design** — deferred from Phase 1 by the no-dev-kit decision, and now mandatory: this phase is the first time reality gets a vote on the in-house noise models, so nothing beyond it can be claimed without it. Steps: bench-record real data for every modality the design kept (corridor, cluttered room, NLOS cases; for the radar-reflector channel, specifically measure angle accuracy across the FoV, mirror-ghost rate near walls, and two-reflector confusion distance); **first, run the Phase-1 acceptance tests against your own recordings** — if the in-house models don't match your own bench data, that supersedes the published-data validation, and if the *trade-study-winning* modality's model was materially wrong, re-run the Phase 2 item-0 trade study with corrected numbers before trusting the BOM decision; then real GAP9 binaries in the SITL loop; replay real logged data through the same estimator code paths; re-run Phase 2/4/5 gates on replayed data. Any gate that fails here means a noise model was wrong — fix the model, don't patch the algorithm. **Gate:** consistent behavior between pure sim and real-log replay. With no in-house bench data earlier in the plan, this phase is the single sim-to-real checkpoint — budget generous time for it and expect at least one round-trip back to re-tune models and re-run Phase 2/4 evals.

---

## TRACK H (parallel hedge, ~10% of effort) — Passive mmWave Backscatter Tags

MiFly-class battery-free tags read by the onboard radar: sticker-weight references that would restore anchor-style bounding without the puck's payload/battery/cost problem. Not the primary architecture; kept alive because it rescues the product if W2 stalls.

1. Track the MiFly line (Adib group / Atheraxon commercialization); note current demonstrated range (~6 m) and what amplifier/antenna work would extend it.
2. Define the interface now: a tag observation is just another range(+bearing) factor with a known-ish landmark — the Phase-2 graph accepts it without rearchitecting. One config flag should turn tags on.
3. **Partial merge with the radar-reflector path:** if the Phase 2 trade study picks radar+corner-reflector inter-drone sensing, Track H's cheapest form is already in the system — bare corner reflectors (no backscatter electronics at all) can be flicked or left as passive breadcrumb landmarks, read by the same 6843AOP channel with the same anonymous-landmark association machinery. MiFly-class *modulated* tags then become the upgrade path that adds identity to those landmarks, directly solving the association problem.
4. If W2 shows loop closures can't reach the wrong-closure gate by end of Phase 4, promote Track H to a workstream and re-scope: mesh + sparse passive reflectors/tags is still dramatically less infrastructure than v2's pucks, and still a strong systems paper.

---

## Ongoing: Regression Discipline (as v2)

From Phase 9 on, every change runs the full suite in CI; a change that improves one metric but regresses mission-success-under-attrition is a regression. Additional v3 tracked metric: **frontier localization error vs hops**, plotted per run — the moonshot's health bar.

---

## Revised timeline (moonshot tax included)

- Phases 0–2 (mesh core solid, Paper 1 material): **6–9 weeks** (Phase 0 mostly done)
- Phases 3–6 (single-drone pipeline + W2 + perching, Paper 2 material): **7–10 weeks**
- Phases 7–9 (swarm + active closing + scale, Paper 3 material): **7–10 weeks**
- Phases 10–11: **3–4 weeks**
- **Total: ~6–8 months** before real-hardware field testing is responsible (vs ~4–6 for v2). The extra 2 months buy three paper-shaped results and the moat, and the v2 architecture remains reachable at any point via Phase 2's control arm + Track H.

**Hardware:** no purchases until Phase 11 (per the in-house noise-model decision) — order **two IWR6843AOP EVMs + corner reflectors + Crazyflie 2.1 Brushless** (and UWB kits only if the trade study kept UWB) roughly 4–6 weeks before Phase 11 starts, to cover lead time. **Download now (free):** the ColoRadar dataset, the EKF-RIO/x-RIO public datasets, and TI's IWR6843AOP datasheet/app notes — the external ground truth for the Phase 1 model acceptance tests.
