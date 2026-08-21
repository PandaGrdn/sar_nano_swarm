# Calibrating a drone radar noise model against ColoRadar

## Files

- `calibrateRadarNoiseFromBag.m` + `inspectRosbag.m` — MATLAB path for a raw
  ColoRadar `.bag` file. No Docker, no HDF5 export step.
- `calibrateRadarNoise.m` + `inspectRadarH5.m` — MATLAB path for an already
  *exported* `.h5` file (produced by `arpg/coloradar-library`'s Docker
  tool). This is the one that's been exercised against real data so far.
- `radar_calibration.py` — Python equivalent of the `.h5`-based path. Note:
  this has NOT been updated with the schema fixes below yet -- if you need
  the Python calibration path specifically, port the confirmed schema and
  the timestamp-matching fix from `calibrateRadarNoise.m` over first.
- `radar_noise_model.py` — loads *any* of the calibration outputs above
  (`.json` from MATLAB, `.yaml` from Python) and turns it into a runtime
  `CalibratedRadarNoiseModel.apply()` function, plus an optional ROS 2 node
  that sits between a Gazebo "ideal" radar topic and your real radar topic.
  This is the piece that actually runs inside Gazebo/ROS 2.

**A note on this repo's reliability so far:** every piece of the `.h5`
schema in `calibrateRadarNoise.m` — dataset naming, point-cloud layout, and
a real timestamp-matching bug — has had to be corrected against actual
error messages and diagnostic output from a real file, because none of this
could be executed in the environment it was originally written in (no
MATLAB, no ColoRadar data). It's converged now (see the "Confirmed against
real data" section below for exactly what's been verified), but if you hit
another wall, the fastest path is pasting the actual MATLAB error/output
back rather than re-guessing.

## 1. Find your topic names (bag path) or inspect the schema (h5 path)

**From a `.bag`:**
```matlab
inspectRosbag('2_23_2021_edgar_army_run5.bag')
```
Look for the radar `PointCloud2` topic, the lidar `PointCloud2` topic, and
one or two pose/ground-truth topics (`nav_msgs/Odometry`,
`geometry_msgs/PoseStamped`, `geometry_msgs/PoseWithCovarianceStamped`, or
`geometry_msgs/TransformStamped`). Check the radar cloud's actual field
names too:
```matlab
sel = select(bag, 'Topic', '<radar topic>');
msgs = readMessages(sel, 1);
rosReadAllFieldNames(msgs{1})
```

**From an exported `.h5`:**
```matlab
inspectRadarH5('dataset.h5')
```
As of the "confirmed against real data" section below, you likely won't
need to change anything — the schema in `calibrateRadarNoise.m` now matches
a real `coloradar-library` export.

## 2. Run calibration

**From a `.bag`:**
```matlab
calibration = calibrateRadarNoiseFromBag('2_23_2021_edgar_army_run5.bag', ...
    'RadarTopic', '/cascade/point_cloud', ...          % <- replace with what you found
    'LidarTopic', '/os1_cloud_node/points', ...         % <- replace with what you found
    'RadarPoseTopic', '/lidar_ground_truth', ...        % <- replace with what you found
    'LidarPoseTopic', '/lidar_ground_truth', ...        % <- replace with what you found
    'OutFile', 'radar_noise_calibration.json');
```

**From a `.h5`:**
```matlab
calibration = calibrateRadarNoise('./data/dataset.h5', ...
    'Runs', {'2_23_2021_edgar_classroom_run4'}, ...
    'FrameStride', 10, ...                              % fast first pass; drop once it works
    'OutFile', 'radar_noise_calibration.json');
```

Both require MATLAB ROS Toolbox only for the `.bag` path (`rosbag`/
`select`/`readMessages`/`rosReadXYZ`/`rosReadField`); the `.h5` path is base
MATLAB only. Watch the Command Window output — on a successful run you
should see a nonzero "radar frames accumulated" count and, at the end,
nonzero counts for true detections / clutter points / doppler residuals.

## 3. Use it in simulation

`radar_noise_model.py` auto-detects `.json` vs `.yaml` by extension:

```python
from radar_noise_model import CalibratedRadarNoiseModel
import numpy as np

model = CalibratedRadarNoiseModel.from_file("radar_noise_calibration.json", seed=42)

# ideal_points: (N,5) array [x, y, z, rcs, true_radial_velocity] in the
# radar's sensor frame, from your noise-free ray-cast against the sim world
noisy_points = model.apply(ideal_points)  # (M,5), M != N due to dropout+clutter
```

For Gazebo/ROS 2: put a plugin or node on the simulated-drone side that
ray-casts against the world and publishes "perfect" detections as a
`PointCloud2` with fields `x,y,z,rcs,velocity`, then run:

```bash
python radar_noise_model.py --ros-args \
    -p calibration_yaml:=radar_noise_calibration.json \
    -p input_topic:=ideal_radar_points \
    -p output_topic:=radar/points
```

(the `calibration_yaml` parameter name is a holdover from an earlier
version of this pipeline — it takes a plain file path and works fine with
the MATLAB `.json`. Requires a sourced ROS 2 environment with
`sensor_msgs_py` installed.)

This two-stage design (ideal ray-cast → calibrated noise node) exists
because Gazebo's built-in `<noise type="gaussian">` tag can only express a
single fixed std-dev per measurement — it can't express range-dependent
dropout probability or a clutter/false-alarm process.

## Confirmed against real data (the .h5 path)

Everything in this section was wrong in an earlier draft and has since been
corrected against actual output from a real export:

**Dataset layout** is flat at the file root — `<prefix>_<field>_<runName>`,
not per-run groups. E.g. `cascade_poses_2_23_2021_edgar_classroom_run4`
(`7x816`), `cascade_timestamps_..._run4` (816), `lidar_poses_..._run4`
(`7x1639`), `lidar_timestamps_..._run4` (1639).

**Point clouds**: `cascade_clouds_<run>` is `5 x totalPoints` — every
frame's points concatenated column-wise, fields `[x,y,z,intensity,doppler]`
(confirmed: column 4 ranged into the hundreds of thousands — raw heatmap
power, not velocity; column 5 ranged ±~1.3 — a plausible radial velocity).
A companion `cascade_clouds_<run>_sizes` (one integer per frame) gives the
point count per frame; `cumsum` of that gives each frame's column offset.
Same pattern for `lidar_clouds_<run>` (`3 x totalPoints`, no intensity) and
`_sizes`. `calibrateRadarNoise.m` reads this via partial `h5read` calls
keyed off the sizes index, so it never loads the multi-GB full array.

**Scale**: the cascade radar's "clouds" are the dense, unthresholded
heatmap — confirmed ~210,672 points per frame, constant across frames
(intensity_threshold was 0 at export). That's not a sparse detected-target
point cloud; it's every range/azimuth/elevation/Doppler bin. Because of
this, `calibrateRadarNoise.m` keeps only the top `RadarTopKPerFrame`
(default 3000) points per frame by intensity before doing anything else —
without it, nearest-neighbor matching against lidar is both computationally
infeasible and conceptually meaningless (literal noise-floor bins would
count as "clutter"). Lidar checks out as expected: 65,536 points/frame,
exactly the Ouster OS1 spec.

**Poses** are `[x,y,z,qx,qy,qz,qw]`, confirmed by checking that the
last-4-as-quaternion grouping has unit norm (the reverse grouping doesn't).

**Bug found and fixed**: matching a radar frame to a lidar frame originally
used "find the last lidar timestamp ≤ this radar timestamp" instead of a
true nearest-neighbor search. Lidar runs at ~10Hz and radar at ~5Hz, both
free-running (no hardware sync) — with a "look backward only" search,
roughly half the time the actually-closest lidar frame is a few
milliseconds *after* the radar timestamp, so the code would grab a stale
frame up to 100ms old, blowing past the 50ms sync window and silently
dropping the frame. Symptom: `[ok] <run>: 0 radar frames accumulated`, no
errors. Fixed by reusing the same true-nearest-neighbor logic that was
already correctly used for pose lookups (`nearestPoseIndex`, now also used
for the radar↔lidar frame match). This fix is in both `calibrateRadarNoise.m`
and `calibrateRadarNoiseFromBag.m`; it has NOT been ported to
`radar_calibration.py` (Python) yet.

**Still unverified**: whether `MaxTimeSyncS` (50ms) is a good threshold now
that matching is correct — with the fix, the true nearest-neighbor gap for
a 10Hz lidar should never exceed 50ms (half the 100ms period), so it should
be fine, but confirm the "true detections" count in your run's output looks
reasonable (order of magnitude: thousands, given `RadarTopKPerFrame=3000`
per processed frame) rather than suspiciously small.

**Bug found and fixed (#2): near-field bias in top-K selection.** A first
real calibration run produced implausible output: azimuth std ~67.6°,
elevation std ~22.2°, detection probability dropping to exactly 0 beyond
~4-9m, clutter concentrated almost entirely within 4m, and empty `bins: []`
for every range-dependent fit. Root cause: `RadarTopKPerFrame` was ranking
the ~210k raw heatmap points per frame by *raw* intensity. Received radar
power falls off as 1/r^4 for a point target, so ranking by raw intensity
structurally favors near-field returns — antenna leakage and direct-path
coupling close to the sensor — over genuine far-range detections that are
only dimmer because they're far away. The result: top-K was picking almost
entirely near-field noise-floor bins, not real detections, which is exactly
the "everything breaks down past ~5m" pattern in the output. Fixed by (1)
adding a `MinRangeM` cutoff (default 0.5m) that drops points closer than
that before ranking at all, and (2) ranking the rest by
`intensity .* r^RangeCompensationExponent` (default exponent 4) instead of
raw intensity, so top-K approximates SNR-above-clutter rather than raw
range. Applied identically to `readCloudFlat` in `calibrateRadarNoise.m`
and `readCloudTopic` in `calibrateRadarNoiseFromBag.m`. **This fix has not
been run against real data yet** — if you rerun and the azimuth/elevation
stds are still in the tens-of-degrees range, or detection probability still
cliffs to 0 at a similarly short range, paste the new output back rather
than assuming the fix is complete; it may mean `RangeCompensationExponent`
needs tuning, or that `RadarTopKPerFrame` needs to be raised so enough
far-range points survive into the top-K at all.

## Caveats you should keep in mind

- **Ground rig, not a drone.** The elevation-noise and dropout curves
  reflect what a wheeled rig at roughly constant height saw. A drone flying
  near a tunnel ceiling, banking through turns, or looking straight down a
  shaft will hit different multipath geometry and grazing angles than
  anything in ColoRadar's trajectories. The range/Doppler noise and the
  SNR-driven detection roll-off (governed by the radar chip, not the
  platform) should transfer reasonably well; the *spatial pattern* of
  clutter and any elevation-dependent bias should be treated as a rough
  starting point, not ground truth, for a flying platform.
- **No dust/fog/smoke labels.** ColoRadar has none, so everything calibrated
  here is a clear-air noise model.
- **The nearest-neighbor lidar/radar association is a proxy for ground
  truth**, not a perfect one — sanity-check the fitted curves against the
  raw per-bin numbers in the output file before trusting them.
- **RadarTopKPerFrame is a modeling choice, not a measurement** — it stands
  in for whatever detection threshold your actual drone radar's onboard
  CFAR applies. Tune it (or extend the script to threshold by an absolute
  intensity value instead of top-K) to better match your real hardware if
  you know its detection budget.
- **None of this has been executed by me** — I don't have a MATLAB license,
  ROS Toolbox, or ColoRadar data in the environment I work in. Everything
  above was corrected iteratively against your real error messages and
  diagnostic output, not verified independently beforehand. Keep pasting
  back what you actually see rather than assuming a fix is complete.
