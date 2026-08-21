function calibration = calibrateRadarNoise(h5Path, varargin)
%CALIBRATERADARNOISE  Fit an FMCW radar noise model against ColoRadar.
%
%   calibration = calibrateRadarNoise(h5Path)
%   calibration = calibrateRadarNoise(h5Path, 'Device', 'cascade_radar', ...
%                                      'Runs', {'2_23_2021_edgar_army_run0'}, ...
%                                      'OutFile', 'radar_noise_calibration.json')
%
% NOTE: this expects an .h5 exported via arpg/coloradar-library's Docker
% tool. If your download is a raw .bag (which is how ColoRadar is actually
% distributed), use calibrateRadarNoiseFromBag.m instead -- it reads the
% .bag directly via MATLAB ROS Toolbox and produces the same output schema.
%
% MATLAB port of radar_calibration.py. Same algorithm, same output schema
% (as a MATLAB struct, exported to JSON so it plugs directly into the
% existing Python `radar_noise_model.py` -> ROS 2 node used inside
% Gazebo). The dataset-naming convention below was confirmed against a real
% export (flat "<prefix>_<field>_<runName>" datasets at the file root) --
% see the comment block above the `devicePrefix` line for exactly what was
% confirmed. Run `inspectRadarH5(h5Path)` on your file if anything doesn't
% match.
%
% ALGORITHM
%   1. Load an exported ColoRadar .h5 (from arpg/coloradar-library). Per
%      run: radar point clouds + timestamps + world-frame poses, and the
%      same for lidar.
%   2. For each radar frame, find the temporally NEAREST lidar frame
%      (proper nearest-neighbor in time, not just "most recent past" --
%      see nearestPoseIndex, reused for this too), transform it into the
%      radar sensor frame using the two independent world-frame pose
%      trajectories, and treat those points as ground truth surface
%      geometry.
%   3. Nearest-neighbor-associate every radar point to that lidar cloud:
%      within tolerance -> true detection (record range/az/el residual);
%      otherwise -> clutter/ghost point.
%   4. Voxelize the lidar scan into az/el/range cells; for each occupied
%      cell, check whether the radar produced any point in the matching
%      cell (+/-1 range bin) -> detection-probability-vs-range curve.
%   5. Differentiate consecutive radar poses for ego-velocity, project onto
%      each matched point's line-of-sight for the expected static-point
%      Doppler, and compare to the measured Doppler -> Doppler noise.
%   6. Fit range-dependent models (std(r) = a + b*r; logistic detection
%      probability) and write everything to JSON (+ optionally .mat).
%
% NAME-VALUE ARGUMENTS
%   Device                 (default 'cascade_radar')
%   Runs                   cell array of run names, default {} = all runs
%   OutFile                (default 'radar_noise_calibration.json')
%   MatOutFile              (default '' = skip .mat export)
%   MaxTimeSyncS           (default 0.05)
%   MatchDistM             (default 0.5)
%   DropoutAzBinDeg        (default 4.0)
%   DropoutElBinDeg        (default 4.0)
%   DropoutRangeBinM       (default 0.5)
%   MaxRangeM              (default 8.0) -- IMPORTANT: this is NOT an
%                          arbitrary cutoff. The cascade radar's heatmap
%                          has a fixed number of range bins (num_range_bins
%                          = 128) of fixed width (range_bin_width ~=
%                          0.0593 m), read from calib.zip in ARPG's own
%                          raw-format reader script -- 128*0.0593 =~ 7.59m
%                          unambiguous max range. Past that, there is no
%                          radar data at all; it's not a detection that
%                          failed, there's nothing to detect. An earlier
%                          version of this script used MaxRangeM=60 (an
%                          unverified guess) which made detection
%                          probability and clutter look like they cliffed
%                          to exactly zero past ~9-11m -- that wasn't a
%                          matching bug, it was 50+ meters of bins that
%                          could never contain radar data by construction.
%                          If your calib.zip has different range_bin_width
%                          / num_range_bins values, recompute this.
%   FovAzimuthDeg          (default 120.0)
%   FovElevationDeg        (default 30.0)
%   MaxLidarPointsPerFrame random-downsample lidar clouds above this count
%                          before nearest-neighbor search, for speed/memory
%                          (default 10000; set Inf to disable)
%   RadarTopKPerFrame      keep only the RadarTopKPerFrame highest-intensity
%                          radar points per frame before association
%                          (default 3000; set Inf to disable). IMPORTANT:
%                          the cascade radar's exported "clouds" are the
%                          dense, unthresholded heatmap (confirmed against a
%                          real file: ~211k points/frame, since
%                          intensity_threshold was 0 at export time), not
%                          sparse CFAR-style detections. Using all of them
%                          would both be computationally infeasible for
%                          nearest-neighbor matching and would misrepresent
%                          "clutter" as literally every noise-floor bin.
%                          This keeps the top-K by intensity per frame as a
%                          stand-in for an onboard detection threshold.
%                          Ranking is RANGE-COMPENSATED (see MinRangeM /
%                          RangeCompensationExponent below), not raw
%                          intensity -- raw intensity falls off as 1/r^4,
%                          which otherwise biases top-K toward near-field
%                          antenna leakage instead of real far detections.
%   MinRangeM              drop radar points closer than this before
%                          top-K ranking, to exclude antenna-leakage /
%                          direct-path near-field artifacts (default 0.5)
%   RangeCompensationExponent  rank radar points by intensity*r^this
%                          instead of raw intensity before top-K, to
%                          counteract 1/r^4 power falloff (default 4)
%   MinIntensity           absolute raw-intensity floor, applied BEFORE
%                          range compensation (default 0 = disabled).
%                          RangeCompensationExponent fixes the near-field
%                          bias but can overcorrect: it amplifies the
%                          background noise floor by r^exponent just as
%                          much as real echoes, so at long range pure
%                          noise can be inflated into outranking genuine
%                          but weak far detections. MinIntensity removes
%                          obviously-noise-floor points first so they
%                          never get amplified in the first place. There
%                          is no universal correct value -- "intensity" is
%                          an uncalibrated linear magnitude specific to
%                          this radar chip/gain setting, not dB or a
%                          physical unit (see the ColoRadar paper: it's
%                          the peak-of-Doppler-spectrum magnitude per
%                          range/azimuth/elevation bin). Use
%                          inspectRadarIntensity.m to look at the actual
%                          distribution in your file before picking one.
%   FrameStride            process every Nth radar frame (default 1 = all).
%                          Use e.g. 10 for a fast first pass before
%                          committing to a full run.
%
% See README_radar_calibration.md for the full workflow (exporting the .h5
% with arpg/coloradar-library, then wiring the resulting calibration file
% into radar_noise_model.py inside Gazebo/ROS 2).

    p = inputParser;
    addRequired(p, 'h5Path', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Device', 'cascade_radar', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Runs', {}, @iscell);
    addParameter(p, 'OutFile', 'radar_noise_calibration.json', @(x) ischar(x) || isstring(x));
    addParameter(p, 'MatOutFile', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'MaxTimeSyncS', 0.05, @isnumeric);
    addParameter(p, 'MatchDistM', 0.5, @isnumeric);
    addParameter(p, 'DropoutAzBinDeg', 4.0, @isnumeric);   % ~2 sigma of measured az residual
    addParameter(p, 'DropoutElBinDeg', 4.0,  @isnumeric);   % ~2 sigma of measured el residual
    addParameter(p, 'DropoutRangeBinM', 0.5, @isnumeric);
    addParameter(p, 'MaxRangeM', 8.0, @isnumeric);
    addParameter(p, 'FovAzimuthDeg', 156.6, @isnumeric);
    addParameter(p, 'FovElevationDeg', 146.0, @isnumeric);
    addParameter(p, 'MaxLidarPointsPerFrame', Inf, @isnumeric);
    addParameter(p, 'RadarTopKPerFrame', 9000, @isnumeric);
    addParameter(p, 'MinRangeM', 0.5, @isnumeric);
    addParameter(p, 'RangeCompensationExponent', 4, @isnumeric);
    addParameter(p, 'MinIntensity', 0, @isnumeric);
    addParameter(p, 'FrameStride', 10, @isnumeric);
    parse(p, h5Path, varargin{:});
    opt = p.Results;

    % ---- naming convention, fully confirmed against a real export ---------
    % The .h5 stores everything FLAT at the file root as
    % "<prefix>_<field>_<runName>" (no per-run groups):
    %   cascade_poses_<run>            7 x nFrames    [x,y,z,qx,qy,qz,qw] per frame
    %   cascade_timestamps_<run>       nFrames
    %   lidar_poses_<run>              7 x nFrames
    %   lidar_timestamps_<run>         nFrames
    %   cascade_clouds_<run>           5 x totalPoints  ALL frames' points
    %                                  concatenated column-wise, fields
    %                                  [x,y,z,intensity,doppler] -- CONFIRMED
    %                                  via real data: column 4 ranged into
    %                                  the hundreds of thousands (raw power,
    %                                  not physically-plausible velocity),
    %                                  column 5 ranged +/-~1.3 (a plausible
    %                                  radial velocity in m/s).
    %   cascade_clouds_<run>_sizes     nFrames -- point count per frame
    %                                  (confirmed constant: ~210672 every
    %                                  frame, i.e. a fixed-resolution
    %                                  heatmap grid, not a variable-count
    %                                  sparse detection list); cumsum of
    %                                  this gives each frame's column offset
    %                                  in the flat clouds array above
    %   lidar_clouds_<run>             3 x totalPoints  [x,y,z], same
    %                                  concatenated-with-sizes-index layout
    %                                  (confirmed: totalPoints / nFrames =
    %                                  65536, exactly the Ouster OS1 spec)
    %   lidar_clouds_<run>_sizes       nFrames
    % Because cascade_clouds is the DENSE, UNTHRESHOLDED heatmap, loading it
    % via partial reads keyed off the _sizes index (readCloudFlat below) and
    % then keeping only the top RadarTopKPerFrame points by intensity is
    % essential, not optional -- both for memory (the full array is several
    % GB) and because comparing every unthresholded noise-floor bin against
    % lidar would make the clutter/dropout statistics meaningless.
    devicePrefix = deviceToPrefix(opt.Device);
    radarPointCols = {'x', 'y', 'z', 'intensity', 'doppler'}; %#ok<NASGU>
    lidarPointCols = {'x', 'y', 'z'}; %#ok<NASGU>

    if isempty(opt.Runs)
        runs = discoverRuns(opt.h5Path);
    else
        runs = opt.Runs;
    end
    fprintf('Processing %d run(s):\n', numel(runs));
    fprintf('  %s\n', strjoin(runs, ', '));

    acc = initAccumulators(opt);

    for i = 1:numel(runs)
        runName = runs{i};
        try
            run = loadRun(opt.h5Path, devicePrefix, runName, opt);
        catch ME
            fprintf('  [skip] %s: %s\n', runName, ME.message);
            continue
        end
        acc = processRun(run, acc, opt);
        fprintf('  [ok] %s: %d radar frames accumulated so far\n', runName, acc.nRadarFrames);
        fprintf(['         diagnostics (cumulative across runs so far): ' ...
            '%d total frames seen | dropped: %d empty-radar-after-filter, ' ...
            '%d sync-fail(>%.3fs), %d empty-lidar-frame, %d no-lidar-in-range(%.0fm) ' ...
            '| %d radar pts seen, %d matched (%.2f%% match rate)\n'], ...
            acc.dbgTotalFrames, acc.dbgEmptyRadarPts, acc.dbgSyncFail, opt.MaxTimeSyncS, ...
            acc.dbgEmptyLidarFrame, acc.dbgNoLidarInRange, opt.MaxRangeM, ...
            acc.dbgTotalRadarPtsSeen, acc.dbgTotalMatchedPts, ...
            100 * acc.dbgTotalMatchedPts / max(acc.dbgTotalRadarPtsSeen, 1));
    end

    if acc.nRadarFrames == 0
        error('calibrateRadarNoise:noData', ...
            'No radar frames were processed -- check that the guessed cloud dataset names in loadRun() match inspectRadarH5(h5Path) output.');
    end

    calibration = buildCalibrationStruct(acc, opt, h5Path);

    jsonStr = jsonencode(calibration, 'PrettyPrint', true);
    fid = fopen(opt.OutFile, 'w');
    fwrite(fid, jsonStr, 'char');
    fclose(fid);
    fprintf('\nWrote calibration to %s\n', opt.OutFile);

    if ~isempty(opt.MatOutFile)
        save(opt.MatOutFile, 'calibration');
        fprintf('Wrote calibration to %s\n', opt.MatOutFile);
    end

    fprintf('  true detections used for residuals: %d\n', numel(acc.rangeResid));
    fprintf('  clutter points: %d\n', numel(acc.clutterRanges));
    fprintf('  doppler residuals: %d\n', numel(acc.dopplerResid));

    far = acc.residRangeBin > 4;
    fprintf('az std (r>4m): %.2f deg | el std: %.2f deg | n=%d\n', ...
    rad2deg(std(acc.azResid(far))), rad2deg(std(acc.elResid(far))), nnz(far));
end


% =========================================================================
% Discovery / loading
% =========================================================================
function prefix = deviceToPrefix(device)
    % 'cascade_radar' -> 'cascade', 'single_chip_radar' -> 'single_chip',
    % otherwise passed through unchanged (in case you already know the
    % exact dataset-name prefix).
    switch device
        case 'cascade_radar'
            prefix = 'cascade';
        case 'single_chip_radar'
            prefix = 'single_chip';
        otherwise
            prefix = device;
    end
end

function runs = discoverRuns(h5Path)
    % Runs are discovered from "lidar_poses_<runName>" dataset names at the
    % file root (every run has lidar data, so this is the most reliable
    % anchor). Adjust the regexp if your file uses a different lidar
    % dataset prefix -- check inspectRadarH5(h5Path) output.
    info = h5info(h5Path);
    runs = {};
    if ~isfield(info, 'Datasets')
        return
    end
    names = {info.Datasets.Name};
    for i = 1:numel(names)
        tok = regexp(names{i}, '^lidar_poses_(.+)$', 'tokens', 'once');
        if ~isempty(tok)
            runs{end+1} = tok{1}; %#ok<AGROW>
        end
    end
end

function path = findExistingDataset(h5Path, candidates)
    % Try each candidate dataset path in order, return the first that
    % actually exists in the file. Errors listing everything it tried if
    % none match -- add the real name here once you know it.
    for i = 1:numel(candidates)
        try
            h5info(h5Path, candidates{i});
            path = candidates{i};
            return
        catch
            % try next candidate
        end
    end
    error('calibrateRadarNoise:datasetNotFound', ...
        'None of these datasets exist: %s -- run inspectRadarH5(h5Path) to find the real name and add it to the candidate list.', ...
        strjoin(candidates, ', '));
end

function run = loadRun(h5Path, devicePrefix, runName, opt)
    radarPosesPath = ['/' devicePrefix '_poses_' runName];
    radarTPath     = ['/' devicePrefix '_timestamps_' runName];
    lidarPosesPath = ['/lidar_poses_' runName];
    lidarTPath     = ['/lidar_timestamps_' runName];

    % confirmed pattern first, a couple of fallbacks in case a different
    % run/device uses a slightly different suffix
    radarCloudsPath = findExistingDataset(h5Path, { ...
        ['/' devicePrefix '_clouds_' runName], ...
        ['/' devicePrefix '_cloud_' runName]});
    lidarCloudsPath = findExistingDataset(h5Path, { ...
        ['/lidar_clouds_' runName], ...
        ['/lidar_cloud_' runName]});
    radarSizesPath = [radarCloudsPath '_sizes'];
    lidarSizesPath = [lidarCloudsPath '_sizes'];

    run.name = runName;
    run.radarFrames = readCloudFlat(h5Path, radarCloudsPath, radarSizesPath, ...
        5, opt.RadarTopKPerFrame, opt.FrameStride, true, opt.MinRangeM, ...
        opt.RangeCompensationExponent, opt.MinIntensity);
    run.radarT = double(h5read(h5Path, radarTPath));
    run.radarT = run.radarT(:);
    run.radarPoses = double(h5read(h5Path, radarPosesPath));
    run.radarPoses = reshapePoses(run.radarPoses);
    run.radarPoseT = matchPoseTimestamps(run.radarT, run.radarPoses);

    run.lidarFrames = readCloudFlat(h5Path, lidarCloudsPath, lidarSizesPath, ...
        3, opt.MaxLidarPointsPerFrame, 1, false, 0, 0, 0);
    run.lidarT = double(h5read(h5Path, lidarTPath));
    run.lidarT = run.lidarT(:);
    run.lidarPoses = double(h5read(h5Path, lidarPosesPath));
    run.lidarPoses = reshapePoses(run.lidarPoses);
    run.lidarPoseT = matchPoseTimestamps(run.lidarT, run.lidarPoses);
end

function frames = readCloudFlat(h5Path, cloudsPath, sizesPath, numFields, topK, frameStride, byIntensity, minRangeM, rangeCompExp, minIntensity)
    % Reads a "<...>_clouds_<run>" dataset (numFields x totalPoints, every
    % frame's points concatenated column-wise) together with its companion
    % "<...>_clouds_<run>_sizes" dataset (per-frame point count), using
    % partial h5read calls keyed off cumsum(sizes) so we never load the
    % full multi-GB array into memory at once.
    %
    % Radar top-K selection (byIntensity=true) ranks by RANGE-COMPENSATED
    % intensity, intensity .* max(r,0.1)^rangeCompExp, not raw intensity.
    % Raw received power falls off as 1/r^4 for a point target, so ranking
    % by raw intensity structurally biases the "top K per frame" selection
    % toward near-field returns (antenna leakage / direct-path coupling)
    % regardless of whether anything real is out there, while discarding
    % genuine far-range detections that are dimmer only because they're
    % far away. Multiplying by r^rangeCompExp before ranking approximately
    % undoes that falloff so top-K reflects SNR-above-clutter rather than
    % raw range. minRangeM additionally drops points closer than that
    % (default 0.5 m) before ranking, to exclude antenna-leakage/near-field
    % artifacts entirely rather than just downweighting them.
    %
    % minIntensity is a SEPARATE, ABSOLUTE raw-intensity floor applied
    % BEFORE range compensation. Range compensation undoes 1/r^4 falloff
    % for real echoes, but it does the same thing to the background noise
    % floor, which does NOT fall off with range -- so at long range,
    % r^rangeCompExp can inflate pure noise into out-ranking genuine (but
    % weak) far detections. minIntensity drops points below a raw-intensity
    % cutoff first, before that multiplication ever happens, so noise-floor
    % bins can't get amplified into the running at all. There's no
    % universal correct value for this (it's an uncalibrated linear
    % magnitude specific to this radar chip/gain setting, not dB or a
    % physical unit) -- use inspectRadarIntensity.m to look at the real
    % distribution in your file before picking one.
    sizes = double(h5read(h5Path, sizesPath));
    sizes = sizes(:);
    nFrames = numel(sizes);
    starts = [0; cumsum(sizes(1:end-1))]; % 0-indexed column offset before each frame

    frames = cell(nFrames, 1);
    for i = 1:nFrames
        frames{i} = zeros(0, numFields);
    end

    for i = 1:frameStride:nFrames
        n = sizes(i);
        if n == 0
            continue
        end
        raw = h5read(h5Path, cloudsPath, [1, starts(i) + 1], [numFields, n]); % numFields x n
        pts = double(raw)'; % -> n x numFields

        if byIntensity && numFields >= 4 && minRangeM > 0
            r  = sqrt(sum(pts(:, 1:3).^2, 2));
                rb = floor(r / 0.25);
                score = zeros(size(r));
                ub = unique(rb);
                for bi = 1:numel(ub)
                    m   = rb == ub(bi);
                    v   = pts(m, 4);
                    med = median(v);
                    mad = median(abs(v - med)) * 1.4826;
                    score(m) = (v - med) / max(mad, eps);
                end
                [~, order] = sort(score, 'descend');
                pts = pts(order(1:topK), :);
        end

        if byIntensity && numFields >= 4 && minIntensity > 0
            pts = pts(pts(:, 4) >= minIntensity, :);
        end

        if isfinite(topK) && size(pts, 1) > topK
            if byIntensity && numFields >= 4
                r = sqrt(sum(pts(:, 1:3).^2, 2));
                rankScore = pts(:, 4) .* max(r, 0.1).^rangeCompExp;
                [~, order] = sort(rankScore, 'descend');
                pts = pts(order(1:topK), :);
            else
                keepIdx = randperm(size(pts, 1), topK);
                pts = pts(keepIdx, :);
            end
        end
        frames{i} = pts;
    end
end

function poses = reshapePoses(raw)
    % h5read returns data with dimensions possibly transposed relative to
    % how it was written by h5py (row-major vs column-major). Normalize to
    % an Nx7 array [x,y,z,qx,qy,qz,qw]. (Confirmed correct against real data:
    % first-3-as-position/last-4-as-quaternion gives a unit quaternion; the
    % reverse grouping does not.)
    if size(raw, 2) == 7
        poses = raw;
    elseif size(raw, 1) == 7
        poses = raw';
    else
        error('calibrateRadarNoise:badPoseShape', ...
            'Expected a Nx7 or 7xN pose array, got %s -- check export config (need x,y,z,qx,qy,qz,qw).', ...
            mat2str(size(raw)));
    end
end

function t = matchPoseTimestamps(frameT, poses)
    n = size(poses, 1);
    if n == numel(frameT)
        t = frameT;
    else
        t = frameT(1:min(n, numel(frameT)));
    end
end

% (readCloudGroup / ensurePointsByColumns / natsort from the earlier guess
% -- a per-run-group or vlen-dataset layout -- were removed: the confirmed
% real layout is the flat-array-plus-sizes-index format handled by
% readCloudFlat() above.)


% =========================================================================
% Geometry
% =========================================================================
function R = quatToRot(q)
    % q = [qx qy qz qw]
    x = q(1); y = q(2); z = q(3); w = q(4);
    n = x*x + y*y + z*z + w*w;
    if n < 1e-12
        R = eye(3);
        return
    end
    s = 2.0 / n;
    X = x*s; Y = y*s; Z = z*s;
    wx = w*X; wy = w*Y; wz = w*Z;
    xx = x*X; xy = x*Y; xz = x*Z;
    yy = y*Y; yz = y*Z; zz = z*Z;
    R = [1-(yy+zz), xy-wz,     xz+wy; ...
         xy+wz,     1-(xx+zz), yz-wx; ...
         xz-wy,     yz+wx,     1-(xx+yy)];
end

function T = poseToT(poseVec)
    T = eye(4);
    T(1:3, 1:3) = quatToRot(poseVec(4:7));
    T(1:3, 4) = poseVec(1:3);
end

function idx = nearestPoseIndex(timestamps, t)
    % TRUE nearest-neighbor search in time: finds the last timestamp <= t,
    % then compares it against the following one and picks whichever is
    % actually closer. (This general-purpose search is reused for matching
    % radar frames to lidar frames too, not just for pose lookups -- an
    % earlier version of processRun used a "last <= t" only search for that,
    % which systematically picked a stale lidar frame roughly half the time
    % given lidar's ~10Hz rate vs. the sync tolerance, and dropped every
    % frame as a result. Fixed by reusing this function everywhere.)
    idx = find(timestamps <= t, 1, 'last');
    if isempty(idx)
        idx = 1;
    elseif idx < numel(timestamps)
        if abs(timestamps(idx+1) - t) < abs(timestamps(idx) - t)
            idx = idx + 1;
        end
    end
end

function [r, az, el] = cartToSph(xyz)
    [az, el, r] = cart2sph(xyz(:,1), xyz(:,2), xyz(:,3)); % MATLAB built-in matches our convention
end

function xyz = sphToCart(r, az, el)
    [x, y, z] = sph2cart(az, el, r);
    xyz = [x(:), y(:), z(:)];
end


% =========================================================================
% Accumulation
% =========================================================================
function acc = initAccumulators(opt)
    acc.rangeResid = [];
    acc.azResid = [];
    acc.elResid = [];
    acc.residRangeBin = [];
    acc.clutterRanges = [];
    acc.nRadarFrames = 0;
    acc.occTotal = containers.Map('KeyType', 'double', 'ValueType', 'double');
    acc.occDetected = containers.Map('KeyType', 'double', 'ValueType', 'double');
    acc.dopplerResid = [];
    acc.dopplerResidRangeBin = [];
    acc.intensityByBin = containers.Map('KeyType', 'double', 'ValueType', 'any');
    acc.fovSolidAngleSr = deg2rad(opt.FovAzimuthDeg) * deg2rad(opt.FovElevationDeg);

    % Diagnostic counters: WHY frames get dropped in processRun, so a low
    % nRadarFrames-vs-total-radar-frames ratio can be root-caused instead of
    % guessed at. Printed as a per-run breakdown at the end of each run.
    acc.dbgTotalFrames = 0;
    acc.dbgEmptyRadarPts = 0;
    acc.dbgSyncFail = 0;
    acc.dbgEmptyLidarFrame = 0;
    acc.dbgNoLidarInRange = 0;
    acc.dbgTotalMatchedPts = 0;
    acc.dbgTotalRadarPtsSeen = 0;
end

function b = rangeBinOf(r, opt)
    nBinsMax = floor(opt.MaxRangeM / opt.DropoutRangeBinM) - 1;
    b = min(max(floor(r / opt.DropoutRangeBinM), 0), nBinsMax);
end

function acc = processRun(run, acc, opt)
    nFrames = min(numel(run.radarFrames), numel(run.radarT));
    for i = 1:nFrames
        acc.dbgTotalFrames = acc.dbgTotalFrames + 1;
        tR = run.radarT(i);
        radarPts = run.radarFrames{i};
        if isempty(radarPts)
            acc.dbgEmptyRadarPts = acc.dbgEmptyRadarPts + 1;
            continue
        end
        acc.dbgTotalRadarPtsSeen = acc.dbgTotalRadarPtsSeen + size(radarPts, 1);

        % TRUE nearest lidar frame in time (see nearestPoseIndex comment --
        % this used to be a "last <= tR" only search, which was the bug
        % that dropped every frame).
        j = nearestPoseIndex(run.lidarT, tR);
        if j > numel(run.lidarFrames) || abs(run.lidarT(j) - tR) > opt.MaxTimeSyncS
            acc.dbgSyncFail = acc.dbgSyncFail + 1;
            continue
        end
        lidarPts = run.lidarFrames{j};
        if isempty(lidarPts)
            acc.dbgEmptyLidarFrame = acc.dbgEmptyLidarFrame + 1;
            continue
        end

        % --- lidar scan -> radar sensor frame at time tR ---
        pi_ = nearestPoseIndex(run.radarPoseT, tR);
        pj_ = nearestPoseIndex(run.lidarPoseT, run.lidarT(j));
        Twr = poseToT(run.radarPoses(pi_, :));
        Twl = poseToT(run.lidarPoses(pj_, :));
        Trl = Twr \ Twl; % inv(Twr) * Twl
        % disp(Trl);

        lidarXyz = lidarPts(:, 1:3);
        if size(lidarXyz, 1) > opt.MaxLidarPointsPerFrame
            keepIdx = randperm(size(lidarXyz, 1), opt.MaxLidarPointsPerFrame);
            lidarXyz = lidarXyz(keepIdx, :);
        end
        lidarH = [lidarXyz, ones(size(lidarXyz, 1), 1)];
        lidarInRadar = (Trl * lidarH')';
        lidarInRadar = lidarInRadar(:, 1:3);

        rL = sqrt(sum(lidarInRadar.^2, 2));
        azLim = deg2rad(opt.FovAzimuthDeg) / 2;
        elLim = deg2rad(opt.FovElevationDeg) / 2;
        [rL, azL0, elL0] = cartToSph(lidarInRadar);
        inFov = rL < opt.MaxRangeM & abs(azL0) <= azLim & abs(elL0) <= elLim;
        lidarInRadar = lidarInRadar(inFov, :);
        if isempty(lidarInRadar)
            acc.dbgNoLidarInRange = acc.dbgNoLidarInRange + 1;
            continue
        end

        radarXyz = radarPts(:, 1:3);
        [radarRange, radarAz, radarEl] = cartToSph(radarXyz);

        [nnDist, nnIdx] = nearestNeighborBruteForce(radarXyz, lidarInRadar);
        isMatch = nnDist < opt.MatchDistM;
        acc.dbgTotalMatchedPts = acc.dbgTotalMatchedPts + nnz(isMatch);

        % (a) localization residuals
        matchedTruth = lidarInRadar(nnIdx(isMatch), :);
        [tRange, tAz, tEl] = cartToSph(matchedTruth);
        acc.rangeResid = [acc.rangeResid; radarRange(isMatch) - tRange]; %#ok<AGROW>
        acc.azResid = [acc.azResid; radarAz(isMatch) - tAz]; %#ok<AGROW>
        acc.elResid = [acc.elResid; radarEl(isMatch) - tEl]; %#ok<AGROW>
        acc.residRangeBin = [acc.residRangeBin; tRange]; %#ok<AGROW>

        % (b) intensity/RCS-proxy by range bin
        if size(radarPts, 2) > 3
            intens = radarPts(isMatch, 4);
            for k = 1:numel(tRange)
                b = rangeBinOf(tRange(k), opt);
                if isKey(acc.intensityByBin, b)
                    acc.intensityByBin(b) = [acc.intensityByBin(b); intens(k)];
                else
                    acc.intensityByBin(b) = intens(k);
                end
            end
        end

        % (c) clutter
        acc.clutterRanges = [acc.clutterRanges; radarRange(~isMatch)]; %#ok<AGROW>
        acc.nRadarFrames = acc.nRadarFrames + 1;

        % (d) dropout: voxelize lidar into az/el/range cells, check radar support
        [rL2, azL, elL] = cartToSph(lidarInRadar);
        azBin = floor(rad2deg(azL) / opt.DropoutAzBinDeg);
        elBin = floor(rad2deg(elL) / opt.DropoutElBinDeg);
        rngBin = floor(rL2 / opt.DropoutRangeBinM);
        cellKeys = uniqueRows([azBin, elBin, rngBin]);

        if ~isempty(radarXyz)
            rAzBin = floor(rad2deg(radarAz) / opt.DropoutAzBinDeg);
            rElBin = floor(rad2deg(radarEl) / opt.DropoutElBinDeg);
            rRngBin = floor(radarRange / opt.DropoutRangeBinM);
            radarCellKeys = uniqueRows([rAzBin, rElBin, rRngBin]);
        else
            radarCellKeys = zeros(0, 3);
        end

        for k = 1:size(cellKeys, 1)
            ab = cellKeys(k, 1); eb = cellKeys(k, 2); rb = cellKeys(k, 3);
            key = min(max(rb, 0), floor(opt.MaxRangeM / opt.DropoutRangeBinM) - 1);
            acc.occTotal(key) = getOr(acc.occTotal, key, 0) + 1;
            hit = any(abs(radarCellKeys(:,1) - ab) <= 1 & abs(radarCellKeys(:,2) - eb) <= 1 & abs(radarCellKeys(:,3) - rb) <= 1);
            if hit
                acc.occDetected(key) = getOr(acc.occDetected, key, 0) + 1;
            end
        end

        % (e) Doppler residual via finite-difference ego-velocity
        if size(radarPts, 2) > 4 && i > 1 && i < nFrames
            tPrev = run.radarT(i-1); tNext = run.radarT(i+1);
            dt = tNext - tPrev;
            if dt > 1e-3
                pPrev = poseToT(run.radarPoses(nearestPoseIndex(run.radarPoseT, tPrev), :));
                pNext = poseToT(run.radarPoses(nearestPoseIndex(run.radarPoseT, tNext), :));
                vWorld = (pNext(1:3,4) - pPrev(1:3,4)) / dt;
                vRadar = Twr(1:3,1:3)' * vWorld;

                matchedRadarXyz = radarXyz(isMatch, :);
                los = matchedRadarXyz ./ max(sqrt(sum(matchedRadarXyz.^2, 2)), 1e-9);
                expectedDoppler = -(los * vRadar);
                measuredDoppler = radarPts(isMatch, 5);
                resid = measuredDoppler - expectedDoppler;
                acc.dopplerResid = [acc.dopplerResid; resid]; %#ok<AGROW>
                acc.dopplerResidRangeBin = [acc.dopplerResidRangeBin; tRange]; %#ok<AGROW>
            end
        end
    end
end

function v = getOr(map, key, default)
    if isKey(map, key)
        v = map(key);
    else
        v = default;
    end
end

function u = uniqueRows(m)
    if isempty(m)
        u = zeros(0, size(m, 2));
    else
        u = unique(m, 'rows');
    end
end

function [dist, idx] = nearestNeighborBruteForce(query, reference)
    % Squared-Euclidean nearest neighbor via the |a-b|^2 = |a|^2+|b|^2-2ab
    % expansion (single BLAS matrix multiply, no toolbox required). For
    % very large lidar clouds, lower MaxLidarPointsPerFrame instead of
    % relying on a KD-tree, to avoid a Statistics and Machine Learning
    % Toolbox dependency.
    q2 = sum(query.^2, 2);
    r2 = sum(reference.^2, 2)';
    d2 = q2 + r2 - 2 * (query * reference');
    d2(d2 < 0) = 0;
    [minD2, idx] = min(d2, [], 2);
    dist = sqrt(minD2);
end


% =========================================================================
% Fitting
% =========================================================================
function model = fitLinearStd(residuals, ranges, nBins, maxRange)
    if nargin < 3, nBins = 12; end
    if nargin < 4, maxRange = 60.0; end
    edges = linspace(0, maxRange, nBins + 1);
    centers = []; stds = []; counts = [];
    for i = 1:nBins
        m = ranges >= edges(i) & ranges < edges(i+1);
        if sum(m) < 20
            continue
        end
        centers(end+1) = (edges(i) + edges(i+1)) / 2; %#ok<AGROW>
        stds(end+1) = std(residuals(m)); %#ok<AGROW>
        counts(end+1) = sum(m); %#ok<AGROW>
    end
    if numel(centers) < 2
        gStd = 0.0;
        if ~isempty(residuals)
            gStd = std(residuals);
        end
        model.a = gStd;
        model.b = 0.0;
        model.bins = struct('range_m', {}, 'std', {}, 'n', {});
        return
    end
    coeffs = polyfit(centers, stds, 1); % [slope, intercept], matches numpy.polyfit
    model.a = max(coeffs(2), 0.0);
    model.b = coeffs(1);
    bins = struct('range_m', {}, 'std', {}, 'n', {});
    for i = 1:numel(centers)
        bins(i).range_m = centers(i);
        bins(i).std = stds(i);
        bins(i).n = counts(i);
    end
    model.bins = bins;
end

function p = logisticP(r, pMax, r0, k)
    p = pMax ./ (1 + exp((r - r0) / k));
end

function model = fitDetectionProbability(occTotal, occDetected, binWidth, maxRangeM)
    keysArr = sort(cell2mat(keys(occTotal)));
    rs = (keysArr + 0.5) * binWidth;
    ps = zeros(size(keysArr));
    ns = zeros(size(keysArr));
    for i = 1:numel(keysArr)
        k = keysArr(i);
        ns(i) = occTotal(k);
        ps(i) = getOr(occDetected, k, 0) / ns(i);
    end

    if numel(rs) >= 3
        p0 = [max(max(ps), 0.5), rs(round(numel(rs)/2)), 5.0];
        objective = @(p) sum((logisticP(rs(:), p(1), p(2), p(3)) - ps(:)).^2);
        opts = optimset('Display', 'off');
        pFit = fminsearch(objective, p0, opts);
        pMax = min(max(pFit(1), 0.0), 1.0);
        r0 = pFit(2);
        k = max(pFit(3), 0.1);
    else
        pMax = 0.9; r0 = maxRangeM / 2; k = 5.0;
        if ~isempty(ps)
            pMax = max(ps);
        end
    end

    model.model = 'logistic: p_max / (1 + exp((r - r0) / k))';
    model.p_max = pMax; model.r0 = r0; model.k = k;
    bins = struct('range_m', {}, 'p_detect', {}, 'n', {});
    for i = 1:numel(rs)
        bins(i).range_m = rs(i);
        bins(i).p_detect = ps(i);
        bins(i).n = ns(i);
    end
    model.bins = bins;
end

function model = fitClutterRate(clutterRanges, nFrames, fovSolidAngleSr, binWidth, maxRangeM)
    edges = 0:binWidth:maxRangeM;
    counts = histcounts(clutterRanges, edges);
    bins = struct('range_m', {}, 'points_per_frame', {}, 'density_per_frame_per_m3', {});
    for i = 1:numel(counts)
        rLo = edges(i); rHi = edges(i+1);
        shellVol = fovSolidAngleSr / 3.0 * (rHi^3 - rLo^3);
        ratePerFramePerM3 = counts(i) / max(nFrames, 1) / max(shellVol, 1e-6);
        bins(i).range_m = (rLo + rHi) / 2;
        bins(i).points_per_frame = counts(i) / max(nFrames, 1);
        bins(i).density_per_frame_per_m3 = ratePerFramePerM3;
    end
    model.n_frames = nFrames;
    model.bins = bins;
end

function out = summarizeIntensity(intensityMap, binWidth)
    binKeys = sort(cell2mat(keys(intensityMap)));
    out = struct('range_m', {}, 'mean', {}, 'std', {}, 'n', {});
    for i = 1:numel(binKeys)
        b = binKeys(i);
        vals = intensityMap(b);
        out(i).range_m = (b + 0.5) * binWidth;
        out(i).mean = mean(vals);
        out(i).std = std(vals);
        out(i).n = numel(vals);
    end
end

function calibration = buildCalibrationStruct(acc, opt, h5Path)
    calibration.source.dataset = 'ColoRadar';
    calibration.source.file = h5Path;
    calibration.source.device = opt.Device;
    calibration.source.n_radar_frames = acc.nRadarFrames;

    calibration.caveats = { ...
        'Collected on a ground rig; multipath/ground-clutter spatial pattern will differ on a drone.', ...
        'No dust/fog/smoke severity labels in this dataset -- these are clear-air noise parameters only.', ...
        'Elevation-angle coverage is limited to what the ground rig''s trajectories explored.', ...
        'Radar clutter/detection stats depend on RadarTopKPerFrame, a stand-in for a real CFAR threshold.', ...
        sprintf(['MaxRangeM=%.2fm is assumed to match this cascade radar''s unambiguous range ' ...
            '(num_range_bins*range_bin_width from calib.zip) -- verify against your own calib ' ...
            'files if this run''s waveform config differs.'], opt.MaxRangeM)};

    calibration.range_noise_std_m = fitLinearStd(acc.rangeResid, acc.residRangeBin, 12, opt.MaxRangeM);
    calibration.azimuth_noise_std_rad = fitLinearStd(acc.azResid, acc.residRangeBin, 12, opt.MaxRangeM);
    calibration.elevation_noise_std_rad = fitLinearStd(acc.elResid, acc.residRangeBin, 12, opt.MaxRangeM);
    calibration.doppler_noise_std_mps = fitLinearStd(acc.dopplerResid, acc.dopplerResidRangeBin, 12, opt.MaxRangeM);
    calibration.detection_probability = fitDetectionProbability(acc.occTotal, acc.occDetected, opt.DropoutRangeBinM, opt.MaxRangeM);
    calibration.clutter = fitClutterRate(acc.clutterRanges, acc.nRadarFrames, acc.fovSolidAngleSr, opt.DropoutRangeBinM, opt.MaxRangeM);
    calibration.intensity_by_range = summarizeIntensity(acc.intensityByBin, opt.DropoutRangeBinM);
    calibration.fov_assumed_deg.azimuth = opt.FovAzimuthDeg;
    calibration.fov_assumed_deg.elevation = opt.FovElevationDeg;
    calibration.match_tolerance_m = opt.MatchDistM;
end
