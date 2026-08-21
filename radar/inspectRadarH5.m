function inspectRadarH5(h5Path, maxDepth)
%INSPECTRADARH5  Print the group/dataset tree of an exported ColoRadar .h5
%file so you can correct the schema in calibrateRadarNoise.m if it doesn't
%match (it should, for cascade_radar / lidar exports -- see that file's
%header comment for the confirmed naming convention).
%
%   inspectRadarH5('dataset.h5')
%   inspectRadarH5('dataset.h5', 3)   % limit recursion depth
%
% Only relevant if you converted ColoRadar's raw .bag files into a
% coloradar-library .h5 export. If you're working from the .bag directly,
% use inspectRosbag.m / calibrateRadarNoiseFromBag.m instead.

    if nargin < 2
        maxDepth = 4;
    end
    info = h5info(h5Path);
    fprintf('File: %s\n', h5Path);
    printGroup(info, 0, maxDepth);
end

function printGroup(groupInfo, depth, maxDepth)
    if depth > maxDepth
        return
    end
    indent = repmat('  ', 1, depth);
    if isfield(groupInfo, 'Datasets')
        for i = 1:numel(groupInfo.Datasets)
            d = groupInfo.Datasets(i);
            sz = strjoin(arrayfun(@num2str, d.Dataspace.Size, 'UniformOutput', false), 'x');
            fprintf('%s[DATASET] %s  shape=%s  class=%s\n', indent, d.Name, sz, d.Datatype.Class);
        end
    end
    if isfield(groupInfo, 'Groups')
        for i = 1:numel(groupInfo.Groups)
            g = groupInfo.Groups(i);
            [~, shortName] = fileparts(g.Name);
            fprintf('%s[GROUP]   %s\n', indent, shortName);
            printGroup(g, depth + 1, maxDepth);
        end
    end
end
