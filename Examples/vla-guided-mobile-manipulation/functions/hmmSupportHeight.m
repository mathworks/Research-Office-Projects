function z = hmmSupportHeight(target)
%HMMSUPPORTHEIGHT Height of the surface a target stands on, from the engine's nadir sweep.
%
%   z = HMMSUPPORTHEIGHT(target) returns the highest swept surface at or just below target(3)
%   under the target's (x, y), or target(3) itself when the sweep has nothing there.
%
%   WHY IT EXISTS. hmmArmLeverBand is indexed by the SUPPORT SURFACE ("THE HEIGHT IS THE SUPPORT
%   SURFACE, NOT THE TOOL"), but a pick target arrives as ObjectTarget.Position, whose height is
%   the object's middle rather than the shelf under it - 0.894 m for the Bottle on a 0.800 m
%   chest. Passing that shrinks the arm's admissible lever from 0.658 m to 0.523 m and refuses
%   every stand-off, which the base then reports as BaseGoalOccupied. A place target is already
%   a support point, and this returns it unchanged.
%
%   THE SURFACE IS data/shellProbe/floorPlan.mat - the same sweep the navigation map, the release
%   settle (hmmSettleProp) and the place slide (hmmReconstructTargets) use. A 3 x 3 cell patch is
%   read so a target over a cell boundary or a thin edge still finds its surface; surfaces more
%   than 20 mm above target(3) are ignored, because nothing rests on what is above it.
%
%   See also hmmArmLeverBand, hmmSelectBaseGoal, hmmSettleProp.

%   Copyright 2026 The MathWorks, Inc.

z  = double(target(3));
hm = heightMap();
if isempty(hm)
    return
end
row = round((hm.world(2, 2) - target(2)) * hm.resolution) + 1 + (-1:1);
col = round((target(1) - hm.world(1, 1)) * hm.resolution) + 1 + (-1:1);
row = row(row >= 1 & row <= size(hm.heightMap, 1));
col = col(col >= 1 & col <= size(hm.heightMap, 2));
h = hm.heightMap(row, col);
h = h(isfinite(h) & h <= z + 0.020);
if ~isempty(h)
    z = max(h);
end
end

function hm = heightMap()
%HEIGHTMAP The nadir-sweep surface heights, loaded once per session, or empty.
persistent held
if isempty(held)
    f = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'shellProbe', ...
        'floorPlan.mat');
    if isfile(f)
        S = load(f, 'report');
        held = {S.report};
    else
        held = {[]};
    end
end
hm = held{1};
end
