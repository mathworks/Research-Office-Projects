function tf = hmmPlaceReachesOver(pose, q, cfg, objExtent)
%HMMPLACEREACHESOVER Does the straight reach from a base pose to a place point cross furniture?
%
%   tf = HMMPLACEREACHESOVER(pose, q, cfg, objExtent) samples the nadir height map
%   (data/shellProbe/floorPlan.mat) along the line from the base pose [x y yaw] to the place
%   point q (3x1), from just outside the chassis to just short of the object's own patch of
%   surface, and is true when anything there is more than cfg.percep.placeReachOverTol above
%   q(3). Movable props are not in the map, so only furniture is seen. An empty pose, or no map,
%   is false.
%
%   WHY. The place gates judge a stand-off in 2-D: the base fits, and the distance is inside the
%   arm's lever band. Neither sees a backrest between the two. The radius ladder made that real:
%   a couch seat became serviceable from BEHIND the couch, over its 0.67 m backrest, at the
%   arm's 0.82 m limit (2026-10-15) - a place the 3-D arm planner is likely to refuse. So
%   hmmSlideOnSurface prefers points whose reach is clear, and hmmReconstructTargets does not
%   keep a provider point that is served ONLY by reaching over something.
%
%   See also hmmSlideOnSurface, hmmPlaceBearings.

%   Copyright 2026 The MathWorks, Inc.

tf = false;
if isempty(pose)
    return
end
hm = heightMap();
if isempty(hm)
    return
end
H    = hm.heightMap;
res  = hm.resolution;
from = 0.5 * cfg.base.chassisSize(1);                        % leave the chassis
to   = 0.5 * max(double(objExtent(1:2))) + cfg.percep.surfaceInset;
dxy  = q(1:2) - pose(1:2);
L    = norm(dxy);
if L <= from + to
    return
end
s  = (from:1 / res:(L - to))' / L;
P  = pose(1:2)' + s * dxy';
r  = round((hm.world(2, 2) - P(:, 2)) * res) + 1;
c  = round((P(:, 1) - hm.world(1, 1)) * res) + 1;
in = r >= 1 & r <= size(H, 1) & c >= 1 & c <= size(H, 2);
h  = H(sub2ind(size(H), r(in), c(in)));
tf = any(isfinite(h) & h > q(3) + cfg.percep.placeReachOverTol);
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
