function tf = hmmWallBetween(from, to)
%HMMWALLBETWEEN True when a wall stands on the straight line between two floor points.
%
%   tf = HMMWALLBETWEEN(from, to) samples the engine's nadir sweep (data/shellProbe/floorPlan.mat)
%   along the segment from(1:2) -> to(1:2) and reports whether any cell on it reaches
%   cfg.plan.wallHeight. The two end cells are skipped: the base stands on one and the target on
%   the other.
%
%   WHY IT EXISTS. Every stand-off gate measured DISTANCE - a free floor cell at the right lever
%   from the target - and nothing asked whether the arm could get there. On 2026-10-12 the Bottle,
%   on a chest standing against the bedroom/bathroom wall, was served from the BATHROOM: a free
%   cell 0.598 m from it with the 2.70 m wall in between. The base drove there and the arm was
%   refused CollisionReject. A wall is the one thing the arm cannot reach over; furniture lower
%   than cfg.plan.wallHeight - including the surface the target stands on - is left to the arm
%   planner's own collision screen, which already handles it.
%
%   See also hmmStandoffRing, hmmSupportHeight.

%   Copyright 2026 The MathWorks, Inc.

tf = false;
[hm, wallHeight] = cached();
if isempty(hm)
    return
end
d    = to(1:2) - from(1:2);
len  = norm(d);
step = 0.5 / hm.resolution;
n    = floor(len / step);
if n < 3
    return
end
s   = (2:n - 1) / n;                     % interior samples only
x   = from(1) + s * d(1);
y   = from(2) + s * d(2);
row = round((hm.world(2, 2) - y) * hm.resolution) + 1;
col = round((x - hm.world(1, 1)) * hm.resolution) + 1;
ok  = row >= 1 & row <= size(hm.heightMap, 1) & col >= 1 & col <= size(hm.heightMap, 2);
h   = hm.heightMap(sub2ind(size(hm.heightMap), row(ok), col(ok)));
tf  = any(h >= wallHeight);
end

function [hm, wallHeight] = cached()
%CACHED The sweep and the wall threshold, once per session.
persistent held
if isempty(held)
    cfg = hmmConfig();
    f = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'shellProbe', ...
        'floorPlan.mat');
    if isfile(f)
        S = load(f, 'report');
        held = {S.report, cfg.plan.wallHeight};
    else
        held = {[], cfg.plan.wallHeight};
    end
end
hm = held{1};
wallHeight = held{2};
end
