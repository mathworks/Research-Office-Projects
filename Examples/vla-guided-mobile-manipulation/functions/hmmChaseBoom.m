function T = hmmChaseBoom(Tchassis)
%HMMCHASEBOOM The chase camera's pose: behind the robot, close rather than high or sideways.
%
%   T = HMMCHASEBOOM(Tchassis) takes chassis_link's pose in 'map' (4x4) and returns the camera's:
%   up to cfg.view.chase.back behind the robot, looking over it at a point cfg.view.chase.aimAhead
%   in front. Out in the room that is the inspect-sim3d-scenes skill's "Robot Third-Person" stance;
%   against a wall it is a low view over the robot's shoulder.
%
%   THE BOOM IS CAST BACKWARDS on a binaryOccupancyMap of every cell of the shell's nadir sweep
%   (data/shellProbe/floorPlan.mat) taller than cfg.view.chase.blockHeight (rayIntersection), and
%   stops cfg.view.chase.wallMargin short of the first hit. It may swing to one of
%   cfg.view.chase.swingOffsets off straight behind - 30 deg at most - but only where that buys
%   cfg.view.chase.swingCost of boom per degree, and it pays half that again per degree away from
%   where the camera already is, so it keeps to one side. A version allowed 100 deg sat at the
%   robot's side at the start pose, and one that rose instead of shortening looked from the top.
%
%   SMOOTHED BOTH WAYS. The direction follows the winner at cfg.view.chase.turnGain a step, so a turn
%   of the base glides. The length shortens at once to what is clear along the camera's own
%   direction - it never enters a wall - and lengthens at cfg.view.chase.extendGain.
%
%   THE HEIGHT FOLLOWS THE BOOM, cfg.view.chase.heightNear at minBack to heightFar at back, and the
%   aim point keeps the pitch near 25-30 deg either way.
%
%   DISPLAY ONLY. Plant/ChaseCamera/ChaseBoom calls it every scene step; nothing reads the camera.
%
%   See also hmmConfig (cfg.view.chase).

%   Copyright 2026 The MathWorks, Inc.

persistent map az boom
cfg = hmmRuntimeConfig();
c = cfg.view.chase;
if isempty(map)
    S = load(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'shellProbe', ...
        'floorPlan.mat'), 'report');
    % Row 1 of the sweep is its highest y, the same convention binaryOccupancyMap reads a matrix
    % in, so the grid goes in as it is and only its corner is placed.
    map = binaryOccupancyMap(S.report.heightMap > c.blockHeight, S.report.resolution);
    map.GridLocationInWorld = [S.report.world(1, 1), S.report.world(2, 1)];
end

xy = Tchassis(1:2, 4)';
behind = atan2(Tchassis(2, 1), Tchassis(1, 1)) + pi;
if isempty(az)
    az = behind;
    boom = c.back;
end

% The clearest direction, both sides of straight behind.
offs = unique([c.swingOffsets(:); -c.swingOffsets(:)])';
best = behind;
bestScore = -Inf;
for k = 1:numel(offs)
    a = behind + offs(k);
    s = clearAlong(map, xy, a, c) - c.swingCost * abs(rad2deg(offs(k))) ...
        - 0.5 * c.swingCost * abs(rad2deg(angdiff(az, a)));
    if s > bestScore
        bestScore = s;
        best = a;
    end
end
az = az + c.turnGain * angdiff(az, best);

target = clearAlong(map, xy, az, c);
if target < boom
    boom = target;                                   % at once: never into the wall
else
    boom = boom + c.extendGain * (target - boom);    % slowly: no spring back
end

% The height follows the boom - low over the shoulder when close, the preset's stance when far -
% and the camera aims aimAhead in front of the robot, so the pitch stays near 25-30 deg throughout.
f = min(max((boom - c.minBack) / (c.back - c.minBack), 0), 1);
height = c.heightNear + f * (c.heightFar - c.heightNear);
pitch = atan2(height - c.aimHeight, boom + c.aimAhead);
pos = [xy + boom * [cos(az), sin(az)], Tchassis(3, 4) + height];
T = [eul2rotm([az + pi, pitch, 0], 'ZYX'), pos(:); 0 0 0 1];
end

%% ------------------------------------------------------------------------
function d = clearAlong(map, xy, a, c)
% How long a boom can be along heading a before the first tall cell, less the margin.
hit = rayIntersection(map, [xy, a], 0, c.back + c.wallMargin);
d = c.back;
if all(isfinite(hit))
    d = max(c.minBack, norm(hit - xy) - c.wallMargin);
end
end
