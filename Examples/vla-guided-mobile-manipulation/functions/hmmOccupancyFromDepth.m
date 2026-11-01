function grid = hmmOccupancyFromDepth(frames, basePose, priorGrid)
%HMMOCCUPANCYFROMDEPTH Accumulate the navigation occupancy grid from observed depth.
%
%   grid = HMMOCCUPANCYFROMDEPTH(frames, basePose, priorGrid) folds this frame
%   set's calibrated depth returns into the occupancy grid and returns the
%   updated probability matrix. basePose is the current [x y theta] of the base,
%   used only to discard the robot's own returns. priorGrid is the grid currently
%   in the NavMapGrid data store, so the data store holds ALL the map state and
%   this function keeps none: two identical calls with the same prior give the
%   same answer.
%
%   The update is a proper log-odds one, through occupancyMap/updateOccupancy
%   (Navigation Toolbox), not an overwrite. Evidence therefore accumulates over a
%   run and a single spurious ray return cannot carve a hole in a wall.
%
%   ONLY THE CALIBRATED RAY PATH FEEDS THE MAP. This rule was made when the
%   render depth from Simulation 3D Camera was believed quantised to 1 m. It is
%   not: cfg.depth.renderQuantum was later MEASURED at 1000/2^24 = 59.6 um (a
%   1000 m far plane over 24 bits), and the wrist refinement now ranges off that
%   depth to the millimetre. The ray path is kept here because it is calibrated
%   and already verified against the shell, not because the render depth is too
%   coarse - camera depth plus pcfromdepth would be a legitimate alternative.
%
%   THE HEIGHT BAND IS THE WHOLE POINT. cfg.depth.occupancyBand is the vertical
%   envelope of the base with the arm stowed. A return below it is the floor and
%   is evidence the cell is FREE. A return inside it blocks the base and is
%   evidence the cell is OCCUPIED. A return above it - a ceiling, a light, the
%   underside of a wall-mounted cabinet - is IGNORED, because a 2-D grid that
%   marks it occupied forbids driving under something the robot fits under.
%
%   ONE UPDATE PER CAMERA, NOT ONE PER FRAME SET. updateOccupancy COLLAPSES
%   duplicate points inside a single call: four identical rows move a cell's
%   log-odds by one increment, not four (measured - a cell goes 0.10 -> 0.50 on a
%   call with four copies of the same point, and 0.50 -> 0.90 on the next call).
%   Four cameras are four independent sensors, so each gets its own pair of calls
%   and contributes its own increment. Grouping them would make an obstacle seen
%   by every camera take as long to appear as one seen by a single camera.
%
%   CONFLICTS RESOLVE TOWARDS OCCUPIED, EXPLICITLY. If one ray of a camera hits
%   the floor in a cell and another hits an obstacle in it, applying both leaves
%   the cell's log-odds exactly where it started, because the free and occupied
%   increments are symmetric by construction. So the free set has the occupied
%   cells REMOVED from it before either update is applied. Between "there is
%   nothing there" and "there is something there", a planner must believe the
%   second one, and the only way to mean that is to say it.
%
%   THE ROBOT IS NOT AN OBSTACLE TO ITSELF. The four fixed cameras look into the
%   room from its corners, so the robot is inside every one of their fields of
%   view and a large share of their returns land on its own body - inside the
%   occupancy band, because that band IS the base envelope. Folding those in
%   marks the cells the robot is standing in occupied, and after the planner's
%   inflation the base can no longer prove its OWN pose is valid: every request
%   fails at the start-state check before a single tree node is sampled. Returns
%   within the stowed envelope of the base's current position are therefore
%   dropped. The radius is cfg.plan.baseRadius + cfg.plan.stowedArmRadius, a disc
%   that contains the chassis rectangle the planner checks (hmmBaseFootprint),
%   because a return inside the volume the robot occupies cannot be evidence about
%   the world in any case. That
%   envelope has to COVER the stowed gripper, not just the chassis:
%   measureStowedEnvelope is what establishes the number, and hmmSetup warns when
%   the assembled tree outgrows it.
%
%   See also hmmPlanBasePath, measureStowedEnvelope, hmmRayDirections, occupancyMap.

%   Copyright 2026 The MathWorks, Inc.

cfg = cachedConfig();

% UNDER A FIXED MAP THIS FUNCTION IS A PASS-THROUGH, and it has to be, not merely
% for the saved effort. The shipped grid marks an occupied cell at
% cfg.map.obsOccupied = 0.90, and the free and occupied increments here are
% symmetric by construction, so ONE camera floor return in a cell the captured map
% calls occupied takes it to exactly 0.50 - below cfg.map.occupiedThreshold, and
% therefore traversable. That is not a rare conflict: a ceiling camera routinely
% sees the floor under a table whose top the nadir capture measured as occupied.
% Folding live returns into a captured map would quietly carve holes in it, which
% is worse than either map alone.
%
% Returning priorGrid also means the fold's rate stops mattering. SceneMapping
% sits inside the Send-triggered Perception subsystem, so it fires once per
% grounding request rather than at cfg.rates.perception. That defect is what
% made the live map empty; under 'fixed' it is inert, so the structural edit to
% move the chain out is no longer on the critical path for navigation. It still
% matters for anyone who sets cfg.map.mode = 'live'.
if strcmp(cfg.map.mode, 'fixed')
    grid = priorGrid;
    return
end

% priorGrid is carried forward through the map rather than through grid: the
% occupancyMap below is seeded with it, every observation updates the map, and
% grid is read back out of the map at the end. There is no path that returns the
% prior unchanged, so grid needs no earlier value.
map = occupancyMap(priorGrid, cfg.map.resolution);
map.GridLocationInWorld = cfg.map.origin;
map.OccupiedThreshold   = cfg.map.occupiedThreshold;
map.FreeThreshold       = cfg.map.freeThreshold;

zLo = cfg.depth.occupancyBand(1);
zHi = cfg.depth.occupancyBand(2);

selfXY     = double(basePose(1:2));
selfRadius = cfg.plan.baseRadius + cfg.plan.stowedArmRadius;

for k = 1:double(cfg.camera.nFixed)
    f = frames(k);
    if ~f.Valid || f.DepthSource ~= cfg.depth.SOURCE_RAYTRACE
        continue
    end
    hit = f.Calib.IsValidHit;
    if ~any(hit)
        continue
    end
    P = double(f.Calib.HitLocations(hit, :));
    P = P(vecnorm(P(:, 1:2) - selfXY(:)', 2, 2) > selfRadius, :);
    if isempty(P)
        continue
    end
    z = P(:, 3);

    occupied = insideMap(P(z >= zLo & z <= zHi, 1:2), map);
    free     = insideMap(P(z < zLo, 1:2), map);
    free     = dropCells(free, occupied, map);

    if ~isempty(free)
        updateOccupancy(map, free, cfg.map.obsFree);
    end
    if ~isempty(occupied)
        updateOccupancy(map, occupied, cfg.map.obsOccupied);
    end
end

grid = occupancyMatrix(map);

end

%% ------------------------------------------------------------------------
function xy = dropCells(xy, remove, map)
%DROPCELLS Remove points that share a grid cell with any point in remove.
%   Comparison is by CELL, not by coordinate: two rays a millimetre apart are the
%   same cell and disagreeing about it is the conflict this resolves.
if isempty(xy) || isempty(remove)
    return
end
a = world2grid(map, xy);
b = world2grid(map, remove);
xy = xy(~ismember(a, b, 'rows'), :);
end

%% ------------------------------------------------------------------------
function xy = insideMap(xy, map)
%INSIDEMAP Drop points outside the grid. updateOccupancy errors on them, and a
%   ray that leaves the mapped area is a fact about the map's extent, not a
%   failure of the sensor.
if isempty(xy)
    return
end
[~, keep] = getOccupancy(map, xy);   % second output: inside the map, edges included
xy = xy(keep, :);
end

%% ------------------------------------------------------------------------
function cfg = cachedConfig()
cfg = hmmRuntimeConfig();
end
