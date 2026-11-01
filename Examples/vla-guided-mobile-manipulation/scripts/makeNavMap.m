function info = makeNavMap(source, opts)
%MAKENAVMAP Capture the navigation occupancy grid once and ship it with the project.
%
%   info = MAKENAVMAP() measures the apartment with straight-down rays against the
%   scene mesh and writes the resulting grid, with its provenance, to
%   data/hmmNavMap.mat. hmmSetup loads that file into the NavMapGrid data store's
%   initial value on every run, so the map is captured once and reused.
%
%   info = MAKENAVMAP("cameras") instead records what the model's own four fixed
%   cameras build during a short run. info = MAKENAVMAP("cameras", StopTime = 8)
%   sets that run's length. MAKENAVMAP(..., Force = true) re-casts the nadir rays
%   even when a floor plan is already on disk.
%
%   WHY NADIR RAYS ARE THE DEFAULT, AND WHAT THE CAMERAS ACTUALLY CANNOT DO. An
%   earlier version of this comment said the four cameras top out near 20 % of the
%   obstacle cells and that the missing 80 % was geometry - a rig at 2.3 m looking
%   down 40 degrees seeing only floor and table tops. That was WRONG, and it was
%   substantially wrong, because it was measured at cfg.depth.rayGrid = [36 64]. At
%   2304 rays a camera samples 2.49 degrees per ray (20 px at f = 460), so a wall band
%   in the middle distance collects a handful of returns. An earlier version of this
%   sentence said 1.3 degrees, which is the figure for the SHIPPED grid, not the one it
%   was explaining. THE SHIPPED GRID IS NOW [72 128] - 5 px per ray at f = 230, so
%   1.246 degrees, half the angular step - which is the 9 216 row below - so the live
%   cameras reach 41.2 % rather than 17.7 %, and the nadir default is a smaller win
%   than it used to be without ceasing to be one. Re-measuring the same mesh, mounts
%   and height band at other ray densities, scoring the INTERSECTION with the obstacle
%   cells this sweep measures, gives:
%
%       rays/camera    cells marked    real obstacles found
%             2 304           1 921        1 333     17.7 %
%             9 216           3 934        3 094     41.2 %
%            36 864           5 837        4 873     64.9 %
%           147 456           6 674        5 643     75.1 %
%
%   So density is worth a 4x improvement and the ceiling is mostly sampling, not
%   geometry - but geometry does account for the residual quarter, and coverage is
%   still climbing at 64x the shipped ray count. The camera route stays off the
%   default path for three reasons that survive the correction. It costs about
%   590 000 rays against this sweep's 89 600. It marks only OCCUPIED cells, leaving
%   free space unmeasured, so unknown has to mean traversable - the convention this
%   file deliberately inverts. And it is occluded by construction, so the floor
%   under a table is never established as floor, whereas a nadir ray measures every
%   cell of the grid from directly above.
%
%   THE CAMERA SOURCE IS KEPT ANYWAY, because it answers a different question -
%   what the model perceives, as opposed to what is there - and because a claim
%   about what the cameras can and cannot do is worth being able to re-measure
%   rather than having to trust this comment. That is not rhetorical: the 20 %
%   claim above survived precisely because nobody re-measured it.
%
%   UNMEASURED IS TREATED AS OCCUPIED, WHICH INVERTS THE LIVE MAP'S CONVENTION and
%   is deliberate. The grid starts fully occupied and free space is carved out of
%   it where a nadir ray found bare floor, so everything the sweep never
%   established as floor - outside the shell, the 5918 cells where a ray hit
%   nothing at all, and the margin between the floor plan's extent and the larger
%   navigation grid - reads as blocked. The live map does the opposite: an
%   unobserved cell sits at cfg.map.initialProbability = 0.10 and is traversable,
%   which is safe only while the map is too empty to be trusted anyway. Once the
%   map is complete, "no evidence of floor" must mean "do not drive there", or the
%   robot will happily leave the building through a region nobody measured.
%
%   A COMPLETE MAP MAKES SOME PREVIOUSLY SUCCESSFUL PLANS FAIL, and that is the
%   point rather than a regression. Room datums measure 0.25, 0.20, 0.00 and
%   0.90 m of clearance, so a goal placed on the Bedroom datum is inside
%   furniture; with an empty map the planner accepted it. Expect base plans to
%   start refusing goals that were never reachable.
%
%   PROVENANCE IS SAVED WITH THE GRID BECAUSE A SILENTLY MISALIGNED MAP IS THE
%   WORST OUTCOME HERE. A grid is just a matrix of probabilities; nothing about it
%   records the resolution or world origin it was built against. Change
%   cfg.map.resolution and reuse this file and the robot plans confidently in a
%   world shifted and scaled from the one it drives through - no error, just
%   collisions. hmmLoadNavMap refuses the file rather than resampling it.
%
%   THE PROPS ARE NOT IN THE MESH THE RAYS ARE CAST AT, which is the one thing
%   this script got categorically wrong for as long as it has existed. The nadir
%   sweep measures cfg.scene.staticMesh; the props are separate actors placed at
%   spawn. So the two 0.30 x 0.30 x 0.66 m stools standing on the kitchen floor
%   and the 0.22 m ball on the living-room floor were bare navigable floor to
%   every base plan ever made against this map, and the shipped place leg parked
%   its chassis inside one of the stools. hmmFloorObstacles now picks out every
%   manifest prop whose box rests on the floor and rises above it, and
%   occupyFloorProps marks those cells occupied after the doorway carve. The test
%   is geometric and names no prop - see cfg.map.floorPropBand for the census.
%
%   AND THE PROPS ARE NOT BOXES EITHER, which is the half of that this script got
%   wrong next. The manifest publishes an extent and no shape, so the first fix
%   stamped each prop's bounding RECTANGLE and the two round stools came out square,
%   blocking four corners of real floor beside the one surface where the place leg
%   has the fewest arrival poses to spare. hmmPropFootprints measures each prop's
%   true footprint by casting nadir rays at the prop MESH - the same instrument this
%   script uses on the shell - so a cylinder marks a disc and nothing anywhere knows
%   the name of any prop. The rectangle remains the fallback when a prop cannot be
%   measured, since too much occupied is the safe direction.
%
%   See also hmmLoadNavMap, hmmFloorObstacles, hmmPropFootprints, probeFloorPlan.

%   Copyright 2026 The MathWorks, Inc.

arguments
    source (1, 1) string {mustBeMember(source, ["floorplan", "cameras"])} = "floorplan"
    opts.StopTime (1, 1) double {mustBePositive} = 6
    opts.Force (1, 1) logical = false
end

root = fileparts(fileparts(mfilename('fullpath')));

% cfg.map.mode is deliberately NOT consulted here. This script's job is to produce
% the file; whether a run uses it is hmmLoadNavMap's decision. Capturing only when
% the mode already says 'fixed' would make the file impossible to create from a
% clean checkout, since the mode cannot sensibly be 'fixed' before it exists.
%
% AND THE CONFIGURATION IS READ DIRECTLY RATHER THAN THROUGH hmmSetup, for the same
% class of reason one step further on. hmmSetup assembles navMapGrid by calling
% hmmLoadNavMap, which REFUSES a map file whose provenance disagrees with the current
% cfg - correct for a run, fatal here, because the only tool that can repair a stale
% map would then be unable to start because the map is stale. This script needs cfg
% and the path, so it takes exactly those. The "cameras" branch below still gets a
% full hmmSetup: it loads the model, whose PostLoadFcn calls it.
addpath(fullfile(root, 'configuration'), fullfile(root, 'functions'), ...
        fullfile(root, 'robots'), fullfile(root, 'scripts'));
cfg = hmmConfig();

switch source
    case "floorplan"
        [grid, detail] = fromFloorPlan(cfg, root, opts.Force);
    case "cameras"
        [grid, detail] = fromCameras(cfg, opts.StopTime);
end

% REFUSE A GRID THAT SAYS NOTHING. A uniform grid - all prior, all free or all
% occupied - is what every failure mode here produces, and saving one would hand
% the planner either a wholly free apartment or a solid block, both of which look
% exactly like a successful capture from the outside.
if numel(unique(grid(:))) < 2
    error('makeNavMap:uniformGrid', ...
        ['Captured grid holds the single value %g in all %d cells, so the capture ' ...
         'measured nothing. For "cameras", check that the ray tracers fire and that ' ...
         'frames report DepthSource == cfg.depth.SOURCE_RAYTRACE; for "floorplan", ' ...
         'check that probeFloorPlan found the shell mesh.'], grid(1), numel(grid));
end

occupied = nnz(grid >= cfg.map.occupiedThreshold);
free     = nnz(grid <= cfg.map.freeThreshold);
fprintf('\nmakeNavMap: %d occupied cells, %d free, %d in between, of %d.\n', ...
    occupied, free, numel(grid) - occupied - free, numel(grid));

% The provenance every consumer must agree with. Stored as its own struct so
% hmmLoadNavMap can compare field by field and name the one that differs.
provenance = struct( ...
    'resolution',         cfg.map.resolution, ...
    'origin',             cfg.map.origin, ...
    'gridSize',           cfg.map.gridSize, ...
    'occupiedThreshold',  cfg.map.occupiedThreshold, ...
    'freeThreshold',      cfg.map.freeThreshold, ...
    'initialProbability', cfg.map.initialProbability, ...
    'sceneExtent',        cfg.scene.extent, ...
    'doorways',           cfg.map.doorways);

% Context that is useful when reading the file months later but is NOT compared,
% because none of it changes what the grid means geometrically.
context = struct( ...
    'source',         char(source), ...
    'detail',         detail, ...
    'sceneMesh',      cfg.scene.staticMesh, ...
    'occupancyBand',  cfg.depth.occupancyBand, ...
    'created',        char(datetime('now', 'Format', 'uuuu-MM-dd HH:mm:ss')), ...
    'matlabRelease',  version('-release'), ...
    'occupiedCells',  occupied, ...
    'freeCells',      free);

outFile = fullfile(root, 'data', 'hmmNavMap.mat');
save(outFile, 'grid', 'provenance', 'context', '-v7.3');
fprintf('makeNavMap: wrote %s\n', outFile);

% WRITING THE FILE IS NOT ENOUGH TO CHANGE WHAT THE NEXT RUN USES. hmmSetup reuses
% the assembled data already in the base workspace unless it is forced, so in a
% session that has already run anything the model would keep planning against the
% PREVIOUS map while this file sits on disk looking authoritative. Said rather than
% done, because forcing a rebuild here would drop caches this script does not own.
fprintf(['makeNavMap: call hmmSetup(true) before the next run - hmmSetup reuses the\n' ...
         '            map already in the base workspace, so this file is not picked up\n' ...
         '            until the cache is dropped.\n']);

% The grid as an image, for the same reason probeFloorPlan writes them: a count
% of occupied cells cannot show that the walls are in the right PLACE.
imwrite(uint8(255 * min(max(grid, 0), 1)), fullfile(root, 'data', 'shellProbe', 'navmap.png'));

info = struct('file', outFile, 'grid', grid, ...
    'provenance', provenance, 'context', context);
if nargout == 0
    clear info
end

end

%% ------------------------------------------------------------------------
function [grid, detail] = fromFloorPlan(cfg, root, force)
%FROMFLOORPLAN Nadir-ray floor plan, resampled onto the navigation grid.

% THE SWEEP MUST COVER THE WHOLE NAVIGATION GRID, not the whole apartment, and the
% two are not the same rectangle. cfg.map is 12 x 12 m from cfg.map.origin, while
% cfg.scene.extent is the 10 x 8 m building, so the grid carries a 1 m skirt on x
% and an asymmetric 1 m / 3 m skirt on y. probeFloorPlan sweeps the extent plus a
% SYMMETRIC margin, so the margin is derived from whichever side needs most rather
% than typed - a hard-coded 0.5 leaves the northern strip of the grid unswept, and
% every unswept cell is then occupied by assumption instead of by measurement.
% Sweeping a superset is harmless: cells outside the grid are dropped below.
xLim = cfg.map.origin(1) + [0 cfg.map.size(1)];
yLim = cfg.map.origin(2) + [0 cfg.map.size(2)];
margin = max([cfg.scene.extent(1, 1) - xLim(1), xLim(2) - cfg.scene.extent(1, 2), ...
              cfg.scene.extent(2, 1) - yLim(1), yLim(2) - cfg.scene.extent(2, 2)]);
margin = ceil(margin * cfg.map.resolution) / cfg.map.resolution;   % whole cells

planFile = fullfile(root, 'data', 'shellProbe', 'floorPlan.mat');
fp = [];
if ~force && isfile(planFile)
    S  = load(planFile, 'report');
    % A plan that does not span the grid is REJECTED rather than used, because the
    % shortfall is invisible in the finished map: the cells it never reached come
    % out occupied, which is exactly what a genuinely blocked cell looks like.
    if covers(S.report, xLim, yLim)
        fprintf('makeNavMap: using the floor plan already in %s\n', planFile);
        fp = S.report;
    else
        fprintf(['makeNavMap: the floor plan on disk spans x [%.2f %.2f] y [%.2f %.2f] ' ...
            'but the navigation grid needs x [%.2f %.2f] y [%.2f %.2f] - re-sweeping.\n'], ...
            S.report.world(1, 1), S.report.world(1, 2), S.report.world(2, 1), ...
            S.report.world(2, 2), xLim(1), xLim(2), yLim(1), yLim(2));
    end
end
if isempty(fp)
    fprintf('makeNavMap: casting nadir rays over the whole grid (margin %.2f m) ...\n', margin);
    % 'Origin' phases the sweep onto this grid's cell CENTRES. Without it every ray
    % lands on a cell boundary and world2grid's tie-break loses thousands of free
    % cells to floating point - see the note in probeFloorPlan/centres.
    fp = probeFloorPlan('Resolution', cfg.map.resolution, 'Margin', margin, ...
        'Origin', cfg.map.origin);
end

% THE FLOOR PLAN MUST NOT BE COARSER THAN THE NAVIGATION GRID. Free space is
% carved out cell by cell, so a plan sampled at half the navigation resolution
% would leave every other navigation cell occupied and produce a striped map that
% no planner can cross. Finer is fine - several plan cells then land in one
% navigation cell and agree.
if fp.resolution < cfg.map.resolution
    error('makeNavMap:planTooCoarse', ...
        ['The floor plan is sampled at %g cells/m but cfg.map.resolution is %g. ' ...
         'Re-run with Force = true so the sweep is cast at the navigation ' ...
         'resolution.'], fp.resolution, cfg.map.resolution);
end

% START SOLID, CARVE OUT THE FLOOR. See the header note on why unmeasured is
% occupied rather than unknown.
P   = cfg.map.obsOccupied * ones(cfg.map.gridSize);
map = occupancyMap(P, cfg.map.resolution);
map.GridLocationInWorld = cfg.map.origin;
map.OccupiedThreshold   = cfg.map.occupiedThreshold;
map.FreeThreshold       = cfg.map.freeThreshold;

% World centres of the bare-floor cells. The row/column convention is the one
% probeFloorPlan reports: column indexes x upward from world(1,1), row indexes y
% DOWNWARD from world(2,2).
[rows, cols] = find(fp.navigable);
xy = [fp.world(1, 1) + (cols - 1) / fp.resolution, ...
      fp.world(2, 2) - (rows - 1) / fp.resolution];

% setOccupancy rejects world coordinates outside the map, and the floor plan is
% swept over a slightly different extent than the navigation grid covers, so the
% overlap is taken explicitly rather than left to an error.
[~, inMap] = getOccupancy(map, xy);
setOccupancy(map, xy(inMap, :), cfg.map.obsFree);

grid = occupancyMatrix(map);

% THE CARVE MUST BE ONE FLOOR CELL TO ONE MAP CELL, and this checks it rather than
% assuming it. Every in-extent floor cell should become exactly one free cell: the
% plan is cast at cfg.map.resolution and phased onto this grid's cell centres, so
% any shortfall means the two lattices are out of phase again and free space is
% being silently eroded - the failure that cost 3468 cells before 'Origin' existed.
% Tolerated only when the plan is FINER than the map, where collapsing is correct.
nFree = nnz(grid <= cfg.map.freeThreshold);
if fp.resolution == cfg.map.resolution && nFree ~= nnz(inMap)
    error('makeNavMap:latticeMisaligned', ...
        ['%d floor cells inside the grid carved only %d free cells, so %d were lost ' ...
         'to a lattice that is out of phase with the map. probeFloorPlan must be ' ...
         'called with ''Origin'', cfg.map.origin.'], nnz(inMap), nFree, nnz(inMap) - nFree);
end

% THE DOORWAYS THE SWEEP CANNOT SEE, opened after the lattice check and not before,
% because that check balances free cells against FLOOR cells and these are free cells
% no ray ever reported as floor. See cfg.map.doorways for the measurement and for why
% a nadir survey is blind to an opening with a header over it.
[grid, nDoor] = carveDoorways(grid, cfg, map);

% THE FURNITURE THE SWEEP CANNOT SEE EITHER, and this one is not a blind spot in the
% method - the props are simply not in the mesh the rays are cast at. Marked AFTER the
% doorway carve rather than before, because a prop standing in a doorway must win: the
% carve asserts "no ray could reach this floor" and this asserts "something is standing
% on it", and only the second is a measurement of the object itself.
[grid, nProp, propBoxes, propNames] = occupyFloorProps(grid, cfg, map);

nObstacle = nnz(~fp.navigable & isfinite(fp.heightMap));
nNoHit    = nnz(~isfinite(fp.heightMap));
fprintf(['makeNavMap: floor plan swept %d cells at %g cells/m - %d bare floor, ' ...
    '%d obstacle, %d no hit.\n'], numel(fp.navigable), fp.resolution, ...
    nnz(fp.navigable), nObstacle, nNoHit);
fprintf('makeNavMap: %d floor cells fell inside the navigation grid, %d outside it.\n', ...
    nnz(inMap), nnz(~inMap));

detail = struct('planResolution', fp.resolution, 'planWorld', fp.world, ...
    'planNavigable', nnz(fp.navigable), 'planObstacle', nObstacle, ...
    'planNoHit', nNoHit, 'floorCellsUsed', nnz(inMap), ...
    'doorwayCells', nDoor, ...
    'propCells', nProp, 'floorProps', propBoxes, ...
    'floorPropNames', propNames, ...
    'startClearance', fp.startClearance);
end

%% ------------------------------------------------------------------------
function [grid, nMarked, boxes, names] = occupyFloorProps(grid, cfg, map)
%OCCUPYFLOORPROPS Mark the cells under every floor-standing manifest prop occupied.
%
%   World metres are handed to the map, not row and column indices, for carveDoorways'
%   reason: the grid's row index runs DOWNWARD in y while its column runs upward in x,
%   and every place in this project that has written that conversion by hand has been
%   a place it could be got wrong.
%
%   WHICH PROPS, AND WHY NOT A NAME: hmmFloorObstacles decides, from the manifest's
%   own geometry. See its header for the twenty-prop census that sized the band.
%
%   THE SHAPE IS MEASURED, NOT ASSUMED, AND THE STOOLS ARE WHY. The manifest carries
%   a position and an extent and no shape at all, so this used to stamp each prop's
%   bounding RECTANGLE - and the two kitchen stools, which are round, came out square
%   with their four corners blocked. hmmPropFootprints casts one nadir ray at each map
%   cell centre the prop's box reaches and returns the centres that struck the mesh,
%   so a cylinder marks a disc and a box marks a box, with nothing here knowing which
%   is which.
%
%   THE RULE CHANGED WITH IT, AND THE SHAPE FIX DOES NOT WORK WITHOUT THAT. This used
%   to mark a cell if ANY point of a footprint fell inside it, sampled at a half-cell
%   step. Under that rule a round stool marks every cell of its bounding square
%   anyway: the nearest point of the corner cell is 0.1414 m from the centre against a
%   0.15 m radius, so it is inside, and no sampling density would have changed it. The
%   cell-centre rule is both the only one that can express a circle at 0.05 m cells
%   and the one the SHELL half of this grid has always used - probeFloorPlan casts one
%   ray per cell centre - so the props were the inconsistent half all along.
%
%   THE BOUNDING RECTANGLE STAYS AS THE FALLBACK, under the old touch rule, for any
%   prop the sweep could not measure: no assets, no Sim3D, a missing mesh. That is
%   deliberately the conservative direction - too much occupied, never too little -
%   and the fprintf below says how many props took it, because a map quietly built
%   entirely from fallbacks is the failure this could otherwise hide.

nMarked = 0;
[boxes, names] = hmmFloorObstacles(cfg);
if isempty(boxes)
    fprintf(['makeNavMap: no floor-standing props found in the manifest, so the map ' ...
             'holds the shell alone.\n']);
    return
end

before = nnz(grid >= cfg.map.occupiedThreshold);
step   = 1 / cfg.map.resolution / 2;
nOut   = 0;

% THE MEASUREMENT FIRST, AND IT IS ALLOWED TO FAIL. hmmPropFootprints needs the
% prop meshes and a Sim3D world; a machine with neither gets empty points back and
% every prop falls through to its rectangle, which is what this script did before.
measured = repmat({zeros(0, 2)}, size(boxes, 1), 1);
shapeInfo = struct([]);
try
    [measured, ~, shapeInfo] = hmmPropFootprints(cfg);
catch err
    fprintf(['makeNavMap: the prop footprint sweep did not run (%s), so every prop ' ...
             'is stamped as its bounding rectangle.\n'], err.identifier);
end

pts = cell(size(boxes, 1), 1);
nRect = 0;
for k = 1:size(boxes, 1)
    if k <= numel(measured) && ~isempty(measured{k})
        pts{k} = measured{k};
    else
        pts{k} = sampleBox(boxes(k, :), step);
        nRect = nRect + 1;
    end
end

for k = 1:numel(pts)
    xy = pts{k};
    % A PROP OUTSIDE THE GRID IS DROPPED, NOT AN ERROR. The navigation grid is 12 x 12 m
    % from cfg.map.origin and the manifest is free to place a prop anywhere; setOccupancy
    % would throw on the first out-of-range point and take the whole capture with it.
    [~, in] = getOccupancy(map, xy);
    nOut = nOut + nnz(~in);
    if any(in)
        setOccupancy(map, xy(in, :), cfg.map.obsOccupied);
    end
end

grid    = occupancyMatrix(map);
nMarked = nnz(grid >= cfg.map.occupiedThreshold) - before;

fprintf(['makeNavMap: marked %d cells occupied under %d floor-standing props (%s)' ...
         ' - %d were already occupied by the shell.\n'], nMarked, size(boxes, 1), ...
    strjoin(cellstr(names), ', '), countCovered(pts, cfg, map) - nMarked);
if nRect > 0
    fprintf(['makeNavMap: %d of %d props were not measured and were stamped as ' ...
             'bounding rectangles.\n'], nRect, size(boxes, 1));
end
% THE RATIO IS THE EVIDENCE THAT THE SHAPE WAS MEASURED AT ALL. A square prop reads
% near 1.00 and a cylinder near pi/4 = 0.785, so this line is how the reader checks
% that the sweep did something rather than quietly reproducing the rectangle.
% The sealed count is worth printing because it is the one number that says a mesh
% had an opening the rays went through - see hmmPropFootprints. Nonzero
% is normal for a hollow asset and does not mean anything went wrong.
for k = 1:numel(shapeInfo)
    fprintf('makeNavMap: %-16s footprint %.4f of its bounding box (%d rays, %d sealed)\n', ...
        shapeInfo(k).name, shapeInfo(k).ratio, shapeInfo(k).rays, shapeInfo(k).sealed);
end
if nOut > 0
    fprintf('makeNavMap: %d prop sample points fell outside the navigation grid.\n', nOut);
end
end

%% ------------------------------------------------------------------------
function n = countCovered(pts, cfg, map)
%COUNTCOVERED How many grid cells the prop footprints touch, occupied or not.
%   Reported so the "already occupied by the shell" figure above is a measurement
%   rather than an inference - a prop pushed against a wall marks fewer new cells
%   than its footprint covers, and that is information, not an error.
%
%   TAKES THE SAME POINT SETS THE STAMP USED, rather than re-deriving them from the
%   boxes. It used to call sampleBox a second time, which was harmless while both
%   were rectangles and would now count a square where a disc was stamped - a
%   "already occupied by the shell" figure larger than the truth by the corners.
idx = [];
for k = 1:numel(pts)
    xy = pts{k};
    [~, in] = getOccupancy(map, xy);
    if any(in)
        ij = world2grid(map, xy(in, :));
        idx = [idx; sub2ind(cfg.map.gridSize, ij(:, 1), ij(:, 2))];    %#ok<AGROW>
    end
end
n = numel(unique(idx));
end

%% ------------------------------------------------------------------------
function xy = sampleBox(b, step)
%SAMPLEBOX World points covering a [cx cy sx sy] footprint, FACES INCLUDED.
%
%   WHY linspace AND NOT A COLON. b(1)-sx/2 : step : b(1)+sx/2 stops short of the far
%   face whenever the span is not a whole number of steps, and it does so silently. The
%   stools are 0.300 m at a 0.025 m step and come out exact; the 0.220 m SoccerBall is
%   8.8 steps, so the colon form stopped 0.020 m inside its +x and +y faces and left 11
%   of its 36 cells unmarked. That was found by round-tripping the footprint through the
%   map and counting occupied cells, NOT by the cell total, which looked entirely
%   plausible at 123. linspace pins both faces and only ever refines the step.
n = max(2, ceil(b(3:4) ./ step) + 1);
[XX, YY] = meshgrid(linspace(b(1) - b(3) / 2, b(1) + b(3) / 2, n(1)), ...
                    linspace(b(2) - b(4) / 2, b(2) + b(4) / 2, n(2)));
xy = [XX(:) YY(:)];
end

%% ------------------------------------------------------------------------
function [grid, nOpened] = carveDoorways(grid, cfg, map)
%CARVEDOORWAYS Mark the measured door openings free, through the map's own coordinates.
%
%   setOccupancy is used rather than row/column arithmetic on purpose. The grid's row
%   index runs DOWNWARD in y while its column index runs upward in x, and every place
%   in this project that has written that conversion by hand has been a place it could
%   be got wrong. Handing world metres to the map makes the convention the toolbox's
%   problem. The sample step is half a cell so no cell inside a span can be stepped
%   over, and duplicates are harmless.

nOpened = 0;
if ~isfield(cfg.map, 'doorways') || isempty(cfg.map.doorways)
    return
end

before = nnz(grid <= cfg.map.freeThreshold);
for k = 1:size(cfg.map.doorways, 1)
    span = cfg.map.doorways(k, :);
    step = 1 / cfg.map.resolution / 2;
    [XX, YY] = meshgrid(span(1):step:span(2), span(3):step:span(4));
    setOccupancy(map, [XX(:) YY(:)], cfg.map.obsFree);
end
grid = occupancyMatrix(map);
nOpened = nnz(grid <= cfg.map.freeThreshold) - before;

names = cfg.map.doorwayNames;
if numel(names) ~= size(cfg.map.doorways, 1)
    names = "doorway " + string(1:size(cfg.map.doorways, 1));
end
fprintf('makeNavMap: opened %d cells across %d measured doorways (%s).\n', ...
    nOpened, size(cfg.map.doorways, 1), strjoin(cellstr(names), ', '));
end

%% ------------------------------------------------------------------------
function tf = covers(fp, xLim, yLim)
%COVERS Does this floor plan span the whole navigation grid?
%   Compared with a one-cell tolerance, because the sweep lands on cell centres
%   while the grid limits are cell edges, and demanding exact equality would
%   re-sweep a perfectly good plan every time.
tol = 1 / fp.resolution;
tf = fp.world(1, 1) <= xLim(1) + tol && fp.world(1, 2) >= xLim(2) - tol && ...
     fp.world(2, 1) <= yLim(1) + tol && fp.world(2, 2) >= yLim(2) - tol;
end

%% ------------------------------------------------------------------------
function [grid, detail] = fromCameras(cfg, stopTime)
%FROMCAMERAS What the model's own four fixed cameras build during a short run.
%
%   A RUN IS NEEDED BECAUSE THE GRID IS NOT IN THE WORKSPACE. NavMapGrid is a Data
%   Store Memory internal to the model and is not one of the logged signals, so no
%   previous run left it anywhere; after sim() returns the only copy has been
%   freed. And no model edit is needed to get it: a Data Store Memory carries a
%   DataLogging dialog parameter, so logging is switched on through a
%   SimulationInput, which the .slx never sees.
%
%   THIS PATH IS SUBJECT TO A KNOWN LIMITATION and will look far worse than the
%   header's coverage table predicts. SceneMapping sits inside the Send-triggered
%   Perception subsystem, so the fold fires once per grounding request rather than
%   at cfg.rates.perception - a 0.5 s run logs exactly one write, at t = 0.040.
%   One fold leaves a cell at exactly p = 0.5, because from a 0.10 prior a single
%   occupied observation lands there and it takes two to cross 0.65, so this
%   returns about 34 occupied cells rather than the 1906 the same returns reach
%   when folded twice. cfg.map.mode must be 'live' for this to fold at all, since
%   hmmOccupancyFromDepth is a pass-through under 'fixed'.

mdl = 'homeMobileManipulator';

if strcmp(cfg.map.mode, 'fixed')
    error('makeNavMap:foldDisabled', ...
        ['cfg.map.mode is ''fixed'', so hmmOccupancyFromDepth returns its prior ' ...
         'unchanged and a camera capture would record only the map it started ' ...
         'from. Set cfg.map.mode = ''live'' to capture from the cameras.']);
end

% A clean model is closed and reloaded before the run, because the masked
% Robotics System Toolbox blocks read mask-workspace variables, mask init runs on
% load and compile but not on an assignin, so a model loaded against an older cfg
% keeps it. A dirty model is left alone - discarding someone's unsaved work to
% capture a map is the wrong trade.
if bdIsLoaded(mdl) && strcmp(get_param(mdl, 'Dirty'), 'off')
    close_system(mdl, 0);
end
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

% FIND THE STORE BY ITS NAME, not by a remembered path. Data Store Memory blocks
% are moved during refactoring far more often than they are renamed, and a
% hard-coded path fails as a confusing "block not found" months later.
dsm = find_system(mdl, 'MatchFilter', @Simulink.match.allVariants, ...
    'BlockType', 'DataStoreMemory', 'DataStoreName', 'NavMapGrid');
if numel(dsm) ~= 1
    error('makeNavMap:storeNotFound', ...
        ['Expected exactly one Data Store Memory named NavMapGrid in %s, found %d. ' ...
         'If the store was renamed, update this script rather than the model.'], ...
        mdl, numel(dsm));
end

in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(stopTime), 'Timeout', 3600);
in = in.setBlockParameter(dsm{1}, 'DataLogging', 'on');
% Cap what is retained. The grid is 240 x 240 doubles - 460 kB a sample, and at the
% perception rate a long run would page gigabytes to keep values that are all
% superseded by the last one.
in = in.setBlockParameter(dsm{1}, 'DataLoggingDecimateData', 'off');
in = in.setBlockParameter(dsm{1}, 'DataLoggingLimitDataPoints', 'off');

fprintf('makeNavMap: running %s for %g s of model time to capture NavMapGrid ...\n', ...
    mdl, stopTime);
t0  = tic;
out = sim(in);
wall = toc(t0);
fprintf('makeNavMap: %.1f s wall clock, stop event %s.\n', ...
    wall, out.SimulationMetadata.ExecutionInfo.StopEvent);

grid   = extractLastGrid(out);
detail = struct('stopTime', stopTime, 'wallSeconds', wall, ...
    'perceptionRate', cfg.rates.perception, 'rayGrid', cfg.depth.rayGrid, ...
    'renderQuantum', cfg.depth.renderQuantum);
end

%% ------------------------------------------------------------------------
function grid = extractLastGrid(out)
%EXTRACTLASTGRID Pull the final NavMapGrid value out of a simulation output.
%   Data store logs arrive in a Simulink.SimulationData.Dataset whose element name
%   is the store name; the value is a timeseries whose Data is [nSample 240 240].
%   The last sample is the accumulated map - log odds only ever integrate, so later
%   is strictly better informed than earlier.
names = {'dsmout', 'NavMapGrid'};
ds = [];
for k = 1:numel(names)
    if isprop(out, names{k}) || isfield(out, names{k})
        ds = out.(names{k});
        break
    end
end
if isempty(ds)
    error('makeNavMap:noLog', ...
        ['The run produced no data store log. Available outputs: %s. ' ...
         'DataLogging was requested on the NavMapGrid store; if this model routes ' ...
         'data store logs elsewhere, add that name to extractLastGrid.'], ...
        strjoin(string(out.who), ', '));
end

el = ds.get('NavMapGrid');
if isempty(el)
    el = ds{1};
end
data = el.Values.Data;

% Squeeze the leading time dimension. A 3-D log is [nSample rows cols]; a
% single-sample log can arrive already 2-D, which is why this is a size test rather
% than an unconditional index.
if ndims(data) == 3
    grid = double(squeeze(data(end, :, :)));
else
    grid = double(data);
end
end
