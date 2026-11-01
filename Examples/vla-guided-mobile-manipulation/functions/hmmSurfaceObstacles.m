function [boxes, tops, tall] = hmmSurfaceObstacles(mountXY, baseXY, baseYaw, cfg, surfTop)
%HMMSURFACEOBSTACLES The furniture within arm reach, as a few axis-aligned boxes.
%
%   boxes = HMMSURFACEOBSTACLES(mountXY, baseXY, baseYaw, cfg) returns an Nx4
%   matrix of [centreX centreY sizeX sizeY] in WORLD coordinates, one row per box,
%   covering the occupancy grid's occupied cells within arm reach of the arm mount.
%   N is 0 when there is no map. baseYaw is in RADIANS, matching
%   RobotState.BasePose(3) and hmmScreenArmPlan's robotConfig(3); hmmReportRun
%   prints that field in degrees, which is a report format and not the signal.
%
%   [boxes, tops] = HMMSURFACEOBSTACLES(...) also returns an Nx1 vector of box TOPS
%   in world metres, NaN where the height is unknown. NaN is the normal case and
%   means "hang this one from the support surface, as before"; a finite entry is a
%   floor-standing prop whose real top the manifest knows.
%
%   FOOTPRINTS ONLY, EXCEPT WHERE THE MANIFEST KNOWS BETTER. An occupancy grid has no
%   height information at all, so for a cell fitted out of the grid this function has
%   nothing honest to say about z and returns NaN: the caller knows the fitted surface
%   top and hangs a slab of cfg.plan.surfaceBoxDepth BELOW it rather than extruding
%   down to the floor; see hmmScreenArmPlan, which measured that a floor-to-top column
%   traps the arm's own upper links at every depth and makes the place unplannable,
%   while a slab under the worktop catches the carried object.
%
%   THE FLOOR PROPS ARE THE EXCEPTION, AND THEY HAD TO BECOME ONE. makeNavMap now marks
%   the floor-standing props (hmmFloorObstacles) as occupied so the base stops routing
%   through them. Those cells then arrived here and were fitted like any other, and the
%   caller hung them from the support surface: a box fitted to a 0.66 m stool next to a
%   0.91 m worktop measured z = [0.660, 0.910] - 0.250 m of pure air standing on the
%   seat, with the stool's real body below it still unmodelled. That is the TALL
%   direction, the one that turns an avoided collision into a plan that cannot be made.
%   So the prop cells are taken OUT of the greedy cover and each in-reach prop is
%   appended instead as its own exact manifest footprint carrying its own top.
%
%   WHY APPEND FROM THE MANIFEST RATHER THAN CAP WHAT THE COVER FOUND. Because asking
%   "is this fitted box inside a prop footprint" depends on cell quantisation and on
%   the cover's tie-breaks. A 0.300 m stool whose faces land exactly on cell edges is
%   marked over SEVEN cells - 0.350 m, a quarter of a cell past each face - so a strict
%   containment test would cap almost nothing and a loosened one would be guesswork.
%   The manifest footprint is exact, strictly tighter than the cells it replaces, and
%   names no prop.
%
%   WHY THIS EXISTS. hmmScreenArmPlan's only model of the support surface is a
%   proxy slab sized from the OBJECT's footprint - cfg.plan.surfaceBoxSideLim
%   clamps it to at most 0.250 m - centred on the place point. It is a stand-in for
%   the patch the object is about to be set down on, and for that it is the right
%   shape. It is not a counter. Measured on the 40 s run of 2026-09-30, the carried
%   mug passed 85.6 mm INSIDE the kitchen counter, 0.348 m from the place point and
%   therefore 0.258 m outside the slab, during the BiRRT-screened free move. The
%   screen was not wrong; it had nothing to screen against.
%
%   WHY NOT JUST MAKE THE SLAB BIGGER. Because a bigger slab is still centred on
%   the place point, so it grows BACKWARDS over the parked chassis and the stowed
%   gripper as fast as it grows forwards over the counter. cfg.plan.surfaceBoxSideLim
%   records what happens: at 0.60 m the stowed gripper is inside the invented patch
%   and manipulatorRRT rejects the START configuration, so nothing plans at all.
%   That objection is to an INVENTED CENTRED patch, and it does not apply here.
%
%   WHY THE MAP IS ALLOWED TO ANSWER. The occupancy grid is the only thing in this
%   project that knows where the counter is, and boxes fitted to its occupied cells
%   sit where the furniture is rather than where the object is. That is also why
%   they cannot swallow the robot: the base planner drove to this pose through the
%   SAME grid, inflated by hmmBaseInflation, so the parked chassis stands in free
%   cells by construction. Measured on the shipped place pose, the nearest occupied
%   cell is 0.4171 m from the base centre against a 0.3183 m half-diagonal - 99 mm
%   of clearance, not luck. The cell-level exclusion below makes it structural
%   anyway rather than relying on that margin holding for every pose.
%
%   THE GRID DOES NOT DISTINGUISH A COUNTER FROM A WALL, and the caller's bounded slab
%   is what makes that safe. A wall or a fridge inside the reach radius comes back as a
%   footprint like any other and is then modelled only as a band just under counter
%   height. The error is one-directional: a box that is too SHORT can let a path
%   through, which is no worse than today's nothing-at-all, whereas a box that is too
%   TALL blocks the pre-place pose, which has to sit above the surface with the arm
%   leaning over it, and turns a collision avoided into a plan that cannot be made.
%   hmmScreenArmPlan has the measurement of exactly that failure.
%
%   A FEW BOXES, NOT A CELL EACH. 192 occupied cells within reach on the shipped
%   pose; as 192 collision boxes that is a collision check no planner should be
%   asked to pay for. Greedy maximal-rectangle covers them with SIX, exactly, with
%   no cells left over, because a counter is a handful of rectangles. The cap is
%   cfg.plan.surfaceObstacleMaxBoxes; when the cover needs more than that the
%   remainder is dropped and a warning is issued, which degrades towards today's
%   behaviour rather than towards a stall. The appended props reserve their own room
%   inside that cap, so adding them cannot silently push the counter out of the cover.
%
%   See also hmmScreenArmPlan, hmmFloorObstacles, hmmLoadNavMap, hmmBaseInflation.

%   Copyright 2026 The MathWorks, Inc.

boxes = zeros(0, 4);
tops  = zeros(0, 1);
tall  = false(0, 1);

grid = cachedNavGrid(cfg);
if isempty(grid)
    return          % 'live' mapping mode, or no shipped map: the caller keeps its slab
end
occ = cachedOccupancyMap(grid, cfg);

mountXY = double(mountXY(:)).';
baseXY  = double(baseXY(:)).';
res     = double(cfg.map.resolution);

% Sample the grid on its own lattice inside the reach disc. The window is world
% aligned because the boxes are, and a rotated window would only add cells the
% radius test then throws away.
R  = double(cfg.arm.reach) + double(cfg.plan.surfaceBoxMargin);
xs = (mountXY(1) - R) : 1/res : (mountXY(1) + R);
ys = (mountXY(2) - R) : 1/res : (mountXY(2) + R);
[X, Y] = meshgrid(xs, ys);
P = [X(:), Y(:)];
P = P(vecnorm(P - mountXY, 2, 2) <= R, :);

% The floor-standing props in reach, collected FIRST because they both reserve room
% in the box cap and remove cells from the cover. A prop counts as in reach when the
% nearest point of its rectangle is inside the disc, not its centre: a stool whose
% near face is 0.1 m away and whose centre is 0.95 m away is still something the arm
% can hit. They are appended even when the grid knows nothing about them, because the
% manifest is the authority on a prop standing on the floor and the map may predate it.
[nearBox, nearTop, okProp] = propsInReach(mountXY, R, baseXY, baseYaw, cfg);
propBox = nearBox(okProp, :);
propTop = nearTop(okProp);
nProp   = size(propBox, 1);

if isempty(P)
    boxes = propBox;        % reach disc off the lattice; the props are still real
    tops  = propTop;
    tall  = false(size(propTop));
    return
end

% Cells outside the map read as free rather than throwing, so a reach disc that
% overhangs the grid edge degrades to "nothing there" instead of failing the plan.
[~, inMap] = getOccupancy(occ, P);   % second output: inside the map, edges included
P = P(inMap, :);
if isempty(P)
    boxes = propBox;
    tops  = propTop;
    tall  = false(size(propTop));
    return
end
keep = checkOccupancy(occ, P) > 0;

% THE ROBOT'S OWN FOOTPRINT IS EXCLUDED AT CELL LEVEL, not by discarding a fitted
% box afterwards. A box that contains the arm mount would put the arm's own base
% links inside an obstacle and manipulatorRRT would reject the start configuration -
% the exact failure cfg.plan.surfaceBoxSideLim warns about, arrived at from the other
% direction. This cannot normally fire: the base planner's inflation guarantees the
% clearance. If it ever does, the chassis is already parked in the furniture, which is
% a navigation defect and not something this function should convert into a dead plan.
c = cos(baseYaw); s = sin(baseYaw);
Rz = [c, -s; s, c];
Pb = (P - baseXY) * Rz;                      % world -> body
half = double([cfg.base.chassisSize(1), cfg.base.chassisSize(2)]) / 2 + ...
       double(cfg.plan.surfaceBoxMargin);
inChassis = abs(Pb(:, 1)) <= half(1) & abs(Pb(:, 2)) <= half(2);
if any(keep & inChassis)
    warning('hmmSurfaceObstacles:chassisInOccupied', ...
        ['%d occupied cell(s) fall inside the parked chassis footprint; excluding them ' ...
         'so the arm plan still has a valid start configuration.'], sum(keep & inChassis));
end
keep = keep & ~inChassis;

% THE PROP CELLS LEAVE THE COVER, for the reason in the header: fitted out of the grid
% they have no height and the caller would hang them from the worktop. The margin is one
% full cell, not a hairline, because the marking in makeNavMap reaches up to half a cell
% past each face and a tie on a cell edge decides which side gets it. Over-excluding
% costs at most a 0.05 m strip of real furniture pressed against a prop, which is the
% SHORT direction and no worse than what shipped before these boxes existed; under-
% excluding would leave exactly the phantom slab this is here to remove.
for k = 1:size(nearBox, 1)
    b   = nearBox(k, :);
    tol = 1 / res;
    keep = keep & ~(abs(P(:, 1) - b(1)) <= b(3) / 2 + tol & ...
                    abs(P(:, 2) - b(2)) <= b(4) / 2 + tol);
end

if ~any(keep)
    boxes = propBox;
    tops  = propTop;
    tall  = false(size(propTop));
    return
end

% Onto a local logical lattice so the rectangles are exact in cell units.
Pk = P(keep, :);
gx = round((Pk(:, 1) - min(xs)) * res) + 1;
gy = round((Pk(:, 2) - min(ys)) * res) + 1;

% FURNITURE THAT RISES ABOVE THE SUPPORT SURFACE GETS ITS OWN BOXES, WITH ITS REAL TOP.
% Every grid box below is hung from the support surface by the caller, which is right for a
% counter and wrong for a couch: placing on the 0.40 m seat capped the 0.67 m BACKREST at 0.40,
% and the gripper passed 0.10-0.12 m through it reaching over (2026-10-16).
% The occupancy grid has no heights, so these come from the nadir height map
% (data/shellProbe/floorPlan.mat): cells between cfg.plan.tallBoxMinRise and
% cfg.plan.tallBoxMaxRise above the surface. The upper bound is what keeps this honest - the
% sweep records only the TOPMOST hit, so a wall cabinet over a worktop reads as a column from
% the worktop up, with real free space beneath it; a rise that large is left to the old rule
% rather than invented into a wall. Backrests, armrests and headboards sit well inside it.
tallM = false(0, 0);
tallH = zeros(0, 0);
nTallMax = 0;
if nargin >= 5 && isfinite(surfTop) && surfTop > 0.01
    hm = cachedHeightMap();
    if ~isempty(hm)
        hr = round((hm.world(2, 2) - Pk(:, 2)) * hm.resolution) + 1;
        hc = round((Pk(:, 1) - hm.world(1, 1)) * hm.resolution) + 1;
        okH = hr >= 1 & hr <= size(hm.heightMap, 1) & hc >= 1 & hc <= size(hm.heightMap, 2);
        h = NaN(size(Pk, 1), 1);
        h(okH) = hm.heightMap(sub2ind(size(hm.heightMap), hr(okH), hc(okH)));
        rise = h - surfTop;
        isTall = isfinite(rise) & rise > cfg.plan.tallBoxMinRise & rise <= cfg.plan.tallBoxMaxRise;
        if any(isTall)
            tallM = false(max(gy), max(gx));
            tallH = zeros(max(gy), max(gx));
            tallM(sub2ind(size(tallM), gy(isTall), gx(isTall))) = true;
            tallH(sub2ind(size(tallH), gy(isTall), gx(isTall))) = h(isTall);
            nTallMax = ceil(double(cfg.plan.surfaceObstacleMaxBoxes) / 2);
        end
    end
end

M  = false(max(gy), max(gx));
M(sub2ind(size(M), gy, gx)) = true;

% The props take their room out of the cap rather than competing with the cover for it.
% At least one box is always left for the cover so a reach disc crowded with props
% cannot hide the counter completely. Tall boxes take theirs first, at most half the cap;
% any tall cell they do not cover falls back to the ordinary cover below.
maxBoxes = max(1, double(cfg.plan.surfaceObstacleMaxBoxes) - nProp);
tallRects = zeros(0, 4);
if nTallMax > 0
    [tallRects, ~] = greedyRectCover(tallM, min(nTallMax, maxBoxes - 1));
    for i = 1:size(tallRects, 1)
        r = tallRects(i, :);
        M(r(1):r(3), r(2):r(4)) = false;
    end
end
[rects, leftover] = greedyRectCover(M, maxBoxes - size(tallRects, 1));
if leftover > 0
    warning('hmmSurfaceObstacles:coverIncomplete', ...
        ['%d of %d occupied cells within reach are not covered by %d boxes; the arm ' ...
         'plan is screened against the rest only.'], leftover, sum(M(:)), maxBoxes);
end

toWorld = @(r) [min(xs) + (r(2) - 1) / res - 0.5 / res, min(xs) + (r(4) - 1) / res + 0.5 / res, ...
                min(ys) + (r(1) - 1) / res - 0.5 / res, min(ys) + (r(3) - 1) / res + 0.5 / res];
boxes = zeros(size(rects, 1), 4);
for i = 1:size(rects, 1)
    w = toWorld(rects(i, :));                % [x0 x1 y0 y1]
    boxes(i, :) = [(w(1) + w(2)) / 2, (w(3) + w(4)) / 2, w(2) - w(1), w(4) - w(3)];
end
tallBox = zeros(size(tallRects, 1), 4);
tallTop = zeros(size(tallRects, 1), 1);
for i = 1:size(tallRects, 1)
    r = tallRects(i, :);
    w = toWorld(r);
    tallBox(i, :) = [(w(1) + w(2)) / 2, (w(3) + w(4)) / 2, w(2) - w(1), w(4) - w(3)];
    sub = tallH(r(1):r(3), r(2):r(4));
    tallTop(i) = max(sub(:));                % the highest cell it covers
end

% Grid-fitted boxes have no height; the props carry their own. The caller reads the
% NaN as "hang it from the support surface", which is what every box did before. The
% tall boxes carry their measured top and are flagged, so the caller builds them from the
% surface UP to that top instead of capping them at it.
tops  = [NaN(size(boxes, 1), 1); tallTop; propTop];
tall  = [false(size(boxes, 1), 1); true(size(tallBox, 1), 1); false(nProp, 1)];
boxes = [boxes; tallBox; propBox];
end

function hm = cachedHeightMap()
%CACHEDHEIGHTMAP The nadir-sweep surface heights, loaded once per session, or empty.
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

%% ------------------------------------------------------------------------
function [nearBox, nearTop, okProp] = propsInReach(mountXY, R, baseXY, baseYaw, cfg)
%PROPSINREACH The floor-standing props the arm could reach, with their real tops.
%   nearBox is Mx4 [centreX centreY sizeX sizeY], nearTop is Mx1 world metres, and
%   okProp is Mx1 logical - false for a prop that overlaps the parked chassis, which
%   must not become a collision box because the arm's own base links are inside it.
%   Every row of nearBox has its cells taken out of the cover whatever okProp says:
%   where the robot is standing in the furniture, modelling nothing is the only answer
%   that leaves a valid start configuration, and hmmSurfaceObstacles:chassisInOccupied
%   has already said so.
[allBox, ~, allTop] = cachedFloorProps(cfg);
nearBox = zeros(0, 4);
nearTop = zeros(0, 1);
okProp  = false(0, 1);
if isempty(allBox)
    return
end

% Nearest point of the rectangle to the mount, which is what the arm actually meets.
d = max(abs(allBox(:, 1:2) - mountXY) - allBox(:, 3:4) / 2, 0);
near = vecnorm(d, 2, 2) <= R;

nearBox = allBox(near, :);
nearTop = allTop(near);
okProp  = true(size(nearBox, 1), 1);

half = double([cfg.base.chassisSize(1), cfg.base.chassisSize(2)]) / 2;
for k = 1:size(nearBox, 1)
    if rectsOverlap(nearBox(k, 1:2), nearBox(k, 3:4) / 2, baseXY, half, baseYaw)
        okProp(k) = false;
        warning('hmmSurfaceObstacles:propInChassis', ...
            ['a floor prop at [%.3f %.3f] overlaps the parked chassis footprint; it is ' ...
             'left out of the arm obstacles so the plan still has a valid start.'], ...
            nearBox(k, 1), nearBox(k, 2));
    end
end
end

%% ------------------------------------------------------------------------
function tf = rectsOverlap(cA, hA, cB, hB, yawB)
%RECTSOVERLAP Separating-axis test, rectangle A world aligned, B rotated by yawB.
%   Four axes are sufficient and exact for two rectangles; a corner-containment test
%   is not, which matters for a long prop lying across the chassis.
u = [cos(yawB), sin(yawB)];
v = [-sin(yawB), cos(yawB)];
axesToTest = [1 0; 0 1; u; v];
dc = double(cB(:)).' - double(cA(:)).';
tf = true;
for i = 1:size(axesToTest, 1)
    a  = axesToTest(i, :);
    rA = hA(1) * abs(a(1)) + hA(2) * abs(a(2));
    rB = hB(1) * abs(dot(a, u)) + hB(2) * abs(dot(a, v));
    if abs(dot(dc, a)) > rA + rB
        tf = false;
        return
    end
end
end

%% ------------------------------------------------------------------------
function [boxes, names, tops] = cachedFloorProps(cfg)
%CACHEDFLOORPROPS The manifest's floor-standing props once per session.
%   hmmFloorObstacles reads the manifest from disk and the manifest does not change
%   during a run; the arm screen asks on every place leg. Same pattern, and the same
%   reason, as cachedNavGrid below.
persistent held
if isempty(held)
    [b, n, t] = hmmFloorObstacles(cfg);
    held = {b, n, t};
end
boxes = held{1};
names = held{2};
tops  = held{3};
end

%% ------------------------------------------------------------------------
function grid = cachedNavGrid(cfg)
%CACHEDNAVGRID The shipped nav map once per session, or empty when there is none.
%   The cell wrapper distinguishes "not looked up yet" from "looked up and there is
%   no map", which a bare [] cannot. Same pattern, and the same reason, as
%   hmmReconstructTargets/cachedNavGrid.
persistent held
if isempty(held)
    if isfield(cfg.map, 'mode') && strcmp(cfg.map.mode, 'live')
        held = {[]};
    else
        held = {hmmLoadNavMap(cfg)};
    end
end
grid = held{1};
end

%% ------------------------------------------------------------------------
function occ = cachedOccupancyMap(grid, cfg)
%CACHEDOCCUPANCYMAP The thresholded grid as a binaryOccupancyMap, built once.
%   Going through the object rather than deriving a row/column rule here is
%   deliberate: hmmPlanBasePath reads the same grid the same way, and two hand-rolled
%   indexing conventions that disagree by one cell would be invisible until a plan
%   grazed a wall.
persistent held
if isempty(held)
    o = binaryOccupancyMap(grid > cfg.map.occupiedThreshold, cfg.map.resolution);
    o.GridLocationInWorld = cfg.map.origin;
    held = {o};
end
occ = held{1};
end

%% ------------------------------------------------------------------------
function [rects, leftover] = greedyRectCover(M, maxBoxes)
%GREEDYRECTCOVER Cover the true cells of M with few axis-aligned rectangles.
%   Repeatedly take the largest all-true rectangle and clear it. The optimal cover
%   is NP-hard and not worth chasing: furniture is rectangles, and on the shipped
%   place pose greedy returns six boxes with zero cells left over.
rects = zeros(0, 4);
W = M;
for it = 1:maxBoxes
    if ~any(W(:)), break, end
    [r0, c0, r1, c1] = largestRect(W);
    rects(end + 1, :) = [r0, c0, r1, c1];  %#ok<AGROW>
    W(r0:r1, c0:c1) = false;
end
leftover = sum(W(:));
end

function [br0, bc0, br1, bc1] = largestRect(W)
%LARGESTRECT Largest all-true axis-aligned rectangle, histogram method per row.
[nr, nc] = size(W);
h = zeros(1, nc);
best = 0; br0 = 1; bc0 = 1; br1 = 1; bc1 = 1;
for r = 1:nr
    h(W(r, :))  = h(W(r, :)) + 1;
    h(~W(r, :)) = 0;
    st = zeros(1, nc + 1); sp = 0;
    for cc = 1:nc + 1
        cur = 0;
        if cc <= nc, cur = h(cc); end
        while sp > 0 && h(st(sp)) >= cur
            top = st(sp); sp = sp - 1;
            left = 1;
            if sp > 0, left = st(sp) + 1; end
            area = h(top) * (cc - left);
            if area > best
                best = area;
                br1 = r; br0 = r - h(top) + 1; bc0 = left; bc1 = cc - 1;
            end
        end
        sp = sp + 1; st(sp) = cc;
    end
end
end
