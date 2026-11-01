function report = probeFloorPlan(varargin)
%PROBEFLOORPLAN The apartment's navigable floor, ray-traced nadir against the mesh.
%
%   report = PROBEFLOORPLAN() casts a grid of straight-down rays over the whole
%   configured scene extent, records the exact world height of the first surface
%   each one meets, and calls a cell navigable when that surface is the bare
%   floor. It then reports the clearance of every room datum, of the configured
%   start pose, and of the best free cell in each room, and says which of those
%   are connected to one another for a chassis of the configured size.
%
%   WHY NADIR RAYS AND NOT AN OVERHEAD RENDER. An overhead camera at any
%   practical height is a perspective camera, so a 2.6 m wall projects its own
%   top surface over up to a metre of the floor behind it and that floor reads as
%   occupied. Straight-down rays have no such smear: a wall covers exactly the
%   cells it stands on. Ray-traced range is also metrically exact and flags rays that
%   hit nothing, which a camera's render depth cannot.
%
%   THIS IS THE WITNESS THE START POSE NEEDED AND COULD NOT GET FROM THE RUN. The
%   run's own occupancy grid, which in the recorded run held 27 occupied cells in
%   57600, was too thin to distinguish a displaced shell from rays that never
%   reached one. This probe measures the mesh directly, so it answers regardless
%   of how the run's perception behaved.
%
%   report fields:
%       .heightMap      [nr nc] m, height of the first surface, NaN where no hit
%       .navigable      [nr nc] logical, bare floor
%       .clearance      [nr nc] m, distance from each free cell to the nearest
%                       non-free cell
%       .world          [xlo xhi; ylo yhi] m spanned by the map
%       .resolution     cells/m
%       .startClearance m at cfg.base.initialPose
%       .rooms          per-room struct: datum clearance, best cell, its clearance
%       .connected      logical matrix, room i reachable from room j
%       .recommended    [x y] the pose this probe recommends starting from

%   Copyright 2026 The MathWorks, Inc.

p = inputParser;
p.addParameter('Resolution', 20, @(x) isnumeric(x) && isscalar(x));
p.addParameter('Margin', 0.5, @(x) isnumeric(x) && isscalar(x));
p.addParameter('Origin', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 2));
p.addParameter('OutDir', '', @(s) ischar(s) || isstring(s));
p.parse(varargin{:});
res    = p.Results.Resolution;
margin = p.Results.Margin;
origin = p.Results.Origin;

root = fileparts(fileparts(mfilename('fullpath')));
d    = hmmSetup();
cfg  = d.cfg;

outDir = char(p.Results.OutDir);
if isempty(outDir)
    outDir = fullfile(root, 'data', 'shellProbe');
end
if ~isfolder(outDir)
    mkdir(outDir);
end

meshFile = '';
roots = hmmAssetRoots();
for k = 1:numel(roots)
    candidate = fullfile(roots{k}, cfg.scene.staticMesh);
    if isfile(candidate)
        meshFile = candidate;
        break
    end
end
if isempty(meshFile)
    error('probeFloorPlan:noMesh', 'Shell mesh %s not found.', cfg.scene.staticMesh);
end

%% The ray grid
xlo = cfg.scene.extent(1, 1) - margin;  xhi = cfg.scene.extent(1, 2) + margin;
ylo = cfg.scene.extent(2, 1) - margin;  yhi = cfg.scene.extent(2, 2) + margin;

% 'Origin' PHASES THE LATTICE ONTO A CONSUMER'S CELL CENTRES, and a consumer that
% resamples this plan onto its own grid needs it. Without it the samples sit at
% whole multiples of 1/res from the extent edge, which for an occupancyMap whose
% GridLocationInWorld is offset by a whole number of cells lands every ray exactly
% on a cell BOUNDARY. world2grid then breaks that tie in floating point, so
% adjacent rays can round into one cell and leave its neighbour uncarved: measured
% at 3468 of 24449 floor cells lost, and which ones are lost changes with the
% extent, because the arithmetic differs in the last bit. Sampling cell CENTRES
% makes the plan-cell-to-map-cell map a bijection and the result reproducible.
if isempty(origin)
    xs = xlo:1/res:xhi;
    ys = yhi:-1/res:ylo;      % rows run in decreasing y, the map convention
else
    xs = centres(origin(1), res, xlo, xhi);
    ys = flip(centres(origin(2), res, ylo, yhi));
end
% The extent is re-bound to the samples actually cast, because report.world is what
% every consumer reconstructs world coordinates from - makeNavMap does exactly that
% to find each floor cell's centre. Leaving it at the requested extent while the
% samples sit half a cell inside would offset the whole plan by that half cell.
xlo = xs(1);  xhi = xs(end);
ylo = ys(end);  yhi = ys(1);

[XX, YY] = meshgrid(xs, ys);
nRay = numel(XX);

% Origins above every roof and furniture top, aimed straight down. The length
% reaches well below the floor so a ray that returns nothing is genuinely void.
zStart = 4.0;
origins    = [XX(:), YY(:), zStart * ones(nRay, 1)];
directions = repmat([0 0 -1], nRay, 1);
lengths    = (zStart + 1.0) * ones(nRay, 1);

fprintf('shell mesh   %s\n', meshFile);
fprintf('ray grid     %d x %d = %d nadir rays at %d cells/m\n', ...
    numel(ys), numel(xs), nRay, res);

%% Cast
world = sim3d.World('StopTime', 0.2, 'ExecCmds', 'r.Fog 0', 'Update', @capture);
shell = sim3d.Actor('ActorName', 'ApartmentStatic', ...
    'Mobility', sim3d.utils.MobilityTypes.Static);
add(world, shell);
load(shell, meshFile, cfg.scene.meshScale);
propagate(shell, 'Collisions', true);
propagate(shell, 'PreciseContacts', false);

sensor = sim3d.sensors.RaytraceSensor('ActorName', 'NadirScan', ...
    'Translation', [0 0 0], 'Rotation', [0 0 0], ...
    'RayOrigins', origins, 'RayDirections', directions, ...
    'RayLengths', lengths, 'NumberOfBounces', 0);
add(world, sensor);

hitZ = [];
run(world);
delete(world);

if isempty(hitZ)
    error('probeFloorPlan:noCapture', 'The world ran but the ray tracer returned nothing.');
end

%% Height map and navigable floor
report = struct();
report.world      = [xlo xhi; ylo yhi];
report.resolution = res;
report.heightMap  = reshape(hitZ, size(XX));

% Bare floor: the first surface is at floor level. The tolerance is one render
% of slack around z = 0, not a height band - anything standing on the floor, rug
% or stool or wall, takes its cells out of the navigable set.
floorTol = 0.06;
report.navigable = abs(report.heightMap) <= floorTol;

fprintf('\nSURFACE HEIGHTS\n');
z = report.heightMap(:);
fprintf('  rays with a hit    %d of %d\n', sum(isfinite(z)), nRay);
fprintf('  height range       %.2f to %.2f m\n', min(z(isfinite(z))), max(z(isfinite(z))));
fprintf('  bare floor         %d cells (%.1f%% of the extent)\n', ...
    nnz(report.navigable), 100 * nnz(report.navigable) / nRay);

%% Clearance
% bwdist reports distance in cells; one cell is 1/res metres.
report.clearance = bwdist(~report.navigable) / res;

%% What the configuration currently asserts
p0 = reshape(cfg.base.initialPose(1:2), 1, 2);
chassisRadius = hypot(cfg.base.chassisSize(1), cfg.base.chassisSize(2)) / 2;
needed = chassisRadius + cfg.safety.clearanceMin;

report.startClearance = valueAt(report.clearance, p0, report);
report.startHeight    = valueAt(report.heightMap, p0, report);
report.chassisRadius  = chassisRadius;
report.clearanceNeeded = needed;

fprintf('\nTHE CONFIGURED START POSE\n');
fprintf('  cfg.base.initialPose  [%.2f %.2f] m, %.0f deg\n', p0, rad2deg(cfg.base.initialPose(3)));
fprintf('  surface height there  %.2f m\n', report.startHeight);
fprintf('  clearance there       %.2f m\n', report.startClearance);
fprintf('  chassis needs         %.2f m (half-diagonal %.2f + clearanceMin %.2f)\n', ...
    needed, chassisRadius, cfg.safety.clearanceMin);
if report.startClearance < needed
    fprintf('  VERDICT: THE START POSE IS NOT ON NAVIGABLE FLOOR.\n');
else
    fprintf('  VERDICT: the start pose is clear.\n');
end

%% Room datums and the best cell in each room
% "Room" here is the connected free region containing the datum, or - when the
% datum itself is not free - the free region nearest to it.
free = report.clearance >= needed;
lbl  = bwlabel(free, 4);

report.rooms = struct('name', {}, 'datum', {}, 'datumClearance', {}, ...
    'best', {}, 'bestClearance', {}, 'label', {});
fprintf('\nROOM DATUMS AND THEIR BEST FREE CELL\n');
for k = 1:numel(cfg.scene.room)
    c = reshape(cfg.scene.room(k).centre, 1, 2);
    [r, cc] = cellOf(c, report);
    dc = valueAt(report.clearance, c, report);

    % The region to search is the labelled component at the datum when there is
    % one, otherwise the whole map restricted to a 2 m box round the datum.
    if r >= 1 && cc >= 1 && r <= size(lbl, 1) && cc <= size(lbl, 2) && lbl(r, cc) > 0
        thisLabel = lbl(r, cc);
        maskK = lbl == thisLabel;
    else
        thisLabel = 0;
        maskK = boxMask(size(free), c, 2.0, report) & free;
    end
    cand = report.clearance;
    cand(~maskK) = -Inf;
    [bestC, idx] = max(cand(:));
    [br, bcc] = ind2sub(size(cand), idx);
    bestXY = worldOf(br, bcc, report);

    report.rooms(k) = struct('name', cfg.scene.room(k).name, 'datum', c, ...
        'datumClearance', dc, 'best', bestXY, 'bestClearance', bestC, ...
        'label', thisLabel);
    fprintf('  %-12s datum [%5.2f %5.2f] clearance %5.2f m   best [%5.2f %5.2f] clearance %5.2f m\n', ...
        cfg.scene.room(k).name, c, dc, bestXY, bestC);
end

%% Connectivity - can the base actually get from one room to another
n = numel(report.rooms);
report.connected = false(n, n);
for i = 1:n
    for j = 1:n
        li = labelNear(lbl, report.rooms(i).best, report);
        lj = labelNear(lbl, report.rooms(j).best, report);
        report.connected(i, j) = li > 0 && li == lj;
    end
end
fprintf('\nCONNECTIVITY FOR A %.2f m CLEARANCE CHASSIS\n', needed);
fprintf('  %-12s %s\n', '', strjoin(cellfun(@(s) sprintf('%-11s', s), ...
    {report.rooms.name}, 'UniformOutput', false), ' '));
for i = 1:n
    marks = arrayfun(@(b) sprintf('%-11s', ternary(b, 'yes', 'no')), ...
        report.connected(i, :), 'UniformOutput', false);
    fprintf('  %-12s %s\n', report.rooms(i).name, strjoin(marks, ' '));
end

%% A recommendation
% The task starts in the living room - that is where the red mug is - so the
% recommendation is the living room's roomiest cell, reported with the distance
% to the mug so a start pose that leaves nothing to drive is visible as such.
iLiving = find(strcmp({report.rooms.name}, 'LivingRoom'), 1);
if isempty(iLiving)
    iLiving = 1;
end
report.recommended = report.rooms(iLiving).best;
fprintf('\nRECOMMENDED START\n');
fprintf('  [%.2f %.2f] in %s, clearance %.2f m\n', report.recommended, ...
    report.rooms(iLiving).name, report.rooms(iLiving).bestClearance);

items = hmmSceneManifest();
if ~isempty(items)
    fprintf('\n  distance from the recommendation to each prop that matters\n');
    for nm = ["Mug_Red", "Glass_Kitchen", "Plate"]
        idx = find(strcmp({items.name}, nm), 1);
        if ~isempty(idx)
            fprintf('    %-16s [%5.2f %5.2f]  %5.2f m away\n', items(idx).name, ...
                items(idx).position(1:2), norm(items(idx).position(1:2) - report.recommended));
        end
    end
end

%% Images, written with imwrite and no figure
pngNav = uint8(255 * report.navigable);
imwrite(pngNav, fullfile(outDir, 'floor_navigable.png'));

hm = report.heightMap;
hm(~isfinite(hm)) = 3.0;
imwrite(uint8(255 * min(max(hm, 0), 3) / 3), fullfile(outDir, 'floor_height.png'));

% A three-colour reading of the same map: free enough for the chassis, floor but
% too tight, and not floor at all - plus the poses marked on it.
rgb = zeros([size(free) 3], 'uint8');
rgb(:, :, 2) = uint8(220 * free);
tight = report.navigable & ~free;
rgb(:, :, 1) = rgb(:, :, 1) + uint8(200 * tight);
rgb(:, :, 2) = rgb(:, :, 2) + uint8(160 * tight);
rgb = markAt(rgb, p0, [255 0 0], report);
rgb = markAt(rgb, report.recommended, [0 128 255], report);
for k = 1:numel(report.rooms)
    rgb = markAt(rgb, report.rooms(k).datum, [255 255 255], report);
end
imwrite(rgb, fullfile(outDir, 'floor_verdict.png'));

fprintf('\nimages written to %s\n', outDir);
fprintf('  floor_verdict.png  green free, amber floor-but-tight, black not floor;\n');
fprintf('  red = configured start, blue = recommendation, white = room datums.\n');
fprintf('  map spans x [%.2f %.2f] y [%.2f %.2f] at %d cells/m\n\n', ...
    report.world(1, :), report.world(2, :), res);

save(fullfile(outDir, 'floorPlan.mat'), 'report');

%% ------------------------------------------------------------------------
    function capture(w)
        %CAPTURE Read the ray tracer once, on the fifth frame.
        persistent nFrame
        if isempty(nFrame)
            nFrame = 0;
        end
        nFrame = nFrame + 1;
        if nFrame ~= 5
            return
        end
        [~, ~, hitLoc, ~, valid] = read(w.Actors.NadirScan);
        z = nan(size(hitLoc, 1), 1);
        z(logical(valid)) = hitLoc(logical(valid), 3);
        hitZ = z;
    end

end

%% ------------------------------------------------------------------------
function v = centres(gridOrigin, res, lo, hi)
%CENTRES Cell centres of a res-cells/m grid starting at gridOrigin, within [lo hi].
%   Cell k of such a grid spans gridOrigin + [k k+1]/res, so its centre is at
%   gridOrigin + (k + 0.5)/res. Both ends are taken inclusively of any centre that
%   falls inside the interval, so a caller asking for a margin gets at least that
%   margin rather than one cell less.
kLo = ceil((lo - gridOrigin) * res - 0.5);
kHi = floor((hi - gridOrigin) * res - 0.5);
v = gridOrigin + ((kLo:kHi) + 0.5) / res;
end

%% ------------------------------------------------------------------------
function [r, c] = cellOf(xy, report)
%CELLOF Map subscripts of a world point, in the map's own convention.
r = round((report.world(2, 2) - xy(2)) * report.resolution) + 1;
c = round((xy(1) - report.world(1, 1)) * report.resolution) + 1;
end

function xy = worldOf(r, c, report)
%WORLDOF The inverse of cellOf.
xy = [report.world(1, 1) + (c - 1) / report.resolution, ...
      report.world(2, 2) - (r - 1) / report.resolution];
end

function v = valueAt(mapMatrix, xy, report)
%VALUEAT A map value at a world point, NaN outside the map.
[r, c] = cellOf(xy, report);
if r < 1 || c < 1 || r > size(mapMatrix, 1) || c > size(mapMatrix, 2)
    v = NaN;
    return
end
v = mapMatrix(r, c);
end

function m = boxMask(sz, xy, halfSide, report)
%BOXMASK A square window round a world point.
[r, c] = cellOf(xy, report);
n = round(halfSide * report.resolution);
m = false(sz);
rows = max(1, r - n):min(sz(1), r + n);
cols = max(1, c - n):min(sz(2), c + n);
m(rows, cols) = true;
end

function l = labelNear(lbl, xy, report)
%LABELNEAR The connected-component label at a world point, 0 when unlabelled.
[r, c] = cellOf(xy, report);
if r < 1 || c < 1 || r > size(lbl, 1) || c > size(lbl, 2)
    l = 0;
    return
end
l = lbl(r, c);
end

function rgb = markAt(rgb, xy, colour, report)
%MARKAT Stamp a small cross on the verdict image at a world point.
[r, c] = cellOf(xy, report);
n = 3;
for dr = -n:n
    for dc = -n:n
        if abs(dr) ~= abs(dc) && dr ~= 0 && dc ~= 0
            continue
        end
        rr = r + dr;
        cc = c + dc;
        if rr < 1 || cc < 1 || rr > size(rgb, 1) || cc > size(rgb, 2)
            continue
        end
        rgb(rr, cc, :) = reshape(uint8(colour), 1, 1, 3);
    end
end
end

function out = ternary(cond, a, b)
%TERNARY Pick one of two values, so a printf line stays one line.
if cond
    out = a;
else
    out = b;
end
end
