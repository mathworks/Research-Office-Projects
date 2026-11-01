function [pts, names, info] = hmmPropFootprints(cfg)
%HMMPROPFOOTPRINTS Each floor-standing prop's REAL footprint, ray-traced from above.
%
%   [pts, names] = HMMPROPFOOTPRINTS(cfg) spawns every floor-standing manifest prop
%   in a standalone Sim3D world, casts one nadir ray at the centre of every map cell
%   its bounding box reaches, and returns pts as a cell array of Kx2 world XY points
%   - the cell centres a ray struck the prop at, plus any cell those enclose. names
%   holds the manifest keys in the same order.
%
%   [pts, names, info] = HMMPROPFOOTPRINTS(...) also reports, per prop, the area its
%   bounding rectangle covers, the area the struck cells imply, their ratio - which
%   is the number that says whether a prop is a box or not - the ray count, and how
%   many cells were sealed because the mesh had an opening over them.
%
%   ONE RAY PER CELL CENTRE, WHICH IS A RULE AND NOT A SAMPLE DENSITY. The shipped
%   map's shell was swept exactly this way - probeFloorPlan casts one nadir ray at
%   each cell centre and the cell is occupied if that ray hit something above the
%   floor - so this is the convention the rest of the grid already follows, and the
%   props were the half that did not. It also happens to be the only rule that can
%   express a circle at this resolution, which is how the inconsistency was found:
%   occupyFloorProps marked a cell if ANY point of a footprint fell inside it, and
%   the nearest point of a 0.30 m square's corner cell is 0.1414 m from its centre
%   against a 0.15 m radius - inside. So under the touch rule a round stool marks
%   every cell of its bounding square, and no sampling density whatsoever would have
%   changed that. MEASURED, before and after: the three floor props stamped 96 cells
%   under the touch rule and stamp 80 now, and each stool went from a solid block
%   filling its bounding square to 32 of those 36 cells with all four corners cut.
%   Read those counts back with checkOccupancy on the loaded map and not by indexing
%   the saved matrix directly - a hand-rolled world-to-row formula got the window
%   wrong here once and reported a correct map as empty.
%
%   AND ONE RAY PER CELL HAS A PRICE, WHICH IS WHY THE MASK IS HOLE-FILLED. A single ray
%   believes whatever the mesh tells it, and one of these meshes lies: the SoccerBall
%   has a circular opening about 50 mm across at its apex, so the sweep first
%   returned it as a ring with two free cells in the middle - which the map then
%   shipped, at 78 prop cells against the 80 it holds now. A hollow
%   obstacle in a navigation map is a worse defect than a square stool. The fix is
%   not more rays - the opening is a whole cell wide - but sealing free cells that no
%   outside cell can reach, which is a statement about where a robot can drive and
%   not about this ball: imfill(mask, 4, 'holes'), four-connected so that a gap a robot
%   could only cross diagonally stays sealed, the conservative reading.
%
%   THE COST IS THE FOUR CORNER CELLS OF EACH STOOL AND IT IS NOT A SAFETY CHANGE.
%   The corners are bare floor, which is why the user reported them. They were also
%   never load-bearing: the base planner inflates every obstacle by the chassis
%   footprint before it searches, so a cell at the rim of a 0.30 m prop is swallowed
%   by a 0.53 by 0.35 m inflation many times over. What the corners did do was block
%   arrival poses next to the kitchen counter, which is the one surface in the
%   apartment where the place leg has the fewest bearings it can use.
%
%   WHY THIS EXISTS: THE MANIFEST HAS NO SHAPE, ONLY A BOX. hmmSceneManifest
%   publishes a position and an extent per prop and nothing else, so makeNavMap had
%   no choice but to stamp the bounding RECTANGLE, and the two kitchen stools - which
%   are round - came out square on the occupancy map with their four corners blocked.
%   The corners are real floor. A base planner refusing them is refusing space it
%   could drive through, next to the one surface in the apartment where the place leg
%   already has the fewest arrival poses it can use.
%
%   AND WHY IT IS MEASURED RATHER THAN DECLARED. The obvious fix is a shape field -
%   "Stool is a cylinder, radius 0.15" - and it was not taken, for two reasons. The
%   manifest ships with the reused apartment assets and is not this project's file to
%   extend; and a table of per-prop shapes is exactly the kind of value keyed to a
%   prop name that this project treats as a defect, because the next asset is not in
%   it. A nadir ray asks the mesh. A round prop comes out round, a square one square,
%   an L-shaped one L-shaped, and nothing here knows or needs the name of any of
%   them. It is the same instrument probeFloorPlan already uses on the shell, pointed
%   at the props the shell mesh does not contain.
%
%   THE SWEEP IS PER PROP AND TINY, which is why it can sit inside a capture script.
%   The shell survey casts 89 600 rays over the whole apartment; this casts one grid
%   per prop over that prop's own bounding box, so the three floor props come to 164
%   rays in one world and one run.
%
%   THE WORLD HOLDS THE PROPS AND NOTHING ELSE, which is what makes reading the
%   result trivial: there is no floor and no shell, so a ray that hits anything hit a
%   prop, and a ray that hits nothing passed through bare air. No height test, no
%   threshold, nothing to tune. Props whose boxes overlap in plan may have a ray cast
%   for one strike the other; that marks a cell occupied either way, which is correct.
%
%   AN EMPTY RETURN IS A VALID ANSWER AND THE CALLER MUST HANDLE IT. No assets on
%   this machine, no Sim3D, a mesh missing for one prop, or a world that runs and
%   reports nothing - all of them come back as empty points for the props concerned,
%   and makeNavMap falls back to the bounding rectangle for those. That fallback is
%   the conservative direction: too much occupied, never too little.
%
%   See also makeNavMap, hmmFloorObstacles, probeFloorPlan, hmmSpawnProps.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg struct = hmmConfig()
end

% NO SAMPLE-SPACING ARGUMENT, DELIBERATELY. The spacing is the map's cell pitch
% because the rule is "is this CELL's centre under the prop", and a caller free to
% pass something else could only make the answer disagree with the grid it is for.
step = 1 / cfg.map.resolution;

[boxes, names] = hmmFloorObstacles(cfg);
n = size(boxes, 1);
pts  = repmat({zeros(0, 2)}, n, 1);
info = struct('name', {}, 'boxArea', {}, 'measuredArea', {}, 'ratio', {}, ...
    'rays', {}, 'sealed', {});
if n == 0
    return
end

[items, propsDir] = hmmSceneManifest();
if isempty(propsDir)
    return
end

% THE RAY GRID, ONE BLOCK PER PROP, CONCATENATED INTO ONE SENSOR. A separate world
% per prop would pay the engine's start-up cost N times for a sweep whose rays are
% free by comparison. The blocks are kept as index ranges so each prop's hits can be
% told apart afterwards.
origins = zeros(0, 3);
span    = zeros(n, 2);          % first and last ray index of each prop's block
dims    = zeros(n, 2);          % rows and columns that block folds back into
spawned = false(n, 1);
zStart  = 4.0;
nFrame  = 0;                    % see capture: a LOCAL counter, not a persistent one

% A WORLD NAME CANNOT BE REUSED IN A SESSION, including by a call that failed before
% it got as far as deleting its world, so the name carries the clock. Measured the
% hard way: "World with name '...' already exists. Use a different name."
worldName = sprintf('hmmPropFootprints_%s', ...
    char(datetime('now', 'Format', 'HHmmssSSS')));
world = sim3d.World('Name', worldName, 'StopTime', 0.2, ...
    'ExecCmds', 'r.Fog 0', 'Update', @capture);
cleanupWorld = onCleanup(@() delete(world));

for k = 1:n
    idx = find(strcmp({items.name}, names(k)), 1);
    if isempty(idx)
        continue
    end
    mesh = fullfile(propsDir, [items(idx).name '.fbx']);
    if ~isfile(mesh)
        continue
    end

    % STATIC IS CORRECT HERE AND IS NOT THE MOBILITY WARNING'S PROBLEM. Nothing in
    % this world moves, has physics or is grasped; the prop is geometry to be
    % measured. See hmmSpawnProps for why a loaded mesh's children are Static
    % whatever the parent asked for.
    actor = sim3d.Actor('ActorName', sprintf('prop_%d', k), ...
        'Mobility', sim3d.utils.MobilityTypes.Static, ...
        'Translation', items(idx).position(:)');
    add(world, actor);
    load(actor, mesh, cfg.scene.meshScale);
    propagate(actor, 'Collisions', true);
    propagate(actor, 'PreciseContacts', false);

    % THE CELL CENTRES THE BOX REACHES, PLUS ONE CELL OF MARGIN ALL ROUND. The
    % margin is not for the measurement's sake - a prop cannot be wider than its own
    % bounding box - but so that a mesh sitting slightly off its manifest position,
    % or an extent that is a shade tight, shows up as a hit in the margin ring
    % instead of being silently clipped at the edge of the window being searched.
    b  = boxes(k, :);
    xs = cellCentres(cfg.map.origin(1), step, b(1) - b(3) / 2 - step, b(1) + b(3) / 2 + step);
    ys = cellCentres(cfg.map.origin(2), step, b(2) - b(4) / 2 - step, b(2) + b(4) / 2 + step);
    if isempty(xs) || isempty(ys)
        continue
    end
    [XX, YY] = meshgrid(xs, ys);
    blk = [XX(:), YY(:), zStart * ones(numel(XX), 1)];

    % THE BLOCK'S SHAPE IS KEPT, NOT JUST ITS LENGTH, because the hits have to be
    % folded back into a 2-D mask to seal openings in the mesh with imfill.
    dims(k, :) = [numel(ys), numel(xs)];
    span(k, :) = [size(origins, 1) + 1, size(origins, 1) + size(blk, 1)];
    origins = [origins; blk];                                       %#ok<AGROW>
    spawned(k) = true;
end

if ~any(spawned)
    return
end

nRay       = size(origins, 1);
directions = repmat([0 0 -1], nRay, 1);
lengths    = (zStart + 1.0) * ones(nRay, 1);

sensor = sim3d.sensors.RaytraceSensor('ActorName', 'PropNadirScan', ...
    'Translation', [0 0 0], 'Rotation', [0 0 0], ...
    'RayOrigins', origins, 'RayDirections', directions, ...
    'RayLengths', lengths, 'NumberOfBounces', 0);
add(world, sensor);

hit = [];
run(world);
clear cleanupWorld                  % delete the world before anything else can throw

if isempty(hit) || numel(hit) ~= nRay
    return
end

for k = 1:n
    if ~spawned(k)
        continue
    end
    blk = origins(span(k, 1):span(k, 2), 1:2);
    h   = hit(span(k, 1):span(k, 2));

    % A HOLE IN A MESH MUST NOT BECOME A HOLE IN THE MAP, and one of these props has
    % one. MEASURED: the SoccerBall carries a circular opening about 50 mm across at
    % its apex - a full map cell - and a nadir ray that enters it registers nothing
    % at all, because the inside of the far shell faces away from the ray. The ball
    % therefore came out of the sweep as a 14-cell RING with two free cells at its
    % centre. Thin radial misses along the panel seams showed up too, further out.
    %
    % No denser sampling inside a cell can fix that: the opening is as wide as the
    % cell, so every ray in the cell goes through it. What fixes it is the question
    % the map is actually asking. A free cell that is wholly enclosed by occupied
    % cells cannot be entered by a ground robot without crossing an obstacle, so it
    % is not free, whatever the mesh does. Sealing those cells is a statement about
    % reachability, not about this ball - it names no prop, measures no shape, and
    % does nothing at all to a footprint that has no enclosed pocket.
    %
    % It does not round the prop back off, either: the stools' cut corners are on
    % the outside of the footprint and stay connected to the margin ring, so they
    % stay free. Only genuinely interior cells change.
    nBefore = nnz(h);
    % Every block carries a one-cell margin of bare floor, so the border is outside.
    h = imfill(reshape(h, dims(k, 1), dims(k, 2)), 4, 'holes');
    h = h(:);
    nSealed = nnz(h) - nBefore;

    pts{k} = blk(h, :);

    boxArea = boxes(k, 3) * boxes(k, 4);
    % ONE CELL OF AREA PER HIT, WHICH IS WHAT THE GRID WILL ACTUALLY HOLD. The ratio
    % against the bounding box is the whole answer to "is this a box?": a square prop
    % reads near 1.00, a cylinder near pi/4 = 0.785, a ball the same. At one ray per
    % cell a 0.30 m prop is only six cells across, so expect the quantisation to show
    % - this is the map's own fidelity, not an estimate of the mesh.
    measured = nnz(h) * step^2;
    info(end + 1, 1) = struct('name', names(k), ...
        'boxArea', boxArea, 'measuredArea', measured, ...
        'ratio', measured / max(boxArea, eps), ...
        'rays', span(k, 2) - span(k, 1) + 1, ...
        'sealed', nSealed);                                         %#ok<AGROW>
end

    function capture(w)
        %CAPTURE Read the ray tracer once, on the fifth frame.
        %   The frame number is probeFloorPlan's, and for its reason: the first frames
        %   of a Sim3D world are rendered before the loaded meshes have settled into
        %   the scene, and a sweep taken on frame 1 reports empty.
        %
        %   A LOCAL COUNTER AND NOT A persistent ONE, which is a real difference and
        %   not style. probeFloorPlan's capture declares nFrame persistent, so a
        %   SECOND call to it in the same MATLAB session starts counting at 6, never
        %   reaches 5, and returns nothing - reported as "the world ran but the ray
        %   tracer returned nothing", which reads like an engine fault. Sharing the
        %   parent's workspace gives one counter per call, which is what was meant.
        nFrame = nFrame + 1;
        if nFrame ~= 5
            return
        end
        [~, ~, ~, ~, valid] = read(w.Actors.PropNadirScan);
        hit = logical(valid(:));
    end

end

%% ------------------------------------------------------------------------
function v = cellCentres(gridOrigin, step, lo, hi)
%CELLCENTRES Cell centres of the map's own grid that fall within [lo hi].
%   Cell k of a grid starting at gridOrigin spans gridOrigin + [k k+1]*step, so its
%   centre is at gridOrigin + (k + 0.5)*step. This is probeFloorPlan's centres(),
%   kept as a copy rather than shared because that one is a local function of a
%   probe script and taking a dependency on a script's internals to build the
%   shipped map is the wrong direction for the dependency to run.
kLo = ceil((lo - gridOrigin) / step - 0.5);
kHi = floor((hi - gridOrigin) / step - 0.5);
v = gridOrigin + ((kLo:kHi) + 0.5) * step;
end
