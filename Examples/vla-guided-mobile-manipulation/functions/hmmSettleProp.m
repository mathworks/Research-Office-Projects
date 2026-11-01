function dz = hmmSettleProp(actorId, robotConfig, cfg)
%HMMSETTLEPROP Stand-in for gravity: rest a released prop on the surface beneath it.
%
%   dz = HMMSETTLEPROP(actorId, robotConfig, cfg) moves the Sim3D actor of manifest entry actorId
%   vertically so that its bottom rests on the highest shell surface under its footprint,
%   and returns the vertical correction applied, m (positive = lifted out of a surface,
%   negative = dropped onto one). Returns 0 and does nothing when there is no live world,
%   no such actor, or no surface measurement under it.
%
%   PLANT-SIDE, AND THAT IS WHY IT MAY READ TRUTH. Every placement in this example is
%   geometric: Sim3D's own detach asks for Physics and Gravity on the released prop, but the
%   props are multi-part meshes whose child actors are Static, so the engine never lets them
%   fall (see hmmSpawnProps). Without this the prop freezes wherever the attach left it - in
%   mid-air above a high release, or inside the counter below a low one. This function is the
%   world doing what the world would do, so it uses the world's own geometry and the robot's
%   configuration and the attach offset it froze. Nothing it computes reaches perception, planning
%   or control.
%
%   IT DOES NOT HIDE A BAD RELEASE. The correction is printed on every release and kept in
%   appdata 'hmmSettleTap', so a release that put the object 53 mm into a counter still says
%   so - it just no longer draws it that way.
%
%   THE SURFACE IS THE ENGINE'S OWN NADIR SWEEP (data/shellProbe/floorPlan.mat, heightMap),
%   the same measurement the navigation map is carved from: the first surface a vertical ray
%   meets in each 50 mm cell. It reproduces every known support exactly - side table 0.550,
%   coffee table 0.400, kitchen counter 0.900.
%
%   OTHER PROPS ARE SUPPORTS TOO, because props are not in the shell. Until 2026-10-16 the
%   sweep was the only surface, and the stool place of that day released the soap dispenser
%   on the stool top at 0.73 m and then "settled" it 663.5 mm down to the floor beneath -
%   through the stool - while the run reported DONE. So every other prop whose box lies under
%   the footprint is a candidate too, its top taken from where the prop IS (its actor's
%   Translation: it may have been carried earlier in the run) and its manifest extent, under
%   the same overhead rule as the sweep.
%
%   THE PIVOT-TO-BOTTOM DISTANCE IS MEASURED, NOT ASSUMED. A mesh's origin can be its bbox
%   centre, its base or anywhere else, so it is taken as the spawn height above the surface
%   the prop was spawned on, from the same height map - one rule for every prop. The prop is
%   carried upright (eeTargetRot stays [0 0 0]), so that distance is unchanged at release.
%
%   See also hmmSpawnProps, hmmAttachOffset, hmmSceneManifest, probeFloorPlan.

%   Copyright 2026 The MathWorks, Inc.

arguments
    actorId (1, 1) double
    robotConfig (1, :) double
    cfg struct = hmmRuntimeConfig()
end

dz = 0;
try
    items = hmmSceneManifest();
    actorId = double(actorId);
    if actorId < 1 || actorId > numel(items) || actorId ~= round(actorId)
        return
    end
    names = hmmActorNames(cfg);
    key   = char(names(actorId));
    label = regexprep(key, '_+$', '');

    w = liveWorld();
    if isempty(w) || ~isfield(w.Actors, key)
        return
    end
    actor = w.Actors.(key);

    hm = heightMap();
    if isempty(hm)
        return
    end

    ext    = double(items(actorId).extent(:))';
    % WHERE THE PROP IS, FROM THE HAND AND NOT FROM THE ACTOR. On the detach step the actor's
    % Translation still holds its END-EFFECTOR-RELATIVE value - the toolbox's detach is only
    % attachTo([]), and the engine reports the world pose back a couple of steps later
    % (measured standalone: relative for 2 world steps, then world; WorldTranslation never
    % updates at all). Reading it here put the Mug at [0.013 -0.009] - its attach offset - and
    % the settle then wrote it onto the floor at the world origin, where the user saw it vanish.
    % So the position is COMPUTED: the tool pose by FK on this step's configuration, times the
    % relative pose hmmAttachOffset froze at jaw closure - exactly the transform the attach has
    % been drawing the prop with. A write right after the detach is applied in world frame.
    atap = getappdata(0, 'hmmAttachOffsetTap');
    if isempty(atap) || ~isfield(atap, 'Offset') || ...
            ~strcmp(char(string(atap.ActorName)), label)
        fprintf('  settle: %s - no attach offset recorded for it, left where it is\n', label);
        return
    end
    model = hmmArmCollisionModel();
    T = getTransform(model.fullTree, robotConfig, 'tcp_frame');
    p = (T(1:3, 4) + T(1:3, 1:3) * double(atap.Offset(:)))';

    % OVERHEAD CELLS ARE NOT SUPPORT. The sweep records the FIRST surface from above, so under a
    % wall cabinet it reads the cabinet (2.70 m over the kitchen counter) and the counter is not in
    % the map at all. A cell higher than the prop's own top is therefore discarded; if every cell
    % is, there is no measurement and the prop is left where it is.
    zCeil = p(3) + 0.5 * ext(3);
    zHere = surfaceUnder(hm, p(1:2), ext, zCeil);
    on    = "the swept surface";
    [zProp, propName] = propUnder(w, items, names, actorId, hm, p(1:2), ext, zCeil);
    if isfinite(zProp) && ~(zProp <= zHere)          % also taken when the sweep has nothing
        zHere = zProp;
        on    = propName;
    end
    if ~isfinite(zHere)
        fprintf('  settle: %s - no swept surface under the release, left where it is\n', label);
        return
    end
    pivotAboveBottom = pivotAbove(hm, items(actorId));

    zNew = zHere + pivotAboveBottom;
    dz   = zNew - p(3);
    if abs(dz) < cfg.actor.settleTol
        dz = 0;
    else
        % THE ROTATION GOES WITH THE POSITION. A write on the detach step resends the whole pose
        % the actor still caches, and its cached Rotation is the GRIPPER-RELATIVE one the toolbox
        % set at attach (TargetGraspUtils/attachTargetActor's "self-rotation compensation": the
        % tool's own rotation at closure). Resent as a world rotation, it turned a top-down grasp
        % upside down: the Bottle of 2026-10-19 lay at [180 0 32] deg, the tool's pose at closure,
        % after a 5.7 mm lift. A release inside settleTol is never written, which is why the mugs
        % came out upright. The rotation the engine was drawing it with is that relative rotation
        % under the tool's pose now - for a top-down place, upright, turned as the hand turned.
        Rw = T(1:3, 1:3) * double(atap.ToolInWorld(1:3, 1:3));
        e  = rotm2eul(Rw, 'ZYX');                          % [yaw pitch roll]
        actor.Rotation    = [-e(3), -e(2), e(1)];          % actor convention: eul2rotm([r3 -r2 -r1], 'ZYX')
        actor.Translation = [p(1) p(2) zNew];
    end

    if dz > 0
        how = 'released INSIDE the surface, lifted onto it';
    elseif dz < 0
        how = 'released above the surface, dropped onto it';
    else
        how = 'released on the surface';
    end
    fprintf('  settle: %s %s - bottom was %+.1f mm from %s at z %.3f m\n', ...
        label, how, -1000 * dz, on, zHere);
    setappdata(0, 'hmmSettleTap', struct('ActorName', string(label), ...
        'Released', p, 'Settled', [p(1) p(2) p(3) + dz], 'Dz', dz, 'Surface', zHere, ...
        'On', on));
catch err
    % A stand-in for gravity must never stop a twenty-minute simulation.
    warning('hmm:settleProp:failed', 'hmmSettleProp left the prop where it was: %s', ...
        err.message);
    dz = 0;
end
end

%% ------------------------------------------------------------------------
function w = liveWorld()
%LIVEWORLD The running world, found by the name hmmSpawnProps retained - see hmmSpawnProps
%   for why the name, not the handle, survives the compile phases.
w = [];
nm = getappdata(0, 'hmmPropWorldName');
if isempty(nm)
    return
end
try
    cw = sim3d.World.getWorld(char(nm));
    if ~isempty(cw) && isvalid(cw)
        w = cw;
    end
catch
end
end

function hm = heightMap()
%HEIGHTMAP The nadir-sweep surface heights, loaded once per session.
persistent cache
if isempty(cache)
    f = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'shellProbe', ...
        'floorPlan.mat');
    if ~isfile(f)
        hm = [];
        return
    end
    S = load(f, 'report');
    cache = S.report;
end
hm = cache;
end

function h = pivotAbove(hm, item)
%PIVOTABOVE A prop's pivot height above its own bottom. Measured where possible: the spawn
%   height above the swept surface it was spawned on. Otherwise the bbox-centre convention
%   every measurable prop follows (17 of 20 manifest props reproduce ext(3)/2 to 1 mm; the
%   other three spawn under cabinets).
ext = double(item.extent(:))';
pos = double(item.position(:))';
zSpawn = surfaceUnder(hm, pos(1:2), ext, pos(3) + 0.5 * ext(3));
if isfinite(zSpawn)
    h = pos(3) - zSpawn;
else
    h = 0.5 * ext(3);
end
end

function [z, name] = propUnder(w, items, names, self, hm, xy, ext, zCeil)
%PROPUNDER Highest top of ANOTHER prop under the released prop's footprint and below zCeil.
%   The footprint is the same yaw-free disc surfaceUnder uses, tested against each prop's
%   axis-aligned box. The box sits where the actor is now, so a prop carried earlier in the
%   run is found where it was put rather than where it was spawned.
z = NaN;
name = "";
r = 0.5 * max(ext(1:2));
for j = 1:numel(items)
    if j == self
        continue
    end
    key = char(names(j));
    pos = double(items(j).position(:))';
    if isfield(w.Actors, key)
        pos = double(w.Actors.(key).Translation(:))';
    end
    e  = double(items(j).extent(:))';
    dx = max(abs(xy(1) - pos(1)) - 0.5 * e(1), 0);
    dy = max(abs(xy(2) - pos(2)) - 0.5 * e(2), 0);
    if hypot(dx, dy) > r
        continue
    end
    top = pos(3) - pivotAbove(hm, items(j)) + e(3);
    if top < zCeil && ~(top <= z)
        z = top;
        name = string(regexprep(key, '_+$', ''));
    end
end
end

function z = surfaceUnder(hm, xy, ext, zCeil)
%SURFACEUNDER Highest swept surface under the prop's footprint and below zCeil - an object rests
%   on the highest thing beneath it. The footprint is the DISC of the extent's larger half-span,
%   which needs no yaw: the carried prop's yaw is not tracked here, and the disc covers it at any.
step = 0.5 / hm.resolution;
r = 0.5 * max(ext(1:2));
[u, v] = ndgrid(-r:step:r, -r:step:r);
in = hypot(u(:), v(:)) <= r + 1e-9;
x = xy(1) + u(in);
y = xy(2) + v(in);
row = round((hm.world(2, 2) - y) * hm.resolution) + 1;
col = round((x - hm.world(1, 1)) * hm.resolution) + 1;
ok  = row >= 1 & row <= size(hm.heightMap, 1) & col >= 1 & col <= size(hm.heightMap, 2);
if ~any(ok)
    z = NaN;
    return
end
h = hm.heightMap(sub2ind(size(hm.heightMap), row(ok), col(ok)));
z = max(h(isfinite(h) & h < zCeil));
if isempty(z)
    z = NaN;
end
end
