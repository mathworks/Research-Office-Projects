function [boxes, names, tops] = hmmFloorObstacles(cfg)
%HMMFLOOROBSTACLES The manifest props that stand on the floor, as footprints.
%
%   boxes = HMMFLOOROBSTACLES(cfg) returns an Nx4 matrix of
%   [centreX centreY sizeX sizeY] in WORLD metres, one row per prop in
%   hmmSceneManifest whose bounding box rests on the floor and rises above it.
%   [boxes, names] = HMMFLOOROBSTACLES(cfg) also returns the manifest keys, in
%   the same order, for reporting. N is 0 when no manifest was found.
%   [boxes, names, tops] = HMMFLOOROBSTACLES(cfg) also returns each prop's box
%   TOP in world metres.
%
%   THE TOPS ARE WHY THIS FUNCTION HAS A SECOND CALLER. The navigation grid wants
%   footprints and discards the height, which is all a base planner can use. The
%   ARM planner cannot: hmmSurfaceObstacles fits boxes to that same grid and
%   hmmScreenArmPlan hangs each one below the fitted support surface, so a 0.66 m
%   stool read out of the grid near a 0.91 m worktop became a slab of air starting
%   at the seat - 0.250 m of obstacle where there is nothing, which is the
%   direction that turns an avoided collision into an unplannable pose. The height
%   was measured here all along to classify the prop; returning it costs nothing
%   and is the only place in the project that knows it.
%
%   WHY THIS EXISTS. makeNavMap builds the navigation grid by casting nadir rays
%   at cfg.scene.staticMesh and carving free space where a ray reached bare
%   floor. The props are NOT in that mesh - they are separate actors placed at
%   spawn - so no prop has ever appeared in the map, however large it is and
%   however firmly it stands on the floor. cfg.map.mode's own comment claims
%   "the shell is Static and the props are placed once at spawn, so a map
%   captured against that geometry stays true for the run"; that was true of the
%   shell and false of the props, and this function is what makes it true of
%   both.
%
%   THE TEST IS GEOMETRIC AND NAMES NOTHING. A prop is a floor obstacle when its
%   box bottom is within cfg.map.floorPropBand of z = 0 - nothing in the scene
%   supports it, so it stands on the floor - and its top is ABOVE that band, so
%   it is something to drive around rather than a rug or a doormat to drive over.
%   Every other prop in the manifest sits on furniture whose cells the nadir
%   sweep already marked occupied, so adding it would mark nothing new.
%
%   WHY BOTH HALVES OF THE TEST ARE NEEDED, with the census that sized the band.
%   Of the twenty props, bottoms measure -0.0008 m (both stools), 0.0000 m
%   (SoccerBall) and then nothing until 0.400 m (Magazine, Mug, Remote). So the
%   0.06 m band has 0.34 m of clear air on either side and is in no danger of
%   catching a prop that is really on a table. The upper test has no prop to
%   exclude today and is kept because a rug or a floor mat is exactly the asset
%   this would otherwise block the apartment with, and it costs one comparison.
%
%   AXIS-ALIGNED, WHICH IS THE MANIFEST'S OWN CONVENTION. A prop's placement
%   metadata carries a position and a size and no orientation, so there is no
%   yaw here to honour. For a 0.3 m square stool that is exact; for anything long
%   and rotated it would be conservative in the occupied direction, which is the
%   safe side for a base planner.
%
%   WHAT THIS CANNOT KNOW. The grid is static, so a prop the robot picks up and
%   carries away leaves its cells blocked for the rest of the run. That is the
%   same assumption cfg.map.mode already documents and it costs nothing for the
%   stools, which are never picked; for a floor-standing prop that IS the task
%   object the stand-off ring has to stand off the prop's own footprint, which is
%   what it would have to do physically anyway.
%
%   See also makeNavMap, hmmSceneManifest, hmmLoadNavMap, hmmSurfaceObstacles.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg struct = hmmConfig()
end

boxes = zeros(0, 4);
names = strings(0, 1);
tops  = zeros(0, 1);

items = hmmSceneManifest();
if isempty(items)
    return      % no assets on this machine; the caller degrades to the shell alone
end

band = cfg.map.floorPropBand;

for k = 1:numel(items)
    pos = items(k).position;
    ext = items(k).extent;
    if numel(pos) ~= 3 || numel(ext) ~= 3 || ~all(ext > 0)
        continue        % a slot with no extent is not an obstacle
    end
    bottom = pos(3) - ext(3) / 2;
    top    = pos(3) + ext(3) / 2;
    if bottom > band || top <= band
        continue
    end
    boxes(end+1, :) = [pos(1) pos(2) ext(1) ext(2)];        %#ok<AGROW>
    names(end+1, 1) = string(items(k).name);                %#ok<AGROW>
    tops(end+1, 1)  = top;                                  %#ok<AGROW>
end

end
