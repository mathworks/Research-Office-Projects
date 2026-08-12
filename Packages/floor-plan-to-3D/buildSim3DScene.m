function world = buildSim3DScene(geometry, options)
%buildSim3DScene Create a Sim3D world from extracted floor plan geometry.
%
%   world = buildSim3DScene(geometry) takes the output of
%   extractFloorPlanGeometry and builds the scene in Sim3D using box
%   primitives for wall panels, lintels, sills, and a floor.
%
%   The world is returned viewing. Call delete(world) to clean up.
%
%   See also extractFloorPlanGeometry, sim3d.World, sim3d.Actor.

arguments
    geometry (1,1) struct
    options.WallColor (1,3) double = [0.85 0.85 0.82]
    options.LintelColor (1,3) double = [0.78 0.78 0.75]
    options.FloorColor (1,3) double = [0.92 0.90 0.85]
    options.Scene (1,1) string = "Empty scene"
end

regions = geometry.Regions;
openings = geometry.Openings;
scale = geometry.Scale;
imgH = geometry.ImageSize(1);
wallHeight = geometry.WallHeight;
doorHeight = geometry.DoorHeight;
windowBottom = geometry.WindowBottom;
windowTop = geometry.WindowTop;
floorDims = geometry.FloorDimensions;

world = sim3d.World(Scene=options.Scene);

%% Floor
floor = sim3d.Actor(ActorName='Floor');
floor.createShape('box');
floor.Translation = [floorDims(1)/2, floorDims(2)/2, -0.005/2];
floor.Scale = [floorDims(1), floorDims(2), 0.005];
floor.Color = options.FloorColor;
world.add(floor);

%% Wall panels (one thick box per wall-face edge)
wallIdx = 0;
for r = 1:length(regions)
    verts = regions{r};
    if isempty(verts), continue; end

    xm = verts(:,1) * scale;
    ym = (imgH - verts(:,2)) * scale;
    n = size(verts, 1);

    % Determine inward normal direction from polygon winding (signed area)
    polyArea = 0;
    for i = 1:n
        j = mod(i, n) + 1;
        polyArea = polyArea + (xm(i)*ym(j) - xm(j)*ym(i));
    end

    for i = 1:n
        j = mod(i, n) + 1;
        p1 = [xm(i), ym(i)];
        p2 = [xm(j), ym(j)];
        edgeVec = p2 - p1;
        edgeLen = norm(edgeVec);
        if edgeLen < 0.01, continue; end

        % Inward-facing normal (right normal for CW, left for CCW)
        if polyArea < 0
            inNorm = [edgeVec(2), -edgeVec(1)] / edgeLen;
        else
            inNorm = [-edgeVec(2), edgeVec(1)] / edgeLen;
        end

        mid = (p1 + p2) / 2;

        % Ray-cast inward to find wall thickness at this edge
        thickness = inf;
        for ii = 1:n
            jj = mod(ii, n) + 1;
            if ii == i || jj == i || ii == j || jj == j, continue; end
            q1 = [xm(ii), ym(ii)];
            q2 = [xm(jj), ym(jj)];
            d = q2 - q1;
            denom = inNorm(1)*d(2) - inNorm(2)*d(1);
            if abs(denom) < 1e-10, continue; end
            t = ((q1(1)-mid(1))*d(2) - (q1(2)-mid(2))*d(1)) / denom;
            u = ((q1(1)-mid(1))*inNorm(2) - (q1(2)-mid(2))*inNorm(1)) / denom;
            if t > 0.001 && u >= 0 && u <= 1 && t < thickness
                thickness = t;
            end
        end

        % Skip connector/end-cap edges (length <= thickness)
        if isinf(thickness) || edgeLen <= thickness
            continue;
        end

        % Center box on wall midline (offset inward by half thickness)
        center = mid + inNorm * (thickness / 2);
        angle = atan2(edgeVec(2), edgeVec(1));

        wallIdx = wallIdx + 1;
        actor = sim3d.Actor(ActorName=sprintf('Wall_%d', wallIdx));
        actor.createShape('box');
        actor.Translation = [center(1), center(2), wallHeight/2];
        actor.Scale = [edgeLen, thickness, wallHeight];
        actor.Rotation = [0, 0, angle];
        actor.Color = options.WallColor;
        world.add(actor);
    end
end

%% Lintels and sills for openings
for k = 1:length(openings)
    op = openings(k);
    c = op.corners;
    if all(c(:) == 0), continue; end

    cx = c(:,1) * scale;
    cy = (imgH - c(:,2)) * scale;

    % Quad vertices: c1, c2, c4, c3 (same order as visualizeFloorPlan3D)
    qx = [cx(1); cx(2); cx(4); cx(3)];
    qy = [cy(1); cy(2); cy(4); cy(3)];

    % Compute box parameters from quad
    edge1 = [qx(2)-qx(1), qy(2)-qy(1)];
    edge2 = [qx(4)-qx(1), qy(4)-qy(1)];
    len1 = norm(edge1);
    len2 = norm(edge2);

    % The longer dimension is the opening width, shorter is wall thickness
    if len1 >= len2
        boxLen = len1;
        boxDepth = len2;
        angle = atan2(edge1(2), edge1(1));
    else
        boxLen = len2;
        boxDepth = len1;
        angle = atan2(edge2(2), edge2(1));
    end

    center = [mean(qx), mean(qy)];

    if op.type == "door"
        lintelZbot = doorHeight;
        lintelZtop = wallHeight;
        hasSill = false;
    elseif op.type == "window"
        lintelZbot = windowTop;
        lintelZtop = wallHeight;
        sillZbot = 0;
        sillZtop = windowBottom;
        hasSill = true;
    else
        lintelZbot = doorHeight;
        lintelZtop = wallHeight;
        hasSill = false;
    end

    % Lintel box
    lintelH = lintelZtop - lintelZbot;
    if lintelH > 0.01
        actor = sim3d.Actor(ActorName=sprintf('Lintel_%d', k));
        actor.createShape('box');
        actor.Translation = [center(1), center(2), lintelZbot + lintelH/2];
        actor.Scale = [boxLen, boxDepth, lintelH];
        actor.Rotation = [0, 0, angle];
        actor.Color = options.LintelColor;
        world.add(actor);
    end

    % Sill box (windows only)
    if hasSill
        sillH = sillZtop - sillZbot;
        if sillH > 0.01
            actor = sim3d.Actor(ActorName=sprintf('Sill_%d', k));
            actor.createShape('box');
            actor.Translation = [center(1), center(2), sillZbot + sillH/2];
            actor.Scale = [boxLen, boxDepth, sillH];
            actor.Rotation = [0, 0, angle];
            actor.Color = options.LintelColor;
            world.add(actor);
        end
    end
end

%% View
view(world);

end
