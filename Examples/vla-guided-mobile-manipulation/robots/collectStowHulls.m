function hulls = collectStowHulls(robot)
%COLLECTSTOWHULLS Per-body convex hulls of the robot's visual meshes, in body frames.
%
%   hulls = COLLECTSTOWHULLS(robot) returns a struct array with one entry per body
%   that carries visual geometry:
%
%       .name    the body name, so a caller can ask getTransform for its pose
%       .points  P-by-3 vertices in the BODY frame, hull-reduced
%       .nSource how many vertices the hull was reduced from, for reporting
%
%   WHY THIS EXISTS. Every footprint and envelope number in this project used to be
%   measured from body frame ORIGINS, because rigidBodyTree reports a body's
%   Collisions as a description string rather than as geometry and the obvious
%   conclusion was that the meshes are unreachable. They are not: getVisual returns
%   the triangulation, and its Points are the internal Vertices exactly. An origin is
%   a point on a link's axis; a footprint is the outline of its MEAT. On this robot
%   the difference is 167.5 mm, which is what made cfg.plan.stowedArmHalfExtent read
%   0.1799 m on the y axis while the stowed upper arm actually reaches 0.3474 m.
%   Anything that optimises the origin measure will keep parking the arm outside the
%   chassis and scoring it as inside.
%
%   WHY HULLS AND NOT THE RAW MESHES. The full set is 238251 vertices over 27
%   visuals. The stow search evaluated its cost several thousand times per start, so
%   transforming all of them per evaluation is what made a footprint term look
%   unaffordable, which is why the search left that term "deliberately not
%   written yet" at the price of the aperture sweep. A convex hull gives the identical
%   answer for every question asked of it here: extents, support radius and the z
%   range are all determined by extreme points, and no extreme point of a set is
%   interior to its hull. The reduction is about two orders of magnitude, which is
%   what turns the honest measurement into the cheap one.
%
%   THE HULL IS TAKEN AFTER THE VISUAL TRANSFORM, NOT BEFORE. getVisual hands back
%   Tform separately from the points, and a body may carry several visuals. Hulling
%   each visual on its own and concatenating would keep interior points of the union,
%   so the transform is applied first and the body's whole visual set is hulled once.
%
%   SCALE IS NOT APPLIED, AND THAT IS NOT AN OVERSIGHT. Nine of this robot's visuals
%   report Scale = 0.001 - the Robotiq meshes, authored in millimetres - while the
%   points getVisual returns are already in metres. The property is a record of a
%   conversion that has happened, not a pending multiplier. Multiplying by it here
%   would shrink the gripper by a thousand and report a delightfully small footprint.
%   hmmMirrorTree's header records the same trap for the same reason.
%
%   CONVEX IS AN OVER-ESTIMATE AND THAT IS THE SAFE DIRECTION. A hull fills the
%   concavities of a C-shaped link, so a footprint built from hulls can only be too
%   large. Every consumer of these numbers - map inflation, doorway clearance, the
%   footprint validator - fails dangerously when the figure is too SMALL and merely
%   conservatively when it is too large, which is the argument measureStowedEnvelope's
%   header makes about its own allowance.
%
%   See also measureStowedEnvelope, hmmBaseFootprint.

%   Copyright 2026 The MathWorks, Inc.

arguments
    robot {mustBeA(robot, 'rigidBodyTree')}
end

hulls = struct('name', {}, 'points', {}, 'nSource', {});

for k = 1:numel(robot.Bodies)
    body = robot.Bodies{k};
    if isempty(body.Visuals)
        continue
    end

    gv = getVisual(body);
    P  = zeros(0, 3);
    for j = 1:numel(gv)
        V = double(gv(j).Triangulation.Points);
        T = gv(j).Tform;
        P = [P; (T(1:3, 1:3) * V' + T(1:3, 4))'];      %#ok<AGROW>
    end

    hulls(end + 1) = struct('name', body.Name, ...
        'points', reducePoints(P), 'nSource', size(P, 1));  %#ok<AGROW>
end

end

% ---------------------------------------------------------------------------------
function P = reducePoints(P)
%REDUCEPOINTS The hull vertices of P, or P itself when it has no hull.
%   convhull requires a full-dimensional point set and errors on a degenerate one -
%   a flat plate, a line of points, fewer than four vertices. Those cases are rare
%   here but they are not impossible (a box visual scaled flat in one axis is a
%   legitimate thing for an assembly to build), and the fallback is simply to keep
%   every point: correct, merely slower, and slower on a set that is small by
%   definition since that is what made it degenerate.
if size(P, 1) < 4
    return
end
try
    P = P(unique(convhull(P(:, 1), P(:, 2), P(:, 3))), :);
catch
    % Degenerate set. Left as-is on purpose; see above.
end
end
