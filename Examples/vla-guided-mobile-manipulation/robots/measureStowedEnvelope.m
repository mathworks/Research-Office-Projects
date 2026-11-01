function [radius, halfExtent, zRange] = measureStowedEnvelope(robot, idx, cpl, cfg, hulls)
%MEASURESTOWEDENVELOPE Horizontal envelope of the robot with the arm stowed.
%
%   radius = MEASURESTOWEDENVELOPE(robot, idx, cpl, cfg) returns the largest
%   horizontal distance from the base origin to any point of the robot's visual
%   geometry that sits inside cfg.depth.occupancyBand, taken over the whole gripper
%   aperture range at cfg.arm.stowConfig. It is the radius of the cylinder the parked
%   robot sweeps out in the plane the navigation grid describes.
%
%   [radius, halfExtent, zRange] = MEASURESTOWEDENVELOPE(...) also returns
%
%       halfExtent  [max|x| max|y|] over the same geometry, band filter included.
%                   This is the pair cfg.plan.stowedArmHalfExtent states and
%                   hmmBaseFootprint builds its rectangle from.
%       zRange      [min max] z of the same geometry, band filter EXCLUDED, so that a
%                   caller can see an arm parked above the band rather than having it
%                   silently filtered out of the answer.
%
%   MEASURESTOWEDENVELOPE(..., hulls) reuses a collectStowHulls result instead of
%   recomputing it, which is what makes this affordable inside a search loop.
%
%   THIS NUMBER IS USED TWICE AND BOTH USES ARE SAFETY-RELEVANT.
%   hmmPlanBasePath inflates the occupancy map by it, so it decides how close
%   the base may be planned to a wall. hmmOccupancyFromDepth discards depth
%   returns inside it, so it decides which returns are the robot's own body
%   rather than the world. Getting it wrong in the SMALL direction fails in a
%   way that looks like anything but a geometry error: the fixed cameras see the
%   robot's own gripper, the cells it stands on are marked occupied, and after
%   inflation the base cannot prove its OWN pose valid, so every base plan
%   returns PathPlanFailed before a single tree node is sampled. That is exactly
%   what a hand-written 0.150 m did: the stowed fingers reach 0.513 m from the
%   base origin, and 0.278 + 0.150 = 0.428 m does not cover them.
%
%   WHY THE VALUE IS MEASURED HERE AND CHECKED, NOT MEASURED AND USED.
%   hmmOccupancyFromDepth and hmmPlanBasePath both run inside MATLAB Function
%   blocks and read hmmConfig directly, so they cannot be handed a rigidBodyTree
%   cheaply. cfg.plan.stowedArmRadius therefore stays the single place the value
%   is stated, and this measurement is the check that the stated value still
%   describes the robot that was actually assembled. Change cfg.arm.stowConfig
%   or the gripper and hmmSetup says so, with the number to write down.
%
%   THIS USED TO MEASURE FRAME ORIGINS, AND THE HEADER USED TO SAY THE MESHES WERE
%   UNREACHABLE. "rigidBodyTree reports a body's Collisions as a description string
%   rather than as geometry, so the STL vertices of the finger pads are not reachable
%   from the tree" - true of Collisions, and the wrong conclusion, because the VISUAL
%   geometry is reachable through getVisual and is the same geometry. The gap that
%   argument left was not small and it was not conservative: an origin is a point on a
%   link's axis, and at the shipped stow pose the arm's body origins stay inside
%   |y| <= 0.1799 m while its meshes reach 0.3474 m. So the stated
%   cfg.plan.stowedArmHalfExtent was 167.5 mm under the truth on the axis a doorway
%   measures, the arm visibly hung outside the chassis in a render nobody could
%   square with the numbers, and the stow search was optimising a quantity that
%   could not see the thing being minimised. The measurement is now taken over
%   hull-reduced visual meshes - see collectStowHulls, including why a hull is the
%   right reduction and why over-estimating is the safe direction.
%
%   cfg.plan.stowedArmRadius still carries an allowance above this, and
%   cfg.safety.plannerMargin sits above that again for planning. What has changed is
%   that the allowance is no longer covering a known blind spot.
%
%   See also collectStowHulls, assembleMobileManipulator, measureGripperCoupling,
%   hmmSetup.

%   Copyright 2026 The MathWorks, Inc.

arguments
    robot rigidBodyTree
    idx   struct
    cpl   struct
    cfg   struct
    hulls struct = collectStowHulls(robot)
end

band = cfg.depth.occupancyBand;

% The aperture is swept because closing the jaws swings the finger links FORWARD
% about the knuckles: the widest stowed pose is not whichever one stowConfig
% happens to be paired with. Sampling through the measured mimic multipliers
% means every configuration tested is one the real gripper can hold.
driver = linspace(cpl.limits(1, 1), cpl.limits(1, 2), 21);
mimic  = cpl.multipliers(:)';

radius     = 0;
halfExtent = [0 0];
zRange     = [inf -inf];

for d = driver
    q = zeros(1, idx.numDOF);
    q(idx.arm)     = cfg.arm.stowConfig;
    q(idx.gripper) = d * mimic;

    for k = 1:numel(hulls)
        T = getTransform(robot, q, hulls(k).name);
        P = (T(1:3, 1:3) * hulls(k).points' + T(1:3, 4))';

        zRange = [min(zRange(1), min(P(:, 3))), max(zRange(2), max(P(:, 3)))];

        % The band filter is applied per POINT rather than per body, which is the
        % only thing it can mean once the measurement is geometric: a link crossing
        % band(2) is partly inside the volume the map describes and partly outside,
        % and the old per-origin test scored the whole link on where its axis
        % happened to sit. Points on a hull edge that crosses the plane are not
        % reconstructed - the clip is by vertex, not by slab intersection - which
        % under-reports only for a body straddling the boundary. The stow search
        % hard-rejected any moving body above the band when it chose stowConfig, so
        % a pose this is asked about in anger has no straddling body to miss.
        in = P(:, 3) >= band(1) & P(:, 3) <= band(2);
        if any(in)
            Q = P(in, :);
            radius        = max(radius, max(hypot(Q(:, 1), Q(:, 2))));
            halfExtent(1) = max(halfExtent(1), max(abs(Q(:, 1))));
            halfExtent(2) = max(halfExtent(2), max(abs(Q(:, 2))));
        end
    end
end

end
