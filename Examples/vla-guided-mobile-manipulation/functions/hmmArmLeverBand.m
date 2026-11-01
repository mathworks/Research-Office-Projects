function [loLever, hiLever] = hmmArmLeverBand(cfg, targetZ, heldExtent)
%HMMARMLEVERBAND How far from its own mount the arm can work, at a target's height.
%
%   [loLever, hiLever] = HMMARMLEVERBAND(cfg, targetZ, heldExtent) returns the horizontal
%   mount-to-target distances at which the arm can hold a vertical tool over a target whose
%   SUPPORT SURFACE is at world height targetZ. hiLever comes back smaller than loLever when
%   no lever works at all, which is the honest answer for a target above the arm's reach and
%   is what lets callers refuse rather than clamp.
%
%   heldExtent IS OPTIONAL AND IT IS THE OBJECT IN THE GRIPPER, not the object being reached
%   for. Omit it - or pass zeros - and the lift falls back to cfg.sup.arrivalLift. Pass the
%   held object's [x y z] extent and the lift is computed from it exactly as the release pose
%   is: where hmmGraspAim aims on its height, plus the pad drop its grasp width implies,
%   plus cfg.plan.releaseClearance.
%
%   WHY A LEVER AND NOT A STAND-OFF, WHICH IS THE WHOLE REASON THIS FUNCTION EXISTS. The arm
%   is not mounted at the base centre. cfg.pedestal.offset puts it 60 mm aft and 27.5 mm to
%   one side, so the distance from the BASE to a target and the distance from the ARM to that
%   target differ by up to 66 mm, and which way they differ depends on where the base is
%   FACING. Every reach bound written as a base-centre radius therefore silently encodes a
%   heading and a mount position, and goes stale when either moves - which is exactly what
%   happened to cfg.sup.placeRadiusMax when the mount came down 200.9 mm and turned round.
%   A lever measured from the mount encodes neither, so this answer is the same whichever way
%   the base is pointing, and hmmStandoffRing is free to choose the heading.
%
%   AND THE ANSWER REALLY IS HEADING-INDEPENDENT, which is a fact about this arm rather than
%   an approximation. hmmArmCollisionModel's planTree is rooted at arm_mount and joint 1
%   turns about that frame's vertical axis with +/-2pi of travel, so moving a target round
%   the mount in azimuth is absorbed entirely by joint 1: measured across twelve azimuths at
%   a fixed lever, the pose error stayed at 1e-8 and the arm configuration reappeared with
%   joint 1 offset by the azimuth. Collision with the WORLD is not azimuth-independent and
%   this function does not claim it is - that is hmmScreenArmPlan's question, asked against
%   the full tree - but REACH is.
%
%   THE HEIGHT IS THE SUPPORT SURFACE, NOT THE TOOL. ObjectTarget.Position(3) is the surface
%   beneath the object, so the tool ends up one lift higher. The subtraction of
%   cfg.pedestal.armMountHeight is what makes the table's index mount-relative, so a change to
%   the layout re-indexes the curve instead of invalidating it.
%
%   AND THE LIFT HAD TO STOP BEING A CONSTANT, WHICH COST A RUN TO LEARN. cfg.sup.arrivalLift
%   was 0.100 m, chosen as a deliberate overestimate on the argument that reading the table
%   too high only costs "a few degrees of unnecessary heading, which reach is provably
%   indifferent to". That argument is wrong, and it is wrong in the one place it mattered: the
%   lift is added to dz, dz indexes the ceiling, and the ceiling is a LEVER. Over-estimating
%   the lift therefore shortens the admissible stand-off one for one, and on this scene 31 mm
%   of it closed the place ring completely - zero free bearings on all 24 at every radius from
%   0.30 to 0.80 m, the goal handed back unchanged, BaseGoalOccupied 20 ms into NAV_TO_PLACE,
%   and the mug dropped. The shipped mug's true place lift is 0.0691 m (0.65 * 0.090 of height,
%   7.62 mm of pad drop at a 70 mm grasp, 3 mm of release clearance) and at that figure the
%   same ring has two free bearings.
%
%   SO THE CONSTANT IS NOW A FALLBACK AND NOT THE ANSWER, and the fallback is still an
%   overestimate because that is the safe direction for the case where the object is unknown.
%   Note that it is an overestimate for a mug and an UNDERestimate for anything much taller -
%   a 0.15 m object lifts 0.108 m - so a single constant could not have been safe at both ends.
%   Every caller that knows what is in the gripper passes it: hmmSelectBaseGoal has
%   carriedExtent and hmmReconstructTargets/biasToReachable has the object's extent. The
%   pick leg carries nothing, so it gets the fallback, which is correct there for a different
%   reason - the object it is reaching for is not yet in the gripper and the tool still rises
%   by the grasp height to clear it.
%
%   BELOW THE MOUNT THE TABLE IS CLAMPED RATHER THAN EXTRAPOLATED. A target under the mount
%   is easier than one level with it - the arm can point down and out - so holding the dz = 0
%   value understates the reach and cannot certify a pose that does not solve. Above the
%   table's last breakpoint nothing solves at any lever, and that is reported rather than
%   extrapolated to a negative ceiling that would read as a very tight band.
%
%   See also hmmStandoffRing, hmmSelectBaseGoal, hmmArmCollisionModel.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg        struct
    targetZ    (1,1) double
    heldExtent (:,1) double = zeros(0, 1)
end

tbl = cfg.arm.leverCeiling;
dz  = targetZ + arrivalLift(cfg, heldExtent) - cfg.pedestal.armMountHeight;

loLever = cfg.arm.leverFloor;

if dz > tbl(1, end)
    % Above the curve. Returned as an empty band rather than as a clamp, so a caller that
    % tests loLever <= lever <= hiLever refuses every candidate instead of accepting the
    % ones nearest the mount.
    hiLever = -inf;
    return
end

% interp1 on the clamped index: linear between breakpoints because the admissible lever set
% was measured CONTIGUOUS at every sampled height, so there is nothing to step over.
hiLever = interp1(tbl(1, :), tbl(2, :), max(dz, tbl(1, 1)), 'linear');

end

%% ------------------------------------------------------------------------
function lift = arrivalLift(cfg, heldExtent)
%ARRIVALLIFT How high above its support surface the tool sits while holding heldExtent.
%
%   The release pose, in other words, because that is the pose a stand-off has to satisfy on
%   the place leg: the object's underside one release clearance off the surface, its grasp band
%   where hmmGraspAim puts it, and the tool one pad drop above the pad contact point.
%   hmmScreenArmPlan builds the same terms into its Tobj and hmmCarryTranslation into its
%   offset, and all three now ask hmmGraspAim rather than each repeating the expression; this is
%   that arithmetic read as a height rather than as a transform.
%
%   AN EMPTY OR ZERO EXTENT IS "NOTHING HELD, OR NOT KNOWN", and both take the fallback. An
%   all-zero extent is what the bus publishes before a grasp and what the chart sends on the
%   pick leg, so the two cases cannot be distinguished here and do not need to be: the
%   fallback overestimates, and overestimating refuses stand-offs rather than certifying them.
if isempty(heldExtent) || numel(heldExtent) < 3 || ~any(heldExtent(1:3))
    lift = cfg.sup.arrivalLift;
    return
end

% THROUGH hmmGraspAim, WHICH IS NOW THE ONLY PLACE THE GRASP HEIGHT IS DECIDED - the pad-drop
% interpolation moved there with it, so the clamp-then-interpolate this function used to do for
% itself is gone rather than duplicated. rise is the tool frame above the object's support
% surface; the release adds one clearance on top, because on the place leg the object's underside
% has to finish that far off the surface.
[~, rise] = hmmGraspAim(cfg, heldExtent(1:3), heldExtent(2));

lift = rise + cfg.plan.releaseClearance;
end
