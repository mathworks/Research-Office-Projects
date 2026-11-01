function [aim, rise, pad] = hmmGraspAim(cfg, objExtent, graspExtent)
%HMMGRASPAIM Where the jaws close on an object, as a height above its support surface.
%
%   [aim, rise, pad] = HMMGRASPAIM(cfg, objExtent) returns, for an object of extent
%   objExtent = [x y z] standing on a surface:
%
%     aim   the height of the PAD FACE CENTRE above that surface - where the gripping
%           surface is aimed, and the only one of the three that is a choice
%     rise  the height of the TOOL FRAME above that surface, which is aim plus the pad
%           drop, and is what a planner aims the end effector at
%     pad   the pad drop itself, interpolated from cfg.grasp.padDropTable
%
%   [aim, rise, pad] = HMMGRASPAIM(cfg, objExtent, graspExtent) takes the jaw gap from
%   graspExtent instead of from objExtent(2). The ObjectTarget bus carries GraspExtent as
%   its own field, so a caller holding the bus should pass it; objExtent(2) is the right
%   default for a caller holding nothing but an extent, and is what the bus field is
%   derived from.
%
%   MID-HEIGHT, CLAMPED, AND THE CLAMP IS WHY THIS IS A FUNCTION. The aim is the object's
%   mid-height - the jaws close across the middle of whatever they are given - bounded by
%   two facts about the gripper rather than about the object:
%
%       aim = max( min( h/2, h - padHalf ), padHalf + cfg.grasp.padGroundClear )
%
%   where h is objExtent(3) and padHalf is half of cfg.gripper.padFaceLength. The pad face
%   is 37.5 mm tall, so it reaches padHalf = 18.75 mm below and above the aim point, and
%   both bounds are statements about where that face ends up:
%
%     the FLOOR keeps the pad's bottom edge cfg.grasp.padGroundClear above the support, so
%     a short object is not grasped by driving the jaws at the tabletop, and
%
%     the CEILING keeps the pad's top edge under the object's top, so a tall object is not
%     grasped by closing partly on the air above it.
%
%   THE FLOOR OUTRANKS THE CEILING, DELIBERATELY, because they can conflict and they are not
%   the same kind of fact. The support surface is real geometry the arm must not drive into;
%   the object's top is a preference, and overshooting it only means part of the pad face
%   stands clear of the object while the rest still closes on it. Written as max(min(...))
%   the floor wins, which is the order that keeps the jaws out of the table on an object too
%   short to satisfy both. An object has to be shorter than
%   2*(padHalf + padGroundClear) = 57.5 mm before the floor binds at all; above that this
%   returns h/2 exactly.
%
%   AND THE CEILING IS INERT AT EVERY HEIGHT, WHICH IS WORTH STATING RATHER THAN DISCOVERING.
%   min(h/2, h - padHalf) picks the second term only when h < 2*padHalf = 37.5 mm, and there
%   h - padHalf < padHalf <= padHalf + padGroundClear, so the floor has already overridden it.
%   So this expression equals max(h/2, padHalf + padGroundClear) for every h and every
%   non-negative padGroundClear; the ceiling could only decide an answer if the pad face were
%   allowed BELOW the support surface, which is not a setting this project has. The term is kept
%   in both implementations because it is the written rule and both constraints should be visible
%   to a reader - but a reader should not believe it is doing work, and counting how often each
%   branch decides the answer measured the zero rather than assumed it.
%
%   WHY NOT A FIXED FRACTION OF THE HEIGHT, WHICH IS WHAT THIS REPLACED. cfg.grasp.
%   heightFraction was 0.65, chosen so that the pad face on the 90 mm shipped mug sat 40.6 mm
%   clear of the table rather than the 26.3 mm mid-height gives. That is a sound choice for a
%   90 mm object and it does not generalise: a fraction scales the clearance with the object,
%   so the clearance vanishes exactly when it matters, and its own comment named 53.6 mm as
%   the height below which 0.65 puts half the pad above the object. Clamping an absolute
%   mid-height is the same reasoning with the two constants expressed as what they are -
%   lengths of the gripper - and it needs no per-object tuning. On the shipped mug it aims at
%   45.0 mm instead of 58.5 mm: pad face 26.25 to 63.75 mm, symmetric about the middle,
%   26.25 mm clear of the table and 26.25 mm below the rim.
%
%   THIS IS THE ONE COPY, AND THE MODEL IS THE OTHER HALF OF THAT CLAIM. The same arithmetic
%   is built from blocks in MotionPlanning/TargetSelect - ObjHeight, MidHeight, PadTopLimit,
%   AimUnderTop, PadFloor, AimAboveTable - because the model is the product and the grasp
%   target belongs in it. Every OFFLINE consumer calls this function instead of rebuilding the
%   expression: hmmArmLeverBand's arrival lift, hmmScreenArmPlan's held-object transform and
%   hmmCarryTranslation's vertical offset each had their own copy of the fraction, and three
%   copies of a rule that changes is three chances to disagree. The block chain was checked
%   against this function by reading the blocks' own parameters.
%
%   See also hmmCarryTranslation, hmmArmLeverBand, hmmScreenArmPlan, hmmConfig.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg         struct
    objExtent   (:,1) double
    graspExtent (1,1) double = objExtent(2)
end

h       = double(objExtent(3));
padHalf = 0.5 * double(cfg.gripper.padFaceLength);

aim = max(min(0.5 * h, h - padHalf), padHalf + double(cfg.grasp.padGroundClear));

% Clamp-then-interpolate: the table is a measurement over the aperture the pads were
% characterised at, and extrapolating off either end invents a contact geometry that was
% never measured. Off the top end it invents a NEGATIVE pad drop, which would move the aim
% the wrong way.
%
% THE MODEL'S LOOKUP TABLE BLOCK NOW DOES THE SAME, AND IT DID NOT USED TO. This line once
% asserted the agreement as though it were obvious; MotionPlanning/TargetSelect/PadDrop was in
% fact set to ExtrapMethod 'Linear', and the gap measured zero inside the breakpoints, 0.91 mm
% at a jaw gap the robot can reach, 3.16 mm at a 100 mm gap it cannot. The block is now 'Clip'
% and every disagreement measures exactly 0. The history stays in this comment because the
% lesson is not about the parameter - it is that the comparison could not see the divergence
% until its gap sweep was made to leave the table's range, since INSIDE the breakpoints every
% interpolation method agrees by construction.
gap = min(max(graspExtent, cfg.grasp.padDropWidth(1)), cfg.grasp.padDropWidth(end));
pad = interp1(cfg.grasp.padDropWidth, cfg.grasp.padDropTable, gap);

rise = aim + pad;

end
