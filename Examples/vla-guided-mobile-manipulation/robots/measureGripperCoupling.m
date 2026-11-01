function cpl = measureGripperCoupling(robot, idx, cfg)
%MEASUREGRIPPERCOUPLING Measure the 2F-85 mimic-joint signs from the tree.
%
%   cpl = MEASUREGRIPPERCOUPLING(robot, idx, cfg) determines the coupling
%   multiplier of every Robotiq 2F-85 joint against the single driver joint
%   by MEASURING the assembled geometry. The design forbids assuming these
%   signs: the shipped rigidBodyTree cuts the physical four-bar loop, so a
%   sign that is right for one import can be wrong for another, and a wrong
%   sign shows up as fingers that open when commanded to close.
%
%   Three independent measurements, one per joint role:
%
%     Outer knuckles - the sign that makes measured pad separation DECREASE
%       as the driver angle increases. The left outer knuckle IS the driver,
%       so only the right one is searched.
%
%     Inner fingers - the sign that keeps each jaw PARALLEL, measured per
%       side as the drift of that pad's orientation relative to the gripper
%       base away from its rest value over the driver range. The two signs
%       are cleanly separated: one holds the drift at 0 rad, the other
%       reaches about 1.6 rad.
%
%     Inner knuckles - these are dangling visual branches in the cut tree
%       and do not move the pads at all, so pad-based criteria cannot
%       constrain them. Their sign comes from the parallel-linkage fact that
%       an inner knuckle shares its outer knuckle's WORLD rotation axis:
%       +1 when the two measured axes agree, -1 when they oppose.
%
%   Returned fields:
%       .joints        1x6 cell, gripper joint names in config order
%       .indices       1x6 config-vector indices (== idx.gripper)
%       .multipliers   6x1 coupling multipliers, driver element == 1
%       .driverIndex   config index of the driver joint
%       .limits        6x2 PositionLimits implied by driverRange
%       .padBodies     1x2 pad body names
%       .padNormalAxis measured pad-frame face-normal axis index
%       .padNormalSign sign of that axis pointing at the opposing pad
%       .evidence      struct of the measurements that decided each sign:
%                      .rightOuterSepPlus/.rightOuterSepMinus (m),
%                      .jawOrientationDrift (2x2, rad),
%                      .knuckleAxisDot (1x2)
%
%   See also assembleMobileManipulator, measureGripperApertureMap.

%   Copyright 2026 The MathWorks, Inc.

arguments
    robot rigidBodyTree
    idx struct
    cfg struct
end

padL = 'left_inner_finger_pad';
padR = 'right_inner_finger_pad';
q0   = homeConfiguration(robot);
drv  = linspace(cfg.gripper.driverRange(1), cfg.gripper.driverRange(2), 9);

%% Measure the pad face normal instead of assuming an axis
TL = getTransform(robot, q0, padL);
TR = getTransform(robot, q0, padR);
joinDir = TR(1:3,4) - TL(1:3,4);
assert(norm(joinDir) > 1e-6, 'measureGripperCoupling:padsCoincident', ...
    'The two finger pads coincide at the home configuration.');
joinDir = joinDir / norm(joinDir);

alignment = TL(1:3,1:3)' * joinDir;          % pad-L axes projected on the join
[bestAlign, normalAxis] = max(abs(alignment));
assert(bestAlign > 0.9, 'measureGripperCoupling:noFaceNormal', ...
    ['No pad-frame axis is aligned with the line joining the pads ' ...
     '(best |cos| = %.3f); the pad frame convention has changed.'], bestAlign);
normalSign = sign(alignment(normalAxis));    % +1 if that axis points at pad R

cpl.padBodies     = {padL, padR};
cpl.padNormalAxis = normalAxis;
cpl.padNormalSign = normalSign;

%% Joint roles, resolved by name against the tree's own joints
cpl.joints  = idx.names(idx.gripper);
cpl.indices = idx.gripper;
cpl.driverIndex = idx.gripperDriver;

iDriver     = jointSlot(cpl.joints, cfg.gripper.driverJoint);
iLeftFing   = jointSlot(cpl.joints, 'left_inner_finger_joint');
iRightFing  = jointSlot(cpl.joints, 'right_inner_finger_joint');
iLeftKnuck  = jointSlot(cpl.joints, 'left_inner_knuckle_joint');
iRightKnuck = jointSlot(cpl.joints, 'right_inner_knuckle_joint');
iRightOuter = jointSlot(cpl.joints, 'right_outer_knuckle_joint');

mult = zeros(numel(cpl.joints), 1);
mult(iDriver) = 1;

%% 1. Right outer knuckle: the sign that closes the jaws
sepTrend = zeros(1,2);
trialSigns = [1 -1];
for t = 1:2
    q = q0;
    q(cpl.indices(iDriver))     = drv(end);
    q(cpl.indices(iRightOuter)) = trialSigns(t) * drv(end);
    sepTrend(t) = padSeparation(robot, q, padL, padR);
end
sepOpen = padSeparation(robot, q0, padL, padR);
[~, tBest] = min(sepTrend);
mult(iRightOuter) = trialSigns(tBest);
assert(sepTrend(tBest) < sepOpen - 1e-4, ...
    'measureGripperCoupling:jawsDoNotClose', ...
    ['Neither sign of right_outer_knuckle_joint closes the jaws ' ...
     '(open %.5f m, best closed %.5f m).'], sepOpen, sepTrend(tBest));
cpl.evidence.rightOuterSepPlus  = sepTrend(1);
cpl.evidence.rightOuterSepMinus = sepTrend(2);

%% 2. Inner fingers: the sign that keeps the jaws PARALLEL as they close
% The 2F-85 is a parallel-jaw gripper: each pad translates without rotating
% relative to the gripper base. Measure that drift and take the sign holding
% it at zero. Two metrics that look equivalent are NOT: pad-face-versus-the
% -line-joining-the-pads fails because the pads also translate laterally as
% the linkage closes, so that line stops being normal to the faces away from
% rest. Each side is measured on its OWN pad against the gripper base, not on the
% two pads against each other: a pad-to-pad metric conflates the two signs,
% because leaving the far finger uncoupled rotates the far pad and shows up
% as a deviation attributed to the near one.
fingerSlots = [iLeftFing, iRightFing];
outerSlots  = [iDriver,   iRightOuter];
padNames    = {padL, padR};
for f = 1:2
    Trest = getTransform(robot, q0, padNames{f}, gripperBaseName());
    Rrest = Trest(1:3,1:3);
    devn = zeros(1,2);
    for t = 1:2
        worst = 0;
        for j = 1:numel(drv)
            q = q0;
            q(cpl.indices(iDriver))        = drv(j);
            q(cpl.indices(iRightOuter))    = mult(iRightOuter) * drv(j);
            q(cpl.indices(outerSlots(f)))  = mult(outerSlots(f)) * drv(j);
            q(cpl.indices(fingerSlots(f))) = trialSigns(t) * drv(j);
            worst = max(worst, jawOrientationDrift(robot, q, ...
                padNames{f}, gripperBaseName(), Rrest));
        end
        devn(t) = worst;
    end
    [bestDev, tBest] = min(devn);
    mult(fingerSlots(f)) = trialSigns(tBest);
    cpl.evidence.jawOrientationDrift(f, :) = devn;
    assert(bestDev < 1e-3, 'measureGripperCoupling:jawsNotParallel', ...
        ['Neither sign of ''%s'' keeps its jaw parallel (best drift ' ...
         '%.4f rad); the 2F-85 linkage is no longer a parallel-jaw ' ...
         'mechanism in this import.'], cpl.joints{fingerSlots(f)}, bestDev);
end

%% 3. Inner knuckles: share their outer knuckle's world rotation axis
knuckPairs = [iLeftKnuck,  jointSlot(cpl.joints, cfg.gripper.driverJoint); ...
              iRightKnuck, iRightOuter];
gripperBase = gripperBaseName();
for p = 1:2
    inner = knuckPairs(p,1); outer = knuckPairs(p,2);
    aInner = jointAxisIn(robot, q0, cpl.joints{inner}, gripperBase, idx);
    aOuter = jointAxisIn(robot, q0, cpl.joints{outer}, gripperBase, idx);
    agree = dot(aInner, aOuter);
    assert(abs(agree) > 0.9, 'measureGripperCoupling:knuckleAxisSkew', ...
        ['Inner knuckle ''%s'' is not parallel to its outer knuckle ' ...
         '(cos = %.3f); the parallel-linkage assumption no longer holds.'], ...
        cpl.joints{inner}, agree);
    mult(inner) = sign(agree) * mult(outer);
    cpl.evidence.knuckleAxisDot(p) = agree;
end

cpl.multipliers = mult;

%% PositionLimits implied by the coupling over the driver range
lo = min(mult * cfg.gripper.driverRange(1), mult * cfg.gripper.driverRange(2));
hi = max(mult * cfg.gripper.driverRange(1), mult * cfg.gripper.driverRange(2));
cpl.limits = [lo, hi];

assert(all(mult ~= 0), 'measureGripperCoupling:unresolvedSign', ...
    'One or more coupling multipliers were not resolved.');

end

%% ------------------------------------------------------------------------
function d = padSeparation(robot, q, padL, padR)
%PADSEPARATION Distance between the two pad frame origins, in metres.
T = getTransform(robot, q, padR, padL);
d = norm(T(1:3,4));
end

function a = jawOrientationDrift(robot, q, padName, refBody, Rrest)
%JAWORIENTATIONDRIFT Angle, in radians, by which one pad's orientation has
%drifted from its rest value. Zero for a true parallel-jaw motion.
T = getTransform(robot, q, padName, refBody);
dR = T(1:3,1:3) * Rrest';
a = acos(min(1, max(-1, (trace(dR) - 1)/2)));
end

function name = gripperBaseName()
%GRIPPERBASENAME Body name of the Robotiq 2F-85 base link.
name = 'robotiq_arg2f_base_link';
end

function a = jointAxisIn(robot, q, jointName, refBody, idx)
%JOINTAXISIN Joint rotation axis expressed in refBody coordinates.
slot = find(strcmp(idx.names, jointName), 1);
bodyName = idx.bodyOfJoint{slot};
b = getBody(robot, bodyName);
T = getTransform(robot, q, bodyName, refBody);
a = T(1:3,1:3) * b.Joint.JointAxis(:);
a = a / max(norm(a), eps);
end

function slot = jointSlot(names, wanted)
%JOINTSLOT Position of a joint name within the gripper joint list.
slot = find(strcmp(names, wanted), 1);
assert(~isempty(slot), 'measureGripperCoupling:missingJoint', ...
    'Gripper joint ''%s'' is not present. Available: %s.', ...
    wanted, strjoin(names, ', '));
end
