function [box, names] = hmmShrinkArmLimits(tree, cfg)
%HMMSHRINKARMLIMITS Hold the planning trees off the arm's real joint position box.
%
%   box = HMMSHRINKARMLIMITS(tree, cfg) narrows the PositionLimits of the arm joints of
%   TREE by cfg.arm.limitMargin at each end, in place, and returns the resulting
%   nJoints-by-2 box. [box, names] = ... also returns the joint names it changed.
%
%   TREE MUST BE A SUBTREE ROOTED AT THE ARM BASE - hmmSetup's armIkTree or
%   hmmArmCollisionModel's planTree - because the arm joints are identified as the FIRST
%   cfg.arm.nJoints movable joints in tree order, which is the same assumption
%   hmmArmCollisionModel already makes when it pins everything after them as the gripper.
%   Handed the full 15-DOF tree it would narrow the three base joints instead, so the
%   count is checked and the caller is told what it actually passed.
%
%   WHY THIS EXISTS. ArmControl/JointLimit is a hard Saturate at the robot's real travel,
%   and a saturation that engages puts a CORNER in the position command: the slope drops
%   discontinuously to zero, and SafetyMonitor's Discrete Derivative of the realised
%   velocity reads the corner as an acceleration step. On joint 3, whose band is +-pi and
%   so half of every other joint's, that step measured 8.55 rad/s^2 against a 4.0 box and
%   latched a JOINT_ACCEL E-stop mid-move. Nothing was being driven too hard; a planner
%   had simply been allowed to plan onto the wall. See cfg.arm.limitMargin.
%
%   IT IS THE TREES THAT ARE NARROWED, NOT A NUMBER PASSED AROUND. manipulatorRRT samples
%   the limits of the tree it is given and the Inverse Kinematics block enforces the
%   limits of the tree in its mask, so narrowing the trees reaches both without either
%   one having to be told about the margin. The consumer that must NOT be narrowed is
%   JointLimit itself, which reads armJointLimits - captured from the tree before this
%   runs - because the true travel is what the robot actually has.
%
%   rigidBodyTree IS A HANDLE CLASS and this mutates in place, deliberately, matching
%   hmmArmCollisionModel's gripper pinning. A caller that needs the unnarrowed tree must
%   copy() it first, which is why hmmSetup derives armDynTree and armTaskTree BEFORE
%   calling this: those two are the dynamics models and they keep the real box.
%
%   See also hmmArmCollisionModel, hmmScreenArmPlan, hmmSetup.

%   Copyright 2026 The MathWorks, Inc.

nArm   = double(cfg.arm.nJoints);
margin = cfg.arm.limitMargin;

joints = movableJoints(tree);
if numel(joints) < nArm
    error('hmm:shrinkArmLimits:tooFewJoints', ...
        ['The tree has %d movable joints but the arm has %d. This wants a subtree ' ...
         'rooted at the arm base, where the arm joints come first.'], ...
        numel(joints), nArm);
end

box   = zeros(nArm, 2);
names = cell(1, nArm);
for k = 1:nArm
    j   = joints{k};
    lim = j.PositionLimits;

    % A joint whose whole travel is narrower than the margin at both ends would come back
    % inverted, and an inverted PositionLimits is accepted silently and then samples
    % nothing - the planner would report no path with no reason given. Refuse instead.
    if diff(lim) <= 2 * margin
        error('hmm:shrinkArmLimits:marginTooLarge', ...
            ['Joint %s spans %.4f rad, which cfg.arm.limitMargin = %.4f cannot be ' ...
             'taken off both ends of.'], j.Name, diff(lim), margin);
    end

    j.PositionLimits = lim + [margin, -margin];
    box(k, :)  = j.PositionLimits;
    names{k}   = j.Name;
end

end

%% ------------------------------------------------------------------------
function joints = movableJoints(tree)
%MOVABLEJOINTS The joint objects in tree order, fixed joints excluded.
%   Handles, not copies, so assigning PositionLimits on one changes the tree.
joints = cell(1, 0);
bodies = tree.Bodies;   % once: the getter rebuilds every rigidBody per access (~92 ms here)
for k = 1:numel(bodies)
    j = bodies{k}.Joint;
    if ~strcmp(j.Type, 'fixed')
        joints{end + 1} = j;                                               %#ok<AGROW>
    end
end
end
