function model = hmmArmCollisionModel()
%HMMARMCOLLISIONMODEL Trees and artefact masks the arm planner checks against.
%
%   model = HMMARMCOLLISIONMODEL() returns, built once per MATLAB session:
%
%       .fullTree      the authoritative 15-DOF rigidBodyTree
%       .fullArtefact  logical mask of body pairs that overlap by construction
%       .fullNB        numel(fullTree.Bodies) + 1
%       .planTree      the 12-DOF arm subtree, gripper joints PINNED OPEN
%       .planArtefact  the same mask for the subtree
%       .planNB        numel(planTree.Bodies) + 1
%       .planBaseFrame name of the frame planTree is rooted in
%       .toolFrame     name of the body carrying the tool centre point
%       .armPosBox     nJoints x 2 arm position limits the ROBOT has
%       .armPlanBox    the same box less cfg.arm.limitMargin, which planTree carries
%
%   THE TWO TREES DO NOT SHARE A BASE FRAME, AND CALLERS MUST KNOW WHICH IS WHICH.
%   fullTree is rooted at 'map', so a collision object for it is posed in WORLD
%   coordinates. planTree is a subtree, so its coordinates are its own base frame's,
%   and the same object must be transformed into that frame before it means the
%   same thing. .planBaseFrame is returned rather than assumed because subtree
%   names the new base after the PARENT of the body asked for, not after the body:
%   subtree(robot, 'base_link') comes back rooted in 'arm_mount'. Rooting it at
%   'arm_mount' instead would silently give a base frame 0.24 m lower - the
%   pedestal - and every collision check would be run against a surface a quarter
%   of a metre out of place.
%
%   planTree IS ROOTED AT THE SAME BODY AS hmmSetup's armIkTree, which is what
%   makes one transform enough: the Inverse Kinematics block solves in that frame,
%   the Get Jacobian block reports in it, and the collision environment is built in
%   it. Three consumers, one frame, no conversion to get wrong.
%
%   WHY AN ARTEFACT MASK EXISTS. checkCollision on the assembled tree reports
%   five body pairs in collision at every configuration, and none of them is a
%   real interference:
%
%       wheel_rr <-> base                        rigidly fixed to each other
%       wrist_2_link <-> tool0                   separated only by a roll joint
%       tool0 <-> gripper_mount                  rigidly fixed to each other
%       left_outer_finger <-> left_inner_finger_pad     the 2F-85 linkage
%       left_inner_knuckle <-> right_outer_finger       the 2F-85 linkage
%
%   These are overlaps in the imported collision meshes of parts that are bolted
%   together or that pivot about a shared axis. Every one is invariant to the
%   joints this stage moves: the first three are rigid or axisymmetric about the
%   joint between them, and the last two are inside the gripper linkage, which
%   arm planning holds fixed. So the mask is measured at the stow configuration
%   and subtracted from every later check. It is DERIVED at runtime from the tree,
%   not written down, which means a change to assembleMobileManipulator changes
%   the mask with it and the two cannot drift apart.
%
%   Without the mask nothing can be planned at all: manipulatorRRT rejects the
%   start configuration as self-colliding and returns immediately.
%
%   WHY THE GRIPPER IS PINNED OPEN. manipulatorRRT samples every movable joint of
%   the tree it is given, so a 12-DOF subtree yields plans that flap the fingers
%   during transit. Setting the six gripper joints' PositionLimits to [0 0] makes
%   the sampler return that value and only that value, so the plan is a pure
%   6-DOF arm motion while the gripper's geometry is still present for collision
%   checking. Zero is the OPEN pose of the 2F-85 in this assembly, which is both
%   the physically correct transit state and the widest the gripper ever is - the
%   conservative choice for a clearance check.
%
%   See also hmmScreenArmPlan, assembleMobileManipulator, checkCollision.

%   Copyright 2026 The MathWorks, Inc.

persistent cached
if ~isempty(cached)
    model = cached;
    return
end

cfg = hmmRuntimeConfig();

[full, armBase, toolBody] = sourceTree();
full.DataFormat = 'row';

planTree = subtree(full, armBase);
planTree.DataFormat = 'row';

% Everything the arm subtree does NOT drive is pinned. The names come from the
% gripper's own joint list rather than from a hard-coded set, so a different
% gripper subtree pins whatever joints it actually has.
armJointNames = movableJointNames(planTree);
gripperNames  = armJointNames(cfg.arm.nJoints + 1:end);
% tree.Bodies IS READ ONCE PER LOOP, NOT PER ITERATION: the getter rebuilds every rigidBody on
% each access (~92 ms on this tree, profiled 2026-10-13), and the four loops of this build read it
% ~110 times. The bodies are handles, so the cell holds the tree's own bodies either way.
planBodies = planTree.Bodies;
for k = 1:numel(planBodies)
    j = planBodies{k}.Joint;
    if any(strcmp(j.Name, gripperNames))
        j.PositionLimits = [0 0];
    end
end

% THE ARM JOINTS ARE NARROWED BEFORE THE SAMPLER EVER SEES THEM. manipulatorRRT samples
% the PositionLimits of the tree it is given, so a limit left at the robot's real travel is
% a limit it will put a waypoint on - and a waypoint on the wall becomes a reference that
% contopptraj's spline overshoots, which ArmControl/JointLimit then clamps, which
% SafetyMonitor reads as an acceleration step and answers with a JOINT_ACCEL E-stop. The
% true box is CAPTURED FIRST, because hmmScreenArmPlan needs it to check the reference it
% finally hands over against what the robot really has rather than against the margin.
model.armPosBox  = jointBox(planTree, cfg.arm.nJoints);
model.armPlanBox = hmmShrinkArmLimits(planTree, cfg);

qStowFull = zeros(1, 3 + cfg.arm.nJoints + cfg.gripper.nJoints);
qStowFull(4:3 + cfg.arm.nJoints) = cfg.arm.stowConfig;
qStowPlan = [cfg.arm.stowConfig, zeros(1, cfg.gripper.nJoints)];

model.fullTree     = full;
model.fullNB       = numel(full.Bodies) + 1;
model.fullArtefact = artefactMask(full, qStowFull);
model.planTree      = planTree;
model.planNB        = numel(planTree.Bodies) + 1;
model.planArtefact  = artefactMask(planTree, qStowPlan);
model.planBaseFrame = planTree.BaseName;
% Named from robotInfo for the same reason armBase is: hmmScreenArmPlan reproduces
% SafetyMonitor's reach test with getTransform between these two frames, and a tool name
% written down in the planner rather than taken from the assembly is a name that can drift
% away from the one the robot actually has.
model.toolFrame     = toolBody;

cached = model;

end

%% ------------------------------------------------------------------------
function mask = artefactMask(tree, q)
%ARTEFACTMASK Body pairs already overlapping at the nominal configuration.
[~, sep] = checkCollision(tree, q, 'IgnoreSelfCollision', 'off', ...
    'Exhaustive', 'on', 'SkippedSelfCollisions', 'parent');
mask = isnan(sep);
end

%% ------------------------------------------------------------------------
function box = jointBox(tree, nJoints)
%JOINTBOX The position limits of the first nJoints movable joints, as nJoints x 2.
%   constraintJointBounds lists the movable joints only, in configuration order.
B = constraintJointBounds(tree).Bounds;
if size(B, 1) < nJoints
    error('hmm:armCollisionModel:tooFewJoints', ...
        'The plan tree has %d movable joints but the arm has %d.', size(B, 1), nJoints);
end
box = B(1:nJoints, :);
end

%% ------------------------------------------------------------------------
function names = movableJointNames(tree)
%MOVABLEJOINTNAMES Joint names in tree order, fixed joints excluded.
names = cell(1, 0);
bodies = tree.Bodies;                       % once - see the note in the main body
for k = 1:numel(bodies)
    j = bodies{k}.Joint;
    if ~strcmp(j.Type, 'fixed')
        names{end + 1} = j.Name;                                       %#ok<AGROW>
    end
end
end

%% ------------------------------------------------------------------------
function [robot, armBase, toolBody] = sourceTree()
%SOURCETREE The one authoritative tree, from the base workspace if it is there.
%   hmmSetup has already assembled it and put it in the base workspace, so
%   reusing that instance costs nothing and guarantees the planner checks against
%   the same geometry the plant simulates. Reassembling is the fallback for a
%   direct call from a test, not the normal route.
%
%   armBase comes from robotInfo alongside the tree, never from a name written
%   here, so the subtree this function builds and the subtree hmmSetup hands the
%   Inverse Kinematics block are rooted at the same body by construction.
try
    robot    = copy(evalin('base', 'robot'));
    armBase  = evalin('base', 'robotInfo.armBase');
    toolBody = evalin('base', 'robotInfo.toolBody');
catch
    [robot, info] = assembleMobileManipulator(hmmConfig());
    armBase  = info.armBase;
    toolBody = info.toolBody;
end
end
