function idx = mobileManipulatorIndexMap(robot)
%MOBILEMANIPULATORINDEXMAP Joint index map for the 15-DOF robot tree.
%
%   idx = MOBILEMANIPULATORINDEXMAP(robot) returns the mapping from named
%   joints to configuration-vector positions for the tree built by
%   assembleMobileManipulator. Every index is LOOKED UP in the tree's own
%   joint names and then ASSERTED, so a change in the shipping UR5e or
%   2F-85 models, or in the order addSubtree grafts them on, fails loudly
%   here instead of silently commanding the wrong joint.
%
%   Fields:
%       .base           1x3  [x y theta]
%       .baseX .baseY .baseTheta   scalars
%       .arm            1x6  UR5e joints, shoulder_pan .. wrist_3
%       .gripper        1x6  2F-85 joints in tree order
%       .gripperDriver  scalar, the one commanded 2F-85 joint
%       .gripperMimic   1x5  the coupled 2F-85 joints
%       .numDOF         total DOF (15)
%       .names          1x15 cell, joint names in configuration order
%       .bodyOfJoint    1x15 cell, body actuated by each joint
%
%   The index map is the single place any code is allowed to know that the
%   arm occupies elements 4:9. Nothing else in the project hard-codes a
%   configuration-vector slice.
%
%   See also assembleMobileManipulator, hmmConfig.

%   Copyright 2026 The MathWorks, Inc.

arguments
    robot rigidBodyTree
end

% Configuration-vector order is the order of non-fixed joints as they
% appear in robot.Bodies. Read it from the tree; never assume it.
allJoints  = cellfun(@(b) b.Joint.Name, robot.Bodies, 'UniformOutput', false);
allTypes   = cellfun(@(b) b.Joint.Type, robot.Bodies, 'UniformOutput', false);
allBodies  = cellfun(@(b) b.Name, robot.Bodies, 'UniformOutput', false);
isMovable  = ~strcmp(allTypes, 'fixed');
idx.names  = allJoints(isMovable);
% The body each movable joint actuates - needed to read a joint's axis back
% out of the tree without re-deriving the body/joint pairing anywhere else.
idx.bodyOfJoint = allBodies(isMovable);
idx.numDOF = numel(idx.names);

baseNames = {'base_x_joint', 'base_y_joint', 'base_theta_joint'};
armNames  = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', ...
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
gripNames = {'finger_joint', 'left_inner_finger_joint', ...
             'left_inner_knuckle_joint', 'right_inner_knuckle_joint', ...
             'right_outer_knuckle_joint', 'right_inner_finger_joint'};

idx.base    = lookupJoints(idx.names, baseNames, 'planar base');
idx.arm     = lookupJoints(idx.names, armNames,  'UR5e arm');
idx.gripper = lookupJoints(idx.names, gripNames, 'Robotiq 2F-85 gripper');

idx.baseX     = idx.base(1);
idx.baseY     = idx.base(2);
idx.baseTheta = idx.base(3);

% The driver joint is the only 2F-85 joint the gripper controller commands;
% the other five are mimic joints driven through the coupling gain.
idx.gripperDriver = lookupJoints(idx.names, {'finger_joint'}, 'gripper driver');
idx.gripperMimic  = setdiff(idx.gripper, idx.gripperDriver, 'stable');

%% Assertions - the map must partition the whole configuration vector
assert(idx.numDOF == 15, 'mobileManipulatorIndexMap:dofMismatch', ...
    'Expected 15 DOF, tree reports %d.', idx.numDOF);

covered = sort([idx.base, idx.arm, idx.gripper]);
assert(isequal(covered, 1:idx.numDOF), ...
    'mobileManipulatorIndexMap:notAPartition', ...
    ['base/arm/gripper indices must cover every configuration element ' ...
     'exactly once; got [%s] for %d DOF.'], num2str(covered), idx.numDOF);

assert(numel(idx.gripperMimic) == 5, ...
    'mobileManipulatorIndexMap:mimicCount', ...
    'Expected 5 mimic joints, found %d.', numel(idx.gripperMimic));

% Joint types must match what the commands assume.
movableTypes = allTypes(isMovable);
assert(all(strcmp(movableTypes(idx.base(1:2)), 'prismatic')), ...
    'mobileManipulatorIndexMap:baseTranslationType', ...
    'base_x_joint and base_y_joint must be prismatic.');
assert(strcmp(movableTypes{idx.baseTheta}, 'revolute'), ...
    'mobileManipulatorIndexMap:baseYawType', ...
    'base_theta_joint must be revolute.');
assert(all(strcmp(movableTypes([idx.arm idx.gripper]), 'revolute')), ...
    'mobileManipulatorIndexMap:rotaryType', ...
    'All arm and gripper joints must be revolute.');

end

%% ------------------------------------------------------------------------
function ind = lookupJoints(available, wanted, groupLabel)
%LOOKUPJOINTS Resolve joint names to indices, erroring on any miss.
ind = zeros(1, numel(wanted));
for k = 1:numel(wanted)
    hit = find(strcmp(available, wanted{k}));
    assert(isscalar(hit), 'mobileManipulatorIndexMap:jointLookup', ...
        ['Joint ''%s'' of the %s was expected exactly once in the tree, ' ...
         'found %d match(es). Available joints: %s.'], ...
        wanted{k}, groupLabel, numel(hit), strjoin(available, ', '));
    ind(k) = hit;
end
end
