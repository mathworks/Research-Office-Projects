function [tree, jointNames] = softgripperReplayTree(P)
% SOFTGRIPPERREPLAYTREE  rigidBodyTree of the UR5e plus the soft gripper, for the 3D replay (REQ_GRIP_029).
%
%   [tree, jointNames] = SOFTGRIPPERREPLAYTREE(P)     P from softgripperPickParams
%
%   The toolbox UR5e (loadrobot, with its meshes) + a palm fixed at tool0, as a SERIAL tree. The fingers are
%   separate robots (softgripperFingerTree), placed on their mounts by softgripperReplayData. Geometry as in
%   SoftGripperPick: FlangeToGripper = 180 deg about tool0 X.
%   The replay drives the 6 arm joints from qArmMeas. Visualization only:
%   nothing here feeds back into the simulation.
%   jointNames: the 6 arm joint names in tree configuration order.

fi = P.finger; b = P.bench;
tree = loadrobot('universalUR5e', 'DataFormat', 'row');
% Serial only (2026-10-26): the toolbox tree branches at base_link (base) and wrist_3_link (ft_frame); both are
% bare reference frames, and in the replay the palm under the flange branch jittered against steady joint data.
removeBody(tree, 'base');
removeBody(tree, 'ft_frame');
palm = rigidBody('palm');
setFixedTransform(palm.Joint, [rotm2tform([1 0 0; 0 -1 0; 0 0 -1]) * 1]);        % 180 deg about X (FlangeToGripper)
% Visuals are MESHES (STL boxes), not 'Box' primitives (revised 2026-10-26): in the replay the primitive boxes
% updated out of step with the UR5e's mesh links, so the palm and fingers wobbled against the wrist while the arm
% moved. The STL files are written next to this function on first use.
meshDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'images', 'mesh');
palmStl = boxStl(meshDir, 'palm_box.stl', [b.wPalm b.wPalm b.tPalm]);
linkStl = boxStl(meshDir, 'finger_link_box.stl', [fi.w_Link fi.h_Link fi.L / fi.N_Joint]);
addVisual(palm, 'Mesh', palmStl, trvec2tform([0 0 b.tPalm/2]));
% Realistic mass properties (revised 2026-10-26): the rigidBody default is 1 kg with inertia 1 kg*m^2. The replay
% renderer drives the joints physically, and 33 such bodies on the UR5e wrist made the wrist twist back and forth.
mPalm = 0.05;                                       % kg  EST (display only)
setBoxMass(palm, mPalm, [b.wPalm b.wPalm b.tPalm], [0 0 b.tPalm/2]);
addBody(tree, palm, 'tool0');
% The fingers are NOT in this tree (2026-10-26): the Simulation 3D Robot block mis-draws branching trees, so
% each finger is its own serial robot (softgripperFingerTree), placed on its mount every frame.
% Gimbal-lock split (2026-10-26, the root cause of the wobbling wrist): the robot block sends each body's rotation
% RELATIVE TO ITS PARENT to Unreal as Euler angles. For wrist_2_link and flange that relative rotation sits at
% pitch = 90 deg, where roll and yaw are undefined; joint changes below 1e-4 deg flipped them by 170-340 deg
% between frames, and the wrist jumped back and forth. Each such body gets an invisible intermediate frame that
% takes half of the rotation, so neither half is near 90 deg pitch. The drawn geometry is unchanged.
tree = splitGimbalBodies(tree, P.arm.qHome);
cfg = homeConfiguration(tree);
jointNames = strings(1, numel(cfg));
n = 0;
for i = 1:tree.NumBodies
    j = tree.Bodies{i}.Joint;
    if ~strcmp(j.Type, 'fixed'), n = n + 1; jointNames(n) = string(j.Name); end
end
end

function tree = splitGimbalBodies(tree, qRef)
% Insert a fixed body '<name>_half' between a body and its parent wherever the parent-to-body rotation at qRef has
% |pitch| (ZYX, the robot block's 3-2-1 decomposition) above pitchMax; it takes half the rotation (axis-angle).
pitchMax = 60 * pi/180;                             % rad  well clear of the 90 deg singularity
names = tree.BodyNames;
for i = 1:numel(names)
    b = names{i};
    par = tree.getBody(b).Parent.Name;
    T = getTransform(tree, qRef, b, par);
    eul = rotm2eul(T(1:3, 1:3), 'ZYX');
    if abs(eul(2)) <= pitchMax, continue, end
    aa = rotm2axang(T(1:3, 1:3));
    Th = [axang2rotm([aa(1:3) aa(4)/2]) zeros(3, 1); 0 0 0 1];
    sub = removeBody(tree, b);
    half = rigidBody([b '_half']);
    setFixedTransform(half.Joint, Th);
    addBody(tree, half, par);
    jOld = sub.getBody(b).Joint;
    jNew = rigidBodyJoint(jOld.Name, jOld.Type);
    if ~strcmp(jOld.Type, 'fixed')
        jNew.JointAxis = jOld.JointAxis; jNew.PositionLimits = jOld.PositionLimits; jNew.HomePosition = jOld.HomePosition;
    end
    setFixedTransform(jNew, Th \ jOld.JointToParentTransform);
    assert(norm(jOld.ChildToJointTransform - eye(4)) < 1e-12, 'splitGimbalBodies: %s has a child-to-joint offset', b);
    replaceJoint(sub, b, jNew);
    addSubtree(tree, half.Name, sub);
end
end

function f = boxStl(dirName, name, sz)
% Axis-aligned box of size sz (m), centred on the origin, as an STL mesh file (created once).
if ~isfolder(dirName), mkdir(dirName); end
f = fullfile(dirName, name);
if isfile(f), return, end
[x, y, z] = ndgrid([-1 1] * sz(1)/2, [-1 1] * sz(2)/2, [-1 1] * sz(3)/2);
V = [x(:) y(:) z(:)];
K = convhull(V, 'Simplify', true);
stlwrite(triangulation(K, V), f);
end

function setBoxMass(body, m, sz, c)
% Solid-box mass properties about the body frame origin; centre of mass c (body frame).
I = m / 12 * [sz(2)^2 + sz(3)^2, sz(1)^2 + sz(3)^2, sz(1)^2 + sz(2)^2];
body.Mass = m;
body.CenterOfMass = c;
body.Inertia = [I + m * [c(2)^2 + c(3)^2, c(1)^2 + c(3)^2, c(1)^2 + c(2)^2], -m*c(2)*c(3), -m*c(1)*c(3), -m*c(1)*c(2)];
end
