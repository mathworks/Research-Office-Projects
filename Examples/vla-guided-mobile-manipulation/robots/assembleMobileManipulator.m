function [robot, info] = assembleMobileManipulator(cfg)
%ASSEMBLEMOBILEMANIPULATOR Build the one authoritative 15-DOF robot tree.
%
%   [robot, info] = ASSEMBLEMOBILEMANIPULATOR(cfg) returns the single
%   rigidBodyTree that every part of the project uses: planning, control,
%   collision checking, forward kinematics for the Sim3D actor chain, and
%   the wrist-camera mount. The design requires ONE tree - no second copy
%   is created anywhere.
%
%   Joint layout (15 non-fixed joints, in tree order):
%       1  base_x_joint          prismatic  world X
%       2  base_y_joint          prismatic  world Y
%       3  base_theta_joint      revolute   world Z
%       4..9   UR5e shoulder_pan .. wrist_3 revolute
%       10..15 Robotiq 2F-85 driver + mimic  revolute
%
%   The chassis body is named chassis_link, not base_link: loadrobot's UR5e
%   already uses base_link, and rigidBodyTree body names must be unique.
%
%   The planar base is three explicit single-DOF joints rather than a
%   floating joint: a floating joint cannot be commanded from the
%   Simulink robotics blocks, and the base is holonomic in the plane, so
%   prismatic-X / prismatic-Y / revolute-Z is both sufficient and exactly
%   what the (vx, vy, omega) command triple integrates into.
%
%   Named frames added by this function:
%       tcp_frame     tool centre point, on the gripper axis between pads
%       grasp_frame   the frame the grasp gate tests, at the pad midpoint
%       wrist_cam     wrist camera optical frame
%
%   info is a struct of provenance and geometry actually realised:
%       .jointNames      1xN cell, tree joint order
%       .bodyNames       1xM cell
%       .armBase         name of the UR5e base body
%       .gripperBase     name of the 2F-85 base body
%       .padBodies       the two finger pad body names
%       .toolBody        body carrying tcp_frame
%       .gripperCoupling measured 2F-85 mimic multipliers and limits
%       .apertureMap     measured aperture <-> driver-angle lookup data
%       .sources         struct of asset provenance strings
%
%   See also hmmConfig, mobileManipulatorIndexMap, loadrobot, addSubtree.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg struct = hmmConfig()
end

%% Vendor base meshes, resolved relative to this file
% NOT an absolute path from cfg, and not a discovery walk either. These are
% vendor assets the user downloads into a fixed folder beside this function
% (README, "Robot Base Meshes"), unlike the runtime assets hmmAssetRoots
% searches for, so they are addressed relative to mfilename and the folder is
% required rather than optional. The absolute
% path this produces is baked into the returned tree, which is rebuilt by
% setupParams on every machine and never committed.
meshDir = fullfile(fileparts(mfilename('fullpath')), 'meshes', 'agv_pro');

% WHEEL MESH PER CORNER, AND THE MAPPING IS MEASURED. The vendor's link names
% are self-contradictory - left_front and left_rear share x = +0.17128 and
% differ only in the sign of y, so "left/right" selects +/-X while the lidar
% and camera at +x say forward is +X - so a corner read off a filename is a
% coin toss. Each wheel mesh is instead offset to one side of its own origin,
% spanning y in [-0.0463, 0] or [0, +0.0463], and the tyre must sit INBOARD of
% its mount; that fixes the y sign for all four independently, and the answer
% agrees with the vendor's joint origins. See meshes/agv_pro/PROVENANCE.md.
%   wheelNames order below is {fl, fr, rl, rr} = (+x+y), (+x-y), (-x+y), (-x-y).
wheelMesh = {'wheel_lf_link.stl', 'wheel_lb_link.stl', ...
             'wheel_rf_link.stl', 'wheel_rb_link.stl'};

meshFiles = [{'base_link.stl', 'laser_link.stl'}, wheelMesh];
% THE MESHES ARE ONLY THE LOOK OF THE BASE, so the example runs without them. Nothing
% that plans, maps or checks safety reads them: the base is the numbers in
% cfg.base, and the collision geometry is a box. Without the six files the base is
% drawn as primitives the same size as each mesh's bounding box (PROVENANCE.md),
% and setup says where to get the real ones.
haveMeshes = all(cellfun(@(f) isfile(fullfile(meshDir, f)), meshFiles));
if ~haveMeshes
    fprintf(['assembleMobileManipulator: the myAGV Pro meshes are not in %s, so the ' ...
             'base is drawn as plain shapes. Everything else is unchanged. To render ' ...
             'the real base, download them: README.md, section "Robot Base Meshes".\n'], ...
            meshDir);
end

%% Planar base chain: three commanded DOF on the floor plane
robot = rigidBodyTree('DataFormat', cfg.arm.dataFormat, ...
    'MaxNumBodies', 128);
robot.BaseName = 'map';
robot.Gravity  = cfg.frames.gravity;

% base_x: slides along world X
bx = rigidBody('base_x_link');
bx.Joint = rigidBodyJoint('base_x_joint', 'prismatic');
bx.Joint.JointAxis      = [1 0 0];
bx.Joint.PositionLimits = [-25 25];
bx.Mass = 0;
bx.Inertia = [1e-6 1e-6 1e-6 0 0 0];
addBody(robot, bx, 'map');

% base_y: slides along world Y
by = rigidBody('base_y_link');
by.Joint = rigidBodyJoint('base_y_joint', 'prismatic');
by.Joint.JointAxis      = [0 1 0];
by.Joint.PositionLimits = [-25 25];
by.Mass = 0;
by.Inertia = [1e-6 1e-6 1e-6 0 0 0];
addBody(robot, by, 'base_x_link');

% base_theta: yaw about world Z, and the body that carries the chassis
bt = rigidBody('chassis_link');
bt.Joint = rigidBodyJoint('base_theta_joint', 'revolute');
bt.Joint.JointAxis      = [0 0 1];
bt.Joint.PositionLimits = [-inf inf];
setFixedTransform(bt.Joint, trvec2tform([0 0 cfg.base.baseLinkHeight]));
bt.Mass = cfg.base.chassisMass;
bt.CenterOfMass = [-0.0076254 -0.00023134 0.06693];   % vendor base_link inertial origin
bt.Inertia = boxInertia(cfg.base.chassisMass, cfg.base.chassisSize);
% VISUAL IS THE VENDOR MESH, COLLISION IS A MEASURED BOX, and the split is
% deliberate. base_link.stl carries 89064 triangles, which is what makes the
% render look like a myAGV Pro and exactly what a collision query must not have
% to walk - the arm self-collision check and the planner both run against this
% body. The box is not a guess either: it is chassisSize, which IS the mesh
% bounding box, so the two agree to within the 0.5 mm by which the mesh is
% off-centre in x and y.
%
% The mesh needs no transform. It is authored in the vendor's own base_link
% frame with its underside on z = 0, and chassis_link IS that frame - see
% cfg.base.baseLinkHeight, which is derived so the two coincide. The box, being
% centred on its own origin, still needs the half-height lift.
if haveMeshes
    addVisual(bt, 'Mesh', fullfile(meshDir, 'base_link.stl'));
else   % the chassis bounding box, resting on z = 0 like the mesh
    addVisual(bt, 'Box', cfg.base.chassisSize, trvec2tform([0 0 cfg.base.chassisSize(3)/2]));
end
addCollision(bt, collisionBox(cfg.base.chassisSize(1), ...
    cfg.base.chassisSize(2), cfg.base.chassisSize(3)), ...
    trvec2tform([0 0 cfg.base.chassisSize(3)/2]));
addBody(robot, bt, 'base_y_link');

% Four mecanum wheels as fixed visual bodies. They spin in the render only
% through the actor layer; their joint state is not part of the 15 DOF
% because the base is commanded as a twist, not per-wheel.
wheelNames = {'wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr'};
wheelSigns = [ 1  1; 1 -1; -1  1; -1 -1];   % [xSign ySign]
% Vendor per-corner inertial masses, in the wheelNames order above. They are
% reproduced as published even though four identical wheels have no business
% massing 0.302, 0.296, 0.217 and 0.217 kg - that spread is a CAD artefact in
% the vendor file, and inventing an average to tidy it up would replace measured
% provenance with a preference. The base is commanded as a twist rather than
% simulated per wheel, so nothing downstream integrates these.
wheelMass = [0.3015, 0.29613, 0.21659122149244, 0.21659];
wheelWidth = 0.0463;    % m, measured from the meshes
for k = 1:4
    w = rigidBody(wheelNames{k});
    w.Joint = rigidBodyJoint([wheelNames{k} '_mount'], 'fixed');
    % z = wheelRadius - baseLinkHeight puts the axle exactly one radius above
    % the floor. With the vendor numbers it evaluates to 0.0520, which is the
    % URDF's own wheel joint origin - two independent conventions agreeing.
    setFixedTransform(w.Joint, trvec2tform([ ...
        wheelSigns(k,1)*cfg.base.halfLength, ...
        wheelSigns(k,2)*cfg.base.halfWidth, ...
        cfg.base.wheelRadius - cfg.base.baseLinkHeight]));
    w.Mass = wheelMass(k);
    w.Inertia = cylinderInertia(wheelMass(k), cfg.base.wheelRadius, wheelWidth);
    % NO TRANSFORM, AND NO COLLISION. The mesh is authored in its own wheel-link
    % frame, already centred on the axle and already spinning about Y, so the
    % eul2tform([0 pi/2 0]) that used to orient the stand-in cylinder would now
    % rotate a correct mesh into being wrong. That rotation was itself wrong:
    % it mapped the cylinder axis onto X, while a wheel at +/-Y must turn about
    % Y, so the four stand-ins were lying on their sides for the whole project.
    % Collision is still omitted, as it was for the cylinders - a wheel is in
    % contact with the floor by definition and the chassis box already covers
    % the footprint the planner cares about.
    if haveMeshes
        addVisual(w, 'Mesh', fullfile(meshDir, wheelMesh{k}));
    else
        % A cylinder turned onto Y (its own axis is Z), and set inboard of the mount
        % by half its width, which is where each tyre mesh sits.
        addVisual(w, 'Cylinder', [cfg.base.wheelRadius wheelWidth], ...
            trvec2tform([0 -wheelSigns(k,2)*wheelWidth/2 0]) * axang2tform([1 0 0 pi/2]));
    end
    addBody(robot, w, 'chassis_link');
end

% Vendor lidar housing: visual provenance only, and deliberately inert.
% It is carried because the myAGV Pro has one and a render of the vendor CAD
% without it is not the vendor CAD. It is NOT wired to anything: the clearance
% channel in SafetyMonitor is fed by the depth cameras, and adding a body here
% does not create a sensor. Mass is the vendor's; there is no collision, since
% a 0.067 m box on the deck cannot reach anything the chassis box does not.
laser = rigidBody('laser_link');
laser.Joint = rigidBodyJoint('lidar_joint', 'fixed');
setFixedTransform(laser.Joint, trvec2tform([0.17891 0 0.20928]));
laser.Mass = 0.049095;
laser.Inertia = boxInertia(0.049095, [0.0667 0.0665 0.0360]);
if haveMeshes
    addVisual(laser, 'Mesh', fullfile(meshDir, 'laser_link.stl'));
else   % laser_link.stl's bounding box
    addVisual(laser, 'Box', [0.0667 0.0665 0.0360], trvec2tform([-0.00395 0 0.0066]));
end
addBody(robot, laser, 'chassis_link');

% Pedestal raising the arm to a useful working height.
%
% THE FOOT IS REFERENCED TO cfg.base.deckHeight, NOT TO chassisSize(3). chassisSize is a
% BOUNDING box, so its height is the rail crown; the deck floor the pedestal actually
% bolts to is 20 mm lower, in the channel between the rails. Using the box height put the
% underside at the crown and left the mount hanging 20 mm above the deck - geometrically
% wrong and plainly visible in any render. cfg.pedestal.height is derived from the arm
% mount invariant, so the 20 mm is absorbed by a longer pedestal and the UR5e base does
% not move.
%
% The pedestal box therefore overlaps the top 20 mm of the chassis collision box, which
% is correct rather than a collision: the foot is recessed into the channel, and the
% simplified chassis box cannot represent the channel. The pair is skipped anyway, since
% chassis_link is this body's parent and every check here passes
% 'SkippedSelfCollisions','parent'.
ped = rigidBody('pedestal_link');
ped.Joint = rigidBodyJoint('pedestal_mount', 'fixed');
setFixedTransform(ped.Joint, trvec2tform( ...
    cfg.pedestal.offset + [0 0 cfg.base.deckHeight]));
ped.Mass = 3.0;
pedSize = [0.12 0.12 cfg.pedestal.height];
ped.Inertia = boxInertia(3.0, pedSize);
addVisual(ped, 'Box', pedSize, trvec2tform([0 0 cfg.pedestal.height/2]));
addCollision(ped, collisionBox(pedSize(1), pedSize(2), pedSize(3)), ...
    trvec2tform([0 0 cfg.pedestal.height/2]));
addBody(robot, ped, 'chassis_link');

% Mounting plate: the frame the UR5e base attaches to
plate = rigidBody('arm_mount');
plate.Joint = rigidBodyJoint('arm_mount_joint', 'fixed');
setFixedTransform(plate.Joint, ...
    trvec2tform([0 0 cfg.pedestal.height]) * ...
    eul2tform([cfg.pedestal.yaw 0 0], 'ZYX'));
plate.Mass = 0;
plate.Inertia = [1e-6 1e-6 1e-6 0 0 0];
addBody(robot, plate, 'pedestal_link');

%% UR5e via loadrobot, grafted on with addSubtree
ur5e = loadrobot(cfg.arm.robotName, 'DataFormat', cfg.arm.dataFormat, ...
    'Gravity', cfg.frames.gravity);
armBase = ur5e.BaseName;
addSubtree(robot, 'arm_mount', ur5e, 'ReplaceBase', false);

info.armBase = armBase;

% The UR5e tool flange. loadrobot ships both 'flange' and 'tool0';
% tool0 is the ISO-9409-1 flange frame the coupling bolts onto.
toolFlange = 'tool0';
ur5eNames = cellfun(@(b) b.Name, ur5e.Bodies, 'UniformOutput', false);
assert(any(strcmp(toolFlange, ur5eNames)), ...
    'assembleMobileManipulator:noToolFlange', ...
    'UR5e model does not expose a ''%s'' body.', toolFlange);

%% Robotiq AGC-CPL-062-002 coupling: a fixed standoff on the flange
cpl = rigidBody('coupling_link');
cpl.Joint = rigidBodyJoint('coupling_joint', 'fixed');
setFixedTransform(cpl.Joint, eul2tform([cfg.coupling.yaw 0 0], 'ZYX'));
cpl.Mass = 0.15;
cplSize = [0.075 0.075 cfg.coupling.thickness];
cpl.Inertia = boxInertia(0.15, cplSize);
addVisual(cpl, 'Cylinder', [0.0375 cfg.coupling.thickness], ...
    trvec2tform([0 0 cfg.coupling.thickness/2]));
addCollision(cpl, collisionCylinder(0.0375, cfg.coupling.thickness), ...
    trvec2tform([0 0 cfg.coupling.thickness/2]));
addBody(robot, cpl, toolFlange);

% Frame the gripper base bolts to
gmount = rigidBody('gripper_mount');
gmount.Joint = rigidBodyJoint('gripper_mount_joint', 'fixed');
setFixedTransform(gmount.Joint, trvec2tform([0 0 cfg.coupling.thickness]));
gmount.Mass = 0;
gmount.Inertia = [1e-6 1e-6 1e-6 0 0 0];
addBody(robot, gmount, 'coupling_link');

%% Robotiq 2F-85 via addSubtree
grip = loadrobot(cfg.gripper.robotName, 'DataFormat', cfg.arm.dataFormat);
gripperBase = grip.BaseName;
addSubtree(robot, 'gripper_mount', grip, 'ReplaceBase', false);
info.gripperBase = gripperBase;

padBodies = {'left_inner_finger_pad', 'right_inner_finger_pad'};
allNames  = cellfun(@(b) b.Name, robot.Bodies, 'UniformOutput', false);
for k = 1:numel(padBodies)
    assert(any(strcmp(padBodies{k}, allNames)), ...
        'assembleMobileManipulator:noPad', ...
        '2F-85 model does not expose a ''%s'' body.', padBodies{k});
end
info.padBodies = padBodies;

%% Named tool frames - measured from the realised tree, never hard-coded
% The TCP sits on the gripper axis, midway between the two pad frames, so
% its offset is read back from the assembled geometry rather than assumed.
qHome  = homeConfiguration(robot);
Tbase  = getTransform(robot, qHome, gripperBase);
padMid = zeros(3, 1);
for k = 1:2
    Tpad = Tbase \ getTransform(robot, qHome, padBodies{k});
    padMid = padMid + Tpad(1:3, 4)/2;
end

tcp = rigidBody('tcp_frame');
tcp.Joint = rigidBodyJoint('tcp_joint', 'fixed');
setFixedTransform(tcp.Joint, trvec2tform(padMid'));
tcp.Mass = 0;
tcp.Inertia = [1e-6 1e-6 1e-6 0 0 0];
addBody(robot, tcp, gripperBase);

% grasp_frame is the frame the capture gate measures against. It is
% deliberately a separate body from tcp_frame so the gate's geometry can be
% retuned without moving the frame the arm planner tracks.
gr = rigidBody('grasp_frame');
gr.Joint = rigidBodyJoint('grasp_joint', 'fixed');
setFixedTransform(gr.Joint, trvec2tform([0 0 0]));
gr.Mass = 0;
gr.Inertia = [1e-6 1e-6 1e-6 0 0 0];
addBody(robot, gr, 'tcp_frame');
info.toolBody = 'tcp_frame';

% Wrist camera optical frame, aimed AT the grasp point
% Ry(cfg.camera.wrist.tilt) carries the sensor's +X optical axis onto the line from
% this camera's origin to the point the descent ends at, which is NOT the same as onto
% the descent direction itself: the camera is mounted 45 mm off the tool axis, so a
% parallel axis looks 45 mm to the side of the object and pushed it against the top edge
% of the frame. The tilt is -1.3262 rad, 14 deg off the parallel -pi/2. See the block
% comment at that constant for the derivation and the measurement.
wc = rigidBody('wrist_cam');
wc.Joint = rigidBodyJoint('wrist_cam_joint', 'fixed');
setFixedTransform(wc.Joint, ...
    trvec2tform(cfg.camera.wrist.offset) * ...
    eul2tform([0 cfg.camera.wrist.tilt 0], 'ZYX'));
wc.Mass = 0;
wc.Inertia = [1e-6 1e-6 1e-6 0 0 0];

% THE CAMERA BODY, SO THE AIM IS VISIBLE IN THE SIM3D VIEW RATHER THAN ONLY IN A LOG.
% An elongated box on the optical frame, extending along the OPTICAL AXIS (+X in the
% sensor convention), so the render shows both where the camera is and which way it
% looks. That second part is the point: the tilt was wrong by 70 deg through a full
% integration run and nothing on screen said so, because a coordinate frame draws as
% nothing. A barrel that visibly points across the room instead of down at the mug is
% caught by looking once.
%
% AND IT SHOWS THE TREE'S AIM, NOT THE RENDERED CAMERA'S - WHICH IS NOT THE SAME CLAIM.
% The barrel is a body in this tree, so it renders wherever the tree says wrist_cam is.
% The Simulation 3D Camera is a separate actor posed through
% Plant/CameraRig/WristCamera/WristMountPose, and when that chain handed Unreal a
% right-handed Euler triple for a left-handed port the two disagreed by 150 deg: the
% barrel pointed correctly down at the mug while the sensor photographed the sky. So
% this marker catches a wrong MOUNT and is blind to a wrong POSE CHAIN. Checking the
% latter means looking at what the camera returns - the frame itself, or the depth at
% the grasp pixel - which is what hmmWristRefine's tap is for.
%
% IT IS A VISUAL AND ONLY A VISUAL. addVisual does not add collision geometry -
% wrist_cam carries no Collisions - so hmmScreenArmPlan, the IK solves and every
% collision check behave exactly as before. This body is also massless, so the
% dynamics are untouched. The tree renders primitive visuals already (pedestal_link
% is a Box, coupling_link a Cylinder), so no new Simulation 3D Actor block is needed:
% Plant/RobotActor/RobotTree is a Simulation 3D Robot block driven by THIS tree with
% enableVisualVisualization on, and it picks the barrel up for free.
%
% IT SITS BEHIND THE LENS, AND THAT IS NOT COSMETIC. addVisual CENTRES a Box on the
% transform it is given - measured by reading the patch vertices back out of show(),
% which put a 70 mm box placed at x = +0.035 across x = [0, 0.070]. Forward of the
% origin is exactly where the sensor is looking, so a barrel there is a wall 0 to 70 mm
% in front of the lens: the camera photographs the inside of its own housing and the
% depth read at the grasp pixel is the barrel, not the mug. Behind the origin is both
% the only placement that leaves the view clear AND the physically honest one, because
% a real camera's body is behind its lens. The extra 5 mm of clearance keeps the near
% face off the exact optical origin, where a rasteriser's near plane makes the result a
% coin toss.
camBarrel = [0.070 0.024 0.024];    % m, [along the optical axis, width, height]
camLensGap = 0.005;                 % m, near face standoff from the optical origin
addVisual(wc, 'Box', camBarrel, ...
    trvec2tform([-(camBarrel(1)/2 + camLensGap) 0 0]));

addBody(robot, wc, 'gripper_mount');

%% Provenance (PROVENANCE.md obligation)
info.sources.ur5e = ['loadrobot(''' cfg.arm.robotName ''') - Robotics ' ...
    'System Toolbox shipping model, Universal Robots UR5e'];
info.sources.gripper = ['loadrobot(''' cfg.gripper.robotName ''') - ' ...
    'Robotics System Toolbox shipping model, Robotiq 2F-85'];
info.sources.base = ['myAGV Pro vendor CAD - elephantrobotics/agv_pro_ros2, ' ...
    'branch humble, package agv_pro_description 1.0.8, BSD-3-Clause, ' ...
    'Copyright (c) 2024 Elephant Robotics. Six STL meshes, downloaded ' ...
    'unmodified into robots/meshes/agv_pro (chassis, four wheels, lidar); ' ...
    'frames and dimensions measured from those meshes and from ' ...
    'urdf/agv_pro.urdf, which is not included. Collision geometry is a box ' ...
    'fitted to the measured chassis bounding box, not the 89064-triangle ' ...
    'visual mesh. See robots/meshes/agv_pro/PROVENANCE.md.'];
if ~haveMeshes
    info.sources.base = [info.sources.base ' The meshes were not found, so the ' ...
        'base is drawn as boxes and cylinders of the same size.'];
end
info.sources.coupling = ['Parametric standoff for Robotiq ' ...
    'AGC-CPL-062-002 (thickness from hmmConfig.coupling).'];

%% Measured gripper coupling, folded back into the tree
% The shipped 2F-85 carries PositionLimits that assume every joint moves in
% the positive sense. Five of the six are mimic joints, and two of those
% must run negative to keep the jaws parallel, so their shipped limits
% exclude the range they actually travel. Left alone, inverseKinematics and
% randomConfiguration would treat the real motion as a limit violation.
% Replace them with the limits the MEASURED coupling implies.
idxLocal = mobileManipulatorIndexMap(robot);
coupling = measureGripperCoupling(robot, idxLocal, cfg);
movable  = robot.Bodies(cellfun(@(b) ~strcmp(b.Joint.Type,'fixed'), robot.Bodies));
for k = 1:numel(coupling.indices)
    movable{coupling.indices(k)}.Joint.PositionLimits = coupling.limits(k, :);
end
info.gripperCoupling = coupling;
info.apertureMap = measureGripperApertureMap(robot, coupling, cfg);

%% Realised topology
info.jointNames = cellfun(@(b) b.Joint.Name, robot.Bodies, ...
    'UniformOutput', false);
info.bodyNames  = cellfun(@(b) b.Name, robot.Bodies, ...
    'UniformOutput', false);
info.nonFixedJoints = info.jointNames(cellfun(@(b) ...
    ~strcmp(b.Joint.Type, 'fixed'), robot.Bodies));
info.numDOF = numel(info.nonFixedJoints);

assert(info.numDOF == 15, 'assembleMobileManipulator:dofMismatch', ...
    'Expected a 15-DOF tree, assembled %d DOF: %s.', ...
    info.numDOF, strjoin(info.nonFixedJoints, ', '));

end

%% ------------------------------------------------------------------------
function I = boxInertia(m, sz)
%BOXINERTIA Inertia vector [Ixx Iyy Izz Iyz Ixz Ixy] for a solid box.
I = [m*(sz(2)^2 + sz(3)^2)/12, ...
     m*(sz(1)^2 + sz(3)^2)/12, ...
     m*(sz(1)^2 + sz(2)^2)/12, 0, 0, 0];
end

function I = cylinderInertia(m, r, h)
%CYLINDERINERTIA Inertia vector for a solid cylinder about its own axes.
Iaxial = m*r^2/2;
Iradial = m*(3*r^2 + h^2)/12;
I = [Iradial, Iradial, Iaxial, 0, 0, 0];
end
