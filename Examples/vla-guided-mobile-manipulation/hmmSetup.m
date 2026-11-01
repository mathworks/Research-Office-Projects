function data = hmmSetup(force)
%HMMSETUP Populate the base workspace for homeMobileManipulator.
%
%   hmmSetup() adds the project folders to the path, builds the configuration
%   and the one authoritative 15-DOF rigidBodyTree, measures the gripper
%   coupling and aperture map, defines the Simulink.Bus objects, and assigns
%   everything the model's block parameters reference to the base workspace.
%
%   This is wired to the model's PostLoadFcn because it is not optional:
%   Simulation 3D Robot resolves its rigidBodyTree by VARIABLE NAME at compile
%   time, and every lookup table in GripperControl reads breakpoints that are
%   measured from the assembled tree rather than written down. A freshly
%   started MATLAB cannot compile the model without this.
%
%   hmmSetup(true) rebuilds even when the base workspace already holds a
%   consistent set. The default reuses what is there, because assembling the
%   tree runs three measurement sweeps and there is no reason to repeat them
%   on every model load.
%
%   data is a struct of everything assigned, so a script can use the values
%   without reaching into the base workspace.
%
%   Base workspace variables created:
%       cfg               configuration struct (hmmConfig)
%       robot             the one 15-DOF rigidBodyTree
%       robotInfo         provenance and realised geometry
%       idx               joint index map, asserted against the tree
%       apertureMap       measured aperture <-> driver-angle lookup data
%       gripperCoupling   measured 2F-85 mimic multipliers and limits
%       armJointLimits    6x2 UR5e position limits read from the tree
%       armIkTree         the arm-and-gripper SUBTREE the IK blocks solve on
%       armDynTree        the 6-DOF ARM-ONLY subtree the Joint Space Motion
%                         Model uses
%       armTaskTree       the 6-DOF subtree the Task Space Motion Model uses:
%                         armIkTree with the finger joints frozen, so it keeps
%                         tcp_frame and the gripper's mass but drops the six
%                         actuated joints that made the model unintegrable
%       rays              ray grid for the calibrated depth path
%       invalidRayHits    all-invalid RayHits value for the wrist camera
%       navMapGrid        initial occupancy grid for the navigation Data Store
%       movableRegistry   runtime registry of movable actors (empty when none)
%       actorNameTable    uint8 rows naming those actors, one per ActorId, which
%                         is how the plant addresses Simulation 3D Robot's
%                         TargetActor input - it takes a name, not an id
%       scan*             constants for the block-only range scan cast from the
%                         occupancy grid, which feeds the Vector Field Histogram
%       robotStateInit    the plant's TRUE initial state, not a zeroed ground
%                         value: all-zero joints are a pose that trips the reach
%                         check and latches the e-stop on the first step
%       *Init             zeroed ground values for the service buses, used as
%                         Unit Delay initial conditions and as type templates
%       taskRequestStub   cfg.task.instruction as a TaskRequest value
%       carryTranslation  tcp_frame to held-object centre, m, for Simulation 3D
%                         Robot's end-effector-to-target field - the block
%                         TELEPORTS the prop to this offset on grasp rather than
%                         keeping the pose the jaws closed on, so a wrong value
%                         here is a released object floating above its surface
%       sunAzimuth        scene sun azimuth, deg (bare scalar for the Sim3D mask)
%       sunAltitude       scene sun altitude, deg (bare scalar for the Sim3D mask)
%       the Simulink.Bus objects listed in hmmDefineBuses
%
%   See also hmmConfig, assembleMobileManipulator, hmmDefineBuses.

%   Copyright 2026 The MathWorks, Inc.

arguments
    force (1,1) logical = false
end

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'configuration'), ...
        fullfile(root, 'robots'), ...
        fullfile(root, 'scripts'), ...
        fullfile(root, 'functions'));

if ~force && isReady()
    data = collectFromBase();
    hmmRuntimeConfig(data.cfg);
    return
end

% EVERY CACHE IN THE PROJECT IS DROPPED FIRST, BECAUSE force MEANS "cfg HAS CHANGED".
% Several functions under functions/ hold a persistent copy of cfg, or of a rigidBodyTree
% derived from it, so that a MATLAB Function block running every tick does not rebuild it
% every tick. Nothing invalidated them, and that made an edit to hmmConfig invisible to
% precisely the code that consumes it: hmmScreenArmPlan went on time-parameterising against
% the cfg it first saw, hmmArmCollisionModel went on handing out a plan tree with the joint
% limits it first read, and a run taken to prove a fix could reproduce the bug the fix had
% already removed. The folder is WALKED rather than a list written down here, because a list
% is a thing to forget to add the seventh cached function to.
clearProjectCaches(fullfile(root, 'functions'));

cfg = hmmConfig();
assertConfigInvariants(cfg);

[robot, robotInfo] = assembleMobileManipulator(cfg);
idx = mobileManipulatorIndexMap(robot);

% Arm position limits read back from the tree, so the plant saturates at the
% real UR5e travel rather than at a number copied into a block.
movable = robot.Bodies(cellfun(@(b) ~strcmp(b.Joint.Type, 'fixed'), robot.Bodies));
armJointLimits = zeros(numel(idx.arm), 2);
for k = 1:numel(idx.arm)
    armJointLimits(k, :) = movable{idx.arm(k)}.Joint.PositionLimits;
end

data.cfg             = cfg;
data.robot           = robot;
% RENDER ONLY, AND IT MUST REACH EXACTLY ONE PARAMETER. Simulation 3D Robot negates y
% on every link of the tree it is given - a right- to left-handed conversion - while
% every Simulation 3D Actor and Camera in this model takes its Translation as given, so
% the scene is already in MATLAB coordinates and the block's conversion is one frame
% change too many. This mirrored copy cancels it, and it is the ONLY thing that may:
% its joint angles do not mean what they say and its frames are left-handed, so control,
% planning, perception, the safety workspace check and every FK measurement keep using
% data.robot above. See hmmMirrorTree for the full account, and hmmConfig's retired
% cfg.render.baseRenderFlip for the base-only patch this replaces.
data.renderRobot     = hmmMirrorTree(robot, hmmRobotPaint(cfg));
data.robotInfo       = robotInfo;
data.idx             = idx;
data.apertureMap     = robotInfo.apertureMap;
data.gripperCoupling = robotInfo.gripperCoupling;
data.armJointLimits  = armJointLimits;

% The Inverse Kinematics and Get Jacobian blocks solve on a SUBTREE of the one
% authoritative tree, rooted at the arm mounting plate. This is a DERIVATION,
% not a second robot model: change assembleMobileManipulator and this changes
% with it, so the two cannot drift. It exists because the design excludes
% whole-body optimisation - solving on the full 15-DOF tree would let IK move
% the base to reach, which is a different (and unspecified) problem. The
% subtree keeps tcp_frame and grasp_frame, and its six gripper joints have a
% structurally zero Jacobian for tcp_frame, so the solver leaves them alone.
data.armIkTree = subtree(robot, robotInfo.armBase);
data.armIkTree.DataFormat = cfg.arm.dataFormat;

% The Joint Space and Task Space Motion Model blocks need a tree whose DOF count
% matches the six joints ArmControl commands, so they get a further derivation
% with the gripper branch removed. rigidBodyTree is a HANDLE class and removeBody
% mutates in place, so this MUST copy first: calling removeBody on data.armIkTree
% would silently turn the 12-DOF IK tree into a 6-DOF one and every Inverse
% Kinematics block would then solve a different problem than the one it reports.
%
% removeBody returns the branch it took out and mutates the tree it was given, so
% the 6-DOF arm is the COPY, not the return value.
data.armDynTree = copy(data.armIkTree);
removeBody(data.armDynTree, robotInfo.gripperBase);
data.armDynTree.DataFormat = 'column';
nDyn = numel(homeConfiguration(data.armDynTree));
if nDyn ~= cfg.arm.nJoints
    error('hmm:setup:armDynTreeDof', ...
        ['The arm-only tree came out with %d DOF but ArmControl commands %d ' ...
         'joints. The motion model blocks size their ports from the tree, so ' ...
         'this would fail as a port-width error far from its cause. Check ' ...
         'that robotInfo.gripperBase (%s) is still the root of the gripper ' ...
         'branch in assembleMobileManipulator.'], ...
        nDyn, cfg.arm.nJoints, robotInfo.gripperBase);
end

% THE TASK SPACE TRACKER NEEDS A THIRD DERIVATION, AND IT IS NOT A CONVENIENCE.
% Neither tree above will do. armIkTree has tcp_frame but twelve actuated joints;
% armDynTree has six but no tcp_frame, because removeBody took the whole gripper
% branch and tcp_frame hangs off it. Task Space Motion Model needs BOTH: it
% tracks a pose at tcp_frame, and it integrates rigid body dynamics over exactly
% the joints ArmControl commands.
%
% So the finger joints are FROZEN rather than removed. Freezing keeps every gram
% where it is - the gripper is 0.38 kg hanging off wrist_3 and it belongs in the
% arm's dynamics - and leaves tcp_frame exactly where the IK put it, while the
% actuated count drops from 12 to 6. It is also the honest model: during a
% task-space move the gripper holds still, so from the arm's point of view it IS
% a rigid payload.
%
% THE MEASUREMENT THAT FORCED THIS. Pointed at armIkTree, the block's fastest
% eigenvalue is 7.1e5 1/s - a 1.4 microsecond time constant - because the six
% finger joints have inertias of 7e-6 to 2e-4 kg m^2 and the tcp_frame Jacobian's
% columns for them are structurally zero, so the task-space law never drives
% them and JointDamping alone divided by a near-zero inertia is all that is left.
% The whole model then cannot be integrated: ode45 collapses to a 5 microsecond
% step and does not reach t = 0.012 in 300 seconds, and any fixed step diverges
% to a non-finite derivative within two steps.
data.armTaskTree = copy(data.armIkTree);
frozen = freezeGripperJoints(data.armTaskTree, cfg);
nTask = numel(homeConfiguration(data.armTaskTree));
hasTool = any(strcmp(data.armTaskTree.BodyNames, robotInfo.toolBody));
if nTask ~= cfg.arm.nJoints || ~hasTool
    error('hmm:setup:armTaskTreeDof', ...
        ['The task-space tree came out with %d DOF and %s present = %d; it must ' ...
         'have %d and 1. Froze: %s.'], ...
        nTask, robotInfo.toolBody, hasTool, cfg.arm.nJoints, strjoin(frozen, ', '));
end

% THE IK TREE IS HELD OFF THE POSITION BOX, AND ONLY IT. armIkTree is what the three
% Inverse Kinematics blocks enforce their joint limits against, so narrowing it by
% cfg.arm.limitMargin is what keeps a grasp candidate from being solved onto the wall -
% where ArmControl/JointLimit's Saturate would clamp the command and hand SafetyMonitor a
% corner to read as a 8.55 rad/s^2 acceleration step. hmmArmCollisionModel narrows the
% subtree manipulatorRRT samples for the same reason.
%
% THE ORDER OF THESE THREE DERIVATIONS IS THEREFORE LOAD BEARING. armDynTree and
% armTaskTree were copied above, so they keep the REAL travel: they are the dynamics
% models the Joint Space and Task Space Motion Model blocks integrate, and a servo model
% that cannot reach a pose the robot can reach is not a better servo model. armJointLimits
% was read off the tree earlier still, which is what leaves JointLimit saturating at the
% travel the UR5e actually has.
data.armPlanLimits = hmmShrinkArmLimits(data.armIkTree, cfg);

% cfg.derived MARKS A cfg THAT CAME THROUGH THIS FUNCTION. isReady needs to tell a derived cfg
% from one some other code published straight from hmmConfig, and it used to do that by the shape
% of the task-space gains this function sized (6x6 here, scalars in hmmConfig). Those gains fed a
% Task Space Motion Model that was removed from ArmControl long ago - nothing in the model read
% them - so the derivation went on 2026-10-08 and an explicit marker replaces it.
cfg.derived = true;

data.cfg = cfg;

% The stowed envelope is re-measured from the tree that was just assembled and
% checked against the number the planner and the mapper both use. This is a
% CHECK rather than an override because both of those run inside MATLAB Function
% blocks that read hmmConfig directly and cannot be handed a rigidBodyTree, so
% cfg.plan.stowedArmRadius has to stay the one place the value is stated. It is
% checked at all because understating it is silent and catastrophic: the fixed
% cameras map the robot's own gripper, and after inflation the base cannot prove
% its own pose valid, so every base plan fails at the start-state check.
robotInfo.stowedEnvelopeRadius = measureStowedEnvelope( ...
    robot, idx, robotInfo.gripperCoupling, cfg);
stated = cfg.plan.baseRadius + cfg.plan.stowedArmRadius;
if robotInfo.stowedEnvelopeRadius > stated
    warning('hmm:setup:stowedEnvelopeTooSmall', ...
        ['The assembled robot reaches %.3f m from the base origin when ' ...
         'stowed, but cfg.plan.baseRadius + cfg.plan.stowedArmRadius is ' ...
         'only %.3f m. Set cfg.plan.stowedArmRadius to at least %.3f in ' ...
         'hmmConfig, or the mapper will treat the robot''s own gripper as ' ...
         'an obstacle and every base plan will fail.'], ...
        robotInfo.stowedEnvelopeRadius, stated, ...
        ceil(100*(robotInfo.stowedEnvelopeRadius - cfg.plan.baseRadius))/100);
end

% AND THE PER-AXIS VERSION OF THE SAME NUMBER, which is what the base planner's footprint
% is built from. cfg.plan.stowedArmHalfExtent is measured off the stowed tree and it
% claims that every body of the stowed arm lies inside that rectangle; the measurement above
% found a body at stowedEnvelopeRadius from the origin. A rectangle that contains that body
% must have a half-diagonal at least as long, so hypot of the pair is a necessary condition
% and it is free. It is not sufficient - it cannot catch a pair that is too small on one axis
% and too large on the other - so the real check is re-measuring the pair after any change
% to cfg.arm.stowConfig or the assembled tree.
%
% THE TEXT IS BUILT WITH SPRINTF AND PASSED AS A SINGLE %s, which is not a style choice.
% warning's own formatted arguments must be SCALAR - unlike sprintf and fprintf, it rejects
% a 1x2 like cfg.plan.stowedArmHalfExtent with "Formatted arguments must be scalar" - so a
% warning written the obvious way THROWS instead of warning, and throws only on the day the
% condition first becomes true. That is exactly what happened to the carryOffset check below
% when the stow pose went vertical: the check had been correct and untested since it was
% written, and the first time it had something to say it took hmmSetup down with it. Both
% checks here take vectors, so both build their text first.
halfDiag = hypot(cfg.plan.stowedArmHalfExtent(1), cfg.plan.stowedArmHalfExtent(2));
if halfDiag < robotInfo.stowedEnvelopeRadius
    msg = sprintf( ...
        ['cfg.plan.stowedArmHalfExtent = [%.4f %.4f] has a half-diagonal of %.4f m, ' ...
         'which cannot contain the body measured %.4f m from the base origin at the ' ...
         'stow pose. The base planner builds its footprint rectangle from that pair, so ' ...
         'it is currently modelling a robot smaller than the one assembled. Set the ' ...
         'pair in hmmConfig to the stowed arm''s half-extents along the base x and y axes.'], ...
        cfg.plan.stowedArmHalfExtent, halfDiag, robotInfo.stowedEnvelopeRadius);
    warning('hmm:setup:stowedHalfExtentTooSmall', '%s', msg);
end

% AND THE SAME QUESTION FOR THE THING IN THE JAWS, WHICH THIS ONE CAN ANSWER EXACTLY.
% The check above is only a necessary condition, because a rectangle's two half-extents
% cannot be recovered from one radius. This one is sufficient: cfg.plan.carryOffset bounds
% where a grasped object's CENTRE sits in the base frame, the object is rigidly held within
% cfg.plan.graspDepth of the tool along the tool's own z, and the tool's position at the
% stow pose is a single forward-kinematics call on the tree just assembled. Adding graspDepth
% to each axis rather than projecting it is what makes the bound hold whichever way the wrist
% has turned, so this check needs no orientation convention and cannot disagree with one.
%
% THAT SLACK IS A REAL DISPLACEMENT AGAIN, WHICH IS WHY THE CHECK IS HERE AND NOT DECORATIVE.
% The pick target is ObjectTarget.Position + hmmGraspAim's rise, so the held centre hangs
% below the tool by rise - Extent(3)/2, which is the PAD DROP ALONE wherever the aim is an
% honest mid-height - 7.6 mm for the shipped mug, and under 10 mm for every prop that survives
% the GraspExtent gate in hmmReconstructTargets, because the pad-drop table spans only the jaw
% apertures the pads were characterised at. It grows above that only for an object shorter than
% 2*(padFaceLength/2 + padGroundClear) = 57.5 mm, where hmmGraspAim's floor lifts the aim off
% the mid-height to keep the pads out of the table; the worst case over the graspable set is
% therefore a SHORT object rather than a tall one, which is the opposite of what the fixed
% fraction this replaced did. The bound belongs to the worst graspable prop and not to the one
% the shipped instruction names, which is the trap the first re-size fell into: it covered the
% mug at 0.030 m and left the then-worst prop over by 9.9 mm. The bound has been wrong in both
% directions across this example's history: a real 20 mm under the original top-face contract,
% where 20 mm was too small; exactly zero under a naive mid-height contract that ignored the
% pad drop, where 20 mm was pure slack; 21.1 mm when the grasp rose to a fixed 0.65 of the
% height, where 20 mm was understated; 39.9 mm once the graspable set was swept, where 30 mm
% still was; and 7.6 mm now that the aim is a clamped mid-height again. This check is what
% turns that from a silent footprint error into a warning at setup - but note it can only check
% the bound against the TREE, never against the scene, so it would not have caught the 0.030 m
% either. Only the sweep in hmmConfig's sizing note does that.
%
% IT IS CHECKED BECAUSE UNDERSTATING IT IS SILENT AND IT LETS THE ROBOT DRIVE THROUGH THE
% OBJECT IT IS CARRYING. Overstating it is what shipped, and that was neither silent nor
% safe in the way it looked: hmmBaseFootprint used to add the object's half-extent to all
% four faces, which took the inflated footprint to 0.3389 m, put the pick stand-off's whole
% arrival ball outside the map while holding, and killed the place leg with
% BaseStartOccupied. Both directions therefore have a check now - this one for too small,
% the stand-off checks below for too large.
%
% cfg.arm.stowConfig is the pose used because it is the pose the base drives in, which is
% the assumption cfg.plan.stowedArmHalfExtent already rests on. armTaskTree is the arm-only
% tree, so its origin is the arm mount and cfg.pedestal.offset carries it to the base origin;
% hmmArmCollisionModel's planTree returns the identical figure from the full assembly.
toolAtStow  = getTransform(data.armTaskTree, cfg.arm.stowConfig, robotInfo.toolBody);
toolInBase  = abs(toolAtStow(1:2, 4) + cfg.pedestal.offset(1:2)');
carryBound  = toolInBase + cfg.plan.graspDepth;
robotInfo.carryBound = carryBound';
if any(cfg.plan.carryOffset(:) < carryBound)
    msg = sprintf( ...
        ['cfg.plan.carryOffset = [%.4f %.4f] is under the [%.4f %.4f] the assembled ' ...
         'tree implies: the tool sits [%.4f %.4f] from the base origin at ' ...
         'cfg.arm.stowConfig, and a grasped object''s centre is within the %.3f m of ' ...
         'cfg.plan.graspDepth of it whichever way the wrist is turned. ' ...
         'hmmBaseFootprint builds the base''s footprint rectangle from that pair, so ' ...
         'the planner is modelling a robot that does not contain the object it is ' ...
         'holding and may drive that object through a wall the map knows about. ' ...
         'Raise cfg.plan.carryOffset in hmmConfig to at least that bound.'], ...
        cfg.plan.carryOffset, carryBound, toolInBase, cfg.plan.graspDepth);
    warning('hmm:setup:carryOffsetTooSmall', '%s', msg);
end

data.robotInfo = robotInfo;

% THE NAVIGATION MAP THE BASE PLANS OVER, and the ONLY place it enters the model:
% the NavMapGrid Data Store Memory's InitialValue is the variable name
% 'navMapGrid', so whatever is put here is what the run starts from. Loading the
% map through a setup variable rather than a block dialog is why shipping a
% captured map needs no model edit at all.
%
% Under cfg.map.mode 'fixed' this is the captured grid from data/hmmNavMap.mat;
% under 'live' it is the uniform prior and SceneMapping fills it in. hmmLoadNavMap
% decides between them and validates the file's provenance against the current
% cfg, because a grid is a bare matrix of probabilities that records neither the
% resolution nor the world origin it was measured against.
%
% EITHER WAY THE MAP STARTS TRAVERSABLE WHERE IT IS UNKNOWN. An all-unknown grid
% would make every base plan fail before the first frame arrives - a startup
% artefact, not a real obstacle - while an all-zero grid could never be updated at
% all, because occupancyMap works in log-odds and zero is minus infinity. See
% cfg.map.initialProbability for the arithmetic.
data.navMapGrid = hmmLoadNavMap(cfg);

% Movable actors are DISCOVERED, never configured. When no prop metadata is
% found the registry is empty and perception reports AmbiguousObject, which
% is the honest answer.
data.movableRegistry = hmmDiscoverMovables(cfg);

% THE NAMES THOSE ACTORS ANSWER TO, which the plant needs and perception must not
% have. Simulation 3D Robot's TargetActor input is addressed by name, so the
% capture gate turns a registry ActorId into a fixed-width uint8 name by selecting
% a row of this table. It is a PLANT-SIDE lookup and that distinction is the whole
% justification for it existing: association stays geometric and nothing in
% perception may look a prop up by name, but the renderer's grasp API has no id
% form, so the plant has to know what the renderer calls the actor it just decided
% to hold. hmmSpawnProps names the actors from the same function, so row k of this
% table and actor k in the world are the same string by construction.
[~, data.actorNameTable] = hmmActorNames(cfg);

% The ray grid and the wrist camera's invalid RayHits value are precomputed
% here rather than written as function calls into block dialogs. Simulink
% evaluates a dialog expression on every compile, and both of these build
% 9216-row arrays - prod(cfg.depth.rayGrid), and 2304 before the grid went to
% [72 128]; naming them once also means a reader of the Ray Tracer
% dialog sees which variable holds the rays instead of a nested expression.
data.rays            = hmmRayDirections(cfg);
data.invalidRayHits  = hmmInvalidRayHits(cfg);

% Sun angles as bare scalars. The Simulation 3D Scene Configuration mask rejects
% a struct field reference for SunAzimuth and SunAltitude - "Input must be a
% valid number or a variable name" - so these two, alone among the model's
% parameters, cannot read cfg directly. They are derived here rather than typed
% into the dialog so cfg.render remains the only place a value is decided.
data.sunAzimuth      = cfg.render.sunAzimuth;
data.sunAltitude     = cfg.render.sunAltitude;

% The bus objects must exist before the ground values below can be built, and
% before any block parameter resolves a bus name.
data.buses = hmmDefineBuses(cfg);

% GROUND VALUES FOR THE SERVICE BUSES. Every planning and perception service is
% conditionally executed, so each one's output is fed back through a Unit Delay
% that has to be given an initial value of exactly the right bus type. Building
% those values with Simulink.Bus.createMATLABStruct means the initial condition
% is DERIVED from hmmDefineBuses rather than written out a second time: add an
% element there and the initial condition grows with it. Every field comes back
% zero, which is the honest starting state - Valid false, Timestamp 0, and a
% FailureCode of OK that nothing may read while Valid is false.
%
% They double as the type/size TEMPLATE each service function is handed. A
% MATLAB Function block cannot declare a bus output type in code without
% deciding element types there, i.e. in a second place competing with
% hmmDefineBuses; taking the previous bus as an input and overwriting its fields
% keeps this file and that one the only two that know the shape.
for name = ["ActionQueue", "ActionItem", "ObjectTarget", "PlaceTarget", ...
            "BasePlan", "ArmPlan", "GraspCandidate", "PerceptionStatus", ...
            "BaseCommand", "BaseStatus", "ArmCommand", "ArmStatus", ...
            "GripperCommand", "GripperStatus", "SafetyStatus", "TaskResult", ...
            "RobotState"]
    field = char(lower(extractBefore(name, 2)) + extractAfter(name, 1) + "Init");
    data.(field) = Simulink.Bus.createMATLABStruct(char(name));
end

% The one field a zeroed ground value gets WRONG. RefTime is the time-point vector of
% the followers' Polynomial Trajectory blocks, so it has to be strictly increasing over
% its whole length, and createMATLABStruct hands back all zeros. A Unit Delay
% initialised with that value would, on the first step of every run, feed the follower
% a degenerate time-point vector. The same repair the planners apply is applied here, so the initial condition obeys
% the same contract as every plan that replaces it. See hmmPadRefTime.
data.basePlanInit.RefTime = hmmPadRefTime(data.basePlanInit.RefTime, 0);
data.armPlanInit.RefTime  = hmmPadRefTime(data.armPlanInit.RefTime, 0);

% Ground values that are not zero, because zero is a meaningful code for them.
% ArmPlan's identity grasp pose keeps the Task Space Motion Model from being
% handed an all-zero homogeneous transform, which is not a valid pose at all.
data.armPlanInit.GraspPose = eye(4);
data.armPlanInit.ApertureCmd = cfg.gripper.maxOpenPos;
data.armPlanInit.AttachActorId = cfg.actor.noneId;

% A cleared e-stop is the ONLY safe ground value here: SafetyStatus feeds all
% four charts through a Unit Delay, so a latched initial value would put the
% whole machine in its terminal stopped mode on the first step of every run and
% the task would never start. The latch itself is what makes this safe to do -
% the monitor re-evaluates every limit at the control rate, so a genuine
% violation present at t = 0 latches on the very first step anyway.
data.safetyStatusInit.EStop = false;
data.safetyStatusInit.Cause = cfg.sup.cause.NONE;

% ROBOTSTATE IS THE ONE BUS WHOSE ZEROED GROUND VALUE IS NOT A STATE THE ROBOT
% CAN BE IN, AND SHIPPING IT ZEROED LATCHED THE E-STOP ON EVERY RUN.
% RobotState is fed back through a Unit Delay to break the plant-to-supervisor
% algebraic loop, so on the first step every consumer sees the initial condition
% rather than a measurement. For a Status bus all-zeros is honest - it reads as
% IDLE with Valid false. For RobotState it is a lie with consequences: all-zero
% ArmJointPos is the UR5e's fully outstretched pose, whose tool sits 1.11 m from
% arm_mount against what was then a monitor bound of 0.85, so SafetyMonitor's reach
% check fired on sample 0, the e-stop latched, and every chart spent the whole run in
% its terminal stopped mode. Nothing moved and no other check ever reported anything,
% which is exactly how it looked: one violation at t = 0 and silence afterwards.
%
% That 1.11 m against 0.85 was ALSO the first sighting of a second, separate defect, and
% it was missed here: the bound itself was wrong. It was cfg.arm.reach, a horizontal
% radius to the FLANGE, used to bound the 3-D distance to a TOOL POINT 0.1416 m further
% out. Fixing the initial condition stopped the symptom and left the mis-sized constant
% in place, where it went on to reject a healthy arm plan for a 1.6% overshoot. The
% monitor now compares against cfg.safety.toolReachMax, which is derived from the tree.
% A number that surprises you is worth chasing even once the run goes green.
%
% So the initial condition is the state the plant genuinely starts in - the same
% cfg.base.initialPose and cfg.arm.stowConfig the plant's own integrators are
% initialised from - and the tool pose is MEASURED off the assembled tree rather
% than written down, so it cannot drift away from the plant's kinematics. Valid is
% true, and deliberately: this is not a placeholder standing in for a missing
% measurement, it is where the robot is before the first one is taken.
q0 = zeros(1, idx.numDOF);
q0(idx.base) = cfg.base.initialPose(:)';
q0(idx.arm)  = cfg.arm.stowConfig(:)';
toolPose = getTransform(robot, q0, robotInfo.toolBody);

data.robotStateInit.BasePose = cfg.base.initialPose(:);
data.robotStateInit.ArmJointPos = cfg.arm.stowConfig(:);
data.robotStateInit.GripperAperture = data.apertureMap.apertureAtOpen;
data.robotStateInit.ToolTranslation = toolPose(1:3, 4);
data.robotStateInit.ToolRotation = rotm2eul(toolPose(1:3, 1:3), cfg.frames.eulerOrder)';
data.robotStateInit.AttachedActorId = cfg.actor.noneId;
data.robotStateInit.Valid = true;

% The operator instruction as a TaskRequest value. Text cannot live on a bus as a
% string, so cfg.task.instruction is encoded to the fixed-capacity uint8 array
% the bus declares and the length is carried alongside. Encoding it here rather
% than in a Constant block dialog is what keeps the truncation VISIBLE: if the
% instruction ever exceeds cfg.provider.instructionChars, InstructionLen equals
% the capacity and the consumer can see that it saturated.
data.taskRequestStub = buildTaskRequest(cfg);

% WHERE THE HELD OBJECT HANGS, WHICH IS THE SECOND THING THE INSTRUCTION DECIDES BEFORE
% THE RUN STARTS. Simulation 3D Robot does not lift the prop from where the jaws closed
% on it; TargetGraspUtils/attachTargetActor parents the prop to the end-effector and then
% overwrites its pose with a mask parameter. It is resolved here from the only
% task-specific datum available at compile time - the sentence itself.
% hmmCarryTranslation explains the resolution and why the offset cannot be one constant:
% it grows with object height, from 21.1 mm for the mug to 39.9 mm for the Bottle.
% Published unconditionally so that RobotTree's mask resolves on a bare load.
%
% THIS IS A DEFAULT AND NO LONGER A COMMITMENT, which is a correction to what used to be
% written here. The claim was that the mask parameter has no input port and therefore had
% to be fixed before the first frame. The block's TargetEndEffectorTranslation is in fact
% tunable - not Nontunable, and re-read by stepImpl every step - so the app retunes it
% mid-run from the sentence the operator types. An unattended caller that never types
% anything still gets a correct offset for cfg.task.instruction, which is what this line
% is for.
[data.carryTranslation, carryOff, carryName] = hmmCarryTranslation(cfg);
if ~isempty(carryName)
    fprintf('hmmSetup: carried object %s, held %.1f mm below tcp_frame', ...
        carryName, carryOff * 1000);
    % The lateral term is reported ONLY when it is non-zero, because a prop with no
    % measured pivot offset genuinely has nothing to say here and a printed "0.0 mm"
    % would imply the offset had been measured and found to be zero.
    if any(abs(data.carryTranslation(1:2)) > 1e-6)
        fprintf(', offset [%.1f %.1f] mm laterally for its mesh pivot', ...
            data.carryTranslation(1:2) * 1000);
    end
    fprintf('.\n');
end

% THE SHIPPED SENTENCE IS SEEDED AS SUBMISSION 1, AND THIS IS WHAT KEEPS EVERY
% UNATTENDED CALLER WORKING. TaskRequestSource is no longer a Constant holding
% taskRequestStub; it is a MATLAB Function block reading whatever hmmSubmitTask
% last submitted, so that a sentence can be typed while the model runs. A script that
% simply calls sim() on this model expects the configured task, and without a seed
% it would quietly watch a stationary robot - no error, no diagnostic, just a run
% that does nothing. Seeding here makes the
% default path byte-identical to the Constant it replaced, TaskId and all, and
% leaves interactive use to opt OUT with hmmSubmitTask('clear').
%
% SAFE TO DO ON EVERY hmmSetup BECAUSE hmmSetup IS NOT A START-OF-RUN HOOK. The
% model's PostLoadFcn calls it, which fires on load and not on each simulation, so
% seeding cannot overwrite a sentence the operator typed after the model opened.
% Were it ever moved to InitFcn, that would silently disable interactive tasking.
%
% taskRequestStub is still published: it records the shipped instruction as a
% resolved bus value, and it is the type template a caller can copy. It is no
% longer wired to anything.
%
% BUT NOT WHILE A RUN IS IN PROGRESS. PostLoadFcn is not the only caller: setupParams
% and other scripts run hmmSetup too, and one run mid-simulation would submit the shipped
% sentence as a NEW task. The planner ignores a new task while it operates, so nothing
% looks wrong - but a cancel pressed afterwards carries the new task's id, matches no
% task in flight and is dropped without a word, and the seeded task starts the moment
% the current one ends (found 2026-10-19, chasing a cancel that did nothing).
hmmRuntimeConfig(data.cfg);
bd = find_system('type', 'block_diagram');
if all(strcmp(get_param(bd, 'SimulationStatus'), 'stopped'))
    hmmSubmitTask(cfg.task.instruction);
end

names = fieldnames(data);
names = names(~strcmp(names, 'buses'));
for k = 1:numel(names)
    assignin('base', names{k}, data.(names{k}));
end

% NOTHING FURTHER IS NEEDED TO MAKE THE MASKED LIBRARY BLOCKS RESOLVE, AND THAT
% IS WORTH RECORDING BECAUSE AN EARLIER VERSION OF THIS FILE THOUGHT OTHERWISE.
% The Robotics System Toolbox blocks - Inverse Kinematics, Get Jacobian, Get
% Transform, the two Motion Models, Simulation 3D Robot - are masked subsystems
% whose inner MATLAB System block entries are the NAMES of mask-workspace
% variables (EndEffector, SourceBody, TargetBody, AlgorithmName, ...). Simulink
% evaluates those names in the mask workspace at compile time, and it does so
% correctly here.
%
% The symptoms that suggested otherwise - "Unable to find body with name
% SourceBody", an unresolved IKInfoBusName - are an ORDERING problem, and this
% function is the fix for it. The masks' initialization code reads cfg and the
% rigidBodyTrees from the base workspace, so on a bare load it cannot run and the
% mask workspace is empty; the inner names then have nothing to resolve against.
% Because this runs as the model's PostLoadFcn, the variables are in place before
% any compile, the masks initialize, and the names resolve.
%
% An earlier version of this setup pushed the resolved VALUES down into the
% inner blocks instead. That is now deleted: it turned every one of those
% parameters into an undefined variable (Get Transform asking for a variable
% called map, Task Space Motion Model asking for one called tcp_frame) and it
% failed the simulation outright. Measured both ways - cold load, 31 parameters
% restored to their shipped variable names, nothing compiled beforehand: the
% model reaches its stop time with zero warnings.

end

%% ------------------------------------------------------------------------
function assertConfigInvariants(cfg)
%ASSERTCONFIGINVARIANTS Relationships between configured values, checked at load.
%   These are not range checks on individual numbers - hmmConfig owns those. Each
%   one here is a relationship BETWEEN two values that the model relies on and
%   that nothing in the model can detect if it breaks. All of them fail silently:
%   the simulation runs and produces plausible numbers that mean something else.
%
%   They run before the tree is assembled, so a bad configuration is reported in
%   one line instead of after three measurement sweeps.

% 1  Every supervisory timeout must exceed the measured latency of what it is
%    waiting for, or the state machine gives up on a service that was about to
%    answer and burns its retry budget doing it. The planning timeouts are the
%    ones with a measured number to check against (2.483 s and 2.970 s at the
%    time of writing); this checks the ordering that makes them meaningful.
if cfg.sup.timeout.ground <= cfg.provider.timeout
    error('hmm:setup:groundTimeoutTooShort', ...
        ['cfg.sup.timeout.ground (%.2f s) does not exceed ' ...
         'cfg.provider.timeout (%.2f s). The supervisor would abandon the ' ...
         'grounding request before the provider itself gives up, so the ' ...
         'provider''s own failure code could never reach the planner.'], ...
        cfg.sup.timeout.ground, cfg.provider.timeout);
end

% 2  The queue capacity is the width of every per-item array on the ActionQueue
%    and TaskResult buses AND the size of the chart-local arrays in TaskPlanner.
%    A mismatch shows up as a bus/chart dimension error at compile, but only if
%    the capacity is what changed - so state the dependency here where it is
%    readable rather than leaving it implicit in eight array declarations.
if cfg.queue.capacity < 2
    error('hmm:setup:queueTooSmall', ...
        ['cfg.queue.capacity is %d. The planner expands one transfer into ' ...
         'at least a pick and a place, so a capacity below 2 reports ' ...
         'overflow on every task.'], cfg.queue.capacity);
end

% 3  The base has to be able to STAND where it has to WORK, and the two numbers
%    that decide those come from opposite ends of the project. hmmPlanBasePath grows
%    the obstacles by the base's footprint plus cfg.safety.plannerMargin; the chart
%    places its goal cfg.sup.baseStandoff back from the target along the approach
%    bearing. The target stands ON a surface the ceiling cameras map as an
%    obstacle, so a stand-off inside the grown region is an invalid goal state
%    before a single tree node is sampled.
%
%    IT IS geom.forward THAT BINDS, NOT geom.inflation, and getting that wrong would make
%    this check pass a stand-off the planner refuses - which is the exact failure mode the
%    paragraph below describes. The footprint is covered by a chain of circles laid along
%    the base's length, so the driving region is geom.inflation wide ACROSS the robot and
%    geom.forward deep AHEAD of it, and geom.forward is the larger of the two because the
%    end circles bulge past the front face. A stand-off is measured along the approach
%    bearing with the base facing the target, so it is the front of the chain that has to
%    clear the surface. hmmBaseFootprint's header has the numbers and the trade.
%
%    THIS IS THE DEFECT THAT SHIPPED, and it is worth stating why nothing caught
%    it. Both numbers were individually defensible - the inflation honestly
%    described a splayed stowed arm, the stand-off honestly described the arm's
%    reach - and the model reported their contradiction as PathPlanFailed, the same
%    code it returns when a room is genuinely walled off. Six months of looking at
%    that code would not find a geometry error, because from inside the planner
%    there isn't one: it was asked for an impossible pose and said so.
%    BOTH STAND-OFFS ARE CHECKED, because there are now two. The place parks closer
%    than the pick - cfg.sup.placeStandoff, for the reasons hmmConfig records - and a
%    per-verb radius that nobody validated would reintroduce exactly this defect on
%    the one verb the check had never seen.
%    AND geom.forward IS NO LONGER THE FLOOR, BECAUSE THE BASE NO LONGER HAS TO ARRIVE
%    NOSE-ON. Everything above assumes the stand-off is measured along the approach bearing
%    with the base facing the target, which was true while the yaw in a goal was whatever the
%    chart derived. hmmStandoffRing now chooses the arrival heading against the arm's own
%    reach, and a base parked ACROSS the approach presents the side of the circle chain
%    instead of its end: all five centres sit at the stand-off and the region reaches
%    geom.inflation toward the surface, not geom.forward. On this robot that is 0.290 m
%    against 0.502 m - 212 mm of stand-off that a turned arrival simply does not need.
%
%    SO THERE ARE TWO THRESHOLDS NOW AND THEY FAIL DIFFERENTLY. Inside geom.inflation no
%    heading fits and the error above still stands, unchanged in kind. Between inflation and
%    forward the leg is feasible but ONLY turned, which is a real coupling rather than a
%    detail: it means the leg depends on the heading search actually running, so a
%    cfg.sup.arrivalYawOffsets that stopped offering alternatives would silently reinstate
%    the shipped defect - a pose the chart derives, the planner refuses, and the supervisor
%    retries from a base that has not moved. That is checked rather than assumed.
%
%    cfg.sup.placeStandoff = 0.480 m is in that band deliberately; hmmConfig records why the
%    arm's lever ceiling left no radius that clears geom.forward as well.
%
%    THE HELD EXTENT IS PART OF EACH LEG NOW, because the arm checks below are indexed by how
%    high the TOOL ends up and that depends on what is in the gripper. It is not part of the
%    footprint checks above: those use the empty-handed geometry deliberately, since a carried
%    object grows the inflation and the error they raise is about the narrowest presentation.
baseGeom  = hmmBaseFootprint(cfg, zeros(3, 1));
%
%    AND THE FOURTH COLUMN IS THE HEIGHT THAT LEG IS REQUIRED TO SERVICE, which is a property
%    of the SCENE rather than of the robot - hmmConfig has no way to see the apartment. The pick
%    lifts from the side table at 0.550 m; the place sets down on a kitchen work surface at
%    0.900 m, the highest this apartment presents. They are stated per leg because the two legs
%    are not interchangeable: the pick parks 270 mm further back and genuinely cannot work at
%    counter height, which is a scope limitation of the shipped scene and not a misconfiguration.
standoffs = {'cfg.sup.baseStandoff',  cfg.sup.baseStandoff,  zeros(3, 1),           0.550; ...
             'cfg.sup.placeStandoff', cfg.sup.placeStandoff, [0.101; 0.070; 0.090], 0.900};
for k = 1:size(standoffs, 1)
    if standoffs{k, 2} <= baseGeom.inflation
        error('hmm:setup:standoffInsideInflation', ...
            ['%s (%.3f m) is not outside the %.3f m the planner''s driving region ' ...
             'reaches sideways from the base origin (footprint %.3f x %.3f m covered by ' ...
             '%d circles of %.3f m, plus a plannerMargin of %.3f m). That is the ' ...
             'NARROWEST the footprint can be presented to the surface, so no arrival ' ...
             'heading fits and the base cannot be planned to a pose that close to the ' ...
             'surface the target stands on: every base plan will report PathPlanFailed at ' ...
             'the goal-state check. Either tuck the arm (cfg.arm.stowConfig) to ' ...
             'shrink the footprint or stand further back.'], ...
            standoffs{k, 1}, standoffs{k, 2}, baseGeom.inflation, ...
            baseGeom.length, baseGeom.width, baseGeom.numCircles, ...
            baseGeom.physical, baseGeom.margin);
    end
    if standoffs{k, 2} <= baseGeom.forward && cfg.sup.arrivalYawOffsets < 2
        error('hmm:setup:standoffNeedsTurnedArrival', ...
            ['%s (%.3f m) is inside the %.3f m the driving region reaches AHEAD of the ' ...
             'base origin, so this leg is only serviceable with the base turned across ' ...
             'the approach - but cfg.sup.arrivalYawOffsets is %d, so hmmStandoffRing has ' ...
             'no heading to offer but facing the target. Either raise the stand-off above ' ...
             '%.3f m or give the search headings to try.'], ...
            standoffs{k, 1}, standoffs{k, 2}, baseGeom.forward, ...
            cfg.sup.arrivalYawOffsets, baseGeom.forward);
    end
end

% 4  And it has to be able to REACH from there. Standing further back to satisfy the check
%    above is free until it is not: every metre of stand-off is a metre of LEVER the arm has to
%    span horizontally from its own mount, and hmmArmLeverBand says how much lever is admissible
%    at a given tool height. Checked at the working height the scene actually presents rather
%    than at a nominal one, so the bound moves when the props do.
%
% THIS USED TO RECOMPUTE THE MOUNT HEIGHT AS baseLinkHeight + pedestal.height, AND
% THAT DROPPED THE CHASSIS DECK. The pedestal is mounted on top of the chassis, so
% the arm sits baseLinkHeight + chassisSize(3) + pedestal.height above the floor,
% and the omitted term understated every reach figure below by chassisSize(3). It
% went unnoticed because the error was 0.110 m and the check still passed: the
% envelope came out at 1.031 m against a 0.95 m counter, so the warning stayed
% quiet while reporting a mount height the tree disagreed with. Importing the
% vendor chassis doubled the missing term to 0.220 m, which dropped the envelope to
% 0.921 m and finally tripped the warning - on a robot whose arm had not moved at
% all. A duplicated formula that has to be kept in step with a geometric assembly
% will eventually drift from it, so read the invariant instead: hmmConfig states
% armMountHeight and derives pedestal.height from it.
% AND THE FIGURE THIS CHECK REPORTED WAS A SPHERE ON A NOSE-ON ARRIVAL, WHICH IS TWO
% ASSUMPTIONS THE ROBOT NO LONGER SATISFIES. It computed lever = standoff - offset(1) and
% zSpan = sqrt(reach^2 - lever^2), then warned when mountZ + zSpan fell short of the counter.
% Both halves are wrong now and they pull in opposite directions, which is why the warning it
% printed was simultaneously alarming and meaningless:
%
%   * THE LEVER. offset(1) is -0.060 m since the assembly was point-reflected, so subtracting
%     it ADDS 60 mm - the nose-on figure, and nose-on is very nearly the WORST heading now that
%     the mount sits aft. hmmStandoffRing chooses the heading, so the mount may sit anywhere on
%     a norm(offset(1:2)) = 66 mm circle about the goal, and the best heading is 132 mm of lever
%     better than the one this line assumed. It also ignored cfg.plan.baseGoalTolerance, which
%     licenses the base to stop 113 mm further out, and reach fails OUTWARD.
%   * THE SPHERE. cfg.arm.reach is the span with the wrist FREE. Every grasp and place in this
%     example holds the tool VERTICAL, so the gripper's length is spent on height and buys no
%     radius at all, and the admissible set is not a ball about the shoulder. hmmArmLeverBand is
%     the measured curve that replaced it - a lever ceiling per tool height, taken with the
%     solver the model runs - and it is what every other consumer now reads.
%
% Composed, the old form warned that "nothing on a 0.95 m work surface is reachable" on a robot
% that services a 0.900 m counter with two free bearings at cfg.sup.placeStandoff, measured by
% the same function the model gates on. A check that is wrong in the permissive direction is
% dangerous; one that is wrong in the refusing direction, as this was, trains the reader to
% ignore setup warnings, which is how the next real one gets missed. So it asks the measured
% question instead, per leg, with that leg's held object.
mountZ = cfg.pedestal.armMountHeight;
tolRad = norm(cfg.plan.baseGoalTolerance(1:2));
for k = 1:size(standoffs, 1)
    % The best heading any arrival can offer, at the furthest the tolerance licenses.
    lever = standoffs{k, 2} + tolRad - norm(cfg.pedestal.offset(1:2));

    % The height band that lever fits in, swept rather than solved: the ceiling is a table and
    % inverting it analytically would be a second formula to keep in step with it. 5 mm is well
    % under the 50 mm breakpoint spacing, so the edges are reported to the interpolation's own
    % resolution rather than to the table's.
    zTry  = 0:0.005:2;
    fits  = false(size(zTry));
    for i = 1:numel(zTry)
        [lo, hi] = hmmArmLeverBand(cfg, zTry(i), standoffs{k, 3});
        fits(i)  = lever >= lo && lever <= hi;
    end

    if ~any(fits)
        error('hmm:setup:standoffBeyondReach', ...
            ['%s (%.3f m) needs %.3f m of lever at the best arrival heading and the ' ...
             'furthest pose cfg.plan.baseGoalTolerance licenses, which is outside ' ...
             'hmmArmLeverBand at EVERY height: the arm could not work at that radius on ' ...
             'any target. Stand closer, or re-measure the hmmArmLeverBand ceiling ' ...
             'if the arm itself changed.'], ...
            standoffs{k, 1}, standoffs{k, 2}, lever);
    end

    % Reported rather than merely checked: it is the single most useful number for deciding
    % whether a scenario is feasible before running it, and without it a target 60 mm too high
    % arrives as ArmPlanFailed with nothing to connect it to. The margin against the height the
    % leg has to service is printed with it, because the ceiling on its own does not say whether
    % the leg is comfortable or one interpolation step from failing - this place leg clears its
    % counter by 25 mm, which is the honest figure and not a reassuring one.
    ceilZ = zTry(find(fits, 1, 'last'));
    fprintf(['Reach envelope: at %s (%.2f m) the arm works on surfaces %.3f to %.3f m ' ...
             'high while holding [%.3f %.3f %.3f]; needs %.3f m (%+.0f mm spare, lever ' ...
             '%.3f m, mount %.3f m).\n'], ...
        standoffs{k, 1}, standoffs{k, 2}, zTry(find(fits, 1)), ceilZ, ...
        standoffs{k, 3}, standoffs{k, 4}, 1000 * (ceilZ - standoffs{k, 4}), lever, mountZ);

    if ceilZ < standoffs{k, 4}
        warning('hmm:setup:legSurfaceOutOfReach', ...
            ['From %s (%.2f m) the arm works no higher than %.3f m while holding ' ...
             '[%.3f %.3f %.3f], and this leg has to service a %.3f m surface, so its ' ...
             'targets will be reported as ArmPlanFailed. The arm mounts at %.3f m: lowering ' ...
             'this stand-off buys height very nearly one for one until the footprint stops ' ...
             'fitting, and raising cfg.pedestal.armMountHeight buys it outright - ' ...
             'cfg.pedestal.height is derived from that and setting it instead will be ' ...
             'overwritten, and the stowed arm moves with it, so cfg.arm.stowConfig must ' ...
             'then be re-derived.'], ...
            standoffs{k, 1}, standoffs{k, 2}, ceilZ, standoffs{k, 3}, ...
            standoffs{k, 4}, mountZ);
    end
end

% 5  THE PLACE IS SCREENED ON A POSE 0.140 m ABOVE THE PLACE POINT, AND THE BASE IS
%    ALLOWED TO ARRIVE 0.113 m OFF ITS GOAL. Check 8 asks whether the arm can reach a
%    target from the stand-off it was TOLD to use. Neither question is the one the run
%    fails on. MotionPlanning/TargetSelect lifts the place tool target by
%    releaseClearance + hmmGraspAim's rise - 0.0556 m for the shipped mug, where the fixed
%    0.65 fraction it replaced gave 0.081 m, a naive mid-height contract that ignored the pad
%    drop gave 0.060 m, and the original Extent(3) + releaseClearance - graspDepth gave
%    0.085 m. The lift has to match whatever the PICK
%    used, because the tool holds the object exactly that far above its centre; that is why
%    raising the grasp raises this too, and why it is not free. hmmScreenArmPlan then gates on the
%    PRE-place a further cfg.sup.approachDepth back along the tool's own z - which for
%    a top-down place is pure height - and the base parks wherever
%    cfg.plan.baseGoalTolerance lets it, which is FURTHER OUT than commanded because it
%    approaches from outside and stops on a tolerance. Multiply those together and the
%    place leg failed with PregraspUnreachable (31) while every check above passed.
%
%    So this is the composed check: the worst RADIUS the base may legally present,
%    against the largest radius the pre-place is known to solve at.
%
%    AND IT IS A RADIUS RATHER THAN A FRACTION OF cfg.arm.reach, DELIBERATELY. Writing
%    it as "need <= cfg.arm.reach" is the obvious form and it is wrong in the dangerous
%    direction: the IK solver reaches this very pose at 101.3% of cfg.arm.reach with
%    a 4.2e-09 residual, because cfg.arm.reach is the arm's nominal span and the pose is
%    solved to the TOOL frame. A check in that currency refuses poses the solver reaches
%    - the same sign of error as the mount-height defect above, and it fails the same
%    way, by refusing work the robot can do. cfg.sup.placeRadiusMax is measured instead,
%    by sweeping the radius and watching where the solver stops converging.
%
%    AND THAT MEASUREMENT IS NOW STALE IN THE SAFE DIRECTION, so it is left alone until it
%    is re-run. It was taken with the place tool target 25 mm higher than it now sits. A
%    lower tool shortens the hypotenuse the solver has to span, so the radius at which the
%    pre-place stops converging can only have moved OUTWARD - the shipped ceiling is
%    therefore a lower bound on the real one and this check is pessimistic, not permissive.
%    The sweep has not been re-run since the lift changed; when it is, the band it reports
%    should be wider than the one this figure came from.
%
%    The comparison is against the base's WHOLE arrival tolerance, not its typical
%    error, because a tolerance is a licence: on the failing run the base used 72% of it
%    and that alone was enough.
%
%    THE TOLERANCE IS ADDED, NOT +/-, BECAUSE ONLY ONE SIDE FAILS BADLY - not because
%    the error only goes one way. It goes both ways: the failing run overshot to 0.7951 m
%    from a 0.750 m goal, and the passing runs have undershot - 0.6320 m and, on the
%    current reference, 0.6229 m, both from a 0.660 m goal, so 28 and 37 mm of RADIAL
%    error against a 113 mm allowance. Read the radius and not the arrival's error
%    against its goal: that figure also contains the arc between the arrival and the
%    ROTATED goal, which on the current reference is 16.3 degrees of bearing.
%    Outward is the side that is checked here because outward walks off the reach ceiling
%    with the object already in the jaws. Inward costs nothing in reach, and the floor it
%    approaches is the DRIVING region - wall plus the footprint's covering radius plus
%    plannerMargin - so ending a few millimetres inside it is still clear of the counter.
%    hmmPlanBasePath validates every reference state against that map, which bounds where
%    the path may GO, not where the follower ends up: it tracks to a tolerance.
tolRadial   = norm(cfg.plan.baseGoalTolerance(1:2));
radiusWorst = cfg.sup.placeStandoff + tolRadial;
fprintf(['Place margin: stand-off %.2f m + arrival tolerance %.3f m = %.3f m worst ' ...
         'radius, against a measured %.3f m ceiling (%.0f mm spare).\n'], ...
    cfg.sup.placeStandoff, tolRadial, radiusWorst, cfg.sup.placeRadiusMax, ...
    1000 * (cfg.sup.placeRadiusMax - radiusWorst));
if radiusWorst > cfg.sup.placeRadiusMax
    warning('hmm:setup:placeStandoffNoToleranceMargin', ...
        ['A base that arrives at the far edge of cfg.plan.baseGoalTolerance ' ...
         '(%.3f m radially) from cfg.sup.placeStandoff (%.2f m) sits %.3f m from ' ...
         'the place point, past the %.3f m at which the PRE-place pose is measured ' ...
         'to stop solving. That arrival is LEGAL, so the run can be refused with ' ...
         'PregraspUnreachable while the object is in the jaws - which drops it. ' ...
         'Lower cfg.sup.placeStandoff (but not below the %.2f m the counter''s ' ...
         'inflation leaves free), shorten cfg.sup.approachDepth, or tighten ' ...
         'cfg.plan.baseGoalTolerance.'], tolRadial, cfg.sup.placeStandoff, ...
        radiusWorst, cfg.sup.placeRadiusMax, 0.64);
end

% THE APPROACH TIMEOUT MUST STAY A BACKSTOP AND NOT BECOME THE SCHEDULE. These two
% constants sit about 160 lines apart in hmmConfig and have to keep a ratio, which is
% exactly the arrangement a comment does not maintain. Checked rather than derived because
% cfg.sup.timeout.approach is declared with the other timeouts, well before approachRate
% and approachDepth exist, so the expression cannot be written in the file without moving
% a constant out of the section it belongs to.
%
% The reason it is worth a check at all: the rate USED to be derived from the timeout
% (3 * approachDepth / timeout.approach in ArmSupervisor), so a generous timeout made the
% arm slower, and half the recorded run went into four 120 mm segments at 30 mm/s. The
% dependency is inverted now, and this is what stops it inverting back by accident.
% MEASURED AT THE LONGEST SEGMENT THE MODEL CAN ASK FOR, WHICH IS NO LONGER approachDepth.
% The pick's standoff is searched per plan down cfg.sup.approachDepthLadder from
% cfg.sup.approachDepthPickMax, so the descent and the retreat are as long as the highest rung
% hmmScreenArmPlan can get away with, not as long as the bottom one. Checking the bottom rung
% would report a 5.0x margin on a segment that can be three times longer - a check that passes
% by measuring the wrong thing is worse than no check, because it is quoted.
tApproachNom = cfg.sup.approachDepthPickMax / cfg.sup.approachRate;
fprintf(['Approach: up to %.0f mm at %.3f m/s = %.2f s worst case, timeout %.2f s ' ...
         '(%.1fx margin).\n'], 1000 * cfg.sup.approachDepthPickMax, cfg.sup.approachRate, ...
    tApproachNom, cfg.sup.timeout.approach, cfg.sup.timeout.approach / tApproachNom);
if cfg.sup.timeout.approach < 2 * tApproachNom
    warning('hmm:setup:approachTimeoutTight', ...
        ['cfg.sup.timeout.approach (%.2f s) leaves less than a factor of two over ' ...
         'the longest segment cfg.sup.approachDepthPickMax / cfg.sup.approachRate = %.2f s. ' ...
         'The ramp reaches its target at the nominal time and then has to SETTLE ' ...
         'inside the arm''s tolerance before ArmSupervisor will report DONE, so a ' ...
         'timeout this close fires on a healthy approach and reports it as F_ARM. ' ...
         'Raise the timeout or lower cfg.sup.approachRate.'], ...
        cfg.sup.timeout.approach, tApproachNom);
end

end

%% ------------------------------------------------------------------------
function frozen = freezeGripperJoints(tree, cfg)
%FREEZEGRIPPERJOINTS Replace the gripper's actuated joints with fixed ones.
%   TREE is mutated in place - rigidBodyTree is a handle class, and the caller
%   has already copied. Returns the body names that were frozen.
%
%   Each joint's JointToParentTransform is carried over, so every body stays
%   exactly where it was at the zero configuration: the fingers are frozen OPEN,
%   which is the pose the arm flies in. Deliberately indexed off BodyNames, which
%   is in configuration order, rather than off gripperCoupling.indices - those are
%   slots in the 15-DOF whole-robot vector, and this is the 12-joint subtree.
names = tree.BodyNames;
movable = names(cellfun(@(n) ~strcmp(getBody(tree, n).Joint.Type, 'fixed'), names));
if numel(movable) ~= cfg.arm.nJoints + cfg.gripper.nJoints
    error('hmm:setup:taskTreeJoints', ...
        'Expected %d actuated joints in the IK subtree, found %d.', ...
        cfg.arm.nJoints + cfg.gripper.nJoints, numel(movable));
end

frozen = movable(cfg.arm.nJoints + 1:end);
for k = 1:numel(frozen)
    joint = rigidBodyJoint([frozen{k} '_frozen'], 'fixed');
    joint.setFixedTransform(getBody(tree, frozen{k}).Joint.JointToParentTransform);
    replaceJoint(tree, frozen{k}, joint);
end
end

%% ------------------------------------------------------------------------
function task = buildTaskRequest(cfg)
%BUILDTASKREQUEST Encode cfg.task.instruction into a TaskRequest ground value.
task = Simulink.Bus.createMATLABStruct('TaskRequest');
codes = uint8(cfg.task.instruction);
n = min(numel(codes), double(cfg.provider.instructionChars));
task.TaskId               = uint32(1);
task.Instruction(1:n)     = codes(1:n);
task.InstructionLen       = uint16(n);
task.SubmitTime           = 0;
task.Cancel               = false;
task.Valid                = true;
end

%% ------------------------------------------------------------------------
function clearProjectCaches(folder)
%CLEARPROJECTCACHES Drop the persistent state of every function in FOLDER.
%   clear NAME removes a function from memory together with its persistent variables, so
%   the next call rebuilds whatever it had cached. Applied to every file rather than only
%   to the ones known to cache something: the cost of clearing a function that caches
%   nothing is that it is read from disk once more, and the cost of missing one is a
%   simulation that quietly answers from a cfg the user has already edited.
%
%   A function that is currently executing cannot be cleared and is silently left alone,
%   which is correct here - hmmSetup lives at the project root, not in functions/, so
%   nothing on this stack is a candidate.
files = dir(fullfile(folder, '*.m'));
for k = 1:numel(files)
    clear(files(k).name(1:end - 2));
end
end

%% ------------------------------------------------------------------------
function names = valueNames()
%VALUENAMES The non-bus variables every block parameter resolves against.
names = {'cfg', 'robot', 'robotInfo', 'idx', 'apertureMap', ...
         'gripperCoupling', 'armJointLimits', 'armIkTree', 'armDynTree', ...
         'armTaskTree', 'armPlanLimits', ...
         'rays', 'invalidRayHits', 'navMapGrid', 'movableRegistry', ...
         'actorNameTable', ...
         'sunAzimuth', 'sunAltitude', ...
         'actionQueueInit', 'actionItemInit', 'objectTargetInit', ...
         'placeTargetInit', 'basePlanInit', 'armPlanInit', ...
         'graspCandidateInit', 'perceptionStatusInit', ...
         'baseCommandInit', 'baseStatusInit', 'armCommandInit', ...
         'armStatusInit', 'gripperCommandInit', 'gripperStatusInit', ...
         'safetyStatusInit', 'taskResultInit', 'robotStateInit', ...
         'taskRequestStub'};
end

function names = busNames()
%BUSNAMES The Simulink.Bus objects, listed once so isReady and collectFromBase
%   cannot disagree about which types the model needs.
names = {'RayHits', 'CameraFrame', 'RobotState', 'GripperState', ...
         'GraspCandidate', 'TaskRequest', 'ActionQueue', 'ActionItem', ...
         'PerceptionStatus', 'MovableRegistry', 'ObjectTarget', ...
         'PlaceTarget', 'BasePlan', 'ArmPlan', ...
         'BaseCommand', 'BaseStatus', 'ArmCommand', 'ArmStatus', ...
         'GripperCommand', 'GripperStatus', 'SafetyStatus', 'TaskResult'};
end

function tf = isReady()
%ISREADY True when the base workspace already holds a consistent 15-DOF set.
%   A NEGATIVE ANSWER IS ALWAYS AVAILABLE, AND THE PROBE MUST NEVER THROW. The
%   base workspace is shared with whatever else the user has been running, and a
%   diagnostic script that happens to use idx as a loop variable leaves a double
%   where the index map was. Every name here can therefore hold the wrong type,
%   not merely be absent. Letting that error out is the wrong answer twice over:
%   it reports a clobbered workspace as a broken hmmSetup, and it blocks the
%   rebuild that would have repaired it. Anything unexpected means "not ready",
%   which is exactly the condition that triggers the rebuild.
wanted = [valueNames(), busNames()];
tf = all(cellfun(@(n) evalin('base', sprintf('exist(''%s'', ''var'') == 1', n)), wanted));
if ~tf
    return
end

% A stale tree from an earlier edit is worse than no tree at all, so the DOF
% count is checked rather than assumed.
try
    n = evalin('base', 'idx.numDOF');
    tf = isnumeric(n) && isscalar(n) && n == 15;
catch
    tf = false;
end
if ~tf
    return
end

% cfg BEING PRESENT IS NOT cfg BEING DERIVED. Anything calling hmmConfig at the base prompt
% publishes an underived cfg, and accepting it would let the next simulation fail far from the
% cause. hmmSetup sets cfg.derived; hmmConfig never does, so its absence triggers the rebuild.
try
    tf = evalin('base', 'isfield(cfg, ''derived'') && isequal(cfg.derived, true)');
catch
    tf = false;
end
end

function data = collectFromBase()
%COLLECTFROMBASE Gather the already-present variables into the return struct.
names = valueNames();
data = struct();
for k = 1:numel(names)
    data.(names{k}) = evalin('base', names{k});
end
bn = busNames();
data.buses = struct();
for k = 1:numel(bn)
    data.buses.(bn{k}) = evalin('base', bn{k});
end
end
