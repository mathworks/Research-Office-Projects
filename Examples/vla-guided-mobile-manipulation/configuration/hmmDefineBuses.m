function buses = hmmDefineBuses(cfg)
%HMMDEFINEBUSES Create the Simulink.Bus objects the model interfaces use.
%
%   buses = HMMDEFINEBUSES(cfg) builds every bus type the model's interfaces
%   use and assigns each to the base workspace under its own name, which is
%   where Simulink resolves bus types from.
%
%   Plant and rig interfaces:
%       RayHits         one camera's calibrated ray-trace return
%       CameraFrame     one camera's complete, self-describing frame
%       RobotState      the plant's full truth and measurement report
%       GripperState    GripperControl -> Plant
%       GraspCandidate  perception -> the plant's grasp capture gate
%
%   Perception and planning service interfaces:
%       TaskRequest       a natural-language instruction submitted to grounding
%       ActionQueue       fixed-capacity queue of grounded actions
%       ActionItem        one dequeued action
%       PerceptionStatus  request outcome, including the credential ROUTE
%       MovableRegistry   runtime registry of movable actors
%       ObjectTarget      grounded pick target in metric 3-D
%       PlaceTarget       grounded place target with its fitted support surface
%       BasePlan          time-parameterised SE(2) base reference
%       ArmPlan           time-parameterised, collision-checked arm reference
%
%   Supervisory interfaces - the ONLY signals crossing between the planner and
%   its three subordinates:
%       BaseCommand    / BaseStatus
%       ArmCommand     / ArmStatus
%       GripperCommand / GripperStatus
%       SafetyStatus      latched e-stop broadcast, monitor -> all four charts
%       TaskResult        the planner's per-action report on one instruction
%
%   Every dimension and data type below was measured by compiling the
%   corresponding Sim3D block and reading its port back, not inferred from
%   documentation. The notable ones:
%
%     - Simulation 3D Camera emits RGB as uint8 [rows cols 3] and depth as
%       DOUBLE, quantised in steps of cfg.depth.renderQuantum (59.6 um, measured).
%       CameraFrame carries an explicit DepthSource element so a consumer knows
%       whether a depth came from the render or from the calibrated ray path.
%     - Simulation 3D Ray Tracer emits single/single/single/uint32/boolean and
%       nRays*(NumberOfBounces+1) rows. With cfg.depth.rayBounces = 0 that is
%       one row per ray, which is what a depth reading means.
%     - All five cameras must share one image size: a bus element cannot hold
%       two different dimensions, so the wrist camera differs by lens instead.
%
%   CameraFrame and RayHits are used as NONVIRTUAL buses in the model so the
%   five camera frames can be concatenated into a [5 1] bus array. The
%   vision-language contract addresses cameras by 1-based index, so an array
%   indexed by that number is the interface that cannot be misread; five
%   separate ports would push the index convention into the wiring.
%
%   See also hmmConfig, hmmSetup, Simulink.Bus.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg struct = hmmConfig()
end

nRayRows = cfg.depth.rayRows;
imgRows  = cfg.camera.imageSize(1);
imgCols  = cfg.camera.imageSize(2);

%% RayHits - the calibrated depth path for one camera
rayHits = makeBus('RayHits', ...
    'One camera calibrated ray-trace return; metrically exact range.', { ...
    'HitDistances', 'single',  [nRayRows 1], 'm',   'First-intersection range per ray.'
    'IsValidHit',   'boolean', [nRayRows 1], '',    'False where the ray hit nothing within rayLength.'
    'HitLocations', 'single',  [nRayRows 3], 'm',   'World-frame intersection points.'
    'SurfaceIds',   'uint32',  [nRayRows 1], '',    'Sim3D surface identifier of the hit.'
    'GridSize',     'uint16',  [2 1],        '',    '[rows cols] of the ray grid.'});

%% CameraFrame - one self-describing camera frame
% Self-describing on purpose: a consumer needs no configuration lookup to
% interpret a frame, which is what keeps the perception stage object-agnostic.
% Translation and Rotation are [1 3] here and [3 1] on RobotState. That is not
% an oversight: these two elements mirror the Simulation 3D Camera output ports
% exactly, so no reshape sits between the sensor and its own record, whereas
% RobotState is the plant's own report and uses column vectors throughout.
cameraFrame = makeBus('CameraFrame', ...
    'One camera frame with its own intrinsics, extrinsics, depth provenance and validity.', { ...
    'CameraIndex',  'uint8',   1,                    '',    '1-based contract index; 5 is the wrist camera.'
    'RGB',          'uint8',   [imgRows imgCols 3],  '',    'Colour image, row 1 is the top image row.'
    'Depth',        'double',  [imgRows imgCols],    'm',   'Render depth. Quantised to renderQuantum; see DepthSource.'
    'DepthSource',  'uint8',   1,                    '',    'cfg.depth.SOURCE_RENDER or SOURCE_RAYTRACE.'
    'Calib',        'Bus: RayHits', 1,               '',    'Calibrated depth path; empty of valid hits when DepthSource is render.'
    'Intrinsics',   'double',  [4 1],                'px',  '[fx fy cx cy].'
    'ImageSize',    'uint16',  [2 1],                'px',  '[rows cols].'
    'Translation',  'double',  [1 3],                'm',   'Camera origin in the world frame.'
    'Rotation',     'double',  [1 3],                'rad', 'Camera orientation, Sim3D Euler order.'
    'Timestamp',    'double',  1,                    's',   'Simulation time the frame was captured.'
    'Valid',        'boolean', 1,                    '',    'False until the render pipeline has produced a frame.'});

%% RobotState - the plant's report
robotState = makeBus('RobotState', ...
    'Plant truth and measurement for the 15-DOF mobile manipulator.', { ...
    'BasePose',           'double',  [3 1], 'm,m,rad',   'World [x y theta] of the chassis.'
    'BaseTwist',          'double',  [3 1], 'm/s,rad/s', 'Realised body-frame [vx vy omega] after wheel saturation.'
    'WheelSpeeds',        'double',  [4 1], 'rad/s',     'Mecanum wheel speeds [FL FR RL RR].'
    'ArmJointPos',        'double',  [6 1], 'rad',       'UR5e joint positions.'
    'ArmJointVel',        'double',  [6 1], 'rad/s',     'UR5e joint velocities.'
    'GripperAperture',    'double',  1,     'm',         'Achieved jaw aperture.'
    'GripperDriverAngle', 'double',  1,     'rad',       '2F-85 driver joint angle.'
    'ToolTranslation',    'double',  [3 1], 'm',         'tcp_frame origin in the world frame.'
    'ToolRotation',       'double',  [3 1], 'rad',       'tcp_frame orientation, Euler XYZ.'
    'ActorTranslation',   'double',  [3 1], 'm',         'Rendered actor root pose reported by Simulation 3D Robot.'
    'ActorRotation',      'double',  [3 1], 'rad',       'Rendered actor root orientation, Sim3D Euler order.'
    'AttachedActorId',    'uint8',   1,     '',          'Actor held by the tool frame; cfg.actor.noneId when none.'
    'IsGrasped',          'boolean', 1,     '',          'True while an actor is attached to the tool frame.'
    'Timestamp',          'double',  1,     's',         'Simulation time of this report.'
    'Valid',              'boolean', 1,     '',          'False before the plant has produced a state.'});

%% GripperState - GripperControl -> Plant
% GraspRequest is carried here, not sent straight from the supervisor to the
% plant, because the design puts the request with the GRIPPER and the decision
% with the PLANT. The gripper asserts a request only once its jaws have
% actually settled on the commanded aperture, which is the one thing about a
% grasp the gripper legitimately knows.
gripperState = makeBus('GripperState', ...
    'Commanded and achieved gripper state in aperture terms, plus the grasp request.', { ...
    'ApertureCmd',  'double',  1,     'm',   'Saturated, rate-limited aperture command.'
    'Aperture',     'double',  1,     'm',   'Achieved aperture through the measured forward map.'
    'DriverAngle',  'double',  1,     'rad', '2F-85 driver joint angle.'
    'JointVector',  'double',  [6 1], 'rad', 'All six 2F-85 joints after measured mimic coupling.'
    'Settled',      'boolean', 1,     '',    'At command and no longer moving.'
    'GraspRequest', 'boolean', 1,     '',    'Grasp commanded AND jaws settled. A request, never a decision.'
    'Valid',        'boolean', 1,     '',    'False before the first command is applied.'});

%% GraspCandidate - perception -> the plant's capture gate
% Deliberately generic. The design forbids any element that names a particular
% scene object, so identity is a runtime integer and extent is a runtime
% measurement.
%
% POSITION IS THE OBJECT'S CENTRE AND IT IS NOT ObjectTarget.Position, WHICH IS THE
% ONE THING TO KNOW ABOUT THIS BUS. ObjectTarget.Position is the SUPPORT SURFACE under
% the object; the centre is that plus Extent(3)/2, and Gripper/GraspCandidate does the
% addition. The reason the gate needs the centre rather than either the surface or the
% point the arm was aimed at is cfg.grasp.captureRadius itself: it is
% cfg.gripper.maxOpenPos/2 + cfg.grasp.captureMargin, the half jaw stroke plus a margin,
% so it asserts that the OBJECT lies inside the volume the jaws sweep. Measuring to the
% commanded grasp point instead would reduce WithinCapture to a tracking check
% ArmControl/TrackingMetrics already performs.
%
% THIS ELEMENT ONCE SAID "candidate grasp point" AND WAS FED THE SUPPORT SURFACE, and
% that cost a run. Once the Position contract moved from the top face to the support
% surface, the whole deliberate grasp rise - hmmGraspAim's aim + padDrop(GraspExtent), which
% was 66.0 mm on the shipped mug and is 52.6 mm now - was charged to the capture gate as
% error. WithinCapture measured 66.1 mm against the 62.5 mm radius and the close faulted
% NotGraspable with the tool 0.1 mm from its target in xy. It had passed until then only
% because the tool used to sit BELOW the support surface. A bus element whose description
% and whose producer disagree is how that happens, which is why the disagreement is
% spelled out here rather than left to the two blocks that resolve it.
graspCandidate = makeBus('GraspCandidate', ...
    'Runtime grasp candidate. Carries no object-specific configuration.', { ...
    'ActorId',      'uint8',   1,     '',  'Movable actor identifier; cfg.actor.noneId when none.'
    'Position',     'double',  [3 1], 'm', 'Candidate object CENTRE in the world frame - what cfg.grasp.captureRadius is measured to. NOT ObjectTarget.Position, which is the support surface.'
    'Extent',       'double',  1,     'm', 'Measured graspable extent across the jaw axis.'
    'NumInCapture', 'uint8',   1,     '',  'Movable actors within the capture radius; must be 1 to grasp.'
    'Valid',        'boolean', 1,     '',  'False when perception cannot vouch for this candidate.'});

%% ---- Perception and planning service interfaces ----
% Text travels as a fixed-capacity uint8 code array with an explicit length. A
% bus element cannot be a string or a variable-size array, and truncating
% silently would turn "put the mug on the left shelf" into a different task, so
% the length is carried and the caller can see when it saturated.
nQ    = cfg.queue.capacity;
nLbl  = cfg.provider.labelChars;
nIns  = cfg.provider.instructionChars;
nMov  = cfg.actor.maxMovable;
nBW   = cfg.plan.baseWaypointCap;
nBS   = cfg.plan.baseSampleCap;
nAW   = cfg.plan.armWaypointCap;
nAS   = cfg.plan.armSampleCap;
nArm  = cfg.arm.nJoints;

taskRequest = makeBus('TaskRequest', ...
    'A free-form natural-language instruction submitted to the perception service.', { ...
    'TaskId',         'uint32',  1,        '',  'Monotonic submission counter.'
    'Instruction',    'uint8',   [nIns 1], '',  'Instruction text as character codes, zero-padded.'
    'InstructionLen', 'uint16',  1,        '',  'Characters actually used; equals capacity when truncated.'
    'SubmitTime',     'double',  1,        's', 'Simulation time the instruction was submitted.'
    'Cancel',         'boolean', 1,        '',  'Abandon any outstanding request and the queued actions.'
    'Valid',          'boolean', 1,        '',  'False before any instruction has been submitted.'});

% ActionQueue is deliberately a set of PARALLEL fixed-size arrays rather than an
% array of ActionItem buses. The queue is produced by a MATLAB Function block,
% and an array-of-buses output there has to be pre-declared as a struct array
% before the extrinsic provider call can be assigned into it - which puts the
% element types in code, i.e. in a second place, competing with this file.
% Parallel arrays carry identical information with the types declared once here.
actionQueue = makeBus('ActionQueue', ...
    'Fixed-capacity queue of grounded actions with per-item execution state.', { ...
    'TaskId',      'uint32',  1,          '',  'Instruction this queue was grounded from.'
    'RequestId',   'uint32',  1,          '',  'Provider request that produced it.'
    'Count',       'uint8',   1,          '',  'Actions actually present, 0..capacity.'
    'Head',        'uint8',   1,          '',  '1-based index of the next action to execute; 0 when empty.'
    'Verb',        'uint8',   [nQ 1],     '',  'cfg.queue.VERB_TRANSFER or VERB_NAVIGATE per slot.'
    'Label',       'uint8',   [nLbl nQ],  '',  'Object label character codes, one column per slot.'
    'LabelLen',    'uint8',   [nQ 1],     '',  'Label characters used per slot.'
    'PickPoint',   'double',  [2 nQ],     '',  'Normalised [y; x] pick pixel per slot, 0..pixelNormScale.'
    'PickImage',   'uint8',   [nQ 1],     '',  '1-based camera index of the pick pixel.'
    'PlacePoint',  'double',  [2 nQ],     '',  'Normalised [y; x] place pixel; unused for navigate.'
    'PlaceImage',  'uint8',   [nQ 1],     '',  '1-based camera index of the place pixel.'
    'Confidence',  'double',  [nQ 1],     '',  'Provider confidence per action, 0..1.'
    'ExecState',   'uint8',   [nQ 1],     '',  'cfg.queue.EXEC_* per slot.'
    'ItemFailure', 'uint8',   [nQ 1],     '',  'cfg.fail.* per slot; OK while admissible.'
    'Overflow',    'boolean', 1,          '',  'True when the provider returned more actions than capacity.'
    'FailureCode', 'uint8',   1,          '',  'cfg.fail.* for the queue as a whole.'
    'Timestamp',   'double',  1,          's', 'Simulation time the queue was grounded.'
    'Valid',       'boolean', 1,          '',  'False until a request has completed.'});

actionItem = makeBus('ActionItem', ...
    'One dequeued action, the unit the executive hands to the planning services.', { ...
    'Index',       'uint8',   1,        '',  'Slot this item came from; 0 when the queue is empty.'
    'Verb',        'uint8',   1,        '',  'cfg.queue.VERB_*.'
    'Label',       'uint8',   [nLbl 1], '',  'Object label character codes, zero-padded.'
    'LabelLen',    'uint8',   1,        '',  'Label characters used.'
    'PickPoint',   'double',  [2 1],    '',  'Normalised [y; x] pick pixel.'
    'PickImage',   'uint8',   1,        '',  '1-based camera index of the pick pixel.'
    'PlacePoint',  'double',  [2 1],    '',  'Normalised [y; x] place pixel.'
    'PlaceImage',  'uint8',   1,        '',  '1-based camera index of the place pixel.'
    'Confidence',  'double',  1,        '',  'Provider confidence, 0..1.'
    'ExecState',   'uint8',   1,        '',  'cfg.queue.EXEC_*.'
    'FailureCode', 'uint8',   1,        '',  'cfg.fail.* for this item.'
    'Timestamp',   'double',  1,        's', 'Simulation time this item was dequeued.'
    'Valid',       'boolean', 1,        '',  'False when there is nothing to execute.'});

% Route is on the bus, the credential is not. Reporting WHICH route resolved is
% what makes a run reproducible; reporting the key would put it in a log file.
perceptionStatus = makeBus('PerceptionStatus', ...
    'Outcome of one grounding request. Carries the credential ROUTE, never a credential.', { ...
    'RequestId',   'uint32',  1, '',  'Monotonic request counter.'
    'TaskId',      'uint32',  1, '',  'Instruction the request was issued for.'
    'Route',       'uint8',   1, '',  'cfg.provider.ROUTE_* actually used. Never key material.'
    'State',       'uint8',   1, '',  '0 idle, 1 pending, 2 complete, 3 failed.'
    'HttpStatus',  'uint16',  1, '',  'Transport status; 0 when no request was sent.'
    'Latency',     'double',  1, 's', 'Wall-clock time the request took.'
    'NumActions',  'uint8',   1, '',  'Admissible actions returned.'
    'FailureCode', 'uint8',   1, '',  'cfg.fail.* for the request.'
    'Timestamp',   'double',  1, 's', 'Simulation time of this status.'
    'Valid',       'boolean', 1, '',  'False before the first request.'});

% The registry is how perception knows which rendered surfaces belong to objects
% that may be moved. It is EMPTY unless the discovery walk finds prop metadata,
% and an empty registry produces an honest AmbiguousObject rather than a guess.
movableRegistry = makeBus('MovableRegistry', ...
    'Runtime registry of movable actors discovered in the scene. Empty when none were found.', { ...
    'Count',     'uint8',   1,        '',  'Registered actors, 0..maxMovable.'
    'ActorId',   'uint8',   [nMov 1], '',  'Actor identifier; matches GraspCandidate.ActorId.'
    'SurfaceId', 'uint32',  [nMov 1], '',  'Sim3D surface identifier the ray tracer reports.'
    'Position',  'double',  [3 nMov], 'm', 'Last known world position, one column per actor.'
    'Extent',    'double',  [3 nMov], 'm', 'Bounding extent, one column per actor.'
    'Valid',     'boolean', 1,        '',  'False when no registry could be discovered.'});

objectTarget = makeBus('ObjectTarget', ...
    'One grounded pick target reconstructed to metric 3-D and associated with a movable actor.', { ...
    'Label',         'uint8',   [nLbl 1], '',        'Provider label character codes, zero-padded.'
    'LabelLen',      'uint8',   1,        '',        'Label characters used.'
    'Position',      'double',  [3 1],    'm',       'World SUPPORT SURFACE under the object centre, not the top face; grasp is this + hmmGraspAim(cfg, Extent, GraspExtent) rise.'
    'Orientation',   'double',  [3 1],    'rad',     'Measured principal-axis orientation, ZYX Euler.'
    'Extent',        'double',  [3 1],    'm',       'Full span; [3] is MEASURED by the wrist refinement when it applies, and sourced otherwise.'
    'GraspExtent',   'double',  1,        'm',       'Extent across the jaw axis; what the capture gate matches.'
    'ExtentSource',  'uint8',   1,        '',        'cfg.percep.EXTENT_NONE, _MEASURED, _ACTOR or _ASSUMED.'
    'ActorId',       'uint8',   1,        '',        'Associated movable actor; cfg.actor.noneId when none.'
    'NumCandidates', 'uint8',   1,        '',        'Movable actors that matched; must be 1 to proceed.'
    'SurfaceId',     'uint32',  1,        '',        'Sim3D surface the grounded pixel actually hit.'
    'Confidence',    'double',  1,        '',        'Provider confidence for the originating action.'
    'SourceCamera',  'uint8',   1,        '',        '1-based camera index the pixel came from.'
    'PixelPoint',    'double',  [2 1],    'px',      'Denormalised [row; col] in that camera.'
    'FailureCode',   'uint8',   1,        '',        'cfg.fail.* for this target.'
    'Timestamp',     'double',  1,        's',       'Frame timestamp the reconstruction used.'
    'Valid',         'boolean', 1,        '',        'False when perception cannot vouch for this target.'});

placeTarget = makeBus('PlaceTarget', ...
    'One grounded place target with the support surface fitted at that point and its admissible region.', { ...
    'Point',          'double',  [3 1], 'm',   'World place point on the fitted surface.'
    'PlaneNormal',    'double',  [3 1], '',    'Unit normal of the fitted support plane.'
    'PlaneOffset',    'double',  1,     'm',   'Plane offset d in n.x + d = 0.'
    'RegionCenter',   'double',  [3 1], 'm',   'Centre of the admissible placement region.'
    'RegionHalfSize', 'double',  [2 1], 'm',   'In-plane half-extent of the admissible region.'
    'Inset',          'double',  1,     'm',   'Inset applied from the fitted surface edge.'
    'NumInliers',     'uint16',  1,     '',    'Ray hits supporting the plane fit.'
    'SurfaceId',      'uint32',  1,     '',    'Sim3D surface identifier of the support.'
    'Confidence',     'double',  1,     '',    'Provider confidence for the originating action.'
    'SourceCamera',   'uint8',   1,     '',    '1-based camera index the pixel came from.'
    'PixelPoint',     'double',  [2 1], 'px',  'Denormalised [row; col] in that camera.'
    'FailureCode',    'uint8',   1,     '',    'cfg.fail.* for this target.'
    'Timestamp',      'double',  1,     's',   'Frame timestamp the reconstruction used.'
    'Valid',          'boolean', 1,     '',    'False when no admissible support surface was found.'});

% Both plan buses carry the geometric waypoints AND the time-parameterised
% reference with its derivative. A tracker needs the derivative as a
% feedforward, and differentiating a sampled reference downstream would
% reintroduce exactly the noise contopptraj was used to avoid.
basePlan = makeBus('BasePlan', ...
    'Time-parameterised SE(2) base reference from the triggered path-planning service.', { ...
    'RequestId',        'uint32',  1,        '',          'Plan request this reference answers.'
    'Waypoints',        'double',  [3 nBW],  'm,m,rad',   'Searched SE(2) waypoints, one column each.'
    'NumWaypoints',     'uint8',   1,        '',          'Waypoints actually present.'
    'RefTime',          'double',  [nBS 1],  's',         'Sample times from the trigger instant. Strictly increasing over the full length: entries past NumSamples are padding above Duration, so the follower can use this as the time points of its Polynomial Trajectory block. See hmmPadRefTime.'
    'RefPose',          'double',  [3 nBS],  'm,m,rad',   'Reference pose per sample.'
    'RefVel',           'double',  [3 nBS],  'm/s,rad/s', 'Reference world-frame derivative per sample.'
    'NumSamples',       'uint16',  1,        '',          'Samples actually present.'
    'Duration',         'double',  1,        's',         'Total reference duration.'
    'PathLength',       'double',  1,        'm',         'pathmetrics path length.'
    'MinClearance',     'double',  1,        'm',         'pathmetrics minimum clearance along the path.'
    'Smoothness',       'double',  1,        '',          'pathmetrics smoothness.'
    'InflationRadius',  'double',  1,        'm',         'Radius the occupancy map was inflated by.'
    'PlanTime',         'double',  1,        's',         'Wall-clock search and parameterisation time.'
    'Truncated',        'boolean', 1,        '',          'True when the plan exceeded a bus capacity.'
    'FailureCode',      'uint8',   1,        '',          'cfg.fail.* for this plan.'
    'Timestamp',        'double',  1,        's',         'Simulation time the plan was produced.'
    'Valid',            'boolean', 1,        '',          'False when no admissible path was found.'
    'GoalPose',         'double',  [3 1],    'm,m,rad',   'The goal this reference actually ends at - BaseCommand.GoalPose or its AltGoalPose, whichever drove cheaper. PathFollowing judges arrival against THIS, not the commanded pose.'});

armPlan = makeBus('ArmPlan', ...
    'Collision-checked, time-parameterised arm reference from the enabled motion-planning service.', { ...
    'RequestId',         'uint32',  1,           '',      'Plan request this reference answers.'
    'Waypoints',         'double',  [nArm nAW],  'rad',   'manipulatorRRT joint waypoints, one column each.'
    'NumWaypoints',      'uint8',   1,           '',      'Waypoints actually present.'
    'RefTime',           'double',  [nAS 1],     's',     'Sample times from the enable instant. Strictly increasing over the full length: entries past NumSamples are padding above Duration, so ArmControl can use this as the time points of its Polynomial Trajectory block. See hmmPadRefTime.'
    'RefPos',            'double',  [nArm nAS],  'rad',   'Reference joint positions per sample.'
    'RefVel',            'double',  [nArm nAS],  'rad/s', 'Reference joint velocities per sample.'
    'NumSamples',        'uint16',  1,           '',      'Samples actually present.'
    'Duration',          'double',  1,           's',     'Total reference duration.'
    'GraspPose',         'double',  [4 4],       '',      'Selected grasp pose in the world frame.'
    'PregraspConfig',    'double',  [nArm 1],    'rad',   'Standoff configuration the approach starts from.'
    'ApproachDepth',     'double',  1,           'm',     'Standoff PregraspConfig was solved at, and therefore the length ArmSupervisor must ramp the approach over. Chosen per pick from cfg.sup.approachDepthLadder, so it is NOT a constant and must not be read from hmmConfig by a consumer.'
    'GraspConfig',       'double',  [nArm 1],    'rad',   'Configuration at the grasp pose.'
    'SelectedCandidate', 'uint8',   1,           '',      'Which grasp candidate survived screening; 0 when none.'
    'PoseErrorNorm',     'double',  1,           '',      'Inverse Kinematics residual for the selected candidate.'
    'Manipulability',    'double',  1,           '',      'Reciprocal condition number of J*J'' at the grasp configuration.'
    'MinClearance',      'double',  1,           'm',     'Smallest checkCollision separation along the plan.'
    'ApertureCmd',       'double',  1,           'm',     'Jaw aperture the grasp needs, from the measured extent.'
    'AttachActorId',     'uint8',   1,           '',      'Actor the plan intends to capture.'
    'PlanTime',          'double',  1,           's',     'Wall-clock screening and planning time.'
    'Truncated',         'boolean', 1,           '',      'True when the plan exceeded a bus capacity.'
    'FailureCode',       'uint8',   1,           '',      'cfg.fail.* for this plan.'
    'Timestamp',         'double',  1,           's',     'Simulation time the plan was produced.'
    'Valid',             'boolean', 1,           '',      'False when no collision-free plan was found.'});

%% Supervisory command and status pairs
% ONE PAIR PER SUBORDINATE, AND NOTHING ELSE CROSSES. The planner writes a
% *Command and reads the matching *Status; it never reads a subordinate's plan,
% mode variable or internal timer. That is what keeps each chart replaceable.
%
% WHY EVERY COMMAND CARRIES A RequestId AND EVERY STATUS ECHOES IT BACK. A bare
% done flag cannot distinguish "finished the request I just issued" from "still
% holding the done of the previous one", and the difference decides whether the
% planner advances or waits. So the planner stamps each command with a
% monotonically increasing id, the subordinate copies that id into
% ServedRequestId the moment it accepts the command, and the planner treats
% State only in conjunction with ServedRequestId == the id it issued. This is why
% a scalar flag interface was rejected.
%
% Cancel and Stop are separate fields, not two values of Request, because both
% must be actionable while a Request is still in flight: Cancel abandons the
% current request and returns to idle, Stop is the e-stop path to a safe terminal
% mode and cannot be cleared by the planner.
nCause = cfg.sup.cause.nCauses;

baseCommand = makeBus('BaseCommand', ...
    'TaskPlanner -> BaseSupervisor. The only signal the planner sends the base.', { ...
    'Request',       'uint8',   1,       '',        'cfg.sup.baseReq.* verb.'
    'RequestId',     'uint32',  1,       '',        'Monotonic id this command must be answered against.'
    'GoalPose',      'double',  [3 1],   'm,m,rad', 'Target base pose in the world frame.'
    'GoalTolerance', 'double',  [3 1],   'm,m,rad', 'Per-axis at-goal band, from cfg.plan.baseGoalTolerance.'
    'CarriedExtent', 'double',  [3 1],   'm',       'Extent of whatever the gripper holds, for map inflation.'
    'Cancel',        'boolean', 1,       '',        'Abandon the in-flight request and return to idle.'
    'Stop',          'boolean', 1,       '',        'Latched e-stop; drive to the safe terminal mode.'
    'Timestamp',     'double',  1,       's',       'Simulation time the command was issued.'
    'Valid',         'boolean', 1,       '',        'False before the planner has issued anything.'
    'AltGoalPose',   'double',  [3 1],   'm,m,rad', 'Same position as GoalPose at a roughly opposite heading that passes the same footprint and arm-reach screen (hmmStandoffPartner), or GoalPose itself when none does. The planner plans both and keeps the cheaper drive; see BasePlan.GoalPose.'});

baseStatus = makeBus('BaseStatus', ...
    'BaseSupervisor -> TaskPlanner and SafetyMonitor. Deliberately small.', { ...
    'Mode',            'uint8',   1, '',    'cfg.sup.base.* mode the chart is in.'
    'State',           'uint8',   1, '',    'cfg.sup.st.* against ServedRequestId.'
    'ServedRequestId', 'uint32',  1, '',    'BaseCommand.RequestId this status answers.'
    'DistanceToGoal',  'double',  1, 'm',   'Planar distance from the measured pose to the goal.'
    'HeadingError',    'double',  1, 'rad', 'Wrapped heading error to the goal.'
    'CrossTrackError', 'double',  1, 'm',   'Distance from the measured pose to the reference pose.'
    'Stalled',         'boolean', 1, '',    'Commanded to move but measured speed below cfg.track.stallSpeed.'
    'ReplanRequest',   'boolean', 1, '',    'The base wants a new plan; the planner decides whether to grant it.'
    'PlanAttempts',    'uint8',   1, '',    'Plan requests spent on the current command.'
    'FailureCode',     'uint8',   1, '',    'cfg.fail.* when State is FAILED.'
    'Timestamp',       'double',  1, 's',   'Simulation time this status was produced.'
    'Valid',           'boolean', 1, '',    'False before the chart has run.'});

armCommand = makeBus('ArmCommand', ...
    'TaskPlanner -> ArmSupervisor.', { ...
    'Request',       'uint8',   1, '',    'cfg.sup.armReq.* verb.'
    'RequestId',     'uint32',  1, '',    'Monotonic id this command must be answered against.'
    'ArmMode',       'uint8',   1, '',    'cfg.plan.ARM_MODE_PICK or ARM_MODE_PLACE for the planning service.'
    'ApproachDepth', 'double',  1, 'm',   'Length of the constrained Cartesian segment.'
    'Cancel',        'boolean', 1, '',    'Abandon the in-flight request and return to idle.'
    'Stop',          'boolean', 1, '',    'Latched e-stop; drive to the safe terminal mode.'
    'Timestamp',     'double',  1, 's',   'Simulation time the command was issued.'
    'Valid',         'boolean', 1, '',    'False before the planner has issued anything.'});

armStatus = makeBus('ArmStatus', ...
    'ArmSupervisor -> TaskPlanner and SafetyMonitor.', { ...
    'Mode',             'uint8',   1,     '',      'cfg.sup.arm.* mode the chart is in.'
    'State',            'uint8',   1,     '',      'cfg.sup.st.* against ServedRequestId.'
    'ServedRequestId',  'uint32',  1,     '',      'ArmCommand.RequestId this status answers.'
    'JointError',       'double',  1,     'rad',   'Infinity-norm joint error against the interpolated reference.'
    'TrackingError',    'double',  1,     'm',     'Tool position error against the reference pose.'
    'ToolTranslation',  'double',  [3 1], 'm',     'Measured tool origin in the world frame.'
    'ApertureCmd',      'double',  1,     'm',     'Jaw aperture the accepted plan asks for; relayed, not acted on.'
    'AttachActorId',    'uint8',   1,     '',      'Actor the accepted plan intends to capture; relayed.'
    'PlanAttempts',     'uint8',   1,     '',      'Plan requests spent on the current command.'
    'FailureCode',      'uint8',   1,     '',      'cfg.fail.* when State is FAILED.'
    'Timestamp',        'double',  1,     's',     'Simulation time this status was produced.'
    'Valid',            'boolean', 1,     '',      'False before the chart has run.'});

gripperCommand = makeBus('GripperCommand', ...
    'TaskPlanner -> GripperSupervisor.', { ...
    'Request',        'uint8',   1, '',  'cfg.sup.gripReq.* verb.'
    'RequestId',      'uint32',  1, '',  'Monotonic id this command must be answered against.'
    'TargetAperture', 'double',  1, 'm', 'Commanded jaw aperture, derived from the measured extent.'
    'AttachActorId',  'uint8',   1, '',  'Actor the grasp should capture; 0 for none.'
    'Cancel',         'boolean', 1, '',  'Abandon the in-flight request and return to idle.'
    'Stop',           'boolean', 1, '',  'Latched e-stop; drive to the safe terminal mode.'
    'Timestamp',      'double',  1, 's', 'Simulation time the command was issued.'
    'Valid',          'boolean', 1, '',  'False before the planner has issued anything.'});

gripperStatus = makeBus('GripperStatus', ...
    'GripperSupervisor -> TaskPlanner and SafetyMonitor.', { ...
    'Mode',            'uint8',   1, '',  'cfg.sup.grip.* mode the chart is in.'
    'State',           'uint8',   1, '',  'cfg.sup.st.* against ServedRequestId.'
    'ServedRequestId', 'uint32',  1, '',  'GripperCommand.RequestId this status answers.'
    'Aperture',        'double',  1, 'm', 'Measured jaw aperture.'
    'ApertureError',   'double',  1, 'm', 'Commanded minus measured aperture.'
    'Grasped',         'boolean', 1, '',  'Plant confirmed capture and it has held for graspSettle.'
    'Slipped',         'boolean', 1, '',  'Capture was confirmed and then lost without a release request.'
    'FailureCode',     'uint8',   1, '',  'cfg.fail.* when State is FAILED.'
    'Timestamp',       'double',  1, 's', 'Simulation time this status was produced.'
    'Valid',           'boolean', 1, '',  'False before the chart has run.'});

%% SafetyStatus - one latched broadcast, four recipients
% The individual flags are carried alongside EStop and Cause because a latch
% reports only the FIRST cause, and a reader diagnosing a run needs to see what
% else was out of limits when it fired. There is no clearance flag: the range scan it read
% was cast through the planner's own map and could never fire, and it was removed with
% the scan on 2026-10-19.
safetyStatus = makeBus('SafetyStatus', ...
    'SafetyMonitor -> every chart. EStop is LATCHED: once true it stays true.', { ...
    'EStop',              'boolean', 1, '',  'Latched. Every chart has a defined safe terminal mode for this.'
    'Cause',              'uint8',   1, '',  'cfg.sup.cause.* of the FIRST violation seen; latched with EStop.'
    'JointLimit',         'boolean', 1, '',  'An arm joint is outside armJointLimits.'
    'JointVel',           'boolean', 1, '',  'An arm joint speed exceeds cfg.arm.jointVelMax.'
    'JointAccel',         'boolean', 1, '',  'An arm joint acceleration exceeds cfg.arm.jointAccMax.'
    'Workspace',          'boolean', 1, '',  'The tool is outside cfg.percep.sceneBounds or beyond cfg.safety.toolReachMax.'
    'BaseVel',            'boolean', 1, '',  'A base twist channel exceeds its cfg.base.*Max limit.'
    'BaseAccel',          'boolean', 1, '',  'A base twist channel accelerates beyond its cfg.base.*Max limit.'
    'StaleState',         'boolean', 1, '',  'RobotState older than cfg.safety.staleTimeout.'
    'StalePlan',          'boolean', 1, '',  'A subordinate status older than cfg.safety.staleTimeout.'
    'StaleProvider',      'boolean', 1, '',  'A pending grounding request older than cfg.provider.timeout.'
    'Timestamp',          'double',  1, 's', 'Simulation time this status was produced.'
    'Valid',              'boolean', 1, '',  'False before the monitor has run.'});

%% TaskResult - the planner's report on the whole instruction
% Per-action state is reported as ARRAYS rather than a single active-item
% summary, because the continue-on-failure policy means a run can end with a mix
% of DONE and FAILED items and "which ones, and why" is the answer an operator
% needs. Overflow is reported here as well as on the incoming ActionQueue: the
% queue records that grounding returned too many actions, this records that the
% planner never executed them.
taskResult = makeBus('TaskResult', ...
    'TaskPlanner -> observer. Per-action outcome for the submitted instruction.', { ...
    'TaskId',        'uint32',  1,     '',  'TaskRequest.TaskId this result answers.'
    'Outcome',       'uint8',   1,     '',  'cfg.sup.out.* for the whole instruction.'
    'PlannerMode',   'uint8',   1,     '',  'cfg.sup.pl.* the planner is in.'
    'NumActions',    'uint8',   1,     '',  'Actions accepted into the planner queue.'
    'NumDone',       'uint8',   1,     '',  'Actions that reached EXEC_DONE.'
    'NumFailed',     'uint8',   1,     '',  'Actions that reached EXEC_FAILED.'
    'ActiveIndex',   'uint8',   1,     '',  'Queue index being executed; 0 when none.'
    'ActiveVerb',    'uint8',   1,     '',  'cfg.queue.VERB_* of the active action.'
    'ActionState',   'uint8',   [nQ 1], '', 'cfg.queue.EXEC_* per queue slot.'
    'ActionFailure', 'uint8',   [nQ 1], '', 'cfg.fail.* per queue slot.'
    'Retries',       'uint8',   1,     '',  'Retries spent on the active action.'
    'Overflow',      'boolean', 1,     '',  'Grounding returned more actions than the queue holds.'
    'FailureCode',   'uint8',   1,     '',  'cfg.fail.* for the task as a whole.'
    'EStopCause',    'uint8',   1,     '',  'cfg.sup.cause.* when the task was aborted by the monitor.'
    'ElapsedTime',   'double',  1,     's', 'Time from task submission to this report.'
    'Timestamp',     'double',  1,     's', 'Simulation time this result was produced.'
    'Valid',         'boolean', 1,     '',  'False before a task has been submitted.'});

if nCause ~= double(cfg.sup.cause.STALE_PROVIDER)
    error('hmm:buses:causeCount', ...
        ['cfg.sup.cause.nCauses is %d but the highest declared cause is %d. ' ...
         'SafetyMonitor sizes its cause-priority vector from nCauses, so the ' ...
         'two must agree or a cause becomes unreportable.'], ...
        nCause, double(cfg.sup.cause.STALE_PROVIDER));
end

buses = struct( ...
    'RayHits',          rayHits, ...
    'CameraFrame',      cameraFrame, ...
    'RobotState',       robotState, ...
    'GripperState',     gripperState, ...
    'GraspCandidate',   graspCandidate, ...
    'TaskRequest',      taskRequest, ...
    'ActionQueue',      actionQueue, ...
    'ActionItem',       actionItem, ...
    'PerceptionStatus', perceptionStatus, ...
    'MovableRegistry',  movableRegistry, ...
    'ObjectTarget',     objectTarget, ...
    'PlaceTarget',      placeTarget, ...
    'BasePlan',         basePlan, ...
    'ArmPlan',          armPlan, ...
    'BaseCommand',      baseCommand, ...
    'BaseStatus',       baseStatus, ...
    'ArmCommand',       armCommand, ...
    'ArmStatus',        armStatus, ...
    'GripperCommand',   gripperCommand, ...
    'GripperStatus',    gripperStatus, ...
    'SafetyStatus',     safetyStatus, ...
    'TaskResult',       taskResult);

names = fieldnames(buses);
for k = 1:numel(names)
    assignin('base', names{k}, buses.(names{k}));
end

end

%% ------------------------------------------------------------------------
function bus = makeBus(name, description, spec)
%MAKEBUS Build a Simulink.Bus from an Nx5 cell {name type dims unit description}.
%   The unit column is folded into the element Description rather than set on
%   the Unit property. Several of these quantities are mixed-unit vectors
%   ('m,m,rad' for a planar pose) or pixel counts, and neither is a valid
%   Simulink unit expression, so setting Unit would fail on exactly the
%   elements whose units most need documenting.
bus = Simulink.Bus;
bus.Description = description;
elems = Simulink.BusElement.empty(0, 1);
for k = 1:size(spec, 1)
    e = Simulink.BusElement;
    e.Name           = spec{k, 1};
    e.DataType       = spec{k, 2};
    e.Dimensions     = spec{k, 3};
    e.DimensionsMode = 'Fixed';
    if isempty(spec{k, 4})
        e.Description = spec{k, 5};
    else
        e.Description = sprintf('[%s] %s', spec{k, 4}, spec{k, 5});
    end
    elems(k, 1) = e;
end
bus.Elements = elems;
assignin('base', name, bus);
end
