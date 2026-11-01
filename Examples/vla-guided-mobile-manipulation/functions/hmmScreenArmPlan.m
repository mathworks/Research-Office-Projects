function aplan = hmmScreenArmPlan(objectTarget, placeTarget, mode, robotConfig, ...
                                 tfArmFromMap, graspConfigs, poseErrors, manipulability, ...
                                 graspPoses, requestId, simTime, planTemplate)
%HMMSCREENARMPLAN Screen the grasp candidates and plan the arm motion to the winner.
%
%   aplan = HMMSCREENARMPLAN(objectTarget, placeTarget, mode, robotConfig,
%       tfArmFromMap, graspConfigs, poseErrors, manipulability, graspPoses,
%       requestId, simTime, planTemplate)
%
%   The candidates arrive ALREADY SOLVED. Each Candidate_k subsystem inside
%   CandidateIK holds an Inverse Kinematics block, a Get Jacobian block and a
%   Reciprocal Condition block, so this function receives, per candidate, a
%   configuration, the solver's residual and a manipulability index. It does not
%   solve kinematics and it does not compute a Jacobian - those are dedicated
%   toolbox blocks in the diagram, which is where they belong.
%
%   What is left is what has no block: screening, collision rejection, the
%   sampling-based path search, time parameterisation, and packing the result.
%
%   tfArmFromMap IS MAP -> ARM, AND THE DIRECTION IS NOT A DETAIL. It is the 4x4
%   that takes a pose expressed in the world frame and re-expresses it in the frame
%   the Inverse Kinematics block solves in, which is hmmArmCollisionModel's
%   .planBaseFrame. Note that getTransform(robot, q, 'base_link', 'map') returns the
%   OPPOSITE transform - the pose of the arm in the world - so a caller building
%   this argument from getTransform must invert it. graspPoses arrive in the same
%   arm frame, because that is the frame the IK block was given them in, and
%   GraspPose is reported back in world coordinates because nothing outside the arm
%   has any reason to know that frame exists.
%
%   SCREENING ORDER, cheapest test first:
%       1  residual        poseErrors(k) <= cfg.plan.poseErrorMax, else Unreachable
%       2  conditioning    manipulability(k) >= cfg.plan.manipulabilityMin
%       3  collision       checkCollision on the FULL 15-DOF tree, else CollisionReject
%   Conditioning then chooses the FIELD and continuity chooses the winner: every
%   survivor within cfg.plan.candidateTieBand of the best conditioning is admissible,
%   and among those the one needing the least joint travel - the SUM of the six joint
%   motions, not the largest of them - from the arm's measured configuration wins. The
%   sum and not the max because the candidates differ in ONE joint, so the max is
%   routinely set by a joint they all share and is then blind to the only joint being
%   chosen between; see the note on the metric at the selection loop. Conditioning
%   rather than residual, because two candidates that
%   both hit the pose to within a fraction of a millimetre are equally correct, and the
%   one further from a singularity is the one that can still be corrected once it gets
%   there. Continuity on top of it, because the three candidates are one approach at
%   three wrist yaws: on an object with no preferred grasp axis their conditioning is
%   near identical, so a bare argmax let the candidate solvers' random restarts decide,
%   and re-planning an UNCHANGED target rotated the wrist by the pi/3 candidate gap.
%   Nothing is weakened - the band only reorders candidates that already passed all
%   three gates, and at 1.0 it reduces to the argmax it replaces.
%
%   THE REPORTED FAILURE DISTINGUISHES WHY, AT EVERY GATE. If no candidate reached
%   the pose at all the answer is Unreachable; if some reached it and every one of
%   those collided the answer is CollisionReject. Those call for different responses
%   - move the base versus try a different approach - so they are not merged. Past
%   the screening the five remaining rejections are separated the same way:
%
%       ArmRrtFailed        the sampling search found no path stow -> grasp
%       ArmSelfCollide      a path waypoint or midpoint folds the arm into itself
%       ArmTimeParamFailed  no parameterisation exists under the joint vel/acc boxes
%       ArmRefOutOfBox      the fitted spline leaves the robot's position box
%       ArmRefCollide       the parameterised reference drives into the surface or floor
%       ArmRefWorkspace     the reference leaves SafetyMonitor's reach or scene bounds
%
%   ArmRrtFailed and ArmSelfCollide are reported only after cfg.plan.armPlanAttempts
%   re-seeded searches have each failed the gate, not after one. The search is told to
%   ignore self-collision - it must be, or the artefact meshes make the start pose
%   illegal to it - so it cannot aim at the constraint that judges its output, and a
%   single draw against that gate is a coin flip rather than a plan.
%
%   All five used to report the single code ArmPlanFailed, and that cost a whole
%   diagnosis: a failing run was read as "the planner refused" when the arm had in
%   fact planned and moved and the SERVO's tracking guard had aborted it. One code
%   for five causes cannot be debugged, only guessed at.
%
%   THE OBJECT IS NOT AN OBSTACLE ON THE PICK. The arm plan's environment is the
%   measured support surface and the floor. A planner cannot be asked to reach a thing
%   while treating that thing as something to avoid, and whether the grasp actually
%   captured the object is the plant's decision through its capture gate, not the
%   planner's. The FLOOR is an obstacle for the opposite reason: it is not negotiable,
%   it is not measured, and leaving it out let the planner ship a path that put the
%   tool 0.18 m underground.
%
%   ON THE PLACE IT IS PART OF THE ROBOT, WHICH IS A DIFFERENT STATEMENT AND WAS
%   MISSING. Once the object is in the gripper it is the part of the system nearest the
%   table, and every check here screened the arm alone: measured on the shipped run the
%   carried mug's bounding box reached 44.2 mm below the fitted place surface as the arm
%   swung in, and every gate passed - correctly, because nothing in the model knew the mug
%   was there. So a place plan builds the held object as a fixed-joint BODY on the tool
%   frame, sized from ObjectTarget.Extent, and it is then screened everywhere the arm is.
%   A body and not a world obstacle because the Sim3D attach is rigid, so the object turns
%   with the wrist. See withHeldObject below, and cfg.plan.surfaceBoxDepth for the other
%   half of the same defect - a support slab thin enough to pass through.
%
%   THE PARAMETERISED TRAJECTORY IS RE-CHECKED. contopptraj fits a smooth curve
%   through the waypoints, and the curve is not the piecewise-linear path
%   manipulatorRRT validated. Every sample is therefore collision-checked again,
%   and a plan whose reference is not collision-free is reported failed.
%
%   THE FREE MOVE ENDS AT THE STANDOFF, NOT AT THE GRASP, and that is a change from
%   how this function used to work. The search used to be planned to graspConfig
%   itself, so the arm flew from stow all the way to the grasp, the task planner then
%   backed the tool out cfg.sup.approachDepth = 0.12 m, and the very next phase drove
%   it 0.12 m straight back in. The round trip is ramped at ArmSupervisor's
%   approachRate and cost about 8.2 s per pick and the same again per place, to arrive
%   exactly where the free move had already been. Planning to the standoff removes
%   both halves of it and leaves the approach as the only motion along the approach
%   axis - which is what makes the approach a guarded, constrained move rather than a
%   formality.
%
%   PregraspConfig IS SOLVED HERE, and is no longer the planner's penultimate
%   waypoint. That waypoint was whatever distance the randomised tree happened to
%   leave between its last two states - it was never 0.12 m along the approach axis -
%   and nothing consumed it, so the discrepancy was invisible. It is now a seeded
%   Inverse Kinematics solve for GraspPose * trvec2tform([0 0 -approachDepth]), the
%   same offset convention ArmControl/ApproachTarget builds from ApproachOffset, so
%   the pose the planner arrives at and the pose the approach ramps away from are the
%   same pose by construction rather than by coincidence.
%
%   SEEDED WITH graspConfig, which is what makes the approach short. The standoff is
%   80 to 240 mm from a pose already solved, so the grasp configuration is a good guess
%   and the solver stays on its branch; an unseeded solve can return a kinematically
%   identical pose on a different elbow, and then the approach becomes a
%   reconfiguration of the whole arm. It is then unwound toward graspConfig for the
%   same reason, not toward stow.
%
%   AND IT IS SCREENED, because a standoff pose can be unreachable or in collision
%   even when the grasp is neither - every millimetre of standoff is a millimetre
%   closer to the reach bound, which is why the pick searches DOWN the ladder from its
%   ceiling rather than assuming the top rung. That is reported as
%   cfg.fail.PregraspUnreachable rather than
%   Unreachable, which would claim the object cannot be grasped when the grasp has
%   just been proved reachable, collision-free and well conditioned.
%
%   ONE REQUEST IS SOLVED ONCE. The caller is an Enabled Subsystem, so it runs on
%   every tick for as long as the enable is held, and planTemplate is the previous
%   answer fed back through a Unit Delay. A template already stamped with this
%   RequestId is therefore returned unchanged - see the memoisation guard below.
%
%   See also hmmArmCollisionModel, manipulatorRRT, contopptraj, checkCollision.

%   Copyright 2026 The MathWorks, Inc.

cfg   = cachedConfig();

%% Memoisation, before anything is built
% MotionPlanning is enabled, not triggered: it executes on every tick while the
% supervisor holds the enable high, and one arm plan costs ~3 s of wall clock. So
% the request is answered once and then held. Timestamp > 0 is what separates a
% real answer from the zeroed ground value, whose RequestId is also 0 before the
% first request ever arrives. FAILURES ARE MEMOISED TOO: none of the inputs
% changed, so re-solving would fail again, at the same price.
if planTemplate.RequestId == uint32(requestId) && planTemplate.Timestamp > 0
    aplan = planTemplate;
    return
end

model = hmmArmCollisionModel();

% THE SELF-COLLISION TREES ARE PINNED HERE, BEFORE ANYTHING CAN BE ADDED TO THE OTHERS.
% withHeldObject re-points .fullTree and .planTree at augmented copies carrying the carried
% object; these two keep the trees whose body ORDER model.fullArtefact and model.planArtefact
% were measured against. Every world check uses the augmented trees and every self check uses
% these, which is the whole of the fix described in withHeldObject's header.
model.fullTreeSelf = model.fullTree;
model.planTreeSelf = model.planTree;

aplan = blankPlan(planTemplate, cfg, requestId, simTime);
t0 = tic;

%% Which target, and is it vouched for
isPlace  = mode == cfg.plan.ARM_MODE_PLACE;
isStow   = mode == cfg.plan.ARM_MODE_STOW;
carrying = isPlace;

if isStow
    % A STOW IS PLANNED LIKE ANY FREE MOVE, against the furniture of the surface it leaves. It
    % used to be a bare joint-space command (ArmSupervisor/Stowing, StowSelect), screened against
    % nothing, and from a reach over a 0.80-0.90 m top the straight joint line drops the gripper
    % through the furniture's front - 191 mm inside the bedroom chest, 73 mm under the kitchen
    % counter (2026-10-12). See stowSurface for which surface and whether the hand is full.
    [zSurface, targetXY, carrying] = stowSurface(objectTarget, placeTarget, robotConfig);
elseif isPlace
    if ~placeTarget.Valid
        aplan.FailureCode = firstNonOk(placeTarget.FailureCode, cfg);
        aplan.PlanTime = toc(t0);
        return
    end
    zSurface = placeTarget.Point(3);
    targetXY = placeTarget.Point(1:2);
else
    if ~objectTarget.Valid
        aplan.FailureCode = firstNonOk(objectTarget.FailureCode, cfg);
        aplan.PlanTime = toc(t0);
        return
    end
    % Position IS the support surface, so there is nothing to subtract. It is worth saying
    % why, because this line used to subtract Extent(3) and that cost 90 mm in the one place
    % it could not be noticed. The ceiling rig's ray grid samples every 130 mm at 3 m against
    % a 101 mm mug, so fewer than one ray lands on the object and the cluster whose centroid
    % becomes Position is a patch of TABLE - which it measures well, the table being far
    % larger than a ray cell. Subtracting an object height from a tabletop drew the collision
    % surface 90 mm BELOW the real one, and a box in the wrong place cannot object: the grasp
    % pose it was screening sat inside the physical table and passed. See the Position
    % contract in hmmReconstructTargets.
    zSurface = objectTarget.Position(3);
    targetXY = objectTarget.Position(1:2);
end

%% Environment: the measured support surface, built TWICE
% One patch of geometry, two frames. checkCollision poses a world object in the
% base frame of the tree it is given, and the two trees have different base
% frames: fullTree is rooted at 'map' and planTree at the arm mounting frame. The
% same box therefore needs two poses, and using one for both would check the
% screening candidates against a surface displaced by the whole base transform.
% The side is the target's own footprint plus a margin on each side, clamped. See
% cfg.plan.surfaceBoxSideLim for why a patch the size of a table is not an option.
% Extent(1:2) are FULL horizontal spans, so the footprint is the larger of them and
% needs no doubling; the clamp is what actually bounds the result either way, but a
% formula that says footprint-plus-margin should compute footprint-plus-margin.
side = max(objectTarget.Extent(1:2)) + 2 * cfg.plan.surfaceBoxMargin;
side = min(max(side, cfg.plan.surfaceBoxSideLim(1)), cfg.plan.surfaceBoxSideLim(2));
% DEPTH, NOT THICKNESS, AND THE DIFFERENCE IS WHAT A TABLE IS. The slab used to be
% cfg.plan.surfaceBoxThickness = 20 mm deep, which models a sheet of glass floating in
% space: anything passing more than 20 mm below the tabletop came out the other side into
% free space and the check cleared it. Furniture is not like that - the volume under a
% tabletop belongs to the table. Measured on the shipped run the carried mug dipped 44.2 mm
% below the place surface as it swung in, which is through the slab and out. See
% cfg.plan.surfaceBoxDepth for why the depth is bounded rather than run to the floor.
boxSize = [side, side, cfg.plan.surfaceBoxDepth];

surfTop = zSurface - cfg.gripper.padCompression;   % touching the surface is not a collision
boxWorld = trvec2tform([targetXY(1), targetXY(2), surfTop - 0.5 * boxSize(3)]);

surfaceFull = collisionBox(boxSize(1), boxSize(2), boxSize(3));
surfaceFull.Pose = boxWorld;                       % fullTree is rooted at 'map'
envFull = {surfaceFull};

surfacePlan = collisionBox(boxSize(1), boxSize(2), boxSize(3));
surfacePlan.Pose = tfArmFromMap * boxWorld;        % planTree is rooted at the arm

% AND THE FLOOR, WHICH THE PLANNER USED TO HAVE NO IDEA ABOUT. The arm plan's world was
% the support surface and nothing else, so a path that swung the tool down past the
% floorboards was collision-free as far as manipulatorRRT could tell - and it planned
% exactly that, driving the tool to z = -0.18 m through a floor that is not optional.
% SafetyMonitor/WorkspaceCheck then latched a WORKSPACE E-stop, correctly, three and a
% half seconds into an otherwise healthy free move. The planner and the guard disagreed
% about what was legal, and when they disagree the guard always wins at the worst moment.
%
% A COLLISION BOX RATHER THAN A POST-CHECK, so the SEARCH avoids the floor instead of the
% screening rejecting a path that has already been found. Rejection would turn one wasted
% plan into two wasted plans and then a failed task; an obstacle makes the sampler route
% around it, which is what a planner is for.
%
% ONLY IN THE PLAN FRAME. envFull is checked against fullTree, which is rooted at 'map'
% and CONTAINS THE CHASSIS - whose wheels rest on the floor by definition. A floor in
% envFull would report every grasp candidate as colliding and nothing would ever plan.
%
% The top face sits at world z = 0, not at the guard's own bound of
% cfg.percep.sceneBounds(3,1) = -0.05 m. The 50 mm between them is deliberate: this box
% keeps the search clear, and the per-sample check further down enforces the guard's exact
% bound. A margin in the search and an exact test at the gate, the same division of labour
% as cfg.arm.limitMargin against model.armPosBox.
floorSide = 2 * (cfg.arm.reach + cfg.plan.surfaceBoxMargin);
floorThk  = cfg.plan.surfaceBoxThickness;
armInWorld = tfArmFromMap \ eye(4);
floorWorld = trvec2tform([armInWorld(1, 4), armInWorld(2, 4), -0.5 * floorThk]);
floorPlan = collisionBox(floorSide, floorSide, floorThk);
floorPlan.Pose = tfArmFromMap * floorWorld;

envPlan = {surfacePlan, floorPlan};

%% AND THE COUNTER, WHICH THE SLAB ABOVE IS NOT AND WAS NEVER MEANT TO BE
% The slab is the patch the object is being set down ON, sized from the OBJECT. It is
% the right shape for that and the wrong shape for a worktop, and the difference was
% measured rather than argued: on the 40 s run of 2026-09-30 the carried mug passed
% 85.6 mm INSIDE the kitchen counter, 0.348 m from the place point and therefore
% 0.258 m outside a slab whose half-side is at most 0.125 m, during the free move
% planned right here. Every gate in this function passed, correctly, because the only
% counter any of them had ever been shown was that slab. hmmSurfaceObstacles has the
% argument for where the geometry comes from instead. The short version is that the
% occupancy grid is the only thing in the project that knows where the counter is,
% and unlike a bigger slab it cannot grow backwards over the robot, because the base
% drove here through the same grid.
%
% envPlan ONLY, NEVER envFull, and this is the same asymmetry the floor has twenty
% lines up. envFull is checked against fullTree, which is rooted at 'map' and CONTAINS
% THE CHASSIS; furniture boxes in envFull would be tested against a chassis parked
% 0.42 m from the counter with a 0.32 m half-diagonal, and the first pose error or map
% coarseness that closed that gap would report every grasp candidate as colliding and
% nothing would ever plan. envPlan is checked against planTree, which is rooted at the
% arm mount and carries no chassis, and it is also the environment manipulatorRRT
% SEARCHES - so putting the counter here makes the sampler route around it instead of
% making the screening reject a path that has already been found.
%
% BOTH LEGS NOW, AND THE PICK WAS THE MEASUREMENT THAT SAID SO. This used to be place-only, on
% the argument that the pick's hand is empty and its free move already worked. It worked on the
% side table because the stow pose sits above a 0.55 m top; on the 0.80 m bedroom chest
% (2026-10-12, Bottle) the gripper swung up from stow THROUGH the chest's front, up to 129 mm
% inside it on the way in and 191 mm on the way back, outside the object-sized slab above and
% under nothing else. The same slab doctrine applies unchanged - top at the support, bounded
% depth - and zSurface on a pick is ObjectTarget.Position(3), which is the support surface.
%
% A SLAB HANGING BELOW THE WORKTOP, NOT A COLUMN STANDING ON THE FLOOR, and this is
% not a refinement - a floor-to-top extrusion makes the place IMPOSSIBLE. Measured
% before running anything: with the boxes extruded 0 -> surfTop the stowed start
% configuration is still clear by 0.2895 m, but the pre-place GOAL collides -
% shoulder_link and upper_arm_link against the counter box, their frames at world
% z = 0.6066, minimum finite separation 0.1122 m. The arm reaches ACROSS the worktop
% from a mount 0.365 m outside its near edge, so its upper links are over the counter
% footprint by construction at any workable pose, and a box that fills the whole
% volume beneath the top catches them every time. A depth sweep over
% [0.06 0.08 0.10 0.12 0.15 0.20 0.25 0.897] returned a colliding goal at EVERY depth
% with minSep pinned at 0.1122 - the same number - because the links cross the box's
% TOP face, not its side, and lowering the floor of the box cannot change that.
% Anchoring the slab's top at surfTop and giving it cfg.plan.surfaceBoxDepth of depth
% instead leaves the links above it and still catches what the defect is about: the
% measured contact sits at z = 0.8114, only 0.0856 m below the fitted top of 0.8970,
% so 0.250 m contains it with room to spare. Verified at that depth with the held
% object present - the start is clear, 9 of 15 candidate pre-place poses across the
% yaw and standoff ladders this function already tries are collision-free, and
% manipulatorRRT found a path on 6 of 6 seeds in a median 0.55 s.
%
% THE UNMODELLED VOLUME IS THE CABINET BELOW THE WORKTOP, and leaving it out is the
% safe direction, the same argument hmmSurfaceObstacles makes about wall height. The
% arm has no business down there on a place - the held object is being lifted onto the
% top, not posted underneath it - so a path through that volume is a path this screen
% would not have rejected yesterday either. A box that is too TALL, by contrast, turns
% an avoided collision into a plan that cannot be made at all, which is what the sweep
% above measured.
%
% EXCEPT THAT surfTop IS NOT EVERY BOX'S TOP ANY MORE, and hanging a short prop from a
% tall worktop is the TALL error by another route. Once makeNavMap started marking the
% floor-standing props as occupied, their cells arrived here too: a box fitted to a
% 0.66 m stool, hung from a 0.91 m counter, spanned z = [0.660, 0.910] - a quarter of a
% metre of air resting on the seat, with the stool's real body below it still unmodelled
% (1 to 3 of the 6 to 12 boxes at each surviving kitchen arrival). hmmSurfaceObstacles
% now returns a TOP per box: NaN means "the grid knew only a footprint, hang it from the
% surface as before", and a finite value is a prop whose real top the manifest knows.
% Capping at min of the two keeps the doctrine intact - the top never rises above the
% support surface, the depth never reaches the floor - while the stool box becomes
% [0.410, 0.660] and stops standing on nothing.
if surfTop > 0.01
    [furnXY, furnTop, furnTall] = hmmSurfaceObstacles(armInWorld(1:2, 4)', ...
        robotConfig(1:2), robotConfig(3), cfg, surfTop);
    for iF = 1:size(furnXY, 1)
        topF = surfTop;
        if iF <= numel(furnTop) && ~isnan(furnTop(iF))
            topF = min(surfTop, furnTop(iF));
        end
        thkF = min(cfg.plan.surfaceBoxDepth, topF);         % never through the floor
        % FURNITURE RISING ABOVE THE SURFACE IS ITS OWN OBSTACLE, from the surface up to its
        % measured top - the couch backrest beside its seat, a headboard beside a bed. Capping it
        % at the surface let the gripper pass 0.10-0.12 m through the backrest (2026-10-16);
        % modelled, manipulatorRRT routes the arm over it instead.
        if iF <= numel(furnTall) && furnTall(iF)
            topF = furnTop(iF);
            thkF = max(topF - surfTop, 1 / double(cfg.map.resolution));
        end
        boxF = collisionBox(furnXY(iF, 3), furnXY(iF, 4), thkF);
        boxF.Pose = tfArmFromMap * ...
            trvec2tform([furnXY(iF, 1), furnXY(iF, 2), topF - 0.5 * thkF]);
        envPlan{end + 1} = boxF;   %#ok<AGROW> bounded by cfg.plan.surfaceObstacleMaxBoxes
    end
end

%% AND THE THING THE ARM IS CARRYING, WHICH WAS IN NONE OF THE CHECKS ABOVE
% Every collision test in this function screens the ROBOT. On a place the robot is holding
% something, and that something is the part of the system nearest the table: measured on the
% shipped run the mug's bounding box went 44.2 mm below the fitted place surface as the arm
% swung in, and every gate here passed, correctly, because no gate had ever been told the mug
% existed. The arm itself never touched anything. It was reported as "the mug looks like it
% crosses the table", which is exactly what it was.
%
% GENERIC, FROM ObjectTarget.Extent. Nothing here names a mug or reads the manifest. The
% object is whatever the instruction grounded, its size is what perception measured, and its
% ExtentSource says whether that was measured, taken from the registry or assumed - so an
% instruction to move some other prop puts THAT prop in the collision tree, at its own size,
% with no edit here. That was the explicit requirement.
%
% ON A PER-CALL COPY OF THE TREES, NEVER ON hmmArmCollisionModel'S. That function caches its
% model in an unkeyed persistent, so a body added to its tree would persist into every later
% call - including the next PICK, whose hand is empty, and the next task, whose object is a
% different size. rigidBodyTree is a handle class, so the copy is not optional and a plain
% struct assignment would not give one.
%
% PICKS GET NOTHING, because at pick time the hand is empty. The place plan is what covers
% the arm's motion while loaded: the pick ends at the grasp, the carry to the next room
% happens with the arm parked, and the free move from stow to the pre-place pose is planned
% here with the object in hand.
if carrying && objectTarget.Extent(3) > 0
    model = withHeldObject(model, objectTarget, cfg);
end

if isStow
%% A stow has no candidates and no standoff: its goal IS the stow configuration
% EXACTLY cfg.arm.stowConfig, NOT UNWOUND toward the arm. Idle commands that literal vector
% (StowSelect), so a goal one revolution away would be the same posture to the planner and a
% full-turn spin to the servo the moment the arm goes idle.
nArm      = double(cfg.arm.nJoints);
qArmNow   = reshape(robotConfig(4:3 + nArm), 1, []);
qGripZero = zeros(1, double(cfg.gripper.nJoints));
pregraspConfig = double(cfg.arm.stowConfig(:))';
graspConfig    = pregraspConfig(:);
aplan.GraspConfig    = graspConfig;
aplan.PregraspConfig = graspConfig;
aplan.ApproachDepth  = 0;
aplan.GraspPose      = tfArmFromMap \ getTransform(model.planTree, ...
    [pregraspConfig, qGripZero], 'tcp_frame');
else
%% Candidate screening
nC   = double(cfg.plan.nGraspCandidates);
nArm = double(cfg.arm.nJoints);

best      = 0;
anyReached = false;
survivor   = false(1, nC);

% THE ARM'S MEASURED CONFIGURATION, needed twice below - once to break ties between candidates
% and once to unwind the winner. The long note further down explains why the reference must be
% the measurement rather than cfg.arm.stowConfig; it is defined here because the tie-break is
% the earlier of the two consumers.
qArmNow = robotConfig(4:3 + nArm)';

for k = 1:nC
    if poseErrors(k) > cfg.plan.poseErrorMax || ~isfinite(poseErrors(k))
        continue
    end
    if manipulability(k) < cfg.plan.manipulabilityMin || ~isfinite(manipulability(k))
        % SOLVED BUT NEAR-SINGULAR IS A REACH FAILURE, NOT A COLLISION: the arm is at full stretch
        % and the answer is to stand closer. Counting it as reached reported the Bottle pick of
        % 2026-10-12 (rcond 0.00105 against 0.002 on all three candidates) as CollisionReject.
        continue
    end
    anyReached = true;

    qFull = robotConfig(:)';
    qFull(4:3 + nArm) = graspConfigs(1:nArm, k)';
    qFull(4 + nArm:end) = 0;      % gripper open; see hmmArmCollisionModel

    if collides(model.fullTree, model.fullTreeSelf, qFull, envFull, ...
            model.fullArtefact, model.fullNB)
        continue
    end

    survivor(k) = true;
end

% CONDITIONING CHOOSES THE FIELD; CONTINUITY CHOOSES THE WINNER. The three candidates are one
% approach at three wrist yaws, so on an object with no preferred grasp axis their conditioning
% is near identical and a bare argmax makes the winner a coin toss decided by the candidate
% solvers' random restarts. The wrist refinement re-enters PlanPick and re-solves the SAME
% target, so that toss happens twice: measured, joint 6 moved 1.0552 rad between two free-move
% legs of one pick while the tool travelled 4.2 mm and the goal did not change at all. The
% candidate yaw gap is pi/3 = 1.047198 rad - the two solves won with opposite candidates.
%
% SO AMONG CANDIDATES WITHIN cfg.plan.candidateTieBand OF THE BEST CONDITIONING, THE ONE NEEDING
% THE LEAST JOINT TRAVEL WINS. Nothing is weakened: every candidate here has already passed the
% residual gate, the absolute cfg.plan.manipulabilityMin floor and the collision check, and at a
% band of 1.0 this reduces exactly to the argmax it replaces.
%
% TRAVEL IS MEASURED AFTER UNWINDING, which is the only measure that means anything. A candidate
% returned a full revolution away is the SAME posture, and unwindToward is about to move it to
% the nearest feasible revolution anyway, so comparing raw solver output would rank candidates by
% which revolution their restarts happened to land on - reintroducing the arbitrariness this is
% here to remove. Three unwinds is a negligible cost once per plan.
%
% AND IT IS THE SUM OF THE JOINT MOTIONS, NOT THE LARGEST OF THEM, because the largest is BLIND TO
% THE ONE JOINT THIS IS SELECTING ON. The three candidates are one posture at three wrist yaws, so
% joints 1-5 are identical across them and whatever they contribute is the SAME number in every
% candidate's max. Measured on the re-plan of a pick: joints 1-5 needed 0.5948 rad, candidate 3
% needed 0.0396 rad of joint 6 and candidate 1 needed 0.4840 - both under 0.5948, so max() returned
% 0.5948 for BOTH, the comparison was strict, and the lower index won. The wrist rolled 27.8 deg
% for nothing and the tie-break that was installed to prevent exactly that could not see it. An
% infinity norm answers "which finishes soonest", which is a fair question and not this one; the
% question here is "which moves joints it does not need to move", and only a norm that is strictly
% monotone in every joint can answer it.
travelRec = inf(1, nC);
admitRec  = false(1, nC);
qUnwound  = nan(nArm, nC);
if any(survivor)
    mBest      = max(manipulability(survivor));
    bestTravel = inf;
    for k = find(survivor)
        if manipulability(k) < cfg.plan.candidateTieBand * mBest
            continue
        end
        admitRec(k) = true;
        qk     = unwindToward(model.planTree, graspConfigs(1:nArm, k)', ...
            qArmNow, model.armPlanBox);
        qUnwound(:, k) = qk(:);
        travel = sum(abs(qk(:) - qArmNow(:)));
        travelRec(k) = travel;
        if travel < bestTravel
            bestTravel = travel;
            best       = k;
        end
    end
end

% THE SELECTION TAP. ArmPlan is not in the logged set, so which candidate won and on what is
% invisible from a recording - and the candidate flip that motivated the tie-break above was
% diagnosed only by inferring the winner from joint 6 after the fact. This records the decision
% itself, in the same appdata style as hmmGroundingTap and hmmWristRefineTap, and it is the
% cheapest thing in this function by three orders of magnitude. It keeps every call of a run
% rather than the last, because the whole question is whether two calls AGREE.
rec = struct('RequestId', double(requestId), 'SimTime', double(simTime), ...
    'Mode', double(mode), 'QArmNow', qArmNow(:)', 'Manipulability', manipulability(:)', ...
    'PoseErrors', poseErrors(:)', 'Survivor', survivor, 'Admissible', admitRec, ...
    'Travel', travelRec, 'Best', best, 'QUnwound', qUnwound, ...
    'TieBand', double(cfg.plan.candidateTieBand), ...
    'QCandidates', double(graspConfigs(1:nArm, :)), 'RobotConfig', double(robotConfig(:)'));
prev = getappdata(0, 'hmmArmPlanTap');
if isempty(prev) || rec.SimTime < prev(end).SimTime
    setappdata(0, 'hmmArmPlanTap', rec);            % new run, or the first call of one
else
    setappdata(0, 'hmmArmPlanTap', [prev, rec]);
end

if best == 0
    if anyReached
        aplan.FailureCode = cfg.fail.CollisionReject;
    else
        aplan.FailureCode = cfg.fail.Unreachable;
    end
    aplan.PlanTime = toc(t0);
    return
end

% THE IK SOLUTION IS UNWOUND BEFORE ANYTHING IS PLANNED TO IT, and this is not a
% refinement - without it the arm plan fails outright. A revolute joint's kinematics are
% 2*pi periodic, so q and q + 2*pi*k put the tool in exactly the same place; but
% armJointLimits gives five of the six joints +-2*pi of travel, and the Inverse Kinematics
% blocks are free to return whichever revolution their restarts happen to land on. On the
% run that exposed this they landed badly:
%
%       joint          1       2       3       4       5       6     total
%       as solved   6.103  -5.504  -1.513   5.446  -1.571   4.413
%       travel      2.772   3.520   4.203   7.233   0.006   4.413   22.15 rad
%       unwound     6.103   0.779  -1.513  -0.837  -1.571  -1.870
%       travel      2.772   2.763   4.203   0.950   0.006   1.870   12.56 rad
%
% TWO JOINTS WERE DELIBERATELY LEFT WHERE THEY WERE, and reading the table without noticing
% that misreads what this function does. Joint 1's nearest revolution to the stow pose OF
% THAT RUN was the one the IK returned - round((3.331 - 6.103) / 2*pi) = 0 - so 6.103 was
% already optimal and stayed, which the run confirms by ending at 6.108. Joint 3 wants
% k = +1 and cannot have it: its band is +-pi rather than +-2*pi, so the only revolution
% that fits its box is the one it is already on, and the clamp below leaves it alone. The
% unwind is not a normalisation to a principal range; it is a per-joint
% nearest-feasible-revolution choice, and on that goal it moved three joints of the six.
%
% AND THE TABLE IS A RECORD, NOT A PREDICTION - THE STOW POSE IT WAS TAKEN AGAINST HAS SINCE
% MOVED. cfg.arm.stowConfig now parks joint 1 at 0 rather than at 3.331, so the same IK
% solution unwinds differently: round((0 - 6.103) / 2*pi) = -1, and joint 1 goes to -0.180
% instead of staying at 6.103. That is the point of unwinding TOWARD the stow pose rather
% than toward zero or toward a principal range - the reference is wherever the arm actually
% is - and it is most of why the forward stow shortens the reach at all. A worked example
% that names a configured value is only valid at that value; nothing here needs changing,
% but do not read the third row as what this code will return today.
%
% Joint 4 alone was asked to turn 7.2 rad - more than a full revolution - to reach a pose
% 0.95 rad away, and a 22 rad sweep of a 6-DOF arm passes through poses a 12.6 rad sweep
% does not. It passed through one of them: manipulatorRRT returned a 14-waypoint path whose
% eleventh waypoint folded base into shoulder_link, and since the search is told to ignore
% self-collision it could not know. Every seed and every retry failed the same way, because
% the cause was never the route - it was the DESTINATION being a full turn from home.
%
% Each joint is moved to the revolution nearest the stow configuration that still fits the
% planner's box. The grasp pose, the IK residual and the manipulability are all unchanged
% by construction, so nothing is traded away: this is a strictly shorter path to the same
% place. It is done here rather than in the diagram because a mask checkbox cannot express
% it and because the planner is the consumer that cares which revolution it was handed.
%
% THE REFERENCE IS THE ARM'S MEASURED CONFIGURATION, NOT cfg.arm.stowConfig. The paragraph
% above already says why - "the reference is wherever the arm actually is" - and for as long
% as a plan was only ever requested with the arm parked, the constant and the measurement were
% the same vector, so the constant read as harmless shorthand. It stopped being the same vector
% the moment the wrist refinement re-plans from the pre-descent pose. robotConfig has been an
% argument to this function since it was written and its arm slice is already used for the
% collision checks a few dozen lines above; this uses it here too. qArmNow is assigned up in the
% screening section, whose tie-break needs the same reference for the same reason.

graspConfig = unwindToward(model.planTree, graspConfigs(1:nArm, best)', ...
    qArmNow, model.armPlanBox);
graspConfig = graspConfig(:);
aplan.SelectedCandidate = uint8(best);
aplan.PoseErrorNorm     = poseErrors(best);
aplan.Manipulability    = manipulability(best);
aplan.GraspConfig       = graspConfig;

% GraspPose is reported in the WORLD frame. The candidate blocks build it in the
% arm base frame because that is the frame the Inverse Kinematics block solves
% in; a consumer outside the arm has no reason to know that frame exists.
aplan.GraspPose = tfArmFromMap \ graspPoses(:, :, best);

% THE RENDER'S ATTACH OFFSET IS FROZEN HERE, AND NOTHING BELOW READS IT BACK.
% Simulation 3D Robot discards the physical relative pose when the jaws close and
% replaces it with a block parameter, so that parameter has to be told what the
% relative pose is GOING to be. This is the only place in the project where the
% closure pose exists: the base has arrived, the candidate is chosen, the wrist yaw
% is decided and graspConfig is the configuration the descent ends at. Everything
% that used to supply that parameter - hmmSetup on load, the app on submit,
% hmmRefineService at the pregrasp - predicts it instead, from the manifest extent
% and an assumed canonical grasp yaw, and so carries both the perception error and
% up to 7.25 mm of yaw residual into the render as a visible jump.
%
% PICK ONLY, and an output of nothing. A place has no attach to arrange, and the
% value leaves hmmAttachOffset by set_param alone - the plan does not read it, is not
% changed by it, and is identical with the call removed. The prop's true pose is a
% plant-side fact (hmmSpawnProps authored it from the manifest) used for plant-side
% geometry; the hard rule that no true pose reaches grounding, ObjectTarget or this
% screen is intact, and hmmAttachOffset's header argues the boundary in full.
%
% FK ON planTree RATHER THAN aplan.GraspPose, which is a line above and would have
% been free: GraspPose is the pose the candidate blocks ASKED the IK for, and
% graspConfig is what the IK actually returned, poseErrors(best) away from it. The
% descent tracks the configuration, so the configuration is what the prop will see.
if ~isPlace
    hmmAttachOffset(cfg, tfArmFromMap \ getTransform(model.planTree, ...
        [graspConfig(:)', zeros(1, double(cfg.gripper.nJoints))], 'tcp_frame'), ...
        double(objectTarget.ActorId));
end

%% The pre-grasp standoff
% WHERE THE FREE MOVE SHOULD END. ArmControl/ApproachTarget builds its task-space
% target as T_grasp * trvec2tform([0 0 -ApproachOffset]), so the pose the approach
% ramps away from is the grasp pose translated backwards along the TOOL's own z axis.
% The same expression is used here rather than a world-frame lift, because "back along
% the approach" and "up" are only the same thing for a vertical grasp, and the grasp
% orientation comes from the candidate blocks' ApproachEuler, not from an assumption.
%
% IN THE ARM BASE FRAME, which is where graspPoses already lives and where
% inverseKinematics solves: model.planBaseFrame and model.planTree.BaseName are both
% arm_mount, so no change of frame is needed and none is done. aplan.GraspPose is the
% world-frame report a line above; this is deliberately not built from it.
qGripZero = zeros(1, double(cfg.gripper.nJoints));

% NO RANDOM RESTARTS, unlike the candidate solves. A restart's whole purpose is to
% escape the seed, and the seed is the point here - graspConfig is the answer we want
% to stay near. A restart that found a different branch would satisfy the residual
% check below and then make the approach a full reconfiguration of the arm.
ikPregrasp = inverseKinematics('RigidBodyTree', model.planTree);
ikPregrasp.SolverParameters.MaxIterations      = cfg.plan.ikMaxIterations;
ikPregrasp.SolverParameters.MaxTime            = cfg.plan.ikMaxTime;
ikPregrasp.SolverParameters.AllowRandomRestart = false;

% THE STANDOFF IS SEARCHED, NOT CONFIGURED, AND BOTH VERBS SEARCH - THEY JUST WANT OPPOSITE
% ENDS OF THEIR OWN LADDER. A pick wants the standoff as HIGH as the arm will hold it,
% because the wrist camera takes its one refining picture from this pose and at a short
% standoff the object overflows the frame - measured at 0.080 m, a 101 mm mug subtends
% ~480 px of a 360-row frame from 65 mm away, so there is no shape for a provider to point
% at. A PLACE wants the opposite: its pre-place pose is already the thing that pushes the arm
% off its reach ceiling, so it starts at cfg.sup.approachDepth and SHORTENS the guarded
% descent from there.
%
% THE PLACE USED TO TAKE ONE RUNG AND THAT IS THE DEFECT THIS LADDER REPAIRS. The sentence
% above - "its pre-place pose is already the thing that pushes the arm off its reach ceiling"
% - was correct, and the conclusion drawn from it was backwards: the verb most likely to be
% refused at its standoff was the one given no alternative. The 40 s integration run failed
% twice in one simulation, at 15.78 s and 33.68 s, with PregraspUnreachable on a place point
% the arm could reach with 100 mm of lever to spare; what it could not reach was the pose
% 80 mm above it, by 48 mm. hmmConfig's comment on cfg.sup.approachDepthPlaceLadder carries
% the measured rung-by-rung table and the reason its bottom is 0.030.
%
% AND NOTHING DOWNSTREAM NEEDED CHANGING, which is the test of whether the ladder belonged
% here. chosenDepth was already returned as aplan.ApproachDepth and already consumed by
% ArmControl/ApproachTarget, because the pick ladder had to thread it; the place simply
% stopped being the verb that always reported the configured value.
%
% HIGHEST THAT PASSES, NOT HIGHEST THAT SOLVES. Every rung is screened on the same four
% tests the grasp candidates passed: finite after unwinding, IK residual inside
% cfg.plan.poseErrorMax, conditioning at or above cfg.plan.manipulabilityMin, and
% collision-free against the surface and the floor. A residual of 1e-9 says the pose is
% solvable, not that it is a good place to begin a guarded descent from; near full
% extension the same joint rates that tracked well lower down leave a Cartesian error the
% servo cannot remove, and manipulabilityMin is the floor this project already states that
% in - see its comment in hmmConfig, and see cfg.sup.approachDepthPickMax's on why the
% standoff is screened on that same absolute floor rather than on a fraction of the grasp's.
%
% WHY NEITHER LADDER CAN END IN A NEW FAILURE. Both are anchored to cfg.sup.approachDepth,
% the single value this function used before either search existed - the pick descends TO it
% and the place descends FROM it - so on the pick the last rung is the old behaviour and on
% the place the FIRST rung is. Exhausting either ladder therefore reports the case
% PregraspUnreachable always meant, and no rung on either is a standoff the old code would
% have accepted but these reject.
if isPlace
    ladder = cfg.sup.approachDepthPlaceLadder(:)';
else
    ladder = cfg.sup.approachDepthLadder(:)';
end

pregraspConfig = [];
chosenDepth    = cfg.sup.approachDepth;

for depth = ladder
    pregraspPose = graspPoses(:, :, best) * trvec2tform([0 0 -depth]);

    [qPregrasp, pregraspInfo] = ikPregrasp(model.toolFrame, pregraspPose, ...
        cfg.plan.ikWeights, [graspConfig(:)', qGripZero]);

    % Unwound toward the GRASP and not toward stow, which is the opposite of the choice
    % made for graspConfig above and for the opposite reason: graspConfig is the end of a
    % long searched path from stow, so it wants to be near stow; this is the start of a
    % short unsearched move to graspConfig, so it wants to be near graspConfig.
    qCand = unwindToward(model.planTree, qPregrasp(1:nArm), ...
        graspConfig(:)', model.armPlanBox);

    if ~all(isfinite(qCand)) || pregraspInfo.PoseErrorNorm > cfg.plan.poseErrorMax
        continue
    end

    % THE CONDITIONING TEST IS THE CHEAPER OF THE TWO REMAINING, so it runs before the
    % collision check: a rung rejected for extension never pays for checkCollision.
    if manipAt(model.planTree, [qCand, qGripZero], model.toolFrame) < ...
            cfg.plan.manipulabilityMin
        continue
    end

    if collides(model.planTree, model.planTreeSelf, [qCand, qGripZero], envPlan, ...
            model.planArtefact, model.planNB)
        continue
    end

    pregraspConfig = qCand;
    chosenDepth    = depth;
    break
end

% Screening here rather than letting the search below fail distinguishes "no standoff is
% solvable" from "no route to one", which is what ArmRrtFailed means.
if isempty(pregraspConfig)
    aplan.FailureCode = cfg.fail.PregraspUnreachable;
    aplan.PlanTime = toc(t0);
    return
end

aplan.PregraspConfig = pregraspConfig(:);
aplan.ApproachDepth  = chosenDepth;
end   % isStow

%% Search stow -> pre-grasp
planner = manipulatorRRT(model.planTree, envPlan);
planner.MaxIterations          = cfg.plan.armRrtMaxIterations;
planner.MaxConnectionDistance  = cfg.plan.armRrtMaxConnDist;
planner.ValidationDistance     = cfg.plan.armRrtValidationDist;
planner.EnableConnectHeuristic = true;
% Self-collision checking is done by this function against the artefact mask.
% The planner's own check cannot be used because it has no way to exclude the
% mesh artefacts and so rejects the start configuration outright.
planner.IgnoreSelfCollision    = true;

% THE GOAL IS THE STANDOFF, NOT THE GRASP. graspConfig is still the plan's grasp
% configuration and still what ApproachTarget builds T_grasp from; it is simply not
% where this search stops. The last ApproachDepth metres belong to the approach phase,
% which drives it in task space under the tracking guard.
% THE START IS WHERE THE ARM IS. This line read [cfg.arm.stowConfig, qGripZero] and that
% was a latent defect the wrist refinement turned into a task failure, so it is worth
% recording precisely what it cost rather than just correcting it.
%
% Every A_PLAN_PICK before the refinement existed was issued with the arm parked, so a
% hard-coded stow start was indistinguishable from the measurement and nothing ever caught
% it. The refinement re-issues A_PLAN_PICK from the PRE-DESCENT pose. The planner then
% searched from stow, ArmSupervisor's JointSpaceTrack began replaying that path from its
% first waypoint - stow - while the arm was physically at the standoff, and the reference
% jumped. Measured on the failing run: ArmStatus.TrackingError went 0.00032 m at t = 5.200
% to 0.51444 m at t = 5.220, and JointError to 2.2837 rad, against a cfg.safety.trackErrMax
% of 0.15 m. MovingFree's tracking guard tripped on the step after entry and reported
% cfg.fail.ArmPlanFailed, which is the same code six planner gates use, so the run looked
% like a planner refusal when the planner had in fact succeeded. The task failed at 8.78 s
% having never entered ApproachObject - no descent at all.
%
% NOTHING CHANGES FOR THE PLANS THAT ALREADY WORKED. With the arm at stow, qArmNow IS
% cfg.arm.stowConfig, so the first plan of every pick and place searches from exactly the
% state it searched from before.
qStart = [qArmNow,               qGripZero];
qGoal  = [pregraspConfig(:)',    qGripZero];

% THE SEARCH IS SAMPLING-BASED, SO WITHOUT A SEED THE EXAMPLE IS A COIN FLIP.
% manipulatorRRT draws from the global random stream, and the stream's state when this
% runs depends on everything the model did before it - so the SAME model, config and
% base pose produced a valid plan on one run and ArmPlanFailed on the next, twice, and
% the difference was read as a regression from an unrelated edit. cfg.plan.ikSeed
% already exists for exactly this reason on the IK restarts; this closes the other half.
%
% AND THE SEARCH IS DRAWN MORE THAN ONCE, because the gate it has to pass is one it was
% never allowed to see. IgnoreSelfCollision is true a dozen lines above, out of necessity,
% so every path returned here is a lottery ticket against the self-collision check below.
% Four consecutive tickets lost in one integration run - all four attempts ArmSelfCollide,
% the arm never moved - which is what a single draw against a 25% failure rate looks like.
% That rate was measured on this exact problem: 9 of 12 seeds pass raw, 12 of 12 after
% shorten(). So the loop below re-seeds and re-searches, keeps the FIRST path that clears
% the gate, and reports failure only when every attempt has failed.
%
% Nothing is weakened to achieve this. The gate is applied unchanged to every candidate
% path; the only change is that a rejected path ends an attempt instead of ending the task.
%
% SEEDED PER ATTEMPT AND PER REQUEST, NOT FROM A CONSTANT. A constant would make the
% supervisor's second plan attempt draw the identical tree and therefore fail identically,
% which turns cfg.sup.retry.armPlan into decoration. Striding by the attempt count keeps
% every (request, attempt) pair on its own reproducible stream, so a retry is a genuinely
% different search, no two attempts ever repeat each other, and the whole run still
% reproduces exactly.
%
% The stream is put back afterwards. This function is not the only user of randomness in
% the run, and silently reseeding the global stream from inside a service would make
% every later draw a function of when a plan happened to be requested.
streamState = rng();
restoreStream = onCleanup(@() rng(streamState));

% Downsampled, never truncated. Truncating would drop the last waypoints, and the
% last waypoint is the grasp - a "plan" that stops partway is worse than no plan,
% because it looks valid. linspace keeps the first and last states exactly. The cap is
% applied INSIDE the loop because the gate is applied to the capped set: dropping
% waypoints changes which poses are checked, so checking before capping would vouch for
% a path that is not the one handed over.
capW  = double(cfg.plan.armWaypointCap);
qGrip = zeros(1, cfg.gripper.nJoints);
nAttempt = double(cfg.plan.armPlanAttempts);

% Hoisted from the time-parameterisation section below, which is its other consumer, because
% the direct-segment gate a few lines down needs the same box and there is no reason to
% derive it twice.
jntLim = jointPositionLimits(model.planTree, nArm);

W = zeros(nArm, 0);
truncated = false;
anyPath = false;

% A RE-PLAN THAT HAS NOWHERE TO GO IS NOT A PLANNING PROBLEM, and without this it reports as
% one. Now that the search starts from the measured configuration, a re-plan issued after the
% wrist refinement DECLINED to move the grasp point - every status other than
% cfg.refine.STATUS_APPLIED returns the coarse position unchanged, which is the advisory
% guarantee working as designed - asks for a path from the standoff to the same standoff.
% manipulatorRRT answers that with a single state, the size(waypoints, 1) < 2 guard below
% rejects it on all three attempts, and the task dies ArmRrtFailed for having nothing to do.
% That would make a REJECTED refinement fatal, which is precisely the outcome hmmWristRefine's
% header promises cannot happen.
%
% The two-state path is the honest answer rather than a special case bolted on: it is what the
% planner would return for a zero-length problem if it could. ArmSupervisor's MovingFree then
% sees a duration of about zero with the arm already settled and takes its DONE exit on the
% first step, so a declined refinement costs one supervisor tick and nothing else.
if norm(qGoal - qStart) <= cfg.plan.armReplanTol
    W = [qStart(1:nArm)', qGoal(1:nArm)'];
    anyPath  = true;
    nAttempt = 0;
end

% AND A REPLAN THAT HAS ALMOST NOWHERE TO GO IS STILL NOT A SEARCH PROBLEM. This is the same
% argument as the clause above, one step weaker: not "the goal is where the arm already is" but
% "the goal is inside one of the planner's own connection steps". THIS IS NOT A SECOND PLANNER
% AND IT DOES NOT DISPLACE BiRRT - a straight extension of that length is the first thing BiRRT
% itself attempts, which is exactly why cfg.plan.armRrtMaxConnDist is the threshold rather than
% some new number: the test is "is this whole problem one extension?", and MaxConnectionDistance
% is the planner's own answer to what one extension is.
%
% WHAT IT FIXES, MEASURED ON THE 12 s RUN OF 2026-09-30. The wrist refinement moved the grasp
% point 40.9 mm and the arm answered with 1.04 s of MOVING_FREE that went the wrong way first -
% the tool swung 65.5 mm sideways off the straight line between its two poses and 62.5 mm along
% a 41.3 mm chord, i.e. it overshot the goal by half again and came back, which reads on screen
% as the arm going BACK AND UP before the corrected descent. The cause is not the destination and
% not the IK: shoulder_pan travels 1.522 deg net and swings 5.449 deg PAST its own endpoint on the
% way, so the path is a genuine random-tree detour rather than the unsynchronised-but-monotone
% staircase the smoothing section downstream describes. The endpoints were right all along -
% holding the tool orientation fixed, the 24.4 deg the exact 6-DOF step needs is MORE than the
% 21.9 deg the planner used, so there is no wasted reconfiguration to remove and nothing about the
% pregrasp seeding above needs revisiting. A straight joint-space ramp between those SAME two
% configurations tracks the chord to 0.72 mm and never overshoots.
%
% THE SEPARATION IS 15x, SO THE THRESHOLD IS NOT A TUNING. The refinement correction spans
% 0.2078 rad against a 0.400 rad MaxConnectionDistance; the stow -> pregrasp transit this
% function exists to plan spans 3.1599 rad. No real transit is going to drift into this branch.
%
% AND IT IS CHECKED HARDER THAN A SEARCHED PATH, not merely trusted for being short.
% smoothedPathIsFree is the gate the smoothed curve faces below - joint box, pathSelfCollides,
% and a world check on a set subdivided until its largest step is under the planner's own
% ValidationDistance - whereas an RRT path reaches the same point having had its world check done
% by a planner running with IgnoreSelfCollision true. A segment that does not clear the gate sets
% nothing and falls through to the search below completely unchanged, so this cannot make a plan
% fail that would previously have succeeded.
if isempty(W) && norm(qGoal - qStart) <= cfg.plan.armRrtMaxConnDist
    Wdirect = [qStart(1:nArm)', qGoal(1:nArm)'];
    if smoothedPathIsFree(model, Wdirect, qGrip, envPlan, ...
            cfg.plan.armRrtValidationDist, jntLim)
        W = Wdirect;
        anyPath  = true;
        nAttempt = 0;
    end
end

% A STOW RETRACES THE WAY IN, WHEN IT CAN, INSTEAD OF SEARCHING. The legs that brought the arm here
% - stow -> pregrasp, and on a pick the refinement's pregrasp -> corrected pregrasp - were searched
% and certified against this same furniture, so their reverse is a known-good route home that costs
% one gate instead of a ~17 s search (measured 2026-10-12). Two things differ from the way in and both
% are handled rather than assumed. The retreat need not end on the hover pose - after the Bottle's
% place it ended 29.4 mm higher, 0.20 rad away - so the reversed chain is entered at the first
% waypoint not BELOW the tool's current height, which also means the arm never drops back toward the
% object it has just put down. And the hand may now be full (after a pick), so the whole thing goes
% through smoothedPathIsFree with the held object in the model. Anything that fails falls through to
% the search below unchanged.
%
% AND THE STRAIGHT JOINT MOVE IS TRIED FIRST, BECAUSE IT IS THE FASTEST AND OFTEN FINE. It is the
% move Stowing always made - but now only when the same gate clears it. Measured on the shipped mug
% transfer: retracing both stows cost 7.3 s of model time (DONE 30.68 s against 23.38 s) over a side
% table the straight move never touched; on the bedroom chest the straight move is the one that went
% 191 mm through it and the gate refuses it, so the retrace is what runs there.
if isStow && isempty(W)
    tStage = tic;
    Wstraight = [qStart(1:nArm)', qGoal(1:nArm)'];
    % AFTER A PLACE, THE OBJECT JUST PUT DOWN IS IN THE STRAIGHT MOVE'S WAY, and only furniture
    % was in its check. The withdraw backs out one approach standoff, which leaves the fingers
    % still around a tall object's upper half - 94 mm of jaw below the Bottle's top on the
    % kitchen counter - and the straight move to stow swept them through it (2026-10-19). So
    % the move is also checked against a box the object's size standing on the place point; if
    % it fails, the retrace below runs, which never drops back toward the object. Only this
    % gate: the retrace is entered above the tool and the search starts where the fingers are.
    envStraight = envPlan;
    if ~carrying && placeTarget.Valid
        hObj = double(objectTarget.Extent(3));
        sObj = max(double(objectTarget.Extent(1:2)));
        placed = collisionBox(sObj, sObj, hObj);
        placed.Pose = tfArmFromMap * trvec2tform([double(placeTarget.Point(1)), ...
            double(placeTarget.Point(2)), double(placeTarget.Point(3)) + 0.5 * hObj]);
        envStraight{end + 1} = placed;
    end
    if smoothedPathIsFree(model, Wstraight, qGrip, envStraight, ...
            cfg.plan.armRrtValidationDist, jntLim)
        W = Wstraight;
        anyPath  = true;
        nAttempt = 0;
        fprintf('  plan: stow goes straight - the direct move clears the furniture (%.1f s)\n', toc(tStage));
    else
        fprintf('  plan: stow straight move refused (%.1f s)\n', toc(tStage));
    end
end
if isStow && isempty(W)
    tStage = tic;
    [Wrev, exact] = reversedApproach(model, cfg, qArmNow, qGoal(1:nArm), capW);
    vd = cfg.plan.armRrtValidationDist;
    % WITH AN EMPTY HAND THE RETRACE IS ALREADY CERTIFIED, except for its two ends. The legs were
    % gated against this same furniture with the arm as it was then - carrying the object, on a
    % place - and an arm without the object occupies strictly less space, so the interior segments
    % cannot collide now if they did not then. What is new is the hop from where the retreat ended
    % into the chain and the last segment onto the exact stow configuration; those two are gated.
    % A thinned path (exact false) is no longer the certified one and is gated in full, and so is
    % any retrace with the object in hand, which the way in never carried (2026-10-14: the full
    % re-gate was most of a 28 s planning freeze after the mug's place).
    if isempty(Wrev)
        ok = false;
    elseif ~carrying && exact && size(Wrev, 2) >= 3
        ok = smoothedPathIsFree(model, Wrev(:, 1:2), qGrip, envPlan, vd, jntLim) && ...
             smoothedPathIsFree(model, Wrev(:, end - 1:end), qGrip, envPlan, vd, jntLim);
    else
        ok = smoothedPathIsFree(model, Wrev, qGrip, envPlan, vd, jntLim);
    end
    tGate = toc(tStage);
    if isempty(Wrev)
        fprintf('  plan: stow searched - no approach chain ends here to retrace\n');
    elseif ok
        % AND THE RETRACE IS SHORTCUT, because it inherits every wander of the random tree that
        % drew the way in: measured 4.2 s per retraced stow against 1.2 s for a straight move.
        % shortcutPath only ever replaces a run of waypoints with one segment that passes the same
        % gate, so the result is never less checked than the path it came from.
        tStage = tic;
        Wcut = shortcutPath(model, Wrev, qGrip, envPlan, vd, jntLim);
        tCut = toc(tStage);
        W = Wcut;
        anyPath  = true;
        nAttempt = 0;
        fprintf(['  plan: stow retraces the approach (%d waypoints, %d after shortcut), no search ' ...
                 'needed - gate %.1f s, shortcut %.1f s of wall clock\n'], ...
            size(Wrev, 2), size(Wcut, 2), tGate, tCut);
    else
        fprintf('  plan: stow searched - the retraced approach failed the gate (carrying %d)\n', ...
            carrying);
    end
end

for attempt = 1:nAttempt
    rng(cfg.plan.rrtSeed + double(requestId) * nAttempt + attempt, 'twister');

    try
        waypoints = plan(planner, qStart, qGoal);
    catch
        continue
    end
    if size(waypoints, 1) < 2
        continue
    end
    anyPath = true;

    % SHORTENED BEFORE IT IS JUDGED, and this is what actually moved the pass rate from
    % 75% to 100%. A randomised tree wanders: the raw path here is three waypoints whose
    % segments are long enough that the midpoint of one is a wholly different arm pose
    % from either end. shorten() returns a finer, straighter polyline through the same
    % free space, which both folds into itself less often and is checked more faithfully
    % by a gate that samples waypoints and midpoints. Failure is tolerated because a raw
    % path is still a valid path - shortening is an improvement, not a precondition.
    try
        waypoints = shorten(planner, waypoints, cfg.plan.armRrtShortenIters);
    catch
        % keep the unshortened path and let the gate judge it
    end

    Wtry = waypoints(:, 1:nArm)';              % nArm x N
    truncated = size(Wtry, 2) > capW;
    if truncated
        Wtry = Wtry(:, round(linspace(1, size(Wtry, 2), capW)));
    end

    % kBad and badPairs are deliberately discarded here. They exist because naming the
    % failing waypoint and the offending body pair is what diagnosed the repeated
    % ArmSelfCollide - the answer was "waypoint 11 of 14, base against shoulder_link",
    % which pointed at the DESTINATION rather than the route and led to the unwind above.
    % They are not reported through the bus because the bus is a fixed-width signal and a
    % body-pair name is a string; a future need for them belongs in an offline diagnostic, not
    % in the plan output.
    if pathSelfCollides(model, Wtry, qGrip)
        continue
    end

    W = Wtry;
    break
end
clear restoreStream

if isempty(W)
    % WHICH FAILURE IT WAS, distinguished. No path at all means the search could not
    % connect start to goal and the base is probably badly placed; paths that were all
    % rejected means the goal is reachable but every route to it folded the arm, which is
    % a different problem calling for a different response.
    if anyPath
        aplan.FailureCode = cfg.fail.ArmSelfCollide;
    else
        aplan.FailureCode = cfg.fail.ArmRrtFailed;
    end
    aplan.PlanTime = toc(t0);
    return
end

nW = size(W, 2);
aplan.Truncated = truncated;
aplan.Waypoints(:, 1:nW) = W;
aplan.NumWaypoints = uint8(nW);

% PregraspConfig is NOT set from W here. It is the searched-for GOAL, solved and
% screened above, and W(:, end) equals it to the planner's validation tolerance; taking
% the penultimate waypoint instead would report a pose one random tree segment short of
% the standoff and call it the standoff.

%% The polyline is SMOOTHED before it is timed, and this is worth 46% of the move
% WHAT IS BEING FIXED. The path above is a polyline, and contopptraj must obey the
% acceleration box THROUGH its corners - so at every direction change the speed has to fall
% almost to zero and climb back. The result was measured on the shipped recording: one
% MOVING_FREE phase, 3.04 s, with 20 interior instants essentially at rest while the peak
% everywhere else sat at 89% of the velocity box. The phase was not speed-limited. It was
% being stopped, repeatedly, and it was visible on screen as a stutter with the tool
% drifting 6 mm BACKWARDS halfway through the move.
%
% AND THE CORNERS COST NOTHING IN DISTANCE, WHICH IS WHY SHORTENING NEVER HELPED. The
% shortened path's joint-space L1 length is 6.023 rad and the straight line from start to
% goal is also 6.023 rad. The planner is not detouring; it returns the straight line as an
% unsynchronised staircase - each joint monotone, none moving in proportion - so the polyline
% is kinked at every step. shorten() removes distance, there is no excess distance, and
% measurement confirmed the length is pinned at 6.023 rad for every iteration count at or
% above the shipped 20 over 18 seeds each. See cfg.plan.armSmoothPoints.
%
% CONTROL POINTS, NOT INTERPOLATION POINTS - that distinction IS the fix. bsplinepolytraj
% treats W as control points: the curve is clamped to the first and last, so the stow pose and
% the pregrasp are still hit EXACTLY, but the interior corners are CUT rather than passed
% through. Cutting them is what removes the decelerations. It also means the curve leaves the
% polyline manipulatorRRT certified, by a measured median of 0.003 rad and a worst case of
% 0.016 rad over 8 seeds - under a fifth of a degree - for 1.55 s of the 3.37 s move.
%
% SO IT IS RE-CHECKED, AND IT FALLS BACK RATHER THAN FAILING. The corner-cut region was
% validated by nothing, so both production collision tests are re-run on the smoothed curve.
% A curve that does not clear them is discarded and the polyline is timed instead - exactly
% what shipped before - so this cannot make a plan fail that would previously have succeeded.
%
% THE CHECK SUBDIVIDES; IT DOES NOT ASSUME 32 SAMPLES ARE FINE ENOUGH. They are not. 32 samples
% over a 2.18 rad span averages 0.070 rad a step against a 0.100 rad ValidationDistance, this
% comment used to cite that average as though it settled the matter, and a collision check is a
% statement about a MAXIMUM. bsplinepolytraj samples uniformly in its parameter, which bunches
% samples on the straight stretches and spreads them through the turns; the measured maximum is
% 0.1424 rad median and 0.2491 rad worst, so the old assertion rejected every seed on this leg
% while both collision tests were clear, and the leg kept its 3.04 s. smoothedPathIsFree now
% subdivides the CHECK set instead - the reference itself is still the 32 samples, so no bus
% width and no executed trajectory changes.
%
% Cost is a few seconds of wall clock inside one triggered step and NO simulated time - arm
% AWAIT_PLAN lasts 0.02 s either way, a single supervisor tick.
% THE CHAIN IS THREE DEEP AND TRIED BEST-FIRST: CHOMP, then the B-spline, then the polyline.
% CHOMP goes first because it is the only one of the three that KNOWS the obstacles are there -
% it descends smoothness plus collision cost, so it does not have to be lucky to clear the gate,
% and it measured 8/8 gate passes on both legs against the B-spline's 8/8 and 4/8. The B-spline
% stays as the second choice because when it does clear the gate it is about 0.1 s quicker, and
% because it needs no signed distance field, so it still works if the field cannot be built.
% See cfg.plan.armChompEnable for the measured table and the expected-value argument.
%
% AND THE GATE NOW CHECKS JOINT LIMITS, WHICH IT DID NOT NEED TO BEFORE. bsplinepolytraj is
% safe on limits for free: the curve lies in the convex hull of its control points, and the
% control points are planner waypoints, which are in-limit by construction. CHOMP has no
% limit awareness at all - there is no limit property on manipulatorCHOMP or on
% chompSolverOptions - so nothing stops a descent step pushing a joint past its stop. An
% out-of-limit reference does NOT degrade gracefully downstream; it fails the plan outright
% with ArmRefOutOfBox. So the bound is a gate condition here, where the fallback still exists.

% The kinematic boxes are derived HERE, before the smoothing rather than after it, because
% CHOMP needs them too: its seed time points come from what contopptraj makes of the raw
% polyline, so the smoother and the parameterisation have to be reading the same limits.
% Inside the boxes SafetyMonitor guards, not on them: the monitor differentiates the
% realised velocity and trips on a strict >, so a reference parameterised to the limit
% is a reference the servo's own overshoot turns into a JOINT_ACCEL E-stop.
velLim = cfg.arm.planMargin * [-cfg.arm.jointVelMax(:), cfg.arm.jointVelMax(:)];
accLim = cfg.arm.planMargin * [-cfg.arm.jointAccMax(:), cfg.arm.jointAccMax(:)];
nS = double(cfg.plan.armSampleCap);
% AND A TWO-WAYPOINT PATH IS NOT SMOOTHED, because there is nothing there to smooth. Both
% smoothers exist to cut the interior corners of a polyline and a two-column W has no interior
% corner: the B-spline over two clamped control points IS the straight segment, and CHOMP seeded
% on a collision-free straight line is descending a cost whose gradient is zero, so the best case
% is that it returns its own seed after paying for the signed distance field. It is skipped rather
% than relied upon to be a fixed point - the straight segment is already the shortest and smoothest
% path through its own endpoints, and letting an optimiser near it could only put wander back into
% the one case that was added above specifically to take wander out.
Wtimed = W;
hasCorners = nW > 2;
smoothed = false;
if hasCorners && cfg.plan.armChompEnable
    [Wchomp, chompOk] = chompPolyline(model, W, qGrip, envPlan, cfg, velLim, accLim, nS);
    if chompOk && smoothedPathIsFree(model, Wchomp, qGrip, envPlan, ...
            cfg.plan.armRrtValidationDist, jntLim)
        Wtimed = Wchomp;
        smoothed = true;
    end
end
if hasCorners && ~smoothed
    [Wsmooth, smoothOk] = smoothPolyline(W, double(cfg.plan.armSmoothPoints));
    if smoothOk && smoothedPathIsFree(model, Wsmooth, qGrip, envPlan, ...
            cfg.plan.armRrtValidationDist, jntLim)
        Wtimed = Wsmooth;
    end
end

%% Time parameterisation under the joint kinematic boxes

[q, qd, tSamp, ok] = parameterise(Wtimed, velLim, accLim, nS);
if ~ok
    aplan.FailureCode = cfg.fail.ArmTimeParamFailed;
    aplan.PlanTime = toc(t0);
    return
end

% THE SPLINE IS CHECKED AGAINST THE POSITION BOX, NOT JUST THE WAYPOINTS. contopptraj
% fits a curve through the waypoints, and a curve through a point that sits near a limit
% passes OUTSIDE it - the same reason the collision check above is re-run on the samples
% rather than trusted from the planner. The box here is the one the ROBOT has,
% model.armPosBox, not the narrowed box planTree carries: cfg.arm.limitMargin is what
% normally keeps the spline clear, and this is what catches a margin that was not enough.
%
% Failing the plan is the correct answer and a strictly better one than shipping it.
% ArmControl/JointLimit is a hard Saturate, so an out-of-box reference is not refused
% downstream - it is CLAMPED, and a clamped position command is a corner whose derivative
% SafetyMonitor reads as an acceleration step and answers with a JOINT_ACCEL E-stop
% halfway through the move. That blames the servo for the planner's mistake, and it was
% measured doing exactly that: 8.55 rad/s^2 against a 4.0 box on joint 3, whose band is
% half of every other joint's.
outsideBox = q < model.armPosBox(:, 1) | q > model.armPosBox(:, 2);
if any(outsideBox(:))
    aplan.FailureCode = cfg.fail.ArmRefOutOfBox;
    aplan.PlanTime = toc(t0);
    return
end

nS = min(size(q, 2), double(cfg.plan.armSampleCap));
aplan.RefTime(1:nS)    = tSamp(1:nS)';
aplan.RefPos(:, 1:nS)  = q(:, 1:nS);
aplan.RefVel(:, 1:nS)  = qd(:, 1:nS);
aplan.NumSamples       = uint16(nS);
aplan.Duration         = tSamp(nS);
aplan.Truncated        = aplan.Truncated || size(q, 2) > double(cfg.plan.armSampleCap);
% ArmControl's Polynomial Trajectory block takes RefTime as its time points, which must be
% strictly increasing over the whole 128 entries, not just the ones a sample occupies.
aplan.RefTime          = hmmPadRefTime(aplan.RefTime, nS);

%% The reference, not just the waypoints, must clear the surface
% Every sample, because this is the check the whole surface proxy exists for and
% the smooth curve is NOT the piecewise-linear path the planner validated. The
% clearance reported on the bus is measured here, on what will actually be driven.
% THE SAME LOOP ALSO APPLIES SafetyMonitor's WORKSPACE TEST, on the guard's own two
% conditions and its own numbers, so a plan that passes here cannot be WORKSPACE-E-stopped
% while it is driven. The guard ORs a reach test against cfg.safety.toolReachMax with a
% bounds test against cfg.percep.sceneBounds, and both are recomputed rather than
% approximated: getTransform on planTree between the same two frames the monitor's Get
% Transform block is configured with, tcp_frame relative to the plan base, which is why
% planBaseFrame is a field of the model rather than a name written here.
%
% cfg.safety.toolReachMax AND NOT cfg.arm.reach, AND THE INVARIANT DEPENDS ON IT. This test
% exists solely so the planner and the guard cannot disagree, so it must use the constant
% the guard uses; pointing it at cfg.arm.reach instead made it disagree by 0.49 m and
% rejected a plan whose worst sample was 0.8634 m against a 0.850 m bound that measures the
% flange while the test measures the tool 0.1416 m beyond it. See the derivation at
% cfg.safety.toolReachMax. If one of these two constants is ever changed, this line and
% SafetyMonitor/WorkspaceCheck must change together or the invariant silently lapses.
% THE WORKSPACE TEST RUNS TO THE END OF THE REFERENCE RATHER THAN BAILING AT THE FIRST
% SAMPLE. A collision is a yes/no and returning early saves real work, but "the reference
% left the workspace" is useless without the margin: 1 mm over the reach bound and 200 mm
% under the floor are the same code and completely different problems, and the only way to
% tell them apart from a failed run is to have recorded the worst excursion on each axis.
% The extra cost is a getTransform per remaining sample on a plan that is being rejected
% anyway.
worstSep = inf;
maxReach = 0;
worstLo  = zeros(3, 1);         % how far past the lower scene bound, per axis
worstHi  = zeros(3, 1);         % how far past the upper scene bound, per axis
% EVERY SAMPLE IS CHECKED, BUT ONLY FOR CONTACT; THE CLEARANCE IS MEASURED ON A STRIDE. The verdict
% needs a yes/no at every sample and gets one, non-exhaustively (stops at the first contact). The
% exhaustive distance pass that measures MinClearance - a report field nothing in the model reads -
% runs on every cfg.plan.armClearanceStride-th sample and the last, which cut ~1.5 s of a ~5 s loop
% per plan (2026-10-14). No collision decision changes.
stride = double(cfg.plan.armClearanceStride);
for k = 1:nS
    [~, sepAny] = checkCollision(model.planTree, [q(:, k)', qGrip], envPlan, ...
        'IgnoreSelfCollision', 'on', 'Exhaustive', 'off');
    if any(isnan(sepAny(:)))
        aplan.FailureCode = cfg.fail.ArmRefCollide;
        aplan.PlanTime = toc(t0);
        return
    end
    if mod(k - 1, stride) == 0 || k == nS
        [~, sepK] = worldCollides(model.planTree, [q(:, k)', qGrip], envPlan);
        worstSep = min(worstSep, sepK);
    end

    tfTool   = getTransform(model.planTree, [q(:, k)', qGrip], ...
        model.toolFrame, model.planBaseFrame);
    maxReach = max(maxReach, norm(tfTool(1:3, 4)));
    toolWorld = tfArmFromMap \ tfTool;
    toolWorld = toolWorld(1:3, 4);
    worstLo = max(worstLo, cfg.percep.sceneBounds(:, 1) - toolWorld);
    worstHi = max(worstHi, toolWorld - cfg.percep.sceneBounds(:, 2));
end
aplan.MinClearance = worstSep;

if maxReach > cfg.safety.toolReachMax || any(worstLo > 0) || any(worstHi > 0)
    aplan.FailureCode = cfg.fail.ArmRefWorkspace;
    aplan.PlanTime = toc(t0);
    return
end

%% Aperture and attachment
if isPlace
    % Release: open past the object so the pads clear it on the way up.
    aperture = objectTarget.GraspExtent + cfg.grasp.captureMargin;
else
    % Grasp: close to the measured extent less the pad compression, which is how
    % much the compliant pads give when they are loaded.
    aperture = objectTarget.GraspExtent - cfg.gripper.padCompression;
end
aplan.ApertureCmd    = min(max(aperture, cfg.gripper.minOpenPos), cfg.gripper.maxOpenPos);
aplan.AttachActorId  = objectTarget.ActorId;

% THE CERTIFIED POLYLINE JOINS THE APPROACH CHAIN a later stow will retrace - see reversedApproach.
% W is the gated polyline, the route the gate vouched for, not the smoothed curve timed from it.
if isStow
    approachChain('reset');
else
    approachChain('add', qStart(1:nArm), W, cfg);
end

aplan.FailureCode = cfg.fail.OK;
aplan.Valid       = true;
aplan.PlanTime    = toc(t0);
if isStow
    fprintf('  plan: stow planned in %.1f s of wall clock, %.2f s of motion\n', ...
        aplan.PlanTime, aplan.Duration);
end

end

%% ------------------------------------------------------------------------
function Wout = shortcutPath(model, W, qGrip, env, vd, jntLim)
%SHORTCUTPATH Greedy shortcut of a gated polyline: from each kept waypoint, jump as far ahead as a
%   straight segment passing smoothedPathIsFree allows. Endpoints are kept exactly. A path no
%   segment of which can be shortened comes back unchanged.
%   AT MOST TWO GATE CALLS PER STEP - the far end, then halfway - because each call is seconds of
%   frozen simulation on a stow-sized segment; trying every waypoint in turn cost more wall clock
%   than the shorter motion saved.
n    = size(W, 2);
keep = 1;
i    = 1;
while i < n
    next = i + 1;
    for j = unique([n, i + ceil((n - i) / 2)], 'stable')
        if j >= i + 2 && smoothedPathIsFree(model, W(:, [i j]), qGrip, env, vd, jntLim)
            next = j;
            break
        end
    end
    keep(end + 1) = next; %#ok<AGROW>
    i = next;
end
Wout = W(:, keep);
end

%% ------------------------------------------------------------------------
function chain = approachChain(op, q0, W, cfg)
%APPROACHCHAIN The certified joint-space legs since the arm last left stow, nArm x N.
%   approachChain('add', q0, W, cfg) appends a plan's polyline W that started at q0: a leg starting
%   at cfg.arm.stowConfig begins a new chain, one starting where the chain ends extends it (the
%   wrist refinement's re-plan), and anything else clears it, because a chain with a gap in it is
%   not a route. approachChain('reset') clears it; chain = approachChain('get') reads it.
%   Persistent, which is safe because every first leg of a run starts at stow and so restarts it.
persistent held
switch op
    case 'reset'
        held = [];
    case 'add'
        tol = cfg.plan.stowChainTol;
        if norm(q0(:) - double(cfg.arm.stowConfig(:))) <= tol
            held = W;
        elseif ~isempty(held) && norm(q0(:) - held(:, end)) <= tol
            held = [held, W(:, 2:end)];
        else
            held = [];
        end
end
chain = held;
end

%% ------------------------------------------------------------------------
function [Wrev, exact] = reversedApproach(model, cfg, qNow, qStow, capW)
%REVERSEDAPPROACH The approach chain run backwards from the arm's current posture to stow, or [].
%   Entered at the first chain waypoint whose tool height is not below the current one, so the move
%   never drops back toward the surface just worked on; the current posture is prepended and the
%   stow configuration, exactly, appended. Downsampled to capW keeping both ends, BEFORE the caller
%   gates it, because the gate must judge the set that is handed over.
%   exact is false when the path had to be thinned to capW, i.e. its interior segments are no
%   longer the ones that were certified.
Wrev  = [];
exact = false;
chain = approachChain('get');
if isempty(chain) || size(chain, 2) < 2
    return
end
R    = fliplr(chain);
nG   = double(cfg.gripper.nJoints);
zOf  = @(q) toolZ(model.planTree, [q(:)', zeros(1, nG)], model.toolFrame);
zNow = zOf(qNow);
i0   = size(R, 2);
for i = 1:size(R, 2)
    if zOf(R(:, i)) >= zNow - 1e-3
        i0 = i;
        break
    end
end
Wrev  = [qNow(:), R(:, i0:end - 1), qStow(:)];
exact = size(Wrev, 2) <= capW;
if ~exact
    Wrev = Wrev(:, round(linspace(1, size(Wrev, 2), capW)));
end
end

function z = toolZ(tree, q, toolFrame)
T = getTransform(tree, q, toolFrame);
z = T(3, 4);
end

%% ------------------------------------------------------------------------
function [z, xy, carrying] = stowSurface(objectTarget, placeTarget, robotConfig)
%STOWSURFACE The surface a stow is leaving, and whether the hand is full.
%   The stow follows either a pick (StowLoaded, object in hand) or a place (StowEmpty), and the
%   base is parked beside the target of the one it just did - so the NEARER valid target is the
%   surface whose furniture the stow must clear, and the hand is full exactly when that target
%   is the object. Nothing here reads a true pose or the gripper state; both targets are buses
%   this function already receives.
base = double(robotConfig(1:2));
dO = Inf;
dP = Inf;
if objectTarget.Valid
    dO = norm(double(objectTarget.Position(1:2)) - base(:));
end
if placeTarget.Valid
    dP = norm(double(placeTarget.Point(1:2)) - base(:));
end
if dP < dO
    z = placeTarget.Point(3);
    xy = placeTarget.Point(1:2);
    carrying = false;
else
    z = objectTarget.Position(3);
    xy = objectTarget.Position(1:2);
    carrying = logical(objectTarget.Valid);
end
end

%% ------------------------------------------------------------------------
function q = unwindToward(tree, q, qRef, box)
%UNWINDTOWARD The same tool pose, on the revolution nearest a reference configuration.
%   For each revolute joint, q(j) + 2*pi*k is kinematically identical to q(j), so k is
%   chosen to put the joint as close to qRef(j) as the box allows. |q + 2*pi*k - qRef| is
%   convex in k, so the unconstrained optimum rounded and then CLAMPED into the feasible
%   integer range is the constrained optimum - no search needed.
%
%   PRISMATIC JOINTS ARE LEFT ALONE, and the check is against the tree rather than
%   assumed, because a translation has no periodicity and shifting one by 2*pi would move
%   the tool 6.28 m rather than leave it where it is. This arm has none today; a future
%   one with a rail would break silently without this guard.
isRev = revoluteFlags(tree, numel(q));
for j = 1:numel(q)
    if ~isRev(j)
        continue
    end
    kLo = ceil((box(j, 1) - q(j)) / (2 * pi));
    kHi = floor((box(j, 2) - q(j)) / (2 * pi));
    if kHi < kLo
        continue           % no revolution of this joint fits the box; leave it as solved
    end
    k = round((qRef(j) - q(j)) / (2 * pi));
    q(j) = q(j) + 2 * pi * min(max(k, kLo), kHi);
end
end

%% ------------------------------------------------------------------------
function flags = revoluteFlags(tree, n)
%REVOLUTEFLAGS Whether each of the first n movable joints is revolute.
%   tree.Bodies is read ONCE: every read of that property copies every body, so indexing it in
%   the loop made this quadratic - 224 calls cost 15.3 s of a 25 s transfer (profiled 2026-10-08).
flags = false(1, n);
k = 0;
bodies = tree.Bodies;
for b = 1:numel(bodies)
    j = bodies{b}.Joint;
    if strcmp(j.Type, 'fixed')
        continue
    end
    k = k + 1;
    if k > n
        return
    end
    flags(k) = strcmp(j.Type, 'revolute');
end
end

%% ------------------------------------------------------------------------
function [Ws, ok] = smoothPolyline(W, nPts)
%SMOOTHPOLYLINE The planner's polyline as a clamped B-spline, sampled for parameterisation.
%   W is used as CONTROL points, not interpolation points, which is the entire mechanism: the
%   curve is clamped to the first and last, so the start and the grasp are hit exactly, while
%   the interior corners are CUT. Cutting them is what stops contopptraj having to decelerate
%   into a direction change. See cfg.plan.armSmoothPoints for what it is worth and what it costs
%   in fidelity.
%
%   ok = false means "no smoothed curve was produced", and the caller then times the polyline
%   exactly as it did before this existed. Three ways that happens, all of them fine:
%     - fewer than three waypoints. A 2-point path has no interior corner, so there is nothing
%       to smooth and the straight segment is already what contopptraj wants.
%     - bsplinepolytraj throws.
%     - the returned curve does not start and end ON the endpoints. bsplinepolytraj is clamped,
%       so this cannot currently happen and the test is an assertion, not a repair - but a
%       reference that stops short of the grasp is the one failure mode that would LOOK valid,
%       so it is checked rather than assumed. Same reasoning as the caller's refusal to
%       truncate a path rather than downsample it.
ok = false;
Ws = W;
if size(W, 2) < 3
    return
end
try
    Q = bsplinepolytraj(W, [0 1], linspace(0, 1, nPts));
catch
    return
end
if norm(Q(:, 1) - W(:, 1)) > 1e-9 || norm(Q(:, end) - W(:, end)) > 1e-9
    return
end
Ws = Q;
ok = true;
end

%% ------------------------------------------------------------------------
function tf = smoothedPathIsFree(model, W, qGrip, env, vDist, lim)
%SMOOTHEDPATHISFREE The joint-limit box and both production collision tests on a smoothed curve.
%   Applied to the curve rather than the polyline because the corner-cut region is not part of
%   the polyline the planner validated. The self-collision test is pathSelfCollides itself, so
%   the smoothed curve faces the SAME gate every candidate path faces - a separate, looser check
%   here would be a second opinion the project does not want, and a stricter one would reject
%   curves whose polylines had already passed.
%
%   THE RESOLUTION IS ACHIEVED, NOT ASSERTED - AND THE DIFFERENCE COST A WHOLE FREE MOVE. This
%   check used to REJECT any curve whose largest joint step exceeded vDist, on the arithmetic
%   that 32 samples over a 2.18 rad span is 0.070 rad a step against a 0.100 rad
%   ValidationDistance. That arithmetic is an AVERAGE, and a collision check is a statement
%   about a maximum. bsplinepolytraj samples uniformly in its PARAMETER, and a B-spline's
%   parameter is not uniform in joint space - the samples bunch where the curve is straight and
%   spread where it turns, which is exactly where the check is needed. The real maximum
%   measured 0.1424 rad median and 0.2491 rad worst, so the clause rejected 8 of 8 seeds on the
%   stow -> pregrasp leg WITH BOTH COLLISION TESTS CLEAR EVERY TIME, and that leg forfeited the
%   whole 1.55 s the smoothing was worth. The curve was never unsafe; the check was too coarse
%   to say so, and a check that cannot certify its input should do more work rather than
%   discard the input.
%
%   So the check set is subdivided until its largest step is under vDist. The subdivision is on
%   the CHECK only - the reference handed to contopptraj is still the nPts samples, so nothing
%   about the executed trajectory or the plan bus width changes. Subdividing is the right
%   geometry because contopptraj traverses the straight segment between consecutive samples, so
%   points on that segment are points the robot will actually pass through.
%
%   THIS IS THE ONE PLACE A DENSIFIED SELF-COLLISION SWEEP IS WORTH ITS COST, against
%   pathSelfCollides' own header. That header is right that a routine densified sweep is the
%   wrong place to spend ~19 ms a pose when a world check costs ~4 ms. This sweep is not routine:
%   it runs once per plan, on one candidate curve, to certify a reference the robot is about to
%   execute through a region no planner validated. Typical cost is a factor of two on 32 samples
%   plus the midpoints pathSelfCollides adds - a few seconds of WALL CLOCK inside one triggered
%   step, and no simulated time at all, because arm AWAIT_PLAN lasts 0.02 s either way.
%
%   Falling back to the polyline is still always safe, so a curve that genuinely collides costs
%   a slower move and nothing else. That case is real and is NOT a checking artefact: on the
%   stow -> preplace leg 4 of 8 seeds cut a corner into the surface box. No amount of checking
%   fixes those - a smoother that cannot see the obstacles will keep proposing them.
%
%   lim is nArm-by-2 joint position bounds, and the clause is FIRST because it is the cheapest
%   and because it is the only one of the three that can fail for a reason the caller can do
%   nothing about. It exists for CHOMP: the B-spline cannot leave the limit box, since a clamped
%   curve lies in the convex hull of in-limit control points, but CHOMP has no notion of a joint
%   stop and descends wherever the cost gradient points. Tested on the subdivided set for form
%   only - interpolating between two in-limit samples cannot leave the box - so the extremes are
%   taken over the whole curve at once rather than pose by pose.
tf = false;
C = subdivideToStep(W, vDist);
if any(min(C, [], 2) < lim(:, 1) - 1e-9) || any(max(C, [], 2) > lim(:, 2) + 1e-9)
    return
end
% THE WORLD IS CHECKED BEFORE THE SELF PAIRS, and only for a yes/no, because that is the order
% candidates fail in: a shortcut or a straight stow is refused by the furniture, and with this
% order it is refused at its first colliding sample without paying ~51 ms of exhaustive self check
% per waypoint first. Non-exhaustive stops at the first contact - 47 ms against 67 ms per sample
% with 16 boxes - and the verdict is identical, since only "any contact" is asked. The stow's
% planning freeze is what this buys back (2026-10-14).
for k = 1:size(C, 2)
    [~, sep] = checkCollision(model.planTree, [C(:, k)', qGrip], env, ...
        'IgnoreSelfCollision', 'on', 'Exhaustive', 'off');
    if any(isnan(sep(:)))
        return
    end
end
if pathSelfCollides(model, C, qGrip, false)   % C is dense already - no midpoints needed
    return
end
tf = true;
end

%% ------------------------------------------------------------------------
function C = subdivideToStep(W, vDist)
%SUBDIVIDETOSTEP Insert evenly spaced points until no joint moves more than vDist in a step.
%   One uniform factor for the whole polyline rather than per-segment, so the returned set is
%   still an ordered resampling of the same curve with the original samples retained - the
%   originals matter, because they are the points contopptraj is actually given.
C = W;
if size(W, 2) < 2
    return
end
step = max(max(abs(diff(W, 1, 2)), [], 1));
if ~isfinite(step) || step <= vDist
    return
end
k = ceil(step / vDist);
lam = (1:k - 1) / k;
C = zeros(size(W, 1), (size(W, 2) - 1) * k + 1);
C(:, 1) = W(:, 1);
c = 1;
for j = 1:size(W, 2) - 1
    seg = W(:, j) + lam .* (W(:, j + 1) - W(:, j));
    C(:, c + 1:c + k) = [seg, W(:, j + 1)];
    c = c + k;
end
end

%% ------------------------------------------------------------------------
function [Wc, ok] = chompPolyline(model, W, qGrip, env, cfg, velLim, accLim, nS)
%CHOMPPOLYLINE The planner's polyline smoothed by manipulatorCHOMP, which SEES the obstacles.
%   This is the difference that matters against smoothPolyline. A B-spline is a geometric
%   operation on a list of numbers: it cuts the corners and has no way of knowing whether the
%   corner it cut was routed around a counter top. CHOMP descends a cost that is smoothness PLUS
%   collision, evaluated against a signed distance field of this scene, so it does not have to be
%   LUCKY to clear the gate downstream - clearance is in the objective it is minimising. Measured
%   over 8 seeds a leg: CHOMP cleared the gate 8/8 on both free moves, the B-spline 8/8 and 4/8.
%   The mechanism is visible in how far each strays from the polyline: on the counter leg CHOMP
%   goes up to 0.512 rad off it where the B-spline goes 0.006. It is not cutting the corner more
%   carefully, it is leaving the polyline to get away from the box.
%
%   ok = false means no curve was produced and the caller falls through to the B-spline and then
%   to the polyline, so this can only ever cost a slower move. Every step below can fail that way,
%   and building the field is one of them, which is why the B-spline stays underneath: it needs no
%   field at all.
%
%   THE SEED IS NOT A DETAIL. CHOMP is a LOCAL method: it descends from where it is put, so a bad
%   initial trajectory is not a slow start, it is a different answer. Two things about the seed
%   were established by measurement rather than assumed:
%     - DEDUPLICATE FIRST. Shortened RRT paths carry near-coincident vertices. Those become
%       near-coincident time points, the initial trajectory fit goes singular, and CHOMP starts
%       from a wild trajectory - the first version of this measurement returned durations in the
%       tens of thousands of seconds off nothing but that.
%     - TIME IN REAL SECONDS, not a normalised parameter. The time scale sets the balance between
%       the smoothness and collision terms, so a path timed over [0 1] is a different optimisation
%       problem from the same path timed over the 3 s it will really take. contopptraj on the raw
%       polyline is the honest estimate of that, and it is already being computed here anyway.
%   InitialTrajectoryFit is "interp1" rather than the default min-jerk because linear
%   interpolation of a polyline IS the polyline - the seed is then exactly the path the planner
%   validated, which is the one starting point known to be collision-free.
%
%   Endpoints are held exactly by optimize (measured start error 0, end 2.9e-15), so the stow pose
%   and the pre-grasp are still reached, and the six gripper columns move by exactly 0 because
%   their endpoints match and the smoothness term keeps them there.
ok = false;
Wc = W;
nArm = size(W, 1);
if size(W, 2) < 3
    return                          % no interior corner to smooth
end

% The seed timing, from the duration the raw polyline would actually take.
[~, ~, ~, tRaw] = contopptrajQuiet(W, velLim, accLim, nS);
if isempty(tRaw) || ~isfinite(tRaw(end)) || tRaw(end) <= 0
    return
end
keep = [true, max(abs(diff(W, 1, 2)), [], 1) > 1e-6];
Wk = W(:, keep);
if size(Wk, 2) < 3
    return
end
c = [0, cumsum(max(abs(diff(Wk, 1, 2)), [], 1))];
if c(end) <= 0
    return
end
tSeed = tRaw(end) * c / c(end);
if ~all(diff(tSeed) > 0)
    return
end

% The signed distance field, built from the SAME collisionBox objects the planner and the gate
% use, so any disagreement between CHOMP's verdict and the gate's is one of representation - a
% voxel field and sphere-approximated bodies against exact meshes - and not one of scene.
try
    g = cell(1, numel(env));
    for k = 1:numel(env)
        g{k} = geom2struct(env{k}, k);
    end
    tsdf = meshtsdf([g{:}], Resolution = cfg.plan.armChompTsdfRes, ...
        TruncationDistance = cfg.plan.armChompTsdfTrunc);
    chomp = manipulatorCHOMP(model.planTree, MeshTSDF = tsdf);
    chomp.SolverOptions.MaxIterations = cfg.plan.armChompMaxIter;
catch
    return
end

% wpts is waypoints-by-DOF - the orientation manipulatorRRT.plan returns, so the gripper columns
% ride along unchanged and no transpose is needed on the way in.
nPts = double(cfg.plan.armSmoothPoints);
wpts = [Wk', repmat(qGrip, size(Wk, 2), 1)];
try
    traj = optimize(chomp, wpts, tSeed, (tSeed(end) - tSeed(1)) / (nPts - 1), ...
        InitialTrajectoryFit = "interp1");
catch
    return
end
if isempty(traj) || size(traj, 2) < nArm || ~all(isfinite(traj(:)))
    return
end
Q = traj(:, 1:nArm)';
if size(Q, 2) < 3
    return
end
% Same endpoint assertion smoothPolyline makes, and for the same reason: a reference that stops
% short of the grasp is the one failure mode that would look valid. The tolerance is looser
% because these endpoints are held by an optimiser rather than by a clamped basis.
if norm(Q(:, 1) - W(:, 1)) > 1e-6 || norm(Q(:, end) - W(:, end)) > 1e-6
    return
end
Wc = Q;
ok = true;
end

%% ------------------------------------------------------------------------
function [q, qd, qdd, t] = contopptrajQuiet(W, velLim, accLim, nS)
%CONTOPPTRAJQUIET contopptraj with failure reported as an empty t rather than as an error.
q = []; qd = []; qdd = []; t = [];
try
    [q, qd, qdd, t] = contopptraj(W, velLim, accLim, NumSamples = nS);
catch
end
end

%% ------------------------------------------------------------------------
function m = manipAt(tree, q, toolFrame)
%MANIPAT The manipulability index at one configuration, in the model's own definition.
%   THE DEFINITION HAS TO MATCH THE THRESHOLD OR THE THRESHOLD MEANS NOTHING.
%   cfg.plan.manipulabilityMin is the RECIPROCAL CONDITION NUMBER of J*J' over the FULL 6x6
%   geometric Jacobian - the index the candidate IK subsystems in the diagram report as
%   manipulability, and the one the screens at the top of this function compare against. Every
%   other standard manipulability measure is a different number: sqrt(det(Jv*Jv')) over the
%   three linear rows is the Yoshikawa velocity volume, it falls at a different rate, and a
%   draft of the standoff search that used it put the 0.240 rung at 90 % of the grasp where
%   rcond puts it at 79.8 %. If this line and cfg.plan.manipulabilityMin's comment ever
%   disagree, the comment is the specification.
%
%   q is the FULL configuration including the gripper columns, because geometricJacobian wants
%   the tree's own width; only the arm columns move the tool frame, so the gripper's value does
%   not affect the result.
J = geometricJacobian(tree, q, toolFrame);
m = rcond(J * J');
end

%% ------------------------------------------------------------------------
function lim = jointPositionLimits(tree, n)
%JOINTPOSITIONLIMITS The first n movable joints' position bounds, n-by-2.
%   Read from the tree rather than from cfg, because the bound that matters to the smoother is
%   the one the model itself will refuse to exceed. constraintJointBounds lists the movable
%   joints only, in configuration order, so the n rows line up with the n arm columns of a
%   planner path.
B = constraintJointBounds(tree).Bounds;
lim = double(B(1:n, :));
end

%% ------------------------------------------------------------------------
function [hit, kBad, pairs] = pathSelfCollides(model, W, qGrip, withMidpoints)
%PATHSELFCOLLIDES The self-collision gate on a whole candidate path.
%   Checked at every waypoint AND every segment midpoint. Not at every interpolated
%   step, because a self-collision check is exhaustive over all body pairs and costs
%   ~19 ms on this tree against ~4 ms for a world check: sweeping a densified path
%   would spend most of a service call re-proving that a 6-DOF arm with a pinned
%   gripper does not fold into itself. The midpoints are what catch a segment that
%   passes through a pose neither of its endpoints is in - which is also why the
%   caller shortens the path first, since a finer polyline puts the midpoints closer
%   to poses the arm will really pass through.
%   kBad is the index into the check set that failed, where indices 1..nW are the
%   waypoints themselves and nW+1.. are the segment midpoints, so a failure AT nW is
%   the grasp configuration and a failure at 1 is the stow pose. pairs names the bodies.
%   withMidpoints (default true) may be false for a set ALREADY subdivided to the planner's own
%   ValidationDistance - smoothedPathIsFree's - where a midpoint sits within half a validation step
%   of two checked samples and only doubled the cost (5.3 s of an 21 s stow plan, 2026-10-14).
if nargin < 4
    withMidpoints = true;
end
nW = size(W, 2);
checkSet = W;
if nW > 1 && withMidpoints
    checkSet = [W, 0.5 * (W(:, 1:nW - 1) + W(:, 2:nW))];
end
for k = 1:size(checkSet, 2)
    % planTreeSelf, not planTree: model.planArtefact indexes the un-augmented tree's body
    % order, and a carried object is masked from every self pair anyway. See withHeldObject.
    [bad, pairs] = selfCollides(model.planTreeSelf, [checkSet(:, k)', qGrip], ...
        model.planArtefact);
    if bad
        hit = true;
        kBad = k;
        return
    end
end
hit = false;
kBad = 0;
pairs = cell(1, 0);
end

%% ------------------------------------------------------------------------
function [hit, pairs] = selfCollides(tree, q, artefact)
%SELFCOLLIDES Artefact-aware self-collision check, no world objects.
%   Called with no environment so sepDist comes back nB x nB and lines up with the
%   mask hmmArmCollisionModel measured the same way. The second output names the
%   offending body pairs, which is the only form of this answer anyone can act on.
[~, sep] = checkCollision(tree, q, 'IgnoreSelfCollision', 'off', ...
    'Exhaustive', 'on', 'SkippedSelfCollisions', 'parent');
mask = isnan(sep) & ~artefact;
hit = any(mask(:));
pairs = cell(1, 0);
if hit && nargout > 1
    [r, c] = find(mask);
    for k = 1:numel(r)
        if r(k) >= c(k)
            continue                       % the matrix reports both triangles
        end
        pairs{end + 1} = sprintf('%s <-> %s', ...
            bodyLabel(tree, r(k)), bodyLabel(tree, c(k)));              %#ok<AGROW>
    end
end
end

%% ------------------------------------------------------------------------
function nm = bodyLabel(tree, idx)
%BODYLABEL Body idx of a collision matrix, where index 1 is the tree's BASE.
%   The base's name is in BaseName and not in Bodies, so an unshifted lookup names the
%   wrong body for every index.
if idx == 1
    nm = tree.BaseName;
else
    nm = tree.BodyNames{idx - 1};
end
end

%% ------------------------------------------------------------------------
function [hit, minSep] = worldCollides(tree, q, env)
%WORLDCOLLIDES Body-to-world check only, with the true minimum separation.
%   IgnoreSelfCollision drops the nB x nB block entirely, so sepDist is nB x nWorld
%   and there are no artefacts in it to mask - the artefacts are all self pairs.
%   That makes this five times cheaper than the combined check, which is what lets
%   it run on every sample of the reference.
[~, sep] = checkCollision(tree, q, env, 'IgnoreSelfCollision', 'on', 'Exhaustive', 'on');
hit = any(isnan(sep(:)));
finite = sep(~isnan(sep));
if isempty(finite)
    minSep = 0;
else
    minSep = min(finite);
end
end

%% ------------------------------------------------------------------------
function model = withHeldObject(model, objectTarget, cfg)
%WITHHELDOBJECT The carried object as a body rigidly attached to the tool.
%   Returns a model whose two trees each carry one extra fixed-joint body holding a
%   collision box the size of what the arm is holding, so every check in the caller -
%   the candidate screening, the standoff ladder, the RRT search itself, the
%   self-collision path gate and the smoothed-curve gate - sees it without any of them
%   being changed. Widths are untouched: the joint is fixed, so the tree's DOF count,
%   every configuration vector and every Jacobian are exactly what they were.
%
%   WHERE THE OBJECT SITS RELATIVE TO THE TOOL IS NOT A GUESS. cfg.plan.graspDepth's note
%   states the offset the pick establishes and the release pays back: hmmGraspAim's rise
%   less half the object's height, below the tool frame. When the aim was a fixed 0.65 of
%   the height that was 21.1 mm for the shipped mug, and 20.9 mm was measured on the run
%   from the logged tool pose and the actor's manifest position - the same number twice
%   by two independent routes. Mid-height makes it the pad drop alone, 7.6 mm for that
%   mug, because the aim and the object's centre now coincide. Asking hmmGraspAim rather
%   than rebuilding the expression here keeps this correct for an object of any height
%   AND identical to what the model commands, which is the point.
%
%   AND IT FOLLOWS THE WRIST, WHICH IS THE WHOLE REASON A BODY IS THE RIGHT ANSWER. The
%   Sim3D attach is rigid: the object keeps its pose relative to the wrist for the entire
%   carry, so it rotates as the wrist rotates. A fixed-joint child of the tool frame has
%   exactly that behaviour for free, at every configuration the planner samples. A world
%   obstacle, or a carry offset applied along world z, would only be right while the wrist
%   happened to be vertical - and on the shipped run the wrist is 20 deg off vertical for
%   the whole carry, which is worth 18 mm on the object's lowest corner by itself.
%
%   THE FOOTPRINT IS SQUARE, for the same reason cfg.plan.surfaceBoxSideLim's slab is. The
%   jaws close across Extent(2) and the object's yaw about the tool axis is a property of
%   which grasp candidate won, so an oriented box would have to track a rotation this
%   function does not know at the time the body is built. The larger horizontal extent,
%   used for both, is the orientation-free bound and errs the safe way.
%
%   HORIZONTAL MARGIN ONLY, AND THE HEIGHT IS LEFT EXACT. A measured bounding box is
%   centred on the cluster's centroid, and on a mug the handle pulls that about 15 mm off
%   the part the jaws hold, so the box needs slack in the plane - see
%   cfg.plan.heldObjectMargin. It must NOT be inflated downward: the place goal deliberately
%   puts the object's underside cfg.plan.releaseClearance above the surface, and a box grown
%   downward would eat that clearance and make the plan collide at its own goal, which is
%   a plan that cannot be made rather than a collision avoided.
%
%   THE OBJECT IS EXCLUDED FROM EVERY SELF-COLLISION QUESTION BY BEING ABSENT FROM THE TREE
%   THAT ANSWERS IT, not by a mask row. It is held: it is inside the gripper by construction
%   and near the wrist by construction, so every self pair it takes part in is a false
%   positive, and the question this body exists to answer is object-against-world. So this
%   function re-points .fullTree/.planTree at augmented copies and leaves .fullTreeSelf and
%   .planTreeSelf - pinned at the top of the caller - on the originals. collides() then asks
%   the world question of the augmented tree with 'IgnoreSelfCollision','on' and the self
%   question of the un-augmented one with the unchanged mask, and the two configurations are
%   the same vector because the held joint is fixed.
%
%   MASKING WAS TRIED FIRST AND IT IS NOT IMPLEMENTABLE, which is why the design is this
%   one. checkCollision's separation matrix is indexed in the tree's INTERNAL body order,
%   and addBody inserts a new body immediately after its parent's existing subtree rather
%   than at the end of the tree. tree.Bodies and tree.BodyNames both append, and
%   tree.TreeInternal is not public, so the internal index simply cannot be read. Measured
%   on this tree: a held box parented to tcp_frame and placed 1.5 m up the tool axis reports
%   its separation on row 38 - the row Bodies order labels wrist_cam, because wrist_cam
%   hangs off gripper_mount and is displaced to the end - while row 39, the row Bodies order
%   calls hmmHeldObject, is Inf. A mask aimed at 39 therefore left the held object unmasked
%   against the gripper holding it and masked the harmless wrist_cam instead, and every
%   PLACE candidate was refused on six false finger-versus-object pairs with gate 3,
%   CollisionReject. No index is computed anywhere now.
%
%   See also hmmArmCollisionModel, cfg.plan.graspDepth.
h    = double(objectTarget.Extent(3));
side = max(double(objectTarget.Extent(1:2))) + 2 * cfg.plan.heldObjectMargin;

% THE PAD DROP AND THE AIM TOGETHER, FROM hmmGraspAim, so this transform cannot disagree with
% the grasp the model actually commands - they are now one expression with one implementation
% offline and one chain of blocks in MotionPlanning/TargetSelect.
%
% CLAMPED TO THE TABLE, NOT EXTRAPOLATED OFF IT. padDropTable is a measurement over the
% driver's working range; past its ends there is no measurement, and interp1's extrapolation
% of a curve that flattens at one end and steepens at the other would invent a pad drop for
% an object the jaws could not close on anyway. The Lookup Table block in
% MotionPlanning/TargetSelect clamps at its endpoints, so this matches what the model does.
[~, graspRise] = hmmGraspAim(cfg, [0; 0; h], double(objectTarget.GraspExtent));

% Along the TOOL's +z, which is the approach direction: the standoff above is built as
% graspPose * trvec2tform([0 0 -depth]), so -z is back up the approach and +z is toward
% the object. The centre hangs by the offset; the box then reaches h/2 further, putting
% the underside graspRise below the tool - the same rise the release pays back, which is
% what makes the object land releaseClearance clear.
Tobj = trvec2tform([0, 0, graspRise - 0.5 * h]);

% The SELF trees are deliberately not copied and not touched: they are the trees
% fullArtefact and planArtefact were measured against, and they stay that way.
model.fullTree = copy(model.fullTree);
model.planTree = copy(model.planTree);
model.fullTree = attachBox(model.fullTree, model.toolFrame, side, h, Tobj);
model.planTree = attachBox(model.planTree, model.toolFrame, side, h, Tobj);
end

%% ------------------------------------------------------------------------
function tree = attachBox(tree, parentFrame, side, height, pose)
%ATTACHBOX One fixed-joint body carrying a collision box.
%   No index is returned, and none is computed: see withHeldObject's header for why the
%   separation-matrix row this body occupies cannot be known from the public interface, and
%   why nothing here needs it.
body = rigidBody('hmmHeldObject');
setFixedTransform(body.Joint, pose);
addCollision(body, collisionBox(side, side, height));
addBody(tree, body, parentFrame);
end

%% ------------------------------------------------------------------------
function [hit, minSep] = collides(tree, selfTree, q, env, artefact, nB)
%COLLIDES Artefact-aware exhaustive collision check.
%   Exhaustive is required, not optional: the default stops at the first
%   contact, and the first contact is always one of the mesh artefacts, so a
%   non-exhaustive check can never see past them.
%
%   THE WORLD QUESTION AND THE SELF QUESTION ARE ASKED OF DIFFERENT TREES when the arm is
%   carrying something, and that is not an optimisation. tree carries the held object so
%   the world check sees it; selfTree does not, because artefact indexes selfTree's body
%   order and the augmented tree's order is NOT selfTree's with one row appended. The two
%   configurations are the same vector - the held object's joint is fixed, so it adds no
%   DOF - which is what makes two calls interchangeable with one. See withHeldObject.
%   The two trees are told apart by body COUNT rather than by handle identity: rigidBodyTree
%   is a handle class but isequal on it compares property values, and withHeldObject hands
%   back a copy, so identity is the wrong test and would silently take the wrong branch.
if tree.NumBodies == selfTree.NumBodies     % NumBodies, not numel(Bodies), which copies them all
    [~, sep] = checkCollision(tree, q, env, ...
        'Exhaustive', 'on', 'SkippedSelfCollisions', 'parent');
    selfPart  = sep(:, 1:nB);
    worldPart = sep(:, nB + 1:end);
else
    [~, worldPart] = checkCollision(tree, q, env, ...
        'IgnoreSelfCollision', 'on', 'Exhaustive', 'on');
    [~, selfPart] = checkCollision(selfTree, q, ...
        'Exhaustive', 'on', 'SkippedSelfCollisions', 'parent');
    selfPart = selfPart(:, 1:nB);
end

hit = any(isnan(selfPart(:)) & ~artefact(:)) || any(isnan(worldPart(:)));

finite = worldPart(~isnan(worldPart));
if isempty(finite)
    minSep = 0;
else
    minSep = min(finite);
end
end

%% ------------------------------------------------------------------------
function [q, qd, t, ok] = parameterise(W, velLim, accLim, nS)
%PARAMETERISE contopptraj first, trapveltraj as the fallback.
%   contopptraj respects the acceleration box as well as the velocity box, which
%   trapveltraj does not, so it is tried first. trapveltraj exists as the
%   fallback because contopptraj fits a spline and can fail to converge on a
%   path with a near-reversal in it, and a trapezoidal profile through the same
%   waypoints is still a valid reference - just a blunter one.
%
%   THE FALLBACK WAS DEAD FOR ITS WHOLE LIFE AND NOTHING SAID SO. It asked
%   trapveltraj for SEVEN outputs from a five-output signature, so it threw
%   "Too many output arguments" on every call, the catch below set ok = false,
%   and the caller reported ArmPlanFailed - which is exactly what it would have
%   reported if both tiers had honestly failed. That is the hazard of a bare
%   catch around a call whose only product is the flag the caller branches on:
%   a broken fallback and a declining fallback are the same observable event.
%   The signature is now pinned in the call below rather than absorbed, so the
%   next release that moves it fails loudly here instead of quietly upstream.
% All four outputs are defaulted before either try, so a parameterisation that
% throws in an unexpected place still returns a well-formed "not ok" rather than
% an unassigned-output error. ok is suppressed rather than dropped because it is
% the flag the caller branches on, and it is the one that must never be missing.
q = []; qd = []; t = []; ok = false;                                   %#ok<NASGU>

try
    [q, qd, ~, t, info] = contopptraj(W, velLim, accLim, NumSamples = nS);
    if info.ExitFlag == 1 && all(isfinite(q(:)))
        ok = true;
        return
    end
catch
    % fall through
end

try
    % PER SEGMENT, AND THAT IS NOT A REFINEMENT. trapveltraj's EndTime is the duration of
    % EACH segment, not of the trajectory: a scalar 3 on a two-segment path returns a 6 s
    % trajectory. This block used to sum travel over the WHOLE path, divide by the velocity
    % limit and hand the result to every segment, so a 32-waypoint plan - which
    % cfg.plan.armWaypointCap explicitly permits - would have been stretched by 31x. The
    % arm would have crawled through a perfectly good path until tApproachMax expired.
    %
    % An n-element EndTime is read as per-JOINT rather than per-segment, so per-segment
    % durations have to go in as the n-by-(p-1) form with each column constant. All joints
    % must share a segment's duration anyway: they arrive together or the path is not the
    % path that was collision-checked.
    seg  = abs(diff(W, 1, 2));                       % nArm x (nW-1), travel per segment
    % Slowest joint in each segment at its own limit, doubled to leave room for the
    % trapezoid's ramps, and floored so a near-zero segment still gets a sane sample grid.
    tSeg = max(max(seg ./ velLim(:, 2), [], 1) * 2, 0.5);
    [q, qd, ~, t] = trapveltraj(W, nS, EndTime = repmat(tSeg, size(W, 1), 1));
    ok = all(isfinite(q(:)));
catch
    ok = false;
end
end

%% ------------------------------------------------------------------------
function code = firstNonOk(code, cfg)
%FIRSTNONOK Report the target's own failure, or ArmPlanFailed if it had none.
if code == cfg.fail.OK
    code = cfg.fail.ArmPlanFailed;
end
end

%% ------------------------------------------------------------------------
function aplan = blankPlan(planTemplate, cfg, requestId, simTime)
%BLANKPLAN An ArmPlan that vouches for nothing.
aplan = planTemplate;
aplan.RequestId         = uint32(requestId);
aplan.Waypoints(:)      = 0;
aplan.NumWaypoints      = uint8(0);
aplan.RefTime(:)        = 0;
aplan.RefTime           = hmmPadRefTime(aplan.RefTime, 0);
aplan.RefPos(:)         = 0;
aplan.RefVel(:)         = 0;
aplan.NumSamples        = uint16(0);
aplan.Duration          = 0;
aplan.GraspPose         = eye(4);
aplan.PregraspConfig(:) = 0;
% THE ONE FIELD HERE THAT IS NOT ZERO OR EMPTY, and deliberately. ApproachDepth is the length
% ArmSupervisor ramps the approach over, so a blank plan carrying 0 would describe a descent of
% no length at all. An invalid plan is never acted on - Valid is false and the chart falls back
% to its own depthDefault - but a bus field that is read before it is checked should still hold
% the value that would be safe, and the safe value is the standoff this function used before
% the search existed.
aplan.ApproachDepth     = cfg.sup.approachDepth;
aplan.GraspConfig(:)    = 0;
aplan.SelectedCandidate = uint8(0);
aplan.PoseErrorNorm     = 0;
aplan.Manipulability    = 0;
aplan.MinClearance      = 0;
aplan.ApertureCmd       = cfg.gripper.maxOpenPos;
aplan.AttachActorId     = cfg.actor.noneId;
aplan.PlanTime          = 0;
aplan.Truncated         = false;
aplan.FailureCode       = cfg.fail.ArmPlanFailed;
aplan.Timestamp         = simTime;
aplan.Valid             = false;
end

%% ------------------------------------------------------------------------
function cfg = cachedConfig()
cfg = hmmRuntimeConfig();
end
