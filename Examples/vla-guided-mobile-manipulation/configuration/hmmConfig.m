function cfg = hmmConfig()
%HMMCONFIG Single configuration source for the home mobile manipulator.
%
%   cfg = HMMCONFIG() returns every tuneable property of the ROBOT and the
%   RIG: frame conventions, chassis and wheel geometry, gripper limits,
%   camera calibration, ray-trace depth grid, velocity and acceleration
%   limits, safety clearance, and render appearance.
%
%   Task data is forbidden here. This file therefore contains NO object
%   identity, NO object pose, NO object extent and NO destination region.
%   Object identity, pose and graspable extent are derived at runtime
%   from perception and travel on the ObjectTarget and PlaceTarget buses.
%
%   See also assembleMobileManipulator, hmmDefineBuses.

%   Copyright 2026 The MathWorks, Inc.

%% Frame conventions (one documented source, never re-derived)
cfg.frames.lengthUnit      = 'm';
cfg.frames.angleUnit       = 'rad';
cfg.frames.handedness      = 'right';
cfg.frames.eulerOrder      = 'ZYX';         % [yaw pitch roll]
cfg.frames.imageOrigin     = 'top-left';    % row 1 is the top image row
cfg.frames.worldUp         = [0 0 1];
cfg.frames.gravity         = [0 0 -9.80665];
cfg.frames.chain = ['Sim3D world -> navigation map -> mobile base -> pedestal -> ' ...
    'UR5e links -> tool0 -> coupling -> 2F-85 -> pads/grasp frame -> wrist camera'];

%% Sample times (one clock definition, inherited everywhere downstream)
cfg.rates.plant            = 0.01;   % base integration and robot actor update, s
cfg.rates.control          = 0.01;   % base/arm/gripper controllers, s
cfg.rates.supervisor       = 0.02;   % Stateflow supervisory charts, s
cfg.rates.perception       = 0.10;   % camera capture and reconstruction, s
% THE SPEED/SMOOTHNESS KNOB. The engine wait and the robot pose push both scale with this rate
% (the RobotTree actor and ReleaseSettle run on it too) and nothing in control or perception
% reads it. Must stay an integer multiple of 0.01 AND an exact divisor of cfg.rates.perception,
% which leaves 0.02, 0.05 and 0.10. 0.05 measured 1.28x faster on a replay transfer (348.2 ->
% 272.6 s, behaviour identical) but the user found the 20 Hz render too choppy, so it ships at
% 50 Hz.
cfg.rates.scene            = 0.02;   % Sim3D engine step, s
% Planning services are triggered/enabled, not periodic, but their contents
% still need a sample time and a planner has no business re-solving at 20 ms.
cfg.rates.planning         = 0.50;   % base and arm planning services, s

%% Mobile base - myAGV Pro omnidirectional (mecanum) chassis
% EVERY NUMBER IN THIS BLOCK IS MEASURED FROM THE VENDOR CAD, not chosen. The source
% is elephantrobotics/agv_pro_ros2, branch humble, package agv_pro_description 1.0.8
% (BSD-3-Clause); the meshes are downloaded unmodified into robots/meshes/agv_pro, and
% robots/meshes/agv_pro/PROVENANCE.md records the bounding boxes these came from and
% how the wheel-to-corner mapping was measured rather than read off the vendor's own
% link names, which are self-contradictory.
%
% THE EARLIER VALUES WERE INVENTED, AND NOT ALL OF THEM WERE CLOSE. wheelRadius was
% 0.0375 against a measured 0.0761 - a factor of two - and since wheelRadius divides
% wheelMatrix below, every wheel speed the saturation check compared against
% wheelSpeedMax was twice what the real chassis would turn. chassisSize was
% [0.44 0.34 0.11] against a measured [0.531 0.351 0.220], so the chassis is 91 mm
% longer and 110 mm taller than the box that stood in for it.
cfg.base.wheelRadius       = 0.0761;  % m, measured from wheel_*_link.stl
cfg.base.halfLength        = 0.1715;  % m, lx: base_link to axle along +X
cfg.base.halfWidth         = 0.1799;  % m, ly: base_link to wheel centre along +Y
cfg.base.rollerSign        = 1;       % +1 = front-left roller at +45 deg
cfg.base.chassisSize       = [0.531 0.351 0.220];  % m, collision box LxWxH
cfg.base.chassisMass       = 19.236;  % kg, vendor base_link inertial mass
% THE DECK IS 20 mm BELOW THE TOP OF THE COLLISION BOX, AND ANYTHING BOLTED TO IT HAS
% TO BE REFERENCED HERE RATHER THAN TO chassisSize(3). The chassis box is a bounding
% box, so its height 0.220 is the RAIL CROWN: two mounting rails run fore-aft at
% y +/-0.0875..0.1275 and stand 20 mm proud of the deck between them. The pedestal is
% 0.120 m wide and sits in that 0.175 m channel, so its foot lands on the deck floor,
% not on the crowns - and referencing it to chassisSize(3) left it hanging 20 mm in the
% air, which is visible in every render of the robot. Measured from the raw chassis
% triangulation, because rails are a CONCAVITY and a convex hull of the chassis fills
% the channel in and reports a flat deck at the crown.
%
% THE INVARIANT ABSORBS THIS IN THE PEDESTAL LENGTH, NOT IN THE ARM HEIGHT. Lowering
% the reference by 20 mm while holding cfg.pedestal.armMountHeight makes the pedestal
% 20 mm longer and leaves the UR5e base exactly where it was measured, so the stow
% pose, the stand-off band and the four certified plan quantities are untouched. The
% alternative - holding the pedestal length and letting the arm drop - would have moved
% the invariant and invalidated all of them to fix an appearance defect.
cfg.base.deckHeight        = 0.200;   % m, chassis deck floor between the rails
% base_link SITS 24.1 mm ABOVE THE FLOOR, AND THAT IS DERIVED FROM THE WHEELS RATHER
% THAN FROM THE VENDOR'S OWN base_footprint. The URDF puts base_link 0.020 above
% base_footprint, but its wheel joints sit 0.052 above base_link on a measured 0.0761
% radius, which puts the contact patch 0.0241 below base_link - 4 mm of disagreement
% inside the vendor's own file. The wheels are what touch the floor, so they win, and
% the two conventions then reconcile exactly: assembleMobileManipulator mounts each
% wheel at wheelRadius - baseLinkHeight = 0.0761 - 0.0241 = 0.0520 relative to the
% chassis, which is the URDF's wheel origin to four decimal places.
cfg.base.baseLinkHeight    = 0.0241;  % m, base_link origin above the floor
% Where the robot is parked when the session starts. This is a property of the
% RIG, not of a task: it is a spot on the apartment's open floor that no prompt
% chooses and no planner assumes, and it was checked against the runtime
% occupancy map during development rather than asserted here.
%
% THE HEADING IS pi AND IT USED TO BE ZERO, WHICH PARKED THE ROBOT NOSE-TO-A-WALL.
% The spot is fine at every heading - it is open floor, and the footprint validates at
% 0, 45, 90, 135, 180, 225 and 270 deg with and without a held mug - so validity does
% not choose the yaw and something else has to. Ray-casting the shipped map from this
% spot gives 0.65 m of free floor dead ahead at theta = 0, against a footprint that
% reaches cfg.base.geom.forward = 0.500 m forward: the robot SPAWNED with 0.15 m of
% air in front of its bumper and a wall filling its view.
%
% WHY pi AND NOT 90 deg, WHICH IS THE CLEAREST BEARING OF ALL. The same sweep reads
% 0.65 m at 0, 0.75 at 30, 1.30 at 60, 5.95 at 90, 3.75 at 120, 1.50 at 150, 2.45 at
% 180. So 90 deg has the most room by a wide margin and is still the wrong answer,
% because it is SIDE-ON to the wall rather than backed onto it: a robot parked across
% its own dock reads as abandoned mid-manoeuvre, not as parked. pi puts the back to
% the wall with 2.45 m of open floor ahead, which is what a docked robot looks like.
%
% AND IT MAKES THE SPAWN AGREE WITH THE HOME VERB, WHICH IS THE REAL DEFECT IT FIXES.
% cfg.task.homePose is already [x y pi] and its comment measured this same wall and
% rejected 90 deg for this same reason. So "go home" already meant back-to-the-wall
% while the session STARTED nose-to-it - the robot began every run in a pose its own
% home verb would immediately have corrected, and "return home" moved it somewhere it
% had never been. The two are now the same pose, and cfg.task.homePose still derives
% its x and y from this line so they cannot drift apart.
%
% IT IS ALSO WORTH ABOUT 150 deg OF OPENING ROTATION. The pick stand-off is reached at
% yaw 163.9 deg, so from theta = 0 the first leg turned 164 deg before it was pointed
% at anything; from pi it turns 16 deg.
cfg.base.initialPose       = [5.2; -4.0; pi];    % [x y theta] world, m/m/rad

% Per-axis limits. The base is holonomic: vx, vy and omega are independent
% and are commanded independently. Never collapse to (v,omega).
% RAISED 25 % ON 2026-10-14 - all six together, so every ratio the margins below were sized on
% (planMargin 0.85 < slewMargin 0.90, cmdMargin 0.95) is unchanged; the figures quoted in those
% notes (0.68 m/s, 1.02 and 1.08 m/s^2) scale by 1.25 with them. SafetyMonitor's TwistLimits and
% AccelLimits read these same fields, so the guard moved with the robot. Straight at 1.0 m/s a
% wheel turns 13.1 rad/s against wheelSpeedMax 24.
cfg.base.vxMax             = 1.00;    % m/s   (was 0.80)
cfg.base.vyMax             = 1.00;    % m/s   (was 0.80)
cfg.base.omegaMax          = 1.875;   % rad/s (was 1.50)
cfg.base.axMax             = 1.50;    % m/s^2 (was 1.20)
cfg.base.ayMax             = 1.50;    % m/s^2 (was 1.20)
cfg.base.alphaMax          = 3.75;    % rad/s^2 (was 3.00)
cfg.base.wheelSpeedMax     = 24.0;    % rad/s, per-wheel saturation

% MobileBase/BaseAccelLimit slews the commanded twist at this FRACTION of the
% acceleration box above, so the command sits strictly inside the envelope
% SafetyMonitor/BaseLimitChecks guards. Two reasons it is not 1.0. The monitor
% differentiates the REALISED twist and compares with a strict >, so commanding
% exactly axMax leaves the verdict to a round-off. And keeping a margin means a
% BASE_ACCEL trip says the PLANT failed to follow a legal command, which is a
% fault worth latching, rather than that the commander asked for the limit.
cfg.base.slewMargin        = 0.90;    % fraction of the axMax/ayMax/alphaMax box

% hmmPlanBasePath time-parameterises the base path at this FRACTION of BOTH boxes
% above, exactly as cfg.arm.planMargin does for the arm - and its absence was a
% run-terminating defect, not a refinement.
%
% THE VELOCITY BOX HAD NO MARGIN ANYWHERE, WHILE THE ACCELERATION BOX HAD TWO.
% slewMargin above and cfg.arm.planMargin below both exist because a commander that
% asks for exactly the limit leaves a strict > to a round-off. Nobody applied that
% reasoning to velocity, so hmmPlanBasePath handed contopptraj the raw vxMax/vyMax/
% omegaMax and the reference rode the bound. A navigate leg then died like this:
%
%   t = 1.770  vx = 0.80000000000000027   <-- Plant/.../TwistLimit clamped to 0.8
%   t = 1.780  BASE_VEL, latched E-stop, task aborted after 1.366 m
%
% The excess is TWO ULPS - 2.2e-16 m/s - and it is not a tracking error at all. The
% plant hard-clamps its own twist at this box, and the mecanum inverse->forward round
% trip through cfg.base.wheelMatrix re-emerges a couple of bits above where the clamp
% put it. In the same sample omega sat at 1.4999999999999987, one ulp BELOW its bound,
% and did not trip. Two channels pinned to their limits, and which one raised the
% E-stop was decided by the direction the last bit rounded.
%
% SO THIS CONSTANT IS NECESSARY AND NOT SUFFICIENT, and saying so here is the point.
% It stops the PLANNER asking for the clamp. It cannot stop PathFollowing's PID from
% reaching its own +/-vxMax clamp on a large enough tracking error, because that
% saturation was set to the very same box the monitor guards. That second gap is
% cfg.base.cmdMargin below, and it was a real run-terminating defect too.
%
% 0.85 AND NOT 0.90: the planned acceleration then sits at 1.02 m/s^2, strictly under
% the 1.08 that slewMargin lets BaseAccelLimit pass, so the slew limiter never clips a
% ramp the planner asked for and the reference the follower tracks is the reference the
% planner published. At 0.90 the two would be equal and clipping would be marginal
% again - the same mistake one level up. The cost is duration on a velocity-limited
% stretch, about 1/0.85, against an E-stop that costs the whole task.
cfg.base.planMargin        = 0.85;    % fraction of BOTH base boxes, for the planner

% MobileBase/PathFollowing/VelocityLimit saturates the COMMANDED twist at this
% fraction of the velocity box, and it is the third and last place the same argument
% belongs. slewMargin keeps the commanded ACCELERATION inside the acceleration box;
% planMargin keeps the planned REFERENCE inside both boxes; neither constrains what
% the follower's three PID channels ADD on top of that reference.
%
% AND THE FOLLOWER IS EXACTLY WHERE THE 40 s RUN DIED. The pick leg peaked at 98% of
% vxMax and survived. The place leg reached 100.0% and latched BASE_VEL at t = 20.690 s,
% 2.79 s into the drive: base STOPPED/FAILED, arm STOPPED/FAILED, gripper FAULTED,
% attachedActor 12 -> 0 - the mug dropped in mid-transfer - and Outcome ABORTED. It was
% not a stall (BaseStatus.Stalled stayed 0) and not the planner, whose reference rides
% at planMargin*vxMax = 0.68 m/s. The PID correction on top of it drove the command into
% VelocityLimit's clamp, which was set to the RAW box, so the commander was permitted to
% ask for precisely the value SafetyMonitor/BaseLimitChecks calls a violation.
%
% THE MONITOR IS RIGHT AND WAS NOT CHANGED. blk_928 TwistLimits compares abs(realised
% twist) with a strict > against the raw box, which is what a safety limit should be:
% the envelope, not the envelope plus a fudge. Widening it with a tolerance - the fix
% the note above used to propose - would have bought the same 2-ulp immunity while
% making every future genuine excursion negotiable. Narrowing the COMMAND instead keeps
% the monitor's verdict meaning what it says: a BASE_VEL trip is now the plant failing
% to follow a legal command, never the commander asking for the limit.
%
% 0.95 AND THE ORDERING THAT MATTERS: planMargin 0.85 < cmdMargin 0.95 < 1. The gap
% below 1 is 0.040 m/s, eleven orders of magnitude above the 2.2e-16 round-off that
% killed the earlier run. The gap above planMargin is 0.080 m/s, which is the PID
% authority left for tracking error on a velocity-saturated stretch - measurably more
% than the 0.016 m/s peak correction the surviving pick leg needed. Reversing the two
% would starve the follower of authority to track the reference the planner published,
% which is a tracking failure traded for a safety trip and not a fix.
cfg.base.cmdMargin         = 0.95;    % fraction of the velocity box, for the follower

% Mecanum inverse (twist -> wheel) and forward (wheel -> twist) matrices.
% Wheel order: [front-left; front-right; rear-left; rear-right].
% This is wheel-geometry arithmetic, not a reimplemented algorithm: no
% Mecanum Kinematics block exists in this release (verified against
% robotmobilelib, which ships Ackermann/Bicycle/DifferentialDrive/Unicycle).
lxy = cfg.base.halfLength + cfg.base.halfWidth;
s   = cfg.base.rollerSign;
cfg.base.wheelMatrix = (1/cfg.base.wheelRadius) * ...
    [1, -s, -lxy; ...
     1,  s,  lxy; ...
     1,  s, -lxy; ...
     1, -s,  lxy];                                    % 4x3
cfg.base.twistMatrix = pinv(cfg.base.wheelMatrix);    % 3x4

%% Pedestal carrying the UR5e on the chassis top face
% 0.440 m IS THE SMALLEST CHANGE THAT MAKES THE TASK POSSIBLE AT ALL, and it is
% measured rather than chosen. At the original 0.240 m there were ZERO complete
% transfers: every one of the nine props that fit the jaws at a workable height,
% against all 517 place points the model's own support rule accepts, at every
% stand-off from 0.62 to 0.94 m. The arm mounted at 0.335 m and a place at 0.910 m
% needed more than its 0.850 m reach from any stand-off the base planner would also
% accept, so the run always ended in PathPlanFailed or ArmPlanFailed. This is the
% change hmmSetup's own counterOutOfReach warning has been asking for.
%
% AND THE CEILING IS A 0.780 m ARM MOUNT, MEASURED, NOT A PREFERENCE. The stowed arm
% rises with the deck one for one, and a stow pose is only acceptable if it stays inside
% cfg.depth.occupancyBand - an arm parked above the band carries the gripper at the
% height of wall cabinets the base planner cannot see. Measured on the assembled tree
% with the shipped cfg.arm.stowConfig, the topmost body the arm moves sits 0.420 m
% above the arm mount, so at a 0.645 m mount the arm parks at 0.810 .. 1.065 m and has
% 0.135 m of headroom under the 1.20 m band top. Above a 0.780 m mount it breaks out,
% and measureStowedEnvelope then reports a SMALLER radius purely because the topmost
% bodies have left the window it scans - the mismeasurement the in-band rule on stow
% poses exists to refuse, and why the in-band and the unfiltered radius were compared
% side by side (both 0.3046 m, which is what agreement looks like). THOSE THREE HEIGHTS
% BELONG TO THE SUPERSEDED 0.645 m MOUNT AND ITS TILTED FOLD; the shipped mount is
% 0.4441 m and the shipped fold extends vertically, so it reaches 0.420 m higher above
% its own mount and the ceiling is 0.5601 m. See the FOOTPRINT block at the assignment
% below for the re-measured figures. The ARGUMENT in this paragraph is unchanged and is
% the reason the new ceiling was re-measured rather than assumed. Going higher
% therefore means re-deriving the stow pose, which changes cfg.plan.stowedArmRadius,
% which changes the planner's inflation, which invalidates the stand-off below.
%
% THAT CEILING ROSE BY 92 mm WHEN THE STOW POSE WENT VERTICAL, which matters because the
% pedestal height is listed below as future work for reach. The tilted pose parked at
% 0.663 .. 1.157 m and left 43 mm of headroom, capping the mount at 0.688 m; the vertical
% pose is a tighter fold and parks 92 mm lower at the top, so the headroom is 135 mm and the
% cap is 0.780 m. That headroom was recorded as not worth spending - and it was then spent in
% the OPPOSITE direction, because the footprint turned out to be worth more than the reach:
% the mount came DOWN by 200.9 mm so the fold could open out vertically instead of splaying
% sideways. See the FOOTPRINT block at the assignment below.
%
% What it costs: the place at 0.910 m still uses 92% of the reach. That 92% is a slant
% radius from the stand-off, not the 0.265 m of height above the mount - height alone
% would read 31% and would be measuring the wrong triangle - so hmmSetup measures it
% against the assembled tree and prints it on every setup. A taller pedestal with a
% re-derived stow pose would buy some of it back, and that trade is left as future
% work rather than taken here; the 0.043 m of headroom available without
% re-deriving the pose is not enough of it to matter.
%
% THE INVARIANT IS THE ARM MOUNT HEIGHT, NOT THE PEDESTAL LENGTH, which is why the
% length below is derived and armMountHeight is the number written down. Everything
% the block above establishes - the stow pose, cfg.plan.stowedArmRadius, the 0.3046 m
% envelope, cfg.sup.baseStandoff and its measured [0.705 0.880] band - is a function
% of how high the arm sits above the FLOOR. Importing the vendor chassis changed two
% of the three terms underneath it: baseLinkHeight 0.095 -> 0.0241 and chassisSize(3)
% 0.11 -> 0.220, a net deck rise of 39.1 mm. Holding cfg.pedestal.height at 0.440
% would have carried that straight into the arm and silently invalidated the whole
% chain; deriving the length instead absorbs it and leaves the arm exactly where it
% was measured. The invariant was also confirmed numerically against the assembled
% tree rather than trusting this arithmetic.
%
% NOTE ON THE 0.335 IN THE FEASIBILITY PARAGRAPH: it omits chassisSize(3). With the
% original 0.240 m pedestal the arm mounted at 0.095 + 0.110 + 0.240 = 0.445 m, not
% 0.335 m. The RELATIVE argument that paragraph makes survives - the reach shortfall
% it reports was measured on the tree, not derived from that figure - but the absolute
% height is not the tree's, and the value below is. The ceiling paragraph's heights
% were re-measured against the assembled tree and are.
% FOOTPRINT IS WHY THE INVARIANT MOVED, AND IT MOVED DOWN. Everything above argues about
% how much HEIGHT the mount can afford, because reach was the binding quantity while the
% stow pose was a tilted fold. It was not: the stowed arm at the 0.645 m mount measured
% halfExtent [0.3155 0.3474] and an envelope radius of 0.4136 m, which drove
% hmmBaseFootprint to 0.4531 m of inflation against the 0.300 m ceiling measured for
% reaching all four rooms. A robot that cannot carry anything out of two of its own
% rooms has a worse problem than 6 % of reach.
%
% HALVING THE PEDESTAL IS WHAT LETS THE FOLD STAND UP. At 0.4441 m the arm's two long links
% extend nearly vertically - one up, one down - instead of splaying out sideways to stay
% under the band, and the whole stowed robot comes in at halfExtent [0.2616 0.1823], radius
% 0.3014 m and inflation 0.2899 m: 163.2 mm better than what shipped and 10.1 mm under the
% ceiling. The arm parks 0.4329 .. 1.0840 m, so 116.0 mm of headroom remains and the ceiling
% on this mount is 0.5601 m rather than 0.780 m - a taller ceiling was never the point.
%
% AND THE REACH IT WAS SUPPOSED TO COST GOT BETTER ON THE AXIS THAT WAS TIGHT. Measured on
% the assembled tree at the configured stand-offs: the 0.55 m pick goes from 93.3 % to
% 88.5 % of cfg.arm.reach and the 0.90 m place from 78.4 % to 85.0 %. Both are inside the
% 0.94 manipulability limit, and the one that was nearly at it is the one that improved,
% because lowering the shoulder shortens the slant to the LOWER surface. The two fractions
% pull in opposite directions across a sweep of pedestal heights, and 0.4441 m is the
% height where neither binds.
%
% THE SIDE-ON ARRIVAL IS A REQUIREMENT OF THIS LAYOUT, NOT A PREFERENCE, and it is the one
% real cost. cfg.pedestal.offset below puts the mount 60 mm AFT of the base centre, so
% cfg.sup.placeRadiusMax - a base-centre radius measured with the mount forward - is only
% 0.7550 m mount-relative. Reaching the place surface nose-on now needs 0.8100 m and FAILS;
% arriving with the base turned 90 deg needs 0.7250 m, 85.3 % of reach, and passes. The
% planner's own 5-circle model is what makes that free: hmmBaseFootprint demands
% forward + inflation = 0.5023 m of clearance nose-on and only inflation = 0.2899 m side-on,
% so the base parks 212.4 mm closer sideways. Both stand-off floors below were measured
% NOSE-ON along a single bearing and are therefore the wrong floors for a side-on park -
% both have to be re-measured with the bearing free.
cfg.pedestal.armMountHeight = 0.4441;  % m, UR5e base above the floor - the invariant
% Referenced to cfg.base.deckHeight and NOT to chassisSize(3), because the pedestal
% stands on the deck floor between the rails rather than on the rail crowns. See the
% deckHeight note above: the 20 mm this adds to the length is 20 mm the pedestal used to
% spend floating, and the arm mount does not move.
cfg.pedestal.height        = cfg.pedestal.armMountHeight - ...
    (cfg.base.baseLinkHeight + cfg.base.deckHeight);       % m, above the chassis deck
%
% THE MOUNT IS POINT-REFLECTED THROUGH THE BASE CENTRE, WHICH IS FREE AND IS NOT A ROTATION
% OF THE BASE. The chassis, the wheels and laser_link do not move: forward stays the end the
% scanner looks out of. What turns around is the pedestal-and-arm assembly, so the arm folds
% FORWARD over the deck the scanner faces rather than aft. Reflecting a mount through the
% centre maps the arm's projected hull onto its own mirror image, so |x| and |y| are
% preserved exactly and the footprint above is unchanged by it - the offset is negated in x
% and y, and the stow pan turns by pi, which is why cfg.arm.stowConfig(1) is now 0.
%
% THE -60 mm IS THE REFLECTION OF THE SHIPPED +60 mm AND THE LASER CAPS THE OTHER DIRECTION.
% laser_link occupies x +0.1417 .. +0.2084 on the deck, so the mount cannot go forward past
% +0.0817 m; aft there is nothing in the way. At -0.060 the stowed arm sits 34.5 mm inside
% the front bumper and 67.7 mm inside the rear one, and clears the scanner by 235.8 mm of
% height.
%
% THE -27.5 mm IS A CENTRING OFFSET AND IT IS CAPPED BY A 20 mm RIDGE. Half-extents are
% measured from the CENTRELINE, and this fold sits entirely on one side of it, so moving the
% mount towards the arm reduces the charge directly in a way no refold can. Footprint keeps
% improving out to 74.9 mm, where the arm would be exactly centred - but the deck carries two
% fore-aft mounting rails and the pedestal is 0.120 m wide in a 0.175 m channel, so past
% 27.5 mm the mount straddles a rail. That cap misses the wheel-track plateau by 2.4 mm and
% costs 2.3 mm of footprint. The channel is measured from the raw chassis triangulation,
% because rails are a CONCAVITY and a convex hull fills them in.
cfg.pedestal.offset        = [-0.060 -0.0275 0];   % m, aft and lateral offset on the deck
cfg.pedestal.yaw           = 0;       % rad, arm base yaw about chassis +Z

assert(cfg.pedestal.height > 0, 'hmmConfig:pedestalBelowDeck', ...
    ['A %.3f m chassis deck leaves no room for the arm under a %.3f m mount ' ...
     'height. Either the chassis geometry or the invariant has to move, and ' ...
     'moving the invariant means re-deriving cfg.arm.stowConfig and the ' ...
     'reach limits for the new mount height.'], ...
    cfg.base.baseLinkHeight + cfg.base.deckHeight, cfg.pedestal.armMountHeight);

%% UR5e manipulator
cfg.arm.robotName          = 'universalUR5e';
cfg.arm.dataFormat         = 'row';
cfg.arm.nJoints            = 6;
cfg.arm.reach              = 0.850;   % m, published UR5e reach

% THE REACHABLE LEVER, MEASURED, AND WHY IT IS A TABLE RATHER THAN A RADIUS. cfg.arm.reach
% is a published sphere about the shoulder and the arm's real working set is not a sphere:
% the tool is held VERTICAL for every grasp and release this example performs, so the
% gripper's own length is spent on HEIGHT and never on radius, and what a stand-off has to
% satisfy is a horizontal lever that shrinks as the target rises. leverCeiling is that
% measured curve - row 1 the tool's height ABOVE THE ARM MOUNT, row 2 the largest horizontal
% mount-to-target distance at which inverseKinematics solves on hmmArmCollisionModel's
% planTree, with a pose cfg.sup.approachDepth above it also solving, since no verb ever
% commands the grasp without first commanding the approach.
%
% THE INDEX IS MOUNT-RELATIVE, AND THAT IS THE WHOLE POINT OF THE PARAMETERISATION. A table
% keyed on the tool's height above the MOUNT is a property of the UR5e and the 2F-85 alone,
% so it survives every change to cfg.pedestal.armMountHeight, the pedestal height and the
% deck. cfg.sup.placeRadiusMax is the same measurement keyed the other way - a BASE-CENTRE
% radius, measured at one mount height with the mount offset forward - and lowering the mount
% by 200.9 mm silently invalidated it. It did not become wrong, it became MIS-INDEXED, and
% this table reproduces it exactly where it was taken: at dz = 0.35, which is where the
% 0.645 m mount put the shipped place pose, the ceiling reads 0.755 m, and 0.755 m is
% placeRadiusMax's 0.815 m less the 0.060 m the mount then stood forward. A constant that can
% be re-derived from a curve is a sample of that curve, and the curve is what to keep.
%
% REACH IS EXACTLY INDEPENDENT OF THE BEARING THE TARGET LIES ON, which is why one lever
% answers for every heading. planTree is the arm subtree rooted at arm_mount and joint 1
% rotates about that frame's vertical axis with +/-2pi of travel, so a target moved round the
% mount in azimuth is absorbed by joint 1 and by nothing else - measured at 1e-8 pose error
% across twelve azimuths at a fixed lever, and the identical arm configuration reappears with
% joint 1 offset by the azimuth. The two azimuths that missed did so because the SEED was
% left at the stow pose 150 deg away, not because the pose was unreachable; rotating the seed
% by the azimuth solved both to 1e-9. Reach is therefore a function of (lever, height) and
% the arrival heading enters only through the lever, which is what hmmStandoffRing exploits.
%
% THE FLOOR IS FLAT AND THE CEILING IS NOT, so only one of them needs a table. Below
% leverFloor the arm cannot fold tightly enough to put a vertical tool that close to its own
% mount, and that limit is set by the link lengths rather than by the height: it measured
% 0.130 m at every sampled height from 0 to 0.65 m. Above 0.65 m nothing solves at any
% lever, which is why the table stops there rather than being extrapolated.
%
% Measured on a 5 mm lever grid with cfg.plan.ikMaxIterations, no random restarts and
% cfg.plan.poseErrorMax as the accept test - the solver hmmScreenArmPlan actually uses, not a
% more generous one - and the admissible set was CONTIGUOUS at every height, so interpolating
% between breakpoints is sound rather than a convenience.
cfg.arm.leverCeiling       = [0.00 0.05 0.10 0.15 0.20 0.25 0.30 0.35 0.40 0.45 0.50 ...
                              0.55 0.60 0.65; ...
                              0.915 0.905 0.890 0.870 0.850 0.825 0.790 0.755 0.715 ...
                              0.665 0.605 0.530 0.435 0.280];   % m, [dz; maxLever]
cfg.arm.leverFloor         = 0.130;   % m, closest a vertical tool gets to its own mount

% THE STOW POSE IS A NAVIGATION PARAMETER, AND IT IS ALSO A TRAJECTORY ONE. It sets
% cfg.plan.stowedArmRadius, which hmmPlanBasePath inflates the occupancy map by, so
% it decides how close the base may be planned to a counter - and therefore whether
% the arm can reach anything standing on one. The earlier
% [0 -1.85 2.20 -2.00 -1.5708 0] parked the arm splayed out at 0.513 m from the base
% origin, giving a 0.628 m inflation against a 0.55 m working stand-off: a robot
% wider while driving than the distance at which it has to work cannot service a
% surface at all, and every base plan beside a mapped object failed the goal-state
% check as PathPlanFailed.
%
% THE FOLD IS THE STOW SEARCH'S FAMILY, VERTICALLY EXTENDED; THE POSE IS NOT THE SEARCH'S
% OUTPUT. The search minimises the radius that measureStowedEnvelope reports subject to the
% arm staying inside cfg.depth.occupancyBand, above its own deck, free of any self-collision
% the assembled robot does not already have, and with the tool pointing straight down - and
% a RADIUS cannot see azimuth, which is most of what a footprint rectangle is. Its winner
% measured 0.4136 m of envelope radius and [0.3155 0.3474] of half-extent once the meshes
% rather than the frame origins were measured, for 0.4531 m of inflation.
%
% So this pose keeps that fold's SHAPE and moves the three things the search had no term
% for: the pedestal came down to 0.4441 m so joints 2 and 3 could open the fold out
% vertically instead of splaying it sideways (swept as exactly that two-parameter
% extension, paired so the tool stays vertical for free), the mount was point-reflected
% onto the forward half of the deck and offset 27.5 mm towards the arm, and joint 6 was
% rolled to square the wrist camera to the travel axis. It measures 0.3014 m of envelope
% radius - inside the 0.3183 m chassis circle, so max(0, radius - baseRadius) is ZERO and
% it adds nothing at all to the driving envelope - and it parks the arm between 0.4329 m
% and 1.0840 m of the 0.05 .. 1.20 m band.
%
% AND THE WRIST NOW POINTS DOWN, WHICH IS WHAT THIS POSE WAS LAST CHANGED FOR. The
% previous [0 -2.175 2.779 -1.898 -1.792 0] parked the tool 20.20 deg off vertical -
% tool z was [-0.2666 -0.2194 -0.9385] in the arm frame, tilted in both x and y, so it
% was not even a planar fold. It looked wrong, which is how it was noticed, and only the
% first of its three costs was cosmetic. The CARRIED OBJECT inherits the tilt, because
% the Sim3D attach is rigid: the mug measured 20.2 deg off vertical for the whole carry,
% which drops the lowest corner of its bounding box 18.4 mm below where an upright mug's
% would sit. And every descent began by undoing the tilt, so the wrist joints paid for it
% twice per task.
%
% THE DIFFERENCE IS JOINT 5, NOT THE FOLD. -1.792 rad against this pose's -1.571, a
% quarter of a radian on one joint, and the fold - joints 2, 3 and 4 - is what moved to
% absorb it. Every start the stow search tried already set joint 5 to -pi/2 and every
% one of them reached 0.00 deg of tilt once the tilt term was priced in, so the vertical
% family was never far away; nothing had ever asked for it. The winner costs 25 mm of
% stowed radius over the pose it replaces, which the chassis circle absorbs entirely,
% and it parks 92 mm lower at the top.
%
% JOINT 1 IS ZERO, AND THAT IS NOT A RETURN TO THE SEARCH'S ANSWER - IT IS A REFLECTION OF
% THE FOOTPRINT SWEEP'S. That sweep grids the pan over the full circle at the lowered
% pedestal and picks +pi: the arm folded across the deck rather than out from it.
% Point-reflecting the mount through the base centre then turns the pan by pi again, because
% a reflection is the composition of that rotation with the offset negation, and pi + pi is
% 0. The pose that ships is therefore the swept winner seen from the other end of the
% chassis, and joint 1 reading 0 here is arithmetic rather than a decision not to rotate.
%
% WHAT ROTATING JOINT 1 COSTS AND WHAT IT BUYS, WHICH IS WHY IT IS SWEPT AND NOT REASONED
% OUT. Joint 1 turns about the vertical, so it leaves the tool's TILT, the tool's height, the
% arm's z band and the fold itself exactly where the stow search left them - every quantity
% the search certified is invariant under it. What it moves is the AZIMUTH of the whole
% stowed assembly, and azimuth is precisely what a footprint rectangle is made of and what a
% radius cannot see.
%
% THERE ARE TWO BULGES AND THEY FIGHT, WHICH IS THE PART THAT IS NOT OBVIOUS AND THAT A
% FIRST ATTEMPT AT THIS GOT WRONG. A vertical wrist is not on the arm's centreline: the UR5e
% carries its tool 0.1914 m to the side of joint 1's axis at any fold that points the tool
% down, because d4 and d6 offset the wrist and q5 = -pi/2 turns that offset sideways. So a
% held object's centre bulges one way and the forearm and wrist bulge another, and the angle
% that zeroes the first makes the second worse - at the previous mount, turning the tool onto
% the base's +x axis zeroed a 0.1784 m lateral carry term and swung the arm out to
% |y| = 0.2464 m, for 0.3536 m of inflation instead of 0.3363 m. Neither bulge can be
% reasoned about from the other's number. The only honest way to choose is to sweep and
% measure BOTH at every angle over the gripper aperture. The arm's footprint was swept that
% way during development, and the carryBound check in hmmSetup does it for the tool.
%
% AT THIS LAYOUT THE FIGHT IS OVER, WHICH IS THE REAL REASON THE PAN NO LONGER HAS TO BE A
% COMPROMISE. The tool sits [+0.1920 +0.1058] from the base origin, so cfg.plan.carryOffset
% is [0.2370 0.1508] rounded up - and 0.1508 is UNDER the 0.1823 m the arm's own upper_arm_link
% already needs, so the held object never sets hmmBaseFootprint's half-width at all. The
% -0.435 rad the previous pose shipped existed only to stop it doing so; the lower mount and
% the 27.5 mm lateral offset achieve the same thing without spending 25 deg of pan on each of
% the four stow-to-target traversals per task.
%
% WHAT IT MEASURES, every figure measured rather than derived by arithmetic: tool at
% [+0.1920 +0.1058] in the base frame, carryBound [0.2370 0.1508], stowed robot half-extent
% [0.2616 0.1823] with |x| set by chassis_link and |y| by upper_arm_link, envelope radius
% 0.3014 m at wheel_fr and therefore inside the 0.3183 m chassis circle, arm z
% 0.4329 .. 1.0840 m, tool vertical to within a rounding, the pedestal foot seated on the
% deck floor to 0.0 mm, 34.5 mm of front bumper margin and 67.7 mm of rear, and a
% five-aperture self-collision sweep gaining zero pairs against the shipped pose and clearing
% itself by the same 0.7 mm the shipped pose clears itself by. hmmBaseFootprint's loaded
% inflation is 0.2899 m - 163.2 mm better than the 0.4531 m that shipped, and 10.1 mm under
% the 0.300 m ceiling for reaching all four rooms.
%
% JOINT 6 POINTS THE WRIST CAMERA ALONG THE BASE'S FORWARD DIRECTION, and it is the second
% element the search did not choose. The search had no term for the camera either. At
% -pi/2 the wrist camera's optical axis leaves the stow at azimuth -0.05 deg in the base
% frame, which is forward; at the +1.135 rad this pose inherited it left at -154.95 deg,
% pointing back over the robot's own deck, because the reflection turned the whole assembly
% and the roll that used to aim forward now aims aft.
%
% THE OPTICAL AXIS IS THE CAMERA'S OWN +x, WHICH IS WORTH STATING BECAUSE IT IS NOT z.
% cfg.camera.wrist.tilt is -1.326241 rad = -76.0 deg, and of the wrist_cam frame's three
% axes it is x that comes out 76.0 deg below horizontal - y is level and z is 14 deg down.
% Measured on the tree, not inferred from the mount expression.
%
% AND BECAUSE THE PARKED TOOL IS VERTICAL, JOINT 6 IS A PURE AZIMUTH FOR THE CAMERA. tcp z
% at this pose is [0.0002 -0.0002 -1.0000], so rolling joint 6 turns the camera's plan
% bearing one-for-one about the world vertical: the measured slope d(azimuth)/d(theta6) is
% -1.0006. That is what makes the alignment exact rather than approximate, and it is also
% why the roll cannot disturb the verticality it depends on - joint 6 turns about the tool
% axis, so the identity theta2 + theta3 + theta4 = -pi/2 that holds the tool down is
% untouched by it.
%
% WHY THAT MATTERS ENOUGH TO OVERRIDE THE SEARCH. The camera is mounted flush into the
% Robotiq base casting, and the casting fills 47.4% of its frame - measured off the wrist
% depth image, where 84% of the sub-100 mm returns lie within 23 mm of the lens. Half the
% frame is therefore the gripper's own shell, and which half the object lands in is decided
% by the wrist's rotation about the tool axis. Swept against that occluder mask, a 101 mm mug
% at the pre-descent standoff shows 25 of 25 rim samples with the camera forward, 24 at
% +30 deg, 23 at -30 deg, and only 9 at the +133.6 deg the map-frame yaw bug used to produce.
% A refinement fitted to 9 of 25 samples is a refinement fitted to a clipped sliver, and it
% returned a grasp point 44 mm from the mug. See MotionPlanning/CandidateIK/Candidate_k for
% the frame fix this pairs with: the stow is the IK seed and the first thing the operator
% sees, and both want the same orientation as the pick.
%
% JOINT 6 IS FREE TO DO THIS AND IT ACTUALLY PAYS FOR ITSELF, WHICH WAS THE SURPRISE.
% tcp_frame is INVARIANT under joint 6 - it sits on that joint's own axis - so the tool
% offset, the carry bound and the tilt this pose was certified on are all untouched and the
% pan question above is not reopened. What moves is the jaw pair, and the roll swings the
% gripper's long axis from diagonal to fore-aft: the stowed arm's own |x| half-extent falls
% from 0.2647 m to 0.2310 m, which takes the front bumper margin from 0.8 mm to 34.5 mm.
% Aligning the camera was expected to COST front room and it buys 33.7 mm of it.
%
% FORWARD RATHER THAN AFT, MEASURED, BECAUSE +pi/2 IS NOT THE MIRROR IMAGE IT LOOKS LIKE.
% The two rolls are identical on inflation (0.2899 m), on self-clearance and on collisions,
% because the footprint is set by the lateral extent and a 180 deg turn about the vertical
% preserves |y|. They differ on |x| and on where the camera looks. Aft leaves the arm
% 1.2 mm OUTSIDE the front bumper, because turning the jaws to the leading side puts the
% fingers there and the jaws reach further from the tool axis than the camera housing does;
% and at 76 deg of down-tilt the optical axis reaches the floor only 141 mm out
% horizontally, which aft means 170 mm BEHIND the bumper - the robot's own deck. Forward
% puts that floor point 23 mm AHEAD of the bumper. The general rule the measurement supports:
% point the wrist camera the way the robot drives and the laser scans.
%
% WHAT THIS IS AND IS NOT A CLAIM ABOUT. Nothing in the shipped pipeline consumes the
% STOW-pose camera bearing - the room cameras do the initial grounding and the wrist camera
% is used at the pregrasp, where the arm is no longer stowed - so the alignment is a geometry
% and future-proofing argument. The occluder argument below is the functional one, and it
% applies at the pregrasp rather than here.
%
% FOLDING THE ELBOW FURTHER BACK TO BUY MORE FRONT ROOM IS NOT TAKEN, AND IT WAS MEASURED
% BEFORE BEING REFUSED. Pulling joint 2 back 0.15 rad buys 16.8 mm at the front and leaves
% only 6.1 mm at the rear; 0.30 rad buys 36.5 mm and hangs 52 mm PAST the rear bumper while
% growing the footprint to 0.2931 m. The roll above already bought the room that refold was
% meant to pay for, so there is nothing left to spend it on.
%
% JOINTS 2, 3 AND 4 ARE THE VERTICAL EXTENSION OF THE SEARCH'S FOLD, NOT THE SEARCH'S OWN
% VALUES. The footprint sweep changed joints 2 and 3 in a pair and let joint 4 absorb the
% remainder, which keeps theta2 + theta3 + theta4 at the -1.571 the search's winner already
% summed to, so the tool stays vertical at every point of that sweep rather than being
% re-certified afterwards. Joint 5 is the search's -pi/2 untouched, and it is what makes the
% verticality identity hold and joint 6 a pure camera azimuth. All six are rounded to
% milliradians and re-measured after rounding. The stow search's cost still lacks a
% footprint term, which belongs in it, and a camera-azimuth term belongs beside it.
%
% Do not re-measure this by differencing logged joint angles. Four of these joints
% span +/-360 deg, so a configuration has several equivalent representations and the
% logged one is whichever turn the planner arrived at; the recorded pre-grasp reads
% 341.9 deg on joint 1 purely because it was reached by increasing from 190.9. Taken
% at face value that single number reverses the conclusion. Every target must be
% wrapped to its nearest equivalent turn from the start being measured from.
cfg.arm.stowConfig         = [0, -1.755, 2.699, -2.515, -1.571, -pi/2];  % rad
cfg.arm.jointVelMax        = [pi pi pi pi pi pi];        % rad/s
cfg.arm.jointAccMax        = 4.0 * ones(1,6);            % rad/s^2
% hmmScreenArmPlan time-parameterises the arm path at this FRACTION of the two
% boxes above, for the same reason cfg.base.slewMargin exists on the base side:
% SafetyMonitor/JointChecks differentiates the REALISED joint velocity with a
% Discrete Derivative and compares with a strict >, so a reference that runs
% exactly to jointAccMax leaves the verdict to whatever the servo overshoots by.
%
% It was not a round-off. With no margin the first free move of a pick reached a
% measured 4.039 rad/s^2 against the 4.0 limit and latched a JOINT_ACCEL E-stop 0.3 s
% into the motion, which aborted the task.
%
% THE OVERSHOOT IS TWELVE PER CENT, NOT ONE, and the first value of this constant was
% sized against the wrong number. 4.039 against 4.0 looked like a one per cent tracking
% error, so 0.90 looked like ten times the headroom needed. It was not: that run had no
% margin at all, so its reference was bounded at 4.0 and 4.039 is what a reference at the
% box overshoots to only because the trip stopped the motion before the overshoot could
% develop. With this constant at 0.90 the reference was bounded at 3.6 and joint 4's
% realised acceleration reached 4.048 - twelve and a half per cent above the reference
% bound, a closed-loop overshoot of the joint servo tracking a spline that sits AT its
% acceleration bound for a stretch, not a spike: a 3-point median moves it only from 4.048
% to 4.030. So the fraction has to cover the servo's overshoot with room left over, and
% 0.75 puts the reference at 3.0 and the realised peak near 3.37, 84 per cent of the box.
%
% THE COST IS DURATION, AND IT IS SMALL. contopptraj's time-optimal parameterisation
% stretches roughly as 1/sqrt(a) on an acceleration-limited stretch, so 3.0 instead of
% 3.6 is about nine per cent more move time - against a JOINT_ACCEL abort, which costs
% the whole task.
cfg.arm.planMargin         = 0.75;    % fraction of the jointVelMax/jointAccMax boxes
% THE POSITION BOX NEEDS THE SAME TREATMENT, AND FOR A SHARPER REASON. The velocity
% and acceleration boxes above are only ever COMPARED against; the position box is
% ENFORCED, by ArmControl/JointLimit, which is a hard Saturate at the UR5e's real
% travel. A saturation that engages is not a small error - it is a corner. The
% position command's slope goes discontinuously to zero, and SafetyMonitor's Discrete
% Derivative of the realised velocity reads that corner as an acceleration step.
%
% So planning right up to the position box cannot work, however legal the plan is.
% Measured, on joint 3, whose band is +-pi and therefore HALF of every other joint's:
% manipulatorRRT put a waypoint against the limit, contopptraj's spline overshot that
% waypoint to about 3.148 rad, the reference staircase at the supervisor rate carried
% the command another 0.012 past it, JointLimit clamped at pi, and the realised
% acceleration jumped from 3.77 to 8.55 rad/s^2 against the 4.0 box - latching a
% JOINT_ACCEL E-stop that had nothing to do with the arm being driven too hard.
%
% This margin is subtracted from both ends of the box for the PLANNING trees only -
% the IK tree the candidate blocks solve on and the subtree manipulatorRRT samples.
% JointLimit keeps the true travel, because the true travel is what the robot has.
% 0.05 rad is 2.9 degrees, under 1 per cent of the narrowest joint's range, and about two
% and a half times the 0.02 rad of spline overshoot plus staircase lead measured above.
cfg.arm.limitMargin        = 0.05;    % rad, kept between a planned pose and the box
% Joint position-loop time constant. Each joint is a first-order servo with a
% saturated velocity, which gives the plant a real joint VELOCITY without
% differentiating a rate-limited signal - a derivative of a signal that has
% discontinuous slope by construction is noise, and the servo lag is
% physically there anyway.
cfg.arm.jointTimeConst     = 0.08;                       % s
% HOW CLOSE COUNTS AS ARRIVED, which is a question about the PLANT and had been sharing a
% number with a question about the SOLVER. ArmControl/TrackingMetrics/SettledCheck (blk_811)
% read cfg.plan.poseErrorMax, the residual hmmScreenArmPlan screens IK candidates against.
% Two unrelated things: one is "is this pose reachable to within solver noise", the other is
% "has the arm stopped where it was sent". Tying them meant the plant declared DONE inside a
% 5 mm ball, and every consumer of DONE acted on it - including GripperSupervisor, which opens
% the jaws the instant the arm reports in. On the verified run the place descent satisfied
% 5 mm while still moving at 27 mm/s and reached its commanded height 0.30 s later, so the mug
% was released 3.2 mm high and, with no physics on the props, stayed there. See
% cfg.plan.releaseClearance for the other 15 mm of that float.
%
% 1 mm IS SIZED ON THE MEASURED TAIL, NOT PICKED. After the reference stops, a first-order
% joint servo closes the remaining error as exp(-t/jointTimeConst), and the logged descent
% settles to 0.1 mm - so 1 mm is an order of magnitude above the floor the plant actually
% reaches and cannot hang a phase. The cost is time: 0.15 m of healthy free-move tracking
% error takes jointTimeConst*log(150) = 0.40 s to fall inside 1 mm, against
% cfg.sup.timeout.approach of 4.0 s over a 1.2 s segment, so every phase keeps better than a
% factor of two. It cannot be made much tighter without paying real settling time on all nine
% phases of the task for a millimetre nothing can see.
cfg.arm.settleBand         = 0.001;   % m, tool error below which a phase has arrived

%% Robotiq AGC-CPL-062-002 wrist coupling
cfg.coupling.thickness     = 0.0113;  % m, flange-to-gripper-base standoff
cfg.coupling.yaw           = 0;       % rad

%% Robotiq 2F-85 gripper
% The commanded gripper signal everywhere in the model is a single scalar
% JAW APERTURE IN METRES - the gap between the pad faces. Linkage
% kinematics are confined to the measured aperture map.
cfg.gripper.robotName      = 'robotiq2F85';
cfg.gripper.nJoints        = 6;
cfg.gripper.driverJoint    = 'finger_joint';
cfg.gripper.driverRange    = [0 0.80];  % rad, published driver limits
cfg.gripper.maxOpenPos     = 0.085;     % m, published stroke
cfg.gripper.minOpenPos     = 0.000;     % m
cfg.gripper.padCompression = 0.003;     % m, pad squeeze allowance at grasp
% THE PAD IS 37.5 mm TALL AND THAT LENGTH IS WHAT CLAMPS THE GRASP HEIGHT.
% Read off left_inner_finger_pad's collision box on armIkTree: 22.0 x 6.35 x 37.5 mm, the
% 37.5 running along the finger. Half of it, 18.75 mm, is how far the gripping surface
% reaches below and above whatever point the aim lands on, and it is the length that must
% fit inside the object - see cfg.grasp.padGroundClear and hmmGraspAim for how the midpoint
% is bounded by it on a short one.
cfg.gripper.padFaceLength  = 0.0375;    % m, along the finger; from the collision box
cfg.gripper.apertureRateMax = 0.060;    % m/s, commanded aperture ramp
cfg.gripper.apertureTol    = 0.0025;    % m, closure-detection band
cfg.gripper.mapBreakpoints = 41;        % samples in the measured aperture map

%% Grasp capture gate - lives in the plant
% The gripper REQUESTS a grasp; the plant DECIDES whether one is possible.
% Both thresholds are properties of the gripper hardware, not of any object.
cfg.grasp.extentMatchTol   = 0.006;    % m, |padSeparation - objectExtent|
cfg.grasp.captureMargin    = 0.020;    % m, added to half-stroke for capture radius
cfg.grasp.captureRadius    = cfg.gripper.maxOpenPos/2 + cfg.grasp.captureMargin;
cfg.grasp.regionHalfSize   = [0.05 0.05 0.05];  % m, Sim3D grasp region half-extent

%% Where on the object the jaws close, and the frame that measurement is about
% THE GRASP HEIGHT IS THE OBJECT'S MID-HEIGHT, CLAMPED BY TWO LENGTHS OF THE GRIPPER.
% hmmGraspAim is the one place that arithmetic lives and its header carries the full
% reasoning; what belongs here is the constant it needs and the history of the field it
% replaced.
%
%     aim = max( min( h/2, h - padFaceLength/2 ), padFaceLength/2 + padGroundClear )
%
% WHAT WAS HERE BEFORE, AND WHY IT HAD TO GO. cfg.grasp.heightFraction was 0.65 of
% Extent(3), chosen because the gripping surface is a 37.5 mm tall pad face on a 90 mm
% tall mug: a pad centred at mid-height reaches to within 26 mm of the tabletop and LOOKS
% like a jaw driven into the table even though it clears it, while 0.65 put the face at
% 0.5906 to 0.6281 m - 40.6 mm clear of the 0.550 m table, 11.9 mm below the rim. Sound
% for a 90 mm object and wrong as a rule, for the reason its own comment recorded: at
% fraction f the headroom above the aim is (1-f)*Extent(3), so 0.65 needs Extent(3) >=
% 18.75/0.35 = 53.6 mm or the pad top closes on air above the object. A fraction scales
% the table clearance WITH the object, so the clearance vanishes exactly where it matters,
% and the comment's own conclusion - "an object under about 54 mm tall wants a smaller f,
% and that is the knob to turn" - is a per-object tuning job that no run can do for itself.
% Expressing the two bounds as the gripper lengths they actually are needs no tuning and
% works for an object nothing in this project has measured.
%
% THE CLEARANCE IS ABSOLUTE BECAUSE THE PAD AND THE TABLE ARE. 10 mm is 3.3 times the
% 3 mm pad compression allowance and about nine render pixels at the wrist camera's
% working distance, so a jaw this far off the surface reads as clear rather than as
% touching. It also sets where the floor starts to bind: below 2*(18.75 + 10) = 57.5 mm of
% object the aim is lifted off mid-height, which is within 4 mm of the 53.6 mm the retired
% fraction named as its own limit - the new rule takes over in the same regime the old one
% admitted it was leaving.
cfg.grasp.padGroundClear   = 0.010;    % m, pad face bottom edge above the support surface

% AND THE AIM POINT IS CORRECTED FOR WHICH FRAME IS BEING AIMED. tcp_frame coincides
% with the pad-face centre only when the jaws are OPEN. Closing them swings the inner
% fingers, and the pad face descends with the swing: measured on armIkTree, 0.00 mm of
% drop at the 92.4 mm open gap, 6.45 mm at the 74.4 mm gap the shipped mug closes to,
% 14.11 mm at a 19.3 mm gap. Commanding tcp_frame to the aim point therefore lands the
% GRIPPING SURFACE that far below it, which is a real error of up to 14 mm and is
% exactly the wrong way - deeper, toward the table.
%
% THE COMPENSATION IS A MEASURED TABLE AND NOT A CONSTANT, because the drop depends on
% how far the jaws close and that depends on the object. The breakpoints are jaw gaps
% and the lookup is driven by ObjectTarget.GraspExtent, the object's width across the
% jaw axis, which is the same quantity the capture gate matches on. Using the object
% width as a proxy for the gap it closes to is accurate to about 1.2 mm on the shipped
% mug - the jaws stop at a 74.4 mm gap on a 70.0 mm object - and 1.2 mm against a 37.5
% mm pad face is not worth a contact model to remove.
%
% MEASURED FROM armIkTree, not published: the z drop of left_inner_finger_pad below
% tcp_frame against the pad separation, over the driver's 0 to 0.70 rad working range.
cfg.grasp.padDropWidth     = [0.0193 0.0363 0.0475 0.0583 0.0687 ...
                              0.0744 0.0787 0.0834 0.0880 0.0924];   % m, jaw gap
cfg.grasp.padDropTable     = [0.01411 0.01331 0.01207 0.01029 0.00797 ...
                              0.00645 0.00514 0.00354 0.00183 0.00000];  % m, pad drop

% AND A MESH'S ORIGIN IS NOT NECESSARILY THE CENTRE OF THE PART THE JAWS HOLD. The
% Sim3D attach pins the prop's ORIGIN to the tool - measured to 0.03 mm, so the
% mechanism is not in doubt - while the grasp aims at the object's visible BODY.
% When those two points differ the prop jumps sideways the instant the gripper
% closes, which is the offset visible on screen at the pick even though the jaws
% closed in the right place. Mug_Red is the clearest case: extent(1) 0.101 spans body
% plus handle while extent(2) 0.070 is the body alone, so the bounding-box centre the
% exporter used as the origin lies about half that difference off the body's axis.
%
% THERE ARE NO pivotOffsetProp / pivotOffsetXY PARAMETERS HERE ANY MORE, and their
% removal is the point of this paragraph rather than an omission. They were a table of
% six prop names against six hand-measured [x y] rows, and a value keyed to a prop name
% is a defect on this project's own standing rule: the manifest holds twenty props, a
% scene can be given one the table never heard of, and the table itself only grew to six
% rows after an unlisted prop - Mug, which is a different mesh file from Mug_Red -
% visibly jumped on a live run.
%
% THE OFFSET IS MEASURED PER RUN INSTEAD, FROM THE ONE LOOK THAT WAS ALREADY HAPPENING.
% hmmSegmentTopFace returns the grasp aim (the top face's plateau centroid, the object's
% axis) and the pivot (the banded footprint's bounding-box midpoint, which is what the
% renderer pins) from a single wrist depth frame, so their difference IS the offset, for
% whatever object is in front of the lens and in whatever yaw it is standing. hmmRefineService
% writes it onto the block during the run. On the non-red Mug - whose row was NOT the one the
% table was calibrated from - it came out [14.95 0.36] mm against the retired [15.50 0.40],
% and the pivot itself landed 1.07 mm from the engine's own reported actor origin. A run-time
% measurement therefore reproduced the hand table to 0.6 mm while needing no object name.
%
% The table had also carried the warning that (extent(1) - extent(2))/2 is not a
% substitute - it predicts 15.50 mm against 14.50 mm measured, but only for a prop whose
% extra span is all on ONE side, and on a centred prop like Magazine it would invent a
% 25 mm offset that does not exist. That warning still stands, and is now moot: nothing
% derives this from an extent, because something measures it.

%% Scene - the reused apartment assets
% The apartment shell and every movable prop are LOADED FROM THE BASELINE
% ASSETS, not modelled here. Nothing the baseline already provides is
% re-derived, and the geometry is the clearest case of it: the shell, the props
% and their placement all ship with the reused example. Only the mesh scale and
% the room datums the camera rig is aimed at live here, because those are
% properties of the RIG rather than of the task.
%
% Nothing in this block names a prop or a destination. The prop list is read at
% run time by hmmSceneManifest as an undifferentiated set; which of them a task
% is about is grounded from the instruction and is unknown until then.
cfg.scene.staticMesh        = 'apartment_static.fbx';  % found by hmmAssetRoots
cfg.scene.meshScale         = 0.01;    % FBX authored in cm, world is in m
cfg.scene.extent            = [0 10; -8 0];  % m, [xlo xhi; ylo yhi] of the shell

% Room datums. A camera sits in a corner of its room and looks at the room's
% centre, which is how the baseline rig is aimed; recording both and deriving
% the yaw means a camera cannot end up pointing somewhere the comment claims it
% does not. Rooms are named because a ROOM is part of the rig, not part of a task.
cfg.scene.room(1).name   = 'LivingRoom'; cfg.scene.room(1).centre = [3.0 -2.0];
cfg.scene.room(2).name   = 'Kitchen';    cfg.scene.room(2).centre = [3.0 -6.5];
cfg.scene.room(3).name   = 'Bedroom';    cfg.scene.room(3).centre = [8.0 -5.5];
cfg.scene.room(4).name   = 'Bathroom';   cfg.scene.room(4).centre = [8.0 -1.5];

% PROPS MOVED BY THE PROJECT, over the shared props.json (applied in hmmSceneManifest). Two tall
% props stood too deep on their shelves for any stand-off the floor allows. Measured 2026-10-12
% against hmmSelectBaseGoal with the support-height fix (hmmSupportHeight) and the wall check
% (hmmWallBetween): the Bottle's chest stands against the bedroom/bathroom wall (y -3.00..-3.10,
% front edge -3.60), so it moves 245 mm SOUTH to the edge limit (half its width plus
% cfg.percep.surfaceInset in from the front) - the 2026-10-07 move went north, toward the wall, and
% was only "reachable" from the bathroom. At -3.45 the base parked correctly in the bedroom but the
% in-model grasp candidates all came out near-singular (rcond 0.00105 against 0.002).
%
% AND EVERY OTHER GRASPABLE PROP THE BASE COULD NOT SERVE, 2026-10-20, after "the cup in the
% bathroom" failed live with BaseGoalOccupied. A census of all 20 props with hmmSelectBaseGoal's own
% pick search - the planner's validator, the lever band at the support, the chart's radius and the
% hmmStandoffRadii ladder - found Glass_Bathroom and Glass_Kitchen with NO free stand-off bearing (each
% ~0.3 m behind the front edge of a 0.91 / 0.90 m top, where the arm's top-down lever is 0.50 m and
% the nearest valid base centre 0.61 m), and SoapDispenser with only 2 of 24. Each slides to the
% nearest spot with at least 4: on its own surface (every heightMap cell under its footprint grown by
% cfg.percep.surfaceInset reads the support height, so not the basin), 40 mm clear of the other props,
% and still in line of sight of its room's camera. Bottle (8 bearings), Mug (12) and Mug_Red (16) stay.
% The other 14 props are wider than the jaw stroke (NotGraspable), so no position would help them.
cfg.scene.propMoves = struct( ...
    'name', {'Bottle', 'SoapDispenser', 'Glass_Bathroom', 'Glass_Kitchen'}, ...
    'xy',   {[8.93 -3.525], [7.95 -2.45], [7.25 -2.45], [2.835 -6.215]});

%% Display window - the Sim3D viewport, NOT a contract camera
% This is the camera a human watches the run through. It is deliberately defined
% here and NOT in cfg.camera: the contract camera list is indexed by the Gemini ER
% protocol (a returned *_image is a 1-based index into cfg.camera.fixed plus the
% wrist), and adding a sixth entry to that list to hold a viewport would corrupt
% every index the provider returns. The viewport observes; it never reports.
%
% WHY THIS EXISTS AT ALL. The Simulation 3D Scene Configuration block shipped with
% its factory default stance of [-6 0 2] metres looking along +X. cfg.scene.extent
% starts at x = 0, so that camera stood six metres beyond the west wall: the window
% rendered the blank outer face of one wall and open sky, with the robot hidden
% behind it. The apartment shell has no ceiling - an overhead render looks straight
% into all four rooms - so a camera lifted above the walls and aimed back down sees
% the whole plan, which is the view this section derives.
%
% The stance is derived from a look-at point, an azimuth, an elevation and how much
% of the frame the building should fill, for the same reason the fixed camera yaws
% are derived from mount and room centre: a viewpoint written as a raw rotation
% triple cannot be checked against the building, and stops being correct the moment
% cfg.scene.extent changes.
%
% IT LOOKS DOWN THE WALL THAT SPLITS THE APARTMENT (2026-10-19, asked for a video). The
% long north-south wall at x = 6.0 m - 81 % of the building's depth is wall there on the
% nav map - has the living room and kitchen on one side and the bathroom and bedroom on
% the other. Standing over its line north of the building, the camera shows it as the
% frame's centre line, one half of the apartment either side: closer than the diagonal
% overview this replaced (range 9.9 m against 13.7 m, from azimuth 140.5 deg) and pitched
% further down (47 deg against 36).
cfg.view.hfov       = 75*pi/180;   % rad, horizontal field of view
cfg.view.splitWallX = 6.0;         % m, x of the north-south wall the camera looks along
cfg.view.azimuth    = pi/2;        % rad, bearing from the look-at point TO the camera: north
cfg.view.elevation  = 47*pi/180;   % rad, above the horizontal
cfg.view.frameFill  = 0.715;       % building width across the view, fraction of the h-field
% 0.6 m south of the building's centre, so the far rooms are whole in the frame.
cfg.view.lookAt     = [cfg.view.splitWallX, mean(cfg.scene.extent(2, :)) - 0.6, 0];  % m

% Range that puts the building's width (across the line of sight) over frameFill of the
% field at the look-at depth. Half the width subtends half the filled angle.
cfg.view.buildingWidth = diff(cfg.scene.extent(1, :));
cfg.view.range = (cfg.view.buildingWidth/2) / ...
                 tan(cfg.view.frameFill * cfg.view.hfov / 2);

cfg.view.translation = cfg.view.lookAt + cfg.view.range * ...
    [cos(cfg.view.elevation)*cos(cfg.view.azimuth), ...
     cos(cfg.view.elevation)*sin(cfg.view.azimuth), ...
     sin(cfg.view.elevation)];

% Looking back down the same ray: yaw reversed, pitch negated.
cfg.view.rotation = [0, -cfg.view.elevation, cfg.view.azimuth - pi];

% The Simulation 3D Scene Configuration block takes its viewport rotation and
% field of view in DEGREES, unlike every other angle in this file (cfg.frames.
% angleUnit is 'rad'). Both forms are published so the block can reference the
% degree fields directly and no conversion has to be written into a mask.
cfg.view.rotationDeg = cfg.view.rotation * 180/pi;
cfg.view.hfovDeg     = cfg.view.hfov * 180/pi;

% A SECOND VIEW THAT FOLLOWS THE ROBOT FROM BEHIND, beside the full one (2026-10-19). Plant/
% ChaseCamera is a Simulation 3D Camera posed every step from chassis_link by the wrist camera's
% own chain (Get Transform -> Coordinate Transformation Conversion -> the Unreal gain), shown in
% its own Video Viewer window. hmmChaseBoom places it.
%
% WALLS SHORTEN AND SWING THE BOOM, as a game's chase camera does. A fixed boom here is often
% inside one: the start pose has the dividing wall 0.8 m behind the robot. So the boom is cast on
% a map of everything taller than blockHeight - walls, wall cabinets, the fridge, from the shell's
% nadir sweep - and stops wallMargin short of the first hit. Counters, the chest and the couch are
% lower, and the camera looks over them. It shortens at once, so the camera never passes into a
% wall, and lengthens at extendGain a step, so it does not spring back as the robot clears a
% doorway. (Three earlier versions: 4.5 m up and 72 deg down to clear the walls - "from the top";
% 1.4 m back at 1.5 m - "only the arm"; rising and steepening when pushed in - "from the top"
% again, since a wall is nearly always close behind.)
%
% OFF BY DEFAULT, because it renders a second view every scene step: about 15-20 % more wall clock.
% It drives Plant/ChaseCamera's enable port - disabled, the camera does not render at all, the
% CameraRig's own trick - and hmmInitializeRun opens the Video Viewer only when it is on. The app's
% "Chase camera view" box sets it at Start; a script sets it before sim().
cfg.view.chase.enabled       = false;
% BEHIND FIRST, AND CLOSE RATHER THAN HIGH OR SIDEWAYS (2026-10-20). Out in the room it is the
% inspect-sim3d-scenes skill's "Robot Third-Person" stance; against a wall it comes in low over the
% robot's shoulder. A version that swung up to 100 deg round a wall sat at the robot's side at the
% start pose - "not really behind" - so the swing is now at most swingOffsets(end), and only when
% it buys more than swingCost per degree. The height follows the boom, heightNear at minBack to
% heightFar at back, and the camera always aims aimAhead in front of the chassis, which keeps the
% pitch near 25-30 deg however short the boom: never from the top.
cfg.view.chase.back          = 2.0;              % m, the longest boom
cfg.view.chase.heightFar     = 1.8;              % m above chassis_link (~0.1 m up), at back
cfg.view.chase.heightNear    = 1.4;              % m above chassis_link, at minBack
% (3.0 m back and 3.0 m up was "too high and too far away"; 1.4 m back at 1.5 m with a fixed
% 22 deg pitch was "only the arm". 2.0 / 1.8 aimed 1 m ahead looks just over the stowed arm.)
cfg.view.chase.aimAhead      = 1.0;              % m in front of chassis_link it aims at
cfg.view.chase.aimHeight     = 0.4;              % m above chassis_link it aims at
cfg.view.chase.swingOffsets  = (0:15:30) * pi/180;    % rad off straight behind, both sides
cfg.view.chase.swingCost     = 0.02;             % m of boom a degree off-axis has to buy
cfg.view.chase.turnGain      = 0.04;             % per step, ~0.5 s to follow a turn at 50 Hz
cfg.view.chase.hfov          = 80*pi/180;        % rad
cfg.view.chase.imageSize     = [540 960];        % [rows cols]
cfg.view.chase.focalLength   = repmat((cfg.view.chase.imageSize(2)/2) / ...
                                      tan(cfg.view.chase.hfov/2), 1, 2);   % px
cfg.view.chase.opticalCenter = fliplr(cfg.view.chase.imageSize) / 2;      % px
cfg.view.chase.blockHeight   = 1.0;              % m, anything taller blocks the boom
cfg.view.chase.wallMargin    = 0.15;             % m short of the first wall
cfg.view.chase.minBack       = 0.6;              % m, the boom never shortens below this
cfg.view.chase.extendGain    = 0.05;             % per step, ~0.4 s to lengthen at 50 Hz

%% Camera rig - contract order is part of the Gemini ER contract
% *_image in a returned action is a 1-based index into THIS list, in this
% order. Index 5 is the wrist camera and is never sent to the provider.
% THE RIG IS A 720 x 1280 CCTV CAMERA SCALED BY ONE FREE NUMBER, and every intrinsic
% below scales with it. Scaling all three together is exactly a change of pixel units:
% hmmRayDirections builds each ray from (u - cx)/fx, which is invariant under a common
% scale, and hmmReconstructTargets derives its ray span as imageSize(2)/rayGrid(2)
% rather than hard-coding the ratio. So the field of view, the ray directions and the
% reconstructed world points are unchanged by pixelScale - only the pixel grid the
% provider's normalised points land on gets finer or coarser.
%
% WHY IT IS WORTH HAVING A KNOB AT ALL. Simulation 3D Camera builds a per-pixel radial
% distortion map in setupImpl and resetImpl, and it builds it whether or not there is
% any distortion to apply - this rig sets RadialDistortion and TangentialDistortion to
% [0 0] and still pays for it. Profiling a 3 s run put 172 s in
% VisionSensor.distortionFunctionMap across 18,432,000 calls to distortionFunction and
% as many to undistortionPoint: 20 x 720 x 1280, five cameras over four compile and
% reset phases. That cost is quadratic in pixelScale and is paid BEFORE the first
% simulation step, which is why no sample-time lever ever moved the run time.
%
% LEGAL VALUES ARE CONSTRAINED, not free: pixelScale must leave imageSize an integer
% multiple of cfg.depth.rayGrid, or the pixel-to-ray registration falls between rays.
% With rayGrid [72 128] that permits 1, 1/2, 1/5 and 1/10 - the condition is that 10 *
% pixelScale be a whole number, because 720/72 and 1280/128 are both 10. The assertion
% after cfg.depth.rayGrid enforces it rather than leaving a misregistration to be found
% as a reconstruction error months later. NOTE THAT RAISING rayGrid NARROWED THIS SET:
% at the former [36 64] the quotients were 20, so 1/4 and 1/20 were legal too and are
% not any more. The shipped 1/2 is unaffected.
%
% AND OF THOSE, TWO ARE SUPPORTED, because two are all this project has reasoned about:
%   1    -> [720 1280]. The baseline rig, and what the shipped replay corpus was captured
%           against. The mug subtends 12.2 px at the reference run's actual grounding
%           geometry - LivingRoomCam, 3.798 m range, fx 460 px.
%   1/2  -> [360 640]. Four times fewer pixels, so the distortion-map term that
%           was measured at 83.9 s on the full frame falls to roughly 21 s of a
%           ~680 s run. The GEOMETRY is unaffected: a returned *_point is normalised
%           0..1000, admitFrame maps it to a FRACTIONAL image position, ray directions
%           come from (u - cx)/fx, and the ray grid that actually turns 2D into 3D stays
%           [72 128]. The mug is sub-ray-pitch either way - 1.22 ray pitches at both
%           scales - which is why cfg.percep.extentAssumed and minClusterHits exist.
%           WHAT IS NOT INVARIANT IS WHAT THE PROVIDER SEES: the mug becomes 6.1 px
%           across, and whether a LIVE provider still localises a 6 px object has NOT
%           been measured. Under the default replay provider it cannot matter at all,
%           because a fixture stores the normalised point and not the image.
% Anything else is refused below rather than half-supported.
%
% THE DEFAULT IS 1/2, AND THE REASON IS THE WRIST CAMERA. A second grounding call from the
% end-effector camera refines the pick point before the descent, and at the pre-descent pose
% that camera is 0.1859 m from the grasp point - NOT the 0.119 m this note used to claim,
% which added approachDepth 0.080 m to the offset's own norm as if the two were collinear.
% The distance is measured off the assembled tree: the camera origin is at
% [0.045 0 -0.1003] in tcp_frame coordinates, so the grasp point at [0 0 0.080] is
% 0.1859 m away. The mug spans ~174 px of a 640-wide frame at that range rather than 263.
% So PRECISION on the wide call stopped being load-bearing: the wrist camera's
% 91.8 deg hfov sees a 0.384 m patch at that range, so the wide call only has to land the
% mug inside a third of a metre rather than hit it. What the wide call must still do is
% DETECT a 6.1 px mug at 3.798 m, and that is not something this file can assert - it is
% measured, per live run, by comparing the wide answer against the wrist answer. Under the
% default replay provider it does not arise at all.
cfg.camera.pixelScale      = 1/2;
assert(ismember(cfg.camera.pixelScale, [1, 1/2]), ...
    'hmmConfig:unsupportedPixelScale', ...
    ['cfg.camera.pixelScale = %g is not one of the two supported rigs. Use 1 for ' ...
     '[720 1280] or 0.5 for [360 640]; see the note above for what each costs and ' ...
     'what it leaves unmeasured.'], cfg.camera.pixelScale);
cfg.camera.imageSize       = [720 1280] * cfg.camera.pixelScale;   % [rows cols]
cfg.camera.focalLength     = [460 460]  * cfg.camera.pixelScale;   % [fx fy] px
cfg.camera.opticalCenter   = [640 360]  * cfg.camera.pixelScale;   % [cx cy] px
assert(all(mod([cfg.camera.imageSize, cfg.camera.focalLength, ...
               cfg.camera.opticalCenter], 1) == 0), ...
    'hmmConfig:fractionalPixels', ...
    ['cfg.camera.pixelScale = %g gives fractional pixel intrinsics. A camera dialog ' ...
     'takes integer pixels, so choose a scale that divides 720, 1280, 460, 640 and ' ...
     '360 exactly.'], cfg.camera.pixelScale);
% NOT SCALED. This is the provider's normalisation, not a property of the sensor: a
% returned *_point is 0..1000 whatever the image happens to be, which is the whole
% reason the contract normalises. Scaling it here would silently rescale every fixture
% in the replay corpus.
cfg.camera.pixelNormScale  = 1000;         % baseline normalisation for *_point
cfg.camera.mountHeight     = 2.30;         % m, fixed camera height
cfg.camera.tilt            = -40*pi/180;   % rad, fixed camera pitch

% Fixed cameras, contract indices 1..4, one per room, in the SAME corner-mounted
% arrangement as the baseline cctv rig: 2.3 m up, tilted 40 deg down, yawed to
% face the room centre. Poses are rig properties, so they belong here; the yaw
% is DERIVED from the mount and the room centre rather than written down, which
% is what keeps a camera aimed at the room it is named after after either datum
% moves.
cfg.camera.mount = [0.5 -2.5;    % LivingRoom
                    4.0 -5.6;    % Kitchen
                    6.5 -4.5;    % Bedroom
                    6.5 -2.5];   % Bathroom
for iCam = 1:size(cfg.camera.mount, 1)
    look = cfg.scene.room(iCam).centre - cfg.camera.mount(iCam, :);
    cfg.camera.fixed(iCam).name        = [cfg.scene.room(iCam).name 'Cam'];
    cfg.camera.fixed(iCam).translation = ...
        [cfg.camera.mount(iCam, :) cfg.camera.mountHeight];
    cfg.camera.fixed(iCam).rotation    = ...
        [0 cfg.camera.tilt atan2(look(2), look(1))];
end
cfg.camera.nFixed               = numel(cfg.camera.fixed);

% Wrist camera - contract index 5, pose driven from forward kinematics.
% Image size is deliberately IDENTICAL to the fixed cameras: all five
% streams travel on one CameraFrame bus type, and a bus element cannot hold
% two different dimensions. The wrist camera differs in its lens (focal
% length), which is a bus element and so may vary per camera.
cfg.camera.wrist.name           = 'WristCam';
cfg.camera.wrist.index          = 5;
cfg.camera.wrist.focalLength    = [620 620] * cfg.camera.pixelScale;
cfg.camera.wrist.imageSize      = cfg.camera.imageSize;
cfg.camera.wrist.opticalCenter  = cfg.camera.opticalCenter;
cfg.camera.wrist.offset         = [-0.045 0 0.030];  % m, on the coupling, FRONT face
%
% THE SIGN OF offset(1) IS WHICH FACE OF THE COUPLING THE CAMERA IS BOLTED TO, and it is
% negative so the housing sits in FRONT of the jaws rather than behind them. It was +0.045
% and the camera rendered visibly at the back of the wrist, which is not where anyone
% mounting a wrist camera would put one.
%
% MIRRORING IS LEGITIMATE HERE, AND THAT IS A MEASUREMENT RATHER THAN AN OPINION. The tool
% frame's x = 0 plane is a true symmetry plane of the gripper: all ten Robotiq jaw bodies sit
% at tool-frame x of +-0.0 mm, with y offsets in symmetric pairs (+-12.7, +-30.6, +-46.2,
% +-62.1, +-68.2 mm). So the two mounts are optically equivalent - same stand-off, same
% obliquity, same jaw occlusion - and nothing downstream of the camera had to move. The tilt
% below is mirrored with it, which is the whole of the change.
%
% WHAT IS NOT CLAIMED: that the front mount sees the object BETTER. It does not; it sees it
% identically, because the aim point and the stand-off are unchanged by reflection. The
% earlier rim-sweep argument for the rear mount was about a DIFFERENT mount (front position
% with the REAR tilt, which aims 28 deg off the grasp point and scores 2-3 of 25). Mirroring
% both together is not that mount, and the aim assert below is what keeps the two in step.
%
% THE TILT AIMS THE AXIS AT THE GRASP POINT, NOT ALONG THE DESCENT, AND THE DIFFERENCE IS
% 14 deg OF SOMETHING THAT MATTERS. robots/assembleMobileManipulator mounts wrist_cam on
% gripper_mount with eul2tform([0 tilt 0], 'ZYX'), which is Ry(tilt), and the sensor's
% optical axis is +X (measured in hmmBackprojectDepth). The descent runs along the TOOL
% frame's +z: hmmScreenArmPlan:330 builds the pre-descent pose as
% grasp * trvec2tform([0 0 -cfg.sup.approachDepth]). The obvious value is therefore
% tilt = -pi/2, since Ry(-pi/2)*[1;0;0] = [0;0;1] puts the axis exactly along the descent -
% and that is what this constant was, and it is NOT the right answer, because the camera is
% not on the tool axis. It sits at [0.045 0 -0.1003] in tcp_frame, so an axis PARALLEL to
% the descent looks 45 mm to the SIDE of the object the arm is about to grasp. Measured:
% the grasp point landed at row 102.6 of 360 instead of 180, and in the first run that
% actually rendered, the red mug occupied rows 1 to 92 - CLIPPED at the top edge, with its
% centre off the picture. A refinement is only as good as the provider's view of the object,
% and half the object was outside the frame.
%
% SO THE AXIS IS AIMED AT THE GRASP POINT INSTEAD. The camera origin is at [0.045 0 -0.1003]
% in tcp_frame and the grasp point at the moment the picture is taken is [0 0 0.080], so the
% line of sight is [-0.045 0 0.1803] (norm 0.1859 m) and the tilt that puts +X along it is
% atan2(-0.1803, -0.045) = -1.8154 rad, 14.01 deg past -pi/2. Measurement confirms
% the grasp point then projects to row 180.00 col 320.00 - the exact centre of the frame, with
% the full 180-row margin in every direction that a clipped object did not have. The cost is
% that the optical axis is 14.01 deg off the descent, which costs NOTHING: hmmBackprojectDepth
% composes the camera's own reported rotation, so it inverts any orientation correctly, and
% the depth it reads is planar along whatever axis the camera has.
%
% THIS IS A DERIVED NUMBER HELD AS A LITERAL, which is the one thing this project has been
% bitten by repeatedly, so it was guarded twice. It cannot be written as an expression here
% because the 0.1303 m gripper length that places tcp_frame relative to gripper_mount is a
% dimension of the imported Robotiq model inside assembleMobileManipulator, not a constant in
% this file - deriving it here would mean copying 0.1303 in, which is the same trap one level
% down. Instead, during development the aiming tilt was printed next to the current one, and
% the replay corpus build asserted the projection lands in the central half of the frame and
% named this constant when it did not. Change cfg.sup.approachDepth or .offset and re-derive
% this tilt, or the mug drifts quietly back towards the edge.
%
% WHAT THE ORIGINAL -0.35 DID, for the record, measured the same way: the optical axis
% came out [0.9394 0 0.3429] in the tool frame, 69.95 deg off the descent - the camera looked
% ACROSS the room. The grasp point projected to row -2749 of a 360-row frame, 2749 rows above
% the top edge, and every refinement in a full run was rejected STATUS_RANGE on a depth of
% 4.1922 m, which is the far wall. The feature was plumbed correctly and pointed at a wall.
%
% AND IT IS NOW THE MIRROR OF THAT NUMBER, because offset(1) moved to the front face. With the
% camera at [-0.045 0 -0.1003] the line of sight to the same grasp point is [+0.045 0 0.1803],
% so the aiming tilt is atan2(-0.1803, +0.045) instead of atan2(-0.1803, -0.045) - which is
% exactly -pi - (-1.815352) = -1.326241, still 14.01 deg off the descent, on the other side.
% Written as the mirror of the old literal rather than re-derived, so it carries the same
% precision the old one did; an independent derivation from 0.030 - 0.1303 and approachDepth
% gives -1.326209, agreeing to 3e-5 rad (0.002 deg). Measurement remains the authority and
% still reports the grasp point at the exact centre of the frame, because reflecting the mount
% and the aim together leaves the stand-off and the obliquity untouched.
cfg.camera.wrist.tilt           = -1.326241;         % rad, axis through the grasp point

cfg.camera.nTotal               = cfg.camera.nFixed + 1;

% THE RIG IS GATHERED ONCE PER TASK, NOT EVERY TENTH OF A SECOND, and this is how long
% "once" lasts. Plant/CameraRig is an ENABLED subsystem whose enable is
% TaskResult.PlannerMode == cfg.sup.pl.GROUND, so the five cameras and the four ray
% tracers run only while the planner is grounding an instruction. Measured over the
% 40 s reference run, the rig used to fire 400 times and be read ONCE: perception is a
% triggered subsystem, it fires on the rising edge of the planner's PerceptionSend, and
% one grounding call yields the whole action queue. 3,686,400 ray casts were made and
% 9,216 were read.
%
% WHY THIS IS A GATE ON THE REQUEST AND NOT A WINDOW MEASURED FROM t = 0. The operator
% app starts a run with nothing submitted - it clears the submission deliberately, so
% the robot waits at its home pose - and the instruction is typed while the apartment is
% already on screen. Grounding therefore happens at an ARBITRARY time, not at t = 0. A
% window keyed to the start of the run would answer every interactive task from a frame
% tens of seconds old, and hmmGroundingService would refuse it as Stale. Keying the gate
% to the grounding phase costs one comparison and works for the first task and the fifth.
%
% WHY IT IS A WINDOW AND NOT AN INSTANT. The capture and the trigger are on different
% clocks. The planner enters Grounding on a supervisor tick (0.02 s) and the rig captures
% on a perception tick (0.1 s), so opening the gate and firing the trigger on the same
% edge hands Perception the HELD output of a subsystem that has not run yet - the bus's
% ground value, Valid = false - or a frame older than the request. So PerceptionSend is
% held back by this window: homeMobileManipulator/Perception/PerceptionSendDelay is a Delay
% of captureDelaySteps rather than a UnitDelay of one, which leaves the gate open across
% whole perception ticks before SceneGrounding ever looks.
%
% WHY THREE TICKS. The gate opens wherever the request lands, unaligned with the
% perception grid, so one tick guarantees only that a capture happens and gives the
% trigger no clearance behind it; three guarantee at least two captures and leave the
% trigger a supervisor step after the last of them. The frame Perception reads is then at
% most one perception period old against a cfg.safety.staleTimeout of 1.5 s - a 15x
% margin - and the cost is that every task starts 0.30 s later than it used to.
%
% WHAT THIS DOES NOT BUY. The per-pixel distortion map above is built at reset, five
% times over, whether the rig ever captures a frame or not. Gating the rig removes the
% per-frame cost (~5 % of a 40 s run) and not one second of the fixed cost; only
% pixelScale touches that. Nor does it change the viewport: the Simulation 3D Engine and
% the main camera run on cfg.rates.scene and are not gated.
cfg.camera.captureWindow        = 3 * cfg.rates.perception;   % s, gate open per task
% LIVE VIEW, FOR THE OPERATOR ONLY. Off by default because the rig costs render time whenever
% it is enabled. When on (HomeMobileManipulatorApp's "Live camera views"), the rig is also
% opened for one camera sample every livePeriod - see CaptureSchedule/LiveViewPulse and LiveViewTap -
% and the four fixed frames are handed to the app. Perception never reads these extra frames:
% grounding still captures in its own window on the planner's Send. Applies at the next start.
cfg.camera.liveView             = false;
cfg.camera.livePeriod           = 1.0;    % s of model time between live frames
% THE RIG ALSO OPENS ONCE EARLY IN EVERY RUN (CaptureSchedule/StartupCapture, an Interval Test), so the
% console's camera panes have a picture before anyone has spoken to the provider. Without it the
% panes stayed on "camera k" until grounding, because the rig only runs while grounding.
% NOT AT t = 0: the first rendered frame comes out dark (measured 2026-10-13: mean 37-89 against
% 73-119 from 0.5 s on, flat to 10 s). LiveViewTap passes on frames only WHILE this window is open
% (or live view is on), so a t = 0 capture can never stand in for the start-up picture.
cfg.camera.startupCapture        = [1.45 1.75];   % s, [from to] of model time

% THE STEP COUNT IS PUBLISHED, AND ROUNDED HERE RATHER THAN IN THE BLOCK. A Delay
% block's DelayLength must be a real nonnegative scalar INTEGER, and 3*0.1/0.02 is
% 15.000000000000002 in binary floating point, which that check rejects. Rounding is
% done once, next to the assertions that make it safe to round, so PerceptionSendDelay
% can name a field instead of carrying arithmetic in a dialog.
cfg.camera.captureTicks         = round(cfg.camera.captureWindow / cfg.rates.perception);
cfg.camera.captureDelaySteps    = round(cfg.camera.captureWindow / cfg.rates.supervisor);
assert(cfg.camera.captureTicks >= 2 && ...
       abs(cfg.camera.captureTicks * cfg.rates.perception - ...
           cfg.camera.captureWindow) < 1e-9, ...
    'hmmConfig:captureWindow', ...
    ['cfg.camera.captureWindow = %g must be a whole number of perception periods ' ...
     '(%g s) and at least two of them, or the gate can open and the trigger fire ' ...
     'with no capture in between.'], cfg.camera.captureWindow, cfg.rates.perception);
assert(abs(cfg.camera.captureDelaySteps * cfg.rates.supervisor - ...
           cfg.camera.captureWindow) < 1e-9, ...
    'hmmConfig:captureDelaySteps', ...
    ['cfg.camera.captureWindow = %g is not a whole number of supervisor steps (%g s), ' ...
     'so PerceptionSendDelay cannot express it as an integer DelayLength.'], ...
    cfg.camera.captureWindow, cfg.rates.supervisor);

%% Depth
% THIS SECTION CARRIED A FALSE CLAIM FOR THE WHOLE OF THE PROJECT'S LIFE, and correcting it
% is what makes a wrist grounding refinement possible. It read: "render depth arrives as a
% double port but its VALUES are quantised to whole metres, so 2.334 m reads as 2 ... useless
% for a 9 cm object." That was inherited documentation, never measured, and it is wrong by
% four orders of magnitude. Logging the Plant CameraFrames bus and taking the difference
% between adjacent sorted DISTINCT values over all 230 400 pixels of all five cameras
% gives the same answer on every camera:
%
%     quantum = 1000 / 2^24 = 5.9605e-05 m  (59.6 um)
%
% which is exactly a 24-bit fixed-point encoding of a 1000 m far plane - i.e. the "1 m" of
% the old comment was the FAR PLANE in metres mistaken for the step size. A 9 cm mug spans
% 1510 quanta, and the 0.10 m planner margin spans 1678. Render depth is not coarse; it is
% finer than anything this project measures.
%
% FOUR THINGS FOLLOW, all measured, none documented by the block:
%   - Depth is PLANAR, the distance along the optical axis, not range along the pixel ray.
%     Both were compared against the ray tracer; planar wins outright.
%   - Depth >= 999.999 is the "ray hit nothing" SENTINEL, not a 1 km reading. It is 0 % of
%     fixed-camera pixels (a room camera always sees a wall) and 21.4 % of wrist pixels,
%     which is why any wrist consumer MUST reject it before averaging.
%   - The sensor frame is +X along the optical axis, +Y image RIGHT, +Z image UP. A roll-free
%     camera cannot see this sign - a left-right mirror preserves height - so it is the wrist's
%     172 deg roll that settles it.
%   - world = Rz(yaw)*Ry(-pitch)*Rx(-roll) * p_sensor + Translation. Three independent
%     tests agree: closure against the ray tracer to 10 mm, each fixed camera's optical
%     axis aiming at its own room centre to 0.00 deg, and 100.00 % of 411 423 reconstructed
%     points landing inside the 10 x 8 m shell with none through a wall.
% functions/hmmBackprojectDepth implements exactly those four and is the only place they
% are written as code; see its header for the falsifiable test behind each.
%
% THE RAY-LABEL MIRROR THAT CLOSURE TEST EXPOSED IS NOW FIXED. hmmRayDirections had
% "+Y is image LEFT" and built [1, -xn, -yn]; the point projection mirrored to match, so
% every closure test between the two passed while the PAIR disagreed with the renderer.
% Both carry +xn now. The corpus was re-baked in the same pass, because the fixtures
% had been authored through the mirrored projection and the two mirrors were what kept
% the reconstruction within 49.1 mm of the mug instead of 2.4 m off it.
%
% WHAT DOES NOT CHANGE is the ray-trace path. Grasp reconstruction from the FIXED rig still
% uses SOURCE_RAYTRACE, because that path is what the shipped replay corpus, the nav map and
% every earlier measurement were built against, and because rewriting a working
% reconstruction on the strength of a new measurement is a separate decision from adding a
% refinement that needs no such rewrite. Every CameraFrame still carries DepthSource so no
% consumer can confuse the two.
cfg.depth.SOURCE_RENDER     = uint8(1);
cfg.depth.SOURCE_RAYTRACE   = uint8(2);
% ONE RAY PER 5 PX, RAISED FROM ONE PER 10 PX, AND THE REASON IS A PLACE THAT COULD NOT BE
% CERTIFIED RATHER THAN A WISH FOR RESOLUTION. [36 64] over a [360 640] image samples one
% pixel in a hundred, and the place window is sized in METRES - fitSupport widens it until it
% spans regionHalfMax + surfaceInset - so the number of rays inside it falls as range
% squared. Measured at the bedroom shelf, 3.43 m from BedroomCam: the window held 41 rays, 16
% of them within the plane slab, and pcfitplane found 6 inliers. No threshold can certify a
% surface from 6 points, and the run duly failed PlaceUnsupported with NumInliers 6 (SDI
% 550470). The same window at one ray per 5 px holds 163 rays, 65 in the slab, and the fit
% returns 18 inliers on a level normal - the place is certified and the transfer proceeds.
%
% AND IT IS FREE, WHICH IS THE PART THAT DECIDED IT. Quadrupling the rays changed the
% measured scan time from 81.7 s to 83.2 s, because that figure is dominated by loading the
% apartment and spawning twenty props, not by tracing rays. Nothing caches a per-ray array
% either: hmmRayDirections, hmmInvalidRayHits and hmmReconstructTargets all size themselves
% from this value, the CameraFrame bus dimensions come from cfg.depth.rayRows so they
% recompile, the nav map is an occupancy grid that denser rays only improve, and the replay
% corpus stores PIXELS rather than rays. The ray outports are not logged, so SDI does not
% grow.
%
% THE ADMISSIBLE VALUES ARE FIXED BY THE ASSERTION BELOW, not by taste: imageSize ./ rayGrid
% must be the same whole number in both axes, so the span must divide both 360 and 640 - 1,
% 2, 4, 5, 8, 10, 20 or 40. This is span 5.
cfg.depth.rayGrid           = [72 128];   % [rows cols] ray grid per camera, one ray per 5 px
cfg.depth.nRays             = prod(cfg.depth.rayGrid);
cfg.depth.rayLength         = 12.0;       % m, beyond this a ray is invalid
% Zero bounces: a depth measurement is the FIRST intersection along a ray.
% The Ray Tracer emits nRays*(bounces+1) rows, and any extra rows are
% reflections that would corrupt a depth reading if mistaken for range.
cfg.depth.rayBounces        = 0;
cfg.depth.rayRows           = cfg.depth.nRays * (cfg.depth.rayBounces + 1);
% MEASURED, not documented - see the note above. 1000 m far plane over 24 bits.
cfg.depth.renderFarPlane    = 1000;               % m, the encoded far plane
cfg.depth.renderQuantum     = 1000 / 2^24;        % m, 5.9605e-05, measured
% The sentinel test. Sits just below the far plane so that quantisation at the top of the
% range cannot make a genuine long reading look like a miss, and so that a miss - which
% arrives as exactly the far plane - always trips it.
cfg.depth.renderSentinel    = 999.999;            % m, >= this means "hit nothing"
cfg.depth.occupancyBand     = [0.05 1.20];  % m, height band for the map

% THE RAY GRID IS THE IMAGE GRID DECIMATED BY AN INTEGER, and this is where that is
% enforced, because cfg.camera.pixelScale can break it from 80 lines further up.
% hmmReconstructTargets derives raySpan = imageSize(2)/rayGrid(2) and uses it to decide
% which rays belong to a grounded pixel's neighbourhood; a fractional span puts that
% neighbourhood between rays, and the symptom is not an error but a reconstruction that
% is quietly a few centimetres out - which is indistinguishable from a calibration
% problem and much harder to find than this assertion.
raySpan = double(cfg.camera.imageSize) ./ double(cfg.depth.rayGrid);
assert(all(mod(raySpan, 1) == 0) && raySpan(1) == raySpan(2), ...
    'hmmConfig:rayGridMismatch', ...
    ['cfg.camera.imageSize %s is not a common integer multiple of cfg.depth.rayGrid ' ...
     '%s (span %s). Choose a cfg.camera.pixelScale that keeps both axes integral and ' ...
     'equal - with rayGrid [72 128] that is 1, 1/2, 1/5 or 1/10.'], ...
    mat2str(cfg.camera.imageSize), mat2str(cfg.depth.rayGrid), mat2str(raySpan));

%% Wrist grasp refinement (the second grounding call)
% WHAT THIS IS. The wide call looks at four ceiling cameras from across the room and lands a
% pick point good to centimetres; the arm drives to a pre-descent pose 0.080 m above it; and
% then - before the descent commits - the wrist camera takes one frame and the provider is
% asked ONE much easier question: where in THIS picture is the object. The answer is a pixel,
% the aligned Depth image supplies the metre reading at that pixel, and hmmBackprojectDepth
% turns the pair into a world point that replaces ObjectTarget.Position for the descent.
%
% WHY IT IS WORTH A SECOND CALL: the geometry, not the model. At the pre-descent pose the
% wrist camera is 0.186 m from the grasp point and one pixel subtends 0.58 mm. At the reference
% run's actual grounding geometry - LivingRoomCam, 3.798 m - one pixel subtends 16.5 mm. The
% same provider, the same schema and the same arithmetic are therefore ~28x more precise on
% the second call purely because the camera is closer, and the mug that spans 6.1 px in the
% wide frame spans ~174 px in the wrist frame.
%
% THE RANGE AND THE TWO FIGURES DERIVED FROM IT WERE WRONG UNTIL THE MOUNT WAS MEASURED. This
% note read 0.119 m, 0.38 mm, 43x and 263 px, all composed from adding cfg.sup.approachDepth to
% the norm of cfg.camera.wrist.offset as though the two were collinear. They are not:
% the camera origin measured off the assembled tree sits at [0.045 0 -0.1003] in
% tcp_frame coordinates, so the grasp point at [0 0 0.080] is 0.1859 m away. The correction
% costs nothing operationally - the wrist call is still an order of magnitude sharper than the
% wide one - but it is the same class of mistake as the tilt, and both were found the same way.
%
% IT IS ADVISORY, AND THAT IS THE WHOLE SAFETY ARGUMENT. Every rejection path - no key, bad
% JSON, a point off the object, a sentinel patch, a displacement larger than maxShift -
% leaves ObjectTarget exactly as the wide call reconstructed it and lets the descent proceed
% as it does today. The refinement can only ever make the pick better or leave it unchanged;
% it cannot introduce a failure the current model does not already have. That is why these
% are Status codes and not cfg.fail codes: nothing here aborts a task, and no chart
% transitions on them.
%
% AND THAT PROMISE HAD A HOLE IN IT, NOW CLOSED IN TWO PLACES. "Leaves ObjectTarget exactly as
% the wide call reconstructed it" is true of this function, but the chart still re-issues
% A_PLAN_PICK afterwards, and a re-plan to an UNCHANGED target asks the arm planner for a path
% from the standoff to the standoff. hmmScreenArmPlan used to search from cfg.arm.stowConfig
% regardless of where the arm was, which made that re-plan replay a stow-to-standoff reference
% against an arm already at the standoff and trip the tracking guard; and even with the start
% corrected, a zero-length problem returns a single state that the "a path needs two waypoints"
% guard rejects. Both are fixed - see the qStart comment in hmmScreenArmPlan and
% cfg.plan.armReplanTol - so a rejected refinement now costs one supervisor tick.
cfg.refine.STATUS_APPLIED    = uint8(0);   % refined point accepted and written
cfg.refine.STATUS_DISABLED   = uint8(1);   % cfg.refine.enabled is false
cfg.refine.STATUS_NOFRAME    = uint8(2);   % wrist frame invalid or stale
cfg.refine.STATUS_PROVIDER   = uint8(3);   % no key, transport, timeout or malformed JSON
cfg.refine.STATUS_LOWCONF    = uint8(4);   % below cfg.provider.confidenceMin
cfg.refine.STATUS_NODEPTH    = uint8(5);   % every pixel in the patch is the miss sentinel
cfg.refine.STATUS_RANGE      = uint8(6);   % depth outside [minRange maxRange]
cfg.refine.STATUS_TOOFAR     = uint8(7);   % displacement exceeds maxShift
cfg.refine.STATUS_NOOBJECT   = uint8(8);   % ranged to a surface that is not the object's top
cfg.refine.STATUS_DEPTHONLY  = uint8(9);   % provider unusable; top face measured from a coarse seed
cfg.refine.enabled           = true;
% THE DEPTH-ONLY FALLBACK, AND WHY THE COARSE POINT IS NO LONGER A SAFE DEFAULT. The coarse
% Position is the ceiling ray's hit on the object's camera-facing WALL: ~56 mm high and 40-50 mm
% out laterally. Since the attach offset is frozen at jaw closure (hmmAttachOffset), a grasp on
% that point is carried all the way to the place - measured on mapVerifyRun9/10 (both HTTP 503):
% frozen offset z 64.1 mm against 7.7 mm on an APPLIED run, i.e. the mug released ~53 mm INTO the
% counter. The wrist depth frame is still in hand when the provider fails, and hmmSegmentTopFace
% needs only a seed pixel inside its search window, so the coarse point's own projection is used
% as that seed. Every gate the APPLIED path runs (range, rise band, maxShift) still runs.
cfg.refine.depthOnlyFallback = true;
% A measured height replaces a KNOWN one only within this fraction of it (hmmRefineService); a
% default extentAssumed guess is always replaced.
cfg.refine.heightAgreeTol = 0.25;

% A PATCH MEDIAN, NEVER ONE PIXEL. Against the ray tracer, single-pixel depth was seen
% jumping METRES at a silhouette, because a pixel on the boundary of the mug reports either
% the mug or the wall behind it and nothing in between. Measured on the wrist patch itself,
% radius 0, 2 and 5 px all return median depth 4.0962 m and the same world point to 1 mm,
% while the spread inside the patch grows 0.0000 -> 0.0185 -> 0.0461 m. So the median is
% stable over the choice and the radius only buys robustness. 2 px - a 5x5 window - is the
% smallest that can outvote a single stray pixel.
cfg.refine.patchRadius       = 2;          % px, half-width of the median window
% THE RANGE WINDOW HAS TO COVER THE WHOLE STANDOFF LADDER, not one standoff. The pick search
% may settle anywhere on cfg.sup.approachDepthLadder, and the camera sits 0.1003 m behind the
% tool along its own axis, so the measured range to the grasp point runs 0.186 m at the
% bottom rung to 0.341 m at the top. Anything nearer than 0.02 m is inside the jaws or a lens
% artefact; anything past 0.60 m is not the object the arm is poised over and is most likely
% the floor or a wall seen past it. 0.60 leaves 0.26 m of headroom over the tallest rung,
% which is what stops a taller ceiling from silently turning every refinement into
% STATUS_RANGE - the exact way the first mount defect hid.
cfg.refine.minRange          = 0.02;       % m
cfg.refine.maxRange          = 0.60;       % m
% THE DISPLACEMENT CAP IS THE LOAD-BEARING GUARD. The refinement's job is to move the grasp
% point by millimetres-to-centimetres; a refinement that wants to move it by a quarter of a
% metre has not refined anything, it has found a different object, and applying it would
% send the descent somewhere the arm plan was never screened for. So the cap is the standoff
% the arm is holding above the object - specifically cfg.sup.approachDepthPickMax, the tallest
% rung the pick search may choose, asserted against it where cfg.sup is defined. It is written
% as a literal rather than a reference because cfg.sup is 1700 lines below this point.
%
% IT WAS 0.08 AND THAT WAS SMALLER THAN THE ERROR IT EXISTS TO CORRECT. The coarse path's own
% uncertainty is set by the ceiling ray grid, which samples every ~165 mm at 3.798 m against a
% 101 mm mug, and a measured coarse estimate was 102 mm out in 3-D. A cap of 80 mm would have
% rejected the correction as STATUS_TOOFAR - advisory, so the run would have looked fine and
% the second look would simply never have applied. A guard against finding the wrong object
% must be looser than the error the instrument is there to fix.
cfg.refine.maxShift          = 0.24;       % m, max distance from the coarse point
% THE RISE BAND IS WHAT LETS THE REFINEMENT WRITE A DEPTH AT ALL. ObjectTarget.Position is the
% SUPPORT SURFACE beneath the object's centre - that is what the ceiling rig measures, its cluster
% being mostly table - while this camera can only range to the object's TOP FACE. So the measured
% point is carried down by Extent(3) before it is written, and that subtraction is valid only if
% the ray reached the object. A pixel that lands on the table beside it would be carried down into
% the table, putting the grasp a full object height low: exactly the defect the whole
% support-surface reading exists to remove, reintroduced by the instrument meant to fix it.
%
% THE DISCRIMINATOR IS ALREADY IN HAND. A top face stands about one Extent(3) above the coarse
% surface; a tabletop stands about zero above it. Requiring the rise to land inside this band
% separates the two with a factor of two either side, and no flat surface can enter it.
%
% A FRACTION, NOT METRES, so it scales with the object. An absolute window wide enough for a
% 90 mm mug would be +/-45 mm, which reaches down to the table and would admit the very
% hypothesis it is there to exclude for anything shorter.
cfg.refine.riseBand          = [0.5 1.15]; % x Extent(3), measured rise above the coarse surface
% 1.15, NOT 1.5, BECAUSE A TALL OBJECT BRINGS THE GRIPPER INTO THE BAND (2026-10-19). The jaws
% hover one approach standoff above the grasp point, which is mid-height, so they rise
% 0.5 + standoff/Extent(3) object heights: 2.3 for a 90 mm mug, but 1.37 for the 188 mm
% Bottle - inside a band reaching 1.5. The highest thing in the band is the top face, so the
% segmentation took the finger pads for the Bottle's top and the jaws closed 30.7 mm off its
% axis. An object's own top measures 0.99-1.00 of its height (the mug 0.999, the Bottle 0.99),
% and on the Bottle's frame a 1.15 cap puts the top face on the cap, 4.5 mm from the axis,
% whichever seed pixel Gemini gave; at 1.25 the fingers still win (22 mm).

% AND THE BAND DOES A SECOND JOB IT WAS NOT DESIGNED FOR: IT EXCLUDES THE ROBOT. The wrist
% camera looks down past its own jaws, and the finger pads sit at the tool point roughly
% 0.10 m below the lens, so they are the nearest thing in frame. Measured, "every pixel
% standing above the support plane within 0.08 m of the seed" returns 19620 pixels with a
% maximum rise of 0.3548 m - the gripper, not the mug - and a 30 mm centroid error. Banding the
% height instead returns 2406 pixels and nothing but the object. The band is therefore load
% bearing for segmentation as well as for the table-versus-object test below.
%
% ONE PIXEL CANNOT LOCATE AN OPEN OBJECT, WHICH IS WHY THE GEOMETRY COMES FROM A REGION.
% Mug_Red is open topped: its top face is an ANNULUS, and measured on the rendered wrist frame
% every plateau pixel lies 27.6 to 35.5 px from the plateau's own centroid, so the centroid
% pixel is in the HOLE and reads the mug's inner floor at z 0.55324 - 87 mm below the rim and
% within 3 mm of the tabletop. Reading depth at the answered pixel therefore cannot work here,
% and the rise gate correctly rejected it. There is no pixel that gives both the axis and the
% top: the centroid pixel gives the axis direction but a depth 87 mm too far, and any rim pixel
% gives the height but an xy offset by the object's radius.
%
% SO THE ANSWERED PIXEL IS A SEED AND THE PLATEAU'S CENTROID IS THE MEASUREMENT, which
% generalises precisely because a disc and an annulus share a centroid. A solid top gives a
% filled plateau, an open top gives a ring, a dome gives a cap, and all three centre on the
% object's axis. Measured against the segmented truth the plateau centroid is exact to 0.00 mm
% where the full band blob is 6.17 mm out - the blob includes outer WALL pixels, which a
% near-overhead view sees asymmetrically, and the plateau excludes them.
cfg.refine.plateauTol        = 0.004;      % m, below the band's highest point is "the top face"
% 4 mm is two depth quantisation steps at this range and narrower than any object feature this
% gripper can pick. Measured, the plateau it selects is 1118 px of the 2406 in the band and its
% rise is 0.0899 m against an Extent(3) of 0.0900 - 0.999 of one height, dead centre of the band.

% THE SEARCH RADIUS SCALES WITH THE OBJECT, because the band cannot bound xy and a fixed radius
% is either too small for a saucepan or wide enough to swallow a neighbour. 1.5 x the larger
% horizontal extent reaches the whole object with half its width to spare on a coarse point that
% is tens of millimetres out. Measured, the result is INSENSITIVE to this over 0.06 to 0.20 m -
% 2406 band pixels and the same centroid at every radius - because nothing else in the scene
% lies inside the height band near the object. The floor guards a degenerate Extent.
cfg.refine.searchRadiusScale = 1.5;        % x max(Extent(1:2))
cfg.refine.searchRadiusMin   = 0.060;      % m
% A FLOOR ON THE PIXEL COUNT, so a handful of stray pixels inside the band cannot be averaged
% into a confident answer. 20 px is far below the 1118 measured here and far above the one or
% two a silhouette artefact contributes. Below it the refinement declines rather than guesses.
cfg.refine.minRegionPixels   = 20;         % px

%% Safety
% The base planner's margin. It used to be the same constant as a clearance monitor's
% threshold; that monitor read a range scan cast through the planner's own map, could never
% fire, and was removed with the scan on 2026-10-19.
cfg.safety.plannerMargin    = 0.10;   % m, planner rejects below this
cfg.safety.staleTimeout     = 1.50;   % s, frames/poses/plans/AI responses
cfg.safety.trackErrMax      = 0.15;   % m, arm end-effector tracking error

% THE MONITOR'S REACH BOUND IS NOT cfg.arm.reach, AND USING IT THERE WAS A DEFECT.
% cfg.arm.reach is 0.850 m, the UR5e's PUBLISHED reach: a horizontal working radius,
% measured to the FLANGE. Every other consumer uses it that way and is right to - the
% stand-off geometry in hmmSetup and every reach measurement behind it are planar
% lever arms to the flange. SafetyMonitor/WorkspaceCheck used it for something else
% entirely: the 3-D distance from arm_mount to the TOOL CENTRE POINT, which on this
% assembly sits 0.1416 m beyond the flange (0.0113 m coupling + 0.1303 m gripper). A radius
% to one point cannot bound the distance to a different point further out, so the guard was
% comparing the wrong quantity against the wrong number.
%
% It shows in the arithmetic. The serial chain arm_mount -> tcp_frame sums to 1.3427 m of
% frame offsets, which for a serial chain is a rigorous upper bound on the tool's distance
% from the mount; 4000 random configurations inside armJointLimits reach 1.2295 m; and
% hmmSetup's own comment records that the fully outstretched pose puts the tool at 1.11 m.
% All three are far beyond 0.850, so the guard could fire on poses that are kinematically
% legal, collision-free and perfectly sensible - and it did. On a measured run the arm
% needed 0.8634 m for 7 of the 128 samples of a transit whose destination was a comfortable
% 0.7152 m, and the plan was rejected for a 1.6% overshoot of a bound that never applied.
% Left unfixed, no plan crossing the mid-transit bulge can ever be accepted.
%
% SIZED AS THE RIGOROUS CHAIN BOUND, rounded up, so the reach half of the guard is a
% KINEMATIC CONSISTENCY CHECK: it cannot fire on any configuration the robot can adopt, and
% it does fire if the tool pose stops agreeing with the joint angles - a wrong tree in the
% Get Transform block, a NaN through the FK, a mismatched frame. That is a real integration
% fault worth latching on, and it is the only thing this test can honestly detect.
%
% THE SPATIAL JOB BELONGS TO cfg.percep.sceneBounds, which is the other half of the same OR
% and is what actually keeps the tool inside the apartment and above the floor. Two tests,
% two distinct purposes; the previous single mis-sized constant blurred them into one that
% did neither job reliably.
cfg.safety.toolReachMax     = 1.35;   % m, arm_mount -> TCP kinematic envelope

% THE MONITOR'S ACCELERATION BOUND IS NOT cfg.arm.jointAccMax EITHER, AND FOR THE SAME
% REASON THE REACH BOUND ABOVE WAS NOT cfg.arm.reach: one constant was doing two jobs that
% happen to be measured in the same units and are otherwise unrelated.
%
%   cfg.arm.jointAccMax is the PLANNER's bound. hmmScreenArmPlan time-parameterises the
%   path at cfg.arm.planMargin of it, so the REFERENCE never demands more than 3.0 rad/s^2.
%   That half works, and it is measured: over the whole of a 4.06 s free move, sampling the
%   realised acceleration on the window derived below, NOT ONE of 407 samples exceeds 4.0.
%
%   SafetyMonitor/JointChecks used the same constant to judge the REALISED acceleration of
%   Plant/ArmKinematics, and that plant cannot honour it. The plant is qdot = sat((cmd - q)
%   / tau): the joint velocity is not a state, it is an ALGEBRAIC function of the command,
%   so it inherits every discontinuity of the command undamped. ArmControl bounds the
%   command's SLOPE at cfg.arm.jointVelMax (ArmControl/JointRateLimit) and nothing bounds
%   its CURVATURE, so the worst legal command transition drives the velocity from 0 to
%   jointVelMax through the lag and the realised acceleration reaches
%
%       jointVelMax / jointTimeConst = pi / 0.08 = 39.27 rad/s^2
%
%   which is 9.8 times the 4.0 the monitor demanded. Measured at the mode handoff of a real
%   run: 38.87 rad/s^2 on joint 1, against a predicted 39.27. The bound was unreachable by
%   the plant it was placed to watch, so it was not a safety threshold at all - it was a
%   guarantee of a latch on the first mode change.
%
% THE TWO LIMITS ALSO CONSTRAIN EACH OTHER, which is worth writing down: a joint cannot
% respect both a velocity ceiling and an acceleration ceiling unless it is allowed
% jointVelMax / jointAccMax = pi / 4 = 0.785 s to reach full speed. Any transition faster
% than that necessarily violates one of the two, whatever the monitor is set to.
%
% THE 39.27 ABOVE IS THE ONE-SIDED CASE, AND THE PLANT IS NOT LIMITED TO IT. That figure is
% the velocity going from rest to jointVelMax. But nothing says the joint is at rest when the
% command steps: if it is already running at -jointVelMax and the new command sends it the
% other way, the same lag carries the velocity across the WHOLE range and the envelope is
%
%       2 * jointVelMax / jointTimeConst = 2 * pi / 0.08 = 78.54 rad/s^2
%
% This is not a hypothetical. Measured at t = 7.34 s of a 20 s run, on the sample after the
% supervisor drops a failed move and returns the arm to IDLE:
%
%       joint   v at t-tau    v at t      dv      dv / vmax
%         2       -2.9442     0.0842    3.0284     0.96
%         5        2.9426    -0.9230   -3.8656     1.23     <-- reversal
%         6       -2.9413     0.9235    3.8647     1.23     <-- reversal
%
% A dv of 1.23 x vmax is arithmetically impossible for any spin-up from rest, so the one-sided
% derivation could not have covered it, and it did not: the realised acceleration was 48.32
% rad/s^2 against a bound of 45. The four joints that reversed did so because the command
% stepped 3.2742 rad in a single sample while they were travelling at 94 per cent of vmax -
% the arm was abandoned mid-flight, which is a legal thing for the supervisor to do and
% therefore has to be inside the monitor's envelope.
%
% SIZED AS THE PLANT'S OWN ENVELOPE, rounded up from 78.54. Like the reach bound, this makes
% the test a CONSISTENCY CHECK rather than a performance check: it cannot fire on any motion
% ArmControl's own rate limiter permits from any initial velocity, and it does fire if the
% realised velocity stops being explicable by the commanded one - a broken rate limiter, a bad
% rate transition, a NaN through the servo. Reference QUALITY is not this test's job and never
% was; hmmScreenArmPlan already certifies it offline against cfg.arm.jointAccMax, which is the
% right constant for that and is unchanged.
%
% THE HEADROOM IS NOT COMFORT, IT IS THE COST OF THE DEFECT UNDERNEATH. A monitor whose
% envelope must admit 78.54 rad/s^2 cannot distinguish a healthy arm from one being thrown
% between disagreeing references, because ArmControl has no bumpless transfer and no curvature
% bound. Widening the bound is the correct fix for the FALSE POSITIVE, and it is explicitly
% not a fix for the step. The step is measured at 3.27 rad here and at 3.32 to 4.29 rad
% between the two motion models, and it is real.
cfg.safety.jointAccMax      = 80.0 * ones(1,6);   % rad/s^2, plant's reachable envelope

% AND THE MONITOR MUST NOT MEASURE IT OVER ONE SAMPLE. This is a measurement fix, separate
% from the threshold above, and it was worth more than the threshold was.
%
% cfg.arm.jointTimeConst's own comment already names the hazard - "a derivative of a signal
% that has discontinuous slope by construction is noise" - and then JointChecks walked into
% it, because sat((cmd - q)/tau) IS that signal: algebraic in a rate-limited command, so its
% slope is discontinuous by construction. A Discrete Derivative over one 0.01 s step divides
% by a hundredth of a second, which multiplies whatever ripple rides on the velocity BY ONE
% HUNDRED. Two per cent of the velocity limit - 67 mrad/s, invisible on any plot of v - reads
% as 6.7 rad/s^2.
%
% MEASURED over the planned free move that the planner had already certified. Exceedances
% of 4.0 rad/s^2 during the move, by differencing window:
%
%       N = 1  (0.01 s)   163 samples over, worst 12.71 rad/s^2
%       N = 2  (0.02 s)     6 samples over, worst  6.36
%       N = 8  (0.08 s)     0 samples over
%
% Every one of them was the ruler. They are not a spike the old three-sample median could
% reject either - they ALTERNATE in sign, sample after sample, so every window of three
% contains a large value and the median stays large. That is why the previous diagnosis read
% "sustained motion" off a joint turning at two per cent of its velocity limit.
%
% THE WINDOW IS THE PLANT'S TIME CONSTANT, not a filter tuned until the warnings stopped.
% The plant's bandwidth is 1/tau = 12.5 rad/s, just under 2 Hz; a one-sample difference at
% 100 Hz resolves content to 50 Hz, twenty-five times past anything this plant can express,
% so it reports almost nothing but artefact. Differencing over tau measures exactly the band
% the plant has. It is deliberately NOT longer: the real violation at the mode handoff
% survives this window undiminished - 124 samples still over, worst 43.41 - so the window
% removes the artefact without blunting the check, which is the whole test of whether a
% smoothing choice is honest.
%
% Expressed in samples because a Delay block counts samples. JointChecks runs at
% cfg.rates.control, VERIFIED as CompiledSampleTime 0.01 s and not assumed - the divisor is
% N * Ts, so reading the rate wrong rescales every number above.
cfg.safety.jointAccelWindow = round(cfg.arm.jointTimeConst / cfg.rates.control);  % samples

%% Sim3D render appearance
cfg.render.creaseAngle      = 30*pi/180;  % rad, split normals above this
cfg.render.metallic         = 0.15;
cfg.render.roughness        = 0.55;
cfg.render.vertexBlend      = 1;
% THE ROBOT'S COLOURS ARE THE REAL HARDWARE'S, applied by hmmMirrorTree to the render tree.
% Before 2026-10-16 every mesh this project adds rendered in addVisual's default 0.5 grey
% and the UR5e in loadrobot's uniform 0.7, and three colour constants that stood here
% (baseColor, armColor, gripperColor) were read by nothing. Each row is a regular expression
% on the BODY NAME and the RGB it renders in; the first match wins, and a body that matches
% none keeps its source colour - which is how the Robotiq 2F-85 keeps the vendor's own
% black-and-silver from loadrobot. The base matches Elephant Robotics' myAGV Pro product
% photos (slate-grey shell, black mecanum wheels and lidar), judged in the RENDER rather
% than by eye on the numbers: matte, a grey of 0.42 came out near white. The UR5e row is
% only a fallback - see cfg.render.armMaterial.
cfg.render.robotColor = { ...
    '^chassis_link$',  [0.16 0.17 0.19]   % myAGV Pro shell, slate grey
    '^wheel_',         [0.04 0.04 0.04]   % mecanum wheels, black
    '^laser_link$',    [0.08 0.08 0.09]   % lidar, black
    '^pedestal_link$', [0.16 0.17 0.19]   % riser, the shell's grey
    '^(base_link_inertia|shoulder_link|upper_arm_link|forearm_link|wrist_[123]_link)$', ...
                       [0.80 0.81 0.82]   % UR5e, if its material labels do not apply
    '^coupling_link$', [0.10 0.10 0.12]   % Robotiq coupling, black anodised
    '^wrist_cam$',     [0.15 0.15 0.16]}; % wrist camera

% THE UR5e IS PAINTED BY MATERIAL, NOT BY LINK. Universal Robots' own visual meshes tag every
% triangle LinkGrey, JointGrey, Black or URBlue, and loadrobot's meshes are those same
% triangles (matched on all seven links at 0.00 mm), so each link
% renders as its housing, its light-blue cap and its black rings. The RGB values are read off
% UR's e-Series product photograph rather than taken from the meshes: their JointGrey (0.28)
% renders the housings near black, where the real ones are the same aluminium as the tubes.
% THE FINISH IS SET ON THE ACTORS, NOT IN THE TREE (hmmStyleRobot): sim3d.Actor.load gives every
% body Metallic 0.70, Shininess 0.70, Specular 0.50, so the base rendered as metal. The base is
% painted shell, rubber and plastic, so it is matte; the arm and the gripper keep the default.
% Rows are body-name regular expressions; robotActorName is RobotTree's ActorName.
cfg.render.robotActorName = 'HomeMobileManipulator';
cfg.render.robotFinish = { ...
    '^(chassis_link|laser_link|pedestal_link)', struct('Metallic', 0, 'Shininess', 0.35, 'Specular', 0.40)
    '^wheel_',                                   struct('Metallic', 0, 'Shininess', 0.20, 'Specular', 0.30)};

cfg.render.armMaterial = struct( ...
    'LinkGrey',  [0.80 0.81 0.82], ...    % tubes, brushed aluminium
    'JointGrey', [0.66 0.67 0.69], ...    % joint housings, aluminium a shade darker
    'Black',     [0.05 0.05 0.05], ...    % seal rings
    'URBlue',    [0.55 0.78 0.92]);       % joint caps, UR light blue
cfg.render.sunAltitude      = 65;
cfg.render.sunAzimuth       = 180;
cfg.render.sceneName        = 'Empty scene';

% THE ROBOT'S RENDER FRAME IS NOT THE SCENE'S WORLD FRAME, and this factor is the
% conversion between them. It is MEASURED, not chosen - see below.
%
% Two different paths place things in the Unreal scene. Everything except the robot
% goes through a Simulation 3D Actor or Camera block's Translation, which the scene
% consumes as given, so cfg.scene.extent, the prop poses, the camera stances and the
% occupancy grid all share one frame. The robot instead has its actor root pinned at
% the origin (initialTrans = [0 0 0], inpTransEnbVal = 'off') and is placed by the
% first three joints of its rigidBodyTree - base_x_joint, base_y_joint,
% base_theta_joint - which the Simulation 3D Robot block reads as a ROS-convention
% tree (x forward, y LEFT) and converts to Unreal's frame (y RIGHT) by negating y.
% Feed that slot a pose that is already in the scene frame and the negation happens
% once too often: the robot lands mirrored about the y = 0 plane.
%
% THE MEASUREMENT that fixes the sign. A camera placed inside the model's own world
% at the cfg.view stance, with its depth back-projected to world coordinates, put the
% rendered robot at x [4.93 5.49] centroid 5.20, y [3.83 4.21] centroid +4.03,
% z [0.01 1.25], against a commanded cfg.base.initialPose of [5.20 -4.00 0] - x
% correct to 0.00 m, y negated - while the SAME frame put the building at x
% [-0.15 10.15] y [-8.14 0.16] z [-0.01 2.70], i.e. exactly where
% the shell mesh's own footprint puts it. One axis, one sign, everything else agreeing.
%
% RETIRED 2026-09-28, AND THE REASON IS WORTH THE SPACE, because the field was RIGHT
% ABOUT THE DEFECT AND WRONG ABOUT ITS EXTENT, and that combination hid a 76 mm grasp
% error behind a correct-looking chassis for the life of the project.
%
% What used to live here was cfg.render.baseRenderFlip = [1 -1 -1], applied by a Gain
% on Plant/RobotActor/RenderFrameFlip as [cfg.render.baseRenderFlip, ones(1,12)]. The
% diagnosis above is correct: the block negates y. The error is in the word ONLY. The
% block negates y on EVERY LINK of the tree it is given, not just on the three base
% joints, and the twelve numbers the Gain passed through untouched are joint ANGLES -
% there is no scalar you can multiply a joint angle by to cancel a frame change. So the
% chassis was un-flipped correctly and every link hanging off it kept the conversion:
% a correctly placed base carrying a mirror-image arm. The error is 2*|y| in
% chassis-local coordinates, which vanishes for a centred arm and grows as the tool
% reaches sideways - exactly what a grasp does. At the grasp instant the rendered tool
% sat 76.6 mm from the commanded pose against 7.5 mm of jaw clearance per side.
%
% THE CLAIM THAT WAS DELETED FROM HERE, so it is not re-derived. This comment used to
% argue the arm was measured too, on the grounds that the rendered jaw silhouette's y
% range -3.0686..-2.9327 STRADDLES the tcp's y of -3.0142, and that "a mirror about the
% chassis xz-plane would put the whole silhouette on one side of it". That inference is
% refuted. Projecting the two finger pads through the same frame's camera pose predicts
% cols 187.5 / 452.2 unmirrored and 404.7 / 140.0 mirrored; the render shows 142.5 and
% 410.8 - the mirrored prediction WITH LEFT AND RIGHT SWAPPED, to 2.5 and 6.1 px,
% against 45 and 41 px for the unmirrored one. A straddling silhouette proves nothing,
% because a mirrored gripper still straddles: the mirror swaps the jaws, it does not
% move them to one side. The lesson is the same one this project keeps relearning - an
% extent test cannot settle a question about a centre.
%
% WHAT REPLACES IT. functions/hmmMirrorTree.m mirrors the WHOLE tree, and the mirrored
% copy is handed to Plant/RobotActor/RobotTree as its RigidBodyTree while the block is
% driven by the TRUE configuration, so the block's own negation cancels exactly and the
% Gain is gone. The witness is M*FK_mirror(q)*M == FK_true(q) to 0.000e+00 over random
% configurations, and the drawn geometry - not just the link frames - agrees to
% 0.0000e+00 m over 81 visual-configuration pairs, because the block reflects the MESHES
% too and mirroring the frames alone leaves every mesh reflected in its own body frame
% (visibly: detached gripper, displaced wrist link, four displaced wheels).
%
% THE SIGN OF THE ENGINE CHECK FLIPPED WHEN THE FIX LANDED, which is the point of it. It
% used to be the MIRRORED tree that reproduced the engine's stored transforms on 37 of 37
% links, to 0.0000 mm and 0.0014 deg - that was the proof the block negates y. Now it is
% the TRUE tree that matches, to 0.0000 mm and 0.0016 deg, and the mirrored tree misses by
% 6396 mm. Do not read the old number as the target; score both trees and name the
% winner. The measurement quoted further up - rendered robot at y +4.03 against a
% commanded -4.00 - still stands as the original evidence that the block negates y; only
% the remedy has changed.

%% Plant actor naming (handle convention)
cfg.actor.robotName         = 'HomeMobileManipulator';
% Registry CAPACITY, not scene content. This sizes the MovableRegistry bus, so
% it has to be a compile-time constant, and it has to exceed the number of props
% the manifest actually lists or hmmDiscoverMovables silently drops the tail -
% an object that cannot be picked for no visible reason. The reused apartment
% publishes 20 props; 24 leaves room for a few more without re-baselining buses.
cfg.actor.maxMovable        = 24;     % capacity of the movable-prop registry
cfg.actor.noneId            = uint8(0);   % TargetActorId sentinel for "none"
% NAME WIDTH, NOT NAMES. Simulation 3D Robot's TargetActor input is addressed by
% actor NAME, not by id: robotics.slsim3d.internal.util.TargetGraspUtils reads a
% scalar uint8 as "this input is disabled" and anything longer as char(name), so
% the grasp command has to cross the diagram as a fixed-width uint8 row. A
% Simulink signal has one width for the whole run and the block does no trimming,
% which is why hmmActorNames pads every actor name to exactly this many
% characters - world.Actors.('Mug_Red   ') is not world.Actors.Mug_Red, and a
% mismatch here is a hard error rather than a missed grasp.
%
% This is a CAPACITY like maxMovable above, and deliberately not the measured
% longest name: a width that tracked the manifest would resize a port whenever
% the scene changed. 24 exceeds the 14 characters the reused apartment's longest
% key needs and stays well inside namelengthmax, so the padded name is still a
% valid struct field. hmmActorNames errors if a manifest key does not fit rather
% than truncating it, because a truncated name is a lookup that finds the wrong
% actor or none at all.
cfg.actor.nameWidth         = 24;     % characters in a padded Sim3D actor name
cfg.actor.namePad           = '_';    % pad character; keeps the name a valid field
% FAKE GRAVITY AT RELEASE (hmmSettleProp). The plant has no physics, so a released prop is
% rested on the swept shell surface under its footprint. Corrections smaller than this are
% left alone - the release already aims cfg.plan.releaseClearance (3 mm) above the surface,
% and moving a prop by a millimetre would only make a correct place flicker.
cfg.actor.settleTol         = 0.005;  % m

%% Rigidity verification tolerance
cfg.verify.rigidityTol      = 1e-9;   % m and rad, actor chain vs getTransform

%% Vision-language provider
% NOTHING here identifies a credential. The key is resolved at RUNTIME by
% hmmProviderRequest in this order: GEMINI_API_KEY environment variable, then
% a key file found by the SAME discovery walk that locates the apartment
% assets, then the sample responses. No absolute path, no file name pattern
% that is only meaningful on one machine, and no key material is committed
% here or written to any project file. Only the resolution ROUTE is reported,
% on the PerceptionStatus bus, as one of the ROUTE_* codes below.
% SAMPLE RESPONSES ARE THE DEFAULT ('replay'), deliberately. 'auto' would send a
% paid request to a remote model on every run of a shipped example, on a
% credential the person running it may not have meant to spend, and two runs
% would not agree - which makes a regression indistinguishable from provider
% variance. The sample responses in perception/gemini/responses are NOT Gemini
% output: they were generated from this scene in the Gemini Robotics-ER response
% format, so the offline default is a real grounding of the real apartment rather
% than a stub. Set GEMINI_PROVIDER=live (or 'auto') to exercise the network path;
% no code change is needed.
cfg.provider.mode            = 'replay';  % 'auto' | 'live' | 'replay' (= sample responses); GEMINI_PROVIDER overrides
% The Embodied Reasoning model, which is the one that answers with grounded image
% POINTS rather than prose. Verified against the credential's own model list:
% 'gemini-robotics-er-1.5-preview' is not on it and returns HTTP 404, which is why a
% model name is confirmed by listing rather than assumed.
cfg.provider.model           = 'gemini-robotics-er-2-preview';
cfg.provider.endpoint        = 'https://generativelanguage.googleapis.com/v1beta/models';
cfg.provider.temperature     = 0.20;      % low: this is grounding, not prose
cfg.provider.timeout         = 20.0;      % s, single request budget

% A 5xx IS RETRIED AND NOTHING ELSE IS, which is a measurement rather than a default.
% gemini-robotics-er-2-preview returned HTTP 503 on the wrist-refinement request of two
% consecutive live runs - at the one instant in the run that matters, because the refinement
% is asked for exactly once - while three standalone refine-shaped requests issued seconds
% apart all returned 200 in 3.6, 11.4 and 6.7 s. So the request shape and the credential are
% not the problem: a preview model answers "overloaded" intermittently, and one transient 5xx
% was costing the whole refinement. The retry is what turns an isolated blip into a
% two-second delay.
%
% BUT IT DOES NOT ALWAYS RESCUE IT, AND THE ORIGINAL WORDING HERE OVER-CLAIMED THAT IT DID.
% Measured again on 2026-10-05, in the two map-verification runs: the refinement reported
% httpStatus 503 with a tap latency of 19.9025 s. That latency IS the whole transaction, so
% all three attempts ran - 19.9025 less the 1.5 s and 3.0 s backoffs leaves 15.40 s over
% three tries, about 5.1 s each, which sits inside the 3.6/11.4/6.7 s spread the successful
% standalone calls showed. So the server spent real time on each one and refused it anyway:
% this was a sustained overload lasting longer than the whole ~20 s retry window, not a
% single blip. Raising the retry count would only spend more of the model's thread on it.
% The run is unharmed either way - the refinement is advisory and the coarse grounding point
% is kept - but a run whose wrist refinement reports PROVIDER/503 has not exercised the
% refinement at all, and should not be read as evidence about it.
%
% NOT A TIMEOUT, AND NOT A 4xx. A timeout has already spent cfg.provider.timeout on the
% model's own thread, so retrying one costs another 20 s inside a single step - the cure is
% worse. A 4xx is a statement about the request, and the request will not improve by being
% sent again: a 400 schema error, a 403 bad key and a 404 wrong model name are all permanent
% until something in the project changes. Retrying them would turn a crisp diagnosis into a
% slow one and blur the five failure modes the provider path keeps distinct.
cfg.provider.retries5xx      = 2;         % extra attempts after a 5xx, 0 disables
% EXCEPT A GROUNDING TIMEOUT, WHICH GETS ONE MORE TRY (2026-10-08). The argument above holds for
% the wrist refinement, which degrades to a depth-only measurement, but not for grounding: with
% no action queue there is no task, and on Run 99 a single lost reply failed a transfer at
% t = 1.9 s while the identical request on the next run grounded normally. Twenty more seconds
% inside one step is cheaper than an operator re-typing the sentence.
cfg.provider.retriesTimeoutGround = 1;    % extra attempts after a grounding timeout, 0 disables
cfg.provider.retryBackoff    = 1.5;       % s before the first retry, doubled each time
% HTTP 429, A FREE KEY'S RATE LIMIT (hmmProviderRequest): waited out once, for as long as Google's
% reply asks (RetryInfo.retryDelay), capped, or the default when it does not say. On 2026-10-16 back-
% to-back test runs were refused 429 and a request a minute later went through.
cfg.provider.retries429      = 1;
cfg.provider.retry429Default = 10;        % s
cfg.provider.retry429MaxWait = 30;        % s
cfg.provider.confidenceMin   = 0.35;      % below this an action is rejected (LowConfidence)
cfg.provider.instructionChars = 256;      % capacity of the instruction field
cfg.provider.labelChars      = 64;        % capacity of an object label
cfg.provider.ROUTE_NONE      = uint8(0);  % no credential and no replay corpus
cfg.provider.ROUTE_ENV       = uint8(1);  % GEMINI_API_KEY environment variable
cfg.provider.ROUTE_KEYFILE   = uint8(2);  % key file found by the asset discovery walk
cfg.provider.ROUTE_REPLAY    = uint8(3);  % recorded response corpus

%% Action queue
% Fixed capacity because the queue crosses a bus and a bus element cannot be
% variable-size. Overflow is REPORTED, never silently dropped.
cfg.queue.capacity           = 8;
cfg.queue.VERB_NONE          = uint8(0);
cfg.queue.VERB_TRANSFER      = uint8(1);
cfg.queue.VERB_NAVIGATE      = uint8(2);
% VERB_HOME is the one verb that needs NO IMAGE and NO GROUNDING. "Go home" names
% a pose this configuration already knows (cfg.task.homePose), so the provider is
% asked only to RECOGNISE the intent, not to locate anything - which is why the
% home case in hmmValidateActions reads no point and no camera index, and why
% hmmReconstructTargets short-circuits it without touching depth. It is also the
% only verb whose goal yaw is prescribed rather than derived from a bearing; see
% cfg.task.homePose.
cfg.queue.VERB_HOME          = uint8(3);
cfg.queue.EXEC_PENDING       = uint8(0);
cfg.queue.EXEC_ACTIVE        = uint8(1);
cfg.queue.EXEC_DONE          = uint8(2);
cfg.queue.EXEC_FAILED        = uint8(3);

%% Structured failure codes
% Every rejection path in the model reports one of these. A code is never
% reused for two different causes, because "it failed" is not a diagnosis.
cfg.fail.OK                  = uint8(0);
cfg.fail.MalformedJSON       = uint8(1);   % response is not the agreed schema
cfg.fail.UnknownAction       = uint8(2);   % verb outside {transfer, navigate, home}
cfg.fail.UnknownCamera       = uint8(3);   % *_image outside 1..nFixed
cfg.fail.OutOfRange          = uint8(4);   % *_point outside 0..pixelNormScale
cfg.fail.LowConfidence       = uint8(5);   % below cfg.provider.confidenceMin
cfg.fail.DepthInvalid        = uint8(6);   % no valid calibrated hit near the pixel
cfg.fail.OutsideRegion       = uint8(7);   % reconstructed point outside the scene bounds
cfg.fail.AmbiguousObject     = uint8(8);   % zero or several movable actors match
cfg.fail.NotGraspable        = uint8(9);   % measured extent outside the jaw stroke
cfg.fail.PlaceUnsupported    = uint8(10);  % no support surface fits at the place point
cfg.fail.Unreachable         = uint8(11);  % IK found no admissible arm solution
cfg.fail.Stale               = uint8(12);  % input older than cfg.safety.staleTimeout
cfg.fail.ProviderNoKey       = uint8(13);  % no credential and no replay corpus
cfg.fail.ProviderError       = uint8(14);  % transport or HTTP failure
cfg.fail.ProviderTimeout     = uint8(15);  % exceeded cfg.provider.timeout
cfg.fail.QueueOverflow       = uint8(16);  % more actions returned than capacity
cfg.fail.PathPlanFailed      = uint8(17);  % no collision-free base path
cfg.fail.ArmPlanFailed       = uint8(18);  % arm planning failed, cause not narrowed further
cfg.fail.CollisionReject     = uint8(19);  % every grasp candidate collided
% THE ARM PLANNER'S SIX REJECTIONS, SEPARATED. hmmScreenArmPlan used to return
% ArmPlanFailed from six different gates, which broke the rule three lines above:
% the search finding no path, the path folding the arm into itself, no time
% parameterisation existing under the joint boxes, the parameterised spline leaving
% the position box, and the reference driving into the support surface are five
% different defects with five different fixes, and one code could not tell them
% apart. It cost a whole diagnosis: a run that ended in ArmPlanFailed was read as
% "the planner refused" when the failure was in fact the servo tracking guard, and
% the only way to find out was to instrument the function by hand. 18 is kept for
% the blank plan and for a target that arrived invalid without saying why.
cfg.fail.ArmRrtFailed        = uint8(20);  % manipulatorRRT found no path stow -> grasp
cfg.fail.ArmSelfCollide      = uint8(21);  % a path waypoint or midpoint self-collides
cfg.fail.ArmTimeParamFailed  = uint8(22);  % no parameterisation under the joint vel/acc boxes
cfg.fail.ArmRefOutOfBox      = uint8(23);  % the parameterised spline leaves the position box
cfg.fail.ArmRefCollide       = uint8(24);  % the parameterised reference hits the surface
cfg.fail.ArmRefWorkspace     = uint8(25);  % the reference leaves SafetyMonitor's workspace
% AND THE BASE PLANNER'S FIVE, SEPARATED FOR THE SAME REASON. hmmPlanBasePath
% returned PathPlanFailed from five gates: the start state occupied, the goal state
% occupied, both searches finding nothing, contopptraj failing, and the
% parameterised reference colliding. The 20 s integration run reported 17 twenty
% milliseconds into a retry, and reading the logs could not distinguish "the goal is
% inside the inflated side table" from "the pose the base is standing in is" - which
% are opposite defects: one moves the goal, the other says the robot may leave where
% it is. Establishing which took a photograph of the map and an offline replan.
% 17 IS KEPT and is still what a blank plan carries, exactly as 18 is for the arm,
% so cfg.fail.PathPlanFailed remains the chart's F_PATH and the code a navigation
% phase that outlives tNavMax reports.
cfg.fail.BaseStartOccupied   = uint8(26);  % start pose in collision, or no escape from the margin
cfg.fail.BaseGoalOccupied    = uint8(27);  % goal state inside the inflated map
cfg.fail.BaseSearchFailed    = uint8(28);  % BiRRT and RRT* both found no path
cfg.fail.BaseTimeParamFailed = uint8(29);  % no parameterisation under the base vel/acc boxes
cfg.fail.BaseRefCollide      = uint8(30);  % the parameterised reference is not collision-free
% AND ONE MORE ON THE ARM SIDE, for the same "one code for several causes cannot be
% debugged" reason as 20-25. The free move is planned to a STANDOFF pose now rather
% than to the grasp itself, so there is a new way to fail that has nothing to do
% with the grasp being reachable: the grasp converges, is collision-free and is well
% conditioned, and the pose 120 mm back along the approach axis is the one IK cannot
% reach or cannot reach without collision. Reporting that as Unreachable (11) would
% say the object cannot be grasped, which is measurably false at that point.
cfg.fail.PregraspUnreachable = uint8(31);  % the standoff pose ahead of the grasp is not solvable
% AND ONE THAT EXISTS TO MAKE A FAILURE FAST RATHER THAN TO MAKE IT LEGIBLE, which is a
% different job from 26-30. Those split one code into five so a log could be read. This
% one is reported by a screen that runs BEFORE any sampling, and the reason it is worth a
% code of its own is that the alternative is what the shipped model did: an unreachable
% goal ran plannerBiRRT to its 4000-iteration budget, then plannerRRTStar to its own,
% then the supervisor retried and both ran again, all inside single time steps in which
% the model clock does not advance. What that looks like is a simulation that has stopped
% - which is precisely how it was reported - and at the end of it the verdict is
% BaseSearchFailed, indistinguishable from a search that was merely unlucky.
%
% A goal in a walled-off pocket is not unlucky and does not need sampling to detect. A
% grid search either connects the start cell to the goal cell or it does not, in
% milliseconds, and when it does not no amount of RRT will help. 28 is kept for a search
% that failed inside a region it could have crossed, which is the case retrying is for.
cfg.fail.BaseGoalUnreachable = uint8(32);  % no connected route exists, found before searching
% AND ONE MORE FOR THE SAME REASON 32 EXISTS - fast and honest instead of slow and
% misattributed - except that the misattribution here cost more than the delay did. A place
% surface ABOVE THE ARM'S REACH used to be reported as BaseGoalOccupied (27), which names the
% map. Nothing was wrong with the map. The route is: hmmArmLeverBand returns an empty band
% (hiLever < loLever) for a height off the end of cfg.arm.leverCeiling, freeBearings asks
% hmmStandoffRing with that band and armCanWork rejects all 24 bearings x 24 headings = 576
% candidates, every candidate scores zero, biasToReachable declines to shift, the place point
% is published anyway, the base drives to it, and hmmPlanBasePath - whose signature carries no
% target and so cannot know about the arm - refuses the goal state and reports 27.
%
% WHAT THAT COSTS IS NOT THE RUN, IT IS THE DIAGNOSIS. 27 sent a reader to the occupancy grid:
% photographing the map, counting occupied cells, measuring free bearings at the shelf,
% widening the shelf, deepening the shelf - and deepening it made the count WORSE, because the
% map is 2-D and a deeper shelf eats more floor. None of it could have worked, because the arm
% cannot hold a tool at 1.3236 m from any base pose whatsoever. See README, Known Limitations.
%
% THE TEST IS SOUND IN THE DIRECTION THAT MATTERS, which is why this is a refusal and not a
% warning. The band is a function of the target height and the held object alone - joint 1
% absorbs azimuth exactly, measured to 1e-8 over twelve azimuths - so an empty band is a
% statement about every base pose at once, not about the one that was tried. And it is screened
% at the SUPPORT height, while the plan must additionally solve cfg.sup.approachDepth ABOVE
% that; a higher target has a smaller band, so empty at the surface guarantees empty at the
% approach. It can therefore refuse a place no base pose could serve, and cannot refuse one
% some pose could. hmmArmLeverBand's header invites exactly this: it returns the empty band
% "to let callers refuse rather than clamp", and until now no caller took it up.
%
% AND IT NOW COVERS A SECOND CAUSE, BECAUSE THE FIRST ONE DID NOT CATCH THE KITCHEN COUNTER.
% An EMPTY band is one way a place is unreachable; the other is a band that exists but is too
% SHORT for any radius the map allows, and that one slipped straight through to 27 again. On
% the counter the provider chose for one transfer the band was [0.130, 0.552] m at z = 0.910 m
% holding the mug - not empty - while the occupancy map would not let the base closer than
% ~0.72 m to the worktop. Every radius the map accepts is beyond 0.552 m of lever and every
% radius the arm can reach is inside the counter: the window is empty by ~0.28 m, and no
% stand-off, arrival heading or goal tolerance recovers it. The run drove to the kitchen,
% picked the mug, dispatched the place goal at t = 11.92 s and got BaseGoalOccupied 0.02 s
% later - the same misattribution as above, arriving after 12 s of committed motion.
%
% SO 33 IS NOW RAISED FROM TWO PLACES IN fitSupport, AND THE SECOND ONE NEEDS THE MAP. It
% screens the point being PUBLISHED, after the local-height correction, by asking freeBearings
% - hmmStandoffRing against the planner's own validator - for a single non-zero count. That
% placement is a measurement, not taste: biasToReachable scores candidates too and on that run
% it found one, a single free bearing at the fitted z = 0.900, and the correction then levelled
% the point onto the counter's upper strip at 0.910, where the ceiling falls 0.567 -> 0.552 m
% and that bearing is gone. Gating on what the bias knew would have passed the run. Being
% map-dependent, the second gate is skipped under cfg.map.mode = 'live'; the first is not,
% which is why they are two gates and not one.
cfg.fail.PlaceUnreachable    = uint8(33);  % no base pose can serve this place point
cfg.fail.ReplayMiss          = uint8(34);  % no recording for this instruction and request kind

%% Perception reconstruction
% All of these are properties of the RIG and the GRIPPER, never of an object.
%
% pixelRadius SCALES WITH THE RIG, and it did not used to, which made it the one place
% cfg.camera.pixelScale was NOT the pure change of pixel units the camera note claims.
% imageSize, focalLength and opticalCenter all scale; a bare 24 px did not, so halving the
% rig doubled this window in ANGLE and quadrupled the rays it admitted:
%
%   pixelScale   raySpan   assoc window            rays      place window (3x)    rays
%   1            20.0 px   24 px = 1.20 pitch, 2.99 deg   4.5   72 px, 8.90 deg    41
%   1/2 unscaled 10.0 px   24 px = 2.40 pitch, 5.96 deg  18.1   72 px, 17.38 deg  163
%   1/2 scaled   10.0 px   12 px = 1.20 pitch, 2.99 deg   4.5   36 px, 8.90 deg    41
%
% The middle row is the bug. A 17 deg plane-fit cone around a place point on a counter
% reaches the wall behind it, so the support fit would have put one plane through both
% and the association would have clustered across surface boundaries - a reconstruction
% error with no failure code, blamed on the provider. Scaling restores the bottom row,
% which is angularly identical to the baseline. At pixelScale 1 this is 24 px exactly as
% before, so the expression changes nothing about the shipped reference run.
cfg.percep.pixelRadius       = 24 * cfg.camera.pixelScale;   % px, association radius
% THIS VALUE IS LOAD-BEARING IN A DIRECTION ITS NAME DOES NOT SUGGEST, AND SINCE THE RAY
% GRID WAS RAISED IT IS LOAD-BEARING ONLY AT LONG RANGE. It is what decides whether
% ExtentSource reads EXTENT_ACTOR or EXTENT_MEASURED, because hmmReconstructTargets switches
% to EXTENT_MEASURED when the seed's cluster holds minClusterHits hits, and a cluster only
% grows when two valid hits land within clusterTol of each other. Neighbouring rays are a
% fixed ANGLE apart, so their metric spacing scales with range:
%
%   ray pitch = imageSize(1)/rayGrid(1) = 360/72 = 5 px
%   angular   = 5 / focalLength(1) = 5/230 = 0.02174 rad
%
%     range    edge-neighbour    diagonal     joins the seed?
%     1.50 m        32.6 mm       46.1 mm     edge yes, diagonal no
%     2.00 m        43.5 mm       61.5 mm     edge yes, barely
%     2.07 m        45.0 mm       63.6 mm     THE CROSSOVER
%     2.42 m        52.6 mm       74.4 mm     no - every cluster is one hit
%     3.00 m        65.2 mm       92.2 mm     no
%
% So clusterTol = 45 mm is exactly the spacing at 2.07 m, and the behaviour is split either
% side of it. Beyond 2.07 m nothing can join, nInCluster is 1, and the extent comes from the
% actor registry. Inside 2.07 m a seed plus its four edge-neighbours clusters to 5 hits,
% clears minClusterHits = 3, and EXTENT_MEASURED fires. The reference pick sits at 2.42 m -
% on the safe side - which is why the shipped run reports EXTENT_ACTOR and why this looked
% like a one-sided guard when it was first written down. It is not one-sided.
%
% THREE OF THE TWENTY MOVABLES ARE ALREADY INSIDE IT, measured against all four fixed camera
% mounts: Glass_Bathroom at 1.58 m from camera 4, Glass_Kitchen at 1.82 m from camera 2, and
% SoapDispenser at 1.94 m from camera 4. Next nearest are Plate at 2.30 m and Stool_1 at
% 2.33 m, then Mug at 2.42 m from camera 1 - the reference pick. Which camera grounds an
% object is the provider's choice, so for those three the branch taken depends on the
% sentence. All three are small objects, which is where an over-measured extent costs most.
%
% WHY THE MEASURED BRANCH IS THE WORSE ONE, so the asymmetry is not read as capability.
% Every movable in this apartment is smaller than one ray cell at working range, which is
% the assumption the Position(3) reading in hmmReconstructTargets rests on: every hit in the
% cluster comes off the SURFACE AROUND the object, not the object's own top face. A cluster
% that does form therefore measures the patch of worktop surrounding the mug and reports it
% as the mug's Extent, inflating it toward the window size with no failure code and no way
% for a consumer to tell. Both the grasp width and the carried-footprint inflation read
% Extent. hmmReconstructTargets already overwrites Extent(3) from the registry on this
% branch, which caps the vertical error but leaves the xy extent measuring the surface.
%
% AT THE PREVIOUS GRID ([36 64], 10 px pitch) the crossover was 1.04 m, closer than the
% arm can work, so the measured branch was genuinely unreachable and the single-hit
% cluster was doing segmentation by accident. Raising the grid for the place leg's inlier
% gate moved the crossover out to 2.07 m and put it inside the range band a pick can
% legitimately happen at. One more doubling puts it at 4.14 m and the measured branch
% becomes the normal case. The fix at that point is segmentation - separating object hits
% from surface hits before measuring - not a larger clusterTol, which only merges the two
% more aggressively; cfg.percep.clusterTol and cfg.depth.rayGrid are coupled and neither
% comment used to say so.
cfg.percep.clusterTol        = 0.045;   % m, ray hits within this join one surface cluster
cfg.percep.minClusterHits    = 3;       % fewer valid hits than this is not a measurement
cfg.percep.extentAssumed     = 0.060;   % m, reported extent when only one hit survives
cfg.percep.assocMargin       = 0.060;   % m, nearest actor must beat the runner-up by this
cfg.percep.assocRadius       = 0.250;   % m, an actor beyond this is not a match
cfg.percep.planeMaxDistance  = 0.020;   % m, pcfitplane inlier distance
cfg.percep.planeMinInliers   = 24;      % below this the surface fit is not trusted
% AND 24 IS A HIDDEN RANGE TEST, which is why the two values below exist. The ray grid is
% fixed in ANGLE, so the number of rays inside the place window falls as range squared even
% though fitSupport deliberately holds the window's METRIC span at 0.90 m. Measured at the
% grid this defect was found on - cfg.depth.rayGrid = [36 64] over [360 640], one ray every
% 10 px - the window holds 150 rays at 1.5 m, 54 at 2.5 m and
% 41 at 3.0 m, so a flat count of 24 demands 16%, 45% and 59% of the whole window be one
% coplanar surface. Expressed as the surface it accepts, 24 inliers means ~0.53 m across at
% 2.5 m and ~0.64 m at 3.0 m - and the window is 0.90 m across while a domestic worktop is
% ~0.6 m deep, so the window ALWAYS straddles the front edge and the wall behind, both of
% which fall outside the slab. The counter therefore passes only when the robot is close to
% it. Measured: a place point on the kitchen worktop at ~2.5 m was refused
% PlaceUnsupported with the fit starved of inliers, and fitSupport's own header already
% records the same gate rejecting a window that held 23 counter hits against the 24.
%
% So the requirement is restated in the units it is actually about - how wide a surface has
% to be before it counts as somewhere to put something down - and converted to a ray count
% at the range the measurement was taken from. supportMinSpan is the physical claim;
% planeMinInliersFloor is the separate claim that below a handful of points a plane fit is
% not worth having however large the surface is. The derived count is CAPPED at
% planeMinInliers so this can only ever relax the gate, never tighten it: every place that
% succeeds today still succeeds, and the change is confined to ranges where the old count
% was asking for a surface the apartment does not contain.
%
% RELAXING THE COUNT IS SAFE BECAUSE THE REGION GATE IS THE REAL BACKSTOP. A plane fitted
% to eight inliers spread over a small patch yields halfU/halfV that the surfaceInset
% leaves at or below zero, so fitSupport rejects it anyway - and the tilt gate still
% screens the normal. The count was never the thing protecting the place point from a bad
% surface; it was protecting it from a bad NORMAL, and 8 points plus a 15 deg cone does
% that.
cfg.percep.supportMinSpan    = 0.300;   % m, narrowest surface that counts as support
cfg.percep.planeMinInliersFloor = 8;    % never trust a plane fitted to fewer than this
% WINDOWTARGETHITS IS NOT AN INLIER GATE, AND IT USED TO BORROW planeMinInliers' NAME.
% nearRays grows its pixel window - 1x, 2x, then 4x the requested radius - until it holds
% this many VALID hits, and then stops growing. Two things make it a different quantity
% from the two above even when the number is the same:
%
%   - it counts valid hits, not COPLANAR ones. A window packed with returns off a wall and
%     a floor satisfies it; whether any of them lie on one plane is fitSupport's question,
%     and fitSupport reports code 10 (PlaceUnsupported) when the answer is no.
%   - IT CANNOT REFUSE ANYTHING. At the 4x growth nearRays returns whatever hits it found
%     even if they are fewer than this, because one valid hit is still a metric measurement
%     of where the surface is. The only outcome it can report is code 6 (DepthInvalid), and
%     only for a window that is entirely empty.
%
% So this is a window-SIZING target and the right value is simply the most any downstream
% fit could want, which is planeMinInliers - asking for more only widens the window, and
% the width is bounded. The range-aware count derived from supportMinSpan deliberately is
% NOT used here: that count needs the seed's range, and establishing the seed is what
% nearRays is for. Using it would be circular.
cfg.percep.windowTargetHits  = cfg.percep.planeMinInliers;   % hits that stop the window growing
cfg.percep.planeMaxTilt      = 0.262;   % rad (15 deg), a support surface is near-level
% HOW MANY LOCAL HITS BUY A LOCAL HEIGHT. fitSupport levels the fitted plane onto the
% inliers within the object's own footprint, because the wide window this scene needs for
% planeMinInliers spans 468 mm and the kitchen counter is two surfaces 10 mm apart over
% that span - RANSAC averaged them and put the place point 6.4 mm above the surface the
% object actually rests on. Five, not planeMinInliers: the footprint is a tenth of the
% window's area, so demanding the same count there would mean the correction never
% applies, and five is enough for a median to reject the other level. Below it the
% full-window plane stands. See fitSupport in hmmReconstructTargets.
cfg.percep.localMinInliers   = 5;       % local hits needed to re-level the plane
cfg.percep.surfaceInset      = 0.050;   % m, admissible region inset from the fitted edge
cfg.percep.regionHalfMax     = 0.400;   % m, cap on the reported admissible half-size
% SLIDING AN UNSERVICEABLE PLACE POINT ALONG ITS OWN SURFACE (hmmSlideOnSurface).
% Only when nothing in the observed patch serves. maxSlide bounds the FIRST tier, how far the
% provider's point - and the operator's intent - is moved when anything nearby serves: the
% island-centre answers of 2026-10-12 needed 0.45 m. maxSlideFar bounds the second tier, tried
% only when the first finds nothing: the rest of the SAME surface, because a couch whose middle
% faces a coffee table 0.4 m away is served only from its ends, ~1 m off (2026-10-15).
cfg.percep.placeMaxSlide         = 0.500;   % m
cfg.percep.placeMaxSlideFar      = Inf;     % m, the whole connected surface
cfg.percep.placeSlideHeightTol   = 0.080;   % m, "the same surface": spans a cushion seam; each spot keeps its own height
cfg.percep.placeSlideAnchorTol   = 0.080;   % m, fitted height vs the map's, to anchor on the map
cfg.percep.placeSlideMinBearings = 2;       % stand-off bearings wanted; 1 is used only as a last resort
cfg.percep.placeReachOverTol     = 0.080;   % m above the surface that counts as reaching over furniture
% The volume a reconstructed point has to land inside to be believed. Derived
% from the shell so gate 7 (OutsideRegion) tracks the scene that is actually
% loaded, with a metre of margin for reconstruction error near a wall.
cfg.percep.sceneBounds       = [cfg.scene.extent(1, :) + [-1 1]; ...
                                cfg.scene.extent(2, :) + [-1 1]; ...
                                -0.05 2.60];   % m, [xlo xhi; ylo yhi; zlo zhi]
% ZERO MEANS THE QUESTION WAS NEVER ASKED, which is not the same as any of the three
% answers below it. A navigate action's target is a PLACE ON THE FLOOR, and a place on
% the floor has no extent to measure, source from a registry or assume: the base
% drives to it and nothing ever closes a jaw around it. Publishing EXTENT_ASSUMED
% there would state 60 mm as the size of a patch of floor, and a consumer reading
% GraspExtent has no way to tell that from a 60 mm object nobody could measure.
cfg.percep.EXTENT_NONE       = uint8(0);   % not an object; no extent was sought
cfg.percep.EXTENT_MEASURED   = uint8(1);   % from the calibrated cluster
cfg.percep.EXTENT_ACTOR      = uint8(2);   % from the movable-actor registry
cfg.percep.EXTENT_ASSUMED    = uint8(3);   % cfg.percep.extentAssumed fallback

%% Navigation map
% The map is built at runtime from the occupancy band of the render depth path
% and lives in a Data Store, so no floor plan is configured here - only the
% grid the map is discretised onto.
cfg.map.resolution           = 20;              % cells/m
cfg.map.size                 = [12 12];         % m, [x y] extent
% The grid is placed so it contains the apartment shell with a metre of margin
% on each side. It is DERIVED from cfg.scene.extent rather than typed, because a
% map that does not contain the scene produces plans that leave it.
cfg.map.origin               = [cfg.scene.extent(1, 1) - 1, ...
                                cfg.scene.extent(2, 1) - 1];  % m, cell (1,1)
cfg.map.gridSize             = cfg.map.size * cfg.map.resolution;   % [rows cols]
% The flattened cell count, needed because the range scan selects samples out of
% the grid as a vector. It is a FIELD rather than prod(cfg.map.gridSize) written
% at the point of use: Simulink evaluates a Selector's InputPortWidth with a
% restricted evaluator that accepts variable references, indexing and arithmetic
% but NOT function calls, so prod() there fails to compile with the unhelpful
% "Invalid setting for parameter 'InputPortWidth'".
cfg.map.numCells             = prod(cfg.map.gridSize);
cfg.map.occupiedThreshold    = 0.65;
cfg.map.freeThreshold        = 0.20;
% The grid starts at this probability, and it is NOT zero. occupancyMap updates
% in log-odds, and the log-odds of probability zero is minus infinity: a cell
% initialised to 0 is "certainly free" and no finite amount of evidence can ever
% move it, so obstacles would never appear. 0.10 is below freeThreshold, so an
% unobserved cell is still traversable at startup, but its log-odds is finite:
% ln(0.1/0.9) = -2.20, and each occupied observation at obsOccupied adds
% ln(0.9/0.1) = +2.20, so two agreeing observations cross occupiedThreshold.
cfg.map.initialProbability   = 0.10;
cfg.map.obsOccupied          = 0.90;   % probability reported for a hit in the band
cfg.map.obsFree              = 0.10;   % probability reported for a floor return

% THE DOORWAYS THE NADIR SURVEY CANNOT SEE, AND WHY THEY HAVE TO BE WRITTEN DOWN.
% makeNavMap carves free space wherever probeFloorPlan's straight-down rays reached
% bare floor. That method has one categorical blind spot and it is exactly a
% doorway: above an opening sits the header carrying the wall across it, a nadir ray
% strikes the header, and the header's top is the same 2.70 m ceiling as the wall
% beside it. So an open door is indistinguishable from masonry, in the height map and
% in every derived quantity - which is how these two openings came to be described as
% "full wall height" and why nobody questioned two sealed rooms.
%
% MEASURED, NOT ASSUMED. HORIZONTAL rays were cast at the apartment spine at the
% heights the chassis occupies. At y = -1.50 and y = -5.50 they pass clean through
% from z = 0.05 m to a door head at 2.00 m and stop on the far wall of the room
% beyond; controls at y = -3.50 and y = -7.00 are stopped at x = 5.93 at every height
% tested. The clear widths below are that sweep at z = 0.11 m, and the x span is the
% spine's full thickness plus one cell either side so the carve joins free space on
% both faces instead of leaving a one-cell seam.
%
% [xMin xMax yMin yMax] in metres, one row per opening. Kept as a bare matrix rather
% than a struct array because it goes into the saved map's provenance, and
% hmmLoadNavMap compares provenance numerically: a map carved with different
% doorways than the configuration asks for is then refused, not silently reused.
% THE TRAILING COMMENTS ARE THE PHYSICAL OPENING, THE SPANS ARE THE CARVE, and they are
% deliberately not the same number - read the span when you need a clearance budget. The
% carve is inset roughly one cell inside each jamb so a rounding error cannot open a cell
% that is really masonry, so the y span is 0.05 m narrower than the measured clear width.
% Verified against the shipped data/hmmNavMap.mat: 248 of 248 and 232 of 232 sampled
% points inside these spans read free. Note also that the free GRID run measures 0.05 m
% wider than the carve, because a 0.05 m cell only partly covered by the carve is set
% free whole - which is why the Bedroom appears as 0.70 m here and 0.75 m on the grid.
% DESIGN AGAINST 0.70 m, the smaller of the two.
cfg.map.doorwayNames         = ["Bathroom", "Bedroom"];
cfg.map.doorways             = [5.88 6.07 -1.90 -1.15;    % 0.75 m carved, 0.80 m physical
                                5.88 6.07 -5.85 -5.15];   % 0.70 m carved, 0.75 m physical

% THE FURNITURE THE NADIR SURVEY CANNOT SEE EITHER, AND FOR A WORSE REASON THAN THE
% DOORWAYS. probeFloorPlan casts its rays at cfg.scene.staticMesh, and the props are
% not in that mesh - they are separate actors placed at spawn. So for as long as this
% map has existed, no prop has been in it: two 0.30 x 0.30 x 0.66 m stools standing on
% the kitchen floor and a 0.22 m ball on the living-room floor were free space to the
% base planner. The cost was not theoretical. The shipped place leg's own arrival pose,
% [2.666 -4.915] measured off the 2026-10-02 run, is 0.286 m from Stool_1's centre
% against a 0.150 m half-width plus a 0.290 m inflation radius - the base parked inside
% a stool, and the map said it was clear.
%
% THIS BAND IS THE WHOLE TEST, AND IT NAMES NO PROP. hmmFloorObstacles calls a prop a
% floor obstacle when its box bottom is within this band of z = 0, so nothing in the
% scene supports it, AND its top is above the band, so it is not a rug. Measured over
% all twenty manifest props, the bottoms are -0.0008 m (both stools), 0.0000 m
% (SoccerBall), then nothing until 0.400 m - so there is 0.34 m of clear air above this
% value and the classification is not close to its own threshold for any prop. It is
% deliberately the same 0.06 m probeFloorPlan uses to call a nadir return bare floor:
% the two tests are the same question asked of a mesh and of a manifest, and letting
% them disagree would mean a prop could be "on the floor" to one and not the other.
cfg.map.floorPropBand        = 0.06;   % m, box bottom within this of z = 0 is on the floor

% WHERE THE GRID THE BASE PLANS OVER COMES FROM. 'fixed' loads the shipped
% data/hmmNavMap.mat through hmmLoadNavMap and holds it for the whole run;
% 'live' starts from initialProbability and lets SceneMapping fold the camera
% returns in, which is what the model did before the map was captured.
%
% 'fixed' IS THE DEFAULT BECAUSE 'live' CANNOT BUILD A USABLE MAP AT THIS RAY
% DENSITY - which is a narrower claim than it once made here, and the narrowing
% matters. Swept against 7511 real obstacle cells, the live grid's coverage measured
% 17.7 % at the former [36 64], 41.2 % at 4x, 64.9 % at 16x and 75.1 % at 64x. The
% default is now [72 128], which IS that 4x point, so live coverage stands at 41.2 %.
% That is mostly a SAMPLING limit rather than a geometric one - each ray at [36 64]
% subtended about 1.3 degrees and could not resolve a wall band in the middle distance,
% and halving the pitch halves that - but geometry still accounts for the residual
% quarter at any density, which is why even 64x stops at three quarters.
%
% So 'live' stays non-default for cost, completeness and convention rather than for
% impossibility. Closing even three quarters of the gap is roughly 590000 rays per
% update against a 0.10 s perception period; the fold marks only OCCUPIED cells, so
% unobserved must mean traversable - the convention makeNavMap inverts on purpose;
% and 75 % of the walls is not a map a base can safely plan over. Raising
% cfg.depth.rayGrid is nonetheless a real lever on live coverage.
%
% THE ENVIRONMENT IS ASSUMED NOT TO CHANGE, which is what makes a captured map
% legitimate rather than a shortcut: the shell is Static and the props are placed
% once at spawn. A map captured against that geometry stays true for the run.
% Anything that moves furniture during a run must set 'live' and accept the
% coverage above.
%
% THAT SENTENCE WAS HALF FALSE UNTIL cfg.map.floorPropBand EXISTED, and the half it
% got wrong is the half it was asserting. The props being placed once makes them
% CAPTURABLE; it did not make them captured, because the sweep only ever saw the
% shell mesh. Re-run makeNavMap after moving a floor-standing prop in props.json -
% the saved file records its footprints in context.detail.floorProps so the map can
% be checked against the manifest it was built from, but nothing refuses a stale one.
% That is the same latitude cfg.scene.staticMesh already gets - it is recorded in
% context and NOT compared in provenance - and for the same reason: provenance exists
% to catch a map read in the wrong COORDINATES, which is silent and fatal, not one
% built from slightly older geometry, which a re-run fixes.
cfg.map.mode                 = 'fixed';

%% Planning services
% Fixed capacities, because a plan crosses a bus. Every one is reported on the
% plan bus alongside the realised count so a consumer can see a truncation.
% A REFERENCE SAMPLE COUNT IS A RESOLUTION, NOT A CONTROL RATE. The samples are a
% time-parameterised curve the tracker interpolates, not a table of per-step
% commands, so the cap does not have to reach StopTime/dt - a 12 m traverse would
% need 1500 samples at the control rate and no bus should carry that. It does set
% cost: contopptraj solves one controllable-interval problem PER SAMPLE, and at 512
% that alone was 1.56 s of a 2.36 s base plan (measured). 256 and 128 keep the
% reference finer than the plans are smooth while halving the service latency.
cfg.plan.baseWaypointCap     = 64;      % SE(2) waypoints from the RRT
cfg.plan.baseSampleCap       = 256;     % time-parameterised reference samples
cfg.plan.armWaypointCap      = 32;      % joint-space waypoints from manipulatorRRT
cfg.plan.armSampleCap        = 128;     % time-parameterised reference samples

% Base search. plannerBiRRT on stateSpaceSE2 with validatorOccupancyMap: the
% base is HOLONOMIC, so a Reeds-Shepp/Dubins/Hybrid A* search would impose a
% turning radius the hardware does not have.
cfg.plan.baseMaxIterations   = 4000;
% 0.250 m, AND THE ROUTE TO THAT NUMBER IS WORTH READING BEFORE CHANGING IT, because it
% was 0.750 for a measured reason that stopped being true. The naive argument for a short
% extension is the textbook narrow passage: 0.750 m is longer than the 0.70 m bedroom
% doorway the footprint threads with 0.575 m of width, so whole extensions get thrown away
% at the one place the flat has no slack. Driving plannerBiRRT and plannerRRTStar directly
% agreed loudly - 0.350 m found a route on 99 of 100 trials against 90 for 0.750 m.
%
% THAT TEST STOPS AT THE SEARCH. It never reaches shortenpath, contopptraj or the
% re-validation of all cfg.plan.baseSampleCap samples, which is where most dispatches
% really fail, and measured through hmmPlanBasePath the recommendation inverted: 0.350 m
% gave 23 of 30 against 25, and 17 first-search against 22, because a shorter extension
% needs many more of them to cross 6-8 m inside the same baseMaxIterations budget. Short
% steps buy tight squeezes and cost range. It was applied and reverted.
%
% THEN THE HEADING FIX CHANGED THE TRADE-OFF AND THE REVERT HAD TO BE REVISITED. Once
% cfg.plan.baseTangentHeading squared up the doorway crossings, the reference stopped
% clipping the jamb and short extensions stopped paying for the squeeze they no longer
% needed. Re-measured through hmmPlanBasePath at 12 seeds x 5 routes, WITH the heading fix:
%
%   connD   served of 60   first search   mean    SearchFailed   RefCollide
%   0.750      51              43         2.44 s       6             3
%   0.250      54              46         2.46 s       2             4
%
% Equal in wall time - the 4x speedup a 6-seed run showed was noise - but 0.250 m serves
% three more dispatches, wins three more on the first search, cuts BaseSearchFailed from 6
% to 2, and returns SHORTER PATHS on all five routes (the bathroom leg 7.0 -> 5.6 m), which
% is a shorter real drive and not just a cheaper plan. A 6-seed scan either side of it gave
% 0.350 m -> 24 of 30 and 0.150 m -> 27 of 30, so the optimum is interior: going lower
% stops helping, because eventually the iteration budget binds again.
cfg.plan.baseMaxConnDistance = 0.250;   % m
cfg.plan.baseValidationDist  = 0.050;   % m, validatorOccupancyMap step
% TIGHTENED FROM [0.08 0.08 0.20] ON 2026-10-04, AND IT IS AN ARRIVAL-HEADING FIX, NOT AN
% ACCURACY ONE. The user's report was that the base "starts in forward direction then
% rotates 180deg to the pick location back"; the run bears it out - 3 m nose-first, then a
% +193 deg STATIONARY PIVOT at [3.08 -1.26], then 0.54 m of reverse into a stand-off
% commanded at psi = 120 deg. The pivot is lateReverse working exactly as designed
% (hmmPlanBasePath stage 4b), so the heading it was asked to achieve is the defect.
%
% WHY THE TOLERANCE DECIDES A HEADING. hmmStandoffRing's armCanWork screens reach not at
% the candidate pose but over EVERYTHING this box licenses - 27 corners of
% [+-tol(1), +-tol(2), +-tol(3)]. At the pick the commanded nose-on pose needs a lever of
% only 0.6905 m against a 0.8896 m ceiling, enormous margin; the worst pose the old
% +-0.113 m / +-0.20 rad licence permits needs 0.8993 m and misses by 9.7 mm. So the
% licence, not the arm, forbade nose-on, and the ring fell through to a heading that needs
% the pivot. Swept against the real hmmBaseValidators and the shipped nav map:
%
%   tolerance              pick arrival        place arrival
%   [0.080 0.080 0.20]     psi  120 deg        psi 0 deg      <- reproduces the run
%   [0.050 0.050 0.12]     psi  -30 deg        psi 0 deg      <- no pivot needed
%
% -30 deg is the best the geometry offers: psi = 0 is refused by the FOOTPRINT validator on
% that bearing whatever the tolerance, so tightening further buys nothing (measured
% identical from 0.060 down to 0.020). The place leg is bit-identical under every tolerance
% because it already arrives nose-on.
%
% IT IS AFFORDABLE BECAUSE THE ARRIVAL ERROR IS A CONSEQUENCE OF THIS NUMBER, not a
% capability limit - the follower stops the moment AtGoal latches, so the base stops as
% soon as it is inside whatever box this is. The run that motivated the change settled at
% 0.0238 m / 0.0615 rad and passed within 0.0120 m of its goal, both comfortably inside the
% new box. Line 2512 records the old box's inward slop nearly closing the pick-standoff
% escape window, so this helps there too.
%
% AND IT MUST NOT LAND ALONE. A tighter box means a longer settle, i.e. more time spent
% near-stationary at the goal, which is more exposure to the false stall in
% BaseSupervisor/Following - see cfg.track.stallSpeed. That fix ships with this one.
cfg.plan.baseGoalTolerance   = [0.05 0.05 0.12];  % m,m,rad
% A WALL, FOR THE STAND-OFF SEARCH (hmmWallBetween): any nadir-sweep cell this high on the line from
% a candidate base position to its target means the arm cannot work from there. The shell's walls
% read 2.70 m; the tallest furniture top an arm reaches over is well under 2.5.
cfg.plan.wallHeight          = 2.50;   % m
% SE(2) heading weight, and it is NOT the knob for the crabbing below - this was
% checked, because it is the first thing anyone reaches for. SE(2) distance is
% sqrt(WeightXY*(dx^2+dy^2) + WeightTheta*dtheta^2), which is invariant under adding a
% constant to EVERY heading: the metric sees the heading difference between two states and
% never the bearing from one to the other, so nose-first and dead-sideways are the same
% distance - measured bit-identical at 1.414214 for every weight from 0.05 to 1e4. Swept
% over a 200x range with the tangent rewrite off, the sideways-per-forward ratio goes
% 1.022, 0.985, 0.961, 0.888, 1.017, 0.950: a 0.134 band of seed noise, no trend, against
% 0.274 shipped. Raising it does not make crabbing worse either, which was the guess. What
% it DOES set is how much rotation a path spends between two given headings.
cfg.plan.thetaWeight         = 0.350;   % not the knob for crabbing; see above

% DRIVE FORWARDS ALONG THE ROUTE, AND KEEP SIDEWAYS MOTION FOR THE ENDS OF IT.
% The base is holonomic, and until this constant existed NOTHING in the pipeline
% preferred longitudinal motion: the box contopptraj is given is isotropic
% (cfg.base.vxMax == vyMax, axMax == ayMax), thetaWeight above is 0.350 so the SE(2)
% metric barely charges for rotation and never charges for facing the wrong way, and
% the headings along a route are whatever plannerBiRRT SAMPLED. Over 46 searched routes
% the consequence measured 0.811 sideways metres per forward metre. The base crabbed
% almost as much as it drove.
%
% AND IT WAS ALSO THE DOMINANT FAILURE. Measurement found the bedroom doorway
% being crossed at 14-16 degrees of yaw, and on one seed at -171.8 - backwards. The
% chassis is longer than it is wide, so a SQUARE crossing presents the narrowest
% profile a doorway ever sees and every degree of yaw widens it. Of 10 rejected
% samples, 0 failed the physical footprint check and 8 sat in that doorway band. So
% "prefer forward motion" and "stop clipping the jamb" are one change, not two.
%
% THE OBVIOUS LEVER IS THE WRONG ONE, and this is the trap to record. Shrinking
% cfg.base.vyMax reads as "penalise sideways", but contopptraj is handed
% q = [x; y; theta] in the WORLD frame, so row two bounds world y-dot rather than
% body-frame lateral velocity - it would forbid driving north-south whichever way the
% robot faces. The isotropy above is the only thing making that box frame-agnostic.
% The constraint belongs in theta, which is where hmmPlanBasePath now puts it.
%
% TANGENT FIRST, SAMPLED HEADING AS FALLBACK, because unconditional is worse. The
% search validated the SAMPLED headings, so rotating the chassis where clearance only
% existed in the sampled orientation puts a waypoint in a jamb. At the shipped connD of
% 0.250 m, 12 seeds x 5 routes - note the denominators differ and are NOT interchangeable,
% the pure variants over SEARCHED routes and the shipped row over DISPATCHES:
%
%   heading                        served      sideways/forward   longitudinal   duration
%   sampled (was shipped)          37 of 54        0.762              57%         8.29 s
%   tangent, unconditional         36 of 54        0.153              87%        10.07 s
%   tangent then sampled, SHIPPED  46 of 60        0.275              78%         9.72 s
%
% THE LONGITUDINAL COLUMN SAYS NOTHING ABOUT WHICH WAY THE CHASSIS POINTED, and that is
% the blind spot cfg.plan.reverseWeight below exists to close: reverse metres ARE
% longitudinal metres, so a route driven entirely backwards scores 100 per cent in that
% column. Measured separately, the shipped row drives 29 per cent of its longitudinal
% metres in reverse, down from 47 at reverseWeight = 0, and every remaining reversal is on
% the one route that SHOULD reverse. Coverage is untouched at every weight tried: 46 of 60,
% and the same 12/12/6/6/10 per route.
%
% THE THIRD ROW IS MEASURED THROUGH hmmPlanBasePath ITSELF and the first two are not -
% they re-implement stages 4 to 6 to isolate the heading, which is all they are for.
% The fallback beats BOTH on coverage, and not by averaging them: the tangent rescues
% routes the sampled heading clips while the sampled heading rescues those where
% re-pointing the chassis puts a waypoint in a jamb.
%
% 0.275 AND NOT 0.153, AND THE AVERAGE IS THE WRONG WAY TO READ IT. The 46 served
% dispatches are two populations, not one: about 37 take the tangent and run at ~0.15,
% which is 87 per cent longitudinal (the kitchen leg does 5.17 m forward to 1.04 m
% sideways), while about 9 fall back to the sampled heading and are indistinguishable from
% the old behaviour (bathroom seed 4 does 4.82 m forward to 3.28 m sideways). The
% requirement is met on roughly 80 per cent of drives and not at all on the rest. Closing
% the remainder means making the re-headed waypoints SEARCHABLE rather than checked after
% the fact.
%
% ONE HONEST COST. Drive duration rises about 17 per cent, 8.29 to 9.70 s, because theta
% must now rotate at corners instead of the chassis sliding through them. This re-baselines
% the recorded timeline. The fallback's second parameterisation adds a 256-sample
% validation pass on the ~20 per cent that need it.
%
% AND A FIDELITY NOTE THIS MEASUREMENT SURFACED, recorded because it is not fixed. The
% velocity box above is a SQUARE in world x-dot/y-dot, so the model permits
% hypot(0.68, 0.68) = 0.96 m/s along a world diagonal against 0.68 along an axis. A real
% omnidirectional base is limited at the wheels and its envelope is closer to a disc.
% That 41 per cent diagonal bonus is part of why crabbing looked cheap, and it means the
% 22 per cent duration cost above is partly an artefact of the box's shape rather than of
% this change. Rounding the box would slow every leg and move SafetyMonitor's thresholds,
% so it is a separate decision.
cfg.plan.baseTangentHeading  = true;    % try tangent-aligned heading before sampled

% ONE CHASSIS LENGTH, and it is geometric rather than tuned. Over a blend shorter than
% the robot the heading would have to change faster than the chassis clears its own
% swept area; over a much longer one the "small adjustment" stops being small. Its job
% is that the COMMANDED start and goal headings are still met exactly - a stand-off
% pose has to face the surface it is standing off from, so this is where the sideways
% motion is deliberately allowed to live, and it is why no pre-grasp, capture or
% approach geometry changes when the constant above is turned on.
%
% AND IT IS HOW FAR PAST A TIGHT SPOT THE BASE IS BACK ON ITS ROUTE (2026-10-20). The
% 'tangent-zoned' rung (hmmPlanBasePath/zonedHeading) lets a doorway, a stand-off slot or a
% corner keep the heading the search sampled and ramps back onto the route tangent over this
% length either side, so one tight spot no longer costs a long drive its nose-first heading.
cfg.plan.baseHeadingBlend    = 0.600;   % m of arc length to blend to/from the tangent
% Half-width of the arc-length window the route tangent is averaged over before the chassis
% is pointed along it (hmmPlanBasePath/tangentHeading). Doglegs shorter than ~2x this are
% crabbed through instead of turned into; 0 disables. See the measurement at its use.
cfg.plan.baseTangentWindow   = 0.500;   % m

% KEEP THE BEST DRAW, NOT THE FIRST, and these two constants are the whole of that policy.
% plannerBiRRT stops the instant its two trees connect and returns that connection: it has no
% ContinueAfterGoalReached property to set, because there is no refinement stage to continue
% into. Whatever route it stumbled onto is what drives, and stage 4b then faithfully points
% the chassis along it. On the place leg, measured through hmmPlanBasePath with the query held
% bit-identical, that lottery is wide:
%
%   wasted rotation   0 deg on 5 draws of 10, 34 to 69 deg on the other 5
%   leg duration      5.72 to 6.18 s for the good draws, 10.3 to 10.6 s for the bad ones
%
% A bad draw leaves the pick pose turning the WRONG WAY by half a right angle, undoes it, and
% costs 4.5 s of drive. A recorded 360x640 run did exactly that, tripped the stall detector at
% t = 9.92 s, replanned, and slipped the place arrival from 13.30 to 15.18 s.
%
% WHY NOT A SEED, which is the obvious lever. A fixed seed makes the lottery REPRODUCIBLE, not
% won - it freezes whichever route that seed happens to draw, and one draw in two is bad. It
% would make the defect permanent and deterministic instead of intermittent.
%
% WHY NOT A SMOOTHER EITHER, which is the other obvious lever and was measured rather than
% assumed. optimizePath removes the wasted rotation on every draw and collapses the spread,
% and it costs 55 per cent MORE drive time, because the timed elastic band returns cusped
% paths: 408 deg of yaw sweep on a leg whose net turn is 46 deg is a path that doubles
% back. Curvature throttles contopptraj - the base may go no faster than omegaMax/kappa -
% so the smoothed route is both longer and slower. A cusp is also the reversing motion
% this project has already been asked twice to remove.
%
% SO THE FIX IS TO DRAW AGAIN AND KEEP THE FASTEST, which is cheap because the expensive part
% is already paid: contopptraj computes the leg's duration during stage 5, so scoring a draw
% costs nothing beyond the draw itself.
%
% AND THE SCORE IS DURATION RATHER THAN THE WASTED ROTATION ABOVE, which is a correction, not
% a preference. Wasted rotation names the defect exactly and is scale-free, so it looked like
% the test to accept a draw on and skip the rest. It cannot be: after unwrapping, a path that
% REVERSES is still monotone in yaw, so a cusped draw scores a perfect zero. Measured, an
% early exit on it shipped 2 dispatches of 12 whose legs then took 9.94 and 10.51 s where
% other draws of the same query managed 6.0 s. Duration has no blind spot - it is what a cusp,
% a detour and a wrong-way rotation all cost, in the units the complaint was made in - and
% between draws of one query it needs no threshold at all. hmmPlanBasePath still measures and
% traces the wasted rotation, as the diagnostic that names the defect for a reader.
%
% THREE DRAWS, because the marginal value falls off geometrically and the cost does not. With
% no early exit all three are paid, so the dispatch costs about 3.3 s against 1.1 s for one
% draw, and it buys back roughly 4.5 s of SIMULATED drive on the half of dispatches that would
% have drawn badly. At one draw in two bad, three leave a 12 per cent chance that every draw
% is poor - in which case the least poor one ships, which is what a single draw gave on
% average anyway. Halving that residual again costs three more full draws for the 12 per cent,
% which is the wrong trade; the lever to reach for first is the search itself.
cfg.plan.baseDrawLimit       = 3;        % BiRRT draws per dispatch, fastest kept
cfg.plan.baseSearchRetries   = 2;        % extra searches after a failed one (hmmPlanBasePath)
% m, straight run either side of a doorway waypoint. At least half the base length plus margin plus
% half cfg.plan.baseTangentWindow, so the heading is settled on the door axis before the nose
% reaches the frame: 0.40 left the smoothed heading 15 deg off at x 6.39 on the bedroom door.
cfg.plan.bottleneckRun       = 0.60;

% THE YAW A DRAW MAY SWEEP BEFORE IT IS GOOD ENOUGH TO STOP DRAWING, and this is an
% EARLY ACCEPT, not a filter: a draw over the cap is still eligible to ship if it is the
% fastest one seen. That asymmetry is the whole safety argument. A cap set too tight costs
% draws that were not needed; a cap set too loose ships a draw a later one would have beaten.
% Neither can fail a dispatch, and nothing is rejected for being ugly.
%
% AN EARLIER VERSION OF THIS GUARD ACCEPTED EARLY ON WASTED ROTATION AND THAT WAS WRONG, so
% the bar for doing it again on a different metric is evidence. Over 120 draws of the place leg
% the sweep predicts the duration well: the 62 draws under 220 deg time-parameterise to
% 5.07-8.22 s, the 58 over it to 7.70-13.08 s. It is a good predictor, NOT a perfect one - those
% ranges overlap between 7.70 and 8.22 s, and a first reading over only 36 draws that showed no
% overlap at all was small-sample luck. The cap is set the way it is BECAUSE the test is
% imperfect, which is the next paragraph.
%
% Wasted rotation cannot do this job at any threshold. Among the draws this cap condemns are
% ones that score a wasted rotation of exactly ZERO while taking 11 s, because they REVERSE, and
% after unwrapping a reversal is still monotone in yaw. Sweep sees a cusp; wasted rotation is
% blind to it. That is why the accept test is this and not that.
%
% 180 SITS IN A FLAT REGION AND THAT IS THE WHOLE REASON FOR THE NUMBER. Replayed over those
% 120 draws, every cap from 160 to 250 deg produces the SAME mean drive of 6.27 s, the same
% 10.38 s worst case and the same three slow legs in forty; only the draws paid move, 66 at
% 180 against 63 at 250. Quality starts to go at 280 (6.37 s, four slow legs) and is worse at
% 300 (6.48 s, five). A cap chosen at the edge of the good population would be buying single
% draws at the price of admitting cusps. 180 is not tuned; it is the middle of the plateau.
%
% IT IS NOT SCALE-FREE ACROSS LEGS and must not be read as if it were. The place leg needs
% about 46 deg of NET heading change and its honest draws still sweep 123-212, because the
% chassis swings onto the route tangent and off it again. A longer leg legitimately sweeps
% more, fails this cap on every draw, and quietly falls back to fastest-of-three - correct
% behaviour, at the old cost. The scale-free version of this test is the largest single yaw
% STEP, which measured 2.4-6.5 deg on the good draws against 17.7-27.1 on the bad and does
% not grow with leg length; hmmPlanBasePath measures it and traces it. Swapping the accept
% onto it is left open and wants a census over every leg first, not one leg's draws.
cfg.plan.baseSweepCap        = deg2rad(180);  % stop drawing once a draw sweeps under this

% WHAT ONE METRE OF DRIVING BACKWARDS IS WORTH IN RADIANS OF TURNING, and it exists
% because the constant above fixed the headings ALONG a route and left the question of
% which way round to drive it decided by a comparison with no distance in it.
% tangentHeading picks nose-first or tail-first for a whole route on
%
%   costF = |angdiff(thStart, tang(1))|      + |angdiff(thGoal, tang(N))|
%   costR = |angdiff(thStart, tang(1) + pi)| + |angdiff(thGoal, tang(N) + pi)|
%
% Both are radians AT THE TWO ENDPOINTS. Neither contains a metre, so reversing was
% free however far it went, and the recorded reference run drove its place leg
% 1.838 m backwards against 0.076 m forwards - 96 per cent of its longitudinal travel.
% The driven path identifies the stage: |heading - tangent| over that leg has median
% 159.1 degrees with 80 per cent of samples past 120, which is a GLOBAL OFFSET rather
% than a random heading, and the pick leg reads 20.9 degrees on the same measure.
%
% AND THE FLIP IS NOT SIMPLY A BUG TO REMOVE. Its own comment is right: forcing
% nose-first would sometimes mean a 180 degree spin the task never asked for. So the
% cost gains an arc-length term, costR + reverseWeight * arcLength, and the units are
% what make the constant readable - a reversal must SAVE this many radians of endpoint
% turning per metre it spends going backwards.
%
% THE WEIGHT SEPARATES TWO POPULATIONS RATHER THAN SPLITTING ONE, which is why it is a
% measured number and not the midpoint of anything. Solving each route for its break-even
% weight w* = (costF - costR)/arcLength over 5 routes x 12 seeds gives:
%
%   route                              reverses  w* rad/m      what reversing buys
%   pick stand-off -> kitchen place     5 of 12  0.059-0.218   0.29-1.30 rad over ~5 m
%   start -> living room stand-off      6 of 12  0.088-0.666   0.26-1.65 rad over ~2.7 m
%   kitchen place -> bedroom            2 of 10  0.203-0.253   ~1 rad over ~4.6 m
%   bedroom -> kitchen                 10 of 10  0.946-1.134   5.1-6.0 rad over ~5.7 m
%   start -> bathroom                   0 of 12  -             never reverses
%
% The bedroom-to-kitchen route asks to reverse because BOTH commanded end headings point
% back the way it came: nose-first costs 6.1 rad there, tail-first 0.15, so suppressing it
% would buy nearly two half-turns to save nothing. That is the case the flip is for and
% every weight below 0.9 keeps it. Everything else on that table is a short-and-steep case
% that a weight in the 0.5-0.8 band removes.
%
% AND WHAT IT DID, measured through hmmPlanBasePath at three weights, same 12 seeds x 5
% routes each. Reversed dispatches, out of those served:
%
%   route                              w = 0     w = 0.5   w = 0.75
%   start -> living room stand-off     6 of 12   2 of 12   0 of 12
%   pick stand-off -> kitchen place    5 of 12   0 of 12   0 of 12
%   kitchen place -> bedroom           2 of 6    0 of 6    0 of 6
%   bedroom -> kitchen                 6 of 6    6 of 6    6 of 6   <-- kept, and it should be
%   start -> bathroom                  0 of 10   0 of 10   0 of 10
%
% At 0.750 the ONLY reversals left in the whole sweep are the six that ought to be there.
% Reverse share of all longitudinal metres 47 -> 30 -> 29 per cent; that last step is small
% because the bedroom route owns most of the reverse metres, not because nothing changed.
% COVERAGE DID NOT MOVE AT ANY OF THE THREE: 46 of 60, and the same 12/12/6/6/10 per route,
% which is the point of stage 4b offering both signs and then the sampled heading rather
% than replacing one candidate with another.
%
% WHY 0.750 AND NOT THE 0.500 THIS SHIPPED AS FIRST, because the difference is the whole
% reason the constant exists. 0.500 suppressed every reversal on the STYLISED route that
% stands in for the reference run's place leg and left the real leg driving 96 per cent
% backwards anyway. cfg.plan.baseHeadingTrace is what settled why: the real leg's w* is
% 0.559, above 0.500, so the reversed sign was genuinely PREFERRED and shipped - no
% fallback, no stage 6 rejection. The stylised route is 5 m long between headings of -75
% and 150 degrees; the real one is 2.29 m between 163.5 and 212.1, and a shorter arc makes
% every rad/m more expensive. THE LESSON IS ABOUT THE PROXY, NOT THE WEIGHT: a constant
% tuned on a proxy route needs confirming on the route it was meant to fix, and until the
% trace existed there was no way to do that.
%
% Note also that reconstructing w* from the DRIVEN path reads low - 0.225 and 0.372 on two
% runs whose true value the trace put at 0.559 - because it takes its endpoint tangents from
% a trajectory that begins and ends at rest. Use the driven path to see WHICH way a leg
% drove; use the trace to decide a weight.
%
% ZERO RESTORES THE PREVIOUS BEHAVIOUR EXACTLY, and that is measured rather than argued:
% at 0.000 the same measurement reproduces the recorded 46 of 60, 0.269 sideways per
% forward, 9.72 s, and 0.147 for the unconditional tangent - the same figures to the digit
% as before this constant existed. That is what makes it safe to tune.
cfg.plan.reverseWeight       = 0.750;   % rad of endpoint turning saved per reversed m
% How much cheaper the stand-off's PARTNER heading must drive before hmmPlanBasePath takes it
% over the heading hmmStandoffRing chose (cost = rad of yaw swept + reverseWeight * reversed m,
% on the timed reference). Large enough that a near-tie never flips the heading; small against
% the differences measured offline on 2026-10-08: Run 77 carry 9.28 -> 6.42 (taken), Run 97
% legs 1.54 vs 2.88 and 0.79 vs 2.91 (kept).
cfg.plan.partnerMargin       = 0.50;    % rad
% Radians of yaw a reversed metre is worth WHEN COMPARING A STAND-OFF WITH ITS PARTNER. Separate
% from cfg.plan.reverseWeight, which decides the sign of a single route's tangent and was tuned
% for that job; the partner comparison expresses the user's standing preference - forward motion
% wherever it is available, rotating where there is room, reversing only when nothing else fits.
% 1.5 rather than reverseWeight's 0.75, measured on Run 103's carry leg (8 replays): at 0.75 a draw
% of 382 deg / 2.35 m reversed tied a partner of 444 deg / 0.42 m (8.43 vs 8.07) and the base backed
% 2.4 m into the counter - the motion the user asked to avoid. At 1.5 one reversed metre is worth
% ~86 deg of turning; that draw goes to the partner (10.2 vs 8.4), while a genuinely forward goal
% route (245 deg, 0 m) still beats the partner, and Run 97's legs (0 m reversed on the goal side)
% are unaffected.
cfg.plan.partnerReverseWeight = 1.50;   % rad per reversed m
% Re-time each base reference so its BODY-frame speed and acceleration stay inside
% cfg.base.planMargin of the base box (hmmPlanBasePath/bodyFrameRetime). contopptraj bounds the
% world axes separately, so diagonals were planned at sqrt(2) the per-axis limit and the
% follower, which clamps the body twist, fell behind - Run 100 bailed on cross-track 0.151 m.
cfg.plan.baseBodyRetime      = true;
cfg.plan.baseRetimeWindow    = 0.03;    % half-width of the factor's smoothing, fraction of samples

% WHICH HEADING CANDIDATE ACTUALLY SHIPPED, printed once per dispatch. OFF, and it must
% ship off: hmmPlanBasePath is extrinsic, so this writes to the command window from inside
% a running simulation.
%
% IT EXISTS BECAUSE THE ANSWER IS OTHERWISE ONLY INFERRABLE, and the inference has been
% wrong before. bplan carries the waypoints but not the parameterisation that produced
% them, so the inference reads the driven heading against the driven path and
% reconstructs costF and costR from the endpoints - and on the reference run's place leg
% that reconstruction says nose-first should have won at this weight while the leg drove
% 96 per cent backwards. Exactly two things can produce that, the trace separates them in
% one word, and neither can be read off a recorded run without it:
%
%   trace says tangent-preferred  the preferred sign IS the reversed one, so the
%                                 reconstruction is what is wrong - it takes its endpoint
%                                 tangents from a trajectory that starts and ends at rest
%   trace says tangent-flipped    the preferred sign was nose-first and stage 6 rejected
%                                 its reference, so the fallback drove and the weight is
%                                 working as designed on a route where it cannot win
%
% HOW TO TURN IT ON, and why through the environment rather than by editing this line.
% hmmPlanBasePath reads its configuration through a PERSISTENT cache, so a field set on a
% cfg struct in a caller's workspace never reaches it - the only two ways in are an edit
% here plus clear functions, or a variable the function can read for itself. An edit to a
% shipped configuration file that has to be reverted afterwards is the worse of the two:
%
%   setenv('HMM_HEADING_TRACE', '1'); clear functions; sim('homeMobileManipulator'); ...
%   setenv('HMM_HEADING_TRACE', '');  clear functions
%
% THE PRECEDENT HERE IS A CAUTIONARY ONE and it is why this is print-only. GEMINI_PROVIDER
% outranks cfg.provider.mode, the app sets it and never puts it back, and a gate that
% inherited it reported one provider route while taking another. That failure mode needs
% the variable to change BEHAVIOUR. This one gates an fprintf and nothing else: a stale
% HMM_HEADING_TRACE costs two lines of command-window output, cannot change a plan, and
% cannot make a run report something it did not do.
cfg.plan.baseHeadingTrace    = strcmp(getenv('HMM_HEADING_TRACE'), '1');

cfg.plan.baseRadius          = 0.5*hypot(cfg.base.chassisSize(1), cfg.base.chassisSize(2));
% Stowed arm reach BEYOND the chassis circle, and the one number that decides
% both how far the planner inflates the map and which depth returns are the
% robot's own body. It is MEASURED, not chosen: measureStowedEnvelope sweeps
% cfg.arm.stowConfig through the whole gripper aperture range and reports the furthest
% hull-reduced VISUAL MESH point inside cfg.depth.occupancyBand. At the shipped stow pose
% that is 0.3014 m from the base origin, and the body that sets it is wheel_fr rather than
% anything on the arm - INSIDE the 0.3183 m chassis circle, so max(0, radius - baseRadius)
% is exactly ZERO and the number below is pure allowance rather than a measurement.
% hmmSetup re-measures on every rebuild and warns if this no longer covers the tree.
%
% AND IT DID NOT COVER THE TREE BEFORE THIS LAYOUT, WHICH IS WORTH RECORDING RATHER THAN
% QUIETLY FIXING. The previous stow pose reached 0.4136 m once the measurement moved from
% frame origins to meshes, against a stated baseRadius + stowedArmRadius of 0.3683 m: the
% robot was 45 mm larger than the figure hmmOccupancyFromDepth's self-return mask and
% hmmBaseInflation both read, and hmmSetup's own warning would have said so. The stowed arm
% coming back inside the chassis circle is what closes that gap - it is not a reason to trim
% the allowance, which is the slack that made the miss survivable.
%
% BOTH DIRECTIONS FAIL, WHICH IS WHY IT IS BOUNDED ON BOTH SIDES. Understating it
% is silent: the fixed cameras map the robot's own gripper, and after inflation the
% base cannot prove its own pose valid, so every base plan fails at the start-state
% check. Overstating it is just as bad and was the shipped defect - at 0.250 m the
% inflation came to 0.628 m, larger than the stand-off the arm has to work from, so
% no goal beside anything the map marks was ever valid. The assertion below ties it
% to cfg.sup.baseStandoff so neither can move without the other being checked.
cfg.plan.stowedArmRadius     = 0.050;   % m, stowed arm beyond the chassis footprint

% THE SAME MEASUREMENT, PER AXIS, WHICH IS WHAT THE BASE PLANNER NOW USES.
% The radius above is honest and it is also the wrong shape to plan a doorway with.
% measureStowedEnvelope reports max(hypot(x, y)) over the stowed geometry - an
% omnidirectional maximum, so it records no direction, and 0.3014 m read against the
% chassis' 0.3183 m circumscribed circle looks like it adds nothing. Against the
% rectangle the chassis actually is, 0.3014 m is well outside its 0.1755 m half-width,
% and whether that matters depends entirely on WHICH WAY the arm sticks out.
%
% measureStowedEnvelope's second output answers that, from the same tree and the same
% 21-step gripper sweep: the whole stowed robot measures 0.2616 m on |x| and 0.1823 m on
% |y|, and the bodies that set them are chassis_link and upper_arm_link. So fore-aft the
% stowed arm is INSIDE the chassis and costs nothing at all, and sideways it makes this
% robot 6.8 mm wider than its own collision box. The two numbers below are |x| and |y|
% rather than a radius because that asymmetry is the whole fact.
%
% THE PAIR IS THE WHOLE ROBOT AND NOT JUST THE ARM, DESPITE THE NAME. It is the maximum over
% every body carrying visual geometry, chassis and wheels included, which is what makes it
% safe to hand straight to hmmBaseFootprint - that function maxes it against half the chassis
% box, and the box understates the measured wheel half-track 0.1799 m by 4.4 mm, so a pair
% measured on the arm alone would have hidden the wheels behind a smaller number.
%
% AND THE y TERM IS WHAT THE WHOLE LAYOUT WAS CHOSEN FOR. Under the previous mount and fold
% the same measurement read [0.3155 0.3474] - 0.3474 m on the axis a doorway measures, which
% took hmmBaseFootprint to 0.4531 m against the measured 0.300 m ceiling and cost the
% Bedroom and the Bathroom on every leg that carried anything. Lowering the mount, folding
% the arm vertically and offsetting it 27.5 mm towards its own bulge brings that to 0.1823 m.
% See cfg.pedestal.armMountHeight and cfg.arm.stowConfig for the measurements.
%
% WHY IT IS WORTH THE TROUBLE. hmmBaseValidators builds the footprint rectangle from
% max(half chassis, this) per axis, covers it with baseFootprintCircles circles and
% rotates the chain with the state's heading. The binding number for passage becomes
% the half-width plus cfg.safety.plannerMargin - 0.280 m - instead of the circumscribed
% radius plus the same margin, 0.468 m. The largest disc measured to reach all four rooms
% through the carved doorways is 0.300 m, so the disc model was above the ceiling and
% the footprint model is 20 mm under it. That difference is the Bedroom and the Bathroom.
%
% MESHES, NOT FRAME ORIGINS, AND THIS NOTE USED TO SAY THE OPPOSITE AND RELY ON IT. It read
% "FRAME ORIGINS, NOT MESHES ... cfg.safety.plannerMargin covers it, and at 0.100 m it covers
% it by an order of magnitude more than the 8.8 mm at issue here". The premise was true of
% Collisions, which a rigidBodyTree does report as a description string, and false of the
% VISUAL geometry, which getVisual returns as a triangulation - so the gap was measurable all
% along, and it was 167.5 mm rather than 8.8 mm: the previous pose's body origins stayed
% inside |y| <= 0.1799 m while its meshes reached 0.3474 m. A 0.100 m margin does not cover
% 167.5 mm, and the reassurance was the reason nobody looked. The pair below is measured over
% hull-reduced visual meshes at 21 apertures, so there is no gap left for the margin to cover.
cfg.plan.stowedArmHalfExtent = [0.2616 0.1823];  % m, |x| and |y| the stowed robot reaches

% AND THE SAME IDEA FOR THE THING IN THE JAWS, WHICH IS THE TERM THAT WAS MISSING.
% The pair above models the arm WHERE IT ACTUALLY IS while the base drives, rather than as
% a shell around every pose it can reach, and that is the only reason the footprint fits
% through a doorway. A grasped object had no such term. hmmBaseFootprint simply added its
% half-extent to all four faces, because that is what hmmBaseInflation does to a RADIUS and
% for a radius it is the only sound choice - a radius has no directions. A rectangle has
% directions, and the object is at a known place in it.
%
% WHAT THE MISSING TERM COST WAS THE WHOLE TASK. The shipped mug's half-extent is 0.0505 m,
% so holding it took the inflated covering radius from 0.2876 to 0.3389 m. Two consequences,
% both measured, neither visible in any check that existed:
%
%   - THE ROBOT COULD NOT LEAVE THE PICK STAND-OFF. cfg.plan.baseGoalTolerance lets the
%     base stop 0.113 m short of a goal, so from a commanded 0.75 m it may park at 0.637 m,
%     and the held footprint needs about 0.702 m. The window is NEGATIVE by 65 mm. The
%     instant the gripper closed, hmmPlanBasePath's start check failed; escapeMargin will
%     not offer an escape from a pose it deems in contact; the place leg was refused with
%     BaseStartOccupied before the base moved, and the recovery re-dispatched the pick and
%     dropped the mug.
%   - THE DOORWAYS CLOSED WHILE CARRYING. The Bedroom was measured to become
%     unreachable at 0.320 m of inflation. 0.2876 m passes with 32 mm to spare and 0.3389 m
%     does not, so the robot could enter a room and then not carry anything out of it -
%     undoing the exact thing the rectangle model was introduced to buy.
%
% IT WAS A LATENT DEFECT WITH A 57 mm HISTORY, NOT A CONSTANT THAT WAS ALWAYS WRONG, and
% that is worth recording because it says what to check rather than what to change. The
% 15-Sep reference run parked at radius 0.7341 m and was valid there. A later run parked at
% 0.6770 m and was not. Nothing touched a stand-off: cfg.plan.baseMaxConnDistance went to
% 0.250 and the tangent-heading rewrite reshaped the trajectory near the goal, and the
% arrival slid 57 mm inward through a margin nothing was guarding. A bound that only holds
% for the pose the base was TOLD to reach is not a bound on anything the base does, which
% is why the stand-off has to hold over the whole arrival box, and hmmSetup checks it.
%
% WHERE THE NUMBERS COME FROM, AND WHY THEY ARE AN UPPER BOUND RATHER THAN A MEASUREMENT.
% The tool frame sits at [0.1920 0.1058] from the base origin at cfg.arm.stowConfig - the
% assembled tree's own forward kinematics, which already carries cfg.pedestal.offset. A grasped
% object's centre now hangs BELOW the tool - TargetSelect aims at hmmGraspAim's clamped
% mid-height and then corrects for the pad drop, which puts the tool 7.6 mm above the centre
% for the shipped mug and at most 14.1 mm above it anywhere in the jaw range -
% so cfg.plan.graspDepth is a real bound on that displacement and is carried here against both
% horizontal axes, whichever way the wrist is turned: [0.2370 0.1508]. The pair below is that
% bound rounded up to the nearest 5 mm.
%
% THIS PAIR MOVES WHENEVER cfg.arm.stowConfig OR cfg.pedestal.offset DOES, and the
% reduced-footprint layout moved both. The history is [0.220 0.130] under the tilted pose,
% [0.290 0.110] once the pose went vertical and joint 1 was rotated to keep the y term small,
% and [0.240 0.155] now. BOTH TERMS MOVED THE OPPOSITE WAY TO THE OBVIOUS PREDICTION, and the
% reason is that the mount is no longer on the centreline: reflecting it 60 mm aft pulls the
% tool's |x| in to 0.1920 m, while offsetting it 27.5 mm sideways towards the arm pushes the
% tool's |y| out to 0.1058 m even though the arm as a whole got narrower. Neither is a problem,
% and the y one is worth checking rather than assuming - 0.1508 m is UNDER the 0.1823 m the
% stowed arm's own upper_arm_link needs, so the held object never sets hmmBaseFootprint's
% half-width, and even the widest graspable prop's 0.060 m does not reach it.
%
% AND THE x TERM SHRINKING IS WORTH 1.4 mm OF DRIVING INFLATION. carryOffset(1) used to be the
% binding term in hmmBaseFootprint's half-LENGTH at 0.290 m, above the chassis' own 0.2655 m.
% At 0.240 m the chassis binds instead, which is what takes the loaded inflation from 0.2913 m
% to 0.2899 m and the nose-on clearance demand from 0.5233 m to 0.5023 m.
%
% THE ASSUMPTION IT CARRIES, STATED SO IT CAN BE CHECKED: the base only drives while the arm
% is at the stow pose. That is what the chart does - RETREAT and STOW both complete before
% NAV_TO_PLACE is entered - and it is measured, not assumed: over the entire grasped-and-
% moving window of the reference run, 10.12 to 17.64 s, no joint is more than 0.0376 rad
% from cfg.arm.stowConfig. It is also the assumption stowedArmHalfExtent above already
% makes, so this adds no new one. If a future chart ever drives with the arm extended, BOTH
% constants become wrong together and the footprint must be computed from the live
% configuration instead.
%
% hmmSetup re-derives the bound from the assembled tree and warns if this pair is under it.
cfg.plan.carryOffset         = [0.240 0.155];    % m, |x| and |y| bounding a held object's centre

% FIVE CIRCLES, AND THE COUNT BARELY MATTERS, WHICH IS THE USEFUL THING ABOUT IT.
% A chain of N equal circles spaced along the length covers the rectangle when each
% reaches the corner of its own slice, so the radius is hypot(length/(2N), width/2). For
% this footprint that is 0.197 m at N = 3, 0.188 m at N = 5 and 0.184 m at N = 7 - the
% sequence has already converged on the half-width 0.180 m by five, and every value from
% three up passes both doorways. Five is chosen because it is where the curve flattens:
% fewer circles buy nothing back and more cost a validator call per circle on every one
% of the hundreds of states a search tests.
cfg.plan.baseFootprintCircles = 5;

% Arm planning. Candidate count is 3 because the Inverse Kinematics block is a
% MATLAB System block and CANNOT be placed inside a For Each Subsystem in this
% release, so candidates are instantiated explicitly and stay visible.
cfg.plan.nGraspCandidates    = 3;
cfg.plan.candidateYaw        = [0, pi/6, -pi/6];   % rad, approach yaw offsets
% Approach: a rotation of pi about the tool Y axis turns the tool +Z to point
% DOWN in the arm base frame, which is the top-down approach a table pick needs.
% The candidate yaw offsets then spin the jaws about that downward axis.

% THE TIE BAND IS WHAT STOPS THE WRIST FLIPPING BETWEEN NEAR-EQUAL CANDIDATES. The candidates
% above are ONE approach at three wrist yaws, so on an object with no preferred grasp axis - a
% mug, a glass, a tin - the +pi/6 and -pi/6 poses are near mirror images and their conditioning
% is near identical. Selecting on conditioning alone then makes the winner a coin toss decided
% by the candidate solvers' random restarts, and the wrist refinement re-plans the SAME target
% from PlanPick a second time, so the coin gets tossed twice.
%
% MEASURED, IT LANDED THE OTHER WAY UP. Between the two free-move legs of one pick the target
% was byte-identical - the refinement had declined and reported a shift of 0.0000 - and joint 6
% moved 1.0552 rad while the tool travelled 4.2 mm. The candidate yaw gap is pi/3 = 1.047198 rad
% and the tool's own-axis rotation was -1.0562 rad: the two solves won with OPPOSITE candidates.
% A visible 60 degree wrist rotation, for no change in the goal.
%
% SO CONDITIONING CHOOSES THE FIELD AND CONTINUITY CHOOSES THE WINNER. Any candidate within this
% fraction of the best conditioning is admissible, and among those the one needing the least
% joint travel from where the arm actually IS wins. 0.80 gives up at most a fifth of the
% conditioning to hold the wrist still, which is affordable because the absolute floor is what
% actually guards the pose: the shipped pick measures 0.0091 against cfg.plan.manipulabilityMin
% of 0.002, a factor of 4.5 of headroom, and every candidate in the band has already passed that
% floor, the residual gate and the collision check. It is a TIE-BREAK, not a re-ranking - at 1.0
% it reduces to the old argmax, and no value of it can admit a candidate the gates rejected.
cfg.plan.candidateTieBand    = 0.80;    % x the best surviving manipulability
cfg.plan.approachPitch       = pi;      % rad, about the tool Y axis
% graspDepth NO LONGER SETS THE GRASP HEIGHT, and the reason it used to is the reason it
% stopped. It was written for a contract in which ObjectTarget.Position was the object's TOP
% FACE, where closing the pads on the reported point would have gripped the topmost millimetre,
% so the tool was lowered 20 mm below it to straddle real material. Position is now the SUPPORT
% SURFACE - which is what the ceiling rig actually resolves, its ray cells being wider than any
% movable object here - so the pick is built in MotionPlanning/TargetSelect off that surface
% instead. There is no face left to hang an offset below: subtracting 20 mm from a tabletop put
% the commanded grasp at 0.530 m for a mug whose centre is 0.595 m, 20 mm INSIDE the table, and
% the arm tracked it faithfully. See the Position contract in hmmReconstructTargets.
%
% IT SURVIVES AS A CARRY BOUND, AND IT IS A REAL BOUND AGAIN RATHER THAN SLACK. hmmSetup and
% hmmBaseFootprint use it to bound where a grasped object's centre can be relative to the tool -
% within graspDepth on each axis, whichever way the wrist is turned.
%
% THE HISTORY MATTERS BECAUSE THIS NUMBER HAS BEEN WRONG IN BOTH DIRECTIONS. Under the ORIGINAL
% top-face contract the centre really sat Extent(3)/2 - graspDepth = 25 mm off for a mug, and
% the 20 mm bound was quietly too small. Then the grasp moved to Position(3) + Extent(3)/2, the
% object's own mid-height, the tool held the object AT its centre, the nominal offset became
% zero, and 20 mm was pure slack - correct by accident. Then the aim rose to a fixed 0.65 of
% the height, the tool sat above the object's centre by (0.65 - 1/2)*Extent(3) +
% padDrop(GraspExtent) - 21.1 mm for the shipped mug and 39.9 mm for the tallest graspable prop -
% and 20 mm was understated, silently, as a side effect of a change made for a different reason
% entirely, which is exactly the failure mode the comment above warned about arriving from the
% other direction. It is now hmmGraspAim's CLAMPED MID-HEIGHT, so the offset is rise -
% Extent(3)/2 and the fraction term is gone from it entirely.
%
% SO IT IS SIZED FROM THE SCENE RATHER THAN CHOSEN, AND THE WORST CASE HAS CHANGED SIDES.
% One number serves every prop, so the bound has to cover the worst GRASPABLE one in the
% manifest, not the one the shipped instruction happens to name. And "graspable" is not a
% judgement here, it is a gate: hmmReconstructTargets sets GraspExtent = Extent(2) and refuses
% the action with cfg.fail.NotGraspable when that exceeds maxOpenPos - apertureTol = 0.0825 m.
% Of the twenty props with extents, six survive it, and they are all tall and narrow because
% that is what a 85 mm jaw stroke selects for. Offset = hmmGraspAim's rise - Extent(3)/2, which
% for any object taller than 2*(padFaceLength/2 + padGroundClear) = 57.5 mm is the PAD DROP
% ALONE, in mm:
%
%   Bottle        11.66   h 188.2   SoapDispenser  9.36   h 137.5
%   Glass_Kitchen  9.92   h 120.1   Glass_Bathrm  11.66   h 100.1
%   Mug / Mug_Red  7.62   h  90.0
%
% The term no longer grows with height, because the aim is mid-height and the only residual is
% the pad drop, which is a function of the JAW GAP. Every one of the six clears 57.5 mm, so the
% bound is set by the widest-dropping gap rather than by the tallest prop: 11.66 mm, and the
% ordering above is now essentially arbitrary. The worst case over the whole jaw range is the
% pad-drop table's own maximum, 14.11 mm at a 19.3 mm gap; below 57.5 mm the floor lifts the aim
% off the mid-height and adds (padFaceLength/2 + padGroundClear) - h/2, at most 28.75 mm for a
% vanishing h, so 0.045 m still covers any graspable object this project can be handed.
%
% 0.045 m IS THEREFORE GENEROUS NOW AND IS LEFT ALONE DELIBERATELY. It was sized to cover the
% old 39.89 mm with 5.1 mm to spare; against 11.66 mm it has 33 mm. Shrinking it would narrow
% the inflated footprint and is a real improvement, but it is a NAVIGATION change - the doorway
% component labelling below and the measured stand-off and doorway clearances all
% rest on this value - so it belongs with that work and not with a grasp-height change.
%
% THE ROUNDING IS UP BECAUSE THE TWO ERRORS ARE NOT SYMMETRIC. Understating it is silent and
% lets the base drive through what it is carrying. Overstating it can only widen a collision
% footprint, which refuses plans rather than permitting collisions.
%
% AND WHAT IT COSTS AT THE DOORWAYS IS ESSENTIALLY NOTHING, WHICH IS WORTH CHECKING AND NOT
% ASSUMING. carryOffset + carried has to beat the chassis half-extent [0.2655 0.1755] before a
% held object widens the rectangle at all, and the mug's carried half-extent is 0.0505 m, so
% this knob only reaches the footprint once carryOffset passes about [0.215 0.125]. At 0.045
% it just does, in x: the driving inflation for the mug goes 0.2876 -> 0.2884 m, eight tenths
% of a millimetre, against the 0.300 m hmmBaseFootprint's header quotes as the all-four-rooms
% ceiling. The Bottle, being narrower, costs nothing at all. Inflating the shipped nav map and
% labelling the connected components of its free space, both carved doorways stay in the start
% pose's component at 0.2876, at 0.2884 and at 0.300, and the Bedroom drops out at 0.310 - so
% there is about 12 mm of real headroom here, not a hair. Anyone raising graspDepth much
% further should re-run that labelling rather than read this paragraph: past roughly 0.060 the
% footprint starts growing in earnest.
cfg.plan.graspDepth          = 0.045;   % m, carry bound; see the sizing note above
% releaseClearance still means exactly what it says, and is still the only thing standing
% between the carried object and the surface. The lift that delivers it tracks the pick:
% TargetSelect raises the tool releaseClearance + hmmGraspAim's rise above the place point -
% the SAME rise the pick used, which is the whole reason the place stayed correct when the pick
% moved, twice.
%
% THE CANCELLATION IS WORTH STATING, because it is the one thing that makes raising the grasp
% safe. The tool holds the object however far above its centre the pick put it. Re-applying
% that same distance at release puts the object's centre back where it belongs - at the place
% point plus releaseClearance plus half the object - so the object's BOTTOM ends up exactly
% releaseClearance clear, independently of the aim rule and of the pad drop. Had the two
% branches been given separate offsets, raising the pick by 21 mm would have set the object
% down 21 mm LOW and driven it through the surface. One shared expression, in
% MotionPlanning/TargetSelect's PadCompensate, is what forecloses that - and it is what made
% the move from a fixed 0.65 fraction to hmmGraspAim's clamped mid-height a change to ONE
% chain rather than a change to the pick that the place had to be re-derived against.
%
% THE COST IS REACH, AND IT HAS JUST COME DOWN. The tool finishes 0.0556 m above the place
% point for a mug, where the fixed 0.65 fraction gave 0.081 m, a naive mid-height contract
% that ignored the pad drop gave 0.060 m, and the original Extent(3) + releaseClearance -
% graspDepth gave 0.085 m. The place leg is the tightest reach constraint in the task, so
% cfg.plan.placeRadiusMax was re-confirmed against this height rather than assumed.
% WHY 3 mm AND NOT THE ORIGINAL 15, and why this is not the whole of the float. There is no
% physics on the props: the object freezes where the release leaves it, so every millimetre of
% this clearance is a millimetre of visible mid-air hover rather than something gravity closes.
% 3 mm is the smallest value the place goal's own collision gate accepts with certainty, and it
% is that value for a reason rather than by trial: hmmScreenArmPlan's surface slab is topped at
% zSurface - cfg.gripper.padCompression, so a commanded underside at zSurface + padCompression
% clears the slab by exactly padCompression. Below that the goal starts asking the gate to
% accept a touch, and a refused plan is worse than a visible one.
%
% MEASURED, the float was 18.0 mm and only 15 of those were this constant. On the verified run
% the mug's underside left the gripper at 0.9180 against a true surface of 0.9000: 15.0 mm of
% releaseClearance, plus 3.2 mm because the gripper opens the instant ArmStatus reports DONE
% and DONE is ArmControl/TrackingMetrics/SettledCheck, TrackingErrorNorm <= cfg.plan.poseErrorMax
% - a 5 mm band the descent satisfies while still moving at 27 mm/s, reaching the commanded
% height 0.30 s later - minus 0.3 mm because the local height estimate published 0.8997.
% Those 3.2 mm are NOT this constant's to absorb. Sizing releaseClearance down to cancel a
% settling lag would silently re-couple the two the moment the band, the descent rate or the
% plant time constant changed. They need their own band on SettledCheck; poseErrorMax cannot
% be tightened in its place because it is simultaneously the IK residual every candidate in
% hmmScreenArmPlan is screened against.
cfg.plan.releaseClearance    = 0.003;   % m, object bottom above the fitted support surface
cfg.plan.ikWeights           = [0.30 0.30 0.30 1.00 1.00 1.00];  % [orient(3) pos(3)]
cfg.plan.ikMaxIterations     = 80;      % the block ships 1500; a service cannot afford that
cfg.plan.ikMaxTime           = 0.200;   % s
% RANDOM RESTARTS ARE ON, AND THE MODEL IS STILL DETERMINISTIC. ConfigAssembly seeds
% every candidate with the arm's CURRENT joints, which is right for a re-plan and wrong
% for the first request of a task: the arm is stowed, and the stow pose is folded tight
% over its own deck. BFGS gradient projection from there stalls in a local minimum about
% 0.18 m from a grasp point that is comfortably in range - measured, the same target
% solves to 0.00000 m from an elbow-up seed - and a stalled residual fails poseErrorMax
% on all three candidates, so hmmScreenArmPlan reports Unreachable for a target the arm
% can plainly reach.
%
% Restarts are the toolbox's own remedy for a bad seed and cost nothing here, because
% ikMaxTime already bounds the whole solve. They are drawn from a SEEDED stream rather
% than the global one so the example gives the same answer on every run; an example whose
% grasp succeeds or fails depending on how many random numbers were drawn earlier in the
% session is not an example anyone can learn from.
%
% AllowRandomRestarts itself is a checkbox on the three IK blocks and so cannot be driven
% from here - a mask checkbox takes on or off, not an expression. Only the seed is a
% variable, which is the part that has to agree across all three candidates.
cfg.plan.ikSeed              = 20260101;% fixed, so restarts do not make the run vary
cfg.plan.poseErrorMax        = 0.005;   % m-equivalent, IK PoseErrorNorm acceptance
% Manipulability is the RECIPROCAL CONDITION NUMBER of J*J', measured by the
% dspmtrx3/Reciprocal Condition block rather than computed in code. There is no
% determinant block in this release, so sqrt(det(J*J')) would have to be
% hand-written; the reciprocal condition number is a standard manipulability
% index, is what a dedicated block actually measures, and is better scaled -
% it saturates at 1 for an isotropic Jacobian instead of growing with reach.
% THIS FLOOR IS NEVER APPLIED TO THE STOW POSE, and the note that used to sit here
% implied otherwise. hmmScreenArmPlan:233 tests it against the IK GRASP CANDIDATES
% only; where the arm parks is not screened by anything. The distinction matters
% because cfg.arm.stowConfig now measures 0.0014, which is BELOW this floor, and a
% reader who took the old note at face value would go looking for a broken check.
%
% The old note also cited 0.0071 "at cfg.arm.stowConfig", which does not reproduce
% and was wrong before the stow pose changed. Measured with rcond(J*J') on
% armTaskTree, consistently across tool0, tcp_frame and wrist_3_link: the previous
% pose [3.331 ...] gives 0.0021 - 0.0024, the current one 0.0014 - 0.0016. The
% ~0.0027 previously recorded for real grasp candidates agrees with that range and
% not with 0.0071.
%
% So the floor is NOT the permissive number the old note made it look like: real
% candidates land near 0.0027 against a 0.002 floor, which is a 35% margin, and it
% is doing work rather than waving things through.
cfg.plan.manipulabilityMin   = 0.002;   % rcond(J*J') floor, rejects near-singular poses
% manipulatorRRT SAMPLES, so it needs a seed for the same reason the IK restarts do -
% and it is the more important of the two, because a search that finds no path reports
% a planning failure and ends the task. Without this the identical model, config and
% base pose planned successfully on one run and failed on the next, and the difference
% was mistaken for a regression from an unrelated edit. hmmScreenArmPlan offsets this by
% the request id so a retry searches differently while the run as a whole still repeats.
cfg.plan.rrtSeed             = 20260102;% fixed, so the arm plan does not vary run to run
cfg.plan.armRrtMaxIterations = 400;
cfg.plan.armRrtMaxConnDist   = 0.400;   % rad
% The planner's edge validation is a SEARCH resolution, not the safety gate: the
% gate is hmmScreenArmPlan's own check of the parameterised reference against the
% surface at every sample. 0.10 rad halves the search cost for that reason.
cfg.plan.armRrtValidationDist = 0.100;  % rad

% THE SEARCH IS BLIND TO SELF-COLLISION, SO ITS OUTPUT HAS TO BE DRAWN MORE THAN ONCE.
% manipulatorRRT is configured with IgnoreSelfCollision = true - it has to be, because the
% imported mesh artefacts make the start configuration self-colliding to its own check, so
% it would refuse to start. Self-collision is therefore enforced AFTER the fact, by
% hmmScreenArmPlan against the artefact mask, which means every path the planner returns is
% a lottery ticket against a constraint the search never saw. One draw is not a plan, it is
% a coin flip, and the flip came up tails four times running: all four attempts of an
% integration run reported ArmSelfCollide and the arm never moved at all.
%
% MEASURED, NOT GUESSED, by planning the same stow -> grasp problem once per seed and
% applying the real gate to each result:
%
%       raw path from plan()          9 of 12 seeds pass   (75%)
%       after shorten(), 20 iters    12 of 12 seeds pass  (100%)
%
% So both constants below earn their place, and the shortening is the more valuable of the
% two. It is not merely a smoothing pass: the raw paths here are THREE waypoints, whose
% segments are long enough that the midpoint of one can be a wholly different arm pose from
% either end, and shortening returns a finer polyline - 4 to 9 waypoints - that both folds
% less and is checked more faithfully, since the gate samples waypoints and midpoints.
%
% Three attempts rather than one for the residual risk. At the measured 100% a retry is
% insurance and normally costs nothing, because the loop stops at the first path that
% passes; at the raw 75% it would still take the failure probability to 1.6%.
cfg.plan.armRrtShortenIters  = 20;      % shorten() iterations on the returned path
%
% WHEN A RE-PLAN IS ASKED FOR A PATH IT ALREADY STANDS ON. hmmScreenArmPlan now searches from
% the arm's MEASURED configuration rather than from cfg.arm.stowConfig, which is what lets the
% wrist refinement re-plan from the pre-descent pose at all. The other side of that is a
% re-plan whose goal is the standoff the arm is already holding, which happens every time the
% refinement DECLINES to move the point - a rejected provider answer, a depth outside range, a
% displacement over cfg.refine.maxShift. manipulatorRRT returns a single state for a
% zero-length problem and the caller's own "a path needs two waypoints" guard then rejects it,
% so without a threshold a DECLINED refinement would fail the task. Inside this, the two-state
% path is used directly and no search is run.
%
% THE VALUE IS A JOINT-SPACE DISTANCE AND IT IS DELIBERATELY TIGHT. 1 mrad over the whole
% 6-vector is far below anything the tracker resolves - cfg.safety.trackErrMax is 0.15 m at
% the tool - so nothing that could move the arm visibly is swallowed by it. An applied
% refinement moves the grasp point by millimetres to centimetres and maps to tens of mrad,
% comfortably outside, so it still gets a real search.
cfg.plan.armReplanTol        = 1e-3;    % rad, norm below which a re-plan has nowhere to go
cfg.plan.armPlanAttempts     = 3;       % re-seeded searches before reporting failure

% AND SHORTENING IS NOT SMOOTHING, WHICH COST 46% OF EVERY FREE MOVE UNTIL IT WAS MEASURED.
% The paragraph above calls shorten() "not merely a smoothing pass". That is true and it was
% read the wrong way round for a long time: shorten() is not a smoothing pass AT ALL. It
% removes DISTANCE, and on this problem there is no excess distance to remove.
%
% The evidence, measured over 8 seeds. The joint-space L1 length of the shortened RRT path is
% 6.023 rad. The straight line from start to goal is ALSO 6.023 rad - and sweeping
% armRrtShortenIters found 6.023 rad at every value at or above 20, over 18 seeds each. The
% planner is not detouring. It returns the straight line delivered as an unsynchronised
% STAIRCASE: every joint monotone, but not moving in proportion, so the polyline is KINKED at
% every step. contopptraj then has to take the speed almost to zero at each kink to turn the
% corner inside the acceleration box, and the measured result is 20 interior instants
% essentially at rest inside a single MOVING_FREE phase whose peak elsewhere is 89% of the
% velocity box. The phase was 3.04 s for work worth 1.71 s.
%
% THE KINKS ARE TINY AND THAT IS EXACTLY WHY THEY WERE EXPENSIVE. Smoothing the polyline with
% bsplinepolytraj - the planner's waypoints used as CONTROL points, so interior corners are cut
% rather than interpolated - strays from the path manipulatorRRT certified by a median of
% 0.003 rad and a worst case of 0.016 rad, under a fifth of a degree, and buys 1.55 s:
%
%       RRT + shorten -> contopptraj                    3.369 s median (2.706 - 4.140)
%       RRT + shorten -> bspline -> contopptraj         1.819 s median (1.705 - 2.124)
%       straight line -> contopptraj                    1.713 s        <- the floor
%
% A sixth of a degree of geometric wiggle was worth nearly half the move. No tuning could have
% found this, because nothing was mistuned: the parameterisation was correct for the path it was
% given, and the path was the problem.
%
% 32 CONTROL SAMPLES, WHICH IS armWaypointCap AND NOT A COINCIDENCE. A smooth curve needs
% nothing like 32 points to be represented, but staying inside the existing cap means the plan
% bus, hmmDefineBuses and the Truncated semantics are all untouched by this. If the cap ever
% changes, this should stay a separate constant rather than follow it - the cap is a bus width,
% this is a curve resolution, and they only happen to agree.
%
% THE FIDELITY DIAL, IF THE DEVIATION EVER MATTERS. Densifying the polyline before smoothing
% pulls the curve back onto it - measured at 0.000 rad median deviation for a x4 subdivision -
% at a cost of 0.45 s (2.273 s median). It is not shipped because 0.016 rad worst case does not
% need it, but it is the knob to reach for rather than abandoning the smoothing.
cfg.plan.armSmoothPoints     = 32;      % B-spline samples handed to contopptraj

% CHOMP IS THE PRIMARY SMOOTHER, AND THE B-SPLINE IS NOW ITS FALLBACK, BECAUSE A SMOOTHER THAT
% CANNOT SEE THE OBSTACLES KEEPS PROPOSING CURVES THAT HIT THEM.
%
% WHAT WENT WRONG WITH SMOOTHING ALONE. bsplinepolytraj cuts the polyline's corners, and the
% corner-cut region is exactly the part manipulatorRRT never validated, so every curve has to
% survive a re-check, measured here over 8 seeds on each of the two free moves: on the stow ->
% preplace leg 4 of 8 curves cut a corner INTO the surface box and were rejected. Those are not
% mischecked, they are wrong, and no amount of checking converts a rejection into a trajectory -
% a rejected curve falls back to the raw polyline and the move takes 2.9 s instead of 1.5 s.
% That is a coin flip on half of the arm's critical path.
%
% manipulatorCHOMP descends a cost that is smoothness PLUS collision, so clearance is part of what
% it is solving rather than a test it has to survive afterwards. The shipped defaults already
% weight collision at 10 against smoothness at 1e-3, which is the balance the documentation
% recommends, so none of chompSmoothnessOptions/chompCollisionOptions is overridden here.
%
% MEASURED, 8 seeds per leg, median MoveTime with gate passes:
%
%   strategy                          leg 1 (mug)       leg 2 (counter)
%   RRT + shorten, no smoothing       3.369 s   8/8     2.877 s   8/8
%   + bsplinepolytraj                 1.819 s   8/8     1.518 s   4/8   <- the coin flip
%   + CHOMP on the RRT path           1.890 s   8/8     1.634 s   8/8
%
% CHOMP IS SLOWER THAN THE B-SPLINE WHEN THE B-SPLINE WORKS, BY ABOUT 0.1 s, AND THAT IS THE
% POINT. Read the expected value, not the median. On leg 2 the B-spline is 1.518 s half the time
% and 2.877 s the other half, so 2.198 s expected against CHOMP's 1.634 s - CHOMP wins by 0.56 s
% by never falling back. Over both legs it is 3.52 s against 4.02 s, and the spread collapses from
% 1.49-3.91 s to 1.55-2.36 s. The 0.1 s is what clearance costs, and it is worth paying.
%
% The mechanism is visible in the deviation from the RRT polyline: 0.006 rad for the B-spline
% against up to 0.512 rad for CHOMP on leg 2. CHOMP is not cutting the corner more carefully, it
% is leaving the polyline altogether to get away from the box.
%
% THE FALLBACK CHAIN IS KEPT THREE DEEP - CHOMP, then the B-spline, then the raw polyline - so
% this cannot make a plan fail that used to succeed. manipulatorRRT stays: it is probabilistically
% complete and CHOMP is a LOCAL gradient method with no completeness guarantee, so the planner
% supplies a route that is known to exist and CHOMP only improves it. Seeding CHOMP from the start
% and goal ALONE was also measured and worked here (1.881 s and 1.573 s, both passing) but it is
% not shipped: on leg 2 CHOMP's own IsCollisionFree came back false while the exact-mesh gate
% passed the curve, which is one seed away from being a genuine failure with no planner to fall
% back on.
%
% COST is 1.66 s of WALL CLOCK median, 3.50 s worst, per plan. It buys no simulated time at all -
% arm AWAIT_PLAN lasts 0.02 s, one supervisor tick - and it is nowhere near
% cfg.sup.timeout.armPlan, which is 8.0 s of MODEL time. Two plans a run against a 743 s
% co-simulation.
cfg.plan.armChompEnable      = true;    % CHOMP as the primary smoother, B-spline beneath it

% MaxIterations, not FunctionTolerance, is what stops this solver on this problem: every one of
% the 16 measured calls ran to the 50-iteration default rather than converging. So 50 is a
% BUDGET and not a convergence criterion, and raising it would cost wall clock for an unmeasured
% and probably small gain. It is named here rather than left as a default precisely because it is
% doing the stopping.
cfg.plan.armChompMaxIter     = 50;      % CHOMP iteration budget, and the binding stop condition

% The signed distance field the collision cost is read from. Resolution and truncation are
% coupled and neither is free: meshtsdf requires TruncationDistance > 3/Resolution, and
% manipulatorCHOMP requires it to exceed the largest sphere in the robot's own approximation,
% which is 0.122 m on this tree. 25 voxels/m gives a 3/25 = 0.12 m floor and 0.20 m clears both
% with room. Too coarse and the field cannot resolve the 20 mm surface box at all; the two boxes
% together voxelise to 29224 active voxels in 0.05 s, so there is no reason to economise.
cfg.plan.armChompTsdfRes     = 25;      % voxels per metre
cfg.plan.armChompTsdfTrunc   = 0.20;    % m, must exceed both 3/res and the largest body sphere

% The arm planner's world is the MEASURED support surface, represented as a thin
% collision box under the target. It is a proxy, not a scene mesh: the surface is
% the geometry the arm can actually drive into on the way to a table pick, and it
% is the only one perception measures. A full scene collision model is out of
% scope for this stage and is recorded as a known limitation.
%
% THE PATCH IS LOCAL, AND ITS SIDE IS DERIVED FROM THE TARGET. A proxy the size of
% a real table top would be wrong, not conservative, because perception measures no
% surface EXTENT: a box centred on a target 25 mm from the table edge fabricates
% table where there is only air, and the fabricated part lands exactly where the
% stowed arm parks. Measured on the assembled tree at the base origin,
% cfg.arm.stowConfig holds the gripper bodies at world z 0.67 - 0.80 m - table
% height - and 0.15 to 0.24 m from the base axis, so a 0.60 m patch reaches 0.30 m
% from its centre, puts gripper bodies inside the invented surface, and
% manipulatorRRT rejects the start configuration outright.
%
% This argument got MORE binding when the stow pose was turned to face forward, not
% less. The previous backward pose held the gripper at x = -0.08 to +0.03 m, i.e.
% essentially on the base axis, so "0.3 m in front of the chassis" - which is what
% this comment used to claim - was not true of it in either magnitude or direction.
% The forward pose puts the gripper at x = +0.13 to +0.21 m, genuinely in front and
% therefore genuinely in the way of a patch invented over a target ahead of the
% robot. The conclusion was right for a reason that had not been checked.
%
% So the side is the target's own footprint plus a margin, clamped: the patch is
% the surface the DESCENDING GRIPPER can strike, which is the failure mode the
% constrained descent could actually cause. Obstacles elsewhere on the surface are
% not modelled here - the base plan is what keeps the chassis clear of them.
%
% AND THAT PATCH HAS TO BE IN THE RIGHT PLACE TO BE WORTH ANYTHING, which it was not.
% hmmScreenArmPlan built it at Position(3) - Extent(3) while Position was already the
% support surface, so the slab sat 90 mm BELOW the real tabletop and the descent it
% exists to screen passed straight through where the table actually is. The box was
% correct in size and useless in position. It now sits at Position(3); this comment is
% the argument for its EXTENT, and that argument was never the part that was broken.
cfg.plan.surfaceBoxThickness = 0.020;        % m, proxy slab thickness
cfg.plan.surfaceBoxMargin    = 0.040;        % m beyond the footprint on each side
cfg.plan.surfaceBoxSideLim   = [0.12 0.25];  % m, clamp on the derived side length
% HOW FAR DOWN THE SLAB REACHES, WHICH IS A DIFFERENT QUESTION FROM HOW THICK IT IS.
% surfaceBoxThickness above still sizes the FLOOR box, where 20 mm is right: nothing
% approaches the floor from underneath. For the support surface it was wrong, and wrong in
% the way a proxy fails silently - a 20 mm sheet is something a carried object passes
% THROUGH and comes out below, back into free space that every check then clears. Measured
% on the shipped run the mug's bounding box reached 44.2 mm below the fitted place surface
% as the arm swung in. The volume under a tabletop belongs to the table, so the slab now
% models it.
%
% BOUNDED, NOT RUN TO THE FLOOR, and the bound is about envFull rather than about tables.
% envFull is checked against the tree rooted at 'map', which CONTAINS THE CHASSIS - the
% same reason the floor box exists only in the plan frame. A slab reaching the floor at a
% target the base is parked 0.5 m from is closer to the chassis than anything else this
% function builds, and the failure mode is every grasp candidate rejected and nothing ever
% planning. 0.250 m clears the deepest measured dip by a factor of five and still stops
% 0.65 m above the floor at this scene's counter and 0.30 m above it at the side table.
%
% IT DOES NOT NEED TO COVER THE DEEP PART OF THE CARRY. The same run carries the mug 300 mm
% below the counter top, but 0.358 m out from the place point - far outside a slab whose half
% width is at most 0.125 m, so no depth would catch that and the fix for it is the held-object
% body seeing the counter at all rather than a taller box. See hmmScreenArmPlan.
%
% AND THAT FIX NOW EXISTS, so this slab is no longer the whole story: the counter itself comes
% from cfg.plan.surfaceObstacleMaxBoxes below, fitted to the nav map. The slab is KEPT because
% it is the one piece of geometry that does not depend on the map being right, and because it
% is the patch the object is actually being set down on.
%
% THIS DEPTH IS NOW THE FURNITURE'S TOO, and the number holds for both consumers for the same
% reason. The map-fitted counter boxes hang this far below the fitted top rather than reaching
% the floor, because a full column traps the arm's own upper links - see below. Measured, the
% deepest counter penetration sits only 0.0856 m below the fitted top, so 0.250 m contains it
% with a factor of nearly three; the same slack that covers the 44.2 mm dip above.
cfg.plan.surfaceBoxDepth     = 0.250;        % m, slab depth below a support surface's top
% THE FURNITURE, AS OPPOSED TO THE PATCH UNDER THE OBJECT. Measured on the 40 s run of
% 2026-09-30 the carried mug passed 85.6 mm INSIDE the kitchen counter, 0.348 m from the place
% point and so 0.258 m outside the slab above, during the BiRRT-screened free move.
% hmmSurfaceObstacles covers the occupancy grid's occupied cells within arm reach with
% axis-aligned footprints, and hmmScreenArmPlan hangs each one surfaceBoxDepth BELOW the
% fitted surface top and adds it to envPlan. Below the top, NOT down to the floor: measured, a
% floor-to-top column puts the arm's own shoulder_link and upper_arm_link inside the counter
% at the pre-place pose at EVERY depth - the mount is 0.365 m outside the near edge so the
% upper links are over the worktop footprint by construction - and the place stops being
% plannable at all. The boxes come from the MAP, not from the object, which is the whole
% difference: a slab big enough to be a counter grows backwards over the parked chassis as
% fast as it grows forwards, and that is what surfaceBoxSideLim above refuses. Boxes fitted to
% measured occupancy sit where the furniture is, and the base planner's own inflation
% guarantees the chassis is not standing in any of them.
%
% SIX IS WHAT THE SHIPPED PLACE POSE NEEDS - 192 occupied cells within reach, covered exactly,
% zero left over - so the cap is set at more than twice that. It is a cap and not a target:
% greedy maximal-rectangle takes the largest rectangle first, so the boxes that survive a
% truncation are the big ones that matter, and the warning says how many cells were dropped.
% The cost is paid per collision check, which is why this is bounded at all: envPlan goes from
% two boxes to eight, and manipulatorRRT, the smoothing verifier and the per-sample gate all
% pay it.
cfg.plan.surfaceObstacleMaxBoxes = uint8(16); % cap on boxes fitted to the nav map
% FURNITURE RISING ABOVE THE SUPPORT SURFACE (hmmSurfaceObstacles): cells of the nadir height
% map between these two rises above the surface become their own boxes, from the surface up to
% their measured top - a couch backrest (0.27 m above the seat), armrests, a headboard. Above the
% upper bound the sweep may be seeing a wall cabinet with free space beneath it, so the old
% hang-below-the-surface rule applies there (2026-10-16).
cfg.plan.tallBoxMinRise = 0.080;   % m
cfg.plan.tallBoxMaxRise = 0.500;   % m
% SLACK IN THE PLANE FOR THE CARRIED OBJECT'S BOX, and it pays for a measurement rather than
% for nerves. The held body is built from ObjectTarget.Extent, which is the extent of the
% CLUSTER the rays measured, centred on that cluster's centroid - and a centroid is not the
% point the jaws hold. On the shipped mug the handle pulls the bounding-box centre 15.5 mm
% toward it, so a box centred on the tool axis under-covers the object by that much on one
% side. 20 mm covers it with margin, and on this footprint it is the difference between a
% 0.101 m box and a 0.141 m one.
%
% IN THE PLANE ONLY. The height is left exact, because the place goal deliberately puts the
% underside cfg.plan.releaseClearance above the surface and a box inflated downward would
% eat that and make the plan collide at its own goal - a plan that cannot be made, not a
% collision avoided. hmmScreenArmPlan's withHeldObject says the same thing at the code.
cfg.plan.heldObjectMargin    = 0.020;        % m beyond the carried footprint, each side
cfg.plan.ARM_MODE_PICK       = uint8(1);   % plan to the ObjectTarget
cfg.plan.ARM_MODE_PLACE      = uint8(2);   % plan to the PlaceTarget
cfg.plan.ARM_MODE_STOW       = uint8(3);   % plan to cfg.arm.stowConfig (hmmScreenArmPlan)
cfg.plan.stowChainTol        = 0.05;       % rad, joint-space join tolerance of the approach chain
cfg.plan.armClearanceStride  = 8;          % ArmPlan.MinClearance measured every Nth reference sample

%% Operator task source
% The one natural-language instruction the run submits. It is the model's only
% task INPUT; everything else about what to do is grounded from it at run time.
% The earlier *Stub trigger times and base goal that used to live here are gone:
% the supervisory charts now fire the three services, so nothing needs a
% hard-coded schedule or a hand-picked goal pose any more.
% This is the OPERATOR'S SENTENCE, and it is the one place a scene object may be
% named. Task-specific datums stay out of configuration, signals and code because
% the SYSTEM must not know which prop the task is about; the prompt is the input
% that tells it, and a prompt that named nothing would be grounding nothing.
% Everything after this line is derived: which actor, which pixel, which surface,
% which grasp.
%
% THE SENTENCE FOLLOWS THE SCENE, NOT THE OTHER WAY AROUND. It used to name the
% glass on the kitchen counter, and neither half of that survived measurement: both
% glasses need 0.873 m of the 0.850 m reach from the stand-off, and the counter at
% 0.95 m is above the arm's 0.927 m ceiling. The only prop that is visible,
% graspable, serviceable AND has a reachable support region to be set down on is the
% red mug on the living-room side table, with the place on the kitchen worktop at
% 0.910 m - so that is what the operator asks for. Building the replay corpus warned
% when this sentence stopped matching what it selected, which is what caught it.
cfg.task.instruction         = 'Pick up the red mug from the side table and set it down on the kitchen counter.';

% HOME IS THE SPAWN POSE, AND IT USED TO BE THE SPAWN POSE TURNED ROUND. This field
% was written when cfg.base.initialPose faced theta = 0, which was into a wall:
% ray-casting data/hmmNavMap.mat out of that point gives 0.70 m of clearance dead
% ahead, and the sweep from there reads 0.70 m at 0 deg, 1.00 at 45, 2.75 at 75,
% 4.00 at 90, 1.70 at 135 and 2.50 at 180. So "go to the home position" meant the
% start position with the BACK to the wall - theta = pi, 2.50 m of open floor ahead;
% 90 deg has more clearance still but that is side-on to the wall, not backed onto it.
%
% THAT REASONING WAS RIGHT AND IT HAS NOW BEEN APPLIED TO THE SPAWN TOO, so the two
% poses are identical rather than 180 deg apart. The old arrangement meant the robot
% began every run in a pose its own home verb would immediately have corrected, and
% "return home" drove it somewhere it had never been - see the yaw discussion at
% cfg.base.initialPose, which carries the current measurements.
%
% THE FIELD IS KEPT, AND KEPT DERIVED, rather than collapsed into initialPose. It
% names a TASK TARGET and initialPose names a RIG PROPERTY; they coincide today and
% nothing requires them to. x and y are taken from initialPose so they cannot drift,
% and the heading is still authored here - it just no longer disagrees.
cfg.task.homePose            = [cfg.base.initialPose(1:2); pi];  % [x y theta], m/m/rad

%% Supervisory modes, requests and status codes
% Every chart mode and every command verb is a NUMBER DECIDED HERE, because the
% same value is read on both sides of a bus: the planner writes
% cfg.sup.armReq.APPROACH and ArmSupervisor compares against it. Two literals
% typed into two charts is the failure this section exists to prevent.
%
% The planner modes are TaskPlanner's state machine, in the order it traverses
% them. GROUND is the one wait on the metered provider; DISPATCH is where an
% action leaves the queue.
cfg.sup.pl.IDLE              = uint8(0);
cfg.sup.pl.GROUND            = uint8(1);   % grounding request issued, awaiting reply
cfg.sup.pl.DISPATCH          = uint8(2);   % pop the next queue item, decide the goal set
cfg.sup.pl.NAV_TO_PICK       = uint8(3);
cfg.sup.pl.PICK              = uint8(4);   % approach, close, validate, retreat
cfg.sup.pl.NAV_TO_PLACE      = uint8(5);
cfg.sup.pl.PLACE             = uint8(6);   % approach, open, retreat
cfg.sup.pl.RECOVER           = uint8(7);   % bounded retry or abandon this action
cfg.sup.pl.DONE              = uint8(8);
cfg.sup.pl.FAILED            = uint8(9);
cfg.sup.pl.STOPPED           = uint8(10);  % latched e-stop terminal mode

cfg.sup.base.IDLE            = uint8(0);
cfg.sup.base.AWAIT_PLAN      = uint8(1);
cfg.sup.base.FOLLOWING       = uint8(2);
cfg.sup.base.RECOVERING      = uint8(3);
cfg.sup.base.HOLDING         = uint8(4);
cfg.sup.base.STOPPED         = uint8(5);

cfg.sup.arm.IDLE             = uint8(0);
cfg.sup.arm.AWAIT_PLAN       = uint8(1);
cfg.sup.arm.MOVING_FREE      = uint8(2);   % joint-space tracking of the planned path
cfg.sup.arm.APPROACH         = uint8(3);   % constrained Cartesian descent
cfg.sup.arm.RETREATING       = uint8(4);   % constrained Cartesian lift
cfg.sup.arm.STOWING          = uint8(5);
cfg.sup.arm.HOLDING          = uint8(6);
cfg.sup.arm.STOPPED          = uint8(7);

cfg.sup.grip.OPEN            = uint8(0);
cfg.sup.grip.CLOSING         = uint8(1);
cfg.sup.grip.CLOSED          = uint8(2);
cfg.sup.grip.VALIDATED       = uint8(3);   % plant confirmed capture
cfg.sup.grip.OPENING         = uint8(4);
cfg.sup.grip.FAULTED         = uint8(5);

% Command verbs. Each subordinate accepts exactly these and nothing else, which
% is what lets the planner drive it without reading its internal state.
cfg.sup.baseReq.NONE         = uint8(0);
cfg.sup.baseReq.GOTO         = uint8(1);   % plan to GoalPose, then follow
cfg.sup.baseReq.HOLD         = uint8(2);
cfg.sup.baseReq.STOP         = uint8(3);

cfg.sup.armReq.NONE          = uint8(0);
cfg.sup.armReq.PLAN_PICK     = uint8(1);   % plan and traverse to the pregrasp
cfg.sup.armReq.PLAN_PLACE    = uint8(2);
cfg.sup.armReq.APPROACH      = uint8(3);
cfg.sup.armReq.RETREAT       = uint8(4);
cfg.sup.armReq.STOW          = uint8(5);
cfg.sup.armReq.HOLD          = uint8(6);
cfg.sup.armReq.STOP          = uint8(7);

cfg.sup.gripReq.NONE         = uint8(0);
cfg.sup.gripReq.OPEN         = uint8(1);
cfg.sup.gripReq.CLOSE        = uint8(2);
cfg.sup.gripReq.STOP         = uint8(3);

% Status states, shared by all three *Status buses. BUSY and DONE are reported
% against the SERVED REQUEST ID, never on their own: a bare done cannot say
% which request it finished, and that ambiguity is what the handshake removes.
cfg.sup.st.IDLE              = uint8(0);
cfg.sup.st.BUSY              = uint8(1);
cfg.sup.st.DONE              = uint8(2);
cfg.sup.st.FAILED            = uint8(3);

% Whole-task outcome, reported on TaskResult.
cfg.sup.out.PENDING          = uint8(0);
cfg.sup.out.SUCCESS          = uint8(1);   % every queued action reached DONE
cfg.sup.out.PARTIAL          = uint8(2);   % some actions failed, the rest completed
cfg.sup.out.FAILED           = uint8(3);
cfg.sup.out.ABORTED          = uint8(4);   % e-stop or operator cancel

% E-stop causes. The monitor LATCHES the first one it sees, so this list is a
% priority order as well as an enumeration: lower number wins a tie.
cfg.sup.cause.NONE           = uint8(0);
cfg.sup.cause.JOINT_LIMIT    = uint8(1);
cfg.sup.cause.JOINT_VEL      = uint8(2);
cfg.sup.cause.JOINT_ACCEL    = uint8(3);
cfg.sup.cause.WORKSPACE      = uint8(4);
cfg.sup.cause.BASE_VEL       = uint8(5);
cfg.sup.cause.BASE_ACCEL     = uint8(6);
cfg.sup.cause.STALE_STATE    = uint8(7);
cfg.sup.cause.STALE_PLAN     = uint8(8);
cfg.sup.cause.STALE_PROVIDER = uint8(9);
cfg.sup.cause.nCauses        = 9;

%% Supervisory timeouts and retry bounds
% MEASURED, NOT GUESSED. Stage 2 timed the two planning services on this
% machine: 2.483 s for a base plan and 2.970 s for an arm plan, both wall clock
% inside one triggered/enabled step. A timeout set at the measured latency plus
% a token margin would fire on a slower machine or a harder query, so each is
% set at roughly 2.5x the measurement. They are still FINITE, which is the point:
% a service that never answers must produce a failure state, not a hang.
cfg.sup.timeout.ground       = cfg.provider.timeout + 2.0;  % s, one grounding round trip
% THE REFINEMENT TIMEOUT IS SIZED LIKE ground BECAUSE IT IS THE SAME ROUND TRIP - one
% provider call over one image - plus the capture window the rig needs to render the wrist
% frame before the request is even issued. It matters that this is finite and SHORT: the
% refinement is advisory, so the only thing a hung provider may cost is the wait itself.
% Pick proceeds with the coarse point when this fires, which is exactly the behaviour of a
% run with cfg.refine.enabled false.
cfg.sup.timeout.refine       = cfg.provider.timeout + 2.0 + cfg.camera.captureWindow;
cfg.sup.timeout.basePlan     = 6.0;    % s, measured 2.483
cfg.sup.timeout.armPlan      = 8.0;    % s, measured 2.970
cfg.sup.timeout.follow       = 60.0;   % s, one base traverse
cfg.sup.timeout.armMove      = 20.0;   % s, one joint-space traverse
% AND THIS ONE IS NOT A MEASUREMENT AT ALL, WHICH IS WORTH SAYING BECAUSE IT USED TO SET
% THE ROBOT'S SPEED. ArmSupervisor derived its descent rate as 3 * approachDepth / this
% number, so until the rate became a constant of its own (cfg.sup.approachRate, further
% down this file) a MORE GENEROUS timeout made the arm SLOWER. The nominal segment is now
% approachDepth / approachRate = 1.2 s and this is a backstop over it, sized like the
% others at a small multiple rather than derived. hmmSetup enforces the relation, because
% two constants that must keep a ratio and sit 160 lines apart will not keep it by comment.
cfg.sup.timeout.approach     = 4.0;    % s, backstop on a 1.2 s segment, see approachRate
cfg.sup.timeout.stow         = 20.0;   % s

% The gripper budget is derived from the hardware it drives rather than picked:
% the full stroke at the commanded ramp rate, doubled to allow for settling and
% the closure-detection dwell.
cfg.sup.timeout.grip         = 2 * cfg.gripper.maxOpenPos / cfg.gripper.apertureRateMax;
cfg.sup.graspSettle          = 0.30;   % s, aperture must hold inside tolerance this long

% Retries are BOUNDED and counted per action, so a plan that fails for a
% structural reason (no path exists) costs a fixed amount of time and then
% reports PathPlanFailed instead of retrying for the length of the run.
cfg.sup.retry.basePlan       = uint8(2);
cfg.sup.retry.armPlan        = uint8(2);
cfg.sup.retry.grasp          = uint8(2);
cfg.sup.retry.action         = uint8(1);   % whole-action retry after RECOVER

% Policy: one failed action does not abandon the queue. The planner marks that
% item EXEC_FAILED, records its cause, and dispatches the next one; the task
% then reports PARTIAL rather than FAILED. A latched e-stop is the exception -
% it is terminal for every chart.
cfg.sup.continueOnActionFailure = true;

% Standoff geometry the planner uses to turn a grounded target into a base goal.
% The base stops far enough back that the arm reaches the target without the
% chassis entering the target's own footprint.
%
% THIS NUMBER IS SQUEEZED FROM BOTH ENDS AND THE SHIPPED 0.55 SATISFIED ONLY ONE.
% From below, the base has to be somewhere the planner will accept: the object
% stands ON a surface the ceiling cameras map as an obstacle, so a stand-off inside
% the inflation radius is an invalid goal state no matter how good the arm is. From
% above, the arm has to still reach the target from there, which for a target at
% height z costs hypot(standoff - pedestalOffset, z - mountHeight) of the 0.850 m
% reach. 0.55 was chosen against the second constraint alone and came out BELOW the
% 0.628 m inflation the old stow pose produced, so every stand-off goal beside a
% mapped object was invalid by construction and the run reported PathPlanFailed.
%
% A THIRD CONSTRAINT, FROM THE PLACE LEG, CLOSED THE WINDOW FURTHER. Those two
% constraints were measured on the pick alone. But the chart puts the place goal on
% the straight line from the PICK stand-off to the place point, so the stand-off also
% decides which place points the map will accept, and a value that suits the pick can
% lose every place. Both legs therefore have to be re-measured together at each
% candidate, and with the 0.440 m pedestal they overlap comfortably rather than in the
% one-centimetre band the old 0.240 m deck left.
%
% AND A FOURTH, WHICH IS WHY EARLIER VALUES LOOKED FINE AND STILL FAILED. Every
% offline check here validated the target's MANIFEST position, while the model plans
% against the position perception RECONSTRUCTS - the centroid of the ray hits on the
% surface facing the ceiling camera. For Mug_Red those differ by 49 mm along the
% approach bearing, and because the goal is the target pushed back by the stand-off,
% a 49 mm error in the target is a 49 mm error in the goal. It moved the goal inside
% the inflated side table, and the run reported PathPlanFailed about a target the
% corpus had just certified. That divergence between manifest and reconstruction means
% a stand-off is only sound if it validates with MARGIN.
%
% 0.75 IS THE MIDDLE OF A MEASURED BAND, NOT A ROUNDED GUESS. Against the
% reconstructed pick position the goal is valid from 0.705 to 0.880 m - a 175 mm band
% - so 0.75 keeps 45 mm below and 130 mm above, and the pick was confirmed to still
% validate across +/-60 mm of bias along its own bearing. From there the pick uses 81%
% of the reach, the place at 0.910 m uses 92%, and 7 accepted place points are
% serviceable while carrying, against a 0.428 m inflation. The assertion below refuses
% a value inside the inflation.
% AND THE BEARING IS NO LONGER PART OF THIS TUNING, WHICH IS WHY 0.75 SURVIVED THE
% REAL MAP. Everything above measures the stand-off along ONE bearing - the straight
% line from wherever the base happened to be - because that was the only bearing the
% chart could produce. Against the near-empty grid that cost nothing. Against the
% measured map it made the pick a dead target: the 144.2 deg line from [5.2 -4.0] to
% the mug puts the goal inside the inflated side table, while 7 of these 24 bearings
% around the same mug at this same 0.75 m are free. hmmSelectBaseGoal now rotates the
% stand-off onto a free bearing without changing its LENGTH, and reach cost has no
% bearing term in it, so the 175 mm band and every reach percentage below still hold
% exactly as measured. The bearing became the free variable; the radius stayed tuned.
cfg.sup.baseStandoff         = 0.75;   % m, base origin to target, in the approach direction
% 24 bearings is 15 deg apart, which at 0.75 m is a 196 mm arc - coarser than a map
% cell, so a free pocket narrower than that can be missed. Missing one reports
% BaseGoalOccupied, which is a refusal to move rather than an unsafe move, and it is
% the same trade hmmPlanBasePath's escape ring makes at the same count.
cfg.sup.goalBearings         = 24;     % stand-off bearings tried around the target
% A NAVIGATE GOES TO ITS POINT (2026-10-12), not a pick's stand-off short of it; when the point is
% furniture, hmmSelectBaseGoal/nearestFreePose moves it to the nearest pose the base fits in.
cfg.plan.navigateFreeMax     = 1.50;   % m, how far that search looks
cfg.plan.navigateHeadings    = 8;      % headings tried per candidate position

% AND THE ARRIVAL HEADING IS A FREE VARIABLE TOO, FOR A REASON THE OLD LAYOUT DID NOT HAVE.
% Every bearing above is driven facing the target, which was the only sensible yaw while the
% arm was mounted 60 mm FORWARD of the base centre: facing the target then put the mount on
% the near side and made the lever SHORTER than the stand-off. The mount is now 60 mm aft and
% 27.5 mm to one side, so facing the target puts it on the far side and the lever is 60 mm
% LONGER than the stand-off - the wrong direction, and enough to put the place leg's pre-place
% pose outside cfg.arm.leverCeiling. Turning the base on the spot moves the mount around a
% circle of radius hypot(0.060, 0.0275) = 66.0 mm, so the heading is worth 132 mm of lever,
% and hmmStandoffRing now spends as little of it as the reach table demands.
%
% arrivalYawOffsets IS ORDERED BY HOW LITTLE IT ASKS FOR, NOT BY WHAT REACHES FURTHEST. The
% minimum-lever heading is a fixed property of the offset - 155.4 deg, where the mount points
% straight at the target - and parking there unconditionally would be a 155 deg turn bought
% for nothing on every target the robot can already reach facing forwards. So the offsets are
% tried smallest-first and the first one the reach table and the map both accept is taken:
% nose-on stays nose-on wherever nose-on works, and the leg that needs turning turns exactly
% as far as it needs. The grid is the BEARING grid's own resolution rather than a second
% constant, so heading and bearing cannot be tuned into disagreement.
cfg.sup.arrivalYawOffsets    = 24;     % arrival headings tried per bearing, 0 first

% THIS IS A FALLBACK NOW AND NOT THE ANSWER, AND THE PARAGRAPH IT REPLACES WAS WRONG.
% cfg.arm.leverCeiling is indexed by the TOOL's height above the mount, and the pose a
% stand-off has to satisfy is the tool at the target's support surface plus whatever the verb
% lifts it by: hmmGraspAim's rise for a pick, that plus cfg.plan.releaseClearance for a place.
% They come to 0.0526 and 0.0556 m for the shipped mug, where the fixed 0.65 fraction this
% replaced gave 0.0661 and 0.0691 m.
%
% WHAT THE OLD COMMENT CLAIMED, AND WHY IT COST A RUN. It said 0.100 m was a deliberate
% overestimate, safe because "reading the table too high costs a few degrees of unnecessary
% heading, which reach is provably indifferent to". Reach is indifferent to the BEARING, which
% is the claim hmmArmLeverBand actually proves; it is not indifferent to the heading, and more
% to the point the thing the lift shortens is not a heading at all. The lift is added to dz, dz
% indexes the ceiling, and the ceiling is a LEVER - so 31 mm of padding is 31 mm off the
% admissible stand-off, one for one. At the shipped 0.900 m counter that closed the place ring
% completely: zero free bearings on all 24 bearings at every radius from 0.30 to 0.80 m, the
% goal handed back unchanged, BaseGoalOccupied 20 ms into NAV_TO_PLACE, and the held mug
% dropped. Padding a number that gates a hard bound is not a conservative choice, it is a
% smaller bound.
%
% AND A SINGLE CONSTANT COULD NOT HAVE BEEN SAFE AT BOTH ENDS ANYWAY. The lift scales with the
% object: 0.069 m for the 90 mm mug, 0.108 m for a 150 mm bottle. 0.100 m over-estimates the
% first and UNDER-estimates the second, and under-estimating is the direction that certifies a
% stand-off the arm cannot work from - the failure that drops a held object. So hmmArmLeverBand
% takes the held object's extent and computes the lift from it; every caller that knows what is
% in the gripper passes it.
%
% WHAT IS LEFT FOR THIS CONSTANT IS THE UNKNOWN OBJECT, and there overestimating genuinely is
% safe, because an over-tight ceiling refuses stand-offs rather than certifying them. It is
% also what the PICK leg uses, correctly: nothing is in the gripper yet, and the tool still has
% to rise hmmGraspAim's aim plus the pad drop to close on it. 0.100 m covers that for anything
% the 85 mm gripper can take, and covers it by MORE than it used to: with the aim now a clamped
% mid-height rather than 0.65 of the height, the worst rise over the graspable set fell by
% roughly 15 per cent of the tallest object's height.
cfg.sup.arrivalLift          = 0.100;  % m, tool above a support surface when the object is unknown

% THE PLACE CANNOT USE THE PICK'S STAND-OFF, AND ASSUMING IT COULD COST A WHOLE RUN.
% Everything above tunes ONE radius for every verb, and the paragraph on the bearing
% explains why that survived the real map. The RADIUS did not. The place leg failed
% with PregraspUnreachable (31) at t = 23.560 s - one supervisor tick after PLACE was
% entered - dropping the mug (AttachActorId 12 -> 0) and falling into RECOVER, on a run
% where the base had driven the whole way and arrived cleanly.
%
% WHY THE PLACE IS THE TIGHT VERB, AND IT IS NOT THE HEIGHT OF THE COUNTER. The tool
% does not go to the place POINT. MotionPlanning/TargetSelect lifts it by
% releaseClearance + Extent(3)/2, because the point is where the object's BOTTOM must end
% up and the tool is holding the object at its CENTRE: 0.060 m with the run's mug. Then
% hmmScreenArmPlan gates on the PRE-place, a further cfg.sup.approachDepth back along the
% TOOL's own z, which for a top-down place is 0.080 m of pure HEIGHT. So the pose the model
% actually screens sits 0.140 m above the place point, and the pick's offset runs the other
% way - it goes UP from the support surface to the object's centre. The two verbs cannot
% share one radius because they are measured from different surfaces.
%
% THE NUMBERS IN THIS PARAGRAPH MOVED TWICE AND BOTH TIMES DOWNWARD, which is the direction
% that buys place reach. The lift was 0.085 m while the tool was placed Extent(3) +
% releaseClearance - graspDepth above the point; redefining Position as the support surface
% made it 0.060 m. The standoff was 0.120 m before the pick's ladder search split the two
% verbs and left the place on 0.080 m. 0.205 m of screened height became 0.140 m, so the
% place arrives 65 mm lower than when the window below was measured - the window is
% therefore conservative, not stale, though it has yet to be re-measured.
%
% THE WINDOW, MEASURED by sweeping the radius and running the solver the model runs,
% against the shipped map grown by hmmBaseInflation's DISC while carrying the mug:
%     reach       solvable at every radius from 0.50 up to 0.78 m; 0.79 m fails with a
%                 residual of 8.9e-03 against cfg.plan.poseErrorMax of 5.0e-03
%     map         no free bearing at all below 0.64 m - the counter's inflation
% so the goal must sit in [0.64, 0.78]. The pick's window is [0.67, 0.90] and 0.75 is
% comfortably inside it, which is why one radius looked sufficient for as long as the
% place was never reached.
%
% THE LOWER BOUND IS NOW CONSERVATIVE, AND DELIBERATELY LEFT THAT WAY. That 0.64 m is a
% MAP bound, and the map bound moved when the base planner stopped modelling the robot as
% a disc: hmmBaseValidators grows the counter by the footprint's half-width plus the
% margin, 0.288 m, where the disc grew it by 0.468 m, so the true floor is lower than
% 0.64 m and the window has not been re-measured to say by how much. Only the floor
% moved. The 0.78 m ceiling is a REACH bound - the arm solver, which knows nothing about
% maps - and it is what cfg.sup.placeStandoff = 0.66 is chosen against, together with the
% 0.113 m arrival tolerance below. So the shipped radius stays valid under either robot
% model and the stale figure cannot license anything; it can only refuse a radius that
% would now be legal, which is the harmless direction.
%
% AND THEN THE ARRIVAL ERROR EATS THE TOP OF IT, WHICH IS THE PART THAT WAS MISSING.
% cfg.plan.baseGoalTolerance is [0.08 0.08 0.20], so the base may legally stop 0.113 m
% off its goal, and it USES that: on the failing run it stopped 0.0574 m from the
% commanded goal - 72% of the per-axis allowance - which put it at radius 0.7951 m from
% the place point against a commanded 0.750 m. 0.795 is outside the window. The goal
% was legal, the arrival was legal, and the pose was impossible.
%
% ONLY THE TOP OF THE WINDOW IS GIVEN THE MARGIN, AND THE REASON IS ASYMMETRIC FAILURE
% RATHER THAN AN ASYMMETRIC ERROR. The first draft of this comment claimed the arrival
% error is biased outward - the base approaches from outside and declares arrival on a
% threshold, so it stops short, at a longer lever. That is what the failing run did
% (0.7951 m against a commanded 0.750 m) and it is NOT what the passing runs do: with
% this constant at 0.66 the base has arrived at radius 0.6320 m and, on the current
% reference, 0.6229 m - 28 and 37 mm INSIDE its goal.
% (Those are RADIAL figures, which is the only kind that is comparable with the radial
% tolerance, and they are not a plain distance to the goal, because the goal the
% base is given is NOT the goal the chart derives - the selector rotated it 15 deg on the
% first run and 48.1 deg on the current one, and comparing the arrival with the un-rotated
% bearing reports a spurious 0.177 m, then 0.358 m, of "error" that is just the arc. The
% error against the given goal carries some of that arc too, so the two must be split.)
% The error is two-sided. What is one-sided is the consequence. Overshooting outward
% walks off the reach ceiling and drops a held object; undershooting inward walks into a
% floor that is not geometry but the INFLATED map - wall plus baseRadius plus
% stowedArmRadius plus plannerMargin - so a few millimetres inside 0.64 m is still
% millimetres of clearance away from the counter, and reach only improves. Hence 0.66:
% it puts the whole 0.113 m tolerance ball under the reach ceiling, and treats the map
% floor as the soft end. The 0.64 m figure bounds where a GOAL may be placed, not where
% the chassis may end up - hmmPlanBasePath validates every reference state against the
% inflated map, but the follower tracks those states to a tolerance, not exactly.
%
% 0.020 m OF MAP MARGIN IS THIN AND THAT IS THE HONEST TRADE. The window is 0.140 m
% wide and the tolerance ball is 0.113 m, so the two barely fit and no value of this
% constant has comfortable margin at BOTH ends. The end that was chosen to be thin is
% the one whose failure is clean: a goal with no free bearing is reported by
% hmmSelectBaseGoal as BaseGoalOccupied and retried, whereas an unreachable pre-place
% drops a held object.
%
% THE LEVER NAMED HERE HAS SINCE BEEN PULLED, and it was cfg.sup.approachDepth: 0.120 ->
% 0.080 raised the measured pre-place ceiling from 0.780 to 0.815 m, so the reach end of
% this trade now has 42 mm rather than 7 and it is the MAP end that is thin. It was done
% globally rather than as the per-verb constant this comment anticipated, because the pick
% leg wanted the same 0.8 s and is nowhere near its own reach limit - a second constant
% would have bought nothing and cost a coupling. A per-verb depth is still the right shape
% if the place leg ever needs more; see cfg.sup.approachDepth.
%
% AND THE MAP END IS NO LONGER WHERE THE MARGIN IS SPENT, WHICH IS WHY THIS CONSTANT DID
% NOT HAVE TO MOVE WHEN THE STOW POSE WENT VERTICAL. What made the map end thin was never
% this radius on its own - it was this radius measured to a place point that sits 128 mm
% INSIDE the kitchen unit's own floor footprint, so only 0.532 m of the 0.660 m was ever
% clearance from the obstacle, against a footprint whose geom.forward was 0.505 m. Twenty-
% seven millimetres. The vertical stow pose added 60 mm to geom.forward and all 24 of the
% stand-off bearings around that point closed; the place leg failed BaseGoalOccupied after
% a successful pick and dropped the mug. Raising this constant cannot fix that - the ring
% does not reopen until 0.725 m and placeRadiusMax minus the arrival tolerance caps it at
% 0.702 - so the fix went where the free variable actually was.
%
% THE FREE VARIABLE IS WHERE ON THE SURFACE THE OBJECT GOES, and nothing was choosing it.
% hmmReconstructTargets/biasToReachable now moves the place point inside the region the
% plane fit already certifies, to the candidate whose stand-off ring has the most free
% bearings. On the shipped scene that is 182 mm along the counter toward the open side:
% zero free bearings become six, and the 27 mm that this radius had to find in the map is
% no longer being asked of it. A serviceable fitted point is never moved, so this changes
% no run that was working. What it does mean for this constant is that its map end now has
% the whole region to spend instead of nothing, and the 0.020 m quoted above is a floor on
% a case that the reconstruction will normally have moved away from.
% AND 0.66 IS NOW 0.48, BECAUSE THE MOUNT CAME DOWN AND THIS RADIUS IS DOWNSTREAM OF IT.
% Everything above is a map argument; what forced this change is the ARM. The place pose puts
% the tool one arrival lift above a 0.900 m counter, and with the mount at 0.4441 m that is
% 0.525 m above the shoulder, where cfg.arm.leverCeiling allows a 0.568 m lever. At 0.660 m the
% worst arrival the base is licensed to make needs 0.707 m and does not solve. Measured on the
% shipped map through hmmStandoffRing, at the place point the run's own reconstruction produces
% and with the mug in the gripper: zero free bearings from 0.52 m up, where reach closes, and
% zero at 0.44 m and below, where the footprint stops fitting. One or two at every 10 mm in
% between, two at 0.48 and 0.50. 0.48 is taken because it is the exact middle of that 0.45 ..
% 0.51 m window - 40 mm of reach margin, three map cells of footprint margin - and the two ends
% are not the same kind of constraint, so the middle is the only defensible place to sit.
% The pre-place IK check agrees: the pose solves to 2.3e-09 at 0.48.
%
% IT WAS BRIEFLY 0.40, AND THAT IS WORTH RECORDING BECAUSE OF HOW IT WAS WRONG RATHER THAN BY
% HOW MUCH. 0.40 was measured, on the shipped map, through this same function - and against two
% pieces of geometry that were not the run's. The sweep hardcoded a place point 447 mm from the
% one the reconstruction actually publishes, and its held extent was [0.070 0.070 0.095] where
% the model carries ObjectTarget.Extent = [0.101 0.070 0.090], which inflates the
% footprint 15.8 mm further (0.2978 -> 0.3136 m). Either error alone moves the free-bearing
% count; together they turned "four bearings at 0.40" into zero, the goal was handed back
% unchanged, and the run failed BaseGoalOccupied 20 ms into NAV_TO_PLACE with the mug in hand.
% A radius tuned against a stand-in target is not tuned. Both stand-ins are fixed.
%
% AND THERE WAS A THIRD, WHICH WAS THE WORST OF THEM BECAUSE IT READ AS CAUTION. That sweep
% also built its own state validator - inflate(map, hmmBaseInflation), a single radius in every
% direction - where hmmPlanBasePath covers the base's footprint RECTANGLE with a chain of
% circles and rotates it with the goal's yaw. The two agree only nose-on. Turned across the
% approach, which is precisely the arrival this counter needs, the disc still projects its full
% 0.314 m at the surface while the real footprint presents its side, so the sweep refused every
% heading at every radius and reported "NO stand-off in 0.40..0.90 m satisfies all three gates"
% on a leg that has a 70 mm window. It understated the PICK leg too - one free bearing at 0.67 m
% against the model's fifteen at 0.68 m. It was rebuilt on the planner's own validator. A sweep
% whose robot is not the model's robot is not a measurement, and a conservative one is not
% safe either: it hides the window it was built to find.
%
% AND THE REACH CEILING MOVED UNDER IT AT THE SAME TIME, WHICH IS THE LARGER HALF OF THE FIX.
% cfg.sup.arrivalLift was a flat 0.100 m; the mug's true place lift is 0.0691 m, and because the
% lift indexes a LEVER ceiling the 31 mm of padding cost 31 mm of admissible radius one for one.
% With the padding in place NO radius worked at this place point - the reach window closed below
% the map window rather than overlapping it. hmmArmLeverBand now takes the held object and the
% constant is only a fallback; see its header, and cfg.sup.arrivalLift below.
%
% NO HEADING RESCUES 0.660, WHICH IS WHY THIS CONSTANT HAD TO MOVE AND NOT JUST THE ARRIVAL.
% hmmStandoffRing chooses the arrival heading against reach now, and turning the base walks the
% mount round a 66 mm circle - worth 132 mm of lever, which is a lot and is still not 211. The
% heading search is what makes 0.48 work at all (nose-on has NO feasible radius, because a
% nose-on park needs geom.forward = 0.502 m of clearance where a turned one needs only
% geom.inflation = 0.290) but it cannot make 0.660 work. Both changes are needed and they fix
% different halves. The pose the ring actually picks is 90 deg across the approach, parked
% square in front of the counter at [3.704 -5.829], which is the heading the measurement above
% predicted and not a coincidence: it is the smallest turn the reach table accepts.
%
% THE FOOTPRINT WIN PAID FOR THIS, AND THE TRADE WAS WORTH MAKING. Lowering the mount 200.9 mm
% took the inflation radius from 0.453 to 0.290 m - under the 0.300 m doorway ceiling, which is
% what made the Bedroom and Bathroom reachable - and cost 211 mm of place-leg lever, because a
% vertical tool spends the gripper's length on height rather than radius. A 0.18 m shorter
% place stand-off is the cheaper side of that trade: it costs margin on one leg, where the
% inflation bought entry to two rooms. The alternative on the table was 56 mm of extra pedestal,
% which buys the window back (max 0.540 m, eleven bearings at 0.46) and reopens a layout that
% was settled; it is noted here rather than taken.
%
% TWO BEARINGS OF TWENTY-FOUR IS THIN, AND IT IS THIN FOR A REASON NO CONSTANT HERE CAN FIX.
% The reach window at this counter height closes at 0.52 m and the map window opens at 0.45 m;
% they overlap by 70 mm. That is what a 0.900 m surface costs an arm mounted 0.4441 m up, and the
% only levers that widen it are the mount height (declined) and cfg.plan.baseGoalTolerance, whose
% 0.113 m diagonal is 20 % of the whole lever budget and which was judged risky to tighten
% because the follower may then never declare arrival. The pick leg has fifteen bearings; the
% asymmetry is real and is a property of the scene, not of this number.
%
% WHAT TO RE-MEASURE IF THE COUNTER HEIGHT OR THE MOUNT MOVES. Not this number by hand - the
% two bounds come from different places. The upper one is cfg.arm.leverCeiling read at the
% place height, less the base's arrival tolerance, plus norm(cfg.pedestal.offset(1:2)); the
% lower one is the map, and only hmmStandoffRing knows it. Sweep both and count the free
% bearings per radius, which is the figure to tune on.
cfg.sup.placeStandoff        = 0.48;   % m, as baseStandoff but for the place leg
% Stand-off radii tried, nearest the chart's first, when its radius has no free bearing - see
% hmmStandoffRadii. 16 rungs of one map cell cover the arm's whole lever window for a low
% surface (0.07-0.82 m at 0.40 m holding a bottle); the old local 6 stopped at ~0.62 m and a
% couch seat needed 0.57-0.67 m (2026-10-15). Perception and the goal search share it.
cfg.sup.standoffMaxRungs     = 16;
% THE CEILING IS MEASURED, NOT DERIVED FROM cfg.arm.reach, AND THE DIFFERENCE MATTERS.
% Every reach figure in this file is a fraction of cfg.arm.reach, which is the right
% currency for a rough feasibility argument and the WRONG one for a gate. The model's own
% pre-place pose solves at radius 0.78 m with a residual of 4.2e-09 - a clean solution - and
% that pose is 101.3% of cfg.arm.reach. So cfg.arm.reach UNDERSTATES what this tool frame
% actually achieves, because it is the nominal span of the arm and the pose is solved to the
% tool, and any check written as "fraction of reach must stay below 1" therefore refuses
% poses the solver reaches. That is the same sign of error as the mount-height defect
% hmmSetup documents, and it fails the same way: by refusing work the robot can do.
% So the bound stated here is the radius at which the SOLVER stops succeeding, measured by
% sweeping it. It is a property of the place pose - the counter height, the release lift and
% approachDepth all move it - so re-measure it after changing any of them rather than scaling
% this by hand. hmmSetup checks placeStandoff plus the base's whole arrival tolerance against
% it, which is the check that was missing when the place leg dropped a mug.
%
% 0.815 AND IT WAS 0.78, WHICH IS A RE-MEASUREMENT AND NOT A RELAXATION. The 0.78 was correct
% at cfg.sup.approachDepth = 0.120; the depth is now 0.080 and this constant is downstream of
% it. Re-sweeping in 0.005 m steps reproduces the old figure exactly at the old depth, which is
% what makes the new one trustworthy rather than convenient:
%
%     depth 0.120  ->  solves to 0.780, fails at 0.785   (the previously recorded value)
%     depth 0.100  ->  solves to 0.800, fails at 0.805
%     depth 0.080  ->  solves to 0.815, fails at 0.820   (current)
%     depth 0.060  ->  solves to 0.830, fails at 0.835
%
% Against the 0.773 m worst-case arrival radius that leaves 42 mm where there were 7. The
% margin is still not generous - the honest reading is that this leg is reach-limited and
% the depth is the only cheap lever on it - but it is no longer inside the noise of a single
% IK solve, and the failure it guards against drops a held object.
% SUPERSEDED BY cfg.arm.leverCeiling, AND KEPT ONLY BECAUSE hmmSetup STILL QUOTES IT. This
% is the same measurement that table holds, keyed the wrong way: a BASE-CENTRE radius, which
% silently encodes both the arrival heading it was measured at (nose-on) and where the mount
% stood when it was measured (60 mm FORWARD). Both of those changed. It did not become wrong so
% much as MIS-INDEXED - the table reads 0.755 m of lever at dz 0.35, which is exactly this
% 0.815 less that 0.060, so the two measurements agree and only the key was fragile - but it is
% no longer the bound anything should be tuned against, and it no longer binds: at the 0.40 m
% stand-off above, hmmSetup's check clears it by 302 mm. Read hmmArmLeverBand instead. Retiring
% the constant means changing hmmSetup's check, left alone by the heading work.
cfg.sup.placeRadiusMax       = 0.815;  % m, SUPERSEDED - see cfg.arm.leverCeiling
% 0.080 m AND IT WAS 0.120, WHICH BUYS BOTH TIME AND THE PLACE LEG'S REACH MARGIN.
% The depth is spent four times a transfer - descend, retreat, descend, retreat - so
% 40 mm off each is 160 mm of constrained travel the run no longer makes, and at the
% approachRate below that is worth about 0.8 s. That is the smaller half of the reason.
%
% THE LARGER HALF IS THAT THIS IS THE LEVER placeStandoff'S COMMENT NAMES. The pre-place
% pose sits approachDepth above the release lift above the counter, so the depth is
% pure height on a top-down grasp and it is what pushes the pre-place out to the edge of
% what the solver reaches. The ceiling was measured rising as the depth comes off -
% 0.780 m at 0.120, 0.815 m at 0.080, 0.830 m at 0.060, the full 0.005 m sweep being in
% placeRadiusMax's comment above - against a worst-case arrival radius of placeStandoff +
% the base's arrival tolerance = 0.773 m. At 0.120 that left SEVEN MILLIMETRES, and the
% failure it leaves is the bad one: an unreachable pre-place drops a held object. At 0.080
% the same sum has 42 mm, which is not generous and is six times what it was.
%
% WHY NOT 0.060, WHICH MEASURES BETTER STILL. The mug is 90 mm tall, so a descent of 80 mm
% is still a move that starts clear of the object and ends on it; at 60 mm the guarded
% approach starts to be a gesture rather than a phase, and the point of the segment is that
% the tool crosses the object's own clearance under a constrained move. cfg.sup.placeRadiusMax
% is measured against THIS value, not scaled by hand, so re-measure it if this changes again.
cfg.sup.approachDepth        = 0.080;  % m, constrained Cartesian descent length

% AND THE PICK LEG NO LONGER USES IT, WHICH IS THE PER-VERB SPLIT THE COMMENT ABOVE
% ANTICIPATED. Everything above this line is an argument about the PLACE leg: the depth is
% pure height on a top-down place, it pushes the pre-place pose off the solver's reach, and
% cfg.sup.placeRadiusMax is the measured ceiling it buys. None of that constrains the pick,
% which the same comment already says is "nowhere near its own reach limit". Measured over
% the shipped mug pick, the arm holds a VERTICAL tool out to 0.680 m of standoff before it
% stretches straight - the joint limits (+-6.23 rad) never come near binding, min margin
% 1.28 rad at 0.080 - so reach is not what decides this.
%
% SO WHY RAISE THE PICK'S, WHEN THE PICK WAS NOT FAILING. Because the wrist camera takes its
% one picture from the pre-descent pose, and at 0.080 there is no picture to take. The lens
% ends up 65 mm from the mug's top face, where the mug's 101 mm width subtends about 480 px
% in a 360-row frame: the object OVERFLOWS the frame by a third, so the second look sees a
% patch of red with no shape and the refinement it returns carries no information about the
% object. Measured at the same base pose, with every rung solved from the grasp configuration
% the way hmmScreenArmPlan solves it:
%
%     d(m)   range   mm/px   top face clear of every edge by   rcond(J*J')
%     0.080  0.186    0.60    -146 px  (overflows the frame)     0.01079
%     0.120  0.225    0.72     -19 px  (still crosses an edge)   0.01065
%     0.160  0.263    0.85     +48 px                            0.01042
%     0.200  0.302    0.98     +88 px                            0.01010
%     0.240  0.341    1.10    +112 px  <- the maximum            0.00976
%     0.280  0.380    1.23    +111 px                            0.00939
%     0.320  0.419    1.35    +110 px                            0.00898
%     0.600  0.690    2.23    +107 px                            0.00294  last rung over the
%     0.640  0.729    2.35    +106 px                            0.00145  floor; this one is
%     0.680  0.768    2.48    +106 px                            6.3e-06  arm straight out
%     0.720     -        -    IK gives up, residual 0.028        1.2e-17
%
% 0.240 IS THE ARGMAX OF THAT COLUMN, NOT A ROUND NUMBER NEAR IT, and that is a better reason
% than the one this comment first gave. The clearance rises steeply to 0.240 and then FALLS
% slowly - the object keeps getting smaller in frame after the standoff stops centring it any
% better - so there is a genuine interior optimum and it is where the ceiling is set. 1.10
% mm/px is still 15x the ceiling cameras' 16.5 mm/px at 3.798 m, which is the whole reason for
% a second look.
%
% THE CONDITIONING COLUMN IS NOT WHAT BINDS HERE, and saying so is the honest reading. Every
% rung on the ladder is at 0.0098 or better against cfg.plan.manipulabilityMin = 0.002; the
% floor does not cut in until about 0.620, where the arm is most of the way to straight. It
% still earns its place in the search - it is what stops a pick on a high surface, or from a
% base that parked far out, from choosing a rung near the singularity - but on this pick the
% ceiling above is what decides, and the conditioning screen passes everything it is shown.
%
% TWO FIGURES IN AN EARLIER DRAFT OF THIS TABLE WERE ARTEFACTS OF THE SWEEP AND BOTH ARE WORTH
% NAMING, because each looked like a physical result. The conditioning column was first quoted
% in sqrt(det(Jv*Jv')) over the linear Jacobian rows while the model screens in rcond(J*J') -
% a different number that falls at a different rate, so a threshold read off one does not
% transfer. And the sweep CHAINED its IK seeds from rung to rung, so each rung inherited the
% previous rung's elbow: that put the reach limit at 0.460 rather than 0.680 and moved 0.240's
% clearance from +112 px to +65 px. Nothing about the standoff changed between those two
% tables - only which posture the solver happened to land in. Seeding every rung from the
% grasp, which is what the planner does, is what makes the column above a property of the
% standoff.
%
% THE MOUNT DOES NOT HAVE TO FOLLOW IT, which is the fact that makes a variable standoff
% cheap. cfg.camera.wrist.tilt aims the optical axis at a point approachDepth along the tool
% axis, so in principle every standoff wants its own tilt - but the camera's 45 mm lateral
% offset subtends less angle the further away that point is, so the aim converges. Over 0.080
% to 0.680 the aim point moves only from row 180 to row 239 of 360, and never off centre in
% column at all: the offset is purely along the tool's x, so the sight line stays in the
% camera's own x-z plane and the aim column is 320.00 at every rung. One fixed tilt covers the
% whole span and no candidate puts the aim point near an edge.
cfg.sup.approachDepthPickMax  = 0.240; % m, ceiling on the pick standoff search
% A LADDER, SEARCHED DOWNWARD, BECAUSE THE HIGHEST FEASIBLE STANDOFF IS A PROPERTY OF THE
% POSE AND NOT OF THIS FILE. It depends on how high the support surface is and how far out
% the base parked, both of which vary per pick, so a single constant would either be
% unreachable on a high surface - reported as PregraspUnreachable, a refusal to pick
% something the arm can reach at a lower standoff - or be set low enough to be useless.
% hmmScreenArmPlan tries these in order and keeps the first that passes the three screens it
% already applies to the standoff plus the conditioning floor below. Each rejected candidate
% costs one seeded IK solve and one collision check, single-digit milliseconds against a
% ~3 s plan, and the ladder ends at cfg.sup.approachDepth so the worst case is today's
% behaviour rather than a failure.
%
% AND IT CONTINUES BELOW cfg.sup.approachDepth, FOR THE PLACE LADDER'S REASON (2026-10-12). The
% Bottle, on a 0.800 m chest, parked with its arm mount 0.585 m out: the grasp itself sits inside
% the lever band ([0.130 0.658]) but the 0.080 pre-grasp does not (ceiling 0.551), so the pick
% failed PregraspUnreachable twice. At 0.060 / 0.050 / 0.040 the ceiling is 0.581 / 0.596 /
% 0.610. The ladder still tries every rung above first, so a pick that works at 0.080 or higher
% is unchanged; the bottom stops at 0.040, above the place ladder's 0.030 floor.
cfg.sup.approachDepthLadder   = [0.240 0.200 0.160 0.120 0.080 0.060 0.050 0.040];   % m, in order
% CONDITIONING, NOT JUST REACH, DECIDES WHERE TO STOP, AND THE TEST FOR IT IS ONE THIS FILE
% ALREADY OWNS. An IK residual of 1e-9 says the pose is solvable, not that the arm is in a
% good place to be: the standoff is where the guarded descent STARTS, and a start near full
% extension means the same joint rates that tracked well lower down produce a Cartesian error
% the servo cannot take out. So hmmScreenArmPlan rejects a rung whose configuration falls
% under cfg.plan.manipulabilityMin - the SAME near-singular floor every grasp candidate must
% clear, rather than a second threshold invented for the standoff.
%
% THERE WAS A cfg.sup.approachManipFrac HERE AND REMOVING IT IS THE POINT. It rejected a rung
% that cost more than a fixed FRACTION of the grasp configuration's manipulability, and it was
% wrong in two ways that are worth leaving on the record because both are easy to write again.
% It was calibrated against sqrt(det(Jv*Jv')) while the model screens on rcond(J*J'), which
% put its author's "0.240 keeps 90 %" at an actual 79.8 % and would have made the search
% settle on 0.200 - a constant that quietly defeated the ceiling above it. And the relative
% shape was itself the wrong instrument: what makes a high standoff dangerous is being close
% to singular in absolute terms, not having dropped by a ratio from wherever the grasp
% happened to sit. A grasp at rcond 0.008 whose standoff sits at 0.005 is nowhere near
% trouble; a grasp at 0.0021 whose standoff sits at 0.0018 is, and only the absolute floor
% says so. On the shipped pick the floor admits every rung on the ladder and would admit up to
% about 0.320, so what binds there is approachDepthPickMax - which is correct, because that
% ceiling was chosen from the framing curve and not from the kinematics.

% AND THE PLACE LEG GETS A LADDER TOO, BECAUSE A SINGLE RUNG GAVE IT NO WAY TO RECOVER FROM
% THE ONE POSE THAT ACTUALLY BINDS. hmmScreenArmPlan's own comment had it right - a place's
% pre-place pose is what pushes the arm off its reach ceiling - and then handed the place one
% rung, so when that rung failed the whole transfer failed. The 40 s integration run did
% exactly that, twice: PregraspUnreachable at 15.78 s and again at 33.68 s, on a place the
% arm could reach perfectly well. Measured at the run's own geometry - base
% [3.7151 -5.9205 268.0 deg], place [3.2235 -5.8293 0.900], carrying the 101x70x90 mm mug,
% so an arm mount at [3.6897 -5.8596] and a lever of 0.4672 m:
%
%   depth   pre-place z   lever ceiling   verdict at lever 0.4672
%   0.080     0.980          0.4194       FAILS by 47.8 mm   <- the shipped single rung
%   0.070     0.970          0.4445       FAILS by 22.7 mm
%   0.060     0.960          0.4635       FAILS by  3.7 mm
%   0.050     0.950          0.4825       OK
%   0.040     0.940          0.5015       OK
%   0.030     0.930          0.5205       OK
%
% THE PLACE POINT ITSELF WAS NEVER THE PROBLEM: at z = 0.900 the band is
% [0.130 .. 0.5675] and the lever 0.4672 sits 100 mm inside it. What every standoff gate in
% this project screens is that number, and what has to solve is the one 80 mm higher. In this
% region hmmArmLeverBand's ceiling falls about 155 mm per 50 mm of height, so 80 mm of
% standoff costs 148 mm of admissible lever - which is why a place can be comfortably in
% reach and its pre-place pose 48 mm out of it.
%
% THE TOP RUNG IS TODAY'S VALUE, so this can only change behaviour where today the place
% FAILS - the same property that makes the pick ladder safe. The bottom rung is 0.030 rather
% than something smaller because the bottom rung is what a gate can promise, and 0.030 is the
% shortest descent at which the standoff window is non-empty against BOTH bounds that close
% on it: the map floor of 0.45 m below (the footprint collides nearer than that) and the
% 0.113 m diagonal arrival tolerance above. At 0.040 the worst licensed arrival leaves 4 mm
% of window; at 0.030 it leaves 23 mm; at 0.060 and above the window is EMPTY, which is the
% arithmetic that says the shipped depth could not have worked at this place height however
% the base parked.
%
% A SHORTER GUARDED DESCENT IS STILL A GUARDED DESCENT, which is the trade being made here.
% 30 mm at cfg.sup.approachSpeed is a real constrained move onto the surface and it still
% clears cfg.plan.releaseClearance by an order of magnitude; what it gives up is margin for
% the surface height being wrong, and the surface height comes from the depth reconstruction
% rather than from a guess. The ladder tries for the full 80 mm first and only shortens when
% the arm cannot hold the tool that high, so a place with room to spare is unchanged.
cfg.sup.approachDepthPlaceLadder = [0.080 0.060 0.050 0.040 0.030];  % m, tried in this order

% THE APPROACH SPEED IS A SPEED NOW, AND IT USED TO BE AN ARTEFACT OF A TIMEOUT.
% ArmSupervisor's approachRate constant read
%
%     3 * cfg.sup.approachDepth / cfg.sup.timeout.approach   =   3 * 0.120 / 12.0
%
% which is 0.030 m/s. Nobody chose 30 mm/s: it is what "take a third of your own timeout"
% happens to evaluate to, and the causation ran backwards - the guard that is supposed to
% catch a segment overrunning was setting how fast the segment went, so making the guard
% more generous made the robot slower. Measured, it cost four constrained segments of 4.12 s
% each, 16.48 s of a 33.26 s run, HALF THE TASK spent covering 480 mm.
%
% AND IT WAS NOT PRESSED AGAINST ANY LIMIT, WHICH IS THE PART THAT MAKES IT WASTE RATHER
% THAN CAUTION. Over those segments the fastest joint reached 0.068 .. 0.112 rad/s, which
% is 2.6 .. 4.8 PER CENT of cfg.arm.planMargin * jointVelMax. The tool lag a ramp implies
% is rate * cfg.arm.jointTimeConst - 2.4 mm at the old rate, 8 mm at this one - against a
% cfg.safety.trackErrMax of 0.15 m. There is more than a decade of headroom either way, so
% the old number was not buying margin anywhere.
%
% 0.200 AND IT WAS 0.100, AND THE CEILING IS SET BY THE CLAIM RATHER THAN BY A LIMIT.
% The approach exists to be a guarded, constrained move: it is the one segment where the
% tool is inside the object's own clearance and the gripper is about to close, and a
% descent that reads as fast on screen stops making that argument. So the question is not
% what the arm tolerates - see below, there is a decade of headroom - but how much slower
% than the surrounding free move this phase still visibly is.
%
% THAT IS MEASURABLE, AND IT WAS MEASURED ON THE REFERENCE RUN RATHER THAN JUDGED BY EYE.
% Tool-frame speed through the free move either side of a grasp is 0.339 m/s in the mean
% and 0.833 m/s at the peak; the approach at the old rate ran 0.079 mean and 0.100 peak,
% so it was 4.3x slower in the mean. At 0.200 m/s it is 2.1x slower in the mean and 4.2x
% slower at the peak - still unmistakably the careful phase, with a factor of two to say so
% by. At 0.300 the mean would land inside the free move's own variation and the claim would
% be gone, so 0.200 is the largest value this example can assert its own behaviour at, not
% a round number. Raising it further IS a change to what the example claims about grasping.
%
% WHAT IT COSTS NOTHING IN. The tool lag a ramp implies is rate * cfg.arm.jointTimeConst =
% 16 mm here, against cfg.safety.trackErrMax of 0.15 m. Joint speeds move from ~16% to
% ~32% of the planner's box. Both were checked and neither is near anything.
%
% WHAT IT BUYS. A segment is cfg.sup.approachDepth / rate = 0.40 s, against 1.20 s at the
% old rate and depth, over four segments a transfer. Lowering it is free and costs 0.4 s
% per segment for each halving.
%
% THE TIMEOUT NOW DERIVES FROM THIS, NOT THE OTHER WAY ROUND. cfg.sup.timeout.approach is
% declared above this line - it belongs with the other timeouts - so the dependency cannot
% be written as an expression here without moving one of them. hmmSetup checks it instead:
% timeout.approach must leave at least a factor of two over approachDepth / approachRate,
% and it says so if it does not. A check is what this coupling always wanted; the formula
% in the chart was the coupling pointing the wrong way.
cfg.sup.approachRate         = 0.200;  % m/s, constrained descent and lift

% THE LADDER'S ENDS ARE NOT FREE, and asserting them is what keeps the search honest. Its top
% IS the ceiling this file argues for, and it passes THROUGH today's depth - so every pick that
% solved before this search existed still solves at the same or a higher rung. Below that it may
% only continue down to the place ladder's floor, the shortest guarded descent the project
% argues for anywhere.
assert(abs(cfg.sup.approachDepthLadder(1) - cfg.sup.approachDepthPickMax) < 1e-12 && ...
       any(abs(cfg.sup.approachDepthLadder - cfg.sup.approachDepth) < 1e-12) && ...
       cfg.sup.approachDepthLadder(end) >= cfg.sup.approachDepthPlaceLadder(end) - 1e-12 && ...
       all(diff(cfg.sup.approachDepthLadder) < 0), ...
    'hmmConfig:approachLadder', ...
    ['cfg.sup.approachDepthLadder must run strictly downward from ' ...
     'cfg.sup.approachDepthPickMax = %g through cfg.sup.approachDepth = %g; it is %s.'], ...
    cfg.sup.approachDepthPickMax, cfg.sup.approachDepth, ...
    mat2str(cfg.sup.approachDepthLadder));

% THE PLACE LADDER'S TOP END IS TIED THE OTHER WAY ROUND, and that asymmetry is the point.
% The pick ladder STARTS above today's depth and descends TO it, so the pick's worst case is
% the old behaviour. The place ladder starts AT today's depth and descends BELOW it, so the
% place's BEST case is the old behaviour and every rung after the first is new ground the old
% code had no way to reach. Tying the top to cfg.sup.approachDepth is therefore what keeps
% "this can only help" true: if the depth ever changes, the place's first attempt changes with
% it and no rung silently becomes an increase.
assert(abs(cfg.sup.approachDepthPlaceLadder(1) - cfg.sup.approachDepth) < 1e-12 && ...
       all(diff(cfg.sup.approachDepthPlaceLadder) < 0) && ...
       cfg.sup.approachDepthPlaceLadder(end) > cfg.plan.releaseClearance, ...
    'hmmConfig:approachPlaceLadder', ...
    ['cfg.sup.approachDepthPlaceLadder must run strictly downward from ' ...
     'cfg.sup.approachDepth = %g and stay above cfg.plan.releaseClearance = %g; it is %s.'], ...
    cfg.sup.approachDepth, cfg.plan.releaseClearance, ...
    mat2str(cfg.sup.approachDepthPlaceLadder));

% AND THE WRIST REFINEMENT'S DISPLACEMENT CAP IS THE TOP OF THAT LADDER, checked here because
% cfg.refine is declared 1700 lines above and cannot reference cfg.sup. The refinement is
% allowed to move the grasp point by up to the standoff the arm is holding above it: a
% correction smaller than that lands somewhere the descent can still reach, and a correction
% larger than it means the second look found a DIFFERENT object rather than the same one more
% precisely.
%
% IT IS THE CEILING AND NOT THE CHOSEN DEPTH, which is a deliberate looseness rather than an
% oversight. hmmWristRefine is called with cfg alone and does not receive the arm plan, so it
% cannot know which rung the search settled on; capping at the ceiling means the cap is never
% TIGHTER than the standoff actually being held, which is the direction that cannot reject a
% correction the descent could have executed. Tying it to the chosen depth instead would need
% the plan threaded into the refinement, and the guard's job is to catch a refinement that
% found a different object - a 0.24 m error is that either way.
assert(abs(cfg.refine.maxShift - cfg.sup.approachDepthPickMax) < 1e-12, ...
    'hmmConfig:refineShiftCap', ...
    ['cfg.refine.maxShift = %g no longer equals cfg.sup.approachDepthPickMax = %g. The ' ...
     'wrist refinement cap is defined as the tallest standoff the pick search may choose; ' ...
     'update cfg.refine.maxShift, or record here why the two have deliberately parted.'], ...
    cfg.refine.maxShift, cfg.sup.approachDepthPickMax);

%% Base trajectory tracking
% Three independent channels because the chassis is HOLONOMIC: vx, vy and omega
% are separately commandable, so the tracker is three single-axis loops on the
% pose error expressed in the BASE frame, not a curvature law. Gains are modest
% on purpose - the feedforward from the plan carries the motion and the loops
% only remove error, so a stiff gain would fight the plan rather than follow it.
cfg.track.kpXY               = 1.20;   % 1/s
cfg.track.kiXY               = 0.05;   % 1/s^2
cfg.track.kdXY               = 0.00;
cfg.track.kpTheta            = 2.00;   % 1/s
cfg.track.kiTheta            = 0.05;   % 1/s^2
cfg.track.kdTheta            = 0.00;
cfg.track.iLimitXY           = 0.20;   % m/s, integrator clamp
cfg.track.iLimitTheta        = 0.40;   % rad/s
cfg.track.goalHoldTime       = 0.20;   % s inside tolerance before at-goal is reported
cfg.track.stallSpeed         = 0.020;  % m/s, measured base speed below this is no progress
cfg.track.stallTime          = 2.00;   % s stalled while not at goal before a replan request
cfg.track.crossTrackMax      = cfg.safety.trackErrMax;   % m, one value

% Arm reference tracking. The natural frequency is DERIVED from the plant's own
% joint servo time constant so the reference governor is not asked to command
% motion the actuator cannot follow.
cfg.track.armNaturalFreq     = 1 / cfg.arm.jointTimeConst;  % rad/s
cfg.track.armDampingRatio    = 1.0;

% NO TASK-SPACE GAINS. cfg.track.taskKp/taskKd/taskJointDamping/taskNaturalFreq fed a Task Space
% Motion Model that was removed from ArmControl; nothing in the model read them, and the gain
% sizing in hmmSetup went with them on 2026-10-08 (hmmSetup now marks a derived cfg with
% cfg.derived instead). The TaskSpaceMode SIGNAL is a different thing and is live: ArmSupervisor
% emits it and ArmControl/TaskTrackEngaged and TrackingMetrics/TargetSelect consume it, which is
% why ArmStatus.TrackingError is large during a stow and the guard on it is mode-scoped.

%% No local obstacle avoidance and no range scan (removed 2026-10-19)
% PathFollowing drives the tracking twist straight to the goal switch. The Vector Field
% Histogram (controllerVFH) had been switched off since every obstacle here is static and the
% planner already routes round all of them, and its range scan (rangeSensor cast through the
% planner's own occupancy grid, from the base centre) fed only a clearance check that could
% never fire. Both still ran every step - 0.12 ms and 0.65 ms of 10 ms - so they were taken out
% with the check, not left switched off.

%% Optional higher-fidelity variant
% First release is kinematic. Set to true to select the Simscape Multibody
% dynamic base/arm variant behind this configuration switch.
cfg.variant.useMultibodyDynamics = false;

end
