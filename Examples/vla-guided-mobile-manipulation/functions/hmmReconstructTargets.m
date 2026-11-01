function [obj, plc] = hmmReconstructTargets(frames, queue, registry, simTime, objTemplate, plcTemplate)
%HMMRECONSTRUCTTARGETS Grounded pixels to metric 3-D targets, gates 6 to 10 and 12.
%
%   [obj, plc] = HMMRECONSTRUCTTARGETS(frames, queue, registry, simTime,
%       objTemplate, plcTemplate)
%
%   Reconstructs the queue's HEAD action into an ObjectTarget and, for a
%   transfer, a PlaceTarget. The head is the action the executive will run next,
%   so reconstructing it and only it keeps the work proportional to what is
%   about to happen instead of to what the provider happened to return.
%
%   objTemplate and plcTemplate supply the output types. See
%   hmmGroundingService for why the types arrive as templates rather than being
%   written out here.
%
%   THE VALIDATION GATES APPLIED HERE:
%
%       6   DepthInvalid       no valid calibrated hit near the grounded pixel
%       7   OutsideRegion      the reconstructed point is outside cfg.percep.sceneBounds
%       8   AmbiguousObject    zero or several movable actors match the pick point
%       9   NotGraspable       the measured extent is outside the jaw stroke
%       10  PlaceUnsupported   no near-level support surface fits at the place point
%       12  Stale              the frame is older than cfg.safety.staleTimeout
%
%   GATES 8, 9 AND 10 ARE ABOUT AN OBJECT AND A SURFACE, SO THEY ARE TRANSFER-ONLY.
%   6, 7 and 12 are about the measurement and apply to both transfer and navigate.
%   Applying the object gates to a navigate action - whose target is a place, not a
%   thing - made the navigate verb unexecutable for the whole history of the project;
%   see the comment above the navigate branch below for what that looked like from
%   the logs.
%
%   A HOME ACTION IS GATED BY NONE OF THEM, because it grounds nothing: its pose is
%   cfg.task.homePose, a surveyed constant, so there is no frame, no pixel and no ray
%   to check. It returns before the frame is even admitted; see the home branch below.
%
%   Gate 11, Unreachable, is NOT applied here and that is deliberate. A target
%   four metres away is not unreachable, it is somewhere the base has to drive
%   to first, and perception has no business deciding that. Unreachable means
%   "inverse kinematics found no admissible arm solution", so it is raised by
%   the motion-planning service, which is the only component that knows.
%
%   AND THE PLACE POINT'S MAP CHECK IS NOT GATE 11 IN DISGUISE, WHICH IS WORTH SAYING
%   PLAINLY BECAUSE IT LOOKS LIKE IT. biasToReachable consults the nav map and the base
%   footprint, which is planner knowledge arriving in perception. What it may do with
%   the answer is the distinction: it can only CHOOSE AMONG POINTS ON A SURFACE THIS
%   FUNCTION HAS ALREADY CERTIFIED, and it never rejects anything. An unserviceable
%   place point with no serviceable alternative is published exactly as fitted, and the
%   run fails at hmmPlanBasePath with the planner's own BaseGoalOccupied, from the
%   component that owns that verdict. The rule the paragraph above states is that
%   perception must not decide reachability; the rule here is that when perception has
%   a free choice to make anyway, making it blind is not a virtue.
%
%   RECONSTRUCTION USES THE CALIBRATED RAY PATH, NOT THE RENDER DEPTH. The reason
%   first given - that render depth is quantised to whole metres - turned out to be
%   wrong (cfg.depth.renderQuantum is a measured 59.6 um; see hmmBackprojectDepth),
%   so full-resolution render depth is a possible future source here. Each ray already
%   carries the image pixel it samples (hmmRayDirections' pixelRow/pixelCol) and
%   the world point it struck, so a grounded pixel maps to a metric point by
%   LOOKUP - no projection is re-derived here and no pinhole model is inverted a
%   second time.
%
%   EXTENT IS MEASURED, WITH A DECLARED PROVENANCE. cfg.percep.EXTENT_MEASURED
%   from the calibrated cluster, EXTENT_ACTOR from the discovered registry when
%   too few hits survive, EXTENT_ASSUMED as the last resort. ExtentSource travels
%   on the bus because a downstream grasp has every right to know whether the
%   number it is closing the jaws around was measured or guessed.
%
%   See also hmmGroundingService, hmmRayDirections, hmmDiscoverMovables,
%   hmmStandoffRing, hmmBaseValidators.

%   Copyright 2026 The MathWorks, Inc.

cfg  = cachedConfig();
rays = cachedRays(cfg);

obj = blankObject(objTemplate, cfg, simTime);
plc = blankPlace(plcTemplate, cfg, simTime);

%% Nothing admissible to reconstruct
head = double(queue.Head);
if ~queue.Valid || head < 1 || head > double(queue.Count)
    obj.FailureCode = queue.FailureCode;
    plc.FailureCode = queue.FailureCode;
    return
end
if queue.ItemFailure(head) ~= cfg.fail.OK
    obj.FailureCode = queue.ItemFailure(head);
    plc.FailureCode = queue.ItemFailure(head);
    return
end

verb = queue.Verb(head);
obj.Label       = queue.Label(:, head);
obj.LabelLen    = queue.LabelLen(head);
obj.Confidence  = queue.Confidence(head);
plc.Confidence  = queue.Confidence(head);

%% ---- A HOME ACTION IS RECONSTRUCTED FROM CONFIGURATION, NOT FROM AN IMAGE ----
%   It returns before admitFrame, and it has to: home carries no camera index and no
%   pixel, so PickImage is 0 and the frame admission below would reject it as
%   UnknownCamera - a failure that would be entirely an artefact of asking an
%   image-shaped question about an action that named no image. Every gate from here
%   down is a gate on a MEASUREMENT, and there is no measurement to gate. Nothing is
%   being trusted less: cfg.task.homePose is a surveyed constant, which is strictly
%   better evidence than a reconstructed ray hit.
%
%   THE POSE TRAVELS ON THE EXISTING BUS RATHER THAN AS A SECOND CHART CONSTANT.
%   Position carries [x y 0] and Orientation carries the heading in its yaw element,
%   exactly as a measured object's does, so TaskPlanner's home branch reads its goal
%   off ObjectTarget like every other branch and cfg.task.homePose stays the single
%   place the pose is written. Declaring the pose again inside the chart would be the
%   two-literals-in-two-places failure that cfg.sup exists to prevent.
%
%   The yaw is the whole reason home is not simply a navigate to a known point: every
%   other goal pose in the chart is derived as "face the target", and home is defined
%   by facing AWAY from the nearest wall. See cfg.task.homePose.
if verb == cfg.queue.VERB_HOME
    obj.Position      = [cfg.task.homePose(1:2); 0];
    obj.Orientation   = [cfg.task.homePose(3); 0; 0];
    obj.SourceCamera  = uint8(0);
    obj.PixelPoint    = zeros(2, 1);
    obj.Timestamp     = simTime;
    obj.ActorId       = cfg.actor.noneId;
    obj.NumCandidates = uint8(0);
    obj.Extent        = zeros(3, 1);
    obj.GraspExtent   = 0;
    obj.ExtentSource  = cfg.percep.EXTENT_NONE;
    obj.Valid         = true;
    % As for navigate: PlaceTarget stays invalid with FailureCode OK, meaning "not
    % asked for" rather than "asked for and failed".
    return
end

%% ---- Pick point. Both verbs use the pick slot; see hmmValidateActions. ----
[f, pixel, fail] = admitFrame(frames, queue.PickImage(head), queue.PickPoint(:, head), simTime, cfg);
obj.SourceCamera = queue.PickImage(head);
obj.PixelPoint   = pixel;
if fail ~= cfg.fail.OK
    obj.FailureCode = fail;
    plc.FailureCode = fail;
    return
end
obj.Timestamp = f.Timestamp;

[idx, seed, fail] = nearRays(f, pixel, rays, cfg, cfg.percep.pixelRadius, cfg.percep.minClusterHits);
if fail ~= cfg.fail.OK
    obj.FailureCode = fail;
    plc.FailureCode = fail;
    return
end

P = double(f.Calib.HitLocations(idx, :));
obj.SurfaceId = f.Calib.SurfaceIds(idx(seed));

% Cluster tolerance from the OPTICS, not from a constant. The metric spacing of
% two adjacent samples grows with range: at focal length fx a pixel subtends
% range/fx metres, so a tolerance fixed in metres is too tight far away and too
% loose close up. cfg.percep.clusterTol is retained as the upper cap.
range   = double(f.Calib.HitDistances(idx(seed)));
intr    = cameraIntrinsics(f.Intrinsics(1:2)', f.Intrinsics(3:4)', double(f.ImageSize)');
raySpan = double(cfg.camera.imageSize(2)) / double(cfg.depth.rayGrid(2));
tol     = min(cfg.percep.clusterTol, 1.5 * raySpan * range / intr.FocalLength(1));

[centre, extent, yaw, nInCluster] = clusterMeasure(P, seed, tol);
obj.Position    = centre;
obj.Orientation = [yaw; 0; 0];

if ~inBounds(centre, cfg)
    obj.FailureCode = cfg.fail.OutsideRegion;
    plc.FailureCode = cfg.fail.OutsideRegion;
    return
end

%% ---- A navigate destination is a PLACE, and the two gates below are about OBJECTS
%   THIS IS WHERE THE NAVIGATE VERB USED TO DIE, AND IT DIED SILENTLY. Everything
%   from here to obj.Valid asks two questions that only make sense of a thing to be
%   picked up: does this point belong to exactly one registered movable actor, and
%   would it fit between the jaws. A navigate action's point is a patch of floor. It
%   belongs to no movable actor, so association returned zero candidates and gate 8
%   reported AmbiguousObject; ObjectTarget therefore came back invalid, and
%   TaskPlanner's Dispatch state leaves for Recover on ~ObjectTarget.Valid before it
%   ever reaches NavToPick. The base was never asked for a plan at all - BaseStatus
%   sat in IDLE for the whole run with FailureCode OK, because nothing had failed on
%   the base's side of the interface. Reading that log, the base looked broken and
%   was not: the request never arrived.
%
%   The chart was verb-aware and this function was not. Dispatch already admits
%   VERB_NAVIGATE, already checks PlaceTarget only for a transfer, and NavToPick
%   already leaves for Done rather than Pick when the verb is not a transfer. The one
%   guard that was not verb-aware is the object-validity one, and the honest place to
%   fix it is here, where the verb is known and the reason is nameable, not by
%   loosening a guard in the chart.
%
%   WHAT IS STILL CHECKED FOR A NAVIGATE POINT: the frame is fresh and calibrated
%   (gate 12), a calibrated ray landed near the grounded pixel (gate 6), and the
%   reconstructed point is inside the scene (gate 7). Those are all properties of
%   the MEASUREMENT, and a destination measured from nothing is worthless whatever
%   the verb.
%
%   AND WHAT IS DELIBERATELY NOT CHECKED: whether the base can stand there. That is
%   the base planner's question and it now answers it with five distinct codes, so
%   duplicating a drivability test here would create a second opinion with no map
%   behind it - the same reasoning the header gives for leaving gate 11 to the arm.
%   It also means "drive to the kitchen counter" stays a legal instruction: the
%   destination need not be floor, only somewhere the planner can approach.
if verb == cfg.queue.VERB_NAVIGATE
    obj.ActorId       = cfg.actor.noneId;
    obj.NumCandidates = uint8(0);
    obj.Extent        = zeros(3, 1);
    obj.GraspExtent   = 0;
    obj.ExtentSource  = cfg.percep.EXTENT_NONE;
    obj.Valid         = true;
    % PlaceTarget stays invalid with FailureCode OK, which reads as "not asked for"
    % rather than "asked for and failed".
    return
end

%% Geometric association with exactly one movable actor
[actorId, nCand] = associate(registry, centre, obj.SurfaceId, cfg);
obj.ActorId       = actorId;
obj.NumCandidates = nCand;
if nCand ~= 1
    obj.FailureCode = cfg.fail.AmbiguousObject;
    plc.FailureCode = cfg.fail.AmbiguousObject;
    return
end

%% Extent, with its provenance declared
%   Extent is a FULL span in metres, not a half-extent, on all three axes:
%   [along the major horizontal axis; across the minor one; the object's height].
%
%   WHAT Position(3) ACTUALLY IS, MEASURED: THE SUPPORT SURFACE. Not the object's
%   top face, which is what this contract used to claim. cfg.camera.imageSize is
%   [360 640] and cfg.depth.rayGrid is [72 128], so one ray carries a 5x5 px
%   block, and at the 3 m range of a ceiling camera with focalLength 230 that ray
%   covers 65 x 65 mm of the scene. Mug_Red is 101 x 70 mm. It therefore spans
%   1.55 x 1.07 rays - so a ray CAN land on it now, where at the former [36 64] it
%   spanned 0.77 x 0.54 and could not. cfg.percep.minClusterHits of 3 is still never
%   met and ExtentSource still comes out EXTENT_ACTOR, but for a different reason,
%   measured and recorded at the foot of this block. The cluster whose centroid becomes Position is a patch of the
%   TABLE, and it is a good measurement of one: the table is far larger than a ray
%   cell. Three independent checks put that surface at 0.550 m - Mug_Red's centre
%   0.595 less its half-height, Lamp_Liv's 0.7278 less its own, and the wrist
%   depth image back-projected in a run, 0.549876 - so the field is published as
%   what it measures and both consumers are written against that.
%
%   THE TWO CONSUMERS, and they are why the height must be the WHOLE height:
%   hmmScreenArmPlan takes the support surface as Position(3) directly, and
%   MotionPlanning/TargetSelect grasps at Position(3) + hmmGraspAim(cfg, Extent,
%   GraspExtent)'s rise, and pays the same rise back at release on top of
%   releaseClearance. Get Extent(3) wrong and the grasp height and the release height
%   are wrong by half the error each, in opposite directions.
%
%   THE AIM IS MID-HEIGHT, CLAMPED, and the padDrop term is there because tcp_frame
%   is not the gripping surface. Both the clamp's constants and the drop are
%   properties of the GRIPPER rather than of this reconstruction, so they live in
%   cfg.gripper and cfg.grasp and are documented there; what matters here is only
%   that Extent(3) enters the grasp height at all, so this field is load-bearing on
%   where the jaws close and not merely on a collision box.
%
%   AND Extent(3) IS NO LONGER NECESSARILY WHAT THIS FUNCTION PUBLISHED. When the
%   wrist refinement applies, hmmRefineService replaces element 3 with the height it
%   MEASURED - the top-face plateau's rise above the support plane under it - and the
%   model's RefinedOrCoarseExtent Switch carries that into ObjectTarget. So the
%   reasoning above still holds, but the number it reasons about may have come from a
%   depth frame rather than from the reconstruction below. That is the point: an
%   object the scene manifest does not describe falls back to a flat
%   cfg.percep.extentAssumed guess here, and the wrist look is what replaces the
%   guess with a measurement before the jaws commit to it.
%
%   THE HISTORY IS WORTH KEEPING, because the old reading was self-concealing. When
%   Position(3) was documented as the top face, cfg.plan.graspDepth was subtracted
%   BELOW it, so the commanded grasp came out at 0.530 m - 20 mm under the real
%   tabletop and one full mug height below the object's centre. The arm tracked
%   that to 0.1 mm, so the motion looked perfect. And the same misreading drew the
%   collision box at 0.460 m, 90 mm low, so the screen that should have rejected a
%   grasp inside the table had no table there to reject it against. One wrong
%   sentence in a contract, and the check that would have caught it was computed
%   from the same wrong sentence.
%
%   A CEILING CAMERA STILL CANNOT MEASURE A HEIGHT, so the measured branch below
%   does not try to. Whatever the cluster is standing on, its vertical spread is
%   that surface's own flatness, not a distance down to anything. The height comes
%   from the registry when the actor is known and from cfg.percep.extentAssumed
%   when it is not.
%
%   AND THE MEASURED HORIZONTAL SPANS ARE NOW SUSPECT FOR THE SAME REASON, which is
%   worth writing down rather than leaving as a pleasing symmetry. If the cluster is
%   a patch of table, its two horizontal spans are the TABLE's, not the object's -
%   and Extent(2) becomes GraspExtent, the width the jaws are asked to close across.
%   The branch is not reached in this environment, and the REASON IS NOT THE ONE THIS
%   HEADER USED TO GIVE. It said a finer rayGrid would enter the measured branch. The
%   grid has since been doubled in each axis, to [72 128], and the before and
%   after were measured at the reference pick pixel: the window grew from 6
%   valid rays to 20, but the cluster stayed at ONE hit at both densities, so
%   ExtentSource is EXTENT_ACTOR at both. What actually bounds it is the cluster
%   TOLERANCE against the ray SPACING. Two adjacent rays are raySpan * range / fx
%   apart: at the reference pick's 2.42 m that is 52.6 mm with raySpan 5, against
%   cfg.percep.clusterTol of 45 mm. Single-linkage growth cannot join two neighbours
%   it cannot reach, so nInCluster is 1 however many rays land nearby, and the measured
%   branch stays unreachable beyond range = clusterTol * fx / raySpan = 2.07 m. Every
%   ceiling camera in this apartment stands further off than that.
%
%   SO THE THING TO WATCH IS clusterTol, NOT rayGrid. Raising the ray count is safe for
%   the pick; raising clusterTol past the ray spacing, or moving a camera closer than
%   2.07 m, is what would enter the measured branch - and would then need the cluster
%   segmented from the surface it sits on before those spans mean anything.
%
%   A SECOND CORRECTION FROM THE SAME MEASUREMENT, because it undercuts the paragraph
%   above about Position(3). At the reference pick pixel the seed does NOT land on the
%   table: it lands 81.5 mm above the local surface at the old density and 56.5 mm
%   above it at the new one, i.e. on the object itself both times, while the modal
%   height in the window - the surface - sits at 0.400 m. So "Position(3) is the
%   support surface" is true of the cases that motivated it and is NOT true here, and
%   the pick nonetheless succeeds. What the field really publishes is the height of
%   whatever the seed ray struck, which is the surface when the object is invisible to
%   the rays and the object when it is not. Both consumers tolerate the difference
%   today because the object is short; neither is written against it.
%
%   THAT IS ALSO THE HONEST LIMIT OF THIS CONTRACT, stated plainly. "Position(3) is
%   the support surface" is not a geometric truth about the field; it is a
%   consequence of the object being smaller than a ray cell, so that every hit in
%   the cluster comes off the surface around it. An object large enough to fill a
%   ray cell would put hits on its OWN top face, and Position(3) would drift from
%   the support surface up toward that face - at which point the grasp, computed off
%   Position(3), would ride high by however far it drifted, up to a full Extent(3)
%   for an object that filled the cell completely. Nothing movable in
%   this apartment is remotely that large, so the reading holds throughout; a
%   sharper camera or a bigger object is what would break it, and the fix then is
%   segmentation, not a different formula. The xy is robust either way, since a
%   centroid over the object and a centroid over the surface patch around it are the
%   same point to within the ray spacing.
regCol = find(registry.ActorId(1:double(registry.Count)) == actorId, 1);
if nInCluster >= cfg.percep.minClusterHits
    obj.Extent       = extent;
    obj.ExtentSource = cfg.percep.EXTENT_MEASURED;
    if registry.Valid && ~isempty(regCol)
        obj.Extent(3) = registry.Extent(3, regCol);
    else
        obj.Extent(3) = cfg.percep.extentAssumed;
    end
elseif registry.Valid && ~isempty(regCol)
    obj.Extent       = registry.Extent(:, regCol);
    obj.ExtentSource = cfg.percep.EXTENT_ACTOR;
else
    obj.Extent       = cfg.percep.extentAssumed * ones(3, 1);
    obj.ExtentSource = cfg.percep.EXTENT_ASSUMED;
end

% The jaws close across the MINOR horizontal axis, which is what
% clusterMeasure' second extent component reports and what yaw is aligned with.
obj.GraspExtent = obj.Extent(2);

% Position(3) IS MADE THE SUPPORT SURFACE HERE, which closes the limit stated above. The seed ray
% strikes whatever is tallest under the pixel, so for an object taller than a ray cell the cluster
% height is the OBJECT, not the surface: the 188 mm Bottle came out at 0.919 m on a 0.800 m chest,
% TargetSelect added the grasp rise on top, and the jaws were aimed at 1.025 m - 37 mm above the
% bottle's own top - which the arm could only reach near-singular (2026-10-12). hmmSupportHeight
% reads the surface under the point from the engine's nadir sweep, the same one the place slide
% and the release settle use; with no swept surface there it leaves the value as measured.
% Association above keeps the raw centre, which is what it was tuned on.
obj.Position(3) = hmmSupportHeight(obj.Position);

strokeMax = cfg.gripper.maxOpenPos - cfg.gripper.apertureTol;
if obj.GraspExtent <= 0 || obj.GraspExtent > strokeMax
    obj.FailureCode = cfg.fail.NotGraspable;
    plc.FailureCode = cfg.fail.NotGraspable;
    return
end

obj.Valid = true;

%% ---- Place point, transfer only ----
%   navigate returned above, so reaching here means the verb is a transfer. The check
%   survives as a guard rather than a live branch: a verb added later that arrives
%   without a place point would publish a valid object and an invalid place, which
%   Dispatch reads as "no surface to place on" - a wrong-but-safe answer, where
%   falling through would fit a plane to whatever the zeroed place slot pointed at.
if verb ~= cfg.queue.VERB_TRANSFER
    return
end

[fp, pixelP, fail] = admitFrame(frames, queue.PlaceImage(head), queue.PlacePoint(:, head), simTime, cfg);
plc.SourceCamera = queue.PlaceImage(head);
plc.PixelPoint   = pixelP;
if fail ~= cfg.fail.OK
    plc.FailureCode = fail;
    return
end
plc.Timestamp = fp.Timestamp;

% A plane fit needs a wide window: cfg.percep.windowTargetHits hits at the ray
% grid pitch cover far more image area than the association radius does. That target
% is a window-SIZING figure and not the inlier gate - see its note in hmmConfig, and
% note that the range-aware count derived from supportMinSpan cannot be used here
% because it needs the seed's range, which is what this call establishes.
placeRadius = 3 * cfg.percep.pixelRadius;
[idxP, seedP, fail] = nearRays(fp, pixelP, rays, cfg, placeRadius, cfg.percep.windowTargetHits);
if fail ~= cfg.fail.OK
    plc.FailureCode = fail;
    return
end

% SECOND PASS, SIZED IN METRES RATHER THAN PIXELS. The window above is a pixel
% count, but what the fit has to certify is a REGION - a disc of
% regionHalfMax + surfaceInset around the place point - and a pixel radius covers
% a metric span that shrinks with range. At this scene's 1.7 m ceiling standoff a
% 72 px window reaches only 0.26 m, so RegionHalfSize came back describing the
% edge of the WINDOW instead of the edge of the counter: the region was reported
% as a few millimetres wide on a surface with a metre of room, and the place was
% rejected as unsupported. Asking for the span the region needs makes the answer a
% property of the surface. The first pass is what supplies the range to convert
% with, which is why this cannot be done in one.
wanted = (cfg.percep.regionHalfMax + cfg.percep.surfaceInset) * ...
    double(cfg.camera.focalLength(1)) / double(fp.Calib.HitDistances(idxP(seedP)));
if wanted > placeRadius
    placeRadius = wanted;
    [idxP, seedP, fail] = nearRays(fp, pixelP, rays, cfg, placeRadius, cfg.percep.windowTargetHits);
    if fail ~= cfg.fail.OK
        plc.FailureCode = fail;
        return
    end
end

% obj.Extent travels into the fit because the surface HEIGHT is a local question - see
% fitSupport's note on the two-level counter. The window above certifies the plane; the
% object's own footprint decides which of that window's hits the object will rest on.
plc = fitSupport(plc, fp, idxP, seedP, placeRadius, cfg, obj.Extent);

end

%% ------------------------------------------------------------------------
function [f, pixel, fail] = admitFrame(frames, camIdx, normPt, simTime, cfg)
%ADMITFRAME Fetch a frame, check it is fresh and calibrated, denormalise the pixel.
f     = frames(1);
pixel = zeros(2, 1);
fail  = cfg.fail.OK;

c = double(camIdx);
if c < 1 || c > double(cfg.camera.nFixed)
    % hmmValidateActions already gates this; repeated here because a queue can
    % also arrive from a replay corpus recorded against a different rig.
    fail = cfg.fail.UnknownCamera;
    return
end
f = frames(c);

% THE ARRAY POSITION IS NOT THE CAMERA. CameraFrame carries CameraIndex because
% the frames array is assembled by a Vector Concatenate whose input ORDER is a
% wiring decision, and a transposed pair there is invisible: a pixel grounded in
% one room is looked up in another room's ray scan, the lookup finds perfectly
% valid hits on somebody else's floor, and the run reports AmbiguousObject as
% though the provider were at fault. Checking that the frame agrees about which
% camera it is converts that silent wrong answer into a reported failure.
if f.CameraIndex ~= camIdx
    fail = cfg.fail.UnknownCamera;
    return
end

if ~f.Valid
    fail = cfg.fail.Stale;
    return
end
if simTime - f.Timestamp > cfg.safety.staleTimeout
    fail = cfg.fail.Stale;
    return
end
if f.DepthSource ~= cfg.depth.SOURCE_RAYTRACE
    % Render depth cannot support a grasp; see the file header.
    fail = cfg.fail.DepthInvalid;
    return
end

imgRows = double(f.ImageSize(1));
imgCols = double(f.ImageSize(2));
scale   = cfg.camera.pixelNormScale;
pixel   = [min(max(normPt(1) / scale * imgRows, 1), imgRows); ...
           min(max(normPt(2) / scale * imgCols, 1), imgCols)];
end

%% ------------------------------------------------------------------------
function [idx, seed, fail] = nearRays(f, pixel, rays, cfg, radius, minHits)
%NEARRAYS Valid calibrated hits whose sampled pixel is near a grounded pixel.
%   The window GROWS, up to four times the requested radius, until minHits valid
%   hits are inside it. A fixed window would report DepthInvalid for a perfectly
%   good pixel that happens to fall between ray grid centres, which is a
%   sampling artefact of the ray grid rather than a fact about the scene. The
%   growth is bounded so a genuinely empty region still fails.
%
%   minHits IS A TARGET, NOT A THRESHOLD, and reading it as a threshold is what let
%   one config name mean two things for a while. This function can report exactly one
%   failure - DepthInvalid, code 6 - and only for a window with no valid hits at all:
%   at the 4x growth it returns however few it found, because one valid hit is still a
%   metric measurement of where the surface is. Whether those hits are COPLANAR, and
%   how many coplanar ones a place needs, is fitSupport's question and reports code 10.
%   Callers pass cfg.percep.windowTargetHits for the place window and
%   cfg.percep.minClusterHits for the object window; neither is the inlier gate.
idx  = [];
seed = 1;
fail = cfg.fail.DepthInvalid;

valid = f.Calib.IsValidHit;
if ~any(valid)
    return
end

dRow = double(rays.pixelRow) - pixel(1);
dCol = double(rays.pixelCol) - pixel(2);

for growth = [1 2 4]
    r = radius * growth;
    inWindow = valid & abs(dRow) <= r & abs(dCol) <= r;
    if sum(inWindow) >= minHits
        idx = find(inWindow);
        break
    end
    if growth == 4 && any(inWindow)
        % Fewer than minHits but not nothing: return what there is and let the
        % caller decide. A single valid hit is still a metric measurement of
        % where the surface is, it just is not a measurement of how big it is.
        idx = find(inWindow);
    end
end

if isempty(idx)
    return
end

d2 = dRow(idx).^2 + dCol(idx).^2;
[~, seed] = min(d2);
fail = cfg.fail.OK;
end

%% ------------------------------------------------------------------------
function [centre, extent, yaw, n] = clusterMeasure(P, seed, tol)
%CLUSTERMEASURE Grow a surface cluster from the seed hit and measure it.
%   Single-link growth: a hit belongs to the cluster when it is within tol of any
%   hit already in it. Linking to the cluster rather than to the seed alone is what
%   lets an elongated object be measured end to end instead of being cut off at
%   a radius around the grounded pixel.
%
%
%   NOT pcsegdist, AND THAT WAS MEASURED (2026-10-08). It looks like the toolbox form of
%   this loop, but on 300 random clouds it returned a different seed cluster 23 times,
%   always SMALLER - dropping up to 69 of 113 points the single-link growth joins - which
%   would silently shrink the measured extent. The loop is the definition; keep it.
n = size(P, 1);
inC = false(n, 1);
inC(seed) = true;

for iter = 1:n
    if all(inC)
        break
    end
    % Pairwise distances, in base MATLAB rather than pdist2, which would make the whole
    % example need Statistics and Machine Learning Toolbox for this one line.
    d = sqrt(sum((permute(P(inC, :), [1 3 2]) - permute(P(~inC, :), [3 1 2])) .^ 2, 3));
    joins = min(d, [], 1) <= tol;
    if ~any(joins)
        break
    end
    outIdx = find(~inC);
    inC(outIdx(joins)) = true;
end

Q = P(inC, :);
n = size(Q, 1);
centre = mean(Q, 1)';

% Horizontal principal axes. Only the top face of an object is visible to a
% ceiling camera, so the measurement that matters is in the world XY plane -
% which is also the plane the jaws close in for a top-down approach.
XY = Q(:, 1:2) - mean(Q(:, 1:2), 1);
if n >= 2
    [~, ~, V] = svd(XY, 0);
    major = V(:, 1);
else
    major = [1; 0];
end

% THE SIGN IS PINNED, because svd does not pin it. A singular vector and its
% negative are both valid returns and LAPACK picks between them on grounds that
% have nothing to do with the object, so an unpinned axis makes the published yaw
% flip by 180 degrees between runs on the same scene. A symmetric two-finger
% gripper does not care - which is why this never showed up - but anything a
% caller expresses in the tool's lateral plane does, and hmmCarryTranslation's
% lateral term now is one.
%
% The convention is: major is signed so that its LARGEST-MAGNITUDE component is
% positive, and the minor axis is major rotated by +90 degrees so the pair is
% right-handed. Chosen so it REPRODUCES the degenerate fallback exactly - major
% [1;0] gives minor [0;1] and yaw +pi/2 - which is what keeps a working run working.
%
% IT IS THE LARGEST COMPONENT AND NOT x FOR A REASON. "Point major into the x > 0
% half-plane" looks like the same rule and is not: for an axis that lies nearly
% along y the sign of major(1) is measurement noise, so that rule flips with the
% noise and pins nothing. Measured on 200 synthetic y-elongated clusters it left the
% yaw scattered across both branches; keying on the dominant component decides on a
% number that is far from zero by construction.
[~, dom] = max(abs(major));
if major(dom) < 0
    major = -major;
end
minor = [-major(2); major(1)];

% The third component is the vertical spread of the TOP-FACE cluster - the
% face's flatness and tilt, NOT the object's height, which nothing looking down
% at an object can see. It is reported for its diagnostic value only; the caller
% replaces it with a sourced height before publishing Extent.
extent = [spanAlong(XY, major); spanAlong(XY, minor); max(Q(:, 3)) - min(Q(:, 3))];

% Yaw of the MINOR axis: the jaws must open across the narrow dimension, so the
% jaw opening axis is aligned with the minor axis, not the major one.
yaw = atan2(minor(2), minor(1));
end

function s = spanAlong(XY, axis)
p = XY * axis;
s = max(p) - min(p);
end

%% ------------------------------------------------------------------------
function [actorId, nCand] = associate(registry, point, surfaceId, cfg)
%ASSOCIATE Match a reconstructed point to exactly one movable actor.
%   Two routes, tried in order. A SurfaceId match is exact: the ray tracer
%   reports which rendered surface it struck and the registry records which
%   surface each movable actor owns, so agreement is identity, not inference.
%   Geometric proximity is the fallback, and it must WIN CLEARLY - the nearest
%   actor has to beat the runner-up by cfg.percep.assocMargin. Without that
%   margin two adjacent objects would each be reported as a confident single
%   match depending on millimetres of reconstruction noise.
%
%   ZERO IS "NO TAG", NOT A TAG, ON BOTH SIDES OF THE COMPARISON. The manifest
%   need not supply surface identifiers, and hmmDiscoverMovables leaves the
%   registry column zero when it does not; the ray tracer likewise reports zero
%   for a surface it cannot name. Comparing those zeros for equality is the worst
%   possible outcome, because it does not fail - it succeeds against EVERY
%   untagged actor at once. The whole registry comes back as candidates, nCand is
%   twenty rather than one, and the run reports AmbiguousObject about an object
%   that was reconstructed to within six centimetres of the right prop. Requiring
%   a non-zero identifier is what makes the geometric route the documented
%   fallback instead of unreachable code.
actorId = cfg.actor.noneId;
nCand   = uint8(0);

count = double(registry.Count);
if ~registry.Valid || count < 1
    return
end

if surfaceId ~= 0
    surfMatch = find(registry.SurfaceId(1:count) == surfaceId);
    if ~isempty(surfMatch)
        nCand   = uint8(numel(surfMatch));
        actorId = registry.ActorId(surfMatch(1));
        return
    end
end

d = vecnorm(registry.Position(:, 1:count) - point, 2, 1);
within = find(d <= cfg.percep.assocRadius);
if isempty(within)
    return
end

[dSorted, order] = sort(d(within));
if numel(within) > 1 && (dSorted(2) - dSorted(1)) < cfg.percep.assocMargin
    % Ambiguous, and reported as such: two actors are equally good matches.
    nCand   = uint8(numel(within));
    actorId = cfg.actor.noneId;
    return
end

nCand   = uint8(1);
actorId = registry.ActorId(within(order(1)));
end

%% ------------------------------------------------------------------------
function plc = fitSupport(plc, f, idx, seed, radius, cfg, objExtent)
%FITSUPPORT Fit the support surface at a place point and size its admissible region.
%   pcfitplane (Computer Vision Toolbox) does the fit. Candidate hits are first
%   restricted to the SAME SURFACE as the seed, so a table edge does not get
%   fitted jointly with the floor behind it and produce a plane through neither.
%
%   THAT RESTRICTION CANNOT RELY ON THE SURFACE TAG ALONE. The ray tracer reports
%   zero for a surface it cannot name, and for this scene it reports zero for
%   every hit, so comparing tags put every hit in one group and the guard above
%   was not in force. What that costs is not a near miss: a window straddling the
%   counter's edge held 23 hits on the counter at 0.90 m and 20 on the floor
%   behind it, RANSAC returned the FLOOR - a perfectly level plane, fitted to real
%   measurements, three metres from anything the operator pointed at - and the
%   place was refused for having too few inliers. So the tag is used when it says
%   something and a SLAB about the seed stands in for it when it does not.
%
%   The slab thickness is derived, not chosen. A support surface is allowed to tilt
%   by planeMaxTilt, so across the window's own metric half-span it may legitimately
%   rise by span*tan(tilt); planeMaxDistance allows for its roughness on top of
%   that. A hit outside that band cannot belong to the seed's plane whatever else
%   it belongs to.
%
%   THE PLANE DECIDES THE TILT AND THE REGION. IT IS THE WRONG ESTIMATOR FOR THE
%   HEIGHT, and that cost 6.4 mm on this scene's own kitchen counter. The window has
%   to be wide - planeMinInliers hits, and a metric span covering the region it
%   certifies, which here is a disc 468 mm across. But real furniture is not one
%   plane over that span: this counter has a main surface at z = 0.9000 and a raised
%   strip at z = 0.9100 running along one side, 10 mm apart, well inside
%   planeMaxDistance so RANSAC accepts both as one plane and returns the average.
%   The place point sits on the LOWER surface - the nearest hit to it reads 0.9000
%   and the nearest upper hit is 55 mm away - and the fit put the surface at 0.9064.
%   An object set down on a surface 6.4 mm lower than the model believes hangs in
%   mid air by 6.4 mm, which is a third of what this scene's release clearance was
%   being blamed for.
%
%   So the height is taken LOCALLY: the inliers within the object's own footprint of
%   the place point, median of their offset from the fitted plane, applied as a shift
%   of the plane along its normal. The median rather than the mean because the point
%   of the estimate is to reject the other level rather than to average it in, and a
%   footprint-sized window can easily be a third strip. The plane's NORMAL is left
%   alone - a tilt fitted to a handful of hits is worse than one fitted to a hundred,
%   and the tilt was never the thing that was wrong. Below a handful of local hits
%   the full-window plane stands, because an estimate from two samples is not an
%   improvement on an estimate from eighty.
%
%   AND THE POINT THE FIT CHOOSES IS NOT NECESSARILY A POINT THE BASE CAN STAND AT,
%   WHICH IS THE OTHER THING THIS FUNCTION NOW DECIDES. The grounding pixel lands
%   wherever the provider aimed it, and on a deep surface that can be well back from
%   the edge the robot approaches from. The shipped kitchen place point sits 128 mm
%   inside the unit's own floor footprint, so of the 0.660 m cfg.sup.placeStandoff of
%   the day only 0.532 m was clearance from the obstacle - against a footprint whose
%   geom.forward was 0.565 m. It fitted, with 27 mm to spare, until the stow pose went
%   vertical and geom.forward grew by 60 mm; then every one of the 24 stand-off
%   bearings around that point was occupied, the place leg failed BaseGoalOccupied
%   after a successful pick, and the held mug was dropped. A run can lose a task to
%   a place point that is 128 mm from a perfectly good one on the same surface.
%
%   THOSE THREE FIGURES ARE ALL SUPERSEDED AND THE MECHANISM IS NOT, which is why they
%   are left standing rather than refreshed in place: they are the measurement that
%   motivated this code and none of them describes the shipped robot any more.
%   cfg.sup.placeStandoff is 0.480 m, and geom.forward is no longer the figure a
%   stand-off is compared against at all - hmmStandoffRing chooses the arrival heading
%   now, and a base parked ACROSS the approach presents geom.inflation (0.290 m
%   empty-handed, 0.314 m carrying the shipped mug) toward the surface rather than
%   geom.forward (0.502 m). What survives unchanged is the claim this function acts on:
%   that a point the fit chose can be unserviceable while a point on the same surface
%   is fine, and that the map is the only thing that can tell them apart.
%
%   AND IT IS WORTH RECORDING THAT THE BIAS CORRECTLY DID NOTHING ON THE RUN THAT FAILED
%   NEXT, because that is the case this code is most likely to be blamed for. On the 40 s
%   run the fit put the place point at the region centre, biasToReachable found no
%   candidate serviceable, and the point was published unmoved - and it was right to:
%   sweeping the WHOLE admissible strip returns zero free bearings at every radius. The
%   cause was not the point. cfg.sup.arrivalLift was over-estimating how high the tool
%   would be held by 31 mm, the lift indexes a lever CEILING, and 31 mm of padding is
%   31 mm off the admissible stand-off - which closed the reach window below the map
%   window everywhere on that counter. See hmmArmLeverBand. A serviceability search can
%   only report that nothing is serviceable; it cannot tell you that the question was
%   being asked about the wrong robot.
%
%   SO THE POINT IS MOVED WITHIN THE REGION THIS FUNCTION ALREADY CERTIFIES, and the
%   choice is made against the MAP rather than against the surface's shape. A shape
%   heuristic is not enough and the measurement says so: at the shipped place point
%   the nearer observed edge is the one along v, 128 mm away against 232 mm along u,
%   so "move toward the nearest edge" picks v - and every bearing stays occupied.
%   Moving the full 182 mm along +u instead opens 6 of the 24. The direction that
%   matters is the one with floor beyond it, which is a fact about the apartment and
%   not about the counter, so biasToReachable asks hmmStandoffRing the same question
%   hmmSelectBaseGoal asks, with the same validator and the same absolute bearing
%   grid. Nothing here can invent a place: a point the fit chose is kept whenever it
%   is already serviceable, the offset is bounded by the region minus the object's
%   own footprint, and when no candidate is serviceable the fit's own point is
%   published unchanged so the failure stays the planner's named BaseGoalOccupied
%   rather than a silent substitution.
%
%   u AND v ARE NOT x AND y, AND GUESSING WHICH IS WHICH WILL WASTE A SESSION. planeBasis
%   picks the axis the normal is SMALLEST along and crosses from there, so for a level
%   surface - n = [0 0 1], every work surface in this apartment - it selects a = [1 0 0],
%   giving u = n x a = +y and v = n x u = -x. The shipped kitchen counter therefore has
%   RegionHalfSize = [0.3580 0.0635] meaning 358 mm ALONG y and 63 mm along x: the strip
%   runs the length of the room, not across it. Net of the object's own footprint and the
%   inset that leaves roomU = +/-0.3075 m of travel along y and roomV = +/-0.0130 m along
%   x, so the candidate fan is very nearly one-dimensional here and sweeping x is
%   measuring an axis this function cannot move along.
%   EVERY EARLY RETURN BELOW REPORTS THE SAME FAILURE CODE, so each one also writes the
%   fields it already knows. plc.FailureCode is set pessimistically here and cleared only on
%   success at the end, which is correct - a half-filled place target must never read OK -
%   but it means all six rejections arrive on the bus as PlaceUnsupported with NumInliers,
%   PlaneNormal, RegionCenter and RegionHalfSize left at their defaults, and a field
%   diagnosis of which gate fired becomes impossible without an instrumented re-run. That
%   cost a session. So NumInliers now reports the count that was actually available at
%   the point of rejection, and the later gates publish the geometry they computed.
%   Read the bus as:
%
%     NumInliers 0-2,  PlaneNormal [0 0 1]      too few candidate hits; the fit never ran
%     NumInliers >= 3, PlaneNormal [0 0 1]      pcfitplane threw, or returned too few inliers
%     NumInliers >= 3, PlaneNormal set,
%                      RegionHalfSize [0 0]     the fitted plane is too steep to place on
%     NumInliers >= 3, PlaneNormal set,
%                      RegionHalfSize <= 0      real surface, but the point is within one
%                                               inset of its edge - the negative value is
%                                               how far inside the inset it sits
%
%   A negative RegionHalfSize is therefore diagnostic, not a corrupt value, and it only ever
%   appears alongside a non-OK FailureCode.
plc.FailureCode = cfg.fail.PlaceUnsupported;

seedPt   = double(f.Calib.HitLocations(idx(seed), :))';
seedDist = double(f.Calib.HitDistances(idx(seed)));
if f.Calib.SurfaceIds(idx(seed)) ~= 0
    same = idx(f.Calib.SurfaceIds(idx) == f.Calib.SurfaceIds(idx(seed)));
else
    span = radius * seedDist / double(cfg.camera.focalLength(1));
    slab = cfg.percep.planeMaxDistance + span * tan(cfg.percep.planeMaxTilt);
    same = idx(abs(double(f.Calib.HitLocations(idx, 3)) - seedPt(3)) <= slab);
end
if numel(same) < 3
    plc.NumInliers = uint16(numel(same));
    return
end

P  = double(f.Calib.HitLocations(same, :));
pc = pointCloud(P);

try
    [model, inlierIdx] = pcfitplane(pc, cfg.percep.planeMaxDistance, [0 0 1], ...
        rad2deg(cfg.percep.planeMaxTilt));
catch
    % The candidates, not the inliers: the fit produced none. NumInliers >= 3 with a default
    % normal is what separates this from the gate above.
    plc.NumInliers = uint16(min(numel(same), double(intmax('uint16'))));
    return
end

% HOW MANY INLIERS THIS RANGE CAN ACTUALLY DELIVER. cfg.percep.planeMinInliers is an
% absolute count sampled on a grid of fixed ANGULAR pitch, so it silently asks for a bigger
% surface the further away the measurement was taken - 0.53 m across at 2.5 m, 0.64 m at
% 3.0 m, against a worktop ~0.6 m deep seen through a window 0.90 m across that must
% therefore straddle its edges. See the note at cfg.percep.supportMinSpan. The requirement
% is restated as a metric width and converted back to a ray count here, and CAPPED at the
% configured count so this can only relax the gate: a place that succeeds today cannot
% start failing because of this line.
rayPitch   = double(cfg.camera.imageSize(1)) / double(cfg.depth.rayGrid(1));
raysAcross = cfg.percep.supportMinSpan * double(cfg.camera.focalLength(1)) / ...
    (max(seedDist, eps) * rayPitch);
minInliers = min(cfg.percep.planeMinInliers, ...
    max(cfg.percep.planeMinInliersFloor, ceil(raysAcross ^ 2)));

if numel(inlierIdx) < minInliers
    plc.NumInliers = uint16(min(numel(inlierIdx), double(intmax('uint16'))));
    return
end

n = model.Normal(:);
n = n / norm(n);
if n(3) < 0
    n = -n;   % report an upward normal; a support surface faces up by definition
end
if acos(min(max(n(3), -1), 1)) > cfg.percep.planeMaxTilt
    plc.NumInliers  = uint16(min(numel(inlierIdx), double(intmax('uint16'))));
    plc.PlaneNormal = n;
    return
end

Q = P(inlierIdx, :);

% The place point is the seed hit PROJECTED ONTO THE FITTED PLANE rather than
% the raw hit. The fit averages many samples, so the plane is a better estimate
% of the surface height than any single ray return on it.
[~, nearest] = min(vecnorm(Q' - seedPt, 2, 1));
raw = Q(nearest, :)';
d   = model.Parameters(4);
pt  = raw - (n' * raw + d) * n;

if ~inBounds(pt, cfg)
    plc.FailureCode = cfg.fail.OutsideRegion;
    return
end

[u, v] = planeBasis(n);
duv = [(Q - pt') * u, (Q - pt') * v];

% ---- The admissible region, sized BEFORE anything is allowed to move inside it.
%
% Symmetric about the fitted point, so the region genuinely contains it: the
% smaller of the two directions bounds each half-extent.
%
% THIS USED TO BE COMPUTED AFTER THE LOCAL HEIGHT AND IT HAD TO COME FORWARD, because
% the region is now what bounds the bias below and the bias is what the local height
% has to be measured at. The arithmetic is untouched - duv is measured from the fitted
% point either way and u and v are orthogonal to n, so nothing here depends on the
% order - but the region has to exist before there is a box to move the point inside.
halfU = min(max(duv(:, 1)), -min(duv(:, 1))) - cfg.percep.surfaceInset;
halfV = min(max(duv(:, 2)), -min(duv(:, 2))) - cfg.percep.surfaceInset;
halfU = min(halfU, cfg.percep.regionHalfMax);
halfV = min(halfV, cfg.percep.regionHalfMax);

if halfU <= 0 || halfV <= 0
    % The surface is real but the place point sits within one inset of its edge,
    % so nothing can be set down there without overhanging. The measured half-extents are
    % published even though they are not positive: the negative value says how far inside
    % the inset the point sits, which is the difference between "aimed at the edge" and
    % "aimed at something 2 cm wide", and no consumer reads these while FailureCode is set.
    plc.NumInliers     = uint16(min(numel(inlierIdx), double(intmax('uint16'))));
    plc.PlaneNormal    = n;
    plc.RegionCenter   = pt;
    plc.RegionHalfSize = [halfU; halfV];
    return
end

% ---- Where in that region the object actually goes. See the header paragraph on
% the place point the base cannot stand at.
%
% regionCentre is kept because the two bus fields now differ: RegionCenter reports
% where the SURFACE was certified and Point reports where the OBJECT goes, which is
% what the bus has always said they mean and what made them two fields.
regionCentre    = pt;

% ---- Can the arm work at this height from ANY base pose? -----------------------
% THIS REFUSES BEFORE THE BIAS RATHER THAN AFTER IT, because no in-plane shift can help: the
% lever band depends on the target's HEIGHT and the held object, never on where the base
% stands, so a band that is empty here is empty for all 576 of the bearing-by-heading
% candidates biasToReachable is about to score. Without this the zero scores read as "no
% bearing is free", the point is published unchanged, the base drives to it, and
% hmmPlanBasePath reports BaseGoalOccupied - which names the map, and the map is innocent.
% cfg.fail.PlaceUnreachable carries the full account of what that misattribution cost.
%
% SCREENED AT THE SUPPORT HEIGHT, WHICH IS THE CONSERVATIVE END. The plan must solve
% cfg.sup.approachDepth above this as well, and a higher target has a smaller band, so empty
% at the surface guarantees empty at the approach. The error this can make is therefore
% refusing late, never refusing wrongly - the opposite of the other lever-band gates, which
% screen the surface and then let a plan be attempted at a height they never checked.
%
% AND IT RUNS EVEN UNDER cfg.map.mode = 'live', unlike the bias below, because it needs no map.
% The band comes from cfg.arm.leverCeiling and cfg.pedestal.armMountHeight; there is nothing to
% look up and no reason for a too-high shelf to be refused honestly in one map mode only.
%
% IT IS NOT THE ONLY PlaceUnreachable GATE AND IT CANNOT BE. This one catches an EMPTY band; a
% band that is merely too SHORT for any radius the map allows passes here and is caught after
% the local-height correction below, on the point that actually ships. The two are kept apart
% rather than merged because only this one works without a map.
[loLever, hiLever] = hmmArmLeverBand(cfg, pt(3), double(objExtent(:)));
if hiLever < loLever
    plc.FailureCode = cfg.fail.PlaceUnreachable;
    return
end

[pt, shiftUV]   = biasToReachable(pt, u, v, halfU, halfV, cfg, objExtent, Q, duv, n);
duv(:, 1)       = duv(:, 1) - shiftUV(1);
duv(:, 2)       = duv(:, 2) - shiftUV(2);
if numel(shiftUV) > 2 && shiftUV(3) ~= 0
    d = -n' * pt;                     % the plane through the point's new height
end

if ~inBounds(pt, cfg)
    plc.FailureCode = cfg.fail.OutsideRegion;
    return
end

% ---- Local height. See the header paragraph on the two-level counter, and levelToLocal.
%
% IT RUNS AFTER THE BIAS, NOT BEFORE IT, which is the whole reason the bias was put where it
% is. This counter has two levels 10 mm apart; a height measured at the fitted point and then
% applied to a point 182 mm away is a height measured on the wrong strip, which is the 6.4 mm
% error this block exists to remove reintroduced by the fix for something else. duv was rebased
% onto the chosen point above so the window is centred where the object will actually rest.
%
% AND THE BIAS NOW APPLIES THE SAME LEVELLING TO EVERY CANDIDATE BEFORE SCORING IT, so this
% line reproduces a height the bias has already seen rather than revealing one to it. That
% ordering cost a run before it was true - see levelToLocal.
%
% duv IS UNAFFECTED BY THIS SHIFT, so the region extents already reported need no
% redoing: u and v are
% orthogonal to n by construction, so moving pt along n leaves every in-plane coordinate
% exactly where it was. The plane OFFSET does have to move with it, because a consumer
% that keeps the normal and the offset would otherwise get the surface the fit returned
% rather than the one this function is reporting.
[pt, levelShift] = levelToLocal(pt, Q, duv, n, cfg, objExtent);
if levelShift ~= 0
    d = -n' * pt;
end

% ---- Can the base stand anywhere that serves the point being published? --------------
%
% NOT ONE OF THE NUMBERED GATES. Gates 1-12 are the perception taxonomy and 7 and 8 are
% already OutsideRegion and association; this raises cfg.fail.PlaceUnreachable (33), which
% sits outside that series precisely because it is a statement about the robot rather than
% about the scan.
%
% THE LEVER-BAND GATE ABOVE IS NOT ENOUGH, AND A RUN PROVED IT. That one refuses only an
% EMPTY band - a surface so high that no rung of the arm's lever table exists. On the counter
% the provider chose for one transfer the band was [0.130, 0.552] m: not empty, just too
% short. At z = 0.910 m holding the mug the arm reaches 0.552 m of lever, while the occupancy
% map will not let the base closer than ~0.72 m to that worktop - every radius the map accepts
% is past what the arm can reach and every radius the arm can reach is inside the counter. The
% window is empty by ~0.28 m and no stand-off, heading or arrival tolerance recovers it. The
% band gate passed, the base drove to the kitchen, the mug was picked, the place goal was
% dispatched at t = 11.92 s and hmmPlanBasePath refused the goal POSE 0.02 s later with
% BaseGoalOccupied (27) - a code about one occupied cell, for a condition that was true of the
% whole surface before the robot moved. This is the same substitution the band gate above
% removed for the empty-band case, one case over.
%
% IT SCREENS THE FINAL POINT, AFTER THE LOCAL-HEIGHT CORRECTION, AND THAT PLACEMENT IS THE
% MEASUREMENT. biasToReachable scores candidates too, and on that run it did find one - a
% single free bearing out of 24 at the fitted z = 0.900. The correction above then levelled
% the chosen point onto the counter's upper strip at 0.910, the lever ceiling fell 0.567 to
% 0.552 m, and that one bearing was gone. Gating on what the bias knew would have passed this
% run; gating here, 15 mm of ceiling later, refuses it. The published point is the only point
% the rest of the run will ever see, so it is the only one worth asking about.
%
% IT ASKS THE PLANNER'S OWN QUESTION, which is why it cannot refuse a place that would have
% worked: freeBearings is hmmStandoffRing against hmmBaseValidators' driving validator with
% the carried extent and the arm's lever band - bearing for bearing what hmmSelectBaseGoal
% applies when the goal is dispatched. If any arrival on the ring passes, so does this.
%
% AND IT IS SKIPPED WHEN THERE IS NO MAP, the same carve-out biasToReachable has always had.
% Under cfg.map.mode = 'live' cachedNavGrid returns empty, against which every cell is unknown
% and every bearing would fail, so refusing on it would break every live-mapping run before
% the cameras have built anything.
%
% THE COST IS ONE RING EVALUATION, MEASURED AT 13.7 ms, against a 0.100 s perception tick and
% a surface fit that costs far more. The validator is the expensive part and it is memoised on
% the carried extent, so on a serviceable surface - where biasToReachable returns after its own
% first ring - this is the only extra work the gate adds.
navGrid = cachedNavGrid(cfg);
if ~isempty(navGrid)
    svFinal = cachedPlaceValidator(cfg, navGrid, objExtent);
    if freeBearings(pt, svFinal, cfg, objExtent) <= 0
        plc.FailureCode = cfg.fail.PlaceUnreachable;
        return
    end
end

plc.Point          = pt;
plc.PlaneNormal    = n;
plc.PlaneOffset    = d;
plc.RegionCenter   = regionCentre;
plc.RegionHalfSize = [halfU; halfV];
plc.Inset          = cfg.percep.surfaceInset;
plc.NumInliers     = uint16(min(numel(inlierIdx), double(intmax('uint16'))));
plc.SurfaceId      = f.Calib.SurfaceIds(idx(seed));
plc.FailureCode    = cfg.fail.OK;
plc.Valid          = true;
end

%% ------------------------------------------------------------------------
function [pt, shiftUV] = biasToReachable(pt, u, v, halfU, halfV, cfg, objExtent, Q, duv, n)
%BIASTOREACHABLE Move a place point inside its own region to somewhere the base fits.
%   [pt, shiftUV] = BIASTOREACHABLE(pt, u, v, halfU, halfV, cfg, objExtent, Q, duv, n)
%   returns pt unchanged and shiftUV = [0 0] unless a stand-off pose exists for some other
%   point in the region and not for pt. shiftUV is the in-plane offset applied, in the
%   (u, v) basis, so the caller can rebase coordinates it has already measured.
%
%   EVERY CANDIDATE IS SCORED AT THE HEIGHT IT WOULD ACTUALLY SHIP AT, which is why Q, duv and
%   n are arguments. That is not symmetry for its own sake - it cost a run. fitSupport levels
%   the chosen point onto its own local patch of surface AFTER this function returns, and on the
%   two-level kitchen counter that correction moved the winner 10 mm up. The arm's lever ceiling
%   falls about 15 mm per cm of support height, so the candidate this function certified with
%   nFree 1 at z = 0.900 (band [0.130 0.567]) shipped with nFree 0 at z = 0.910 (band
%   [0.130 0.552]), the base drove to it, and the place goal was refused as BaseGoalOccupied.
%   Scoring an unlevelled candidate is scoring a point the run will never visit.
%
%   WHAT IT DOES NOT BUY, MEASURED SO IT IS NOT ASSUMED: on that counter there is no better
%   candidate to find. All 24 were scored at the fitted height and exactly 3 were serviceable -
%   all of them at the far +u extreme, which IS the upper strip that levels to 0.910, where
%   every one of the 24 scores zero. So this change does not make that counter work; it stops
%   this function certifying a point it cannot serve, which is what lets fitSupport refuse
%   before the base moves instead of after the mug is in the gripper. The surface is genuinely
%   unserviceable and the honest code was the available fix.
%
%   IT IS STILL NOT THE LAST WORD. fitSupport re-asks freeBearings about the published point and
%   raises cfg.fail.PlaceUnreachable if it is unserviceable, because this function has two early
%   returns that score nothing - no map, and a region with no room - and a refusal has to cover
%   those too.
%
%   IT ASKS THE PLANNER'S OWN QUESTION, WHICH IS THE ONLY WAY THIS CAN BE TRUSTED. The
%   test is hmmStandoffRing against hmmBaseValidators' driving validator, built from the
%   same cfg and the same carried extent the model builds it from - the identical call
%   hmmSelectBaseGoal makes at the supervisory rate. A place point certified here by a
%   slightly different rule than the planner applies would be worse than no
%   certification, because the failure would then have two places to hide; that
%   argument is hmmBaseValidators' and this is it reused. hmmStandoffRing's bearing
%   grid is absolute about the target rather than phased on the caller's pose, which
%   is what makes the question answerable at all from here: perception does not know
%   where the base will be standing when the place is dispatched, and with an absolute
%   grid it does not need to.
%
%   A SERVICEABLE FITTED POINT IS NEVER MOVED, which is what makes this safe to add. pt
%   is tested first and kept if its ring admits anything at all, so this function can
%   only turn a failing place into a working one - it can never change a run that was
%   already working, and it is not perception second-guessing where the operator pointed.
%   On a surface with room to spare it costs one ring evaluation and does nothing.
%
%   BUT ONCE IT IS CHOOSING, IT CHOOSES FOR MARGIN RATHER THAN FOR LEAST DISTURBANCE,
%   and the measurement is why. Eight directions at three fractions of the room available
%   gives 24 candidates; at the shipped kitchen place point seven of them are serviceable,
%   and the NEAREST of those - 69 mm away - has exactly one free bearing out of 24, which
%   is the same knife edge that failed in the first place. The candidates 182 mm out have
%   six. So the rule is the largest hmmStandoffRing nFree, with the smallest displacement
%   breaking ties; on that scene it selects a pure 182 mm move toward the open side and
%   leaves the v axis alone, because moving along v buys no bearings and a bias that is
%   not buying anything should not be spending anything.
%
%   THE BOUND IS THE OBSERVED REGION MINUS THE OBJECT'S OWN FOOTPRINT, AND THE INSET IS
%   DELIBERATELY NOT CHARGED TWICE. halfU and halfV arrive with cfg.percep.surfaceInset
%   already taken off, because as bus fields they bound where a place POINT may sit. The
%   bound this search needs is a different one - where the object's near FACE may sit - so
%   the inset is added back before half the object's larger horizontal extent is taken off.
%   A narrow object therefore gets more room to move than a wide one, which is the correct
%   asymmetry and is why this takes objExtent rather than a constant. The regionHalfMax cap
%   is reapplied after the inset goes back, so the bias still cannot move a point further
%   from where the operator pointed than that constant allows.
%
%   THE DOUBLE CHARGE COST THE KITCHEN PLACE LEG A RUN, which is the measurement that
%   motivated this line. On the counter transfer the fit put the place point 0.192 m inside
%   the observed near edge of the worktop; subtracting the inset AND the mug's 0.0505 m half
%   extent left roomU = 0.0915 m, and the furthest candidate the fan could reach was still
%   0.020 m short of the first serviceable point. Every one of the 25 candidates scored zero
%   free bearings, the fitted point was published unmoved - correctly, by this function's own
%   rule - the base drove to it and hmmPlanBasePath reported BaseGoalOccupied (27). Adding the
%   inset back gives roomU = 0.1415 m, the fan's pure +u candidate scores 2 free bearings at
%   the shipped cfg.sup.placeStandoff and the shipped cfg.plan.baseGoalTolerance, and the leg
%   works. Nothing was retuned to achieve that; the arithmetic was simply charging one margin
%   for two different purposes.
%
%   AND THERE IS NO SAMPLING MARGIN TO ADD BACK ON TOP, WHICH IS NOT OBVIOUS AND WAS CHECKED.
%   The intuition that the observed edge needs its own allowance is right and the allowance is
%   already there: halfU is measured to the furthest INLIER, and an inlier exists only where a
%   ray actually hit the surface, so the true edge is at or beyond it by up to one ray pitch.
%   That pitch is 43 mm at 2.0 m and 76 mm at 3.5 m on this ray grid - larger than the 50 mm
%   inset it would replace, so charging it explicitly is strictly worse than charging nothing.
%   Measured against the occupancy map on the counter above: the furthest inlier sat 0.192 m
%   out while the counter's real edge is 0.267 m out, so an object placed flush with the
%   OBSERVED edge still rests 75 mm inside the real one.
%
%   IT IS SKIPPED ENTIRELY UNDER cfg.map.mode = 'live'. There is no map to reason about
%   before the cameras have built one - hmmLoadNavMap returns the uniform prior, against
%   which every cell is unknown and every candidate would fail - so the fitted point is
%   published and behaviour is exactly what it was. That also means the grid is a
%   constant here and needs no place in the memo key, unlike in hmmSelectBaseGoal.
%
%   See also hmmStandoffRing, hmmBaseValidators, hmmSelectBaseGoal, hmmArmLeverBand.
shiftUV = [0 0];

grid = cachedNavGrid(cfg);
if isempty(grid)
    return
end

% The inset goes back on before the object's footprint comes off - see the header. The cap
% is reapplied so the bias still respects how far perception may move the operator's point.
objHalf = 0.5 * max(double(objExtent(1:2)));
roomU   = max(min(halfU + cfg.percep.surfaceInset, cfg.percep.regionHalfMax) - objHalf, 0);
roomV   = max(min(halfV + cfg.percep.surfaceInset, cfg.percep.regionHalfMax) - objHalf, 0);
sv = cachedPlaceValidator(cfg, grid, objExtent);

% A serviceable point is never moved. See the header. LEVELLED FIRST, like every candidate
% below, so the question asked here is the one fitSupport will act on.
if servedClearly(levelToLocal(pt, Q, duv, n, cfg, objExtent), sv, cfg, objExtent)
    return
end

% No room in the observed patch: only the surface-wide slide can still help.
if roomU <= 0 && roomV <= 0
    [shiftUV, ok] = hmmSlideOnSurface(pt, u, v, sv, cfg, objExtent);
    if ok
        pt = pt + shiftUV(1) * u + shiftUV(2) * v;
        if numel(shiftUV) > 2
            pt(3) = pt(3) + shiftUV(3);      % the landing cell's own height (hmmSlideOnSurface)
        end
    end
    return
end

dirs = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
cand = [dirs .* [roomU roomV]; dirs .* (0.7 * [roomU roomV]); dirs .* (0.35 * [roomU roomV])];

% Most free bearings wins; the smaller displacement breaks the tie. Sorting by
% displacement FIRST is what makes the tie-break a one-liner - max() returns the first
% maximum, so the winner is the least disturbing member of the best-served group.
[~, ord] = sort(vecnorm(cand, 2, 2));
cand = cand(ord, :);

% duv IS REBASED ONTO EACH CANDIDATE BEFORE LEVELLING IT, exactly as fitSupport rebases it
% onto the winner. The window has to be centred where the object would rest, or the candidate
% is levelled against the patch of surface under the FITTED point - which is the whole error
% this scoring exists to avoid, reintroduced one function in.
nFree = zeros(size(cand, 1), 1);
for k = 1:size(cand, 1)
    pc   = pt + cand(k, 1) * u + cand(k, 2) * v;
    duvC = duv - cand(k, :);
    nFree(k) = freeBearings(levelToLocal(pc, Q, duvC, n, cfg, objExtent), sv, cfg, objExtent);
end

[best, k] = max(nFree);
if best > 0
    shiftUV = cand(k, :);
    pt      = pt + shiftUV(1) * u + shiftUV(2) * v;
    return
end

% Nothing in the observed patch serves. Look further along the same surface - see hmmSlideOnSurface.
[shiftUV, ok] = hmmSlideOnSurface(pt, u, v, sv, cfg, objExtent);
if ok
    pt = pt + shiftUV(1) * u + shiftUV(2) * v;
    if numel(shiftUV) > 2
        pt(3) = pt(3) + shiftUV(3);      % the landing cell's own height (hmmSlideOnSurface)
    end
end

end

%% ------------------------------------------------------------------------
function [pt, shift] = levelToLocal(pt, Q, duv, n, cfg, objExtent)
%LEVELTOLOCAL Sit the place point on its OWN patch of surface, not on the region's average.
%   [pt, shift] = LEVELTOLOCAL(pt, Q, duv, n, cfg, objExtent) moves pt along n onto the median
%   height of the inliers within the object's own footprint, and returns the signed distance
%   moved. duv must already be rebased onto pt: it is the in-plane coordinate of every inlier
%   RELATIVE TO pt, so a caller testing a candidate elsewhere in the region has to subtract
%   that candidate's offset first.
%
%   THE WINDOW IS THE OBJECT'S FOOTPRINT PLUS ONE INSET, which is the patch the object will
%   actually stand on plus the margin the region already reserves around anything set down
%   there. Using the footprint rather than a fixed radius is what makes this work for an object
%   other than a mug - a wider object rests on more of the surface and should be levelled
%   against more of it.
%
%   IT IS A FUNCTION BECAUSE TWO CALLERS MUST AGREE, AND THEY DID NOT. It was inline in
%   fitSupport, applied after biasToReachable had already chosen on unlevelled heights; the
%   kitchen counter has two levels 10 mm apart, the correction moved the winner onto the upper
%   one, and 10 mm of support height costs ~15 mm of the arm's lever ceiling - enough to erase
%   the single free bearing the candidate was picked for. Both callers now level before they
%   judge, which is only enforceable while there is one implementation of it.
%
%   A SHORT WINDOW LEAVES pt ALONE rather than levelling against two or three hits. Below
%   cfg.percep.localMinInliers the median is noise, and noise applied along the normal is a
%   height error in the quantity the arm's reach is most sensitive to.
shift       = 0;
localRadius = 0.5 * max(double(objExtent(1:2))) + cfg.percep.surfaceInset;
local       = hypot(duv(:, 1), duv(:, 2)) <= localRadius;
if sum(local) >= cfg.percep.localMinInliers
    shift = median((Q(local, :) - pt') * n);
    pt    = pt + shift * n;
end
end

%% ------------------------------------------------------------------------
function n = freeBearings(p, sv, cfg, objExtent)
%FREEBEARINGS How many poses on p's place stand-off ring the planner would accept.
%   The shared implementation is hmmPlaceBearings, which hmmSlideOnSurface also calls,
%   so the two give one answer.
n = hmmPlaceBearings(p, sv, cfg, objExtent);
end

function tf = servedClearly(p, sv, cfg, objExtent)
%SERVEDCLEARLY Serviceable, and not only by reaching over furniture to get there.
%   A provider point that passes this is kept as it is. One served only across something
%   taller than its surface - a couch seat reached over its backrest - goes on to
%   hmmSlideOnSurface, which prefers a clear spot and still falls back to the reach-over.
[nF, pose] = hmmPlaceBearings(p, sv, cfg, objExtent);
tf = nF > 0 && ~hmmPlaceReachesOver(pose, p, cfg, objExtent);
end

%% ------------------------------------------------------------------------
function grid = cachedNavGrid(cfg)
%CACHEDNAVGRID The shipped nav map once per session, or empty when there is none.
%   The cell wrapper is load bearing: it distinguishes "not looked up yet" from
%   "looked up and there is no map", which a bare [] cannot, and without it 'live'
%   mode would reload and re-check provenance on every reconstruction.
persistent held
if isempty(held)
    if isfield(cfg.map, 'mode') && strcmp(cfg.map.mode, 'live')
        held = {[]};
    else
        held = {hmmLoadNavMap(cfg)};
    end
end
grid = held{1};
end

%% ------------------------------------------------------------------------
function sv = cachedPlaceValidator(cfg, grid, objExtent)
%CACHEDPLACEVALIDATOR hmmBaseValidators' driving validator, memoised on the load.
%   Inflating a 240x240 costmap is the expensive part and the carried extent is the
%   only thing that varies between reconstructions, so it is the whole key. The grid
%   is not in it because this is reached only in 'fixed' mode - see biasToReachable.
persistent key val
e = double(objExtent(:));
if isempty(val) || ~isequal(key, e)
    val = hmmBaseValidators(grid, cfg, e);
    key = e;
end
sv = val;
end

%% ------------------------------------------------------------------------
function [u, v] = planeBasis(n)
%PLANEBASIS Any orthonormal in-plane pair. The seed axis is the world axis least
%   aligned with the normal, which keeps the basis well conditioned for both a
%   level surface and a tilted one.
[~, least] = min(abs(n));
a = zeros(3, 1);
a(least) = 1;
u = cross(n, a);
u = u / norm(u);
v = cross(n, u);
end

%% ------------------------------------------------------------------------
function tf = inBounds(p, cfg)
%INBOUNDS Inside the configured scene extent (gate 7).
b = cfg.percep.sceneBounds;
tf = all(p(:) >= b(:, 1)) && all(p(:) <= b(:, 2));
end

%% ------------------------------------------------------------------------
function obj = blankObject(objTemplate, cfg, simTime)
%BLANKOBJECT An ObjectTarget that vouches for nothing.
obj = objTemplate;
obj.Label(:)      = 0;
obj.LabelLen      = uint8(0);
obj.Position      = zeros(3, 1);
obj.Orientation   = zeros(3, 1);
obj.Extent        = zeros(3, 1);
obj.GraspExtent   = 0;
% EXTENT_NONE, not EXTENT_ASSUMED. A blank target publishes zeros, and _ASSUMED
% claims the extent is cfg.percep.extentAssumed - 60 mm - which it is not. The two
% disagreed here from the start; the difference only became visible once _NONE
% existed to say the truthful thing.
obj.ExtentSource  = cfg.percep.EXTENT_NONE;
obj.ActorId       = cfg.actor.noneId;
obj.NumCandidates = uint8(0);
obj.SurfaceId     = uint32(0);
obj.Confidence    = 0;
obj.SourceCamera  = uint8(0);
obj.PixelPoint    = zeros(2, 1);
obj.FailureCode   = cfg.fail.OK;
obj.Timestamp     = simTime;
obj.Valid         = false;
end

%% ------------------------------------------------------------------------
function plc = blankPlace(plcTemplate, cfg, simTime)
%BLANKPLACE A PlaceTarget that vouches for nothing.
plc = plcTemplate;
plc.Point          = zeros(3, 1);
plc.PlaneNormal    = [0; 0; 1];
plc.PlaneOffset    = 0;
plc.RegionCenter   = zeros(3, 1);
plc.RegionHalfSize = zeros(2, 1);
plc.Inset          = cfg.percep.surfaceInset;
plc.NumInliers     = uint16(0);
plc.SurfaceId      = uint32(0);
plc.Confidence     = 0;
plc.SourceCamera   = uint8(0);
plc.PixelPoint     = zeros(2, 1);
plc.FailureCode    = cfg.fail.OK;
plc.Timestamp      = simTime;
plc.Valid          = false;
end

%% ------------------------------------------------------------------------
function cfg = cachedConfig()
cfg = hmmRuntimeConfig();
end

%% ------------------------------------------------------------------------
function rays = cachedRays(cfg)
%CACHEDRAYS The ray grid is a compile-time constant; build it once.
persistent r
if isempty(r)
    r = hmmRayDirections(cfg);
end
rays = r;
end
