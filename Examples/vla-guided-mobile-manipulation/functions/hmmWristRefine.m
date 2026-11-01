function [position, status, confidence, pixel, shift, pivotXY, height] = ...
    hmmWristRefine(frame, coarsePosition, coarseExtent, instruction, simTime, cfg)
%HMMWRISTREFINE Refine a grasp point from the wrist camera, just before the descent.
%
%   [position, status, confidence, pixel, shift, pivotXY, height] =
%       HMMWRISTREFINE(frame, coarsePosition, coarseExtent, instruction, simTime, cfg)
%
%   frame is the wrist camera's CameraFrame (contract index 5), coarsePosition the [3 1]
%   ObjectTarget.Position the wide grounding call reconstructed, coarseExtent its [3 1] Extent,
%   and instruction the task sentence. position is the refined ObjectTarget.Position, status one
%   of cfg.refine.STATUS_*, pixel the [row; col] the provider pointed at and shift the distance
%   the point moved.
%
%   height IS THE OBJECT'S OWN HEIGHT, MEASURED, and it is 0 WHENEVER THERE IS NO MEASUREMENT -
%   on every status but STATUS_APPLIED, and on an APPLIED frame whose support plane could not be
%   read. Zero rather than coarseExtent(3) on purpose: a caller has the coarse extent already and
%   does not need it echoed, whereas "I measured nothing" is a fact only this function knows, and
%   a number that silently alternates between a measurement and a manifest lookup is the kind of
%   output that gets believed in the wrong half of its range. hmmRefineService is the caller that
%   resolves the two, and it does so on RefineValid rather than on this number's sign.
%
%   IT IS THE SAME SUBTRACTION THE POSITION IS CARRIED DOWN BY, which is the property worth
%   stating: the top face measured in this frame, less the support plane measured in the same
%   frame. So Position(3) + height lands back on the top face by construction, and the two outputs
%   cannot disagree about where the object's extremes are. MotionPlanning/TargetSelect aims the
%   jaws at hmmGraspAim's clamped mid-height of whatever Extent(3) reaches it, so this is the
%   number that decides the grasp band for an object the scene manifest has never heard of.
%
%   pivotXY IS A SECOND PRODUCT OF THE SAME LOOK, AND IT IS NOT ABOUT WHERE TO GRASP. It is the
%   [1 2] WORLD offset from the point the jaws will close on to the point the renderer will pin
%   to the tool when they do - the object's mesh pivot - and it is [0 0] on every path that is not
%   STATUS_APPLIED, which is the same degrade-to-today's-behaviour rule everything else here
%   follows. hmmCarryTranslation turns it into the lateral part of the Simulation 3D Robot block's
%   end-effector-to-target translation, which is what stops a grasped prop snapping sideways the
%   instant the gripper closes. It replaced a hand-measured table of six prop names; see
%   hmmSegmentTopFace, which measures both points, and hmmRefineService, which applies this one.
%
%   WHY THE CARRY OFFSET IS MEASURED HERE AND NOT AT SETUP. It is a property of the mesh, so it
%   looks like a constant that belongs in hmmConfig, and that is exactly how it shipped - one row
%   per graspable prop, resolved from the instruction text before the first frame. Two things are
%   wrong with that. A name table cannot answer for a prop nobody measured, and the project's
%   standing rule is that a fix keyed to one prop name is a defect. And the offset is not actually
%   a property of the mesh alone: it is the mesh's pivot expressed against the object's own axis,
%   in the yaw the object is STANDING in, and nothing before the first frame knows that yaw. This
%   camera does, because it is looking at the thing. Measured on a live frame of the non-red Mug -
%   a prop whose row was not the one the table was calibrated from - the run-time measurement
%   reproduced the tabulated constant to 0.6 mm and the engine's own actor origin to 1.07 mm.
%
%   THE SEQUENCE THIS IMPLEMENTS. Four ceiling cameras ground the instruction, the base drives
%   to the pick, and the arm moves to a pre-descent pose ArmPlan.ApproachDepth above the grasp
%   point - a per-pick height, not a constant: hmmScreenArmPlan searches cfg.sup.approachDepth
%   Ladder downward from 0.240 m and keeps the highest rung the arm can hold with the tool still
%   vertical. AT THAT MOMENT, and before the constrained descent commits, the wrist camera is
%   looking at that grasp point from between 0.186 m (the 0.080 rung) and 0.341 m (the 0.240
%   one) and one frame is taken. THE SEARCH EXISTS FOR THIS FUNCTION, which is worth saying
%   plainly: at the 0.080 standoff the lens ends up 65 mm from a mug's top face, where a 101 mm
%   object subtends about 480 px of a 360-row frame, and there is no shape left in the picture
%   for a provider to point at. The top rung frames the whole object with 65 px to spare. Its RGB
%   goes to the provider - and only its RGB, because a vision-language model asked to read a
%   depth map is being asked to eyeball intensities. The answer is a PIXEL. The metre reading
%   comes from the aligned Depth image of the same frame, which is the entire reason an RGBD
%   sensor is the right instrument here: the two channels are registered by construction, so
%   the z the model cannot supply is simply looked up at the pixel it can.
%
%   "ALONG THE DESCENT" WAS WRONG TWICE, IN TWO INDEPENDENT PLACES, AND BOTH LOOKED THE SAME
%   FROM HERE. Where the camera points is decided by cfg.camera.wrist.tilt AND by the pose
%   chain that hands the tree's answer to Unreal, and each was broken on its own:
%
%     THE MOUNT, WHICH TOOK TWO CORRECTIONS RATHER THAN ONE. At tilt = -0.35 rad the optical
%     axis came out 69.95 deg off the descent and the grasp point projected 2749 rows above a
%     360-row frame. The obvious repair is -pi/2, which puts the axis exactly 0.00 deg off the
%     descent - and it is still not right, because the camera is 45 mm off the tool axis, so a
%     PARALLEL axis looks 45 mm to the side of the object. Measured: the grasp point landed at
%     row 102.6 of 360 and the mug ran off the top edge, rows 1 to 92. The tilt was then
%     -1.8154 rad, aimed at the grasp point rather than along the descent, and is now the
%     MIRROR of that, -1.3262, because the camera moved to the FRONT face of the coupling
%     (cfg.camera.wrist.offset(1) = -0.045); the aim is the same geometric construction seen
%     from the other side. Re-derived independently from the grasp point, -1.3262 puts the
%     axis 14.01 deg off the descent, projecting to col 320.00 exactly, in frame and centred.
%
%     THE ROW IS 144.79, NOT THE 180.00 THIS HEADER USED TO CLAIM, and the difference is not a
%     defect. The tilt is derived against cfg.sup.approachDepth, but the standoff is now
%     SEARCHED per pick and the run holds the 0.240 m ceiling rung, so the camera is further
%     back than the tilt was solved for and the grasp point rides higher in the frame. It is
%     well inside the frame's central half, and the mug's own top face lands at row 174.47.
%
%     THE RENDER CONVENTION. Plant/CameraRig/WristCamera/WristMountPose converts the Get
%     Transform block's 4x4 with a Coordinate Transformation Conversion block, and that block
%     emits a RIGHT-handed Euler triple while the Simulation 3D Camera's Rotation port wants
%     Unreal's LEFT-handed pitch and roll composed ZYX - the convention hmmBackprojectDepth
%     measures and asserts three ways. Feeding one straight into the other rendered the camera
%     150.05 deg away from where the tree says it looks: measured at the actual pre-descent
%     configuration, the tree gives [-0.0003 0.0000 -1.0000] and the render gave
%     [-0.0003 -0.4992 +0.8665]. The wrist frame is a picture of the SKY. The block now
%     outputs ZYX and a Gain of [0 0 -1; 0 -1 0; 1 0 0] maps [z y x] to [-x -y z], which
%     reproduces the tree's orientation to 7e-14. The four ceiling cameras never showed this
%     because their Rotation is a hmmConfig constant authored in the port's own convention;
%     the wrist is the first camera in this model whose rotation is COMPUTED.
%
%   WHAT IT COST, AND WHY IT WAS NEARLY INVISIBLE. Under the first defect every refinement in a
%   full run was rejected STATUS_RANGE on a depth of 4.1922 m; under the second, TOOFAR on a
%   0.0352 m reading of the robot's own geometry. Both are advisory rejections, so the task
%   still succeeded and the only trace was a status code. Note that the refinement was never
%   NONSENSE in either case - hmmBackprojectDepth inverts using the camera's OWN reported
%   rotation, so it faithfully reconstructed a point the camera really was looking at. It was
%   pointing somewhere useless, accurately. robots/assembleMobileManipulator now hangs a
%   visible box on the wrist frame, behind the lens, so the next mistake of this kind is
%   something an operator SEES in the Sim3D view rather than a number in a rejected status.
%
%   WHY THE SECOND LOOK IS WORTH A SECOND CALL, in one line: the geometry, not the model. One
%   pixel subtends 1.10 mm at the wrist's pre-descent range on the top rung, and 16.5 mm at a
%   ceiling camera's 3.798 m, so the same provider answering the same schema is ~15x more
%   precise from here. The mug that spans 6.1 px in a ceiling frame spans ~92 px in this one -
%   the same object at both ends, so the two figures are comparable; see cfg.camera.pixelScale.
%   The bottom rung is 0.60 mm and ~174 px, which is 28x and is the figure this header used to
%   quote, and it is the WORSE picture rather than the better one: at that range the 174 px
%   overflow the frame. Precision per pixel is not worth anything on a pixel that does not
%   exist, which is the whole reason the standoff is searched upward.
%
%   IN REPLAY THE FIXTURE IS A TEST, NOT A REHEARSAL, AND IT AIMS AT THE MUG RATHER THAN AT
%   THE ROBOT'S OWN GUESS. cfg.provider.mode is 'replay' by default, and the recorded refine
%   answer is the pixel centroid of the mug's top face, segmented out of the very depth image
%   this function consumes; it is in frame, and it sits more than 10 px from the projection of
%   the COARSE grasp point. The second is the one that matters - if the answered pixel were
%   the coarse point's own projection, the refinement would be handed its input back and any
%   displacement it reported would be closure rather than correction.
%
%   SO A REPLAY RUN MEASURES BOTH THINGS AT ONCE. The chain still has to close - forward
%   kinematics against the Sim3D camera pose, the depth encoding and the pinhole - and on top
%   of that the applied shift has to move the grasp point ONTO the mug, about 39 mm laterally
%   from where the four room cameras put it. The rise gate is what separates the two failures:
%   a correct chain reading the real top face rises about one Extent(3), and that is checked
%   before any displacement is accepted.
%
%   AN EARLIER FIXTURE HERE WAS FORWARD-PROJECTED AND SILENTLY DID NOTHING, which is why the
%   paragraphs above insist on the image as the source. It was computed from a world point via
%   the pinhole rather than read off the render, landed on the TABLETOP at z 0.54989, and the
%   rise gate rejected it at -0.00011 m against a band of [0.0450 0.1350] - STATUS_NOOBJECT,
%   shift 0.0000, coarse point kept, every run. The gate did its job; the fixture was wrong.
%   Read the pixel off the frame, never project a point into it.
%
%   THE PROVIDER IS NO LONGER REQUIRED FOR A MEASUREMENT (STATUS_DEPTHONLY). When the provider
%   path fails - a 503 or timeout, a malformed or found:false answer, a low confidence, or a
%   pixel whose region fails a gate - the coarse point is projected into this same frame and the
%   identical measurement is run from that seed. Falling back to the coarse point used to be the
%   safe default; it is not since the attach offset is frozen at jaw closure, because a grasp on
%   that point (~56 mm high on the mug) is carried into the place and released into the surface.
%   See cfg.refine.depthOnlyFallback. Only when both seeds fail does the text below still hold,
%   and that case is now printed to stderr rather than left to a status code.
%
%   IT IS ADVISORY, AND THAT IS THE SAFETY ARGUMENT IN FULL. Every path out of this function
%   that is not STATUS_APPLIED or STATUS_DEPTHONLY returns coarsePosition UNCHANGED, so the descent proceeds
%   exactly as it does without the refinement: no key, no fixture, a malformed answer, a
%   found:false, a low confidence, an all-sentinel depth patch, a range outside
%   [minRange maxRange], a surface that is not the object's top face (STATUS_NOOBJECT) or a
%   displacement over cfg.refine.maxShift all degrade to today's behaviour rather than to a
%   failure. The refinement can improve the pick or leave it alone.
%   It cannot introduce a way for the task to fail that the model does not already have, which
%   is why none of its outcomes is a cfg.fail code and no chart transitions on them.
%
%   THE DISPLACEMENT CAP IS THE ONE GUARD THAT MATTERS. A refinement is supposed to move the
%   grasp point by millimetres to centimetres. One that wants to move it 0.4 m has not refined
%   anything - it has found a different object, or the arm is poised over a cluttered surface
%   and the provider picked the wrong item on it - and applying it would send the descent to a
%   place the arm plan was never screened against for collision or reach. cfg.refine.maxShift is
%   the standoff height, and because the standoff is now searched per pick it is asserted equal
%   to cfg.sup.approachDepthPickMax - the CEILING of that search, not the rung chosen. This
%   function is handed cfg and a frame, never the plan, so it cannot know which rung it is
%   standing on; capping at the ceiling is the one bound that is never TIGHTER than the standoff
%   actually being held, which is the direction an advisory guard should err in.
%
%   THE TWO STAGES DO NOT RANGE TO THE SAME SURFACE, and that is why coarseExtent is an
%   argument. ObjectTarget.Position is the SUPPORT SURFACE beneath the object's centre, because
%   that is what the ceiling rig can actually resolve: at 3 m one ray of the 36x64 grid covers
%   130 x 130 mm, the mug is 101 x 70 mm, so fewer than one ray lands on it and the cluster whose
%   centroid becomes Position is a patch of TABLE. It is a good measurement of the table - the
%   table is far larger than a ray cell - and hmmReconstructTargets publishes it as what it is.
%   This camera, ranging down from 0.24 m at 1.10 mm/px, has the opposite problem and the
%   opposite virtue: it resolves the object easily but can only ever reach its TOP FACE, the
%   first surface the ray meets. The two differ by the object's height, so writing the
%   back-projected point into Position unconverted would move the grasp UP by a full object
%   height while reporting success. The conversion is one subtraction, Extent(3), and it is done
%   before the displacement cap so that cap measures the quantity it is expressed in.
%
%   THE CONVERSION NEEDS A GUARD, AND THE GUARD IS FREE. Subtracting Extent(3) is only valid if
%   the ray reached the OBJECT; a pixel that lands on the table beside it would be converted into
%   a support surface one object height too LOW - the mirror of the error above, and the more
%   likely of the two, because the coarse point this camera is aimed from sits about 49 mm out,
%   at the mug's edge. The discriminator costs nothing: a ray that hit the object rises about one
%   object height above the coarse surface, and a ray that hit the table rises about zero. So the
%   rise is required to fall inside cfg.refine.riseBand * Extent(3), a band of [0.5 1.5] which no
%   flat surface can enter and no plausible object can miss. Failing it is STATUS_NOOBJECT and
%   keeps the coarse point, which is the honest answer: the second look did not find the thing it
%   was sent to look at. THE BAND IS A FRACTION RATHER THAN A TOLERANCE IN METRES on purpose -
%   it then scales with the object and cannot be made meaningless by a short one, where an
%   absolute band wide enough for a mug would swallow the table hypothesis entirely.
%
%   WHAT THIS FUNCTION DELIBERATELY DOES NOT TOUCH. It does not go through
%   hmmValidateActions, because the refine schema has no verb, no second point and no camera
%   index, so none of that function's five gates apply and gate 3's rejection of camera 5 stays
%   correct for the wide call. It does not go through hmmReconstructTargets, because there is
%   no ray registration to do: Depth is read at the pixel directly. And it does not change what
%   is being picked up or what happens after the pick - only where, within maxShift, the jaws
%   close.
%
%   See also hmmBackprojectDepth, hmmProviderRequest, hmmGroundingService, hmmGroundingTap.

%   Copyright 2026 The MathWorks, Inc.

arguments
    frame          struct
    coarsePosition double
    coarseExtent   double
    instruction    (1, :) char
    simTime        double
    cfg            struct = hmmConfig()
end

% THE COARSE POINT IS THE DEFAULT, ASSIGNED FIRST, so that every early return below is
% automatically the safe outcome rather than relying on each path to remember.
position   = reshape(double(coarsePosition), 3, 1);
confidence = 0;
pixel      = zeros(2, 1);
shift      = 0;
pivotXY    = [0 0];
height     = 0;

if ~cfg.refine.enabled
    status = cfg.refine.STATUS_DISABLED;
    return
end

%% Frame admissibility, before any request is issued
% Same two tests the wide call applies to the fixed rig, for the same reasons: an invalid
% frame is one the render pipeline has not filled yet, and a stale one is a picture of where
% the object WAS, which at this range is worse than no picture at all.
if ~frame.Valid || simTime - frame.Timestamp > cfg.safety.staleTimeout
    status = cfg.refine.STATUS_NOFRAME;
    return
end

%% Request
index = double(frame.CameraIndex);
% The operator console shows the frame while the request is in flight - see hmmRefineNotify.
hmmRefineNotify('request', struct('RGB', frame.RGB, 'CameraIndex', index, ...
    'SimTime', simTime, 'Instruction', instruction));
[raw, route, httpStatus, latency, providerFailure] = ...
    hmmProviderRequest(instruction, {frame.RGB}, index, cfg, 'refine');

% THE PROVIDER PATH PRODUCES A SEED PIXEL OR A REASON IT COULD NOT. Every reason is now a
% candidate for the depth-only fallback below rather than an immediate return, because the
% coarse point is not a safe default any more - see cfg.refine.depthOnlyFallback.
depth = 0;
providerPixel = [];
if providerFailure ~= cfg.fail.OK
    status = cfg.refine.STATUS_PROVIDER;
else
    [found, pt, conf, okJson] = decodeRefine(raw, cfg);
    if ~okJson
        status = cfg.refine.STATUS_PROVIDER;
    else
        confidence = conf;
        % found:false IS A CLEAN ANSWER, NOT AN ERROR. The prompt asks for it explicitly and the
        % object may genuinely not be in the wrist frame. It is reported as LOWCONF.
        if ~found || conf < cfg.provider.confidenceMin
            status = cfg.refine.STATUS_LOWCONF;
        else
            % Normalised 0..pixelNormScale, [y, x], y downward from the top row - the same
            % contract the wide call uses, so a reader does not have to hold two conventions.
            rows = double(frame.ImageSize(1));
            cols = double(frame.ImageSize(2));
            providerPixel = [pt(1) / cfg.camera.pixelNormScale * rows; ...
                             pt(2) / cfg.camera.pixelNormScale * cols];
            pixel = providerPixel;
            [m, status, depth] = measureFromSeed(frame, providerPixel, position, ...
                coarseExtent, cfg);
        end
    end
end

%% Depth-only fallback, seeded by the coarse point's own projection
% THE SEGMENTATION NEVER NEEDED THE PROVIDER'S PIXEL TO BE PRECISE - only to land inside the
% search window, whose radius is cfg.refine.searchRadiusScale * max(Extent(1:2)) (~150 mm on a
% mug) against a coarse error of 40-50 mm. So when the provider is unusable, or its pixel seeded a
% region that failed a gate, the coarse point is projected into this same frame and the same
% measurement is run from there. The result is reported as DEPTHONLY, never as APPLIED, so a run
% still says plainly that the provider did not contribute.
providerStatus = status;
if status ~= cfg.refine.STATUS_APPLIED && cfg.refine.depthOnlyFallback
    [seed, inFrame] = projectIntoFrame(frame, position);
    if inFrame
        [mSeed, seedStatus, seedDepth] = measureFromSeed(frame, seed, position, ...
            coarseExtent, cfg);
        if seedStatus == cfg.refine.STATUS_APPLIED
            m      = mSeed;
            status = cfg.refine.STATUS_DEPTHONLY;
            pixel  = seed;
            depth  = seedDepth;
        end
    end
end

if status ~= cfg.refine.STATUS_APPLIED && status ~= cfg.refine.STATUS_DEPTHONLY
    % LOUD, BECAUSE THE GRACEFUL PATH IS NOT HARMLESS. A grasp on the coarse point is carried
    % into the place by the frozen attach offset, so this line predicts a misplaced release.
    fprintf(2, ['  refine: NOT APPLIED - status %s (provider %s, http %d) - the pick will use ' ...
        'the COARSE point, expect a high grasp and a low release\n'], ...
        hmmEnumName(cfg.refine, status, 'STATUS_'), ...
        hmmEnumName(cfg.refine, providerStatus, 'STATUS_'), httpStatus);
    tap(instruction, frame, position, status, confidence, pixel, shift, depth, ...
        route, httpStatus, latency, simTime, raw);
    return
end

shift    = m.shift;
position = m.position;
pivotXY  = m.pivotXY;
height   = m.height;

if status == cfg.refine.STATUS_DEPTHONLY
    fprintf(['  refine: provider unusable (%s, http %d) - top face measured from the coarse ' ...
        'seed instead, shift %.1f mm\n'], hmmEnumName(cfg.refine, providerStatus, 'STATUS_'), ...
        httpStatus, 1000 * shift);
end

% THE PROVIDER'S OWN XY, AS A WITNESS AND NOT AS THE ANSWER. The prompt asks for the centre between the jaws
% (2026-10-16; it used to say "the body, not the rim", which pulled it to the edge), so the provider's pixel refers to the object's axis at mid-height;
% its ray crossing that plane is an xy estimate with no depth reading in it at all. The top-face
% centroid stays the answer (it is ~1 px/mm here against the provider's +-10-20 px), but the
% disagreement between the two is printed so a biased segmentation cannot hide.
if ~isempty(providerPixel)
    pAxis = rayAtHeight(frame, providerPixel, position(3) + 0.5 * m.heightUsed);
    if all(isfinite(pAxis))
        fprintf('  refine: provider ray at mid-height differs from top face by [%.1f %.1f] mm\n', ...
            1000 * (pAxis(1) - position(1)), 1000 * (pAxis(2) - position(2)));
    end
end

% The console line stays, because it is the only place the measurement can be read against a
% manifest entry that independently knows the answer. One line per pick, next to the carry offset's.
if m.zMeasured
    fprintf(['  refine: object height measured %.1f mm (manifest says %.1f mm), ' ...
        'support plane z %.4f m\n'], 1000 * m.heightUsed, 1000 * double(coarseExtent(3)), m.zUsed);
end

tap(instruction, frame, position, status, confidence, pixel, shift, depth, ...
    route, httpStatus, latency, simTime, raw);

end

%% ------------------------------------------------------------------------
function [m, status, depth] = measureFromSeed(frame, pixel, coarse, coarseExtent, cfg)
%MEASUREFROMSEED The whole depth measurement, from one seed pixel to a gated Position.
%   Shared by the provider path and the depth-only fallback so the two cannot drift. m.position
%   is [pTopFace(1:2); zUsed] - the support surface under the object's axis - and every gate the
%   refinement has always applied is applied here: depth validity and range at the seed, a
%   segmented top face, the rise band against zUsed, and the displacement cap.
m = struct('position', coarse, 'shift', 0, 'pivotXY', [0 0], 'height', 0, ...
    'heightUsed', double(coarseExtent(3)), 'zUsed', coarse(3), 'zMeasured', false);

% THE SINGLE-PIXEL READ IS A VALIDITY TEST, NOT THE MEASUREMENT: for an open-topped object the
% seed looks straight through the hole. It decides NODEPTH and RANGE only.
[~, depth, status] = hmmBackprojectDepth(frame, pixel, cfg);
if status ~= cfg.refine.STATUS_APPLIED
    return
end

% ONE PIXEL CANNOT LOCATE AN OPEN OBJECT, so the seed only centres the search; the plateau's
% centroid is the object's axis because a disc and an annulus share a centroid.
heightRef = double(coarseExtent(3));
[pTopFace, ~, ~, status, pPivot, zUsed, zMeasured] = ...
    hmmSegmentTopFace(frame, pixel, coarse(3), coarseExtent, cfg);
if status ~= cfg.refine.STATUS_APPLIED
    return
end

% THE RISE GATE, AGAINST zUsed AND NOT THE HINT - the hint is the coarse Position(3), which reads
% ~56 mm high; gating on it would reject exactly the frames the measured plane rescued. The band is
% taken against the REFERENCE height so the test stays independent of the measurement it checks.
rise = pTopFace(3) - zUsed;
if rise < cfg.refine.riseBand(1) * heightRef || rise > cfg.refine.riseBand(2) * heightRef
    status = cfg.refine.STATUS_NOOBJECT;
    return
end

% Carried down by the MEASURED height when the plane was read, by the reference otherwise.
if zMeasured
    heightUsed = rise;
else
    heightUsed = heightRef;
end
pRefined = pTopFace - [0; 0; heightUsed];

shift = norm(pRefined - coarse);
if shift > cfg.refine.maxShift
    status = cfg.refine.STATUS_TOOFAR;
    m.shift = shift;
    return
end

m.position   = pRefined;
m.shift      = shift;
m.pivotXY    = reshape(pPivot(1:2) - pTopFace(1:2), 1, 2);
m.heightUsed = heightUsed;
m.zUsed      = zUsed;
m.zMeasured  = zMeasured;
if zMeasured
    m.height = heightUsed;
end
status = cfg.refine.STATUS_APPLIED;

end

%% ------------------------------------------------------------------------
function [R, t, fx, fy, cx, cy] = cameraModel(frame)
%CAMERAMODEL The wrist pinhole, in exactly hmmSegmentTopFace's convention: sensor +X optical
%   axis, +Y image right, +Z image up, Rotation decoded as eul2rotm([r3 -r2 -r1], 'ZYX').
K  = double(frame.Intrinsics);
fx = K(1);
fy = K(2);
cx = K(3);
cy = K(4);
rot = double(frame.Rotation);
R   = eul2rotm([rot(3), -rot(2), -rot(1)], 'ZYX');
t   = reshape(double(frame.Translation), 3, 1);
end

function [pixel, inFrame] = projectIntoFrame(frame, p)
%PROJECTINTOFRAME [row; col] of world point p, the exact inverse of the segmentation's back-projection.
[R, t, fx, fy, cx, cy] = cameraModel(frame);
s = R' * (reshape(double(p), 3, 1) - t);
pixel = [cy - fy * s(3) / s(1); cx + fx * s(2) / s(1)];
rows = double(frame.ImageSize(1));
cols = double(frame.ImageSize(2));
inFrame = s(1) > 0 && all(isfinite(pixel)) && pixel(1) >= 1 && pixel(1) <= rows && ...
    pixel(2) >= 1 && pixel(2) <= cols;
end

function p = rayAtHeight(frame, pixel, z)
%RAYATHEIGHT Where the ray through pixel crosses the horizontal plane at height z (NaN if it cannot).
[R, t, fx, fy, cx, cy] = cameraModel(frame);
dirW = R * [1; (double(pixel(2)) - cx) / fx; -(double(pixel(1)) - cy) / fy];
if dirW(3) >= -1e-9
    p = nan(3, 1);
    return
end
p = t + ((z - t(3)) / dirW(3)) * dirW;
end


%% ------------------------------------------------------------------------
function [found, pt, conf, ok] = decodeRefine(raw, cfg)
%DECODEREFINE Parse {"found":..,"point":[y,x],"confidence":..}, trusting nothing.
%   Deliberately separate from hmmValidateActions: this is a different schema, and sharing a
%   parser would mean one function that has to know which of two shapes it was handed.
found = false;
pt    = [0 0];
conf  = 0;
ok    = false;

inner = hmmUnwrapFence(raw);
if isempty(strtrim(inner))
    return
end

% TWO ENVELOPE FAULTS, NOT ONE, AND ONLY THE FENCE WAS HANDLED. The provider also
% appends a stray closing brace to this schema - three of five live calls on one
% wrist frame returned '{"found":..,"point":..,"confidence":..}}' - and jsondecode
% throws on the whole string, so ok stayed false, hmmWristRefine answered
% STATUS_PROVIDER and the arm descended to the unrefined 40.1 mm point. The
% balance test inside hmmExtractJsonObject is what keeps this envelope handling
% rather than repair: a TRUNCATED object never balances and comes back untouched,
% to be rejected below exactly as before.
inner = hmmExtractJsonObject(inner);

try
    decoded = jsondecode(inner);
catch
    return
end

if ~isstruct(decoded) || ~isscalar(decoded)
    return
end

% found MUST BE PRESENT. Its absence is a malformed answer rather than an implied true: a
% model that omits it has not answered the question that was asked, and defaulting to true
% would turn a non-answer into a grasp correction.
if ~isfield(decoded, 'found') || ~isscalar(decoded.found)
    return
end
found = logical(decoded.found);

if isfield(decoded, 'confidence') && isnumeric(decoded.confidence) && ...
        isscalar(decoded.confidence) && isfinite(decoded.confidence)
    conf = double(decoded.confidence);
end

% A found:false answer is well formed WITHOUT a point, which is exactly what the prompt asks
% for, so the point is only required when the provider claims to have found something.
if ~found
    ok = true;
    return
end

if ~isfield(decoded, 'point')
    return
end
p = double(decoded.point(:))';
if numel(p) ~= 2 || any(~isfinite(p)) || any(p < 0) || ...
        any(p > cfg.camera.pixelNormScale)
    return
end

pt = p;
ok = true;

end

%% ------------------------------------------------------------------------
function tap(instruction, frame, position, status, confidence, pixel, shift, depth, ...
             route, httpStatus, latency, simTime, raw)
%TAP Publish the refinement transaction for the app, exactly as the wide call is tapped.
%   A DISPLAY TAP, AND IT AFFECTS NOTHING. Every value the model acts on has been assigned
%   before this is called. It exists because the refinement is advisory and therefore
%   invisible: a run in which every refinement was rejected looks identical to a run with the
%   feature switched off, and an operator needs to be able to tell those apart while the run
%   is still going. Logged signals do not exist until the simulation stops.
%
%   NO CREDENTIAL PASSES THROUGH HERE. route is which of the four discovery mechanisms
%   resolved, which is a fact about this machine's configuration; the key is a secret and
%   never leaves hmmProviderRequest. raw is the provider's own generated answer, capped at the
%   same 600 characters hmmGroundingTap caps it at - this is a diagnosis, not a log - and it is
%   empty on every provider error path.
answerChars = 600;
try
    % THE POSE AND THE DEPTH ARE RECORDED, NOT JUST THE PICTURE, and that is a lesson paid
    % for twice. With RGB alone, a candidate pixel cannot be back-projected after the fact:
    % answering "what would the refinement have returned if the provider had pointed HERE
    % instead?" costs a 250 s re-run for every guess. Translation, Rotation and Intrinsics are
    % exactly what hmmBackprojectDepth consumes, so with DepthImage alongside them the whole
    % pixel-to-world question becomes offline arithmetic on one saved record. It is also the
    % only in-run evidence of where the camera actually pointed, which is the thing the
    % visible barrel on the tree CANNOT show - see hmmWristRefine's header on the render
    % convention. DepthImage is a 360x640 double, ~1.8 MB of appdata per run; the scalar
    % Depth below stays the one metre reading the refinement actually used.
    rec = struct( ...
        'Instruction', instruction, ...
        'CameraIndex', double(frame.CameraIndex), ...
        'RGB',         frame.RGB, ...
        'DepthImage',  double(frame.Depth), ...
        'Translation', double(frame.Translation(:))', ...
        'Rotation',    double(frame.Rotation(:))', ...
        'Intrinsics',  double(frame.Intrinsics(:))', ...
        'Position',    position(:)', ...
        'Status',      status, ...
        'Confidence',  confidence, ...
        'Pixel',       pixel(:)', ...
        'Shift',       shift, ...
        'Depth',       depth, ...
        'Route',       route, ...
        'HttpStatus',  httpStatus, ...
        'Latency',     latency, ...
        'SimTime',     simTime, ...
        'Raw',         raw(1:min(numel(raw), answerChars)));
    setappdata(0, 'hmmWristRefineTap', rec);
    hmmRefineNotify('answer', rec);
catch
    % A display tap that throws would take the simulation with it, and it is showing an
    % operator a picture. Failing silently is the correct trade here.
end
end
