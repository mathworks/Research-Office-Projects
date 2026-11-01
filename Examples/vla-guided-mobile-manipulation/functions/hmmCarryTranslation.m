function [v, offset, objName] = hmmCarryTranslation(cfg, pivotXY)
%HMMCARRYTRANSLATION Where the held object hangs below the tool, for Simulation 3D Robot.
%
%   v = HMMCARRYTRANSLATION(cfg) returns the 1x3 translation, in metres and in the
%   END-EFFECTOR's own frame, from tcp_frame to the centre of the object the task
%   instruction names. It is the value the Simulation 3D Robot block's "End-effector
%   to target actor translation (m)" field needs, and hmmSetup publishes it to the
%   base workspace as carryTranslation for that block's mask to resolve.
%
%   v = HMMCARRYTRANSLATION(cfg, pivotXY) also corrects the LATERAL snap, from a
%   pivot offset MEASURED in world xy for the object actually in front of the lens -
%   hmmWristRefine's sixth output. Omit it, or pass [0 0], and v(1:2) is zero, which
%   is the behaviour that predates the lateral term and the only honest answer before
%   anything has looked at the object.
%
%   THIS IS NOW THE PREDICTION, NOT THE ANSWER, and the distinction is the fix of
%   2026-10-03. Everything below describes a value DERIVED from belief: the manifest
%   extent, a perceived pivot offset and an ASSUMED canonical grasp yaw. It is still
%   needed and still published, because the mask has to resolve on a bare load and the
%   app has to answer a typed sentence before anything has been planned. But once the
%   pick is planned, the closure pose EXISTS, and hmmAttachOffset overwrites this value
%   with the relative translation that actually obtains at jaw closure - no extent, no
%   pivot, no yaw assumption, no perception error. Read it for the error terms this one
%   carries, which are the reason the white-mug run of 2026-10-02 retuned correctly and
%   still jumped. Anything here that looks like it needs to get more accurate probably
%   does not: it needs to survive until the plan arrives.
%
%   WHY THIS EXISTS: THE SIM3D GRASP DOES NOT PICK AN OBJECT UP WHERE IT IS, IT
%   TELEPORTS IT. robotics.slsim3d.internal.util.TargetGraspUtils/attachTargetActor
%   parents the prop to the end-effector actor and then OVERWRITES its pose -
%
%       targetActor.attachTo(endEffectorActor);
%       targetActor.Translation = sim3d.utils.Math.posToUnreal( ...
%           obj.TargetGraspOptions.TargetEndEffectorTranslation, 'matlab');
%
%   - so the physical relative pose at the instant the jaws close is discarded and
%   replaced by a block parameter. The parameter shipped here as [0 0 0], which pinned
%   the prop's ORIGIN to the tcp_frame origin for the whole carry. Measured on the
%   40 s run that finally exposed it: the mug's underside sat 24.5 mm above the kitchen
%   surface at release, 21.1 mm of which was this snap and only 3.3 mm the honest sum of
%   cfg.plan.releaseClearance and the arm's tracking error. The same 21.1 mm is visible at
%   the PICK as the mug jumping upward the instant the gripper closes. One wrong constant,
%   two symptoms, and neither of them anything to do with the arm - which tracks the
%   commanded descent to 0.6 mm.
%
%   THE OFFSET IS A PROPERTY OF THE OBJECT, NOT OF THE ROBOT, WHICH IS THE WHOLE
%   DIFFICULTY. Every prop's manifest position is its CENTRE - Mug_Red is listed at
%   z 0.5950 on a 0.5500 surface with Extent(3) 0.0900 - while the jaws close where
%   hmmGraspAim aims them and the tool frame itself sits padDrop above the pad contact
%   point. So the centre hangs below the tool by
%
%       offset = hmmGraspAim's rise - Extent(3)/2
%
%   which is the same quantity hmmSetup's carry bound and hmmScreenArmPlan/withHeldObject
%   build, from the same function rather than from three copies of the expression. It was
%   21.12 mm for the mug and 39.89 mm for the Bottle while the aim was a fixed 0.65 of the
%   height; now that the aim IS mid-height the term is the pad drop alone wherever the clamps
%   do not bind - 7.62 mm for the mug - and it still varies with the object, because the pad
%   drop varies with the jaw gap and the clamps bind on anything under 57.5 mm tall. No single
%   constant is right for the manifest, so the value has to be resolved per task.
%
%   IT IS RESOLVED AT SETUP BECAUSE THE INSTRUCTION IS THE ONLY TASK-SPECIFIC DATUM
%   AVAILABLE BEFORE THE FIRST FRAME - not, as this header claimed until it was checked,
%   because the block offers no runtime route. The retracted claim was that the translation
%   reaches TargetGraspUtils from a mask parameter evaluated at compile time and that only
%   ActivateTargetGrasp and TargetActor are inputs. Simulation3DRobotBlock says otherwise in
%   three places: TargetEndEffectorTranslation is declared in a plain properties block and
%   NOT under Nontunable, so it is tunable while the simulation runs; stepImpl calls
%   updateTargetGraspSettings on every step rather than latching at setup; and
%   validateTargetEEOffset branches on IsTargetEEInputEnabled, which promotes the offset to a
%   genuine input port when the mask's eeOffsetInpChekBox is on. Measured on a live run: a
%   set_param on RobotTree's eeTargetTrans mid-simulation is accepted and the run continues.
%
%   SO THE APP RETUNES THIS MID-RUN AND THE OLD WARNING IS GONE. HomeMobileManipulatorApp's
%   onSend calls this function on the sentence the operator actually typed and writes the
%   result onto the block, which is why any object can now be named in any order without the
%   grasped prop jumping. The offset is read at the grasp transition, hundreds of steps after
%   a submission, so there is no race. What remains true is that the value must come from the
%   instruction TEXT: the object a run will pick is grounded by the provider and is not known
%   to a mask.
%
%   THE MATCH IS ALL-TOKENS AND MOST-SPECIFIC-WINS, AND THE GRASPABILITY GATE IS WHAT
%   MAKES IT SAFE. Only props that survive hmmReconstructTargets' gate are candidates -
%   Extent(2) at most cfg.gripper.maxOpenPos - cfg.gripper.apertureTol - which leaves six
%   of the manifest's twenty. A prop is a candidate only if EVERY token of its key appears
%   in the instruction, and among candidates the one with the most tokens wins. On the
%   shipped sentence that is Mug_Red on {mug, red} over Mug on {mug}; Glass_Kitchen matches
%   "kitchen" but not "glass" and is correctly not a candidate. The gate also disposes of
%   the obvious hazard: the instruction says "side table", but no table is graspable, so no
%   table can be resolved as the carried object.
%
%   IT WARNS RATHER THAN GUESSING WHEN IT CANNOT RESOLVE. An unresolved instruction falls
%   back to zero - the pre-fix behaviour, whose failure mode is known and documented above -
%   and says so, because the alternative is a silent placement error of up to 40 mm that
%   looks exactly like a planner fault. There is no conservative direction to round in here:
%   too small floats the object, too large drives it into the surface.
%
%   THE SIGN IS POSITIVE AND THE FRAME IS THE TOOL'S, WHICH IS NOT INTERCHANGEABLE. A
%   Sim3D child's Translation is relative to its parent INCLUDING the parent's rotation -
%   the same fact hmmSpawnProps' header depends on when it notes that manifest coordinates
%   stay world coordinates only while the parent sits unrotated at the origin. tcp_frame's
%   +z points DOWN at a top-down grasp, so a positive z here puts the object below the tool,
%   where it is. sim3d.utils.Math.posToUnreal leaves z alone and flips only y, so no axis
%   bookkeeping survives into the value this returns.
%
%   AND THE VERTICAL TERM IS NOT THE WHOLE STORY, BECAUSE THE PINNED POINT IS THE
%   MESH ORIGIN AND THE GRASP AIMS AT THE VISIBLE BODY. Where those differ the attach
%   drags the prop sideways: Mug_Red's origin is the bounding-box centre of body PLUS
%   handle, 14.5 mm off the body's own axis, so the jaws close correctly and the prop
%   then jumps 14.5 mm the instant they close. v(1:2) corrects that, rotated from world
%   into the tool frame by the canonical grasp orientation below.
%
%   THE LATERAL TERM USED TO BE A TABLE AND IS NOW A MEASUREMENT, which is the whole
%   point of the pivotXY argument. cfg.grasp.pivotOffsetProp / pivotOffsetXY listed six
%   prop names against six hand-measured [x y] rows, and that was a defect on this
%   project's own standing rule - a value keyed to one prop name does not generalise, and
%   the 20-prop manifest is not even the limit, since a scene can be given a prop that no
%   table anticipated. It survived only because the offset was believed unmeasurable at
%   run time. It is not: hmmSegmentTopFace returns the grasp aim and the pivot from ONE
%   look at ONE frame, their difference is this offset, and it comes out [14.95 0.36] mm
%   for the non-red Mug against a tabulated [15.50 0.40] - 0.6 mm apart, on a prop whose
%   row was not the one the table was calibrated from. The table is gone. Nothing here is
%   keyed to a name any more except the VERTICAL term, which is derived manifest geometry
%   rather than a tuned constant.
%
%   THE CORRECTION THAT WAS OWED HERE: the earlier reading of the carry's measured 0.03 mm
%   lateral residual as "do not add a lateral term" was an over-read. The residual
%   proves the block APPLIES this translation faithfully; it says nothing about whether
%   the VALUE is right, because a zero lateral term reproduces itself perfectly.
%
%   See also hmmWristRefine, hmmRefineService, hmmSetup, hmmSceneManifest.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg     struct             = hmmConfig()
    pivotXY (1,2) double       = [0 0]
end

items = hmmSceneManifest();

objName = '';
offset  = 0;
bestExt = [0 0 0];

% THE GRASPABLE SET, gated exactly as hmmReconstructTargets gates it so that this
% function and the model cannot disagree about what is pickable.
apertureLimit = cfg.gripper.maxOpenPos - cfg.gripper.apertureTol;

instruction = lower(cfg.task.instruction);
bestScore   = 0;

for k = 1:numel(items)
    extent = items(k).extent;
    if numel(extent) ~= 3 || extent(2) > apertureLimit
        continue
    end

    % Manifest keys are CamelCase, underscore separated, or both: Mug_Red,
    % SoapDispenser, Glass_Kitchen. Both separators become spaces so the key turns
    % into the words an operator would actually type.
    words  = lower(strrep(regexprep(items(k).name, '([a-z0-9])([A-Z])', '$1 $2'), '_', ' '));
    tokens = strsplit(strtrim(words));

    if ~all(cellfun(@(t) contains(instruction, t), tokens))
        continue
    end
    if numel(tokens) <= bestScore
        continue
    end

    bestScore = numel(tokens);
    objName   = items(k).name;
    bestExt   = extent(:)';
    % THROUGH hmmGraspAim, so this offset cannot disagree with the height the model actually
    % grasps at - the whole quantity is "how far below the tool frame the object's centre sits",
    % which is the grasp rise less half the object. At mid-height the two coincide and the offset
    % is the pad drop alone; it was 21.1 mm when the aim was a fixed 0.65 of the height.
    [~, graspRise] = hmmGraspAim(cfg, extent(:), extent(2));
    offset    = graspRise - 0.5 * extent(3);
end

if isempty(objName)
    msg = sprintf(['No graspable prop in the scene manifest matches the task ' ...
        'instruction "%s", so the end-effector to target translation falls back to ' ...
        'zero. Simulation 3D Robot will then pin the held object''s centre to the ' ...
        'tool frame instead of the few millimetres below it that the grasp establishes, ' ...
        'and the object will be released that far above its support surface. Name the ' ...
        'object the way the manifest does, or set carryTranslation by hand.'], ...
        cfg.task.instruction);
    warning('hmm:carryTranslation:unresolved', '%s', msg);
end

v = [lateral(objName, bestExt, pivotXY, cfg), offset];

end

%% ------------------------------------------------------------------------
function xy = lateral(objName, extent, pivotXY, cfg)
%LATERAL The tool-frame lateral term that keeps the attach from moving the prop.
%   Zero until something has MEASURED the offset, so a run that has not yet looked at
%   the object - and every caller that does not pass a measurement - behaves exactly as
%   it did before this term existed. [0 0] in is [0 0] out, which makes an unmeasured
%   prop and an unmeasurable one the same case rather than two.
xy = [0 0];
if isempty(objName) || ~any(pivotXY)
    return
end

% THE CANONICAL GRASP ORIENTATION, which is what lets a WORLD offset be turned into
% a TOOL one without asking the arm where it is. The jaws open across the object's
% MINOR horizontal axis, so the grasp yaw is decided by which of the manifest extents
% is the longer one, and hmmReconstructTargets/clusterMeasure pins the sign of that
% axis to a convention rather than leaving it to svd. Under that convention the minor
% axis is the major rotated by +90 degrees with the major taken in the x > 0
% half-plane: a prop longer in x gives a jaw axis along +y, hence yaw +pi/2.
%
% THE MEASURED YAW AGREES WITH THE EXTENT HEURISTIC AND IS DELIBERATELY NOT USED. On
% the retained live frame the band's own major axis put the minor-axis yaw at 96.04
% degrees against this expression's 90, with band spans [99.8 69.0] mm reproducing the
% manifest's [101.0 70.0] to about a millimetre - so the heuristic is right for the
% reason it claims, every prop spawning unrotated. Through the rotation the 6-degree
% difference is worth 1.6 mm of lateral error against the 15.5 mm defect being
% corrected, and it is below this frame's own 1.1 mm pixel. Left alone because THIS
% arithmetic is the version a completed live run validated, and because the one
% remaining way to get rid of the reconstruction entirely - taking the yaw from the
% wrist camera's own rotation, which is rigidly mounted to the wrist - would reintroduce
% a camera-to-tool sign convention, and that is the exact class of error this feature
% already lost several runs to.
if extent(1) >= extent(2)
    psi = pi/2;
else
    psi = pi;
end

% Rotate the measured world offset into the tool frame. The full orientation is needed,
% not just the yaw: cfg.plan.approachPitch is pi for a top-down grasp, which inverts
% the tool's y and z, and dropping it would put this term in backwards. R maps TOOL
% into WORLD, so R' maps the measured world-at-spawn vector into the tool frame.
%
% CHECKED AGAINST THE 40 s RUN OF 2026-09-29, where the grasp put tcp_frame at
% [3.8510 -3.0504 0.6162] and the engine held Mug_Red's origin at
% [3.8655 -3.0500 0.5950]. The translation that makes the attach a lateral no-op is
% therefore [-1.60 -14.40] mm in the tool frame, and this expression returns
% [-0.40 -14.50] mm - agreement to 1.2 mm, the residual being the 4.78 degrees
% between the commanded grasp yaw and the one the arm actually held. That check used
% the retired table's Mug_Red row; the measured offset it is now fed differs from that
% row by 0.6 mm, so the agreement carries over unchanged.
%
% The remaining run-time freedom is which of cfg.plan.candidateYaw the screen picks.
% A +-30 degree candidate leaves 14.5*sin(30) = 7.25 mm of the offset uncorrected,
% which is better than the 14.5 mm it corrects; no candidate can make it worse,
% because a 180 degree candidate - the one case that would double the error - is not
% in the list and nothing downstream can introduce one (unwindToward moves joints by
% whole revolutions only, which cannot change the tool's orientation at all).
R = eul2rotm([psi cfg.plan.approachPitch 0], 'ZYX');
d = R' * [pivotXY(:)' 0]';
xy = d(1:2)';
end

% The local padDrop that used to live here is gone. It was deliberately unshared - "so that this
% function has no dependency on either" - and that argument lost to a stronger one: the grasp
% height and the pad drop are one decision, the aim moved from a fixed fraction to a clamped
% mid-height, and a copy of half of that decision is exactly how two files come to command
% different grasps. hmmGraspAim owns both terms now and this function asks it.
