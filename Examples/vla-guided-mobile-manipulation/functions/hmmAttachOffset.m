function [v, actorName] = hmmAttachOffset(cfg, toolInWorld, actorId)
%HMMATTACHOFFSET The relative translation that makes the render's attach a no-op.
%   v = HMMATTACHOFFSET(cfg, toolInWorld, actorId) returns the translation, in the
%   TOOL frame, that holds the named prop exactly where it already is at the instant
%   the jaws close, and writes it onto the Simulation 3D Robot block of the running
%   model. toolInWorld is the world pose of the attach parent - tcp_frame - at jaw
%   closure; actorId is ObjectTarget.ActorId, the index the grounding chose into
%   hmmActorNames. With no model running it computes the value and writes nothing,
%   so it can be called offline.
%
%   WHY A PARAMETER DECIDES THIS AT ALL. Simulation 3D Robot's attach, in the
%   toolbox's own robotics.slsim3d.internal.util.TargetGraspUtils/attachTargetActor:
%
%       targetActor.attachTo(endEffectorActor);
%       targetActor.Translation = sim3d.utils.Math.posToUnreal( ...
%           obj.TargetGraspOptions.TargetEndEffectorTranslation, 'matlab');
%
%   The physical relative pose at the instant the jaws close is discarded and
%   replaced by a block parameter. The engine does not simulate contact, so there is
%   no grip to preserve the pose for it; whatever that parameter says is where the
%   mesh goes. Shipped as [0 0 0] it pins the prop's ORIGIN to tcp_frame's origin,
%   which is the upward snap at the pick and the float at the release.
%
%   WHAT THIS DROPS, AND IT IS THE WHOLE POINT. hmmCarryTranslation answers the same
%   question by PREDICTION, from the manifest extent, a measured pivot offset and an
%   ASSUMED canonical grasp yaw, and its own header concedes the yaw assumption alone
%   leaves up to 14.5*sin(30 deg) = 7.25 mm uncorrected on a plus-or-minus-30-degree
%   candidate. On top of that it is built from the object position the PERCEPTION
%   pipeline believes, so it also carries the perception error - 63.8 mm of
%   refinement shift on the white-mug run of 2026-10-02, whose log shows the retune
%   firing correctly and the mug jumping anyway. This function assumes nothing: no
%   extent, no pivot, no canonical yaw, no object height, no belief about where the
%   object is. It is the same expression for a mug, a bottle and a cutting board.
%
%   WHERE THE PROP'S TRUE POSE COMES FROM, AND WHY THAT IS NOT A LEAK. The hard line
%   this project draws is that a true object pose must never reach the CONTROL path -
%   not grounding, not the arm plan, not ObjectTarget - because the moment the robot
%   is told where the object really is, the VLA pipeline the example exists to
%   demonstrate becomes decoration. The attach is not the control path: it is the
%   PLANT, standing in for contact physics the engine does not have, and a plant is
%   precisely the thing allowed to know the truth about its own scene. Nor is the
%   number even a measurement here. hmmSpawnProps creates each prop with
%   'Translation', items(k).position - the manifest verbatim - as a child of a parent
%   that sits at the origin with no rotation, with no Physics and so nothing to
%   settle it. The prop's pose before the grasp IS its spawn pose, a static scene
%   fact the plant authored. Nothing in this function is ever returned into the plan:
%   the value leaves by set_param and by nothing else, and hmmScreenArmPlan ignores
%   both outputs.
%
%   THE MESH PIVOT DOES NOT MATTER, which is why no mesh is read. The attach moves
%   the prop's ACTOR ORIGIN, and the actor origin is the point hmmSpawnProps placed.
%   Pinning that point back to the value it was placed at leaves the drawn mesh
%   exactly where it was drawn, wherever inside the mesh the FBX put its pivot and
%   whether or not the manifest extent spans a handle.
%
%   THE TOOL FRAME, NOT THE WORLD. The parameter is a vector in the attach parent's
%   own rotated frame. The evidence is the sign of the vertical term that shipped:
%   the carried mug was held 21.1 mm BELOW tcp_frame by a POSITIVE 0.0211, and the
%   tool z axis points down through cfg.plan.approachPitch, so a world-axis reading
%   would have floated the prop above the hand instead.
%
%   AND THE y AXIS IS NEGATED ON THE WAY IN, which is a defect in the toolbox's
%   attach and the last of the jump the user kept reporting. attachTargetActor does:
%
%       targetActor.Translation = sim3d.utils.Math.posToUnreal( ...
%           obj.TargetGraspOptions.TargetEndEffectorTranslation, 'matlab');
%
%   posToUnreal(v, 'matlab') returns [v(1) -v(2) v(3)] - measured, not assumed:
%   [0.1 0.2 0.3] comes back [0.1 -0.2 0.3], and it does not rescale. But
%   sim3d.Actor.Translation is read in MATLAB convention, so that conversion is one
%   frame change too many and the net effect is a plain negation of y. The proof is
%   the engine's OWN stored link transforms, captured by the hmmWristRefineTap
%   witness: wrist_2_link reads [0 -0.0997 0] and the true tree holds
%   [0 -0.0997 0] while the mirrored tree holds [0 +0.0997 0]; wrist_3_link and
%   left_inner_finger_pad agree the same way. A y-flipped property could not match
%   the unflipped tree on three links whose y offsets have opposite signs.
%
%   So the prop is placed at [v(1) -v(2) v(3)] and the visible jump is 2*|v(2)|
%   along the tool y axis - 10.8 mm on the run of 2026-10-04, 19.3 mm on the run
%   before it. That is why the magnitude looked frozen while the DIRECTION rotated
%   between runs: the error is a fixed multiple of one component of a vector
%   expressed in a frame that turns with the hand.
%
%   WHY THIS WENT UNSEEN FOR SO LONG. The 40 s measurement that concluded
%   "carryTranslation is applied exactly" was taken while the parameter was
%   [0 0 0.0211] - v(2) was ZERO, so the negation had nothing to act on, and the
%   lateral residual duly measured 0.01 mm. The bug became visible only once the
%   lateral terms became nonzero, which is exactly when the pivot retune and then
%   this function started writing them.
%
%   THE COMPENSATION IS PRE-NEGATION, in writeBlock and nowhere else: the block is
%   handed v .* [1 -1 1] so the toolbox's conversion cancels and the prop lands at
%   v. It is a property of the attach, identical for every prop and every pose, so
%   it is not a value keyed to an object. v itself - the quantity this function
%   documents, returns and taps - stays the true relative translation.
%
%   THE ERROR BUDGET, which is now the only error left. toolInWorld is the COMMANDED
%   closure pose, solved here; the arm tracks the commanded descent to 0.6 mm, the
%   base has arrived and settled before a pick plan is requested, and the tree this
%   FK runs on is the tree the render block is given. There is no perception term and
%   no yaw term left in it.
%
%   WHAT IT DELIBERATELY DOES NOT FIX. Freezing the real relative pose makes a bad
%   grasp VISIBLE: a mug straddled 15 mm off centre stays 15 mm off centre in the
%   hand and is placed 15 mm off, where the old snap hid exactly that error by
%   moving the mesh onto the tool. An error along the jaw-opening axis is invisible
%   to the eye anyway - the jaws open about 70 mm, so a 15 mm offset still reads as a
%   clean straddle - which is why the pick could look right and the attach look
%   wrong. Rotation is left alone: eeTargetRot stays [0 0 0] and the toolbox's own
%   end-effector self-rotation compensation stands.
%
%   A PROP ALREADY MOVED IN THIS RUN is out of scope and would need the engine's own
%   pose rather than the manifest's - sim3d.World.getWorld(modelName) resolves live
%   mid-simulation and w.Actors is keyed by the padded actor name. Nothing in the
%   task set picks the same prop twice, so that route is named here and not taken.
%
%   See also hmmCarryTranslation, hmmScreenArmPlan, hmmSpawnProps, hmmActorNames.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg struct
    toolInWorld (4, 4) double
    actorId (1, 1) double
end

v         = [];
actorName = '';

% NOTHING HERE MAY THROW, because the caller is the arm planner and this correction is
% cosmetic. An error escaping an extrinsic call stops a twenty-minute simulation over
% where a mesh is drawn, so a failure degrades to the predicted offset that was on the
% block already - the behaviour this project had before this function existed. It
% degrades LOUDLY though: a warning is visible, suppressible and not fatal, and the
% missing log line from writeBlock is the second tell.
try
    items = hmmSceneManifest();
    if actorId < 1 || actorId > numel(items) || ~all(isfinite(toolInWorld(:)))
        return
    end

    % THE PADDING COMES OFF FOR THE LOG AND NOWHERE ELSE. hmmActorNames pads every
    % name to a fixed width because the plant addresses actors with a fixed-width
    % uint8 row; the pad character is an underscore, so trailing underscores - not
    % whitespace - are what has to go, and the manifest key is what is left.
    names     = hmmActorNames(cfg);
    actorName = regexprep(char(names(actorId)), '_+$', '');

    % The pose hmmSpawnProps gave this actor, in world coordinates, which is still its
    % pose because nothing in the scene moves a prop before it is grasped.
    pProp = items(actorId).position(:);

    R = toolInWorld(1:3, 1:3);
    v = (R' * (pProp - toolInWorld(1:3, 4)))';

    % THE TAP IS NOT BELT-AND-BRACES, it is the only record that survives. The run of
    % 2026-10-03 proved this function had written - the vertical term it left on the
    % block, 7.735 mm, is 0.112 mm from the 7.6233 mm hmmCarryTranslation can ONLY
    % ever produce - and yet neither fprintf below reached the log or the captured
    % stdout, because the app harvests the command window into its own console and
    % that console goes deaf during a driven run. So the number this function exists
    % to report, the distance from the prediction it replaces, had to be inferred
    % instead of read. appdata is written by the same extrinsic call and read back
    % after the run - the route hmmWristRefineTap already uses - so it cannot be
    % starved by a timer.
    [wrote, prev, delta, written] = writeBlock(v, actorName);
    setappdata(0, 'hmmAttachOffsetTap', struct( ...
        'ActorName', string(actorName), 'Offset', v(:)', 'Previous', prev(:)', ...
        'Delta', delta, 'Wrote', wrote, 'Written', written(:)', ...
        'ToolInWorld', toolInWorld));
catch ME
    warning('hmm:attachOffset:failed', ...
        'hmmAttachOffset left the predicted offset in place: %s', ME.message);
end
end

%% ------------------------------------------------------------------------
function [wrote, prev, delta, vBlock] = writeBlock(v, actorName)
%WRITEBLOCK Put the offset on the running model's Simulation 3D Robot block.
%   Returns a word for what happened, the value the block held before, and the
%   distance from it - so the caller can record the outcome whether or not the
%   command window is being harvested by something else.
%
%   NO FAILURE IS SILENT ANY MORE. It is still not fatal - a run that cannot set the
%   parameter is the run this project had before the offset was tuned at all, with a
%   visible snap and nothing else wrong, and letting an error out of an extrinsic
%   call would stop a long simulation over a cosmetic correction. But the first
%   version swallowed every failure into a bare catch, which made "wrote it" and
%   "could not write it" produce the same output: nothing. That is the vacuous-PASS
%   trap this project has already paid for once, and it cost a 14-minute run on
%   2026-10-03 that could not be read. So the catch warns, and each early return
%   names itself. Offline there is no running diagram, and the reason given for not
%   writing is "no model running", which is a fact and not a failure.
%
%   THE MODEL IS FOUND BY ITS STATE AND THE BLOCK BY ITS MASK, neither by a hardcoded
%   path, the same identify-by-mask rule hmmRefineService and the app both use.
%
%   A LITERAL AND NOT A VARIABLE NAME. The mask ships the expression carryTranslation,
%   a base-workspace variable, and rewriting that variable cannot reach a compiled
%   model without a diagram update, which this project never does mid-run.
% Every path below assigns wrote, which is why the analyzer calls this one unused. It
% stays because the cost of being wrong about that is an undefined-output error thrown
% out of an extrinsic call, which stops the simulation - exactly what this function
% promises never to do.
wrote  = "not attempted";      %#ok<NASGU>
prev   = [NaN NaN NaN];
delta  = NaN;
vBlock = [NaN NaN NaN];

try
    mdls = find_system('type', 'block_diagram', 'SimulationStatus', 'running');
    if isempty(mdls)
        wrote = "no model running";
        return
    end
    mdl = mdls{1};

    % getSimulinkBlockHandle, not find_system: find_system on a path that has moved THROWS,
    % and the catch below turned that into a silent skip of the whole write (2026-10-19).
    blk = [mdl '/MobileManipulator/Plant/RobotActor/RobotTree'];
    if getSimulinkBlockHandle(blk) < 0 || ~strcmp(get_param(blk, 'MaskType'), 'Simulation 3D Robot')
        hit = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
            'MaskType', 'Simulation 3D Robot');
        if isempty(hit)
            wrote = "no Simulation 3D Robot block found";
            warning('hmm:attachOffset:noBlock', ...
                'hmmAttachOffset found no Simulation 3D Robot block in %s.', mdl);
            return
        end
        blk = hit{1};
    end

    % WHAT THE BLOCK HELD BEFORE IS THE MEASUREMENT OF THE PREDICTION IT REPLACES.
    % Before the first pick plan the parameter is whatever hmmSetup, the app or the
    % refinement left there, so printing the difference states in the run log how far
    % the predicted offset was from the real one - which is the number the snap was
    % made of. A non-numeric expression (the shipped variable name) simply has no
    % difference to report.
    % PRE-NEGATED IN y SO THE TOOLBOX'S CONVERSION CANCELS. attachTargetActor passes
    % this parameter through posToUnreal(.,'matlab'), which negates y, and then stores
    % the result in sim3d.Actor.Translation, which is read in MATLAB convention - one
    % frame change too many, see the header. Handing it v .* [1 -1 1] makes the round
    % trip the identity, so the prop lands at v. The factor is a property of the
    % attach and not of any prop: it is the same three numbers for a mug, a bottle
    % and a cutting board, in every pose.
    %
    % THE COMPARISON IS DONE IN THE BLOCK'S OWN CONVENTION, because that is what the
    % parameter holds. prev is whatever hmmSetup, the app or the refinement left
    % there, and every one of those writers was subject to the same negation, so
    % delta is a like-for-like distance between two block values rather than a
    % mixture of conventions.
    vBlock = v(:)' .* [1 -1 1];

    had = str2num(get_param(blk, 'eeTargetTrans'));         %#ok<ST2NM> expression, not a scalar
    if isnumeric(had) && numel(had) == 3
        prev  = had(:)';
        delta = norm(vBlock(:) - had(:));
    end

    set_param(blk, 'eeTargetTrans', mat2str(vBlock, 6));
    wrote = "written";

    if isfinite(delta)
        fprintf(['  plan: attach offset frozen for %s - [%.1f %.1f %.1f] mm in the ' ...
            'tool frame, written as [%.1f %.1f %.1f] mm against the attach''s y flip, ' ...
            '%.1f mm from the offset it replaces\n'], ...
            actorName, v(1) * 1000, v(2) * 1000, v(3) * 1000, vBlock * 1000, delta * 1000);
    else
        fprintf(['  plan: attach offset frozen for %s - [%.1f %.1f %.1f] mm in the ' ...
            'tool frame, written as [%.1f %.1f %.1f] mm against the attach''s y flip\n'], ...
            actorName, v(1) * 1000, v(2) * 1000, v(3) * 1000, vBlock * 1000);
    end
catch ME
    wrote = "failed: " + string(ME.message);
    warning('hmm:attachOffset:write', ...
        'hmmAttachOffset could not write the offset, the predicted one stands: %s', ...
        ME.message);
end
end
