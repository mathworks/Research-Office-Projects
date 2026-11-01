function [position, valid, status, shift, servedId, extent] = ...
    hmmRefineService(frames, task, coarsePosition, coarseExtent, requestId, simTime)
%HMMREFINESERVICE One wrist-refinement transaction, for the GraspRefinement block.
%
%   [position, valid, status, shift, servedId, extent] =
%       HMMREFINESERVICE(frames, task, coarsePosition, coarseExtent, requestId, simTime)
%
%   frames         5x1 CameraFrame struct array, contract order - the wrist is the last
%   task           TaskRequest bus, for the instruction
%   coarsePosition [3 1] ObjectTarget.Position the wide grounding call reconstructed, which is
%                  the SUPPORT SURFACE beneath the object's centre
%   coarseExtent   [3 1] ObjectTarget.Extent, needed because the wrist ranges to the object's
%                  TOP FACE and the two differ by Extent(3) - see hmmWristRefine
%   requestId      uint32 the chart incremented on entering RefineGrasp
%   simTime        s, supervisor time
%
%   position       [3 1] world point the descent should use
%   valid          true ONLY when the refinement was applied; false means "use the coarse
%                  point", and position is then the coarse point unchanged
%   status         cfg.refine.STATUS_*
%   shift          m, how far the point moved
%   servedId       requestId echoed back, which is how the chart knows the answer is in
%   extent         [3 1] the Extent the descent should use - coarseExtent with element 3
%                  replaced by the height this frame MEASURED, and coarseExtent unchanged on
%                  every path that measured nothing
%
%   extent IS RESOLVED HERE RATHER THAN DOWNSTREAM, which is the thin-service rule applied in the
%   one direction it is worth bending. hmmWristRefine returns the measured height as a bare number
%   and 0 for "no measurement", because that is the only honest thing a measuring function can
%   return; turning the pair into a usable Extent needs the coarse one to fall back on, and this
%   function is already holding it. Doing it in the model instead would mean a Switch on a
%   magnitude, and a zero-height Extent reaching a collision box or a grasp clamp is a much worse
%   failure than a stale one.
%
%   ONLY ELEMENT 3 MOVES, AND THE OTHER TWO ARE NOT A DEFECT WAITING TO BE FIXED. The horizontal
%   extents drive the held-object collision box's width and the jaw-aperture gate, and this look
%   measures a top-face plateau, which is the object's silhouette at its widest only for a prism.
%   The height is different: the plateau's rise above the support plane IS the height, for any
%   shape, because both surfaces are the object's own extremes along the one axis the camera is
%   looking down. So the measurement is published for the component it actually measures, and
%   Extent(1:2) stays whatever the wide grounding reconstructed.
%
%   THIS IS THE SERVICE BOUNDARY, AND IT IS DELIBERATELY THIN. hmmWristRefine holds the
%   entire algorithm; this function's only jobs are to pick the wrist frame out of the
%   contract array, unpack the instruction the same way hmmGroundingService does, and echo
%   the request id so the request/serve idiom the rest of the supervisor uses also works
%   here. Keeping it thin is what lets hmmWristRefine be tested without a model.
%
%   IT ALSO HAS ONE SIDE EFFECT, AND THINNESS IS WHY IT BELONGS HERE RATHER THAN DEEPER. The
%   same wrist look that refines the descent point also measures the held object's PIVOT
%   OFFSET, the quantity that decides whether the prop snaps sideways when the jaws close, and
%   that quantity's consumer is a parameter on a running Simulation 3D Robot block. This
%   function runs under coder.extrinsic, so it is the only point on the path from the depth
%   image to that block where a set_param is legal. hmmWristRefine stays a pure function that
%   measures; applyCarryOffset below applies. Every failure of the write is swallowed, because
%   the correction is cosmetic and a stopped simulation is not.
%
%   See hmmCarryTranslation for what replaced the six-row table of prop names this measurement
%   retired, and why a name-keyed constant was a defect rather than a shortcut.
%
%   WHY servedId EXISTS RATHER THAN A "DONE" PULSE. The chart cannot see a pulse it did not
%   catch on the one step it was emitted. BaseStatus, ArmStatus and GripperStatus all settle
%   this the same way - the server echoes the id it served, and the client compares - so the
%   refinement uses the established idiom instead of inventing a second one.
%
%   valid IS NOT "THE TRANSACTION SUCCEEDED". It means "a refined point was accepted".
%   Every other outcome - no key, a malformed answer, found:false, low confidence, an
%   all-sentinel depth patch, an out-of-range depth, a displacement over cfg.refine.maxShift
%   - returns valid false with position set to the coarse point, so the caller needs no
%   knowledge of why. The status code carries the why, for the operator and the plots only.
%
%   See also hmmWristRefine, hmmBackprojectDepth, hmmGroundingService.

%   Copyright 2026 The MathWorks, Inc.

cfg = hmmRuntimeConfig();

position = reshape(double(coarsePosition), 3, 1);
valid    = false;
status   = cfg.refine.STATUS_DISABLED;
shift    = 0;
servedId = uint32(requestId);
extent   = reshape(double(coarseExtent), 3, 1);

% THE WRIST IS THE LAST FRAME IN THE CONTRACT, expressed against nFixed rather than as the
% literal 5 so that adding a ceiling camera does not silently start refining from one.
wrist = frames(double(cfg.camera.nFixed) + 1);

% The same unpacking hmmGroundingService uses, for the same reason: Instruction is a
% fixed-size char array in the bus and only InstructionLen of it is meaningful.
instruction = char(task.Instruction(1:min(double(task.InstructionLen), ...
    numel(task.Instruction))))';
instruction = strtrim(instruction);

% A cancelled or empty request is not refined, and is not an error either. This mirrors
% hmmGroundingService's guard so an unused trigger stays distinguishable from a failure.
if task.Cancel || ~task.Valid || isempty(instruction)
    return
end

[position, status, confidence, pixel, shift, pivotXY, height] = ...
    hmmWristRefine(wrist, position, coarseExtent, instruction, simTime, cfg);  %#ok<ASGLU>

% DEPTHONLY IS A MEASUREMENT TOO - the same top-face segmentation, gates and carry-down as
% APPLIED, seeded from the coarse point instead of the provider's pixel - so it is accepted.
valid = (status == cfg.refine.STATUS_APPLIED || status == cfg.refine.STATUS_DEPTHONLY);

% THE MEASURED HEIGHT ONTO THE EXTENT, GATED ON valid AND ON THE HEIGHT BEING POSITIVE - two
% conditions rather than one because they fail independently. valid false means the refinement was
% rejected and nothing it produced is believed, including this; height 0 on an accepted frame means
% the support plane could not be read, which hmmSegmentTopFace reports separately from whether the
% top face was found. Either way the coarse extent stands, which is the behaviour that predates this
% output.
%
% AND A KNOWN HEIGHT IS ONLY REPLACED BY ONE THAT AGREES WITH IT. The measurement is there to replace
% a GUESS - cfg.percep.extentAssumed, for an object the scene manifest does not describe - and to
% refine a known height, not to overrule it. On 2026-10-12 the depth-only fallback measured the
% 188 mm Bottle at 258 mm (something above its top was in the band); that carried height closed
% every place stand-off and the start pose with it, and the base was stranded holding the bottle.
coarseH  = extent(3);
isGuess  = abs(coarseH - cfg.percep.extentAssumed) < 1e-9;
agrees   = coarseH > 0 && abs(height - coarseH) <= cfg.refine.heightAgreeTol * coarseH;
if valid && height > 0 && (isGuess || agrees)
    extent(3) = height;
elseif valid && height > 0
    fprintf(['  refine: measured height %.1f mm disagrees with the known %.1f mm by more than ' ...
             '%.0f %%, so the known height stands.\n'], 1000 * height, 1000 * coarseH, ...
             100 * cfg.refine.heightAgreeTol);
end

% THE MEASURED CARRY OFFSET IS APPLIED HERE, AND THIS IS THE ONLY PLACE IT CAN BE. The
% quantity is measured inside hmmWristRefine, which is a pure function and must stay one;
% the consumer is a block parameter on the running model. This function is the single point
% between them that may touch a model at all - the GraspRefinement chart declares
% coder.extrinsic('hmmRefineService'), so this body runs in the interpreter and a set_param
% is legal, which is exactly what is NOT true one level down inside a codegen path.
%
% SO "DELIBERATELY THIN" NOW MEANS SOMETHING SHARPER THAN "SHORT". hmmWristRefine measures
% and hmmRefineService applies; the split is what keeps the algorithm testable with no model
% present and the model write confined to a boundary that is already extrinsic by contract.
%
% AND IT IS NOT A RACE. The refinement runs at the coarse pregrasp, the offset is read when
% the grasp transition fires, and the descent plus the approach stand between them - the same
% comfortable margin HomeMobileManipulatorApp's onSend retune relies on, with far more slack.
%
% NOTHING HERE READS A TRUE OBJECT POSE. pivotXY is a difference of two points measured in
% the wrist DEPTH IMAGE. No actor is queried, which is the constraint that governs this
% entire example: the moment the robot is told where the object really is, the perception
% pipeline becomes decoration.
if valid && any(pivotXY)
    applyCarryOffset(cfg, instruction, pivotXY);
end

end

%% ------------------------------------------------------------------------
function applyCarryOffset(cfg, instruction, pivotXY)
%APPLYCARRYOFFSET Write the measured pivot offset onto the Simulation 3D Robot block.
%
%   IT IS NO LONGER THE LAST WRITER, AND THAT IS DELIBERATE. The refinement re-enters
%   PlanPick, so hmmScreenArmPlan runs AFTER this and calls hmmAttachOffset, which
%   replaces this predicted offset with the relative pose that actually obtains at jaw
%   closure. This write is kept because the ordering is a property of the chart rather
%   than of this file: if a run ever closes the jaws on a plan made before the
%   refinement, the pivot correction is still better than nothing. The run log shows
%   both lines and the second one reports how far apart they were.
%
%   EVERY FAILURE HERE IS SWALLOWED, ON PURPOSE. The refinement is advisory and so is this:
%   a run that cannot write the parameter is the run this project had before the lateral term
%   existed, with a visible 15 mm snap and nothing else wrong. Letting an error out of an
%   extrinsic call would stop a 20-minute simulation over a cosmetic correction, and would
%   also break every offline caller of hmmWristRefine's own test path, which has no model.
%
%   THE MODEL IS FOUND BY ITS STATE AND THE BLOCK BY ITS MASK, NEITHER BY A HARDCODED NAME.
%   A running block_diagram is the only model this call can possibly belong to, and the
%   Simulation 3D Robot mask is what makes the block the right one - the same identify-by-mask
%   rule HomeMobileManipulatorApp/carryBlock uses, for the same reason: a path that has moved
%   should find the block, and a wrong block should never be written silently. Offline there
%   is no running diagram, so this returns having done nothing at all.
try
    mdls = find_system('type', 'block_diagram', 'SimulationStatus', 'running');
    if isempty(mdls)
        return
    end
    mdl = mdls{1};

    % getSimulinkBlockHandle, not find_system: find_system on a path that has moved THROWS,
    % and the catch below turned that into a silent skip (2026-10-19).
    blk = [mdl '/MobileManipulator/Plant/RobotActor/RobotTree'];
    if getSimulinkBlockHandle(blk) < 0 || ~strcmp(get_param(blk, 'MaskType'), 'Simulation 3D Robot')
        hit = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
            'MaskType', 'Simulation 3D Robot');
        if isempty(hit)
            return
        end
        blk = hit{1};
    end

    % THE SENTENCE, NOT cfg.task.instruction, because cfg here is hmmConfig's shipped default
    % and the run may have been given anything. The vertical term still comes from the
    % manifest match on that text - it is derived geometry and needs the object's height - so
    % passing the real instruction is what keeps this write from undoing the vertical fix.
    % Its unresolved warning is silenced for the same reason the app silences it: this is a
    % speculative question, and a navigate carries nothing.
    taskCfg = cfg;
    taskCfg.task.instruction = instruction;

    ws = warning('off', 'hmm:carryTranslation:unresolved');
    restoreWarn = onCleanup(@() warning(ws));
    [v, ~, objName] = hmmCarryTranslation(taskCfg, pivotXY);

    % A sentence naming no carriable prop leaves the block alone - there is no offset to be
    % right or wrong about, and zeros would discard whatever setup correctly put there.
    if isempty(objName)
        return
    end

    % A LITERAL AND NOT A VARIABLE NAME. The mask ships the expression carryTranslation, a
    % base-workspace variable, and rewriting that variable cannot reach a compiled model
    % without a diagram update, which this project never does mid-run. The numbers do.
    set_param(blk, 'eeTargetTrans', mat2str(v(:)', 6));

    fprintf(['  refine: carry offset measured for %s - pivot [%.1f %.1f] mm in world, ' ...
        '[%.1f %.1f] mm lateral in the tool frame\n'], objName, ...
        pivotXY(1) * 1000, pivotXY(2) * 1000, v(1) * 1000, v(2) * 1000);
catch
end

end
