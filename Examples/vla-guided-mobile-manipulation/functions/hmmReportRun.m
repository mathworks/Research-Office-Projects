function s = hmmReportRun(out, cfg)
%HMMREPORTRUN Print what a run of homeMobileManipulator actually did.
%
%   s = HMMREPORTRUN(out, cfg) prints the reduction hmmRunSummary computes and
%   returns it, so a caller can both read the table and keep the numbers.
%
%   The last block is the one to read first. ReachedStopTime says the solver was
%   happy; it says nothing about whether the instruction was grounded, whether
%   the base drove anywhere or whether the gripper ever closed on anything. Those
%   three lines are the difference between a working example and a model that
%   idles convincingly for the whole run.
%
%   See also hmmRunSummary.

%   Copyright 2026 The MathWorks, Inc.

arguments
    out
    cfg struct = hmmConfig()
end

s = hmmRunSummary(out, cfg);

fprintf('\nGROUNDING\n');
fprintf('  route              %s (latency %.3f s)\n', s.route, s.latency);
fprintf('  actions returned   %d, failure %s\n', s.numActions, s.percepFail);
fprintf('  queue              %d item(s) [%s], failure %s%s\n', s.queueCount, ...
    strjoin(cellstr(s.verbs), ' '), s.queueFail, ...
    repmat(' OVERFLOW', 1, s.overflow));

fprintf('\nRECONSTRUCTION (metres, measured from the scene)\n');
fprintf('  object             valid=%d "%s" at [%.2f %.2f %.2f] cam %d\n', ...
    s.objectValid, s.objectLabel, s.objectPos, s.objectCamera);
fprintf('  pixel             (%.1f, %.1f)\n', s.objectPixel);
fprintf('  association        %d candidate(s), actor %d\n', ...
    s.numCandidates, s.objectActor);
fprintf('  grasp extent       %.0f mm from %s, failure %s\n', ...
    s.graspExtent * 1e3, s.extentSource, s.objectFail);
fprintf('  place              valid=%d at [%.2f %.2f %.2f], failure %s\n', ...
    s.placeValid, s.placePoint, s.placeFail);

% WRIST REFINEMENT, AND WHY IT GETS ITS OWN SECTION AT THREE DECIMALS. The section
% above reports the point grounded from the room cameras at 2.5-4 m; this one reports
% the point the ARM ACTUALLY DESCENDS TO, re-measured from one wrist RGBD frame at the
% pre-descent pose. The difference is millimetric - 40.09 mm -> 0.99 mm from the mug's
% body axis on the shipped run - so the %.2f above would print both points identically
% and the second look would read as a no-op. It is not a no-op; it is the difference
% between the jaws closing on the mug and closing beside it.
fprintf('\nWRIST REFINEMENT (the point the arm descends to)\n');
if isempty(s.refinedPos)
    % An archived run from before the refined bus was logged. Printing nothing here
    % would be worse than saying so: a blank section reads as a refinement that never
    % ran, which is exactly the wrong conclusion.
    fprintf('  refined point      not logged in this run record\n');
else
    fprintf('  refined point      [%.3f %.3f %.3f]\n', s.refinedPos);
    fprintf('  moved from coarse  %.1f mm\n', s.refineDelta * 1e3);
end
if isempty(s.refineServed)
    fprintf('  status             %s\n', s.refineStatus);
else
    % The served count comes first because the status cannot be read without it: a
    % triggered subsystem holds its outports at 0 before the first trigger and
    % STATUS_APPLIED is 0, so "APPLIED" with 0 served would mean the opposite.
    fprintf('  status             %s (%d transaction(s) served)\n', ...
        s.refineStatus, s.refineServed);
end
if ~isempty(s.refineShift)
    % The shift the refinement itself measured, which is capped at
    % cfg.refine.maxShift. It agrees with the delta above when the refinement was
    % APPLIED; when it does not, the cap or the advisory fallback intervened.
    fprintf('  shift requested    %.1f mm (largest of the run)\n', s.refineShift * 1e3);
end

fprintf('\nPLANNER\n');
fprintf('  outcome            %s, mode %s after %.1f s\n', ...
    s.outcome, s.plannerMode, s.elapsed);
fprintf('  actions            %d done, %d failed, failure %s\n', ...
    s.numDone, s.numFailed, cleared(s.taskFail, s.taskFailCleared));
fprintf('  modes visited      %s\n', strjoin(cellstr(s.modeVisited), ' '));
% The MODE line says where each chart went; the STATE line says how the requests it was
% given turned out. Both are printed because either one alone can read as success: an arm
% that visited MOVING_FREE and STOPPED went somewhere and then was halted, and only State
% FAILED says the request it was serving did not complete.
fprintf('  base modes         %s -> states %s (plan failure %s)\n', ...
    strjoin(cellstr(s.baseModes), ' '), strjoin(cellstr(s.baseStates), ' '), ...
    cleared(s.basePlanFail, s.basePlanFailCleared));
fprintf('  arm modes          %s -> states %s (plan failure %s)\n', ...
    strjoin(cellstr(s.armModes), ' '), strjoin(cellstr(s.armStates), ' '), ...
    cleared(s.armPlanFail, s.armPlanFailCleared));
fprintf('  gripper modes      %s -> states %s (failure %s)\n', ...
    strjoin(cellstr(s.gripModes), ' '), strjoin(cellstr(s.gripStates), ' '), ...
    cleared(s.gripFail, s.gripFailCleared));

fprintf('\nSAFETY\n');
if s.eStop
    fprintf('  e-stop             LATCHED at t = %.2f s, cause %s\n', ...
        s.eStopFirst, s.eStopCause);
else
    fprintf('  e-stop             clear\n');
end

fprintf('\nPHYSICAL SANITY\n');
fprintf('  chassis actor z    %.3f .. %.3f m  (grounded means near zero, not drifting)\n', ...
    s.actorHeight);
fprintf('  tool z             %.3f .. %.3f m\n', s.toolHeight);
% Each axis against ITS OWN bound. Printed as fractions as well as values because 1.38
% against 1.50 and 1.38 against 0.80 are the same number and opposite conclusions.
bounds = [cfg.base.vxMax, cfg.base.vyMax, cfg.base.omegaMax];
fprintf('  peak base twist    vx %.2f/%.2f  vy %.2f/%.2f m/s  wz %.2f/%.2f rad/s  (%.0f%% %.0f%% %.0f%% of bound)\n', ...
    [s.baseTwistMax; bounds], 100 * s.baseTwistMax ./ bounds);

fprintf('\nDID ANYTHING HAPPEN\n');
fprintf('  stop event         %s\n', s.stopEvent);
fprintf('  base travelled     %.3f m, from [%.2f %.2f %.1f deg] to [%.2f %.2f %.1f deg]\n', ...
    s.baseDistance, s.baseStart(1:2), rad2deg(s.baseStart(3)), ...
    s.baseEnd(1:2), rad2deg(s.baseEnd(3)));
fprintf('  arm joint travel   %.2f rad summed over all six joints\n', s.armTravel);
fprintf('  aperture           %.0f .. %.0f mm\n', s.apertureRange * 1e3);
% The attached actor is reported as the one that was HELD and the window it was held for, not
% as the value at the final sample. A completed transfer releases the object, so the final
% sample is 0 by design and printing it here read as "validated but never attached" - see the
% note at this field in hmmRunSummary. A non-zero FINAL value is the anomaly, so it is printed
% only when it occurs.
if s.attachedActor == 0
    fprintf('  grasp validated    %d (no actor ever attached)\n', s.graspValidated);
else
    fprintf('  grasp validated    %d (actor %d held %.2f .. %.2f s)\n', ...
        s.graspValidated, s.attachedActor, s.attachedActorHeld);
end
if s.attachedActorFinal ~= 0
    fprintf('  STILL HOLDING      actor %d at the final sample - the run did not release it\n', ...
        s.attachedActorFinal);
end

end

%% ------------------------------------------------------------------------
function txt = cleared(name, wasCleared)
%CLEARED A failure code, marked when the chart had returned to OK by the end of the run.
%   The mark matters: 'Unreachable' and 'Unreachable (cleared)' are a run that ended in the
%   failure and a run that gave up on that action and carried on, which are different
%   outcomes and used to print identically once the code had been reset.
txt = name;
if wasCleared
    txt = [name ' (cleared)'];
end
end
