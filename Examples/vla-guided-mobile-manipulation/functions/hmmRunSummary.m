function s = hmmRunSummary(out, cfg)
%HMMRUNSUMMARY Reduce one simulation output to the facts that say what happened.
%
%   s = HMMRUNSUMMARY(out, cfg) returns a struct of scalars, short vectors and
%   decoded enum names describing a run of homeMobileManipulator: how the
%   instruction was grounded, what the planner did with it, how far the base
%   drove, whether a grasp was ever validated, and whether the safety monitor
%   latched.
%
%   WHY THIS IS A FUNCTION AND NOT A SCRIPT SECTION. Two callers need the same
%   numbers - the live script's Results section and the app's results panel, both
%   through hmmReportRun. A second copy of the reduction is a chance for the app
%   to report a success the live script would have called a failure. The printing
%   lives in hmmReportRun; this function only measures, so a caller that wants
%   the numbers without the text can have them.
%
%   The fields deliberately include the NEGATIVE evidence. baseDistance and
%   graspValidated exist because a run can reach ReachedStopTime with a perfectly
%   healthy solver, an empty action queue and a robot that never moved, and
%   nothing in the solver's own report distinguishes that from success.
%
%   THE FOUR SUPERVISORY FAILURE CODES ARE STICKY, AND THE FINAL SAMPLE IS THE
%   WRONG THING TO REPORT FOR THEM. A chart's FailureCode is an output of its
%   CURRENT state, so a chart that failed a plan, gave up on the action and
%   returned to IDLE is publishing OK again by the end of the run - and reading the
%   last sample printed "plan failure OK" for a run whose whole story was an arm
%   plan that could not be found. The failure is what happened; OK afterwards only
%   means the chart moved on. So basePlanFail, armPlanFail, gripFail and taskFail
%   carry the LAST NON-OK code seen, with a companion *Cleared flag saying whether
%   the channel had returned to OK by the end. The perception and reconstruction
%   codes keep the final sample deliberately: those are service ANSWERS rather than
%   chart states, and a later valid answer genuinely supersedes an earlier failure.
%
%   TWO OBJECT POSITIONS ARE REPORTED, NOT ONE, AND THE FIRST IS NOT THE ONE THE ARM
%   USES. objectPos is what Perception grounded from the four room cameras at 2.5-4 m
%   range; refinedPos is what the wrist RGBD camera re-measured at the pre-descent pose,
%   and it is the ObjectTargetRefined bus - not ObjectTarget - that feeds Manipulator
%   and Gripper. For a long time only the coarse bus was named and logged, so every
%   report, plot and app panel showed the coarse point while the arm correctly descended
%   to the refined one, and a reader scoring a run against the report concluded the
%   second look did nothing. It moves the shipped pick 40.09 mm -> 0.99 mm from the
%   mug's body axis. refineDelta is the gap between the two, in metres.
%
%   refineServed MUST BE READ BEFORE refineStatus. GraspRefinement is a triggered
%   subsystem with inherited InitialOutput, so RefineStatus holds 0 until the first
%   trigger - and STATUS_APPLIED is 0. The served id also starts at 0, so a nonzero
%   ID is the only thing separating a successful refinement from one
%   that never ran. This function will not decode the status while the count is 0.
%
%   The three refine fields and refinedPos use loggedIf rather than logged, because a
%   saved SimulationOutput from before those signals were logged does not hold them.
%
%   See also hmmReportRun, hmmEnumName, hmmRefineService.

%   Copyright 2026 The MathWorks, Inc.

arguments
    out
    cfg struct = hmmConfig()
end

s = struct();
s.stopEvent = out.SimulationMetadata.ExecutionInfo.StopEvent;
s.stopTime  = out.SimulationMetadata.ModelInfo.StopTime;

%% Perception: how the instruction was grounded
p = logged(out, 'PerceptionStatus');
s.route       = hmmEnumName(cfg.provider, lastVal(p.Route), 'ROUTE_');
s.numActions  = lastVal(p.NumActions);
s.percepFail  = hmmEnumName(cfg.fail, lastVal(p.FailureCode));
s.latency     = lastVal(p.Latency);
s.requestId   = lastVal(p.RequestId);

%% The action queue the planner was handed
q = logged(out, 'ActionQueue');
s.queueCount = lastVal(q.Count);
verbs        = lastRow(q.Verb);
s.verbs      = string(arrayfun(@(v) hmmEnumName(cfg.queue, v, 'VERB_'), ...
    verbs(1:max(s.queueCount, 0)), 'UniformOutput', false));
s.queueFail  = hmmEnumName(cfg.fail, lastVal(q.FailureCode));
s.overflow   = logical(lastVal(q.Overflow));

%% What perception reconstructed in metres
o = logged(out, 'ObjectTarget');
s.objectValid  = logical(lastVal(o.Valid));
s.objectLabel  = decodeLabel(lastRow(o.Label), lastVal(o.LabelLen));
s.objectPos    = lastRow(o.Position);
s.graspExtent  = lastVal(o.GraspExtent);
s.extentSource = hmmEnumName(cfg.percep, lastVal(o.ExtentSource), 'EXTENT_');
s.objectFail   = hmmEnumName(cfg.fail, lastVal(o.FailureCode));
s.objectCamera = lastVal(o.SourceCamera);
s.objectPixel  = lastRow(o.PixelPoint);
% NumCandidates is the whole story behind AmbiguousObject: zero means the
% reconstruction landed nowhere near a registered actor, several means it landed
% between two. The failure code alone cannot tell those apart.
s.numCandidates = lastVal(o.NumCandidates);
s.objectActor   = lastVal(o.ActorId);

%% And what the wrist made of it on the second look
% THE COARSE POINT ABOVE IS NOT WHAT THE ARM DESCENDS TO, and reading only
% ObjectTarget made every report say it was. Perception's bus is grounded from the
% four room cameras at 2.5-4 m; the wrist RGBD camera takes one frame at the
% pre-descent pose and hmmRefineService re-measures the grasp point from its aligned
% depth channel. The result is assigned into Position by the ObjectTargetRefined
% BusAssignment at root, and THAT bus - not this one - feeds Manipulator and Gripper.
% Both are logged now, so the pair below is the before and after of the second look:
% on the shipped run 40.09 mm -> 0.99 mm from the mug's body axis.
%
% TaskPlanner keeps the coarse bus on purpose; see the block comment on
% ObjectTargetRefined for why refining its base goal would be an anachronism.
g = loggedIf(out, 'ObjectTargetRefined');
if isempty(g)
    % A run recorded before the refined bus was named and logged. Say so rather than
    % reporting the coarse point twice, which would read as a refinement that moved
    % nothing - the one wrong conclusion this whole section exists to prevent.
    s.refinedPos   = [];
    s.refineDelta  = [];
else
    s.refinedPos  = lastRow(g.Position);
    s.refineDelta = norm(s.refinedPos - s.objectPos);
end

% RefineStatus and RefineShift come straight off the GraspRefinement subsystem rather
% than out of the bus, because they describe the TRANSACTION and not the object: a
% refinement can be declined for five distinct reasons and every one of them leaves
% Position holding the coarse point, indistinguishable from a refinement that agreed.
r1 = loggedIf(out, 'RefineStatus');
r2 = loggedIf(out, 'RefineShift');
r3 = loggedIf(out, 'ServedRefineId');

% HOW MANY SECOND LOOKS ACTUALLY HAPPENED, and this is not a nicety. GraspRefinement is
% a TRIGGERED subsystem whose outports hold when disabled with InitialOutput inherited,
% so RefineStatus sits at 0 until the first trigger - and cfg.refine.STATUS_APPLIED IS
% 0. Read alone, a run in which the wrist never fired reports a clean APPLIED. The
% served id holds 0 too but becomes nonzero when a transaction is served, so it is the signal
% that separates "refined successfully" from "never asked". Read the status only when
% this is nonzero.
if isempty(r3)
    s.refineServed = [];
    s.refineRequestId = [];
else
    ids = asMatrix(r3);
    % IDs can skip values when the planner invalidates a previous action's result.
    s.refineServed = numel(unique(ids(ids > 0)));
    s.refineRequestId = lastVal(r3);
end

if isempty(r1)
    s.refineStatus = "not logged";
elseif isequal(s.refineServed, 0)
    s.refineStatus = "no transaction served";
else
    s.refineStatus = string(hmmEnumName(cfg.refine, lastVal(r1), 'STATUS_'));
end
if isempty(r2)
    s.refineShift = [];
else
    % The largest shift the wrist ever asked for, not the final sample: the signal
    % returns to 0 between transactions, so the end of the run reads as no shift.
    s.refineShift = max(asMatrix(r2), [], 'omitnan');
end

t = logged(out, 'PlaceTarget');
s.placeValid = logical(lastVal(t.Valid));
s.placePoint = lastRow(t.Point);
s.placeFail  = hmmEnumName(cfg.fail, lastVal(t.FailureCode));

%% What the planner made of it
r = logged(out, 'TaskResult');
s.outcome     = hmmEnumName(cfg.sup.out, lastVal(r.Outcome));
s.plannerMode = hmmEnumName(cfg.sup.pl, lastVal(r.PlannerMode));
s.numDone     = lastVal(r.NumDone);
s.numFailed   = lastVal(r.NumFailed);
[s.taskFail, s.taskFailCleared] = stickyFail(r.FailureCode, cfg);
s.elapsed     = lastVal(r.ElapsedTime);
% The whole mode history, not just the end state: a planner that reached PICK and
% fell back to RECOVER looks identical at t = tEnd to one that never left GROUND.
s.modeVisited = string(arrayfun(@(v) hmmEnumName(cfg.sup.pl, v), ...
    unique(asMatrix(r.PlannerMode))', 'UniformOutput', false));

%% Did the robot move
x = logged(out, 'RobotState');
pose         = asMatrix(x.BasePose);
s.time       = x.BasePose.Time;
s.basePose   = pose;
s.baseStart  = pose(1, :);
s.baseEnd    = pose(end, :);
s.baseDistance = sum(vecnorm(diff(pose(:, 1:2)), 2, 2));
% PER AXIS, BECAUSE THE THREE AXES HAVE THREE DIFFERENT BOUNDS. BaseTwist is
% [vx vy omega], bounded by cfg.base.vxMax, vyMax and omegaMax - 0.80, 0.80 and 1.50.
% A single max over all three reported the yaw rate, which is allowed to be nearly
% twice the linear bound, and hmmReportRun then compared it against vxMax alone: the
% run that printed "peak base twist 1.38 against the 0.80 m/s vx bound" was a base
% turning at 1.38 rad/s well inside its 1.50 limit, reported as a 70% overshoot that
% never happened. 1x3, in bus order, so the caller can pair each with its own bound.
s.baseTwistMax = max(abs(asMatrix(x.BaseTwist)), [], 1);

joints        = asMatrix(x.ArmJointPos);
s.armJoints   = joints;
s.armTravel   = sum(sum(abs(diff(joints)), 2));
s.armStart    = joints(1, :);
s.armEnd      = joints(end, :);

s.aperture       = asMatrix(x.GripperAperture);
s.apertureRange  = [min(s.aperture) max(s.aperture)];
s.graspValidated = any(asMatrix(x.IsGrasped) ~= 0);

% THE ACTOR THAT WAS HELD, NOT THE ONE HELD AT THE END, AND THE DIFFERENCE COST SEVERAL
% SESSIONS OF SUSPICION. This was lastVal(AttachedActorId). On a run that completes the
% transfer the actor is RELEASED, so the final sample is 0 by design - and the report printed
% "grasp validated 1 (attached actor 0)" beside it, which reads as a grasp that validated
% without ever attaching. It was chased as an open defect more than once. The measurement that
% settled it: on the reference run AttachedActorId is 12 from t = 7.81 s to 20.58 s, exactly
% the carry window, and 0 either side of it.
%
% s.graspValidated one line above already takes `any` over the whole series for this reason;
% this field now matches it. attachedActorFinal keeps the end state for anyone who wants it -
% a NON-zero value there is the interesting case, because it means the run stopped still
% holding something.
attachedSeries      = asMatrix(x.AttachedActorId);
heldIdx             = find(attachedSeries ~= 0);
s.attachedActor     = 0;
s.attachedActorHeld = [NaN NaN];
if ~isempty(heldIdx)
    s.attachedActor     = attachedSeries(heldIdx(end));
    s.attachedActorHeld = [s.time(heldIdx(1)) s.time(heldIdx(end))];
end
s.attachedActorFinal = lastVal(x.AttachedActorId);
s.tool           = asMatrix(x.ToolTranslation);
s.actor          = asMatrix(x.ActorTranslation);

%% Safety, and the physical plausibility of the plant
f = logged(out, 'SafetyStatus');
s.eStop        = any(asMatrix(f.EStop) ~= 0);
s.eStopCause   = hmmEnumName(cfg.sup.cause, lastVal(f.Cause));
s.eStopFirst   = firstTrueTime(f.EStop);

% MODE AND STATE ARE TWO DIFFERENT ENUMERATIONS AND EACH NEEDS ITS OWN LIST. Every
% status bus carries both: Mode is where the chart is (cfg.sup.base/arm/grip), State is
% how the current request is going (cfg.sup.st - IDLE, BUSY, DONE, FAILED). Naming one
% through the other's list fails SILENTLY, because the four State values are all in
% range of the longer Mode lists and so all come back with a plausible name. It was
% reading State and printing Mode names, which is why an 8 s run whose arm had reached
% Mode STOPPED reported "arm states IDLE AWAIT_PLAN APPROACH" (State 0/1/3) and a
% gripper that ended in State FAILED reported "VALIDATED" - a failure rendered as the
% success it is the opposite of. Both histories are kept now, each through its own list.
b = logged(out, 'BaseStatus');
s.baseModes = uniqueNames(asMatrix(b.Mode), cfg.sup.base);
s.baseStates = uniqueNames(asMatrix(b.State), cfg.sup.st);
s.baseStalled = any(asMatrix(b.Stalled) ~= 0);
[s.basePlanFail, s.basePlanFailCleared] = stickyFail(b.FailureCode, cfg);

a = logged(out, 'ArmStatus');
s.armModes = uniqueNames(asMatrix(a.Mode), cfg.sup.arm);
s.armStates = uniqueNames(asMatrix(a.State), cfg.sup.st);
[s.armPlanFail, s.armPlanFailCleared] = stickyFail(a.FailureCode, cfg);

g = logged(out, 'GripperStatus');
s.gripModes = uniqueNames(asMatrix(g.Mode), cfg.sup.grip);
s.gripStates = uniqueNames(asMatrix(g.State), cfg.sup.st);
[s.gripFail, s.gripFailCleared] = stickyFail(g.FailureCode, cfg);

% Grounding: the chassis actor's z is the plant's own answer to "is the robot
% resting on the floor". A drifting or negative value is a body falling through
% the scene, which a solver that reaches StopTime will never complain about.
s.actorHeight = [min(s.actor(:, 3)) max(s.actor(:, 3))];
s.toolHeight  = [min(s.tool(:, 3)) max(s.tool(:, 3))];

end

%% ------------------------------------------------------------------------
function v = logged(out, name)
%LOGGED One logged bus, as a struct of timeseries.
v = out.logsout.get(name).Values;
end

function v = loggedIf(out, name)
%LOGGEDIF One logged signal if the run has it, [] if it does not.
%   WHY THIS EXISTS RATHER THAN CALLING logged AND CATCHING. Simulink's get returns an
%   EMPTY for a name it does not hold, so logged's out.logsout.get(name).Values errors
%   on the dot rather than returning nothing - and it errors with a message about a
%   bad index, which reads like a corrupt record rather than a signal that was simply
%   not logged at the time. A SimulationOutput saved before ObjectTargetRefined,
%   RefineStatus and RefineShift were logged does not hold them at all, so every
%   caller that opens such a run would die on the newest fields. Only use this for
%   signals an older saved run may lack; the rest stay on logged, where a missing
%   signal SHOULD be a loud failure.
v = [];
h = out.logsout.get(name);
if ~isempty(h)
    v = h.Values;
end
end

function M = asMatrix(ts)
%ASMATRIX Logged samples as [nT k], whichever way Simulink laid them out.
%   A logged [k 1] bus element arrives either as [nT k] or as [k 1 nT] depending
%   on the element's dimensions, and reading the wrong one silently returns k
%   time steps of one channel - data-shaped nonsense that plots without error.
d = ts.Data;
n = numel(ts.Time);
if ndims(d) == 3
    M = reshape(permute(d, [3 1 2]), n, []);
elseif size(d, 1) == n
    M = reshape(d, n, []);
else
    M = reshape(d, n, []);
end
M = double(M);
end

function v = lastVal(ts)
%LASTVAL Final sample of a scalar element.
M = asMatrix(ts);
v = M(end, 1);
end

function v = lastRow(ts)
%LASTROW Final sample of a vector or matrix element, flattened.
M = asMatrix(ts);
v = M(end, :);
end

function [name, cleared] = stickyFail(ts, cfg)
%STICKYFAIL The last non-OK code a chart published, and whether it later cleared.
%   A chart's FailureCode describes the state it is in now, not the run: after an action is
%   abandoned the chart returns to IDLE and publishes OK again. Reporting the final sample
%   therefore erases exactly the event the reader is looking for. The last non-zero code is
%   kept instead - the last, not the first, because a run can fail twice for two different
%   reasons and the later one is the one the chart gave up on.
M = asMatrix(ts);
v = M(:, 1);
k = find(v ~= 0);
if isempty(k)
    name    = hmmEnumName(cfg.fail, 0);
    cleared = false;
else
    name    = hmmEnumName(cfg.fail, v(k(end)));
    cleared = v(end) == 0;
end
end

function t = firstTrueTime(ts)
%FIRSTTRUETIME When a boolean element first went true, NaN if it never did.
M = asMatrix(ts);
k = find(M(:, 1) ~= 0, 1);
if isempty(k)
    t = NaN;
else
    t = ts.Time(k);
end
end

function names = uniqueNames(M, namespace)
%UNIQUENAMES Every distinct state a status element visited, decoded.
names = string(arrayfun(@(v) hmmEnumName(namespace, v), unique(M(:, 1))', ...
    'UniformOutput', false));
end

function label = decodeLabel(bytes, len)
%DECODELABEL The uint8 label buffer as text, truncated to its declared length.
len = max(min(round(len), numel(bytes)), 0);
label = char(bytes(1:len));
end
