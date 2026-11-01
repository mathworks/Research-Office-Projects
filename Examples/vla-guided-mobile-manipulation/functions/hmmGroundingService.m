function [queue, route, state, httpStatus, latency, numActions] = ...
    hmmGroundingService(frames, task, requestId, simTime, queueTemplate, activeIndex)
%HMMGROUNDINGSERVICE One complete grounding transaction, request to queue.
%
%   [queue, route, state, httpStatus, latency, numActions] =
%       HMMGROUNDINGSERVICE(frames, task, requestId, simTime, queueTemplate)
%
%   frames        5x1 CameraFrame struct array, contract order
%   task          TaskRequest struct
%   requestId     monotonic request counter from the model
%   simTime       simulation time of this trigger
%   queueTemplate an ActionQueue type template, or the previously grounded queue
%   activeIndex   optional queued action to reconstruct without a provider call;
%                 zero (the default) starts a fresh grounding transaction
%
%   For a fresh transaction queueTemplate supplies only the element types. For
%   a later action it also supplies the previous queue to hmmAdvanceGrounding.
%   The caller
%   is a MATLAB Function block that must pre-declare the type of an extrinsic
%   call's result, and passing the delayed queue back in means the types come
%   from the ActionQueue bus object and nowhere else. Writing them out here
%   would create a second declaration that could silently drift from the first.
%
%   The transaction is: check the frames are worth sending, pack the four FIXED
%   camera images, issue the request, unwrap the fence, apply gates 1-5, and
%   report. The wrist camera is never sent - it is contract index 5 and the
%   provider is only ever shown the fixed rig.
%
%   FRAMES ARE CHECKED BEFORE THE REQUEST IS SENT. A stale or invalid frame set
%   produces Stale without contacting the provider: grounding a black image
%   costs a paid request and returns coordinates that mean nothing.
%
%   state is 0 idle, 1 pending, 2 complete, 3 failed. Idle is reported for a
%   cancel or an empty instruction, so an unused trigger is distinguishable from
%   a failed one.
%
%   See also hmmProviderRequest, hmmValidateActions, hmmReconstructTargets.

%   Copyright 2026 The MathWorks, Inc.

cfg = cachedConfig();

if nargin >= 6 && activeIndex > 0
    [queue, route, state, httpStatus, latency, numActions] = ...
        hmmAdvanceGrounding(queueTemplate, task, requestId, simTime, activeIndex, cfg);
    return
end

queue            = queueTemplate;
queue.RequestId  = uint32(requestId);
queue.TaskId     = task.TaskId;
queue.Count      = uint8(0);
queue.Head       = uint8(0);
queue.Verb(:)    = cfg.queue.VERB_NONE;
queue.Label(:)   = 0;
queue.LabelLen(:) = 0;
queue.PickPoint(:)  = 0;
queue.PickImage(:)  = 0;
queue.PlacePoint(:) = 0;
queue.PlaceImage(:) = 0;
queue.Confidence(:) = 0;
queue.ExecState(:)  = cfg.queue.EXEC_PENDING;
queue.ItemFailure(:) = cfg.fail.OK;
queue.Overflow   = false;
queue.FailureCode = cfg.fail.OK;
queue.Timestamp  = simTime;
queue.Valid      = false;

route      = cfg.provider.ROUTE_NONE;
state      = uint8(0);
httpStatus = uint16(0);
latency    = 0;
numActions = uint8(0);

%% Nothing to do
instruction = char(task.Instruction(1:min(double(task.InstructionLen), numel(task.Instruction))))';
instruction = strtrim(instruction);
if task.Cancel || ~task.Valid || isempty(instruction)
    return
end

%% Frame admissibility, before any request is issued
nFixed = double(cfg.camera.nFixed);
images = cell(1, 0);
indices = zeros(1, 0);
for k = 1:nFixed
    f = frames(k);
    if ~f.Valid
        continue
    end
    if simTime - f.Timestamp > cfg.safety.staleTimeout
        continue
    end
    images{end + 1} = f.RGB;                                       %#ok<AGROW>
    indices(end + 1) = double(f.CameraIndex);                      %#ok<AGROW>
end

if isempty(images)
    queue.FailureCode = cfg.fail.Stale;
    queue.Valid = true;
    state = uint8(3);
    hmmGroundingTap(instruction, images, indices, queue, route, state, ...
        httpStatus, latency, simTime, cfg, emptyBoxes(cfg), '');
    return
end

%% Request
% There is deliberately no "requesting" state written here. The call below is
% synchronous, so state 1 could never be observed by anything reading this
% function's output - it would be overwritten before the caller saw it.
[raw, route, httpStatus, latency, providerFailure] = ...
    hmmProviderRequest(instruction, images, indices, cfg, 'actions', reaskNote(task.TaskId, requestId, instruction));

if providerFailure ~= cfg.fail.OK
    queue.FailureCode = providerFailure;
    queue.Valid = true;
    state = uint8(3);
    % Tapped on the failure path as well as the success path, because which of the
    % five failure modes occurred is exactly what an operator staring at a robot
    % that did not move needs to be told, and PerceptionStatus does not reach them
    % until the run has stopped.
    %
    % raw IS PASSED ON THIS PATH TOO, AND IT IS THE WHOLE POINT OF PASSING IT. A
    % MalformedJSON raised HERE means the envelope carried no text part, so raw is
    % empty; the same code raised by the gates below comes with the text that failed
    % to decode. The tap is the only place the two can be told apart, because the
    % failure code the model acts on is identical.
    hmmGroundingTap(instruction, images, indices, queue, route, state, ...
        httpStatus, latency, simTime, cfg, emptyBoxes(cfg), raw);
    return
end

%% Gates 1-5
% boxes is the fourth output and it goes NOWHERE NEAR queue. It is Gemini
% Robotics-ER's box_2d for the actions it proposed, carried straight to the
% display tap so the app can outline what the provider was looking at. Nothing
% the model simulates reads it; see hmmValidateActions' header.
[gated, queueFailure, overflow, boxes] = hmmValidateActions(raw, cfg);

queue.Count       = gated.Count;
queue.Verb        = gated.Verb;
queue.Label       = gated.Label;
queue.LabelLen    = gated.LabelLen;
queue.PickPoint   = gated.PickPoint;
queue.PickImage   = gated.PickImage;
queue.PlacePoint  = gated.PlacePoint;
queue.PlaceImage  = gated.PlaceImage;
queue.Confidence  = gated.Confidence;
queue.ItemFailure = gated.ItemFailure;
queue.Overflow    = overflow;
queue.FailureCode = queueFailure;
queue.Valid       = true;

% A rejected action is marked FAILED in place rather than removed, so the queue
% still shows what the provider proposed and why each item was refused.
bad = gated.ItemFailure ~= cfg.fail.OK;
queue.ExecState(bad)  = cfg.queue.EXEC_FAILED;
queue.ExecState(~bad) = cfg.queue.EXEC_PENDING;

admissible = find(~bad(1:double(gated.Count)), 1);
if isempty(admissible)
    queue.Head = uint8(0);
else
    queue.Head = uint8(admissible);
end

numActions = uint8(sum(~bad(1:double(gated.Count))));
if queueFailure == cfg.fail.OK || queueFailure == cfg.fail.QueueOverflow
    state = uint8(2);
else
    state = uint8(3);
end

% A DISPLAY TAP, AND IT AFFECTS NOTHING ABOVE IT. Every value the model acts on has
% already been assigned; this publishes a copy of the transaction - the frames that
% were sent, the decoded actions, the route, the status - to root appdata so that an
% app can show an operator the answer to the sentence they typed while the run is
% still going. Logged signals do not exist until the simulation stops, and the
% frames are on no port at all, so there is no other way to see it in time. It
% returns nothing, it cannot throw, and deleting the three calls to it would change
% no simulation result. No credential passes through it; see its own header.
hmmGroundingTap(instruction, images, indices, queue, route, state, ...
    httpStatus, latency, simTime, cfg, boxes, raw);
rememberPlace(task.TaskId, requestId, instruction, queue, cfg);

end

%% ------------------------------------------------------------------------
function note = reaskNote(taskId, requestId, instruction)
%REASKNOTE Tell the provider why it is being asked the same thing twice.
%   A second FRESH transaction for the same task happens on one path only: TaskPlanner's
%   Recover -> Grounding re-ground after PlaceUnreachable (later queued actions take
%   hmmAdvanceGrounding and never get here). Re-asking unchanged returned the identical island
%   centre twice on 2026-10-12, so the note names the point that was refused. The point is in the
%   provider's own 0..pixelNormScale units, so it can find it in the same image.
%   THE SAME TASK, THE SAME SENTENCE AND A LATER REQUEST, all three. The store outlives a run and
%   requestId restarts with every run, so a fresh run never inherits a note from the last one.
prev = rememberPlace();
note = '';
if isempty(prev) || isempty(prev.point) || prev.taskId ~= taskId || ...
        requestId <= prev.requestId || ~strcmp(prev.instruction, instruction)
    return
end
prevPoint = prev.point;
prevImage = prev.image;
note = sprintf(['A previous answer to this instruction put place_point at [%d, %d] in camera ' ...
    '%d, and the robot could not reach that spot: it can only set an object down within about ' ...
    'half a metre of an edge it can stand at. Choose a different place_point on the same ' ...
    'surface, nearer such an edge, still respecting any location the instruction names.\n\n'], ...
    round(prevPoint(1)), round(prevPoint(2)), prevImage);
fprintf('hmmGroundingService: re-asking task %d with the refused place point named.\n', taskId);
end

%% ------------------------------------------------------------------------
function prev = rememberPlace(taskId, requestId, instruction, queue, cfg)
%REMEMBERPLACE Hold the first transfer's place point of the last fresh transaction.
%   rememberPlace(taskId, requestId, instruction, queue, cfg) stores it; prev = rememberPlace()
%   reads it back, [] before anything has been stored.
persistent held
if nargin == 0
    prev = held;
    return
end
prev = [];
k = find(queue.Verb(1:double(queue.Count)) == cfg.queue.VERB_TRANSFER, 1);
held.taskId = taskId;
held.requestId = double(requestId);
held.instruction = instruction;
if isempty(k)
    held.point = [];
    held.image = 0;
else
    held.point = queue.PlacePoint(:, k)';
    held.image = double(queue.PlaceImage(k));
end
end

%% ------------------------------------------------------------------------
function b = emptyBoxes(cfg)
%EMPTYBOXES The box struct for a transaction that never reached the gates.
%   Right-sized rather than empty so the tap's decode does not have to special-case
%   a stale or provider-failed transaction: every slot simply has no box.
nQ = double(cfg.queue.capacity);
b.PickBox  = nan(4, nQ);
b.PlaceBox = nan(4, nQ);
end

%% ------------------------------------------------------------------------
function cfg = cachedConfig()
%CACHEDCONFIG Effective run configuration, without rebuilding it per trigger.
cfg = hmmRuntimeConfig();
end
