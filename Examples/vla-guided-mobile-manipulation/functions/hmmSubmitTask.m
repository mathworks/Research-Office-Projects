function info = hmmSubmitTask(sentence)
%HMMSUBMITTASK Hand the running model a natural-language instruction.
%
%   info = HMMSUBMITTASK(sentence) submits one instruction to whichever run of
%   homeMobileManipulator is in progress, or to the next one to start. The
%   submission counter increments, which is what the TaskPlanner chart reacts to,
%   and info reports what was submitted:
%
%       .taskId     the new counter value
%       .sentence   the sentence as submitted, trimmed
%       .truncated  true when it was longer than the bus can carry
%       .cancel     false
%
%   HMMSUBMITTASK('clear') removes any submission, so a model that starts now
%   waits at its home pose instead of grounding whatever was last typed. Every run
%   must begin with this: root appdata outlives a simulation, and a second run in
%   the same session would otherwise silently re-ground the previous sentence at
%   t = 0.
%
%   HMMSUBMITTASK('cancel') submits a cancellation of the task in flight. The
%   chart's Operating -> Cancelled transition takes it, reports OUT_ABORTED, and the
%   robot stops pursuing the action. Cancel keeps the submitted task's ID; the
%   separate Cancel flag distinguishes it from an instruction.
%
%   THE SENTENCE IS THE ONLY PLACE A SCENE OBJECT MAY BE NAMED. That is section
%   4.1's rule and it is why this function exists as the operator's whole
%   interface: the system must not know which prop the task concerns, so the words
%   are the input and everything after them - which actor, which pixel, which
%   surface, which grasp - is grounded from the cameras. Type what you want in
%   ordinary language; do not encode coordinates here.
%
%   IT DOES NOT WAIT, AND IT DOES NOT VALIDATE THE WORDS. Whether the instruction
%   can be grounded is the provider's answer and the five gates' answer, reported
%   on PerceptionStatus and in the ActionQueue's per-item failures. A submission
%   this function accepted is not a promise that anything is achievable.
%
%   REPLAY WILL NOT ANSWER AN ARBITRARY SENTENCE HONESTLY, so check the route
%   before believing a plan. With cfg.provider.mode at its shipped 'replay',
%   hmmProviderRequest looks for a recording whose name matches a hash of the
%   instruction. A missing recording is reported as ReplayMiss.
%
%   See also hmmOperatorTask, hmmGroundingService, hmmProviderRequest.

%   Copyright 2026 The MathWorks, Inc.

arguments
    sentence (1, :) char
end

txt = strtrim(sentence);

if strcmpi(txt, 'clear')
    if isappdata(0, 'hmmOperatorTask')
        rmappdata(0, 'hmmOperatorTask');
    end
    info = struct('taskId', uint32(0), 'sentence', '', ...
        'truncated', false, 'cancel', false);
    return
end

% The counter continues from whatever is already there rather than restarting, so
% a cleared and re-submitted task still looks new to a chart that is mid-run and
% still holding the previous id in taskIdRef.
isCancel = strcmpi(txt, 'cancel');
if isCancel
    active = getappdata(0, 'hmmOperatorTask');
    taskId = uint32(0);
    if isstruct(active) && isfield(active, 'taskId')
        taskId = uint32(active.taskId);
    end
    setappdata(0, 'hmmOperatorTask', struct( ...
        'sentence', '', 'taskId', taskId, 'cancel', true));
    info = struct('taskId', taskId, 'sentence', '', ...
        'truncated', false, 'cancel', true);
    return
end

prev = getappdata(0, 'hmmOperatorTaskCounter');
if isempty(prev)
    prev = uint32(0);
end
taskId = uint32(prev) + 1;
setappdata(0, 'hmmOperatorTaskCounter', taskId);

nChars = double(hmmRuntimeConfig().provider.instructionChars);
truncated = numel(txt) > nChars;

setappdata(0, 'hmmOperatorTask', struct( ...
    'sentence', txt, 'taskId', taskId, 'cancel', isCancel));

info = struct('taskId', taskId, 'sentence', txt, ...
    'truncated', truncated, 'cancel', isCancel);

end
