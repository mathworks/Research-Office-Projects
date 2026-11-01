function [codes, len, taskId, valid, cancel] = hmmOperatorTask()
%HMMOPERATORTASK The instruction the operator has typed, read while the model runs.
%
%   [codes, len, taskId, valid, cancel] = HMMOPERATORTASK() returns the sentence
%   currently submitted by the operator, encoded for the TaskRequest bus:
%
%       codes    uint8 [instructionChars 1], character codes, zero padded
%       len      uint16, characters actually used
%       taskId   uint32, monotonic submission counter, 0 before the first submit
%       valid    logical, false until something has been submitted
%       cancel   logical, true to abandon the outstanding request and the queue
%
%   IT IS THE MODEL'S ONLY TASK INPUT, AND IT IS READ AT RUN TIME. The block that
%   calls this replaced a Constant holding cfg.task.instruction, which was Valid
%   from t = 0 with TaskId = 1: the model grounded one hard-wired sentence
%   immediately and could never be asked anything else. Nothing else had to
%   change to make the model interactive, because TaskPlanner's Idle -> Grounding
%   transition already fires on [TaskRequest.Valid && TaskRequest.TaskId ~=
%   taskIdRef] - a NEW TASK ID, not a time - and Done -> Idle and Failed -> Idle
%   fire on the same condition. The chart was always able to accept a second
%   instruction; there was simply never a source that could offer one.
%
%   THE COUNTER IS WHAT MAKES A SUBMISSION NEW, NOT THE SENTENCE. taskId
%   increments on every submit even when the words are identical, because the
%   chart compares TaskId against the one it is already working on. Keying on the
%   text instead would make "just try that again" impossible - the commonest thing
%   an operator wants after a failure.
%
%   valid = false COSTS NOTHING. hmmGroundingService returns state 0 for an empty
%   or invalid instruction BEFORE it touches the provider, and Perception is a
%   triggered subsystem fired by the chart's own PerceptionSend, so a model
%   waiting at its home pose for someone to type issues no request and spends no
%   credit. That is why waiting is expressed as "not valid yet" rather than as a
%   timer.
%
%   WHY APPDATA ON THE ROOT. The value has to be readable from inside a
%   simulation step and writable from a UI callback while that simulation runs,
%   which rules out anything scoped to a function. The base workspace would work
%   and is deliberately not used: hmmSetup owns the base workspace and publishes a
%   documented set of names there, and a variable that appears and disappears with
%   an operator's typing does not belong in that set. setappdata(0, ...) is a
%   channel with no such contract to break.
%
%   STALE STATE BETWEEN RUNS IS THE HAZARD, and hmmSubmitTask('clear') is the
%   answer. Root appdata outlives a simulation, so a second run in the same
%   session would otherwise start already carrying the previous run's sentence and
%   ground it at t = 0 - the exact behaviour this function was written to remove,
%   reappearing as a surprise. The app clears before every start.
%
%   See also hmmSubmitTask, hmmGroundingService, hmmConfig.

%   Copyright 2026 The MathWorks, Inc.

nChars = cachedCapacity();

codes  = zeros(nChars, 1, 'uint8');
len    = uint16(0);
taskId = uint32(0);
valid  = false;
cancel = false;

s = getappdata(0, 'hmmOperatorTask');
if isempty(s) || ~isstruct(s)
    return
end

if isfield(s, 'cancel') && ~isempty(s.cancel)
    cancel = logical(s.cancel);
end
if isfield(s, 'taskId') && ~isempty(s.taskId)
    taskId = uint32(s.taskId);
end

if ~isfield(s, 'sentence') || isempty(s.sentence)
    % A cancel with no sentence is a legitimate submission: it is how an operator
    % abandons a task in flight, and it must reach the chart as Valid so the
    % Operating -> Idle transition can see it.
    valid = cancel && taskId > 0;
    return
end

txt = char(s.sentence);
txt = strtrim(txt(:)');
n = min(numel(txt), nChars);
codes(1:n) = uint8(txt(1:n));
len   = uint16(n);
valid = taskId > 0;

end

%% ----------------------------------------------------------------------------
function n = cachedCapacity()
%CACHEDCAPACITY Read the published configuration without rebuilding the rig.
n = double(hmmRuntimeConfig().provider.instructionChars);
end
