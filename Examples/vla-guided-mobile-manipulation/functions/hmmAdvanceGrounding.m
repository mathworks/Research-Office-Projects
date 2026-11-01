function [queue, route, state, httpStatus, latency, numActions] = ...
    hmmAdvanceGrounding(previous, task, requestId, simTime, activeIndex, cfg)
%HMMADVANCEGROUNDING Select a queued action without another provider request.
queue = previous;
queue.RequestId = uint32(requestId);
queue.TaskId = task.TaskId;
queue.Timestamp = simTime;
queue.Head = uint8(activeIndex);
route = cfg.provider.ROUTE_NONE;
state = uint8(3);
httpStatus = uint16(0);
latency = 0;
numActions = uint8(0);
if task.Cancel || ~task.Valid
    queue.Valid = false;
    state = uint8(0);
    return
end
if ~previous.Valid || previous.TaskId ~= task.TaskId || ...
        activeIndex < 1 || activeIndex > previous.Count
    queue.FailureCode = cfg.fail.Stale;
    queue.Valid = true;
    return
end
if previous.ItemFailure(activeIndex) ~= cfg.fail.OK
    queue.FailureCode = previous.ItemFailure(activeIndex);
    return
end
state = uint8(2);
numActions = uint8(sum(previous.ItemFailure(1:double(previous.Count)) == cfg.fail.OK));
end
