function index = hmmNextPendingAction(states, count, pending)
%HMMNEXTPENDINGACTION First pending action within the populated queue slots.
%#codegen
index = uint8(0);
for k = 1:min(double(count), numel(states))
    if states(k) == pending
        index = uint8(k);
        return
    end
end
end
