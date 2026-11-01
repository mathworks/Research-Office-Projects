function refTime = hmmPadRefTime(refTime, numSamples)
%HMMPADREFTIME Make a fixed-capacity reference time vector strictly increasing.
%   REFTIME = HMMPADREFTIME(REFTIME, NUMSAMPLES) keeps the first NUMSAMPLES
%   entries of REFTIME untouched and replaces the unused tail with a ramp that
%   continues rising past the last valid sample.
%
%   BasePlan.RefTime and ArmPlan.RefTime are fixed-capacity columns - 256 and 128
%   entries - of which only the first NumSamples describe the trajectory. Padding
%   the tail with zeros, which is what a bus ground value gives you, leaves the
%   vector NON-MONOTONIC. That matters because the only consumer of RefTime is the
%   Robotics System Toolbox Polynomial Trajectory block in PathFollowing and in
%   ArmControl, which takes it as its time points, and those must be strictly
%   increasing over the whole vector.
%
%   contopptraj returns a non-uniform time grid, so there is no way to recover
%   the sample times arithmetically from Duration and NumSamples and skip
%   RefTime. Repairing the vector here, where the plan is built, keeps the bus
%   contract sound for every consumer rather than making each one defend itself.
%
%   The contract this establishes:
%
%     * RefTime is strictly increasing over its FULL length
%     * RefTime(1:NumSamples) are the trajectory sample times, starting at 0
%     * RefTime(NumSamples) equals Duration
%     * entries past NumSamples are padding above Duration and are never reached,
%       because consumers clamp their query time to Duration
%
%   When NUMSAMPLES is zero - a plan that failed - the whole vector becomes the
%   ramp starting at PADSTEP. Every entry is then above a t = 0 query, so the
%   reference starts from sample 1, which a failed plan leaves as zeros. A failed plan therefore commands a zero reference
%   rather than an arbitrary one.
%
%   See also HMMPLANBASEPATH, HMMSCREENARMPLAN, HMMDEFINEBUSES.

%   Copyright 2026 The MathWorks, Inc.

% One second per padding entry. The value only has to be positive: the padding is
% never interpolated, it exists solely so the binary search is well posed.
padStep = 1.0;

n = numel(refTime);
nValid = min(max(double(numSamples), 0), n);

% Assign through a linear index and reshape the ramp to the destination, so the
% function is correct whether RefTime arrives as the bus's column or as a row.
if nValid < 1
    refTime(:) = reshape(padStep * (1:n), size(refTime));
    return
end

nPad = n - nValid;
if nPad > 0
    tail = (nValid + 1:n)';
    refTime(tail) = refTime(nValid) + reshape(padStep * (1:nPad), size(tail));
end

end
