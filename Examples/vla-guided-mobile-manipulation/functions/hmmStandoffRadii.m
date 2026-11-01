function rungs = hmmStandoffRadii(cfg, loLever, hiLever, radius)
%HMMSTANDOFFRADII The stand-off radii to try when the chart's own radius has no free bearing.
%
%   rungs = HMMSTANDOFFRADII(cfg, loLever, hiLever, radius) returns base-centre radii from the
%   target, one map cell apart, spanning the arm's lever window [loLever, hiLever] (shifted by
%   the mount's forward offset, cfg.pedestal.offset(1)), ordered by distance from radius - the
%   one the chart asked for - and capped at cfg.sup.standoffMaxRungs. Empty when the window is
%   empty (hiLever < loLever).
%
%   ONE LIST, TWO CALLERS, AND THAT IS THE POINT. hmmSelectBaseGoal searches these radii for
%   the base's goal, and hmmPlaceBearings - which perception uses to accept or refuse a place
%   point - scores a point on the same radii. When perception only knew the chart's 0.48 m
%   and the goal search knew six rungs either side of it, a couch seat the base could serve
%   from 0.57-0.67 m was refused PlaceUnreachable before the goal search was ever asked
%   (2026-10-15); with the cap at 6 the goal search itself stopped at ~0.62 m.
%
%   The cap bounds the worst case, an exhausted ladder, which hmmSelectBaseGoal caches per
%   target; see its notes on cost. The window is 0.07-0.82 m for a 0.40 m surface holding a
%   bottle, 16 rungs.
%
%   See also hmmSelectBaseGoal, hmmPlaceBearings, hmmArmLeverBand.

%   Copyright 2026 The MathWorks, Inc.

rungs    = zeros(1, 0);
cellSize = 1 / cfg.map.resolution;
mountFwd = cfg.pedestal.offset(1);
soLo     = max(loLever + mountFwd, cellSize);
soHi     = hiLever + mountFwd;
if hiLever < loLever || soHi < soLo
    return
end
rungs = soLo:cellSize:soHi;
if isempty(rungs) || soHi - rungs(end) > 1e-9
    rungs = [rungs, soHi];      % the ceiling itself is always a candidate
end
% Ordered by distance from what the chart asked for, so the goal moves as little as the arm's
% own band requires - the same "least change" rule the heading search uses.
[~, ord] = sort(abs(rungs - radius));
rungs    = rungs(ord);
rungs    = rungs(1:min(numel(rungs), cfg.sup.standoffMaxRungs));
end
