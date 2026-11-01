function [n, pose] = hmmPlaceBearings(p, sv, cfg, objExtent, rMin)
%HMMPLACEBEARINGS How many poses on p's place stand-off ring the planner would accept.
%
%   n = HMMPLACEBEARINGS(p, sv, cfg, objExtent) counts the cfg.sup.goalBearings base poses
%   cfg.sup.placeStandoff from the place point p (3x1, world) that the base fits at
%   (validator sv, built with the held object's extent) and from which the arm reaches p
%   while holding an object of objExtent. When that radius has none, it is the count on the
%   first rung of hmmStandoffRadii that has any - the radius hmmSelectBaseGoal will then
%   drive to. Zero means the place would fail BaseGoalOccupied at every radius the arm
%   allows; more is more margin. The seed bearing is +x, which orders
%   hmmStandoffRing's candidates and does not choose them - the grid it searches is absolute
%   about p, which is why perception can ask this without knowing where the base stands.
%
%   AND IT ASKS ABOUT THE ARM AS WELL AS THE MAP. A bearing where the base FITS but the arm
%   cannot reach the place point is not margin. The reach struct is built from cfg exactly as
%   hmmSelectBaseGoal builds it, because the value of this call is that the answer is the
%   one the model will give; see hmmStandoffRing on three callers and one implementation.
%   objExtent goes to the lever band as well as to the validator: the object is in the
%   gripper by the time this stand-off is driven to, so it sets how high the tool is held as
%   well as how wide the base is.
%
%   Shared by hmmReconstructTargets (its place gates) and hmmSlideOnSurface, and by the
%   offline census of place surfaces, so the census and the model cannot disagree.
%
%   See also hmmStandoffRing, hmmArmLeverBand, hmmSlideOnSurface.

%   Copyright 2026 The MathWorks, Inc.

if nargin < 5
    rMin = 0;                       % no rung ruled out in advance
end
[loLever, hiLever] = hmmArmLeverBand(cfg, p(3), double(objExtent(:)));
reach = struct('mountOffset', cfg.pedestal.offset(1:2), ...
               'leverBand',   [loLever, hiLever], ...
               'arrivalTol',  cfg.plan.baseGoalTolerance, ...
               'nYaw',        double(cfg.sup.arrivalYawOffsets));

nB = double(cfg.sup.goalBearings);
n = 0;
pose = [];
if cfg.sup.placeStandoff >= rMin
    goalIn = [p(1) + cfg.sup.placeStandoff; p(2); pi];
    [pose, ~, ~, n] = hmmStandoffRing(goalIn, p, sv, nB, reach);
    if n > 0
        return
    end
end
% THE SAME RADIUS LADDER THE GOAL SEARCH WILL TRY, in the same order, scored on its first
% rung with a free bearing - which is the rung hmmSelectBaseGoal will drive to. Scoring the
% chart's radius alone refused places the base could serve from further out. A caller that
% knows no base pose fits closer than rMin (hmmSlideOnSurface) skips those rungs, which could
% only return 0: the answer is the same, without ~14 ms per ring it cannot use.
for r = hmmStandoffRadii(cfg, loLever, hiLever, cfg.sup.placeStandoff)
    if r < rMin
        continue
    end
    [pose, ~, ~, n] = hmmStandoffRing([p(1) + r; p(2); pi], p, sv, nB, reach);
    if n > 0
        return
    end
end
end
