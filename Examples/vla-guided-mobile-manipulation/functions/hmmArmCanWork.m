function ok = hmmArmCanWork(cand, target, reach)
%HMMARMCANWORK Does the arm's lever stay inside its band everywhere the base may stop?
%
%   Not at cand, but over everything cfg.plan.baseGoalTolerance licenses - the 160 mm box
%   INCLUDING its corners, crossed with the yaw extremes - because the base arrives within a
%   tolerance rather than at a point and reach fails outward. The corners are sampled rather
%   than an inscribed disc because the corner is 113 mm out where the edge is 80 mm, and a
%   bound measured on the disc certifies poses the base can legally beat by 33 mm.
%
%   The yaw extremes are load bearing twice over. They move the footprint rectangle, which
%   the validator already accounts for, and they also swing the MOUNT around the base centre,
%   which nothing else in this file would notice.
%
%   ONE IMPLEMENTATION, TWO CALLERS. Lifted out of hmmStandoffRing on 2026-10-08 so that
%   hmmStandoffPartner screens the opposite heading with exactly the test the primary passed;
%   a second copy would let the two certify different poses.
%
%   See also hmmStandoffRing, hmmStandoffPartner, hmmArmLeverBand.

%   Copyright 2026 The MathWorks, Inc.

tol = reach.arrivalTol;
[dx, dy, dth] = ndgrid([-tol(1) 0 tol(1)], [-tol(2) 0 tol(2)], [-tol(3) 0 tol(3)]);

yaw = cand(3) + dth(:)';
% The mount in world, for every licensed pose at once: the base centre offset by the box
% sample, plus the mount offset turned by that pose's own yaw.
mx = cand(1) + dx(:)' + cos(yaw) * reach.mountOffset(1) - sin(yaw) * reach.mountOffset(2);
my = cand(2) + dy(:)' + sin(yaw) * reach.mountOffset(1) + cos(yaw) * reach.mountOffset(2);

lever = hypot(target(1) - mx, target(2) - my);
ok    = all(lever >= reach.leverBand(1)) && all(lever <= reach.leverBand(2));
end
