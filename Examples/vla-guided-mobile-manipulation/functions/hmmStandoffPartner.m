function [partner, found] = hmmStandoffPartner(goal, target, validator, reach)
%HMMSTANDOFFPARTNER The best admissible heading roughly OPPOSITE a chosen stand-off, same spot.
%
%   [partner, found] = HMMSTANDOFFPARTNER(goal, target, validator, reach) returns a pose at
%   goal(1:2) whose heading is at least 120 deg from goal(3), and which passes the very
%   tests hmmStandoffRing applied to goal: the driving validator (footprint rectangle at that
%   heading) and hmmArmCanWork (arm lever inside its band over the whole arrival-tolerance box).
%   Headings are tried closest-to-exactly-opposite first. found is false, and partner equals
%   goal, when no such heading exists.
%
%   WHY A PARTNER AT ALL. A stand-off lies parallel to the furniture it serves, so its arrival
%   cell admits headings near one axis and their reverse - and hmmStandoffRing picks between
%   them by distance from "facing the target" with the lever as tie-break, knowing nothing
%   about the direction the base will ARRIVE from or LEAVE towards. Measured on Run 77's carry
%   leg: both stand-off headings faced away from the route at their end, so the forward drive
%   paid ~180 deg of turning at each end and backed 0.8 m out and 1.3 m in. The partner is
%   what lets hmmPlanBasePath compare the two by their actual plans instead of guessing.
%
%   SAME POSITION, ON PURPOSE. Moving the base changes the reach geometry and the free
%   bearing; holding the position means the partner differs from goal in heading only, so the
%   comparison the planner makes is purely about how the base gets there.
%
%   See also hmmStandoffRing, hmmArmCanWork, hmmSelectBaseGoal, hmmPlanBasePath.

%   Copyright 2026 The MathWorks, Inc.

arguments
    goal      (3,1) double
    target    (3,1) double
    validator
    reach     struct
end

partner = goal;
found   = false;

nY  = max(round(double(reach.nYaw)), 1);
psi = wrapToPi((0:nY - 1) * 2 * pi / nY);
[~, ord] = sort(abs(psi));
minSep = 2 * pi / 3;

for p = psi(ord)
    cand = [goal(1:2); wrapToPi(goal(3) + pi + p)];
    if abs(angdiff(goal(3), cand(3))) < minSep
        continue
    end
    if ~isStateValid(validator, cand')
        continue
    end
    if ~hmmArmCanWork(cand, target, reach)
        continue
    end
    partner = cand;
    found   = true;
    return
end
end
