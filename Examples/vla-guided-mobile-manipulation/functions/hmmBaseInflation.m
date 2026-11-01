function [inflation, physical, margin, carried] = hmmBaseInflation(cfg, carriedExtent)
%HMMBASEINFLATION The two radii a DISC model of the base inflates the map by.
%
%   [inflation, physical, margin, carried] = HMMBASEINFLATION(cfg, carriedExtent)
%
%   physical  what the chassis PHYSICALLY occupies: its footprint radius plus half
%             the widest horizontal extent of whatever it is carrying. Nothing on
%             any path, anywhere, may violate this.
%   margin    the driving margin ON TOP of that: the stowed arm's overhang past the
%             chassis circle plus cfg.safety.plannerMargin. A manipulation
%             stand-off deliberately gives this up.
%   inflation physical + margin.
%   carried   the term the held object contributed, on its own. hmmBaseValidators
%             needs it to grow a RECTANGLE by the same rule this grows a radius by,
%             and returning it here is what keeps that rule in one place.
%
%   THE BASE PLANNER NO LONGER USES THIS TO VALIDATE, AND THAT IS DELIBERATE. A
%   single radius has to be the chassis' CIRCUMSCRIBED circle for a base that can
%   rotate in place, and at 0.468 m the resulting disc needs 0.94 m of opening while
%   the apartment's internal doorways are 0.80 and 0.75 m - so two rooms were
%   unreachable by a robot that fits through both with 0.195 m to spare at each jamb.
%   hmmBaseValidators replaces the disc with the footprint RECTANGLE and the heading;
%   read its header for the measurement and the argument.
%
%   WHAT THIS IS STILL FOR. It is the honest omnidirectional envelope of the base, so
%   it remains the right number wherever a heading is not available or not meaningful:
%   hmmOccupancyFromDepth's self-return mask, hmmSetup's stowed-envelope warning,
%   hmmStandoffRing's distance reasoning, and any question of the form "how much room
%   does this robot need, worst case". It is also what hmmBaseValidators calls for the
%   carried term. It is NOT the planner's validation radius any more, and any caller
%   that wants to know what the planner will accept should ask hmmBaseValidators.
%
%   WHY THIS IS A FUNCTION AND NOT THREE LINES IN EACH CALLER. Two places have to
%   agree about it exactly, and they are not adjacent: hmmPlanBasePath inflates the
%   map and rejects an invalid goal with BaseGoalOccupied, while hmmSelectBaseGoal
%   CHOOSES the goal and has to choose one that will survive that rejection. A
%   selection judged by even a slightly different radius than the planner applies is
%   worse than no selection at all - it hands over a pose it has certified and the
%   planner refuses it anyway, and the failure now has two places to hide instead of
%   one. So the arithmetic is written once.
%
%   THE CARRIED TERM CANCELS OUT OF THE MARGIN, WHICH IS NOT AN OVERSIGHT. A robot
%   holding a wide object is a wider robot, so the object belongs in the physical
%   footprint; it does not make the stowed arm overhang further or the safety margin
%   larger. That asymmetry is what lets hmmPlanBasePath bound its escape by margin
%   alone regardless of what is in the gripper, and hmmBaseValidators inherits it by
%   growing its rectangle with the fourth output rather than by recomputing the rule.
%
%   HALF THE MAX EXTENT, NOT THE HALF-DIAGONAL. The carried object is treated as a
%   disc circumscribing its widest horizontal side, which over-covers a long thin
%   object held end-on and under-covers nothing. Only extents that are actually
%   positive count, so a zeroed extent - the bus's own "carrying nothing" - adds
%   exactly zero rather than a spurious half-cell.
%
%   See also hmmBaseValidators, hmmPlanBasePath, hmmSelectBaseGoal.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg           struct
    carriedExtent (:,1) double
end

carried = 0;
if numel(carriedExtent) >= 2 && any(carriedExtent(1:2) > 0)
    carried = 0.5 * max(carriedExtent(1:2));
end

physical  = cfg.plan.baseRadius + carried;
margin    = cfg.plan.stowedArmRadius + cfg.safety.plannerMargin;
inflation = physical + margin;

end
