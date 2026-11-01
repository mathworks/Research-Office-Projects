function hmmBasePlanTap(bplan, startPose, goalPose, simTime)
%HMMBASEPLANTAP Publish the base path just accepted, for a user interface to draw.
%
%   HMMBASEPLANTAP(bplan, startPose, goalPose, simTime) writes the route the base
%   is about to drive to root appdata under 'hmmBasePlanTap', where an app can poll
%   it while the simulation runs. It returns nothing and it changes nothing.
%
%   WHY A TAP AND NOT A LOGGED SIGNAL. The app's own header used to say the map
%   shows the path the robot DROVE and not the one it PLANNED, because "the RRT
%   waypoints live on a BasePlan bus that is not among the twelve logged ports, so
%   drawing the planned route would mean either a model edit or re-planning here -
%   and a re-plan is a different random tree from the one the robot followed, which
%   would put a confident wrong line on the map." Both halves of that stay true.
%   This is the third option: the plan is published by the function that PRODUCED
%   it, so the line on the map is the actual route, by construction, with no model
%   edit and no second search. hmmGroundingTap established the idiom and this
%   follows it exactly.
%
%   PUBLISHED FROM THE SUCCESS PATH ONLY, which is a deliberate choice rather than
%   an omission. hmmPlanBasePath has five returns and four of them are failures
%   that carry no trajectory at all - there is nothing to draw and nothing a map
%   could show that the status panel does not already say. A failed draw inside a
%   dispatch that then succeeds would also overwrite the route being driven with a
%   route that was rejected, which is the one outcome worse than showing nothing.
%
%   A DISPLAY TAP, NOT AN OUTPUT. Everything here is already on the BasePlan bus
%   and reaches the follower through it; that bus remains the contract and the only
%   thing the model acts on. Deleting this function would change no simulation
%   result. Nothing reads it except a user interface.
%
%   WHY THE GOAL IS PUBLISHED TOO, AND IT IS NOT DECORATION. A replan inside one
%   leg and the start of the next leg are indistinguishable from the plan alone -
%   both arrive as a fresh RequestId with a fresh tree. The GOAL is what tells them
%   apart: a replan drives at the same goal, a transfer's second leg at a different
%   one. The app clears the previous leg's trail when the goal moves and leaves it
%   alone when only the route changed, so a retry does not erase the history of the
%   drive it is retrying.
%
%   IT CANNOT FAIL THE SIMULATION. The whole body is guarded, for the same reason
%   hmmGroundingTap's is: this runs inside an extrinsic call from a MATLAB Function
%   block, so an error here would surface as a block fault and read like a defect
%   in the planner. A tap that cannot be shown is a missing line, not a failed run.
%
%   See also hmmPlanBasePath, hmmGroundingTap, HomeMobileManipulatorApp.

%   Copyright 2026 The MathWorks, Inc.

try
    tap = struct();
    tap.simTime   = double(simTime);
    tap.requestId = double(bplan.RequestId);
    tap.startPose = double(startPose(:))';
    tap.goalPose  = double(goalPose(:))';

    % TRIMMED TO WHAT WAS MEASURED, because the bus fields are FIXED WIDTH and the
    % unused tail is zeros - a 256-sample RefPose with 40 samples in it would draw
    % 216 points at the world origin, i.e. a line from the route to the corner of
    % the apartment and back.
    nS = double(bplan.NumSamples);
    nW = double(bplan.NumWaypoints);
    tap.refPose   = double(bplan.RefPose(:, 1:nS));
    tap.refTime   = double(bplan.RefTime(1:nS));
    tap.waypoints = double(bplan.Waypoints(:, 1:nW));

    tap.duration     = double(bplan.Duration);
    tap.pathLength   = double(bplan.PathLength);
    tap.minClearance = double(bplan.MinClearance);

    setappdata(0, 'hmmBasePlanTap', tap);
catch
    % Silent on purpose, and for the reason hmmGroundingTap gives: there is no
    % channel to report on from inside a solver step, and raising would abort a
    % run that is otherwise fine for the sake of a line on a picture.
end

end
