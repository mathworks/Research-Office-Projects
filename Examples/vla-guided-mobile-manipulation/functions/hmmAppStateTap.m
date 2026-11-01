function hmmAppStateTap(rs, tr, bs, as, gs, ss, otRefined, ot, pt)
%HMMAPPSTATETAP Publish what the operator console shows, so it need not read the SDI.
%
%   HMMAPPSTATETAP(rs, tr, bs, as, gs, ss, otRefined, ot, pt) is called by the root
%   AppStateTap block every 0.2 s of model time with the RobotState, TaskResult,
%   BaseStatus, ArmStatus, GripperStatus, SafetyStatus, ObjectTargetRefined,
%   ObjectTarget and PlaceTarget buses. It writes one struct to root appdata as
%   'hmmAppState' - the same fields HomeMobileManipulatorApp's readLiveState builds -
%   with the base trail accumulated here, sample by sample.
%
%   WHY. The console's 1 s poll used to read ~10-23 SDI channels, ~11 ms each however
%   little it fetched, on the MATLAB thread the simulation steps in: ~0.25 s of every
%   second the run stood still (profiled 2026-10-15). A getappdata costs microseconds.
%
%   NOTHING HAPPENS UNLESS THE CONSOLE ASKED: it sets appdata 'hmmAppStateWanted' at
%   Start, so a script or test run pays one getappdata per call and builds nothing.
%   A run restarting (model time going backwards) clears the trail.
%
%   See also hmmGroundingTap, hmmBasePlanTap, HomeMobileManipulatorApp.

%   ALSO, ONCE PER RUN, THE ROBOT'S SURFACE FINISH (hmmStyleRobot), whether or not the console
%   asked: this is the model's one display-only call that runs from t = 0, and the finish cannot
%   be set any earlier - hmmSpawnProps' init script runs before Simulation 3D Robot creates the
%   robot (measured 2026-10-16: three init phases, the robot in none of them), while a finish
%   set on a live actor does reach the renderer.

%   Copyright 2026 The MathWorks, Inc.

styleOncePerRun();
if ~isappdata(0, 'hmmAppStateWanted')
    return
end
try
    t = double(rs.Timestamp);
    prev = [];
    if isappdata(0, 'hmmAppState')
        prev = getappdata(0, 'hmmAppState');
    end
    pose = double(rs.BasePose(:));
    if isempty(prev) || t < prev.t
        trailX = pose(1); trailY = pose(2); trailT = t;
        seq = 1;
    else
        trailX = [prev.trailX; pose(1)];
        trailY = [prev.trailY; pose(2)];
        trailT = [prev.trailT; t];
        seq = prev.seq + 1;
    end

    s = struct('ok', true, 't', t, 'x', pose(1), 'y', pose(2), 'yaw', pose(3), ...
        'trailX', trailX, 'trailY', trailY, 'trailT', trailT, ...
        'plannerMode', double(tr.PlannerMode), 'activeVerb', double(tr.ActiveVerb), ...
        'outcome', double(tr.Outcome), 'baseState', double(bs.Mode), ...
        'armMode', double(as.Mode), 'gripState', double(gs.Mode), ...
        'gripFail', double(gs.FailureCode), 'gripSlipped', logical(gs.Slipped), ...
        'eStop', logical(ss.EStop), 'eStopCause', double(tr.EStopCause), ...
        'pickXY', [], 'placeXY', [], 'elapsed', double(tr.ElapsedTime), ...
        'distToGoal', double(bs.DistanceToGoal), 'taskFail', double(tr.FailureCode), ...
        'seq', seq);
    % The refined target first - it is what the arm goes to - and the coarse one only
    % as a fallback, exactly as readLiveState chooses.
    if otRefined.Valid
        s.pickXY = double(otRefined.Position(1:2))';
    elseif ot.Valid
        s.pickXY = double(ot.Position(1:2))';
    end
    if pt.Valid
        s.placeXY = double(pt.Point(1:2))';
    end
    setappdata(0, 'hmmAppState', s);
catch
    % A display channel: silent, like hmmGroundingTap, rather than aborting the run.
end
end

%% ------------------------------------------------------------------------
function styleOncePerRun()
%STYLEONCEPERRUN Set the robot's finish on the first call at which the robot exists in the live
%   world, and not again until a run brings a new world. The world is found by the name
%   hmmSpawnProps retained, as hmmSettleProp does; a handle kept from setup is invalid by now.
persistent styledWorld
try
    nm = getappdata(0, 'hmmPropWorldName');
    if isempty(nm)
        return
    end
    w = sim3d.World.getWorld(char(nm));
    if isempty(w) || ~isvalid(w)
        return
    end
    if ~isempty(styledWorld) && isvalid(styledWorld) && styledWorld == w
        return
    end
    if hmmStyleRobot(w) > 0
        styledWorld = w;
    end
catch
    % Display only: a finish that cannot be set leaves the robot as loaded.
end
end
