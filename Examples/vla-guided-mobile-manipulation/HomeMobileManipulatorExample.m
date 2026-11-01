%[text] # VLA-Guided Mobile Manipulation in a Home
%[text] A mobile manipulator is given one sentence of plain English - *"Pick up the red mug from the side table and set it down on the kitchen counter."* - and has to turn it into a drive across an apartment, a grasp and a place. This example builds the whole loop in Simulink: a vision-language-action provider (Gemini Robotics-ER) grounds the instruction against camera frames from a photorealistic Unreal scene, three planning services turn the grounded targets into base paths and arm trajectories, four Stateflow supervisors sequence the task, and a safety monitor watches nine independent limits with the authority to stop the robot.
%[text] The model is the product. Everything above - grounding, reconstruction, planning, tracking, supervision, the plant and the render - lives in `homeMobileManipulator.slx`. This script sets the parameters, runs it, and reads the result.
%[text] There are two ways to run it, chosen by the **Quick run** check box in the **Run** section:
%[text] - **Quick run** (the default) starts the model and hands the Command Window back to you. The only views are the Unreal window and one figure that shows each Gemini ER answer - the camera frames it was sent, with its boxes and points - as it arrives. Type further instructions while it runs, for example `hmmSubmitTask("go back to the home position")`.
%[text] - **Full run** runs a fixed 40 s of model time, then draws the analysis plots and the run report further down. \

%%
%[text] ## The task, and what makes it hard
%[text] Three things about this problem are not visible in the sentence itself.
%[text] - **The instruction names nothing the robot can act on.** "The red mug" is a phrase; a grasp needs a 6-DOF pose in metres, an aperture and an approach direction. Turning the first into the second is the perception service's entire job, and it can fail in ways a planner cannot recover from - an object named that is not in view, a reconstruction that lands between two registered actors, a support surface that cannot be fitted.
%[text] - **The base and the arm cannot both be right at once.** The mug is on a side table in another room, so the base has to place itself within a stand-off that leaves the target reachable *and* leaves room to approach it without the elbow entering the table. That coupling is why the planners are separate services and the supervisors are separate charts.
%[text] - **Nothing here is trustworthy enough to run unsupervised.** A network provider can time out, a reconstruction can be metrically wrong, an inverse-kinematics solution can be found for a pose the arm should not visit. So every service publishes a validity flag and a failure code, every plan is screened before it is tracked, and the safety monitor holds a latch that no planner can clear. \

%%
%[text] ## Architecture
%[text] The root of the model holds the functional groups and the plant.
%[text] - **`Perception`** - a triggered subsystem. `Grounding` sends the instruction and the camera frames to the VLA provider and validates the reply; `Reconstruction` turns the returned pixel points into metric 3-D, measures the object's extent and fits its support surface; `SceneMapping` passes the pre-measured occupancy map through under the default `cfg.map.mode = 'fixed'`.
%[text] - **`GraspRefinement`** - just before the descent commits, the wrist camera takes a close-up and Gemini ER is asked again where to grasp; the refined target replaces the coarse one only if it agrees with depth.
%[text] - **`TaskPlanner`** - the Stateflow chart that owns the action queue. It decides what to ask for next and publishes the outcome.
%[text] - **`MobileBase`** - `PathPlanning` plans a collision-free path over the occupancy grid; `PathFollowing` tracks it with PID controllers, with a Vector Field Histogram for local avoidance wired in behind a switch.
%[text] - **`Manipulator`** - `MotionPlanning` solves inverse kinematics, screens the result against the furniture and plans with `manipulatorRRT`; `ArmControl` tracks it.
%[text] - **`Gripper`** - converts a commanded aperture to the coupled finger joints of a 2F-85 gripper and detects closure.
%[text] - **`SafetyMonitor`** - nine checks feeding one latched e-stop that names its own first cause.
%[text] - **`Plant`** - mecanum base kinematics, a first-order arm joint servo, forward kinematics for the tool, a `Simulation 3D Robot` actor in an Unreal apartment, five cameras, and the grasp capture gate that is the only place a grasp is authorised. The cameras fire only when a grounding needs them. \

%%
%[text] ## Set up
%[text] `setupParams` builds the configuration, assembles the one authoritative 15-DOF `rigidBodyTree`, measures the gripper's aperture-to-joint map from that tree, defines the buses and loads the measured occupancy map the base plans over. The model cannot compile without it, which is why it is also wired to the model's `PostLoadFcn`.

clear; clc; close all;

modelName = 'homeMobileManipulator';

% A MODEL LEFT OPEN FROM AN EARLIER SESSION IS CLOSED FIRST. The masked Robotics System
% Toolbox blocks read mask-workspace variables, and mask initialization runs on load and on
% compile but NOT on the assignin that setupParams does, so a model still open against the
% workspace that `clear` just wiped keeps stale mask state. An UNSAVED model is left alone -
% someone is working on it - so in that case save or close it by hand and re-run this cell.
if bdIsLoaded(modelName) && strcmp(get_param(modelName, 'Dirty'), 'off') && ...
        strcmp(get_param(modelName, 'SimulationStatus'), 'stopped')
    close_system(modelName, 0);
end

params = setupParams;
cfg = params.cfg;
open_system(modelName)      % the architecture above, in the model itself

%%
%[text] ## Run
%[text] Choose the instruction and how to run it. **Ask Gemini ER live** sends the instruction to Gemini Robotics-ER and needs a key (the `GEMINI_API_KEY` environment variable, or a `gemini.key` file in the project); unticked, the answer comes from `perception/gemini/responses`, which holds a sample response in the Gemini Robotics-ER format (generated from the scene, not from Gemini) for each of three sentences - the transfer below, `"Drive to the open floor near the mug."` and `"go back to the home position"` - and refuses any other with `ReplayMiss`.

instruction = "Pick up the red mug from the side table and set it down on the kitchen counter."; %[control:editfield:7a1e]{"position":[15,96]}
askGeminiLive = false; %[control:checkbox:2b6c]{"position":[17,22]}
quickRun = true; %[control:checkbox:5d04]{"position":[12,16]}

%[text] The provider mode is read from `GEMINI_PROVIDER`, and the grounding service caches its configuration, so both are reset here before the run.
if askGeminiLive
    [~, route] = hmmProviderKey('live', cfg);
    if route == cfg.provider.ROUTE_NONE
        warning('No Gemini key found - using the sample responses instead.');
        askGeminiLive = false;
    end
end
if askGeminiLive
    setenv('GEMINI_PROVIDER', 'live');
else
    setenv('GEMINI_PROVIDER', 'replay');
end
clear hmmGroundingService hmmProviderRequest hmmOperatorTask
hmmSubmitTask('clear');
clear out

% One figure for the Gemini ER answers, in both modes. It is pushed to by the model as each
% answer arrives, so it costs the run nothing between answers.
hmmGroundingView('start');
hmmSubmitTask(instruction);

%[text] **Quick run.** The model starts in the background (it is saved in accelerator mode with an infinite stop time) and this script carries on to the end without waiting for it. While the apartment is open in the Unreal window you can type into the Command Window:
%[text] - `hmmSubmitTask("go back to the home position")` - another instruction, once the current one has finished;
%[text] - `hmmSubmitTask("cancel")` - abandon the task in flight;
%[text] - `set_param(modelName, "SimulationCommand", "stop")` - end the run. The logged signals then appear as `out`, and the sections below plot them when you run them again. \
%[text] **Full run.** `sim` runs 40 s of model time and returns when it is done - the transfer reports `DONE` well inside that, and the margin covers the randomised planners taking longer on an unlucky draw. Expect several minutes of wall-clock time: almost all of it is the Unreal scene, not the robot.

if quickRun
    set_param(modelName, 'SimulationCommand', 'start');
else
    in = Simulink.SimulationInput(modelName);
    in = in.setModelParameter('StopTime', '40');
    in = in.setModelParameter('Timeout', 2400);     % a guard against a hang, well clear of the slowest run
    out = sim(in);
end

%%
%[text] ## Results
%[text] Every figure below is drawn by `hmmPlotRun`, which is also what the companion app's Plots tab calls, and `cfg` is passed explicitly so that the reference lines describe the limits *this* run had. After a quick run they are skipped until the run is stopped.

haveRun = exist('out', 'var') == 1;
if ~haveRun
    disp('No finished run yet - stop the quick run, then run the sections below again.')
end

%%
%[text] ### Where the robot went
%[text] Two ground tracks in the apartment plane: the base origin and the tool centre point. The interesting part of a transfer is where they diverge - the base holds its stand-off while the arm reaches past it, so a tool track that never leaves the base track is an arm that never moved.

if haveRun
    hmmPlotRun(axes(figure), out, 'BasePath', cfg)
end

%%
%[text] ### How closely the base held its path
%[text] Distance to goal should fall and settle inside the goal tolerance; cross-track error is what the base trades away when it steers around an obstacle, and `crossTrackMax` is the value above which it reports itself off-path.

if haveRun
    hmmPlotRun(axes(figure), out, 'BaseTracking', cfg)
end

%%
%[text] ### What the arm was asked for, and what it did
%[text] Measured joint angles solid, the selected command dashed.

if haveRun
    hmmPlotRun(axes(figure), out, 'ArmJoints', cfg)
end

%%
%[text] The same story in the task space. When the tool error grows while the joint error does not, the servo is tracking its command faithfully and the *command* is the problem.

if haveRun
    hmmPlotRun(axes(figure), out, 'ArmTracking', cfg)
end

%%
%[text] ### The gripper
%[text] Commanded and measured aperture in millimetres, with the plant's own capture verdict scaled onto the same axes. `IsGrasped` comes from the capture gate, not from the controller: the gripper knowing it has closed and the plant agreeing that something is held are two different claims.

if haveRun
    hmmPlotRun(axes(figure), out, 'Gripper', cfg)
end

%%
%[text] ### Safety
%[text] Eleven checks as a raster, each row filled only where that check was asserted. An empty row is a check that never fired, which is the normal and expected picture.

if haveRun
    hmmPlotRun(axes(figure), out, 'SafetyFlags', cfg)
end

%%
%[text] ### The four supervisors against one another
%[text] The planner, base, arm and gripper charts on one time axis, each in its own numbering. Reading down a vertical line says what the whole robot was doing at that instant, which is how a stall gets attributed to the chart that is actually waiting.

if haveRun
    hmmPlotRun(axes(figure), out, 'Modes', cfg)
end

%%
%[text] ### The run in numbers
%[text] `hmmReportRun` prints the reduction and returns it. Read the last block first: `ReachedStopTime` says the solver was happy and says nothing about whether the instruction was grounded, whether the base drove anywhere or whether the gripper ever closed on anything. `hmmRunTimeline` then lists one row per instant at which any supervisor changed state.

if haveRun
    summary = hmmReportRun(out, cfg);
    timeline = hmmRunTimeline(out);
end

%%
%[text] ## The companion app
%[text] `HomeMobileManipulatorApp` is the interactive console for the same model: type an instruction, watch the base on the floor plan, the camera views with Gemini ER's boxes, the state of every supervisor and the plots growing during the run. *Record run* saves a run, camera images included, to `records/`, and *Load recorded run* replays one with a time slider. It draws through `hmmPlotRun` and `hmmShowGrounding`, so it cannot describe a run differently from this script.

%%
%[text] ## Known limitations
%[text] - **Local obstacle avoidance ships off, on purpose.** `cfg.track.useVFH` is `false`: every obstacle in this apartment is static and the base planner already sees all of them on the measured map, so reactive avoidance could only disagree with a path that was globally correct. The `controllerVFH` block stays wired and compiled behind the switch.
%[text] - **The scene map folds on the perception trigger.** In `SceneMapping`, under `cfg.map.mode = 'live'` the occupancy grid is updated once per grounding request rather than at `cfg.rates.perception`, so a live map stays nearly empty. The default `'fixed'` mode plans over the measured `data/hmmNavMap.mat` and is unaffected.
%[text] - **The planners are randomised.** `manipulatorRRT` and `plannerBiRRT` draw a different tree every run, so leg durations vary and an unlucky draw can need a retry; `BaseSearchFailed` is retried for that reason.
%[text] - **Sample responses exist only for the three example sentences.** Any other instruction needs **Ask Gemini ER live** and a key; the key is discovered at run time and never written into this project. \

%%
%[text] ## Clean up
%[text] Closes the model, unless a quick run is still going.

if strcmp(get_param(modelName, 'SimulationStatus'), 'stopped')
    hmmGroundingView('stop');
    bdclose(modelName)
end

%%
%[text] *Copyright 2026 The MathWorks, Inc.*

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
%[control:editfield:7a1e]
%   data: {"defaultValue":"\"Pick up the red mug from the side table and set it down on the kitchen counter.\"","label":"Instruction","run":"Nothing","valueType":"String"}
%---
%[control:checkbox:2b6c]
%   data: {"defaultValue":false,"label":"Ask Gemini ER live","run":"Nothing"}
%---
%[control:checkbox:5d04]
%   data: {"defaultValue":true,"label":"Quick run","run":"Nothing"}
%---
