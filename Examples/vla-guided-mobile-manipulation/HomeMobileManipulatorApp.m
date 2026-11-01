classdef HomeMobileManipulatorApp < matlab.apps.AppBase
%HOMEMOBILEMANIPULATORAPP Task the robot in natural language and watch it work.
%   
%   Start the model, watch the apartment come up in the Sim3D window with the robot at
%   its home pose, type an instruction - "move the mug to the kitchen" - and press Send.
%   Gemini ER is shown the four fixed camera frames, answers with a plan, and the app
%   draws the boxes and points it chose on the frames it chose them in. The left column
%   is the console: the sentence, the parameters, and a running account of what the
%   robot is doing. The right column is the evidence: the camera frames, the occupancy
%   map with the robot moving over it, and the plots of the finished run.
%   
%   THE THREE THINGS THAT CAN BE ASKED FOR, because they are not interchangeable:
%   
%     "move the mug to the kitchen"     transfer  - two points, two frames, a grasp
%     "move to the kitchen"             navigate  - one point, one frame, no grasp
%     "go back to the home position"    home      - no point and no frame at all
%   
%   Home is the odd one and the app says so where it matters. It is grounded from
%   cfg.task.homePose rather than from an image, which is the whole reason it exists as
%   its own verb: the robot starts on that spot already, but nose to a wall with 0.70 m
%   of clearance, and home means the same spot turned round so the BACK faces the wall.
%   A drive home reports the planner mode PL_NAV_PICK, which reads oddly on its own, so
%   this app names the phase from the VERB as well as the mode and says "driving home".
%   
%   THE MODEL RUNS ASYNCHRONOUSLY, AND EVERYTHING ELSE FOLLOWS FROM THAT. sim() blocks
%   the MATLAB thread for the whole run, which for this model is several minutes; while
%   it blocks, no callback can fire, so there is no instant at which anyone could type
%   anything. set_param(model, 'SimulationCommand', 'start') returns immediately and
%   leaves MATLAB free to run callbacks while the solver advances. That is the only way
%   an operator can be in the loop, and it is why this app polls on a timer instead of
%   waiting on a result.
%   
%   THE PRICE OF ASYNCHRONOUS IS THAT OVERRIDES GO THROUGH THE BASE WORKSPACE.
%   Simulink.SimulationInput and its setVariable only apply to a sim() call, so the
%   overridden cfg is assigned into the base workspace instead, which is where hmmSetup
%   publishes it anyway, and StopTime is set on the model rather than on an input
%   object. Setting a model parameter marks the model dirty; the app never saves it and
%   puts StopTime back as it found it when it closes, so a later save by a human does
%   not quietly persist a stop time somebody chose for one experiment.
%   
%   WHERE THE LIVE STATE COMES FROM, AND WHY IT IS NOT A MODEL EDIT. Two of the three
%   obvious channels are unavailable while a run is in flight: out.logsout does not
%   exist until the simulation stops, and RuntimeObject is not created at all because
%   the model is saved in accelerator mode. The third works. Logged signals STREAM to
%   the Simulation Data Inspector as the solver steps - measured, not assumed, by
%   watching RobotState.BasePose grow from 0 to 401 samples mid-run - so the pose, the
%   planner mode and the sub-states are read from there and no block was added to the
%   model to feed this panel.
%   
%   THE SDI FLATTENS A VECTOR INTO ONE CHANNEL PER ELEMENT, and that is a trap worth
%   naming. A [3 1] BasePose is not one channel carrying three numbers; it is
%   RobotState.BasePose(1,1), (2,1) and (3,1), and they are created LAZILY - element 1
%   exists several seconds before elements 2 and 3. So channels are looked up by their
%   exact name, never by position in the signal list, and a name that misses once
%   triggers one rebuild of the lookup rather than being treated as absent forever.
%   
%   HOW THE PLAN GETS BACK WHILE THE RUN IS STILL GOING. The frames sent to the
%   provider are on no port at all, so they could never be logged. hmmGroundingService
%   publishes each transaction to root appdata through hmmGroundingTap and this app
%   polls it. The model still acts only on PerceptionStatus and ActionQueue; the tap is
%   a display channel and nothing here feeds back into the simulation.
%   
%   THE BOXES ARE REAL AND THEY ARE DECORATION. Gemini Robotics-ER reports box_2d as
%   [ymin xmin ymax xmax] normalised 0-1000, which is exactly cfg.camera.pixelNormScale,
%   so nothing is rescaled to draw them. They travel to this app on the tap and on no
%   bus, because the model does not act on them: hmmValidateActions discards a malformed
%   box without failing the action that carried it, since refusing a valid grasp because
%   a drawing was badly formed would trade a working robot for a rectangle.
%   
%   THE MAP SHOWS BOTH PATHS AND THEY COME FROM DIFFERENT PLACES. The blue trail is the
%   one the robot DROVE, read from the pose history, so it cannot disagree with what
%   happened. The magenta dashed line is the one it PLANNED. This comment used to say the
%   planned route could not be drawn at all, because the RRT waypoints live on a BasePlan
%   bus that is not among the twelve logged ports, so drawing it would mean either a model
%   edit or re-planning here - and a re-plan is a different random tree from the one the
%   robot followed, which would put a confident wrong line on the map. Both objections
%   still stand and neither is what happens: hmmPlanBasePath publishes the plan it just
%   accepted to root appdata through hmmBasePlanTap, the same display-channel idiom
%   hmmGroundingTap uses, so the line is the actual route by construction, with no model
%   edit and no second search.
%   
%   AND ONLY THE CURRENT LEG IS SHOWN. A transfer drives twice - to the object, then to
%   the place surface - and two routes with two trails on one picture is four lines
%   through the same apartment. So the trail is drawn from the moment the base's GOAL last
%   moved, which is a leg boundary; a replan at the same goal is not one, and keeps the
%   history of the drive it is retrying. See updatePlan.
%   
%   NO SENTENCE, NO REQUEST, NO CREDIT. TaskRequestSource reads hmmOperatorTask, which
%   reports Valid = false until something is submitted, and hmmGroundingService returns
%   idle for an invalid instruction before it contacts the provider. A model sitting in
%   the apartment waiting to be spoken to costs nothing. The app clears the channel at
%   every start, because root appdata outlives a run and the previous sentence would
%   otherwise be re-grounded at t = 0.
%   
%   REPLAY CANNOT ANSWER AN ARBITRARY SENTENCE AND THE APP WILL NOT PRETEND IT CAN. In
%   replay mode hmmProviderRequest matches a recording against a hash of the
%   instruction. Only an exact match of the instruction and request kind can run;
%   a missing recording reports ReplayMiss rather than executing another task.
%   Typed instructions are accepted in replay mode under that same rule.
%   Live mode is a checkbox because a live request spends credit and that should be a
%   decision, not a default someone discovers afterwards.
%   
%   RUN IT DIRECTLY: type HomeMobileManipulatorApp, or press Run with this file open.
%   The app is a plain classdef that builds its own components, so there is no .mlapp
%   and nothing to open in App Designer; this file is the app.
%   
%   WHAT THE PARAMETER CONTROLS CAN AND CANNOT REACH. Overrides resolve for every BLOCK
%   DIALOG that names a cfg field. They do not reach the perception and planning
%   services, which are MATLAB Function blocks that call hmmConfig() in their own code
%   and several of which cache the result in a persistent variable. Only
%   dialog-reachable parameters are exposed here, and that was checked against every
%   cfg consumer rather than assumed.
%   
%   THE DERIVED FIELDS ARE RECOMPUTED, NOT LEFT STALE. cfg.arm.jointTimeConst is not a
%   leaf: hmmConfig derives cfg.track.armNaturalFreq = 1/tau from it and sizes
%   cfg.safety.jointAccelWindow in samples of it, and both of those are dialog-reachable
%   too. Changing tau alone would leave the arm servo's bandwidth and the acceleration
%   monitor's averaging window describing the old value, so the app repeats hmmConfig's
%   own derivation.
%   
%   See also hmmSubmitTask, hmmGroundingTap, hmmPlotRun, hmmReportRun, hmmSetup.

%   Copyright 2026 The MathWorks, Inc.

    properties (Access = public)
        UIFigure          matlab.ui.Figure
        MainLayout        matlab.ui.container.GridLayout

        TaskPanel         matlab.ui.container.Panel
        TaskLayout        matlab.ui.container.GridLayout
        PromptLabel       matlab.ui.control.Label
        PromptArea        matlab.ui.control.TextArea
        ExampleDropDown   matlab.ui.control.DropDown
        SendButton        matlab.ui.control.Button
        CancelButton      matlab.ui.control.Button
        LiveCheckBox      matlab.ui.control.CheckBox
        RouteLayout       matlab.ui.container.GridLayout
        RouteLamp         matlab.ui.control.Lamp
        RouteLabel        matlab.ui.control.Label

        ControlPanel      matlab.ui.container.Panel
        ControlLayout     matlab.ui.container.GridLayout
        StopTimeLabel     matlab.ui.control.Label
        StopTimeField     matlab.ui.control.NumericEditField
        LiveViewBox       matlab.ui.control.CheckBox
        ChaseViewBox      matlab.ui.control.CheckBox
        StartStopButton   matlab.ui.control.Button
        LoadButton        matlab.ui.control.Button
        StatusLayout      matlab.ui.container.GridLayout
        StatusLamp        matlab.ui.control.Lamp
        StatusLabel       matlab.ui.control.Label
        ScopeLabel        matlab.ui.control.Label

        StatePanel        matlab.ui.container.Panel
        StateLayout       matlab.ui.container.GridLayout
        PhaseLayout       matlab.ui.container.GridLayout
        PhaseLamp         matlab.ui.control.Lamp
        PhaseLabel        matlab.ui.control.Label
        PlannerNameLabel  matlab.ui.control.Label
        PlannerValueLabel matlab.ui.control.Label
        BaseNameLabel     matlab.ui.control.Label
        BaseValueLabel    matlab.ui.control.Label
        ArmNameLabel      matlab.ui.control.Label
        ArmValueLabel     matlab.ui.control.Label
        GripNameLabel     matlab.ui.control.Label
        GripValueLabel    matlab.ui.control.Label
        StateLog          matlab.ui.control.TextArea

        ResultPanel       matlab.ui.container.Panel
        ResultLayout      matlab.ui.container.GridLayout
        TabGroup          matlab.ui.container.TabGroup

        CameraTab         matlab.ui.container.Tab
        CameraLayout      matlab.ui.container.GridLayout
        CameraBlurb       matlab.ui.control.Label
        CameraGrid        matlab.ui.container.GridLayout
        CameraAxes1       matlab.ui.control.UIAxes
        CameraAxes2       matlab.ui.control.UIAxes
        CameraAxes3       matlab.ui.control.UIAxes
        CameraAxes4       matlab.ui.control.UIAxes
        PlanArea          matlab.ui.control.TextArea

        MapTab            matlab.ui.container.Tab
        MapLayout         matlab.ui.container.GridLayout
        MapBlurb          matlab.ui.control.Label
        MapAxes           matlab.ui.control.UIAxes
        MapCaption        matlab.ui.control.Label

        PlotTab           matlab.ui.container.Tab
        PlotLayout        matlab.ui.container.GridLayout
        ViewDropDown      matlab.ui.control.DropDown
        BlurbLabel        matlab.ui.control.Label
        PlotAxes          matlab.ui.control.UIAxes
        ReportArea        matlab.ui.control.TextArea
    end

    properties (Access = private)
        Model = 'homeMobileManipulator'   % the one model this app drives
        Root                              % project root, found from this app's own location
        Cfg                               % the derived configuration hmmSetup published
        RunCfg                            % the configuration the displayed run was made with
        Out                               % the SimulationOutput currently on display
        Source = ''                       % how Out was obtained, for the status line
        Views                             % the hmmPlotRun catalogue
        Poll                              % timer watching the run, the tap and the SDI
        WasRunning = false                % to catch the running-to-stopped edge once
        LastTap = -1                      % request id of the tap already drawn
        SdiSampleCache                    % incremental scalar-channel exports
        StopTimeWas = ''                  % the model's own StopTime, restored on close
        CarryTransWas = ''                 % RobotTree's own eeTargetTrans expression, ditto
        CarryNameApplied = ''              % which prop's pivot offset is currently on the block

        NavMap                            % the shipped occupancy grid, in world coordinates
        MapH                              % the map's graphics handles, updated not redrawn
        MapReady = false                  % whether those handles exist yet
        CamAxes                           % the four camera axes as a cell, for looping
        WristPane = 0                     % camera pane currently showing the wrist frame, 0 = none
        LastLiveSeq = 0                   % hmmLiveViewTap capture already drawn
        LastPollError = ''                % last onPoll error printed, so each is shown once
        PlotScratchFig = []               % hidden figure the live plot is computed in
        PlotScratchAx  = []               % its axes; data is copied to PlotAxes from here
        PlotRefreshTic = []               % tic of the last live plot refresh
        PlotRefreshEvery = 5.0            % s of wall clock between live plot refreshes; each costs ~0.3 s
        RecordBox                         % "Record run" checkbox
        RecordHint                        % the two lines under it
        RecDir = ''                       % folder the run in progress is recorded to, '' if not
        RecFrames = struct('t', {}, 'files', {})   % live camera captures written so far
        RecPlans = {}                     % hmmBasePlanTap records, in order
        RecTaps = {}                      % hmmGroundingTap records, images on disk
        RecWrist = struct('t', {}, 'file', {}, 'pane', {})  % wrist frames sent to Gemini
        RecLastPlanT = -Inf               % simTime of the last plan recorded
        ReplayBar                         % the grid under the tabs, shown with a recording
        ReplayPlay                        % Play / Pause
        ReplaySlider                      % model time of the recording on display
        ReplayTime                        % "t = 12.3 / 26.4 s"
        ReplaySpeed                       % 1x / 2x / 4x
        Replay = []                       % the loaded recording, or []
        ReplayTimer = []                  % drives Play

        % THE LEG, WHICH IS NOT THE PLAN. A transfer plans at least twice - once to reach
        % the object and once to reach the place surface - and may replan inside either
        % leg. PlanGoal is what tells those apart: a replan drives at the same goal, the
        % next leg at a different one. LegStartTime is the model time the current goal
        % first appeared, and the trail is drawn from there, so a transfer's second leg
        % starts on a clean map while a retry keeps the history of the drive it retries.
        PlanGoal = []                     % goal pose of the plan currently drawn
        LegStartTime = -Inf               % model time that goal was first seen

        SdiRunId = -1                     % the SDI run the lookup table belongs to
        SdiRunBefore = -1                 % the run that existed BEFORE this start
        SdiMap                            % channel name -> signal id, for this run
        SdiRebuilt = false                % one rebuild per poll, no more
        SdiRebuildTic                     % when the table was last rebuilt on a miss
        PollCount = 0                     % polls read, for the slow tier of readLiveState
        SlowState = []                    % that tier's fields as last read

        LastPhase = ''                    % so the log records changes, not every poll
        LogLines = {}                     % the running account, oldest first
        LogStamps = []                    % model time of each line, NaN for the anchors
    end

    methods (Access = private)

        function createComponents(app)
            %CREATECOMPONENTS Build the interface.
            %   Every child names its own Layout.Row and Layout.Column. Relying on a grid's
            %   automatic placement instead puts each component in the next free cell in
            %   row-major order and then setting Layout.Column afterwards changes only the
            %   column, leaving a full-width button sharing a row with whatever had already
            %   been placed there.
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [40 20 1520 980];
            app.UIFigure.Name = 'Home Mobile Manipulator - VLA-guided mobile manipulation';

            % THE LEFT COLUMN IS THE CONSOLE, THE RIGHT COLUMN IS THE EVIDENCE. Three rows
            % on the left in the order an operator uses them - say what you want, set how
            % it should be done, then watch what happens - and one tall cell on the right
            % holding the tabs. The left column is a fixed 380 px because every control in
            % it is a label and a field; stretching those buys nothing, while the frames
            % and the map on the right use every pixel they are given.
            app.MainLayout = uigridlayout(app.UIFigure, [3 2]);
            app.MainLayout.ColumnWidth = {380, '1x'};
            app.MainLayout.RowHeight = {308, 368, '1x'};

            buildTaskPanel(app);
            buildControlPanel(app);
            buildStatePanel(app);
            buildResultPanel(app);

            app.UIFigure.Visible = 'on';
        end

        function buildTaskPanel(app)
            %BUILDTASKPANEL The prompt, the Send button and the provider's route.
            app.TaskPanel = uipanel(app.MainLayout);
            app.TaskPanel.Title = 'Task - what should the robot do?';
            app.TaskPanel.Layout.Row = 1;
            app.TaskPanel.Layout.Column = 1;

            app.TaskLayout = uigridlayout(app.TaskPanel, [6 2]);
            app.TaskLayout.ColumnWidth = {'1x', '1x'};
            app.TaskLayout.RowHeight = {20, '1x', 26, 30, 22, 40};
            app.TaskLayout.RowSpacing = 6;

            app.PromptLabel = uilabel(app.TaskLayout);
            app.PromptLabel.Text = 'Instruction, in ordinary language';
            app.PromptLabel.Layout.Row = 1;
            app.PromptLabel.Layout.Column = [1 2];

            app.PromptArea = uitextarea(app.TaskLayout);
            app.PromptArea.Layout.Row = 2;
            app.PromptArea.Layout.Column = [1 2];
            app.PromptArea.Value = {['Pick up the red mug from the side table and set it ' ...
                'down on the kitchen counter.']};
            % THE DESIGN'S RULE, RESTATED WHERE IT IS ACTED ON: the sentence is the only
            % place a scene object may be named. Which actor, which pixel, which surface
            % and which grasp are all grounded from the cameras, so an operator who types
            % coordinates here is defeating the thing the example exists to show.
            app.PromptArea.Tooltip = ['Name the object and the destination in words. Do not ' ...
                'give coordinates: the pick and place pixels are grounded from the camera ' ...
                'frames, which is the point of the example.'];

            % THE THREE VERBS, MADE DISCOVERABLE. Nothing in a free-text box tells an
            % operator that "go back to the home position" is a different kind of command
            % from "move to the kitchen", and the difference is the interesting part of the
            % model: one is grounded from an image and the other from configuration. So one
            % example of each is offered rather than documented.
            app.ExampleDropDown = uidropdown(app.TaskLayout);
            % THE TRANSFER EXAMPLE NAMES THE COLOUR, AND IT IS NOW ONLY DISAMBIGUATION.
            % There are two mugs in the manifest, Mug and Mug_Red; they are separate mesh
            % files with different pivot offsets, and "the mug" grounds whichever one the
            % camera happens to find. Naming the colour says which one is meant.
            %
            % IT USED TO BE LOAD-BEARING FOR A SECOND REASON THAT NO LONGER HOLDS, and the
            % retraction belongs here because this comment is where the workaround was
            % recorded. The claim was that the Sim3D Robot block's end-effector-to-target
            % translation is a mask parameter read at compile time with no runtime route, so
            % the carried object had to be fixed from cfg.task.instruction before the first
            % frame; name a different mug than the config names and the prop was attached
            % with the other mesh's pivot and jumped as the jaws closed. That was measured on
            % the transfer run of 2026-09-30, and the diagnosis was right while the premise
            % was wrong - the parameter is tunable, and onSend now retunes it to whatever the
            % typed sentence names. Any object may be named in any order.
            app.ExampleDropDown.Items = { ...
                'one of each kind of command...', ...
                'Pick up the red mug from the side table and set it down on the kitchen counter.', ...
                'Drive to the open floor near the mug.', ...
                'go back to the home position'};
            app.ExampleDropDown.Layout.Row = 3;
            app.ExampleDropDown.Layout.Column = [1 2];
            app.ExampleDropDown.Tooltip = ['transfer, navigate and home. The first needs two ' ...
                'points in two frames, the second one point in one frame, and the third ' ...
                'needs no image at all - it drives to cfg.task.homePose.'];
            app.ExampleDropDown.ValueChangedFcn = createCallbackFcn(app, @onExample, true);

            app.SendButton = uibutton(app.TaskLayout, 'push');
            app.SendButton.Text = 'Send to Gemini ER';
            app.SendButton.Layout.Row = 4;
            app.SendButton.Layout.Column = 1;
            app.SendButton.ButtonPushedFcn = createCallbackFcn(app, @onSend, true);

            app.CancelButton = uibutton(app.TaskLayout, 'push');
            app.CancelButton.Text = 'Cancel task';
            app.CancelButton.Layout.Row = 4;
            app.CancelButton.Layout.Column = 2;
            app.CancelButton.Tooltip = ['Abandons the task in flight. The supervisor takes ' ...
                'its Operating to Idle transition and reports ABORTED, and another ' ...
                'instruction can then be sent.'];
            app.CancelButton.ButtonPushedFcn = createCallbackFcn(app, @onCancel, true);

            app.LiveCheckBox = uicheckbox(app.TaskLayout);
            app.LiveCheckBox.Text = 'Ask Gemini ER live (spends credit)';
            app.LiveCheckBox.Value = true;
            app.LiveCheckBox.Layout.Row = 5;
            app.LiveCheckBox.Layout.Column = [1 2];
            app.LiveCheckBox.Tooltip = ['On, the instruction goes to Gemini ER and the plan is ' ...
                'its answer. Off, the plan comes from a sample response in the Gemini ER format, ' ...
                'chosen by the exact sentence - the three examples above have one. Any other ' ...
                'sentence has no sample response (ReplayMiss) and nothing runs.'];
            app.LiveCheckBox.ValueChangedFcn = createCallbackFcn(app, @onLiveChanged, true);

            % Same nested-grid trick as the status lamp below, for the same reason.
            app.RouteLayout = uigridlayout(app.TaskLayout, [1 2]);
            app.RouteLayout.ColumnWidth = {22, '1x'};
            app.RouteLayout.RowHeight = {22, '1x'};
            app.RouteLayout.Padding = [0 0 0 0];
            app.RouteLayout.ColumnSpacing = 6;
            app.RouteLayout.Layout.Row = 6;
            app.RouteLayout.Layout.Column = [1 2];

            app.RouteLamp = uilamp(app.RouteLayout);
            app.RouteLamp.Color = [0.80 0.80 0.80];
            app.RouteLamp.Layout.Row = 1;
            app.RouteLamp.Layout.Column = 1;

            app.RouteLabel = uilabel(app.RouteLayout);
            app.RouteLabel.Text = 'nothing submitted';
            app.RouteLabel.WordWrap = 'on';
            app.RouteLabel.VerticalAlignment = 'top';
            app.RouteLabel.Layout.Row = [1 2];
            app.RouteLabel.Layout.Column = 2;
        end

        function buildControlPanel(app)
            %BUILDCONTROLPANEL The dialog-reachable parameters and the run controls.
            app.ControlPanel = uipanel(app.MainLayout);
            app.ControlPanel.Title = 'Run';
            app.ControlPanel.Layout.Row = 2;
            app.ControlPanel.Layout.Column = 1;

            % NO TUNING FIELDS (2026-10-14). Base speed, the two follower gains and the arm servo
            % time constant used to be editable here; each one is tuned against a measured safety
            % margin (cfg.base.planMargin, cfg.arm.planMargin) or is a property of the robot model,
            % so a shipped operator console runs hmmConfig as shipped.
            app.ControlLayout = uigridlayout(app.ControlPanel, [9 2]);
            app.ControlLayout.ColumnWidth = {'1.5x', '1x'};
            % Row 8 is 46 and not 34 because the status line is a SENTENCE. Two of the
            % messages it carries wrap to two lines at this column width, and a 34 px row
            % clipped the second one half way through - which is worse than a short label,
            % because the message that gets cut is the one explaining what to do next.
            app.ControlLayout.RowHeight = {24, 24, 24, 34, 6, 30, 30, 46, '1x'};
            app.ControlLayout.RowSpacing = 6;

            [app.StopTimeLabel, app.StopTimeField] = numericRow(app, 1, ...
                'Stop time (s)', [2 Inf], '%.0f', ...
                ['Simulation stop time, set on the model rather than through cfg. Inf (the ' ...
                 'default) runs until you press Stop simulation, so a session can take as ' ...
                 'many instructions as you like. Type a number for a fixed budget: this ' ...
                 'model runs about 25 times slower than real time, so 40 s of model time ' ...
                 'is roughly 8 minutes of wall clock.']);

            % LIVE CAMERA VIEWS COST RENDER TIME, so they are opt-in and apply at the next start
            % like every other control here: the rig is opened for one camera sample every
            % cfg.camera.livePeriod and the four panes are refreshed, keeping the pick and
            % place marks (the cameras are fixed, so their pixels stay valid).
            app.LiveViewBox = uicheckbox(app.ControlLayout);
            app.LiveViewBox.Text = 'Live camera views';
            app.LiveViewBox.Layout.Row = 2;
            app.LiveViewBox.Layout.Column = 1;
            app.LiveViewBox.Tooltip = ['Refresh the four camera panes during the run, about ' ...
                'once per second of model time, so the robot can be watched moving. Costs some ' ...
                'simulation speed. Applies at the next start.'];

            % THE CHASE VIEW IS OPT-IN FOR THE SAME REASON: it renders a second view every scene
            % step (cfg.view.chase), so it costs about 15-20 % more wall clock while it is on.
            app.ChaseViewBox = uicheckbox(app.ControlLayout);
            app.ChaseViewBox.Text = 'Chase camera';
            app.ChaseViewBox.Layout.Row = 2;
            app.ChaseViewBox.Layout.Column = 2;
            app.ChaseViewBox.Tooltip = ['Open a second window that follows the robot from ' ...
                'behind, beside the full view. Costs about 15-20 % simulation speed while it ' ...
                'is on. Takes effect at once, also during a run.'];
            app.ChaseViewBox.ValueChangedFcn = createCallbackFcn(app, @onChaseChanged, true);

            app.RecordBox = uicheckbox(app.ControlLayout);
            app.RecordBox.Text = 'Record run';
            app.RecordBox.Layout.Row = 3;
            app.RecordBox.Layout.Column = [1 2];
            app.RecordBox.Tooltip = ['Save this run to the records folder when it ends, so it ' ...
                'can be loaded and replayed later. Applies at the next start.'];

            app.RecordHint = uilabel(app.ControlLayout);
            app.RecordHint.Text = ['Saves the run to records/ when it ends, for Load recorded ' ...
                'run and the replay bar. Tick Live camera views too to keep the camera images.'];
            app.RecordHint.Layout.Row = 4;
            app.RecordHint.Layout.Column = [1 2];
            app.RecordHint.WordWrap = 'on';
            app.RecordHint.FontSize = 10;
            app.RecordHint.FontAngle = 'italic';
            app.RecordHint.FontColor = [0.35 0.35 0.35];
            app.RecordHint.VerticalAlignment = 'top';

            app.StartStopButton = uibutton(app.ControlLayout, 'push');
            app.StartStopButton.Text = 'Start simulation';
            app.StartStopButton.Layout.Row = 6;
            app.StartStopButton.Layout.Column = [1 2];
            app.StartStopButton.ButtonPushedFcn = createCallbackFcn(app, @onStartStop, true);

            app.LoadButton = uibutton(app.ControlLayout, 'push');
            app.LoadButton.Text = 'Load recorded run';
            app.LoadButton.Layout.Row = 7;
            app.LoadButton.Layout.Column = [1 2];
            app.LoadButton.ButtonPushedFcn = createCallbackFcn(app, @onLoad, true);

            % The lamp gets its own nested grid rather than a cell of the outer one. A grid
            % stretches every child to fill its cell, and the outer cell here is about 190 px
            % wide by 34 tall, which draws the lamp as a wide grey oval instead of a light. A
            % fixed 22 px column is the only way to say "this one keeps its size", because
            % Position is ignored for anything inside a grid.
            app.StatusLayout = uigridlayout(app.ControlLayout, [1 2]);
            % TWO ROWS SO THAT ONLY THE LABEL GROWS. A grid stretches every child to fill its
            % cell, so a single 46 px row would draw the lamp as a tall grey oval. The lamp
            % keeps a 22 px row of its own and the label spans both.
            app.StatusLayout.ColumnWidth = {22, '1x'};
            app.StatusLayout.RowHeight = {22, '1x'};
            app.StatusLayout.Padding = [0 0 0 0];
            app.StatusLayout.ColumnSpacing = 6;
            app.StatusLayout.Layout.Row = 8;
            app.StatusLayout.Layout.Column = [1 2];

            app.StatusLamp = uilamp(app.StatusLayout);
            app.StatusLamp.Color = [0.80 0.80 0.80];
            app.StatusLamp.Layout.Row = 1;
            app.StatusLamp.Layout.Column = 1;

            app.StatusLabel = uilabel(app.StatusLayout);
            app.StatusLabel.Text = 'starting up';
            app.StatusLabel.WordWrap = 'on';
            app.StatusLabel.VerticalAlignment = 'top';
            app.StatusLabel.Layout.Row = [1 2];
            app.StatusLabel.Layout.Column = 2;

            app.ScopeLabel = uilabel(app.ControlLayout);
            app.ScopeLabel.Layout.Row = 9;
            app.ScopeLabel.Layout.Column = [1 2];
            app.ScopeLabel.WordWrap = 'on';
            app.ScopeLabel.FontAngle = 'italic';
            app.ScopeLabel.FontColor = [0.35 0.35 0.35];
            app.ScopeLabel.VerticalAlignment = 'top';
            app.ScopeLabel.Text = ['The run uses configuration/hmmConfig.m as shipped; these ' ...
                'two controls apply at the next start. The apartment is a separate Sim3D ' ...
                'window; this one is the console.'];
        end

        function buildStatePanel(app)
            %BUILDSTATEPANEL What the robot is doing, in words, as it does it.
            %   THE HEADLINE IS PROSE AND THE FOUR ROWS ARE THE MACHINE'S OWN WORDS. An
            %   operator wants to read "driving home - planning a path"; someone debugging
            %   wants to know that the planner is in NAV_TO_PICK while the base is in
            %   AWAIT_PLAN. Those are different questions, so both are answered rather than
            %   one being made to serve for the other.
            app.StatePanel = uipanel(app.MainLayout);
            app.StatePanel.Title = 'Robot state';
            app.StatePanel.Layout.Row = 3;
            app.StatePanel.Layout.Column = 1;

            app.StateLayout = uigridlayout(app.StatePanel, [6 2]);
            app.StateLayout.ColumnWidth = {88, '1x'};
            app.StateLayout.RowHeight = {28, 19, 19, 19, 19, '1x'};
            app.StateLayout.RowSpacing = 4;

            app.PhaseLayout = uigridlayout(app.StateLayout, [1 2]);
            app.PhaseLayout.ColumnWidth = {22, '1x'};
            app.PhaseLayout.RowHeight = {24};
            app.PhaseLayout.Padding = [0 0 0 0];
            app.PhaseLayout.ColumnSpacing = 6;
            app.PhaseLayout.Layout.Row = 1;
            app.PhaseLayout.Layout.Column = [1 2];

            app.PhaseLamp = uilamp(app.PhaseLayout);
            app.PhaseLamp.Color = [0.80 0.80 0.80];
            app.PhaseLamp.Layout.Row = 1;
            app.PhaseLamp.Layout.Column = 1;

            app.PhaseLabel = uilabel(app.PhaseLayout);
            app.PhaseLabel.Text = 'not running';
            app.PhaseLabel.FontSize = 15;
            app.PhaseLabel.FontWeight = 'bold';
            app.PhaseLabel.Layout.Row = 1;
            app.PhaseLabel.Layout.Column = 2;

            [app.PlannerNameLabel, app.PlannerValueLabel] = stateRow(app, 2, 'planner', ...
                'TaskResult.PlannerMode, named from cfg.sup.pl.');
            [app.BaseNameLabel, app.BaseValueLabel] = stateRow(app, 3, 'base', ...
                'BaseStatus.State, named from cfg.sup.base.');
            [app.ArmNameLabel, app.ArmValueLabel] = stateRow(app, 4, 'arm', ...
                'ArmStatus.Mode, named from cfg.sup.arm.');
            [app.GripNameLabel, app.GripValueLabel] = stateRow(app, 5, 'gripper', ...
                'GripperStatus.State, named from cfg.sup.grip.');

            app.StateLog = uitextarea(app.StateLayout);
            app.StateLog.Layout.Row = 6;
            app.StateLog.Layout.Column = [1 2];
            app.StateLog.Editable = 'off';
            app.StateLog.FontName = 'Consolas';
            app.StateLog.FontSize = 11;
            app.StateLog.Value = {'nothing has happened yet.'};
            app.StateLog.Tooltip = ['One line per CHANGE of state, stamped with the model ' ...
                'time it happened at. A line per poll would bury the changes in repetition.'];
        end

        function [name, value] = stateRow(app, row, text, tip)
            %STATEROW One name-and-value line of the state breakdown.
            name = uilabel(app.StateLayout);
            name.Text = text;
            name.FontColor = [0.40 0.40 0.40];
            name.Layout.Row = row;
            name.Layout.Column = 1;

            value = uilabel(app.StateLayout);
            value.Text = '-';
            value.FontName = 'Consolas';
            value.Tooltip = tip;
            value.Layout.Row = row;
            value.Layout.Column = 2;
        end

        function buildResultPanel(app)
            %BUILDRESULTPANEL Three tabs, because they answer three different questions.
            %   Cameras: what did Gemini ER look at and where did it point?
            %   Map:     where is the robot, and where has it been?
            %   Plots:   how did the finished run behave?
            %
            %   TABS RATHER THAN A STACK OF SMALL AXES. Four 1280x720 frames and a 240x240
            %   occupancy map all want the whole pane; sharing it between them would make
            %   each one too small to read the thing it exists to show. Only one question is
            %   being asked at a time, so only one tab needs to be visible at a time.
            app.ResultPanel = uipanel(app.MainLayout);
            app.ResultPanel.Title = 'Results';
            app.ResultPanel.Layout.Row = [1 3];
            app.ResultPanel.Layout.Column = 2;

            % Row 2 is the replay bar, height 0 until a recording is loaded.
            app.ResultLayout = uigridlayout(app.ResultPanel, [2 1]);
            app.ResultLayout.RowHeight = {'1x', 0};
            app.ResultLayout.ColumnWidth = {'1x'};
            app.ResultLayout.Padding = [4 4 4 4];

            app.TabGroup = uitabgroup(app.ResultLayout);
            app.TabGroup.Layout.Row = 1;
            app.TabGroup.Layout.Column = 1;
            app.TabGroup.SelectionChangedFcn = createCallbackFcn(app, @onTabChanged, true);

            buildCameraTab(app);
            buildMapTab(app);
            buildPlotTab(app);

            app.ReplayBar = uigridlayout(app.ResultLayout, [1 4]);
            app.ReplayBar.Layout.Row = 2;
            app.ReplayBar.Layout.Column = 1;
            app.ReplayBar.ColumnWidth = {70, '1x', 120, 60};
            app.ReplayBar.Padding = [2 2 2 2];
            app.ReplayPlay = uibutton(app.ReplayBar, 'push');
            app.ReplayPlay.Text = 'Play';
            app.ReplayPlay.ButtonPushedFcn = createCallbackFcn(app, @onReplayPlay, true);
            app.ReplaySlider = uislider(app.ReplayBar);
            app.ReplaySlider.MajorTicks = [];
            app.ReplaySlider.MinorTicks = [];
            app.ReplaySlider.ValueChangingFcn = createCallbackFcn(app, @onReplayScrub, true);
            app.ReplaySlider.ValueChangedFcn  = createCallbackFcn(app, @onReplayScrub, true);
            app.ReplayTime = uilabel(app.ReplayBar);
            app.ReplayTime.Text = '';
            app.ReplaySpeed = uidropdown(app.ReplayBar);
            app.ReplaySpeed.Items = {'1x', '2x', '4x', '8x'};
            app.ReplaySpeed.Value = '2x';
            app.ReplaySpeed.Tooltip = 'Replay speed, in model time';
        end

        function buildCameraTab(app)
            %BUILDCAMERATAB All four fixed frames at once, with the plan underneath.
            %   ALL FOUR, NOT THE ONE THE FIRST ACTION USED. A pick pixel and a place pixel
            %   routinely come from different cameras across the room from each other, and
            %   showing one frame at a time hides that - it also makes an operator wonder
            %   where the other marker went. Four panes show every marker on the frame it
            %   was actually measured in, and a camera that was not sent says so instead of
            %   borrowing somebody else's picture.
            app.CameraTab = uitab(app.TabGroup);
            app.CameraTab.Title = 'Cameras';

            app.CameraLayout = uigridlayout(app.CameraTab, [3 1]);
            app.CameraLayout.RowHeight = {30, '1x', 150};
            app.CameraLayout.ColumnWidth = {'1x'};

            app.CameraBlurb = uilabel(app.CameraLayout);
            app.CameraBlurb.Layout.Row = 1;
            app.CameraBlurb.Layout.Column = 1;
            app.CameraBlurb.WordWrap = 'on';
            app.CameraBlurb.FontAngle = 'italic';
            app.CameraBlurb.FontColor = [0.35 0.35 0.35];
            app.CameraBlurb.Text = ['The four fixed cameras are what the provider sees. ' ...
                'The wrist camera is never sent.'];

            app.CameraGrid = uigridlayout(app.CameraLayout, [2 2]);
            app.CameraGrid.Layout.Row = 2;
            app.CameraGrid.Layout.Column = 1;
            app.CameraGrid.Padding = [0 0 0 0];
            app.CameraGrid.RowSpacing = 4;
            app.CameraGrid.ColumnSpacing = 4;

            app.CameraAxes1 = camAxes(app, 1, 1);
            app.CameraAxes2 = camAxes(app, 1, 2);
            app.CameraAxes3 = camAxes(app, 2, 1);
            app.CameraAxes4 = camAxes(app, 2, 2);
            % Held as a cell as well, so the four are drawn by a loop rather than by four
            % near-identical blocks that could drift apart.
            app.CamAxes = {app.CameraAxes1, app.CameraAxes2, ...
                           app.CameraAxes3, app.CameraAxes4};

            app.PlanArea = uitextarea(app.CameraLayout);
            app.PlanArea.Layout.Row = 3;
            app.PlanArea.Layout.Column = 1;
            app.PlanArea.Editable = 'off';
            app.PlanArea.FontName = 'Consolas';
            app.PlanArea.Value = {'Nothing grounded yet.'};
        end

        function ax = camAxes(app, row, col)
            %CAMAXES One camera pane, with the ticks off before anything is drawn in it.
            ax = uiaxes(app.CameraGrid);
            ax.Layout.Row = row;
            ax.Layout.Column = col;
            ax.XTick = [];
            ax.YTick = [];
            ax.Box = 'on';
        end

        function buildMapTab(app)
            %BUILDMAPTAB The occupancy grid, the planned and travelled paths, and the robot.
            app.MapTab = uitab(app.TabGroup);
            app.MapTab.Title = 'Map';

            app.MapLayout = uigridlayout(app.MapTab, [3 1]);
            app.MapLayout.RowHeight = {30, '1x', 46};
            app.MapLayout.ColumnWidth = {'1x'};

            app.MapBlurb = uilabel(app.MapLayout);
            app.MapBlurb.Layout.Row = 1;
            app.MapBlurb.Layout.Column = 1;
            app.MapBlurb.WordWrap = 'on';
            app.MapBlurb.FontAngle = 'italic';
            app.MapBlurb.FontColor = [0.35 0.35 0.35];
            app.MapBlurb.Text = ['The shipped navigation map, in metres. The robot is drawn ' ...
                'to its real chassis footprint.'];

            app.MapAxes = uiaxes(app.MapLayout);
            app.MapAxes.Layout.Row = 2;
            app.MapAxes.Layout.Column = 1;

            app.MapCaption = uilabel(app.MapLayout);
            app.MapCaption.Layout.Row = 3;
            app.MapCaption.Layout.Column = 1;
            app.MapCaption.WordWrap = 'on';
            app.MapCaption.FontSize = 11;
            app.MapCaption.FontColor = [0.35 0.35 0.35];
            app.MapCaption.VerticalAlignment = 'top';
            % THE LEGEND IS WRITTEN OUT BECAUSE THERE ARE NOW TWO PATHS AND A READER WILL
            % NOT GUESS WHICH IS WHICH. One is a measurement and the other a reference, and
            % the whole value of drawing both is seeing where they part company.
            app.MapCaption.Text = ['Orange box: the chassis, with a spur showing which way ' ...
                'it faces. Blue line: the path the robot HAS DRIVEN, read back from the ' ...
                'logged pose. Magenta dashes and dots: the route it PLANNED for the leg ' ...
                'it is on - the smoothed reference and the search waypoints behind it. ' ...
                'Both are cleared when the base is given a new goal, so a transfer shows ' ...
                'the leg in progress rather than all of them at once. Green ' ...
                'star and arrow: the home pose and its heading. Grey squares: the four ' ...
                'fixed cameras. + and x: the pick and place targets, once grounded.'];
        end

        function buildPlotTab(app)
            %BUILDPLOTTAB The hmmPlotRun catalogue and the hmmReportRun text, unchanged.
            app.PlotTab = uitab(app.TabGroup);
            app.PlotTab.Title = 'Plots';

            app.PlotLayout = uigridlayout(app.PlotTab, [4 1]);
            app.PlotLayout.RowHeight = {26, 46, '2.2x', '1x'};     % three wrapped lines of blurb
            app.PlotLayout.ColumnWidth = {'1x'};

            app.ViewDropDown = uidropdown(app.PlotLayout);
            app.ViewDropDown.Layout.Row = 1;
            app.ViewDropDown.Layout.Column = 1;
            app.ViewDropDown.ValueChangedFcn = createCallbackFcn(app, @onView, true);

            app.BlurbLabel = uilabel(app.PlotLayout);
            app.BlurbLabel.Layout.Row = 2;
            app.BlurbLabel.Layout.Column = 1;
            app.BlurbLabel.WordWrap = 'on';
            app.BlurbLabel.FontAngle = 'italic';
            app.BlurbLabel.FontColor = [0.35 0.35 0.35];
            app.BlurbLabel.VerticalAlignment = 'top';

            app.PlotAxes = uiaxes(app.PlotLayout);
            app.PlotAxes.Layout.Row = 3;
            app.PlotAxes.Layout.Column = 1;

            app.ReportArea = uitextarea(app.PlotLayout);
            app.ReportArea.Layout.Row = 4;
            app.ReportArea.Layout.Column = 1;
            app.ReportArea.Editable = 'off';
            app.ReportArea.FontName = 'Consolas';
            app.ReportArea.Value = {''};
        end

        function [lbl, fld] = numericRow(app, row, text, limits, format, tip)
            %NUMERICROW One labelled numeric control, so five of them are five lines.
            lbl = uilabel(app.ControlLayout);
            lbl.Text = text;
            lbl.Layout.Row = row;
            lbl.Layout.Column = 1;

            fld = uieditfield(app.ControlLayout, 'numeric');
            fld.Limits = limits;
            fld.ValueDisplayFormat = format;
            fld.Tooltip = tip;
            fld.Layout.Row = row;
            fld.Layout.Column = 2;
        end

        %% --------------------------------------------------------------- startup

        function startupFcn(app)
            %STARTUPFCN Find the project, publish its configuration, open on the cameras.
            %   The app locates the project from its OWN file rather than from the current
            %   folder, so it works when launched from anywhere. which() is safe here because
            %   the app had to be resolvable to be launched at all.
            app.Root = fileparts(which(class(app)));
            addpath(app.Root);

            setStatus(app, 'busy', 'building the configuration and the robot tree');
            app.Cfg = hmmSetup().cfg;

            % RE-DERIVED FROM hmmSetup, which is at the project root by construction, so
            % the app finds data/hmmNavMap.mat even if a copy of this file is run from
            % somewhere else - it would otherwise quietly show an empty map.
            atRoot = fileparts(which('hmmSetup'));
            if ~isempty(atRoot)
                app.Root = atRoot;
            end

            % The catalogue is hmmPlotRun's, unchanged, so the app cannot describe a run
            % differently from the documentation. The detections no longer appear in this
            % list because they have a tab of their own.
            app.Views = hmmPlotRun();
            app.ViewDropDown.Items = {app.Views.Title};
            app.ViewDropDown.ItemsData = {app.Views.Key};
            app.ViewDropDown.Value = app.Views(1).Key;

            loadNavMap(app);
            showDefaults(app);
            forgetPreviousSession(app);
            showGrounding(app);
            initMap(app);
            resetState(app);
            startPolling(app);
            updateLoadEnable(app);

            % LIVE WHEN THERE IS A KEY, SAMPLE RESPONSES WHEN THERE IS NOT. Real answers are the
            % point, but without a key the example commands still run from their sample responses.
            [~, route] = hmmProviderKey('live', app.Cfg);
            if route == app.Cfg.provider.ROUTE_NONE
                app.LiveCheckBox.Value = false;
                setRoute(app, 'idle', ['no Gemini key - the example commands use sample ' ...
                    'responses']);
            else
                app.LiveCheckBox.Value = true;
            end

            setStatus(app, 'ok', ['ready - press Start simulation, then type an ' ...
                'instruction and Send it']);
        end

        function forgetPreviousSession(app)
            %FORGETPREVIOUSSESSION Open on nothing, rather than on somebody else's run.
            %   BOTH OF THE APP'S LIVE CHANNELS OUTLIVE A RUN, AND BOTH LIE ON A FRESH START.
            %   The SDI keeps every archived run and getCurrentSimulationRun goes on
            %   returning the last one indefinitely; root appdata keeps the last grounding
            %   for the whole MATLAB session. Measured on the first launch after a scripted run:
            %   the console opened reading "picking the object up - moving the arm", with the
            %   chassis parked 0.03 m from a goal on the map and a green route lamp saying
            %   "grounded, 1 action", none of which had anything to do with the session that
            %   had just started. Every one of those is a confident statement about a robot
            %   that is not moving.
            %
            %   So both are disowned here, on exactly the same terms startRun disowns them.
            %   The cost is that a second console opened onto a run already in flight would
            %   blank the first one's detections until the next grounding, which is why the
            %   run in progress is the one case left alone.
            if isRunning(app)
                return
            end
            markSdiBaseline(app);
            if isappdata(0, 'hmmGroundingTap')
                rmappdata(0, 'hmmGroundingTap');
            end
            % AND THE ROUTE, FOR THE SAME REASON. A plan left in appdata by the previous
            % run - or by a script that calls hmmPlanBasePath directly - would be
            % drawn over the new map as though this robot were about to drive it, and it
            % would also set the leg boundary and so cut the new run's trail.
            if isappdata(0, 'hmmBasePlanTap')
                rmappdata(0, 'hmmBasePlanTap');
            end
            % The pushed state too (AppStateTap): left over, it put the last run's robot
            % on a freshly opened map (2026-10-15).
            if isappdata(0, 'hmmAppState')
                rmappdata(0, 'hmmAppState');
            end
            app.PlanGoal = [];
            app.LegStartTime = -Inf;
            app.LastTap = -1;
        end

        function loadNavMap(app)
            %LOADNAVMAP The shipped occupancy grid, converted to world coordinates once.
            %   READ FROM THE FILE RATHER THAN FROM A RUN, so the Map tab is worth looking at
            %   before anything has been simulated: the apartment, the home pose and the
            %   cameras are all facts about the project, not about a particular run.
            %
            %   ROW INDEXES y AND COLUMN INDEXES x, which is the transpose of how a matrix
            %   reads on the page and the single most likely thing to get backwards here. The
            %   cell centres are computed once, from the map's own provenance, so every
            %   drawing afterwards is in metres and nothing else in this app touches a cell
            %   index at all.
            %
            %   AND ROW 1 IS THE LARGEST y, NOT THE SMALLEST. That is the direction this app
            %   had backwards, and it drew a mirrored apartment for several sessions. The grid
            %   on disk is an occupancyMatrix, whose first row is the TOP of the map, and
            %   makeNavMap says so in as many words about the floor plan it is built from:
            %   "column indexes x upward from world(1,1), row indexes y DOWNWARD from
            %   world(2,2)". Measured against the shipped file: the building is at
            %   y in [-8 0], grid2world puts the free cells at y in [-7.97 -0.03], and this
            %   app was drawing them at y in [-5.97 1.97] - a mirror about the grid's centre
            %   line, y = -3, which moved the kitchen from y = -6.5 to y = +0.5.
            %
            %   yc IS STILL ASCENDING, because it is the AXIS extent and ylim demands an
            %   increasing pair. The image is what has to be told which end row 1 belongs at,
            %   and initMap does that by handing it a descending YData.
            app.NavMap = [];
            file = fullfile(app.Root, 'data', 'hmmNavMap.mat');
            if ~isfile(file)
                return
            end
            try
                M = load(file);
                m.grid   = M.grid;
                m.res    = M.provenance.resolution;
                m.origin = M.provenance.origin;
                m.thr    = M.provenance.occupiedThreshold;
                [nr, nc] = size(m.grid);
                m.xc = m.origin(1) + ((1:nc) - 0.5) / m.res;
                m.yc = m.origin(2) + ((1:nr) - 0.5) / m.res;
                app.NavMap = m;
            catch
                app.NavMap = [];
            end
        end

        function showDefaults(app)
            %SHOWDEFAULTS Put the shipped configuration into the controls.
            %   Inf rather than a budget: this is an interactive session, and a finite stop
            %   time either cut a transfer off mid-place (60 s left ~20 s of margin over a
            %   ~38 s transfer) or ended the session before a second instruction. The
            %   operator ends the run with Stop simulation.
            app.StopTimeField.Value = Inf;
            app.LiveViewBox.Value   = false;
            app.ChaseViewBox.Value  = false;
        end

        function cfg = overriddenConfig(app)
            %OVERRIDDENCONFIG The derived cfg with this session's control values written in.
            %   It starts from the cfg HMMSETUP published, not from hmmConfig(), and that is
            %   load bearing: hmmConfig ships cfg.track.taskKp and taskKd as scalar
            %   placeholders and hmmSetup replaces them with the 6x6 operational-space
            %   matrices the Task Space Motion Model block requires. Passing a cfg carrying
            %   the placeholders fails inside the arm controller's mask with a message about
            %   proportionalGain having the wrong number of columns, which names the wrong
            %   culprit entirely.
            %   ONLY THE TWO VIEW SWITCHES ARE OVERRIDES; the run uses hmmConfig as shipped.
            %   cfg.view.chase.enabled reaches the model through the base cfg this is assigned
            %   to: it drives Plant/ChaseCamera's enable port, and hmmInitializeRun opens the
            %   chase Video Viewer only when it is on.
            cfg = app.Cfg;
            cfg.camera.liveView = logical(app.LiveViewBox.Value);
            cfg.view.chase.enabled = logical(app.ChaseViewBox.Value);
        end

        %% ----------------------------------------------------------------- running

        function tf = isRunning(app)
            %ISRUNNING Whether the solver is advancing right now.
            %   Anything that is not 'stopped' counts. A model that is 'initializing' or
            %   'compiling' has certainly not finished, and treating those as stopped would
            %   have the app declare the run over during the several seconds Sim3D spends
            %   loading the apartment.
            tf = bdIsLoaded(app.Model) && ...
                ~strcmp(get_param(app.Model, 'SimulationStatus'), 'stopped');
        end

        function onStartStop(app, ~)
            %ONSTARTSTOP One button for both, because they are the same decision.
            if isRunning(app)
                setStatus(app, 'busy', 'stopping');
                set_param(app.Model, 'SimulationCommand', 'stop');
                return
            end
            startRun(app);
        end

        function blk = carryBlock(app)
            %CARRYBLOCK The Simulation 3D Robot block whose carry offset onSend retunes.
            %   THE PATH IS TRIED FIRST AND SEARCHED SECOND, deliberately in that order. The
            %   known path is one get_param on a loaded model; find_system under masks and
            %   links walks the whole hierarchy and is worth paying for only when the
            %   architecture has moved. Either way the block is identified by the mask
            %   parameter it must carry, so a wrong block cannot be returned silently.
            %   getSimulinkBlockHandle, not find_system, tests the path: find_system on a
            %   path that has moved throws, so the search below was never reached.
            blk = [app.Model '/MobileManipulator/Plant/RobotActor/RobotTree'];
            if getSimulinkBlockHandle(blk) > 0 && ...
                    strcmp(get_param(blk, 'MaskType'), 'Simulation 3D Robot')
                return
            end
            hit = find_system(app.Model, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                'MaskType', 'Simulation 3D Robot');
            if isempty(hit)
                error('hmmApp:carryBlock', ...
                    'no Simulation 3D Robot block found in %s', app.Model);
            end
            blk = hit{1};
        end

        function startRun(app)
            %STARTRUN Launch the model asynchronously with the current values.
            try
                if ~bdIsLoaded(app.Model)
                    load_system(app.Model);
                end

                % Remembered the first time only, so that a second start does not record
                % the value the app itself left behind and lose the model's own.
                if isempty(app.StopTimeWas)
                    app.StopTimeWas = get_param(app.Model, 'StopTime');
                end

                app.RunCfg = overriddenConfig(app);
                app.SdiSampleCache = [];
                % The previous run's output would otherwise stand in for this run's live plots.
                app.Out = [];
                app.PlotRefreshTic = [];
                assignin('base', 'cfg', app.RunCfg);
                hmmRuntimeConfig(app.RunCfg);
                set_param(app.Model, 'StopTime', num2str(app.StopTimeField.Value));

                % WHICH PROP'S PIVOT OFFSET THE FRESH COMPILE IS ABOUT TO BAKE IN. The mask
                % expression evaluates cfg.task.instruction, so this is not a guess - it is the
                % same resolver on the same sentence. Recording it here means onSend retunes
                % only when the operator's sentence names something ELSE, so the common case of
                % typing the shipped instruction writes nothing and logs nothing.
                try
                    ws = warning('off', 'hmm:carryTranslation:unresolved');
                    restoreWarn = onCleanup(@() warning(ws));
                    [~, ~, app.CarryNameApplied] = hmmCarryTranslation(app.RunCfg);
                catch
                    % Unknown is the safe value: it makes the first submission retune rather
                    % than assume, which is the right way round to be wrong.
                    app.CarryNameApplied = '';
                end

                applyProviderMode(app);
                markSdiBaseline(app);
                resetState(app);

                % NOTHING IS SUBMITTED AT A START, and this line is what guarantees it.
                % hmmSetup seeds cfg.task.instruction so that unattended callers keep
                % working, and root appdata survives a run, so without clearing here the
                % model would ground either the shipped sentence or the last one typed,
                % at t = 0, before the operator had seen the apartment.
                hmmSubmitTask('clear');
                if isappdata(0, 'hmmGroundingTap')
                    rmappdata(0, 'hmmGroundingTap');
                end
                % The previous run's route goes with it. See markSdiBaseline's note: a
                % stale plan is drawn as though it were about to be driven, and it sets
                % the leg boundary that cuts this run's trail.
                if isappdata(0, 'hmmBasePlanTap')
                    rmappdata(0, 'hmmBasePlanTap');
                end
                app.PlanGoal = [];
                app.LegStartTime = -Inf;
                if app.MapReady
                    % Guarded, because with no data/hmmNavMap.mat initMap draws a message
                    % instead of a map and these handles were never created.
                    set(app.MapH.plan, 'XData', NaN, 'YData', NaN);
                    set(app.MapH.waypoints, 'XData', NaN, 'YData', NaN);
                end
                app.LastTap = -1;
                app.WristPane = 0;
                app.LastLiveSeq = 0;
                if isappdata(0, 'hmmLiveViewTap')
                    rmappdata(0, 'hmmLiveViewTap');
                end
                % The pushed state: last run's must not be shown as this one's, and the
                % tap publishes nothing unless asked (hmmAppStateTap).
                if isappdata(0, 'hmmAppState')
                    rmappdata(0, 'hmmAppState');
                end
                setappdata(0, 'hmmAppStateWanted', true);
                showGrounding(app);

                % A live run replaces any recording on display, and starts one if asked.
                stopReplay(app);
                beginRecording(app);

                % THE WRIST LOOK IS PUSHED, NOT POLLED - see hmmRefineNotify. The refinement
                % blocks a simulation step on the provider, so this timer could only ever see
                % the frame and Gemini's answer together, afterwards.
                setappdata(0, 'hmmRefineListener', @(ev, rec) onRefineEvent(app, ev, rec));

                set_param(app.Model, 'SimulationCommand', 'start');
                app.WasRunning = true;
                app.LoadButton.Enable = 'off';               % no loading during a run
                app.StartStopButton.Text = 'Stop simulation';
                pushLog(app, NaN, 'run started - waiting for an instruction');
                setStatus(app, 'busy', ['starting - the apartment opens in the Sim3D ' ...
                    'window; the robot waits at its home pose']);
            catch ME
                setStatus(app, 'error', 'the model would not start - see the report pane');
                app.ReportArea.Value = cellstr(splitlines(string(ME.getReport('basic'))));
                uialert(app.UIFigure, ME.message, 'Could not start');
            end
        end

        function markSdiBaseline(app)
            %MARKSDIBASELINE Remember which SDI run already existed, so it is not mistaken.
            %   getCurrentSimulationRun KEEPS RETURNING THE LAST RUN AFTER IT ENDS, which is
            %   exactly what the plots want and exactly what a fresh start must not have: for
            %   the few seconds between pressing Start and the new run appearing, the SDI
            %   would hand over the PREVIOUS run and the map would draw its trail as though
            %   the robot had already driven it. So the old run's id is recorded here and
            %   ignored until a different one shows up.
            app.SdiRunBefore = -1;
            try
                prev = Simulink.sdi.getCurrentSimulationRun(app.Model);
                if ~isempty(prev)
                    app.SdiRunBefore = prev.ID;
                end
            catch
            end
            app.SdiRunId = -1;
            app.SdiMap = [];
            app.SlowState = [];
            app.PollCount = 0;
        end

        function applyProviderMode(app)
            %APPLYPROVIDERMODE Put the provider into the mode the checkbox asks for.
            %   THE PERSISTENTS HAVE TO GO WITH IT, and forgetting that is a trap worth
            %   spelling out. hmmConfig reads GEMINI_PROVIDER every call, but
            %   hmmGroundingService caches the whole configuration in a persistent that
            %   lives for the MATLAB session, not for one run. Setting the variable without
            %   clearing that cache changes the mode everywhere except the one place that
            %   decides - so the app would report live and replay a recording.
            %
            %   Only between runs. clear on a function whose persistent is mid-simulation
            %   would drop state the solver is relying on.
            if app.LiveCheckBox.Value
                setenv('GEMINI_PROVIDER', 'live');
            else
                setenv('GEMINI_PROVIDER', 'replay');
            end
            clear hmmGroundingService hmmProviderRequest hmmOperatorTask
        end

        function onLiveChanged(app, ~)
            %ONLIVECHANGED Say what the new setting means, and when it takes effect.
            if app.LiveCheckBox.Value
                setStatus(app, 'ok', ['live at the next start - the instruction will be ' ...
                    'sent to the provider']);
            else
                setStatus(app, 'ok', ['sample responses at the next start - only the ' ...
                    'example commands can run']);
            end
        end

        function onChaseChanged(app, ~)
            %ONCHASECHANGED Turn the chase view on or off now, during a run as well as before one.
            %   Before a run, overriddenConfig applies the box at Start. DURING ONE, the box was
            %   ticked after the run read it, and the first user to try it saw nothing - so the
            %   change is pushed through root appdata 'hmmChaseViewOn', which Plant/ChaseCamera's
            %   enable reads every step (hmmChaseViewOn), and the viewer window, whose
            %   OpenAtMdlStart was read at the start, is opened or closed by hand.
            %
            %   NOT A PARAMETER UPDATE, which is what a first version tried: cfg is one struct,
            %   so updating it re-evaluates every block that reads it, and the five Simulation 3D
            %   Camera blocks refuse mid-run. And not hmmRuntimeConfig: that version also
            %   republished app.RunCfg, which is empty on a run started from the Simulink window,
            %   so it published a struct holding this one field and every block reading the
            %   configuration failed at once (2026-10-20).
            if ~isRunning(app)
                return
            end
            on = logical(app.ChaseViewBox.Value);
            try
                setappdata(0, 'hmmChaseViewOn', on);
                viewer = find_system(app.Model, 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
                    'Name', 'ChaseView');
                if ~isempty(viewer) && on
                    open_system(viewer{1});
                elseif ~isempty(viewer)
                    close_system(viewer{1});
                end
                if on
                    setStatus(app, 'ok', 'chase camera on');
                else
                    setStatus(app, 'ok', 'chase camera off');
                end
            catch err
                setStatus(app, 'error', sprintf('chase camera not switched: %s', err.message));
            end
        end

        %% -------------------------------------------------------------- submitting

        function onExample(app, ~)
            %ONEXAMPLE Put one of the three kinds of command into the prompt box.
            %   The dropdown springs back to its own first item afterwards, so it reads as a
            %   menu of things to try rather than as a record of what was sent - the prompt
            %   box below it is the only statement of that, and it stays editable.
            items = app.ExampleDropDown.Items;
            chosen = app.ExampleDropDown.Value;
            if ~strcmp(chosen, items{1})
                app.PromptArea.Value = {chosen};
            end
            app.ExampleDropDown.Value = items{1};
        end

        function onSend(app, ~)
            %ONSEND Hand the running model the sentence in the prompt box.
            txt = strtrim(strjoin(string(app.PromptArea.Value), ' '));

            if strlength(txt) == 0
                uialert(app.UIFigure, ...
                    'Type an instruction first, for example "move the mug to the kitchen".', ...
                    'Nothing to send', 'Icon', 'info');
                return
            end

            if ~isRunning(app)
                uialert(app.UIFigure, ['The model is not running, so there is nothing to ' ...
                    'submit to. Press Start simulation, wait for the apartment to appear, ' ...
                    'and then send the instruction.'], 'Not running', 'Icon', 'info');
                return
            end

            state = readLiveState(app);
            if state.ok && state.outcome == double(app.RunCfg.sup.out.PENDING) && ...
                    state.plannerMode ~= double(app.RunCfg.sup.pl.IDLE)
                uialert(app.UIFigure, 'Cancel or finish the active task before sending another.', ...
                    'Task in progress', 'Icon', 'info');
                return
            end

            % RETUNE THE CARRY OFFSET TO THE OBJECT THIS SENTENCE NAMES, NOW, MID-RUN.
            % Simulation 3D Robot does not lift the prop from where the jaws closed on it:
            % TargetGraspUtils/attachTargetActor parents it to the end effector and then
            % overwrites its pose with the block's end-effector-to-target translation. That
            % offset is per-MESH - 21.1 mm for the mug, 39.9 mm for the Bottle, and up to
            % 15.5 mm of lateral pivot across the graspable set - so a value chosen for one
            % prop pins the wrong origin on another and the object visibly jumps as the
            % gripper closes.
            %
            % THIS USED TO BE A WARNING, AND THE WARNING WAS BUILT ON A FALSE PREMISE. Both
            % this file and hmmCarryTranslation asserted that the offset had to be resolved at
            % compile time because the block gives no runtime route to it, so the prop was
            % fixed by cfg.task.instruction before Start and a sentence naming anything else
            % could only be reported. Reading the block's own source refutes it twice over:
            % Simulation3DRobotBlock declares TargetEndEffectorTranslation in a plain
            % properties block and NOT under Nontunable, and stepImpl calls
            % updateTargetGraspSettings on every step rather than latching the value at setup.
            % So the parameter is tunable while the simulation runs. Measured directly against
            % a live run: set_param on RobotTree's eeTargetTrans mid-run is accepted and the
            % simulation keeps going.
            %
            % AND THE TIMING IS COMFORTABLE, which is what makes this a fix rather than a
            % race. The offset is read when the grasp transition fires, and that is hundreds of
            % steps after a submission - the base still has to plan, drive and arrive. There is
            % no need to enable the block's input port, so there is no model edit here.
            %
            % NOTHING HERE READS A TRUE OBJECT POSE, which is the constraint that governs this
            % whole project. hmmCarryTranslation matches instruction TEXT against manifest
            % keys and touches no actor; a mesh pivot offset is a calibration constant about a
            % .fbx file, in the same class as a camera intrinsic, not a measurement of where
            % anything currently is.
            try
                % ITS UNRESOLVED WARNING IS SILENCED HERE AND ONLY HERE. That warning is
                % addressed to whoever set up a TRANSFER whose object cannot be found, and it
                % is right to shout then. This call asks the question speculatively, about a
                % sentence that may well be a navigate or a home - neither of which carries
                % anything - so on those it would fire on every single submission and teach
                % the operator to ignore it. The state is restored immediately.
                ws = warning('off', 'hmm:carryTranslation:unresolved');
                restoreWarn = onCleanup(@() warning(ws));

                % RESOLVED FROM THE TYPED SENTENCE, through the same function hmmSetup used,
                % so the two cannot drift. RunCfg is what startRun published to the base
                % workspace as cfg, so everything else about the resolution - the manifest, the
                % aperture gate, the grasp fractions - is identical to the compile-time call.
                typedCfg = app.RunCfg;
                typedCfg.task.instruction = char(txt);
                [carryVec, carryOff, typedName] = hmmCarryTranslation(typedCfg);

                % A SENTENCE THAT NAMES NO CARRIABLE PROP LEAVES THE BLOCK ALONE. A navigate or
                % a home command grasps nothing, so there is no offset to be right or wrong
                % about, and writing zeros would quietly discard a correct value that a
                % following transfer would then need.
                if ~isempty(typedName) && ~strcmp(typedName, app.CarryNameApplied)
                    blk = carryBlock(app);

                    % Remembered once, so a second submission does not record the literal this
                    % app itself wrote and lose the model's own expression. Same reasoning as
                    % StopTimeWas, and restored in the same place.
                    if isempty(app.CarryTransWas)
                        app.CarryTransWas = get_param(blk, 'eeTargetTrans');
                    end

                    % A LITERAL AND NOT A VARIABLE NAME. The mask expression ships as
                    % carryTranslation, a base-workspace variable; rewriting that variable
                    % would not reach a compiled model without a diagram update, which this
                    % project does not do mid-run. Writing the numbers themselves does.
                    set_param(blk, 'eeTargetTrans', mat2str(carryVec(:)', 6));
                    app.CarryNameApplied = typedName;

                    pushLog(app, NaN, sprintf(['carry offset retuned for %s: held %.1f mm ' ...
                        'below tcp_frame, pivot [%.1f %.1f] mm lateral'], typedName, ...
                        carryOff * 1000, carryVec(1) * 1000, carryVec(2) * 1000));
                end
            catch ME
                % A DIAGNOSTIC MAY NOT BREAK A SUBMISSION, so the task still goes either way.
                % But if the retune itself failed - a future release marking the property
                % Nontunable, a renamed mask parameter, an old RunCfg without the field - then
                % the old fault is back and silence would be the worst answer. So this reports
                % what it could not do, by name.
                pushLog(app, NaN, sprintf(['could not retune the carry offset (%s) - if this ' ...
                    'sentence names a different object than the last one, expect it to jump ' ...
                    'when the gripper closes'], ME.message));
            end

            info = hmmSubmitTask(char(txt));
            setRoute(app, 'busy', sprintf('task %d sent, waiting for Gemini ER', info.taskId));
            pushLog(app, NaN, sprintf('sent: "%s"', txt));
            if info.truncated
                uialert(app.UIFigure, sprintf(['The instruction is longer than the %d ' ...
                    'characters the TaskRequest bus carries and was truncated. What the ' ...
                    'provider sees is the beginning of it.'], ...
                    app.Cfg.provider.instructionChars), 'Instruction truncated', ...
                    'Icon', 'warning');
            end
        end

        function onCancel(app, ~)
            %ONCANCEL Abandon the task in flight.
            if ~isRunning(app)
                setRoute(app, 'idle', 'nothing to cancel - the model is not running');
                return
            end
            info = hmmSubmitTask('cancel');
            setRoute(app, 'busy', sprintf('cancel submitted as task %d', info.taskId));
            pushLog(app, NaN, 'cancel submitted');
        end

        %% ------------------------------------------------- the live state, from the SDI

        function s = readLiveState(app)
            %READLIVESTATE The pose and the phase, as they are right now.
            %   THIS IS THE WHOLE REASON THE PANEL NEEDS NO MODEL EDIT. Logged signals stream
            %   to the Simulation Data Inspector while the solver steps, so everything the
            %   left column and the map need is already on a signal that is already logged.
            %
            %   Every field has a defined empty value, and s.ok is the single question a
            %   caller has to ask. A half-populated struct with some fields missing would put
            %   the burden of that check on every use site instead.
            s = struct('ok', false, 't', NaN, 'x', NaN, 'y', NaN, 'yaw', NaN, ...
                'trailX', [], 'trailY', [], 'trailT', [], ...
                'plannerMode', NaN, 'activeVerb', NaN, ...
                'outcome', NaN, 'baseState', NaN, 'armMode', NaN, 'gripState', NaN, ...
                'gripFail', NaN, 'gripSlipped', false, ...
                'eStop', false, 'eStopCause', NaN, 'pickXY', [], 'placeXY', [], ...
                'elapsed', NaN, 'distToGoal', NaN);

            % THE MODEL PUSHES THIS STATE, so the SDI is only the fallback. The root
            % AppStateTap block publishes these same fields every 0.2 s of model time
            % (hmmAppStateTap), and a getappdata costs microseconds where the ~10 SDI reads
            % below cost ~0.25 s of every poll - time the simulation, on the same thread,
            % lost. The SDI path stays for a model saved without the tap.
            if isappdata(0, 'hmmAppState')
                pushed = getappdata(0, 'hmmAppState');
                if isstruct(pushed) && isfield(pushed, 'seq')
                    s = pushed;
                    return
                end
            end

            % THE CURRENT RUN CANNOT CHANGE WHILE THE MODEL IS RUNNING, so once this run's
            % table is built it is not asked for again until the run ends: the query costs
            % ~60 ms during a run, a quarter of what a poll was left costing.
            known = app.SdiRunId > 0 && app.SdiRunId ~= app.SdiRunBefore && ...
                ~isempty(app.SdiMap) && isRunning(app);
            if ~known
                run = [];
                try
                    run = Simulink.sdi.getCurrentSimulationRun(app.Model);
                catch
                end
                if isempty(run) || run.ID == app.SdiRunBefore
                    % Nothing has ever run, or the only run the SDI knows about is the one
                    % that was already there when this one started. See markSdiBaseline.
                    return
                end
                if run.ID ~= app.SdiRunId || isempty(app.SdiMap)
                    buildSdiMap(app, run);
                end
            end
            if isempty(app.SdiMap)
                return
            end

            % THE TIME VECTOR COMES OFF THE SAME CHANNEL AS THE X HISTORY, which matters
            % for the per-leg clearing below. RobotState.Timestamp is a separate SDI
            % channel and can be a sample ahead or behind, so indexing the pose history
            % with it would cut the trail at the wrong point - near-invisibly, by one
            % sample, until the sample rates differ.
            [s.x, s.trailX, s.trailT] = sdiRead(app, 'RobotState.BasePose(1,1)', true);
            [s.y, s.trailY] = sdiRead(app, 'RobotState.BasePose(2,1)', true);
            s.yaw           = sdiRead(app, 'RobotState.BasePose(3,1)', false);
            if ~isempty(s.trailT)
                s.t = s.trailT(end);             % the pose's own clock, not a 4th read
            end

            s.plannerMode = sdiRead(app, 'TaskResult.PlannerMode', false);

            % TWO TIERS, BECAUSE EVERY READ COSTS ~11 ms WHATEVER IT FETCHES. The pose, the
            % four modes and the e-stop are read every poll; the fields that change at a
            % phase boundary (verb, outcome, codes, targets) every third poll, and at once
            % when the planner mode changes. 23 reads a poll held MATLAB for 26% of the
            % run (profiled 2026-10-14) - time the simulation, on the same thread, lost.
            slow = isempty(app.SlowState) || mod(app.PollCount, 3) == 0 || ...
                ~isequaln(s.plannerMode, app.SlowState.plannerMode);
            app.PollCount = app.PollCount + 1;
            if slow
                s.activeVerb  = sdiRead(app, 'TaskResult.ActiveVerb', false);
                s.outcome     = sdiRead(app, 'TaskResult.Outcome', false);
                s.elapsed     = sdiRead(app, 'TaskResult.ElapsedTime', false);
                s.eStopCause  = sdiRead(app, 'TaskResult.EStopCause', false);
            end
            % .Mode, NOT .State, AND THE THREE STATUS BUSES ALL CARRY BOTH. hmmDefineBuses
            % (lines 377, 404, 430) says Mode is the cfg.sup.base/arm/grip mode the chart is
            % in, while State is cfg.sup.st - IDLE, BUSY, DONE, FAILED - the service handshake
            % against ServedRequestId. Two of these read State and decoded it with the mode
            % table, which is a silent mistranslation because both enumerations start at 0 and
            % count up. Measured on the live navigate run of 2026-09-30: BaseStatus.State went
            % 0 -> 1 at t = 1.96 s -> 2 at t = 10.20 s, i.e. IDLE -> BUSY -> DONE, so the panel
            % read "driving to the place you named - PLANNING A PATH" for the whole 8-second
            % drive (BUSY = 1 = AWAIT_PLAN) and would have read "moving" while parked at the
            % goal (DONE = 2 = FOLLOWING). BaseStatus.Mode on the same run went 0 -> 1 at
            % 1.96 -> 2 at 1.98 -> 4 at 10.20: one tick of AWAIT_PLAN, then FOLLOWING for the
            % drive, then HOLDING on arrival, which is what the operator should have been shown.
            %
            % The gripper had the same read and it is worse for being nearly invisible: BUSY
            % lands on CLOSING and DONE on CLOSED, which look right during a grasp, but FAILED
            % lands on VALIDATED - a refused grasp displayed as plant-confirmed capture - and
            % OPENING and FAULTED can never appear at all, because cfg.sup.st stops at 3.
            s.baseState   = sdiRead(app, 'BaseStatus.Mode', false);
            s.armMode     = sdiRead(app, 'ArmStatus.Mode', false);
            s.gripState   = sdiRead(app, 'GripperStatus.Mode', false);
            s.eStop       = sdiRead(app, 'SafetyStatus.EStop', false) > 0;
            if ~slow
                % The slow tier as last read; see TWO TIERS above. Its plannerMode equals
                % this poll's, or the poll would have been a slow one.
                for f = fieldnames(app.SlowState)'
                    s.(f{1}) = app.SlowState.(f{1});
                end
                s.ok = isfinite(s.x) && isfinite(s.y) && isfinite(s.yaw);
                return
            end
            % GRIPPER MODE ALONE CANNOT TELL A FAULT FROM A CANCELLATION, and reporting it
            % as though it could cost a whole debugging session on 2026-10-04. The user's
            % report was "in this run there was a faulted grip"; the gripper had not
            % faulted. GripperSupervisor has two ways into Faulted that mean opposite
            % things: [951] GraspValidated -> Faulted [~IsGrasped], a real dropped grasp,
            % which sets FailureCode = F_NOTGRASP and Slipped = true; and [954] Operating ->
            % Faulted [GripperCommand.Cancel || GripperCommand.Stop], a COMMANDED stand-down
            % by the task planner, which sets State = S_IDLE and deliberately touches
            % neither code nor Slipped. On that run the mode went VALIDATED -> FAULTED at
            % t = 31.18 s with Grasped still 1, FailureCode still 0, Slipped still 0 and the
            % aperture bit-identical - [954], not [951], because the planner had just
            % abandoned NavToPlace. So the two are distinguished here by the fields the
            % chart does set, and the operator is told which one happened.
            s.gripFail    = sdiRead(app, 'GripperStatus.FailureCode', false);
            s.gripSlipped = sdiRead(app, 'GripperStatus.Slipped', false) > 0;
            % THE TASK'S OWN CAUSE, for RECOVER and FAILED. Without it the only detail on a
            % failure was whichever subsystem was last touched - on 2026-10-12 a place point
            % refused as PlaceUnreachable read "the gripper was stood down by the task".
            s.taskFail    = sdiRead(app, 'TaskResult.FailureCode', false);
            s.distToGoal  = sdiRead(app, 'BaseStatus.DistanceToGoal', false);

            % The two world targets, drawn on the map only while the model says they mean
            % something. A stale target left on screen after a failure would suggest the
            % robot is still going for it.
            % THE REFINED BUS FIRST, THE COARSE ONE ONLY AS A FALLBACK. ObjectTargetRefined
            % is what Manipulator and Gripper consume, so it is the point the arm is
            % actually going to; ObjectTarget is what the room cameras grounded before the
            % wrist looked again. The difference is about 40 mm, far below what a 10 x 8 m
            % map can show, so this is not a visible correction - it is drawn from the
            % right source so that a reader who zooms in, or who compares the marker
            % against the report, is not shown two different numbers for one pick. A run
            % recorded before the refined bus was logged misses the channel, sdiRead
            % returns a non-finite, and the isfinite guard falls through to the coarse one.
            if sdiRead(app, 'ObjectTargetRefined.Valid', false) > 0
                px = sdiRead(app, 'ObjectTargetRefined.Position(1,1)', false);
                py = sdiRead(app, 'ObjectTargetRefined.Position(2,1)', false);
                if isfinite(px) && isfinite(py)
                    s.pickXY = [px py];
                end
            end
            % isempty, NOT isfield: s is built above with every field already present and
            % set to a defined empty, so isfield would be true here no matter what and the
            % fallback would never run.
            if isempty(s.pickXY) && sdiRead(app, 'ObjectTarget.Valid', false) > 0
                px = sdiRead(app, 'ObjectTarget.Position(1,1)', false);
                py = sdiRead(app, 'ObjectTarget.Position(2,1)', false);
                if isfinite(px) && isfinite(py)
                    s.pickXY = [px py];
                end
            end
            if sdiRead(app, 'PlaceTarget.Valid', false) > 0
                qx = sdiRead(app, 'PlaceTarget.Point(1,1)', false);
                qy = sdiRead(app, 'PlaceTarget.Point(2,1)', false);
                if isfinite(qx) && isfinite(qy)
                    s.placeXY = [qx qy];
                end
            end

            app.SlowState = struct('plannerMode', s.plannerMode, 'activeVerb', s.activeVerb, ...
                'outcome', s.outcome, 'elapsed', s.elapsed, 'eStopCause', s.eStopCause, ...
                'gripFail', s.gripFail, 'gripSlipped', s.gripSlipped, 'taskFail', s.taskFail, ...
                'distToGoal', s.distToGoal, 'pickXY', {s.pickXY}, 'placeXY', {s.placeXY});
            s.ok = isfinite(s.x) && isfinite(s.y) && isfinite(s.yaw);
        end

        function buildSdiMap(app, run)
            %BUILDSDIMAP Channel name to signal id, once per run.
            %   182 signals is far too many to search by name on every poll, and the ids are
            %   stable for the life of a run, so the lookup is built once. It is rebuilt when
            %   a wanted name misses, because the SDI creates a vector's later elements
            %   LAZILY: RobotState.BasePose(1,1) exists for several seconds before (2,1) and
            %   (3,1) do, and a table built in that window is genuinely incomplete rather
            %   than wrong.
            app.SdiMap = containers.Map('KeyType', 'char', 'ValueType', 'double');
            try
                ids = run.getAllSignalIDs();
                for k = 1:numel(ids)
                    sig = Simulink.sdi.getSignal(ids(k));
                    app.SdiMap(char(sig.Name)) = double(ids(k));
                end
                app.SdiRunId = run.ID;
            catch
                app.SdiMap = [];
            end
        end

        function id = sdiLookup(app, name)
            %SDILOOKUP One channel's signal id, under either element spelling.
            %   THE SDI SPELLS AN ELEMENT BY THE SIGNAL'S SHAPE: X(2,1) for a 3x1 column, X(2)
            %   for a 1-D vector of 3. RobotState.BasePose changed shape and the map went blank
            %   with no error (2026-10-12) - every pose read came back NaN, so the robot and
            %   route were silently skipped. So the other spelling is tried before giving up.
            %
            %   BOTH SPELLINGS ARE TRIED BEFORE ANY REBUILD. Trying the exact one, rebuilding
            %   on its miss and only then trying the other made every poll rebuild the whole
            %   table for BasePose, which is stored as X(1): 0.9 s of a 1.27 s poll, measured
            %   with the profiler over a transfer (2026-10-14), and the run three times slower.
            alt = regexprep(name, '\((\d+),1\)$', '($1)');
            if strcmp(alt, name)
                alt = regexprep(name, '\((\d+)\)$', '($1,1)');
            end
            id = sdiFind(app, name, alt);
            if ~isempty(id) || isempty(app.SdiMap) || app.SdiRebuilt
                return
            end
            % A genuine miss: the SDI creates a vector's later elements lazily. Rebuild at
            % most once per poll and once per 5 s, so a channel that never appears (an older
            % recording) cannot put the 0.9 s rebuild back into every poll.
            if ~isempty(app.SdiRebuildTic) && toc(app.SdiRebuildTic) < 5
                return
            end
            app.SdiRebuilt = true;
            app.SdiRebuildTic = tic;
            try
                run = Simulink.sdi.getCurrentSimulationRun(app.Model);
                if ~isempty(run)
                    buildSdiMap(app, run);
                    id = sdiFind(app, name, alt);
                end
            catch
            end
        end

        function id = sdiFind(app, name, alt)
            %SDIFIND The id under either spelling, from the table as it stands.
            id = [];
            if isempty(app.SdiMap)
                return
            end
            if isKey(app.SdiMap, name)
                id = app.SdiMap(name);
            elseif ~strcmp(alt, name) && isKey(app.SdiMap, alt)
                id = app.SdiMap(alt);
            end
        end

        function [val, series, times] = sdiRead(app, name, wantSeries)
            %SDIREAD The last value of one channel, and optionally its whole history.
            %   NaN FOR ABSENT, rather than an error or a zero. A channel that has not been
            %   created yet is the normal condition in the first seconds of a run, and zero
            %   is a plausible pose - it would draw the robot at the origin, in a wall.
            %
            %   THE THIRD OUTPUT IS THIS CHANNEL'S OWN TIME, which is the only safe way to
            %   cut a history at a model time: a timeseries read separately can differ in
            %   length from this one by a sample, and the map's per-leg clearing indexes
            %   one history with the other's time.
            val = NaN;
            series = [];
            times = [];
            id = sdiLookup(app, name);
            if isempty(id)
                return
            end
            try
                if isempty(app.SdiSampleCache)
                    app.SdiSampleCache = containers.Map('KeyType', 'double', 'ValueType', 'any');
                end
                cache = [];
                if isKey(app.SdiSampleCache, double(id))
                    cache = app.SdiSampleCache(double(id));
                end
                [val, series, times, cache] = hmmReadSdiSignal(id, cache, wantSeries);
                app.SdiSampleCache(double(id)) = cache;
            catch
            end
        end

        %% ------------------------------------------------------------------ polling

        function startPolling(app)
            %STARTPOLLING Watch the run, the tap and the SDI once a second.
            %   A TIMER, BECAUSE THERE IS NOTHING TO WAIT ON. An asynchronous simulation
            %   offers no completion callback that also fires while it progresses, and the
            %   three things this app must notice - a new answer from the provider, the state
            %   changing, and the run ending - all happen at times nobody chose. One second
            %   is well under human patience and far above the cost of the reads it does.
            stopPolling(app);
            app.Poll = timer('Name', 'HomeMobileManipulatorAppPoll', ...
                'ExecutionMode', 'fixedSpacing', 'Period', 1.0, ...
                'TimerFcn', @(~, ~) onPoll(app));
            start(app.Poll);
        end

        function stopPolling(app)
            %STOPPOLLING Take the timer down, whatever state it is in.
            %   Timers are not owned by the figure and outlive it. One left running after
            %   the app closes fires into a deleted object every second for the rest of the
            %   session.
            %
            %   GUARDED, BECAUSE isvalid IS NOT ENOUGH FOR A TIMER. Packaging the app caught
            %   this: the destructor ran on a timer that MATLAB had already taken down, and
            %   isvalid still said true, so stop() threw "Invalid timer object" out of a
            %   destructor - which turns a clean close into a page of warnings. There is
            %   nothing to recover here, and nothing to report: the only goal is that the
            %   timer is not running afterwards, and it already is not.
            try
                if ~isempty(app.Poll) && isa(app.Poll, 'timer')
                    stop(app.Poll);
                    delete(app.Poll);
                end
            catch
            end
            app.Poll = [];
        end

        function onPoll(app)
            %ONPOLL Notice a new grounding, follow the state, and notice the run finishing.
            if ~isvalid(app) || ~isvalid(app.UIFigure)
                stopPolling(app);
                return
            end

            try
                % One rebuild of the SDI lookup per poll at most. Reset here rather than in
                % sdiLookup, because "once" has to mean once per sweep of all the channels,
                % not once per channel.
                app.SdiRebuilt = false;

                running = isRunning(app);

                if running
                    t = get_param(app.Model, 'SimulationTime');
                    if isinf(app.StopTimeField.Value)
                        setStatus(app, 'busy', sprintf('running, t = %.1f s (until stopped)', t));
                    else
                        setStatus(app, 'busy', sprintf('running, t = %.1f of %g s', ...
                            t, app.StopTimeField.Value));
                    end
                    app.StartStopButton.Text = 'Stop simulation';
                elseif app.WasRunning
                    % The edge, taken once. Reading the outputs on every poll after a run
                    % ends would re-plot the same result for as long as the app is open.
                    app.WasRunning = false;
                    app.StartStopButton.Text = 'Start simulation';
                    collectRun(app);
                end

                % THE STATE IS FOLLOWED WHETHER OR NOT THE MODEL IS STILL RUNNING. The SDI
                % keeps the finished run, so the panel and the map hold the final pose and
                % the whole travelled path instead of blanking the moment the solver stops.
                s = readLiveState(app);
                updateState(app, s);
                updateMap(app, s);

                % THE PLOTS GROW DURING THE RUN, but only while their tab is the one on screen:
                % a refresh costs a recomputation of the view (~0.1-0.3 s), so a hidden tab
                % costs nothing. See refreshLivePlot.
                if running && app.TabGroup.SelectedTab == app.PlotTab && ...
                        (isempty(app.PlotRefreshTic) || toc(app.PlotRefreshTic) >= app.PlotRefreshEvery)
                    refreshLivePlot(app);
                    app.PlotRefreshTic = tic;
                end

                % THE WRIST FRAME IS TEMPORARY. It stands in for one ceiling view from the
                % moment the refinement is sent; once the arm starts its constrained descent -
                % or the planner has left PICK - the second look is over and the four fixed
                % views, with their pick and place marks, come back.
                if app.WristPane > 0 && (isequal(s.armMode, app.Cfg.sup.arm.APPROACH) || ...
                        (isfinite(s.plannerMode) && s.plannerMode ~= app.Cfg.sup.pl.PICK))
                    app.WristPane = 0;
                    showGrounding(app);
                end

                % A NEW ANSWER IS DETECTED BY REQUEST ID, NOT BY THE TAP EXISTING. The
                % struct sits in appdata for the rest of the session, so "is it there"
                % would redraw the same detections every second and would also show the
                % previous run's answer as though it were this one's.
                tap = getappdata(0, 'hmmGroundingTap');
                if ~isempty(tap) && tap.requestId ~= app.LastTap
                    app.LastTap = tap.requestId;
                    reportGrounding(app, tap);
                    showGrounding(app);
                    recordTap(app, tap);
                end
                recordPlan(app);

                % A NEW LIVE FRAME, drawn under the grounding marks - unless the wrist frame
                % holds one pane, which is the more important picture while it is up.
                % THE FIRST CAPTURE IS DRAWN EVEN WITH LIVE VIEW OFF - it is the start-up picture
                % (cfg.camera.startupCapture), and without it the panes stay empty until grounding.
                if (app.LiveViewBox.Value || app.LastLiveSeq == 0) && app.WristPane == 0
                    lv = getappdata(0, 'hmmLiveViewTap');
                    if isstruct(lv) && isfield(lv, 'seq') && lv.seq ~= app.LastLiveSeq
                        app.LastLiveSeq = lv.seq;
                        showGrounding(app, lv);
                        recordFrame(app, lv);
                    end
                end
            catch err
                % A poll that throws must not take the timer with it, or the app goes
                % quietly deaf: the run continues, the provider answers, and nothing
                % ever appears. Errors here are transient by nature - a get_param
                % against a model in the middle of terminating, an appdata struct read
                % while it is being written, an SDI signal queried as it is created.
                %
                % BUT NOT SILENTLY, EACH DISTINCT ONE IS PRINTED ONCE. A persistent error here
                % skips everything after it in the poll - on 2026-10-12 the map stopped
                % drawing with nothing to say why - so the first sighting of each message is
                % written to the command window, with where it came from.
                if ~strcmp(err.message, app.LastPollError)
                    app.LastPollError = err.message;
                    where = '';
                    if ~isempty(err.stack)
                        where = sprintf(' (%s, line %d)', err.stack(1).name, err.stack(1).line);
                    end
                    fprintf(2, 'HomeMobileManipulatorApp: poll error%s: %s\n', where, err.message);
                end
            end
        end

        function collectRun(app)
            %COLLECTRUN Pick up the outputs of the run that has just finished.
            %   NOT A BARE logsout. This model has ReturnWorkspaceOutputs on, so an
            %   asynchronous run packs everything into one SimulationOutput object in the
            %   base workspace and the individual logging variables are never created.
            name = get_param(app.Model, 'ReturnWorkspaceOutputsName');
            if evalin('base', sprintf('exist(''%s'', ''var'')', name))
                app.Out = evalin('base', name);
                % WHERE THE RUN STOPPED, not the stop time asked for, which is Inf by default.
                tEnd = app.StopTimeField.Value;
                try
                    tEnd = app.Out.tout(end);
                catch
                end
                app.Source = sprintf('run to t = %.1f s', tEnd);
                setStatus(app, 'ok', ['finished - ' app.Source]);
                pushLog(app, NaN, 'run finished - the full plots and the report are on the Plots tab');
                finishRecording(app);
                updateLoadEnable(app);
                if app.TabGroup.SelectedTab == app.PlotTab
                    showRun(app);
                end
            else
                setStatus(app, 'error', ...
                    sprintf('the run ended but produced no ''%s'' to read', name));
            end
        end

        %% ------------------------------------------------------------- the state feed

        function resetState(app)
            %RESETSTATE Blank the state panel and its log for a fresh run.
            app.LastPhase = '';
            app.LogLines = {};
            app.LogStamps = [];
            app.StateLog.Value = {'nothing has happened yet.'};
            app.PhaseLamp.Color = lampColour('idle');
            app.PhaseLabel.Text = 'not running';
            app.PlannerValueLabel.Text = '-';
            app.BaseValueLabel.Text = '-';
            app.ArmValueLabel.Text = '-';
            app.GripValueLabel.Text = '-';
        end

        function updateState(app, s)
            %UPDATESTATE The headline, the four sub-states, and a line in the log on a change.
            if ~isfinite(s.plannerMode)
                return
            end

            sup = app.Cfg.sup;
            app.PlannerValueLabel.Text = enumName(sup.pl, s.plannerMode);
            app.BaseValueLabel.Text    = enumName(sup.base, s.baseState);
            app.ArmValueLabel.Text     = enumName(sup.arm, s.armMode);
            app.GripValueLabel.Text    = enumName(sup.grip, s.gripState);

            [msg, level] = phaseMessage(app, s);
            app.PhaseLabel.Text = msg;
            app.PhaseLamp.Color = lampColour(level);

            % ONE LINE PER CHANGE. At one poll a second a 200 s run would otherwise write
            % 200 lines saying the same thing, and the four or five moments that matter -
            % planning, moving, closing, done - would be invisible inside them.
            if ~strcmp(msg, app.LastPhase)
                app.LastPhase = msg;
                pushLog(app, s.t, msg);
            end
        end

        function [msg, level] = phaseMessage(app, s)
            %PHASEMESSAGE What the robot is doing, in words an operator would use.
            %   NAMED FROM THE VERB AS WELL AS THE MODE, which is the whole point. A drive
            %   home and a drive to a mug are both PL_NAV_PICK - the chart deliberately has
            %   no fourth planner mode, because adding one would mean a new state and a
            %   fourth copy of the navigation timeout and failure wiring for the sake of a
            %   label. The label belongs here, where TaskResult.ActiveVerb is available to
            %   distinguish them.
            sup = app.Cfg.sup;
            q   = app.Cfg.queue;

            % An e-stop outranks whatever phase the planner believes it is in, so it is
            % tested before anything else rather than folded in as one mode among many.
            if s.eStop
                msg = sprintf('EMERGENCY STOP - %s', enumName(sup.cause, s.eStopCause));
                level = 'error';
                return
            end

            switch s.outcome
                case sup.out.SUCCESS
                    msg = 'task complete';
                    level = 'ok';
                    return
                case sup.out.PARTIAL
                    msg = 'finished, but not all of it';
                    level = 'error';
                    return
                case sup.out.FAILED
                    msg = 'task failed';
                    level = 'error';
                    return
                case sup.out.ABORTED
                    msg = 'task abandoned';
                    level = 'error';
                    return
            end

            switch s.plannerMode
                case sup.pl.IDLE
                    head = 'waiting for an instruction';
                case sup.pl.GROUND
                    head = 'asking Gemini ER where things are';
                case sup.pl.DISPATCH
                    head = 'reading the plan';
                case sup.pl.NAV_TO_PICK
                    % The one place the verb changes the words rather than the detail.
                    if s.activeVerb == q.VERB_HOME
                        head = 'driving home';
                    elseif s.activeVerb == q.VERB_NAVIGATE
                        head = 'driving to the place you named';
                    else
                        head = 'driving to the object';
                    end
                case sup.pl.PICK
                    head = 'picking the object up';
                case sup.pl.NAV_TO_PLACE
                    head = 'carrying it to the destination';
                case sup.pl.PLACE
                    head = 'putting it down';
                case sup.pl.RECOVER
                    head = 'recovering';
                case sup.pl.DONE
                    head = 'done';
                case sup.pl.FAILED
                    head = 'failed';
                case sup.pl.STOPPED
                    head = 'stopped';
                otherwise
                    head = sprintf('planner mode %g', s.plannerMode);
            end

            % The detail is the "moving, planning" half of the message, and it is taken from
            % whichever subsystem is the one actually doing something. The tests run base
            % first, then arm, then gripper, so the LAST one that matches wins - which is the
            % right order, because the gripper only leaves OPEN during a grasp and the arm
            % only leaves IDLE once the base has stopped needing to be talked about.
            detail = '';
            switch s.baseState
                case sup.base.AWAIT_PLAN
                    detail = 'planning a path';
                case sup.base.FOLLOWING
                    detail = 'moving';
                case sup.base.RECOVERING
                    detail = 'backing out and trying again';
            end
            switch s.armMode
                case sup.arm.AWAIT_PLAN
                    detail = 'planning an arm motion';
                case sup.arm.MOVING_FREE
                    detail = 'moving the arm';
                case sup.arm.APPROACH
                    detail = 'reaching in';
                case sup.arm.RETREATING
                    detail = 'backing the arm out';
                case sup.arm.STOWING
                    detail = 'stowing the arm';
            end
            switch s.gripState
                case sup.grip.CLOSING
                    detail = 'closing the gripper';
                case sup.grip.OPENING
                    detail = 'opening the gripper';
                case sup.grip.FAULTED
                    % SEE THE NOTE BESIDE s.gripFail. Faulted is reached both by a real
                    % dropped grasp and by the task planner cancelling the gripper, and only
                    % the first is the gripper's doing. A cancelled gripper is reported as
                    % what it is - the task standing down - because telling the operator the
                    % hardware failed sends them to the wrong subsystem, which is exactly
                    % what happened on 2026-10-04.
                    if s.gripSlipped || (isfinite(s.gripFail) && s.gripFail ~= 0)
                        detail = 'the gripper has faulted';
                    else
                        detail = 'the gripper was stood down by the task';
                    end
            end

            % ON RECOVER AND FAILED THE CAUSE OUTRANKS THE SUBSYSTEM DETAIL. Recover cancels
            % every subordinate, so the detail above is the cancellation, never the reason.
            if any(s.plannerMode == [sup.pl.RECOVER, sup.pl.FAILED]) && ...
                    isfinite(s.taskFail) && s.taskFail ~= 0
                detail = failureText(app, s.taskFail);
            end

            if isempty(detail)
                msg = head;
            else
                msg = sprintf('%s - %s', head, detail);
            end

            level = 'busy';
            switch s.plannerMode
                case sup.pl.IDLE
                    level = 'idle';
                case sup.pl.DONE
                    level = 'ok';
                case {sup.pl.FAILED, sup.pl.STOPPED}
                    level = 'error';
            end
        end

        function txt = failureText(app, code)
            %FAILURETEXT A cfg.fail code in the operator's words, the code name beside it.
            %   The common causes get a sentence; anything else falls back to the code's own
            %   name in cfg.fail, so a new code is never shown as a bare number.
            f = app.Cfg.fail;
            names = fieldnames(f);
            name = sprintf('code %d', code);
            for k = 1:numel(names)
                v = f.(names{k});
                if isnumeric(v) && isscalar(v) && double(v) == code
                    name = names{k};
                    break
                end
            end
            switch name
                case 'PlaceUnreachable'
                    why = 'no base pose can reach the place point';
                case 'Unreachable'
                    why = 'the object is out of the arm''s reach';
                case {'BaseGoalOccupied', 'BaseGoalUnreachable', 'BaseStartOccupied'}
                    why = 'the base cannot get to or stand at its goal';
                case {'BaseSearchFailed', 'PathPlanFailed'}
                    why = 'no base path was found';
                case {'ArmPlanFailed', 'ArmRrtFailed', 'PregraspUnreachable'}
                    why = 'no arm motion was found';
                case 'NotGraspable'
                    why = 'the grasp did not hold';
                case 'ProviderTimeout'
                    why = 'Gemini ER did not answer in time';
                case {'ProviderError', 'ProviderNoKey'}
                    why = 'Gemini ER could not be asked';
                case 'ReplayMiss'
                    why = 'there is no sample response for this instruction';
                case {'LowConfidence', 'AmbiguousObject', 'OutsideRegion', 'DepthInvalid'}
                    why = 'the object or place could not be located';
                otherwise
                    why = 'the task failed';
            end
            txt = sprintf('%s (%s)', why, name);
        end

        function pushLog(app, stampSeconds, text)
            %PUSHLOG One line onto the running account, oldest first.
            %   STAMPED IN MODEL TIME, NOT WALL TIME. The run is about 25 times slower than
            %   real time, so a wall clock would put "closing the gripper" minutes away from
            %   the t = 14 s the plots and the report both name it at.
            %
            %   AND INSERTED BY STAMP, NOT APPENDED, BECAUSE THE TWO SOURCES ARRIVE OUT OF
            %   ORDER. The phase lines come from the SDI poll and the provider lines from the
            %   grounding tap, and the tap is read AFTER the poll within the same timer tick -
            %   so a request answered at t = 1.62 s lands underneath a phase change the poll
            %   already saw at t = 1.68 s. The measured live run read:
            %
            %       1.62 s  asking Gemini ER where things are
            %       1.68 s  driving home - planning a path
            %       1.62 s  Gemini ER answered with 1 action(s)
            %
            %   which says the robot set off before the answer that sent it arrived. Both
            %   stamps were right; only the order was wrong, and a causal account that reads
            %   backwards is worse than no account.
            if isfinite(stampSeconds)
                stamp = sprintf('%7.2f s', stampSeconds);
            else
                stamp = '         ';
            end
            line = sprintf('%s  %s', stamp, text);

            % Backwards over the lines this one predates. A NaN stamp STOPS the walk: the
            % unstamped lines are the run's own boundaries ("run started", "run finished")
            % and nothing may be inserted above one, however early its model time.
            at = numel(app.LogLines);
            if isfinite(stampSeconds)
                while at > 0 && isfinite(app.LogStamps(at)) && ...
                        app.LogStamps(at) > stampSeconds
                    at = at - 1;
                end
            end
            app.LogLines  = [app.LogLines(1:at)  {line}       app.LogLines(at+1:end)];
            app.LogStamps = [app.LogStamps(1:at) stampSeconds app.LogStamps(at+1:end)];

            if numel(app.LogLines) > 400
                app.LogLines  = app.LogLines(end-399:end);
                app.LogStamps = app.LogStamps(end-399:end);
            end
            app.StateLog.Value = app.LogLines(:);
            try
                % Newest line kept in view. Guarded because scroll is the kind of
                % convenience that is not worth an error dialog if a release lacks it.
                scroll(app.StateLog, 'bottom');
            catch
            end
        end

        %% ---------------------------------------------------------------- the map

        function initMap(app)
            %INITMAP Draw the apartment once, and keep handles to everything that moves.
            %   BUILT ONCE AND UPDATED, NOT REDRAWN. A cla and a fresh image every second
            %   flickers, throws away the axes limits, and spends real time re-rasterising
            %   57,600 cells that never change. So the static scene is drawn here and the
            %   six things that move - the trail, the chassis, its nose spur, the two target
            %   markers and the planned route - are created empty and filled by updateMap.
            ax = app.MapAxes;
            cla(ax, 'reset');
            app.MapReady = false;
            % The handles are about to be replaced, so the leg the old ones were showing
            % is gone with them. Left stale, LegStartTime would hold a time from the
            % previous run and the new run's trail would be drawn from the middle.
            app.PlanGoal = [];
            app.LegStartTime = -Inf;

            if isempty(app.NavMap)
                axis(ax, 'off');
                text(ax, 0.5, 0.5, {'data/hmmNavMap.mat is missing,', ...
                    'so there is no map to draw.'}, 'HorizontalAlignment', 'center', ...
                    'FontAngle', 'italic', 'Color', [0.35 0.35 0.35]);
                return
            end

            m = app.NavMap;

            % OCCUPANCY AS RGB RATHER THAN THROUGH A COLORMAP. A colormap is a property of
            % the AXES, so it would also recolour the trail, the chassis and the markers
            % drawn on top - and those need to be told apart by colour. Building the grey
            % directly leaves the colormap free for nothing to depend on.
            %
            % uint8 rather than double: a 240 by 240 by 3 double CData is 1.3 MB for a
            % picture with two colours in it.
            occ = m.grid >= m.thr;
            shade = uint8(255 * (0.97 - 0.70 * double(occ)));
            rgb = repmat(shade, 1, 1, 3);

            app.MapH = struct();
            % YData RUNS DOWNWARD, BECAUSE ROW 1 OF AN occupancyMatrix IS THE LARGEST y.
            % This read [m.yc(1) m.yc(end)] and so put row 1 at the BOTTOM of the axes, which
            % mirrored the apartment about y = -3 and drew the kitchen where the bedroom is -
            % measured, not theorised: see loadNavMap. Every marker on top of this image is in
            % world metres and was always right, which is what made the flip survive so long:
            % the chassis really was at [5.20 -4.00] on a map whose walls were not.
            app.MapH.img = image(ax, 'XData', [m.xc(1) m.xc(end)], ...
                'YData', [m.yc(end) m.yc(1)], 'CData', rgb);
            hold(ax, 'on');

            % REVERSED, SO THE MAP LOOKS LIKE THE SIM3D WINDOW. World y is drawn downward:
            % Unreal's frame is left-handed (its Y is the world's -Y), and with y upward the
            % map read as the scene's mirror image to anyone comparing the two (2026-10-15).
            % Only the display flips - the image's YData, every marker and every route stay
            % in world metres, so the y labels simply count downward.
            ax.YDir = 'reverse';
            axis(ax, 'equal');
            xlim(ax, [m.xc(1) m.xc(end)]);
            ylim(ax, [m.yc(1) m.yc(end)]);
            xlabel(ax, 'x (m)');
            ylabel(ax, 'y (m)');

            % The four fixed cameras where they actually stand. Without them there is no way
            % to see that a pick pixel was measured from across the room.
            mnt = app.Cfg.camera.mount;
            plot(ax, mnt(:, 1), mnt(:, 2), 's', 'MarkerSize', 7, ...
                'MarkerEdgeColor', [0.45 0.45 0.50], 'MarkerFaceColor', [0.85 0.85 0.90]);
            for k = 1:size(mnt, 1)
                text(ax, mnt(k, 1) + 0.14, mnt(k, 2), sprintf('cam %d', k), ...
                    'FontSize', 8, 'Color', [0.40 0.40 0.45]);
            end

            % HOME IS DRAWN WITH ITS HEADING, because the heading is the whole content of
            % the home command: the position is where the robot already starts.
            hp = app.Cfg.task.homePose;
            plot(ax, hp(1), hp(2), 'p', 'MarkerSize', 14, ...
                'MarkerEdgeColor', [0.20 0.45 0.18], 'MarkerFaceColor', [0.60 0.85 0.50]);
            plot(ax, hp(1) + [0 0.55 * cos(hp(3))], hp(2) + [0 0.55 * sin(hp(3))], ...
                '-', 'Color', [0.20 0.45 0.18], 'LineWidth', 1.6);

            % THE PLANNED ROUTE, UNDER THE TRAVELLED ONE AND IN A DIFFERENT COLOUR. Drawn
            % first so the blue trail lies ON TOP of it: the interesting picture is where
            % the drive departs from the plan, and that is only visible if the thing being
            % compared against is the one underneath. Magenta because every other line on
            % this axes is already spoken for - blue is the travelled path, orange the
            % chassis, green home - and dashed because a reference is not a measurement.
            %
            % THE WAYPOINTS ARE DRAWN AS WELL, and they are not the same curve. Waypoints
            % are what BiRRT returned; RefPose is what contopptraj made of them under the
            % velocity and acceleration limits, which is what the follower actually tracks.
            % A corner that the smoothed trajectory cuts is the normal case, not a defect,
            % and showing only one of the two makes that unreadable.
            %
            % AND IT IS WIDER THAN THE TRAIL BECAUSE THE FOLLOWER IS GOOD. Drawn at the
            % trail's own width it was invisible: a full transfer measured 17 mm of peak
            % cross-track error, so the blue line covered the magenta one along almost its
            % whole length and only the waypoint rings showed. At 3.5 against the trail's
            % 1.6 the plan reads as a magenta band flanking the blue, which keeps the
            % under-the-trail ordering and its reason while actually being visible. If the
            % drive ever does depart from the plan, the two separate and both are legible.
            app.MapH.plan = plot(ax, NaN, NaN, '--', ...
                'Color', [0.72 0.20 0.72], 'LineWidth', 3.5);
            app.MapH.waypoints = plot(ax, NaN, NaN, 'o', ...
                'MarkerSize', 4, 'MarkerEdgeColor', [0.72 0.20 0.72], ...
                'MarkerFaceColor', [0.95 0.80 0.95], 'LineStyle', 'none');

            app.MapH.trail = plot(ax, NaN, NaN, '-', ...
                'Color', [0.20 0.55 0.95], 'LineWidth', 1.6);
            app.MapH.chassis = plot(ax, NaN, NaN, '-', ...
                'Color', [0.90 0.40 0.10], 'LineWidth', 2.2);
            app.MapH.nose = plot(ax, NaN, NaN, '-', ...
                'Color', [0.90 0.40 0.10], 'LineWidth', 2.2);
            app.MapH.pick = plot(ax, NaN, NaN, '+', ...
                'Color', [0.95 0.35 0.15], 'MarkerSize', 15, 'LineWidth', 2.2);
            app.MapH.place = plot(ax, NaN, NaN, 'x', ...
                'Color', [0.20 0.55 0.95], 'MarkerSize', 15, 'LineWidth', 2.2);

            hold(ax, 'off');
            app.MapReady = true;

            % Drawn at the starting pose so the tab is worth opening before anything runs.
            setChassis(app, app.Cfg.base.initialPose);
            title(ax, 'not running - the robot is at its starting pose');
        end

        function setChassis(app, pose)
            %SETCHASSIS The base as a rectangle of its real footprint, turned to its yaw.
            %   THE FOOTPRINT IS cfg.base.chassisSize AND NOT A DECORATIVE MARKER. A 0.53 by
            %   0.35 m box against 0.70 m of wall clearance is the difference between a home
            %   pose that fits and one that does not, and a fixed-size dot would hide that
            %   entirely.
            if ~app.MapReady
                return
            end
            pose = pose(:);
            L = app.Cfg.base.chassisSize(1);
            W = app.Cfg.base.chassisSize(2);

            corners = [ L/2  L/2 -L/2 -L/2  L/2
                       -W/2  W/2  W/2 -W/2 -W/2];
            c = cos(pose(3));
            sn = sin(pose(3));
            R = [c -sn; sn c];

            body = R * corners + pose(1:2);
            set(app.MapH.chassis, 'XData', body(1, :), 'YData', body(2, :));

            % A spur out of the front face. Without it a base that has turned through pi is
            % drawn identically to one that has not, which is precisely the distinction the
            % home command exists to make.
            spur = R * [L/2 L/2 + 0.24; 0 0] + pose(1:2);
            set(app.MapH.nose, 'XData', spur(1, :), 'YData', spur(2, :));
        end

        function updateMap(app, s)
            %UPDATEMAP Move the robot, extend the trail, and place the targets.
            if ~app.MapReady
                return
            end
            if ~s.ok
                return
            end

            % The planned route first, because it decides where this leg began and the
            % trail below is drawn from there.
            updatePlan(app);

            % TRUNCATED TO THE SHORTER OF THE TWO. x and y are separate SDI channels and one
            % can be a sample ahead of the other, which plot would reject outright.
            n = min(numel(s.trailX), numel(s.trailY));
            if n > 0
                % THE CURRENT LEG ONLY. Before any plan arrives LegStartTime is -Inf, so
                % this is the whole history and the behaviour is what it always was; from
                % the first plan onward it is the drive since the goal last moved. The
                % cut is by TIME rather than by the sample count at the moment the plan
                % was noticed, because the poll runs at 1 Hz and a 0.5 m/s base covers
                % half a metre between ticks - which would leave half a metre of the
                % previous leg hanging off the start of the new one.
                first = 1;
                if isfinite(app.LegStartTime) && numel(s.trailT) >= n
                    k = find(s.trailT(1:n) >= app.LegStartTime, 1, 'first');
                    if ~isempty(k)
                        first = k;
                    end
                end
                set(app.MapH.trail, 'XData', s.trailX(first:n), 'YData', s.trailY(first:n));
            end

            setChassis(app, [s.x; s.y; s.yaw]);
            setMarker(app, app.MapH.pick, s.pickXY);
            setMarker(app, app.MapH.place, s.placeXY);

            yawDeg = rad2deg(atan2(sin(s.yaw), cos(s.yaw)));
            if isfinite(s.distToGoal)
                title(app.MapAxes, sprintf(...
                    't = %.1f s    base [%.2f %.2f] m, heading %+.0f deg    %.2f m to go', ...
                    s.t, s.x, s.y, yawDeg, s.distToGoal));
            else
                title(app.MapAxes, sprintf('t = %.1f s    base [%.2f %.2f] m, heading %+.0f deg', ...
                    s.t, s.x, s.y, yawDeg));
            end
        end

        function updatePlan(app)
            %UPDATEPLAN The route the planner returned, and where the current leg began.
            %   READ FROM hmmBasePlanTap, which hmmPlanBasePath writes on its success path.
            %   This is the plan the follower was handed, not a re-plan: a fresh BiRRT
            %   search here would return a different random tree and draw a confident wrong
            %   line beside the one the robot drove.
            if ~app.MapReady
                return
            end
            if ~isappdata(0, 'hmmBasePlanTap')
                return
            end
            tap = getappdata(0, 'hmmBasePlanTap');
            if ~isstruct(tap) || ~isfield(tap, 'refPose') || ~isfield(tap, 'goalPose')
                return
            end

            % A NEW LEG IS A NEW GOAL, NOT A NEW PLAN. Both arrive as a fresh request id
            % with a fresh tree, and only the goal tells them apart. The tolerance is one
            % map cell: the goal is selected on the stand-off ring in continuous metres and
            % a retry at "the same" goal can differ in the last bits, which an == test
            % would read as a transfer and wipe the trail mid-leg.
            % cellSize AND NOT cell: a local named cell shadows the builtin for the rest
            % of the method, and this project has lost time twice to a shadowed function
            % name reading as an index.
            cellSize = 1 / app.Cfg.map.resolution;
            goal = tap.goalPose(:)';
            moved = isempty(app.PlanGoal) || numel(app.PlanGoal) ~= numel(goal) || ...
                any(abs(goal(1:2) - app.PlanGoal(1:2)) > cellSize);
            if moved
                app.PlanGoal = goal;
                app.LegStartTime = tap.simTime;
            elseif numel(goal) >= 3
                % SAME LEG, POSSIBLY A DIFFERENT HEADING. hmmPlanBasePath may send the base to
                % the stand-off's PARTNER heading - same position, roughly opposite yaw - so the
                % position test above cannot see it. The heading is always taken from the plan
                % being drawn, so the marker shows where the base will actually face.
                app.PlanGoal(3) = goal(3);
            end

            % THE SMOOTHED TRAJECTORY IS WHAT THE FOLLOWER TRACKS, so it is the line; the
            % waypoints are the search's own answer and are drawn as points beside it.
            % Both are already trimmed to what was measured - see hmmBasePlanTap for why
            % the fixed-width bus tail would otherwise draw a spur to the world origin.
            if size(tap.refPose, 1) >= 2 && ~isempty(tap.refPose)
                set(app.MapH.plan, 'XData', tap.refPose(1, :), 'YData', tap.refPose(2, :));
            else
                set(app.MapH.plan, 'XData', NaN, 'YData', NaN);
            end
            if isfield(tap, 'waypoints') && size(tap.waypoints, 1) >= 2 && ...
                    ~isempty(tap.waypoints)
                set(app.MapH.waypoints, 'XData', tap.waypoints(1, :), ...
                    'YData', tap.waypoints(2, :));
            else
                set(app.MapH.waypoints, 'XData', NaN, 'YData', NaN);
            end
        end

        function setMarker(~, h, xy)
            %SETMARKER A world marker, or nothing at all when there is nothing to mark.
            %   NaN rather than a hidden object: an invisible marker left at a stale position
            %   comes back on screen the moment anything toggles visibility, still holding
            %   coordinates from a target the robot has long since given up on.
            if isempty(xy)
                set(h, 'XData', NaN, 'YData', NaN);
            else
                set(h, 'XData', xy(1), 'YData', xy(2));
            end
        end

        %% ------------------------------------------------------------- the detections

        function showGrounding(app, live)
            %SHOWGROUNDING The frames the provider was shown, with its own marks on them.
            %   showGrounding(app, live) draws the live-view capture LIVE (an hmmLiveViewTap
            %   record) under the same marks instead - the cameras are fixed, so a pick pixel
            %   measured at grounding still lands on the same place in a later frame.
            %   THE FRAMES IT CHOSE, NOT FRESH ONES. A pick pixel only means something in the
            %   image it was measured in, and by the time it is drawn the base may have driven
            %   past that view entirely. hmmGroundingTap therefore keeps the images as they
            %   were sent and this draws on those.
            tap = getappdata(0, 'hmmGroundingTap');
            haveTap = ~isempty(tap) && ~isempty(tap.images);
            haveLive = nargin > 1 && isstruct(live) && isfield(live, 'images');

            for camIdx = 1:numel(app.CamAxes)
                ax = app.CamAxes{camIdx};
                cla(ax, 'reset');
                ax.XTick = [];
                ax.YTick = [];
                ax.Box = 'on';

                if ~haveTap && ~haveLive
                    axis(ax, 'off');
                    text(ax, 0.5, 0.5, sprintf('camera %d', camIdx), ...
                        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                        'FontAngle', 'italic', 'Color', [0.55 0.55 0.55]);
                    continue
                end

                if haveLive && camIdx <= numel(live.images)
                    img = live.images{camIdx};
                    slot = camIdx;
                else
                    [img, slot] = frameFor(tap, camIdx);
                end
                if slot == 0
                    % NOT SOMEBODY ELSE'S PICTURE. Only admissible frames are sent, so a
                    % stale camera drops out of the request; showing another camera's frame
                    % here would imply the markers on it were measured in this view.
                    axis(ax, 'off');
                    text(ax, 0.5, 0.5, {sprintf('camera %d', camIdx), 'was not sent'}, ...
                        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                        'FontAngle', 'italic', 'Color', [0.55 0.55 0.55]);
                    continue
                end

                % The frame and Gemini ER's marks, drawn by the same function the live
                % script's grounding view uses.
                acts = [];
                if haveTap
                    acts = tap.actions;
                end
                nMark = hmmShowGrounding(ax, img, acts, camIdx, app.Cfg.camera.pixelNormScale);

                if haveLive
                    title(ax, sprintf('camera %d - live, t = %.1f s', camIdx, live.simTime));
                elseif nMark == 0
                    title(ax, sprintf('camera %d - sent, nothing marked here', camIdx));
                else
                    title(ax, sprintf('camera %d', camIdx));
                end
            end

            if ~haveTap && haveLive
                app.CameraBlurb.Text = sprintf(['Live views, t = %.1f s. Nothing grounded ' ...
                    'yet - type an instruction and press Send to Gemini ER; its pick and ' ...
                    'place marks will then stay on these views.'], live.simTime);
                return
            end
            if ~haveTap
                if isRunning(app)
                    app.CameraBlurb.Text = ['Nothing grounded yet. The apartment is in the ' ...
                        'Sim3D window and the robot is waiting at its home pose - type an ' ...
                        'instruction and press Send to Gemini ER.'];
                else
                    app.CameraBlurb.Text = ['Nothing grounded yet. Press Start simulation, ' ...
                        'wait for the apartment to appear, then type an instruction and ' ...
                        'press Send to Gemini ER.'];
                end
                return
            end

            blurb = sprintf(['Frames %s, sent at t = %.2f s. A marks where Gemini ER says to ' ...
                'pick, B where to place, and each box is its own box_2d for that object - ' ...
                'all in its normalised pixels mapped back onto the frame it measured them ' ...
                'in. Nothing here is a world coordinate.'], ...
                mat2str(tap.indices), tap.simTime);

            % THE WRIST CAMERA HAS NO PANE, and saying so beats leaving a marker nowhere.
            % cfg.camera.nTotal is five: four fixed views and one on the wrist, which moves
            % with the arm and so is not one of a fixed 2x2 grid of rooms. If the provider
            % put a point in it, the plan text below still records which image.
            if any(tap.indices > numel(app.CamAxes))
                blurb = [blurb sprintf([' Camera %s is the wrist view, which has no pane ' ...
                    'here - anything marked in it is listed in the plan below instead.'], ...
                    mat2str(tap.indices(tap.indices > numel(app.CamAxes))))];
            end
            app.CameraBlurb.Text = blurb;
        end

        function onRefineEvent(app, ev, rec)
            %ONREFINEEVENT The wrist camera's second look, drawn as it happens.
            %   Called by hmmRefineNotify from inside the refinement. 'request' puts the wrist
            %   frame over the pane of the ceiling camera that grounded the pick - the same
            %   object, seen from the hand - and logs the request; 'answer' marks where Gemini
            %   ER pointed and logs what came back. onPoll restores the pane when the arm
            %   starts reaching in.
            if ~isvalid(app) || ~isvalid(app.UIFigure)
                return
            end
            switch ev
                case 'request'
                    pane = 1;
                    tap = getappdata(0, 'hmmGroundingTap');
                    if ~isempty(tap) && isfield(tap, 'actions') && ~isempty(tap.actions)
                        k = find([tap.actions.pickImage] >= 1 & ...
                            [tap.actions.pickImage] <= numel(app.CamAxes), 1);
                        if ~isempty(k)
                            pane = double(tap.actions(k).pickImage);
                        end
                    end
                    app.WristPane = pane;
                    ax = app.CamAxes{pane};
                    cla(ax, 'reset');
                    image(ax, uint8(rec.RGB));
                    ax.YDir = 'reverse';
                    axis(ax, 'image');
                    ax.XTick = [];
                    ax.YTick = [];
                    title(ax, 'wrist camera - asking Gemini ER to refine the grasp', ...
                        'Color', [0.85 0.45 0.05]);
                    recordWrist(app, rec, pane);
                    pushLog(app, rec.SimTime, ['asking Gemini ER to refine the grasp ' ...
                        '(wrist camera)']);

                case 'answer'
                    refine = app.Cfg.refine;
                    status = double(rec.Status);
                    name = enumName(refine, status);
                    if app.WristPane > 0
                        ax = app.CamAxes{app.WristPane};
                        hold(ax, 'on');
                        px = double(rec.Pixel(:));
                        haveProviderPoint = numel(px) == 2 && any(px) && ...
                            status == refine.STATUS_APPLIED;
                        if haveProviderPoint
                            plot(ax, px(2), px(1), '+', 'Color', [0.95 0.35 0.15], ...
                                'MarkerSize', 18, 'LineWidth', 2.4);
                            plot(ax, px(2), px(1), 'o', 'Color', [0.95 0.35 0.15], ...
                                'MarkerSize', 26, 'LineWidth', 1.6);
                            title(ax, sprintf(['wrist camera - Gemini ER: grasp here ' ...
                                '(conf %.2f, %.0f mm shift)'], rec.Confidence, 1000 * rec.Shift), ...
                                'Color', [0.95 0.35 0.15]);
                        elseif status == refine.STATUS_DEPTHONLY
                            plot(ax, px(2), px(1), 'x', 'Color', [0.20 0.55 0.95], ...
                                'MarkerSize', 16, 'LineWidth', 2.2);
                            title(ax, ['wrist camera - Gemini ER unavailable, grasp ' ...
                                'measured from depth'], 'Color', [0.20 0.55 0.95]);
                        else
                            title(ax, sprintf('wrist camera - refinement not applied (%s)', ...
                                name), 'Color', [0.80 0.10 0.10]);
                        end
                        hold(ax, 'off');
                    end
                    if status == refine.STATUS_APPLIED
                        msg = sprintf(['Gemini ER refined the grasp: point [%.0f %.0f] px, ' ...
                            'conf %.2f, moved %.0f mm (%.2f s)'], rec.Pixel(1), rec.Pixel(2), ...
                            rec.Confidence, 1000 * rec.Shift, rec.Latency);
                    elseif status == refine.STATUS_DEPTHONLY
                        msg = sprintf(['Gemini ER refinement unavailable (http %d, %.1f s) - ' ...
                            'grasp measured from the wrist depth, moved %.0f mm'], ...
                            rec.HttpStatus, rec.Latency, 1000 * rec.Shift);
                    else
                        msg = sprintf(['Gemini ER refinement not applied: %s (http %d) - ' ...
                            'grasping at the coarse point'], name, rec.HttpStatus);
                    end
                    pushLog(app, rec.SimTime, msg);
                    reportRefinement(app, rec, status, name);
            end
        end

        function reportRefinement(app, rec, status, name)
            %REPORTREFINEMENT Gemini ER's wrist answer, under the grounding answer it refines.
            %   The camera tab's pane already shows the grounding reply; the second look is
            %   Gemini's answer about the image above it too, and it used to reach only the
            %   state log. A failed request says WHICH failure, because "unavailable" hid
            %   that four runs in a row were 503 and 429 - Google's capacity and rate limits,
            %   not our request (2026-10-16).
            refine = app.Cfg.refine;
            lines = {''; sprintf('GEMINI ER REFINE  t = %.2f s   wrist camera', rec.SimTime)};
            lines{end+1} = sprintf('  http status  %d %s     latency %.2f s', rec.HttpStatus, ...
                httpMeaning(rec.HttpStatus), rec.Latency);
            px = double(rec.Pixel(:));
            switch status
                case refine.STATUS_APPLIED
                    lines{end+1} = sprintf(['  answer       grasp at [%.0f %.0f] px, confidence ' ...
                        '%.2f - grasp moved %.0f mm'], px(1), px(2), rec.Confidence, 1000 * rec.Shift);
                case refine.STATUS_DEPTHONLY
                    lines{end+1} = sprintf(['  answer       none - the grasp was measured from ' ...
                        'the wrist depth instead, moved %.0f mm'], 1000 * rec.Shift);
                    lines{end+1} = ['               (the blue x is that fallback, seeded from the ' ...
                        'room cameras'' estimate)'];
                otherwise
                    lines{end+1} = sprintf(['  answer       not applied (%s) - grasping at the ' ...
                        'coarse point'], name);
            end
            app.PlanArea.Value = [app.PlanArea.Value(:); lines(:)];
            scroll(app.PlanArea, 'bottom');
        end

        function reportGrounding(app, tap)
            %REPORTGROUNDING The plan, and the route it came by, as text.
            %   WRITTEN TO THE CAMERA TAB'S OWN PANE, beside the frames it describes, rather
            %   than to the run report on the Plots tab. The two are answers to different
            %   questions and one used to overwrite the other.
            lines = {};
            lines{end+1} = sprintf('GEMINI ER  t = %.2f s   request %d   task %d', ...
                tap.simTime, tap.requestId, tap.taskId);
            lines{end+1} = sprintf('  instruction  "%s"', tap.instruction);
            lines{end+1} = sprintf('  route        %s', routeName(app, tap.route));
            lines{end+1} = sprintf('  http status  %d      latency %.2f s', ...
                tap.httpStatus, tap.latency);
            lines{end+1} = sprintf('  outcome      %s', ...
                enumName(app.Cfg.sup.st, tap.state));
            if ~strcmp(tap.failure, 'ok')
                lines{end+1} = sprintf('  FAILURE      %s', tap.failure);
            end
            if tap.overflow
                lines{end+1} = '  OVERFLOW     more actions were returned than the queue holds';
            end

            % WHAT THE PROVIDER ACTUALLY SAID, WHEN WHAT IT SAID IS THE PROBLEM. A
            % malformed answer is the one failure an operator cannot act on from its
            % code alone: the same code covers "the envelope carried no text at all"
            % and "text arrived and did not decode". The tap distinguishes them by
            % whether the answer is empty, so this does too. On a good answer the text
            % is redundant with the plan below and is not shown.
            if isfield(tap, 'answer') && ~strcmp(tap.failure, 'ok')
                lines{end+1} = '';
                if isempty(tap.answer)
                    lines{end+1} = ['  REPLY        the envelope carried no text part - ' ...
                        'nothing was parsed'];
                else
                    lines{end+1} = sprintf('  REPLY        %d chars, as it arrived:', ...
                        tap.answerChars);
                    for s = string(splitlines(tap.answer))'
                        lines{end+1} = ['    | ' char(s)];        %#ok<AGROW>
                    end
                end
            end
            lines{end+1} = '';

            if isempty(tap.actions)
                lines{end+1} = 'PLAN  none - no action was returned or none survived the gates.';
            else
                lines{end+1} = 'PLAN';
                for k = 1:numel(tap.actions)
                    a = tap.actions(k);
                    switch a.verb
                        case 'transfer'
                            lines{end+1} = sprintf(...
                                '  %d %-8s "%s"  A [%5.1f %5.1f] img %d   B [%5.1f %5.1f] img %d  conf %.2f  %s', ...
                                k, a.verb, a.label, a.pickPoint, a.pickImage, ...
                                a.placePoint, a.placeImage, a.confidence, a.execState); %#ok<AGROW>
                        case 'home'
                            % NO POINT AND NO IMAGE TO PRINT, and saying so is the honest
                            % report: home is grounded from cfg.task.homePose, so there is
                            % no pixel behind it to show and none should be implied.
                            lines{end+1} = sprintf(...
                                '  %d %-8s  no point, no image - the pose comes from cfg.task.homePose  conf %.2f  %s', ...
                                k, a.verb, a.confidence, a.execState);   %#ok<AGROW>
                        otherwise
                            lines{end+1} = sprintf(...
                                '  %d %-8s          A [%5.1f %5.1f] img %d   conf %.2f  %s', ...
                                k, a.verb, a.pickPoint, a.pickImage, ...
                                a.confidence, a.execState);              %#ok<AGROW>
                    end
                    pb = boxOf(a, 'pickBox');
                    if numel(pb) == 4
                        lines{end+1} = sprintf('      box A  [%4.0f %4.0f %4.0f %4.0f]  (ymin xmin ymax xmax, 0-%d)', ...
                            pb, app.Cfg.camera.pixelNormScale);          %#ok<AGROW>
                    end
                    qb = boxOf(a, 'placeBox');
                    if numel(qb) == 4
                        lines{end+1} = sprintf('      box B  [%4.0f %4.0f %4.0f %4.0f]', qb); %#ok<AGROW>
                    end
                    if ~strcmp(a.failure, 'ok')
                        lines{end+1} = sprintf('      refused by the gates: %s', a.failure); %#ok<AGROW>
                    end
                end
            end

            app.PlanArea.Value = lines(:);

            if tap.state == app.Cfg.sup.st.DONE
                setRoute(app, 'ok', sprintf('grounded, route %s, %d action(s)', ...
                    routeName(app, tap.route), numel(tap.actions)));
                pushLog(app, tap.simTime, sprintf('Gemini ER answered with %d action(s)', ...
                    numel(tap.actions)));
            else
                setRoute(app, 'error', sprintf('%s, route %s', ...
                    tap.failure, routeName(app, tap.route)));
                pushLog(app, tap.simTime, sprintf('Gemini ER failed: %s', tap.failure));
            end
        end

        %% -------------------------------------------------------- the finished run

        function onLoad(app, ~)
            %ONLOAD Pick one of this app's recordings and put it on screen with the replay bar.
            %   IT IS NOT LABELLED AS THIS SESSION'S. The status line names the recording, because
            %   a plot of another run presented without provenance is worse than an empty axes.
            % THE RUNS THIS APP RECORDED, from records/ - not a single shipped reference file,
            % which showed a robot several releases old (2026-10-14). Newest first, named by when
            % and what. Loading puts the replay bar under the tabs.
            if isRunning(app)
                setStatus(app, 'busy', 'stop the run before loading a recording');
                return
            end
            list = listRecordings(app);
            if isempty(list)
                setStatus(app, 'idle', 'no recordings yet - tick Record run and start a run');
                updateLoadEnable(app);
                return
            end
            [k, ok] = listdlg('ListString', {list.label}, 'SelectionMode', 'single', ...
                'PromptString', 'Recorded runs (newest first):', 'Name', 'Load recorded run', ...
                'ListSize', [460 260]);
            if ~ok
                return
            end
            try
                loadRecording(app, list(k).dir);
                setStatus(app, 'ok', ['showing the recording ' list(k).label]);
            catch ME
                setStatus(app, 'error', 'the recording would not load');
                app.ReportArea.Value = cellstr(splitlines(string(ME.getReport('basic'))));
            end
        end

        %% ------------------------------------------------------ recording and replay

        function d = recordsRoot(app)
            d = fullfile(app.Root, 'records');
        end

        function beginRecording(app)
            %BEGINRECORDING Open a folder for this run if Record run is ticked; else record nothing.
            app.RecDir = '';
            app.RecFrames = struct('t', {}, 'files', {});
            app.RecPlans = {};
            app.RecTaps = {};
            app.RecWrist = struct('t', {}, 'file', {}, 'pane', {});
            app.RecLastPlanT = -Inf;
            if ~app.RecordBox.Value
                return
            end
            d = fullfile(recordsRoot(app), char(datetime('now', 'Format', 'yyyy-MM-dd_HHmmss')));
            [ok, msg] = mkdir(d);
            if ~ok
                pushLog(app, NaN, ['not recording - ' msg]);
                return
            end
            app.RecDir = d;
            pushLog(app, NaN, ['recording to ' strrep(d, [app.Root filesep], '')]);
        end

        function recordTap(app, tap)
            %RECORDTAP One grounding transaction; its frames go to JPEG files beside it.
            if isempty(app.RecDir) || ~isstruct(tap)
                return
            end
            k = numel(app.RecTaps) + 1;
            files = cell(1, numel(tap.images));
            for i = 1:numel(tap.images)
                files{i} = sprintf('ground%02d_cam%d.jpg', k, i);
                try
                    imwrite(uint8(tap.images{i}), fullfile(app.RecDir, files{i}), 'Quality', 90);
                catch
                    files{i} = '';
                end
            end
            rec = rmfield(tap, 'images');
            rec.imageFiles = files;
            app.RecTaps{end + 1} = rec;
        end

        function recordPlan(app)
            %RECORDPLAN Each new base route, so the replay draws the line the robot followed.
            if isempty(app.RecDir) || ~isappdata(0, 'hmmBasePlanTap')
                return
            end
            p = getappdata(0, 'hmmBasePlanTap');
            if isstruct(p) && isfield(p, 'simTime') && p.simTime > app.RecLastPlanT
                app.RecLastPlanT = p.simTime;
                app.RecPlans{end + 1} = p;
            end
        end

        function recordFrame(app, lv)
            %RECORDFRAME One live camera capture, four JPEGs.
            if isempty(app.RecDir) || ~isfield(lv, 'images')
                return
            end
            k = numel(app.RecFrames) + 1;
            files = cell(1, numel(lv.images));
            for i = 1:numel(lv.images)
                files{i} = sprintf('live%04d_cam%d.jpg', k, i);
                try
                    imwrite(uint8(lv.images{i}), fullfile(app.RecDir, files{i}), 'Quality', 85);
                catch
                    files{i} = '';
                end
            end
            app.RecFrames(end + 1) = struct('t', double(lv.simTime), 'files', {files});
        end

        function recordWrist(app, rec, pane)
            %RECORDWRIST The wrist frame sent to Gemini for the refinement.
            if isempty(app.RecDir) || ~isfield(rec, 'RGB')
                return
            end
            f = sprintf('wrist%02d.jpg', numel(app.RecWrist) + 1);
            try
                imwrite(uint8(rec.RGB), fullfile(app.RecDir, f), 'Quality', 90);
                app.RecWrist(end + 1) = struct('t', double(rec.SimTime), 'file', f, 'pane', pane);
            catch
            end
        end

        function finishRecording(app)
            %FINISHRECORDING Write run.mat beside the images, then forget the folder.
            if isempty(app.RecDir) || isempty(app.Out)
                app.RecDir = '';
                return
            end
            out = app.Out;
            cfg = app.RunCfg;
            log = struct('lines', {app.LogLines}, 'stamps', app.LogStamps);
            plans = app.RecPlans;
            taps = app.RecTaps;
            frames = app.RecFrames;
            wrist = app.RecWrist;
            instr = '';
            if ~isempty(app.RecTaps) && isfield(app.RecTaps{1}, 'instruction')
                instr = app.RecTaps{1}.instruction;
            end
            tEnd = 0;
            try
                tEnd = out.tout(end);
            catch
            end
            meta = struct('when', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm')), ...
                'instruction', instr, 'tEnd', tEnd, 'nFrames', numel(app.RecFrames));
            d = app.RecDir;
            saved = false;
            try
                save(fullfile(d, 'run.mat'), 'meta', 'out', 'cfg', 'log', 'plans', ...
                    'taps', 'frames', 'wrist');
                pushLog(app, NaN, sprintf('recording saved (%d camera captures)', numel(app.RecFrames)));
                saved = true;
            catch ME
                pushLog(app, NaN, ['recording NOT saved - ' ME.message]);
            end
            app.RecDir = '';
            updateLoadEnable(app);
            % The run just recorded goes straight onto the replay bar, so it can be scrubbed
            % without a trip through Load recorded run.
            if saved
                try
                    loadRecording(app, d);
                catch
                end
            end
        end

        function list = listRecordings(app)
            %LISTRECORDINGS Folders under records/ holding a run.mat, newest first.
            list = struct('dir', {}, 'label', {}, 'datenum', {});
            r = recordsRoot(app);
            if ~isfolder(r)
                return
            end
            d = dir(r);
            d = d([d.isdir] & ~startsWith({d.name}, '.'));
            for k = 1:numel(d)
                f = fullfile(r, d(k).name, 'run.mat');
                if ~isfile(f)
                    continue
                end
                label = d(k).name;
                try
                    m = load(f, 'meta');
                    label = sprintf('%s   %.0f s   %s', m.meta.when, m.meta.tEnd, m.meta.instruction);
                    if m.meta.nFrames > 0
                        label = [label sprintf('   (%d camera captures)', m.meta.nFrames)];
                    end
                catch
                end
                list(end + 1) = struct('dir', fullfile(r, d(k).name), 'label', label, ...
                    'datenum', d(k).datenum); %#ok<AGROW>
            end
            [~, o] = sort([list.datenum], 'descend');
            list = list(o);
        end

        function updateLoadEnable(app)
            %UPDATELOADENABLE Load recorded run is greyed out until there is something to load.
            has = ~isempty(listRecordings(app));
            app.LoadButton.Enable = matlab.lang.OnOffSwitchState(has && ~isRunning(app));
            if has
                app.LoadButton.Tooltip = 'Load one of the runs recorded in records/ and replay it.';
            else
                app.LoadButton.Tooltip = 'No recordings yet: tick Record run and start a run.';
            end
        end

        function loadRecording(app, d)
            %LOADRECORDING Put a recording on display: plots, report, and the replay bar.
            S = load(fullfile(d, 'run.mat'));
            S.dir = d;
            app.Replay = S;
            app.Out = S.out;
            app.RunCfg = S.cfg;
            tEnd = S.meta.tEnd;
            if ~(tEnd > 0)
                tEnd = 1;
            end
            app.ReplaySlider.Limits = [0 tEnd];
            app.ReplaySlider.Value = tEnd;
            app.ResultLayout.RowHeight = {'1x', 34};
            app.ReplayPlay.Text = 'Play';
            showRun(app);
            replayAt(app, tEnd);
        end

        function stopReplay(app)
            %STOPREPLAY Leave replay: stop Play, hide the bar, forget the recording.
            if ~isempty(app.ReplayTimer) && isvalid(app.ReplayTimer)
                stop(app.ReplayTimer);
                delete(app.ReplayTimer);
            end
            app.ReplayTimer = [];
            app.Replay = [];
            app.ResultLayout.RowHeight = {'1x', 0};
            app.ReplayPlay.Text = 'Play';
        end

        function onReplayScrub(app, ev)
            if isempty(app.Replay)
                return
            end
            replayAt(app, ev.Value);
        end

        function onReplayPlay(app, ~)
            %ONREPLAYPLAY Play from the slider position, or pause.
            if isempty(app.Replay)
                return
            end
            if ~isempty(app.ReplayTimer) && isvalid(app.ReplayTimer) && ...
                    strcmp(app.ReplayTimer.Running, 'on')
                stop(app.ReplayTimer);
                app.ReplayPlay.Text = 'Play';
                return
            end
            if app.ReplaySlider.Value >= app.ReplaySlider.Limits(2) - 1e-6
                app.ReplaySlider.Value = 0;
            end
            if isempty(app.ReplayTimer) || ~isvalid(app.ReplayTimer)
                app.ReplayTimer = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.2, ...
                    'BusyMode', 'drop', 'Name', 'hmmReplayTimer', ...
                    'TimerFcn', @(~, ~) replayTick(app));
            end
            app.ReplayPlay.Text = 'Pause';
            start(app.ReplayTimer);
        end

        function replayTick(app)
            %REPLAYTICK Advance the replay by 0.2 s of wall clock times the chosen speed.
            try
                if ~isvalid(app) || isempty(app.Replay)
                    return
                end
                sp = str2double(erase(app.ReplaySpeed.Value, 'x'));
                t = min(app.ReplaySlider.Value + 0.2 * sp, app.ReplaySlider.Limits(2));
                app.ReplaySlider.Value = t;
                replayAt(app, t);
                if t >= app.ReplaySlider.Limits(2) - 1e-6
                    stop(app.ReplayTimer);
                    app.ReplayPlay.Text = 'Play';
                end
            catch
            end
        end

        function replayAt(app, t)
            %REPLAYAT Show the recording as it was at model time t, in every pane.
            %   THE SAME DRAWING CODE AS A LIVE RUN: a state struct like readLiveState's is
            %   sampled from the recording at t and handed to updateMap, the panel labels and
            %   showGrounding, so a replay looks exactly like the run did. The log is cut at t.
            R = app.Replay;
            if isempty(R)
                return
            end
            out = R.out;
            s = struct('ok', false, 't', t, 'x', NaN, 'y', NaN, 'yaw', NaN, ...
                'trailX', [], 'trailY', [], 'trailT', [], 'plannerMode', NaN, 'activeVerb', NaN, ...
                'outcome', NaN, 'baseState', NaN, 'armMode', NaN, 'gripState', NaN, ...
                'gripFail', NaN, 'gripSlipped', false, 'eStop', false, 'eStopCause', NaN, ...
                'pickXY', [], 'placeXY', [], 'elapsed', NaN, 'distToGoal', NaN, 'taskFail', NaN);
            [tb, B] = recLeaf(out, 'RobotState', 'BasePose');
            k = find(tb <= t, 1, 'last');
            if ~isempty(k)
                s.x = B(k, 1); s.y = B(k, 2); s.yaw = B(k, 3);
                s.trailX = B(1:k, 1); s.trailY = B(1:k, 2); s.trailT = tb(1:k);
            end
            s.plannerMode = recAt(out, 'TaskResult', 'PlannerMode', t);
            s.activeVerb  = recAt(out, 'TaskResult', 'ActiveVerb', t);
            s.outcome     = recAt(out, 'TaskResult', 'Outcome', t);
            s.taskFail    = recAt(out, 'TaskResult', 'FailureCode', t);
            s.baseState   = recAt(out, 'BaseStatus', 'Mode', t);
            s.distToGoal  = recAt(out, 'BaseStatus', 'DistanceToGoal', t);
            s.armMode     = recAt(out, 'ArmStatus', 'Mode', t);
            s.gripState   = recAt(out, 'GripperStatus', 'Mode', t);
            s.gripFail    = recAt(out, 'GripperStatus', 'FailureCode', t);
            s.gripSlipped = recAt(out, 'GripperStatus', 'Slipped', t) > 0;
            s.eStop       = recAt(out, 'SafetyStatus', 'EStop', t) > 0;
            if recAt(out, 'ObjectTargetRefined', 'Valid', t) > 0
                [tp, P] = recLeaf(out, 'ObjectTargetRefined', 'Position');
                j = find(tp <= t, 1, 'last');
                if ~isempty(j), s.pickXY = P(j, 1:2); end
            end
            if recAt(out, 'PlaceTarget', 'Valid', t) > 0
                [tp, P] = recLeaf(out, 'PlaceTarget', 'Point');
                j = find(tp <= t, 1, 'last');
                if ~isempty(j), s.placeXY = P(j, 1:2); end
            end
            s.ok = isfinite(s.x) && isfinite(s.y) && isfinite(s.yaw);

            % Panel labels and headline, without writing log lines (the recorded log is shown).
            if isfinite(s.plannerMode)
                sup = app.Cfg.sup;
                app.PlannerValueLabel.Text = enumName(sup.pl, s.plannerMode);
                app.BaseValueLabel.Text    = enumName(sup.base, s.baseState);
                app.ArmValueLabel.Text     = enumName(sup.arm, s.armMode);
                app.GripValueLabel.Text    = enumName(sup.grip, s.gripState);
                [msg, level] = phaseMessage(app, s);
                app.PhaseLabel.Text = msg;
                app.PhaseLamp.Color = lampColour(level);
            end
            L = R.log;
            keep = ~(L.stamps > t);                         % NaN stamps are run boundaries
            app.StateLog.Value = L.lines(keep);

            % The route active at t, through the same appdata the live map reads.
            p = [];
            for i = 1:numel(R.plans)
                if R.plans{i}.simTime <= t
                    p = R.plans{i};
                end
            end
            if isempty(p)
                if isappdata(0, 'hmmBasePlanTap'), rmappdata(0, 'hmmBasePlanTap'); end
                if app.MapReady
                    set(app.MapH.plan, 'XData', NaN, 'YData', NaN);
                    set(app.MapH.waypoints, 'XData', NaN, 'YData', NaN);
                end
            else
                setappdata(0, 'hmmBasePlanTap', p);
            end
            app.PlanGoal = [];
            app.LegStartTime = -Inf;
            updateMap(app, s);

            % Cameras: the grounding in force at t, under the latest capture at or before t.
            tap = [];
            for i = 1:numel(R.taps)
                if R.taps{i}.simTime <= t
                    tap = R.taps{i};
                end
            end
            % The question Gemini was asked, in the prompt box: the one in force at t, or
            % the run's first before it was sent.
            if ~isempty(tap)
                app.PromptArea.Value = {tap.instruction};
            elseif ~isempty(R.taps) && isfield(R.taps{1}, 'instruction')
                app.PromptArea.Value = {R.taps{1}.instruction};
            end
            if isempty(tap)
                if isappdata(0, 'hmmGroundingTap'), rmappdata(0, 'hmmGroundingTap'); end
            else
                imgs = cell(1, numel(tap.imageFiles));
                for i = 1:numel(imgs)
                    imgs{i} = readRecImage(R.dir, tap.imageFiles{i});
                end
                tap.images = imgs;
                setappdata(0, 'hmmGroundingTap', rmfield(tap, 'imageFiles'));
            end
            live = [];
            j = find([R.frames.t] <= t, 1, 'last');
            if ~isempty(j)
                live = struct('images', {cellfun(@(f) readRecImage(R.dir, f), ...
                    R.frames(j).files, 'UniformOutput', false)});
            end
            if isempty(live)
                showGrounding(app);
            else
                showGrounding(app, live);
            end
            % The wrist frame, while the refinement it was sent for is in progress.
            for i = 1:numel(R.wrist)
                w = R.wrist(i);
                if t >= w.t && t < w.t + 2.0 && isequal(s.plannerMode, app.Cfg.sup.pl.PICK) && ...
                        ~isequal(s.armMode, app.Cfg.sup.arm.APPROACH)
                    ax = app.CamAxes{w.pane};
                    cla(ax, 'reset');
                    image(ax, readRecImage(R.dir, w.file));
                    ax.YDir = 'reverse';
                    axis(ax, 'image');
                    ax.XTick = [];
                    ax.YTick = [];
                    title(ax, 'wrist camera - sent to Gemini ER to refine the grasp', ...
                        'Color', [0.85 0.45 0.05]);
                end
            end

            % A time cursor on the plot, for the views whose x axis is time.
            delete(findobj(app.PlotAxes, 'Tag', 'replayCursor'));
            if ~strcmp(app.ViewDropDown.Value, 'BasePath')
                xline(app.PlotAxes, t, '-', 'Color', [0.9 0.2 0.2], 'LineWidth', 1.2, ...
                    'Tag', 'replayCursor', 'HandleVisibility', 'off');
            end
            app.ReplayTime.Text = sprintf('t = %.1f / %.1f s', t, app.ReplaySlider.Limits(2));
        end

        function onView(app, ~)
            %ONVIEW Another view of the same run.
            showRun(app);
            if ~isempty(app.Replay)
                replayAt(app, app.ReplaySlider.Value);      % keep the time cursor
            end
        end

        function refreshLivePlot(app)
            %REFRESHLIVEPLOT Draw the selected view from the run in progress, growing in place.
            %   THE VIEW IS COMPUTED OFF SCREEN AND ITS DATA COPIED ONTO THE LINES ALREADY DRAWN,
            %   so the visible plot is never cleared and its lines simply lengthen - a refresh
            %   the operator can watch, not a flicker. hmmPlotRun is reused unchanged: it is
            %   handed a Dataset assembled from the SDI channels it reads (liveOut), drawn into a
            %   hidden axes, and when that axes holds the same objects as the visible one only
            %   their data moves across. When the structure differs - the first draw, a view
            %   change, a safety check firing for the first time - the visible axes is redrawn.
            %   Anything not yet logged (the first seconds of a run) is simply skipped.
            %
            %   NO TABLE REBUILD FROM HERE. liveOut probes element spellings that whole-vector
            %   channels never have (ArmJointPos(1,1)), and each miss rebuilt the SDI table:
            %   0.74 s, 24 times in one profiled transfer. readLiveState runs first in the poll
            %   and keeps the table current; this poll's one rebuild is spent.
            app.SdiRebuilt = true;
            out = liveOut(app);
            if isempty(out)
                return
            end
            if isempty(app.RunCfg)
                cfg = app.Cfg;
            else
                cfg = app.RunCfg;
            end
            key = app.ViewDropDown.Value;
            if isempty(app.PlotScratchAx) || ~isvalid(app.PlotScratchAx)
                app.PlotScratchFig = figure('Visible', 'off', 'HandleVisibility', 'off', ...
                    'IntegerHandle', 'off', 'Name', 'hmmLivePlotScratch');
                app.PlotScratchAx = axes(app.PlotScratchFig);
            end
            sc = app.PlotScratchAx;
            cla(sc, 'reset');
            try
                info = hmmPlotRun(sc, out, key, cfg);
            catch
                return
            end
            vis = app.PlotAxes;
            a = sc.Children;
            b = vis.Children;
            same = numel(a) == numel(b) && numel(a) > 0 && ...
                isequal(get(a, {'Type'}), get(b, {'Type'}));
            if same
                for k = 1:numel(a)
                    copyData(a(k), b(k));
                end
                set(vis, 'XLim', sc.XLim, 'YLim', sc.YLim);
                vis.Title.String = sc.Title.String;
            else
                cla(vis, 'reset');
                hmmPlotRun(vis, out, key, cfg);
            end
            app.BlurbLabel.Text = info.Blurb;
            app.ReportArea.Value = {'Live view, growing while the run goes on. The full report appears when the run ends.'};
        end

        function out = liveOut(app)
            %LIVEOUT The leaves hmmPlotRun reads, as a Dataset built from the run in progress.
            %   Only these ~45 channels, through sdiRead's incremental cache - not an export of
            %   the whole run, which measured 0.40 s for 30 s of model time and would have cost
            %   most of every second. Each leaf becomes a timeseries on its own channel's time;
            %   a vector leaf's elements are cut to the shortest, since SDI creates them a
            %   sample apart. Empty until the first leaf has data.
            safety = {'JointLimit', 'JointVel', 'JointAccel', 'Workspace', 'BaseVel', ...
                'BaseAccel', 'StaleState', 'StalePlan', 'StaleProvider', ...
                'EStop', 'Cause'};
            spec = [{'RobotState', 'BasePose', 3; 'RobotState', 'ArmJointPos', 6; ...
                'RobotState', 'ToolTranslation', 3; 'RobotState', 'IsGrasped', 1; ...
                'BaseStatus', 'CrossTrackError', 1; 'BaseStatus', 'DistanceToGoal', 1; ...
                'BaseStatus', 'HeadingError', 1; 'BaseStatus', 'Mode', 1; ...
                'ArmStatus', 'ApertureCmd', 1; 'ArmStatus', 'JointError', 1; ...
                'ArmStatus', 'TrackingError', 1; 'ArmStatus', 'Mode', 1; ...
                'GripperStatus', 'Aperture', 1; 'GripperStatus', 'Mode', 1; ...
                'TaskResult', 'PlannerMode', 1; 'SelectedJointCmd', '', 6}; ...
                [repmat({'SafetyStatus'}, numel(safety), 1), safety(:), num2cell(ones(numel(safety), 1))]];
            vals = struct();
            for k = 1:size(spec, 1)
                [sig, fld, n] = spec{k, :};
                if isempty(fld)
                    base = sig;
                else
                    base = [sig '.' fld];
                end
                cols = cell(1, n);
                tt = [];
                % A VECTOR LEAF IS SOMETIMES ONE CHANNEL, NOT n: the SDI splits RobotState.BasePose
                % into (1)..(3) but keeps RobotState.ArmJointPos and SelectedJointCmd whole. The
                % scalar reader refuses those, so a whole channel is read directly and laid out
                % time-by-element, whichever way round Simulink stored it.
                if n > 1 && isempty(sdiLookup(app, sprintf('%s(1,1)', base)))
                    idWhole = sdiLookup(app, base);
                    if isempty(idWhole)
                        continue
                    end
                    v = Simulink.sdi.getSignal(idWhole).Values;
                    tt = double(v.Time(:));
                    d = double(v.Data);
                    if ndims(d) == 3
                        d = reshape(permute(d, [3 1 2]), numel(tt), []);
                    elseif size(d, 1) ~= numel(tt)
                        d = d.';
                    end
                    if isempty(tt) || size(d, 2) ~= n
                        continue
                    end
                    ts = timeseries(d, tt);
                    if isempty(fld)
                        vals.(sig) = ts;
                    else
                        vals.(sig).(fld) = ts;
                    end
                    continue
                end
                for j = 1:n
                    if n == 1
                        name = base;
                    else
                        name = sprintf('%s(%d,1)', base, j);
                    end
                    [~, y, t] = sdiRead(app, name, true);
                    if isempty(y)
                        cols = {};
                        break
                    end
                    cols{j} = y(:);
                    if isempty(tt) || numel(t) < numel(tt)
                        tt = t(:);
                    end
                end
                if isempty(cols) || isempty(tt)
                    continue
                end
                m = min([numel(tt), cellfun(@numel, cols)]);
                data = cell2mat(cellfun(@(c) c(1:m), cols, 'UniformOutput', false));
                ts = timeseries(data, tt(1:m));
                if isempty(fld)
                    vals.(sig) = ts;
                else
                    vals.(sig).(fld) = ts;
                end
            end
            sigs = fieldnames(vals);
            if isempty(sigs)
                out = [];
                return
            end
            ds = Simulink.SimulationData.Dataset;
            for k = 1:numel(sigs)
                e = Simulink.SimulationData.Signal;
                e.Name = sigs{k};
                e.Values = vals.(sigs{k});
                ds = ds.addElement(e);
            end
            out = struct('logsout', ds);
        end

        function onTabChanged(app, ~)
            %ONTABCHANGED Draw the tab that has just come forward, and only that one.
            %   THE PLOTS ARE DRAWN ON DEMAND. Redrawing seven views of a 60 s run on every
            %   poll would spend the whole second between polls in hmmPlotRun, and six of
            %   the seven would be behind a tab nobody is looking at. The cameras and the map
            %   are cheap and are kept current continuously; the plots are not.
            if app.TabGroup.SelectedTab == app.PlotTab
                showRun(app);
                if ~isempty(app.Replay)
                    replayAt(app, app.ReplaySlider.Value);  % keep the time cursor
                end
            elseif app.TabGroup.SelectedTab == app.MapTab && ~app.MapReady
                initMap(app);
            end
        end

        function showRun(app)
            %SHOWRUN One view of the run on display, with the report beneath it.
            %   THE cfg THE RUN WAS MADE WITH, not the one in the controls now. An operator
            %   who moves the speed slider after a run and then changes view would otherwise
            %   get a limit line that no longer describes what they are looking at.
            if isempty(app.Out) && isRunning(app)
                % A run is going: the view and its tab are drawn live from the SDI.
                refreshLivePlot(app);
                app.PlotRefreshTic = tic;
                return
            end
            if isempty(app.Out)
                cla(app.PlotAxes, 'reset');
                title(app.PlotAxes, '');
                app.BlurbLabel.Text = ['Nothing to plot yet. A running simulation draws the ' ...
                    'selected view live, and the full plots and report appear when it ends. ' ...
                    'Load recorded run shows a saved one.'];
                app.ReportArea.Value = {'No run on display.'};
                return
            end

            if isempty(app.RunCfg)
                cfg = app.Cfg;
            else
                cfg = app.RunCfg;
            end

            key = app.ViewDropDown.Value;
            try
                info = hmmPlotRun(app.PlotAxes, app.Out, key, cfg);
                app.BlurbLabel.Text = info.Blurb;
            catch ME
                cla(app.PlotAxes, 'reset');
                app.BlurbLabel.Text = sprintf('%s could not be drawn.', key);
                app.ReportArea.Value = cellstr(splitlines(string(ME.getReport('basic'))));
                return
            end

            % CAPTURED RATHER THAN REIMPLEMENTED. hmmReportRun prints, and the alternative
            % to evalc is a second copy of the reduction in this file - which is the exact
            % duplication hmmRunSummary's own header exists to prevent. A report the app
            % wrote itself could call a run successful that the live script called failed.
            try
                txt = evalc('hmmReportRun(app.Out, cfg)');
                app.ReportArea.Value = cellstr(splitlines(string(txt)));
            catch ME
                app.ReportArea.Value = cellstr(splitlines(string(ME.getReport('basic'))));
            end
        end

        %% ------------------------------------------------------------- the two lamps

        function setStatus(app, level, text)
            %SETSTATUS The app's own state: what it is doing, or why it stopped.
            app.StatusLamp.Color = lampColour(level);
            app.StatusLabel.Text = text;
            drawnow limitrate
        end

        function setRoute(app, level, text)
            %SETROUTE The provider's state, kept separate from the app's on purpose.
            %   TWO LAMPS BECAUSE THERE ARE TWO THINGS THAT CAN GO WRONG INDEPENDENTLY. A
            %   healthy simulation with a refused request and a failed start with a cached
                %   plan are different situations, and one lamp would have to pick one of
            %   them to report.
            app.RouteLamp.Color = lampColour(level);
            app.RouteLabel.Text = text;
            drawnow limitrate
        end

        function name = routeName(app, code)
            %ROUTENAME Which discovery mechanism answered, in words - and NEVER the key.
            %   THE ROUTE IS REPORTABLE AND THE CREDENTIAL IS NOT, which is the project's
            %   rule and the reason this function exists rather than the app printing
            %   anything it received. A route is a fact about how this machine is
            %   configured; it names a mechanism, not a secret, and no value here is derived
            %   from the key's contents.
            switch code
                case double(app.Cfg.provider.ROUTE_ENV)
                    name = 'GEMINI_API_KEY';
                case double(app.Cfg.provider.ROUTE_KEYFILE)
                    name = 'key file';
                case double(app.Cfg.provider.ROUTE_REPLAY)
                    name = 'sample response';
                otherwise
                    name = 'none';
            end
        end

    end

    methods (Access = public)

        function app = HomeMobileManipulatorApp
            %HOMEMOBILEMANIPULATORAPP Construct and show the console.
            createComponents(app);
            registerApp(app, app.UIFigure);
            runStartupFcn(app, @startupFcn);

            if nargout == 0
                clear app
            end
        end

        function delete(app)
            %DELETE Stop the timer, put the model's StopTime back, and leave the run alone.
            %   THE TIMER MUST GO FIRST AND UNCONDITIONALLY. A timer whose callback holds a
            %   deleted app fires once a second into a destroyed handle for the rest of the
            %   MATLAB session, and the only symptom is a warning printed over whatever the
            %   user is doing next.
            stopPolling(app);
            if isappdata(0, 'hmmAppStateWanted')
                rmappdata(0, 'hmmAppStateWanted');    % script runs pay nothing after this
            end
            if ~isempty(app.PlotScratchFig) && isvalid(app.PlotScratchFig)
                delete(app.PlotScratchFig);           % the live plot's hidden figure
            end
            if ~isempty(app.ReplayTimer) && isvalid(app.ReplayTimer)
                stop(app.ReplayTimer);
                delete(app.ReplayTimer);
            end
            % The refinement listener holds this app too; a run that outlives the window
            % must find no listener rather than a deleted one (hmmRefineNotify would swallow
            % the error, but there is no reason to make it).
            if isappdata(0, 'hmmRefineListener')
                rmappdata(0, 'hmmRefineListener');
            end

            % THE RUN IS DELIBERATELY NOT STOPPED. Closing a console is not a decision to
            % abandon a simulation that may be twenty minutes in, and the run is
            % asynchronous - it belongs to the model, not to this window. What is undone is
            % only what this app changed: the StopTime it wrote onto the model, and the carry
            % offset literal onSend may have written over RobotTree's own expression.
            try
                if ~isempty(app.StopTimeWas) && bdIsLoaded(app.Model)
                    set_param(app.Model, 'StopTime', app.StopTimeWas);
                end
            catch
            end

            % PUT THE EXPRESSION BACK, NOT A NUMBER. The shipped mask reads carryTranslation,
            % a base-workspace variable hmmSetup publishes; leaving this app's literal behind
            % would make the model's behaviour depend on which sentence happened to be typed in
            % a window that has since closed, and would survive a save.
            try
                if ~isempty(app.CarryTransWas) && bdIsLoaded(app.Model)
                    set_param(carryBlock(app), 'eeTargetTrans', app.CarryTransWas);
                end
            catch
            end

            delete(app.UIFigure);
        end

    end
end

%% ============================================================================
% The helpers below are ordinary functions rather than methods because none of
% them touches the app: they map a value to a value. Keeping them out of the
% class says so, and it also lets them be read without knowing anything about the
% interface they happen to serve.

function c = lampColour(level)
%LAMPCOLOUR One of four states, as a colour.
%   FOUR LEVELS AND NOT A COLOUR PER CALLER. Every lamp in the app means one of
%   these four things - nothing yet, working, went well, went wrong - so the
%   mapping lives once and a new message cannot invent a fifth shade of amber.
switch level
    case 'ok'
        c = [0.20 0.70 0.30];
    case 'busy'
        c = [0.95 0.70 0.10];
    case 'error'
        c = [0.85 0.20 0.20];
    otherwise
        c = [0.80 0.80 0.80];
end
end

%% ----------------------------------------------------------------------------
function name = enumName(namespace, value)
%ENUMNAME One of hmmConfig's uint8 codes, in words an operator can read.
%   READ BACK OUT OF THE SAME STRUCT THE MODEL COMPILED AGAINST, for the reason
%   hmmEnumName's own header gives: a list of names written out here would be a
%   second, unchecked copy of the enumeration, and it would keep printing
%   confidently after somebody inserted a code in the middle.
%
%   A COUNT IS NOT A CODE. cfg.sup.cause holds nCauses = 10 alongside
%   STALE_PROVIDER = 10, so a plain search of the struct can return the field that
%   says how many causes there are as though it were the cause. Only the ALL-CAPS
%   fields are enumeration members, which is the convention hmmConfig follows
%   throughout, so lower-case fields are skipped rather than tie-broken on
%   declaration order.
%
%   UNMATCHED IS REPORTED, NOT THROWN. A state panel that errored on an unexpected
%   code would destroy the evidence of the very thing worth seeing.
name = sprintf('%g?', value);
if ~isstruct(namespace) || ~isfinite(value)
    return
end
f = fieldnames(namespace);
for k = 1:numel(f)
    if ~strcmp(f{k}, upper(f{k}))
        continue
    end
    v = namespace.(f{k});
    if isnumeric(v) && isscalar(v) && double(v) == double(value)
        % Underscores out and lower case in: NAV_TO_PICK is the model's spelling and
        % "nav to pick" is the operator's. The name still matches the plots and the
        % report word for word, which is what makes the panel cross-referenceable.
        name = lower(strrep(f{k}, '_', ' '));
        return
    end
end
end

%% ----------------------------------------------------------------------------
function [img, slot] = frameFor(tap, cameraIndex)
%FRAMEFOR The frame this camera contributed to the request, or nothing.
%   slot IS THE ANSWER TO "WAS THIS CAMERA SENT AT ALL", and it is returned
%   separately for a reason. Only admissible frames go into a request, so a camera
%   whose view was stale simply is not in the list - and the honest display for it
%   is an empty pane saying so. Falling back to another camera's picture would put
%   a marker measured somewhere else onto a view it does not belong to, which looks
%   exactly like a grounding error in the model.
img = [];
slot = 0;
if ~isstruct(tap) || ~isfield(tap, 'images') || ~isfield(tap, 'indices')
    return
end
hit = find(tap.indices == cameraIndex, 1);
if isempty(hit) || hit > numel(tap.images)
    return
end
candidate = tap.images{hit};
if isempty(candidate) || ndims(candidate) ~= 3
    return
end
img = candidate;
slot = hit;
end

%% ----------------------------------------------------------------------------
function s = httpMeaning(code)
%HTTPMEANING The few HTTP statuses a provider call returns, in words for the operator.
switch double(code)
    case 200, s = '(OK)';
    case 400, s = '(Bad Request)';
    case 401, s = '(Unauthorized - check the key)';
    case 403, s = '(Forbidden - check the key)';
    case 404, s = '(Not Found - check the model name)';
    case 429, s = '(Too Many Requests - rate limit or quota)';
    case 500, s = '(Internal Server Error, Google side)';
    case 503, s = '(Service Unavailable, Google side)';
    case 504, s = '(Gateway Timeout, Google side)';
    case 0,   s = '(no reply)';
    otherwise, s = '';
end
end

function b = boxOf(action, field)
%BOXOF One [ymin xmin ymax xmax] box off a decoded action, or empty.
%   TOLERANT OF AN ACTION THAT HAS NO BOX FIELD AT ALL, which is not a hypothetical:
%   root appdata outlives a MATLAB path change, so a tap struct published before
%   hmmGroundingTap grew its boxes argument can still be sitting there when a newer
%   app polls it. An empty box draws nothing, which is the right outcome; a missing
%   field would take the whole camera pane down with it.
b = zeros(1, 0);
if ~isstruct(action) || ~isfield(action, field)
    return
end
v = action.(field);
if numel(v) ~= 4 || any(~isfinite(v))
    return
end
b = reshape(double(v), 1, 4);
end

%% ----------------------------------------------------------------------------
function [t, d] = recLeaf(out, sig, fld)
%RECLEAF One logged leaf of a recording as time-by-element, or empty if it is not there.
t = [];
d = zeros(0, 1);
try
    v = out.logsout.getElement(sig).Values;
    if ~isempty(fld)
        v = v.(fld);
    end
catch
    return
end
t = double(v.Time(:));
d = double(v.Data);
if ndims(d) == 3
    d = reshape(permute(d, [3 1 2]), numel(t), []);
elseif size(d, 1) ~= numel(t)
    d = d.';
end
end

function v = recAt(out, sig, fld, t0)
%RECAT A scalar leaf's value at the last sample at or before t0, NaN when there is none.
v = NaN;
[t, d] = recLeaf(out, sig, fld);
k = find(t <= t0, 1, 'last');
if ~isempty(k)
    v = d(k, 1);
end
end

function img = readRecImage(d, f)
%READRECIMAGE A recorded JPEG, or a grey frame when the file is missing.
img = uint8(128 * ones(360, 640, 3));
if isempty(f)
    return
end
try
    img = imread(fullfile(d, f));
catch
end
end

%% ----------------------------------------------------------------------------
function copyData(src, dst)
%COPYDATA Move one graphics object's data onto its twin, whatever kind of object it is.
%   Lines, stairs, scatter and images carry X/Y(/Z/C)Data, text a Position and String,
%   constant lines a Value, patches Vertices and Faces. Properties a type lacks are skipped.
props = {'XData', 'YData', 'ZData', 'CData', 'Vertices', 'Faces', 'Position', 'String', 'Value'};
for p = props
    if isprop(src, p{1}) && isprop(dst, p{1})
        try
            dst.(p{1}) = src.(p{1});
        catch
        end
    end
end
end
