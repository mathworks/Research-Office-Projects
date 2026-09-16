classdef DroneLightShowApp < handle
    % DroneLightShowApp Interactive Multi-UAV Drone Light Show Control
    %   Launch with: DroneLightShowApp
    %
    %   Provides fleet configuration, show generation, real-time 3D
    %   visualization with quadrotor meshes (uavScenario), and live
    %   telemetry during playback or full Simulink simulation.

    properties (Access = private)
        % UI Figure and Layout
        Fig

        % Fleet & Formation controls
        NumUAVSpinner
        FormationList    % the show's sequence, as one "A→B→C" string
        FormationPicker  % which single formation the Add button appends
        AddFormBtn
        RemoveFormBtn
        ClearFormBtn
        SequenceLabel    % the sequence spelled out, step by step
        LoadShapeBtn     % Load Image / STL: builds a formation from a file
        ShapeLabel       % which shapes are loaded
        TextField        % the string the fleet spells
        FlyTextBtn       % Fly Text: builds a formation from that string

        % The sequence being built, as display names. Always kept equal to
        % FormationList.Value split on the arrow, so the preset dropdown and the
        % Add/Undo buttons are two views of one sequence rather than two sequences
        % that can disagree about what the show is.
        SequenceNames = {}
        % Which dropdown item, if any, is the sequence the buttons are editing. Held
        % so it can be REPLACED as the sequence grows: appending each intermediate
        % sequence instead would leave "Grid", "Grid→Circle", "Grid→Circle→Grid" all
        % sitting in the list after three clicks.
        BuiltSeqItem = ''

        % Shapes loaded from a picture or an STL, or rendered from typed text, in
        % the order they were added. The formation type is 4 + index here: that is
        % what setupParams looks up in custom_formations and what lighting_colors
        % row 5 onward colours. Text is registered the same way a file is so a show
        % can spell SEVERAL different words -- one string in formation_text would
        % only ever give one.
        CustomFormations = struct('name', {}, 'kind', {}, 'cloud', {}, ...
            'cloudFill', {}, 'source', {})

        SpacingField
        AltitudeField
        TransitionField
        HoldField
        RotateCheckbox   % master on/off for rotation; the angles below say which and how far
        RotateStepDropdown  % which step of the sequence the angle field is editing
        RotateAngleField    % degrees that step turns; 0 = it stays still
        RotationLabel    % how far it actually got to spin -- see refreshRotationLabel

        % How far each step of the sequence turns, in degrees, one entry per step. This is
        % the app's copy of setupParams' rotation_deg_request and it is edited one step at
        % a time through the picker above -- a table showing all of them at once was the
        % alternative, and it costs ~120 px of panel for something usually set once.
        %
        % Zero means that formation does not rotate, which is the whole point: "rotate
        % everything" was the only thing the checkbox alone could say. Kept in step with
        % SequenceNames by refreshRotationSteps, which RESIZES rather than resetting, so
        % adding a sixth formation does not discard the angles chosen for the first five.
        RotationDegPerStep = []

        % Flight & Safety
        VmaxField
        AmaxField
        DminField

        % Navigation
        RTKTierLabel     % live readout, NOT a setting -- see createNavPanel
        DegradeUAVField
        UavLossField
        DegradeBtn
        RestoreBtn
        DenyLabel

        % Communication
        LatencyField
        JitterField
        PacketLossField
        TimeoutField
        % Live readouts, NOT settings -- see createCommPanel. These name the MAVLink
        % message on the wire right now, in each direction, plus what the viewer is
        % currently drawing.
        UplinkMsgLabel
        DownlinkMsgLabel
        RtcmMsgLabel
        ViewSourceLabel

        % Simulation Controls
        GenerateBtn
        PlayBtn
        SimBtn
        StopBtn
        AbortBtn         % Abort & Land: a mission command, not a halt. See abortAndLand.
        FlowHintLabel        % Generate -> fly -> Play chain, rewritten by setFlowHint
        ResetBtn
        SpeedSlider
        ScrubSlider          % playback position, in FRAMES: 1 .. NumSamples. See seekFrame
        ScrubTimeLabel       % "12.3 / 72.3 s" beside the scrub bar
        SimModeDropdown
        SimModeNote

        % Telemetry Labels
        TimeLabel
        StateLabel
        MinSepLabel
        MaxErrLabel
        SpeedLabel
        FormLabel

        % Status bar
        StatusBar

        % 3D scene
        ScenarioAxes
        DronesPlot     % scatter3 handle for drone LEDs
        GroundPatch    % ground plane patch
        TitleText      % cached axes Title handle; title() per frame is 13x dearer
        ViewerTabGroup      % right-hand tab group: 3D Viewer + Guide
        ViewerTab           % tab 1, holds ScenarioAxes
        GuideTab            % tab 2, holds the Guide
        GuideHtml           % uihtml Guide, or a uitextarea where uihtml is unavailable
        LastFormType = []   % last formation type drawn, to skip needless recolours
        DenyMaskCache = []  % logical fleet vector: who is RTK-denied, for the red highlight
        LastDenyMask = []   % the mask the last frame was drawn with, same purpose as above
        DenyRings           % one scatter3 of hollow rings marking the denied drones. The
                            % palette-independent half of the marking: red alone collides
                            % with the default formation colours (see buildScenario).
        TrailPlots     % array of animatedline handles per UAV, which keep their own history

        % Drone rendering, three modes. 'Markers only' and 'Spheres' are both a
        % single vectorized scatter3 update and differ only in marker size and
        % edge; 'Quadrotor meshes' is one patch per drone carrying the UAV
        % Toolbox geometry (792 vertices each), translated per frame. Whichever
        % family is inactive has no handles at all, and that is how drawFrame
        % tells them apart.
        %
        % Be clear about where the saving is: Markers-vs-Spheres buys a little
        % rasterising, Meshes-vs-either buys the N per-drone .Vertices writes,
        % which is the term that actually hurts at 200 drones. Hence the hint.
        DroneRender = 'Spheres'
        RenderDropdown
        RenderHintLabel
        MeshPatches = gobjects(0)
        MeshV0 = []          % [nv x 3] quadrotor vertices, plot frame, body-centred
        MeshF  = []          % [nf x 3] faces
        MeshScale = 2        % m across the rotors; 1 m is too small to read at
                             % show scale, 4 m starts to overlap at 5 m spacing

        % Light trails. One animatedline per drone, so drawFrame pays an addpoints
        % per drone per frame whether or not anyone is looking at them -- on a large
        % fleet that is the second-biggest per-frame cost after meshes. Off means no
        % handles at all rather than hidden ones, because a hidden animatedline still
        % costs the addpoints.
        ShowTrails = true
        TrailsCheckBox

        % The planned-path overlay, one phase at a time. It is the opposite of the trails
        % in every way that matters to the cost: a trail is HISTORY, per drone, grown by a
        % point per frame; this is the PLAN, drawn whole the instant a phase starts and
        % then left completely alone until the phase changes. Nothing here runs per frame
        % except the "is it still the same phase?" test.
        %
        % Hence ONE line object for the entire fleet, with the drones' paths separated by
        % NaN, rather than one per drone: the paths do not move within a phase, so there
        % is nothing to update, and 500 handles created and destroyed on every phase
        % change would be felt where a single XData write is not. TrajSegKey is the phase
        % currently drawn, [NaN NaN] meaning "nothing to draw".
        ShowTrajectory = false
        TrajCheckBox
        TrajPlot = gobjects(0)
        TrajSegKey = []      % [tStart tEnd] of the drawn phase, show-relative seconds

        % Per-formation-TYPE lighting colour, as chosen on the panel. NaN rows mean
        % "leave setupParams' default alone", which is what keeps this from having to
        % duplicate the 12-row default table -- there would then be two sources of
        % truth for the show's colours and no way to tell which one flew.
        ColorOverrides = nan(12, 3)
        ColorFormDropdown
        ColorSwatchBtn

        % Live-view rate control. The run is asynchronous but single-threaded with
        % the UI, so the solver only advances inside pause(): each cycle is
        % (UI cost u) + (pause p), and the sim advances by about S*p where S is the
        % solver's own rate in sim-seconds per second of uninterrupted compute.
        % Achieved real-time factor is therefore R = S*p/(u+p), which inverts to
        % p = R*u/(S-R) -- the interval that HITS a chosen factor instead of a fixed
        % 0.1 s that happens to give whatever it gives. u and S are both measured per
        % poll and smoothed, because u depends far more on the render mode and the
        % trails than on the fleet size, so no threshold on N_uav could stand in for it.
        %
        % R IS FIXED AT 1, AND USED TO BE A DROPDOWN. "Real time / 2x / Fastest" was
        % offered for a while and then removed, because a live rate is not a viewing
        % preference the way playback speed is:
        %   - No setting of it made the live view good. At 1x the hold below dominates
        %     the cycle and the solver yields in ~315 ms chunks, so the animation was
        %     ~3 fps whatever the target; unpaced, the wall clock is short but each
        %     frame jumps further. Smooth AND quick was never on the menu.
        %     THAT SECOND CLAUSE NO LONGER HOLDS ON A SMALL FLEET, and the conclusion
        %     survives anyway. Simulink's pacing (see PaceLive) makes the solver wait
        %     between steps rather than padding the cycle, which cuts the chunk to ~60 ms
        %     and put a 1x live run at 7.7 fps measured over a full 10-drone MAVLink show.
        %     So smooth and quick IS now on the menu -- but it arrived by measuring the
        %     machine rather than by asking the operator, which is the argument against the
        %     dropdown, not for it. At 120 drones it is still ~3 fps, because pacing can
        %     only reclaim the SOLVER's share of a cycle and there the renderer owns it.
        %   - "Fastest" erased the phenomenon the example exists to show. The downlink
        %     gives each drone one slot per tick, which reads as a wave crossing the
        %     fleet -- measured, 26% of drones move between frames at 120, and re-measured
        %     at 27% after the pacing change, which is the check that mattered because the
        %     figure is a frame PERIOD read against the slot schedule. Run unpaced
        %     and the model outruns the wall clock enough to refresh the whole table
        %     between frames (99%), and the wave vanishes. That measurement is what
        %     README leads with; it must not depend on where a dropdown was left.
        %   - The speed axis already exists and is honest about its cost: Sim Mode.
        %     Rapid Accelerator finishes in 14.9 s against ~44 s and says outright that
        %     there is no live view. "I am not watching this" is the real request
        %     Fastest was standing in for, and Sim Mode answers it better.
        % So a streamed run is always paced to 1x where that is reachable, and the
        % readout reports the factor actually achieved where it is not. Two overlapping
        % speed knobs also cost five separate pieces of prose to keep apart, which is
        % what a control that should not exist looks like from the documentation side.
        RateTarget = 1        % real-time factor the governor aims for; not user-settable
        RateLabel
        PollInterval = 0.1    % what the controller last chose, s
        PollMin = 0.02        % below this the UI is starving the solver for no visible gain
        PollMax = 0.5         % above this the stream is too coarse to be a replay
        % The pause is how the solver is GRANTED time, so it can only make the show
        % faster; with a floor on it, the slowest the show can go is S*PollMin/(u+PollMin),
        % measured at 2.7x with a 1x target. The hold is the other direction: wall clock
        % spent blocking, which the asynchronous run cannot use. See nextPollInterval.
        HoldInterval = 0      % what the controller last chose to withhold, s
        % Blocking, so this IS the Stop-button latency, and it is set by what real time
        % costs rather than by taste: at 8 drones the solver advances ~315 ms of show per
        % yield, so holding that back to 1x needs ~240 ms and a smaller cap would make
        % "Real time" unreachable on the most ordinary fleet size there is.
        HoldMax = 0.35
        % SIMULINK'S OWN PACING does the waiting instead wherever the model accepts it,
        % and the spin above is the fallback. Not a preference: the spin cannot buy
        % frames. A cycle is at least one solver chunk long, an unthrottled chunk is
        % 230 ms of show at 8 drones, and the spin only pads the cycle further -- so a 1x
        % target reached by spinning streams at 4.3 fps and visibly skips. Pacing makes
        % the solver wait BETWEEN steps, which cuts the chunk to ~60 ms, and the same 1x
        % target then streams at 10.1 fps with the Stop button live throughout. Both
        % measured on the same show with 40 ms of drawing per frame.
        PaceLive = true       % false falls back to the spin hold
        PacingOn = false      % pacing is engaged on the model right now
        PacingFailed = false  % the model refused it; stop asking for the rest of the run
        % What to ASK FOR is not the target -- see nextPollInterval. Pacing catches up
        % only within a slice this loop has granted, so the drawing cost comes straight
        % off the achieved factor: asking for 1x measured 0.71x with 40 ms frames and
        % 0.46x with 100 ms. The value below is only a placeholder: pacingMark seeds the
        % opening ask each time the throttle engages, generously, because starting the
        % show below real time is the one thing it must never do.
        PacingRate = 2        % what the loop is currently asking Simulink for
        PacingRateMax = 20    % past this pacing cannot bind on any machine; it is inert
        PacingWritten = NaN   % last value pushed to the model, to throttle the set_params
        PacingWas = {}        % {PacingRate} as found, to put back on the way out
        % Sustained rather than instantaneous, because one slow cycle is a garbage
        % collect or a phase change and not a verdict on the throttle.
        LowRateWall = 0       % wall clock spent under target while paced, s
        HighRateWall = 0      % wall clock spent over target while not paced, s
        UICostEMA = []        % smoothed u, s per poll
        FrameCostEMA = []     % smoothed non-solver wall clock, s per sim ADVANCE
        SolverRateEMA = []    % smoothed S, sim-s per solver-s
        CycleEMA = []         % smoothed MEASURED cycle length, s -- the honest fps source
        AchievedRate = NaN    % smoothed measured R -- drives the readout AND gradeLiveRate
        % Wall clock spent since the sim clock last moved, carried forward so that a poll
        % which lands inside a solver chunk is charged to the advance it helped buy rather
        % than thrown away. See nextPollInterval; without these the law overstated the
        % rate by 1.42x at a 60 ms slice.
        PendWall = 0          % all of it, s
        PendGrant = 0         % the part the solver was allowed to use, s
        BehindWall = 0        % wall clock paced and under target with r still climbing, s
        % Whole-run accounting, so the readout can end on something meaningful. The
        % per-poll numbers above are instantaneous by design -- the controller needs the
        % latest estimate, not the average -- but that makes the LAST poll of a run a
        % terrible summary of it, and the last poll is exactly the one left on screen.
        % These sum the same quantities over the run so the end of a show can report the
        % factor the show actually ran at, and a caller can close the wall-clock budget
        % (u + pause + everything neither of those accounts for).
        PollStats = struct('n', 0, 'nMoved', 0, 'sumU', 0, 'sumP', 0, 'sumH', 0, 'sumSim', 0)
        ShowRate = NaN        % sim-seconds per wall-second over the whole polled loop

        % Playback State
        Playing = false
        PlayLoopActive = false  % a playLoop is already on the stack; see playLoop
        PlayIdx = 1
        PlaySpeed = 1.0
        Scrubbing = false     % a scrub-bar drag is in progress; see seekFrame
        ScrubWasPlaying = false  % ...and it interrupted playback, so the release resumes it
        % Playback is paced against a WALL-CLOCK DEADLINE, not by skipping a fixed number
        % of samples per fixed pause. PaceClock starts when playback is armed and
        % PaceRefTime is the show time it started from, so the sample due now is always
        % PaceRefTime + elapsed*PlaySpeed. Deriving each frame from the clock rather than
        % from the last frame is what keeps it drift-free: pause() overshoots by a few ms
        % on every call, and 300 frames of that is seconds of accumulated error.
        FrameWait = 0         % wall seconds playLoop should wait before the next frame
        PaceClock = []        % tic handle, armed by beginPlayback
        PaceRefTime = 0       % show time playback was armed at, s
        % The SMALLEST yield playLoop is allowed to take, whatever the pacing says. It is
        % only ever reached when FrameWait is zero, i.e. when the renderer is already slower
        % than the requested speed -- 2x on the default 10-drone show, 1.0x at 60 drones,
        % any speed on a big fleet -- which is exactly when a click has the least room.
        %
        % IT IS NOT WHAT FIXED THE LAGGY PAUSE, and the number is small enough to invite the
        % opposite guess, so: playback used to ignore its own Pause for SECONDS. Measured as
        % the wall time from posting a Pause to the show stopping -- 10 drones 0.06 s at 1.0x
        % but 3.29 s at 2x and 4.85 s at 4x, 60 drones 4.76 s at 1.0x -- and sporadically, the
        % same case landing in 0.01 s on the next run. Widening this floor from 1 ms to 15 ms
        % barely moved it (still 2.31 s at 2x), which is what ruled the sleep out. The gate was
        % the FLUSH: see the plain `drawnow` in playLoop and the note there. What separated the
        % two was splitting the lag into DISPATCH (the queue not being drained) and RETURN (the
        % loop carrying on after the click was serviced) -- dispatch was the whole 4.41 s and
        % return never exceeded 0.07 s, so no amount of sleeping was going to help.
        %
        % With the full flush in place the floor is worth about 0.01 s either way, so it sits
        % at 2 ms: enough that the loop never asks for pause(0), cheap enough to cost no
        % frames. Frames are the currency, not speed -- advanceFrame picks the sample the
        % clock says is due and skips the rest, so a wider yield only draws the same show
        % more coarsely. Worst Pause over three repeats of each of four configurations is
        % now 0.04-0.05 s.
        YieldFloor = 0.002
        CameraMode = 'Static'  % 'Static' or 'Dynamic'
        CameraModeDropdown
        CameraAzOffset = 0

        % Which pose the 3-D view shows during a MAVLink run. The telemetry downlink
        % is modelled as a self-scheduled broadcast -- TelemetryEncoder's free-running
        % TelCounter gives each drone a recurring slot and stamps its own SystemID, and
        % the BaseStation just listens -- so the received table is refreshed one ROW at
        % a time and a whole pass takes N_uav*Ts_sim (0.2 s at 20 drones, 5 s at 500).
        % That row-by-row sweep is the modelled radio, not a viewer artefact, but it is
        % surprising to watch, so the operator can ask for the true airframe pose
        % instead. 'downlink' is what the ground station actually knows; 'true' taps
        % FlightDynamics directly and is the honest answer to "where is the fleet".
        PoseView = 'downlink'   % 'downlink' or 'true'
        LiveLogIsTruePose = false  % the streamed frames are true pose, so finishLiveRun
                                   % may replace the poll log with the dense Ts_sim one
        LiveStreamedFrames = 0     % frames actually drawn live, kept for the closing
                                   % message after the replay log is made denser
        PoseViewDropdown

        % Cached Data
        TrajectoryData      % planned trajectory, DISPLAY copy: [N x 3 x samples],
                            % resampled to a uniform ~20 Hz grid by
                            % resampleViewerTrajectory. The 7-column keyframe grid that
                            % goes on the wire stays in base workspace trajectory_data.
        SimTrajectoryData   % simulated positions [N x 3 x samples] (after Simulate)
        SimTimeVector       % simulated time vector
        SimPhaseData        % logged Phase (0=Idle,1=Upload,2=Armed,3=Show,4=Landed,5=Landing)
        SimUploadCount      % logged upload arrival count
        UploadTotal         % total items expected (N_uav * num_waypoints)
        UseSimData = false  % true after Simulate completes
        TimeVector
        LightingData
        NumUAVs
        NumSamples
        FormationColors

        % Re-entrancy guard for fitTransitionToPlan, which regenerates the show and so
        % re-enters generateShow -- where the auto-fit is triggered from. Without it a
        % plan that stays too fast even at the longest transition would recurse.
        AutoFitBusy = false

        % Show timeline, cached from setupParams by cacheShowTimeline. These are
        % the authoritative segment boundaries the plan was actually built from,
        % in show-relative time: re-deriving them in the app got the takeoff
        % climb wrong and reported every formation 5 s early.
        TimelineTimes = []   % start of each segment
        TimelineTypes = []   % 0 = hold on a formation, 1 = transition
        FormSeq = []         % formation type per formation, e.g. [1 2 1]
        PreShowDelay = 0     % PREDICTED sim time of show-relative 0, from setupParams
        ShowEntryTime = []   % OBSERVED sim time the chart entered SHOW; [] until seen
        PlanLandStart = []   % show-relative time the planned descent begins
        PlanLandEnd = []     % show-relative touchdown; parked from here to the end

        % Phase + Upload UI
        PhaseLabel
        UploadLabel
        TrajSourceDropdown   % how the waypoints reach the drones
        UploadBtn
        UploadLamp
        UploadBar        % uihtml wrapping an HTML5 <progress> element
        UploadGauge      % fallback linear gauge if uihtml is unavailable
        UseHtmlBar = true

        % Pre-flight upload state
        UploadOK = false     % true once every waypoint is confirmed onboard
        UploadAttempt = 0    % attempt counter; drives a fresh packet-loss seed
        UploadBarHistory = []  % every pct pushed to the bar, for diagnostics
        % Predicted streaming time for THIS upload, sim seconds, cached once when the
        % upload starts. The bar cannot say how long it will take -- the mission streams
        % one MISSION_ITEM_INT per waypoint per drone at one packet per tick, so the wait
        % is linear in the fleet and around 1.09 s per drone, which is over a minute
        % before the fleet leaves the ground on any fleet worth calling a show. Worse,
        % the bar reads 0% for the whole GPS-lock wait that precedes the first packet,
        % because there is genuinely nothing delivered yet. A bar creeping toward an
        % unstated target is what makes an operator conclude the app has hung and kill
        % it, so the target is stated instead. Cached rather than read per poll: the poll
        % cost is the term the live-rate controller is spending (see UICostEMA).
        UploadEstimate = NaN

        % Fleet size the currently loaded model was loaded for. Reloading costs
        % ~26 s (44 MAVLink masked blocks re-evaluate their dialects), so it is
        % only done when the port dimensions actually change.
        LoadedNUAV = []

        % Sim status flag
        SimRunning = false

        % Set by Abort & Land, cleared when the next run is prepared. Kept past the
        % end of the run so the closing status message can say the show was aborted
        % rather than flown.
        AbortRequested = false

        % Phase read off the running model while a live show is streaming.
        % Empty at every other time, which is how updateTelemetry tells a live
        % frame from a replayed one.
        LivePhase = []
        LiveStopTime = []    % StopTime of the run being streamed
        LivePhaseLog = []    % phase per streamed frame, for replay afterwards
        LiveCountLog = []    % items confirmed onboard per streamed frame

        % rtk_sigma_table, read ONCE per run rather than per poll. Each poll costs the
        % solver directly (the run is async but single-threaded with the UI), and an
        % evalin into the base workspace is far more expensive than a numeric compare.
        SigmaTiers = []

        % Last MAVLink-received pose per drone, [N x 3] NED. Full-fidelity mode draws
        % the decoded telemetry table rather than the true airframe position, and that
        % table is round-robin: each drone's row only refreshes in its own slot, and a
        % row that has never arrived reads exactly all-zero because the receiver's
        % TelMemory initialises to zeros(N_uav, 6). Holding the last value seen -- and
        % seeding the hold from the pad -- is what stops an unheard-from drone being
        % drawn at the origin. Display layer only; no model edit, nothing logged.
        RxPoseHold = []

        % Where each drone is STANDING, [N x 3] NED, captured once per run from
        % init_positions. Kept separate from RxPoseHold, which the received rows
        % overwrite: the pad pose has to stay pristine to be drawn.
        PadPose = []

        % RuntimeObject handles for the Radio-traffic readout, resolved once per run.
        % Cached for the same reason SigmaTiers is: a get_param per poll is time the
        % solver does not get. A field left empty simply reads '—'; it never skips a frame.
        RadioRto = struct()

        % The largest distance between a drone's received pose and its pad while the
        % pre-show hold is drawing the pads instead. This is what the hold SUPPRESSES, so
        % it is reported on the panel rather than silently discarded -- see poseForView.
        PadHoldOffset = 0
    end

    methods
        function app = DroneLightShowApp()
            app.createUI();
            % So the workspace agrees with the delivery dropdown from launch,
            % even if the operator never touches it.
            app.publishTrajSource();
            % An rtk_deny_mask left in the workspace by an earlier session is live from
            % the first run, so the panel reads it at launch rather than claiming 'none'.
            app.refreshDenyLabel();
            app.updateStatus('Ready. Configure parameters and click Generate.');
        end

        function delete(app)
            app.Playing = false;   % lets a suspended playLoop fall out on resume
            if ~isempty(app.Fig) && isvalid(app.Fig)
                delete(app.Fig);
            end
        end

        % Playback runs in a loop (see playLoop), so clicking Play hands control
        % to the app until the show ends. These two are the seam for a caller that
        % wants to clock playback itself -- arm it, then step a frame at a time --
        % which is how the viewer and the telemetry panel can be sampled frame by frame.
        function armPlayback(app)
            app.beginPlayback();
        end

        function stepPlayback(app)
            app.advanceFrame();
        end

        % The same seam for the live-rate controller. Its whole job is arithmetic on
        % measured costs, and the measurements only exist inside a streaming run whose
        % poll loop holds the thread -- so without this the control law could only be
        % checked by watching it, which is not a check. Arm it, feed it (uiCost, dSim,
        % dPause), and read back the interval it chose.
        function armLiveRate(app, paced)
            % `paced` forces which throttle is under test. The pacing law is only reachable
            % while pacing is engaged, and engaging it for real takes a running model --
            % exactly what a seam does not have. Defaults to the fallback, so every check
            % written against the hold reads the same as it did before pacing existed.
            app.armPollRate();
            if nargin > 1
                % Through pacingMark, so a seam run gets the same opening ask a real one
                % does rather than whatever armPollRate left in PacingRate.
                app.pacingMark(logical(paced));
            end
        end

        function [p, w, pace] = stepLiveRate(app, uiCost, dSim, dPause, dHold)
            if nargin < 5, dHold = 0; end
            [p, w, pace] = app.nextPollInterval(uiCost, dSim, dPause, dHold);
        end

        function s = liveRatePacing(app)
            % What the pacing law is asking Simulink for, so a caller can check the solve
            % r = R * cycle / slice -- and the measured cycle behind the quoted frame rate.
            s = struct('on', app.PacingOn, 'rate', app.PacingRate, ...
                       'cycle', app.CycleEMA, 'achieved', app.AchievedRate, ...
                       'target', app.RateTarget);
        end

        function applyLiveRatePacing(app, pace)
            % Act on the decision stepLiveRate returned. The real loop hands it to
            % pacingEngage, which also pushes the rate at the model; a seam has no model to
            % push it at, and the law itself only ever reads whether pacing is engaged --
            % so this is the whole of that side effect, and the hysteresis in gradeLiveRate
            % can be driven round both directions without a simulation.
            app.pacingMark(logical(pace) && app.PaceLive);
        end

        % The blocking hold, exposed so a caller can confirm it withholds roughly the wall
        % clock it was asked for. What it cannot show is the part that matters -- that the
        % SOLVER gets none of it -- because there is no solver in a seam; only a live run
        % answers that.
        function dHold = holdLiveRate(app, w)
            dHold = app.holdSolver(w);
        end

        % Separate from stepLiveRate on purpose: the poll loop repaints the telemetry
        % text on a throttle, not every poll, so folding this into the step above
        % would have the seam do something the real loop does not.
        function refreshLiveRateLabel(app)
            app.refreshRateLabel();
        end

        % End-of-run summary, and the run totals behind it. A caller reads the stats to
        % close the time budget: sumU + sumP is what the controller accounts for, and the
        % wall clock minus that is what it does not.
        function finishLiveRate(app, wall, simSpan)
            app.summarizeLiveRate(wall, simSpan);
        end

        function st = liveRateStats(app)
            st = app.PollStats;
            st.wallRate = app.ShowRate;
            st.meanU = st.sumU / max(st.n, 1);
            % Per second of GRANTED time, not per second of wall clock: that is what S
            % means, and dividing by the hold as well would report the throttled figure
            % under the name of the unthrottled one.
            st.meanS = st.sumSim / max(st.sumP, eps);
        end

        % Set a formation type's colour without the modal picker. This is what the
        % Colour button calls once uisetcolor has returned, so a script drives exactly
        % the path the operator does.
        function setFormationColor(app, ftype, rgb)
            if ~isnumeric(rgb) || ~isequal(size(rgb), [1 3]) || ~all(isfinite(rgb))
                error('DroneLightShowApp:badColor', ...
                    'A formation colour must be a 1-by-3 finite RGB row.');
            end
            rgb = min(1, max(0, double(rgb)));
            ftype = round(ftype);
            if ftype < 1
                error('DroneLightShowApp:badFormationType', ...
                    'Formation types start at 1; got %d.', ftype);
            end

            % Grow rather than index past the end: type 13 is the ninth custom shape,
            % and nothing stops an operator from loading that many.
            if ftype > size(app.ColorOverrides, 1)
                app.ColorOverrides(end+1:ftype, :) = NaN;
            end
            app.ColorOverrides(ftype, :) = rgb;

            % Recolour the preview now, and be explicit that the FLOWN lighting signal
            % is a Generate away: lighting_timeline is built by setupParams, not here.
            names = app.formationNames();
            if ftype <= numel(names), shown = names{ftype}; else, shown = sprintf('formation type %d', ftype); end
            applied = false;
            if ~isempty(app.FormationColors)
                row = mod(ftype-1, size(app.FormationColors,1)) + 1;
                app.FormationColors(row, :) = rgb;
                app.LastFormType = [];        % force drawFrame to reassign colours
                app.drawCurrentFrame();
                applied = true;
            end
            app.refreshColorSwatch();
            if applied
                app.updateStatus(sprintf(['%s is now [%.2f %.2f %.2f]. The viewer is ' ...
                    'updated; the flown lighting signal follows at the next Generate.'], ...
                    shown, rgb(1), rgb(2), rgb(3)));
            else
                app.updateStatus(sprintf('%s will fly [%.2f %.2f %.2f] from the next Generate.', ...
                    shown, rgb(1), rgb(2), rgb(3)));
            end
        end

        % Load a formation from a file, without the dialog. The button calls
        % loadShapeFile, which asks for a path and then calls this; a caller that
        % already has a path -- a test, or a script setting up a canned show --
        % goes straight here. uigetfile is modal and blocks forever in a test.
        function name = addCustomFormation(app, filePath)
            name = app.registerFormation(formationFromMedia(filePath));
        end

        % Spell a string out in drones. Same deal as addCustomFormation: the button
        % reads the text field and calls this, and a test or a script that already
        % knows what it wants to fly calls it directly.
        function name = addTextFormation(app, str)
            name = app.registerFormation(formationFromText(str));
        end

        % Everything that happens once a formation EXISTS, whichever way it was
        % built: name it uniquely, add it to the list and the picker, say so, and
        % generate a show with it. Shared rather than duplicated because a text
        % formation has to arrive in the app in exactly the same state a loaded
        % picture does -- anything less and only one of the two would preview.
        function name = registerFormation(app, form)
            form.name = app.uniqueFormationName(form.name);
            if isempty(app.CustomFormations)
                app.CustomFormations = form;
            else
                app.CustomFormations(end+1) = form;
            end
            name = form.name;
            ftype = 4 + numel(app.CustomFormations);

            % Auto-add the two sequences worth having: the shape on its own, and the
            % shape between two Grids so there is a takeoff pattern to fly out of and
            % back into. Anything else the operator wants, they build from these.
            solo = name;
            sandwiched = sprintf('Grid→%s→Grid', name);
            app.FormationList.Items = [app.FormationList.Items, {sandwiched, solo}];
            app.FormationList.Value = sandwiched;

            % The picker is how a new formation gets built into a longer sequence, so
            % it has to learn the new name as well -- and it is selected, because what
            % was just added is what the operator is about to place.
            app.FormationPicker.Items = app.formationNames();
            app.FormationPicker.Value = name;
            % So does the colour picker: a shape that cannot be coloured is a shape
            % whose colour is whichever default row its type landed on.
            app.ColorFormDropdown.Items = app.formationNames();
            app.ColorFormDropdown.Value = name;
            app.refreshColorSwatch();
            % Assigning Value does not fire ValueChangedFcn, so the builder is told
            % explicitly that the show is now the sandwiched sequence.
            app.syncSequenceFromList();

            app.ShapeLabel.Text = sprintf('Added: %s', ...
                strjoin(arrayfun(@(s) sprintf('%s (%s)', s.name, s.kind), ...
                app.CustomFormations, 'UniformOutput', false), ', '));

            % Generate straight away. It is the only honest preview -- it runs the
            % real placement in setupParams and surfaces the real feasibility
            % warnings -- and it means the operator sees the shape in the viewer
            % rather than being told to press another button to find out.
            if strcmp(form.kind, 'text')
                verb = 'Spelling';
            else
                verb = 'Loaded';
            end
            app.updateStatus(sprintf('%s "%s" (%s). Generating a show with it...', ...
                verb, name, form.kind));
            drawnow;
            % generateShow applies the auto-fit itself (fitTransitionToPlan), so a
            % billboard too big to cross in the current transition is resolved here
            % rather than reported for the operator to retype.
            app.generateShow();
            app.previewFormation(ftype);
        end
    end

    methods (Access = private)

        %% ---- UI Creation ----
        function createUI(app)
            app.Fig = uifigure('Name', 'Multi-UAV Drone Light Show', ...
                'Position', [50 50 1400 850], ...
                'CloseRequestFcn', @(~,~) delete(app));

            topGrid = uigridlayout(app.Fig, [1, 2]);
            topGrid.ColumnWidth = {380, '1x'};
            topGrid.Padding = [5 5 5 5];
            topGrid.ColumnSpacing = 8;

            % Left panel: scrollable controls (fixed row heights exceed the
            % figure height, so the column scrolls rather than clipping)
            leftScroll = uigridlayout(topGrid, [9, 1]);
            % Fleet & Formation (row 1) is 310 rather than 190: it gained the
            % "Load Image / STL" button and the line naming the loaded shapes, and
            % at 190 the button was the last thing visible and the line was cut.
            % Then it gained two more rows for the sequence builder (the Add row and
            % the numbered sequence line), and at 240 "Custom shape" was clipped in
            % half and the loaded-shapes line was gone entirely. 310 leaves a little
            % slack on purpose: the loaded-shapes line wraps once several shapes are
            % named, and that wrap has to come out of somewhere. Then it gained the
            % Text field and its Fly Text button, so 310 -> 345.
            % (Rows 5-7 were "Trajectory Delivery", "Controls" and "Viewer & speed" for most
            % of the history below. They are now "Run the show", "Playback" and "Viewer" --
            % see the last paragraph for what moved where. The reasoning is kept under the
            % old names because that is what it was measured against.)
            % Trajectory Delivery (row 5) is 134 rather than 108: it gained the
            % delivery dropdown above the button, and at 108 the progress label
            % underneath the bar was clipped away entirely. Row 7 is 88 rather
            % than 55 because the speed slider now has the camera and drone
            % rendering dropdowns on a second line beneath it. Controls (row 6) is
            % 132 rather than 95: it grew a third row for Abort & Land, and at 95 all
            % three rows of buttons were squeezed to 22 px.
            % Navigation (row 3) is 168 rather than 112: 112 was measured when it had
            % three rows, and it now has five (the Degrade/Restore button row and the
            % Per-UAV loss field went in, and Denied went in under them). At 112 the
            % Denied readout was clipped away entirely -- the same failure as row 5's
            % progress label, and it is silent, because a uigridlayout that is too short
            % still takes the child.
            % Then Fleet & Formation gained the Colour picker row, so 345 -> 378.
            % Row 7 is 148 rather than 88: the two dropdowns on line 2 were joined by a
            % third line (Live rate + Show trails) and a wrapped hint line under it, and
            % at 88 the hint was gone entirely -- the same silent clipping as row 5's
            % progress label, because a uigridlayout that is too short still takes the
            % child. Telemetry (row 8) is 215 rather than 200 for the Live Rate readout.
            %
            % Then the relabelling pass moved three of these again, and it is worth saying
            % WHY each moved, because the amounts are not guesses:
            %   row 3  168 -> 196   GNSS / RTK gained a sixth row: the italic "Live readouts"
            %                       heading that separates the settings from the two readouts
            %                       underneath it. One label plus RowSpacing is ~21 px; 196
            %                       leaves slack.
            %   row 5  134 -> 180   Trajectory Delivery gained the static hint naming which
            %                       button the delivery dropdown enables. At 380 px wide and
            %                       FontSize 10 that text wraps to three lines, ~43 px.
            %   row 7  148 -> 186   Viewer & speed gained a panel TITLE where it previously
            %                       had none (~20 px of title bar), and its hint line grew a
            %                       sentence about the two rate controls, taking it from two
            %                       wrapped lines to three.
            % Controls (row 6) did NOT move: the Build/Run/Safety labels went into a fourth
            % COLUMN, so the row count is unchanged.
            %
            % Row 2 then moved too, 130 -> 164, and this one was a PRE-EXISTING bug rather than
            % fallout from the relabelling: Flight limits has always had a fifth row holding
            % SimModeNote, and at 130 that row overflowed the panel by 13 px empty and 25 px
            % once it holds text. SimModeNote is empty unless Sim Mode is Rapid Accelerator, so
            % the only thing ever clipped was the warning that the 3-D view will not move --
            % i.e. it was invisible in exactly the situation it exists to explain. Found by
            % driving the dropdown to Rapid Accelerator specifically to populate that note.
            %
            % Then Fleet & Formation gained the "Rotate step" row -- the step picker and its
            % degrees field -- so 378 -> 405. Measured, not guessed: at 378 the panel sat 4.0 px
            % inside with the rotation readout blank and overflowed by 8.2 px the moment Rotate
            % was ticked and the readout wrapped to two lines. The clipped control was the
            % LAST child, "Colour:", which is the thing worth remembering about this failure:
            % the overflow does not appear where the row was added, it pushes whatever happens
            % to be at the bottom off the panel, so the symptom never points at the cause. 405
            % leaves ~19 px of slack with the readout showing, in line with the other panels.
            %
            % Rows are pixels rather than 'fit' on purpose. The measurement behind that claim
            % was three panels with 4, 6 and 12 rows inside a Scrollable grid declared
            % {'fit','fit','fit'} all reporting the same height, which read as "'fit' does not
            % size to content here, it distributes like '1x'". Treat the reasoning as suspect:
            % identical heights across differently-sized panels is also the signature of
            % reading the geometry before the layout has settled, which happens readily here
            % (all eight panels report one identical height straight after construction, and
            % their real, distinct heights a fraction of a second later). The pixel values do
            % work, so they stay -- but if these ever need revisiting, re-test 'fit' properly
            % before concluding it cannot be used. Either way these numbers are the only
            % mechanism there is, and one being too small has to be caught by measuring the
            % laid-out panel against its content -- the failure is silent otherwise.
            % Row 4 then moved 132 -> 262: Radio link gained the italic "Live readouts"
            % heading and four readout rows naming the MAVLink message on the wire. Four
            % label rows plus RowSpacing measured ~24 px each and the heading ~22, i.e.
            % ~118 px, so 132 + 118 = 250 with slack to 262 in line with the other panels.
            % Rows 5-7 were {180, 132, 186} for Trajectory Delivery / Controls / Viewer &
            % speed, and are now {236, 132, 146} for Run the show / Playback / Viewer. The
            % three sum to 514 against the old 498, i.e. the merge cost 16 px overall -- the
            % two duplicated hint labels came out, one Fly row replaced two button rows, and
            % the slider moved rather than multiplied.
            %
            % These three were CHECKED with probePanelFit.m (tempdir), which is the check
            % this failure mode actually needs: it constructs the app, waits for the layout
            % to settle, then walks every descendant of every panel and flags any whose
            % extent falls outside the panel. Worth doing -- Playback was first written as
            % 124 and the probe caught its hint label hanging 3 px below the panel floor,
            % which is exactly the silent clip described above and would not have been
            % visible as anything but a slightly cropped last line of text. At 132 every
            % panel reports its children inside its bounds with the 21 px title bar as the
            % only slack.
            % Row 9, the status bar, is the ONE 'fit' row -- and it is the counter-example to
            % the paragraph above, so it was tested rather than assumed. It holds a single
            % WordWrap label, and a 'fit' row asks such a label for its wrapped height, which
            % is a real measurement of the text. Measured in the app: 16 px for a one-line
            % "Setup complete", 70 px for the 320-character big-fleet warning, 110 px for a
            % 460-character refusal. Any fixed number here is wrong in one direction or the
            % other -- 25 clipped every long message (which is how this started), and a fixed
            % worst case would leave ~90 px of blank under every short one.
            %
            % 'fit' failing the way the paragraph above describes would be unmissable rather
            % than silent: with the eight fixed rows already taller than the figure there is
            % no leftover space for a '1x'-like row to take a share of, so it would come out
            % 0 px and the status bar would simply not be there.
            leftScroll.RowHeight = {405, 164, 196, 262, 236, 132, 146, 215, 'fit'};
            leftScroll.Padding = [4 4 4 4];
            leftScroll.RowSpacing = 6;
            leftScroll.Scrollable = 'on';

            app.createFleetPanel(leftScroll);
            app.createFlightPanel(leftScroll);
            app.createNavPanel(leftScroll);
            app.createCommPanel(leftScroll);
            app.createRunPanel(leftScroll);
            app.createPlaybackPanel(leftScroll);
            app.createViewerPanel(leftScroll);
            app.createTelemetryPanel(leftScroll);
            app.createStatusBar(leftScroll);

            % Right side: a TAB GROUP, not a single panel. The 3-D view is tab 1 and stays
            % the selected tab, so the app opens looking exactly as it did; tab 2 is the
            % Guide. The reason it is a tab rather than a panel in the left column or a
            % separate window: the left column is already a scroll region nine panels deep
            % and prose in it would be unreadable, and a separate window would be a second
            % thing to manage that goes stale the moment it is closed. A tab is the only
            % place in this layout with room for full sentences.
            app.ViewerTabGroup = uitabgroup(topGrid);
            app.ViewerTab = uitab(app.ViewerTabGroup, 'Title', '3D Viewer', ...
                'BackgroundColor', [0.05 0.05 0.15]);
            rightGrid = uigridlayout(app.ViewerTab, [1, 1]);
            rightGrid.Padding = [0 0 0 0];
            app.ScenarioAxes = uiaxes(rightGrid);
            app.ScenarioAxes.Color = [0.05 0.05 0.15];
            app.ScenarioAxes.GridColor = [0.3 0.3 0.3];
            app.ScenarioAxes.XColor = [0.7 0.7 0.7];
            app.ScenarioAxes.YColor = [0.7 0.7 0.7];
            app.ScenarioAxes.ZColor = [0.7 0.7 0.7];
            xlabel(app.ScenarioAxes, 'North (m)');
            ylabel(app.ScenarioAxes, 'East (m)');
            zlabel(app.ScenarioAxes, 'Up (m)');
            view(app.ScenarioAxes, 35, 25);
            title(app.ScenarioAxes, 'Generate a show to begin', 'Color', [0.5 0.5 0.5]);

            app.createGuideTab();
        end

        function createGuideTab(app)
            % Tab 2: what every panel and every control in the left column is for.
            %
            % The organising question, and the one the tooltips could not answer on their
            % own, is WHEN a control takes effect. This app has three kinds and they look
            % identical: settings that are pushed to the model at Generate and do nothing
            % until you press it; live controls that retune a running simulation through a
            % workspace variable behind a Constant; and display controls that never reach
            % the model at all. Guessing wrong in either direction is the main way to
            % conclude the app is broken -- either you wait for a setting to act and it
            % never does, or you re-Generate for something that would have worked live.
            % So the tags come first, with their legend.
            %
            % Tagged BY EXCEPTION: the tag goes on the panel heading, and on a line only
            % where that line differs from its panel. Tagging every line was the first
            % attempt and it defeated itself -- most of the eight panels are uniform, so
            % Fleet & Formation carried nine identical GENERATE badges down the left margin
            % and the reader's takeaway from a badge that never varies is that it means
            % nothing. Now a badge in the body always marks something worth noticing: the
            % two buttons in GNSS/RTK that act immediately, the readouts, and "Run the show",
            % where every line is genuinely different from its neighbours.
            app.GuideTab = uitab(app.ViewerTabGroup, 'Title', 'Guide');
            gGrid = uigridlayout(app.GuideTab, [1, 1]);
            gGrid.Padding = [0 0 0 0];

            % uihtml, with the same fallback the upload bar uses: it is the only component
            % here that can lay out headings, a table and wrapped prose, but it is also the
            % one that can be unavailable. A uitextarea says the same words in plain text
            % and scrolls, so the Guide degrades rather than disappearing.
            try
                app.GuideHtml = uihtml(gGrid, 'HTMLSource', app.guideHTML());
            catch
                app.GuideHtml = uitextarea(gGrid, 'Value', app.guideText(), ...
                    'Editable', 'off', 'FontSize', 12);
            end
        end

        function html = guideHTML(app) %#ok<MANU>
            % The Guide, as one static HTML document. Static on purpose: a Guide that
            % re-rendered from the live component tree would be a second description of the
            % app maintained by code, and the failure mode of that is a Guide that is subtly
            % wrong rather than obviously absent. This is prose, reviewed alongside the panel
            % it describes.
            %
            % The tags are the point of the whole tab, and they are written by EXCEPTION --
            % on the heading, and on a line only where it differs. See createGuideTab.
            g = { ...
    '<!DOCTYPE html><html><head><meta charset="utf-8"><style>', ...
    'html,body{margin:0;padding:0;background:#fbfbfc;color:#222;', ...
    'font-family:Helvetica,Arial,sans-serif;font-size:13px;line-height:1.45;}', ...
    '#doc{padding:14px 18px 28px 18px;max-width:820px;}', ...
    'h1{font-size:17px;margin:0 0 2px 0;}', ...
    'h2{font-size:14px;margin:18px 0 5px 0;padding-bottom:3px;', ...
    'border-bottom:1px solid #dcdce0;color:#1a3a6b;}', ...
    'p.lead{margin:0 0 12px 0;color:#555;}', ...
    'dl{margin:0;} dt{font-weight:bold;margin-top:7px;}', ...
    'h2 .tag{margin-right:7px;vertical-align:2px;}', ...
    'p.byexc{margin:6px 0 0 0;color:#666;font-size:11px;}', ...
    'dd{margin:1px 0 0 0;padding-left:14px;color:#333;}', ...
    '.tag{display:inline-block;font-size:10px;font-weight:bold;padding:1px 5px;', ...
    'border-radius:3px;margin-right:5px;vertical-align:1px;letter-spacing:.3px;}', ...
    '.gen{background:#e3f0e6;color:#1d6b32;border:1px solid #b8dcc2;}', ...
    '.live{background:#fdeede;color:#96500a;border:1px solid #f0cfa6;}', ...
    '.disp{background:#e8ecf5;color:#33477a;border:1px solid #c4cfe6;}', ...
    '.out{background:#eceaf2;color:#4a3d6b;border:1px solid #cfc8de;}', ...
    '.legend{background:#fff;border:1px solid #dcdce0;border-radius:4px;', ...
    'padding:8px 11px;margin:0 0 4px 0;}', ...
    '.legend div{margin:3px 0;}', ...
    '.note{background:#fff8e8;border-left:3px solid #e0b458;padding:7px 10px;', ...
    'margin:9px 0;color:#4a3c1c;}', ...
    '</style></head><body><div id="doc">', ...
    '<h1>Guide</h1>', ...
    '<p class="lead">What every panel in the left column is for, and — the part that is ', ...
    'not visible from the controls themselves — <b>when each one takes effect</b>.</p>', ...
    '<div class="legend">', ...
    '<div><span class="tag gen">GENERATE</span> Controls that describe the show you want ', ...
    'to fly. Pressing <b>Generate</b> reads them all and works out the ', ...
    '<b>flight plan</b>: where each drone has to be at every instant from takeoff to ', ...
    'landing, which drone takes which slot in each formation, and the list of waypoints to ', ...
    'upload. Edit a control tagged this way and nothing happens until you press Generate ', ...
    'again — including during a run, which keeps flying the plan it started with.</div>', ...
    '<div><span class="tag live">LIVE</span> Takes effect the moment you use it, ', ...
    'including in the middle of a running simulation.</div>', ...
    '<div><span class="tag disp">DISPLAY</span> Changes only what you see. Never reaches ', ...
    'the flight plan or the simulation; never changes a flown number.</div>', ...
    '<div><span class="tag out">READOUT</span> Output, not input. The show is telling ', ...
    'you something.</div>', ...
    '<p class="byexc">Each heading below carries the tag that applies to the whole panel. ', ...
    'Individual lines are tagged only where they <b>differ</b> from their heading, so an ', ...
    'untagged line behaves the way its heading says.</p>', ...
    '</div>', ...
    '<div class="note"><b>The short version.</b> Set up the show, press ', ...
    '<b>Generate</b> to build the flight plan, then either <b>Simulate</b> or ', ...
    '<b>Upload &amp; Fly</b> — whichever the <b>Deliver</b> row leaves live. All three are ', ...
    'in the <b>Run the show</b> panel, in that order. Generate on its ', ...
    'own flies nothing: it plans the show and draws it. <b>Play</b> animates whatever data ', ...
    'exists — planned or flown — without flying anything either.</div>', ...
    ...
    '<h2><span class="tag gen">GENERATE</span>Fleet &amp; Formation — what the show is</h2>', ...
    '<p class="lead">The whole show is described here, and none of it exists until you press ', ...
    'Generate.</p>', ...
    '<dl>', ...
    '<dt>Number of UAVs</dt>', ...
    '<dd>4 to 500. Cost, not feasibility, is the reason to stay low: the MAVLink upload ', ...
    'is one mission item per waypoint per drone at one packet per tick, so it is linear ', ...
    'in the fleet and a large fleet spends minutes on the ground before it launches. A ', ...
    'warning appears on screen when you pick one.</dd>', ...
    '<dt>Formations / Add / the numbered sequence</dt>', ...
    '<dd>The list of shapes the show flies, in order. Repeats are allowed — Grid → ', ...
    'Circle → Grid is three steps. The numbered line beneath is the sequence as it will ', ...
    'actually be flown.</dd>', ...
    '<dt>Custom shape → Load Image / STL…</dt>', ...
    '<dd>Samples a picture or a mesh into a point cloud for the current fleet and adds it ', ...
    'to the Formations list as a new named shape. This one <b>re-Generates by itself</b>, ', ...
    'so the new shape is immediately flyable.</dd>', ...
    '<dt>Text + Fly Text</dt>', ...
    '<dd>Spells the string. "|" starts a second line — use it for anything long, since one ', ...
    'line of 13 characters is a letterbox with no height left to fly in. Reckon on ~10 ', ...
    'drones per character; too few and it looks like a bug rather than a small fleet. ', ...
    'Fly Text also <b>re-Generates by itself</b>.</dd>', ...
    '<dt>Colour</dt>', ...
    '<dd>Per formation <i>type</i>, not per step: a Grid → Circle → Grid show has two ', ...
    'colours and the two Grids cannot differ. The colour is <i>flown</i> rather than just ', ...
    'drawn — it becomes the lighting each drone is commanded to display, which is why it ', ...
    'lives here and not under Viewer.</dd>', ...
    '<dt>Spacing (m)</dt>', ...
    '<dd>Distance between neighbouring drones, so it sets the <i>size</i> of every shape. ', ...
    'Raising it makes the show physically larger and the transitions longer. It is also ', ...
    'what bounds the flown separation, so keep it well above d_min.</dd>', ...
    '<dt>Altitude (m)</dt>', ...
    '<dd>The height of the <i>lowest</i> drone — a floor, not a centre and not a ceiling. ', ...
    'Flat formations (Grid, Circle) all sit at it; a Sphere or a text billboard rests its ', ...
    'bottom on it and builds <i>upward</i>, so the show occupies this height plus the ', ...
    'height of the shape. A tall shape at a big fleet is tall: check the ceiling you are ', ...
    'allowed to fly to.</dd>', ...
    '<dt>Transition (s)</dt>', ...
    '<dd>Time to fly from one formation to the next. Generate <b>raises this by itself</b> ', ...
    'when the plan would exceed v_max, and says so in the status bar — so what you type is ', ...
    'a floor. It cannot improve separation: every drone flies the same fraction of its own ', ...
    'straight segment, so the paths are the same shape however long you take over them.</dd>', ...
    '<dt>Hold (s)</dt>', ...
    '<dd>Time spent in each formation. Dead time for the planner unless <b>Rotate</b> is ', ...
    'ticked — it otherwise lengthens the show and the simulation without adding any ', ...
    'flying. With Rotate on it is also what decides how far each formation gets to turn.</dd>', ...
    '<dt>Rotate + Rotate step</dt>', ...
    '<dd>Spins a formation about its own vertical axis while the fleet holds it. Every ', ...
    'drone keeps its altitude and circles the formation centre at <i>its own</i> radius, so ', ...
    'the outer drones sweep wide and the middle barely moves — a Sphere turns as a ball, a ', ...
    'Grid as a turntable, a text billboard goes edge-on and back. It is a <b>plan</b> ', ...
    'setting: the rotation is uploaded as waypoints and flown, not drawn on top.', ...
    '<p><b>Which</b> formations turn and <b>how far</b> is set on the Rotate step row: pick ', ...
    'a step of the sequence and give it an angle in degrees. 0 leaves that formation still, ', ...
    'and a negative angle turns the other way — so a Grid → Circle → Grid show can spin the ', ...
    'first Grid one way, leave the Circle alone and turn the last Grid back. Rotation is per ', ...
    '<i>step</i>, unlike Colour, which is per formation type and so cannot tell two Grids ', ...
    'apart. The checkbox is the master mute: unticking it flies a still show without ', ...
    'discarding the angles you typed.</p>', ...
    '<p>Ask for an angle <i>inside</i> what the hold can deliver and it is flown ', ...
    '<b>exactly</b> — 20° means 20.00°. That is the practical reason to set an angle rather ', ...
    'than ask for a full turn, which almost never fits:</p>', ...
    '<p>The <i>planned</i> separation is safe by construction — a rigid rotation preserves ', ...
    'every distance exactly, so a rotating hold cannot breach d_min if the still formation ', ...
    'clears it. That guarantee covers the plan and stops there: drones at different radii ', ...
    'fly at different speeds, so if the spin is faster than they can hold they lag by ', ...
    'different amounts and the flown formation shears even though the commanded one is ', ...
    'rigid. Which is why the sweep is capped below.</p>', ...
    '<p>What it costs is <b>sweep</b>. Holding a drone at radius r on its circle needs a ', ...
    'centripetal ω²·r for as long as the turn lasts — not just once, continuously — and ', ...
    'that runs out of acceleration budget well before tangential speed ω·r runs out of ', ...
    'v_max. So a full turn is usually not possible in the hold you have: a 12-drone Grid at ', ...
    '5 m spacing sweeps about 100° in a 10 s hold and would need roughly 32 s for one full ', ...
    'revolution, and bigger fleets need longer still because the radius grows. Rather than ', ...
    'command a circle the drones would spiral out of, the plan sweeps as far as it can ', ...
    'hold, and the grey line under the field reports what it managed against what you ', ...
    'asked for, which limit stopped it, and what hold the full angle would need. It also ', ...
    'costs upload time, but only for the formations that turn: a still hold uploads as two ', ...
    'waypoints, a rotating one has to be sampled along its arc.</p></dd>', ...
    '</dl>', ...
    ...
    '<h2><span class="tag gen">GENERATE</span>Flight limits — the envelope the plan is ', ...
    'sized against</h2>', ...
    '<p class="lead">All three <b>check</b> the plan; none of them clips a drone in flight. ', ...
    'If the plan violates them, the Transition auto-fit lengthens the transition and tells ', ...
    'you.</p>', ...
    '<dl>', ...
    '<dt>v_max (m/s), a_max (m/s²)</dt>', ...
    '<dd>Speed and acceleration the fleet is expected to track. The min-jerk profile peaks ', ...
    'well above its average, so a plan that looks gentle on paper can still exceed ', ...
    'a_max. Both are checked against the finished plan, and a_max does double duty: it is ', ...
    'also the ceiling the position controller clamps its commanded lateral acceleration to, ', ...
    'so it is a real limit in flight, and it is the budget <b>Rotate</b> sizes its sweep ', ...
    'inside. <b>Acceleration is usually the one that binds, not speed.</b> The two scale ', ...
    'differently with Transition (s) — speed as 1/T, acceleration as 1/T² — so shortening a ', ...
    'transition raises the acceleration four times as fast as the speed, and there is a wide ', ...
    'band of durations that are legal on speed and illegal on acceleration. The auto-fit ', ...
    'sizes for whichever binds.</dd>', ...
    '<dt>d_min (m)</dt>', ...
    '<dd>Required separation. The geometry is sized against it, and the transition ', ...
    'assignment guarantees the flown separation stays above the tighter formation''s ', ...
    'spacing ÷ √2 — so raising d_min makes formations <i>bigger</i>, not slower.</dd>', ...
    '<dt>Sim Mode</dt>', ...
    '<dd>Normal is the default and the safe choice. <b>Accelerator</b> is ~18% faster with ', ...
    'the live 3-D view fully intact. <b>Rapid Accelerator</b> is much faster again and ', ...
    'numerically identical, but it runs as a separate executable, so the 3-D view and the ', ...
    'progress bars stay frozen — it is for long batch runs nobody is watching. A warning ', ...
    'appears in the panel when you select it.</dd>', ...
    '</dl>', ...
    ...
    '<h2><span class="tag gen">GENERATE</span>GNSS / RTK — breaking the corrections on ', ...
    'purpose</h2>', ...
    '<p class="lead">This panel is the interesting one. The base station sends RTK ', ...
    'corrections; a drone that stops receiving them degrades <b>RTK Fix (2 cm) → Float ', ...
    '(0.3 m) → Standalone (1.5 m)</b> as its last correction ages, and stops there. A ', ...
    'degraded drone drifts and spoils the formation — it does not fly away.</p>', ...
    '<dl>', ...
    '<dt>Degrade UAV(s)</dt>', ...
    '<dd>Which drones to target: one number, a list (<code>1 3 5</code> or ', ...
    '<code>1,3,5</code>), a range (<code>1:5</code>), or <code>0</code> for the whole ', ...
    'fleet. Numbers past the fleet size are dropped with a note. This field only says ', ...
    '<i>who</i> — the buttons below are what act.</dd>', ...
    '<dt><span class="tag live">LIVE</span>Degrade / Restore</dt>', ...
    '<dd>Press Degrade and those drones lose their corrections from that instant, before ', ...
    'a run or in the middle of one. Restore gives them back, and the error does not vanish ', ...
    '— it bleeds off over the reconverge time, like a real receiver re-fixing.</dd>', ...
    '<dt>Per-UAV loss (%)</dt>', ...
    '<dd>Each drone independently misses this fraction of its corrections. Unlike Packet ', ...
    'Loss under Radio link — which erases the frame for everyone at once — this ', ...
    'decorrelates the fleet, so drones degrade at different times.</dd>', ...
    '<dt><span class="tag out">READOUT</span>RTK tier</dt>', ...
    '<dd>The <i>worst</i> tier any drone is on right now.</dd>', ...
    '<dt><span class="tag out">READOUT</span>Denied</dt>', ...
    '<dd>Which drones are currently denied their corrections.</dd>', ...
    '</dl>', ...
    '<div class="note"><b>Which drone is which?</b> A denied drone gets a <b>white ring</b> ', ...
    'around it in the 3-D view, and is drawn red as well. Marking is the only way to tell — ', ...
    'formation slots are re-assigned by cost at every transition, so drone 3 sits somewhere ', ...
    'different in every formation and there is no position to learn. Name a drone, press ', ...
    'Degrade, and it is ringed straight away — no need to press Play first. The ring is what ', ...
    'to look for rather than the colour: you can set any formation to any colour from the ', ...
    'Colour picker, and the first formation is red by default, so red on red would be no ', ...
    'marking at all.</div>', ...
    ...
    '<h2><span class="tag gen">GENERATE</span>Radio link — the channel to the fleet</h2>', ...
    '<p class="lead">One shared radio channel, carrying the mission upload out and the ', ...
    'telemetry back.</p>', ...
    '<dl>', ...
    '<dt>Latency (ms) / Jitter (ms)</dt>', ...
    '<dd>One-way delay, and the random variation added to it frame by frame.</dd>', ...
    '<dt>Packet Loss (%)</dt>', ...
    '<dd>Fraction of frames erased outright, for <b>every</b> drone at once — this is the ', ...
    'shared link failing. It goes to 90% deliberately: high loss is what drives the ', ...
    'pre-flight upload into failure, so the retry path is reachable from here.</dd>', ...
    '<dt>Timeout (s)</dt>', ...
    '<dd>How long the base station waits for a mission acknowledgement before retrying ', ...
    'the item.</dd>', ...
    '</dl>', ...
    ...
    '<h2>Run the show — one panel, walked top to bottom</h2>', ...
    '<p class="lead">The only panel with no single answer to "when does this take effect", ', ...
    'because most of what is in it <i>are</i> the actions. It is numbered because the order ', ...
    'is real: <b>1 Plan</b> the show, <b>2 Deliver</b> the waypoints, <b>3 Fly</b> it. Step 2 ', ...
    'is what decides which of the two Fly buttons is live — they sit side by side with exactly ', ...
    'one of them enabled, so the either/or is on screen rather than described. <b>Halt</b> is ', ...
    'below the rule because neither of its buttons is part of the sequence.</p>', ...
    '<dl>', ...
    '<dt><span class="tag gen">GENERATE</span>1 Plan → Generate</dt>', ...
    '<dd>Start here, and press it again after any change you want flown. It reads every ', ...
    'setting in this column and works the show out as a <b>trajectory</b>: takeoff, which ', ...
    'drone takes which slot in each formation, the path each one flies between formations, ', ...
    'the rotation during the holds, and the landing — then it draws the result in the 3-D ', ...
    'view and hands the waypoints to whichever delivery path you chose. It <i>plans</i> ', ...
    'only: no flight dynamics, no radio, no GNSS error, so the drones sit exactly where the ', ...
    'plan says. Flying it is step 3. Nothing else in this panel works until you have ', ...
    'pressed it.</dd>', ...
    '<dt><span class="tag disp">DISPLAY</span>1 Plan → Reset</dt>', ...
    '<dd>Clears the plan, the logged run and the scene, back to the state before the first ', ...
    'Generate. Your settings are kept.</dd>', ...
    '<dt><span class="tag gen">GENERATE</span>2 Deliver — how the waypoints reach the ', ...
    'drones</dt>', ...
    '<dd>Both paths fly the same keyframes, so a healthy link gives the same show either ', ...
    'way. The difference is whether the radio is in the loop — and, because of that, which ', ...
    'pose the viewer can show you. The two choices are below.</dd>', ...
    '<dt>MAVLink upload (full fidelity)</dt>', ...
    '<dd>Streams the mission over the radio, verifies every waypoint landed onboard, then ', ...
    'flies it from the onboard buffer in one continuous run. Enables <b>Upload &amp; ', ...
    'Fly</b> and greys out Simulate. This is the path the protocol detail exists for.<br>', ...
    'The radio is also in the loop on the way <i>back</i>: the 3-D view shows the pose the ', ...
    'ground station <b>received over MAVLink</b> — each drone''s own navigation estimate, ', ...
    'packed into a telemetry message, sent down a shared radio and decoded here — not the ', ...
    'true airframe position. The title reads <i>[Live - MAVLink downlink]</i> while it does. ', ...
    'So during the pre-flight you will see the fleet drift metres off its pad while the RTK ', ...
    'engine is still code-only, then snap to centimetres the moment it fixes; that gap is ', ...
    'the navigation error, and watching it converge is the point of this mode. Telemetry is ', ...
    'round-robin, one drone per slot, so a drone that has not reported yet is drawn on its ', ...
    'pad and each drone''s marker steps rather than glides.</dd>', ...
    '<dt>Workspace (quick)</dt>', ...
    '<dd>Feeds the controller directly and skips the pre-flight entirely. Enables ', ...
    '<b>Simulate</b> and greys out Upload &amp; Fly. Use it while iterating on the ', ...
    'show. The 3-D view here shows the <b>true</b> airframe position straight off the flight ', ...
    'dynamics, titled <i>[Live - direct]</i>: no radio, no telemetry, no estimator in the ', ...
    'way. That is the mode to use when you want to know where the fleet actually went.</dd>', ...
    '<dt><span class="tag live">LIVE</span>3 Fly → Upload &amp; Fly</dt>', ...
    '<dd>Streams MISSION_ITEM_INT to the fleet, verifies every waypoint landed onboard, then ', ...
    'flies the show from the onboard buffer in the same run. Live when Deliver is MAVLink; ', ...
    'greyed otherwise, because there is no upload on the other path.</dd>', ...
    '<dt><span class="tag live">LIVE</span>3 Fly → Simulate</dt>', ...
    '<dd>Flies the plan through the full model — dynamics, GNSS noise, radio link — and ', ...
    'streams it to the view. The result replaces the planned trajectory, so Play ', ...
    'afterwards shows what was actually flown rather than what was intended. Live when ', ...
    'Deliver is Workspace; Upload &amp; Fly does the same job on the MAVLink path.</dd>', ...
    '<dt><span class="tag out">READOUT</span>3 Fly → lamp, progress bar, status line</dt>', ...
    '<dd>Grey = not uploaded, green = confirmed onboard, red = gave up. The bar fills as ', ...
    'items are acknowledged, so it stalls visibly when the link is bad.</dd>', ...
    '<dt><span class="tag live">LIVE</span>Halt → Stop</dt>', ...
    '<dd>Halts whatever is moving, playback or simulation. The drones are left wherever ', ...
    'the last frame put them. It stays in this panel rather than moving to Playback with ', ...
    'Play, because a <i>run</i> is the thing you urgently need to stop.</dd>', ...
    '<dt><span class="tag live">LIVE</span>Halt → Abort &amp; Land</dt>', ...
    '<dd>The only button in the app that changes what the aircraft do: abandons the show and ', ...
    'brings the fleet straight down from wherever it is. The run <i>keeps going</i> until ', ...
    'they are parked — this lands them, it does not stop the sim. Stop is what stops the ', ...
    'sim, and it leaves them in the air.</dd>', ...
    '</dl>', ...
    ...
    '<h2><span class="tag disp">DISPLAY</span>Playback — redraws data that already ', ...
    'exists</h2>', ...
    '<p class="lead">Nothing in here flies, plans or computes anything: all three controls ', ...
    'redraw a trajectory that has already been worked out, which is why they are together and ', ...
    'why the whole panel is one DISPLAY tag. There is <b>one</b> speed control in the app and ', ...
    'it is this one, pacing <i>replay</i>. A simulation that is streaming is always paced to ', ...
    '<b>real time</b> — that is not a setting, and it is deliberate: the show is meant to take ', ...
    'as long to watch as it takes to fly. If you want it over with sooner, that is a Sim Mode ', ...
    'question, not a speed one (Rapid Accelerator, which is explicit that there is no live ', ...
    'view).</p>', ...
    '<dl>', ...
    '<dt>Play / Pause</dt>', ...
    '<dd>One button, two states: it animates data that already exists — the planned ', ...
    'trajectory, or the log of a finished run — and reads <b>Pause</b> while it does. ', ...
    'Nothing is flown and nothing is recomputed. Pressing it at the end of the show starts ', ...
    'again from the top. The button that <i>runs</i> the model is Simulate, in Run the ', ...
    'show.</dd>', ...
    '<dt>The bar beside it</dt>', ...
    '<dd>Where you are in the show, and a way to go anywhere else in it: drag and the fleet ', ...
    'moves with the thumb, frame by frame, with the time and the length of the show read out ', ...
    'to the right. Dragging while it plays pauses it for the drag and carries on from where ', ...
    'you let go. It goes flat while a simulation is streaming, because the frames ahead of ', ...
    '"now" have not been computed yet. This replaced a <b>Restart</b> button, which was one ', ...
    'position on this bar — the left-hand end — given a control of its own.</dd>', ...
    '<dt>Playback speed (0.25x – 4x)</dt>', ...
    '<dd>How fast Play animates. <b>1.0x is real time</b> — one second of show ', ...
    'per second of wall clock. It applies to the planned trajectory and to a logged ', ...
    'simulation equally, so yes: you can replay a finished run at real time, slower, or ', ...
    'faster. It does nothing while a simulation is streaming. It is also the smooth way to ', ...
    'watch a show: fly it once, then replay the log.</dd>', ...
    '<dt>Why a streaming run is steppier than a replay</dt>', ...
    '<dd>Roughly 8 fps on a small fleet, about 3 fps at 120 drones, and there is no setting ', ...
    'for it. The model flies the show 2 to 5x faster than real time, so holding it to 1x means ', ...
    'withholding wall clock — and the only question is whether the drawing gets any of it. ', ...
    'Simulink''s own pacing makes the solver wait <i>between</i> steps, which shortens the ', ...
    'chunk it hands back to about 60 ms of show and leaves room for a frame in each; the ', ...
    'fallback, used where the model will not accept pacing, waits <i>after</i> the chunk ', ...
    'instead and gets about 3 fps for the same 1x. On a large fleet the two converge, because ', ...
    'pacing can only reclaim the <i>solver''s</i> share of a cycle and 120 drones as meshes ', ...
    'cost ~0.3 s a frame to draw — there the lever is <b>Drones</b> (a scatter mode), not the ', ...
    'throttle. Either way <b>Live Rate</b> reports the factor and the frame rate actually ', ...
    'achieved. For a genuinely smooth watch, use Play afterwards.</dd>', ...
    '</dl>', ...
    ...
    '<h2><span class="tag disp">DISPLAY</span>Viewer — nothing here changes a flown ', ...
    'number</h2>', ...
    '<p class="lead">How the show is drawn, and which pose is drawn. The speed slider used to ', ...
    'live here; it is in Playback now, next to the button it paces.</p>', ...
    '<dl>', ...
    '<dt>Camera (Static / Dynamic)</dt>', ...
    '<dd>Dynamic tracks the swarm centroid and zooms to its spread; Static leaves the view ', ...
    'fixed.</dd>', ...
    '<dt>Drones (meshes / Spheres / Markers only)</dt>', ...
    '<dd>Meshes draw the real quadrotor geometry and look far better, at 792 vertices per ', ...
    'drone rewritten every frame — which is what makes a large fleet crawl. Above about ', ...
    '100 drones use one of the scatter modes. Measured at 50 drones the two scatter modes ', ...
    'cost the same, so pick between them on looks.</dd>', ...
    '<dt>Pose shown (As received / True airframe pose)</dt>', ...
    '<dd>MAVLink delivery only; greyed out in Workspace mode, which has no downlink. ', ...
    '<b>As received</b> draws the ground station''s telemetry table. The fleet <i>takes ', ...
    'turns</i> on the link — each drone reports its own pose in a recurring slot, one drone ', ...
    'per time step — so a row is exactly as fresh as that drone''s last slot and the whole fleet ', ...
    'refreshes once every N × 0.01 s: 0.2 s at 20 drones, 5 s at 500. Above a few dozen ', ...
    'drones that shows as a wave crossing the fleet. It is the modelled radio, not a viewer ', ...
    'artefact, and it is what a real operator sees. <b>True airframe pose</b> taps the flight ', ...
    'dynamics instead: every drone, every frame, no radio in the way. Use it to judge the ', ...
    'flight, As received to judge the link. Neither changes what flies, and switching takes ', ...
    'effect on the next run.</dd>', ...
    '<dt>Show trails</dt>', ...
    '<dd>The streak behind each drone — where it has <i>been</i>. It costs one update ', ...
    'per drone per frame, so what it costs depends on the fleet: measured at 4x, ', ...
    '<b>+17%</b> per frame on ten drones but <b>+81%</b> on sixty. That makes it the first ', ...
    'thing to turn off on a big fleet, and barely worth turning off on a small one.</dd>', ...
    '<dt>Show trajectory</dt>', ...
    '<dd>The other half of that pair: where every drone is <i>going</i>. It draws the ', ...
    'planned path of the whole fleet for the phase on screen — the transition being flown, ', ...
    'or, while the fleet holds a formation, the one it is about to fly — so it changes by ', ...
    'itself as the show moves on: grid, then grid → circle, then circle → the next shape, ', ...
    'and the descent at the end. It is always the <b>plan</b>, never the log, which is why ', ...
    'it works during a live run as well and why a gap between a drone and its line is worth ', ...
    'looking at — that gap is tracking error. Cheaper than trails, too: one line object for ', ...
    'the whole fleet, rebuilt when the phase changes rather than every frame. A run that has ', ...
    'just finished leaves the fleet landed on screen, and a landed fleet has no next move, so ', ...
    'the overlay is blank there until you press Play or move the bar.</dd>', ...
    '</dl>', ...
    ...
    '<h2><span class="tag out">READOUT</span>Telemetry — all output, no settings</h2>', ...
    '<dl>', ...
    '<dt>Time, Phase, Formation, Show State</dt>', ...
    '<dd>Where the show is: elapsed against total, the supervisor''s phase (takeoff, show, ', ...
    'landing, landed), which formation is current, and the state machine''s state.</dd>', ...
    '<dt>Min Separation</dt>', ...
    '<dd>Closest pair in the fleet at this instant. This is the number d_min is about — ', ...
    'watch it during transitions, which is where it is smallest.</dd>', ...
    '<dt>Max Speed</dt>', ...
    '<dd>Fastest drone at this instant. Compare against v_max.</dd>', ...
    '<dt>Live Rate</dt>', ...
    '<dd>Real-time factor the live stream is actually achieving, against the 1x it is aiming ', ...
    'for. Worth watching: on a large fleet the model cannot reach real time, and this says ', ...
    'so rather than pretending. Blank outside a streamed run, because replay is paced by ', ...
    'Playback speed instead and reporting a number here would mean something else.</dd>', ...
    '</dl>', ...
    '</div></body></html>'};
            html = strjoin(g, newline);
        end

        function txt = guideText(app) %#ok<MANU>
            % Plain-text Guide for the uitextarea fallback. Deliberately shorter than the
            % HTML: it exists so the tab is not empty where uihtml is unavailable, and a
            % 200-line wall of unformatted text would not be read anyway. It keeps the parts
            % that are load-bearing -- the four tags, what the one speed control does and does
            % not pace, the either/or between Simulate and Upload, and how a degraded drone is
            % marked.
            %
            % The tag is written on the SECTION here rather than on every line, which is where
            % the HTML version got the same idea: most of its eight panels are uniform, so a
            % badge on each row was a column of identical badges saying nothing. Both now tag
            % the panel and tag a line only where it differs. Run the show is the one panel
            % that still needs per-line tags, because its rows genuinely differ -- Generate
            % plans, Reset only redraws, and the Fly and Halt rows act immediately.
            txt = { ...
    'GUIDE'
    ''
    'Four tags, because the controls all look alike:'
    '  GENERATE - controls that describe the show you want to fly. Pressing'
    '             Generate reads them all and works out the FLIGHT PLAN:'
    '             where each drone has to be at every instant from takeoff to'
    '             landing, which drone takes which slot in each formation, and'
    '             the waypoints to upload. Edit one of these and nothing happens'
    '             until you press Generate again; a run already going keeps'
    '             flying the plan it started with.'
    '  LIVE     - takes effect the moment you use it, including mid-simulation.'
    '  DISPLAY  - changes only what you see. Never reaches the plan.'
    '  READOUT  - output, not input. The show is telling you something.'
    '  A tag on a section applies to every line in it; individual lines are'
    '  tagged only where they differ.'
    ''
    'THE SHORT VERSION'
    '  Set up the show, press Generate to build the flight plan, then either'
    '  Simulate or Upload & Fly, whichever the Deliver row leaves live. All three'
    '  are in RUN THE SHOW, in that order.'
    '  Generate itself flies nothing - it plans the show and draws it. Play'
    '  animates whatever data exists, planned or flown, without flying anything.'
    ''
    'FLEET & FORMATION [GENERATE] - what the show is, then the numbers that size'
    '  and time it. First the fleet size, the ordered list of shapes, the shapes'
    '  you can add to it (Load Image/STL, Fly Text - both re-Generate on their'
    '  own) and their colours; then, at the bottom, their size (Spacing), the'
    '  height of the LOWEST drone (Altitude - a floor, not a centre: a Sphere or'
    '  a text billboard builds upward from it), and the time spent flying between'
    '  them (Transition) and parked in them (Hold).'
    '  Generate RAISES Transition by itself if the plan would exceed v_max.'
    ''
    '  ROTATE spins each formation about its own vertical axis during the hold.'
    '  Each drone keeps its altitude and circles the formation centre at its own'
    '  radius, so the outside sweeps wide and the middle barely moves. The PLANNED'
    '  separation is safe by construction - a rigid rotation preserves every'
    '  distance - but that says nothing about the flown one, which shears if the'
    '  spin outruns what the drones can hold. Hence the cap. What is limited is'
    '  SWEEP, and by ACCELERATION rather than speed: staying on a circle of radius'
    '  r needs a centripetal w^2*r continuously, which runs out sooner than the'
    '  ~7 m/s speed limit does. A full turn usually needs a longer Hold than you'
    '  have (about 32 s for a 12-drone Grid at 5 m). The plan sweeps what it can'
    '  and the grey line under the field says how far that was, which limit bit,'
    '  and what a full turn would need.'
    ''
    'FLIGHT LIMITS [GENERATE] - v_max and d_min CHECK the plan; they do not clip a'
    '  drone in flight. a_max is both: it is checked against the plan AND it is the'
    '  ceiling PositionController clamps commanded lateral acceleration to, so it'
    '  is the budget a rotating hold is sized inside. Acceleration is usually the'
    '  limit that binds, not speed: it scales as 1/T^2 against the transition time'
    '  where speed scales as 1/T, so plenty of plans are legal on speed and well'
    '  over on acceleration. The auto-fit sizes for whichever binds. Sim Mode: Accelerator is'
    '  ~18% faster with the live'
    '  view intact; Rapid Accelerator is faster still and exact, but the 3-D view'
    '  and progress bars stay frozen.'
    ''
    'GNSS / RTK - the place you break the corrections. A drone that stops'
    '  receiving them degrades RTK Fix (2 cm) -> Float (0.3 m) -> Standalone'
    '  (1.5 m) and stops there: it drifts and spoils the formation, it does not'
    '  fly away. Degrade UAV(s) [GENERATE] says WHO (one number, 1 3 5, 1:5, or 0'
    '  for all). Degrade/Restore [LIVE] act immediately, mid-run included.'
    '  Per-UAV loss [GENERATE] decorrelates the fleet, unlike Packet Loss which'
    '  erases the frame for everyone at once.'
    ''
    '  WHICH DRONE IS WHICH: a denied drone gets a WHITE RING around it in the 3-D'
    '  view, and is drawn red as well. Marking is the only way to tell - formation'
    '  slots are re-assigned by cost at every transition, so there is no fixed'
    '  position per index to learn. The marking appears as soon as you press'
    '  Degrade; you do not have to press Play. Watch the ring rather than the'
    '  colour: the first formation is red by default, so red on red shows nothing.'
    ''
    'RADIO LINK [GENERATE] - one shared channel: Latency, Jitter, Packet Loss'
    '  (everyone at once), and the mission-ack Timeout.'
    ''
    'RUN THE SHOW - every line is tagged here, because most of these are the'
    '  actions. Numbered, because the order is real: 1 Plan (Generate, Reset),'
    '  2 Deliver (the dropdown), 3 Fly (Upload & Fly | Simulate), then Halt'
    '  (Stop, Abort & Land) below the rule because it is not part of the'
    '  sequence. Play and the scrub bar are their own PLAYBACK panel.'
    '  Generate [GENERATE] PLANS the show - trajectory, slot assignment, the 3-D'
    '  scene - and flies nothing. Reset [DISPLAY] clears the plan and the scene,'
    '  keeping your settings.'
    '  2 DELIVER [GENERATE] decides WHICH of the two Fly buttons is live. They'
    '  sit SIDE BY SIDE with exactly one enabled, and the grey line at the'
    '  bottom of the panel says which and why. MAVLink upload enables Upload &'
    '  Fly and greys out Simulate; Workspace does the reverse. Both fly the same'
    '  keyframes - the difference is whether the radio is in the loop.'
    '  IT ALSO SETS WHAT THE 3-D VIEW SHOWS BY DEFAULT. MAVLink upload draws the'
    '  pose the ground station RECEIVED - each drone''s own estimate, packed into a'
    '  telemetry message and decoded here - titled [Live - MAVLink downlink]. So'
    '  the fleet drifts metres off the pad while the RTK engine is code-only and'
    '  snaps to centimetres when it fixes: that gap is the navigation error, and'
    '  seeing it converge is what the mode is for. Pose shown, in VIEWER,'
    '  overrides it with the true airframe pose if you would rather watch the'
    '  flight than the link. Workspace always draws the TRUE airframe position'
    '  off the flight dynamics, titled [Live - direct]; it has no downlink.'
    '  3 FLY [LIVE] - Upload & Fly streams the mission and flies it from the'
    '  onboard buffer in one run; Simulate feeds the controller directly. Either'
    '  way the dynamics, the radio and the GNSS error are in the loop.'
    '  HALT [LIVE] - Abort & Land is the only button in the app that changes what'
    '  the aircraft do: it brings them down and the run KEEPS GOING until they'
    '  are parked. Stop halts the run instead and leaves them in the air. Stop'
    '  stays here rather than in Playback because a RUN is what you urgently'
    '  need to stop; it halts playback too.'
    ''
    'PLAYBACK [DISPLAY] - the three controls that only ever REDRAW data that'
    '  already exists. Nothing here flies, plans or computes.'
    '  Play / Pause  one button, two states: it animates the planned trajectory'
    '                or the log of a finished run, and reads Pause while it does.'
    '                From the end of the show it starts again from the top. The'
    '                button that RUNS the model is Simulate.'
    '  The bar       where you are, and a way to go anywhere else: drag and the'
    '                fleet moves with the thumb, with the show time and length'
    '                read out beside it. Dragging while it plays pauses it for the'
    '                drag and carries on from where you let go. Flat during a'
    '                streaming run - the frames ahead of "now" do not exist yet.'
    '                It replaced a Restart button, which was one position on this'
    '                bar (the left-hand end) given a control of its own.'
    '  Playback speed is the ONE speed control in the app and it paces REPLAY'
    '                (planned OR logged). 1.0x is real time. Does nothing while'
    '                streaming.'
    '  A streaming run is always paced to real time - not a setting. It draws at'
    '  ~8 fps on a small fleet: Simulink pacing makes the solver wait BETWEEN'
    '  steps, cutting the chunk it yields to ~60 ms of show so a frame fits in'
    '  each. That reclaims the SOLVER''s share of a cycle and nothing else, so at'
    '  120 drones, where drawing costs ~0.3 s a frame, it is back to ~3 fps and'
    '  the lever is the render mode. Fly it once, then Play the log. To finish a'
    '  run sooner, use Sim Mode (Rapid Accelerator), which drops the live view.'
    ''
    'VIEWER [DISPLAY] - how the show is drawn, and which pose is drawn. Above'
    '  ~100 drones prefer a scatter mode over meshes, and turn trails off'
    '  - they cost an update per drone per frame, +81% at 60 drones.'
    ''
    '  POSE SHOWN - MAVLink mode only. As received draws the ground'
    '  station telemetry table. The fleet TAKES TURNS on the link, each drone'
    '  self-reporting in a recurring slot, one drone per time step, so the'
    '  whole fleet refreshes every N x 0.01 s (0.2 s at 20 drones, 5 s at 500)'
    '  and a big fleet visibly updates in a wave. That is the modelled radio and'
    '  it is what a real operator sees. True airframe pose taps flight dynamics:'
    '  every drone, every frame. Judge the FLIGHT with one, the LINK with the'
    '  other. Neither changes what flies; it applies from the next run.'
    ''
    '  SHOW TRAILS / SHOW TRAJECTORY - the same choice twice, one each way.'
    '  Trails are where each drone HAS BEEN, a streak per drone grown a point'
    '  per frame, so the cost scales with the fleet: measured at 4x, +17% a'
    '  frame at 10 drones and +81% at 60. Show trajectory'
    '  is where the fleet is GOING: every drone''s planned path for the phase on'
    '  screen - the transition being flown, or the next one while the fleet'
    '  holds - so it follows the show by itself (grid, grid->circle,'
    '  circle->next, then the descent). Always the PLAN, so it works during a'
    '  live run too, and a gap between a drone and its line IS the tracking'
    '  error. One line object for the whole fleet, rebuilt once a phase.'
    ''
    'TELEMETRY - all readouts. Min Separation is the number d_min is about;'
    '  watch it during transitions. Live Rate is blank outside a streamed run.'
    };
        end

        function createFleetPanel(app, parent)
            panel = uipanel(parent, 'Title', 'Fleet & Formation');
            % FOURTEEN rows: the rotation readout went in under the Hold row (Rotate
            % itself shares Hold's row, so it costs nothing here), and per-formation
            % angles added one more for the step picker. A uigridlayout declared too
            % short still accepts the extra child and silently drops it off the visible
            % area, so the count has to be kept in step by hand -- nothing warns when it
            % is not.
            %
            % ORDER: WHAT the show is, then the NUMBERS that size and time it. Fleet
            % size, the sequence, the shapes that can go in it (custom shape, text) and
            % their colours all answer "what gets flown"; Spacing, Altitude, Transition
            % and Hold answer "how big, how high, how fast, how long". They used to be
            % interleaved -- the four numbers sat between the sequence and the shape
            % loaders -- so reading the panel top to bottom crossed between the two kinds
            % of decision twice. The rotation rows travel with Hold rather than staying
            % behind: Rotate shares Hold's row, and rotation is a property OF the hold,
            % since the hold length is the only thing that decides how far a formation
            % gets to turn. Children are placed in CREATION order, so the order below IS
            % the order on screen -- the one hard-coded row index (RowHeight{4}, the
            % sequence line) has to be moved by hand if anything above it moves.
            g = uigridlayout(panel, [14, 2]);
            g.ColumnWidth = {120, '1x'};
            g.RowHeight = repmat({'fit'}, 1, 14);
            g.Padding = [5 5 5 5]; g.RowSpacing = 3;

            uilabel(g, 'Text', 'Number of UAVs:');
            % 500 is verified, not guessed: the plan builds and clears d_min with room
            % to spare. Feasibility was never the ceiling -- even 1000 drones plan
            % cleanly. What used to cap this at 200 was the GEOMETRY LAW:
            % Circle and Sphere held the spacing around a single ring, radius =
            % N_uav*spacing/(2*pi), so the transit grew linearly with the fleet while the
            % Transition field does not, and at 220 drones the transition the planner
            % needed passed this panel's 120 s limit. setupParams now switches to
            % concentric rings (and a thinner sphere shell) past formation_radius_max, so
            % the extent grows as sqrt(N) and 500 drones fit in a 65 m disc.
            %
            % What a big fleet still costs is TIME. The MAVLink upload is one mission item
            % per waypoint per drone at one packet per tick, so it is linear in the fleet
            % and deliberately left that way -- it is the part of the model that is
            % actually modelled in protocol detail. At 500 drones that is several minutes
            % of wall clock before the fleet leaves the ground; warnFleetCost() below says
            % so on screen rather than leaving it to be discovered.
            app.NumUAVSpinner = uispinner(g, 'Value', 10, 'Limits', [4 500], 'Step', 1, ...
                'ValueChangedFcn', @(~,~) app.warnFleetCost());

            uilabel(g, 'Text', 'Formations:');
            app.FormationList = uidropdown(g, ...
                'Items', {'Grid→Circle→Sphere→Circle→Grid', ...
                          'Grid→Circle→Grid', ...
                          'Circle→Sphere→Circle', ...
                          'Grid→Sphere→Grid'}, ...
                'Value', 'Grid→Circle→Sphere→Circle→Grid', ...
                'ValueChangedFcn', @(~,~) app.syncSequenceFromList());

            % Build a sequence a step at a time instead of choosing a canned one. The
            % picker lists everything a formation can be -- the four built-in patterns
            % and every shape loaded from a file -- so a file-based formation is added
            % exactly like a Circle is. Add appends to the end, Undo drops the last
            % step, Clear starts again from a single Grid.
            uilabel(g, 'Text', 'Add formation:');
            addRow = uigridlayout(g, [1, 4]);
            addRow.ColumnWidth = {'1x', 46, 52, 50};
            addRow.RowHeight = {'fit'};
            addRow.Padding = [0 0 0 0];
            addRow.ColumnSpacing = 3;
            app.FormationPicker = uidropdown(addRow, ...
                'Items', app.formationNames(), 'Value', 'Grid');
            app.AddFormBtn = uibutton(addRow, 'Text', 'Add', ...
                'ButtonPushedFcn', @(~,~) app.addToSequence());
            app.RemoveFormBtn = uibutton(addRow, 'Text', 'Undo', ...
                'ButtonPushedFcn', @(~,~) app.undoSequence());
            app.ClearFormBtn = uibutton(addRow, 'Text', 'Clear', ...
                'ButtonPushedFcn', @(~,~) app.clearSequence());

            % Spelling the sequence out numbered is what makes a long one readable:
            % "Grid→Star→Circle→Star→Grid" in a dropdown that is 200 px wide is
            % truncated exactly where it stops being obvious.
            % Two wrapped lines rather than one ellipsized one: at 8 formations a
            % single line ended in "6. Sphere ..." and the tail of the show was
            % unreadable. Beyond ~10 steps even two lines run out, which is why
            % refreshSequenceLabel also puts the whole sequence in the tooltip.
            app.SequenceLabel = uilabel(g, 'Text', '', ...
                'FontSize', 10, 'FontColor', [0.2 0.2 0.45], 'WordWrap', 'on');
            app.SequenceLabel.Layout.Column = [1 2];
            g.RowHeight{4} = 30;

            % Build a formation out of a file. The loaded shape is added to the
            % Formations dropdown above rather than replacing anything, so the
            % operator picks it like any built-in pattern.
            uilabel(g, 'Text', 'Custom shape:');
            app.LoadShapeBtn = uibutton(g, 'Text', 'Load Image / STL...', ...
                'ButtonPushedFcn', @(~,~) app.loadShapeFile());

            % Type a word and fly it. The picker's built-in "Text" entry spells
            % whatever is in this field, and Fly Text also registers the string as a
            % formation of its own -- which is how one show spells two words, since
            % the built-in type carries a single string.
            uilabel(g, 'Text', 'Text:');
            textRow = uigridlayout(g, [1, 2]);
            textRow.ColumnWidth = {'1x', 74};
            textRow.RowHeight = {'fit'};
            textRow.Padding = [0 0 0 0];
            textRow.ColumnSpacing = 3;
            app.TextField = uieditfield(textRow, 'text', 'Value', 'HI', ...
                'Tooltip', ['What the fleet spells. "|" starts a second line -- ' ...
                    'use it for anything long, since one line of 13 characters ' ...
                    'is a 13:1 letterbox with no height left to fly in. Reckon ' ...
                    'on ~10 drones per character.']);
            app.FlyTextBtn = uibutton(textRow, 'Text', 'Fly Text', ...
                'ButtonPushedFcn', @(~,~) app.flyText());

            app.ShapeLabel = uilabel(g, 'Text', 'No custom shapes added.', ...
                'FontSize', 10, 'FontColor', [0.35 0.35 0.35]);
            app.ShapeLabel.Layout.Column = [1 2];

            % Colour per formation TYPE, not per step in the sequence. That is not a
            % simplification, it is what the model does: lighting_colors is indexed by
            % formation_sequence(k) (setupParams.m:903), so a Grid→Circle→Grid show has
            % two colours and the two Grids cannot differ. Pick the formation, then its
            % colour.
            uilabel(g, 'Text', 'Colour:');
            colRow = uigridlayout(g, [1, 2]);
            colRow.ColumnWidth = {'1x', 62};
            colRow.RowHeight = {'fit'};
            colRow.Padding = [0 0 0 0];
            colRow.ColumnSpacing = 3;
            app.ColorFormDropdown = uidropdown(colRow, 'Items', app.formationNames(), ...
                'Value', 'Grid', ...
                'ValueChangedFcn', @(~,~) app.refreshColorSwatch());
            app.ColorSwatchBtn = uibutton(colRow, 'Text', '', ...
                'Tooltip', ['The colour this formation flies. It drives the model''s ' ...
                            'lighting_timeline as well as the viewer, so it is a show ' ...
                            'setting, not a display one — the flown signal changes at ' ...
                            'the next Generate.'], ...
                'ButtonPushedFcn', @(~,~) app.pickFormationColor());

            uilabel(g, 'Text', 'Spacing (m):');
            app.SpacingField = uieditfield(g, 'numeric', 'Value', 5, 'Limits', [1.5 30], ...
                'Tooltip', ['Distance between neighbouring drones inside a formation. ' ...
                            'It sets the SIZE of every shape, so raising it makes the ' ...
                            'show physically larger and the transitions longer, not ' ...
                            'just the drones further apart. It is also what bounds the ' ...
                            'flown separation (spacing / sqrt(2)), so keep it well ' ...
                            'above d_min. Takes effect at the next Generate.']);

            uilabel(g, 'Text', 'Altitude, lowest drone (m):');
            app.AltitudeField = uieditfield(g, 'numeric', 'Value', 10, 'Limits', [5 100], ...
                'Tooltip', ['Height of the LOWEST drone in every formation -- a floor, ' ...
                            'not a centre. Flat formations (Grid, Circle) all sit at it; ' ...
                            'a Sphere or a text billboard rests its bottom on it and ' ...
                            'builds upward, so the show occupies this height PLUS the ' ...
                            'height of the shape. A tall shape at a big fleet is tall: ' ...
                            'check the ceiling you are allowed to fly to. Takes effect ' ...
                            'at the next Generate.']);

            uilabel(g, 'Text', 'Transition (s):');
            % 120 s, and it stays 120 s now the fleet cap is 500. It was sized for the
            % worst case under the old single-ring geometry, where 200 drones needed
            % 108.8 s; the area-based law replacing it makes a 500-drone formation SMALLER
            % in extent than a 200-drone one was, so the transition the planner needs went
            % down rather than up. It was 30 once, which fitTransitionToPlan would have
            % tried to exceed -- and a uieditfield throws rather than clamps, so the
            % auto-fit clamps to this value itself and says so.
            app.TransitionField = uieditfield(g, 'numeric', 'Value', 8, 'Limits', [3 120], ...
                'Tooltip', ['How long the fleet takes to fly from one formation to the ' ...
                            'next. Generate RAISES this by itself when the plan would ' ...
                            'exceed v_max, and says so -- so a value here is a floor, ' ...
                            'not a promise. Note it cannot fix separation: every drone ' ...
                            'flies the same fraction of its own segment, so the paths ' ...
                            'are the same shape however long you take over them.']);

            uilabel(g, 'Text', 'Hold (s):');
            % The hold shares its row with Rotate, because rotation is a property OF the
            % hold: it is the hold that stopped being dead time, and the hold length is
            % the only thing that decides how far the formation gets to turn.
            holdRow = uigridlayout(g, [1, 2]);
            holdRow.ColumnWidth = {'1x', 76};
            holdRow.RowHeight = {'fit'};
            holdRow.Padding = [0 0 0 0];
            holdRow.ColumnSpacing = 3;
            % 120 s, raised from 30 to match Transition. 30 s made a full turn
            % unreachable for most fleets rather than merely slow: the sweep is capped by
            % tangential speed, so one turn of a 10-drone Grid needs an 18.8 s hold and a
            % 100-drone one needs far more. A long hold is a legitimate show element now
            % that something happens during it.
            app.HoldField = uieditfield(holdRow, 'numeric', 'Value', 5, 'Limits', [2 120], ...
                'ValueChangedFcn', @(~,~) app.refreshRotationLabel(), ...
                'Tooltip', ['How long the fleet stays in each formation before the next ' ...
                            'transition. Dead time for the planner unless Rotate is on -- ' ...
                            'with it, this is also what sets how far each formation gets ' ...
                            'to turn. Takes effect at the next Generate.']);
            app.RotateCheckbox = uicheckbox(holdRow, 'Text', 'Rotate', 'Value', false, ...
                'ValueChangedFcn', @(~,~) app.rotateToggled(), ...
                'Tooltip', ['Master switch for rotation — WHICH formations turn and how ' ...
                            'far is set on the row below. Spinning is about a ' ...
                            'formation''s own vertical axis while the fleet holds it: ' ...
                            'every drone keeps its altitude and circles the formation ' ...
                            'centre at its own radius, so the outer drones sweep wide ' ...
                            'and the middle barely moves — a Sphere turns as a ball, a ' ...
                            'Grid as a turntable, a text billboard goes edge-on and ' ...
                            'back. Separation is unaffected: a rigid rotation preserves ' ...
                            'every distance exactly. Unticking this leaves the angles ' ...
                            'alone, so it is a mute rather than a reset.']);

            % Which formation turns, and how far. One step at a time rather than a table of
            % all of them: the panel has room for one row, not for five, and a show usually
            % has one or two formations worth spinning.
            %
            % Degrees, not turns, because a full turn is almost never what gets flown -- the
            % acceleration cap trims it to a few tens of degrees at a typical hold, and a
            % request INSIDE the cap is delivered exactly. That makes a small angle a promise
            % in a way "one turn" never was, which is the reason this field exists.
            uilabel(g, 'Text', 'Rotate step:');
            rotRow = uigridlayout(g, [1, 3]);
            rotRow.ColumnWidth = {'1x', 58, 16};
            rotRow.RowHeight = {'fit'};
            rotRow.Padding = [0 0 0 0];
            rotRow.ColumnSpacing = 3;
            app.RotateStepDropdown = uidropdown(rotRow, 'Items', {'1. Grid'}, ...
                'ValueChangedFcn', @(~,~) app.rotateStepChanged(), ...
                'Tooltip', ['Which step of the sequence the angle applies to. Steps are ' ...
                            'numbered as the Sequence line above shows them, so a ' ...
                            'Grid→Circle→Grid show can spin step 1 and leave step 3 ' ...
                            'still — rotation is per STEP, unlike Colour, which is per ' ...
                            'formation type.']);
            % Limits, not a spinner: ±3 turns is more than any hold can deliver, and a
            % uieditfield THROWS on an out-of-range assignment rather than clamping, so the
            % range has to be wide enough that nothing sensible hits it.
            app.RotateAngleField = uieditfield(rotRow, 'numeric', 'Value', 0, ...
                'Limits', [-1080 1080], 'ValueChangedFcn', @(~,~) app.rotateAngleChanged(), ...
                'Tooltip', ['Degrees this formation turns during its hold. 0 leaves it ' ...
                            'still; a negative value turns the other way. Ask for more ' ...
                            'than the fleet can hold and Generate replaces it with the ' ...
                            'largest angle that fits, so the number here is always one ' ...
                            'that will be flown exactly — and typing a big angle is how ' ...
                            'you find out what that limit is. The line below says which ' ...
                            'bound it ran into and what hold the full angle would need.']);
            uilabel(rotRow, 'Text', '°');

            % What the rotation actually managed. Worth a permanent label rather than a
            % line in the status bar: the sweep is almost always LESS than the full turn
            % that was asked for, and the status bar is overwritten by the next action.
            app.RotationLabel = uilabel(g, 'Text', '', ...
                'FontSize', 10, 'FontColor', [0.35 0.35 0.35], 'WordWrap', 'on');
            app.RotationLabel.Layout.Column = [1 2];

            % Seed SequenceNames from the preset that starts selected, so Add extends
            % the show that is actually shown rather than starting from nothing.
            app.syncSequenceFromList();
            app.refreshColorSwatch();
            app.refreshRotationLabel();
        end

        function rotateToggled(app)
            % Ticking Rotate SEEDS NOTHING. It enables the panel and leaves every angle at
            % 0, so the field reads 0 until an angle is typed.
            %
            % It used to seed 360 as a "spin it as far as it will go" sentinel, relying on
            % clampRotationToCap to rewrite each step to its own cap at the first Generate.
            % That was reported as a bug, and it is the right report: 0 is the value of an
            % angle nobody has chosen, and a field that jumps to a full turn on a tick reads
            % as the app deciding the show rather than as a placeholder -- especially since a
            % full turn is out of reach at any ordinary hold, so the number shown was never
            % the number flown. Nothing is lost: typing 360, or any over-cap angle, still
            % clamps to the true cap at Generate, which remains the way to ask for the most
            % the fleet can manage. The readout now says so instead of the field implying it.
            %
            % The cost is a ticked box that spins nothing until an angle is typed, which the
            % old comment called the worse default. It is answered in the readout rather than
            % in the value: refreshRotationLabel names the empty request and says what to do
            % about it. Generating in that state is well defined -- the app always assigns
            % rotation_deg_request, so setupParams takes the explicit path and re-derives
            % formation_rotate from the angles, giving a still show bit-identical to the
            % pre-rotation plan.
            app.syncRotationLength();
            app.refreshRotationSteps();
            app.refreshRotationLabel();
        end

        function rotateStepChanged(app)
            % Picking a different step just shows that step's angle. Nothing is written,
            % so browsing the sequence cannot change the show.
            k = app.rotationStepIndex();
            if k > 0
                app.RotateAngleField.Value = app.RotationDegPerStep(k);
            end
        end

        function rotateAngleChanged(app)
            k = app.rotationStepIndex();
            if k > 0
                app.RotationDegPerStep(k) = app.RotateAngleField.Value;
            end
            % The checkbox FOLLOWS the angles. Typing 90 into a step with Rotate unticked
            % would otherwise be silently discarded at the next Generate, and zeroing every
            % angle would leave the box ticked claiming a spin that is not there. It stays
            % on the panel because it is still the one-click mute -- unticking it pushes
            % zeros without discarding what is typed here.
            app.RotateCheckbox.Value = any(app.RotationDegPerStep ~= 0);
            app.refreshRotationSteps();
            app.refreshRotationLabel();
        end

        function nTrim = clampRotationToCap(app, pushed)
            % Bring an over-cap request DOWN to the angle the plan could actually fly.
            %
            % An angle INSIDE the cap is delivered exactly, so an over-cap one is the only
            % case in this panel where what is typed is not what happens. Rewriting it to
            % the cap is what fitTransitionToPlan already does for an unflyable transition:
            % fix the field, then say what changed, rather than leaving the operator to
            % read the shortfall off a label after every Generate.
            %
            % The cap CANNOT be computed when the angle is typed. It depends on the
            % formation's largest radius from the spin axis, which comes out of setupParams'
            % geometry -- so this runs after Generate, against the plan just built.
            % rotation_sweep IS the cap wherever the request was trimmed
            % (formationHoldSamples.m:157 takes min(|request|, thetaMax)), so no new export
            % is needed and the number here can never drift from what the planner did.
            nTrim = 0;
            % Only when the box is ticked. Unticked, the request PUSHED was all zeros, so
            % every sweep is zero -- this would read the mute as a cap of nothing and wipe
            % the very angles the mute exists to preserve.
            if ~app.RotateCheckbox.Value
                return
            end
            try
                swept = abs(rad2deg(evalin('base', 'rotation_sweep')));
            catch
                return          % no plan in the workspace, so nothing to clamp against
            end
            if numel(swept) ~= numel(pushed) || numel(pushed) ~= numel(app.RotationDegPerStep)
                return          % setupParams resized the request; leave the panel alone
            end
            for k = 1:numel(pushed)
                if pushed(k) == 0 || abs(pushed(k)) <= swept(k) + 1e-6
                    continue    % left still on purpose, or the request was flown in full
                end
                % Round DOWN, never to nearest: the cap is the largest angle the plan will
                % fly, so rounding up would land back above it and the next Generate would
                % trim again -- the field would never settle. Whole degrees where there is
                % a whole degree to be had, hundredths below that, so a formation that can
                % only manage half a degree does not silently become a still one.
                if swept(k) >= 1
                    mag = floor(swept(k));
                else
                    mag = floor(swept(k) * 100) / 100;
                end
                % Keep the DIRECTION asked for. The cap bounds how FAR, never which way,
                % and -360 has to come back as -39 rather than +39.
                app.RotationDegPerStep(k) = sign(pushed(k)) * mag;
                nTrim = nTrim + 1;
            end
            if nTrim > 0
                % The checkbox follows the angles, exactly as rotateAngleChanged has it. A
                % formation whose cap floors to zero genuinely cannot rotate at this hold,
                % and leaving the box ticked would claim a spin that is not in the plan.
                app.RotateCheckbox.Value = any(app.RotationDegPerStep ~= 0);
                app.refreshRotationSteps();
            end
        end

        function k = rotationStepIndex(app)
            % The picker's items read "N. Name (A°)", so the step is the leading number.
            % Parsed back out rather than cached alongside the dropdown: the items are
            % rebuilt whenever the sequence changes, and a stored index could outlive the
            % sequence it indexed -- which is the failure mode where an angle lands on the
            % wrong formation.
            k = 0;
            if isempty(app.RotateStepDropdown) || ~isvalid(app.RotateStepDropdown)
                return
            end
            n = str2double(strtok(app.RotateStepDropdown.Value, '.'));
            if ~isnan(n) && n >= 1 && n <= numel(app.RotationDegPerStep)
                k = n;
            end
        end

        function syncRotationLength(app)
            % RESIZE the angle vector to the sequence, never reset it: a show whose
            % sequence grew from 3 steps to 5 keeps the angles already chosen for 1-3 and
            % leaves the two new holds still. Discarding the lot would read as the app
            % forgetting a setting because an unrelated one was edited. setupParams'
            % own guard on rotation_deg_request resizes for the same reason.
            n = numel(app.SequenceNames);
            m = numel(app.RotationDegPerStep);
            if m < n
                app.RotationDegPerStep(m+1:n) = 0;
            elseif m > n
                app.RotationDegPerStep = app.RotationDegPerStep(1:n);
            end
        end

        function refreshRotationSteps(app)
            % Rebuild the step picker from the sequence. Called on every sequence edit
            % (through refreshSequenceLabel, which they all funnel through) and whenever an
            % angle changes.
            %
            % The angle goes in the ITEM TEXT, not just in the field, so the answer to
            % "which formations rotate" is visible without opening anything -- that is the
            % job a table of all the steps would have done, at one row of panel instead of
            % five.
            if isempty(app.RotateStepDropdown) || ~isvalid(app.RotateStepDropdown)
                return
            end
            app.syncRotationLength();
            n = numel(app.SequenceNames);
            if n == 0
                return
            end
            keep = app.rotationStepIndex();
            items = cell(1, n);
            for k = 1:n
                if app.RotationDegPerStep(k) == 0
                    items{k} = sprintf('%d. %s', k, app.SequenceNames{k});
                else
                    items{k} = sprintf('%d. %s (%g°)', k, app.SequenceNames{k}, ...
                        app.RotationDegPerStep(k));
                end
            end
            app.RotateStepDropdown.Items = items;
            if keep < 1 || keep > n
                keep = 1;
            end
            app.RotateStepDropdown.Value = items{keep};
            app.RotateAngleField.Value = app.RotationDegPerStep(keep);
        end

        function refreshRotationLabel(app, nTrim)
            % Say how far the formations will actually turn. Called when Rotate or Hold
            % changes, and again after Generate with the real numbers.
            %
            % NTRIM is how many angles clampRotationToCap just rewrote, and defaults to
            % none so the three UI callbacks that call this can go on calling it bare.
            % Passed in rather than re-derived: by the time this runs the panel already
            % AGREES with the plan, so the fact that it disagreed a moment ago is not
            % recoverable from anything either of them holds.
            if nargin < 2
                nTrim = 0;
            end
            %
            % This exists because the honest answer is almost never "one turn". Sweep is
            % bounded by what the fleet can fly, so the hold that was chosen for its own
            % reasons decides the angle, and leaving the operator to discover a 100 degree
            % sweep by watching a formation not come back round would read as a bug in the
            % rotation.
            %
            % Two different bounds can be the one that bit, and the label names which,
            % because they point at different dials. Speed-limited means the outermost drone
            % would fly faster than the fleet tracks. Acceleration-limited -- the usual case
            % -- means holding it ON its circle needs a centripetal pull the controller
            % cannot deliver, which is a bound on w^2*r rather than on w*r and so bites
            % sooner. Both are cured by a longer hold or a smaller formation.
            if isempty(app.RotationLabel) || ~isvalid(app.RotationLabel)
                return
            end
            if ~app.RotateCheckbox.Value
                app.RotationLabel.Text = '';
                app.RotationLabel.Tooltip = '';
                return
            end

            % Which steps were asked to turn, named the way the picker names them. This is
            % the part the old single-checkbox label could not say at all.
            app.syncRotationLength();
            askedSteps = find(app.RotationDegPerStep ~= 0);
            stepStr = app.rotationStepsPhrase(askedSteps);

            % Ticked with every angle still 0. This is now a state the panel reaches on the
            % first tick -- rotateToggled seeds nothing -- so it needs its own line rather
            % than falling through to "Rotating nothing", and it is the one place to say
            % where the ceiling comes from. Said here instead of in the field's value
            % because a placeholder angle claims a show setting that was never chosen.
            if isempty(askedSteps)
                app.RotationLabel.Text = ['Rotate is on but no angle is set. Type ' ...
                    'degrees for each step you want to turn.'];
                app.RotationLabel.Tooltip = ['An angle bigger than the fleet can fly in ' ...
                    'the hold is brought down to that limit at Generate, so entering a ' ...
                    'deliberately large one — 360 — asks for as much turn as the ' ...
                    'formation can manage.'];
                return
            end

            % Before the first Generate there is no plan to read, so predict from the
            % geometry the plan will use. Only an estimate -- the exact radius depends on
            % which formations are in the sequence -- so it is labelled as one.
            if isempty(app.TrajectoryData)
                app.RotationLabel.Text = sprintf( ...
                    ['Rotating %s. Press Generate for the sweep each can actually ' ...
                     'fly in %.0f s.'], stepStr, app.HoldField.Value);
                app.RotationLabel.Tooltip = '';
                return
            end

            try
                sweep = rad2deg(evalin('base', 'rotation_sweep'));
                asked = abs(evalin('base', 'rotation_deg_request'));
                needHold = evalin('base', 'rotation_min_hold');
                limitedBy = evalin('base', 'rotation_limit');
                peakAccel = max(evalin('base', 'rotation_peak_accel'));
                peakSpeed = max(evalin('base', 'rotation_peak_speed'));
            catch
                app.RotationLabel.Text = sprintf('Rotating %s.', stepStr);
                return
            end

            % Report over the ROTATING formations only. A still hold's 0° folded into the
            % range would read as a spin that nearly stopped -- "0-39°" -- when what it
            % means is that the operator left that formation alone on purpose.
            rotF = find(sweep ~= 0);
            if isempty(rotF)
                app.RotationLabel.Text = '';
                app.RotationLabel.Tooltip = '';
                return
            end
            swept = sweep(rotF);
            limitedBy = limitedBy(rotF);
            % Re-derive the step names from the PLAN rather than from the panel. They agree
            % today, but the plan is what flew and the panel is what is typed, and the two
            % drift the moment an angle is edited without pressing Generate.
            stepStr = app.rotationStepsPhrase(rotF);
            askedStr = '';
            if max(swept) - min(swept) < 0.5
                sweepStr = sprintf('%.0f°', swept(1));
            else
                sweepStr = sprintf('%.0f-%.0f°', min(swept), max(swept));
            end
            % Say what was ASKED alongside what was managed, whenever they differ. Without
            % it the label reports 39° against a field reading 90 and leaves the operator to
            % guess which of the two the fleet is flying.
            if numel(asked) == numel(sweep) && any(abs(asked(rotF) - swept) > 0.5)
                a = asked(rotF);
                if max(a) - min(a) < 0.5
                    askedStr = sprintf(' of the %.0f° asked', a(1));
                else
                    askedStr = sprintf(' of the %.0f-%.0f° asked', min(a), max(a));
                end
            end
            sweepStr = [sweepStr askedStr ' on ' stepStr];
            if needHold > app.HoldField.Value + 1e-9
                % The radius quoted has to come from a ROTATING formation. rotation_radius
                % is geometric and is recorded for every hold whether it spins or not, so a
                % plain max() over the whole show could blame the cap on the extent of a
                % formation the operator deliberately left still.
                radii = evalin('base', 'rotation_radius');
                rotRadius = max(radii(rotF));
                % The binding request is the one needing the LONGEST hold, which with
                % per-formation angles is not necessarily the largest angle: 90° on a wide
                % Sphere needs longer than 180° on a tight Grid.
                needs = evalin('base', 'rotation_need_hold');
                [~, iBind] = max(needs);
                askBind = abs(asked(min(iBind, numel(asked))));
                if any(strcmp(limitedBy, 'acceleration'))
                    whyShort = 'acceleration-limited';
                    whyLong = sprintf( ...
                        ['Holding a drone %.1f m from the spin axis ON its circle needs ' ...
                         'a centripetal %.2f m/s², sustained for the whole hold, and the ' ...
                         'plan budgets %.2f of the %.1f m/s² a_max allows -- the rest is ' ...
                         'left for the controller''s own tracking error, which draws on ' ...
                         'the same limit. Turning %.0f° in %.0f s would exceed it, so the ' ...
                         'plan sweeps as far as it can hold rather than commanding a ' ...
                         'circle the drones would spiral out of. Ask for a smaller angle ' ...
                         'and it is flown exactly.'], ...
                        rotRadius, peakAccel, ...
                        evalin('base', 'rotation_accel_target'), ...
                        evalin('base', 'a_max'), askBind, app.HoldField.Value);
                else
                    whyShort = 'speed-limited';
                    whyLong = sprintf( ...
                        ['The widest rotating formation has a drone %.1f m from the spin ' ...
                         'axis, so it flies at %.2f m/s. Turning it %.0f° in %.0f s would ' ...
                         'need more than the %.1f m/s the fleet tracks, so the plan ' ...
                         'sweeps as far as it can instead of commanding a speed the ' ...
                         'drones will not follow.'], ...
                        rotRadius, peakSpeed, askBind, ...
                        app.HoldField.Value, evalin('base', 'v_track_target'));
                end
                % Say that the FIELD was rewritten, not only that the sweep fell short.
                % The angle on the panel changing without being typed is a surprise on its
                % own, and "39° of the 90° asked" explains the plan while saying nothing
                % about why the box no longer reads 90.
                if nTrim > 1
                    trimNote = ' Angles reduced to the most that fits.';
                elseif nTrim == 1
                    trimNote = ' Angle reduced to the most that fits.';
                else
                    trimNote = '';
                end
                app.RotationLabel.Text = sprintf( ...
                    'Rotating %s — %s. The full angle needs Hold ≥ %.0f s.%s', ...
                    sweepStr, whyShort, ceil(needHold), trimNote);
                app.RotationLabel.Tooltip = whyLong;
            else
                app.RotationLabel.Text = sprintf('Rotating %s.', sweepStr);
                app.RotationLabel.Tooltip = '';
            end
        end

        function s = rotationStepsPhrase(app, steps)
            % Name the rotating steps the way the picker names them, so the readout and the
            % dropdown agree about what "step 2" is. A run of consecutive steps collapses to
            % a range because "steps 1,2,3,4,5" for a whole show is noise.
            n = numel(app.SequenceNames);
            if isempty(steps)
                s = 'nothing';
            elseif numel(steps) == n && n > 1
                s = sprintf('all %d formations', n);
            elseif isscalar(steps)
                s = sprintf('step %d (%s)', steps, app.SequenceNames{steps});
            elseif isequal(steps, steps(1):steps(end))
                s = sprintf('steps %d-%d', steps(1), steps(end));
            else
                s = sprintf('steps %s', strjoin(string(steps), ','));
            end
        end

        function createFlightPanel(app, parent)
            % "Flight limits", not "Flight & Safety". The "& Safety" half was doing no work:
            % it invited you to look here for the abort, which is in Run the show, and for the
            % RTK degradation, which is its own panel. What these three fields are is the
            % envelope the PLANNER sizes the show against -- so they are named for that, and
            % each tooltip says the thing that is easy to get wrong about them: they
            % constrain the plan, they do not clip a drone in flight.
            panel = uipanel(parent, 'Title', 'Flight limits');
            % FIVE rows: v_max, a_max, d_min, Sim Mode, and the Sim Mode note. The row count has
            % to be kept in step -- a grid declared too short still accepts the extra children,
            % so the last one just vanishes off the bottom with no warning anywhere.
            g = uigridlayout(panel, [5, 2]);
            g.ColumnWidth = {120, '1x'};
            g.RowHeight = repmat({'fit'}, 1, 5);
            g.Padding = [5 5 5 5]; g.RowSpacing = 3;

            uilabel(g, 'Text', 'v_max (m/s):');
            app.VmaxField = uieditfield(g, 'numeric', 'Value', 5.0, 'Limits', [1 20], ...
                'Tooltip', ['Speed the fleet is expected to track. Used to CHECK the ' ...
                            'planned show, not to clip it: if the plan demands more, the ' ...
                            'Transition auto-fit lengthens the transition until it fits ' ...
                            'and tells you. Takes effect at the next Generate.']);

            uilabel(g, 'Text', 'a_max (m/s²):');
            app.AmaxField = uieditfield(g, 'numeric', 'Value', 3.0, 'Limits', [0.5 15], ...
                'Tooltip', ['Acceleration budget, checked the same way as v_max. The ' ...
                            'min-jerk profile peaks well above the average, so a plan ' ...
                            'that looks gentle on paper can still exceed this. Takes ' ...
                            'effect at the next Generate.']);

            uilabel(g, 'Text', 'd_min (m):');
            app.DminField = uieditfield(g, 'numeric', 'Value', 2.0, 'Limits', [0.5 10], ...
                'Tooltip', ['Minimum separation the show must keep. This is the number ' ...
                            'the launch-pad and formation geometry are sized against, ' ...
                            'and the assignment guarantees the flown separation stays ' ...
                            'above the tighter formation''s spacing / sqrt(2) -- so ' ...
                            'raising it makes formations bigger rather than slower. ' ...
                            'Takes effect at the next Generate.']);

            uilabel(g, 'Text', 'Sim Mode:');
            % Normal is the default deliberately. The upload pins itself to Normal (it needs
            % runtime reads for live progress), so any other choice here makes
            % Upload -> Simulate cross a mode boundary and rebuild the target (~100 s).
            %
            % WHAT SIM MODE IS FOR, NOW THAT PACING IS FIXED AT 1x. Less than it was, on a
            % small fleet. Accelerator measured 36.1 s median against Normal's 44.0 s over the
            % same show (non-overlapping ranges) with the live view fully intact -- but a
            % streamed run is held to real time, and both modes already outrun 1x there, so the
            % governor just withholds more and the wall clock lands in the same place either
            % way. Where the choice earns its keep is the other end: a large fleet with meshes
            % runs BELOW real time, and there Accelerator is the difference between reaching 1x
            % and the readout reporting that it could not.
            %
            % Rapid Accelerator is much faster again (14.9 s) and numerically identical, but it
            % runs the model as a separate executable, so SimulationStatus reads 'external' and
            % block RuntimeObject values never advance -- there is no live view to be had. That
            % is also what makes it the honest answer to "I do not want to sit through this",
            % which is what a Live sim rate of "Fastest" was really being asked for.
            app.SimModeDropdown = uidropdown(g, ...
                'Items', {'Normal', 'Accelerator', 'Rapid Accelerator'}, ...
                'Value', 'Normal', ...
                'Tooltip', ['Accelerator: ~18% more solver throughput, live 3-D view ' ...
                            'works. On a small fleet it will not shorten the run -- a live ' ...
                            'run is paced to real time either way -- but on a large fleet ' ...
                            'it is what makes real time reachable. Rapid Accelerator: much ' ...
                            'faster still and results are exact, but the 3-D view and ' ...
                            'progress bars stay frozen; use it when nobody is watching.'], ...
                'ValueChangedFcn', @(~,~) app.simModeChanged());

            app.SimModeNote = uilabel(g, 'Text', '', 'FontSize', 10, ...
                'FontColor', [0.55 0.35 0.0], 'WordWrap', 'on');
            app.SimModeNote.Layout.Column = [1 2];
        end

        function simModeChanged(app)
            % A visible warning, not just a tooltip: in Rapid Accelerator the show still runs and
            % the logged results are exact, but the drones do not move on screen. Without this the
            % frozen view reads as a hung simulation.
            if strcmp(app.SimModeDropdown.Value, 'Rapid Accelerator')
                app.SimModeNote.Text = ['Rapid Accelerator: no live view. The 3-D view and ' ...
                    'progress bars stay frozen while it runs; results are still exact.'];
            else
                app.SimModeNote.Text = '';
            end
        end

        function createNavPanel(app, parent)
            % "GNSS / RTK", not "Navigation". Navigation described the subject loosely
            % enough to cover half the app; what this panel actually is, is the place you
            % break the corrections and watch what that does to the fleet.
            panel = uipanel(parent, 'Title', 'GNSS / RTK');
            % SIX rows, and they are in two groups: three SETTINGS first, then a heading,
            % then two LIVE READOUTS. The old order opened with a readout (RTK tier) and
            % closed with another (Denied) with the settings sandwiched between, so nothing
            % on screen distinguished what you type from what the model tells you -- and one
            % of these rows used to be a dropdown that looked exactly like a setting while
            % being read by no block at all. Grouping them is the fix for that whole class
            % of confusion.
            %
            % The row count has to be kept in step with leftScroll.RowHeight{3} -- a grid
            % declared too short still accepts the extra children, so the last one just
            % vanishes off the bottom with no warning anywhere.
            g = uigridlayout(panel, [6, 2]);
            g.ColumnWidth = {120, '1x'};
            g.RowHeight = repmat({'fit'}, 1, 6);
            g.Padding = [5 5 5 5]; g.RowSpacing = 3;

            % A text field, not numeric, because one drone was never the interesting
            % case: a formation loses its shape when a HANDFUL of drones drift, and
            % rtk_deny_mask has always been a full logical vector behind the Constant.
            % Numeric also forced Limits to be the fleet CAP rather than the fleet, so
            % validation had to live in the callback anyway -- now all of it does, and
            % the field cannot throw on assignment the way a uieditfield past its
            % Limits does.
            uilabel(g, 'Text', 'Degrade UAV(s):');
            app.DegradeUAVField = uieditfield(g, 'text', 'Value', '0', ...
                'Tooltip', ['Which drones lose their RTK corrections. One number, a ' ...
                            'list (1 3 5, or 1,3,5, or 1 and 3), a range (1:5 or 1-5), ' ...
                            'brackets optional, or 0 for the whole ' ...
                            'fleet. Each named drone falls back RTK Fix -> Float -> ' ...
                            'Standalone and STOPS there (1.5 m) -- it does not fly ' ...
                            'away. Numbers past the fleet size are dropped with a note. ' ...
                            'Once denied, a drone is drawn RED in the 3-D view, so you ' ...
                            'can see which one you named.']);

            % Degrade and Restore instead of a "degrade at (s)" field. The mask is a
            % workspace variable behind a Constant, so it can be retuned mid-run the way
            % Abort & Land is: press it when you want it and it takes effect from that
            % moment. Naming a second in advance was the old interface, and it required
            % the operator to know when the interesting part of the show would be.
            uilabel(g, 'Text', 'RTK corrections:');
            btnRow = uigridlayout(g, [1, 2]);
            btnRow.ColumnWidth = {'1x', '1x'};
            btnRow.RowHeight = {'fit'};
            btnRow.Padding = [0 0 0 0];
            btnRow.ColumnSpacing = 3;
            app.DegradeBtn = uibutton(btnRow, 'Text', 'Degrade', ...
                'BackgroundColor', [0.85 0.65 0.2], ...
                'Tooltip', ['Deny that drone its corrections from now on. Works before ' ...
                            'a run (applies from t=0) and during one (applies from the ' ...
                            'moment you press it). The drones you name are ringed in ' ...
                            'white in the 3-D view straight away -- that ring is how you ' ...
                            'tell which drone is which, since the formation slots are ' ...
                            're-assigned at every transition.'], ...
                'ButtonPushedFcn', @(~,~) app.setRtkDeny(true));
            app.RestoreBtn = uibutton(btnRow, 'Text', 'Restore', ...
                'Tooltip', ['Give the corrections back. The error does not vanish -- it ' ...
                            'bleeds off over reconverge_time, like a real receiver ' ...
                            're-fixing.'], ...
                'ButtonPushedFcn', @(~,~) app.setRtkDeny(false));

            % The other per-drone correction knob, and it needed a control rather than
            % being left as a bare workspace variable. setupParams only defaults it when
            % it is missing, so a value left behind by an earlier script stayed live and
            % quietly put the WHOLE fleet on the Float tier with no way to see
            % why. Generate now pushes this field like every other one.
            uilabel(g, 'Text', 'Per-UAV loss (%):');
            app.UavLossField = uieditfield(g, 'numeric', 'Value', 0, 'Limits', [0 100], ...
                'Tooltip', ['Each drone independently misses this fraction of the ' ...
                            'corrections. Unlike Packet Loss, which erases the frame ' ...
                            'for everyone at once, this decorrelates the fleet.']);

            % ---- live readouts, below the settings and marked as such ----------------
            hdr = uilabel(g, 'Text', 'Live readouts', 'FontSize', 10, ...
                'FontAngle', 'italic', 'FontColor', [0.4 0.4 0.4]);
            hdr.Layout.Column = [1 2];

            % This row used to be an RTK Mode dropdown, and it was the one control in
            % the app that lied: it assigned nav_mode in the base workspace and NO block
            % read it (verified by scanning every dialog parameter of every block --
            % zero hits), so all three settings simulated identically. The three tiers
            % it named are real, but a tier is an OUTCOME of a correction going stale,
            % not something to select. So it is now an output, showing which tier the
            % fleet is actually on, polled off InjectGate while the show streams --
            % and it now sits under the "Live readouts" heading where an output belongs,
            % rather than at the top of the panel still looking like the dropdown it was.
            uilabel(g, 'Text', 'RTK tier:');
            app.RTKTierLabel = uilabel(g, 'Text', '—', ...
                'FontColor', [0.35 0.35 0.35], ...
                'Tooltip', ['Live readout, not a setting. The worst tier any drone is ' ...
                            'on right now: RTK Fix (2 cm) -> Float (0.3 m) -> ' ...
                            'Standalone (1.5 m). Reached by letting a correction go ' ...
                            'stale -- use Degrade above, or Packet Loss.']);

            uilabel(g, 'Text', 'Denied:');
            app.DenyLabel = uilabel(g, 'Text', 'none', 'FontColor', [0.35 0.35 0.35], ...
                'Tooltip', ['Which drones are currently denied their corrections. These ' ...
                            'are drawn RED in the 3-D view, which is the only way to ' ...
                            'tell which drone is which: slots are re-assigned by cost at ' ...
                            'every transition, so index 3 is somewhere different in ' ...
                            'every formation and there is no position to learn.']);
        end

        function createCommPanel(app, parent)
            % "Radio link", not "Communication": these four fields are all properties of the
            % one radio channel between the base station and the fleet, and naming the thing
            % rather than the topic says which of the app's several message paths they touch.
            % All four are pushed at Generate, so none of them changes a run already going.
            %
            % NINE rows, in the same two groups the GNSS / RTK panel uses: four SETTINGS,
            % then an italic heading, then four LIVE READOUTS. The readouts name the MAVLink
            % message actually on the wire in each direction, which is what this panel was
            % missing -- every field here described the link's PROPERTIES and nothing said
            % what was travelling over it.
            %
            % Row count must stay in step with leftScroll.RowHeight{4}: a grid declared too
            % short still accepts the extra children, and the overflow pushes whatever is at
            % the BOTTOM off the panel, so the symptom never points at the row that was
            % added.
            panel = uipanel(parent, 'Title', 'Radio link');
            g = uigridlayout(panel, [9, 2]);
            g.ColumnWidth = {120, '1x'};
            g.RowHeight = repmat({'fit'}, 1, 9);
            g.Padding = [5 5 5 5]; g.RowSpacing = 3;

            uilabel(g, 'Text', 'Latency (ms):');
            app.LatencyField = uieditfield(g, 'numeric', 'Value', 20, 'Limits', [0 500], ...
                'Tooltip', ['One-way delay on the radio link. Applies to the whole link, ' ...
                            'so it delays the MAVLink upload and the telemetry coming ' ...
                            'back. Takes effect at the next Generate.']);

            uilabel(g, 'Text', 'Jitter (ms):');
            app.JitterField = uieditfield(g, 'numeric', 'Value', 5, 'Limits', [0 100], ...
                'Tooltip', ['Random variation added to the latency, frame by frame. ' ...
                            'Takes effect at the next Generate.']);

            uilabel(g, 'Text', 'Packet Loss (%):');
            % Up to 90%: high loss is what drives the pre-flight upload into
            % UPLOAD_FAILED, so the retry path has to be reachable from here.
            app.PacketLossField = uieditfield(g, 'numeric', 'Value', 1, 'Limits', [0 90], ...
                'Tooltip', ['Fraction of frames erased outright, for EVERY drone at once ' ...
                            '-- this is the shared link failing, not a per-receiver one. ' ...
                            'Per-UAV loss under GNSS / RTK is the decorrelated version. ' ...
                            'Goes to 90% on purpose: high loss is what drives the ' ...
                            'pre-flight upload into UPLOAD_FAILED, so the retry path is ' ...
                            'reachable from here. Takes effect at the next Generate.']);

            uilabel(g, 'Text', 'Timeout (s):');
            app.TimeoutField = uieditfield(g, 'numeric', 'Value', 3.0, 'Limits', [0.5 10], ...
                'Tooltip', ['How long the base station waits for a MISSION_ACK before it ' ...
                            'retries the item. Takes effect at the next Generate.']);

            % ---- live readouts, below the settings and marked as such ----------------
            hdr = uilabel(g, 'Text', 'Live readouts (full fidelity only)', 'FontSize', 10, ...
                'FontAngle', 'italic', 'FontColor', [0.4 0.4 0.4]);
            hdr.Layout.Column = [1 2];

            % These are CURRENT-MESSAGE readouts and deliberately not frame counters.
            % Measured on a live run: a counter incremented once per poll sees 1.7% of
            % the frames -- 1 of 63 HEARTBEATs, 10 of 940 MISSION_ITEM_INTs --
            % because the poll gap reaches seconds of sim time while the messages run at
            % 1-5.5 Hz. A sampled number printed as a total would be wrong by 60x, so the
            % panel reports what is on the wire NOW, and the one exact count that already
            % exists (waypoints confirmed, off ArrCount) stays on the upload label where it
            % has always been.
            uilabel(g, 'Text', 'Uplink msg:');
            app.UplinkMsgLabel = uilabel(g, 'Text', '—', 'FontColor', [0.35 0.35 0.35], ...
                'Tooltip', ['Which MAVLink message the base station is transmitting at ' ...
                            'this instant, off the uplink scheduler''s own selector: ' ...
                            'HEARTBEAT, MISSION_COUNT, MISSION_ITEM_INT, COMMAND_LONG, ' ...
                            'SYSTEM_TIME, or idle. Idle dominates -- ~98% of ticks carry ' ...
                            'no frame, which is what a 1 Hz heartbeat looks like at a ' ...
                            '100 Hz tick rate.']);

            uilabel(g, 'Text', 'Downlink msg:');
            app.DownlinkMsgLabel = uilabel(g, 'Text', '—', 'FontColor', [0.35 0.35 0.35], ...
                'Tooltip', ['What the fleet is sending back. The cascade is ' ...
                            'MISSION_ACK (a waypoint was accepted) outranks ' ...
                            'MISSION_REQUEST_INT (a gap is being re-requested) outranks ' ...
                            'LOCAL_POSITION_NED, which is the default and carries the ' ...
                            'pose this viewer draws.']);

            uilabel(g, 'Text', 'Corrections:');
            app.RtcmMsgLabel = uilabel(g, 'Text', '—', 'FontColor', [0.35 0.35 0.35], ...
                'Tooltip', ['GPS_RTCM_DATA, the base station''s RTK correction broadcast ' ...
                            '(MAVLink msgid 233), with the age of the oldest correction ' ...
                            'any drone is holding. It is a BROADCAST: there is no ' ...
                            'acknowledgement to show, because a real base station never ' ...
                            'hears whether a rover got it. Silent until the supervisor ' ...
                            'leaves IDLE.']);

            uilabel(g, 'Text', 'Viewer shows:');
            app.ViewSourceLabel = uilabel(g, 'Text', '—', 'FontColor', [0.35 0.35 0.35], ...
                'Tooltip', ['Which pose the 3-D view is drawing. Before the show starts ' ...
                            'the fleet is parked on its pads and the offset that is being ' ...
                            'suppressed is reported here, because that offset is mostly ' ...
                            'REAL station-keeping error, not a display artefact. From ' ...
                            'SHOW onwards it is the received MAVLink pose, unmodified.']);
        end

        function createRunPanel(app, parent)
            % ONE panel for the whole path from settings to a flown show, in the order you
            % walk it: plan it, choose how the waypoints get there, fly it.
            %
            % WHY IT IS ONE PANEL. This used to be two -- "Trajectory Delivery" holding the
            % dropdown and Upload & Fly, and "Controls" holding Generate, Simulate, Reset,
            % Play, Restart, Stop and Abort. The delivery dropdown enables exactly ONE of
            % Simulate and Upload & Fly and disables the other, so the single most important
            % consequence of the most consequential control in the app happened in a panel
            % the operator was not looking at. Both panels had grown a prose hint label
            % whose only job was to describe the other panel -- which is the tell that the
            % split was wrong. Merged, the two fly buttons sit SIDE BY SIDE on the "Fly:"
            % row with exactly one live, so the either/or is visible instead of needing to
            % be narrated, and one hint label replaced two.
            %
            % Play and the scrub bar left for their own Playback panel, with the speed
            % slider that paces them: they are the controls that only ever redraw data that
            % already exists. Stop stays HERE, next to the fly buttons, because the run is
            % what you urgently need to stop -- it still halts playback too. Abort & Land
            % keeps its own colour, which is what marks it as the only button in the app
            % that changes what the aircraft do; it no longer keeps a wider footprint,
            % because the width was never carrying that meaning -- see pairGrid.
            panel = uipanel(parent, 'Title', 'Run the show');

            % Column 1 is the step label. Numbered, because the panel's whole claim is that
            % these happen in an order -- "Deliver" before "Fly" is not obvious from the
            % words alone, and the numbers are what make a greyed-out Fly row read as "not
            % yet" rather than "broken". 60 px fits "2 Deliver:" at FontSize 10; the old
            % 52 px column was sized for "Safety:".
            g = uigridlayout(panel, [7, 4]);
            g.ColumnWidth = {60, '1x', '1x', 26};
            g.RowHeight = {26, 26, 26, 20, 'fit', 26, 'fit'};
            g.Padding = [5 4 5 4];
            g.RowSpacing = 4;
            g.ColumnSpacing = 4;
            % Every DIRECT child gets an explicit Layout below. Flow placement would work
            % for the full rows and then silently misplace everything after the first
            % spanned cell. The two pairGrid subgrids are the exception: a fresh 1x2 grid
            % has no spans, so left-to-right flow inside one is unambiguous.
            lbl = @(row, txt, tip) app.stepLabel(g, row, txt, tip);

            % ---- 1: plan --------------------------------------------------------------
            lbl(1, '1 Plan:', 'Push the settings to the model and compute the trajectory.');
            p1 = app.pairGrid(g, 1);
            app.GenerateBtn = uibutton(p1, 'Text', 'Generate', ...
                'BackgroundColor', [0.3 0.7 0.4], 'FontColor', 'w', ...
                'Tooltip', ['Start here. Pushes every setting in this column to the ' ...
                            'model, plans the trajectory, and builds the 3-D scene. ' ...
                            'Nothing else in this panel works until you have.'], ...
                'ButtonPushedFcn', @(~,~) app.generateShow());
            app.ResetBtn = uibutton(p1, 'Text', 'Reset', ...
                'Tooltip', ['Clear the plan, the logged run and the 3-D scene, back to ' ...
                            'the state before the first Generate. Your settings are ' ...
                            'kept.'], ...
                'ButtonPushedFcn', @(~,~) app.resetAll());

            % ---- 2: deliver -----------------------------------------------------------
            % How the waypoints get to the drones. Both choices fly the same
            % keyframes, so a healthy link gives the same show either way - the
            % difference is whether the radio is in the loop.
            lbl(2, '2 Deliver:', ['How the waypoints reach the drones. This is what ' ...
                                  'decides which button on the Fly row is live.']);
            app.TrajSourceDropdown = uidropdown(g, ...
                'Items', {'MAVLink upload (full fidelity)', 'Workspace (quick)'}, ...
                'Value', 'MAVLink upload (full fidelity)', ...
                'Tooltip', ['MAVLink: stream the mission over the radio and fly what ' ...
                            'arrived onboard, in one continuous run, and watch the pose ' ...
                            'the ground station received rather than the true one. ' ...
                            'Workspace: feed the controller directly, skip the ' ...
                            'pre-flight entirely and watch the true position, for ' ...
                            'iterating on the show quickly.'], ...
                'ValueChangedFcn', @(~,~) app.trajSourceChanged());
            app.TrajSourceDropdown.Layout.Row = 2;
            app.TrajSourceDropdown.Layout.Column = [2 4];

            % ---- 3: fly ---------------------------------------------------------------
            % The two buttons are ALTERNATIVES for one step, so they share one row. Adjacent
            % is the point: whichever is greyed, the live one is right next to it under the
            % same label, which is the arrangement no tooltip could substitute for.
            lbl(3, '3 Fly:', ['Fly the plan. Exactly one of these is live at a time, ' ...
                              'chosen by the Deliver row above.']);
            app.UploadBtn = uibutton(g, 'Text', 'Upload & Fly', ...
                'BackgroundColor', [0.25 0.45 0.8], 'FontColor', 'w', ...
                'Tooltip', ['Stream MISSION_ITEM_INT to the fleet, verify every ' ...
                            'waypoint landed onboard, then fly the show from the ' ...
                            'onboard buffer in the same run. Live when Deliver is ' ...
                            'MAVLink.'], ...
                'ButtonPushedFcn', @(~,~) app.uploadTrajectory(), 'Enable', 'off');
            app.UploadBtn.Layout.Row = 3; app.UploadBtn.Layout.Column = 2;
            app.SimBtn = uibutton(g, 'Text', 'Simulate', ...
                'Tooltip', ['Fly the plan through the full Simulink model -- dynamics, ' ...
                            'GNSS noise, radio link and all -- and watch it live. The ' ...
                            'result replaces the planned trajectory in the viewer, so ' ...
                            'Play afterwards shows what was actually flown. Live when ' ...
                            'Deliver is Workspace; Upload & Fly does the same job on ' ...
                            'the MAVLink path.'], ...
                'ButtonPushedFcn', @(~,~) app.runSimulation(), 'Enable', 'off');
            app.SimBtn.Layout.Row = 3; app.SimBtn.Layout.Column = 3;
            app.UploadLamp = uilamp(g, 'Color', [0.55 0.55 0.55], ...
                'Tooltip', 'Grey = not uploaded, green = confirmed, red = failed');
            app.UploadLamp.Layout.Row = 3; app.UploadLamp.Layout.Column = 4;

            % HTML5 <progress> bar under the Fly row, indented to line up with the buttons
            % it reports on. Falls back to a linear gauge where uihtml is unavailable.
            try
                app.UploadBar = uihtml(g, 'HTMLSource', app.uploadBarHTML());
                app.UploadBar.Layout.Row = 4;
                app.UploadBar.Layout.Column = [2 4];
                app.UseHtmlBar = true;
            catch
                app.UseHtmlBar = false;
                app.UploadGauge = uigauge(g, 'linear', 'Limits', [0 100], ...
                    'MajorTicks', [0 50 100], 'MinorTicks', []);
                app.UploadGauge.Layout.Row = 4;
                app.UploadGauge.Layout.Column = [2 4];
            end

            app.UploadLabel = uilabel(g, 'Text', 'Not uploaded.', 'FontSize', 11);
            app.UploadLabel.Layout.Row = 5;
            app.UploadLabel.Layout.Column = [2 4];

            % ---- halt -----------------------------------------------------------------
            lbl(6, 'Halt:', ['Stop what is moving, or bring the fleet down. Neither ' ...
                             'is part of the sequence above.']);
            p6 = app.pairGrid(g, 6);
            app.StopBtn = uibutton(p6, 'Text', 'Stop', ...
                'BackgroundColor', [0.8 0.2 0.2], 'FontColor', 'w', ...
                'Tooltip', ['Halt whatever is moving -- the simulation or playback -- ' ...
                            'immediately. The drones are left wherever the last frame ' ...
                            'put them; use Abort & Land to bring them down instead.'], ...
                'ButtonPushedFcn', @(~,~) app.stopAll(), 'Enable', 'off');
            app.AbortBtn = uibutton(p6, 'Text', 'Abort & Land', ...
                'BackgroundColor', [0.85 0.5 0.1], 'FontColor', 'w', ...
                'Tooltip', ['Abandon the show and bring the fleet straight down from ' ...
                            'wherever it is. The run keeps going until the drones ' ...
                            'are parked — this lands them, it does not stop the sim.'], ...
                'ButtonPushedFcn', @(~,~) app.abortAndLand(), 'Enable', 'off');

            % ---- the hint -------------------------------------------------------------
            % ONE label where there were two. Rewritten by setFlowHint on every delivery
            % change, so it always describes the state the panel is in rather than both
            % cases at once -- and it no longer has to say where the other button lives,
            % because the other button is six pixels away.
            app.FlowHintLabel = uilabel(g, 'Text', '', ...
                'FontSize', 10, 'FontColor', [0.35 0.35 0.35], 'WordWrap', 'on');
            app.FlowHintLabel.Layout.Row = 7;
            app.FlowHintLabel.Layout.Column = [1 4];
            app.setFlowHint();
        end

        function sub = pairGrid(~, g, row)
            % A 1x2 cell for a row holding two buttons that are ALTERNATIVES at the same
            % step, so they have to be the same width. It spans the outer grid's columns
            % 2-3 and stops there: column 4 exists for the Fly row's lamp, and spanning it
            % is what used to make Reset and Abort & Land ~30 px wider than the buttons
            % beside them -- a difference that reads as significance and is not. With the
            % nested split at the outer column boundary and the same 4 px spacing, each
            % half comes out exactly one outer column wide, so all six buttons in the panel
            % share one width and one pair of edges.
            sub = uigridlayout(g, [1, 2]);
            sub.ColumnWidth = {'1x', '1x'};
            sub.Padding = [0 0 0 0];
            sub.ColumnSpacing = 4;
            sub.Layout.Row = row;
            sub.Layout.Column = [2 3];
        end

        function h = stepLabel(~, g, row, txt, tip)
            % The step labels in column 1 of the run panel. Factored out only because
            % there are four of them and the property list is identical each time.
            h = uilabel(g, 'Text', txt, 'FontSize', 10, 'FontColor', [0.4 0.4 0.4], ...
                'HorizontalAlignment', 'right', 'Tooltip', tip);
            h.Layout.Row = row;
            h.Layout.Column = 1;
        end

        function s = trajSourceValue(app)
            % 1 = From Workspace straight into the controller, 2 = read back from
            % the onboard buffer the MAVLink upload filled. Matches setupParams.
            if startsWith(app.TrajSourceDropdown.Value, 'Workspace')
                s = 1;
            else
                s = 2;
            end
        end

        function trajSourceChanged(app)
            % Switching delivery path changes the timeline (skip_preflight drops
            % the upload and arm phases out of preShowDelay), so the parameters
            % have to be rebuilt before either button is pressed.
            app.stopPlayback();
            app.resetUploadState();
            app.UseSimData = false;
            app.SimTrajectoryData = [];
            app.SimTimeVector = [];
            app.SimPhaseData = [];
            app.ShowEntryTime = [];   % cleared with the phase log it is derived from
            app.SimUploadCount = [];
            % Same ordering trap as in resetAll: the label has to be re-derived AFTER the
            % sim data is dropped, or it keeps the '[Sim]' marker with nothing behind it.
            app.PlayBtn.Text = app.playIdleText();

            haveShow = ~isempty(app.TrajectoryData);
            % The hint names the fly button, and which one that is has just changed.
            app.setFlowHint();
            % The pose tap is a MAVLink-only choice. Workspace mode has no telemetry
            % downlink to display, so it always shows the true airframe pose; leaving the
            % dropdown live there would offer a choice that does nothing.
            if ~isempty(app.PoseViewDropdown) && isvalid(app.PoseViewDropdown)
                if app.trajSourceValue() == 1
                    app.PoseViewDropdown.Enable = 'off';
                else
                    app.PoseViewDropdown.Enable = 'on';
                end
            end
            if app.trajSourceValue() == 1
                app.UploadBtn.Enable = 'off';
                app.UploadLabel.Text = 'Not used in Workspace mode.';
                if haveShow
                    app.SimBtn.Enable = 'on';
                    app.updateStatus(['Workspace mode: the controller is fed the ' ...
                        'trajectory directly and the pre-flight is skipped. Click ' ...
                        'Simulate.']);
                else
                    app.updateStatus('Workspace mode selected. Generate a show.');
                end
            else
                app.SimBtn.Enable = 'off';
                app.UploadLabel.Text = 'Not uploaded.';
                if haveShow
                    app.UploadBtn.Enable = 'on';
                    app.updateStatus(['MAVLink mode: click Upload & Fly to stream ' ...
                        'the mission and fly it from the onboard buffer in one run.']);
                else
                    app.UploadBtn.Enable = 'off';
                    app.updateStatus('MAVLink mode selected. Generate a show.');
                end
            end
            % The flags go out either way, so the workspace always agrees with the
            % dropdown; the rebuild only matters once there is a plan to re-time.
            app.publishTrajSource();
            if haveShow
                app.pushTrajSource();
            end
        end

        function publishTrajSource(app)
            % Publish the delivery choice. traj_source picks the Variant Source
            % branch in DroneFleet; skip_preflight and upload_request decide
            % whether the supervisor runs a pre-flight at all.
            src = app.trajSourceValue();
            assignin('base', 'traj_source', src);
            assignin('base', 'skip_preflight', src == 1);
            assignin('base', 'upload_request', src == 2);
        end

        function pushTrajSource(app)
            % Publish the choice and rebuild the derived timing. The rebuild is
            % not optional: preShowDelay and total_sim_duration both depend on
            % skip_preflight, and the From Workspace timeseries is time-shifted
            % by preShowDelay.
            app.publishTrajSource();
            % The planner's transition-speed advisory is silenced HERE and nowhere
            % else. planShow cannot know the transition is too short until it has
            % computed the peak speed, so it warns as it discovers -- and on this
            % path the caller then either FIXES it (fitTransitionToPlan raises the
            % Transition field and re-plans) or names it in the status bar. Letting
            % the warning through as well puts a red twelve-line stack trace in
            % front of an operator whose show is already being repaired, which
            % reads as a crash rather than as the automatic re-timing it is.
            % Restored on the way out, so running setupParams from the command line
            % or from DroneShowExample still warns -- there the console IS the only
            % channel and the advisory is the whole point.
            ws = warning('off', 'setupParams:transitionTooFast');
            restore = onCleanup(@() warning(ws));   % restores on any exit, error included
            evalin('base', 'setupParams');
            app.cacheShowTimeline();
        end

        function html = uploadBarHTML(~)
            % Inline HTML for the embedded upload progress bar. Driven from
            % MATLAB by assigning app.UploadBar.Data = struct('pct', 0..100,
            % 'state', 'busy'|'ok'|'fail').
            % Built from divs rather than <progress>: the embedded Chromium
            % keeps the colour of ::-webkit-progress-value from its first paint,
            % so the fill could never turn red on failure. role="progressbar"
            % preserves the semantics.
            src = { ...
                '<!DOCTYPE html><html><head><style>', ...
                'html,body{margin:0;padding:0;overflow:hidden;background:transparent;', ...
                'font-family:Helvetica,Arial,sans-serif;}', ...
                '#wrap{display:flex;align-items:center;padding:1px 0;}', ...
                '#track{flex:1 1 auto;height:14px;background:#d8d8d8;', ...
                'border-radius:7px;overflow:hidden;}', ...
                '#fill{height:100%;width:0%;background:#3a7bd5;border-radius:7px;', ...
                'transition:width .12s linear,background .12s linear;}', ...
                '#pct{font-size:11px;width:36px;text-align:right;color:#333;}', ...
                '</style></head><body>', ...
                '<div id="wrap">', ...
                '<div id="track" role="progressbar" aria-valuemin="0" ', ...
                'aria-valuemax="100" aria-valuenow="0"><div id="fill"></div></div>', ...
                '<span id="pct">0%</span>', ...
                '</div>', ...
                '<script type="text/javascript">', ...
                'function setup(htmlComponent){', ...
                '  function render(){', ...
                '    var d = htmlComponent.Data || {};', ...
                '    var v = (typeof d.pct === "number") ? d.pct : 0;', ...
                '    if (v < 0) { v = 0; } else if (v > 100) { v = 100; }', ...
                '    var col = "#3a7bd5";', ...
                '    if (d.state === "ok") { col = "#2e9e4f"; }', ...
                '    else if (d.state === "fail") { col = "#cc3322"; }', ...
                '    var fill = document.getElementById("fill");', ...
                '    fill.style.width = v + "%";', ...
                '    fill.style.background = col;', ...
                '    document.getElementById("track")', ...
                '      .setAttribute("aria-valuenow", Math.round(v));', ...
                '    document.getElementById("pct").textContent = Math.round(v) + "%";', ...
                '  }', ...
                '  htmlComponent.addEventListener("DataChanged", render);', ...
                '  render();', ...
                '}', ...
                '</script></body></html>'};
            html = strjoin(src, newline);
        end

        function setUploadBar(app, pct, state)
            % state: 'busy' (in progress), 'ok' (complete), 'fail' (gave up)
            pct = max(0, min(100, pct));
            % Keep a trace so a test can prove the bar filled progressively
            % instead of snapping 0 -> 100 while the callback blocked.
            app.UploadBarHistory(end+1) = pct;
            if app.UseHtmlBar
                if ~isempty(app.UploadBar) && isvalid(app.UploadBar)
                    app.UploadBar.Data = struct('pct', pct, 'state', state);
                end
            elseif ~isempty(app.UploadGauge) && isvalid(app.UploadGauge)
                app.UploadGauge.Value = pct;
            end
        end

        function createPlaybackPanel(app, parent)
            % Play, the scrub bar and the speed slider: the controls in the app that only
            % ever REDRAW data that already exists. Nothing in here plans, flies or computes
            % anything, and that is the whole reason the panel exists -- they used to be the
            % "View:" row of a panel whose other two rows ran the model and commanded the
            % fleet, with the slider that paces them sitting one panel further down under
            % viewer settings. Grouping them with the control that governs them says more
            % than any label could.
            %
            % Stop deliberately did NOT come along. It stops the simulation as well as
            % playback, and a run is the thing you urgently need to stop, so it belongs next
            % to the fly buttons -- Play toggles to Pause, which is what this panel needs.
            % See createRunPanel.
            %
            % WHY THERE IS NO RESTART BUTTON any more. There was one, and everything it did
            % is now a place on the scrub bar: drag to the left end. Keeping it would have
            % been a button for one value of a control that offers every value, and it was
            % the wrong one to privilege -- "watch that transition again" wants a point in
            % the middle far more often than it wants t = 0. Play still restarts from the top
            % by itself once the show has run out, so the one case the button existed for
            % costs no click at all. What the bar cannot do is re-fly anything: it moves
            % where you are in cached frames, which is why it lives in this panel and not
            % next to Simulate.
            panel = uipanel(parent, 'Title', 'Playback');
            outer = uigridlayout(panel, [2, 1]);
            outer.RowHeight = {'fit', 'fit'};
            outer.Padding = [0 4 0 4];
            outer.RowSpacing = 2;

            % ONE grid for both slider rows, and that is the only reason the two bars line
            % up. They used to be two grids with their own columns -- {62,'1x',104} over
            % {88,'1x',40} -- so the scrub bar started 26 px right of the speed bar and
            % ended 64 px short of it, and two misaligned bars read as two unrelated
            % controls rather than one pair. Sharing the column set makes the alignment
            % structural instead of two numbers somebody has to remember to keep equal.
            %
            % Column 1 is 112 px because 'Playback speed:' wants about 92 at the default
            % font and had 88 -- it was clipped, and fixing that is what this width is
            % for. The Play button inherits the column and comes out wider than 'Pause'
            % needs, which costs nothing and makes the two rows start on one edge.
            % Column 3 is 104 to hold '1234.5 / 1234.5 s' without eliding; both readouts
            % are right-aligned into it, so they share their right edge as well.
            b = uigridlayout(outer, [2, 3]);
            b.ColumnWidth = {112, '1x', 104};
            b.RowHeight = {26, 26};   % both bars are tickless; see the SpeedSlider note
            b.Padding = [5 0 5 0];
            b.ColumnSpacing = 6;
            b.RowSpacing = 2;
            app.PlayBtn = uibutton(b, 'Text', 'Play', ...
                'Tooltip', ['Animate whatever data exists -- the planned trajectory, or ' ...
                            'the logged output of a finished run. Push again to pause. ' ...
                            'From the end of the show it starts again from the top. ' ...
                            'Paced by the speed slider below: 1.0x is real time.'], ...
                'ButtonPushedFcn', @(~,~) app.togglePlay(), 'Enable', 'off');

            % The scrub bar. Indexed in FRAMES, not seconds, because the frame is what the
            % animation is addressed by -- PlayIdx into a uniformly resampled series -- so
            % every position on the bar is a frame that exists and there is no rounding to
            % explain. The readout beside it is in seconds, which is the units the operator
            % thinks in, and the tooltip carries the count.
            %
            % Ticks off deliberately: a frame number is not a quantity anyone wants read off
            % an axis, and the labels cost ~20 px of panel for the privilege.
            %
            % ValueChangingFcn as well as ValueChangedFcn, so the fleet moves WHILE you drag
            % rather than jumping when you let go -- the difference between scrubbing to find
            % a moment and guessing at it. Both go through seekFrame, which is also what
            % keeps a drag from fighting the play loop for the axes.
            app.ScrubSlider = uislider(b, 'Limits', [1 2], 'Value', 1, ...
                'MajorTicks', [], 'MinorTicks', [], 'Enable', 'off', ...
                'Tooltip', ['Drag to any moment in the show. It moves the fleet as you ' ...
                            'drag, and it redraws cached frames only -- nothing is ' ...
                            're-planned, re-uploaded or re-flown. Dragging while it is ' ...
                            'playing pauses it for the drag and carries on where you let ' ...
                            'go. Disabled while a simulation is streaming: you cannot ' ...
                            'seek a run that has not happened yet.'], ...
                'ValueChangingFcn', @(~,e) app.seekFrame(e.Value, true), ...
                'ValueChangedFcn',  @(src,~) app.seekFrame(src.Value, false));
            app.ScrubTimeLabel = uilabel(b, 'Text', '—', 'FontSize', 11, ...
                'FontColor', [0.35 0.35 0.35], 'HorizontalAlignment', 'right', ...
                'Tooltip', 'Where the bar is: show time now, and the length of the show.');

            % The slider, moved here from the viewer panel. There used to be TWO speed
            % controls -- "Speed:" and "Live rate:" side by side with no indication that
            % they govern DIFFERENT runs: PlaySpeed is read only by advanceFrame (replaying
            % cached or logged data) and RateTarget only by nextPollInterval (a simulation
            % streaming right now). Renaming them helped and did not fix it. The live one is
            % gone: streaming is always paced to real time, and this is the control you
            % reach for to watch a finished run faster or slower.
            %
            % Row 2 of the grid above, not a grid of its own -- see the note there.
            uilabel(b, 'Text', 'Playback speed:', ...
                'Tooltip', ['How fast Play animates data that already exists -- ' ...
                            'either the planned trajectory or the logged output of a ' ...
                            'finished simulation. 1.0x is real time: one second of show ' ...
                            'per second of wall clock. This does nothing while a ' ...
                            'simulation is streaming -- a live run is always paced ' ...
                            'to real time.']);
            % TICKS OFF, and measured rather than chosen: a uislider shrinks its bar to
            % keep the outermost tick LABEL inside the cell, so with '0.25' under its left
            % end this slider came out 97.8 px wide against the scrub bar's 102.0 in the
            % same column -- the 4.2 px misalignment that made the two rows look unrelated
            % even after they were put in one grid. The labels were redundant anyway: the
            % readout to the right says the value exactly, which is more than '0.25' and
            % '4' at the ends ever did, and dropping them also takes this row from 46 px to
            % 26 in a column that has none to spare. The range moves into the tooltip.
            app.SpeedSlider = uislider(b, 'Value', 1, 'Limits', [0.25 4], ...
                'MajorTicks', [], 'MinorTicks', [], ...
                'Tooltip', ['0.25x to 4x. 1.0x is real time; the readout to the right ' ...
                            'is the current setting. Playback only -- a live run is ' ...
                            'always paced to real time.'], ...
                'ValueChangedFcn', @(src,~) app.setSpeed(src.Value));
            % Right-aligned to sit under the scrub readout above it. Found by Tag rather
            % than by a property, which is why it needs no handle of its own.
            uilabel(b, 'Text', '1.0x', 'Tag', 'SpeedVal', ...
                'HorizontalAlignment', 'right');

            uilabel(outer, ...
                'Text', ['Redraws data that already exists — never flies, plans or ' ...
                         'computes. 1.0x is real time; a live run is always paced to ' ...
                         'real time regardless of this slider.'], ...
                'FontSize', 10, 'FontColor', [0.35 0.35 0.35], 'WordWrap', 'on');
        end

        function setFlowHint(app)
            % Describe the state the Fly row is actually in. This label replaced TWO --
            % one in the old Controls panel and one in the old Trajectory Delivery panel --
            % each of which existed to describe the other panel's button. With both fly
            % buttons on one row it no longer has to say WHERE the other button is, only
            % why it is greyed, so it can say something useful instead.
            %
            % Guarded on the dropdown as well as the label, because panel creation order
            % is not fixed: whichever is built first calls this before the other exists,
            % and trajSourceValue dereferences the dropdown unguarded.
            if isempty(app.FlowHintLabel) || ~isvalid(app.FlowHintLabel) || ...
                    isempty(app.TrajSourceDropdown) || ~isvalid(app.TrajSourceDropdown)
                return;
            end
            if app.trajSourceValue() == 1
                app.FlowHintLabel.Text = ['Deliver = Workspace, so Simulate is live: the ' ...
                    'controller is fed the plan directly and the pre-flight is skipped. ' ...
                    'Upload & Fly is greyed — there is no upload on this path, and the ' ...
                    'viewer shows the true pose rather than the downlink.'];
            else
                app.FlowHintLabel.Text = ['Deliver = MAVLink, so Upload & Fly is live: the ' ...
                    'mission is streamed over the radio and flown from the onboard buffer ' ...
                    'in one run. Simulate is greyed — switch Deliver to Workspace for it.'];
            end
        end

        function createViewerPanel(app, parent)
            % TITLED, and the title matters: this was the one block in the column with no
            % title at all, which put the only group that changes nothing about the flight
            % directly under the button panel where it read as more of them. Everything here is
            % display -- none of it is pushed to the model, and none of it needs a
            % re-Generate.
            %
            % The speed slider USED to be the first row in here and is now in the Playback
            % panel, next to the two buttons it paces. What is left is genuinely one
            % category: how the scene is drawn, plus which signal it is drawn from. The
            % title dropped "& speed" with it.
            panel = uipanel(parent, 'Title', 'Viewer (display only)');

            % Each row gets its own sub-grid. Sharing one column set made them fight:
            % a wide middle column starved the rendering dropdown until "Quadrotor
            % meshes" truncated to an ellipsis.
            outer = uigridlayout(panel, [4, 1]);
            outer.RowHeight = {'fit', 'fit', 'fit', 'fit'};
            outer.Padding = [0 4 0 4];
            outer.RowSpacing = 2;

            d = uigridlayout(outer, [1, 4]);
            d.ColumnWidth = {55, 78, 52, '1x'};
            d.Padding = [5 0 5 0];
            d.ColumnSpacing = 4;

            uilabel(d, 'Text', 'Camera:');
            app.CameraModeDropdown = uidropdown(d, ...
                'Items', {'Static', 'Dynamic'}, 'Value', 'Static', ...
                'ValueChangedFcn', @(src,~) app.setCameraMode(src.Value));

            uilabel(d, 'Text', 'Drones:');
            app.RenderDropdown = uidropdown(d, ...
                'Items', {'Quadrotor meshes', 'Spheres', 'Markers only'}, ...
                'Value', app.DroneRender, ...
                'Tooltip', ['How each drone is drawn. Markers only and Spheres are ' ...
                            'the same single scatter3 update, small dots against ' ...
                            'large ones. Meshes draw the UAV Toolbox quadrotor ' ...
                            'geometry — far better looking, but 792 vertices per ' ...
                            'drone rewritten every frame, which is what makes a ' ...
                            'large fleet crawl. Above about 100 drones use either ' ...
                            'of the scatter modes rather than meshes — measured at ' ...
                            '50 drones the two cost the same 17 ms per frame, so ' ...
                            'pick between them on looks.'], ...
                'ValueChangedFcn', @(src,~) app.setRenderMode(src.Value));

            % The pose tap. On its own row because it is the one control in this panel
            % that changes WHICH SIGNAL is read rather than how it is drawn, and it only
            % means anything on the MAVLink path -- the Workspace path has no downlink to
            % choose against. trajSourceChanged greys it out there.
            %
            % ABOVE the two checkboxes rather than below them, which is the order the
            % panel now reads in: pick the signal, then pick what is drawn on top of it.
            % The checkboxes are also the pair that grows -- there are two now where there
            % was one -- so they belong at the end of the group rather than in the middle.
            d3 = uigridlayout(outer, [1, 2]);
            d3.ColumnWidth = {88, '1x'};
            d3.Padding = [5 0 5 0];
            d3.ColumnSpacing = 4;

            uilabel(d3, 'Text', 'Pose shown:', ...
                'Tooltip', 'Which pose the 3-D view reads during a MAVLink run.');
            app.PoseViewDropdown = uidropdown(d3, ...
                'Items', {'As received (downlink)', 'True airframe pose'}, ...
                'ItemsData', {'downlink', 'true'}, ...
                'Value', app.PoseView, ...
                'Tooltip', ['As received shows the ground station''s telemetry table. ' ...
                            'The fleet takes turns on the link, one drone per time ' ...
                            'step, so a whole ' ...
                            'fleet refreshes in N x 0.01 s — 0.2 s at 20 drones but ' ...
                            '5 s at 500, which is why a large fleet appears to update ' ...
                            'row by row. Measured at 120 drones, only 26% of the fleet ' ...
                            'moves between frames — which you see because a live run is ' ...
                            'paced to real time; let the model outrun the wall clock and ' ...
                            'the table refreshes fully between frames, hiding it. ' ...
                            'It is what the operator of a real show actually sees. ' ...
                            'True airframe pose taps the flight ' ...
                            'dynamics directly: every drone, every frame, no radio in ' ...
                            'the way. Use it to judge the flight; use As received to ' ...
                            'judge the link. Neither changes the flight itself.'], ...
                'ValueChangedFcn', @(src,~) app.setPoseView(src.Value));

            % The two overlays, side by side because they are the same kind of choice --
            % extra lines drawn over the fleet, neither of which touches the flight -- and
            % because they are each other's contrast: trails are where the drones HAVE
            % been, paths are where they are GOING. 118 px is what 'Show trails' needs
            % with its box; the second column takes the rest.
            d2 = uigridlayout(outer, [1, 2]);
            d2.ColumnWidth = {118, '1x'};
            d2.Padding = [5 0 5 0];
            d2.ColumnSpacing = 4;

            app.TrailsCheckBox = uicheckbox(d2, 'Text', 'Show trails', ...
                'Value', app.ShowTrails, ...
                'Tooltip', ['The short light streak behind each drone. One ' ...
                            'animatedline per drone, updated per frame, so turning ' ...
                            'it off is worth real time on a large fleet.'], ...
                'ValueChangedFcn', @(src,~) app.setShowTrails(src.Value));

            app.TrajCheckBox = uicheckbox(d2, 'Text', 'Show trajectory', ...
                'Value', app.ShowTrajectory, ...
                'Tooltip', ['Every drone''s planned path for the phase on screen: the ' ...
                            'transition being flown, or -- while the fleet holds a ' ...
                            'formation -- the one it is about to fly. It changes by ' ...
                            'itself as the show moves on, so grid gives way to ' ...
                            'grid -> circle, then circle -> the next shape. Always the ' ...
                            'PLAN, which is why it works during a live run too, and why ' ...
                            'a gap between a line and its drone is worth looking at. ' ...
                            'Cheaper than trails: one line object for the whole fleet, ' ...
                            'rebuilt only when the phase changes rather than per frame.'], ...
                'ValueChangedFcn', @(src,~) app.setShowTrajectory(src.Value));

            % The render advice, and the two things it says are the two the measurement
            % supports: avoid meshes on a big fleet, and turn trails off. Markers-vs-Spheres
            % is not one of them -- at 50 drones both drew in 17 ms, and trails alone took
            % that to 37 ms. That pair was measured on the SCRUB path, which then also wiped
            % the trails on every event; timed on the PLAYBACK path, which only ever adds
            % points, trails are +17% a frame at 10 drones and +81% at 60. The advice is the
            % same either way, but the fleet size is the whole story, so the label says so.
            % The speed-scope sentence that used to lead this label went with the slider to
            % the Playback panel, where the buttons it describes now are.
            app.RenderHintLabel = uilabel(outer, ...
                'Text', ['Above ~100 drones: a scatter mode rather than meshes, and ' ...
                         'trails off — they cost an update per drone per frame (+81% at ' ...
                         '60 drones). None of this changes the flight.'], ...
                'FontSize', 10, 'FontColor', [0.35 0.35 0.35], 'WordWrap', 'on');
        end

        function setShowTrails(app, val)
            % Handled live, unlike the render mode: creating or deleting
            % animatedlines does not touch the axes' other children, so there is no
            % cla and nothing the running poll loop holds a handle to gets
            % invalidated. Off deletes the handles rather than hiding them, because
            % a hidden animatedline still costs its addpoints every frame.
            app.ShowTrails = logical(val);
            if ~isempty(app.TrailsCheckBox) && isvalid(app.TrailsCheckBox)
                app.TrailsCheckBox.Value = app.ShowTrails;
            end
            if app.ShowTrails
                app.createTrails();
                app.updateStatus('Light trails on.');
            else
                for k = 1:numel(app.TrailPlots)
                    if isvalid(app.TrailPlots(k))
                        delete(app.TrailPlots(k));
                    end
                end
                app.TrailPlots = gobjects(0);
                app.updateStatus(['Light trails off — one addpoints per drone per ' ...
                    'frame saved.']);
            end
        end

        function setShowTrajectory(app, val)
            % Live, like the trails: one line object appears or goes away, no cla, so
            % nothing a running poll loop holds a handle to is invalidated.
            app.ShowTrajectory = logical(val);
            if ~isempty(app.TrajCheckBox) && isvalid(app.TrajCheckBox)
                app.TrajCheckBox.Value = app.ShowTrajectory;
            end
            if app.ShowTrajectory
                % Draw at the frame already on screen rather than waiting for the next
                % one: with playback paused -- which is when anyone reads a path -- there
                % is no next frame, and the checkbox would look dead.
                app.refreshTrajOverlay();
                if isempty(app.TrajectoryData)
                    app.updateStatus(['Planned paths on — they appear from the next ' ...
                        'Generate, since there is no plan to draw yet.']);
                else
                    app.updateStatus(['Planned paths on — every drone''s path for the ' ...
                        'phase on screen: the transition being flown, or the next one ' ...
                        'while the fleet holds. It follows the show by itself.']);
                end
            else
                app.clearTrajOverlay();
                app.updateStatus('Planned paths off.');
            end
        end

        function refreshTrajOverlay(app)
            % Redraw the overlay at the frame currently on screen, for the paths that are
            % NOT followed by a drawFrame. generateShow and the post-run replay rebuild
            % both leave the fleet exactly where buildScenario placed it and draw no frame
            % of their own, so without this the overlay would stay missing until the
            % operator pressed Play -- i.e. the checkbox would look broken in the two
            % moments it is most likely to be ticked. Both call sites are AFTER PlayIdx has
            % been parked, which is what makes "the frame currently on screen" true.
            app.updateTrajOverlay(app.currentFrameTime());
        end

        function t = currentFrameTime(app)
            % The time of the frame currently on screen, in the units drawFrame is handed
            % it: absolute for logged or streaming data, show-relative for the plan. Empty
            % when there is no data at all. formationAt and the overlay both expect this
            % convention, which is the reason it is derived in one place.
            t = [];
            if app.UseSimData && ~isempty(app.SimTimeVector)
                tv = app.SimTimeVector;
            elseif ~isempty(app.TimeVector)
                tv = app.TimeVector;
            else
                return;
            end
            t = tv(min(max(app.PlayIdx, 1), numel(tv)));
        end

        function ensureTrajPlot(app)
            % The single line the whole overlay lives in, created on demand. buildScenario
            % cla's the axes and so destroys it, and that is the usual reason it is missing.
            if ~isempty(app.TrajPlot) && all(isvalid(app.TrajPlot))
                return;
            end
            if isempty(app.ScenarioAxes) || ~isvalid(app.ScenarioAxes)
                return;   % nothing built yet; the next buildScenario will come back here
            end
            % hold on defensively, for the same reason createTrails does it: with hold off
            % this plot3 would CLEAR the axes the live poll loop is drawing into.
            hold(app.ScenarioAxes, 'on');
            % One colour for the whole overlay rather than each drone's own. These lines
            % are the plan, not the fleet, and drawing them in the formation palette would
            % say "drone" at sixty places along every path -- exactly the confusion the
            % overlay is meant to resolve. A pale blue-grey reads as scaffolding against
            % the dark ground plane and against any palette the operator picks, and thin
            % keeps 500 of them from filling the sky.
            app.TrajPlot = plot3(app.ScenarioAxes, NaN, NaN, NaN, '-', ...
                'Color', [0.55 0.70 0.95], 'LineWidth', 0.75);
            app.TrajSegKey = [];   % a new line holds no phase yet
        end

        function clearTrajOverlay(app)
            app.TrajPlot = app.TrajPlot(isvalid(app.TrajPlot));
            delete(app.TrajPlot);
            app.TrajPlot = gobjects(0);
            app.TrajSegKey = [];
        end

        function win = trajPhaseWindow(app, tShow)
            % Which slice of the plan counts as "this phase"? [tStart tEnd] in
            % show-relative seconds, or [NaN NaN] for nothing to draw.
            %
            % The rule is the one the panel promises: while the fleet is MOVING, show the
            % move it is making; while it HOLDS, show the move it is about to make. So the
            % opening climb shows the climb, the grid hold shows grid -> circle, and the
            % circle hold shows circle -> whatever is next. A transition and the hold
            % before it therefore draw the SAME paths, and that is the point rather than a
            % coincidence -- the paths appear the moment the fleet arrives and stay up
            % while it flies them, so nothing flickers at the phase boundary.
            %
            % [NaN NaN] rather than [] for "nothing", because isequal treats NaN as equal
            % to itself: the caller's "same phase as last frame?" test then costs one
            % isequal in the landed case too, instead of falling through and recomputing.
            win = [NaN NaN];
            tt = app.TimelineTimes;
            ty = app.TimelineTypes;
            if isempty(tt) || isempty(ty) || isempty(app.TimeVector)
                return;
            end
            tEndAll = app.TimeVector(end);

            % The descent lies PAST the end of the timeline -- TimelineTimes stops at the
            % start of the last hold -- so it is checked before the lookup, exactly as
            % formationAt does and for the same reason.
            if ~isempty(app.PlanLandStart) && tShow >= app.PlanLandStart
                if isempty(app.PlanLandEnd)
                    win = [app.PlanLandStart, tEndAll];
                elseif tShow < app.PlanLandEnd
                    win = [app.PlanLandStart, app.PlanLandEnd];
                end
                return;   % once landed there is no next move to show
            end

            seg = find(tShow >= tt, 1, 'last');
            if isempty(seg)
                win = [0, tt(1)];   % still climbing to formation 1
                return;
            end
            if ty(seg) == 0
                seg = seg + 1;      % a hold: the move to show is the one after it
            end
            if seg > numel(tt)
                % The last hold, whose next move is the descent if the plan has one.
                if ~isempty(app.PlanLandStart)
                    if isempty(app.PlanLandEnd)
                        win = [app.PlanLandStart, tEndAll];
                    else
                        win = [app.PlanLandStart, app.PlanLandEnd];
                    end
                end
            elseif seg < numel(tt)
                win = [tt(seg), tt(seg + 1)];
            else
                win = [tt(seg), tEndAll];
            end
        end

        function updateTrajOverlay(app, t)
            % Redraw the overlay IF the phase has changed. Called from drawFrame, so the
            % common path through here has to be cheap: one timeline lookup and one
            % isequal, then out.
            if ~app.ShowTrajectory || isempty(t)
                return;
            end
            % ALWAYS THE PLAN, even when logged [Sim] data is what is being animated, and
            % the second reason is the deciding one. First, "the trajectory for this phase"
            % means where the fleet is being sent -- a path is a commanded thing, and
            % comparing it against the drones drawn on top of it is the whole value of the
            % overlay. Second, the plan is the only series that extends into a phase that
            % has not happened yet, which is precisely what a hold has to show: a logged
            % run reaches the current instant and no further, so during a live stream --
            % the one time knowing where the fleet is about to go matters most -- the next
            % transition would be undrawable.
            if isempty(app.TrajectoryData) || isempty(app.TimeVector)
                app.clearTrajOverlay();
                return;
            end
            if app.UseSimData
                tShow = t - app.showAnchor();
            else
                tShow = t;
            end

            win = app.trajPhaseWindow(tShow);
            app.ensureTrajPlot();
            if isempty(app.TrajPlot) || ~all(isvalid(app.TrajPlot))
                return;   % no axes yet
            end
            if isequal(win, app.TrajSegKey)
                return;   % same phase, same paths: this is the per-frame exit
            end
            app.TrajSegKey = win;
            if any(isnan(win))
                app.blankTrajOverlay();
                return;
            end

            % The frames inside the phase, thinned. The paths are smooth minimum-jerk
            % curves, so ~60 points per drone is indistinguishable from all 400 of them and
            % is a seventh of the data to push into the line. The LAST frame is always
            % kept: dropping it would stop every path short of the formation it is the
            % path TO, which is the one place the eye checks.
            k = find(app.TimeVector >= win(1) & app.TimeVector <= win(2));
            if numel(k) < 2
                app.blankTrajOverlay();
                return;
            end
            stride = max(1, floor(numel(k) / 60));
            k = unique([k(1:stride:end); k(end)]);

            N = size(app.TrajectoryData, 1);
            M = numel(k);
            xs = reshape(app.TrajectoryData(:, 1, k), N, M);
            ys = reshape(app.TrajectoryData(:, 2, k), N, M);
            zs = -reshape(app.TrajectoryData(:, 3, k), N, M);   % NED -> plot Z up
            % One line for the fleet, drone paths separated by NaN. See the property
            % comment: N Line objects would draw the same picture and cost N handles per
            % phase change for it.
            nanCol = NaN(N, 1);
            set(app.TrajPlot, ...
                'XData', reshape([xs, nanCol]', 1, []), ...
                'YData', reshape([ys, nanCol]', 1, []), ...
                'ZData', reshape([zs, nanCol]', 1, []));
        end

        function blankTrajOverlay(app)
            % Nothing to draw, but keep the handle. NaN rather than [] because assigning
            % empty XData to a line whose YData is not empty is an inconsistent-size
            % error, not an empty plot -- the same trap as the deny rings.
            set(app.TrajPlot, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        end

        function createTrails(app)
            % The trails for the CURRENT axes and fleet. Called from buildScenario and
            % from the checkbox, so the two cannot drift apart.
            if ~app.ShowTrails || isempty(app.NumUAVs) || app.NumUAVs < 1
                app.TrailPlots = gobjects(0);
                return;
            end
            if ~isempty(app.TrailPlots) && all(isvalid(app.TrailPlots)) && ...
                    numel(app.TrailPlots) == app.NumUAVs
                return;   % already there; the checkbox was ticked twice
            end
            if isempty(app.LastFormType)
                if isempty(app.FormSeq), ftype = 1; else, ftype = app.FormSeq(1); end
            else
                ftype = app.LastFormType;
            end
            colors = app.getUAVColors(ftype);
            trailLen = 12;
            % Defensive, and not theoretical: animatedline obeys NextPlot, so with
            % hold off the first one would CLEAR the axes the live poll loop is
            % drawing into. buildScenario always leaves hold on; this makes the
            % checkbox safe even if some future path does not.
            hold(app.ScenarioAxes, 'on');
            app.TrailPlots = gobjects(app.NumUAVs, 1);
            for k = 1:app.NumUAVs
                app.TrailPlots(k) = animatedline(app.ScenarioAxes, ...
                    'MaximumNumPoints', trailLen, ...
                    'Color', colors(k,:), 'LineWidth', 1.5);
            end
        end

        function setRenderMode(app, val)
            app.DroneRender = val;
            % Say it where it is actionable rather than only in a tooltip nobody
            % hovers: the fleet size is known here, and meshes at 200 drones is the
            % one choice that turns a watchable stream into a slideshow. Carried as a
            % prefix rather than sent as its own message -- a second updateStatus
            % would overwrite the first before it had been read.
            N = app.NumUAVSpinner.Value;
            note = '';
            if strcmp(val, 'Quadrotor meshes') && N > 50
                note = sprintf(['%d drones as meshes is %d vertex rewrites per ' ...
                    'frame — Spheres or Markers only is recommended at this fleet ' ...
                    'size. '], N, N * 792);
            end
            if app.SimRunning
                % buildScenario would cla the axes the live poll loop is drawing
                % into, invalidating the handles it holds.
                app.updateStatus([note 'Drone rendering set to ' val ...
                    ' — it takes effect after the current run.']);
                return;
            end
            if isempty(app.TrajectoryData) && isempty(app.SimTrajectoryData)
                if ~isempty(note)
                    app.updateStatus([note 'It will apply from the next Generate.']);
                end
                return;   % nothing built yet; buildScenario will pick it up
            end
            wasPlaying = app.Playing;
            app.pausePlayback();
            app.buildScenario();
            app.drawCurrentFrame();
            if wasPlaying
                app.startPlayback();
            else
                app.updateStatus([note 'Drone rendering: ' val '.']);
            end
        end

        function warnFleetCost(app)
            % Say what a big fleet costs BEFORE it is generated and flown, because the
            % cost is not in the geometry and is therefore invisible from the panel.
            %
            % The MAVLink upload is the term that matters: one MISSION_ITEM_INT per
            % waypoint per drone at one packet per tick. That is deliberately linear in
            % the fleet and deliberately left linear -- the protocol detail IS the point
            % of the base station -- so a 500-drone show spends most of its sim seconds on
            % the ground uploading, not flying. Raising packets_per_tick would hide that,
            % which is why the honest fix is a warning and a documented alternative.
            N = app.NumUAVSpinner.Value;
            if N <= 100
                return;               % under a minute of upload; not worth a message
            end
            % Scale from the last plan actually built rather than from a hard-coded
            % constant: num_waypoints depends on the sequence, the transition and the hold,
            % so a per-drone figure measured on THIS operator's settings is the only one
            % that is right. Falls back to the default 5-formation sequence's 1.09 s per
            % drone when nothing has been generated yet.
            perDrone = 1.09;
            src = 'estimated for the default sequence';
            try
                if evalin('base', 'exist(''upload_duration'', ''var'')') == 1 && ...
                        evalin('base', 'exist(''N_uav'', ''var'')') == 1
                    nPrev = evalin('base', 'N_uav');
                    if nPrev >= 1
                        perDrone = evalin('base', 'upload_duration') / nPrev;
                        src = 'scaled from the last plan built';
                    end
                end
            catch
                % Leave the default. A base workspace that cannot be read here is not a
                % reason to withhold the warning.
            end
            uploadS = perDrone * N;
            if strcmp(app.TrajSourceDropdown.Value, 'Workspace (quick)')
                % Quick mode skips the upload entirely, so the warning would be false.
                app.updateStatus(sprintf(['%d drones. Workspace (quick) skips the ' ...
                    'MAVLink upload, so this stays fast — switching to the full ' ...
                    'protocol path would add about %.0f s of sim time (%s).'], ...
                    N, uploadS, src));
                return;
            end
            app.updateStatus(sprintf(['%d drones on the full MAVLink path: about ' ...
                '%.0f s of sim time is spent uploading the mission before the fleet ' ...
                'leaves the ground (%s), and the wall clock is longer than that. ' ...
                'Set Trajectory source to Workspace (quick) to skip the upload if you ' ...
                'are here for the flying rather than the protocol.'], N, uploadS, src));
        end

        function drawCurrentFrame(app)
            % Repaint wherever playback is sitting, so a rendering change shows
            % immediately instead of waiting for the next Play.
            if app.UseSimData && ~isempty(app.SimTrajectoryData)
                data = app.SimTrajectoryData; tv = app.SimTimeVector;
                srcLabel = ' [Simulated]';
            elseif ~isempty(app.TrajectoryData)
                data = app.TrajectoryData; tv = app.TimeVector;
                srcLabel = '';
            else
                return;
            end
            idx = min(max(app.PlayIdx, 1), size(data, 3));
            app.drawFrame(squeeze(data(:, 1:3, idx)), tv(idx), srcLabel);
        end

        function loadQuadMesh(app)
            % Vertices and faces of the UAV Toolbox quadrotor, fetched once.
            % uavPlatform exposes the geometry as an extendedObjectMesh on its
            % .Mesh property; there is no lighter public accessor, so a
            % throwaway uavScenario is the cheapest way to reach it.
            if ~isempty(app.MeshV0)
                return;
            end
            try
                sc = uavScenario('UpdateRate', 100);
                plat = uavPlatform('MeshProbe', sc, 'InitialPosition', [0 0 0]);
                updateMesh(plat, 'quadrotor', {app.MeshScale}, [1 0 0], ...
                    [0 0 0], [1 0 0 0]);
                V = plat.Mesh.Vertices;
                app.MeshF = plat.Mesh.Faces;
                % Taken as it comes, deliberately. This used to negate Z "because
                % the mesh is body NED and the viewer plots Z up", and that flipped
                % every quadrotor upside down. Measured rather than reasoned about,
                % because an 8-vertex sample cannot tell you which way up a mesh is:
                % the rotor discs are two 307-vertex rings at radius 1.000 sitting at
                % raw z = +0.007, while the body and legs run down to z = -0.129. The
                % rotors are already ABOVE the airframe, so the geometry is Z-up as
                % delivered and -V(:,3) inverted it. (It was also a reflection, not a
                % rotation, so the faces wound the wrong way as well.)
                app.MeshV0 = V;
            catch e
                app.MeshV0 = [];
                app.MeshF = [];
                app.updateStatus(['Quadrotor mesh unavailable (' e.message ...
                    ') — drawing spheres instead.']);
            end
        end

        function createTelemetryPanel(app, parent)
            panel = uipanel(parent, 'Title', 'Telemetry');
            g = uigridlayout(panel, [8, 1]);
            g.RowHeight = repmat({'fit'}, 1, 8);
            g.Padding = [5 5 5 5]; g.RowSpacing = 2;

            app.TimeLabel = uilabel(g, 'Text', 'Time: 0.0 / 0.0 s', 'FontWeight', 'bold');
            app.PhaseLabel = uilabel(g, 'Text', 'Phase: --', 'FontWeight', 'bold');
            app.FormLabel = uilabel(g, 'Text', 'Formation: --');
            app.StateLabel = uilabel(g, 'Text', 'Show State: Idle');
            app.MinSepLabel = uilabel(g, 'Text', 'Min Separation: -- m');
            app.MaxErrLabel = uilabel(g, 'Text', 'Max Speed: -- m/s');
            app.SpeedLabel = uilabel(g, 'Text', 'Fleet Size: --');
            % Live only. Playback has its own Speed slider and no solver to share the
            % thread with, so this stays '--' outside a streamed run rather than
            % reporting a number that means something else.
            app.RateLabel = uilabel(g, 'Text', 'Live Rate: --', ...
                'FontSize', 10, 'FontColor', [0.35 0.35 0.35], ...
                'Tooltip', ['Real-time factor the live stream is actually achieving, ' ...
                            'against the 1x it is paced to. 1.00x means a second of show ' ...
                            'per second of wall clock; below that the fleet is too big ' ...
                            'for the solver to keep up, and this says so.']);
        end

        function createStatusBar(app, parent)
            % This row carries the app's LONGEST strings -- the UNFLYABLE PLAN refusal runs
            % past 450 characters, because a refusal has to say what was asked, what the
            % limit is and what to change -- and it used to be a bare uilabel with no
            % WordWrap in a fixed 25 px row. One line of ~60 characters showed and the rest
            % was simply gone, so the messages that mattered most were the least readable.
            %
            % WordWrap turns it into as many lines as the text needs, and leftScroll's row 9
            % is 'fit' so the row follows. That pairing is the whole fix, and it works
            % because a 'fit' row asks a WordWrap label for its WRAPPED height -- i.e. MATLAB
            % does the line counting. Measured in this column, at its 357 px: a one-line
            % "Setup complete" gets 16 px, the 320-character big-fleet warning 70 px, the
            % 280-character "Show ready" 56 px, and a 460-character refusal 110 px, all of
            % which come to ~13.9 px per wrapped line. Worth writing down, because the
            % obvious alternative was to estimate the line count from a per-character width
            % and set a pixel height -- a magic constant standing in for a number the
            % framework already knows exactly.
            %
            % Tooltip (set in updateStatus) mirrors the text anyway. It costs nothing and it
            % does not depend on the layout being right.
            app.StatusBar = uilabel(parent, 'Text', '', ...
                'FontColor', [0.1 0.1 0.5], 'FontSize', 11, ...
                'WordWrap', 'on', 'VerticalAlignment', 'top');
        end

        %% ---- Generate Show ----
        function generateShow(app)
            app.updateStatus('Generating show plan...');
            drawnow;

            try
                app.stopPlayback();
                N = app.NumUAVSpinner.Value;

                % Parse formation sequence
                formStr = app.FormationList.Value;
                formSeq = app.parseFormationString(formStr);

                % Push parameters to base workspace
                assignin('base', 'N_uav', N);
                assignin('base', 'formation_spacing', app.SpacingField.Value);
                assignin('base', 'show_altitude', -app.AltitudeField.Value);
                assignin('base', 'transition_duration', app.TransitionField.Value);
                assignin('base', 'hold_duration', app.HoldField.Value);
                % Rotation is a PLAN setting, not a view one: it turns each hold from a
                % single anchor into a sample series, so it changes what is uploaded and
                % what the drones fly, not just what the viewer draws.
                %
                % The per-step ANGLES are what setupParams acts on; formation_rotate is
                % pushed too because a stale true left in the workspace would otherwise
                % make an all-zero request expand back to a full turn on every hold.
                % Unticking the box is a MUTE: it pushes zeros without discarding the
                % angles on the panel, so ticking it again brings the same show back.
                app.syncRotationLength();
                if app.RotateCheckbox.Value
                    rotReq = app.RotationDegPerStep;
                else
                    rotReq = zeros(1, numel(formSeq));
                end
                assignin('base', 'rotation_deg_request', rotReq);
                assignin('base', 'formation_rotate', app.RotateCheckbox.Value);
                assignin('base', 'v_max', app.VmaxField.Value);
                assignin('base', 'a_max', app.AmaxField.Value);
                assignin('base', 'd_min', app.DminField.Value);
                assignin('base', 'formation_sequence', formSeq);
                assignin('base', 'num_formations', numel(formSeq));
                % The clouds any type >= 5 in formSeq refers to. Pushed on every
                % Generate, not once at load: setupParams re-samples them for the
                % current fleet size, so this has to be there whenever it runs.
                assignin('base', 'custom_formations', app.CustomFormations);
                % What formation type 4 spells. This is what makes the picker's
                % built-in "Text" entry honest rather than a fourth fixed shape: it
                % renders whatever is in the field. An empty field is left alone so
                % setupParams keeps its own default instead of being handed ''.
                txt = strtrim(app.TextField.Value);
                if ~isempty(txt)
                    assignin('base', 'formation_text', txt);
                end

                % No nav_mode here any more. It was pushed for its own sake -- nothing in
                % the model has ever read it -- and the dropdown that set it is now the
                % RTK tier readout instead.
                %
                % Resize the degradation mask to the fleet being generated, keeping the
                % drones already named. setupParams would otherwise do it for us -- but
                % its guard REPLACES a wrong-length mask with all-false, so changing the
                % fleet size would quietly undo a Degrade the operator can still see on
                % the panel. Better to carry it across explicitly and update the label.
                app.publishDenyMask(app.currentDenyMask(N));
                app.refreshDenyLabel();
                assignin('base', 'uav_loss_rate', app.UavLossField.Value / 100);
                assignin('base', 'comm_latency', app.LatencyField.Value / 1000);
                assignin('base', 'comm_jitter', app.JitterField.Value / 1000);
                assignin('base', 'packet_loss_rate', app.PacketLossField.Value / 100);
                assignin('base', 'comm_timeout', app.TimeoutField.Value);
                % Generating a show cannot inherit an abort. AbortReqConst evaluates
                % this expression when the model compiles, so a stale true left in the
                % workspace would put the next run into ABORT_LANDING out of ARMED.
                assignin('base', 'abort_request', false);
                % Per-formation colours the operator picked. Pushed as a full table
                % with NaN for "not chosen", so setupParams overrides row by row and
                % keeps its own defaults for the rest -- the app never has to hold a
                % second copy of the default palette.
                assignin('base', 'lighting_colors_override', app.ColorOverrides);

                % Run setup. pushTrajSource publishes the delivery choice first,
                % because setupParams derives preShowDelay and the total duration
                % from it, then runs setupParams itself.
                app.pushTrajSource();

                % Cache trajectory and lighting data
                app.TrajectoryData = evalin('base', 'trajectory_data');
                app.TimeVector = evalin('base', 'time_vector');
                % Onto a uniform grid before anything draws it. The planned grid is a
                % KEYFRAME grid, deliberately sparse to bound the upload, and it is far
                % too sparse to animate: measured on the 6-drone default it is 256
                % samples over 87.07 s = 2.93 Hz, with dt running 0.10, 0.14, 0.60,
                % 2.0 and 5.0 s. The long ones are the HOLDS, so Play at 1.0x froze the
                % picture for up to five seconds and then jumped -- which reads as
                % playback being broken rather than as a formation being held, and is
                % why raising Playback speed appeared to make it "more realistic":
                % sparse data consumed faster just means more frames per second of wall
                % clock. Resampled here, once, rather than in advanceFrame, so the
                % extent pass, the formation preview and the Max Speed readout all see
                % the same uniform grid the animation does.
                app.resampleViewerTrajectory();
                app.LightingData = evalin('base', 'lighting_timeline');
                app.NumUAVs = N;
                app.NumSamples = size(app.TrajectoryData, 3);
                app.FormationColors = evalin('base', 'lighting_colors');
                % The resolved palette, overrides already folded in by setupParams, so
                % the swatch shows what flew rather than what was asked for.
                app.refreshColorSwatch();
                % An over-cap angle comes down to what the plan actually flew, BEFORE the
                % readout is built: refreshRotationLabel names the steps off the panel, so
                % the two have to be clamped and read in that order or the label quotes an
                % angle the field no longer holds.
                nTrim = app.clampRotationToCap(rotReq);
                % Now there is a plan, so the rotation readout can report what the sweep
                % actually came out as instead of estimating it.
                app.refreshRotationLabel(nTrim);

                % Reset sim data and build scenario
                app.UseSimData = false;
                app.SimTrajectoryData = [];
                app.SimTimeVector = [];
                app.buildScenario();

                % A new plan invalidates any previous upload: the fleet is
                % holding the old waypoints, so it must be re-uploaded before
                % the show can be flown.
                app.resetUploadState();

                app.PlayBtn.Enable = 'on';
                % Not playIdleText's default by accident: the sim dataset was dropped four
                % lines up, and stopPlayback set this label before that happened, so the
                % '[Sim]' marker would otherwise outlive the data it points at.
                app.PlayBtn.Text = app.playIdleText();
                app.PlayIdx = 1;
                app.refreshScrub();   % new plan, new length: re-limit the bar and park it
                app.refreshTrajOverlay();   % buildScenario cla'd it; nothing else redraws

                % Only one of the two run buttons is live, depending on how the
                % waypoints are being delivered.
                if app.trajSourceValue() == 1
                    app.UploadBtn.Enable = 'off';
                    app.SimBtn.Enable = 'on';
                    app.UploadLabel.Text = 'Not used in Workspace mode.';
                    nextStep = 'Simulate to fly it';
                else
                    app.UploadBtn.Enable = 'on';
                    app.SimBtn.Enable = 'off';
                    nextStep = 'Upload & Fly to stream the mission and fly it';
                end

                showDur = evalin('base', 'show_duration');
                minSep = evalin('base', 'min_sep_achieved');

                % setupParams warns when the plan is unflyable, but it runs under
                % evalin and warnings never reach the UI -- so a show that will
                % visibly fall apart at the end used to report "Show ready". Check
                % the numbers it computed and say so instead. Both constraints are
                % surfaced, because either can be the one that is violated.
                vPeak = evalin('base', 'traj_peak_speed');
                vMax  = evalin('base', 'v_track_max');
                aPeak = evalin('base', 'traj_peak_accel');
                aMax  = evalin('base', 'a_track_max');
                needT = evalin('base', 'traj_min_transition');
                % BOTH limits, because acceleration scales as 1/T^2 where speed scales as
                % 1/T, so a plan can be comfortably inside the speed limit and still demand
                % more lateral acceleration than PositionController is allowed to command.
                % That gap is not hypothetical: 36 drones over Grid-Circle-Grid at 8 s
                % transitions asks 6.97 m/s of an 8.0 m/s limit -- silent on speed -- while
                % asking 2.98 m/s^2 of a 3.0 m/s^2 clamp, and one drone ends 127.66 m out.
                % Checking speed alone let that Generate report "Show ready".
                if vPeak > vMax || aPeak > aMax
                    % Fix it rather than reporting it. Every fleet size above roughly 17
                    % drones is too fast at the default 8 s transition, so at the sizes
                    % the spinner now reaches "set Transition to at least X and press
                    % Generate again" would be the normal outcome of pressing Generate.
                    % fitTransitionToPlan regenerates and reports what it changed; if it
                    % did, that regenerate has already written the status bar and updated
                    % the labels from the new plan, so this one must not overwrite them.
                    if app.fitTransitionToPlan(needT)
                        return;
                    end
                    % Name the constraint that actually binds. Telling an operator the
                    % speed is too high when the speed is legal sends them to the wrong
                    % control -- and the cure differs: spacing helps both, but only time
                    % helps acceleration at four times the leverage it has on speed.
                    if aPeak > aMax && vPeak <= vMax
                        app.updateStatus(sprintf( ...
                            ['UNFLYABLE PLAN: transitions demand %.1f m/s^2 of lateral ' ...
                             'acceleration but the fleet tracks about %.1f. Speed is ' ...
                             'fine (%.1f m/s) -- this is the turn-in, not the cruise. ' ...
                             'Drones will saturate and may not recover. Set Transition ' ...
                             '(s) to at least %.1f (now %.1f), or reduce spacing. ' ...
                             'Regenerate after changing it.'], ...
                            aPeak, aMax, vPeak, ceil(needT * 10) / 10, ...
                            app.TransitionField.Value));
                    else
                        app.updateStatus(sprintf( ...
                            ['UNFLYABLE PLAN: transitions demand %.1f m/s but the fleet ' ...
                             'tracks about %.0f m/s. Drones will leave formation and may ' ...
                             'not recover. Set Transition (s) to at least %.1f (now %.1f), ' ...
                             'or reduce spacing. Regenerate after changing it.'], ...
                            vPeak, vMax, ceil(needT * 10) / 10, app.TransitionField.Value));
                    end
                elseif minSep < evalin('base', 'd_min')
                    app.updateStatus(sprintf( ...
                        ['SEPARATION VIOLATED: min sep %.2f m against a %.2f m ' ...
                         'requirement. Increase Spacing (m) and regenerate.'], ...
                        minSep, evalin('base', 'd_min')));
                else
                    app.updateStatus(sprintf( ...
                        ['Show ready: %d UAVs, %d formations, %.0fs. Min sep: %.2fm, ' ...
                         'peak %.1f m/s, %.1f m/s². Click Play to preview, or %s.'], ...
                        N, numel(formSeq), showDur, minSep, vPeak, aPeak, nextStep));
                end
                app.SpeedLabel.Text = sprintf('Fleet Size: %d UAVs', N);
                app.TimeLabel.Text = sprintf('Time: 0.0 / %.1f s', showDur);
            catch e
                app.updateStatus(['ERROR: ' e.message]);
            end
        end

        %% ---- Build 3D Scene ----
        function buildScenario(app)
            initPos = evalin('base', 'init_positions');

            ax = app.ScenarioAxes;
            cla(ax, 'reset');
            ax.Color = [0.03 0.03 0.10];
            ax.GridColor = [0.25 0.25 0.35];
            ax.XColor = [0.7 0.7 0.7];
            ax.YColor = [0.7 0.7 0.7];
            ax.ZColor = [0.7 0.7 0.7];
            hold(ax, 'on');
            grid(ax, 'on');
            xlabel(ax, 'North (m)');
            ylabel(ax, 'East (m)');
            zlabel(ax, 'Up (m)');

            % Determine extent
            if app.UseSimData && ~isempty(app.SimTrajectoryData)
                allXY = app.SimTrajectoryData(:, 1:2, :);
                allZ_NED = app.SimTrajectoryData(:, 3, :);
            else
                allXY = app.TrajectoryData(:, 1:2, :);
                allZ_NED = app.TrajectoryData(:, 3, :);
            end
            xVals = allXY(:,1,:); yVals = allXY(:,2,:);
            margin = 8;
            xCenter = (max(xVals(:)) + min(xVals(:))) / 2;
            yCenter = (max(yVals(:)) + min(yVals(:))) / 2;
            maxAltitude = max(-allZ_NED(:));
            halfSpan = max([max(xVals(:))-min(xVals(:)), ...
                           max(yVals(:))-min(yVals(:)), ...
                           maxAltitude]) / 2 + margin;

            % Ground plane
            gx = [xCenter-halfSpan, xCenter+halfSpan, xCenter+halfSpan, xCenter-halfSpan];
            gy = [yCenter-halfSpan, yCenter-halfSpan, yCenter+halfSpan, yCenter+halfSpan];
            gz = [0 0 0 0];
            app.GroundPatch = patch(ax, gx, gy, gz, [0.08 0.10 0.14], ...
                'EdgeColor', 'none', 'FaceAlpha', 0.9);

            % Initial drone scatter (positions in NED → convert Z to up for plot).
            % getUAVColors keys off the formation *type*, so the fleet starts in
            % the colour of the formation it is climbing to.
            if isempty(app.FormSeq)
                initType = 1;
            else
                initType = app.FormSeq(1);
            end
            initColors = app.getUAVColors(initType);
            app.LastFormType = [];   % cla reset the axes; force a recolour

            % Only the active rendering gets handles; the other is left empty,
            % which is how drawFrame decides what to update.
            useMesh = strcmp(app.DroneRender, 'Quadrotor meshes');
            if useMesh
                app.loadQuadMesh();
                useMesh = ~isempty(app.MeshV0);
            end
            if useMesh
                app.DronesPlot = gobjects(0);
                app.MeshPatches = gobjects(app.NumUAVs, 1);
                for k = 1:app.NumUAVs
                    app.MeshPatches(k) = patch(ax, ...
                        'Faces', app.MeshF, ...
                        'Vertices', app.MeshV0 + ...
                            [initPos(k,1), initPos(k,2), -initPos(k,3)], ...
                        'FaceColor', initColors(k,:), 'EdgeColor', 'none', ...
                        'FaceLighting', 'none', 'SpecularStrength', 0);
                end
            elseif strcmp(app.DroneRender, 'Markers only')
                % No white edge: at this size the edge is a third of the marker, so
                % 200 drones read as a field of white dots rather than coloured ones.
                app.MeshPatches = gobjects(0);
                app.DronesPlot = scatter3(ax, initPos(:,1), initPos(:,2), -initPos(:,3), ...
                    18, initColors, 'filled');
            else
                % 60, not 180. At 180 a drone was ~4 m across on screen against 5 m
                % spacing, so a Grid rendered as one continuous slab and the whole
                % point of a formation was invisible.
                app.MeshPatches = gobjects(0);
                app.DronesPlot = scatter3(ax, initPos(:,1), initPos(:,2), -initPos(:,3), ...
                    60, initColors, 'filled', ...
                    'MarkerEdgeColor', 'w', 'LineWidth', 0.5);
            end

            % Hollow rings around the degraded drones. Created empty and moved by
            % drawFrame; one object for the whole fleet, so the per-frame cost is a single
            % scatter update no matter how many drones are denied.
            %
            % This exists because COLOUR ALONE CANNOT ANSWER "which drone is which".
            % Denied drones are also drawn red, and that read fine right up until the
            % default palette was checked against it: formation 1 is pure red
            % [1 0 0] and the marker is [0.95 0.15 0.10], so degrading a drone in the
            % opening formation marked it invisibly. Orange (formation 7) and pink
            % (formation 10) are nearly as bad. Picking a different marker colour does not
            % fix it either -- the operator can set any formation to any colour from the
            % Colour picker, so there is no colour that is guaranteed to contrast.
            %
            % A ring is palette-independent: no face, white edge, and large enough to sit
            % outside the drone itself, so it reads against a red drone on a dark
            % background as well as against any other. It also works unchanged in all
            % three rendering modes, which per-drone colour does not -- meshes, spheres
            % and markers each assign colour by a different route.
            app.DenyRings = scatter3(ax, NaN, NaN, NaN, 220, 'o', ...
                'MarkerEdgeColor', [1 1 1], 'MarkerFaceColor', 'none', ...
                'LineWidth', 1.5);

            % Rolling trails (short light streaks), if they are wanted at all.
            % createTrails reads LastFormType, which was just cleared, so it picks the
            % same initial colours the drones got.
            app.TrailPlots = gobjects(0);
            app.createTrails();

            % The planned-path overlay, likewise. The cla above destroyed the line AND the
            % axes limits it was drawn under, and dropping the phase cache with it is the
            % part that matters: a re-Generate can produce a different plan whose first
            % phase spans the same seconds, so a surviving cache would leave the OLD paths
            % on screen and never notice. The next drawFrame rebuilds it.
            app.clearTrajOverlay();

            xlim(ax, [xCenter - halfSpan, xCenter + halfSpan]);
            ylim(ax, [yCenter - halfSpan, yCenter + halfSpan]);
            zlim(ax, [-1, maxAltitude + margin]);
            daspect(ax, [1 1 1]);
            view(ax, 35, 25);   % fixed; drawFrame must not reset it per frame
            title(ax, 'Multi-UAV Drone Light Show', 'Color', 'w');
            % cla(ax,'reset') above destroyed the previous Title object, so cache
            % the new one for drawFrame.
            app.TitleText = get(ax, 'Title');
        end

        %% ---- Playback Controls ----
        function togglePlay(app)
            if app.Playing
                app.pausePlayback();
            else
                app.startPlayback();
            end
        end

        function txt = playIdleText(app)
            % What the Play button says when it is NOT playing. In one place because it is
            % written from three (pause, stop, and the end of a scrub that does not resume)
            % and they disagreed: all three wrote a bare 'Play', so the '[Sim]' marker that
            % says "this is the logged run, not the plan" survived until the first pause and
            % then vanished, leaving no way to tell which dataset was on screen.
            if app.UseSimData
                txt = 'Play [Sim]';
            else
                txt = 'Play';
            end
        end

        function clearTrails(app)
            % Wipe the streaks. Any jump in playback position has to do this: a trail is a
            % record of where a drone came FROM, and after a seek it did not come from
            % there. Left alone it draws a straight line across the sky between the frame
            % you left and the frame you landed on.
            if isempty(app.TrailPlots)
                return;
            end
            for k = 1:numel(app.TrailPlots)
                if isvalid(app.TrailPlots(k))
                    clearpoints(app.TrailPlots(k));
                end
            end
        end

        function seekFrame(app, val, dragging)
            % Move playback to a frame. Both scrub-bar callbacks land here: `dragging` is
            % true for ValueChanging (the thumb is moving under the pointer, many calls a
            % second) and false for ValueChanged (released).
            %
            % A DRAG SUSPENDS PLAYBACK and the release resumes it if it was running. Not
            % just politeness: playLoop is a while loop that yields inside pause(), so these
            % callbacks fire from INSIDE it, and if the loop kept advancing frames while the
            % drag drew its own, the two would fight over the axes and the fleet would
            % flicker between two positions. Clearing Playing drops the loop out on its next
            % test; setting it again at the release is picked up by the same loop if it is
            % still on the stack, which is exactly the case playLoop's PlayLoopActive guard
            % exists for.
            if isempty(app.NumSamples) || app.NumSamples < 1
                return;
            end
            if app.SimRunning
                % Seeking a live run is meaningless -- the frames past "now" have not been
                % computed. Put the thumb back rather than leaving it somewhere it is not.
                app.refreshScrub();
                app.updateStatus(['The scrub bar works on a finished run. A live one is ' ...
                    'still being flown, so there is nothing ahead of it to seek to.']);
                return;
            end

            newDrag = dragging && ~app.Scrubbing;
            if newDrag
                app.Scrubbing = true;
                app.ScrubWasPlaying = app.Playing;
                app.Playing = false;
            end

            idx = min(max(round(val), 1), app.NumSamples);
            app.PlayIdx = idx;
            % ONCE per drag, not once per drag event. clearTrails is a clearpoints per
            % drone and drawFrame's trail loop is an addpoints per drone, and on a uifigure
            % each of those is its own round trip to the view: measured at 10 drones, a
            % scrub frame costs 79.9 ms with the trails in play against 6.9 ms without, so
            % a drag was redrawing at ~13 fps and felt like dragging through treacle. What
            % it bought was nothing at all -- the old code wiped the streaks on every event,
            % so what the operator saw mid-drag was a one-point trail, i.e. no trail. Now
            % the wipe happens at the start of the drag and again at the release (see
            % drawFrame, which skips the addpoints while Scrubbing), which is the same
            % picture at a sixth of the cost.
            if newDrag || ~dragging
                app.clearTrails();
            end
            app.drawSeekFrame(idx);

            if ~dragging
                % Released. Re-anchor the pacing clock at the frame we landed on, or the
                % deadline would be computed from the show time we left -- the same trap
                % setSpeed documents, and here it would make the resume sprint to catch up.
                app.Scrubbing = false;
                resume = app.ScrubWasPlaying;
                app.ScrubWasPlaying = false;
                if resume && idx < app.NumSamples
                    app.startPlayback();
                else
                    app.PaceClock = [];
                    app.FrameWait = 0;
                    % The drag cleared Playing without touching the button, on the
                    % assumption that the release would put it back. This is the branch
                    % where it does not: dragged to the very end, there is nothing left to
                    % resume, and without this the button sits reading "Pause" over a show
                    % that is not playing.
                    app.PlayBtn.Text = app.playIdleText();
                end
            end
        end

        function drawSeekFrame(app, idx)
            % Paint one frame and the telemetry that goes with it, without touching any
            % playback state. This is what a scrub shows you.
            if app.UseSimData
                if isempty(app.SimTrajectoryData), return; end
                idx = min(idx, size(app.SimTrajectoryData, 3));
                t = app.SimTimeVector(idx);
                pos = squeeze(app.SimTrajectoryData(:, 1:3, idx));
                srcLabel = ' [Simulated]';
            else
                if isempty(app.TrajectoryData), return; end
                idx = min(idx, size(app.TrajectoryData, 3));
                t = app.TimeVector(idx);
                pos = squeeze(app.TrajectoryData(:, 1:3, idx));
                srcLabel = '';
            end
            app.drawFrame(pos, t, srcLabel);
            app.updateTelemetry(idx, pos, t);
            app.refreshScrub();
        end

        function refreshScrub(app)
            % The ONE place the scrub bar is synced to playback state, called both when the
            % dataset changes (new length) and on every frame (new position). Kept in one
            % function because two of them would disagree the first time a code path only
            % updated one.
            %
            % Limits(1) is always 1 so that Value = 1 is legal at every length: setting
            % Limits while Value sits outside the new range is an error, so the sequence has
            % to be park-at-1, re-limit, then move.
            if isempty(app.ScrubSlider) || ~isvalid(app.ScrubSlider)
                return;
            end
            % Gated on the PLAY BUTTON rather than on the data, deliberately. "Is there
            % something to scrub" and "is there something to play" are the same question,
            % and asking it twice is how the two controls end up disagreeing -- Reset, for
            % one, disables Play and clears the scene while leaving TrajectoryData in place,
            % so a data test would leave a live bar pointing at a show that is no longer
            % drawn.
            %
            % A live run is excluded for a different reason: the frames ahead of "now" do
            % not exist yet, so there is nowhere to seek to. seekFrame refuses anyway, but a
            % control that refuses is worse than one that is visibly unavailable.
            n = app.NumSamples;
            haveData = strcmp(app.PlayBtn.Enable, 'on') && ~app.SimRunning && ...
                ~isempty(n) && n > 1;
            if ~haveData
                app.ScrubSlider.Value = 1;
                app.ScrubSlider.Limits = [1 2];
                app.ScrubSlider.Enable = 'off';
                app.ScrubTimeLabel.Text = '—';
                return;
            end
            idx = min(max(app.PlayIdx, 1), n);
            % Not while the thumb is being dragged: the pointer owns the position then, and
            % writing Value under it makes the thumb jump back to where the last redraw
            % thought it was. The readout below still updates, because that is the thing a
            % drag most needs. The length cannot change mid-drag either, so skipping the
            % re-limit costs nothing. (Writing Value does not re-enter this function --
            % programmatic sets do not fire ValueChangedFcn -- so the guard is only about
            % the pointer, not about recursion.)
            if ~app.Scrubbing
                app.ScrubSlider.Value = 1;
                app.ScrubSlider.Limits = [1 n];
                app.ScrubSlider.Value = idx;
            end
            app.ScrubSlider.Enable = 'on';

            if app.UseSimData && ~isempty(app.SimTimeVector)
                tv = app.SimTimeVector;
            else
                tv = app.TimeVector;
            end
            if isempty(tv)
                app.ScrubTimeLabel.Text = sprintf('frame %d / %d', idx, n);
            else
                app.ScrubTimeLabel.Text = sprintf('%.1f / %.1f s', ...
                    tv(min(idx, numel(tv))), tv(end));
                app.ScrubSlider.Tooltip = sprintf(['Drag to any moment in the show -- ' ...
                    '%d cached frames over %.1f s. It moves the fleet as you drag, and ' ...
                    'redraws only: nothing is re-planned, re-uploaded or re-flown. ' ...
                    'Dragging while it plays pauses it for the drag and carries on where ' ...
                    'you let go.'], n, tv(end));
            end
        end

        function startPlayback(app)
            app.beginPlayback();
            app.playLoop();
        end

        function beginPlayback(app)
            % Arm playback without animating: sets the state advanceFrame needs.
            if app.PlayIdx >= app.NumSamples
                % Play from the end means play again from the top -- which is the whole of
                % what the Restart button used to do, so it has to do the rest of what
                % Restart did too. Wiping the trails is that rest: they are a record of the
                % run that just finished, and left up they would be redrawn over from frame
                % 1 with the old streaks still hanging in the air.
                app.PlayIdx = 1;
                app.CameraAzOffset = 0;
                app.clearTrails();
            end
            app.Playing = true;
            % Arm the pacing clock HERE rather than in playLoop, so a caller driving
            % playback through the armPlayback/stepPlayback seam is paced by the same
            % rule the Play button is. Resuming from a pause re-arms it against the
            % current sample, which is what makes Pause cost no show time.
            app.PaceClock = tic;
            if app.UseSimData && ~isempty(app.SimTimeVector)
                app.PaceRefTime = app.SimTimeVector(min(app.PlayIdx, end));
            elseif ~isempty(app.TimeVector)
                app.PaceRefTime = app.TimeVector(min(app.PlayIdx, end));
            else
                app.PaceRefTime = 0;
            end
            app.PlayBtn.Text = 'Pause';
            app.StopBtn.Enable = 'on';
            app.GenerateBtn.Enable = 'off';
            if app.UseSimData
                app.updateStatus('Playing simulated data...');
            else
                app.updateStatus('Playing planned trajectory...');
            end
        end

        function playLoop(app)
            % Animate at ~20 fps until the show ends or Playing is cleared.
            %
            % This used to be a timer('ExecutionMode','fixedRate','Period',0.05).
            % Timer callbacks do not fire in this MATLAB session at all --
            % TasksExecuted stays 0 even after 45 s of pause(0.05), through both
            % MCP entry points and with a uifigure open.
            % So Play armed the animation and then nothing happened: the fleet
            % never moved and the telemetry panel never updated, because both are
            % painted by advanceFrame and advanceFrame was only ever called by the
            % timer. Loop here instead -- the same mechanism runShowPolled already
            % uses to stream a live run, which does work.
            %
            % drawFrame ends with `drawnow limitrate` and pause() yields, so
            % Pause/Stop/Replay and the dropdowns are still serviced mid-loop.
            % They act by clearing Playing, which is what drops us out.
            if app.PlayLoopActive
                % Reached from a callback that fired inside our own pause(), e.g.
                % Replay or a rendering change. The loop below is still on the
                % stack and picks up the state that callback just set.
                return;
            end
            app.PlayLoopActive = true;
            try
                while app.Playing && ~isempty(app.Fig) && isvalid(app.Fig)
                    % No frame-cost bookkeeping is needed here: FrameWait is the wall
                    % clock still owed measured against PaceClock, so whatever the draw
                    % just cost is already inside the elapsed time advanceFrame read.
                    % A frame that overran its budget simply asks for no wait at all.
                    app.advanceFrame();
                    if ~app.Playing
                        break;   % advanceFrame reached the end and stopped us
                    end
                    % A FULL flush, not the `drawnow limitrate` drawFrame ends with. Both
                    % paint, but they do not drain the event queue alike: limitrate caps
                    % updates at 20 fps and discards the rest, and playback runs right at
                    % that cap (measured 20-29 fps at 10 drones, 12-16 at 60), so most
                    % calls fall on the discarding side of it. Measured in isolation on a
                    % loop doing drawFrame's work: a queued callback waited 0.09 s behind
                    % limitrate against 0.01 s behind a full drawnow. It costs 0.6 ms.
                    drawnow;
                    % Never pause(0): Pause, Stop, Replay and the dropdowns are serviced
                    % inside this yield, so a frame that owes nothing still has to give
                    % the callbacks a turn -- and it has to be a REAL turn, which is what
                    % YieldFloor is for. See the property.
                    pause(max(app.FrameWait, app.YieldFloor));
                end
            catch err
                app.PlayLoopActive = false;
                app.Playing = false;
                rethrow(err);
            end
            app.PlayLoopActive = false;
        end

        function pausePlayback(app)
            app.Playing = false;
            app.PlayBtn.Text = app.playIdleText();
            app.updateStatus('Paused.');
        end

        function stopPlayback(app)
            % Announce the stop only if there was something to stop. This is also
            % called as housekeeping by the two run paths before they start, and
            % reporting "Stopped." to an operator who has just pressed Simulate
            % reads as a refusal — the status bar then sat on it for the whole of
            % prepareModel, which is seconds and up to ~26 s when the model has to
            % be reloaded and the MAVLink masks re-parse.
            wasPlaying = app.Playing;
            app.Playing = false;
            app.PlayIdx = 1;
            % Disarm the pacing clock, so the next Play anchors at the sample it
            % actually starts from rather than inheriting a deadline from the last run.
            app.PaceClock = [];
            app.FrameWait = 0;
            if ~isempty(app.Fig) && isvalid(app.Fig)
                app.PlayBtn.Text = app.playIdleText();
                app.StopBtn.Enable = 'off';
                app.GenerateBtn.Enable = 'on';
                % PlayIdx went back to 1 above, so the thumb has to follow it. This is
                % also the call that brings the bar back to life after a live run, which
                % is why it is not inside the wasPlaying test below: a run the operator
                % never played is exactly the case that needs re-enabling.
                app.refreshScrub();
                if wasPlaying
                    app.updateStatus('Stopped.');
                end
            end
        end

        function abortAndLand(app)
            % Abandon the show and bring the fleet down. This is a MISSION command:
            % the drones leave the plan, descend from wherever they are at land_speed
            % and park, and the simulation carries on until they are down. Stop is the
            % other thing entirely — it kills the run on the spot and leaves the fleet
            % hanging in mid-air at whatever the last frame showed.
            %
            % Delivered by retuning AbortReqConst while the model runs, which is what
            % ShowSupervisor reads as AbortRequest: UPLOADING, ARMED and SHOW all
            % transition to ABORT_LANDING on it, and from there DroneFleet's landing
            % network takes the position command off the plan. There is no need for the fleet
            % to have any uploaded waypoints, which is the point — a planned landing
            % cannot help a show that is being abandoned mid-flight.
            modelName = 'MultiUAV_DroneShow';
            if ~app.SimRunning
                app.updateStatus(['Abort & Land needs a live run — nothing is ' ...
                    'flying. It commands the fleet down; there is no fleet to command.']);
                return;
            end
            try
                set_param([modelName '/AbortReqConst'], 'Value', 'true');
                tNow = get_param(modelName, 'SimulationTime');
            catch e
                app.updateStatus(['Abort & Land could not reach the model: ' e.message]);
                return;
            end
            app.AbortRequested = true;
            % Latched in the chart, so pressing it again means nothing.
            app.AbortBtn.Enable = 'off';
            app.updateStatus(sprintf(['ABORT & LAND at t = %.1f s — the fleet is ' ...
                'leaving the plan and descending to the ground. The run continues ' ...
                'until it is parked.'], tNow));
        end

        function clearAbort(~, modelName)
            % Untie the abort. Asserting it is a live retune of a Constant block, so
            % leaving it at 'true' would send the NEXT run diving out of ARMED into
            % ABORT_LANDING before it ever flew. Called when a run is prepared and
            % again when it finishes, so neither an abort nor a crash can arm it.
            if ~bdIsLoaded(modelName)
                return;
            end
            try
                set_param([modelName '/AbortReqConst'], 'Value', 'abort_request');
            catch
            end
        end

        function setRtkDeny(app, deny)
            % Deny or restore RTK corrections for any set of drones, or for the whole
            % fleet if the list is 0. This is the operator face of rtk_deny_mask.
            %
            % It works before a run and during one, and there is no "degrade at (s)"
            % to fill in, which is the point. rtk_deny_mask is a workspace variable
            % behind a Constant that feeds the per-drone correction-arrival AND, so
            % retuning it mid-run is the same mechanism Abort & Land uses: write the
            % variable, then SimulationCommand 'update' to make the block re-evaluate
            % its expression. Verified live -- a mask written at sim time 4.34 s first
            % shows up in the sigma at 5.50 s, which is the timeout plus one broadcast
            % interval, exactly as it should be.
            %
            % What this does NOT do is deny GNSS. The drone keeps its satellites and
            % falls back RTK Fix -> Float -> Standalone, stopping at 1.5 m. That is
            % the bounded failure, and it is the only one the model claims -- see
            % gnss_deny_mask in setupParams.m for why the unbounded one is not exposed.
            modelName = 'MultiUAV_DroneShow';
            N = app.denyMaskLength();

            % One status message, not two: the second updateStatus would overwrite the
            % first before the operator ever saw it, so every note is carried and
            % prefixed.
            [idx, note] = app.parseDroneList(app.DegradeUAVField.Value, N);
            if isempty(idx)
                app.updateStatus([note 'Nothing to do — name a drone, a list ' ...
                    'like 1 3 5, or 0 for the whole fleet.']);
                return;
            end

            m = app.currentDenyMask(N);
            if any(idx == 0)
                m(:) = deny;
                who = sprintf('the whole fleet (%d drones)', N);
            else
                m(idx) = deny;
                if isscalar(idx)
                    who = sprintf('UAV %d', idx);
                else
                    who = sprintf('UAVs %s', strjoin(string(idx), ', '));
                end
            end
            app.publishDenyMask(m);
            app.refreshDenyLabel(m);

            % Mark them in the 3-D view straight away. The red marking is the only
            % answer there is to "which drone is which": matchpairs re-runs the
            % assignment at every transition, so slot k is a different aircraft in
            % every formation and no stable index-to-position mapping exists to read
            % off the screen. drawFrame picks the mask up on its next frame, which
            % covers a degradation made mid-flight -- but degrading while stopped or
            % paused would change the panel text and nothing else, and an operator who
            % just asked which drone is 3 reads that as the marking not working.
            % Skipped while a run is streaming: the poll loop is already painting live
            % positions there, and this would paint planned ones over them.
            if ~app.SimRunning
                app.drawCurrentFrame();
            end

            if deny
                verb = 'lost its RTK corrections';
                tail = ['It will drift out to the Standalone tier (1.5 m) over the ' ...
                        'next few seconds and stay there.'];
            else
                verb = 'has its RTK corrections back';
                tail = ['The error bleeds off over reconverge_time rather than ' ...
                        'snapping back — a real receiver takes seconds to re-fix.'];
            end
            % Plural for a list as well as for the whole fleet — "UAVs 1, 3, 5 has
            % lost its corrections" was the first version and read as a bug report.
            if any(idx == 0) || ~isscalar(idx)
                verb = strrep(verb, 'its', 'their');
                verb = strrep(verb, 'has', 'have');
                tail = strrep(tail, 'It will', 'They will');
            end

            if app.SimRunning
                try
                    set_param(modelName, 'SimulationCommand', 'update');
                    tNow = get_param(modelName, 'SimulationTime');
                    app.updateStatus(sprintf('%sAt t = %.1f s, %s %s. %s', ...
                        note, tNow, who, verb, tail));
                catch e
                    app.updateStatus([note 'The mask is set, but the running model ' ...
                        'could not be retuned: ' e.message]);
                end
            else
                app.updateStatus(sprintf('%s%s %s from the start of the next run. %s', ...
                    note, who, verb, tail));
            end
        end

        function N = denyMaskLength(app)
            % The mask has to be exactly N_uav long or the Constant driving the AND is
            % the wrong width. Which fleet size is authoritative depends on whether
            % anything is flying:
            %
            %   running  -> N_uav, the size the model actually compiled against. The
            %               spinner can be moved mid-run and must not be believed.
            %   stopped  -> the spinner, because the press applies to the NEXT run and
            %               that is the fleet it will have.
            %
            % Reading N_uav unconditionally was the first version and it was wrong in a
            % way that only a full session showed: N_uav survives in the base workspace
            % from whatever ran last, so pressing Degrade in a fresh app built a mask
            % sized for someone else's fleet.
            if app.SimRunning
                try
                    N = evalin('base', 'N_uav');
                    N = max(1, round(N));
                    return;
                catch
                end
            end
            N = max(1, round(app.NumUAVSpinner.Value));
        end

        function [idx, note] = parseDroneList(~, str, N)
            % Turn what the operator typed into drone indices, and say what was
            % thrown away rather than silently degrading a different set than asked
            % for. Accepts "3", "1 3 5", "1,3,5", "[1 3 5]", "(1 3 5)", "1:5", "1-5",
            % "1 to 5", "1 and 5" and "0".
            %
            % PARSED BY HAND, which it did not used to be. str2num was doing this in one
            % line, and str2num is eval -- so the operator's punctuation was arithmetic:
            %
            %   "2-1"   -> 1      degraded UAV 1 alone
            %   "1 & 2" -> true   degraded UAV 1 alone
            %   "1*2"   -> 2      degraded UAV 2
            %
            % None of those FAILED. Each one degraded a different set of drones than was
            % named and then reported success, which is the worst thing this field can do:
            % the operator watches the wrong aircraft fall back and concludes the RTK model
            % is broken. Meanwhile "(1,2)" was rejected outright even though "[1,2]" was
            % accepted and this field's own tooltip writes its examples in parentheses.
            %
            % So the grammar is now explicit and closed: a token is either a number or a
            % range, and anything else is reported instead of evaluated.
            idx = [];
            note = '';
            str = strtrim(char(string(str)));
            if isempty(str)
                note = 'The Degrade field is empty. ';
                return;
            end

            % Fold every separator someone might reach for into a space, and every way of
            % writing a span into a colon, so the token loop below only has to know two
            % shapes. Order matters: the bracket pass runs first so "[1-3]" reduces
            % cleanly, and ' and ' is matched with its spaces so it cannot eat a substring.
            s = lower(str);
            s = replace(s, {'[', ']', '(', ')', '{', '}'}, ' ');
            s = replace(s, {' and ', '&', ',', ';'}, ' ');
            s = replace(s, {'..', ' to ', '-'}, ':');
            toks = split(strtrim(regexprep(s, '\s+', ' ')), ' ');

            raw = [];
            badTok = {};
            for q = 1:numel(toks)
                t = toks{q};
                if isempty(t)
                    continue;
                end
                one  = regexp(t, '^\d+$', 'match', 'once');
                span = regexp(t, '^(\d+):(\d+)$', 'tokens', 'once');
                if ~isempty(one)
                    raw(end+1) = str2double(one);                        %#ok<AGROW>
                elseif ~isempty(span)
                    % A span typed backwards is still a span. "5:2" names four drones to
                    % anyone who types it, where a literal a:b would silently give none --
                    % and "silently none" is how this field got into trouble before.
                    a = str2double(span{1});
                    b = str2double(span{2});
                    raw = [raw, min(a, b):max(a, b)];                    %#ok<AGROW>
                else
                    badTok{end+1} = t;                                   %#ok<AGROW>
                end
            end

            if ~isempty(badTok)
                % The complaint is deliberately about SYNTAX, not about the drones: an
                % unparseable field is a typo, not a request the fleet cannot honour.
                note = sprintf(['"%s" is not a drone list. Use a number, a list ' ...
                    'like 1 3 5, a range like 1:5, or 0 for the whole fleet. '], ...
                    strjoin(badTok, '", "'));
            end
            if isempty(raw)
                return;
            end
            raw = raw(:)';
            % Every note from here on is APPENDED, never assigned. setRtkDeny shows one
            % status message, so a second assignment would drop the first complaint --
            % and "1, foo, 150" has two things wrong with it that the operator needs to
            % hear together.
            if any(raw == 0)
                idx = 0;          % the whole fleet; anything else in the list is moot
                if numel(raw) > 1
                    note = [note '0 means the whole fleet, so the other numbers in ' ...
                        'the list make no difference. '];
                end
                return;
            end

            % Past-the-fleet numbers are DROPPED, not clamped. The old numeric field
            % clamped 150 to the last drone, which quietly degraded a drone nobody asked
            % about; with a list there is no defensible single drone to clamp onto, and
            % dropping is the only honest answer. Negatives can no longer arrive here at
            % all -- the token grammar has no sign -- so "-5" is refused as syntax rather
            % than reported as a drone the fleet does not have.
            bad = raw(raw < 1 | raw > N);
            idx = unique(raw(raw >= 1 & raw <= N));
            if isscalar(bad)
                note = [note sprintf(['There is no UAV %d — the fleet is %d drones, ' ...
                    'so it was ignored. '], bad, N)];
            elseif ~isempty(bad)
                note = [note sprintf(['The fleet is %d drones, so %s do not exist ' ...
                    'and were ignored. '], N, mat2str(bad))];
            end
        end

        function m = currentDenyMask(~, N)
            % Read the mask back, and resize rather than discard: an operator who
            % degraded UAV 3, changed the fleet size and pressed Degrade again should
            % not silently lose UAV 3. Entries past the new end are dropped, which is
            % the only thing that can be done with them.
            m = false(N, 1);
            try
                old = evalin('base', 'rtk_deny_mask');
                if islogical(old) || isnumeric(old)
                    old = logical(old(:));
                    k = min(numel(old), N);
                    m(1:k) = old(1:k);
                end
            catch
            end
        end

        function publishDenyMask(app, m) %#ok<INUSD>
            % Publish the mask AND the gate that is derived from it. Never assign
            % rtk_deny_mask on its own -- these two have to move together.
            %
            % rtk_inject_enable is what CorrectionAgeMonitor/InjectEnable actually reads,
            % and setupParams builds it as ~skip_preflight | mask so that a degraded drone
            % keeps its degradation in fast mode while the rest of the fleet stays on the
            % 2 cm INS floor. The catch is that setupParams only runs BETWEEN runs, so a
            % mask retuned mid-flight has to carry the derived gate with it: otherwise the
            % gate stays frozen at whatever the mask was when the model compiled, and in
            % fast mode that is all zeros -- the Degrade button would set the mask, mark
            % the drone red, print a status line, and change nothing about the flight.
            % That is exactly the bug this pairing exists to prevent.
            m = logical(m(:));
            assignin('base', 'rtk_deny_mask', m);
            % Default to false rather than erroring: an unset skip_preflight means full
            % fidelity, and full fidelity is the all-ones gate, which is also the safe
            % direction to guess -- it injects too much rather than too little. Any
            % pre-run call is overwritten by setupParams before the model compiles anyway.
            try
                skip = logical(evalin('base', 'skip_preflight'));
            catch
                skip = false;
            end
            assignin('base', 'rtk_inject_enable', double(~skip | m'));
        end

        function armPollRate(app)
            % Fresh controller state per run. Not carried across runs: u depends on the
            % render mode and the trails, both of which can have changed since the last
            % one, and starting from a stale estimate makes the first seconds of the new
            % run poll at the old show's cost.
            app.UICostEMA = [];
            app.FrameCostEMA = [];
            app.SolverRateEMA = [];
            app.CycleEMA = [];
            app.PendWall = 0;
            app.PendGrant = 0;
            app.BehindWall = 0;
            app.PollInterval = 0.1;
            app.HoldInterval = 0;
            app.PacingRate = 2;
            app.PacingWritten = NaN;
            app.LowRateWall = 0;
            app.HighRateWall = 0;
            app.AchievedRate = NaN;
            app.PollStats = struct('n', 0, 'nMoved', 0, 'sumU', 0, 'sumP', 0, ...
                'sumH', 0, 'sumSim', 0);
            app.ShowRate = NaN;
        end

        function [p, w, pace] = nextPollInterval(app, uiCost, dSim, dPause, dHold)
            % Solve for the poll interval that HITS the chosen real-time factor, and for
            % the throttle that catches what the poll interval alone cannot reach: the
            % pacing rate to ask Simulink for where the model accepts pacing, and the spin
            % hold where it does not. `pace` is which of the two the caller should have
            % engaged on the next cycle -- returned rather than acted on here because this
            % method does not know the model name.
            %
            % Each cycle is (uiCost) + (pause p) + (hold w) of wall clock. The simulation
            % advances only while we are inside the pause -- the run is asynchronous but
            % single-threaded with the UI -- so it advances by about S*p sim-seconds,
            % where S is the solver's own rate in sim-seconds per second of
            % uninterrupted compute. With no hold the achieved factor is
            %
            %     R = S*p / (uiCost + p)   =>   p = R*uiCost / (S - R)
            %
            % Both S and uiCost are MEASURED and smoothed rather than assumed, which is
            % the whole reason this is not a threshold on the fleet size. uiCost is
            % dominated by the render mode and the trails: meshes at 200 drones cost
            % more per frame than markers at 200 drones by a large factor, and no rule
            % of the form "if N > 50, poll at 0.35" can know which one is on. S is
            % measured because it is not a constant either -- 3.2 sim-s/s at 8 drones
            % against 0.64 at 50 with spheres and trails on, and it drops in the phases
            % where the MAVLink chains are executing.
            %
            % WHY A HOLD IS NEEDED AT ALL. Shortening the pause is how the law slows the
            % show down, and the pause has a floor: below ~20 ms the solver is being handed
            % slices too short to be worth the per-yield overhead, and the viewer is
            % repainting at a rate nobody can see. So the slowest the pause alone can go is
            %
            %     R_min = S*PollMin / (u + PollMin)
            %
            % which at S = 3.2 and u = 5 ms is 2.6x -- and that is measured, not
            % hypothetical: with the pause as the only lever, a 1x target streamed at 2.72x
            % and the readout said so ("2.72x of 1x"). Real time was unreachable from
            % ABOVE, which is the opposite of the case the law was written for. The fix is
            % not a smaller floor -- that spends per-yield overhead to buy frames nobody
            % asked for -- it is to stop granting the solver time at all for the rest of the
            % cycle: hold the thread, blocking, so the async run cannot advance. Hence
            %
            %     cycle needed for R = S*p/R   =>   w = S*p/R - u - p
            %
            % which is exactly zero whenever the pause could hit the target on its own, so
            % the two levers never fight: w > 0 means precisely "the floor is in the way".
            %
            % S > 1 is still what makes real time reachable from BELOW. When S <= R the
            % target is not reachable at any pause length, the interval goes to its ceiling,
            % there is nothing to hold back, and the readout reports the factor actually
            % achieved instead of pretending.
            if nargin < 5, dHold = 0; end
            % SMOOTHED BY WALL CLOCK, not per cycle, because the cycles are wildly uneven
            % and the long ones are the ones that decide the run. Simulink sometimes does
            % not yield for several hundred ms whatever pause was asked for, and across a
            % chunk like that the drawing cost is amortised away, so the delivered factor
            % approaches r instead of r*slice/(u+slice). Weighting every cycle equally, the
            % law tracked the median cycle (1.07x, correct) while the run averaged 1.42x --
            % the fat cycles hold most of the wall clock and got a sample's worth of say.
            % One tick of the filter per second of wall clock instead, so the fixed point of
            % the loop below is the run average, which is the thing being controlled.
            TAU = 1;
            dFrame = max(dPause, 0) + max(dHold, 0);
            if isfinite(uiCost)
                dFrame = dFrame + max(uiCost, 0);
            end
            % NOT EVERY POLL SEES THE CLOCK MOVE, and the ones that do not still cost wall
            % clock. Simulink updates SimulationTime a chunk at a time, so a poll can come
            % and go inside one: measured at 38% of polls once pacing had the slice down to
            % 60 ms (against 1% at the fallback's coarse poll, which is why this only shows
            % up here). Charging an advance to the single cycle it happened to land in then
            % overstates the rate by exactly the ratio of all polls to moving ones -- 1.42x
            % on that run, the law reading 1.08x while the show ran at 0.76x, which is the
            % whole of the gap and in the one direction that matters. So the wall clock and
            % the granted time of a stalled poll are carried forward and spent on the
            % advance they actually bought.
            app.PendWall = app.PendWall + dFrame;
            app.PendGrant = app.PendGrant + max(dPause, 0);
            dWall = app.PendWall;
            dGrant = app.PendGrant;
            if dSim > 0
                app.PendWall = 0;
                app.PendGrant = 0;
            end
            % aF for what a FRAME costs, aW for what an ADVANCE cost: the two are the same
            % except across a stalled poll, where several frames paid for one advance.
            aF = min(dFrame / TAU, 1);
            aW = min(dWall / TAU, 1);
            if isfinite(uiCost) && uiCost > 0
                if isempty(app.UICostEMA)
                    app.UICostEMA = uiCost;
                else
                    app.UICostEMA = (1 - aF) * app.UICostEMA + aF * uiCost;
                end
            end
            if dGrant > 0 && dSim > 0
                sNow = dSim / dGrant;
                if isempty(app.SolverRateEMA)
                    app.SolverRateEMA = sNow;
                else
                    app.SolverRateEMA = (1 - aW) * app.SolverRateEMA + aW * sNow;
                end
            end
            % What one sim advance cost in wall clock the solver did NOT get: drawing, plus
            % any hold. Per ADVANCE, not per frame, and that is the whole point -- ask for
            % a slice shorter than the chunk the solver hands back regardless and the extra
            % polls each draw a frame and buy no sim time at all, so the true price of an
            % advance is k frames, not one. Measured at a 20 ms slice: 19% of polls stalled
            % and the heavy phase ran at 0.77x while the coarse-polling fallback held 1.03x.
            % Sizing the slice from this instead of from u makes the solve find its own
            % floor -- it returns roughly the chunk length, because that is where k = 1.
            if dSim > 0 && dWall > dGrant
                cNow = dWall - dGrant;
                if isempty(app.FrameCostEMA)
                    app.FrameCostEMA = cNow;
                else
                    app.FrameCostEMA = (1 - aW) * app.FrameCostEMA + aW * cNow;
                end
            end
            % dSim > 0 as well, and not for symmetry with the branch above. A poll that
            % catches the run between output writes -- the last one always does, since the
            % loop exits on 'stopped' and the sim time has stopped moving by then -- sees
            % dSim = 0 through a perfectly healthy cycle. Updating on it wrote 0.00x over
            % the real number and left it on screen, so a run ended reading
            % 'Live Rate: 0.00x of 1x' no matter how well it had streamed.
            % The hold belongs in here: it is wall clock the show took and sim time it did
            % not get, so leaving it out would report the factor the show would have run at
            % if it had not been throttled -- which is the one number nobody needs.
            % Smoothed, on the same time constant as the rest, because two things read it
            % and both want the trend rather than the last cycle. gradeLiveRate decides
            % which throttle to use from it, and a single lumpy chunk -- the solver hands
            % back whatever it happened to finish -- reads as 2x for one cycle; unsmoothed
            % that was enough to flip the throttle back on three cycles after it had
            % correctly let go, and the run then spent itself toggling (measured at 300 ms
            % frames: 40% of cycles paced, 53%, 20%, and 0.81x for the trouble). The label
            % is the other reader, and an instantaneous factor there just flickers.
            if dSim > 0 && dWall > 0
                rNow = dSim / dWall;
                if isnan(app.AchievedRate)
                    app.AchievedRate = rNow;
                else
                    app.AchievedRate = (1 - aW) * app.AchievedRate + aW * rNow;
                end
            end
            % The MEASURED cycle, which is the only honest source for a frame rate once
            % pacing is doing the throttling: the loop asks for a 20 ms pause and gets
            % ~60 ms back, because the solver runs a whole chunk before returning control.
            % 1/(p + w + u) would then advertise 40 fps on a stream that is visibly running
            % at 15. Every poll, including the stalled ones, because a poll is a FRAME
            % whether or not the clock moved -- unlike the rate above, which is about sim
            % time and must not count a frame the show did not pay for.
            if dFrame > 0
                if isempty(app.CycleEMA)
                    app.CycleEMA = dFrame;
                else
                    app.CycleEMA = (1 - aF) * app.CycleEMA + aF * dFrame;
                end
            end

            % Run totals. Kept separately from the EMAs because they answer a different
            % question: these are what the show DID, the EMAs are what to do next.
            st = app.PollStats;
            st.n = st.n + 1;
            if isfinite(uiCost) && uiCost > 0, st.sumU = st.sumU + uiCost; end
            if dPause > 0, st.sumP = st.sumP + dPause; end
            if dHold > 0, st.sumH = st.sumH + dHold; end
            if dSim > 0
                st.nMoved = st.nMoved + 1;
                st.sumSim = st.sumSim + dSim;
            end
            app.PollStats = st;

            R = app.RateTarget;
            u = app.UICostEMA;
            S = app.SolverRateEMA;

            % ---- Pacing: ask Simulink to do the waiting ----
            % WHAT TO ASK FOR. Pacing makes the solver wait between steps, but it catches
            % up freely inside any slice this loop grants it, and the slice (dPause) is
            % only part of a cycle that also spends uiCost drawing. So asking for exactly
            % 1x delivers slice/(uiCost + slice) and no more -- 0.71x at 40 ms of drawing,
            % 0.46x at 100 ms, both measured. The solver's EXCESS speed is what pays for
            % the viewer, so pacing has to be asked for more than the target:
            %
            %     cycle = uiCost + slice,  want dSim = R * cycle
            %     =>  r <- r * R * cycle / dSim
            %
            % Written as a correction to the DELIVERED factor rather than as the geometric
            % solve r = R*cycle/slice, and the difference is measurable. The geometric form
            % assumes dSim comes out at exactly r*slice; it does not, because pacing also
            % catches up a little inside the slice, so the delivered factor runs about 10%
            % over -- the solve settled at r = 1.67 and streamed 1.09x, on the machine and
            % in the seam alike. The ratio form divides that constant out whatever it is:
            % its fixed point is dSim = R*cycle by construction, i.e. exactly the target.
            %
            % What makes either form work is that the delivered factor is very nearly
            % linear in r -- 0.71x, 1.29x, 1.84x at r = 1, 2, 3 on the identical show. That
            % is the difference from the hold, where S was not independent of the answer at
            % all (polling less often measurably speeds the solver up). Half a step rather
            % than the whole correction, because slice wobbles by a few ms per cycle.
            %
            % THE FLOOR IS R AND IS NOT TIGHTER THAN THAT, deliberately. r = R is a corner
            % that cannot deliver the target -- the drawing cost sits outside the slice, so
            % the factor is r*slice/(u+slice) -- but it is a transient and not a trap: the
            % ratio above returns 1/0.82 there, so r climbs out of it about 10% a cycle,
            % which at 10 fps is under a second. Flooring at the rate that would deliver
            % the target instead, R*cycle/slice, was tried and is the geometric solve
            % wearing a different hat: it overshot to 1.10x and, being a floor, the ratio
            % loop could not come back down through it.
            %
            % AND IT NEVER SLOWS A SHOW DOWN, which matters more than the frame rate.
            % Asking for more than the run can manage makes pacing inert
            % rather than slow -- it only ever waits when the sim clock is AHEAD of the rate
            % it was given. Those two are the same condition: the correction above is above
            % 1 exactly when dSim < R*cycle, so a show that cannot make the target drives r
            % UP, past anything it could deliver, and pacing stops binding at all. Measured
            % on a deliberately unaffordable viewer (300 ms frames): 0.72x paced against
            % 0.76x free-running, and gradeLiveRate then drops the throttle outright.
            %
            % AND THE SLICE IS STILL SOLVED, not pinned at the floor. Asking for PollMin
            % was the first version and it cost more than it bought: the loop then yields
            % three times as often, and the solver's throughput per second of granted time
            % drops with the slice length (measured: 6.1 sim-s/s at 88 polls against 3.2 at
            % 212, for the identical show). During the show that is affordable -- 1.00x at
            % 10.2 fps against the fallback's 1.00x at 5.3 -- but during the upload and arm
            % phases, where the MAVLink chains are executing and the solver is far slower,
            % the same short slice delivered 0.57x where the coarsely-polled fallback held
            % 1.03x. Slower, which is the one outcome the throttle may not produce.
            %
            % So the slice is sized by WHICH THING IS ACTUALLY THE LIMITER this cycle, and
            % the test is whether pacing is binding -- whether the solver was held back, or
            % was merely as fast as it gets:
            %   held back   -> there is headroom, and a shorter slice may be affordable.
            %                  Ratchet p down and let r rise to keep the rate. This is what
            %                  buys the frames, and it stops on its own: shrink far enough
            %                  and the solver stops keeping up, which is the other case.
            %   as fast as  -> the machine is the limiter, so S is the honest machine rate
            %   it goes        and the fallback's solve says outright what slice the target
            %                  needs; p goes there, up or down -- to the ceiling when
            %                  S <= R, which hands the paced run the fallback's slice and
            %                  with it the fallback's capability. Immediately, because
            %                  falling behind is urgent where a frame is not.
            % The solve alone was not enough, which is worth recording: while pacing binds
            % it is a FIXED POINT (substitute S = R*(u+p)/p and it returns p unchanged), so
            % it can restore a slice but never shrink one, and p stayed where the heavy
            % phase had put it for the rest of the run -- 5.6 fps where the ratchet gets
            % 10.2 on the same show. Only trying a shorter slice can discover it is
            % affordable, because while pacing binds, S measures the pacing rate rather
            % than the machine. Nor is the ratchet enough on its own: a 300 ms viewer never
            % lets pacing bind at all, so the ratchet never fires, and clamping the solve
            % to max(p, ...) left the slice wherever it had drifted and the show at 1.36x.
            % Each lever works only in the case the other cannot see.
            if app.PacingOn
                w = 0;                % nothing to withhold; Simulink is doing the waiting
                p = app.PollInterval;
                pWas = p;             % for the feedforward at the end of the branch
                if dSim > 0 && dWall > 0 && dGrant > 0
                    % Was the solver held back, or is that simply as fast as it goes?
                    % Pacing caps the advance to r per granted second, so being held back
                    % shows up as dSim reaching r times the granted time -- a little over
                    % it, in fact, since pacing also catches up inside the slice.
                    bound = dSim >= 0.9 * app.PacingRate * dGrant;
                    rNeed = app.PacingRate * R * dWall / dSim;
                    % Gain scaled by the sample's share of a second, for the same reason
                    % the filters are: a fixed step per SAMPLE would settle wherever the
                    % equally-weighted mean is R, and the run average is the wall-weighted
                    % one. Like this the loop moves about half way to what it needs per
                    % second of wall clock however many cycles that took.
                    %
                    % Taken in the log domain because this is a correction on a RATIO:
                    % rNeed/r is the factor the rate is out by, and a linear step of the
                    % same gain moves a fraction of how LARGE r is rather than of how WRONG
                    % it is. That matters at the blind opening ask of PacingRateMax, which
                    % is 13x above where an ordinary viewer settles: linearly, a 100 ms
                    % cycle walks down from it at 5%/cycle and a 120-cycle run ended still
                    % descending, at 1.12x. Multiplicatively the same gain takes a fifth as
                    % long. The fixed point is untouched -- rNeed == r still means no move.
                    %
                    % Smoothed only while pacing BINDS, which is the case the smoothing is
                    % for: there r is the thing setting the rate, and a step straight to
                    % what the last cycle asked for would chase every lumpy chunk. When
                    % pacing is not binding, r is inert -- the machine is delivering less
                    % than r would allow -- so it can be put anywhere without touching the
                    % show, and the useful place is the exact rate that would deliver the
                    % target over the whole cycle: dSim = R*dWall at r per granted second.
                    % Walking there smoothly instead cost the expensive viewer the run: from
                    % the opening ask of 20 it was still at 10.2 when the show ended, having
                    % never regained authority, and the run came out at 1.13x on feedforward
                    % alone. Jumping cannot cause a dip, because r above the binding point
                    % throttles nothing.
                    %
                    % And smoothed only on the way DOWN. Going up means the show is behind
                    % the target, which is the one thing the throttle may not cause, so it
                    % goes straight to what the last cycle says it needs -- the same reason
                    % the slice below goes up in one step and down in probes. Overshooting
                    % upward costs a cycle that ran fast, which is allowed; smoothing upward
                    % cost the run. Note that rNeed IS the unbiased geometric answer, so
                    % this is also what makes r = R safe to keep as the hard floor: pacing
                    % at exactly the target rate per GRANTED second is below the target per
                    % CYCLE, always, so the floor has to be a place r leaves at once. It did
                    % not, at 1%/cycle -- the run ended pinned there at 0.90x.
                    if ~bound
                        r = R * dWall / dGrant;
                    elseif rNeed > app.PacingRate
                        r = rNeed;
                    else
                        r = app.PacingRate * (rNeed / app.PacingRate) ^ (0.5 * aW);
                    end
                    app.PacingRate = min(max(r, R), app.PacingRateMax);
                    if app.AchievedRate < 0.98 * R
                        app.BehindWall = app.BehindWall + dWall;
                    else
                        app.BehindWall = 0;
                    end
                    if bound
                        % Bound and short of the target means r is too low, which the loop
                        % above is already correcting -- and the slice must NOT be cut here,
                        % because while pacing binds a shorter slice is a slower show.
                        if dGrant > 1.1 * dPause
                            % Stalled polls: this advance needed more granted time than one
                            % poll gave it, so the polls in between each drew a frame and
                            % bought nothing. Ask for what the advance actually took. Only
                            % on the cycles that prove it, so this cannot become a floor
                            % that ratchets itself up.
                            p = max(p, dGrant);
                        elseif app.AchievedRate >= 0.98 * R
                            % Headroom, and the target is being met: spend it on frames. A
                            % quarter off per cycle, not a few percent -- the probe has to
                            % cross the whole range while the cycles are still long, and at
                            % 8% a slice parked at PollMax by the upload phase needed 26
                            % cycles of 540 ms to walk back, which outlasted the show (the
                            % run ended still converging, at 2.6 fps and 1.61x). It self-
                            % accelerates: every step down shortens the cycle it is measured
                            % over. Overshooting costs one short cycle, because the branch
                            % below puts the slice straight back.
                            p = max(0.75 * p, app.PollMin);
                        elseif app.BehindWall >= 0.5
                            % Behind the target while pacing binds, and r has had half a
                            % second to fix it and has not. Normally r is the lever for
                            % this, but the bind test cannot tell "pacing is holding the
                            % solver back" from "the machine happens to deliver about what
                            % pacing allows", and in that ambiguous zone a falsely-bound
                            % cycle never reaches the solve below -- so without this the
                            % slice would never come back and the show sat at 0.907x with a
                            % longer one available the whole time. Growing is always the
                            % safe direction, because a longer slice is more sim time per
                            % frame drawn whichever thing turns out to be the limiter.
                            %
                            % On a wall-clock timer rather than every cycle, because the two
                            % loops would otherwise be fighting over one error signal with
                            % gains three orders apart: 25% per cycle here against the rate
                            % loop's ~1%, so the slice raced to the ceiling while r crawled,
                            % and each step up raised the rate r had to reach. Measured that
                            % way the paced run gave up its whole frame advantage -- a 100 ms
                            % cycle against the fallback's 61 ms, 5.8 fps against 5.9. The
                            % timer makes it a slow probe of last resort instead.
                            p = min(p / 0.75, app.PollMax);
                            app.BehindWall = 0;
                        end
                    elseif isempty(app.FrameCostEMA) || isempty(S)
                        % Nothing measured yet, so open where the fallback opens: as much
                        % solver time as the loop can grant. Opening at the last interval
                        % instead left the first cycles of an expensive viewer at 0.47x
                        % against the 0.75x the same show managed unthrottled -- not pacing
                        % (at the opening ask it cannot even bind), just too short a slice.
                        p = app.PollMax;
                    elseif S <= R
                        p = app.PollMax;                      % out of reach; grant the most
                    else
                        % Exactly what the target needs, priced per ADVANCE rather than per
                        % frame -- see FrameCostEMA. Solved outright, up or down: here the
                        % machine is the limiter, so S is the honest machine rate and this
                        % is the slice at which it delivers the target.
                        p = R * app.FrameCostEMA / (S - R);
                    end
                    % Whatever the branch above just did to the slice, pay for it in the
                    % rate now instead of leaving the feedback loop to discover it. The
                    % slice change is OURS, so its arithmetic is known exactly: holding
                    % dSim = R*dWall with dSim = r*p and dWall = cost + p makes the rate a
                    % function of the slice, r = R*(cost + p)/p. Left to the loop the two
                    % ran at gains three orders apart -- the cold-start ratchet took seven
                    % 25% cuts in the time r managed one 1% step, so the slice arrived at
                    % 100 ms with pacing still asking 1.16 where 1.40 was needed, and the
                    % first five seconds of the show ran at 0.70x. Applied as a RATIO of
                    % the two requirements rather than as the requirement itself, which is
                    % what cancels the geometric form's ~10% bias.
                    %
                    % One-sided, like the loop above: it may only ask for MORE. A slice that
                    % grew needs less rate, but handing rate back on a prediction rather
                    % than a measurement is how the rate reached its floor and stayed there
                    % (r = 1.00, the show at 0.90x, a slice that had grown a few times in a
                    % row each knocking 22% off). Giving rate back is the smooth loop's job,
                    % because only it is working from what the show actually delivered.
                    if ~isempty(app.FrameCostEMA) && p > 0 && pWas > 0
                        c = app.FrameCostEMA;
                        app.PacingRate = min(max(app.PacingRate, app.PacingRate * ...
                            (pWas / p) * ((c + p) / (c + pWas))), app.PacingRateMax);
                    end
                end
                p = min(max(p, app.PollMin), app.PollMax);
                app.PollInterval = p;
                app.HoldInterval = 0;
                pace = app.gradeLiveRate(true);
                return;
            end
            pace = app.gradeLiveRate(false);

            % Ordered so no branch ever compares against an empty S: `[] <= R` is
            % empty, and an empty operand to || is an error, not a false.
            if isempty(u)
                % Nothing has been drawn yet, so there is no u to solve against: poll as
                % little as a streamed replay can stand until there is.
                p = app.PollMax;
            elseif isempty(S)
                p = 0.1;                      % nothing measured yet; the old fixed rate
            elseif S <= R
                p = app.PollMax;              % target unreachable at any pause length
            else
                p = R * u / (S - R);
            end
            p = min(max(p, app.PollMin), app.PollMax);
            app.PollInterval = p;

            % And the hold. NOT solved from S and p, which was the first attempt and missed
            % the target by 1.9x: `S*p` is the sim time a cycle should advance if the
            % solver's slice were the pause we ASKED for, and it is not. pause(0.02) comes
            % back after ~50 ms, because Simulink runs a whole chunk before returning
            % control, and that chunk advanced ~315 ms of sim time -- against the 123 ms the
            % S*p model predicted. Worse, S is not even independent of the answer: polling
            % less often measurably speeds the solver up (6.1 sim-s/s at 88 polls against
            % 3.2 at 212 for the identical show), so any open-loop sizing is solving for a
            % plant that changes when it acts.
            %
            % So the hold is a controller on the MEASURED error instead, and it assumes
            % nothing about where the sim time came from: this cycle advanced dSim, at the
            % target it should have taken dSim/R of wall clock, and it took dCycle. Feed
            % half the shortfall back into the hold and let it settle.
            %
            %     w <- w + gain * (dSim/R - dCycle)
            %
            % Integral, not proportional-and-done, because the disturbance is persistent:
            % the show is simply faster than real time and something has to absorb the
            % difference every cycle. Half-gain because the plant moves when the controller
            % does, and a deadbeat step on a measurement that includes its own effect rings.
            w = app.HoldInterval;
            if dSim > 0 && dWall > 0
                % dWall, not this cycle: the same carried-forward wall clock the rate
                % above is measured against, so the hold is sized from what the advance
                % really cost rather than from the last slice of it.
                w = w + 0.5 * (dSim / R - dWall);
            end
            % Capped, because the hold is BLOCKING: nothing is serviced while it runs, Stop
            % included. A quarter second of latency on a button is the edge of tolerable. A
            % target that needs more than this is unreachable from ABOVE, which is the same
            % situation as S <= R is from below -- the label reports what was achieved and
            % does not pretend the target was met.
            w = min(max(w, 0), app.HoldMax);
            app.HoldInterval = w;
        end

        function pace = gradeLiveRate(app, paced)
            % Is the throttle currently in use the right one? Hysteretic, and on the
            % ACHIEVED factor, which is measured the same way in both states so the two
            % directions cannot disagree about what real time was.
            pace = paced;
            R = app.RateTarget;
            if isnan(app.AchievedRate) || isempty(app.CycleEMA)
                return;
            end
            if paced
                % Under target while paced means the viewer costs more than the solver's
                % excess can cover, and no pacing rate fixes that -- r has already been
                % driven past what the run can do in a slice, so pacing is no longer even
                % binding. Stop asking, and let the show run at whatever it can manage,
                % which is the fastest it is ever going to go. This is the guarantee that
                % the throttle can only ever hold a fast show back and never drag a slow
                % one down.
                %
                % ONLY ONCE THE OTHER LEVER IS SPENT, though: being under target with the
                % slice still short says the slice is short, not that pacing cannot work,
                % and nextPollInterval is already lengthening it. Judging before that had
                % the two fighting -- every downward probe of the ratchet reads as a dip,
                % and three of them in the opening transient were enough to make the
                % throttle let go of a show it went on to hold at 1.00x. So the verdict
                % waits until there is nothing left to give the solver: the slice at its
                % ceiling, or the rate at its own.
                spent = app.PollInterval >= 0.98 * app.PollMax || ...
                        app.PacingRate >= 0.98 * app.PacingRateMax;
                if spent && app.AchievedRate < 0.9 * R
                    app.LowRateWall = app.LowRateWall + app.CycleEMA;
                else
                    app.LowRateWall = 0;
                end
                if app.LowRateWall > 1
                    pace = false;
                    app.LowRateWall = 0;
                    app.HighRateWall = 0;
                end
            else
                % And back again, because the cost is not fixed for the run: trails fill
                % up, drones land and stop being drawn, the operator can switch the render
                % mode mid-show, and the solver rate itself moves by a large factor
                % between the MAVLink phases and the show.
                %
                % A live hold is the signal, not the achieved factor: the fallback law
                % holds a fast show AT the target, so the factor reads 1.00x whether the
                % show had 1.1x of headroom or 5x of it. A hold above zero is exactly the
                % statement "wall clock is being wasted to slow this down", which is the
                % job pacing does better.
                if app.HoldInterval > 0 || app.AchievedRate > 1.05 * R
                    app.HighRateWall = app.HighRateWall + app.CycleEMA;
                else
                    app.HighRateWall = 0;
                end
                if app.HighRateWall > 1
                    pace = true;
                    app.LowRateWall = 0;
                    app.HighRateWall = 0;
                end
            end
        end

        function pacingArm(app, modelName)
            % Baseline for this run, taken before the model starts.
            %
            % Normalises to OFF on the way out rather than restoring whatever was found:
            % the throttle exists for the live view, and a value left behind would
            % silently pace the next plain sim() -- including the ones DroneShowExample
            % runs, where a 25 s show would then take 25 s of wall clock and read as a
            % performance regression in the example. The model ships with pacing off, so
            % off is the honest baseline as well as the safe one. The rate is preserved
            % because that one is a harmless dialog setting.
            app.PacingOn = false;
            app.PacingFailed = false;
            app.PacingWritten = NaN;
            app.PacingWas = {};
            if ~app.PaceLive
                return;
            end
            try
                % The rate only. The dirty flag is not saved here because pacingWrite
                % preserves it across each individual write, which is both narrower and
                % correct on the paths that never come back through this method.
                app.PacingWas = {get_param(modelName, 'PacingRate')};
                app.pacingWrite(modelName, 'off', app.PacingWas{1});
            catch
                % An older release without the parameters at all, or a model that will not
                % take them. The spin hold governs the whole run and nothing else changes.
                app.PacingFailed = true;
            end
        end

        function pacingEngage(app, modelName, want)
            % Hand the waiting to Simulink, keep the rate current, or take it back.
            % Called every governed cycle, so it must be cheap: the set_params only happen
            % when the state or the rate has actually moved.
            want = want && app.PaceLive && ~app.PacingFailed;
            if ~want && ~app.PacingOn
                return;
            end
            try
                if want
                    r = app.PacingRate;
                    % 2% band. Without it every cycle writes a parameter that has not
                    % meaningfully changed, on the UI thread, in the loop whose whole
                    % problem is how much time it spends off the solver.
                    if ~app.PacingOn || ~(abs(r - app.PacingWritten) <= 0.02 * max(r, 1))
                        app.pacingWrite(modelName, 'on', num2str(r, '%.4f'));
                        app.PacingWritten = r;
                    end
                    app.pacingMark(true);
                else
                    rate = '1';
                    if ~isempty(app.PacingWas)
                        rate = app.PacingWas{1};
                    end
                    app.pacingWrite(modelName, 'off', rate);
                    app.pacingMark(false);
                end
            catch
                % Pacing is a comfort, not a requirement. A mode that refuses it or a model
                % that has already terminated both land here, and the spin hold takes over.
                app.pacingMark(false);
                app.PacingFailed = true;
            end
        end

        function pacingMark(app, on)
            % The state change itself, with no model write in it, so the poll loops and the
            % test seam go through the same transition rather than two versions of it.
            if on
                if ~app.PacingOn
                    % THE OPENING ASK IS DELIBERATELY GENEROUS, and this is the pacing
                    % counterpart of the fallback opening at p = PollMax. A first ask that
                    % is too low holds the show below the target from the first cycle,
                    % which is the one thing the throttle must never do; one that is too
                    % high simply does not bind, and the loop walks it down inside a
                    % second. So: the geometric estimate of what the target needs where
                    % there is anything measured to compute it from -- its known ~10% bias
                    % does not matter in an opening guess -- and otherwise a rate that
                    % cannot bind at all, which leaves the first cycles running exactly as
                    % they would with no throttle.
                    if ~isempty(app.CycleEMA) && ~isempty(app.UICostEMA)
                        slice = max(app.CycleEMA - app.UICostEMA, eps);
                        app.PacingRate = min(max( ...
                            app.RateTarget * app.CycleEMA / slice, ...
                            app.RateTarget), app.PacingRateMax);
                    else
                        app.PacingRate = app.PacingRateMax;
                    end
                end
                app.PacingOn = true;
                return;
            end
            app.PacingOn = false;
            app.PacingWritten = NaN;
            % Fresh S on the way out. Every sample it holds was taken while Simulink was
            % throttling the solver, so it measures the pacing rate rather than the machine,
            % and the fallback law is about to size the poll interval from it.
            app.SolverRateEMA = [];
        end

        function pacingWrite(~, modelName, onoff, rate)
            % Dirty preserved across both writes. Without it a live run leaves the model
            % marked modified, and the next close_system offers to save a throttle setting
            % the operator never touched.
            d = get_param(modelName, 'Dirty');
            set_param(modelName, 'PacingRate', rate);
            set_param(modelName, 'EnablePacing', onoff);
            set_param(modelName, 'Dirty', d);
        end

        function dHold = holdSolver(~, w)
            % Burn wall clock while making sure the solver gets NONE of it.
            %
            % THE FALLBACK, not the usual path: pacing does this job inside the solver
            % wherever the model accepts it, and does it better -- see the PacingOn branch
            % in nextPollInterval. This runs when pacing has been refused or dropped.
            %
            % A pause() here would defeat the purpose: pause is precisely how this loop
            % hands time to the asynchronous run, so pausing to slow the show down would
            % speed it up. drawnow is the same. What is left is to keep the MATLAB thread
            % busy in interpreted code, which the run cannot preempt -- so the hold is a
            % spin. It burns a core to do nothing, which is the honest cost of holding a
            % show that the machine could fly three times faster than real life.
            %
            % Returned measured rather than as asked for, because that is what the achieved
            % factor has to be computed against.
            dHold = 0;
            if ~(w > 0)
                return;
            end
            tH = tic;
            while toc(tH) < w
                % Deliberately empty. See above: anything that yields hands the solver
                % time, and time is what is being withheld.
            end
            dHold = toc(tH);
        end

        function refreshRateLabel(app)
            % What was actually achieved, next to what was asked for. Without this the
            % control law is unfalsifiable from the UI: a target of 1x that silently
            % delivers 0.6x looks exactly like one that delivers 1.0x.
            if isempty(app.RateLabel) || ~isvalid(app.RateLabel)
                return;
            end
            if isnan(app.AchievedRate)
                app.RateLabel.Text = 'Live Rate: --';
                return;
            end
            % The MEASURED cycle, where there is one. The cycle was once reconstructible as
            % poll + hold + u, because those three were the whole of it; under pacing they
            % are not -- the loop asks for a 20 ms pause and the solver returns after ~60,
            % so the sum understates the frame period by 3x and would advertise 40 fps on a
            % stream running at 15. The reconstruction stays as the fallback for the first
            % cycle or two, before anything has been timed.
            if ~isempty(app.CycleEMA)
                cyc = app.CycleEMA;
            else
                u = app.UICostEMA;
                if isempty(u), u = 0; end
                cyc = app.PollInterval + app.HoldInterval + u;
            end
            fps = 1 / max(cyc, eps);
            % Which throttle is doing the work, and what it is asking for. Worth the
            % characters: 1.00x at 4 fps and 1.00x at 15 fps are the same reading of the
            % same show and the difference is the entire complaint the pacing path exists
            % to answer, so the label has to distinguish them.
            if app.PacingOn
                how = sprintf('paced %.2fx', app.PacingRate);
            else
                how = sprintf('poll %.0f ms', 1000 * app.PollInterval);
            end
            app.RateLabel.Text = sprintf('Live Rate: %.2fx of %gx (%.1f fps, %s)', ...
                app.AchievedRate, app.RateTarget, fps, how);
        end

        function summarizeLiveRate(app, wall, simSpan)
            % Called once, after the poll loop has exited. Replaces the last poll's
            % instantaneous factor with the factor the whole governed section ran at,
            % which is the number an operator is actually asking about when they look at
            % this label after a show.
            %
            % Both arguments span the SAME window -- from the first poll the controller
            % governed to the last -- and both are measured rather than derived. wall is
            % not computed as sumU + sumP, because those two do not add up to it: the loop
            % also spends time in the telemetry repaint throttle, in drawnow, and inside
            % pause() overshooting what it was asked for. That gap is the interesting
            % part, so it must not be defined away. simSpan is the sim clock's own
            % difference across the window rather than the sum of the per-poll deltas, so
            % it cannot drift from the model's idea of how far it got.
            st = app.PollStats;
            if st.n == 0 || ~(wall > 0) || ~(simSpan > 0)
                return;
            end
            app.ShowRate = simSpan / wall;
            if isempty(app.RateLabel) || ~isvalid(app.RateLabel)
                return;
            end
            fps = st.n / wall;
            app.RateLabel.Text = sprintf('Live Rate: %.2fx of %gx over the run (%.1f fps)', ...
                app.ShowRate, app.RateTarget, fps);
        end

        function armRtkTier(app)
            % Called once per run, before the model starts. Caches the tier table so
            % updateRtkTier never has to reach into the base workspace: the run is
            % asynchronous but single-threaded with the UI, so every millisecond a poll
            % spends is a millisecond the solver does not get, and an evalin costs
            % orders of magnitude more than a numeric compare.
            app.SigmaTiers = [];
            try
                app.SigmaTiers = evalin('base', 'rtk_sigma_table');
            catch
            end
            app.RTKTierLabel.Text = '…';
            app.RTKTierLabel.FontColor = [0.35 0.35 0.35];
        end

        function updateRtkTier(app, rtoS)
            % Which tier is the fleet actually on, right now — the worst any drone is
            % on, plus how many are off RTK Fix. This is the readout that replaced the
            % RTK Mode dropdown: the same three names, but reporting rather than
            % pretending to set.
            %
            % Read from InjectGate's output port, which is where the posSigma log is
            % taken from too, so the readout and the log cannot disagree. Not from the
            % To Workspace block — those have no RuntimeObject.
            %
            % It used to read SigmaRateLimit, and that was measurably the wrong tap.
            % CorrectionAgeMonitor has two sigma paths and SigmaRateLimit is only one
            % of them: the correction-AGE staircase, which contributes 0.0000 m for an
            % undegraded drone in both modes, because corrAge peaks at 1.990 s when
            % packet_loss_rate erases a broadcast and rtk_timeout is 2.50 s — so one lost
            % correction does not reach the staircase's first breakpoint. It took that
            % margin to be true: at the old 1.5 s timeout a single 1 % roll took the WHOLE
            % fleet to Float at 0.299 m for ~0.5 s, and since the roll is transmitter-side
            % every drone stepped up to 0.8 m sideways within one Ts_sim. The shipped seed
            % draws two such rolls over the run's 62 broadcasts and one of them lands
            % mid-climb, which is how it got reported as a takeoff glitch. What still
            % drives this term is rtk_deny_mask, which drives it permanently, and that is
            % the tap this readout has to be right about. The metre-class term is the other path — the base
            % engine's own live sigma, 0.17-1.60 m until it reports L1 fixed at t = 13 s.
            % So the panel read "RTK Fix (2 cm)" from t = 0 through the whole
            % convergence, i.e. it was confident exactly where the story is. InjectGate
            % is downstream of both paths AND of the fast-mode gate, so it reports the
            % sigma the fleet actually receives: Standalone/Float while the base
            % converges, and Fix from t = 0 when there is no link to model.
            if isempty(rtoS) || isempty(app.SigmaTiers), return; end
            sig = double(rtoS.OutputPort(1).Data);
            if isempty(sig), return; end

            % Classified at half of each injected tier rather than by equality: the
            % rate limiter makes sigma a continuous ramp on the way back down, so a
            % recovering drone sits between tiers for reconverge_time.
            floatInj = app.SigmaTiers(2);
            standInj = app.SigmaTiers(3);
            worst = max(sig(:));
            nOff  = nnz(sig(:) > 0.5 * floatInj);
            nAll  = numel(sig);

            if worst <= 0.5 * floatInj
                txt = 'RTK Fix (2 cm)';
                col = [0.15 0.5 0.15];
            elseif worst <= 0.5 * standInj
                txt = sprintf('Float — %d of %d', nOff, nAll);
                col = [0.75 0.55 0.05];
            else
                txt = sprintf('Standalone — %d of %d', nOff, nAll);
                col = [0.8 0.2 0.1];
            end

            % Only touch the properties when the text actually changes. Setting
            % uifigure properties is the expensive half of a poll, and for most of a
            % nominal show this string never moves off 'RTK Fix'.
            if ~strcmp(app.RTKTierLabel.Text, txt)
                app.RTKTierLabel.Text = txt;
                app.RTKTierLabel.FontColor = col;
            end
        end

        function armCommStatus(app, modelName)
            % Resolve the Radio-traffic taps once, before the run starts. Five signals, all
            % of them already in the model -- this readout is display-only and needed no
            % model edit.
            %
            % Resolved by LEAF NAME rather than by literal path. Two of these have already
            % been reparented once by the gating work, and a stale literal path throws
            % Simulink:Commands:InvSimulinkObjectName, which reads as "the block is gone"
            % and sends you looking at the wrong change.
            %
            % Why these blocks and not the obvious ones: MissionAckEncoder and
            % MissionRequestEncoder are VIRTUAL subsystems (TreatAsAtomicUnit is off), and a
            % virtual block has no RuntimeObject at all -- polling them returned empty and
            % the readout showed '?' for the whole run. The flags come off the Logic blocks
            % that drive those subsystems' flag outports instead, which are real blocks.
            % PATHS here, handles later. A RuntimeObject only exists while the model is
            % running, and this method is called before SimulationCommand start -- asking
            % now returns empty for every tap, permanently, and the panel would read '—'
            % for the whole run with nothing to say why. fetchCommHandles does the
            % dereference from inside the poll loop, alongside the pose and phase handles.
            app.RadioRto = struct();
            taps = { ...
                'ul',   'Sw_CL',         1; ...   % uplink selector, 1..6
                'ack',  'AckActiveAnd',  1; ...   % an ack is on the wire now
                'req',  'ReqActive',     1; ...   % a gap is being re-requested
                'rtcm', 'PhaseGE2',      1; ...   % the correction broadcast is live
                'age',  'AgeReset',      1};      % correction age, [N x 1]
            for k = 1:size(taps, 1)
                p = '';
                try
                    % MatchFilter is not decoration: a bare find_system on this model warns
                    % (Simulink:Commands:FindSystemDefaultVariantsOptionWithVariantModel)
                    % because it silently skips inactive variant choices today and will stop
                    % doing so in a future release. Five taps meant five warnings dumped into
                    % every full-fidelity run log. activeVariants asks for exactly what a
                    % RuntimeObject tap needs -- the choice that will actually execute -- and
                    % it resolves without a compile: all five taps return the same single hit
                    % as the bare call. NOT allVariants, which would also return inactive
                    % duplicates, and b{1} would then be free to pick a block that never runs
                    % and hands back an empty handle, i.e. a silent '—'.
                    b = find_system(modelName, 'LookUnderMasks', 'all', ...
                        'FollowLinks', 'on', 'MatchFilter', @Simulink.match.activeVariants, ...
                        'Name', taps{k, 2});
                    if ~isempty(b), p = b{1}; end
                catch
                    % Left empty: a missing tap costs one '—' on the panel and nothing else.
                end
                app.RadioRto.(taps{k, 1}) = struct('path', p, 'h', [], 'port', taps{k, 3});
            end
            app.setRadioLabels('—', '—', '—', '—', [0.35 0.35 0.35]);
        end

        function fetchCommHandles(app)
            % Dereference the cached paths into RuntimeObjects. Called from the poll loop
            % once the run is up, and cheap to call again: each tap is resolved at most once
            % because a successful handle is kept and a failed one leaves .h empty, which is
            % the same thing the readout does with a missing tap.
            if ~isstruct(app.RadioRto), return; end
            for f = fieldnames(app.RadioRto)'
                t = app.RadioRto.(f{1});
                if ~isempty(t.h) || isempty(t.path), continue; end
                try
                    app.RadioRto.(f{1}).h = get_param(t.path, 'RuntimeObject');
                catch
                end
            end
        end

        function updateCommStatus(app, ph)
            % Name the message on the wire in each direction, right now. Called on the same
            % ~5 Hz throttle as the telemetry text, never per frame: setting uifigure
            % properties is the expensive half of a poll and these strings are stable for
            % seconds at a time.
            if ~isfield(app.RadioRto, 'ul'), return; end

            ulNames = {'HEARTBEAT', 'MISSION_COUNT', 'MISSION_ITEM_INT', ...
                       'COMMAND_LONG', 'SYSTEM_TIME', 'idle'};
            i = app.readTap('ul', NaN);
            if isnan(i) || i < 1 || i > numel(ulNames)
                ulTxt = '—';
            else
                ulTxt = ulNames{round(i)};
            end

            % The cascade's own precedence, so the label cannot claim two messages at once:
            % an ack outranks a re-request, which outranks the pose that is otherwise always
            % flowing.
            %
            % Defaulted to NaN and not to 0, which matters more than it looks: with a 0
            % default an UNRESOLVED tap falls through to LOCAL_POSITION_NED, and a dead
            % readout is then indistinguishable from a correct one. That is the bug this
            % readout already hit once -- MissionAckEncoder and MissionRequestEncoder are
            % virtual subsystems and have no RuntimeObject, so the flags come off the Logic
            % blocks driving them (AckActiveAnd, ReqActive). An empty handle now says '—'.
            ack = app.readTap('ack', NaN);
            req = app.readTap('req', NaN);
            if any(isnan(ack)) || any(isnan(req))
                dlTxt = '—';
            elseif any(ack > 0.5)
                dlTxt = 'MISSION_ACK';
            elseif any(req > 0.5)
                dlTxt = 'MISSION_REQUEST_INT';
            else
                dlTxt = 'LOCAL_POSITION_NED';
            end

            rtcm = app.readTap('rtcm', NaN);
            age  = app.readTap('age',  NaN);
            if any(isnan(rtcm))
                rtTxt = '—';
            elseif any(rtcm > 0.5)
                rtTxt = sprintf('GPS_RTCM_DATA, age %.1f s', max(age(:)));
            else
                rtTxt = 'silent (no broadcast yet)';
            end

            % What the viewer is drawing, and what that costs. Stated rather than implied:
            % the pre-show hold is the one place the app shows something other than the
            % pose it received, so it says so, and it says by how much.
            if ph < 3 && ~isempty(app.PadPose)
                vwTxt = sprintf('pads — %.1f m offset held back', app.PadHoldOffset);
            else
                vwTxt = 'received MAVLink pose';
            end

            app.setRadioLabels(ulTxt, dlTxt, rtTxt, vwTxt, []);
        end

        function v = readTap(app, name, dflt)
            % One cached RuntimeObject read, with every way it can fail folded into the
            % default. A tap that goes stale must degrade to '—' on a label, never throw
            % inside a poll: the poll loop is what keeps the Stop button alive.
            v = dflt;
            try
                t = app.RadioRto.(name);
                if ~isempty(t.h)
                    v = double(t.h.OutputPort(t.port).Data);
                end
            catch
            end
        end

        function setRadioLabels(app, ul, dl, rt, vw, col)
            % Only touch a property when its text actually changes -- the same rule
            % updateRtkTier follows, and for the same reason: for most of a run these four
            % strings do not move, and a uifigure write costs the solver.
            pairs = {app.UplinkMsgLabel, ul; app.DownlinkMsgLabel, dl; ...
                     app.RtcmMsgLabel, rt; app.ViewSourceLabel, vw};
            for k = 1:size(pairs, 1)
                h = pairs{k, 1};
                if isempty(h) || ~isvalid(h), continue; end
                if ~strcmp(h.Text, pairs{k, 2})
                    h.Text = pairs{k, 2};
                end
                if ~isempty(col)
                    h.FontColor = col;
                end
            end
        end

        function pos = poseForView(app, pos, ph)
            % Park the fleet on its pads until the show starts, then draw what arrived.
            %
            % WHY THIS EXISTS. The received pose carries each drone's own navigation
            % estimate, and before the RTK engine resolves ambiguities the fleet is standing
            % still on a 5 m grid while being drawn up to 3.7 m off its pads -- which reads
            % as a scrambled grid rather than as a stationary fleet.
            %
            % AND WHAT IT HIDES, which matters more than what it fixes. Measured over 1600
            % pre-show samples at N = 20: displayed-vs-pad peaks at 3.71 m, but
            % TRUE-airframe-vs-pad peaks at 3.33 m. The display adds only 0.38 m.
            % So all but a few centimetres of that offset is the fleet REALLY station-keeping
            % badly on a bad estimate -- the position loop is closed on the estimate from
            % t = 0, and the launch is fix-interlocked while the airframe is not. Parking the
            % view is therefore cosmetic, and the number it suppresses is put on the Radio
            % link panel so it is reported rather than lost.
            %
            % RELEASED ON THE PHASE, not on the RTK fix. A drone under rtk_deny_mask keeps a
            % high sigma for the whole run, so a "the fleet is fixed" test would never go
            % true and the viewer would sit on the pads through the entire show. Phase >= 3
            % is also the literal statement of the thing being claimed -- they are not flying
            % yet. Measured: the engine fixes at t = 13.00 s and SHOW is entered at 16.78 s,
            % so the hold releases 3.8 s AFTER the pose became trustworthy, not before it.
            % UPLOAD_FAILED is 6, so it clears "phase >= 3" while meaning the opposite of
            % it: the upload never completed, the fleet never armed, and it is standing on
            % the pads on a raw estimate -- precisely the picture this hold exists to keep
            % off the screen. Excluded by number rather than by rewriting the test, because
            % the ordering IDLE < UPLOADING < ARMED < SHOW is real and 6 is simply bolted on
            % past the end of it.
            if (ph >= 3 && ph ~= 6) || isempty(app.PadPose) || ...
                    size(app.PadPose, 1) ~= size(pos, 1)
                app.PadHoldOffset = 0;
                return;
            end
            app.PadHoldOffset = max(vecnorm(pos - app.PadPose, 2, 2));
            pos = app.PadPose;
        end

        function refreshDenyLabel(app, m)
            % Which drones are currently denied, spelled out. A degradation left set
            % from an earlier press is exactly the kind of thing that gets forgotten
            % and then read as a model bug, so it is on the panel rather than implied.
            if nargin < 2
                m = app.currentDenyMask(app.denyMaskLength());
            end

            % Cache it for the 3-D view. This method is the single funnel every path to the
            % mask goes through -- setRtkDeny, generateShow and the nargin<2 fallback all end
            % up here -- so caching HERE is what makes the red highlight follow the mask
            % without drawFrame having to evalin('base','rtk_deny_mask') sixty times a second.
            % Stored as a full logical column of the fleet length so getUAVColors can index it
            % directly; an empty mask stays empty and costs nothing.
            app.DenyMaskCache = logical(m(:));

            idx = find(m(:))';
            if isempty(idx)
                app.DenyLabel.Text = 'none';
                app.DenyLabel.FontColor = [0.35 0.35 0.35];
                return;
            end
            if numel(idx) == numel(m)
                app.DenyLabel.Text = sprintf('all %d', numel(m));
            elseif numel(idx) > 6
                app.DenyLabel.Text = sprintf('%d drones (%d, %d, … %d)', ...
                    numel(idx), idx(1), idx(2), idx(end));
            else
                app.DenyLabel.Text = strjoin(string(idx), ', ');
            end
            app.DenyLabel.FontColor = [0.75 0.35 0.05];
        end

        function stopAll(app)
            % Emergency abort: halt animation timer AND running Simulink sim
            aborted = false;
            modelName = 'MultiUAV_DroneShow';
            try
                if bdIsLoaded(modelName)
                    st = get_param(modelName, 'SimulationStatus');
                    if any(strcmp(st, {'running','paused','compiled','initializing'}))
                        set_param(modelName, 'SimulationCommand', 'stop');
                        aborted = true;
                    end
                end
            catch
            end
            app.SimRunning = false;
            % Belt and braces on the poll loops, which drop the throttle on their own way
            % out. This is the path where they do not get one: an error inside a loop
            % unwinds past that line, and EnablePacing outlives the run -- so a stopped run
            % could leave the model throttled for the next plain sim().
            app.pacingEngage(modelName, false);
            app.clearAbort(modelName);
            app.AbortBtn.Enable = 'off';
            app.stopPlayback();
            if aborted
                app.updateStatus(['STOP: Simulink simulation halted by the operator. ' ...
                    'The fleet was not landed — Abort & Land does that.']);
            end
        end

        function resampleViewerTrajectory(app)
            % Put the planned trajectory on a uniform ~20 Hz grid for the viewer.
            %
            % VIEWER ONLY. The plan that goes on the wire is `trajectory_data` in the
            % base workspace and it is NOT touched: packShowUpload reads that, and the
            % sparse keyframe grid is the whole point there -- one MISSION_ITEM_INT per
            % Ts_sim tick means keyframe count IS upload seconds. This resamples the
            % app's display copy, which nothing but the 3-D view, the extent pass, the
            % preview and the Max Speed readout ever reads.
            %
            % Columns 4:7 are dropped: the display copy is indexed (:,1:3,:) at every
            % use site, so carrying the rest at 20 Hz would be pure memory.
            tv = app.TimeVector;
            td = app.TrajectoryData;
            if isempty(tv) || isempty(td) || numel(tv) < 2
                return;
            end
            tv = tv(:);

            % Duplicate timestamps are real -- the grid marks segment boundaries with a
            % repeated instant (measured dt of exactly 0) and interp1 rejects that. Keep
            % the LAST sample at each time, i.e. the value after the discontinuity, which
            % is the one a viewer should be showing from that instant on.
            [tvu, ia] = unique(tv, 'last');
            pos = td(:, 1:3, ia);

            if numel(tvu) < 2
                return;
            end

            % 20 Hz, matching the downsample the non-streamed path already targets, but
            % capped by element count so a 500-drone show cannot resample itself into
            % hundreds of MB. The cap costs frame rate, not correctness, and only bites
            % on fleet-and-duration combinations well past anything the live view can
            % keep up with anyway.
            nUav = size(pos, 1);
            span = tvu(end) - tvu(1);
            if span <= 0
                return;
            end
            dtTarget = 0.05;
            maxElems = 8e6;                       % ~64 MB of doubles
            nMax = max(2, floor(maxElems / max(nUav * 3, 1)));
            nWant = min(nMax, max(2, floor(span / dtTarget) + 1));
            tNew = linspace(tvu(1), tvu(end), nWant);

            % Already at least this dense: leave it alone rather than resampling a
            % streamed-quality grid down to 20 Hz.
            if numel(tvu) >= nWant
                app.TrajectoryData = pos;
                app.TimeVector = tvu;
                return;
            end

            % permute so time is the first dimension, interpolate every drone and axis
            % in one call, then permute back.
            p = permute(pos, [3 1 2]);                        % [T x N x 3]
            q = interp1(tvu, reshape(p, numel(tvu), []), tNew, 'linear');
            app.TrajectoryData = permute(reshape(q, nWant, nUav, 3), [2 3 1]);
            app.TimeVector = tNew(:);
        end

        function advanceFrame(app)
            if ~app.Playing || app.PlayIdx > app.NumSamples
                if app.PlayIdx > app.NumSamples
                    app.stopPlayback();
                    app.updateStatus('Show complete.');
                end
                return;
            end

            idx = app.PlayIdx;

            if app.UseSimData
                t = app.SimTimeVector(idx);
                pos = squeeze(app.SimTrajectoryData(:, 1:3, idx));
                timeVec = app.SimTimeVector;
            else
                t = app.TimeVector(idx);
                pos = squeeze(app.TrajectoryData(:, 1:3, idx));
                timeVec = app.TimeVector;
            end

            srcLabel = '';
            if app.UseSimData, srcLabel = ' [Simulated]'; end
            app.drawFrame(pos, t, srcLabel);

            app.updateTelemetry(idx, pos, t);
            % Move the scrub thumb with the show. refreshScrub holds off on the thumb itself
            % while a drag is in progress, so there is no guard needed here.
            app.refreshScrub();

            % ---- pacing ----------------------------------------------------------
            % PACED AGAINST A WALL-CLOCK DEADLINE. This used to advance
            %   stepSize = max(1, round(PlaySpeed * samplesPerSec * 0.05))
            % samples per fixed 0.05 s pause, which made Playback speed very nearly a
            % no-op: measured on a 6-drone show sampled at 7.79 samples/s, every speed
            % from 0.25x to 2x rounded to a step of 1 and only 4x reached 2, so the
            % slider selected between "1 sample per 50 ms" and, at one end, two. That is
            % a fixed 2.6x real time whatever the operator asked for, against a label
            % promising 1.0x was real time. The floor at 1 also meant the mechanism could
            % only ever skip samples -- there was no way to slow anything down at all.
            %
            % Now the clock decides. The sample due is the one whose show time has been
            % reached at the current speed, which handles both directions with one rule:
            % below 1x it waits, above what the renderer can draw it skips, and it does
            % the right thing on a planned show sampled at ~8 Hz and on a streamed run
            % sampled irregularly at the poll interval -- an order of magnitude apart,
            % and the reason a step computed from mean sample density could not pace both.
            %
            % Derived from PaceClock rather than from the previous frame on purpose:
            % pause() overshoots by a few milliseconds every call, and 300 frames of that
            % is seconds of drift. Against the clock the overshoot is absorbed by the next
            % frame instead of accumulating.
            if isempty(app.PaceClock)
                app.PaceClock = tic;            % stepped without arming; start now
                app.PaceRefTime = t;
            end
            elapsed = toc(app.PaceClock);
            showNow = app.PaceRefTime + elapsed * app.PlaySpeed;

            % At least one sample forward, so playback always progresses even when the
            % renderer is slower than the requested speed.
            nxt = idx + 1;
            while nxt < numel(timeVec) && timeVec(nxt) < showNow
                nxt = nxt + 1;
            end
            app.PlayIdx = nxt;

            if nxt <= numel(timeVec)
                due = (timeVec(nxt) - app.PaceRefTime) / app.PlaySpeed;
                app.FrameWait = max(0, due - elapsed);
            else
                app.FrameWait = 0;
            end
        end

        function drawFrame(app, pos, t, srcLabel)
            % Paint one frame of the fleet. Shared by timer playback and by the
            % live stream off a running simulation, so both look identical.
            zUp = -pos(:, 3);                       % NED -> plot: Z-up positive

            ftype = app.formationTypeAt(t);
            colors = app.getUAVColors(ftype);

            % Colours only change when the formation does. Reassigning them every
            % frame cost 0.22 ms on the trails alone, and rather more on meshes.
            %
            % ...OR when the deny mask does, which is the trap this optimisation sets for the
            % red highlight. The scatter path below writes CData unconditionally, so in scatter
            % mode a mid-run Degrade turns drones red immediately and the bug is invisible. The
            % mesh and trail paths only assign colour when `recolour` is true, so with meshes on
            % -- the DEFAULT under ~100 drones, i.e. the usual case -- pressing Degrade would
            % have changed nothing on screen until the next transition happened to arrive, and
            % the button would read as broken. Comparing the cached mask costs one isequal on a
            % logical vector per frame, against a per-drone FaceColor write it still avoids on
            % every frame where neither changed.
            recolour = ~isequal(ftype, app.LastFormType) || ...
                       ~isequal(app.DenyMaskCache, app.LastDenyMask);

            if isempty(app.MeshPatches)
                set(app.DronesPlot, 'XData', pos(:,1), 'YData', pos(:,2), ...
                    'ZData', zUp, 'CData', colors);
            else
                % One rigid translation of the cached vertex block per drone —
                % the quadrotor never rotates in this show, so there is no
                % orientation to apply.
                for k = 1:app.NumUAVs
                    app.MeshPatches(k).Vertices = app.MeshV0 + ...
                        [pos(k,1), pos(k,2), zUp(k)];
                    if recolour
                        app.MeshPatches(k).FaceColor = colors(k,:);
                    end
                end
            end

            % Skipped entirely when the trails are off, rather than drawing into
            % hidden lines: this loop is one addpoints per drone per frame, which is
            % the second-largest per-frame cost after the meshes.
            %
            % ...and skipped while the scrub bar is being DRAGGED, which is not the same
            % thing. A drag is a series of jumps, so the points this would add are not a
            % path the fleet flew; seekFrame wipes them at the start of the drag for exactly
            % that reason, and adding them back per event only to wipe them again on the
            % next one was most of what made dragging expensive (79.9 ms a frame against
            % 6.9 ms at 10 drones). The release clears and normal playback grows them again.
            if ~isempty(app.TrailPlots) && ~app.Scrubbing
                for k = 1:app.NumUAVs
                    addpoints(app.TrailPlots(k), pos(k,1), pos(k,2), zUp(k));
                    if recolour
                        app.TrailPlots(k).Color = colors(k,:);
                    end
                end
            end
            % Move the rings onto the degraded drones. Unconditional, not gated on
            % `recolour`: the mask may not have changed but the drones have moved, and a
            % ring left at last frame's coordinates is worse than no ring at all -- it
            % marks whichever drone has since flown into that spot.
            if ~isempty(app.DenyRings) && isvalid(app.DenyRings)
                if ~isempty(app.DenyMaskCache) && ...
                        numel(app.DenyMaskCache) == size(pos, 1) && any(app.DenyMaskCache)
                    d = app.DenyMaskCache;
                    set(app.DenyRings, 'XData', pos(d,1), 'YData', pos(d,2), ...
                        'ZData', zUp(d));
                else
                    % NaN rather than [], because assigning empty XData to a scatter with
                    % non-empty YData is an inconsistent-size error rather than an empty plot.
                    set(app.DenyRings, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                end
            end

            % The planned-path overlay. Deliberately not gated on `recolour`: the phase it
            % keys on changes at the START of a transition, whereas the formation type only
            % changes when the fleet ARRIVES, so sharing that test would show every phase's
            % paths one phase late. It does its own comparison and returns on the frames
            % where nothing has changed, which is nearly all of them.
            if app.ShowTrajectory
                app.updateTrajOverlay(t);
            end

            app.LastFormType = ftype;
            app.LastDenyMask = app.DenyMaskCache;

            % Dynamic camera: track swarm centroid + zoom. One set() on the axes
            % rather than three xlim/ylim/zlim calls (1.75 ms -> 0.45 ms); the
            % view angle is fixed, so buildScenario sets it once.
            if strcmp(app.CameraMode, 'Dynamic')
                centroid = [mean(pos(:,1)), mean(pos(:,2)), mean(zUp)];
                spread = max(max(pos,[],1) - min(pos,[],1)) / 2 + 5;
                set(app.ScenarioAxes, ...
                    'XLim', [centroid(1)-spread, centroid(1)+spread], ...
                    'YLim', [centroid(2)-spread, centroid(2)+spread], ...
                    'ZLim', [-1, max(centroid(3)+spread, 5)]);
            end

            % Cached title handle: title() re-resolves the axes every call and
            % measured 2.59 ms against 0.20 ms for setting .String directly.
            if isempty(app.TitleText) || ~isvalid(app.TitleText)
                app.TitleText = get(app.ScenarioAxes, 'Title');
            end
            app.TitleText.String = sprintf('Drone Show%s — t = %.1f s', srcLabel, t);
            drawnow limitrate;
        end

        function cacheShowTimeline(app)
            % Snapshot the timeline setupParams just built. Called after every
            % setupParams run, because preShowDelay changes with the delivery
            % mode (5.7 s with a pre-flight, 2.0 s without).
            app.TimelineTimes = evalin('base', 'timeline_times');
            app.TimelineTypes = evalin('base', 'timeline_types');
            app.FormSeq       = evalin('base', 'formation_sequence');
            app.PreShowDelay  = evalin('base', 'preShowDelay');
            % Nothing caches takeoff_duration, deliberately. planShow raises it off its
            % floor on every show -- by climb_speed, and on a tall one by trackability --
            % and the two readouts that care both derive it instead: TimelineTimes(1) IS
            % the end of the climb, and the phase readout no longer has a pre-flight
            % window to size. A cached copy would be a second source of truth for a
            % number that is already in the timeline.
            % The descent the plan ends with, so planned playback can report Landing
            % and Landed at the times the trajectory actually does them.
            app.PlanLandStart = evalin('base', 'land_start_time');
            app.PlanLandEnd   = evalin('base', 'land_end_time');
        end

        function a = showAnchor(app)
            % Absolute sim time at which show-relative time 0 happens.
            %
            % OBSERVED in preference to predicted. The launch is gated on reception: ShowSupervisor
            % holds ARMED until the base station reports a fixed solution and the dwell expires, so
            % the instant depends on how the convergence went, not on a number computed before the
            % run. preShowDelay is only a prediction of it -- a good one on the fast path, where
            % there is no gate at all and the chart leaves IDLE on the GPS lock timer, but on the
            % full path it can be several seconds out on a slow draw. Subtracting the prediction
            % rather than the fact is what used to shift every formation label and every live
            % marker by the difference.
            %
            % Three sources, best first: the live latch while streaming, the logged phase trace once
            % a run has finished, and the prediction if neither exists yet (early frames of a live
            % run, before SHOW).
            if ~isempty(app.ShowEntryTime)
                a = app.ShowEntryTime;
                return;
            end
            if ~isempty(app.SimPhaseData)
                ph = round(squeeze(app.SimPhaseData.Data));
                k  = find(ph == 3, 1);
                if ~isempty(k)
                    % Cached, because formationAt is called per frame and this walks the trace.
                    app.ShowEntryTime = app.SimPhaseData.Time(k);
                    a = app.ShowEntryTime;
                    return;
                end
            end
            a = app.PreShowDelay;
        end

        function latchShowEntry(app, ph, simT, showT)
            % First frame in SHOW wins, and only the first: the chart re-enters nothing, but a
            % latch that kept updating would track LANDING back through phase 3 on a re-run and
            % move the axis mid-show.
            if ~isempty(app.ShowEntryTime) || round(ph) ~= 3
                return;
            end

            % The poll time is only accurate to the POLL INTERVAL, and during the pre-flight
            % that interval is PollMax -- the scene is static, so there is deliberately nothing
            % to poll hard for. Latching simT therefore reports the first poll AFTER the gate
            % opened, not the gate. Measured on two runs of the same seed and the same plan:
            % 16.06 s and 17.38 s, against a true entry of 16.00 s from the logged phase trace.
            % A 1.38 s error is not cosmetic -- the whole readout is keyed on this anchor, so
            % the panel went on claiming a formation hold while the fleet was already
            % descending.
            %
            % ShowTime removes the quantisation instead of throwing polls at it. The chart
            % starts that clock on SHOW ENTRY, so simT - ShowTime IS the entry instant however
            % late the poll lands, exact to the chart's own step. It is also the same signal
            % the onboard trajectory is clocked by, which is the point: the app's time axis and
            % the fleet's trajectory now share one clock rather than two estimates of it.
            app.ShowEntryTime = simT;
            if nargin >= 4 && isscalar(showT) && isfinite(showT) && showT > 0 && showT <= simT
                app.ShowEntryTime = simT - showT;
            end
        end

        function [formIdx, state] = formationAt(app, t)
            % Which formation is the fleet on, at playback/sim time t?
            %
            % Two conversions used to be missing, which is why the panel lagged
            % reality. First, simulated and live time is absolute: it includes
            % the pre-flight, so show-relative time is t - preShowDelay. Planned
            % playback is already show-relative. Second, the show opens with a
            % takeoff climb before the first formation -- 6.7 s on the default show
            % and longer on a tall one, since planShow sizes it -- which
            % timeline_times accounts for and the app's old arithmetic did not.
            % Read from the timeline for exactly that reason: nothing here needs to
            % know the number, only that segment 1 does not start at 0.
            %
            % formIdx indexes formation_sequence; state is 'Takeoff', 'Hold',
            % 'Transition', 'Landing' or 'Landed'.
            if app.UseSimData
                tShow = t - app.showAnchor();
            else
                tShow = t;
            end

            % The plan ends with a real descent and then parks, and TimelineTimes stops at
            % the START of the last hold -- so without this the whole landing falls through
            % to the last timeline segment and is reported as a HOLD, i.e. the readout
            % claims a formation is being held while the fleet is dropping 10 m, and the
            % run never reports Landed. advanceFrame's switch already has cases for both
            % states; nothing was producing them. Checked before the timeline lookup
            % because the landing window lies past the end of it.
            if ~isempty(app.PlanLandStart) && tShow >= app.PlanLandStart
                formIdx = max(1, numel(app.FormSeq));
                if ~isempty(app.PlanLandEnd) && tShow >= app.PlanLandEnd
                    state = 'Landed';
                else
                    state = 'Landing';
                end
                return;
            end

            tt = app.TimelineTimes;
            if isempty(tt)
                formIdx = 1; state = 'Hold';
                return;
            end
            seg = find(tShow >= tt, 1, 'last');
            if isempty(seg)
                % Still climbing to the first formation.
                formIdx = 1; state = 'Takeoff';
            elseif app.TimelineTypes(seg) == 0
                formIdx = (seg + 1) / 2;   % holds sit at odd segment indices
                state = 'Hold';
            else
                formIdx = seg / 2;         % transitions at even ones
                state = 'Transition';
            end
            formIdx = max(1, min(formIdx, numel(app.FormSeq)));
        end

        function ftype = formationTypeAt(app, t)
            % Colour and formation name key off the formation *type*, not its
            % position in the sequence: Grid->Circle->Grid is red, green, red.
            [formIdx, ~] = app.formationAt(t);
            if isempty(app.FormSeq)
                ftype = 1;
            else
                ftype = app.FormSeq(formIdx);
            end
        end

        function updateTelemetry(app, idx, pos, t)
            if ~isempty(app.LiveStopTime)
                % Live stream: SimTimeVector only reaches the current instant, so
                % it would report the run as always complete.
                totalTime = app.LiveStopTime;
            elseif app.UseSimData
                totalTime = app.SimTimeVector(end);
            else
                totalTime = app.TimeVector(end);
            end
            app.TimeLabel.Text = sprintf('Time: %.1f / %.1f s', t, totalTime);

            % Min separation
            minSep = inf;
            for i = 1:app.NumUAVs
                for j = i+1:app.NumUAVs
                    d = norm(pos(i,:) - pos(j,:));
                    if d < minSep, minSep = d; end
                end
            end
            dmin = app.DminField.Value;
            if minSep < dmin
                app.MinSepLabel.Text = sprintf('Min Separation: %.2f m  ⚠ VIOLATION', minSep);
                app.MinSepLabel.FontColor = [0.9 0 0];
            else
                app.MinSepLabel.Text = sprintf('Min Separation: %.2f m', minSep);
                app.MinSepLabel.FontColor = [0 0.6 0];
            end

            % Max speed
            if idx > 1
                if app.UseSimData
                    dt = app.SimTimeVector(idx) - app.SimTimeVector(max(1,idx-1));
                    prevPos = squeeze(app.SimTrajectoryData(:, 1:3, max(1,idx-1)));
                else
                    dt = app.TimeVector(idx) - app.TimeVector(max(1,idx-1));
                    prevPos = squeeze(app.TrajectoryData(:, 1:3, max(1,idx-1)));
                end
                if dt > 0
                    speeds = vecnorm(pos - prevPos, 2, 2) / dt;
                    maxSpd = max(speeds);
                    app.MaxErrLabel.Text = sprintf('Max Speed: %.2f m/s', maxSpd);
                end
            end

            % Phase indicator and upload progress
            % ShowSupervisor emits: 0=Idle, 1=Upload, 2=Armed, 3=Show,
            % 4=Landed, 5=Landing, 6=UploadFailed.
            %
            % Phase 5 covers both descents — the planned one at the end of the show and
            % an operator abort — because to the fleet they are the same thing: coming
            % down. Which one it is shows in the status bar, not here.
            phaseNames = {'Idle','Upload','Armed','Show','Landed','Landing','Upload FAILED'};
            % Kept so the formation readout below can defer to it. The formation
            % timeline stops at the last hold and knows nothing about the descent.
            phaseNow = '';
            if ~isempty(app.LivePhase)
                pIdx = max(1, min(numel(phaseNames), round(app.LivePhase) + 1));
                phaseNow = phaseNames{pIdx};
                app.PhaseLabel.Text = sprintf('Phase: %s (live)', phaseNow);
            elseif app.UseSimData && ~isempty(app.SimPhaseData)
                try
                    pv = interp1(app.SimPhaseData.Time, double(app.SimPhaseData.Data), t, ...
                        'previous', 'extrap');
                    pIdx = max(1, min(numel(phaseNames), round(pv) + 1));
                    phaseNow = phaseNames{pIdx};
                    app.PhaseLabel.Text = sprintf('Phase: %s', phaseNow);
                catch
                    app.PhaseLabel.Text = 'Phase: --';
                end
            else
                % Planned playback: infer phase from time relative to show timeline.
                % The landing window comes from setupParams rather than being guessed
                % at "the last second of the plan" -- the plan now ends with a real
                % descent that takes land_duration and then parks, and calling all of
                % that 'Landed' had the readout claim the fleet was down while it was
                % still 10 m up and dropping.
                %
                % There is no Idle or Armed window here, and there used to be: the
                % first half of a hard-coded 5 s read 'Idle' and the second half
                % 'Armed'. Both were wrong in kind rather than just in the number.
                % Planned playback's t = 0 IS show-relative 0 -- the instant the chart
                % enters SHOW -- so the fleet is climbing from the very first frame,
                % and the readout announced 'Idle' over drones visibly leaving the
                % pads. The pre-flight simply is not in this time base to report.
                % Deleting the window rather than resizing it also drops the last
                % place the climb length was written down as a constant, which
                % matters now that climb_speed sets it: it is 6.7 s on the default
                % show and 31.8 s on a text billboard.
                %
                % Simulated playback reads phase 3 with state 'Takeoff' through the
                % climb, so this now agrees with it instead of contradicting it.
                showDur = app.TimeVector(end);
                landFrom = showDur;
                landTo = showDur;
                if ~isempty(app.PlanLandStart)
                    landFrom = app.PlanLandStart;
                    landTo = app.PlanLandEnd;
                end
                if t < landFrom
                    plannedPhase = 'Show';
                elseif t < landTo
                    plannedPhase = 'Landing';
                else
                    plannedPhase = 'Landed';
                end
                phaseNow = plannedPhase;
                app.PhaseLabel.Text = sprintf('Phase: %s (planned)', plannedPhase);
            end

            % Upload bar — replay the logged arrival count as playback advances
            if app.UseSimData && ~isempty(app.SimUploadCount) && ...
                    ~isempty(app.UploadTotal) && app.UploadTotal > 0
                try
                    uc = interp1(app.SimUploadCount.Time, double(app.SimUploadCount.Data), t, ...
                        'previous', 'extrap');
                    uc = max(0, round(uc));
                    tot = app.UploadTotal;
                    pct = 100 * min(1, uc/tot);
                    if uc >= tot, barState = 'ok'; else, barState = 'busy'; end
                    app.UploadLabel.Text = sprintf('%d / %d items confirmed onboard', uc, tot);
                    app.setUploadBar(pct, barState);
                catch
                    app.UploadLabel.Text = 'Upload progress unavailable.';
                end
            end

            % Formation phase — same mapping the viewer colours off, so the label
            % and the LED colour can never disagree.
            formNames = app.formationNames();
            [formIdx, segState] = app.formationAt(t);
            if isempty(app.FormSeq)
                app.FormLabel.Text = 'Formation: --';
                app.StateLabel.Text = 'Show State: Idle';
                return;
            end
            fIdx = min(app.FormSeq(formIdx), numel(formNames));
            % timeline_times stops at the START of the last hold, so every instant
            % after it -- the descent and the parked settle included -- lands in that
            % final Hold segment, and the readout used to sit on "(hold)" while the
            % fleet was visibly coming down. An abort is worse: it lands early, so the
            % formation clock has no idea a landing is happening at all. The phase
            % does, in all three modes, so it wins. The formation itself is still the
            % last one -- the fleet lands on the pattern it finished on, which is also
            % why the LEDs keep its colour.
            if any(strcmp(phaseNow, {'Landing', 'Landed'}))
                segState = phaseNow;
            end
            switch segState
                case 'Takeoff'
                    app.FormLabel.Text = sprintf('Formation: %s (climbing)', formNames{fIdx});
                    app.StateLabel.Text = 'Show State: Takeoff';
                case 'Transition'
                    % Name where it is going, not just that it is moving.
                    nextIdx = min(formIdx + 1, numel(app.FormSeq));
                    nIdx = min(app.FormSeq(nextIdx), numel(formNames));
                    app.FormLabel.Text = sprintf('Formation: %s → %s', ...
                        formNames{fIdx}, formNames{nIdx});
                    app.StateLabel.Text = 'Show State: Transition';
                case 'Landing'
                    app.FormLabel.Text = sprintf('Formation: %s (landing)', formNames{fIdx});
                    app.StateLabel.Text = 'Show State: Landing';
                case 'Landed'
                    app.FormLabel.Text = sprintf('Formation: %s (landed)', formNames{fIdx});
                    app.StateLabel.Text = 'Show State: Landed';
                otherwise
                    app.FormLabel.Text = sprintf('Formation: %s (hold)', formNames{fIdx});
                    app.StateLabel.Text = 'Show State: Hold';
            end
        end

        %% ---- Pre-Flight MAVLink Upload ----
        function uploadTrajectory(app)
            % Stream the mission over MAVLink and then fly it, in ONE continuous
            % run, with the viewer updating as the simulation advances.
            %
            % One run is not a convenience. With traj_source = 2 the drones fly
            % what is in the OnboardBuffer data store, and data store contents do
            % not survive across sim() calls — so the pre-flight upload and the
            % show have to share a simulation. That also means the pre-flight is
            % no longer paid for twice, which is what the old Upload-then-Simulate
            % pair did.
            %
            % The upload stays load-bearing: at high packet loss the gap-repair
            % phase gives up, the supervisor latches UPLOAD_FAILED, the show never
            % starts, and the operator retries.
            if isempty(app.TrajectoryData)
                app.updateStatus('Generate a show first.');
                return;
            end
            if app.trajSourceValue() ~= 2
                app.updateStatus(['Workspace mode feeds the controller directly — ' ...
                    'there is nothing to upload. Click Simulate.']);
                return;
            end

            modelName = 'MultiUAV_DroneShow';
            app.UploadAttempt = app.UploadAttempt + 1;
            attempt = app.UploadAttempt;

            % A retry must see a different loss pattern, otherwise it fails
            % identically. Deterministic per attempt so runs stay reproducible.
            seed = 12345 + 977 * attempt;
            assignin('base', 'loss_seed', seed);
            assignin('base', 'upload_request', true);

            % Before the buttons are locked down: stopPlayback re-enables Generate.
            app.stopPlayback();

            app.UploadOK = false;
            app.UploadBarHistory = [];
            app.UploadLamp.Color = [0.85 0.75 0.2];
            app.setUploadBar(0, 'busy');
            app.UploadEstimate = app.estimateUploadSeconds();
            app.UploadLabel.Text = app.uploadProgressText(attempt, 0, 0, 0, 0, 0);
            app.UploadBtn.Enable = 'off';
            app.SimBtn.Enable = 'off';
            app.GenerateBtn.Enable = 'off';
            app.StopBtn.Enable = 'on';
            % Live for the whole mission, not just the show: an abort out of UPLOADING
            % is legal and is the honest answer to a stream that is going nowhere.
            app.AbortBtn.Enable = 'on';
            % Say up front how long this will take, and that Stop is the way out. The
            % upload is the one part of a show whose duration is knowable in advance and
            % is long enough to be mistaken for a hang -- so quoting it here is the
            % difference between waiting and force-quitting.
            if isfinite(app.UploadEstimate)
                app.updateStatus(sprintf(['Attempt %d: uploading the mission over ' ...
                    'MAVLink. This streams one MISSION_ITEM_INT per waypoint per ' ...
                    'drone and takes about %.0f s of sim time for %d drones (longer ' ...
                    'in wall clock), after a GPS-lock wait first. Stop or Abort & ' ...
                    'Land both work throughout.'], ...
                    attempt, app.UploadEstimate, app.NumUAVSpinner.Value));
            else
                app.updateStatus(sprintf(['Attempt %d: uploading trajectory over ' ...
                    'MAVLink. Stop or Abort & Land both work throughout.'], attempt));
            end

            % Streamed frames land in the same slots a logged run fills, so
            % Play [Sim] afterwards replays this run like any other.
            app.UseSimData = true;
            app.SimTrajectoryData = [];
            app.SimTimeVector = [];
            app.SimPhaseData = [];
            app.ShowEntryTime = [];   % cleared with the phase log it is derived from
            app.SimUploadCount = [];
            app.buildScenario();
            drawnow;

            try
                app.prepareModel(modelName);

                % The whole mission in one run: GPS lock, item stream, arm, show,
                % land. setupParams already sized total_sim_duration for exactly
                % this, including margin for gap-repair retransmissions.
                stopT = evalin('base', 'total_sim_duration');
                total = evalin('base', 'N_uav') * evalin('base', 'num_waypoints');
                app.UploadTotal = total;
                app.LiveStopTime = stopT;

                % Normal OR Accelerator: both keep the live reads this path needs. The upload bar
                % reads ArrCount and the viewer reads TransposePos/ShowSupervisor, and all of them
                % were measured live in Accelerator -- ArrCount specifically, with the upload
                % running. The older claim here, that RunTimeBlock "exists only in
                % Normal", was simply wrong; it is only RAPID
                % accelerator that runs out-of-process and goes blind.
                %
                % This is ONE run for the whole mission -- lock, item stream, arm, show, land -- so
                % the mode applies to all of it. There is no "interpreted upload then accelerated
                % show": SimulationMode is per-run, while SimulateUsing is per-block and unchanged
                % by it, so the 22 interpreted MAVLink blocks stay interpreted here exactly as they
                % do in Normal.
                %
                % No separate speedup estimate for this path: the ~18 % was measured on a run with
                % upload_request = true over the full 38.5 s mission, so the upload window was
                % already inside it. MAVLink cost is not confined to that window anyway -- HB and
                % ST are serialised at ~35 Hz and ~60 Hz for the entire run, which is what the
                % 1 Hz work targets and what a mode change cannot touch.
                simMode = app.simModeString();
                if strcmp(simMode, 'rapid')
                    simMode = 'normal';   % Rapid cannot drive the upload bar at all
                end
                set_param(modelName, 'SimulationMode', simMode);
                set_param(modelName, 'StopTime', num2str(stopT));

                r = app.runUploadPolled(modelName, total, attempt);
                app.finishLiveRun();

                pct = 100 * r.delivered / total;
                if r.aborted
                    app.UploadLamp.Color = [0.55 0.55 0.55];
                    app.setUploadBar(pct, 'busy');
                    app.UploadLabel.Text = sprintf( ...
                        'Run stopped by operator at %d / %d items (%.1f%%).', ...
                        r.delivered, total, pct);
                    app.UploadBtn.Text = 'Upload & Fly';
                    app.updateStatus(sprintf(['ABORTED on attempt %d — the fleet has ' ...
                        'no usable mission. Click Upload & Fly to start over.'], attempt));
                elseif r.armed && ~r.failed
                    app.UploadOK = true;
                    app.UploadLamp.Color = [0.15 0.7 0.25];
                    app.setUploadBar(100, 'ok');
                    app.UploadBtn.Text = 'Re-upload & Fly';
                    if r.flew
                        app.UploadLabel.Text = sprintf( ...
                            '%d / %d items confirmed onboard — show flown from the buffer', ...
                            total, total);
                        % Frame counts differ only when the replay log was upgraded to
                        % the dense one, i.e. Pose shown = True airframe pose. On the
                        % as-received tap nothing logs the table, so both are the poll
                        % count and the sentence collapses to one number.
                        app.updateStatus(sprintf(['Upload OK on attempt %d (seed %d, ' ...
                            '%d items retransmitted by per-item gap repair) and the ' ...
                            'show flew from the onboard buffer — %d frames drawn live, ' ...
                            '%d replayable. Click Play [Sim] to replay.'], ...
                            attempt, seed, max(0, r.count - total), ...
                            app.LiveStreamedFrames, numel(app.SimTimeVector)));
                    else
                        % Armed, but the run ended before SHOW. Nothing in the model
                        % does that on its own, so say so rather than imply a flight.
                        app.UploadLabel.Text = sprintf( ...
                            '%d / %d items confirmed onboard — ARMED at %.1f s, show did not start', ...
                            total, total, r.tArm);
                        app.updateStatus(sprintf(['Upload OK on attempt %d but the run ' ...
                            'ended at t = %.1f s before the show started.'], ...
                            attempt, r.simT));
                    end
                else
                    app.UploadLamp.Color = [0.85 0.15 0.15];
                    app.setUploadBar(pct, 'fail');
                    app.UploadLabel.Text = sprintf( ...
                        'FAILED: only %d / %d items confirmed (%.1f%%)', ...
                        r.delivered, total, pct);
                    app.UploadBtn.Text = 'Retry Upload';
                    if r.failed
                        why = ['a drone stopped answering MISSION_REQUEST_INT, so ' ...
                               'per-item gap repair could not converge'];
                    else
                        why = 'the stream did not finish inside the pre-flight window';
                    end
                    app.updateStatus(sprintf(['UPLOAD FAILED on attempt %d — %s. ' ...
                        'The show was never started. Reduce Packet Loss (%%) or ' ...
                        'click Retry Upload.'], attempt, why));
                end

                % The branches above describe the upload, which succeeded or failed on
                % its own terms; an abort mid-flight is a separate fact about the show
                % and would otherwise go unmentioned in a run reported as flown.
                if app.AbortRequested
                    app.updateStatus([app.StatusBar.Text ' NOTE: the mission was ' ...
                        'ABORTED mid-flight and the fleet was landed on command.']);
                end

                app.UploadBtn.Enable = 'on';
                app.GenerateBtn.Enable = 'on';
                app.StopBtn.Enable = 'off';
            catch e
                app.SimRunning = false;
                app.finishLiveRun();
                app.UploadLamp.Color = [0.85 0.15 0.15];
                app.setUploadBar(0, 'fail');
                app.UploadLabel.Text = 'Run aborted.';
                app.UploadBtn.Enable = 'on';
                app.UploadBtn.Text = 'Retry Upload';
                app.GenerateBtn.Enable = 'on';
                app.StopBtn.Enable = 'off';
                app.updateStatus(['Upload ERROR: ' e.message]);
            end
        end

        function finishLiveRun(app)
            % Turn the streamed frames into a replayable run and leave live mode.
            %
            % Every live path comes through here, success or error, which is why the
            % abort is untied here: nothing is flying any more, so nothing should be
            % holding an abort request for the next run to inherit. AbortRequested
            % itself survives, so the closing message can still say what happened.
            app.clearAbort('MultiUAV_DroneShow');
            app.AbortBtn.Enable = 'off';
            app.LivePhase = [];
            app.LiveStopTime = [];
            n = numel(app.SimTimeVector);
            app.LiveStreamedFrames = n;
            app.NumSamples = n;
            app.PlayIdx = 1;
            if n == 0
                app.UseSimData = false;
                return;
            end
            % The phase and arrival-count traces were sampled per frame, so the
            % telemetry panel and upload bar replay exactly what was shown live.
            % Guarded separately: the Workspace path streams a phase trace but has
            % no upload at all, and a zero-filled count log would replay as a bar
            % stuck at 0 % rather than as "no upload in this mode".
            if ~isempty(app.LivePhaseLog)
                app.SimPhaseData = timeseries(app.LivePhaseLog(:), app.SimTimeVector(:));
            end
            if ~isempty(app.LiveCountLog)
                app.SimUploadCount = timeseries(app.LiveCountLog(:), app.SimTimeVector(:));
            end
            app.LivePhaseLog = [];
            app.LiveCountLog = [];
            % Replay off the DENSE log, not the poll log. The poll log is one frame per
            % governor cycle -- ~3 Hz, because the cycle is ~315 ms at 1x pacing -- and
            % that is a property of how often the viewer looked, not of the run: the
            % solver stepped at Ts_sim throughout and LogFleetPos recorded every step.
            % Replaying the poll log meant a finished run was as steppy as the live view
            % had been, on BOTH delivery paths and in both Sim Modes that can stream, so
            % "fly it once then Play it back smoothly" did not actually work anywhere
            % except Rapid Accelerator -- the one mode with no live view at all.
            app.upgradeReplayLog();
            app.NumSamples = numel(app.SimTimeVector);
            app.PlayBtn.Text = 'Play [Sim]';
            app.PlayBtn.Enable = 'on';
            % The dense log is a different length from the poll log the live view drew, so
            % the bar has to be re-limited before it can be scrubbed.
            app.PlayIdx = 1;
            app.refreshScrub();
            % NO refreshTrajOverlay here, deliberately, and NOT an oversight copied from
            % the non-streaming path (see runSimulation, which does call it). The two ends
            % differ in what is left ON SCREEN. runSimulation calls buildScenario, which
            % cla's the axes back to the pad, so the drawn scene IS frame 1 and the overlay
            % has to be redrawn to match it. A streamed run ends with the LAST FLOWN FRAME
            % still painted -- landed fleet, full trails -- and only the scrub thumb goes
            % back to 1. The overlay tracks the frame that is drawn, not the thumb, so
            % blank (there is no move after touchdown) is the honest state: drawing the
            % climb paths here would put the start of the show over the end of it. Play or
            % any scrub redraws it from drawFrame on the next frame.
        end

        function ok = upgradeReplayLog(app)
            % Swap the ~3 Hz poll log for the Ts_sim log the run already produced.
            %
            % WHERE THE DENSE DATA COMES FROM. LogFleetPos is a To Workspace on
            % DroneFleet outport 1 (FleetPositions, true pose) at an inherited sample
            % time, so Ts_sim. An interactive run started with SimulationCommand still
            % publishes `out` (a Simulink.SimulationOutput) to the base workspace, which
            % is what makes this cheap -- verified on a 3 s run: out.fleetPositions is
            % [6 3 301], 100.0 Hz.
            %
            % ONLY FOR TRUE-POSE RUNS. See LiveLogIsTruePose: the as-received tap
            % (BaseStation/Receiver/SwTelTable) is not logged by anything, so on that
            % path the poll snapshots are the only record that exists and the run keeps
            % them. Substituting true pose there would quietly replace the received
            % telemetry with ground truth and erase the staleness wave the mode exists
            % to show.
            ok = false;
            if ~app.LiveLogIsTruePose || isempty(app.SimTimeVector)
                return;
            end
            try
                fp = evalin('base', 'out.fleetPositions');
                tD = fp.Time(:);
                pD = fp.Data;                       % [N x 3 x T]
            catch
                return;                             % no dense log; poll log stands
            end
            if numel(tD) < 2 || size(pD, 1) ~= app.NumUAVs || size(pD, 3) ~= numel(tD)
                return;
            end
            % Never past where the stream actually got to. A run the operator stopped
            % leaves a log that ends at the stop, but an abort leaves one that runs on
            % to the landing, and replaying further than the live view reached would
            % show frames that were never on screen.
            keep = tD <= app.SimTimeVector(end) + eps(app.SimTimeVector(end));
            if nnz(keep) < 2
                return;
            end
            tD = tD(keep);
            pD = pD(:, 1:3, keep);

            % Same 20 Hz target the non-streamed path uses, so all three replay sources
            % now land on one frame rate.
            dsRatio = max(1, round(0.05 / mean(diff(tD))));
            tD = tD(1:dsRatio:end);
            pD = pD(:, :, 1:dsRatio:end);

            % Reproduce the pad hold. poseForView parks the fleet on its pads for every
            % frame before phase 3, and it did so on the way into the poll log, so the
            % dense log has to be parked the same way or the replay would show the
            % pre-show station-keeping wander the live view deliberately hid.
            if ~isempty(app.PadPose) && size(app.PadPose, 1) == app.NumUAVs
                phD = app.densePhaseAt(tD);
                if ~isempty(phD)
                    park = phD < 3;
                    if any(park)
                        pD(:, :, park) = repmat(app.PadPose(:, 1:3), 1, 1, nnz(park));
                    end
                end
            end

            app.SimTimeVector = tD(:);
            app.SimTrajectoryData = pD;
            ok = true;
        end

        function ph = densePhaseAt(~, tQuery)
            % ShowSupervisor's phase at each dense sample, for the pad hold above.
            % Nearest-neighbour rather than linear: phase is an enumeration, and
            % interpolating it would invent values between 2 and 3.
            ph = [];
            try
                lp = evalin('base', 'out.logPhase');
                tP = lp.Time(:);
                vP = double(lp.Data(:));
            catch
                return;
            end
            if numel(tP) < 2 || numel(vP) ~= numel(tP)
                return;
            end
            ph = interp1(tP, vP, tQuery(:), 'previous', 'extrap');
        end

        function seedRxPose(app)
            % Prime the received-pose hold with where each drone is standing, so the
            % first frames of a full-fidelity run show the fleet on the pad instead of
            % waiting for the first telemetry slot. Measured: the whole table is
            % all-zero on poll 1 and the first row arrives at t ~ 1.7-2.0 s, which is a
            % second of blank viewer otherwise.
            app.RxPoseHold = [];
            try
                app.RxPoseHold = evalin('base', 'init_positions');
            catch
                % No workspace yet. The planned trajectory's first sample IS
                % init_positions (setupParams writes it there), so it is an exact
                % substitute rather than an approximation.
                if ~isempty(app.TrajectoryData)
                    app.RxPoseHold = squeeze(app.TrajectoryData(:, 1:3, 1));
                end
            end
            if ~isempty(app.RxPoseHold) && size(app.RxPoseHold, 1) ~= app.NumUAVs
                app.RxPoseHold = [];   % stale fleet size; better to hold nothing
            end
            % Kept as its own copy, because RxPoseHold is overwritten row by row as
            % telemetry arrives and poseForView needs the pads to still be the pads.
            app.PadPose = app.RxPoseHold;
            app.PadHoldOffset = 0;
        end

        function pos = rxPose(app, raw)
            % Resolve one poll of the decoded telemetry table into a [N x 3] NED pose.
            % Returns [] when nothing is yet known about any drone, which is the
            % caller's signal to skip the frame.
            %
            % reshape rather than raw(:, 1:3): at N_uav = 1 a 2-D signal can present as
            % a bare 6-element vector, and a direct slice would then take x, y and z
            % from the wrong elements. Column-major reshape recovers [1 x 6] from
            % either presentation. Confirmed [N x 6] at N = 5 and at N = 1.
            tbl = reshape(double(raw), [], 6);
            p   = tbl(:, 1:3);

            % A row is "heard from" if any of its six fields is non-zero. The only pose
            % that aliases the never-received state is a drone at exactly (0,0,0) with
            % exactly zero velocity -- and the fallback for such a row is the pad, which
            % is where that drone is, so the aliasing is harmless by construction.
            heard = any(tbl ~= 0, 2);

            if isempty(app.RxPoseHold) || size(app.RxPoseHold, 1) ~= size(p, 1)
                if ~any(heard)
                    pos = [];
                    return;
                end
                % Unseeded fallback: this branch is not reachable from Upload & Fly,
                % which requires a generated show, so the unheard rows are left at the
                % origin rather than invented.
                app.RxPoseHold = zeros(size(p));
            end
            app.RxPoseHold(heard, :) = p(heard, :);
            pos = app.RxPoseHold;
        end

        function r = runUploadPolled(app, modelName, total, attempt)
            % Run the whole mission with a non-blocking SimulationCommand so the
            % progress bar can fill while the waypoints are on the wire, and the
            % viewer can show the show as it is flown.
            %
            % Everything reported here comes from live reads, not logged output: a
            % non-blocking run leaves only SimulationMetadata in the base workspace
            % 'out', so logPhase / logUploadCount are not available. That is fine —
            % Phase 2 lasts arm_duration and Phase 6 latches, so neither can slip
            % between polls, and the streamed frames replace the position log.
            phaseBlk = [modelName '/ShowSupervisor'];        % output 1 = Phase
            cntBlk   = [modelName '/DroneFleet/MissionStatus/ArrCount'];   % out 1 = arrivals
            sigBlk   = [modelName '/RadioChannel/CorrectionAgeMonitor/InjectGate'];  % out 1 = [N x 1] sigma

            % THE POSE THE GROUND STATION ACTUALLY HAS. Full-fidelity mode exists to run
            % the whole comms chain, so the viewer shows the far end of it: the decoded
            % MAVLink telemetry table, [N x 6] = [x y z vx vy vz], columns 1:3 being
            % position. It is not the true airframe state -- the payload is packed from
            % Transmitter/MeasPos, which comes off DroneFleet/Navigation, i.e. each
            % drone's own estimate, and it then crosses the radio and the round-robin
            % scheduler before arriving here.
            %
            % So the difference between this and the truth is the navigation error, and
            % showing it is the point rather than a cost: measured 4.79 m during the
            % pre-flight while the RTK engine is still code-only, collapsing 34x to
            % 0.14 m once it fixes. The quick path deliberately keeps the direct tap --
            % see runShowPolled.
            %
            % WHY THIS IS A CHOICE NOW. The downlink is a SELF-SCHEDULED broadcast, one
            % drone per tick: TelemetryEncoder's free-running TelCounter hands drone k the
            % slot on tick k and stamps k as the message's own SystemID, and the base
            % station simply files whatever arrives by that ID -- it never asks. So
            % BaseStation/Receiver holds a table whose rows are each as fresh as that
            % drone's last slot and no fresher. The whole table therefore refreshes once
            % every N_uav * Ts_sim -- 0.2 s at 20 drones, 1 s at 100, 5 s at 500 -- and
            % above a few dozen drones that staleness is plainly visible as a wave
            % crossing the fleet. It is the modelled radio rather than a viewer artefact,
            % and at 100 drones a 1 Hz per-drone report is about what a real telemetry
            % link gives. But it is not what "where is the fleet" means, so the operator
            % can ask for the airframe instead and get every drone every tick.
            useDownlink = strcmp(app.PoseView, 'downlink');
            if useDownlink
                posBlk = [modelName '/BaseStation/Receiver/SwTelTable'];   % [N x 6] received
            else
                posBlk = [modelName '/DroneFleet/FlightDynamics/TransposePos']; % [N x 3] true
            end
            % Whether finishLiveRun may swap the poll log for the dense one. Only the
            % true tap has a dense equivalent: LogFleetPos records DroneFleet outport 1
            % (FleetPositions, true pose) at Ts_sim, and NOTHING logs SwTelTable -- there
            % is no To Workspace anywhere in BaseStation and logsout carries only the
            % four dbg_/ss_ signals. So an as-received run keeps its poll log.
            app.LiveLogIsTruePose = ~useDownlink;

            % End of the bulk stream; past this the indexer is repairing gaps.
            tBulkEnd = evalin('base', 'gps_lock_duration') + ...
                       evalin('base', 'upload_duration');

            r = struct('armed', false, 'failed', false, 'aborted', false, ...
                       'flew', false, 'count', 0, 'delivered', 0, ...
                       'tArm', NaN, 'simT', 0);
            rtoP = []; rtoC = []; rtoX = []; rtoS = [];
            lastPhase = -1; lastTextT = -inf;
            app.LivePhaseLog = [];
            app.LiveCountLog = [];
            app.armRtkTier();
            app.armCommStatus(modelName);
            app.armPollRate();
            app.seedRxPose();
            prevSimT = 0; lastPause = 0; lastHold = 0;
            tGov = []; simT0 = 0;   % window the rate summary covers; opens when the show does

            app.SimRunning = true;
            % Greys the scrub bar out for the duration. seekFrame refuses during a live
            % run anyway -- the frames ahead of "now" do not exist yet -- but a control
            % that looks usable and then refuses is worse than one that is visibly out.
            app.refreshScrub();
            % Drop any previous run's `out` before starting. upgradeReplayLog reads
            % out.fleetPositions afterwards, and a run that dies before publishing one
            % would otherwise let it splice the PREVIOUS run's flight into this run's
            % replay -- the fleet-size and end-time guards there would not catch it,
            % because a re-run at the same fleet size passes both.
            evalin('base', 'clear out');
            % Armed but NOT engaged: pacing is a throttle, the upload is the part of this
            % run that must not be throttled, and the same reasoning that lets the poll
            % interval sit at its ceiling through the pre-flight applies to the pacing
            % rate. It is engaged below, once the drones are moving.
            app.pacingArm(modelName);
            set_param(modelName, 'SimulationCommand', 'start');
            while true
                tBody = tic;   % everything up to the pause is time the solver is idle
                % stopAll clears SimRunning, which is how the Stop button reaches
                % us. Test it before SimulationStatus: stopAll also issues the
                % stop command, so by now the status may already read 'stopped'
                % and would otherwise look like an ordinary end of run.
                if ~app.SimRunning
                    r.aborted = true;
                    break;
                end
                st = get_param(modelName, 'SimulationStatus');
                if ~any(strcmp(st, {'running', 'initializing', 'paused'}))
                    break;
                end

                r.simT = get_param(modelName, 'SimulationTime');
                % Fetch the runtime objects once; reading .Data off a cached
                % handle is much cheaper than get_param each poll.
                if isempty(rtoP)
                    rtoP = get_param(phaseBlk, 'RuntimeObject');
                    rtoC = get_param(cntBlk, 'RuntimeObject');
                    rtoX = get_param(posBlk, 'RuntimeObject');
                    % Not in the guard below: the tier readout is a nicety and must
                    % never be the reason a frame is skipped.
                    rtoS = get_param(sigBlk, 'RuntimeObject');
                    % Same reason, same place: the radio readout's five taps cannot be
                    % dereferenced in armCommStatus because the model was not running yet.
                    app.fetchCommHandles();
                end
                if ~isempty(rtoP) && ~isempty(rtoC) && ~isempty(rtoX)
                    ph = double(rtoP.OutputPort(1).Data);
                    r.count = double(rtoC.OutputPort(1).Data);
                    r.delivered = min(r.count, total);
                    if useDownlink
                        pos = app.rxPose(rtoX.OutputPort(1).Data); % [N x 3] NED, as received
                    else
                        % Direct tap, same as the quick path. The zero-buffer hazard the
                        % seeded hold covers for the downlink is covered here the way
                        % runShowPolled covers it: an all-zero first read is the port
                        % before it has been written, not a fleet at the origin.
                        pos = double(rtoX.OutputPort(1).Data);
                        if isempty(app.SimTimeVector) && all(pos(:) == 0)
                            pos = [];
                        end
                    end

                    % UPLOAD_FAILED is 6, so it is numerically above SHOW without being
                    % anywhere near it -- excluded explicitly rather than relying on the
                    % ~r.failed guard the caller happens to apply.
                    if ph >= 3 && ph ~= 6, r.flew = true; end
                    if ph == 6, r.failed = true; end
                    % Latched from ANY phase that can only be reached THROUGH arming, not
                    % just from catching ph == 2 in the act. On this path the supervisor's
                    % only route to SHOW, LANDING or LANDED runs through ARMED -- the one
                    % edge that skips it, IDLE -> SHOW on SkipPreflight, is the Workspace
                    % (quick) delivery and never reaches this loop. ARMED itself lasts only
                    % arm_duration, which a poll can step straight over: when it did, a show
                    % that uploaded, armed, flew and landed perfectly reported 'FAILED: only
                    % N / M items confirmed', because every branch of the report keys off
                    % this flag. Sampling a transient state was the bug; the later phases are
                    % evidence of it that cannot be missed.
                    if ~r.armed && ph >= 2 && ph ~= 6
                        r.armed = true;
                        if ph == 2
                            % Caught in the act, so this is the real arming time. Inferred
                            % from a later phase it would not be, and tArm is quoted to the
                            % operator -- left NaN rather than quoted wrongly. Only the
                            % 'armed but the show never started' branch reads it, and that
                            % branch is unreachable without having seen ph == 2.
                            r.tArm = r.simT;
                        end
                    end

                    % Stream this instant into the viewer and keep it, so the run
                    % can be replayed afterwards without a second simulation.
                    %
                    % rxPose returns [] only when nothing is known about any drone --
                    % no telemetry has arrived AND there was no pad to seed the hold
                    % from. That covers the same hazard the direct tap had (poll 1 can
                    % land after SimulationStatus reads 'running' but before the ports
                    % have been written once, giving an all-zero buffer that would stack
                    % the whole fleet on the origin and report min separation as 0) and
                    % it covers it more narrowly: with a seed the frame is meaningful
                    % from t = 0, because a drone that has not reported yet is on its
                    % pad, which is exactly what the seed says.
                    if isempty(pos)
                        drawnow;
                        tP = tic; pause(0.05); lastPause = toc(tP);
                        prevSimT = r.simT;
                        continue;
                    end

                    % Park the fleet on its pads until the show starts. Applied AFTER the
                    % isempty guard above, so the seeded-hold logic still decides whether
                    % there is a frame at all, and BEFORE the frame is stored, so a replay
                    % of this run shows exactly what was on screen during it.
                    pos = app.poseForView(pos, ph);

                    % Indexed explicitly rather than with end+1: on the first frame
                    % SimTrajectoryData is [], whose size along dim 3 already reads
                    % as 1, so end+1 would leave an empty leading slice behind.
                    k = numel(app.SimTimeVector) + 1;
                    app.SimTimeVector(k, 1) = r.simT;
                    app.SimTrajectoryData(:, :, k) = pos;
                    app.LivePhaseLog(k, 1) = ph;
                    app.LiveCountLog(k, 1) = r.delivered;
                    app.LivePhase = ph;
                    % Outport 2 is show_time. Read only until the anchor is latched, so the
                    % steady-state poll cost is unchanged.
                    if isempty(app.ShowEntryTime)
                        app.latchShowEntry(ph, r.simT, double(rtoP.OutputPort(2).Data));
                    end
                    % Named, because the two modes now show different quantities and the
                    % pre-flight wander is only explicable if you know you are watching
                    % the downlink rather than the airframe. The pre-show hold gets its own
                    % suffix for the same reason: the title is the only place a still fleet
                    % is distinguishable from a fleet whose pose is simply not being drawn.
                    if ph < 3 && ~isempty(app.PadPose)
                        srcLbl = ' [Live - pre-show, fleet on pads]';
                    elseif useDownlink
                        srcLbl = ' [Live - MAVLink downlink]';
                    else
                        srcLbl = ' [Live - true airframe pose]';
                    end
                    app.drawFrame(pos, r.simT, srcLbl);

                    % Re-test the abort HERE, and not only at the top of the loop.
                    % drawFrame ends in a drawnow, which is where the Stop button's
                    % callback runs -- so the operator can stop the model halfway
                    % through this iteration, and everything below is then reading
                    % handles into a terminated model. updateRtkTier dereferences
                    % rtoS.OutputPort(1).Data unguarded, which throws "Invalid or
                    % deleted object"; the caller's catch then reports the abort as
                    % "Upload ERROR" and resets the bar to 0/fail, destroying exactly
                    % the partial progress the abort branch exists to report.
                    %
                    % Latent since the tier readout was added, and only became certain
                    % when the poll governor made the pre-flight poll coarse: the text
                    % block below is gated on 0.2 s of SIM time, so at 382 streamed
                    % frames most iterations skipped it and the Stop usually landed on
                    % one of those, while at 123 frames nearly every iteration runs it.
                    if ~app.SimRunning
                        r.aborted = true;
                        break;
                    end

                    % The 3D view is redrawn every frame, but the text panels are
                    % throttled to ~5 Hz. Simulink's async run only advances while
                    % we yield, so every millisecond spent setting uifigure
                    % properties is a millisecond the simulation is not running --
                    % and the numbers are unreadable at 14 Hz anyway.
                    if r.simT - lastTextT >= 0.2 || ph ~= lastPhase
                        lastTextT = r.simT;
                        app.updateTelemetry(k, pos, r.simT);
                        app.updateRtkTier(rtoS);
                        % Only in this loop, not in runShowPolled: quick mode gates the
                        % uplink, the telemetry encoder and the ack chains off entirely, so
                        % the readouts there would report a silent radio as though it were a
                        % measurement. Hence the panel heading says full fidelity only.
                        app.updateCommStatus(ph);
                        % updateTelemetry leaves the upload label alone while
                        % SimUploadCount is empty, so the pre-flight wording wins.
                        app.setUploadBar(100 * r.delivered / total, 'busy');
                        app.UploadLabel.Text = app.uploadProgressText( ...
                            attempt, ph, r.simT, tBulkEnd, r.delivered, total);
                        app.refreshRateLabel();
                    end
                    if ph ~= lastPhase
                        app.updateStatus(sprintf( ...
                            'Attempt %d: %s (t = %.1f s, %d / %d items confirmed)', ...
                            attempt, app.uploadPhaseName(ph), r.simT, ...
                            r.delivered, total));
                        lastPhase = ph;
                    end
                    % A failed upload never reaches SHOW, so there is nothing left
                    % to watch — stop rather than idle to StopTime.
                    if r.failed
                        break;
                    end
                end
                % No drawnow here: drawFrame already ends with `drawnow limitrate`,
                % and a full drawnow measured 9.93 ms against 2.15 ms because it
                % bypasses the rate limiter and forces a complete flush every
                % frame. limitrate still services the Stop button, and pause()
                % below is what yields to the simulation.
                %
                % Throttle to real time once the drones are moving, and only then: while
                % the waypoints are on the wire the scene is static, so there is nothing a
                % real-time target could buy. Throttling is the question here, not the poll
                % rate -- the pre-flight still polls, finely enough that SHOW entry is not
                % stepped over, which is what the branches below size. At 200 drones the upload alone is ~108 s
                % of sim time and holding it to 1x would mean nearly two minutes of
                % watching a stationary fleet. So the rate
                % controller governs the SHOW, and the upload runs as fast as it can
                % while still filling the bar.
                w = 0;
                pace = false;      % the upload is not throttled; see pacingArm above
                if r.flew
                    if isempty(tGov)
                        % First poll of the show. The cycle before it paused at the
                        % ceiling through the static upload, so the sim time it covers
                        % belongs to the upload and not to the show -- hence the window
                        % opens here, and this one cycle runs at the same fallback the law
                        % itself uses before anything is measured.
                        tGov = tic; simT0 = r.simT;
                        p = 0.1;
                        % The show has started, so the throttle goes on -- from here rather
                        % than from the model's start, which would have paced the upload.
                        pace = true;
                    else
                        [p, w, pace] = app.nextPollInterval(toc(tBody), ...
                            r.simT - prevSimT, lastPause, lastHold);
                    end
                else
                    p = app.PollMax;
                    % ...except while the items are actually on the wire, which is the
                    % one part of the pre-flight that is not static. PollMax is 0.5 s of
                    % wall clock and the pre-flight advances up to 4.15 s of SIM per poll
                    % (measured), so the whole MISSION_ITEM_INT stream -- ~1.2 s of sim --
                    % fell between two polls: the bar stepped twice on the way from 0 to
                    % 100, against 15 partial frames back when the pre-flight polled at a
                    % fixed 0.1 s. A bar that snaps is not showing the upload, and the
                    % upload is what this mode is for.
                    %
                    % Gated on the stream being IN PROGRESS rather than on the phase, so
                    % it costs a handful of extra polls inside that window and nothing at
                    % all during the GPS lock, the gap repair or the arm dwell. 0.03 s
                    % puts ~0.25 s of sim between polls, which also clears the 0.2 s
                    % sim-time throttle on the text panels below -- polling faster than
                    % that would spin the bar without adding a single bar update.
                    if r.delivered > 0 && r.delivered < total
                        p = min(p, 0.03);
                    elseif r.delivered >= total
                        % ...and once the stream is DONE, for the opposite reason. PollMax is
                        % a wall clock, and what it buys in sim time depends entirely on how
                        % fast the model happens to be running: with items on the wire a poll
                        % covers 0.14 s of sim (measured), but the instant the radio goes
                        % quiet the same 0.5 s poll covers 5.44 s -- an ~11x sprint, unpaced
                        % on purpose so nobody watches a static fleet in real time.
                        %
                        % The cost is that SHOW entry is discovered in ARREARS, and whatever
                        % the model flew inside the crossing poll is never drawn at all.
                        % Measured on the default show: the last pre-show frame was t = 11.59
                        % and the first in-show frame t = 17.03, so the show opened already
                        % 1.03 s into a 6.67 s climb. Nothing bounds that but where the poll
                        % boundary happens to land, and a 5.44 s blind window against a
                        % 6.67 s climb loses most of it about as often as not -- which is
                        % what "after Upload & Fly the takeoff is instantaneous" was. Not a
                        % bad plan and not a coarse buffer: a climb the viewer joined most of
                        % the way up, on the one path that has an unpaced window to cross.
                        %
                        % Same root cause as the r.armed latch above, and the file already
                        % records the other half of it: a poll stepping over arm_duration
                        % reported a perfect show as FAILED. A phase can be latched after the
                        % fact, but a climb has to be drawn while it is happening, so this
                        % one needs the blind window shrunk rather than worked around.
                        %
                        % Nearly free, because the poll interval is wall clock and the sim
                        % advances unpaced either way: the gate wait still passes at ~11x, it
                        % is simply sampled ~10x more finely, so the wall clock is unchanged
                        % and the only added cost is drawing a couple of dozen static frames.
                        % Deliberately not paced instead -- pacing the RTK gate would buy the
                        % operator 13 s of real-time footage of a fleet sitting on its pads.
                        p = min(p, 0.05);
                    end
                end
                prevSimT = r.simT;
                % Before the pause, because the pause is the slice the new rate applies to.
                app.pacingEngage(modelName, pace);
                tP = tic; pause(p); lastPause = toc(tP);
                lastHold = app.holdSolver(w);
            end
            % Dropped before anything else, including the abort path: EnablePacing outlives
            % the run, and a throttle left on the model would pace the next plain sim().
            app.pacingEngage(modelName, false);
            % Stopped here rather than after haltModel: halting waits on the model to come
            % to rest, and that wait is not the viewer streaming at any factor.
            govWall = [];
            if ~isempty(tGov), govWall = toc(tGov); end

            app.haltModel(modelName);
            app.SimRunning = false;
            if ~isempty(govWall)
                app.summarizeLiveRate(govWall, r.simT - simT0);
            end
        end

        function txt = uploadProgressText(app, attempt, ph, simT, tBulkEnd, delivered, total)
            switch ph
                case 0
                    % The bar is pinned at 0% through this phase because nothing has
                    % been delivered yet -- there is no fix, so there is no stream. The
                    % elapsed count alone reads as a stall, so name what is being waited
                    % for and what follows it.
                    txt = sprintf('Attempt %d: waiting for GPS lock (%.1f s)...%s', ...
                        attempt, simT, app.uploadEtaSuffix(' — then ~%.0f s of streaming'));
                case 1
                    if simT <= tBulkEnd
                        txt = sprintf(['Attempt %d: streaming MISSION_ITEM_INT — ' ...
                            '%d / %d confirmed%s'], attempt, delivered, total, ...
                            app.uploadEtaSuffix(' of ~%.0f s'));
                    else
                        txt = sprintf(['Attempt %d: repairing gaps per item — ' ...
                            '%d / %d confirmed'], attempt, delivered, total);
                    end
                otherwise
                    txt = sprintf('Attempt %d: %s — %d / %d confirmed', ...
                        attempt, app.uploadPhaseName(ph), delivered, total);
            end
        end

        function s = uploadEtaSuffix(app, fmt)
            % The cached estimate rendered into fmt, or nothing at all if there isn't
            % one. Silence is the right fallback: a label that says "of ~NaN s" is worse
            % than a label that does not mention the total.
            if isfinite(app.UploadEstimate)
                s = sprintf(fmt, app.UploadEstimate);
            else
                s = '';
            end
        end

        function secs = estimateUploadSeconds(app)
            % How long this upload will stream, in SIM seconds.
            %
            % upload_duration is what setupParams computed for THIS plan, so it is the
            % authoritative figure and is preferred whenever it is there -- num_waypoints
            % depends on the sequence, the transition and the hold, so no constant is
            % right for every show. The 1.09 s per drone fallback is the measured rate
            % for the default 5-formation sequence and only applies before a plan has
            % been built, which on this path should not happen: the upload button is only
            % live once a show exists. Kept anyway because the estimate is a courtesy and
            % must never be the thing that breaks an upload.
            secs = NaN;
            try
                if evalin('base', 'exist(''upload_duration'', ''var'')') == 1
                    secs = evalin('base', 'upload_duration');
                end
            catch
                % A base workspace that cannot be read is not a reason to refuse to
                % start the upload; fall through to the per-drone estimate.
            end
            if ~isscalar(secs) || ~isfinite(secs) || secs <= 0
                secs = 1.09 * app.NumUAVSpinner.Value;
            end
        end

        function name = uploadPhaseName(~, ph)
            names = {'waiting for GPS lock', 'uploading mission', 'ARMED', ...
                     'show running', 'landed', 'landing', 'UPLOAD FAILED'};
            idx = round(ph) + 1;
            if idx >= 1 && idx <= numel(names)
                name = names{idx};
            else
                name = sprintf('phase %g', ph);
            end
        end

        function prepareModel(app, modelName)
            % Get the model ready to simulate, reloading only when it has to.
            %
            % A close/load cycle re-evaluates every mask in the model, and the 44
            % MAVLink Serializer / Deserializer / Blank Message masks re-parse the
            % dialect when they do: measured at ~26 s of the upload's wall clock.
            % Simulink re-resolves workspace variables on every compile, so the
            % only thing a reload is really needed for is a change in N_uav, which
            % changes port dimensions. Repeated uploads and retries at one fleet
            % size now skip it entirely.
            app.haltModel(modelName);
            % A fresh run starts un-aborted, whatever the last one did.
            app.AbortRequested = false;
            app.clearAbort(modelName);
            n = evalin('base', 'N_uav');
            if bdIsLoaded(modelName) && isequal(app.LoadedNUAV, n)
                return;
            end
            if bdIsLoaded(modelName)
                close_system(modelName, 0);
            end
            load_system(modelName);
            app.LoadedNUAV = n;
        end

        function haltModel(app, modelName)
            % Bring the model to a full stop; close_system and load_system both
            % fail while a simulation is still running or compiling.
            if ~bdIsLoaded(modelName)
                return;
            end
            % The last of the three places the live throttle is dropped, and the only one
            % that survives an error thrown from inside a poll loop: that unwinds past the
            % loop's own disengage, and EnablePacing outlives the run, so the model would be
            % left throttled for whatever ran next -- including DroneShowExample's sim().
            % Free to call when nothing is engaged; it returns immediately.
            app.pacingEngage(modelName, false);
            try
                for k = 1:100
                    if strcmp(get_param(modelName, 'SimulationStatus'), 'stopped')
                        return;
                    end
                    set_param(modelName, 'SimulationCommand', 'stop');
                    pause(0.1);
                end
            catch
            end
        end

        function m = simModeString(app)
            simModeMap = struct('Normal', 'normal', ...
                'Accelerator', 'accelerator', ...
                'RapidAccelerator', 'rapid');
            m = simModeMap.(strrep(app.SimModeDropdown.Value, ' ', ''));
        end

        %% ---- Dynamics Simulation (Simulink) ----
        function runSimulation(app)
            % Simulate is the quick path. In MAVLink mode the show is flown by
            % Upload & Fly instead, in the same run as the upload, because the
            % OnboardBuffer the drones read cannot be carried between sim() calls.
            if app.trajSourceValue() ~= 1
                app.updateStatus(['MAVLink mode flies the show as part of the ' ...
                    'upload — click Upload & Fly.']);
                return;
            end
            if isempty(app.TrajectoryData)
                app.updateStatus('Generate a show first.');
                return;
            end
            % Streaming needs Simulink.RunTimeBlock reads. Rather than silently overriding the
            % Sim Mode dropdown, honour it: Normal and Accelerator stream, Rapid keeps the
            % simulate-then-animate behaviour and says why.
            %
            % ACCELERATOR IS INCLUDED ON MEASUREMENT, not assumption. It runs in-process, so all
            % three live-data routes work: SimulationStatus reads 'running', RuntimeObject values
            % advance, and SDI streams. Accelerator does apply block reduction, which would
            % have been the way this failed -- a reduced block returns
            % an EMPTY RuntimeObject rather than erroring, and the frame guard below needs three of
            % them at once, so one reduced block would mean zero frames for the whole show, not a
            % degraded view. All four polled blocks were checked directly: the guard passed on
            % 238/238 polls in accelerator vs 242/242 in normal.
            %
            % Only RAPID accelerator genuinely cannot stream: it runs as a separate executable, so
            % status reads 'external' and RuntimeObject values never advance.
            %
            % Accelerator is also ~18 % faster warm (36.09 s vs 43.95 s median, non-overlapping
            % ranges) but pays a one-off ~24 s target build, so it is NOT made the default here --
            % that is a separate call, and a run that rebuilds is a net loss. The dropdown stays
            % the user's choice; this only stops us breaking the live view when they make it.
            if any(strcmp(app.simModeString(), {'normal', 'accelerator'}))
                app.runSimulationStreamed();
                return;
            end
            app.updateStatus(['Running Simulink simulation... (' ...
                app.SimModeDropdown.Value ' cannot stream — the live viewer ' ...
                'needs Normal or Accelerator mode. The show will animate when the run finishes.)']);
            drawnow;

            try
                modelName = 'MultiUAV_DroneShow';
                % Full sim spans pre-show (lock+upload+arm) + show + land.
                % show_duration alone stops before the trajectory even starts playing.
                simDur = evalin('base', 'total_sim_duration');

                % Reload only if the fleet size changed; the upload usually leaves
                % the model loaded and ready, and reloading is expensive.
                app.prepareModel(modelName);

                % The show sim honours the Sim Mode dropdown; only the upload is
                % pinned to Normal (it needs runtime reads for live progress).
                simMode = app.simModeString();
                set_param(modelName, 'SimulationMode', simMode);
                set_param(modelName, 'StopTime', num2str(simDur));
                app.updateStatus(sprintf('Simulating %s (%s) for %.1fs... (Stop to abort)', modelName, simMode, simDur));
                app.SimRunning = true;
                app.refreshScrub();   % greys the bar out: frames ahead of "now" do not exist
                app.StopBtn.Enable = 'on';
                app.SimBtn.Enable = 'off';
                app.GenerateBtn.Enable = 'off';
                drawnow;

                out = sim(modelName);
                app.SimRunning = false;
                app.StopBtn.Enable = 'off';
                app.SimBtn.Enable = 'on';
                app.GenerateBtn.Enable = 'on';

                % Extract fleet positions from simulation output
                % out.fleetPositions is a timeseries; .Data is [N x 3 x T]
                simData = out.fleetPositions.Data;
                simTime = out.fleetPositions.Time;

                % Downsample to ~20 Hz for smooth playback
                dt_target = 0.05;
                dt_sim = mean(diff(simTime));
                dsRatio = max(1, round(dt_target / dt_sim));
                simData = simData(:, :, 1:dsRatio:end);
                simTime = simTime(1:dsRatio:end);
                nSimSamp = length(simTime);

                app.SimTrajectoryData = simData;
                app.SimTimeVector = simTime;
                app.UseSimData = true;
                app.PlayIdx = 1;
                app.NumSamples = nSimSamp;

                % Capture phase and upload progress logs
                app.SimPhaseData = [];
                app.ShowEntryTime = [];   % cleared with the phase log it is derived from
                app.SimUploadCount = [];
                try
                    app.SimPhaseData = out.logPhase;
                catch
                end
                try
                    app.SimUploadCount = out.logUploadCount;
                catch
                end
                app.UploadTotal = evalin('base','N_uav') * evalin('base','num_waypoints');

                % Rebuild scenario for replay with sim data extents
                app.buildScenario();

                app.PlayBtn.Text = 'Play [Sim]';
                app.refreshScrub();   % the logged run is a different length from the plan
                app.refreshTrajOverlay();   % same as after Generate: no frame follows this
                app.updateStatus(sprintf( ...
                    'Simulation complete (%.1fs, %d frames). Click Play [Sim] for simulated data.', ...
                    simTime(end), nSimSamp));
            catch e
                app.updateStatus(['Simulation ERROR: ' e.message]);
                disp(['DroneLightShowApp simulation error: ' e.message]);
                disp(getReport(e, 'extended'));
                app.SimRunning = false;
                app.StopBtn.Enable = 'off';
                app.SimBtn.Enable = 'on';
                app.GenerateBtn.Enable = 'on';
            end
        end

        function runSimulationStreamed(app)
            % The quick path, streamed. Same non-blocking pattern Upload & Fly
            % uses: start the model with SimulationCommand and poll the fleet
            % positions off a cached RunTimeBlock, painting each instant as it
            % happens instead of animating from `out` once the run is over.
            %
            % The frames are kept, so finishLiveRun turns the run into an ordinary
            % replay (Play [Sim], and scrubbable) without simulating a second time.
            modelName = 'MultiUAV_DroneShow';
            app.stopPlayback();

            app.UseSimData = true;
            app.SimTrajectoryData = [];
            app.SimTimeVector = [];
            app.SimPhaseData = [];
            app.ShowEntryTime = [];   % cleared with the phase log it is derived from
            app.SimUploadCount = [];
            app.buildScenario();
            app.StopBtn.Enable = 'on';
            app.AbortBtn.Enable = 'on';
            app.SimBtn.Enable = 'off';
            app.GenerateBtn.Enable = 'off';
            % Said before prepareModel, not after: compiling the model takes
            % seconds, and whatever the status bar last held is what the operator
            % reads for all of it. The streaming message with the duration follows
            % once the run actually starts.
            app.updateStatus('Preparing model for a live run...');
            drawnow;

            try
                simDur = evalin('base', 'total_sim_duration');
                app.prepareModel(modelName);
                % Honour the dropdown rather than forcing Normal. Accelerator streams too --
                % verified on all four polled blocks, see the note in runSimulation. Hardcoding
                % 'normal' here would have silently defeated that gate: the run would have been
                % routed to the streaming path and then quietly demoted back to Normal, so the
                % dropdown would have looked broken rather than slow. Rapid never reaches here.
                simMode = app.simModeString();
                set_param(modelName, 'SimulationMode', simMode);
                set_param(modelName, 'StopTime', num2str(simDur));
                app.LiveStopTime = simDur;
                app.UploadTotal = [];

                % Accelerator builds a target on its first run for a given model structure
                % (~24 s, and changing fleet size invalidates it). runShowPolled treats
                % 'initializing' as alive so the build is simply absorbed, but the operator
                % would otherwise be watching a still viewport with no idea why.
                if strcmp(simMode, 'accelerator')
                    app.updateStatus(sprintf(['Streaming %.1f s of flight live in Accelerator ' ...
                        'mode... (first run rebuilds the target, which takes a few seconds; ' ...
                        'Stop to abort)'], simDur));
                else
                    app.updateStatus(sprintf( ...
                        'Streaming %.1f s of flight live... (Stop to abort)', simDur));
                end
                r = app.runShowPolled(modelName);

                app.finishLiveRun();

                % Two counts, because they are now different things: what was drawn
                % live is governor-limited, what is replayable came off the dense log.
                n = numel(app.SimTimeVector);
                nLive = app.LiveStreamedFrames;
                if app.AbortRequested
                    % Checked before r.aborted: the run reached its stop time
                    % normally, so nothing else here would mention the abort at all.
                    app.updateStatus(sprintf(['Mission ABORTED — the fleet was landed ' ...
                        'on operator command and the run finished at t = %.1f s ' ...
                        '(%d frames, replayable).'], r.simT, n));
                elseif r.aborted
                    app.updateStatus(sprintf(['Run stopped by operator at ' ...
                        't = %.1f s — %d frames drawn live, %d replayable.'], ...
                        r.simT, nLive, n));
                elseif n == 0
                    app.updateStatus(['No frames were streamed — the run produced ' ...
                        'no fleet positions.']);
                else
                    app.updateStatus(sprintf(['Show flown and streamed live ' ...
                        '(%.1f s, %d frames drawn live, %d replayable). Click ' ...
                        'Play [Sim] to watch it back smoothly, or drag the bar to a ' ...
                        'moment you want again.'], ...
                        app.SimTimeVector(end), nLive, n));
                end
                app.StopBtn.Enable = 'off';
                app.SimBtn.Enable = 'on';
                app.GenerateBtn.Enable = 'on';
            catch e
                app.SimRunning = false;
                app.finishLiveRun();
                app.StopBtn.Enable = 'off';
                app.SimBtn.Enable = 'on';
                app.GenerateBtn.Enable = 'on';
                app.updateStatus(['Simulation ERROR: ' e.message]);
            end
        end

        function r = runShowPolled(app, modelName)
            % Poll loop for the quick path: no upload, so no progress bar and no
            % arrival count — just phase and positions.
            phaseBlk = [modelName '/ShowSupervisor'];
            sigBlk   = [modelName '/RadioChannel/CorrectionAgeMonitor/InjectGate'];

            % THE DIRECT TAP, deliberately. This mode exists to answer "where did the
            % fleet go" quickly, so it reads the true airframe position straight off the
            % dynamics and never asks what the ground station heard. The full-fidelity
            % path draws the decoded downlink instead -- see runUploadPolled. Keeping the
            % difference here rather than behind a shared helper is the point: the two
            % modes are answering different questions, not trading resolution.
            posBlk   = [modelName '/DroneFleet/FlightDynamics/TransposePos'];

            r = struct('aborted', false, 'simT', 0);
            rtoP = []; rtoX = []; rtoS = []; lastPhase = -1; lastTextT = -inf;
            app.LivePhaseLog = [];
            app.LiveCountLog = [];      % stays empty: nothing is being uploaded
            % Always the true tap here, so the dense log is always the same quantity
            % the poll log holds and finishLiveRun can always upgrade to it.
            app.LiveLogIsTruePose = true;
            app.armRtkTier();
            app.armPollRate();
            prevSimT = 0; lastPause = 0; lastHold = 0;
            % The summary window opens at the first governed poll rather than at
            % SimulationCommand start: model startup is wall clock the operator waits
            % through, but it is not the viewer streaming at some factor, and folding it
            % in would report a slow show whenever the model happened to load cold.
            tGov = []; simT0 = 0;

            app.SimRunning = true;
            % Greys the scrub bar out for the duration. seekFrame refuses during a live
            % run anyway -- the frames ahead of "now" do not exist yet -- but a control
            % that looks usable and then refuses is worse than one that is visibly out.
            app.refreshScrub();
            % Drop any previous run's `out` before starting. upgradeReplayLog reads
            % out.fleetPositions afterwards, and a run that dies before publishing one
            % would otherwise let it splice the PREVIOUS run's flight into this run's
            % replay -- the fleet-size and end-time guards there would not catch it,
            % because a re-run at the same fleet size passes both.
            evalin('base', 'clear out');
            % Engaged BEFORE the start here, unlike on the upload path: there is no upload
            % to watch, the governor rules this run from its first poll, and a parameter set
            % before the start is one the solver never has to be interrupted for.
            app.pacingArm(modelName);
            app.pacingEngage(modelName, true);
            set_param(modelName, 'SimulationCommand', 'start');
            while true
                tBody = tic;   % everything up to the pause is time the solver is idle
                % Checked before SimulationStatus: stopAll clears SimRunning and
                % issues the stop, so the status may already read 'stopped' and
                % would otherwise look like an ordinary end of run.
                if ~app.SimRunning
                    r.aborted = true;
                    break;
                end
                st = get_param(modelName, 'SimulationStatus');
                if ~any(strcmp(st, {'running', 'initializing', 'paused'}))
                    break;
                end

                r.simT = get_param(modelName, 'SimulationTime');
                if isempty(rtoP)
                    rtoP = get_param(phaseBlk, 'RuntimeObject');
                    rtoX = get_param(posBlk, 'RuntimeObject');
                    rtoS = get_param(sigBlk, 'RuntimeObject');
                end
                if ~isempty(rtoP) && ~isempty(rtoX)
                    ph = double(rtoP.OutputPort(1).Data);
                    pos = double(rtoX.OutputPort(1).Data);      % [N x 3] NED

                    % The first poll can land after SimulationStatus reads
                    % 'running' but before the fleet outputs have been written
                    % once, so the port still holds its zero-initialised buffer.
                    % Keeping it stacks the whole fleet on the origin for a frame
                    % and makes min separation read 0.
                    if isempty(app.SimTimeVector) && all(pos(:) == 0)
                        drawnow;
                        tP = tic; pause(0.05); lastPause = toc(tP);
                        prevSimT = r.simT;
                        continue;
                    end

                    k = numel(app.SimTimeVector) + 1;
                    app.SimTimeVector(k, 1) = r.simT;
                    app.SimTrajectoryData(:, :, k) = pos;
                    app.LivePhaseLog(k, 1) = ph;
                    app.LivePhase = ph;
                    % Outport 2 is show_time; see latchShowEntry for why the poll time alone
                    % is not good enough. Read only until the anchor is latched.
                    if isempty(app.ShowEntryTime)
                        app.latchShowEntry(ph, r.simT, double(rtoP.OutputPort(2).Data));
                    end
                    app.drawFrame(pos, r.simT, ' [Live - direct]');

                    % Same reason as in runUploadPolled: the Stop button is serviced by
                    % the drawnow inside drawFrame, and updateRtkTier below reads a
                    % RuntimeObject that is dead the instant the model terminates.
                    if ~app.SimRunning
                        r.aborted = true;
                        break;
                    end

                    % Text panels throttled to ~5 Hz: the async run only advances
                    % while we yield, so every millisecond spent setting uifigure
                    % properties is a millisecond the simulation is not running.
                    if r.simT - lastTextT >= 0.2 || ph ~= lastPhase
                        lastTextT = r.simT;
                        app.updateTelemetry(k, pos, r.simT);
                        app.updateRtkTier(rtoS);
                        app.refreshRateLabel();
                    end
                    if ph ~= lastPhase
                        app.updateStatus(sprintf('Live: %s (t = %.1f s)', ...
                            app.uploadPhaseName(ph), r.simT));
                        lastPhase = ph;
                    end
                end
                % No drawnow: drawFrame ends with `drawnow limitrate`, which still
                % services the Stop button. pause() is what yields to the solver.
                %
                % The interval is solved for the operator's target rather than fixed at
                % 0.1 s -- see nextPollInterval for the control law. The fixed value was
                % chosen against one measurement (at 0.05 s a 22 s show streamed 784
                % frames and took 104 s wall against 39 s for the old animate-afterwards
                % path) and it could only ever be right for that show: the cost it is
                % trading against is the DRAWING cost, which changes by a large factor
                % with the render mode and the trails.
                if isempty(tGov)
                    tGov = tic; simT0 = r.simT;
                end
                [p, w, pace] = app.nextPollInterval(toc(tBody), r.simT - prevSimT, ...
                    lastPause, lastHold);
                prevSimT = r.simT;
                % Before the pause, because the pause is the slice the new rate applies to.
                app.pacingEngage(modelName, pace);
                tP = tic; pause(p); lastPause = toc(tP);
                % After the pause, not before: the pause is the solver's slice and the hold
                % is the part of the cycle it does not get. Swapping them would work out to
                % the same arithmetic but would make the frame land at the wrong moment --
                % the drawing happens at the top of the loop, so holding last is what keeps
                % the display an even interval apart.
                lastHold = app.holdSolver(w);
            end
            % Dropped before anything else, the abort path included: EnablePacing outlives
            % the run, and a throttle left on the model would pace the next plain sim().
            app.pacingEngage(modelName, false);
            % Before haltModel, which waits on the model to come to rest — a wait that is
            % not the viewer streaming at any factor.
            govWall = [];
            if ~isempty(tGov), govWall = toc(tGov); end

            app.haltModel(modelName);
            app.SimRunning = false;
            if ~isempty(govWall)
                app.summarizeLiveRate(govWall, r.simT - simT0);
            end
        end

        %% ---- Reset ----
        function resetAll(app)
            app.stopPlayback();
            app.PlayIdx = 1;
            app.UseSimData = false;
            app.SimTrajectoryData = [];
            app.SimTimeVector = [];
            app.SimPhaseData = [];
            app.ShowEntryTime = [];   % cleared with the phase log it is derived from
            app.SimUploadCount = [];
            app.UploadTotal = [];
            app.PlayBtn.Enable = 'off';
            % After the clear above, not before: stopPlayback labelled the button while the
            % sim data was still there, so '[Sim]' has to be taken back off it here.
            app.PlayBtn.Text = app.playIdleText();
            app.refreshScrub();   % reads PlayBtn, so it must follow the line above
            app.UploadBtn.Enable = 'off';
            app.AbortBtn.Enable = 'off';
            app.AbortRequested = false;
            app.clearAbort('MultiUAV_DroneShow');
            % Reset means back to a healthy fleet. Unlike the abort this one is visible
            % on the panel, so leaving it set would not be a silent trap -- but Reset
            % clearing everything else and not this would be the surprise.
            app.publishDenyMask(false(app.denyMaskLength(), 1));
            app.refreshDenyLabel();
            % The tier readout is a property of a run, not of the fleet, so it goes back
            % to '—' rather than to 'RTK Fix' -- nothing is flying to be on a tier.
            app.RTKTierLabel.Text = '—';
            app.RTKTierLabel.FontColor = [0.35 0.35 0.35];
            app.resetUploadState();
            app.TimeLabel.Text = 'Time: 0.0 / 0.0 s';
            app.PhaseLabel.Text = 'Phase: --';
            app.FormLabel.Text = 'Formation: --';
            app.StateLabel.Text = 'Show State: Idle';
            app.MinSepLabel.Text = 'Min Separation: -- m';
            app.MinSepLabel.FontColor = [0 0 0];
            app.MaxErrLabel.Text = 'Max Speed: -- m/s';
            app.SpeedLabel.Text = 'Fleet Size: --';
            cla(app.ScenarioAxes);
            % The cla destroyed the overlay's line, so drop the handle and the phase it had
            % cached rather than holding a dead one until the next buildScenario.
            app.clearTrajOverlay();
            title(app.ScenarioAxes, 'Generate a show to begin', 'Color', [0.5 0.5 0.5]);
            app.updateStatus('Reset. Configure and generate a new show.');
        end

        %% ---- Helpers ----
        function resetUploadState(app)
            % Back to "nothing is onboard": the show cannot be flown until a
            % fresh upload confirms every waypoint.
            app.UploadOK = false;
            app.UploadAttempt = 0;
            app.SimBtn.Enable = 'off';
            app.UploadBtn.Text = 'Upload & Fly';
            app.UploadLamp.Color = [0.55 0.55 0.55];
            app.UploadLabel.Text = 'Not uploaded.';
            app.setUploadBar(0, 'busy');
        end

        function setSpeed(app, val)
            app.PlaySpeed = val;
            lbl = findall(app.Fig, 'Tag', 'SpeedVal');
            if ~isempty(lbl)
                lbl.Text = sprintf('%.1fx', val);
            end
            % Re-anchor the pacing clock at the sample showing right now. The deadline
            % is PaceRefTime + elapsed*PlaySpeed, so changing the speed without moving
            % the anchor would re-scale time already spent: dragging 1x -> 4x thirty
            % seconds in would put the deadline at 120 s of show and jump the fleet to
            % the end. Anchoring here means a speed change takes effect from now on,
            % which is what dragging a slider mid-show is asking for.
            if ~isempty(app.PaceClock)
                app.PaceClock = tic;
                if app.UseSimData && ~isempty(app.SimTimeVector)
                    app.PaceRefTime = app.SimTimeVector(min(app.PlayIdx, end));
                elseif ~isempty(app.TimeVector)
                    app.PaceRefTime = app.TimeVector(min(app.PlayIdx, end));
                end
            end
        end

        function setPoseView(app, val)
            % Read at the top of runUploadPolled, so switching mid-run does not take
            % effect until the next run. Deliberately not live: the tap is a different
            % block, and rebinding the runtime object handle in the middle of the poll
            % loop would have to survive a rebuild of the very thing being polled.
            app.PoseView = val;
            if app.SimRunning
                app.updateStatus(['Pose tap set to ' val ...
                    ' — takes effect on the next run; this one keeps the tap it started with.']);
            elseif strcmp(val, 'true')
                app.updateStatus(['3-D view will show the TRUE airframe pose: all ' ...
                    'drones every frame, straight from flight dynamics. The radio is ' ...
                    'still fully simulated — this changes what is displayed, not what flies.']);
            else
                app.updateStatus(['3-D view will show the pose AS RECEIVED by the ' ...
                    'ground station. The fleet takes turns on the link, one drone per ' ...
                    'time step, so the table refreshes row by row and a big fleet looks stale.']);
            end
        end

        function setCameraMode(app, mode)
            app.CameraMode = mode;
            if strcmp(mode, 'Static')
                view(app.ScenarioAxes, 35, 25);
            end
        end

        function formSeq = parseFormationString(app, str)
            parts = strsplit(str, '→');
            % Built from the live name list rather than a fixed map, so a shape
            % loaded from a file parses the same way a built-in pattern does. Its
            % type is its position in that list, which is exactly the convention
            % setupParams and lighting_colors use.
            names = app.formationNames();
            formSeq = zeros(1, numel(parts));
            for i = 1:numel(parts)
                hit = find(strcmp(strtrim(parts{i}), names), 1);
                if isempty(hit)
                    formSeq(i) = 1;
                else
                    formSeq(i) = hit;
                end
            end
        end

        %% ---- Sequence builder ----
        function syncSequenceFromList(app)
            % Adopt whatever the dropdown is showing as the sequence under edit. Called
            % when the operator picks a preset, and once at construction.
            %
            % One sequence, two views: without this, choosing a preset and then
            % pressing Add would append to whatever the buttons last built, and the
            % show would not be the string on screen.
            parts = strtrim(strsplit(app.FormationList.Value, '→'));
            app.SequenceNames = parts(~cellfun(@isempty, parts));
            app.refreshSequenceLabel();
        end

        function addToSequence(app)
            app.setSequence([app.SequenceNames, {app.FormationPicker.Value}]);
        end

        function undoSequence(app)
            % A show needs at least one formation, so the last step will not come off.
            % Refusing is better than allowing an empty sequence that only fails later
            % inside setupParams.
            if numel(app.SequenceNames) <= 1
                app.updateStatus(['A show needs at least one formation. Use Clear to ' ...
                    'start again, or pick a different one and press Add.']);
                return;
            end
            app.setSequence(app.SequenceNames(1:end-1));
        end

        function clearSequence(app)
            app.setSequence({'Grid'});
        end

        function setSequence(app, names)
            % Write a sequence into the dropdown and make it the selection.
            %
            % The dropdown stays the single place the sequence lives -- generateShow
            % reads FormationList.Value and nothing else -- so building one is really
            % just composing that string. The item is REPLACED rather than appended so
            % the list does not fill up with every intermediate step; and if the
            % sequence happens to equal a preset, the preset is selected and no
            % duplicate is added.
            app.SequenceNames = names;
            str = strjoin(names, '→');
            items = app.FormationList.Items;
            if ~isempty(app.BuiltSeqItem)
                items(strcmp(items, app.BuiltSeqItem)) = [];
            end
            if any(strcmp(items, str))
                app.BuiltSeqItem = '';       % it is a preset now; nothing to replace
            else
                items{end+1} = str;
                app.BuiltSeqItem = str;
            end
            app.FormationList.Items = items;
            app.FormationList.Value = str;
            app.refreshSequenceLabel();
            % A new sequence invalidates the generated plan, and saying so is the only
            % hint that Add did not fly anything by itself.
            app.updateStatus(sprintf('Sequence: %s (%d formations). Press Generate.', ...
                str, numel(names)));
        end

        function refreshSequenceLabel(app)
            n = numel(app.SequenceNames);
            if n == 0
                app.SequenceLabel.Text = 'Sequence: empty.';
                app.SequenceLabel.Tooltip = '';
                return;
            end
            steps = arrayfun(@(k) sprintf('%d. %s', k, app.SequenceNames{k}), ...
                1:n, 'UniformOutput', false);
            app.SequenceLabel.Text = sprintf('Sequence (%d): %s', n, ...
                strjoin(steps, '   '));
            % The label wraps to two lines; a long show outruns even that, so the
            % tooltip carries every step one per line and is never truncated.
            app.SequenceLabel.Tooltip = strjoin(steps, newline);
            % Rotation angles are per STEP, so the step picker has to follow the sequence.
            % Hooked here because every sequence edit -- preset, Add, Undo, Clear -- funnels
            % through this one function, and a second place to remember would eventually
            % leave the picker naming a formation the show no longer contains.
            app.refreshRotationSteps();
            app.refreshRotationLabel();
        end

        function names = formationNames(app)
            % Formation type -> display name. The four built-ins, then the loaded
            % shapes in load order: type 5 is the first one loaded.
            names = {'Grid', 'Circle', 'Sphere', 'Text'};
            if ~isempty(app.CustomFormations)
                names = [names, {app.CustomFormations.name}];
            end
        end

        function name = uniqueFormationName(app, name)
            % Two files called logo.png in different folders would otherwise both
            % want to be "Logo", and the dropdown string is how a formation is
            % identified -- the second one would silently parse as the first.
            taken = app.formationNames();
            if ~any(strcmp(name, taken))
                return;
            end
            for k = 2:99
                cand = sprintf('%s%d', name, k);
                if ~any(strcmp(cand, taken))
                    name = cand;
                    return;
                end
            end
        end

        function loadShapeFile(app)
            [file, folder] = uigetfile( ...
                {'*.png;*.jpg;*.jpeg;*.bmp;*.tif;*.tiff;*.gif', ...
                    'Images (*.png, *.jpg, *.bmp, *.tif, *.gif)'; ...
                 '*.stl', '3D models (*.stl)'; ...
                 '*.*', 'All files'}, ...
                'Pick a picture or an STL to fly');
            % uigetfile puts the dialog behind the app on Windows often enough to
            % look like a hang, so bring the figure back to the front afterwards.
            figure(app.Fig);
            if isequal(file, 0)
                app.updateStatus('Load cancelled.');
                return;
            end
            try
                app.addCustomFormation(fullfile(folder, file));
            catch e
                app.updateStatus(['ERROR loading shape: ' e.message]);
            end
        end

        function flyText(app)
            str = strtrim(app.TextField.Value);
            if isempty(str)
                app.updateStatus('Type something in the Text box first.');
                return;
            end
            try
                name = app.addTextFormation(str);
            catch e
                app.updateStatus(['ERROR building text: ' e.message]);
                return;
            end
            % registerFormation ends in generateShow, which writes its own status, so
            % the legibility note goes AFTER it or it would never be seen. It is the
            % one thing about a text formation the operator cannot read off the
            % viewer: 20 drones spelling a long word looks like a bug, not like a
            % fleet that is simply too small to draw the letters.
            nChars = numel(regexprep(str, '[\s|]', ''));
            perChar = app.NumUAVSpinner.Value / max(nChars, 1);
            if perChar < 10
                app.updateStatus(sprintf(['Flying "%s" -- but %d drones over %d ' ...
                    'characters is %.1f each, and below ~10 it will not read as ' ...
                    'text. Split it with "|", shorten it, or raise the fleet.'], ...
                    name, app.NumUAVSpinner.Value, nChars, perChar));
            end
        end

        function fitted = fitTransitionToPlan(app, needT)
            % Give a too-fast plan a transition long enough to actually fly, then
            % regenerate once. Returns true if it did, meaning the caller's status text
            % and labels are stale and it should not write over them.
            %
            % Two things make a plan too fast, and the cure is the same for both.
            % A LOADED SHAPE is too big for its fleet: an outline strings its drones
            % along a curve, so to hold d_min it has to be scaled well past the
            % footprint a Grid of the same fleet occupies -- a 40-drone billboard ends
            % up ~60 m wide against a Grid's ~30 m. A LARGE FLEET is too big for any
            % formation: the built-in Circle and Sphere radii are
            % N_uav*formation_spacing/(2*pi), so the transit grows linearly with the
            % fleet while Transition (s) stays whatever was typed. Both end as the same
            % arithmetic -- span over time exceeding what the fleet can track -- and by
            % 100 drones it is not marginal: 47 m/s against a limit of 8.
            %
            % setupParams already computes the transition that fixes it
            % (traj_min_transition, sized to land at v_track_target rather than right on
            % the limit). Geometry does not change with transition duration, so scaling
            % the time is exact and one regenerate is enough -- no search.
            fitted = false;
            if app.AutoFitBusy
                return;      % already inside the one regenerate; do not recurse
            end
            oldT = app.TransitionField.Value;
            wantT = ceil(needT * 10) / 10;
            % A uieditfield THROWS on an out-of-range assignment rather than clamping,
            % and inside generateShow's try that would surface as a bare ERROR with no
            % hint of the real problem. Clamp here, and if the clamp bites, let the
            % regenerated plan report itself unflyable -- it still is.
            maxT = app.TransitionField.Limits(2);
            newT = min(wantT, maxT);
            if newT <= oldT
                return;
            end

            app.AutoFitBusy = true;
            restore = onCleanup(@() app.setAutoFitBusy(false));
            app.TransitionField.Value = newT;
            app.generateShow();

            % Say what was changed and why. Silently lengthening the show would leave
            % the operator wondering why a 45 s show became 60 s.
            fitted = true;
            if wantT > maxT
                note = sprintf(['  [Transition raised %.1f -> %.1f s automatically, ' ...
                    'the most this field allows; %d drones in "%s" need %.1f s. ' ...
                    'Reduce Spacing (m) or the fleet size too.]'], ...
                    oldT, newT, app.NumUAVSpinner.Value, app.FormationList.Value, wantT);
            else
                note = sprintf(['  [Transition raised %.1f -> %.1f s automatically: ' ...
                    '%d drones in "%s" cannot cross that far in %.1f s.]'], ...
                    oldT, newT, app.NumUAVSpinner.Value, app.FormationList.Value, oldT);
            end
            app.updateStatus([app.StatusBar.Text note]);
        end

        function setAutoFitBusy(app, tf)
            % Exists only so onCleanup can clear the guard: an anonymous function
            % cannot assign to a property, but it can call a method that does.
            app.AutoFitBusy = tf;
        end

        function previewFormation(app, ftype)
            % Paint the frame the PLAN holds that formation on, taken straight out of
            % trajectory_data. Re-deriving the geometry here to preview it would be a
            % second copy of the placement maths, and a preview that agrees with the
            % picture while disagreeing with the show is worse than none.
            if isempty(app.TrajectoryData) || isempty(app.FormSeq)
                return;
            end
            fi = find(app.FormSeq == ftype, 1);
            if isempty(fi)
                return;
            end
            seg = 2 * fi - 1;              % holds sit at odd segment indices
            if seg > numel(app.TimelineTimes)
                return;
            end
            [~, idx] = min(abs(app.TimeVector - (app.TimelineTimes(seg) + 0.5)));
            pos = squeeze(app.TrajectoryData(:, 1:3, idx));
            app.drawFrame(pos, app.TimeVector(idx), ' [preview]');
            app.updateTelemetry(idx, pos, app.TimeVector(idx));
            app.PlayIdx = 1;               % Play still starts from the top
        end

        function c = formationColorFor(app, ftype)
            % What that formation type will fly with: the operator's override if there
            % is one, otherwise whatever setupParams last resolved. Returns [] when
            % nothing has been generated yet and there is no override, which is the
            % swatch's cue to say "default" rather than invent a colour.
            %
            % The 12-row default table is NOT copied here on purpose. Two copies of a
            % show's colours is two answers to "what did it fly", and only one of them
            % would be the one in setupParams.
            c = [];
            if ftype >= 1 && ftype <= size(app.ColorOverrides, 1) && ...
                    all(isfinite(app.ColorOverrides(ftype, :)))
                c = app.ColorOverrides(ftype, :);
                return;
            end
            if ~isempty(app.FormationColors)
                % Same wrap getUAVColors uses, so the swatch shows what the viewer
                % will actually draw.
                c = app.FormationColors(mod(ftype-1, size(app.FormationColors,1)) + 1, :);
            end
        end

        function refreshColorSwatch(app)
            if isempty(app.ColorSwatchBtn) || ~isvalid(app.ColorSwatchBtn)
                return;
            end
            % The dropdown is rebuilt whenever a shape is loaded, so re-seed it here
            % rather than in every caller that can add one.
            names = app.formationNames();
            if ~isequal(app.ColorFormDropdown.Items, names)
                keep = app.ColorFormDropdown.Value;
                app.ColorFormDropdown.Items = names;
                if any(strcmp(keep, names))
                    app.ColorFormDropdown.Value = keep;
                end
            end
            ftype = find(strcmp(app.ColorFormDropdown.Value, names), 1);
            if isempty(ftype)
                return;
            end
            c = app.formationColorFor(ftype);
            if isempty(c)
                app.ColorSwatchBtn.BackgroundColor = [0.94 0.94 0.94];
                app.ColorSwatchBtn.Text = 'default';
                app.ColorSwatchBtn.FontColor = [0.35 0.35 0.35];
                return;
            end
            app.ColorSwatchBtn.BackgroundColor = c;
            app.ColorSwatchBtn.Text = '';
            % A dark swatch with dark text is unreadable, and Text is set to '' here
            % anyway -- FontColor is carried for the 'default' state above.
            if mean(c) < 0.5
                app.ColorSwatchBtn.FontColor = [1 1 1];
            else
                app.ColorSwatchBtn.FontColor = [0 0 0];
            end
        end

        function pickFormationColor(app)
            names = app.formationNames();
            ftype = find(strcmp(app.ColorFormDropdown.Value, names), 1);
            if isempty(ftype)
                return;
            end
            c0 = app.formationColorFor(ftype);
            if isempty(c0)
                c0 = [1 1 1];
            end
            c = uisetcolor(c0, sprintf('Colour for %s', names{ftype}));
            % Cancel gives back a scalar 0 in some releases and the input colour in
            % others, so both are treated as "no change".
            if ~isequal(size(c), [1 3]) || isequal(c, c0)
                return;
            end
            app.setFormationColor(ftype, c);
        end

        function colors = getUAVColors(app, formIdx)
            if ~isempty(app.FormationColors)
                cIdx = mod(formIdx-1, size(app.FormationColors,1)) + 1;
                baseColor = app.FormationColors(cIdx, :);
            else
                baseColor = [1.0 0.9 0.0];
            end
            colors = repmat(baseColor, app.NumUAVs, 1);
            % Slight per-UAV variation — deterministic (no rng reseed)
            persistent jitter;
            if isempty(jitter) || size(jitter,1) < app.NumUAVs
                % Grow to the fleet, never to a fixed 100. The old version regenerated
                % 100 rows however big the fleet was, so above 100 drones the guard
                % noticed the table was short, rebuilt it exactly as short, and the
                % indexing below threw -- the real reason the spinner could not be
                % raised. The floor of 100 keeps the call itself unchanged for any fleet
                % up to 100, so those shows dither exactly as they did before (rand
                % fills column-major, so a taller table would reshuffle every row).
                s = RandStream('twister','Seed',42);
                jitter = 0.08 * (rand(s, max(100, app.NumUAVs), 3) - 0.5);
            end
            colors = colors + jitter(1:app.NumUAVs, :);
            colors = max(0, min(1, colors));

            % ---- denied drones are drawn RED. Marking them is the whole answer to "which
            % drone is which", and there is no other one available: formation slots are
            % re-assigned by matchpairs on squared distance at every transition
            % (setupParams.m:770), so drone 3 occupies a different position in every
            % formation and there is no index-to-place rule an operator could learn.
            % Marking the ones you named is therefore not a convenience, it is the only
            % way the Degrade UAV(s) field means anything on screen.
            %
            % COLOUR ALONE IS NOT ENOUGH, and the rings drawn by buildScenario/drawFrame
            % are what actually carry the identification. The default palette's first
            % formation is pure red [1 0 0], so a drone degraded during the opening
            % formation was being "marked" in a shade its neighbours already wore -- and
            % choosing some other marker colour does not fix it, because the Colour picker
            % lets the operator set any formation to any colour. The colour here is a
            % secondary cue for the formations it does contrast with; the ring is the one
            % that always works. (An earlier version of this comment claimed the flat red
            % sat outside the range any formation could reach after jitter. It does not.)
            %
            % A FLAT red with the jitter removed, not a red plus jitter: the jitter is
            % what makes a formation look like individual aircraft rather than a solid
            % block, and here it works against us -- 8% dither on a red is still a red,
            % but a red that varies reads as "several different things went wrong". One
            % exact colour reads as one state.
            %
            % Guarded on length as well as emptiness: the cache is written by
            % refreshDenyLabel at the fleet size that was current THEN, and the spinner can
            % move between a degrade and the next Generate. A stale short or long mask must
            % not throw in the draw loop, so anything that does not match the fleet exactly
            % is ignored until the next refresh brings it back into step.
            if ~isempty(app.DenyMaskCache) && numel(app.DenyMaskCache) == app.NumUAVs
                colors(app.DenyMaskCache, :) = repmat([0.95 0.15 0.10], ...
                    nnz(app.DenyMaskCache), 1);
            end
        end

        function updateStatus(app, msg)
            app.StatusBar.Text = msg;
            % The whole message on hover as well as on the panel. The label wraps and its
            % row is 'fit', so it should always be fully visible -- this is here because the
            % long refusals are exactly the messages an operator wants to re-read, and a
            % tooltip does not depend on the layout being right to deliver one.
            app.StatusBar.Tooltip = msg;
        end
    end
end
