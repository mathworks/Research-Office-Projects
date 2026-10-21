function app = launchVirtualLabApp()
%launchVirtualLabApp Launch a uifigure-based Virtual Lab prototype UI.
scriptDir = fileparts(mfilename("fullpath"));
addpath(scriptDir);
defaultSDKRoot = defaultSDKLocation(scriptDir);
ensureSDKOnPath(defaultSDKRoot);

artifactRecordTemplate = struct( ...
    "JsonPath", "", ...
    "MarkdownPath", "", ...
    "Status", "", ...
    "NumTurns", [], ...
    "ElapsedSeconds", []);

boardTurnTemplate = struct( ...
    "Agent", "", ...
    "DisplayAgent", "", ...
    "MetaText", "", ...
    "Message", "", ...
    "CardColor", "", ...
    "AccentColor", "");

phaseSummaryTemplate = struct( ...
    "PhaseName", "", ...
    "Agenda", "", ...
    "IterationText", "", ...
    "Status", "", ...
    "MergedSummary", "", ...
    "IterationSummaries", strings(1, 0), ...
    "IterationArtifacts", repmat(artifactRecordTemplate, 1, 0), ...
    "MergedArtifact", artifactRecordTemplate);

defaultConfigs = defaultAgentConfigs();
defaultEnvFile = defaultEnvLocation(scriptDir);
appSettings = readVirtualLabAppSettings(defaultVirtualLabAppSettingsPath(scriptDir));

state = struct( ...
    "TranscriptText", "", ...
    "EventLogText", "", ...
    "SummaryText", "", ...
    "ArtifactText", "", ...
    "StatusText", "Ready.", ...
    "LatestEvent", struct(), ...
    "Running", false, ...
    "Closing", false, ...
    "CancelRequested", false, ...
    "AgentConfigs", defaultConfigs, ...
    "SDKLocation", string(defaultSDKRoot), ...
    "SDKEnvPath", string(defaultEnvFile), ...
    "SDKProvider", "OpenAI", ...
    "SDKBaseURL", "", ...
    "SDKTestModel", "gpt-5-mini", ...
    "SDKConnectionStatus", "Not tested.", ...
    "SingleTeamLeadKey", "PrincipalInvestigator", ...
    "SingleTeamMemberKeys", ["Immunologist", "MachineLearningSpecialist", "ScientificCritic"], ...
    "SingleTeamMemberOrder", "fixed", ...
    "SingleIndividualKey", "MachineLearningSpecialist", ...
    "WorkflowPhases", defaultWorkflowPhaseConfigs(), ...
    "PhaseSummaries", repmat(phaseSummaryTemplate, 1, 0), ...
    "WorkflowIterationTarget", 0, ...
    "SelectedPhaseName", "", ...
    "BoardTurns", repmat(boardTurnTemplate, 1, 0), ...
    "BoardRevision", 0, ...
    "LastWorkflowPresetPath", string(appSettings.LastWorkflowPresetPath), ...
    "SavedRunPaths", strings(1,0), ...
    "LoadedRunPath", "");

configDialog = [];
configDialogControls = struct();
sdkDialog = [];
sdkDialogControls = struct();
workflowDialog = [];
workflowDialogControls = struct();

fig = uifigure( ...
    Name="Virtual Lab for MATLAB", ...
    Position=[100 100 1480 900], ...
    Color=[0.97 0.97 0.97], ...
    CloseRequestFcn=@(~,~) requestClose());

mainGrid = uigridlayout(fig, [3 3]);
mainGrid.RowHeight = {190, "1x", 64};
mainGrid.ColumnWidth = {230, "1x", 400};
mainGrid.Padding = [12 12 12 12];
mainGrid.RowSpacing = 10;
mainGrid.ColumnSpacing = 10;

topPanel = uipanel(mainGrid, Title="Controls");
topPanel.Layout.Row = 1;
topPanel.Layout.Column = [1 3];
topGrid = uigridlayout(topPanel, [3 11]);
topGrid.RowHeight = {22, "1x", 34};
topGrid.ColumnWidth = {"1x", 92, 102, 92, 102, 96, 118, 124, 128, 118, 168};

projectBriefLabel = uilabel(topGrid, Text="Project Brief / Agenda", FontWeight="bold");
projectBriefLabel.Layout.Row = 1;
projectBriefLabel.Layout.Column = [1 11];

projectBriefArea = uitextarea(topGrid, ...
    Value=defaultProjectBrief(), ...
    FontName="Consolas");
projectBriefArea.Layout.Row = 2;
projectBriefArea.Layout.Column = [1 5];

meetingTypeDropdown = uidropdown(topGrid, ...
    Items=["team", "individual"], ...
    Value="team", ...
    ValueChangedFcn=@(~,~) refreshMainMeetingTypeUI());
meetingTypeDropdown.Layout.Row = 2;
meetingTypeDropdown.Layout.Column = 6;

runModeLabel = uilabel(topGrid, Text="Run Mode", HorizontalAlignment="right");
runModeLabel.Layout.Row = 2;
runModeLabel.Layout.Column = 7;

runModeDropdown = uidropdown(topGrid, ...
    Items=["Use SDK", "Simulate"], ...
    Value="Simulate", ...
    Tooltip="Choose whether to simulate responses or make real SDK-backed model calls.");
runModeDropdown.Layout.Row = 2;
runModeDropdown.Layout.Column = 8;

configureAgentsButton = uibutton(topGrid, Text="Configure Agents", ...
    ButtonPushedFcn=@(~,~) openAgentConfigDialog());
configureAgentsButton.Layout.Row = 2;
configureAgentsButton.Layout.Column = 9;

sdkSettingsButton = uibutton(topGrid, Text="SDK Settings", ...
    ButtonPushedFcn=@(~,~) openSDKSettingsDialog());
sdkSettingsButton.Layout.Row = 2;
sdkSettingsButton.Layout.Column = 10;

configureWorkflowButton = uibutton(topGrid, Text="Configure Workflow", ...
    ButtonPushedFcn=@(~,~) openWorkflowConfigDialog());
configureWorkflowButton.Layout.Row = 2;
configureWorkflowButton.Layout.Column = 11;

numRoundsSpinner = uispinner(topGrid, Limits=[0 10], Value=2, Step=1);
numRoundsSpinner.Layout.Row = 3;
numRoundsSpinner.Layout.Column = 2;
roundsLabel = uilabel(topGrid, Text="Rounds", HorizontalAlignment="right");
roundsLabel.Layout.Row = 3;
roundsLabel.Layout.Column = 1;

numIterationsSpinner = uispinner(topGrid, Limits=[1 10], Value=2, Step=1);
numIterationsSpinner.Layout.Row = 3;
numIterationsSpinner.Layout.Column = 4;
iterationsLabel = uilabel(topGrid, Text="Iterations", HorizontalAlignment="right");
iterationsLabel.Layout.Row = 3;
iterationsLabel.Layout.Column = 3;

turnPauseSpinner = uispinner(topGrid, Limits=[0 2], Value=0.15, Step=0.05);
turnPauseSpinner.Layout.Row = 3;
turnPauseSpinner.Layout.Column = 6;
turnPauseLabel = uilabel(topGrid, Text="Turn Pause", HorizontalAlignment="right");
turnPauseLabel.Layout.Row = 3;
turnPauseLabel.Layout.Column = 5;

runMeetingButton = uibutton(topGrid, Text="Run Meeting", ...
    ButtonPushedFcn=@(~,~) runSingleMeeting());
runMeetingButton.Layout.Row = 3;
runMeetingButton.Layout.Column = 7;

runWorkflowButton = uibutton(topGrid, Text="Run Workflow", ...
    ButtonPushedFcn=@(~,~) runWorkflow());
runWorkflowButton.Layout.Row = 3;
runWorkflowButton.Layout.Column = 8;

teamMemberOrderLabel = uilabel(topGrid, Text="Member Order", HorizontalAlignment="right");
teamMemberOrderLabel.Layout.Row = 3;
teamMemberOrderLabel.Layout.Column = 9;

teamMemberOrderDropdown = uidropdown(topGrid, ...
    Items=teamMemberOrderItems(), ...
    Value=char(teamMemberOrderDisplayFromValue(state.SingleTeamMemberOrder)), ...
    ValueChangedFcn=@(~,~) handleSingleMeetingOrderChanged(), ...
    Tooltip="For team meetings, keep the chair fixed and either preserve or randomize the non-lead member order once per meeting.");
teamMemberOrderDropdown.Layout.Row = 3;
teamMemberOrderDropdown.Layout.Column = [10 11];

leftPanel = uipanel(mainGrid, Title="Workflow");
leftPanel.Layout.Row = 2;
leftPanel.Layout.Column = 1;
leftGrid = uigridlayout(leftPanel, [5 1]);
leftGrid.RowHeight = {22, 120, 22, 42, "1x"};

uilabel(leftGrid, Text="Phases", FontWeight="bold");
phaseNames = workflowPhaseNames(state.WorkflowPhases);
phaseListBox = uilistbox(leftGrid, ...
    Items=phaseNames, ...
    Value=phaseNames(1), ...
    ValueChangedFcn=@(~,~) handlePhaseListSelection());
phaseListBox.Layout.Row = 2;

savedRunsLabel = uilabel(leftGrid, Text="Saved Runs", FontWeight="bold");
savedRunsLabel.Layout.Row = 3;
savedRunsLabel.Layout.Column = 1;

savedRunsButtonGrid = uigridlayout(leftGrid, [1 2]);
savedRunsButtonGrid.RowHeight = {"1x"};
savedRunsButtonGrid.ColumnWidth = {"1x", "1x"};
savedRunsButtonGrid.Layout.Row = 4;

refreshRunsButton = uibutton(savedRunsButtonGrid, Text="Refresh List", ...
    ButtonPushedFcn=@(~,~) refreshSavedRuns());
refreshRunsButton.Layout.Row = 1;
refreshRunsButton.Layout.Column = 1;

loadRunButton = uibutton(savedRunsButtonGrid, Text="Open Run", ...
    ButtonPushedFcn=@(~,~) loadSelectedRun());
loadRunButton.Layout.Row = 1;
loadRunButton.Layout.Column = 2;

savedRunsListBox = uilistbox(leftGrid, Items="No saved runs found", Value="No saved runs found");
savedRunsListBox.Layout.Row = 5;

centerPanel = uipanel(mainGrid, Title="Meeting Workspace");
centerPanel.Layout.Row = 2;
centerPanel.Layout.Column = 2;
centerGrid = uigridlayout(centerPanel, [2 1]);
centerGrid.RowHeight = {22, "1x"};
uilabel(centerGrid, Text="Board, raw transcript, and event history", FontAngle="italic");

workspaceTabs = uitabgroup(centerGrid);
workspaceTabs.Layout.Row = 2;

boardTab = uitab(workspaceTabs, Title="Board");
boardTabGrid = uigridlayout(boardTab, [1 1]);
boardTabGrid.RowHeight = {"1x"};
boardTabGrid.ColumnWidth = {"1x"};
boardHTML = uihtml(boardTabGrid, ...
    HTMLSource=virtualLabBoardHTMLSource(scriptDir), ...
    Data=boardPayloadJSON(state.BoardTurns, state.BoardRevision));
boardHTML.Layout.Row = 1;
boardHTML.Layout.Column = 1;

transcriptTab = uitab(workspaceTabs, Title="Raw Transcript");
transcriptTabGrid = uigridlayout(transcriptTab, [1 1]);
transcriptTabGrid.RowHeight = {"1x"};
transcriptTabGrid.ColumnWidth = {"1x"};
transcriptArea = uitextarea(transcriptTabGrid, ...
    Editable="off", ...
    FontName="Consolas");
transcriptArea.Layout.Row = 1;
transcriptArea.Layout.Column = 1;

eventLogTab = uitab(workspaceTabs, Title="Event Log");
eventLogTabGrid = uigridlayout(eventLogTab, [1 1]);
eventLogTabGrid.RowHeight = {"1x"};
eventLogTabGrid.ColumnWidth = {"1x"};
eventLogArea = uitextarea(eventLogTabGrid, ...
    Editable="off", ...
    FontName="Consolas");
eventLogArea.Layout.Row = 1;
eventLogArea.Layout.Column = 1;

rightPanel = uipanel(mainGrid, Title="Context");
rightPanel.Layout.Row = 2;
rightPanel.Layout.Column = 3;
rightGrid = uigridlayout(rightPanel, [8 1]);
rightGrid.Scrollable = "on";
rightGrid.RowHeight = {22, 170, 22, 118, 22, 140, 22, 140};

uilabel(rightGrid, Text="Agenda", FontWeight="bold");
agendaArea = uitextarea(rightGrid, Editable="off", FontName="Consolas");
agendaArea.Layout.Row = 2;

phaseSummaryLabel = uilabel(rightGrid, Text="Workflow Summary", FontWeight="bold");
phaseSummaryLabel.Layout.Row = 3;
phaseSummaryLabel.Layout.Column = 1;
phaseSummaryTable = uitable(rightGrid, ...
    Data=cell(0,4), ...
    ColumnName={"Phase", "Iter", "State", "Merged Summary"}, ...
    ColumnEditable=[false false false false], ...
    ColumnWidth={88, 44, 68, 120}, ...
    RowName=[], ...
    CellSelectionCallback=@handlePhaseSummarySelection);
phaseSummaryTable.Layout.Row = 4;

summaryLabel = uilabel(rightGrid, Text="Phase Details", FontWeight="bold");
summaryLabel.Layout.Row = 5;
summaryLabel.Layout.Column = 1;
summaryArea = uitextarea(rightGrid, Editable="off", FontName="Consolas");
summaryArea.Layout.Row = 6;

artifactLabel = uilabel(rightGrid, Text="Artifacts", FontWeight="bold");
artifactLabel.Layout.Row = 7;
artifactLabel.Layout.Column = 1;
artifactArea = uitextarea(rightGrid, Editable="off", FontName="Consolas");
artifactArea.Layout.Row = 8;

bottomPanel = uipanel(mainGrid, Title="Status");
bottomPanel.Layout.Row = 3;
bottomPanel.Layout.Column = [1 3];
bottomGrid = uigridlayout(bottomPanel, [1 4]);
bottomGrid.ColumnWidth = {"1x", 100, 76, 76};

statusLabel = uilabel(bottomGrid, Text=state.StatusText, FontWeight="bold");
statusLabel.Layout.Row = 1;
statusLabel.Layout.Column = 1;

clearButton = uibutton(bottomGrid, Text="Clear Board", ...
    ButtonPushedFcn=@(~,~) resetView());
clearButton.Layout.Row = 1;
clearButton.Layout.Column = 2;

stopButton = uibutton(bottomGrid, Text="Stop", ...
    ButtonPushedFcn=@(~,~) requestStop(), ...
    Enable="off");
stopButton.Layout.Row = 1;
stopButton.Layout.Column = 3;

closeButton = uibutton(bottomGrid, Text="Close", ...
    ButtonPushedFcn=@(~,~) requestClose());
closeButton.Layout.Row = 1;
closeButton.Layout.Column = 4;

app = struct();
app.Figure = fig;
app.ProjectBriefArea = projectBriefArea;
app.MeetingTypeDropdown = meetingTypeDropdown;
app.TeamMemberOrderDropdown = teamMemberOrderDropdown;
app.RunModeDropdown = runModeDropdown;
app.ConfigureAgentsButton = configureAgentsButton;
app.SDKSettingsButton = sdkSettingsButton;
app.ConfigureWorkflowButton = configureWorkflowButton;
app.NumRoundsSpinner = numRoundsSpinner;
app.NumIterationsSpinner = numIterationsSpinner;
app.TurnPauseSpinner = turnPauseSpinner;
app.PhaseListBox = phaseListBox;
app.SavedRunsListBox = savedRunsListBox;
app.BoardGrid = boardHTML;
app.BoardHTML = boardHTML;
app.TranscriptArea = transcriptArea;
app.EventLogArea = eventLogArea;
app.PhaseSummaryTable = phaseSummaryTable;
app.AgendaArea = agendaArea;
app.SummaryArea = summaryArea;
app.ArtifactArea = artifactArea;
app.StatusLabel = statusLabel;
app.Actions = struct( ...
    "RunMeeting", @runSingleMeeting, ...
    "RunWorkflow", @runWorkflow, ...
    "OpenAgentConfig", @openAgentConfigDialog, ...
    "OpenSDKSettings", @openSDKSettingsDialog, ...
    "OpenWorkflowConfig", @openWorkflowConfigDialog, ...
    "RequestStop", @requestStop, ...
    "RefreshRuns", @refreshSavedRuns, ...
    "LoadSelectedRun", @loadSelectedRun, ...
    "ResetView", @resetView, ...
    "Snapshot", @snapshotState);

refreshMainMeetingTypeUI();
resetView();
refreshSavedRuns();

    function runSingleMeeting()
        if state.Running || ~isUIAlive()
            return
        end

        resetView();
        setRunning(true, "Running single meeting...");

        try
            agents = configuredAgentMap();
            meetingType = string(meetingTypeDropdown.Value);
            agenda = normalizeInputText(projectBriefArea.Value);
            if strlength(strtrim(agenda)) == 0
                agenda = defaultProjectBrief();
            end

            saveDir = fullfile(scriptDir, "ui_runs", "single_meeting");
            saveName = "meeting_" + string(datetime("now", Format="yyyyMMdd_HHmmss"));
            commonArgs = { ...
                "SaveDir", string(saveDir), ...
                "SaveName", saveName, ...
                "AgendaQuestions", defaultAgendaQuestions(), ...
                "NumRounds", numRoundsSpinner.Value, ...
                "UseSDK", shouldUseSDKMode(), ...
                "DryRun", shouldSimulateRun(), ...
                "Provider", sdkProviderAPI(state.SDKProvider), ...
                "BaseURL", effectiveSDKBaseURL(state.SDKProvider, state.SDKBaseURL), ...
                "TurnPauseSeconds", turnPauseSpinner.Value, ...
                "OnEvent", @handleEvent, ...
                "ShouldCancel", @shouldCancel};

            if shouldUseSDKMode()
                ensureConfiguredSDKAvailable();
            end

            if meetingType == "team"
                teamLead = agentForKey(agents, state.SingleTeamLeadKey);
                teamMemberKeys = configuredTeamMemberKeys();
                if isempty(teamMemberKeys)
                    error("launchVirtualLabApp:MissingTeamMembers", ...
                        "Select at least one team member besides the team lead.");
                end
                [~, ~, meta] = runVirtualMeeting("team", agenda, ...
                    commonArgs{:}, ...
                    "TeamLead", teamLead, ...
                    "TeamMembers", agentsForKeys(agents, teamMemberKeys), ...
                    "TeamMemberOrder", state.SingleTeamMemberOrder);
            else
                individualAgent = agentForKey(agents, state.SingleIndividualKey);
                [~, ~, meta] = runVirtualMeeting("individual", agenda, ...
                    commonArgs{:}, ...
                    "TeamMember", individualAgent);
            end
            if isfield(meta, "Status") && string(meta.Status) == "cancelled"
                setRunning(false, "Single meeting cancelled.");
            else
                setRunning(false, "Single meeting complete.");
            end
        catch ME
            if isUIAlive()
                appendEventLog("ERROR: " + string(ME.message));
                setRunning(false, "Single meeting failed.");
                uialert(fig, ME.message, "Single Meeting Failed");
            end
        end
    end

    function runWorkflow()
        if state.Running || ~isUIAlive()
            return
        end

        resetView();
        setRunning(true, "Running workflow...");

        try
            projectBrief = normalizeInputText(projectBriefArea.Value);
            if strlength(strtrim(projectBrief)) == 0
                projectBrief = defaultProjectBrief();
            end

            if shouldUseSDKMode()
                ensureConfiguredSDKAvailable();
            end

            saveRoot = fullfile(scriptDir, "ui_runs", "workflow_" + string(datetime("now", Format="yyyyMMdd_HHmmss")));
            results = runVirtualLabWorkflow(projectBrief, ...
                "SaveRoot", string(saveRoot), ...
                "NumIterations", numIterationsSpinner.Value, ...
                "NumRounds", numRoundsSpinner.Value, ...
                "Agents", configuredWorkflowAgents(), ...
                "Phases", state.WorkflowPhases, ...
                "UseSDK", shouldUseSDKMode(), ...
                "DryRun", shouldSimulateRun(), ...
                "Provider", sdkProviderAPI(state.SDKProvider), ...
                "BaseURL", effectiveSDKBaseURL(state.SDKProvider, state.SDKBaseURL), ...
                "TurnPauseSeconds", turnPauseSpinner.Value, ...
                "OnEvent", @handleEvent, ...
                "ShouldCancel", @shouldCancel);

            if isfield(results, "Status") && string(results.Status) == "cancelled"
                setRunning(false, "Workflow cancelled.");
            else
                setRunning(false, "Workflow complete.");
            end
        catch ME
            if isUIAlive()
                appendEventLog("ERROR: " + string(ME.message));
                setRunning(false, "Workflow failed.");
                uialert(fig, ME.message, "Workflow Failed");
            end
        end
    end

    function handleEvent(event)
        if ~isUIAlive()
            return
        end
        state.LatestEvent = event;

        switch string(event.Type)
            case "workflow_started"
                phaseListBox.Items = string(event.PhaseNames);
                phaseListBox.Value = phaseListBox.Items(1);
                state.SelectedPhaseName = phaseListBox.Value;
                initializePhaseSummaries(event.PhaseNames, event.NumIterations);
                appendEventLog("Workflow started.");
                statusLabel.Text = "Workflow started.";
            case "phase_started"
                phaseListBox.Value = string(event.PhaseName);
                state.SelectedPhaseName = string(event.PhaseName);
                setPhaseSummary( ...
                    event.PhaseName, ...
                    "Agenda", event.Agenda, ...
                    "IterationText", iterationProgressText(0), ...
                    "Status", "Running");
                renderSelectedPhaseDetails();
                appendEventLog("Phase started: " + event.PhaseName);
                statusLabel.Text = "Phase: " + event.PhaseName;
            case "phase_iteration_started"
                setPhaseSummary(event.PhaseName, "Status", "Iterating");
                renderSelectedPhaseDetails();
                appendEventLog("Iteration " + string(event.IterationIndex) + " started for " + event.PhaseName + ".");
            case "phase_iteration_completed"
                if event.IterationIndex >= state.WorkflowIterationTarget
                    phaseStatus = "Merging";
                else
                    phaseStatus = "Iterating";
                end
                setPhaseSummary( ...
                    event.PhaseName, ...
                    "IterationText", iterationProgressText(event.IterationIndex), ...
                    "Status", phaseStatus);
                setPhaseIterationResult(event.PhaseName, event.IterationIndex, event.Summary, event.Meta);
                renderSelectedPhaseDetails();
                appendEventLog("Iteration " + string(event.IterationIndex) + " complete for " + event.PhaseName + ".");
            case "phase_merged"
                setPhaseSummary( ...
                    event.PhaseName, ...
                    "IterationText", iterationProgressText(state.WorkflowIterationTarget), ...
                    "Status", "Merged", ...
                    "MergedSummary", event.MergedSummary);
                setPhaseMergedResult(event.PhaseName, event.MergedSummary, event.MergedMeta);
                renderSelectedPhaseDetails();
                appendEventLog("Merged summary complete for " + event.PhaseName + ".");
            case "workflow_cancelled"
                markIncompletePhasesCancelled();
                appendEventLog("Workflow cancelled.");
                statusLabel.Text = "Workflow cancelled.";
            case "workflow_completed"
                finalizePhaseSummaries();
                appendEventLog("Workflow completed.");
                statusLabel.Text = "Workflow complete.";
            case "meeting_started"
                clearBoardCards();
                agendaArea.Value = asLines(event.Agenda);
                appendEventLog("Meeting started: " + event.MeetingType);
                if isfield(event, "SpeakingOrder") && ~isempty(event.SpeakingOrder)
                    appendEventLog("Speaking order: " + join(string(event.SpeakingOrder), " -> "));
                end
                statusLabel.Text = "Meeting started: " + event.MeetingType;
            case "turn_added"
                if shouldShowBoardTurn(event)
                    addTurnCard(event);
                end
                appendTranscript(formatTurn(event));
            case "meeting_saved"
                if isWorkflowMeetingEvent(event)
                    setPhaseArtifactPaths(event.PhaseName, event.IterationIndex, event.IsMerge, event.JsonPath, event.MarkdownPath);
                    renderSelectedPhaseDetails();
                else
                    artifactArea.Value = asLines( ...
                        "JSON: " + string(event.JsonPath) + newline + ...
                        "Markdown: " + string(event.MarkdownPath));
                end
                appendEventLog("Transcript saved.");
                refreshSavedRuns();
            case "meeting_cancelled"
                appendEventLog("Meeting cancelled.");
                statusLabel.Text = "Meeting cancelled.";
            case "meeting_completed"
                if isWorkflowMeetingEvent(event)
                    if event.IsMerge
                        setPhaseMergedResult(event.PhaseName, event.Summary, event.Meta);
                    elseif event.IterationIndex > 0
                        setPhaseIterationResult(event.PhaseName, event.IterationIndex, event.Summary, event.Meta);
                    end
                    renderSelectedPhaseDetails();
                else
                    summaryArea.Value = asLines(event.Summary);
                end
                if isfield(event, "Meta") && isfield(event.Meta, "Status") && string(event.Meta.Status) == "cancelled"
                    appendEventLog("Meeting cancelled.");
                    statusLabel.Text = "Meeting cancelled.";
                else
                    appendEventLog("Meeting completed.");
                    statusLabel.Text = "Meeting completed.";
                end
        end
    end

    function appendTranscript(text)
        if ~isUIAlive()
            return
        end
        if state.TranscriptText == ""
            state.TranscriptText = text;
        else
            state.TranscriptText = state.TranscriptText + newline + newline + text;
        end
        refreshTranscriptView();
    end

    function appendEventLog(text)
        if ~isUIAlive()
            return
        end
        stamped = "[" + string(datetime("now", Format="HH:mm:ss")) + "] " + text;
        if state.EventLogText == ""
            state.EventLogText = stamped;
        else
            state.EventLogText = state.EventLogText + newline + stamped;
        end
        refreshEventLogView();
    end

    function refreshTranscriptView()
        if ~isUIAlive()
            return
        end
        transcriptArea.Value = asLines(state.TranscriptText);
    end

    function refreshEventLogView()
        if ~isUIAlive()
            return
        end
        eventLogArea.Value = asLines(state.EventLogText);
    end

    function initializePhaseSummaries(phaseNames, numIterations)
        state.WorkflowIterationTarget = double(numIterations);
        if isempty(phaseNames)
            state.PhaseSummaries = repmat(phaseSummaryTemplate, 1, 0);
        else
            state.PhaseSummaries = repmat(phaseSummaryTemplate, 1, numel(phaseNames));
            for idx = 1:numel(phaseNames)
                state.PhaseSummaries(idx).PhaseName = string(phaseNames(idx));
                state.PhaseSummaries(idx).Status = "Pending";
            end
        end
        refreshPhaseSummaryTable();
    end

    function setPhaseSummary(phaseName, varargin)
        rowIndex = ensurePhaseSummaryRow(phaseName);
        for idx = 1:2:numel(varargin)
            fieldName = char(string(varargin{idx}));
            fieldValue = varargin{idx + 1};
            state.PhaseSummaries(rowIndex).(fieldName) = string(fieldValue);
        end
        refreshPhaseSummaryTable();
    end

    function setPhaseIterationResult(phaseName, iterationIndex, summary, meta)
        if nargin < 4
            meta = struct();
        end
        rowIndex = ensurePhaseSummaryRow(phaseName);
        ensurePhaseIterationStorage(rowIndex, iterationIndex);
        state.PhaseSummaries(rowIndex).IterationSummaries(iterationIndex) = string(summary);
        state.PhaseSummaries(rowIndex).IterationArtifacts(iterationIndex) = mergeArtifactMeta( ...
            state.PhaseSummaries(rowIndex).IterationArtifacts(iterationIndex), meta);
        refreshPhaseSummaryTable();
    end

    function setPhaseMergedResult(phaseName, summary, meta)
        if nargin < 3
            meta = struct();
        end
        rowIndex = ensurePhaseSummaryRow(phaseName);
        state.PhaseSummaries(rowIndex).MergedSummary = string(summary);
        state.PhaseSummaries(rowIndex).MergedArtifact = mergeArtifactMeta( ...
            state.PhaseSummaries(rowIndex).MergedArtifact, meta);
        refreshPhaseSummaryTable();
    end

    function setPhaseArtifactPaths(phaseName, iterationIndex, isMerge, jsonPath, markdownPath)
        rowIndex = ensurePhaseSummaryRow(phaseName);
        if isMerge
            state.PhaseSummaries(rowIndex).MergedArtifact.JsonPath = string(jsonPath);
            state.PhaseSummaries(rowIndex).MergedArtifact.MarkdownPath = string(markdownPath);
        elseif iterationIndex > 0
            ensurePhaseIterationStorage(rowIndex, iterationIndex);
            state.PhaseSummaries(rowIndex).IterationArtifacts(iterationIndex).JsonPath = string(jsonPath);
            state.PhaseSummaries(rowIndex).IterationArtifacts(iterationIndex).MarkdownPath = string(markdownPath);
        end
    end

    function ensurePhaseIterationStorage(rowIndex, iterationIndex)
        currentCount = numel(state.PhaseSummaries(rowIndex).IterationSummaries);
        if currentCount < iterationIndex
            state.PhaseSummaries(rowIndex).IterationSummaries(currentCount + 1:iterationIndex) = "";
        end

        artifactCount = numel(state.PhaseSummaries(rowIndex).IterationArtifacts);
        if artifactCount < iterationIndex
            template = state.PhaseSummaries(rowIndex).MergedArtifact;
            for expandIndex = artifactCount + 1:iterationIndex
                state.PhaseSummaries(rowIndex).IterationArtifacts(expandIndex) = template;
            end
        end
    end

    function rowIndex = ensurePhaseSummaryRow(phaseName)
        phaseName = string(phaseName);
        rowIndex = find(string({state.PhaseSummaries.PhaseName}) == phaseName, 1, "first");
        if isempty(rowIndex)
            rowIndex = numel(state.PhaseSummaries) + 1;
            state.PhaseSummaries(rowIndex) = phaseSummaryTemplate;
            state.PhaseSummaries(rowIndex).PhaseName = phaseName;
        end
    end

    function refreshPhaseSummaryTable()
        if ~isUIAlive()
            return
        end

        if isempty(state.PhaseSummaries)
            phaseSummaryTable.Data = cell(0,4);
            return
        end

        tableData = cell(numel(state.PhaseSummaries), 4);
        for idx = 1:numel(state.PhaseSummaries)
            row = state.PhaseSummaries(idx);
            tableData{idx, 1} = char(row.PhaseName);
            tableData{idx, 2} = char(row.IterationText);
            tableData{idx, 3} = char(row.Status);
            tableData{idx, 4} = char(compactText(row.MergedSummary, 72));
        end
        phaseSummaryTable.Data = tableData;
    end

    function handlePhaseSummarySelection(~, event)
        if ~isUIAlive() || isempty(event.Indices)
            return
        end

        rowIndex = event.Indices(1);
        if rowIndex < 1 || rowIndex > numel(state.PhaseSummaries)
            return
        end

        phaseData = state.PhaseSummaries(rowIndex);
        state.SelectedPhaseName = string(phaseData.PhaseName);
        if any(string(phaseListBox.Items) == string(phaseData.PhaseName))
            phaseListBox.Value = string(phaseData.PhaseName);
        end
        renderPhaseDetails(phaseData);
    end

    function handlePhaseListSelection()
        if ~isUIAlive()
            return
        end
        selectedPhaseName = string(phaseListBox.Value);
        if strlength(selectedPhaseName) == 0
            return
        end
        state.SelectedPhaseName = selectedPhaseName;
        renderSelectedPhaseDetails();
    end

    function renderPhaseDetailsByName(phaseName)
        rowIndex = find(string({state.PhaseSummaries.PhaseName}) == string(phaseName), 1, "first");
        if isempty(rowIndex)
            return
        end
        renderPhaseDetails(state.PhaseSummaries(rowIndex));
    end

    function renderSelectedPhaseDetails()
        if strlength(state.SelectedPhaseName) == 0
            return
        end
        renderPhaseDetailsByName(state.SelectedPhaseName);
    end

    function renderPhaseDetails(phaseData)
        if ~isUIAlive()
            return
        end

        if strlength(phaseData.Agenda) > 0
            agendaArea.Value = asLines(phaseData.Agenda);
        end
        summaryArea.Value = asLines(formatPhaseDetailText(phaseData));
        artifactArea.Value = asLines(formatPhaseArtifactText(phaseData));
    end

    function finalizePhaseSummaries()
        for idx = 1:numel(state.PhaseSummaries)
            if strlength(state.PhaseSummaries(idx).PhaseName) > 0
                state.PhaseSummaries(idx).Status = "Complete";
            end
        end
        refreshPhaseSummaryTable();
    end

    function markIncompletePhasesCancelled()
        for idx = 1:numel(state.PhaseSummaries)
            statusText = string(state.PhaseSummaries(idx).Status);
            if any(statusText == ["Running", "Iterating", "Merging"])
                state.PhaseSummaries(idx).Status = "Cancelled";
            end
        end
        refreshPhaseSummaryTable();
    end

    function text = iterationProgressText(completedIterations)
        if state.WorkflowIterationTarget <= 0
            text = "";
        else
            text = string(completedIterations) + "/" + string(state.WorkflowIterationTarget);
        end
    end

    function resetView()
        if ~isUIAlive()
            return
        end
        state.TranscriptText = "";
        state.EventLogText = "";
        state.SummaryText = "";
        state.ArtifactText = "";
        state.StatusText = "Ready.";
        state.CancelRequested = false;
        state.PhaseSummaries = repmat(phaseSummaryTemplate, 1, 0);
        state.WorkflowIterationTarget = 0;
        state.SelectedPhaseName = "";
        state.LoadedRunPath = "";
        clearBoardCards();
        refreshTranscriptView();
        refreshEventLogView();
        refreshPhaseSummaryTable();
        agendaArea.Value = asLines(normalizeInputText(projectBriefArea.Value));
        summaryArea.Value = asLines("");
        artifactArea.Value = asLines("");
        statusLabel.Text = state.StatusText;
        phaseListBox.Items = workflowPhaseNames(state.WorkflowPhases);
        phaseListBox.Value = phaseListBox.Items(1);
        workspaceTabs.SelectedTab = boardTab;
        refreshSavedRuns();
    end

    function setRunning(tf, statusText)
        if ~isUIAlive()
            return
        end
        state.Running = tf;
        if tf
            enabledState = "off";
            stopState = "on";
        else
            enabledState = "on";
            stopState = "off";
        end
        runMeetingButton.Enable = enabledState;
        runWorkflowButton.Enable = enabledState;
        configureAgentsButton.Enable = enabledState;
        sdkSettingsButton.Enable = enabledState;
        configureWorkflowButton.Enable = enabledState;
        clearButton.Enable = enabledState;
        stopButton.Enable = stopState;
        refreshRunsButton.Enable = enabledState;
        if isempty(state.SavedRunPaths)
            loadRunButton.Enable = "off";
        else
            loadRunButton.Enable = enabledState;
        end
        statusLabel.Text = statusText;
        drawnow limitrate
    end

    function refreshMainMeetingTypeUI()
        isTeam = string(meetingTypeDropdown.Value) == "team";
        teamMemberOrderLabel.Visible = ternaryVisible(isTeam);
        teamMemberOrderDropdown.Visible = ternaryVisible(isTeam);
    end

    function handleSingleMeetingOrderChanged()
        state.SingleTeamMemberOrder = teamMemberOrderValueFromDisplay(string(teamMemberOrderDropdown.Value));
    end

    function tf = isUIAlive()
        tf = ~state.Closing && isvalid(fig);
    end

    function requestClose()
        if state.Closing
            return
        end
        state.CancelRequested = true;
        state.Closing = true;
        closeAgentConfigDialog();
        closeSDKSettingsDialog();
        closeWorkflowConfigDialog();
        if isvalid(fig)
            delete(fig);
        end
    end

    function dialogStruct = openAgentConfigDialog()
        dialogStruct = struct();
        if state.Running || ~isUIAlive()
            return
        end

        if ~isempty(configDialog) && isvalid(configDialog)
            dialogStruct = configDialogControls;
            return
        end

        configDialog = uifigure( ...
            Name="Agent Configuration", ...
            Position=[160 140 1120 720], ...
            WindowStyle="modal", ...
            CloseRequestFcn=@(~,~) closeAgentConfigDialog());

        selectedAgentRow = 1;

        dialogGrid = uigridlayout(configDialog, [5 1]);
        dialogGrid.RowHeight = {180, 180, "1x", 24, 40};
        dialogGrid.Padding = [12 12 12 12];
        dialogGrid.RowSpacing = 10;

        participantsPanel = uipanel(dialogGrid, Title="Meeting Participants");
        participantsPanel.Layout.Row = 1;
        participantsGrid = uigridlayout(participantsPanel, [3 2]);
        participantsGrid.RowHeight = {22, "1x", 22};
        participantsGrid.ColumnWidth = {150, "1x"};

        teamLeadLabel = uilabel(participantsGrid, Text="Team Lead (Chair)", FontWeight="bold");
        teamLeadLabel.Layout.Row = 1;
        teamLeadLabel.Layout.Column = 1;
        teamLeadDropdown = uidropdown(participantsGrid, ...
            Items=agentTitlesFromConfigs(state.AgentConfigs), ...
            Value=agentTitleForKey(state.AgentConfigs, state.SingleTeamLeadKey), ...
            Tooltip="For team meetings, the selected team lead is the visible chair that opens, steers, and summarizes the meeting.");
        teamLeadDropdown.Layout.Row = 1;
        teamLeadDropdown.Layout.Column = 2;

        teamMembersLabel = uilabel(participantsGrid, Text="Team Members", FontWeight="bold");
        teamMembersLabel.Layout.Row = 2;
        teamMembersLabel.Layout.Column = 1;
        teamMembersListBox = uilistbox(participantsGrid, ...
            Items=agentTitlesFromConfigs(state.AgentConfigs), ...
            Value=agentTitlesForKeys(state.AgentConfigs, state.SingleTeamMemberKeys), ...
            Multiselect="on");
        teamMembersListBox.Layout.Row = 2;
        teamMembersListBox.Layout.Column = 2;

        individualLabel = uilabel(participantsGrid, Text="Individual Meeting", FontWeight="bold");
        individualLabel.Layout.Row = 3;
        individualLabel.Layout.Column = 1;
        individualDropdown = uidropdown(participantsGrid, ...
            Items=agentTitlesFromConfigs(state.AgentConfigs), ...
            Value=agentTitleForKey(state.AgentConfigs, state.SingleIndividualKey));
        individualDropdown.Layout.Row = 3;
        individualDropdown.Layout.Column = 2;

        agentTable = uitable(dialogGrid, ...
            Data=agentConfigTableData(state.AgentConfigs), ...
            ColumnName={"Title", "Model", "Expertise", "Goal", "Role"}, ...
            ColumnEditable=[false false false false false], ...
            ColumnWidth={170, 110, 220, 240, 280}, ...
            RowName=[], ...
            CellSelectionCallback=@handleAgentTableSelection);
        agentTable.Layout.Row = 2;

        editorPanel = uipanel(dialogGrid, Title="Selected Agent");
        editorPanel.Layout.Row = 3;
        editorGrid = uigridlayout(editorPanel, [5 2]);
        editorGrid.RowHeight = {22, 28, "1x", "1x", "1x"};
        editorGrid.ColumnWidth = {90, "1x"};

        selectedAgentLabel = uilabel(editorGrid, Text="Title", FontWeight="bold");
        selectedAgentLabel.Layout.Row = 1;
        selectedAgentLabel.Layout.Column = 1;
        selectedAgentValue = uilabel(editorGrid, Text="", FontWeight="bold");
        selectedAgentValue.Layout.Row = 1;
        selectedAgentValue.Layout.Column = 2;

        modelEditorLabel = uilabel(editorGrid, Text="Model");
        modelEditorLabel.Layout.Row = 2;
        modelEditorLabel.Layout.Column = 1;
        modelEditor = uieditfield(editorGrid, "text", Value="");
        modelEditor.Layout.Row = 2;
        modelEditor.Layout.Column = 2;

        expertiseEditorLabel = uilabel(editorGrid, Text="Expertise");
        expertiseEditorLabel.Layout.Row = 3;
        expertiseEditorLabel.Layout.Column = 1;
        expertiseEditor = uitextarea(editorGrid, Value={""}, FontName="Consolas");
        expertiseEditor.Layout.Row = 3;
        expertiseEditor.Layout.Column = 2;

        goalEditorLabel = uilabel(editorGrid, Text="Goal");
        goalEditorLabel.Layout.Row = 4;
        goalEditorLabel.Layout.Column = 1;
        goalEditor = uitextarea(editorGrid, Value={""}, FontName="Consolas");
        goalEditor.Layout.Row = 4;
        goalEditor.Layout.Column = 2;

        roleEditorLabel = uilabel(editorGrid, Text="Role");
        roleEditorLabel.Layout.Row = 5;
        roleEditorLabel.Layout.Column = 1;
        roleEditor = uitextarea(editorGrid, Value={""}, FontName="Consolas");
        roleEditor.Layout.Row = 5;
        roleEditor.Layout.Column = 2;

        infoLabel = uilabel(dialogGrid, ...
            Text="For team meetings, the selected Team Lead is the visible chair. Select an agent row, edit its fields below, then click Apply. Workflow phase assignments remain fixed by phase.", ...
            FontAngle="italic");
        infoLabel.Layout.Row = 4;

        buttonGrid = uigridlayout(dialogGrid, [1 4]);
        buttonGrid.Layout.Row = 5;
        buttonGrid.ColumnWidth = {150, 140, "1x", 120};

        resetDefaultsButton = uibutton(buttonGrid, Text="Reset Defaults", ...
            ButtonPushedFcn=@(~,~) resetAgentConfigDialogControls());
        resetDefaultsButton.Layout.Row = 1;
        resetDefaultsButton.Layout.Column = 1;

        applyAgentButton = uibutton(buttonGrid, Text="Apply Agent Edit", ...
            ButtonPushedFcn=@(~,~) applySelectedAgentEdits());
        applyAgentButton.Layout.Row = 1;
        applyAgentButton.Layout.Column = 2;

        saveConfigButton = uibutton(buttonGrid, Text="Save", ...
            ButtonPushedFcn=@(~,~) saveAgentConfigDialog());
        saveConfigButton.Layout.Row = 1;
        saveConfigButton.Layout.Column = 3;
        saveConfigButton.HorizontalAlignment = "right";

        cancelConfigButton = uibutton(buttonGrid, Text="Cancel", ...
            ButtonPushedFcn=@(~,~) closeAgentConfigDialog());
        cancelConfigButton.Layout.Row = 1;
        cancelConfigButton.Layout.Column = 4;

        configDialogControls = struct( ...
            "Figure", configDialog, ...
            "TeamLeadDropdown", teamLeadDropdown, ...
            "TeamMembersListBox", teamMembersListBox, ...
            "IndividualDropdown", individualDropdown, ...
            "AgentTable", agentTable, ...
            "SelectedAgentValue", selectedAgentValue, ...
            "ModelEditor", modelEditor, ...
            "ExpertiseEditor", expertiseEditor, ...
            "GoalEditor", goalEditor, ...
            "RoleEditor", roleEditor, ...
            "ResetDefaultsButton", resetDefaultsButton, ...
            "ApplyAgentButton", applyAgentButton, ...
            "SaveButton", saveConfigButton, ...
            "CancelButton", cancelConfigButton);
        dialogStruct = configDialogControls;
        loadSelectedAgentIntoEditors(selectedAgentRow);

        function resetAgentConfigDialogControls()
            defaults = defaultAgentConfigs();
            teamLeadDropdown.Items = agentTitlesFromConfigs(defaults);
            teamLeadDropdown.Value = agentTitleForKey(defaults, "PrincipalInvestigator");
            teamMembersListBox.Items = agentTitlesFromConfigs(defaults);
            teamMembersListBox.Value = agentTitlesForKeys(defaults, ["Immunologist", "MachineLearningSpecialist", "ScientificCritic"]);
            individualDropdown.Items = agentTitlesFromConfigs(defaults);
            individualDropdown.Value = agentTitleForKey(defaults, "MachineLearningSpecialist");
            agentTable.Data = agentConfigTableData(defaults);
            selectedAgentRow = 1;
            loadSelectedAgentIntoEditors(selectedAgentRow);
        end

        function handleAgentTableSelection(~, event)
            if isempty(event.Indices)
                return
            end
            selectedAgentRow = event.Indices(1);
            loadSelectedAgentIntoEditors(selectedAgentRow);
        end

        function loadSelectedAgentIntoEditors(rowIndex)
            if rowIndex < 1 || rowIndex > size(agentTable.Data, 1)
                return
            end

            selectedAgentValue.Text = string(agentTable.Data{rowIndex, 1});
            modelEditor.Value = string(agentTable.Data{rowIndex, 2});
            expertiseEditor.Value = asLines(string(agentTable.Data{rowIndex, 3}));
            goalEditor.Value = asLines(string(agentTable.Data{rowIndex, 4}));
            roleEditor.Value = asLines(string(agentTable.Data{rowIndex, 5}));
        end

        function applySelectedAgentEdits()
            rowIndex = selectedAgentRow;
            if rowIndex < 1 || rowIndex > size(agentTable.Data, 1)
                return
            end

            agentTable.Data{rowIndex, 2} = char(string(modelEditor.Value));
            agentTable.Data{rowIndex, 3} = char(normalizeInputText(expertiseEditor.Value));
            agentTable.Data{rowIndex, 4} = char(normalizeInputText(goalEditor.Value));
            agentTable.Data{rowIndex, 5} = char(normalizeInputText(roleEditor.Value));
        end

        function saveAgentConfigDialog()
            applySelectedAgentEdits();
            updatedConfigs = tableDataToAgentConfigs(agentTable.Data, state.AgentConfigs);
            leadKey = agentKeyForTitle(updatedConfigs, string(teamLeadDropdown.Value));
            memberKeys = agentKeysForTitles(updatedConfigs, string(teamMembersListBox.Value));
            individualKey = agentKeyForTitle(updatedConfigs, string(individualDropdown.Value));

            if strlength(leadKey) == 0
                uialert(configDialog, "Select a valid team lead.", "Invalid Agent Configuration");
                return
            end
            if strlength(individualKey) == 0
                uialert(configDialog, "Select a valid individual meeting agent.", "Invalid Agent Configuration");
                return
            end
            memberKeys = memberKeys(memberKeys ~= leadKey);
            if isempty(memberKeys)
                uialert(configDialog, "Select at least one team member besides the team lead.", "Invalid Agent Configuration");
                return
            end

            state.AgentConfigs = updatedConfigs;
            state.SingleTeamLeadKey = leadKey;
            state.SingleTeamMemberKeys = memberKeys;
            state.SingleIndividualKey = individualKey;
            appendEventLog("Agent configuration updated.");
            closeAgentConfigDialog();
        end
    end

    function closeAgentConfigDialog()
        dialogHandle = configDialog;
        configDialog = [];
        if ~isempty(dialogHandle) && isvalid(dialogHandle)
            delete(dialogHandle);
        end
        configDialogControls = struct();
    end

    function dialogStruct = openSDKSettingsDialog()
        dialogStruct = struct();
        if state.Running || ~isUIAlive()
            return
        end

        if ~isempty(sdkDialog) && isvalid(sdkDialog)
            dialogStruct = sdkDialogControls;
            return
        end

        sdkDialog = uifigure( ...
            Name="SDK Settings", ...
            Position=[220 180 760 610], ...
            WindowStyle="modal", ...
            CloseRequestFcn=@(~,~) closeSDKSettingsDialog());

        dialogGrid = uigridlayout(sdkDialog, [8 2]);
        dialogGrid.RowHeight = {28, 28, 28, 28, 28, 90, "1x", 40};
        dialogGrid.ColumnWidth = {120, "1x"};
        dialogGrid.Padding = [12 12 12 12];
        dialogGrid.RowSpacing = 10;

        providerLabel = uilabel(dialogGrid, Text="Provider", FontWeight="bold");
        providerLabel.Layout.Row = 1;
        providerLabel.Layout.Column = 1;
        providerDropdown = uidropdown(dialogGrid, ...
            Items=sdkProviderItems(), ...
            Value=state.SDKProvider, ...
            ValueChangedFcn=@(~,~) refreshSDKProviderUI());
        providerDropdown.Layout.Row = 1;
        providerDropdown.Layout.Column = 2;

        locationLabel = uilabel(dialogGrid, Text="SDK Location", FontWeight="bold");
        locationLabel.Layout.Row = 2;
        locationLabel.Layout.Column = 1;
        locationField = uieditfield(dialogGrid, "text", Value=char(state.SDKLocation));
        locationField.Layout.Row = 2;
        locationField.Layout.Column = 2;

        envPathLabel = uilabel(dialogGrid, Text=".env File", FontWeight="bold");
        envPathLabel.Layout.Row = 3;
        envPathLabel.Layout.Column = 1;
        envPathField = uieditfield(dialogGrid, "text", Value=char(state.SDKEnvPath));
        envPathField.Layout.Row = 3;
        envPathField.Layout.Column = 2;

        baseURLLabel = uilabel(dialogGrid, Text="Base URL", FontWeight="bold");
        baseURLLabel.Layout.Row = 4;
        baseURLLabel.Layout.Column = 1;
        baseURLField = uieditfield(dialogGrid, "text", Value=char(state.SDKBaseURL));
        baseURLField.Layout.Row = 4;
        baseURLField.Layout.Column = 2;

        testModelLabel = uilabel(dialogGrid, Text="Test Model", FontWeight="bold");
        testModelLabel.Layout.Row = 5;
        testModelLabel.Layout.Column = 1;
        testModelField = uieditfield(dialogGrid, "text", Value=char(state.SDKTestModel));
        testModelField.Layout.Row = 5;
        testModelField.Layout.Column = 2;

        hintLabel = uilabel(dialogGrid, Text="Current Config", FontWeight="bold");
        hintLabel.Layout.Row = 6;
        hintLabel.Layout.Column = 1;
        hintArea = uitextarea(dialogGrid, Editable="off", FontName="Consolas");
        hintArea.Layout.Row = 6;
        hintArea.Layout.Column = 2;

        statusLabelLocal = uilabel(dialogGrid, Text="Connection Status", FontWeight="bold");
        statusLabelLocal.Layout.Row = 7;
        statusLabelLocal.Layout.Column = 1;
        statusArea = uitextarea(dialogGrid, Editable="off", FontName="Consolas");
        statusArea.Layout.Row = 7;
        statusArea.Layout.Column = 2;

        buttonGrid = uigridlayout(dialogGrid, [1 5]);
        buttonGrid.Layout.Row = 8;
        buttonGrid.Layout.Column = [1 2];
        buttonGrid.ColumnWidth = {90, 120, 110, "1x", 120};

        loadEnvButton = uibutton(buttonGrid, Text="Load Env", ...
            ButtonPushedFcn=@(~,~) loadSDKEnvironment());
        loadEnvButton.Layout.Row = 1;
        loadEnvButton.Layout.Column = 1;

        testButton = uibutton(buttonGrid, Text="Test Connection", ...
            ButtonPushedFcn=@(~,~) testSDKConnection());
        testButton.Layout.Row = 1;
        testButton.Layout.Column = 2;

        saveButton = uibutton(buttonGrid, Text="Save", ...
            ButtonPushedFcn=@(~,~) saveSDKSettings());
        saveButton.Layout.Row = 1;
        saveButton.Layout.Column = 3;

        closeButtonLocal = uibutton(buttonGrid, Text="Close", ...
            ButtonPushedFcn=@(~,~) closeSDKSettingsDialog());
        closeButtonLocal.Layout.Row = 1;
        closeButtonLocal.Layout.Column = 5;

        sdkDialogControls = struct( ...
            "Figure", sdkDialog, ...
            "ProviderDropdown", providerDropdown, ...
            "LocationField", locationField, ...
            "EnvPathField", envPathField, ...
            "BaseURLField", baseURLField, ...
            "TestModelField", testModelField, ...
            "HintArea", hintArea, ...
            "StatusArea", statusArea, ...
            "LoadEnvButton", loadEnvButton, ...
            "TestButton", testButton, ...
            "SaveButton", saveButton, ...
            "CloseButton", closeButtonLocal);
        dialogStruct = sdkDialogControls;

        refreshSDKProviderUI();
        refreshSDKStatus();

        function refreshSDKProviderUI()
            providerValue = string(providerDropdown.Value);
            if sdkProviderUsesBaseURL(providerValue)
                dialogGrid.RowHeight = {28, 28, 28, 28, 28, 90, "1x", 40};
                baseURLLabel.Visible = "on";
                baseURLField.Visible = "on";
            else
                dialogGrid.RowHeight = {28, 28, 28, 0, 28, 90, "1x", 40};
                baseURLLabel.Visible = "off";
                baseURLField.Visible = "off";
            end
            hintArea.Value = asLines(sdkProviderHint(string(providerDropdown.Value)));
        end

        function refreshSDKStatus()
            statusArea.Value = asLines(state.SDKConnectionStatus);
        end

        function saveSDKSettings()
            locationValue = string(strtrim(locationField.Value));
            if strlength(locationValue) == 0
                locationValue = string(defaultSDKLocation(scriptDir));
                locationField.Value = char(locationValue);
            end
            if ~isfolder(locationValue)
                uialert(sdkDialog, "SDK location does not exist.", "Invalid SDK Location");
                return
            end
            envPathValue = string(strtrim(envPathField.Value));
            if strlength(envPathValue) == 0
                envPathValue = string(defaultEnvLocation(scriptDir));
                envPathField.Value = char(envPathValue);
            end

            state.SDKProvider = string(providerDropdown.Value);
            state.SDKLocation = locationValue;
            state.SDKEnvPath = envPathValue;
            if sdkProviderUsesBaseURL(state.SDKProvider)
                state.SDKBaseURL = string(strtrim(baseURLField.Value));
            else
                state.SDKBaseURL = "";
            end
            modelValue = string(strtrim(testModelField.Value));
            if strlength(modelValue) == 0
                modelValue = "gpt-5-mini";
                testModelField.Value = char(modelValue);
            end
            state.SDKTestModel = modelValue;
            ensureSDKOnPath(state.SDKLocation);
            appendEventLog("SDK settings updated: provider=" + state.SDKProvider + ", test model=" + state.SDKTestModel + ".");
            refreshSDKStatus();
        end

        function loadSDKEnvironment()
            envPathValue = string(strtrim(envPathField.Value));
            if strlength(envPathValue) == 0
                envPathValue = string(defaultEnvLocation(scriptDir));
                envPathField.Value = char(envPathValue);
            end
            if ~isfile(envPathValue)
                uialert(sdkDialog, ".env file does not exist.", "Missing .env File");
                return
            end

            try
                loadenv(char(envPathValue));
                state.SDKEnvPath = envPathValue;
                state.SDKConnectionStatus = ".env loaded successfully." + newline + ...
                    "File: " + envPathValue;
                statusArea.Value = asLines(state.SDKConnectionStatus);
                appendEventLog("Loaded SDK environment from " + envPathValue + ".");
            catch ME
                state.SDKConnectionStatus = ".env load failed." + newline + ...
                    "File: " + envPathValue + newline + ...
                    "Error: " + string(ME.message);
                statusArea.Value = asLines(state.SDKConnectionStatus);
            end
        end

        function testSDKConnection()
            provider = string(providerDropdown.Value);
            providerAPI = sdkProviderAPI(provider);
            locationValue = string(strtrim(locationField.Value));
            baseURLValue = string(strtrim(baseURLField.Value));
            modelValue = string(strtrim(testModelField.Value));
            if strlength(locationValue) == 0
                locationValue = string(defaultSDKLocation(scriptDir));
                locationField.Value = char(locationValue);
            end
            if ~isfolder(locationValue)
                uialert(sdkDialog, "SDK location does not exist.", "Invalid SDK Location");
                return
            end
            if strlength(modelValue) == 0
                uialert(sdkDialog, "Provide a test model before running the SDK connection test.", "Missing Test Model");
                return
            end
            if provider == "OpenAI Compatible" && strlength(baseURLValue) == 0
                uialert(sdkDialog, "Provide a Base URL for OpenAI Compatible endpoints.", "Missing Base URL");
                return
            end

            statusArea.Value = asLines("Testing " + provider + " with model " + modelValue + "...");
            drawnow
            try
                ensureSDKOnPath(locationValue);
                clientArgs = sdkConnectionClientArgs(providerAPI, modelValue, 0, baseURLValue);
                client = aisdk.LLMClient(providerAPI, modelValue, clientArgs{:});
                agent = aisdk.AIAgent(client, ...
                    SystemPrompt="Reply with the single word OK.", ...
                    DisplayMode="off", ...
                    MaxIterations=5);
                response = string(run(agent, "Reply with OK only."));
                state.SDKConnectionStatus = "Connected successfully." + newline + ...
                    "Provider: " + provider + newline + ...
                    "SDK Location: " + locationValue + newline + ...
                    "Model: " + modelValue + newline + ...
                    "Response preview: " + compactText(response, 80);
                statusArea.Value = asLines(state.SDKConnectionStatus);
            catch ME
                state.SDKConnectionStatus = "Connection test failed." + newline + ...
                    "Provider: " + provider + newline + ...
                    "SDK Location: " + locationValue + newline + ...
                    "Model: " + modelValue + newline + ...
                    "Error: " + string(ME.message);
                statusArea.Value = asLines(state.SDKConnectionStatus);
            end
        end
    end

    function closeSDKSettingsDialog()
        dialogHandle = sdkDialog;
        sdkDialog = [];
        if ~isempty(dialogHandle) && isvalid(dialogHandle)
            delete(dialogHandle);
        end
        sdkDialogControls = struct();
    end

    function dialogStruct = openWorkflowConfigDialog()
        dialogStruct = struct();
        if state.Running || ~isUIAlive()
            return
        end

        if ~isempty(workflowDialog) && isvalid(workflowDialog)
            dialogStruct = workflowDialogControls;
            return
        end

        phases = state.WorkflowPhases;
        selectedPhaseRow = 1;
        presetPaths = discoverWorkflowPresets(defaultWorkflowPresetLocation(scriptDir));

        workflowDialog = uifigure( ...
            Name="Workflow Configuration", ...
            Position=[180 120 1180 760], ...
            WindowStyle="modal", ...
            CloseRequestFcn=@(~,~) closeWorkflowConfigDialog());

        dialogGrid = uigridlayout(workflowDialog, [5 1]);
        dialogGrid.RowHeight = {180, "1x", 34, 24, 40};
        dialogGrid.Padding = [12 12 12 12];
        dialogGrid.RowSpacing = 10;

        phaseTable = uitable(dialogGrid, ...
            Data=workflowPhaseTableData(phases, state.AgentConfigs), ...
            ColumnName={"Phase", "Type", "Order", "Participants"}, ...
            ColumnEditable=[false false false false], ...
            ColumnWidth={140, 80, 110, 540}, ...
            RowName=[], ...
            CellSelectionCallback=@handleWorkflowPhaseSelection);
        phaseTable.Layout.Row = 1;

        editorPanel = uipanel(dialogGrid, Title="Selected Phase");
        editorPanel.Layout.Row = 2;
        editorGrid = uigridlayout(editorPanel, [9 2]);
        editorGrid.RowHeight = {22, 28, 28, 110, 28, "1x", 90, 70, 70};
        editorGrid.ColumnWidth = {120, "1x"};

        phaseNameLabel = uilabel(editorGrid, Text="Phase Name", FontWeight="bold");
        phaseNameLabel.Layout.Row = 1;
        phaseNameLabel.Layout.Column = 1;
        phaseNameField = uieditfield(editorGrid, "text", Value="");
        phaseNameField.Layout.Row = 1;
        phaseNameField.Layout.Column = 2;

        meetingTypeLabel = uilabel(editorGrid, Text="Meeting Type", FontWeight="bold");
        meetingTypeLabel.Layout.Row = 2;
        meetingTypeLabel.Layout.Column = 1;
        meetingTypeDropdown = uidropdown(editorGrid, ...
            Items=["team", "individual"], ...
            Value="team", ...
            ValueChangedFcn=@(~,~) refreshWorkflowMeetingTypeUI());
        meetingTypeDropdown.Layout.Row = 2;
        meetingTypeDropdown.Layout.Column = 2;

        teamLeadLabel = uilabel(editorGrid, Text="Team Lead (Chair)");
        teamLeadLabel.Layout.Row = 3;
        teamLeadLabel.Layout.Column = 1;
        teamLeadDropdown = uidropdown(editorGrid, ...
            Items=agentTitlesFromConfigs(state.AgentConfigs), ...
            Tooltip="For team phases, the selected team lead is the visible chair for that phase.");
        teamLeadDropdown.Layout.Row = 3;
        teamLeadDropdown.Layout.Column = 2;

        teamMembersLabel = uilabel(editorGrid, Text="Team Members");
        teamMembersLabel.Layout.Row = 4;
        teamMembersLabel.Layout.Column = 1;
        teamMembersListBox = uilistbox(editorGrid, ...
            Items=agentTitlesFromConfigs(state.AgentConfigs), ...
            Multiselect="on");
        teamMembersListBox.Layout.Row = 4;
        teamMembersListBox.Layout.Column = 2;

        phaseTeamMemberOrderLabel = uilabel(editorGrid, Text="Member Order");
        phaseTeamMemberOrderLabel.Layout.Row = 5;
        phaseTeamMemberOrderLabel.Layout.Column = 1;
        phaseTeamMemberOrderDropdown = uidropdown(editorGrid, ...
            Items=teamMemberOrderItems(), ...
            Value=char(teamMemberOrderDisplayFromValue("fixed")), ...
            Tooltip="Keep the chair fixed and either preserve or randomize the non-lead member order once per meeting.");
        phaseTeamMemberOrderDropdown.Layout.Row = 5;
        phaseTeamMemberOrderDropdown.Layout.Column = 2;

        individualLabel = uilabel(editorGrid, Text="Individual Agent");
        individualLabel.Layout.Row = 3;
        individualLabel.Layout.Column = 1;
        individualDropdown = uidropdown(editorGrid, Items=agentTitlesFromConfigs(state.AgentConfigs));
        individualDropdown.Layout.Row = 3;
        individualDropdown.Layout.Column = 2;

        agendaLabel = uilabel(editorGrid, Text="Agenda Template", FontWeight="bold");
        agendaLabel.Layout.Row = 6;
        agendaLabel.Layout.Column = 1;
        agendaAreaEditor = uitextarea(editorGrid, Value={""}, FontName="Consolas");
        agendaAreaEditor.Layout.Row = 6;
        agendaAreaEditor.Layout.Column = 2;

        questionsLabel = uilabel(editorGrid, Text="Questions");
        questionsLabel.Layout.Row = 7;
        questionsLabel.Layout.Column = 1;
        questionsAreaEditor = uitextarea(editorGrid, Value={""}, FontName="Consolas");
        questionsAreaEditor.Layout.Row = 7;
        questionsAreaEditor.Layout.Column = 2;

        rulesLabel = uilabel(editorGrid, Text="Rules");
        rulesLabel.Layout.Row = 8;
        rulesLabel.Layout.Column = 1;
        rulesAreaEditor = uitextarea(editorGrid, Value={""}, FontName="Consolas");
        rulesAreaEditor.Layout.Row = 8;
        rulesAreaEditor.Layout.Column = 2;

        contextsLabel = uilabel(editorGrid, Text="Contexts");
        contextsLabel.Layout.Row = 9;
        contextsLabel.Layout.Column = 1;
        contextsAreaEditor = uitextarea(editorGrid, Value={""}, FontName="Consolas");
        contextsAreaEditor.Layout.Row = 9;
        contextsAreaEditor.Layout.Column = 2;

        presetDisplayNames = workflowPresetDisplayNames(presetPaths);

        presetGrid = uigridlayout(dialogGrid, [1 8]);
        presetGrid.Layout.Row = 3;
        presetGrid.ColumnWidth = {48, "1x", 90, 170, 90, 90, 80, 80};
        presetGrid.Padding = [0 0 0 0];

        presetLabel = uilabel(presetGrid, Text="Preset", FontWeight="bold");
        presetLabel.Layout.Row = 1;
        presetLabel.Layout.Column = 1;

        presetDropdown = uidropdown(presetGrid, ...
            Items=presetDisplayNames, ...
            ValueChangedFcn=@(~,~) handlePresetSelection());
        presetDropdown.Layout.Row = 1;
        presetDropdown.Layout.Column = 2;

        refreshPresetButton = uibutton(presetGrid, Text="Refresh", ...
            ButtonPushedFcn=@(~,~) refreshPresetControls());
        refreshPresetButton.Layout.Row = 1;
        refreshPresetButton.Layout.Column = 3;

        presetNameField = uieditfield(presetGrid, "text", Value="");
        presetNameField.Layout.Row = 1;
        presetNameField.Layout.Column = 4;

        savePresetButton = uibutton(presetGrid, Text="Save Preset", ...
            ButtonPushedFcn=@(~,~) saveWorkflowPreset());
        savePresetButton.Layout.Row = 1;
        savePresetButton.Layout.Column = 5;

        loadPresetButton = uibutton(presetGrid, Text="Load Preset", ...
            ButtonPushedFcn=@(~,~) loadWorkflowPreset());
        loadPresetButton.Layout.Row = 1;
        loadPresetButton.Layout.Column = 6;

        renamePresetButton = uibutton(presetGrid, Text="Rename", ...
            ButtonPushedFcn=@(~,~) renameWorkflowPreset());
        renamePresetButton.Layout.Row = 1;
        renamePresetButton.Layout.Column = 7;

        deletePresetButton = uibutton(presetGrid, Text="Delete", ...
            ButtonPushedFcn=@(~,~) deleteWorkflowPreset());
        deletePresetButton.Layout.Row = 1;
        deletePresetButton.Layout.Column = 8;

        infoLabel = uilabel(dialogGrid, ...
            Text="Use {{PROJECT_BRIEF}} in agenda, questions, rules, or contexts to inject the current project brief at run time. For team phases, the selected Team Lead is the visible chair.", ...
            FontAngle="italic");
        infoLabel.Layout.Row = 4;

        buttonGrid = uigridlayout(dialogGrid, [1 6]);
        buttonGrid.Layout.Row = 5;
        buttonGrid.ColumnWidth = {100, 110, 120, 120, "1x", 120};

        addPhaseButton = uibutton(buttonGrid, Text="Add Phase", ...
            ButtonPushedFcn=@(~,~) addWorkflowPhase());
        addPhaseButton.Layout.Row = 1;
        addPhaseButton.Layout.Column = 1;

        removePhaseButton = uibutton(buttonGrid, Text="Remove Phase", ...
            ButtonPushedFcn=@(~,~) removeWorkflowPhase());
        removePhaseButton.Layout.Row = 1;
        removePhaseButton.Layout.Column = 2;

        resetPhasesButton = uibutton(buttonGrid, Text="Reset Defaults", ...
            ButtonPushedFcn=@(~,~) resetWorkflowPhases());
        resetPhasesButton.Layout.Row = 1;
        resetPhasesButton.Layout.Column = 3;

        applyPhaseButton = uibutton(buttonGrid, Text="Apply Phase", ...
            ButtonPushedFcn=@(~,~) applySelectedWorkflowPhase());
        applyPhaseButton.Layout.Row = 1;
        applyPhaseButton.Layout.Column = 4;

        saveWorkflowButton = uibutton(buttonGrid, Text="Save", ...
            ButtonPushedFcn=@(~,~) saveWorkflowConfig());
        saveWorkflowButton.Layout.Row = 1;
        saveWorkflowButton.Layout.Column = 5;
        saveWorkflowButton.HorizontalAlignment = "right";

        closeWorkflowButton = uibutton(buttonGrid, Text="Close", ...
            ButtonPushedFcn=@(~,~) closeWorkflowConfigDialog());
        closeWorkflowButton.Layout.Row = 1;
        closeWorkflowButton.Layout.Column = 6;

        workflowDialogControls = struct( ...
            "Figure", workflowDialog, ...
            "PhaseTable", phaseTable, ...
            "PhaseNameField", phaseNameField, ...
            "MeetingTypeDropdown", meetingTypeDropdown, ...
            "TeamLeadDropdown", teamLeadDropdown, ...
            "TeamMembersListBox", teamMembersListBox, ...
            "TeamMemberOrderDropdown", phaseTeamMemberOrderDropdown, ...
            "IndividualDropdown", individualDropdown, ...
            "AgendaArea", agendaAreaEditor, ...
            "QuestionsArea", questionsAreaEditor, ...
            "RulesArea", rulesAreaEditor, ...
            "ContextsArea", contextsAreaEditor, ...
            "PresetDropdown", presetDropdown, ...
            "PresetNameField", presetNameField, ...
            "RefreshPresetButton", refreshPresetButton, ...
            "SavePresetButton", savePresetButton, ...
            "LoadPresetButton", loadPresetButton, ...
            "RenamePresetButton", renamePresetButton, ...
            "DeletePresetButton", deletePresetButton, ...
            "AddPhaseButton", addPhaseButton, ...
            "RemovePhaseButton", removePhaseButton, ...
            "ResetDefaultsButton", resetPhasesButton, ...
            "ApplyPhaseButton", applyPhaseButton, ...
            "SaveButton", saveWorkflowButton, ...
            "CloseButton", closeWorkflowButton);
        dialogStruct = workflowDialogControls;

        refreshWorkflowPhaseTable();
        loadWorkflowPhaseIntoEditors(selectedPhaseRow);
        refreshPresetControls(state.LastWorkflowPresetPath);

        function refreshWorkflowPhaseTable()
            phaseTable.Data = workflowPhaseTableData(phases, state.AgentConfigs);
        end

        function handlePresetSelection()
            [selectedIndex, selectedPath] = selectedPresetIndex();
            presetItems = string(presetDropdown.Items);
            if isempty(presetItems) || presetItems(1) == "No saved presets" || isempty(selectedIndex)
                presetNameField.Value = "";
            else
                presetNameField.Value = char(string(presetDropdown.Value));
                persistLastWorkflowPresetPath(selectedPath);
            end
        end

        function refreshPresetControls(preferredPresetPath)
            if nargin < 1
                preferredPresetPath = "";
            end
            presetPaths = discoverWorkflowPresets(defaultWorkflowPresetLocation(scriptDir));
            presetDisplayNames = workflowPresetDisplayNames(presetPaths);
            presetItems = presetDisplayNames;
            presetDropdown.Items = presetItems;
            if isempty(presetPaths)
                presetDropdown.Value = presetItems(1);
                presetNameField.Value = "";
                loadPresetButton.Enable = "off";
                renamePresetButton.Enable = "off";
                deletePresetButton.Enable = "off";
                persistLastWorkflowPresetPath("");
            else
                preferredPresetPath = string(preferredPresetPath);
                if strlength(preferredPresetPath) == 0
                    preferredPresetPath = state.LastWorkflowPresetPath;
                end
                selectedIndex = find(presetPaths == preferredPresetPath, 1, "first");
                if isempty(selectedIndex)
                    selectedIndex = 1;
                end
                if any(presetItems == string(presetDropdown.Value)) && isempty(preferredPresetPath)
                    presetDropdown.Value = char(string(presetDropdown.Value));
                else
                    presetDropdown.Value = char(presetItems(selectedIndex));
                end
                presetNameField.Value = char(string(presetDropdown.Value));
                loadPresetButton.Enable = "on";
                renamePresetButton.Enable = "on";
                deletePresetButton.Enable = "on";
                persistLastWorkflowPresetPath(presetPaths(selectedIndex));
            end
        end

        function [selectedIndex, selectedPath] = selectedPresetIndex()
            selectedIndex = [];
            selectedPath = "";
            if isempty(presetPaths)
                return
            end
            selectedDisplay = string(presetDropdown.Value);
            selectedIndex = find(presetDisplayNames == selectedDisplay, 1, "first");
            if isempty(selectedIndex)
                return
            end
            selectedPath = presetPaths(selectedIndex);
        end

        function persistLastWorkflowPresetPath(presetPath)
            state.LastWorkflowPresetPath = string(presetPath);
            writeVirtualLabAppSettings(defaultVirtualLabAppSettingsPath(scriptDir), ...
                struct("LastWorkflowPresetPath", state.LastWorkflowPresetPath));
        end

        function handleWorkflowPhaseSelection(~, event)
            if isempty(event.Indices)
                return
            end
            selectedPhaseRow = event.Indices(1);
            loadWorkflowPhaseIntoEditors(selectedPhaseRow);
        end

        function loadWorkflowPhaseIntoEditors(rowIndex)
            if rowIndex < 1 || rowIndex > numel(phases)
                return
            end
            phase = phases(rowIndex);
            phaseNameField.Value = char(phase.Name);
            meetingTypeDropdown.Value = char(phase.MeetingType);
            if phase.MeetingType == "team"
                teamLeadDropdown.Value = char(agentTitleForKey(state.AgentConfigs, phase.TeamLead));
                teamMembersListBox.Value = agentTitlesForKeys(state.AgentConfigs, phase.TeamMembers);
                phaseTeamMemberOrderDropdown.Value = char(teamMemberOrderDisplayFromValue(phase.TeamMemberOrder));
                individualDropdown.Value = char(agentTitleForKey(state.AgentConfigs, state.SingleIndividualKey));
            else
                individualDropdown.Value = char(agentTitleForKey(state.AgentConfigs, phase.TeamMember));
                if strlength(agentTitleForKey(state.AgentConfigs, state.SingleTeamLeadKey)) > 0
                    teamLeadDropdown.Value = char(agentTitleForKey(state.AgentConfigs, state.SingleTeamLeadKey));
                end
                teamMembersListBox.Value = agentTitlesForKeys(state.AgentConfigs, state.SingleTeamMemberKeys);
                phaseTeamMemberOrderDropdown.Value = char(teamMemberOrderDisplayFromValue("fixed"));
            end
            agendaAreaEditor.Value = asLines(phase.Agenda);
            questionsAreaEditor.Value = asLines(join(string(phase.AgendaQuestions), newline));
            rulesAreaEditor.Value = asLines(join(string(phase.AgendaRules), newline));
            contextsAreaEditor.Value = asLines(join(string(phase.Contexts), newline));
            refreshWorkflowMeetingTypeUI();
        end

        function refreshWorkflowMeetingTypeUI()
            isTeam = string(meetingTypeDropdown.Value) == "team";
            teamLeadLabel.Visible = ternaryVisible(isTeam);
            teamLeadDropdown.Visible = ternaryVisible(isTeam);
            teamMembersLabel.Visible = ternaryVisible(isTeam);
            teamMembersListBox.Visible = ternaryVisible(isTeam);
            phaseTeamMemberOrderLabel.Visible = ternaryVisible(isTeam);
            phaseTeamMemberOrderDropdown.Visible = ternaryVisible(isTeam);
            individualLabel.Visible = ternaryVisible(~isTeam);
            individualDropdown.Visible = ternaryVisible(~isTeam);
            if isTeam
                editorGrid.RowHeight = {22, 28, 28, 110, 28, "1x", 90, 70, 70};
            else
                editorGrid.RowHeight = {22, 28, 0, 0, 0, "1x", 90, 70, 70};
            end
        end

        function tf = applySelectedWorkflowPhase()
            tf = false;
            rowIndex = selectedPhaseRow;
            if rowIndex < 1 || rowIndex > numel(phases)
                return
            end

            updatedPhase = phases(rowIndex);
            updatedPhase.Name = string(strtrim(phaseNameField.Value));
            updatedPhase.MeetingType = string(meetingTypeDropdown.Value);
            updatedPhase.Agenda = normalizeInputText(agendaAreaEditor.Value);
            updatedPhase.AgendaQuestions = nonemptyLines(questionsAreaEditor.Value);
            updatedPhase.AgendaRules = nonemptyLines(rulesAreaEditor.Value);
            updatedPhase.Contexts = nonemptyLines(contextsAreaEditor.Value);

            if strlength(updatedPhase.Name) == 0
                uialert(workflowDialog, "Phase name cannot be empty.", "Invalid Workflow Phase");
                return
            end

            if updatedPhase.MeetingType == "team"
                updatedPhase.TeamLead = agentKeyForTitle(state.AgentConfigs, string(teamLeadDropdown.Value));
                updatedPhase.TeamMembers = agentKeysForTitles(state.AgentConfigs, string(teamMembersListBox.Value));
                updatedPhase.TeamMembers = updatedPhase.TeamMembers(updatedPhase.TeamMembers ~= updatedPhase.TeamLead);
                updatedPhase.TeamMember = "";
                updatedPhase.TeamMemberOrder = teamMemberOrderValueFromDisplay(string(phaseTeamMemberOrderDropdown.Value));
                if strlength(updatedPhase.TeamLead) == 0 || isempty(updatedPhase.TeamMembers)
                    uialert(workflowDialog, "Team phases require one lead and at least one team member.", "Invalid Workflow Phase");
                    return
                end
            else
                updatedPhase.TeamLead = "";
                updatedPhase.TeamMembers = strings(1, 0);
                updatedPhase.TeamMember = agentKeyForTitle(state.AgentConfigs, string(individualDropdown.Value));
                updatedPhase.TeamMemberOrder = "fixed";
                if strlength(updatedPhase.TeamMember) == 0
                    uialert(workflowDialog, "Individual phases require one selected agent.", "Invalid Workflow Phase");
                    return
                end
            end

            phases(rowIndex) = updatedPhase;
            refreshWorkflowPhaseTable();
            loadWorkflowPhaseIntoEditors(rowIndex);
            tf = true;
        end

        function addWorkflowPhase()
            if ~applySelectedWorkflowPhase()
                return
            end
            phases(end+1) = newWorkflowPhaseTemplate(numel(phases) + 1, state.SingleTeamLeadKey, state.SingleIndividualKey);
            selectedPhaseRow = numel(phases);
            refreshWorkflowPhaseTable();
            loadWorkflowPhaseIntoEditors(selectedPhaseRow);
        end

        function removeWorkflowPhase()
            if numel(phases) <= 1
                uialert(workflowDialog, "At least one workflow phase must remain.", "Cannot Remove Phase");
                return
            end
            phases(selectedPhaseRow) = [];
            selectedPhaseRow = min(selectedPhaseRow, numel(phases));
            refreshWorkflowPhaseTable();
            loadWorkflowPhaseIntoEditors(selectedPhaseRow);
        end

        function resetWorkflowPhases()
            phases = defaultWorkflowPhaseConfigs();
            selectedPhaseRow = 1;
            refreshWorkflowPhaseTable();
            loadWorkflowPhaseIntoEditors(selectedPhaseRow);
        end

        function saveWorkflowPreset()
            try
                if ~applySelectedWorkflowPhase()
                    return
                end

                requestedName = string(strtrim(presetNameField.Value));
                if strlength(requestedName) == 0
                    uialert(workflowDialog, "Enter a preset name before saving.", "Missing Preset Name");
                    return
                end

                sanitizedName = sanitizeWorkflowPresetName(requestedName);
                if strlength(sanitizedName) == 0
                    uialert(workflowDialog, "Preset name must contain at least one letter or number.", "Invalid Preset Name");
                    return
                end

                presetDir = defaultWorkflowPresetLocation(scriptDir);
                presetPath = fullfile(presetDir, sanitizedName + ".json");
                if isfile(presetPath)
                    overwriteChoice = uiconfirm(workflowDialog, ...
                        sprintf("Overwrite the existing preset '%s'?", sanitizedName), ...
                        "Overwrite Preset", ...
                        Options=["Overwrite", "Cancel"], ...
                        DefaultOption="Overwrite", ...
                        CancelOption="Cancel");
                    if overwriteChoice ~= "Overwrite"
                        return
                    end
                end

                writeWorkflowPreset(presetPath, requestedName, phases);
                refreshPresetControls(presetPath);
                presetNameField.Value = char(requestedName);
                appendEventLog("Workflow preset saved: " + requestedName);
            catch ME
                uialert(workflowDialog, ME.message, "Preset Save Failed");
            end
        end

        function loadWorkflowPreset()
            try
                if isempty(presetPaths)
                    return
                end

                [selectedIndex, selectedPath] = selectedPresetIndex();
                selectedPreset = string(presetDropdown.Value);
                if isempty(selectedIndex)
                    return
                end

                loadChoice = uiconfirm(workflowDialog, ...
                    sprintf("Replace the current workflow editor contents with preset '%s'?", selectedPreset), ...
                    "Load Workflow Preset", ...
                    Options=["Load", "Cancel"], ...
                    DefaultOption="Load", ...
                    CancelOption="Cancel");
                if loadChoice ~= "Load"
                    return
                end

                preset = readWorkflowPreset(selectedPath);
                phases = preset.Phases;
                selectedPhaseRow = 1;
                refreshWorkflowPhaseTable();
                loadWorkflowPhaseIntoEditors(selectedPhaseRow);
                presetNameField.Value = char(preset.PresetName);
                persistLastWorkflowPresetPath(selectedPath);
                appendEventLog("Workflow preset loaded: " + preset.PresetName);
            catch ME
                uialert(workflowDialog, ME.message, "Preset Load Failed");
            end
        end

        function renameWorkflowPreset()
            try
                [selectedIndex, selectedPath] = selectedPresetIndex();
                if isempty(selectedIndex)
                    return
                end

                requestedName = string(strtrim(presetNameField.Value));
                if strlength(requestedName) == 0
                    uialert(workflowDialog, "Enter a preset name before renaming.", "Missing Preset Name");
                    return
                end

                sanitizedName = sanitizeWorkflowPresetName(requestedName);
                if strlength(sanitizedName) == 0
                    uialert(workflowDialog, "Preset name must contain at least one letter or number.", "Invalid Preset Name");
                    return
                end

                [selectedDir, ~, ~] = fileparts(char(selectedPath));
                targetPath = string(fullfile(selectedDir, sanitizedName + ".json"));
                preset = readWorkflowPreset(selectedPath);
                if targetPath == selectedPath
                    writeWorkflowPreset(targetPath, requestedName, preset.Phases);
                    refreshPresetControls(targetPath);
                    presetNameField.Value = char(requestedName);
                    appendEventLog("Workflow preset updated: " + requestedName);
                    return
                end

                if isfile(targetPath)
                    uialert(workflowDialog, ...
                        sprintf("A preset named '%s' already exists.", sanitizedName), ...
                        "Rename Preset Failed");
                    return
                end

                writeWorkflowPreset(targetPath, requestedName, preset.Phases);
                deleteWorkflowPresetFile(selectedPath);
                refreshPresetControls(targetPath);
                presetNameField.Value = char(requestedName);
                appendEventLog("Workflow preset renamed: " + requestedName);
            catch ME
                uialert(workflowDialog, ME.message, "Rename Preset Failed");
            end
        end

        function deleteWorkflowPreset()
            try
                [selectedIndex, selectedPath] = selectedPresetIndex();
                if isempty(selectedIndex)
                    return
                end

                selectedPreset = string(presetDropdown.Value);
                deleteChoice = uiconfirm(workflowDialog, ...
                    sprintf("Delete the preset '%s'?", selectedPreset), ...
                    "Delete Workflow Preset", ...
                    Options=["Delete", "Cancel"], ...
                    DefaultOption="Cancel", ...
                    CancelOption="Cancel");
                if deleteChoice ~= "Delete"
                    return
                end

                deleteWorkflowPresetFile(selectedPath);
                refreshPresetControls("");
                appendEventLog("Workflow preset deleted: " + selectedPreset);
            catch ME
                uialert(workflowDialog, ME.message, "Delete Preset Failed");
            end
        end

        function saveWorkflowConfig()
            if ~applySelectedWorkflowPhase()
                return
            end
            state.WorkflowPhases = phases;
            phaseListBox.Items = workflowPhaseNames(state.WorkflowPhases);
            phaseListBox.Value = phaseListBox.Items(1);
            appendEventLog("Workflow configuration updated.");
            closeWorkflowConfigDialog();
        end
    end

    function closeWorkflowConfigDialog()
        dialogHandle = workflowDialog;
        workflowDialog = [];
        if ~isempty(dialogHandle) && isvalid(dialogHandle)
            delete(dialogHandle);
        end
        workflowDialogControls = struct();
    end

    function ensureConfiguredSDKAvailable()
        sdkLocation = string(strtrim(state.SDKLocation));
        if strlength(sdkLocation) == 0
            sdkLocation = string(defaultSDKLocation(scriptDir));
            state.SDKLocation = sdkLocation;
        end
        if state.SDKProvider == "OpenAI Compatible" && strlength(strtrim(state.SDKBaseURL)) == 0
            error("launchVirtualLabApp:MissingBaseURL", ...
                "OpenAI Compatible requires a Base URL in SDK Settings.");
        end
        if ~isfolder(sdkLocation)
            error("launchVirtualLabApp:MissingSDK", ...
                "Configured SDK location does not exist: %s", sdkLocation);
        end
        ensureSDKOnPath(sdkLocation);
        if isempty(which("aisdk.LLMClient"))
            error("launchVirtualLabApp:MissingSDK", ...
                "MATLAB AI Agent SDK was not found at: %s", sdkLocation);
        end
    end

    function agents = configuredAgentMap()
        agents = struct();
        for idx = 1:numel(state.AgentConfigs)
            config = state.AgentConfigs(idx);
            agents.(char(config.Key)) = virtualLabAgent( ...
                config.Title, ...
                config.Expertise, ...
                config.Goal, ...
                config.Role, ...
                config.Model);
        end
    end

    function agentArray = configuredWorkflowAgents()
        agentArray = repmat(agentStructFromConfig(state.AgentConfigs(1)), 1, numel(state.AgentConfigs));
        for idx = 1:numel(state.AgentConfigs)
            agentArray(idx) = agentStructFromConfig(state.AgentConfigs(idx));
        end
    end

    function keys = configuredTeamMemberKeys()
        keys = unique(string(state.SingleTeamMemberKeys), "stable");
        keys = keys(keys ~= string(state.SingleTeamLeadKey));
    end

    function agent = agentForKey(agentMap, key)
        key = char(string(key));
        if ~isfield(agentMap, key)
            error("launchVirtualLabApp:UnknownAgentKey", ...
                "Agent key '%s' is not configured.", key);
        end
        agent = agentMap.(key);
    end

    function agentArray = agentsForKeys(agentMap, keys)
        keys = string(keys);
        if isempty(keys)
            agentArray = struct.empty(1,0);
            return
        end

        agentArray = repmat(agentForKey(agentMap, keys(1)), 1, numel(keys));
        for idx = 1:numel(keys)
            agentArray(idx) = agentForKey(agentMap, keys(idx));
        end
    end

    function requestStop()
        state.CancelRequested = true;
        if isUIAlive()
            appendEventLog("Stop requested.");
            statusLabel.Text = "Stopping after current turn...";
        end
    end

    function tf = shouldCancel()
        tf = state.CancelRequested || ~isUIAlive();
    end

    function mode = currentRunMode()
        mode = string(runModeDropdown.Value);
    end

    function tf = shouldSimulateRun()
        tf = currentRunMode() == "Simulate";
    end

    function tf = shouldUseSDKMode()
        tf = currentRunMode() == "Use SDK";
    end

    function refreshSavedRuns()
        if ~isUIAlive()
            return
        end

        runPaths = discoverSavedRuns(scriptDir);
        state.SavedRunPaths = runPaths;

        if isempty(runPaths)
            savedRunsListBox.Items = "No saved runs found";
            savedRunsListBox.Value = "No saved runs found";
            loadRunButton.Enable = "off";
        else
            displayItems = displayNamesForRuns(scriptDir, runPaths);
            savedRunsListBox.Items = displayItems;
            savedRunsListBox.Value = displayItems(1);
            if ~state.Running
                loadRunButton.Enable = "on";
            end
        end
    end

    function loadSelectedRun()
        if ~isUIAlive() || isempty(state.SavedRunPaths)
            return
        end

        selectedDisplay = string(savedRunsListBox.Value);
        displayItems = string(savedRunsListBox.Items);
        selectedIndex = find(displayItems == selectedDisplay, 1, "first");
        if isempty(selectedIndex)
            return
        end

        loadTranscriptFromJson(state.SavedRunPaths(selectedIndex));
    end

    function loadTranscriptFromJson(jsonPath)
        if ~isUIAlive()
            return
        end

        transcript = jsondecode(fileread(jsonPath));
        resetView();
        state.LoadedRunPath = string(jsonPath);
        clearBoardCards();

        for idx = 1:numel(transcript)
            event = struct( ...
                "Agent", string(transcript(idx).agent), ...
                "Message", string(transcript(idx).message), ...
                "TurnKind", inferTurnKind(string(transcript(idx).agent)));
            if shouldShowBoardTurn(event)
                addTurnCard(event);
            end
            appendTranscript(formatTurn(event));
        end

        artifactArea.Value = asLines("Loaded JSON: " + string(jsonPath));
        summaryArea.Value = asLines(extractSummaryFromTranscript(transcript));
        statusLabel.Text = "Loaded saved run.";
        appendEventLog("Loaded saved run: " + string(jsonPath));
    end

    function snapshot = snapshotState()
        if ~isUIAlive()
            snapshot = struct( ...
                "TranscriptText", state.TranscriptText, ...
                "EventLogText", state.EventLogText, ...
                "Summary", "", ...
                "Artifacts", "", ...
                "Status", "closed", ...
                "CancelRequested", state.CancelRequested, ...
                "AgentConfigCount", numel(state.AgentConfigs), ...
                "SDKLocation", state.SDKLocation, ...
                "SDKEnvPath", state.SDKEnvPath, ...
                "SDKProvider", state.SDKProvider, ...
                "SDKBaseURL", state.SDKBaseURL, ...
                "SDKTestModel", state.SDKTestModel, ...
                "SDKConnectionStatus", state.SDKConnectionStatus, ...
                "RunMode", currentRunMode(), ...
                "SingleTeamLead", state.SingleTeamLeadKey, ...
                "SingleTeamMemberCount", numel(state.SingleTeamMemberKeys), ...
                "SingleTeamMemberOrder", state.SingleTeamMemberOrder, ...
                "SingleIndividual", state.SingleIndividualKey, ...
                "WorkflowPhaseCount", numel(state.WorkflowPhases), ...
                "PhaseSummaryRowCount", numel(state.PhaseSummaries), ...
                "BoardCardCount", numel(state.BoardTurns), ...
                "LoadedRunPath", state.LoadedRunPath);
            return
        end
        snapshot = struct( ...
            "TranscriptText", state.TranscriptText, ...
            "EventLogText", state.EventLogText, ...
            "Summary", normalizeInputText(summaryArea.Value), ...
            "Artifacts", normalizeInputText(artifactArea.Value), ...
            "Status", string(statusLabel.Text), ...
            "CancelRequested", state.CancelRequested, ...
            "AgentConfigCount", numel(state.AgentConfigs), ...
            "SDKLocation", state.SDKLocation, ...
            "SDKEnvPath", state.SDKEnvPath, ...
            "SDKProvider", state.SDKProvider, ...
            "SDKBaseURL", state.SDKBaseURL, ...
            "SDKTestModel", state.SDKTestModel, ...
            "SDKConnectionStatus", state.SDKConnectionStatus, ...
            "RunMode", currentRunMode(), ...
            "SingleTeamLead", state.SingleTeamLeadKey, ...
            "SingleTeamMemberCount", numel(state.SingleTeamMemberKeys), ...
            "SingleTeamMemberOrder", state.SingleTeamMemberOrder, ...
            "SingleIndividual", state.SingleIndividualKey, ...
            "WorkflowPhaseCount", numel(state.WorkflowPhases), ...
            "PhaseSummaryRowCount", numel(state.PhaseSummaries), ...
            "BoardCardCount", numel(state.BoardTurns), ...
            "LoadedRunPath", state.LoadedRunPath);
    end

    function clearBoardCards()
        if ~isUIAlive()
            return
        end
        state.BoardTurns = repmat(boardTurnTemplate, 1, 0);
        refreshBoardView();
    end

    function addTurnCard(event)
        if ~isUIAlive()
            return
        end
        state.BoardTurns(end+1) = boardTurnRecord(event);
        refreshBoardView();
    end

    function refreshBoardView()
        if ~isUIAlive()
            return
        end
        state.BoardRevision = state.BoardRevision + 1;
        boardHTML.Data = boardPayloadJSON(state.BoardTurns, state.BoardRevision);
        drawnow limitrate
    end

    function tf = shouldShowBoardTurn(event)
        tf = ~isControllerTurn(event);
    end
end

function text = formatTurn(event)
header = upper(displayAgentName(event.Agent));
if isfield(event, "TurnKind")
    header = header + " [" + upper(string(event.TurnKind)) + "]";
end
text = header + newline + string(event.Message);
end

function text = turnMetaText(event)
parts = strings(1,0);
if isfield(event, "TurnKind")
    parts(end+1) = upper(string(event.TurnKind));
end
if isfield(event, "RoundNumber") && ~isempty(event.RoundNumber)
    parts(end+1) = "Round " + string(event.RoundNumber);
end
if isfield(event, "IsMerge") && logical(event.IsMerge)
    parts(end+1) = "Merge";
end
if isfield(event, "PhaseName") && strlength(string(event.PhaseName)) > 0
    parts(end+1) = string(event.PhaseName);
end

if isempty(parts)
    text = "";
else
    text = join(parts, " | ");
    text = text(1);
end
end

function color = getCardColor(agent)
agent = string(agent);
switch agent
    case "User"
        color = [0.92 0.93 0.95];
    case "Principal Investigator"
        color = [0.96 0.92 0.82];
    case "Immunologist"
        color = [0.88 0.95 0.89];
    case "Machine Learning Specialist"
        color = [0.88 0.93 0.98];
    case "Computational Biologist"
        color = [0.87 0.94 0.96];
    case "Scientific Critic"
        color = [0.96 0.89 0.89];
    otherwise
        color = [0.94 0.94 0.95];
end
end

function color = getAccentColor(agent)
agent = string(agent);
switch agent
    case "User"
        color = [0.26 0.26 0.28];
    case "Principal Investigator"
        color = [0.46 0.39 0.26];
    case "Immunologist"
        color = [0.27 0.44 0.33];
    case "Machine Learning Specialist"
        color = [0.27 0.39 0.54];
    case "Computational Biologist"
        color = [0.26 0.42 0.46];
    case "Scientific Critic"
        color = [0.50 0.31 0.31];
    otherwise
        color = [0.26 0.26 0.28];
end
end

function htmlSource = virtualLabBoardHTMLSource(scriptDir)
htmlSource = fullfile(scriptDir, "board_ui.html");
if ~isfile(htmlSource)
    error("launchVirtualLabApp:MissingBoardHTML", ...
        "Board HTML source not found: %s", htmlSource);
end
end

function payloadText = boardPayloadJSON(turns, revision)
payload = struct( ...
    "revision", revision, ...
    "title", "Live Board", ...
    "subtitle", "Controller prompts stay hidden here while the raw transcript keeps them.", ...
    "emptyMessage", "Run a meeting, run a workflow, or load a saved transcript to populate the board.", ...
    "turns", boardPayloadTurns(turns));
payloadText = jsonencode(payload);
end

function payloadTurns = boardPayloadTurns(turns)
if isempty(turns)
    payloadTurns = cell(1, 0);
else
    payloadTurns = reshape(turns, 1, []);
end
end

function turn = boardTurnRecord(event)
turn = struct( ...
    "Agent", string(event.Agent), ...
    "DisplayAgent", displayAgentName(event.Agent), ...
    "MetaText", turnMetaText(event), ...
    "Message", string(event.Message), ...
    "CardColor", rgbToHex(getCardColor(event.Agent)), ...
    "AccentColor", rgbToHex(getAccentColor(event.Agent)));
end

function hex = rgbToHex(color)
color = max(0, min(255, round(double(color) .* 255)));
hex = "#" + join(compose("%02X", color), "");
hex = upper(hex(1));
end

function lines = asLines(text)
normalized = join(string(text), "");
normalized = normalized(1);
lines = cellstr(splitlines(normalized));
lines = reshape(lines, [], 1);
if isempty(lines)
    lines = {""};
end
end

function text = normalizeInputText(value)
if isstring(value) || ischar(value)
    text = string(value);
else
    text = join(string(value), newline);
end
text = join(string(text), "");
text = text(1);
end

function text = compactText(value, maxLength)
text = normalizeInputText(value);
text = regexprep(char(text), "\s+", " ");
text = string(strtrim(text));
if strlength(text) <= maxLength
    return
end
text = extractBefore(text, maxLength + 1) + "...";
end

function tf = isWorkflowMeetingEvent(event)
tf = isfield(event, "PhaseName") && strlength(string(event.PhaseName)) > 0;
end

function artifact = mergeArtifactMeta(artifact, meta)
if isfield(meta, "JsonPath") && strlength(string(meta.JsonPath)) > 0
    artifact.JsonPath = string(meta.JsonPath);
end
if isfield(meta, "MarkdownPath") && strlength(string(meta.MarkdownPath)) > 0
    artifact.MarkdownPath = string(meta.MarkdownPath);
end
if isfield(meta, "Status") && strlength(string(meta.Status)) > 0
    artifact.Status = string(meta.Status);
end
if isfield(meta, "NumTurns") && ~isempty(meta.NumTurns)
    artifact.NumTurns = meta.NumTurns;
end
if isfield(meta, "ElapsedSeconds") && ~isempty(meta.ElapsedSeconds)
    artifact.ElapsedSeconds = meta.ElapsedSeconds;
end
end

function text = formatPhaseDetailText(phaseData)
text = "Phase: " + string(phaseData.PhaseName);
text = text + newline + newline + "Status: " + string(phaseData.Status);

if strlength(string(phaseData.IterationText)) > 0
    text = text + newline + newline + "Iterations: " + string(phaseData.IterationText);
end

if strlength(string(phaseData.MergedSummary)) > 0
    text = text + newline + newline + "Merged Summary" + newline + newline + string(phaseData.MergedSummary);
else
    text = text + newline + newline + "Merged Summary" + newline + newline + "No merged summary yet.";
end

iterationSummaries = string(phaseData.IterationSummaries);
iterationSummaries = reshape(iterationSummaries, 1, []);
iterationSummaries = iterationSummaries(strlength(iterationSummaries) > 0);
if isempty(iterationSummaries)
    text = text + newline + newline + "Iteration Summaries" + newline + newline + "No completed iterations yet.";
else
    rawIterationSummaries = string(phaseData.IterationSummaries);
    iterationText = "";
    for idx = 1:numel(rawIterationSummaries)
        currentSummary = string(rawIterationSummaries(idx));
        if strlength(currentSummary) == 0
            continue
        end
        if strlength(iterationText) > 0
            iterationText = iterationText + newline + newline;
        end
        iterationText = iterationText + "Iteration " + string(idx) + newline + currentSummary;
    end
    text = text + newline + newline + "Iteration Summaries" + newline + newline + iterationText;
end
end

function text = formatPhaseArtifactText(phaseData)
parts = strings(1, 0);

mergedText = formatArtifactEntryText("Merged", phaseData.MergedArtifact);
if strlength(mergedText) > 0
    parts(end+1) = mergedText;
end

iterationArtifacts = phaseData.IterationArtifacts;
for idx = 1:numel(iterationArtifacts)
    currentText = formatArtifactEntryText("Iteration " + string(idx), iterationArtifacts(idx));
    if strlength(currentText) > 0
        parts(end+1) = currentText; %#ok<AGROW>
    end
end

if isempty(parts)
    text = "No saved artifacts yet.";
else
    text = join(parts, newline + newline);
    text = text(1);
end
end

function text = formatArtifactEntryText(label, artifact)
detailParts = strings(1, 0);
if strlength(string(artifact.Status)) > 0
    detailParts(end+1) = "Status: " + string(artifact.Status);
end
if ~isempty(artifact.NumTurns)
    detailParts(end+1) = "Turns: " + string(artifact.NumTurns);
end
if ~isempty(artifact.ElapsedSeconds)
    detailParts(end+1) = "Elapsed: " + string(round(double(artifact.ElapsedSeconds), 2)) + " s";
end
if strlength(string(artifact.JsonPath)) > 0
    detailParts(end+1) = "JSON: " + string(artifact.JsonPath);
end
if strlength(string(artifact.MarkdownPath)) > 0
    detailParts(end+1) = "Markdown: " + string(artifact.MarkdownPath);
end

if isempty(detailParts)
    text = "";
else
    text = string(label) + newline + join(detailParts, newline);
    text = text(1);
end
end

function items = sdkProviderItems()
items = ["OpenAI", "OpenAI Compatible", "Ollama"];
end

function api = sdkProviderAPI(provider)
provider = string(provider);
switch provider
    case {"OpenAI", "OpenAI Compatible"}
        api = "openai";
    case "Ollama"
        api = "ollama";
    otherwise
        error("launchVirtualLabApp:UnknownSDKProvider", ...
            "Unknown SDK provider: %s", provider);
end
end

function args = sdkConnectionClientArgs(providerAPI, modelValue, temperature, baseURLValue)
args = {};
if shouldSetSDKClientTemperature(providerAPI, modelValue)
    args = [args, {"Temperature", temperature}];
end
if strlength(baseURLValue) > 0
    args = [args, {"BaseURL", baseURLValue}];
end
end

function tf = shouldSetSDKClientTemperature(providerAPI, modelValue)
providerAPI = lower(string(providerAPI));
modelValue = lower(strtrim(string(modelValue)));
tf = true;
if providerAPI == "openai" && startsWith(modelValue, "gpt-5")
    tf = false;
end
end

function tf = sdkProviderUsesBaseURL(provider)
provider = string(provider);
tf = any(provider == ["OpenAI Compatible", "Ollama"]);
end

function baseURL = effectiveSDKBaseURL(provider, configuredBaseURL)
if sdkProviderUsesBaseURL(provider)
    baseURL = string(configuredBaseURL);
else
    baseURL = "";
end
end

function text = sdkProviderHint(provider)
provider = string(provider);
switch provider
    case "OpenAI"
        text = "Provider: OpenAI" + newline + ...
            "Expected environment variables:" + newline + ...
            "  OPENAI_API_KEY" + newline + ...
            "  OPENAI_API_ENDPOINT (optional)" + newline + newline + ...
            "Base URL is not shown here because the standard OpenAI endpoint is used by default. " + ...
            "Actual run models are still configured per agent.";
    case "OpenAI Compatible"
        text = "Provider: OpenAI Compatible" + newline + ...
            "SDK API path: openai" + newline + ...
            "Expected environment variables:" + newline + ...
            "  OPENAI_API_KEY" + newline + ...
            "  OPENAI_API_ENDPOINT (optional)" + newline + newline + ...
            "Set Base URL to the compatible API endpoint. " + ...
            "If the endpoint does not require an API key, the SDK README recommends OPENAI_API_KEY=EMPTY.";
    case "Ollama"
        text = "Provider: Ollama" + newline + ...
            "Expected environment variables:" + newline + ...
            "  OLLAMA_API_KEY (optional depending on server)" + newline + ...
            "  OLLAMA_API_ENDPOINT" + newline + newline + ...
            "Optional Base URL override can be set in SDK Settings. " + ...
            "Actual run models are still configured per agent.";
    otherwise
        text = "Unknown provider.";
end
end

function sdkRoot = defaultSDKLocation(scriptDir)
sdkRoot = fullfile(fileparts(scriptDir), "external", "matlab-ai-agent-sdk");
end

function envPath = defaultEnvLocation(scriptDir)
envPath = fullfile(fileparts(scriptDir), ".env");
end

function presetDir = defaultWorkflowPresetLocation(scriptDir)
presetDir = fullfile(scriptDir, "workflow_presets");
end

function settingsPath = defaultVirtualLabAppSettingsPath(scriptDir)
settingsPath = fullfile(scriptDir, ".virtual_lab_app_settings.json");
end

function ensureSDKOnPath(sdkRoot)
if isfolder(sdkRoot)
    addpath(char(sdkRoot));
end
end

function brief = defaultProjectBrief()
brief = "Develop a MATLAB-centered virtual lab workflow for early-stage nanobody design against a new SARS-CoV-2 variant. The first version should focus on planning, tool selection, and workflow definition.";
end

function questions = defaultAgendaQuestions()
questions = [ ...
    "What is the first concrete planning decision the team should make?", ...
    "Which tools or methods belong in the first implementation slice?", ...
    "What would count as a credible first success signal?"];
end

function items = teamMemberOrderItems()
items = ["Fixed Order", "Randomize Members"];
end

function displayValue = teamMemberOrderDisplayFromValue(value)
value = normalizeTeamMemberOrderValue(value);
switch value
    case "fixed"
        displayValue = "Fixed Order";
    case "randomized"
        displayValue = "Randomize Members";
    otherwise
        error("launchVirtualLabApp:InvalidTeamMemberOrder", ...
            "Unsupported team member order mode: %s", value);
end
end

function value = teamMemberOrderValueFromDisplay(displayValue)
displayValue = strtrim(string(displayValue));
switch displayValue
    case "Fixed Order"
        value = "fixed";
    case "Randomize Members"
        value = "randomized";
    otherwise
        value = normalizeTeamMemberOrderValue(displayValue);
end
end

function value = normalizeTeamMemberOrderValue(value)
value = lower(strtrim(string(value)));
if strlength(value) == 0
    value = "fixed";
elseif any(value == ["fixed order", "fixed_order"])
    value = "fixed";
elseif any(value == ["randomize members", "randomized_members", "randomize_members"])
    value = "randomized";
end
if ~any(value == ["fixed", "randomized"])
    error("launchVirtualLabApp:InvalidTeamMemberOrder", ...
        "Unsupported team member order mode: %s", value);
end
end

function names = workflowPhaseNames(phases)
if isempty(phases)
    names = "no_phases";
else
    names = string({phases.Name});
end
end

function tableData = workflowPhaseTableData(phases, configs)
tableData = cell(numel(phases), 4);
for idx = 1:numel(phases)
    tableData{idx, 1} = char(phases(idx).Name);
    tableData{idx, 2} = char(phases(idx).MeetingType);
    tableData{idx, 3} = char(workflowPhaseOrderSummary(phases(idx)));
    tableData{idx, 4} = char(workflowParticipantSummary(phases(idx), configs));
end
end

function summary = workflowPhaseOrderSummary(phase)
if string(phase.MeetingType) == "team"
    summary = teamMemberOrderDisplayFromValue(phase.TeamMemberOrder);
else
    summary = "-";
end
summary = summary(1);
end

function summary = workflowParticipantSummary(phase, configs)
if string(phase.MeetingType) == "team"
    leadTitle = agentTitleForKey(configs, phase.TeamLead);
    memberTitles = agentTitlesForKeys(configs, phase.TeamMembers);
    summary = "Chair: " + leadTitle + " | Members: " + join(memberTitles, ", ");
else
    summary = "Agent: " + agentTitleForKey(configs, phase.TeamMember);
end
summary = summary(1);
end

function phase = newWorkflowPhaseTemplate(index, teamLeadKey, individualKey)
phase = struct( ...
    "Name", "phase_" + string(index), ...
    "MeetingType", "team", ...
    "TeamLead", string(teamLeadKey), ...
    "TeamMembers", strings(1, 0), ...
    "TeamMember", string(individualKey), ...
    "TeamMemberOrder", "fixed", ...
    "Agenda", "Project brief:" + newline + "{{PROJECT_BRIEF}}" + newline + newline + ...
        "Describe the purpose of this phase.", ...
    "AgendaQuestions", strings(1, 0), ...
    "AgendaRules", strings(1, 0), ...
    "Contexts", strings(1, 0));
end

function lines = nonemptyLines(value)
text = normalizeInputText(value);
rawLines = splitlines(text);
rawLines = strtrim(rawLines);
lines = rawLines(strlength(rawLines) > 0);
if isempty(lines)
    lines = strings(1, 0);
end
end

function visibleState = ternaryVisible(tf)
if tf
    visibleState = "on";
else
    visibleState = "off";
end
end

function presetPaths = discoverWorkflowPresets(presetDir)
if ~isfolder(presetDir)
    presetPaths = strings(1, 0);
    return
end

listing = dir(fullfile(presetDir, "*.json"));
if isempty(listing)
    presetPaths = strings(1, 0);
    return
end

presetPaths = string(fullfile({listing.folder}, {listing.name}));
presetNames = workflowPresetDisplayNames(presetPaths);
[~, order] = sort(lower(presetNames));
presetPaths = presetPaths(order);
end

function presetItems = workflowPresetDisplayNames(presetPaths)
if isempty(presetPaths)
    presetItems = "No saved presets";
    return
end

presetItems = strings(size(presetPaths));
for idx = 1:numel(presetPaths)
    presetItems(idx) = workflowPresetDisplayName(presetPaths(idx));
end
end

function presetItem = workflowPresetDisplayName(presetPath)
[~, presetName] = fileparts(char(presetPath));
presetItem = string(presetName);
try
    payload = jsondecode(fileread(presetPath));
    if isfield(payload, "PresetName")
        configuredName = normalizePresetScalar(payload.PresetName);
        if strlength(configuredName) > 0
            presetItem = configuredName;
        end
    end
catch
end
end

function presetName = sanitizeWorkflowPresetName(value)
presetName = string(strtrim(value));
presetName = regexprep(presetName, "\s+", "_");
presetName = regexprep(presetName, "[^\w-]", "_");
presetName = regexprep(presetName, "_{2,}", "_");
presetName = regexprep(presetName, "^[_-]+|[_-]+$", "");
end

function deleteWorkflowPresetFile(presetPath)
if ~isfile(presetPath)
    return
end

try
    delete(char(presetPath));
catch ME
    error("launchVirtualLabApp:WorkflowPresetDeleteFailed", ...
        "Unable to delete workflow preset '%s': %s", presetPath, ME.message);
end
end

function writeWorkflowPreset(presetPath, presetName, phases)
presetDir = fileparts(presetPath);
if ~isfolder(presetDir)
    mkdir(presetDir);
end

payload = struct( ...
    "Version", 1, ...
    "PresetName", string(strtrim(presetName)), ...
    "SavedAt", string(datetime("now", Format="yyyy-MM-dd'T'HH:mm:ss")), ...
    "Phases", phases);

try
    jsonText = jsonencode(payload, PrettyPrint=true);
catch
    jsonText = jsonencode(payload);
end

fileId = fopen(presetPath, "w");
if fileId < 0
    error("launchVirtualLabApp:WorkflowPresetWriteFailed", ...
        "Unable to open workflow preset for writing: %s", presetPath);
end

cleanupObj = onCleanup(@() fclose(fileId));
fprintf(fileId, "%s", jsonText);
clear cleanupObj
end

function preset = readWorkflowPreset(presetPath)
payload = jsondecode(fileread(presetPath));
if ~isfield(payload, "Phases")
    error("launchVirtualLabApp:InvalidWorkflowPreset", ...
        "Workflow preset does not contain a Phases field: %s", presetPath);
end

if isfield(payload, "PresetName")
    presetName = normalizePresetScalar(payload.PresetName);
else
    [~, fileStem] = fileparts(char(presetPath));
    presetName = string(fileStem);
end

preset = struct( ...
    "PresetName", presetName, ...
    "Phases", normalizeWorkflowPhases(payload.Phases), ...
    "SavedAt", "");

if isfield(payload, "SavedAt")
    preset.SavedAt = normalizePresetScalar(payload.SavedAt);
end
end

function settings = readVirtualLabAppSettings(settingsPath)
settings = struct("LastWorkflowPresetPath", "");
if ~isfile(settingsPath)
    return
end

try
    payload = jsondecode(fileread(settingsPath));
    if isfield(payload, "LastWorkflowPresetPath")
        settings.LastWorkflowPresetPath = normalizePresetScalar(payload.LastWorkflowPresetPath);
    end
catch
end
end

function writeVirtualLabAppSettings(settingsPath, settings)
payload = struct("LastWorkflowPresetPath", normalizePresetScalar(settings.LastWorkflowPresetPath));
try
    jsonText = jsonencode(payload, PrettyPrint=true);
catch
    jsonText = jsonencode(payload);
end

fileId = fopen(settingsPath, "w");
if fileId < 0
    warning("launchVirtualLabApp:SettingsWriteFailed", ...
        "Unable to open app settings for writing: %s", settingsPath);
    return
end

cleanupObj = onCleanup(@() fclose(fileId));
fprintf(fileId, "%s", jsonText);
clear cleanupObj
end

function phases = normalizeWorkflowPhases(rawPhases)
if ~isstruct(rawPhases) || isempty(rawPhases)
    error("launchVirtualLabApp:InvalidWorkflowPreset", ...
        "Workflow preset must contain a non-empty struct array of phases.");
end

phases = repmat(struct( ...
    "Name", "", ...
    "MeetingType", "team", ...
    "TeamLead", "", ...
    "TeamMembers", strings(1, 0), ...
    "TeamMember", "", ...
    "TeamMemberOrder", "fixed", ...
    "Agenda", "", ...
    "AgendaQuestions", strings(1, 0), ...
    "AgendaRules", strings(1, 0), ...
    "Contexts", strings(1, 0)), 1, numel(rawPhases));

for idx = 1:numel(rawPhases)
    phase = phases(idx);
    phase.Name = requiredPresetField(rawPhases(idx), "Name");
    phase.MeetingType = lower(requiredPresetField(rawPhases(idx), "MeetingType"));
    phase.TeamLead = optionalPresetField(rawPhases(idx), "TeamLead");
    phase.TeamMembers = optionalPresetListField(rawPhases(idx), "TeamMembers");
    phase.TeamMember = optionalPresetField(rawPhases(idx), "TeamMember");
    phase.TeamMemberOrder = normalizeTeamMemberOrderValue(optionalPresetField(rawPhases(idx), "TeamMemberOrder"));
    phase.Agenda = requiredPresetField(rawPhases(idx), "Agenda");
    phase.AgendaQuestions = optionalPresetListField(rawPhases(idx), "AgendaQuestions");
    phase.AgendaRules = optionalPresetListField(rawPhases(idx), "AgendaRules");
    phase.Contexts = optionalPresetListField(rawPhases(idx), "Contexts");

    if ~any(phase.MeetingType == ["team", "individual"])
        error("launchVirtualLabApp:InvalidWorkflowPreset", ...
            "Workflow preset phase '%s' has unsupported meeting type '%s'.", ...
            phase.Name, phase.MeetingType);
    end

    if phase.MeetingType == "team"
        phase.TeamMembers = phase.TeamMembers(phase.TeamMembers ~= phase.TeamLead);
        if strlength(phase.TeamLead) == 0 || isempty(phase.TeamMembers)
            error("launchVirtualLabApp:InvalidWorkflowPreset", ...
                "Workflow preset team phase '%s' requires one lead and at least one member.", ...
                phase.Name);
        end
        phase.TeamMember = "";
    else
        if strlength(phase.TeamMember) == 0
            error("launchVirtualLabApp:InvalidWorkflowPreset", ...
                "Workflow preset individual phase '%s' requires one agent.", ...
                phase.Name);
        end
        phase.TeamLead = "";
        phase.TeamMembers = strings(1, 0);
        phase.TeamMemberOrder = "fixed";
    end

    phases(idx) = phase;
end
end

function value = requiredPresetField(structValue, fieldName)
if ~isfield(structValue, fieldName)
    error("launchVirtualLabApp:InvalidWorkflowPreset", ...
        "Workflow preset is missing required field '%s'.", fieldName);
end

value = normalizePresetScalar(structValue.(fieldName));
if strlength(value) == 0
    error("launchVirtualLabApp:InvalidWorkflowPreset", ...
        "Workflow preset field '%s' cannot be empty.", fieldName);
end
end

function value = optionalPresetField(structValue, fieldName)
if ~isfield(structValue, fieldName)
    value = "";
    return
end
value = normalizePresetScalar(structValue.(fieldName));
end

function values = optionalPresetListField(structValue, fieldName)
if ~isfield(structValue, fieldName)
    values = strings(1, 0);
    return
end

values = normalizePresetList(structValue.(fieldName));
values = values(strlength(values) > 0);
end

function value = normalizePresetScalar(rawValue)
if isstring(rawValue) || ischar(rawValue)
    value = string(rawValue);
elseif iscell(rawValue)
    if isempty(rawValue)
        value = "";
    else
        value = string(rawValue{1});
    end
else
    value = string(rawValue);
end

value = join(value, "");
value = value(1);
value = string(strtrim(value));
end

function values = normalizePresetList(rawValue)
if isempty(rawValue)
    values = strings(1, 0);
    return
end

if isstring(rawValue)
    values = reshape(rawValue, 1, []);
elseif ischar(rawValue)
    values = string(rawValue);
elseif iscell(rawValue)
    values = string(rawValue);
else
    values = string(rawValue);
end

values = reshape(values, 1, []);
values = string(strtrim(values));
end

function runPaths = discoverSavedRuns(rootDir)
searchRoots = [
    string(fullfile(rootDir, "ui_runs"))
    string(fullfile(rootDir, "output"))
    string(fullfile(rootDir, "workflow_output"))
    string(fullfile(rootDir, "tmp_workflow_test"))
    string(fullfile(rootDir, "tmp_workflow_test_2"))
    ];

paths = strings(0,1);
timestamps = zeros(0,1);
for idx = 1:numel(searchRoots)
    currentRoot = searchRoots(idx);
    if isfolder(currentRoot)
        listing = dir(fullfile(currentRoot, "**", "*.json"));
        if ~isempty(listing)
            rootPaths = string(fullfile({listing.folder}, {listing.name}))';
            paths = [paths; rootPaths]; %#ok<AGROW>
            timestamps = [timestamps; [listing.datenum]']; %#ok<AGROW>
        end
    end
end

if isempty(paths)
    runPaths = strings(1,0);
    return
end

[paths, uniqueIndex] = unique(paths, "stable");
timestamps = timestamps(uniqueIndex);
[~, order] = sort(timestamps, "descend");
runPaths = paths(order);
end

function items = displayNamesForRuns(rootDir, runPaths)
items = strings(size(runPaths));
prefix = string(rootDir) + filesep;
for idx = 1:numel(runPaths)
    relativePath = erase(runPaths(idx), prefix);
    items(idx) = relativePath;
end
end

function summary = extractSummaryFromTranscript(transcript)
if isempty(transcript)
    summary = "";
else
    summary = string(transcript(end).message);
end
end

function kind = inferTurnKind(agent)
agent = string(agent);
if isControllerAgentName(agent)
    kind = "loaded_prompt";
else
    kind = "loaded_response";
end
end

function tf = isControllerTurn(event)
if isfield(event, "Agent")
    tf = isControllerAgentName(event.Agent);
else
    tf = false;
end
end

function tf = isControllerAgentName(agent)
tf = string(agent) == "User";
end

function name = displayAgentName(agent)
agent = string(agent);
if isControllerAgentName(agent)
    name = "Agent";
else
    name = agent;
end
end

function configs = defaultAgentConfigs()
configs(1) = struct( ...
    "Key", "PrincipalInvestigator", ...
    "Title", "Principal Investigator", ...
    "Expertise", "running interdisciplinary scientific research programs", ...
    "Goal", "lead the team toward a concrete, high-impact project decision", ...
    "Role", "convene meetings, synthesize input, make decisions, and define next steps", ...
    "Model", "gpt-5-mini");
configs(2) = struct( ...
    "Key", "Immunologist", ...
    "Title", "Immunologist", ...
    "Expertise", "antibody engineering and immune response characterization", ...
    "Goal", "identify biologically credible molecular strategies", ...
    "Role", "advise on immune relevance, specificity, and experimental plausibility", ...
    "Model", "gpt-5-mini");
configs(3) = struct( ...
    "Key", "MachineLearningSpecialist", ...
    "Title", "Machine Learning Specialist", ...
    "Expertise", "protein modeling, candidate ranking, and ML workflow design", ...
    "Goal", "propose models and evaluation loops that reduce search cost", ...
    "Role", "select computational methods and define ranking strategy", ...
    "Model", "gpt-5-mini");
configs(4) = struct( ...
    "Key", "ComputationalBiologist", ...
    "Title", "Computational Biologist", ...
    "Expertise", "structural modeling and simulation-driven analysis", ...
    "Goal", "translate biological questions into executable computational workflows", ...
    "Role", "define data flow, structure analysis, and simulation steps", ...
    "Model", "gpt-5-mini");
configs(5) = struct( ...
    "Key", "ScientificCritic", ...
    "Title", "Scientific Critic", ...
    "Expertise", "stress-testing scientific plans for rigor and feasibility", ...
    "Goal", "identify weak assumptions before implementation", ...
    "Role", "challenge unsupported claims and force sharper decisions", ...
    "Model", "gpt-5-mini");
end

function agent = agentStructFromConfig(config)
agent = virtualLabAgent( ...
    config.Title, ...
    config.Expertise, ...
    config.Goal, ...
    config.Role, ...
    config.Model);
end

function titles = agentTitlesFromConfigs(configs)
titles = string({configs.Title});
end

function title = agentTitleForKey(configs, key)
key = string(key);
matchIndex = find(string({configs.Key}) == key, 1, "first");
if isempty(matchIndex)
    title = "";
else
    title = string(configs(matchIndex).Title);
end
end

function titles = agentTitlesForKeys(configs, keys)
keys = string(keys);
titles = strings(1, 0);
for idx = 1:numel(keys)
    currentTitle = agentTitleForKey(configs, keys(idx));
    if strlength(currentTitle) > 0
        titles(end+1) = currentTitle; %#ok<AGROW>
    end
end
end

function key = agentKeyForTitle(configs, title)
title = string(title);
matchIndex = find(string({configs.Title}) == title, 1, "first");
if isempty(matchIndex)
    key = "";
else
    key = string(configs(matchIndex).Key);
end
end

function keys = agentKeysForTitles(configs, titles)
titles = string(titles);
keys = strings(1, 0);
for idx = 1:numel(titles)
    currentKey = agentKeyForTitle(configs, titles(idx));
    if strlength(currentKey) > 0
        keys(end+1) = currentKey; %#ok<AGROW>
    end
end
end

function tableData = agentConfigTableData(configs)
tableData = cell(numel(configs), 5);
for idx = 1:numel(configs)
    tableData{idx, 1} = char(configs(idx).Title);
    tableData{idx, 2} = char(configs(idx).Model);
    tableData{idx, 3} = char(configs(idx).Expertise);
    tableData{idx, 4} = char(configs(idx).Goal);
    tableData{idx, 5} = char(configs(idx).Role);
end
end

function configs = tableDataToAgentConfigs(tableData, existingConfigs)
configs = existingConfigs;
for idx = 1:size(tableData, 1)
    configs(idx).Model = string(strtrim(string(tableData{idx, 2})));
    configs(idx).Expertise = string(strtrim(string(tableData{idx, 3})));
    configs(idx).Goal = string(strtrim(string(tableData{idx, 4})));
    configs(idx).Role = string(strtrim(string(tableData{idx, 5})));
    if strlength(configs(idx).Model) == 0
        configs(idx).Model = "gpt-5-mini";
    end
end
end
