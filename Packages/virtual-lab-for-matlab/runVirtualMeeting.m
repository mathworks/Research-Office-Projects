function [summary, discussion, meta] = runVirtualMeeting(meetingType, agenda, nvp)
%runVirtualMeeting Run a Virtual Lab style meeting in MATLAB.
arguments
    meetingType (1,1) string {mustBeMember(meetingType, ["team", "individual"])}
    agenda (1,1) string
    nvp.SaveDir (1,1) string = "discussions"
    nvp.SaveName (1,1) string = "discussion"
    nvp.TeamLead struct = struct.empty(1,0)
    nvp.TeamMembers (1,:) struct = struct.empty(1,0)
    nvp.TeamMember struct = struct.empty(1,0)
    nvp.TeamMemberOrder (1,1) string {mustBeMember(nvp.TeamMemberOrder, ["fixed", "randomized"])} = "fixed"
    nvp.AgendaQuestions (1,:) string = strings(1,0)
    nvp.AgendaRules (1,:) string = strings(1,0)
    nvp.Summaries (1,:) string = strings(1,0)
    nvp.Contexts (1,:) string = strings(1,0)
    nvp.NumRounds (1,1) double {mustBeNonnegative, mustBeInteger} = 1
    nvp.UseSDK (1,1) logical = true
    nvp.DryRun (1,1) logical = false
    nvp.Provider (1,1) string = "openai"
    nvp.BaseURL (1,1) string = ""
    nvp.Client = []
    nvp.Tools = []
    nvp.Workspace (1,1) struct = struct()
    nvp.Temperature = 0.2
    nvp.DisplayMode (1,1) string {mustBeMember(nvp.DisplayMode, ["off", "detailed"])} = "off"
    nvp.MaxIterations (1,1) double {mustBePositive, mustBeInteger} = 25
    nvp.OnEvent = []
    nvp.TurnPauseSeconds (1,1) double {mustBeNonnegative} = 0
    nvp.ShouldCancel = []
end

validateMeetingInputs(meetingType, nvp);
resolvedNvp = resolveMeetingConfiguration(meetingType, nvp);

saveDir = char(nvp.SaveDir);
if ~exist(saveDir, "dir")
    mkdir(saveDir);
end

discussion = repmat(turn("", ""), 1, expectedTurnCount(meetingType, resolvedNvp));
discussionIndex = 0;
if resolvedNvp.DryRun
    sharedMessages = [];
else
    sharedMessages = aisdk.message.LLMMessage.empty(1,0);
end
workspace = resolvedNvp.Workspace;
inputTokens = 0;
outputTokens = 0;
elapsedStart = tic;
cancelled = false;

[team, ~] = getMeetingTeam(meetingType, resolvedNvp);
speakingOrder = speakingOrderTitles(meetingType, resolvedNvp);

emitEvent(resolvedNvp.OnEvent, struct( ...
    "Type", "meeting_started", ...
    "MeetingType", meetingType, ...
    "Agenda", agenda, ...
    "NumRounds", resolvedNvp.NumRounds, ...
    "DryRun", resolvedNvp.DryRun, ...
    "UseSDK", resolvedNvp.UseSDK && ~resolvedNvp.DryRun, ...
    "SaveDir", resolvedNvp.SaveDir, ...
    "SaveName", resolvedNvp.SaveName, ...
    "TeamMemberOrder", resolvedNvp.TeamMemberOrder, ...
    "SpeakingOrder", speakingOrder));

for roundIndex = 0:resolvedNvp.NumRounds
    if shouldCancel(resolvedNvp.ShouldCancel)
        cancelled = true;
        break
    end
    roundNumber = roundIndex + 1;
    for agentIndex = 1:numel(team)
        if shouldCancel(resolvedNvp.ShouldCancel)
            cancelled = true;
            break
        end
        currentAgent = team(agentIndex);
        prompt = buildTurnPrompt(meetingType, currentAgent, roundIndex, roundNumber, agenda, resolvedNvp);

        if resolvedNvp.DryRun
            response = buildDryRunResponse(currentAgent, prompt, roundIndex == resolvedNvp.NumRounds);
        else
            [response, sharedMessages, workspace, inputDelta, outputDelta] = ...
                runAgentTurn(currentAgent, prompt, sharedMessages, workspace, resolvedNvp);
            inputTokens = inputTokens + inputDelta;
            outputTokens = outputTokens + outputDelta;
        end

        [discussion, discussionIndex] = appendDiscussionTurn( ...
            discussion, discussionIndex, currentAgent.Title, response, resolvedNvp.OnEvent, ...
            struct( ...
                "TurnKind", "response", ...
                "RoundIndex", roundIndex, ...
                "RoundNumber", roundNumber, ...
                "Speaker", currentAgent.Title), ...
            resolvedNvp.TurnPauseSeconds);

        if shouldCancel(resolvedNvp.ShouldCancel)
            cancelled = true;
            break
        end

        if roundIndex == resolvedNvp.NumRounds
            break
        end
    end
end

discussion = discussion(1:discussionIndex);
if isempty(discussion)
    summary = "";
else
    summary = discussion(end).message;
end
[jsonPath, markdownPath] = saveDiscussion(discussion, resolvedNvp.SaveDir, resolvedNvp.SaveName);
if cancelled
    emitEvent(resolvedNvp.OnEvent, struct( ...
        "Type", "meeting_cancelled", ...
        "MeetingType", meetingType, ...
        "SaveDir", resolvedNvp.SaveDir, ...
        "SaveName", resolvedNvp.SaveName, ...
        "SpeakingOrder", speakingOrder));
end
emitEvent(resolvedNvp.OnEvent, struct( ...
    "Type", "meeting_saved", ...
    "MeetingType", meetingType, ...
    "JsonPath", jsonPath, ...
    "MarkdownPath", markdownPath, ...
    "SaveName", resolvedNvp.SaveName, ...
    "SaveDir", resolvedNvp.SaveDir, ...
    "SpeakingOrder", speakingOrder));
meta = struct( ...
    "MeetingType", meetingType, ...
    "Status", ternary(cancelled, "cancelled", "completed"), ...
    "UsedSDK", resolvedNvp.UseSDK && ~resolvedNvp.DryRun, ...
    "DryRun", resolvedNvp.DryRun, ...
    "NumTurns", numel(discussion), ...
    "NumInputTokens", inputTokens, ...
    "NumOutputTokens", outputTokens, ...
    "ElapsedSeconds", toc(elapsedStart), ...
    "JsonPath", jsonPath, ...
    "MarkdownPath", markdownPath, ...
    "TeamMemberOrder", resolvedNvp.TeamMemberOrder, ...
    "SpeakingOrder", speakingOrder, ...
    "Workspace", workspace);
emitEvent(resolvedNvp.OnEvent, struct( ...
    "Type", "meeting_completed", ...
    "MeetingType", meetingType, ...
    "Summary", summary, ...
    "Meta", meta));
end

function validateMeetingInputs(meetingType, nvp)
if meetingType == "team"
    if isempty(nvp.TeamLead) || isempty(nvp.TeamMembers)
        error("runVirtualMeeting:MissingTeam", ...
            "Team meetings require TeamLead and TeamMembers.");
    end
    if ~isempty(nvp.TeamMember)
        error("runVirtualMeeting:UnexpectedTeamMember", ...
            "Team meetings do not use TeamMember.");
    end
else
    if isempty(nvp.TeamMember)
        error("runVirtualMeeting:MissingTeamMember", ...
            "Individual meetings require TeamMember.");
    end
    if ~isempty(nvp.TeamLead) || ~isempty(nvp.TeamMembers)
        error("runVirtualMeeting:UnexpectedTeamConfig", ...
            "Individual meetings do not use TeamLead or TeamMembers.");
    end
end
end

function resolvedNvp = resolveMeetingConfiguration(meetingType, nvp)
resolvedNvp = nvp;
if meetingType == "team"
    resolvedNvp.TeamMembers = orderedTeamMembers(nvp.TeamMembers, nvp.TeamMemberOrder);
else
    resolvedNvp.TeamMemberOrder = "fixed";
end
end

function members = orderedTeamMembers(teamMembers, orderMode)
members = teamMembers;
if orderMode == "randomized" && numel(teamMembers) > 1
    members = teamMembers(randperm(numel(teamMembers)));
end
end

function [team, primaryAgent] = getMeetingTeam(meetingType, nvp)
if meetingType == "team"
    team = [nvp.TeamLead, nvp.TeamMembers];
    primaryAgent = nvp.TeamLead;
else
    critic = virtualLabAgent( ...
        "Scientific Critic", ...
        "providing critical feedback for scientific research", ...
        "ensure the proposal is rigorous, feasible, and scientifically sound", ...
        "critique answers, demand missing detail, and avoid implementing the answer directly", ...
        nvp.TeamMember.Model);
    team = [nvp.TeamMember, critic];
    primaryAgent = nvp.TeamMember;
end
end

function client = resolveClient(defaultModel, nvp)
if ~isempty(nvp.Client)
    client = nvp.Client;
    return
end

clientArgs = llmClientArgs(nvp.Provider, defaultModel, nvp.Temperature, nvp.BaseURL);
client = aisdk.LLMClient(nvp.Provider, defaultModel, clientArgs{:});
end

function [response, sharedMessages, workspace, inputTokens, outputTokens] = ...
        runAgentTurn(agentSpec, prompt, sharedMessages, workspace, nvp)
client = resolveClient(agentSpec.Model, nvp);
agentArgs = { ...
    "SystemPrompt", agentSpec.SystemPrompt, ...
    "Messages", sharedMessages, ...
    "Workspace", workspace, ...
    "DisplayMode", nvp.DisplayMode, ...
    "MaxIterations", nvp.MaxIterations};

if ~isempty(nvp.Tools)
    agentArgs = [agentArgs, {"Tools", nvp.Tools}];
end

sessionAgent = aisdk.AIAgent(client, agentArgs{:});
response = string(run(sessionAgent, prompt));
sharedMessages = sessionAgent.Messages;
workspace = sessionAgent.Workspace;
inputTokens = sessionAgent.NumInputTokens;
outputTokens = sessionAgent.NumOutputTokens;
end

function args = llmClientArgs(provider, model, temperature, baseURL)
args = {};
if shouldSetTemperature(provider, model)
    args = [args, {"Temperature", temperature}];
end
if strlength(baseURL) > 0
    args = [args, {"BaseURL", baseURL}];
end
end

function tf = shouldSetTemperature(provider, model)
provider = lower(string(provider));
model = lower(strtrim(string(model)));
tf = true;
if provider == "openai" && startsWith(model, "gpt-5")
    tf = false;
end
end

function prompt = buildTurnPrompt(meetingType, currentAgent, roundIndex, roundNumber, agenda, nvp)
if meetingType == "team"
    if currentAgent.Title == nvp.TeamLead.Title
        if roundIndex == 0
            prompt = buildTeamMeetingStartPrompt( ...
                nvp.TeamLead, nvp.TeamMembers, agenda, nvp.AgendaQuestions, ...
                nvp.AgendaRules, nvp.Summaries, nvp.Contexts, nvp.NumRounds);
        elseif roundIndex == nvp.NumRounds
            prompt = currentAgent.Title + ", please summarize the meeting for future discussions, provide a specific recommendation regarding the agenda, and answer the agenda questions while strictly following the agenda rules." + newline + newline + ...
                formatAgenda(agenda, "As a reminder, here is the agenda for the meeting:") + ...
                formatAgendaQuestions(nvp.AgendaQuestions, "As a reminder, here are the agenda questions that must be answered:") + ...
                formatAgendaRules(nvp.AgendaRules, "As a reminder, here are the agenda rules that must be followed:") + ...
                "Your summary should include the sections Agenda, Team Member Input, Recommendation, and Next Steps. " + ...
                "Keep each section compact and focused on decisions, disagreements, and next actions.";
        else
            prompt = "This concludes round " + string(roundNumber - 1) + " of " + string(nvp.NumRounds) + ...
                " of discussion. " + currentAgent.Title + ...
                ", please synthesize the points raised by each team member, make decisions regarding the agenda, and ask follow-up questions. " + ...
                "Close by inviting " + nvp.TeamMembers(1).Title + " to respond first in the next round. " + ...
                conciseTurnInstruction("Keep the synthesis short and only ask the next questions needed to move the discussion forward.");
        end
    else
        prompt = currentAgent.Title + ", please provide your thoughts on the discussion (round " + ...
            string(roundNumber) + " of " + string(nvp.NumRounds) + "). " + ...
            "If you do not have anything new or relevant to add, you may say ""pass"". " + ...
            "Remember that you can and should politely disagree with other team members if you have a different perspective. " + ...
            conciseTurnInstruction("Keep your response short and add only the most decision-relevant points.");
    end
else
    if currentAgent.Title == "Scientific Critic"
        prompt = "Scientific Critic, please critique " + nvp.TeamMember.Title + "'s most recent answer. " + ...
            "Suggest improvements that directly address the agenda and agenda questions. " + ...
            "Only provide feedback; do not implement the answer yourself. " + ...
            conciseTurnInstruction("Limit the critique to the highest-impact gaps or corrections.");
    elseif roundIndex == 0
        prompt = "This is the beginning of an individual meeting with " + nvp.TeamMember.Title + ...
            " to discuss your research project." + newline + newline + ...
            formatReferences(nvp.Contexts, "context", "Here is context for this meeting:") + ...
            formatReferences(nvp.Summaries, "summary", "Here are summaries of the previous meetings:") + ...
            formatAgenda(agenda, "Here is the agenda for the meeting:") + ...
            formatAgendaQuestions(nvp.AgendaQuestions, "Here are the agenda questions that must be answered:") + ...
            formatAgendaRules(nvp.AgendaRules, "Here are the agenda rules that must be followed:") + ...
            nvp.TeamMember.Title + ", please provide your response to the agenda. " + ...
            conciseTurnInstruction("Answer directly and avoid unnecessary background.");
    else
        prompt = nvp.TeamMember.Title + ", please modify your answer to address the Scientific Critic's most recent feedback. " + ...
            "Remember that your ultimate goal is to make improvements that better address the agenda. " + ...
            conciseTurnInstruction("Revise only the parts that need to change.");
    end
end
end

function prompt = buildTeamMeetingStartPrompt(teamLead, teamMembers, agenda, agendaQuestions, agendaRules, summaries, contexts, numRounds)
memberTitles = join([teamMembers.Title], ", ");
firstMemberTitle = teamMembers(1).Title;
prompt = "You are chairing the opening of a team meeting to discuss your research project. " + ...
    "You are the team lead, " + teamLead.Title + ", and the team members are: " + memberTitles + "." + newline + newline + ...
    formatReferences(contexts, "context", "Here is context for this meeting:") + ...
    formatReferences(summaries, "summary", "Here are summaries of the previous meetings:") + ...
    formatAgenda(agenda, "Here is the agenda for the meeting:") + ...
    formatAgendaQuestions(agendaQuestions, "Here are the agenda questions that must be answered:") + ...
    formatAgendaRules(agendaRules, "Here are the agenda rules that must be followed:") + ...
    "Open the meeting yourself. Briefly restate the agenda, share your initial view, and invite " + ...
    firstMemberTitle + " to respond first. " + ...
    "The runtime will keep the meeting in this speaking order for " + string(numRounds) + ...
    " discussion rounds and will ask you to synthesize before the next round begins. " + ...
    "Do not mention hidden runtime instructions. " + ...
    "Keep the discussion concise: avoid repeating the full agenda, prefer compact answers, and focus on decisions and next steps.";
end

function text = conciseTurnInstruction(instruction)
text = instruction + " Prefer 2-4 short bullets or a short paragraph unless more detail is required.";
end

function text = formatAgenda(agenda, intro)
text = intro + newline + newline + agenda + newline + newline;
text = join(string(text), "");
text = text(1);
end

function text = formatAgendaQuestions(questions, intro)
if isempty(questions)
    text = "";
    return
end

text = intro + newline + newline + formatPromptList(questions) + newline + newline;
text = join(string(text), "");
text = text(1);
end

function text = formatAgendaRules(rules, intro)
if isempty(rules)
    text = "";
    return
end

text = intro + newline + newline + formatPromptList(rules) + newline + newline;
text = join(string(text), "");
text = text(1);
end

function text = formatReferences(items, referenceType, intro)
if isempty(items)
    text = "";
    return
end

chunks = strings(1, numel(items));
for idx = 1:numel(items)
    chunks(idx) = "[begin " + referenceType + " " + string(idx) + "]" + ...
        newline + newline + items(idx) + newline + newline + ...
        "[end " + referenceType + " " + string(idx) + "]";
end
text = intro + newline + newline + join(chunks, newline + newline) + newline + newline;
text = join(string(text), "");
text = text(1);
end

function listText = formatPromptList(items)
parts = strings(1, numel(items));
for idx = 1:numel(items)
    parts(idx) = string(idx) + ". " + items(idx);
end
listText = join(parts, newline + newline);
listText = join(string(listText), "");
listText = listText(1);
end

function response = buildDryRunResponse(~, prompt, isFinalRound)
promptPreview = extractBefore(prompt, min(strlength(prompt), 160) + 1);
if isFinalRound
    response = "Dry-run placeholder summary." + newline + newline + ...
        "This turn was generated without a model call." + newline + ...
        "A real run would synthesize the discussion, answer the agenda, and recommend next steps.";
else
    response = "Dry-run placeholder response." + newline + newline + ...
        "This turn was generated without a model call." + newline + ...
        "Prompt preview: " + promptPreview;
end
end

function [jsonPath, markdownPath] = saveDiscussion(discussion, saveDir, saveName)
jsonPath = fullfile(saveDir, saveName + ".json");
markdownPath = fullfile(saveDir, saveName + ".md");

jsonText = jsonencode(discussion, PrettyPrint=true);
fid = fopen(jsonPath, "w");
cleanupJson = onCleanup(@() fclose(fid));
fprintf(fid, "%s", jsonText);
clear cleanupJson

fid = fopen(markdownPath, "w");
cleanupMd = onCleanup(@() fclose(fid));
for idx = 1:numel(discussion)
    fprintf(fid, "## %s\n\n%s\n\n", discussion(idx).agent, discussion(idx).message);
end
clear cleanupMd
end

function item = turn(agentName, message)
item = struct("agent", string(agentName), "message", string(message));
end

function [discussion, discussionIndex] = appendDiscussionTurn(discussion, discussionIndex, agentName, message, callback, eventData, pauseSeconds)
discussionIndex = discussionIndex + 1;
discussion(discussionIndex) = turn(agentName, message);

event = eventData;
event.Type = "turn_added";
event.TurnIndex = discussionIndex;
event.Turn = discussion(discussionIndex);
event.Agent = discussion(discussionIndex).agent;
event.Message = discussion(discussionIndex).message;
emitEvent(callback, event);

if pauseSeconds > 0
    pause(pauseSeconds);
end
end

function count = expectedTurnCount(meetingType, nvp)
if meetingType == "team"
    numAgents = 1 + numel(nvp.TeamMembers);
    count = nvp.NumRounds * numAgents + 1;
else
    count = 2 * nvp.NumRounds + 1;
end
end

function titles = speakingOrderTitles(meetingType, nvp)
if meetingType == "team"
    titles = [string(nvp.TeamLead.Title), string({nvp.TeamMembers.Title})];
else
    titles = [string(nvp.TeamMember.Title), "Scientific Critic"];
end
titles = reshape(titles, 1, []);
end

function emitEvent(callback, event)
if isempty(callback)
    return
end

try
    callback(event);
catch ME
    warning("runVirtualMeeting:EventCallbackFailed", ...
        "Event callback failed: %s", ME.message);
end

drawnow limitrate
end

function tf = shouldCancel(callback)
if isempty(callback)
    tf = false;
    return
end

try
    tf = logical(callback());
catch
    tf = false;
end
end

function out = ternary(condition, trueValue, falseValue)
if condition
    out = trueValue;
else
    out = falseValue;
end
end
