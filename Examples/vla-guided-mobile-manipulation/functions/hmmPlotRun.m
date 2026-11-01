function info = hmmPlotRun(ax, out, view, cfg)
%HMMPLOTRUN One named view of a homeMobileManipulator run, drawn into an axes.
%
%   HMMPLOTRUN(ax, out, view) draws VIEW of the SimulationOutput OUT into the axes AX.
%
%   views = HMMPLOTRUN() returns the catalogue of available views as a struct array with
%   fields Key, Title and Blurb, and draws nothing.
%
%   info = HMMPLOTRUN(ax, out, view) also returns the catalogue entry that was drawn.
%
%   HMMPLOTRUN(ax, out, view, cfg) judges the run against CFG instead of against the
%   configuration hmmSetup published. A caller that simulated with an overridden cfg - the
%   app does, through setVariable - must pass the same struct back, or the reference lines
%   will describe limits the run did not have.
%
%   WHY THE PLOTS LIVE HERE AND NOT IN THE SCRIPT OR THE APP. HomeMobileManipulatorExample
%   and HomeMobileManipulatorApp both present the same run, and a second copy of a plot is a
%   second thing to be wrong: the live script would show a tracking-error trace against one
%   bound and the app against another, and neither would be visibly stale. So every figure
%   in the project comes from this one function, the script by naming the views it narrates
%   and the app by listing them in a dropdown. Adding a view here adds it to both.
%
%   EVERY BOUND IS READ FROM cfg RATHER THAN DRAWN AT A LITERAL. The tracking-error limit,
%   the clearance threshold and the joint travel are all configured values, and a reference
%   line drawn at a number typed here would keep claiming the old limit after hmmConfig
%   changed. cfg defaults to hmmSetup's, so it is the same struct the simulation ran against
%   unless the caller says otherwise.
%
%   Views:
%     BasePath      base and tool ground tracks in the apartment plane
%     BaseTracking  distance to goal, cross-track and heading error against their bounds
%     ArmJoints     the six measured joint angles against the selected command
%     ArmTracking   task-space and joint-space error against cfg.safety.trackErrMax
%     Gripper       aperture against command, with the capture flag
%     SafetyFlags   every SafetyMonitor check as a raster, with the latched cause
%     Modes         planner, base, arm and gripper supervisory modes on one time axis
%
%   Example:
%       d   = hmmSetup();
%       in  = Simulink.SimulationInput('homeMobileManipulator');
%       out = sim(in);
%       hmmPlotRun(axes, out, 'BaseTracking')
%
%   See also hmmRunTimeline, hmmRunSummary, HomeMobileManipulatorApp.

%   Copyright 2026 The MathWorks, Inc.

catalogue = struct( ...
    'Key',   {'BasePath', 'BaseTracking', 'ArmJoints', 'ArmTracking', 'Gripper', ...
              'SafetyFlags', 'Modes'}, ...
    'Title', {'Ground tracks', 'Base path following', 'Arm joint angles', ...
              'Arm tracking error', 'Gripper aperture', 'Safety monitor checks', ...
              'Supervisory modes'}, ...
    'Blurb', {'Where the base drove and where the tool went with it.', ...
              'What the follower was asked for and how closely it held it.', ...
              'The six commanded joints and what the servo plant realised.', ...
              'Tool pose error against the limit that latches the e-stop.', ...
              'Commanded and measured jaw opening, and whether capture held.', ...
              'Which limit each check saw exceeded, sample by sample.', ...
              'The four state machines against one another in time.'});

if nargin == 0
    info = catalogue;
    return
end

k = find(strcmpi({catalogue.Key}, view), 1);
if isempty(k)
    error('hmmPlotRun:unknownView', 'Unknown view ''%s''. Available: %s.', ...
        view, strjoin({catalogue.Key}, ', '));
end
info = catalogue(k);

% hmmSetup rather than hmmConfig, because the drawn bounds must be the DERIVED ones: hmmSetup
% is where the placeholder gains become their operational-space forms, and it is also what the
% simulation ran against. An arguments block cannot express this - the catalogue return above
% has to run before any argument validation - so the default is taken here.
if nargin < 4 || isempty(cfg)
    cfg = hmmSetup().cfg;
end

cla(ax, 'reset');
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');

switch info.Key
    case 'BasePath',     drawBasePath(ax, out, cfg);
    case 'BaseTracking', drawBaseTracking(ax, out, cfg);
    case 'ArmJoints',    drawArmJoints(ax, out, cfg);
    case 'ArmTracking',  drawArmTracking(ax, out, cfg);
    case 'Gripper',      drawGripper(ax, out, cfg);
    case 'SafetyFlags',  drawSafetyFlags(ax, out, cfg);
    case 'Modes',        drawModes(ax, out, cfg);
end

hold(ax, 'off');

if nargout == 0
    clear info
end

end

%% ------------------------------------------------------------------------
function drawBasePath(ax, out, cfg)
%DRAWBASEPATH The two ground tracks, in the apartment's own coordinates.
%   The tool track is drawn alongside the base track because the interesting part of a
%   transfer is where they DIVERGE: the base holds its stand-off while the arm reaches past
%   it, so a tool track that never leaves the base track is an arm that never moved.
[t, base] = leaf(out, 'RobotState', 'BasePose');
[~, tool] = leaf(out, 'RobotState', 'ToolTranslation');

plot(ax, base(:, 1), base(:, 2), '-', 'LineWidth', 1.8, 'DisplayName', 'base origin');
plot(ax, tool(:, 1), tool(:, 2), '-', 'LineWidth', 1.0, 'DisplayName', 'tool centre point');
plot(ax, base(1, 1), base(1, 2), 'o', 'MarkerSize', 9, 'LineWidth', 1.5, ...
    'DisplayName', sprintf('start, t = %.1f s', t(1)));
plot(ax, base(end, 1), base(end, 2), 'x', 'MarkerSize', 11, 'LineWidth', 1.8, ...
    'DisplayName', sprintf('end, t = %.1f s', t(end)));

% The rooms, so a reader can tell a kitchen traverse from a bathroom one without the scene.
for r = 1:numel(cfg.scene.room)
    c = cfg.scene.room(r).centre;
    text(ax, c(1), c(2), cfg.scene.room(r).name, 'HorizontalAlignment', 'center', ...
        'FontAngle', 'italic', 'Color', [0.45 0.45 0.45]);
end

axis(ax, 'equal');
xlim(ax, cfg.scene.extent(1, :));
ylim(ax, cfg.scene.extent(2, :));
% World y downward, as the Sim3D window shows the apartment (Unreal's Y is the world's
% -Y); with y upward this plot read as the scene's mirror image. Data stay in world metres.
ax.YDir = 'reverse';
title(ax, 'Ground tracks in the apartment plane');
xlabel(ax, 'world x (m)');
ylabel(ax, 'world y (m), downward as in Sim3D');
legend(ax, 'Location', 'best');
end

%% ------------------------------------------------------------------------
function drawBaseTracking(ax, out, cfg)
%DRAWBASETRACKING The follower's three errors against the two bounds that judge them.
[t, dist]  = leaf(out, 'BaseStatus', 'DistanceToGoal');
[tc, xtrk] = leaf(out, 'BaseStatus', 'CrossTrackError');
[th, head] = leaf(out, 'BaseStatus', 'HeadingError');

plot(ax, t,  dist,        '-', 'LineWidth', 1.4, 'DisplayName', 'distance to goal (m)');
plot(ax, tc, abs(xtrk),   '-', 'LineWidth', 1.2, 'DisplayName', '|cross-track error| (m)');
plot(ax, th, abs(head),   '-', 'LineWidth', 1.0, 'DisplayName', '|heading error| (rad)');
yline(ax, cfg.track.crossTrackMax, '--', ...
    sprintf('crossTrackMax %.2f m', cfg.track.crossTrackMax), ...
    'LabelHorizontalAlignment', 'left', 'DisplayName', 'cross-track bound');
yline(ax, cfg.plan.baseGoalTolerance(1), ':', ...
    sprintf('goal tolerance %.2f m', cfg.plan.baseGoalTolerance(1)), ...
    'LabelHorizontalAlignment', 'right', 'DisplayName', 'goal tolerance');

title(ax, 'Base path following');
xlabel(ax, 'time (s)');
ylabel(ax, 'error');
legend(ax, 'Location', 'best');
end

%% ------------------------------------------------------------------------
function drawArmJoints(ax, out, cfg)
%DRAWARMJOINTS Command against measurement, six channels, one axes.
%   The command is the SELECTED one - what ModeSelect actually handed the plant - rather
%   than either planner's output, because the handover between the joint-space and
%   task-space laws is exactly where a discontinuity would show.
[t,  q]   = leaf(out, 'RobotState', 'ArmJointPos');
[tc, cmd] = leaf(out, 'SelectedJointCmd', '');

c = lines(cfg.arm.nJoints);
for j = 1:cfg.arm.nJoints
    plot(ax, t, q(:, j), '-', 'Color', c(j, :), 'LineWidth', 1.4, ...
        'DisplayName', sprintf('q_%d measured', j));
end
for j = 1:cfg.arm.nJoints
    plot(ax, tc, cmd(:, j), '--', 'Color', c(j, :), 'LineWidth', 0.8, ...
        'HandleVisibility', 'off');
end

title(ax, 'Arm joint angles: measured (solid) against selected command (dashed)');
xlabel(ax, 'time (s)');
ylabel(ax, 'joint angle (rad)');
legend(ax, 'Location', 'eastoutside');
end

%% ------------------------------------------------------------------------
function drawArmTracking(ax, out, cfg)
%DRAWARMTRACKING The two arm errors, and the guard that acts on one of them.
%   TrackingError is a tool-pose distance and cfg.safety.trackErrMax is the threshold it is
%   judged against, so the bound is drawn on the same axes rather than described in prose.
%   JointError shares the axes deliberately: when the tool error grows while the joint error
%   does not, the servo is tracking its command and the COMMAND is the problem.
%
%   THE BOUND IS NOT AN E-STOP BOUND, AND THIS LINE USED TO SAY IT WAS. Nothing in
%   SafetyMonitor reads cfg.safety.trackErrMax - its eleven checks are joint limit, joint
%   velocity, joint acceleration, workspace, base velocity, base acceleration, clearance and
%   three staleness watchdogs. The threshold is consumed as chart data instead: xTrackMax in
%   ArmSupervisor, which guards Retreating -> Holding, and in BaseSupervisor via
%   cfg.track.crossTrackMax. So an excursion above this line is a SUPERVISOR-scoped event and
%   the run can legitimately show one with EStop clear for all 40 s. That combination read as
%   an unexplained inconsistency for as long as the label said "e-stop bound", which is the
%   cost of naming a threshold after a mechanism that does not own it.
%
%   AND THE GUARD DOES NOT APPLY IN EVERY MODE, WHICH THE FLAT LINE CANNOT SHOW. The shipped
%   run exceeds it twice - 0.5779 m over t = 15.12-15.48 s and 0.6257 m over 32.36-32.92 s,
%   about 4x - and both excursions are STOWING, entered DONE and clean. The mechanism is a
%   Switch, not a tolerance: ArmControl/TrackingMetrics/TargetSelect measures this error
%   against the tracked reference while TaskSpaceMode is on and against the LAST TASK TARGET
%   while it is off, and it is off during a stow. So the number is not meaningful there
%   rather than tolerated there. Read the bound against approach and retreat only.
%
%   Earlier drafts of this comment quoted 0.9827 m at 16.18-17.86 s and a second at 35.8 s.
%   Those were measured before cfg.arm.stowConfig was rotated to face forward; the slew got
%   shorter, so the excursions did. The mechanism did not change, which is why the shrinking
%   numbers are not a fix.
[t,  te] = leaf(out, 'ArmStatus', 'TrackingError');
[tj, je] = leaf(out, 'ArmStatus', 'JointError');

plot(ax, t,  te, '-', 'LineWidth', 1.6, 'DisplayName', 'tool tracking error (m)');
plot(ax, tj, je, '-', 'LineWidth', 1.0, 'DisplayName', 'joint error norm (rad)');
yline(ax, cfg.safety.trackErrMax, '--', ...
    sprintf('trackErrMax %.2f m', cfg.safety.trackErrMax), ...
    'LabelHorizontalAlignment', 'left', ...
    'DisplayName', 'ArmSupervisor retreat guard');

title(ax, 'Arm tracking error');
xlabel(ax, 'time (s)');
ylabel(ax, 'error');
legend(ax, 'Location', 'best');
end

%% ------------------------------------------------------------------------
function drawGripper(ax, out, cfg)
%DRAWGRIPPER Aperture against command, with the plant's capture verdict beneath it.
[t,  ap]  = leaf(out, 'GripperStatus', 'Aperture');
[tc, cmd] = leaf(out, 'ArmStatus',     'ApertureCmd');
[tg, gr]  = leaf(out, 'RobotState',    'IsGrasped');

plot(ax, tc, 1000 * cmd, '--', 'LineWidth', 1.2, 'DisplayName', 'aperture command (mm)');
plot(ax, t,  1000 * ap,  '-',  'LineWidth', 1.6, 'DisplayName', 'measured aperture (mm)');
plot(ax, tg, 1000 * cfg.gripper.maxOpenPos * gr, ':', 'LineWidth', 1.4, ...
    'DisplayName', 'IsGrasped (scaled)');
yline(ax, 1000 * cfg.gripper.maxOpenPos, '--', ...
    sprintf('stroke %.0f mm', 1000 * cfg.gripper.maxOpenPos), ...
    'LabelHorizontalAlignment', 'left', 'DisplayName', 'full stroke');

title(ax, 'Gripper aperture');
xlabel(ax, 'time (s)');
ylabel(ax, 'aperture (mm)');
legend(ax, 'Location', 'best');
end

%% ------------------------------------------------------------------------
function drawSafetyFlags(ax, out, cfg)
%DRAWSAFETYFLAGS Every check on its own row, so a latched e-stop names its own cause.
%   A raster and not a set of traces: these are nine booleans plus a latch, and overlaid
%   step traces at the same amplitude are unreadable. Each row is filled only where the
%   check was asserted, so an empty row is a check that never fired - which is the normal
%   and expected picture, and is worth being able to see at a glance.
checks = {'JointLimit', 'JointVel', 'JointAccel', 'Workspace', 'BaseVel', 'BaseAccel', ...
          'StaleState', 'StalePlan', 'StaleProvider', 'EStop'};

for i = 1:numel(checks)
    [t, v] = leaf(out, 'SafetyStatus', checks{i});
    y      = numel(checks) - i + 1;
    on     = v ~= 0;
    if any(on)
        plot(ax, t(on), y * ones(sum(on), 1), '.', 'MarkerSize', 8, ...
            'HandleVisibility', 'off');
    end
    yline(ax, y, ':', 'Color', [0.85 0.85 0.85], 'HandleVisibility', 'off');
end

[t, cause] = leaf(out, 'SafetyStatus', 'Cause');
latched    = find(cause ~= 0, 1);
if isempty(latched)
    sub = 'no check asserted at any sample';
else
    sub = sprintf('first cause %s at t = %.3f s', ...
        hmmEnumName(cfg.sup.cause, cause(latched)), t(latched));
end

ylim(ax, [0.5, numel(checks) + 0.5]);
yticks(ax, 1:numel(checks));
yticklabels(ax, flip(checks));
xlim(ax, [t(1) t(end)]);
title(ax, {'Safety monitor checks', sub});
xlabel(ax, 'time (s)');
ylabel(ax, 'check');
end

%% ------------------------------------------------------------------------
function drawModes(ax, out, cfg)
%DRAWMODES The four supervisors on one time axis, each in its own numbering.
%   Stairs and not a raster, because these are not booleans: the VALUE carries the meaning
%   and the sequence is the diagnosis. The lanes are offset so four ranges of small
%   integers do not draw on top of one another, and each lane's own tick labels name its
%   modes - a shared numeric axis would have IDLE at 0 meaning four different things.
lanes = { ...
    'TaskResult',    'PlannerMode', cfg.sup.pl,   'planner'
    'BaseStatus',    'Mode',        cfg.sup.base, 'base'
    'ArmStatus',     'Mode',        cfg.sup.arm,  'arm'
    'GripperStatus', 'Mode',        cfg.sup.grip, 'gripper'
    };

offset = 0;
ticks  = [];
labels = {};
c      = lines(size(lanes, 1));
for i = 1:size(lanes, 1)
    [t, v]  = leaf(out, lanes{i, 1}, lanes{i, 2});
    names   = enumNames(lanes{i, 3});
    stairs(ax, t, v + offset, '-', 'Color', c(i, :), 'LineWidth', 1.5, ...
        'DisplayName', lanes{i, 4});
    used    = unique(v(:))';
    ticks   = [ticks, used + offset];                                       %#ok<AGROW>
    labels  = [labels, names(used + 1)];                                    %#ok<AGROW>
    offset  = offset + numel(names) + 1;
end

[t, ~] = leaf(out, 'TaskResult', 'PlannerMode');
xlim(ax, [t(1) t(end)]);
ylim(ax, [-1 offset]);
[ticks, order] = sort(ticks);
yticks(ax, ticks);
yticklabels(ax, labels(order));
% THE TICK LABELS ARE MODE NAMES AND MODE NAMES CONTAIN UNDERSCORES, so the default TeX
% interpreter renders NAV_TO_PLACE as NAV<sub>TO</sub>PLACE - it takes each _ as a
% subscript and eats it. That made the shipped docs/images/result_modes.png read
% "NAV_T_OP_LACE", which is the one figure a reader consults to see the order the task
% actually ran in. Set on the axes rather than per-label because every lane's labels come
% from a cfg enumeration and all four use underscores.
ax.TickLabelInterpreter = 'none';
title(ax, 'Supervisory modes');
xlabel(ax, 'time (s)');
legend(ax, 'Location', 'eastoutside');
end

%% ------------------------------------------------------------------------
function names = enumNames(s)
%ENUMNAMES The names of a cfg mode enumeration, in code order from zero.
%   Only the contiguous block from zero is returned, because that is what a Mode field
%   holds; count fields such as nCauses are excluded by the same rule hmmRunTimeline uses.
f    = fieldnames(s);
keep = cellfun(@(n) isempty(regexp(n, '^n[A-Z]', 'once')) && ...
    isscalar(s.(n)) && isnumeric(s.(n)), f);
f    = f(keep);
v    = cellfun(@(n) double(s.(n)), f);
names = cell(1, max(v) + 1);
for k = 1:numel(f)
    names{v(k) + 1} = f{k};
end
names(cellfun(@isempty, names)) = {'?'};
end

%% ------------------------------------------------------------------------
function [t, d] = leaf(out, signal, field)
%LEAF One logged leaf as an nSamples-by-nElements double, with its own time vector.
%   Each leaf carries its own time base and they genuinely differ - the supervisory buses
%   are logged at cfg.rates.supervisor and the plant's at cfg.rates.plant, so a run holds
%   1001-sample and 2001-sample series side by side. Plotting them against a single shared
%   time vector is how a 20 s trace ends up drawn over 10 s.
%
%   FIELD is '' for a signal logged as a bare timeseries rather than a bus.
el = out.logsout.find('Name', signal);
if isempty(el)
    error('hmmPlotRun:missingSignal', ...
        ['The run does not contain the logged signal ''%s''. This view needs it. ' ...
         'Check that signal logging is still enabled on it in the model.'], signal);
end
v = el{1}.Values;

if ~isempty(field)
    if ~isfield(v, field)
        error('hmmPlotRun:missingField', ...
            '''%s'' has no element ''%s''. The bus definition may have changed.', ...
            signal, field);
    end
    v = v.(field);
end

t    = v.Time(:);
data = v.Data;
if ndims(data) == 3
    d = double(reshape(data, [], size(data, 3))');
else
    d = double(reshape(data, size(data, 1), []));
end
end
