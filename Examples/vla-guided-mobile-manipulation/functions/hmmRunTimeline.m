function T = hmmRunTimeline(out)
%HMMRUNTIMELINE Every discrete state change in a run, on one time axis.
%
%   T = HMMRUNTIMELINE(out) returns a table with one row per instant at which any
%   supervisor changed mode, state, request id or failure code, and prints it.
%
%   WHY A TIMELINE AND NOT A SUMMARY. hmmReportRun reports the SET of modes a run
%   visited, which is the right shape for a pass/fail gate and the wrong shape for a
%   diagnosis. "arm states IDLE AWAIT_PLAN APPROACH, plan failure Unreachable" is true of
%   a run whose arm was asked to reach an object 1.7 m away while the base was still
%   driving - a correct refusal - and equally true of a run whose arm could not reach an
%   object parked squarely in front of it, which is a defect. Only the ORDER separates
%   them, and only against the base's arrival time.
%
%   Rows are emitted on change rather than on every sample because these are all discrete
%   supervisors sampled at 20 to 100 Hz; printing every sample of a 30 s run buries the
%   six instants that matter in six thousand that do not. ServedRequestId and PlanAttempts
%   are carried alongside the modes for the same reason the ordering matters: they are the
%   only record of WHEN a service was asked, as distinct from when it answered.

%   Copyright 2026 The MathWorks, Inc.

arguments
    out   % the SimulationOutput of a run
end

cfg = hmmConfig();

% Each channel is a logged bus and the fields of it that are discrete. Which enumeration
% decodes which field is decided in DECODE, from the field and channel names, so this list
% stays a list of what to watch rather than a second copy of hmmConfig.
chan = { ...
    'TaskResult',       {'PlannerMode', 'Outcome', 'ActiveVerb', 'ActionState', ...
                         'ActiveIndex', 'NumActions', 'NumDone', 'NumFailed', ...
                         'ActionFailure', 'Retries', 'FailureCode', 'EStopCause'}
    'PerceptionStatus', {'Route', 'State', 'RequestId', 'NumActions', 'HttpStatus', ...
                         'FailureCode'}
    'ActionQueue',      {'Count', 'Head', 'Verb', 'ExecState', 'ItemFailure', ...
                         'Overflow', 'FailureCode'}
    'BaseStatus',       {'Mode', 'State', 'ServedRequestId', 'PlanAttempts', ...
                         'ReplanRequest', 'Stalled', 'FailureCode'}
    'ArmStatus',        {'Mode', 'State', 'ServedRequestId', 'PlanAttempts', ...
                         'AttachActorId', 'FailureCode'}
    'GripperStatus',    {'Mode', 'State', 'ServedRequestId', 'Grasped', 'Slipped', ...
                         'FailureCode'}
    'SafetyStatus',     {'EStop', 'Cause'}
    };

rows = struct('time', {}, 'channel', {}, 'field', {}, 'value', {});
for c = 1:size(chan, 1)
    el = out.logsout.find('Name', chan{c, 1});
    if isempty(el)
        fprintf('(%s was not logged in this run)\n', chan{c, 1});
        continue
    end
    v = el{1}.Values;
    for f = chan{c, 2}
        if ~isfield(v, f{1})
            continue
        end
        rows = collect(rows, chan{c, 1}, f{1}, v.(f{1}), cfg);
    end
end

T = struct2table(rows);
if isempty(T)
    fprintf('nothing discrete changed in this run.\n');
    return
end
T = sortrows(T, {'time', 'channel', 'field'});

fprintf('\n%-9s %-18s %-16s %s\n', 't (s)', 'channel', 'field', 'value');
fprintf('%s\n', repmat('-', 1, 78));
for k = 1:height(T)
    fprintf('%-9.3f %-18s %-16s %s\n', T.time(k), T.channel{k}, T.field{k}, T.value{k});
end

if nargout == 0
    clear T
end

end

%% ------------------------------------------------------------------------
function rows = collect(rows, channel, field, ts, cfg)
%COLLECT One row per change of a discrete series, including its first sample.
%   Vector fields - the action queue carries one Verb and one ExecState per slot - change
%   as a whole, so a change in any element emits one row holding the whole decoded row.
d = flatten(ts.Data);
k = [1; find(any(diff(d, 1, 1) ~= 0, 2)) + 1];
for i = k(:)'
    rows(end + 1) = struct('time', ts.Time(i), 'channel', channel, ...
        'field', field, 'value', decodeRow(channel, field, d(i, :), cfg)); %#ok<AGROW>
end
end

%% ------------------------------------------------------------------------
function d = flatten(data)
%FLATTEN Any logged leaf as nSamples-by-nElements of double.
%   Simulink logs a scalar leaf as a column, a vector leaf as rows-by-1-by-samples and a
%   matrix leaf as rows-by-cols-by-samples, so the sample dimension is last except when it
%   is first. Only the number of samples tells the two apart.
if ndims(data) == 3
    d = reshape(data, [], size(data, 3))';
else
    d = reshape(data, size(data, 1), []);
end
d = double(d);
end

%% ------------------------------------------------------------------------
function s = decodeRow(channel, field, x, cfg)
%DECODEROW One sample of a field as text, element by element for vector fields.
%   Long queue rows are truncated because eight slots of an eight-slot queue that has two
%   entries is six repetitions of the same word.
n = min(numel(x), 8);
parts = strings(1, n);
for i = 1:n
    parts(i) = decode(channel, field, x(i), cfg);
end
s = char(strjoin(parts, ' '));
if numel(x) > n
    s = [s ' ...'];
end
end

%% ------------------------------------------------------------------------
function s = decode(channel, field, x, cfg)
%DECODE A raw code as the name the configuration gives it, or as a number if none.
%   The lookup is BY VALUE over the relevant cfg struct rather than by a table repeated
%   here, so renaming or renumbering a mode in hmmConfig cannot leave this lying.
switch field
    case {'FailureCode', 'ActionFailure', 'ItemFailure'}
        s = nameOf(cfg.fail, x, '');
    case {'Cause', 'EStopCause'}
        s = nameOf(cfg.sup.cause, x, '');
    case {'EStop', 'Stalled', 'ReplanRequest', 'Slipped', 'Grasped', 'Overflow'}
        s = boolName(field, x);
    case 'PlannerMode'
        s = nameOf(cfg.sup.pl, x, '');
    case 'Outcome'
        s = nameOf(cfg.sup.out, x, '');
    case {'ActiveVerb', 'Verb'}
        s = nameOf(cfg.queue, x, 'VERB_');
    case {'ActionState', 'ExecState'}
        s = nameOf(cfg.queue, x, 'EXEC_');
    case 'Route'
        s = nameOf(cfg.provider, x, 'ROUTE_');
    case 'Mode'
        s = nameOf(modeEnum(cfg.sup, channel), x, '');
    case 'State'
        s = nameOf(cfg.sup.st, x, '');
    otherwise
        s = '';                     % a plain count or id: the number is the information
end
if isempty(s)
    s = sprintf('%g', x);
else
    s = sprintf('%s (%g)', s, x);
end
end

%% ------------------------------------------------------------------------
function sub = modeEnum(supCfg, channel)
%MODEENUM The cfg.sup sub-struct that enumerates one channel's Mode.
switch channel
    case 'BaseStatus',    name = 'base';
    case 'ArmStatus',     name = 'arm';
    case 'GripperStatus', name = 'grip';
    otherwise,            name = '';
end
if ~isempty(name) && isfield(supCfg, name)
    sub = supCfg.(name);
else
    sub = struct();
end
end

%% ------------------------------------------------------------------------
function name = nameOf(s, x, prefix)
%NAMEOF The field of a code struct whose value is x, or empty if there is no such field.
%   PREFIX restricts the search to one family within a shared struct - cfg.queue holds
%   both VERB_ and EXEC_ codes numbered from zero, and cfg.provider holds four ROUTE_
%   codes among endpoints and tolerances that are also small numbers - and is stripped
%   from the answer.
%
%   Count fields such as nCauses are skipped: they are sizes, not codes, and nCauses
%   happens to equal STALE_PROVIDER, so matching one would label a cause with a total.
name = '';
if ~isstruct(s)
    return
end
for f = string(fieldnames(s))'
    if ~startsWith(f, prefix) || ~isempty(regexp(f, '^n[A-Z]', 'once'))
        continue
    end
    v = s.(f);
    if isscalar(v) && isnumeric(v) && double(v) == x
        name = char(extractAfter(f, strlength(prefix)));
        return
    end
end
end

%% ------------------------------------------------------------------------
function s = boolName(field, x)
%BOOLNAME A flag as the word that says which way it is set.
if strcmp(field, 'EStop')
    s = ternary(x ~= 0, 'LATCHED', 'clear');
else
    s = ternary(x ~= 0, ['+' field], ['-' field]);
end
end

%% ------------------------------------------------------------------------
function v = ternary(tf, a, b)
%TERNARY Pick one of two values, so decode stays a flat switch.
if tf
    v = a;
else
    v = b;
end
end
