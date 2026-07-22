function taSpec = reqRenderToTA(spec)
%REQRENDERTOTA Render a requirement spec to Test Assessment block artifacts.
%
%   TASPEC = reqRenderToTA(SPEC) takes a requirement specification struct
%   (created by reqCreateSpec) and produces the information needed to
%   create a Test Assessment block in Simulink Test.
%
%   Output struct fields:
%     taSpec.renderable   - true if this spec can be rendered to TA
%     taSpec.reason       - explanation if not renderable
%     taSpec.type         - 'custom' | 'trigger_response' | 'bounds_check'
%     taSpec.summary      - assessment description
%
%   For type='custom':
%     taSpec.guard        - guard expression (if non-empty, verify only
%                           fires when guard is true; UNTESTED otherwise)
%     taSpec.expression   - custom formula (MATLAB syntax)
%
%   For type='trigger_response':
%     taSpec.trigger.type       - 'whenever_true' | 'becomes_true' |
%                                 'becomes_true_stays'
%     taSpec.trigger.expr       - trigger expression (MATLAB syntax)
%     taSpec.trigger.duration   - hold duration for 'becomes_true_stays'
%     taSpec.response.type      - 'must_be_true' | 'must_stay_true_at_least'
%                                 | 'must_stay_true_at_most' |
%                                 'must_stay_true_between' | 'weak_until'
%     taSpec.response.expr      - response expression (MATLAB syntax)
%     taSpec.response.duration  - duration value(s) for stay-true responses
%     taSpec.response.untilExpr - until condition for weak_until
%     taSpec.delay.type         - 'none' | 'at_most' | 'between'
%     taSpec.delay.value        - delay value(s) in seconds
%
%   For type='bounds_check':
%     taSpec.signal     - signal name
%     taSpec.lowerBound - lower bound value
%     taSpec.upperBound - upper bound value
%
%   Example:
%     spec = reqCreateSpec('REQ-07', ...
%         'When brake pressed, disengage within 0.5s.', ...
%         'TriggerResponse', ...
%         TriggerType='level', TriggerExpr='brake_pressed == 1', ...
%         ResponseType='must_be_true', ResponseExpr='cruise == 0', ...
%         DelayType='at_most', DelayValue=0.5);
%     ta = reqRenderToTA(spec)
%
%   See also reqCreateSpec, reqCheckRenderability, reqRenderToRT

    arguments
        spec (1,1) struct
    end

    taSpec.renderable = false;
    taSpec.reason     = "";
    taSpec.type       = "";
    taSpec.summary    = "";

    % Check renderability first
    rend = reqCheckRenderability(spec);
    if ~rend.ta
        taSpec.reason = rend.reason;
        return
    end

    taSpec.renderable = true;

    switch spec.pattern
        case {'Invariant', 'BoundedOutput', 'ModeDependent'}
            taSpec = renderInvariantToTA(spec, taSpec);

        case 'TriggerResponse'
            taSpec = renderTriggerResponseToTA(spec, taSpec);

        case 'Persistence'
            taSpec = renderPersistenceToTA(spec, taSpec);

        case 'DurationLimit'
            taSpec = renderDurationLimitToTA(spec, taSpec);

        case 'Latching'
            taSpec = renderLatchingToTA(spec, taSpec);

        otherwise
            taSpec.renderable = false;
            taSpec.reason = sprintf("Pattern '%s' has no TA renderer.", spec.pattern);
    end
end

%% --- Invariant → TA (custom assessment) ---
function taSpec = renderInvariantToTA(spec, taSpec)
    taSpec.type = "custom";
    taSpec.summary = sprintf('%s: %s', spec.id, spec.description);

    if spec.guard ~= ""
        % Guarded invariant: verify only when guard is active.
        % Uses if-guard so unexercised requirements show UNTESTED (not
        % vacuous PASS), matching the structured editor semantics.
        guard = toTASyntax(spec.guard);
        prop  = toTASyntax(spec.property);
        taSpec.guard      = guard;
        taSpec.expression = prop;
    else
        % always (property) → custom: property
        taSpec.guard      = "";
        taSpec.expression = toTASyntax(spec.property);
    end
end

%% --- TriggerResponse → TA ---
function taSpec = renderTriggerResponseToTA(spec, taSpec)
    taSpec.type = "trigger_response";
    taSpec.summary = sprintf('%s: %s', spec.id, spec.description);

    % Map trigger
    taSpec.trigger = mapTrigger(spec);

    % Map response
    taSpec.response = mapResponse(spec);

    % Map delay
    taSpec.delay = mapDelay(spec);
end

%% --- Persistence → TA ---
function taSpec = renderPersistenceToTA(spec, taSpec)
    taSpec.type = "trigger_response";
    taSpec.summary = sprintf('%s: %s', spec.id, spec.description);

    % Persistence: always (rise(T) -> hold_at_*(R, dur))
    taSpec.trigger = mapTrigger(spec);
    taSpec.response = mapResponse(spec);
    taSpec.delay.type  = "none";
    taSpec.delay.value = [];
end

%% --- DurationLimit → TA ---
function taSpec = renderDurationLimitToTA(spec, taSpec)
    taSpec.type = "trigger_response";
    taSpec.summary = sprintf('%s: %s', spec.id, spec.description);

    % Duration limit: always (P -> eventually[0,T] (not P))
    % Maps to: whenever P is true → within T seconds, not P must be true
    taSpec.trigger.type     = "whenever_true";
    taSpec.trigger.expr     = toTASyntax(spec.trigger.expr);
    taSpec.trigger.duration = [];

    taSpec.response.type     = "must_be_true";
    taSpec.response.expr     = toTASyntax(spec.response.expr);
    taSpec.response.duration = [];
    taSpec.response.untilExpr = "";

    taSpec.delay = mapDelay(spec);
end

%% --- Latching → TA ---
function taSpec = renderLatchingToTA(spec, taSpec)
    taSpec.type = "trigger_response";
    taSpec.summary = sprintf('%s: %s', spec.id, spec.description);

    % Latching: always (rise(X) -> always(X))
    % Approximate: becomes_true → must_stay_true_at_least(T_large)
    T_LARGE = 1e6;  % large constant for "forever"

    taSpec.trigger.type     = "becomes_true";
    taSpec.trigger.expr     = toTASyntax(spec.trigger.expr);
    taSpec.trigger.duration = [];

    taSpec.response.type     = "must_stay_true_at_least";
    taSpec.response.expr     = toTASyntax(spec.response.expr);
    taSpec.response.duration = T_LARGE;
    taSpec.response.untilExpr = "";

    taSpec.delay.type  = "none";
    taSpec.delay.value = [];
end

%% --- Trigger mapping ---
function trig = mapTrigger(spec)
    trig.expr = toTASyntax(spec.trigger.expr);
    trig.duration = spec.trigger.duration;

    switch spec.trigger.type
        case 'level'
            if ~isempty(spec.trigger.duration)
                trig.type = "becomes_true_stays";
            else
                trig.type = "whenever_true";
            end
        case 'rising_edge'
            if ~isempty(spec.trigger.duration)
                trig.type = "becomes_true_stays";
            else
                trig.type = "becomes_true";
            end
        case 'falling_edge'
            % TA triggers are rising-edge based.
            % Rewrite: fall(P) → rise(~P)
            trig.type = "becomes_true";
            trig.expr = sprintf('~(%s)', toTASyntax(spec.trigger.expr));
            trig.duration = spec.trigger.duration;
        otherwise
            trig.type = "whenever_true";
    end
end

%% --- Response mapping ---
function resp = mapResponse(spec)
    resp.expr      = toTASyntax(spec.response.expr);
    resp.duration   = spec.response.duration;
    resp.untilExpr  = "";

    switch spec.response.type
        case 'must_be_true'
            resp.type = "must_be_true";
        case 'hold_at_least'
            resp.type = "must_stay_true_at_least";
        case 'hold_at_most'
            resp.type = "must_stay_true_at_most";
        case 'hold_between'
            resp.type = "must_stay_true_between";
        case 'weak_until'
            resp.type = "weak_until";
            resp.untilExpr = toTASyntax(spec.response.untilExpr);
        otherwise
            resp.type = "must_be_true";
    end
end

%% --- Delay mapping ---
function d = mapDelay(spec)
    d.type  = spec.delay.type;
    d.value = spec.delay.value;
end

%% --- Syntax translation ---
function out = toTASyntax(expr)
    out = string(expr);
    % TA blocks use & and | (not && ||) — Stateflow step action syntax
    out = regexprep(out, '\<and\>',  '&');
    out = regexprep(out, '\<or\>',   '|');
    out = regexprep(out, '\<not\>',  '~');
    out = strrep(out, '!=', '~=');
end
