function result = reqCheckRenderability(spec)
%REQCHECKRENDERABILITY Check which rendering targets a requirement supports.
%
%   RESULT = reqCheckRenderability(SPEC) takes a requirement specification
%   struct (created by reqCreateSpec) and determines which rendering
%   targets are available.
%
%   Output struct fields:
%     result.rt     - true if renderable to Requirements Table block
%     result.ta     - true if renderable to Test Assessment block
%     result.reason - string explaining any limitations
%     result.recommended - string: 'RT', 'TA', or 'Both'
%
%   Rendering target capabilities:
%     Requirements Table: State-based properties only. Supports
%       precondition/postcondition pairs with optional duration column.
%       No temporal operators in postconditions.
%     Test Assessment block: Flat trigger-response template with optional
%       delay. Supports edge triggers, persistence, weak_until.
%       No nested temporal operators.
%
%   Example:
%     spec = reqCreateSpec('REQ-01', 'Output >= 0', 'Invariant', ...
%         Property='output >= 0');
%     r = reqCheckRenderability(spec)
%     % r.rt = true, r.ta = true
%
%   See also reqCreateSpec, reqRenderToRT, reqRenderToTA

    arguments
        spec (1,1) struct
    end

    result.rt     = false;
    result.ta     = false;
    result.reason = "";
    result.recommended = "";

    switch spec.pattern
        case 'Summary'
            result.reason = "Summary/parent requirement — not formalizable.";
            result.recommended = "None";
            return

        case {'Invariant', 'BoundedOutput', 'ModeDependent'}
            result.rt     = true;
            result.ta     = true;
            result.reason = "State-based invariant — both targets supported.";
            result.recommended = "Both";

        case 'Initialization'
            result.rt     = false;
            result.ta     = false;
            result.reason = "Initialization property (t=0 only). " + ...
                "RT and TA do not support initial-state-only assertions.";
            result.recommended = "None";

        case 'TriggerResponse'
            result = checkTriggerResponseRenderability(spec, result);

        case 'Persistence'
            result = checkPersistenceRenderability(spec, result);

        case 'DurationLimit'
            result.ta     = true;
            result.rt     = false;
            result.reason = "Duration limit uses temporal response (eventually). " + ...
                "RT does not support temporal operators in postconditions.";
            result.recommended = "TA";

        case 'Latching'
            result = checkLatchingRenderability(spec, result);

        otherwise
            result.reason = sprintf("Unknown pattern '%s'.", spec.pattern);
            result.recommended = "None";
    end
end

%% --- Pattern-specific renderability checks ---

function result = checkTriggerResponseRenderability(spec, result)
    % TA: supports flat trigger -> delay -> response
    result.ta = true;

    % RT: level triggers always work; rising_edge triggers can use
    % prev()-based edge detection: Pre=(C) && ~prev(C), Post=P
    isLevelTrigger    = spec.trigger.type == "level";
    isEdgeTrigger     = spec.trigger.type == "rising_edge";
    hasNoPersistence  = isempty(spec.trigger.duration);
    noDelay           = spec.delay.type == "none";
    simpleResponse    = spec.response.type == "must_be_true";

    if isLevelTrigger && noDelay && simpleResponse
        result.rt = true;
        result.reason = "Level-triggered with no delay and simple response — both targets.";
        result.recommended = "Both";
    elseif isLevelTrigger && ~hasNoPersistence && simpleResponse
        result.rt = true;
        result.reason = "Persistent trigger with simple response — RT uses Duration column.";
        result.recommended = "Both";
    elseif isEdgeTrigger && noDelay && simpleResponse
        result.rt = true;
        result.reason = "Edge-triggered immediate — RT uses prev()-based edge detection.";
        result.recommended = "Both";
    else
        result.rt = false;
        reasons = strings(0);
        if ~isLevelTrigger && ~isEdgeTrigger
            reasons(end+1) = "trigger type not supported in RT";
        elseif isEdgeTrigger && ~noDelay
            reasons(end+1) = "edge trigger with delay not supported in RT";
        elseif isEdgeTrigger && ~simpleResponse
            reasons(end+1) = sprintf("edge trigger with response type '%s' not supported in RT", ...
                spec.response.type);
        end
        if ~noDelay && ~isEdgeTrigger
            reasons(end+1) = "delay (eventually) not supported in RT";
        end
        if ~simpleResponse && ~isEdgeTrigger
            reasons(end+1) = sprintf("response type '%s' not supported in RT", ...
                spec.response.type);
        end
        result.reason = "TA only — " + join(reasons, "; ") + ".";
        result.recommended = "TA";
    end
end

function result = checkPersistenceRenderability(~, result)
    % Persistence: always (rise(T) -> hold_at_*(R, duration))
    result.ta = true;
    result.rt = false;
    result.reason = "Persistence pattern uses edge trigger and temporal response. " + ...
        "Not RT-renderable; use TA.";
    result.recommended = "TA";
end

function result = checkLatchingRenderability(~, result)
    % Latching: always (rise(X) -> always(X))
    % TA: can approximate with hold_at_least(X, T_large)
    result.ta = true;
    result.rt = false;
    result.reason = "Latching uses edge trigger and unbounded temporal response. " + ...
        "TA renders with large hold duration. Not RT-renderable.";
    result.recommended = "TA";
end
