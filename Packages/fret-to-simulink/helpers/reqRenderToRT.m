function rtSpec = reqRenderToRT(spec)
%REQRENDERTORT Render a requirement spec to Requirements Table artifacts.
%
%   RTSPEC = reqRenderToRT(SPEC) takes a requirement specification struct
%   (created by reqCreateSpec) and produces the information needed to
%   create a Requirements Table row in Simulink.
%
%   Output struct fields:
%     rtSpec.renderable   - true if this spec can be rendered to RT
%     rtSpec.reason       - explanation if not renderable
%     rtSpec.summary      - row summary string
%     rtSpec.precondition - precondition expression (MATLAB syntax), '' if unconditional
%     rtSpec.postcondition - postcondition expression (MATLAB syntax)
%     rtSpec.duration     - duration value for Duration column, [] if none
%     rtSpec.rowType      - 'requirement' or 'assumption'
%
%   The returned precondition/postcondition strings use MATLAB syntax:
%     && (and), || (or), ~ (not), ~= (not equal)
%
%   Example:
%     spec = reqCreateSpec('TSM-R01', ...
%         'Output equals median when no faults.', 'Invariant', ...
%         Guard='FC == 0', ...
%         Property='sel_val == median(ia, ib, ic)');
%     rt = reqRenderToRT(spec)
%     % rt.precondition  = 'FC == 0'
%     % rt.postcondition = 'sel_val == median(ia, ib, ic)'
%
%   See also reqCreateSpec, reqCheckRenderability, reqRenderToTA

    arguments
        spec (1,1) struct
    end

    rtSpec.renderable    = false;
    rtSpec.reason        = "";
    rtSpec.summary       = "";
    rtSpec.precondition  = "";
    rtSpec.postcondition = "";
    rtSpec.duration      = [];
    rtSpec.rowType       = "requirement";

    % Check renderability first
    rend = reqCheckRenderability(spec);
    if ~rend.rt
        rtSpec.reason = rend.reason;
        return
    end

    rtSpec.renderable = true;

    switch spec.pattern
        case {'Invariant', 'BoundedOutput', 'ModeDependent'}
            rtSpec = renderInvariantToRT(spec, rtSpec);

        case 'TriggerResponse'
            rtSpec = renderTriggerResponseToRT(spec, rtSpec);

        otherwise
            rtSpec.renderable = false;
            rtSpec.reason = sprintf("Pattern '%s' passed renderability check but has no RT renderer.", ...
                spec.pattern);
    end
end

%% --- Invariant pattern ---
function rtSpec = renderInvariantToRT(spec, rtSpec)
    % always (guard -> property) OR always (property)
    if spec.description ~= ""
        rtSpec.summary = sprintf('%s: %s', spec.id, spec.description);
    else
        rtSpec.summary = string(spec.id);
    end

    if spec.guard ~= ""
        rtSpec.precondition = toRTSyntax(spec.guard);
    end
    rtSpec.postcondition = toRTSyntax(spec.property);

end

%% --- TriggerResponse pattern ---
function rtSpec = renderTriggerResponseToRT(spec, rtSpec)
    if spec.description ~= ""
        rtSpec.summary = sprintf('%s: %s', spec.id, spec.description);
    else
        rtSpec.summary = string(spec.id);
    end

    if spec.trigger.type == "rising_edge"
        % Edge-triggered: use prev()-based edge detection
        % Rising edge of C = (C) && ~prev(C)
        trigExpr = toRTSyntax(spec.trigger.expr);
        rtSpec.precondition = sprintf('(%s) && ~prev(%s)', trigExpr, trigExpr);
    else
        % Level-triggered: precondition is the trigger expression directly
        rtSpec.precondition = toRTSyntax(spec.trigger.expr);
    end

    rtSpec.postcondition = toRTSyntax(spec.response.expr);

    % Persistent trigger → Duration column
    if ~isempty(spec.trigger.duration)
        rtSpec.duration = spec.trigger.duration;
    end

end

%% --- Syntax translation ---
function out = toRTSyntax(expr)
    % Convert specification syntax to MATLAB/Stateflow syntax.
    % Most expressions are already in MATLAB syntax when created via
    % reqCreateSpec, but handle common patterns defensively.
    out = string(expr);

    % Normalize whitespace-padded logical operators if present in
    % natural-language-style expressions
    out = regexprep(out, '\<and\>',  '&&');
    out = regexprep(out, '\<or\>',   '||');
    out = regexprep(out, '\<not\>',  '~');

    % != to ~= (MATLAB style)
    out = strrep(out, '!=', '~=');
end

