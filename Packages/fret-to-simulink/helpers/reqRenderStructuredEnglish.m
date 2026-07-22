function english = reqRenderStructuredEnglish(spec)
%REQRENDERSTRUCTUREDENGLISH Render a requirement spec as structured English.
%
%   ENGLISH = reqRenderStructuredEnglish(SPEC) takes a requirement
%   specification struct (created by reqCreateSpec) and produces a
%   deterministic structured English sentence for human review.
%
%   The rendering is a mechanical 1-to-1 mapping from the spec struct to
%   English. It is NOT a creative rewrite — distinct specs produce distinct
%   sentences, and an engineer can mentally "unfold" the English back to
%   the specification. This is the review mechanism: the engineer checks
%   that the structured English captures their intent.
%
%   Example:
%     spec = reqCreateSpec('TSM-R01', ...
%         'Output shall equal median of inputs.', 'Invariant', ...
%         Guard='FC == 0', ...
%         Property='sel_val == median(ia, ib, ic)');
%     english = reqRenderStructuredEnglish(spec)
%     % Returns: "At any point in time, if FC == 0, then sel_val == median(ia, ib, ic)."
%
%   See also reqCreateSpec, reqCheckRenderability

    arguments
        spec (1,1) struct
    end

    validateSpec(spec);

    switch spec.pattern
        case 'Summary'
            english = sprintf('[%s] Summary requirement — not formalizable. See child requirements.', ...
                spec.id);
            return

        case 'Initialization'
            english = sprintf('At initialization, %s.', ...
                renderExpr(spec.initProperty));
            return

        case {'Invariant', 'BoundedOutput', 'ModeDependent'}
            english = renderGuardedInvariant(spec);
            return

        case {'TriggerResponse', 'Persistence', 'DurationLimit', 'Latching'}
            english = renderTriggerResponse(spec);
            return

        otherwise
            english = sprintf('[%s] Unrecognized pattern "%s".', ...
                spec.id, spec.pattern);
    end
end

%% --- Invariant / BoundedOutput / ModeDependent ---
function english = renderGuardedInvariant(spec)
    prop = renderExpr(spec.property);

    if spec.guard == ""
        english = sprintf('At any point in time, %s.', prop);
    else
        guard = renderExpr(spec.guard);
        english = sprintf('At any point in time, if %s, then %s.', guard, prop);
    end
end

%% --- TriggerResponse / Persistence / DurationLimit / Latching ---
function english = renderTriggerResponse(spec)
    trigSeg  = buildTriggerSegment(spec);
    delaySeg = buildDelaySegment(spec);
    respSeg  = buildResponseSegment(spec);

    if delaySeg == ""
        english = sprintf('At any point in time, if %s, then %s.', ...
            trigSeg, respSeg);
    else
        english = sprintf('At any point in time, if %s, then %s, %s.', ...
            trigSeg, delaySeg, respSeg);
    end
end

function seg = buildTriggerSegment(spec)
    expr = renderExpr(spec.trigger.expr);

    switch spec.trigger.type
        case 'level'
            seg = sprintf('%s is true', expr);
        case 'rising_edge'
            seg = sprintf('%s becomes true', expr);
        case 'falling_edge'
            seg = sprintf('%s becomes false', expr);
        otherwise
            seg = expr;
    end

    % Add persistence if specified
    if ~isempty(spec.trigger.duration)
        seg = sprintf('%s and stays true for at least %.4g seconds', ...
            seg, spec.trigger.duration);
    end

    % Special case: Latching pattern
    if spec.pattern == "Latching"
        seg = sprintf('%s becomes true', renderExpr(spec.trigger.expr));
    end
end

function seg = buildDelaySegment(spec)
    switch spec.delay.type
        case 'none'
            seg = "";
        case 'at_most'
            seg = sprintf('within at most %.4g seconds', spec.delay.value);
        case 'between'
            seg = sprintf('within %.4g to %.4g seconds', ...
                spec.delay.value(1), spec.delay.value(2));
        otherwise
            seg = "";
    end
end

function seg = buildResponseSegment(spec)
    expr = renderExpr(spec.response.expr);

    switch spec.response.type
        case 'must_be_true'
            if spec.pattern == "Latching"
                seg = sprintf('%s must remain true permanently', expr);
            else
                seg = sprintf('%s must be true', expr);
            end
        case 'hold_at_least'
            seg = sprintf('%s must stay true for at least %.4g seconds', ...
                expr, spec.response.duration);
        case 'hold_at_most'
            seg = sprintf('%s must stay true for at most %.4g seconds', ...
                expr, spec.response.duration);
        case 'hold_between'
            seg = sprintf('%s must stay true for between %.4g and %.4g seconds', ...
                expr, spec.response.duration(1), spec.response.duration(2));
        case 'weak_until'
            until = renderExpr(spec.response.untilExpr);
            seg = sprintf('%s must stay true until %s (or indefinitely)', ...
                expr, until);
        otherwise
            seg = expr;
    end
end

%% --- Expression rendering ---
function out = renderExpr(expr)
    % Pass through the expression as-is. Expressions use MATLAB syntax
    % (model-grounded signal names) which engineers can read directly.
    out = string(expr);
end

%% --- Validation ---
function validateSpec(spec)
    requiredFields = {'id', 'nl', 'pattern'};
    for i = 1:numel(requiredFields)
        if ~isfield(spec, requiredFields{i})
            error('reqRenderStructuredEnglish:InvalidSpec', ...
                'Spec struct is missing required field "%s". Use reqCreateSpec to build specs.', ...
                requiredFields{i});
        end
    end
end
