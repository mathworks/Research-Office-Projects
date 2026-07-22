function spec = fretToSpec(id, nl, fretResult, fretVars)
%FRETTOSPEC Convert a FRET formalization result to a reqCreateSpec struct.
%
%   spec = fretToSpec(ID, NL, FRETRESULT) takes a FRET requirement ID,
%   the original natural language text, and the FRET semantics result
%   struct (from FRET's compile() or from a FRET JSON export), and
%   produces a reqCreateSpec-compatible struct that can be passed to
%   reqRenderToRT and reqRenderToTA.
%
%   spec = fretToSpec(ID, NL, FRETRESULT, FRETVARS) also takes a FRET
%   variable mapping array (from the FRET JSON "variables" field). When
%   provided, the adapter performs mechanical substitution:
%     - Internal variables with assignments are replaced by their constant
%       values (e.g., ap_standby_state -> 3.0)
%     - Variable idType (Input/Output/Internal) determines RT symbol scope
%   This matches CoCoSim's approach: 1:1 name-to-value substitution with
%   no semantic interpretation.
%
%   FRETRESULT must contain the following fields from FRET's output:
%     .scope.type           - 'null', 'in', 'after', etc.
%     .condition            - 'null', 'regular', 'holding'
%     .timing               - 'always', 'immediately', 'within', etc.
%     .post_condition       - the satisfy <expr> expression
%     .regular_condition    - the when/if <expr> expression (if condition ~= 'null')
%     .component_name       - the component name
%     .ftExpanded           - the fmLTL formula (for reference)
%
%   FRETVARS (optional) is a struct array with fields:
%     .variable_name  - variable identifier used in FRETish
%     .idType         - 'Input', 'Output', 'Internal', 'Mode'
%     .assignment     - constant value string (for Internal variables)
%     .dataType       - 'boolean', 'double', 'integer', etc.
%
%   Example (from FRET JSON export with variable mapping):
%     fretData = jsondecode(fileread('project.json'));
%     req = fretData.requirements(1);
%     vars = fretData.variables;
%     spec = fretToSpec(req.reqid, req.fulltext, req.semantics, vars);
%     rt = reqRenderToRT(spec);
%
%   See also reqCreateSpec, reqRenderToRT, reqRenderToTA

    arguments
        id         (1,1) string
        nl         (1,1) string
        fretResult (1,1) struct
        fretVars          = {}
    end

    % Large duration constant for "forever" in persistence/latching patterns.
    % TA renders hold_at_least(T_PERSIST) as the finite approximation of G(P).
    T_PERSIST = 1e6;

    % Extract FRET fields
    scopeType = extractField(fretResult, 'scope');
    condition = string(fretResult.condition);
    timing    = string(fretResult.timing);

    % Extract expressions, converting from SMV to MATLAB syntax
    postCond = smvToMatlab(extractPostCondition(fretResult));
    regCond  = smvToMatlab(extractRegularCondition(fretResult));

    % Mechanical constant substitution from FRET variable mapping
    % Internal variables with assignments are replaced by their values.
    % Exception: Internals whose assignment references an Output variable
    % are kept as symbols (they are mode guards, not true constants).
    if ~isempty(fretVars)
        % Accept both struct array and cell array of structs
        if iscell(fretVars)
            varsArr = fretVars;
        else
            varsArr = num2cell(fretVars);
        end
        outputNames = collectOutputNames(varsArr);
        % Two-pass substitution on postconditions:
        % Pass 1: substitute with outputNames protection so mode guards
        %   (Internals referencing Outputs, e.g. "normal") stay as symbols.
        % Pass 2: split any top-level implication, then substitute freely
        %   on the property side only. This allows Internals like
        %   "roll_command_acceleration" to expand in postconditions while
        %   keeping mode guards unexpanded in the guard/precondition side.
        postCond = substituteConstants(postCond, varsArr, outputNames);
        [implGuard, implProp] = splitImplication(postCond);
        if implGuard ~= ""
            implProp = substituteConstants(implProp, varsArr, {});
            postCond = implGuard + " -> " + implProp;
        else
            postCond = substituteConstants(postCond, varsArr, {});
        end
        regCond  = substituteConstants(regCond, varsArr, outputNames);
    end

    % Extract duration and stop condition if present
    duration = extractDuration(fretResult);
    stopCond = smvToMatlab(extractStopCondition(fretResult));
    if ~isempty(fretVars)
        if iscell(fretVars), varsArr = fretVars; else, varsArr = num2cell(fretVars); end
        stopCond = substituteConstants(stopCond, varsArr, outputNames);
    end

    % Component name for description
    component = "";
    if isfield(fretResult, 'component_name')
        component = string(fretResult.component_name);
    elseif isfield(fretResult, 'component')
        component = string(fretResult.component);
    end

    % Scope mode (for 'in' scope: the mode/condition that scopes the requirement)
    scopeMode = "";
    if scopeType == "in" || scopeType == "after"
        if isfield(fretResult, 'scope_mode')
            scopeMode = smvToMatlab(string(fretResult.scope_mode));
            if ~isempty(fretVars)
                scopeMode = substituteConstants(scopeMode, varsArr, outputNames);
            end
        end
    end

    % Determine the pattern and build the spec
    templateKey = sprintf('%s,%s,%s', scopeType, condition, timing);

    switch templateKey
        % --- Unconditional invariant ---
        case 'null,null,always'
            % "Component shall always satisfy post"
            % Check if postcondition contains implication (guard -> prop)
            [guard, prop] = splitImplication(postCond);
            if guard ~= ""
                spec = reqCreateSpec(id, nl, 'Invariant', ...
                    Guard=guard, Property=prop, ...
                    Description=buildDescription(component, "guarded invariant"));
            else
                spec = reqCreateSpec(id, nl, 'Invariant', ...
                    Property=postCond, ...
                    Description=buildDescription(component, "invariant"));
            end

        % --- Scoped invariant (in mode) ---
        case 'in,null,always'
            % "In mode, component shall always satisfy post"
            % FRET formula: G(scope_mode -> post_condition)
            % scope_mode becomes the guard
            [guard, prop] = splitImplication(postCond);
            if scopeMode ~= "" && guard == ""
                % Scope mode is the guard, postcondition is the property
                spec = reqCreateSpec(id, nl, 'Invariant', ...
                    Guard=scopeMode, Property=postCond, ...
                    Description=buildDescription(component, "scoped invariant (in mode)"));
            elseif scopeMode ~= "" && guard ~= ""
                % Both scope mode and implication guard: conjoin
                combinedGuard = sprintf('(%s) && (%s)', scopeMode, guard);
                spec = reqCreateSpec(id, nl, 'Invariant', ...
                    Guard=combinedGuard, Property=prop, ...
                    Description=buildDescription(component, "scoped guarded invariant (in mode)"));
            else
                % Fallback: no scope_mode available, treat like null scope
                if guard ~= ""
                    spec = reqCreateSpec(id, nl, 'Invariant', ...
                        Guard=guard, Property=prop, ...
                        Description=buildDescription(component, "guarded invariant"));
                else
                    spec = reqCreateSpec(id, nl, 'Invariant', ...
                        Property=postCond, ...
                        Description=buildDescription(component, "invariant"));
                end
            end

        % --- Initialization / noTrigger immediate ---
        case {'null,null,immediately', 'null,noTrigger,immediately'}
            % noTrigger with a regular_condition is actually a conditional
            % immediate: "whenever cond, shall immediately satisfy post"
            if regCond ~= ""
                spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                    TriggerType='level', ...
                    TriggerExpr=regCond, ...
                    ResponseType='must_be_true', ...
                    ResponseExpr=expandResidualImplication(postCond), ...
                    Description=buildDescription(component, "conditional immediate (noTrigger)"));
            else
                [guard, prop] = splitImplication(postCond);
                if guard ~= ""
                    spec = reqCreateSpec(id, nl, 'Invariant', ...
                        Guard=guard, Property=prop, ...
                        Description=buildDescription(component, "initialization (guarded)"));
                else
                    spec = reqCreateSpec(id, nl, 'Initialization', ...
                        InitProperty=postCond, ...
                        Description=buildDescription(component, "initialization"));
                end
            end

        % --- Scoped immediate (on entering mode) ---
        case 'in,null,immediately'
            % "In mode, component shall immediately satisfy post"
            % FRET semantics: on rising edge of scope_mode, postcondition holds
            if scopeMode ~= ""
                [guard, prop] = splitImplication(postCond);
                if guard ~= ""
                    respExpr = sprintf('~(%s) || (%s)', guard, prop);
                else
                    respExpr = postCond;
                end
                spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                    TriggerType='rising_edge', ...
                    TriggerExpr=scopeMode, ...
                    ResponseType='must_be_true', ...
                    ResponseExpr=respExpr, ...
                    Description=buildDescription(component, "scoped immediate (on mode entry)"));
            else
                % Fallback: no scope_mode, treat as initialization
                [guard, prop] = splitImplication(postCond);
                if guard ~= ""
                    spec = reqCreateSpec(id, nl, 'Invariant', ...
                        Guard=guard, Property=prop, ...
                        Description=buildDescription(component, "initialization (guarded)"));
                else
                    spec = reqCreateSpec(id, nl, 'Initialization', ...
                        InitProperty=postCond, ...
                        Description=buildDescription(component, "initialization"));
                end
            end

        % --- Conditional always (edge-triggered, persistent obligation) ---
        case 'null,regular,always'
            % "When cond, component shall always satisfy post"
            % FRET semantics: on rising edge of cond, post must hold
            % from that point FOREVER (even if cond becomes false again).
            % This is a persistent obligation — NOT a guarded invariant.
            if isLatchingPattern(regCond, postCond)
                spec = reqCreateSpec(id, nl, 'Latching', ...
                    TriggerType='rising_edge', ...
                    TriggerExpr=regCond, ...
                    ResponseType='must_be_true', ...
                    ResponseExpr=expandResidualImplication(postCond), ...
                    Description=buildDescription(component, "latching"));
            else
                spec = reqCreateSpec(id, nl, 'Persistence', ...
                    TriggerType='rising_edge', ...
                    TriggerExpr=regCond, ...
                    ResponseType='hold_at_least', ...
                    ResponseExpr=expandResidualImplication(postCond), ...
                    ResponseDuration=T_PERSIST, ...
                    Description=buildDescription(component, "persistent obligation (edge-triggered)"));
            end

        % --- Scoped conditional always (in mode, edge-triggered, persistent) ---
        case 'in,regular,always'
            % "In mode, when cond, component shall always satisfy post"
            % Scope constrains when the obligation is active. Trigger fires
            % within the scope window; post persists until scope ends.
            trigExpr = regCond;
            if scopeMode ~= ""
                trigExpr = sprintf('(%s) && (%s)', scopeMode, regCond);
            end
            if isLatchingPattern(regCond, postCond)
                spec = reqCreateSpec(id, nl, 'Latching', ...
                    TriggerType='rising_edge', ...
                    TriggerExpr=trigExpr, ...
                    ResponseType='must_be_true', ...
                    ResponseExpr=expandResidualImplication(postCond), ...
                    Description=buildDescription(component, "scoped latching (in mode)"));
            else
                spec = reqCreateSpec(id, nl, 'Persistence', ...
                    TriggerType='rising_edge', ...
                    TriggerExpr=trigExpr, ...
                    ResponseType='hold_at_least', ...
                    ResponseExpr=expandResidualImplication(postCond), ...
                    ResponseDuration=T_PERSIST, ...
                    Description=buildDescription(component, "scoped persistent obligation (in mode)"));
            end

        % --- Conditional immediate (edge-triggered) ---
        case 'null,regular,immediately'
            % "When cond, component shall immediately satisfy post"
            % If postcondition contains implication, expand to disjunction
            % since it's used as a response expression (not split into guard/prop).
            respExpr = expandResidualImplication(postCond);
            spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                TriggerType='rising_edge', ...
                TriggerExpr=regCond, ...
                ResponseType='must_be_true', ...
                ResponseExpr=respExpr, ...
                Description=buildDescription(component, "conditional immediate"));

        % --- Scoped conditional immediate (in mode, edge-triggered) ---
        case 'in,regular,immediately'
            % "In mode, when cond, component shall immediately satisfy post"
            trigExpr = regCond;
            if scopeMode ~= ""
                trigExpr = sprintf('(%s) && (%s)', scopeMode, regCond);
            end
            respExpr = expandResidualImplication(postCond);
            spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                TriggerType='rising_edge', ...
                TriggerExpr=trigExpr, ...
                ResponseType='must_be_true', ...
                ResponseExpr=respExpr, ...
                Description=buildDescription(component, "scoped conditional immediate (in mode)"));

        % --- Level-triggered always (persistent obligation) ---
        case 'null,holding,always'
            % "Whenever cond, component shall always satisfy post"
            % FRET semantics: G(cond -> G(post)). Whenever cond is true,
            % post must hold at ALL future time points — persistent.
            % NOT the same as G(cond -> post) which is a simple guard.
            spec = reqCreateSpec(id, nl, 'Persistence', ...
                TriggerType='level', ...
                TriggerExpr=regCond, ...
                ResponseType='hold_at_least', ...
                ResponseExpr=expandResidualImplication(postCond), ...
                ResponseDuration=T_PERSIST, ...
                Description=buildDescription(component, "persistent obligation (level-triggered)"));

        % --- Scoped level-triggered always (in mode, persistent) ---
        case 'in,holding,always'
            % "In mode, whenever cond, component shall always satisfy post"
            trigExpr = regCond;
            if scopeMode ~= ""
                trigExpr = sprintf('(%s) && (%s)', scopeMode, regCond);
            end
            spec = reqCreateSpec(id, nl, 'Persistence', ...
                TriggerType='level', ...
                TriggerExpr=trigExpr, ...
                ResponseType='hold_at_least', ...
                ResponseExpr=expandResidualImplication(postCond), ...
                ResponseDuration=T_PERSIST, ...
                Description=buildDescription(component, "scoped persistent obligation (in mode, level)"));

        % --- Level-triggered immediate ---
        case 'null,holding,immediately'
            % "Whenever cond, component shall immediately satisfy post"
            spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                TriggerType='level', ...
                TriggerExpr=regCond, ...
                ResponseType='must_be_true', ...
                ResponseExpr=expandResidualImplication(postCond), ...
                Description=buildDescription(component, "level-triggered immediate"));

        % --- Scoped level-triggered immediate (in mode) ---
        case 'in,holding,immediately'
            % "In mode, whenever cond, component shall immediately satisfy post"
            trigExpr = regCond;
            if scopeMode ~= ""
                trigExpr = sprintf('(%s) && (%s)', scopeMode, regCond);
            end
            spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                TriggerType='level', ...
                TriggerExpr=trigExpr, ...
                ResponseType='must_be_true', ...
                ResponseExpr=expandResidualImplication(postCond), ...
                Description=buildDescription(component, "scoped level-triggered immediate (in mode)"));

        % --- Conditional until ---
        case {'null,regular,until', 'in,regular,until', ...
              'null,holding,until', 'in,holding,until'}
            % "When cond, component shall until stopCond satisfy post"
            trigType = "rising_edge";
            if condition == "holding"
                trigType = "level";
            end
            trigExpr = applyScopeToTrigger(regCond, scopeMode);
            % Handle implication in postcondition
            [guard, prop] = splitImplication(postCond);
            if guard ~= ""
                respExpr = sprintf('~(%s) || (%s)', guard, prop);
            else
                respExpr = postCond;
            end
            spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                TriggerType=trigType, ...
                TriggerExpr=trigExpr, ...
                ResponseType='weak_until', ...
                ResponseExpr=respExpr, ...
                UntilExpr=stopCond, ...
                Description=buildDescription(component, "until response"));

        % --- Conditional with bounded timing ---
        case {'null,regular,within', 'in,regular,within'}
            % "When cond, component shall within N units satisfy post"
            trigExpr = applyScopeToTrigger(regCond, scopeMode);
            spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                TriggerType='rising_edge', ...
                TriggerExpr=trigExpr, ...
                ResponseType='must_be_true', ...
                ResponseExpr=expandResidualImplication(postCond), ...
                DelayType='at_most', DelayValue=duration, ...
                Description=buildDescription(component, "bounded response"));

        % --- Conditional with delayed timing ---
        case {'null,regular,after', 'in,regular,after'}
            % "When cond, component shall after N units satisfy post"
            trigExpr = applyScopeToTrigger(regCond, scopeMode);
            spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                TriggerType='rising_edge', ...
                TriggerExpr=trigExpr, ...
                ResponseType='must_be_true', ...
                ResponseExpr=expandResidualImplication(postCond), ...
                DelayType='at_most', DelayValue=duration, ...
                Description=buildDescription(component, "delayed response"));

        % --- Conditional next-step ---
        case {'null,regular,next', 'null,holding,next', ...
              'in,regular,next', 'in,holding,next'}
            % "When/Whenever cond, component shall at the next timepoint satisfy post"
            % FRET "next" = P must hold one step AFTER edge.
            % RT cannot express this (prev(prev()) on expressions unsupported),
            % so we encode as delay=1 which routes to TA-only.
            trigType = "rising_edge";
            if condition == "holding"
                trigType = "level";
            end
            trigExpr = applyScopeToTrigger(regCond, scopeMode);
            spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                TriggerType=trigType, ...
                TriggerExpr=trigExpr, ...
                ResponseType='must_be_true', ...
                ResponseExpr=expandResidualImplication(postCond), ...
                DelayType='at_most', DelayValue=1, ...
                Description=buildDescription(component, "next-step response"));

        % --- Unconditional within bounded timing ---
        case {'null,null,within', 'in,null,within'}
            % "Component shall within N units satisfy post"
            % No trigger condition → use implication split if present
            [guard, prop] = splitImplication(postCond);
            if guard ~= ""
                trigExpr = applyScopeToTrigger(guard, scopeMode);
                spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                    TriggerType='rising_edge', ...
                    TriggerExpr=trigExpr, ...
                    ResponseType='must_be_true', ...
                    ResponseExpr=prop, ...
                    DelayType='at_most', DelayValue=duration, ...
                    Description=buildDescription(component, "unconditional bounded response"));
            else
                trigExpr = applyScopeToTrigger("true", scopeMode);
                spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                    TriggerType='level', ...
                    TriggerExpr=trigExpr, ...
                    ResponseType='must_be_true', ...
                    ResponseExpr=expandResidualImplication(postCond), ...
                    DelayType='at_most', DelayValue=duration, ...
                    Description=buildDescription(component, "unconditional bounded response"));
            end

        % --- Unconditional for duration ---
        case {'null,null,for', 'in,null,for'}
            % "Component shall for N units satisfy post"
            % No trigger condition → use implication split if present
            [guard, prop] = splitImplication(postCond);
            if guard ~= ""
                trigExpr = applyScopeToTrigger(guard, scopeMode);
                spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                    TriggerType='rising_edge', ...
                    TriggerExpr=trigExpr, ...
                    ResponseType='hold_at_least', ...
                    ResponseExpr=prop, ...
                    ResponseDuration=duration, ...
                    Description=buildDescription(component, "unconditional sustained response"));
            else
                trigExpr = applyScopeToTrigger("true", scopeMode);
                spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                    TriggerType='level', ...
                    TriggerExpr=trigExpr, ...
                    ResponseType='hold_at_least', ...
                    ResponseExpr=expandResidualImplication(postCond), ...
                    ResponseDuration=duration, ...
                    Description=buildDescription(component, "unconditional sustained response"));
            end

        % --- Unconditional eventually ---
        case 'null,null,eventually'
            % "Component shall eventually satisfy post"
            spec = reqCreateSpec(id, nl, 'TriggerResponse', ...
                TriggerType='level', ...
                TriggerExpr='true', ...
                ResponseType='must_be_true', ...
                ResponseExpr=expandResidualImplication(postCond), ...
                Description=buildDescription(component, "eventual satisfaction"));

        % --- Unconditional never ---
        case 'null,null,never'
            % "Component shall never satisfy post"
            % Equivalent to: always satisfy ~post
            spec = reqCreateSpec(id, nl, 'Invariant', ...
                Property=sprintf('~(%s)', postCond), ...
                Description=buildDescription(component, "never (negated invariant)"));

        otherwise
            warning('fretToSpec:UnsupportedTemplate', ...
                'FRET template key "%s" is not yet supported. Creating stub spec.', ...
                templateKey);
            spec = reqCreateSpec(id, nl, 'Summary', ...
                Description=sprintf('Unsupported FRET template: %s', templateKey));
    end
end

%% --- Field extraction helpers ---

function st = extractField(fretResult, fieldName)
    if strcmp(fieldName, 'scope')
        if isfield(fretResult, 'scope') && isstruct(fretResult.scope)
            st = string(fretResult.scope.type);
        else
            st = "null";
        end
    else
        if isfield(fretResult, fieldName)
            st = string(fretResult.(fieldName));
        else
            st = "null";
        end
    end
end

function pc = extractPostCondition(fretResult)
    if isfield(fretResult, 'post_condition')
        pc = string(fretResult.post_condition);
    else
        pc = "";
    end
end

function rc = extractRegularCondition(fretResult)
    if isfield(fretResult, 'regular_condition')
        rc = string(fretResult.regular_condition);
    elseif isfield(fretResult, 'pre_condition')
        rc = string(fretResult.pre_condition);
    else
        rc = "";
    end
end

function sc = extractStopCondition(fretResult)
    if isfield(fretResult, 'stop_condition')
        sc = string(fretResult.stop_condition);
    else
        sc = "";
    end
end

function dur = extractDuration(fretResult)
    dur = [];
    if isfield(fretResult, 'duration')
        if isnumeric(fretResult.duration)
            dur = fretResult.duration;
        else
            dur = str2double(string(fretResult.duration));
        end
    end
end

%% --- SMV to MATLAB syntax conversion ---

function out = smvToMatlab(expr)
    %SMVTOMATLAB Convert nuXmv/SMV syntax to MATLAB syntax.
    %   SMV uses: !, &, |, ->, =, !=
    %   MATLAB uses: ~, &&, ||, implies(), ==, ~=

    out = string(expr);
    if out == ""
        return
    end

    % Strip outer parentheses if fully wrapped
    out = stripOuterParens(out);

    % Replace SMV operators with MATLAB equivalents
    % Order matters: do multi-char operators first

    % -> (implication) is tricky — in postconditions it typically appears as
    % guard -> property. We leave it and handle in splitImplication.
    % For standalone conversion:
    %   (A -> B) becomes (~(A) || (B))

    % <=> (bi-implication) — expand before = → == conversion
    % a <=> b → (~(a) || (b)) && (~(b) || (a))
    out = expandBiImplication(out);

    % != to ~=
    out = strrep(out, '!=', '~=');

    % => (implication) — preserve as -> for splitImplication to handle
    out = strrep(out, '=>', '->');

    % SMV = (equality) to == (MATLAB equality)
    % But be careful not to double-convert <= >= -> ~=
    out = regexprep(out, '(?<![<>!~=\-])=(?!=)', '==');

    % ! to ~ (negation)
    out = strrep(out, '!', '~');

    % & to && (short-circuit AND)
    % Avoid converting && to &&&&
    out = regexprep(out, '(?<!&)&(?!&)', '&&');

    % | to || (short-circuit OR)
    out = regexprep(out, '(?<!\|)\|(?!\|)', '||');

    % Verbose logical keywords (used in FRET variable assignments)
    out = regexprep(out, '(?<!\w)not(?!\w)', '~');
    out = regexprep(out, '(?<!\w)and(?!\w)', '&&');
    out = regexprep(out, '(?<!\w)or(?!\w)', '||');

    % Expand functions not supported in RT/TA postconditions
    out = expandFunctions(out);

    % Convert SMV if/then/else to MATLAB numeric conditional
    out = expandIfThenElse(out);
end

%% --- Constant substitution (mechanical, like CoCoSim) ---

function out = substituteConstants(expr, fretVarsCell, outputNames)
    %SUBSTITUTECONSTANTS Replace Internal variable names with their constant values.
    %   This is a mechanical 1:1 substitution — no semantic interpretation.
    %   Only replaces variables with idType='Internal' that have a non-empty assignment.
    %   fretVarsCell is a cell array of structs.
    %
    %   Assignments are converted from SMV to MATLAB syntax before injection,
    %   handling bare "pre x" → "prev(x)" and "init -> expr" → "expr".
    %
    %   If an Internal's assignment references any Output variable name (from
    %   outputNames), the substitution is skipped — such Internals are mode
    %   guards that should remain as symbols to avoid IsDesignOutput conflicts.
    %
    %   Example: 'state == ap_standby_state' with ap_standby_state=3.0
    %            becomes 'state == 3.0'

    out = expr;
    if out == "" || isempty(fretVarsCell)
        return
    end

    for i = 1:numel(fretVarsCell)
        if iscell(fretVarsCell)
            v = fretVarsCell{i};
        else
            v = fretVarsCell(i);
        end
        if ~isfield(v, 'idType') || ~isfield(v, 'assignment')
            continue
        end
        if strcmpi(v.idType, 'Internal') && ~isempty(v.assignment) && strlength(string(v.assignment)) > 0
            varName = string(v.variable_name);
            asgn = string(v.assignment);
            % Skip if assignment references an Output variable — these are
            % mode guards (e.g., "normal = not RESET and yout <= TL") that
            % should stay as symbols to avoid precondition/output conflicts.
            if referencesOutput(asgn, outputNames)
                continue
            end
            constVal = convertAssignment(asgn);
            pat = sprintf('(?<!\\w)%s(?!\\w)', char(varName));
            out = regexprep(out, pat, char(constVal));
        end
    end
end

function names = collectOutputNames(fretVarsCell)
    %COLLECTOUTPUTNAMES Return cell array of Output variable names from FRET vars.
    names = {};
    for i = 1:numel(fretVarsCell)
        if iscell(fretVarsCell)
            v = fretVarsCell{i};
        else
            v = fretVarsCell(i);
        end
        if isfield(v, 'idType') && strcmpi(v.idType, 'Output')
            names{end+1} = char(string(v.variable_name)); %#ok<AGROW>
        end
    end
    names = unique(names);
end

function tf = referencesOutput(assignment, outputNames)
    %REFERENCESOUTPUT True if the assignment string references any Output variable.
    tf = false;
    for i = 1:numel(outputNames)
        pat = sprintf('(?<!\\w)%s(?!\\w)', outputNames{i});
        if ~isempty(regexp(char(assignment), pat, 'once'))
            tf = true;
            return
        end
    end
end

function out = convertAssignment(asgn)
    %CONVERTASSIGNMENT Convert a FRET Internal variable assignment to MATLAB syntax.
    %   Handles SMV operators, bare "pre", and the arrow (initial-value) operator.

    out = smvToMatlab(asgn);
    out = convertBarePre(out);
    out = stripArrowInit(out);
end

function out = convertBarePre(expr)
    %CONVERTBAREPRE Convert bare "pre varname" or "pre (expr)" to "prev(varname)".
    %   Handles both "pre identifier" and "pre (parenthesized expr)".
    out = expr;
    % "pre (expr)" — parenthesized argument
    out = regexprep(out, '(?<!\w)pre\s*(\([^)]*\))', 'prev$1');
    % "pre identifier" — single variable name
    out = regexprep(out, '(?<!\w)pre\s+(\w+)', 'prev($1)');
end

function out = stripArrowInit(expr)
    %STRIPARROWINIT Remove the SMV arrow initial-value prefix "init -> expr".
    %   The arrow "A -> B" in SMV means "A at t=0, B thereafter".
    %   RT/TA handle initialization via symbol InitialValue properties,
    %   so we keep only the steady-state expression (B).
    %   Finds the top-level -> and returns the RHS.
    out = expr;
    s = char(out);
    depth = 0;
    for idx = 1:length(s)-1
        switch s(idx)
            case '(', depth = depth + 1;
            case ')', depth = depth - 1;
        end
        if depth == 0 && idx+1 <= length(s) && s(idx) == '-' && s(idx+1) == '>'
            out = strtrim(string(s(idx+2:end)));
            return
        end
    end
end

%% --- Function expansion ---

function out = expandFunctions(expr)
    %EXPANDFUNCTIONS Expand functions not natively supported in RT/TA.
    %   absReal(x) → abs(x)
    %   preBool(init, expr) → prev(expr)  (RT/TA prev() operator)
    %   preReal(init, expr) → prev(expr)
    %   FTP → (t == 0)  (first time point sentinel)
    %   a xor b → xor(a, b)
    %   median(a, b, c) → (a + b + c - max(a, max(b, c)) - min(a, min(b, c)))

    out = expr;

    % absReal(x) → abs(x)
    out = strrep(out, 'absReal', 'abs');

    % preBool(init, expr) → prev(expr)  — drop the initial value arg
    % Pattern: preBool( init , expr )
    out = expandPrevFunction(out, 'preBool');
    out = expandPrevFunction(out, 'preReal');
    out = expandPrevFunction(out, 'preInt');

    % FTP (First Time Point) — exempts t=0 from the obligation.
    % Pattern is always "FTP | expr_with_prev" meaning "at t=0 trivially
    % true, after that expr must hold."
    % Replace FTP with a prev-based first-step detector: a local flag
    % __past_first_step that is 0 at t=0 (via InitialValue) and 1 thereafter.
    % FTP → (__past_first_step == 0) so at t=0 the disjunction is true.
    % The flag must be added as a symbol with InitialValue=0 by the caller.
    out = regexprep(out, '(?<!\w)FTP(?!\w)', '(__past_first_step == 0)');

    % Infix xor: "a xor b" → "xor(a, b)"
    % Matches single identifiers or parenthesized groups on each side
    out = regexprep(out, '(\w+|\([^)]+\))\s+xor\s+(\w+|\([^)]+\))', 'xor($1, $2)');

    % Expand median(a, b, c) for 3 arguments
    pat = 'median\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^)]+)\s*\)';
    tokens = regexp(char(out), pat, 'tokens');
    while ~isempty(tokens)
        a = strtrim(tokens{1}{1});
        b = strtrim(tokens{1}{2});
        c = strtrim(tokens{1}{3});
        expansion = sprintf('(%s + %s + %s - max(%s, max(%s, %s)) - min(%s, min(%s, %s)))', ...
            a, b, c, a, b, c, a, b, c);
        out = regexprep(out, pat, expansion, 'once');
        tokens = regexp(char(out), pat, 'tokens');
    end
end

function out = expandIfThenElse(expr)
    %EXPANDIFTHENELSE Convert SMV "if C then A else B" to MATLAB conditional.
    %   Produces "(C) * (A) + (~(C)) * (B)" which is valid in Stateflow-based
    %   RT/TA blocks (scalar operations). Handles nested if/then/else recursively.
    out = expr;
    s = char(out);

    % Find "if" keyword at word boundary
    ifStarts = regexp(s, '(?<!\w)if(?!\w)', 'start');
    if isempty(ifStarts)
        return
    end

    % Process innermost "if" first (last occurrence) to handle nesting
    for k = length(ifStarts):-1:1
        pos = ifStarts(k);
        rest = s(pos:end);

        % Find matching "then" and "else" at the same nesting depth
        depth = 0;
        thenPos = [];
        elsePos = [];
        i = 3; % skip "if"
        while i <= length(rest)
            ch = rest(i);
            if ch == '(', depth = depth + 1;
            elseif ch == ')', depth = depth - 1;
            end
            if depth == 0
                if isempty(thenPos) && i+3 <= length(rest) && ...
                        strcmp(rest(i:i+3), 'then') && ...
                        (i == 1 || ~isletter(rest(i-1))) && ...
                        (i+4 > length(rest) || ~isletter(rest(i+4)))
                    thenPos = i;
                    i = i + 4;
                    continue
                end
                if ~isempty(thenPos) && isempty(elsePos) && i+3 <= length(rest) && ...
                        strcmp(rest(i:i+3), 'else') && ...
                        (i == 1 || ~isletter(rest(i-1))) && ...
                        (i+4 > length(rest) || ~isletter(rest(i+4)))
                    elsePos = i;
                    break
                end
            end
            i = i + 1;
        end

        if isempty(thenPos) || isempty(elsePos)
            continue
        end

        % Extract condition, then-branch, else-branch
        condStr = strtrim(rest(3:thenPos-1));
        thenStr = strtrim(rest(thenPos+4:elsePos-1));

        % For else-branch, find extent: either to end of rest or balanced parens
        elseTail = rest(elsePos+4:end);
        % Find extent of else branch — goes until unbalanced paren or end
        depth2 = 0;
        endPos = length(elseTail);
        for j = 1:length(elseTail)
            if elseTail(j) == '(', depth2 = depth2 + 1;
            elseif elseTail(j) == ')'
                if depth2 == 0
                    endPos = j - 1;
                    break
                end
                depth2 = depth2 - 1;
            end
        end
        elseStr = strtrim(elseTail(1:endPos));
        trailingStr = elseTail(endPos+1:end);

        % Recursively expand nested if/then/else in branches
        thenStr = char(expandIfThenElse(string(thenStr)));
        elseStr = char(expandIfThenElse(string(elseStr)));

        replacement = sprintf('((%s) * (%s) + (~(%s)) * (%s))', ...
            condStr, thenStr, condStr, elseStr);

        % Reconstruct
        before = s(1:pos-1);
        s = [before replacement trailingStr];
        break % restart after one substitution
    end

    out = string(s);
    % Recurse in case there are more if/then/else at the same level
    if contains(out, ' then ')
        out = expandIfThenElse(out);
    end
end

function out = expandBiImplication(expr)
    %EXPANDBIIMPLICATION Expand a <=> b to (~(a) || (b)) && (~(b) || (a)).
    %   Finds top-level <=> respecting parenthesis nesting.
    %   Recursively expands sub-expressions that may contain nested <=>.
    out = expr;
    s = char(out);
    depth = 0;
    for i = 1:length(s)-2
        switch s(i)
            case '(', depth = depth + 1;
            case ')', depth = depth - 1;
        end
        if depth == 0 && i+2 <= length(s) && s(i) == '<' && s(i+1) == '=' && s(i+2) == '>'
            lhs = strtrim(string(s(1:i-1)));
            rhs = strtrim(string(s(i+3:end)));
            lhs = stripOuterParens(lhs);
            rhs = stripOuterParens(rhs);
            % Recursively expand any nested <=> in both sides
            lhs = expandBiImplication(lhs);
            rhs = expandBiImplication(rhs);
            out = sprintf('(~(%s) || (%s)) && (~(%s) || (%s))', lhs, rhs, rhs, lhs);
            return
        end
    end
end

function out = expandPrevFunction(expr, funcName)
    %EXPANDPREVFUNCTION Convert preBool(init, expr) or preReal(init, expr) to prev(expr).
    %   Drops the initial value argument since RT's prev() handles initialization
    %   via symbol properties.
    out = expr;
    pat = [funcName '\s*\('];
    startIdx = regexp(char(out), pat, 'start');
    while ~isempty(startIdx)
        s = char(out);
        % Find the opening paren
        openParen = startIdx(1) + length(funcName);
        while openParen <= length(s) && s(openParen) ~= '('
            openParen = openParen + 1;
        end
        % Find the matching close paren
        depth = 1;
        pos = openParen + 1;
        commaPos = [];
        while pos <= length(s) && depth > 0
            if s(pos) == '('
                depth = depth + 1;
            elseif s(pos) == ')'
                depth = depth - 1;
            elseif s(pos) == ',' && depth == 1 && isempty(commaPos)
                commaPos = pos;
            end
            pos = pos + 1;
        end
        closeParen = pos - 1;
        if ~isempty(commaPos) && closeParen > openParen
            % Extract the second argument (the expression)
            secondArg = strtrim(s(commaPos+1:closeParen-1));
            % Replace: funcName(init, expr) → prev(expr)
            replacement = sprintf('prev(%s)', secondArg);
            out = string([s(1:startIdx(1)-1) replacement s(closeParen+1:end)]);
        else
            break
        end
        startIdx = regexp(char(out), pat, 'start');
    end
end

function out = stripOuterParens(expr)
    out = strtrim(expr);
    while startsWith(out, '(') && endsWith(out, ')')
        inner = extractBetween(out, 2, strlength(out)-1);
        if parenDepthValid(inner)
            out = strtrim(inner);
        else
            break
        end
    end
end

function valid = parenDepthValid(s)
    %PARENDEPTHVALID Check that parentheses never go negative (i.e., outer parens were matched).
    depth = 0;
    for i = 1:strlength(s)
        c = extractBetween(s, i, i);
        if c == "("
            depth = depth + 1;
        elseif c == ")"
            depth = depth - 1;
        end
        if depth < 0
            valid = false;
            return
        end
    end
    valid = (depth == 0);
end

%% --- Implication splitting ---

function [guard, prop] = splitImplication(expr)
    %SPLITIMPLICATION Split "guard -> property" or "guard => property".
    %   Returns guard="" if no implication found.
    %   If the property side still contains a top-level implication
    %   (chained case: A -> B -> C), expand it to disjunction form.

    guard = "";
    prop = expr;

    % Look for top-level -> or =>
    % Must respect parenthesis nesting
    s = char(expr);
    depth = 0;
    for i = 1:length(s)
        switch s(i)
            case '('
                depth = depth + 1;
            case ')'
                depth = depth - 1;
        end
        if depth == 0
            % Check for -> at this position
            if i < length(s) && s(i) == '-' && s(i+1) == '>'
                guard = strtrim(string(s(1:i-1)));
                prop  = strtrim(string(s(i+2:end)));
                % Strip outer parens from both sides
                guard = stripOuterParens(guard);
                prop  = stripOuterParens(prop);
                % If prop still contains a top-level ->, expand it
                prop = expandResidualImplication(prop);
                return
            end
            % Check for =>
            if i < length(s) && s(i) == '=' && s(i+1) == '>'
                guard = strtrim(string(s(1:i-1)));
                prop  = strtrim(string(s(i+2:end)));
                guard = stripOuterParens(guard);
                prop  = stripOuterParens(prop);
                prop = expandResidualImplication(prop);
                return
            end
        end
    end
end

function out = expandResidualImplication(expr)
    %EXPANDRESIDUALIMPLICATION If expr contains a top-level ->, expand to disjunction.
    %   "A -> B" becomes "~(A) || (B)" so it's valid MATLAB syntax.
    %   Applied recursively until no top-level implications remain.
    out = expr;
    s = char(out);
    depth = 0;
    for i = 1:length(s)
        switch s(i)
            case '(', depth = depth + 1;
            case ')', depth = depth - 1;
        end
        if depth == 0
            if i < length(s) && s(i) == '-' && s(i+1) == '>'
                lhs = strtrim(string(s(1:i-1)));
                rhs = strtrim(string(s(i+2:end)));
                lhs = stripOuterParens(lhs);
                rhs = stripOuterParens(rhs);
                rhs = expandResidualImplication(rhs);
                out = sprintf('~(%s) || (%s)', lhs, rhs);
                return
            end
            if i < length(s) && s(i) == '=' && s(i+1) == '>'
                lhs = strtrim(string(s(1:i-1)));
                rhs = strtrim(string(s(i+2:end)));
                lhs = stripOuterParens(lhs);
                rhs = stripOuterParens(rhs);
                rhs = expandResidualImplication(rhs);
                out = sprintf('~(%s) || (%s)', lhs, rhs);
                return
            end
        end
    end
end

%% --- Latching detection ---

function tf = isLatchingPattern(condition, postcondition)
    %ISLATCHINGPATTERN Detect if condition and postcondition are the same expression.
    %   "when X, always satisfy X" is a latch pattern.
    tf = (strtrim(condition) == strtrim(postcondition));
end

%% --- Scope application ---

function trigExpr = applyScopeToTrigger(regCond, scopeMode)
    %APPLYSCOPETOTRIGGER Conjoin scope mode into trigger expression.
    %   For 'in' scoped requirements, the trigger only fires while the
    %   scope mode is active.
    if scopeMode ~= "" && regCond ~= ""
        trigExpr = sprintf('(%s) && (%s)', scopeMode, regCond);
    elseif scopeMode ~= ""
        trigExpr = scopeMode;
    else
        trigExpr = regCond;
    end
end

%% --- Description builder ---

function desc = buildDescription(component, patternName)
    if component ~= ""
        desc = sprintf("[%s] %s", component, patternName);
    else
        desc = string(patternName);
    end
end
