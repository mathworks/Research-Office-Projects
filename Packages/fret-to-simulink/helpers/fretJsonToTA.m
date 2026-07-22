function taBlock = fretJsonToTA(jsonFile, modelName, opts)
%FRETJSONTOTA Mechanical pipeline: FRET JSON export → Test Assessment block.
%
%   taBlock = fretJsonToTA(JSONFILE) loads a FRET JSON export file containing
%   requirements and variable mappings, and creates a new Simulink model
%   with a Test Assessment block containing verify statements. Internal
%   (constant) variables are mechanically substituted with their assigned
%   values — no semantic interpretation, matching CoCoSim's approach.
%
%   When the JSON contains multiple FRET components, a separate model with
%   its own TA block is created per component to avoid cross-component
%   symbol conflicts. Set PerComponent=false to merge all requirements into
%   a single TA block.
%
%   taBlock = fretJsonToTA(JSONFILE, MODELNAME) creates the TA block in the
%   specified model. If MODELNAME is an existing model, the TA block is
%   added to it. If not, a new model is created.
%
%   Name-Value Options:
%     ReqFilter    - cell array of reqids to include (default: all)
%     Tolerance    - numeric tolerance for double equality (default: 0)
%     PerComponent - create one TA block per FRET component (default: true)
%
%   Example:
%     taBlock = fretJsonToTA('fsm_reqts_and_vars.json');
%     taBlock = fretJsonToTA('fsm_reqts_and_vars.json', 'FSM_TA');
%     taBlks = fretJsonToTA('LM_requirements.json', 'LMCPS_TA');
%     taBlock = fretJsonToTA('lm_reqs.json', 'MyModel', PerComponent=false);
%
%   See also fretToSpec, reqRenderToTA, fretJsonToRT

    arguments
        jsonFile  (1,1) string {mustBeFile}
        modelName (1,1) string = ""
        opts.ReqFilter cell = {}
        opts.Tolerance (1,1) double = 0
        opts.PerComponent (1,1) logical = true
    end

    %% 1. Load and normalize FRET JSON
    fretData = jsondecode(fileread(jsonFile));
    reqs = normalizeCellArray(fretData.requirements);
    vars = normalizeFretVars(fretData.variables);

    fprintf('Loaded %d requirements, %d variables from %s\n', ...
        numel(reqs), numel(vars), jsonFile);

    %% 2. Convert each requirement through fretToSpec → reqRenderToTA
    %  Spec conversion is pure computation (no model I/O) and can be
    %  parallelized with parfor when Parallel Computing Toolbox is available.
    tConvert = tic;

    % Apply filter up front to reduce work
    if ~isempty(opts.ReqFilter)
        keepIdx = false(numel(reqs), 1);
        for i = 1:numel(reqs)
            keepIdx(i) = ismember(char(string(reqs{i}.reqid)), opts.ReqFilter);
        end
        reqs = reqs(keepIdx);
    end

    nReqs = numel(reqs);
    taCell   = cell(nReqs, 1);
    skipCell = cell(nReqs, 1);
    compCell = cell(nReqs, 1);
    tolVal   = opts.Tolerance;

    % Use parfor if pool is available, otherwise falls back to for
    parfor i = 1:nReqs
        r = reqs{i};
        reqid = string(r.reqid);

        % Extract component name
        if isfield(r.semantics, 'component_name')
            compCell{i} = string(r.semantics.component_name);
        elseif isfield(r.semantics, 'component')
            compCell{i} = string(r.semantics.component);
        else
            compCell{i} = "Default";
        end

        % Convert to spec
        try
            spec = fretToSpec(reqid, string(r.fulltext), r.semantics, vars);
        catch ME
            skipCell{i} = struct('reqid', reqid, 'reason', string(ME.message));
            continue
        end

        % Check TA renderability
        if spec.pattern == "Summary"
            skipCell{i} = struct('reqid', reqid, 'reason', "Unsupported template");
            continue
        end

        ta = reqRenderToTA(spec);
        if ~ta.renderable
            skipCell{i} = struct('reqid', reqid, 'reason', string(ta.reason));
            continue
        end

        % Skip temporal operators and unsupported constructs (same as RT)
        combined = extractAllExprs(ta);
        if ~isempty(regexp(char(combined), '(?<!\w)persisted\s*\(', 'once'))
            skipCell{i} = struct('reqid', reqid, ...
                'reason', "Contains persisted() temporal operator — not TA-expressible.");
            continue
        end
        if ~isempty(regexp(char(combined), '(mag|dot|det_3x3)\s*\(', 'once'))
            skipCell{i} = struct('reqid', reqid, ...
                'reason', "Uses external function call — not TA-expressible without extrinsic definitions.");
            continue
        end
        prevArgs = regexp(char(combined), 'prev\(([^)]+)\)', 'tokens');
        hasComplexPrev = false;
        for pa = 1:numel(prevArgs)
            if isempty(regexp(strtrim(prevArgs{pa}{1}), '^[a-zA-Z_]\w*$', 'once'))
                hasComplexPrev = true;
                break
            end
        end
        if hasComplexPrev
            skipCell{i} = struct('reqid', reqid, ...
                'reason', "prev() with complex expression — TA only supports prev(single_symbol).");
            continue
        end

        % Apply tolerance to equality expressions
        if tolVal > 0
            ta = applyToleranceToTA(ta, tolVal);
        end

        % Stamp reqid directly on the TA result for downstream step naming
        ta.reqid = reqid;

        taCell{i} = ta;
    end

    % Collect results
    validIdx = ~cellfun(@isempty, taCell);
    taResults = taCell(validIdx);
    compResults = compCell(validIdx);
    skipped = skipCell(~cellfun(@isempty, skipCell));

    fprintf('Converted: %d TA-renderable, %d skipped (%.1f s)\n', ...
        numel(taResults), numel(skipped), toc(tConvert));
    for i = 1:numel(skipped)
        fprintf('  Skipped %s: %s\n', skipped{i}.reqid, skipped{i}.reason);
    end

    if isempty(taResults)
        error('fretJsonToTA:NoRenderableReqs', 'No requirements could be rendered to TA.');
    end

    %% 3. Derive base model name
    if modelName == ""
        [~, baseName] = fileparts(jsonFile);
        modelName = matlab.lang.makeValidName(baseName + "_TA");
    end

    %% 4. Group by component and create models
    uniqueComps = unique(string(compResults));
    if ~opts.PerComponent || isscalar(uniqueComps)
        % Single model with one TA block
        mdlName = char(modelName);
        closeAndDelete(mdlName);
        new_system(mdlName);
        open_system(mdlName);
        cleanupObj = onCleanup(@() closeIfOpen(string(mdlName)));
        configureTA(mdlName);
        taBlockPath = buildTABlock(mdlName, taResults, vars);
        save_system(mdlName);
        clear cleanupObj
        fprintf('Saved model: %s (1 TA block, %d assessments)\n', mdlName, numel(taResults));
        taBlock = taBlockPath;
    else
        % One model per component
        taBlock = cell(numel(uniqueComps), 1);
        for c = 1:numel(uniqueComps)
            compName = uniqueComps(c);
            idx = strcmp(string(compResults), compName);
            groupResults = taResults(idx);

            mdlName = char(modelName + "_" + matlab.lang.makeValidName(compName));
            closeAndDelete(mdlName);
            new_system(mdlName);
            open_system(mdlName);
            configureTA(mdlName);
            taBlockPath = buildTABlock(mdlName, groupResults, vars);
            save_system(mdlName);
            close_system(mdlName, 0);
            taBlock{c} = taBlockPath;
            fprintf('  %s: %d assessments → %s.slx\n', compName, numel(groupResults), mdlName);
        end
        fprintf('Created %d TA models from %s\n', numel(uniqueComps), modelName);
    end
end

%% --- Core TA Block Construction ---

function taBlockPath = buildTABlock(mdlName, taResults, vars)
    %BUILDTABLOCK Create and populate a Test Assessment block in the model.

    % Add Test Assessment block
    taBlockPath = [char(mdlName) '/FRET_Assessments'];
    if getSimulinkBlockHandle(taBlockPath) ~= -1
        delete_block(taBlockPath);
    end
    add_block('sltestlib/Test Assessment', taBlockPath);

    % Remove default When decomposition steps
    defaultSteps = sltest.testsequence.findStep(taBlockPath);
    childSteps = defaultSteps(contains(defaultSteps, '.'));
    for i = numel(childSteps):-1:1
        sltest.testsequence.deleteStep(taBlockPath, childSteps{i});
    end
    topSteps = defaultSteps(~contains(defaultSteps, '.'));
    if ~isempty(topSteps)
        sltest.testsequence.editStep(taBlockPath, topSteps{1}, ...
            'Name', 'Assess', 'IsWhenStep', false);
    end

    % Collect all signal names from TA expressions for input symbols
    allExprText = "";
    for i = 1:numel(taResults)
        ta = taResults{i};
        allExprText = allExprText + " " + extractAllExprs(ta);
    end
    tokens = regexp(char(allExprText), '(?<!\w)([a-zA-Z_]\w*)(?!\w)', 'tokens');
    allIdentifiers = unique(cellfun(@(t) t{1}, tokens, 'UniformOutput', false));

    % Add input symbols for non-Internal FRET variables that appear in expressions
    addedSymbols = {};
    existingSyms = sltest.testsequence.findSymbol(taBlockPath);
    for j = 1:numel(vars)
        v = vars{j};
        vn = string(v.variable_name);
        if strcmpi(v.idType, 'Internal') && ~isempty(v.assignment) && strlength(string(v.assignment)) > 0
            continue
        end
        if ismember(char(vn), allIdentifiers) && ~ismember(char(vn), addedSymbols) && ...
                ~ismember(char(vn), existingSyms)
            sltest.testsequence.addSymbol(taBlockPath, char(vn), 'Data', 'Input');
            addedSymbols{end+1} = char(vn); %#ok<AGROW>
        end
    end

    % Add remaining identifiers not in the FRET variable list (catch-all).
    builtins = {'prev','abs','cos','sin','max','min','xor','sign','sqrt',...
        'true','false','nan','inf','pi','end','if','else','elseif',...
        'persisted','__past_first_step','mag','dot','det_3x3','duration','after','verify',...
        'hasChangedTo'};
    for j = 1:numel(allIdentifiers)
        id = allIdentifiers{j};
        if any(strcmp(id, addedSymbols)) || any(strcmp(id, builtins)) || ...
                any(strcmp(id, existingSyms))
            continue
        end
        sltest.testsequence.addSymbol(taBlockPath, id, 'Data', 'Input');
        addedSymbols{end+1} = id; %#ok<AGROW>
    end
    % Infer vector dimensions from indexing expressions like var(3).
    % TA requires explicit Size for vector signals via editSymbol.
    vecDims = containers.Map('KeyType','char','ValueType','double');
    idxTokens = regexp(char(allExprText), '([a-zA-Z_]\w*)\s*\(\s*(\d+)\s*\)', 'tokens');
    for k = 1:numel(idxTokens)
        vName = idxTokens{k}{1};
        idx = str2double(idxTokens{k}{2});
        if any(strcmp(vName, addedSymbols)) && ~any(strcmp(vName, builtins))
            if ~vecDims.isKey(vName) || vecDims(vName) < idx
                vecDims(vName) = idx;
            end
        end
    end
    % Propagate dimensions through multiplication
    if ~isempty(vecDims.keys)
        mulTokens = regexp(char(allExprText), '([a-zA-Z_]\w*)\s*\*\s*([a-zA-Z_]\w*)', 'tokens');
        changed = true;
        while changed
            changed = false;
            for k = 1:numel(mulTokens)
                lhs = mulTokens{k}{1}; rhs = mulTokens{k}{2};
                if vecDims.isKey(lhs) && ~vecDims.isKey(rhs) ...
                        && any(strcmp(rhs, addedSymbols)) && ~any(strcmp(rhs, builtins))
                    vecDims(rhs) = vecDims(lhs);
                    changed = true;
                elseif vecDims.isKey(rhs) && ~vecDims.isKey(lhs) ...
                        && any(strcmp(lhs, addedSymbols)) && ~any(strcmp(lhs, builtins))
                    vecDims(lhs) = vecDims(rhs);
                    changed = true;
                end
            end
        end
    end
    vecSymbols = vecDims.keys;
    for k = 1:numel(vecSymbols)
        sltest.testsequence.editSymbol(taBlockPath, vecSymbols{k}, ...
            'Size', char(string(vecDims(vecSymbols{k}))));
    end
    % Set explicit Size=1 for all scalar input symbols. TA inports with
    % Size=-1 (inherited) fail compilation when nothing is connected.
    for k = 1:numel(addedSymbols)
        if ~vecDims.isKey(addedSymbols{k})
            sltest.testsequence.editSymbol(taBlockPath, addedSymbols{k}, 'Size', '1');
        end
    end
    if ~isempty(vecSymbols)
        fprintf('    Set vector dimensions: %s\n', strjoin(vecSymbols, ', '));
    end

    % Rewrite vector-vector multiplications as dot products (A' * B)
    if ~isempty(vecSymbols)
        for i = 1:numel(taResults)
            ta = taResults{i};
            if isfield(ta, 'expression')
                ta.expression = rewriteVectorMul(ta.expression, vecSymbols);
            end
            if isfield(ta, 'guard') && strlength(string(ta.guard)) > 0
                ta.guard = rewriteVectorMul(ta.guard, vecSymbols);
            end
            if isfield(ta, 'trigger') && isfield(ta.trigger, 'expr')
                ta.trigger.expr = rewriteVectorMul(ta.trigger.expr, vecSymbols);
            end
            if isfield(ta, 'response') && isfield(ta.response, 'expr')
                ta.response.expr = rewriteVectorMul(ta.response.expr, vecSymbols);
            end
            if isfield(ta, 'response') && isfield(ta.response, 'untilExpr')
                ta.response.untilExpr = rewriteVectorMul(ta.response.untilExpr, vecSymbols);
            end
            taResults{i} = ta;
        end
    end

    % Replace prev(symbol) with Local variables. TA Input symbols don't
    % support prev() — we create prev_<name> Locals and update them each step.
    prevSyms = regexp(char(allExprText), 'prev\(([a-zA-Z_]\w*)\)', 'tokens');
    prevSymNames = unique(cellfun(@(t) t{1}, prevSyms, 'UniformOutput', false));
    prevUpdate = "";
    for k = 1:numel(prevSymNames)
        symName = prevSymNames{k};
        localName = ['prev_' symName];
        sizeVal = '1';
        if vecDims.isKey(symName)
            sizeVal = char(string(vecDims(symName)));
        end
        sltest.testsequence.addSymbol(taBlockPath, localName, 'Data', 'Local');
        sltest.testsequence.editSymbol(taBlockPath, localName, ...
            'DataType', 'double', 'Size', sizeVal, 'InitialValue', '0');
        prevUpdate = prevUpdate + sprintf('%s = %s;\n', localName, symName);
    end
    % Rewrite prev(<name>) → prev_<name> in all TA results
    if ~isempty(prevSymNames)
        for i = 1:numel(taResults)
            ta = taResults{i};
            if isfield(ta, 'expression')
                ta.expression = rewritePrev(ta.expression, prevSymNames);
            end
            if isfield(ta, 'guard') && strlength(string(ta.guard)) > 0
                ta.guard = rewritePrev(ta.guard, prevSymNames);
            end
            if isfield(ta, 'trigger') && isfield(ta.trigger, 'expr')
                ta.trigger.expr = rewritePrev(ta.trigger.expr, prevSymNames);
            end
            if isfield(ta, 'response') && isfield(ta.response, 'expr')
                ta.response.expr = rewritePrev(ta.response.expr, prevSymNames);
            end
            if isfield(ta, 'response') && isfield(ta.response, 'untilExpr')
                ta.response.untilExpr = rewritePrev(ta.response.untilExpr, prevSymNames);
            end
            taResults{i} = ta;
        end
    end

    % Handle __past_first_step if used (FTP construct from FRET).
    % Needs a Local boolean: 0 at t=0, set to 1 at end of each step.
    if contains(allExprText, '__past_first_step')
        sltest.testsequence.addSymbol(taBlockPath, '__past_first_step', 'Data', 'Local');
        sltest.testsequence.editSymbol(taBlockPath, '__past_first_step', ...
            'DataType', 'boolean', 'Size', '1', 'InitialValue', 'false');
        prevUpdate = prevUpdate + sprintf('__past_first_step = true;\n');
    end

    fprintf('    %d input symbols, %d assessments\n', numel(addedSymbols), numel(taResults));

    % Separate custom (invariant) from trigger-response assessments
    customTAs = {};
    trigRespTAs = {};
    for i = 1:numel(taResults)
        if taResults{i}.type == "custom"
            customTAs{end+1} = taResults{i}; %#ok<AGROW>
        else
            trigRespTAs{end+1} = taResults{i}; %#ok<AGROW>
        end
    end

    % Determine which trigger-response assessments need manual edge detection
    edgeLocals = {};
    for i = 1:numel(trigRespTAs)
        ta = trigRespTAs{i};
        if needsManualEdge(ta)
            curName  = sprintf('cur_trig_%d', i);
            prevName = sprintf('prev_trig_%d', i);
            trigExpr = convertToTAOperators(ta.trigger.expr);
            edgeLocals{end+1} = struct('cur', curName, 'prev', prevName, ...
                'expr', char(trigExpr)); %#ok<AGROW>

            sltest.testsequence.addSymbol(taBlockPath, curName, 'Data', 'Local');
            sltest.testsequence.editSymbol(taBlockPath, curName, ...
                'DataType', 'boolean', 'InitialValue', 'false');
            sltest.testsequence.addSymbol(taBlockPath, prevName, 'Data', 'Local');
            sltest.testsequence.editSymbol(taBlockPath, prevName, ...
                'DataType', 'boolean', 'InitialValue', 'false');
        else
            edgeLocals{end+1} = []; %#ok<AGROW>
        end
    end

    % Build the edge-tracking and prev-update code that must appear in EVERY step
    edgeCompute = "";
    edgeUpdate  = "";
    for i = 1:numel(edgeLocals)
        if ~isempty(edgeLocals{i})
            el = edgeLocals{i};
            edgeCompute = edgeCompute + sprintf('%s = %s;\n', el.cur, el.expr);
            edgeUpdate  = edgeUpdate  + sprintf('%s = %s;\n', el.prev, el.cur);
        end
    end
    edgeUpdate = edgeUpdate + prevUpdate;

    % Build the Assess step action with all custom (invariant) assessments
    assessAction = edgeCompute;
    for i = 1:numel(customTAs)
        ta = customTAs{i};
        label = makeVerifyLabel(ta.summary);
        expr = ensureLogicalExpr(convertToTAOperators(ta.expression));
        assessAction = assessAction + sprintf('%% %s\n', ta.summary);
        if isfield(ta, 'guard') && strlength(ta.guard) > 0
            guard = convertToTAOperators(ta.guard);
            assessAction = assessAction + sprintf('if %s\n', guard);
            assessAction = assessAction + sprintf('  verify(%s, ''%s'');\n', expr, label);
            assessAction = assessAction + sprintf('end\n\n');
        else
            assessAction = assessAction + sprintf('verify(%s, ''%s'');\n\n', expr, label);
        end
    end
    assessAction = assessAction + edgeUpdate;

    if strlength(assessAction) > 0
        sltest.testsequence.editStep(taBlockPath, 'Assess', ...
            'Action', char(assessAction));
    end

    % Add trigger-response assessments as sibling step pairs
    lastStepName = 'Assess';
    for i = 1:numel(trigRespTAs)
        ta = trigRespTAs{i};
        label = makeVerifyLabel(ta.summary);

        stepId = matlab.lang.makeValidName(ta.reqid);
        waitName = sprintf('Wait_%s', stepId);
        respName = sprintf('Respond_%s', stepId);

        waitAction = char(edgeCompute + ...
            sprintf('%% Waiting for trigger: %s\n', ta.summary) + edgeUpdate);
        respExpr = ensureLogicalExpr(convertToTAOperators(ta.response.expr));
        respAction = char(edgeCompute + ...
            sprintf('verify(%s, ''%s'');\n', respExpr, label) + edgeUpdate);

        if i == 1 && isempty(customTAs)
            sltest.testsequence.editStep(taBlockPath, 'Assess', ...
                'Name', waitName, 'Action', waitAction);
            lastStepName = waitName; %#ok<NASGU>
        else
            sltest.testsequence.addStepAfter(taBlockPath, waitName, lastStepName);
            sltest.testsequence.editStep(taBlockPath, waitName, ...
                'Action', waitAction);
        end

        sltest.testsequence.addStepAfter(taBlockPath, respName, waitName);
        sltest.testsequence.editStep(taBlockPath, respName, ...
            'Action', respAction);

        lastStepName = respName;

        trigExpr = buildTriggerTransition(ta, edgeLocals{i});
        sltest.testsequence.addTransition(taBlockPath, waitName, trigExpr, respName);

        returnCond = buildReturnTransition(ta);
        sltest.testsequence.addTransition(taBlockPath, respName, returnCond, waitName);
    end
end

%% --- Helpers ---

function closeIfOpen(modelName)
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
end

function closeAndDelete(mdlName)
    if bdIsLoaded(mdlName)
        close_system(mdlName, 0);
    end
    if exist(mdlName, 'file') == 4
        delete(which(mdlName));
    end
end

function configureTA(mdlName)
    set_param(mdlName, 'SolverType', 'Fixed-step');
    set_param(mdlName, 'Solver', 'FixedStepDiscrete');
    set_param(mdlName, 'FixedStep', '1');
end

function out = normalizeCellArray(arr)
    if iscell(arr)
        out = arr;
    else
        out = num2cell(arr);
    end
end

function vars = normalizeFretVars(rawVars)
    %NORMALIZEFRETVARS Normalize FRET variables to cell array of structs
    %   with canonical fields: variable_name, idType, assignment, dataType.
    %   Deduplicates by variable_name — FRET JSON may contain the same
    %   variable declared across multiple components. First occurrence with
    %   a non-default idType wins.

    if ~iscell(rawVars)
        rawVars = num2cell(rawVars);
    end

    vars = {};
    seenNames = {};
    for i = 1:numel(rawVars)
        rv = rawVars{i};
        v.variable_name = getFieldOr(rv, 'variable_name', '');
        if isempty(v.variable_name), continue; end
        % Skip duplicates
        if any(strcmp(v.variable_name, seenNames))
            continue
        end
        v.idType     = getFieldOr(rv, 'idType', '');
        if isempty(v.idType)
            pt = getFieldOr(rv, 'portType', 'Input');
            if strcmpi(pt, 'Outport'), v.idType = 'Output';
            elseif strcmpi(pt, 'Inport'), v.idType = 'Input';
            else, v.idType = 'Input';
            end
        end
        v.assignment = getFieldOr(rv, 'assignment', '');
        v.dataType   = getFieldOr(rv, 'dataType', 'double');
        vars{end+1} = v; %#ok<AGROW>
        seenNames{end+1} = v.variable_name; %#ok<AGROW>
    end
end

function val = getFieldOr(s, field, default)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = default;
    end
end

function allText = extractAllExprs(ta)
    %EXTRACTALLEXPRS Collect all expression text from a TA result.
    allText = "";
    if isfield(ta, 'guard') && strlength(string(ta.guard)) > 0
        allText = allText + " " + string(ta.guard);
    end
    if isfield(ta, 'expression')
        allText = allText + " " + string(ta.expression);
    end
    if isfield(ta, 'trigger') && isfield(ta.trigger, 'expr')
        allText = allText + " " + string(ta.trigger.expr);
    end
    if isfield(ta, 'response') && isfield(ta.response, 'expr')
        allText = allText + " " + string(ta.response.expr);
    end
    if isfield(ta, 'response') && isfield(ta.response, 'untilExpr')
        allText = allText + " " + string(ta.response.untilExpr);
    end
end

function label = makeVerifyLabel(summary)
    %MAKEVERIFYLABEL Create a valid MATLAB identifier from a summary string.
    label = regexprep(char(summary), '[^a-zA-Z0-9_]', '_');
    label = regexprep(label, '_+', '_');
    label = regexprep(label, '^_|_$', '');
    if strlength(label) > 60
        label = label(1:60);
    end
end

function expr = convertToTAOperators(expr)
    %CONVERTTOTAOPERATORS Convert MATLAB short-circuit operators to TA-compatible.
    %   TA step actions use & and | (not && and ||).
    expr = string(expr);
    expr = strrep(expr, '&&', '&');
    expr = strrep(expr, '||', '|');
end

function out = ensureLogicalExpr(expr)
    %ENSURELOGICALEXPR Ensure expression is a logical comparison.
    %   TA verify() requires a scalar logical expression. Bare identifiers
    %   (e.g. "pullup") are converted to "pullup == true".
    out = expr;
    s = strtrim(char(out));
    if ~isempty(regexp(s, '^[a-zA-Z_]\w*$', 'once'))
        out = string([s ' == true']);
    end
end

function tf = needsManualEdge(ta)
    %NEEDSMANUALEDGE True if the trigger needs manual prev/cur edge detection.
    %   hasChangedTo() only accepts Input-scoped data objects — compound
    %   expressions (anything with operators or spaces) require manual edge.
    if ta.trigger.type ~= "becomes_true" && ta.trigger.type ~= "becomes_true_stays"
        tf = false;
        return
    end
    expr = strtrim(char(ta.trigger.expr));
    % A bare identifier (single word, no operators) can use hasChangedTo
    tf = isempty(regexp(expr, '^[a-zA-Z_]\w*$', 'once'));
end

function trigExpr = buildTriggerTransition(ta, edgeLocal)
    %BUILDTRIGGERTRANSITION Build the transition condition for trigger detection.
    %   edgeLocal is [] for simple triggers, or a struct with .cur/.prev for
    %   compound triggers that need manual edge detection.
    trigExpr = convertToTAOperators(ta.trigger.expr);
    switch ta.trigger.type
        case 'becomes_true'
            if ~isempty(edgeLocal)
                % Manual edge: cur is true AND prev was false
                trigExpr = sprintf('%s & ~%s', edgeLocal.cur, edgeLocal.prev);
            else
                trigExpr = sprintf('hasChangedTo(%s, true)', char(trigExpr));
            end
        case 'becomes_true_stays'
            dur = ta.trigger.duration;
            if ~isempty(dur)
                if ~isempty(edgeLocal)
                    trigExpr = sprintf('duration(%s) >= %g', edgeLocal.cur, dur);
                else
                    trigExpr = sprintf('duration(%s) >= %g', char(trigExpr), dur);
                end
            else
                if ~isempty(edgeLocal)
                    trigExpr = sprintf('%s & ~%s', edgeLocal.cur, edgeLocal.prev);
                else
                    trigExpr = sprintf('hasChangedTo(%s, true)', char(trigExpr));
                end
            end
        case 'whenever_true'
            % Level trigger — just the expression itself
            trigExpr = char(trigExpr);
    end
end

function returnCond = buildReturnTransition(ta)
    %BUILDRETURNTRANSITION Build the return transition condition.
    if ta.response.type == "weak_until" && ta.response.untilExpr ~= ""
        returnCond = char(convertToTAOperators(ta.response.untilExpr));
    elseif ta.delay.type == "at_most" && ~isempty(ta.delay.value)
        returnCond = sprintf('after(%g, sec)', ta.delay.value);
    elseif ta.response.type == "must_stay_true_at_least" && ~isempty(ta.response.duration)
        returnCond = sprintf('after(%g, sec)', ta.response.duration);
    elseif ta.response.type == "must_stay_true_at_most" && ~isempty(ta.response.duration)
        returnCond = sprintf('after(%g, sec)', ta.response.duration);
    else
        % Default: return after one time step
        returnCond = 'after(1, sec)';
    end
end

function ta = applyToleranceToTA(ta, tol)
    %APPLYTOLERANCETOTA Replace exact equality with tolerance in TA expressions.
    if isfield(ta, 'expression')
        ta.expression = applyTolExpr(ta.expression, tol);
    end
    if isfield(ta, 'response') && isfield(ta.response, 'expr')
        ta.response.expr = applyTolExpr(ta.response.expr, tol);
    end
end

function out = applyTolExpr(postcond, tol)
    %APPLYTOLEXPR Replace equality with tolerance, handling compound expressions.
    %   Splits on top-level && / || operators, applies tolerance to each
    %   clause that is a simple 'LHS == RHS', and rejoins.
    out = postcond;
    s = char(out);

    % Split into clauses at top-level logical operators
    [clauses, operators] = splitTopLevelLogical(s);

    % Apply tolerance to each clause independently
    for k = 1:numel(clauses)
        clause = strtrim(clauses{k});
        tokens = regexp(clause, '^(.+?)\s*==\s*(.+)$', 'tokens');
        if ~isempty(tokens)
            lhs = strtrim(tokens{1}{1});
            rhs = strtrim(tokens{1}{2});
            % Only transform if neither side contains logical operators
            if isempty(regexp(lhs, '&&|\|\||&(?!&)|\|(?!\|)', 'once')) && ...
               isempty(regexp(rhs, '&&|\|\||&(?!&)|\|(?!\|)', 'once'))
                clauses{k} = sprintf('abs(%s - (%s)) < %g', lhs, rhs, tol);
            end
        end
    end

    % Rejoin with original operators
    result = clauses{1};
    for k = 2:numel(clauses)
        result = [result ' ' operators{k-1} ' ' clauses{k}]; %#ok<AGROW>
    end
    out = string(result);
end

function [clauses, operators] = splitTopLevelLogical(s)
    %SPLITTOPLEVELLOGICAL Split expression on top-level && and || operators.
    %   Returns cell arrays of clauses and the operators between them.
    clauses = {};
    operators = {};
    depth = 0;
    start = 1;
    i = 1;
    while i <= length(s)
        switch s(i)
            case '(', depth = depth + 1;
            case ')', depth = depth - 1;
        end
        if depth == 0 && i < length(s)
            if s(i) == '&' && s(i+1) == '&'
                clauses{end+1} = s(start:i-1); %#ok<AGROW>
                operators{end+1} = '&&'; %#ok<AGROW>
                start = i + 2;
                i = i + 2;
                continue
            elseif s(i) == '|' && s(i+1) == '|'
                clauses{end+1} = s(start:i-1); %#ok<AGROW>
                operators{end+1} = '||'; %#ok<AGROW>
                start = i + 2;
                i = i + 2;
                continue
            end
        end
        i = i + 1;
    end
    clauses{end+1} = s(start:end);
end

function out = rewriteVectorMul(expr, vecSymbols)
    %REWRITEVECTORMUL Rewrite A * B as A' * B when both are vector symbols.
    out = expr;
    if strlength(string(out)) == 0
        return
    end
    for k = 1:numel(vecSymbols)
        for j = 1:numel(vecSymbols)
            lhs = vecSymbols{k};
            rhs = vecSymbols{j};
            pat = ['(?<!\w)(' lhs ')\s*\*\s*(' rhs ')(?!\w)'];
            out = regexprep(string(out), pat, [lhs '''' ' * ' rhs]);
        end
    end
end

function out = rewritePrev(expr, prevSymNames)
    %REWRITEPREV Replace prev(<symbol>) with prev_<symbol> in TA expressions.
    out = expr;
    if strlength(string(out)) == 0
        return
    end
    for k = 1:numel(prevSymNames)
        out = regexprep(string(out), ...
            ['prev\(' prevSymNames{k} '\)'], ['prev_' prevSymNames{k}]);
    end
end
