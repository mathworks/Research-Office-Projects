function rtBlks = fretJsonToRT(jsonFile, modelName, opts)
%FRETJSONTORT Mechanical pipeline: FRET JSON export → Requirements Table block(s).
%
%   rtBlk = fretJsonToRT(JSONFILE) loads a FRET JSON export file containing
%   requirements and variable mappings, and creates a new Simulink model
%   with Requirements Table block(s). Internal (constant) variables are
%   mechanically substituted with their assigned values — no semantic
%   interpretation, matching CoCoSim's approach.
%
%   When the JSON contains multiple FRET components (e.g., an LMCPS
%   benchmark export), a separate RT block is created per component to
%   avoid cross-component symbol conflicts. Set PerComponent=false to
%   merge all requirements into a single RT block.
%
%   rtBlk = fretJsonToRT(JSONFILE, MODELNAME) creates the RT block(s) in
%   the specified model.
%
%   Name-Value Options:
%     ReqFilter    - cell array of reqids to include (default: all)
%     Tolerance    - numeric tolerance for double equality (default: 0)
%     SLDVReady    - configure model for SLDV compatibility (default: true)
%     PerComponent - create one RT block per FRET component (default: true)
%
%   Example:
%     rtBlk = fretJsonToRT('fsm_reqts_and_vars.json');
%     rtBlks = fretJsonToRT('LM_requirements.json', 'LMCPS_RT');
%     rtBlk = fretJsonToRT('lm_reqs.json', 'MyModel', PerComponent=false);
%
%   See also fretToSpec, reqRenderToRT

    arguments
        jsonFile  (1,1) string {mustBeFile}
        modelName (1,1) string = ""
        opts.ReqFilter cell = {}
        opts.Tolerance (1,1) double = 0
        opts.SLDVReady (1,1) logical = true
        opts.PerComponent (1,1) logical = true
    end

    %% 1. Load and normalize FRET JSON
    fretData = jsondecode(fileread(jsonFile));
    reqs = normalizeCellArray(fretData.requirements);
    vars = normalizeFretVars(fretData.variables);

    fprintf('Loaded %d requirements, %d variables from %s\n', ...
        numel(reqs), numel(vars), jsonFile);

    %% 2. Convert each requirement through fretToSpec
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
    rtCell    = cell(nReqs, 1);
    skipCell  = cell(nReqs, 1);
    compCell  = cell(nReqs, 1);
    tolVal    = opts.Tolerance;

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

        % Convert
        try
            spec = fretToSpec(reqid, string(r.fulltext), r.semantics, vars);
        catch ME
            skipCell{i} = struct('reqid', reqid, 'reason', string(ME.message));
            continue
        end

        % Check RT renderability
        if spec.pattern == "Summary"
            skipCell{i} = struct('reqid', reqid, 'reason', "Unsupported template");
            continue
        end

        rt = reqRenderToRT(spec);
        if ~rt.renderable
            skipCell{i} = struct('reqid', reqid, 'reason', string(rt.reason));
            continue
        end

        % Skip if expressions contain unresolved temporal operators
        combined = string(rt.precondition) + " " + string(rt.postcondition);
        if ~isempty(regexp(char(combined), '(?<!\w)persisted\s*\(', 'once'))
            skipCell{i} = struct('reqid', reqid, ...
                'reason', "Contains persisted() temporal operator — not RT-expressible.");
            continue
        end

        % Skip if expressions use external function calls (mag, dot, det_3x3)
        % that have no RT equivalent without extrinsic function definitions.
        if ~isempty(regexp(char(combined), '(mag|dot|det_3x3)\s*\(', 'once'))
            skipCell{i} = struct('reqid', reqid, ...
                'reason', "Uses external function call — not RT-expressible without extrinsic definitions.");
            continue
        end

        % Skip if prev() is called with a complex expression.
        % RT prev() only accepts a single symbol name as argument.
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
                'reason', "prev() with complex expression — RT only supports prev(single_symbol).");
            continue
        end

        % Apply tolerance to equality postconditions
        if tolVal > 0
            rt.postcondition = applyTolerance(rt.postcondition, tolVal);
        end

        rtCell{i} = rt;
    end

    % Collect results
    validIdx = ~cellfun(@isempty, rtCell);
    rtResults = rtCell(validIdx);
    compResults = compCell(validIdx);
    skipped = skipCell(~cellfun(@isempty, skipCell));

    fprintf('Converted: %d renderable, %d skipped (%.1f s)\n', ...
        numel(rtResults), numel(skipped), toc(tConvert));
    for i = 1:numel(skipped)
        fprintf('  Skipped %s: %s\n', skipped{i}.reqid, skipped{i}.reason);
    end

    if isempty(rtResults)
        error('fretJsonToRT:NoRenderableReqs', 'No requirements could be rendered to RT.');
    end

    %% 3. Derive base model name
    if modelName == ""
        [~, baseName] = fileparts(jsonFile);
        modelName = matlab.lang.makeValidName(baseName + "_RT");
    end

    %% 4. Group by component and create models
    uniqueComps = unique(string(compResults));
    if ~opts.PerComponent || isscalar(uniqueComps)
        % Single model with one RT block
        mdlName = char(modelName);
        closeAndDelete(mdlName);
        rtBlk = slreq.modeling.create(mdlName);
        cleanupObj = onCleanup(@() closeIfOpen(string(mdlName)));
        if opts.SLDVReady
            configureSLDV(mdlName);
        end
        compVars = filterVarsForComponent(vars, "All");
        populateRTBlock(rtBlk, rtResults, compVars);
        save_system(mdlName);
        clear cleanupObj
        fprintf('Saved model: %s (1 RT block, %d rows)\n', mdlName, numel(rtResults));
        rtBlks = rtBlk;
    else
        % One model per component
        rtBlks = cell(numel(uniqueComps), 1);
        for c = 1:numel(uniqueComps)
            compName = uniqueComps(c);
            idx = strcmp(string(compResults), compName);
            groupResults = rtResults(idx);

            mdlName = char(modelName + "_" + matlab.lang.makeValidName(compName));
            closeAndDelete(mdlName);
            rtBlk = slreq.modeling.create(mdlName);
            if opts.SLDVReady
                configureSLDV(mdlName);
            end
            compVars = filterVarsForComponent(vars, compName);
            populateRTBlock(rtBlk, groupResults, compVars);
            save_system(mdlName);
            rtBlks{c} = rtBlk;
            fprintf('  %s: %d rows → %s.slx\n', compName, numel(groupResults), mdlName);
        end
        fprintf('Created %d models from %s\n', numel(uniqueComps), modelName);
    end
end

%% --- Core RT Block Population ---

function populateRTBlock(rtBlk, rtResults, vars)
    %POPULATERTBLOCK Add symbols and requirement rows to an RT block.

    % Extract identifiers from all RT expressions
    allIdentifiers = {};
    postcondIdentifiers = {};
    precondDirectIdentifiers = {};
    for i = 1:numel(rtResults)
        rt = rtResults{i};
        expr = char(string(rt.precondition) + " " + string(rt.postcondition));
        tokens = regexp(expr, '(?<!\w)([a-zA-Z_]\w*)(?!\w)', 'tokens');
        for k = 1:numel(tokens)
            allIdentifiers{end+1} = tokens{k}{1}; %#ok<AGROW>
        end
        postExpr = char(string(rt.postcondition));
        postTokens = regexp(postExpr, '(?<!\w)([a-zA-Z_]\w*)(?!\w)', 'tokens');
        for k = 1:numel(postTokens)
            postcondIdentifiers{end+1} = postTokens{k}{1}; %#ok<AGROW>
        end
        preExpr = char(string(rt.precondition));
        stripped = regexprep(preExpr, 'prev\([^)]*\)', '');
        preTokens = regexp(stripped, '(?<!\w)([a-zA-Z_]\w*)(?!\w)', 'tokens');
        for k = 1:numel(preTokens)
            precondDirectIdentifiers{end+1} = preTokens{k}{1}; %#ok<AGROW>
        end
    end
    allIdentifiers = unique(allIdentifiers);
    postcondIdentifiers = unique(postcondIdentifiers);
    precondDirectIdentifiers = unique(precondDirectIdentifiers);

    canBeDesignOutput = setdiff(postcondIdentifiers, precondDirectIdentifiers);

    % Add symbols for FRET variables that appear in expressions.
    % Skip Function-type variables.
    addedSymbols = {};
    designOutputs = {};
    fretFunctions = {};
    for j = 1:numel(vars)
        v = vars{j};
        vn = string(v.variable_name);
        if isfield(v, 'idType') && strcmpi(v.idType, 'Function')
            fretFunctions{end+1} = char(vn); %#ok<AGROW>
            continue
        end
        if ~ismember(char(vn), allIdentifiers) || any(strcmp(char(vn), addedSymbols))
            continue
        end
        rtBlk.addSymbol('Name', char(vn), 'Scope', 'Input');
        if isfield(v, 'idType') && strcmpi(v.idType, 'Output') ...
                && ismember(char(vn), canBeDesignOutput)
            designOutputs{end+1} = char(vn); %#ok<AGROW>
        end
        addedSymbols{end+1} = char(vn); %#ok<AGROW>
    end

    % Add remaining identifiers not in the FRET variable list.
    builtins = {'prev','abs','cos','sin','max','min','xor','sign','sqrt',...
        'true','false','nan','inf','pi','end','if','else','elseif',...
        'persisted','__past_first_step','mag','dot','det_3x3'};
    builtins = [builtins, fretFunctions];
    for j = 1:numel(allIdentifiers)
        id = allIdentifiers{j};
        if any(strcmp(id, addedSymbols)) || ismember(id, builtins)
            continue
        end
        rtBlk.addSymbol('Name', id, 'Scope', 'Input');
        if ismember(id, canBeDesignOutput)
            designOutputs{end+1} = id; %#ok<AGROW>
        end
        addedSymbols{end+1} = id; %#ok<AGROW>
    end

    % If no FRET Output variables were identified, infer design outputs
    % from postcondition structure: use canBeDesignOutput symbols that are
    % actual RT symbols (not builtins).
    if isempty(designOutputs) && ~isempty(canBeDesignOutput)
        candidates = intersect(canBeDesignOutput, addedSymbols);
        candidates = setdiff(candidates, builtins);
        for k = 1:numel(candidates)
            designOutputs{end+1} = candidates{k}; %#ok<AGROW>
        end
    end

    % Mark design output symbols and filter out incompatible rows.
    if ~isempty(designOutputs)
        for k = 1:numel(designOutputs)
            sym = rtBlk.findSymbol(Name=designOutputs{k});
            if ~isempty(sym)
                sym.IsDesignOutput = true;
            end
        end
        % Remove rows whose postconditions lack a design output reference
        keepIdx = true(numel(rtResults), 1);
        for i = 1:numel(rtResults)
            postExpr = char(string(rtResults{i}.postcondition));
            hasOutput = false;
            for k = 1:numel(designOutputs)
                if ~isempty(regexp(postExpr, ['(?<!\w)' designOutputs{k} '(?!\w)'], 'once'))
                    hasOutput = true;
                    break
                end
            end
            if ~hasOutput
                keepIdx(i) = false;
                fprintf('    Excluded %s: postcondition lacks a design output symbol.\n', ...
                    rtResults{i}.summary);
            end
        end
        rtResults = rtResults(keepIdx);
    end
    % Set InitialValue for symbols used inside prev() — RT requires it.
    allExpr = "";
    for i = 1:numel(rtResults)
        allExpr = allExpr + " " + string(rtResults{i}.precondition) ...
            + " " + string(rtResults{i}.postcondition);
    end
    prevSyms = regexp(char(allExpr), 'prev\(([a-zA-Z_]\w*)\)', 'tokens');
    prevSymNames = unique(cellfun(@(t) t{1}, prevSyms, 'UniformOutput', false));
    for k = 1:numel(prevSymNames)
        sym = rtBlk.findSymbol(Name=prevSymNames{k});
        if ~isempty(sym) && isempty(sym.InitialValue)
            sym.InitialValue = '0';
        end
    end

    % Infer vector dimensions from indexing expressions like var(3).
    % RT requires explicit Size for vector signals.
    vecDims = containers.Map('KeyType','char','ValueType','double');
    idxTokens = regexp(char(allExpr), '([a-zA-Z_]\w*)\s*\(\s*(\d+)\s*\)', 'tokens');
    for k = 1:numel(idxTokens)
        vName = idxTokens{k}{1};
        idx = str2double(idxTokens{k}{2});
        if any(strcmp(vName, addedSymbols)) && ~any(strcmp(vName, builtins))
            if ~vecDims.isKey(vName) || vecDims(vName) < idx
                vecDims(vName) = idx;
            end
        end
    end
    % Propagate dimensions through multiplication: if A * B and one is
    % a known vector, the other must be the same size for a valid product.
    if ~isempty(vecDims.keys)
        mulTokens = regexp(char(allExpr), '([a-zA-Z_]\w*)\s*\*\s*([a-zA-Z_]\w*)', 'tokens');
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
        sym = rtBlk.findSymbol(Name=vecSymbols{k});
        if ~isempty(sym)
            sym.Size = char(string(vecDims(vecSymbols{k})));
        end
    end

    % Rewrite vector-vector multiplications as dot products (A' * B) so
    % that the result is scalar. RT verify() requires scalar expressions.
    if ~isempty(vecSymbols)
        for i = 1:numel(rtResults)
            rtResults{i}.postcondition = rewriteVectorMul( ...
                rtResults{i}.postcondition, vecSymbols);
            rtResults{i}.precondition = rewriteVectorMul( ...
                rtResults{i}.precondition, vecSymbols);
        end
    end

    fprintf('    %d symbols (%d design outputs), %d rows\n', ...
        numel(addedSymbols), numel(designOutputs), numel(rtResults));

    % Add requirement rows
    for i = 1:numel(rtResults)
        rt = rtResults{i};
        postCond = ensureLogicalExpr(rt.postcondition);

        if i == 1
            defaultRow = rtBlk.getRequirementRows();
            defaultRow(1).Summary = char(rt.summary);
            defaultRow(1).Preconditions = {char(rt.precondition)};
            defaultRow(1).Postconditions = {char(postCond)};
        elseif rt.precondition ~= ""
            rtBlk.addRequirementRow(...
                'Summary', char(rt.summary), ...
                'Preconditions', {char(rt.precondition)}, ...
                'Postconditions', {char(postCond)});
        else
            rtBlk.addRequirementRow(...
                'Summary', char(rt.summary), ...
                'Postconditions', {char(postCond)});
        end
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

function configureSLDV(mdlName)
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
    %   with canonical fields: variable_name, idType, assignment, dataType,
    %   component_name. Deduplicates by (variable_name, component_name) pair
    %   so per-component filtering works correctly.

    if ~iscell(rawVars)
        rawVars = num2cell(rawVars);
    end

    vars = {};
    seenKeys = {};
    for i = 1:numel(rawVars)
        rv = rawVars{i};
        v.variable_name = getFieldOr(rv, 'variable_name', '');
        if isempty(v.variable_name), continue; end
        v.component_name = getFieldOr(rv, 'component_name', '');
        key = [v.variable_name '::' v.component_name];
        if any(strcmp(key, seenKeys))
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
        seenKeys{end+1} = key; %#ok<AGROW>
    end
end

function compVars = filterVarsForComponent(vars, compName)
    %FILTERVARSFORCOMPONENT Return vars relevant to a component.
    %   Includes vars from the named component plus vars with no component.
    %   Deduplicates by variable_name (component-specific entry wins over global).
    %   For the merged case ("All"), returns all vars deduplicated by name.
    if compName == "All"
        compVars = {};
        seen = {};
        for i = 1:numel(vars)
            vn = vars{i}.variable_name;
            if ~any(strcmp(vn, seen))
                compVars{end+1} = vars{i}; %#ok<AGROW>
                seen{end+1} = vn; %#ok<AGROW>
            end
        end
        return
    end
    compVars = {};
    seen = {};
    % First pass: add component-specific vars
    for i = 1:numel(vars)
        v = vars{i};
        if strcmp(v.component_name, char(compName))
            compVars{end+1} = v; %#ok<AGROW>
            seen{end+1} = v.variable_name; %#ok<AGROW>
        end
    end
    % Second pass: add global vars not already covered
    for i = 1:numel(vars)
        v = vars{i};
        if isempty(v.component_name) && ~any(strcmp(v.variable_name, seen))
            compVars{end+1} = v; %#ok<AGROW>
            seen{end+1} = v.variable_name; %#ok<AGROW>
        end
    end
end

function val = getFieldOr(s, field, default)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = default;
    end
end

function out = applyTolerance(postcond, tol)
    %APPLYTOLERANCE Replace exact equality with tolerance-based comparison.
    out = postcond;
    s = char(out);
    [clauses, operators] = splitTopLevelLogical(s);
    for k = 1:numel(clauses)
        clause = strtrim(clauses{k});
        tokens = regexp(clause, '^(.+?)\s*==\s*(.+)$', 'tokens');
        if ~isempty(tokens)
            lhs = strtrim(tokens{1}{1});
            rhs = strtrim(tokens{1}{2});
            if isempty(regexp(lhs, '&&|\|\||&(?!&)|\|(?!\|)', 'once')) && ...
               isempty(regexp(rhs, '&&|\|\||&(?!&)|\|(?!\|)', 'once'))
                clauses{k} = sprintf('abs(%s - (%s)) < %g', lhs, rhs, tol);
            end
        end
    end
    result = clauses{1};
    for k = 2:numel(clauses)
        result = [result ' ' operators{k-1} ' ' clauses{k}]; %#ok<AGROW>
    end
    out = string(result);
end

function [clauses, operators] = splitTopLevelLogical(s)
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

function out = ensureLogicalExpr(expr)
    out = expr;
    s = strtrim(char(out));
    if ~isempty(regexp(s, '^[a-zA-Z_]\w*$', 'once'))
        out = string([s ' == true']);
    end
end

function out = rewriteVectorMul(expr, vecSymbols)
    %REWRITEVECTORMUL Rewrite A * B as A' * B when both are vector symbols.
    %   RT verify() requires scalar logical expressions. When two vector
    %   symbols are multiplied, the intent is a dot product — rewrite the
    %   left operand with transpose to produce a scalar result.
    out = expr;
    if out == ""
        return
    end
    for k = 1:numel(vecSymbols)
        for j = 1:numel(vecSymbols)
            lhs = vecSymbols{k};
            rhs = vecSymbols{j};
            pat = ['(?<!\w)(' lhs ')\s*\*\s*(' rhs ')(?!\w)'];
            out = regexprep(out, pat, [lhs ''' * ' rhs]);
        end
    end
end
