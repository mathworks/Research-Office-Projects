function S = sampleModelTrajectories(mdl, tGrid, radii, nSamples, varargin)
%SAMPLEMODELTRAJECTORIES Ground-truth trajectories from the ENGINE, not from a
%hand-written ODE.
%
%   S = sampleModelTrajectories(mdl, tGrid, radii, nSamples) samples the box of
%   half-width RADII about the model's own initial continuous state, simulates
%   each sample under a stock variable-step solver, and returns
%
%       S.t   numel(tGrid)-by-1
%       S.X   numel(tGrid)-by-nx-by-nSamples
%       S.X0  nx-by-nSamples, the sampled initial states
%
%   ready to hand to plotSetTube as 'Samples'.
%
%   WHY THE ENGINE AND NOT AN ODE. Writing the plant out by hand as an ODE means
%   transcribing it, and a transcription error lands precisely where the picture is
%   supposed to be ground truth. Asking the engine to re-simulate its OWN model
%   from a perturbed initial state has no transcription step to get wrong, and it
%   is what an end user would do. The cost is one simulation per sample, which is
%   why nSamples is small and explicit rather than the hundreds a membership test
%   would want.
%
%   THE BOX IS THE SAME BOX THE SOLVER STARTED FROM. SetReach.setRadii(w) makes
%   the initial set the axis-aligned box of half-width w about x0, so sampling
%   that box is the matching ground truth. The corners go in first: on a linear
%   map the corners attain the extremes, so a random-only sample understates the
%   spread exactly where it matters.
%
%   THE FLATTENING MATTERS. The engine's initial state is a Dataset of ELEMENTS,
%   and one element can hold several scalars -- a Second-Order Integrator
%   contributes a single element carrying both position and velocity. Perturbing
%   per element rather than per scalar would leave some states unperturbed and
%   silently produce a narrower truth set than the solver was given. So the
%   dataset is flattened to a scalar vector, perturbed, and scattered back.
%
%   Options
%     'Solver'    stock solver for the truth runs.                (default ode45)
%     'RelTol'    relative tolerance.                              (default 1e-10)
%     'AbsTol'    absolute tolerance.                              (default 1e-12)
%     'Seed'      rng seed, so the picture is reproducible.            (default 0)
%     'Radii'     per-state half-widths may also be given as a vector in the
%                 positional RADII argument; a scalar is expanded.
%
%   See also PLOTSETTUBE, COUNTCONTINUOUSSTATES, SETREACH.

p = inputParser;
p.addParameter('Solver', 'ode45');
p.addParameter('RelTol', '1e-10');
p.addParameter('AbsTol', '1e-12');
p.addParameter('Seed', 0);
p.parse(varargin{:});
opt = p.Results;

mdl = char(mdl);
if ~bdIsLoaded(mdl)
    load_system(mdl);
end
tGrid = tGrid(:);

% Every model parameter that is touched is restored, including on error: these
% are shipping demo models, and leaving one configured for somebody else's
% experiment is how the next run silently measures the wrong thing. The capture
% happens BEFORE the initial state is read, because reading it needs a parameter
% change too (see below).
keys = {'SolverType', 'Solver', 'RelTol', 'AbsTol', 'StopTime', ...
        'SaveState', 'StateSaveName', 'SaveFormat', 'SaveTime', 'TimeSaveName', ...
        'OutputOption', 'OutputTimes', 'ReturnWorkspaceOutputs', 'SaveOutput'};
old = cell(size(keys));
for k = 1:numel(keys)
    try old{k} = get_param(mdl, keys{k}); catch, old{k} = []; end
end
restore = onCleanup(@() restoreParams(mdl, keys, old));

% ------------------------------------------------- the model's own initial state
% SaveFormat is forced to Dataset first, and that is not cosmetic:
% Simulink.BlockDiagram.getInitialState returns a plain STRUCT when the model's
% SaveFormat is 'Array', and a Dataset otherwise. Callers reach this function
% straight after a set-solver run, which sets SaveFormat to 'Array' -- so the
% return type depends on what the previous, unrelated run happened to leave
% behind. Forcing it means one code path instead of two, and setElement below only
% exists on the Dataset.
set_param(mdl, 'SaveFormat', 'Dataset');
% THE STOCK SOLVER IS SELECTED BEFORE THE STATE IS READ, and that is not tidiness.
% getInitialState compiles the model, and compiling one whose Solver names a PLUGIN
% solver throws
%
%   SL_SERVICES:utils:STD_EXCEPTION  STD exception 'class std::out_of_range':
%   'invalid unordered_map<K, T> key'
%
% whenever the plugin solver table has been (re-)registered since that model last
% compiled -- registerSetSolvers() a second time in a session is enough, and so is
% one 'off'/'on' cycle. Measured: stock solver compiles fine, plugin solver throws,
% the same plugin solver then compiles fine once any stock compile has run, and one
% off/on cycle breaks it again. sim() is never affected, only this compile path, so
% the symptom is a hard C++ exception from a function that has nothing to do with
% the solver. That is what made it look flaky: it depends on session history, not on
% the model or the call.
%
% Switching first is free as well as safe. An initial state is a property of the
% blocks, not of the integrator, the pair is already captured above for restore, and
% the same pair is overwritten below for the truth runs anyway.
set_param(mdl, 'SolverType', 'Variable-step', 'Solver', opt.Solver);
ds  = Simulink.BlockDiagram.getInitialState(mdl);
[x0, shape] = flattenState(ds);
nx  = numel(x0);

r = radii(:);
if isscalar(r)
    r = repmat(r, nx, 1);
elseif numel(r) ~= nx
    error('sampleModelTrajectories:badRadii', ...
        ['RADII has %d entries but the continuous state vector is %d long. ' ...
         'Pass a scalar\nor one half-width per SCALAR state (not per logged ' ...
         'Dataset element -- a\nSecond-Order Integrator is one element and two ' ...
         'states).'], numel(r), nx);
end

% ------------------------------------------------------------- sample the box
rng(opt.Seed);
N = max(1, round(nSamples));
B = 2 * rand(nx, N) - 1;
% Corners, because on a linear map the extremes are attained at corners and a
% purely random sample is biased inwards precisely where the comparison against
% the tube is decided.
%
% WHICH corners matters, and enumerating them in order is a trap. With nx = 4 and
% N = 6, bitget(k-1, 1:4) for k = 1..6 never sets the top bit, so every sample
% shares the same fourth coordinate and the truth set is flat in one direction --
% measured on sldemo_foucault, where all six initial states had x4 = -0.02. So
% enumerate exhaustively only when the budget covers the corners, and otherwise
% draw random distinct sign vectors, which is unbiased across coordinates.
if N >= 2^nx
    for k = 1:2^nx
        B(:, k) = 2 * double(bitget(k-1, 1:nx))' - 1;
    end
else
    nc = max(2, floor(N/2));       % half corners, half interior
    seen = containers.Map();
    k = 0; guard = 0;
    while k < nc && guard < 1000*nc
        guard = guard + 1;
        s = sign(rand(nx,1) - 0.5);
        s(s == 0) = 1;
        key = sprintf('%d', (s+1)/2);
        if isKey(seen, key), continue, end
        seen(key) = true;
        k = k + 1;
        B(:, k) = s;
    end
end
X0 = x0 + r .* B;

% ------------------------------------------------- simulate each, on the t grid
set_param(mdl, ...
    'SolverType',             'Variable-step', ...
    'Solver',                 opt.Solver, ...
    'RelTol',                 opt.RelTol, ...
    'AbsTol',                 opt.AbsTol, ...
    'StopTime',               num2str(tGrid(end)), ...
    'SaveState',              'on', ...
    'StateSaveName',          'xout', ...
    'SaveFormat',             'Array', ...
    'SaveTime',               'on', ...
    'TimeSaveName',           'tout', ...
    'OutputOption',           'SpecifiedOutputTimes', ...
    'OutputTimes',            mat2str(tGrid'), ...
    'ReturnWorkspaceOutputs', 'on');

S    = struct();
S.t  = tGrid;
S.X  = nan(numel(tGrid), nx, N);
S.X0 = X0;
for k = 1:N
    dsk = scatterState(ds, shape, X0(:, k));
    si  = Simulink.SimulationInput(mdl);
    % InitialState on the SimulationInput and LoadInitialState as a model
    % parameter are mutually exclusive, not complementary: setting both raises
    % Simulink:Commands:SimInputRepeatedInitialStateSpec. The property route is
    % the one that leaves the model itself unmodified, so it wins.
    si.InitialState = dsk;
    so  = sim(si);
    xk  = so.xout;
    tk  = so.tout;
    if size(xk, 2) ~= nx
        error('sampleModelTrajectories:stateWidth', ...
            ['The engine logged %d state columns but the initial state ' ...
             'flattens to %d.\nThis is the element-versus-scalar trap: check ' ...
             'countContinuousStates on this model.'], size(xk, 2), nx);
    end
    % SpecifiedOutputTimes is honoured, but the engine may add its own hit times
    % (zero crossings, sample hits), so align rather than assume equality.
    S.X(:, :, k) = alignTo(tk, xk, tGrid);
end
clear restore
end

% =========================================================================

function [x, shape] = flattenState(ds)
%FLATTENSTATE Dataset of elements -> scalar column, plus what it takes to undo it.
if ~isa(ds, 'Simulink.SimulationData.Dataset')
    error('sampleModelTrajectories:noDataset', ...
        ['getInitialState returned a %s, not a Dataset. That happens when the ' ...
         'model''s\nSaveFormat is ''Array''; this function forces ''Dataset'' ' ...
         'before asking, so reaching\nhere means the set_param did not take.'], ...
        class(ds));
end
shape = struct('n', {}, 'sz', {}, 'label', {});
x = [];
for k = 1:ds.numElements
    v = ds{k}.Values.Data;
    v = v(:);
    shape(k).n     = numel(v);
    shape(k).sz    = size(ds{k}.Values.Data);
    shape(k).label = ds{k}.Label;
    x = [x; v];                                                     %#ok<AGROW>
end
end

function ds = scatterState(ds, shape, x)
%SCATTERSTATE The inverse of flattenState, into a copy of the dataset.
i = 1;
for k = 1:numel(shape)
    v = x(i : i + shape(k).n - 1);
    i = i + shape(k).n;
    e = ds{k};
    e.Values.Data = reshape(v, shape(k).sz);
    ds = ds.setElement(k, e);
end
end

function Xg = alignTo(t, X, tGrid)
%ALIGNTO Pick the engine's sample at (or immediately before) each grid time.
% Nearest rather than interpolated: at a zero crossing the engine logs the value
% twice at one time, and interpolating across that pair averages a jump into a
% ramp that no trajectory took.
Xg = nan(numel(tGrid), size(X, 2));
for j = 1:numel(tGrid)
    k = find(t <= tGrid(j) + 1e-12, 1, 'last');
    if isempty(k), k = 1; end
    Xg(j, :) = X(k, :);
end
end

function restoreParams(mdl, keys, old)
if ~bdIsLoaded(mdl), return, end
for k = 1:numel(keys)
    if isempty(old{k}), continue, end
    try
        set_param(mdl, keys{k}, old{k});
    catch
        % Some parameters become read-only once others change back. Restoring as
        % much as possible beats aborting and leaving the rest wrong.
    end
end
end
