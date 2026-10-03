function R = splitSetReach(mdl, opts)
%SPLITSETREACH Split the initial set until each piece's linearisation is trusted.
%   R = splitSetReach(mdl, Tol = tol) propagates the model's initial set with a
%   set solver, then bisects it, re-simulating each piece from its own centre,
%   until the HSCC'09 error indicator of every piece is at or below TOL. The
%   result is a union of pieces, each with its own reach tube.
%
%   WHY SPLIT. Every set solver here linearises about ONE centre trajectory, so
%   the further a point of the set is from that centre, the worse its predicted
%   position (Lemma 1 of HSCC'09: the error is O(||S||^2) in the set's diameter).
%   A piece half as wide, linearised about its own centre, has about a quarter of
%   the error. That cuts both ways: true states the tube misses, and tube width no
%   true state reaches.
%
%   THE INDICATOR is Proposition 1 of Donze, Krogh and Rajhans, HSCC'09, in the
%   generator form a zonotope affords. A child piece Pj of a parent P is written
%   in P's own coordinates as  c0(Pj) = c0(P) + G0(P)*db,  G0(Pj) = G0(P)*D,  so
%   P's linear estimate of Pj at time t is  { xi_P(t) + G_P(t)*(db + D*b) }  and
%   Pj's own is  { xi_Pj(t) + G_Pj(t)*b },  with ||b||_inf <= 1 for both. Their
%   Hausdorff distance is at most
%
%       Err(t) = ||xi_Pj(t) - xi_P(t) - G_P(t)*db||  +  sum_i ||(G_Pj(t) - G_P(t)*D) e_i||
%
%   which is the paper's term 1 (where the parent says the child's centre goes,
%   against where it goes) plus term 2 (how far the sensitivities drifted, scaled
%   by the child's extent). A piece is ACCEPTED when max_t Err(t) <= Tol.
%
%   WHY EVERY DIRECTION AT ONCE. Err is a difference, parent against child, so it
%   only sees the error the split removed. Halve one direction and the error
%   from the others is left whole and invisible: on vdp that accepted pieces
%   whose true error was 2.5 times their Err, and tightening Tol stopped
%   helping. Halve every direction and every second-order term, cross terms
%   included, falls by 4 in the limit; on vdp the true error fell by 3.3 to 3.6
%   per level, and Err tracked the parent's true error less the child's. Only
%   directions the initial set has are split: a state given no spread is not.
%   Split = 'one' keeps the cheaper rule for comparison.
%
%   WHAT AN ACCEPTED PIECE MEANS, stated narrowly because the method invites more.
%   Err bounds the disagreement between two LINEARISATIONS, the parent's and the
%   child's; it is not a distance to the true reachable set. On the shipping vdp
%   at Tol = 0.05, 955 leaves, every leaf's true error at its four corners,
%   against ode45 at RelTol 1e-10, was at most 0.042, but on 233 leaves it
%   exceeded that leaf's own Err, by up to 1.4 times. Err is an
%   estimate that drives refinement, not a per-piece bound. So the union is not
%   a certified enclosure. It is the HSCC'09 guarantee: refinement stops where
%   splitting further no longer changes the answer by more than Tol.
%
%   THE STEP'S OWN ERROR. Parent and child share the stepper, so Err cannot see
%   the error of the step itself, and no Tol lowers it; only a smaller step does.
%   With StepCheck, the leaf with the largest Err and the three widest are run
%   once more at half the step, R.stepErr is the most any of them moved, and a
%   warning is issued when that exceeds Tol/2. It is a sample, not every leaf.
%   The root is not the one to check: the generator part of the step error
%   scales with the set, and the unsplit tube's is set by the very width that
%   splitting replaces.
%
%   A run that stops refining before every piece meets Tol, at MaxDepth or
%   MaxPieces, warns and sets R.resolved = false. It does not fail silently.
%
%   NO STATE JUMPS. A run in which a continuous state jumps, a bouncing ball's
%   impact, is an error, splitSetReach:stateJump, carrying the time SetReach
%   reports. The jump map is not visible to a solver, so a set cannot be carried
%   across it, and no amount of splitting changes that. Stop before the first
%   jump. Only a jump of a piece's own centre is seen: a set whose edge crosses
%   the guard while its centre does not is not detected.
%
%   WHY AN OUTER LOOP. A plugin solver advances one trajectory, so a set cannot
%   branch inside a run. It does not need to. The logged set is S(t) = Phi(t)*G0
%   in the coordinates b of the initial set, so splitting at any time is exactly
%   a split of the initial set, and a piece is a fresh run from t = 0 with a
%   smaller G0 and a shifted centre.
%   That is the shape Breach and S-TaLiRo use too: many calls to sim, nothing
%   inside the model. Each level of the refinement is one batch, run under Fast
%   Restart, in parallel when a pool is available.
%
%   HOW A PIECE REACHES THE SOLVER, both measured on the shipping vdp:
%     centre   the InitialState model parameter, as an array in the engine's own
%              state order. Works serially, under Fast Restart and under parsim.
%              The array is the root's whole saved initial state with only its
%              continuous entries replaced, so discrete states, which carry no
%              spread, start every piece where they started the root.
%     spread   SetReach.config(), set in a PreSimFcn so it also runs on parsim
%              workers. NOT a SimulationInput variable: setVariable(...,
%              'Workspace', mdl) is invisible to the ModelWorkspace object, so
%              SetReach would ignore it. config() also outranks a model SetIC,
%              and every run is checked afterwards against the G0 it was given.
%
%   Options
%     Tol            accept a piece when max_t Err <= Tol.            (required)
%     Radii          initial spread, in SetReach.setRadii's form: scalar, per-
%                    state vector, or generator matrix. Empty uses whatever a
%                    plain sim of the model would use (model SetIC, then
%                    SetReach.setRadii).                          (default [])
%     Solver         a fixed-step SetReach solver that propagates a zonotope;
%                    any other errors.             (default SetReachZonotope)
%     StopTime       char or number.                   (default: the model's own)
%     FixedStep      char or number.                   (default: the model's own)
%     MaxDepth       split at most this many times along any branch.  (default 8)
%     MaxPieces      stop splitting once this many pieces exist, counting every
%                    piece simulated; vdp at Tol = 0.05 needs 1273. (default 2048)
%     Relevant       @(piece) -> logical. A piece that fails Tol is only split if
%                    this says it matters, e.g. its tube nears a bad set. That is
%                    HSCC'09's refinement near the boundary; the default refines
%                    everywhere.                                  (default [])
%     Split          'all' halves every direction of the initial set, 2^m
%                    children; 'one' halves only the direction whose
%                    generator drifted most, 2 children, and under-reports
%                    error (see above).                        (default 'all')
%     Parallel       'auto' uses parsim when Parallel Computing Toolbox is
%                    installed and a pool is open, and runs serially otherwise;
%                    true opens a pool and needs the toolbox.  (default 'auto')
%     UseFastRestart                                          (default true)
%     StepCheck      rerun a few leaves at half the step, see above.
%                                                                 (default true)
%
%   R.pieces is a struct array, one per piece ever simulated, with
%     lo, hi     the piece as a box in the root's b coordinates, in [-1, 1]^m
%     c0, G0     its initial centre and generators
%     c, G       n-by-K centres and n-by-m-by-K generators on R.t
%     parent     index into R.pieces, 0 for the root
%     depth      0 for the root
%     err        max_t Err against the parent, NaN for the root
%     status     'accepted'     err <= Tol
%                'split'        refined; its children replace it
%                'unresolved'   err > Tol, but MaxDepth or MaxPieces stopped it
%                'coarse'       err > Tol, but Relevant said it does not matter
%   R.leaves indexes the pieces that together cover the initial set. R.resolved
%   is false when any leaf is 'unresolved'. R.stepErr is the step check's
%   measurement, NaN without it. R.stats counts simulations and batches and
%   times them.
%
%   See also SETREACH, PLOTSPLITSETTUBE, VDPSPLITTUBE.

arguments
    mdl (1,:) char
    opts.Tol (1,1) double {mustBePositive}
    opts.Radii double = []
    opts.Solver (1,:) char = 'SetReachZonotope'
    opts.StopTime = []
    opts.FixedStep = []
    opts.MaxDepth (1,1) double {mustBeNonnegative, mustBeInteger} = 8
    opts.MaxPieces (1,1) double {mustBePositive, mustBeInteger} = 2048
    opts.Relevant = []
    opts.Split (1,:) char {mustBeMember(opts.Split, {'all', 'one'})} = 'all'
    opts.Parallel = 'auto'
    opts.UseFastRestart (1,1) logical = true
    opts.StepCheck (1,1) logical = true
end
if ~isfield(opts, 'Tol')
    error('splitSetReach:noTol', 'Tol is required: splitSetReach(mdl, Tol = 0.01).');
end
if ~bdIsLoaded(mdl)
    load_system(mdl);
end
if ~isempty(which(opts.Solver)) && ~any(strcmp(superclasses(opts.Solver), ...
        'Simulink.Solver.FixedStepSolver'))
    % Err compares a parent and a child at the same times; two variable-step
    % runs choose different grids. Resampling through SetReach.dense would lift
    % this, and is the natural next step, but it is not done yet.
    error('splitSetReach:variableStep', ...
        '%s is not a fixed-step solver. splitSetReach needs a common time grid.', ...
        opts.Solver);
end
if ~strcmp(opts.Solver, 'SetReach') && ~any(strcmp(superclasses(opts.Solver), 'SetReach'))
    % Pieces are read back from SetReach's log, so a solver that does not
    % write it would leave the previous run's log to be read instead.
    error('splitSetReach:solver', ...
        '%s is not a SetReach solver. Use SetReachZonotope.', opts.Solver);
end

par = useParallel(opts.Parallel);
tStart = tic;

% config() and setRadii() are process-wide statics, so a caller's own values
% are put back afterwards. On parsim workers they are set per run anyway.
oldCfg = SetReach.config();
oldRad = SetReach.setRadii();
restore = onCleanup(@() restoreStatics(oldCfg, oldRad));

% ------------------------------------------------------------------ the root
root = runBatch(mdl, opts, par, {[]}, {[]}, []);
L0 = root{1};
if ~strcmp(L0.S{1}.kind, 'zonotope')
    % A zonotope's box of coefficients halves into children that cover it
    % exactly; an ellipsoid can only be covered, with overlap, and the
    % indicator is written in generator form.
    error('splitSetReach:shape', ...
        ['%s propagates the %s representation, and splitting needs a ' ...
         'zonotope. Use SetReachZonotope.'], opts.Solver, L0.S{1}.kind);
end
c0 = L0.c{1}(:);
G0 = L0.S{1}.G;
% The whole initial state the root started from, discrete states included. A
% piece replaces only the continuous entries, so its discrete states start
% exactly where the root's did. The continuous entries must be the logged
% centre, or the state order is not the one assumed here.
X0 = L0.X0;
if nnz(X0.cont) ~= numel(c0) || norm(X0.x(X0.cont) - c0) > 1e-12 * max(1, norm(c0))
    error('splitSetReach:stateLayout', ...
        'The root''s saved initial state does not match its logged centre.');
end
if isempty(G0)
    error('splitSetReach:pointSet', ...
        ['The initial set is a single point, so there is nothing to split. ' ...
         'Pass Radii, or configure SetIC or SetReach.setRadii.']);
end
if ~isempty(opts.Radii)
    want = SetRep.expandRadii(opts.Radii, numel(c0));
    if norm(G0 - want, 1) > 1e-12 * max(1, norm(want, 1))
        error('splitSetReach:spreadOverridden', ...
            ['The root run did not start from Radii; something with higher ' ...
             'precedence (a model-workspace SetIC, or SetReach.config) won.']);
    end
end
[n, m] = size(G0);
t = L0.t(:);
P = makePiece(-ones(m, 1), ones(m, 1), c0, G0, L0, 0, 0, t);
P.status = 'split';
nSims = 1;
nBatches = 1;

% Only directions the set actually has are split. A zero generator, a state
% given no spread, would be halved into two identical children.
live = find(vecnorm(G0, 2, 1) > 0);
if strcmp(opts.Split, 'all'), perSplit = 2^numel(live); else, perSplit = 2; end
if 1 + perSplit > opts.MaxPieces
    error('splitSetReach:tooManyDirections', ...
        ['Splitting all %d directions makes %d children per split, more than ' ...
         'MaxPieces = %d allows. Raise MaxPieces, give fewer states a spread, ' ...
         'or use Split = ''one''.'], numel(live), perSplit, opts.MaxPieces);
end

% ---------------------------------------------------------- refine by levels
toSplit = 1;
while ~isempty(toSplit)
    % Every piece to split becomes its children. Build them all, run them as
    % one batch, then judge each against its parent.
    kids = struct('lo', {}, 'hi', {}, 'parent', {});
    for p = toSplit
        if strcmp(opts.Split, 'all')
            dims = live;
        else
            dims = splitDim(P, p);
        end
        for c = bisect(P(p).lo, P(p).hi, dims)
            kids(end+1) = struct('lo', c.lo, 'hi', c.hi, 'parent', p); %#ok<AGROW>
        end
    end
    cs = cell(1, numel(kids));
    gs = cell(1, numel(kids));
    for k = 1:numel(kids)
        [cs{k}, gs{k}] = pieceIC(c0, G0, kids(k).lo, kids(k).hi);
    end
    logs = runBatch(mdl, opts, par, cs, gs, X0);
    nSims = nSims + numel(kids);
    nBatches = nBatches + 1;

    toSplit = [];
    for k = 1:numel(kids)
        L = logs{k};
        if ~isequal(size(L.t(:)), size(t)) || max(abs(L.t(:) - t)) > 1e-9
            error('splitSetReach:grid', ...
                'A piece ran on a different time grid from the root.');
        end
        if norm(L.c{1}(:) - cs{k}) > 1e-9 * max(1, norm(cs{k})) || ...
                norm(L.S{1}.G - gs{k}, 1) > 1e-12 * max(1, norm(gs{k}, 1))
            error('splitSetReach:icIgnored', ...
                'A piece did not start from the centre and spread it was given.');
        end
        q = makePiece(kids(k).lo, kids(k).hi, cs{k}, gs{k}, L, ...
            kids(k).parent, P(kids(k).parent).depth + 1, t);
        q.err = expansionError(P(kids(k).parent), q);
        if q.err <= opts.Tol
            q.status = 'accepted';
        elseif ~isempty(opts.Relevant) && ~opts.Relevant(q)
            q.status = 'coarse';
        elseif q.depth >= opts.MaxDepth || ...
                numel(kids) - k + numel(P) + 1 + perSplit * (numel(toSplit) + 1) > opts.MaxPieces
            % Counted as: kids still to be judged, pieces so far, this one, and
            % the children of every piece queued to split including this one.
            q.status = 'unresolved';
        else
            q.status = 'split';
        end
        P(end+1) = q; %#ok<AGROW>
        if strcmp(q.status, 'split')
            toSplit(end+1) = numel(P); %#ok<AGROW>
        end
    end
end

leaves = find(~strcmp({P.status}, 'split'));

% --------------------------------------------------------- the step's own error
% Err cannot see it, so it is measured directly: the leaf with the largest Err
% and the widest few are run again at half the step and compared at the shared
% times. Leaves, not the root: the generator part scales with the set, and the
% root's is set by a tube that splitting has just replaced.
stepErr = NaN;
if opts.StepCheck
    [~, worst] = max([P(leaves).err]);
    width = arrayfun(@(q) max(sum(vecnorm(q.G, 2, 1), 2)), P(leaves));
    [~, wide] = sort(width, 'descend');
    probe = leaves(unique([worst, wide(1:min(3, end))], 'stable'));
    half = opts;
    half.FixedStep = median(diff(t)) / 2;
    Lh = runBatch(mdl, half, par, {P(probe).c0}, {P(probe).G0}, X0);
    stepErr = 0;
    for k = 1:numel(probe)
        stepErr = max(stepErr, stepError(P(probe(k)), t, Lh{k}));
    end
    nSims = nSims + numel(probe);
    nBatches = nBatches + 1;
    if stepErr > opts.Tol / 2
        warning('splitSetReach:stepError', ...
            ['Halving the step moves an accepted piece by %.3g, more than half ' ...
             'of Tol = %g. Err cannot see that error, so a smaller FixedStep is ' ...
             'needed before Tol means what it says.'], stepErr, opts.Tol);
    end
end

R.mdl = mdl;
R.t = t;
R.n = n;
R.tol = opts.Tol;
R.pieces = P;
R.leaves = leaves;
R.stepErr = stepErr;
R.stats = struct('sims', nSims, 'batches', nBatches, 'parallel', par, ...
    'seconds', toc(tStart));
open = strcmp({P(R.leaves).status}, 'unresolved');
R.resolved = ~any(open);
if ~R.resolved
    warning('splitSetReach:unresolved', ...
        ['%d of %d pieces still fail Tol = %g (worst Err %.3g): MaxDepth = %d or ' ...
         'MaxPieces = %d stopped them. The union is not refined to Tol there.'], ...
        nnz(open), numel(R.leaves), opts.Tol, max([P(R.leaves(open)).err]), ...
        opts.MaxDepth, opts.MaxPieces);
end
end

% ===================================================================== helpers

function c = bisect(lo, hi, dims)
%BISECT The 2^numel(dims) boxes from halving [lo, hi] along each of dims.
c = struct('lo', lo, 'hi', hi);
for d = dims
    mid = (lo(d) + hi(d)) / 2;
    a = c; b = c;
    for i = 1:numel(c)
        a(i).hi(d) = mid;
        b(i).lo(d) = mid;
    end
    c = [a, b];
end
end

function [c, G] = pieceIC(c0, G0, lo, hi)
%PIECEIC Centre and generators of the box [lo, hi] in the root's b coordinates.
c = c0 + G0 * ((lo + hi) / 2);
G = G0 * diag((hi - lo) / 2);
end

function p = makePiece(lo, hi, c0, G0, L, parent, depth, t)
K = numel(t);
[n, m] = size(G0);
c = zeros(n, K);
G = zeros(n, m, K);
for k = 1:K
    c(:, k) = L.c{k}(:);
    G(:, :, k) = L.S{k}.G;
end
p = struct('lo', lo, 'hi', hi, 'c0', c0, 'G0', G0, 'c', c, 'G', G, ...
    'parent', parent, 'depth', depth, 'err', NaN, 'errT', [], 'status', '');
end

function e = expansionError(P, q)
%EXPANSIONERROR max_t of HSCC'09 Proposition 1, generator form. See the header.
hwP = (P.hi - P.lo) / 2;
db  = ((q.lo + q.hi) / 2 - (P.lo + P.hi) / 2) ./ hwP;
D   = diag(((q.hi - q.lo) / 2) ./ hwP);
K = size(q.c, 2);
errT = zeros(1, K);
for k = 1:K
    term1 = norm(q.c(:, k) - P.c(:, k) - P.G(:, :, k) * db);
    term2 = sum(vecnorm(q.G(:, :, k) - P.G(:, :, k) * D));
    errT(k) = term1 + term2;
end
e = max(errT);
end

function e = stepError(q, t, Lh)
%STEPERROR max over the shared times of how far piece q moved at half the step,
%   centre plus generators, the same norm as Err with no split.
th = Lh.t(:);
e = 0;
for k = 1:numel(t)
    j = find(abs(th - t(k)) <= 1e-9 * max(1, abs(t(k))), 1);
    if isempty(j)
        error('splitSetReach:stepGrid', 'The half-step run missed time %g.', t(k));
    end
    e = max(e, norm(q.c(:, k) - Lh.c{j}(:)) + sum(vecnorm(q.G(:, :, k) - Lh.S{j}.G)));
end
end

function d = splitDim(P, p)
%SPLITDIM Which coordinate of the initial set to bisect.
%   A piece that failed against its parent is split along the generator whose
%   drift (term 2 of Err) was largest, which is the direction its linearisation
%   is worst in. The root has no parent, so it is split along the generator
%   whose image grows largest over the run.
q = P(p);
if q.parent == 0
    w = squeeze(max(vecnorm(q.G, 2, 1), [], 3));
else
    par = P(q.parent);
    hwP = (par.hi - par.lo) / 2;
    D = diag(((q.hi - q.lo) / 2) ./ hwP);
    w = zeros(1, size(q.G, 2));
    for k = 1:size(q.G, 3)
        w = max(w, vecnorm(q.G(:, :, k) - par.G(:, :, k) * D));
    end
end
[~, d] = max(w);
end

function logs = runBatch(mdl, opts, par, cs, gs, X0)
%RUNBATCH One simulation per piece, returning each run's SetReach log. An empty
%centre leaves the model's own initial state, and saves it whole in the log's
%X0; otherwise X0 supplies the discrete entries. An empty spread leaves the
%model's own initial set, or Radii when given.
N = numel(cs);
for k = N:-1:1
    in = Simulink.SimulationInput(mdl);
    in = in.setModelParameter('SolverType', 'Fixed-step', 'Solver', opts.Solver);
    % Nothing the engine logs is read: a run's result is SetReach's own log,
    % taken in the PostSimFcn. Off, a run of vdp is 0.2 s cheaper, a third of
    % what Fast Restart costs per run before the first step.
    in = in.setModelParameter('SaveOutput', 'off', 'SaveTime', 'off', ...
        'SignalLogging', 'off', 'DSMLogging', 'off', 'SaveState', 'off');
    if ~isempty(opts.StopTime)
        in = in.setModelParameter('StopTime', num2str(opts.StopTime, 17));
    end
    if ~isempty(opts.FixedStep)
        in = in.setModelParameter('FixedStep', num2str(opts.FixedStep, 17));
    end
    if isempty(cs{k})
        in = in.setModelParameter('SaveState', 'on', 'StateSaveName', 'splitX0', ...
            'SaveFormat', 'Structure');
    else
        % Array format is what makes this model-agnostic: it is the engine's own
        % state order, the same order the log uses for the continuous entries.
        % The diagnostic it raises is about Rapid Accelerator rebuilds, which do
        % not apply here.
        x = X0.x;
        x(X0.cont) = cs{k};
        in = in.setModelParameter('LoadInitialState', 'on', ...
            'InitialState', mat2str(x(:)', 17), 'InitInArrayFormatMsg', 'none');
    end
    in = in.setPreSimFcn(@(~) presetSet(gs{k}, opts.Radii));
    in = in.setPostSimFcn(@takeLog);
    ins(k) = in;
end
fr = matlab.lang.OnOffSwitchState(opts.UseFastRestart);
% A jump is reported once below, as an error, rather than once per piece.
jumpId = 'SetReach:stateJump';
if par
    out = parsim(ins, 'UseFastRestart', char(fr), 'ShowProgress', 'off', ...
        'SetupFcn', @() workerSetup(closureFolders(opts.Solver), opts.Solver), ...
        'CleanupFcn', @() warning('on', jumpId));
else
    ws = warning('off', jumpId);
    unmute = onCleanup(@() warning(ws));
    out = sim(ins, 'UseFastRestart', char(fr), 'ShowProgress', 'off');
    clear unmute
end
logs = cell(1, N);
for k = 1:N
    if ~isempty(out(k).ErrorMessage)
        error('splitSetReach:simFailed', 'Piece %d failed to simulate: %s', ...
            k, out(k).ErrorMessage);
    end
    if ~isempty(out(k).jump)
        error('splitSetReach:stateJump', ...
            ['A run crossed a state jump, and splitting cannot refine across one: ' ...
             'each piece is linearised about one smooth trajectory, and the jump ' ...
             'map is not visible to the solver. Set StopTime before the jump. ' ...
             'SetReach reported: %s'], out(k).jump);
    end
    if isempty(out(k).L.t)
        error('splitSetReach:noLog', 'Piece %d ran but logged no set.', k);
    end
    logs{k} = out(k).L;
    logs{k}.X0 = out(k).X0;
end
end

function presetSet(G, radii)
% A run that logs nothing then returns an empty log, never the last run's.
SetReach.resetLog();
if ~isempty(G)
    SetReach.config(struct('kind', 'zonotope', 'G', G));
else
    SetReach.config([]);
    if ~isempty(radii)
        SetReach.setRadii(radii);
    end
end
end

function s = takeLog(so)
%TAKELOG The run's log, its whole initial state if it saved one, and SetReach's
%   own jump message if a state jumped. The warning itself is muted around the
%   batch, but lastwarn still records it. What this returns replaces the run's
%   outputs, so the saved state is read here or not at all.
s = struct('L', SetReach.getLog(), 'X0', [], 'jump', '');
if any(strcmp(so.who, 'splitX0'))
    % Each state's first sample is its value at the start time: the first row
    % of a vector state's values, the first page of a matrix state's.
    sig = so.splitX0.signals;
    v = arrayfun(@firstSample, sig, 'UniformOutput', false);
    w = cellfun(@numel, v);
    s.X0 = struct('x', vertcat(v{:}), ...
        'cont', repelem(strcmp({sig.label}, 'CSTATE'), w)');
end
if SetReach.warnedJump()
    [msg, id] = lastwarn;
    if strcmp(id, 'SetReach:stateJump')
        s.jump = msg;
    else
        s.jump = 'A continuous state jumped.';
    end
end
end

function v = firstSample(q)
if ismatrix(q.values)
    v = q.values(1, :);
else
    v = q.values(:, :, 1);
end
v = reshape(double(v), [], 1);
end

function workerSetup(folders, solver)
addpath(folders{:});
warning('off', 'SetReach:stateJump');
try Simulink.Solver.unregister(solver); catch, end
Simulink.Solver.register(solver);
end

function tf = useParallel(p)
%USEPARALLEL 'auto' is parallel only when Parallel Computing Toolbox is there
%   and a pool is already open, so the default never needs the toolbox.
have = ~isempty(ver('parallel')) && license('test', 'Distrib_Computing_Toolbox');
if isequal(p, 'auto')
    tf = have && ~isempty(gcp('nocreate'));
elseif p
    if ~have
        error('splitSetReach:noParallel', ...
            'Parallel = true needs Parallel Computing Toolbox. Use Parallel = false.');
    end
    if isempty(gcp('nocreate'))
        parpool('Processes');
    end
    tf = true;
else
    tf = false;
end
end

function f = closureFolders(solver)
%CLOSUREFOLDERS Every folder a worker needs for the solver to run. The package
%   keeps solvers, set representations and helpers in separate folders, and a
%   worker only has the path it started with.
names = [{solver, 'SetReach', 'SetRep', 'isStateJump', 'simulatingModel'}, ...
    cellfun(@(k) [upper(k(1)) k(2:end) 'Rep'], SetRep.kinds(), 'UniformOutput', false)];
f = unique(cellfun(@(s) fileparts(which(s)), names, 'UniformOutput', false));
f = f(~cellfun(@isempty, f));
end

function restoreStatics(cfg, rad)
SetReach.config(cfg);
SetReach.setRadii(rad);
end
