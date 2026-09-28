function [fig, L] = plotSetTube(varargin)
%PLOTSETTUBE Plot the set trace logged by any SetReach solver, in one call.
%
%   plotSetTube() straight after sim() draws the tube. It fetches the log
%   itself, so there is nothing to pass and nothing to have kept:
%
%       sim(mdl);
%       plotSetTube;
%
%   The log does NOT appear in your workspace after a simulation, and it never
%   will. It lives in a persistent variable inside SetReach.store, shared by
%   every SetReach and SetReachVar subclass, because a plugin solver is
%   constructed by the engine -- there is no object for you to hold and no
%   `To Workspace` block on the path. Ask the class for it:
%
%       L = SetReach.getLog();     % .t .c .S .mu .A .h .f0 .mdl
%
%   or take the second output here, [fig, L] = plotSetTube().
%
%   Two panels, shape-agnostically:
%     (a) the phase-plane tube -- the CONNECTED tube from encloseSetLog, one
%         filled region per logged interval, with the sampled sets drawn on top.
%         The sets alone are a stack of snapshots: at a coarse step consecutive
%         sets do not touch, and what the eye then reads as a tube is an
%         assertion about the times in between that nothing computed. The
%         enclosure covers each CLOSED interval [tL, tR], so the union really is
%         a tube. Both layers go through SetRep/vertices, so zonotopes,
%         ellipsoids, support sets, sensitivity sets and the hulls all draw with
%         no special-casing. With more than two states the set is projected onto
%         the chosen plane, which is EXACT for every representation here
%         (rho_{Px}(d) = rho_x(P'd));
%     (b) the time-domain band per state. With the enclosure this is the true
%         per-interval extent, sup over the closed interval, rather than a chord
%         drawn between two samples. Reset times from SetReach.getResets() are
%         marked -- the set does not survive a jump, so a band across one is not
%         a tube.
%
%   If the picture is a bare line rather than a tube, the tube has zero width and
%   the panel says which of the two causes it is. Either no initial set was
%   configured, so every logged set is the single point x0 (SetReach.setRadii, or
%   a SetIC variable in the model workspace, fixes that), or the dynamics have
%   contracted the set onto a curve, which on van der Pol they genuinely do:
%   trajectories collapse onto the limit cycle, the set becomes a needle whose
%   short axis is many orders of magnitude below its long one, and a line IS the
%   right picture. The two look identical on screen and have nothing in common,
%   so the panel distinguishes them rather than leaving the reader to guess.
%
%   Works unchanged on the variable-step solvers: t is simply non-uniform, and
%   the band is drawn against the logged t rather than an assumed grid.
%
%   plotSetTube(L, ...) plots a log you already have, rather than fetching the
%   most recent one.
%
%   Options
%     'Samples'  struct with .t and .X (nT-by-nx-by-nSamples) to overlay.
%                Draw this whenever the point is soundness rather than
%                presentation: on nonlinear dynamics the samples leave the
%                tube, and a tube plotted alone looks authoritative in a way it
%                has not earned.                              (default none)
%     'SampleColor'
%                RGB or RGBA for the sampled trajectories, used in both the
%                phase panel and the time panels. The default is a light grey:
%                forty curves stack toward opaque wherever the bundle is dense,
%                so a saturated colour covers the tube the samples exist to be
%                evidence about, and grey also leaves parula uncontested as the
%                set colouring.            (default [0.20 0.20 0.20 0.18])
%     'Every'    plot every k-th set in the phase plane. Default caps the
%                drawing at ~120 polygons, which matters on variable step where
%                a run can log thousands. The enclosure tube is NOT subsampled --
%                it is drawn for every interval, because a subsampled tube is
%                disconnected, which is the defect it exists to fix.
%                                                              (default auto)
%     'Tube'     'auto' (default) draws the encloseSetLog tube when the log
%                carries what it needs (.A, .f0, .h) and falls back to sampled
%                sets alone when it does not; 'on' demands it and reports why if
%                it cannot; 'off' draws the sampled sets only.
%     'TubeDirections'
%                directions per enclosure polygon. 48 is ~1.4 s for a
%                749-interval run against 21.5 s at SetRep's drawing default of
%                720, and at tube scale each polygon is a few pixels across, so
%                the finer outline is invisible.                    (default 48)
%     'MaxBloat' passed to encloseSetLog: subdivide any logged interval whose
%                enclosure would bloat by more than this. The default is 5% of
%                the widest sampled set, so the tube is drawn at the scale of the
%                sets rather than at the scale of the solver's longest step.
%                Inf reproduces one enclosure per step.          (default auto)
%     'Title'    figure title. Default names the shape and the run.
%     'Axes'     which two states span the phase plane          (default [1 2])
%
%   See also SETREACH, SETREP, ENCLOSESETLOG, RESAMPLESETLOG.

% ------------------------------------------------------------------- arguments
if nargin >= 1 && isstruct(varargin{1})
    L    = varargin{1};
    args = varargin(2:end);
else
    L    = SetReach.getLog();
    args = varargin;
end

p = inputParser;
p.addParameter('Samples', []);
p.addParameter('SampleColor', [0.20 0.20 0.20 0.18]);
p.addParameter('Every', []);
p.addParameter('Title', '');
p.addParameter('Axes', [1 2]);
p.addParameter('Tube', 'auto');
p.addParameter('TubeDirections', 48);
p.addParameter('MaxBloat', []);
p.parse(args{:});
opt = p.Results;

nT = numel(L.t);
if nT == 0
    % TWO causes, and the message names both.
    %
    % The second is worth stating explicitly because it looks like a plotting bug
    % and is not: a model with NO CONTINUOUS STATES never runs a continuous solver
    % at all, so selecting a set solver on it is silently inert. Measured on a
    % Clock -> Unit Delay model -- the solver object is never even CONSTRUCTED
    % (an instrumented subclass printed nothing from either start() or step()),
    % the engine simulates the discrete state correctly, and the log stays empty.
    % Nothing can be diagnosed from inside the solver in that case, which is why
    % the diagnostic lives here.
    error('plotSetTube:emptyLog', ...
        ['The set log is empty. The log is a persistent store inside the ' ...
         'SetReach class,\nnot a workspace variable, so an empty log means ' ...
         'no set solver ran -- not that the\ndata was lost. The solver clears ' ...
         'the log itself at start(), so there is nothing\nto reset by hand. ' ...
         'Two things cause this:\n' ...
         '  1. the model''s Solver is not one of the registered set solvers.\n' ...
         '     Check with get_param(mdl, ''Solver''); see registerSetSolvers.\n' ...
         '  2. the model has NO CONTINUOUS STATES, so Simulink never runs a\n' ...
         '     continuous solver and never constructs the plugin solver. A set\n' ...
         '     solver propagates the continuous state vector only: discrete\n' ...
         '     states (Unit Delay, Memory, Discrete-Time Integrator, Stateflow)\n' ...
         '     are advanced by the engine and are not in the set at all. Check\n' ...
         '     with  countContinuousStates(mdl). If that is 0, add a continuous\n' ...
         '     state or use a discrete reachability tool: this solver cannot help.']);
end
nx = numel(L.c{1});
ix = opt.Axes(1);
iy = opt.Axes(2);
if nx >= 2 && (max(opt.Axes) > nx || min(opt.Axes) < 1)
    error('plotSetTube:badAxes', ...
        '''Axes'' must index two of the %d states; got [%s].', ...
        nx, num2str(opt.Axes));
end
every = opt.Every;
if isempty(every)
    every = max(1, round(nT / 120));
end

% Interval hull of every SAMPLED set, needed before the phase plane rather than
% after it: whether the tube has any width at all decides what the phase plane
% has to explain, and that question is answered here.
ctr = zeros(nT, nx);
rad = zeros(nT, nx);
for j = 1:nT
    ctr(j,:) = L.c{j}(:)';
    if isa(L.S{j}, 'SetRep') && L.S{j}.dim() == nx
        rad(j,:) = L.S{j}.halfWidths()';
    end
end

% ------------------------------------------------------------ the tube itself
% encloseSetLog is what makes this a tube rather than a stack of snapshots. It
% needs .A, .f0 and .h, which every SetReach log carries and a hand-built or
% trimmed one may not, so 'auto' degrades to sampled sets while 'on' says what
% was missing.
E    = [];
why  = '';
want = validatestring(opt.Tube, {'auto', 'on', 'off'}, mfilename, 'Tube');
if ~strcmp(want, 'off') && nT >= 2
    % Tolerance scaled by the tube's own width, not an absolute number: the only
    % meaningful question about the bloat is how big it is COMPARED TO the set it
    % is covering. 5% is invisible on the plot and still leaves most intervals
    % unsplit. Without this the picture is dominated by the handful of long steps
    % a variable-step solver takes, whose enclosures are discs wider than the
    % trajectory -- sound, and no use to anybody looking at them.
    tol = opt.MaxBloat;
    if isempty(tol)
        tol = 0.05 * max(rad(:));
        if ~(tol > 0)
            tol = Inf;                  % point sets: nothing to be relative to
        end
    end
    try
        E = encloseSetLog(L, MaxBloat = tol);
    catch err
        why = err.message;
        E   = [];
    end
    if isempty(E) && strcmp(want, 'on')
        warning('plotSetTube:noEnclosure', ...
            ['''Tube'', ''on'' was asked for but encloseSetLog could not run on ' ...
             'this log:\n  %s\nDrawing the sampled sets only, which leaves the ' ...
             'gaps between them uncovered.'], why);
    end
end

% Name the shape from ALL the entries, not from entry 1. Reading the title off
% one entry is what made a two-run log announce itself as the first run's shape.
% Solver start() now resets the log, so a mixed log can no longer arise from
% simulating -- but one can still be passed in, and it must not be plotted as if
% it were a single run: t runs backwards at the seam, which makes the band fill()
% self-intersect, and the phase-plane colour ramp becomes meaningless.
kind = 'set';
isRep = cellfun(@(s) isa(s, 'SetRep'), L.S);
if any(isRep)
    kinds = unique(cellfun(@(s) s.kind, L.S(isRep), 'UniformOutput', false));
    kind  = strjoin(kinds, ' + ');
    if numel(kinds) > 1
        warning('plotSetTube:mixedLog', ...
            ['This log holds %d different set kinds (%s), so it spans more ' ...
             'than one simulation.\nPlotting it anyway, but the tube is a ' ...
             'concatenation of runs, not a tube. Call SetReach.resetLog() ' ...
             'and re-simulate.'], numel(kinds), kind);
    end
end
if any(diff(L.t) < 0)
    j = find(diff(L.t) < 0, 1);
    warning('plotSetTube:nonMonotonicTime', ...
        ['Logged time runs backwards at entry %d (t = %g then %g), so this ' ...
         'log spans\nmore than one simulation. The time-domain band will be ' ...
         'meaningless.'], j, L.t(j), L.t(j + 1));
end

% Taller than wide per state, because the phase plane is drawn with equal data
% units (see below) and a tube around a vdp limit cycle is about twice as tall as
% it is broad: at 480 px the equal-aspect box is limited by the tile HEIGHT and
% the panel collapses to a narrow strip with most of the tile empty.
fig = figure('Name', sprintf('%s set trace', kind), 'Color', 'w', ...
    'Position', [100 100 1180 max(480, 200 * max(nx, 1) + 180)]);
tl = tiledlayout(fig, max(nx, 1), 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

% ---------------------------------------------------------------- phase plane
axP = nexttile(tl, 1, [max(nx, 1) 1]);
hold(axP, 'on');

nEncl = 0;
if nx >= 2
    % The connected tube goes down FIRST, so the sampled sets read as samples of
    % it rather than as the whole story. One patch with a Faces matrix, not one
    % patch per interval: a variable-step run logs thousands of intervals, and
    % thousands of graphics objects is what made drawing the enclosure look
    % unaffordable. Every interval is drawn -- subsampling the tube would
    % disconnect it, which is the exact defect the enclosure exists to remove.
    if ~isempty(E)
        [Vt, Ft] = tubeFaces(E, opt.Axes, opt.TubeDirections);
        nEncl    = size(Ft, 1);
        if nEncl > 0
            patch(axP, 'Faces', Ft, 'Vertices', Vt, ...
                'FaceColor', [0.30 0.45 0.72], 'FaceAlpha', 0.16, ...
                'EdgeColor', 'none');
        end
    end

    % Colour encodes TIME, not position in the log. For a fixed-step run the two
    % coincide, but a variable-step run logs non-uniform steps, and indexing the
    % ramp by position would then paint equal colour changes over unequal elapsed
    % time. The colorbar below is labelled in t, so this has to hold for both.
    NCOL  = 256;
    cmap  = parula(NCOL);
    tRamp = L.t(:);
    tSpan = tRamp(end) - tRamp(1);
    if tSpan > 0
        cIdx = 1 + round((NCOL - 1) * (tRamp - tRamp(1)) / tSpan);
        cIdx = min(max(cIdx, 1), NCOL);
    else
        cIdx = ones(nT, 1);   % a single instant carries no ramp
    end
    drawn = 0;
    for j = 1:every:nT
        V = projectedVertices(L.S{j}, L.c{j}, opt.Axes);
        if isempty(V)
            continue
        end
        drawn = drawn + 1;
        if size(V, 2) >= 3
            patch(axP, 'XData', V(1,:), 'YData', V(2,:), ...
                'FaceColor', cmap(cIdx(j),:), 'FaceAlpha', 0.28, ...
                'EdgeColor', cmap(cIdx(j),:), 'LineWidth', 0.5);
        else
            plot(axP, V(1,:), V(2,:), '-', 'Color', cmap(cIdx(j),:), 'LineWidth', 1.2);
        end
    end
    C = cell2mat(cellfun(@(c) c(opt.Axes), L.c(:)', 'UniformOutput', false));
    plot(axP, C(1,:), C(2,:), 'k-', 'LineWidth', 1.6);
    plot(axP, C(1,1), C(2,1), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 6);
    if drawn == 0
        text(axP, 0.5, 0.5, 'no set could be drawn on this plane', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
    end
    annotateWidth(axP, L, rad, opt.Axes);
else
    text(axP, 0.5, 0.5, 'phase plane needs nx >= 2', 'Units', 'normalized', ...
        'HorizontalAlignment', 'center');
end

if ~isempty(opt.Samples)
    S = opt.Samples;
    for k = 1:size(S.X, 3)
        plot(axP, S.X(:, ix, k), S.X(:, iy, k), '-', ...
            'Color', opt.SampleColor, 'LineWidth', 0.4);
    end
end

xlabel(axP, sprintf('x_%d', ix));
ylabel(axP, sprintf('x_%d', iy));
% Two lines, not one. This tile is the narrow one -- axis equal gives it roughly a
% third of the figure width -- and a one-line title long enough to carry the counts
% overflows the tile and runs to the figure edge. A cell array wraps it instead.
if nEncl > 0
    title(axP, {'phase plane', sprintf('tube over %d intervals, %d of %d sets shown', ...
        nEncl, numel(1:every:nT), nT)});
else
    title(axP, {'phase plane', sprintf('%d of %d sets (samples only, no tube)', ...
        numel(1:every:nT), nT)});
end
grid(axP, 'on'); box(axP, 'on');
% Equal data units, not a filled tile. The panel exists to show the SHAPE of the
% set; unequal axes render a circular ellipsoid as a squashed one, which is the
% one thing this plot must not do.
axis(axP, 'tight'); axis(axP, 'equal');

% Legend for the ramp. Without it the reader has no way to tell that the pale blues
% and yellows are late sets rather than a second kind of object, given that the
% enclosure is a third blue again. clim is set in t, so the ticks read as time.
%
% southoutside, not the default east: axis equal has already width-limited this tile,
% and a vertical bar would take width from the one panel that can least spare it. The
% colormap is set per-axes so the bar legends the phase panel alone -- the enclosure
% and the samples carry explicit RGB and are not colormapped, so neither is disturbed.
if nx >= 2 && nT > 1 && L.t(end) > L.t(1)
    colormap(axP, parula(256));
    clim(axP, [L.t(1) L.t(end)]);
    cb = colorbar(axP, 'southoutside');
    cb.Label.String = 'set colour = simulation time t';
    cb.FontSize = 8;
end

% ------------------------------------------------------------ time-domain band
% ctr and rad were computed above, before the phase plane needed them.

% Exactly one entry, at t = 0, means nothing jumped. Anything later is a real
% discontinuity, and the set is NOT carried across it.
jumps = [];
try
    R = SetReach.getResets();
    jumps = R.t(R.t > L.t(1) & R.t < L.t(end));
catch
    % getResets is a convenience; its absence must not break the plot.
end

for i = 1:nx
    ax = nexttile(tl, 2*i);
    hold(ax, 'on');
    tt = L.t(:);
    if nEncl > 0
        % One box per interval, spanning the interval in t and the enclosure's
        % true extent in x_i. Asymmetric on purpose: HullRep/halfWidths widens to
        % the larger side to stay a symmetric box, and the band has no reason to
        % pay for that, so the two supports are taken separately. The chord fill
        % below is what this replaces, and the chord asserted a straight edge
        % between samples that nothing had computed.
        [Vb, Fb] = bandFaces(E, i);
        patch(ax, 'Faces', Fb, 'Vertices', Vb, 'FaceColor', [0.2 0.4 0.8], ...
            'FaceAlpha', 0.22, 'EdgeColor', 'none');
    else
        fill(ax, [tt; flipud(tt)], ...
            [ctr(:,i) - rad(:,i); flipud(ctr(:,i) + rad(:,i))], ...
            [0.2 0.4 0.8], 'FaceAlpha', 0.22, 'EdgeColor', 'none');
    end
    plot(ax, tt, ctr(:,i), 'b-', 'LineWidth', 1.4);
    for k = 1:numel(jumps)
        xline(ax, jumps(k), 'r--', 'LineWidth', 1.0);
    end
    if ~isempty(opt.Samples)
        S = opt.Samples;
        for k = 1:size(S.X, 3)
            plot(ax, S.t, S.X(:, i, k), '-', ...
                'Color', opt.SampleColor, 'LineWidth', 0.4);
        end
    end
    ylabel(ax, sprintf('x_%d', i));
    grid(ax, 'on'); box(ax, 'on');
    if i == 1
        if nEncl > 0
            base = 'reachable interval per state, over the closed intervals';
        else
            base = 'interval hull of the sampled sets, per state';
        end
        if isempty(jumps)
            title(ax, base);
        else
            title(ax, sprintf(['%s ' ...
                '(%d reset(s) dashed -- the set does not cross them)'], ...
                base, numel(jumps)));
        end
    end
    if i == nx, xlabel(ax, 't'); end
end

ttl = opt.Title;
if isempty(ttl)
    % Name the SOURCE MODEL, not just the shape. The log is a persistent store that
    % outlives any one run, so a figure drawn from it is only as trustworthy as the
    % reader's memory of what ran last. Two ways that goes wrong: redrawing after a
    % different model has run, and a model with NO continuous states leaving the
    % previous run's log in place because its start() never fired (see
    % SetReach.stampModel). A title naming the model makes both visible.
    src = '';
    if isfield(L, 'mdl') && ~isempty(L.mdl)
        src = sprintf('%s: ', L.mdl);
    end
    ttl = sprintf('%s%s tube -- %d sets, t = %g .. %g', src, kind, nT, ...
        L.t(1), L.t(end));
end
% INTERPRETER OFF, because this title names a model and Simulink model names are full
% of underscores: under the default TeX interpreter sldemo_metro_basic renders as
% "sldemo(m)etro(b)asic", which is not the name of any model. The axis labels keep TeX
% on purpose -- they are state indices, so x_1 is a subscript because it means one.
title(tl, ttl, 'FontWeight', 'bold', 'Interpreter', 'none');
end

% ------------------------------------------------------------------- helpers

function V = projectedVertices(rep, c, idx)
% The 2-D shadow of one logged set. Projection first, then the boundary, so
% every representation draws through its own exact construction where it has
% one. Returns [] for anything that cannot be drawn on a plane.
% Always project, never special-case nx == 2: 'Axes', [2 1] must swap the set as
% well as the centre, and projecting is how that happens.
V = [];
if ~isa(rep, 'SetRep')
    return
end
V = rep.project(idx).vertices(c(idx));
end

function [Vt, Ft] = tubeFaces(E, idx, nDir)
% One face per interval, as a single Faces/Vertices pair. Every enclosure is a
% HullRep, which does not override SetRep/vertices, so each polygon comes back
% with exactly nDir vertices and the Faces matrix is a plain reshape -- no
% padding with NaN and no per-interval graphics object.
nDir = max(3, round(nDir));
nInt = numel(E.tL);
Vt   = nan(nInt * nDir, 2);
keep = false(nInt, 1);
for k = 1:nInt
    if ~isa(E.Om{k}, 'SetRep')
        continue
    end
    V = E.Om{k}.project(idx).vertices(E.c{k}(idx), nDir);
    if size(V, 2) ~= nDir || ~all(isfinite(V(:)))
        continue
    end
    Vt((k-1)*nDir + (1:nDir), :) = V';
    keep(k) = true;
end
Ft = reshape(1:(nInt * nDir), nDir, nInt)';
Ft = Ft(keep, :);
end

function [Vb, Fb] = bandFaces(E, i)
% The time-domain counterpart: one axis-aligned box per interval, [tL tR] by
% [lower upper] for state i. The two bounds are separate support evaluations
% because the enclosure is not symmetric about its centre.
nInt = numel(E.tL);
Vb   = nan(4 * nInt, 2);
keep = false(nInt, 1);
for k = 1:nInt
    Om = E.Om{k};
    if ~isa(Om, 'SetRep') || i > Om.dim()
        continue
    end
    e    = zeros(Om.dim(), 1);
    e(i) = 1;
    ci   = E.c{k}(i);
    hi   = ci + Om.support(e);
    lo   = ci - Om.support(-e);
    if ~isfinite(hi) || ~isfinite(lo)
        continue
    end
    tL = E.tL(k);
    tR = E.tR(k);
    Vb((k-1)*4 + (1:4), :) = [tL lo; tR lo; tR hi; tL hi];
    keep(k) = true;
end
Fb = reshape(1:(4 * nInt), 4, nInt)';
Fb = Fb(keep, :);
end

function annotateWidth(axP, L, rad, idx)
% Say why the tube looks like a line, when it does. Two causes produce the same
% picture and want opposite responses, so guessing between them is exactly what
% this saves the reader.
%
% The note goes in the SUBTITLE, not in the axes. Inside the axes it is clipped:
% the phase plane is drawn with equal data units, so on a tall narrow set the
% panel is a narrow strip and a three-line note does not fit across it. The
% subtitle is laid out against the tile rather than the data box.
if isempty(rad)
    return
end
if max(rad(:)) <= 0
    % Nothing was ever wider than a point. resolveRadii warns at simulation time
    % now, but a log can be plotted long after that scrolled away, and the plot
    % is where the question gets asked.
    subtitle(axP, ...
        ["every set is a POINT: no initial set was configured, so this is only " + ...
         "the centre trajectory"; ...
         "fix: SetReach.setRadii(0.1) or a SetIC in the model workspace, then re-simulate"], ...
        'FontSize', 8, 'Color', [0.6 0 0], 'FontWeight', 'bold');
    return
end

% Needle test on the drawn plane, from the polygons themselves rather than from
% any one representation's payload, so it reads the same for all of them.
nT = numel(L.S);
js = unique(round(linspace(1, nT, min(nT, 40))));
ar = nan(size(js));
for q = 1:numel(js)
    V = projectedVertices(L.S{js(q)}, L.c{js(q)}, idx);
    if isempty(V) || size(V, 2) < 3
        continue
    end
    s = svd(V - mean(V, 2));
    if s(1) > 0
        ar(q) = s(end) / s(1);
    end
end
ar = ar(isfinite(ar));
if isempty(ar) || median(ar) > 1e-3
    return
end
subtitle(axP, ...
    [sprintf("sets are NEEDLES: short/long axis ~ %.1e (median), so a line " + ...
             "IS the right picture", median(ar)); ...
     "the dynamics contracted the set transversally; the time panels show the " + ...
     "width that remains"], ...
    'FontSize', 8, 'Color', [0 0.35 0]);
end
