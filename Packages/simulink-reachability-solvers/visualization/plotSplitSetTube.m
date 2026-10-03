function [fig, draw, K] = plotSplitSetTube(R, varargin)
%PLOTSPLITSETTUBE Unsplit against split, in the phase plane and in time.
%
%   plotSplitSetTube(R, 'Samples', S) where R = splitSetReach(...) draws a 2-by-2
%   comparison. The rows are the unsplit tube (the root piece alone) and the
%   split tube (the union of the leaves); the columns are the phase plane and
%   the tube against time, one axes per state, so the two bands sit one above
%   the other on the same time axis. The same true trajectories are drawn in
%   every panel, so each tube is judged against the same truth.
%
%   WHAT IS DRAWN. The sets are the pieces' own sets at the logged steps, not a
%   between-step enclosure; at the fine steps splitSetReach needs, consecutive
%   sets overlap. In the phase plane a faint set is drawn every few steps, the
%   set at the current time over it, and the true states at those same times as
%   dots. In time, each piece is one band, c +- the sum of its generators'
%   magnitudes in that state, so the split band is the union of the pieces'
%   intervals, exactly. The bands are rasterised, a pixel row lit when an
%   interval covers it and every interval lighting at least one, because 955
%   bands as patches take 12 s a frame to print. The unsplit tube leaves the
%   truth's frame by orders of magnitude, so every axis is framed by the truth
%   and the unsplit sets are clipped to it, which the panel title says.
%
%   Pieces are coloured by depth, the number of times each was halved, and the
%   deepest are drawn last. A shallower colour then shows only where a piece
%   accepted early reaches outside every deeper one, which is where the union is
%   loose. Drawn the other way round, 23 shallow pieces of 955 painted half the
%   time row.
%
%   [fig, draw, K] = plotSplitSetTube(...) also returns draw(k), which redraws
%   every panel as of logged step k, 1 <= k <= K, for animating the run. Nothing
%   on screen at step k depends on t > t(k).
%
%   Options
%     'Samples'  struct with .t and .X, nT-by-nx-by-nSamples. Pass it: without
%                the truth neither tube can be judged.            (default none)
%     'Axes'     the two states of the phase plane.               (default [1 2])
%     'States'   the states drawn against time.            (default: as 'Axes')
%     'Every'    a faint set every this many steps.          (default ~20 sets)
%     'Upto'     draw as of this time.                   (default: the end time)
%     'Paper'    true sizes the figure in inches for a two-column page, with
%                small fonts, thin lines and no figure title.     (default false)
%     'Size'     [W H], pixels, or inches when Paper.    (default [1400 820] or
%                                                                  [7 3.6])
%     'Title'    figure title, ignored when Paper.                 (default '')
%     'Visible'  'on' or 'off'.                                    (default 'on')
%
%   See also SPLITSETREACH, PLOTSETTUBE, VDPSPLITTUBE.

p = inputParser;
p.addParameter('Samples', []);
p.addParameter('Axes', [1 2]);
p.addParameter('States', []);
p.addParameter('Every', []);
p.addParameter('Upto', []);
p.addParameter('Paper', false);
p.addParameter('Size', []);
p.addParameter('Title', '');
p.addParameter('Visible', 'on');
p.parse(varargin{:});
opt = p.Results;
if isempty(opt.States), opt.States = opt.Axes; end
if isempty(opt.Size)
    if opt.Paper, opt.Size = [7 3.6]; else, opt.Size = [1400 820]; end
end

t = R.t(:)';
K = numel(t);
if isempty(opt.Every), opt.Every = max(1, round((K - 1) / 20)); end
D.t = t;
D.trail = unique([1:opt.Every:K, K]);
D.ax2 = opt.Axes;
D.states = opt.States;

% Leaves, deepest last, and their colours.
P = R.pieces(R.leaves);
[~, order] = sort([P.depth], 'ascend');
P = P(order);
depth = [P.depth];
cmap = depthColours(max(depth));      % no colourbar: the caption carries it
D.pieces = {R.pieces(1), P};
D.colours = {repmat([0.2 0.4 0.8], 1, 1), cmap(depth + 1, :)};

% The truth, on the tube's own grid.
S = opt.Samples;
D.hasTruth = ~isempty(S);
if D.hasTruth
    j = zeros(1, K);
    for k = 1:K
        [~, j(k)] = min(abs(S.t(:) - t(k)));
    end
    D.X = S.X(j, :, :);                % K-by-nx-by-N
end

[phaseLim, timeLim] = frames(D);
if opt.Paper, ny = 900; else, ny = 600; end

% Time bands, per column and state, as an RGB image with an alpha mask.
for c = 1:2
    Q = D.pieces{c};
    for i = 1:numel(D.states)
        s = D.states(i);
        lo = zeros(numel(Q), K);
        hi = lo;
        for q = 1:numel(Q)
            r = squeeze(sum(abs(Q(q).G(s, :, :)), 2))';
            lo(q, :) = Q(q).c(s, :) - r;
            hi(q, :) = Q(q).c(s, :) + r;
        end
        D.band{c, i} = bandImage(t, lo, hi, timeLim(i, :), ny, D.colours{c});
    end
end

% Faint sets: every column of XData is one piece at one trail time, ordered by
% time, so as of step k the first nTrail(k) columns are drawn.
for c = 1:2
    Q = D.pieces{c};
    Vx = {}; Vy = {}; C = {};
    for k = D.trail
        [vx, vy] = outlines(Q, D.ax2, k);
        Vx{end+1} = vx; Vy{end+1} = vy; C{end+1} = D.colours{c}; %#ok<AGROW>
    end
    nv = max(cellfun(@(v) size(v, 1), Vx));
    Vx = cellfun(@(v) padRows(v, nv), Vx, 'UniformOutput', false);
    Vy = cellfun(@(v) padRows(v, nv), Vy, 'UniformOutput', false);
    D.trailX{c} = [Vx{:}];
    D.trailY{c} = [Vy{:}];
    % Faint by tint rather than by transparency: opaque, they print in about a
    % tenth of the time.
    D.trailC{c} = 0.45 * vertcat(C{:}) + 0.55;
    D.perTrail(c) = numel(Q);
end

% ------------------------------------------------------------------ layout
if opt.Paper
    fs = 7; lw = 0.4; ms = 2.5;
    fig = figure('Color', 'w', 'Units', 'inches', 'Position', [1 1 opt.Size], ...
        'Visible', opt.Visible);
else
    fs = 11; lw = 0.6; ms = 5;
    fig = figure('Color', 'w', 'Position', [60 40 opt.Size], 'Visible', opt.Visible);
end
D.ms = ms;
% The phase plane is drawn at equal scale, so its width follows the truth's
% aspect ratio. A tall frame, such as a full vdp loop, in a third of the figure
% leaves a blank strip beside it, so the phase plane gets about the share of the
% width it fills, never more than a third, and the time panels the rest. The
% share is the axes' width at that aspect, a row being about 0.44 of the
% figure's height, plus a margin for the tick labels.
ar = diff(phaseLim(1:2)) / diff(phaseLim(3:4));
nCol = min(8, max(3, round(1 / (ar * 0.44 * opt.Size(2) / opt.Size(1) + 0.05))));
tl = tiledlayout(fig, 2, nCol, 'TileSpacing', 'compact', 'Padding', 'compact');
nSt = numel(D.states);
dr = sprintf('depth %d', min(depth));
if max(depth) > min(depth), dr = sprintf('depth %d to %d', min(depth), max(depth)); end
if opt.Paper
    heads = {'(a) unsplit', '(b) unsplit, clipped to the frame of the truth'; ...
             sprintf('(c) split: %d pieces', numel(P)), ...
             sprintf('(d) split: %s, Tol = %g', dr, R.tol)};
else
    % The depth range goes over the wide time panels: a phase-plane title is
    % centred on axes that can be narrow, and a long one ran to the figure edge.
    heads = {'unsplit: one linearisation', 'unsplit, clipped to the frame of the truth'; ...
             sprintf('split: %d pieces', numel(P)), ...
             sprintf('split: %s, every piece within Tol = %g of its parent', dr, R.tol)};
end
emptyPoly = {'Vertices', [nan nan], 'Faces', 1, 'FaceVertexCData', [1 1 1]};
for c = 1:2
    a = nexttile(tl, nCol * (c - 1) + 1);
    hold(a, 'on');
    H.phase(c) = a;
    H.pLine(c) = plot(a, nan, nan, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', lw / 2);
    H.trail(c) = patch(a, emptyPoly{:}, 'FaceColor', 'flat', 'EdgeColor', 'flat', ...
        'LineWidth', lw / 2);
    H.now(c) = patch(a, emptyPoly{:}, 'FaceColor', 'flat', 'EdgeColor', 'flat', ...
        'LineWidth', lw);
    % The truth goes on top, so a true state outside a set is never hidden by it.
    H.pDots(c) = plot(a, nan, nan, '.', 'Color', [0.25 0.25 0.25], 'MarkerSize', 0.6 * ms);
    H.pNow(c) = plot(a, nan, nan, 'k.', 'MarkerSize', 1.4 * ms);
    axis(a, 'equal');
    axis(a, phaseLim);
    title(a, heads{c, 1}, 'FontWeight', 'normal');
    xlabel(a, sprintf('x_%d', D.ax2(1)));
    ylabel(a, sprintf('x_%d', D.ax2(2)));

    % Against time: one inner layout across the other tiles, one axes per
    % state, so the unsplit and split bands sit one above the other on the
    % same t axis.
    inner = tiledlayout(tl, nSt, 1, 'TileSpacing', 'tight', 'Padding', 'tight');
    inner.Layout.Tile = nCol * (c - 1) + 2;
    inner.Layout.TileSpan = [1 nCol - 1];
    for i = 1:nSt
        b = nexttile(inner);
        hold(b, 'on');
        H.time(c, i) = b;
        Bi = D.band{c, i};
        H.band(c, i) = image(b, 'XData', Bi.t([1 end]), 'YData', Bi.y([1 end]), ...
            'CData', Bi.rgb, 'AlphaData', zeros(size(Bi.alpha)));
        H.tLine(c, i) = plot(b, nan, nan, '-', 'Color', [0 0 0 0.2], 'LineWidth', lw / 2);
        xlim(b, [t(1) t(end)]);
        ylim(b, timeLim(i, :));
        ylabel(b, sprintf('x_%d', D.states(i)));
        if i == 1
            title(b, heads{c, 2}, 'FontWeight', 'normal');
        end
        if i < nSt
            b.XTickLabel = {};
        else
            xlabel(b, 't');
        end
    end
end
for a = [H.phase(:); H.time(:)]'
    a.FontSize = fs;
    grid(a, 'on');
    a.Layer = 'top';
end
if ~opt.Paper && ~isempty(opt.Title)
    H.title = title(tl, opt.Title, 'FontWeight', 'bold', 'FontSize', fs + 1);
else
    H.title = [];
end
D.titleText = opt.Title;
D.paper = opt.Paper;

draw = @(k) drawAt(H, D, k);
if isempty(opt.Upto)
    draw(K);
else
    [~, k] = min(abs(t - opt.Upto));
    draw(k);
end
end

% =====================================================================

function drawAt(H, D, k)
%DRAWAT Every panel as of logged step k.
t = D.t;
nT = sum(D.trail <= k);
for c = 1:2
    n = nT * D.perTrail(c);
    setPolys(H.trail(c), D.trailX{c}(:, 1:n), D.trailY{c}(:, 1:n), D.trailC{c}(1:n, :));
    [vx, vy] = outlines(D.pieces{c}, D.ax2, k);
    setPolys(H.now(c), vx, vy, D.colours{c});

    for i = 1:numel(D.states)
        A = D.band{c, i}.alpha;
        A(:, D.band{c, i}.t > t(k) + 1e-12) = 0;
        H.band(c, i).AlphaData = A;
    end

    if D.hasTruth
        X = D.X;
        N = size(X, 3);
        a1 = D.ax2(1); a2 = D.ax2(2);
        set(H.pLine(c), 'XData', nanJoin(reshape(X(1:k, a1, :), k, [])), ...
            'YData', nanJoin(reshape(X(1:k, a2, :), k, [])));
        tk = D.trail(D.trail <= k);
        set(H.pDots(c), 'XData', reshape(X(tk, a1, :), 1, []), ...
            'YData', reshape(X(tk, a2, :), 1, []));
        set(H.pNow(c), 'XData', reshape(X(k, a1, :), 1, []), ...
            'YData', reshape(X(k, a2, :), 1, []));
        for i = 1:numel(D.states)
            s = D.states(i);
            set(H.tLine(c, i), 'XData', nanJoin(repmat(t(1:k)', 1, N)), ...
                'YData', nanJoin(reshape(X(1:k, s, :), k, [])));
        end
    end
end
if ~isempty(H.title)
    if isempty(D.titleText)
        H.title.String = sprintf('t = %.2f', t(k));
    else
        H.title.String = sprintf('%s   |   t = %.2f', D.titleText, t(k));
    end
end
end

function B = bandImage(t, lo, hi, ylim, ny, colours)
%BANDIMAGE Rasterise the union of the intervals [lo, hi], piece-by-K, into ny rows.
%   Each pixel takes the colour of the LAST piece covering it, matching the draw
%   order of the phase plane. An interval thinner than a pixel still lights the
%   row nearest its middle, so no piece vanishes for being thin. The bounds are
%   interpolated linearly onto about 1600 columns, as a patch through the
%   samples would be, so a fast edge does not print as a staircase.
ups = max(1, ceil(1600 / max(1, numel(t) - 1)));
tc = interp1(1:numel(t), t, linspace(1, numel(t), (numel(t) - 1) * ups + 1));
lo = interp1(t, lo', tc)';
hi = interp1(t, hi', tc)';
if size(lo, 2) ~= numel(tc)        % a single piece comes back as a row
    lo = lo'; hi = hi';
end
[np, K] = size(lo);
y = linspace(ylim(1), ylim(2), ny);
dy = y(2) - y(1);
own = zeros(ny, K);
for q = 1:np
    r0 = ceil((lo(q, :) - y(1)) / dy) + 1;
    r1 = floor((hi(q, :) - y(1)) / dy) + 1;
    mid = round(((lo(q, :) + hi(q, :)) / 2 - y(1)) / dy) + 1;
    thin = r1 < r0;
    r0(thin) = mid(thin);
    r1(thin) = mid(thin);
    r0 = max(r0, 1);
    r1 = min(r1, ny);
    for k = find(r0 <= r1)
        own(r0(k):r1(k), k) = q;
    end
end
lit = own > 0;
B.t = tc;
B.y = y;
B.alpha = double(lit);
B.rgb = ones(ny, K, 3);
for ch = 1:3
    col = colours(:, ch);
    plane = ones(ny, K);
    plane(lit) = col(own(lit));
    B.rgb(:, :, ch) = plane;
end
end

function setPolys(h, vx, vy, col)
%SETPOLYS One polygon per column of vx, vy, coloured by the rows of col.
%   Vertices and Faces with a colour per vertex, rather than XData and CData,
%   because 'flat' edges need per-vertex colours.
[nv, np] = size(vx);
if np == 0
    set(h, 'Vertices', [nan nan], 'Faces', 1, 'FaceVertexCData', [1 1 1]);
    return
end
set(h, 'Vertices', [vx(:), vy(:)], 'Faces', reshape(1:nv * np, nv, np)', ...
    'FaceVertexCData', repelem(col, nv, 1));
end

function [vx, vy] = outlines(Q, ax2, k)
%OUTLINES Vertex columns of every piece of Q at step k, for one patch.
V = cell(1, numel(Q));
for j = 1:numel(Q)
    V{j} = zonotopeVertices(Q(j).c(ax2, k), Q(j).G(ax2, :, k));
end
nv = max(2, max(cellfun(@(v) size(v, 2), V)));
vx = zeros(nv, numel(Q));
vy = vx;
for j = 1:numel(Q)
    % Repeat the last vertex rather than pad with NaN, which patch would read
    % as a hole in the face.
    v = [V{j}, repmat(V{j}(:, end), 1, nv - size(V{j}, 2))];
    vx(:, j) = v(1, :)';
    vy(:, j) = v(2, :)';
end
end

function v = padRows(v, nv)
v = [v; repmat(v(end, :), nv - size(v, 1), 1)];
end

function z = nanJoin(Y)
%NANJOIN Columns of Y as one NaN-separated row, for a single line object.
z = reshape([Y; nan(1, size(Y, 2))], 1, []);
end

function [phaseLim, timeLim] = frames(D)
%FRAMES Axis limits from the truth, or from the split tube without one.
if D.hasTruth
    X = D.X;
else
    % Without samples the split tube's centres stand in for the truth.
    Q = D.pieces{2};
    X = permute(cat(3, Q.c), [2 1 3]);
end
pts = reshape(permute(X(:, D.ax2, :), [2 1 3]), 2, []);
lo = min(pts, [], 2);
hi = max(pts, [], 2);
pad = 0.06 * max(hi - lo);
phaseLim = [lo(1) - pad, hi(1) + pad, lo(2) - pad, hi(2) + pad];
timeLim = zeros(numel(D.states), 2);
for i = 1:numel(D.states)
    x = X(:, D.states(i), :);
    r = max(x(:)) - min(x(:));
    timeLim(i, :) = [min(x(:)) - 0.12 * r, max(x(:)) + 0.12 * r];
end
end

function c = depthColours(dmax)
% parula without its darkest and lightest ends.
c = parula(dmax + 3);
c = c(2:dmax + 2, :);
end
