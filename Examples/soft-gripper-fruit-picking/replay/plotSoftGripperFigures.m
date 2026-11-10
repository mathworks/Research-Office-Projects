function plotSoftGripperFigures(objId, imgDir)
% PLOTSOFTGRIPPERFIGURES  The README figures: finger vs the hardware data, and the timeline of one pick.
%
%   PLOTSOFTGRIPPERFIGURES()          apple pick (objId 3), writes images/finger_validation.png, images/apple_timeline.png
%   PLOTSOFTGRIPPERFIGURES(4)         tomato pick timeline
%   PLOTSOFTGRIPPERFIGURES(3, dir)    write the PNGs to another folder
%
%   Reads saved results only; it simulates nothing.
%   - Finger panel: the cubic PRB identification (fit_zhu_finger_prb, Optimization Toolbox) against Zhu Table 1.
%     The Simscape finger reproduces this fit at all 29 points (T-F2, RMS difference 0.0000 N).
%   - Hold-pressure panel: T-I2 saved runs (results/TI2p_<id>.mat, check_softgripper_hold evaluate-only). Skipped
%     if those files are not present.
%   - Timeline: results/pick_<objId>.mat (runSoftGripperPick).

arguments
    objId (1,1) double = 3
    imgDir (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'images')
end
ink = [0.04 0.04 0.04]; ink2 = [0.32 0.32 0.31]; grid = [0.88 0.88 0.87];
ser = [0.165 0.471 0.839; 0.922 0.408 0.204; 0.106 0.686 0.478; 0.929 0.631 0];   % categorical slots 1-4
blue = @(k, n) [0.80 0.88 0.97] + (k - 1) / max(n - 1, 1) * ([0.06 0.20 0.45] - [0.80 0.88 0.97]);  % sequential
style = @(ax) set(ax, 'Box', 'off', 'TickDir', 'out', 'XColor', ink2, 'YColor', ink2, 'GridColor', grid, ...
    'GridAlpha', 1, 'XGrid', 'on', 'YGrid', 'on', 'FontSize', 10, 'LineWidth', 0.75);
if ~isfolder(imgDir), mkdir(imgDir); end

% ---- 1. finger validation ---------------------------------------------------------------------------------------
fit = fit_zhu_finger_prb(8, "cubic", Quiet=true);
f1 = figure('Color', 'w', 'Position', [100 100 1200 400]);
tl = tiledlayout(f1, 1, 3, 'TileSpacing', 'loose', 'Padding', 'compact');
ax = nexttile(tl); hold(ax, 'on'); style(ax);
pk = unique(fit.p); n = numel(pk);
for k = 1:n
    i = fit.p == pk(k); a = rad2deg(fit.ang(i)); [a, o] = sort(a);
    m = fit.meas(i); pr = fit.pred(i); c = blue(k, n);
    plot(ax, a, pr(o), '-', 'Color', c, 'LineWidth', 2);
    plot(ax, a, m(o), 'o', 'MarkerSize', 7, 'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', 'LineWidth', 1);
    text(ax, a(1) - 2, pr(o(1)), sprintf('%d kPa', pk(k) / 1e3), 'HorizontalAlignment', 'right', 'Color', ink2, 'FontSize', 9);
end
xlim(ax, [-16 95]); ylim(ax, [-0.08 1.5]); xticks(ax, 0:15:90); xlabel(ax, 'Bend angle held by the gauge (deg)', 'Color', ink);
ylabel(ax, 'Fingertip force (N)', 'Color', ink);
title(ax, sprintf('Tip force: model (lines) vs Zhu (dots), RMS %.3f N', fit.rmsErr), 'Color', ink, 'FontWeight', 'normal', 'FontSize', 10);

ax = nexttile(tl); style(ax);
b = bar(ax, [40 50 60], [rad2deg(fit.freeBend40_50_60)' [75 90 110]'], 0.8, 'EdgeColor', 'w', 'LineWidth', 2);
b(1).FaceColor = ser(1, :); b(2).FaceColor = [0.70 0.70 0.68];
legend(ax, {'Model', 'Zhu 2022'}, 'Location', 'northwest', 'Box', 'off', 'TextColor', ink);
xlabel(ax, 'Pressure (kPa)', 'Color', ink); ylabel(ax, 'Free bend (deg)', 'Color', ink); ylim(ax, [0 130]);
title(ax, 'Free bend, limit \pm10\circ', 'Color', ink, 'FontWeight', 'normal', 'FontSize', 10);

ax = nexttile(tl); style(ax);
here = fileparts(fileparts(mfilename('fullpath'))); ids = [3 4 5 6];
if all(arrayfun(@(k) isfile(fullfile(here, 'results', sprintf('TI2p_%d.mat', k))), ids))
    H = check_softgripper_hold(ids, [], true, "pick");
    b = bar(ax, categorical(cellstr([H.name]), cellstr([H.name])), [[H.pHold]' [H.zhu]'] / 1e3, 0.8, 'EdgeColor', 'w', 'LineWidth', 2);
    b(1).FaceColor = ser(1, :); b(2).FaceColor = [0.70 0.70 0.68];
    legend(ax, {'Model', 'Zhu 2022'}, 'Location', 'northwest', 'Box', 'off', 'TextColor', ink);
    ylabel(ax, 'Minimum hold pressure (kPa)', 'Color', ink); ylim(ax, [0 75]);
    title(ax, 'Hold pressure per fruit, limit \pm10 kPa', 'Color', ink, 'FontWeight', 'normal', 'FontSize', 10);
else
    axis(ax, 'off'); text(ax, 0.5, 0.5, 'Run check\_softgripper\_hold first', 'HorizontalAlignment', 'center', 'Color', ink2);
end
exportgraphics(f1, fullfile(imgDir, 'finger_validation.png'), 'Resolution', 150); close(f1);

% ---- 2. pick timeline -------------------------------------------------------------------------------------------
S = load(fullfile(here, 'results', sprintf('pick_%d.mat', objId)), 'R', 'out');
ls = S.out.logsout; P = softgripperPickParams(objId);
sig = @(nm) deal(ls.get(nm).Values.Time, squeeze(ls.get(nm).Values.Data));
[tc, pc] = sig('pGripCmd'); [tp, pm] = sig('pGripMeas'); [tb, bd] = sig('bendMeas'); [tz, zd] = sig('zTruth');
if size(bd, 1) ~= numel(tb), bd = bd.'; end
if size(zd, 1) ~= numel(tz), zd = zd.'; end
ph = split(S.R.timeline, "@"); ph = reshape(ph, [], 2); phName = ph(:, 1); phT = double(ph(:, 2));
phEnd = [phT(2:end); S.R.tEnd];
f2 = figure('Color', 'w', 'Position', [100 100 1000 760]);
tl = tiledlayout(f2, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
fC = zd(:, 1:4); zObj = zd(:, 7);
rows = {
    @(ax) plot(ax, tc, pc / 1e3, '--', tp, pm / 1e3, '-'), 'Pressure (kPa)', {'Command', 'Manifold'}
    @(ax) plot(ax, tb, rad2deg(bd)), 'Bend sensor (deg)', {'Finger 1', 'Finger 2', 'Finger 3', 'Finger 4'}
    @(ax) plot(ax, tz, fC), 'Contact force (N)', {'Finger 1', 'Finger 2', 'Finger 3', 'Finger 4'}
    @(ax) plot(ax, tz, zObj * 1e3), 'Fruit height (mm)', {}};
for r = 1:size(rows, 1)
    ax = nexttile(tl); hold(ax, 'on'); style(ax);
    h = rows{r, 1}(ax);
    for k = 1:numel(h), h(k).Color = ser(k, :); h(k).LineWidth = 1.5; end
    if r == 3, yline(ax, P.obj.F_Bruise, ':', sprintf('bruise limit %.0f N', P.obj.F_Bruise), 'Color', ink2, 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off'); end
    ylabel(ax, rows{r, 2}, 'Color', ink); xlim(ax, [0 S.R.tEnd]);
    yl = ylim(ax); if r == 3, yl(2) = max(yl(2), 1.15 * P.obj.F_Bruise); end; ylim(ax, yl);
    for k = 2:2:numel(phName)                     % shade every other phase, behind the data
        q = patch(ax, [phT(k) phEnd(k) phEnd(k) phT(k)], yl([1 1 2 2]), [0.96 0.96 0.95], 'EdgeColor', 'none', 'HandleVisibility', 'off');
        uistack(q, 'bottom');
    end
    if ~isempty(rows{r, 3}), legend(ax, h, rows{r, 3}, 'Location', 'eastoutside', 'Box', 'off', 'TextColor', ink); end
    if r == 1
        for k = 1:numel(phName)
            text(ax, (phT(k) + phEnd(k)) / 2, yl(2), phName(k), 'Rotation', 90, 'HorizontalAlignment', 'right', ...
                'VerticalAlignment', 'middle', 'FontSize', 8, 'Color', ink2);
        end
    end
    if r < size(rows, 1), ax.XTickLabel = []; else, xlabel(ax, 'Time (s)', 'Color', ink); end
end
title(tl, sprintf('Closed-loop %s pick: %d attempt, in tray = %d, slip %.1f mm, peak contact %.1f N', P.obj.name, ...
    S.R.attempts, S.R.inTray, S.R.slip * 1e3, S.R.fMax), 'Color', ink, 'FontSize', 11);
nm = strrep(P.obj.name, " ", "_");
exportgraphics(f2, fullfile(imgDir, nm + "_timeline.png"), 'Resolution', 150); close(f2);
fprintf('Wrote finger_validation.png and %s_timeline.png to %s\n', nm, imgDir);
end
