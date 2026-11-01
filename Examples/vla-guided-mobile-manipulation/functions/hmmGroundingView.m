function hmmGroundingView(action)
%HMMGROUNDINGVIEW A figure that shows each Gemini ER answer as it arrives.
%
%   HMMGROUNDINGVIEW('start') opens (or reuses) one figure and registers two
%   listeners: every grounding answer redraws it with the frames that were sent and
%   Gemini ER's boxes and points on them, and the wrist refinement adds the close-up
%   frame with its refined point. HMMGROUNDINGVIEW('stop') removes the listeners and
%   leaves the figure as it is.
%
%   NOTHING IS POLLED. hmmGroundingTap and hmmRefineNotify call the listeners from
%   inside the simulation step that produced the answer, so this costs the run
%   nothing between answers - which is the point of the live script's quick run,
%   where this figure and the Unreal window are the only visualisation.
%
%   See also hmmShowGrounding, hmmGroundingTap, hmmRefineNotify, hmmSubmitTask.

%   Copyright 2026 The MathWorks, Inc.

arguments
    action (1, :) char {mustBeMember(action, {'start', 'stop'})} = 'start'
end

if strcmp(action, 'stop')
    if isappdata(0, 'hmmGroundingListener'), rmappdata(0, 'hmmGroundingListener'); end
    if isappdata(0, 'hmmRefineListener'), rmappdata(0, 'hmmRefineListener'); end
    return
end

cfg = hmmConfig();
fig = viewFigure();
clf(fig);
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', 'Position', [0 0.45 1 0.1], ...
    'String', 'Waiting for Gemini ER - submit an instruction with hmmSubmitTask("...")', ...
    'FontAngle', 'italic', 'BackgroundColor', fig.Color);
setappdata(0, 'hmmGroundingListener', @(tap) onTap(tap, cfg));
setappdata(0, 'hmmRefineListener', @(ev, rec) onRefine(ev, rec, cfg));
end

function fig = viewFigure()
fig = findall(groot, 'Type', 'figure', 'Tag', 'hmmGroundingView');
if isempty(fig)
    fig = figure('Tag', 'hmmGroundingView', 'Name', 'Gemini ER grounding', ...
        'NumberTitle', 'off', 'Color', 'w');
end
fig = fig(1);
end

function onTap(tap, cfg)
% One tile per frame that was sent, plus a spare tile for the wrist look.
fig = viewFigure();
clf(fig);
n = numel(tap.images);
tl = tiledlayout(fig, 'flow', 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:n
    ax = nexttile(tl);
    cam = tap.indices(i);
    marks = hmmShowGrounding(ax, tap.images{i}, tap.actions, cam, cfg.camera.pixelNormScale);
    if marks == 0
        title(ax, sprintf('camera %d', cam));
    else
        title(ax, sprintf('camera %d - %d mark(s)', cam, marks));
    end
end
verbs = strjoin(unique({tap.actions.verb}, 'stable'), ', ');
if isempty(verbs)
    verbs = 'no action';
end
title(tl, {sprintf('"%s"', tap.instruction), sprintf(['t = %.2f s, %s, %.1f s - ' ...
    'A pick (orange), B place (blue)'], tap.simTime, verbs, tap.latency)}, ...
    'Interpreter', 'none', 'FontSize', 10);
setappdata(fig, 'tiles', tl);
drawnow limitrate
end

function onRefine(ev, rec, cfg)
% The wrist camera's second look, in its own tile beside the room cameras.
fig = viewFigure();
tl = getappdata(fig, 'tiles');
if isempty(tl) || ~isvalid(tl)
    return
end
ax = findobj(fig, 'Type', 'axes', 'Tag', 'wrist');
if strcmp(ev, 'request') || isempty(ax)
    delete(ax);
    ax = nexttile(tl);
    ax.Tag = 'wrist';
    image(ax, uint8(rec.RGB));
    ax.YDir = 'reverse';
    axis(ax, 'image');
    ax.XTick = [];
    ax.YTick = [];
    ax.Tag = 'wrist';
    title(ax, 'wrist camera - asking Gemini ER to refine', 'Color', [0.85 0.45 0.05]);
end
if strcmp(ev, 'answer')
    px = double(rec.Pixel(:));
    if double(rec.Status) == cfg.refine.STATUS_APPLIED && numel(px) == 2
        hold(ax, 'on');
        plot(ax, px(2), px(1), '+', 'Color', [0.95 0.35 0.15], 'MarkerSize', 18, 'LineWidth', 2.4);
        plot(ax, px(2), px(1), 'o', 'Color', [0.95 0.35 0.15], 'MarkerSize', 26, 'LineWidth', 1.6);
        hold(ax, 'off');
        title(ax, sprintf('wrist camera - grasp here (%.0f mm shift)', 1000 * rec.Shift), ...
            'Color', [0.95 0.35 0.15]);
    elseif double(rec.Status) == cfg.refine.STATUS_DEPTHONLY
        % Applied too, but measured from the wrist depth alone: Gemini's pixel was not
        % usable, so it is not drawn as the grasp point.
        title(ax, sprintf('wrist camera - grasp measured from depth only (%.0f mm shift)', ...
            1000 * rec.Shift), 'Color', [0.85 0.45 0.05]);
    else
        title(ax, 'wrist camera - refinement not applied', 'Color', [0.80 0.10 0.10]);
    end
end
drawnow limitrate
end
