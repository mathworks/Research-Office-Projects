function nMark = hmmShowGrounding(ax, img, actions, cameraIndex, scale)
%HMMSHOWGROUNDING One camera frame with Gemini ER's boxes and points drawn on it.
%
%   nMark = HMMSHOWGROUNDING(ax, img, actions, cameraIndex, scale) draws the frame
%   img into ax and, for every grounded action whose pick or place was measured in
%   this camera, its box_2d and its point: A (orange) where to pick, B (blue) where
%   to place. actions is hmmGroundingTap's decoded queue, cameraIndex the camera the
%   frame came from, and scale cfg.camera.pixelNormScale - Gemini ER answers in
%   normalised [row col] pixels, so they are mapped back onto THIS frame's size.
%   nMark is how many marks landed here.
%
%   The operator console and the live script both draw through this, so the two
%   cannot show the same answer differently.
%
%   See also hmmGroundingTap, hmmGroundingView, HomeMobileManipulatorApp.

%   Copyright 2026 The MathWorks, Inc.

image(ax, uint8(img));
ax.YDir = 'reverse';
axis(ax, 'image');
ax.XTick = [];
ax.YTick = [];
hold(ax, 'on');

nRow = size(img, 1);
nCol = size(img, 2);
pickColour = [0.95 0.35 0.15];
placeColour = [0.20 0.55 0.95];
nMark = 0;
for k = 1:numel(actions)
    a = actions(k);
    if a.pickImage == cameraIndex
        drawBox(ax, boxOf(a, 'pickBox'), nRow, nCol, scale, pickColour);
        drawPoint(ax, a.pickPoint, nRow, nCol, scale, pickColour, sprintf('%s  A', a.label));
        nMark = nMark + 1;
    end
    if strcmp(a.verb, 'transfer') && a.placeImage == cameraIndex
        drawBox(ax, boxOf(a, 'placeBox'), nRow, nCol, scale, placeColour);
        drawPoint(ax, a.placePoint, nRow, nCol, scale, placeColour, 'B');
        nMark = nMark + 1;
    end
end
hold(ax, 'off');
end

function drawPoint(ax, pt, nRow, nCol, scale, colour, label)
% [y x] AND y DOWNWARD, which is the provider's convention and not MATLAB's plotting
% one. Swapping them is the classic way to get a marker that lands plausibly on the
% wrong object, so the mapping is written out once: row from the first element,
% column from the second, each scaled by the frame's own size.
row = pt(1) / scale * nRow;
col = pt(2) / scale * nCol;
plot(ax, col, row, '+', 'Color', colour, 'MarkerSize', 16, 'LineWidth', 2.2);
plot(ax, col, row, 'o', 'Color', colour, 'MarkerSize', 22, 'LineWidth', 1.4);
text(ax, col + 0.02 * nCol, row, label, 'Color', colour, ...
    'FontWeight', 'bold', 'Interpreter', 'none');
end

function drawBox(ax, box, nRow, nCol, scale, colour)
% box_2d is [ymin xmin ymax xmax] NORMALISED TO 0-1000, which is exactly
% cfg.camera.pixelNormScale, so the boxes need no conversion beyond the frame size.
% EMPTY MEANS NO BOX WAS OFFERED - the normal case for navigate and home, and for a
% replayed answer - and is drawn as nothing rather than as a degenerate corner box.
if numel(box) ~= 4 || any(~isfinite(box))
    return
end
r1 = box(1) / scale * nRow;
c1 = box(2) / scale * nCol;
r2 = box(3) / scale * nRow;
c2 = box(4) / scale * nCol;
plot(ax, [c1 c2 c2 c1 c1], [r1 r1 r2 r2 r1], '-', 'Color', colour, 'LineWidth', 1.8);
end

function b = boxOf(action, field)
b = zeros(1, 0);
if ~isstruct(action) || ~isfield(action, field)
    return
end
v = action.(field);
if numel(v) ~= 4 || any(~isfinite(v))
    return
end
b = reshape(double(v), 1, 4);
end
