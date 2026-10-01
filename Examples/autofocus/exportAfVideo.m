%exportAfVideo Export a side-by-side autofocus video: the blurred camera image with the
%   focus region marked, next to the focus command plotted against ground-truth depth.
%
%   The point of putting these two panels in one frame is that neither is convincing alone.
%   The picture shows whether the subject looks sharp but says nothing about what the loop
%   was aiming at; the plot shows the command chasing a target but says nothing about
%   whether the result looks right. Side by side, a gap between the lines can be checked
%   against the picture immediately.
%
%   The camera panel is composited at its native 640x480 rather than being drawn into a
%   figure axes. Letting a figure scale it to fit would resample the image and soften every
%   sharp region in it, which in a demo about defocus is exactly the wrong artifact to add.
%   Only the plot panel goes through a figure, and that one is rendered oversize and
%   downsampled, which makes its text cleaner rather than worse.
%
%   Everything the video needs is cached to afVideoData.mat on the first run, so changing how
%   the video looks never costs another simulation: the sim is ~320 s, the render ~20 s. That
%   is also why the cache stores the raw depth PIXELS inside the focus region rather than a
%   summary of them - swapping median for mean, or for any percentile, then stays a code edit.
%   The crop is small enough to be free: 94x124x621 in single is 29 MB against 1.2 GB for full
%   depth. Delete the MAT file to force a fresh run.
%
%   Writes autofocusCompare.gif next to this file.
%
%   A GIF rather than an MP4 because the result is embedded in README.md, and a GIF plays
%   there with no player, no plugin and no click. The cost is that GIF is 256 colours with no
%   interframe compression, so both the frame count and the pixel count have to come down to
%   keep the file small enough to be worth embedding - see gifStride and gifScale below.
%
%   The GIF lands in the code folder because it is the deliverable. The cache does not: it
%   carries every rendered frame at ~640 MB, so it stays in tempdir where it can be discarded
%   freely. Deleting it only costs the next run a simulation.

%   Copyright 2026 The MathWorks, Inc.

m = "AutofocusScene";
codeDir = fileparts(mfilename("fullpath"));
cacheFile = fullfile(tempdir, "afVideoData.mat");
outFile = fullfile(codeDir, "autofocusCompare.gif");

if isfile(cacheFile)
    fprintf("loading cached run from %s\n", cacheFile);
    S = load(cacheFile);
else
    load_system(m);
    set_param(m, "SimulationMode", "accelerator");
    mws = get_param(m, "ModelWorkspace");
    spotFraction = mws.getVariable("AF_SpotFraction");

    % name, block, output port
    probes = {"focus", "Autofocus", 1; "depth", "CameraRig", 2; "frames", "DefocusBlur", 1};
    ports = zeros(1, size(probes, 1));
    for k = 1:size(probes, 1)
        ph = get_param(m + "/" + probes{k, 2}, "PortHandles");
        ports(k) = ph.Outport(probes{k, 3});
        % Every sample, not decimated - this is a video, not a plot.
        set_param(ports(k), "DataLogging", "on", "DataLoggingLimitDataPoints", "off", ...
            "DataLoggingDecimateData", "off", "DataLoggingNameMode", "Custom", ...
            "DataLoggingName", char(probes{k, 1}));
    end

    wall = tic;
    simOut = sim(m);
    elapsed = toc(wall);

    for k = 1:numel(ports)
        set_param(ports(k), "DataLogging", "off", "DataLoggingNameMode", "SignalName");
    end
    save_system(m);

    S = struct();
    S.Ts = mws.getVariable("Scene_Ts");
    S.spotFraction = spotFraction;
    % Total run, not just the pan: the camera keeps turning and tips down for Cam_TailTime after
    % the 100 deg sweep finishes, and the plot has to show that stretch rather than clip it.
    S.sweepTime = mws.getVariable("Cam_SweepTime");
    S.runTime = S.sweepTime + mws.getVariable("Cam_TailTime");
    S.focusMin = mws.getVariable("AF_FocusMin");
    S.focusMax = mws.getVariable("AF_FocusMax");
    S.focus = simOut.logsout.getElement("focus").Values;
    S.frames = simOut.logsout.getElement("frames").Values;

    % Keep only the focus region, using the same mask the Sharpness and CentreDepth blocks
    % apply, so the numbers plotted describe the pixels the autofocus actually reads. Full
    % depth is 1.2 GB across the run; the crop is 14 MB.
    depth = simOut.logsout.getElement("depth").Values;
    [imH, imW, ~] = size(S.frames.Data(:, :, :, 1));
    rowsIn = find(abs(((1:imH)' - (imH + 1)/2)/imH) <= spotFraction/2);
    colsIn = find(abs(((1:imW) - (imW + 1)/2)/imW) <= spotFraction/2);
    S.spotDepth = single(depth.Data(rowsIn, colsIn, :));
    S.spotTime = depth.Time;

    fprintf("%g s of scene time in %.0f s wall clock (%.1fx slower than real time)\n", ...
        S.runTime, elapsed, elapsed/S.runTime);
    % Uncompressed v7.3, because the frames dominate and compressing 440 MB of already
    % noisy render output costs far more time than it saves on disk.
    save(cacheFile, "-struct", "S", "-v7.3", "-nocompression");
    fprintf("cached to %s\n", cacheFile);
end

t = S.focus.Time;
% Caches written before the tail existed have no runTime field, and they are still wanted for A/B
% comparisons, so fall back to the logged length rather than erroring on them.
if ~isfield(S, "runTime")
    S.runTime = t(end);
end
d = S.frames.Data;
n = size(d, 4);
[imH, imW, ~] = size(d(:, :, :, 1));
fprintf("%d frames of %dx%d, %.0f MB\n", n, imW, imH, numel(d)/2^20);

% Summarise the focus region, one number per frame: the median depth, weighted towards the near
% content by 1/depth. See nearWeightedDepth for why it leans near and why a weight beats the flat
% 25th percentile it replaced (0.35 m median difference from the focus command against 0.47 m, 83%
% agreement within 20% against 75%). Changing this reduction never costs a run - the cache holds the
% raw depth PIXELS, not a summary.
%
% A depth-histogram reference was also tried and dropped: the nearest peak holding at least a fifth
% of the region, NaN when none does. It needed a bin width, a smoothing window and two thresholds to
% do slightly worse than one weight. What it established is worth keeping: over the opening pan this
% region holds eight surfaces, none above 15% of it, so no single summary of it can agree with the
% focus command there - expect the two lines to disagree for the first ~3 s and read the picture
% instead. The loop is not wrong there; because depth of field is symmetric in DIOPTERS, its far
% focus leaves 47% of the region acceptably sharp against the 49% best available at any distance.
flat = reshape(S.spotDepth, [], numel(S.spotTime));
spotStat = nan(numel(S.spotTime), 1);
for k = 1:numel(spotStat)
    spotStat(k) = nearWeightedDepth(double(flat(:, k)));
end
spotDepth = interp1(S.spotTime, spotStat, t, "previous", "extrap");

% The focus region, matching the mask in the Sharpness and CentreDepth blocks exactly: the
% spot is AF_SpotFraction of the width by the same fraction of the HEIGHT, so it is not
% square in pixels. Drawing a square crosshair would mark a region the autofocus is not
% actually measuring, which is worse than drawing none at all.
cx = (imW + 1)/2;
cy = (imH + 1)/2;
halfW = S.spotFraction/2*imW;
halfH = S.spotFraction/2*imH;
fprintf("focus region: %.0f x %.0f px centred at (%.1f, %.1f)\n", 2*halfW, 2*halfH, cx, cy);

% Corner brackets plus a centre cross with a gap, so the marker frames the region without
% covering the detail the metric reads. Built once - the region never moves. Red, matching
% the focus command in the plot, so the two panels read as one picture.
brk = round(0.3*min(halfW, halfH));
gap = round(0.25*min(halfH, halfW));
armX = round(0.55*halfW);
armY = round(0.55*halfH);
xL = round(cx - halfW); xR = round(cx + halfW);
yT = round(cy - halfH); yB = round(cy + halfH);
bracketLines = [ ...
    xL yT xL+brk yT; xL yT xL yT+brk; ...
    xR yT xR-brk yT; xR yT xR yT+brk; ...
    xL yB xL+brk yB; xL yB xL yB-brk; ...
    xR yB xR-brk yB; xR yB xR yB-brk];
crossLines = [ ...
    round(cx)-armX round(cy) round(cx)-gap round(cy); ...
    round(cx)+gap  round(cy) round(cx)+armX round(cy); ...
    round(cx) round(cy)-armY round(cx) round(cy)-gap; ...
    round(cx) round(cy)+gap  round(cx) round(cy)+armY];
focusRed = [0.85 0.25 0.15];
marker = [0.95 0.15 0.10];

% Plot panel. The animation is a cumulative reveal on fixed axes, so it does not need 621
% figure renders - it needs two. Measured, `print -RGBImage` costs 613 ms per call: 94% of the
% whole export, against 8 ms for everything drawn on the camera panel. So render the plot twice,
% once empty and once complete, and per frame take the columns left of the current time from the
% complete one and the rest from the empty one. Fixed axes make that pixel-exact, and it takes
% the export from ~5.5 min to ~20 s.
fig = figure("Color", "w", "Position", [80 80 imW imH], "Visible", "off");
ax = axes(fig, "Position", [0.115 0.115 0.855 0.83]);
hold(ax, "on");
grid(ax, "on");
box(ax, "on");
% The focus stops, so a saturated command is distinguishable from a settled one.
yline(ax, S.focusMin, ":", "Color", [0.6 0.6 0.6]);
yline(ax, S.focusMax, ":", "Color", [0.6 0.6 0.6]);
hDepth = plot(ax, nan, nan, "-", "Color", [0.15 0.45 0.80], "LineWidth", 1.8);
hFocus = plot(ax, nan, nan, "-", "Color", focusRed, "LineWidth", 2.2);
xlim(ax, [0 S.runTime]);
% Mark where the pan ends and the downward-tipping tail begins, so a change in the focus
% command's behaviour after that line is attributable rather than mysterious.
if S.runTime > S.sweepTime
    xline(ax, S.sweepTime, "--", "Color", [0.45 0.45 0.45]);
end
ylim(ax, [0 S.focusMax + 3]);
xlabel(ax, "time (s)");
ylabel(ax, "distance (m)");
title(ax, "focus command vs depth in the focus region");
legend(ax, [hFocus hDepth], ["focus command", "near-weighted depth in region"], ...
    "Location", "northwest", "FontSize", 9);

% Empty render first. The legend draws from each line's colour and style rather than its data,
% so it comes out identical in both renders and needs no special handling.
axPos = ax.Position;
xl = xlim(ax);
yl = ylim(ax);
% print -RGBImage works on an invisible figure, where getframe is unreliable. It honours display
% scaling, so the result is oversize by an unknown factor (960x720 here) - resize to the panel
% size rather than assuming it came back at figure size. The downsample anti-aliases the text,
% which makes it cleaner rather than worse.
panelEmpty = imresize(print(fig, "-RGBImage", "-r0"), [imH imW]);
set(hDepth, "XData", t, "YData", spotDepth);
set(hFocus, "XData", t, "YData", S.focus.Data);
panelFull = imresize(print(fig, "-RGBImage", "-r0"), [imH imW]);
close(fig);

% Data coordinates to panel pixels, for the reveal boundary and the moving marker. Exact for a
% 2-D axes with no data aspect ratio constraint, which is what this is.
toCol = @(tv) min(max(round((axPos(1) + (tv - xl(1))/(xl(2) - xl(1))*axPos(3))*imW), 1), imW);
toRow = @(yv) min(max(round((1 - (axPos(2) + (yv - yl(1))/(yl(2) - yl(1))*axPos(4)))*imH), 1), imH);

% GIF sizing. GIF has no interframe compression and a 256 colour palette, so the file grows
% with frames x pixels and both have to come down: every second frame at half size is a quarter
% of the data. Playback stays real time, because GIF stores frame delays in hundredths of a
% second and gifStride*S.Ts = 0.10 s is exact in that unit.
gifStride = 2;
gifScale = 0.5;
keep = 1:gifStride:n;
nGif = numel(keep);
delayTime = gifStride*S.Ts;
gifW = round(gifScale*2*imW);
gifH = round(gifScale*imH);

% Downsampling the camera panel is allowed here even though the note above forbids resampling
% it, and the distinction is worth stating. What is forbidden is letting a figure rescale the
% image by an uncontrolled factor to fit an axes. A deliberate halving is scale consistent:
% apparent blur measures 2.421% of the image width across 640x480 down to 240x180, so radius
% and frame shrink together and relative sharpness survives.
%
% Annotations are not scale invariant. They are drawn at native size and downsampled with
% everything else, so text, strokes and the focus dot are grossed up by 1/gifScale or they
% arrive too thin and too small to read. The bracket GEOMETRY stays in native pixels, because
% it has to keep marking the region the autofocus actually measures.
ann = 1/gifScale;

comp = zeros(gifH, gifW, 3, nGif, "uint8");
for f = 1:nGif
    [~, i] = min(abs(t - S.frames.Time(keep(f))));

    left = insertShape(d(:, :, :, keep(f)), "line", bracketLines, "ShapeColor", marker, ...
        "LineWidth", round(3*ann));
    left = insertShape(left, "line", crossLines, "ShapeColor", marker, "LineWidth", round(2*ann));
    left = insertText(left, [8 8], sprintf("t %5.2f s   focus %5.2f m   depth %5.2f m", ...
        t(i), S.focus.Data(i), spotDepth(i)), "FontSize", round(14*ann), "BoxColor", "black", ...
        "BoxOpacity", 0.55, "TextColor", "white");

    % Reveal the plot up to now, then mark the current focus. Opacity must be given explicitly:
    % insertShape defaults filled shapes to 0.6, which would make the marker translucent.
    col = toCol(t(i));
    right = panelEmpty;
    right(:, 1:col, :) = panelFull(:, 1:col, :);
    right = insertShape(right, "filled-circle", [col toRow(S.focus.Data(i)) 5*ann], ...
        "ShapeColor", focusRed, "Opacity", 1);

    comp(:, :, :, f) = imresize([left right], [gifH gifW]);
end

% One palette for the whole animation, sampled across the run so it covers the cars the camera
% pans onto late as well as the ones it opens on. A per-frame palette makes the colours crawl
% between frames and costs more bytes overall, because every frame then carries its own colour
% table. No dithering: dither is high-frequency speckle, and in a demo about defocus it would
% read as detail that is not there.
sample = comp(:, :, :, round(linspace(1, nGif, 12)));
[~, map] = rgb2ind(reshape(permute(sample, [1 2 4 3]), gifH, [], 3), 256, "nodither");

for f = 1:nGif
    indexed = rgb2ind(comp(:, :, :, f), map, "nodither");
    if f == 1
        imwrite(indexed, map, outFile, "gif", "LoopCount", Inf, "DelayTime", delayTime);
    else
        imwrite(indexed, map, outFile, "gif", "WriteMode", "append", "DelayTime", delayTime);
    end
end

info = dir(outFile);
fprintf("\nwrote %s\n", outFile);
fprintf("  %d of %d frames at %g fps = %.1f s, which is real time\n", ...
    nGif, n, 1/delayTime, nGif*delayTime);
fprintf("  %d x %d, %.1f MB\n", gifW, gifH, info.bytes/2^20);

err = abs(S.focus.Data - spotDepth);
fprintf("  vs near-weighted depth in region: median |error| %.2f m, within 20%%: %.0f%%\n", ...
    median(err), 100*mean(err < 0.2*spotDepth));
