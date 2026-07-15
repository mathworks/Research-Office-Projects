%% Transient Rendering — Light in Motion
% Renders a Cornell box using mitransient to produce a time-resolved
% animation of light propagating through the scene. Each frame shows where
% light has reached at a given moment, creating a "light cube" effect.
%
% This demonstrates LiDAR-style time-of-flight rendering: the transient
% film records photon arrival times as optical path length (OPL) histograms,
% producing a 4-D dataset (H x W x T x 3) alongside the steady-state image.
%
% Requires: mitransient (installed automatically on first use).
%
% See also: mi.transientFilm, mi.integrator.transientPath,
%           mi.Scene.renderTransient

%% 1. Setup
addpath(fullfile(fileparts(mfilename("fullpath")), ".."));
mi.setup;

%% 2. Build Cornell box with transient film and integrator
fprintf("Building transient Cornell box scene...\n");
scene = buildTransientCornellBox();

%% 3. Render — returns steady-state image and transient 4-D data
fprintf("Rendering transient scene...\n");
tic;
[steady, transient] = scene.renderTransient(SamplesPerPixel=1024);
fprintf("  Done in %.1f s\n", toc);
fprintf("  Steady-state: %s\n", mat2str(size(steady)));
fprintf("  Transient:    %s\n", mat2str(size(transient)));

%% 4. Post-process steady-state image
steadyRGB = mi.postprocess(steady);

%% 5. Temporal smoothing — average adjacent bins to reduce noise
% At N spp with T bins, each bin gets ~N/T effective samples, so smoothing
% over a few temporal neighbors dramatically improves signal-to-noise.
fprintf("  Applying temporal smoothing...\n");
kernelWidth = 5;
kernel = ones(1, 1, kernelWidth, 1) / kernelWidth;
transient = convn(transient, kernel, "same");

nFrames = size(transient, 3);
fprintf("  Temporal bins: %d\n", nFrames);
energy = squeeze(sum(transient, [1 2 4]));
threshold = max(energy) * 0.005;
activeFrames = find(energy > threshold);
if isempty(activeFrames)
    warning("No active transient frames found.");
    return
end
startFrame = activeFrames(1);
endFrame = activeFrames(end);
fprintf("  Active range: frames %d to %d (of %d)\n", startFrame, endFrame, nFrames);

%% 6. OPL parameters for labeling
startOPL = 3.5;
binWidth = 0.048;

%% 7. Create animation figure
fig = figure("Name", "Transient Light Transport — Cornell Box", ...
    Position=[100 100 900 420], Color="k");

% Left: steady-state reference
ax1 = subplot(1,2,1);
imshow(steadyRGB, Parent=ax1);
title(ax1, "Steady-State", Color="w", FontSize=12);

% Right: transient overlay animation
ax2 = subplot(1,2,2);
hImg = imshow(zeros(size(steadyRGB)), Parent=ax2);
hTitle = title(ax2, "", Color="w", FontSize=12);

% Animate through active frames
step = max(1, floor((endFrame - startFrame + 1) / 120));
frameIndices = startFrame:step:endFrame;

fprintf("Animating %d frames...\n", numel(frameIndices));
for idx = 1:numel(frameIndices)
    t = frameIndices(idx);
    blended = tonemapTransientFrame(transient(:,:,t,:), steadyRGB);
    hImg.CData = blended;
    opl = startOPL + (t - 1) * binWidth;
    hTitle.String = sprintf("OPL = %.2f m  (frame %d)", opl, t);
    drawnow;
    pause(0.03);
end

%% 8. Export animation as GIF
gifPath = fullfile(fileparts(mfilename("fullpath")), "transient_cornell_box.gif");
fprintf("Exporting GIF (%d frames)...\n", numel(frameIndices));
for idx = 1:numel(frameIndices)
    t = frameIndices(idx);
    blended = tonemapTransientFrame(transient(:,:,t,:), steadyRGB);

    % Convert to indexed color for GIF
    rgb8 = im2uint8(blended);
    [ind, cmap] = rgb2ind(rgb8, 256, "dither");

    if idx == 1
        imwrite(ind, cmap, gifPath, "gif", ...
            LoopCount=Inf, DelayTime=0.04);
    else
        imwrite(ind, cmap, gifPath, "gif", ...
            WriteMode="append", DelayTime=0.04);
    end
end
fprintf("Saved GIF to %s\n", gifPath);

%% 9. Save a montage of key frames
fprintf("Saving montage...\n");
montFig = figure("Name", "Transient Montage", ...
    Position=[100 100 1000 220], Color="k");

nPanels = 5;
panelFrames = round(linspace(startFrame, endFrame, nPanels));
for i = 1:nPanels
    t = panelFrames(i);
    ax = subplot(1, nPanels, i);
    blended = tonemapTransientFrame(transient(:,:,t,:), steadyRGB);
    imshow(blended, Parent=ax);
    opl = startOPL + (t - 1) * binWidth;
    title(ax, sprintf("OPL=%.2f", opl), Color="w", FontSize=9);
end

outPath = fullfile(fileparts(mfilename("fullpath")), "transient_cornell_box.png");
exportgraphics(montFig, outPath, Resolution=150, BackgroundColor="k");
fprintf("Saved to %s\n", outPath);

delete(scene);
close(fig);
close(montFig);
fprintf("Done!\n");


%% --- Local functions ---

function scene = buildTransientCornellBox()
%BUILDTRANSIENTCORNELLBOX Construct a Cornell box with transient rendering.

    white = mi.bsdf.diffuse(reflectance=mi.rgb([0.885 0.698 0.666]));
    green = mi.bsdf.diffuse(reflectance=mi.rgb([0.105 0.37 0.067]));
    red   = mi.bsdf.diffuse(reflectance=mi.rgb([0.57 0.043 0.043]));

    floor_ = mi.shape.rectangle(bsdf=white, key_="floor", ...
        to_world=mi.Transform.translate([0 -1 0]) * rotX(-90));
    ceiling = mi.shape.rectangle(bsdf=white, key_="ceiling", ...
        to_world=mi.Transform.translate([0 1 0]) * rotX(90));
    back = mi.shape.rectangle(bsdf=white, key_="back", ...
        to_world=mi.Transform.translate([0 0 -1]));
    greenWall = mi.shape.rectangle(bsdf=green, key_="green_wall", ...
        to_world=mi.Transform.translate([-1 0 0]) * rotY(90));
    redWall = mi.shape.rectangle(bsdf=red, key_="red_wall", ...
        to_world=mi.Transform.translate([1 0 0]) * rotY(-90));

    tallBox = mi.shape.cube(bsdf=white, key_="tall_box", ...
        to_world=mi.Transform.translate([0.335 -0.4 -0.29]) ...
               * rotY(18.3) * mi.Transform.scale([0.31 0.6 0.31]));
    shortBox = mi.shape.cube(bsdf=white, key_="short_box", ...
        to_world=mi.Transform.translate([-0.33 -0.7 0.27]) ...
               * rotY(-17.3) * mi.Transform.scale([0.285 0.285 0.285]));

    light_ = mi.shape.rectangle(key_="light", ...
        emitter=mi.emitter.area(radiance=mi.rgb([18.387 13.9873 6.7574])), ...
        to_world=mi.Transform.translate([0 0.99 0]) ...
               * mi.Transform.scale([0.23 0.23 0.19]) * rotX(90));

    % Camera with transient film
    % OPL range: 3.5 to 3.5 + 0.048*125 = 9.5 covers the Cornell box
    camera = mi.sensor.perspective( ...
        fov=39.3, ...
        to_world=mi.Transform.lookAt([0 0 3.9], [0 0 0], [0 1 0]), ...
        film=mi.transientFilm( ...
            Width=256, Height=256, ...
            TemporalBins=125, ...
            BinWidthOPL=0.048, ...
            StartOPL=3.5));

    scene = mi.Scene.build( ...
        floor_, ceiling, back, greenWall, redWall, ...
        tallBox, shortBox, light_, ...
        camera, mi.integrator.transientPath(max_depth=6));
end

function blended = tonemapTransientFrame(frameHDR, steadyRGB)
%TONEMAPTRANSIENTFRAME Tonemap and denoise one transient frame, then overlay.
%   Uses firefly clamping, adaptive exposure, Reinhard tonemapping, and
%   adaptive median denoising so that dim transient light is visible.

    frame = squeeze(frameHDR);          % (H x W x 3)
    luminance = 0.2126*frame(:,:,1) + 0.7152*frame(:,:,2) + 0.0722*frame(:,:,3);

    litPixels = luminance(luminance > 0);
    if isempty(litPixels)
        blended = steadyRGB * 0.5;
        return
    end

    % Firefly clamping (99th percentile of lit pixels)
    threshold = prctile(litPixels, 99);
    threshold = max(threshold, 1e-6);
    clampScale = min(threshold ./ max(luminance, 1e-6), 1);
    frame = frame .* clampScale;
    luminance = luminance .* clampScale;

    % Adaptive exposure: target median-lit pixel at ~0.4
    medianL = median(litPixels(litPixels <= threshold));
    exposure = 0.4 / max(medianL, 1e-6);
    exposed = frame * exposure;

    % Reinhard tonemapping: L_mapped = L / (1 + L)
    lumExposed = luminance * exposure;
    mapped = lumExposed ./ (1 + lumExposed);
    scale = mapped ./ max(lumExposed, 1e-8);
    tonemapped = exposed .* scale;

    % Gamma correction
    tonemapped = min(max(tonemapped, 0), 1) .^ (1/2.2);

    % Adaptive median denoising — replace only outlier pixels
    for c = 1:3
        ch = tonemapped(:,:,c);
        med = medfilt2(ch, [3 3]);
        absDev = abs(ch - med);
        localMAD = ordfilt2(absDev, 5, ones(3));
        isOutlier = absDev > 2.5 * max(localMAD, 1e-4);
        ch(isOutlier) = med(isOutlier);
        tonemapped(:,:,c) = ch;
    end

    % Blend over steady-state: use luminance-based alpha
    alpha = min(1, mapped * 2.5);
    blended = steadyRGB .* 0.5 .* (1 - alpha) + tonemapped;
    blended = min(1, max(0, blended));
end

function T = rotX(deg)
    c = cosd(deg); s = sind(deg);
    T = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1];
end

function T = rotY(deg)
    c = cosd(deg); s = sind(deg);
    T = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
end
