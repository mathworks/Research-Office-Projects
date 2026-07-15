%% Cornell Box AOV Rendering — MATSUBA Example
% Renders a Cornell box scene and extracts Arbitrary Output Variables
% (AOVs) — depth, surface normals, world position, and UV coordinates —
% displayed as a montage alongside the RGB image.
%
% AOVs are useful for compositing, deferred shading, machine learning
% training data, and scene analysis. MATSUBA's renderAOV method handles
% the Mitsuba AOV integrator setup automatically.
%
% See also: mi.Scene.renderAOV, mi.integrator.aov

%% 1. Setup
addpath(fullfile(fileparts(mfilename("fullpath")), ".."));
mi.setup;

%% 2. Build the Cornell box from primitives
fprintf("Building Cornell box scene...\n");
scene = buildCornellBox();

%% 3. Standard RGB render
fprintf("Rendering RGB image...\n");
hdr = scene.render(SamplesPerPixel=64);
rgb = mi.postprocess(hdr);

%% 4. AOV render — depth, normals, position
fprintf("Rendering AOVs...\n");
r = scene.renderAOV(["depth", "normals", "position"], SamplesPerPixel=64);
fprintf("  Depth range:    [%.2f, %.2f]\n", min(r.depth(:)), max(r.depth(:)));
fprintf("  Position range: [%.2f, %.2f]\n", min(r.position(:)), max(r.position(:)));

%% 5. Prepare visualizations
% Depth: normalize and apply a colormap
depthVis = r.depth;
depthMask = depthVis > 0;
depthVis(~depthMask) = NaN;
depthVis = (depthVis - min(depthVis(:))) / (max(depthVis(:)) - min(depthVis(:)));

% Normals: remap [-1, 1] -> [0, 1] for display
normalsVis = r.normals * 0.5 + 0.5;

% Position: remap to [0, 1] per channel
posVis = r.position;
posVis = (posVis - min(posVis(:))) / (max(posVis(:)) - min(posVis(:)));

%% 6. Display as 2x2 montage
fig = figure("Name", "Cornell Box — AOV Montage", ...
    Position=[100 100 800 800], Color="k");

% RGB
ax1 = subplot(2,2,1);
imshow(rgb, Parent=ax1);
title(ax1, "RGB Render", Color="w");

% Depth
ax2 = subplot(2,2,2);
imagesc(ax2, depthVis, AlphaData=depthMask);
axis(ax2, "image", "off");
colormap(ax2, "turbo");
cb = colorbar(ax2, Color="w");
cb.Label.String = "Normalized Depth";
cb.Label.Color = "w";
title(ax2, "Depth", Color="w");
set(ax2, Color="k");

% Normals
ax3 = subplot(2,2,3);
imshow(normalsVis, Parent=ax3);
title(ax3, "Shading Normals", Color="w");

% World Position
ax4 = subplot(2,2,4);
imshow(posVis, Parent=ax4);
title(ax4, "World Position", Color="w");

%% 7. Save output
outPath = fullfile(fileparts(mfilename("fullpath")), "cornell_box_aov.png");
exportgraphics(fig, outPath, Resolution=150, BackgroundColor="k");
fprintf("Saved to %s\n", outPath);

delete(scene);
fprintf("Done!\n");


%% --- Local functions ---

function scene = buildCornellBox()
%BUILDCORNELLBOX Construct a Cornell box scene from rectangles and cubes.
%   Builds the classic Cornell box: white floor/ceiling/back, green left
%   wall, red right wall, two rotated boxes, and a ceiling area light.

    white = mi.bsdf.diffuse(reflectance=mi.rgb([0.885 0.698 0.666]));
    green = mi.bsdf.diffuse(reflectance=mi.rgb([0.105 0.37 0.067]));
    red   = mi.bsdf.diffuse(reflectance=mi.rgb([0.57 0.043 0.043]));

    % Floor (y = -1, facing up)
    floor_ = mi.shape.rectangle( ...
        bsdf=white, key_="floor", ...
        to_world=mi.Transform.translate([0 -1 0]) ...
               * mi.Transform.scale([1 1 1]) ...
               * rotX(-90));

    % Ceiling (y = 1, facing down)
    ceiling = mi.shape.rectangle( ...
        bsdf=white, key_="ceiling", ...
        to_world=mi.Transform.translate([0 1 0]) ...
               * mi.Transform.scale([1 1 1]) ...
               * rotX(90));

    % Back wall (z = -1, facing forward)
    back = mi.shape.rectangle( ...
        bsdf=white, key_="back", ...
        to_world=mi.Transform.translate([0 0 -1]));

    % Green wall (x = -1, facing right)
    greenWall = mi.shape.rectangle( ...
        bsdf=green, key_="green_wall", ...
        to_world=mi.Transform.translate([-1 0 0]) ...
               * rotY(90));

    % Red wall (x = 1, facing left)
    redWall = mi.shape.rectangle( ...
        bsdf=red, key_="red_wall", ...
        to_world=mi.Transform.translate([1 0 0]) ...
               * rotY(-90));

    % Tall box
    tallBox = mi.shape.cube( ...
        bsdf=white, key_="tall_box", ...
        to_world=mi.Transform.translate([0.335 -0.4 -0.29]) ...
               * rotY(18.3) ...
               * mi.Transform.scale([0.31 0.6 0.31]));

    % Short box
    shortBox = mi.shape.cube( ...
        bsdf=white, key_="short_box", ...
        to_world=mi.Transform.translate([-0.33 -0.7 0.27]) ...
               * rotY(-17.3) ...
               * mi.Transform.scale([0.285 0.285 0.285]));

    % Area light on ceiling (rotX(90) so light faces downward)
    light_ = mi.shape.rectangle( ...
        key_="light", ...
        emitter=mi.emitter.area(radiance=mi.rgb([18.387 13.9873 6.7574])), ...
        to_world=mi.Transform.translate([0 0.99 0]) ...
               * mi.Transform.scale([0.23 0.23 0.19]) ...
               * rotX(90));

    % Camera — standard Cornell box view
    camera = mi.sensor.perspective( ...
        fov=39.3, ...
        to_world=mi.Transform.lookAt([0 0 3.9], [0 0 0], [0 1 0]), ...
        film=mi.film(Width=512, Height=512));

    scene = mi.Scene.build( ...
        floor_, ceiling, back, greenWall, redWall, ...
        tallBox, shortBox, light_, ...
        camera, mi.integrator.path(max_depth=6));
end

function T = rotX(deg)
%ROTX Rotation matrix around X axis.
    c = cosd(deg); s = sind(deg);
    T = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1];
end

function T = rotY(deg)
%ROTY Rotation matrix around Y axis.
    c = cosd(deg); s = sind(deg);
    T = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
end
