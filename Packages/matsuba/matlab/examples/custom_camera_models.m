%% Custom Camera Models with MATSUBA
% Demonstrates the programmable camera sensor (mi.sensor.custom) using the
% Cornell box scene. Shows fisheye, panoramic, and depth-of-field rendering
% with custom lens models.
%
% The custom sensor lets you define arbitrary per-pixel ray maps, enabling
% any camera projection (fisheye, panoramic, anamorphic, etc.) and optical
% effects like depth of field — useful for simulating real lens designs.
%
% See also: mi.sensor.custom, mi.sensor.fisheye, mi.sensor.panoramic

%% 1. Setup
matsubaDir = fullfile(fileparts(mfilename("fullpath")), "..", "..");
addpath(fullfile(matsubaDir, "matlab"), matsubaDir);
mi.setup();

%% 2. Build a Cornell box scene (without camera — we'll add our own)
fprintf("Building Cornell box scene...\n");
[shapes, light] = buildCornellBox();

%% 3. Standard perspective camera (reference)
fprintf("Rendering perspective reference...\n"); tic;
perspCam = mi.sensor.perspective( ...
    fov=39.3, ...
    to_world=mi.Transform.lookAt([0 0 3.9], [0 0 0], [0 1 0]), ...
    film=mi.film(Width=256, Height=256));

scenePersp = mi.Scene.build(shapes{:}, light, ...
    perspCam, mi.integrator.path(max_depth=6));
imgPersp = mi.postprocess(scenePersp.render(SamplesPerPixel=64));
fprintf("  done in %.1f s\n", toc);

%% 4. Fisheye camera — equidistant 180 degree FOV (inside the box)
fprintf("Rendering fisheye (180 deg equidistant)...\n"); tic;
fisheyeCam = mi.sensor.custom( ...
    RayFunction=@mi.sensor.fisheye, ...
    to_world=mi.Transform.lookAt([0 0 0.5], [0 0 -1], [0 1 0]), ...
    film=mi.film(Width=256, Height=256));

sceneFisheye = mi.Scene.build(shapes{:}, light, ...
    fisheyeCam, mi.integrator.path(max_depth=6));
imgFisheye = mi.postprocess(sceneFisheye.render(SamplesPerPixel=64));
fprintf("  done in %.1f s\n", toc);

%% 5. Equisolid fisheye — 220 degree super-wide (inside the box)
fprintf("Rendering equisolid fisheye (220 deg)...\n"); tic;
wideCam = mi.sensor.custom( ...
    RayFunction=@(u,v) mi.sensor.fisheye(u, v, FOV=220, Projection="equisolid"), ...
    to_world=mi.Transform.lookAt([0 0 0.5], [0 0 -1], [0 1 0]), ...
    film=mi.film(Width=256, Height=256));

sceneWide = mi.Scene.build(shapes{:}, light, ...
    wideCam, mi.integrator.path(max_depth=6));
imgWide = mi.postprocess(sceneWide.render(SamplesPerPixel=64));
fprintf("  done in %.1f s\n", toc);

%% 6. Panoramic camera — 360 degree equirectangular (inside the box)
fprintf("Rendering panoramic (360 deg)...\n"); tic;
panoCam = mi.sensor.custom( ...
    RayFunction=@mi.sensor.panoramic, ...
    to_world=mi.Transform.lookAt([0 0 0.5], [0 0 -1], [0 1 0]), ...
    film=mi.film(Width=512, Height=256));

scenePano = mi.Scene.build(shapes{:}, light, ...
    panoCam, mi.integrator.path(max_depth=6));
imgPano = mi.postprocess(scenePano.render(SamplesPerPixel=64));
fprintf("  done in %.1f s\n", toc);

%% 7. Depth of field — focus on the tall box
% Camera at z=3.9 (standard Cornell view), tall box at z~-0.29 -> distance ~4.2
% Large aperture for obvious bokeh
fprintf("Rendering with depth of field...\n"); tic;
dofCam = mi.sensor.custom( ...
    RayFunction=@(u,v) mi.sensor.fisheye(u, v, FOV=39.3), ...
    ApertureRadius=0.12, ...
    FocusDistance=4.2, ...
    to_world=mi.Transform.lookAt([0 0 3.9], [0 0 0], [0 1 0]), ...
    film=mi.film(Width=256, Height=256));

sceneDOF = mi.Scene.build(shapes{:}, light, ...
    dofCam, mi.integrator.path(max_depth=6));
imgDOF = mi.postprocess(sceneDOF.render(SamplesPerPixel=256));
fprintf("  done in %.1f s\n", toc);

%% 8. Display results
figure(Name="Custom Camera Models — Cornell Box");
tiledlayout(2, 3, TileSpacing="compact", Padding="compact");

nexttile;
imshow(imgPersp);
title("Perspective (39.3\circ)");

nexttile;
imshow(imgFisheye);
title("Fisheye Equidistant (180\circ)");

nexttile;
imshow(imgWide);
title("Fisheye Equisolid (220\circ)");

nexttile;
imshow(imgDOF);
title("DOF (focus=4.2, f/1.4)");

nexttile([1 2]);
imshow(imgPano);
title("Panoramic (360\circ equirectangular)");

fprintf("Done.\n");

%% --- Local functions ---

function [shapes, light] = buildCornellBox()
%BUILDCORNELLBOX Construct Cornell box geometry and light (no camera).

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

    shapes = {floor_, ceiling, back, greenWall, redWall, tallBox, shortBox};

    light = mi.shape.rectangle(key_="light", ...
        emitter=mi.emitter.area(radiance=mi.rgb([18.387 13.9873 6.7574])), ...
        to_world=mi.Transform.translate([0 0.99 0]) ...
               * mi.Transform.scale([0.23 0.23 0.19]) * rotX(90));
end

function T = rotX(deg)
    c = cosd(deg); s = sind(deg);
    T = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1];
end

function T = rotY(deg)
    c = cosd(deg); s = sind(deg);
    T = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
end
