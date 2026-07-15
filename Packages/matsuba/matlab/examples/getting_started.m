%% Getting Started with MATSUBA
% A minimal example that builds a scene from scratch and renders it with
% Mitsuba 3. Shows the core workflow: setup, build, render, display.
%
% No external files or optional toolboxes required.
%
% See also: mi.setup, mi.Scene.build, mi.postprocess

%% 1. Setup — configure Mitsuba
matsubaDir = fullfile(fileparts(mfilename("fullpath")), "..", "..");
addpath(fullfile(matsubaDir, "matlab"), matsubaDir);
mi.setup;

%% 2. Define materials
red    = mi.bsdf.plastic(diffuse_reflectance=mi.rgb([0.8 0.05 0.05]));
gold   = mi.bsdf.roughconductor(material="Au", alpha=0.05);
white  = mi.bsdf.diffuse(reflectance=mi.rgb([0.9 0.9 0.9]));

%% 3. Create shapes
% A red sphere
sphere = mi.shape.sphere( ...
    center=[0 0 0], radius=0.5, ...
    bsdf=red, key_="sphere");

% A gold cube, rotated and placed beside the sphere
cube = mi.shape.cube( ...
    bsdf=gold, key_="cube", ...
    to_world=mi.Transform.translate([1.2 -0.25 0.3]) ...
           * mi.Transform.rotateY(30) ...
           * mi.Transform.scale(0.5));

% Ground plane
ground = mi.shape.groundPlane(height=-0.5, size=20, ...
    reflectance=[0.7 0.7 0.72]);

% Back wall — gives the scene depth and catches light
wall = mi.shape.rectangle(bsdf=white, key_="wall", ...
    to_world=mi.Transform.translate([0 0 -3]) ...
           * mi.Transform.scale(10));

%% 4. Add lighting — an area light above the scene
lightShape = mi.shape.rectangle(key_="light", ...
    emitter=mi.emitter.area(radiance=mi.rgb([20 20 18])), ...
    to_world=mi.Transform.translate([0 3 1]) ...
           * mi.Transform.rotateX(90) ...
           * mi.Transform.scale(1.5));

%% 5. Set up the camera
camera = mi.sensor.perspective( ...
    fov=40, ...
    to_world=mi.Transform.lookAt([3 2 3], [0.3 -0.1 0], [0 1 0]), ...
    film=mi.film(Width=512, Height=384));

%% 6. Build the scene
scene = mi.Scene.build( ...
    sphere, cube, ground, wall, lightShape, ...
    camera, mi.integrator.path(max_depth=6));

%% 7. Render
fprintf("Rendering...\n");
tic;
hdr = scene.render(SamplesPerPixel=64);
fprintf("Done in %.1f s\n", toc);

%% 8. Post-process and display
rgb = mi.postprocess(hdr);
figure("Name", "Getting Started — MATSUBA");
imshow(rgb);
title("First render with MATSUBA");

%% 9. Save output
outPath = fullfile(fileparts(mfilename("fullpath")), "getting_started.png");
imwrite(rgb, outPath);
fprintf("Saved to %s\n", outPath);

delete(scene);
