%% Gallery Scene: Country Kitchen — MATSUBA Example
% Downloads the "Country Kitchen" scene from the Mitsuba 3 gallery and
% renders it. The scene is cached locally after the first download.
%
% Demonstrates: mi.io.gallery, mi.io.downloadScene, mi.Scene.load

%% 1. Setup
addpath(fullfile(fileparts(mfilename("fullpath")), ".."));
mi.setup;
mi.setVariant(mi.bestVariant());

%% 2. Browse available scenes
fprintf("Available gallery scenes:\n");
t = mi.io.gallery();
disp(t);

%% 3. Download the Country Kitchen scene
xmlPath = mi.io.downloadScene("kitchen");

%% 4. Load and render
% Use Scene.load for gallery scenes — they use XML features (cross-refs)
% that require Mitsuba's native parser.
scene = mi.Scene.load(xmlPath);

fprintf("Rendering Country Kitchen (512x512, 256 spp)...\n");
img = scene.renderProgressive(SamplesPerPixel=256, Convergence=true);
result = mi.postprocess(img);
fprintf("Done.\n");

%% 5. Display and save
figure;
imshow(result);
title("Country Kitchen (Mitsuba 3 Gallery)");

outPath = fullfile(fileparts(mfilename("fullpath")), "gallery_kitchen.png");
imwrite(result, outPath);
fprintf("Saved to %s\n", outPath);
