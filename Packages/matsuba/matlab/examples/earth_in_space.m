%% Earth in Space — MATSUBA Example
% Renders a textured Earth sphere with clouds against a Milky Way star
% field. Uses four source textures: daymap color, tangent-space normals,
% specular map, and cloud opacity. Clouds and roughness are composited
% onto the base textures in MATLAB before rendering for noise-free output.
%
% An extended area light (emitting sphere) creates a soft specular sun
% glint on the Atlantic ocean.
%
% Demonstrates: mi.texture.bitmap, mi.bsdf.normalmap, mi.bsdf.principled
% with texture-mapped roughness, mi.emitter.envmap, mi.emitter.area,
% mi.Transform rotations.
%
% See also: mi.texture.bitmap, mi.bsdf.normalmap, mi.emitter.envmap

%% 1. Setup
addpath(fullfile(fileparts(mfilename("fullpath")), ".."));
mi.setup;

%% 2. Prepare textures
assetsDir = fullfile(fileparts(mfilename("fullpath")), "assets");

% Composite clouds onto the daymap (avoids stochastic transparency noise)
daymap = im2double(imread(fullfile(assetsDir, "daytime_earth_texture.png")));
clouds = im2double(imread(fullfile(assetsDir, "clouds.png")));
cloudAlpha = clouds(:,:,1) * 0.7;  % reduce cloud strength for better contrast
composited = daymap .* (1 - cloudAlpha) + cloudAlpha;

% Composite roughness: clouds are matte (0.8), ocean is smooth (from roughness map)
roughMap = im2double(imread(fullfile(assetsDir, "earth_roughness.png")));
roughComposited = roughMap .* (1 - cloudAlpha) + 0.8 * cloudAlpha;

% Write composited textures to temp files
colorPath = fullfile(tempdir, "earth_color.png");
roughPath = fullfile(tempdir, "earth_rough.png");
imwrite(composited, colorPath);
imwrite(roughComposited, roughPath);
cleanup = onCleanup(@() delete(colorPath, roughPath));

%% 3. Build Earth material
earthColor  = mi.texture.bitmap(colorPath);
earthNormal = mi.texture.bitmap(fullfile(assetsDir, "earth_normals.png"), raw=true);
earthRough  = mi.texture.bitmap(roughPath);

% Principled BSDF: composited color + roughness, normal map for detail
innerBsdf = mi.bsdf.principled( ...
    base_color=earthColor, ...
    roughness=earthRough, ...
    specular=0.8, ...
    metallic=0);
earthMat = mi.bsdf.normalmap(normalmap=earthNormal, bsdf=innerBsdf);

%% 4. Build the Earth sphere
% rotateX(-90) corrects Mitsuba's sphere UV pole orientation
% rotateY(150) faces the Americas + Atlantic toward camera
T_earth = mi.Transform.rotateY(150) * mi.Transform.rotateX(-90);
earth = mi.shape.sphere(radius=1, to_world=T_earth, bsdf=earthMat);

%% 5. Lighting
% Milky Way star field for background and subtle ambient fill on dark side
envMap = mi.emitter.envmap( ...
    filename=fullfile(assetsDir, "star_background.png"), ...
    scale=4);
% Extended sun (emitting sphere) for soft specular reflection on the ocean
T_sun = mi.Transform.translate([6.5 3 6.5]) * mi.Transform.scale(0.3);
sun = mi.shape.sphere( ...
    to_world=T_sun, ...
    emitter=mi.emitter.area(radiance=mi.rgb([10000 9700 8900])));

%% 6. Camera
cam = mi.sensor.perspective( ...
    fov=35, ...
    to_world=mi.Transform.lookAt([0 0.3 5], [0 0 0], [0 1 0]), ...
    film=mi.film(Width=512, Height=512));

%% 7. Render
scene = mi.Scene.build(earth, envMap, sun, cam);
fprintf("Rendering Earth in space (512x512, 64 spp)...\n");
img = scene.renderProgressive(SamplesPerPixel=64);
result = mi.postprocess(img);
fprintf("Done.\n");

%% 8. Display and save
figure;
imshow(result);
title("Earth in Space");

outPath = fullfile(fileparts(mfilename("fullpath")), "earth_in_space.png");
imwrite(result, outPath);
fprintf("Saved to %s\n", outPath);
