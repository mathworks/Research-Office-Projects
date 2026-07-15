%% Nighttime Earth — MATSUBA Example
% Renders a nighttime Earth with glowing city lights on the dark side, a
% warm sun just peeking over the planet's limb, and a physically-based
% atmosphere with Rayleigh scattering that produces a blue ring. The
% Americas are visible in darkness with city lights; a thin daylit crescent
% is visible on the sunlit side.
%
% The city lights use the nightmap as an emissive texture on an area
% emitter. The daymap is used as the surface albedo so the sunlit crescent
% shows actual earth features. The atmosphere is a homogeneous
% participating medium with wavelength-dependent scattering (blue >> red)
% and a Rayleigh phase function, rendered with the volpath integrator.
%
% The sun is placed at realistic distance with ~2x angular size for
% visual impact.
%
% Demonstrates: textured area emitter, participating media with Rayleigh
% scattering, volpath integrator, composited daymap/nightmap textures,
% luminance-based Reinhard tonemapping.
%
% See also: earth_in_space, mi.emitter.area, mi.texture.bitmap

%% 1. Setup
addpath(fullfile(fileparts(mfilename("fullpath")), ".."));
mi.setup;

%% 2. Prepare textures
assetsDir = fullfile(fileparts(mfilename("fullpath")), "assets");

% Composite daymap with clouds for surface albedo (same as earth_in_space)
daymap = im2double(imread(fullfile(assetsDir, "daytime_earth_texture.png")));
clouds = im2double(imread(fullfile(assetsDir, "clouds.png")));
cloudAlpha = clouds(:,:,1) * 0.7;
composited = daymap .* (1 - cloudAlpha) + cloudAlpha;
colorPath = fullfile(tempdir, "earth_color.png");
imwrite(composited, colorPath);

% Prepare nightmap: suppress dark ocean areas, boost city lights, save HDR
nightmap = im2double(imread(fullfile(assetsDir, "nighttime_earth_emission.png")));
lum = 0.2126*nightmap(:,:,1) + 0.7152*nightmap(:,:,2) + 0.0722*nightmap(:,:,3);
mask = max(0, min(1, (lum - 0.04) / 0.02));
mask = imgaussfilt(mask, 2);
nightCleaned = nightmap .* mask;
nightCleaned(:,:,1) = nightCleaned(:,:,1) * 8;   % warm city light tint
nightCleaned(:,:,2) = nightCleaned(:,:,2) * 6;
nightCleaned(:,:,3) = nightCleaned(:,:,3) * 3;
nightExrPath = fullfile(tempdir, "earth_night_emissive.exr");
exrwrite(single(nightCleaned), nightExrPath);

%% 3. Build Earth
% Daymap albedo for the sunlit crescent + nightmap emission for city lights.
% rotateY(160) faces Americas toward camera with daylit crescent on the right.
earthColor = mi.texture.bitmap(colorPath);
earthBsdf = mi.bsdf.principled( ...
    base_color=earthColor, roughness=0.6, specular=0.3);

nightTex = mi.texture.bitmap(nightExrPath, raw=true);
earthEmitter = mi.emitter.area(radiance=nightTex);

T_earth = mi.Transform.rotateY(160) * mi.Transform.rotateX(-90);
earth = mi.shape.sphere(radius=1, to_world=T_earth, ...
    bsdf=earthBsdf, emitter=earthEmitter);

%% 4. Rayleigh atmosphere
% Homogeneous participating medium with wavelength-dependent extinction.
% Blue scatters ~24x more than red (exaggerated Rayleigh), producing a
% blue ring at the limb and warm transmitted light near the sun.
rayleighPhase = struct('type', 'rayleigh', 'category_', 'phase');
atmosMedium = struct( ...
    'type', 'homogeneous', ...
    'category_', 'medium', ...
    'sigma_t', mi.rgb([0.04 0.25 0.95]), ...
    'albedo', mi.rgb([0.99 0.99 0.99]), ...
    'phase', rayleighPhase);

T_atmos = T_earth * mi.Transform.scale(1.05);
atmosBsdf = struct('type', 'null', 'category_', 'bsdf');
atmosShape = struct( ...
    'type', 'sphere', ...
    'category_', 'shape', ...
    'radius', 1, ...
    'to_world', T_atmos, ...
    'interior', atmosMedium, ...
    'bsdf', atmosBsdf);

%% 5. Lighting
% Sun at realistic distance with correct angular size (~0.53 degrees).
% Radiance boosted to compensate for inverse-square falloff at distance 30.
sunDist = 30;
sunRad = sunDist * tand(0.265) * 2;  % 2x angular size for visual impact
sunRadiance = struct('type', 'rgb', 'category_', 'color', ...
    'value', [187500, 150000, 75000]);
T_sun = mi.Transform.translate([15, 0.2, -sunDist]) * mi.Transform.scale(sunRad);
sun = struct('type', 'sphere', 'category_', 'shape', ...
    'to_world', T_sun, ...
    'emitter', struct('type', 'area', 'category_', 'emitter', ...
        'radiance', sunRadiance));

% Star field with subtle ambient fill to reveal continent outlines
envMap = mi.emitter.envmap( ...
    filename=fullfile(assetsDir, "star_background.png"), scale=0.7);

%% 6. Camera
cam = mi.sensor.perspective( ...
    fov=45, ...
    to_world=mi.Transform.lookAt([0 0.5 4.5], [0.4 -0.1 0], [0 1 0]), ...
    film=mi.film(Width=768, Height=512));

%% 7. Render (volpath for participating media)
integrator = mi.integrator.custom("volpath", "max_depth", int32(8));
scene = mi.Scene.build(earth, atmosShape, sun, envMap, cam, integrator);
fprintf("Rendering nighttime Earth (768x512, 512 spp, volpath)...\n");
img = scene.renderProgressive(SamplesPerPixel=512, Convergence=true);
fprintf("Done.\n");

%% 8. Post-process (luminance-based Reinhard tonemapping)
% Preserves color ratios in bright highlights (sun, atmosphere glow)
hdr = min(img, prctile(img(:), 99.9));  % firefly clamp
hdr = hdr * 2.0;                         % exposure boost
lum = 0.2126*hdr(:,:,1) + 0.7152*hdr(:,:,2) + 0.0722*hdr(:,:,3);
lumTM = lum ./ (1 + lum);
scale = lumTM ./ max(lum, 1e-8);
result = max(0, min(1, (hdr .* scale) .^ (1/2.2)));

%% 9. Display and save
figure;
imshow(result);
title("Nighttime Earth");

outPath = fullfile(fileparts(mfilename("fullpath")), "nighttime_earth.png");
imwrite(result, outPath);
fprintf("Saved to %s\n", outPath);
