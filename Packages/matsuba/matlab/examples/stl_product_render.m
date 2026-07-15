%% Photorealistic Product Render from STL — MATSUBA Example
% Loads an STL file, applies a brushed-metal material, and renders it with
% studio lighting using Mitsuba 3.
%
% This demonstrates a common engineering workflow: receiving geometry from
% a CAD tool (as STL) and producing a photorealistic product image directly
% in MATLAB — no external rendering software required.
%
% Uses BracketWithHole.stl from PDE Toolbox example data.
%
% See also: stlread, mi.shape.fromMesh, mi.bsdf.roughconductor,
%           mi.shape.groundPlane, mi.lighting.threePoint

%% 1. Setup
addpath(fullfile(fileparts(mfilename("fullpath")), ".."));
mi.setup;

%% 2. Load STL and prepare geometry
fprintf("Loading STL...\n");
stlPath = fullfile(matlabroot, "toolbox", "pde", "pde", "pdedata", "BracketWithHole.stl");
tr = stlread(stlPath);
V = tr.Points;
F = double(tr.ConnectivityList);

% Center the mesh at the origin and normalize to unit scale
center = (max(V) + min(V)) / 2;
V = V - center;
extents = max(V) - min(V);
scaleFactor = 2 / max(extents);   % fit into a ~2-unit bounding box
V = V * scaleFactor;

fprintf("  %d vertices, %d faces\n", size(V,1), size(F,1));

%% 3. Build the scene
scene = buildScene(V, F);

%% 4. Render
fprintf("Rendering...\n");
spp = 64;
hdr = scene.renderProgressive(SamplesPerPixel=spp, PassSize=32);
rgb = mi.postprocess(hdr);
fprintf("Render complete.\n");

%% 5. Display — side-by-side comparison
fig = figure("Name", "STL Product Render", Position=[100 100 900 420]);

% Left: MATLAB built-in viewer
ax1 = subplot(1,2,1);
trisurf(F, V(:,1), V(:,2), V(:,3), ...
    FaceColor=[0.75 0.75 0.78], EdgeColor="none", ...
    FaceLighting="gouraud", AmbientStrength=0.4, ...
    DiffuseStrength=0.6, SpecularStrength=0.8);
light(ax1, Position=[3 4.5 2.5]);
light(ax1, Position=[-3 3 1.5], Color=[0.3 0.3 0.35]);
axis(ax1, "equal", "off");
% Match the Mitsuba camera
campos(ax1, [2.8 2.0 2.8]);
camtarget(ax1, [0 -0.1 0]);
camup(ax1, [0 1 0]);
camva(ax1, 35);
title(ax1, "MATLAB trisurf");
set(ax1, Color=[0.88 0.88 0.90]);

% Right: Mitsuba path-traced render
ax2 = subplot(1,2,2);
imshow(rgb, Parent=ax2);
title(ax2, "Mitsuba 3 — Brushed Chrome");

%% 6. Save output
outPath = fullfile(fileparts(mfilename("fullpath")), "bracket_render.png");
exportgraphics(fig, outPath, Resolution=150);
fprintf("Saved to %s\n", outPath);

delete(scene);
fprintf("Done!\n");

%% 7. Interactive viewer (optional)
% Uncomment to launch the progressive viewer with camera interactivity.
% Rotate/zoom/pan the figure — the Mitsuba overlay re-renders automatically.
%
%   scene2 = buildScene(V, F);
%   v = mi.show(scene2, TargetSpp=64);

%% --- Local functions ---

function scene = buildScene(V, F)
%BUILDSCENE Construct a studio scene with the bracket.

    % Brushed stainless steel — moderate roughness for soft highlights
    bracketMat = mi.bsdf.roughconductor(material="Cr", alpha=0.15);
    bracketShape = mi.shape.fromMesh(V, F, bsdf=bracketMat, key_="bracket");

    % Infinite floor — large ground plane, no back walls
    ground = mi.shape.groundPlane(height=-1, size=100, ...
        reflectance=[0.6 0.6 0.62]);

    % Studio lighting — 3-point rig aimed at the bracket center
    lights = mi.lighting.threePoint(target=[0 0.2 0], distance=5, ...
        intensity=1.2, temperature=1.1);

    % Camera — three-quarter view, pulled back to center the bracket
    camera = mi.sensor.perspective( ...
        fov=35, ...
        to_world=mi.Transform.lookAt([2.8 2.0 2.8], [0 -0.1 0], [0 1 0]), ...
        film=mi.film(Width=384, Height=384), ...
        key_="sensor");

    integrator = mi.integrator.path(max_depth=6, hide_emitters=true);

    scene = mi.Scene.build( ...
        bracketShape, ground, lights{:}, ...
        camera, integrator);
end

