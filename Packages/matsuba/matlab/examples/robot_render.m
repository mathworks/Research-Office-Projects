%% Photorealistic Robot Arm Render — MATSUBA Example
% Loads a Fanuc M-16iB robot from the Robotics System Toolbox, extracts
% link meshes, and renders the arm with metallic/plastic materials using
% Mitsuba 3. Displays a side-by-side comparison against MATLAB's show().
%
% Requires: Robotics System Toolbox
%
% See also: loadrobot, mi.shape.fromMesh, mi.bsdf.plastic, mi.bsdf.roughconductor,
%           mi.shape.groundPlane, mi.lighting.threePoint

%% 1. Setup
addpath(fullfile(fileparts(mfilename("fullpath")), ".."));
mi.setup;

%% 2. Load robot and extract meshes
fprintf("Loading robot model...\n");
robot = loadrobot("fanucM16iB", DataFormat="column");

fig = figure(Visible="off");
ax = show(robot, Visuals="on", Collisions="off");
patches = findobj(ax, "Type", "Patch");

% Classify patches by color type — skip coordinate frame indicators
% (per-face colored patches are axis visualization, not geometry)
meshes = struct("V", {}, "F", {}, "color", {});
for i = 1:numel(patches)
    p = patches(i);
    fc = p.FaceColor;
    if ischar(fc) || isstring(fc)
        continue   % "flat" — coordinate frame indicator, skip
    end
    V = double(p.Vertices);
    F = double(p.Faces);
    F = F(:, [1 3 2]);   % flip winding — show() patches have inward normals
    meshes(end+1).V = V;  
    meshes(end).F = F;
    meshes(end).color = double(fc);
end
close(fig);
fprintf("  Extracted %d mesh parts\n", numel(meshes));

%% 3. Build Mitsuba scene
fprintf("Building scene...\n");
% scene = buildScene(meshes);

% %% 4. Render
% fprintf("Rendering...\n");
% spp = 64;
% hdr = scene.renderProgressive(SamplesPerPixel=spp, PassSize=32);
% rgb = mi.postprocess(hdr);
% fprintf("Render complete.\n");

% 
% %% 5. Display — side-by-side comparison
% fig = figure("Name", "Robot Arm Render", Position=[100 100 960 440]);
% 
% % Left: MATLAB built-in show()
% ax1 = subplot(1,2,1);
% robot2 = loadrobot("fanucM16iB", DataFormat="column");
% show(robot2, Parent=ax1, Visuals="on", Collisions="off");
% % Match the Mitsuba camera angle
% campos(ax1, [2.6 -2.4 1.0]);
% camtarget(ax1, [0.3 0.05 0.6]);
% camup(ax1, [0 0 1]);
% camva(ax1, 35);
% axis(ax1, "equal", "off");
% title(ax1, "MATLAB show()");
% 
% % Right: Mitsuba path-traced render
% ax2 = subplot(1,2,2);
% imshow(rgb, Parent=ax2);
% title(ax2, "Mitsuba 3 — Photorealistic");
% 
% %% 6. Save output
% outPath = fullfile(fileparts(mfilename("fullpath")), "robot_render.png");
% exportgraphics(fig, outPath, Resolution=150);
% fprintf("Saved to %s\n", outPath);
% 
% delete(scene);
% fprintf("Done!\n");

%% 7. Interactive viewer (optional)
% Uncomment to launch the progressive viewer with camera interactivity.
% Rotate/zoom/pan the figure — the Mitsuba overlay re-renders automatically.
%
  scene2 = buildScene(meshes);
  v = mi.show(scene2, TargetSpp=64);

%% --- Local functions ---

function scene = buildScene(meshes)
%BUILDSCENE Construct the Mitsuba scene from extracted robot meshes.

    % Classify meshes by color and assign materials
    % Yellow: [0.96 0.76 0.13] — Fanuc signature yellow body panels
    % Dark:   [0.12-0.15 ...] — joint housings / base
    shapes = {};
    for i = 1:numel(meshes)
        V = meshes(i).V;
        F = meshes(i).F;
        c = meshes(i).color;

        brightness = mean(c);
        if brightness > 0.4
            % Yellow body — industrial painted plastic with clearcoat
            mat = mi.bsdf.custom("principled", ...
                "base_color", mi.rgb([0.9 0.58 0.01]), ...
                "roughness", 0.25, "specular", 0.6, ...
                "clearcoat", 0.8, "clearcoat_gloss", 0.9);
        else
            % Dark joint/base — dark charcoal plastic
            mat = mi.bsdf.custom("principled", ...
                "base_color", mi.rgb([0.04 0.04 0.04]), ...
                "roughness", 0.35, "specular", 0.4);
        end

        key = sprintf("link_%d", i);
        shapes{end+1} = mi.shape.fromMesh(V, F, bsdf=mat, key_=key); %#ok<AGROW>
    end

    % Ground plane — Z-up scene, slight offset to avoid z-fighting
    ground = mi.shape.groundPlane(up="Z", height=-0.005, size=15, ...
        reflectance=[0.6 0.6 0.62]);

    % Studio room — large inverted cube, corners well outside camera FOV
    roomT = mi.Transform.translate([-20 -20 -0.01]) ...
          * mi.Transform.scale([40 40 30]);
    room = mi.shape.cube(flip_normals=true, ...
        bsdf=mi.bsdf.diffuse(reflectance=mi.rgb([0.55 0.55 0.57])), ...
        to_world=roomT, key_="room");

    % Studio lighting — 3-point rig aimed at robot center, Z-up
    lights = mi.lighting.threePoint(target=[0.3 0 0.6], distance=6, ...
        intensity=0.8, up="Z");

    % Camera — three-quarter view, Z-up scene
    camera = mi.sensor.perspective( ...
        fov=35, ...
        to_world=mi.Transform.lookAt([2.6 -2.4 1.0], [0.3 0.05 0.6], [0 0 1]), ...
        film=mi.film(Width=512, Height=512), ...
        key_="sensor");

    integrator = mi.integrator.path(max_depth=6, hide_emitters=true);

    args = [shapes, {ground, room}, lights, {camera, integrator}];
    scene = mi.Scene.build(args{:});
end
