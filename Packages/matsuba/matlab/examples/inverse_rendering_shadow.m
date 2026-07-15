%% Inverse Rendering: Shadow Art — MATSUBA Example
% Given two target shadow silhouettes — a circle and a square — optimizes
% the vertex positions of a mesh so that it casts those shadows on two
% perpendicular walls. Uses dual proxy scenes with the projective sampling
% integrator (prb_projective) which computes gradients through shadow
% discontinuities. MATLAB's adamupdate (Deep Learning Toolbox) drives the
% optimization. Inspired by Mitsuba's shadow art tutorial.
%
% Requires an AD variant (llvm_ad_rgb or cuda_ad_rgb).
%
% See also: mi.Scene.renderDiff, adamupdate

%% 1. Setup
addpath(fullfile(fileparts(mfilename("fullpath")), ".."));
try terminate(pyenv); catch; end
pause(1);
mi.setup;
mi.setVariant("llvm_ad_rgb");

%% 2. Create mesh and synthetic target silhouettes
% Start from a 12-vertex icosphere. Define the desired shadow shapes
% directly in MATLAB — no rendering needed. Pixel values are matched to
% the proxy scene's constant-emitter rendering (measured empirically).
[Vmesh, Fmesh] = unitIcosphere(0);  % 12 vertices
Vmesh = Vmesh * 0.4;

sz = 64;
[X, Y] = meshgrid(linspace(-1, 1, sz), linspace(-1, 1, sz));
bgVal = [1.205, 0.948, 0.909];  % proxy background (constant emitter)
fgVal = [0.012, 0.010, 0.009];  % proxy foreground (near-black mesh)

% Circle target
circR = 0.35;
circMask = (X.^2 + Y.^2) < circR^2;
circTarget = zeros(sz, sz, 3);
for c = 1:3
    circTarget(:,:,c) = bgVal(c) * (~circMask) + fgVal(c) * circMask;
end

% Square target
squareMask = (abs(X) < 0.28) & (abs(Y) < 0.28);
squareTarget = zeros(sz, sz, 3);
for c = 1:3
    squareTarget(:,:,c) = bgVal(c) * (~squareMask) + fgVal(c) * squareMask;
end
fprintf("Target silhouettes: circle (%.0f%% px) + square (%.0f%% px)\n", ...
    100*mean(circMask(:)), 100*mean(squareMask(:)));

%% 3. Build dual proxy scenes for optimization
% Two proxy scenes with cameras aligned to the light positions: one views
% the mesh from the front (controls back wall shadow), the other from the
% side (controls side wall shadow). Near-black mesh minimizes indirect
% illumination so the synthetic targets match the rendered pixel values.
darkFg = [0.01, 0.01, 0.01];

[proxyFront, proxySide] = buildProxyScenes(Vmesh, Fmesh, darkFg, sz);
vpInit = proxyFront.getParam("shape.vertex_positions");

%% 4. Optimize vertex positions with adamupdate (dual-view)
% Restarts Python every restartEvery iterations to flush leaked Dr.Jit LLVM
% kernels from prb_projective, then rebuilds scenes and restores state.
lr = 0.012;
spp = 2;
nIters = 50;
restartEvery = 10;
checkIters = [1 10 20 30 40];

vpCur    = dlarray(vpInit);
avgGrad  = dlarray(zeros(size(vpInit)));
avgSqGrad = dlarray(zeros(size(vpInit)));

losses = [];  lossesFront = [];  lossesSide = [];
vpCheckpoints = {};

fprintf("Optimizing %d vertex positions (dual-proxy, spp=%d)...\n", ...
    numel(vpInit)/3, spp);
iter = 1;
nRetries = 0;
while iter <= nIters
    % Periodic or recovery restart to reclaim leaked Dr.Jit memory
    needsRestart = nRetries > 0 || (iter > 1 && mod(iter-1, restartEvery) == 0);
    if needsRestart
        try delete(proxyFront); catch; end
        try delete(proxySide); catch; end
        try terminate(pyenv); catch; end
        pause(3);
        mi.setup;
        mi.setVariant("llvm_ad_rgb");
        [proxyFront, proxySide] = buildProxyScenes(Vmesh, Fmesh, darkFg, sz);
        vpRestore = extractdata(vpCur);
        proxyFront.setParam("shape.vertex_positions", vpRestore);
        proxySide.setParam("shape.vertex_positions", vpRestore);
        fprintf("  [restart Python at iter %d]\n", iter);
    end

    try
        [~, lF, gF] = proxyFront.renderDiff(circTarget, ...
            "shape.vertex_positions", SamplesPerPixel=spp, Seed=iter);
        [~, lS, gS] = proxySide.renderDiff(squareTarget, ...
            "shape.vertex_positions", SamplesPerPixel=spp, Seed=iter+1000);

        grad = dlarray(gF("shape.vertex_positions") + gS("shape.vertex_positions"));
        [vpCur, avgGrad, avgSqGrad] = adamupdate(vpCur, grad, ...
            avgGrad, avgSqGrad, iter, lr);

        vpDouble = extractdata(vpCur);
        proxyFront.setParam("shape.vertex_positions", vpDouble);
        proxySide.setParam("shape.vertex_positions", vpDouble);

        losses(end+1) = (lF + lS) / 2; %#ok<SAGROW>
        lossesFront(end+1) = lF; %#ok<SAGROW>
        lossesSide(end+1) = lS; %#ok<SAGROW>
        nRetries = 0;

        if ismember(iter, checkIters)
            vpCheckpoints{end+1} = struct("iter", iter, "vp", vpDouble); %#ok<SAGROW>
        end
        if mod(iter, 5) == 0 || iter == 1
            fprintf("  iter %2d: loss=%.6f (front=%.4f side=%.4f)\n", ...
                iter, (lF+lS)/2, lF, lS);
        end
        iter = iter + 1;
    catch me
        nRetries = nRetries + 1;
        if nRetries <= 3
            fprintf("  iter %d failed, restarting Python (retry %d)...\n", iter, nRetries);
        else
            fprintf("  Giving up at iter %d after %d retries: %s\n", iter, nRetries, me.message);
            vpCheckpoints{end+1} = struct("iter", iter, "vp", extractdata(vpCur)); %#ok<SAGROW>
            break;
        end
    end
end
finalVp = extractdata(vpCur);
nCompleted = numel(losses);
fprintf("Completed %d iterations. Loss: %.6f -> %.6f\n", ...
    nCompleted, losses(1), losses(end));
try delete(proxyFront); catch; end
try delete(proxySide); catch; end

%% 5. Restart renderer and render wall-and-shadow display images
% The prb_projective integrator leaks Dr.Jit LLVM kernels. Restart Python
% to reclaim memory before rendering the display scene with path tracing.
fprintf("Restarting renderer for display renders...\n");
try terminate(pyenv); catch; end
pause(2);
mi.setup;
mi.setVariant("llvm_ad_rgb");

displayScene = buildShadowScene(Vmesh, Fmesh);

% Render checkpoints — lift mesh so shadows separate from floor
yOff = 0.2;
snapImages = {};
snapLabels = {};
for i = 1:numel(vpCheckpoints)
    cp = vpCheckpoints{i};
    vpDisp = cp.vp;  vpDisp(2:3:end) = vpDisp(2:3:end) + yOff;
    displayScene.setParam("shape.vertex_positions", vpDisp);
    hdr = displayScene.render(SamplesPerPixel=64);
    snapImages{end+1} = mi.postprocess(hdr); %#ok<SAGROW>
    snapLabels{end+1} = sprintf("Iter %d", cp.iter); %#ok<SAGROW>
    fprintf("  Iter %d rendered.\n", cp.iter);
end

% Render final
finalVpDisp = finalVp;  finalVpDisp(2:3:end) = finalVpDisp(2:3:end) + yOff;
displayScene.setParam("shape.vertex_positions", finalVpDisp);
hdr = displayScene.render(SamplesPerPixel=64);
finalShadowImg = mi.postprocess(hdr);
fprintf("  Optimized result rendered.\n");

delete(displayScene);

%% 6. Display
circDisp = mi.postprocess(circTarget, Denoise="none");
squareDisp = mi.postprocess(squareTarget, Denoise="none");

allImgs = [{circDisp}, {squareDisp}, snapImages, {finalShadowImg}];
allTitles = ["Target: Circle", "Target: Square", string(snapLabels), "Optimized"];
nP = numel(allImgs);

fig = figure("Name", "Inverse Rendering: Shadow Art", ...
    Position=[50 50 1400 550], Color="k");

for i = 1:nP
    ax = subplot(2, nP, i);
    imshow(allImgs{i}, Parent=ax);
    title(ax, allTitles(i), Color="w", FontSize=11);
end

% Per-view loss
ax = subplot(2, nP, nP+1);
hold(ax, "on");
plot(1:nCompleted, lossesFront, "c-", LineWidth=1.5);
plot(1:nCompleted, lossesSide, "m-", LineWidth=1.5);
xlabel("Iteration", Color="w"); ylabel("MSE Loss", Color="w");
title(ax, "Per-view Loss", Color="w", FontSize=11);
legend(ax, ["Front (circle)", "Side (square)"], TextColor="w", ...
    Color=[0.2 0.2 0.2], Location="best");
set(ax, Color=[0.1 0.1 0.1], XColor="w", YColor="w");
grid(ax, "on"); ax.GridColor = [0.3 0.3 0.3];

% Combined loss
ax2 = subplot(2, nP, nP+2);
plot(1:nCompleted, losses, "y-", LineWidth=1.5);
xlabel("Iteration", Color="w"); ylabel("Combined Loss", Color="w");
title(ax2, "Total Loss", Color="w", FontSize=11);
set(ax2, Color=[0.1 0.1 0.1], XColor="w", YColor="w");
grid(ax2, "on"); ax2.GridColor = [0.3 0.3 0.3];

drawnow;

%% 7. Save
outPath = fullfile(fileparts(mfilename("fullpath")), "inverse_rendering_shadow.png");
exportgraphics(fig, outPath, Resolution=150, BackgroundColor="k");
fprintf("Saved: %s\n", outPath);
fprintf("Done!\n");


%% =====================================================================
%  Local functions
%  =====================================================================

function [front, side] = buildProxyScenes(V, F, darkFg, sz)
%BUILDPROXYSCENES Build the two proxy scenes for dual-view optimization.
    s_proj = struct("type", "prb_projective", "category_", "integrator", ...
        "max_depth", int32(2));
    front = mi.Scene.build( ...
        mi.shape.fromMesh(V, F, face_normals=true, ...
            bsdf=mi.bsdf.diffuse(reflectance=mi.rgb(darkFg)), key_="shape"), ...
        mi.emitter.constant(radiance=mi.spectrum(value=1.0)), ...
        mi.sensor.perspective(fov=50, ...
            to_world=mi.Transform.lookAt([0 0 2.5], [0 0 0], [0 1 0]), ...
            film=mi.film(Width=sz, Height=sz)), ...
        s_proj);
    side = mi.Scene.build( ...
        mi.shape.fromMesh(V, F, face_normals=true, ...
            bsdf=mi.bsdf.diffuse(reflectance=mi.rgb(darkFg)), key_="shape"), ...
        mi.emitter.constant(radiance=mi.spectrum(value=1.0)), ...
        mi.sensor.perspective(fov=50, ...
            to_world=mi.Transform.lookAt([2.5 0 0], [0 0 0], [0 1 0]), ...
            film=mi.film(Width=sz, Height=sz)), ...
        s_proj);
end

function scene = buildShadowScene(V, F)
%BUILDSHADOWSCENE Build a wall-and-shadow scene for display rendering.
%   Two perpendicular walls, a floor, two point lights (one per wall),
%   and the mesh occluder. Uses the path integrator for stable rendering.

    white = mi.bsdf.diffuse(reflectance=mi.rgb([0.85 0.85 0.85]));
    gray  = mi.bsdf.diffuse(reflectance=mi.rgb([0.6 0.6 0.55]));

    backWall = mi.shape.rectangle(bsdf=white, key_="back_wall", ...
        to_world=mi.Transform.translate([0, 0, -1.5]) ...
               * mi.Transform.scale([2 2 1]));

    sideWall = mi.shape.rectangle(bsdf=white, key_="side_wall", ...
        to_world=mi.Transform.translate([-1.5, 0, 0]) ...
               * rotY(90) * mi.Transform.scale([2 2 1]));

    floor_ = mi.shape.rectangle(bsdf=gray, key_="floor", ...
        to_world=mi.Transform.translate([0, -1, 0]) ...
               * rotX(-90) * mi.Transform.scale([2 2 1]));

    mesh_ = mi.shape.fromMesh(V, F, face_normals=true, ...
        bsdf=mi.bsdf.diffuse(reflectance=mi.rgb([0.5 0.35 0.25])), ...
        key_="shape");

    light1 = mi.emitter.point(position=[0.3, 0.3, 2.0], ...
        intensity=mi.spectrum(value=8));
    light2 = mi.emitter.point(position=[2.0, 0.3, 0.3], ...
        intensity=mi.spectrum(value=8));
    ambient = mi.emitter.constant(radiance=mi.spectrum(value=0.02));

    camera = mi.sensor.perspective(fov=55, ...
        to_world=mi.Transform.lookAt([2, 0.8, 2.5], [-0.2, 0.15, -0.3], [0, 1, 0]), ...
        film=mi.film(Width=256, Height=256));

    scene = mi.Scene.build(backWall, sideWall, floor_, mesh_, ...
        light1, light2, ambient, camera, mi.integrator.path(max_depth=3));
end

function [V, F] = unitIcosphere(subdivisions)
%UNITICOSPHERE Generate a unit icosphere mesh.
    t = (1 + sqrt(5)) / 2;
    verts = [ -1  t  0;  1  t  0; -1 -t  0;  1 -t  0;
               0 -1  t;  0  1  t;  0 -1 -t;  0  1 -t;
               t  0 -1;  t  0  1; -t  0 -1; -t  0  1];
    verts = verts ./ vecnorm(verts, 2, 2);
    faces = [
         1 12  6;  1  6  2;  1  2  8;  1  8 11;  1 11 12;
         2  6 10;  6 12  5;  12 11  3; 11  8  7;  8  2  9;
         4 10  5;  4  5  3;  4  3  7;  4  7  9;  4  9 10;
         5 10  6;  3  5 12;  7  3 11;  9  7  8; 10  9  2];
    for s = 1:subdivisions
        newFaces = zeros(size(faces,1)*4, 3);
        edgeMap = containers.Map("KeyType", "char", "ValueType", "double");
        fi = 0;
        for f = 1:size(faces,1)
            tri = faces(f,:);
            mids = zeros(1,3);
            edges = [tri(1) tri(2); tri(2) tri(3); tri(3) tri(1)];
            for e = 1:3
                eKey = sprintf("%d-%d", min(edges(e,:)), max(edges(e,:)));
                if edgeMap.isKey(eKey)
                    mids(e) = edgeMap(eKey);
                else
                    midPt = (verts(edges(e,1),:) + verts(edges(e,2),:)) / 2;
                    midPt = midPt / norm(midPt);
                    verts(end+1,:) = midPt; %#ok<AGROW>
                    mids(e) = size(verts,1);
                    edgeMap(eKey) = mids(e);
                end
            end
            fi = fi + 1; newFaces(fi,:) = [tri(1) mids(1) mids(3)];
            fi = fi + 1; newFaces(fi,:) = [tri(2) mids(2) mids(1)];
            fi = fi + 1; newFaces(fi,:) = [tri(3) mids(3) mids(2)];
            fi = fi + 1; newFaces(fi,:) = [mids(1) mids(2) mids(3)];
        end
        faces = newFaces;
    end
    V = verts;
    F = faces;
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
