%% Inverse Rendering: Color Recovery — MATSUBA Example
% Recovers the original colors of three spheres from randomized starting
% colors using gradient-based optimization. The Mitsuba PRB integrator
% provides material-property gradients, and MATLAB's fmincon (Optimization
% Toolbox) drives the optimization with box constraints [0, 1].
%
% Requires an AD variant (llvm_ad_rgb or cuda_ad_rgb).
%
% See also: mi.Scene.renderDiff, mi.integrator.prb, fmincon

%% 1. Setup
addpath(fullfile(fileparts(mfilename("fullpath")), ".."));
mi.setup;
mi.setVariant("llvm_ad_rgb");

%% 2. Build scene with three colored spheres
paramNames = ["red.bsdf.reflectance.value"; ...
              "green.bsdf.reflectance.value"; ...
              "blue.bsdf.reflectance.value"];

targetColors = [0.65 0.05 0.05; ...   % red
                0.12 0.45 0.05; ...   % green
                0.05 0.1  0.65];      % blue

scene = mi.Scene.build( ...
    mi.shape.sphere(radius=0.45, ...
        to_world=mi.Transform.translate([-0.55 0 0.1]), ...
        bsdf=mi.bsdf.diffuse(reflectance=mi.rgb(targetColors(1,:)), key_="red"), ...
        key_="red"), ...
    mi.shape.sphere(radius=0.45, ...
        to_world=mi.Transform.translate([0 0 -0.3]), ...
        bsdf=mi.bsdf.diffuse(reflectance=mi.rgb(targetColors(2,:)), key_="green"), ...
        key_="green"), ...
    mi.shape.sphere(radius=0.45, ...
        to_world=mi.Transform.translate([0.55 0 0.2]), ...
        bsdf=mi.bsdf.diffuse(reflectance=mi.rgb(targetColors(3,:)), key_="blue"), ...
        key_="blue"), ...
    mi.emitter.point(position=[1 3 2], intensity=mi.spectrum(value=15)), ...
    mi.emitter.constant(radiance=mi.spectrum(value=0.15)), ...
    mi.sensor.perspective(fov=50, ...
        to_world=mi.Transform.lookAt([0 0.5 2.5], [0 0 0], [0 1 0]), ...
        film=mi.film(Width=128, Height=128)), ...
    mi.integrator.prb(max_depth=3));

%% 3. Render target
fprintf("Rendering target...\n");
targetHdr = scene.render(SamplesPerPixel=64);
targetImg = mi.postprocess(targetHdr, Denoise="none");

%% 4. Randomize colors (all gray)
x0 = repmat([0.3 0.3 0.3], 1, 3)';  % 9x1 vector
for i = 1:3
    scene.setParam(paramNames(i), [0.3 0.3 0.3]);
end
initHdr = scene.render(SamplesPerPixel=32);
initImg = mi.postprocess(initHdr, Denoise="none");

%% 5. Optimize with fmincon
spp = 8;

% Track progress via OutputFcn (containers.Map is a handle class)
tracker = containers.Map();
tracker("losses") = [];
tracker("colors") = [];

objFun = @(x) colorObjective(x, scene, targetHdr, paramNames, spp);
outFcn = @(x, ov, s) trackProgress(x, ov, s, tracker);

options = optimoptions("fmincon", ...
    Algorithm="sqp", ...
    SpecifyObjectiveGradient=true, ...
    MaxIterations=60, ...
    MaxFunctionEvaluations=200, ...
    Display="iter", ...
    OutputFcn=outFcn, ...
    OptimalityTolerance=1e-8, ...
    StepTolerance=1e-8);

lb = zeros(9, 1);
ub = ones(9, 1);

fprintf("Optimizing 9 color parameters with fmincon (SQP)...\n");
[xOpt, fval] = fmincon(objFun, x0, [], [], [], [], lb, ub, [], options);
fprintf("Final loss = %.6f\n", fval);

losses = tracker("losses");
colorHistory = tracker("colors");
nIters = size(colorHistory, 1);

%% 6. Render snapshots at key iterations
snapIters = unique([1, round(nIters*0.25), round(nIters*0.5), nIters]);
snapshots = {};
for k = 1:numel(snapIters)
    idx = snapIters(k);
    for i = 1:3
        scene.setParam(paramNames(i), colorHistory(idx, 3*(i-1)+1:3*i));
    end
    hdr = scene.render(SamplesPerPixel=32);
    snapshots{k} = mi.postprocess(hdr, Denoise="none"); %#ok<SAGROW>
end

%% 7. Display
fig = figure("Name", "Inverse Rendering: Color Recovery", ...
    Position=[50 100 1200 500], Color="k");

titles1 = ["Target", "Initial (gray)", compose("Iter %d", snapIters)];
images1 = [{targetImg}, {initImg}, snapshots];
nPanels = numel(images1);
for i = 1:nPanels
    ax = subplot(2, nPanels, i);
    imshow(images1{i}, Parent=ax);
    title(ax, titles1(i), Color="w", FontSize=10);
end

ax = subplot(2, nPanels, nPanels+1);
semilogy(1:numel(losses), losses, "c-", LineWidth=1.5);
xlabel("Iteration", Color="w"); ylabel("MSE Loss", Color="w");
title(ax, "Convergence", Color="w", FontSize=10);
set(ax, Color=[0.1 0.1 0.1], XColor="w", YColor="w");

ax2 = subplot(2, nPanels, nPanels+2);
hold(ax2, "on");
plotColors = {"r", [0 0.7 0], "b"};
labels = ["Red", "Green", "Blue"];
for i = 1:3
    dominant = find(targetColors(i,:) == max(targetColors(i,:)), 1);
    col = 3*(i-1) + dominant;
    plot(ax2, 1:nIters, colorHistory(:, col), "-", Color=plotColors{i}, LineWidth=1.5);
    yline(ax2, targetColors(i, dominant), "--", Color=plotColors{i}, Alpha=0.5);
end
xlabel("Iteration", Color="w"); ylabel("Dominant channel", Color="w");
title(ax2, "Color Convergence", Color="w", FontSize=10);
set(ax2, Color=[0.1 0.1 0.1], XColor="w", YColor="w");
legend(ax2, labels, TextColor="w", Location="best", Color=[0.2 0.2 0.2]);
ylim(ax2, [0 0.9]);

delete(scene);
drawnow;

%% 8. Save
outPath = fullfile(fileparts(mfilename("fullpath")), "inverse_rendering_colors.png");
exportgraphics(fig, outPath, Resolution=150, BackgroundColor="k");
fprintf("Saved: %s\n", outPath);
fprintf("Done!\n");


%% =====================================================================
%  Local functions
%  =====================================================================

function [loss, grad] = colorObjective(x, scene, targetHdr, paramNames, spp)
%COLOROBJECTIVE Objective function for fmincon: render, compute MSE + gradient.
    for i = 1:3
        scene.setParam(paramNames(i), x(3*(i-1)+1 : 3*i)');
    end
    [~, loss, grads] = scene.renderDiff(targetHdr, paramNames, ...
        SamplesPerPixel=spp, Seed=randi(1e6));
    grad = zeros(9, 1);
    for i = 1:3
        grad(3*(i-1)+1 : 3*i) = grads(paramNames(i));
    end
end

function stop = trackProgress(x, optimValues, state, tracker)
%TRACKPROGRESS OutputFcn for fmincon — records losses and colors.
    if strcmp(state, "iter") || strcmp(state, "init")
        tracker("losses") = [tracker("losses"); optimValues.fval];
        tracker("colors") = [tracker("colors"); x(:)'];
    end
    stop = false;
end
