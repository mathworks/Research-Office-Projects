function [mesh, info] = pc2mplicitmesh(ptCloud, options)
%PC2MPLICITMESH Surface mesh from an oriented point cloud via M-plicits.
%
%   mesh = pc2mplicitmesh(ptCloud) fits a multiscale neural implicit
%   surface (M-plicits) to the points and normals of ptCloud and returns
%   its zero level set as a surfaceMesh, in the coordinates of ptCloud.
%   ptCloud is a pointCloud object with a Normal property; the normals
%   must point OUT of the shape.
%
%   The surface is a sum of sinusoidal neural networks (SIRENs),
%       F = F0 + R1 + R2,
%   trained coarse to fine: F0 is a smooth coarse signed distance field,
%   and each residual Rk is a higher-frequency network trained, with the
%   coarser levels frozen, only in a band around the surface to add the
%   detail they miss.
%
%   [mesh, info] = pc2mplicitmesh(ptCloud) also returns a struct with
%       Field          - function handle to evaluate the trained field at
%                        N-by-3 points in the coordinates of ptCloud:
%                        F = info.Field(X) is the full field (approximate
%                        signed distance, negative inside), and
%                        F = info.Field(X, k) uses only the first k levels.
%                        The field is fitted only inside the cube
%                        Center +/- Scale; outside it, it is meaningless.
%       BandWidths     - half-width of the band around the surface in which
%                        each residual level was trained (one per residual,
%                        in the units of ptCloud)
%       Model          - the trained field, in normalized coordinates
%       Center, Scale  - normalization: x = xNormalized * Scale + Center
%       UsedGPU        - true when training ran on the GPU
%       TrainTime      - training time in seconds
%       MeshTime       - field evaluation and meshing time in seconds
%       NumRemoved     - small components removed (see MinComponentFraction)
%
%   Name-value options:
%       NumLevels            - levels to train, 1 to 3 (default 3). Fewer
%                              levels are faster but smoother.
%       Resolution           - meshing grid samples per axis (default 256)
%       MinComponentFraction - remove connected components with less than
%                              this fraction of the total surface area
%                              (default 0.01; 0 keeps everything)
%       UseGPU               - train on the GPU when one is available
%                              (default true)
%       Verbose              - print training progress (default true)
%       Levels               - advanced: struct array of per-level
%                              settings with fields HiddenSizes, Omega0
%                              and NumIterations (default: the M-plicits
%                              schedule [128 128]/30, [256 256]/45,
%                              [400 400]/100, with 3500/750/750
%                              iterations)
%
%   Training is designed for the GPU. On the CPU it gives the same result
%   but a full 3-level fit takes on the order of an hour.
%
%   Example:
%       ptCloud = pcread("teapot.ply");
%       ptCloud = pcdownsample(ptCloud, "gridAverage", 0.05);
%       ptCloud.Normal = pcnormals(ptCloud);   % check the orientation!
%       mesh = pc2mplicitmesh(ptCloud);
%       surfaceMeshShow(mesh)
%
%   Reference: M-plicits: multiscale implicit surfaces, coarse-to-fine
%   residual SIRENs fitted to oriented point clouds.
%
%   See also pc2surfacemesh, pcnormals, surfaceMesh, surfaceMeshShow.

arguments
    ptCloud (1,1) pointCloud
    options.NumLevels (1,1) double {mustBeInteger, mustBeInRange(options.NumLevels, 1, 3)} = 3
    options.Resolution (1,1) double {mustBeInteger, mustBeGreaterThanOrEqual(options.Resolution, 16)} = 256
    options.MinComponentFraction (1,1) double {mustBeInRange(options.MinComponentFraction, 0, 1)} = 0.01
    options.UseGPU (1,1) logical = true
    options.Verbose (1,1) logical = true
    options.Levels struct = struct([])
end

[points, normals] = orientedPoints(ptCloud);
levels = levelSchedule(options.Levels, options.NumLevels);

% The M-plicits schedule (frequencies, band widths) assumes the shape fills
% the unit sphere.
center = (max(points, [], 1) + min(points, [], 1)) / 2;
scale = max(vecnorm(points - center, 2, 2));
p = (points - center) / scale;

usedGPU = options.UseGPU && canUseGPU();
if ~usedGPU
    warning("pc2mplicitmesh:CPU", ...
        "Training on the CPU. This is slow: a full 3-level fit can take about an hour. " + ...
        "A supported GPU and Parallel Computing Toolbox make it much faster.");
end

tTrain = tic;
model = trainMultiscaleSdf(p, normals, levels, usedGPU, options.Verbose);
trainTime = toc(tTrain);

tMesh = tic;
fieldFcn = @(X) fieldValues(model, single(X), numel(model.Stages));
[V, gridVector] = sampleFieldNearCloud(fieldFcn, p, options.Resolution);
[Xg, Yg, Zg] = meshgrid(gridVector, gridVector, gridVector);
fv = isosurface(Xg, Yg, Zg, V, 0);
[faces, vertices, numRemoved] = removeSmallComponents(fv.faces, fv.vertices, ...
    options.MinComponentFraction);
if isempty(faces)
    % surfaceMesh cannot be empty, so there is nothing to return.
    error("pc2mplicitmesh:EmptySurface", ...
        "The fitted field has no zero level set, so there is no surface to mesh. " + ...
        "Check that the normals point out of the shape, or train for more iterations.");
end
mesh = surfaceMesh(vertices * scale + center, faces);
meshTime = toc(tMesh);

info = struct( ...
    "Field", @(X, varargin) evaluateField(model, center, scale, X, varargin{:}), ...
    "BandWidths", [model.Stages(2:end).Delta] * scale, ...
    "Model", model, "Center", center, "Scale", scale, ...
    "UsedGPU", usedGPU, "TrainTime", trainTime, "MeshTime", meshTime, ...
    "NumRemoved", numRemoved);
end

% ========================================================================
% Input
% ========================================================================
function [points, normals] = orientedPoints(ptCloud)
% Finite points with unit normals, as N-by-3 doubles.
if isempty(ptCloud.Normal)
    error("pc2mplicitmesh:NoNormals", ...
        "The point cloud has no normals. Estimate them with pcnormals and " + ...
        "make sure they point out of the shape.");
end
points = double(reshape(ptCloud.Location, [], 3));
normals = double(reshape(ptCloud.Normal, [], 3));
len = vecnorm(normals, 2, 2);
valid = all(isfinite(points), 2) & all(isfinite(normals), 2) & len > 0;
points = points(valid, :);
normals = normals(valid, :) ./ len(valid);
if size(points, 1) < 10
    error("pc2mplicitmesh:TooFewPoints", ...
        "The point cloud needs at least 10 points with valid normals.");
end
end

% ------------------------------------------------------------------------
function levels = levelSchedule(levels, numLevels)
% Per-level settings: the M-plicits coarse/medium/fine schedule (paper
% Sec. 4) unless given, truncated to numLevels.
if isempty(levels)
    levels = struct( ...
        "HiddenSizes",   {[128 128], [256 256], [400 400]}, ...
        "Omega0",        {30,        45,        100}, ...
        "NumIterations", {3500,      750,       750});
end
names = string(fieldnames(levels)).';
unknown = setdiff(names, ["HiddenSizes" "Omega0" "NumIterations"]);
if ~isempty(unknown) || ~all(ismember(["HiddenSizes" "Omega0"], names))
    error("pc2mplicitmesh:InvalidLevels", ...
        "Levels must have fields HiddenSizes and Omega0, and optionally " + ...
        "NumIterations, and no others.");
end
levels = levels(1:min(numLevels, numel(levels)));
for k = 1:numel(levels)
    if ~isfield(levels, "NumIterations") || isempty(levels(k).NumIterations)
        levels(k).NumIterations = 3500 * (k == 1) + 750 * (k > 1);
    end
end
end

% ========================================================================
% Training
% ========================================================================
function model = trainMultiscaleSdf(points, normals, levels, useGPU, verbose)
%TRAINMULTISCALESDF Train the M-plicits coarse-to-fine hierarchy.
%
%   model = trainMultiscaleSdf(points, normals, levels, useGPU, verbose)
%   trains a multiscale implicit surface from an oriented point cloud only
%   (points N-by-3 in the unit sphere, unit outward normals N-by-3). It
%   trains the coarse base field, then each residual level in turn with all
%   previous levels FROZEN:
%
%       F_0 = coarse SIREN, trained over the whole domain
%       F_i = F_{i-1} + R_i, R_i a higher-frequency residual SIREN trained
%             only in a band around the zero level set of F_{i-1}
%
%   levels is a struct array, one element per level, with fields
%   HiddenSizes, Omega0 and NumIterations (see pc2mplicitmesh).
%
%   model.Stages(k) holds each level: Role ("coarse" or "residual"),
%   Enabled, Net (the SIREN struct), HiddenSizes, Omega0, OmegaHidden, Delta
%   (refinement-band half-width, NaN for the coarse level), and Info
%   (training history). Evaluate the field with evaluateCompositeSdf.
%
%   The training functions below are in order: this driver, the coarse
%   stage and its loss, the residual stage and its loss, band sampling,
%   then shared utilities.

numLevels = numel(levels);
nets = cell(1, numLevels);
infos = cell(1, numLevels);
deltas = NaN(1, numLevels);

for i = 1:numLevels
    L = levels(i);
    if verbose
        fprintf("\n=== Level %d/%d: [%s] omega0=%g ===\n", i, numLevels, ...
            strjoin(string(L.HiddenSizes), " "), L.Omega0);
    end
    if i == 1
        [nets{1}, infos{1}] = trainCoarseStage(points, normals, L, useGPU, verbose);
    else
        [nets{i}, infos{i}] = trainResidualStage(nets(1:i-1), points, normals, ...
            L, useGPU, verbose);
        deltas(i) = infos{i}.Delta;
    end
end

stages = struct([]);
for i = 1:numLevels
    net = nets{i};
    if i == 1
        role = "coarse";
    else
        role = "residual";
    end
    stage = struct( ...
        "Role",        role, ...
        "Enabled",     true, ...
        "Net",         net, ...
        "HiddenSizes", cellfun(@(w) size(w, 1), net.Learnables.W(1:end-1)), ...
        "Omega0",      net.Omega0, ...
        "OmegaHidden", net.OmegaHidden, ...
        "Delta",       deltas(i), ...
        "Info",        infos{i});
    stages = [stages, stage]; %#ok<AGROW>
end
model = struct("Stages", stages);
end

% ========================================================================
% Training: coarse stage
% ========================================================================
function [net, info] = trainCoarseStage(points, normals, level, useGPU, verbose)
% Fit the coarse SIREN F_0 over the domain cube [-1 1]^3 with the
% point-cloud loss (coarseLoss). Every iteration sees the whole cloud plus
% fresh off-surface samples with mesh-free signed distance targets.

numIter = level.NumIterations;
N = size(points, 1);
net = initializeSiren(level.HiddenSizes, level.Omega0, useGPU);

normals = normals ./ max(vecnorm(normals, 2, 2), eps);
pts = single(points);
nrm = single(normals);

% Mesh-free signed distance supervision on the off-surface samples. The
% estimate is trusted over the whole domain cube (the truncation is its
% diagonal). Truncating nearer the surface leaves the zero-crossing penalty
% exp(-Alpha*|f|) as the only off-surface term beyond it, and that rewards a
% large |f| of EITHER sign: the far field's sign goes unconstrained and the
% fit can settle on f < 0 over most of the domain, which puts a spurious
% second zero crossing just outside the shape. pointCloudSignedDistance caps
% its magnitude at the nearest-sample distance, so trusting the estimate
% over the whole cube only adds the sign.
sdf = struct("Truncation", 2 * sqrt(3), "NearCount", round(N / 2), ...
    "NearSigma", 0.03, "Searcher", KDTreeSearcher(points));

% Fixed reference batch for choosing between the final and checkpoint
% weights at the end (sampled once so both are scored on identical data).
refIdx = randperm(N, N);
Xref = dlarray(pts(refIdx, :));
Nref = dlarray(nrm(refIdx, :));
[Oref, Dref] = sampleOffSurface(N, points, normals, sdf);
Oref = dlarray(Oref);
Xs = dlarray(pts);
Ns = dlarray(nrm);
if useGPU
    Xref = gpuArray(Xref); Nref = gpuArray(Nref); Oref = gpuArray(Oref);
    Xs = gpuArray(Xs); Ns = gpuArray(Ns);
end

history = struct("Surface", [], "Normal", [], "Eikonal", [], ...
    "Inter", [], "Sdf", [], "Total", []);
tracker = checkpointTracker(net);
for iter = 1:numIter
    [Xo, Do] = sampleOffSurface(N, points, normals, sdf);
    Xo = dlarray(Xo);
    if useGPU
        Xo = gpuArray(Xo);
    end
    [~, grads, comps] = dlfeval(@coarseLossAndGradient, net, Xs, Ns, Xo, Do);
    [net.Learnables, tracker] = adamStep(net.Learnables, grads, tracker, iter);
    [history, tracker] = recordIteration(history, tracker, comps, net.Learnables, iter);
    if verbose && (iter == 1 || mod(iter, 100) == 0)
        fprintf("Iter %5d / %5d --- total %.4e | surf %.3e normal %.3e " + ...
            "eik %.3e inter %.3e sdf %.3e\n", iter, numIter, ...
            comps.Total, comps.Surface, comps.Normal, comps.Eikonal, ...
            comps.Inter, comps.Sdf);
    end
end

refLoss = @(learnables) dlfeval(@coarseLoss, withLearnables(net, learnables), ...
    Xref, Nref, Oref, Dref);
[net.Learnables, selection] = pickBest(net.Learnables, tracker, refLoss);
if verbose
    fprintf("Selected %s weights (ref loss %.4e; checkpoint iter %d).\n", ...
        selection.Chosen, min(selection.Losses), tracker.BestIter);
end
info = struct("Loss", history.Total, "Comps", history, ...
    "Selection", selection, "BestIteration", tracker.BestIter);
end

% ------------------------------------------------------------------------
function [Xo, Do] = sampleOffSurface(count, points, normals, sdf)
% Uniform samples in the domain cube plus near-surface samples (surface
% points with Gaussian jitter), and their estimated signed distances
% (NaN beyond the truncation distance = unsupervised).
Xo = 2 * rand(count, 3) - 1;
if sdf.NearCount > 0
    j = randi(size(points, 1), sdf.NearCount, 1);
    Xo = [Xo; points(j, :) + sdf.NearSigma * randn(sdf.NearCount, 3)];
end
Do = pointCloudSignedDistance(points, normals, Xo, sdf.Searcher);
Do(abs(Do) >= sdf.Truncation) = NaN;
Do = single(Do);
Xo = single(Xo);
end

% ------------------------------------------------------------------------
function s = pointCloudSignedDistance(points, normals, X, searcher)
% Mesh-free signed distance estimate from the K = 8 nearest samples x_j:
%     p(x) = sum_j w_j (x - x_j) . n_j / sum_j w_j,   w_j = 1 / d_j^2
%     s(x) = sign(p) * min(|p|, d_1)
% The weighted point-to-plane distance p gives the sign and is accurate
% near the surface; capping the magnitude at the nearest-sample distance
% d_1 stops tangent-plane extrapolation from overshooting far away.
% Positive outside, negative inside (normals point outward).
K = min(8, size(points, 1));
[idx, d] = knnsearch(searcher, X, K=K);
plane = zeros(size(idx));
for k = 1:K
    j = idx(:, k);
    plane(:, k) = sum((X - points(j, :)) .* normals(j, :), 2);
end
w = 1 ./ (d.^2 + 1e-12);
p = sum(w .* plane, 2) ./ sum(w, 2);
s = sign(p) .* min(abs(p), d(:, 1));
end

% ------------------------------------------------------------------------
function [loss, grads, comps] = coarseLossAndGradient(net, Xs, Ns, Xo, Do)
[loss, comps] = coarseLoss(net, Xs, Ns, Xo, Do);
grads = dlgradient(loss, net.Learnables);
end

% ------------------------------------------------------------------------
function [loss, comps] = coarseLoss(net, Xsurf, Nsurf, Xoff, Doff)
% Mesh-free point-cloud loss for the coarse level (paper Eq. 1, plus the
% SIREN off-surface regularizer and signed distance supervision):
%   Surface : mean f(x_i)^2 on the samples                   (weight 3e3)
%   Normal  : mean (1 - cos(grad f(x_i), n_i))               (weight 1e2)
%   Eikonal : mean (||grad f|| - 1)^2 on all points          (weight 5e1)
%   Sdf     : mean (f - Doff)^2 on supervised off-surface    (weight 1e4)
%   Inter   : mean exp(-100 |f|) on unsupervised off-surface (weight 1e2)
% Sdf is weighted above the paper's 3e3 because the estimate UNDERSHOOTS
% far from the cloud (its magnitude is capped at the nearest-sample
% distance), and a weak target let the field drift back inside the
% refinement band, where an unconstrained residual could then push it
% through zero and leave a blob. With the targets spanning the whole
% domain, Inter normally has no points left to act on.
numSurf = size(Xsurf, 1);
[f, g] = evaluateCompositeSdf(net, [Xsurf; Xoff]);
fSurf = f(1:numSurf);
gSurf = g(1:numSurf, :);
fOff = f(numSurf+1:end);

surfaceTerm = mean(fSurf.^2);
cosSim = sum(gSurf .* Nsurf, 2) ./ sqrt(sum(gSurf.^2, 2) + 1e-12);
normalTerm = mean(1 - cosSim);
eikonalTerm = mean((sqrt(sum(g.^2, 2) + 1e-12) - 1).^2);

supervised = ~isnan(Doff);
if any(supervised)
    sdfTerm = mean((fOff(supervised) - Doff(supervised)).^2);
else
    sdfTerm = 0 * surfaceTerm;
end
if any(~supervised)
    interTerm = mean(exp(-100 * abs(fOff(~supervised))));
else
    interTerm = 0 * surfaceTerm;
end

loss = 3e3 * surfaceTerm + 1e2 * normalTerm + 5e1 * eikonalTerm ...
     + 1e2 * interTerm + 1e4 * sdfTerm;
if nargout > 1
    comps = struct("Surface", toHost(surfaceTerm), "Normal", toHost(normalTerm), ...
        "Eikonal", toHost(eikonalTerm), "Inter", toHost(interTerm), ...
        "Sdf", toHost(sdfTerm), "Total", toHost(loss));
end
end

% ========================================================================
% Training: residual stage
% ========================================================================
function [net, info] = trainResidualStage(frozenNets, points, normals, level, useGPU, verbose)
% Train a residual SIREN R so that F_i = F_{i-1} + R fits the cloud better
% near the current zero level set. The frozen levels are never retrained:
% the loss is differentiated only with respect to R's learnables, and the
% Eikonal and normal terms act on the COMBINED field (residualLoss).

numIter = level.NumIterations;
N = size(points, 1);
net = initializeSiren(level.HiddenSizes, level.Omega0, useGPU);
normals = normals ./ max(vecnorm(normals, 2, 2), eps);

% Adaptive band half-width (paper Eq. 5), mesh-free over the input points:
% delta = (1 + epsilon) max_j |F_{i-1}(x_j)|, epsilon = 0.3. The frozen
% field is constant, so this and the neighbour spacing are computed once.
f0all = gather(evaluateCompositeSdf(frozenNets, points));
delta = double((1 + 0.3) * max(abs(f0all)));
[~, d2] = knnsearch(points, points, K=2);
neighborDist = d2(:, 2);

% Fixed reference band for choosing between the final and checkpoint
% weights at the end.
refBand = toDlarray(sampleRefinementBand(frozenNets, points, normals, ...
    delta, neighborDist, N), useGPU);

% Band samples come from a pool 10 batches large that carries the frozen
% field's value and gradient, so the frozen levels are evaluated once per
% pool (redrawn every 100 iterations) rather than every iteration. Each
% iteration draws a fresh random batch from it.
history = struct("Sdf", [], "Normal", [], "Eikonal", [], "Total", []);
tracker = checkpointTracker(net);
for iter = 1:numIter
    if mod(iter - 1, 100) == 0
        pool = sampleRefinementBand(frozenNets, points, normals, ...
            delta, neighborDist, 10 * N);
    end
    band = toDlarray(drawFromPool(pool, N), useGPU);
    [~, grads, comps] = dlfeval(@residualLossAndGradient, net, band);
    [net.Learnables, tracker] = adamStep(net.Learnables, grads, tracker, iter);
    [history, tracker] = recordIteration(history, tracker, comps, net.Learnables, iter);
    if verbose && (iter == 1 || mod(iter, 100) == 0)
        fprintf("Iter %5d / %5d --- total %.4e | sdf %.3e normal %.3e " + ...
            "eik %.3e  (delta %.3e)\n", iter, numIter, ...
            comps.Total, comps.Sdf, comps.Normal, comps.Eikonal, delta);
    end
end

refLoss = @(learnables) dlfeval(@residualLoss, withLearnables(net, learnables), refBand);
[net.Learnables, selection] = pickBest(net.Learnables, tracker, refLoss);
if verbose
    fprintf("Selected %s residual weights (ref loss %.4e; checkpoint iter %d).\n", ...
        selection.Chosen, min(selection.Losses), tracker.BestIter);
end
info = struct("Delta", delta, "Loss", history.Total, "Comps", history, ...
    "Selection", selection, "BestIteration", tracker.BestIter);
end

% ------------------------------------------------------------------------
function [loss, grads, comps] = residualLossAndGradient(net, band)
% Differentiate ONLY the residual's learnables; the coarse levels stay frozen.
[loss, comps] = residualLoss(net, band);
grads = dlgradient(loss, net.Learnables);
end

% ------------------------------------------------------------------------
function [loss, comps] = residualLoss(net, band)
% Narrow-band loss for one residual (paper Eq. 1/4, residual form). Every
% term uses the COMBINED field F = F_{i-1} + R and its gradient
% grad F_{i-1} + grad R. The frozen part comes precomputed with the band
% (Ffrozen, Gfrozen); only R goes through automatic differentiation.
%   Sdf     : mean (F - d)^2 on on-surface (d = 0) and offset points (3e3)
%   Normal  : mean (1 - cos(grad F, n)) on the same points          (1e2)
%   Eikonal : mean (||grad F|| - 1)^2 on all band points            (5e1)
Xsup = [band.Xsurf; band.Xoff];
Nsup = [band.Nsurf; band.Noff];
Dsup = [zeros(size(band.Xsurf, 1), 1, "like", band.Doff); band.Doff];
numSup = size(Xsup, 1);

[fr, gr] = evaluateCompositeSdf(net, [Xsup; band.Xdither]);
f = band.Ffrozen + fr;
g = band.Gfrozen + gr;
fSup = f(1:numSup);
gSup = g(1:numSup, :);

sdfTerm = mean((fSup - Dsup).^2);
cosSim = sum(gSup .* Nsup, 2) ./ sqrt(sum(gSup.^2, 2) + 1e-12);
normalTerm = mean(1 - cosSim);
eikonalTerm = mean((sqrt(sum(g.^2, 2) + 1e-12) - 1).^2);

loss = 3e3 * sdfTerm + 1e2 * normalTerm + 5e1 * eikonalTerm;
if nargout > 1
    comps = struct("Sdf", toHost(sdfTerm), "Normal", toHost(normalTerm), ...
        "Eikonal", toHost(eikonalTerm), "Total", toHost(loss));
end
end

% ------------------------------------------------------------------------
function band = sampleRefinementBand(nets, points, normals, delta, neighborDist, count)
% count samples of each kind in the band |F_{i-1}| < delta, from the
% oriented cloud only (no mesh, no ground-truth SDF):
%   on-surface     x_j          target 0, normal n_j
%   normal-offset  x_j + a n_j  target a, normal n_j, with a = u min(delta,
%                               t_j), u ~ U(-1, 1) and t_j the distance to
%                               x_j's nearest neighbour, so x_j stays the
%                               nearest surface sample and a is a valid
%                               signed distance (paper: f(x_j + t n_j) = t)
%   dithered       x_j + U(-2 delta, 2 delta) per coordinate, kept where
%                               |F_{i-1}| < delta; Eikonal term only
% The frozen field's value and gradient at every sample ([Xsurf; Xoff;
% Xdither], in that order) come along, because F_{i-1} is constant while R
% trains.
N = size(points, 1);
normals = normals ./ max(vecnorm(normals, 2, 2), eps);

sIdx = drawIndices(N, count);
Xsurf = points(sIdx, :);
Nsurf = normals(sIdx, :);

oIdx = drawIndices(N, count);
amp = (2 * rand(count, 1) - 1) .* min(delta, neighborDist(oIdx));
Xoff = points(oIdx, :) + amp .* normals(oIdx, :);
Noff = normals(oIdx, :);

Xdither = zeros(0, 3);
attempts = 0;
while size(Xdither, 1) < count && attempts < 10
    cand = points(drawIndices(N, count), :) + (4 * delta) * (rand(count, 3) - 0.5);
    fCand = gather(evaluateCompositeSdf(nets, cand));
    Xdither = [Xdither; cand(abs(fCand) < delta, :)]; %#ok<AGROW>
    attempts = attempts + 1;
end
Xdither = Xdither(1:min(end, count), :);

band = struct("Delta", delta, ...
    "Xsurf", single(Xsurf), "Nsurf", single(Nsurf), ...
    "Xoff", single(Xoff), "Noff", single(Noff), "Doff", single(amp), ...
    "Xdither", single(Xdither));

% Frozen field at every sample, in chunks to bound memory. It stays on the
% nets' device (gpuArray for GPU-trained levels).
X = [band.Xsurf; band.Xoff; band.Xdither];
M = size(X, 1);
for head = 1:50000:M
    rows = head:min(head + 49999, M);
    [fi, gi] = evaluateCompositeSdf(nets, X(rows, :));
    if head == 1
        band.Ffrozen = zeros(M, 1, "like", fi);
        band.Gfrozen = zeros(M, 3, "like", gi);
    end
    band.Ffrozen(rows) = fi;
    band.Gfrozen(rows, :) = gi;
end
end

% ------------------------------------------------------------------------
function idx = drawIndices(N, count)
% Random sample indices, with replacement only when asking for more than N.
if count <= N
    idx = randperm(N, count);
else
    idx = randi(N, count, 1);
end
end

% ------------------------------------------------------------------------
function band = drawFromPool(pool, count)
% Random batch from a band pool, keeping the frozen-field rows aligned with
% the [Xsurf; Xoff; Xdither] samples they belong to.
nS = size(pool.Xsurf, 1);
nO = size(pool.Xoff, 1);
nD = size(pool.Xdither, 1);
iS = randperm(nS, min(count, nS));
iO = randperm(nO, min(count, nO));
iD = randperm(nD, min(count, nD));
rows = [iS, nS + iO, nS + nO + iD];
band = struct("Delta", pool.Delta, ...
    "Xsurf", pool.Xsurf(iS, :), "Nsurf", pool.Nsurf(iS, :), ...
    "Xoff", pool.Xoff(iO, :), "Noff", pool.Noff(iO, :), "Doff", pool.Doff(iO), ...
    "Xdither", pool.Xdither(iD, :), ...
    "Ffrozen", pool.Ffrozen(rows), "Gfrozen", pool.Gfrozen(rows, :));
end

% ------------------------------------------------------------------------
function band = toDlarray(band, useGPU)
fn = ["Xsurf" "Nsurf" "Xoff" "Noff" "Doff" "Xdither" "Ffrozen" "Gfrozen"];
for k = 1:numel(fn)
    v = dlarray(band.(fn(k)));
    if useGPU
        v = gpuArray(v);
    end
    band.(fn(k)) = v;
end
end

% ========================================================================
% Training: shared utilities
% ========================================================================
function net = initializeSiren(hiddenSizes, omega0, useGPU)
% SIREN R^3 -> R with sine initialization (Sitzmann et al. 2020, as used by
% M-plicits): first-layer weights ~ U(-1/fanIn, 1/fanIn), later weights
% (including the linear output) ~ U(+-sqrt(6/fanIn)/omega), biases
% ~ U(+-1/sqrt(fanIn)). Every layer uses the same frequency omega0.
layerSizes = [3, hiddenSizes, 1];
numLayers = numel(layerSizes) - 1;
W = cell(1, numLayers);
b = cell(1, numLayers);
for k = 1:numLayers
    fanIn = layerSizes(k);
    if k == 1
        weightBound = 1 / fanIn;
    else
        weightBound = sqrt(6 / fanIn) / omega0;
    end
    Wk = (2 * rand([layerSizes(k+1) fanIn], "single") - 1) * weightBound;
    bk = (2 * rand([layerSizes(k+1) 1], "single") - 1) * (1 / sqrt(fanIn));
    if useGPU
        Wk = gpuArray(Wk);
        bk = gpuArray(bk);
    end
    W{k} = dlarray(Wk);
    b{k} = dlarray(bk);
end
net = struct("Learnables", struct("W", {W}, "b", {b}), ...
    "Omega0", omega0, "OmegaHidden", omega0);
end

% ------------------------------------------------------------------------
function t = checkpointTracker(net)
% Adam state plus a best-loss checkpoint on an exponentially smoothed loss
% (smoothing factor beta = 0.9 denoises the stochastic per-iteration loss).
t = struct("AvgGrad", [], "AvgSqGrad", [], "SmoothLoss", [], ...
    "BestSmooth", Inf, "BestLearnables", net.Learnables, "BestIter", 0);
end

% ------------------------------------------------------------------------
function [learnables, t] = adamStep(learnables, grads, t, iter)
% Adam with the M-plicits learning rate of 1e-4, constant.
[learnables, t.AvgGrad, t.AvgSqGrad] = adamupdate(learnables, grads, ...
    t.AvgGrad, t.AvgSqGrad, iter, 1e-4);
end

% ------------------------------------------------------------------------
function [history, t] = recordIteration(history, t, comps, learnables, iter)
% Append this iteration's loss terms and update the best-loss checkpoint.
fn = fieldnames(history);
for k = 1:numel(fn)
    history.(fn{k})(end+1, 1) = comps.(fn{k});
end
if isempty(t.SmoothLoss)
    t.SmoothLoss = comps.Total;
else
    beta = 0.9;
    t.SmoothLoss = beta * t.SmoothLoss + (1 - beta) * comps.Total;
end
if t.SmoothLoss < t.BestSmooth
    t.BestSmooth = t.SmoothLoss;
    t.BestLearnables = learnables;
    t.BestIter = iter;
end
end

% ------------------------------------------------------------------------
function [learnables, selection] = pickBest(finalLearnables, t, refLoss)
% Keep the final or the checkpoint weights, whichever has the lower loss on
% the fixed reference batch.
names = ["final" "checkpoint"];
candidates = {finalLearnables, t.BestLearnables};
losses = zeros(1, 2);
for i = 1:2
    losses(i) = double(gather(extractdata(refLoss(candidates{i}))));
end
[~, best] = min(losses);
learnables = candidates{best};
selection = struct("Names", names, "Losses", losses, "Chosen", names(best));
end

% ------------------------------------------------------------------------
function net = withLearnables(net, learnables)
net.Learnables = learnables;
end

% ------------------------------------------------------------------------
function v = toHost(x)
v = gather(extractdata(x));
end

% ========================================================================
% Field evaluation
% ========================================================================
function F = fieldValues(model, X, numLevels)
% Band-culled field of the first numLevels levels at normalized points X:
% residual k is added only where the coarser field is within its band
% (paper Sec. 10), so each level leaves the field unchanged elsewhere.
nets = {model.Stages(1:numLevels).Net};
deltas = [model.Stages(2:numLevels).Delta];
if isempty(deltas)
    F = evaluateCompositeSdf(nets, X);
else
    F = evaluateCompositeSdf(nets, X, Deltas=deltas);
end
F = double(gather(F));
end

% ------------------------------------------------------------------------
function F = evaluateField(model, center, scale, X, numLevels)
% Field of the first numLevels levels at points X in input coordinates.
% Distances scale with the normalization, so F is in input units too.
arguments
    model
    center
    scale
    X (:,3) {mustBeNumeric}
    numLevels (1,1) double {mustBeInteger, mustBePositive} = numel(model.Stages)
end
if numLevels > numel(model.Stages)
    error("pc2mplicitmesh:NumLevels", ...
        "The model has %d levels; numLevels must not exceed that.", ...
        numel(model.Stages));
end
p = single((double(X) - center) / scale);
F = zeros(size(p, 1), 1);
batch = 2^18;
for head = 1:batch:size(p, 1)
    s = head:min(head + batch - 1, size(p, 1));
    F(s) = fieldValues(model, p(s, :), numLevels);
end
F = F * scale;
end

% ------------------------------------------------------------------------
function [f, gradF] = evaluateCompositeSdf(nets, X, options)
%EVALUATECOMPOSITESDF Evaluate an additive M-plicits field and its gradient.
%
%   The M-plicits field is a sum of a coarse SIREN and zero or more residual
%   SIRENs:
%
%       F_k(x) = F_0(x) + R_1(x) + ... + R_k(x)
%
%   and its spatial gradient is the sum of the per-level gradients.
%
%   f = evaluateCompositeSdf(nets, X) returns the N-by-1 composite field at
%   the N-by-3 points X. nets is a cell array of SIREN structs ordered
%   coarse-first {F_0, R_1, ...}; a single struct is also accepted.
%
%   [f, gradF] = evaluateCompositeSdf(nets, X) also returns the N-by-3
%   gradient of the COMBINED field, by automatic differentiation. Inside a
%   dlfeval loss (X a traced dlarray) this is differentiable with respect to
%   each net's learnables, so freezing a level is done simply by not
%   requesting its gradient in the outer dlgradient call. Numeric X returns
%   numeric outputs.
%
%   f = evaluateCompositeSdf(nets, X, Deltas=deltas) band-culls the value
%   (paper Sec. 10 / Eq. 5): residual R_i is added only where
%   |F_{i-1}| < deltas(i), so the field is left exactly equal to the coarser
%   level outside each band. Value only, because the mask is discontinuous.
%
%   A SIREN struct (see initializeSiren) holds
%   Learnables.W and Learnables.b, cell arrays of dlarray weights and biases,
%   and the frequencies Omega0 (first layer) and OmegaHidden:
%
%       h_1 = sin(Omega0      * (W_1 x       + b_1))
%       h_k = sin(OmegaHidden * (W_k h_{k-1} + b_k)),  k = 2..L
%       f   = W_{L+1} h_L + b_{L+1}

arguments
    nets
    X
    options.Deltas (1,:) double = []
end

if isstruct(nets)
    nets = num2cell(nets);
end

if nargout < 2
    f = compositeValue(nets, X, options.Deltas);
    return
end

if ~isempty(options.Deltas)
    error("evaluateCompositeSdf:GradWithBandCull", ...
        "Band-culled evaluation (Deltas) is value-only.");
end

returnNumeric = ~isa(X, "dlarray");
if returnNumeric
    W1 = extractdata(nets{1}.Learnables.W{1});
    X = dlarray(cast(X, "like", W1));
    [f, gradF] = dlfeval(@compositeAutodiff, nets, X);
    f = extractdata(f);
    gradF = extractdata(gradF);
else
    [f, gradF] = compositeAutodiff(nets, X);
end
end

% ------------------------------------------------------------------------
function f = compositeValue(nets, X, deltas)
% Plain sum, or band-culled composition when deltas is non-empty.
f = sirenValue(nets{1}, X);
for i = 2:numel(nets)
    r = sirenValue(nets{i}, X);
    if isempty(deltas)
        f = f + r;
    else
        % Add this residual only inside the current level's band. The mask
        % uses the running composite F_{i-1}, matching paper Sec. 10.
        inBand = abs(f) < deltas(i-1);
        f(inBand) = f(inBand) + r(inBand);
    end
end
end

% ------------------------------------------------------------------------
function [f, gradF] = compositeAutodiff(nets, X)
% Combined value, then one reverse-mode gradient of the SUMMED field with
% respect to X. Differentiating the composite (rather than each level) is
% what makes gradF = grad F_0 + grad R_1 + ... automatically.
f = sirenValue(nets{1}, X);
for i = 2:numel(nets)
    f = f + sirenValue(nets{i}, X);
end
gradF = dlgradient(sum(f, "all"), X, EnableHigherDerivatives=true);
end

% ------------------------------------------------------------------------
function f = sirenValue(net, X)
% One SIREN at N-by-3 points X. Numeric X is cast to the learnables' type
% and device and the result returned numeric; a dlarray X stays traced.
returnNumeric = ~isa(X, "dlarray");
if returnNumeric
    W1 = extractdata(net.Learnables.W{1});
    X = dlarray(cast(X, "like", W1));
end
W = net.Learnables.W;
b = net.Learnables.b;
h = X.';
for k = 1:numel(W) - 1
    if k == 1
        omega = net.Omega0;
    else
        omega = net.OmegaHidden;
    end
    h = sin(omega * (W{k} * h + b{k}));
end
f = (W{end} * h + b{end}).';
if returnNumeric
    f = extractdata(f);
end
end

% ========================================================================
% Meshing
% ========================================================================
function [V, xv] = sampleFieldNearCloud(fieldFcn, points, n)
% Field on an n^3 grid over [-1 1]^3 (V(i,j,k) at (xv(j), xv(i), xv(k)),
% the meshgrid layout), evaluated only in a band around the cloud. The zero
% level set is a surface, so it touches few cells; the rest are filled with
% a constant of the right sign instead of evaluated. The sign follows from
% connectivity: the band is a closed shell around the surface, so every
% connected region outside it lies wholly inside or wholly outside the
% solid, and the regions that reach the grid boundary are outside.
%
% The band is deliberately generous, so that the mesh matches a dense
% evaluation exactly: on the fixedwing at 128^3 the mesh becomes identical
% to dense once the radius reaches 0.24, and 0.15 * span = 0.30 leaves a
% margin. It scales with the domain rather than the cell size, because how
% far the field strays from the data is a property of the field, not of
% the grid.
xv = linspace(-1, 1, n);
h = 2 / (n - 1);
[~, d2] = knnsearch(points, points, K=2);     % column 2 skips self
radius = max(0.15 * 2, 2 * median(d2(:, 2)));
% At least one cell diagonal, so both endpoints of every edge that crosses
% the surface have a real value.
radius = max(radius, sqrt(3) * h);
rVox = max(1, ceil(radius / h));

% Cloud voxels, dilated to the band (meshgrid layout: x indexes dim 2).
jx = min(max(round((points(:, 1) + 1) / h) + 1, 1), n);
iy = min(max(round((points(:, 2) + 1) / h) + 1, 1), n);
kz = min(max(round((points(:, 3) + 1) / h) + 1, 1), n);
band = false(n, n, n);
band(sub2ind([n n n], iy, jx, kz)) = true;
band = imdilate(band, strel("cuboid", (2 * rVox + 1) * [1 1 1]));

% Sign outside the band, from connectivity.
L = bwlabeln(~band, 6);
outerLabels = unique([reshape(L([1 n], :, :), [], 1); ...
                      reshape(L(:, [1 n], :), [], 1); ...
                      reshape(L(:, :, [1 n]), [], 1)]);
outerLabels(outerLabels == 0) = [];
outside = ismember(L, outerLabels);
V = zeros(n, n, n);
V(~band & outside) = 1;
V(~band & ~outside) = -1;

% Evaluate the band, in batches.
idx = find(band);
[bi, bj, bk] = ind2sub([n n n], idx);
P = [reshape(xv(bj), [], 1), reshape(xv(bi), [], 1), reshape(xv(bk), [], 1)];
vals = zeros(numel(idx), 1);
batch = 2^18;
for head = 1:batch:numel(idx)
    s = head:min(head + batch - 1, numel(idx));
    vals(s) = fieldFcn(P(s, :));
end
V(idx) = vals;
end

% ------------------------------------------------------------------------
function [faces, vertices, numRemoved] = removeSmallComponents(faces, vertices, fraction)
% Drop connected components whose area is below fraction of the total.
% Neural fields often leave small closed blobs ("floaters") away from the
% data; they are separate components and tiny next to the real surface.
numRemoved = 0;
if fraction == 0 || isempty(faces)
    return
end
G = graph(faces(:, [1 2 3]), faces(:, [2 3 1]), [], size(vertices, 1));
bins = conncomp(G);
faceBin = bins(faces(:, 1)).';
e1 = vertices(faces(:, 2), :) - vertices(faces(:, 1), :);
e2 = vertices(faces(:, 3), :) - vertices(faces(:, 1), :);
area = vecnorm(cross(e1, e2, 2), 2, 2) / 2;
binArea = accumarray(faceBin, area, [max(bins) 1]);
keepBin = binArea >= fraction * sum(area);
numRemoved = nnz(binArea > 0 & ~keepBin);

faces = faces(keepBin(faceBin), :);
[used, ~, newIdx] = unique(faces(:));
vertices = vertices(used, :);
faces = reshape(newIdx, [], 3);
end
