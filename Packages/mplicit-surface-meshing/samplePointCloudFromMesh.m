function [points, normals, TR] = samplePointCloudFromMesh(geometry, options)
%SAMPLEPOINTCLOUDFROMMESH Sample an oriented point cloud from an STL mesh.
%
%   [points, normals] = samplePointCloudFromMesh(geometry) reads a triangle
%   mesh and returns an oriented point cloud: points (N-by-3) sampled
%   uniformly over the surface by area, and normals (N-by-3) unit outward
%   normals taken from the sampled triangle's face normal. This produces
%   exactly the input M-plicits expects (points + normals, no faces).
%
%   geometry is either:
%     * a name of a MATLAB built-in STL (case-insensitive), such as
%       "torus", "block", "tetrahedron", "forearmlink", "brackettwoholes",
%       "bracketwithhole", "dampingmounts", "motherboard", "frame",
%       "platehole", "fixedwing", or "groundvehicle"; or
%     * a path to any .stl file.
%   Call samplePointCloudFromMesh("list") to print the built-in catalog.
%
%   [points, normals, TR] = samplePointCloudFromMesh(___) also returns the
%   source triangulation (in the same, possibly normalized, coordinates as
%   points) so the original mesh can be visualized alongside the samples.
%
%   Name-value options:
%     Density    - target points per unit surface area, measured in the
%                  output coordinate frame (default 5000). The point count
%                  is round(Density * totalArea).
%     NumPoints  - exact number of points to sample. Overrides Density.
%     Normalize  - center and scale the mesh to the unit sphere, matching
%                  the M-plicits convention (default true).
%     NoiseLevel - amount of noise added along the normal, as a fraction
%                  of the model's bounding-box diagonal (default 0, i.e.
%                  clean). Its meaning depends on NoiseModel.
%     NoiseModel - distribution of that displacement (default "gaussian"):
%                    "gaussian" N(0, s^2), with std s = NoiseLevel * diag.
%                    "uniform"  U(-d, d), with half-width d = NoiseLevel *
%                               diag (std d/sqrt(3)). Bounded, so the
%                               largest excursion is exactly d. This is the
%                               M-plicits paper's protocol ("displaced along
%                               the normal by up to 1% of the bounding box").
%                  Normals are left clean in both cases.
%     Seed       - RNG seed for reproducible sampling (default [], no reset).
%     Method     - point placement strategy (default "area"):
%                    "area"      i.i.d. area-weighted barycentric sampling.
%                                Unbiased but Poisson-like: visible clumps and
%                                voids in local density.
%                    "bluenoise" even, Poisson-disk-like coverage via weighted
%                                sample elimination (Yuksel 2015). Oversamples
%                                by OverSample, then eliminates to the target
%                                count. Normals are carried from the surviving
%                                samples' triangles.
%     OverSample - oversampling factor for Method="bluenoise" (default 5):
%                  round(OverSample * numPoints) candidates are drawn before
%                  elimination. Higher is more even but slower.
%     ExteriorOnly - keep only samples on the visible outer surface
%                  (default false). Some STLs (e.g. "fixedwing") are
%                  overlapping closed parts with internal caps, so plain
%                  sampling also covers faces buried inside the solid that
%                  a scan would never see. A sample is kept when a tiny
%                  step along its normal leaves the solid (generalized
%                  winding number of the mesh < 1/2). NumPoints is still
%                  met exactly.
%
%   Example:
%       [p, n, TR] = samplePointCloudFromMesh("torus", Density=8000);
%       trisurf(TR, FaceAlpha=0.3); hold on;
%       quiver3(p(:,1),p(:,2),p(:,3), n(:,1),n(:,2),n(:,3), 0.5);
%
%   See also stlread, triangulation, faceNormal.

arguments
    geometry (1,1) string
    options.Density (1,1) double {mustBePositive} = 5000
    options.NumPoints double {mustBeScalarOrEmpty, mustBePositive, mustBeInteger} = []
    options.Normalize (1,1) logical = true
    options.NoiseLevel (1,1) double {mustBeNonnegative} = 0
    options.NoiseModel (1,1) string {mustBeMember(options.NoiseModel, ["gaussian" "uniform"])} = "gaussian"
    options.Seed double {mustBeScalarOrEmpty} = []
    options.Method (1,1) string {mustBeMember(options.Method, ["area" "bluenoise"])} = "area"
    options.OverSample (1,1) double {mustBePositive} = 5
    options.ExteriorOnly (1,1) logical = false
end

if strcmpi(geometry, "list")
    printCatalog();
    [points, normals, TR] = deal([]);
    return
end

if ~isempty(options.Seed)
    rng(options.Seed);
end

% ---- Load the mesh -------------------------------------------------------
stlPath = resolveGeometry(geometry);
TR = stlread(stlPath);
V = double(TR.Points);
F = double(TR.ConnectivityList);

% ---- Normalize to the unit sphere (M-plicits convention) -----------------
bboxDiag = norm(max(V, [], 1) - min(V, [], 1));
if options.Normalize
    center = (max(V, [], 1) + min(V, [], 1)) / 2;
    V = V - center;
    scale = max(sqrt(sum(V.^2, 2)));
    V = V / scale;
    bboxDiag = bboxDiag / scale;
    TR = triangulation(F, V);
end

% ---- Per-triangle geometry ----------------------------------------------
v1 = V(F(:, 1), :);
v2 = V(F(:, 2), :);
v3 = V(F(:, 3), :);
crossProd = cross(v2 - v1, v3 - v1, 2);
triArea = 0.5 * sqrt(sum(crossProd.^2, 2));
totalArea = sum(triArea);

faceN = faceNormal(TR);

% ---- Optional exterior-only filter ---------------------------------------
% Fraction of the area that is exterior, estimated from a pilot sample, so
% Density and the blue-noise radius refer to the visible surface only.
extFrac = 1;
if options.ExteriorOnly
    [pp, pn] = sampleAreaWeighted(20000, v1, v2, v3, faceN, triArea, ...
        totalArea, RandStream("twister", Seed=0));
    extFrac = max(mean(exteriorMask(pp, pn, F, V)), 1e-3);
end
draw = @(count) sampleExterior(count, v1, v2, v3, faceN, triArea, ...
    totalArea, options.ExteriorOnly, F, V, extFrac);

% ---- Decide how many points to sample ------------------------------------
if ~isempty(options.NumPoints)
    numPoints = options.NumPoints;
else
    numPoints = max(1, round(options.Density * totalArea * extFrac));
end

% ---- Sample positions and normals ----------------------------------------
if options.Method == "bluenoise"
    % Oversample by area, then thin to an even blue-noise set by weighted
    % sample elimination. Normals follow the surviving samples.
    numCand = max(numPoints + 1, round(options.OverSample * numPoints));
    [candP, candN] = draw(numCand);
    keep = blueNoiseEliminate(candP, numPoints, totalArea * extFrac);
    points = candP(keep, :);
    normals = candN(keep, :);
else
    [points, normals] = draw(numPoints);
end

% ---- Optional noise along the normal -------------------------------------
if options.NoiseLevel > 0
    if options.NoiseModel == "uniform"
        offset = 2 * rand(numPoints, 1) - 1;
    else
        offset = randn(numPoints, 1);
    end
    points = points + (options.NoiseLevel * bboxDiag) .* offset .* normals;
end

% Ensure unit normals (faceNormal already returns unit vectors).
normals = normals ./ max(vecnorm(normals, 2, 2), eps);
end

% =========================================================================
function [points, normals] = sampleExterior(count, v1, v2, v3, faceN, ...
    triArea, totalArea, exteriorOnly, F, V, extFrac)
% COUNT area-weighted samples, keeping only exterior ones if requested
% (drawn in rounds sized by the expected exterior fraction).
if ~exteriorOnly
    [points, normals] = sampleAreaWeighted(count, v1, v2, v3, faceN, triArea, totalArea);
    return
end
points = zeros(0, 3);
normals = zeros(0, 3);
while size(points, 1) < count
    need = count - size(points, 1);
    [p, n] = sampleAreaWeighted(ceil(1.1 * need / extFrac) + 16, ...
        v1, v2, v3, faceN, triArea, totalArea);
    keep = exteriorMask(p, n, F, V);
    points = [points; p(keep, :)]; %#ok<AGROW>
    normals = [normals; n(keep, :)]; %#ok<AGROW>
end
points = points(1:count, :);
normals = normals(1:count, :);
end

% =========================================================================
function ext = exteriorMask(P, N, F, V)
% True for samples on the visible surface: a tiny step along the normal
% leaves the solid, i.e. the generalized winding number (Jacobson et al.
% 2013) of the whole mesh there is below 1/2. Samples on faces buried
% inside the solid (overlapping parts, internal caps) step into it. The
% step is relative to the mesh size: touching parts' caps are often only
% NEARLY coincident (fixedwing's nose/fuselage caps are ~2e-4 apart in STL
% units), and a step that stays in that sliver reads as exterior.
step = 1e-4 * norm(max(V, [], 1) - min(V, [], 1));
Q = P + step * N;
a = V(F(:, 1), :).';
b = V(F(:, 2), :).';
c = V(F(:, 3), :).';
ext = true(size(P, 1), 1);
chunk = max(1, floor(2e6 / size(F, 1)));
for head = 1:chunk:size(Q, 1)
    idx = head:min(head + chunk - 1, size(Q, 1));
    ext(idx) = windingNumber(Q(idx, :), a, b, c) < 0.5;
end
end

function w = windingNumber(Q, a, b, c)
% Q is M-by-3; a, b, c are 3-by-K triangle corners. Returns M-by-1.
ax = a(1, :) - Q(:, 1); ay = a(2, :) - Q(:, 2); az = a(3, :) - Q(:, 3);
bx = b(1, :) - Q(:, 1); by = b(2, :) - Q(:, 2); bz = b(3, :) - Q(:, 3);
cx = c(1, :) - Q(:, 1); cy = c(2, :) - Q(:, 2); cz = c(3, :) - Q(:, 3);
la = sqrt(ax.^2 + ay.^2 + az.^2);
lb = sqrt(bx.^2 + by.^2 + bz.^2);
lc = sqrt(cx.^2 + cy.^2 + cz.^2);
tripleProd = ax .* (by .* cz - bz .* cy) - ay .* (bx .* cz - bz .* cx) + ...
    az .* (bx .* cy - by .* cx);
den = la .* lb .* lc + (ax .* bx + ay .* by + az .* bz) .* lc + ...
    (bx .* cx + by .* cy + bz .* cz) .* la + (cx .* ax + cy .* ay + cz .* az) .* lb;
w = sum(atan2(tripleProd, den), 2) / (2 * pi);
end

% =========================================================================
function [points, normals] = sampleAreaWeighted(count, v1, v2, v3, faceN, triArea, totalArea, rs)
% I.i.d. area-weighted barycentric sampling of COUNT oriented points, from
% the global RNG or the optional RandStream rs.
if nargin < 8
    rs = RandStream.getGlobalStream();
end

% Area-weighted triangle selection.
cumArea = cumsum(triArea) / totalArea;
r = rand(rs, count, 1);
triIdx = discretize(r, [0; cumArea]);
% Guard against numerical edge cases at the top of the range.
triIdx(isnan(triIdx)) = numel(triArea);

% Uniform barycentric sampling within each chosen triangle.
u = rand(rs, count, 1);
w = rand(rs, count, 1);
reflect = (u + w) > 1;
u(reflect) = 1 - u(reflect);
w(reflect) = 1 - w(reflect);

a = v1(triIdx, :);
e1 = v2(triIdx, :) - a;
e2 = v3(triIdx, :) - a;
points = a + u .* e1 + w .* e2;
normals = faceN(triIdx, :);
end

% =========================================================================
function stlPath = resolveGeometry(geometry)
% Resolve a friendly name or a path to an STL file on disk.
if endsWith(geometry, ".stl", IgnoreCase=true) && isfile(geometry)
    stlPath = char(geometry);
    return
end

cat = builtinCatalog();
key = lower(geometry);
if ~isKey(cat, key)
    error("samplePointCloudFromMesh:UnknownGeometry", ...
        "Unknown geometry '%s'. Call samplePointCloudFromMesh(""list"") " + ...
        "to see available names, or pass a path to an .stl file.", geometry);
end

relPaths = cat(key);
for k = 1:numel(relPaths)
    candidate = fullfile(matlabroot, relPaths{k});
    if isfile(candidate)
        stlPath = candidate;
        return
    end
end

% Fall back to a scoped search for the file name.
fname = relPaths{1};
[~, name, ext] = fileparts(fname);
hit = dir(fullfile(matlabroot, "toolbox", "**", name + string(ext)));
if ~isempty(hit)
    stlPath = fullfile(hit(1).folder, hit(1).name);
    return
end

error("samplePointCloudFromMesh:FileNotFound", ...
    "Could not locate STL for '%s' under %s. The owning toolbox may " + ...
    "not be installed.", geometry, matlabroot);
end

% =========================================================================
function cat = builtinCatalog()
% Friendly name -> candidate relative paths under matlabroot.
pde = fullfile("toolbox", "pde", "pde", "pdedata");
rob = fullfile("toolbox", "shared", "robotics", "robotdata", "meshes");
cat = dictionary();
cat("torus")           = {fullfile(pde, "Torus.stl")};
cat("block")           = {fullfile(pde, "Block.stl")};
cat("tetrahedron")     = {fullfile(pde, "Tetrahedron.stl")};
cat("forearmlink")     = {fullfile(pde, "ForearmLink.stl")};
cat("brackettwoholes") = {fullfile(pde, "BracketTwoHoles.stl")};
cat("bracketwithhole") = {fullfile(pde, "BracketWithHole.stl")};
cat("dampingmounts")   = {fullfile(pde, "DampingMounts.stl")};
cat("motherboard")     = {fullfile(pde, "MotherboardFragment1.stl")};
cat("frame")           = {fullfile(pde, "Frame.stl")};
cat("platehole")       = {fullfile(pde, "PlateHoleSolid.stl")};
cat("fixedwing")       = {fullfile(rob, "fixedwing.stl")};
cat("groundvehicle")   = {fullfile(rob, "groundvehicle.stl")};
end

% =========================================================================
function printCatalog()
cat = builtinCatalog();
names = sort(keys(cat));
fprintf("Built-in STL geometries:\n");
for k = 1:numel(names)
    rel = cat(names(k));
    available = isfile(fullfile(matlabroot, rel{1}));
    status = "   (not installed)";
    if available
        status = "";
    end
    fprintf("  %-18s %s%s\n", names(k), rel{1}, status);
end
end
