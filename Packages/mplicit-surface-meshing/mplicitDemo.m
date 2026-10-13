%[text] # Neural Surface Reconstruction with M-plicits
%[text] This example reconstructs a watertight surface mesh from an oriented point cloud using `pc2mplicitmesh`, then looks inside the fitted field to see how M-plicits works.
%[text] M-plicits represents the surface as the zero level set of a field built from sinusoidal neural networks (SIRENs), $F = F\_0 + R\_1 + R\_2$. A smooth coarse network $F\_0$ is trained first over the whole domain. Each residual $R\_k$ uses a higher frequency, is trained with the coarser levels frozen, and only adds detail in a thin band around the surface.
%[text] Training is designed for the GPU. On the CPU it gives the same result, but expect it to take about an hour.
%%
%[text] ## Set Up
useGPU = canUseGPU()
%%
%[text] ## Create an Oriented Point Cloud
%[text] Sample 10,000 points evenly over the outer surface of a fixed-wing aircraft model, with the outward normal of the triangle each point lies on. M-plicits needs normals that point *out* of the shape. For a scanned cloud, estimate them with `pcnormals` and check their orientation.
[points, normals] = samplePointCloudFromMesh("fixedwing", NumPoints=10000, ...
    Method="bluenoise", ExteriorOnly=true, Normalize=false, Seed=0);
ptCloud = pointCloud(points, Normal=normals);
pcshow(ptCloud, MarkerSize=10)
title("Oriented point cloud (" + ptCloud.Count + " points)")
%%
%[text] ## Reconstruct the Surface
%[text] `pc2mplicitmesh` trains the three levels in turn and prints its progress, then extracts the zero level set with marching cubes. It also removes tiny disconnected blobs, which neural fields sometimes leave far from the data.
[mplicitMesh, info] = pc2mplicitmesh(ptCloud);
figure
showMesh(mplicitMesh, "M-plicits reconstruction")
%%
%[text] ## Look Inside the Field
%[text] The mesh is only the zero level set. The second output, `info`, gives access to the whole field: `info.Field(X)` evaluates it at the N-by-3 points `X`, in the coordinates of the point cloud, and `info.Field(X,k)` uses only the first `k` levels.
%[text] Evaluate the field on two planes: a cut across the tail, perpendicular to the fuselage, through the fuselage, the vertical fin, and the two horizontal stabilizers; and a horizontal cut through the wings, seen from above, that shows the whole aircraft.
lo = min(points);
hi = max(points);
inTail = points(:, 1) <= lo(1) + 0.15 * (hi(1) - lo(1));
xSlice = median(points(inTail, 1));
pad = 0.05 * norm(hi - lo);
yLim = [min(points(inTail, 2)) max(points(inTail, 2))] + [-pad pad];
zLim = [min(points(inTail, 3)) max(points(inTail, 3))] + [-pad pad];
[Y, Z] = meshgrid(linspace(yLim(1), yLim(2), 400), linspace(zLim(1), zLim(2), 400));
slicePoints = [xSlice * ones(numel(Y), 1), Y(:), Z(:)];

numLevels = numel(info.Model.Stages);
F = cell(1, numLevels);
for k = 1:numLevels
    F{k} = reshape(info.Field(slicePoints, k), size(Y));
end

% Horizontal plane through the middle of the wings and stabilizers, the
% thin parts far from the fuselage. A plane that grazes their flat skins
% would run along the surface instead of across it.
span = abs(points(:, 2));
thinParts = span > 0.4 * max(span);
zSlice = mean([min(points(thinParts, 3)) max(points(thinParts, 3))]);
[Xtop, Ytop] = meshgrid(linspace(lo(1) - pad, hi(1) + pad, 400), ...
    linspace(lo(2) - pad, hi(2) + pad, 400));
Ftop = reshape(info.Field([Xtop(:), Ytop(:), zSlice * ones(numel(Xtop), 1)]), size(Xtop));
%[text] The field is an approximate signed distance: zero on the surface, negative inside, positive outside, and its value is the distance to the surface. The evenly spaced contours show this; the black curve is the zero level set, the surface itself.
figure
tiledlayout(1, 2, TileSpacing="compact")
nexttile
showField(Xtop, Ytop, Ftop, "From above, through the wings", ["x" "y"])
nexttile
showField(Y, Z, F{end}, "Across the tail", ["y" "z"])
%%
%[text] ## See What Each Residual Adds
%[text] Each residual $R\_k$ is added only where the coarser field $F\_{k-1}$ is within $\\delta\_k$ of zero, that is, in a thin band around the surface. Everywhere else the finer level leaves the field unchanged. `info.BandWidths` holds the band half-widths $\\delta\_k$, in the units of the point cloud.
info.BandWidths
%[text] So the difference between two consecutive levels, $F\_k - F\_{k-1}$, is the residual $R\_k$ inside its band and exactly zero outside it. Plot it on the same tail slice. The solid black curve is the final surface, and the dashed curves are the edges of the band.
figure
tiledlayout(1, numLevels - 1, TileSpacing="compact")
for k = 2:numLevels
    nexttile
    R = F{k} - F{k - 1};
    showSlice(Y, Z, R, sprintf("Residual R_%d", k - 1), ["y" "z"])
    hold on
    contour(Y, Z, F{k - 1}, info.BandWidths(k - 1) * [-1 1], "k--")
    contour(Y, Z, F{end}, [0 0], LineColor="k", LineWidth=1)
    hold off
    fprintf("R_%d: largest correction %.2g, %.1f%% of the shape size\n", ...
        k - 1, max(abs(R), [], "all"), 100 * max(abs(R), [], "all") / norm(hi - lo));
end
%%
%[text] ## See Where the Detail Went
%[text] On the final surface $F = 0$, so the coarse field there equals minus the total correction, $F\_0 = -(R\_1 + R\_2)$. We can color the mesh by it to see where the residual levels did their work: mostly on the wings, fins, and edges, where the coarse field is too smooth.
correction = -info.Field(mplicitMesh.Vertices, 1);
figure
showMesh(mplicitMesh, "Correction added by the residuals", correction)
%%
%[text] ## Compare with Screened Poisson
%[text] Screened Poisson reconstruction from `pc2surfacemesh` fits one global field.
tic
poissonMesh = pc2surfacemesh(ptCloud, "poisson", 8);
poissonTime = toc
figure
tiledlayout(2, 1, TileSpacing="compact")
nexttile
showMesh(poissonMesh, sprintf("Screened Poisson (%.0f s)", poissonTime))
nexttile
showMesh(mplicitMesh, sprintf("M-plicits (%.0f s)", info.TrainTime + info.MeshTime))
%%
%[text] ## Helper Functions
function showMesh(mesh, titleText, vertexData)
% Shaded surface mesh in the current axes, optionally colored by signed
% per-vertex data. Outward vertex normals make inside-out patches render
% dark.
computeNormals(mesh, "vertex");
if nargin > 2 && ~isempty(vertexData)
    color = {"FaceVertexCData", vertexData, "FaceColor", "interp"};
else
    color = {"FaceColor", [0.75 0.78 0.85]};
end
patch("Faces", mesh.Faces, "Vertices", mesh.Vertices, ...
    "VertexNormals", mesh.VertexNormals, color{:}, "EdgeColor", "none", ...
    "FaceLighting", "gouraud", "BackFaceLighting", "unlit");
axis equal tight off
view(-35, 30)
camlight headlight
if nargin > 2 && ~isempty(vertexData)
    lim = prctile(abs(vertexData), 99);
    colormap(gca, divergingColormap)
    clim([-lim lim])
    colorbar
end
title(titleText)
end

function showField(A, B, V, titleText, axisNames)
% Signed field on a slice, with distance contours and the zero level set.
showSlice(A, B, V, titleText, axisNames)
hold on
contour(A, B, V, 20, LineColor=[0.4 0.4 0.4], LineWidth=0.25)
contour(A, B, V, [0 0], LineColor="k", LineWidth=1.5)
hold off
end

function showSlice(A, B, V, titleText, axisNames)
% Signed values on a slice sampled on the meshgrid A, B, with zero in white.
imagesc(A(1, :), B(:, 1), V)
axis xy equal tight
lim = max(abs(V), [], "all");
clim([-lim lim])
colormap(gca, divergingColormap)
colorbar
xlabel(axisNames(1))
ylabel(axisNames(2))
title(titleText)
end

function showLevelContours(info, x, center, halfWidth, slicePts, titleText)
% Zero level set of each level in a square window on the slice x = const.
[Y, Z] = meshgrid(linspace(center(1) - halfWidth, center(1) + halfWidth, 300), ...
    linspace(center(2) - halfWidth, center(2) + halfWidth, 300));
X = [x * ones(numel(Y), 1), Y(:), Z(:)];
numLevels = numel(info.Model.Stages);
colors = lines(numLevels);
names = ["F_0", "F_0 + R_1", "F_0 + R_1 + R_2"];
hold on
for k = 1:numLevels
    Fk = reshape(info.Field(X, k), size(Y));
    contour(Y, Z, Fk, [0 0], LineColor=colors(k, :), LineWidth=1.5, ...
        DisplayName=names(k));
end
plot(slicePts(:, 1), slicePts(:, 2), "k.", MarkerSize=10, DisplayName="Samples")
hold off
axis equal
xlim(center(1) + [-halfWidth halfWidth])
ylim(center(2) + [-halfWidth halfWidth])
box on
xlabel("y")
ylabel("z")
title(titleText)
end

function cmap = divergingColormap
% Blue for negative (inside), white for zero, red for positive (outside).
cmap = interp1([-1 0 1], [0.23 0.30 0.75; 1 1 1; 0.71 0.02 0.15], ...
    linspace(-1, 1, 256));
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
