%% MATLAB Logo Material Showcase — MATSUBA Example
% Renders the MATLAB membrane logo in three materials side by side:
%   1. Glass   — smooth dielectric with warm tint
%   2. Metal   — polished gold
%   3. Plastic — glossy red
%
% See also: membrane, mi.shape.fromMesh, mi.bsdf.dielectric, mi.bsdf.conductor

%% 1. Setup
addpath(fullfile(fileparts(mfilename("fullpath")), ".."));
mi.setup;

%% 2. Generate the MATLAB logo mesh from membrane()
L = 160 * membrane(1, 20);  % 41x41 grid — fast to render
[V, F] = logoMesh(L);

%% 3. Define materials
materials = { ...
    "Glass",   mi.bsdf.dielectric(int_ior=1.5, ...
                   specular_transmittance=mi.rgb([1.0 0.6 0.3])); ...
    "Metal",   mi.bsdf.roughconductor(material="Au", alpha=0.1); ...
    "Plastic", mi.bsdf.plastic( ...
                   diffuse_reflectance=mi.rgb([0.8 0.2 0.1]), ...
                   int_ior=1.5) ...
};
nMat = size(materials, 1);

%% 4. Render each material
tileSize = 256;
spp = 256;
tiles = cell(nMat, 1);

for i = 1:nMat
    fprintf("Rendering %s (%d/%d)...\n", materials{i,1}, i, nMat);
    scene = buildScene(V, F, materials{i,2}, tileSize);
    if i == 1  % Glass — let convergence decide when noise is low enough
        hdr = scene.renderProgressive(SamplesPerPixel=spp*4, PassSize=64, Convergence=true);
        tiles{i} = mi.postprocess(hdr, FireflyClamp=96, Denoise="median");
    else
        hdr = scene.renderProgressive(SamplesPerPixel=spp, PassSize=64);
        tiles{i} = mi.postprocess(hdr);
    end
    delete(scene);
end
fprintf("All renders complete.\n");

%% 5. Assemble 1×3 montage
montage = cat(2, tiles{:});

figure("Name", "MATLAB Logo — Material Showcase", ...
    Position=[100 100 900 340]);
imshow(montage);
for i = 1:nMat
    text(tileSize*(i-0.5), tileSize*0.06, materials{i,1}, ...
        HorizontalAlignment="center", FontSize=14, ...
        FontWeight="bold", Color="w");
end
title("MATLAB Logo — Material Showcase");

%% 6. Save output
outPath = fullfile(fileparts(mfilename("fullpath")), "matlab_logo_materials.png");
exportgraphics(gcf, outPath, Resolution=150);
fprintf("Saved to %s\n", outPath);
fprintf("Done!\n");


%% --- Local functions ---

function scene = buildScene(V, F, logoBsdf, filmSize)
%BUILDSCENE Construct the Mitsuba scene with the given logo material.

    logoShape = mi.shape.fromMesh(V, F, bsdf=logoBsdf, key_="logo");

    % Floor — large light surface for the logo to sit on
    floorMat = mi.bsdf.diffuse(reflectance=mi.rgb([0.55 0.55 0.58]));
    floorT = mi.Transform.lookAt([0 0 0], [0 1 0], [0 0 1]) ...
           * mi.Transform.scale([10 10 1]);
    floorShape = mi.shape.rectangle(bsdf=floorMat, to_world=floorT, key_="floor");

    % Area lights — three-quarter key + soft fill
    keyLightT = mi.Transform.lookAt([2.5 4.5 2], [0 0 0], [0 0 1]) ...
              * mi.Transform.scale([0.5 0.5 1]);
    keyLight = mi.shape.rectangle( ...
        emitter=mi.emitter.area(radiance=mi.rgb([600 540 440])), ...
        to_world=keyLightT, key_="key_light");
    fillLightT = mi.Transform.lookAt([-2 3.5 1.5], [0 0 0], [0 0 1]) ...
               * mi.Transform.scale([0.4 0.4 1]);
    fillLight = mi.shape.rectangle( ...
        emitter=mi.emitter.area(radiance=mi.rgb([80 90 110])), ...
        to_world=fillLightT, key_="fill_light");

    % Dark environment with enough brightness for clean metal reflections
    envLight = mi.emitter.constant( ...
        radiance=mi.rgb([0.04 0.04 0.05]), key_="env");

    camera = mi.sensor.perspective( ...
        fov=38, ...
        to_world=mi.Transform.lookAt([3.2 2.8 3.2], [0 0.2 0], [0 1 0]), ...
        film=mi.film(Width=filmSize, Height=filmSize), ...
        key_="sensor");

    integrator = mi.integrator.path(max_depth=12);

    scene = mi.Scene.build( ...
        logoShape, floorShape, ...
        keyLight, fillLight, envLight, ...
        camera, integrator);
end

function [V, F] = logoMesh(L)
%LOGOMESH Build a solid prism from the membrane surface.
%   Creates a closed mesh: membrane top + flat bottom + side walls.

    [ny, nx] = size(L);
    [X, Y] = meshgrid(linspace(-1, 1, nx), linspace(-1, 1, ny));

    Z = L / max(abs(L(:))) * 0.8;
    V_top = [X(:), Z(:), Y(:)];
    heights = Z(:);
    nVerts = size(V_top, 1);

    % Triangulate the grid
    allF = zeros(2 * (ny - 1) * (nx - 1), 3);
    idx = 0;
    for row = 1:(ny - 1)
        for col = 1:(nx - 1)
            v1 = (row - 1) * nx + col;
            v2 = v1 + 1;
            v3 = row * nx + col;
            v4 = v3 + 1;
            idx = idx + 1;
            allF(idx, :) = [v1 v2 v4];
            idx = idx + 1;
            allF(idx, :) = [v1 v4 v3];
        end
    end

    % Remove faces in the zero-height quadrant (the missing part of the L)
    keep = false(idx, 1);
    for i = 1:idx
        if any(abs(heights(allF(i, :))) > 1e-6)
            keep(i) = true;
        end
    end
    topF = allF(keep, :);

    % Bottom surface: constant offset below top for uniform thickness
    thickness = 0.1;
    V_bottom = V_top;
    V_bottom(:, 2) = V_top(:, 2) - thickness;
    bottomF = topF(:, [1 3 2]) + nVerts;

    % Side walls: use directed boundary edges for consistent outward winding.
    edges = [topF(:,[1 2]); topF(:,[2 3]); topF(:,[3 1])];
    isBoundary = ~ismember(edges, edges(:,[2 1]), "rows");
    directedBE = edges(isBoundary, :);

    nBE = size(directedBE, 1);
    sideF = zeros(2 * nBE, 3);
    for i = 1:nBE
        a = directedBE(i, 1);
        b = directedBE(i, 2);
        c = a + nVerts;
        d = b + nVerts;
        sideF(2*i-1, :) = [a b d];
        sideF(2*i,   :) = [a d c];
    end

    V = [V_top; V_bottom];
    F = [topF; bottomF; sideF];

    % Remove orphan vertices (from the deleted zero-height quadrant) so that
    % every vertex is referenced by at least one face. Orphan vertices cause
    % invalid zero-length normals when smooth normals are computed.
    usedVerts = unique(F(:));
    newIdx = zeros(size(V, 1), 1);
    newIdx(usedVerts) = 1:numel(usedVerts);
    V = V(usedVerts, :);
    F = newIdx(F);

    % Raise so bottom surface sits at floor level (Y=0)
    V(:,2) = V(:,2) + 0.1;
end
