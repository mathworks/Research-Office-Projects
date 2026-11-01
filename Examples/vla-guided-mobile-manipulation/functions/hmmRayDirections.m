function rays = hmmRayDirections(cfg)
%HMMRAYDIRECTIONS Ray grid for the calibrated depth path of a fixed camera.
%
%   rays = HMMRAYDIRECTIONS(cfg) returns the compile-time ray set for
%   Simulation 3D Ray Tracer, sampling the same pinhole camera model as the
%   fixed Simulation 3D Camera blocks on a coarser grid.
%
%   The rays are PARAMETERS of the Ray Tracer block, not inputs: the block
%   exposes RayOrigins, RayDirections and RayLengths as dialog parameters and
%   has no external mount inputs at all. That is why the calibrated depth path
%   serves the four FIXED cameras only - a wrist-mounted ray tracer is not
%   expressible with this block - and why the wrist camera reports
%   DepthSource = SOURCE_RENDER instead.
%
%   Why a separate calibrated path exists: it was built when the render depth from
%   Simulation 3D Camera was believed to be quantised to whole metres. That was
%   wrong - hmmBackprojectDepth records the measured step, 59.6 um - but the ray
%   path is still what the fixed cameras use: its range is metrically exact and it
%   flags rays that hit nothing.
%
%   Returned fields:
%       .origins     [nRays 3] all zeros - rays leave the sensor origin
%       .directions  [nRays 3] unit directions in the SENSOR frame
%       .lengths     [nRays 1] cfg.depth.rayLength
%       .gridSize    uint16 [2 1], [rows cols] of the sampled grid
%       .pixelRow    [nRays 1] image row each ray samples (1 = top)
%       .pixelCol    [nRays 1] image column each ray samples
%       .nRays       scalar
%
%   FRAME AND ORDER CONVENTIONS (both documented here so nothing downstream
%   re-derives them):
%
%     Sensor frame: +X is the optical axis, +Y is image RIGHT, +Z is image UP,
%     right-handed, consistent with cfg.frames.worldUp = [0 0 1]. A pixel
%     offset therefore maps to direction [1, xn, -yn] normalised, where xn
%     and yn are the normalised image-right and image-DOWN offsets. Image row
%     1 is the top row (cfg.frames.imageOrigin), so increasing row is -Z.
%
%     Ray order: row-major over the sampled grid, index = (r-1)*cols + c.
%     A consumer reshapes a returned column to [rows cols] with the same
%     row-major reading, which is why GridSize travels on the RayHits bus.
%
%   THE +Y SIGN WAS WRONG FOR THE WHOLE HISTORY OF THE PROJECT AND IS FIXED HERE.
%   This header used to say "+Y is image LEFT" and build [1, -xn, -yn] from it,
%   which mirrored every ray's pixel label about the vertical centreline: ray i
%   pointed at column nCols+1-c while claiming column c, so
%   hmmReconstructTargets associated a grounded pixel with rays from the wrong
%   half of the frame. Two measurements settle it, both against ground truth
%   rather than against this file:
%
%     1. Forward-projecting Mug_Red's manifest position [3.8655 -3.0500 0.5950]
%        into LivingRoomCam puts it at row 131.56, col 242.11. The shipped
%        replay fixture's wide-call pixel is row 131.40, col 398.08 - the row
%        agrees to 0.2 px and 641 - 398.08 = 242.92, an exact column mirror.
%        The fixtures had been AUTHORED mirrored to compensate, so the two
%        mirrors nearly cancelled and the reconstruction landed 49.1 mm from the
%        mug instead of failing outright, which is why this survived so long.
%     2. The same projection against the wrist depth path, which never uses
%        rays, is NOT mirrored: the manifest point lands 12.6 px from the mug's
%        visible axis (located three independent ways from the depth image
%        alone) where the mirrored column would be 53 px away.
%
%   hmmBackprojectDepth's header carries the closure test that first exposed
%   this and recorded it as "a defect in the shipped fixed-camera
%   reconstruction and a separate decision". This is that decision. The Z sign
%   was always right and is unchanged; only the Y sign moved, and the replay
%   corpus was re-baked from true projections in the same pass, so the coarse
%   estimate and the wrist refinement now speak one convention.
%
%   See also hmmConfig, hmmInvalidRayHits, hmmDefineBuses.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg struct = hmmConfig()
end

nRows = cfg.depth.rayGrid(1);
nCols = cfg.depth.rayGrid(2);
imgRows = cfg.camera.imageSize(1);
imgCols = cfg.camera.imageSize(2);

fx = cfg.camera.focalLength(1);
fy = cfg.camera.focalLength(2);
cx = cfg.camera.opticalCenter(1);
cy = cfg.camera.opticalCenter(2);

% Sample pixel centres spanning the full image, not the pixel corners: a grid
% that starts at row 1 and ends at row imgRows would weight the image edges
% twice as heavily as its interior.
rowEdges = linspace(0, imgRows, nRows + 1);
colEdges = linspace(0, imgCols, nCols + 1);
rowCentres = (rowEdges(1:end-1) + rowEdges(2:end)) / 2;
colCentres = (colEdges(1:end-1) + colEdges(2:end)) / 2;

[colGrid, rowGrid] = meshgrid(colCentres, rowCentres);
pixelRow = reshape(rowGrid', [], 1);   % transpose gives row-major order
pixelCol = reshape(colGrid', [], 1);

xn = (pixelCol - cx) / fx;   % normalised image-right offset
yn = (pixelRow - cy) / fy;   % normalised image-down offset

% +Y is image RIGHT so the Y component carries xn unnegated; +Z is image UP and
% yn is an image-DOWN offset, so Z carries -yn. This matches, component for
% component, the pSensor that hmmBackprojectDepth builds for the wrist depth
% path: depth * [1; (col - cx)/fx; -(row - cy)/fy]. The two paths must agree,
% because a coarse ray estimate and a refined depth estimate of the same object
% are differenced against cfg.refine.maxShift.
dirs = [ones(size(xn)), xn, -yn];
dirs = dirs ./ vecnorm(dirs, 2, 2);

rays.nRays      = nRows * nCols;
rays.origins    = zeros(rays.nRays, 3);
rays.directions = dirs;
rays.lengths    = cfg.depth.rayLength * ones(rays.nRays, 1);
rays.gridSize   = uint16([nRows; nCols]);
rays.pixelRow   = pixelRow;
rays.pixelCol   = pixelCol;

assert(rays.nRays == cfg.depth.nRays, 'hmmRayDirections:rayCount', ...
    'Grid produced %d rays but cfg.depth.nRays is %d.', ...
    rays.nRays, cfg.depth.nRays);
assert(all(abs(vecnorm(rays.directions, 2, 2) - 1) < 1e-12), ...
    'hmmRayDirections:notUnit', 'Ray directions are not unit vectors.');

end
