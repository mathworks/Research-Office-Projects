function hits = hmmInvalidRayHits(cfg)
%HMMINVALIDRAYHITS An all-invalid RayHits value for a camera with no ray path.
%
%   hits = HMMINVALIDRAYHITS(cfg) returns a RayHits-shaped struct in which
%   every ray is marked invalid and GridSize is [0 0].
%
%   The wrist camera uses this. Simulation 3D Ray Tracer exposes its rays as
%   dialog parameters and has no external mount inputs, so it cannot be
%   carried on a moving wrist frame; the wrist camera therefore has render
%   depth only. Rather than leave the Calib element of its CameraFrame
%   undefined - where a consumer could read zeros as "range 0 m, dead ahead" -
%   it carries an explicitly invalid value. Combined with
%   DepthSource = SOURCE_RENDER, a consumer that checks either field cannot
%   mistake the wrist camera's depth for a metric measurement.
%
%   See also hmmRayDirections, hmmDefineBuses, hmmConfig.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg struct = hmmConfig()
end

n = cfg.depth.rayRows;

hits.HitDistances = zeros(n, 1, 'single');
hits.IsValidHit   = false(n, 1);
hits.HitLocations = zeros(n, 3, 'single');
hits.SurfaceIds   = zeros(n, 1, 'uint32');
hits.GridSize     = zeros(2, 1, 'uint16');

end
