function [origins, directions] = panoramic(u, v, options)
%MI.SENSOR.PANORAMIC Generate ray maps for a panoramic (360) camera.
%   [ORIGINS, DIRECTIONS] = MI.SENSOR.PANORAMIC(U, V) generates per-pixel
%   ray origins and directions for a full 360x180 equirectangular projection.
%   U and V are H-by-W matrices of normalized pixel coordinates in [0, 1].
%
%   [...] = MI.SENSOR.PANORAMIC(U, V, Projection=...) specifies the projection:
%       "equirectangular" - Standard lat/lon mapping (default)
%       "cylindrical"     - Cylindrical projection (360 horizontal, limited vertical)
%
%   [...] = MI.SENSOR.PANORAMIC(U, V, HorizontalFOV=360, VerticalFOV=180)
%   sets the angular range in degrees.
%
%   The camera looks along +Z with Y up (Mitsuba's local camera frame).
%   The horizontal angle sweeps left (+X) to right (-X), and the vertical
%   angle goes from top (+Y) to bottom (-Y).
%
%   Usage:
%       cam = mi.sensor.custom(RayFunction=@mi.sensor.panoramic, ...
%           film=mi.film(Width=1024, Height=512));
%
%   See also mi.sensor.custom, mi.sensor.fisheye

    arguments
        u double
        v double
        options.Projection (1,1) string = "equirectangular"
        options.HorizontalFOV (1,1) double = 360
        options.VerticalFOV (1,1) double = 180
    end

    [H, W] = size(u);

    hFov = deg2rad(options.HorizontalFOV);
    vFov = deg2rad(options.VerticalFOV);

    % Map u -> azimuth (horizontal angle), v -> elevation (vertical angle)
    % Negate azimuth: Mitsuba's +X is "left", so u=0 (left edge) -> positive azimuth
    % v=0 is top edge -> positive elevation (+Y is up)
    azimuth = (0.5 - u) * hFov;      % [hFov/2, -hFov/2]
    elevation = (0.5 - v) * vFov;    % [vFov/2, -vFov/2] (top=positive)

    switch lower(options.Projection)
        case "equirectangular"
            % Standard spherical mapping (+Z forward in Mitsuba local frame)
            dx = sin(azimuth) .* cos(elevation);
            dy = sin(elevation);
            dz = cos(azimuth) .* cos(elevation);
        case "cylindrical"
            % Cylindrical: horizontal is equidistant, vertical is linear
            dx = sin(azimuth);
            dy = tan(elevation);
            dz = cos(azimuth);
        otherwise
            error("mi:sensor:panoramic:BadProjection", ...
                "Unknown projection '%s'. Use equirectangular or cylindrical.", ...
                options.Projection);
    end

    % All rays originate from the optical center
    origins = zeros(H, W, 3);
    directions = cat(3, dx, dy, dz);
end
