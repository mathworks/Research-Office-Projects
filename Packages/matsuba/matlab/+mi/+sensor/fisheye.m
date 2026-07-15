function [origins, directions] = fisheye(u, v, options)
%MI.SENSOR.FISHEYE Generate ray maps for a fisheye lens model.
%   [ORIGINS, DIRECTIONS] = MI.SENSOR.FISHEYE(U, V) generates per-pixel ray
%   origins and directions for an equidistant fisheye projection with 180
%   degree field of view. U and V are H-by-W matrices of normalized pixel
%   coordinates in [0, 1].
%
%   [...] = MI.SENSOR.FISHEYE(U, V, Projection=...) specifies the fisheye
%   projection model:
%       "equidistant"   - r = f * theta (default)
%       "equisolid"     - r = 2*f * sin(theta/2)
%       "stereographic" - r = 2*f * tan(theta/2)
%       "orthographic"  - r = f * sin(theta)
%
%   [...] = MI.SENSOR.FISHEYE(U, V, FOV=180) sets the field of view in degrees.
%
%   The camera looks along +Z with Y up (Mitsuba's local camera frame).
%   Rays outside the circular FOV are assigned zero direction (masked out
%   during rendering).
%
%   This function can be used directly as a RayFunction:
%       cam = mi.sensor.custom(RayFunction=@mi.sensor.fisheye, ...
%           film=mi.film(Width=512, Height=512));
%
%   Or with custom parameters via an anonymous function:
%       cam = mi.sensor.custom( ...
%           RayFunction=@(u,v) mi.sensor.fisheye(u,v, FOV=220, Projection="equisolid"), ...
%           film=mi.film(Width=512, Height=512));
%
%   See also mi.sensor.custom, mi.sensor.panoramic

    arguments
        u double
        v double
        options.FOV (1,1) double = 180
        options.Projection (1,1) string = "equidistant"
    end

    [H, W] = size(u);

    % Map pixel coords to centered normalized coords [-1, 1]
    % Negate cx: Mitsuba's local camera +X is "left" (screen left = +X)
    % Negate cy: v=0 is image top, but +Y is up in camera space
    cx = 1 - 2 * u;  % [-1, 1]
    cy = 1 - 2 * v;  % [-1, 1]

    % Radial distance from center (normalized to [0, 1] at image edge)
    r = sqrt(cx.^2 + cy.^2);

    % Maximum theta (half-angle of FOV)
    thetaMax = deg2rad(options.FOV / 2);

    % Map r -> theta based on projection model
    switch lower(options.Projection)
        case "equidistant"
            % r = f * theta => theta = r * thetaMax
            theta = r * thetaMax;
        case "equisolid"
            % r = 2f * sin(theta/2) => theta = 2*asin(r * sin(thetaMax/2))
            sinHalfMax = sin(thetaMax / 2);
            arg = r * sinHalfMax;
            arg = min(arg, 1);  % Clamp for asin domain
            theta = 2 * asin(arg);
        case "stereographic"
            % r = 2f * tan(theta/2) => theta = 2*atan(r * tan(thetaMax/2))
            tanHalfMax = tan(min(thetaMax / 2, pi/2 - 0.001));
            theta = 2 * atan(r * tanHalfMax);
        case "orthographic"
            % r = f * sin(theta) => theta = asin(r * sin(thetaMax))
            sinMax = sin(thetaMax);
            arg = r * sinMax;
            arg = min(arg, 1);  % Clamp for asin domain
            theta = asin(arg);
        otherwise
            error("mi:sensor:fisheye:BadProjection", ...
                "Unknown projection '%s'. Use equidistant, equisolid, stereographic, or orthographic.", ...
                options.Projection);
    end

    % Azimuthal angle from center pixel
    phi = atan2(cy, cx);

    % Convert spherical to Cartesian directions (camera looks along +Z, Y up)
    % Mitsuba's local camera frame: +Z = forward, +Y = up, +X = right
    dx = sin(theta) .* cos(phi);
    dy = sin(theta) .* sin(phi);
    dz = cos(theta);

    % Mask pixels outside the circular FOV
    % Outside pixels get zero direction vector — the sensor returns zero
    % weight for these, producing black pixels.
    mask = r <= 1;
    dx = dx .* mask;
    dy = dy .* mask;
    dz = dz .* mask;

    % All rays originate from the optical center
    origins = zeros(H, W, 3);
    directions = cat(3, dx, dy, dz);
end
