function s = custom(options)
%MI.SENSOR.CUSTOM Create a programmable camera sensor with custom ray maps.
%   S = MI.SENSOR.CUSTOM(RayFunction=@fn) creates a sensor where @fn(u,v)
%   returns per-pixel ray origins and directions. u and v are H-by-W matrices
%   of normalized pixel coordinates in [0, 1].
%
%   S = MI.SENSOR.CUSTOM(Origins=O, Directions=D) creates a sensor from
%   explicit H-by-W-by-3 ray origin and direction arrays.
%
%   Name-Value Arguments:
%       RayFunction     - Function handle @(u,v) returning {origins, directions}
%                         where each is H-by-W-by-3.
%       Origins         - H-by-W-by-3 array of ray origins.
%       Directions      - H-by-W-by-3 array of ray directions (normalized
%                         automatically).
%       ApertureRadius  - Radius of the exit pupil in scene units. Controls
%                         depth of field. Set to 0 (default) for pinhole.
%                         Alternatively compute from f-number and focal length:
%                         aperture_radius = focal_length / (2 * f_number).
%       FocusDistance    - Distance along chief ray where objects are in focus.
%                         Scalar for uniform focus, or H-by-W array for field
%                         curvature. Required when ApertureRadius > 0.
%       Vignetting      - H-by-W array in [0,1] specifying effective aperture
%                         fraction at each pixel. Models natural light falloff
%                         at field edges. Optional.
%       to_world        - 4x4 transform matrix applied to all rays.
%       film            - Film descriptor (from mi.film). Required to determine
%                         resolution when using RayFunction.
%       sampler         - Sampler descriptor (from mi.sampler).
%       key_            - Custom key name for this sensor in the scene.
%
%   Examples:
%       % Pinhole fisheye (no DOF)
%       cam = mi.sensor.custom( ...
%           RayFunction=@mi.sensor.fisheye, ...
%           film=mi.film(Width=512, Height=512));
%
%       % Fisheye with depth of field (50mm f/2 equivalent)
%       cam = mi.sensor.custom( ...
%           RayFunction=@mi.sensor.fisheye, ...
%           ApertureRadius=0.025, ...
%           FocusDistance=3.0, ...
%           film=mi.film(Width=512, Height=512));
%
%   See also mi.sensor.fisheye, mi.sensor.panoramic, mi.sensor.perspective

    arguments
        options.RayFunction function_handle = function_handle.empty
        options.Origins double = []
        options.Directions double = []
        options.ApertureRadius (1,1) double = 0
        options.FocusDistance double = []
        options.Vignetting double = []
        options.to_world double = []
        options.film struct = struct([])
        options.sampler struct = struct([])
        options.key_ string = string.empty
    end

    hasFunction = ~isempty(options.RayFunction);
    hasArrays = ~isempty(options.Origins) && ~isempty(options.Directions);

    if ~hasFunction && ~hasArrays
        error("mi:sensor:custom:NoRays", ...
            "Provide either RayFunction or both Origins and Directions.");
    end
    if hasFunction && hasArrays
        error("mi:sensor:custom:Ambiguous", ...
            "Provide either RayFunction or Origins/Directions, not both.");
    end

    % Validate DOF parameters
    if options.ApertureRadius > 0 && isempty(options.FocusDistance)
        error("mi:sensor:custom:NoFocus", ...
            "FocusDistance is required when ApertureRadius > 0.");
    end

    % Determine resolution from film or arrays
    if hasArrays
        [H, W, c] = size(options.Origins);
        if c ~= 3
            error("mi:sensor:custom:BadOrigins", ...
                "Origins must be H-by-W-by-3.");
        end
        [Hd, Wd, cd] = size(options.Directions);
        if cd ~= 3 || Hd ~= H || Wd ~= W
            error("mi:sensor:custom:BadDirections", ...
                "Directions must match Origins dimensions (H-by-W-by-3).");
        end
        origins = options.Origins;
        directions = options.Directions;
    else
        % Evaluate RayFunction over the pixel grid
        if isempty(options.film)
            error("mi:sensor:custom:NoFilm", ...
                "film is required when using RayFunction to determine resolution.");
        end
        W = options.film.width;
        H = options.film.height;
        [u, v] = meshgrid( ...
            linspace(0.5/W, 1 - 0.5/W, W), ...
            linspace(0.5/H, 1 - 0.5/H, H));
        [origins, directions] = options.RayFunction(u, v);
        if ~isequal(size(origins), [H, W, 3])
            error("mi:sensor:custom:BadRayOrigins", ...
                "RayFunction must return H-by-W-by-3 origins.");
        end
        if ~isequal(size(directions), [H, W, 3])
            error("mi:sensor:custom:BadRayDirections", ...
                "RayFunction must return H-by-W-by-3 directions.");
        end
    end

    % Build sensor struct
    s.type = "raymap";
    s.category_ = "sensor";

    % Pack ray data for the Python bridge (intercepted by _normalize)
    s.raymap_data_.origins = origins;
    s.raymap_data_.directions = directions;

    % DOF parameters
    if options.ApertureRadius > 0
        s.raymap_data_.aperture_radius = options.ApertureRadius;
        if isscalar(options.FocusDistance)
            s.raymap_data_.focus_distance = options.FocusDistance;
        else
            if ~isequal(size(options.FocusDistance), [H, W])
                error("mi:sensor:custom:BadFocusDistance", ...
                    "FocusDistance map must be H-by-W.");
            end
            s.raymap_data_.focus_distance = options.FocusDistance;
        end
        if ~isempty(options.Vignetting)
            if ~isequal(size(options.Vignetting), [H, W])
                error("mi:sensor:custom:BadVignetting", ...
                    "Vignetting map must be H-by-W.");
            end
            s.raymap_data_.vignetting = options.Vignetting;
        end
    end

    % Optional to_world transform
    if ~isempty(options.to_world)
        s.to_world = options.to_world;
    end

    % Film (ensure resolution matches ray arrays)
    if ~isempty(options.film)
        s.film = options.film;
    else
        s.film = mi.film(Width=W, Height=H);
    end

    % Sampler
    if ~isempty(options.sampler)
        s.sampler = options.sampler;
    end

    % Key
    if ~isempty(options.key_) && strlength(options.key_) > 0
        s.key_ = options.key_;
    end
end
