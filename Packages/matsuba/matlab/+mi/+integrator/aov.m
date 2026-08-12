function s = aov(aovs, subIntegrator, options)
%MI.INTEGRATOR.AOV Create an AOV (Arbitrary Output Variable) integrator.
%   S = MI.INTEGRATOR.AOV(AOVS) wraps the default path integrator and
%   adds the specified AOV channels to the render output.
%
%   S = MI.INTEGRATOR.AOV(AOVS, SUB) wraps the given sub-integrator.
%
%   AOVS is a string array of AOV type names. Supported types:
%     "depth"       - Distance from camera (1 channel)
%     "position"    - World-space hit position (3 channels)
%     "uv"          - Texture coordinates (2 channels)
%     "geo_normal"  - Geometry normal (3 channels)
%     "sh_normal"   - Shading normal (3 channels)
%     "dp_du"       - Position partial derivative (3 channels)
%     "dp_dv"       - Position partial derivative (3 channels)
%     "prim_index"  - Primitive index (1 channel)
%     "shape_index" - Shape index (1 channel)
%
%   The friendly alias "normals" maps to "sh_normal".
%
%   Example:
%       scene = mi.Scene.build( ...
%           mi.shape.sphere(), ...
%           mi.emitter.constant(), ...
%           mi.integrator.aov(["depth", "normals"], mi.integrator.path()));
%
%   See also mi.Scene.renderAOV
    arguments
        aovs (1,:) string
        subIntegrator (1,1) struct = mi.integrator.path()
        options.key_ string = string.empty
    end

    % Build AOV specification string
    aovStr = buildAovString(aovs);

    s.type = "aov";
    s.category_ = "integrator";
    s.aovs = char(aovStr);
    s.sub = subIntegrator;
    if ~isempty(options.key_)
        s.key_ = options.key_;
    end
end

function str = buildAovString(aovs)
%BUILDAOVSTRING Convert AOV names to Mitsuba's "name:type" format.
    parts = strings(1, numel(aovs));
    for i = 1:numel(aovs)
        [name, type] = resolveAov(aovs(i));
        parts(i) = name + ":" + type;
    end
    str = join(parts, ",");
end

function [name, type] = resolveAov(aov)
%RESOLVEAOV Map a user-facing AOV name to a Mitsuba output name and type.
    aliases = struct("normals", "sh_normal");
    if isfield(aliases, aov)
        aov = aliases.(aov);
    end
    info = aovInfo();
    if ~isfield(info, aov)
        error("mi:integrator:aov:UnknownAOV", ...
            "Unknown AOV type '%s'. Supported: %s", ...
            aov, strjoin(string(fieldnames(info)), ", "));
    end
    name = info.(aov).name;
    type = string(aov);
end

function info = aovInfo()
%AOVINFO Return metadata for each supported AOV type.
%   Output names are the short identifiers used in Mitsuba's AOV string.
%   Channel counts are sourced from mi.internal.aovChannelCount.
    persistent cached
    if isempty(cached)
        names = struct( ...
            "depth", "depth", "position", "pos", "uv", "uv", ...
            "geo_normal", "gn", "sh_normal", "nn", ...
            "dp_du", "dpdu", "dp_dv", "dpdv", ...
            "prim_index", "pi", "shape_index", "si");
        counts = mi.internal.aovChannelCount();
        types = fieldnames(names);
        cached = struct();
        for i = 1:numel(types)
            t = types{i};
            cached.(t) = struct("name", names.(t), "channels", counts.(t));
        end
    end
    info = cached;
end
