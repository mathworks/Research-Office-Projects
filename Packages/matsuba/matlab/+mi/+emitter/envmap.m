function s = envmap(options)
%MI.EMITTER.ENVMAP Create an environment map emitter descriptor.
%   S = MI.EMITTER.ENVMAP(filename="sky.exr") loads an HDR environment map.
%   S = MI.EMITTER.ENVMAP(Name=Value) sets additional properties.
%
%   Example:
%       env = mi.emitter.envmap(filename="nebula.hdr", scale=1.5);
    arguments
        options.filename string = string.empty
        options.scale double = []
        options.to_world double = []
        options.key_ string = string.empty
    end

    % Resolve filename to absolute path so Mitsuba can find it
    if ~isempty(options.filename)
        f = char(options.filename);
        if ~isAbsolutePath(f)
            f = fullfile(pwd, f);
        end
        if ~isfile(f)
            error("mi:emitter:envmap:FileNotFound", ...
                "Environment map file not found: %s", f);
        end
        options.filename = string(f);
    end

    s = mi.internal.pluginStruct("envmap", "emitter", options);
end

function tf = isAbsolutePath(p)
    tf = startsWith(p, '/') || startsWith(p, '\') || ...
         (length(p) >= 2 && p(2) == ':');
end
