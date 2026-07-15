function s = point(options)
%MI.EMITTER.POINT Create a point light emitter descriptor.
%   S = MI.EMITTER.POINT(position=[0 2 0]) creates a point light.
%   S = MI.EMITTER.POINT(Name=Value) sets additional properties.
    arguments
        options.position double = []
        options.intensity struct = struct([])
        options.to_world double = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("point", "emitter", options);
end
