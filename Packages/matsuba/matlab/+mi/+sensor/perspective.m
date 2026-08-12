function s = perspective(options)
%MI.SENSOR.PERSPECTIVE Create a perspective camera sensor descriptor.
%   S = MI.SENSOR.PERSPECTIVE(fov=45) creates a perspective camera.
%   S = MI.SENSOR.PERSPECTIVE(Name=Value) sets additional properties.
    arguments
        options.fov double = []
        options.fov_axis string = string.empty
        options.near_clip double = []
        options.far_clip double = []
        options.to_world double = []
        options.film struct = struct([])
        options.sampler struct = struct([])
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("perspective", "sensor", options);
end
