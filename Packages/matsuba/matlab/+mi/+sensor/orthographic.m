function s = orthographic(options)
%MI.SENSOR.ORTHOGRAPHIC Create an orthographic camera sensor descriptor.
%   S = MI.SENSOR.ORTHOGRAPHIC() creates an orthographic camera.
%   S = MI.SENSOR.ORTHOGRAPHIC(Name=Value) sets additional properties.
    arguments
        options.near_clip double = []
        options.far_clip double = []
        options.to_world double = []
        options.film struct = struct([])
        options.sampler struct = struct([])
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("orthographic", "sensor", options);
end
