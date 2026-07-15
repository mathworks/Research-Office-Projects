function s = spot(options)
%MI.EMITTER.SPOT Create a spot light emitter descriptor.
%   S = MI.EMITTER.SPOT(cutoff_angle=30) creates a spot light.
%   S = MI.EMITTER.SPOT(Name=Value) sets additional properties.
    arguments
        options.intensity struct = struct([])
        options.cutoff_angle double = []
        options.beam_width double = []
        options.to_world double = []
        options.texture struct = struct([])
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("spot", "emitter", options);
end
