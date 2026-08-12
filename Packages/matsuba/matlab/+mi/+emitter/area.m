function s = area(options)
%MI.EMITTER.AREA Create an area light emitter descriptor.
%   S = MI.EMITTER.AREA() creates a white area emitter.
%   S = MI.EMITTER.AREA(radiance=mi.rgb(1,0.8,0.6)) sets the radiance.
    arguments
        options.radiance struct = struct([])
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("area", "emitter", options);
end
