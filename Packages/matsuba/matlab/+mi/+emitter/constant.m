function s = constant(options)
%MI.EMITTER.CONSTANT Create a constant environment emitter descriptor.
%   S = MI.EMITTER.CONSTANT() creates a uniform white environment light.
%   S = MI.EMITTER.CONSTANT(radiance=mi.rgb(0.5,0.5,0.8)) sets the color.
    arguments
        options.radiance struct = struct([])
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("constant", "emitter", options);
end
