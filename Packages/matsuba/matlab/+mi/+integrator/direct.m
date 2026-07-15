function s = direct(options)
%MI.INTEGRATOR.DIRECT Create a direct illumination integrator descriptor.
%   S = MI.INTEGRATOR.DIRECT() creates a default direct illumination integrator.
%   S = MI.INTEGRATOR.DIRECT(Name=Value) sets additional properties.
    arguments
        options.shading_samples double = []
        options.emitter_samples double = []
        options.bsdf_samples double = []
        options.hide_emitters logical = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("direct", "integrator", options);
end
