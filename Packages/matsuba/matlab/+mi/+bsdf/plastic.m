function s = plastic(options)
%MI.BSDF.PLASTIC Create a smooth plastic BSDF descriptor.
%   S = MI.BSDF.PLASTIC() creates a white plastic material.
%   S = MI.BSDF.PLASTIC(Name=Value) sets additional properties.
    arguments
        options.diffuse_reflectance struct = struct([])
        options.int_ior double = []
        options.nonlinear logical = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("plastic", "bsdf", options);
end
