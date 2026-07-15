function s = dielectric(options)
%MI.BSDF.DIELECTRIC Create a smooth dielectric BSDF descriptor.
%   S = MI.BSDF.DIELECTRIC(int_ior=1.5) creates a glass-like material.
%   S = MI.BSDF.DIELECTRIC(Name=Value) sets additional properties.
    arguments
        options.int_ior double = []
        options.ext_ior double = []
        options.specular_reflectance struct = struct([])
        options.specular_transmittance struct = struct([])
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("dielectric", "bsdf", options);
end
