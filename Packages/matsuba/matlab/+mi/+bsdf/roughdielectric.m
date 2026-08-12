function s = roughdielectric(options)
%MI.BSDF.ROUGHDIELECTRIC Create a rough dielectric BSDF descriptor.
%   S = MI.BSDF.ROUGHDIELECTRIC(alpha=0.1, int_ior=1.5) creates frosted glass.
%   S = MI.BSDF.ROUGHDIELECTRIC(Name=Value) sets additional properties.
    arguments
        options.alpha = []
        options.distribution string = string.empty
        options.int_ior double = []
        options.ext_ior double = []
        options.specular_reflectance struct = struct([])
        options.specular_transmittance struct = struct([])
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("roughdielectric", "bsdf", options);
end
