function s = roughconductor(options)
%MI.BSDF.ROUGHCONDUCTOR Create a rough conductor BSDF descriptor.
%   S = MI.BSDF.ROUGHCONDUCTOR(material="Cu", alpha=0.1) creates rough copper.
%   S = MI.BSDF.ROUGHCONDUCTOR(Name=Value) sets additional properties.
%   The alpha parameter also accepts a texture struct for spatially-varying roughness.
    arguments
        options.alpha = []
        options.distribution string = string.empty
        options.material string = string.empty
        options.specular_reflectance struct = struct([])
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("roughconductor", "bsdf", options);
end
