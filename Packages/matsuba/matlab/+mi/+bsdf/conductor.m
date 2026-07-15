function s = conductor(options)
%MI.BSDF.CONDUCTOR Create a smooth conductor (metal) BSDF descriptor.
%   S = MI.BSDF.CONDUCTOR(material="Au") creates a gold conductor.
%   S = MI.BSDF.CONDUCTOR(Name=Value) sets additional properties.
    arguments
        options.material string = string.empty
        options.specular_reflectance struct = struct([])
        options.eta struct = struct([])
        options.k struct = struct([])
        options.to_world double = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("conductor", "bsdf", options);
end
