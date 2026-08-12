function s = twosided(options)
%MI.BSDF.TWOSIDED Create a two-sided BSDF adapter descriptor.
%   S = MI.BSDF.TWOSIDED(bsdf=mi.bsdf.diffuse()) wraps a BSDF for both sides.
%   S = MI.BSDF.TWOSIDED(Name=Value) sets additional properties.
    arguments
        options.bsdf struct = struct([])
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("twosided", "bsdf", options);
end
