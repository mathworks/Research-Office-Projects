function s = diffuse(options)
%MI.BSDF.DIFFUSE Create a diffuse (Lambertian) BSDF descriptor.
%   S = MI.BSDF.DIFFUSE() creates a white diffuse material.
%   S = MI.BSDF.DIFFUSE(reflectance=mi.rgb(0.5,0.1,0.1)) sets the color.
    arguments
        options.reflectance struct = struct([])
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("diffuse", "bsdf", options);
end
