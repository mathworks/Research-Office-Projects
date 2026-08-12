function s = normalmap(options)
%MI.BSDF.NORMALMAP Wrap a BSDF with a normal map.
%   S = MI.BSDF.NORMALMAP(normalmap=TEX, bsdf=INNER) applies a tangent-
%   space normal map texture to an existing BSDF.
%
%   Example:
%       nmap = mi.texture.bitmap("normals.png", raw=true);
%       inner = mi.bsdf.diffuse(reflectance=mi.rgb([0.8 0.2 0.1]));
%       mat = mi.bsdf.normalmap(normalmap=nmap, bsdf=inner);
    arguments
        options.normalmap struct = struct([])
        options.bsdf struct = struct([])
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("normalmap", "bsdf", options);
end
