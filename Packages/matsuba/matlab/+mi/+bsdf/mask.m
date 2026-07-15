function s = mask(options)
%MI.BSDF.MASK Create a mask (opacity) BSDF wrapper.
%   S = MI.BSDF.MASK(opacity=TEX, bsdf=INNER) makes INNER partially
%   transparent based on the opacity texture. White = opaque, black =
%   fully transparent.
%
%   Example:
%       % Cloud layer: white diffuse where clouds are, transparent elsewhere
%       clouds = mi.texture.bitmap("clouds.jpg");
%       mat = mi.bsdf.mask( ...
%           opacity=clouds, ...
%           bsdf=mi.bsdf.diffuse(reflectance=mi.rgb([1 1 1])));
    arguments
        options.opacity = []
        options.bsdf struct = struct([])
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("mask", "bsdf", options);
end
