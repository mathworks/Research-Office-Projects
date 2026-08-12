function s = principled(options)
%MI.BSDF.PRINCIPLED Create a Disney principled BSDF descriptor.
%   S = MI.BSDF.PRINCIPLED(metallic=1, roughness=0.2) creates a metallic material.
%   S = MI.BSDF.PRINCIPLED(Name=Value) sets additional properties.
%   Scalar properties also accept texture structs for spatially-varying values.
%
%   Example:
%       % Scalar parameters
%       mat = mi.bsdf.principled(metallic=1, roughness=0.2);
%       % Texture-mapped roughness
%       mat = mi.bsdf.principled(roughness=mi.texture.bitmap("rough.png"));
    arguments
        options.base_color struct = struct([])
        options.roughness = []
        options.metallic = []
        options.specular = []
        options.spec_tint = []
        options.anisotropic = []
        options.sheen = []
        options.sheen_tint = []
        options.clearcoat = []
        options.clearcoat_gloss = []
        options.spec_trans = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("principled", "bsdf", options);
end
