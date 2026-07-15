function s = checkerboard(options)
%MI.TEXTURE.CHECKERBOARD Create a checkerboard procedural texture.
%   S = MI.TEXTURE.CHECKERBOARD() creates a black-and-white checkerboard.
%   S = MI.TEXTURE.CHECKERBOARD(color0=mi.rgb([.8 .8 .8]), color1=mi.rgb([.2 .2 .2]))
%   sets custom colors.
%   S = MI.TEXTURE.CHECKERBOARD(to_uv=T) sets a 3x3 UV transform to
%   control tiling scale and orientation.
%
%   Example:
%       % Checkerboard floor
%       tex = mi.texture.checkerboard();
%       floor = mi.shape.rectangle( ...
%           bsdf=mi.bsdf.diffuse(reflectance=tex));
    arguments
        options.color0 struct = struct([])
        options.color1 struct = struct([])
        options.to_uv = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("checkerboard", "texture", options);
end
