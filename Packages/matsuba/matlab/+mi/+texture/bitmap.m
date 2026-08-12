function s = bitmap(filename, options)
%MI.TEXTURE.BITMAP Create a bitmap (image file) texture descriptor.
%   S = MI.TEXTURE.BITMAP(FILENAME) creates a texture from an image file.
%   Supports PNG, JPG, EXR, and other formats supported by Mitsuba.
%
%   S = MI.TEXTURE.BITMAP(FILENAME, filter_type="nearest") sets filtering.
%   S = MI.TEXTURE.BITMAP(FILENAME, wrap_mode="repeat") sets wrapping.
%   S = MI.TEXTURE.BITMAP(FILENAME, raw=true) disables sRGB conversion.
%   S = MI.TEXTURE.BITMAP(FILENAME, to_uv=T) sets a 3x3 UV transform.
%
%   Example:
%       % Textured diffuse material
%       tex = mi.texture.bitmap("wood.png");
%       mat = mi.bsdf.diffuse(reflectance=tex);
%
%       % Repeating checkerboard from an image
%       tex = mi.texture.bitmap("tile.jpg", wrap_mode="repeat");
    arguments
        filename (1,1) string
        options.filter_type string = string.empty
        options.wrap_mode string = string.empty
        options.raw logical = logical.empty
        options.to_uv = []
        options.key_ string = string.empty
    end

    % Resolve to absolute path so Mitsuba can find the file
    f = char(filename);
    if ~mi.internal.isAbsolutePath(f)
        f = fullfile(pwd, f);
    end
    if ~isfile(f)
        error("mi:texture:bitmap:FileNotFound", ...
            "Texture file not found: %s", f);
    end

    s.type = "bitmap";
    s.category_ = "texture";
    s.filename = f;

    if ~isempty(options.filter_type)
        s.filter_type = options.filter_type;
    end
    if ~isempty(options.wrap_mode)
        s.wrap_mode = options.wrap_mode;
    end
    if ~isempty(options.raw)
        s.raw = options.raw;
    end
    if ~isempty(options.to_uv)
        s.to_uv = options.to_uv;
    end
    if ~isempty(options.key_)
        s.key_ = char(options.key_);
    end
end
