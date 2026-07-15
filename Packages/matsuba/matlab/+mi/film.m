function s = film(options)
%MI.FILM Create an HDR film descriptor for Mitsuba.
%   S = MI.FILM() returns a default hdrfilm struct (768x576, OpenEXR).
%   S = MI.FILM(Width=1920, Height=1080) specifies resolution.
%
%   Example:
%       f = mi.film(Width=1024, Height=768, PixelFormat="rgb");
    arguments
        options.Width double = 768
        options.Height double = 576
        options.FileFormat string = "openexr"
        options.RFilter struct = struct([])
        options.PixelFormat string = string.empty
        options.key_ string = string.empty
    end

    opts.width = options.Width;
    opts.height = options.Height;
    opts.file_format = options.FileFormat;
    opts.rfilter = options.RFilter;
    opts.pixel_format = options.PixelFormat;
    opts.key_ = options.key_;

    s = mi.internal.pluginStruct("hdrfilm", "film", opts);
end
