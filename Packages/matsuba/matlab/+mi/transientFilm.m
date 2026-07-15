function s = transientFilm(options)
%MI.TRANSIENTFILM Create a transient HDR film descriptor for time-resolved rendering.
%   S = MI.TRANSIENTFILM() returns a transient_hdr_film struct with defaults.
%   S = MI.TRANSIENTFILM(Width=256, Height=256, TemporalBins=300) configures resolution and bins.
%
%   The transient film records optical path length (OPL) histograms per pixel,
%   producing a 4-D output (H x W x TemporalBins x 3) alongside the steady-state image.
%
%   Parameters:
%       Width        - Image width in pixels (default 256)
%       Height       - Image height in pixels (default 256)
%       TemporalBins - Number of time bins (default 300)
%       BinWidthOPL  - Width of each bin in optical path length units (default 0.003)
%       StartOPL     - Starting OPL for the histogram (default 0)
%
%   Requires mitransient (installed automatically on first use).
%
%   Example:
%       f = mi.transientFilm(Width=256, Height=256, TemporalBins=200);
%       sensor = mi.sensor.perspective(fov=39, film=f);
%
%   See also mi.film, mi.integrator.transientPath, mi.Scene.renderTransient
    arguments
        options.Width double = 256
        options.Height double = 256
        options.TemporalBins double = 300
        options.BinWidthOPL double = 0.003
        options.StartOPL double = 0
        options.key_ string = string.empty
    end

    opts.width = options.Width;
    opts.height = options.Height;
    opts.temporal_bins = options.TemporalBins;
    opts.bin_width_opl = options.BinWidthOPL;
    opts.start_opl = options.StartOPL;
    opts.key_ = options.key_;

    % mitransient requires a box rfilter — other filters are not supported
    opts.rfilter = struct("type", "box");

    s = mi.internal.pluginStruct("transient_hdr_film", "film", opts);
end
