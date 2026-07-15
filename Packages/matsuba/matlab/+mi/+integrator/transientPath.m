function s = transientPath(options)
%MI.INTEGRATOR.TRANSIENTPATH Create a transient path tracing integrator descriptor.
%   S = MI.INTEGRATOR.TRANSIENTPATH() creates a default transient path tracer.
%   S = MI.INTEGRATOR.TRANSIENTPATH(max_depth=8) sets the maximum bounce depth.
%
%   The transient path integrator extends standard path tracing to track
%   optical path lengths, producing time-resolved renderings when combined
%   with a transient film (mi.transientFilm).
%
%   Requires mitransient (installed automatically on first use).
%
%   See also mi.transientFilm, mi.Scene.renderTransient, mi.integrator.path
    arguments
        options.max_depth double = []
        options.rr_depth double = []
        options.hide_emitters logical = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("transient_path", "integrator", options);
end
