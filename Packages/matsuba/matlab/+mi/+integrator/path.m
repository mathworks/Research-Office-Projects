function s = path(options)
%MI.INTEGRATOR.PATH Create a path tracing integrator descriptor.
%   S = MI.INTEGRATOR.PATH() creates a default path tracer.
%   S = MI.INTEGRATOR.PATH(max_depth=8) sets the maximum bounce depth.
    arguments
        options.max_depth double = []
        options.rr_depth double = []
        options.hide_emitters logical = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("path", "integrator", options);
end
