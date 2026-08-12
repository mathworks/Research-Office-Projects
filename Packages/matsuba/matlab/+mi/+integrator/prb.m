function s = prb(options)
%MI.INTEGRATOR.PRB Create a Path Replay Backpropagation integrator descriptor.
%   S = MI.INTEGRATOR.PRB() creates a default PRB integrator for
%   differentiable rendering.
%   S = MI.INTEGRATOR.PRB(max_depth=8) sets the maximum bounce depth.
%
%   The PRB integrator is required for gradient-based optimization.
%   It is essentially a path tracer augmented with an efficient algorithm
%   to compute gradients in a separate adjoint pass.
%
%   Requires an AD-capable variant (e.g. "llvm_ad_rgb", "cuda_ad_rgb").
%
%   See also mi.Scene.renderDiff, mi.integrator.path
    arguments
        options.max_depth double = []
        options.rr_depth double = []
        options.hide_emitters logical = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("prb", "integrator", options);
end
