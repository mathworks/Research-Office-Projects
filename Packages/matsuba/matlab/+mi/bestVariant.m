function name = bestVariant()
%MI.BESTVARIANT Select the best Mitsuba variant for the current system.
%   NAME = MI.BESTVARIANT() returns the name of the best available Mitsuba
%   rendering variant based on the system's capabilities.
%
%   Selection priority (highest to lowest):
%       cuda_ad_rgb       — NVIDIA GPU with autodiff
%       cuda_rgb          — NVIDIA GPU
%       llvm_ad_rgb       — LLVM JIT with autodiff
%       llvm_rgb          — LLVM JIT
%       scalar_rgb        — Single-threaded CPU
%       (spectral variants used as fallbacks within each tier)
%
%   Example:
%       v = mi.bestVariant()     % returns e.g. "scalar_rgb"
%       mi.setVariant(v)
%
%   See also MI.SETVARIANT, MI.AVAILABLEVARIANTS

    try
        name = string(py.matlab_mitsuba.bridge.best_variant());
    catch ex
        error("mi:bestVariant:Failed", ...
            "Failed to determine best variant.\n%s", ex.message);
    end
end
