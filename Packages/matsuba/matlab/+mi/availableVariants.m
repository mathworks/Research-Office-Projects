function variants = availableVariants()
%MI.AVAILABLEVARIANTS List installed Mitsuba rendering variants.
%   VARIANTS = MI.AVAILABLEVARIANTS() returns a string array of variant
%   names that are compiled and available in the current Mitsuba install.
%
%   Example:
%       mi.availableVariants()
%       % ans = ["scalar_rgb", "scalar_spectral", "llvm_ad_rgb", ...]

    try
        pyList = py.matlab_mitsuba.bridge.available_variants();
        variants = string(cell(pyList));
    catch ex
        error("mi:availableVariants:Failed", ...
            "Failed to query available variants.\n%s", ex.message);
    end
end
