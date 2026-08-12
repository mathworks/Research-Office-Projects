function setVariant(name)
%MI.SETVARIANT Set the active Mitsuba variant.
%   MI.SETVARIANT(NAME) sets the Mitsuba rendering variant.
%   MI.SETVARIANT("auto") automatically selects the best variant for the
%   current system (prefers GPU over LLVM over scalar).
%   If the variant is not available, an error lists the installed variants.
%
%   Examples:
%       mi.setVariant("auto")           % auto-detect best backend
%       mi.setVariant("scalar_rgb")
%       mi.setVariant("cuda_ad_rgb")
%
%   See also MI.BESTVARIANT, MI.AVAILABLEVARIANTS

    arguments
        name (1,1) string
    end

    if name == "auto"
        name = mi.bestVariant();
        fprintf("Auto-selected variant: %s\n", name);
    end

    % Warn if switching between incompatible variants (DrJit limitation)
    try
        current = string(py.mitsuba.variant());
    catch
        current = "";
    end
    if current ~= "" && current ~= name
        isJitCurrent = contains(current, "llvm") || contains(current, "cuda");
        isJitNew = contains(name, "llvm") || contains(name, "cuda");
        if isJitCurrent && isJitNew && current ~= name
            warning("mi:setVariant:ThreadState", ...
                "Switching from '%s' to '%s' may cause DrJit ThreadState errors.\n" + ...
                "Restart MATLAB if you encounter issues.", current, name);
        elseif isJitCurrent && ~isJitNew
            warning("mi:setVariant:ThreadState", ...
                "Switching from JIT variant '%s' to '%s' may cause issues.\n" + ...
                "Restart MATLAB if you encounter errors.", current, name);
        end
    end

    try
        py.matlab_mitsuba.bridge.set_variant(name);
    catch ex
        % Provide actionable error with available variants
        try
            variants = mi.availableVariants();
            variantList = strjoin(variants, ", ");
        catch
            variantList = "(unable to query)";
        end
        error("mi:setVariant:NotAvailable", ...
            "Variant '%s' is not available.\nInstalled variants: %s", ...
            name, variantList);
    end
end
