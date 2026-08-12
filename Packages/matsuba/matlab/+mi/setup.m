function setup(options)
%MI.SETUP Configure Python environment for Mitsuba rendering.
%   MI.SETUP() uses mpyreq to install Python 3.12, mitsuba, and numpy,
%   then configures MATLAB to use the environment and selects the best
%   rendering variant.
%
%   MI.SETUP(Variant=NAME) overrides automatic variant selection with the
%   specified variant (e.g. "scalar_rgb", "cuda_ad_rgb"). Use "auto" to
%   explicitly request automatic selection (the default).
%
%   Note: pyenv can only be configured once per MATLAB session, before any
%   py.* call. Restart MATLAB if you need to change the Python environment.
%
%   Examples:
%       mi.setup                           % auto variant
%       mi.setup(Variant="scalar_rgb")     % force a specific variant

    arguments
        options.Variant (1,1) string = "auto"
    end

    % --- Fast path: already configured this session ---
    pe = pyenv;
    if pe.Status ~= "NotLoaded"
        fprintf("Python already configured: %s (v%s, %s)\n", ...
            pe.Executable, pe.Version, pe.ExecutionMode);
        ensureModuleLoaded();
        mi.setVariant(options.Variant);
        return
    end

    % --- Configure Python via mpyreq ---
    installDir = fullfile(mi.internal.matsubaRoot(), ".mpyreq");
    MPyReq.setInstallFolder(installDir);
    MPyReq.python("3.12");
    MPyReq.pipPackage("mitsuba");
    MPyReq.pipPackage("numpy");
    MPyReq.addToPythonPath(fullfile(mi.internal.matsubaRoot(), "python"));

    % --- Verify bridge module is loadable ---
    ensureModuleLoaded();

    % --- Auto-select rendering variant ---
    mi.setVariant(options.Variant);
end


function ensureModuleLoaded()
%ENSUREMODULELOADED Verify matlab_mitsuba.bridge is importable and
%   add to sys.path as a fallback.
    try
        py.importlib.import_module("matlab_mitsuba.bridge");
    catch
        % Fallback: add to sys.path directly and retry
        adapterDir = fullfile(mi.internal.matsubaRoot(), "python");
        if isfolder(adapterDir)
            sysPath = py.sys.path;
            sysPath.insert(int32(0), adapterDir);
            py.importlib.import_module("matlab_mitsuba.bridge");
        end
    end
end
