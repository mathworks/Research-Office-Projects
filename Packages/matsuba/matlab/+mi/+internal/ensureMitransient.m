function ensureMitransient()
%ENSUREMITRANSIENT Ensure the mitransient package is installed and loaded.
%   Checks if mitransient is importable in the current Python environment.
%   If not, installs it via mpyreq. Then imports it so its Mitsuba plugins
%   (transient_hdr_film, transient_path, etc.) are registered.

    % Note: this cache is invalidated if terminate(pyenv) is called,
    % since the Python process is killed but the persistent variable
    % remains true. Restart MATLAB to reset.
    persistent isLoaded
    if ~isempty(isLoaded) && isLoaded
        return
    end

    % Try importing mitransient via the bridge
    try
        py.matlab_mitsuba.bridge.ensure_mitransient();
        isLoaded = true;
        return
    catch
        % Not installed — fall through to install
    end

    % Install mitransient via mpyreq
    fprintf("Installing mitransient...\n");
    MPyReq.pipPackage("mitransient");
    fprintf("mitransient installed successfully.\n");

    % Now import it
    py.matlab_mitsuba.bridge.ensure_mitransient();
    isLoaded = true;
end
