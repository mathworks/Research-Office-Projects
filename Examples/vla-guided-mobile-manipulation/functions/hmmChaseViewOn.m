function on = hmmChaseViewOn()
%HMMCHASEVIEWON Whether the chase camera renders now.
%
%   on = HMMCHASEVIEWON() reads root appdata 'hmmChaseViewOn', which hmmInitializeRun sets from
%   cfg.view.chase.enabled at the start of every run and the app's "Chase camera" box sets
%   whenever it is clicked. Plant/ChaseCamera's enable port is driven by it every scene step.
%
%   AN APPDATA CHANNEL AND NOT A PARAMETER, because a parameter cannot be changed mid-run here:
%   cfg is one struct, a parameter update re-evaluates every block that reads it, and the five
%   Simulation 3D Camera blocks refuse ("Error due to multiple causes", 2026-10-20). The same
%   channel carries typed instructions (hmmSubmitTask -> hmmOperatorTask).
%
%   See also hmmInitializeRun, hmmChaseBoom, HomeMobileManipulatorApp.

%   Copyright 2026 The MathWorks, Inc.

v = getappdata(0, 'hmmChaseViewOn');
if isempty(v)
    on = logical(hmmRuntimeConfig().view.chase.enabled);
else
    on = logical(v);
end
end
