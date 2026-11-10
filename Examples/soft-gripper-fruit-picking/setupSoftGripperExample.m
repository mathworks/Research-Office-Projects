function setupSoftGripperExample
% SETUPSOFTGRIPPEREXAMPLE  Put the example's code folders on the MATLAB path for this session.
%
%   Run once per MATLAB session before calling any script of the example directly. runSoftGripperExample
%   calls it for you. Nothing is saved: the path change ends when MATLAB closes.
%
%   models/    closed-loop model, components (Subsystem References), Unreal scene
%   benches/   component and integrated test benches
%   params/    all parameters, each with unit and source; finger identification
%   checks/    validation tests (resumable, with an evaluate-only mode)
%   pick/      one closed-loop pick: plan, simulate, score
%   replay/    Unreal replay and README figures

root = fileparts(mfilename('fullpath'));
dirs = fullfile(root, ["models" "benches" "params" "checks" "pick" "replay"]);
addpath(dirs{:});
end
