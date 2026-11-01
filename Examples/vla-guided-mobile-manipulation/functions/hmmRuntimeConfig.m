function cfg = hmmRuntimeConfig(value)
%HMMRUNTIMECONFIG Effective configuration shared by model and MATLAB services.
% Publish once at setup or before starting a run. Reads do not rebuild the rig.
persistent current
if nargin > 0
    if ~isequaln(current, value)
        % These caches contain geometry derived from the effective configuration.
        clear hmmArmCollisionModel hmmReconstructTargets hmmSelectBaseGoal
        clear hmmSurfaceObstacles
    end
    current = value;
elseif isempty(current)
    current = hmmConfig();
end
cfg = current;
end
