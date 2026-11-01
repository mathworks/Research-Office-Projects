function hmmInitializeRun(model)
%HMMINITIALIZERUN Publish the configuration resolved for this simulation.
% slResolve includes SimulationInput overrides; evalinGlobal can return base cfg.
cfg = slResolve('cfg', model);
hmmRuntimeConfig(cfg);

% THE CHASE VIEW STARTS AS cfg.view.chase.enabled SAYS. The ChaseCamera's enable is read every
% step from appdata (hmmChaseViewOn), so the app can switch it mid-run; each run starts here from
% the configuration, so a box ticked in an earlier run does not carry over. Its Video Viewer
% opens only when it is on, so it never opens black. Found by name, so it does not depend on
% where Plant sits in the hierarchy.
chaseOn = isfield(cfg.view, 'chase') && cfg.view.chase.enabled;
setappdata(0, 'hmmChaseViewOn', chaseOn);
viewer = find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'off', 'Name', 'ChaseView');
if ~isempty(viewer)
    want = 'off';
    if chaseOn
        want = 'on';
    end
    if ~strcmp(get_param(viewer{1}, 'OpenAtMdlStart'), want)
        set_param(viewer{1}, 'OpenAtMdlStart', want);
    end
end
end
