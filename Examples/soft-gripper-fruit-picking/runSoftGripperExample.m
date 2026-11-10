%% Soft pneumatic gripper picking fruit — Simscape + Model-Based Design example
% Run this script from its folder; it puts the subfolders on the path (setupSoftGripperExample). It simulates
% one closed-loop pick in models/SoftGripperPick.slx (UR5e + a four-finger soft pneumatic gripper modelled in
% Simscape), scores it against the requirements, and, if Simulink 3D Animation is installed, replays it in
% Unreal through models/SoftGripperScene3D.slx.
%
% One pick takes about 23-30 minutes of computing (6-7 s simulated; REQ_GRIP_031 limit 35 min).
% Nothing here changes a model file: parameters go in through Simulink.SimulationInput.
%
% Objects in the pick set (params_softgripper_ctrl: ctrl.pickSet):
%   3 apple (200.6 g, 45 kPa)   4 tomato (282.9 g, 39.2 kPa)   5 pear (389.1 g, 60 kPa, lifted at 1 m/s^2)
% Not picked (see README): 6 mango (beyond the 60 kPa finger rating), 1 raspberry and 2 cherry tomato (too
% small for this finger spacing).

objId = 3;              % 3 apple, 4 tomato, 5 pear
doSimulate = true;      % false: skip the simulation and replay the saved run in results/pick_<objId>.mat
doReplay = true;        % replay the logged pick in Unreal afterwards (Simulink 3D Animation)

setupSoftGripperExample;    % models/, params/, checks/, pick/, replay/ on the path
fi = params_softgripper_finger; ctrl = params_softgripper_ctrl(params_softgripper_gas(fi));
assert(ismember(objId, ctrl.pickSet), 'objId %d is not in the pick set %s (see README).', objId, mat2str(ctrl.pickSet));

if doSimulate
    R = runSoftGripperPick(objId, 35 * 60);      % simulate + score; saves results/pick_<objId>.mat
else
    S = load(fullfile('results', sprintf('pick_%d.mat', objId)), 'R'); R = S.R; %#ok<UNRCH> (switch set above)
end
disp(R.timeline')

if doReplay && license('test', 'Virtual_Reality_Toolbox')
    playSoftGripperPick(objId);                  % reads results/pick_<objId>.mat only; writes images/<fruit>_pick.gif
end
