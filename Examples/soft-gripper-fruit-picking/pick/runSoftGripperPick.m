function R = runSoftGripperPick(objId, wallLimit, vCrit)
% RUNSOFTGRIPPERPICK  One closed-loop pick in SoftGripperPick.slx, scored against REQ_GRIP_016-018.
%
%   R = RUNSOFTGRIPPERPICK(objId)            objId 1..6; wall limit 3600 s
%   R = RUNSOFTGRIPPERPICK(objId, wallLimit)
%
%   Builds P (softgripperPickParams), simulates with SimulationInput (finite StopTime = the planned duration
%   + 2 s, never above T_PickMax + 1 s), saves results/pick_<objId>.mat and prints:
%     final controller state and its time, attempts, whether the object rests in the tray, the peak per-finger
%     contact force vs F_Bruise (016), the slip of the object relative to the palm while carried (017), and the
%     pick time vs T_PickMax (018).
%   Slip = change of the object position in the palm frame (tool0 from the measured joints by forward
%   kinematics) between the end of CheckGrasp and the start of Release.
%   Read-only on the model; does not cd. Long runs can go in a separate MATLAB (matlab -batch).

arguments
    objId (1,1) double
    wallLimit (1,1) double = 3600
    vCrit double = []                  % friction critical velocity override (m/s); default P.bench.vCrit
end
P = softgripperPickParams(objId);
if ~isempty(vCrit), P.bench.vCrit = vCrit; end
c = P.ctrl;
% Stop 0.6 s after the planned release: REQ_GRIP_018 asks for the object IN THE TRAY within T_PickMax; the arm's
% retreat home is not part of it and costs about 2 s of simulated time (about 6 min wall).
tPlan = c.tSettle + sum(P.plan.segT(1:5)) + c.tCloseRamp + c.nStall * c.Ts_Ctrl + c.tVent;
tStop = min(c.T_PickMax + 1, tPlan + 0.6);       % 0.6 s after the planned release: the fruit has settled (T-O1 < 0.13 s)
in = Simulink.SimulationInput('SoftGripperPick');
in = in.setVariable('P', P);
ds = Simulink.SimulationData.Dataset;
ds{1} = timeseries([P.objPoseInit; P.objPoseInit], [0; tStop]);
ds{2} = timeseries([P.gas.p_Supply; P.gas.p_Supply], [0; tStop]);
in = in.setExternalInput(ds);
in = in.setModelParameter('StopTime', num2str(tStop));
in = in.setModelParameter('TimeOut', wallLimit);
objs = params_softgripper_objects(); o = objs(objId);
fprintf('\n=== pick %d: %s (%.1f g), pGrip %.1f kPa, planned %.1f s, StopTime %.1f s ===\n', objId, o.name, o.mass*1e3, c.pGrip(objId)/1e3, tPlan, tStop);
t0 = tic;
out = sim(in, 'ShowProgress', 'off');
wall = toc(t0);
R = scoreSoftGripperPick(out, P, tStop);
R.wall = wall;
resDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~isfolder(resDir), mkdir(resDir); end
save(fullfile(resDir, sprintf('pick_%d.mat', objId)), 'R', 'out');
fprintf('  final state %s at %.2f s (sim reached %.2f s, %.0f s wall); attempts %d\n', R.finalState, R.tFinal, R.tEnd, wall, R.attempts);
fprintf('  object at [%s] m: in tray %s\n', strjoin(compose('%.3f', R.objEnd), ' '), passStr(R.inTray));
fprintf('  peak finger contact %.2f N vs bruise %.1f N -> %s (REQ_GRIP_016)\n', R.fMax, o.F_Bruise, passStr(R.fMax < o.F_Bruise));
fprintf('  slip in the palm while carried %.1f mm (limit %.0f mm) -> %s (REQ_GRIP_017)\n', R.slip*1e3, c.s_SlipMax*1e3, passStr(R.slip <= c.s_SlipMax));
fprintf('  placed (Release) at %.2f s (limit %.0f s), in tray %d, %d attempt(s) (limit %d) -> %s (REQ_GRIP_018)\n', ...
    R.tRelease, c.T_PickMax, R.inTray, R.attempts, c.N_Attempts, passStr(R.inTray && R.tRelease <= c.T_PickMax && R.attempts <= c.N_Attempts));
end

function s = passStr(ok)
s = 'FAIL'; if ok, s = 'PASS'; end
end
