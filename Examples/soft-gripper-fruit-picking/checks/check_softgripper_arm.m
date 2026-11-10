function R = check_softgripper_arm()
% CHECK_SOFTGRIPPER_ARM  M2 UR5e kinematics check on UR5eArm.slx (test plan section 2.3, T-O2).
%
%   R = CHECK_SOFTGRIPPER_ARM()
%
%   T-O2  five joint configurations held constant; the Simscape tool0 pose (Transform Sensor, World -> tool0)
%         after 1 s vs getTransform(loadrobot('universalUR5e'), q, 'tool0').
%         Prediction: equal to numerical precision (same robot description). Pass: position error < 0.1 mm.
%   Read-only on the models; does not cd; SimulationInput only.

ar = params_softgripper_arm;
rb = loadrobot('universalUR5e', 'DataFormat', 'row');
Q = [zeros(1,6)
     ar.qHome
     0.5 -1.2 1.4 -1.8 -1.57 0.3
     -1.0 -2.0 2.2 -0.2 1.0 -2.5
     2.0 -0.6 -1.9 1.2 0.4 1.5];
fprintf('\n=== T-O2 UR5e tool0 pose, %d configurations ===\n', size(Q,1));
fprintf('  PREDICTION: equal to getTransform to numerical precision (< 1 um)\n');
in = Simulink.SimulationInput.empty;
for k = 1:size(Q,1)
    ds = Simulink.SimulationData.Dataset;
    ds{1} = timeseries([Q(k,:); Q(k,:)], [0; 1]);
    in(k) = Simulink.SimulationInput('UR5eArm'); %#ok<AGROW>
    in(k) = in(k).setVariable('P', struct('arm', ar));
    in(k) = in(k).setExternalInput(ds);
    in(k) = in(k).setModelParameter('StopTime', '1');
end
out = sim(in, 'ShowProgress', 'off');
R = struct('q', {}, 'ePos', {}, 'eRot', {}, 'pass', {});
for k = 1:size(Q,1)
    T = getTransform(rb, Q(k,:), 'tool0');
    p = squeeze(out(k).logsout.get('toolPos').Values.Data); p = p(:, end);
    Rm = out(k).logsout.get('toolR').Values.Data; Rm = Rm(:, :, end);
    qm = out(k).logsout.get('qArmMeas').Values.Data; qm = qm(end, :);
    eP = norm(p - T(1:3,4)); eR = norm(Rm - T(1:3,1:3), 'fro');
    ok = eP < 1e-4;
    fprintf('  q = [%s]: tool at [%s] m, position error %.2e m, rotation error %.1e, joint error %.1e rad -> %s\n', ...
        strjoin(compose('%5.2f', Q(k,:)), ' '), strjoin(compose('%6.3f', T(1:3,4)'), ' '), eP, eR, max(abs(qm - Q(k,:))), passStr(ok));
    R(k) = struct('q', Q(k,:), 'ePos', eP, 'eRot', eR, 'pass', ok);
end
fprintf('  T-O2: %d of %d PASS\n', nnz([R.pass]), numel(R));
end

function s = passStr(ok)
s = 'FAIL'; if ok, s = 'PASS'; end
end
