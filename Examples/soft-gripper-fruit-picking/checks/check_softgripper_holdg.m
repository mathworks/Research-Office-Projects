function R = check_softgripper_holdg(ids, cases, wallLimit, evalOnly)
% CHECK_SOFTGRIPPER_HOLDG  T-I5: minimum hold pressure under the gripper's own acceleration (d'Alembert gravity).
%
%   R = CHECK_SOFTGRIPPER_HOLDG(1:6, ["lift" "carry0" "carry45"])
%   R = CHECK_SOFTGRIPPER_HOLDG(ids, cases, [], true)     evaluate saved results only
%
%   T-I2 and T-I4 load only the FRUIT, and overestimate the grip (in the closed loop the pear slips in Lift and
%   the apple sideways in Transport at pressures those tests call safe). In the pick the
%   whole gripper accelerates, so the soft fingers carry their own inertia and open. An accelerating gripper is
%   equivalent to a still one in the effective gravity g_eff = g - a_gripper, which loads fingers AND fruit.
%   Cases (a = aTransportObj(id), 2 m/s^2; mango 0.5):
%     lift     g_eff = [0 0 -(g + a)]                 upward acceleration (Lift, Lower braking)
%     carry0   g_eff = [a 0 -g]                       sideways, toward finger 1
%     carry45  g_eff = [a cos45, a sin45, -g]          sideways, between fingers 1 and 2
%   Procedure: the fruit sits where the pick grips it (planner grasp height); a support force -m*g_eff (like the
%   table) carries it while the gripper closes at 60 kPa, released 0.5 -> 0.6 s; from 0.7 s the pressure ramps
%   down at 10 kPa/s. p_hold = the manifold pressure when the fruit has moved 5 mm. Also reports the peak and the
%   held per-finger contact force (bruise). RESUMABLE: results/TI5_<id>_<case>.mat.

arguments
    ids double = 1:6
    cases (1,:) string = ["lift", "carry0", "carry45"]
    wallLimit double = 900
    evalOnly (1,1) logical = false
end
objs = params_softgripper_objects(); g = 9.80665;
fi = params_softgripper_finger; ctrl = params_softgripper_ctrl(params_softgripper_gas(fi));
tW0 = 0.5; tW1 = 0.6; tDown = 0.7; rDown = 10e3; pMax = 60e3; tEnd = tDown + pMax / rDown; sSlip = 5e-3;
resDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
R = struct('id', {}, 'case', {}, 'pHold', {}, 'fHeld', {}, 'status', {});
fprintf('\n=== T-I5 hold pressure under g_eff = g - a_gripper (fingers and fruit loaded) ===\n');
for k = ids
    o = objs(k); a = ctrl.aTransportObj(k);
    Pk = softgripperPickParams(k); z = -(Pk.plan.zPalm - Pk.objPoseInit(3));
    for c = cases
        switch c
            case "lift",    gE = [0 0 -(g + a)];
            case "carry0",  gE = [a 0 -g];
            case "carry45", gE = [a*cosd(45) a*sind(45) -g];
        end
        f = fullfile(resDir, sprintf('TI5_%d_%s.mat', k, c));
        if isfile(f), S = load(f); r = S.r;
        elseif evalOnly, fprintf('  %-13s %-8s not run yet\n', o.name, c); continue
        else
            P = gripperBenchParams(o, z); P.bench.gravity = gE;
            sup = -o.mass * gE;                                 % support force while closing (N, world)
            pCmd = timeseries([0; pMax; pMax; 0], [0; 0.1; tDown; tEnd]);
            fPull = timeseries([sup(3); sup(3); 0; 0], [0; tW0; tW1; tEnd]);
            fSide = timeseries([sup(1:2); sup(1:2); 0 0; 0 0], [0; tW0; tW1; tEnd]);
            r = runGripperBenchCase(P, pCmd, fPull, tEnd, wallLimit, [], [], fSide);
            save(f, 'r');
        end
        if isempty(r.t) || r.t(end) < tDown
            fprintf('  %-13s %-8s %s (%s)\n', o.name, c, upper(r.status), r.msg); continue
        end
        k0 = find(r.t >= tDown, 1);
        d = sqrt(sum((r.xy - r.xy(k0,:)).^2, 2) + (r.z - r.z(k0)).^2);
        kS = find(r.t > tDown & d >= sSlip, 1);
        if isempty(kS), pHold = 0; note = ' (held to 0 kPa)'; else, pHold = r.pMan(kS); note = ''; end
        fHeld = max(r.fn(k0, :));
        fprintf('  %-13s %-8s g_eff [%s] m/s^2: p_hold %5.1f kPa%s | held contact %.2f N/finger at 60 kPa | %s, %.0f s wall\n', ...
            o.name, c, strjoin(compose('%.1f', gE), ' '), pHold/1e3, note, fHeld, r.status, r.wall);
        R(end+1) = struct('id', k, 'case', c, 'pHold', pHold, 'fHeld', fHeld, 'status', r.status); %#ok<AGROW>
    end
end
end
