function R = check_softgripper_hold(ids, wallLimit, evalOnly, placement)
% CHECK_SOFTGRIPPER_HOLD  Integrated open-loop tests T-I2 and T-I3 on GripperBench.slx.
%
%   R = CHECK_SOFTGRIPPER_HOLD()                 the five Table 4 fruits (cherry tomato, apple, tomato, pear, mango)
%   R = CHECK_SOFTGRIPPER_HOLD([3 6], 1800)      a subset (objId), wall limit per case (s)
%   R = CHECK_SOFTGRIPPER_HOLD(ids, [], true)    evaluate saved results only, never simulate
%   R = CHECK_SOFTGRIPPER_HOLD(ids, [], false, "pick")  the fruit where the PICK grips it (palm at the planner's
%       grasp height, so a gap above the fruit), not against the palm. The palm-contact hold pressures do not
%       apply to the pick grasp. Files TI2p_<id>.mat.
%
%   T-I2  minimum hold pressure. One run per fruit instead of a bisection: grasp at p_GripMax, then apply the
%         fruit's weight plus the transport acceleration, m*(g + a_Transport), as a constant downward force, and
%         ramp the pressure down at rDown. The minimum hold pressure = the manifold pressure when the fruit has
%         slipped s_SlipMax = 5 mm. Pass: within +/-10 kPa of Zhu Table 4 (REQ_GRIP_013).
%         Caveat: Zhu do not say how the Table 4 pressures were chosen; they are pressures that worked, which may
%         be above the minimum.
%   T-I3  bruise check: the largest per-finger contact force (sum over its 8 links) while the pressure is within
%         2 kPa of the Table 4 value, vs F_Bruise (REQ_GRIP_016).
%   Placement: enveloping fruits centred on the axis 1 mm under the palm; fingertip fruits (cherry tomato) where
%   the tips meet, as in T-I1. Gravity off; the load is explicit (gripperBenchParams).
%   RESUMABLE like check_softgripper_grasp: results/TI2_<objId>.mat per fruit. SimulationInput only.

arguments
    ids double = [2 3 4 5 6]
    wallLimit double = 1800
    evalOnly (1,1) logical = false
    placement (1,1) string = "palm"
end
objs = params_softgripper_objects();
g = 9.80665; aT = 2.0; sSlip = 5e-3;
tGrasp = 0.5; tLoad = 0.6; rDown = 10e3; pMax = 60e3;
tDown = tLoad + 0.1; tEnd = tDown + pMax / rDown;
resDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~isfolder(resDir), mkdir(resDir); end
fprintf('\n=== T-I2 hold pressure: load m*(g + %.0f m/s^2), pressure ramps down %.0f kPa/s from %.1f s ===\n', aT, rDown/1e3, tDown);
R = struct('name', {}, 'pHold', {}, 'zhu', {}, 'fMaxAtZhu', {}, 'F_Bruise', {}, 'status', {}, 'pass', {}, 'passBruise', {});
for k = ids
    o = objs(k);
    if placement == "pick"
        Pk = softgripperPickParams(k);                         % the planner's grasp height above the fruit centre
        z = -(Pk.plan.zPalm - Pk.objPoseInit(3)); tagF = 'TI2p';
    elseif o.graspMode == "fingertip", z = -0.090; tagF = 'TI2';
    else, z = -(o.semiAxes(3) + 1e-3); tagF = 'TI2';
    end
    W = o.mass * (g + aT);
    f = fullfile(resDir, sprintf('%s_%d.mat', tagF, k));
    if isfile(f)
        S = load(f); r = S.r;
    elseif evalOnly
        fprintf('  %-13s not run yet\n', o.name); continue
    else
        P = gripperBenchParams(o, z);
        pCmd = timeseries([0; pMax; pMax; 0], [0; 0.1; tDown; tEnd]);
        fPull = timeseries([0; 0; -W; -W], [0; tGrasp; tLoad; tEnd]);
        r = runGripperBenchCase(P, pCmd, fPull, tEnd, wallLimit);
        r.objId = k; save(f, 'r');
    end
    if isempty(r.t) || r.t(end) < tDown
        fprintf('  %-13s %s after %.0f s wall (%s)\n', o.name, upper(r.status), r.wall, r.msg); continue
    end
    z0 = interp1(r.t, r.z, tDown);
    kS = find(r.t > tDown & z0 - r.z >= sSlip, 1);
    if isempty(kS), pHold = 0; note = ' (held to 0 kPa)'; else, pHold = r.pMan(kS); note = ''; end
    if isnan(o.p_HoldZhu), atZhu = r.pMan >= pMax - 1e3; else, atZhu = abs(r.pMan - o.p_HoldZhu) < 2e3 & r.t > tDown; end
    fMax = max([0; max(r.fn(atZhu, :), [], 2)]);
    ok = abs(pHold - o.p_HoldZhu) <= 10e3;
    okB = fMax < o.F_Bruise;
    fprintf('  %-13s %6.1f g, load %.2f N: minimum hold %5.1f kPa%s vs Zhu %s kPa -> %s | max finger contact %.2f N vs bruise %.1f N -> %s | %s, %.0f s wall\n', ...
        o.name, o.mass*1e3, W, pHold/1e3, note, num2str(o.p_HoldZhu/1e3), passStr(ok), fMax, o.F_Bruise, passStr(okB), r.status, r.wall);
    R(end+1) = struct('name', o.name, 'pHold', pHold, 'zhu', o.p_HoldZhu, 'fMaxAtZhu', fMax, 'F_Bruise', o.F_Bruise, ...
                      'status', r.status, 'pass', ok, 'passBruise', okB); %#ok<AGROW>
end
fprintf('  REQ_GRIP_013: %d of %d within +/-10 kPa | REQ_GRIP_016: %d of %d below the bruise limit\n', ...
    nnz([R.pass]), numel(R), nnz([R.passBruise]), numel(R));
end

function s = passStr(ok)
s = 'FAIL'; if ok, s = 'PASS'; end
end
