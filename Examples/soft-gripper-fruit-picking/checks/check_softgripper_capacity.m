function R = check_softgripper_capacity(objId, pList, cases, wallLimit, evalOnly)
% CHECK_SOFTGRIPPER_CAPACITY  T-I6: load-capacity ratio of a grasp FORMED at pressure p, under g_eff = g - a_gripper.
%
%   R = CHECK_SOFTGRIPPER_CAPACITY(3, [38.6 45]*1e3, "carry0")
%
%   2026-10-25. T-I5 (and T-I2) closed at 60 kPa and then LOWERED the pressure: the hard squeeze seats the fruit
%   deeper, so those tests measure release, not grasp formation (apple carry: T-I5 said 34.8 kPa holds, the real
%   pick slipped at 38.6). Here the gripper closes AT p (as the controller does) in the effective gravity g_eff
%   (fingers and fruit loaded, d'Alembert), the table-like support is released 0.5 -> 0.6 s, then an EXTRA force
%   along g_eff ramps on the fruit: F = m*|g_eff|*s(t), s: 0 -> sMax over tRamp. Capacity ratio = 1 + s at 5 mm
%   slip (1 = holds exactly its working load). A ratio below 1 means it slips as soon as it is released.
%   Calibration (closed-loop apple pick): 38.6 kPa slipped in Transport, 45 kPa held -> expect ratio(38.6) < 1 <
%   ratio(45) for "carry0". RESUMABLE: results/TI6_<id>_<case>_<kPa>.mat.

arguments
    objId (1,1) double
    pList double
    cases (1,:) string = ["lift", "carry0"]
    wallLimit double = 600
    evalOnly (1,1) logical = false
end
objs = params_softgripper_objects(); o = objs(objId); g = 9.80665;
fi = params_softgripper_finger; ctrl = params_softgripper_ctrl(params_softgripper_gas(fi)); a = ctrl.aTransportObj(objId);
Pk = softgripperPickParams(objId); z = -(Pk.plan.zPalm - Pk.objPoseInit(3));
tW0 = 0.5; tW1 = 0.6; tL = 0.7; sMax = 2; tRamp = 2; tEnd = tL + tRamp; sSlip = 5e-3; tClose = 0.1;
resDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
R = struct('p', {}, 'case', {}, 'ratio', {}, 'status', {});
fprintf('\n=== T-I6 capacity of a grasp formed at p: %s (a = %.1f m/s^2) ===\n', o.name, a);
for c = cases
    switch c
        case "lift",    gE = [0 0 -(g + a)];
        case "carry0",  gE = [a 0 -g];
        case "carry45", gE = [a*cosd(45) a*sind(45) -g];
        case "stem",    gE = [0 0 -g];   % stem/vine pick: the stem pulls down with up to F_Detach before it lets go
    end
    u = gE / norm(gE); W = o.mass * norm(gE);
    if c == "stem"                       % ratio 1 = holds F_Detach + weight; closes with the pick's ramp (R2b)
        W = ctrl.F_Detach + o.mass * g; tClose = ctrl.tCloseRamp;
        tW0 = tClose + 0.4; tW1 = tW0 + 0.1; tL = tW1 + 0.1; tEnd = tL + tRamp;
    end
    for p = pList
        f = fullfile(resDir, sprintf('TI6_%d_%s_%02.0f.mat', objId, c, p/1e3));
        if c == "stem", f = strrep(f, '.mat', sprintf('_z%.0f.mat', -z*1e3)); end   % placement-specific
        if isfile(f), S = load(f); r = S.r;
        elseif evalOnly, fprintf('  %-8s %4.1f kPa: not run yet\n', c, p/1e3); continue
        else
            P = gripperBenchParams(o, z); P.bench.gravity = gE;
            sup = -o.mass * gE;
            Fx = W * sMax * u;                              % extra load at the end of the ramp
            pCmd = timeseries([0; p; p], [0; tClose; tEnd]);
            fPull = timeseries([sup(3); sup(3); 0; 0; Fx(3)], [0; tW0; tW1; tL; tEnd]);
            fSide = timeseries([sup(1:2); sup(1:2); 0 0; 0 0; Fx(1:2)], [0; tW0; tW1; tL; tEnd]);
            r = runGripperBenchCase(P, pCmd, fPull, tEnd, wallLimit, [], [], fSide);
            save(f, 'r');
        end
        if isempty(r.t) || r.t(end) < tW1
            fprintf('  %-8s %4.1f kPa: %s (%s)\n', c, p/1e3, upper(r.status), r.msg); continue
        end
        k0 = find(r.t >= tW0, 1);                          % reference: grasped, still supported
        d = sqrt(sum((r.xy - r.xy(k0,:)).^2, 2) + (r.z - r.z(k0)).^2);
        kS = find(r.t > tW0 & d >= sSlip, 1);
        if isempty(kS), ratio = 1 + sMax * (r.t(end) - tL) / tRamp; note = ' (held to the end: lower bound)';
        elseif r.t(kS) < tW1, ratio = (r.t(kS) - tW0) / (tW1 - tW0); note = ' (slipped while the support was released)';
        elseif r.t(kS) < tL, ratio = 1; note = ' (slipped under its working load, before any extra load)';
        else, ratio = 1 + sMax * (r.t(kS) - tL) / tRamp; note = ''; end
        if c == "stem"                   % gravity supplies only m*g here: ratio = (m g + extra) / (F_Detach + m g)
            s = ratio - 1; if isempty(kS) || r.t(kS) >= tL, ratio = (o.mass*g + s*W) / W; else, ratio = ratio * o.mass*g / W; end
        end
        if c == "stem", dMax = max(sqrt(sum((r.xy(1:k0,:) - r.xy(1,:)).^2, 2) + (r.z(1:k0) - r.z(1)).^2)); ...
            note = note + sprintf(" | pushed %.1f mm while closing (stem pull %.2f N vs %.1f)", dMax*1e3, Pk.scene.kStem*dMax, ctrl.F_Detach); end
        fprintf('  %-8s %4.1f kPa: contact %s N | capacity ratio %.2f%s | %s, %.0f s wall\n', c, p/1e3, ...
            mat2str(round(interp1(r.t, r.fn, tW0), 2)), ratio, note, r.status, r.wall);
        R(end+1) = struct('p', p, 'case', c, 'ratio', ratio, 'status', r.status); %#ok<AGROW>
    end
end
end
