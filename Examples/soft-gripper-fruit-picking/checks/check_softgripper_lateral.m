function R = check_softgripper_lateral(objId, pList, dirDeg, wallLimit, evalOnly, gravityOn)
% CHECK_SOFTGRIPPER_LATERAL  T-I4: sideways holding capacity of the pick grasp on GripperBench.slx.
%
%   R = CHECK_SOFTGRIPPER_LATERAL(3, [40 50 60]*1e3, [0 45])
%
%   Added 2026-10-24 after the first closed-loop apple pick slipped out SIDEWAYS during Transport at about
%   2 m/s^2, while T-I2 (vertical load only) said 34 kPa holds it. The bench gripper is fixed, so a horizontal
%   force m*a on the object is the d'Alembert equivalent of the gripper accelerating sideways at a.
%   Procedure: the object sits where the pick grips it (planner grasp height), the gripper closes at p (ramp
%   0.1 s), the weight m*g is applied downward from 0.5 to 0.6 s, then a horizontal force m*a(t) ramps from
%   0.7 s at 2 m/s^2 per s in direction dirDeg (0 = toward finger 1; 45 = between fingers 1 and 2).
%   Result: a_cap = the lateral acceleration at which the object has moved 5 mm from its loaded position.
%   Prediction for the apple at 38.6 kPa: a_cap < 2 m/s^2 (it slipped in the pick). RESUMABLE (TI4_<id>_<kPa>_<deg>.mat).

arguments
    objId (1,1) double
    pList double
    dirDeg double = [0 45]
    wallLimit double = 1500
    evalOnly (1,1) logical = false
    gravityOn (1,1) logical = false   % true: real gravity on fingers AND object (no explicit weight force), as in the pick
end
objs = params_softgripper_objects(); o = objs(objId); g = 9.80665;
Pk = softgripperPickParams(objId); zObj = -(Pk.plan.zPalm - Pk.objPoseInit(3));
P = gripperBenchParams(o, zObj);
gTag = '';
if gravityOn, P.bench.gravity = [0 0 -g]; gTag = '_g'; end
tW0 = 0.5; tW1 = 0.6; tL = 0.7; aRate = 2; aMax = 6; tEnd = tL + aMax / aRate; sSlip = 5e-3;
resDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
fprintf('\n=== T-I4 lateral capacity, %s at %.0f mm below the palm, weight + lateral m*a, a ramps %.0f m/s^2/s ===\n', o.name, -zObj*1e3, aRate);
R = struct('p', {}, 'dir', {}, 'aCap', {}, 'status', {});
for p = pList
    for d = dirDeg
        f = fullfile(resDir, sprintf('TI4_%d_%02.0f_%02.0f%s.mat', objId, p/1e3, d, gTag));
        if isfile(f), S = load(f); r = S.r;
        elseif evalOnly, fprintf('  %2.0f kPa, %2.0f deg: not run yet\n', p/1e3, d); continue
        else
            u = [cosd(d) sind(d)];
            pCmd = timeseries([0; p; p], [0; 0.1; tEnd]);
            if gravityOn   % a support force (like the table) carries the weight while closing, released 0.5 -> 0.6 s
                fPull = timeseries([o.mass*g; o.mass*g; 0; 0], [0; tW0; tW1; tEnd]);
            else
                fPull = timeseries([0; 0; -o.mass*g; -o.mass*g], [0; tW0; tW1; tEnd]);
            end
            fSide = timeseries([0 0; 0 0; o.mass*aMax*u], [0; tL; tEnd]);
            r = runGripperBenchCase(P, pCmd, fPull, tEnd, wallLimit, [], [], fSide);
            save(f, 'r');
        end
        aCap = NaN;
        if ~isempty(r.t) && r.t(end) > tL
            k0 = find(r.t >= tL, 1);
            dpos = sqrt(sum((r.xy - r.xy(k0,:)).^2, 2) + (r.z - r.z(k0)).^2);
            kS = find(r.t > tL & dpos >= sSlip, 1);
            if isempty(kS), aCap = aRate * (r.t(end) - tL); note = ' (held to the end: lower bound)';
            else, aCap = aRate * (r.t(kS) - tL); note = ''; end
            fn = interp1(r.t, r.fn, tL);
            fprintf('  %2.0f kPa, %2.0f deg: contact at load [%s] N | lateral capacity %.2f m/s^2%s (need 2.0) -> %s | %s, %.0f s wall\n', ...
                p/1e3, d, strjoin(compose('%.2f', fn), ' '), aCap, note, passStr(aCap >= 2), r.status, r.wall);
        else
            fprintf('  %2.0f kPa, %2.0f deg: %s (%s)\n', p/1e3, d, upper(r.status), r.msg);
        end
        R(end+1) = struct('p', p, 'dir', d, 'aCap', aCap, 'status', r.status); %#ok<AGROW>
    end
end
end

function s = passStr(ok)
s = 'FAIL'; if ok, s = 'PASS'; end
end
