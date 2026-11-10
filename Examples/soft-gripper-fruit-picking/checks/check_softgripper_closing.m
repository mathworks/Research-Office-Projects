function R = check_softgripper_closing(ids, tRamps, wallLimit, evalOnly)
% CHECK_SOFTGRIPPER_CLOSING  T-I7: closing-speed study on GripperBench.slx (engineer decision 2026-10-25).
%
%   R = CHECK_SOFTGRIPPER_CLOSING([4 5 1], [0.1 0.3 0.6 1.0])
%
%   The closed-loop picks closed the fingers with a 0.1 s pressure ramp (R2): the tomato saw an 18.7 N and the
%   pear a 46.5 N one-sample contact spike at impact (bruise 11 / 40 N), and the raspberry was flicked 41 mm up
%   and broke its stem. Here the gripper closes on the fruit, placed where the pick grips it, with gravity on and
%   a table-like support force, at pGrip(obj) with each ramp time, and holds 0.5 s. Reported per ramp:
%     peak per-finger contact (sum over its 8 links) vs F_Bruise(obj)   (REQ_GRIP_016)
%     peak fruit displacement during closing; for the stem fruit x kStem vs F_Detach (the flick)
%   RESUMABLE: results/TI7_<id>_<ms>.mat.

arguments
    ids double = [4 5 1]
    tRamps double = [0.1 0.3 0.6 1.0]
    wallLimit double = 600
    evalOnly (1,1) logical = false
end
objs = params_softgripper_objects(); g = 9.80665;
resDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
R = struct('id', {}, 'tRamp', {}, 'fPeak', {}, 'dPeak', {}, 'passBruise', {}, 'passFlick', {});
fprintf('\n=== T-I7 closing speed: peak contact and fruit displacement vs the pressure ramp ===\n');
for k = ids
    o = objs(k); Pk = softgripperPickParams(k); p = Pk.ctrl.pGrip(k);
    z = -(Pk.plan.zPalm - Pk.objPoseInit(3));
    for T = tRamps
        f = fullfile(resDir, sprintf('TI7_%d_%04.0f.mat', k, T*1e3));
        tEnd = T + 0.5;
        if isfile(f), S = load(f); r = S.r;
        elseif evalOnly, fprintf('  %-13s ramp %.1f s: not run yet\n', o.name, T); continue
        else
            P = gripperBenchParams(o, z); P.bench.gravity = [0 0 -g];
            pCmd = timeseries([0; p; p], [0; T; tEnd]);
            fPull = timeseries([o.mass*g; o.mass*g], [0; tEnd]);          % the table (or stem) carries the weight
            r = runGripperBenchCase(P, pCmd, fPull, tEnd, wallLimit);
            save(f, 'r');
        end
        if isempty(r.t), fprintf('  %-13s ramp %.1f s: %s (%s)\n', o.name, T, upper(r.status), r.msg); continue, end
        fPeak = max(r.fn, [], 'all');
        d = sqrt(sum((r.xy - r.xy(1,:)).^2, 2) + (r.z - r.z(1)).^2); dPeak = max(d);
        okB = fPeak < o.F_Bruise;
        if k == 1, fFlick = Pk.scene.kStem * dPeak; okF = fFlick < Pk.ctrl.F_Detach; flick = sprintf(' | stem pull %.2f N vs detach %.1f -> %s', fFlick, Pk.ctrl.F_Detach, passStr(okF));
        else, okF = true; flick = ''; end
        fprintf('  %-13s %4.1f kPa, ramp %.1f s: peak contact %6.2f N vs bruise %4.1f -> %s | fruit moved %5.1f mm%s | %s, %.0f s wall\n', ...
            o.name, p/1e3, T, fPeak, o.F_Bruise, passStr(okB), dPeak*1e3, flick, r.status, r.wall);
        R(end+1) = struct('id', k, 'tRamp', T, 'fPeak', fPeak, 'dPeak', dPeak, 'passBruise', okB, 'passFlick', okF); %#ok<AGROW>
    end
end
end

function s = passStr(ok)
s = 'FAIL'; if ok, s = 'PASS'; end
end
