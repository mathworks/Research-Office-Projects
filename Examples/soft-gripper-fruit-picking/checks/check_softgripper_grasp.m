function R = check_softgripper_grasp(modes, pList, wallLimit, evalOnly, zFingertip)
% CHECK_SOFTGRIPPER_GRASP  Phase 2 integrated open-loop test T-I1 on GripperBench.slx.
%
%   R = CHECK_SOFTGRIPPER_GRASP()                              both modes, 10..60 kPa, 900 s wall limit per case
%   R = CHECK_SOFTGRIPPER_GRASP("fingertip", [30 60]*1e3, 600)  a subset
%   R = CHECK_SOFTGRIPPER_GRASP(modes, pList, [], true)         evaluate saved results only, never simulate
%   R = CHECK_SOFTGRIPPER_GRASP("fingertip", pList, [], false, -0.080)  fingertip ball at another height (m):
%       the engineer-approved height sweep (2026-10-24), since Zhu give no ball placement
%
%   T-I1  Zhu Table 3 grasp force, measured like Zhu's force gauge (FGJ-50 on a moving arm = displacement
%         controlled): the ball hangs on a gauge spring (kGauge) whose end is fixed while the gripper closes
%         at pressure p (ramp 0.1 s), then moves down at vPull from tPull. Grasp force = the peak gauge force.
%         At that peak the ball is quasi-static, so its inertia does not enter (the earlier force-ramp version
%         read a free ball's inertia as grip). Pass: within +/-30 % of Table 3 (REQ_GRIP_012).
%   ASSUMPTIONS (Zhu: "a blue ball", no size): fingertip ball radius 15 mm where the tips meet; enveloping ball
%   radius 40 mm just under the palm. Ball on silicone: k 5000 N/m, mu 0.8 (EST). Gauge 2000 N/m, 20 mm/s (EST).
%   Gravity off (bench note in gripperBenchParams). Solver daessc (ode23t/ode15s crawl at 1e-6 s steps once
%   the friction contacts stick).
%   RESUMABLE: each case is saved to results/TI1g_<mode>_<kPa>.mat as soon as it ends and is skipped if that
%   file exists; a case that hits the wall limit is saved and reported as "timeout", not hidden. Long sweeps can run
%   in a separate MATLAB (matlab -batch). Read-only on the models; does not cd.

arguments
    modes (1,:) string = ["fingertip", "enveloping"]
    pList double = (10:10:60) * 1e3
    wallLimit double = 900
    evalOnly (1,1) logical = false
    zFingertip double = []
end
zhu.fingertip = [0.41 0.79 1.29 1.51 1.85 2.13];
zhu.enveloping = [0.79 1.34 2.28 3.47 4.68 5.8];
tPull = 0.5; vPull = 0.02; tEnd = 2.0; kG = 2000;
resDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~isfolder(resDir), mkdir(resDir); end
R = struct();
for mode = modes
    [ball, zBall] = testBall(mode);
    tag = '';
    if mode == "fingertip" && ~isempty(zFingertip), zBall = zFingertip; tag = sprintf('_z%03.0f', -zBall*1e3); end
    P = gripperBenchParams(ball, zBall);
    P.bench.kGauge = kG; P.bench.cGauge = 2 * 0.5 * sqrt(kG * ball.mass);
    ref = zhu.(mode)(round(pList / 10e3));
    fprintf('\n=== T-I1 %s grasp: ball r = %.0f mm at z = %.0f mm; gauge %.0f N/m pulled at %.0f mm/s from %.1f s ===\n', ...
        mode, ball.semiAxes(1)*1e3, zBall*1e3, kG, vPull*1e3, tPull);
    F = nan(1, numel(pList)); status = strings(1, numel(pList));
    for k = 1:numel(pList)
        f = fullfile(resDir, sprintf('TI1g_%s_%02.0f%s.mat', mode, pList(k)/1e3, tag));
        if isfile(f)
            S = load(f); r = S.r;
        elseif evalOnly
            fprintf('  %2.0f kPa: not run yet\n', pList(k)/1e3); continue
        else
            pCmd = timeseries([0; pList(k); pList(k)], [0; 0.1; tEnd]);
            fPull = timeseries([0; 0], [0; tEnd]);
            zG = timeseries([zBall; zBall; zBall - vPull*(tEnd - tPull)], [0; tPull; tEnd]);
            vG = timeseries([0; 0; -vPull; -vPull], [0; tPull; tPull + 1e-3; tEnd]);
            r = runGripperBenchCase(P, pCmd, fPull, tEnd, wallLimit, zG, vG);
            r.mode = mode; r.p = pList(k); r.zhu = ref(k);
            save(f, 'r');
        end
        status(k) = r.status;
        if isempty(r.t) || r.t(end) < tPull
            fprintf('  %2.0f kPa: %s after %.0f s wall (%s)\n', pList(k)/1e3, upper(r.status), r.wall, r.msg);
            continue
        end
        fn = interp1(r.t, r.fn, tPull); bendG = rad2deg(interp1(r.t, r.bend, tPull));
        pull = -r.fLoad(r.t >= tPull);
        [F(k), iMax] = max(pull);
        F0 = pull(1);                      % gauge reading as the pull starts (the closing squeeze already loads it)
        tPk = r.t(find(r.t >= tPull, 1) - 1 + iMax);
        note = '';
        if tPk > r.t(end) - 0.05, note = sprintf(' (still rising at t = %.2f s: lower bound)', r.t(end)); end
        e = (F(k) - ref(k)) / ref(k);
        fprintf('  %2.0f kPa: bend at grasp [%s] deg, contact [%s] N | grasp force %.2f N at %.2f s (%.2f N at pull start) vs Zhu %.2f (%+.0f %%) %s%s | %s, %.0f s wall\n', ...
            pList(k)/1e3, strjoin(compose('%.0f', bendG), ' '), strjoin(compose('%.2f', fn), ' '), ...
            F(k), tPk, F0, ref(k), 100*e, passStr(abs(e) <= 0.30), note, r.status, r.wall);
    end
    ok = abs(F - ref) ./ ref <= 0.30;
    fprintf('  REQ_GRIP_012 %s: %d of %d within +/-30 %%\n', mode, nnz(ok), numel(ok));
    R.(mode) = struct('p', pList, 'z', zBall, 'F', F, 'zhu', ref, 'status', status, 'pass', ok);
end
end

function [o, z] = testBall(mode)
o = struct('name', "test ball", 'shape', "sphere", 'mass', 0.02, 'F_Bruise', Inf, ...
           'k_Contact', 5000, 'k_Table', 5000, 'mu_s', 0.8, 'mu_d', 0.64, 'estimated', "all", ...
           'p_HoldZhu', NaN, 'graspMode', mode);
if mode == "fingertip"
    r = 0.015; z = -0.090;
else
    r = 0.040; z = -(r + 1e-3); o.mass = 0.10;
end
o.semiAxes = [r r r];
zeta = 0.5;
o.c_Contact = 2 * zeta * sqrt(o.k_Contact * o.mass);
o.c_Table = o.c_Contact;
end

function s = passStr(ok)
s = 'FAIL'; if ok, s = 'PASS'; end
end
