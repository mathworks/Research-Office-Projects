function R = check_softgripper_gas(tests)
% CHECK_SOFTGRIPPER_GAS  Gripper-gas validation on GasBench.slx.
%
%   R = CHECK_SOFTGRIPPER_GAS()           runs T-G1 and T-G2
%   R = CHECK_SOFTGRIPPER_GAS("T-G1")     runs a subset
%
%   GasBench: GripperGas (ITV101x-class PI regulator, supply/exhaust valves, line + manifold volume, four 2 mm
%   finger ports) feeding four sealed 10 mL volumes = fingers held straight.
%   T-G1  step pGripCmd 0 -> p_GripMax: finger t95 vs t_FillPred from params_softgripper_gas (REQ_GRIP_010,
%         +/-10 %); choked supply mass flow vs the catalog ISO 6358 flow (calibration check, 1.00 +/- 0.5 %)
%   T-G2  vent: step p_GripMax -> 0 after the fill: no negative gauge pressure
%   Prints the prediction before the Simscape result. Read-only on the models; does not cd; SimulationInput only.

arguments
    tests (1,:) string = ["T-G1", "T-G2"]
end
fi = params_softgripper_finger; ga = params_softgripper_gas(fi);
b = struct('tStep', 0.01, 'pCmd', ga.p_GripMax, 'pSupply', ga.p_Supply);
P = struct('finger', fi, 'gas', ga, 'bench', b);
R = struct();
g = ga.gamma_Air;
mdotChoked = ga.CdA_Catalog * (ga.p_Supply + ga.p_Atm) * sqrt(g / (ga.R_Air * ga.T_Amb)) * ga.Psi_Choked;

if any(tests == "T-G1")
    fprintf('\n=== T-G1 fill 0 -> %.0f kPa, %.1f mL (line + manifold + 4 straight fingers) ===\n', ga.p_GripMax/1e3, ga.V_FillTest*1e6);
    fprintf('  PREDICTION: t95 = %.1f ms (lumped volume, catalog nozzle flow); choked supply flow %.3f g/s\n', ga.t_FillPred*1e3, mdotChoked*1e3);
    in = Simulink.SimulationInput('GasBench');
    in = in.setVariable('P', P);
    in = in.setModelParameter('StopTime', '0.1', 'SimscapeLogType', 'all');
    out = sim(in);
    pf = out.logsout.get('pFinger1').Values; pm = out.logsout.get('pManifold').Values;
    t95f = crossTime(pf, 0.95 * ga.p_GripMax) - b.tStep;
    t95m = crossTime(pm, 0.95 * ga.p_GripMax) - b.tStep;
    sv = out.simlog.Gas.SupplyValve.mdot_A.series; ts = sv.time; md = abs(sv.values('kg/s'));
    early = ts > b.tStep + 1e-3 & ts < b.tStep + 3e-3;          % choked phase: manifold well below 0.53 p_supply
    ratio = mean(md(early)) / mdotChoked;
    e = (t95f - ga.t_FillPred) / ga.t_FillPred;
    fprintf('  Simscape:   t95 finger %.1f ms, manifold %.1f ms; choked supply flow %.3f g/s\n', t95f*1e3, t95m*1e3, mean(md(early))*1e3);
    fprintf('  REQ_GRIP_010 (finger t95 within 10 %% of %.1f ms): error %+.1f %% -> %s\n', ga.t_FillPred*1e3, 100*e, passStr(abs(e) <= 0.10));
    fprintf('  calibration: choked flow ratio %.4f (1.00 +/- 0.5 %%) -> %s\n', ratio, passStr(abs(ratio - 1) <= 0.005));
    fprintf('  final: finger %.2f kPa, manifold %.2f kPa (command %.0f kPa)\n', pf.Data(end)/1e3, pm.Data(end)/1e3, ga.p_GripMax/1e3);
    R.TG1 = struct('t95Finger', t95f, 't95Manifold', t95m, 'pred', ga.t_FillPred, 'err', e, 'flowRatio', ratio, ...
                   'pass', abs(e) <= 0.10 && abs(ratio - 1) <= 0.005);
end

if any(tests == "T-G2")
    fprintf('\n=== T-G2 vent %.0f -> 0 kPa at 0.1 s ===\n', ga.p_GripMax/1e3);
    fprintf('  PREDICTION: exhaust-limited decay, gauge pressure stays >= 0\n');
    in = Simulink.SimulationInput('GasBench');
    in = in.setVariable('P', P);
    % command p_GripMax from t = 0 (fills by 0.1 s, as T-G1 shows), then 0 at 0.1 s
    blk = 'GasBench/pGripCmd';
    in = in.setBlockParameter(blk, 'Time', '0.1');
    in = in.setBlockParameter(blk, 'InitialValue', 'P.bench.pCmd');
    in = in.setBlockParameter(blk, 'FinalValue', '0');
    in = in.setModelParameter('StopTime', '0.3');
    out = sim(in);
    pf = out.logsout.get('pFinger1').Values; pm = out.logsout.get('pManifold').Values;
    sel = pf.Time > 0.1;
    fprintf('  Simscape: before vent %.2f kPa; min after vent finger %.3f kPa, manifold %.3f kPa; at 0.3 s %.3f kPa\n', ...
        interp1(pf.Time, pf.Data, 0.099)/1e3, min(pf.Data(sel))/1e3, min(pm.Data(pm.Time > 0.1))/1e3, pf.Data(end)/1e3);
    tVent = crossTimeDown(pf, 0.05 * ga.p_GripMax) - 0.1;
    fprintf('  vent to 5 %% in %.1f ms\n', tVent*1e3);
    ok = min(pf.Data(sel)) >= -1 && min(pm.Data(pm.Time > 0.1)) >= -1;   % 1 Pa numerical tolerance
    fprintf('  no negative gauge pressure: %s\n', passStr(ok));
    R.TG2 = struct('pMin', min(pf.Data(sel)), 'tVent', tVent, 'pass', ok);
end
end

% ==================================================================================================
function t = crossTime(x, level)
k = find(x.Data(:) >= level, 1);
if isempty(k), t = NaN; return, end
t = interp1(x.Data(k-1:k), x.Time(k-1:k), level);
end

function t = crossTimeDown(x, level)
k = find(x.Time > 0.1 & x.Data(:) <= level, 1);
if isempty(k), t = NaN; return, end
t = interp1(x.Data(k-1:k), x.Time(k-1:k), level);
end

function s = passStr(ok)
s = 'FAIL'; if ok, s = 'PASS'; end
end
