function R = check_softgripper_finger(tests, idx)
% CHECK_SOFTGRIPPER_FINGER  M2 finger validation on FingerBench.slx.
%
%   R = CHECK_SOFTGRIPPER_FINGER()            runs T-F1 and T-F2
%   R = CHECK_SOFTGRIPPER_FINGER(["T-F2"])    runs a subset
%   R = CHECK_SOFTGRIPPER_FINGER("T-F2", idx) T-F2 on a subset of the 29 Table 1 points (row indices)
%
%   T-F1  free bend at 10..60 kPa vs Zhu 75/90/110 deg at 40/50/60 kPa           (REQ_GRIP_005, +/-10 deg)
%   T-F2  blocked tip force at the 29 Zhu Table 1 points, RMS vs Zhu              (REQ_GRIP_004, <= 0.08 N)
%   T-F3  sealed finger (FingerBenchSealed): 40 kPa, valve shut, tip moment bends it 10 deg more;
%         chamber pressure vs the adiabatic law p*V^gamma = const, V = V0 + D*sum(theta)  (REQ_GRIP_006, 1 %)
%
%   Every test prints its PREDICTION (the offline PRB model with the same identified parameters) before the
%   Simscape result. The gauge plate is placed perpendicular to the tip at total bend Theta, at the tip
%   position the PRB statics give under load -- the same geometry assumption the identification used
%   (Zhu do not state the gauge orientation). No Optimization Toolbox needed (small Newton solver).
%   Read-only on the models; does not cd. Uses SimulationInput only.

arguments
    tests (1,:) string = ["T-F1", "T-F2"]
    idx double = []
end
fi = params_softgripper_finger; ga = params_softgripper_gas(fi);
base = struct('finger', fi, 'gas', ga, 'bench', benchDefaults());
R = struct();

if any(tests == "T-F1")
    pList = (10:10:60) * 1e3; zhu = [NaN NaN NaN 75 90 110];
    pred = arrayfun(@(p) rad2deg(fi.N_Joint * freeJoint(fi, p)), pList);
    fprintf('\n=== T-F1 free bend: prediction (offline PRB) %s deg ===\n', mat2str(round(pred, 1)));
    in = arrayfun(@(p) mkInput(base, p, farGauge()), pList);
    out = sim(in, 'ShowProgress', 'off');
    sim1 = arrayfun(@(o) rad2deg(lastVal(o, 'bend')), out);
    for k = 1:numel(pList)
        fprintf('  %2.0f kPa: Simscape %6.1f deg | prediction %6.1f | Zhu %s\n', pList(k)/1e3, sim1(k), pred(k), num2str(zhu(k)));
    end
    ok = all(abs(sim1(4:6) - zhu(4:6)) <= 10);
    fprintf('  REQ_GRIP_005 (+/-10 deg at 40/50/60 kPa): %s  (errors %s deg)\n', passStr(ok), mat2str(round(sim1(4:6) - zhu(4:6), 1)));
    R.TF1 = struct('p', pList, 'sim', sim1, 'pred', pred, 'zhu', zhu, 'pass', ok);
end

if any(tests == "shapes")      % diagnostic: identified loaded shape at selected gauge points
    for cs = [60e3 0; 60e3 deg2rad(15); 40e3 deg2rad(30); 60e3 deg2rad(60)]'
        g = gaugePose(fi, base.bench, cs(1), cs(2));
        fprintf('  %2.0f kPa %3.0f deg: F = %.2f N, joints [%s] deg\n', cs(1)/1e3, rad2deg(cs(2)), g.Fpred, ...
                strjoin(compose('%.1f', rad2deg(g.th')), ' '));
    end
end

if any(tests == "T-F2")
    [pts, meas] = zhuTable1();
    if ~isempty(idx), pts = pts(idx, :); meas = meas(idx); end
    gauge = arrayfun(@(k) gaugePose(fi, base.bench, pts(k,1), pts(k,2)), 1:size(pts,1));
    pred = [gauge.Fpred]';
    fprintf('\n=== T-F2 blocked force, %d points: prediction RMS vs Zhu %.3f N ===\n', numel(meas), rms(pred - meas));
    in = arrayfun(@(k) mkInput(base, pts(k,1), gauge(k), 'FingerBenchGauge'), 1:size(pts,1));
    out = sim(in, 'ShowProgress', 'off');
    simF = arrayfun(@(o) gaugeReading(o), out)';
    for k = 1:numel(meas)
        fprintf('  %2.0f kPa %3.0f deg: Zhu %.2f | prediction %.2f | Simscape %.2f N\n', pts(k,1)/1e3, rad2deg(pts(k,2)), meas(k), pred(k), simF(k));
    end
    e = simF - meas;
    ok = rms(e) <= 0.08;
    fprintf('  REQ_GRIP_004: Simscape RMS vs Zhu %.3f N, max %.3f N (limit 0.08 N RMS): %s\n', rms(e), max(abs(e)), passStr(ok));
    fprintf('  Simscape vs offline prediction: RMS %.4f N (model-consistency check)\n', rms(simF - pred));
    R.TF2 = struct('pts', pts, 'zhu', meas, 'pred', pred, 'sim', simF, 'rms', rms(e), 'max', max(abs(e)), 'pass', ok);
end

if any(tests == "T-F3")
    b = base.bench; b.pTest = 40e3; b.tStep = 0.05; b.tauRamp = 0.05;
    b.tSeal = 1.0; b.tauValve = 0.01; b.aLeak = 1e-12;   % valve shut to a 1e-12 m^2 leak (block minimum area)
    % Test valve = 0.3 mm needle valve with ports 1.2x its orifice. Local Restriction (G) conducts heat through its
    % PORT area even when shut; with 4 mm ports the fill-heated air (about 314 K, insulated chambers) lost 0.02 W to
    % the 293 K supply and p*V^gamma drifted 0.5 %/s. Small ports cut that about 150x.
    b.aValve = pi/4 * (0.3e-3)^2;
    b.tPush = 1.5; b.tauPush = 0.05; b.dBend = deg2rad(10);
    gam = sealedGamma(ga);
    s = sealedPred(fi, ga, b.pTest, b.dBend, gam);
    b.mPush = s.M;
    fprintf('\n=== T-F3 sealed finger at %.0f kPa, +%.0f deg by a tip moment ===\n', b.pTest/1e3, rad2deg(b.dBend));
    fprintf('  PREDICTION: bend %.1f -> %.1f deg, V %.2f -> %.2f mL, p %.2f -> %.2f kPa gauge (gamma %.4f), tip moment %.4f N*m\n', ...
        rad2deg(s.bend1), rad2deg(s.bend2), s.V1*1e6, s.V2*1e6, (s.p1-ga.p_Atm)/1e3, (s.p2-ga.p_Atm)/1e3, gam, s.M);
    P = base; P.bench = b;
    in = Simulink.SimulationInput('FingerBenchSealed');
    in = in.setVariable('P', P);
    in = in.setModelParameter('StopTime', '3', 'Solver', 'ode23t', 'RelTol', '1e-5');
    out = sim(in);
    win1 = @(t) t >= b.tPush - 0.2 & t < b.tPush;  win2 = @(t) t >= 2.8;
    th = out.logsout.get('theta').Values; pp = out.logsout.get('pFinger').Values;
    sumTh = sum(squeeze(th.Data), find(size(squeeze(th.Data)) == fi.N_Joint, 1));
    [t1, sth1] = deal(th.Time, sumTh(:));
    p1 = mean(pp.Data(win1(pp.Time))); p2 = mean(pp.Data(win2(pp.Time)));
    S1 = mean(sth1(win1(t1))); S2 = mean(sth1(win2(t1)));
    V1 = fi.V0 + fi.D*S1; V2 = fi.V0 + fi.D*S2;
    p2law = p1 * (V1/V2)^gam;
    err = (p2 - p2law) / p2law;
    errG = (p2 - p2law) / (p1 - p2law);
    fprintf('  Simscape:   bend %.1f -> %.1f deg, V %.2f -> %.2f mL, p %.2f -> %.2f kPa gauge\n', ...
        rad2deg(S1), rad2deg(S2), V1*1e6, V2*1e6, (p1-ga.p_Atm)/1e3, (p2-ga.p_Atm)/1e3);
    fprintf('  adiabatic law from the simulated volumes: p2 = %.2f kPa gauge; error %.3f %% of p (abs), %.2f %% of the pressure change\n', ...
        (p2law-ga.p_Atm)/1e3, 100*err, 100*errG);
    ok = abs(err) <= 0.01;
    fprintf('  REQ_GRIP_006 (within 1 %% of the adiabatic prediction): %s\n', passStr(ok));
    R.TF3 = struct('pred', s, 'p1', p1, 'p2', p2, 'p2law', p2law, 'V1', V1, 'V2', V2, 'err', err, 'errChange', errG, 'pass', ok);
end
end

function g = sealedGamma(ga)
% the model's gas: Gas Properties (G) perfect gas, cp_ref 1 kJ/(kg*K), R = P.gas.R_Air
cp = 1000; g = cp / (cp - ga.R_Air);
end

function s = sealedPred(fi, ga, pG, dBend, gam)
% Offline: free bend at pG, sealed; a uniform tip moment M adds dBend. Each hinge: K th + K3 th^3 = D max(p-p0,0) + M,
% with p = p1 (V1/V)^gam, V = V0 + D N th. Solve for th2 = th1 + dBend/N, then M.
N = fi.N_Joint;
th1 = freeJoint(fi, pG); p1 = pG + ga.p_Atm;
V1 = fi.V0 + fi.D*N*th1; th2 = th1 + dBend/N; V2 = fi.V0 + fi.D*N*th2;
p2 = p1 * (V1/V2)^gam;
M = fi.K*th2 + fi.K3*th2^3 - fi.D*max(p2 - ga.p_Atm - fi.p0, 0);
s = struct('bend1', N*th1, 'bend2', N*th2, 'V1', V1, 'V2', V2, 'p1', p1, 'p2', p2, 'M', M);
end

% ==================================================================================================
function b = benchDefaults()
b.pTest = 0; b.tauRamp = 0.05; b.tStep = 0.05;
b.rProbe = 1e-3; b.mProbe = 1e-4; b.tPlate = 5e-3;
b.kGauge = 1e5; b.cGauge = 50;        % N/m, N/(m/s): stiff gauge -> 14 um penetration at 1.4 N
b.gaugeR = eye(3); b.gaugeT = [1 1 1];
end

function g = farGauge()
g = struct('R', eye(3), 'T', [1 1 1], 'Fpred', NaN);   % plate 1.7 m away: free bend
end

function in = mkInput(base, p, g, mdl)
if nargin < 4, mdl = 'FingerBench'; end
P = base; P.bench.pTest = p; P.bench.gaugeR = g.R; P.bench.gaugeT = g.T;
in = Simulink.SimulationInput(mdl);
in = in.setVariable('P', P);
in = in.setModelParameter('StopTime', '2', 'Solver', 'ode23t', 'RelTol', '1e-4');
end

function v = lastVal(o, name)
x = o.logsout.get(name).Values; v = x.Data(end);
end

function v = gaugeReading(o)
% FingerBenchGauge: prismatic slide (axis = tip direction) holds the tip angle at Theta and blocks motion normal
% to the tip; its constraint force resolved in the base frame has the in-plane normal as component 2.
x = o.logsout.get('fGaugeVec').Values; t = x.Time; d = squeeze(x.Data);
if size(d, 1) ~= numel(t), d = d.'; end
v = mean(abs(d(t >= t(end) - 0.2, 2)));
end

function v = meanLast(o, name, dur)
x = o.logsout.get(name).Values; t = x.Time; d = squeeze(x.Data);
v = mean(d(t >= t(end) - dur));
end

function s = passStr(ok)
s = 'FAIL'; if ok, s = 'PASS'; end
end

function th = freeJoint(fi, p)
tau = fi.D * max(p - fi.p0, 0);
if tau <= 0, th = 0; return, end
th = fzero(@(t) fi.K*t + fi.K3*t^3 - tau, [0 2*pi]);
end

function [pts, meas] = zhuTable1()
% Zhu et al. 2022, Table 1 (no rings): rows 10..60 kPa, columns 0..90 deg; NaN = not recorded
F = [0.07 NaN  NaN  NaN  NaN  NaN  NaN
     0.24 0.15 0.08 NaN  NaN  NaN  NaN
     0.46 0.36 0.29 0.21 0.11 NaN  NaN
     0.72 0.61 0.53 0.45 0.36 0    NaN
     1.08 0.95 0.84 0.73 0.64 0.52 0
     1.36 1.22 1.09 0.95 0.84 0.71 0.55];
[iP, iA] = find(~isnan(F));
p = (10:10:60)' * 1e3; a = deg2rad(0:15:90)';
pts = [p(iP), a(iA)]; meas = F(~isnan(F));
end

function g = gaugePose(fi, bench, p, Theta)
% Tip position under the gauge load (PRB statics, tip held at total bend Theta, force normal to the tip) and
% the gauge-slide base pose for FingerBenchGauge: rotation Rz(Theta)*Ry(+90 deg), so the slide axis (base Z)
% lies along the tip direction and base Y is the in-plane normal; translation = the loaded tip point. If the
% finger cannot reach Theta, the slide sits on the uniform arc and the force is 0.
N = fi.N_Joint; l = fi.L / N;
tau = fi.D * max(p - fi.p0, 0);
if N * freeJoint(fi, p) <= Theta
    th = repmat(Theta / N, N, 1); F = 0;
else
    z = newton(@(z) eqs(z, tau, fi.K, fi.K3, l, N, Theta), [repmat(Theta/N, N, 1); 0.5]);
    th = z(1:N); F = max(z(end), 0);
end
c = cumsum(th); tip = [sum(l*cos(c)), sum(l*sin(c)), 0];
Rz = [cos(Theta) -sin(Theta) 0; sin(Theta) cos(Theta) 0; 0 0 1];
Ry = [0 0 1; 0 1 0; -1 0 0];                 % +90 deg about Y: new Z = old X (the tip direction)
g.T = tip;
g.R = Rz * Ry;
g.Fpred = F;
g.th = th;
end

function r = eqs(z, tau, K, K3, l, N, Theta)
th = z(1:N); Fg = z(end); c = cumsum(th);
pts = [0 0; cumsum(l*[cos(c) sin(c)], 1)];
tipDir = [cos(c(end)) sin(c(end))];
fVec = -Fg * [-tipDir(2) tipDir(1)];
r = zeros(N+1, 1);
for i = 1:N
    arm = pts(end,:) - pts(i,:);
    r(i) = K*th(i) + K3*th(i)^3 - tau - (arm(1)*fVec(2) - arm(2)*fVec(1));
end
r(end) = sum(th) - Theta;
end

function z = newton(fun, z)
for it = 1:60
    r = fun(z); if norm(r) < 1e-12, return, end
    n = numel(z); J = zeros(n); h = 1e-7 * max(1, abs(z));
    for j = 1:n, dz = zeros(n,1); dz(j) = h(j); J(:,j) = (fun(z + dz) - r) / h(j); end
    step = -J \ r; a = 1;
    while norm(fun(z + a*step)) > norm(r) && a > 1e-4, a = a/2; end
    z = z + a*step;
end
end
