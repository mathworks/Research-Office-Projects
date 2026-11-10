function r = runGripperBenchCase(P, pCmd, fPull, tEnd, wallLimit, zGauge, vGauge, fSide)
% RUNGRIPPERBENCHCASE  One GripperBench.slx run with a wall-clock limit; never throws.
%
%   r = RUNGRIPPERBENCHCASE(P, pCmd, fPull, tEnd, wallLimit)                 force-driven load (T-I2)
%   r = RUNGRIPPERBENCHCASE(P, pCmd, fPull, tEnd, wallLimit, zGauge, vGauge) plus a gauge spring (T-I1)
%     P          parameter struct (gripperBenchParams; P.bench.kGauge = 0 switches the gauge spring off)
%     pCmd       timeseries, gripper pressure command (Pa gauge)
%     fPull      timeseries, vertical force on the object (N, world z)
%     tEnd       stop time (s)
%     wallLimit  wall-clock limit (s); a run that hits it is reported, not hidden
%     zGauge, vGauge  timeseries, gauge-spring end position (m) and velocity (m/s), world z
%     fSide      timeseries (n x 2), horizontal force on the object (N, world x y); default zero
%
%   r.status   "ok" | "timeout" | "error";  r.msg  the message;  r.wall  wall time (s)
%   r.t, r.z, r.bend (n x 4), r.fn (n x 4, summed over the 8 links), r.pMan, r.fPalm, r.fLoad  (as logged)

if nargin < 6 || isempty(zGauge)
    zGauge = timeseries([0; 0], [0; tEnd]); vGauge = zGauge;
end
if nargin < 8
    fSide = timeseries(zeros(2, 2), [0; tEnd]);
end
ds = Simulink.SimulationData.Dataset;
ds{1} = pCmd; ds{2} = fPull; ds{3} = zGauge; ds{4} = vGauge; ds{5} = fSide;
in = Simulink.SimulationInput('GripperBench');
in = in.setVariable('P', P);
in = in.setExternalInput(ds);
in = in.setModelParameter('StopTime', num2str(tEnd));
in = in.setModelParameter('TimeOut', wallLimit);
r = struct('status', "ok", 'msg', "", 'wall', NaN, 't', [], 'z', [], 'xy', [], 'bend', [], 'fn', [], 'pMan', [], 'fPalm', [], 'fLoad', []);
t0 = tic;
try
    out = sim(in, 'ShowProgress', 'off');
catch err
    r.status = "error"; r.msg = string(err.message); r.wall = toc(t0);
    return
end
r.wall = toc(t0);
if ~isempty(out.ErrorMessage), r.status = "error"; r.msg = string(out.ErrorMessage); end
if isempty(out.tout) || out.tout(end) < tEnd - 1e-6
    if r.status == "ok", r.status = "timeout"; end
    r.msg = r.msg + sprintf(" stopped at t = %.3f s of %.3f s", max([out.tout; 0]), tEnd);
end
if isempty(out.tout), return, end
y = out.yout;
z = y.get('zObj').Values; r.t = z.Time; r.z = z.Data(:);
if any(strcmp(y.getElementNames, 'pObj'))
    x = y.get('pObj').Values; xd = squeeze(x.Data); if size(xd, 1) ~= numel(x.Time), xd = xd.'; end
    r.xy = interp1(x.Time, xd(:, 1:2), r.t);
end
b = y.get('bend').Values; r.bend = interp1(b.Time, b.Data, r.t);
r.fn = zeros(numel(r.t), 4);
for k = 1:4
    f = y.get(sprintf('fn%d', k)).Values; r.fn(:, k) = interp1(f.Time, sum(f.Data, 2), r.t);
end
p = y.get('pManifold').Values; r.pMan = interp1(p.Time, p.Data, r.t);
f = y.get('fPalm').Values; r.fPalm = interp1(f.Time, f.Data(:), r.t);
f = y.get('fLoad').Values; r.fLoad = interp1(f.Time, f.Data(:), r.t);
end
