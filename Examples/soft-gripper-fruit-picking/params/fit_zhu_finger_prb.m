function fit = fit_zhu_finger_prb(N, springModel, opts)
% FIT_ZHU_FINGER_PRB  Identify a pseudo-rigid-body (PRB) finger from Zhu et al. 2022, Table 1.
%
%   fit = FIT_ZHU_FINGER_PRB()                  N = 8 joints (one per chamber), linear joint springs
%   fit = FIT_ZHU_FINGER_PRB(N, "cubic")        joint spring K*theta + K3*theta^3 (silicone stiffens as
%                                               it stretches; Zhu's own model is hyperelastic, Yeoh)
%   fit = FIT_ZHU_FINGER_PRB(..., Quiet=true)   no printout
%
%   PARAMETER IDENTIFICATION. Zhu publish no finger length, hinge stiffness or pressure-to-torque factor,
%   but they do publish the fingertip force at 29 combinations of pressure and bend angle. This function
%   recovers the unknowns by matching that table.
%
%   Uses Optimization Toolbox (lsqnonlin, fsolve). Engineer decision 2026-10-23: keep the toolbox
%   version; the product is listed as needed only to re-run the identification.
%
%   Model, per joint i (links L/N long, joint at each link root):
%       K*theta_i [+ K3*theta_i^3] = D*max(p - p0, 0) + Mz_i(F)   (torque balance; Mz_i < 0 resists)
%       sum(theta_i) = Theta                                     (tip held at total bend Theta)
%   F is the gauge force at the tip, ASSUMED perpendicular to the tip direction (Zhu do not state the
%   gauge orientation -- labelled assumption). Fitted: D [N*m/Pa = m^3/rad per joint], K [N*m/rad],
%   p0 [Pa], L [m] and, for "cubic", K3 [N*m/rad^3].
%   Data: Zhu et al., Sensors 22:4532 (2022), PMC9230056, Table 1 (no rings), 29 recorded points.

arguments
    N (1,1) double {mustBeInteger, mustBePositive} = 8
    springModel (1,1) string {mustBeMember(springModel, ["linear","cubic"])} = "linear"
    opts.Quiet (1,1) logical = false
end
cubic = springModel == "cubic";
p = (10:10:60)' * 1e3;                         % Pa
ang = deg2rad(0:15:90);                        % rad
F = [0.07 NaN  NaN  NaN  NaN  NaN  NaN
     0.24 0.15 0.08 NaN  NaN  NaN  NaN
     0.46 0.36 0.29 0.21 0.11 NaN  NaN
     0.72 0.61 0.53 0.45 0.36 0    NaN
     1.08 0.95 0.84 0.73 0.64 0.52 0
     1.36 1.22 1.09 0.95 0.84 0.71 0.55];      % N, Table 1 ('/' = not recorded -> NaN)
[iP, iA] = find(~isnan(F));
meas = F(~isnan(F));

x0 = [2e-6, 0.03, 10e3, 0.08, 0];              % D, K, p0, L, K3   (initial guesses only)
lb = [1e-8, 1e-4, 0, 0.03, 0];
ub = [1e-3, 1, 30e3, 0.15, 10*cubic];
opt = optimoptions('lsqnonlin', 'Display', 'off', 'FunctionTolerance', 1e-12, 'StepTolerance', 1e-12);
res = @(x) arrayfun(@(k) tipForce(x, p(iP(k)), ang(iA(k)), N), (1:numel(meas))') - meas;
[x, rn] = lsqnonlin(res, x0, lb, ub, opt);

pred = res(x) + meas;
err = pred - meas;
thFree = arrayfun(@(pp) N * freeJointAngle(x, pp), [40e3 50e3 60e3]);
fit = struct('N', N, 'spring', springModel, 'D', x(1), 'K', x(2), 'p0', x(3), 'L', x(4), 'K3', x(5), ...
             'rmsErr', sqrt(rn/numel(meas)), 'maxErr', max(abs(err)), 'meas', meas, 'pred', pred, ...
             'p', p(iP), 'ang', ang(iA)', 'freeBend40_50_60', thFree);
if opts.Quiet, return, end
fprintf('\nPRB finger fit to Zhu Table 1 (N = %d joints, %s spring, %d points)\n', N, springModel, numel(meas));
fprintf('  D  = %.4g N*m/Pa per joint  (= %.3f mL of chamber growth per radian)\n', x(1), x(1)*1e6);
fprintf('  K  = %.4g N*m/rad,  K3 = %.4g N*m/rad^3  per joint\n', x(2), x(5));
fprintf('  p0 = %.2f kPa  (pressure offset)\n', x(3)/1e3);
fprintf('  L  = %.1f mm   (finger length -- not published, identified)\n', x(4)*1e3);
fprintf('  RMS error %.3f N, max %.3f N  (forces span 0..1.36 N)\n', fit.rmsErr, fit.maxErr);
fprintf('  free bend at 40/50/60 kPa: %.0f / %.0f / %.0f deg (Table 1 zero-force: 75 / 90; abstract: 110 at 60)\n', rad2deg(thFree));
fprintf('  point-by-point (measured -> model):\n');
for k = 1:numel(meas)
    fprintf('    %2.0f kPa %3.0f deg: %.2f -> %.2f N\n', fit.p(k)/1e3, rad2deg(fit.ang(k)), meas(k), pred(k));
end
end

function th = freeJointAngle(x, p)
% Joint angle with no tip load: K*th + K3*th^3 = D*max(p - p0, 0).
tauP = x(1) * max(p - x(3), 0);
if tauP <= 0, th = 0; return, end
th = fzero(@(t) x(2)*t + x(5)*t^3 - tauP, [0 2*pi]);
end

function Ft = tipForce(x, p, Theta, N)
% Joint angles and gauge force for a tip held at total bend Theta. Returns the gauge force (>= 0).
if N * freeJointAngle(x, p) <= Theta          % finger does not reach the gauge: no contact force
    Ft = 0; return
end
tauP = x(1) * max(p - x(3), 0); l = x(4) / N;
z0 = [repmat(Theta/N, N, 1); 0.5];
opt = optimoptions('fsolve', 'Display', 'off', 'FunctionTolerance', 1e-14, 'StepTolerance', 1e-14);
z = fsolve(@(z) eqs(z, tauP, x(2), x(5), l, N, Theta), z0, opt);
Ft = max(z(end), 0);
end

function r = eqs(z, tauP, K, K3, l, N, Theta)
th = z(1:N); Fg = z(end);
cumTh = cumsum(th);
pts = zeros(N+1, 2);
for i = 1:N
    pts(i+1, :) = pts(i, :) + l*[cos(cumTh(i)) sin(cumTh(i))];
end
tipDir = [cos(cumTh(end)) sin(cumTh(end))];
fVec = -Fg * [-tipDir(2) tipDir(1)];          % gauge pushes back against the bending direction
r = zeros(N+1, 1);
for i = 1:N
    arm = pts(end, :) - pts(i, :);
    Mz = arm(1)*fVec(2) - arm(2)*fVec(1);
    r(i) = K*th(i) + K3*th(i)^3 - tauP - Mz;
end
r(end) = sum(th) - Theta;
end
