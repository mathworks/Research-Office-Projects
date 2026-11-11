function P = gripperBenchParams(o, zObj)
% GRIPPERBENCHPARAMS  Parameter struct for GripperBench.slx (4 fingers on a fixed flange, pointing down).
%
%   P = GRIPPERBENCHPARAMS(o, zObj)
%     o     object struct (an element of params_softgripper_objects, or a test sphere with the same fields)
%     zObj  object centre height below the flange (m, negative = below)
%
%   Flange at the world origin, fingers hanging along -Z. Finger k sits at angle phi_k = (k-1)*90 deg on radius
%   r_Mount. Finger base frame: X = finger axis (down), Y = bend direction (inward), Z = hinge axis (tangential),
%   the convention of FingerBench (links along +X, bending toward +Y).
%   Gravity is OFF on this bench: the identification (FingerBench) had gravity normal to the bend plane, so
%   the fingers carried none; the object's weight and any pull are applied as an explicit force (fPull).

fi = params_softgripper_finger; ga = params_softgripper_gas(fi);
b.gravity = [0 0 0];
b.wContact = 1e-5;                  % m      contact transition width (as T-O1; below the smallest penetration)
b.vCrit = 1e-3;                     % m/s    friction critical velocity
b.tPalm = 0.01; b.wPalm = 0.16;     % m      EST flange/palm plate, underside at z = 0 (fingers mount there)
b.kGauge = 0; b.cGauge = 0;         % N/m, N/(m/s)  gauge spring on the object; 0 = off (force-driven tests)
b.tauLoad = 1e-4;                   % s      lag on the load signal: breaks the zObj -> force algebraic loop
b.mountR = zeros(3, 3, 4); b.mountT = zeros(4, 3);
for k = 1:4
    phi = (k - 1) * pi/2;
    er = [cos(phi) sin(phi) 0];
    X = [0 0 -1]; Y = -er; Z = cross(X, Y);
    b.mountR(:, :, k) = [X' Y' Z'];
    b.mountT(k, :) = fi.r_Mount * er;
end
o.posInit = [0 0 zObj];
P = struct('finger', fi, 'gas', ga, 'obj', o, 'bench', b);
end
