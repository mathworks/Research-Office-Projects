function plan = planSoftGripperPick(objPose, objId, P)
% PLANSOFTGRIPPERPICK  Offline pick plan: IK waypoints + minimum-jerk durations.
%
%   plan = PLANSOFTGRIPPERPICK(objPose, objId, P)
%     objPose  [x y z roll pitch yaw] camera estimate of the object pose (m, rad), objPoseInit
%     objId    1..6 (params_softgripper_objects order)
%     P        struct with fields finger, ctrl, arm, bench (tPalm)
%
%   plan.qWp     (7 x 6) rad  waypoints: Home, Above, Grasp, Lift, OverTray, Release, Home
%   plan.segQ0, plan.segQ1 (6 x 6), plan.segT (1 x 6) s   segments between consecutive waypoints
%   plan.zPalm   grasp height of the palm underside (m, world)
%   plan.aPeak   (1 x 6) planned peak tool acceleration (m/s^2); plan.ikErr  max IK position error (m)
%
%   Grasp height: table objects: palm underside at max(L + hClear, h + 1 mm) above the table. Stem objects
%   (raspberry, cherry tomato): the palm sits so the fully curled fingertips finish just under the fruit
%   (stemPalmOffset below). Durations: minimum-jerk (quintic) in joint space;
%   its peak tool acceleration scales exactly as 1/T^2 along a fixed path, so T = sqrt(a(T = 1 s) / a_lim),
%   then T >= the UR5e joint-speed bound 1.875*dq/qdMax and >= tMin. Lift and Lower use aLiftObj(objId),
%   Transport aTransportObj(objId); unloaded ones use a_Transport. Does not touch any model.

fi = P.finger; c = P.ctrl; tPalm = P.bench.tPalm;
objs = params_softgripper_objects(); o = objs(objId);
rb = loadrobot('universalUR5e', 'DataFormat', 'row');
ik = inverseKinematics('RigidBodyTree', rb);
Rdown = [1 0 0; 0 -1 0; 0 0 -1];
% Tool yaw: a fixed yaw would turn the wrist 95 deg on the way to the fruit. The four fingers sit at 90 deg, so
% any quarter turn gives the same grasp; take the quarter turn closest to the home tool yaw, so the wrist barely
% turns.
Th = getTransform(rb, P.arm.qHome, 'tool0');
best = Inf;
for kq = 0:3
    Rk = Rdown * [cos(kq*pi/2) -sin(kq*pi/2) 0; sin(kq*pi/2) cos(kq*pi/2) 0; 0 0 1];
    ang = acos(max(-1, min(1, (trace(Th(1:3,1:3)' * Rk) - 1) / 2)));
    if ang < best, best = ang; Rbest = Rk; end
end
Rdown = Rbest;
qdMax = pi; tMin = 0.3;                    % rad/s UR5e joint speed limit (datasheet 180 deg/s); s minimum segment
xy = objPose(1:2);
if ismember(objId, c.stemSet)              % stem / vine pick
    plan.zPalm = objPose(3) + stemPalmOffset(fi, o, c.pGrip(objId));
else
    plan.zPalm = max(fi.L + c.hClear, 2*o.semiAxes(3) + 1e-3);
end
tool = @(xy, z) [Rdown [xy(:); z + tPalm]; 0 0 0 1];
L = c.layout;
% Release: the fruit centre sits (zPalm - z_centre) below the palm while held, so its bottom is hRelease over
% the tray floor when the palm is at trayZ + c + hRelease + that offset.
zRel = L.trayZ + o.semiAxes(3) + L.hRelease + (plan.zPalm - objPose(3));
% ...but never so low that the fingers, straightening after the vent, reach into the tray floor (no finger-tray
% contact is modelled): palm >= tray floor + L + hClear. Binds only for the stem pick (raspberry).
zRel = max(zRel, L.trayZ + fi.L + c.hClear);
plan.zRelease = zRel;
% Tray waypoints: instead of holding the gripper square to the world (wrist 3 counter-turning about 90 deg
% against the base pan), let the gripper turn WITH the arm: rotate
% the tray-side tool orientation about world z by the base-angle change, so wrist 3 holds its angle. The fruit is
% already held, so its yaw in the tray does not matter.
dPsi = atan2(L.trayXY(2), L.trayXY(1)) - atan2(xy(2), xy(1));
Rz = [cos(dPsi) -sin(dPsi) 0; sin(dPsi) cos(dPsi) 0; 0 0 1];
toolTray = @(xy, z) [Rz * Rdown [xy(:); z + tPalm]; 0 0 0 1];
wp = {tool(xy, plan.zPalm + L.hAbove), tool(xy, plan.zPalm), tool(xy, plan.zPalm + L.hLift), ...
      toolTray(L.trayXY, zRel + L.hLift), toolTray(L.trayXY, zRel)};
q = zeros(7, 6); q(1,:) = P.arm.qHome; qg = q(1,:); plan.ikErr = 0;
for i = 1:numel(wp)
    q(i+1,:) = ik('tool0', wp{i}, ones(1,6), qg); qg = q(i+1,:);
    T = getTransform(rb, q(i+1,:), 'tool0');
    plan.ikErr = max(plan.ikErr, norm(T(1:3,4) - wp{i}(1:3,4)));
end
q(7,:) = P.arm.qHome;
plan.qWp = q;
plan.segQ0 = q(1:6,:); plan.segQ1 = q(2:7,:);
aLim = [c.a_Transport c.a_Transport c.aLiftObj(objId) c.aTransportObj(objId) c.aLiftObj(objId) c.a_Transport];
plan.segT = zeros(1, 6); plan.aPeak = zeros(1, 6);
for k = 1:6
    a1 = peakToolAccel(rb, plan.segQ0(k,:), plan.segQ1(k,:), 1);
    dq = max(abs(plan.segQ1(k,:) - plan.segQ0(k,:)));
    plan.segT(k) = max([sqrt(a1 / aLim(k)), 1.875 * dq / qdMax, tMin]);
    plan.aPeak(k) = a1 / plan.segT(k)^2;
end
plan.aLim = aLim;
plan.segName = ["Approach", "Descend", "Lift", "Transport", "Lower", "Retreat"];
end

function a = peakToolAccel(rb, q0, q1, T)
% Peak tool0 Cartesian acceleration of the quintic joint path q0 -> q1 in T (finite differences on FK).
n = 401; s = linspace(0, 1, n); h = T / (n - 1);
p = zeros(n, 3);
for i = 1:n
    sig = 10*s(i)^3 - 15*s(i)^4 + 6*s(i)^5;
    Tf = getTransform(rb, q0 + (q1 - q0) * sig, 'tool0'); p(i,:) = Tf(1:3,4)';
end
acc = (p(3:end,:) - 2*p(2:end-1,:) + p(1:end-2,:)) / h^2;
a = max(vecnorm(acc, 2, 2));
end

function d = stemPalmOffset(fi, o, pGrip)
% Palm-underside height above the hanging fruit's centre. Take the 8-link chain fully curled at its free bend
% for pGrip (aiming at the first touch of a smooth arc instead makes the real tips sweep past the fruit).
N = fi.N_Joint; l = fi.L / N; h2 = fi.h_Link / 2;
tau = fi.D * max(pGrip - fi.p0, 0);
th = fzero(@(t) fi.K*t + fi.K3*t^3 - tau, [0 2*pi]);       % free hinge angle at pGrip
ang = (1:N) * th;
inward = sum(l * sin(ang)); depth = sum(l * cos(ang));      % tip from its base: toward the axis, down
rTip = fi.r_Mount - inward - h2;                            % tip inner face from the axis
if rTip >= o.semiAxes(1)
    warning('stemPalmOffset:noReach', 'curled tips stay %.1f mm from the axis, outside the fruit (%.1f mm)', rTip*1e3, o.semiAxes(1)*1e3);
end
% The curled tips finish 1 mm under the fruit's bottom, so they close under it (tips 0.5c below the centre
% lift the berry 39 mm, stem pull 7.7 N > F_Detach, T-I7).
d = depth - o.semiAxes(3) - 1e-3;
end
