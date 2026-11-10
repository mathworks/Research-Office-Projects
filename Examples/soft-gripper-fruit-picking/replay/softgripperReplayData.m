function replay = softgripperReplayData(objId, fps, view)
% SOFTGRIPPERREPLAYDATA  Logged pick -> the inputs of SoftGripperScene3D.slx (REQ_GRIP_029: reads logs only).
%
%   replay = SOFTGRIPPERREPLAYDATA(objId)        from results/pick_<objId>.mat, 30 frames/s
%   replay = SOFTGRIPPERREPLAYDATA(objId, fps)
%   replay = SOFTGRIPPERREPLAYDATA(objId, fps, "close")   camera close to the grasp
%
%   replay.tree, replay.cfg0, replay.cfg (timeseries, 6 arm joints qArmMeas); per finger k: replay.fCfgk (8 hinges
%   from thetaJoints), replay.fTk / replay.fRk (base translation, [yaw pitch roll]); replay.objPos /
%   replay.objRot (timeseries, from zTruth objPose), sizes, colours, table and tray poses, replay.Ts, replay.tEnd.

arguments
    objId (1,1) double
    fps (1,1) double = 30
    view (1,1) string = "wide"      % "wide" (whole scene) or "close" (the grasp, fingers and fruit)
end
resDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
S = load(fullfile(resDir, sprintf('pick_%d.mat', objId)), 'out');
P = softgripperPickParams(objId);
ls = S.out.logsout;
e = ls.find('Name', 'qArmMeas'); q = e{1}.Values; qd = squeeze(q.Data); if size(qd, 1) ~= numel(q.Time), qd = qd.'; end
e = ls.find('Name', 'zTruth'); z = e{1}.Values; zd = squeeze(z.Data); if size(zd, 1) ~= numel(z.Time), zd = zd.'; end
replay.Ts = 1 / fps;
t = (0:replay.Ts:min(q.Time(end), z.Time(end)))';
qa = interp1(q.Time, qd, t, 'previous', 'extrap');
th = interp1(z.Time, zd(:, 12:43), t, 'linear', 'extrap');
pose = interp1(z.Time, zd(:, 5:10), t, 'linear', 'extrap');
replay.tree = softgripperReplayTree(P);              % UR5e + palm only (serial)
replay.cfg0 = qa(1, :);
replay.cfg = timeseries(qa, t);
% Fingers (revised 2026-10-26): the Simulation 3D Robot block mis-draws branching trees (one fingertip angle moved
% the palm, so the wrist wobbled). Each finger is now its own serial robot (softgripperFingerTree) whose BASE is
% placed every frame on its mount: T_base = T_tool0(q) * FlangeToGripper * Mount_k * C, the same chain as
% SoftGripperPick/Plant/Gripper. C maps the display finger frame (hinge X, link Z) onto the simulation finger
% frame (hinge z, link x). The robot block takes its base pose in the MATLAB frame of the arm drawing (no actor
% mirror), as translation [x y z] and rotation [yaw pitch roll] (ZYX), lifted by h like the arm.
h = 0.75; replay.h = h;
replay.fingerTree = softgripperFingerTree(P);
rb = loadrobot('universalUR5e', 'DataFormat', 'row');
Rflange = [1 0 0; 0 -1 0; 0 0 -1];                   % FlangeToGripper: 180 deg about tool0 X
C = [0 0 1; 0 -1 0; 1 0 0];                          % columns: display X, Y, Z in the simulation finger frame
n = numel(t); N = P.finger.N_Joint;
fT = zeros(n, 3, 4); fR = zeros(n, 3, 4);
for i = 1:n
    Tt = getTransform(rb, qa(i, :), 'tool0');
    for k = 1:4
        Tk = Tt * [Rflange zeros(3, 1); 0 0 0 1] * [P.bench.mountR(:, :, k) * C, P.bench.mountT(k, :)'; 0 0 0 1];
        fT(i, :, k) = Tk(1:3, 4)' + [0 0 h];
        fR(i, :, k) = rotm2eul(Tk(1:3, 1:3), 'ZYX');
    end
end
fR = unwrap(fR);                                     % no 2*pi jumps between frames
fingerSign = +1;   % (was -1: that "reversed bend" was a gimbal-lock artefact of the old branching tree)
for k = 1:4
    replay.(sprintf('fCfg%d', k)) = timeseries(fingerSign * th(:, (k-1)*N + (1:N)), t);
    replay.(sprintf('fT%d', k)) = timeseries(fT(:, :, k), t);
    replay.(sprintf('fR%d', k)) = timeseries(fR(:, :, k), t);
end
replay.fCfg0 = zeros(1, N);
replay.fT0 = squeeze(fT(1, :, :))';                  % 4 x 3, initial base poses
replay.fR0 = squeeze(fR(1, :, :))';
% Display transforms only (the simulation is unchanged): (1) the Simulation 3D Robot block and the Simulation 3D
% Actor blocks mirror each other across the X-Z plane (seen in the first replay: the arm went to the mirror side),
% so actor and camera poses get y -> -y, roll -> -roll, yaw -> -yaw; (2) the scene is lifted by a table height h
% so the table shows above Unreal's ground (table top at z = 0 in the simulation).
mir = @(p) [p(:, 1), -p(:, 2), p(:, 3)];
mirR = @(r) [-r(:, 1), r(:, 2), -r(:, 3)];
lift = @(p) p + [0 0 h];
replay.objPos = timeseries(lift(mir(pose(:, 1:3))), t);
replay.objRot = timeseries(mirR(pose(:, 4:6)), t);
replay.objPos0 = lift(mir(pose(1, 1:3)));
replay.robotT = [0 0 h];
replay.pedestalT = mir([0 0 h/2]);
o = P.obj;
replay.objSize = 2 * o.semiAxes;
colours = struct('raspberry', [0.75 0.10 0.25], 'cherrytomato', [0.85 0.15 0.10], 'apple', [0.80 0.12 0.12], ...
                 'tomato', [0.90 0.25 0.10], 'pear', [0.70 0.75 0.20], 'mango', [0.95 0.65 0.15]);
replay.objColor = colours.(matlab.lang.makeValidName(strrep(o.name, " ", "")));
sc = P.scene;
replay.tableSize = [sc.wTable sc.wTable h];                      % the table reaches the ground
replay.tableT = mir([sc.tableXY h/2]);
replay.traySize = [sc.wTray sc.wTray sc.trayZ];
replay.trayT = lift(mir([sc.trayXY sc.trayZ/2]));
replay.trayStandT = mir([sc.trayXY h/2]);                         % a second table under the tray (display only)
replay.tEnd = t(end);
% ---- recording camera (inspect-sim3d-scenes conventions: Scene Origin mount, offsets on, rotation in RADIANS) ----
replay.camTs = 1 / 15;              % s      15 frames/s for the GIF
if view == "close"                  % close-up of the grasp: the fingers curling around the fruit (the softness)
    pG = pose(1, 1:3);
    look = lift(mir(pG + [0 0 0.03]));
    replay.camT = lift(mir(pG + [0.22 -0.20 0.05]));
else
    look = lift(mir([0.27 -0.15 0.12]));        % m  between the table and the tray (simulation coords -> display)
    replay.camT = lift(mir([0.95 -1.05 0.70]));  % m  camera position: tray side, so the table does not hide the arm
end
replay.view = view;
d = look - replay.camT;
replay.camR = [0, -atan2(-d(3), hypot(d(1), d(2))), atan2(d(2), d(1))];   % [roll pitch yaw] rad, pitch < 0 looks down
replay.objId = objId; replay.name = o.name;
end
