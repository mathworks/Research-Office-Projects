function scene = params_softgripper_scene(ctrl, obj, objId)
% PARAMS_SOFTGRIPPER_SCENE  Table, tray and stem of the pick scene (SoftGripperPick/Plant/Object).
%
%   scene = PARAMS_SOFTGRIPPER_SCENE(ctrl, obj, objId)
%     ctrl   params_softgripper_ctrl (layout, F_Detach)
%     obj    the selected object struct (params_softgripper_objects(objId))
%     objId  1..6; ctrl.stemSet hang from a stem: the raspberry and the cherry tomato on its vine
%
%   Layout values are EST. Units SI, world frame, table top z = 0.

g = 9.80665;
scene.tableXY = ctrl.layout.objXY;  % m      table centred under the object
scene.wTable = 0.30;                % m      EST
scene.tTable = 0.02;                % m      EST
scene.trayXY = ctrl.layout.trayXY;  % m      EST
scene.trayZ = ctrl.layout.trayZ;    % m      tray floor thickness = floor height over the table (EST)
scene.wTray = 0.20;                 % m      EST, > the mango (94 mm) + clearance
scene.hWall = 0.05;                 % m      EST
scene.tWall = 0.005;                % m      EST
% ---- stem / vine (ctrl.stemSet) -----------------------------------------------------------------------------
scene.stemOn = double(ismember(objId, ctrl.stemSet));  % -  1 = a stem/vine holds the object
scene.kStem = 200;                  % N/m    EST: soft cane, 10 mm stretch at F_Detach (2.0 N)
scene.cStem = 2 * 0.5 * sqrt(scene.kStem * obj.mass);   % N/(m/s)  zeta 0.5, derived
scene.tauBreak = 1e-3;              % s      the stem breaks within 1 ms once overloaded (latch)
pInit = [ctrl.layout.objXY ctrl.layout.stemZ];
scene.stemAnchor = pInit + [0 0 obj.mass * g / scene.kStem] * scene.stemOn;   % m  spring in equilibrium at pInit
end
