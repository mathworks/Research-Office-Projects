function P = softgripperPickParams(objId)
% SOFTGRIPPERPICKPARAMS  The full parameter struct for one pick in SoftGripperPick.slx.
%
%   P = SOFTGRIPPERPICKPARAMS(objId)     objId 1..6 (raspberry, cherry tomato, apple, tomato, pear, mango)
%
%   P.finger, P.gas, P.ctrl, P.arm      the params_softgripper_* files
%   P.obj                                the selected object, with posInit (on the table, or hanging from the stem)
%   P.bench                              gripper geometry and contact settings (shared with GripperBench)
%   P.scene                              table, tray, stem
%   P.plan                               the offline pick plan (planSoftGripperPick, Gate D candidate A)
%   P.objPoseInit                        the camera estimate given to the controller (here: the true initial pose)

objs = params_softgripper_objects();
o = objs(objId);
fi = params_softgripper_finger; ga = params_softgripper_gas(fi);
ctrl = params_softgripper_ctrl(ga);
if ismember(objId, ctrl.stemSet)
    zc = ctrl.layout.stemZ;                         % hanging from its stem / vine (R8)
else
    zc = o.semiAxes(3);                             % resting on the table top (z = 0)
end
Pb = gripperBenchParams(o, zc);                     % bench geometry; sets o.posInit = [0 0 zc]
P = Pb;
P.obj.posInit = [ctrl.layout.objXY zc];
P.bench.gravity = [0 0 -9.80665];                   % the pick has gravity (the bench had it off)
P.ctrl = ctrl;
P.arm = params_softgripper_arm;
P.scene = params_softgripper_scene(ctrl, o, objId);
P.objId = objId;
P.objPoseInit = [P.obj.posInit 0 0 0];
P.plan = planSoftGripperPick(P.objPoseInit, objId, P);
end
