function ctrl = params_softgripper_ctrl(gas)
% PARAMS_SOFTGRIPPER_CTRL  Task limits, sensor models and pick-controller parameters.
%
%   ctrl = PARAMS_SOFTGRIPPER_CTRL(gas)
%
%   Every design value traces to a row of docs/rationale.md (R1..R16),
%   approved at GATE D 2026-10-24. Per-object values are indexed by objId (params_softgripper_objects order:
%   raspberry, cherry tomato, apple, tomato, pear, mango).

arguments
    gas (1,1) struct
end
% ---- task limits (requirements) ----------------------------------------------------------------------
ctrl.T_PickMax = 15;                % s        REQ_GRIP_018 (engineer)
ctrl.N_Attempts = 3;                % -        REQ_GRIP_018
ctrl.s_SlipMax = 5e-3;              % m        REQ_GRIP_017 (GATE A)
ctrl.a_Transport = 2;               % m/s^2    REQ_GRIP_017 (GATE A)
ctrl.mu_Min = 0.5;                  % -        REQ_GRIP_024 (test plan)

% ---- sensors (y) ------------------------------------------------------------------------------------
ctrl.sens.pAcc = gas.p_GripAcc * gas.p_GripFS;   % Pa   +/- 2 kPa band (ITV101x output accuracy)
ctrl.sens.pQuant = 1e3;             % Pa       ITV display unit 0.001 MPa (p. 1183)
ctrl.sens.bendNoise = deg2rad(0.5); % rad      EST: Flex Sensor 2.2 noise, not published by Zhu
ctrl.sens.qArmNoise = 0;            % rad      UR5e encoders treated as exact (kinematic arm)
ctrl.sens.seed = 23;                % -        bend-noise seed (4 channels: seed..seed+3); change for Monte Carlo

% ---- pick controller (rationale.md) ---------------------------------------------------------------------
ctrl.Ts_Ctrl = 5e-3;                % s        R1: >= 7 samples over the 35 ms closing transient; tauMotion/4
ctrl.tCloseRamp = 0.3;              % s        R2 (revised 2026-10-25, T-I7): shortest ramp keeping the closing
                                    %          impact under the bruise limits (tomato 9.7 < 11 N, pear 24.9 < 40 N)
ctrl.pGripMin = 30e3;               % Pa       validity range while gripping (engineer 2026-10-24)
ctrl.pGripMax = gas.p_GripMax;      % Pa       finger rating 60 kPa
%                 rasp    cherry  apple   tomato  pear    mango
ctrl.pHold    = [ NaN     39.1e3  34.2e3  34.0e3  43.7e3  59.8e3];   % Pa  T-I2 minimum hold, PICK placement
                                    %          (TI2p; pear and mango lying on their side; raspberry: stem, R8)
ctrl.pLateral = [ NaN     NaN     45e3    39.2e3  50.1e3  60e3];     % Pa  R3b: lowest tested p with sideways
                                    %          capacity >= 2*aTransportObj (T-I4, gravity on, toward a finger)
ctrl.aTransportObj = [2 2 2 2 2 0.5];                                  % m/s^2  R6; mango R7 (Gate D)
ctrl.aLiftObj      = [2 2 2 2 1 0.5];                                  % m/s^2  Lift and Lower; pear 1 (R3c)
ctrl.dFdp = 0.21 / 1e3;             % N/Pa     R3: T-I1 enveloping slope 40 -> 50 kPa
ctrl.loadMargin = 0.2;              % -        R3: 20 % load margin
ctrl.F_Detach = 2.0;                % N        REQ_GRIP_027 stem detach force (raspberry); the cherry-tomato vine
                                    %          uses the same EST value until sourced
ctrl.stemSet = [1 2];               % -        objIds hanging from a stem/vine: raspberry (Gate D), cherry tomato
                                    %          (engineer 2026-10-25: unreachable from the table at r_Mount 60 mm)
ctrl.pGripStem = 55e3;              % Pa       R17 (EST): grip >= 1.2*(F_Detach + m g) = 2.45 N; the fingertip
                                    %          height sweep gave 2.3-2.8 N at 50-60 kPa (z -80 mm). M4 verifies
ctrl.pGripR3 = gripPressures(ctrl, gas);                               % Pa   R3/R3b (superseded, kept for trace)
% R3c (engineer 2026-10-25): T-I6 capacity ratio of a grasp FORMED at p under g_eff = g - a_gripper (fingers and
% fruit loaded), calibrated on the closed-loop picks: held apple 45 (1.66), tomato 39.2 (1.68/1.75); slipped apple
% 38.6 (1.27), pear 50.1 lift (1.26) -> require ratio >= 1.66 in lift and carry. Values below meet it (pear only
% with a 1 m/s^2 lift, R3c), except the mango (ratio about 1.0 at 60 kPa: beyond the rating, out of the pick set).
%                 rasp    cherry  apple   tomato  pear    mango
ctrl.pGrip    = [ 55e3    41.3e3  45e3    39.2e3  60e3    60e3];      % Pa   R3c (raspberry R17 EST, stem)
ctrl.pickSet = [3 4 5];             % -        objIds picked: apple, tomato, pear (engineer 2026-10-25). The mango is
                                    %          beyond the 60 kPa rating (R3c); the raspberry and cherry tomato are
                                    %          too small for the 60 mm finger spacing, from the table or a stem (R8b)
ctrl.dBendContact = deg2rad(5);     % rad      R4
ctrl.stallRate = deg2rad(30);       % rad/s    R5
ctrl.nStall = 10;                   % samples  R5 (50 ms)
ctrl.tSettle = 0.3;                 % s        R9
ctrl.qTol = 5e-4;                   % rad      R10
ctrl.tCloseMax = 0.85;              % s        R11: (ramp 0.3 + bend 0.06 + stall 0.05) x 2
ctrl.tVent = 0.2;                   % s        R12
ctrl.pDroopTol = 4e3;               % Pa       R13
ctrl.tDroop = 0.1;                  % s        R13
ctrl.hClear = 5e-3;                 % m        R14
ctrl.bendFreeP = (10:10:60) * 1e3;                                     % Pa   R16 (T-F1)
ctrl.bendFreeTh = deg2rad([0 38.0 64.6 81.1 93.5 103.7]);              % rad  R16
ctrl.bendValid = deg2rad([-10 200]);% rad      spec section 6: outside -> treated as no contact

% ---- layout (R15, EST) ------------------------------------------------------------------------------------
ctrl.layout.objXY = [0.45 0.10];    % m
ctrl.layout.trayXY = [0.10 -0.45];  % m
ctrl.layout.trayZ = 0.02;           % m        tray floor above the table (EST)
ctrl.layout.stemZ = 0.25;           % m        raspberry centre height when hanging from its stem (EST, R8)
ctrl.layout.hAbove = 0.10;          % m        approach height over the grasp pose (spec 4.2)
ctrl.layout.hLift = 0.15;           % m        lift height (spec 4.2)
ctrl.layout.hRelease = 0.02;        % m        release height over the tray (spec 4.2)
end

function p = gripPressures(ctrl, gas)
% R3: p = p_hold + p_Acc + margin*W/(dF/dp), W = m*(g + a); R3b: at least pLateral; clamped to [pGripMin, pGripMax].
% The raspberry (stem pick, no T-I2 value) uses pGripStem (R17).
objs = params_softgripper_objects();
g = 9.80665;
p = zeros(1, numel(objs));
for k = 1:numel(objs)
    W = objs(k).mass * (g + ctrl.aTransportObj(k));
    if isnan(ctrl.pHold(k)), p(k) = ctrl.pGripStem; continue, end
    p(k) = ctrl.pHold(k) + ctrl.sens.pAcc + ctrl.loadMargin * W / ctrl.dFdp;
    if ~isnan(ctrl.pLateral(k)), p(k) = max(p(k), ctrl.pLateral(k)); end   % R3b sideways rule
end
p = min(max(p, ctrl.pGripMin), gas.p_GripMax);
end
