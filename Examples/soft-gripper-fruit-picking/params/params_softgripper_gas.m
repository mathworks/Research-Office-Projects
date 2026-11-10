function gas = params_softgripper_gas(finger)
% PARAMS_SOFTGRIPPER_GAS  Gripper pneumatics: ITV101x-class regulator, valves, manifold, finger chambers.
%
%   gas = PARAMS_SOFTGRIPPER_GAS(finger)
%
%   Same derivation chain as the validated soft-arm gas network (../soft-arm/params_softarm_pneu.m):
%   ISO 6358 sonic conductance -> ideal-nozzle Cd*A -> divide by Cd and by the measured Local Restriction (G)
%   capacity factor (k_BlockCapacity = 1.1027, soft-arm S-07). Regulator gains by the same rule: full valve
%   opening at the output-accuracy band; integral time = flow-limited fill time. REQ_GRIP_008..010.

arguments
    finger (1,1) struct
end
% ---- air ------------------------------------------------------------------------------------------
gas.p_Atm = 101325;                 % Pa abs   standard
gas.T_Amb = 293.15;                 % K        standard
gas.R_Air = 287.05;                 % J/(kg*K) standard
gas.gamma_Air = 1.4;                % -        standard
gas.rho_ANR = gas.p_Atm / (gas.R_Air * gas.T_Amb);   % kg/m^3, ANR (SMC: 20 degC, 101.3 kPa)
g = gas.gamma_Air;
gas.Psi_Choked = (2/(g+1))^((g+1)/(2*(g-1)));        % -

% ---- regulator, SMC ITV101x (catalog p. 1183, 1186) ----------------------------------------------------
gas.p_GripFS = 0.1e6;               % Pa       set range 0.005-0.1 MPa
gas.p_GripMax = 60e3;               % Pa       finger rating, Zhu                     REQ_GRIP_008
gas.p_GripAcc = 0.02;               % fraction FS  output pressure accuracy
gas.p_Supply = 0.2e6;               % Pa gauge max supply 0.2 MPa
gas.p_SupplyMin = 0.16e6;           % Pa gauge set + 0.1 MPa for 60 kPa              REQ_GRIP_026
gas.Q_ANR = 80e-3/60;               % m^3/s    GRAPH READ: about 80 L/min ANR at 0.2 MPa supply, p. 1186
gas.C = gas.Q_ANR / (gas.p_Supply + gas.p_Atm);       % m^3/(s*Pa) ISO 6358
gas.Cd = 0.64;                      % -        modelling choice; only Cd*A sets choked flow
gas.k_BlockCapacity = 1.1027;       % -        measured, soft-arm S-07 (Local Restriction (G) vs nozzle)
gas.CdA_Catalog = gas.C * gas.rho_ANR * sqrt(gas.R_Air*gas.T_Amb/g) / gas.Psi_Choked;   % m^2
gas.a_Max = gas.CdA_Catalog / gas.Cd / gas.k_BlockCapacity;                           % m^2 supply = exhaust
gas.a_Leak = 1e-6 * gas.a_Max;      % m^2      numerical floor (> 0 required by the block)
gas.a_LeakFault = 0.1 * gas.a_Max;  % m^2      fault scenario T-N2                         REQ_GRIP_025

% ---- volumes ---------------------------------------------------------------------------------------------
gas.d_Tube = 4e-3;                  % m        ASSUMPTION (as the soft arm): 4 mm bore
gas.l_Tube = 1.0;                   % m        ASSUMPTION: regulator to manifold
gas.V_Line = pi/4 * gas.d_Tube^2 * gas.l_Tube;        % m^3  derived
gas.V_Manifold = 5e-6;              % m^3      EST
gas.V_Finger0 = finger.V0;          % m^3      EST, per finger (see params_softgripper_finger)
gas.V_FillTest = gas.V_Line + gas.V_Manifold + finger.n_Fingers * gas.V_Finger0;   % m^3, fingers straight
gas.a_FingerPort = pi/4 * (2e-3)^2; % m^2      ASSUMPTION: 2 mm finger inlet

% ---- regulator gains (plant, not design; never tuned for picking) --------------------------------------
gas.t_FillPred = fillTime(gas);     % s        independent prediction, REQ_GRIP_010
gas.Kp = gas.a_Max / (gas.p_GripAcc * gas.p_GripFS);  % m^2/Pa
gas.Ki = gas.Kp / gas.t_FillPred;   % m^2/(Pa*s)
gas.tau_PsFilter = 1e-4;            % s        numerical lag that breaks Simscape feedthrough loops
gas.p_Init = 0;                     % Pa gauge vented at start
end

function t95 = fillTime(gas)
% Ideal-nozzle + adiabatic charging of the straight-finger volume, 0 -> 95 % of p_GripMax.
g = gas.gamma_Air; R = gas.R_Air; T1 = gas.T_Amb;
p1 = gas.p_Supply + gas.p_Atm; CdA = gas.CdA_Catalog;
rc = (2/(g+1))^(g/(g-1));
pTarget = gas.p_Atm + 0.95 * gas.p_GripMax;
p = gas.p_Atm; t = 0; dt = 1e-6;
while p < pTarget
    r = p / p1;
    if r <= rc
        mdot = CdA * p1 * sqrt(g/(R*T1)) * gas.Psi_Choked;
    else
        mdot = CdA * p1 * sqrt(2*g/((g-1)*R*T1) * (r^(2/g) - r^((g+1)/g)));
    end
    p = p + g * R * T1 * mdot / gas.V_FillTest * dt;
    t = t + dt;
end
t95 = t;
end
