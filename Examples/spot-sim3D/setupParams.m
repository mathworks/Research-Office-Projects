%% setupParams - Parameters for Spot Quadruped Trotting Gait Simulation
% This script defines all physical, gait, and control parameters used by
% spot_physics.slx. It also builds the rigidBodyTree for 3D visualization.
%
% Called automatically by the model's InitFcn callback, or run manually
% before simulation.

%% Robot geometry (from spot.urdf)
% These lengths define the kinematic chain of each 3-DOF leg.
% Measured from the URDF joint origins.
L_hip = 0.1109;              % Lateral offset from hip_x to upper_leg joint (m)
L_upper = 0.3205;            % Upper leg length, hip_y to knee (m)
L_lower = 0.33;              % Lower leg length, knee to foot (m)
L_knee_x = 0.025;            % Knee x-offset in upper_leg frame (m)

% Hip mount positions in body frame [x, y, z] per leg
% Measured from body center. Front hips are +x, rear are -x.
% Left hips are +y, right are -y.
hip_pos = [
    0.2979,  0.0550, 0;      % Front-Left
    0.2979, -0.0550, 0;      % Front-Right
   -0.2979,  0.0550, 0;      % Rear-Left
   -0.2979, -0.0550, 0;      % Rear-Right
];

%% Gait parameters
% These control the trotting gait timing and step geometry.
gait_period = 0.5;           % Full stride period (s) — one complete swing+stance cycle
step_length = 0.10;          % Forward step length (m) — foot travel during swing
step_height = 0.05;          % Foot lift height during swing (m)
body_height = 0.50;          % Nominal body height above ground (m)
body_speed = step_length / gait_period;  % Nominal forward speed (m/s)

%% Standing configuration
% Joint angles that place feet directly below hips with legs slightly bent.
% hip_x=0 (no abduction), hip_y=0.76rad (~44deg), knee=-1.4rad (~-80deg)
q_stand = [0; 0.76; -1.4];  % Per-leg standing angles [hip_x; hip_y; knee] (rad)

% Nominal foot position in hip frame (computed via FK at q_stand).
% Used by the gait planner as the neutral foot placement target.
foot_nom_L = [-0.0056; 0.1109; -0.5142];  % Left legs (m)
foot_nom_R = [-0.0056; -0.1109; -0.5142]; % Right legs (m)

%% Controller gains — PD joint tracking
% The PD controller uses different gains for swing vs stance legs.
% Stance legs are softer to allow compliant ground interaction.
% Swing legs are stiffer for accurate trajectory tracking.

% Stance: lower gains — ground contact provides most of the support
Kp_stance = 150;             % Proportional gain, stance (Nm/rad)
Kd_stance = 12;              % Derivative gain, stance (Nm*s/rad)

% Swing: higher gains — fast tracking of foot trajectory
Kp_swing = 300;              % Proportional gain, swing (Nm/rad)
Kd_swing = 20;               % Derivative gain, swing (Nm*s/rad)

% Body attitude stabilization gains (used by the height controller in GaitPlanner)
Kp_height = 3.0;            % Height proportional gain
Kd_height = 0.3;            % Height derivative gain
Kp_pitch = 1.5;             % Pitch proportional gain
Kd_pitch = 0.3;             % Pitch derivative gain
Kp_roll = 1.5;              % Roll proportional gain
Kd_roll = 0.3;              % Roll derivative gain

%% Gravity compensation constants
% Analytical approximation of gravitational torques on leg joints.
% Avoids full recursive Newton-Euler at each timestep.
% Derived from link masses and lengths at standing configuration:
%   tau_grav(hip_y) = c_hy * sin(q_hip_y) + c_kn * sin(q_hip_y + q_knee)
%   tau_grav(knee)  = c_kn * sin(q_hip_y + q_knee)
c_hy = 5.0306;              % Hip_y gravity torque coefficient (Nm)
c_kn = 0.8093;              % Knee gravity torque coefficient (Nm)

%% Torque limits
tau_max = 150;               % Joint torque saturation limit (Nm)

%% Simulation settings
stopTime = 5;                % Default simulation stop time (s)

%% 3D Visualization — Load pre-built rigidBodyTree for Simulation 3D Robot block
% The Sim3D Robot block requires a rigidBodyTree with 18 DOF:
%   6 floating-base joints (3 prismatic + 3 revolute) + 12 leg joints.
% The tree is pre-built with mesh visuals and golden color, saved to .mat.
% Loading it avoids re-parsing the URDF and reattaching visuals each run.
%
% To regenerate spotRobot3D.mat (e.g. after changing meshes or URDF):
%   warning('off','robotics:robotmanip:joint:ResettingHomePosition');
%   spotRobot3D = robotWith6DoFFloatingBase('row');
%   spotFixed = importrobot('spot.urdf');
%   spotFixed.DataFormat = 'row';
%   addSubtree(spotRobot3D, 'baseZRevBody', spotFixed, ReplaceBase=false);
%   warning('on','robotics:robotmanip:joint:ResettingHomePosition');
%   spotColor3D = [0.635 0.502 0.176];
%   meshDir3D = fullfile(pwd, 'spot_description', 'meshes');
%   for iBody = 1:numel(spotRobot3D.Bodies)
%       b3d = spotRobot3D.Bodies{iBody};
%       if ~isempty(b3d.Visuals)
%           vd = getVisual(b3d);
%           clearVisual(b3d);
%           meshFile3D = fullfile(meshDir3D, [b3d.Name '.dae']);
%           if isfile(meshFile3D)
%               addVisual(b3d, "Mesh", meshFile3D, vd(1).Tform, FaceColor=spotColor3D);
%           end
%       end
%   end
%   save('spotRobot3D.mat', 'spotRobot3D');
load('spotRobot3D.mat', 'spotRobot3D');
