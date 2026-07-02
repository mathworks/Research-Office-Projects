%% runSpotKeyboard - Run Spot quadruped with keyboard control and Sim3D visualization
%
% This script is the main entry point. It:
%   1. Loads the spot_physics model (which triggers setupParams via InitFcn)
%   2. Sets simulation time to 30 seconds for interactive use
%   3. Opens the keyboard control window (must stay focused for input)
%   4. Runs the simulation with Unreal Engine 3D rendering
%
% Controls:
%   Arrow Up/Down    — Walk forward/backward
%   Arrow Left/Right — Turn left/right
%   A/D              — Sidestep left/right
%   Combinations     — e.g. Up+Left = steer left while walking
%   No keys          — Stand still (idle gait)

% Open model if needed
if ~bdIsLoaded('spot_physics')
    open_system('spot_physics');
end

% Set longer sim time for interactive use
set_param('spot_physics', 'StopTime', '30');

% Load parameters (creates spotRobot3D for Sim3D block)
setupParams;

% Open keyboard control window
startKeyboard();
fprintf('Keyboard control active. Keep the control window focused!\n');
fprintf('Press Up=forward, Left/Right=turn, A/D=sidestep\n');
fprintf('Starting simulation...\n');

% Run simulation
out = sim('spot_physics');

fprintf('Simulation complete.\n');
