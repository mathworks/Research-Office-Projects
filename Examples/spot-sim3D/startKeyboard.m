function startKeyboard()
%startKeyboard Open a figure window for keyboard control of the quadruped.
%
%   startKeyboard() creates a MATLAB figure that captures key press/release
%   events and stores them in global variables. These globals are read by
%   readKeyboard() during simulation to drive the robot.
%
%   Controls:
%     Arrow Up/Down   — Forward/backward walking
%     Arrow Left/Right — Turn left/right
%     A/D             — Sidestep left/right
%
%   The figure must remain focused (clicked on) to capture key events.
%   Closing the figure resets all commands to zero.

global g_key_up g_key_down g_key_left g_key_right g_key_a g_key_d
g_key_up = false;
g_key_down = false;
g_key_left = false;
g_key_right = false;
g_key_a = false;
g_key_d = false;

fig = figure('Name', 'Quadruped Keyboard Control', ...
    'NumberTitle', 'off', ...
    'MenuBar', 'none', ...
    'Position', [100 100 400 250], ...
    'KeyPressFcn', @keyDown, ...
    'KeyReleaseFcn', @keyUp, ...
    'CloseRequestFcn', @onClose);

annotation(fig, 'textbox', [0.05 0.05 0.9 0.9], ...
    'String', {'QUADRUPED KEYBOARD CONTROL', '', ...
    'Up = Forward', ...
    'Down = Backward', ...
    'Left = Turn Left', ...
    'Right = Turn Right', ...
    'A = Sidestep Left', ...
    'D = Sidestep Right', ...
    'Up/Down + Left/Right = Steer', ...
    '', 'Keep this window focused!'}, ...
    'FontSize', 12, 'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center');
end

function keyDown(~, event)
global g_key_up g_key_down g_key_left g_key_right g_key_a g_key_d
switch event.Key
    case 'uparrow',    g_key_up = true;
    case 'downarrow',  g_key_down = true;
    case 'leftarrow',  g_key_left = true;
    case 'rightarrow', g_key_right = true;
    case 'a',          g_key_a = true;
    case 'd',          g_key_d = true;
end
end

function keyUp(~, event)
global g_key_up g_key_down g_key_left g_key_right g_key_a g_key_d
switch event.Key
    case 'uparrow',    g_key_up = false;
    case 'downarrow',  g_key_down = false;
    case 'leftarrow',  g_key_left = false;
    case 'rightarrow', g_key_right = false;
    case 'a',          g_key_a = false;
    case 'd',          g_key_d = false;
end
end

function onClose(src, ~)
global g_key_up g_key_down g_key_left g_key_right g_key_a g_key_d
g_key_up = false;
g_key_down = false;
g_key_left = false;
g_key_right = false;
g_key_a = false;
g_key_d = false;
delete(src);
end
