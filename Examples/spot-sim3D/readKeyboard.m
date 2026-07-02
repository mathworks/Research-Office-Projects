function [forward_cmd, yaw_cmd, lateral_cmd] = readKeyboard()
%readKeyboard Read current keyboard state and return normalized commands.
%
%   [forward_cmd, yaw_cmd, lateral_cmd] = readKeyboard()
%
%   Reads global key-state variables set by startKeyboard() and returns:
%     forward_cmd  — +1 (forward), -1 (backward), or 0 (idle)
%     yaw_cmd      — -1 (turn left), +1 (turn right), or 0
%     lateral_cmd  — +1 (sidestep left), -1 (sidestep right), or 0
%
%   Called at each simulation timestep by the GaitPlanner Stateflow chart
%   via coder.extrinsic.
global g_key_up g_key_down g_key_left g_key_right g_key_a g_key_d
forward_cmd = 0.0;
yaw_cmd = 0.0;
lateral_cmd = 0.0;
if isempty(g_key_up)
    return
end
if g_key_up && ~g_key_down
    forward_cmd = 1.0;
elseif g_key_down && ~g_key_up
    forward_cmd = -1.0;
end
if g_key_left && ~g_key_right
    yaw_cmd = -1.0;
elseif g_key_right && ~g_key_left
    yaw_cmd = 1.0;
end
if ~isempty(g_key_a) && g_key_a && ~g_key_d
    lateral_cmd = 1.0;
elseif ~isempty(g_key_d) && g_key_d && ~g_key_a
    lateral_cmd = -1.0;
end
end
