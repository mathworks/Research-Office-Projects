% Path setup for the Simulink reachability plugin solvers.
% Run once per MATLAB session, then call registerSetSolvers to put the
% solvers into the Solver dropdown in Configuration Parameters.
%
%   setup
%   registerSetSolvers
%
% Registering is deliberately NOT done here. It is persistent across MATLAB
% restarts, so it is a once-ever step, and putting a folder on the path should
% not permanently change the Solver dropdown of every model. See
% registerSetSolvers.

thisDir = fileparts(mfilename('fullpath'));

addpath(fullfile(thisDir, 'solvers'));
addpath(fullfile(thisDir, 'representations'));
addpath(fullfile(thisDir, 'visualization'));
addpath(fullfile(thisDir, 'helpers'));
addpath(fullfile(thisDir, 'examples'));
addpath(fullfile(thisDir, 'tests'));

fprintf('Simulink reachability solvers added to path. Run registerSetSolvers to register them.\n');
