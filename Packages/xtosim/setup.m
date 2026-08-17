% xToSim setup — adds all subdirectories to MATLAB path
% Run once per session: run(fullfile(SKILL_DIR, 'setup.m'))

skillDir = fileparts(mfilename('fullpath'));

addpath(skillDir);
addpath(fullfile(skillDir, 'builders'));
addpath(fullfile(skillDir, 'compose'));
addpath(fullfile(skillDir, 'validate'));
addpath(fullfile(skillDir, 'package'));
addpath(fullfile(skillDir, 'util'));

fprintf('xToSim: paths added (%s)\n', skillDir);
