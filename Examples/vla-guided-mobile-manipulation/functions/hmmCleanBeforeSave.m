function hmmCleanBeforeSave(mdl)
%HMMCLEANBEFORESAVE Put back what a run retunes, before the model is written (its PreSaveFcn).
%
%   HMMCLEANBEFORESAVE(mdl) sets every Simulation 3D Robot block's eeTargetTrans back to the
%   expression carryTranslation when it holds anything else, and says so.
%
%   WHY. The app retunes eeTargetTrans to a LITERAL while a run is going (onSend, one value per
%   object named) and restores the expression afterwards - so a model saved in between keeps a
%   run's value, which then overrides setupParams' carryTranslation for every later run that is
%   not driven through the app. It happened twice: once, undone on 2026-10-15, and again in
%   4319cbd, whose chart fix was saved after a run with [0.00640896 -0.0129259 0.00760232] in
%   memory.
%
%   See also HomeMobileManipulatorApp, hmmCarryTranslation, setupParams.

%   Copyright 2026 The MathWorks, Inc.

blks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
    'MaskType', 'Simulation 3D Robot');
for k = 1:numel(blks)
    v = get_param(blks{k}, 'eeTargetTrans');
    if ~strcmp(strtrim(v), 'carryTranslation')
        set_param(blks{k}, 'eeTargetTrans', 'carryTranslation');
        fprintf('hmmCleanBeforeSave: %s eeTargetTrans %s -> carryTranslation\n', blks{k}, v);
    end
end
end
