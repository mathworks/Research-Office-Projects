function mdl = simulatingModel()
%SIMULATINGMODEL Which model is being simulated right now, from inside a solver hook.
%   mdl = simulatingModel() returns the name of the model whose simulation is
%   currently in progress, or '' if that cannot be determined. Safe to call from
%   start(), step(), reset() and stop().
%
%   WHY NOT bdroot(gcs). Because it is wrong, and wrong silently. gcs is the CURRENT
%   system -- a GUI/last-touched notion -- and has nothing to do with which model the
%   engine is running. Measured: with vdp made current and sldemo_bounce simulating
%   under a set solver, bdroot(gcs) inside start() returns 'vdp' while
%   find_system(...,'SimulationStatus','running') returns 'sldemo_bounce'. One model
%   loaded and everything looks fine, which is why this survived; two models loaded
%   and the answer silently comes from the wrong one.
%
%   That matters in two places, both of which read MODEL CONFIGURATION during a
%   run and would otherwise read another model's:
%
%     SetReach.resolveRadii            the SetIC model-workspace variable, i.e.
%                                      the INITIAL SET itself
%     SetReachVar.resolveStepControl   MaxStep, MinStep, StopTime
%
%   It is also what SetReach.stampModel records into the log, so a figure can name
%   the model it came from; that is a third caller but not a configuration read.
%
%   A wrong initial set is not a cosmetic defect: the whole point of reading SetIC
%   from the model workspace is that the set IC travels with the model, and picking up
%   a different model's SetIC produces a confidently wrong reach tube with no warning.
%
%   HOW IT DECIDES. Any block diagram not in the 'stopped' state counts as in
%   progress; during start() the status is 'running' (measured). Ambiguity is resolved
%   conservatively rather than by picking one:
%
%     exactly one in progress  -> that one, which is the normal case
%     several in progress      -> the one gcs points at IF it is among them (a model
%                                calling sim() on another), else '' -- because
%                                guessing here would reintroduce exactly the silent
%                                wrong-model bug this function exists to remove
%     none in progress         -> bdroot(gcs), which is right when this is called
%                                outside a simulation (a script setting things up)
%
%   SearchDepth 0 keeps this to top-level diagrams, so it does not walk the block
%   hierarchy of every loaded model on every call.
%
%   See also SETREACH, SETREACHVAR, COUNTCONTINUOUSSTATES.
%
mdl = '';
running = {};
try
    open = find_system('SearchDepth', 0, 'type', 'block_diagram');
    for k = 1:numel(open)
        if ~strcmp(get_param(open{k}, 'SimulationStatus'), 'stopped')
            running{end+1} = open{k}; %#ok<AGROW> -- at most a handful of models
        end
    end
catch
    running = {};
end

try
    cur = bdroot(gcs);
catch
    cur = '';       % no current system at all, e.g. nothing loaded
end

if isscalar(running)
    mdl = running{1};
elseif numel(running) > 1
    if ~isempty(cur) && ismember(cur, running)
        mdl = cur;
    end
else
    mdl = cur;
end
end
