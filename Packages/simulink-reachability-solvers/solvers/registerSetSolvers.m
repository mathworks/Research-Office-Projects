function varargout = registerSetSolvers(action)
%REGISTERSETSOLVERS Put every set representation into the Simulink Solver dropdown.
%
%   registerSetSolvers() registers two plugin solver classes per set
%   representation, a fixed-step and a variable-step flavour, and prints what is
%   now selectable, paired by shape. After it runs, both the shape and the step
%   discipline are ordinary Solver choices:
%
%       registerSetSolvers();
%       set_param(mdl, 'SolverType', 'Fixed-step');
%       set_param(mdl, 'Solver', 'SetReachEllipsoid');      % or ...Zonotope, etc.
%
%       set_param(mdl, 'SolverType', 'Variable-step');
%       set_param(mdl, 'Solver', 'SetReachVarEllipsoid');   % same, adaptive h
%
%   registerSetSolvers('off') unregisters them again.
%
%   Registration is PERSISTENT, not per-session: the names stay in the Solver
%   dropdown of every model across MATLAB restarts, so this is a once-ever step
%   and 'off' is the only way to take them back out. That is why setup.m sets the
%   path and leaves registering to the caller, rather than doing it for them:
%   putting a folder on the path should not permanently change a dialog.
%
%   names = registerSetSolvers() returns the class names instead of printing, for
%   a caller that wants to loop over them. Asking for the output is what turns the
%   printing off.
%
%   Several plugin solver classes can be registered at once and each is
%   independently settable, with get_param reading back what was set, so the
%   Solver dropdown itself is what selects the set representation.
%
%   The list of shapes lives in SetRep.kinds() rather than being repeated here, so
%   adding a representation means adding a rep class and two two-line subclasses.
%
%   Requires MATLAB and Simulink R2026b or later, where plugin solvers and the
%   Simulink.Solver.register interface were introduced.
%
%   See also SETREACH, SETREACHVAR, SETREP, PLOTSETTUBE.

if nargin < 1
    action = 'on';
end

% Two solver classes per shape, a fixed-step and a variable-step flavour. Both
% share the static configuration and the static log on SetReach, so switching
% between them is one set_param and nothing else needs re-saying.
kinds = SetRep.kinds();
names = cell(1, 2 * numel(kinds));
for k = 1:numel(kinds)
    Shape = [upper(kinds{k}(1)), kinds{k}(2:end)];
    names{k}                = ['SetReach',    Shape];
    names{numel(kinds) + k} = ['SetReachVar', Shape];
end

if strcmpi(action, 'off')
    for k = 1:numel(names)
        try Simulink.Solver.unregister(names{k}); catch, end
    end
    if nargout > 0
        varargout{1} = names;
    else
        fprintf('%d set solvers removed from the Simulink Solver dropdown.\n', ...
            numel(names));
    end
    return
end

for k = 1:numel(names)
    if isempty(which(names{k}))
        error('registerSetSolvers:missing', ...
            ['Solver class %s is not on the path. Run setup.m in the package ' ...
             'root first.'], names{k});
    end
    % Re-registering the same name errors, so clear it first. A name that was
    % never registered makes the unregister throw, which is harmless here.
    try Simulink.Solver.unregister(names{k}); catch, end
    try
        Simulink.Solver.register(names{k});
    catch ME
        error('registerSetSolvers:unavailable', ...
            ['Could not register the plugin solver %s. Plugin solvers ' ...
             'require R2026b or later.\nUnderlying error: %s'], ...
            names{k}, ME.message);
    end
end

% Return the list or describe it, never both. The caller asking for names is a
% script that wants to loop; the caller asking for nothing is a person at the
% prompt, for whom the structure that matters is that each shape comes in two
% step disciplines.
if nargout > 0
    varargout{1} = names;
else
    describeRegistration(kinds, names);
end
end

% ------------------------------------------------------------------- helpers

function describeRegistration(kinds, names)
%DESCRIBEREGISTRATION Print the dropdown as a shape-by-step-discipline table.
nk    = numel(kinds);
fixed = names(1:nk);
vars  = names(nk + 1 : 2 * nk);
w     = max(cellfun(@numel, names)) + 2;

fprintf('\n%d set solvers are now in the Simulink Solver dropdown.\n\n', numel(names));
fprintf('  %-*s %-*s %s\n', w, 'Fixed-step', w, 'Variable-step', 'shape');
fprintf('  %-*s %-*s %s\n', w, repmat('-', 1, w - 2), w, ...
    repmat('-', 1, w - 2), repmat('-', 1, 11));
for k = 1:nk
    fprintf('  %-*s %-*s %s\n', w, fixed{k}, w, vars{k}, kinds{k});
end

fprintf(['\nPick one the way you would pick ode45:\n' ...
         '  set_param(mdl, ''SolverType'', ''Fixed-step'', ''Solver'', ''%s'')\n' ...
         'then sim(mdl) and plotSetTube. Use ''Variable-step'' with a SetReachVar\n' ...
         'name for the adaptive flavour, and registerSetSolvers(''off'') to remove\n' ...
         'them all again.\n\n'], fixed{min(2, nk)});
end
