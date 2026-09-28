%% Reachable tube of the van der Pol oscillator, in four representations
% Runs the shipping |vdp| model under each of the four fixed-step set solvers and
% compares them. The point is that the choice of representation is a choice about
% what you can read off cheaply, not about what set you get: three of the four
% return the same set to machine precision, and the ellipsoid differs only because
% an inscribed ellipsoid is a different shape from a box.
%
% Run |setup| once in the session before this script.
%
% See also LORENZREACHTUBE, SETREACH, PLOTSETTUBE.

%% Register the solvers
% Eight names appear in Configuration Parameters -> Solver, four fixed-step and
% four variable-step. They coexist, so the dropdown itself picks the shape.
registerSetSolvers();

mdl   = 'vdp';
RADII = 0.1;      % half-width of the initial box about the model's own x0
T     = 10;       % one full trip around the limit cycle and then some
H     = 0.005;

% vdp is a shipping example model. Typing its name runs a stub that fetches it and
% opens it, which also changes the current folder, so pwd is restored on both paths.
% Simulink will warn that the model file is shadowed by that stub; that is how the
% model is distributed, not a problem with this script.
if ~bdIsLoaded(mdl)
    pwd0 = pwd;
    try
        vdp;
    catch ME
        cd(pwd0);
        rethrow(ME);
    end
    cd(pwd0);
end

% mu = 2 makes the limit cycle sharp enough that the set's collapse onto it is
% visible rather than a subtlety. It is set explicitly so the numbers below do not
% depend on what the local copy of the model happens to hold, and put back at the
% end so the shipping model is left as it was found.
muWas = get_param([mdl '/Mu'], 'Gain');
set_param([mdl '/Mu'], 'Gain', '2');

%% Every representation, same model, same initial set
shapes = SetRep.kinds();
hw     = zeros(numel(shapes), 2);
logs   = cell(numel(shapes), 1);

for k = 1:numel(shapes)
    solver = ['SetReach' upper(shapes{k}(1)) shapes{k}(2:end)];
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', solver, ...
        'FixedStep', num2str(H), 'StopTime', num2str(T));

    % Ask for an ANALYTICAL JACOBIAN. Simulink's default is perturbation, which
    % leaves about 1e-8 of error in A and so about 1e-6 in the propagated set. The
    % set map is Phi = expm(A*h), so it is only as good as A. Without this the four
    % representations would still agree, but they would agree on a slightly wrong
    % answer, and the agreement measured below would be limited by A rather than by
    % the geometry it is meant to be testing.
    %
    % ON THIS MODEL THE SETTING IS INERT, and the line stays anyway. An analytical
    % Jacobian needs every block on the state path to implement one, and vdp has a
    % block that does not, so the engine falls back to differencing SILENTLY and A is
    % perturbed regardless of what is asked for. The intent is right and the setting
    % costs nothing, so it is kept rather than deleted, but it should not be read as
    % fixing anything here. It does work on lorenzReachTube.m.
    set_param(mdl, 'SolverJacobianMethodControl', 'SparseAnalytical');

    SetReach.setRadii(RADII);
    sim(mdl);

    logs{k}  = SetReach.getLog();
    hw(k, :) = logs{k}.S{end}.halfWidths();
    fprintf('%-12s %-22s final half-widths [%.6g %.6g]\n', ...
        shapes{k}, solver, hw(k, 1), hw(k, 2));
end

%% How closely do they agree
% Measured against the zonotope. The ellipsoid is expected to differ: EllipsoidRep
% inscribes the initial box by default, so it starts smaller and stays a different
% shape. Use SetReach.setFit('circumscribed') to bound the box instead.
fprintf('\nagreement of the final interval hull, relative to the zonotope\n');
for k = 1:numel(shapes)
    rel = norm(hw(k, :) - hw(1, :)) / norm(hw(1, :));
    fprintf('  %-12s %.3e\n', shapes{k}, rel);
end

%% The set flattens onto the limit cycle, and the flattening is the failure
% Van der Pol contracts transversally to its limit cycle, so the reachable set
% becomes a sliver aligned with the flow. That much is real. What is NOT real is how
% far it goes: the linearised tube contracts onto the cycle harder than the true
% reachable set does, so the sliver is thinner than the truth and the samples counted
% below get out of it. A collapse this clean is easy to read as the method nailing the
% answer, and on this model it is the opposite.
%
% Measure it on the SET, not on its
% interval hull: the hull is axis aligned, so a sliver lying at an angle to the axes
% still has a fat bounding box, and the hull's aspect ratio badly understates the
% flattening. The singular values of the generator matrix are the set's own
% principal half-extents, which is what the contraction acts on.
Z  = logs{1};
s0 = svd(Z.S{1}.payloadMatrix());
sN = svd(Z.S{end}.payloadMatrix());
fprintf('\nzonotope principal half-extents, start [%.4g %.4g] -> end [%.4g %.4g]\n', ...
    s0(1), s0(end), sN(1), sN(end));
fprintf('aspect ratio of the set,   start %.3g -> end %.3g\n', ...
    s0(end)/s0(1), sN(end)/sN(1));

h0 = Z.S{1}.halfWidths();
hN = Z.S{end}.halfWidths();
fprintf('aspect ratio of its hull,  start %.3g -> end %.3g   (axis aligned, so looser)\n', ...
    min(h0)/max(h0), min(hN)/max(hN));

%% Draw it against sampled trajectories
% The samples are the honest part of the picture: they come from re-simulating the
% model itself from perturbed initial states, so nothing about the plant is
% transcribed by hand. On nonlinear dynamics samples can leave the tube, and a tube
% drawn alone looks more authoritative than it has earned.
S = sampleModelTrajectories(mdl, Z.t, RADII, 40);

%% Count the escapes, do not just draw them
% Same test as lorenzReachTube.m, so the two examples report containment the same way.
% Both the RELATIVE and the ABSOLUTE excursion are printed, because on this model the
% relative one is misleading on its own: it divides by a half-width that has collapsed
% to order 1e-04, so it reads in the hundreds while describing a gap of a couple of
% state units. The absolute figure is the one to quote.
nSamples = size(S.X, 3);
everOut  = false(1, nSamples);   % did this trajectory ever leave
nOut     = 0;                    % how many (time, sample) pairs were outside
worstRel = 0;                    % largest excursion, relative to the half-width
worstAbs = 0;                    % largest excursion, in state units
hwMin    = Inf;                  % thinnest the tube ever claims to be

for k = 1:numel(Z.t)
    c  = Z.c{k};
    h  = Z.S{k}.halfWidths();
    Xk = squeeze(S.X(k, :, :));                        % nx-by-nSamples
    ab = abs(Xk - c(:)) - h(:);                        % > 0 means outside
    ex = ab ./ max(h(:), eps);
    out = any(ex > 1e-9, 1);
    nOut     = nOut + sum(out);
    everOut  = everOut | out;
    worstRel = max(worstRel, max(ex(:)));
    worstAbs = max(worstAbs, max(ab(:)));
    hwMin    = min(hwMin, min(h));
end

fprintf('\n%d of %d sample-instants lie outside the interval hull of the set\n', ...
    nOut, numel(Z.t) * nSamples);
fprintf('%d of %d sampled trajectories leave it at some time\n', ...
    sum(everOut), nSamples);
fprintf('largest excursion: %.3g state units, %.1fx the half-width there\n', ...
    worstAbs, worstRel);
fprintf('thinnest half-width the tube ever reports: %.3g\n', hwMin);

plotSetTube(Z, 'Samples', S, 'Axes', [1 2], ...
    'Title', sprintf('van der Pol, mu = 2, zonotope, radii = %g', RADII));

%% Leave the shipping model as it was found
% Clearing the dirty flag as well, so closing the model does not offer to save a
% change that has already been undone.
set_param([mdl '/Mu'], 'Gain', muWas);
set_param(mdl, 'Dirty', 'off');
