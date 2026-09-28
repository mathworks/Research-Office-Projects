%% Reachable tube of the Lorenz system, and where the approximation shows
% Propagates a box of initial conditions through the shipping |Lorenz_system| model
% and draws the resulting tube against trajectories re-simulated from sampled
% initial states.
%
% This is the example to read for the method's limit as well as its best side, since
% one run of it shows both. Over the approach onto the attractor the tube holds the
% sampled trajectories comfortably. Later, once Lorenz has stretched nearby
% trajectories apart exponentially, the linearisation about the centre stops
% describing the set's own extent and some sampled trajectories leave the tube. The
% script counts them instead of leaving the picture to imply they do not exist, and
% the count is deliberately unflattering: most of the trajectories get out somewhere.
%
% Run |setup| once in the session before this script.
%
% See also VDPREACHTUBE, SETREACH, PLOTSETTUBE, SAMPLEMODELTRAJECTORIES.

%% Fetch the model
% Lorenz_system is a shipping example model built from base Simulink blocks, but it
% is distributed with a Global Optimization Toolbox example, so openExample needs
% that toolbox installed to find it. Nothing beyond Simulink is needed to simulate
% it once it is on disk. openExample changes the current folder, so pwd is restored.
mdl = 'Lorenz_system';
if ~bdIsLoaded(mdl)
    % The 'supportingFile' form opens the MODEL. Asking for the example itself opens
    % its script, which needs the MATLAB Editor and so fails under matlab -batch.
    % openExample also leaves the current folder inside the example, including when
    % it throws, so pwd is restored on both paths.
    pwd0 = pwd;
    try
        openExample('globaloptim/OptimizeSimulinkModelInParallelExample', ...
            'supportingFile', mdl);
    catch ME
        cd(pwd0);
        rethrow(ME);
    end
    cd(pwd0);
end

%% Keep the model's own initial condition, and let the transient show
% Left at the model's own [10; 20; 10], which sits well off the attractor. A
% trajectory from there spends its first couple of seconds spiralling in before it
% winds onto one lobe, and that approach is the most informative part of the picture:
% the sampled trajectories stay tightly bundled INSIDE the tube the whole way down,
% and the tube only starts to fail late, once the set has been stretched out along
% the attractor. So the figure shows the method working and the method failing in one
% frame. Starting ON the attractor instead, at say [-8; 8; 27], is the sharper test
% of the sensitivity alone, because contraction onto the attractor no longer masks
% the stretching along it, but over the shorter window it needs it makes a far less
% legible figure.
%
% Nothing is written to the model here, which is worth a note only because the
% obvious way to move the initial condition does not work. The model is
% self-contained: its MODEL WORKSPACE holds Sigma, Rho, Beta and the three initial
% conditions X0, Y0, Z0, the latter as Simulink.Parameter objects. A model workspace
% takes precedence over the base workspace, so assigning X0 at the command line does
% nothing at all and the run silently uses the model's own value, with no error and a
% perfectly plausible answer. Moving it means writing into the model workspace itself
% and restoring it afterwards, the way vdpReachTube.m does with Mu.

%% Register the solvers and configure the run
registerSetSolvers();

% Per-state half-widths, each roughly 2% of how far that state travels over the run.
% A single scalar is the wrong choice on this model: the three states have quite
% different ranges, so one half-width is a large set in one coordinate and a
% negligible one in another.
RADII = [0.6056; 0.7781; 0.7432];
T     = 5.0;
H     = 0.001;

set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'SetReachZonotope', ...
    'FixedStep', num2str(H), 'StopTime', num2str(T));

% Ask for an ANALYTICAL JACOBIAN. Simulink's default is perturbation, which leaves
% about 1e-8 of error in A and so about 1e-6 in the propagated set. The set map is
% Phi = expm(A*h), so it is only as good as A: with an analytical Jacobian the
% propagation is exact to machine precision on linear dynamics, and here it removes
% a source of error that has nothing to do with the approximation being studied.
set_param(mdl, 'SolverJacobianMethodControl', 'SparseAnalytical');

SetReach.setRadii(RADII);

sim(mdl);
L = SetReach.getLog();
fprintf('%d logged sets, t = %g .. %g\n', numel(L.t), L.t(1), L.t(end));

%% How far the set spreads
% The expansion factor is a property of the CENTRE TRAJECTORY, not of how big the
% initial set was: every representation here propagates the same product of
% Phi = expm(A*h) factors, and that product does not depend on the radii. Halving
% RADII halves the set and leaves these ratios unchanged.
%
% Measured on the SET, through the singular values of the generator matrix, and not
% on its interval hull. Lorenz stretches along the flow and contracts hard across
% it, so the set becomes a thin sheet lying at an angle to the state axes, and an
% axis-aligned bounding box of a tilted sheet is nearly as fat as the sheet is long.
% So the hull grows in all three directions at once and describes a roughly
% isotropic blob. It is not that the hull understates the growth; it is that the
% hull cannot express the SHAPE. Compare the two aspect ratios printed below: the
% hull's is a fraction of one, the set's is around 1e-16, which is to say the set has
% collapsed to effectively rank two of three. The flattening is the thing worth
% seeing here, and it is the thing an interval loses.
%
% It is also why the figure over-covers and under-covers at the same time. The tube
% is wide along the direction the set was stretched, which is where the yellow late
% sets reach out further than any sampled trajectory goes, and it is paper-thin
% across that direction, which is where the samples get out.
s0 = svd(L.S{1}.payloadMatrix());
sN = svd(L.S{end}.payloadMatrix());
fprintf('\nprincipal half-extents, start [%s]\n', num2str(s0', '%.4g  '));
fprintf('principal half-extents, end   [%s]\n', num2str(sN', '%.4g  '));
fprintf('stretched by %.1fx along its longest axis, flattened to %.2e of it\n', ...
    sN(1) / s0(1), sN(end) / sN(1));

hw0 = L.S{1}.halfWidths();
hwN = L.S{end}.halfWidths();
fprintf('interval hull, start [%.4g %.4g %.4g] -> end [%.4g %.4g %.4g]\n', hw0, hwN);
fprintf('aspect ratio at the end, the set %.2e against its interval hull %.2f\n', ...
    sN(end) / sN(1), min(hwN) / max(hwN));

%% Ground truth, from the model itself
% Re-simulating the same model from perturbed initial states has no transcription
% step to get wrong, unlike writing the Lorenz equations out again by hand next to
% the plot they are supposed to be checking.
S = sampleModelTrajectories(mdl, L.t, RADII, 40);

nSamples = size(S.X, 3);
everOut  = false(1, nSamples);   % did this trajectory ever leave
nOut     = 0;                    % how many (time, sample) pairs were outside
worst    = 0;                    % largest excursion, relative to the half-width

for k = 1:numel(L.t)
    c  = L.c{k};
    h  = L.S{k}.halfWidths();
    Xk = squeeze(S.X(k, :, :));                        % nx-by-nSamples
    ex = (abs(Xk - c(:)) - h(:)) ./ max(h(:), eps);    % > 0 means outside
    out = any(ex > 1e-9, 1);
    nOut    = nOut + sum(out);
    everOut = everOut | out;
    worst   = max(worst, max(ex(:)));
end

fprintf('\n%d of %d sample-instants lie outside the interval hull of the set\n', ...
    nOut, numel(L.t) * nSamples);
fprintf('%d of %d sampled trajectories leave it at some time\n', ...
    sum(everOut), nSamples);
fprintf('largest excursion, as a fraction of the half-width: %.2f\n', worst);

%% Why, and what would fix it
% The error is the linearisation about the centre, not the shape: as h -> 0 all four
% representations converge to the same variational equation, so refining the step
% does not remove it. Per HSCC'09 the error is quadratic in the diameter of the set,
% so splitting the initial box into smaller pieces and propagating each converges to
% the true reachable set at the cost of more simulations. That is not implemented
% here; see the README.
fprintf('\nRefining H will not fix this: the gap is the linearisation about the\n');
fprintf('centre, and every representation shares it. Splitting the initial set is\n');
fprintf('what converges, and is not implemented.\n');

%% Draw it
% Both projections, because they answer different questions and the package ships
% both. (x, z) is the classic view of the attractor; (x, y) is the cleaner read of
% the approach down from [10; 20; 10] and of the late sets fanning out. Projection is
% exact for every representation here, since rho_{Px}(d) = rho_x(P'd). The state
% vector is ordered [x; y; z] by the model's Integrator, Integrator1, Integrator2.
%
% The grey curves are the sampled trajectories and the parula ramp is the sets,
% coloured by simulation time against the colorbar. The broad pale-blue band is a
% third thing again, the connected enclosure covering the gaps BETWEEN logged sets,
% and it is not time-coded.
for ax = {[1 2], [1 3]}
    plotSetTube(L, 'Samples', S, 'Axes', ax{1}, ...
        'Title', sprintf('Lorenz, zonotope, radii 2%% of travel, h = %g, t = 0..%g', ...
            H, T));
end

%% Leave the shipping model as we found it
% This script leaves the initial conditions alone, but it does change the solver, the
% step size, the stop time and the Jacobian method, and that is a change to a file in
% the user's Examples folder. Clear the dirty flag so the model does not prompt to be
% saved on close.
set_param(mdl, 'Dirty', 'off');
