classdef tSplitSetReach < matlab.unittest.TestCase
%TSPLITSETREACH Tests for splitSetReach, splitting the initial set.
%
%   Run setup once, then:
%       runtests('tSplitSetReach')
%
%   With an exit code, for continuous integration:
%       matlab -batch "run('setup.m'); assertSuccess(runtests('tSplitSetReach'))"
%
%   THE PLANTS ARE BUILT IN CODE, for the reason tSetReach gives: the models the
%   examples use arrive through openExample, which is not usable in a batch
%   session. They are saved into a temporary folder, which is also the working
%   folder, so the Fast Restart cache lands there and nothing is left behind.
%   They use built-in blocks, not library paths, so the Simulink library is
%   never loaded; that alone was 7.6 s, a quarter of the suite.
%
%   WHAT IS CHECKED. Err is a difference between two linearisations, so on an
%   affine plant it must vanish and nothing past the first split may be refined.
%   On a quadratic plant it is an estimate, not a bound, so the leaves are also
%   checked against the truth, a tight ode45 run from their corners, with a
%   tolerance that says so. The rest is what a user hits first: an unfinished
%   run must warn, a piece must start where it was put, a state with no spread
%   must not be split, a discrete state must start every piece where it started
%   the root, and the cases splitting cannot handle must error by name.
%
%   See also SPLITSETREACH, TSETREACH, REGISTERSETSOLVERS.

    properties (Constant)
        % The affine plant, xdot = A*x + b: a spiral sink with a constant drive,
        % so the centre is not an equilibrium and the set map is exactly expm.
        A   = [-0.5 -2; 2 -0.5]
        B   = [0.3; -0.1]

        % Lotka-Volterra, xdot = [x1 - x1*x2; x1*x2 - x2], quadratic in the state
        % and so the simplest plant on which a linearisation has a real error.
        % Both plants start from X0 with spread RAD in each state.
        X0  = [1.5; 1]
        RAD = 0.4
        H   = 0.01
        T   = 2
        TOL = 0.1
    end

    properties
        Lin     % the affine plant
        Quad    % the Lotka-Volterra plant
    end

    methods (TestClassSetup)
        function requirePluginSolvers(tc)
            tc.assumeNotEmpty(which('Simulink.Solver.register'), ...
                'Plugin solvers require R2026b or later.');
        end

        function setUp(tc)
            % One registration for the class, BEFORE any model is built:
            % registering invalidates models already compiled in the session.
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            [~] = registerSetSolvers();
            tc.addTeardown(@() unregister());
            tc.Lin  = tc.affinePlant('tSplit_affine', false);
            tc.Quad = tc.lotkaPlant('tSplit_lotka', tc.X0);
        end
    end

    methods (Test)

        function tAffinePlantHasNoErrorAndDoesNotRefine(tc)
            % Parent and child both propagate the exact expm map, so they agree
            % to rounding. The root is always split once, to have something to
            % compare; nothing below it may be.
            R = tc.verifyWarningFree(@() tc.split(tc.Lin, Tol = 1e-8));
            leaves = R.pieces(R.leaves);
            tc.log(1, sprintf('Worst Err on the affine plant %.3g.', max([leaves.err])));
            tc.verifyLessThan(max([leaves.err]), 1e-12, ...
                sprintf('Err on an affine plant is %.3g, not rounding.', ...
                        max([leaves.err])));
            tc.verifyEqual([leaves.depth], ones(1, 4), ...
                'An affine plant was refined past the first split.');
            tc.verifyEqual({leaves.status}, repmat({'accepted'}, 1, 4));
            tc.verifyTrue(R.resolved);
        end

        function tQuadraticLeavesMeetTolAgainstTheTruth(tc)
            % Every leaf meets Tol by Err, and then against the truth: each
            % leaf's own linear prediction of its four corners, against ode45
            % from those corners at RelTol 1e-12, over the whole run.
            %
            % THE TOLERANCE IS NOT TOL. Err is an estimate that drives
            % refinement, not a per-piece bound; on vdp the true error exceeded
            % a leaf's own Err by up to 1.4 times. So the leaves are held to
            % 1.5*Tol, and the root, unsplit, is checked to be far worse, so the
            % assertion is not met by a plant too tame to need splitting.
            R = tc.verifyWarningFree(@() tc.split(tc.Quad, Tol = tc.TOL, ...
                StepCheck = true));
            tc.verifyTrue(R.resolved);
            leaves = R.pieces(R.leaves);
            tc.verifyEqual({leaves.status}, repmat({'accepted'}, 1, numel(leaves)));
            tc.verifyLessThanOrEqual(max([leaves.err]), tc.TOL);
            tc.verifyGreaterThan(numel(leaves), 4, ...
                'The plant needed no refinement past the first split.');
            tc.verifyLessThan(R.stepErr, tc.TOL / 2);

            leafErr = tc.cornerError(R, R.leaves);
            rootErr = tc.cornerError(R, 1);
            tc.log(1, sprintf(['%d leaves, worst Err %.3g, worst true error %.3g, ' ...
                'unsplit %.3g, step check %.3g.'], numel(leaves), ...
                max([leaves.err]), max(leafErr), rootErr, R.stepErr));
            tc.verifyLessThanOrEqual(max(leafErr), 1.5 * tc.TOL, ...
                sprintf('A leaf misses the truth by %.3g, against Tol = %g.', ...
                        max(leafErr), tc.TOL));
            tc.verifyGreaterThan(rootErr, 4 * max(leafErr), ...
                sprintf('Splitting cut the error only from %.3g to %.3g.', ...
                        rootErr, max(leafErr)));
        end

        function tUnresolvedRunWarns(tc)
            % MaxPieces = 5 is the root and its four children, so a child that
            % fails Tol cannot be split and must be reported.
            R = tc.verifyWarning(@() tc.split(tc.Quad, Tol = tc.TOL, ...
                MaxPieces = 5), 'splitSetReach:unresolved');
            tc.verifyFalse(R.resolved);
            tc.verifyNumElements(R.pieces, 5);
            tc.verifyTrue(any(strcmp({R.pieces(R.leaves).status}, 'unresolved')));
        end

        function tPiecesStartFromTheirOwnCentreAndSpread(tc)
            % The root starts from the model's initial state and Radii. Each
            % piece starts from the box it covers in the root's coordinates,
            % computed here from lo and hi rather than read back, and the
            % leaves tile the initial box exactly.
            R  = tc.split(tc.Quad, Tol = 0.15);
            P  = R.pieces;
            G0 = diag([tc.RAD; tc.RAD]);
            tc.verifyEqual(P(1).c0, tc.X0, 'AbsTol', 1e-12);
            tc.verifyEqual(P(1).G0, G0, 'AbsTol', 1e-15);
            for k = 1:numel(P)
                c = tc.X0 + G0 * (P(k).lo + P(k).hi) / 2;
                G = G0 * diag((P(k).hi - P(k).lo) / 2);
                tc.verifyEqual(P(k).c(:, 1), c, 'AbsTol', 1e-12, ...
                    sprintf('Piece %d did not start from its centre.', k));
                tc.verifyEqual(P(k).G(:, :, 1), G, 'AbsTol', 1e-15, ...
                    sprintf('Piece %d did not start from its spread.', k));
            end
            leaves = P(R.leaves);
            area = arrayfun(@(q) prod(q.hi - q.lo), leaves);
            tc.verifyEqual(sum(area), 4, 'RelTol', 1e-12, ...
                'The leaves do not tile the initial box.');
            tc.verifyGreaterThanOrEqual(min([leaves.lo], [], 'all'), -1);
            tc.verifyLessThanOrEqual(max([leaves.hi], [], 'all'), 1);
        end

        function tStateWithNoSpreadIsNotSplit(tc)
            % A zero generator halved gives two identical children, so only the
            % direction with spread is split: two children per split, not four,
            % and the second coordinate stays the whole interval.
            R = tc.split(tc.Quad, Tol = 0.02, Radii = [tc.RAD; 0]);
            P = R.pieces;
            tc.verifyGreaterThan(numel(P), 3, 'Nothing below the root was split.');
            for p = find(strcmp({P.status}, 'split'))
                tc.verifyNumElements(find([P.parent] == p), 2, ...
                    sprintf('Piece %d has children along a direction with no spread.', p));
            end
            lo = [P.lo];
            hi = [P.hi];
            tc.verifyEqual(lo(2, :), -ones(1, numel(P)));
            tc.verifyEqual(hi(2, :),  ones(1, numel(P)));
        end

        function tDiscreteStateStartsEveryPiece(tc)
            % xdot = z - x^2, with z a Unit Delay decaying from 1. A piece that
            % started with z = 0, the Unit Delay's default, would be off by
            % order 1; matching the truth to the step's own error is the check
            % that the discrete state was carried.
            mdl = tc.discretePlant('tSplit_discrete', 0.5);
            R = tc.split(mdl, Tol = 0.01, Radii = 0.3);
            tc.verifyTrue(R.resolved);
            tc.verifyGreaterThan(numel(R.leaves), 2);

            c0 = [R.pieces.c0];
            truth = tc.reference(@(name) tc.discretePlant(name, c0), R.t);
            c = cell2mat(arrayfun(@(q) q.c', R.pieces, 'UniformOutput', false));
            e = max(abs(c - truth), [], 'all');
            tc.log(1, sprintf('%d pieces, worst centre error against ode45 %.3g.', ...
                numel(R.pieces), e));
            tc.verifyLessThan(e, 1e-3, ...
                sprintf('A piece''s centre is %.3g off ode45.', e));
        end

        function tStateJumpErrors(tc)
            % An Integrator reset at t = 0.5 sends the state back to its initial
            % value: a jump no set can be carried across.
            mdl = tc.affinePlant('tSplit_reset', true);
            tc.verifyError(@() tc.split(mdl, Tol = 0.01), 'splitSetReach:stateJump');
        end

        function tUnsupportedCasesErrorByName(tc)
            tc.verifyError(@() splitSetReach(tc.Lin), 'splitSetReach:noTol');
            tc.verifyError(@() tc.split(tc.Lin, Tol = 0.1, ...
                Solver = 'SetReachEllipsoid'), 'splitSetReach:shape');
            tc.verifyError(@() tc.split(tc.Lin, Tol = 0.1, Solver = 'ode4'), ...
                'splitSetReach:solver');
            tc.verifyError(@() tc.split(tc.Lin, Tol = 0.1, ...
                Solver = 'SetReachVarZonotope'), 'splitSetReach:variableStep');
            % Two directions make four children, and with the root that is five.
            tc.verifyError(@() tc.split(tc.Lin, Tol = 0.1, MaxPieces = 4), ...
                'splitSetReach:tooManyDirections');
        end

    end

    % ------------------------------------------------------------- helpers

    methods (Access = private)

        function R = split(tc, mdl, opts)
            % splitSetReach with this suite's defaults: the plant's radius, a
            % short run, serial, and no step check unless a test asks for it.
            arguments
                tc
                mdl
                opts.Tol
                opts.Radii = tc.RAD
                opts.Solver = 'SetReachZonotope'
                opts.MaxPieces = 2048
                opts.StepCheck = false
            end
            args = namedargs2cell(opts);
            R = splitSetReach(mdl, args{:}, StopTime = tc.T, FixedStep = tc.H, ...
                Parallel = false);
        end

        function e = cornerError(tc, R, idx)
            % For each piece in idx, the largest distance over the run between
            % where its corners truly go and where its own set says they go.
            b = [-1 1 -1 1; -1 -1 1 1];
            x0 = [];
            for p = idx
                x0 = [x0, R.pieces(p).c0 + R.pieces(p).G0 * b]; %#ok<AGROW>
            end
            truth = tc.reference(@(name) tc.lotkaPlant(name, x0), R.t);
            e = zeros(1, numel(idx));
            for i = 1:numel(idx)
                q = R.pieces(idx(i));
                for j = 1:4
                    col = 4 * (i - 1) + j;
                    got = truth(:, [col, size(x0, 2) + col]);
                    want = (q.c + squeeze(pagemtimes(q.G, b(:, j))))';
                    e(i) = max(e(i), max(vecnorm(got - want, 2, 2)));
                end
            end
        end

        function y = reference(tc, build, t)
            % The plant's output at t from a tight ode45 run, one row per time.
            % build(name) returns a plant whose initial state is the points to
            % run, all at once, as a wider state.
            mdl = build('tSplit_reference');
            set_param(mdl, 'SolverType', 'Variable-step', 'Solver', 'ode45', ...
                'RelTol', '1e-12', 'AbsTol', '1e-14', 'StopTime', num2str(tc.T), ...
                'OutputOption', 'SpecifiedOutputTimes', ...
                'OutputTimes', mat2str(t(:)', 17), ...
                'SaveOutput', 'on', 'SaveTime', 'on', 'SaveFormat', 'Array');
            out = sim(mdl);
            close_system(mdl, 0);
            % Deleted, not left to shadow the next reference run's model.
            delete([mdl '.slx']);
            tc.assertEqual(out.tout(:), t(:), 'AbsTol', 1e-12);
            y = out.yout;
        end

        function mdl = affinePlant(tc, name, withReset)
            % xdot = A*x + b. With a reset, a Step at t = 0.5 resets the
            % Integrator to its initial condition.
            mdl = tc.newModel(name);
            add_block('built-in/Integrator', [mdl '/x'], ...
                'InitialCondition', mat2str(tc.X0, 17));
            add_block('built-in/Gain', [mdl '/A'], ...
                'Gain', mat2str(tc.A, 17), 'Multiplication', 'Matrix(K*u)');
            add_block('built-in/Constant', [mdl '/b'], ...
                'Value', mat2str(tc.B, 17));
            add_block('built-in/Sum', [mdl '/xdot'], 'Inputs', '++');
            add_line(mdl, 'x/1', 'A/1');
            add_line(mdl, 'A/1', 'xdot/1');
            add_line(mdl, 'b/1', 'xdot/2');
            add_line(mdl, 'xdot/1', 'x/1');
            if withReset
                set_param([mdl '/x'], 'ExternalReset', 'rising');
                add_block('built-in/Step', [mdl '/reset'], 'Time', '0.5', ...
                    'SampleTime', '0');
                add_line(mdl, 'reset/1', 'x/2');
            end
            tc.finish(mdl, 'x');
        end

        function mdl = lotkaPlant(tc, name, x0)
            % xdot = [x1 - x1*x2; x1*x2 - x2]. Every block works elementwise, so
            % a 2-by-N x0 runs N copies at once, x1 of all of them first.
            N = size(x0, 2);
            mdl = tc.newModel(name);
            add_block('built-in/Integrator', [mdl '/x'], ...
                'InitialCondition', mat2str(reshape(x0', [], 1), 17));
            add_block('built-in/Demux', [mdl '/split'], ...
                'Outputs', mat2str([N N]));
            add_block('built-in/Product', [mdl '/x1x2'], ...
                'Inputs', '2');
            add_block('built-in/Sum', [mdl '/x1dot'], 'Inputs', '+-');
            add_block('built-in/Sum', [mdl '/x2dot'], 'Inputs', '+-');
            add_block('built-in/Mux', [mdl '/xdot'], 'Inputs', '2');
            add_line(mdl, 'x/1',     'split/1');
            add_line(mdl, 'split/1', 'x1x2/1');
            add_line(mdl, 'split/2', 'x1x2/2');
            add_line(mdl, 'split/1', 'x1dot/1');
            add_line(mdl, 'x1x2/1',  'x1dot/2');
            add_line(mdl, 'x1x2/1',  'x2dot/1');
            add_line(mdl, 'split/2', 'x2dot/2');
            add_line(mdl, 'x1dot/1', 'xdot/1');
            add_line(mdl, 'x2dot/1', 'xdot/2');
            add_line(mdl, 'xdot/1',  'x/1');
            tc.finish(mdl, 'x');
        end

        function mdl = discretePlant(tc, name, x0)
            % xdot = z - x.*x, with z(k+1) = 0.8*z(k), z(0) = 1, every 0.1 s.
            % The discrete state drives the continuous one and not the other
            % way, the case the solver propagates exactly.
            mdl = tc.newModel(name);
            add_block('built-in/Integrator', [mdl '/x'], ...
                'InitialCondition', mat2str(x0(:), 17));
            add_block('built-in/UnitDelay', [mdl '/z'], ...
                'InitialCondition', '1', 'SampleTime', '0.1');
            add_block('built-in/Gain', [mdl '/decay'], 'Gain', '0.8');
            add_block('built-in/Product', [mdl '/xsq'], 'Inputs', '2');
            add_block('built-in/Sum', [mdl '/xdot'], 'Inputs', '+-');
            add_line(mdl, 'z/1',     'decay/1');
            add_line(mdl, 'decay/1', 'z/1');
            add_line(mdl, 'x/1',     'xsq/1');
            add_line(mdl, 'x/1',     'xsq/2');
            add_line(mdl, 'z/1',     'xdot/1');
            add_line(mdl, 'xsq/1',   'xdot/2');
            add_line(mdl, 'xdot/1',  'x/1');
            tc.finish(mdl, 'x');
        end

        function mdl = newModel(tc, name)
            if bdIsLoaded(name)
                close_system(name, 0);
            end
            new_system(name);
            load_system(name);
            mdl = name;
            tc.addTeardown(@() closeIfLoaded(name));
        end

        function finish(tc, mdl, state)
            % An output to read the reference runs from, the set solver, an
            % exact Jacobian, and a file, since sim with Fast Restart wants one.
            add_block('built-in/Outport', [mdl '/y']);
            add_line(mdl, [state '/1'], 'y/1');
            set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'SetReachZonotope', ...
                'FixedStep', num2str(tc.H), 'StopTime', num2str(tc.T), ...
                'SolverJacobianMethodControl', 'SparseAnalytical');
            save_system(mdl);
        end

    end
end

% ----------------------------------------------------------- local functions

function unregister()
% Asking for the output is what keeps registerSetSolvers quiet.
[~] = registerSetSolvers('off');
end

function closeIfLoaded(name)
if bdIsLoaded(name)
    close_system(name, 0);
end
end
