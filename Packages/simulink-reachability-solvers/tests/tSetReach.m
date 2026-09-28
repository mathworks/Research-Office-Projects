classdef tSetReach < matlab.unittest.TestCase
%TSETREACH Tests for the set-valued plugin solvers.
%
%   Run setup once, then:
%       runtests('tSetReach')
%
%   With an exit code, for continuous integration:
%       matlab -batch "run('setup.m'); assertSuccess(runtests('tSetReach'))"
%
%   THE PLANTS ARE BUILT IN CODE. The package ships no models, and the two models
%   its examples use both arrive through openExample, which writes into the user's
%   Examples folder, changes the current folder, and is not usable in a batch
%   session. A test that depended on either would not run unattended.
%   Both plants use core Simulink blocks only, so the suite needs nothing
%   beyond MATLAB and Simulink, which is also all the package itself needs.
%
%   WHAT IS CHECKED, and why these properties and not others. The claim the package
%   rests on is that one step maps the set by Phi = expm(A*h), exactly. For any
%   representation that is equivalent to
%
%       rho_{S(T)}(d) = rho_{S(0)}(Phi' * d)    for every direction d
%
%   so that single support-function identity tests all four shapes without knowing
%   how any of them stores its set. The remaining tests cover what is NOT exact:
%   the Jacobian Simulink supplies, the gap between samples, and the linearisation
%   about the centre.
%
%   See also SETREACH, SETREACHVAR, REGISTERSETSOLVERS, SETREP, ENCLOSESETLOG.

    properties (Constant)
        % A spiral sink. The off-diagonal and diagonal magnitudes differ, which is
        % what makes a perturbation Jacobian visibly inexact on it; a plant like
        % [0 1; -1 0] hides that, because differencing it happens to round exactly.
        A  = [-0.5 -2; 2 -0.5]
        X0 = [1; 0]
        R  = 0.05
        H  = 0.01
        T  = 1.0
        TOL = 1e-12

        % The van der Pol damping, named once. vdpPlant puts it in the Gain
        % block and vdpJacobianError differentiates it by hand, so a single
        % constant is what keeps the plant and its Jacobian the same system.
        MU = 2
    end

    properties
        Names   % the registered solver class names
    end

    methods (TestClassSetup)
        function requirePluginSolvers(tc)
            tc.assumeNotEmpty(which('Simulink.Solver.register'), ...
                'Plugin solvers require R2026b or later.');
        end
    end

    methods (TestMethodSetup)
        function registerFresh(tc)
            % ONE REGISTRATION PER TEST, and each test builds its own model after
            % it. Registering a solver invalidates models already compiled in this
            % session, so a model built before a registration cannot be reused
            % after one.
            tc.Names = registerSetSolvers();
            tc.addTeardown(@() tc.unregister());
        end
    end

    methods (Test)

        function tAllSolversAreSettable(tc)
            % Eight names, registered at once, each independently selectable. This
            % is what lets the Solver dropdown be the set-representation picker.
            tc.verifyNumElements(tc.Names, 2 * numel(SetRep.kinds()));
            mdl = tc.linearPlant();
            for k = 1:numel(tc.Names)
                name = tc.Names{k};
                type = 'Fixed-step';
                if startsWith(name, 'SetReachVar')
                    type = 'Variable-step';
                end
                set_param(mdl, 'SolverType', type, 'Solver', name);
                tc.verifyEqual(get_param(mdl, 'Solver'), name, ...
                    sprintf('Solver did not read back as %s.', name));
            end
        end

        function tSetMapIsExactOnLinearDynamics(tc)
            % rho_{S(T)}(d) = rho_{S(0)}(Phi'd) for every shape and direction.
            Phi = expm(tc.A * tc.T);
            D   = tc.directions(64);
            for kind = SetRep.kinds()
                L = tc.runLinear(tc.solverFor(kind{1}), 'SparseAnalytical');
                tc.verifyEqual(L.c{end}, Phi * tc.X0, 'AbsTol', tc.TOL, ...
                    sprintf('%s: centre is not the true flow.', kind{1}));

                got  = zeros(1, size(D, 2));
                want = zeros(1, size(D, 2));
                for j = 1:size(D, 2)
                    got(j)  = L.S{end}.support(D(:, j));
                    want(j) = L.S{1}.support(Phi' * D(:, j));
                end
                tc.verifyEqual(got, want, 'AbsTol', tc.TOL, ...
                    sprintf('%s: S(T) is not Phi*S(0).', kind{1}));
            end
        end

        function tAnalyticalJacobianIsFarMoreAccurate(tc)
            % Simulink's DEFAULT Jacobian method is perturbation, and the set map is
            % only as good as the A it is handed. The difference is not marginal:
            % about 1e-16 against about 1e-6 on this plant. Asserting it keeps the
            % package honest about needing the analytical setting to claim
            % exactness, rather than quoting a number measured with it and a default
            % that does not deliver it.
            Phi = expm(tc.A * tc.T);

            La = tc.runLinear('SetReachZonotope', 'SparseAnalytical');
            Lp = tc.runLinear('SetReachZonotope', 'SparsePerturbation');

            ea = norm(La.A{end} - tc.A);
            ep = norm(Lp.A{end} - tc.A);
            tc.verifyLessThan(ea, tc.TOL, ...
                'Analytical Jacobian should reproduce A exactly.');
            tc.verifyGreaterThan(ep, 1e-11, ...
                ['Perturbation Jacobian was exact, so this test no longer ' ...
                 'demonstrates the difference it exists to document.']);

            ca = norm(La.c{end} - Phi * tc.X0);
            cp = norm(Lp.c{end} - Phi * tc.X0);
            tc.verifyLessThan(ca, tc.TOL);
            tc.verifyGreaterThan(cp, 1e-9);
        end

        function tVdpPlantJacobianIsAnalytical(tc)
            % THE PREMISE OF TWO TESTS BELOW, asserted instead of assumed.
            % cornerError and tVariableStepTakesNonUniformSteps both ask for
            % SparseAnalytical, and the engine's fallback to differencing is
            % SILENT, so no passing run would ever reveal that the setting
            % had been ignored. It had been: the MATLAB Function block this
            % plant used to be built from supplies no Jacobian method, and
            % both settings returned a bitwise identical A.
            %
            % This is also the guard on vdpPlant's block choice. Putting a
            % Math Function(square) back in place of the Product block fails
            % HERE, and says why, rather than quietly weakening the other
            % two tests into measuring something else.
            ea = tc.vdpJacobianError('SparseAnalytical');
            ep = tc.vdpJacobianError('SparsePerturbation');

            tc.verifyLessThan(ea, 1e-12, ...
                ['SparseAnalytical did not reproduce the hand-derived ' ...
                 'Jacobian, so a block on the state path implements no ' ...
                 'Jacobian method and the engine fell back to differencing.']);
            tc.verifyGreaterThan(ep, 1e-9, ...
                ['Perturbation was exact on this plant, so this test no ' ...
                 'longer demonstrates that the analytical setting does ' ...
                 'anything at all.']);
        end

        function tThreeRepresentationsAgreeAndTheEllipsoidIsInscribed(tc)
            % Zonotope, support and sensitivity carry the same set in different
            % storage, so they must agree in every direction. The ellipsoid is a
            % different SHAPE, inscribed in the initial box by default, so it is
            % contained rather than equal. Testing containment states that
            % difference instead of hiding it behind a loose tolerance.
            D  = tc.directions(64);
            ref = [];
            for kind = {'zonotope', 'support', 'sensitivity'}
                L = tc.runLinear(tc.solverFor(kind{1}), 'SparseAnalytical');
                s = tc.supports(L.S{end}, D);
                if isempty(ref)
                    ref = s;
                else
                    tc.verifyEqual(s, ref, 'RelTol', 1e-12, ...
                        sprintf('%s disagrees with the zonotope.', kind{1}));
                end
            end

            L = tc.runLinear('SetReachEllipsoid', 'SparseAnalytical');
            se = tc.supports(L.S{end}, D);
            tc.verifyLessThanOrEqual(se, ref + tc.TOL, ...
                'The inscribed ellipsoid should be contained in the zonotope.');
            tc.verifyLessThan(min(se ./ ref), 0.999, ...
                'The ellipsoid should be strictly smaller in some direction.');
        end

        function tVariableStepTakesNonUniformSteps(tc)
            % The point of the SetReachVar family: the step is chosen by the error
            % estimate, so the log is not a uniform grid. A test that only checked
            % it ran would pass on a fixed-step solver too.
            %
            % ON VAN DER POL, NOT ON THE LINEAR PLANT, and that is not an arbitrary
            % choice. Under relative error control the linear plant gets a CONSTANT
            % step: its solution decays exponentially, so the local error decays with
            % it and the relative error per step is already constant. A uniform grid
            % there is the solver being right, not being lazy. Adaptivity only has
            % something to do where the timescale actually changes.
            mdl = tc.vdpPlant(tc.X0);
            set_param(mdl, 'SolverType', 'Variable-step', ...
                'Solver', 'SetReachVarZonotope', 'StopTime', '10', ...
                'RelTol', '1e-6', 'AbsTol', '1e-8', ...
                'SolverJacobianMethodControl', 'SparseAnalytical');
            SetReach.setRadii(tc.R);
            sim(mdl);
            L = SetReach.getLog();
            close_system(mdl, 0);

            tc.verifyGreaterThan(numel(L.t), 2);
            dt = diff(L.t);
            tc.verifyGreaterThan(std(dt) / mean(dt), 1e-3, ...
                'Variable-step solver produced a uniform grid.');
        end

        function tEnclosureCoversTheWholeInterval(tc)
            % encloseSetLog claims to cover the CLOSED interval between samples, not
            % just its endpoints. Checked against expm at interior times, which is
            % the true flow on this plant and owes nothing to the code under test.
            L = tc.runLinear('SetReachZonotope', 'SparseAnalytical');
            E = encloseSetLog(L);
            tc.verifyNotEmpty(E.tL);

            worst = 0;
            for i = 1:numel(E.tL)
                for frac = [0 0.25 0.5 0.75 1]
                    t = E.tL(i) + frac * (E.tR(i) - E.tL(i));
                    x = expm(tc.A * t) * tc.X0;
                    worst = max(worst, E.Om{i}.ratio(x - E.c{i}));
                end
            end
            tc.verifyLessThanOrEqual(worst, 1 + 1e-9, ...
                sprintf(['The true centre trajectory left the enclosure ' ...
                         '(worst ratio %.6g).'], worst));
        end

        function tLinearisationErrorIsQuadraticInTheRadius(tc)
            % The error no representation bounds. Propagating a set about the centre
            % is a linearisation, so a corner of the initial box lands away from its
            % linear prediction by O(r^2). That is the HSCC'09 result the README's
            % splitting paragraph rests on, and it is the reason halving the radius
            % is worth four times more than halving the step here.
            %
            % Measured on the corner itself rather than on random samples, so the
            % quantity is deterministic and does not depend on a seed.
            radii = [0.2 0.1 0.05];
            err   = zeros(size(radii));
            for k = 1:numel(radii)
                err(k) = tc.cornerError(radii(k));
            end

            tc.verifyTrue(all(err > 0), ...
                'No linearisation error at all, so the rate is untestable.');
            ratios = err(1:end-1) ./ err(2:end);
            tc.verifyEqual(ratios, repmat(4, 1, numel(ratios)), 'RelTol', 0.35, ...
                sprintf('Expected a factor of 4 per halving, measured [%s].', ...
                        num2str(ratios, '%.2f ')));
        end

    end

    % ------------------------------------------------------------- helpers

    methods (Access = private)

        function unregister(~)
            % Asking for the output is what keeps registerSetSolvers quiet.
            [~] = registerSetSolvers('off');
        end

        function name = solverFor(~, kind)
            name = ['SetReach' upper(kind(1)) kind(2:end)];
        end

        function D = directions(~, n)
            % A fixed set of directions, so a failure is reproducible.
            th = linspace(0, 2*pi, n + 1);
            D  = [cos(th(1:n)); sin(th(1:n))];
        end

        function s = supports(~, S, D)
            s = zeros(1, size(D, 2));
            for j = 1:size(D, 2)
                s(j) = S.support(D(:, j));
            end
        end

        function mdl = linearPlant(tc, x0)
            % xdot = A*x, as two blocks. A vector Integrator and a matrix Gain say
            % the dynamics once, rather than spelling them out one state at a time.
            if nargin < 2
                x0 = tc.X0;
            end
            mdl = tc.newModel('tSetReach_linear');
            add_block('simulink/Continuous/Integrator', [mdl '/x'], ...
                'InitialCondition', mat2str(x0, 17));
            add_block('simulink/Math Operations/Gain', [mdl '/A'], ...
                'Gain', mat2str(tc.A, 17), 'Multiplication', 'Matrix(K*u)');
            add_line(mdl, 'x/1', 'A/1');
            add_line(mdl, 'A/1', 'x/1');
        end

        function mdl = newModel(tc, name)
            if bdIsLoaded(name)
                close_system(name, 0);
            end
            new_system(name);
            load_system(name);
            mdl = name;
            tc.addTeardown(@() close_system(name, 0));
        end

        function L = runLinear(tc, solver, jacobian, solverType)
            if nargin < 4
                solverType = 'Fixed-step';
            end
            mdl = tc.linearPlant();
            set_param(mdl, 'SolverType', solverType, 'Solver', solver, ...
                'StopTime', num2str(tc.T), ...
                'SolverJacobianMethodControl', jacobian);
            if strcmp(solverType, 'Fixed-step')
                set_param(mdl, 'FixedStep', num2str(tc.H));
            else
                set_param(mdl, 'RelTol', '1e-8', 'AbsTol', '1e-10');
            end
            SetReach.setRadii(tc.R);
            sim(mdl);
            L = SetReach.getLog();
            close_system(mdl, 0);
        end

        function e = cornerError(tc, r)
            % Distance between where a corner of the initial box actually goes and
            % where the propagated set says it goes. On LINEAR dynamics this is zero
            % by construction, so the plant here is van der Pol.
            corner = [1; 1];

            % The set, and with it the linear prediction for that corner.
            mdl = tc.vdpPlant(tc.X0);
            set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'SetReachZonotope', ...
                'FixedStep', '0.001', 'StopTime', '1.0', ...
                'SolverJacobianMethodControl', 'SparseAnalytical');
            SetReach.setRadii(r);
            sim(mdl);
            L = SetReach.getLog();
            predicted = L.c{end} + L.S{end}.payloadMatrix() * corner;
            close_system(mdl, 0);

            % Where that corner really goes, from a tight reference solver.
            mdl = tc.vdpPlant(tc.X0 + r * corner);
            set_param(mdl, 'SolverType', 'Variable-step', 'Solver', 'ode45', ...
                'RelTol', '1e-12', 'AbsTol', '1e-14', 'StopTime', '1.0', ...
                'SaveFormat', 'Dataset', 'SaveState', 'on', 'SaveOutput', 'off');
            out = sim(mdl);
            xs  = out.xout{1}.Values.Data;
            actual = xs(end, :)';
            close_system(mdl, 0);

            e = norm(actual - predicted);
        end

        function mdl = vdpPlant(tc, x0)
            % xdot = [x2; mu*(1 - x1^2)*x2 - x1], with mu = 2, built from
            % CORE SIMULINK BLOCKS ONLY. The equation reads more directly
            % as a MATLAB Function block carrying it as text, which is what
            % this was, but setting that block's script needs the Stateflow
            % API, and sfroot was the only thing in the whole package
            % reaching outside MATLAB and Simulink.
            %
            % x1^2 IS A PRODUCT BLOCK, not Math Function(square), and that
            % is load-bearing rather than stylistic. Two tests here ask for
            % SparseAnalytical, and the engine honours it only if EVERY
            % block on the state path implements a Jacobian method;
            % otherwise it falls back to differencing silently. Math
            % Function does not implement one, which is why the setting
            % does nothing on the shipping vdp model. Product does.
            % Measured on this construction, analytical reproduces the
            % hand-derived Jacobian to 9.2e-16 where perturbation sits at
            % 2.0e-06. The MATLAB Function version returned a BITWISE
            % IDENTICAL A for both settings, so the two tests below had
            % been running on a perturbed Jacobian all along while asking
            % for an exact one. tVdpPlantJacobianIsAnalytical is what keeps
            % that from coming back.
            mdl = tc.newModel('tSetReach_vdp');
            add_block('simulink/Continuous/Integrator', [mdl '/x'], ...
                'InitialCondition', mat2str(x0, 17));
            add_block('simulink/Signal Routing/Demux', [mdl '/split'], ...
                'Outputs', '2');
            add_block('simulink/Math Operations/Product', [mdl '/x1sq'], ...
                'Inputs', '2');
            add_block('simulink/Sources/Constant', [mdl '/one'], 'Value', '1');
            add_block('simulink/Math Operations/Sum', [mdl '/damp'], ...
                'Inputs', '+-');
            add_block('simulink/Math Operations/Product', [mdl '/scale'], ...
                'Inputs', '2');
            add_block('simulink/Math Operations/Gain', [mdl '/mu'], ...
                'Gain', mat2str(tc.MU));
            add_block('simulink/Math Operations/Sum', [mdl '/x2dot'], ...
                'Inputs', '+-');
            add_block('simulink/Signal Routing/Mux', [mdl '/xdot'], ...
                'Inputs', '2');

            % x1 fans out three ways and x2 twice, so it is the branching
            % that makes this read as the equation rather than as a pile of
            % blocks:
            %     damp  = 1 - x1*x1
            %     x2dot = mu*(damp*x2) - x1
            %     xdot  = [x2; x2dot]
            add_line(mdl, 'x/1',     'split/1');
            add_line(mdl, 'split/1', 'x1sq/1');
            add_line(mdl, 'split/1', 'x1sq/2');
            add_line(mdl, 'one/1',   'damp/1');
            add_line(mdl, 'x1sq/1',  'damp/2');
            add_line(mdl, 'damp/1',  'scale/1');
            add_line(mdl, 'split/2', 'scale/2');
            add_line(mdl, 'scale/1', 'mu/1');
            add_line(mdl, 'mu/1',    'x2dot/1');
            add_line(mdl, 'split/1', 'x2dot/2');
            add_line(mdl, 'x2dot/1', 'xdot/2');
            add_line(mdl, 'split/2', 'xdot/1');
            add_line(mdl, 'xdot/1',  'x/1');
        end

        function e = vdpJacobianError(tc, jacobian)
            % Largest deviation of the engine's A from the hand-derived
            % Jacobian of xdot = [x2; mu*(1-x1^2)*x2 - x1] along the centre.
            %
            % L.A{k} is the Jacobian the step INTO t_k used, so it was
            % evaluated at the centre at t_{k-1}. Comparing it against the
            % centre at t_k instead reports about 4e-03 on every setting
            % alike, which is the Jacobian moving across one step rather
            % than any setting being ignored, and it masks the real signal
            % completely.
            mdl = tc.vdpPlant(tc.X0);
            set_param(mdl, 'SolverType', 'Fixed-step', ...
                'Solver', 'SetReachZonotope', 'FixedStep', num2str(tc.H), ...
                'StopTime', num2str(tc.T), ...
                'SolverJacobianMethodControl', jacobian);
            SetReach.setRadii(tc.R);
            sim(mdl);
            L = SetReach.getLog();
            close_system(mdl, 0);

            e = 0;
            for k = 2:numel(L.t)
                x   = L.c{k-1};
                Aex = [0,                            1; ...
                       -2*tc.MU*x(1)*x(2) - 1,       tc.MU*(1 - x(1)^2)];
                e   = max(e, norm(L.A{k} - Aex));
            end
        end

    end
end
