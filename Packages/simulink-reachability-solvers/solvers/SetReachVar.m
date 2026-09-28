classdef SetReachVar < Simulink.Solver.VariableStepSolver
    % Set-based post operator as a VARIABLE-STEP plugin solver. Same math as
    % SetReach, same log, same four representations -- the difference is who
    % chooses h, and what happens at a guard.
    %
    % WHY THIS EXISTS. Sets are usually checked against a reference trajectory
    % from ode45, which is variable step while SetReach is fixed step, so the two
    % were never solving the same problem. On sldemo_bounce that showed up as a
    % measurable defect and not just an aesthetic one: a fixed-step run can only
    % apply the restitution at a grid multiple, so the impact time is wrong by
    % O(h), and the deviation from ode45 after the bounce converges at FIRST
    % order (measured 1.234e-01 / 4.350e-02 / 1.612e-02 / 7.665e-03 at
    % h = 0.008 .. 0.001, ratios 2.84 / 2.70 / 2.10) where before the bounce the
    % same run is exact to machine precision. Letting the engine tell us the
    % located crossing time is what removes that.
    %
    % THE CONTRACT IS DIFFERENT IN ONE PLACE THAT MATTERS
    %
    %     fixed     step(obj, t0, x0, h)         -> x1
    %     variable  step(obj, t0, x0, tnextMax)  -> [tnext, xout]
    %
    % We are handed a CEILING and report the time we stopped at. Two measured
    % facts about how the engine uses the two channels it has:
    %
    %   * tnextMax carries events known BEFORE the step -- output times, sample
    %     hits, the stop time. Under OutputOption = 'SpecifiedOutputTimes', 98 of
    %     103 ceilings were a requested output time.
    %   * The ceiling is NEVER narrowed for a zero crossing (0 of 51 steps, with
    %     the crossing 65% of the way into one step). Causality forces this: the
    %     engine sees the sign change only from the completed step's endpoint. So
    %     a crossing arrives AFTER the fact, as the next step's t0, together with
    %     a reset() carrying the post-jump state.
    %
    % Which means the engine DISCARDS steps, and that is the one genuinely new
    % obligation here. It lets our step finish, localises the root, throws our
    % endpoint away and re-drives us from t*. Measured at a nominal h of 0.05 on
    % sldemo_bounce, the discarded step had reached pos = -0.383. So step() below
    % begins by checking whether the step it last took survived, and if not
    % rewinds the log and re-derives the set at t* -- exactly, by re-evaluating
    % the SAME frozen-Jacobian flow at tau = t* - t0 instead of at h. Nothing is
    % approximated there that the freeze had not already approximated.
    %
    % ZERO CROSSINGS ARE STILL NOT EXPOSED, and cannot be asked for. Nothing on
    % any plugin solver base class matches zero|cross|event|guard|bisect|root, and
    % getProperties has no flag for it. We are never told a guard exists. What we
    % DO get is a place in the search: interpolateState supplies the states the
    % engine's bisection evaluates the crossing signal on, and the shipped
    % default there is plain LINEAR. Overriding it with this step's exact affine
    % flow moved the located sldemo_bounce impact by 164us at h = 0.05 (the error
    % is O(h^2), which is why measurements at h = 0.002 all missed it).
    %
    % WHAT A RUN GIVES YOU is a sequence of sets at the times t_k, not a tube.
    % Fixed step at h = 0.002 hides that behind 120 overlapping sets; adaptive
    % steps do not, so the gap between samples is visible here. encloseSetLog
    % closes it, returning the between-step enclosures Omega_k. The linearisation
    % error about the centre is the part no representation bounds; see SetReach
    % and the README.
    %
    % ADAPTIVE STEPPING IS SET-AWARE, which is the point of this class. The
    % matrix measure mu2(A) = lambda_max((A+A')/2) bounds the rate at which
    % the set can grow, so capping relative growth per step at SetTol gives
    % the step size h = log1p(SetTol)/mu. mu is logged by both hierarchies;
    % here it also steers. The cap is a heuristic and not a bound, because
    % mu is evaluated at the centre rather than over the set. See
    % chooseStep.
    %
    % TIME SAMPLING IS NOT THE DISPLAY GRID. Adapting h is an internal accuracy
    % decision, so this class does NOT change what times you report on: run it,
    % then resampleSetLog(L, tq) puts the tube on whatever uniform grid you want,
    % exactly rather than by interpolation. Comparing a run sampled at 2.1307
    % against ground truth sampled at 2.13 is not a comparison.
    %
    % Usage -- identical to SetReach except for SolverType:
    %   registerSetSolvers();
    %   set_param(mdl, 'SolverType', 'Variable-step');
    %   set_param(mdl, 'Solver', 'SetReachVarZonotope');
    %   SetReach.setRadii(0.15);
    %   sim(mdl);
    %   L  = SetReach.getLog();                 % non-uniform t
    %   Lq = resampleSetLog(L, 0:0.002:3);      % uniform, exact
    %
    % See also SETREACH, RESAMPLESETLOG, REGISTERSETSOLVERS.
    %
    properties
        S    = [];   % the active SetRep, valid at the time of the last log entry
        Base = [];   % the step in flight: .t .tEnd .x .S .A .f0 .mu
        Ctl  = [];   % resolved step control: .MaxStep .MinStep .SetTol
    end

    methods
        function k = shapeKind(~)
            %SHAPEKIND Which representation this solver class means.
            %   Empty here: the registered per-shape subclasses override it, which
            %   is what turns the Solver dropdown into the shape picker. The
            %   static configuration (setShape/setRadii/setFit/config) is SHARED
            %   with SetReach rather than duplicated, so a script switches between
            %   fixed and variable step by changing one set_param.
            k = '';
        end

        function start(obj)
            start@Simulink.Solver.VariableStepSolver(obj);
            % One simulation, one log -- see SetReach.start for what accumulating
            % across runs did. It is worse here: rewind() prunes by t <= t0, so a
            % stale log from a previous run is both kept and pruned wrongly.
            SetReach.resetLog();
            SetReach.stampModel();
            SetReach.clearResets();
            obj.Base = [];
            obj.Ctl  = SetReachVar.resolveStepControl();
            n = [];
            try
                n = obj.nx();
            catch
                % nx unavailable in this configuration: step() falls back lazily
            end
            if ~isempty(n)
                obj.S = SetReach.resolveInitialSet(obj.shapeKind(), n);
            end
        end

        function reset(obj, t, x, dx)
            % A continuous state jumped. Unlike the fixed-step case, t is now the
            % TRUE crossing time rather than the grid point after it, so the set
            % does at least arrive at the guard at the right moment -- step()
            % rewinds to exactly here. What has not changed is that the jump MAP
            % is not exposed, and no single map is correct anyway once the set
            % straddles the guard, so obj.S is still not transported through it.
            %
            % As in SetReach.reset, a reset is not necessarily a JUMP: a sampled
            % input kinks xdot at every sample hit without moving any state, and
            % warning there would be a false alarm on an exact answer. Discriminate
            % first; see isStateJump.
            reset@Simulink.Solver.VariableStepSolver(obj, t, x, dx);
            cPrev = SetReach.loggedCentre(t);       % before correctCentre
            SetReach.noteReset(t, x);
            SetReach.correctCentre(t, x);

            if t > 0 && ~isempty(obj.S) && ~SetReach.warnedJump() && ...
                    isStateJump(cPrev, x)
                SetReach.warnedJump(true);
                warning('SetReach:stateJump', ...
                    ['State discontinuity at t = %.12g: the solver was reset. The ' ...
                     'centre and the crossing time are now the engine''s own, but a ' ...
                     '%s still cannot be transported across a jump without its jump ' ...
                     'map, which a plugin solver cannot see. The set after this ' ...
                     'point is not meaningful. (Warned once per simulation; ' ...
                     'SetReach.getResets() lists them all.)'], t, obj.S.kind);
            end
        end

        function [tnext, xout] = step(obj, t0, x0, tnextMax)
            n = numel(x0);
            if isempty(obj.S) || obj.S.dim() ~= n
                obj.S = SetReach.resolveInitialSet(obj.shapeKind(), n);
            end
            if isempty(obj.Ctl)
                obj.Ctl = SetReachVar.resolveStepControl();
            end
            tol = 1e-12 * max(1, abs(t0));

            % ---- 1. did the engine keep the step we last took? -------------
            % If t0 is BEFORE the end of the step in flight, it did not: a zero
            % crossing was localised at t0, strictly inside that step, and the
            % endpoint we already logged is not part of the solution. Drop it and
            % continue the same linearisation as far as t0.
            if ~isempty(obj.Base) && t0 < obj.Base.tEnd - tol
                B = obj.Base;
                SetReach.rewind(t0);
                Phi   = SetReach.dense(B.A, B.f0, t0 - B.t);
                obj.S = B.S.map(Phi);
                % Log x0, not B.x + d: x0 is the engine's clamped, post-jump
                % state and is what it will carry forward. Logging the pre-clamp
                % value instead shows up as the centre penetrating the guard, by
                % 8.5mm of floor on sldemo_bounce.
                SetReach.record(t0, x0(:), obj.S, B.mu, B.A, t0 - B.t, B.f0);
            end

            % ---- 2. this step's linearisation ------------------------------
            A  = obj.Jacobian(t0, x0);
            f0 = obj.forcingFunction(t0, x0);
            mu = max(real(eig((A + A') / 2)));

            % ---- 3. choose h, then honour the ceiling ----------------------
            h     = obj.chooseStep(mu, t0, tnextMax);
            tnext = t0 + h;

            % First entry needs x0, so it cannot be made in start(). Matches
            % SetReach: entry 1 duplicates entry 2's step data.
            if isempty(SetReach.store().t)
                SetReach.record(t0, x0(:), obj.S, mu, A, h, f0);
            end

            % ---- 4. advance, and remember enough to be rewound -------------
            [Phi, d] = SetReach.dense(A, f0, h);
            xout     = x0 + d;
            obj.Base = struct('t', t0, 'tEnd', tnext, 'x', x0(:), 'S', obj.S, ...
                'A', A, 'f0', f0(:), 'mu', mu);
            obj.S    = obj.S.map(Phi);
            SetReach.record(tnext, xout, obj.S, mu, A, h, f0);
        end

        function xi = interpolateState(obj, ti, tL, tR, xL, xR)
            % The engine's zero-crossing bisection evaluates the crossing
            % signal on states it gets from HERE, so this method is the
            % only influence a plugin solver has over where a root is
            % located. The shipped default is plain linear, which
            % mislocates the sldemo_bounce impact by 164us at h = 0.05.
            %
            % SetReach.dense is not a better interpolant of the step, it is
            % not an interpolant of it at all: it is the same expm the step
            % took with h replaced by tau, so tau = h reproduces the step
            % exactly and in between it adds no error the step did not
            % already have. A polynomial through the two endpoints would
            % add a second error on top of that one. The cost is an expm
            % per call, so what this buys is consistency, not speed; what
            % it saves is storage, since A and f0 are already in Base.
            %
            % Only the CENTRE is interpolated. The engine asks for a state
            % and has no notion of a set, so the Phi half of dense is
            % discarded here. The set's own between-step story is Omega_k
            % in encloseSetLog, which is that same discarded half over tau
            % in [0, h].
            %
            % Falls back to the default whenever ti is not inside the step
            % we are holding, which covers the first call of a run and any
            % bracket the engine forms outside it. Never guesses.
            B = obj.Base;
            if ~isempty(B)
                tol = 1e-12 * max(1, abs(B.tEnd));
                if ti >= B.t - tol && ti <= B.tEnd + tol
                    [~, d] = SetReach.dense(B.A, B.f0, ti - B.t);
                    xi = B.x + d;
                    return
                end
            end
            xi = interpolateState@Simulink.Solver.VariableStepSolver( ...
                obj, ti, tL, tR, xL, xR);
        end

        function h = chooseStep(obj, mu, t0, tnextMax)
            %CHOOSESTEP Set-aware step control.
            %   mu2(A) = lambda_max((A+A')/2) is the logarithmic norm, and
            %   ||expm(A*h)|| <= exp(mu*h), so mu bounds the rate at which
            %   the set can grow. Allowing at most SetTol relative growth
            %   per step gives h = log(1+SetTol)/mu. Where mu <= 0 the step
            %   is CONTRACTING and no growth limit applies, so MaxStep
            %   governs -- which is the right answer and not a fallback.
            %
            %   What is under control is the SET, not the centre's
            %   accuracy: the engine's RelTol/AbsTol machinery is not in
            %   play for a plugin solver, since we report tnext rather than
            %   an error estimate.
            %
            %   SetTol is a cap the step aims at, not one it guarantees,
            %   for two reasons. The bound above needs mu over a region and
            %   this is mu at the centre, so a neighbourhood that stretches
            %   faster than its centre grows by more than SetTol. And the
            %   MinStep floor outranks the cap, deliberately: a step too
            %   small to be meaningful is worse than an overshoot.
            %
            %   The ceiling is not negotiable, so it is applied last.
            c = obj.Ctl;
            h = c.MaxStep;
            if mu > 0
                h = min(h, log1p(c.SetTol) / mu);
            end
            h = max(h, c.MinStep);
            room = tnextMax - t0;
            if room > 0
                h = min(h, room);
            end
        end
    end

    methods (Static)
        function props = getProperties()
            props = struct('MassMatrix', true, 'DAE', false, 'Jacobian', true);
        end

        function k = kinds()
            k = SetRep.kinds();
        end

        function out = stepControl(spec)
            %STEPCONTROL Override the step-size rule for this and later runs.
            %   SetReachVar.stepControl(struct('SetTol', 0.05)) allows 5% set
            %   growth per step; stepControl() reads the override back, and
            %   stepControl([]) clears it. Fields MaxStep, MinStep, SetTol, any
            %   subset. A static, because the engine constructs the solver and
            %   there is no constructor to pass anything through.
            persistent C
            if nargin > 0
                C = spec;
            end
            out = C;
        end

        function c = resolveStepControl()
            %RESOLVESTEPCONTROL Merge the model's Solver pane with stepControl().
            %   The model comes first: MaxStep and MinStep are ordinary
            %   variable-step solver settings and a solver that ignores them has
            %   quietly disconnected the config dialog. 'auto' parses to NaN and
            %   is skipped, in which case MaxStep falls back to Simulink's own
            %   documented auto rule, StopTime/50.
            %
            %   simulatingModel(), not bdroot(gcs): with two models loaded, gcs names
            %   the current one rather than the simulating one, and the step control
            %   would then be taken from a model that is not running. See
            %   simulatingModel.
            c = struct('MaxStep', [], 'MinStep', [], 'SetTol', 0.02);
            try
                mdl = simulatingModel();
                if ~isempty(mdl)
                    v = str2double(get_param(mdl, 'MaxStep'));
                    T = str2double(get_param(mdl, 'StopTime'));
                    if isfinite(v) && v > 0
                        c.MaxStep = v;
                    elseif isfinite(T) && T > 0
                        c.MaxStep = T / 50;
                    end
                    v = str2double(get_param(mdl, 'MinStep'));
                    if isfinite(v) && v > 0
                        c.MinStep = v;
                    end
                end
            catch
                % no model context: the defaults below stand
            end
            s = SetReachVar.stepControl();
            if ~isempty(s) && isstruct(s)
                for f = intersect(fieldnames(s)', {'MaxStep', 'MinStep', 'SetTol'})
                    if ~isempty(s.(f{1}))
                        c.(f{1}) = s.(f{1});
                    end
                end
            end
            if isempty(c.MaxStep)
                c.MaxStep = Inf;
            end
            if isempty(c.MinStep)
                % A floor relative to the cap rather than absolute, so it scales
                % with the model's time unit instead of assuming seconds.
                c.MinStep = min(1e-6, c.MaxStep / 1e6);
            end
        end
    end
end
