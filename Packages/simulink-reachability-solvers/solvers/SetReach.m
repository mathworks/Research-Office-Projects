classdef SetReach < Simulink.Solver.FixedStepSolver
    % Set-based post operator as a fixed-step plugin solver, with the SET
    % REPRESENTATION swappable.
    %
    % Instead of integrating one trajectory, the solver propagates a SET of
    % initial conditions and logs the resulting sequence of sets.
    %
    % ONE STEP, FOUR SHAPES. Everything a plugin solver contributes -- a place to
    % carry state across steps, and the exact linear map Phi = expm(A*h) -- is
    % representation agnostic, so every representation closed under linear maps
    % drops straight in. This is the base class. Shape-specific code   
    % lives in one SetRep subclass each:
    %
    %   'zonotope'      ZonotopeRep      G <- Phi*G
    %   'ellipsoid'     EllipsoidRep     Q <- Phi*Q*Phi'
    %   'support'       SupportRep       Phi <- Phi_h*Phi         (LGG / SpaceEx)
    %   'sensitivity'   SensitivityRep   S <- Phi*S               (HSCC'09)
    %
    % THREE WAYS TO PICK ONE, in decreasing order of convenience:
    %
    %   1. THE SIMULINK SOLVER DROPDOWN. registerSetSolvers() registers a thin
    %      subclass per shape, so the shape becomes a Configuration Parameters ->
    %      Solver choice like any other solver, or using command-line apis:
    %
    %          registerSetSolvers();
    %          set_param(mdl, 'SolverType', 'Fixed-step');
    %          set_param(mdl, 'Solver', 'SetReachEllipsoid');
    %
    %      Several plugin solvers can be registered at once and each is
    %      independently settable, so this genuinely is a dropdown.
    %
    %   2. setShape / setRadii on this class, when using 'SetReach' itself.
    %   3. config(), which takes a fully built rep or a struct payload.
    %
    % WHERE THE INITIAL SET COMES FROM. Only the spread needs a mechanism:
    % the CENTRE is the model's own Integrator initial conditions, which
    % the engine hands to step() as x0. The SPREAD is read from the model
    % workspace variable SetIC, else from setRadii(). The spec is
    % shape-agnostic -- a scalar half-width, a per-state vector, or a full
    % generator matrix -- so switching the dropdown never silently changes
    % what the initial condition MEANS. The one exception is deliberate: an
    % ellipsoid must choose whether to inscribe or circumscribe the box.
    % See setFit() and EllipsoidRep.
    %
    % start() ESTABLISHES THE SET, reset() REPORTS A JUMP. 
    %
    % WHAT DOES *NOT* CHANGE WITH REPRESENTATION
    %   The approximation on nonlinear dynamics. It is not a shape problem: as
    %   h->0 every one of these converges to the same variational equation
    %   Sdot = A(t,c(t))*S linearised about the centre trajectory, so all four
    %   wrap the same object. On a LINEAR model the propagation is exact, because
    %   expm(A*h) is. The linearisation error is quadratic in the diameter of the
    %   set, so splitting the initial set into smaller pieces converges; splitting
    %   is not implemented here. See the README.
    %
    %   Three of the four are the SAME SET, differing only in what they answer
    %   cheaply and how they draw. 
    %
    % Configure before simulating:
    %   SetReach.setShape('ellipsoid'); SetReach.setRadii(0.15);
    %
    % Then, after sim():
    %   plotSetTube;                    % that is the whole call
    %   L = SetReach.getLog();          % .t .c .S .mu .A .h .f0 .mdl
    %
    % The log will NOT appear in your workspace. It is a persistent store inside
    % SetReach.store, shared by every subclass here and in SetReachVar, because
    % the engine constructs the solver: there is no object to hold. getLog() is
    % the only way in.
    %
    % See also SETREACHVAR, SETREP, PLOTSETTUBE, REGISTERSETSOLVERS.

    properties
        S = [];        % the active SetRep. Named S because the log field is .S
    end

    methods
        function k = shapeKind(~)
            %SHAPEKIND Which representation this solver class means.
            %   Empty on this base class. The
            %   registered per-shape subclasses override this with a constant,
            %   which is what turns the Solver dropdown into the shape picker.
            k = '';
        end

        function start(obj)
            % Establish the initial set. Fires once per simulation, is passed no
            % state, and needs none: obj.nx() gives the width and the model
            % workspace gives the spread.
            start@Simulink.Solver.FixedStepSolver(obj);
            % One simulation, one log. Clearing here rather than leaving it to
            % the caller is what makes hitting Run twice give one run's worth of
            % log instead of two appended together.
            SetReach.resetLog();
            SetReach.stampModel();
            SetReach.clearResets();
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
            % The engine's integration history is void. That is NOT the same as "a
            % continuous state jumped": reset also fires on a DERIVATIVE
            % discontinuity across a perfectly continuous state, which is what a
            % sampled input does at every sample hit. Those two cases need
            % opposite handling -- a jump invalidates the set, a kink in xdot does
            % not -- so discriminate before warning. See isStateJump.
            %
            % Either way obj.S is left alone: no jump map is exposed, and no single
            % map is correct anyway once the set straddles the guard.
            reset@Simulink.Solver.FixedStepSolver(obj, t, x, dx);
            % BEFORE correctCentre, which overwrites exactly the value being
            % compared against.
            cPrev = SetReach.loggedCentre(t);
            SetReach.noteReset(t, x);
            SetReach.correctCentre(t, x);

            if t > 0 && ~isempty(obj.S) && ~SetReach.warnedJump() && ...
                    isStateJump(cPrev, x)
                SetReach.warnedJump(true);
                warning('SetReach:stateJump', ...
                    ['State discontinuity at t = %g: the solver was reset, but a ' ...
                     '%s cannot be transported across a jump without its jump map, ' ...
                     'which a plugin solver cannot see. The set after this point is ' ...
                     'not meaningful. (Warned once per simulation; ' ...
                     'SetReach.getResets() lists them all.)'], t, obj.S.kind);
            end
        end

        function x1 = step(obj, t0, x0, h)
            n = numel(x0);
            % WHERE A COMES FROM, AND HOW GOOD IT IS. The engine supplies it, not
            % this code, and the set map is Phi = expm(A*h), so a propagated set is
            % only ever as accurate as A. SolverJacobianMethodControl defaults to
            % 'auto', and 'auto' together with both perturbation values gives a
            % DIFFERENCED A, good to around 1e-8 and so around 1e-6 in the set;
            % refining h does not help, because the error is not in the integration.
            % An Analytical value can make the map exact to machine precision, but
            % only if every block on the state path implements a Jacobian method, and
            % the fallback to differencing is SILENT. That is why the setting works on
            % Lorenz_system and does nothing at all on vdp.
            %
            % No warning is raised about this, deliberately. The line worth holding:
            % warn when a PRECONDITION of the method is broken, not about how
            % approximate the approximation is. SetReach:stateJump is the first kind,
            % a jump invalidates the tube outright and the user must know.  The
            % Jacobian method is the second kind, a small term inside something the
            % README already labels approximate, and on the shipping examples it is
            % orders of magnitude below the linearisation error that dominates. See
            % the README's accuracy discussion.
            A  = obj.Jacobian(t0, x0);
            f0 = obj.forcingFunction(t0, x0);

            % Matrix measure (logarithmic norm) of the Jacobian in the
            % 2-norm: mu2(A) = lambda_max((A + A')/2), one eigenvalue of
            % the symmetric part. Sontag's bound, stated as Proposition 4.2
            % in Fan's thesis: if mu(J(x)) <= c on a region S, and the
            % trajectories from the segment between x1 and x2 stay in S up
            % to T, then ||xi(x1,t) - xi(x2,t)|| <= ||x1-x2||*exp(c*t) for
            % t <= T. Unlike a Lipschitz constant it can be NEGATIVE, so
            % contracting stretches shrink the bound instead of inflating
            % it. Logged every step; the fixed-step solvers do not
            % otherwise use it, while SetReachVar steers the step size on
            % it (chooseStep). Either way this is an evaluation at the
            % centre, not the region bound the proposition asks for.
            mu = max(real(eig((A + A') / 2)));

            % Fallback only: start() normally builds the set. The initial log
            % entry does belong here, because it needs x0.
            if isempty(obj.S) || obj.S.dim() ~= n
                obj.S = SetReach.resolveInitialSet(obj.shapeKind(), n);
            end
            if isempty(SetReach.store().t)
                SetReach.record(t0, x0(:), obj.S, mu, A, h, f0);
            end

            % One expm: exact affine flow for the centre, exact linear map for
            % the set. Identical for every representation -- the shape's only
            % contribution is the map() on the next line.
            E   = expm([A, f0; zeros(1, n + 1)] * h);
            Phi = E(1:n, 1:n);
            x1  = x0 + E(1:n, n + 1);

            obj.S = obj.S.map(Phi);
            SetReach.record(t0 + h, x1, obj.S, mu, A, h, f0);
        end
    end

    methods (Static)
        function props = getProperties()
            props = struct('MassMatrix', true, 'DAE', false, 'Jacobian', true);
        end

        function k = kinds()
            %KINDS The dropdown's contents, defined once in SetRep.
            k = SetRep.kinds();
        end

        function rep = resolveInitialSet(kind, n)
            % Build the initial representation. Precedence, and it matters:
            %
            %   1. a rep or struct in config() WHOSE KIND MATCHES the class -- an
            %      explicitly built payload wins, since the caller has said
            %      exactly what they want
            %   2. the class's own shape (a registered subclass) or setShape(),
            %      with the spread from the model workspace or setRadii()
            %
            % A shape-specific subclass never silently propagates a different
            % shape: if config() holds an ellipsoid and the solver is
            % SetReachZonotope, the class wins and the config payload is ignored.
            cfg = SetReach.config();
            if isempty(kind)
                kind = SetReach.setShape();
            end
            if isempty(kind) && ~isempty(cfg)
                % Nothing has named a shape, but a payload was handed over:
                % take the shape from the payload, so a struct payload
                % names its own shape without a separate setShape call.
                if isa(cfg, 'SetRep')
                    kind = cfg.kind;
                elseif isstruct(cfg) && isfield(cfg, 'kind')
                    kind = cfg.kind;
                end
            end
            if isempty(kind)
                kind = 'zonotope';
            end
            if ~isempty(cfg)
                try
                    candidate = SetRep.fromStruct(cfg, n);
                catch err
                    if strcmp(err.identifier, 'SetRep:badSize')
                        rethrow(err);       % a mis-sized IC is a user error
                    end
                    candidate = [];
                end
                if ~isempty(candidate) && strcmpi(candidate.kind, kind)
                    rep = candidate;
                    return
                end
            end
            rep = SetRep.fromSpec(kind, SetReach.resolveRadii(n), n, ...
                struct('Fit', SetReach.setFit()));
        end

        function spec = resolveRadii(n)
            % The spread half of the initial set: the model workspace variable
            % SetIC first, so the set IC travels with the model, then the static
            % setter. Looking the variable up is best-effort (there may be no
            % model context); VALIDATING it is not -- an ill-sized IC must
            % surface, so expansion happens in SetRep.
            %
            % simulatingModel(), not bdroot(gcs): gcs is the CURRENT system, which
            % is not the simulating one whenever more than one model is loaded.
            % Reading SetIC from the wrong model's workspace would defeat the
            % entire purpose of putting it there, silently.
            spec    = [];
            fromMdl = false;
            try
                mdl = simulatingModel();
                if ~isempty(mdl)
                    ws = get_param(mdl, 'ModelWorkspace');
                    if ~isempty(ws) && hasVariable(ws, 'SetIC')
                        spec    = getVariable(ws, 'SetIC');
                        fromMdl = true;
                    end
                end
            catch
                fromMdl = false;
            end
            if ~fromMdl
                spec = SetReach.setRadii();
            end
            if isempty(spec)
                % Nothing configured anywhere. zeros(n, 0) is the right NUMERICS
                % -- a set with no generators is a point, the solver then
                % propagates that point exactly, and inventing a radius here
                % would be this code guessing at a modelling decision. But it
                % must not happen SILENTLY: every logged set would be a single
                % point, so plotSetTube draws a tube of zero width, which is
                % indistinguishable from the centre trajectory and looks like a
                % broken plot rather than an unconfigured one. Say so once.
                warning('SetReach:pointInitialSet', ...
                    ['No initial set configured, so the initial set is the single ' ...
                     'point x0 and the reach tube will have zero width.\n' ...
                     'Set one of:\n' ...
                     '  SetReach.setRadii(0.1)                      %% isotropic half-width\n' ...
                     '  SetReach.setRadii([0.1; 0.05])              %% per-state half-widths\n' ...
                     'or put a SetIC variable in the model workspace so the initial\n' ...
                     'set travels with the model:\n' ...
                     '  ws = get_param(bdroot, ''ModelWorkspace''); assignin(ws, ''SetIC'', 0.1)']);
                spec = zeros(n, 0);
            end
        end

        function out = setShape(kind)
            % setShape('ellipsoid') picks the representation for plain 'SetReach';
            % setShape() reads it back. Ignored by the per-shape subclasses, whose
            % shape is fixed by the class.
            persistent K
            if nargin > 0
                if ~isempty(kind) && ~any(strcmpi(kind, SetRep.kinds()))
                    error('SetReach:kind', ...
                        'Unknown shape ''%s''. Choose one of: %s.', ...
                        kind, strjoin(SetRep.kinds(), ', '));
                end
                K = kind;
            end
            out = K;
        end

        function out = setRadii(spec)
            % setRadii(0.15) or setRadii([0.2 0.1]) or setRadii(G0) configures the
            % spread of the initial set; setRadii() reads it back. Needed because
            % the engine constructs the solver, so there is no constructor to pass
            % it through.
            persistent W
            if nargin > 0
                W = spec;
            end
            out = W;
        end

        function out = setFit(fit)
            % 'inscribed' (default) or 'circumscribed': how an ellipsoid initial
            % set relates to the box of the given half-widths. The one place where
            % the shape choice really does change the initial condition, so it is
            % explicit rather than buried. Irrelevant to the other three shapes.
            persistent F
            if isempty(F)
                F = 'inscribed';
            end
            if nargin > 0
                F = fit;
            end
            out = F;
        end

        function out = config(S0)
            % Explicit path: hand over a fully built SetRep, or a
            % struct('kind',...) payload. Useful for building an initial set that
            % the shape-agnostic spec cannot express, such as a circumscribed
            % rather than inscribed ellipsoid.
            persistent S
            if nargin > 0
                S = S0;
            end
            out = S;
        end

        function noteReset(t, x)
            R = SetReach.resetStore();
            R.t(end+1, 1) = t;
            R.x{end+1, 1} = x(:);
            SetReach.resetStore(R);
        end

        function out = getResets()
            % .t, .x for every reset. Exactly one entry, at t = 0, means no
            % continuous state jumped anywhere in the run.
            out = SetReach.resetStore();
        end

        function clearResets()
            SetReach.resetStore(struct('t', [], 'x', {{}}));
            SetReach.warnedJump(false);
        end

        function out = warnedJump(v)
            %WARNEDJUMP One state-jump warning per simulation, not one per impact.
            %   An explicit flag, rather than inferring "first reset after t = 0"
            %   from the reset record: on a model with a sampled input AND a real
            %   jump, the sample hits would consume the "first" slot and the
            %   genuine jump would then never warn.
            persistent W
            if isempty(W)
                W = false;
            end
            if nargin > 0
                W = v;
            end
            out = W;
        end

        function c = loggedCentre(t)
            %LOGGEDCENTRE The centre already propagated to time t, or [] if none.
            %   Read this BEFORE correctCentre, which overwrites it with the
            %   engine's own state and so destroys the comparison. Measured on
            %   sldemo_bounce: an entry exists at every reset time under BOTH the
            %   fixed-step and variable-step solvers, so the [] case is the t = 0
            %   establishing reset rather than a routine occurrence.
            L = SetReach.store();
            c = [];
            if isempty(L.t)
                return
            end
            k = find(abs(L.t - t) < 1e-12, 1, 'last');
            if ~isempty(k)
                c = L.c{k};
            end
        end

        function out = resetStore(in)
            persistent R
            if isempty(R)
                R = struct('t', [], 'x', {{}});
            end
            if nargin > 0
                R = in;
            end
            out = R;
        end

        function [Phi, d] = dense(A, f0, tau)
            %DENSE The step's exact affine flow, evaluated partway through it.
            %   [Phi, d] = SetReach.dense(A, f0, tau) gives the state at time
            %   t0 + tau as x0 + d and the set as S0.map(Phi), for any tau in
            %   [0, h] -- not just at tau = h.
            %
            %   This is the SAME expm step() takes, with h replaced by tau, so it
            %   is not an interpolant of the solver's answer: it IS the solver's
            %   answer, asked at a different time. Nothing is approximated that
            %   was not already approximated by freezing A and f0 at t0, and
            %   tau = h reproduces the step exactly.
            %
            %   One primitive, three uses:
            %     * resampleSetLog.m -- report a non-uniform run on a uniform grid,
            %       so a variable-step run and its ground truth stay comparable
            %       point for point;
            %     * SetReachVar.interpolateState -- the states the engine's
            %       zero-crossing bisection searches. The shipped default is
            %       LINEAR, which mislocates the sldemo_bounce impact by 164us at
            %       h = 0.05;
            %     * the between-step enclosure Omega_k in encloseSetLog, which
            %       needs Phi(tau) over tau in [0, h], exactly this.
            n = size(A, 1);
            if isempty(f0)
                f0 = zeros(n, 1);
            end
            E   = expm([A, f0(:); zeros(1, n + 1)] * tau);
            Phi = E(1:n, 1:n);
            d   = E(1:n, n + 1);
        end

        function correctCentre(t, x)
            %CORRECTCENTRE Replace a logged centre with the engine's own state.
            %   step() returns x1 and we log it immediately, but at a zero crossing
            %   the engine is not finished with it: it clamps the crossing state to
            %   the guard and applies the jump map, and reset() is where it hands
            %   the corrected state back. Until that is folded in, the log disagrees
            %   with the engine's solution on exactly the steps that matter.
            %
            %   Measured on sldemo_bounce: 1 of 1501 logged times disagreed with
            %   tout, at the bounce, by 8.5mm in position and 39.87 in the 2-norm
            %   -- the pre-impact velocity (-22.1) where the engine holds the
            %   post-impact one (+17.7). The tell that this is bookkeeping and not
            %   discretisation is that 39.87 did not move across h = 0.008 .. 0.001
            %   while the genuine guard-timing error fell 0.123 -> 0.0077.
            %
            %   Position is continuous across a jump, so only the jumping states
            %   change; the entry is overwritten wholesale because x is exactly what
            %   the engine will carry forward.
            L = SetReach.store();
            if isempty(L.t)
                return
            end
            k = find(abs(L.t - t) < 1e-12, 1, 'last');
            if isempty(k)
                % No entry at this time yet. Fixed-step: reset at t=0 before any
                % step has run. Variable-step: t is a located root strictly inside
                % a step, and the next step() call will record it as its own t0.
                return
            end
            L.c{k} = x(:);
            SetReach.store(L);
        end

        function rewind(t0)
            %REWIND Drop log entries for steps the engine did not keep.
            %   A variable-step engine DISCARDS completed steps: at a zero crossing
            %   it lets the step finish, localises the root itself, throws the
            %   endpoint away and re-drives the solver from t*. Measured with a
            %   nominal h of 0.05 on sldemo_bounce, the discarded step had reached
            %   pos = -0.383.
            %
            %   Deliberately NOT called from this class's step(): under a fixed-step
            %   base t0 always advances past the last entry, so it could only ever
            %   be a no-op here, and dead machinery with a live-sounding comment is
            %   worse than none. It lives here because the log it repairs is static
            %   and shared; the variable-step sibling is what calls it.
            %
            %   NOTE that it only repairs the LOG. The set in obj.S must be rewound
            %   by the caller, which needs the set as it stood BEFORE the discarded
            %   step -- so a caller must cache that itself.
            L = SetReach.store();
            if isempty(L.t)
                return
            end
            keep = L.t <= t0 + 1e-12;
            if all(keep)
                return
            end
            L.t = L.t(keep);
            L.c = L.c(keep);
            L.S = L.S(keep);
            L.mu = L.mu(keep);
            L.A = L.A(keep);
            L.h = L.h(keep);
            L.f0 = L.f0(keep);
            SetReach.store(L);
        end

        function record(t, c, S, mu, A, h, f0)
            % NOTE ON ALIGNMENT: step() evaluates the Jacobian at t0 but
            % records against t0+h, so entry k carries mu and A evaluated at
            % the centre of entry k-1. Consumers must shift.
            %
            % Read positively, the same convention says: for k >= 2, entry k
            % carries the data of the step that PRODUCED it -- h, mu, A and f0
            % all describe the interval (t_{k-1}, t_k]. Entry 1 is a harmless
            % duplicate of entry 2's step data, because step() records both from
            % the same call. h and f0 are logged for two reasons:
            %
            %   * once h varies, "shift one entry" stops meaning "shift by h",
            %     and diff(L.t) is not a substitute -- across a discarded step or
            %     a reset the logged gap is not the h that was passed to step();
            %   * A and f0 together are the DENSE OUTPUT of the step,
            %     Phi(tau) = expm([A f0; 0]*tau) for tau in [0, h], which is what
            %     lets the tube be evaluated at times the solver never stopped at.
            %     See resampleSetLog.m.
            L = SetReach.store();
            L.t(end+1, 1) = t;
            L.c{end+1, 1} = c;
            L.S{end+1, 1} = S;
            L.mu(end+1, 1) = mu;
            L.A{end+1, 1} = A;
            L.h(end+1, 1) = h;
            L.f0{end+1, 1} = f0(:);
            SetReach.store(L);
        end

        function out = getLog()
            out = SetReach.store();
        end

        function resetLog()
            SetReach.store(SetReach.emptyLog());
        end

        function L = emptyLog()
            %EMPTYLOG The log's shape, in one place so consumers can build one too.
            L = struct('t', [], 'c', {{}}, 'S', {{}}, 'mu', [], 'A', {{}}, ...
                'h', [], 'f0', {{}}, 'mdl', '');
        end

        function stampModel()
            %STAMPMODEL Record WHICH MODEL produced this log.
            %   Called from start(), so the stamp and the clearing happen together.
            %
            %   The log is a persistent store shared by every set solver, and until
            %   this field existed a log carried no evidence of where it came from.
            %   That is a silent-wrong-answer trap, and it fires for real in one
            %   specific case: a model with NO CONTINUOUS STATES never runs a
            %   continuous solver, so start() is never called, so the PREVIOUS run's
            %   log is neither cleared nor replaced. Measured -- simulating
            %   sldemo_bounce and then a Clock -> Unit Delay model leaves
            %   SetReach.getLog() holding the bouncing ball's 1401 entries, and
            %   plotSetTube draws them without complaint as if they were the delay
            %   model's.
            %
            %   A stamp cannot prevent the staleness -- there is no solver to run --
            %   but it makes it VISIBLE: plotSetTube names the source model in its
            %   title, so a figure can no longer be silently attributed to the wrong
            %   one. Best-effort, because there may be no model context at all.
            L = SetReach.store();
            L.mdl = '';
            try
                L.mdl = simulatingModel();
            catch
                % no model context: an unstamped log is honest, a guessed one is not
            end
            SetReach.store(L);
        end

        function out = store(in)
            persistent L
            if isempty(L)
                L = SetReach.emptyLog();
            end
            if nargin > 0
                L = in;
            end
            out = L;
        end
    end
end
