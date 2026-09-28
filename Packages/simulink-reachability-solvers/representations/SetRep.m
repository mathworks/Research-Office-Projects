classdef (Abstract) SetRep
    %SETREP One set representation, as a swappable strategy.
    %
    %   The plugin solver supplies exactly two things: somewhere to carry state
    %   across steps, and the exact linear map Phi = expm(A*h). Any representation
    %   CLOSED UNDER LINEAR MAPS therefore drops straight in, and the only
    %   genuinely representation-specific method is map(). Everything else here --
    %   containment ratios, interval hulls, plot polygons -- follows from the
    %   support function, which every convex symmetric set has.
    %
    %   The four shipped representations, and what actually differs between them:
    %
    %     ZonotopeRep     G <- Phi*G           exact. Payload n-by-m; stable under
    %                                          a pure map, grows once you add
    %                                          input or remainder generators.
    %     EllipsoidRep    Q <- Phi*Q*Phi'      exact. Payload ALWAYS n-by-n, so it
    %                                          can never grow. Phi is invertible
    %                                          (expm always is) so Q stays PD.
    %     SupportRep      Phi <- Phi_h*Phi     lazy: store no set at all, just
    %                                          the accumulated map, and evaluate
    %                                          rho on demand. The LGG / SpaceEx
    %                                          recursion.
    %     SensitivityRep  S <- Phi*S           the flow's Jacobian d(x_t)/d(x_0),
    %                                          i.e. HSCC'09 sensitivity.
    %
    %   HONEST WARNING, AND IT IS THE POINT OF HAVING THEM SIDE BY SIDE. Three of
    %   these four propagate the SAME OBJECT. A zonotope with G = S*W, a support
    %   function over the box W mapped by S, and a sensitivity matrix S with
    %   initial radii W are three encodings of one set, and they agree to machine
    %   precision. What differs is:
    %
    %     - the QUESTION each answers cheaply (containment vs a halfspace query vs
    %       per-state gain and ||S||),
    %     - the PAYLOAD's growth behaviour once you add generators,
    %     - how each RENDERS: SupportRep draws the outer polygon from N sampled
    %       directions, which visibly tightens as N grows, where ZonotopeRep draws
    %       exact vertices.
    %
    %   The ellipsoid is the one that genuinely differs, and only through its
    %   initial set: a ball of radius w is a strict subset of the box of
    %   half-width w. Use Fit = 'circumscribed' to compare like with like.
    %
    %   WHAT NO CHOICE HERE CHANGES is the approximation on nonlinear dynamics. As
    %   h->0 every one converges to the same variational equation
    %   Sdot = A(t,c(t))*S about the centre trajectory, so it is not a shape
    %   problem. The error is quadratic in the diameter of the set; see the README.
    %
    %   PROJECTION IS EXACT FOR ALL FOUR, which is what makes a phase-plane plot of
    %   a 6-state model honest rather than decorative. rho_{Px}(d) = rho_x(P'd), so
    %   projecting is just slicing the payload: G(idx,:), Q(idx,idx), Phi(idx,:),
    %   S(idx,:). No enclosure and no sampling error is introduced -- the picture is
    %   the true shadow of the true set. What projection does NOT do is tell you
    %   about the states you left out, so a 2-D view can look tight while the set is
    %   enormous in a direction you are not looking at.
    %
    %   Reps are VALUE classes: map() returns a new object rather than mutating,
    %   so a logged history is genuinely a history and not N aliases of one
    %   handle. They are also centred at the ORIGIN -- the centre rides in the
    %   engine's state vector, and is passed in where it is needed.
    %
    %   See also SETREACH, ZONOTOPEREP, ELLIPSOIDREP, SUPPORTREP, SENSITIVITYREP.

    properties (Abstract, Constant)
        % The shape's name: the stem registerSetSolvers builds the solver
        % names from, the field a struct payload sets, and what describe
        % and the diagnostics report.
        kind
    end

    methods (Abstract)
        obj = map(obj, Phi)         % push the set through a linear map
        r   = support(obj, d)       % half-width along unit d, measured from c
        s   = payload(obj)          % what is being carried, for display
        M   = payloadMatrix(obj)    % the matrix that fixes the dimension
        obj = project(obj, idx)     % coordinate projection, EXACT for all four
    end

    methods
        function b = halfWidths(obj)
            % Interval hull half-widths: support along each +e_i.
            n = obj.dim();
            b = zeros(n, 1);
            for k = 1:n
                e = zeros(n, 1);
                e(k) = 1;
                b(k) = obj.support(e);
            end
        end

        function q = ratio(obj, dx)
            % Containment ratio of the point c + dx: <= 1 means inside.
            %   q = max_d  d'dx / rho(d)
            % which is exact for a convex symmetric set. Sampling the directions
            % can only UNDER-report q, so a reported violation is real; a reported
            % containment is right to the discretisation. Reps with a closed form
            % override this (EllipsoidRep does).
            D = SetRep.directions(obj.dim());
            q = 0;
            for k = 1:size(D, 2)
                d = D(:, k);
                r = obj.support(d);
                if r > eps
                    q = max(q, (d' * dx) / r);
                end
            end
        end

        function V = vertices(obj, c, nDir)
            % 2-D boundary polygon, 2-by-nv, or [] if this is not a plane.
            %
            % Default: the OUTER polygon cut out by the supporting halfspaces
            % along sampled directions -- exactly what a support-function
            % implementation can actually draw. Consecutive directions (sorted by
            % angle) give consecutive vertices, so each is one 2-by-2 solve.
            % ZonotopeRep and EllipsoidRep override with exact boundaries.
            %
            % nDir optionally overrides the direction count. The default 720 is
            % right for drawing ONE set, where the cost is invisible and the
            % outline should look smooth. It is the wrong default for drawing a
            % whole reach tube: a 749-interval enclosure costs 21.5 s at 720
            % directions and 1.4 s at 48, and at tube scale each outline is a few
            % pixels across, so the extra directions buy nothing a viewer can
            % see. The caller that knows which case it is passes the count.
            if obj.dim() ~= 2
                V = [];
                return
            end
            if nargin < 3 || isempty(nDir)
                D = SetRep.directions(2);
            else
                D = SetRep.directions(2, nDir);
            end
            nd = size(D, 2);
            r = zeros(nd, 1);
            for k = 1:nd
                r(k) = obj.support(D(:, k));
            end
            V = zeros(2, nd);
            for k = 1:nd
                j = mod(k, nd) + 1;
                M = [D(:, k)'; D(:, j)'];
                if abs(det(M)) < 1e-12
                    V(:, k) = c + r(k) * D(:, k);       % degenerate: touch point
                else
                    V(:, k) = c + M \ [r(k); r(j)];
                end
            end
        end

        function n = dim(obj)
            n = size(obj.payloadMatrix(), 1);
        end

        function s = describe(obj)
            s = sprintf('%s (%s)', obj.kind, obj.payload());
        end
    end

    methods (Static)
        function D = directions(n, m)
            % Unit directions for support sampling. 2-D gets an angle sweep; other
            % dimensions get a deterministic pseudo-random sphere sample, seeded so
            % repeated runs are comparable.
            if nargin < 2
                m = 720;
            end
            if n == 2
                th = linspace(0, 2*pi, m + 1);
                th(end) = [];
                D = [cos(th); sin(th)];
            else
                rs = RandStream('twister', 'Seed', 0);
                D  = randn(rs, n, m);
                D  = D ./ max(vecnorm(D), eps);
            end
        end

        function rep = fromSpec(kind, spec, n, opts)
            % Build a representation from a shape name and a radius spec.
            %
            %   kind   'zonotope' | 'ellipsoid' | 'support' | 'sensitivity'
            %   spec   scalar half-width (isotropic box), per-state vector of
            %          half-widths, or a full n-by-m generator matrix
            %   opts   optional struct; .Fit = 'inscribed' (default) or
            %          'circumscribed' for EllipsoidRep
            if nargin < 4 || isempty(opts)
                opts = struct();
            end
            G0 = SetRep.expandRadii(spec, n);
            switch lower(kind)
                case 'zonotope'
                    rep = ZonotopeRep(G0);
                case 'ellipsoid'
                    fit = 'inscribed';
                    if isfield(opts, 'Fit') && ~isempty(opts.Fit)
                        fit = opts.Fit;
                    end
                    rep = EllipsoidRep.fromGenerators(G0, fit);
                case 'support'
                    rep = SupportRep(eye(n), G0);
                case 'sensitivity'
                    rep = SensitivityRep(eye(n), G0);
                otherwise
                    error('SetRep:kind', ...
                        ['Unknown set representation ''%s''. Choose zonotope, ' ...
                         'ellipsoid, support, or sensitivity.'], kind);
            end
        end

        function rep = fromStruct(s, n)
            % Accept either form config() can hold: a SetRep, or a struct
            % payload. The fields are kind-specific and all of them are
            % required: G for zonotope, Q for ellipsoid, Phi and G0 for
            % support, S and G0 for sensitivity.
            %
            % Every field goes through requirePayload rather than being read
            % directly. This is the entry point the README advertises as the
            % escape hatch for sets the radii spec cannot express, so a
            % hand-written payload is exactly where a typo is likely, and a
            % raw MATLAB:nonExistentField names the field without naming the
            % kind that wanted it.
            if isa(s, 'SetRep')
                rep = s;
                return
            end
            if ~isstruct(s) || ~isfield(s, 'kind')
                error('SetRep:badStruct', ...
                    'Expected a SetRep object or a struct with a ''kind'' field.');
            end
            switch lower(s.kind)
                case 'zonotope'
                    rep = ZonotopeRep(SetRep.requirePayload(s, 'G'));
                case 'ellipsoid'
                    rep = EllipsoidRep(SetRep.requirePayload(s, 'Q'));
                case 'support'
                    rep = SupportRep(SetRep.requirePayload(s, 'Phi'), ...
                                     SetRep.requirePayload(s, 'G0'));
                case 'sensitivity'
                    rep = SensitivityRep(SetRep.requirePayload(s, 'S'), ...
                                         SetRep.requirePayload(s, 'G0'));
                otherwise
                    error('SetRep:kind', 'unknown kind %s', s.kind);
            end
            if nargin > 1 && ~isempty(n) && rep.dim() ~= n
                error('SetRep:badSize', ...
                    'Representation is %d-dimensional but the model has %d states.', ...
                    rep.dim(), n);
            end
        end

        function v = requirePayload(s, field)
            %REQUIREPAYLOAD Read a required payload field, or say what is missing.
            %   fromStruct dispatches on .kind and then reads kind-specific
            %   fields, so the error a reader most needs names both: which
            %   field is absent and which kind asked for it.
            if ~isfield(s, field)
                error('SetRep:badStruct', ...
                    'A ''%s'' payload needs a ''%s'' field.', s.kind, field);
            end
            v = s.(field);
        end

        function G0 = expandRadii(v, n)
            % The three natural spellings of an initial set: a full n-by-m
            % generator matrix, a per-state vector of half-widths, or a scalar
            % half-width for an isotropic box. Shared by every representation so
            % the dropdown choice never changes what the IC MEANS.
            if isempty(v)
                G0 = zeros(n, 0);
                return
            end
            if isstruct(v) && isfield(v, 'G')
                v = v.G;
            end
            if isscalar(v)
                G0 = v * eye(n);
            elseif isvector(v) && numel(v) == n
                G0 = diag(v(:));
            else
                G0 = v;
            end
            if size(G0, 1) ~= n
                error('SetRep:badInitialSet', ...
                    ['Initial set has %d rows but the model has %d continuous ' ...
                     'states. It must span the state space.'], size(G0, 1), n);
            end
        end

        function k = kinds()
            % The shape list, in one place. Everything that offers a shape
            % choice reads this rather than hardcoding its own copy.
            k = {'zonotope', 'ellipsoid', 'support', 'sensitivity'};
        end
    end
end
