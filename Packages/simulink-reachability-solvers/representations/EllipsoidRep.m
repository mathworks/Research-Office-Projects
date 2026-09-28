classdef EllipsoidRep < SetRep
    %ELLIPSOIDREP E = { c + x : x'*inv(Q)*x <= 1 }, propagated as Q <- Phi*Q*Phi'.
    %
    %   The one representation here that is genuinely a DIFFERENT SET from the
    %   others, and the only one whose payload cannot grow: Q is n-by-n whatever
    %   you do to it. That fixed cost is the whole appeal (Kurzhanski/Varaiya,
    %   and the ellipsoidal reachability in the STRONG toolbox and Yi
    %   Deng's thesis, which wrap each simulated trajectory of a hybrid
    %   system in an ellipsoidal neighbourhood; see the README references).
    %
    %   Exactness. Under a linear map an ellipsoid maps to an ellipsoid EXACTLY, so
    %   Q <- Phi*Q*Phi' loses nothing -- and Phi = expm(A*h) is always invertible,
    %   so Q stays positive definite and never degenerates. What ellipsoids cannot
    %   do exactly is Minkowski SUM, which is where the conservatism enters as soon
    %   as you add input sets or error terms. Nothing here sums yet, so on these
    %   models the ellipsoid is as exact as the zonotope.
    %
    %   INITIAL-SET FIT, and it is the only reason ellipsoid results differ from
    %   zonotope results at all. Given per-state half-widths w, there are two
    %   honest ellipsoids:
    %
    %     'inscribed'      Q = W*W'      the largest ellipsoid INSIDE the box.
    %                                    Smaller set, so tighter numbers -- but it
    %                                    is not the same initial condition, and
    %                                    comparing it to the zonotope is unfair.
    %     'circumscribed'  Q = n*W*W'    the smallest ellipsoid CONTAINING the box.
    %                                    The apples-to-apples control: same initial
    %                                    set, so any difference downstream is the
    %                                    representation's doing. The factor n is
    %                                    exact for a box (its corners are at
    %                                    distance sqrt(n) in W-normalised
    %                                    coordinates).
    %
    %   See also SETREP, ZONOTOPEREP.

    properties (Constant)
        kind = 'ellipsoid'
    end

    properties
        Q       % shape matrix, n-by-n, symmetric PSD
    end

    methods
        function obj = EllipsoidRep(Q)
            if nargin > 0
                obj.Q = (Q + Q') / 2;    % keep it symmetric against drift
            end
        end

        function obj = map(obj, Phi)
            Qm = Phi * obj.Q * Phi';
            obj.Q = (Qm + Qm') / 2;
        end

        function r = support(obj, d)
            % rho_E(d) - d'c = sqrt(d'*Q*d). Exact.
            r = sqrt(max(d' * obj.Q * d, 0));
        end

        function q = ratio(obj, dx)
            % The Mahalanobis distance IS the exact containment ratio, in closed
            % form -- no direction sweep and no discretisation error. Solve rather
            % than invert.
            if rcond(obj.Q) < 1e-14
                q = ratio@SetRep(obj, dx);   % degenerate: fall back
                return
            end
            q = sqrt(max(dx' * (obj.Q \ dx), 0));
        end

        function V = vertices(obj, c, nDir)
            % Exact boundary: c + Q^(1/2)*[cos; sin]. chol is cheaper and gives a
            % different-but-equivalent parameterisation of the same curve; the
            % eigendecomposition below covers the semidefinite case chol rejects.
            %
            % nDir is honoured rather than ignored, unlike ZonotopeRep: the curve
            % is exact but its rendering is sampled, so a caller drawing a whole
            % tube can ask for a coarser one.
            if obj.dim() ~= 2
                V = [];
                return
            end
            if nargin < 3 || isempty(nDir)
                nDir = 360;
            end
            th = linspace(0, 2*pi, max(3, round(nDir)) + 1);
            [R, p] = chol(obj.Q);
            if p == 0
                M = R';
            else
                % chol rejecting Q does not mean the set is broken. On a contracting
                % flow the ellipse collapses onto a needle and the smaller eigenvalue
                % lands at something like -1e-30 instead of +0, which is roundoff, not
                % geometry. sqrtm() then warns "Matrix is singular and may not have a
                % square root" -- measured on the vdp four-shape render, once per
                % frame -- and returns a complex factor that has to be re-realed.
                %
                % A SYMMETRIC EIGENDECOMPOSITION is both exact and quiet. Q is
                % symmetric by construction (the map is Q <- Phi*Q*Phi'), so
                % Q = W*diag(e)*W' and the PSD square root is W*diag(sqrt(e))*W'.
                % Clamping e at zero is the only approximation and it is the right
                % one: a negative eigenvalue of a matrix that is PSD in exact
                % arithmetic is roundoff, and the honest square root of that
                % direction is a flat one, which draws the needle the set has
                % actually become.
                [W, e] = eig(full(obj.Q), 'vector');
                M = W * diag(sqrt(max(real(e), 0))) * W';
            end
            V = c(:) + M * [cos(th); sin(th)];
        end

        function s = payload(obj)
            n = size(obj.Q, 1);
            s = sprintf('Q is %d-by-%d, always', n, n);
        end

        function M = payloadMatrix(obj)
            M = obj.Q;
        end

        function obj = project(obj, idx)
            % rho(d) = sqrt(d'*P*Q*P'*d) for a coordinate selector P, and P*Q*P' is
            % just the submatrix. Exact -- the shadow of an ellipsoid is an
            % ellipsoid, no enclosure needed.
            obj.Q = obj.Q(idx, idx);
        end
    end

    methods (Static)
        function obj = fromGenerators(G0, fit)
            %FROMGENERATORS Ellipsoid from a generator/half-width matrix.
            %   fit = 'inscribed' (default) or 'circumscribed'. See the class
            %   comment: the choice is a statement about what the initial
            %   condition IS, not a tuning knob.
            if nargin < 2 || isempty(fit)
                fit = 'inscribed';
            end
            n = size(G0, 1);
            switch lower(fit)
                case 'inscribed'
                    s = 1;
                case 'circumscribed'
                    s = n;
                otherwise
                    error('EllipsoidRep:fit', ...
                        'Fit must be ''inscribed'' or ''circumscribed'', got ''%s''.', fit);
            end
            obj = EllipsoidRep(s * (G0 * G0'));
        end
    end
end
