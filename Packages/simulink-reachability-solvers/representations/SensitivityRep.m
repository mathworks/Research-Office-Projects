classdef SensitivityRep < SetRep
    %SENSITIVITYREP The flow's Jacobian S = d(x_t)/d(x_0), propagated as S <- Phi*S.
    %
    %   The HSCC'09 (Donze-Krogh-Rajhans) object. S starts at the identity and
    %   satisfies the variational equation Sdot = A(t,x(t))*S along the centre
    %   trajectory; the exact discrete solution of that over one step with A frozen
    %   is exactly S <- expm(A*h)*S, which is the same recursion ZonotopeRep runs.
    %
    %   Selecting 'sensitivity' in the dropdown instead of 'zonotope' changes
    %   nothing about the numbers -- the two agree to machine precision, because
    %   with initial radii W the set is literally the
    %   zonotope with G = S*W. What changes is the READOUT, and the readouts are the
    %   reason to have the entry at all:
    %
    %     gain()          ||S||_2: how much the flow amplifies ANY initial
    %                     perturbation. One number per time point, the quantity
    %                     sensitivity analysis actually reports.
    %     columnGains()   ||S(:,j)||: how much state j's initial uncertainty
    %                     contributes. This is what tells you WHICH initial
    %                     condition to pin down, and no set representation exposes
    %                     it -- once you form G = S*W the columns are mixed.
    %     conditioning    svd(S): the directions of maximum and minimum expansion.
    %
    %   It is also the ingredient the HSCC'09 error indicator needs. Prop. 1's
    %   Err = ||xi_pj - xihat|| + ||S_pj - S_p||*||Sj|| is stated in terms of
    %   sensitivity MATRICES of two neighbouring trajectories, not of sets, so a
    %   run that carries S can compute it directly.
    %
    %   The honest cost of the identification: because it is the same recursion, it
    %   inherits the same approximation, for the same reason. A measured h-plateau
    %   (0.469 -> 0.332 over an 8x refinement) shows that the h->0
    %   limit of the zonotope recursion IS this sensitivity matrix -- the excess
    %   does not vanish with h because linearising about the centre trajectory is
    %   not the same as mapping the set.
    %
    %   See also SETREP, ZONOTOPEREP.

    properties (Constant)
        kind = 'sensitivity'
    end

    properties
        S       % sensitivity matrix d(x_t)/d(x_0), n-by-n; S(0) = I
        G0      % initial half-widths, n-by-m; what S multiplies to become a set
    end

    methods
        function obj = SensitivityRep(S, G0)
            % G0 IS REQUIRED. It used to default to eye(size(S,1)), which
            % conflated two different identities: S(0) = I is genuinely
            % neutral, "no propagation has happened yet", while G0 = I is a
            % size claim, "every initial half-width is 1". The examples here
            % run at half-widths of 0.05 to 0.1, so that default returned a
            % set ten to twenty times too large with no error and no warning,
            % and the tube looked entirely plausible while being wrong by an
            % order of magnitude. The initial spread is setRadii's and
            % SetIC's to decide, not this constructor's.
            if nargin > 0
                if nargin < 2
                    error('SensitivityRep:missingG0', ...
                        ['SensitivityRep(S, G0) needs G0, the initial ' ...
                         'half-widths. There is no default; see setRadii.']);
                end
                obj.S  = S;
                obj.G0 = G0;
            end
        end

        function obj = map(obj, Phi)
            obj.S = Phi * obj.S;
        end

        function r = support(obj, d)
            % Reading the sensitivity matrix AS a set: the image S*X0 of the
            % initial box. Identical to ZonotopeRep with G = S*G0, by construction.
            if isempty(obj.G0)
                r = 0;
            else
                r = norm((obj.S * obj.G0)' * d, 1);
            end
        end

        function g = gain(obj)
            %GAIN ||S||_2 -- worst-case amplification of an initial perturbation.
            g = norm(obj.S, 2);
        end

        function g = columnGains(obj)
            %COLUMNGAINS Per-initial-state contribution, ||S(:,j)||_2.
            %   The readout that justifies this entry existing: it survives only
            %   while the columns are still separate, i.e. before S*W is formed.
            g = vecnorm(obj.S)';
        end

        function [smax, smin] = spread(obj)
            %SPREAD Largest and smallest singular values of S.
            %   smax/smin is the conditioning of the flow map: how anisotropic the
            %   expansion has become.
            s = svd(obj.S);
            smax = s(1);
            smin = s(end);
        end

        function V = vertices(obj, c, ~)
            % Same boundary as the equivalent zonotope -- it is the same set.
            % Direction count accepted and ignored, as in ZonotopeRep: exact.
            if obj.dim() ~= 2
                V = [];
                return
            end
            V = zonotopeVertices(c(:), obj.S * obj.G0);
        end

        function q = ratio(obj, dx)
            q = ZonotopeRep(obj.S * obj.G0).ratio(dx);
        end

        function s = payload(obj)
            s = sprintf('S is %d-by-%d, ||S|| = %.4g', ...
                size(obj.S, 1), size(obj.S, 2), obj.gain());
        end

        function M = payloadMatrix(obj)
            M = obj.S;
        end

        function obj = project(obj, idx)
            % Slice the rows: S(idx,:) is d(x_idx)/d(x_0), the sensitivity of the
            % PLOTTED states to every initial state. Exact as a set projection, and
            % still meaningful as a sensitivity -- but note gain() and spread() then
            % describe the projected map, not the full flow. Report those from the
            % unprojected rep.
            obj.S = obj.S(idx, :);
        end

        function Z = toZonotope(obj)
            Z = ZonotopeRep(obj.S * obj.G0);
        end
    end
end
