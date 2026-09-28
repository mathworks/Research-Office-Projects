classdef SupportRep < SetRep
    %SUPPORTREP The LGG / SpaceEx recursion: store no set, only the accumulated map.
    %
    %   The trick (Le Guernic-Girard; SpaceEx) is that a support function commutes
    %   with a linear map for free:
    %
    %       rho_{Phi X}(d) = rho_X(Phi'*d)
    %
    %   so propagating a set through Phi_k...Phi_1 needs no set operations at all.
    %   Accumulate Phi <- Phi_h*Phi and answer any query by pushing the
    %   direction BACKWARDS through Phi' onto the initial set, whose support is
    %   known in closed form. Every query is exact; nothing is ever
    %   over-approximated by the recursion itself.
    %
    %   What you buy: a directional query costs one matrix-vector product against
    %   the initial set, no matter how many steps have elapsed, and you pay only for
    %   the directions you actually ask about. On a 200-state model where you care
    %   about one safety halfspace, that is the difference between tractable and
    %   not.
    %
    %   What you pay, and it is visible in the plots: you never HAVE the set. To
    %   draw it you must sample directions and intersect the supporting halfspaces,
    %   which gives a polygon that strictly CONTAINS the true set and tightens as
    %   the sample count grows. SetRep's default vertices() does exactly that, so
    %   selecting 'support' in the dropdown renders a visibly polygonal outer
    %   approximation where 'zonotope' renders exact vertices -- of the same set.
    %
    %   HONEST NOTE. With a box initial set this carries mathematically the same
    %   object as ZonotopeRep with G = Phi*W: rho = ||W'*Phi'*d||_1 is ||G'd||_1,
    %   and the two agree to machine precision. The representation differs in cost
    %   profile and in what it can draw, not in the set.
    %
    %   See also SETREP, ZONOTOPEREP.

    properties (Constant)
        kind = 'support'
    end

    properties
        Phi     % accumulated linear map, n-by-n; the ONLY thing that propagates
        G0      % initial set's generators, n-by-m; never touched after construction
    end

    methods
        function obj = SupportRep(Phi, G0)
            % G0 IS REQUIRED and deliberately has no default. eye(n) is the
            % tempting one and it is wrong: for Phi the identity is neutral,
            % "nothing has propagated yet", but for G0 it is a size claim,
            % "every initial half-width is 1", and the examples here run at
            % half-widths of 0.05 to 0.1. Defaulting it would hand back a set
            % an order of magnitude too large, with no error and no warning,
            % which is the worst failure mode available. The initial spread
            % already has an owner in setRadii and the SetIC model-workspace
            % variable, so a default here would give the radius two sources
            % of truth as well.
            if nargin > 0
                if nargin < 2
                    error('SupportRep:missingG0', ...
                        ['SupportRep(Phi, G0) needs G0, the initial set''s ' ...
                         'generators. There is no default; see setRadii.']);
                end
                obj.Phi = Phi;
                obj.G0  = G0;
            end
        end

        function obj = map(obj, Phi)
            % No set operation. This is the entire propagation.
            obj.Phi = Phi * obj.Phi;
        end

        function r = support(obj, d)
            % rho_{Phi X0}(d) = rho_{X0}(Phi'd), evaluated lazily and exactly.
            if isempty(obj.G0)
                r = 0;
            else
                r = norm(obj.G0' * (obj.Phi' * d), 1);
            end
        end

        function s = payload(obj)
            s = sprintf('Phi is %d-by-%d, set never formed', ...
                size(obj.Phi, 1), size(obj.Phi, 2));
        end

        function M = payloadMatrix(obj)
            M = obj.Phi;
        end

        function obj = project(obj, idx)
            % rho_{P Phi X0}(d) = rho_{X0}(Phi'*P'*d), so slicing the ROWS of Phi is
            % the projection -- and it is still lazy. Phi becomes 2-by-n, which is
            % what dim() then reports, and vertices() still has to sample directions.
            % So a projected support-function set still renders as an outer polygon,
            % correctly: the honesty about rendering survives the projection.
            obj.Phi = obj.Phi(idx, :);
        end

        function Z = toZonotope(obj)
            %TOZONOTOPE Materialise the set this rep is implicitly carrying.
            %   Only for cross-checking against ZonotopeRep -- doing this every
            %   step would throw away the reason to use a support function.
            Z = ZonotopeRep(obj.Phi * obj.G0);
        end
    end
end
