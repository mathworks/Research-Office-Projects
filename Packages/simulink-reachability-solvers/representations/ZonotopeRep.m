classdef ZonotopeRep < SetRep
    %ZONOTOPEREP Z = { c + G*b : ||b||_inf <= 1 }, propagated as G <- Phi*G.
    %
    %   The baseline. Exact under a linear map, exact support function, and exact
    %   vertices in 2-D.
    %
    %   Cost: the payload is n-by-m and m is whatever you put in it. Under a PURE
    %   map m never changes, which is why this solver's G stays 2-by-2 forever. Add
    %   input uncertainty or a remainder term per step and m grows linearly, which
    %   is where order reduction becomes necessary. Nothing here does that yet.
    %
    %   See also SETREP, ELLIPSOIDREP, ZONOTOPEVERTICES.

    properties (Constant)
        kind = 'zonotope'
    end

    properties
        G       % generator matrix, n-by-m
    end

    methods
        function obj = ZonotopeRep(G)
            if nargin > 0
                obj.G = G;
            end
        end

        function obj = map(obj, Phi)
            obj.G = Phi * obj.G;
        end

        function r = support(obj, d)
            % rho_Z(d) - d'c = ||G'd||_1. Exact, no sampling.
            if isempty(obj.G)
                r = 0;
            else
                r = norm(obj.G' * d, 1);
            end
        end

        function q = ratio(obj, dx)
            % Exact and O(n^3) when G is square and well conditioned: the b that
            % reaches dx is G\dx, and the set is ||b||_inf <= 1. Otherwise fall
            % back to the support sweep, which is what an over- or
            % under-determined generator matrix needs.
            G = obj.G;
            if isempty(G)
                q = Inf * (norm(dx) > 0);
                return
            end
            if size(G, 1) == size(G, 2) && rcond(G) > 1e-12
                q = max(abs(G \ dx));
            else
                q = ratio@SetRep(obj, dx);
            end
        end

        function V = vertices(obj, c, ~)
            % The direction count the base class takes is accepted and ignored:
            % this boundary is exact and costs one sort of the generators, so
            % there is no accuracy to trade against speed.
            if obj.dim() ~= 2
                V = [];
                return
            end
            V = zonotopeVertices(c(:), obj.G);
        end

        function s = payload(obj)
            s = sprintf('G is %d-by-%d', size(obj.G, 1), size(obj.G, 2));
        end

        function M = payloadMatrix(obj)
            M = obj.G;
        end

        function obj = project(obj, idx)
            % Exact: the shadow of a zonotope is the zonotope of the shadowed
            % generators. Note m is unchanged, so a projected 2-D zonotope can have
            % many generators and genuinely more than four vertices.
            obj.G = obj.G(idx, :);
        end
    end
end
