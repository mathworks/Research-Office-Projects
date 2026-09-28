classdef SetReachVarEllipsoid < SetReachVar
    %SETREACHVARELLIPSOID Variable-step SetReach with the shape fixed to an ellipsoid.
    %
    %   See SETREACHVARZONOTOPE for why getProperties is re-declared, and
    %   ELLIPSOIDREP / SetReach.setFit for the one place where choosing this shape
    %   really does change the initial condition: an ellipsoid must decide whether
    %   to inscribe or circumscribe the box of the given half-widths.
    %
    %   See also SETREACHVAR, SETREACHELLIPSOID, REGISTERSETSOLVERS.

    methods
        function k = shapeKind(~)
            k = 'ellipsoid';
        end
    end

    methods (Static)
        function props = getProperties()
            props = SetReachVar.getProperties();
        end
    end
end
