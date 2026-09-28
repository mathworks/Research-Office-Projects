classdef SetReachVarSupport < SetReachVar
    %SETREACHVARSUPPORT Variable-step SetReach carrying a support function (LGG / SpaceEx).
    %
    %   See SETREACHVARZONOTOPE for why getProperties is re-declared.
    %
    %   Worth knowing before reading a difference into the plots: this and the
    %   zonotope entry represent the SAME set, to 7e-16 over 720 directions. They
    %   differ in what they answer cheaply and how they draw, not in what they
    %   contain.
    %
    %   See also SETREACHVAR, SETREACHSUPPORT, REGISTERSETSOLVERS.

    methods
        function k = shapeKind(~)
            k = 'support';
        end
    end

    methods (Static)
        function props = getProperties()
            props = SetReachVar.getProperties();
        end
    end
end
