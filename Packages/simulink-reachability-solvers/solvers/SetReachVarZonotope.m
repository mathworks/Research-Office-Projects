classdef SetReachVarZonotope < SetReachVar
    %SETREACHVARZONOTOPE Variable-step SetReach with the shape fixed to a zonotope.
    %
    %   The variable-step twin of SetReachZonotope. Same two lines, same reason:
    %   getProperties is re-declared because the plugin solver registry reads that
    %   static off the registered class name rather than walking up the hierarchy,
    %   so an inherited one is not found.
    %
    %       set_param(mdl, 'SolverType', 'Variable-step');
    %       set_param(mdl, 'Solver', 'SetReachVarZonotope');
    %
    %   See also SETREACHVAR, SETREACHZONOTOPE, REGISTERSETSOLVERS.

    methods
        function k = shapeKind(~)
            k = 'zonotope';
        end
    end

    methods (Static)
        function props = getProperties()
            props = SetReachVar.getProperties();
        end
    end
end
