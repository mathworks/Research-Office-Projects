classdef SetReachZonotope < SetReach
    %SETREACHZONOTOPE SetReach with the shape fixed to a zonotope.
    %
    %   One of four thin subclasses whose only job is to put the set
    %   representation into the Simulink Solver dropdown. Register them all with
    %   registerSetSolvers() and the shape becomes a Configuration Parameters
    %   choice like any other solver setting:
    %
    %       set_param(mdl, 'Solver', 'SetReachZonotope');
    %
    %   getProperties is re-declared deliberately. It is a static method, and the
    %   plugin solver registry reads it off the registered class name rather than
    %   walking up the hierarchy, so an inherited one is not found. Two lines per
    %   subclass buys the dropdown entry.
    %
    %   See also SETREACH, REGISTERSETSOLVERS, ZONOTOPEREP.

    methods
        function k = shapeKind(~)
            k = 'zonotope';
        end
    end

    methods (Static)
        function props = getProperties()
            props = SetReach.getProperties();
        end
    end
end
