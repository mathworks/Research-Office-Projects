classdef SetReachSupport < SetReach
    %SETREACHSUPPORT SetReach carrying only the accumulated map (LGG / SpaceEx).
    %
    %   Phi <- Phi_h*Phi, and no set is ever formed. Directional queries are
    %   exact and cost one product against the initial set regardless of elapsed
    %   steps; drawing the set requires sampling directions, so the plot is an outer
    %   polygon that tightens with the sample count.
    %
    %   See also SETREACH, SUPPORTREP, REGISTERSETSOLVERS.

    methods
        function k = shapeKind(~)
            k = 'support';
        end
    end

    methods (Static)
        function props = getProperties()
            props = SetReach.getProperties();
        end
    end
end
