classdef SetReachEllipsoid < SetReach
    %SETREACHELLIPSOID SetReach with the shape fixed to an ellipsoid.
    %
    %   Q <- Phi*Q*Phi'. The payload is n-by-n and stays n-by-n forever, which is
    %   the reason to choose this one.
    %
    %   The initial set is NOT the same as the other three shapes get from the same
    %   half-widths: an ellipsoid must either inscribe or circumscribe the box.
    %   SetReach.setFit('circumscribed') makes it the box's enclosure, which is
    %   the apples-to-apples comparison; the default 'inscribed' is a strictly
    %   smaller initial condition and its tighter numbers are not a representation
    %   win. See EllipsoidRep.
    %
    %   See also SETREACH, ELLIPSOIDREP, REGISTERSETSOLVERS.

    methods
        function k = shapeKind(~)
            k = 'ellipsoid';
        end
    end

    methods (Static)
        function props = getProperties()
            props = SetReach.getProperties();
        end
    end
end
