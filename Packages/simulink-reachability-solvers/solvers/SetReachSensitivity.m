classdef SetReachSensitivity < SetReach
    %SETREACHSENSITIVITY SetReach carrying the flow Jacobian S = d(x_t)/d(x_0).
    %
    %   S <- Phi*S from S(0) = I: the HSCC'09 sensitivity matrix. Numerically this
    %   is the SAME recursion as the zonotope, agreeing to machine precision, and
    %   that identity is the point rather than a redundancy. What this entry adds
    %   is the readouts a set representation throws away: ||S||, the
    %   per-initial-state column gains, and the singular values of the flow map.
    %   Those are also what the HSCC'09 error indicator needs.
    %
    %   See also SETREACH, SENSITIVITYREP, REGISTERSETSOLVERS.

    methods
        function k = shapeKind(~)
            k = 'sensitivity';
        end
    end

    methods (Static)
        function props = getProperties()
            props = SetReach.getProperties();
        end
    end
end
