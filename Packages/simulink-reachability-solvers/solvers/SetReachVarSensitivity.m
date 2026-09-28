classdef SetReachVarSensitivity < SetReachVar
    %SETREACHVARSENSITIVITY Variable-step SetReach carrying a sensitivity matrix (HSCC'09).
    %
    %   See SETREACHVARZONOTOPE for why getProperties is re-declared.
    %
    %   S <- Phi*S is the discrete form of the variational equation
    %   Sdot = A(t,c(t))*S linearised about the centre trajectory, which is the
    %   object all four representations actually propagate -- this one just makes
    %   that visible. It is also the ingredient the HSCC'09 expansion-error
    %   indicator needs.
    %
    %   See also SETREACHVAR, SETREACHSENSITIVITY, REGISTERSETSOLVERS.

    methods
        function k = shapeKind(~)
            k = 'sensitivity';
        end
    end

    methods (Static)
        function props = getProperties()
            props = SetReachVar.getProperties();
        end
    end
end
