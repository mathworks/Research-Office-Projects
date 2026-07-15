classdef Transform
%MI.TRANSFORM Static utility class for 4x4 transform matrices.
%   Produces standard 4x4 homogeneous transformation matrices.
%   Angles are in degrees (consistent with MATLAB's rotx/roty/rotz).
%
%   Examples:
%       T = mi.Transform.translate([1 0 0]);
%       T = mi.Transform.scale(2);
%       T = mi.Transform.rotateX(90);
%       T = mi.Transform.rotate([0 1 0], 45);
%       T = mi.Transform.lookAt([0 1 5], [0 1 0], [0 1 0]);

    methods (Static)
        function T = translate(v)
            %TRANSLATE Create a translation matrix.
            arguments
                v (1,3) double
            end
            T = eye(4);
            T(1:3, 4) = v(:);
        end

        function T = scale(s)
            %SCALE Create a scaling matrix.
            %   T = MI.TRANSFORM.SCALE(S) with scalar S scales uniformly.
            %   T = MI.TRANSFORM.SCALE(S) with 3-element S scales per axis.
            if isscalar(s)
                s = [s s s];
            end
            T = diag([double(s(1)), double(s(2)), double(s(3)), 1.0]);
        end

        function T = rotateX(angleDeg)
            %ROTATEX Rotation around the X axis.
            %   T = MI.TRANSFORM.ROTATEX(ANGLE) rotates by ANGLE degrees.
            arguments
                angleDeg (1,1) double
            end
            T = mi.Transform.rotate([1 0 0], angleDeg);
        end

        function T = rotateY(angleDeg)
            %ROTATEY Rotation around the Y axis.
            %   T = MI.TRANSFORM.ROTATEY(ANGLE) rotates by ANGLE degrees.
            arguments
                angleDeg (1,1) double
            end
            T = mi.Transform.rotate([0 1 0], angleDeg);
        end

        function T = rotateZ(angleDeg)
            %ROTATEZ Rotation around the Z axis.
            %   T = MI.TRANSFORM.ROTATEZ(ANGLE) rotates by ANGLE degrees.
            arguments
                angleDeg (1,1) double
            end
            T = mi.Transform.rotate([0 0 1], angleDeg);
        end

        function T = rotate(axis, angleDeg)
            %ROTATE Rotation around an arbitrary axis (Rodrigues' formula).
            %   T = MI.TRANSFORM.ROTATE(AXIS, ANGLE) rotates by ANGLE
            %   degrees around the unit direction AXIS.
            arguments
                axis (1,3) double {mustBeNonzeroNorm}
                angleDeg (1,1) double
            end
            axis = axis / norm(axis);
            c = cosd(angleDeg);
            s = sind(angleDeg);
            t = 1 - c;
            x = axis(1); y = axis(2); z = axis(3);
            R = [t*x*x+c,   t*x*y-s*z, t*x*z+s*y;
                 t*x*y+s*z, t*y*y+c,   t*y*z-s*x;
                 t*x*z-s*y, t*y*z+s*x, t*z*z+c  ];
            T = eye(4);
            T(1:3, 1:3) = R;
        end

        function T = lookAt(origin, target, up)
            %LOOKAT Create a look-at transform matching Mitsuba's convention.
            %   Camera looks from ORIGIN toward TARGET with UP orientation.
            arguments
                origin (1,3) double
                target (1,3) double
                up (1,3) double
            end
            dir = target - origin;
            dir = dir / norm(dir);
            left = cross(up, dir);
            left = left / norm(left);
            newUp = cross(dir, left);

            T = eye(4);
            T(1:3, 1) = left(:);
            T(1:3, 2) = newUp(:);
            T(1:3, 3) = dir(:);
            T(1:3, 4) = origin(:);
        end
    end
end

function mustBeNonzeroNorm(v)
    if norm(v) < eps
        error("mi:Transform:ZeroAxis", "Rotation axis must be non-zero.");
    end
end
