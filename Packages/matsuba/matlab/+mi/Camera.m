classdef Camera < handle
%MI.CAMERA Convenience wrapper for scene camera manipulation.
%   This class provides a MATLAB-native interface for camera control.
%   It is created automatically by mi.Scene and should not be
%   instantiated directly.
%
%   Example:
%       scene.camera.lookAt([0 1 5], [0 1 0], [0 1 0]);

    properties (SetAccess = private)
        SceneRef        % Reference to parent mi.Scene
        TransformParam  string = ""  % Discovered sensor transform param name
    end

    methods
        function obj = Camera(scene)
            %CAMERA Construct a Camera bound to a Scene.
            obj.SceneRef = scene;
        end

        function lookAt(obj, origin, target, up)
            %LOOKAT Set camera using look-at parameters.
            %   CAM.LOOKAT(ORIGIN, TARGET, UP) sets the camera transform.
            %   Each argument is a 3-element numeric vector.
            arguments
                obj
                origin (1,3) double
                target (1,3) double
                up (1,3) double
            end
            T = mi.Transform.lookAt(origin, target, up);
            obj.SceneRef.setTransform(obj.discoverTransformParam(), T);
        end
    end

    methods (Access = private)
        function param = discoverTransformParam(obj)
            %DISCOVERTRANSFORMPARAM Find the sensor transform parameter name.
            %   Searches scene params for a name ending in ".to_world" that
            %   corresponds to the sensor. Falls back to "sensor.to_world".
            if obj.TransformParam ~= ""
                param = obj.TransformParam;
                return
            end
            try
                pnames = obj.SceneRef.params();
                idx = find(endsWith(pnames, ".to_world"), 1);
                if ~isempty(idx)
                    obj.TransformParam = pnames(idx);
                end
            catch
                % Scene not yet built — fall through to default
            end
            if obj.TransformParam == ""
                obj.TransformParam = "sensor.to_world";
            end
            param = obj.TransformParam;
        end
    end
end
