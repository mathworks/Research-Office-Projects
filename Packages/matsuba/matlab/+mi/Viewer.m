classdef Viewer < handle
%MI.VIEWER Interactive progressive viewer with HG patch preview and Mitsuba overlay.
%   V = MI.VIEWER(SCENE) creates a viewer that shows MATLAB patches for
%   instant interactivity (rotate/zoom/pan), then progressively overlays
%   a Mitsuba render that improves over time. When the user manipulates
%   the camera, the overlay resets and re-renders from the new viewpoint.
%
%   The viewer has two stacked axes:
%     - PatchAxes (bottom): 3D patch objects for instant camera feedback
%     - OverlayAxes (top): 2D image with alpha blending for Mitsuba render
%
%   Properties:
%     Scene          - mi.Scene handle
%     TargetSpp      - Total samples per pixel to accumulate (default 64)
%
%   Methods:
%     stop()         - Stop the progressive render timer
%     resume()       - Resume progressive rendering
%     reset()        - Reset accumulator and re-render from current view
%
%   Example:
%       scene = mi.Scene.build( ...
%           mi.shape.fromMesh(V, F, bsdf=mi.bsdf.diffuse()), ...
%           mi.emitter.constant(), ...
%           mi.sensor.perspective(fov=45));
%       v = mi.Viewer(scene);
%       % Rotate the figure with the mouse — Mitsuba overlay updates
%       % automatically
%
%   See also: mi.show, mi.Scene

    properties (SetAccess = private)
        Scene           % mi.Scene handle
        Figure          % MATLAB figure handle
        PatchAxes       % 3D axes with patch objects
        OverlayAxes     % 2D axes for Mitsuba overlay image
        OverlayImage    % Image object in OverlayAxes
        Accumulator     % Running weighted sum (H x W x 3)
        SampleCount     % Total spp accumulated so far
        RenderTimer     % MATLAB timer for progressive passes
        IsDirty         % Flag: camera moved, restart rendering
        RenderPhase     % Current phase index
        LastCamState    % Cached camera state to detect changes
        FilmSizeParam   % Name of the film size param (e.g. "sensor.film.size")
        FovParam        % Name of the x_fov param (e.g. "sensor.x_fov")
        TransformParam  % Name of the to_world param (e.g. "sensor.to_world")
        LastFigSize     % Cached [W H] figure pixel size for resize detection
    end

    properties
        TargetSpp (1,1) double {mustBePositive} = 64
    end

    methods
        function obj = Viewer(scene, options)
            %VIEWER Construct a progressive viewer for a scene.
            arguments
                scene (1,1) mi.Scene
                options.TargetSpp (1,1) double {mustBePositive} = 64
            end

            obj.Scene = scene;
            obj.TargetSpp = options.TargetSpp;
            obj.SampleCount = 0;
            obj.IsDirty = true;
            obj.RenderPhase = 1;
            obj.Accumulator = [];

            % Extract geometry from description
            desc = scene.description();
            geom = mi.internal.extractGeometry(desc);

            % Discover sensor parameter names by finding the film.size
            % param and deriving the sensor prefix from it
            obj.FilmSizeParam = "";
            obj.FovParam = "";
            obj.TransformParam = "";
            pnames = scene.params();
            idx = find(endsWith(pnames, ".film.size"), 1);
            if ~isempty(idx)
                obj.FilmSizeParam = pnames(idx);
                sensorPrefix = extractBefore(pnames(idx), ".film.size");
                obj.FovParam = sensorPrefix + ".x_fov";
                obj.TransformParam = sensorPrefix + ".to_world";
            end

            % Create figure
            obj.Figure = figure('Name', 'mi.Viewer', ...
                'NumberTitle', 'off', ...
                'Color', [0.15 0.15 0.15], ...
                'SizeChangedFcn', @(~,~) obj.onResize(), ...
                'CloseRequestFcn', @(~,~) obj.onClose());

            % Create 3D patch axes (fills the figure)
            obj.PatchAxes = axes(obj.Figure, ...
                'Units', 'normalized', 'Position', [0 0 1 1], ...
                'Color', [0.15 0.15 0.15], ...
                'XColor', 'none', 'YColor', 'none', 'ZColor', 'none');
            hold(obj.PatchAxes, 'on');
            axis(obj.PatchAxes, 'equal', 'off');
            obj.PatchAxes.Clipping = 'off';

            % Draw patches
            for i = 1:numel(geom)
                patch(obj.PatchAxes, ...
                    'Vertices', geom(i).V, ...
                    'Faces', geom(i).F, ...
                    'FaceColor', geom(i).color, ...
                    'EdgeColor', 'none', ...
                    'FaceLighting', 'gouraud', ...
                    'AmbientStrength', 0.4, ...
                    'DiffuseStrength', 0.6, ...
                    'SpecularStrength', 0.2);
            end

            % Add a light source for the patches
            camlight(obj.PatchAxes, 'headlight');

            % Create overlay axes (on top, for Mitsuba image)
            obj.OverlayAxes = axes(obj.Figure, ...
                'Units', 'normalized', 'Position', [0 0 1 1], ...
                'Visible', 'off', ...
                'HitTest', 'off', ...
                'PickableParts', 'none');
            obj.OverlayAxes.XLim = [0.5 1.5];
            obj.OverlayAxes.YLim = [0.5 1.5];

            % Placeholder 1x1 transparent image
            obj.OverlayImage = image(obj.OverlayAxes, ...
                'CData', zeros(1,1,3), ...
                'AlphaData', 0, ...
                'HitTest', 'off', ...
                'PickableParts', 'none');
            obj.OverlayAxes.XLim = [0.5 1.5];
            obj.OverlayAxes.YLim = [0.5 1.5];
            obj.OverlayAxes.YDir = 'reverse';

            % Sync initial camera from Mitsuba sensor — must happen after
            % patches are drawn so axis limits are established, then lock
            % the camera to manual mode to prevent auto-fitting.
            obj.syncCameraFromScene(desc, geom);
            obj.PatchAxes.CameraPositionMode = 'manual';
            obj.PatchAxes.CameraTargetMode = 'manual';
            obj.PatchAxes.CameraUpVectorMode = 'manual';
            obj.PatchAxes.CameraViewAngleMode = 'manual';

            % Enable 3D rotation
            rotate3d(obj.Figure, 'on');

            % Cache camera state
            obj.LastCamState = obj.getCamState();

            % Start progressive render timer
            obj.RenderTimer = timer( ...
                'ExecutionMode', 'fixedSpacing', ...
                'Period', 0.5, ...
                'BusyMode', 'drop', ...
                'TimerFcn', @(~,~) obj.onTimerTick(), ...
                'ErrorFcn', @(~,e) fprintf(2, 'Viewer timer error: %s\n', e.Data.message));
            start(obj.RenderTimer);
        end

        function stop(obj)
            %STOP Stop the progressive render timer.
            if ~isempty(obj.RenderTimer) && isvalid(obj.RenderTimer)
                stop(obj.RenderTimer);
            end
        end

        function resume(obj)
            %RESUME Resume progressive rendering.
            if ~isempty(obj.RenderTimer) && isvalid(obj.RenderTimer)
                if strcmp(obj.RenderTimer.Running, "off")
                    start(obj.RenderTimer);
                end
            end
        end

        function reset(obj)
            %RESET Reset accumulator and re-render from current viewpoint.
            obj.IsDirty = true;
        end

        function delete(obj)
            %DELETE Clean up timer and figure.
            if ~isempty(obj.RenderTimer) && isvalid(obj.RenderTimer)
                stop(obj.RenderTimer);
                delete(obj.RenderTimer);
            end
        end
    end

    methods (Access = private)
        function onTimerTick(obj)
            %ONTIMERTICK Progressive render callback.
            if ~isvalid(obj) || ~isvalid(obj.Figure)
                return
            end

            % Check if camera moved
            currentState = obj.getCamState();
            if ~isequal(currentState, obj.LastCamState)
                obj.IsDirty = true;
                obj.LastCamState = currentState;
            end

            % If dirty, reset accumulator
            if obj.IsDirty
                obj.Accumulator = [];
                obj.SampleCount = 0;
                obj.RenderPhase = 1;
                obj.IsDirty = false;
                % Set overlay transparent
                if isvalid(obj.OverlayImage)
                    obj.OverlayImage.AlphaData = 0;
                end
                % Sync film resolution to figure pixel size
                obj.syncFilmSize();
            end

            % Already done?
            if obj.SampleCount >= obj.TargetSpp
                return
            end

            % Sync camera to Mitsuba
            obj.syncCameraToScene();

            % Determine spp for this phase — small batches for fast updates
            if obj.RenderPhase == 1
                batchSpp = 1;
            else
                batchSpp = 2;
            end
            batchSpp = min(batchSpp, obj.TargetSpp - obj.SampleCount);

            % Render
            try
                seed = obj.RenderPhase - 1;
                hdr = obj.Scene.render( ...
                    SamplesPerPixel=batchSpp, Seed=seed);
            catch ex
                fprintf(2, 'Viewer render error: %s\n', ex.message);
                return
            end

            % Accumulate
            if isempty(obj.Accumulator)
                obj.Accumulator = hdr * batchSpp;
            else
                obj.Accumulator = obj.Accumulator + hdr * batchSpp;
            end
            obj.SampleCount = obj.SampleCount + batchSpp;

            % Tonemap accumulated image
            accumulated = obj.Accumulator / obj.SampleCount;
            rgb = mi.postprocess(accumulated);

            % Update overlay
            alpha = min(1, sqrt(obj.SampleCount / obj.TargetSpp));
            if isvalid(obj.OverlayImage)
                obj.OverlayImage.CData = rgb;
                obj.OverlayImage.AlphaData = alpha;
                % Fit image to axes
                [h, w, ~] = size(rgb);
                obj.OverlayImage.XData = [0.5, w + 0.5];
                obj.OverlayImage.YData = [0.5, h + 0.5];
                obj.OverlayAxes.XLim = [0.5, w + 0.5];
                obj.OverlayAxes.YLim = [0.5, h + 0.5];
            end

            obj.RenderPhase = obj.RenderPhase + 1;

            % Update title with progress
            pct = min(100, round(100 * obj.SampleCount / obj.TargetSpp));
            if isvalid(obj.Figure)
                obj.Figure.Name = sprintf('mi.Viewer  [%d/%d spp  %d%%]', ...
                    obj.SampleCount, obj.TargetSpp, pct);
            end

            drawnow limitrate;
        end

        function syncCameraFromScene(obj, desc, geom)
            %SYNCCAMERAFROMSCENE Set HG camera from the Mitsuba sensor transform.
            sensorStruct = [];
            fields = fieldnames(desc);
            for i = 1:numel(fields)
                v = desc.(fields{i});
                if isstruct(v) && isfield(v, "category_") ...
                        && strcmp(v.category_, "sensor")
                    sensorStruct = v;
                    break
                end
            end

            if isempty(sensorStruct)
                return
            end

            % Compute scene centroid for orbit target distance
            sceneCentroid = [0 0 0];
            if ~isempty(geom)
                allV = vertcat(geom.V);
                sceneCentroid = mean(allV, 1);
            end

            % Extract to_world
            if isfield(sensorStruct, "to_world") && ~isempty(sensorStruct.to_world)
                T = sensorStruct.to_world;
                origin = T(1:3, 4)';
                dir = T(1:3, 3)';
                up = T(1:3, 2)';

                % Project scene centroid onto view direction to get a
                % meaningful orbit target distance (not just 1 unit away)
                toCenter = sceneCentroid - origin;
                dist = dot(toCenter, dir);
                if dist < 0.1
                    dist = norm(toCenter);
                end
                if dist < 0.1
                    dist = 1;
                end
                target = origin + dir * dist;

                campos(obj.PatchAxes, origin);
                camtarget(obj.PatchAxes, target);
                camup(obj.PatchAxes, up);
            end

            % Extract FOV
            if isfield(sensorStruct, "fov") && ~isempty(sensorStruct.fov)
                fov = double(sensorStruct.fov);
                % Mitsuba default fov_axis is "x", MATLAB camva is vertical.
                % Convert horizontal FOV to vertical using film aspect ratio.
                filmW = 512; filmH = 512;
                if isfield(sensorStruct, "film") && isstruct(sensorStruct.film)
                    film = sensorStruct.film;
                    if isfield(film, "width"), filmW = film.width; end
                    if isfield(film, "height"), filmH = film.height; end
                end
                aspect = filmW / filmH;
                fovAxis = "x";
                if isfield(sensorStruct, "fov_axis") && ~isempty(sensorStruct.fov_axis)
                    fovAxis = string(sensorStruct.fov_axis);
                end
                if fovAxis == "x"
                    % Convert horizontal to vertical
                    vfov = 2 * atand(tand(fov/2) / aspect);
                else
                    vfov = fov;
                end
                camva(obj.PatchAxes, vfov);
            end
        end

        function syncFilmSize(obj)
            %SYNCFILMSIZE Update Mitsuba film resolution to match figure pixel size.
            if obj.FilmSizeParam == ""
                return
            end
            if ~isvalid(obj.Figure)
                return
            end
            oldUnits = obj.Figure.Units;
            obj.Figure.Units = 'pixels';
            pos = obj.Figure.Position;
            obj.Figure.Units = oldUnits;
            W = max(64, round(pos(3)));
            H = max(64, round(pos(4)));
            % Cap resolution for interactive speed
            maxDim = 512;
            if max(W, H) > maxDim
                scale = maxDim / max(W, H);
                W = max(64, round(W * scale));
                H = max(64, round(H * scale));
            end
            figSize = [W H];
            if isequal(figSize, obj.LastFigSize)
                return
            end
            obj.LastFigSize = figSize;
            try
                obj.Scene.setParam(obj.FilmSizeParam, figSize);
            catch
                % Film size may not be settable on some scenes
            end
        end

        function syncCameraToScene(obj)
            %SYNCCAMERATOSCENE Push HG camera state to Mitsuba sensor.
            ax = obj.PatchAxes;
            origin = campos(ax);
            target = camtarget(ax);
            up = camup(ax);

            T = mi.Transform.lookAt(origin, target, up);
            if obj.TransformParam ~= ""
                try
                    obj.Scene.setTransform(obj.TransformParam, T);
                catch
                    % Scene may not be built yet on first tick
                end
            end

            % Sync FOV: MATLAB camva is vertical FOV, Mitsuba x_fov is horizontal
            if obj.FovParam ~= "" && ~isempty(obj.LastFigSize)
                vfov = camva(ax);
                aspect = obj.LastFigSize(1) / obj.LastFigSize(2);
                xfov = 2 * atand(tand(vfov/2) * aspect);
                try
                    obj.Scene.setParam(obj.FovParam, xfov);
                catch
                end
            end
        end

        function state = getCamState(obj)
            %GETCAMSTATE Get a snapshot of HG camera state for change detection.
            ax = obj.PatchAxes;
            state = struct( ...
                'Position', campos(ax), ...
                'Target', camtarget(ax), ...
                'Up', camup(ax), ...
                'VA', camva(ax));
        end

        function onResize(obj)
            %ONRESIZE Keep overlay axes aligned with patch axes and detect size changes.
            if ~isempty(obj.OverlayAxes) && isvalid(obj.OverlayAxes)
                obj.OverlayAxes.Position = [0 0 1 1];
            end
            if ~isempty(obj.PatchAxes) && isvalid(obj.PatchAxes)
                obj.PatchAxes.Position = [0 0 1 1];
            end
            % Mark dirty so next tick re-renders at new resolution
            obj.IsDirty = true;
        end

        function onClose(obj)
            %ONCLOSE Clean up when figure closes.
            if ~isempty(obj.RenderTimer) && isvalid(obj.RenderTimer)
                stop(obj.RenderTimer);
                delete(obj.RenderTimer);
                obj.RenderTimer = [];
            end
            fig = obj.Figure;
            obj.Figure = [];
            if ~isempty(fig) && isvalid(fig)
                fig.CloseRequestFcn = "";
                delete(fig);
            end
        end
    end
end
