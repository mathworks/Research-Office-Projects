classdef EventVisualizer < matlab.System
    % EventVisualizer  Real-time event camera visualization in one figure.
    %
    %   Inputs: events (Nx4), timestamp (scalar), cameraImage (HxWx3 uint8)
    %   Displays three subplots updated in real-time:
    %     1. Raw camera image
    %     2. Polarity frame — ON=blue, OFF=red, background=gray
    %     3. Time surface — exponential decay of most recent event time

    properties (Nontunable)
        ImageHeight  (1,1) double {mustBePositive, mustBeInteger} = 240
        ImageWidth   (1,1) double {mustBePositive, mustBeInteger} = 320
        TimeSurfaceDecay (1,1) double {mustBePositive} = 0.3  % seconds
    end

    properties (Access = private)
        LastEventTime
        Figure
        RawImageHandle
        PolarityImageHandle
        TimeSurfaceImageHandle
    end

    methods
        function obj = EventVisualizer(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj, ~, ~, ~)
            H = obj.ImageHeight;
            W = obj.ImageWidth;
            obj.LastEventTime = -inf(H, W);

            obj.Figure = figure('Name', 'Event Camera Viewer', ...
                'Position', [50 200 W*3*1.5 H*1.5+60], 'NumberTitle', 'off');

            subplot(1,3,1);
            obj.RawImageHandle = imshow(zeros(H, W, 3, 'uint8'));
            title('Raw Camera');

            subplot(1,3,2);
            obj.PolarityImageHandle = imshow(128 * ones(H, W, 3, 'uint8'));
            title('Polarity Frame');

            subplot(1,3,3);
            obj.TimeSurfaceImageHandle = imshow(zeros(H, W, 'uint8'));
            title('Time Surface');
            colormap(gca, 'hot');
        end

        function stepImpl(obj, events, timestamp, cameraImage)
            H = obj.ImageHeight;
            W = obj.ImageWidth;

            % --- Raw camera image ---
            if isvalid(obj.RawImageHandle)
                obj.RawImageHandle.CData = cameraImage;
            end

            % --- Polarity frame ---
            polarityFrame = 128 * ones(H, W, 3, 'uint8');
            nEvents = size(events, 1);
            if nEvents > 0 && size(events, 2) >= 4
                for i = 1:nEvents
                    x = round(events(i, 1));
                    y = round(events(i, 2));
                    if x >= 1 && x <= W && y >= 1 && y <= H
                        if events(i, 4) > 0
                            polarityFrame(y, x, :) = uint8([0 0 255]);
                        else
                            polarityFrame(y, x, :) = uint8([255 0 0]);
                        end
                        obj.LastEventTime(y, x) = events(i, 3);
                    end
                end
            end

            if isvalid(obj.PolarityImageHandle)
                obj.PolarityImageHandle.CData = polarityFrame;
            end

            % --- Time surface ---
            age = timestamp - obj.LastEventTime;
            intensity = exp(-age / obj.TimeSurfaceDecay);
            intensity = max(0, min(1, intensity));
            tsImage = uint8(255 * intensity);

            if isvalid(obj.TimeSurfaceImageHandle)
                obj.TimeSurfaceImageHandle.CData = tsImage;
            end

            drawnow limitrate;
        end

        function resetImpl(obj)
            obj.LastEventTime = -inf(obj.ImageHeight, obj.ImageWidth);
        end

        function releaseImpl(~)
            % Keep figure open after simulation ends so user can inspect
        end

        % --- Simulink interface ---
        function flag = isInputSizeMutableImpl(~, idx)
            flag = (idx == 1);  % only events input is variable-size
        end

        function num = getNumInputsImpl(~)
            num = 3;  % events, timestamp, cameraImage
        end

        function num = getNumOutputsImpl(~)
            num = 0;
        end
    end
end
