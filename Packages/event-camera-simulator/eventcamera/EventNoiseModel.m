classdef EventNoiseModel < matlab.System
    % EventNoiseModel  Post-processing noise model for DVS event streams.
    %
    %   Sits downstream of EventCamera and adds realistic noise effects:
    %     - Background activity (random spurious events)
    %     - Hot pixels (fixed pixels that fire constantly)
    %     - Timestamp jitter (Gaussian noise on event timestamps)
    %     - Leak events (slow reference drift in static regions)
    %
    %   Usage:
    %     noise = EventNoiseModel('BackgroundRate', 100, 'NumHotPixels', 5);
    %     noisyEvents = noise(events, timestamp);

    properties (Nontunable)
        % Image dimensions (must match EventCamera output)
        ImageHeight  (1,1) double {mustBePositive, mustBeInteger} = 240
        ImageWidth   (1,1) double {mustBePositive, mustBeInteger} = 320

        % Background activity: random events per second across the whole sensor
        BackgroundRate (1,1) double {mustBeNonnegative} = 0

        % Hot pixels: number of pixels that fire at a high constant rate
        NumHotPixels (1,1) double {mustBeNonnegative, mustBeInteger} = 0
        HotPixelRate (1,1) double {mustBePositive} = 1000  % events/s per hot pixel

        % Timestamp jitter: std dev of Gaussian noise added to timestamps (seconds)
        TimestampJitter (1,1) double {mustBeNonnegative} = 0

        % Leak events: rate of spurious events from reference drift (events/s/pixel)
        LeakRate (1,1) double {mustBeNonnegative} = 0

        % Maximum output events per step (for Simulink sizing)
        MaxEventsPerStep (1,1) double {mustBePositive, mustBeInteger} = 500000
    end

    properties (Access = private)
        HotPixelLocations   % Nx2 matrix of [x, y] coordinates
        PreviousTimestamp
    end

    methods
        function obj = EventNoiseModel(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj, ~, ~)
            obj.PreviousTimestamp = 0;

            % Select random hot pixel locations
            if obj.NumHotPixels > 0
                nPix = obj.ImageHeight * obj.ImageWidth;
                hotIdx = randperm(nPix, min(obj.NumHotPixels, nPix));
                [hotY, hotX] = ind2sub([obj.ImageHeight, obj.ImageWidth], hotIdx);
                obj.HotPixelLocations = [hotX(:), hotY(:)];
            else
                obj.HotPixelLocations = zeros(0, 2);
            end
        end

        function noisyEvents = stepImpl(obj, events, timestamp)
            dt = timestamp - obj.PreviousTimestamp;
            obj.PreviousTimestamp = timestamp;

            if dt <= 0
                noisyEvents = events;
                return;
            end

            tStart = timestamp - dt;
            noiseEvents = zeros(0, 4);

            % --- Background activity noise ---
            if obj.BackgroundRate > 0
                nBg = poissrnd(obj.BackgroundRate * dt);
                if nBg > 0
                    bgX = randi(obj.ImageWidth, nBg, 1);
                    bgY = randi(obj.ImageHeight, nBg, 1);
                    bgT = tStart + rand(nBg, 1) * dt;
                    bgP = 2 * (rand(nBg, 1) > 0.5) - 1;  % random polarity
                    noiseEvents = [noiseEvents; bgX, bgY, bgT, bgP];                end
            end

            % --- Hot pixels ---
            if obj.NumHotPixels > 0 && ~isempty(obj.HotPixelLocations)
                for h = 1:size(obj.HotPixelLocations, 1)
                    nHot = poissrnd(obj.HotPixelRate * dt);
                    if nHot > 0
                        hx = obj.HotPixelLocations(h, 1);
                        hy = obj.HotPixelLocations(h, 2);
                        hotT = tStart + rand(nHot, 1) * dt;
                        hotP = 2 * (rand(nHot, 1) > 0.5) - 1;
                        noiseEvents = [noiseEvents; ...
                            repmat([hx, hy], nHot, 1), hotT, hotP];                    end
                end
            end

            % --- Leak events ---
            if obj.LeakRate > 0
                nLeak = poissrnd(obj.LeakRate * dt * obj.ImageHeight * obj.ImageWidth);
                if nLeak > 0
                    lkX = randi(obj.ImageWidth, nLeak, 1);
                    lkY = randi(obj.ImageHeight, nLeak, 1);
                    lkT = tStart + rand(nLeak, 1) * dt;
                    lkP = 2 * (rand(nLeak, 1) > 0.5) - 1;
                    noiseEvents = [noiseEvents; lkX, lkY, lkT, lkP];                end
            end

            % Merge signal events with noise events
            noisyEvents = [events; noiseEvents];

            % --- Timestamp jitter ---
            if obj.TimestampJitter > 0 && ~isempty(noisyEvents)
                jitter = obj.TimestampJitter * randn(size(noisyEvents, 1), 1);
                noisyEvents(:, 3) = noisyEvents(:, 3) + jitter;
                % Clamp to frame interval
                noisyEvents(:, 3) = max(tStart, min(timestamp, noisyEvents(:, 3)));
            end

            % Sort by timestamp and cap
            if ~isempty(noisyEvents)
                [~, sortIdx] = sort(noisyEvents(:, 3));
                noisyEvents = noisyEvents(sortIdx, :);
                if size(noisyEvents, 1) > obj.MaxEventsPerStep
                    noisyEvents = noisyEvents(1:obj.MaxEventsPerStep, :);
                end
            else
                noisyEvents = zeros(0, 4);
            end
        end

        function resetImpl(obj)
            obj.PreviousTimestamp = 0;
        end

        % --- Simulink interface ---
        function flag = isOutputFixedSizeImpl(~)
            flag = false;
        end

        function sz = getOutputSizeImpl(obj)
            sz = [obj.MaxEventsPerStep, 4];
        end

        function dt = getOutputDataTypeImpl(~)
            dt = 'double';
        end

        function flag = isOutputComplexImpl(~)
            flag = false;
        end

        function flag = isInputSizeMutableImpl(~, idx)
            flag = (idx == 1);  % events input is variable-size
        end

        function num = getNumInputsImpl(~)
            num = 2;  % events, timestamp
        end

        function num = getNumOutputsImpl(~)
            num = 1;
        end
    end
end
