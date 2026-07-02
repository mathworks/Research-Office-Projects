classdef EventCamera < matlab.System
    % EventCamera  DVS event camera simulator.
    %
    %   Converts a sequence of intensity frames into asynchronous events
    %   [x, y, timestamp, polarity] by analytically computing threshold
    %   crossings in log-intensity space.
    %
    %   Usage:
    %     cam = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2);
    %     events = cam(frame, timestamp);

    % Public properties (Nontunable)
    properties (Nontunable)
        ContrastThresholdOn  (1,1) double {mustBePositive} = 0.2
        ContrastThresholdOff (1,1) double {mustBePositive} = 0.2
        RefractoryPeriod     (1,1) double {mustBeNonnegative} = 0
        ThresholdMismatch    (1,1) double {mustBeNonnegative} = 0
        BandwidthHz          (1,1) double {mustBePositive} = Inf
        Latency              (1,1) double {mustBeNonnegative} = 100e-6
        LogEpsilon           (1,1) double {mustBePositive} = 1e-3
        MaxEventsPerFrame    (1,1) double {mustBePositive, mustBeInteger} = 500000
    end

    methods
        function obj = EventCamera(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    % Private state
    properties (Access = private)
        LogIntensityReference
        PreviousLogIntensity
        FilteredLogIntensity  % low-pass filtered version for bandwidth limiting
        PreviousTimestamp
        LastEventTimestamp
        IsFirstFrame
        PerPixelThresholdOn
        PerPixelThresholdOff
        ImageSize
    end

    methods (Access = protected)
        function setupImpl(obj, frame, ~)
            if ndims(frame) == 3 && size(frame, 3) == 3
                gray = rgb2gray(im2double(frame));
            else
                gray = im2double(frame);
            end
            [M, N] = size(gray);
            obj.ImageSize = [M, N];
            obj.IsFirstFrame = true;
            obj.LogIntensityReference = zeros(M, N);
            obj.PreviousLogIntensity  = zeros(M, N);
            obj.FilteredLogIntensity  = zeros(M, N);
            obj.PreviousTimestamp      = 0;
            obj.LastEventTimestamp      = -inf(M, N);

            if obj.ThresholdMismatch > 0
                rng('default');
                obj.PerPixelThresholdOn  = obj.ContrastThresholdOn  + obj.ThresholdMismatch * randn(M, N);
                obj.PerPixelThresholdOff = obj.ContrastThresholdOff + obj.ThresholdMismatch * randn(M, N);
                obj.PerPixelThresholdOn  = max(obj.PerPixelThresholdOn, 1e-6);
                obj.PerPixelThresholdOff = max(obj.PerPixelThresholdOff, 1e-6);
            else
                obj.PerPixelThresholdOn  = obj.ContrastThresholdOn  * ones(M, N);
                obj.PerPixelThresholdOff = obj.ContrastThresholdOff * ones(M, N);
            end
        end

        function events = stepImpl(obj, frame, timestamp)
            % Convert frame to log-intensity
            if ndims(frame) == 3 && size(frame, 3) == 3
                gray = rgb2gray(im2double(frame));
            else
                gray = im2double(frame);
            end
            logI = log(max(gray, obj.LogEpsilon));

            % First frame: store reference and return empty
            if obj.IsFirstFrame
                obj.LogIntensityReference = logI;
                obj.PreviousLogIntensity  = logI;
                obj.FilteredLogIntensity  = logI;
                obj.PreviousTimestamp     = timestamp;
                obj.IsFirstFrame = false;
                events = zeros(0, 4);
                return;
            end

            dt = timestamp - obj.PreviousTimestamp;

            % Bandwidth limiting: intensity-dependent first-order low-pass filter
            % Bright pixels respond faster (more photocurrent), dark pixels lag
            if isfinite(obj.BandwidthHz) && dt > 0
                tau = 1 / (2 * pi * obj.BandwidthHz);
                inten01 = gray / max(gray(:));
                alpha = min(inten01 * (dt / tau), 1);
                logI = obj.FilteredLogIntensity + alpha .* (logI - obj.FilteredLogIntensity);
            end
            obj.FilteredLogIntensity = logI;

            % Compute event counts from reference
            delta = logI - obj.LogIntensityReference;
            numEventsOn  = floor(max(delta, 0)  ./ obj.PerPixelThresholdOn);
            numEventsOff = floor(max(-delta, 0) ./ obj.PerPixelThresholdOff);
            totalEvents  = numEventsOn + numEventsOff;

            maxK = max(totalEvents(:));
            if maxK == 0
                obj.PreviousLogIntensity = logI;
                obj.PreviousTimestamp    = timestamp;
                events = zeros(0, 4);
                return;
            end

            % Compute interpolated timestamps for each event ordinal
            allX = [];
            allY = [];
            allT = [];
            allP = [];

            for k = 1:maxK
                % ON events at ordinal k
                maskOn = (k <= numEventsOn);
                if any(maskOn(:))
                    [rows, cols] = find(maskOn);
                    threshChange = k * obj.PerPixelThresholdOn;
                    targetLog = obj.LogIntensityReference + threshChange;
                    idx = sub2ind(obj.ImageSize, rows, cols);
                    prevL = obj.PreviousLogIntensity(idx);
                    currL = logI(idx);
                    diffL = currL - prevL;
                    targetL = targetLog(idx);
                    frac = (targetL - prevL) ./ diffL;
                    frac(diffL == 0) = 0.5;
                    frac = max(0, min(1, frac));
                    tk = obj.PreviousTimestamp + dt * frac;

                    allX = [allX; cols];
                    allY = [allY; rows];
                    allT = [allT; tk];
                    allP = [allP; ones(numel(rows), 1)];
                end

                % OFF events at ordinal k
                maskOff = (k <= numEventsOff);
                if any(maskOff(:))
                    [rows, cols] = find(maskOff);
                    threshChange = k * obj.PerPixelThresholdOff;
                    targetLog = obj.LogIntensityReference - threshChange;
                    idx = sub2ind(obj.ImageSize, rows, cols);
                    prevL = obj.PreviousLogIntensity(idx);
                    currL = logI(idx);
                    diffL = currL - prevL;
                    targetL = targetLog(idx);
                    frac = (targetL - prevL) ./ diffL;
                    frac(diffL == 0) = 0.5;
                    frac = max(0, min(1, frac));
                    tk = obj.PreviousTimestamp + dt * frac;

                    allX = [allX; cols];
                    allY = [allY; rows];
                    allT = [allT; tk];
                    allP = [allP; -ones(numel(rows), 1)];
                end
            end

            % Update reference: staircase
            obj.LogIntensityReference = obj.LogIntensityReference ...
                + numEventsOn  .* obj.PerPixelThresholdOn ...
                - numEventsOff .* obj.PerPixelThresholdOff;

            % Update previous frame state
            obj.PreviousLogIntensity = logI;
            obj.PreviousTimestamp    = timestamp;

            % Apply pixel latency (pure output delay)
            if obj.Latency > 0 && ~isempty(allT)
                allT = allT + obj.Latency;
            end

            % Sort by timestamp and cap
            if isempty(allX)
                events = zeros(0, 4);
            else
                events = [allX, allY, allT, allP];
                [~, sortIdx] = sort(events(:, 3));
                events = events(sortIdx, :);
                if size(events, 1) > obj.MaxEventsPerFrame
                    events = events(1:obj.MaxEventsPerFrame, :);
                end
            end

            % Refractory period: suppress events at pixels that fired too recently
            if obj.RefractoryPeriod > 0 && ~isempty(events)
                keep = true(size(events, 1), 1);
                for i = 1:size(events, 1)
                    px = events(i, 1);
                    py = events(i, 2);
                    et = events(i, 3);
                    if (et - obj.LastEventTimestamp(py, px)) < obj.RefractoryPeriod
                        keep(i) = false;
                    else
                        obj.LastEventTimestamp(py, px) = et;
                    end
                end
                events = events(keep, :);
            elseif ~isempty(events)
                % Update last event timestamps even without refractory period
                for i = 1:size(events, 1)
                    obj.LastEventTimestamp(events(i,2), events(i,1)) = events(i,3);
                end
            end
        end

        function resetImpl(obj)
            obj.IsFirstFrame = true;
            if ~isempty(obj.ImageSize)
                M = obj.ImageSize(1);
                N = obj.ImageSize(2);
                obj.LogIntensityReference = zeros(M, N);
                obj.PreviousLogIntensity  = zeros(M, N);
                obj.FilteredLogIntensity  = zeros(M, N);
                obj.LastEventTimestamp     = -inf(M, N);
            end
            obj.PreviousTimestamp = 0;
        end

        function validateInputsImpl(~, frame, timestamp)
            validateattributes(frame, {'numeric'}, {'nonempty', 'real', 'nonsparse'}, '', 'frame');
            validateattributes(timestamp, {'numeric'}, {'scalar', 'real', 'nonnegative'}, '', 'timestamp');
        end

        % Simulink interface methods
        function flag = isOutputFixedSizeImpl(~)
            flag = false;
        end

        function sz = getOutputSizeImpl(obj)
            sz = [obj.MaxEventsPerFrame, 4];
        end

        function dt = getOutputDataTypeImpl(~)
            dt = 'double';
        end

        function flag = isOutputComplexImpl(~)
            flag = false;
        end

        function num = getNumInputsImpl(~)
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 1;
        end
    end
end
