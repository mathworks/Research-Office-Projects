classdef tEventCamera < matlab.unittest.TestCase

    methods (Test)
        function testFirstFrameReturnsEmpty(testCase)
            cam = EventCamera();
            frame = rand(10, 10);
            events = cam(frame, 0.0);
            testCase.verifyEmpty(events);
            testCase.verifySize(events, [0, 4]);
        end

        function testConstantSceneZeroEvents(testCase)
            cam = EventCamera();
            frame = 0.5 * ones(10, 10);
            cam(frame, 0.0);       % first frame
            events = cam(frame, 1.0); % same frame
            testCase.verifyEmpty(events);
        end

        function testSinglePixelOnEvents(testCase)
            % 1x1 pixel: brightness exp(0)=1 -> exp(0.5)~1.6487
            % log-intensity change = 0.5, C_on = 0.2
            % Expected ON events: floor(0.5/0.2) = 2
            cam = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2, ...
                'Latency', 0);
            cam(exp(0) * ones(1,1), 0.0);  % ref = log(1) = 0
            events = cam(exp(0.5) * ones(1,1), 1.0);

            testCase.verifySize(events, [2, 4]);
            testCase.verifyEqual(events(:,4), [1; 1]); % both ON

            % Verify timestamps: event k at t = k*C / delta * dt
            % delta (prev to curr) = 0.5, C = 0.2
            % t1 = 0 + 1*(0.2/0.5)*1 = 0.4
            % t2 = 0 + 1*(0.4/0.5)*1 = 0.8
            testCase.verifyEqual(events(:,3), [0.4; 0.8], 'AbsTol', 1e-10);
        end

        function testSinglePixelOffEvents(testCase)
            % 1x1 pixel: brightness exp(0)=1 -> exp(-0.5)
            % log-intensity change = -0.5, C_off = 0.2
            % Expected OFF events: floor(0.5/0.2) = 2
            cam = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2, ...
                'Latency', 0);
            cam(exp(0) * ones(1,1), 0.0);
            events = cam(exp(-0.5) * ones(1,1), 1.0);

            testCase.verifySize(events, [2, 4]);
            testCase.verifyEqual(events(:,4), [-1; -1]); % both OFF
            testCase.verifyEqual(events(:,3), [0.4; 0.8], 'AbsTol', 1e-10);
        end

        function testAsymmetricThresholds(testCase)
            % C_on=0.1, C_off=0.3, delta=+0.5
            % ON events: floor(0.5/0.1) = 5
            cam = EventCamera('ContrastThresholdOn', 0.1, 'ContrastThresholdOff', 0.3);
            cam(exp(0) * ones(1,1), 0.0);
            events = cam(exp(0.5) * ones(1,1), 1.0);
            testCase.verifySize(events, [5, 4]);
            testCase.verifyTrue(all(events(:,4) == 1));

            % Now decrease by 0.5 from the updated reference
            % Reference was updated: 0 + 5*0.1 = 0.5, current logI = 0.5
            % So ref = 0.5. Go to exp(0) -> logI = 0, delta = -0.5
            % OFF events: floor(0.5/0.3) = 1
            cam2 = EventCamera('ContrastThresholdOn', 0.1, 'ContrastThresholdOff', 0.3);
            cam2(exp(0) * ones(1,1), 0.0);
            cam2(exp(0.5) * ones(1,1), 1.0); % sets ref to 0.5
            events2 = cam2(exp(0) * ones(1,1), 2.0);
            testCase.verifySize(events2, [1, 4]);
            testCase.verifyEqual(events2(1,4), -1);
        end

        function testStaircaseReferenceUpdate(testCase)
            % 4-frame sequence on a 1x1 pixel verifying staircase reference
            % C_on = C_off = 0.25
            cam = EventCamera('ContrastThresholdOn', 0.25, 'ContrastThresholdOff', 0.25);

            % Frame 1: logI = 0 (ref = 0)
            cam(exp(0) * ones(1,1), 0.0);

            % Frame 2: logI = 0.6, delta from ref = 0.6
            %   ON events: floor(0.6/0.25) = 2
            %   ref updated: 0 + 2*0.25 = 0.5
            events2 = cam(exp(0.6) * ones(1,1), 1.0);
            testCase.verifySize(events2, [2, 4]);

            % Frame 3: logI = 0.8, delta from ref(0.5) = 0.3
            %   ON events: floor(0.3/0.25) = 1
            %   ref updated: 0.5 + 1*0.25 = 0.75
            events3 = cam(exp(0.8) * ones(1,1), 2.0);
            testCase.verifySize(events3, [1, 4]);

            % Frame 4: logI = 0.2, delta from ref(0.75) = -0.55
            %   OFF events: floor(0.55/0.25) = 2
            %   ref updated: 0.75 - 2*0.25 = 0.25
            events4 = cam(exp(0.2) * ones(1,1), 3.0);
            testCase.verifySize(events4, [2, 4]);
            testCase.verifyTrue(all(events4(:,4) == -1));
        end

        function testRGBInput(testCase)
            cam1 = EventCamera();
            cam2 = EventCamera();

            grayFrame1 = 0.5 * ones(10, 10);
            rgbFrame1  = repmat(grayFrame1, [1, 1, 3]);
            grayFrame2 = 0.8 * ones(10, 10);
            rgbFrame2  = repmat(grayFrame2, [1, 1, 3]);

            cam1(grayFrame1, 0.0);
            cam2(rgbFrame1, 0.0);
            events1 = cam1(grayFrame2, 1.0);
            events2 = cam2(rgbFrame2, 1.0);

            testCase.verifyEqual(events1, events2, 'AbsTol', 1e-10);
        end

        function testResetClearsState(testCase)
            cam = EventCamera();
            frame1 = 0.5 * ones(5, 5);
            frame2 = 0.9 * ones(5, 5);
            cam(frame1, 0.0);
            events1 = cam(frame2, 1.0);
            testCase.verifyNotEmpty(events1);

            reset(cam);
            % After reset, next frame should be treated as first frame
            events_after = cam(frame1, 0.0);
            testCase.verifyEmpty(events_after);
        end

        function testTimestampsInRange(testCase)
            cam = EventCamera('Latency', 0);
            cam(rand(20, 20), 1.0);
            events = cam(rand(20, 20), 2.0);
            if ~isempty(events)
                testCase.verifyGreaterThanOrEqual(events(:,3), 1.0);
                testCase.verifyLessThanOrEqual(events(:,3), 2.0);
            end
        end

        function testTimestampsSorted(testCase)
            cam = EventCamera();
            cam(rand(20, 20), 0.0);
            events = cam(rand(20, 20), 1.0);
            if size(events, 1) > 1
                testCase.verifyTrue(issorted(events(:,3)));
            end
        end

        function testZeroIntensityNoCrash(testCase)
            cam = EventCamera();
            frame = zeros(5, 5);
            cam(frame, 0.0);
            events = cam(frame, 1.0);
            testCase.verifyEmpty(events);
        end

        function testOutputColumnsFormat(testCase)
            cam = EventCamera();
            cam(0.3 * ones(5,5), 0.0);
            events = cam(0.9 * ones(5,5), 1.0);
            testCase.verifySize(events(:, 1:4), [size(events,1), 4]);
            % x (col) in [1, 5], y (row) in [1, 5]
            testCase.verifyGreaterThanOrEqual(min(events(:,1)), 1);
            testCase.verifyLessThanOrEqual(max(events(:,1)), 5);
            testCase.verifyGreaterThanOrEqual(min(events(:,2)), 1);
            testCase.verifyLessThanOrEqual(max(events(:,2)), 5);
            % polarity is +1 or -1
            testCase.verifyTrue(all(events(:,4) == 1 | events(:,4) == -1));
        end

        %% Refractory period tests
        function testRefractoryPeriodReducesEvents(testCase)
            % Without refractory period
            cam1 = EventCamera('ContrastThresholdOn', 0.1, 'ContrastThresholdOff', 0.1);
            cam1(exp(0) * ones(5,5), 0.0);
            events1 = cam1(exp(0.5) * ones(5,5), 0.01); % short dt = closely spaced events

            % With refractory period longer than the inter-event spacing
            cam2 = EventCamera('ContrastThresholdOn', 0.1, 'ContrastThresholdOff', 0.1, ...
                'RefractoryPeriod', 0.005);
            cam2(exp(0) * ones(5,5), 0.0);
            events2 = cam2(exp(0.5) * ones(5,5), 0.01);

            testCase.verifyLessThan(size(events2, 1), size(events1, 1));
        end

        function testRefractoryZeroMatchesDefault(testCase)
            cam1 = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2);
            cam2 = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2, ...
                'RefractoryPeriod', 0);
            frame1 = 0.5 * ones(10, 10);
            frame2 = 0.9 * ones(10, 10);
            cam1(frame1, 0.0); cam2(frame1, 0.0);
            e1 = cam1(frame2, 1.0); e2 = cam2(frame2, 1.0);
            testCase.verifyEqual(e1, e2);
        end

        %% Bandwidth limiting tests
        function testBandwidthLimitingReducesEvents(testCase)
            % Infinite bandwidth (default) vs low bandwidth
            cam1 = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2);
            cam2 = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2, ...
                'BandwidthHz', 5);

            frame1 = 0.3 * ones(10, 10);
            frame2 = 0.9 * ones(10, 10);
            cam1(frame1, 0.0); cam2(frame1, 0.0);
            e1 = cam1(frame2, 1.0); e2 = cam2(frame2, 1.0);

            % Low bandwidth smooths the change, so fewer events
            testCase.verifyLessThanOrEqual(size(e2, 1), size(e1, 1));
        end

        function testInfiniteBandwidthMatchesDefault(testCase)
            cam1 = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2);
            cam2 = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2, ...
                'BandwidthHz', Inf);
            frame1 = 0.5 * ones(10, 10);
            frame2 = 0.9 * ones(10, 10);
            cam1(frame1, 0.0); cam2(frame1, 0.0);
            e1 = cam1(frame2, 1.0); e2 = cam2(frame2, 1.0);
            testCase.verifyEqual(e1, e2);
        end

        %% Latency tests
        function testLatencyShiftsTimestamps(testCase)
            % Same scene, with and without latency
            cam1 = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2, ...
                'Latency', 0);
            cam2 = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2, ...
                'Latency', 100e-6);
            frame1 = exp(0) * ones(1,1);
            frame2 = exp(0.5) * ones(1,1);
            cam1(frame1, 0.0); cam2(frame1, 0.0);
            e1 = cam1(frame2, 1.0);
            e2 = cam2(frame2, 1.0);

            % Same number of events
            testCase.verifyEqual(size(e1, 1), size(e2, 1));
            % Timestamps shifted by exactly the latency
            testCase.verifyEqual(e2(:,3), e1(:,3) + 100e-6, 'AbsTol', 1e-12);
        end

        function testZeroLatencyMatchesDefault(testCase)
            % Latency=0 should produce same results as before latency was added
            cam = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2, ...
                'Latency', 0);
            cam(exp(0) * ones(1,1), 0.0);
            events = cam(exp(0.5) * ones(1,1), 1.0);
            testCase.verifyEqual(events(:,3), [0.4; 0.8], 'AbsTol', 1e-10);
        end

        %% Intensity-dependent bandwidth tests
        function testBandwidthIntensityDependence(testCase)
            % Same log-intensity step applied to bright and dark regions.
            % The dark region's filter attenuates more, producing fewer events.
            cam = EventCamera('ContrastThresholdOn', 0.1, 'ContrastThresholdOff', 0.1, ...
                'BandwidthHz', 10, 'Latency', 0);

            % Frame with bright left half and dark right half
            frame1 = ones(10, 10);
            frame1(:, 1:5) = 0.8;   % bright
            frame1(:, 6:10) = 0.05; % dark

            cam(frame1, 0.0);

            % Apply same multiplicative factor so log change is identical
            % log(factor) = 0.5 for both regions
            factor = exp(0.5);
            frame2 = frame1 * factor;

            events = cam(frame2, 0.1);

            % Bright pixels respond faster -> more events pass the filter
            brightEvents = sum(events(:,1) <= 5);
            darkEvents = sum(events(:,1) > 5);
            testCase.verifyGreaterThan(brightEvents, darkEvents);
        end

        %% EventNoiseModel tests
        function testNoiseModelPassthroughWhenDisabled(testCase)
            % With all noise sources at zero, output should equal input
            noise = EventNoiseModel('ImageHeight', 10, 'ImageWidth', 10);
            events = [5 5 0.5 1; 3 3 0.6 -1];
            noise(events, 0.0);  % first call to set previous timestamp
            result = noise(events, 1.0);
            testCase.verifyEqual(result, events);
        end

        function testBackgroundNoiseAddsEvents(testCase)
            noise = EventNoiseModel('ImageHeight', 100, 'ImageWidth', 100, ...
                'BackgroundRate', 10000);
            events = zeros(0, 4);
            noise(events, 0.0);
            result = noise(events, 1.0);
            % Should have ~10000 noise events (Poisson)
            testCase.verifyGreaterThan(size(result, 1), 5000);
        end

        function testHotPixelsGenerateEvents(testCase)
            noise = EventNoiseModel('ImageHeight', 10, 'ImageWidth', 10, ...
                'NumHotPixels', 2, 'HotPixelRate', 500);
            events = zeros(0, 4);
            noise(events, 0.0);
            result = noise(events, 1.0);
            % Should have ~1000 hot pixel events (2 pixels * 500/s * 1s)
            testCase.verifyGreaterThan(size(result, 1), 500);
            % Hot pixels should concentrate at exactly 2 locations
            uniqueLocs = unique(result(:, 1:2), 'rows');
            testCase.verifyEqual(size(uniqueLocs, 1), 2);
        end

        function testTimestampJitterPreservesCount(testCase)
            noise = EventNoiseModel('ImageHeight', 10, 'ImageWidth', 10, ...
                'TimestampJitter', 0.001);
            events = [5 5 0.5 1; 3 3 0.6 -1; 7 7 0.8 1];
            noise(events, 0.0);
            result = noise(events, 1.0);
            % Same number of events, but timestamps shifted
            testCase.verifySize(result, size(events));
            % Timestamps should still be in [0, 1] range
            testCase.verifyGreaterThanOrEqual(min(result(:,3)), 0.0);
            testCase.verifyLessThanOrEqual(max(result(:,3)), 1.0);
        end

        function testNoiseModelOutputFormat(testCase)
            noise = EventNoiseModel('ImageHeight', 50, 'ImageWidth', 50, ...
                'BackgroundRate', 100, 'TimestampJitter', 0.0001);
            events = [25 25 0.5 1];
            noise(events, 0.0);
            result = noise(events, 1.0);
            testCase.verifyEqual(size(result, 2), 4);
            % All polarities are +1 or -1
            testCase.verifyTrue(all(result(:,4) == 1 | result(:,4) == -1));
            % Coordinates in range
            testCase.verifyGreaterThanOrEqual(min(result(:,1)), 1);
            testCase.verifyLessThanOrEqual(max(result(:,1)), 50);
            testCase.verifyGreaterThanOrEqual(min(result(:,2)), 1);
            testCase.verifyLessThanOrEqual(max(result(:,2)), 50);
        end
    end
end
