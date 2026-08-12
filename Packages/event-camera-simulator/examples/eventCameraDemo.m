%% Event Camera Simulator Demo
% Synthetic scene: bright vertical bar moving horizontally across a dark
% background. Visualizes ON (blue) and OFF (red) events as x-vs-time.

%% Parameters
imgHeight = 120;
imgWidth  = 160;
numFrames = 100;
fps       = 30;
dt        = 1 / fps;

barWidth    = 10;
barSpeed    = 2;   % pixels per frame
bgIntensity = 0.1;
barIntensity = 0.8;

%% Create Event Camera
cam = EventCamera('ContrastThresholdOn', 0.2, 'ContrastThresholdOff', 0.2);

%% Generate frames and collect events
allEvents = [];

for i = 1:numFrames
    % Build frame: dark background with bright vertical bar
    frame = bgIntensity * ones(imgHeight, imgWidth);
    barCenter = round(barSpeed * i);
    barLeft   = max(1, barCenter - floor(barWidth/2));
    barRight  = min(imgWidth, barCenter + floor(barWidth/2));
    if barLeft <= imgWidth && barRight >= 1
        frame(:, barLeft:barRight) = barIntensity;
    end

    timestamp = (i - 1) * dt;
    events = cam(frame, timestamp);

    if ~isempty(events)
        allEvents = [allEvents; events];
    end
end

%% Visualize: x position vs. time scatter plot
figure('Name', 'Event Camera Output', 'Position', [100 100 900 500]);

if ~isempty(allEvents)
    onMask  = allEvents(:,4) ==  1;
    offMask = allEvents(:,4) == -1;

    hold on;
    scatter(allEvents(offMask, 3), allEvents(offMask, 1), 2, 'r', 'filled', ...
        'DisplayName', 'OFF events');
    scatter(allEvents(onMask, 3), allEvents(onMask, 1), 2, 'b', 'filled', ...
        'DisplayName', 'ON events');
    hold off;
end

xlabel('Time (s)');
ylabel('X position (pixel)');
title('Event Camera: Moving Bar');
legend('Location', 'best');
grid on;
set(gca, 'Color', [0.95 0.95 0.95]);

fprintf('Total events generated: %d\n', size(allEvents, 1));
fprintf('  ON events:  %d\n', sum(allEvents(:,4) == 1));
fprintf('  OFF events: %d\n', sum(allEvents(:,4) == -1));
