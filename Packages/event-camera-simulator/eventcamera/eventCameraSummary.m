function eventCameraSummary(simOut)
% EVENTCAMERASUMMARY Post-simulation summary figure for event camera output.
%   eventCameraSummary(simOut) extracts events from the simulation output
%   and displays event rate over time and an accumulated event image.

if ~isfield(simOut, 'eventLog') && ~isprop(simOut, 'eventLog')
    warning('No eventLog found in simulation output.');
    return;
end

el = simOut.eventLog;

%% Extract events from timetable
allEvents = [];
frameTimes = seconds(el.Time);
eventsPerFrame = zeros(height(el), 1);

for i = 1:height(el)
    chunk = el.Data{i};
    if iscell(chunk), chunk = chunk{1}; end
    if ~isempty(chunk) && size(chunk, 2) == 4
        allEvents = [allEvents; chunk];        eventsPerFrame(i) = size(chunk, 1);
    end
end

if isempty(allEvents)
    warning('No events generated during simulation.');
    return;
end

nOn  = sum(allEvents(:,4) == 1);
nOff = sum(allEvents(:,4) == -1);

%% Summary figure
figure('Name', 'Event Camera Summary', 'Position', [100 100 1100 400], ...
    'NumberTitle', 'off');

% Panel 1: Event rate over time
subplot(1,3,1);
plot(frameTimes, eventsPerFrame, 'k', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Events per frame');
title('Event Rate');
grid on;

% Panel 2: Accumulated event image
subplot(1,3,2);
xRange = [min(allEvents(:,1)), max(allEvents(:,1))];
yRange = [min(allEvents(:,2)), max(allEvents(:,2))];
imgW = round(xRange(2));
imgH = round(yRange(2));
eventImage = zeros(imgH, imgW, 3);
for i = 1:size(allEvents, 1)
    x = round(allEvents(i,1));
    y = round(allEvents(i,2));
    if x >= 1 && x <= imgW && y >= 1 && y <= imgH
        if allEvents(i,4) == 1
            eventImage(y, x, 3) = min(eventImage(y, x, 3) + 0.02, 1);
        else
            eventImage(y, x, 1) = min(eventImage(y, x, 1) + 0.02, 1);
        end
    end
end
imshow(eventImage);
title('Accumulated Events');

% Panel 3: Text summary
subplot(1,3,3);
axis off;
simDuration = frameTimes(end) - frameTimes(1);
avgRate = size(allEvents,1) / max(simDuration, eps);
summaryText = {
    sprintf('Total events: %s', formatNum(size(allEvents,1)))
    sprintf('ON events: %s', formatNum(nOn))
    sprintf('OFF events: %s', formatNum(nOff))
    ''
    sprintf('Duration: %.1f s', simDuration)
    sprintf('Avg rate: %s events/s', formatNum(round(avgRate)))
    sprintf('Peak frame: %s events', formatNum(max(eventsPerFrame)))
};
text(0.1, 0.7, summaryText, 'FontSize', 11, 'VerticalAlignment', 'top', ...
    'FontName', 'FixedWidth');
title('Statistics');
end

function s = formatNum(n)
    if n >= 1e6
        s = sprintf('%.2fM', n/1e6);
    elseif n >= 1e3
        s = sprintf('%.1fK', n/1e3);
    else
        s = sprintf('%d', n);
    end
end
