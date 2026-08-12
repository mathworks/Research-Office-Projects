%[text] # Solar Panel Power Output Demo
%[text] This live script demonstrates how to compute the sun's position and estimate solar panel power output for any location and date using the `sunPosition` and `solarPanelPower` functions.
%%
%[text] ## Setup
%[text] Define our location (Boston, MA) and a south-facing residential solar panel.
latitude = 42.36;   % degrees north
longitude = -71.06; % degrees east
panelEfficiency = 0.20; % 20% efficient
panelArea = 1.6;        % m^2
panelTilt = 30;         % degrees from horizontal
panelAzimuth = 180;     % south-facing
%%
%[text] ## Sun Path on the Summer Solstice
%[text] Compute the sun's trajectory across the sky on June 21, sampled every 10 minutes.
dt = datetime(2024, 6, 21, 'TimeZone', 'UTC');
times = dt + minutes(0:10:1439);
[az, el] = sunPosition(latitude, longitude, times);
%%
%[text] Plot the sun path as a polar diagram where the radial axis is zenith angle and the angular axis is azimuth.
daytime = el > 0;
figure
polarplot(deg2rad(az(daytime)), 90 - el(daytime), 'Color', [0.9 0.6 0], 'LineWidth', 2)
ax = gca;
ax.ThetaZeroLocation = 'top';
ax.ThetaDir = 'clockwise';
ax.RLim = [0 90];
ax.RTickLabel = {'90°','60°','30°','0°'};
ax.RDir = 'normal';
title("Sun Path - Boston, Summer Solstice")
%%
%[text] ## Daily Power Profile
%[text] Estimate power output throughout the day for the summer solstice.
watts = solarPanelPower(panelEfficiency, panelArea, az, el, panelTilt, panelAzimuth);
localTimes = times;
localTimes.TimeZone = 'America/New_York';

figure
plot(localTimes, watts, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.5)
xlabel("Time (EDT)")
ylabel("Power (W)")
title("Solar Panel Output - June 21, Boston")
grid on
dailyEnergy = trapz(hours(times - times(1)), watts) / 1000;
subtitle(sprintf("Daily yield: %.2f kWh (clear sky)", dailyEnergy))
%%
%[text] ## Seasonal Comparison
%[text] Compare power output across four representative days of the year: spring equinox, summer solstice, autumn equinox, and winter solstice.

dates = [datetime(2024,3,20), datetime(2024,6,21), ...
         datetime(2024,9,22), datetime(2024,12,21)];
labels = ["Spring Equinox", "Summer Solstice", "Autumn Equinox", "Winter Solstice"];
colors = [0.2 0.7 0.3; 0.85 0.33 0.1; 0.6 0.4 0.0; 0.1 0.4 0.8];
dailyYield = zeros(1, 4);

figure
hold on
for k = 1:4
    t = datetime(dates(k), 'TimeZone', 'UTC') + minutes(0:10:1439);
    [a, e] = sunPosition(latitude, longitude, t);
    w = solarPanelPower(panelEfficiency, panelArea, a, e, panelTilt, panelAzimuth);
    localT = t;
    localT.TimeZone = 'America/New_York';
    plot(timeofday(localT), w, 'Color', colors(k,:), 'LineWidth', 1.5)
    dailyYield(k) = trapz(hours(t - t(1)), w) / 1000;
end
hold off
xlabel("Time of Day (local)")
ylabel("Power (W)")
title("Seasonal Power Output - Boston, South-Facing 30° Tilt")
legend(labels + " (" + compose("%.1f", dailyYield) + " kWh)", 'Location', 'northwest')
grid on
xlim([hours(4) hours(22)])
%%
%[text] ## Effect of Panel Tilt Angle
%[text] Find the optimal tilt angle for each season by sweeping from 0° to 90°.

tilts = 0:5:90;
energy = zeros(numel(tilts), 4);

for d = 1:4
    t = datetime(dates(d), 'TimeZone', 'UTC') + minutes(0:10:1439);
    [a, e] = sunPosition(latitude, longitude, t);
    for j = 1:numel(tilts)
        w = solarPanelPower(panelEfficiency, panelArea, a, e, tilts(j), panelAzimuth);
        energy(j, d) = trapz(hours(t - t(1)), w) / 1000;
    end
end

%%
%[text] Plot daily energy yield as a function of tilt angle for each season.
figure
plot(tilts, energy, 'LineWidth', 1.5)
xlabel("Panel Tilt (degrees from horizontal)")
ylabel("Daily Energy (kWh)")
title("Optimal Tilt Angle by Season - Boston")
legend(labels, 'Location', 'eastoutside')
grid on
%%
%[text] Mark the optimal tilt for each season.

[~, optIdx] = max(energy);
optTilts = tilts(optIdx);
hold on
for k = 1:4
    plot(optTilts(k), energy(optIdx(k),k), 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', colors(k,:), 'MarkerEdgeColor', 'k')
end
hold off
subtitle("Optimal tilts: " + strjoin(labels + " = " + optTilts + "°", ", "))
%%
%[text] ## Annual Energy Heatmap
%[text] Compute hourly power output for every day of the year and display as a heatmap.

startDate = datetime(2024, 1, 1, 'TimeZone', 'UTC');
daysOfYear = 0:364;
hoursOfDay = 0:23;
powerMap = zeros(365, 24);

for d = 1:365
    t = startDate + days(d-1) + hours(hoursOfDay);
    [a, e] = sunPosition(latitude, longitude, t);
    powerMap(d, :) = solarPanelPower(panelEfficiency, panelArea, a, e, panelTilt, panelAzimuth);
end

%%
%[text] Display the heatmap with months on the vertical axis and hour of day on the horizontal axis.

figure
imagesc(hoursOfDay, 1:365, powerMap)
colormap(hot)
cb = colorbar;
cb.Label.String = "Power (W)";
xlabel("Hour of Day (UTC)")
ylabel("Day of Year")
title("Annual Solar Power Heatmap - Boston, 30° South-Facing Panel")
set(gca, 'YDir', 'normal')
yticks(cumsum([1 31 29 31 30 31 30 31 31 30 31 30]))
yticklabels(["Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"])
%%
%[text] ## Summary
%[text] This demo showed how to:
%[text] - Compute sun position (azimuth and elevation) for any location and time
%[text] - Estimate solar panel power output under clear-sky conditions
%[text] - Visualize daily and seasonal patterns
%[text] - Find optimal tilt angles for different times of year \
