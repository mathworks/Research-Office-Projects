%[text] # Solar Analysis for Any Address
%[text] Enter a street address and get a complete solar energy analysis for that location. Uses the OpenStreetMap Nominatim geocoder to convert the address to coordinates, then computes sun position and panel output across the year.

%%
%[text] ## Enter Your Address
%[text] Change the address below to analyze any location.

address = "3 Apple Hill Drive, Natick, MA";

%%
%[text] ## Geocode the Address

[lat, lon, displayName] = geocodeAddress(address);
fprintf("Location: %s\n", displayName)
fprintf("Coordinates: %.4f°N, %.4f°E\n", lat, lon)

%%
%[text] ## Panel Configuration

panelEfficiency = 0.20;
panelArea = 1.6; % m^2
panelTilt = abs(lat); % rule-of-thumb: tilt = latitude
panelAzimuth = 180;   % south-facing (northern hemisphere)

%%
%[text] ## Show Location on Map
%[text] Display the location on a geographic map with satellite basemap.

figure('Position', [100 100 700 500])
geoplot(lat, lon, 'rp', 'MarkerSize', 20, 'MarkerFaceColor', 'r')
geobasemap satellite
title(sprintf("Solar Analysis Site: %s", address))
geolimits([lat-0.01 lat+0.01], [lon-0.01 lon+0.01])

%%
%[text] ## Sun Path Diagram
%[text] Compute sun paths for solstices and equinoxes at this location.

figure('Position', [100 100 600 600])
dates = [datetime(2024,3,20), datetime(2024,6,21), ...
         datetime(2024,9,22), datetime(2024,12,21)];
labels = ["Spring Equinox", "Summer Solstice", "Autumn Equinox", "Winter Solstice"];
colors = [0.2 0.7 0.3; 0.85 0.33 0.1; 0.6 0.4 0.0; 0.1 0.4 0.8];

polaraxes;
hold on
for k = 1:4
    t = datetime(dates(k), 'TimeZone', 'UTC') + minutes(0:5:1439);
    [az, el] = sunPosition(lat, lon, t);
    daytime = el > 0;
    polarplot(deg2rad(az(daytime)), 90 - el(daytime), ...
        'Color', colors(k,:), 'LineWidth', 2)
end
hold off
ax = gca;
ax.ThetaZeroLocation = 'top';
ax.ThetaDir = 'clockwise';
ax.RLim = [0 90];
ax.RTickLabel = {'90°','60°','30°','0°'};
title(sprintf("Sun Path — %s (%.2f°N)", address, lat))
legend(labels, 'Location', 'southoutside', 'Orientation', 'horizontal')

%%
%[text] ## Monthly Energy Yield
%[text] Compute the expected monthly clear-sky energy production.

daysPerMonth = [31 29 31 30 31 30 31 31 30 31 30 31];
monthlyEnergy = zeros(1, 12);

for m = 1:12
    t = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(0:23);
    [az, el] = sunPosition(lat, lon, t);
    w = solarPanelPower(panelEfficiency, panelArea, az, el, panelTilt, panelAzimuth);
    dailyKWh = sum(w) / 1000;
    monthlyEnergy(m) = dailyKWh * daysPerMonth(m);
end

annualTotal = sum(monthlyEnergy);

%%
%[text] Plot the monthly breakdown.

figure
bar(1:12, monthlyEnergy, 'FaceColor', [0.9 0.5 0.1], 'EdgeColor', 'none')
xlabel("Month")
ylabel("Energy (kWh)")
title(sprintf("Monthly Solar Energy — %s", address))
subtitle(sprintf("Annual total: %.0f kWh/panel (clear sky) | Tilt: %.0f° South", ...
    annualTotal, panelTilt))
xticklabels(["Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"])
grid on

%%
%[text] ## Daily Power Curves by Season
%[text] Show how power output varies through the day for each season.

figure
hold on
dailyYield = zeros(1, 4);
for k = 1:4
    t = datetime(dates(k), 'TimeZone', 'UTC') + minutes(0:10:1439);
    [az, el] = sunPosition(lat, lon, t);
    w = solarPanelPower(panelEfficiency, panelArea, az, el, panelTilt, panelAzimuth);
    plot(hours(t - t(1)), w, 'Color', colors(k,:), 'LineWidth', 1.5)
    dailyYield(k) = trapz(hours(t - t(1)), w) / 1000;
end
hold off
xlabel("Hour of Day (UTC)")
ylabel("Power (W)")
title(sprintf("Daily Power Profiles — %s", address))
legend(labels + " (" + compose("%.1f", dailyYield) + " kWh)", 'Location', 'northwest')
grid on
xlim([0 24])

%%
%[text] ## Summary
%[text] This analysis provides clear-sky estimates. Actual production will be lower due to:
%[text] - Cloud cover and weather
%[text] - Shading from buildings and trees
%[text] - Panel degradation and inverter losses
%[text] - Temperature effects \
%[text]
%[text] A typical derating factor is 0.75–0.80 for real-world conditions. Multiply the annual total by this factor for a more realistic estimate.

realisticEstimate = annualTotal * 0.77;
fprintf("Realistic annual estimate (77%% derating): %.0f kWh/panel\n", realisticEstimate)

%%
%[text] ---
%[text] *Functions used: `geocodeAddress`, `sunPosition`, `solarPanelPower`, `geoplot`*

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
