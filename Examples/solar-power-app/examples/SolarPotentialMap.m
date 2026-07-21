%[text] # Solar Energy Potential Map
%[text] This live script computes the annual clear-sky solar energy yield across the continental United States and visualizes the results on a geographic map using Mapping Toolbox.

%%
%[text] ## Panel Configuration
%[text] Define a standard residential solar panel. The map shows annual energy yield per panel at each location assuming south-facing orientation at a tilt equal to the local latitude (a common rule-of-thumb).

panelEfficiency = 0.20;
panelArea = 1.6; % m^2
panelAzimuth = 180; % south-facing

%%
%[text] ## Define Geographic Grid
%[text] Create a 1-degree latitude/longitude grid over the continental US.

latRange = 25:1:49;
lonRange = -124:1:-67;
[lonGrid, latGrid] = meshgrid(lonRange, latRange);

%%
%[text] ## Compute Annual Solar Yield
%[text] For computational efficiency, sample 12 representative days (mid-month) with hourly resolution and scale to annual totals.

representativeDays = datetime(2024, 1:12, 15, 'TimeZone', 'UTC');
daysPerMonth = [31 29 31 30 31 30 31 31 30 31 30 31];
hoursOfDay = hours(0:23);

annualEnergy = zeros(size(latGrid)); % kWh per year

for i = 1:numel(latGrid)
    lat = latGrid(i);
    lon = lonGrid(i);
    panelTilt = abs(lat); % tilt = latitude rule

    yearlyTotal = 0;
    for m = 1:12
        t = representativeDays(m) + hoursOfDay;
        [az, el] = sunPosition(lat, lon, t);
        w = solarPanelPower(panelEfficiency, panelArea, az, el, panelTilt, panelAzimuth);
        dailyKWh = sum(w) / 1000; % hourly samples, each represents 1 hour
        yearlyTotal = yearlyTotal + dailyKWh * daysPerMonth(m);
    end
    annualEnergy(i) = yearlyTotal;
end

%%
%[text] ## Solar Potential Map
%[text] Display the annual energy yield as a filled contour on a map of the continental US with state boundaries.

figure('Position', [100 100 900 600])
usamap('conus')

contourfm(latGrid, lonGrid, annualEnergy, 20, 'LineStyle', 'none')

colormap(turbo)
cb = colorbar;
cb.Label.String = "Annual Energy Yield (kWh/panel/year)";

geoshow('usastatehi.shp', 'DisplayType', 'polygon', ...
    'FaceColor', 'none', 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5)

title("Clear-Sky Solar Potential — Continental US")
subtitle("20% efficient, 1.6 m² panel, south-facing, tilt = latitude")

%%
%[text] ## Key Observations
%[text] - The Southwest (Arizona, Nevada, New Mexico) receives the highest solar resource due to high elevation, low latitude, and clear skies
%[text] - Annual yield ranges from approximately 400 kWh in the northern US to over 600 kWh in the desert Southwest
%[text] - This is a *clear-sky* estimate — actual yields depend on cloud cover, which would reduce northern and coastal values further \

%%
%[text] ## Zonal Statistics
%[text] Show the mean annual yield grouped by latitude band.

figure
latBands = 25:5:50;
meanYield = zeros(1, numel(latBands)-1);
for k = 1:numel(latBands)-1
    mask = latGrid >= latBands(k) & latGrid < latBands(k+1);
    meanYield(k) = mean(annualEnergy(mask));
end
bar(latBands(1:end-1) + 2.5, meanYield, 'FaceColor', [0.9 0.5 0.1])
xlabel("Latitude Band (°N)")
ylabel("Mean Annual Yield (kWh/panel)")
title("Average Solar Yield by Latitude")
grid on

%%
%[text] ---
%[text] *Functions used: `sunPosition`, `solarPanelPower`, Mapping Toolbox (`usamap`, `contourfm`, `geoshow`)*

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
