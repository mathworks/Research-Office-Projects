function watts = solarPanelPower(efficiency, area, sunAzimuth, sunElevation, panelTilt, panelAzimuth, options)
%SOLARPANELPOWER Estimate power output of a solar panel given sun position.
%   WATTS = SOLARPANELPOWER(EFFICIENCY, AREA, SUNAZIMUTH, SUNELEVATION,
%   PANELTILT, PANELAZIMUTH) computes estimated electrical power output.
%
%   WATTS = SOLARPANELPOWER(..., ClearSkyFraction=KT) scales output by a
%   clearness index (0 to 1) to account for average cloud cover. Use the
%   CLEARNESSINDEX function to obtain a value for a given location.
%
%   WATTS = SOLARPANELPOWER(..., AmbientTemp=T) applies a temperature
%   derating based on cell temperature. T is ambient temperature in °C.
%
%   WATTS = SOLARPANELPOWER(..., SystemLoss=L) applies a fixed system loss
%   factor (inverter, wiring, soiling). L is a fraction from 0 to 1;
%   default is 0.14 (14% loss). Set to 0 to disable.
%
%   Inputs:
%       efficiency   - Panel efficiency (0 to 1, e.g. 0.20 for 20%)
%       area         - Panel area in m^2
%       sunAzimuth   - Solar azimuth in degrees (from sunPosition)
%       sunElevation - Solar elevation in degrees (from sunPosition)
%       panelTilt    - Panel tilt from horizontal in degrees (0=flat, 90=vertical)
%       panelAzimuth - Direction the panel faces in degrees clockwise from
%                      north (180=south-facing in northern hemisphere)
%
%   Name-Value Arguments:
%       ClearSkyFraction - Clearness index from 0 to 1 (default: 1)
%       AmbientTemp      - Ambient temperature in °C (default: [], no derating)
%       SystemLoss       - Fractional system loss from 0 to 1 (default: 0.14)
%
%   Output:
%       watts - Estimated electrical power output in watts
%
%   Uses a clear-sky direct+diffuse irradiance model based on air mass.
%   Returns 0 when the sun is below the horizon or behind the panel.
%
%   Example:
%       [az, el] = sunPosition(42.36, -71.06, datetime(2024,6,21,16,0,0,'TimeZone','UTC'));
%       W = solarPanelPower(0.20, 1.6, az, el, 30, 180)
%       W = solarPanelPower(0.20, 1.6, az, el, 30, 180, ClearSkyFraction=0.71)

    arguments
        efficiency (1,1) double {mustBeInRange(efficiency, 0, 1)}
        area (1,1) double {mustBePositive}
        sunAzimuth double
        sunElevation double
        panelTilt (1,1) double {mustBeInRange(panelTilt, 0, 90)} = 30
        panelAzimuth (1,1) double = 180
        options.ClearSkyFraction (1,1) double {mustBeInRange(options.ClearSkyFraction, 0, 1)} = 1
        options.AmbientTemp double = []
        options.SystemLoss (1,1) double {mustBeInRange(options.SystemLoss, 0, 1)} = 0
    end

    watts = zeros(size(sunAzimuth));

    % Sun below horizon produces no power
    aboveHorizon = sunElevation > 0;

    % Direct Normal Irradiance using simplified clear-sky air mass model
    % Air mass (Kasten-Young approximation)
    zenith = 90 - sunElevation(aboveHorizon);
    airMass = 1 ./ (cosd(zenith) + 0.50572 * (96.07995 - zenith).^(-1.6364));

    % Clear-sky DNI (W/m^2) using Beer-Lambert with typical atmosphere
    DNI = 1361 * 0.7 .^ (airMass .^ 0.678);

    % Diffuse horizontal irradiance (fraction of scattered beam)
    extraterrestrialH = 1361 * sind(sunElevation(aboveHorizon));
    DHI = 0.2 * (extraterrestrialH - DNI .* sind(sunElevation(aboveHorizon)));

    % Global horizontal irradiance (for ground reflection)
    GHI = DNI .* sind(sunElevation(aboveHorizon)) + DHI;

    % Angle of incidence between sun and panel normal
    sunElRad = deg2rad(sunElevation(aboveHorizon));
    sunAzRad = deg2rad(sunAzimuth(aboveHorizon));
    tiltRad = deg2rad(panelTilt);
    panelAzRad = deg2rad(panelAzimuth);

    cosIncidence = sin(sunElRad) * cos(tiltRad) ...
                 + cos(sunElRad) .* sin(tiltRad) .* cos(sunAzRad - panelAzRad);

    % Panel only collects when sun is in front of it
    cosIncidence = max(cosIncidence, 0);

    % Total irradiance on panel surface (direct + diffuse + ground-reflected)
    albedo = 0.2;
    irradianceOnPanel = DNI .* cosIncidence ...
                      + DHI * (1 + cos(tiltRad)) / 2 ...
                      + GHI * albedo * (1 - cos(tiltRad)) / 2;

    watts(aboveHorizon) = efficiency * area * irradianceOnPanel;

    % Apply clearness index (cloud cover reduction)
    watts = watts * options.ClearSkyFraction;

    % Apply temperature derating if ambient temperature is provided
    if ~isempty(options.AmbientTemp)
        Tamb = options.AmbientTemp;
        if isscalar(Tamb)
            Tamb = repmat(Tamb, size(sunAzimuth));
        end
        % Simplified NOCT cell temperature model
        Tcell = Tamb(aboveHorizon) + 25 * (irradianceOnPanel / 1000);
        % Typical silicon panel: -0.4%/°C above 25°C
        tempDerating = 1 + (-0.004) * (Tcell - 25);
        tempDerating = max(tempDerating, 0);
        watts(aboveHorizon) = watts(aboveHorizon) .* tempDerating;
    end

    % Apply system losses (inverter, wiring, soiling)
    watts = watts * (1 - options.SystemLoss);

    % Ensure non-negative
    watts = max(watts, 0);
end
