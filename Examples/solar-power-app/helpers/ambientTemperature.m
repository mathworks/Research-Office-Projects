function T = ambientTemperature(latitude, dateTimeUTC)
%AMBIENTTEMPERATURE Estimate ambient temperature from latitude and date.
%   T = AMBIENTTEMPERATURE(LATITUDE, DATETIMEUTC) returns estimated ambient
%   air temperature in degrees Celsius using a sinusoidal seasonal model.
%
%   Inputs:
%       latitude    - Observer latitude in degrees (positive north)
%       dateTimeUTC - datetime scalar or vector
%
%   Output:
%       T - Estimated ambient temperature in degrees Celsius
%
%   The model captures the annual mean (warmer near equator) and seasonal
%   amplitude (larger swings at higher latitudes), with a thermal lag of
%   ~27 days after the solstice. Does not include a diurnal cycle; pair
%   with a cell temperature model that uses irradiance for time-of-day
%   effects.
%
%   Accuracy is approximately ±5-10°C for continental locations.
%
%   Example:
%       dt = datetime(2024,7,15,14,0,0,'TimeZone','UTC');
%       T = ambientTemperature(42.36, dt)  % Boston in July: ~25°C

    arguments
        latitude (1,1) double {mustBeInRange(latitude, -90, 90)}
        dateTimeUTC datetime
    end

    absLat = abs(latitude);

    % Annual mean temperature decreases with latitude
    Tmean = 30 - 0.5 * absLat;

    % Seasonal amplitude increases with latitude
    Tamp = 5 + 0.3 * absLat;

    % Day of year with thermal lag (~27 days after solstice peak)
    doy = day(dateTimeUTC, 'dayofyear');
    peakDay = 200;  % ~July 19 in northern hemisphere
    if latitude < 0
        peakDay = peakDay - 183;  % ~Jan 18 in southern hemisphere
    end

    T = Tmean + Tamp * cos(2 * pi * (doy - peakDay) / 365);
end
