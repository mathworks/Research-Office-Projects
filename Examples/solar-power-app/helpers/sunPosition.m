function [azimuth, elevation] = sunPosition(latitude, longitude, dateTimeUTC)
%SUNPOSITION Compute sun azimuth and elevation for a given location and time.
%   [AZIMUTH, ELEVATION] = SUNPOSITION(LATITUDE, LONGITUDE, DATETIMEUTC)
%   returns the sun's position in the local horizontal coordinate frame.
%
%   Inputs:
%       latitude    - Observer latitude in degrees (positive north)
%       longitude   - Observer longitude in degrees (positive east)
%       dateTimeUTC - datetime scalar or vector in UTC (no timezone or UTC)
%
%   Outputs:
%       azimuth   - Solar azimuth in degrees, measured clockwise from north
%                   (0=N, 90=E, 180=S, 270=W)
%       elevation - Solar elevation (altitude) in degrees above the horizon
%                   (-90 to 90)
%
%   Algorithm based on Jean Meeus, "Astronomical Algorithms", 2nd ed.
%   Accuracy approximately 0.01 degrees.
%
%   Example:
%       dt = datetime(2024, 6, 21, 12, 0, 0, 'TimeZone', 'UTC');
%       [az, el] = sunPosition(42.36, -71.06, dt)

    arguments
        latitude (1,1) double {mustBeInRange(latitude, -90, 90)}
        longitude (1,1) double {mustBeInRange(longitude, -180, 180)}
        dateTimeUTC datetime
    end

    if ~isempty(dateTimeUTC.TimeZone) && ~strcmpi(dateTimeUTC.TimeZone, "UTC")
        dateTimeUTC.TimeZone = "UTC";
    end

    % Julian Day Number (days since Jan 1, 4713 BC)
    JD = datenum(dateTimeUTC) + 1721058.5;

    % Julian centuries from J2000.0
    T = (JD - 2451545.0) / 36525.0;

    % Geometric mean longitude of the sun (degrees)
    L0 = mod(280.46646 + T .* (36000.76983 + 0.0003032 * T), 360);

    % Mean anomaly of the sun (degrees)
    M = mod(357.52911 + T .* (35999.05029 - 0.0001537 * T), 360);
    Mrad = deg2rad(M);

    % Equation of center (degrees)
    C = (1.914602 - T .* (0.004817 + 0.000014 * T)) .* sin(Mrad) ...
      + (0.019993 - 0.000101 * T) .* sin(2 * Mrad) ...
      + 0.000289 * sin(3 * Mrad);

    % Sun's true longitude and true anomaly
    sunLon = L0 + C;

    % Apparent longitude (correcting for nutation and aberration)
    omega = 125.04 - 1934.136 * T;
    lambda = sunLon - 0.00569 - 0.00478 * sin(deg2rad(omega));
    lambdaRad = deg2rad(lambda);

    % Mean obliquity of the ecliptic (degrees)
    epsilon0 = 23.439291 - T .* (0.013004 + 0.00000016 * T);
    % Corrected obliquity
    epsilon = epsilon0 + 0.00256 * cos(deg2rad(omega));
    epsilonRad = deg2rad(epsilon);

    % Sun's right ascension (radians)
    RA = atan2(cos(epsilonRad) .* sin(lambdaRad), cos(lambdaRad));

    % Sun's declination (radians)
    decl = asin(sin(epsilonRad) .* sin(lambdaRad));

    % Greenwich Mean Sidereal Time (degrees)
    GMST = mod(280.46061837 + 360.98564736629 * (JD - 2451545.0) ...
           + 0.000387933 * T.^2 - T.^3 / 38710000, 360);

    % Local hour angle (radians)
    HA = deg2rad(GMST + longitude) - RA;

    % Convert observer latitude to radians
    latRad = deg2rad(latitude);

    % Elevation (altitude) angle
    sinEl = sin(latRad) * sin(decl) + cos(latRad) * cos(decl) .* cos(HA);
    elevation = rad2deg(asin(sinEl));

    % Azimuth (measured clockwise from north)
    azimuth = mod(rad2deg(atan2(-sin(HA), ...
        tan(decl) * cos(latRad) - sin(latRad) * cos(HA))) , 360);
end
