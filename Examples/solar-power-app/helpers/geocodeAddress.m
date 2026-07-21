function [latitude, longitude, displayName] = geocodeAddress(address)
%GEOCODEADDRESS Convert a street address to latitude and longitude.
%   [LAT, LON, NAME] = GEOCODEADDRESS(ADDRESS) uses the OpenStreetMap
%   Nominatim API to geocode the given address string.
%
%   Inputs:
%       address - Character vector or string with the address to look up
%
%   Outputs:
%       latitude    - Latitude in degrees (positive north)
%       longitude   - Longitude in degrees (positive east)
%       displayName - Full display name returned by the geocoder
%
%   Example:
%       [lat, lon, name] = geocodeAddress("1 Apple Park Way, Cupertino, CA")

    arguments
        address (1,1) string {mustBeNonzeroLengthText}
    end

    url = "https://nominatim.openstreetmap.org/search";
    options = weboptions('ContentType', 'json', ...
                         'UserAgent', 'MATLAB Solar Demo/1.0', ...
                         'Timeout', 10);

    result = webread(url, 'q', address, 'format', 'json', 'limit', '1', options);

    if isempty(result)
        error('geocodeAddress:notFound', ...
            'Could not find coordinates for address: "%s"', address);
    end

    latitude = str2double(result(1).lat);
    longitude = str2double(result(1).lon);
    displayName = string(result(1).display_name);
end
