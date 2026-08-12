function s = rgb(value)
%MI.RGB Create an RGB color descriptor for Mitsuba.
%   S = MI.RGB(VALUE) returns a struct representing an RGB color.
%   VALUE can be a 3-element [R G B] vector or a scalar (grayscale).
%
%   Example:
%       red = mi.rgb([0.8 0.1 0.1]);
%       gray = mi.rgb(0.5);
    arguments
        value double {mustBeNonempty}
    end
    s = struct("type", "rgb", "value", value);
end
