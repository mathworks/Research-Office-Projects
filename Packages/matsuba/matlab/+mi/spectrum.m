function s = spectrum(options)
%MI.SPECTRUM Create a spectrum descriptor for Mitsuba.
%   S = MI.SPECTRUM(value=0.5) returns a uniform spectrum struct.
%   S = MI.SPECTRUM(filename="measured.spd") returns a file-based spectrum.
%   S = MI.SPECTRUM(wavelengths=[400 500 600], values=[0.5 0.8 0.2])
%       returns a sampled spectrum.
%
%   Example:
%       uniform = mi.spectrum(value=0.5);
%       fromFile = mi.spectrum(filename="measured.spd");
%       sampled = mi.spectrum(wavelengths=[400 500 600], values=[0.5 0.8 0.2]);
    arguments
        options.value double = []
        options.filename string = string.empty
        options.wavelengths double = []
        options.values double = []
    end

    s.type = "spectrum";

    if ~isempty(options.value)
        s.value = options.value;
    elseif ~isempty(options.filename)
        s.filename = char(options.filename);
    elseif ~isempty(options.wavelengths) && ~isempty(options.values)
        s.wavelengths = options.wavelengths;
        s.values = options.values;
    else
        error("mi:spectrum:invalidInput", ...
            "Provide value, filename, or both wavelengths and values.");
    end
end
