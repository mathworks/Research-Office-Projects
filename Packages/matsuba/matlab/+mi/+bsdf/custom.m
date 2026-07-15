function s = custom(type, varargin)
%MI.BSDF.CUSTOM Create a BSDF descriptor for any Mitsuba BSDF plugin.
%   S = MI.BSDF.CUSTOM(TYPE, 'param1', val1, ...) passes all parameters through.
    s.type = char(type);
    s.category_ = "bsdf";
    assert(mod(numel(varargin), 2) == 0, ...
        "mi:bsdf:custom", "Parameters must be name-value pairs.");
    for i = 1:2:numel(varargin)
        name = varargin{i};
        if isstring(name), name = char(name); end
        s.(name) = varargin{i+1};
    end
end
