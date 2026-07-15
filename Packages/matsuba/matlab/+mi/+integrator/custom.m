function s = custom(type, varargin)
%MI.INTEGRATOR.CUSTOM Create an integrator descriptor for any Mitsuba integrator plugin.
%   S = MI.INTEGRATOR.CUSTOM(TYPE, 'param1', val1, ...) passes all parameters through.
    s.type = char(type);
    s.category_ = "integrator";
    assert(mod(numel(varargin), 2) == 0, ...
        "mi:integrator:custom", "Parameters must be name-value pairs.");
    for i = 1:2:numel(varargin)
        name = varargin{i};
        if isstring(name), name = char(name); end
        s.(name) = varargin{i+1};
    end
end
