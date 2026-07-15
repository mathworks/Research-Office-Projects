function s = custom(type, varargin)
%MI.SHAPE.CUSTOM Create a shape descriptor for any Mitsuba shape plugin.
%   S = MI.SHAPE.CUSTOM(TYPE, 'param1', val1, ...) passes all parameters through.
    s.type = char(type);
    s.category_ = "shape";
    assert(mod(numel(varargin), 2) == 0, ...
        "mi:shape:custom", "Parameters must be name-value pairs.");
    for i = 1:2:numel(varargin)
        name = varargin{i};
        if isstring(name), name = char(name); end
        s.(name) = varargin{i+1};
    end
end
