function s = custom(type, varargin)
%MI.EMITTER.CUSTOM Create an emitter descriptor for any Mitsuba emitter plugin.
%   S = MI.EMITTER.CUSTOM(TYPE, 'param1', val1, ...) passes all parameters through.
    s.type = char(type);
    s.category_ = "emitter";
    assert(mod(numel(varargin), 2) == 0, ...
        "mi:emitter:custom", "Parameters must be name-value pairs.");
    for i = 1:2:numel(varargin)
        name = varargin{i};
        if isstring(name), name = char(name); end
        s.(name) = varargin{i+1};
    end
end
