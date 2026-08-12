function s = pluginStruct(type, category, options)
%MI.INTERNAL.PLUGINSTRUCT Assemble a Mitsuba plugin descriptor struct.
    arguments
        type (1,1) string
        category (1,1) string
        options (1,1) struct
    end
    s.type = type;
    s.category_ = category;
    fields = fieldnames(options);
    for i = 1:numel(fields)
        v = options.(fields{i});
        if isSkippable(v)
            continue
        end
        s.(fields{i}) = v;
    end
end

function tf = isSkippable(v)
    if isnumeric(v) && isempty(v)
        tf = true;
    elseif islogical(v) && isempty(v)
        tf = true;
    elseif isstruct(v) && isempty(v)
        tf = true;
    elseif isstring(v) && isempty(v)
        tf = true;
    else
        tf = false;
    end
end
