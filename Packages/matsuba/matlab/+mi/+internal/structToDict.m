function d = structToDict(s)
%STRUCTTODICT Recursively convert a MATLAB struct to a Python dict.

    fields = fieldnames(s);
    keys = cell(1, 2*numel(fields));
    for i = 1:numel(fields)
        keys{2*i-1} = fields{i};
        val = s.(fields{i});
        if isstruct(val)
            keys{2*i} = mi.internal.structToDict(val);
        else
            keys{2*i} = mi.internal.toPython(val);
        end
    end
    d = py.dict(pyargs(keys{:}));
end
