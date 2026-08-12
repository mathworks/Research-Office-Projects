function result = toPython(value)
%TOPYTHON Convert MATLAB values to Python-compatible types.

    if isstring(value) || ischar(value)
        result = py.str(string(value));
    elseif islogical(value) && isscalar(value)
        result = py.bool(value);
    elseif isnumeric(value) && isscalar(value)
        if ~isfinite(value)
            error("mi:toPython:NonFinite", ...
                "Cannot convert NaN or Inf to a Mitsuba parameter.");
        end
        if isinteger(value) || value == floor(value)
            result = py.int(int64(value));
        else
            result = py.float(value);
        end
    elseif isnumeric(value) && isvector(value)
        if any(~isfinite(value))
            error("mi:toPython:NonFinite", ...
                "Cannot convert NaN or Inf to a Mitsuba parameter.");
        end
        result = py.list(num2cell(double(value(:)')));
    elseif isnumeric(value) && ndims(value) == 3
        if any(~isfinite(value), "all")
            error("mi:toPython:NonFinite", ...
                "Cannot convert NaN or Inf to a Mitsuba parameter.");
        end
        % Convert 3D array to numpy array (for raymap data etc.)
        % Permute to (channel, col, row) so column-major flatten matches
        % numpy's default C-order reshape to (row, col, channel).
        sz = size(value);
        perm = permute(value, [3 2 1]);  % (3, W, H) in MATLAB
        result = py.numpy.array(double(perm(:)'));
        result = result.reshape(py.tuple({py.int(sz(1)), py.int(sz(2)), py.int(sz(3))}));
    elseif isnumeric(value) && ismatrix(value)
        if any(~isfinite(value), "all")
            error("mi:toPython:NonFinite", ...
                "Cannot convert NaN or Inf to a Mitsuba parameter.");
        end
        % Convert matrix to nested Python list (row-major)
        rows = cell(1, size(value, 1));
        for i = 1:size(value, 1)
            rows{i} = py.list(num2cell(double(value(i,:))));
        end
        result = py.list(rows);
    elseif iscell(value)
        items = cell(1, numel(value));
        for i = 1:numel(value)
            items{i} = mi.internal.toPython(value{i});
        end
        result = py.list(items);
    elseif isstruct(value)
        result = mi.internal.structToDict(value);
    else
        result = value;
    end
end
