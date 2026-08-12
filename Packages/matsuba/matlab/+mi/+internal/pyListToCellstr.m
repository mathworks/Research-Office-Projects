function out = pyListToCellstr(pyList)
%PYLISTTOCELLSTR Convert a Python list of strings to MATLAB string array.

    cellResult = cell(pyList);
    out = string(cellfun(@char, cellResult, UniformOutput=false));
end
