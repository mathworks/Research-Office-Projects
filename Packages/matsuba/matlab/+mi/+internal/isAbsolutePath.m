function tf = isAbsolutePath(p)
%MI.INTERNAL.ISABSOLUTEPATH Check if a path is absolute (Windows or Unix).
%   TF = MI.INTERNAL.ISABSOLUTEPATH(P) returns true if P starts with '/',
%   '\', or a Windows drive letter (e.g. 'C:').
    if isempty(p)
        tf = false;
    elseif p(1) == '/' || p(1) == '\'
        tf = true;
    elseif numel(p) >= 2 && p(2) == ':'
        tf = true;
    else
        tf = false;
    end
end
