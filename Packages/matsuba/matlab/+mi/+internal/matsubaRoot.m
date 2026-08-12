function root = matsubaRoot()
%MATSUBAROOT Return the MATSUBA project root directory.
%   Derives the root from the location of this file:
%   <root>/matlab/+mi/+internal/matsubaRoot.m -> <root>

    thisFile = mfilename("fullpath");
    root = fileparts(fileparts(fileparts(fileparts(thisFile))));
end
