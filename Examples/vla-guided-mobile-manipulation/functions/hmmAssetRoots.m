function roots = hmmAssetRoots()
%HMMASSETROOTS Candidate asset roots, found by walking up from this project.
%
%   roots = HMMASSETROOTS() returns a nearest-first cell array of folders that
%   may hold runtime assets: the apartment scene, the movable-prop metadata,
%   the recorded provider corpus, and a provider key file.
%
%   WHY A WALK RATHER THAN A PATH. The design requires that nothing which is
%   only meaningful on one machine is committed. An absolute path in a
%   configuration file is exactly that, and for a key file it is worse than
%   useless: it publishes where to look. This function therefore searches
%   RELATIVE to its own location and reports only folders that actually exist.
%   No credential, no file content and no discovered path is written back to
%   any project file by this function or its callers - the provider reports
%   which ROUTE resolved, never what it resolved to.
%
%   The walk climbs at most maxLevels parents. Beyond that a hit is more likely
%   to be an unrelated folder that happens to share a name than the asset root,
%   and silently reading a stranger's folder is not a behaviour worth having.
%
%   See also hmmDiscoverMovables, hmmProviderRequest.

%   Copyright 2026 The MathWorks, Inc.

maxLevels = 6;
siblings = {'assets', 'apartment', 'secrets', '.secrets'};

here = fileparts(fileparts(mfilename('fullpath')));   % the project root
found = cell(1, 0);
d = here;
for level = 1:maxLevels
    found{end+1} = d;                                              %#ok<AGROW>
    for k = 1:numel(siblings)
        p = fullfile(d, siblings{k});
        if isfolder(p)
            found{end+1} = p;                                      %#ok<AGROW>
        end
    end
    up = fileparts(d);
    if isempty(up) || strcmp(up, d)
        break
    end
    d = up;
end

roots = unique(found, 'stable');

end
