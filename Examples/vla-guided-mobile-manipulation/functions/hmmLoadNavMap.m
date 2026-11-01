function grid = hmmLoadNavMap(cfg)
%HMMLOADNAVMAP Return the saved occupancy grid, or the prior if there is none.
%
%   grid = HMMLOADNAVMAP() loads data/hmmNavMap.mat and returns its grid, after
%   checking that every geometric parameter the file was built against still
%   matches the current configuration. grid = HMMLOADNAVMAP(cfg) uses a supplied
%   configuration instead of calling hmmConfig.
%
%   THE RETURN IS ALWAYS A VALID INITIAL GRID. With no file, or with cfg.map.mode
%   set to 'live', the uniform prior comes back - which is exactly what the model
%   starts from today, so a missing file degrades to current behaviour rather than
%   to an error. That matters because this function is called from a data store's
%   initial value, and an error there fails the compile with a diagnostic pointing
%   at a block rather than at the absent file.
%
%   A MISMATCH IS AN ERROR, NOT A WARNING, and that asymmetry is the point. A grid
%   is a bare matrix of probabilities; nothing in it records the resolution or world
%   origin it was measured against. Load a 20 cells/m map into a 25 cells/m
%   configuration and every obstacle sits at 80 % of its true distance from an
%   origin that has also moved - the planner returns confident, valid-looking paths
%   through furniture. There is no safe silent fallback: resampling would invent
%   measurements, and warning-and-continuing would put the diagnostic in a console
%   nobody reads during a long run. Refusing is the only honest option.
%
%   WHY THE MODE LIVES IN cfg AND NOT IN THE FILE'S PRESENCE. Deleting the .mat as
%   a way to get live mapping back would make the behaviour depend on a file the
%   user has to remember not to regenerate. cfg.map.mode says which behaviour is
%   wanted; this function says whether it is achievable.
%
%   See also makeNavMap, hmmOccupancyFromDepth, hmmConfig.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg struct = hmmConfig()
end

prior = cfg.map.initialProbability * ones(cfg.map.gridSize);

% 'live' means build it from the cameras as before, so start from the prior and let
% the update path fill it in.
if isfield(cfg.map, 'mode') && strcmp(cfg.map.mode, 'live')
    grid = prior;
    return
end

mapFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'hmmNavMap.mat');
if ~isfile(mapFile)
    warning('hmmLoadNavMap:noFile', ...
        ['cfg.map.mode is ''fixed'' but %s does not exist, so the run starts from ' ...
         'the uniform prior. Run makeNavMap to capture one.'], mapFile);
    grid = prior;
    return
end

S = load(mapFile, 'grid', 'provenance');

% FIELD BY FIELD, NAMING THE CULPRIT. isequal on the whole struct would report
% "provenance differs", which sends the reader to compare two structs by hand.
expected = struct( ...
    'resolution',         cfg.map.resolution, ...
    'origin',             cfg.map.origin, ...
    'gridSize',           cfg.map.gridSize, ...
    'occupiedThreshold',  cfg.map.occupiedThreshold, ...
    'freeThreshold',      cfg.map.freeThreshold, ...
    'initialProbability', cfg.map.initialProbability, ...
    'sceneExtent',        cfg.scene.extent, ...
    'doorways',           cfg.map.doorways);

for f = string(fieldnames(expected))'
    if ~isfield(S.provenance, f)
        error('hmmLoadNavMap:provenanceIncomplete', ...
            ['%s was saved without the field ''%s'', so it predates the current ' ...
             'provenance check and cannot be verified. Regenerate it with makeNavMap.'], ...
            mapFile, f);
    end
    have = S.provenance.(f);
    want = expected.(f);
    if ~isequal(size(have), size(want)) || ~all(abs(double(have) - double(want)) < 1e-9, 'all')
        error('hmmLoadNavMap:provenanceMismatch', ...
            ['%s was built against cfg.map/scene ''%s'' = %s, but the current ' ...
             'configuration says %s. The grid carries no record of its own geometry, ' ...
             'so reusing it would silently shift or rescale every obstacle. ' ...
             'Regenerate with makeNavMap, or set cfg.map.mode = ''live''.'], ...
            mapFile, f, mat2str(double(have)), mat2str(double(want)));
    end
end

grid = double(S.grid);
if ~isequal(size(grid), double(cfg.map.gridSize))
    error('hmmLoadNavMap:sizeMismatch', ...
        ['%s holds a %s grid but cfg.map.gridSize is %s. The provenance passed, ' ...
         'so the file itself is inconsistent - regenerate it with makeNavMap.'], ...
        mapFile, mat2str(size(grid)), mat2str(double(cfg.map.gridSize)));
end

end
