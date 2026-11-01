function [items, propsDir, manifestFile] = hmmSceneManifest()
%HMMSCENEMANIFEST Read the scene's movable-prop placement metadata.
%
%   [items, propsDir, manifestFile] = HMMSCENEMANIFEST() finds the prop
%   manifest in the folders reported by hmmAssetRoots and returns it as an
%   UNDIFFERENTIATED struct array, one element per prop:
%
%       .name       manifest key, used as the Sim3D actor name
%       .position   [x y z] world placement, m
%       .extent     [ex ey ez] bounding extent, m
%
%   propsDir is the folder holding the prop meshes and manifestFile the file
%   that was read. Both are empty when nothing was found.
%
%   ORDER IS ALPHABETICAL BY KEY AND NOTHING ELSE. The design requires the
%   props to reach the system as a set of equal candidates: the manifest says
%   where a prop starts, not which prop the task is about. Sorting by key
%   rather than preserving file order is what makes the identity a prop is
%   given - its registry index - reproducible without making any prop special.
%   No entry is filtered, renamed, weighted or looked up by name here or in any
%   caller.
%
%   TWO SCHEMAS ARE ACCEPTED, and that is deliberate rather than defensive. The
%   BASELINE schema is a map of key to placement, which is the form the reused
%   apartment assets actually ship:
%
%       {"Bottle": {"position": [x, y, z], "size": [ex, ey, ez]}, ...}
%
%   The EXPLICIT schema carries a rendered-surface identifier as well, which a
%   scene able to report one can supply so that association becomes identity
%   rather than proximity:
%
%       {"movables": [{"id": 1, "surfaceId": 1234,
%                      "position": [x, y, z], "extent": [ex, ey, ez]}, ...]}
%
%   Reading only the second would mean re-deriving placement metadata the
%   baseline already publishes, which the design forbids.
%
%   See also hmmAssetRoots, hmmDiscoverMovables, hmmSpawnProps.

%   Copyright 2026 The MathWorks, Inc.

items = struct('name', {}, 'position', {}, 'extent', {}, 'surfaceId', {});
propsDir = '';
manifestFile = '';

roots = hmmAssetRoots();
candidates = cell(1, 0);
for k = 1:numel(roots)
    candidates{end+1} = fullfile(roots{k}, 'props', 'props.json');     %#ok<AGROW>
    candidates{end+1} = fullfile(roots{k}, 'props.json');              %#ok<AGROW>
end

for k = 1:numel(candidates)
    if isfile(candidates{k})
        manifestFile = candidates{k};
        break
    end
end

if isempty(manifestFile)
    return
end

propsDir = fileparts(manifestFile);

try
    raw = jsondecode(fileread(manifestFile));
catch err
    warning('hmmSceneManifest:unreadable', ...
        'Prop manifest %s could not be decoded (%s). No props loaded.', ...
        manifestFile, err.identifier);
    return
end

if isstruct(raw) && isfield(raw, 'movables')
    items = fromExplicit(raw.movables);
else
    items = fromBaseline(raw);
end
items = applyMoves(items);

end

%% ------------------------------------------------------------------------
function items = applyMoves(items)
%APPLYMOVES Shift props by cfg.scene.propMoves, the project's own layout over the shared assets.
%   The manifest belongs to the shared asset folder, so the project does not edit it; a move
%   made here reaches every reader of the manifest - the spawn, the registry, the planners and
%   the settle - because they all come through this function. Height is left alone: a move
%   slides a prop along the surface it already stands on.
cfg = hmmConfig();
if ~isfield(cfg.scene, 'propMoves')
    return
end
for m = cfg.scene.propMoves(:)'
    k = find(strcmp({items.name}, m.name), 1);
    if ~isempty(k)
        items(k).position(1:2) = m.xy;
    end
end
end

%% ------------------------------------------------------------------------
function items = fromBaseline(raw)
%FROMBASELINE Key-to-placement map, the form the reused apartment assets ship.
items = struct('name', {}, 'position', {}, 'extent', {}, 'surfaceId', {});
if ~isstruct(raw)
    return
end
keys = sort(fieldnames(raw));
for k = 1:numel(keys)
    entry = raw.(keys{k});
    if ~isstruct(entry) || ~isfield(entry, 'position')
        continue
    end
    pos = double(entry.position(:))';
    if numel(pos) ~= 3 || all(pos == 0)
        % A prop with no placement is not in the scene. The baseline manifest
        % carries a few such entries as slots rather than as objects.
        continue
    end
    if isfield(entry, 'size')
        ext = double(entry.size(:))';
    elseif isfield(entry, 'extent')
        ext = double(entry.extent(:))';
    else
        ext = [0 0 0];
    end
    items(end+1) = struct('name', keys{k}, 'position', pos, ...
        'extent', ext, 'surfaceId', uint32(0));                        %#ok<AGROW>
end
end

%% ------------------------------------------------------------------------
function items = fromExplicit(movables)
%FROMEXPLICIT Array form, which can additionally name a rendered surface.
items = struct('name', {}, 'position', {}, 'extent', {}, 'surfaceId', {});
if iscell(movables)
    movables = [movables{:}];
end
for k = 1:numel(movables)
    it = movables(k);
    if ~all(isfield(it, {'position', 'extent'}))
        continue
    end
    if isfield(it, 'name')
        name = char(it.name);
    elseif isfield(it, 'id')
        name = sprintf('Movable_%d', double(it.id));
    else
        continue
    end
    surf = uint32(0);
    if isfield(it, 'surfaceId')
        surf = uint32(it.surfaceId);
    end
    items(end+1) = struct('name', name, ...
        'position', double(it.position(:))', ...
        'extent', double(it.extent(:))', 'surfaceId', surf);           %#ok<AGROW>
end
[~, order] = sort(string({items.name}));
items = items(order);
end
