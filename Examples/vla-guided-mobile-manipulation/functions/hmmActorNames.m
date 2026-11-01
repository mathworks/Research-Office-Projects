function [names, table] = hmmActorNames(cfg)
%HMMACTORNAMES Padded Sim3D actor names for the movable props, and the plant's lookup table.
%
%   [names, table] = HMMACTORNAMES(cfg) returns the name each movable prop is
%   given in the Sim3D world, padded to cfg.actor.nameWidth characters, together
%   with the uint8 table the model uses to turn a registry ActorId into that name.
%
%       names   1-by-n string, names(k) is the actor name of manifest entry k
%       table   max(n,1)-by-cfg.actor.nameWidth uint8, row k = char(names(k))
%
%   Row k IS ActorId k. hmmDiscoverMovables assigns ActorId from the manifest's
%   own alphabetical index, and this function walks the same manifest in the same
%   order, so the table needs no key of its own: the identity a prop is given by
%   the registry is the row that names it. Nothing here privileges, filters or
%   renames an entry, and the registry still carries no names - association stays
%   geometric.
%
%   WHY THE NAMES ARE PADDED AT ALL, since a padded name is uglier than a real
%   one. The Simulation 3D Robot block's TargetActor input is addressed by NAME:
%   robotics.slsim3d.internal.util.TargetGraspUtils.processTargetActorInput reads
%   a scalar uint8 as "this input is disabled" and returns an empty name, and
%   Simulation3DRobotBlock.stepImpl then reports isTargetGrasped = false and
%   returns before the grasp is even considered. That is what the model used to
%   do: it put a uint8 ActorId on that port, every capture condition held, and
%   the object never attached - no warning, no error, a well-formed false. The
%   port wants a uint8 ROW that char() turns into the actor's name, which the
%   block looks up as world.Actors.(name).
%
%   A Simulink signal has one width for the whole run, the block does no
%   trimming, and world.Actors.('Mug_Red   ') is not world.Actors.Mug_Red - so
%   trailing blanks would turn a silent missed grasp into a hard error. Padding
%   the ACTOR name itself, with a character that keeps it a valid struct field,
%   is what makes a fixed-width signal and a name lookup agree. hmmSpawnProps
%   names the actors from this same function, so the scene and the table cannot
%   disagree: there is one source, not two that have to be kept in step.
%
%   THE WIDTH IS A CAPACITY, NOT A MEASUREMENT (cfg.actor.nameWidth, 24). A width
%   derived from the longest key would resize a Simulink port whenever the scene
%   changed. A key that does not fit is an ERROR here rather than a truncation,
%   because a truncated name is a lookup that finds the wrong actor or none.
%
%   WITH NO PROPS the table is one row holding cfg.actor.robotName. That row can
%   never be selected as a grasp target - with an empty registry there are no
%   candidates and the capture gate never authorises anything - but the block
%   dereferences the name BEFORE it consults its activate flag, so the row has to
%   name an actor that certainly exists. The robot does.
%
%   See also hmmSceneManifest, hmmDiscoverMovables, hmmSpawnProps.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg struct = hmmConfig()
end

width = double(cfg.actor.nameWidth);
pad   = cfg.actor.namePad;

items = hmmSceneManifest();
names = strings(1, numel(items));

for k = 1:numel(items)
    raw = items(k).name;
    if strlength(raw) > width
        error('hmmActorNames:nameTooLong', ...
            ['Manifest key ''%s'' is %d characters and cfg.actor.nameWidth is %d. ' ...
             'Raise the width rather than shortening the key: the width sizes a ' ...
             'Simulink port, the key is what the scene assets are called.'], ...
            raw, strlength(raw), width);
    end
    if ~isvarname(raw)
        error('hmmActorNames:nameNotIdentifier', ...
            ['Manifest key ''%s'' is not a valid MATLAB identifier, so the Sim3D ' ...
             'world cannot hold it as world.Actors.(name).'], raw);
    end
    names(k) = string(pad(ones(1, width)));
    names(k) = replaceBetween(names(k), 1, strlength(raw), raw);
end

% UNIQUENESS AFTER PADDING, not before. Padding cannot merge two distinct keys of
% the same length, but 'Mug' and 'Mug_' would both become 'Mug_...' and the second
% would silently overwrite the first in world.Actors.
if numel(unique(names)) < numel(names)
    [~, first] = unique(names, 'stable');
    dup = setdiff(1:numel(names), first);
    error('hmmActorNames:nameCollision', ...
        ['Manifest keys collide once padded with ''%s'': ''%s'' is not distinct. ' ...
         'Choose a pad character no key ends with.'], pad, items(dup(1)).name);
end

if isempty(names)
    rows = string(cfg.actor.robotName);
else
    rows = names;
end

table = zeros(numel(rows), width, 'uint8');
for k = 1:numel(rows)
    table(k, :) = uint8(char(pad(ones(1, width))));
    c = char(rows(k));
    table(k, 1:numel(c)) = uint8(c);
end

end
