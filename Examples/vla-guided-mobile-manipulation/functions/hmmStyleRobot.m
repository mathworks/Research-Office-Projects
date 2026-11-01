function n = hmmStyleRobot(world, cfg)
%HMMSTYLEROBOT Give the rendered robot's actors their surface finish (metallic or matte).
%
%   n = HMMSTYLEROBOT(world, cfg) finds the actor Simulation 3D Robot created for the robot
%   (cfg.render.robotActorName) in the Sim3D world, walks its body actors, and sets on each
%   whose body matches a row of cfg.render.robotFinish the sim3d.Actor properties that row
%   gives. Returns how many actors it styled; 0, and nothing touched, when the robot actor is
%   not in the world yet.
%
%   WHY IT EXISTS. The block builds the robot with sim3d.Actor.load, which takes each body's
%   colour from the rigidBodyTree but gives EVERY body the same finish - Metallic 0.70,
%   Shininess 0.70, Specular 0.50 - so the painted base of the myAGV Pro rendered as metal. The
%   tree has no way to carry a finish, so it is set on the actors themselves. A finish set on
%   an actor after it was created was measured to reach the renderer, and it has to be set
%   late: called from hmmSpawnProps' init script it found no robot in any of three init
%   phases, so hmmAppStateTap calls it once per run instead.
%
%   A body's actor is named after the body, sometimes with a suffix (see the block's
%   RobotSim3DActor/getBodyNameInSim3D), so the rows are anchored at the start of the name
%   only, and the first row that matches wins.
%
%   DISPLAY ONLY AND NEVER FATAL: nothing here reaches perception, planning or control, and a
%   failure is a warning.
%
%   See also hmmSpawnProps, hmmMirrorTree.

%   Copyright 2026 The MathWorks, Inc.

arguments
    world
    cfg struct = hmmConfig()
end

n = 0;
try
    name = cfg.render.robotActorName;
    found = ~isempty(world) && isfield(world.Actors, name);
    % Each call is recorded, styled or not, so a run can say afterwards whether this was early
    % enough (the block creates the robot in its own setup, in an order this cannot choose).
    taps = getappdata(0, 'hmmStyleRobotTap');
    taps = [taps, struct('when', datetime('now'), 'robotFound', found, 'styled', 0)];
    taps = taps(max(1, end - 19):end);
    setappdata(0, 'hmmStyleRobotTap', taps);
    if ~found
        return
    end
    actors = bodyActors(world.Actors.(name));
    for k = 1:numel(actors)
        a = actors{k};
        body = char(a.Name);
        for r = 1:size(cfg.render.robotFinish, 1)
            if isempty(regexp(body, cfg.render.robotFinish{r, 1}, 'once'))
                continue
            end
            props = cfg.render.robotFinish{r, 2};
            for p = fieldnames(props)'
                a.(p{1}) = props.(p{1});
            end
            n = n + 1;
            break
        end
    end
    taps(end).styled = n;
    setappdata(0, 'hmmStyleRobotTap', taps);
    fprintf('hmmStyleRobot: finish set on %d robot actors.\n', n);
catch err
    warning('hmm:styleRobot:failed', 'hmmStyleRobot left the robot as loaded: %s', err.message);
end
end

function out = bodyActors(parent)
%BODYACTORS Every actor below parent, depth first.
out = {};
ch = parent.Children;
for f = fieldnames(ch)'
    c = ch.(f{1});
    out = [out, {c}, bodyActors(c)];                                        %#ok<AGROW>
end
end
