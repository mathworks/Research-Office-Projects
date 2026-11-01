function n = hmmSpawnProps(world, parent, cfg)
%HMMSPAWNPROPS Add every movable prop in the scene manifest to the Sim3D world.
%
%   n = HMMSPAWNPROPS(world, parent) reads the prop manifest found by
%   hmmAssetRoots and adds one movable sim3d.Actor per entry that has both a
%   placement and a mesh, PARENTED TO parent, returning how many were added.
%
%   PARENT IS NOT OPTIONAL AND THAT IS THE WHOLE FIX. This function used to call
%   add(world, actor), which put each prop at the world root. It reported "added
%   20 movable props" on every run and the apartment rendered with nothing on any
%   surface - no lamp, no mug, no remote - for the entire history of the project,
%   which means the provider was being asked to ground objects in an empty room.
%   A photograph of the render settled why: in a world created by a Simulation 3D
%   Scene Configuration block, an actor added from a block init script as a
%   SIBLING at the world root does not reach the renderer, while the same actor
%   added as a CHILD of the block's own actor does. The parent is the third
%   argument of add; there is no add(actor, child), because parenting belongs to
%   the world that owns both. Requiring parent here rather than defaulting it is
%   deliberate: a default would let a caller reintroduce the silent version of
%   this bug, and silent is what made it survive so long.
%
%   THE PROPS DO NOT MOVE BECAUSE OF THIS. A child's Translation is relative to
%   its parent, so manifest coordinates stay world coordinates only while the
%   parent sits at the origin with no rotation - which the SceneProps block does,
%   and which is why that block carries no transform of its own.
%
%   THE PROPS ARE ADDED AS AN UNDIFFERENTIATED SET. The manifest says where a
%   prop starts; it does not say which prop the task is about, and nothing here
%   privileges, filters or renames any entry. The actor name is the manifest key
%   PADDED TO A FIXED WIDTH by hmmActorNames, so the registry built by
%   hmmDiscoverMovables, the actors in the world and the name table the plant
%   addresses them with all agree by construction.
%
%   THE PADDING IS NOT COSMETIC. Simulation 3D Robot's TargetActor input carries
%   an actor NAME as a fixed-width uint8 row, and the block does no trimming, so
%   the name the model sends and the name the world holds have to be the same
%   length as well as the same characters. hmmActorNames explains why in full.
%   The MESH filename is still the unpadded key, because that is what is on disk.
%
%   Called from the InitScriptText of the SceneProps actor block, which is where
%   both the World and the Actor handles exist - they are the only two keywords a
%   Simulation 3D Actor init script is given, and this function needs both. One
%   block spawning every prop is deliberate: a block per prop would put the
%   scene's contents in the model, which is exactly the task-specific datum the
%   design forbids.
%
%   See also hmmDiscoverMovables, hmmAssetRoots, hmmSceneManifest.

%   Copyright 2026 The MathWorks, Inc.

arguments
    world
    parent
    cfg struct = hmmConfig()
end

[items, propsDir] = hmmSceneManifest();
n = 0;
if isempty(items)
    return
end

% One source for the names, shared with the plant's lookup table. Read from the
% same manifest in the same order, so actorNames(k) belongs to items(k) and to
% registry ActorId k.
actorNames = hmmActorNames(cfg);

% IDEMPOTENT BY NAME. Simulink evaluates a block's init script once per compile
% phase - three times per run here - and each phase is handed a fresh World, so
% no duplication happens today. The guard costs one property read and means a
% second evaluation against the SAME world cannot silently stack twenty
% coincident meshes, which would break actor lookup by name.
existing = string(world.ActorNames);

% THE MESH LOAD BELOW IS WHY THIS WARNING IS SILENCED HERE AND NOWHERE ELSE.
% load(actor, mesh, scale) turns the FBX hierarchy into one sim3d.Actor per node -
% seven of them for Bottle - and every child comes out Mobility = Static however
% the parent was created. Sim3D's TargetGraspUtils/detachTargetActor then calls
% propagate('Physics'/'Gravity', true, 'unselected') on the released prop, which
% walks into those children and warns once per child per property: fourteen lines
% at every place, out of p-coded toolbox code this example does not own.
%
% MEASURED, so the silence is informed rather than convenient. Of the four
% properties the detach restores, only Physics and Gravity are mobility-gated.
% Collisions and Friction propagate into Static children with no warning and no
% refusal, so the collision propagate below is NOT being silently dropped and the
% ray tracer still has its geometry - which was the one way this could have been
% hiding a real defect instead of noise.
%
% The alternative is propagate(actor, 'Mobility', Movable), which kills the
% warning at its cause and is deliberately NOT taken: it would let the detach's
% Physics = true actually engage, handing a seven-part mesh to the physics engine
% as seven independent dynamic bodies with nothing constraining them to each
% other. Every placement in this example is geometric by design.
warning('off', 'shared_sim3d:sim3dActor:UnsupportedMobilityType');

for k = 1:numel(items)
    mesh = fullfile(propsDir, [items(k).name '.fbx']);
    if ~isfile(mesh) || any(existing == actorNames(k))
        continue
    end
    actor = sim3d.Actor('ActorName', char(actorNames(k)), ...
        'Mobility', sim3d.utils.MobilityTypes.Movable, ...
        'Translation', items(k).position(:)');
    add(world, actor, parent);
    load(actor, mesh, cfg.scene.meshScale);
    % Collision geometry is what the ray tracer measures range against. Without
    % it the props are visible in the render and invisible to perception, so
    % every grounded pixel returns the floor behind the object and the depth
    % gate rejects a target that is plainly there.
    propagate(actor, 'Collisions', true);
    n = n + 1;
end

% THE WORLD'S NAME, FOR THE PLANT-SIDE CODE THAT ACTS ON THE RENDER WHILE THE RUN GOES:
% hmmSettleProp (the stand-in for gravity, at every release) and hmmStyleRobot (the robot's
% surface finish, once per run from hmmAppStateTap). Both write actor properties and neither
% feeds perception, planning or control.
%
% THE NAME, NOT THE HANDLE. This function runs once per COMPILE PHASE - three times per run
% here - and each phase is handed a FRESH world, so a handle retained here is already dead by
% the time the simulation is running (measured: isvalid == 0, "Invalid or deleted object",
% zero actors at t = 5.330 s). The name is a string, it survives, and sim3d.World.getWorld is
% static - so a reader can fetch whichever world is LIVE at the instant it asks.
%
% It began as a diagnosis-only witness, which also kept the handle and let hmmWristRefine's
% tap read every actor's engine pose; that witness is gone, and the name stayed because the
% settle and the finish came to need it.
try
    setappdata(0, 'hmmPropWorldName', char(world.Name));
catch
    setappdata(0, 'hmmPropWorldName', '');
end

fprintf('hmmSpawnProps: added %d movable props.\n', n);

end
