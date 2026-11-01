function meshFile = hmmLoadSceneShell(actor, cfg)
%HMMLOADSCENESHELL Load the apartment shell into a Sim3D actor.
%
%   meshFile = HMMLOADSCENESHELL(actor) finds the reused apartment mesh with
%   hmmAssetRoots, loads it into actor at the configured scale, makes it static
%   and gives it collision geometry, and returns the file it used.
%
%   Called from the InitScriptText of the Plant's SceneShell actor block. THE
%   PATH IS DISCOVERED, NEVER COMMITTED. An absolute mesh path typed into a
%   block dialog is meaningful on exactly one machine, and the design's rule
%   about not committing machine-specific paths is not limited to credentials.
%
%   COLLISION IS PROPAGATED BUT PRECISE CONTACTS ARE NOT. The shell is what the
%   ray tracer measures range against and what the base has to avoid, so it must
%   be collidable; per-triangle contact resolution on a 24 MB architectural mesh
%   costs frame time the first release has no use for, because the base plant is
%   kinematic.
%
%   When the mesh is not found the actor is left empty and a warning says so.
%   That is preferable to a substituted box: an empty room renders as an empty
%   room, whereas a stand-in looks like a scene and grounds like nothing.
%
%   See also hmmAssetRoots, hmmSpawnProps, hmmSceneManifest.

%   Copyright 2026 The MathWorks, Inc.

arguments
    actor
    cfg struct = hmmConfig()
end

meshFile = '';
roots = hmmAssetRoots();
for k = 1:numel(roots)
    candidate = fullfile(roots{k}, cfg.scene.staticMesh);
    if isfile(candidate)
        meshFile = candidate;
        break
    end
end

if isempty(meshFile)
    warning('hmmLoadSceneShell:notFound', ...
        ['Apartment shell %s was not found beside this project. The scene ' ...
         'will render empty and every grounded pixel will report DepthInvalid.'], ...
        cfg.scene.staticMesh);
    return
end

actor.Mobility = sim3d.utils.MobilityTypes.Static;
load(actor, meshFile, cfg.scene.meshScale);
propagate(actor, 'Collisions', true);
propagate(actor, 'PreciseContacts', false);

fprintf('hmmLoadSceneShell: loaded %s.\n', cfg.scene.staticMesh);

end
