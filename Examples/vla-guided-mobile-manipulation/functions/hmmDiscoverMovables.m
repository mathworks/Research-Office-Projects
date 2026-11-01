function reg = hmmDiscoverMovables(cfg)
%HMMDISCOVERMOVABLES Runtime registry of movable actors in the scene.
%
%   reg = HMMDISCOVERMOVABLES(cfg) returns a struct matching the
%   MovableRegistry bus, built from the prop placement metadata that
%   hmmSceneManifest reads out of the reused apartment assets.
%
%   EVERY PROP IS EQUAL AND NONE IS NAMED HERE. The design requires the scene's
%   movables to reach the system as an undifferentiated set of candidates. The
%   registry therefore holds whatever the manifest lists, in the manifest's own
%   alphabetical order, and the index a prop lands on is its ActorId. Nothing in
%   this function or downstream of it looks a prop up by name: association is
%   geometric, and Gemini's object string is carried for reporting only.
%
%   THE SAME MANIFEST FEEDS THE SCENE AND THE REGISTRY. hmmSpawnProps loads the
%   actors from hmmSceneManifest and names each one after its manifest key, so
%   an actor in the world and its registry entry cannot disagree about position
%   or extent - there is one source, not two that have to be kept in step.
%
%   SurfaceId is the rendered-surface identifier the ray tracer reports, when
%   the manifest supplies one. The baseline manifest does not, so it stays zero
%   and association falls to geometric proximity, which is the documented
%   fallback rather than a degraded path.
%
%   WHEN NOTHING IS FOUND the registry is empty and Valid is false. That is the
%   designed behaviour, not a degraded one. The design forbids task object data
%   in configuration, so there is no fallback list to fall back to: perception
%   reports AmbiguousObject and the run fails with a reason. A fabricated object
%   would make the pipeline look like it worked.
%
%   Entries beyond cfg.actor.maxMovable are ignored and reported at the command
%   line, because a silently truncated registry is an object that cannot be
%   picked for no visible reason.
%
%   See also hmmSceneManifest, hmmSpawnProps, hmmReconstructTargets.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg struct = hmmConfig()
end

nMax = double(cfg.actor.maxMovable);

reg.Count     = uint8(0);
reg.ActorId   = zeros(nMax, 1, 'uint8');
reg.SurfaceId = zeros(nMax, 1, 'uint32');
reg.Position  = zeros(3, nMax);
reg.Extent    = zeros(3, nMax);
reg.Valid     = false;

items = hmmSceneManifest();
if isempty(items)
    return
end

n = min(numel(items), nMax);
if numel(items) > nMax
    fprintf('hmmDiscoverMovables: manifest lists %d movables, registry holds %d.\n', ...
        numel(items), nMax);
end

for k = 1:n
    reg.ActorId(k)     = uint8(k);
    reg.SurfaceId(k)   = items(k).surfaceId;
    reg.Position(:, k) = items(k).position(:);
    reg.Extent(:, k)   = items(k).extent(:);
end

reg.Count = uint8(n);
reg.Valid = n > 0;

end
