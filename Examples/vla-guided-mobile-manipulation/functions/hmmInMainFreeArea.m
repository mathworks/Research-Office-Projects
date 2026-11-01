function tf = hmmInMainFreeArea(pos)
%HMMINMAINFREEAREA Is a base position in the apartment's main free area, not a sealed pocket?
%
%   tf = HMMINMAINFREEAREA(pos) is true when the base centre at pos ([x y], world) lies in
%   the largest connected region of the shipped nav map's free cells, after obstacles are
%   grown by the footprint's inscribed radius (half-width plus the planner margin) - the same
%   growth hmmPlanBasePath's connectedOnGrid screens a route with. True also when there is no
%   map, so a missing map never refuses anything.
%
%   WHY. hmmStandoffRing judges a base pose by whether the footprint FITS there, not by
%   whether the base can GET there. That never mattered while the place stand-off was one
%   0.48 m radius hugging the furniture; with the radius ladder out to 0.82 m the ring found
%   free spots BEHIND furniture. On 2026-10-16 the bed place chose (9.50, -6.45), between the
%   bed and the wall: free, sealed off, and refused BaseGoalUnreachable in 0.6 s on both
%   tries. The robot always drives in the main area, so a candidate outside it is a pocket.
%
%   ERRS TOWARDS YES, like connectedOnGrid: obstacles are grown half a cell LESS than the
%   inscribed radius and the test is rotation-blind, so a reachable pose is never refused;
%   a false yes only costs the planner's own refusal, which is the behaviour before this.
%
%   See also hmmStandoffRing, hmmWallBetween, hmmPlanBasePath.

%   Copyright 2026 The MathWorks, Inc.

tf = true;
[mask, map] = cached();
if isempty(mask)
    return
end
ij = world2grid(map, reshape(double(pos(1:2)), 1, 2));
ij = reshape(ij, 1, 2);
if any(ij < 1) || ij(1) > size(mask, 1) || ij(2) > size(mask, 2)
    tf = false;                          % outside the map is not the apartment
    return
end
tf = mask(ij(1), ij(2));
end

function [mask, map] = cached()
persistent held
if isempty(held)
    cfg = hmmConfig();
    grid = [];
    if ~(isfield(cfg.map, 'mode') && strcmp(cfg.map.mode, 'live'))
        grid = hmmLoadNavMap(cfg);
    end
    if isempty(grid)
        held = {[], []};
    else
        map = occupancyMap(grid, cfg.map.resolution);
        map.GridLocationInWorld = cfg.map.origin;
        occ  = grid > cfg.map.occupiedThreshold;
        geom = hmmBaseFootprint(cfg, zeros(3, 1));
        clear = bwdist(occ) / double(cfg.map.resolution);  % metres to the nearest obstacle cell
        free = clear > geom.inscribed - 0.5 / double(cfg.map.resolution);
        cc = bwconncomp(free, 8);
        mask = false(size(free));
        if cc.NumObjects > 0
            [~, k] = max(cellfun(@numel, cc.PixelIdxList));
            mask(cc.PixelIdxList{k}) = true;
        end
        held = {mask, map};
    end
end
mask = held{1};
map  = held{2};
end
