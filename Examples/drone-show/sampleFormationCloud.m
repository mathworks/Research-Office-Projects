function [pts, info] = sampleFormationCloud(form, N)
%SAMPLEFORMATIONCLOUD Pick exactly N drone slots off a candidate point cloud.
%
%   pts = sampleFormationCloud(form, N) takes a struct from formationFromMedia and
%   returns an [N x 3] set of points, still in the normalised East/North/Up frame
%   (largest half-extent 1). setupParams scales and places them.
%
%   [pts, info] = ... also returns .usedFill, .nCandidates and .minSep (the closest
%   pair, in normalised units) so the caller can report what it got.
%
%   OUTLINE, FALLING BACK TO FILL. The outline reads best with tens of drones, but
%   a thin or tiny picture can have fewer edge pixels than there are drones. When
%   that happens the filled silhouette is used instead -- automatically, because
%   the alternative is placing two drones on one pixel.
%
%   WHY K-MEANS AND THEN A SNAP. Farthest-point seeding spreads N points over the
%   shape, k-means (Statistics and Machine Learning Toolbox) then equalises the
%   spacing -- but a k-means centroid is a *mean*, so on a curved outline it sits
%   inside the curve. Each centroid is therefore snapped to the nearest candidate
%   point that no other drone has claimed. A drone should be ON the shape.
%
%   Deterministic: the seeding, the Lloyd iterations and the snap are all
%   order-driven, so the same file and the same N always give the same formation
%   and the show is reproducible between runs.
%
%   See also FORMATIONFROMMEDIA, SETUPPARAMS, KMEANS.

if ~isstruct(form) || ~isfield(form, 'cloud')
    error('sampleFormationCloud:badInput', ...
        'form must be a struct from formationFromMedia.');
end
N = double(N);
if ~isscalar(N) || N < 1 || N ~= round(N)
    error('sampleFormationCloud:badCount', 'N must be a positive integer.');
end

cloud = form.cloud;
usedFill = false;
if isfield(form, 'cloudFill') && ~isempty(form.cloudFill) && size(cloud, 1) < N
    % Fewer edge pixels than drones: the outline cannot seat the fleet.
    cloud = form.cloudFill;
    usedFill = true;
end
if isempty(cloud)
    error('sampleFormationCloud:emptyCloud', 'The formation has no candidate points.');
end

% Last resort for a shape that is small even filled (a 5-pixel icon, a two-triangle
% mesh): interpolate midpoints between nearest neighbours until there are enough
% distinct positions. Better than duplicating points, which would stack drones.
cloud = densify(cloud, N);

nCand = size(cloud, 1);
if nCand == N
    pts = cloud;
else
    seeds = cloud(farthestPointSeeds(cloud, N), :);
    if N == 1
        pts = seeds;
    else
        % 'Start' as an explicit matrix keeps kmeans off the global rng; 'singleton'
        % re-seeds an emptied cluster from the worst-fitting point, also deterministic.
        C = kmeans(cloud, N, 'Start', seeds, 'MaxIter', 300, ...
            'EmptyAction', 'singleton', 'Display', 'off');
        C = clusterCentroids(cloud, C, N, seeds);
        pts = snapToCloud(cloud, C);
    end
end

if nargout > 1
    if N > 1
        info.minSep = min(pdist(pts));
    else
        info.minSep = inf;
    end
    info.usedFill = usedFill;
    info.nCandidates = nCand;
end
end

%% ------------------------------------------------------------------------
function cloud = densify(cloud, N)
% Add midpoints until there are at least N distinct candidates. Guarded by an
% iteration cap so a degenerate cloud (every point identical) fails loudly rather
% than looping: dedup below cannot grow it, so the count would never reach N.
for iter = 1:12
    cloud = unique(cloud, 'rows');
    if size(cloud, 1) >= N
        return;
    end
    if size(cloud, 1) < 2
        error('sampleFormationCloud:tooFewPoints', ...
            ['This shape resolves to a single point -- there is nothing to ' ...
             'spread %d drones over.'], N);
    end
    [idx, ~] = knnsearch(cloud, cloud, 'K', 2);
    cloud = [cloud; (cloud + cloud(idx(:, 2), :)) / 2]; %#ok<AGROW>
end
error('sampleFormationCloud:tooFewPoints', ...
    'Could not find %d distinct positions in this shape.', N);
end

%% ------------------------------------------------------------------------
function idx = farthestPointSeeds(cloud, N)
% Greedy farthest-point sampling: start from the most extreme point (deterministic,
% and it guarantees the silhouette's tips get a drone), then repeatedly take the
% candidate furthest from everything chosen so far. Even coverage from the outset,
% which is what stops k-means settling into a lopsided local minimum.
[~, first] = max(sum(cloud.^2, 2));
idx = zeros(N, 1);
idx(1) = first;
d = vecnorm(cloud - cloud(first, :), 2, 2);
for k = 2:N
    [~, nxt] = max(d);
    idx(k) = nxt;
    d = min(d, vecnorm(cloud - cloud(nxt, :), 2, 2));
end
end

%% ------------------------------------------------------------------------
function C = clusterCentroids(cloud, labels, N, seeds)
% kmeans returns labels here (we asked for one output); recompute the centroids.
% An empty label keeps its seed rather than collapsing to the origin -- the origin
% is usually off the shape entirely, and one drone parked in mid-air inside a logo
% is exactly the kind of defect that survives a visual check.
C = seeds;
for k = 1:N
    m = labels == k;
    if any(m)
        C(k, :) = mean(cloud(m, :), 1);
    end
end
end

%% ------------------------------------------------------------------------
function pts = snapToCloud(cloud, C)
% Move each centroid onto the nearest candidate point not already taken. Processed
% in order, and each claim removes that candidate, so no two drones share a slot.
N = size(C, 1);
pts = zeros(N, 3);
taken = false(size(cloud, 1), 1);
for k = 1:N
    d = vecnorm(cloud - C(k, :), 2, 2);
    d(taken) = inf;
    [~, j] = min(d);
    taken(j) = true;
    pts(k, :) = cloud(j, :);
end
end
