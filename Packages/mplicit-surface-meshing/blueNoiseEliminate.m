function keep = blueNoiseEliminate(P, numKeep, area, alpha)
%BLUENOISEELIMINATE Weighted sample elimination for blue-noise point sets.
%
%   keep = blueNoiseEliminate(P, numKeep, area) selects numKeep indices of the
%   M-by-3 candidate points P (M > numKeep) so the retained points have an
%   even, blue-noise (Poisson-disk-like) distribution. It implements the
%   greedy weighted sample elimination of Yuksel, "Sample Elimination for
%   Generating Poisson Disk Sample Sets" (Eurographics 2015): candidates are
%   weighted by how crowded their neighbourhood is, and the most crowded
%   sample is repeatedly removed (its neighbours' weights decreased) until
%   numKeep remain.
%
%   area is the total surface area the samples cover (used to set the
%   neighbourhood radius for a 2-manifold). alpha (default 8) is the weight
%   falloff exponent.
%
%   keep is a column of indices into P, so points P(keep,:) and any parallel
%   per-sample data (e.g. normals) can be subselected together.
%
%   See also samplePointCloudFromMesh.

if nargin < 4 || isempty(alpha)
    alpha = 8;
end

M = size(P, 1);
if numKeep >= M
    keep = (1:M).';
    return
end

% Neighbourhood radius for a target of numKeep samples over a 2-manifold of
% the given area (Yuksel, Sec. 3.1), and the elimination cutoff 2*r_max.
rmax = sqrt(area / (2 * sqrt(3) * numKeep));
dmax = 2 * rmax;
wfun = @(d) (1 - min(d, dmax) ./ dmax) .^ alpha;

% All neighbours within dmax (KD-tree range search).
mdl = KDTreeSearcher(P);
[idxC, dC] = rangesearch(mdl, P, dmax);

% Vectorized initial weights: sum of wfun over each point's neighbours,
% excluding the point itself (distance 0).
cnt = cellfun(@numel, idxC);
src = repelem((1:M).', cnt);
nbAll = [idxC{:}].';
dAll = [dC{:}].';
notSelf = nbAll ~= src;
w = accumarray(src(notSelf), wfun(dAll(notSelf)), [M 1]);

alive = true(M, 1);
numAlive = M;

% Max-heap of (weight, id) with lazy deletion: when a weight is decreased we
% push a fresh entry rather than decrease-key; stale entries are skipped on
% pop by comparing the stored weight against the live weight.
cap = numel(nbAll) + M + 1;        % generous: one push per neighbour edge
hW = zeros(cap, 1);
hI = zeros(cap, 1);
hn = 0;
for i = 1:M
    heapPush(w(i), i);
end

while numAlive > numKeep
    [wi, i] = heapPop();
    if i == 0
        break                       % heap empty (should not happen)
    end
    if ~alive(i) || wi ~= w(i)
        continue                    % stale entry
    end
    % Remove the most crowded sample and relieve its neighbours.
    alive(i) = false;
    numAlive = numAlive - 1;
    nb = idxC{i};
    dd = dC{i};
    for t = 1:numel(nb)
        j = nb(t);
        if alive(j) && j ~= i
            w(j) = w(j) - wfun(dd(t));
            heapPush(w(j), j);
        end
    end
end

keep = find(alive);

    % ---- binary max-heap (nested, shares hW/hI/hn) ------------------------
    function heapPush(weight, id)
        hn = hn + 1;
        if hn > numel(hW)
            hW(2 * numel(hW)) = 0;  % grow by doubling
            hI(2 * numel(hI)) = 0;
        end
        hW(hn) = weight;
        hI(hn) = id;
        c = hn;
        while c > 1
            p = floor(c / 2);
            if hW(p) >= hW(c)
                break
            end
            [hW(p), hW(c)] = deal(hW(c), hW(p));
            [hI(p), hI(c)] = deal(hI(c), hI(p));
            c = p;
        end
    end

    function [weight, id] = heapPop()
        if hn == 0
            weight = 0; id = 0;
            return
        end
        weight = hW(1);
        id = hI(1);
        hW(1) = hW(hn);
        hI(1) = hI(hn);
        hn = hn - 1;
        p = 1;
        while true
            l = 2 * p;
            r = l + 1;
            big = p;
            if l <= hn && hW(l) > hW(big), big = l; end
            if r <= hn && hW(r) > hW(big), big = r; end
            if big == p
                break
            end
            [hW(p), hW(big)] = deal(hW(big), hW(p));
            [hI(p), hI(big)] = deal(hI(big), hI(p));
            p = big;
        end
    end
end
