function [shiftUV, ok, info] = hmmSlideOnSurface(pt, u, v, sv, cfg, objExtent)
%HMMSLIDEONSURFACE The nearest serviceable point on the same surface as a place point.
%
%   [shiftUV, ok, info] = HMMSLIDEONSURFACE(pt, u, v, sv, cfg, objExtent) searches the whole
%   connected top the place point pt sits on - not just the patch the ceiling cameras observed
%   around it - and returns the in-plane offset (along u, v) to the NEAREST point the base can
%   serve (hmmPlaceBearings > 0). ok is false when there is none, and pt is then left for the
%   place gates to refuse. info reports the distance moved, the tier used and how many cells
%   were tried.
%
%   TWO TIERS, NEAREST FIRST IN EACH. The provider's point carries the operator's intent -
%   "the right edge of the table" must stay on the right edge - so the first tier looks only
%   within cfg.percep.placeMaxSlide (0.5 m). Only when nothing there serves does the second
%   tier look over the rest of the same surface, out to cfg.percep.placeMaxSlideFar. Measured
%   on 2026-10-15: Gemini ER points at the middle of a couch seat, and the whole middle scores
%   0 bearings - the coffee table is 0.4 m in front of it - while the couch's left end serves
%   10. A 0.5 m cap refused every couch place as PlaceUnreachable; the far tier moves it to
%   that end instead, and says so in the log.
%
%   THE SURFACE IS THE ENGINE'S NADIR SWEEP, data/shellProbe/floorPlan.mat: cells within
%   cfg.percep.placeSlideHeightTol of the surface height, 4-connected to the cell under pt,
%   and at least half the object's larger extent plus the inset from that surface's edge, so
%   the object rests wholly on it. Movable props are not in the sweep.
%
%   ANCHORED ON THE MAP'S OWN HEIGHT under pt, not on pt(3). pt(3) is fitted from camera
%   depth, and a fitted support height has read 56 mm off the true one in this project; with
%   the comparison made against pt(3) such an error made "the same surface" empty and this
%   search silently did nothing. The map height is used when it is within
%   cfg.percep.placeSlideAnchorTol of pt(3); the candidates keep pt's height and plane.
%
%   cfg.percep.placeSlideMinBearings keeps the chosen point off a one-bearing knife edge; a
%   one-bearing point is used only if nothing better lies in range of its tier.
%
%   IT COSTS ONE RING EVALUATION PER CANDIDATE (~14 ms), nearest first and stopping at the
%   first acceptable one, and it runs only on the path that would otherwise fail.
%
%   See also hmmPlaceBearings, hmmReconstructTargets.

%   Copyright 2026 The MathWorks, Inc.

shiftUV = [0 0];
ok      = false;
info    = struct('moved', 0, 'tier', 0, 'tried', 0);

hm = heightMap();
if isempty(hm)
    return
end
H   = hm.heightMap;
res = hm.resolution;
r0  = round((hm.world(2, 2) - pt(2)) * res) + 1;
c0  = round((pt(1) - hm.world(1, 1)) * res) + 1;
if r0 < 1 || r0 > size(H, 1) || c0 < 1 || c0 > size(H, 2)
    return
end
zRef = pt(3);
if isfinite(H(r0, c0)) && abs(H(r0, c0) - pt(3)) <= cfg.percep.placeSlideAnchorTol
    zRef = H(r0, c0);
end
same = abs(H - zRef) <= cfg.percep.placeSlideHeightTol;
if ~same(r0, c0)
    return
end
surf = bwselect(same, c0, r0, 4);

% Distance from a cell centre to the surface edge is half a cell less than bwdist's centre-to-centre.
keep  = 0.5 * max(double(objExtent(1:2))) + cfg.percep.surfaceInset;
inner = surf & (bwdist(~surf) - 0.5) / res >= keep;
[rr, cc] = find(inner);
% EVERY OTHER CELL, 0.10 m apart: still finer than the base's own arrival tolerance (0.071 m),
% and a quarter of the ring evaluations. The cell under pt is kept on whichever parity it has.
even = mod(rr - r0, 2) == 0 & mod(cc - c0, 2) == 0;
rr = rr(even);
cc = cc(even);
xy = [hm.world(1, 1) + (cc - 1) / res, hm.world(2, 2) - (rr - 1) / res];
d  = vecnorm(xy - pt(1:2)', 2, 2);
[d, ord] = sort(d);
xy = xy(ord, :);

% A CHEAP NECESSARY CONDITION FIRST. A point can only be served if the base fits somewhere
% within the largest stand-off radius the arm allows at this height (hmmStandoffRadii), so the
% cells where the base fits at ANY of 16 headings are found once, in one batched validator call,
% and a candidate with none of them in range is skipped without its ~17 ring evaluations
% (~14 ms each). Without this a couch seat - every middle cell refused at every radius - took
% minutes; the necessary condition is all it takes to rule those cells out.
[loL, hiL] = hmmArmLeverBand(cfg, pt(3), double(objExtent(:)));
rMax = max([cfg.sup.placeStandoff, hmmStandoffRadii(cfg, loL, hiL, cfg.sup.placeStandoff)]);
fitXY = baseFitCells(xy, rMax, sv, res);
slack = hypot(cfg.plan.baseGoalTolerance(1), cfg.plan.baseGoalTolerance(2)) + 2 / res;
if isempty(fitXY)
    xy   = zeros(0, 2);
    d    = zeros(0, 1);
    rMin = zeros(0, 1);
else
    % The nearest fitting base cell also bounds the radius from BELOW: no ring closer than it
    % can hold a free pose, so hmmPlaceBearings skips those rungs (rMin), with a cell of slack.
    dFit = zeros(size(xy, 1), 1);
    for k = 1:size(xy, 1)
        dFit(k) = min(vecnorm(fitXY - xy(k, :), 2, 2));
    end
    near = dFit <= rMax + slack;
    xy   = xy(near, :);
    d    = d(near);
    rMin = max(dFit(near) - slack, 0);
end

% A CLEAR REACH IS PREFERRED OVER A NEAR ONE. A point served only by a base pose whose straight
% line to it crosses something taller than the surface - the couch seat reached from behind,
% over its 0.67 m backrest, at the arm's 0.82 m limit (2026-10-15) - is a place the 2-D ring
% accepts and the 3-D arm planner is likely to refuse. So within each tier a candidate with a
% clear line wins, and one that reaches over furniture is used only if no clear one exists
% anywhere on the surface.
B = [u(1:2), v(1:2)];
limits = [cfg.percep.placeMaxSlide, cfg.percep.placeMaxSlideFar];
from = 0;
overReach = [];
info.reachOver = false;
for tier = 1:2
    in = find(d > from & d <= limits(tier));
    fallback = [];
    for k = in(:)'
        ab = (B \ (xy(k, :)' - pt(1:2)))';
        q  = pt + ab(1) * u + ab(2) * v;
        % ITS OWN HEIGHT, FROM THE MAP. "The same surface" spans cfg.percep.placeSlideHeightTol,
        % so cushions either side of a seam are one surface; the object goes down at the height
        % of the cell it lands on, not at the height Gemini's point had.
        hq = H(round((hm.world(2, 2) - q(2)) * res) + 1, round((q(1) - hm.world(1, 1)) * res) + 1);
        if isfinite(hq)
            q(3) = hq;
            ab(3) = hq - pt(3);
        end
        [nF, pose] = hmmPlaceBearings(q, sv, cfg, objExtent, rMin(k));
        info.tried = info.tried + 1;
        if nF <= 0
            continue
        end
        if hmmPlaceReachesOver(pose, q, cfg, objExtent)
            if isempty(overReach)
                overReach = ab;
            end
            continue
        end
        if nF >= cfg.percep.placeSlideMinBearings
            shiftUV = ab;
            ok = true;
            break
        end
        if isempty(fallback)
            fallback = ab;
        end
    end
    if ~ok && ~isempty(fallback)
        shiftUV = fallback;
        ok = true;
    end
    if ok
        info.tier = tier;
        break
    end
    from = limits(tier);
end
if ~ok && ~isempty(overReach)
    shiftUV = overReach;
    ok = true;
    info.tier = 2;
    info.reachOver = true;
end

info.moved = norm(B * shiftUV(1:2)');
if ok && info.tier == 1
    fprintf(['hmmSlideOnSurface: place point slid %.0f mm along its surface to a ' ...
             'serviceable spot.\n'], 1000 * info.moved);
elseif ok
    fprintf(['hmmSlideOnSurface: nothing serviceable within %.2f m of the place point; ' ...
             'moved it %.2f m along the same surface to the nearest spot that is.\n'], ...
             cfg.percep.placeMaxSlide, info.moved);
else
    fprintf(['hmmSlideOnSurface: no serviceable spot anywhere on the place surface ' ...
             '(%d candidates tried).\n'], info.tried);
end
end


function fitXY = baseFitCells(xy, rMax, sv, res)
%BASEFITCELLS Base-centre cells around the candidates where the base fits at some heading.
fitXY = zeros(0, 2);
if isempty(xy)
    return
end
step = 1 / res;
lo = min(xy, [], 1) - rMax - step;
hi = max(xy, [], 1) + rMax + step;
[gx, gy] = meshgrid(lo(1):step:hi(1), lo(2):step:hi(2));
P  = [gx(:), gy(:)];
th = (0:15) * pi / 8;
ok = false(size(P, 1), 1);
for h = th
    ok = ok | isStateValid(sv, [P, repmat(h, size(P, 1), 1)]);
end
fitXY = P(ok, :);
end

function hm = heightMap()
%HEIGHTMAP The nadir-sweep surface heights, loaded once per session, or empty.
persistent held
if isempty(held)
    f = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'shellProbe', ...
        'floorPlan.mat');
    if isfile(f)
        S = load(f, 'report');
        held = {S.report};
    else
        held = {[]};
    end
end
hm = held{1};
end
