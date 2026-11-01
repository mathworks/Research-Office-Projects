function [pTop, nRegion, riseMax, status, pPivot, zUsed, zMeasured] = ...
    hmmSegmentTopFace(frame, pixel, zSupport, extent, cfg)
%HMMSEGMENTTOPFACE Locate an object's top face in a wrist depth frame, from a seed pixel.
%
%   [pTop, nRegion, riseMax, status, pPivot, zUsed, zMeasured] =
%       HMMSEGMENTTOPFACE(frame, pixel, zSupport, extent, cfg)
%   returns a WORLD point on the object's top face as pTop - xy the object's vertical axis, z the
%   height of its top face - the number of pixels it was averaged over as nRegion, the greatest
%   height the segmented region reached above zUsed as riseMax, and one of cfg.refine's status
%   codes.
%
%   pPivot is a SECOND world point measured from the same region: the midpoint of the banded
%   footprint's bounding box, which is where the renderer believes the object's origin is. It is
%   not an alternative to pTop and must never be used as the grasp aim - see the bounding-box
%   paragraph below - but pPivot(1:2) - pTop(1:2) is the object's PIVOT OFFSET, measured for
%   whatever object is actually in front of the lens, and that is the quantity
%   hmmCarryTranslation needs to stop a grasped prop snapping sideways.
%
%   zUsed is the support height the band was actually taken against, and zMeasured says whether
%   it was READ OUT OF THE FRAME or is still the caller's zSupport. Callers that gate on rise must
%   measure it against zUsed rather than against what they passed in, or they will reject exactly
%   the frames the measured plane just rescued.
%
%   zMeasured IS WHAT LETS A CALLER MEASURE AN OBJECT'S HEIGHT, and that is why the plane is now
%   measured on every frame rather than only as a retry. When it is true, pTop(3) - zUsed is the
%   object's height, taken entirely from this one depth image: the top face from the plateau above,
%   the surface it stands on from the annulus around it. Measured on the shipped fixture that
%   difference is 0.0899 m against a manifest Extent(3) of 0.0900 - 0.1 mm - and on the retained
%   live frame 0.0898 m. When it is false no plane could be read and the caller has nothing better
%   than whatever height it brought with it, which for an object outside the manifest is
%   cfg.percep.extentAssumed and is a guess.
%
%   frame is hmmBackprojectDepth's frame struct - Valid, Depth, Intrinsics, Rotation,
%   Translation, ImageSize - and every convention here is taken from that function so the two
%   cannot disagree: planar depth as the sensor +X, cfg.depth.renderSentinel as the miss
%   marker, and eul2rotm([yaw, -pitch, -roll], 'ZYX') for Sim3D's left-handed pitch and roll.
%
%   WHY A REGION AND NOT THE ANSWERED PIXEL. The provider returns one pixel, and reading depth
%   at one pixel cannot locate an open-topped object. Measured on the rendered wrist frame of
%   Mug_Red, whose top face is an ANNULUS: every plateau pixel lies 27.6 to 35.5 px from the
%   plateau's own centroid, so the centroid pixel falls in the HOLE and ranges to the mug's
%   inner floor at z 0.55324 - 87 mm below the rim and within 3 mm of the tabletop, which the
%   rise gate then correctly rejects. No single pixel carries both facts: the centroid pixel
%   gives the axis but a depth 87 mm too far, and any rim pixel gives the height but an xy
%   displaced by the object's radius. So the pixel becomes a SEED and the geometry comes from
%   the region it seeds.
%
%   AND THAT GENERALISES, WHICH IS THE WHOLE POINT - a disc and an annulus share a centroid. A
%   solid top yields a filled plateau, an open top a ring, a dome a cap, and all three centre
%   on the object's vertical axis, which is where the jaws have to go.
%
%   A BOUNDING-BOX CENTRE IS NOT THE AIM, AND IT IS NOW RETURNED ANYWAY, WHICH IS THE WHOLE
%   POINT OF pPivot. robotics.slsim3d TargetGraspUtils/attachTargetActor does not weld a grasped
%   prop where the jaws closed; it pins the prop's PIVOT - its bounding-box centre - to the tool
%   frame, so with Mug_Red's 101 x 70 mm extent the handle puts that pivot 15.5 mm off the body
%   axis and the mug visibly snaps sideways by that much the instant the gripper closes
%   (measured: dx -15.9, dy +0.6 mm). hmmCarryTranslation fixes the same defect in z.
%
%   AIMING AT THE BOX CENTRE WAS TRIED AND REVERTED, and that verdict stands: it does remove the
%   snap, and it also moves the JAWS 15.5 mm off the mug's body, which on the render is far worse
%   than the artefact it cures - the gripper closes nearly clear of the object. The two targets
%   are irreconcilable in a single answer, so this function now returns BOTH and lets the two
%   consumers take the one each needs. pTop, the plateau centroid, stays the aim because it is
%   the object's axis. pPivot, the band's bounding-box midpoint, goes to the carry parameter.
%
%   THE PREMISE THAT USED TO CLOSE THIS PARAGRAPH WAS FALSE, and it is worth saying so because it
%   is what kept the offset a hand-tabulated constant for so long. It read: "the snap is a
%   property of a mask parameter that has no lateral value this function could supply - the
%   correct one depends on the prop's yaw and hmmSceneManifest carries no rotation field." Both
%   halves fail. The parameter does take a lateral value - Simulation3DRobotBlock declares
%   TargetEndEffectorTranslation outside its Nontunable block and re-reads it every step, so a
%   set_param during a running simulation is accepted (measured, not inferred). And the yaw is
%   not needed from the manifest: this function sees the prop in whatever yaw it is actually
%   standing in, so the difference pPivot - pTop is already the right world vector. Measured on a
%   live wrist frame of the non-red Mug, whose row was NOT the one the old table was calibrated
%   from: pPivot landed 1.07 mm from the engine's own reported actor origin, and the offset came
%   out [14.95 0.36] mm against a tabulated [15.50 0.40]. A run-time measurement therefore
%   reproduces the hand-measured constant to 0.6 mm while needing no object name at all.
%
%   IT HAS TO BE THE BAND'S BOX AND NOT THE PLATEAU'S, which is not obvious and was measured.
%   A mug's handle hangs BELOW its rim, so it is in the band and absent from the plateau - that
%   is exactly why the plateau is a good axis estimator - and the pivot is the box around body
%   AND handle. On the frame above: band box 1.07 mm from truth, plateau box 16.1 mm, plateau
%   centroid 15.8 mm, band mean 11.7 mm. Only the first measures what the renderer pins.
%
%   THE HEIGHT BAND DOES TWO JOBS. It is the object-versus-tabletop discriminator cfg.refine.
%   riseBand was introduced for, and it is also what keeps the ROBOT out of the measurement:
%   the jaws hang about 0.10 m below the lens and are the nearest thing in frame, so "every
%   pixel above the support plane" returns 19620 px rising 0.3548 m - the gripper. Banding the
%   height returns 2406 px and nothing but the object.
%
%   THE SEARCH WINDOW IS CENTRED ON THE SEED RAY WHERE IT MEETS THE SUPPORT PLANE, not on the
%   seed's own ranged point. The ranged point is exactly what is unreliable for a concave
%   object; the ray's direction is not, and intersecting it with a plane the caller already
%   knows costs nothing and cannot be thrown off by what the ray happened to hit.
%
%   zSupport IS A HINT AND NOT A MEASUREMENT, AND TAKING IT ON TRUST KILLED THE WHOLE FEATURE IN
%   LIVE RUNS. The caller's support height is ObjectTarget.Position(3) from the wide ceiling
%   grounding, and that number carries a known systematic error: the ceiling ray cells are wider
%   than the object, so the cluster whose centroid becomes Position is largely tabletop, and on
%   the current [72 128] grid it reads about 56 mm HIGH. The band is zSupport + riseBand * height,
%   a 90 mm window for a 90 mm object, so a 56 mm error is most of the window. Measured on a
%   retained live wrist frame: the hint was 0.4565 against a true support of 0.4000, the band came
%   out [0.5015 0.5915], the mug's top face sits at 0.4900, the band was EMPTY, and this function
%   answered STATUS_NOOBJECT. hmmWristRefine then kept the coarse point, which is its documented
%   advisory behaviour, so the run succeeded and the only trace was a status code. The refinement
%   was not being rejected on its merits - it was never being given a window to look in.
%
%   SO THE PLANE IS MEASURED WHEN THE HINT FAILS, AND ONLY THEN. If the hinted band holds enough
%   pixels nothing changes, which is what keeps every frame that works today working identically -
%   including the shipped replay fixture. If it does not, supportHeight below reads the plane out
%   of the frame: the strongest flat horizontal mode in an ANNULUS between one and two window radii
%   of the seed crossing, which is outside the object by construction, restricted to within two
%   object heights of the hint. The annulus is what keeps the object itself out of its own support
%   estimate; the two-height bound is what stops the FLOOR winning, and it is also the precise
%   sense in which the hint is still used - near enough to disambiguate, not trusted to be exact.
%   On the frame above it returned 0.4002 against a truth of 0.4000, the band then held 1465 px,
%   riseMax came out 0.0900 against an extent of 0.0900, and the refinement applied.
%
%   STATUS. STATUS_APPLIED when a region was found and measured. STATUS_NOFRAME for an invalid
%   frame, STATUS_NODEPTH when the frame carries no usable depth at all, and STATUS_NOOBJECT
%   when nothing inside the height band survives near the seed, which the caller degrades to the
%   coarse point on.
%
%   WHAT NOOBJECT DOES AND DOES NOT MEAN, stated precisely because the obvious reading is too
%   strong. It does NOT mean "the seed pixel was not on the object" - the window is centred on
%   the seed ray and is one and a half object widths across, so a seed that misses the object
%   but lands NEAR it still finds it, and that is a feature rather than a leak. Measured by
%   sweeping the seed down the shipped frame: the mug is still found and measured exactly at 60
%   px of seed error, found with a clipped 955-pixel region at 120 px, and declined at 180 px
%   and beyond. So NOOBJECT means "no object of about the expected height anywhere near where you
%   pointed". The guard against locking onto the WRONG object is not this status but the
%   component selection below plus the caller's cfg.refine.maxShift cap.
%
%   See also hmmBackprojectDepth, hmmWristRefine, hmmConfig.

%   Copyright 2026 The MathWorks, Inc.

pTop      = zeros(3, 1);
nRegion   = 0;
riseMax   = 0;
pPivot    = zeros(3, 1);
zUsed     = double(zSupport);
zMeasured = false;

if ~frame.Valid
    status = cfg.refine.STATUS_NOFRAME;
    return
end

D     = double(frame.Depth);
valid = D < cfg.depth.renderSentinel;
if ~any(valid(:))
    status = cfg.refine.STATUS_NODEPTH;
    return
end

K  = double(frame.Intrinsics);
fx = K(1);
fy = K(2);
cx = K(3);
cy = K(4);

rot = double(frame.Rotation);
R   = eul2rotm([rot(3), -rot(2), -rot(1)], 'ZYX');
t   = reshape(double(frame.Translation), 3, 1);

% EVERY PIXEL TO WORLD, VECTORISED. This runs once per refinement - once per pick - so the
% whole frame costs less than the HTTP call that preceded it, and doing it wholesale keeps the
% arithmetic identical to hmmBackprojectDepth's single-pixel form rather than a windowed
% variant of it that could drift.
[rGrid, cGrid] = ndgrid(1:size(D, 1), 1:size(D, 2));
d      = D(:)';
pSens  = [d; ((cGrid(:)' - cx) / fx) .* d; (-(rGrid(:)' - cy) / fy) .* d];
pWorld = R * pSens + t;
X = reshape(pWorld(1, :), size(D));
Y = reshape(pWorld(2, :), size(D));
Z = reshape(pWorld(3, :), size(D));

% The seed ray, and where it crosses the support plane. A ray that does not descend cannot
% cross it; the mount aims downward by construction, so this is a guard and not a case.
dirW = R * [1; (double(pixel(2)) - cx) / fx; -(double(pixel(1)) - cy) / fy];
if dirW(3) >= -1e-9
    status = cfg.refine.STATUS_NOOBJECT;
    return
end

% The window, and the height band inside it.
height = double(extent(3));
radius = max(double(cfg.refine.searchRadiusMin), ...
             double(cfg.refine.searchRadiusScale) * max(double(extent(1:2))));

centre = t + ((zUsed - t(3)) / dirW(3)) * dirW;
band   = bandMask(valid, X, Y, Z, centre, radius, zUsed, height, cfg);

% THE SUPPORT PLANE, READ OUT OF THE FRAME ON EVERY PASS - see the header. This used to be a
% RETRY, reached only when the caller's hint banded nothing. That rescued the band but it left
% zUsed equal to the hint on every frame that worked first time, and the hint is the wide
% grounding's Position(3), which reads about 56 mm high on a 90 mm object. Measuring the plane
% every time is what turns pTop(3) - zUsed into the OBJECT'S HEIGHT instead of a number that is
% only as good as the hint - and a measured height is the only kind that exists for an object no
% manifest describes.
%
% IT IS NO-WORSE THAN THE RETRY IT REPLACES, BY CONSTRUCTION. The hint's band is computed first,
% above, and the measured plane replaces it only if its OWN band also holds a region; a frame that
% segments today therefore segments the same region tomorrow. The plane has to differ from the hint
% by more than one plateau tolerance to be worth a second pass - inside that, the two are the same
% flat surface and the hint is already confirmed, which is itself worth knowing and is what
% zMeasured then reports. The cost is one annulus histogram per refinement, once per pick, against
% a whole-frame backprojection already paid for above.
%
% A PLANE THAT EXPLAINS NO REGION IS NOT BELIEVED. If the measured height bands nothing at the
% expected rise then whatever the annulus found is not the surface this object stands on - a
% neighbouring piece of furniture, a sill, the floor through a gap - so the hint's band is kept and
% zMeasured goes back to false rather than handing the caller a height datum built on it.
zMeas = supportHeight(valid, X, Y, Z, centre, radius, zUsed, height, cfg);
if ~isnan(zMeas)
    zMeasured = true;
    if abs(zMeas - zUsed) > double(cfg.refine.plateauTol)
        centreMeas = t + ((zMeas - t(3)) / dirW(3)) * dirW;
        bandMeas   = bandMask(valid, X, Y, Z, centreMeas, radius, zMeas, height, cfg);
        if nnz(bandMeas) >= double(cfg.refine.minRegionPixels)
            zUsed  = zMeas;
            centre = centreMeas;
            band   = bandMeas;
        else
            zMeasured = false;
        end
    end
end

if nnz(band) < double(cfg.refine.minRegionPixels)
    status = cfg.refine.STATUS_NOOBJECT;
    return
end

% ONE OBJECT, NOT THE AVERAGE OF TWO. The height band and the search radius together can admit
% a NEIGHBOUR - a second mug 0.15 m away stands in the same band - and averaging two blobs would
% put the grasp point in the gap between them, which is the one answer that is wrong about both.
% So the band is split into connected components and only the component nearest the window
% centre survives. bwconncomp is the right tool for this and it is already a dependency of the
% perception path; hand-rolling a flood fill here would be slower and less tested.
%
% NEAREST TO THE WINDOW CENTRE, NOT CONTAINING THE SEED PIXEL. For an open object the seed pixel
% often falls in the HOLE and so belongs to no component at all - that is the defect this whole
% function exists to fix - whereas the seed ray's support-plane crossing is a point on the
% object's axis and is the right thing to measure proximity from.
CC = bwconncomp(band, 8);
if CC.NumObjects > 1
    bestK = 1;
    bestD = inf;
    for k = 1:CC.NumObjects
        idx = CC.PixelIdxList{k};
        dk  = hypot(mean(X(idx)) - centre(1), mean(Y(idx)) - centre(2));
        if dk < bestD
            bestD = dk;
            bestK = k;
        end
    end
    band = false(size(band));
    band(CC.PixelIdxList{bestK}) = true;
    if nnz(band) < double(cfg.refine.minRegionPixels)
        status = cfg.refine.STATUS_NOOBJECT;
        return
    end
end
riseMax = max(Z(band)) - zUsed;

% THE PLATEAU, WITH THE BAND AS ITS FALLBACK. The plateau is the better estimator and it is
% preferred; if it is too thin to average - a strongly tilted or very shallow top - the band it
% came from is still a measurement of the same object and is used instead, because declining
% here would throw away a correction over a choice of estimator rather than over the evidence.
region = band & Z > max(Z(band)) - double(cfg.refine.plateauTol);
if nnz(region) < double(cfg.refine.minRegionPixels)
    region = band;
end

% THE PLATEAU'S CENTROID, WHICH IS THE OBJECT'S AXIS, AND THE AXIS IS WHAT THE JAWS WANT. See
% the header for why the bounding-box centre is not this answer, and is returned separately.
nRegion = nnz(region);
pTop    = [mean(X(region)); mean(Y(region)); mean(Z(region))];

% THE PIVOT, FROM THE BAND AND NOT THE PLATEAU. This is where the renderer will put the prop's
% origin when the jaws close, so pPivot(1:2) - pTop(1:2) is the sideways snap, measured rather
% than tabulated. The band is the right set because a handle hangs below the rim: it is in the
% band, it is out of the plateau, and it is half the reason the box centre is off the axis at all.
% z is carried over from the plateau so pPivot is a point on the top face like pTop, which keeps
% the two differenceable without a caller having to know which z each one means.
xBand   = X(band);
yBand   = Y(band);
pPivot  = [0.5 * (min(xBand) + max(xBand)); 0.5 * (min(yBand) + max(yBand)); pTop(3)];
status  = cfg.refine.STATUS_APPLIED;

end

%% ------------------------------------------------------------------------
function m = bandMask(valid, X, Y, Z, centre, radius, zRef, height, cfg)
%BANDMASK The window-and-height-band test, in one place because it is now applied twice.
%   Factored out when the support-height retry was added, so the second pass cannot drift from
%   the first - which would be the worst kind of bug here, a fallback that segments a slightly
%   different region than the primary path and so reports a different object geometry.
m = valid ...
    & hypot(X - centre(1), Y - centre(2)) < radius ...
    & Z >= zRef + cfg.refine.riseBand(1) * height ...
    & Z <= zRef + cfg.refine.riseBand(2) * height;

end

%% ------------------------------------------------------------------------
function z = supportHeight(valid, X, Y, Z, centre, radius, zHint, height, cfg)
%SUPPORTHEIGHT The plane the object is standing on, read out of the frame. NaN if none.
%
%   AN ANNULUS, NOT A DISC, AND THAT IS THE WHOLE TRICK. The object occupies the middle of the
%   window, so a disc would measure the object's own top face as its support and the band would
%   then sit a full height too high. Between one and two window radii there is nothing of the
%   object left - radius is cfg.refine.searchRadiusScale (1.5) times its largest horizontal
%   extent, so the inner edge already stands clear of a centred object by half that extent - and
%   what remains is the surface it rests on.
%
%   THE HINT STILL CONSTRAINS THE ANSWER, BY TWO OBJECT HEIGHTS. The annulus also sees the floor,
%   and in a room with a small table the floor can easily be the larger mode; bounding the search
%   to the hint's neighbourhood removes it without trusting the hint to be right. Two heights is
%   chosen against the error this is correcting: the wide grounding's Position(3) reads about 56 mm
%   high on a 90 mm object, comfortably inside 180 mm, while the floor is most of a metre away.
%
%   THE STRONGEST MODE, NOT THE NEAREST. Measured on the retained live frame the annulus held two
%   sharp planes 12 mm apart - 25311 px at z 0.400-0.402 and 15851 px at 0.412-0.414, two distinct
%   pieces of furniture - and the mug stands on the first. Nearest-to-the-hint picked the second
%   and came out 12.2 mm high; strongest picked the first and came out 0.2 mm high. Pixel count is
%   the better discriminator because the supporting surface is by construction the one the object
%   is standing in the middle of, and so the one filling the annulus.
z = NaN;

inAnn = valid ...
    & hypot(X - centre(1), Y - centre(2)) >= radius ...
    & hypot(X - centre(1), Y - centre(2)) <  2 * radius ...
    & abs(Z - zHint) <= 2 * height;
if nnz(inAnn) < double(cfg.refine.minRegionPixels)
    return
end

% BINNED AT cfg.refine.plateauTol, WHICH IS THIS FUNCTION'S OWN DEFINITION OF "ONE FLAT
% SURFACE" - it is what separates the plateau from the rest of the band - so the two places
% that ask "is this all one level face?" answer at the same thickness. cfg.percep.
% planeMaxDistance was the first choice and is wrong for this job by a measured margin: at
% 20 mm it swallows the two 12 mm-apart planes of the frame above into one bin and returns
% their blend, 4.6 mm high, where the 4 mm tolerance resolves them and returns 0.2 mm high.
zAnn  = Z(inAnn);
bin   = max(double(cfg.refine.plateauTol), eps);
edges = (floor(min(zAnn) / bin) : ceil(max(zAnn) / bin)) * bin;
if numel(edges) < 2
    return
end
counts = histcounts(zAnn, edges);
[peak, k] = max(counts);
if peak < double(cfg.refine.minRegionPixels)
    return
end

% The mode locates the plane to one bin; its own pixels then locate it to the render's precision.
z = mean(zAnn(zAnn >= edges(k) & zAnn < edges(k + 1)));

end
