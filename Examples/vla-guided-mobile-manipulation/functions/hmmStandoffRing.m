function [goalOut, bearingDelta, nTried, nFree] = hmmStandoffRing(goalIn, target, validator, nBearings, reach)
%HMMSTANDOFFRING The nearest stand-off pose around a target that a validator accepts.
%
%   [goalOut, bearingDelta, nTried, nFree] = HMMSTANDOFFRING(goalIn, target, validator,
%       nBearings, reach)
%
%   goalIn is a stand-off pose already at the wanted distance from target. goalOut is
%   a pose at that SAME distance, on the free bearing nearest to goalIn's own.
%   bearingDelta is the rotation applied, and goalOut is returned bit-identical to
%   goalIn when nothing could be changed. nTried counts candidates examined.
%
%   reach IS OPTIONAL AND IT ADDS THE ARRIVAL HEADING AS A SECOND FREE VARIABLE. Omit it
%   and every candidate faces the target, which is what this function did for its first two
%   versions and is still exactly what it does - the same candidates in the same order, so a
%   caller that does not pass it gets bit-identical answers. Pass it and each bearing is also
%   tried at a range of headings, and a candidate must satisfy the ARM as well as the map.
%   Fields:
%
%       .mountOffset  1x2, the arm mount in the base frame, cfg.pedestal.offset(1:2)
%       .leverBand    1x2, [min max] horizontal mount-to-target distance the arm works at,
%                     from hmmArmLeverBand
%       .arrivalTol   1x3, cfg.plan.baseGoalTolerance - the pose licence, see below
%       .nYaw         how many headings to try per bearing, cfg.sup.arrivalYawOffsets
%
%   WHY THE HEADING BECAME A VARIABLE, AND WHY IT IS NOT A FIXED +/-90 deg. The arm is not
%   mounted at the base centre, so the distance the ARM has to reach is not the stand-off
%   distance the BASE was sent to: turning the base on the spot walks the mount round a
%   circle of radius norm(mountOffset) and changes the lever by twice that, 132 mm on this
%   robot. While the mount stood 60 mm FORWARD, facing the target was also the shortest
%   lever and there was nothing to choose. The mount is now 60 mm AFT, so facing the target
%   is the LONGEST lever bar a few degrees, and the place leg's pre-place pose falls outside
%   cfg.arm.leverCeiling nose-on while clearing it comfortably turned.
%
%   A FIXED SIDEWAYS ARRIVAL WOULD HAVE BEEN THE WRONG FIX, which is worth recording because
%   it was the first one proposed. +/-90 deg is neither necessary nor sufficient: it is more
%   than the pick needs (71 deg suffices at the worst licensed arrival, and nose-on suffices
%   at the commanded radius) and it is not the best the geometry offers either, since the
%   minimum-lever heading is 155.4 deg, where the mount points straight at the target. So the
%   offsets are tried SMALLEST FIRST and the first one that clears both tests wins. Nose-on
%   stays nose-on wherever nose-on works, and no leg turns further than its own reach
%   requires. Reach is provably indifferent to the bearing the target sits on - see
%   hmmArmLeverBand - so the only thing a heading costs is the turn itself.
%
%   THE REACH TEST IS TAKEN OVER THE WHOLE ARRIVAL LICENCE, NOT AT THE COMMANDED POSE, and
%   that is what makes it worth having. cfg.plan.baseGoalTolerance licenses a 160 mm box and
%   +/-0.20 rad of yaw, so the base may legally stop 113 mm out along the diagonal and turned
%   11 deg off - and reach fails OUTWARD, so the pose that matters is the furthest one
%   allowed rather than the one commanded. Measured on the shipped pick: nose-on at the
%   commanded 0.750 m needs a 0.810 m lever against a 0.860 m ceiling and passes, while the
%   same goal at the worst licensed arrival needs 0.924 m and does not. A test at the
%   commanded pose would certify that goal and the run would fail at it.
%
%   WHAT IT STILL CANNOT PROMISE. The band is a REACH bound, measured on the arm subtree
%   alone; whether the arm can get there without hitting the furniture is hmmScreenArmPlan's
%   question against the full tree, and unlike reach that one does depend on the bearing.
%   The footprint test is unchanged and still the planner's own, so a heading that reaches
%   but does not fit is refused by the validator exactly as before.
%
%   nFree IS OPT-IN AND CHANGES WHAT nTried MEANS, which is the only wrinkle in this
%   signature. Asking for it counts every free bearing on the ring, so the loop cannot
%   stop at the first success and nTried comes back at its maximum; ask for three
%   outputs or fewer and the early return is exactly as it was. It is gated on nargout
%   rather than always computed because the three-output form runs at the supervisory
%   rate inside the model and the four-output form runs once per reconstruction.
%
%   nFree COUNTS BEARINGS AND NOT CANDIDATES, WHICH MATTERS ONCE reach IS PASSED. A bearing
%   that is serviceable at four headings is one place the base can stand, not four, and
%   biasToReachable compares place points by this number against thresholds measured when a
%   bearing was the only unit there was - "7 of 24", "exactly one free bearing". So a bearing
%   contributes at most one, and the count stays on the same scale whether or not headings are
%   searched. nTried does not: it counts poses examined, which is the honest cost figure and
%   is what a run reports.
%
%   WHAT nFree IS FOR IS MARGIN, WHICH goalOut CANNOT EXPRESS. A ring with one free
%   bearing and a ring with six both yield a valid goalOut, and biasToReachable has to
%   choose between candidate place points rather than accept the first that works - the
%   shipped kitchen place point was serviceable with exactly one free bearing for as
%   long as it was serviceable at all, and 60 mm of footprint growth ended it. The count
%   lives here rather than in the caller because the BEARING GRID has to have one
%   definition: a caller that counted free bearings on a grid of its own would be
%   measuring a different ring than this function searches, which is the defect
%   described below in its second form.
%
%   THE RADIUS IS READ OFF goalIn, NEVER SUPPLIED. Reach depends on the stand-off DISTANCE,
%   the target height and the arrival HEADING, and on the bearing not at all, so rotating
%   round the target is free and scaling is not - which is why this function searches the two
%   variables it does and leaves the radius to its callers. Taking it from the caller's own
%   pose rather than from
%   cfg.sup.baseStandoff means this function cannot silently move a goal onto a
%   different ring than the one its caller measured.
%
%   THE CANDIDATE BEARINGS ARE FIXED TO THE WORLD, NOT TO goalIn, AND THAT IS THE WHOLE
%   POINT OF THIS FUNCTION'S SECOND VERSION. They used to be b0 + k*2*pi/nBearings,
%   where b0 is the bearing goalIn happens to sit on - so the SET of poses examined
%   rotated with wherever the robot was standing, and "is this target serviceable" was
%   not a question about the target at all. Now the set is k*2*pi/nBearings measured
%   from +x, ordered by angular distance to b0. goalIn still decides which free bearing
%   is chosen; it no longer decides which bearings exist.
%
%   THE DEFECT THAT FORCED IT, because the size of the effect is not obvious. The
%   shipped place point admits a single free arc 5.0 deg wide - 1.5 % of its ring - and
%   the ring's spacing is 15 deg. A phase-locked ring therefore found that arc for 35 %
%   of dispatch bearings and missed it for the other 65 %. An offline replay predicted
%   the pick stand-off at b0 = 60.232 deg and certified the place; the base actually
%   parked 0.185 m and 18.7 deg away, at b0 = 61.725 deg, and the identical call on the
%   identical map with the identical footprint returned no bearing at all. The run
%   failed BaseGoalOccupied on a target its own certifier had passed, and the 1.5 deg of
%   b0 between the two verdicts is smaller than the arrival tolerance the base is
%   allowed. That is why guessing the dispatch pose better is not a fix: the base
%   arrives within a tolerance, not at a point, so no certifier can know the phase in
%   advance. Removing the phase is the only way the two can agree.
%
%   WHAT WAS GIVEN UP FOR IT. A goalIn already valid on an off-grid bearing used to be
%   returned untouched with bearingDelta exactly 0; now it is rotated up to half a grid
%   step - 7.5 deg at 24 bearings - onto the nearest free grid bearing. That costs
%   nothing measurable, because reach has no bearing term (see hmmSelectBaseGoal), and
%   it buys the invariant that every caller examines the same poses. Callers that
%   treated bearingDelta == 0 as "the search did no work" should read nTried == 1
%   instead.
%
%   NEAREST-BEARING IS A BIAS TOWARD REACHABILITY, NOT A GUARANTEE OF IT. A free
%   bearing behind a wall is a valid goal that no path reaches, and the base planner
%   reports BaseSearchFailed for it. What keeps that rare is that goalIn's bearing
%   points at the side of the target the robot is already on, so the nearest free
%   bearing is usually on that side too. Nothing here tests connectivity - doing so
%   would need the start pose and a graph search, which is the planner's job.
%
%   WHY TWO CALLERS AND ONE IMPLEMENTATION. hmmSelectBaseGoal runs inside the model and
%   owns its inflated map. Every caller needs the identical candidate order and the
%   identical accept test, because a caller that certifies targets by a slightly
%   different rule than the model applies is one that passes its own check and fails
%   the run. Only the source of the validator differs, so only that is left to the
%   callers.
%
%   THE SECOND IS UPSTREAM OF THE MODEL, AND IT IS THE REASON nFree EXISTS.
%   hmmReconstructTargets/biasToReachable chooses WHERE ON A SURFACE to put a place
%   point, and the only honest criterion is whether the base will be able to stand at
%   it - so perception asks this function the same question the planner will ask later.
%   That it can ask at all is a consequence of the absolute bearing grid described
%   above: the base's pose at dispatch is unknown when the target is reconstructed, and
%   a phased ring would have made the answer depend on it. The count rather than the
%   verdict is what that caller needs, because it is comparing candidate points and a
%   ring with one free bearing is a place that works until the footprint changes.
%
%   AND ONE IMPLEMENTATION WAS NOT ENOUGH ON ITS OWN, WHICH IS WORTH RECORDING. Both
%   callers ran this exact code, on the same map, with the same footprint, and still
%   disagreed - because they passed different goalIn, and goalIn used to select the
%   candidate set. Sharing an implementation only guarantees agreement when the shared
%   answer is a function of the arguments the two callers actually agree on.
%
%   validator is any object isStateValid accepts as [x y yaw], which in practice is
%   hmmBaseValidators' driving validator. The yaw in each candidate is therefore load
%   bearing, not decoration: that validator covers the footprint RECTANGLE and turns it
%   with the state, so a bearing can be free facing the target and blocked turned across
%   it. Facing the target is the yaw the chart commands, so the candidate is judged in the
%   orientation it will be driven in.
%
%   See also hmmSelectBaseGoal, hmmBaseValidators, hmmReconstructTargets.

%   Copyright 2026 The MathWorks, Inc.

arguments
    goalIn    (3,1) double
    target    (3,1) double
    validator
    nBearings (1,1) double {mustBePositive}
    reach     struct = struct([])
end

goalOut      = goalIn;
bearingDelta = 0;
nTried       = 0;
nFree        = 0;
countAll     = nargout > 3;
searchYaw    = ~isempty(reach);

radius = norm(goalIn(1:2) - target(1:2));

% Bearing FROM THE TARGET to where the base stands - the quantity being rotated. A
% target directly under the base has no such bearing and no ring to search; the
% caller's own floor decides whether that case reaches here.
b0 = atan2(goalIn(2) - target(2), goalIn(1) - target(1));

% THE GRID IS ABSOLUTE - measured from +x about the target, so it depends on the target
% and nBearings and on nothing else. b0 orders the grid and does not define it; see the
% header for the failure that distinction cost.
nB    = round(nBearings);
bGrid = wrapToPi((0:nB - 1) * 2 * pi / nB);

% Nearest to the caller's own bearing first, which keeps the bias toward the side of the
% target the robot is already on. wrapToPi on the difference so that a b0 just below +pi
% is nearest to the grid bearings just above -pi and not furthest from them.
[~, ord] = sort(abs(wrapToPi(bGrid - b0)));
bGrid = bGrid(ord);

% THE HEADING GRID, ordered by how little it asks for. psi is measured from facing the
% target, so psi = 0 is the only candidate when reach was not supplied and the behaviour
% below collapses to exactly what it was. Ties in magnitude are broken by the resulting
% lever rather than by sign, because which of +psi and -psi brings the mount nearer the
% target is a fact about mountOffset and not a constant: with the mount to starboard the
% positive offset is the short one, and a hard-coded preference would silently invert if
% cfg.pedestal.offset ever moved to the other rail.
if searchYaw
    nY      = round(reach.nYaw);
    psiGrid = wrapToPi((0:nY - 1) * 2 * pi / nY);
    tb      = radius * [cos(psiGrid); -sin(psiGrid)];        % target in the base frame
    lv      = vecnorm(tb - reach.mountOffset(:), 2, 1);
    [~, ordY] = sortrows([abs(psiGrid(:)), lv(:)]);
    % ordY transposed to a ROW on purpose: psiGrid drives a for loop below, and a for over a
    % COLUMN vector iterates exactly once with the whole column as the loop variable.
    psiGrid = psiGrid(ordY(:)');
else
    psiGrid = 0;
end

for k = 1:nB
    b   = bGrid(k);
    pos = [target(1) + radius * cos(b); target(2) + radius * sin(b)];
    hit = false;
    % A BEARING BEHIND A WALL IS NOT A PLACE TO WORK FROM, however close it is - see
    % hmmWallBetween. Only when the arm has to work (reach given); a navigate goal has no arm.
    if searchYaw && hmmWallBetween(pos, target)
        continue
    end
    % AND A SPOT THE BASE FITS IN BUT CANNOT DRIVE TO IS NOT ONE EITHER: the pocket between a
    % bed and the wall, found once the stand-off radius ladder reached past the furniture
    % (2026-10-16, BaseGoalUnreachable). See hmmInMainFreeArea.
    if ~hmmInMainFreeArea(pos)
        continue
    end
    for psi = psiGrid
        % psi = 0 faces the target, which is the yaw the chart itself derives.
        cand   = [pos; wrapToPi(b + pi + psi)];
        nTried = nTried + 1;
        if ~isStateValid(validator, cand')
            continue
        end
        if searchYaw && ~hmmArmCanWork(cand, target, reach)
            continue
        end
        hit = true;
        % The FIRST success is the answer, whether or not the loop keeps going: bGrid is
        % ordered by angular distance to b0 and psiGrid by how far the base must turn, so
        % a later candidate is further from the caller's own pose in one of those two
        % senses and must not displace this one.
        if nFree == 0
            goalOut      = cand;
            bearingDelta = wrapToPi(b - b0);
        end
        break        % a bearing is one place to stand; the rest of its headings are not news
    end
    if hit
        nFree = nFree + 1;          % BEARINGS, not candidates - see the header
        if ~countAll
            return
        end
    end
end

end
