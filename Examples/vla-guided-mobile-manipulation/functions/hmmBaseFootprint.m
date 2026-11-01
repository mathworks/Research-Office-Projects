function [geom, dims, ccPhys, ccDrive] = hmmBaseFootprint(cfg, carriedExtent)
%HMMBASEFOOTPRINT The base's footprint rectangle and the circle chain that covers it.
%
%   [geom, dims, ccPhys, ccDrive] = HMMBASEFOOTPRINT(cfg, carriedExtent)
%
%   geom     the numbers. Field list at the end of this header.
%   dims     the vehicleDimensions the checkers are built from.
%   ccPhys   inflationCollisionChecker covering the PHYSICAL footprint, no margin.
%   ccDrive  the same chain inflated to physical + cfg.safety.plannerMargin.
%
%   THIS IS THE GEOMETRY WITHOUT THE MAP, and it is separate for one caller: hmmSetup,
%   which validates the stand-off radii against what the planner will accept and has no
%   business building two 240 x 240 costmaps to find out a number that depends only on
%   cfg. hmmBaseValidators calls this and then builds the maps. Nobody computes any of it
%   twice.
%
%   WHY A RECTANGLE AND NOT A RADIUS is hmmBaseValidators' header, and it is the argument
%   this whole family exists for. The one-paragraph version: a single inflation radius has
%   to be the CIRCUMSCRIBED circle of the chassis, because this base is holonomic and the
%   planner may not assume a heading; for a 0.531 x 0.360 m robot with the configured
%   margin that disc was 0.468 m and needed 0.94 m of opening; the apartment's internal
%   doorways are 0.80 and 0.75 m; so two rooms were unreachable by a robot that clears
%   both jambs by 0.195 m.
%
%   THE CHAIN IS TIGHT ACROSS AND LOOSE ALONG, WHICH IS THE TRADE AND IT IS THE RIGHT WAY
%   ROUND. N circles of radius hypot(L/(2N), W/2) laid along the length cover the
%   rectangle exactly, but the two end circles bulge past the end faces. So:
%
%     N   lateral half-extent   reach past the origin, fore and aft
%     1        0.3207 m                  0.3207 m
%     2        0.2236                     0.3564
%     3        0.2005                     0.3775
%     5        0.1877                     0.4001
%     7        0.1839                     0.4115
%
%   against a true half-width of 0.1799 m and a true half-length of 0.2655 m. More circles
%   fit the width better and the ends worse. cfg.plan.baseFootprintCircles = 5 because the
%   lateral figure is what a DOORWAY measures and the ceiling for reaching all four rooms
%   was measured at 0.300 m: 0.1877 + 0.100 = 0.2877 m passes with 12 mm to spare, while
%   three circles would sit on the line at 0.3005 m. Carrying the shipped mug spends 0.8 mm
%   of that 12, reaching 0.2884 m - see the carried field below.
%
%   WHAT THAT COSTS IS 0.135 m OF PHANTOM NOSE AND TAIL, and it is charged to the
%   stand-off rather than to the doorways. hmmSetup check 7 uses geom.forward, not
%   geom.inflation, for exactly this reason - a stand-off is measured along the approach
%   bearing with the base facing the target, so what has to clear the surface the target
%   stands on is the front of the chain and not its side. Both configured stand-offs are
%   comfortably outside it; a tighter one would have to raise the circle count, which
%   makes the nose worse, or accept the disc, which closes the doorways.
%
%   geom fields
%     length, width      m, the footprint rectangle, carried object included
%     halfLength         m, length/2 - the true front and rear faces
%     halfWidth          m, width/2 - the number a doorway actually measures
%     numCircles         how many circles cover it
%     physical           m, the covering radius, which the physical checker uses
%     margin             m, cfg.safety.plannerMargin
%     inflation          m, physical + margin, which the driving checker uses. Also the
%                        LATERAL half-extent of the driving region, because the chain's
%                        centres all lie on the base's own x axis
%     forward            m, how far the driving region reaches ahead of the base origin:
%                        the frontmost circle centre plus inflation. The number a
%                        stand-off has to beat
%     forwardPhysical    m, the same with no margin
%     inscribed          m, halfWidth + margin. The largest disc a footprint-valid pose is
%                        guaranteed to have free, so a point-robot search on a map inflated
%                        by it can only be MORE permissive than the driving validator -
%                        which is what makes it sound as a reachability pre-screen
%     carried            m, the held object's own half-extent. NOT what it added to the
%                        rectangle: it is offset by cfg.plan.carryOffset and only widens an
%                        axis when the sum of the two beats the chassis and the stowed arm.
%                        For the shipped mug it now just does, by 5.0 mm of length and 0.6 mm
%                        of width - it did NOT until cfg.plan.graspDepth was sized to the
%                        tallest graspable prop rather than to the mug, so any reading of
%                        this taken before that change understates it. Compare geom.length
%                        and geom.width across two calls to learn what an object actually cost
%
%   See also hmmBaseValidators, hmmBaseInflation, hmmSetup.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg           struct
    carriedExtent (:,1) double = zeros(3, 1)
end

%% 1  The rectangle
% max, not a sum: the arm is ON the chassis, not bolted to its front face, so whichever of
% the three reaches further in an axis is what the robot presents in that axis.
%
% AND THE CARRIED OBJECT IS A THIRD CANDIDATE RATHER THAN A BLANKET ADDITION, WHICH IS THE
% CORRECTION THIS LINE CARRIES. It used to read max(chassis, stowedArm) + carried, so
% holding anything grew the robot by the object's half-extent on all FOUR sides. That is
% the rectangle's version of what hmmBaseInflation does to a radius, and for a radius it is
% right: a radius has no directions, so the only sound thing to do with a carried object is
% add it. A rectangle does have directions, and the object is at a KNOWN PLACE in it -
% clamped in the jaws, and the jaws are at the stow pose whenever the base is driving. So
% the object contributes |carryOffset| + carried in each axis, and contributes nothing at
% all when that is inside what the chassis and the stowed arm already present.
%
% WHAT THE BLANKET ADDITION COST, because "conservative" was not free. The shipped mug -
% ObjectTarget.Extent [0.101 0.070 0.090], carried 0.0505 m - took the physical covering
% radius from 0.1876 to 0.2389 m and the inflated one from 0.2876 to 0.3389 m, while
% measurement puts the object's own centre inside |x| <= 0.193, |y| <= 0.101 of the base
% origin, against a rectangle already 0.2655 x 0.1799 m. It was entirely inside the robot,
% and modelling it as sticking 50 mm out of every face broke two things:
%
%   - THE PLACE LEG COULD NOT BE PLANNED. At cfg.sup.baseStandoff the base may legally
%     stop 0.113 m short of its goal, and the held footprint needed about 65 mm more room
%     than the pick stand-off has. So the instant the gripper closed, the base was standing
%     somewhere the map said it may not stand: hmmPlanBasePath's start check failed,
%     escapeMargin refuses to offer an escape from a pose it deems in contact, and the run
%     died with BaseStartOccupied without the base moving. That window measures NEGATIVE
%     under the old rule.
%   - THE DOORWAYS CLOSED. 0.3389 m of inflation is well past the 0.300 m at which the
%     Bedroom stops being reachable, so under the old rule the robot could reach a room and
%     then not carry anything out of it. The whole reason hmmBaseValidators models a
%     rectangle instead of a disc was to get under that ceiling, and the carried term put it
%     back over. (The ceiling is 0.300 m and not the 0.320 m once quoted here: labelling the
%     connected components of the shipped nav map's free space keeps both carved doorways in
%     the start pose's component at 0.300 and loses the Bedroom at 0.310.)
%
% THIS IS NOT A RELAXATION OF SAFETY, AND THE PRECEDENT IS ONE LINE ABOVE. cfg.plan.
% stowedArmHalfExtent is already a stow-pose-dependent footprint term: the arm is modelled
% where it actually is when the base drives, not as a shell around every pose it can reach.
% The carried object was the one term that ignored where the thing it models sits. It is
% still bounded from ABOVE - carryOffset is the assembled tree's own figure plus
% cfg.plan.graspDepth, so it holds whatever way the tool is turned - and it still grows the
% robot for an object big enough to stick out: a 0.30 m cube takes the rectangle to
% 0.345 x 0.255 m, and a 0.60 x 0.40 m one to 0.495 x 0.405 m.
%
% hmmBaseInflation is still called for the carried term rather than restating the
% half-extent rule, so that rule keeps exactly one home.
[~, ~, ~, carried] = hmmBaseInflation(cfg, carriedExtent);

halfL = max([cfg.base.chassisSize(1) / 2, cfg.plan.stowedArmHalfExtent(1), ...
             cfg.plan.carryOffset(1) + carried]);
halfW = max([cfg.base.chassisSize(2) / 2, cfg.plan.stowedArmHalfExtent(2), ...
             cfg.plan.carryOffset(2) + carried]);

geom = struct( ...
    'length',          2 * halfL, ...
    'width',           2 * halfW, ...
    'halfLength',      halfL, ...
    'halfWidth',       halfW, ...
    'numCircles',      double(cfg.plan.baseFootprintCircles), ...
    'physical',        0, ...
    'margin',          cfg.safety.plannerMargin, ...
    'inflation',       0, ...
    'forward',         0, ...
    'forwardPhysical', 0, ...
    'inscribed',       halfW + cfg.safety.plannerMargin, ...
    'carried',         carried);

%% 2  The circle chain
% WHY vehicleDimensions IS SET UP THIS WAY. Its origin is the centre of the REAR AXLE,
% which is where a car's turn centre is and is not where this robot's origin is. Setting
% RearOverhang to half the length puts the rear face at -L/2, and the front face then falls
% at Wheelbase + FrontOverhang = L/2, so the rectangle is centred on the base origin -
% which is what every pose in this project is measured from. The wheelbase is a token 10 mm
% because zero is not a legal wheelbase and nothing here reads it: no steering model is
% involved, only the rectangle the collision checker covers.
dims = vehicleDimensions(geom.length, geom.width, cfg.base.chassisSize(3), ...
    RearOverhang  = halfL, ...
    FrontOverhang = halfL - 0.010, ...
    Wheelbase     = 0.010);

% The covering radius is READ BACK from inflationCollisionChecker rather than computed
% here. It is hypot(length/(2*numCircles), width/2) and writing that out would be one more
% place for the toolbox's convention and this project's to drift apart; asking the object
% what it chose cannot drift.
ccPhys = inflationCollisionChecker(dims, geom.numCircles);
geom.physical  = ccPhys.InflationRadius;
geom.inflation = geom.physical + geom.margin;

% The frontmost circle's centre, in the base's own frame. N circles evenly spaced along the
% length put centres at +/-halfL*(1 - 1/N) at the extremes, so the driving region reaches
% that far plus the inflation.
frontCentre           = halfL * (1 - 1 / geom.numCircles);
geom.forward          = frontCentre + geom.inflation;
geom.forwardPhysical  = frontCentre + geom.physical;

ccDrive = inflationCollisionChecker(dims, geom.numCircles);
ccDrive.InflationRadius = geom.inflation;

end
