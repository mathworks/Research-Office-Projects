function [sv, svPhys, ss, geom] = hmmBaseValidators(gridMatrix, cfg, carriedExtent)
%HMMBASEVALIDATORS The two state validators every base decision in this project uses.
%
%   [sv, svPhys, ss, geom] = HMMBASEVALIDATORS(gridMatrix, cfg, carriedExtent)
%
%   sv        the DRIVING validator: the base's footprint plus cfg.safety.plannerMargin.
%             What the search plans against and what a goal must satisfy.
%   svPhys    the PHYSICAL validator: the footprint alone, no margin. Nothing on any
%             path, anywhere, may violate this.
%   ss        the stateSpaceSE2 both are built on, so every state either validator is
%             asked about is a state in the same space.
%   geom      the numbers, for reporting and for callers that need the arithmetic
%             without the objects. See the field list at the end of this header.
%
%   WHY A RECTANGLE AND NOT A RADIUS, WHICH IS THE WHOLE POINT OF THIS FUNCTION.
%   A single inflation radius has to be the CIRCUMSCRIBED circle of the chassis, because
%   this base is holonomic - it can rotate in place anywhere, so the planner may not
%   assume any heading. For a 0.531 x 0.360 m robot that circle is 0.318 m, and with the
%   configured margin the disc the planner inflated by was 0.468 m: it needed 0.94 m of
%   opening to pass through anything. The apartment's two internal doorways are 0.80 and
%   0.75 m wide. So the Bedroom and the Bathroom were unreachable - not because the robot
%   does not fit, it has 0.195 m to spare at each jamb, but because a disc that big does
%   not. The ceiling was measured exactly: with the doorways carved free, all four rooms
%   are reachable at an inflation of 0.180-0.300 m and the Bedroom stops being reachable
%   at 0.320 m. Every honest disc for this robot is above that ceiling.
%
%   The fix is to give the validator the robot's SHAPE and its HEADING.
%   validatorVehicleCostmap covers the footprint rectangle with a chain of circles and,
%   with ThetaIndex pointing at the heading element, rotates that chain with the state.
%   The same point in a doorway is then passable pointing through it and not passable
%   turned across it, which is true of the robot and was not expressible before. The
%   binding number becomes the footprint's HALF-WIDTH plus the margin - 0.280 m - which
%   is under the measured ceiling with 20 mm to spare.
%
%   THIS IS NOT A RELAXATION OF SAFETY, AND THE DISTINCTION MATTERS. The disc was not
%   conservative on purpose; it was the only shape a single radius can describe. Two
%   things it papered over are now measured instead of padded:
%
%     - The stowed arm. cfg.plan.stowedArmRadius = 0.050 m was a PAD chosen to keep
%       baseRadius + stowedArmRadius above measureStowedEnvelope's reading, and that
%       reading is an omnidirectional max(hypot(x, y)) over body frames, which records no
%       direction. Measured PER AXIS instead, the stowed arm reaches 0.303 m forward, PAST
%       the chassis' own 0.266 m half-length, and 0.180 m sideways, 4.4 mm outside its
%       0.176 m half-width. So the arm makes the robot 8.8 mm wider and 37 mm longer, and
%       cfg.plan.stowedArmHalfExtent states that instead of a radius standing in for it.
%       The two axes are worth different amounts and that is the point of keeping them
%       apart: the length is divided by the circle count before it becomes a covering
%       radius and the width is not, so the 37 mm costs under 2 mm of inflation while the
%       8.8 mm costs all of itself.
%     - The heading. A disc is the union of the rectangle over ALL headings. Planning
%       against it forbids manoeuvres the robot performs, which is not caution, it is a
%       wrong model.
%
%   cfg.safety.plannerMargin is untouched at 0.100 m and is still added on top. What
%   changed is the shape it is added to.
%
%   ONE BUILDER, BECAUSE EVERY CALLER HAS TO AGREE EXACTLY. hmmPlanBasePath plans with
%   these; hmmSelectBaseGoal has to certify stand-off poses the planner will then accept.
%   A check that models the robot even slightly differently than the run does is worse
%   than no check, because it certifies targets the run refuses and the failure then has
%   two places to hide. hmmBaseInflation exists for the same reason and says so; this is
%   that argument applied to a shape instead of a scalar, and it calls hmmBaseInflation
%   for the carried term so that rule still has exactly one home.
%
%   THE RECTANGLE, THE CHAIN AND EVERY NUMBER IN geom ARE hmmBaseFootprint'S. That split
%   exists because hmmSetup has to validate the stand-off radii against what the planner
%   will accept, and it should not have to build two 240 x 240 costmaps to learn a figure
%   that depends only on cfg. Read that function's header for the circle-count table, for
%   why vehicleDimensions is set up the way it is, and for the one thing this model is MORE
%   conservative about than the disc was: the chain's end circles bulge 0.135 m past the
%   front and rear faces, which is charged to the stand-off through geom.forward rather
%   than to the doorways through geom.inflation.
%
%   See also hmmBaseFootprint, hmmBaseInflation, hmmPlanBasePath, hmmSelectBaseGoal,
%   validatorVehicleCostmap.

%   Copyright 2026 The MathWorks, Inc.

arguments
    gridMatrix    double
    cfg           struct
    carriedExtent (:,1) double = zeros(3, 1)
end

%% 1-2  The rectangle and the circle chain that covers it
% hmmBaseFootprint's, because hmmSetup needs the same numbers and has no map to build them
% from. Its header carries the measurement, the circle-count table and the reason the chain
% is tight across the robot and loose along it.
[geom, ~, ccPhys, ccDrive] = hmmBaseFootprint(cfg, carriedExtent);

%% 3  The two costmaps and the two validators
% BOTH COSTMAPS ARE FULLY SPECIFIED AT CONSTRUCTION, including the collision checker.
% A vehicleCostmap inflates itself when it is built, using whatever checker it has, and
% a fresh one's default checker is a 4.7 x 1.8 m passenger car whose 1.6 m radius
% swallows a 12 m apartment whole. Assigning the real checker afterwards works but
% inflates the map twice and leaves a window in which the object is wrong; handing it
% over at construction does neither.
mapDrive = costmapFor(gridMatrix, cfg, ccDrive);
mapPhys  = costmapFor(gridMatrix, cfg, ccPhys);

% ONE STATE SPACE. The extents come from the map so both validators and every metric
% speak about the same world, exactly as the two occupancyMaps did before.
ss = stateSpaceSE2([mapDrive.MapExtent(1:2); mapDrive.MapExtent(3:4); -pi pi]);
ss.WeightTheta = cfg.plan.thetaWeight;

% ThetaIndex = 3 IS THE ENTIRE POINT. Its default is NaN, and with NaN the validator
% checks only whether the xy location is occupied in the inflated map - which is a disc
% again, and this function would have achieved nothing.
sv = validatorVehicleCostmap(ss, Map = mapDrive, ThetaIndex = 3);
sv.ValidationDistance = cfg.plan.baseValidationDist;

svPhys = validatorVehicleCostmap(ss, Map = mapPhys, ThetaIndex = 3);
svPhys.ValidationDistance = cfg.plan.baseValidationDist;

end

%% ------------------------------------------------------------------------
function map = costmapFor(gridMatrix, cfg, cc)
%COSTMAPFOR The observed grid as a vehicleCostmap under one collision checker.
%   The grid goes in unchanged, which was checked rather than assumed: vehicleCostmap
%   and occupancyMap agree cell for cell and at every one of nine known world points,
%   so the two share a row order and an origin. That was worth checking - a row
%   convention got the kitchen drawn in the wrong half of the map once already.
map = vehicleCostmap(gridMatrix, ...
    CellSize          = 1 / cfg.map.resolution, ...
    MapLocation       = cfg.map.origin, ...
    FreeThreshold     = cfg.map.freeThreshold, ...
    OccupiedThreshold = cfg.map.occupiedThreshold, ...
    CollisionChecker  = cc);
end
