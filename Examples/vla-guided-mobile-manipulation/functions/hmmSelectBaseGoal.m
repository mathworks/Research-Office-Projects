function [goalOut, bearingDelta, nTried, altGoal] = hmmSelectBaseGoal(gridMatrix, goalIn, target, carriedExtent)
%HMMSELECTBASEGOAL Choose a stand-off pose the base planner will actually accept.
%
%   [goalOut, bearingDelta, nTried, altGoal] = HMMSELECTBASEGOAL(gridMatrix, goalIn, target,
%       carriedExtent)
%
%   goalIn is the stand-off pose TaskPlanner derived by stepping cfg.sup.baseStandoff
%   back from target along the straight bearing from the base. goalOut is a pose at
%   the SAME distance from the same target, on a bearing the map says is free.
%   bearingDelta is how far it had to rotate, in radians, and is exactly zero when
%   goalIn was already valid. nTried counts candidates examined, so a run can report
%   whether the search did any work. altGoal is goalOut's partner - the same position at a
%   roughly opposite heading that passes the same footprint and reach screen - or goalOut itself
%   when none does (hmmStandoffPartner). It is an offer to hmmPlanBasePath, which plans both and
%   keeps the cheaper drive.
%
%   THE DEFECT THIS EXISTS TO FIX. The chart has no map knowledge, so its bearing is
%   not a choice - it is whatever line happens to connect the robot to the object -
%   and a blocked bearing was a dead target. hmmPlanBasePath would reject the pose
%   with BaseGoalOccupied before searching anything, the supervisor would retry, the
%   chart would derive the identical pose from a base that had not moved, and the
%   action would be reported FAILED with the base still at its start. With the mug at
%   [3.83 -3.01] and the base at [5.2 -4.0] that is precisely what happened: the
%   144.2 deg bearing puts the goal inside the inflated side table, while 7 of 24
%   bearings around the same mug at the same 0.75 m are free.
%
%   AND IT WAS INVISIBLE UNTIL THE MAP BECAME REAL. Under cfg.map.mode = 'live' the
%   grid the base planned over held 27 occupied cells of 57600, so every bearing was
%   free and a rule that considers one bearing was indistinguishable from a rule that
%   considers the right one. The static map (cfg.map.mode = 'fixed', 33111 occupied
%   cells) is what exposed it.
%
%   THREE FREE VARIABLES NOW: THE BEARING, THE ARRIVAL HEADING AND THE RADIUS. What the
%   arm has to reach is a function of the stand-off DISTANCE, the target height and the
%   base's HEADING - never of the bearing the stand-off sits on, because joint 1 absorbs a
%   pure azimuth exactly (hmmArmLeverBand documents the measurement). So rotating the
%   stand-off round the target cannot make a reachable target unreachable, which is why the
%   bearing was the first variable searched and is still searched first.
%
%   THE RADIUS USED TO BE HELD FIXED AND THAT WAS THE REMAINING DEFECT. The old header
%   promised the distance is never changed, on the grounds that rotating is free while
%   scaling is not, so the radius should stay with the chart as the tuned quantity. The
%   premise is true; the conclusion was wrong, because the chart's radius is not tuned per
%   target - it cannot be, having neither the map nor an arm model. It is the constant
%   cfg.sup.baseStandoff for every pick and cfg.sup.placeStandoff for every place, chosen
%   for the worst case and applied to all of them. At a 0.90 m support the arm's ceiling is
%   0.459 m empty-handed against a commanded 0.750 m, so the ring was being rotated at a
%   radius every candidate had already been disqualified by. Eight props in the shipped
%   scene fail on that alone. The radius is now searched too, inside hmmArmLeverBand's own
%   window and only after the chart's value has been tried and refused - see the body for
%   the ordering, the cost argument and the compatibility guarantee.
%
%   WHAT IS NOT KEYED TO ANYTHING. The window comes from the lever band, which is already
%   indexed by the target height and by what the gripper holds, so one code path serves any
%   object on any surface for either verb. No prop name, no lookup table and no hand-tuned
%   distance survives on this path.
%
%   A FAILED SEARCH RETURNS goalIn UNCHANGED, ON PURPOSE. When no bearing is free the
%   target genuinely cannot be serviced from that radius, and the right outcome is the
%   planner's existing BaseGoalOccupied - a named, distinct failure - not a silent
%   substitution of some other invalid pose. This function can improve the goal or
%   leave it alone; it can never manufacture success.
%
%   IT JUDGES BY THE PLANNER'S OWN FOOTPRINT. The validator is hmmBaseValidators' - the
%   same object hmmPlanBasePath plans against, built from the same call - so a pose this
%   function certifies is a pose that function accepts. Any drift between the two would
%   produce a goal certified here and refused there, which is a worse failure than the one
%   being fixed because it has two places to hide.
%
%   AND THE HEADING NOW MATTERS, WHICH SUITS THIS SEARCH RATHER THAN COSTING IT. The
%   validator covers the base's footprint rectangle and rotates it with the state, so a
%   pose is judged in the orientation the robot will actually hold. hmmStandoffRing's
%   candidates face the target, which is the yaw the chart commands, so what is certified
%   here is what is driven. Under the old disc a stand-off was accepted or refused by its
%   position alone, and a bearing that only fits sideways-on was indistinguishable from
%   one that fits any way round.
%
%   WHAT IT DOES IS BUILD THE MAP; hmmStandoffRing DOES THE CHOOSING. This function's
%   own content is the validator and the memo. The candidate order, the accept test and
%   what it does and does not promise about reachability are all documented there, and
%   are shared with the offline reachability census so it cannot certify targets by a
%   different rule than the model applies.
%
%   COST. Inflating a 240x240 map is the expensive part and this is called at the
%   supervisory rate, so both the map and the answer are memoised on the full input
%   tuple. Nothing is cached across a change of input - the key includes the grid
%   itself - so this is a memo and not hidden state, and 'live' mode remains correct.
%
%   See also hmmBaseValidators, hmmPlanBasePath, hmmStandoffRing.

%   Copyright 2026 The MathWorks, Inc.

arguments
    gridMatrix    double
    goalIn        (3,1) double
    target        (3,1) double
    carriedExtent (:,1) double
end

persistent memoKey memoGoal memoDelta memoTried memoAlt
persistent mapGrid mapCarried mapValidator
persistent noRadiusKey goodRadiusKey goodRadius

cfg = cachedConfig();

goalOut      = goalIn;
bearingDelta = 0;
nTried       = 0;
altGoal      = goalIn;

% ONE CELL IS THE FLOOR, AND IT IS ALSO THE IDLE TEST. Before the chart has entered a
% navigation state both goalIn and target are the bus's zeros, so the ring has no
% radius and there is nothing to choose. Below one map cell the candidates would all
% quantise into the same cell as the nominal anyway.
cellSize = 1 / cfg.map.resolution;
radius   = norm(goalIn(1:2) - target(1:2));
if radius < cellSize
    % A GOAL ON ITS OWN TARGET IS A MOVE, NOT A WORK POSE - HOME, and since 2026-10-12 NAVIGATE,
    % which used to stop a pick's 0.75 m stand-off short of the point it was sent to. No arm works
    % from here, so the only question is whether the BASE fits: if it does the goal is kept exactly,
    % and if the point is furniture it moves to the nearest pose that fits - see nearestFreePose.
    % The idle bus (all zeros) is not a target and stays the instant pass-through it always was.
    if any(target(1:2) ~= 0)
        goalOut = nearestFreePose(gridMatrix, goalIn, cfg, carriedExtent);
        altGoal = goalOut;
    end
    return
end

key = {gridMatrix, goalIn, target, carriedExtent};
if ~isempty(memoKey) && isequal(key, memoKey)
    goalOut      = memoGoal;
    bearingDelta = memoDelta;
    nTried       = memoTried;
    altGoal      = memoAlt;
    return
end

%% The validator, which is the planner's own
% Keyed on carriedExtent rather than on a radius, because the footprint is what the
% carried object grows and hmmBaseValidators owns that arithmetic. The grid is in the key
% too, so 'live' mode still rebuilds when the map changes.
if isempty(mapValidator) || ~isequal(mapCarried, carriedExtent) || ...
        ~isequal(mapGrid, gridMatrix)
    mapValidator = hmmBaseValidators(gridMatrix, cfg, carriedExtent);
    mapCarried   = carriedExtent;
    mapGrid      = gridMatrix;
end

%% The search itself, shared with the offline census so the two cannot disagree
% THE ARRIVAL HEADING IS PART OF THE SEARCH NOW, and the reach band is why this call passes a
% fifth argument. cfg.pedestal.offset puts the arm 60 mm AFT of the base centre, so facing the
% target - the yaw the chart derives and the only one this search used to consider - is very
% nearly the heading that holds the arm FURTHEST from its work. hmmStandoffRing turns the base
% as little as the arm's own reach requires; hmmArmLeverBand is what "requires" means, and it
% is keyed on the target's height because the reachable lever shrinks as the target rises.
%
% AND carriedExtent IS PASSED TO IT, WHICH IS NOT DECORATION. The lever ceiling is indexed by
% how high the TOOL ends up, and on the place leg that is set by the object in the gripper -
% 0.0691 m for the shipped mug against the 0.100 m fallback. Those 31 mm cost 31 mm of
% admissible stand-off and closed the place ring completely; see hmmArmLeverBand. This
% function already holds the extent, for the validator, so passing it costs nothing. On the
% pick leg it is zeros and the fallback applies, which is right there: nothing is held yet.
% THE SUPPORT, NOT THE OBJECT: see hmmSupportHeight. A place target is already a support point.
% Screening the pick at its lowest pre-grasp rung instead was tried (2026-10-12) and refused the
% Bottle and SoapDispenser outright, because it charges the full worst-case arrival tolerance; the
% Bottle's real arrival left its mount 0.585 m out, inside the 0.040 rung's 0.610 m ceiling.
[loLever, hiLever] = hmmArmLeverBand(cfg, hmmSupportHeight(target), carriedExtent);
reach = struct('mountOffset', cfg.pedestal.offset(1:2), ...
               'leverBand',   [loLever, hiLever], ...
               'arrivalTol',  cfg.plan.baseGoalTolerance, ...
               'nYaw',        double(cfg.sup.arrivalYawOffsets));

nB = double(cfg.sup.goalBearings);

% A TARGET ALREADY PROVED UNSERVICEABLE IS NOT RE-PROVED. See the long note below for why
% the key is sound and why it is quantised; the reason the test sits HERE, ahead of the
% chart's own radius, is that the same invariant covers that call too. The verdict for a
% fixed map, target, held object and radius does not depend on goalIn, so if this tuple
% failed once it fails now, and the 576-candidate opening call is as redundant as the
% ladder. Skipping both is what brings a failing target back inside the supervisory
% budget; skipping only the ladder left it at 28.5 ms against 20 ms.
%
% nTried COMES BACK 0 ON THIS PATH, WHICH IS THE HONEST FIGURE. Its contract is candidates
% examined so a run can report whether the search did any work, and on a cache hit it did
% none. A reader comparing nTried across ticks will see the first tick's full count and
% zeros after it, which is the behaviour to expect rather than a gap in the record.
ladderKey = {gridMatrix, target, carriedExtent, round(radius * 1000)};
if ~isempty(noRadiusKey) && isequal(noRadiusKey, ladderKey)
    memoKey   = key;
    memoGoal  = goalOut;        % still goalIn: set at the top and never touched
    memoDelta = bearingDelta;
    memoTried = nTried;
    memoAlt   = altGoal;        % likewise goalIn: an unserviceable target has no partner
    return
end

% AND A RADIUS THE LADDER ALREADY FOUND IS REUSED, WHICH IS THE OTHER HALF OF THE COST
% ARGUMENT. The negative cache above spares the targets that fail; this spares the ones
% that only succeed via the ladder, and without it they are the expensive case. Measured
% on the bedroom lamp: it is serviceable at a searched 0.682 m, but the main memo keys on
% goalIn and the chart moves goalIn every tick, so the whole ladder re-ran each time at
% 34.4 ms against the 20 ms budget - worse than the failing target it was built for.
%
% ONLY THE RADIUS IS CACHED AND NEVER THE POSE, and that distinction is the point. The
% radius is a property of the target, the map and the arm, so it is stable while any of
% them is; the bearing is chosen relative to where the robot is standing NOW, so reusing a
% pose would peg the goal to a stale dispatch position and undo hmmStandoffRing's whole
% reason for ordering candidates by angular distance to goalIn. So the known radius is
% re-rung through the ordinary three-output call, which early-returns on the first free
% bearing and costs what a normal success costs.
%
% A MISS HERE FALLS THROUGH RATHER THAN FAILING. If the remembered radius no longer yields
% a free bearing - or yields one that happens to be bit-identical to the probe, which this
% test cannot tell apart from failure - control simply continues to the chart's radius and
% then to the full ladder, so the worst case is the cost of not having cached anything.
bear = atan2(goalIn(2) - target(2), goalIn(1) - target(1));
if ~isempty(goodRadiusKey) && isequal(goodRadiusKey, ladderKey)
    probe = [target(1:2) + goodRadius * [cos(bear); sin(bear)]; goalIn(3)];
    [gC, dC, ntC] = hmmStandoffRing(probe, target, mapValidator, nB, reach);
    if ~isequal(gC, probe)
        altGoal   = partnerOf(gC, target, mapValidator, reach);
        memoKey   = key;
        memoGoal  = gC;
        memoDelta = dC;
        memoTried = ntC;
        memoAlt   = altGoal;
        goalOut      = gC;
        bearingDelta = dC;
        nTried       = ntC;
        return
    end
end

% THE CHART'S OWN RADIUS IS STILL TRIED FIRST, AT EXACTLY THE OLD COST. When it works
% this call is the whole function, the answer is bit-identical to every previous version,
% and the ladder below never runs. That is the compatibility guarantee: no target that
% is serviced today changes its goal, because the first thing asked is the thing that
% used to be the only thing asked.
[goalOut, bearingDelta, nTried] = hmmStandoffRing(goalIn, target, mapValidator, nB, reach);

if isequal(goalOut, goalIn)

    % THE RADIUS IS NOW SEARCHED TOO, AND THIS REVERSES WHAT THE HEADER ABOVE USED TO
    % PROMISE. The old invariant was that the distance is never changed - rotating round
    % the target is free, because joint 1 absorbs azimuth exactly, while scaling is not,
    % so the radius was left to the chart as the tuned quantity. The reasoning is still
    % correct and the conclusion was still wrong, for a reason the invariant could not
    % see: the chart has no map and no arm model, so its radius is a CONSTANT -
    % cfg.sup.baseStandoff = 0.750 m for every pick and cfg.sup.placeStandoff = 0.480 m
    % for every place - and rotating a ring whose radius the arm cannot work at searches
    % 576 poses that were all refused before the search began.
    %
    % MEASURED, which is what turned this from an opinion into a change. At a 0.90 m
    % support the admissible stand-off ceiling is 0.459 m empty-handed and 0.528 m
    % holding the shipped mug, against a commanded 0.750 m. Eight props in the shipped
    % scene - the two glasses, the kettle, the knife, the plate, the cutting board, the
    % soap dispenser, the jewellery box - sit at supports between 0.80 and 0.92 m, and
    % every one of them is refused on the radius alone. Yet the 40 s reference run
    % PLACES at exactly 0.90 m, because a place point is chosen by biasToReachable to
    % sit where the base can stand while a prop sits where the manifest puts it. Same
    % surface, same height, opposite outcome - so height was never the discriminator and
    % the fixed radius was.
    %
    % NOTHING HERE IS KEYED TO AN OBJECT. The window is [loLever hiLever] from
    % hmmArmLeverBand, converted to a base-centre distance by the mount offset, and the
    % band is already indexed by the target height and by what is in the gripper. A mug,
    % a knife, or something that is not in this scene at all gets the same code and the
    % same four inputs. There is no table, no name test and no tuned constant left on
    % this path: cfg.sup.baseStandoff survives only as the chart's opening bid.
    %
    % NEAREST RUNG FIRST, WHICH GIVES BOTH BEHAVIOURS WANTED WITHOUT A SPECIAL CASE. If
    % the chart's radius lies inside the window it IS the first rung, so the retry below
    % re-asks the same question and the only thing gained is the four-output form's
    % unambiguous verdict. If it lies outside - which is the counter-height case, where
    % the chart bids 0.750 m against a 0.459 m ceiling - the nearest rung is the top of
    % the window, and the ladder descends from there. The base therefore never parks
    % closer to the furniture than the arm actually requires, which is the property a
    % fixed radius was chosen by hand to approximate.
    %
    % COST IS WHY THIS IS INSIDE THE FAILURE BRANCH. A ring call is
    % cfg.sup.goalBearings * cfg.sup.arrivalYawOffsets = 576 candidates and the window
    % is 8 to 15 rungs, so an unconditional ladder would be up to 8640 footprint-and-
    % reach tests per supervisory tick. Here it runs only for a goal that is already
    % failing, where the alternative is not a cheaper answer but BaseGoalOccupied, and it
    % is capped at maxRungs so the worst case is bounded rather than merely unlikely.
    %
    % AND IT STILL CANNOT MANUFACTURE SUCCESS. Every rung is judged by the same validator
    % and the same reach band as before; an empty band (hiLever < loLever, as at the
    % 1.32 m picture frame) skips the ladder entirely, and exhausting it leaves goalIn
    % untouched for the planner's own named failure. The promise given up is only that
    % the radius returned is the radius asked for - callers that assumed it should read
    % the radius back off goalOut, which is where it has always been legible.
    % A NEGATIVE CACHE, BECAUSE WITHOUT ONE THIS BRANCH BLOWS THE SUPERVISORY BUDGET.
    % Measured on the shipped knife: a ladder that exhausts costs 111 ms median against
    % a cfg.rates.supervisor period of 20 ms, so a target with no admissible radius would
    % be re-proving it 5.6x over budget on every tick for as long as the chart kept
    % asking. The main memo above cannot absorb that - it keys on goalIn, which the chart
    % recomputes from the live base pose, so a driving robot misses it every tick.
    %
    % WHAT MAKES THE WEAKER KEY SOUND is hmmStandoffRing's own invariant: its candidate
    % bearings are fixed to the world, not phase-locked to goalIn, so goalIn decides
    % which free bearing is CHOSEN and never which bearings exist. Whether any rung has a
    % free bearing at all is therefore a function of the map, the target, what is held and
    % the rung set - and not of where the robot happens to be standing. radius is in the
    % key because it orders the rungs and truncates them at maxRungs; it is constant per
    % verb, being whichever cfg.sup constant the chart used, so it does not weaken the
    % cache in practice. Nothing is remembered across a change of any of those, so this
    % stays a memo rather than becoming state, and a 'live' map still invalidates it.
    %
    % THE RADIUS IS QUANTISED TO A MILLIMETRE IN THE KEY, AND IT HAS TO BE. It is derived
    % by norm() from a pose the chart rebuilds every tick, so the last bits move as the
    % base's bearing moves even though the chart is holding one constant. An isequal on
    % the raw double therefore misses on every tick and the cache does nothing at all:
    % measured as 104 ms median against the 20 ms budget, indistinguishable from having
    % no cache, which is how this was caught. A millimetre is far finer than the 50 mm
    % rung spacing it feeds, so the quantisation cannot change which rungs are searched.
    %
    % ONLY FAILURE IS CACHED, which is the safe direction. A success is returned through
    % the main memo and is re-derived whenever goalIn moves, so the base still gets the
    % bearing nearest its actual pose. A cached failure only ever suppresses a search that
    % was about to fail, and the outcome - goalIn untouched, planner reports its own named
    % error - is identical either way.
    % THE RUNGS ARE hmmStandoffRadii's, the same list perception scores a place point on
    % (hmmPlaceBearings), so the two cannot disagree about which places are serviceable.
    % The cap is cfg.sup.standoffMaxRungs; it was a local 6, which stopped at ~0.62 m and
    % left a couch seat served only from 0.57-0.67 m out of reach (2026-10-15).
    rungs = hmmStandoffRadii(cfg, loLever, hiLever, radius);
    if ~isempty(rungs)

        for r = rungs
            probe = [target(1:2) + r * [cos(bear); sin(bear)]; goalIn(3)];

            % THE FOUR-OUTPUT FORM IS NEEDED HERE AND ONLY HERE. The three-output form
            % signals failure by returning its input bit-identical, which is
            % indistinguishable from a goalIn that was already valid on a grid bearing -
            % an ambiguity that does not matter when there is one radius and does when
            % the answer decides whether to try another. nFree counts free bearings and
            % is unambiguous. It costs the ring's early return, which is affordable
            % exactly because this branch is the one that was going to fail anyway.
            [g, d, nt, nFree] = hmmStandoffRing(probe, target, mapValidator, nB, reach);
            nTried = nTried + nt;
            if nFree >= 1
                goalOut       = g;
                bearingDelta  = d;
                goodRadiusKey = ladderKey;
                goodRadius    = r;
                break
            end
        end
    end

    if isequal(goalOut, goalIn)
        noRadiusKey = ladderKey;   % no radius in the arm's window is free; stop retrying
    end
end

% THE PARTNER, COMPUTED ONCE PER NEW KEY AND MEMOISED WITH THE GOAL. It is the same position at
% a roughly opposite heading that passes the same two screens - see hmmStandoffPartner - and it is
% only an OFFER: hmmPlanBasePath plans both and keeps the cheaper drive, so a partner can never
% cost a dispatch. At most cfg.sup.arrivalYawOffsets footprint-and-reach tests, paid only when
% the memo misses, which is the same event that pays for the ring search above.
altGoal = partnerOf(goalOut, target, mapValidator, reach);

memoKey   = key;
memoGoal  = goalOut;
memoDelta = bearingDelta;
memoTried = nTried;
memoAlt   = altGoal;

end

%% ------------------------------------------------------------------------
function alt = partnerOf(goal, target, validator, reach)
%PARTNEROF hmmStandoffPartner's pose, or the goal itself when no heading qualifies.
[alt, found] = hmmStandoffPartner(goal, target, validator, reach);
if ~found
    alt = goal;
end
end

%% ------------------------------------------------------------------------
function goal = nearestFreePose(gridMatrix, goalIn, cfg, carriedExtent)
%NEARESTFREEPOSE goalIn if the base fits there, else the nearest pose that does, else goalIn.
%   Rings of one map cell outward to cfg.plan.navigateFreeMax, cfg.sup.goalBearings bearings each,
%   the commanded heading first and then the rest of cfg.plan.navigateHeadings. The validator is
%   hmmBaseValidators' driving one - the planner's own - so a pose accepted here is one
%   hmmPlanBasePath accepts. Returning goalIn when nothing fits keeps BaseGoalOccupied honest.
%   Memoised on the inputs: the block calls this at the supervisory rate.
persistent key held
k = {gridMatrix, goalIn, carriedExtent};
if ~isempty(key) && isequal(k, key)
    goal = held;
    return
end
sv   = hmmBaseValidators(gridMatrix, cfg, carriedExtent);
goal = goalIn;
yaws = wrapToPi(goalIn(3) + (0:cfg.plan.navigateHeadings - 1) * 2 * pi / cfg.plan.navigateHeadings);
if ~isStateValid(sv, goalIn')
    found = false;
    cellSize = 1 / cfg.map.resolution;
    nB = double(cfg.sup.goalBearings);
    for r = cellSize:cellSize:cfg.plan.navigateFreeMax
        for b = (0:nB - 1) * 2 * pi / nB
            p = goalIn(1:2) + r * [cos(b); sin(b)];
            for y = yaws
                if isStateValid(sv, [p', y])
                    goal  = [p; y];
                    found = true;
                    break
                end
            end
            if found, break, end
        end
        if found, break, end
    end
end
key  = k;
held = goal;
end

%% ------------------------------------------------------------------------
function cfg = cachedConfig()
%CACHEDCONFIG hmmConfig once per session, as hmmPlanBasePath does it.
cfg = hmmRuntimeConfig();
end
