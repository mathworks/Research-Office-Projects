function bplan = hmmPlanBasePath(gridMatrix, startPose, goalPose, carriedExtent, ...
                                 requestId, simTime, planTemplate, altGoalPose)
%HMMPLANBASEPATH Search and time-parameterise one SE(2) base path.
%
%   bplan = HMMPLANBASEPATH(gridMatrix, startPose, goalPose, carriedExtent,
%       requestId, simTime, planTemplate)
%   bplan = HMMPLANBASEPATH(..., altGoalPose) also plans to altGoalPose - the stand-off's
%       partner heading from hmmSelectBaseGoal - and returns whichever drive is cheaper; see
%       choosePartner below. bplan.GoalPose always says which goal the reference ends at.
%
%   The triggered path-planning service's whole algorithm, in order:
%
%       1  a vehicleCostmap from the observed grid, under a collision checker that
%          covers the base's footprint RECTANGLE - chassis, stowed arm and carried
%          object - with a chain of circles
%       2  stateSpaceSE2 + validatorVehicleCostmap, whose ThetaIndex turns the chain
%          with the state so the model knows which way the robot is pointing
%       3  plannerBiRRT, falling back to plannerRRTStar
%       4  shortenpath, then pathmetrics for length, clearance and smoothness
%       5  contopptraj under the configured per-axis velocity and acceleration
%          boxes, giving the reference AND its derivative
%       6  re-validation of the parameterised samples
%
%   THE SHAPE CHANGED IN STEPS 1 AND 2, AND WHY IS hmmBaseValidators' HEADER. In one
%   line: a single inflation radius must be the chassis' circumscribed circle for a
%   base that rotates in place, that disc needed 0.94 m of opening, the apartment's
%   internal doorways are 0.80 and 0.75 m, and so two rooms were unreachable by a
%   robot that clears both jambs by 0.195 m. Everything from step 3 down is unchanged.
%
%   AND A SCREEN RUNS BEFORE THE SEARCH, which is new and is the answer to a
%   different complaint: that the simulation appeared to HANG when a destination
%   could not be reached. It did not hang. An unreachable goal ran BiRRT to its full
%   iteration budget, then RRT* to its own, then the supervisor retried and both ran
%   again - all inside time steps during which the model clock does not move, so
%   minutes of wall time bought no simulated time and nothing on screen changed. A
%   grid search settles connectivity in milliseconds, so it is asked first, and a
%   goal with no connected route is refused as BaseGoalUnreachable instead of being
%   sampled at.
%
%   THE START POSE IS NOT AN OBSTACLE TEST, AND TREATING IT AS ONE STRANDED THE
%   ROBOT. The inflation above is two different things added together: the space the
%   robot PHYSICALLY occupies (its footprint, plus whatever it is carrying), and a
%   driving MARGIN on top of that (cfg.safety.plannerMargin). A manipulation
%   stand-off deliberately gives up the
%   margin - the whole point of standing 0.75 m from a mug on a side table is to be
%   close enough to reach it, and the table is an obstacle the ceiling cameras map.
%   So the pose the base ends every navigation leg in is, by construction, inside
%   the inflated region, and a guard that refuses to plan from an invalid start
%   means a base parked at a stand-off can never drive anywhere again. That is what
%   the 20 s integration run did: leg one planned and tracked to within 21 mm, and
%   the next request died in 20 ms with the start pose as the only complaint.
%
%   SO THE START MAY BE INSIDE THE MARGIN, AND MAY NOT BE INSIDE AN OBSTACLE. The
%   two are separated by carrying TWO validators over the same grid: one covering the
%   footprint plus the margin, which every metre of the path after the first must
%   honour, and one covering the PHYSICAL footprint alone, which nothing anywhere may
%   violate. When the start fails the first and passes the second, a short straight
%   escape is planned out of the margin, the search proper runs from there, and the
%   true start is prepended so the reference still begins where the robot is. When
%   the start fails the second, the base is in contact with something and the plan
%   is refused, which is the one case the old guard was right about.
%
%   THE ESCAPE CANNOT BE LONG, AND THAT IS A THEOREM RATHER THAN A TUNING CHOICE.
%   A base that is not physically in collision has its whole footprint clear, and the
%   driving region is that same footprint grown by the margin, so the base's
%   penetration into the driving region is at most the margin
%   itself - cfg.safety.plannerMargin alone, the footprint and the carried extent both
%   cancelling because they are in both terms. Nothing further out is ever searched, so
%   the escape cannot quietly become a licence to drive through the inflation for half
%   a room.
%
%   WHY THIS SEARCH AND NOT A CAR ONE. The mecanum base is HOLONOMIC: it can
%   translate sideways and rotate in place. plannerHybridAStar,
%   stateSpaceReedsShepp, stateSpaceDubins and mobileRobotPropagator all encode
%   a minimum turning radius, so every one of them would refuse manoeuvres this
%   hardware performs and would produce three-point turns where a sideways slide
%   is available. cfg.plan.thetaWeight is small for the same reason - rotating
%   costs this base almost nothing.
%
%   GROW THE OBSTACLES, DO NOT SHRINK THE ROBOT. The collision checker covers the
%   footprint with a chain of circles and grows the occupied cells by their radius, so
%   the search still reasons about a point - which is what makes an SE(2) occupancy
%   check cheap enough to run thousands of times - while the SHAPE it is a point of
%   turns with the heading. The carried object is included because a robot holding a
%   wide object is a wider robot.
%
%   THE PARAMETERISED PATH IS RE-VALIDATED. contopptraj fits a smooth curve
%   THROUGH the waypoints; the curve is not the straight-line path the validator
%   approved, so it can bulge into an obstacle the search avoided. The samples
%   are therefore checked against both validators - physically everywhere, and
%   against the driving margin everywhere the escape does not cover - and a plan
%   whose reference is not collision-free is reported failed rather than handed to
%   a tracker.
%
%   REFERENCE HEADING IS CONTINUOUS, NOT WRAPPED. The heading row is unwrapped
%   before parameterisation and left unwrapped in RefPose, so RefVel's angular
%   component is the true reference rate. Wrapping would put a 2*pi step in the
%   position reference and an impulse in its derivative. A consumer comparing
%   RefPose(3,:) against a measured heading must use angdiff.
%
%   See also hmmBaseValidators, hmmOccupancyFromDepth, plannerBiRRT, contopptraj,
%   pathmetrics.

%   Copyright 2026 The MathWorks, Inc.

cfg = cachedConfig();

% THE PARTNER HEADING IS PLANNED AS A WHOLE SECOND DISPATCH AND COMPARED ON WHAT IT DRIVES.
% A stand-off's arrival cell admits a heading and roughly its reverse, and hmmStandoffRing
% picks between them knowing nothing about the route. Deciding from a straight-line guess of
% the arrival direction would be exactly the estimate that adds useless turns, so the decision
% is made on the two TIMED REFERENCES instead. Offline on Run 77's carry leg (4 draws) the
% partner cut the leg from 426 deg / 2.46 m reversed to 327 deg / 0.96 m, and on Run 97 both
% legs kept the primary heading - see choosePartner for the rule and its margin.
if nargin >= 8 && ~isempty(altGoalPose) && ...
        any(abs(double(altGoalPose(:)) - double(goalPose(:))) > 1e-9)
    bA = hmmPlanBasePath(gridMatrix, startPose, goalPose, carriedExtent, ...
        requestId, simTime, planTemplate);
    % THE PARTNER IS PLANNED ONLY WHEN IT COULD WIN, and "could" is exact, not a heuristic. Any route
    % to altGoalPose sweeps at least the net turn from the start heading to it, so that turn is a
    % lower bound on its driveCost; choosePartner needs cB + partnerMargin < cA. When the bound
    % already fails, the second full plan - half of all base planning (profiled 2026-10-14) - could
    % only ever be discarded, and the decision is identical without it.
    cA0   = driveCost(bA, cfg);
    lbAlt = abs(angdiff(double(startPose(3)), double(altGoalPose(3))));
    if isfinite(cA0) && lbAlt + cfg.plan.partnerMargin >= cA0
        bplan = bA;
        [~, yA, rA] = driveCost(bA, cfg);
        fprintf(['  base plan: goal heading %.0f deg costs %.2f (%.0f deg, %.2f m reversed); ' ...
            'partner %.0f deg needs >= %.2f, not planned -> goal\n'], rad2deg(goalPose(3)), cA0, ...
            rad2deg(yA), rA, rad2deg(altGoalPose(3)), lbAlt);
        if bplan.Valid
            hmmBasePlanTap(bplan, startPose, bplan.GoalPose, simTime);
        end
        return
    end
    bB = hmmPlanBasePath(gridMatrix, startPose, altGoalPose, carriedExtent, ...
        requestId, simTime, planTemplate);
    [bplan, usedAlt, cA, cB] = choosePartner(bA, bB, cfg);
    bplan.PlanTime = bA.PlanTime + bB.PlanTime;
    % One line per dispatch that HAD a partner - the only record of which heading the base was
    % sent to, since the chart's commanded GoalPose no longer says.
    [~, yA, rA] = driveCost(bA, cfg);
    [~, yB, rB] = driveCost(bB, cfg);
    fprintf(['  base plan: goal heading %.0f deg costs %.2f (%.0f deg, %.2f m reversed), ' ...
        'partner %.0f deg costs %.2f (%.0f deg, %.2f m) -> %s\n'], rad2deg(goalPose(3)), cA, ...
        rad2deg(yA), rA, rad2deg(altGoalPose(3)), cB, rad2deg(yB), rB, ...
        ternaryStr(usedAlt, 'PARTNER', 'goal'));
    if bplan.Valid
        hmmBasePlanTap(bplan, startPose, bplan.GoalPose, simTime);
    end
    return
end

bplan = blankPlan(planTemplate, cfg, requestId, simTime);
bplan.GoalPose = reshape(double(goalPose), 3, 1);
t0 = tic;

%% 1-2  The footprint, the two costmaps, the state space and the two validators
% All of it in one call, because hmmSelectBaseGoal has to certify goals against
% exactly this robot. See hmmBaseValidators' header for why the shape is a
% rectangle and not a radius.
[sv, svPhys, ss, geom] = hmmBaseValidators(gridMatrix, cfg, carriedExtent(:));

% Still reported as a single number, because the bus field is a single number and a
% consumer asking "how much room did the planner insist on" wants the binding figure.
% For a rectangle that is the covering radius plus the margin.
bplan.InflationRadius = geom.inflation;
margin = geom.margin;

startState = wrapState(startPose(:)', ss);
goalState  = wrapState(goalPose(:)',  ss);

if ~isStateValid(sv, goalState)
    % Searching TO an occupied state cannot succeed, and saying so immediately is
    % more useful than four thousand wasted iterations. This is the code the
    % stand-off geometry produces when it is too tight.
    bplan.FailureCode = cfg.fail.BaseGoalOccupied;
    bplan.PlanTime = toc(t0);
    return
end

% The start is allowed to be inside the margin and is not allowed to be inside an
% obstacle; see the header. searchState is where the search proper begins.
searchState = startState;
escapeLen   = 0;
if ~isStateValid(sv, startState)
    % A START IN CONTACT ONLY THROUGH THE HELD OBJECT IS STOOD IN, NOT COLLIDING. The 2-D
    % map flattens every obstacle to every height, while the object rides at the stow
    % height, and the arm's stow to that pose was just screened against the furniture in
    % 3-D: the robot is physically there, holding it. Measured 2026-10-15: a bottle picked
    % lying on the bathroom floor measured [0.152 0.061 0.138] m, its box stuck out past
    % the chassis over the basin, and the carry leg died BaseStartOccupied with no escape
    % offered - while the empty-handed body was clear and a 0.15 m escape existed. So for
    % that one case the PHYSICAL test inside the escape ball is the empty-handed body's;
    % every pose outside the ball still has to clear the carrying footprint plus the
    % driving margin (sv), which is stricter, so the rest of the path is unchanged.
    if ~isStateValid(svPhys, startState) && any(double(carriedExtent(:)) > 0)
        [~, svPhysEmpty] = hmmBaseValidators(gridMatrix, cfg, zeros(3, 1));
        if isStateValid(svPhysEmpty, startState)
            svPhys = svPhysEmpty;
        end
    end
    [searchState, escapeLen] = escapeMargin(startState, sv, svPhys, cfg, margin);
    if escapeLen <= 0
        bplan.FailureCode = cfg.fail.BaseStartOccupied;
        bplan.PlanTime = toc(t0);
        return
    end
end

%% 2b  Is there a route at all? Grid connectivity, before any sampling
%   BOTH SEEDS ARE NOW KNOWN GOOD, which is why the screen is here and not earlier. It
%   answers one question - does any corridor of free space join these two cells - and
%   it answers it by exhaustive search on a 240 x 240 grid, in milliseconds. A sampling
%   planner asked the same question spends its entire iteration budget discovering that
%   the answer is no, twice, and the supervisor then retries and it spends it again.
if ~connectedOnGrid(gridMatrix, cfg, geom, searchState, goalState)
    bplan.FailureCode = cfg.fail.BaseGoalUnreachable;
    bplan.PlanTime = toc(t0);
    return
end

%% 3-6  Draw a route, score it, and keep the BEST draw rather than the first
%   ONE DRAW USED TO BE THE WHOLE PLANNER, AND THAT WAS THE DEFECT. plannerBiRRT stops the
%   instant its two trees connect and returns that connection - it has no
%   ContinueAfterGoalReached to set, because there is no refinement stage to continue into.
%   Whatever route it stumbled onto drives, and stage 4b then faithfully points the chassis
%   along it. Measured on the place leg with the query held bit-identical, one draw in two
%   leaves the pick pose turning the wrong way by half a right angle and costs 4.5 s of
%   drive. hmmConfig at cfg.plan.baseDrawLimit carries the numbers, and why neither a seed
%   nor optimizePath is the answer.
%
%   THE SCORE IS DURATION, AND IT IS THE ONLY SCORE, which is a correction of this guard's
%   first version and the reason is worth keeping. Wasted rotation looks like the right test -
%   it names the defect exactly and it is scale-free, so one tolerance would serve every leg -
%   and it was used to accept a draw early and skip the remaining ones. It does not work,
%   because after unwrapping a path that REVERSES is still monotone in yaw: the tangent flips
%   by pi and keeps going the same way, so a cusped draw scores a perfect zero. Measured, an
%   early exit on it accepted two dispatches of twelve whose references then took 9.94 and
%   10.51 s against the 6.0 s the other draws of the same query managed. An early exit on an
%   unreliable metric is worse than none at all: it ships the bad draw AND skips the good one.
%
%   Duration has no such blind spot. It is what a cusp, a detour and a wrong-way rotation all
%   cost, in the units the complaint was made in, and across draws of the SAME query it is
%   directly comparable with no threshold to tune. It is also already computed - contopptraj
%   produces it in stage 5 - so scoring a draw costs nothing beyond the draw itself.
%
%   THE CHEAP EXIT IS BACK, ON A METRIC THAT EARNED IT. A draw that sweeps less yaw than
%   cfg.plan.baseSweepCap stops the loop, so a dispatch whose first draw is good pays for one
%   draw and not three. That is the same architectural move that failed on wasted rotation, and
%   the difference is evidence rather than argument. Over 120 draws of the place leg, replayed
%   so that every policy is scored on the SAME draws, the cap pays 66 draws where taking all
%   three pays 120 - 45 per cent less planning - for a mean drive of 6.27 s against 6.18 s, with
%   an identical 10.38 s worst case and an identical three slow legs in forty. A single draw, for
%   comparison, means 7.76 s and sixteen slow legs in forty. The sweep also condemns draws that
%   score a wasted rotation of exactly zero while taking 11 s, because they REVERSE.
%
%   IT CANNOT COST A DISPATCH ITS QUALITY, only its compute, because it gates when to stop
%   looking and never what ships - best is still the fastest draw seen, and a draw over the cap
%   stays eligible. A leg whose geometry honestly sweeps more than the cap fails it on every
%   draw and falls back to fastest-of-three, which is exactly the behaviour that ran before.
%
%   Wasted rotation is still measured and still traced, because it is the diagnostic that names
%   the defect for a reader - it is just not trusted to decide.
%
%   THE GUARD CAN ONLY IMPROVE A DISPATCH, NEVER COST ONE. It keeps the fastest draw seen, so
%   when every draw is poor it ships the least poor one, which is what a single draw produced
%   on average anyway. And earning a re-draw after a stage 6 rejection is a strict gain: that
%   dispatch used to fail outright.
%   AND DURATION IS NO LONGER THE WHOLE SCORE, BECAUSE DURATION ALONE BUYS CRABBING.
%   Stage 4b points the chassis along the route where the re-headed waypoints validate and
%   falls back to the sampled heading where they do not, so every draw arrives here already
%   labelled with which heading it ships. Pointing the chassis along the route COSTS TIME -
%   it spends seconds turning that a crabbing draw never spends, about 17 per cent on the
%   mean - so wherever both are available a duration-only score prefers the crab. Measured
%   over 60 dispatches of five routes with cfg.plan.baseHeadingTrace on: 17 of the 53 served
%   shipped the sampled heading, and SEVEN of those had a nose-first draw in hand and threw
%   it away to save between 0.18 and 2.96 s, mean 1.43 s. The comparison is therefore
%   lexicographic now - a nose-first draw beats a crabbing one, and duration decides between
%   draws of the same kind. It cannot cost a dispatch: best is still whichever ok draw was
%   seen, so a leg that only ever produces crabbing draws ships one exactly as before.
%
%   THE CHEAP EXIT CARRIED THE SAME BIAS MORE SHARPLY, AND IT WAS THE BIGGER HALF. sweep is
%   total yaw swept, and pointing the chassis along the route is precisely what sweeps yaw -
%   so a crabbing draw clears cfg.plan.baseSweepCap almost by construction while a nose-first
%   draw of the same route frequently does not. The remaining TEN of those 17 took exactly
%   ONE draw: the cap fired on a sampled first draw and the loop never looked for a
%   nose-first alternative at all. The exit now requires the draw to be nose-first as well as
%   smooth. The cost is compute on the legs that need it - a route whose geometry genuinely
%   cannot be driven nose-first pays the full cfg.plan.baseDrawLimit and then ships the crab,
%   which is the honest reading of "drive forwards where there is clearance".
best = struct('ok', false, 'code', cfg.fail.BaseSearchFailed, 'candName', 'none');
searchRetriesLeft = double(cfg.plan.baseSearchRetries);
attempt = 0;
while attempt < double(cfg.plan.baseDrawLimit)
    attempt = attempt + 1;
    dr = drawOnce(gridMatrix, cfg, sv, svPhys, ss, searchState, goalState, ...
        startState, escapeLen);
    traceDraw(cfg, attempt, dr);

    if dr.ok
        if betterDraw(dr, best)
            best = dr;
        end
        % ACCEPTED EARLY, AND WHAT SHIPS IS STILL best, NOT dr. A draw may clear the cap and
        % yet be slower than one already kept, in which case this stops drawing and ships the
        % faster one. The cap decides when to STOP LOOKING; the score above decides what ships.
        %
        % THE TEST IS drivesForward AND NOT noseFirst, AND THAT DISTINCTION IS THE WHOLE REASON
        % THE BASE DROVE BACKWARDS. noseFirst is true of a fully reversed route, and a reversed
        % route sweeps LESS yaw than the forward one - it skips the turn at the start - so it
        % cleared cfg.plan.baseSweepCap first and broke out of this loop before a forward draw
        % was ever drawn. The stopping rule was selecting for reverse directly, on top of
        % betterDraw being unable to tell the two apart. Settling for a reversed route is now
        % something this loop does only after it has spent every draw looking for a forward
        % one, which is the same bargain the comment above the loop already describes for the
        % crab: a route that cannot be driven nose-leading pays the full cfg.plan.baseDrawLimit
        % and then ships the reversal, honestly.
        if drivesForward(dr) && dr.sweep <= cfg.plan.baseSweepCap
            break
        end
        continue
    end

    if ~best.ok
        % The FIRST failure's code, so a consumer asking why a dispatch failed is told what
        % went wrong rather than what went wrong last.
        best.code = dr.code;
    end

    % A SEARCH FAILURE DOES NOT CONSUME DRAWS, which keeps the failure path exactly as cheap
    % as it was before this loop existed. connectedOnGrid has already proved a corridor
    % exists and both planners have already spent their full iteration budget hunting for
    % it, so a third and fourth attempt would triple the cost of the slowest case there is.
    % Rejections from stages 5 and 6 DO earn another draw: a different route genuinely may
    % time-parameterise and validate where this one could not.
    %
    % BUT A FAILED SEARCH IS RETRIED cfg.plan.baseSearchRetries TIMES, without consuming a draw,
    % because "a corridor exists and the budget was spent" does not make the next sample the same
    % one. Measured 2026-10-12: leaving the bedroom with the Bottle in hand, from an arrival the
    % inflated map calls occupied so every search begins with the escape, the identical query
    % succeeded on 1 of 3 calls - and the one failure, with the object held, ended the task. The
    % cost lands only on the failure path.
    if dr.code == cfg.fail.BaseSearchFailed
        if searchRetriesLeft > 0 && ~best.ok
            searchRetriesLeft = searchRetriesLeft - 1;
            attempt = attempt - 1;
            continue
        end
        break
    end
end

if ~best.ok
    bplan.FailureCode = best.code;
    bplan.PlanTime = toc(t0);
    return
end

%% 7  Publish the winning draw
bplan.PathLength   = best.pathLength;
bplan.Smoothness   = best.smoothness;
bplan.MinClearance = best.minClearance;
bplan.Truncated    = best.truncated;

% Published as USED, so bplan.Waypoints and bplan.RefPose describe the same drive.
nOut = best.nOut;
bplan.Waypoints(:, 1:nOut) = best.W;
bplan.NumWaypoints = uint8(nOut);

ref = best.ref;
nS = ref.nSamples;
bplan.RefTime(1:nS)     = ref.tSamp(1:nS)';
bplan.RefPose(:, 1:nS)  = ref.q(:, 1:nS);
bplan.RefVel(:, 1:nS)   = ref.qd(:, 1:nS);
bplan.NumSamples        = uint16(nS);
bplan.Duration          = ref.tSamp(nS);
% contopptraj's grid is non-uniform, so the follower's Polynomial Trajectory block takes
% RefTime as its time points. Those must be strictly increasing over the whole 256 entries.
bplan.RefTime           = hmmPadRefTime(bplan.RefTime, nS);

bplan.FailureCode = cfg.fail.OK;
bplan.Valid       = true;
bplan.PlanTime    = toc(t0);

% THE ROUTE, TO A DISPLAY CHANNEL AND NOWHERE ELSE. This is the last statement on
% the only path that produces a trajectory, so the tap sees exactly the plans the
% follower is handed and never a rejected draw. It writes root appdata, changes
% nothing here, cannot throw, and deleting the line would change no simulation
% result - see hmmBasePlanTap for why the map could not get this from a logged
% signal or from a second search.
hmmBasePlanTap(bplan, startPose, goalPose, simTime);

end

%% ------------------------------------------------------------------------
function tf = noseFirst(dr)
%NOSEFIRST Does this draw point the chassis along the route it drives?
%   ASKED OF THE CANDIDATE NAME, NOT OF THE GEOMETRY, because the name IS the decision
%   stage 4b made and re-deriving it from the heading channel would be a second, weaker
%   implementation of the same test.
%
%   BOTH TANGENT SIGNS COUNT. Reverse metres are longitudinal metres, which is the
%   convention cfg.plan.reverseWeight and the body-frame split of the heading
%   measurements already use: which way round the chassis drives is a separate question
%   from whether it drives along its own long axis, and it already has its own term.
tf = ~strcmp(dr.candName, 'sampled');
end

%% ------------------------------------------------------------------------
function tf = drivesForward(dr)
%DRIVESFORWARD Does this draw drive the route NOSE LEADING, rather than tail leading?
%   noseFirst above asks whether the chassis is aligned with its own path at all; this asks
%   which END of the chassis leads. The two were the same test for as long as the only
%   alternative to a tangent was the sampled crab, and keeping them the same test is what
%   made the base prefer driving backwards.
%
%   WHY THIS HAD TO BE SEPARATED, MEASURED ON A FULL RUN. Both legs of the 2026-10-02
%   transfer drove essentially entirely in reverse - median |heading - tangent| 177.4 and
%   175.9 deg, 4.247 m of reverse against 0.000 m forward on the pick leg and 5.052 against
%   0.128 on the carry - while cfg.plan.reverseWeight was doing its job and the per-draw sign
%   choice reported nose-first on 56 of 61 dispatches. The weight was never the problem. The
%   DRAW COMPARISON was: noseFirst is true for tangent-flipped, so a fully reversed route
%   scored identically to a forward one and betterDraw fell through to duration - and a
%   reversed route is usually the quicker of the two, because it skips the large turn at the
%   start that a forward route has to make. The early exit at the draw loop was worse still:
%   it stopped drawing on the first noseFirst draw inside cfg.plan.baseSweepCap, and a
%   reversed route sweeps LESS yaw for exactly the same reason, so it cleared the cap first
%   and no forward draw was ever drawn.
%
%   READ OFF THE NAME AND THE SIGN TOGETHER, because neither alone says which way round the
%   chassis ends up. tangentHeading returns the PREFERRED sign first, so 'tangent-preferred'
%   is reversed exactly when the sign choice preferred reversing, and 'tangent-flipped' - the
%   fallback the ladder reaches when the preferred sign fails stage 6 - is the opposite of
%   whatever that was. 'tangent-repaired' is built from the preferred parameterisation, so it
%   inherits its sign.
switch dr.candName
    case {'tangent-preferred', 'tangent-repaired', 'tangent-pivot-ends', 'tangent-zoned', ...
          'tangent-late-reverse'}
        % 'tangent-late-reverse' is MIXED, and ranking it with the preferred sign is the
        % honest reading rather than a convenience: it follows that sign for every metre
        % except the last stretch into the goal, 3.615 m of 4.254 measured on the pick leg.
        % Ranking it with the sign of its tail would make it indistinguishable from the
        % whole-route flip it exists to avoid.
        tf = ~dr.sgn.reversed;
    case 'tangent-flipped'
        tf = dr.sgn.reversed;
    otherwise
        % 'sampled' follows no tangent, so it leads with no end in particular. False keeps
        % this test from ever promoting a crab over a tangent; noseFirst has already ranked
        % it below both.
        tf = false;
end
end

%% ------------------------------------------------------------------------
function tf = betterDraw(dr, best)
%BETTERDRAW Is DR the draw to keep? Nose-first first, then duration.
%   LEXICOGRAPHIC AND NOT A WEIGHTED SUM, so there is no exchange rate between seconds and
%   sideways metres to tune - and a weighted sum would need one that held over every route
%   length, which is exactly the kind of threshold the duration score was chosen to avoid.
%
%   WITH cfg.plan.baseTangentHeading OFF THIS IS THE OLD COMPARISON. Every draw is then
%   labelled 'sampled', the first two tests are never decisive, and what remains is the
%   duration test they replace, unchanged.
%
%   THE MIDDLE TEST IS NEW AND IT IS WHY THE BASE USED TO DRIVE BACKWARDS. Without it the
%   comparison was lexicographic on (follows-a-tangent, duration), and since a reversed route
%   follows a tangent just as well as a forward one, DURATION decided every forward-versus-
%   reverse pair - which a reversed route wins, because it does not pay the turn at the start.
%   See drivesForward for the measurement. Direction is ranked ABOVE duration rather than
%   folded into it for the same reason the first test is: there is no exchange rate between
%   seconds and driving backwards that would hold over every route length.
%
%   AND IT CANNOT LOSE A DISPATCH. A reversed draw still beats a crab, and still ships when it
%   is the only thing that validates - the test only decides which of two VALIDATED draws is
%   kept. On a route that genuinely ought to reverse, where no forward draw ever validates,
%   every draw is reversed, this test is never decisive, and the comparison collapses to the
%   duration test exactly as before.
if ~best.ok
    tf = true;
    return
end
if noseFirst(dr) ~= noseFirst(best)
    tf = noseFirst(dr);
    return
end
if drivesForward(dr) ~= drivesForward(best)
    tf = drivesForward(dr);
    return
end
tf = dr.duration < best.duration;
end

%% ------------------------------------------------------------------------
function dr = drawOnce(gridMatrix, cfg, sv, svPhys, ss, searchState, goalState, ...
    startState, escapeLen)
%DRAWONCE One complete attempt: search, shorten, densify, point, time, validate.
%   Returns .ok and, when ok, everything the caller needs to publish the draw plus the two
%   scores it is judged on - .waste and .duration. On failure .code carries the verdict.
%
%   THIS IS A FUNCTION SO THAT EACH ATTEMPT GETS A FRESH SCOPE. Run inline, the loop would
%   carry sgn, candNames and the candidate index across attempts, and a draw that failed
%   early would be scored against the previous draw's leftovers.
dr = struct('ok', false, 'code', cfg.fail.BaseSearchFailed, 'waste', Inf, ...
    'sweep', Inf, 'maxStep', Inf, ...
    'duration', Inf, 'W', [], 'nOut', 0, 'pathLength', 0, 'smoothness', 0, ...
    'minClearance', 0, 'truncated', false, 'ref', [], 'candName', 'none', ...
    'sgn', struct('costF', NaN, 'costR', NaN, 'arcLen', 0, 'reversed', false, 'wStar', NaN));

%% 3  Search. BiRRT first because it is fast; RRT* on failure because it is
%     thorough. This division of labour is what makes having both worth it.
[pth, found] = searchWith(plannerBiRRT(ss, sv), cfg, searchState, goalState);
if ~found
    [pth, found] = searchWith(plannerRRTStar(ss, sv), cfg, searchState, goalState);
end
if ~found
    [pth, found] = searchViaBottleneck(gridMatrix, cfg, ss, sv, searchState, goalState);
end
if ~found
    return
end

%% 4  Shorten, then measure
shortened = shortenpath(pth, sv);

% Smoothness is measured on the SHORTENED path and clearance on a DENSIFIED one,
% and the asymmetry is deliberate. Both metrics are evaluated at the path's
% states: a six-state RRT path reports the clearance at six points and misses
% whatever the segments between them pass close to, while a densely resampled
% path reports a smoothness inflated by the near-collinearity of adjacent
% samples. Each metric is taken where it means something.
metricsCoarse = pathmetrics(shortened, sv);
dr.pathLength = pathLength(shortened);
dr.smoothness = smoothness(metricsCoarse);

nW = min(double(cfg.plan.baseWaypointCap), max(shortened.NumStates, 32));
dense = navPath(ss, shortened.States);
if nW > dense.NumStates
    interpolate(dense, nW);
end

% Clearance is measured against the RAW grid, so it is the absolute distance from each
% path state to the nearest occupied cell. That is a change and it is an improvement:
% read against an inflated map the figure is quantised to the inflation and every
% comfortably-clear plan reports the same small number. The reader who wants
% margin-on-top-of-the-footprint can subtract bplan.InflationRadius; the reader who
% wants the physical distance could not recover it from the old reading at all.
%
% It is also the only reading available: pathmetrics/clearance refuses a
% validatorVehicleCostmap outright ("Out of range subscript"). A separate uninflated
% validatorOccupancyMap on the same state space serves.
%
% AND IT IS MEASURED ON THE SEARCHED PATH, WHICH EXCLUDES THE ESCAPE. A stand-off is
% chosen to be close to something, so including the escape would report the distance to
% the very object the robot came to work on and would tell a consumer nothing about the
% drive. What the escape is checked against instead is the physical footprint, in stage 6
% and along its own straight segment.
dr.minClearance = clearance(pathmetrics(dense, rawValidator(gridMatrix, cfg, ss)));

W = dense.States';                                  % 3 x N
if escapeLen > 0
    % The reference must begin where the robot IS, not where the search began, or
    % the follower sees a step of up to the margin at handover.
    W = [startState', W];
    dr.pathLength = dr.pathLength + escapeLen;
end
cap = double(cfg.plan.baseWaypointCap);
dr.truncated = size(W, 2) > cap;
nOut = min(size(W, 2), cap);

%% 4b  Point the chassis along the route it is about to drive
%   THE BASE IS HOLONOMIC AND THAT IS NOT A LICENCE TO CRAB THE WHOLE WAY. The
%   headings on the path above are whatever the sampling planner drew, and nothing
%   downstream prefers forward motion: the contopptraj box is isotropic and
%   cfg.plan.thetaWeight barely charges for rotation. Before this stage existed the
%   base drove 0.811 sideways metres per forward metre. The same yaw is what clips
%   the bedroom doorway, because the chassis is longer than it is wide and a square
%   crossing is the narrowest one - see hmmConfig at cfg.plan.baseTangentHeading
%   for the whole measurement.
%
%   TRIED FIRST, NOT IMPOSED. The search validated the SAMPLED headings, so
%   re-pointing the chassis where clearance only existed in the sampled orientation
%   can put a waypoint in a jamb; unconditionally re-heading serves 39 routes of 46
%   against 42. Trying the tangent and falling back serves 43 - better than either,
%   because the tangent also rescues a route the sampled heading clips.
%
%   WHICH WAY ROUND IS A SEPARATE QUESTION FROM WHICH WAY TO POINT, and it used to be
%   settled by endpoint rotation alone - see cfg.plan.reverseWeight. Both signs are now
%   offered, PREFERRED FIRST, so the sign choice cannot cost a dispatch: the set below is
%   a superset of what shipped before it, and the second parameterisation is only paid
%   when the preferred sign's reference fails stage 6.
%
%   AND THE FALLBACK IS NO LONGER ALL-OR-NOTHING, WHICH IS WHAT KEPT THE LAST OF THE
%   CRABBING ALIVE. The rewrite was accepted or rejected as a whole route, so ONE waypoint
%   in a jamb condemned the heading of every other waypoint on the drive and the chassis
%   crabbed the entire leg. Measured with the trace on, after the draw score was fixed:
%   7 of 53 served dispatches still shipped the sampled heading, and every one of them had
%   paid two or three draws first, so the whole-route rewrite was failing repeatedly rather
%   than unluckily. repairHeading gives ground ONLY at the waypoints that do not validate -
%   which is the per-waypoint version of the test this stage used to apply once - so a route
%   with one bad corner drives nose-first everywhere else. It is a THIRD candidate and not a
%   replacement, because a repaired route asks for more rotation than either pure one and
%   must not be preferred to a tangent that validates outright.
%
%   BUILT ONLY IF IT IS REACHED, which is why the third slot holds a CLOSURE rather than a
%   matrix: repairHeading costs two validator calls per waypoint it has to fix, and the four
%   dispatches in five whose preferred tangent validates outright must not pay for it. Stage
%   5-6 calls the handle when it gets that far and never otherwise.
Wsampled = W(:, 1:nOut);
candidates = {Wsampled};
candNames  = {'sampled'};
if cfg.plan.baseTangentHeading
    [Wpref, Walt, dr.sgn, geo] = tangentHeading(Wsampled, startState(3), goalState(3), ...
        cfg.plan.baseHeadingBlend, cfg.plan.reverseWeight, cfg.plan.baseTangentWindow);
    % THE REPAIR COMES BEFORE THE FLIP, AND THAT ORDER IS WHY THE BASE NO LONGER DRIVES
    % BACKWARDS. Both of them are fallbacks for a preferred channel that stage 6 rejected, but
    % they give away completely different amounts: repairHeading surrenders the tangent at the
    % handful of waypoints that do not validate and keeps the PREFERRED SIGN everywhere else,
    % whereas Walt reverses the WHOLE ROUTE. Measured on the pick leg of the 2026-10-02
    % transfer, three draws of three: 'tangent-preferred' rejected with
    % cfg.fail.BaseRefCollide, 'tangent-flipped' validated, and the base drove 3.95 m of 3.98
    % backwards - while the sign choice itself preferred nose-first on every draw (w* 0.53-0.61
    % against cfg.plan.reverseWeight 0.750). The reversal was never chosen; it was reached,
    % because it sat in front of the repair in this list. A route whose forward chassis clips
    % one corner should give up that corner, not its direction of travel.
    %
    % IT CANNOT COST A DISPATCH, for the same reason the flip could not: this is still a
    % superset of what shipped before, in a different order, and every candidate that validated
    % before still validates. What changes is only WHICH of two validating fallbacks is taken.
    % Nor does it cost the common case any compute - the repair is still a CLOSURE, still built
    % only if it is reached, so the four dispatches in five whose preferred tangent validates
    % outright still never call it. It is now paid on exactly the dispatches that used to pay
    % for Walt instead.
    % AND THE LATE REVERSAL COMES BEFORE THE FLIP FOR THE SAME REASON THE REPAIR DOES: it
    % gives away 0.6 m of reversing where the flip gives away the whole route. The three
    % fallbacks now surrender strictly increasing amounts - a few waypoints' yaw, then the
    % last stretch's direction, then every metre of it - and the ladder stops at the first
    % one that validates. See lateReverse for the measurement that put it here.
    % pivotEnds SITS BETWEEN THE REPAIR AND lateReverse, by the same surrender ordering: it gives
    % away where the turns happen and a short slide at each end, where lateReverse gives away a
    % 180 deg turn and a reversed stretch. See pivotEnds for the carry leg it rescues.
    % zonedHeading SITS AFTER pivotEnds, so the rungs above still ship wherever they did; it is
    % for the route that clips at ONE tight spot - a doorway, a stand-off slot, a corner - and
    % used to lose its whole heading there: on 2026-10-20 every forward rung failed on the
    % 10.8 m bathroom -> kitchen leg and the flip drove all of it tail-first and sideways. It
    % gives way only where the chassis does not fit and is back on the route within
    % cfg.plan.baseHeadingBlend after it.
    candidates = {Wpref, ...
        @() repairHeading(Wpref, Wsampled, sv, svPhys, ss, startState, escapeLen, cfg), ...
        @() pivotEnds(Wsampled, geo, startState(3), goalState(3), sv, svPhys, ss, ...
            startState, escapeLen, cfg), ...
        @() zonedHeading(Wpref, Wsampled, sv, svPhys, ss, startState, escapeLen, cfg), ...
        @() lateReverse(Wsampled, geo, startState(3), goalState(3), ...
            cfg.plan.baseHeadingBlend, sv, svPhys, ss, startState, escapeLen, cfg), ...
        Walt, Wsampled};
    candNames  = {'tangent-preferred', 'tangent-repaired', 'tangent-pivot-ends', ...
        'tangent-zoned', 'tangent-late-reverse', 'tangent-flipped', 'sampled'};
end

%% 5-6  Time-parameterise, then require the REFERENCE to be collision-free
ref = struct('ok', false, 'code', cfg.fail.BaseTimeParamFailed);
for c = 1:numel(candidates)
    % A DEFERRED CANDIDATE IS BUILT HERE AND WRITTEN BACK, so dr.W below still reads the
    % matrix that was timed rather than the handle that made it.
    if isa(candidates{c}, 'function_handle')
        candidates{c} = candidates{c}();
    end
    % A CANDIDATE MAY DECLINE TO EXIST, which lateReverse does on a route with nowhere to turn
    % round. Skipping leaves ref holding the PREVIOUS candidate's verdict, which is what a
    % consumer should be told anyway: the reason this draw failed is the reason the candidate
    % the planner would have shipped failed, and a candidate that was never built has no verdict.
    % A CANDIDATE MAY DECLINE TO EXIST, AND IT MAY NOT CHANGE WIDTH. The width test is a
    % contract, not a sanity check: stage 7 publishes into a fixed-width bus field with
    % best.nOut, which was fixed above from the SAMPLED channel, so a candidate of any other
    % width overruns the field and takes the whole dispatch down with a size error rather
    % than falling back. Rejecting it here costs the ladder one rung and nothing else.
    if isempty(candidates{c}) || size(candidates{c}, 2) ~= nOut
        if cfg.plan.baseHeadingTrace
            fprintf('hmmPlanBasePath:   cand %-20s not available\n', candNames{c});
        end
        continue
    end
    ref = timeAndValidate(candidates{c}, cfg, sv, svPhys, ss, startState, escapeLen);
    % WHICH CANDIDATE LOST AND WHY, which the draw trace cannot say because it only ever sees
    % the WINNER. A leg that drives backwards because 'tangent-flipped' shipped has had its
    % forward channel rejected here, and the verdict code is the whole diagnosis - a collision
    % rejection means the forward chassis sweep genuinely does not fit, and a time-parameter
    % rejection means it would fit if it were given the time to turn.
    if cfg.plan.baseHeadingTrace && ~ref.ok
        fprintf('hmmPlanBasePath:   cand %-18s rejected, code %d\n', candNames{c}, ref.code);
    end
    if ref.ok
        break
    end
end
if ~ref.ok
    % The LAST attempt's code, which is the sampled heading's whenever the rewrite is on: a
    % consumer asking why a draw failed wants the verdict on the reference the planner would
    % have shipped, not on either variant it preferred.
    dr.code = ref.code;
    return
end

%% 6b  Score the draw on the two axes the caller chooses between draws on
%   MEASURED ON THE TIMED REFERENCE, NOT ON THE WAYPOINTS, because the reference is what the
%   follower tracks. contopptraj fits a curve THROUGH the waypoints and may bulge between
%   them, so a waypoint channel that looks monotone can still hand the base a reversal.
nS = ref.nSamples;
dr.ok       = true;
dr.code     = cfg.fail.OK;
dr.waste    = wastedRotation(ref.q(3, 1:nS));
[dr.sweep, dr.maxStep] = headingSweep(ref.q(3, 1:nS));
dr.duration = ref.tSamp(nS);
dr.W        = candidates{c};
dr.nOut     = nOut;
dr.candName = candNames{c};
dr.ref      = ref;
dr.truncated = dr.truncated || ref.overCap;

end

%% ------------------------------------------------------------------------
function w = wastedRotation(th)
%WASTEDROTATION Radians of rotation spent and then undone along a heading channel.
%   Zero for a monotone turn, which is what a clean leg does. The recorded 360x640 run's
%   place leg scores 1.29 rad: the chassis turned 73.7 deg one way, stalled, and turned 156
%   the other. That is the defect this is the accept test for.
%
%   IT CANNOT SEE A CUSP, and the caller closes that gap with duration rather than this being
%   made cleverer. After unwrapping, a path that reverses direction is still MONOTONE in yaw
%   - the tangent flips by pi and keeps going the same way - so it scores a perfect zero.
%   Adding a sweep term here would also charge honest legs for the turning they genuinely
%   need, which is why the second metric is duration and not total rotation.
h = unwrap(th);
net = h(end) - h(1);
if net >= 0
    w = max(0, h(1) - min(h));      % excursion below the start, against a net + turn
else
    w = max(0, max(h) - h(1));      % excursion above the start, against a net - turn
end
end

%% ------------------------------------------------------------------------
function [s, mx] = headingSweep(th)
%HEADINGSWEEP Total radians of yaw the chassis turns through, and the largest single step.
%   s is the ARC LENGTH of the heading channel, so unlike wastedRotation it charges for turning
%   whether or not the turn is later undone - which is exactly why it SEES A CUSP that
%   wastedRotation is blind to. A reversal shows up as a pi step, monotone in yaw and therefore
%   invisible to an excursion test, but unmistakable here.
%
%   IT CANNOT BE USED ALONE, because a leg that genuinely needs to turn pays it too. The place
%   leg's honest draws sweep 168 to 200 deg for about 46 deg of net heading change: the chassis
%   must first swing from its arrival pose onto the route tangent and then off it again onto
%   the place yaw. A cap tight enough to reject a cusp would reject those. mx is returned for
%   the same reason - it separates "turned a lot, smoothly" from "flipped by pi once".
h  = unwrap(th);
d  = abs(diff(h));
s  = sum(d);
if isempty(d)
    mx = 0;
else
    mx = max(d);
end
end

%% ------------------------------------------------------------------------
function traceDraw(cfg, attempt, dr)
%TRACEDRAW Which draw did what, said out loud when asked.
%   Nothing downstream records this - bplan carries the waypoints but not which draw or which
%   parameterisation produced them - so every attempt to check the sign choice against a
%   recorded run has had to INFER it from the driven heading, and the reconstruction of costF
%   and costR from the driven path disagrees with cfg.plan.reverseWeight on the place leg by
%   about 0.13 rad/m. This settles which of the two readings is right: if the trace says
%   "tangent-flipped" then the preferred sign was rejected by stage 6 and the fallback drove,
%   and if it says "tangent-preferred" on a leg that drove backwards then the preferred sign
%   WAS the reversed one and the reconstruction is what is wrong.
%
%   It now also says what each draw scored, which is the only way to see the guard working:
%   a dispatch that prints two lines re-drew, and the wasted rotation on the first is why.
%
%   Default OFF, and it must stay that way: hmmPlanBasePath is extrinsic, so this prints into
%   the command window mid-simulation, once per DRAW per dispatch.
if ~cfg.plan.baseHeadingTrace
    return
end
if ~dr.ok
    fprintf('hmmPlanBasePath: draw %d REJECTED, code %d\n', attempt, dr.code);
    return
end
fprintf(['hmmPlanBasePath: draw %d shipped %s | sign %s | wasted %5.1f deg | ' ...
         'sweep %5.1f deg | step %5.1f deg | %5.2f s | ' ...
         'costF %.3f costR %.3f arc %.3f w* %.3f vs w %.3f\n'], ...
    attempt, dr.candName, reverseWord(dr.sgn.reversed), rad2deg(dr.waste), ...
    rad2deg(dr.sweep), rad2deg(dr.maxStep), dr.duration, ...
    dr.sgn.costF, dr.sgn.costR, dr.sgn.arcLen, dr.sgn.wStar, cfg.plan.reverseWeight);
end

%% ------------------------------------------------------------------------
function s = reverseWord(tf)
%REVERSEWORD The sign decision as a word, for the trace above.
if tf
    s = 'REVERSED';
else
    s = 'nose-first';
end
end

%% ------------------------------------------------------------------------
function Wfix = repairHeading(Wtan, Wsam, sv, svPhys, ss, startState, escapeLen, cfg)
%REPAIRHEADING Keep the tangent heading where it fits; give ground only where it does not.
%   THE PROBLEM THIS SOLVES IS THAT THE REWRITE WAS ALL-OR-NOTHING. tangentHeading produces a
%   whole route and stage 6 accepts or rejects a whole route, so a single waypoint whose
%   re-headed chassis clips a door jamb threw away the heading of every OTHER waypoint on the
%   drive and the base crabbed the entire leg. That is a bad trade on its face: one corner
%   cannot be a reason to drive ten metres sideways.
%
%   SO THE TEST STAGE 6 APPLIES ONCE IS APPLIED PER WAYPOINT, with the same two validators and
%   the same escape-ball exemption, and the waypoints that pass keep the tangent. The ones that
%   fail walk a short ladder back toward the heading the SEARCH sampled - a third of the way,
%   two thirds, then all the way, which is the sampled heading exactly, since the step is
%   angdiff(tangent, sampled). Three rungs rather than a fine sweep because the whole point is
%   to give up as little yaw as the obstacle demands, not to find an optimum: the rungs cost
%   two validator calls each and anything finer is paying for precision stage 6 cannot see.
%
%   THE TWO END WAYPOINTS ARE NOT TOUCHED. They carry the COMMANDED start and goal headings,
%   which the stand-off geometry depends on being met exactly - the arrival heading is the one
%   thing the user asked to stay "whatever is reachable". Skipping them is free rather than a
%   compromise: the sampled channel's ends are those same commanded headings, so the ladder's
%   step there is zero and there is nothing to give.
%
%   IT CANNOT MAKE A DRAW WORSE. The result is still judged by stage 6 as a whole, and it sits
%   BELOW both pure tangents in the candidate order, so it is only ever reached when neither of
%   those validated. If it fails too, the sampled heading follows it exactly as before.
Wfix = Wtan;
N = size(Wtan, 2);
if N < 3
    return
end
idx = (2:N-1)';
xy  = Wtan(1:2, idx)';
th  = Wtan(3, idx)';
sam = Wsam(3, idx)';
ok  = @(h) validRows(sv, svPhys, ss, xy, h, startState, escapeLen, cfg);

bad = ~ok(th);
if ~any(bad)
    return                      % the tangent fits everywhere; stage 6 rejected it between
end                             % waypoints, and no amount of per-waypoint yaw will fix that
for a = [0.34 0.67]
    thTry = th + a * angdiff(th, sam);
    fixed = bad & ok(thTry);
    th(fixed) = thTry(fixed);
    bad(fixed) = false;
    if ~any(bad)
        break
    end
end
th(bad) = sam(bad);             % the last rung, taken unconditionally: if even the sampled
Wfix(3, idx) = th';             % heading does not fit there, this candidate was never viable
end

%% ------------------------------------------------------------------------
function tf = validRows(sv, svPhys, ss, xy, th, startState, escapeLen, cfg)
%VALIDROWS Stage 6's pose test, asked of a batch of waypoints instead of a reference.
%   DELIBERATELY THE SAME TWO TESTS IN THE SAME ORDER as stage 6, including the escape ball:
%   inside the ball the robot already stands where it stands, so holding those poses to the
%   DRIVING margin would report a clip that stage 6 will not, and repairHeading would hand
%   back yaw it never needed to spend. A weaker copy of the accept test is worse than none,
%   because it gives ground for free.
S  = wrapState([xy th], ss);
tf = isStateValid(svPhys, S);
outside = hypot(S(:, 1) - startState(1), S(:, 2) - startState(2)) > ...
    escapeLen + 1 / cfg.map.resolution;
if escapeLen <= 0
    outside(:) = true;
end
chk = tf & outside;
if any(chk)
    tf(chk) = isStateValid(sv, S(chk, :));
end
end

%% ------------------------------------------------------------------------
function Wpe = pivotEnds(W, geo, thStart, thGoal, sv, svPhys, ss, startState, escapeLen, cfg)
%PIVOTENDS Hold each end's heading, turn only where there is room, drive the middle on the tangent.
%   WHY THIS RUNG EXISTS, MEASURED. Replaying Run 65's carry leg (pick stand-off at the coffee
%   table to the kitchen counter) six times, EVERY forward rung - preferred, repaired and
%   lateReverse - failed stage 6 on every draw, and 'tangent-flipped' shipped: 8.5-10.6 m driven
%   entirely backwards, which is what the user saw. The sign choice preferred nose-first each
%   time; the blocker is the START. A stand-off lies parallel to the furniture, so the base cannot
%   turn onto the route inside it, and blendEnds asks it to turn over the first blendLen metres.
%   lateReverse only relaxes the GOAL end, so the only thing left that fit was turning the route
%   round. This relaxes BOTH ends the way only a holonomic base can:
%
%     start  hold thStart and translate out (sideways or backwards, whatever the route asks) to
%            the FIRST waypoint that admits a full circle of headings, and turn onto the tangent
%            there, the shorter way;
%     middle follow the preferred tangent, nose-leading where that is the preferred sign;
%     end    turn to thGoal at the LAST full-circle waypoint, the shorter way, and translate in
%            holding it - a sideways slide into a stand-off rather than lateReverse's U-turn.
%
%   It surrenders less than lateReverse (no 180 deg turn, no reversed stretch) and far less than
%   the flip, so it sits between the repair and lateReverse. Each turn is stationary columns of
%   at most 30 deg, paid for by thinning the middle so the candidate keeps its width (see the
%   column contract at stage 5-6), and stage 6 judges the result like every other rung.
Wpe = [];
if isempty(geo.tang) || geo.total <= 0
    return
end
N = size(W, 2);
if N < 6
    return
end

% ROOM FOR THE TURN THAT IS ACTUALLY MADE, NOT FOR A FULL CIRCLE. lateReverse asks for a full
% circle because it does not know which way it will turn; this rung does, so it asks only for the
% arc it sweeps, sampled every 7.5 deg. Measured on the carry leg, the full-circle rule put the
% first turning cell ~2.4 m along the route and the base backed all the way out to it.
stepMax = pi / 6;
tang = geo.tang;
arcOk = @(i, a, b) all(validRows(sv, svPhys, ss, repmat(W(1:2, i)', ...
    max(ceil(abs(angdiff(a, b)) / (pi / 24)), 1) + 1, 1), ...
    (a + linspace(0, 1, max(ceil(abs(angdiff(a, b)) / (pi / 24)), 1) + 1) * angdiff(a, b))', ...
    startState, escapeLen, cfg));
i1 = [];
for i = 2:N-3
    if arcOk(i, thStart, tang(i))
        i1 = i;
        break
    end
end
if isempty(i1)
    return
end
k = [];
for i = N-1:-1:i1+2
    if arcOk(i, tang(i), thGoal)
        k = i;
        break
    end
end
if isempty(k)
    return
end

d1 = angdiff(thStart, tang(i1));
% The middle, unwrapped so that it continues the start turn without a 2*pi step.
tm = tang + 2 * pi * round((thStart + d1 - tang(i1)) / (2 * pi));
d2 = angdiff(tm(k), thGoal);
n1 = max(ceil(abs(d1) / stepMax) - 1, 0);
n2 = max(ceil(abs(d2) / stepMax) - 1, 0);

% The turn columns come out of the middle, thinned uniformly; k itself is always kept.
mid = i1 + 1:k;
nKeep = numel(mid) - (n1 + n2);
if nKeep < 2
    return
end
keepMid = unique(round(linspace(i1 + 1, k, nKeep)));
if numel(keepMid) ~= nKeep
    return
end

Wpe = [[W(1:2, 1:i1); repmat(thStart, 1, i1)], ...
       [repmat(W(1:2, i1), 1, n1); thStart + (1:n1) * (d1 / (n1 + 1))], ...
       [W(1:2, keepMid); tm(keepMid)], ...
       [repmat(W(1:2, k), 1, n2); tm(k) + (1:n2) * (d2 / (n2 + 1))], ...
       [W(1:2, k+1:N); repmat(tm(k) + d2, 1, N - k)]];
end

%% ------------------------------------------------------------------------
function Wlr = lateReverse(W, geo, thStart, thGoal, blendLen, sv, svPhys, ss, startState, ...
    escapeLen, cfg)
%LATEREVERSE Drive the route on the preferred sign, then turn round and back into the goal.
%   WHY THIS CANDIDATE HAD TO EXIST, AND IT IS THE REASON THE BASE DROVE BACKWARDS FOR FOUR
%   METRES TO PICK UP A MUG. A stand-off pose is by construction wedged against the thing the
%   robot came to work on, so the chassis has to lie PARALLEL to that furniture face - and the
%   route arrives roughly PERPENDICULAR to it. Measured at the two real stand-offs of the
%   2026-10-02 transfer, sweeping the driving validator over a full circle of headings at the
%   arrival cell: the coffee-table pose admits only 31 headings of 73, all within about 40 deg
%   of the x axis, and the kitchen-counter pose only 19 of 73, within about 20 deg. So the
%   arrival heading the arm needs is not reachable nose-leading, and it is not reachable by
%   pirouetting on the spot either, because the blocked band sits between the two.
%
%   THE OLD LADDER THEREFORE HAD ONLY ONE WAY TO SATISFY THE GOAL HEADING: reverse the WHOLE
%   route. 'tangent-preferred' was rejected with cfg.fail.BaseRefCollide on every draw - not
%   between waypoints that could be repaired, but because the terminal turn cannot be made at
%   all - and 'tangent-flipped' then validated and drove 4.236 m of 4.236 backwards. The sign
%   choice itself had preferred nose-first throughout (w* 0.53-0.68 against
%   cfg.plan.reverseWeight 0.750), so no weight anywhere would have changed the outcome.
%
%   WHAT THIS DOES INSTEAD IS WHAT A DRIVER DOES: go forwards until the last place with room to
%   turn round, turn round there, and reverse the short distance into the slot. The pivot goes
%   at the LAST waypoint that admits a full circle of headings, so the reversed stretch is as
%   short as the geometry allows. On the pick leg that waypoint is 3.83 m along a 4.45 m route,
%   and the resulting reference drives 3.615 m forwards and 0.639 m backwards - 15 per cent
%   reversed where the shipped draw was 100 - clear at all 256 samples under both validators,
%   for 11.59 s against 8.33.
%
%   THAT 3.26 s IS THE HONEST PRICE AND IT IS PAID ONLY WHERE IT BUYS SOMETHING. The candidate
%   sits BELOW both pure tangents and the repair, so a route whose forward channel validates
%   outright never builds it; and it sits ABOVE 'tangent-flipped', so the whole-route reversal
%   is now what the ladder reaches when even this fails. betterDraw ranks it with the preferred
%   sign, which is honest - the bulk of the drive follows it - so it beats a flipped draw of any
%   duration and loses to a clean forward draw that is quicker.
%
%   IT IS WRITTEN FOR THE PREFERRED SIGN AND NOT FOR 'FORWARD', which is what keeps it general.
%   On the one leg that genuinely ought to reverse - bedroom to kitchen, where the sign choice
%   picks reverse and cfg.plan.reverseWeight documents that every weight below 0.9 keeps it -
%   this builds the mirror image: reverse for the bulk, nose-first into the goal. There is no
%   direction named anywhere below.
Wlr = [];
if isempty(geo.tang) || geo.total <= 0
    return
end
N = size(W, 2);
if N < 4
    return
end

% THE LAST ROTATION-FREE WAYPOINT, FOUND FROM THE BACK, which is both the right answer and the
% cheap way to get it: the pivot wants to be as late as possible so the reversed stretch is as
% short as possible, and searching backwards stops at the first hit rather than scanning the
% whole route. A full circle is required rather than just the two headings the pivot passes
% through, because a cell that cannot be rotated in is a cell where a follower with any
% tracking error at all is in the furniture.
hs = linspace(-pi, pi, 25)';
hs(end) = [];                       % -pi and +pi are the same heading; keep one
k = 0;
for i = N-1:-1:2
    xy = repmat(W(1:2, i)', numel(hs), 1);
    if all(validRows(sv, svPhys, ss, xy, hs, startState, escapeLen, cfg))
        k = i;
        break
    end
end
if k == 0
    return                          % nowhere on this route has room to turn round
end

% The two halves, each blended to the end it owns. The forward half keeps the commanded start
% heading; the reversed half keeps the commanded goal heading, which is the one the arm's reach
% depends on and the one thing that may not be given away.
tang = geo.tang;
s    = geo.s;
tot  = geo.total;
wS   = min(s / max(blendLen, eps), 1);
thA  = tang + (1 - wS) .* angdiff(tang, repmat(thStart, 1, N));
tb   = tang + pi;
wG   = min((tot - s) / max(blendLen, eps), 1);
thB  = tb + (1 - wG) .* angdiff(tb, repmat(thGoal, 1, N));

% THE PIVOT IS STATIONARY COLUMNS AND NOT A STEP IN THE CHANNEL. A 180 deg jump between two
% waypoints that are 0.1 m apart is a reference that turns while it translates, which is
% precisely the manoeuvre the arrival cell has no room for - and contopptraj would smooth it
% into a bulge nothing validated. Holding the position for six 30 deg steps instead asks the
% base to do what it is holonomically able to do: stop, spin, carry on.
%
% AND THE PIVOT PAYS FOR ITS OWN COLUMNS, which is not tidiness, it is the defect this
% candidate shipped with for one development run. bplan.Waypoints is a FIXED-WIDTH bus field and
% hmmPlanBasePath publishes into it with best.nOut, so a candidate that is wider than the
% sampled one it was built from overruns the field - "the size of the left side is 3-by-32 and
% the size of the right side is 3-by-37". Nothing in a candidate's contract promised headroom
% and the interpolation at stage 4 is free to fill the cap exactly, so the rule is that a
% candidate returns the width it was handed. The five columns come out of the FORWARD block,
% thinned uniformly: on the pick leg that rarefies its spacing from 0.143 m to about 0.176 m,
% which is a milder change than the draw-to-draw variation between two BiRRT routes of the
% same query, and stage 6 judges the result either way.
steps = 6;
if k - 2 < steps - 1
    return                          % the forward block is too short to lend the columns
end
keep = unique(round(linspace(1, k, k - (steps - 1))));
if numel(keep) ~= k - (steps - 1)
    return                          % the thinning collapsed; do not publish a short channel
end
piv = thA(k) + (1:steps - 1) * (pi / steps);
Wlr = [[W(1:2, keep); thA(keep)], ...
       [repmat(W(1:2, k), 1, steps - 1); piv], ...
       [W(1:2, k+1:N); thB(k+1:N)]];
end

%% ------------------------------------------------------------------------
function Wz = zonedHeading(Wtan, Wsam, sv, svPhys, ss, startState, escapeLen, cfg)
%ZONEDHEADING The tangent wherever the chassis fits along it; the sampled heading only where not.
%   THE RULE A DRIVER USES THROUGH A DOOR: square up for the tight spot, then face the way you
%   are going again. The rungs above are whole-route answers - a tangent, or a tangent with both
%   ends held - so one doorway that the tangent clips sends the ladder down to the flip or the
%   sampled crab, and the heading of every metre is given away for that one spot. Asked by the
%   user on 2026-10-20 after the bathroom -> kitchen leg drove 8.6 m tail-first and 5.2 m
%   sideways: "whatever heading for the doorway is fine, but from the living room to the kitchen
%   there is a lot of space".
%
%   THE ZONES START WHERE A DENSE CHECK OF THE TANGENT CLIPS AND GROW WHERE STAGE 6 DOES. Each
%   segment between waypoints is checked at half-cell spacing with the tangent heading - the
%   same two validators stage 6 uses (validRows) - and both ends of a clipping segment take the
%   heading plannerBiRRT SAMPLED there, which is the heading the search already validated. The
%   result is then timed and validated for real; contopptraj's curve is not the straight segment
%   that was checked, so where the reference still clips, the waypoints within growRadius of the
%   first hit join the zone and it is tried again, a bounded number of times. Outside the zones
%   the weight on the sampled heading ramps to zero over cfg.plan.baseHeadingBlend, which is how
%   far past a tight spot the base is back on its route.
%
%   EMPTY WHEN THE TANGENT FITS EVERY SEGMENT, because then this candidate IS the preferred one,
%   which stage 6 has already refused between waypoints, and nothing here would change that.
growRadius = 0.35;      % m of waypoints around a stage-6 hit that join the zone
maxGrow    = 4;         % zone growths before the candidate is handed back as it stands
Wz = [];
N = size(Wtan, 2);
if N < 3
    return
end
xy  = Wtan(1:2, :);
s   = [0, cumsum(vecnorm(diff(xy, 1, 2), 2, 1))];
thT = unwrap(Wtan(3, :));
thS = Wsam(3, :);
step = 0.5 / cfg.map.resolution;
bad = false(1, N);
for i = 1:N-1
    n = max(2, ceil((s(i+1) - s(i)) / step) + 1);
    a = linspace(0, 1, n)';
    P = xy(:, i)' + a .* (xy(:, i+1) - xy(:, i))';
    H = thT(i) + a .* (thT(i+1) - thT(i));
    if ~all(validRows(sv, svPhys, ss, P, H, startState, escapeLen, cfg))
        bad([i, i+1]) = true;
    end
end
if ~any(bad)
    return
end
for grow = 0:maxGrow
    Wz = zonedChannel(Wtan, thT, thS, s, bad, cfg.plan.baseHeadingBlend);
    ref = timeAndValidate(Wz, cfg, sv, svPhys, ss, startState, escapeLen);
    if ref.ok || isempty(ref.hit)
        return
    end
    d = vecnorm(xy - ref.hit(1:2)', 2, 1);
    near = d <= growRadius;
    if ~any(near)
        near = d == min(d);
    end
    if all(bad(near))
        return              % the hit is already inside a zone: growing cannot help
    end
    bad(near) = true;
end
end

%% ------------------------------------------------------------------------
function Wz = zonedChannel(Wtan, thT, thS, s, bad, blendLen)
%ZONEDCHANNEL The sampled heading inside the zones, the tangent outside, ramped over blendLen.
%   The two end columns keep Wtan's, which are the commanded start and goal headings.
N = numel(s);
sb = s(bad);
w = zeros(1, N);
for i = 1:N
    w(i) = max(0, 1 - min(abs(s(i) - sb)) / max(blendLen, eps));
end
Wz = Wtan;
Wz(3, :) = thT + w .* angdiff(thT, thS);
Wz(3, [1 N]) = Wtan(3, [1 N]);
end

%% ------------------------------------------------------------------------
function [Wpref, Walt, sgn, geo] = tangentHeading(W, thStart, thGoal, blendLen, reverseWeight, ...
    tangentWindow)
%TANGENTHEADING Rewrite the heading channel to follow the path, keeping both ends.
%   Returns the PREFERRED sign first and the opposite sign second, so the caller can
%   fall back rather than lose a route to the sign choice. SGN reports the decision -
%   the two costs, the arc length, and the break-even weight - for the trace below;
%   it is computed either way, so asking for it costs nothing.
%
%   The tangent is taken by CENTRAL difference on position, so an interior waypoint's
%   heading depends on the segment either side of it rather than on whichever
%   neighbour comes first - a forward difference alone puts a step in the heading at
%   every corner, which is exactly what contopptraj would then have to smooth away.
%
%   The blend is by ARC LENGTH and not by waypoint index, because the interpolation
%   upstream is uniform in the state space rather than in metres, so an index-based
%   blend would cover a different distance on every route.
Wpref = W;
Walt  = W;
sgn   = struct('costF', NaN, 'costR', NaN, 'arcLen', 0, 'reversed', false, 'wStar', NaN);
% The raw pieces, handed out so lateReverse can build a THIRD parameterisation from the same
% tangent rather than recomputing it. Empty until they exist, so every early return below
% leaves a geo a caller can test with isempty and skip.
geo   = struct('tang', [], 's', [], 'total', 0);
N = size(W, 2);
if N < 2
    return
end

x = W(1, :);
y = W(2, :);
dx = zeros(1, N);
dy = zeros(1, N);
dx(2:N-1) = x(3:N) - x(1:N-2);
dy(2:N-1) = y(3:N) - y(1:N-2);
dx(1) = x(2) - x(1);        dy(1) = y(2) - y(1);
dx(N) = x(N) - x(N-1);      dy(N) = y(N) - y(N-1);

tang = atan2(dy, dx);
% A stationary waypoint has no tangent, so carry the last one forward rather than
% letting atan2(0, 0) plant a zero heading in the middle of the route.
still = hypot(dx, dy) < 1e-9;
if all(still)
    return
end
if still(1)
    tang(1) = tang(find(~still, 1, 'first'));
end
for i = 2:N
    if still(i)
        tang(i) = tang(i - 1);
    end
end

s = [0, cumsum(hypot(diff(x), diff(y)))];
total = s(end);
if total <= 0
    return
end

% THE TANGENT IS SMOOTHED OVER ARC LENGTH, BECAUSE A SHORTENED BiRRT ROUTE IS A POLYLINE WITH
% DOGLEGS AND THE CHASSIS WAS POINTED ALONG EVERY ONE. Measured on Run 65's home->pick leg: a
% 0.6 m kink north before the route turned north-west swung the chassis 180 -> 76 -> 143 deg,
% about 170 deg of turning to settle 40 deg from where it started; replays swept 139-213 deg on
% a 4 m drive. A holonomic base does not need to face a short kink - it can crab through it -
% so the heading follows the route's direction averaged over +-cfg.plan.baseTangentWindow of
% arc (a circular mean of unit vectors, so it cannot wrap). Corners longer than the window are
% still followed; only the wiggle is dropped. Stage 6 still validates the result, and a heading
% that clips something falls through to the repair exactly as before.
win = tangentWindow;
if win > 0
    c = cos(tang);
    sn = sin(tang);
    tSm = tang;
    for i = 1:N
        m = abs(s - s(i)) <= win;
        tSm(i) = atan2(sum(sn(m)), sum(c(m)));
    end
    tang = tSm;
end

% REVERSE IS STILL LONGITUDINAL, and insisting on nose-first would sometimes mean a
% 180 degree spin the task never asked for - so the choice stays, and it now has a
% distance in it. Both terms below are radians of rotation AT THE TWO ENDPOINTS;
% reverseWeight * total converts the metres the route would spend going backwards into
% the same currency, so a reversal has to save that many radians per reversed metre to
% be worth taking. Without the third term this comparison was scale-free, and a 1.8 m
% reversal that saved a quarter of a radian read as cheaply as a 0.2 m one would - which
% is what drove the reference run's place leg 96 per cent backwards. See
% cfg.plan.reverseWeight.
costF = abs(angdiff(thStart, tang(1)))      + abs(angdiff(thGoal, tang(N)));
costR = abs(angdiff(thStart, tang(1) + pi)) + abs(angdiff(thGoal, tang(N) + pi)) ...
        + reverseWeight * total;
if costR < costF
    tangPref = tang + pi;
    tangAlt  = tang;
else
    tangPref = tang;
    tangAlt  = tang + pi;
end

% The decision, in the form the trace reports it. costR is recorded WITHOUT the
% arc-length term so that wStar - the weight at which the choice would flip - is read
% straight off the pair, and a recorded run can be compared against a reconstruction of
% the same two numbers from the driven path.
sgn.costF    = costF;
sgn.costR    = costR - reverseWeight * total;
sgn.arcLen   = total;
sgn.reversed = costR < costF;
sgn.wStar    = (sgn.costF - sgn.costR) / max(total, eps);

geo.tang  = tangPref;
geo.s     = s;
geo.total = total;

Wpref(3, :) = blendEnds(tangPref, s, total, thStart, thGoal, blendLen);
Walt(3, :)  = blendEnds(tangAlt,  s, total, thStart, thGoal, blendLen);
end

%% ------------------------------------------------------------------------
function th = blendEnds(tang, s, total, thStart, thGoal, blendLen)
%BLENDENDS Hold the commanded heading at each end and reach the tangent blendLen along.
%   The goal blend is applied to the ALREADY-BLENDED heading, so a route shorter than two
%   blends degenerates to a smooth turn between the two commanded headings rather than to
%   a discontinuity in the middle of it.
%
%   THE RESULT IS DELIBERATELY NOT WRAPPED, and a wrap here cost a measurement. Stage 5
%   unwraps this channel before handing it to contopptraj, so wrapToPi is a no-op up to a
%   constant 2*pi - and that constant is enough to change the rounding of every downstream
%   float. Adding one moved the reference run's place leg by 1e-6 m at t = 7.96 s, which
%   grew to 19.6 mm by the end of the leg, moved the place arrival, and changed summed arm
%   travel from 22.30 to 21.23 rad. Nothing about the drive was different in intent; the
%   whole delta was rounding. Leave the channel continuous and let stage 5 unwrap it.
N  = numel(tang);
wS = min(s / max(blendLen, eps), 1);
th = tang + (1 - wS) .* angdiff(tang, repmat(thStart, 1, N));
wG = min((total - s) / max(blendLen, eps), 1);
th = th   + (1 - wG) .* angdiff(th,   repmat(thGoal,  1, N));
end

%% ------------------------------------------------------------------------
function ref = timeAndValidate(Wu, cfg, sv, svPhys, ss, startState, escapeLen)
%TIMEANDVALIDATE Stages 5 and 6 on one candidate set of waypoints.
%   Returns .ok and, when ok, .q .qd .tSamp .nSamples .overCap. On failure .code
%   carries BaseTimeParamFailed or BaseRefCollide, exactly as the inline version did.
ref = struct('ok', false, 'code', cfg.fail.BaseTimeParamFailed, 'q', [], 'qd', [], ...
    'tSamp', [], 'nSamples', 0, 'overCap', false, 'hit', []);

Wu(3, :) = unwrap(Wu(3, :));

% INSIDE the boxes, not ON them - cfg.base.planMargin, the same idiom
% hmmScreenArmPlan applies to the arm. A reference that rides vxMax exactly is a
% reference the plant's own TwistLimit clamp re-emerges two ulps above, which
% SafetyMonitor's strict > latches as BASE_VEL. It is also a reference the follower
% has no authority to correct upward from, because it is already at the clamp.
m = cfg.base.planMargin;
velLim = m * [-cfg.base.vxMax,    cfg.base.vxMax; ...
              -cfg.base.vyMax,    cfg.base.vyMax; ...
              -cfg.base.omegaMax, cfg.base.omegaMax];
accLim = m * [-cfg.base.axMax,    cfg.base.axMax; ...
              -cfg.base.ayMax,    cfg.base.ayMax; ...
              -cfg.base.alphaMax, cfg.base.alphaMax];

cap = double(cfg.plan.baseSampleCap);
try
    [q, qd, ~, tSamp, solnInfo] = contopptraj(Wu, velLim, accLim, NumSamples = cap);
catch
    return
end
if solnInfo.ExitFlag ~= 1
    return
end

nS = min(size(q, 2), cap);

%% 5b  Re-time so the BODY-FRAME demand is one the follower can deliver
% contopptraj bounds each WORLD axis separately, so a diagonal is planned at sqrt(2) times the
% per-axis limit - 0.96 m/s and 1.44 m/s^2 - while PathFollowing clamps the BODY twist at
% cmdMargin * vxMax = 0.76 m/s and BaseAccelLimit at 1.08 m/s^2. Nose-first on a diagonal the
% whole excess lands on body vx. Measured on Run 100's pick leg: body vx pinned at 0.760 for
% two seconds, the base fell behind, and the next corner raised cross-track to 0.151 m against
% a 0.15 bail - a FollowFailed replan on a route that was fine. So time is stretched locally
% wherever the body-frame speed or acceleration exceeds the planner's own margin of the base
% box; position, and therefore everything stage 6 validates, is untouched.
[qd, tSamp] = bodyFrameRetime(q, qd, tSamp, nS, cfg);

%% 6  The reference itself must be collision-free, not just the waypoints
%   TWO CHECKS, AND THE STRICTER ONE IS UNIVERSAL. Every sample, escape included,
%   must clear the base's physical footprint - there is no pose on any path that is
%   allowed to drive the chassis into the furniture. Only samples that have left the
%   ball the escape covers are additionally held to the driving margin, because
%   inside that ball the robot already stands where it stands.
%
%   With no escape the ball is empty, every sample is held to the margin, and this
%   is exactly the check it replaces. The physical test is then implied by the
%   margin test - more inflation can only occupy more cells - so it costs the
%   no-escape case nothing but a second pass over 256 samples.
ref.code = cfg.fail.BaseRefCollide;
S = wrapState(q(:, 1:nS)', ss);
okPhys = isStateValid(svPhys, S);
if ~all(okPhys)
    ref.hit = S(find(~okPhys, 1), :);       % where, for zonedHeading
    traceCollide(cfg, 'physical', ref.hit);
    return
end
% One cell of slack, because contopptraj fits a curve THROUGH the waypoints and the
% curve may bulge a little past the straight escape segment. The slack cannot excuse
% a collision: everything inside the ball has just been checked physically.
outside = hypot(S(:, 1) - startState(1), S(:, 2) - startState(2)) > ...
    escapeLen + 1 / cfg.map.resolution;
if escapeLen <= 0
    outside(:) = true;
end
if any(outside)
    So = S(outside, :);
    okMargin = isStateValid(sv, So);
    if ~all(okMargin)
        ref.hit = So(find(~okMargin, 1), :);
        traceCollide(cfg, 'margin', ref.hit);
        return
    end
end

ref.ok       = true;
ref.code     = cfg.fail.OK;
ref.q        = q;
ref.qd       = qd;
ref.tSamp    = tSamp;
ref.nSamples = nS;
ref.overCap  = size(q, 2) > cap;
end

%% ------------------------------------------------------------------------
function traceCollide(cfg, which, s)
%TRACECOLLIDE Where a reference first failed stage 6, under cfg.plan.baseHeadingTrace only.
if cfg.plan.baseHeadingTrace
    fprintf('hmmPlanBasePath:     first %s collision at [%.2f %.2f %.0f deg]\n', ...
        which, s(1), s(2), rad2deg(s(3)));
end
end

%% ------------------------------------------------------------------------
function map = gridAsMap(gridMatrix, cfg, radius)
%GRIDASMAP The observed grid as an occupancyMap, optionally inflated.
%   The costmaps the planner validates against are hmmBaseValidators' business. This is
%   for the two places that still want a plain occupancyMap over the same grid: the
%   uninflated one clearance is read from, and the inscribed-radius one the connectivity
%   screen searches. Written once so neither can drift from the other, or from
%   hmmBaseValidators, in resolution, origin or thresholds.
map = occupancyMap(gridMatrix, cfg.map.resolution);
map.GridLocationInWorld = cfg.map.origin;
map.OccupiedThreshold   = cfg.map.occupiedThreshold;
map.FreeThreshold       = cfg.map.freeThreshold;
if radius > 0
    inflate(map, radius);
end
end

%% ------------------------------------------------------------------------
function sv = rawValidator(gridMatrix, cfg, ss)
%RAWVALIDATOR A validator over the UNINFLATED grid, for measuring clearance only.
%   Never used to accept or reject a pose - a point robot would drive through walls.
%   pathmetrics/clearance needs a validator to find the obstacles, and refuses a
%   validatorVehicleCostmap with "Out of range subscript", so this is what answers it.
%   Built on the state space hmmBaseValidators returned, so the states it is handed are
%   states in the same space with the same limits.
sv = validatorOccupancyMap(ss, Map = gridAsMap(gridMatrix, cfg, 0));
sv.ValidationDistance = cfg.plan.baseValidationDist;
end

%% ------------------------------------------------------------------------
function ok = connectedOnGrid(gridMatrix, cfg, geom, startState, goalState)
%CONNECTEDONGRID Whether ANY route joins two poses, decided exhaustively on the grid.
%
%   THE ASYMMETRY IS THE WHOLE DESIGN, and it is one-sided on purpose: this may say yes
%   where the sampling search then fails, and it may NOT say no where the sampling search
%   would have succeeded. A false yes costs one wasted search - the behaviour without the
%   screen. A false no refuses a reachable destination, which is a new bug. So every
%   uncertainty below resolves to true.
%
%   WHY IT IS SOUND TO SCREEN A FOOTPRINT SEARCH WITH A POINT SEARCH. The screen inflates
%   by geom.inscribed = the footprint's half-width plus the margin, which is no larger
%   than the radius the driving costmap grows obstacles by. Every pose the footprint
%   validator accepts therefore has its ORIGIN at least that far from any obstacle - the
%   origin is inside the rectangle, so it is inside the middle covering circle - so every
%   state on every path the real planner could ever return is free on this map too. A
%   point search over a superset of the reachable set cannot rule out a reachable goal.
%
%   IT IS ALSO ROTATION-BLIND, WHICH IS THE SAME ARGUMENT. Dropping the heading only ever
%   adds routes, because it permits turns the footprint would foul. That is the permitted
%   direction of error.
%
%   WHAT IT COSTS. Building and inflating the map, then a full 240 x 240 expansion in the
%   worst case, which is the walled-off case and which measured 0.196 s. That is the price
%   of the answer nobody was getting: the same question put to plannerBiRRT ran 4000
%   iterations, then plannerRRTStar ran its own, then the supervisor retried and both ran
%   again, all inside time steps in which the model clock does not advance.

ok = true;

map = gridAsMap(gridMatrix, cfg, geom.inscribed);

% BOTH SEEDS MUST BE FREE CELLS OR THERE IS NOTHING TO ASK. plannerAStarGrid ERRORS with
% nav:navalgs:plannerastargrid:OccupiedLocation on an occupied start or goal rather than
% reporting no path, and either seed can be occupied here for a reason that is not a
% reachability failure: world2grid snaps a pose to a cell centre up to half a cell
% diagonal away, and this inflation is a hair under the one the goal was already accepted
% at. Skipping the screen hands the case back to the search, which is where it was before.
rc  = world2grid(map, [startState(1:2); goalState(1:2)]);
if any(getOccupancy(map, rc, 'grid') >= map.OccupiedThreshold)
    return
end

% The no-path warning is expected output here, not a diagnostic - it is precisely the
% answer being asked for - and it must not print on every refused goal. Suppressed by ID
% rather than by 'all' so a genuine warning from inside the planner still surfaces.
ws = warning('off', 'nav:navalgs:plannerastargrid:NoPathFound');
try
    route = plan(plannerAStarGrid(map), rc(1, :), rc(2, :));
    ok = ~isempty(route);
catch
    % A screen that cannot run is not a screen that refuses. See the asymmetry above.
    ok = true;
end
warning(ws);
end

%% ------------------------------------------------------------------------
function [escapeState, escapeLen] = escapeMargin(startState, sv, svPhys, cfg, margin)
%ESCAPEMARGIN The nearest state out of the driving margin that the base can reach.
%
%   Returns escapeLen = 0 when there is none, which the caller reports as
%   BaseStartOccupied. Two distinct situations produce that answer and both deserve
%   it: the chassis is in contact with something, or it is wedged in a pocket of
%   margin with no way out inside the bound the header derives.
%
%   PURE TRANSLATION, NO ROTATION. The escape keeps the start heading, because the
%   mecanum base can translate sideways and because a segment that only translates
%   is a straight line in the plane - which is what isMotionValid can check
%   honestly. Rotating while inside the margin would also sweep the arm's overhang
%   through space this function is not modelling.
%
%   BACKWARDS FIRST. Bearings are tried in order of how nearly they point behind the
%   robot, because the reason the base is inside the margin is almost always that it
%   drove up to something to work on it, and the way out of that is the way it came.
%   Forward is where the object it is standing at is, so forward is tried last.
%
%   RINGS, AND THEIR RESOLUTION IS A REAL LIMIT. The disc is sampled on rings one
%   map cell apart at nBearing bearings, so a free pocket narrower than the arc
%   between two bearings at the radius where it appears can be missed - about 60 mm
%   at the outermost ring with 24 bearings. Missing one reports BaseStartOccupied,
%   which is a refusal to move rather than an unsafe move.

% The bound the header derives: a base that is not physically in collision cannot be
% deeper into the inflated region than the margin part of the inflation, plus one
% cell diagonal for the grid's own quantisation.
cellSize  = 1 / cfg.map.resolution;
escapeMax = margin + sqrt(2) * cellSize;

escapeState = startState;
escapeLen   = 0;

% IN CONTACT IS NOT IN THE MARGIN, and no escape may be offered for it. Everything
% below trusts that the straight segment out of the start is physically clear, and
% that trust is only earned if the start itself is.
if ~isStateValid(svPhys, startState)
    return
end

nBearing = 24;
delta    = wrapToPi((0:nBearing - 1) * 2 * pi / nBearing);
[~, ord] = sort(abs(delta));
bearings = startState(3) + pi + delta(ord);

radii = cellSize:cellSize:escapeMax;
if isempty(radii) || radii(end) < escapeMax
    radii(end + 1) = escapeMax;
end

for r = radii
    for b = bearings
        cand = [startState(1) + r * cos(b), startState(2) + r * sin(b), startState(3)];
        if ~isStateValid(sv, cand)
            continue
        end
        if ~isMotionValid(svPhys, startState, cand)
            continue
        end
        escapeState = cand;
        escapeLen   = r;
        return
    end
end
end

%% ------------------------------------------------------------------------
function [pth, found] = searchViaBottleneck(gridMatrix, cfg, ss, sv, startState, goalState)
%SEARCHVIABOTTLENECK BiRRT in two halves, split at the narrowest point of the grid route.
%   A NARROW PASSAGE IS WHAT A SAMPLING PLANNER IS WORST AT, AND A DOORWAY IS ONE IN SE(2). The
%   bedroom door leaves the base 0.35 m of clearance either side, against 0.275 m needed when it
%   is lined up with the opening and 0.366 m when it is not, so a pose that passes must be in the
%   doorway AND nearly aligned with it. Measured 2026-10-12 leaving the bedroom for the kitchen:
%   2 of 8 single BiRRT searches succeeded at the shipped budget and 4 of 8 at three times it,
%   empty-handed or carrying, from the parked pose or 0.35 m further in - the route, not the load
%   or the start. So the route is found on the grid first (plannerAStarGrid on the map inflated
%   by the base's half-width plus cfg.safety.plannerMargin, the least any heading needs), its
%   narrowest cell becomes a waypoint headed along the route, and BiRRT solves the two halves.
%   The halves are joined into ONE path before shortening and timing, so the base drives through
%   the waypoint without stopping. Runs only after BiRRT and RRT* have both failed.
pth   = [];
found = false;
res   = cfg.map.resolution;
occ   = gridMatrix >= cfg.map.occupiedThreshold;
map   = binaryOccupancyMap(occ, res);
map.GridLocationInWorld = cfg.map.origin(1:2);
inflate(map, 0.5 * cfg.base.chassisSize(2) + cfg.safety.plannerMargin);
try
    route = plan(plannerAStarGrid(map), world2grid(map, startState(1:2)), ...
        world2grid(map, goalState(1:2)));
catch
    return          % start or goal inside the inflation, or no grid route at all
end
if size(route, 1) < 8
    return
end
clr = bwdist(occ) / res;
cl  = clr(sub2ind(size(clr), route(:, 1), route(:, 2)));
ends = 4;                                   % not the stand-off ends: those are close on purpose
cl([1:ends, end - ends + 1:end]) = Inf;
[~, k] = min(cl);
xy  = grid2world(map, route);

% THE PASSAGE AXIS, FROM THE MAP AND NOT FROM THE ROUTE. An 8-connected grid route crosses a door
% at whatever slant its neighbours allow, and the first version headed the waypoint along it: the
% base then entered the bedroom door 5-15 deg off its axis and every timed reference clipped the
% frame (BaseRefCollide at [5.7..6.4 -5.5]). The axis is instead the direction in which a segment
% of +-cfg.plan.bottleneckRun through the bottleneck keeps the most clearance - along a corridor,
% never across it - and the route's own direction only decides which way round to drive it.
run  = cfg.plan.bottleneckRun;
c0   = xy(k, :);
best = -Inf;
for a = deg2rad(0:5:175)
    u   = [cos(a) sin(a)];
    seg = c0 + (-run:1 / res:run)' * u;
    ij  = world2grid(map, seg);
    in  = all(ij >= 1, 2) & ij(:, 1) <= size(clr, 1) & ij(:, 2) <= size(clr, 2);
    if ~all(in)
        continue
    end
    m = min(clr(sub2ind(size(clr), ij(:, 1), ij(:, 2))));
    if m > best
        best = m;
        th   = a;
    end
end
if ~isfinite(best)
    return
end
d = xy(min(k + 3, end), :) - xy(max(k - 3, 1), :);
if dot(d, [cos(th) sin(th)]) < 0
    th = wrapToPi(th + pi);                 % drive it the way the route goes
end
u  = [cos(th) sin(th)];
W3 = [c0 - run * u, th; c0, th; c0 + run * u, th];
if ~all(isStateValid(sv, W3)) || ~isMotionValid(sv, W3(1, :), W3(2, :)) || ...
        ~isMotionValid(sv, W3(2, :), W3(3, :))
    return
end
[p1, f1] = searchWith(plannerBiRRT(ss, sv), cfg, startState, W3(1, :));
if ~f1
    return
end
[p2, f2] = searchWith(plannerBiRRT(ss, sv), cfg, W3(3, :), goalState);
if ~f2
    return
end
pth   = navPath(ss, [p1.States; W3(2, :); p2.States]);
found = true;
wp    = c0;
fprintf('hmmPlanBasePath: route found through the bottleneck at [%.2f %.2f] (%.2f m clear).\n', ...
    wp(1), wp(2), cl(k));
end

%% ------------------------------------------------------------------------
function [pth, found] = searchWith(planner, cfg, startState, goalState)
%SEARCHWITH One search attempt with the configured budget.
planner.MaxIterations         = cfg.plan.baseMaxIterations;
planner.MaxConnectionDistance = cfg.plan.baseMaxConnDistance;
if isprop(planner, 'EnableConnectHeuristic')
    % BiRRT only: join the two trees directly instead of stopping at
    % MaxConnectionDistance, which is what makes the bidirectional search pay.
    planner.EnableConnectHeuristic = true;
end
[pth, info] = plan(planner, startState, goalState);
found = info.IsPathFound && pth.NumStates >= 2;

% A FOUND PATH MUST END ON THE GOAL, AND THE RRT* FALLBACK'S DOES NOT. Its default
% GoalReachedFcn, nav.algs.checkIfGoalIsReached, accepts any state within 0.5 of the goal in
% the SE(2) metric, and nothing downstream appended the goal itself - so the reference ended
% wherever that state was. Measured on the 2026-10-16 floor place: the base drove its place
% leg, stopped 0.47 m short of a goal it reported Valid, stood for the 2 s stall timer, and
% only the replan's short hop reached the stand-off. Offline, one draw in six of the same leg
% ended 0.361 m short. BiRRT joins its trees AT the goal, so this only ever bites the
% fallback; it is applied to every search because it costs one motion check.
%
% THE CLOSING SEGMENT IS CHECKED, NOT ASSUMED: if the straight move from the last state to
% the goal is not collision-free the search has not found the goal, and the caller's next
% fallback runs instead of shipping a short route.
if found
    last = pth.States(end, :);
    if distance(planner.StateSpace, last, goalState) > 1e-6
        if isMotionValid(planner.StateValidator, last, goalState)
            append(pth, goalState);
        else
            found = false;
        end
    end
end
end

%% ------------------------------------------------------------------------
function s = wrapState(s, ss)
%WRAPSTATE Heading into the state space's bounds so the validator accepts it.
%   Only the heading is wrapped. XY is clamped instead: a position outside the
%   map is a real problem and silently wrapping it to the far side of the
%   apartment would hide it. enforceStateBounds does exactly that for stateSpaceSE2.
s = enforceStateBounds(ss, s);
end

%% ------------------------------------------------------------------------
function bplan = blankPlan(planTemplate, cfg, requestId, simTime)
%BLANKPLAN A BasePlan that vouches for nothing.
bplan = planTemplate;
bplan.RequestId       = uint32(requestId);
bplan.Waypoints(:)    = 0;
bplan.NumWaypoints    = uint8(0);
bplan.RefTime(:)      = 0;
bplan.RefTime         = hmmPadRefTime(bplan.RefTime, 0);
bplan.RefPose(:)      = 0;
bplan.RefVel(:)       = 0;
bplan.NumSamples      = uint16(0);
bplan.Duration        = 0;
bplan.PathLength      = 0;
bplan.MinClearance    = 0;
bplan.Smoothness      = 0;
bplan.InflationRadius = 0;
bplan.PlanTime        = 0;
bplan.Truncated       = false;
bplan.FailureCode     = cfg.fail.PathPlanFailed;
bplan.Timestamp       = simTime;
bplan.Valid           = false;
end

%% ------------------------------------------------------------------------
function cfg = cachedConfig()
cfg = hmmRuntimeConfig();
end

%% ------------------------------------------------------------------------
function [qd, tSamp] = bodyFrameRetime(q, qd, tSamp, nS, cfg)
%BODYFRAMERETIME Stretch time locally so the body-frame speed stays inside the plan box.
%   The factor at each sample is its body-velocity ratio against cfg.base.planMargin of the
%   base box (the margin contopptraj was given), floored at 1, then SMOOTHED along the
%   reference - a moving maximum so no sample is under-stretched, then a moving average so
%   the factor ramps rather than steps. A stepped factor turns into a step in RefVel, which
%   is an acceleration spike: measured, a per-segment factor took the pick leg's peak
%   body-frame acceleration from 3.0 to 8.2 m/s^2. Acceleration is deliberately NOT a
%   criterion: the body frame rotates with the chassis, so its acceleration carries an
%   omega x v term that contopptraj's world-axis box never sees (the unmodified reference
%   already reads 2.4-3.0 m/s^2), and slowing down only shrinks it. Velocities are divided
%   by the factor, so RefVel stays consistent with the stretched RefTime; a reference already
%   inside the box is returned bit-identical.
if nS < 3 || ~cfg.plan.baseBodyRetime
    return
end
m   = cfg.base.planMargin;
vLx = m * cfg.base.vxMax;
vLy = m * cfg.base.vyMax;
t   = reshape(double(tSamp(1:nS)), 1, nS);
th  = q(3, 1:nS);
c   = cos(th);
s   = sin(th);
vbx = c .* qd(1, 1:nS) + s .* qd(2, 1:nS);
vby = -s .* qd(1, 1:nS) + c .* qd(2, 1:nS);
f   = max(1, max(abs(vbx) / vLx, abs(vby) / vLy));
if all(f <= 1)
    return
end
w = max(1, round(cfg.plan.baseRetimeWindow * nS));
f = movmean(movmax(f, 2 * w + 1), 2 * w + 1);
f = max(f, 1);
dt   = diff(t);
tNew = [t(1), t(1) + cumsum(dt .* 0.5 .* (f(1:end-1) + f(2:end)))];
qd(:, 1:nS) = qd(:, 1:nS) ./ f;
tSamp(1:nS) = reshape(tNew, size(tSamp(1:nS)));
end

%% ------------------------------------------------------------------------
function [bplan, usedAlt, cA, cB] = choosePartner(bA, bB, cfg)
%CHOOSEPARTNER Keep the goal's plan unless the partner's is CLEARLY cheaper.
%   Cost is radians of yaw swept plus cfg.plan.reverseWeight per metre driven tail-first, both
%   measured on the timed reference - the two things a viewer reads as wasted motion, in the
%   exchange rate the sign choice already uses. The partner must win by
%   cfg.plan.partnerMargin, so a near-tie keeps the heading hmmStandoffRing chose and the
%   partner can only ever remove motion, never trade it. An invalid plan costs Inf, so the
%   partner also rescues a goal that does not plan at all.
cA = driveCost(bA, cfg);
cB = driveCost(bB, cfg);
usedAlt = cB + cfg.plan.partnerMargin < cA;
if usedAlt
    bplan = bB;
else
    bplan = bA;
end
end

function [c, yaw, rev] = driveCost(bp, cfg)
%DRIVECOST Yaw swept plus weighted reverse metres on a plan's reference; Inf if it did not plan.
c = Inf;
yaw = NaN;
rev = NaN;
if ~bp.Valid
    return
end
n   = double(bp.NumSamples);
q   = double(bp.RefPose(:, 1:n));
dq  = diff(q, 1, 2);
th  = q(3, 1:end-1);
vx  = cos(th) .* dq(1, :) + sin(th) .* dq(2, :);
yaw = sum(abs(diff(unwrap(q(3, :)))));
rev = sum(max(-vx, 0));
c   = yaw + cfg.plan.partnerReverseWeight * rev;
end

function s = ternaryStr(c, a, b)
if c
    s = a;
else
    s = b;
end
end
