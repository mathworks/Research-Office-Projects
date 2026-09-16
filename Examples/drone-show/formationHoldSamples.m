function [pos, vel, times, info] = formationHoldSamples(P, t0, T, nSamples, turns, ...
    vTarget, peakRatio, aTarget, peakRatio2)
%formationHoldSamples  Trajectory samples for one hold, rotating or still.
%
%   [POS, VEL, TIMES, INFO] = formationHoldSamples(P, T0, T, NSAMPLES, TURNS,
%   VTARGET, PEAKRATIO, ATARGET, PEAKRATIO2) returns the plan samples for a
%   single hold on the formation P (N-by-3, NED, one row per drone, already in
%   drone order).
%
%   POS   is N-by-3-by-NSAMPLES, ready to drop straight into trajectory_data.
%   VEL   is N-by-3-by-NSAMPLES of commanded velocity (the feedforward the
%         position controller reads from columns 4:6).
%   TIMES is 1-by-NSAMPLES of show times, spanning [T0, T0+T].
%   INFO  reports what the rotation actually managed -- see below.
%
%   NSAMPLES == 1 means DO NOT ROTATE, and returns exactly one sample at T0
%   holding P with zero velocity. That is the shape the plan had before
%   rotation existed, so a non-rotating show is bit-identical to what it always
%   was rather than merely close.
%
% WHY THE HOLD IS WHERE ROTATION LIVES
%
% The plan is sparse and interpolated: a hold used to be ONE anchor, and the
% next anchor (the first sample of the following transition) carries the same
% positions, so linear interpolation between them is a fleet standing still.
% Rotation is therefore not a change to the transitions at all -- it is a hold
% that gains a sample series. Nothing in the model needed touching for it.
%
% WHAT ROTATES, AND ABOUT WHAT
%
% A rigid rotation of the whole formation about the VERTICAL axis through its
% own horizontal centroid. Each drone keeps its altitude and traces a circle
% whose radius is its distance from that axis, so drones near the centre barely
% move and the outermost ones sweep the widest arc. A Sphere spins as a ball, a
% Grid as a turntable, and a text billboard turns edge-on and back -- all three
% fall out of the same two lines of arithmetic.
%
% Rigid rotation is an ISOMETRY, so every pairwise distance is preserved
% exactly. A rotating hold therefore cannot breach d_min if the static
% formation clears it, and min_sep_achieved is unchanged by switching rotation
% on. This is the one part of the feature that needs no verification by
% measurement -- though it was measured anyway, because "should be an isometry"
% and "is implemented as one" are different claims. Note also that an isometry in
% the PLAN says nothing about the FLOWN result: the drones still have to track it,
% and that is where the separation margin actually gets spent.
%
% WHY THE SWEEP IS CAPPED, AND WHY IT IS USUALLY LESS THAN A FULL TURN
%
% TWO things bind, and for a sustained circle the second one binds harder.
%
% Tangential speed is the obvious one: a drone at radius r turning at rate w
% flies at w*r, and the fleet only tracks about vTarget. Inverting the min-jerk
% peak the same way the takeoff and the transitions are sized:
%
%   w_peak = peakRatio * theta / T          (peakRatio = 35/16 for minjerkpolytraj)
%   w_peak * rMax <= vTarget   =>   theta <= vTarget*T / (peakRatio*rMax)
%
% ACCELERATION is the one that actually matters here, and sizing rotation on
% speed alone was wrong. Every straight segment in this show accelerates only to
% change speed, so bounding speed bounds acceleration too by implication. A
% circle does not: holding radius r at rate w demands a centripetal w^2*r
% CONTINUOUSLY, for the whole hold, whatever the tangential speed is. The total
% commanded acceleration is the tangential and radial parts in quadrature,
%
%   a = r * sqrt(alpha^2 + w^4),   alpha = theta*s''(t),  w = theta*s'(t)
%
% and bounding it by aTarget gives a quadratic in theta^2. Both peaks are taken
% simultaneously, which they are not -- s' peaks at the midpoint of the hold and
% s'' at about 28% of it -- so the bound is conservative by a little, in exchange
% for staying closed-form and monotone.
%
% Ignoring this is not a rounding error. A 6-drone fleet was flown with the
% speed-capped sweep, which demanded 8.8 m/s^2 against the 3.0 the controller's
% SatX/SatY clamp will pass, and the result was a fleet 4.95 m off plan, still
% moving at 6.16 m/s where the plan said rest, and closing to 0.23 m against a
% d_min of 2.0. The isometry argument below is untouched by that -- the PLAN was
% a perfect rotation throughout -- which is exactly why it had to be flown.
%
% INFO.holdForFullTurns inverts BOTH bounds for T and reports the larger, so the
% caller can say what hold duration the requested turns would really need, and
% INFO.limitedBy says which one is doing the limiting. Expect acceleration to
% win and the answer to be long: a 6-drone Grid at 5 m spacing has rMax = 5.6 m,
% and one full turn needs about 25 s of hold.
%
% The angle profile is minjerkpolytraj on the SCALAR angle, one call per hold
% rather than one per drone -- every drone shares the same s(t), which is also
% what the separation argument for the transitions assumes. Min-jerk matters
% here for a specific reason beyond consistency: it brings the angular RATE to
% zero at both ends, so the fleet enters the hold from a transition that
% finished at rest and leaves it at rest. A constant-rate spin would put a step
% in commanded velocity at every hold boundary, and at the last hold it would
% hand the landing a fleet with tangential speed the descent does not expect.

arguments
    P           (:,3) double
    t0          (1,1) double
    T           (1,1) double {mustBePositive}
    nSamples    (1,1) double {mustBeInteger, mustBePositive}
    turns       (1,1) double = 1
    vTarget     (1,1) double {mustBePositive} = 7
    peakRatio   (1,1) double {mustBePositive} = 35/16
    aTarget     (1,1) double {mustBePositive} = 1.8
    % Peak of |s''| scaled by T^2 for the same 7th-order min-jerk profile peakRatio
    % describes, measured off minjerkpolytraj rather than assumed: 2.187500 and 7.513188
    % for any T. The pair belongs together -- change the profile and both move.
    peakRatio2  (1,1) double {mustBePositive} = 7.513188
end

N = size(P, 1);
centre = mean(P(:, 1:2), 1);
d = P(:, 1:2) - repmat(centre, N, 1);
radii = vecnorm(d, 2, 2);
rMax = max([radii; 0]);           % [;0] so an empty fleet gives 0 rather than -Inf

thetaReq = 2 * pi * turns;

% A fleet with no horizontal extent -- one drone, or every drone stacked on the
% axis -- has nothing to rotate, so the speed cap does not apply and dividing by
% rMax would give Inf or NaN. Report the request as met: it is, trivially.
if rMax > 1e-9
    % Speed bound: linear in theta.
    thetaMaxV = vTarget * T / (peakRatio * rMax);
    holdForV  = peakRatio * abs(thetaReq) * rMax / vTarget;

    % Acceleration bound. With A = peakRatio2/T^2 and B = peakRatio/T standing for the
    % peaks of |s''| and |s'|, requiring rMax*sqrt((theta*A)^2 + (theta*B)^4) <= aTarget
    % is a quadratic in u = theta^2:
    %
    %   rMax^2*B^4*u^2 + rMax^2*A^2*u - aTarget^2 <= 0
    %
    % whose positive root is the bound. Monotone in u, so the root is the whole story.
    A = peakRatio2 / T^2;
    B = peakRatio / T;
    qa = rMax^2 * B^4;
    qb = rMax^2 * A^2;
    thetaMaxA = sqrt((-qb + sqrt(qb^2 + 4 * qa * aTarget^2)) / (2 * qa));

    % Inverting the same expression for T instead: the T^4 falls out cleanly because both
    % A and B carry all of the T dependence.
    holdForA = (rMax^2 * thetaReq^2 * ...
                (peakRatio2^2 + thetaReq^2 * peakRatio^4) / aTarget^2)^0.25;

    if thetaMaxA < thetaMaxV
        limitedBy = 'acceleration';
    else
        limitedBy = 'speed';
    end
    thetaMax = min(thetaMaxV, thetaMaxA);
    holdForFull = max(holdForV, holdForA);
else
    % A fleet with no horizontal extent -- one drone, or every drone stacked on the axis --
    % has nothing to rotate, so neither cap applies and dividing by rMax would give Inf or
    % NaN. Report the request as met: it is, trivially.
    thetaMax = abs(thetaReq);
    holdForFull = 0;
    limitedBy = 'none';
end
thetaTotal = sign(thetaReq) * min(abs(thetaReq), thetaMax);

% Peak commanded tangential speed and total acceleration actually delivered, so a caller can
% report what the plan asks of the fleet without re-deriving the profile. Both use
% the same conservative simultaneous-peak assumption the cap does.
if rMax > 1e-9
    peakSpeed = peakRatio * abs(thetaTotal) * rMax / T;
    peakAccel = rMax * sqrt((abs(thetaTotal) * peakRatio2 / T^2)^2 + ...
                            (abs(thetaTotal) * peakRatio / T)^4);
else
    peakSpeed = 0;
    peakAccel = 0;
end

info = struct('thetaTotal', thetaTotal, 'radiusMax', rMax, ...
    'holdForFullTurns', holdForFull, 'capped', abs(thetaTotal) < abs(thetaReq) - 1e-12, ...
    'rotating', nSamples > 1, 'limitedBy', limitedBy, ...
    'peakSpeed', peakSpeed, 'peakAccel', peakAccel);

% ---- still hold: one anchor, exactly as the plan has always emitted ----------
if nSamples == 1
    pos = reshape(P, N, 3, 1);
    vel = zeros(N, 3, 1);
    times = t0;
    info.thetaTotal = 0;
    info.capped = false;
    info.limitedBy = 'none';       % nothing was asked of the fleet, so nothing limited it
    info.peakSpeed = 0;
    info.peakAccel = 0;
    return
end

% ---- rotating hold ----------------------------------------------------------
% One scalar min-jerk profile, shared by the whole fleet. minjerkpolytraj wants
% waypoints as [dim x npoints], so a 1-by-2 is the scalar 0 -> 1 ramp.
[s, sd] = minjerkpolytraj([0 1], [0 T], nSamples);

times = linspace(t0, t0 + T, nSamples);
pos = zeros(N, 3, nSamples);
vel = zeros(N, 3, nSamples);

for k = 1:nSamples
    th = thetaTotal * s(k);
    w  = thetaTotal * sd(k);

    c = cos(th); sn = sin(th);
    rot = [d(:,1)*c - d(:,2)*sn, d(:,1)*sn + d(:,2)*c];

    pos(:, 1:2, k) = repmat(centre, N, 1) + rot;
    pos(:, 3, k)   = P(:, 3);              % altitude is untouched: horizontal circles

    % d/dt R(th)*d = th_dot * J * (R(th)*d) with J = [0 -1; 1 0], i.e. the
    % rotated offset turned another quarter turn and scaled by the rate. Taking
    % it from `rot` rather than re-deriving keeps position and velocity exactly
    % consistent -- a velocity feedforward that disagrees with the position
    % command is a steady-state error the controller has to fight.
    vel(:, 1:2, k) = w * [-rot(:,2), rot(:,1)];
    vel(:, 3, k)   = 0;
end

% Land the last sample exactly on the requested end state. linspace and the
% polynomial evaluation are both fine to ~1e-12, but the caller uses this final
% sample as the START of the next transition and as the point the landing
% descends from, so any drift here becomes a position discontinuity in the plan.
th = thetaTotal; c = cos(th); sn = sin(th);
rot = [d(:,1)*c - d(:,2)*sn, d(:,1)*sn + d(:,2)*c];
pos(:, 1:2, nSamples) = repmat(centre, N, 1) + rot;
pos(:, 3, nSamples)   = P(:, 3);
vel(:, :, nSamples)   = 0;                 % min-jerk ends at rest, exactly
times(nSamples)       = t0 + T;
end
