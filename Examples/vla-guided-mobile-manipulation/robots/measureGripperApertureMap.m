function map = measureGripperApertureMap(robot, cpl, cfg)
%MEASUREGRIPPERAPERTUREMAP Measured aperture-to-driver-angle map for the 2F-85.
%
%   map = MEASUREGRIPPERAPERTUREMAP(robot, cpl, cfg) sweeps the 2F-85
%   driver joint over its range with the measured coupling applied, measures
%   the resulting jaw aperture from the assembled geometry, and returns a
%   monotonic breakpoint/table pair for a 1-D Lookup Table block.
%
%   The whole point is that the model never contains a closed-form linkage
%   formula. GripperControl commands an aperture in metres; this measured
%   map is the only thing that knows how many radians of driver angle that
%   aperture costs. The design requires the map to be measured, monotonic,
%   and to span the published stroke.
%
%   Pad frames sit a small distance behind the pad faces, so the measured
%   frame separation is offset to a true face-to-face aperture. The offset
%   is calibrated so that a fully open gripper reads exactly the published
%   stroke (cfg.gripper.maxOpenPos); the closed end is then CHECKED, not
%   fitted, and must land near zero for the calibration to be credible.
%
%   Returned fields:
%       .apertureBreakpoints  1xN ascending aperture values, m
%       .driverTable          1xN driver angle for each aperture, rad
%       .driverBreakpoints    1xN ascending driver angles, rad
%       .apertureTable        1xN aperture for each driver angle, m
%       .padFrameOffset       m, frame separation minus true face aperture
%       .apertureAtOpen       m, measured aperture at the open end
%       .apertureAtClosed     m, measured aperture at the closed end
%       .maxNonMonotonicity   m, worst forward step (must be negative)
%
%   The two directions are both returned because the model needs both: the
%   forward map (driver -> aperture) reports achieved aperture back to the
%   supervisors, and the inverse map (aperture -> driver) is what the
%   commanded aperture is converted through.
%
%   See also measureGripperCoupling, assembleMobileManipulator.

%   Copyright 2026 The MathWorks, Inc.

arguments
    robot rigidBodyTree
    cpl struct
    cfg struct
end

n   = cfg.gripper.mapBreakpoints;
drv = linspace(cfg.gripper.driverRange(1), cfg.gripper.driverRange(2), n);
q0  = homeConfiguration(robot);
padL = cpl.padBodies{1};
padR = cpl.padBodies{2};

sep = zeros(1, n);
for j = 1:n
    q = q0;
    q(cpl.indices) = cpl.multipliers(:)' * drv(j);
    T = getTransform(robot, q, padR, padL);
    sep(j) = norm(T(1:3,4));
end

%% Calibrate the pad-frame offset against the published stroke
map.padFrameOffset = sep(1) - cfg.gripper.maxOpenPos;
aperture = sep - map.padFrameOffset;

map.apertureAtOpen   = aperture(1);
map.apertureAtClosed = aperture(end);

%% Validate before anything downstream can rely on the map
map.maxNonMonotonicity = max(diff(aperture));
assert(map.maxNonMonotonicity < -1e-6, ...
    'measureGripperApertureMap:notMonotonic', ...
    ['Measured aperture is not strictly decreasing in driver angle ' ...
     '(worst forward step %+0.3e m). The coupling multipliers ' ...
     '[%s] cannot be inverted.'], map.maxNonMonotonicity, ...
    num2str(cpl.multipliers(:)'));

assert(abs(map.apertureAtOpen - cfg.gripper.maxOpenPos) < 1e-9, ...
    'measureGripperApertureMap:openEnd', ...
    'Open-end calibration failed: %.6f m against %.6f m.', ...
    map.apertureAtOpen, cfg.gripper.maxOpenPos);

% The closed end is a genuine check on the calibration: nothing above was
% fitted to it, so a large residual means the pad-frame offset assumption or
% the published stroke is wrong.
closedTol = 0.002;   % m
assert(abs(map.apertureAtClosed - cfg.gripper.minOpenPos) < closedTol, ...
    'measureGripperApertureMap:closedEnd', ...
    ['Closed-end aperture is %.6f m, more than %.4f m from the expected ' ...
     '%.6f m. The pad-frame offset calibrated at the open end does not ' ...
     'hold at the closed end, so the measured stroke disagrees with the ' ...
     'published stroke.'], map.apertureAtClosed, closedTol, ...
    cfg.gripper.minOpenPos);

%% Forward map: driver angle -> aperture (already ascending in driver)
map.driverBreakpoints = drv;
map.apertureTable     = aperture;

%% Inverse map: aperture -> driver angle. Lookup Table breakpoints must
% ascend, and aperture decreases with driver angle, so reverse both.
map.apertureBreakpoints = fliplr(aperture);
map.driverTable         = fliplr(drv);

end
