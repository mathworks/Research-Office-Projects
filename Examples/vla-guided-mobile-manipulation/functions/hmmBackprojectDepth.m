function [pWorld, depth, status] = hmmBackprojectDepth(frame, pixel, cfg)
%HMMBACKPROJECTDEPTH One pixel of a CameraFrame's render depth to a world point.
%
%   [pWorld, depth, status] = HMMBACKPROJECTDEPTH(frame, pixel, cfg) reads the Depth image of
%   a CameraFrame bus at pixel = [row; col] and returns the world point that pixel sees.
%   depth is the metre reading actually used and status is one of cfg.refine.STATUS_*;
%   pWorld is [0;0;0] on any status other than APPLIED. pixel may be fractional - a provider
%   returns a normalised point, not an integer - and is rounded and clamped to the image.
%
%   THIS FUNCTION EXISTS BECAUSE A DOCUMENTED CONSTANT WAS WRONG BY FOUR ORDERS OF MAGNITUDE.
%   cfg.depth.renderQuantum read 1.0 m and was labelled "documented", and on the strength of
%   it four separate places in this project record that render depth is "useless for a 9 cm
%   object" and the wrist camera cannot be used for grasping. It was measured instead, by
%   differencing adjacent distinct values over all 230 400 pixels of all five cameras: the
%   quantum is 1000/2^24 = 59.6 um, a 24-bit encoding of a 1000 m far plane. The "1 m" was
%   the far plane misread as the step size. A 9 cm mug spans 1510 quanta. Render depth was
%   never the limitation.
%
%   THE CONVENTION, AND WHAT EACH PART OF IT IS CHECKED AGAINST. Every line below is a
%   measurement, not a datasheet reading, because the block documents none of it:
%
%     depth form   PLANAR - the distance along the optical axis, not the range along the
%                  pixel's own ray. Both interpretations were computed against the ray
%                  tracer's HitDistances; planar wins outright.
%     sentinel     Depth >= cfg.depth.renderSentinel means the pixel hit NOTHING. It is 0 %
%                  of pixels on a room camera, which always sees a wall, and 21.4 % on the
%                  wrist, which looks past the object into open space. Averaging a sentinel
%                  in silently places the point a kilometre away, so it is rejected first.
%     sensor axes  +X along the optical axis, +Y image RIGHT, +Z image UP, so image-DOWN is
%                  -Z. A roll-free camera is BLIND to this sign, because a left-right mirror
%                  preserves height; it is the wrist's 172 deg roll that decides it, and it
%                  decides it for image RIGHT.
%     pose         world = Rz(yaw)*Ry(-pitch)*Rx(-roll) * p_sensor + Translation. Unreal's
%                  pitch and roll are left-handed, hence the two negations.
%
%   THREE INDEPENDENT TESTS AGREE ON THAT POSE, which is why it is asserted rather than hoped.
%   It matters that there are three, because each is blind to something:
%
%     1. CLOSURE AGAINST A DIFFERENT SENSOR. Back-project render depth at the pixel each ray
%        of the Simulation 3D Ray Tracer looks at and compare against Calib.HitLocations, the
%        ray caster's own world points. A rasterised depth buffer and a ray caster agreeing on
%        a 3-D point cannot happen by accident. Measured on all four fixed cameras: median
%        error 0.0096-0.0120 m, 74-95 % within 20 mm, against 2.4-4.2 m for the wrong Euler
%        convention and 1.1-3.2 m for the wrong yaw sign. The ~10 mm residual is the ray
%        grid's sub-pixel sampling at a 10 px pitch. BLIND TO: roll, which is 0 on all four.
%     2. THE OPTICAL AXIS vs THE ROOM. hmmConfig aims each fixed camera at the centre of the
%        room it is named after, so R(:,1) must come out along that bearing. It does, to
%        0.00 deg, for all four; the opposite yaw sign misses by 22-84 deg. BLIND TO: roll.
%     3. THE WALLS. The shell is 10 x 8 m and a camera standing near the middle of it cannot
%        see through it. Under this pose 100.00 % of the 411 423 pixels that hit a surface -
%        across a roll-free camera AND the 172 deg-roll wrist - land inside the footprint, and
%        0.00 % land outside. The opposite yaw sign puts 3.7 % and 6.1 % through solid walls.
%        This is the test that covers roll, and the only one that does.
%
%   THE WRIST NUMBERS ABOVE DESCRIBE THE OLD MOUNT, and are left as recorded rather than
%   rewritten, because they are what the tests actually ran against. cfg.camera.wrist.tilt was
%   -0.35 rad then, which aimed the camera 69.95 deg away from the descent - it was looking past
%   the object at the far wall, which is where the "21.4 % sentinel" and the "172 deg roll" both
%   come from. The tilt was then corrected to -1.8154 rad, and is now -1.3262: the camera moved
%   to the FRONT face of the coupling (cfg.camera.wrist.offset(1) = -0.045) and the tilt was
%   mirrored to match, so it still aims at the grasp point rather than along the descent.
%   MEASURED ON THE FRONT MOUNT, from one hmmWristRefineTap record: the sentinel fraction is
%   0.0 % - nothing in frame is further than the far plane, because the tabletop fills it - the
%   range to the mug is 0.4134 m, and Rotation reads [3.1413 -1.3262 -0.2870], so the roll is
%   179.98 deg rather than 172. The rolled-camera tie-break in test 3 still holds, and holds
%   harder: a camera rolled essentially pi is maximally sensitive to the Y sign.
%   NONE OF THE CONCLUSIONS MOVE. The sensor convention, the pose composition and the +Y-is-
%   image-RIGHT sign are properties of the Simulation 3D Camera block, not of where it is
%   bolted; test 3 needed SOME rolled camera to break the tie and the wrist happened to be one.
%   What changes is only the illustration. See the block comment at cfg.camera.wrist.tilt.
%
%   AND A DEFECT TEST 1 TURNED UP ON THE WAY, WHICH IS NOW FIXED ELSEWHERE. Closure used to
%   hold only at the COLUMN-MIRRORED pixel, (row, nCols+1-col), not at the pixel
%   functions/hmmRayDirections labelled. Read alone that was ambiguous - "+Y is right and the
%   ray label is mirrored" and "+Y is left and the label is correct" predict the same closure.
%   Test 3 broke the tie, because the Y sign tilts a 172 deg-rolled camera's reconstruction out
%   through the walls and image RIGHT is the sign that keeps it inside. So hmmRayDirections'
%   "+Y is image LEFT", and the [1, -xn, -yn] it built from it, were wrong: ray i carried the
%   wrong pixel label, and hmmReconstructTargets associated a grounded pixel with rays from the
%   mirrored side of the frame.
%
%   hmmRayDirections and the projection now both carry +xn, so closure holds at the labelled
%   pixel and this paragraph is history rather than a warning. It is kept because it records
%   which measurement settled the sign, and because the fix had to be made in TWO places at
%   once: the projection was mirrored to match the ray grid, so the pair was self-consistent
%   and no test between them could see it. This function was the arbiter precisely because it
%   never touches the ray path - it reads Depth directly and needs no ray registration at all,
%   which is also why the wrist refinement was unaffected by the bug it exposed.
%
%   WHY A PATCH MEDIAN AND NOT THE PIXEL. Single-pixel depth jumps metres at a silhouette: a
%   pixel on the boundary of the mug reports either the mug or the wall behind it, never
%   anything between. Measured on the wrist frame, radius 0, 2 and 5 px all return the SAME
%   median depth to four decimal places while the spread INSIDE the patch grows
%   0.0000 -> 0.0185 -> 0.0461 m. The median is stable over the choice of radius, so the
%   radius costs nothing and buys a vote against a stray pixel.
%
%   See also hmmWristRefine, hmmRayDirections, hmmReconstructTargets.

%   Copyright 2026 The MathWorks, Inc.

pWorld = zeros(3, 1);
depth  = 0;

if ~frame.Valid
    status = cfg.refine.STATUS_NOFRAME;
    return
end

D    = double(frame.Depth);
nRow = size(D, 1);
nCol = size(D, 2);

% Round and clamp. A normalised provider point maps to a fractional pixel, and a point at the
% very edge of the frame must not index outside the image.
r0 = min(max(round(double(pixel(1))), 1), nRow);
c0 = min(max(round(double(pixel(2))), 1), nCol);

% The patch, clipped at the frame border rather than mirrored: a border pixel has fewer
% neighbours and that is all it means.
rad = double(cfg.refine.patchRadius);
rLo = max(1, r0 - rad);
rHi = min(nRow, r0 + rad);
cLo = max(1, c0 - rad);
cHi = min(nCol, c0 + rad);
patch = D(rLo:rHi, cLo:cHi);

% SENTINELS OUT BEFORE THE MEDIAN, NOT AFTER. A patch that is half sky would otherwise
% return a median of ~500 m, which passes no range test but for the wrong reason.
hits = patch(patch < cfg.depth.renderSentinel);
if isempty(hits)
    status = cfg.refine.STATUS_NODEPTH;
    return
end
depth = median(hits);

if depth < cfg.refine.minRange || depth > cfg.refine.maxRange
    status = cfg.refine.STATUS_RANGE;
    return
end

% The pinhole, in the sensor frame, with planar depth. No trigonometry: planar depth IS the
% +X component, so the other two are the normalised image offsets scaled by it.
K  = double(frame.Intrinsics);
fx = K(1);
fy = K(2);
cx = K(3);
cy = K(4);
pSensor = depth * [1; (c0 - cx) / fx; -(r0 - cy) / fy];

% And into the world. rot is [roll pitch yaw] in Sim3D's left-handed pitch/roll.
rot = double(frame.Rotation);
R   = eul2rotm([rot(3), -rot(2), -rot(1)], 'ZYX');
pWorld = R * pSensor + reshape(double(frame.Translation), 3, 1);

status = cfg.refine.STATUS_APPLIED;

end
