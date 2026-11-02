# Stage A (Video Mode): Video -> Motion Data -> System Identification

What can (and cannot) be simulated from video, recording conditions and required products: `video.md`.

Use when the source is a video file, a webcam recording, or an existing
`motion.mat` / `motion_summary.json`. Stages B-F run as usual afterwards.

## The rule: CV measures, you interpret

| Job | Who | Why |
|-----|-----|-----|
| Per-frame positions (joints, objects, masks) | `perceive/` functions (deterministic MATLAB) | Sub-pixel, consistent, repeatable, fast. Your per-frame estimates are none of these. |
| What the system is, which parts matter, joint types, builder, actuation mode | **You** | Needs understanding, not measurement |
| Prompts for SAM (which object, roughly where) | **You**, from a key frame with a pixel grid | Rough points are enough; SAM finds the exact boundary |
| Quality check (swapped limbs, mask jumped, missed bounce) | **You**, from key frames / overlay | |

**NEVER** read the per-frame arrays (`md.entities(k).keypoints`) into the conversation.
Read `motion_summary.json` (`summarizeMotionData`) and key frame PNGs (`selectKeyFrames`).

## A0: Get the video

- **File:** `md = newMotionData(path, 'InTime', t0, 'OutTime', t1)` — trim to the relevant motion.
- **Record:** `app = captureVideo(outDir)` returns immediately (non-blocking UI). The user presses
  Start/Stop; the window then closes and releases the camera (`'CloseAfterTake', false` keeps it
  open for retakes). Do not wait for the user to say "done": watch `status.json` for
  `"state": "done"` (background shell loop), then
  `md = newMotionData(fullfile(outDir,'video.mp4'), 'SourceType','webcam')`.
  Webcam time stamps (`video_timestamps.mat`) are picked up automatically.
- Ask the user to film **side-on** (motion parallel to the image) with the whole subject in view —
  or facing the camera for a 3D skeleton (A3d).

## A1: Look first, then choose the tracker

`md = selectKeyFrames(md, fullfile(outDir,'keyframes'))` (no tracking yet: plain frames with a pixel grid).
Read 3-6 of them, then pick:

| Scene | Tracker | Notes |
|-------|---------|-------|
| **Objects (default): ball, bob, tool, anything you can point at** | `trackObjects(md, prompts)` | Give `.points`/`.box` on the first frame (read from the grid). `'Method','auto'`: NVIDIA GPU -> SAM 2 every frame; otherwise SAM 2 once + RTMDet detector + Kalman filter + colour fallback (~1 s/frame CPU). Class is guessed from the first frame (e.g. `'sports ball'`) or set with `.class`. |
| One compact object, static camera, nothing else moving | `trackForeground(md,'Ids',{...})` | No deep learning, fastest. Fails when a person or anything else moves in the frame. |
| Person (body, arm, leg) | `extractPose(md,'PersonBox',...)` | HRNet, 17 COCO keypoints (`right_elbow`, ...). `'full'` if one person fills the frame (no detector package needed). |
| Robot links (need the link ends) | `trackSAM(md, prompts)` | Mask every frame: returns `center`, `end1`, `end2` (ends along the long axis = link ends). Slow without a GPU (SAM 2 ~6 s/frame, SAM 1 ~14 s/frame). |

Then `md = cleanMotionData(md)` (removes spikes, keeps bounce cusps; fills gaps <= 3 frames).
Then `selectKeyFrames` again — now with overlays — and **check the tracking visually** before going on.

## A2: Scale and plane

| Situation | Function | `scale_source` |
|-----------|----------|----------------|
| Something in free flight (thrown, dropped, bouncing) | `calibrateScaleFromGravity(md, id)` | `gravity` — also measures camera tilt |
| A rigid segment of known length (user tells you, or a known object) | `calibrateScaleFromLength(md, id, kpA, kpB, L)` | `known_length` / `user` |
| Human with known height only | `calibrateScaleFromHeight(md, id, h)` (legs = 0.491 x height; also the floor) | `user height` (a proportion prior) — say so in the report |
| Nothing | leave unscaled | lengths in px; dynamics cannot be identified — tell the user |

**Planarity:** `computePlanarJointAngles` returns `info.length_cv` per link. A rigid link's
image length should be constant; `length_cv > 0.08` means motion out of the image plane
(foreshortening) or tracking error. Then: ask for a side-on re-recording, or state that joint
angles are approximate. Record in the spec which quantities were **measured** and which were
**inferred** (priors, assumed masses).

## A3: Identify the system -> builder and mode

| What the video shows | Identification | Builder |
|----------------------|----------------|---------|
| Bouncing / thrown object | `cleanMotionData(md,'MaxGap',0)` (no gap filling across bounces), `calibrateScaleFromGravity`, `analyzeBallistic` -> restitution per bounce, initial state | `odeBuilder_cps` (flight + impact reset) |
| Object rolling/sliding along a visible path | path from mask, position along it | `odeBuilder` (1-DOF along the path) |
| Articulated body (limb, robot arm), joints driven | `computePlanarJointAngles` -> angles, link lengths | `multibodyBuilder`, `actuation='motion'` (inverse dynamics: torques) |
| Pendulum / swinging object | `analyzePendulum` -> pivot, angle, period -> length (and scale) from gravity, damping from the decay | `multibodyBuilder`, `actuation='passive'`, link + `tip_mass` (bob), damping / `q0` / `w0` as variable names, `fitParameters` (damping + start state; length too unless measured) |

Masses/inertias are not visible. Use priors (body-segment tables, robot datasheets) and mark them
as assumed, or fit them with `fitParameters` when the motion is passive (dynamics visible).

## A3b: Body + object scenes (a person and a thing interact)

Throw, catch, kick, bat, carry, drop, an object hitting the scene. Do not write Stateflow or
contact maths: describe the scene and let the functions write the model.

**1. Track both into one `md`:** `extractPose` + `trackObjects` (objects first-frame prompt, as usual).
Where one tracker loses the object (behind a net, a rim, a hand) run another on just those frames
(`trimMotionData(..., 'Rebase', false)` -> `trackSAM` seeded at the last good frame) and
`mergeObjectTracks`. Behind a mesh (a net) a propagating mask spreads into the mesh after a
frame or two: then read one point on the object per frame off the key frames and use
`segmentAtFrames(md, id, frames, points)` (one SAM pass per frame, oversized masks rejected).
The same with `'Replace', true` fixes frames where a detector locked onto something of the same
colour (a red rim next to a red ball).

**2. Events:** `[ev, ph] = detectInteractions(md, objId, personId)` labels phases (`held` by a body
point, `rest`, `flight` = gravity parabola, `other`) and events (`release`, `catch`, `strike` by a
body segment, `contact` with the scene with impulse normal and restitution, `lost` / `reappear`).
Read the event list, not the frames. Gaps (`lost` ... `reappear`) are yours to interpret from key
frames (behind the net, out of view). A flight phase's `acc_px` gives the scale:
`m_per_px = g / acc_px(2)` (or `calibrateScaleFromGravity` on a trimmed copy).

**3. Choose the coupling** (the key decision):

| Object vs body | Coupling | How |
|----------------|----------|-----|
| Light (ball, stone, small tool) | body drives object, one way | body: `multibodyBuilder` motion-driven, `tip_velocity` true; object: `objectInteractionSpec` -> `odeBuilder_cps`; `plan.wiring = info.wiring(bodyComp)` |
| Heavy, carried (box, bag, dumbbell) | object loads the body | `multibodyBuilder` `.payloads` (`held_<name>` 0/1 from the held phases, `.mass` known or a variable to fit from torques); no separate object model while held |
| Body on the object (swing, bike, board, chair) | shared motion | floating base (not yet) — tell the user |
| Rigid tool in the hand (racket, bat, stick) | part of the body | extend the chain: `computePlanarJointAngles(md, id, {..., 'right_wrist', 'tool:tip'})` and add a link |

**4. The body chain may end on the object:** while held, the object's centre is the end of a
"hand" link (`{'right_elbow','right_wrist','ball:center'}`); hold that angle after the release.

**5. The scene:** read the scene off a key frame where the object does not hide it, as pixel
shapes, and convert: `scene = sceneFromPixels(md, shapes)` -> `.obstacles`, `.regions` (for the
object) and `.scenery` (for `multibodyBuilder`, Mechanics Explorer). Shapes: `line` (floor, wall),
`segment` (edge, board), `circle` (post seen end-on), `ring` (rim seen side-on -> two lips),
`box`, `net` (drag region + cone), `region`. Moving body segments (a kicking shank, a bat) are
obstacles of type `body` in `objectInteractionSpec` (end points = two body points with tip outputs).

**5b. Resolve every gap and break next to the scene — from the frames, not from a model.**
`detectInteractions(md, obj, person, 'Scene', scene)` lists them (`info.to_resolve`) and prints
each: e.g. "gap 1.43-1.57 s next to rim, board, pole, net". For each one look at those frames and
decide: it **hit** X (make X a contact obstacle; recover the frames with `segmentAtFrames` if
needed), it **passed behind / in front of** X (no contact: one camera flattens depth, so overlap is
not contact), or it went **through** X (a region). Do not make every nearby solid a contact by
default, and do not leave a listed object unresolved. (The throw test: the ball bounced off the
basket's pole while it was lost below the net; modelled as "net + pull towards its centre"
instead, the fit looked fine and was wrong.)

**6. The object:** `[spec, info] = objectInteractionSpec(obj)` with `.holders`, `.start`,
`.events` (release / catch times from step 2), `.obstacles`, `.regions`. Add `info.marker` to
the body spec's `.markers` so Mechanics Explorer shows the object, and `.scenery` from step 5.

**7. Fit in stages, each checked on data it did not see** (`info.fit` lists the names):

| Stage | Fit | On | Check on |
|-------|-----|----|----------|
| 1 | release gains `info.fit.release` (a rigid hand tip is slower than the real release) | the free flight right after the release | the rest — with contacts / regions at priors: does it hit and go where the video shows? |
| 2 | contacts `info.fit.contacts.<group>` (e, f), regions `info.fit.regions.<name>` | the samples after the first contact | — (identification; say so) |

Report stage 1's prediction separately from stage 2's fit.

**Every fitted term needs a visible cause and a check on data it did not see.** Before adding a
parameter (a drag, a funnel, a correction gain), name what in the frames causes it; if nothing
visible does, look at the frames again (5b) — a missing contact is the usual reason. Then check
the fitted term on samples it was not fitted on (a later phase, a held-out part, another event).
A term that only fits the samples it was fitted on is not evidence. Store fitted values with the
model: `storeModelData(mdl, fitted, 'Tag', 'fit')` (executePlan already stores the measured
inputs), so the model runs with Play later.

**Body velocities are derivatives of noisy angles — smooth them.** A release or a strike
hands the body's velocity to the object. Unsmoothed, 1.5 px keypoint noise on a 0.45 m limb
gives ~55 % angular-velocity error at 60 fps (synthetic kick). Use `motionToInputs(...,
'Smooth', w)` with `w` ≈ 0.15 s of samples (5 at 30 fps, 9 at 60 fps) and `motion_filter_tc`
0.01: rms error drops to ~10 % with no visible peak loss — **as long as the window is shorter
than about a third of the fastest motion**. A real underhand throw at 30 fps ends in a ~0.1 s
wrist flick: 5 samples erase it (release speed 1/3 low); there use no smoothing and let the
fitted release correction absorb the noise. Torques (second derivatives, carrying a load) need
more: `tc` 0.05, `w` ≈ 0.3 s. Even then:
- the **release gain** k and angle a absorb what is left (and fingertip roll): report them as
  corrections, not as physics;
- a **strike** only identifies (1 + e) x (limb speed at contact) — report e as effective and
  validate on the launch velocity / trajectory, not on e itself.

Recording / scene pitfalls: the scale fit can report a camera roll that the scene's verticals
(door edges, poles) contradict — that is perspective, set `down_px = [0 1]` and use the vertical
acceleration. A kick off the floor with a straight shank skims (the impulse is horizontal);
strikes that lift the object need the contact in front of the joint. Planar only: the object
must move in (about) the body's plane.

## A3c: Whole body (floating base): squat, jump, lift, arm raise — side-on or from the front

The body is not attached to anything: model it with a **floating base** and read the ground
reaction off it. Side-on: motion in the sagittal plane (squat, jump, lift, forward arm raise).
From the front: motion in the frontal plane (sideways arm raise, lateral lean, side step).

1. `extractPose`; `cleanMotionData`; scale: **ask the user for the height** and use
   `[md, hinfo] = calibrateScaleFromHeight(md, 'person1', h)` (legs = 0.491 h, independent of the
   knee bend; cross-check stance nose-ankle = 0.871 h in `hinfo.agreement_pct`; it also sets the
   world origin on the floor: world y = height above the floor, `FloorY` = 0). A known length works
   too (`calibrateScaleFromLength`).
2. `[spec, in, info] = bodyTreeFromPose(md, 'person1', 'Mass', m)` — **ask the user for the body
   mass** (not visible). Side-on: pelvis frame at the hips, trunk, head, legs (`'Legs'`, default
   both), arm (`'Arms'`, default the near side, mass doubled: side-on the far arm is mostly hidden).
   From the front: `'View', 'frontal'` — hips and shoulders at their own keypoints (`.offset`), both
   legs and arms. Masses: de Leva fractions (state it: a prior). The start pose (shown before the
   run) is the first measured one.
3. `spec.name = ...; spec.motion_filter_tc = 0.05;` -> `multibodyBuilder` (motion);
   `plan.topInputs = motionToInputs(in.t, in.Q, comp, in.ports, 'LeadTime', 2*tc, 'Lowpass', fc)`:
   **fc = 2 Hz for slow motions** (squat, arm raise, lift), **5 Hz for fast ones** (jump, throw,
   impacts). Measured 2026-10-28: synthetic squats vs Newton-Euler truth, vertical force 1.6 % bw rms
   at 2 Hz vs 3.4 % with `'Smooth'` 9; real arm raise noise halved; jump, airborne force 0.04 bw at
   5 Hz (3 Hz cut the push-off peak by 5 %). Keep tc 0.05: with the low-pass setting the band, a
   smaller tc only adds noise (jump, tc 0.02: airborne 0.09 bw, peaks +20 %). Peak forces of a jump
   depend on the filter (2.15-2.65 bw on the real clip) — report them as filter-dependent.
   `topOutputs` base_fx, base_fy, base_tz, tipx_/tipy_shank_r/l (for step 5).
4. `grf = groundReaction(simOut, t, 'Base', in.Q(:,1:2), 'FloorY', 0, 'Mass', m)` — ground reaction
   force (N, and in body weights) and centre of pressure. Notes from the real clips: it is dominated
   by the trunk and pelvis — link-length errors barely change it (`bodyTreeFromPose 'Height', h`
   gives proportional lengths; HRNet's hip point sits high).
5. **Joint torques need a second pass.** Driven this way, the base carries the whole external
   load: the ground reaction is right but the legs hang from the pelvis — their joint torques are
   wrong (synthetic squats: knee 67 % off). Apply the ground reaction at the feet:
   `L = footLoads(grf, [tipx_shank_r tipy_shank_r], [tipx_shank_l tipy_shank_l], 'FloorY', 0)`,
   `spec.loads = struct('name', {'foot_r','foot_l'}, 'body', {'shank_r','shank_l'})`, add
   `motionToInputs(t, L.Q, comp, L.ports, 'Lowpass', fc)` to the inputs, rebuild, simulate. Then
   knee torque 3.7 % of peak rms vs truth, and base_fx/fy ~0 (check it: < 2 % bw). The split
   between the feet uses the centre of pressure (lever rule) when the feet are apart in the plane
   (front view) — an estimate: no force plate.
6. Jumps: `[fl, finfo] = detectFlight(md, 'person1')` — take-off / landing (ankles off their
   standing level, shifted one frame for the toes: matched the real clip read off the frames), the
   flight core (judge physics there: fy ≈ 0) and a gravity scale from the hips' parabola **with its
   95 % interval** (`finfo.ci95_pct`). A 0.4 s flight has ~9 core samples: ±20 % on the real clip —
   a check of the height scale, not a replacement for it.
7. Validate without a force plate: standing still -> fy = m g; airborne -> fy ≈ 0 (core); CoP under
   the feet (side-on: between heel and toe; front: between the feet) whenever they are on the
   ground; shoulder torque of a slow arm raise = the arm's gravity moment (m g x, from the
   keypoints). These are physics checks, not fits — report them. The ground reaction only holds
   while nothing else touches the body (a hand on a rail, a chair): resolve that from the frames
   (A3b 5b).

## A3d: 3D skeleton from one camera (person facing the camera)

When the motion is toward / away from the camera (a squat or forward arm raise filmed from the
front), a planar model cannot be built. `[pose, info] = liftPose3D(md, 'person1', 'Height', h)`
fits a 3D skeleton (bone lengths from the height) to the 2D keypoints through a pinhole camera:
foreshortening gives each bone's tilt, anatomy (hinges, limits) and continuity its sign, the body's
apparent size its distance. `'FeetPlanted', true` and `'Balance', 1` for motions in place (squats,
arm raises, lifting); not for jumps or walking. **Ask the user for the height.**

- Recording: whole body in view for the entire motion, raised hands included — about 3-4 m from
  the camera; stand still first, facing the camera, arms hanging at the sides (the first frames
  are taken as an upright stance and the segments are measured there).
- Segment lengths and widths come from those first frames (`'Segments', 'image'`), the height only
  sets the distance. HRNet's points are not the joint centres of the body proportions (hips 0.22 m
  apart instead of 0.32, arms 15 % shorter on the real clip); with proportion lengths the fit
  turned the body 56 deg and tilted hanging arms 30 deg toward the camera to make them look
  shorter. Check `info.segments_from_image`.
- Keypoints on the image border have left the frame: they are used as "outside the image"
  (`'EdgeMargin'`), and a limb whose end is unseen holds its direction. Its pose is then a guess
  — say so (real clip at 1.9 m: hands over the head out of view, one arm placed 30 cm behind the
  body before this handling).
- Feet that shift during the clip break `'FeetPlanted'`: split the clip at the shift.
- **Check the start frame.** `liftPose3D` starts at frame 1 if the person stands nearly still
  there, else at the first still, upright 0.3 s (`info.start_frame`); it fits forwards from it and
  backwards to frame 1. Look at that frame and the ankles after it: it must be the reference stance
  (upright, facing, arms hanging) **after** any step into place — otherwise set `'StartFrame'`. Real
  clip: a still stance at 0.5 s, then a step forward; anchored there, the feet-planted term fought
  the step and the arm raises flipped. Fitting backwards through a squat is unreliable (bent knees
  stayed bent): start on the stance before the motion, not between repetitions.
- What to trust: large flexions (knees, hips, forward arm raises) and depth changes of the pelvis
  and knees; not small out-of-plane angles (< ~15 deg) or the absolute distance (focal length from
  the camera's field of view, ~10 %). Report the result as an estimate. A 3D Simscape model of it
  is not available yet (planar `bodyTreeFromPose` needs a side-on view).
- Plot hip flexion as trunk vs thigh (hip -> knee), not pelvis -> knee (the hip's lateral offset
  adds ~21 deg to a straight leg).
- Runtime: ~1-2 s per frame on CPU; trim to the motion first.

## Stage B notes for video-derived plans

- **multibodyBuilder spec:** `bodies(i)` with `.name .parent .length .width .mass .joint`; `base_position`
  from `info.base_xy` (median); `actuation`. Joint angles are relative (link i from link i-1).
- **Driving with measured angles:** `topInputs = motionToInputs(t, Q, comp, {'q_<joint>',...},
  'LeadTime', 2*tc, 'Smooth', 9)` with `spec.motion_filter_tc = tc`. For 30 fps video use
  `tc = 0.05` (sweep on synthetic arm: torque peak error 6% vs 17% at tc = 0.03).
- **odeBuilder_cps parameters go to the base workspace** — use distinctive names (`e_b`, `g_b`).

## Stage E for video sources

1. `fitParameters(mdl, params, data)` on the **first part** of the clip only.
2. Validate on the **held-out** rest (prediction, not replay): bounce times, apex heights,
   tip/keypoint error in px. Score with `evaluateTests`.
3. `renderOverlayVideo(md, sims, 'overlay.mp4')` — simulated motion drawn on the real video;
   include it in the report.
4. Report: scale source, camera tilt, planarity (`length_cv`), measured vs inferred parameters,
   tracker used, support packages used.

## Optional: processing videos (demos, explaining, debugging)

Only when the user asks to *see* the steps — nothing else depends on them.

| Function | Shows |
|----------|-------|
| `trackSAM(..., 'MaskDir', dir)` / `trackForeground(..., 'MaskDir', dir)` | (opt-in) saves per-frame masks so the videos can draw them |
| `renderTrackingVideo(md, file, 'Raw', true)` | stage 1: source video with time stamps |
| `renderTrackingVideo(md, file)` | boxes, masks, skeleton, keypoints (filled = measured, hollow = gap-filled), trails |
| `renderTrackingVideo(md, file, 'Keypoints', chain, 'Chains', {chain})` | joint-angle arcs with values on the limb + strip chart of the angles (same convention as the model) |
| `renderOverlayVideo(md, sims, file)` | simulation drawn on the video |
| `renderPipelineVideo(stages, file, 'Layout', 'grid'|'sequence')` | all stages side by side in sync, or one after another with title cards |

Use `trimMotionData(md, t0, t1, 'Keypoints', chain)` so every stage covers the same frames as the model.

## Pitfalls (found while building this)

| Pitfall | Effect | Handled by |
|---------|--------|-----------|
| Moving-median outlier filters | delete real bounce cusps | `cleanMotionData` spike test (vs median frame step) |
| `smoothdata(...,'SamplePoints',t)` | window is in **seconds**, not samples | converted in `cleanMotionData` / `motionToInputs` |
| VideoWriter MP4s start at t = 1/fps; seeking reports requested time | one-frame time shift | `MotionVideoReader` |
| Motion blur | measured position ~ mid-exposure (few ms late) | accepted; visible as tiny constant time offset |
| Fit segment spanning an unresolved bounce | wrong scale/restitution | rejected by parabola residual / contact-height checks |
| Physical lines at a model's root | xToSim layout pass deletes them | `multibodyBuilder` builds the mechanism inside a subsystem |
| Stale `<name>.slx` in the output folder | builder renames model to `<name>_mdl` | clean the output folder before rebuilding |
| Objects at the image border | centre of a cut-off box is biased | `cleanMotionData` drops object samples whose box touches the border |
| Gap filling across a bounce | draws a smooth curve through the impact | `MaxGap, 0` for ballistic analysis |
| One motion-blurred flight arc | wrong gravity scale | `calibrateScaleFromGravity` drops arcs > 25 % from the median |
| Joint units | Multibody joints default to deg, N*m/(deg/s) | `multibodyBuilder` sets rad units explicitly |
| Opening videos with the Windows player | Windows may ask for confirmation; MATLAB waits | `playVideo` plays inside MATLAB (`'Play', true`) |
| Model names reused across runs | a stale model of the same name is simulated | close/reload by full path; unique names per run (follow-up) |
| Hand-held pivot | pivot moves with the swing; circle-fit pivot too low | fix the pivot (hook) for clean pendulum tests |
| Object collides (rim, foot) | the tracker's motion prediction is wrong afterwards; object never found again | `trackObjects` `'Reacquire'` (detection near the last seen position) |
| Red ball next to a red rim, ball merged with the hand | tracker jumps to the wrong blob | `trackObjects` size check per side, `'MaxJump'` |
| Body drives object, object drawn in the body model | algebraic loop through one Simscape network | `multibodyBuilder` markers use their own network |
| Payload force from sensed acceleration | Simulink reports an algebraic loop *inside* the network | solved by Simscape (results exact); warning only |

## Support packages

| Feature | Support package |
|---------|-----------------|
| `extractPose` | Computer Vision Toolbox Model for Object Keypoint Detection (HRNet) |
| `extractPose('PersonBox','detect')` | Computer Vision Toolbox Model for RTMDet Object Detection |
| `trackSAM` | Image Processing Toolbox Model for Segment Anything Model 2 (preferred; MathWorks-bundled Python/PyTorch) or ... Segment Anything Model (SAM 1, fallback) |
| `captureVideo` | Image Acquisition Toolbox Support Package for OS Generic Video Interface |

`trackForeground`, calibration, identification, `multibodyBuilder` and the overlay need none of them.

## Tests

`tests/videoToSimTests/` — `runVideoToSimTests` (list and what each checks: its `README.md`).
8 synthetic tests with exact truth (ball, arm, pendulum, throw + bounces, kick, carry, whole body
with leg torques, 3D lift from a front camera) and 6 replays of real webcam clips (ball, pendulum,
arm curl, throw into a basket, squat, jump). Do not read the tests while solving a task: they are
the check, not a recipe.
