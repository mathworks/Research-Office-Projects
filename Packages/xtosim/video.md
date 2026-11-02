# Video Sources — What Works

Video mode turns a video file or a webcam recording into a validated model. How the agent runs it:
`phases/stage_A_video.md`. Overview: `README.md` → *Video sources*.

## What you can simulate

| Video of… | Result | Status |
|-----------|--------|--------|
| **Bouncing ball** (or a dropped / thrown object) | Scale and camera tilt from gravity, restitution per bounce, hybrid model (`odeBuilder_cps`) fitted on part of the clip that predicts the later bounces | Tested (`tests/videoToSimTests/test_video_ball_e2e.m`): scale within 0.2 %, restitution 0.751 vs 0.75, predicted heights within ~1 px |
| **A person's arm** (shoulder–elbow–wrist) | Joint angles from HRNet, Simscape Multibody arm (`multibodyBuilder`) driven by the measured angles, joint torques, Mechanics Explorer | Tested (`tests/videoToSimTests/test_video_arm_e2e.m`): RMS torques within 2.4 % of the analytic solution; runs on real webcam clips |
| **One moving object**, fixed camera | Position track, masks, trail | Tested on a real clip |
| **Body + object, in general** (throw, catch, kick / bat, carry, drop; the object hitting floor, walls, edges, a rim, a net) | `detectInteractions` finds held / rest / flight phases and release, catch, strike and contact events; the agent reads the scene off a key frame (`sceneFromPixels`), picks the coupling (light object driven by the body, heavy one as a `payload`), and `objectInteractionSpec` writes the object's hybrid model (contacts with static shapes and moving body segments, drag regions) — no hand-written Stateflow | Tested on synthetic data with exact physics: throw + wall + floor bounces (restitutions within 1–2 %, held-out bounces 1.8 px), kick off a shelf (launch speed 3 %, held-out 2.2 px), carrying a 2 kg box (joint torques within 2–3 % of analytic). See `phases/stage_A_video.md` A3b |
| **A person throwing a ball** into a basket (real clip) | The generic recipe above: Multibody arm whose hand drives the ball (held → flight → rim contact → net → bounce off the pole), basket shown in Mechanics Explorer | Tested on a real clip (`tests/videoToSimTests/replay_arm_throw.m`): release direction from the arm within 3 deg; with the release corrected on the flight, the ball hits the front of the rim, goes through and bounces off the pole as in the video, already with rim / net / pole at prior values (after the rim 4 px fitted, 9 px on held-out frames). Planar; the body's velocity at release is a noisy derivative, corrected on the flight |
| **Whole body, side-on** (squat, jump, lift, forward arm raise) | Floating-base Multibody body (`bodyTreeFromPose`): ground reaction force and centre of pressure; with the ground reaction at the feet (`footLoads`, second pass) the hip / knee joint torques; take-off / landing (`detectFlight`) | Tested: synthetic squats vs Newton-Euler — vertical force 1.6 % bw, CoP 1.0 cm, knee torque 3.7 % of peak (`test_video_fullbody_e2e`); real squat and jump (`replay_body_squat`, `replay_body_jump`): weight standing within ~1 %, ~0 in the air. Peak forces of fast motions depend on the filter. See `phases/stage_A_video.md` A3c |
| **Whole body from the front** (sideways arm raise, lateral lean, side step) | Same, in the frontal plane (`bodyTreeFromPose 'View','frontal'`: hips / shoulders at their own positions) | Tested on a real clip (sideways arm raise): vertical force 1.00 bw mean, lateral CoP between the feet, shoulder torques = the arms' gravity moment. The split of the weight between the feet is an estimate |
| **3D skeleton from one front camera** (squat, forward arm raise filmed from the front; height known) | `liftPose3D`: 3D joint angles and the motion toward the camera — no 3D Simscape model yet | Tested: synthetic knee / hip 7.6 / 5.0 deg rms, distance within 1 % (`test_video_lift3d_e2e`); real squats plausible (knee 108–128°, hips 18–24 cm back). Limbs that leave the frame are guessed. See A3d |
| Other limbs (leg, both arms) | Same as the arm with another keypoint chain | Built, not yet tested on a clip |
| Robot arm / mechanism | Segment Anything per link → link angles → `multibodyBuilder` | Built, not yet tested on a clip |
| **Pendulum** / swinging object | Pivot, length from the swing period, damping; passive Multibody model (tip mass) fitted on part of the clip, predicts the rest | Tested (`tests/videoToSimTests/test_video_pendulum_e2e.m`, `replay_pendulum.m`): length from the period 0.307 m vs 0.305 m measured, held-out angle 4.4 deg RMS |

## Recording conditions

- **Fixed camera** — no panning or zooming.
- **Motion parallel to the camera** (film side-on; or from the front for frontal-plane motion such as
  a sideways arm raise). Motion toward or away from the camera can only be approximated (a person:
  `liftPose3D`, A3d); the planarity check reports when that happens.
- **Whole subject in view** for the entire motion — raised hands and feet included (a person: about
  3–4 m from the camera); plain background if possible. Start by standing still for ~2 s.
- **A known length** for objects, **the height (and mass)** for people, for results in metres and
  forces. Objects in free flight need none: gravity gives the scale.
- **Short clips** (3–6 s). On CPU, pose tracking takes ~0.6 s per frame.

## Not supported yet

- Several objects at once (traffic, crowds, several balls)
- Moving or zooming cameras, strong perspective (camera looking down a road)
- 3D models: a person filmed facing the camera gives a rough 3D skeleton (`liftPose3D`, known
  height; joint angles, motion toward the camera), but there is no 3D Simscape model of it yet.
  Body turning, hands and fingers are not supported
- Deforming objects (cloth, liquids), physics that is not visible (electrical, thermal)

## Required products

| Feature | Needs |
|---------|-------|
| All of video mode | Image Processing Toolbox, Computer Vision Toolbox |
| Articulated models (`multibodyBuilder`) | Simscape Multibody |
| Parameter fitting (`fitParameters`) | Optimization Toolbox (fallback: `fminsearch`) |
| Body pose (`extractPose`) | Computer Vision Toolbox Model for Object Keypoint Detection; Computer Vision Toolbox Model for RTMDet Object Detection (automatic person boxes, optional) |
| Object / link masks (`trackSAM`) | Image Processing Toolbox Model for Segment Anything Model 2 (or ... Segment Anything Model) |
| Webcam recording (`captureVideo`) | Image Acquisition Toolbox + Image Acquisition Toolbox Support Package for OS Generic Video Interface |

## Optional outputs

| Output | Function |
|--------|----------|
| Simulation drawn on the real video | `renderOverlayVideo` (always produced) |
| Tracking video: boxes, masks, skeleton, joint-angle arcs, angle chart | `renderTrackingVideo` |
| Several stages side by side or in sequence | `renderPipelineVideo` |

All three accept `'Play', true` to start playing as soon as the video is written.
