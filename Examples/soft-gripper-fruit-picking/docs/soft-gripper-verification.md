# Soft gripper — M4 verification report

**Date:** 2026-10-25. **Scope (engineer decision 2026-10-25): minimal M4** — the structural truth-isolation check,
the component and integrated MIL checks from M2, and the three nominal closed-loop picks. The off-nominal set
(friction, leak, supply droop) and the robustness sweeps were **not run** and are listed as not verified.
**Stages:** component MIL (finger, gas, objects, arm benches), integrated MIL (GripperBench), system MIL
(SoftGripperPick closed loop). No SIL/PIL: M5 is skipped (the deliverable is a model and an example).

## 1. Pass/fail per requirement

| Requirement | What it asks | Evidence (stage) | Result |
|---|---|---|---|
| REQ_GRIP_001 | pick each object of the set table → tray | system MIL: apple, tomato, pear picked (below) | **PASS for the pick set**; mango, raspberry, cherry tomato descoped by the engineer (see 015/027) |
| REQ_GRIP_003 | 8-joint PRB finger in Simscape Multibody | `SoftFinger.slx` (8 × `Hinge.slx`) | **PASS** (structure) |
| REQ_GRIP_004 | Zhu Table 1 end force, RMS ≤ 0.08 N | T-F2, 29 points: RMS 0.050 N, max 0.127 N | **PASS** |
| REQ_GRIP_005 | free bend 75/90/110° ± 10° | T-F1: 81.1 / 93.5 / 103.7° | **PASS** |
| REQ_GRIP_006 | volume–bend coupling D per joint | T-F3 sealed finger: −0.008 % vs the adiabatic law | **PASS** |
| REQ_GRIP_008 | one regulator, 0.1 MPa F.S., ≤ 60 kPa | `GripperGas.slx` (CmdSat, valve limits); T-G1 | **PASS** |
| REQ_GRIP_009 | one shared manifold | `GripperGas.slx` | **PASS** (structure) |
| REQ_GRIP_010 | fill 0 → 95 % within 17.3 ms ± 10 % | T-G1: 17.7 ms (+2.2 %); flow calibration 0.9950 (edge of ± 0.5 %) | **PASS** |
| REQ_GRIP_012 | Zhu Table 3 grasp force ± 30 % | enveloping 30–60 kPa: 1.64/3.30/5.40/7.51 vs 2.28/3.47/4.68/5.80 N | **PASS 30–60 kPa (4/4)**; 10–20 kPa NOT MET (outside the validity range, engineer); fingertip **not verifiable** (ball placement unpublished; ×7 sensitivity over 20 mm) |
| REQ_GRIP_013 | Zhu Table 4 hold pressure ± 10 kPa | T-I2 (pick placement, lying pear/mango): apple 34.2, tomato 34.0, pear 43.7, mango 59.8 vs 30/35/45/60 | **PASS (4/4)** — caveat: a ramp-down test measures *release* (closes at 60 kPa first); see §3 |
| REQ_GRIP_015 | the six-object set | all six modelled; picked: apple, tomato, pear | **PARTIAL**: mango beyond the 60 kPa rating; raspberry and cherry tomato too small for the 60 mm spacing (engineer, R3c, R8b) |
| REQ_GRIP_016 | contact ≤ F_Bruise | picks: apple 15.6/40, tomato **10.5/11**, pear 26.3/40 N | **PASS (3/3)**; tomato margin 5 % |
| REQ_GRIP_017 | slip ≤ 5 mm under transport | picks: 2.1 / 2.1 / 3.1 mm (pear lifted at 1 m/s², engineer) | **PASS (3/3)** |
| REQ_GRIP_018 | in the tray ≤ 15 s, ≤ 3 attempts | picks: released in the tray at 5.21 / 5.21 / 5.72 s, 1 attempt each | **PASS (3/3)** |
| REQ_GRIP_020 | arm follows exactly; README states it | motion-driven joints; T-O2 tool pose 2e-15 m | **PASS** (model); README statement at M6 |
| REQ_GRIP_021 | controller uses only y + objPoseInit | T-C7 structural line trace (`verify_softgripper_hierarchy`) | **PASS** |
| REQ_GRIP_022 | no truth signal reaches the controller | T-C7 | **PASS** |
| REQ_GRIP_024 | μ_Min = 0.5 | — | **NOT VERIFIED** (minimal M4) |
| REQ_GRIP_025 | leaking finger | — | **NOT VERIFIED** (minimal M4) |
| REQ_GRIP_026 | supply droop 0.16 MPa | — (the controller logs `supplyLow`) | **NOT VERIFIED** (minimal M4) |
| REQ_GRIP_027 | raspberry stem pick | stem picks and T-I6/T-I7: the fruit is shoved 33–41 mm up, stem pull 6.6–8.2 N > 2 N | **NOT MET** — out of range (engineer, R8b) |
| REQ_GRIP_029 | 3D replay of logged picks | `playSoftGripperPick` reads `results/pick_<id>.mat` only; GIFs for apple, tomato, pear (M6, 2026-10-26) | **PASS** |
| REQ_GRIP_030 | product list | the model uses Simulink, Simscape, Simscape Multibody, Stateflow, Robotics System Toolbox; Optimization Toolbox only in `fit_zhu_finger_prb.m`; Requirements Toolbox only in the `update_*` scripts | **PASS by inspection + clean-folder test** (2026-10-26, from the packaged folder alone: hierarchy check 10/10, T-F1 finger bench re-run 81.1/93.5/103.7°, replay loader for apple and pear). Not run: a machine with only the listed products installed |
| REQ_GRIP_031 | one pick within T_RunPick (35 min, Gate E) | final model (20 finger contacts, stop 0.6 s after release): apple 1414 s, tomato 1319 s, pear 1910 s (23.6 / 22.0 / 31.8 min) | **PASS (3/3)** |

## 2. Closed-loop picks (system MIL, final design)

| Fruit | pGrip | Released in tray | Attempts | Slip | Peak contact / F_Bruise |
|---|---|---|---|---|---|
| apple 200.6 g | 45 kPa | 5.21 s | 1 | 2.1 mm | 15.6 / 40 N |
| tomato 282.9 g | 39.2 kPa | 5.21 s | 1 | 2.1 mm | 10.5 / 11 N |
| pear 389.1 g (lying) | 60 kPa, lift 1 m/s² | 5.72 s | 1 | 3.1 mm | 26.3 / 40 N |

Settings: Ts_Ctrl 5 ms, closing ramp 0.3 s, solver daessc, RelTol 1e-4, zero-crossing detection off; tool yaw
held at the home quarter-turn and turned with the base on the carry (wrist 3 moves ≤ 0.07 rad); finger contacts on
links 4–8 only (R18). Final runs 2026-10-26.
Files: `results/pick_3.mat`, `pick_4.mat`, `pick_5.mat` (and `runSoftGripperPick`, `scoreSoftGripperPick`).

## 3. What the closed loop taught (evidence the benches did not give)

1. **Bench benches that load only the fruit overestimate the grip.** In the pick the whole gripper accelerates, so
   the fingers carry their own inertia. Fix: T-I5/T-I6 use g_eff = g − a_gripper.
2. **Ramp-down hold tests measure release, not formation** (they close at 60 kPa first). T-I6 closes at the
   commanded pressure; calibrated on the picks, a capacity ratio ≥ 1.66 held and ≤ 1.27 slipped (R3c).
   The REQ_GRIP_013 comparison with Zhu Table 4 used the ramp-down method, as Zhu's own procedure is unpublished.
3. **The 0.1 s closing ramp caused impact spikes** above the bruise limits (tomato 18.7 N, pear 46.5 N). 0.3 s
   fixes it (T-I7, R2b).
4. **Zero-crossing detection on the regulator limits** made the solver crawl (2e-6 s steps) while the arm
   accelerated; turned off.

## 4. Not verified, and why

- REQ_GRIP_024, 025, 026 (off-nominal): not run, engineer chose the minimal M4.
- Robustness (test plan §6–7: tolerance, contact width, damping ×0.3…×3, finger mass, dead volume): not run. The
  tomato's 5 % bruise margin depends on the estimated finger and contact damping — the first item to run if M4
  is extended.
- Coverage and guideline compliance: not measured (not used elsewhere in this repo; offered as follow-up).
- Persistent `test_create` tests: approved at the test plan, not created in this minimal scope.
