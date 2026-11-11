# Soft gripper: verification results

Results per requirement (REQ_GRIP_001–031). Three kinds of test:

- **Component**: one part on its own test bench (finger, gas supply, fruit, arm).
- **Integrated**: the whole gripper on `GripperBench.slx`, without the arm.
- **Closed loop**: a full pick in `SoftGripperPick.slx`, controller and plant together.

## Tests

| ID | What it checks | Script |
|---|---|---|
| T-F1 | free bend at 40 / 50 / 60 kPa | `check_softgripper_finger("T-F1")` |
| T-F2 | tip force, Zhu Table 1 (29 points) | `check_softgripper_finger("T-F2")` |
| T-F3 | sealed finger obeys the gas law | `check_softgripper_finger("T-F3")` |
| T-G1, T-G2 | fill and vent times | `check_softgripper_gas` |
| T-O1 | fruit settles on the table | `check_softgripper_objects` |
| T-O2 | UR5e tool pose vs Robotics System Toolbox | `check_softgripper_arm` |
| T-I1 | grasp force, Zhu Table 3 | `check_softgripper_grasp` |
| T-I2 | minimum hold pressure, Zhu Table 4 | `check_softgripper_hold` |
| T-I4 | sideways holding capacity | `check_softgripper_lateral` |
| T-I5, T-I6 | holding capacity under the gripper's own acceleration | `check_softgripper_holdg`, `check_softgripper_capacity` |
| T-I7 | impact force vs closing speed | `check_softgripper_closing` |
| — | model structure; no simulation-only signal reaches the controller | `verify_softgripper_hierarchy` |

All check scripts save one result file per case in `results/` and can re-score saved runs without simulating.

## Pass/fail per requirement

| Requirement | What it asks | Evidence | Result |
|---|---|---|---|
| REQ_GRIP_001 | pick each object from the table into the tray | closed loop: apple, tomato, pear (below) | **PASS** for the pick set (see 015) |
| REQ_GRIP_003 | 8-hinge finger in Simscape Multibody | `SoftFinger.slx` = 8 × `Hinge.slx` | **PASS** |
| REQ_GRIP_004 | tip force vs Zhu Table 1, RMS ≤ 0.08 N | T-F2: RMS 0.050 N, max 0.127 N | **PASS** |
| REQ_GRIP_005 | free bend 75 / 90 / 110° ± 10° | T-F1: 81.1 / 93.5 / 103.7° | **PASS** |
| REQ_GRIP_006 | chamber volume follows the bend | T-F3: −0.008 % from the adiabatic gas law | **PASS** |
| REQ_GRIP_008 | one regulator, 0.1 MPa full scale, ≤ 60 kPa | `GripperGas.slx`; T-G1 | **PASS** |
| REQ_GRIP_009 | one shared manifold | `GripperGas.slx` | **PASS** |
| REQ_GRIP_010 | fill 0 → 95 % in 17.3 ms ± 10 % | T-G1: 17.7 ms (+2.2 %) | **PASS** |
| REQ_GRIP_012 | grasp force vs Zhu Table 3 ± 30 % | T-I1, enveloping, 30–60 kPa: 1.64 / 3.30 / 5.40 / 7.51 N vs 2.28 / 3.47 / 4.68 / 5.80 N | **PASS** at 30–60 kPa (4/4). 10–20 kPa is outside the valid range. Fingertip grasp cannot be checked: the paper does not give the ball position, and the force changes ×7 over 20 mm. |
| REQ_GRIP_013 | hold pressure vs Zhu Table 4 ± 10 kPa | T-I2: apple 34.2, tomato 34.0, pear 43.7, mango 59.8 vs 30 / 35 / 45 / 60 kPa | **PASS** (4/4). This test closes at 60 kPa and then lowers the pressure, so it measures release; see note 2 below. |
| REQ_GRIP_015 | the six-object set | all six modelled; apple, tomato, pear picked | **PARTIAL**: the mango needs more than the 60 kPa finger rating while moving; the raspberry and cherry tomato are too small for the 60 mm finger spacing. |
| REQ_GRIP_016 | contact force ≤ bruise limit | closed loop: apple 15.6 / 40, tomato 10.5 / 11, pear 26.3 / 40 N | **PASS** (3/3); tomato margin 5 % |
| REQ_GRIP_017 | slip ≤ 5 mm while carried | closed loop: 2.1 / 2.1 / 3.1 mm | **PASS** (3/3) |
| REQ_GRIP_018 | in the tray within 15 s, ≤ 3 attempts | closed loop: 5.21 / 5.21 / 5.72 s, 1 attempt each | **PASS** (3/3) |
| REQ_GRIP_020 | the arm follows its commands exactly | motion-driven joints; T-O2 tool pose error 2e-15 m | **PASS** |
| REQ_GRIP_021 | the controller uses only measurable signals | `verify_softgripper_hierarchy` traces every line into the controller | **PASS** |
| REQ_GRIP_022 | no simulation-only signal reaches the controller | `verify_softgripper_hierarchy` | **PASS** |
| REQ_GRIP_024 | grip with friction as low as 0.5 | — | **NOT VERIFIED** |
| REQ_GRIP_025 | a leaking finger | — | **NOT VERIFIED** |
| REQ_GRIP_026 | supply pressure droop to 0.16 MPa | — (the controller flags `supplyLow`) | **NOT VERIFIED** |
| REQ_GRIP_027 | raspberry picked from its stem | T-I6, T-I7: the fingers push the fruit 33–41 mm up; stem pull 6.6–8.2 N > 2 N | **NOT MET** (too small for the finger spacing) |
| REQ_GRIP_029 | 3D replay from the logged picks | `playSoftGripperPick` reads `results/pick_<id>.mat` only; GIFs for apple, tomato, pear | **PASS** |
| REQ_GRIP_030 | product list | the model uses Simulink, Simscape, Simscape Multibody, Stateflow, Robotics System Toolbox; Optimization Toolbox only in `fit_zhu_finger_prb.m` | **PASS** by inspection and a run from a clean copy of the folder |
| REQ_GRIP_031 | one pick runs in under 35 min | apple 23.6, tomato 22.0, pear 31.8 min | **PASS** (3/3) |

## Closed-loop picks

| Fruit | Grip pressure | Released in tray | Attempts | Slip | Peak contact / bruise limit |
|---|---|---|---|---|---|
| apple 200.6 g | 45 kPa | 5.21 s | 1 | 2.1 mm | 15.6 / 40 N |
| tomato 282.9 g | 39.2 kPa | 5.21 s | 1 | 2.1 mm | 10.5 / 11 N |
| pear 389.1 g (lying) | 60 kPa, lift at 1 m/s² | 5.72 s | 1 | 3.1 mm | 26.3 / 40 N |

Settings: controller sample time 5 ms, closing ramp 0.3 s, solver daessc, RelTol 1e-4, zero-crossing detection
off; finger contacts on links 4–8 only. Logs: `results/pick_3.mat`, `pick_4.mat`, `pick_5.mat`.

## What the closed loop shows that the benches do not

1. **Benches that load only the fruit overestimate the grip.** In a pick the whole gripper accelerates, so the
   fingers carry their own inertia. T-I5 and T-I6 therefore load fingers and fruit through *g* − *a*<sub>gripper</sub>.
2. **Lowering the pressure after closing measures release, not grasp formation.** T-I6 closes at the working
   pressure. Against the picks, a capacity ratio ≥ 1.66 held and ≤ 1.27 slipped.
3. **A 0.1 s closing ramp gives impact spikes** above the bruise limits (tomato 18.7 N, pear 46.5 N); 0.3 s does
   not (T-I7).
4. **Zero-crossing detection on the regulator limits** shrinks the solver step to 2e-6 s while the arm
   accelerates; it is off.

## Not verified

- Low friction, a leaking finger and supply droop (REQ_GRIP_024–026).
- Robustness to parameter spread (contact stiffness and damping, finger mass, dead volume). The tomato's 5 %
  bruise margin depends on the estimated finger and contact damping, so that is the first sweep to run.
- The logged truth signal `objState` (from `Plant/Object`) is not modelled and is always zero; the fruit pose
  is in `objPose`. It is logging only and never reaches the controller.
