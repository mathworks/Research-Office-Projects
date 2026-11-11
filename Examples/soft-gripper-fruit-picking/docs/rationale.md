# Pick controller: where each value comes from

Every controller value in `params/params_softgripper_ctrl.m` is derived the same way: measure a quantity on the
model, turn it into a constraint, choose the value. "Estimated" marks a value with no measurement behind it.
Test IDs (T-F1, T-I6, ...) are listed in [`soft-gripper-verification.md`](soft-gripper-verification.md).

## Timing

| Parameter | Value | Derived from |
|---|---|---|
| `Ts_Ctrl` controller sample time | **5 ms** | The fastest finger closing (10–90 % bend) takes 35 ms at 60 kPa, and the arm input filter has a 20 ms time constant. 5 ms gives at least 7 samples across the closing and 4 per filter time constant. |
| `tCloseRamp` pressure ramp when closing | **0.3 s** | Closing-speed study T-I7, peak impact force on the fruit: tomato 17.4 / 9.7 / 6.0 / 3.8 N at 0.1 / 0.3 / 0.6 / 1.0 s ramps (bruise limit 11 N); pear 24.9 / 15.1 N at 0.3 / 0.6 s (limit 40 N). 0.3 s is the shortest ramp under every limit. |
| `tSettle` wait before the pick starts | **0.3 s** | The fruit settles on the table in at most 0.13 s (T-O1); 2× margin. |
| `tCloseMax` closing timeout | **0.85 s** | Ramp 0.3 s + bend transient 0.06 s + stall window 0.05 s, about 2× margin. |
| `tVent` venting time at release | **0.2 s** | The fingers vent to 5 % pressure in 47 ms (T-G2) and unbend in about 0.1 s; 1.3× margin. |

## Grip pressure and motion

| Parameter | Value | Derived from |
|---|---|---|
| `pGrip` grip pressure | apple **45**, tomato **39.2**, pear **60 kPa** | Capacity test T-I6: the gripper closes *at* the working pressure, then an extra load ramps on until the fruit slips 5 mm. Fingers and fruit are both loaded through an effective gravity *g* − *a*<sub>gripper</sub>. The capacity ratio (load at slip / working load) must be at least **1.66** in lift and carry: the lowest ratio that held in a full pick (ratios of 1.27 slipped). |
| `pGripMin`, `pGripMax` limits while gripping | **30 … 60 kPa** | Below about 30 kPa the fingers do not reach the fruit; 60 kPa is the finger rating. |
| `aTransportObj` carry acceleration | **2 m/s²** | Requirement REQ_GRIP_017. Motion segments are minimum-jerk, scaled until the peak tool acceleration is at most this value. |
| `aLiftObj` lift and lower acceleration | **2 m/s²**, pear **1 m/s²** | The pear at 60 kPa reaches a capacity ratio of 1.60 at 2 m/s² and about 1.75 at 1 m/s². |
| Fruit orientation | pear and mango **lying on their side** | Upright, the pear held only 1.45 m/s² sideways and the mango 0.02 m/s² (T-I4). |

## Grasp check and drop detection

| Parameter | Value | Derived from |
|---|---|---|
| `bendFree(p)` free-bend angle | 0 / 38.0 / 64.6 / 81.1 / 93.5 / 103.7° at 10 … 60 kPa | Measured on the Simscape finger (T-F1). A finger bent less than this is touching the fruit. |
| `dBendContact` contact threshold | **5°** | At least 6× the bend-sensor noise (0.5°, estimated) = 3°, and at most half the smallest contact deficit (12° at 30 kPa) = 6°. |
| Stall detector | **30 °/s over 10 samples (50 ms)** | Sensor noise on a 10-sample bend difference is about 14 °/s; the closing peak is 3300 °/s. The threshold is 2× the noise and far below the peak. |
| `qTol` arm arrival tolerance | **5e-4 rad** | With 0.85 m reach this is at most 0.4 mm at the tool, under half the 1 mm palm-to-fruit clearance. |
| `pDroopTol`, `tDroop` supply-droop flag | **4 kPa for 0.1 s** | 2× the regulator accuracy (±2 kPa), and much longer than the 17.7 ms fill time. |
| `hClear` fingertip clearance over the table | **5 mm** | At least 10× the 0.5 mm arm tolerance above. |

## Scene and model

| Item | Value | Derived from |
|---|---|---|
| Layout | object (0.45, 0.10) m, tray (0.10, −0.45) m, table top z = 0 | Estimated. Inside the UR5e's 0.85 m reach; inverse kinematics reaches every waypoint to 2e-8 m. |
| Finger contacts in the pick model | links 4–8 of each finger (20 of 32) | In the logged picks, links 1–3 never come within 14.5 mm of the fruit; link 4 comes within 6.5 mm. Dropping the unused contacts makes the pick faster. The test benches keep all 32. |
