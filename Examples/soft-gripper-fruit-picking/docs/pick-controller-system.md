# Pick controller — system + architecture spec (quick mode)

**Status:** 🛑 **GATE D PASSED 2026-10-24**: offline planning (A); mango carried at 0.5 m/s²; raspberry picked from a stem. **Last updated:** 2026-10-24.
**Model:** `SoftGripperPick.slx` → `Controller` subsystem (the Phase 0 stub, frozen ports, system spec §7).
**Quick mode** (this spec plus `rationale.md`): one builder, single rate, and a sequencer whose modes are
all specified here. The engineer prefers lean paperwork.
**Every number** in this spec traces to a row in [`rationale.md`](rationale.md).

## 1. What it does

It picks one object (objId 1–6) from the table and places it in the tray with the UR5e and the soft gripper,
within `T_PickMax` = 15 s and at most 3 attempts (REQ_GRIP_018), without bruising (016) and with slip
≤ 5 mm under the transport acceleration (017). It sees only `qArmMeas`, `pGripMeas`, `bendMeas` and the
camera estimate `objPoseInit` (REQ_GRIP_021/022).

## 2. External interface (frozen, system spec §7)

| Dir | Signal | Unit | Use |
|---|---|---|---|
| in | `objPoseInit` (6) | m, rad | camera estimate of the object pose → the plan (an offline computation, §4.2) |
| in | `qArmMeas` (6) | rad | arrival check at each waypoint |
| in | `pGripMeas` | Pa gauge | pressure reached / supply-droop detection |
| in | `bendMeas` (4) | rad | contact detection, grasp check, drop detection |
| out | `qArmCmd` (6) | rad | UR5e joint commands (the arm follows them exactly, REQ_GRIP_020) |
| out | `pGripCmd` | Pa gauge | regulator set pressure, [0, 60 kPa]; **[30, 60] kPa while gripping** (validity range, engineer 2026-10-24) |

Parameters (calibratable, in `params_softgripper_ctrl.m`): `Ts_Ctrl`, per-object `pGrip`, `aTransport`,
grasp heights, contact and drop thresholds, timeouts. None is hardcoded in a block.

## 3. Context characterization (measured, M2)

| Quantity | Value | Source |
|---|---|---|
| Fill time 0 → 95 % of 60 kPa | 17.7 ms | T-G1 |
| Finger closing, 10–90 % bend, 0.1 s pressure ramp | 35 ms (60 kPa) … 59 ms (20 kPa) | GripperBench T-I1 runs |
| Peak bend rate | 3300 °/s (60 kPa) | same |
| Minimum hold pressure, load m(g + 2 m/s²) | apple 34.3, tomato 34.0, pear 51.9, **mango 59.9** kPa | T-I2 |
| Grasp-force slope (enveloping, 40 → 50 kPa) | 0.21 N/kPa | T-I1 |
| Bend-sensor noise | 0.5° (EST) | `params_softgripper_ctrl` |
| Arm input filter | 20 ms | `params_softgripper_arm` |
| Reach | every waypoint of §4.3 by IK, error ≤ 2e-8 m | `inverseKinematics`, 2026-10-24 |
| **Fingertip reach, palm 110 mm above table** (straight fingers must clear it) | raspberry: tips meet **above** it (16 mm over its centre) → **not graspable**; cherry tomato: at its equator (marginal); apple … mango: 25–42 mm **below** the centre (cradled) | free-bend arc geometry, `r_Mount` 60 mm |

## 4. Architecture

```
objPoseInit ─► PickPlan (MATLAB, before t = 0: IK waypoints + segment durations) ─► P.plan
                                                                                    │
qArmMeas, pGripMeas, bendMeas ─► PickTask (Stateflow, Ts_Ctrl) ──► segment k, t_seg ─► ArmTraj (quintic per segment) ─► qArmCmd
                                        │                                                                  
                                        └──► grip mode ─► GripCmd (per-object pressure, closing ramp, [30,60] kPa clamp) ─► pGripCmd
```

| Component | Implementation | Rate | States |
|---|---|---|---|
| **PickPlan** | MATLAB function `planSoftGripperPick(objPose, objId, P)` called in the run script; result injected by `in.setVariable` | offline | — |
| **PickTask** | Stateflow chart, MATLAB action language | `Ts_Ctrl` | mode, attempt counter, timers |
| **ArmTraj** | Stateflow MATLAB function: minimum-jerk (quintic) interpolation between the waypoints of segment k | `Ts_Ctrl` | none (time since segment entry) |
| **GripCmd** | inside PickTask: pressure ramp generator + clamp | `Ts_Ctrl` | ramp start value |
| **Detectors** | inside PickTask: contact, grasp-OK, drop, pressure-reached | `Ts_Ctrl` | sample counters |

**No integrators** in the controller: the only pressure loop is the ITV regulator inside the plant. So there is
no anti-windup to specify; every timer and counter is reset on state entry (§5).

### 4.1 Why offline planning (candidate A) — the tradeoff

| | A: offline plan (recommended) | B: online IK block + Polynomial Trajectory |
|---|---|---|
| What the example teaches | soft-gripper modelling (the point of the example) | more robotics plumbing |
| Sim cost | none at run time | IK at every sample |
| Information used | `objPoseInit` only (the same as B: the camera estimate is constant per pick) | same |
| Retry | re-uses the same plan (the object stays near its pose) | could re-plan |
| Limit | an object that moves a lot after a failed attempt is not re-planned | — |

### 4.2 Waypoints (each a tool0 pose, gripper pointing down)

`Home` → `Above` (100 mm over the grasp height) → `Grasp` → `Lift` (+150 mm) → `OverTray` → `Release` (20 mm
over the tray height) → `Home`. Grasp height = palm underside at `max(L_Finger + 5 mm, h_obj + 1 mm)` above
the table: the fingertips clear the table, and the palm clears the fruit.
Segment durations: minimum-jerk, scaled until the peak tool acceleration ≤ `aTransport(obj)`.

### 4.3 Layout (EST, scene not fixed before M3)

UR5e base at the world origin, table top at z = 0, object at (0.45, 0.10) m, tray at (0.10, −0.45) m.

## 5. Modes (PickTask)

| State | Entry | Does | Exit → |
|---|---|---|---|
| `Settle` | t = 0 | arm at Home, p = 0; lets the object settle | t ≥ `tSettle` → `Approach` |
| `Approach` | seg Home→Above→Grasp | arm moves, fingers vented | arm arrived (‖qMeas − qCmd_end‖ < `qTol`) → `Close` |
| `Close` | ramp p 0 → `pGrip(obj)` over `tCloseRamp` | watch bends | all 4 bends stalled (contact) AND p ≥ `pGrip` − `pAcc` → `CheckGrasp`; `tCloseMax` elapsed → `Retry` |
| `CheckGrasp` | hold p | contact = bend < `bendFree(p)` − `dBendContact` on ≥ 3 fingers | yes → `Lift`; no → `Retry` |
| `Lift`, `Transport`, `Lower` | segments Grasp→Lift→OverTray→Release | **drop detector** active: ≥ 2 bends rise above `bendFree(p)` − `dBendContact` | arrived → `Release`; drop → `Retry` |
| `Release` | p → 0 (vent) | wait `tVent` | → `Retreat` |
| `Retreat` | seg Release→Home | | arrived → `Done` |
| `Retry` | p → 0, attempt += 1 | segment back to Above | attempt ≤ `N_Attempts` → `Approach`; else → `Failed` |
| `Done` / `Failed` | terminal, safe state: p = 0, arm at Home | | — |
| **Global** | any state: `t ≥ T_PickMax` → `Failed`; `pGripMeas` < `pGrip` − `pDroopTol` for `tDroop` while gripping → flag `supplyLow` (logged, no action: REQ_GRIP_026 reports it) | | |

Bumpless transfer: every arm segment starts at the previous segment's end waypoint; the pressure ramp starts
at the present command. Initialization on entry: segment timer = 0, counters = 0.

## 6. Failure paths

| Path | Handling |
|---|---|
| Pressure saturation | `pGripCmd` clamped to [0, 60 kPa]; while gripping to [30, 60] |
| Mango needs > 60 kPa for the 20 % load margin | **Decided:** transport at 0.5 m/s² (14 % margin), rationale R7 |
| Raspberry unreachable from the table | **Decided:** stem pick (REQ_GRIP_027), rationale R8 |
| Missed grasp / drop | `Retry`, ≤ 3 attempts (REQ_GRIP_018) |
| Supply droop | flag `supplyLow`; the pick continues (REQ_GRIP_026 asks to report the limit) |
| Pick overrun | `T_PickMax` → `Failed` safe state |
| Bend sensor out of range | values outside [−10°, 200°] → treated as no-contact; logged |
| Finger touching the table | prevented by the grasp height (5 mm clearance); checked in M4 from the truth fingertip height |

## 7. Architecture review gate (item by item)

| Item | Status |
|---|---|
| Dimensional consistency | rad, Pa gauge, m, s throughout; `bendFree(p)` takes Pa gauge |
| Feedback polarity | no control loop in the controller; the detectors compare bend < threshold (contact) and > threshold (drop) — signs checked in §5 |
| Anti-windup / reset | no integrators; every timer and counter is reset on state entry |
| Mode transitions | every state has a guarded exit; global timeout; bumpless segment start |
| Rate transitions | single rate `Ts_Ctrl`; the plant is continuous; ZOH at the controller boundary (SensorSet) |
| Saturation | pressure clamp; joint commands are IK solutions within limits; arm filter in the plant |
| Algebraic loops | **Corrected at build (2026-10-24):** a Stateflow chart is direct feedthrough, so chart → plant → ZOH → chart was an algebraic loop. Fixed with a **Unit Delay on each y input** (one sample, 5 ms sensor latency); IC: qArmMeas = qHome, pGripMeas = 0, bendMeas = 0 (the plant's initial state) |
| Codegen | not a deliverable (M5 skipped); MATLAB action language kept codegen-friendly |
| Calibratable parameters | all in `params_softgripper_ctrl.m`, none in a block |

## Appendix B — API verification

| API | Checked | Result |
|---|---|---|
| `inverseKinematics('RigidBodyTree', rb)`, `step`-free call `ik('tool0', T, w, q0)` | 2026-10-24, UR5e, 5 waypoints × 4 objects | converges, position error ≤ 1.7e-8 m |
| `loadrobot('universalUR5e')` home pose `qHome` | tool0 z axis = [0 0 −1] | points down |
