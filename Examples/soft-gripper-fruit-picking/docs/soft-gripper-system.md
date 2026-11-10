# Soft gripper on a UR5e — plant model System spec

## Status: ✅ Reviewed by the engineer 2026-10-23 ("ok"). §7 freezes after the architecture review.
**Author:** agent, following the `specifying-plant-models` skill (full-spec mode: 4 fingers × 8 joints = 32
mechanical DOF, gas states, nonlinear contact).
**Reviewer:** engineer. ⛔ **§7 becomes the frozen interface on approval.**
**Parent decisions:** the approved scope (engineer, 2026-10-23, with the amendment), an internal planning note not shipped here.
**Pending inputs:** fruit dimensions, per-fruit bruise limits and friction, from a research agent →
§5 and §8.2 are marked TBD.

---

## 1. Executive Summary

This plant model is a **four-finger soft pneumatic gripper mounted on a rigid UR5e arm**, built in
Simscape. It exists to teach how soft robots are modelled.

- **Each finger** is a Simscape Multibody chain of eight hinges, one per air chamber of the published finger
  (Zhu et al. 2022).
- **The air** comes from one low-range regulator, fed through a manifold to the fingers and modelled in
  Simscape Gas. Air pressure turns each hinge, and bending changes the chamber volume.
- **The fingers touch objects** of different shape and mass through Multibody contact with friction.

The controller it closes the loop with (designed at M3) sequences the pick, plans the arm motion and
regulates the grip. Using only realistic measurements (gripper pressure, finger bend sensors, arm joint
angles), it picks a raspberry, a cherry tomato, an apple, a tomato, a pear and a mango off a table into a
tray.

## 2. Problem Statement

### Current situation
1. **No soft-gripper plant exists.** The earlier soft-arm models are a different robot, parked as a future
   project.
2. **Soft-finger behaviour is strongly nonlinear.** The silicone stiffens as it bends, and contact changes
   with object shape. Simple "pressure in, force out" models cannot show pinch and wrap grasps.

### Opportunity
A validated, reusable Simscape soft-finger model that readers can adapt: a different finger, more fingers,
other objects. The published data checks it at three levels: finger force table, gripper grasp force, and
the hold pressure per fruit.

## 3. Goals & Success Metrics

| Goal | Description |
|---|---|
| **G1 Finger fidelity** | One finger reproduces Zhu Table 1 (end force vs pressure 10–60 kPa × bend 0–90°) |
| **G2 Gripper fidelity** | Four fingers reproduce Zhu Table 3 (fingertip and enveloping grasp force vs pressure) |
| **G3 Task fidelity** | The model predicts the minimum hold pressure per fruit, compared with Zhu Table 4 (apple 30, tomato 35, pear 45, mango 60 kPa; cherry tomato 10 kPa fingertip) |
| **G4 Usability** | One pick runs fast enough to iterate (target set at M2, measured) |

**Success metrics (tolerances = 🛑 GATE A engineer decisions, not set here):**
- Finger: RMS end-force error vs Table 1 ≤ `e_FingerRms`. The offline feasibility fit reached **0.050 N RMS**
  with cubic springs.
- Gripper: grasp force vs Table 3 within `e_GraspRel`.
- Task: predicted hold pressure vs Table 4 within `e_HoldPress`.

## 4. Non-Goals (v1)

| Non-Goal | Rationale |
|---|---|
| UR5e dynamics (masses, motors, joint flexibility) | Engineer-approved kinematic arm. Its servos are much faster and stiffer than the grasp task, and the arm is not the subject |
| Finger material FEM / continuum hyperelasticity | A lumped 8-joint chain with stiffening springs already fits the measured table (feasibility fit); FEM is not the teaching point |
| Object deformation beyond a contact spring | Objects are rigid solids with a contact stiffness; bruising is judged from contact force |
| Thermal effects in the air | Adiabatic, as in the soft-arm work |
| Vision / object detection | The object pose is given to the controller (w), as a camera would on a real cell |
| Fixed-step / real-time / codegen | M5 skipped; nothing deploys |

## 5. Operating Scenarios

### Scenario 1: Pick a small, delicate object (raspberry, cherry tomato) with a fingertip pinch
**Conditions:** object on the table; supply nominal; μ nominal.
1. The arm moves the open gripper above the object.
2. The grip pressure rises; the fingertips close on the object before full bend.
3. The object is lifted, carried and released over the tray.

**Controller behaviour:** keep the contact force inside `[F_GripMin, F_Crush]`; for the raspberry,
`F_Crush` = 3 N.

### Scenario 2: Pick a large, heavy object (apple → mango) with an enveloping wrap
**Conditions:** object on the table.
1. The fingers wrap around the object, contacting along their length.
2. The pressure needed rises with mass. Zhu measured 30–60 kPa for 200–580 g.

**Controller behaviour:** choose a pressure that holds the object through the transport acceleration
without slip.

### Scenario 3: Off-nominal
- Low friction (μ sweep).
- A leaking finger (one finger chamber vented through an orifice).
- Supply droop.
- (Optional) raspberry still on its stem.

## 6. Physical Model Requirements

### 6.1 States & governing equations

| State | Physical meaning | Equation type |
|---|---|---|
| θ_f,i (4 × 8) | Bend angle of finger f, joint i | 2nd-order ODE (Multibody) |
| p_m, p_f (1 + 4) | Manifold and finger-chamber pressures | ODE, mass + energy balance (Simscape Gas) |
| T_m, T_f | Gas temperatures | ODE (adiabatic walls) |
| x_obj (6 DOF) | Object pose and velocity | 6-DOF rigid body (Multibody) |
| Contact states | Penetration and slip per finger-object pair | Algebraic / event (Spatial Contact Force) |
| q_arm (6) | UR5e joints | **Prescribed** (motion-actuated, no dynamics) |
| Regulator integrator | PI state | ODE |

### 6.2 Key assumptions

| Assumption | Justification |
|---|---|
| Finger = 8-joint pseudo-rigid-body chain, one joint per chamber, joint spring K θ + K3 θ³ | Matches Zhu's 8-chamber finger; offline fit to Table 1: RMS 0.050 N, free bend 81/94/104° vs about 75/90/110° |
| Gauge force perpendicular to the fingertip in Zhu's force test | **Assumption**: Zhu do not state the orientation |
| Torque per joint = D·max(p − p0, 0), via Rotational Mechanical Converter (G); chamber volume grows by D per radian | Energy-consistent gas↔mechanics coupling; p0 captures the wall pre-stretch (identified ≈ 13 kPa) |
| Finger length L ≈ 105 mm | **Identified** from Table 1 (not published) |
| All four fingers share one chamber pressure through a manifold | Engineer decision (one regulator), and how commercial soft grippers work |
| Fruits are rigid solids (sphere or ellipsoid) with contact stiffness | Bruising is judged from contact force against per-fruit limits |
| Adiabatic gas, rigid manifold and tubes | Same as the validated soft-arm gas network (S-07/S-08 passed) |
| UR5e follows its commanded joint trajectory exactly | Approved kinematic arm |

### 6.3 Fidelity level
**Chosen: high in the fingers and gas, low (kinematic) in the arm.** Fidelity goes where the published
evidence is (Zhu Tables 1, 3, 4; SMC catalog) and where the teaching point is.

### 6.4 Coordinate frames & sign conventions

| Convention | Definition |
|---|---|
| World | Z up, origin at the UR5e base, metres, radians internally |
| Finger joint angle θ | 0 = straight; positive = bending toward the gripper axis (closing) |
| Finger joint lower limit | θ ≥ 0, a hard stop: an unpressurized PneuNet does not curl outward |
| Pressure | Gauge, Pa; the chambers push only |
| Finger placement | 4 fingers at 90° spacing on the flange, pointing along the tool Z axis |

## 7. Controller Interface Contract

> ⛔ **FROZEN 2026-10-23** by explicit engineer approval ("ok"), after the architecture review. Any change to
> §7.1–7.4 now needs engineer confirmation plus a logged blast radius (`SoftGripperPick.slx` ports, the
> controller, sensors, and the scenarios that use the signal). Not frozen: the value of `Ts_Ctrl` (derived at
> M3).

### 7.1 Plant inputs (u)

| Signal | Sym | Unit | Type | Sample time | Description |
|---|---|---|---|---|---|
| `qArmCmd` | u₁ | rad (6) | double | `Ts_Ctrl` | UR5e joint position commands |
| `pGripCmd` | u₂ | Pa (gauge) | double | `Ts_Ctrl` | Gripper regulator set pressure, [0, p_GripMax = 60 kPa] |

### 7.2 Plant outputs (y, with sensor effects)

| Signal | Sym | Unit | Type | Sample time | Description |
|---|---|---|---|---|---|
| `qArmMeas` | y₁ | rad (6) | double | `Ts_Ctrl` | Joint encoder readings |
| `pGripMeas` | y₂ | Pa | double | `Ts_Ctrl` | Gripper pressure transducer, ±2 % F.S. of 0.1 MPa (ITV101x class) |
| `bendMeas` | y₃ | rad (4) | double | `Ts_Ctrl` | Finger bend sensors (Zhu fingers embed a Flex Sensor 2.2); total bend per finger, with noise. Contact is inferred when a finger stops short of its free bend |

### 7.3 Exogenous inputs (w)

| Signal | Sym | Unit | Source | Description |
|---|---|---|---|---|
| `objPoseInit` | w₁ | m, rad | scenario | Object pose on the table; also given to the controller as the "camera" estimate |
| `objId` | w₂ | — | scenario | Which object: sets shape, mass, contact limits |
| `pSupplyGrip` | w₃ | Pa | parameter / signal | Regulator supply (0.2 MPa nominal), droopable |
| `mu` | w₄ | — | parameter | Finger–object friction (swept) |

### 7.4 Truth outputs (z — validation only, ⛔ never to the controller)

| Signal | Unit | Description |
|---|---|---|
| `fContact` | N (4) | Normal contact force per finger: the crush and slip checks |
| `objPose` | m, rad | True object pose (also drives the 3D replay) |
| `objState` | enum | `OnTable` / `Held` / `Slipped` / `Dropped` / `Placed` / `Crushed` |
| `thetaJoints` | rad (32) | All finger joint angles |
| `pFinger` | Pa (4) | Each finger chamber pressure |

REQ_SOFTARM_067's rule (truth signals must not reach the controller) carries over as a `REQ_GRIP`
requirement and is checked structurally.

## 8. Initialization & Operating Points

### 8.1 Nominal operating point

| State | Initial value | How determined |
|---|---|---|
| θ_f,i | 0 (straight, open) | Unpressurized |
| p_m, p_f | 0 gauge | Vented |
| Object | resting on the table | Static equilibrium (contact settle) |
| q_arm | a home pose above the table | From inverse kinematics at M3 |

### 8.2 Operating range

| Parameter | Min | Nominal | Max | Unit |
|---|---|---|---|---|
| Grip pressure | 0 | object-dependent | 60 | kPa |
| Object mass | 4 | — | 580 | g |
| Object size | TBD (agent) | | TBD | mm |
| μ | TBD (agent) | | | — |

### 8.3 Initialization strategy
Parameters come from `params_softgripper_*.m` functions composed into a struct and injected with
`in.setVariable`. The object settles on the table for a short warm-up before the arm moves.

## 9. Rate & Timing Alignment

| Component | Rate | Type | Notes |
|---|---|---|---|
| Fingers, gas, contact, object | continuous | variable-step implicit (ode23t or ode15s; decided at M2) | Contact stiffness makes it stiff |
| UR5e | continuous, prescribed | motion input | Smoothed joint trajectories |
| Controller | `Ts_Ctrl` | discrete | Derived at M3 from the gripper fill time (12.8 ms predicted) and the finger dynamics |
| Sensors | `Ts_Ctrl` | discrete (ZOH) | |

## 10. Validation Evidence

| Evidence | Available? | Description | Covers |
|---|---|---|---|
| Hardware test data | **Yes** | Zhu 2022 Table 1 (29 points of end force vs p, θ), Table 2 (with rings), Table 3 (grasp force by mode), Table 4 (hold pressure per fruit) | Finger, gripper, task |
| Datasheets | Yes | SMC ITV101x (range, accuracy, supply, flow graph) | Regulator, gas |
| Reference model | Partial | Zhu's analytic Yeoh-based model (equations only) | Bend vs pressure |
| Analytic expectations | Yes | Adiabatic fill (12.8 ms predicted), sealed-chamber compression (S-08 pattern), static force balance | Gas, coupling |
| Standard maneuvers | Partly | RoboSoft 2025 harvesting task (structure, limits) | Task scenarios |

## 11. Reference Sources

| Source | Used for |
|---|---|
| Zhu, Feng, Hua, Wang, Hu, Wang, Su, *Sensors* 22(12):4532 (2022), PMC9230056 | Finger geometry and material, Tables 1–4, flex sensor |
| SMC ITV series catalog (p. 1183, 1186) | Regulator |
| Junge et al., Commun. Eng. 2023 | Raspberry limits (detach, slip) |
| RoboSoft Competition 2025 rules | Task structure |
| Armanini et al., IEEE T-RO 2023 | Pseudo-rigid-body as a valid soft-robot discretization |
| `PneumaticActuationCircuitExample` (MathWorks) | Gas block patterns |
| TBD (agent) | Fruit sizes, bruise limits, friction |

## 12. Open Questions

| # | Question | Options | Decision |
|---|---|---|---|
| 1 | Cubic joint spring implementation | (a) Simscape Driveline Nonlinear Rotational Spring (+1 product), (b) Foundation linear spring + cubic correction torque from the sensed angle | ✅ **(b)**, engineer 2026-10-23: no extra product |
| 2 | Finger rings (Zhu Table 2) as a variant? | (a) no rings only, (b) both variants | ✅ **(a) plain fingers for v1**, engineer 2026-10-23; rings are a future variant |
| 3 | Finger dynamics (damping, mass) are not published | (a) estimate + sweep, (b) derive from the gas fill time | 🟡 M2 |
| 4 | Tolerances `e_FingerRms`, `e_GraspRel`, `e_HoldPress` | — | 🟡 GATE A (engineer) |

## 13. Future Considerations
- **Dynamic UR5e** (Simscape Multibody import with joint dynamics) as an extension.
- **Ring-reinforced fingers** (Zhu Table 2) as a design variant.
- **Stem picking** for the raspberry (detach mechanism, already sourced).
- **Soft continuum arm** as a separate project.

## Appendix A: Related Documents
- `params/fit_zhu_finger_prb.m` — the offline feasibility fit
- `soft-gripper-object-sourcing.md` — object properties and their sources
- `soft-gripper-architecture.md` — architecture spec (this spec's §7 interface is inherited there)
- `soft-gripper-verification.md` — M4 pass/fail per requirement
