# Soft gripper on a UR5e — Architecture spec

## Status: ✅ Reviewed by the engineer 2026-10-23 · ⛔ parent §7 FROZEN the same day
**Author:** agent, following the `specifying-plant-models` skill (Step 5 template).
**Parent:** `soft-gripper-system.md` (reviewed 2026-10-23). Its **§7 interface is inherited here and not
restated**. It freezes after this review.

---

## 1. Overview

One Simulink model, **`SoftGripperPick.slx`**, holds the plant and the controller. The plant is
**entirely Simscape**:
- **Simscape Multibody:** the UR5e, imported from its URDF; the flange; four 8-joint fingers; the object;
  the table and tray.
- **Simscape Foundation Gas:** one regulated supply, a manifold, and four finger chambers.
- **Simscape Foundation Mechanical:** the hinge springs, plus the gas-to-hinge coupling through Rotational
  Multibody Interfaces.

The controller (M3) is Simulink plus Stateflow at `Ts_Ctrl`. A separate optional viewer replays runs in
Unreal through the **Simulation 3D Robot** block.

**The architectural idea worth stating once:** each finger hinge is driven *physically* by the gas. A
Rotational Mechanical Converter (G) on the finger's chamber turns pressure into hinge torque, and hinge
rotation into chamber volume. The finger "inflates" in Simscape, not in a formula.

## 2. Goals, Non-Goals & Constraints

### 2.1 Design goals
| ID | Goal |
|---|---|
| G1 | The finger is **one reusable subsystem**, parameterized by joint count `N_Joint` (8), length, stiffness and displacement, so readers can swap in their own finger |
| G2 | Every physical domain is Simscape (Gas, Mechanical, Multibody); Simulink only for control, sensing and the cubic spring correction |
| G3 | Every parameter is a `P.*` field from `params_softgripper_*.m`, with its source (identified, datasheet or estimate) |
| G4 | Truth signals (`fContact`, `objPose`, `objState`, …) structurally cannot reach the controller |
| G5 | The viewer is optional: no check or requirement needs Unreal |

### 2.2 Non-goals
| ID | Non-goal | Rationale |
|---|---|---|
| NG1 | UR5e dynamics | Approved kinematic arm (system spec §4) |
| NG2 | Finger FEM / continuum hyperelasticity | The 8-joint chain with cubic springs fits Zhu Table 1 at 0.050 N RMS |
| NG3 | Ring-reinforced fingers | Engineer decision: plain fingers in v1 |
| NG4 | Object deformation and bruise mechanics | Bruising is judged from the contact force against per-object limits |

### 2.3 Constraints
| # | Constraint | Consequence |
|---|---|---|
| C1 | **No extra product for the cubic spring** (engineer) | Foundation linear spring + a Simulink cubic correction torque from the sensed joint angle |
| C2 | Products: MATLAB, Simulink, Simscape, Simscape Multibody, Stateflow, Robotics System Toolbox (UR5e model, IK); optional Simulink 3D Animation (viewer), Requirements Toolbox, and **Optimization Toolbox (only to re-run the finger identification)** | ✅ Engineer decision 2026-10-23: **keep the Optimization Toolbox version** of `fit_zhu_finger_prb.m` (`lsqnonlin`, `fsolve`). The identified values ship in the params file, so the model itself runs without it |
| C3 | Finite StopTime everywhere | ⛔ toolkit rule |
| C4 | Interface = system spec §7 | Frozen after this review |

---

## 3. Architecture

### 3.1 Subsystem diagram

```
 SoftGripperPick.slx
 ┌──────────────────────────────── Controller (Ts_Ctrl, M3) ─────────────────────────────────┐
 │  PickTask (Stateflow) ──► ArmPlanner (IK + trajectory) ──► qArmCmd (u1)                    │
 │        └──────────────► GripController ─────────────────► pGripCmd (u2)                    │
 │  ◄── y: qArmMeas, pGripMeas, bendMeas      ◄── w: objPoseInit (camera estimate), objId      │
 └──────────────────────────────────────────────────────────────────────────────────────────┘
          │ u                                                     ▲ y
          ▼                                                       │
 ┌──────────────────────────────── Plant (continuous, Simscape) ─────────────────────────────┐
 │  UR5e (Multibody, smimport) ── joints motion-driven by smoothed qArmCmd ──► tool frame       │
 │      └─ Flange (Multibody) ── 4 × SoftFinger ──┐                                            │
 │                                                │ hinge torque / rotation (Rotational MB IF)  │
 │  GripperGas (Gas): PressReg PI ─► supply / exhaust valves ─► manifold ─► 4 finger chambers ──┘│
 │  Object (Multibody: sphere or ellipsoid by objId, 6-DOF joint) ◄─ Spatial Contact Force ─► finger links, table, tray │
 │  SensorSet: qArmMeas, pGripMeas (±2 % F.S.), bendMeas (Σθ per finger + noise)                │
 │  TruthBus (z): fContact, objPose, objState (ObjectMonitor), thetaJoints, pFinger            │
 └──────────────────────────────────────────────────────────────────────────────────────────┘
 Viewer (optional): SoftGripperScene3D.slx — Simulation 3D Robot (UR5e) + actors, driven by logged runs
```

### 3.2 Component catalog

| Component | Implementation | Domain | States | Port interface | Depends on |
|---|---|---|---|---|---|
| **UR5e** | Subsystem from `smimport('universalUR5e.urdf')`; 6 revolute joints, motion = input | Multibody | prescribed | in `qArm` (6) → frame `Tool` | Robotics System Toolbox URDF |
| **Flange** | Brick solid, 4 finger mounts at 90° | Multibody | — | frame in ← `Tool`; 4 frames out | `P.grip.flange` |
| **SoftFinger** (× 4, one library-style subsystem) | 8 × (Revolute Joint + link solid); per joint a Rotational Multibody Interface → (Foundation Rotational Spring K + Rotational Damper + gas converter) | Multibody + Mechanical + Gas | θ_i, ω_i (8 + 8) | frame base; gas port `A`; PS out θ (8); PS in τ_cubic (8) | `P.grip.finger` |
| **CubicSpring** (per finger) | Simulink: τ_i = −K3·θ_i³ from sensed θ_i | Signal | 0 | in θ (8) → out τ (8) | `P.grip.finger.K3` |
| **GripperGas** | ITV101x-class supply (Reservoir, 0.2 MPa) → supply + exhaust Local Restriction (G) → manifold Constant Volume Chamber → 4 tube restrictions → finger chamber nodes (each a Constant Volume Chamber for the dead volume, plus the 8 converters' swept volume) | Gas | p, T at manifold + 4 nodes | in `aSup`, `aExh`, `pSupply`; out `pManifold` | `P.grip.gas`, block calibration `k_BlockCapacity` |
| **PressReg** | PI on `pGripCmd − pManifold` (signed area out, clamping anti-windup); the soft-arm pattern | Signal | 1 | in `pGripCmd`, `pMeas` → `aValve` | `P.grip.reg` |
| **Object** | Variant by `objId`: Spherical or Ellipsoidal Solid, mass from Zhu, 6-DOF joint | Multibody | 12 | frames | `P.obj(objId)` |
| **Contacts** | Spatial Contact Force: 8 links × 4 fingers vs object, object vs table, object vs tray | Multibody | event | sensed normal force, penetration | `P.contact` |
| **ObjectMonitor** | Stateflow: `OnTable` / `Held` / `Slipped` / `Dropped` / `Placed` / `Crushed` from contact forces and pose | Signal | — | z only | `P.obj` limits |
| **SensorSet** | ZOH at `Ts_Ctrl`, quantization, noise | Signal | 0 | → y | `P.sens` |
| **Controller** | PickTask, ArmPlanner, GripController | Signal / Stateflow | M3 | u ← y, w | M3 |

### 3.3 Signal flow
u → UR5e joint motion and PressReg → gas → hinge torques → finger bend → contact forces on the object → object
motion. Sensors read joint angles, manifold pressure and finger bends; truth signals go only to the logs.
**The two-way coupling is closed physically**: a contact that blocks a finger stops its hinges, so the
chamber volume stops growing and the pressure trace changes. That is how real soft grippers sense a grasp.

## 4. Subsystem details

### 4.1 SoftFinger

| Dir | Port | Signal | Unit | Description |
|---|---|---|---|---|
| In | frame B | base | — | rigid mount on the flange |
| In | gas A | finger chamber | Pa, K | connects to its tube from the manifold |
| In | PS | `tauCubic` (8) | N·m | cubic correction torque per hinge |
| Out | PS | `theta` (8) | rad | hinge angles (truth; Σθ also feeds the bend sensor) |
| Out | frame F | tip | — | fingertip, for contact geometry |

**Behaviour.** Each hinge i: rotational network = gas converter (torque (p − p_atm)·D, volume D per rad) +
linear spring K with equilibrium offset θ_eq = −D·p0/K (wall pre-stretch, identified p0) + damper. It is
coupled to the Multibody Revolute Joint by a Rotational Multibody Interface. The joint lower limit θ = 0 is a
hard stop (a PneuNet does not curl outward). Each link solid is a rounded brick (L/N long) carrying mass
`m_Finger/N` and contact geometry.

✅ **Verified from the block documentation (2026-10-23).** The Rotational Multibody Interface couples a
Simscape rotational network to a **revolute** joint through the joint's own ports:
- **w** = velocity sensing and **t** = torque actuation, enabled on the joint, same DOF.
- The converter's volume depends on the hinge angle, so also enable the joint's position sensing **q**. Then
  set the gas converter's **Interface rotation = "Provide input signal from Multibody joint"** and wire **q**.

Doc caveat: put **no inertia on the Simscape side**. Spring, damper and converter only; masses stay in the
Multibody link solids. (A fallback was planned in case this failed; it is no longer needed.)

### 4.2 GripperGas
The soft-arm `ChamberGas` + `PressReg` pattern, re-parameterized for the ITV101x:
- 0.2 MPa supply, 0.1 MPa F.S., ±2 % F.S.
- Supply and exhaust valves `a_GripMax` = 3.2 mm², derived from C via ISO 6358 and block-calibrated.
- One manifold, four tubes, four finger nodes.

## 5. Equations of motion

Derivations are not reproduced (skill rule). The governing relations and where they come from:

| Element | Relation | Source |
|---|---|---|
| Hinge torque balance | K(θ_i − θ_eq) + K3 θ_i³ + c θ̇_i = D (p − p_atm) + τ_contact,i + τ_inertial | PRB discretization (Armanini et al. T-RO 2023); identified offline from Zhu Table 1 (`fit_zhu_finger_prb.m`) |
| Gas chamber | Foundation Gas mass and energy balance; volume V0 + D Σθ | Simscape Foundation documentation |
| Valve | Local Restriction (G), momentum-balance orifice | Simscape; calibrated (k = 1.1027) |
| Contact | Spatial Contact Force: smoothed penalty normal force, Coulomb friction with critical velocity | Simscape Multibody documentation |

**Identified parameters (offline fit, cubic spring, 29 points):**
- D = 1.594e-6 m³/rad (N·m/Pa) per hinge. This implies **2.9 mL of chamber growth at full bend**, an
  independent physical cross-check.
- K = 0.104 N·m/rad, K3 = 4.45 N·m/rad³, p0 = 13.0 kPa, L = 105 mm.
- Fit RMS 0.050 N, max 0.127 N.
- Damping and finger mass are **not identified** (no dynamic data): estimate + sweep (system spec Q3).

## 6. Nonlinearities & constraints

| Nonlinearity | Type | Location | Parameters | Physical basis |
|---|---|---|---|---|
| Stiffening hinge | cubic spring | SoftFinger + CubicSpring | K3 | hyperelastic silicone (Zhu use a Yeoh model) |
| Wall pre-stretch | spring offset | SoftFinger | θ_eq = −D p0/K | no bend below about 13 kPa |
| Hinge lower limit | hard stop | SoftFinger | θ ≥ 0 | a PneuNet does not curl outward |
| Contact | penalty + friction | Contacts | k, c, μ per object | rigid object vs silicone link |
| Valve choking | orifice | GripperGas | a_GripMax | ITV101x flow capacity |
| Pressure limits | saturation | PressReg | [0, 60 kPa] | finger rating |

## 7. Cross-cutting concerns

### 7.1 Numerical considerations
| Concern | Approach |
|---|---|
| Solver | **ode15s or ode23t with MaxStep bounded**, decided at M2 by a tolerance-sensitivity run. Contact plus gas is stiff |
| Stiffness | Contact stiffness is the stiffest element; the finger-fruit combined stiffness is about half the fruit's (sourcing note). Use the transition-region width to smooth contact onset |
| Algebraic loops | Simscape PS outputs read as feedthrough. Break them with a state (the soft-arm lesson): a sensor lag in PressReg, and the cubic torque computed from the sensed angle (a state), so no loop |
| Zero crossings | Contact start/end detection on; hard stops at θ = 0 |
| Model size | 32 hinge DOF + 6 object + 5 gas nodes; 34 contact pairs. Bounding all links against the object is the expected cost driver |

### 7.2 Parameter management
`params_softgripper_finger.m` (identified + estimates), `_gas.m` (ITV101x, reusing the soft-arm chain),
`_objects.m` (from `soft-gripper-object-sourcing.md`), `_ctrl.m` (M3). They are composed into `P` and set
with `in.setVariable`. No literals in dialogs.

### 7.3 Simulation performance
- **Target:** one object pick (≤ 15 s simulated) in ≤ 2 min wall time, to be measured at M2.
- **Bottleneck:** contacts. Mitigation: contact only on the links that can touch, and a variant to use the
  fingertip links only for fingertip grasps.

## 8. Uncertainty & sensitivity hooks

| Parameter | Nominal | Range | Why |
|---|---|---|---|
| μ (object) | 0.8–1.4 | 0.5 – 1.5 | Slip is the binding failure; pear, mango and raspberry are unsourced |
| Object contact stiffness | per object | ×0.5 – ×2 | Estimated by Hertz for most objects |
| Finger damping c | estimate | ×0.3 – ×3 | Not published |
| Finger mass | 15 g [EST] | 10 – 20 g | Not published |
| D, K, K3, p0 | fit | fit confidence intervals | Identification uncertainty |
| Finger dead volume V0 | estimate | 5 – 15 mL | Not published (the fit gives only the swept part, 2.9 mL) |
| Regulator speed | derived | ×0.1 – ×1 | Same what-if as the soft arm |

## 9. Key decisions

| # | Decision | Options | Choice | Rationale |
|---|---|---|---|---|
| 1 | Finger discretization | PCC continuum (signal domain) / PRB chain in Multibody | **PRB, 8 joints** | One joint per chamber; native contact; Simscape-only (engineer: keep Simscape) |
| 2 | Gas↔finger coupling | formula torque / physical converter | **Rotational Mechanical Converter (G)** per hinge | Energy-consistent and two-way; identified D *is* its displacement |
| 3 | Cubic spring | Driveline block / Foundation spring + Simulink correction | **Foundation + correction** | Engineer: no extra product |
| 4 | Objects | all six in one model / one variant per run | **One Object with a shape variant, `objId` per run** | Smaller model; sweeps by `SimulationInput` |
| 5 | Arm | dynamic / kinematic | **Kinematic** (motion-driven joints) | Engineer-approved |
| 6 | Visualization | in-loop Sim3D / separate replay | **Separate replay model** with Simulation 3D Robot | Checks never need Unreal (G5) |

## 10. Known limitations & deferred items

| Item | Description | Why acceptable |
|---|---|---|
| Gauge orientation assumed | Zhu's force-test geometry is not stated | Labelled; affects only the identified values, which Tables 3 and 4 then check independently |
| Finger dynamics unidentified | Damping and mass have no data | Grasping is quasi-static over a 15 s pick; swept |
| Bruise limits for pear, mango, raspberry estimated | No published thresholds | Labelled estimates; engineer confirms at GATE A |
| Rigid objects | No fruit deformation beyond contact stiffness | Bruising judged by force |

### Review items for 🛑 GATE B
1. ~~Rotational Multibody Interface on a Revolute Joint~~ ✅ verified from the docs (w/t/q ports, Interface
   rotation from the joint). Confirm by `model_check` at the first build.
2. Contact count vs speed (34 pairs).
3. ~~Optimization Toolbox in the fit script~~ ✅ keep it (engineer 2026-10-23); optional product, needed only
   to re-identify.
4. ~~Bend sensor as a measurement~~ ✅ accepted (engineer 2026-10-23); Zhu's fingers embed a Flex Sensor 2.2.
5. Solver choice from the tolerance-sensitivity run.

## Appendix A: Related documents
`soft-gripper-system.md` · `soft-gripper-object-sourcing.md` · `soft-gripper-verification.md` ·
`params/fit_zhu_finger_prb.m`.
