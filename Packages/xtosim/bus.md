# Simulink Bus Objects — When and How to Use

## When to Use

**Subsystem boundary bus guideline:**
Use typed buses when ports form a **semantically meaningful group** — signals that belong to the same physical domain or represent a coherent interface (e.g., all flux states, all aerodynamic coefficients, a 6-DOF state vector). Port count alone does not justify a bus.

**Advisory threshold:** `validatePlan` issues a warning (not a block) when a component has **more than 6 Inports** OR **more than 6 Outports** without a bus. This is advisory — the LLM decides whether the signals form a meaningful bus or are better left as named scalars.

| Ports at boundary | Action |
|---|---|
| ≤6 Inports AND ≤6 Outports | Scalar lines OK |
| >6 Inports OR >6 Outports | Consider bus IF signals form a meaningful group |

**When to use a bus (semantic criteria):**
- Signals share a physical domain (all electrical quantities, all mechanical states)
- Signals are produced/consumed as a coherent set (a state vector, a force/moment group)
- Multiple consumers need the same bundle (shared interface)
- The bus name would be meaningful to an engineer reading the model

**When NOT to use a bus:**
- Signals are heterogeneous (mixed domains, unrelated quantities)
- Each signal is individually meaningful at the boundary (e.g., torque, speed, voltage from different subsystems)
- Adding a bus would just add Select/Assign complexity without clarity

Examples:
- Dynamics_6DOF (6 in, 12 out) → `ForcesAndMoments` bus input + `AircraftState` bus output (**meaningful**: coherent state/force vectors)
- AeroModel (12 in, 6 out) → bus on both sides (**meaningful**: aero coefficients are a natural group)
- FluxLinkages (3 in, 4 out) → scalar OK (**4 outputs are individually named flux states, no natural grouping beyond "flux"**)
- CurrentAlgebra (4 in, 4 out) → scalar OK (**inputs come from different subsystems, outputs individually consumed**)
- FlightControl (3 in, 3 out) → scalar OK
- Engine (3 in, 2 out) → scalar OK

**This rule applies to:**
- Components built by any builder (odeBuilder, blocksetBuilder, stateflowBuilder, etc.)
- Wrapped blockset blocks (use `wrapBlocksetBlock` which already produces bus ports)
- Any subsystem copied into composition

**Never work around this rule** with Mux/Demux at the subsystem boundary. If a blockset block has vector ports, wrap it with `wrapBlocksetBlock` to get typed bus interfaces. If an odeBuilder component has many outputs, use `internalizeBusOutput` to bundle them into a bus at the subsystem boundary.

**Legacy threshold (Phase 11 — single-model hierarchy):**
For single-model reorganization (not multi-builder composition), the softer threshold still applies:
- 4+ subsystems in the model
- 10+ inter-subsystem signal lines (count before grouping)
- Signals group naturally by producer (3+ signals per bus)

**User override:** Respect `spec.use_bus = true/false` if set.

**Opt-out for simple models (`plan.busMode = 'none'`):**
When a model uses direct scalar wiring and the architecture doesn't benefit from typed buses (e.g., single-component odeBuilder models reorganized via `createHierarchy`, or models where all subsystems have ≤3 ports but the threshold is borderline), set:
```matlab
plan.busMode = 'none';
```
This skips the bus enforcement check in `validatePlan`. Use this when:
- All inter-subsystem signals are already named scalars (not vectors)
- The model is a teaching/demo model where bus complexity would obscure the physics
- The paper's block diagram shows direct point-to-point wiring

Do NOT use `busMode = 'none'` when subsystems genuinely have >6 ports with semantically coherent signal groups that would benefit from bus bundling.

## Core Concept

A Bus Object is a typed schema that bundles multiple signals into one line:
- **Define** the schema (names, units, dimensions)
- **Create** at the producing subsystem (Bus Creator)
- **Select** at the consuming subsystem (Bus Selector)

Result: N individual lines become 1 typed bus line per producer group.

## Implementation Pattern

### Step 1: Group signals by producer

Each bus corresponds to ONE producing subsystem. Signals that originate from the same subsystem go in the same bus.

Example (full vehicle model):
```
TireForceBus    = {Fyf, Fyr, alphaf, alphar}   % from TireForces
VehicleStateBus = {vy, r}                        % from LateralYawDynamics
LoadBus         = {ay, dFzf, dFzr}              % from LoadTransfer
```

Do NOT mix signals from different producers in one bus.

### Step 2: Define Bus Objects in InitFcn

```matlab
% In model InitFcn callback
TireForceBus = Simulink.Bus;
TireForceBus.Elements = Simulink.BusElement.empty(0,4);
TireForceBus.Elements(1) = Simulink.BusElement;
TireForceBus.Elements(1).Name = 'Fyf';
TireForceBus.Elements(1).Unit = 'N';
TireForceBus.Elements(2) = Simulink.BusElement;
TireForceBus.Elements(2).Name = 'Fyr';
TireForceBus.Elements(2).Unit = 'N';
% ... etc for each element
```

Rules:
- One `Simulink.BusElement` per signal
- `.Name` must match the signal name exactly
- `.Unit` preserves physical units (optional but recommended)
- Place definitions in model InitFcn (standalone model, no external scripts needed)

### Step 3: Add Bus Creator inside producing subsystem

```matlab
sub = [mdl '/TireForces'];

% Add Bus Creator
bc = add_block('simulink/Signal Routing/Bus Creator', [sub '/BusCreator']);
set_param(bc, 'Inputs', '4');  % match element count
set_param(bc, 'OutDataTypeStr', 'Bus: TireForceBus');

% Connect internal signals to Bus Creator inputs
% (port order must match BusElement order in the definition)
add_line(sub, 'Fyf_source/1', 'BusCreator/1');
add_line(sub, 'Fyr_source/1', 'BusCreator/2');
add_line(sub, 'alphaf_source/1', 'BusCreator/3');
add_line(sub, 'alphar_source/1', 'BusCreator/4');

% Connect Bus Creator output to subsystem Outport
add_line(sub, 'BusCreator/1', 'Out1/1');

% Set outport data type
outport = [sub '/Out1'];
set_param(outport, 'OutDataTypeStr', 'Bus: TireForceBus');
```

### Step 4: Add Bus Selector inside consuming subsystem

```matlab
sub = [mdl '/LateralYawDynamics'];

% Add Bus Selector
bs = add_block('simulink/Signal Routing/Bus Selector', [sub '/BusSelector']);
set_param(bs, 'OutputSignals', 'Fyf,Fyr');  % comma-separated, only what's needed

% Connect from inport to Bus Selector
add_line(sub, 'In1/1', 'BusSelector/1');

% Connect Bus Selector outputs to internal blocks
add_line(sub, 'BusSelector/1', 'Fyf_consumer/1');  % Fyf
add_line(sub, 'BusSelector/2', 'Fyr_consumer/1');  % Fyr
```

### Step 5: Name signal lines (CRITICAL)

Lines feeding Bus Creator inputs MUST be named to match BusElement names:

```matlab
% Find the line handle
lh = get_param([sub '/BusCreator'], 'LineHandles');
set_param(lh.Inport(1), 'Name', 'Fyf');
set_param(lh.Inport(2), 'Name', 'Fyr');
set_param(lh.Inport(3), 'Name', 'alphaf');
set_param(lh.Inport(4), 'Name', 'alphar');
```

**If you skip this step, Bus Creator will show `signal1`, `signal2` etc. and type checking will fail.**

### Step 6: Name bus lines at top level (cosmetic but recommended)

```matlab
% After wiring at top level, name the bus line
lh = get_param([mdl '/TireForces'], 'LineHandles');
set_param(lh.Outport(1), 'Name', 'TireForceBus');
```

## Pipeline Integration (Phase 11)

Bus Objects are added AFTER `createHierarchy` completes:

1. `createHierarchy(mdl, c, m, cellEq, subs)` — creates subsystems with individual ports
2. Count inter-subsystem lines; if threshold met, proceed with buses
3. Group signals by producer subsystem
4. For each group with 3+ signals:
   a. Define Bus Object (add to InitFcn)
   b. Inside producer: add Bus Creator, wire internal signals, replace multiple outports with single bus outport
   c. Inside each consumer: add Bus Selector at inport, wire to internal blocks
   d. At top level: delete old individual lines, wire single bus line
5. Name all signal lines (Step 5 above)
6. Delete unused individual outports/inports

## Common Pitfalls

| Pitfall | Fix |
|---------|-----|
| Unnamed signals → `signal1` errors | Always `set_param(line, 'Name', ...)` on Bus Creator inputs |
| Port order mismatch | Bus Creator port order must match `BusElement` array order |
| Bus definition not in workspace | Put in InitFcn, not a separate script |
| Fan-out (same bus to 2+ consumers) | One bus line can branch — just connect to multiple inports |
| Adding a signal later | Must update: BusElement array + Bus Creator ports + all Bus Selectors that need it |
| Debugging | Can't probe inside bus at top level; add Bus Selector or use Signal Logging inside subsystem |

## Decision Record

When buses are applied, document in the spec:
```matlab
spec.use_bus = true;
spec.buses = {
    struct('name','TireForceBus', 'producer','TireForces', 'elements',{{'Fyf','Fyr','alphaf','alphar'}})
    struct('name','VehicleStateBus', 'producer','LateralYawDynamics', 'elements',{{'vy','r'}})
};
```

## Example: Before vs After

**Before (16 individual lines):**
```
TireForces ──Fyf──> LateralYawDynamics
TireForces ──Fyr──> LateralYawDynamics
TireForces ──Fyf──> LoadTransfer
TireForces ──Fyr──> LoadTransfer
TireForces ──alphaf──> [logging]
TireForces ──alphar──> [logging]
LateralYawDynamics ──vy──> TireForces
LateralYawDynamics ──r──> TireForces
LateralYawDynamics ──vy──> PathKinematics
LateralYawDynamics ──r──> PathKinematics
LoadTransfer ──ay──> RollDynamics
LoadTransfer ──dFzf──> [logging]
LoadTransfer ──dFzr──> [logging]
... (16 total)
```

**After (4 lines: 1 scalar + 3 buses):**
```
Step_delta_f ──delta_f──> TireForces
TireForces ══TireForceBus══> LateralYawDynamics, LoadTransfer
LateralYawDynamics ══VehicleStateBus══> TireForces, PathKinematics
LoadTransfer ══LoadBus══> RollDynamics
```
