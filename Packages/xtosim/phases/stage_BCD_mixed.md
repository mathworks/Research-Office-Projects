# Mixed-Builder Pipeline: Multi-Builder Model Construction

**When to use:** The system has components that naturally map to different builders — e.g., custom ODEs for one part, a physical network for another, data-driven lookup tables for a third, and standard library blocks for a fourth.

**Key difference from other pipelines:** No single builder owns the whole model. Each subsystem is built by the most appropriate builder, then a top-level composer wires them together.

**Execution order:** M1 -> M2 -> M3 -> M4 -> M5 -> M6

---

## Pipeline Comparison

| Single-Builder Path | Mixed-Builder Path | What changes |
|--------------------|--------------------|-------------|
| Phase 1-3: Extract all | **M1**: Extract all (same) | No change |
| Phase 4: Normalize all | **M2**: Classify + route each component | Per-component builder assignment |
| Phase 5: Translate to one format | **M3**: Normalize only ODE-targeted equations | Only equations going to odeBuilder are normalized |
| Phase 6-7: Build with one builder | **M4**: Build each component with its builder | Multiple builders in parallel |
| Phase 11: createHierarchy | **M5**: composeModel (top-level wiring) | Bottom-up composition, not hierarchy from flat |
| Phase 10/12: Validate + Package | **M6**: Same validation + package | No change |

---

## M1: Extract (same as Phase 1-3)

Read the source document. Extract all equations, parameter tables, block diagrams, data curves, and component descriptions. Produce the spec.

No changes from standard Phase 1-3. Read `stage_A_intake.md`.

---

## M2: Classify and Route

**Input:** spec with extracted equations and component descriptions
**Output:** spec with `implementation_plan` — each component assigned to a builder

### M2a: Identify components

For each equation group or subsystem identified in the source, determine:
1. What physical component does it represent?
2. Is it a standard engineering component with a validated library block?
3. What is the source of truth — equations, data tables, or block diagrams?

### M2b: Classify each component

Apply the decision tree from SKILL.md section "B2: Builder Assignment" per component. The canonical tree is there — do not duplicate it here.

### M2b-ii: Decision Path (MANDATORY per component)

After classifying each component, record the decision tree traversal as a structured `decision_path`. This is NOT optional — `validatePlan` will reject any plan missing this field.

For each component, traverse the tree top-to-bottom and record:
- Which branches were evaluated (true/false)
- What evidence was found at each branch (especially `findBlock` results)
- Where the traversal terminated
- If a higher-priority builder was available but rejected: the specific reason why

```matlab
plan.components(i).decision_path = struct( ...
    'is_empirical_data', false, ...         % Branch 1: tables/curves/maps?
    'empirical_evidence', '', ...           % what data was found (or why not)
    'is_standard_block', true, ...          % Branch 2: library block exists?
    'standard_block_name', '6DoF (Euler Angles)', ... % findBlock result
    'findBlock_searched', true, ...         % did you actually call findBlock?
    'use_standard_block', true, ...         % did you select it?
    'reject_standard_reason', '', ...       % if rejected: WHY (cite paper requirement)
    'is_physical_network', false, ...       % Branch 3: bidirectional energy?
    'is_discrete_event', false, ...         % Branch 4: entities/queues?
    'is_pure_logic', false, ...             % Branch 5: FSM/protocol?
    'has_mode_switching', false, ...        % Branch 6: different ODEs per mode?
    'has_tightly_coupled_conditionals', false, ... % Branch 7: coupled nonlinear + if/else?
    'conditional_description', '', ...      % what are the coupled conditionals?
    'builder_selected', 'blocksetBuilder', ...
    'tree_terminal_node', 'Branch 2: standard block exists');
```

**Key enforcement rules:**
- `findBlock_searched` must be `true` before `is_standard_block` can be `false`
- If `is_standard_block == true` but `use_standard_block == false`, `reject_standard_reason` must cite a specific requirement from the source document
- MATLAB Function is only valid when: `is_standard_block == false` AND `has_tightly_coupled_conditionals == true` with specific `conditional_description`
- `tree_terminal_node` must name which branch terminated the traversal

### M2c: Present classification to user for confirmation

**This is an interactive checkpoint.** Present the proposed routing table:

```
=== Component Classification ===
| Component    | Builder            | Reason                              |
|--------------|--------------------|-------------------------------------|
| <CompA>      | odeBuilder         | Custom nonlinear ODEs from paper    |
| <CompB>      | blocksetBuilder    | Standard block exists in library    |
| <CompC>      | lookupTableBuilder | Data table provided                 |
| <CompD>      | simscapeBuilder    | Physical network, bidirectional     |
| ...          | ...                | ...                                 |

Change any? (or confirm to proceed)
```

The user can override any assignment. Common overrides:
- "Use equations for X" → switch from blocksetBuilder to odeBuilder
- "Use Simscape for everything" → switch all to simscapeBuilder
- "Keep it simple" → odeBuilder for all

### M2d: Record implementation decisions (traceability)

For each component, record:

```matlab
spec.components(i).implementation = struct( ...
    'builder', '<builder_name>', ...
    'reason', '<why this builder was chosen>', ...
    'source_equations', {{'<relevant equations from paper>'}}, ...
    'recognized_model', '<name if recognized, empty otherwise>', ...
    'parameter_mapping', struct('<paper_name>','<builder_param_name>', ...), ...
    'alternative', '<what else could work>', ...
    'confidence', 0.9);
```

### M2e: Sketch wiring topology (BEFORE building)

**Critical for Simscape components.** Before any component is built, sketch the inter-component signal flow. For each pair of connected components, identify:
- What signal crosses the boundary?
- Which direction (A→B, B→A, or bidirectional)?
- Is it a Simulink signal or a physical conserving connection?

```
=== Wiring Sketch ===
<CompA>.<output_name>  →  <CompB>.<input_name>   (signal)
<CompB>.<output_name>  →  <CompC>.<input_name>   (signal)
<CompC>.<output_name>  →  <CompA>.<input_name>   (signal)
```

**Why this must happen at M2, not M5:** odeBuilder components can have ports added after the fact (`materializePorts` replaces Constants with Inports). Simscape components CANNOT — controlled sources and sensors must be in the spec at build time. If you defer wiring decisions to M5, Simscape subsystems will be built as closed circuits with no external interfaces, and composition will fail.

This sketch feeds directly into M3d (Simscape specs must include a Controlled Source for each incoming signal, and a Sensor for each outgoing signal).

---

## M3: Normalize (selective)

Only equations assigned to **odeBuilder** need Phase 4-5 normalization (first-order form, state naming, algebraic sorting, `normalizeUnsupported`).

Components assigned to other builders skip normalization:
- `blocksetBuilder` → needs parameter mapping only
- `lookupTableBuilder` → needs data extraction only
- `simscapeBuilder` → needs topology/netlist only (see `stage_C_simscape.md` S1-S2)
- MATLAB Function → needs the function code written

### M3a: Normalize ODE-targeted equations

Standard Phase 4-5 normalization for each odeBuilder component. Read `stage_B_plan.md`.

### M3b: Prepare blockset component specs

For each blocksetBuilder component:
```matlab
blkSpec(i).name = '<ComponentName>';
blkSpec(i).library = '<full/library/path>';       % from findBlock()
blkSpec(i).parameters = struct('<param1>',val1, '<param2>',val2, ...);
blkSpec(i).inputs = {'<input1>', '<input2>', ...};
blkSpec(i).outputs = {'<output1>', '<output2>'};
```

### M3c: Prepare lookup table data

For each lookupTableBuilder component:
```matlab
ltSpec(i).name = '<ComponentName>';
ltSpec(i).breakpoints = {bp1_vector, bp2_vector};  % one per dimension
ltSpec(i).tableData = data_matrix;                  % n-D array
ltSpec(i).inputs = {'<bp1_name>', '<bp2_name>'};
ltSpec(i).outputs = {'<output_name>'};
```

### M3d: Prepare Simscape netlists

For each simscapeBuilder component, follow `stage_C_simscape.md` S1-S2 to produce the spec struct.

**Use the M2e wiring sketch to ensure the spec includes external interfaces:**
- For each **incoming signal** in the wiring sketch → add a Controlled Source (voltage, current, force, etc.) driven by an Inport + Simulink-PS Converter
- For each **outgoing signal** in the wiring sketch → add a Sensor + PS-Simulink Converter → Outport
- The component must still simulate standalone (M4b) — use default constant values at the Inports for isolation testing

Unlike odeBuilder (where `materializePorts` can retrofit interfaces after building), Simscape interfaces must be baked into the spec. A Simscape subsystem built without external ports cannot be opened for composition after the fact.

---

## M4: Build Each Component

Build each component independently using its assigned builder. Components are built as **standalone subsystem models** (not one flat model).

### M4a: Build order

Build in dependency order where possible. Independent components build in parallel.

```matlab
% Each builder produces a standalone model + metadata
[mdl_A, meta_A] = odeBuilder(eqsA, 'CompA', 'Params', paramsA, 'IC', icsA);
[mdl_B, meta_B] = blocksetBuilder(blkSpecB, 'CompB');
[mdl_C, meta_C] = lookupTableBuilder(ltSpecC, 'CompC');
[mdl_D, meta_D] = simscapeBuilder(specD, 'CompD');
```

### M4b: Verify each component in isolation

Each component model must simulate independently with constant inputs:
```matlab
set_param(mdl_i, 'StopTime', '1');
simOut = sim(mdl_i);  % should not error
% Check outputs are in expected range
```

### M4c: Define port contracts

Each component exposes:
- **Inputs:** named inports with expected signal names, units, and ranges
- **Outputs:** named outports with signal names, units, and ranges

Port contracts are the interface specification for composition. Use `buildInterface()`:

```matlab
ports.inputs = struct('name',{'<iface_name1>','<iface_name2>'}, ...
    'type',{'signal','signal'}, 'unit',{'<unit1>','<unit2>'}, ...
    'port_index',{1,2}, 'domain',{'',''}, 'port_id',{'',''}, ...
    'internal_name',{'<builder_block_name1>','<builder_block_name2>'});
ports.outputs = struct('name',{'<iface_name1>','<iface_name2>'}, ...
    'type',{'signal','signal'}, ...
    'unit',{'<unit1>','<unit2>'}, 'port_index',{1,2}, ...
    'domain',{'',''}, 'port_id',{'',''}, ...
    'internal_name',{'<integrator_name1>',''});  % '' = same as .name
iface = buildInterface('<CompName>', '<builder>', mdl, ports);
```

**What IS an interface port (include in `buildInterface`):**
- Signals that flow between components at runtime (forces, velocities, torques, currents, states)
- External command/reference inputs from outside the system
- Measured outputs needed by other components for feedback

**What is NOT an interface port (do NOT include):**
- Parameters set once at initialization (mass, resistance, gain, drag coefficient, etc.)
- Constants that never change during simulation
- Internal states not needed by any other component

Parameters stay as Constant blocks inside each component, driven by InitFcn. Making them into interface ports creates 10+ Inports per subsystem, most dangling unconnected at the top level. **Target: 2-5 interface ports per component** for physical signals only.

**`internal_name` is critical:** When the LLM-chosen interface name differs from the builder's internal block name, set `internal_name` so `materializePorts` can find the correct block to replace/tap:
- **odeBuilder inputs:** `internal_name` = the Constant block name (odeBuilder names Constants after LaTeX variable names)
- **odeBuilder outputs:** `internal_name` = the Integrator block name (odeBuilder appends `1` to state names)
- **blocksetBuilder:** `internal_name` = the SubSystem block name inside the model (e.g., the library block's name)
- **If interface name matches the internal name:** leave `internal_name` empty (`''`)

---

## M5: Compose Top-Level Model

**This is NOT `createHierarchy`.** `createHierarchy` reorganizes a flat odeBuilder model into subsystems. Here we compose independently-built components into a new top-level model.

### M5a: Use composeModel (preferred)

```matlab
% Collect interfaces from all components
interfaces = [meta_A.interface, meta_B.interface, meta_C.interface, meta_D.interface];

% Define wiring plan (LLM decides based on physics)
wiring(1) = struct('from_component','CompA', 'from_port','<out_name>', ...
    'to_component','CompB', 'to_port','<in_name>', 'type','signal');
wiring(2) = struct('from_component','CompB', 'from_port','<out_name>', ...
    'to_component','CompA', 'to_port','<in_name>', 'type','signal');
% ... one entry per inter-component connection

% Compose — handles signal routing, domain converters, solver config
[topMdl, composeMeta] = composeModel(interfaces, wiring, 'system_name', ...
    'TopInputs', {struct('name','<ext_input>','component','CompA','port','<port>')}, ...
    'TopOutputs', {struct('name','<ext_output>','component','CompA','port','<port>')});
```

### M5b: Manual composition (fallback)

If `composeModel` doesn't cover a special case, compose manually:

```matlab
topModel = '<system_name>';
new_system(topModel);
open_system(topModel);

% Copy each component as a subsystem
add_block('built-in/SubSystem', [topModel '/' compName]);
Simulink.BlockDiagram.copyContentsToSubSystem(mdl_i, [topModel '/' compName]);

% Wire subsystems together using port contracts
add_line(topModel, [srcSubsys '/' srcPort], [dstSubsys '/' dstPort], 'autorouting', 'smart');
```

### M5c: Cross-domain boundaries

When connecting signal-flow (odeBuilder) to physical (simscapeBuilder) subsystems, `composeModel` auto-inserts converters:
- **Signal → Physical:** Simulink-PS Converter
- **Physical → Signal:** PS-Simulink Converter
- **Physical → Physical (same domain):** Direct conserving connection

### M5d: Algebraic loop handling

If subsystem A's output feeds B's input, and B's output feeds A's input in the same timestep, insert a unit delay or Algebraic Constraint block at the loop boundary. Use `insertAlgebraicConstraints` where applicable.

### M5e: Add sources and sinks

Add top-level input sources (step, ramp, sine, from-workspace) and output sinks (scopes, ToWorkspace, outports) for the primary scenario.

### M5f: Write InitFcn

Combine all subsystem parameters into a single InitFcn:
```matlab
writeInitFcn(topModel, allParams, allICs, modelId);
```

### M5g: Layout

```matlab
layoutSignalFlow(topModel, subsystemChain, sourceBlocks);
```

---

## M6: Validate and Package (same as Phase 10 + 12)

Standard validation and packaging. Read `stage_E_validate.md` and `stage_F_deliver.md`.

- Minimum 3 tests, at least 1 open-loop
- `evaluateTests` for scoring
- `autoPlotValidation` for signal plots
- `fillReport` for HTML report
- `writeValidationScript` for standalone reproducibility

**Additional for mixed-builder models:**
- Report must include the classification table (which builder for which component)
- Report must include per-component traceability (source equation → recognized model → implementation → parameter mapping)
- Report should show component isolation test results (from M4b)

---

## Gate Checklist: Mixed-Builder Model Complete

```
- [ ] All components classified and user confirmed (M2c)
- [ ] Implementation decisions recorded in spec (M2d)
- [ ] Each component built and verified in isolation (M4b)
- [ ] Port contracts defined for all inter-subsystem signals (M4c)
- [ ] Top-level model wired and simulates without error (M5)
- [ ] No unconnected ports (programmatic audit)
- [ ] InitFcn covers all parameters (standalone play-button works)
- [ ] At least 3 validation tests, 1 open-loop (M6)
- [ ] Classification table in report
- [ ] Per-component traceability in report
- [ ] run_validation.m reproduces all tests standalone
```

---

## When NOT to Use Mixed-Builder Pipeline

- **Single-domain ODE system** (one equation block, same builder everywhere) → use Simple or Full pipeline
- **Pure Simscape model** (entire system is a physical network) → use Simscape pipeline
- **Small model** (< 6 equations, one subsystem) → overkill, use Simple pipeline
- **Exact paper reproduction** where the paper presents all equations uniformly → use Full pipeline with odeBuilder

## When TO Use Mixed-Builder Pipeline

- Systems where components naturally span multiple modeling paradigms
- Models mixing validated library blocks with custom research equations
- Systems with empirical data (maps, curves) alongside differential equations
- Mechatronic/multi-physics systems crossing domain boundaries
- Large models (10+ subsystems) where different fidelity levels coexist

---

## Composition Patterns

### Pattern 1: Signal-Flow Composition (most common)

Subsystems exchange Simulink signals (forces, velocities, angles, etc.):
```
odeBuilder subsystem <-> blocksetBuilder subsystem <-> lookupTableBuilder subsystem
     (via Inport/Outport signal connections)
```

All subsystems live in the same Simulink model, connected by signal lines.

### Pattern 2: Physical Network Composition

Some subsystems connect via Simscape conserving ports:
```
simscapeBuilder subsystem <-> simscapeBuilder subsystem
     (via physical connection lines — across/through variables)
```

Simscape subsystems connect to signal-flow subsystems via **PS-Simulink** and **Simulink-PS** converters at the boundary.

### Pattern 3: Hybrid Composition

Mix of signal-flow and physical network:
```
odeBuilder (controller) -> Simulink-PS -> simscapeBuilder (plant) -> PS-Simulink -> odeBuilder (observer)
```

The boundary between signal-flow and physical-network domains requires explicit converter blocks.

---

## Role of createHierarchy in Mixed Pipeline

`createHierarchy` is used **within** a single odeBuilder component to organize its internal structure when that component has many states (e.g., 12+ DOF). It groups blocks into named subsystems INSIDE the component.

It is NOT used for top-level composition of heterogeneous subsystems. That's `composeModel`'s job.
