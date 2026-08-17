# Phases 4-5: Normalize Equations and Convert to Build-Tool Format

**Requires:** `rules.md`, `builders.md` (recognition table for B2 routing)
**Inputs:** spec JSON with `equations_raw_latex` (status: `extracted`)
**Outputs:** spec JSON with `equations_builder` populated, pipeline variant locked (status: `translated`)
**Next stage:** Stage C (Read `stage_C_build.md`)

---

## Phase 4: Normalize Equations

Normalize equations before conversion to build-tool format. See [builders.md](../builders.md) for full normalization rules.

### Key rules

1. **Convert higher-order to first-order form** -- introduce state variables (x1, x2, ...) for each derivative
2. **Explicitly name states** -- use canonical names, maintain mapping to source symbols
3. **Separate categories** -- keep state equations, output equations, algebraic constraints, and events/resets separate
4. **Preserve events** -- condition and reset as separate entries
5. **Do not fabricate equations** -- record ambiguities instead

6. **Handle unsupported math functions** -- odeBuilder supports: `\sin`, `\cos`, `\tan`, `\exp`, `\log`, `\abs`, and arithmetic (`+`, `-`, `*`, `/`, `^`). Any other function is treated as a variable name, creating a dead Constant block.

   **Use `normalizeUnsupported` (MANDATORY at Phase 5d, before building):**
   ```matlab
   [cleanEq, fcnBlocks] = normalizeUnsupported(eqStr);
   % cleanEq: all Category A rewritten, Category B replaced with placeholder vars
   % fcnBlocks: struct array of Fcn block specs for Phase 8
   ```
   This function programmatically handles both categories — see `builders.md` for the full Category A/B tables. The `fcnBlocks` output feeds directly into `addProgrammaticBlocks` in Phase 8.

7. **Choose equations to prevent dead signals (CRITICAL)** -- This is the primary defense against dead subsystems, unused outputs, and duplicate ports. These problems originate at equation selection, not at hierarchy creation.
   - **Before choosing equation forms, trace the signal chain.** For each planned subsystem, ask: "Does every output of this subsystem feed a downstream consumer?"
   - **Lumped/substituted forms for ODEs** when they reduce input count (fewer Constant blocks, simpler wiring).
   - **Original forms for algebraic outputs** when they keep intermediate variables active as subsystem boundary signals.
   - **Original forms for downstream consumers** when the downstream equation references the intermediate variable.
   - **Never inline away a variable that a planned subsystem needs as an output.** If Fslf is a boundary signal between Suspension and Handling, keep `Fslf = kf*(Zwlf - Zs)` as a separate algebraic equation even if it could be substituted into the ODE.
   - **Rule of thumb:** If inlining removes an intermediate variable from the signal path (making a subsystem dead), use the original form. If inlining reduces external inputs without killing a subsystem, use the lumped form.
   - **Verify the full chain before building:** Write out the planned signal flow and confirm every arrow represents a real variable dependency.
   - Never fabricate new equation forms not present in the source.

8. **Check variable names against MATLAB built-ins:**
   ```matlab
   names = {'cd', 'Rs', 'Lm', 'wr', 'mean', 'length'};
   for i = 1:numel(names)
       w = which(names{i});
       if ~isempty(w) && ~contains(w, 'variable')
           fprintf('WARNING: "%s" collides with %s -- rename it\n', names{i}, w);
       end
   end
   ```
   Common collisions: `cd`, `mean`, `length`, `alpha`, `beta`, `gamma`, `i`, `j`. See builders.md "Variable Naming Rules" for the full collision list.

### Nondimensional and polynomial model pitfalls

These apply to **any** system where coefficients are nondimensionalized by a state variable that can approach zero (ships/U, aircraft/V, machines/omega, reactors/concentration).

1. **Check the nondimensionalization basis per parameter.** Different tables in the same paper may use different references (draft vs length, rated vs instantaneous, phase vs line voltage). Always verify what each coefficient is divided by. Using the wrong basis causes orders-of-magnitude errors.

2. **State-variable denominators create singularities.** If equations normalize by a state (e.g., `u' = u/U`, `U = sqrt(u^2+v^2)`), add a floor for nondimensionalization only: `Und = max(U, 0.45*U0)`. Apply it to the normalization denominator, not to the state itself.

3. **Clamp selectively, not globally.** When polynomial terms diverge at extreme nondimensional values, find which terms create positive feedback (diverge) vs which are self-damping (stabilize). Only clamp the divergent terms. Blanket saturation kills the physics -- the model will run but give wrong results.

4. **Perturbation derivatives != absolute values.** A coefficient from linearization (d2F/dx2) is not the same as the full nonlinear term at equilibrium. For resistance/drag terms, compute the equilibrium force balance at the design operating point.

Record any nondimensionalization assumptions in `spec.assumptions`.

Update `equations_normalized_latex` in the spec JSON. Do NOT write a separate `equations.tex` file.

Update spec status to `normalized`.

### Phase 4 exit: Pipeline decisions (LLM-driven)

At this point you have read the entire paper, extracted all equations, parameters, block diagrams, and validation data. You make the pipeline decisions based on what you learned. Record each decision with a one-line justification in `spec.pipeline_decisions`.

**Decision 1: Pipeline variant** (record in `spec.pipeline`):

- `'simple'` -- if ALL 6 criteria are met (<=5 states, 0 algebraic, 0 programmatic, 1 section, not CPS, 0 external feedback loops)
- `'full'` -- otherwise (including CPS/hybrid, which is always full)

**Decision 2: Subsystem validation** (record in `spec.incremental_validation`):

- `true` -- DEFAULT when `spec.subsystem_tests` is non-empty (paper provides intermediate data)
- `false` -- when `spec.subsystem_tests` is empty (paper provides no intermediate data)

If `true`, Phase 7b will run subsystem validation before hierarchy/controller build.

**Decision 3: Controller strategy** (record in `spec.controller_strategy`):

- `'replicate'` -- DEFAULT when paper provides block diagrams or explicit controller equations. Build the paper's exact structure (first-order lag, integrator chain, look-up table — whatever is shown).
- `'derive'` -- Only when paper describes control objectives but not implementation. You design the controller.

**Defaults apply unless you explicitly opt out with justification:**
- DEFAULT: If paper has block diagrams for controllers → `replicate`
- DEFAULT: If model has feedback (Phase 8 components) AND paper provides subsystem data → `incremental_validation: true`
- OPT-OUT: requires one-line justification recorded in `spec.pipeline_decisions`

**Decision 4: Component filtering** (record in `spec.components[i].status`):

For each block identified from the paper's block diagram, classify it:

| Category | Test | Action |
|----------|------|--------|
| **Plant dynamics** | Has states, OR algebraic outputs consumed within the feedback loop | Keep as separate component per paper's decomposition |
| **Cascaded stateless input generation** | No states, intermediate signals have no other consumer | Merge into one programmatic block. Mark merged blocks `status: 'merged_into:<target>'` |
| **Monitoring-only outputs** | Outputs feed nothing in the dynamic loop | Exclude from `plan.components`. If needed for a `spec.validation_figures` entry, add as `plan.programmatic` with `role: 'monitoring'`. Otherwise mark `status: 'not_needed'` with justification |

**Print the filtering result:**
```
Component Filtering:
  PlantDynamics    — has states (x1,x2,x3,x4) → KEEP (plant dynamics)
  OutputAlgebra    — algebraic, consumed by Coupling → KEEP (plant dynamics)
  CouplingEq       — algebraic, consumed by Actuator → KEEP (plant dynamics)
  ActuatorDynamics — has state (x5), feedback to PlantDynamics → KEEP (plant dynamics)
  InputStage       — stateless, intermediate unused → MERGE into InputBlock
  CoordTransform   — stateless, cascaded after InputStage → MERGE into InputBlock
  MonitorOutput    — outputs (m1,m2,m3) feed nothing → MONITORING (needed for Fig.N: m1)
```

Only components marked KEEP appear in `plan.components`. Merged chains become a single entry in `plan.programmatic`. Monitoring-only blocks that produce signals required by `spec.validation_figures` are added to `plan.programmatic` with `role: 'monitoring'` — they receive loop signals as inputs but do NOT feed back into the plant.

Record all decisions in the spec JSON now. All subsequent gates depend on these fields.

---

## Phase 5: Convert to Build-Tool Format

### !! STOP GATE: Builder Search (MANDATORY — execute before ANY builder assignment) !!

**You MUST run `routeComponent` for EVERY component and print the results in chat before proceeding.**

This is not optional. This is not something you can skip because you "know the equations." The #1 failure mode is assigning odeBuilder without searching — wasting build time on blocks that Simulink already provides.

**Required sequence:**

```matlab
% Step 1: Run routeComponent for each component — paste ALL output in chat
for i = 1:N
    comp_i = struct('name','<Name>', 'description','<what it does>', ...
        'equations',{equations_i}, 'keywords',{{'<kw1>','<kw2>'}});
    [builder_i, dp_i, reasoning_i] = routeComponent(comp_i);
    fprintf('  %s → %s (%s)\n', comp_i.name, builder_i, reasoning_i);
end
```

```matlab
% Step 2: If routeComponent returns blocksetBuilder, verify the block:
results = findBlock('<block name>');
info = getBlockInfo(results(1).library);
% Print: port count, parameter names — confirm it matches your equations
```

```matlab
% Step 3: Print the routing table (ALL components):
% | Component | routeComponent says | You assign | Override reason (if different) |
% |-----------|--------------------:|:-----------|-------------------------------|
```

**Rules:**
- If `routeComponent` returns `blocksetBuilder` and you override to `odeBuilder`, you MUST provide `reject_standard_reason`. "I already wrote the equations" is NOT valid.
- If `routeComponent` returns `odeBuilder`, still call `findBlock` with domain-relevant keywords from the source (Level 2/3 check).
- A single odeBuilder component with >8 equations is suspicious. You MUST show that no subset can be separated into a standard block or a smaller independent component.

**DO NOT proceed to 5b until this table is printed and reviewed.**

---

### 5a-ii: Vector Port Expansion (MANDATORY for blocksetBuilder with vector ports)

After `getBlockInfo` confirms a blockset block, check its port labels. If any port accepts/produces a **vector** (e.g., `F_{XYZ} (N)` = width 3), you MUST populate `spec.vectorInputs` / `spec.vectorOutputs` so `executePlan` can expand them into named scalars for composition.

**How to detect vector ports:**
- Port labels with subscript notation: `F_{XYZ}`, `M_{XYZ}`, `V_b`, `\omega`, `X_e`
- Mask IC parameters that are vectors: `Vm_0 = [0 0 0]` → that port is width 3
- Block documentation mentioning "3x1 vector" inputs/outputs

**Required action when vector ports detected:**

```matlab
% For each vector INPUT port:
spec.vectorInputs(j).portIndex = <block_input_port_number>;
spec.vectorInputs(j).elementNames = {'<signal1>', '<signal2>', '<signal3>'};
% Element names come from domain knowledge:
%   F_{XYZ} → {'Fx','Fy','Fz'}; M_{XYZ} → {'La','Ma','Na'}

% For each vector OUTPUT port you need:
spec.vectorOutputs(j).portIndex = <block_output_port_number>;
spec.vectorOutputs(j).elementNames = {'<signal1>', '<signal2>', '<signal3>'};
% Element names must match what other components expect in plan.wiring:
%   V_b → {'u','v','w'}; \omega → {'p','q','r'}; euler → {'phi','theta','psi'}
```

**You can skip output ports you don't need** (e.g., DCM[9], d\omega/dt[3] on 6DOF). Only expand ports that feed into wiring or are declared in `interface.externalOutputs`.

### 5a-iii: Multi-Instance Blocks (USE when N identical blocks needed)

When the same library block is needed N times with different parameters (e.g., 5 actuator Transfer Functions, 4 tire models), use `spec.instances` instead of creating N separate components:

```matlab
spec.library = 'simulink/Continuous/Transfer Fcn';
spec.parameters = struct('Numerator','[1/0.05]','Denominator','[1 1/0.05]');  % base/default

spec.instances(1).name = 'tl';
spec.instances(1).parameters = struct('Numerator','[1/1.5]','Denominator','[1 1/1.5]');
spec.instances(2).name = 'tr';
spec.instances(2).parameters = struct('Numerator','[1/1.5]','Denominator','[1 1/1.5]');
spec.instances(3).name = 'de';
spec.instances(3).parameters = struct();  % uses base params
```

**Result:** One subsystem with N inputs (`<name>_cmd`) and N outputs (`<name>`). This avoids 5 plan.components entries for identical blocks.

---

### 5a: Builder selection — three-level block recognition

Before routing ANY equation or component to odeBuilder, apply this three-level recognition protocol. The goal: if a standard Simulink block exists for this dynamic, use `blocksetBuilder` — it's cleaner, more maintainable, and composes naturally.

#### Level 1: Static pattern match (no MATLAB call needed)

Check the **Block Recognition Table** in `builders.md`. If the equation pattern matches:
- `y(t) = u(t-τ)` → Transport Delay
- `Y(s)/U(s) = b(s)/a(s)` → Transfer Fcn
- `ẋ = Ax+Bu` → State-Space
- PID, saturation, dead zone, rate limiter, relay, unit delay, etc.

→ Route to `blocksetBuilder` immediately. No MATLAB call needed.

#### Level 2: Keyword/name discovery (one MATLAB call)

If the paper **names or describes** a component by keyword but it's not in the static table, call `findBlock`:

```matlab
results = findBlock('servo valve');         % paper says "servo-valve model"
results = findBlock('butterworth filter');  % paper says "Butterworth low-pass"
results = findBlock('smith predictor');     % paper says "Smith Predictor"
results = findBlock('rate limiter');        % paper says "output rate constrained"
```

If `findBlock` returns a valid library path:
```matlab
info = getBlockInfo(results{1});  % check ports, parameters, description
```

If the block matches what the paper describes → `blocksetBuilder` with that library path.

**When to invoke Level 2:**
- Paper names a specific technique, filter type, or component by name
- Paper describes behavior that sounds like a standard block but isn't in Level 1 table
- You're unsure whether a standard block exists for this function
- The equation involves something odeBuilder cannot express (delays, discrete dynamics, discontinuities with memory)

#### Level 3: Description/behavior matching (multiple MATLAB calls)

When the paper describes **behavior** without naming a specific block:
- "output cannot exceed ±25 V" → `findBlock('saturation')` → Saturation block
- "the signal is sampled at 1 kHz" → `findBlock('zero order hold')` → ZOH block  
- "velocity is limited to change no faster than 5 m/s²" → `findBlock('rate')` → Rate Limiter
- "backlash in the gear mechanism introduces 0.5° of play" → `findBlock('backlash')` → Backlash block

For Level 3, the flow is:
1. LLM interprets the behavior description
2. Generates candidate keywords
3. Calls `findBlock(keyword)` for each
4. Calls `getBlockInfo(path)` to verify the block matches
5. If confirmed → `blocksetBuilder`; if no match → `odeBuilder`

#### Builder routing (final priority order)

After applying the three levels above:
1. Empirical data (tables, curves) → `lookupTableBuilder`
2. **Recognized standard block** (Level 1/2/3 match) → `blocksetBuilder`
3. Physical network (bidirectional energy) → `simscapeBuilder`
4. Discrete-event (queues, entities) → `simeventsBuilder`
5. Pure discrete logic FSM → `stateflowBuilder`
6. Modes with different ODEs per mode → `odeBuilder_cps`
7. Novel nonlinear ODEs (no standard block equivalent) → `odeBuilder`

**The static table is a cache; `findBlock` is the authoritative lookup.** When unsure, query — don't guess.

For `odeBuilder_cps`, Phase 5c produces struct arrays instead of a single equation string.

### 5a2: Decision Path Struct (MANDATORY per component)

Every component in the plan MUST include a `decision_path` struct that records which tree branches were evaluated. This is structured proof of tree traversal, not free-text justification:

```matlab
% For plan.components entries (require a builder):
plan.components(i).decision_path = struct( ...
    'is_existing', false, ...               % Branch -1: component already exists in a loaded model?
    'existing_model_name', '', ...          % if yes: which model contains it?
    'existing_subsystem_path', '', ...      % if yes: full path to the subsystem
    'is_combinational', false, ...          % Branch 0: pure combinational logic?
    'combinational_reason', '', ...         % why it's combinational (or why not)
    'is_empirical_data', false, ...         % Branch 1: tables/curves/maps?
    'empirical_evidence', '', ...           % what data was found (or why not)
    'is_standard_block', true, ...          % Branch 2: library block exists?
    'standard_block_name', '6DoF (Euler Angles)', ... % findBlock result (empty if none)
    'findBlock_searched', true, ...         % did you actually call findBlock?
    'use_standard_block', true, ...         % did you select it?
    'reject_standard_reason', '', ...       % if rejected: WHY (must cite specific requirement)
    'is_physical_network', false, ...       % Branch 3: bidirectional energy?
    'is_discrete_event', false, ...         % Branch 4: entities/queues?
    'is_pure_logic', false, ...             % Branch 5: FSM/protocol?
    'has_mode_switching', false, ...        % Branch 6: different ODEs per mode?
    'has_tightly_coupled_conditionals', false, ... % Branch 7: coupled nonlinear + if/else?
    'conditional_description', '', ...      % what are the coupled conditionals?
    'builder_selected', 'blocksetBuilder', ...
    'tree_terminal_node', 'Branch 2: standard block exists');

% For plan.programmatic entries (Branch 0 = true AND no standard block → no builder needed):
% NOTE: If is_combinational == true BUT a standard block exists (e.g., Multiport Switch,
% Lookup Table), route to plan.components with blocksetBuilder instead.
plan.programmatic(j).name = 'InputGenerator';
plan.programmatic(j).type = 'MATLABFunction';  % 'MATLABFunction', 'Subsystem', 'Block'
plan.programmatic(j).code = 'function [ua,ub,uc] = fcn(theta,Amp) ...';
plan.programmatic(j).inputs = {'theta', 'Amp'};
plan.programmatic(j).outputs = {'ua', 'ub', 'uc'};
plan.programmatic(j).wiring.inputs(1) = struct('signal','theta','source','block');
plan.programmatic(j).wiring.outputs(1) = struct('port',1,'target','replace_constant:Transform/ua');
plan.programmatic(j).decision_path = struct('is_combinational', true, ...
    'combinational_reason', 'switching table: N sectors, no state memory, outputs = f(theta,Amp)', ...
    'tree_terminal_node', 'Branch 0: pure combinational logic');
```

**Calling `validatePlan`:** Accepts the plan struct, and optionally a spec struct for hierarchy enforcement checks.
```matlab
[ok, issues] = validatePlan(plan);        % without spec (skips hierarchy check)
[ok, issues] = validatePlan(plan, spec);  % with spec (enforces decomposition match)
```

`validatePlan` enforces: missing `decision_path` → plan blocked; `is_combinational == true` AND `is_standard_block == false` → must be in `plan.programmatic`; `findBlock_searched == false` with `is_standard_block == false` → blocked; MATLAB Function route requires `has_tightly_coupled_conditionals == true`.

### 5b: Build strategy — BOTTOM-UP preferred

**PREFERRED: Bottom-up decomposition.** When the source shows N physical subsystems, create N separate `plan.components` — one odeBuilder call per subsystem — and wire them together via `plan.wiring`. Each component stays small (≤5 equations), builds reliably, and `composeModel` handles inter-subsystem connections.

**AVOID: Top-down monolith.** Do NOT put 6+ equations into a single odeBuilder component and rely on `physical_subsystems` + `buildOdeInterface` to post-hoc partition the flat model into subsystems. This path is fragile (odeBuilder reorders equations, breaking index-based grouping) and produces worse error diagnostics.

**When to use a single odeBuilder call:**
- ≤5 tightly coupled equations with NO natural subsystem boundary
- The equations share state-dependent cross-coupling that cannot be separated without algebraic loops

**When to split into multiple components:**
- Source shows a block diagram with labeled subsystems → one component per subsystem
- Equations group by physical domain (electrical, mechanical, thermal)
- Clear port boundaries exist (outputs of one group feed inputs of another)

odeBuilder natively supports `\sin`, `\cos`, `\tan`, `\exp`, `\log`, `\abs`, `*`, `/`, `+`, `-`, `^`. This covers most nonlinear ODE systems -- even 12+ state models with trig products and Coriolis cross-terms. However, if a subsystem is a **recognized dynamic element** (transport delay, transfer function, PID, saturation, state-space, etc.), use `blocksetBuilder` — it produces a cleaner, more maintainable model than trying to express standard blocks as raw ODEs.

| Goes into odeBuilder | Goes into blocksetBuilder | Goes into MATLAB Function (Phase 8) | Other Phase 8 blocks |
|---|---|---|---|
| Novel nonlinear ODEs (`\dot{x} = ...`) with trig, products, fractions | Transport delays `y=u(t-τ)` | Non-diagonal mass matrix `M\f` | Lookup tables, efficiency maps |
| Custom algebraic equations (`y = f(x)`) | Transfer functions `b(s)/a(s)` | State-dependent singularity protection | Switches, conditional logic |
| Non-standard output equations | State-space `ẋ=Ax+Bu` | >30 tightly coupled polynomial coefficients | External data (From Workspace) |
| 12+ state systems with sin/cos/products | PID controllers | Conditional logic (`if`/`else`) | Signal generators |
| Coriolis cross-terms (m22*v*r) | Saturation, rate limiters, dead zones | | |
| Coupling between sections | Discrete TF, unit delays, ZOH | | |
| Linear gains, sums, products | Relay, backlash, quantizer | | |

**Rule: odeBuilder is used for novel plant ODEs that don't map to any known Simulink block.** After building flat, use `Simulink.BlockDiagram.createSubsystem(blockHandles, 'Name', 'SubsysName')` to create visual hierarchy. A MATLAB Function block is only used when no builder **structurally can express** the equation -- not because it "looks complex." If the equation IS a known block (delay, TF, PID, saturation), use `blocksetBuilder` — it participates in composition just like odeBuilder does.

Structural reasons for MATLAB Function (exhaustive -- if not on this list, use the builder):
1. **Non-diagonal mass matrix** -- `M * xdot = f(x)` where M has off-diagonal terms requiring `M\f` or `inv(M)*f`
2. **State-dependent denominators with singularity** -- `u/U` where `U = sqrt(u^2+v^2)` needs floor/clamp logic
3. **Conditional logic** -- `if`/`else` for controllers, clamping, saturation, mode switching
4. **Dense polynomial coupling** -- 30+ coefficients that all reference the same intermediate variables

**NOT valid reasons for MATLAB Function block:**
- "Equations look complex" -- odeBuilder handles complexity
- "12+ states" -- quadrotor 6-DOF (12 ODEs with sin/cos) built perfectly with odeBuilder
- "Trig products like cos(phi)sin(theta)cos(psi)" -- native odeBuilder
- "Coriolis cross-terms like m22*v*r" -- Product blocks, native odeBuilder
- "Many algebraic outputs" -- odeBuilder auto-wires these

If none of the structural reasons apply, the equations go into odeBuilder even if they look complex.

**NEVER fall back to fully programmatic builds** -- see `rules.md` §"Stage C-F Enforcement" for the full rationale. The builder is always the core; MATLAB Function blocks only for structural reasons listed above.

### 5c: Builder failure recovery

| Error | Cause | Fix |
|---|---|---|
| Algebraic output key error | Algebraic output on ODE RHS, wrong ordering | **odeBuilder auto-sorts -- this should not happen** |
| Variable name collision | Multi-word names or MATLAB keywords | Use short, unique variable names |
| Build succeeds but no integrators | Used `\\dot{x}` instead of `\dot{x}` | **Use single backslash** |
| odeBuilder parse error | Unsupported LaTeX construct | Simplify: expand macros, use explicit `*` for multiplication |

**Try at most 3 restructurings** (rename variables, inline algebraics, simplify). If an error is unclear, check the builder source to understand the parsing logic before retrying. If all 3 fail:
1. **Split the equation set.** Build in two batches.
2. **If splitting still fails**, isolate the failing equation as a single programmatic block.
3. **Document the failure** in `spec.assumptions`.

### 5d: Convert equations to builder format

**For `odeBuilder` (v2 default):**
```matlab
eqStr = '"\dot{x1}=x2" "\dot{x2}=(u-c*x2-k*x1)/m" "y=x1"';
params = struct('c', 10, 'k', 100, 'm', 1, 'u', 0);
ics = struct('x1', 0.1, 'x2', 0);
% odeBuilder accepts char or string
% odeBuilder auto-sorts algebraics before ODEs — ordering doesn't matter
```

**For `odeBuilder_cps`:** Build struct arrays for modes, transitions, params, and outputs.

**Format rules:**
1. **Single backslash for derivatives:** `\dot{x}` not `\\dot{x}`. Double backslash creates an Outport instead of an Integrator.
2. **No MATLAB keywords as variable names:** Avoid `i`, `j`, `pi`, `inf`, `end`, `if`, etc.

**MANDATORY (even in autonomous mode):** Save the final `eqStr` into the spec JSON as `equations_builder`. The report generator (`fillReport`) uses this field to populate the model structure table.

### 5e0: Input completeness check (MANDATORY — before building)

**For every `topInput` in the plan with a constant value, verify it is a true external boundary:**

For each constant topInput, answer:
1. Does the paper describe how this signal is generated? (any mechanism: equations, logic, block, table)
2. Does the paper's block diagram show a labeled block producing this signal?
3. Do the validation figures show behavior in downstream signals that a constant input cannot produce?

If ANY answer is YES → this input MUST become a component (in `plan.components` or `plan.programmatic`).

**Print the check result before proceeding:**
```
Input Completeness:
  <name> = <value> (constant)  → <reasoning: what the paper says about this signal's source> → OK | NEEDS COMPONENT
```

If any input fails: add the missing component before building. Do NOT proceed with constant inputs that the paper describes a source for.

### 5e: Subsystem liveness check (full pipeline only)

**Skip for `pipeline: 'simple'` and CPS.** Only run for the **full continuous pipeline**.

For each subsystem in `physical_subsystems`:
1. List the variables it computes (LHS of its equations).
2. Check whether any of those appear on the RHS of another subsystem's equations.
3. If a subsystem has **zero** consumed outputs, **STOP** -- switch equation form.

Print the check:
```
Subsystem Liveness Check:
  PlantDynamics      -> x1, x2, x3, x4          -> consumed by: OutputAlgebra, CouplingEquation  ok
  OutputAlgebra      -> y1, y2, y3, y4           -> consumed by: CouplingEquation  ok
  CouplingEquation   -> z1                       -> consumed by: ActuatorDynamics  ok
  ActuatorDynamics   -> y4                       -> consumed by: PlantDynamics  ok
```

### 5f: Verify spec completeness for deterministic hierarchy (full pipeline only)

**Phase 11 will read `physical_subsystems` from spec and execute it verbatim.** Verify now that the spec has all needed structural info:

```
Spec Hierarchy Completeness:
- [ ] physical_subsystems has N entries (matches paper's block count from Phase 2-3)
- [ ] Each entry has: name, equation_ref, equations[], inports[], outports[]
- [ ] Port names are physical signal names (not "In1", "Out1")
- [ ] Port ordering matches paper's figure (top-to-bottom) or alphabetical
- [ ] Every outport of subsystem S appears as an inport of at least one other subsystem
- [ ] Signal names are consistent across subsystems (same signal = same name everywhere)
```

If any field is missing, fill it now from the paper. **Do NOT leave port ordering to Phase 11 runtime decisions.**

**MANDATORY: `equation_ref` in plan struct.** When building from a paper, every component and hierarchy entry MUST carry `equation_ref` — the equation numbers from the source paper that this subsystem implements.

```matlab
% Multi-component path:
plan.components(i).equation_ref = '28-31';  % paper equation numbers

% Single-ode + hierarchy path:
plan.hierarchy(i).equation_ref = '28-31';   % paper equation numbers
```

**Common mistake:** Labeling with the ORIGINAL equation numbers instead of the IMPLEMENTED ones. If the paper derives simplified/substituted forms (e.g., substituting (11-14) into (1-4) to get (28-31)), use the FINAL implemented form's equation numbers — that's what the subsystem actually computes.

Update spec status to `translated`.

**In interactive mode:** Present build-tool strings to user and confirm. **In autonomous mode:** Proceed directly.

---

## Gate: Ready for Phase 6/7

```
Phase Gate -- Equations normalized and converted?
- [ ] spec JSON updated with equations_normalized_latex
- [ ] Pipeline variant locked: pipeline = 'simple' or 'full' recorded in spec
- [ ] All variable names validated against MATLAB built-ins (no collisions)
- [ ] spec JSON has equations_builder populated
- [ ] spec JSON status: translated
- [ ] [Full pipeline only] Subsystem liveness check passed (5e)
- [ ] [Full pipeline only] Hierarchy completeness verified (5f):
      - physical_subsystems has name, equation_ref, equations, inports, outports
      - Port ordering is deterministic (from paper figure or alphabetical)
      - Signal names are consistent across subsystem boundaries
- [ ] User confirmed builder strings (interactive mode only)
```
