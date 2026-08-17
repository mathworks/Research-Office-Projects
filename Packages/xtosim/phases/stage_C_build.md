# Phases 6-7: Build Section Models and Flat Combined Model

**Requires:** `rules.md`, `builders.md` (builder APIs and spec formats)
**Inputs:** spec JSON with `equations_builder` (status: `translated`)
**Outputs:** Flat combined Simulink model, open-loop verified, `c`/`m`/`cellEq` in workspace
**Next stage (depends on variant):**
- **Full pipeline (continuous):** Stage D (Read `stage_D_hierarchy.md`), then controllers (Read `stage_D_controllers.md`)
- **Simple path:** Stage E (Read `stage_E_validate.md`) — flat model IS final
- **CPS path:** Stage D controllers (Read `stage_D_controllers.md`)

---

## Builder Selection (v2 Default: odeBuilder)

| Builder | When | Post-build needed |
|---------|------|-------------------|
| **odeBuilder** (default) | All continuous ODE/algebraic systems | None — handles Constants, ICs, InitFcn, algebraic wiring internally |
| odeBuilder_cps | Hybrid/CPS with mode switching | None (partial internal handling) |

---

**Key function calls this phase:** `odeBuilder`, `writeInitFcn`, `addSignalLogging`. See SKILL.md "Mandatory Helper Function Calls" table for signatures.

---

## Phase 6: Build and Verify Each Section Model

**Skip rule:** Skip Phase 6 when `numel(sections) == 1` AND total equation count <= 6. Most journal papers have a single section -- go straight to Phase 7.

**Use Phase 6 when:** (a) multi-section sources (multi-chapter theses, 3+ independent equation sets), OR (b) single section with 7+ total equations.

---

## Phase 6-BU: Bottom-Up Composition

**Use when `pipeline == "bottom_up"`** (>10 total equations, or >=3 physically coupled subsystems). This replaces the Phase 7 -> Phase 11 sequence. Build subsystems independently, then compose directly.

### 6-BU.a: Partition equations into subsystem groups

Use `spec.physical_subsystems` to partition. Each group should have 5-8 equations max, mixing its ODEs and algebraic outputs.

### 6-BU.b: Build ALL equations in one flat call, then group

**Preferred approach (simpler, fewer tool calls):** Build ALL equations in a single odeBuilder call, verify open-loop, then use `createSubsystem` to group into physical subsystems:

```matlab
% Step 1: Build flat (ALL equations in one call) — odeBuilder handles everything
eqStr = '"alg1" "alg2" "\dot{x1} = f1" ... "\dot{x12} = f12"';
[c, m, cellEq, info] = odeBuilder(eqStr, mdl, ...
    'Params', params, 'IC', ics, 'Logging', true);

% Step 2: Verify open-loop with constant inputs
simOut = sim(mdl);  % states should stay at ICs or converge

% Step 3: Group into subsystems using createSubsystem
posBlocks = [];
for blk = {'xp','yp','zp','vx','vy','vz'}
    posBlocks(end+1) = get_param([mdl '/' blk{1}], 'Handle');
end
Simulink.BlockDiagram.createSubsystem(posBlocks, 'Name', 'PositionDynamics');
```

**Alternative approach (for very large models where single call is impractical):** Build each subsystem group separately, then compose:

```matlab
eqStr_i = '"alg1" "alg2" "\dot{x1} = f1" "\dot{x2} = f2"';
[c_i, m_i, cellEq_i] = odeBuilder(eqStr_i, subsysModel_i, ...
    'Params', params, 'IC', ics_i);
```

### 6-BU.c: Verify open-loop

Simulate with constant inputs at equilibrium. States should remain at ICs or converge. Fix any issues before grouping/composing.

### 6-BU.d: Create hierarchy with createSubsystem

**If using flat-first approach (preferred):**
```matlab
% Gather block handles for each physical subsystem group
% Then create subsystems
Simulink.BlockDiagram.createSubsystem(groupHandles, 'Name', 'SubsysName');
```

**If using per-subsystem approach:** Compose into top-level model:
```matlab
new_system(topModel); open_system(topModel);
add_block('built-in/SubSystem', [topModel '/' name]);
Simulink.BlockDiagram.copyContentsToSubSystem(subsysModel_i, [topModel '/' name]);
```

For each subsystem: replace cross-subsystem Constant blocks with Inport blocks, add Outport blocks for states needed by other subsystems, then wire subsystem ports at the top level.

### 6-BU.e: Resolve algebraic loops

Use `insertAlgebraicConstraints(mdl, loops)` to insert Algebraic Constraint blocks at loop boundaries. Prefer AC blocks (exact solution) over Memory blocks (1-step delay approximation). See helper docstring for struct format.

**Fallback: Memory blocks** where the algebraic solver cannot converge (highly nonlinear, discontinuous, or poorly conditioned). Typical location: acceleration feedback between subsystems (see Phase 9c).

### 6-BU.f: Add programmatic blocks and verify

Add steering inputs, coordinate transforms, torque sources at the top level. Then proceed to Phase 8 -> Phase 9 -> Phase 10 -> Phase 12.

### When to use MATLAB Function blocks vs odeBuilder

**Default: ALWAYS use odeBuilder, then `Simulink.BlockDiagram.createSubsystem` to group into hierarchy.** odeBuilder natively supports `\sin`, `\cos`, `\tan`, `\exp`, `\log`, `\abs`, `*`, `/`, `+`, `-`, `^` -- this covers nearly all ODE systems including 12+ state models with trig products and Coriolis cross-terms.

| Situation | Approach |
|-----------|----------|
| Any ODE system with supported math functions (sin, cos, exp, products, fractions) | **odeBuilder** -- even for 12+ states, trig products, cross-coupling terms |
| Many equations (12+) needing visual grouping | odeBuilder flat build -> `createSubsystem` to group into physical subsystems |
| Algebraic outputs coupling across subsystems | odeBuilder (auto-wires algebraic outputs internally) |
| Non-diagonal mass matrix: `M*xdot = f(x)` requiring `M\f` | MATLAB Function block (cannot express matrix inversion) |
| State-dependent normalization in denominators (e.g., `u/sqrt(u^2+v^2)`) | MATLAB Function block (singularity handling needs `if`/`max`) |
| Conditional logic (`if`/`else`, clamping, saturation, mode switching) | MATLAB Function block (no conditional constructs in builders) |
| Empirical nonlinear functions (lookup tables, piecewise curves) | MATLAB Function block or lookupTableBuilder |

**NOT valid reasons for MATLAB Function block:**
- "Equations look complex" -- odeBuilder handles complexity fine
- "12+ states" -- odeBuilder handles 12+ state systems with trig products perfectly
- "Trig products like cos(phi)sin(theta)cos(psi)" -- native odeBuilder
- "Coriolis cross-terms like m22*v*r" -- just Product blocks, native odeBuilder
- "Many algebraic outputs" -- odeBuilder auto-wires these

---

### 6a: Build each section model

```matlab
[c, m, cellEq, info] = odeBuilder(eqStr, sectionModel, ...
    'Params', params, 'IC', ics);
% odeBuilder handles Constants, ICs, InitFcn, and algebraic wiring internally
```

Then verify initial conditions were set correctly.

### 6b: Add data logging to each section model

```matlab
% Use addSignalLogging (NOT addLogging -- which adds ToWorkspace blocks)
addSignalLogging(sectionModel, sectionSpec, 'flat');
```

**Every section model must have logged outputs before running any verification.**

### 6c: Verify each section model independently

For each section model, check: (1) steady-state convergence at known operating point, (2) step response direction/magnitude/time constant, (3) unit consistency, (4) sign conventions.

**Cross-section coupling in multi-section models:** Set cross-section inputs to **known constant values** from the paper's operating point. Full coupling verification happens in Phase 7.

If verification fails: check parameters -> equation transcription -> sign conventions -> units. Fix BEFORE combining.

Save each verified model. Update spec status to `verified`.

---

## Phase 7: Build the Flat Combined Model

**Phase 7 builds and open-loop verifies the ODE/algebraic core only.** Programmatic blocks (inverters, transforms, controllers) are added later in Phase 8 -- after hierarchy creation (Phase 11). This ensures `c`, `m`, `cellEq` are never stale when `createHierarchy` runs.

### 7a: Combine all equations into one model

Build a single model containing ALL section equations together, including **cross-section coupling equations and simple feedback ODEs**. Put as much as possible into the single build-tool call -- only leave truly non-ODE elements for Phase 8.

**Why maximize the build-tool call?** Every equation inside the build-tool call gets auto-wired. Every block added manually is a potential wiring bug.

**Why flat?** Shared variables are connected automatically when all equations are in the same model.

```matlab
[c, m, cellEq, info] = odeBuilder(combinedStr, flatModel, ...
    'Params', params, ...
    'IC', ics, ...
    'Logging', true, ...    % auto-log all states
    'Arrange', true);       % auto-arrange layout

% odeBuilder handles all of these internally:
%   - Constants set to workspace variable names (not literal 1)
%   - Algebraic outputs auto-wired to ODE consumers
%   - InitFcn callback written (standalone Play button works)
%   - Integrator ICs set from 'IC' struct
%   - Signal logging enabled on all states
```

### 7b: Set InitFcn and ICs

**odeBuilder:** Already done internally. Skip to 7c.

**IMPORTANT: InitFcn must include ALL parameters the model will ever need**, including parameters for programmatic blocks added later in Phase 8 (gains, saturation limits, controller constants, etc.). If you add a Gain block with value `Kgpri` in Phase 8, `Kgpri` must be in the InitFcn. Update the InitFcn in Phase 8 after adding programmatic blocks, or the model will fail the standalone Play-button test in Phase 12c.

### 7c: Add data logging to the flat model (ALL pipelines)

**Log ALL states and ALL algebraic outputs** using **signal logging** (not To Workspace blocks). Signal logging is invisible -- it doesn't add blocks to the model, keeping the diagram clean. Do NOT limit to 3-5 signals -- that produces incomplete reports.

**If you used `'Logging', true` in odeBuilder:** State signals are already logged. You may still call `addSignalLogging` to ensure subsystem-level coverage:
```matlab
addSignalLogging(mdl, spec, 'flat');   % ensures all signals logged
```

**For the full pipeline:** After Phase 11, integrator-level logging becomes inaccessible (integrators move inside subsystems). After Phase 11, call:
```matlab
addSignalLogging(mdl, spec, 'hierarchical');  % Phase 11 post: log subsystem outputs
```

The helper handles duplicate prevention -- calling it twice at different stages won't create duplicate signal names. The 'hierarchical' stage logs subsystem OUTPUT ports (the correct level for the final model).

**Accessing logged data after simulation:**
```matlab
simOut = sim(mdl);
% Signal logging data is in simOut.logsout
sig = simOut.logsout.get('wr');
if isa(sig, 'Simulink.SimulationData.Dataset')
    sig = sig.get(sig.numElements);  % last one = subsystem level
end
wr_data = sig.Values.Data;
```

### 7d: Open-loop verification (constant inputs only)

Verify the flat model runs correctly with **constant inputs at equilibrium values**. At this point the model has NO inverter, NO transforms, NO controllers -- just the builder core.

```matlab
set_param(flatModel, 'StopTime', '100');
simOut = sim(flatModel);
% States should remain at or converge to equilibrium
```

This catches: incorrect signal wiring, parameter/unit mismatches, sign convention errors.

**Save the flat model as backup:**
```matlab
save_system(flatModel);
% Copy to backup file WITHOUT closing/reloading:
copyfile([flatModel '.slx'], [flatModel '_flat.slx']);
```

**CRITICAL: Do NOT close or reload the model after this point.** The `c`, `m`, `cellEq` variables contain block handles tied to THIS specific model load session. If you `close_system` + `load_system`, all handles become stale and `createHierarchy` will silently fail. If you accidentally close the model, you must rebuild from scratch.

**Next step depends on pipeline variant:**
- **Full pipeline (continuous):** Phase 7b (if `incremental_validation: true`), then Phase 11 (hierarchy), then Phase 8.
- **Simple path:** Phase 10 directly (flat model IS final). No hierarchy, no Phase 8, no Phase 9.
- **CPS path:** See Phase 7-CPS below.

---

## Phase 7b: Subsystem Validation (before loop closure)

**Skip if** `spec.incremental_validation == false` (or `spec.subsystem_tests` is empty).

**Purpose:** Verify each subsystem's physics are correct BEFORE wiring controllers. This isolates plant bugs from controller/wiring bugs. If a subsystem fails here, you know the plant equations or parameters are wrong — not the controller.

### 7b.1: Run subsystem tests

Use the `validateSubsystems` helper:

```matlab
results = validateSubsystems(mdl, spec);
```

This function:
1. For each entry in `spec.subsystem_tests`:
   - Temporarily disconnects the subsystem inputs from the rest of the model
   - Feeds constant inputs at the values specified in `.inputs`
   - Simulates for `.duration` seconds
   - Checks each `.checks` entry against the simulated output
2. Returns struct array with `.subsystem`, `.test`, `.status` ('pass'/'fail'), `.details`

### 7b.2: Act on results

```matlab
if all(strcmp({results.status}, 'pass'))
    fprintf('Phase 7b PASSED: all subsystem tests pass.\n');
else
    for i = 1:numel(results)
        if strcmp(results(i).status, 'fail')
            fprintf('FAIL: %s — %s\n', results(i).subsystem, results(i).details);
        end
    end
    % Fix plant before proceeding. Max 5 attempts per subsystem.
end
```

**If a subsystem test fails:**
1. The problem is in the plant (equations, parameters, signs, or nondimensionalization) — NOT the controller
2. Fix the issue (max 5 attempts per subsystem)
3. Re-run `validateSubsystems` until pass
4. If 5 attempts exhausted: document in `spec.assumptions`, mark subsystem as `validation_uncertain`, proceed

**Do NOT proceed to Phase 11/8 with failing subsystem tests.** Controller tuning cannot fix a broken plant. The whole point is to catch the error HERE where diagnosis is easy.

### 7b.3: What subsystem tests validate

| Check type | What it proves | Example |
|---|---|---|
| `steady_state` | Equilibrium matches paper's operating point | V → 23.646 ft/s at cruise torque |
| `settling_time` | Time constants/eigenvalues correct | Steering settles in ~6s (1/λ_fast) |
| `peak` | Transient magnitude correct | Overshoot matches step response |
| `frequency` | Oscillation frequency correct | Motor electrical frequency |
| `eigenvalues` | Linearized dynamics match paper | λ₁ = −0.106, λ₂ = −1.769 |
| `final_value` | Integration over known time matches | Xo(600) = V₀×600 − 3000 = 11188 ft |

### 7b.4: Gate

```
Phase Gate -- Subsystem validation (7b)?
- [ ] spec.incremental_validation is true (otherwise skip)
- [ ] validateSubsystems(mdl, spec) all pass (or documented failures)
- [ ] Plant bugs fixed BEFORE proceeding to hierarchy/controllers
- [ ] No sign hacks, no parameter inversions "to make it work"
```

---

## 7-CPS: Build and verify CPS/hybrid models (odeBuilder_cps only)

**This section replaces 7a-7d for CPS models.**

### 7-CPS.a: Build the CPS model

```matlab
% Example: bouncing ball
modes(1).name = 'FreeFall';
modes(1).equations = '"\dot{h}=v" "\dot{v}=-g"';
modes(1).ic = struct('h', 10, 'v', 0);

modes(2).name = 'Rest';
modes(2).equations = '"\dot{h}=0" "\dot{v}=0"';
modes(2).ic = struct('h', 0, 'v', 0);

transitions(1).source = 'FreeFall';
transitions(1).destination = 'Rest';
transitions(1).guard = 'FreeFall.h <= 0 && abs(FreeFall.v) < tol';
transitions(1).action = '';

transitions(2).source = 'FreeFall';
transitions(2).destination = 'FreeFall';
transitions(2).guard = 'FreeFall.h <= 0 && abs(FreeFall.v) >= tol';
transitions(2).action = 'FreeFall.v = -COR * FreeFall.v';

params = struct('g', 9.81, 'COR', 0.8, 'tol', 0.01);
outputs = {'h', 'v'};

opts.inputs = {'throttle', 'brake'};  % optional chart-level inputs

[modelName, chartObj] = odeBuilder_cps(modes, transitions, params, outputs, 'bouncing_ball', opts);
```

### 7-CPS.b: Set InitFcn and ICs

Use `writeInitFcn` as in 7b. CPS ICs are in `modes.ic`.

### 7-CPS.c: Add data logging

Add `ToWorkspace` blocks for chart outputs.

### 7-CPS.d: Verify CPS model

Verify:
1. **Mode transitions fire correctly**
2. **State resets are correct**
3. **Terminal behavior is physical**
4. **No spurious transitions**

**Next: Phase 8 -> Phase 9 -> Phase 10.**

---

## Gate: After Phase 7/7b / Before Phase 11 (MANDATORY)

**Skip for simple path** (proceed to Phase 10). **Skip for CPS** (proceed to Phase 8).

```
Phase Gate -- Flat model ready for hierarchy? (continuous full pipeline only)
- [ ] Flat model builds and simulates without error (open-loop, constant inputs)
- [ ] Data logging added to all key state and output signals (7c)
- [ ] Open-loop verification passed (7d): constant inputs at equilibrium,
      states converge to expected values (print actual vs expected)
- [ ] InitFcn and ICs set -- model runs with just Play button
- [ ] [If incremental_validation] Phase 7b subsystem tests all PASS
- [ ] Flat model saved as backup (<model_id>_flat.slx)
- [ ] c, m, cellEq variables from builder still in MATLAB workspace
```
