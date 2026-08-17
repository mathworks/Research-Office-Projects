# Spec & Plan Format Reference

> **Purpose:** Exact struct formats for calling `executePlan`. Read this before constructing ANY plan struct.  
> **Rule:** If a field is marked REQUIRED, `validatePlan` or `executePlan` will error without it.

---

## 1. Plan Struct (top-level)

```matlab
plan.name           = 'ModelName';           % REQUIRED: char
plan.components     = [comp1, comp2, ...];   % REQUIRED: struct array
plan.wiring         = [wire1, wire2, ...];   % REQUIRED (can be empty for single component)
plan.topInputs      = {ti1, ti2, ...};       % OPTIONAL: CELL array of structs
plan.topOutputs     = {to1, to2, ...};       % OPTIONAL: CELL array of structs
plan.programmatic   = {prog1, prog2, ...};   % OPTIONAL: CELL array of structs
plan.solver         = 'ode45';               % OPTIONAL: char (default: auto)
plan.stopTime       = '10';                  % OPTIONAL: char (default: '10')
plan.busMode        = 'none';                % OPTIONAL: 'none'|'per_subsystem'|'grouped'
```

**CRITICAL:** `topInputs`, `topOutputs`, and `programmatic` are **CELL arrays**, not struct arrays.

---

## 2. Component Struct (`plan.components(i)`)

```matlab
comp.name           = 'Stator Flux';         % REQUIRED: char
comp.builder        = 'odeBuilder';          % REQUIRED: one of VALID_BUILDERS
comp.spec           = statorSpec;            % REQUIRED: builder-specific struct (see below)
comp.decision_path  = dpStruct;             % REQUIRED: see Section 7
comp.interface      = ifaceStruct;          % RECOMMENDED: see Section 3
comp.description    = 'Stator flux ODEs';   % OPTIONAL: for report
comp.equation_ref   = 'Eqs 28-29';         % OPTIONAL: for report table
```

Valid builders: `'odeBuilder'`, `'odeBuilder_cps'`, `'simscapeBuilder'`, `'blocksetBuilder'`, `'lookupTableBuilder'`, `'stateflowBuilder'`, `'simeventsBuilder'`, `'existing'`

---

## 3. Component Interface (`comp.interface`)

This tells `buildOdeInterface` which `u_` Constants are external inputs and which signals are outputs.

```matlab
comp.interface.externalInputs  = {'u1', 'u2', 'x3'};            % CELL of input var names
comp.interface.externalOutputs = {'y1', 'y2', 'x1'};            % CELL of output var names
```

**WHY THIS IS NEEDED:**  
odeBuilder creates `u_<varname>` Constant blocks for variables in equations that aren't parameters or states. `buildOdeInterface` only exposes them as interface inputs if you declare them in `externalInputs`. Without this, composition will fail with "0 inputs" and all wiring will error.

**HOW TO DETERMINE:**  
Any variable in your equations that is:
- NOT a parameter (not in `spec.params`)
- NOT a state (not on the LHS of a `\dot{}` equation)
- Comes from another subsystem or the outside world

...MUST be listed in `externalInputs`.

**OUTPUT NAMING:**  
- Algebraic outputs (LHS of non-`\dot{}` equations): use the variable name directly (e.g., `'y1'`)
- State outputs (from integrators): use the state name directly (e.g., `'x1'`)
- `buildOdeInterface` handles the internal `_out` suffix stripping automatically

---

## 4. odeBuilder Spec (`comp.spec` when `builder = 'odeBuilder'`)

```matlab
spec.equations = { ...
    '\dot{x1} = u1 - (p1*p2/p5)*x1 + p4*x2', ...                          % ODE
    '\dot{x2} = u2 - (p1*p2/p5)*x2 - p4*x1', ...                          % ODE
    'y1 = (p2/p5)*x1 - (p3/p5)*x3', ...                                    % algebraic
    'y2 = (p2/p5)*x2 - (p3/p5)*x4'};                                       % algebraic

spec.equation_refs = {'Paper Eq. X1', 'Paper Eq. X2', 'Paper Eq. Y1', 'Paper Eq. Y2'};

spec.params = struct('p1', 0.087, 'p2', 35.5e-3, 'p3', 34.7e-3, ...
                     'p5', 5.616e-5, 'p4', 376.99);

spec.ic = struct('x1', 0, 'x2', 0);
```

### Smoke Test (`comp.spec.smoke_test`) — OPTIONAL

Per-component magnitude sanity check. Runs after build with constant inputs.
Catches coefficient errors (wrong denominator, missing conversion factor) before composition.

```matlab
comp.spec.smoke_test = struct( ...
    'inputs', struct('name',{'y1','y2','x1','x3'}, ...
                     'value',{200, 50, 0.9, 0.3}), ...
    'outputs', struct('name',{'z1'}, 'min',{-2000}, 'max',{2000}), ...
    'duration', 0.01);
```

**Fields:**
- `inputs` — struct array: `name` (matches u_<name> constant block), `value` (plausible operating point)
- `outputs` — struct array: `name`, `min` (lower bound), `max` (upper bound), optionally `max_abs`
- `duration` — simulation time in seconds (default 0.01)

**When to use:** Any component where the output magnitude is predictable from the physics. If the coupling equation should produce O(1000) at rated inputs, set `max=5000`. A 500x violation means the equation is wrong.

---

### Equation Format Rules

| Pattern | Meaning | Example |
|---------|---------|---------|
| `\dot{x} = ...` | ODE (x is a state) | `\dot{v} = F/m` |
| `y = ...` | Algebraic output | `a = F/m` |
| `\frac{a}{b}` | Fraction | `\frac{dv}{dt}` |
| `\sin`, `\cos`, `\tan` | Trig functions | `\sin(theta)` |
| `\exp`, `\log`, `\sqrt` | Math functions | `\exp(-t/tau)` |

**CRITICAL RULES:**
1. ODEs MUST use `\dot{statename}` on the LHS — NOT `d<state> = ...` or `dx = ...`
2. Equations can be cell array of chars OR string array
3. odeBuilder auto-sorts: algebraic first, then ODEs
4. Parameters in equations must match field names in `spec.params`
5. Variables NOT in params and NOT states become `u_<varname>` Constants (external inputs)

### Initial Conditions (`spec.ic`)

```matlab
spec.ic = struct('state1', value1, 'state2', value2);
```

- **MUST be a struct**, not a vector
- Field names must match state names (what appears inside `\dot{}`)
- Missing states default to 0

### Parameters (`spec.params`)

```matlab
spec.params = struct('k', 100, 'b', 10, 'm', 1.5);
```

- **MUST be a struct**, not a cell or table
- odeBuilder creates Constant blocks with `Value` set to the parameter variable name (not the number)
- Parameters are initialized via the model's InitFcn callback

### Equation Refs (`spec.equation_refs`)

```matlab
spec.equation_refs = {'Paper Eq. 28', 'Paper Eq. 29', 'Paper Eq. 15', 'Paper Eq. 16'};
```

- **REQUIRED** — `validatePlan` blocks without it
- Cell array, same length as `spec.equations`
- Used for report equation numbering (shows paper numbers, not sequential)
- Format: free text identifying the source (e.g., "Eq. 7", "Table 2 row 3", "Derived")

---

## 4a. blocksetBuilder Spec (`comp.spec` when `builder = 'blocksetBuilder'`)

```matlab
spec.name       = 'Dynamics_6DOF';                            % REQUIRED: block instance name
spec.library    = 'aerolib6dof/6DoF (Euler Angles)';          % REQUIRED: full Simulink library path
spec.parameters = struct('Mass','18900','Inertia','eye(3)');   % REQUIRED: mask parameter name-value pairs
```

### Vector Port Expansion (`spec.vectorInputs` / `spec.vectorOutputs`) — USE FOR VECTOR-PORT BLOCKS

When a blockset block has vector ports (width > 1), composition with scalar-port components fails because `composeModel` wires port-to-port by name. To expose individual scalar signals at the subsystem boundary, provide these fields:

```matlab
% Input expansion: block expects [3x1] vectors, but composition sends scalars
spec.vectorInputs(1).portIndex = 1;                    % block input port number
spec.vectorInputs(1).elementNames = {'Fx','Fy','Fz'};  % scalar signal names (width = numel)

spec.vectorInputs(2).portIndex = 2;
spec.vectorInputs(2).elementNames = {'La','Ma','Na'};

% Output expansion: block outputs [3x1] vectors, composition expects scalars
spec.vectorOutputs(1).portIndex = 5;                   % block output port number
spec.vectorOutputs(1).elementNames = {'u','v','w'};

spec.vectorOutputs(2).portIndex = 6;
spec.vectorOutputs(2).elementNames = {'p','q','r'};
```

**How `executePlan` uses this:** Inserts Mux blocks before each vector input (N scalar Inports → Mux → block port) and Demux blocks after each vector output (block port → Demux → N scalar Outports). The resulting interface exposes scalar signals that match `plan.wiring` entries.

**When to use:** Call `getBlockInfo(libraryPath)` — if port labels show vector notation (e.g., `F_{XYZ}`, `\omega`) and the block will be wired to scalar-output components, use these fields.

**Omitting:** If omitted, ports stay as-is (vector). Only works if all connected components also use matching vector widths.

### Multi-Instance (`spec.instances`) — USE FOR N IDENTICAL BLOCKS

When multiple instances of the same block type are needed (e.g., 5 actuator Transfer Functions), define them in one component instead of 5:

```matlab
spec.instances(1).name = 'tl';
spec.instances(1).parameters = struct('Numerator','[1/1.5]','Denominator','[1 1/1.5]');

spec.instances(2).name = 'tr';
spec.instances(2).parameters = struct('Numerator','[1/1.5]','Denominator','[1 1/1.5]');

spec.instances(3).name = 'de';
spec.instances(3).parameters = struct();  % uses base spec.parameters

spec.instances(4).name = 'da';
spec.instances(4).parameters = struct();

spec.instances(5).name = 'dr';
spec.instances(5).parameters = struct();
```

**How `executePlan` uses this:** Places N copies of the library block in one subsystem model. Each instance gets an Inport named `<name>_cmd` and an Outport named `<name>`. Interface exposes N inputs + N outputs with scalar wiring.

**Instance parameter merging:** Each `instances(k).parameters` overrides the base `spec.parameters`. Fields not overridden use the base value.

### Optional Fields

```matlab
spec.inputNames       = {'Forces','Moments'};   % OPTIONAL: name vector ports (without expansion)
spec.outputNames      = {'Ve','Xe','Euler',...}; % OPTIONAL: name vector ports (without expansion)
spec.source_equation  = 'F=ma, Euler angles';    % OPTIONAL: traceability
spec.recognized_model = '6DOF Euler Angles';     % OPTIONAL: what model this represents
spec.reason           = 'Standard block exists'; % OPTIONAL: why blocksetBuilder was chosen
```

---

## 4b. existing Builder Spec (`comp.spec` when `builder = 'existing'`)

```matlab
spec.model     = 'vehiclePlant';          % REQUIRED: name of loaded model (bdIsLoaded)
spec.subsystem = 'Chassis';               % OPTIONAL: subsystem path within model
                                          %   Omit to use entire model as one subsystem
spec.port_overrides = struct();           % OPTIONAL: rename ports for cleaner wiring
spec.port_overrides.inputs  = {'u1','Force'; 'u2','Torque'};   % Nx2 cell {old, new}
spec.port_overrides.outputs = {'y1','Position'; 'y2','Speed'}; % Nx2 cell {old, new}
```

**No `equations`, `params`, `ic`, or `equation_refs` needed** — those belong to the source model.

**Interface discovery:** `executePlan` auto-discovers ports from the subsystem's Inport/Outport blocks. You can still declare `comp.interface.externalInputs`/`externalOutputs` explicitly (recommended for clarity), but if omitted they are inferred from the block.

**Decision path for existing:**
```matlab
dp.tree_terminal_node = 'existing (pre-built subsystem)';
dp.is_existing        = true;              % NEW field — signals skip of build
dp.source_model       = 'vehiclePlant';
dp.source_subsystem   = 'Chassis';         % empty if whole model
```

---

## 5. Wiring Struct (`plan.wiring(i)`)

```matlab
wire.from_component = 'OutputAlgebra';      % REQUIRED: must be in plan.components[].name
wire.from_port      = 'y1';                % REQUIRED: must be in source's externalOutputs
wire.to_component   = 'CouplingEquation';  % REQUIRED: must be in plan.components[].name
wire.to_port        = 'y1';                % REQUIRED: must be in target's externalInputs
wire.type           = 'signal';            % OPTIONAL: default 'auto'
```

**PORT NAME MATCHING:**
- `from_port` must appear in `source_comp.interface.externalOutputs`
- `to_port` must appear in `target_comp.interface.externalInputs`
- Names are EXACT string matches — no fuzzy matching

---

## 6. TopInput / TopOutput Structs

**IMPORTANT: topOutputs vs Signal Logging**

`topOutputs` creates Outport blocks at the model's top level. Only use them for signals that must be **physically routed** — consumed by programmatic blocks, external controllers, or other models.

For signals you only need to **observe** (plot, validate, report), rely on **signal logging** — `executePlan` calls `addSignalLogging` automatically. Access data via `simOut.logsout.get('signal_name')`.

**Guideline:** A typical model has 1-3 topOutputs (key system outputs like speed, position, voltage), not 10+ mirroring every internal state. If a signal is only needed for validation plots, do NOT add it to topOutputs.

### TopInput (`plan.topInputs{i}`)

```matlab
ti.name      = 'TL';                    % REQUIRED: block name in model
ti.component = 'Torque and Speed';      % REQUIRED: target component name
ti.port      = 'TL';                    % REQUIRED: target port name
ti.type      = 'Step';                  % OPTIONAL: source type (default 'Constant')
ti.value     = '100';                   % OPTIONAL: Constant value or Step "After"
ti.params    = struct('Time','2');       % OPTIONAL: generic set_param passthrough
% NOTE: ti.value must be a NUMERIC STRING (e.g., '100', '460', '0').
% Variable names (e.g., 'TL_value') will only resolve if the variable exists
% in the base workspace or spec.parameters. Prefer literal values.
```

**FIELD NAMES:** `.name`, `.component`, `.port` — NOT `.to_component`, `.to_port`

**Supported source types** (case-insensitive):

| Type | Library Block | Common params |
|------|--------------|---------------|
| `'Constant'` | Constant (default) | `Value` |
| `'Step'` | Step | `Time`, `Before`, `After` |
| `'Ramp'` | Ramp | `Slope`, `StartTime`, `InitialOutput` |
| `'Pulse'` | Pulse Generator | `Amplitude`, `Period`, `PulseWidth`, `PhaseDelay` |
| `'Sine'` | Sine Wave | `Amplitude`, `Frequency`, `Phase`, `Bias` |
| `'Signal_Generator'` | Signal Generator | `WaveForm`, `Amplitude`, `Frequency` |
| `'Repeating_Sequence'` | Repeating Sequence | `rep_seq_t`, `rep_seq_y` |
| `'From_Workspace'` | From Workspace | `VariableName`, `OutputAfterFinalValue` |
| `'Clock'` | Clock | — |
| `'Random'` | Uniform Random Number | `Minimum`, `Maximum`, `Seed` |
| `'Band_Limited_Noise'` | Band-Limited White Noise | `Cov`, `Ts`, `seed` |
| `'Chirp'` | Chirp Signal | `T1`, `f1`, `T2`, `f2` |
| `'programmatic'` | *(skipped — handled by plan.programmatic)* | — |

**Any unrecognized type** is treated as a direct Simulink library path (e.g., `'simulink/Sources/Counter Free-Running'`).

**Generic params passthrough:** Instead of type-specific shorthand fields, use `ti.params` — a struct whose field names map directly to `set_param` calls. Works with ANY block type.

```matlab
% Step with shorthand fields (legacy, still works):
ti = struct('name','TL', 'component','Mech', 'port','TL', ...
    'type','Step', 'step_time','2', 'value','100', 'initial_value',0);

% Same Step with generic params (preferred for new code):
ti = struct('name','TL', 'component','Mech', 'port','TL', ...
    'type','Step', 'params',struct('Time','2','Before','0','After','100'));

% Pulse generator:
ti = struct('name','PWM', 'component','Inverter', 'port','gate', ...
    'type','Pulse', 'params',struct('Amplitude','1','Period','0.001','PulseWidth','50'));

% Sine wave:
ti = struct('name','Vac', 'component','Grid', 'port','vac', ...
    'type','Sine', 'params',struct('Amplitude','325.27','Frequency','376.99'));

% Programmatic source (handled by plan.programmatic, not composeModel):
ti = struct('name','u1', 'component','PlantDynamics', 'port','u1', ...
    'type','programmatic');
```

### TopOutput (`plan.topOutputs{i}`)

```matlab
to.name      = 'y4';                    % REQUIRED: Outport block name in model
to.component = 'ActuatorDynamics';      % REQUIRED: source component name
to.port      = 'y4';                    % REQUIRED: source port name
```

**FIELD NAMES:** `.name`, `.component`, `.port` — NOT `.from_component`, `.from_port`

---

## 7. Decision Path (`comp.decision_path`)

```matlab
dp.tree_terminal_node = 'odeBuilder (novel nonlinear ODEs)';  % REQUIRED
dp.is_combinational   = false;          % REQUIRED for validatePlan checks
dp.is_empirical_data  = false;          % recommended
dp.is_standard_block  = false;          % REQUIRED if claiming no standard block
dp.findBlock_searched = true;           % REQUIRED if is_standard_block=false
dp.findBlock_result   = 'no match';     % recommended: what findBlock returned
dp.is_physical_network = false;         % recommended
dp.is_DES             = false;          % recommended
dp.is_FSM             = false;          % recommended
dp.has_modal_ODEs     = false;          % recommended
dp.final_builder      = 'odeBuilder';   % recommended: matches comp.builder
dp.reasoning          = 'Novel coupled ODEs'; % recommended
```

**VALIDATION GATES:**
- Missing `tree_terminal_node` → plan BLOCKED
- `is_combinational=true` AND `is_standard_block=false` → must be in `plan.programmatic`, not `plan.components`
- `is_standard_block=true` AND `use_standard_block=false` → `reject_standard_reason` required
- `findBlock_searched=false` AND `is_standard_block=false` → plan BLOCKED

---

## 8. Programmatic Blocks (`plan.programmatic{i}`)

```matlab
prog.name    = 'InputGenerator';         % REQUIRED
prog.type    = 'MATLABFunction';        % REQUIRED: 'MATLABFunction' | 'Subsystem' | 'Block'
prog.code    = 'function [y] = ...';    % REQUIRED for MATLABFunction
prog.inputs  = {'theta', 'Amp'};        % REQUIRED: cell of input names
prog.outputs = {'ua', 'ub', 'uc'};      % REQUIRED: cell of output names
prog.wiring  = struct(...);             % REQUIRED: how to connect (see below)
prog.equation_ref = 'Fig N';           % OPTIONAL: for report
prog.description  = 'Switching logic'; % OPTIONAL: for report
```

### Programmatic Wiring

```matlab
prog.wiring.inputs  = {'theta <- Integrator.theta', 'Amp <- Constant(460)'};
prog.wiring.outputs = {'ua -> Transform.ua', 'ub -> Transform.ub'};
```

**IMPORTANT — `Constant()` values:** Use **literal numeric values** in `Constant(...)`, e.g., `Constant(460)`, NOT variable names like `Constant(Vdc)`. Variable names require the variable to exist in the base workspace at compile time. Literal values always work.

**Monitoring block outputs:** For programmatic blocks with `role: 'monitoring'` whose outputs are for logging only (not wired anywhere), use either:
- `wiring.outputs = {}` (empty — preferred, outputs are auto-terminated)
- `'y_mon -> signal_logging_only'` (recognized termination keyword — output is terminated)

Other recognized termination keywords: `terminate`, `none`, `unconnected`, `logging_only`.

**NOTE:** `executePlan` now builds programmatic blocks automatically (Phase 3a). It creates the block, writes the MATLAB Function code via Stateflow API, and wires using `addProgrammaticBlocks`. The wiring format above (`'inputName <- Source.port'`, `'outputName -> Target.port'`) is parsed automatically.

### Programmatic Smoke Test (`prog.smoke_test`)

Optional but **recommended** for MATLABFunction blocks. Runs the function standalone with known inputs and checks output assertions BEFORE wiring into the model. Catches sign errors, convention mistakes, and logic bugs cheaply.

```matlab
prog.smoke_test = struct( ...
    'inputs', {{pi/2, 460, 376.99}}, ...  % NON-DEGENERATE input (sin≠0, exercises cross terms)
    'assertions', {{ ...
        struct('output_index', 2, 'type', 'sign', 'expected', 1) ...  % y2 must be positive
    }} ...
);
```

**Assertion types:**
| Type | Fields | Checks |
|------|--------|--------|
| `sign` | `.output_index`, `.expected` (+1 or -1) | Output has expected sign |
| `near` | `.output_index`, `.expected`, `.tolerance` | Output within relative tolerance of expected |
| `bounded` | `.output_index`, `.value` | |output| < value |

**How to choose test inputs:**
1. **Avoid degenerate inputs.** Do NOT use inputs where terms cancel out (e.g., θ=0 makes sin=0, hiding sign errors in cross terms). Pick inputs that exercise ALL code paths and ALL terms.
2. **Use inputs where the output is hand-calculable but non-trivial.** For rotations, use θ=π/2 (cos=0, sin=1) — this exposes sign errors in cross terms. For switching logic, pick a mid-sector input.
3. **Assert the property most likely to be wrong.** For transforms: output sign. For switching: output value. For scaling: output magnitude.

Example — rotation/transform smoke test (good):
```matlab
% At theta = pi/2: sin=1, cos=0, cross terms dominate.
% Correct: y2 = u1_static > 0. Wrong signs: y2 = -u1_static < 0.
prog.smoke_test = struct( ...
    'inputs', {{pi/2, 460, 376.99}}, ...
    'assertions', {{struct('output_index',2, 'type','sign', 'expected',1)}} );
```
Bad: theta=0 gives correct output regardless of sign errors (sin=0 kills cross terms).

---

## 9. Top-Level Spec Struct (for `validateSpec` / `executePlan`)

The **spec** is the Stage A extraction output. It describes the source paper, not the build plan.
`validateSpec` checks this before building. All struct-array fields use `.name` as the key field.

```matlab
spec.model_name          = 'MyDynamicSystem';    % REQUIRED: char
spec.status              = 'extracted';          % REQUIRED: 'extracted' | 'normalized' | 'built'
spec.paper_id            = 'SourcePaper2023';    % optional: citation key
spec.paper_title         = '...';               % RECOMMENDED (document-mode): full title
spec.paper_authors       = '...';               % RECOMMENDED (document-mode): author names
spec.paper_journal       = '...';               % RECOMMENDED (document-mode): venue, vol, year
spec.implementation_ref  = struct( ...          % from 1b-iv scan
    'equations', [5 6 7 8 9 10 19 20 21], ...  % eq numbers paper says to build with
    'source_text', 'model constructed using (5-8), (9,10), and (15-31)', ...
    'page', 4);                                 % [] if no directive found

% STRUCT ARRAYS with .name field (NOT cell arrays of strings, NOT keyed structs):
spec.states     = struct('name', {'x', 'v'});                          % struct array
spec.inputs     = struct('name', {'F', 'TL'}, 'source', {'external', 'external'});
spec.outputs    = struct('name', {'x', 'v', 'a'});                     % struct array
spec.parameters = struct('name', {'k','b','m'}, ...                    % struct array
                         'value', {100, 10, 1}, ...
                         'unit', {'N/m','Ns/m','kg'}, ...
                         'source', {'Table 1','Table 1','Table 1'});
spec.components = struct('name', {'Dynamics'}, ...                     % struct array
                         'builder', {'odeBuilder'}, ...
                         'description', {'Spring-damper ODEs'});

spec.equations_raw_latex  = {'\dot{x} = v', '\dot{v} = (F - k*x - b*v)/m'};   % cell of char
spec.equation_refs        = {'Eq. 1', 'Eq. 2'};    % cell, same length as equations

spec.validation_targets   = struct('name', {'Overshoot'}, 'signal', {'x'}, ...
                                   'type', {'peak'}, 'expected', {1.5}, ...
                                   'tolerance_pct', {10}, 'source', {'Fig. 3'});

spec.simulation_setup = struct('solver','ode45','stopTime','10','maxStep','auto');
spec.physical_subsystems  = {};                    % cell (can be empty)
spec.validation_figures   = {};                    % cell of structs (optional)

% Physical invariants — checked immediately after first simulation.
% Catches sign errors, divergence, and physically impossible states.
% NOTE: Use CELL ARRAY (not struct array) because different conditions
% have different fields (e.g., 'bounded' has 'value', others don't).
spec.invariants = { ...
    struct('signal','wr', 'condition','final_positive', ...
           'description','Motor speed must be positive', ...
           'reason','Forward rotation under forward voltage'), ...
    struct('signal','Te', 'condition','bounded', 'value', 5000, ...
           'description','Torque must not diverge', ...
           'reason','Physical torque bounded by magnetic saturation'), ...
    struct('signal','wr', 'condition','finite', ...
           'description','Speed must remain finite', ...
           'reason','Divergent speed indicates unstable model') ...
};
```

**CRITICAL FORMAT RULES (prevent validateSpec errors):**
1. `spec.states`, `spec.inputs`, `spec.outputs`, `spec.parameters`, `spec.components` → **struct arrays with `.name` field**
2. `spec.inputs` → each entry needs `.source` field (`'external'` or component name that produces the signal). Inputs fed by programmatic blocks (e.g., InputGenerator producing u1/u2) should use `source: 'external'` — programmatic blocks are plan-level implementation details, not spec-level components.
3. `spec.parameters` → struct array with `.name`, `.value`, `.unit`, `.source` fields (NOT a keyed struct like `struct('k', 100, 'b', 10)`)
4. `spec.equations_raw_latex` → cell array of char (matches `plan.components(i).spec.equations`)
5. States should be in alphabetical order (warning if not)

---

## 9b. Physical Invariants (`spec.invariants`) — RECOMMENDED

Physical invariants are simple assertions checked **immediately after the first simulation** (before test evaluation). They catch sign errors, divergence, and nonsensical outputs that no numerical tolerance check would flag.

**When to define:** Always. Every physical system has invariants that don't depend on the paper's specific results. Derive from system type:
- Motors: speed positive, torque bounded, currents finite
- Thermal: temperature > ambient, temperature finite
- Mechanical: position bounded, velocity finite
- Electrical: voltages bounded, currents finite

**Format (CELL ARRAY — different conditions have different fields):**
```matlab
spec.invariants = { ...
    struct('signal','wr', 'condition','final_positive', ...
           'description','Motor speed must be positive', ...
           'reason','Forward rotation under forward excitation'), ...
    struct('signal','Te', 'condition','bounded', 'value',5000, ...
           'description','Torque must not exceed 5000 N-m', ...
           'reason','Physical limit for this motor class') ...
};
```

**Available conditions:**
| Condition | What it checks | `value` field |
|-----------|---------------|---------------|
| `positive` | All samples >= 0 | — |
| `negative` | All samples <= 0 | — |
| `final_positive` | Last sample > 0 | — |
| `final_negative` | Last sample < 0 | — |
| `bounded` | max\|data\| < value | required |
| `finite` | No Inf/NaN | — |
| `final_near` | Final ≈ value (within `tolerance`) | required |
| `max_below` | max(data) < value | required |
| `min_above` | min(data) > value | required |
| `monotonic_increase` | Non-decreasing | — |
| `monotonic_decrease` | Non-increasing | — |

**Key principle:** Invariants should be **generous** bounds derived from physics, not tight tolerances from the paper. If your motor's rated torque is 1100 N-m, set `bounded` at 5000 — this catches a 600x error but won't false-alarm on 2x overshoot during startup.

---

## 9c. Validation Figures (`spec.validation_figures`) — RECOMMENDED

Validation figures define paper figures that the model should reproduce. `executePlan` automatically generates these plots from the simulation output — the reader compares them visually against the paper.

**Format:**
```matlab
spec.validation_figures = { ...
    struct( ...
        'paper_fig', 'Fig N', ...
        'title', 'Primary Output and Disturbance', ...
        'signals', {{'y4', 'd1'}}, ...
        'units', {{'units_y', 'units_d'}}, ...
        'layout', 'stacked', ...            % 'stacked' | 'overlay' | 'xy'
        't_range', [0 4], ...               % optional x-axis limits
        'y_ranges', {{[0 200], [0 150]}}, ... % optional per-signal y limits
        'targets', struct('value', {188.5, 100}, 'label', {'steady-state', 'rated input'}) ... % optional reference lines
    ), ...
    struct( ...
        'paper_fig', 'Fig M', ...
        'title', 'Coupling Output and Algebraic State', ...
        'signals', {{'z1', 'y1'}}, ...
        'units', {{'units_z', 'units_y'}}, ...
        'layout', 'stacked', ...
        't_range', [0 4], ...
        'y_ranges', {{[-500 1200], [-600 600]}} ...
    ) ...
};
```

**Fields:**
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `paper_fig` | char | yes | Paper figure number (e.g., `'Fig 16'`) |
| `title` | char | yes | Descriptive title for the plot |
| `signals` | cell of char | yes | Signal names to plot (must match logged signals) |
| `units` | cell of char | no | Unit strings for axis labels, one per signal |
| `layout` | char | no | `'stacked'` (default), `'overlay'`, or `'xy'` |
| `t_range` | [t0 t1] | no | Time axis limits |
| `y_ranges` | cell of [y0 y1] | no | Per-signal y-axis limits |
| `targets` | struct array | no | Horizontal reference lines (`.value`, `.label`) |
| `expected_range` | struct array | no | Per-signal expected bounds (see below) |
| `notes` | char | no | Conversion notes (e.g., 'wr_mech = wr_elec * 2/P') |

**`expected_range` field (RECOMMENDED for each signal):**

Declares quantitative expectations so `checkFigureConsistency` can catch "model runs but scenario not exercised" failures. This is the PRIMARY defense against flat/zero validation plots.

```matlab
'expected_range', struct( ...
    'final', [100 140], ...     % final value must be in [100, 140]
    'peak', [120 160], ...      % max value must be in [120, 160]
    'trough', [-10 5], ...      % min value must be in [-10, 5] (optional)
    'nonzero', true)            % signal must be non-trivial (default: true)
```

If `expected_range` is omitted, `checkFigureConsistency` still checks for flat/zero signals (the most common failure mode). Adding explicit ranges catches subtler issues like wrong magnitude or sign.

**Pipeline behavior:**
- `executePlan` runs `checkFigureConsistency` after invariants, before `evaluateTests`
- FAIL on any signal → blocks validation (same as invariant failure)
- For each `validation_figures` entry, extracts the named signals from `logsout` and generates a figure
- Output filename: `fig<N>_reproduction.png` (e.g., `fig16_reproduction.png`)
- The report places these figures in the validation section so the reader can visually compare against the paper

---

## 9d. Report Content (`spec` narrative fields) — RECOMMENDED

These fields carry LLM-authored narrative content from Stage A (when understanding is deepest) through to the report. `buildReportStruct` passes them through; `fillReport` renders them. If absent, the report falls back to mechanical extraction (equations + parameter tables only).

```matlab
% --- Report style (user preference, captured at start) ---
spec.report_style = 'detailed';   % 'detailed' | 'simple' | 'executive' | 'none'
% 'detailed'  — full derivation, all diagrams, experiments, user guide (default)
% 'simple'    — skip equations section, minimal tables, results + figures only
% 'executive' — 1-page summary: what was built, validation result, key plot
% 'none'      — no report generated

% --- Narrative content (LLM-written in Stage A) ---
spec.narrative_intro = [ ...
    'This model captures the dynamics of a variable-speed wind turbine ' ...
    'operating across below-rated, rated, and above-rated wind conditions. ' ...
    'The aerodynamic subsystem uses a Cp-lambda-beta lookup to compute rotor torque, ' ...
    'while the drivetrain represents the flexibility between rotor and generator ' ...
    'as a two-mass shaft system. A PI pitch controller regulates generator speed ' ...
    'in above-rated conditions by adjusting blade pitch angle.'];

spec.subsystem_context = { ...
    struct('name', 'Aerodynamics', ...
           'context', ['The Cp-lambda model is the standard way to capture ' ...
                       'how a wind turbine rotor converts kinetic wind energy to shaft torque. ' ...
                       'At any operating point, the power coefficient Cp depends on tip-speed ' ...
                       'ratio (lambda) and blade pitch (beta). This is the ''engine'' of the system.']), ...
    struct('name', 'Drivetrain', ...
           'context', ['The two-mass model captures the dominant torsional mode of the ' ...
                       'low-speed shaft connecting the rotor hub to the gearbox. ' ...
                       'Without shaft flexibility, the natural frequency response ' ...
                       'of the drivetrain would be missing from the model.']) ...
};

spec.user_guide = [ ...
    'To change wind conditions, edit the WindProfile MATLAB Function block. ' ...
    'Pitch controller gains (Kp_pitch, Ki_pitch) are in the InitFcn callback — ' ...
    'increase Ki for faster above-rated regulation (but watch for oscillation). ' ...
    'The model assumes constant air density (rho=1.225); for altitude effects, modify rho. ' ...
    'To add tower dynamics, insert a tower fore-aft DOF between Aerodynamics and Drivetrain.'];

% --- Interesting experiments (LLM-designed, run in Stage E) ---
spec.interesting_experiments = { ...
    struct('name', 'Emergency Pitch to Feather', ...
           'description', 'At t=200s, command beta=90 deg (feather). Observe rotor deceleration.', ...
           'setup', 'Modify WindProfile to hold 18 m/s; add step to beta_cmd at t=200.', ...
           'what_to_check', 'Rotor should decelerate to zero within ~30s. Generator torque drops instantly.'), ...
    struct('name', 'Turbulent Wind (Random Gusts)', ...
           'description', 'Replace step wind with Band-Limited White Noise around 14 m/s mean.', ...
           'setup', 'Replace WindProfile with BLN source: mean=14, variance=4, Ts=0.1.', ...
           'what_to_check', 'Pitch controller should keep omega_g near 122.9 despite gusts. Check std(omega_g) < 2 rad/s.') ...
};

% --- Derivation schematic path (generated in Stage A) ---
spec.illustration_path = '';       % System cartoon ("what is this?") — MATLAB figure
spec.derivation_schematic = '';    % Domain-appropriate derivation diagram (FBD, circuit, etc.)
```

**Field contracts:**

| Field | Type | When to populate | Fallback if missing |
|-------|------|-----------------|---------------------|
| `report_style` | char | Start of conversation (user preference) | `'detailed'` |
| `narrative_intro` | char | Stage A (during derivation or after extraction) | Auto-generated: `"{model}. N-state model with M equations."` |
| `subsystem_context` | cell of struct | Stage A (per-component physical meaning) | Subsystems listed without explanation |
| `user_guide` | char | Stage A (what can user tune/extend) | Omitted from report |
| `interesting_experiments` | cell of struct | Stage A (LLM-designed beyond-paper scenarios) | Only paper validation targets in report |
| `illustration_path` | char (path) | Stage A (MATLAB figure or extracted image) | No overview image in report |
| `derivation_schematic` | char (path) | Stage A (TikZ or MATLAB diagram) | No schematic in report |

**`interesting_experiments` struct fields:**
- `.name` — experiment title (char)
- `.description` — what this experiment does (1-2 sentences)
- `.setup` — how to configure the model for this experiment (char)
- `.what_to_check` — expected outcome / what makes it interesting (char)
- `.result` — (filled by Stage E) actual outcome after running (char, optional)
- `.figure` — (filled by Stage E) path to result figure (char, optional)

---

## 10. Complete Example (Minimal Working Plan)

```matlab
%% Spec
spec.model_name = 'SpringDamper';
spec.status = 'extracted';
spec.states = struct('name', {'v', 'x'});  % alphabetical
spec.inputs = struct('name', {'F'}, 'source', {'external'});
spec.outputs = struct('name', {'v', 'x'});
spec.components = struct('name', {'Dynamics'}, 'builder', {'odeBuilder'}, 'description', {'ODEs'});
spec.physical_subsystems = {};
spec.parameters = [struct('name','k','value',100,'unit','N/m','source','Given'), ...
                   struct('name','b','value',10,'unit','Ns/m','source','Given'), ...
                   struct('name','m','value',1,'unit','kg','source','Given')];
spec.equations_raw_latex = {'\dot{x} = v', '\dot{v} = (F - k*x - b*v)/m'};
spec.equation_refs = {'Eq. 1', 'Eq. 2'};

%% Plan
plan.name = 'SpringDamper';
plan.busMode = 'none';
plan.solver = 'ode45';
plan.stopTime = '10';

% Single component
plan.components(1).name = 'Dynamics';
plan.components(1).builder = 'odeBuilder';
plan.components(1).spec = struct( ...
    'equations', {{'\dot{x} = v', '\dot{v} = (F - k*x - b*v)/m'}}, ...
    'equation_refs', {{'Eq. 1', 'Eq. 2'}}, ...
    'params', struct('k', 100, 'b', 10, 'm', 1), ...
    'ic', struct('x', 0, 'v', 0));
plan.components(1).interface = struct( ...
    'externalInputs', {{'F'}}, ...
    'externalOutputs', {{'x', 'v'}});
plan.components(1).decision_path = struct( ...
    'tree_terminal_node', 'odeBuilder (novel nonlinear ODEs)', ...
    'is_combinational', false, ...
    'is_standard_block', false, ...
    'findBlock_searched', true, ...
    'findBlock_result', 'no standard block for coupled spring-damper ODE');

% No wiring needed (single component)
plan.wiring = struct('from_component',{}, 'from_port',{}, 'to_component',{}, 'to_port',{});

% Top I/O
plan.topInputs = {struct('name','F','component','Dynamics','port','F','value','0')};
plan.topOutputs = {struct('name','x','component','Dynamics','port','x'), ...
                   struct('name','v','component','Dynamics','port','v')};

%% Execute
[mdl, result] = executePlan(plan, 'Spec', spec, 'OutputDir', outputDir, ...
    'SessionStart', sessionStart);
```

---

## 11. Common Mistakes (and what happens)

### Plan/Builder Mistakes

| Mistake | Error you'll see |
|---------|-----------------|
| `spec.ic = [0, 0]` (vector) | "IC must satisfy isstruct" |
| `spec.equations = {'d_x = v'}` (no `\dot{}`) | odeBuilder error: "Detected N equation(s) with d_ prefix... requires \dot{}" |
| `spec.equations = {'dx = v'}` (no `\dot{}`) | "0 ODE, 1 algebraic" — silently treated as algebraic |
| Missing `comp.interface.externalInputs` | "0 inputs" warning, composition fails |
| `topInputs` as struct array (not cell) | "Dot indexing not supported" |
| `topOutput.source` (wrong field name) | "Unrecognized field name 'component'" — use `.component` |
| Missing `equation_refs` | validatePlan BLOCKS plan |
| Missing `decision_path.tree_terminal_node` | validatePlan BLOCKS plan |
| Port name in wiring doesn't match interface | "Port X not found on component Y" |
| `plan.programmatic` as struct array | "Too many input arguments" on `isempty(pg.type)` — use **cell array** |

### Top-Level Spec Mistakes (validateSpec)

| Mistake | Error you'll see |
|---------|-----------------|
| `spec.states = {'x','v'}` (cell of strings) | "Dot indexing is not supported for variables of this type" |
| `spec.parameters = struct('k',100,'b',10)` (keyed struct) | "Unrecognized field name 'name'" |
| `spec.inputs` missing `.source` field | "Input 'u1' needs a source component" ERROR |
| Missing `spec.components` | "No components defined" warning |
| `spec.states` not alphabetical | Warning (non-blocking) |
