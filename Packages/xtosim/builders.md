# Builders Reference

MATLAB APIs that turn specs into Simulink models. Each builder handles a different class of system:

| Builder | System type | Output |
|---------|------------|--------|
| `existing` | Pre-built subsystem from a loaded model | Copied subsystem with discovered port interface |
| `odeBuilder` | Continuous ODEs, algebraic equations, events/resets, single-mode dynamics | Flat Simulink model with auto-wired blocks |
| `odeBuilder_cps` | Hybrid/CPS systems with distinct operating modes and discrete transitions | Stateflow chart with Simulink-based states |
| `lookupTableBuilder` | Empirical data, engine maps, tire curves, efficiency maps | 1-D/2-D/n-D Lookup Table blocks |
| `blocksetBuilder` | Standard engineering components (tires, batteries, motors, etc.) | Validated built-in domain blocks (Simscape, VDX, Aero, etc.) |
| `simscapeBuilder` | Physical networks (circuits, mechanisms, thermal, hydraulic) | Simscape model with bidirectional energy ports |
| `stateflowBuilder` | Pure discrete FSMs, mode managers, supervisory control, protocols | Stateflow chart with Inport/Outport blocks |
| `simeventsBuilder` | Discrete-event simulation, queues, hybrid DES-continuous | SimEvents entity-flow model (optionally hybrid) |

### Builder Selection Priority (decision tree order)

When routing a component to a builder, evaluate in this order — stop at the first match:

0. **Already built** (subsystem exists in a loaded model) → `existing`
1. **Pure combinational logic** (no states, no dynamics) → if a standard block exists (findBlock): `blocksetBuilder`; otherwise: `plan.programmatic`
2. **Empirical data** (tables, curves, maps) → `lookupTableBuilder`
3. **Known standard block** (Level 1/2/3 recognition — see Block Recognition Table below) → `blocksetBuilder`
4. **Physical network** (bidirectional energy, circuit, mechanism) → `simscapeBuilder`
5. **Discrete-event** (queues, entities, resource allocation) → `simeventsBuilder`
6. **Pure discrete logic** (FSM, protocol, mode manager) → `stateflowBuilder`
7. **Mode-switching ODEs** (different equations per mode) → `odeBuilder_cps`
8. **Novel nonlinear ODEs** (none of the above) → `odeBuilder`

`existing` is the first check (don't rebuild what already works). `odeBuilder` is the **last resort**, not the default. See `SKILL.md` Stage B2 for the full decision tree diagram.

---

## odeBuilder — Flat Model Builder

### Function Signature

```matlab
[c, m, cellEq] = odeBuilder(eqStr, modelName)
[c, m, cellEq] = odeBuilder(eqStr, modelName, 'Params', params, 'IC', ic, ...)
```

### Inputs
| Parameter | Type | Description |
|-----------|------|-------------|
| `eqStr` | char array | Equations separated by double quotes |
| `modelName` | char array | Name for the Simulink model |
| `'Params'` | struct (optional) | Parameter name-value pairs (auto-assigns to workspace, sets Constants) |
| `'IC'` | struct (optional) | Initial conditions keyed by state name |
| `'Logging'` | logical (optional) | Enable signal logging (default: true) |
| `'Arrange'` | logical (optional) | Auto-arrange layout (default: true) |

### Outputs
| Output | Description |
|--------|-------------|
| `c` | Computational graph object |
| `m` | Model metadata (varNode map, block info) |
| `cellEq` | Cell array of individual equation strings |

### What odeBuilder handles internally (no post-build fixes needed)
- Constant block values set to workspace variable names (not '1')
- Unary-minus decomposition (uses 0, not 1, for placeholder)
- pow blocks replaced with Product(u*u) for integer powers
- Algebraic outputs wired to ODE consumers
- Integrator ICs set from the IC struct
- Event/reset logic (zero-crossing + integrator reset)
- Layout arrangement (when Arrange=true)

---

## Equation String Format

All equations are wrapped in double quotes and concatenated into a single char array:

```matlab
str = '"equation1" "equation2" "equation3"';
```

### CRITICAL: Use Single Backslash for LaTeX Commands

odeBuilder uses **LaTeX notation** for derivatives and symbols. In MATLAB char arrays, use **single backslash** `\dot{x}`, NOT double backslash `\\dot{x}`.

| Correct | Wrong | What happens |
|---------|-------|-------------|
| `'\dot{x} = -a*x'` | `'\\dot{x} = -a*x'` | Double backslash creates an **Outport** for the derivative instead of an **Integrator** with feedback. The model will not have closed-loop state dynamics. |

### Supported LaTeX Notation

| Notation | Meaning | Example |
|----------|---------|---------|
| `\dot{x}` | dx/dt (creates Integrator) | `"\dot{x} = -a*x + u"` |
| `\ddot{x}` | d^2x/dt^2 (creates 2 Integrators) | `"\ddot{x} = -k*x"` |
| `\sin(x)`, `\cos(x)`, `\tan(x)` | Trigonometric functions | `"\dot{x} = \sin(\theta)"` |
| `\exp(x)` | Exponential | `"y = \exp(-a*t)"` |
| `\log(x)` | Natural logarithm | `"y = \log(x)"` |
| `\abs(x)` | Absolute value | `"y = \abs(x)"` |
| `\theta`, `\omega`, `\tau`, `\psi` etc. | Greek letter variable names | `"\dot{\omega} = \tau/I"` |
| `t` | Time (independent variable) | `"\dot{x} = \sin(t)"` |
| `+`, `-`, `*`, `/`, `^` | Standard arithmetic | `"\dot{x} = -k*x^2"` |

**Unsupported functions:** `sqrt`, `atan2`, `sign`, `max`, `min`, `tanh`, `sinh`, `cosh`, `mod`, `floor`, `ceil` — handle during Phase 4 normalization:
- **If an algebraic equivalent exists**: `sqrt(x)` → `(x)^(0.5)`, `sinh(x)` → `(\exp(x)-\exp(-x))/2`
- **If no equivalent exists**: extract as named variable, replace with Simulink Fcn block in Phase 8

### Equation Types

| Type | Syntax | odeBuilder generates | Example |
|------|--------|---------------------|---------|
| **ODE** | `"\dot{x} = f(x,u)"` | Integrator + computational graph + auto-wired feedback | `"\dot{x} = -2*x + b*u"` |
| **Output** | `"y = g(x)"` (y on LHS only) | Outport block + computational graph | `"y = x1 + x2"` |
| **Algebraic** | `"y = f(y, ...)"` (y on both sides) | Algebraic Constraint block | `"y = 0.5*y + 3"` |
| **Event** | `"x >= 0"` | Compare To Zero + zero-crossing detection | `"x >= 0"` |
| **Reset** | `"v = -0.8*v, x = 0"` | Integrator reset via state port + Gain + IC block | `"v = -0.8*v, x = 0"` |

### Verified Examples

```matlab
% Spring-mass-damper
odeBuilder('"\dot{x} = v" "\dot{v} = (-k*x - c*v)/m"', 'smd', ...
    'Params', struct('k',10,'c',2,'m',1), 'IC', struct('x',1,'v',0));

% Lorenz attractor
odeBuilder('"\dot{x} = sigma*(y-x)" "\dot{y} = x*(rho-z)-y" "\dot{z} = x*y-beta*z"', ...
    'lorenz', 'Params', struct('sigma',10,'rho',28,'beta',8/3), ...
    'IC', struct('x',1,'y',1,'z',1));

% Bouncing ball (events + resets)
odeBuilder('"\dot{v} = -9.81" "\dot{h} = v" "h >= 0" "v = -0.8*v, h = 0"', ...
    'bouncing_ball', 'IC', struct('h',10,'v',0));

% Algebraic output
odeBuilder('"\dot{x} = -a*x" "y = b*x"', 'alg_test', ...
    'Params', struct('a',1,'b',2), 'IC', struct('x',5));
```

---

## Variable Naming Rules

### Avoid MATLAB built-in function names

| Collision | Fix |
|-----------|-----|
| `cd` | Rename to `cda`, `Cd`, or `c_d` |
| `mean` | Rename to `mu` or `avg` |
| `length` | Rename to `len` or `L` |
| `i`, `j` | Use `ii`, `jj` or descriptive names |

**Check before building:** `which('varname')` — if it returns a path, the name is taken.

### Variable names in equation strings

- Use short, simple names: `V`, `x1`, `Ft`, `omega`
- LaTeX Greek: `\theta`, `\omega`, `\psi` etc. are valid variable names
- Avoid multi-word underscored names (`motor_speed`) — may cause parsing issues
- The same variable name in multiple equations tells odeBuilder to auto-wire them

---

## odeBuilder_cps — CPS/Hybrid Model Builder

`odeBuilder_cps` builds Stateflow charts with Simulink-based states for cyber-physical / hybrid systems. Each operating mode gets its own SimulinkBasedState whose internal dynamics are built by `odeBuilder`. Transitions with guards and reset actions connect the modes.

### Function Signature

```matlab
[modelName, chartObj] = odeBuilder_cps(modes, transitions, params, outputs, modelName, opts)
```

### Inputs

| Parameter | Type | Description |
|-----------|------|-------------|
| `modes` | struct array | One entry per operating mode (see below) |
| `transitions` | struct array | One entry per transition (see below) |
| `params` | struct | Parameter name-value pairs (assigned to base workspace) |
| `outputs` | cell array of strings | State names to log as chart outputs |
| `modelName` | string | Name for the Simulink model |
| `opts` | struct (optional) | Options: `solver` (default `'ode45'`), `stopTime` (default `'10'`), `maxStep` (default `'auto'`), `chartName` (default = modelName), `inputs` (cell array of variable names), `state_topology` (`'shared'` or `'disjoint'`), `derived_outputs` (struct array for disjoint-state common-frame outputs; each: `.name`, `.expressions` as cell `{mode1, expr1, mode2, expr2, ...}`) |

### Outputs

| Output | Description |
|--------|-------------|
| `modelName` | string, model name (same as input) |
| `chartObj` | Stateflow.Chart object handle |

### Mode Struct

| Field | Type | Description |
|-------|------|-------------|
| `.name` | string | Mode name (e.g., `'FreeFall'`, `'Slipping'`) — must be valid MATLAB identifier |
| `.equations` | string | odeBuilder equation string for this mode's dynamics. Empty `''` = no dynamics (static mode). |
| `.ic` | struct or `'from_transition'` | Initial conditions keyed by state name, e.g., `struct('v', 0, 'y', 10)`. Use `'from_transition'` for modes only entered via transition actions (the builder internally converts this to an empty struct). |

### Transition Struct

| Field | Type | Description |
|-------|------|-------------|
| `.source` | string | Source mode name. Empty `''` = default transition (initial mode) |
| `.destination` | string | Destination mode name |
| `.guard` | string | Stateflow guard expression (without `[]`). Uses `ModeName.varName` for remote state access |
| `.action` | string | Stateflow action expression (without `{}`). Can set states via `ModeName.varName = expr` |

### opts.inputs — Chart-Level Input Signals

By default, `odeBuilder_cps` treats all variables that aren't states or outputs as **chart parameters**. When a variable should be a **time-varying input signal**, list it in `opts.inputs`:

```matlab
opts.inputs = {'F', 'Te'};
[modelName, chartObj] = odeBuilder_cps(modes, transitions, params, outputs, 'my_model', opts);
```

**After build**, wire external signal sources to the chart's input ports:
```matlab
F_ts = timeseries([0; 0; 1300; 1300], [0; 1; 6; 15]);
assignin('base', 'F_ts', F_ts);
add_block('simulink/Sources/From Workspace', [mdl '/F_input'], ...
    'VariableName', 'F_ts', 'SampleTime', '0', 'Interpolate', 'on');
add_line(mdl, 'F_input/1', 'chartName/1', 'autorouting', 'smart');
```

### CPS Topology Classification

Two fundamental patterns exist. The LLM must identify which one applies:

| Aspect | Shared-state | Disjoint-state |
|--------|-------------|----------------|
| State vector | Same across all modes | Different per mode |
| Example | PLL (3 modes, same [vi,vp1,vp,φv,φref]) | Pole vaulter (Run_up:[px,py,vx,vy], Take_off:[r,θ,...]) |
| Outputs | Plot any state directly (always meaningful) | Stale after mode exit; need common-frame transform |
| Transition action | Simple reset (φ = φ-1) | Coordinate transform (Cartesian→polar) |
| Validation | Check non-reset states are continuous | Check physical invariants (energy conservation) |

**Disjoint-state pitfalls:**
- Logged outputs hold stale values after mode exit (flat line)
- Must stitch signals in a common coordinate frame for plotting
- Transition actions are effectively inverse kinematics — silent bugs if wrong

### Local Variable Detection in Transition Actions

The builder auto-classifies symbols in transition guards/actions:
- **Parameters:** symbols that exist in the `params` struct → `Scope = 'Parameter'`
- **Locals:** symbols that appear on the LHS of `var = expr` in actions AND are NOT in params → `Scope = 'Local'`
- **State accessors:** `ModeName.state` patterns → already handled by Stateflow

Example: `'L = Take_off.r + Pole; Fly.xf = L*cos(th)'` → `L` is Local (temporary), `Pole` is Parameter.

### CSA Initialization (Critical)

Stateflow has **two** IC mechanisms for ContinuousStateAttributes states:
1. Integrator IC block value (set by odeBuilder)
2. Chart-level `Stateflow.Data` `Props.InitialValue`

The builder now sets **both** from `modes(i).ic`. It also clears `slprj/` cache before build to prevent stale IC contamination from previously loaded models with the same name.

### Disjoint-State Post-Simulation Workflow

For `opts.state_topology = 'disjoint'`, outputs are stitched after simulation:

```matlab
% After simulation:
cps_derived = evalin('base', 'cps_derived_outputs');
stitched = stitchCPSOutputs(cps_derived, params);
plot(stitched.x_pos.Time, stitched.x_pos.Data);

% Transition verification (energy/continuity checks):
% Requires mode_id_log in workspace (inferred from state activity)
results = validateCPSTransitions(mdl, simOut);
```

**Why post-simulation?** SimulinkBasedStates do not support `entry`/`during` actions, so `mode_id` cannot be reliably output from within the chart. Mode identity is inferred from which states are actively changing.

### Internal Phases

`odeBuilder_cps` runs these phases internally:

- **Phase 0a** — Validate `from_transition` modes (not initial, has incoming action)
- **Phase 0b** — Validate guard directions (warn if guard true at source mode ICs)
1. **Build per-mode dynamics** — calls `odeBuilder` for each mode with non-empty equations
2. **Create Simulink model and Stateflow chart** — clears `slprj` cache, `ChartUpdate = 'CONTINUOUS'`, `ActionLanguage = 'MATLAB'`
3. **Declare chart-level output data** — one `Stateflow.Data` with `Scope = 'Output'` per output name
3b. **Declare chart-level parameters and locals** — auto-detects symbols; classifies as Parameter (known value) or Local (action temporary)
4. **Create SimulinkBasedStates and copy dynamics** — sets `ContinuousStateAttributes` + `Props.InitialValue` on integrators; handles `from_transition` modes (CSA with IC=0)
5. **Add transitions** — creates `Stateflow.Transition` with guards and actions, auto-positions
6. **Add logging** — `ToWorkspace` blocks for each chart output
6b. **Disjoint-state metadata** — stores `cps_derived_outputs` in workspace (if `state_topology='disjoint'`)
7. **Configure solver, InitFcn, save** — solver/stopTime, `ReturnWorkspaceOutputs = 'off'`, stores verify specs

### Stateflow API Details

| API | Purpose |
|-----|---------|
| `Stateflow.SimulinkBasedState(chart)` | Create a Simulink-based state inside a chart |
| `set_param(integrator, 'ContinuousStateAttributes', '''varName''')` | Enable remote access as `ModeName.varName` |
| `Stateflow.Transition(chart)` | Create a transition |
| `t.LabelString` | Set label: `'[guard]{action}'` |
| `t.SourceOClock`, `t.DestinationOClock` | Position (0=top, 3=right, 6=bottom, 9=left) |
| `Stateflow.Data(chart)` | Create chart-level data |
| `chart.ChartUpdate = 'CONTINUOUS'` | Enable continuous-time dynamics |
| `chart.ActionLanguage = 'MATLAB'` | Use MATLAB (not C) action language |

### Zeno Avoidance Pattern

For systems with potentially infinite events (bouncing ball → rest):

1. **Active mode** — has full dynamics
2. **Terminal mode** — no dynamics (empty equations), acts as a sink
3. **Self-transition** with energy threshold: `[abs(ModeName.v) >= threshold]`
4. **Exit transition** with opposite condition: `[abs(ModeName.v) < threshold]`

### Example: Bouncing Ball

```matlab
modes(1).name = 'FreeFall';
modes(1).equations = '"\dot{v} = -g" "\dot{y} = v"';
modes(1).ic = struct('v', 0, 'y', 10);

modes(2).name = 'Rest';
modes(2).equations = '';
modes(2).ic = struct();

transitions(1) = struct('source','', 'destination','FreeFall', 'guard','', 'action','');
transitions(2) = struct('source','FreeFall', 'destination','FreeFall', ...
    'guard','FreeFall.y <= 0 && abs(FreeFall.v) >= 0.5', ...
    'action','FreeFall.v = -0.8 * FreeFall.v; FreeFall.y = abs(FreeFall.y);');
transitions(3) = struct('source','FreeFall', 'destination','Rest', ...
    'guard','FreeFall.y <= 0 && abs(FreeFall.v) < 0.5', 'action','');

params = struct('g', 9.81);
outputs = {'v', 'y'};
[mdl, ch] = odeBuilder_cps(modes, transitions, params, outputs, 'bouncing_ball');
```

---

## Per-Section Verification Patterns

### Pattern 1: Steady-State Check
```matlab
set_param(modelName, 'StopTime', '200');
simOut = sim(modelName);
% Check final value converged
```

### Pattern 2: Eigenvalue / Time Constant Check
```matlab
A = [a11 a12; a21 a22];
eigenvalues = eig(A);
tauSlow = -1/min(abs(real(eigenvalues)));
```

### Pattern 3: Step Response — does a positive step produce the physically expected response?

### Pattern 4: Unit Consistency — are outputs in expected units?

---

## Normalization Rules

Normalize equations **before** building.

### Rule 1: Convert higher-order to first-order form

```latex
m\ddot{x} + c\dot{x} + kx = u  →  \dot{x1} = x2,  \dot{x2} = (u - c*x2 - k*x1)/m
```

### Rules 2–6 (concise)
2. **Name states** canonically unless paper symbols are simple. 3. **Preserve mapping**. 4. **Separate** ODEs, outputs, algebraics, events. 5. **Preserve events**. 6. **Do not fabricate** — record ambiguities.

### Rule 7: Intermediate variables consumed by ODEs

odeBuilder auto-sorts algebraic equations before ODEs. However, **do NOT blindly inline** algebraic variables into ODEs — inlining kills intermediate signals and makes subsystems dead. Only inline when the intermediate is trivial with no other consumers.

---

## Pipeline Execution Order

**Phase 11 (hierarchy creation) runs BEFORE Phase 8 (programmatic blocks)**:

1. `odeBuilder` builds the flat model (Phase 7) → returns `c`, `m`, `cellEq`
2. `createHierarchy` partitions into subsystems (Phase 11) — uses `c`, `m`, `cellEq`
3. `cleanupHierarchy` cleans ports (Phase 11c-fix)
4. Programmatic blocks added to hierarchical model (Phase 8)

**Why?** `createHierarchy` uses metadata from the build. Adding blocks first would make metadata stale.

**Port naming:** After `cleanupHierarchy`, the LLM names ports using physics knowledge.

**Prefer keeping intermediates** when they serve as subsystem boundaries (Te, ids, iqs, etc.).

---

## lookupTableBuilder — Empirical Data Builder

### Function Signature

```matlab
[blkPath, meta, info] = lookupTableBuilder(spec, mdl)
[blkPath, meta, info] = lookupTableBuilder(spec, mdl, 'DataSource', 'workspace', 'VarPrefix', 'eng')
info = lookupTableBuilder('info')
```

### Spec Struct Fields

| Field | Required | Description |
|-------|----------|-------------|
| `.name` | Yes | Block name |
| `.type` | No | `'1D'`, `'2D'`, or `'nD'`. Auto-detected from breakpoints length if omitted. |
| `.breakpoints` | Yes | Cell array of breakpoint vectors: `{bp1}`, `{bp1, bp2}`, etc. |
| `.tableData` | Yes | Numeric array matching breakpoint dimensions |
| `.inputNames` | No | Cell array of input signal names |
| `.outputName` | No | Output signal name |
| `.outputUnit` | No | Unit string for annotation |
| `.inputUnits` | No | Cell array of input unit strings, one per breakpoint dimension |
| `.interpMethod` | No | `'Linear'` (default), `'Nearest'`, `'Cubic spline'` |
| `.extrapMethod` | No | `'Clip'` (default), `'Linear'` |
| `.subsystem` | No | Target subsystem path |
| `.source_equation` | No | Traceability: original equation/curve reference |
| `.recognized_model` | No | Traceability: classified model type |

### Name-Value Options

| Option | Default | Description |
|--------|---------|-------------|
| `'DataSource'` | `'inline'` | `'inline'` embeds data in block; `'workspace'` uses workspace variables |
| `'VarPrefix'` | `spec.name` | Prefix for workspace variable names |
| `'Position'` | auto | Block position `[l t r b]` |
| `'Connect'` | `struct()` | Auto-wiring: `.inputs` cell, `.output` string |

### Outputs

| Output | Description |
|--------|-------------|
| `blkPath` | Full Simulink path to created block |
| `meta` | Struct: `.block_path`, `.dimensions`, `.breakpoints`, `.table_size`, `.input_ports`, `.output_ports`, `.parameters` (if workspace), `.traceability` |
| `info` | Struct: `.elapsed_s`, `.warnings`, `.dimensions`, `.data_source` |

### When to Use

- Source provides data tables or measured curves
- Any n-D empirical relationship (input breakpoints → output values)
- Coefficient tables indexed by operating conditions
- Efficiency/performance maps from manufacturer datasheets
- Any empirical relationship without a clean closed-form equation

### Example

```matlab
spec.name = '<ComponentName>';
spec.breakpoints = {bp1_vector, bp2_vector};   % one per input dimension
spec.tableData = dataMatrix;                    % n-D array matching breakpoint grid
spec.inputNames = {'<input1_name>', '<input2_name>'};
spec.outputName = '<output_name>';
[blk, meta] = lookupTableBuilder(spec, '<model_name>', 'DataSource', 'workspace');
```

---

## blocksetBuilder — Recognized Component Builder

**Design principle:** Use `blocksetBuilder` whenever the LLM **recognizes** an equation as mapping to a known Simulink block. Use `odeBuilder` only when the equation does NOT map to any known block. Recognition is the boundary — not component complexity or toolbox licensing.

### Block Recognition Table

Before routing equations to `odeBuilder`, check this table. If an equation matches a pattern below, use `blocksetBuilder` instead.

#### Continuous Dynamics

| Equation pattern | Recognized as | Library path | Key parameters |
|---|---|---|---|
| `y(t) = u(t - τ)` | Transport delay | `simulink/Continuous/Transport Delay` | `DelayTime` |
| `y(t) = u(t - d(t))` | Variable delay | `simulink/Continuous/Variable Transport Delay` | `MaximumDelay` |
| `Y(s)/U(s) = b(s)/a(s)` | Transfer function | `simulink/Continuous/Transfer Fcn` | `Numerator`, `Denominator` |
| `ẋ=Ax+Bu, y=Cx+Du` | State-space | `simulink/Continuous/State-Space` | `A`, `B`, `C`, `D`, `X0` |
| `Y(s)/U(s) = K·∏(s-z)/∏(s-p)` | Zero-pole-gain | `simulink/Continuous/Zero-Pole` | `Zeros`, `Poles`, `Gain` |
| `u = Kp·e + Ki·∫e + Kd·ė` | PID controller | `simulink/Continuous/PID Controller` | `P`, `I`, `D`, `N` |
| `G(s) = K/(τs+1)` | First-order lag | `simulink/Continuous/Transfer Fcn` | `Numerator=[K]`, `Denominator=[τ 1]` |
| `G(s) = ωn²/(s²+2ζωns+ωn²)` | Second-order | `simulink/Continuous/Transfer Fcn` | `Num=[wn^2]`, `Den=[1 2*z*wn wn^2]` |
| Padé approx of `e^{-sτ}` | Padé delay | `simulink/Continuous/Transfer Fcn` | Compute Padé coefficients |

#### Discontinuities / Nonlinearities

| Equation pattern | Recognized as | Library path | Key parameters |
|---|---|---|---|
| `y = clamp(u, lo, hi)` or `y = min(max(u,lo),hi)` | Saturation | `simulink/Discontinuities/Saturation` | `UpperLimit`, `LowerLimit` |
| `y = 0 for |u|<δ, y = u-δ·sign(u)` | Dead zone | `simulink/Discontinuities/Dead Zone` | `LowerValue`, `UpperValue` |
| `|ẏ| ≤ R` (rate-limited output) | Rate limiter | `simulink/Discontinuities/Rate Limiter` | `RisingSlewLimit`, `FallingSlewLimit` |
| Backlash / hysteresis band | Backlash | `simulink/Discontinuities/Backlash` | `BacklashWidth`, `InitialOutput` |
| `y = Δ·round(u/Δ)` | Quantizer | `simulink/Discontinuities/Quantizer` | `QuantizationInterval` |
| `y = sign(u)·max(0, |u|-Fc)` | Coulomb friction | `simulink/Discontinuities/Coulomb & Viscous Friction` | `Coulomb`, `Viscous` |
| `y = {a if u>on, b if u<off}` | Relay | `simulink/Discontinuities/Relay` | `OnSwitchValue`, `OffSwitchValue`, `OnOutputValue`, `OffOutputValue` |

#### Discrete-Time

| Equation pattern | Recognized as | Library path | Key parameters |
|---|---|---|---|
| `y[k] = u[k-1]` | Unit delay | `simulink/Discrete/Unit Delay` | `SampleTime`, `InitialCondition` |
| `y[k] = u[k-N]` | Integer delay | `simulink/Discrete/Tapped Delay` | `DelayOrder`, `SampleTime` |
| `Y(z)/U(z) = b(z)/a(z)` | Discrete TF | `simulink/Discrete/Discrete Transfer Fcn` | `Numerator`, `Denominator`, `SampleTime` |
| `x[k+1]=Ax[k]+Bu[k]` | Discrete state-space | `simulink/Discrete/Discrete State-Space` | `A`, `B`, `C`, `D`, `SampleTime`, `X0` |
| Continuous → sampled | Zero-order hold | `simulink/Discrete/Zero-Order Hold` | `SampleTime` |
| Digital filter coefficients | Discrete filter | `simulink/Discrete/Discrete Filter` | `Numerator`, `Denominator`, `SampleTime` |

#### Domain Blocksets (Toolbox-Specific)

Many Simulink toolboxes provide validated blocks for domain-specific components. **Do NOT memorize library paths — use `findBlock(keyword)` to discover them at runtime.** Library paths change between releases and depend on installed toolboxes.

The process for any domain-specific component:
1. Identify the component by keyword from the paper (e.g., "battery", "tire", "atmosphere model")
2. Call `findBlock('<keyword>')` to discover if a library block exists
3. Call `getBlockInfo(path)` to verify it matches the paper's model
4. If confirmed → `blocksetBuilder`; if no match → `odeBuilder`

This applies to ALL toolboxes: Vehicle Dynamics, Simscape Electrical, Aerospace, Powertrain, Robotics, Communications, etc. The static table above covers generic Simulink blocks; domain-specific blocks are discovered dynamically.

### Three-Level Recognition Protocol

The static table above is **Level 1** — a fast cache for common patterns. It is NOT exhaustive. Simulink has thousands of blocks across all toolboxes, and the table only covers ~30. When Level 1 doesn't match, use Levels 2 and 3 before defaulting to odeBuilder.

#### Level 1: Static pattern match (no MATLAB call)

Scan the tables above. If the equation form matches → `blocksetBuilder` immediately.

**Cost:** Zero. LLM reads the table from context.
**Coverage:** ~30 most common patterns (delays, TFs, PID, saturation, discrete, etc.)

#### Level 2: Keyword/name discovery via `findBlock`

When the paper **names** a component or technique but it's not in the Level 1 table:

```matlab
results = findBlock('servo valve');       % paper says "servo-valve actuator"
results = findBlock('dead zone');         % paper says "dead zone nonlinearity"
results = findBlock('Smith Predictor');   % paper says "Smith Predictor for dead-time"
results = findBlock('backlash');          % paper says "gear backlash of 0.5°"
results = findBlock('coulomb');           % paper says "Coulomb friction model"
```

If `findBlock` returns results, verify with `getBlockInfo`:
```matlab
info = getBlockInfo(results(1).library);
% Check: do the ports/params match what the paper describes?
```

If confirmed → `blocksetBuilder` with that library path.

**Cost:** 1-2 MATLAB calls.
**Coverage:** Anything whose block name contains a searchable keyword.

**When to invoke Level 2:**
- Paper names a specific technique, filter type, or component
- Paper describes behavior that sounds like a standard block
- You're unsure whether a standard block exists
- The equation involves something odeBuilder cannot express (delays, discrete, discontinuities with memory)

#### Level 3: Behavior-to-keyword inference

When the paper describes **behavior** without naming a block:

| Paper says | LLM infers | `findBlock` query |
|---|---|---|
| "output cannot exceed ±25 V" | Clamping/limiting | `findBlock('saturation')` |
| "signal is sampled every 1 ms" | Sample-and-hold | `findBlock('zero order hold')` |
| "velocity limited to 5 m/s² change rate" | Rate constraint | `findBlock('rate limiter')` |
| "0.5° of play in the gear mesh" | Mechanical backlash | `findBlock('backlash')` |
| "first-order sensor lag, τ = 50 ms" | Low-pass dynamics | `findBlock('transfer')` → use TF with `[1]/[0.05 1]` |
| "PWM switching at 20 kHz" | Pulse generator | `findBlock('pulse')` |
| "lookup for efficiency vs speed" | Interpolation | Route to `lookupTableBuilder` |

**Cost:** 2-3 MATLAB calls (findBlock + getBlockInfo).
**Coverage:** Anything the LLM can describe in engineering terms.

#### When all levels fail → odeBuilder

If none of the three levels finds a matching block, the equation is a **novel ODE** — route to `odeBuilder`. Common examples:
- Custom nonlinear coupling between multiple states
- Paper-specific empirical models with no standard form
- Coupled PDEs discretized into ODEs
- Novel hybrid formulations
- Equations with multiple interacting state variables and no standard block decomposition

### Function Signature

```matlab
[blkPath, meta, info] = blocksetBuilder(spec, mdl)
[blkPath, meta, info] = blocksetBuilder(spec, mdl, 'CheckLicense', true, 'Validate', true)
info = blocksetBuilder('info')
```

### Spec Struct Fields

| Field | Required | Description |
|-------|----------|-------------|
| `.name` | Yes | Block instance name |
| `.library` | Yes | Full Simulink library path (use `findBlock()` to discover) |
| `.parameters` | No | Struct of block dialog parameter name-value pairs (use `getBlockInfo()` to discover). **Values must be char/string** (e.g., `'10'`, `'[1 0.5 4]'`, `'eye(3)'`) since they are passed to `set_param()` which expects string representations. Do NOT pass numeric values. |
| `.paramMapping` | No | Struct array mapping source params to block params |
| `.inputNames` | No | Cell array of input port signal names |
| `.outputNames` | No | Cell array of output port signal names |
| `.subsystem` | No | Target subsystem path |
| `.source_equation` | No | Traceability: original equation from paper |
| `.recognized_model` | No | Traceability: classified model type |
| `.reason` | No | Traceability: why built-in block was chosen |
| `.alternative` | No | Traceability: what to use for exact reproduction |

### Name-Value Options

| Option | Default | Description |
|--------|---------|-------------|
| `'CheckLicense'` | `true` | Verify toolbox license before adding |
| `'Validate'` | `true` | Run `validateBlock` before adding |
| `'Position'` | auto | Block position `[l t r b]` |
| `'Connect'` | `struct()` | Auto-wiring: `.inputs` cell, `.output` string |

### Outputs

| Output | Description |
|--------|-------------|
| `blkPath` | Full Simulink path to created block |
| `meta` | Struct: `.block_path`, `.block_type`, `.library`, `.input_ports`, `.output_ports`, `.is_simscape`, `.physical_ports`, `.parameters`, `.param_mapping`, `.traceability` |
| `info` | Struct: `.elapsed_s`, `.warnings`, `.n_inputs`, `.n_outputs`, `.license_name`, `.params_set`, `.params_failed` |

### Workflow

1. **Discover** — `findBlock('tire')` to find library paths
2. **Inspect** — `getBlockInfo('vdynblks/Tires/...')` to get ports and parameters
3. **Validate** — `validateBlock(libPath, params)` to pre-check (done automatically)
4. **Build** — `blocksetBuilder(spec, mdl)` to add and configure

### License Detection

The builder auto-detects required toolbox from library path prefix:

| Prefix | Toolbox |
|--------|---------|
| `vdynblks/`, `autoblks/` | Vehicle Dynamics Blockset |
| `ee_lib/`, `powerlib/` | Simscape Electrical |
| `fl_lib/` | Simscape Fluids |
| `simscape/`, `nesl_utility/` | Simscape |
| `sm_lib/` | Simscape Multibody |
| `sdl_lib/` | Simscape Driveline |
| `aerolibatmos/`, `aerolibguidance/` | Aerospace Blockset |
| `ptblks/` | Powertrain Blockset |
| `simulink/` | Simulink (always available) |

### When to Use

- Equation represents a standard engineering component (tire, motor, battery, etc.)
- User wants production-quality model (not paper reproduction)
- Validated behavior and parameter tuning matter
- Block needs to integrate with other physical components
- Code generation or HIL testing planned

### When NOT to Use

- Exact textbook/paper equation reproduction required
- Custom or research-specific formulation
- Teaching model where all intermediate variables must be visible
- No suitable built-in block matches the equation
- Required toolbox is not installed

### Example

```matlab
spec.name = 'SpeedController';
spec.library = 'simulink/Continuous/PID Controller';
spec.parameters = struct('P', '10', 'I', '5', 'D', '0.1');
spec.source_equation = 'u = Kp*e + Ki*int(e) + Kd*de/dt';
spec.recognized_model = 'PID controller';
spec.reason = 'Standard PID; built-in has anti-windup + code generation';
spec.alternative = 'Equation-based: Gain+Integrator+Derivative blocks';
[blk, meta] = blocksetBuilder(spec, 'control_model');
```

### Wrapping Vector-Port Blockset Blocks with Bus Interfaces

Many blockset blocks have **vector ports** (e.g., Aerospace 6DOF takes Forces[3] and Moments[3] as vectors). When composing with odeBuilder components that produce scalar signals, a **bus wrapper** is needed at the subsystem boundary.

Use `wrapBlocksetBlock` to automate this wrapping:

```matlab
%% Example: Wrap 6DOF (Euler Angles) with typed bus interfaces
inBus.portIndex = 1;  % Not used when portMap is provided
inBus.busName = 'ForcesAndMoments';
inBus.elements = struct('name',{'Fx','Fy','Fz','Lroll','Mpitch','Nyaw'}, ...
                        'dimensions',{1,1,1,1,1,1});
inBus.portMap = {1, [1 2 3]; 2, [4 5 6]};  % Fx,Fy,Fz→port1; L,M,N→port2

outBus.portIndex = [5 6 3 2];  % Which block output ports to pack
outBus.busName = 'AircraftState';
outBus.elements = struct('name',{'u','v','w','p','q','r','phi','theta','psi','x_e','y_e','z_e'}, ...
                         'dimensions',{1,1,1,1,1,1,1,1,1,1,1,1});
outBus.portMap = {5,[1 2 3]; 6,[4 5 6]; 3,[7 8 9]; 2,[10 11 12]};

[mdl, ~, ~] = wrapBlocksetBlock('aerolib6dof/6DoF (Euler Angles)', ...
    'Name', 'Dynamics_6DOF', ...
    'Parameters', struct('Mass','65000','Inertia','eye(3)'), ...
    'InputBuses', inBus, 'OutputBuses', outBus);
```

**Result pattern inside the wrapper model:**
```
[Inport: ForcesAndMoments] → BusSelector → Mux → [6DOF block port 1 (Forces)]
                                         → Mux → [6DOF block port 2 (Moments)]
[6DOF port 5 (Vb)] → Demux ─┐
[6DOF port 6 (pqr)] → Demux ─┤→ BusCreator → [Outport: AircraftState]
[6DOF port 3 (Euler)] → Demux─┤
[6DOF port 2 (Xe)] → Demux ──┘
```

**When to use `wrapBlocksetBlock`:**
- Blockset block has vector ports (Forces[3], Moments[3], state[12], etc.)
- Composition model uses typed bus interfaces between subsystems (`busMode != 'none'`)
- Need scalar-compatible bus elements flowing to/from vector-port blocks

**When NOT needed (use `spec.vectorInputs`/`spec.vectorOutputs` instead):**
- Blockset block has vector ports BUT composition uses `busMode = 'none'` (scalar wiring)
- In this case, `executePlan` auto-expands via Mux/Demux — no bus infrastructure needed
- Blockset block already has scalar ports (PID controller, Transfer Fcn, etc.)

### Scalar Vector Expansion (Preferred for `busMode = 'none'`)

When `busMode = 'none'` and the blockset block has vector ports, use `spec.vectorInputs`/`spec.vectorOutputs` in the plan instead of `wrapBlocksetBlock`. `executePlan` handles the expansion automatically:

```matlab
% In plan.components(i).spec:
spec.library = 'aerolib6dof/6DoF (Euler Angles)';
spec.parameters = struct('Mass','18900','Inertia','diag([39800 367000 393600])');

% Expand vector inputs into named scalars
spec.vectorInputs(1).portIndex = 1;
spec.vectorInputs(1).elementNames = {'Fx','Fy','Fz'};
spec.vectorInputs(2).portIndex = 2;
spec.vectorInputs(2).elementNames = {'La','Ma','Na'};

% Expand vector outputs into named scalars
spec.vectorOutputs(1).portIndex = 5;
spec.vectorOutputs(1).elementNames = {'u','v','w'};
spec.vectorOutputs(2).portIndex = 6;
spec.vectorOutputs(2).elementNames = {'p','q','r'};
spec.vectorOutputs(3).portIndex = 3;
spec.vectorOutputs(3).elementNames = {'phi','theta','psi'};
spec.vectorOutputs(4).portIndex = 2;
spec.vectorOutputs(4).elementNames = {'xe','ye','h'};
```

**Result:** The subsystem model has scalar Inports (`Fx, Fy, Fz, La, Ma, Na`) and scalar Outports (`u, v, w, ...`). `composeModel` wires them by name like any other scalar-port component.

### Multi-Instance Blocks

When N copies of the same library block are needed (e.g., 5 actuator Transfer Functions):

```matlab
spec.library = 'simulink/Continuous/Transfer Fcn';
spec.parameters = struct('Numerator','[1/0.05]','Denominator','[1 1/0.05]');
spec.instances(1) = struct('name','tl','parameters',struct('Numerator','[1/1.5]','Denominator','[1 1/1.5]'));
spec.instances(2) = struct('name','tr','parameters',struct('Numerator','[1/1.5]','Denominator','[1 1/1.5]'));
spec.instances(3) = struct('name','de','parameters',struct());
spec.instances(4) = struct('name','da','parameters',struct());
spec.instances(5) = struct('name','dr','parameters',struct());
```

**Result:** One subsystem with 5 Inports (`tl_cmd, tr_cmd, ...`) and 5 Outports (`tl, tr, ...`). Avoids 5 separate components in the plan.

**Key fields in InputBuses/OutputBuses:**
| Field | Description |
|-------|-------------|
| `.portIndex` | Which block port (1-based). For portMap mode, not directly used. |
| `.busName` | Bus type name — becomes Simulink.Bus in workspace |
| `.elements` | Struct array: `.name`, `.dimensions` (default 1) |
| `.portMap` | Cell array `{blockPort, [elemIndices]; ...}` — maps ports to element groups |

**Integration with `composeModel`:**

After wrapping, the wrapped model's subsystem participates in bus composition via `internalizeBusInput` (input side) and `internalizeBusOutput` (output side). The composition pipeline:

1. `wrapBlocksetBlock` builds a standalone model with bus Inports/Outports
2. `composeModel` copies the wrapped subsystem into the top-level model
3. `internalizeBusOutput` / `internalizeBusInput` restructures ports at subsystem boundaries
4. Bus Selectors distribute signals to downstream consumers

---

### blocksetBuilder vs addProgrammaticBlocks

Both can add blocks, but they serve different roles in the pipeline:

| | `blocksetBuilder` | `addProgrammaticBlocks` |
|---|---|---|
| **Stage** | C (Build) — creates a system component | D (Compose) — augments existing model |
| **Lifecycle** | Participates in `executePlan` composition | Runs after composition is complete |
| **Interface** | Returns `buildInterface`-compatible struct for wiring | No interface — wires directly at top level |
| **Validation** | License check + `validateBlock` pre-flight | None — trusts the spec |
| **Traceability** | Full: source equation, recognized model, reason | None |
| **Fallback** | Returns recommendation to use odeBuilder if unavailable | Warns and continues |
| **Scope** | One validated component per call | Batch of blocks in one call |
| **Use case** | "This subsystem IS a transport delay" | "Add a Step input and wire it to port 3" |

**Rule of thumb:**
- If the block represents a **recognized component** from the paper's equations → `blocksetBuilder`
- If the block is **infrastructure** added after the model is built (test signals, logging, manual overrides) → `addProgrammaticBlocks`

---

## Discrete-Time Equation Handling

Three tiers handle discrete-time systems. The choice depends on equation linearity and complexity.

### Tier 1: Linear discrete → blocksetBuilder (conversion guidance)

When a paper presents linear difference equations, convert them to standard block parameters:

#### Difference equation → Discrete Transfer Function

Paper form: `y[k] + a1*y[k-1] + ... + an*y[k-n] = b0*u[k] + b1*u[k-1] + ... + bm*u[k-m]`

```matlab
% Convert to Discrete Transfer Fcn block
spec.name = 'DigitalFilter';
spec.library = 'simulink/Discrete/Discrete Transfer Fcn';
spec.parameters = struct( ...
    'Numerator', '[b0 b1 ... bm]', ...   % numerator coefficients
    'Denominator', '[1 a1 ... an]', ...   % denominator (leading 1)
    'SampleTime', 'Ts');                   % sample period in seconds
```

#### Discrete state equations → Discrete State-Space

Paper form: `x[k+1] = A*x[k] + B*u[k]`, `y[k] = C*x[k] + D*u[k]`

```matlab
spec.name = 'DiscreteController';
spec.library = 'simulink/Discrete/Discrete State-Space';
spec.parameters = struct( ...
    'A', mat2str(A), ...     % n×n state matrix
    'B', mat2str(B), ...     % n×m input matrix
    'C', mat2str(C), ...     % p×n output matrix
    'D', mat2str(D), ...     % p×m feedthrough matrix
    'X0', mat2str(x0), ...   % initial state vector
    'SampleTime', 'Ts');
```

#### Z-transform → coefficient vectors

| Paper notation | Conversion |
|---|---|
| `H(z) = (z+0.5)/(z^2 - 1.2z + 0.4)` | Num=`[1 0.5]`, Den=`[1 -1.2 0.4]`, in **descending** z powers |
| `H(z^{-1}) = (1+0.5z^{-1})/(1-1.2z^{-1}+0.4z^{-2})` | Same — already in z^{-1} form = descending z powers |
| Bilinear/Tustin from `G(s)` | `[num,den] = c2d(tf(numS,denS), Ts, 'tustin')` |
| ZOH discretization from `G(s)` | `[num,den] = c2d(tf(numS,denS), Ts, 'zoh')` |

#### Discrete PID (velocity/positional form)

Paper form: `u[k] = u[k-1] + Kp*(e[k]-e[k-1]) + Ki*Ts*e[k] + Kd/Ts*(e[k]-2e[k-1]+e[k-2])`

```matlab
spec.name = 'DiscretePID';
spec.library = 'simulink/Discrete/Discrete PID Controller';
spec.parameters = struct( ...
    'P', num2str(Kp), ...
    'I', num2str(Ki), ...
    'D', num2str(Kd), ...
    'N', '100', ...              % filter coefficient
    'SampleTime', 'Ts');
spec.source_equation = 'Velocity-form discrete PID';
spec.recognized_model = 'Discrete PID controller';
```

#### Recursive filter (IIR/FIR)

Paper form: `y[k] = sum(b_i * u[k-i]) - sum(a_j * y[k-j])`

```matlab
spec.name = 'RecursiveFilter';
spec.library = 'simulink/Discrete/Discrete Filter';
spec.parameters = struct( ...
    'Numerator', '[b0 b1 b2 ...]', ...
    'Denominator', '[1 a1 a2 ...]', ...
    'SampleTime', 'Ts');
```

### Tier 2: Nonlinear discrete → odeBuilder with `'Discrete'` (planned)

For nonlinear difference equations like `x[k+1] = f(x[k], u[k])` that don't map to any standard block, a future extension to odeBuilder will support:

```matlab
% PLANNED — not yet implemented
eqStr = '"\next{x1} = a*x1 + b*\sin(x2)" "\next{x2} = x1*x2 - c*u"';
[c, m, cellEq] = odeBuilder(eqStr, 'nonlinear_discrete', ...
    'Params', struct('a',0.9,'b',0.3,'c',1.2), ...
    'IC', struct('x1',1,'x2',0), ...
    'Discrete', 0.01);  % Ts = 10 ms
```

When `'Discrete'` is specified:
- Parser recognizes `\next{x}` notation (instead of `\dot{x}`)
- Builds **Unit Delay** blocks instead of Integrators
- Sets sample time on Unit Delay blocks
- All other wiring (algebraic, output equations, Constant blocks) remains identical

**Current workaround** (until implemented): Build with odeBuilder using `\dot{x}` notation, then manually replace Integrators with Unit Delay blocks and set sample time. Or use a MATLAB Function block (Tier 3).

### Tier 3: Algorithmic discrete — blocksetBuilder first, MATLAB Function as fallback

**Apply the three-level recognition protocol FIRST.** Many "algorithmic" components have standard Simulink blocks:

| Algorithm | Standard block | Toolbox | `findBlock` query |
|---|---|---|---|
| Linear Kalman Filter | Kalman Filter | Control System Toolbox | `findBlock('kalman')` |
| Extended Kalman Filter | Extended Kalman Filter | Control System Toolbox | `findBlock('extended kalman')` |
| Unscented Kalman Filter | Unscented Kalman Filter | Control System Toolbox | `findBlock('unscented kalman')` |
| Particle Filter | Particle Filter | Navigation Toolbox | `findBlock('particle filter')` |
| RLS adaptive filter | RLS Filter | DSP System Toolbox | `findBlock('RLS')` |
| LMS adaptive filter | LMS Filter | DSP System Toolbox | `findBlock('LMS')` |
| PID autotuner | PID Tuner | Control System Toolbox | `findBlock('PID tuner')` |
| Neural network (inference) | Predict | Deep Learning Toolbox | `findBlock('predict')` |

**Decision flow:**

```
Paper describes a discrete algorithm
    |
    v
Does a standard block exist? (findBlock)
    |-- yes, paper uses standard formulation → blocksetBuilder
    |-- yes, but paper customizes it (extra logic, non-standard interface) → MATLAB Function
    |-- no → MATLAB Function
    |-- yes, but toolbox not licensed → MATLAB Function (fallback)
```

**When blocksetBuilder (standard block):**
- Paper says "Kalman filter with matrices A, B, C, Q, R" → set matrices as block parameters
- Paper says "RLS with forgetting factor λ=0.99" → set as block parameter
- Paper uses the algorithm in its textbook form with no modifications

**When MATLAB Function (custom/non-standard):**

| Scenario | Why block won't work |
|---|---|
| Custom covariance reset logic | Block doesn't expose internal state for conditional modification |
| Variable-structure observer (switching models) | Conditional model selection not in standard block |
| Paper's EKF uses non-standard Jacobian computation | Block expects specific function signature |
| Trellis search / dynamic programming | No standard block exists |
| Paper adds saturation/anti-windup inside the estimator | Block's internal structure is fixed |

For MATLAB Function blocks, the surrounding plant is still built with odeBuilder; the MATLAB Function receives states via input ports and outputs control/estimated signals:

```matlab
% Phase 8: add MATLAB Function for custom estimator
pb(1).name = 'CustomEKF';
pb(1).type = 'MATLABFunction';
pb(1).value = ekfCode;  % string with function body
pb(1).inputs = struct('signal',{{'y_meas','u_cmd'}},'source',{{'block','block'}});
pb(1).outputs = struct('port',1,'target','PlantInput','target_port',1);
addProgrammaticBlocks(mdl, pb);
```

**Key rule:** Check `findBlock` before assuming MATLAB Function is needed. Don't decompose matrix operations into individual Simulink blocks — but DO use standard blocks when they exist.

---

## simscapeBuilder — Physical Network Builder

### Function Signature

```matlab
[mdl, meta, info] = simscapeBuilder(spec, modelName)
[mdl, meta, info] = simscapeBuilder(spec, modelName, 'Solver', 'ode23t', 'StopTime', '10')
info = simscapeBuilder('info')
```

### How Simscape Differs from Signal-Flow

Simscape uses **bidirectional energy ports** (across/through variables), not directed signals. Key differences:

- Physical connections are non-directional (conserving ports)
- Every network needs a **Solver Configuration** block and a **Reference** block (ground)
- Measurements require **Sensors** + **PS-Simulink Converters** to extract Simulink signals
- Inputs require **Simulink-PS Converters** to inject signals into the physical network
- Multiple elements on one node connect through a **Connection Label** (bus bar)

### Spec Struct Fields

| Field | Required | Description |
|-------|----------|-------------|
| `.domain` | Yes | `'electrical'`, `'rotational'`, `'translational'`, `'thermal'`, `'hydraulic'` |
| `.components` | Yes | Struct array: `.name`, `.library`, `.parameters`, `.nodes` |
| `.sources` | No | Struct array: `.name`, `.library`, `.parameters`, `.nodes`, `.input` |
| `.sensors` | No | Struct array: `.name`, `.library`, `.nodes`, `.output` |
| `.ground_node` | No | Node name for reference (default: `'gnd'`) |
| `.source_equations` | No | Cell array of original equations (traceability) |
| `.recognized_model` | No | Classified model type (traceability) |

### Port Mapping (`.ports` vs `.nodes`)

Each element can specify connectivity via either `.nodes` or `.ports`:

- **`.nodes`** (legacy 2-terminal): `{'+_node', '-_node'}`. Maps to `LConn1`=+, `RConn1`=-. Suitable only for simple 2-terminal elements.
- **`.ports`** (preferred for multi-port): Explicit port-to-node mapping struct. E.g., `struct('LConn1','node_a','RConn1','node_b','RConn2','node_c')`. When present, `.ports` takes precedence over `.nodes`.

For **sensors with multiple Physical Signal outputs**, use `.ps_ports` to specify which port IDs are PS outputs:
```matlab
spec.sensors(1).ps_ports = {'RConn1', 'RConn2'};  % e.g., motion sensor: speed + angle
spec.sensors(1).output = {'speed', 'angle'};       % one name per ps_port
```
Default: `{'RConn1'}` (single-output legacy format).

### Node Convention

Each element's `.nodes` is a 2-element cell: `{'+_node', '-_node'}`.
- Elements sharing a node name are physically connected (same voltage/force/temperature)
- The ground node gets the Reference block + Solver Configuration

### Critical Simscape Wiring Rules (R2025b)

1. **Use name-based `add_line`**: `add_line(mdl, 'R1/LConn1', 'node_n1/LConn1')` — NOT port handles
2. **Connection Labels** are the hub for multi-element nodes (unlimited connections to one label)
3. **Two-terminal elements**: `LConn1`=positive, `RConn1`=negative
4. **Sensors**: `LConn1`=+, `RConn1`=measurement output (PS), `RConn2`=- terminal
5. **Parameter names are lowercase** in Simscape blocks (`v0` not `V0`, `R` not `Resistance`)
6. **Signal logging** via `DataLogging` on PS-Simulink converter outputs — access via `simOut.logsout.get('name')` (consistent with odeBuilder)
7. Solver Configuration and Reference MUST share a physical node

### Supported Domains

| Domain | Reference block | Typical elements |
|--------|----------------|-----------------|
| `electrical` | Electrical Reference | Resistor, Capacitor, Inductor, Voltage/Current Source |
| `rotational` | Mechanical Rotational Reference | Inertia, Rotational Spring/Damper, Torque Source |
| `translational` | Translational World (PB) | Mass, Translational Spring/Damper, Force Source |
| `thermal` | Thermal Reference | Thermal Resistance, Heat Capacity, Heat Source |
| `hydraulic` | Hydraulic Reference | Pipe, Orifice, Pump |

### When to Use

- The source equations represent a standard physical network (circuit, mechanism, thermal system)
- Energy conservation and bidirectional coupling matter
- The model needs physical domain accuracy (no causal assumptions)
- User wants production-style Simscape model (not equation-based approximation)

### When NOT to Use

- Equations are custom ODEs that don't map cleanly to physical network elements
- Teaching model where signal-flow is clearer
- Single-domain, simple dynamics (odeBuilder is simpler)
- No Simscape license available (falls back gracefully)

### Example

```matlab
spec.domain = 'electrical';
spec.components(1) = struct('name','R1', ...
    'library','fl_lib/Electrical/Electrical Elements/Resistor', ...
    'parameters',struct('R','100'), 'nodes',{{'n1','n2'}});
spec.components(2) = struct('name','C1', ...
    'library','fl_lib/Electrical/Electrical Elements/Capacitor', ...
    'parameters',struct('C','1e-6'), 'nodes',{{'n2','gnd'}});
spec.sources(1) = struct('name','Vs', ...
    'library','fl_lib/Electrical/Electrical Sources/DC Voltage Source', ...
    'parameters',struct('v0','12'), 'nodes',{{'n1','gnd'}}, 'input','');
spec.sensors(1) = struct('name','Vc_sense', ...
    'library','fl_lib/Electrical/Electrical Sensors/Voltage Sensor', ...
    'nodes',{{'n2','gnd'}}, 'output','Vc');
spec.ground_node = 'gnd';
[mdl, meta] = simscapeBuilder(spec, 'rc_circuit');
```

---

## stateflowBuilder — Pure Discrete FSM Builder

Builds **pure Stateflow state machines** — no ODEs per state, no Simulink-based states. For supervisory control, protocol controllers, mode managers, and any logic-only FSM. Distinct from `odeBuilder_cps` which embeds continuous dynamics inside each state.

### Function Signature

```matlab
[mdl, meta, info] = stateflowBuilder(spec, modelName)
[mdl, meta, info] = stateflowBuilder(spec, modelName, 'SampleTime', '0.01', ...)
info = stateflowBuilder('info')
```

### Spec Struct Fields

| Field | Required | Description |
|-------|----------|-------------|
| `.states` | Yes | Struct array of states (see below) |
| `.transitions` | Yes | Struct array of transitions (see below) |
| `.inputs` | No | Struct array: `.name`, `.type` (default `'double'`), `.size` (default `'1'`). Also accepts simple cell array shorthand (see below). |
| `.outputs` | No | Struct array: `.name`, `.type`, `.size`. Also accepts simple cell array shorthand (see below). |
| `.initial` | No | String — name of the initial state (default: first state) |
| `.local_data` | No | Struct array: `.name`, `.type`, `.initial_value` |
| `.events` | No | Struct array: `.name`, `.scope` (`'Input'`/`'Output'`/`'Local'`) |
| `.description` | No | String — model description |

**Cell array shorthand for inputs/outputs:** Instead of the full struct array form, you can pass a simple cell array of signal names. The builder will treat each as a `double` scalar:
```matlab
spec.inputs = {'signal1', 'signal2'};          % shorthand
% equivalent to:
spec.inputs(1) = struct('name','signal1','type','double','size','1');
spec.inputs(2) = struct('name','signal2','type','double','size','1');
```

### State Struct

```matlab
spec.states(1).name = 'Idle';
spec.states(1).entry = 'y = 0;';       % (optional) entry action
spec.states(1).during = '';             % (optional) during action
spec.states(1).exit = '';               % (optional) exit action
spec.states(1).parent = '';             % (optional) parent state name for hierarchy
```

**Hierarchy:** Set `.parent` to the name of a containing state. The builder uses **position containment** — children are placed inside the parent's bounding box in chart coordinates. Do NOT expect constructor-based hierarchy (Stateflow ignores it).

### Transition Struct

```matlab
spec.transitions(1).from = 'Idle';       % source state ('' for default transition)
spec.transitions(1).to = 'Running';      % destination state
spec.transitions(1).guard = 'start == 1'; % (optional) condition
spec.transitions(1).action = 'count = 0;'; % (optional) transition action
```

### Name-Value Pairs

| Parameter | Default | Description |
|-----------|---------|-------------|
| `'SampleTime'` | `'0.01'` | Discrete sample time (string!) |
| `'Layout'` | `true` | Auto-arrange after build |

### Critical API Notes (R2025b)

1. **`SampleTime` must be a string**, not numeric: `set_param(chartBlk, 'SampleTime', '0.01')`
2. **`ChartUpdate = 'DISCRETE'`** — pure discrete chart, no inherited sample time
3. **Hierarchy = position containment** — a child state's bounding box must be INSIDE the parent's box in chart coordinates. `Stateflow.State(parent)` does NOT make it a child.
4. **`LabelString` format**: `'StateName\nentry: action;\nduring: action;\nexit: action;'`
5. **Default transition**: `Stateflow.Transition(chart)` with `.Source = []` and `.Destination = initialState`
6. **Typed I/O**: Use `Stateflow.Data` with `Scope='Input'`/`'Output'`, `DataType`, `Props.InitialValue`
7. **`containers.Map.Count`** returns `uint64` — cast to `double()` before arithmetic

### Auto-Generated Infrastructure

The builder automatically:
- Creates `Inport` blocks for each input and wires to the chart
- Creates `Outport` blocks for each output and wires from the chart
- Sets `ChartUpdate = 'DISCRETE'` and `SampleTime` on the chart block

### When to Use

- Pure control logic (no physics per state)
- Mode managers (select operating regime, no ODE switching)
- Protocol state machines (handshakes, communication sequences)
- Supervisory controllers (enable/disable subsystems based on conditions)

### When NOT to Use (use odeBuilder_cps instead)

- Each mode has its own continuous dynamics (ODEs differ per state)
- Transitions reset state variables (physical state resets)
- Hybrid systems where modes contain integrators

### Example

```matlab
% Traffic light FSM
spec.states(1) = struct('name','Red','entry','light=1;','during','','exit','','parent','');
spec.states(2) = struct('name','Green','entry','light=2;','during','','exit','','parent','');
spec.states(3) = struct('name','Yellow','entry','light=3;','during','','exit','','parent','');

spec.transitions(1) = struct('from','Red','to','Green','guard','after(30,sec)','action','');
spec.transitions(2) = struct('from','Green','to','Yellow','guard','after(25,sec)','action','');
spec.transitions(3) = struct('from','Yellow','to','Red','guard','after(5,sec)','action','');

spec.outputs(1) = struct('name','light','type','double','size','1');
spec.initial = 'Red';

[mdl, meta] = stateflowBuilder(spec, 'traffic_light');
```

---

## simeventsBuilder — Discrete-Event Simulation Builder

Builds **SimEvents discrete-event models** — entity-flow networks for queuing systems, manufacturing lines, resource allocation, and packet networks. Supports **hybrid DES-continuous** mode for systems where continuous dynamics interact with entity events (like tank filling, batch process control).

### Function Signature

```matlab
[mdl, meta, info] = simeventsBuilder(spec, modelName)
[mdl, meta, info] = simeventsBuilder(spec, modelName, 'StopTime', '200', 'EntityType', 'Structured')
info = simeventsBuilder('info')
```

### Spec Struct Fields — Core DES

| Field | Required | Description |
|-------|----------|-------------|
| `.blocks` | Yes | Struct array of DES blocks (see Block Types below) |
| `.connections` | Yes | Struct array: `.from`, `.from_port`, `.to`, `.to_port` |
| `.attributes` | No | Struct array: `.name`, `.initial` (entity attributes on generators) |
| `.signal_inputs` | No | Simulink→DES bridge: `.name`, `.target`, `.target_port`, `.use_message` |
| `.signal_outputs` | No | DES→Simulink bridge: `.name`, `.source`, `.source_port`, `.use_message` |
| `.resources` | No | Struct array: `.name`, `.amount` (resource pools) |
| `.description` | No | String |

### Spec Struct Fields — Hybrid DES-Continuous

These fields enable hybrid mode (solver switches from `VariableStepDiscrete` to `ode45`):

| Field | Required | Description |
|-------|----------|-------------|
| `.simulink_functions` | No | Callable Simulink Functions (entity actions invoke them) |
| `.continuous_dynamics` | No | Integrators, gains, sums running alongside DES |
| `.continuous_wiring` | No | Wiring between continuous blocks (supports `DataStoreRead:/Write:`) |
| `.hit_crossings` | No | Threshold detectors (continuous→event bridge) |
| `.data_stores` | No | Shared state between DES entity actions and continuous blocks |

**Hybrid field structures:**

- **`.simulink_functions`** — Struct array. Each element: `.name` (function name callable from entity actions), `.arguments` (cell of input arg names), `.returns` (cell of output arg names), `.body` (blocks inside the function subsystem), `.body_wiring` (internal wiring of function body blocks).
- **`.continuous_dynamics`** — Struct array. Each element: `.name` (block instance name), `.type` (block type, e.g., `'Integrator'`, `'Gain'`, `'Sum'`), `.parameters` (struct of block parameters).
- **`.continuous_wiring`** — Struct array. Each element: `.from`, `.from_port`, `.to`, `.to_port`. Supports `DataStoreRead:varName` / `DataStoreWrite:varName` as pseudo-block endpoints for DES-continuous coupling via shared data stores.

### Block Types

| Type string | SimEvents block | Entity ports |
|---|---|---|
| `'generator'` | Entity Generator | 1 out |
| `'queue'` | Entity Queue | 1 in, 1 out |
| `'server'` | Entity Server | 1 in, 1 out |
| `'terminator'` | Entity Terminator | 1 in |
| `'gate'` | Entity Gate | 1 in, 1 out, 1 control (message!) |
| `'output_switch'` | Entity Output Switch | 1 in, N out |
| `'input_switch'` | Entity Input Switch | N in, 1 out |
| `'delay'` | Entity Transport Delay | 1 in, 1 out |
| `'resource_pool'` | Resource Pool | (standalone, no entity ports) |
| `'resource_acquirer'` | Resource Acquirer | 1 in, 1 out |
| `'resource_releaser'` | Resource Releaser | 1 in, 1 out |
| `'batch_creator'` | Entity Batch Creator | 1 in, 1 out |
| `'batch_splitter'` | Entity Batch Splitter | 1 in, 1 out |
| `'replicator'` | Entity Replicator | 1 in, 1+ out |
| `'entity_store'` | Entity Store | 1 in |
| `'entity_find'` | Entity Find | 1 out |
| `'entity_selector'` | Entity Selector | 1 in, 1 out |
| `'message_send'` | Message Send | 1 signal in, 1 msg out |
| `'message_receive'` | Message Receive | 1 msg in, 1 signal out |
| `'discrete_event_chart'` | Discrete-Event Chart | varies |

### Block Struct

```matlab
spec.blocks(i).name = 'Buffer';       % instance name
spec.blocks(i).type = 'queue';         % from table above
spec.blocks(i).parameters = struct('Capacity','50', 'QueueType','FIFO');
spec.blocks(i).actions = struct('entry','', 'exit','', 'service_complete','');
```

### Simulink Function Struct (Hybrid Mode)

Simulink Functions are callable from entity actions. They bridge the DES and continuous domains via Data Stores.

```matlab
spec.simulink_functions(1).name = 'startFilling';
spec.simulink_functions(1).arguments = {'capacity'};   % input args
spec.simulink_functions(1).returns = {'started'};      % output returns
spec.simulink_functions(1).body(1) = struct( ...
    'name', 'WriteCapacity', ...
    'type', 'DataStoreWrite', ...
    'library', 'simulink/Signal Routing/Data Store Write', ...
    'parameters', struct('DataStoreName', 'TargetCapacity'));
spec.simulink_functions(1).body_wiring(1) = struct( ...
    'from', 'arg:capacity', 'from_port', 1, ...  % 'arg:name' references input argument
    'to', 'WriteCapacity', 'to_port', 1);
spec.simulink_functions(1).body_wiring(2) = struct( ...
    'from', 'SomeBlock', 'from_port', 1, ...
    'to', 'ret:started', 'to_port', 1);          % 'ret:name' references output return
```

**Key**: The default `u->y` line inside Simulink Functions is auto-deleted before custom wiring.

### Continuous Dynamics Struct (Hybrid Mode)

```matlab
spec.continuous_dynamics(1) = struct('name','FlowRate', 'type','constant', ...
    'parameters', struct('Value','2'));
spec.continuous_dynamics(2) = struct('name','FillIntegrator', 'type','integrator', ...
    'parameters', struct('InitialCondition','0'));
```

Supported types: `integrator`, `transfer_fcn`, `gain`, `sum`, `constant`, `product`, `math_function`, `scope`, `data_store_read`, `data_store_write`, `switch`, `saturation`, `abs`, `mux`, `demux`.

### Continuous Wiring Struct (Hybrid Mode)

```matlab
spec.continuous_wiring(1) = struct('from','FlowRate','from_port',1,'to','FillIntegrator','to_port',1);
spec.continuous_wiring(2) = struct('from','FillIntegrator','from_port',1, ...
    'to','DataStoreWrite:CurrentFill','to_port',1);  % auto-creates DSW block
spec.continuous_wiring(3) = struct('from','DataStoreRead:TargetCapacity','from_port',1, ...
    'to','DiffBlock','to_port',2);                   % auto-creates DSR block
```

**Special syntax:**
- `'DataStoreRead:StoreName'` — auto-creates a Data Store Read block for that store
- `'DataStoreWrite:StoreName'` — auto-creates a Data Store Write block for that store

### Hit Crossing Struct (Hybrid Mode)

```matlab
spec.hit_crossings(1).name = 'CapacityReached';
spec.hit_crossings(1).offset = '0';         % threshold value
spec.hit_crossings(1).direction = 'rising';  % 'rising', 'falling', 'either'
spec.hit_crossings(1).source = 'FillDifference'; % continuous block feeding detector
spec.hit_crossings(1).source_port = 1;
spec.hit_crossings(1).target = 'FillStation';    % (optional) DES block to notify via Message Send
spec.hit_crossings(1).target_port = 2;           % (optional) which port on target
```

When `.target` is specified, the builder auto-inserts a **Message Send** block between the Hit Crossing output and the DES target.

### Data Store Struct (Hybrid Mode)

```matlab
spec.data_stores(1) = struct('name','TargetCapacity', 'initial_value','100');
spec.data_stores(2) = struct('name','CurrentFill', 'initial_value','0');
```

### Critical API Notes (R2025b)

1. **Library is `sldelib`** (not `simeventslib`): `'sldelib/Entity Generator'`
2. **Multi-line block names** use `sprintf` with `\n`: `sprintf('sldelib/Entity\nOutput Switch')`
3. **Hit Crossing library path** needs space before newline: `sprintf('simulink/Discontinuities/Hit \nCrossing')`
4. **Entity Gate control port expects messages**, not signals — use Message Send to bridge
5. **Stats ports** (`NumberEntitiesInBlock='on'`) don't work with Structured entity type in R2025b
6. **`ver('slde')`** to check SimEvents version (not `ver('simevents')`)
7. **Solver selection**: pure DES → `VariableStepDiscrete`; hybrid → `ode45`
8. Simulink Functions are `SubSystem` blocks with a `TriggerPort` (not Function-Call Subsystems)

### Hybrid Architecture Pattern (Tank-Filling Style)

The hybrid DES-continuous pattern from the MathWorks tank-filling example:

```
┌─────────────────────────────────────────────────────┐
│  DES DOMAIN (entity flow)                           │
│  Generator → Queue → Server → Terminator            │
│       │                  │                          │
│       │   entity action calls                       │
│       │   startFilling(capacity)                    │
│       v                  v                          │
├─────────────────────────────────────────────────────┤
│  BRIDGE (Simulink Functions + Data Stores)          │
│  startFilling() writes TargetCapacity to DS         │
│  release() reads CurrentFill from DS                │
├─────────────────────────────────────────────────────┤
│  CONTINUOUS DOMAIN (signal flow)                    │
│  FlowRate → Integrator → [DSW: CurrentFill]        │
│             Integrator ─┬─ Sum(−) ← [DSR: Target]  │
│                         └── Hit Crossing → MsgSend  │
│                                           → Server  │
└─────────────────────────────────────────────────────┘
```

**Key design principles:**
- Entity actions communicate with continuous domain via **Simulink Functions** that read/write **Data Stores**
- Continuous dynamics (integrators) run independently at the top level
- **Hit Crossing** detects threshold events and bridges back to DES via **Message Send**
- The two domains share state through Data Store Memory blocks

### When to Use simeventsBuilder

- Queuing systems (M/M/1, M/M/c, priority queues)
- Manufacturing lines (workstations, conveyors, batching)
- Resource allocation (shared machines, operators, tools)
- Packet/communication networks
- **Hybrid:** Continuous processes triggered/controlled by discrete events (tank filling, batch reactors, traffic flow)

### When NOT to Use

- Continuous-only systems (use odeBuilder)
- Mode-switching with ODEs per mode (use odeBuilder_cps)
- Pure logic FSMs without entity flow (use stateflowBuilder)

### Example — Pure DES (M/M/1 Queue)

```matlab
spec.blocks(1) = struct('name','Arrivals','type','generator', ...
    'parameters',struct('Period','2'), 'actions',struct());
spec.blocks(2) = struct('name','Buffer','type','queue', ...
    'parameters',struct('Capacity','50'), 'actions',struct());
spec.blocks(3) = struct('name','Server','type','server', ...
    'parameters',struct('ServiceTimeValue','1.5'), 'actions',struct());
spec.blocks(4) = struct('name','Sink','type','terminator', ...
    'parameters',struct(), 'actions',struct());

spec.connections(1) = struct('from','Arrivals','from_port',1,'to','Buffer','to_port',1);
spec.connections(2) = struct('from','Buffer','from_port',1,'to','Server','to_port',1);
spec.connections(3) = struct('from','Server','from_port',1,'to','Sink','to_port',1);

[mdl, meta] = simeventsBuilder(spec, 'mm1_queue');
```

### Example — Hybrid DES-Continuous (Tank Filling)

```matlab
% DES section
spec.blocks(1) = struct('name','TankGen','type','generator', ...
    'parameters',struct('Period','8'), 'actions',struct());
spec.blocks(2) = struct('name','Queue','type','queue', ...
    'parameters',struct('Capacity','10'), 'actions',struct());
spec.blocks(3) = struct('name','FillStation','type','server', ...
    'parameters',struct('ServiceTimeValue','15'), 'actions',struct());
spec.blocks(4) = struct('name','Done','type','terminator', ...
    'parameters',struct(), 'actions',struct());

spec.connections(1) = struct('from','TankGen','from_port',1,'to','Queue','to_port',1);
spec.connections(2) = struct('from','Queue','from_port',1,'to','FillStation','to_port',1);
spec.connections(3) = struct('from','FillStation','from_port',1,'to','Done','to_port',1);

% Data stores (shared state)
spec.data_stores(1) = struct('name','TargetCapacity','initial_value','100');
spec.data_stores(2) = struct('name','CurrentFill','initial_value','0');

% Simulink Function (entity actions call this)
spec.simulink_functions(1).name = 'startFilling';
spec.simulink_functions(1).arguments = {'capacity'};
spec.simulink_functions(1).returns = {'ok'};
spec.simulink_functions(1).body(1) = struct('name','WriteTarget','type','DataStoreWrite', ...
    'library','simulink/Signal Routing/Data Store Write', ...
    'parameters',struct('DataStoreName','TargetCapacity'));
spec.simulink_functions(1).body_wiring(1) = struct( ...
    'from','arg:capacity','from_port',1,'to','WriteTarget','to_port',1);

% Continuous dynamics
spec.continuous_dynamics(1) = struct('name','FlowRate','type','constant', ...
    'parameters',struct('Value','2'));
spec.continuous_dynamics(2) = struct('name','Integrator','type','integrator', ...
    'parameters',struct('InitialCondition','0'));
spec.continuous_dynamics(3) = struct('name','Diff','type','sum', ...
    'parameters',struct('Inputs','+-'));

% Continuous wiring (DataStoreRead:/Write: syntax auto-creates blocks)
spec.continuous_wiring(1) = struct('from','FlowRate','from_port',1,'to','Integrator','to_port',1);
spec.continuous_wiring(2) = struct('from','Integrator','from_port',1,'to','DataStoreWrite:CurrentFill','to_port',1);
spec.continuous_wiring(3) = struct('from','Integrator','from_port',1,'to','Diff','to_port',1);
spec.continuous_wiring(4) = struct('from','DataStoreRead:TargetCapacity','from_port',1,'to','Diff','to_port',2);

% Hit Crossing (continuous -> event bridge)
spec.hit_crossings(1) = struct('name','Full','offset','0','direction','rising', ...
    'source','Diff','source_port',1,'target','','target_port',[]);

[mdl, meta] = simeventsBuilder(spec, 'tank_filling', 'StopTime', '100');
% Model uses ode45 solver automatically in hybrid mode
```

---

## existingBuilder — Copy Existing Subsystem

### Purpose

Copies an existing subsystem from a loaded model into the composed model **as a black box**. Use when:
- Extending an existing model with new components
- Combining two existing models
- Replacing one subsystem while keeping others intact
- Adding a controller to an existing plant

The `existing` builder does NOT introspect or modify the subsystem internals. It discovers the port interface (inports, outports, conserving ports) and exposes it for wiring.

### Spec Format

```matlab
spec.model       = 'myPlant';                 % REQUIRED: loaded model name (bdIsLoaded must be true)
spec.subsystem   = 'PlantDynamics';           % OPTIONAL: subsystem path within model
                                              %   If omitted, copies the entire model as one subsystem
spec.port_overrides = struct();               % OPTIONAL: rename ports for cleaner wiring
spec.port_overrides.inputs  = {'u1', 'Force'; 'u2', 'Torque'};  % {old, new; ...}
spec.port_overrides.outputs = {'y1', 'Position'; 'y2', 'Velocity'};
```

### Behavior

1. **Port discovery:** Reads all Inport/Outport blocks (and LConn/RConn for Simscape) from the source subsystem
2. **Copy:** Copies the subsystem block into the target model at the top level
3. **Interface:** Returns a standard `buildInterface` struct so `composeModel` can wire it like any other component
4. **Parameters:** Does NOT copy workspace variables or InitFcn — the LLM must ensure shared parameters are in the composed model's InitFcn

### Limitations

- Cannot modify internals — the subsystem is copied verbatim
- Source model must be loaded (`bdIsLoaded` must return true)
- If the subsystem references base workspace variables, those must exist when the composed model runs
- Conserving (Simscape) ports require domain-compatible connections in wiring

### Example

```matlab
% Extend an existing vehicle model with a new controller
plan.components(1).name = 'Vehicle Plant';
plan.components(1).builder = 'existing';
plan.components(1).spec = struct('model', 'vehiclePlant', 'subsystem', 'Chassis');
plan.components(1).interface = struct( ...
    'externalInputs', {{'Fx', 'Fy', 'Mz'}}, ...
    'externalOutputs', {{'vx', 'vy', 'yaw_rate', 'X', 'Y'}});
plan.components(1).decision_path = struct( ...
    'tree_terminal_node', 'existing (pre-built subsystem)', ...
    'is_existing', true, ...
    'source_model', 'vehiclePlant', ...
    'source_subsystem', 'Chassis');

plan.components(2).name = 'Path Controller';
plan.components(2).builder = 'odeBuilder';
plan.components(2).spec = struct(...);  % New controller equations
% ... normal odeBuilder spec ...

plan.wiring(1) = struct('from_component','Path Controller','from_port','Fx', ...
    'to_component','Vehicle Plant','to_port','Fx');
```
