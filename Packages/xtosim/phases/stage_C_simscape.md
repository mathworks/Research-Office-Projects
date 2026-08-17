# Simscape Pipeline: Physical Network Modeling Path

**When to use:** System is best modeled as a physical network (energy-conserving connections, multi-domain coupling, standard engineering components). The user explicitly requests Simscape, or the system is inherently a netlist topology (circuit, mechanism, thermal network, hydraulic system, electromechanical transducer).

**Key difference from ODE path:** No equation derivation, normalization, or algebraic sorting needed. Simscape solves the governing equations internally from the network topology and component constitutive laws.

**Execution order:** S1 -> S2 -> S3 -> S4 -> S5 -> S6

---

## Pipeline Comparison

| ODE Path | Simscape Path | What changes |
|----------|---------------|-------------|
| Phase 1-3: Extract equations | **S1**: Identify topology | No LaTeX equations needed |
| Phase 4-5: Normalize + translate | **S2**: Map to Simscape library | Block discovery, not algebra |
| Phase 6-7: Build with odeBuilder | **S3**: Build with simscapeBuilder | Netlist spec, not equation string |
| Phase 7b: Subsystem validation | **S4**: Simulate + validate | Same physics checks |
| Phase 8-9: Programmatic blocks | (not needed — topology IS the model) | No Gain/Sum/Fcn blocks |
| Phase 10: Full validation | **S5**: Full validation | Same test framework |
| Phase 12: Package | **S6**: Package | Same deliverable structure |

---

## S1: Identify System Topology

**Input:** Source (document, description, schematic, or derivation request)
**Output:** Topology description — components, domains, nodes, connectivity

### S1a: Identify physical domains

List all physical domains present in the system:
- Electrical (voltage/current)
- Mechanical translational (force/velocity)
- Mechanical rotational (torque/angular velocity)
- Thermal (temperature/heat flow)
- Hydraulic (pressure/flow)

Multi-domain systems have **transducers** coupling domains (motors, generators, actuators, piezoelectrics, hydraulic cylinders).

### S1b: Identify components

For each domain, list the physical elements:
- **Energy storage:** capacitors, inductors, springs, masses, inertias, thermal capacitances, accumulators
- **Energy dissipation:** resistors, dampers, friction, thermal resistances, orifices
- **Sources:** voltage/current sources, force/torque sources, flow/pressure sources, heat sources
- **Transducers:** electromechanical converters, transformers, gyrators, hydraulic actuators, thermoelectric devices
- **Sensors:** what needs to be measured and in which domain

### S1c: Identify connectivity (nodes)

A **node** is a point where components share the same across variable (voltage, velocity, temperature, pressure). Through variables (current, force, heat flow, flow rate) sum to zero at each node.

Name each node descriptively. For multi-domain systems, use domain-prefixed names to prevent confusion:
```
<domain>_<location>
```

Examples across domains:
```
elec_plus, elec_minus, elec_mid      (electrical)
mech_body, mech_ground, mech_joint   (mechanical)
therm_hot, therm_cold, therm_ambient (thermal)
hyd_high, hyd_low, hyd_tank          (hydraulic)
```

### S1d: Identify parameters

Extract numerical values for each component. Sources of truth (priority order):
1. Code listings in source document
2. Parameter tables
3. Prose/text values
4. Datasheets / typical values (document source as "typical" or "derived")

---

## S2: Map to Simscape Library Blocks

**Input:** Topology from S1
**Output:** Spec struct ready for simscapeBuilder

### S2a: Find library paths

Common Simscape blocks by domain:

**Electrical:**
| Component | Library Path |
|-----------|-------------|
| Resistor | `fl_lib/Electrical/Electrical Elements/Resistor` |
| Capacitor | `fl_lib/Electrical/Electrical Elements/Capacitor` |
| Inductor | `fl_lib/Electrical/Electrical Elements/Inductor` |
| DC Voltage Source | `fl_lib/Electrical/Electrical Sources/DC Voltage Source` |
| AC Voltage Source | `fl_lib/Electrical/Electrical Sources/AC Voltage Source` |
| Controlled Voltage Source | `fl_lib/Electrical/Electrical Sources/Controlled Voltage Source` |
| Current Sensor | `fl_lib/Electrical/Electrical Sensors/Current Sensor` |
| Voltage Sensor | `fl_lib/Electrical/Electrical Sensors/Voltage Sensor` |
| Electrical Reference | `fl_lib/Electrical/Electrical Elements/Electrical Reference` |
| Ideal Transformer | `fl_lib/Electrical/Electrical Elements/Ideal Transformer` |
| Translational EMC | `fl_lib/Electrical/Electrical Elements/Translational Electromechanical Converter` |
| Rotational EMC | `fl_lib/Electrical/Electrical Elements/Rotational Electromechanical Converter` |

**Mechanical Translational:**
| Component | Library Path |
|-----------|-------------|
| Mass | `fl_lib/Mechanical/Translational Elements/Mass` |
| Translational Spring | `fl_lib/Mechanical/Translational Elements/Translational Spring` |
| Translational Damper | `fl_lib/Mechanical/Translational Elements/Translational Damper` |
| Translational Friction | `fl_lib/Mechanical/Translational Elements/Translational Friction` |
| Translational Hard Stop | `fl_lib/Mechanical/Translational Elements/Translational Hard Stop` |
| Mech Trans Reference | `fl_lib/Mechanical/Translational Elements/Mechanical Translational Reference` |
| Motion Sensor (Trans) | `fl_lib/Mechanical/Mechanical Sensors/Ideal Translational Motion Sensor` |
| Force Sensor | `fl_lib/Mechanical/Mechanical Sensors/Ideal Force Sensor` |

**Mechanical Rotational:**
| Component | Library Path |
|-----------|-------------|
| Inertia | `fl_lib/Mechanical/Rotational Elements/Inertia` |
| Torsional Spring | `fl_lib/Mechanical/Rotational Elements/Torsional Spring` |
| Rotational Damper | `fl_lib/Mechanical/Rotational Elements/Rotational Damper` |
| Rotational Friction | `fl_lib/Mechanical/Rotational Elements/Rotational Friction` |
| Mech Rot Reference | `fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference` |
| Motion Sensor (Rot) | `fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor` |
| Torque Sensor | `fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor` |

**Thermal:**
| Component | Library Path |
|-----------|-------------|
| Thermal Mass | `fl_lib/Thermal/Thermal Elements/Thermal Mass` |
| Conductive Heat Transfer | `fl_lib/Thermal/Thermal Elements/Conductive Heat Transfer` |
| Convective Heat Transfer | `fl_lib/Thermal/Thermal Elements/Convective Heat Transfer` |
| Thermal Reference | `fl_lib/Thermal/Thermal Elements/Thermal Reference` |

For blocks not in these tables, use:
```matlab
results = findBlock('keyword');
```

### S2b: Determine port mapping for multi-port blocks

**Standard 2-terminal elements** are simple: LConn1 = + terminal, RConn1 = - terminal. No further investigation needed.

**Multi-port and multi-domain blocks** require explicit port identification. Known mappings:

| Block | LConn1 | LConn2 | RConn1 | RConn2 | RConn3 |
|-------|--------|--------|--------|--------|--------|
| Translational EMC | Elec+ | Mech Case(C) | Elec- | Mech Rod(R) | — |
| Rotational EMC | Elec+ | Mech Case(C) | Elec- | Mech Rotor(R) | — |
| Ideal Transformer | Primary+ | Secondary+ | Primary- | Secondary- | — |
| Current Sensor | Elec+(series) | — | PS(I output) | Elec-(series) | — |
| Voltage Sensor | Elec+(parallel) | — | PS(V output) | Elec-(parallel) | — |
| Motion Sensor (Trans) | Mech(follower/R) | — | Mech(base/C) | PS(velocity) | PS(position) |
| Motion Sensor (Rot) | Mech(follower/R) | — | Mech(base/C) | PS(ang vel) | PS(angle) |
| Force Sensor | Mech(through+) | — | PS(F output) | Mech(through-) | — |
| Controlled Voltage Src | Elec+ | — | **PS(V input)** | Elec- | — |
| Controlled Current Src | Elec+ | — | **PS(I input)** | Elec- | — |
| Ideal Force Source | Mech(+) | — | **PS(F input)** | Mech(-) | — |

**CRITICAL (R2025b):** Controlled sources use **RConn1** as the Physical Signal input, NOT LConn1. This differs from some documentation. Always probe if unsure.

**If port mapping is unknown for a block**, use the domain-probe technique:
```matlab
% Create a test model with the block + one reference per domain
% Try connecting each port to each reference type:
add_line(mdl, 'Block/RConn1', 'ElecRef/LConn1');  % SUCCESS → electrical port
add_line(mdl, 'Block/RConn1', 'MechRef/LConn1');  % SUCCESS → mechanical port
add_line(mdl, 'Block/RConn1', 'PS2SL/LConn1');    % SUCCESS → Physical Signal port
% Simscape enforces domain — wrong domain throws an error immediately.
```

### S2c: Build the spec struct

Choose format based on block complexity:

**Two-terminal, single-domain** → use `.nodes` (legacy format):
```matlab
comp.nodes = {'nodeA', 'nodeB'};  % LConn1→nodeA, RConn1→nodeB
```

**Multi-port or multi-domain** → use `.ports` (explicit mapping):
```matlab
comp.ports = struct('LConn1','node1', 'LConn2','node2', ...
                    'RConn1','node3', 'RConn2','node4');
```

**Single-port** (e.g., Mass, Inertia) → use `.ports`:
```matlab
comp.ports = struct('LConn1','node_body');  % implicit reference to inertial frame
```

**Rule:** If a block has more than 2 physical ports, or ports in different domains, always use `.ports`. Never guess which port maps to LConn1/RConn1 — use the table in S2b or the domain-probe technique.

### S2d: Define references

One reference block per physical domain present in the model:
```matlab
spec.references(1) = struct('domain','electrical', 'node','elec_gnd', 'library','');
spec.references(2) = struct('domain','translational', 'node','mech_gnd', 'library','');
% 'library' = '' means auto-infer from domain name (getDomainReference helper)
% Or specify explicitly:
spec.references(3) = struct('domain','thermal', 'node','therm_amb', ...
    'library','fl_lib/Thermal/Thermal Elements/Thermal Reference');
```

Every domain in the model MUST have exactly one reference. The Solver Configuration block connects to the first reference's node.

### S2e: Define sensors

Sensors have two kinds of ports:
1. **Conserving ports** — connect to the physical network (same as components)
2. **Physical Signal (PS) ports** — measurement outputs (go through PS-Simulink converter)

```matlab
% Multi-output sensor (e.g., Motion Sensor with velocity + position):
sen.name = 'MotionSensor1';
sen.library = 'fl_lib/Mechanical/Mechanical Sensors/Ideal Translational Motion Sensor';
sen.ports = struct('LConn1','mech_body', 'RConn1','mech_gnd');  % conserving ports
sen.ps_ports = {'RConn2', 'RConn3'};    % Physical Signal output ports
sen.output = {'body_vel', 'body_pos'};  % one name per PS port
```

For legacy single-output sensors (e.g., Voltage Sensor, Current Sensor):
```matlab
sen.name = 'Vsense1';
sen.library = 'fl_lib/Electrical/Electrical Sensors/Voltage Sensor';
sen.nodes = {'node_a', 'node_b'};  % LConn1=+, RConn2=-
sen.output = 'V_ab';               % single string, PS on RConn1
```

---

## S3: Build with simscapeBuilder

**Input:** Spec struct from S2
**Output:** Working Simscape model, open for inspection

```matlab
run(fullfile(SKILL_DIR, 'setup.m'));

[mdl, meta, info] = simscapeBuilder(spec, modelName, ...
    'Solver', 'ode23t', ...   % DAE-appropriate solver
    'StopTime', '10', ...
    'Logging', true, ...      % auto-add ToWorkspace blocks
    'Layout', true);          % auto-arrange

% Check for warnings
if ~isempty(info.warnings)
    fprintf('BUILD WARNINGS:\n');
    cellfun(@(w) fprintf('  %s\n', w), info.warnings);
end
```

### S3 post: Verify model compiles

```matlab
set_param(mdl, 'StopTime', '0.001');
try
    simOut = sim(mdl);
    fprintf('Model compiles and simulates.\n');
catch ME
    fprintf('BUILD FAILED: %s\n', ME.message);
    % Common issues and fixes:
    %   "Initial conditions solve failed"
    %     → missing reference block or disconnected node
    %   "Domain mismatch" / "connection rules"
    %     → wrong port assignment in .ports struct; re-check with domain-probe
    %   "Unconnected conserving port"
    %     → a port in .ports maps to a node that no reference touches
    %   Solver fails immediately
    %     → Solver Configuration not connected to the physical network
end
```

---

## S4: Simulate and Validate

**Input:** Working model from S3
**Output:** Validated model with logged signals

### S4a: Steady-state validation

Apply known DC/constant inputs and verify convergence to expected equilibrium:
- Through-variable balance (KCL for current, force balance for mechanics)
- Across-variable distribution (voltage divider, velocity matching)
- Power balance (total source power = total dissipated power at steady state)

```matlab
set_param(mdl, 'StopTime', stopTime);  % long enough to reach steady state
simOut = sim(mdl);
% Logged data in workspace as <output_name>_log (timeseries)
```

### S4b: Transient validation

Verify dynamic behavior against known analytical results:
- **Natural frequencies:** eigenvalues of the linearized system
- **Damping ratios:** rate of decay
- **Time constants:** L/R, RC, m/c, etc.
- **Peak response magnitude:** first overshoot

### S4c: Cross-domain validation (multi-domain models only)

For transducer-coupled systems, verify:
- Power conservation across domain boundary (lossless transducers)
- Loading effects: impedance reflection between domains
- Back-effect: secondary domain modifies primary domain behavior (e.g., back-EMF reduces current, counter-torque reduces speed)

### S4d: Eigenvalue check (optional)

For linear or linearizable systems:
```matlab
[A,B,C,D] = linmod(mdl);
eigs_sim = eig(A);
% Compare to hand-calculated or paper-reported eigenvalues
```

---

## S5: Full Validation (same framework as Phase 10)

Run the standard xToSim validation framework:
- Minimum 3 tests (at least 1 open-loop / no controller)
- Use `evaluateTests` for PASS/CLOSE/FAIL scoring
- Generate comparison plots if reference data available
- `autoPlotValidation` for signal coverage

```matlab
results = evaluateTests(tests, 'Setpoints', setpoints);
autoPlotValidation(mdl, simOut, spec, figDir);
```

---

## S6: Package (same as Phase 12)

Same deliverable structure as the ODE path:
- Model .slx file
- params.m (all component values, editable)
- run_validation.m (standalone reproducible script)
- report.html (via fillReport)
- figures/ directory (plots + model screenshot + schematic)

```matlab
htmlFile = fillReport(outputDir, rpt);
[pass, issues] = validateReport(rpt);
saveModelScreenshots(mdl, figDir);
```

---

## Gate Checklist: Simscape Build Complete

```
- [ ] All components placed and connected (0 warnings from simscapeBuilder)
- [ ] Model simulates without error
- [ ] Steady-state values match analytical/expected (document actual vs expected)
- [ ] Transient dynamics plausible (natural frequency, damping, time constants)
- [ ] Multi-domain coupling verified (if applicable)
- [ ] All sensors logging correctly
- [ ] At least 3 validation tests (1 open-loop minimum)
- [ ] Schematic/cartoon figure included
- [ ] report.html generated and verified
- [ ] run_validation.m reproduces all tests standalone
```

---

## When NOT to Use Simscape Path

- **Paper reproduction mode** where the paper presents explicit ODEs and exact equation fidelity is the goal
- **Custom nonlinear constitutive laws** not available as Simscape library blocks
- **Tight coupling with controllers** where plant equations are deeply interleaved with control logic
- **No Simscape license** (simscapeBuilder auto-detects and returns fallback recommendation)
- **Lookup-table-based models** (empirical maps, measured data) → lookupTableBuilder

## When TO Use Simscape Path

- **Multi-domain energy systems** where energy flows bidirectionally between domains
- **Circuit simulation** (power electronics, filters, amplifiers, motor drives)
- **Thermal networks** (heat sinks, enclosures, multi-node thermal)
- **Mechanism simulation** (gearboxes, linkages, drivetrain)
- **Hydraulic systems** (actuators, valves, piping networks)
- **Production-quality models** where standard validated components are preferred over custom math
- **User explicitly requests** physical network / Simscape / energy-based approach
