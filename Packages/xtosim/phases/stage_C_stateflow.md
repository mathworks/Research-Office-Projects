# Stateflow Pipeline: Pure Discrete Logic / FSM Path

**When to use:** System is best modeled as a finite state machine — control logic, mode managers, protocol FSMs, sequencers, supervisory controllers, fault detection. **No ODEs per state.** If each state has its own continuous dynamics, use `odeBuilder_cps` instead.

**Key difference from ODE path:** No equations, no normalization. You define states, transitions, guards, and actions directly.

**Builder:** `stateflowBuilder`

---

## Pipeline Comparison

| ODE Path | Stateflow Path | What changes |
|----------|----------------|-------------|
| Phase 1-3: Extract equations | **SF1**: Identify states/transitions | State diagram, not LaTeX |
| Phase 4-5: Normalize + translate | (not needed) | No algebra |
| Phase 6-7: Build with odeBuilder | **SF2**: Build with stateflowBuilder | Spec struct, not equation string |
| Phase 7b: Subsystem validation | **SF3**: Verify transitions | Simulate and check mode sequences |
| Phase 8-9: Programmatic blocks | **SF4**: Wire into system | If part of a larger model |
| Phase 10: Full validation | **SF5**: Full validation | Same test framework |

---

## SF1: Identify States and Transitions

**Input:** Source (state diagram, textual description, protocol spec, truth table)
**Output:** Stateflow spec struct

### SF1a: Extract state information

For each state, identify:
- **Name** — valid MATLAB identifier
- **Entry actions** — assignments that execute on entry (e.g., `gear=1; led=1;`)
- **During actions** — execute every time step while in this state
- **Exit actions** — cleanup on leaving
- **Parent** — for hierarchical (nested) states

### SF1b: Extract transitions

For each transition, identify:
- **Source state** — empty string for the default (initial) transition
- **Destination state**
- **Guard condition** — boolean expression (e.g., `speed > 5 && brake == 0`)
- **Transition action** — executes during the transition
- **Event trigger** — (optional) named event that fires the transition
- **Order** — evaluation priority when multiple transitions leave the same state

### SF1c: Identify I/O

- **Inputs** — signals read by guards/actions (e.g., `speed`, `button`, `temperature`)
- **Outputs** — signals set by actions (e.g., `gear`, `mode`, `alarm`)
- **Local data** — variables used internally but not exposed as ports

---

## SF2: Build with stateflowBuilder

### Spec struct format

```matlab
spec.states(1) = struct('name','Idle', 'entry','output=0;', 'during','', 'exit','', 'parent','');
spec.states(2) = struct('name','Active', 'entry','output=1;', 'during','', 'exit','', 'parent','');

spec.transitions(1) = struct('from','', 'to','Idle', 'guard','', 'action','', 'event','', 'order',1);
spec.transitions(2) = struct('from','Idle', 'to','Active', 'guard','trigger>0.5', 'action','', 'event','', 'order',1);
spec.transitions(3) = struct('from','Active', 'to','Idle', 'guard','trigger<0.1', 'action','', 'event','', 'order',1);

spec.inputs = {'trigger'};
spec.outputs = struct('name','output', 'type','double', 'size','1', 'initial','0');
spec.initial = 'Idle';

[mdl, meta, info] = stateflowBuilder(spec, 'my_fsm');
```

### Key options

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `'SampleTime'` | `'-1'` (inherited) | Chart execution rate. Use `'0.01'` for 100Hz discrete. |
| `'ChartUpdate'` | `'DISCRETE'` | `'DISCRETE'` or `'INHERITED'` |
| `'ActionLanguage'` | `'MATLAB'` | `'MATLAB'` or `'C'` |
| `'Layout'` | `true` | Auto-position states in chart |

### Hierarchical states

Use the `.parent` field to nest states:

```matlab
spec.states(1) = struct('name','Off', 'entry','', 'during','', 'exit','', 'parent','');
spec.states(2) = struct('name','On', 'entry','', 'during','', 'exit','', 'parent','');
spec.states(3) = struct('name','Low', 'entry','power=1;', 'during','', 'exit','', 'parent','On');
spec.states(4) = struct('name','High', 'entry','power=3;', 'during','', 'exit','', 'parent','On');
```

---

## SF3: Verify Transitions

After building, simulate and verify:
1. Model starts in the initial state
2. Guard conditions trigger correct transitions
3. Entry/exit actions produce expected outputs
4. No deadlocks (unreachable states with no exit transitions)

```matlab
simOut = sim(mdl, 'StopTime', '10');
% Check output transitions match expected sequence
```

---

## SF4: Wire into System (mixed-builder context)

When the Stateflow chart is one component in a larger model (via `executePlan`):
- `meta.interface` provides the port contract for `composeModel`
- Inputs wire from other subsystem outputs (sensor signals, flags)
- Outputs wire to actuators, mode selectors, or enable ports

---

## SF5: Validation

Use the standard `evaluateTests` framework:
- Test each transition individually (stimulus → expected output sequence)
- Test temporal logic (after N ticks in a state)
- Test hierarchical entry/exit ordering

---

## When NOT to use stateflowBuilder

| Situation | Use instead |
|-----------|------------|
| Each state has its own ODE (e.g., bouncing ball, gear shifting with inertia) | `odeBuilder_cps` |
| Simple switching logic (6 cases, no memory) | `plan.programmatic` as MATLABFunction |
| Lookup table (output = f(discrete input)) | `lookupTableBuilder` |
| Continuous control (PID, filters) | `blocksetBuilder` |
