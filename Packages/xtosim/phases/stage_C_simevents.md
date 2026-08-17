# SimEvents Pipeline: Discrete-Event Simulation Path

**When to use:** System involves entity flows — queues, servers, resource allocation, scheduling, manufacturing lines, network packet routing, logistics. Entities are discrete "things" that move through a process. Optionally includes continuous dynamics (hybrid DES-continuous).

**Key difference from ODE path:** No differential equations drive the primary dynamics. Instead, entities flow through a network of blocks. Time advances event-by-event (not step-by-step).

**Builder:** `simeventsBuilder`

---

## Pipeline Comparison

| ODE Path | SimEvents Path | What changes |
|----------|----------------|-------------|
| Phase 1-3: Extract equations | **SE1**: Identify entity flow | Process diagram, not LaTeX |
| Phase 4-5: Normalize + translate | (not needed) | No algebra |
| Phase 6-7: Build with odeBuilder | **SE2**: Build with simeventsBuilder | Spec struct with blocks/connections |
| Phase 7b: Subsystem validation | **SE3**: Verify entity flow | Simulate and check throughput/queues |
| Phase 8-9: Programmatic blocks | **SE4**: Wire signal I/O | Bridge DES ↔ Simulink signals |
| Phase 10: Full validation | **SE5**: Full validation | Statistics-based metrics |

---

## SE1: Identify Entity Flow

**Input:** Source (process description, queueing model, manufacturing spec)
**Output:** SimEvents spec struct

### SE1a: Identify blocks

Map the process to SimEvents block types:

| Process element | Block type | Purpose |
|-----------------|-----------|---------|
| Arrival of items | `'generator'` | Creates entities at specified rate |
| Waiting area | `'queue'` | FIFO/LIFO/priority buffer |
| Processing step | `'server'` | Holds entity for service time |
| End of process | `'terminator'` | Destroys completed entities |
| Routing decision | `'output_switch'` | Routes entities to one of N paths |
| Merge paths | `'input_switch'` | Combines N paths into one |
| Resource needed | `'resource_acquirer'` | Blocks until resource available |
| Resource returned | `'resource_releaser'` | Returns resource to pool |
| Shared resource | `'resource_pool'` | Finite pool of reusable resources |
| Delay (no server) | `'delay'` | Time-based entity delay |
| Batch items | `'batch_creator'` | Groups N entities into one |
| Unbatch | `'batch_splitter'` | Splits batch into individual entities |
| Storage | `'entity_store'` | Stores entities for later retrieval |

### SE1b: Define connections (entity flow)

Connections define how entities flow between blocks:
```matlab
spec.connections(1) = struct('from','Generator', 'from_port',1, 'to','Queue', 'to_port',1);
spec.connections(2) = struct('from','Queue', 'from_port',1, 'to','Server', 'to_port',1);
spec.connections(3) = struct('from','Server', 'from_port',1, 'to','Terminator', 'to_port',1);
```

### SE1c: Define entity attributes

Attributes travel with each entity (like struct fields):
```matlab
spec.attributes(1) = struct('name','priority', 'initial',1);
spec.attributes(2) = struct('name','arrival_time', 'initial',0);
```

### SE1d: Define signal I/O (optional DES ↔ Simulink bridging)

Signal inputs bring Simulink signals into DES actions:
```matlab
spec.signal_inputs(1) = struct('name','rate_cmd', 'target','Generator', 'target_port',1, 'use_message',false);
```

Signal outputs extract DES statistics as Simulink signals:
```matlab
spec.signal_outputs(1) = struct('name','queue_length', 'source','Queue', 'source_port',2, ...
    'use_message',false, 'stat_param','NumberEntitiesInBlock');
```

**IMPORTANT:** Statistics ports are disabled by default on SimEvents blocks. The builder's `enableStatPort` function handles enabling them — specify `stat_param` for direct control, or let the builder probe for available toggles.

---

## SE2: Build with simeventsBuilder

### Full spec example

```matlab
spec.blocks(1) = struct('name','Arrivals', 'type','generator', ...
    'parameters', struct('GenerationMethod','Intergeneration time', ...
                         'Period','exprnd(2)'), ...
    'actions', struct('generate','entity.arrival_time = getCurrentTime;'));

spec.blocks(2) = struct('name','WaitQueue', 'type','queue', ...
    'parameters', struct('Capacity','20', 'SortingDirection','FIFO'), ...
    'actions', struct());

spec.blocks(3) = struct('name','Processor', 'type','server', ...
    'parameters', struct('ServiceTime','2+rand*0.5'), ...
    'actions', struct('service_complete','entity.wait_time = getCurrentTime - entity.arrival_time;'));

spec.blocks(4) = struct('name','Exit', 'type','terminator', ...
    'parameters', struct(), 'actions', struct());

spec.connections = [ ...
    struct('from','Arrivals', 'from_port',1, 'to','WaitQueue', 'to_port',1), ...
    struct('from','WaitQueue', 'from_port',1, 'to','Processor', 'to_port',1), ...
    struct('from','Processor', 'from_port',1, 'to','Exit', 'to_port',1)];

spec.attributes = struct('name',{'arrival_time','wait_time'}, 'initial',{0,0});

spec.signal_outputs(1) = struct('name','queue_len', 'source','WaitQueue', ...
    'source_port',2, 'use_message',false, 'stat_param','NumberEntitiesInBlock');

[mdl, meta, info] = simeventsBuilder(spec, 'mm1_queue');
```

### Key options

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `'StopTime'` | `'100'` | Simulation duration |
| `'Layout'` | `true` | Auto-arrange blocks |
| `'EntityType'` | `'Structured'` | `'Structured'` (with attributes) or `'Anonymous'` |

---

## SE2b: Hybrid DES-Continuous (optional)

For systems where entities trigger or interact with continuous dynamics (e.g., a tank filling process where entity arrival starts a continuous fill):

```matlab
% Simulink Functions (callable from DES actions)
spec.simulink_functions(1).name = 'startFilling';
spec.simulink_functions(1).arguments = {'capacity'};
spec.simulink_functions(1).returns = {'fillLevel'};
spec.simulink_functions(1).body = [...];  % Blocks inside function
spec.simulink_functions(1).body_wiring = [...];

% Continuous blocks (integrators, gains, etc.)
spec.continuous_dynamics(1) = struct('name','FlowIntegrator', 'type','integrator', ...
    'parameters', struct('InitialCondition','0'));

% Hit crossing detectors (continuous → event bridge)
spec.hit_crossings(1) = struct('name','TankFull', 'offset','100', ...
    'direction','rising', 'source','FlowIntegrator', 'source_port',1);

% Shared data stores (DES ↔ continuous communication)
spec.data_stores(1) = struct('name','FlowRate', 'initial_value','0');
```

The solver is automatically set to `ode45` when hybrid fields are detected.

---

## SE3: Verify Entity Flow

After building, simulate and verify:
1. Entities reach the terminator (no deadlocks)
2. Queue does not overflow (check `queue_len` signal)
3. Server utilization is reasonable
4. Entity count at terminator matches expected throughput

```matlab
simOut = sim(mdl, 'StopTime', '100');
% Check statistics outputs
```

---

## SE4: Wire into System (mixed-builder context)

When the SimEvents component is part of a larger model (via `executePlan`):
- `meta.interface` provides the port contract
- Signal inputs connect from continuous subsystems (e.g., rate commands)
- Signal outputs carry statistics to Simulink scopes or controllers
- Use `use_message: true` for event-triggered bridging

---

## SE5: Validation

Typical SimEvents validation metrics:
- **Throughput** — entities/second at terminator
- **Average wait time** — from arrival to service start
- **Queue length** — max/mean over simulation
- **Server utilization** — fraction of time busy
- **Resource contention** — how often acquirers block

Use `evaluateTests` with statistical comparisons (tolerances typically wider than ODE models due to stochastic variability).

---

## When NOT to use simeventsBuilder

| Situation | Use instead |
|-----------|------------|
| Continuous dynamics (ODEs, state-space) | `odeBuilder` |
| Pure state machine logic (no entities) | `stateflowBuilder` |
| Physical network (circuits, mechanisms) | `simscapeBuilder` |
| Mode switching with ODEs per mode | `odeBuilder_cps` |
| Simple lookup/scheduling table | `lookupTableBuilder` |
