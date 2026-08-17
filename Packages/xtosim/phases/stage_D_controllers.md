# Phases 8-9: Programmatic Blocks and Sign Analysis

**Inputs:** Hierarchical model (after Phase 11), or CPS model (after Phase 7-CPS), component registry
**Outputs:** Model with all programmatic blocks added, sign chains documented, feedback loops closed
**Next stage:** Stage E (Read `stage_E_validate.md`)
**Prerequisite:** Stage D hierarchy must be complete before controllers (for continuous full pipeline)

---

**Key function calls this phase:** `findBlock`, `getBlockInfo`, `validateBlock`, `addProgrammaticBlocks`, `layoutSignalFlow`, `verifyConnections`. See SKILL.md "Mandatory Helper Function Calls" table for signatures.

**`addProgrammaticBlocks` struct format:** `.name` (must already exist in model), `.inputs` (struct array: `.signal` = source block or `'Subsys/portName'`, `.source` = `'block'`|`'subsystem_outport'`|`'constant'`|`'clock'`, `.port` = source output port), `.outputs` (struct array: `.port` = this block's output port, `.target` = dest block or `'replace_constant:Subsys/ConstName'`, `.target_port`), `.terminate` (optional output ports to ground).

---

## Phase 8: Add Control and Nonlinear Blocks (on Hierarchical Model)

**PREREQUISITE: Phase 11 (hierarchy creation) must be complete before starting Phase 8.** For CPS models, Phase 11 is skipped (Stateflow chart IS the hierarchy) -- Phase 8 adds blocks to the Simulink model surrounding the chart.

**Phase 8 runs AFTER Phase 11 (hierarchy creation).** The build tool generates the ODE/algebraic core, Phase 7 verifies it open-loop, Phase 11 creates hierarchy while `c`/`m`/`cellEq` are fresh, and then Phase 8 adds programmatic blocks to the **hierarchical** model.

**After adding programmatic blocks, update the InitFcn** to include any new parameters referenced by the blocks (gains, saturation limits, controller constants). The model must run standalone from the InitFcn alone -- see Phase 12c.

### Effort Limit: 10 Attempts Per Component

**For each programmatic component, you have at most 10 attempts to get it working.** Each attempt must be meaningfully different from the previous one. After 10 failed attempts on a single component:

1. Document what was tried and what failed in `spec.assumptions`
2. Set the component status to `build_failed`
3. Move on to the next component
4. The validation report (Phase 12) will note the incomplete component

**What counts as one component?** A single logical functional block: `PID_speed`, `Sat_output`, `Transform_coordinates`, `Feedforward_gain`. If a complex feature needs sub-blocks (e.g., a controller needs both a filter and a limiter), treat it as one component.

**What counts as an attempt:** Any modify-simulate-check cycle. Changing a single parameter is an attempt. Restructuring the entire block is an attempt. Keep an explicit count.

**What counts as a failed attempt?** The component block is added to the model AND the model either: (a) fails to simulate with an error, OR (b) simulates but produces physically implausible output (wrong sign, magnitude wildly off, oscillating when should be stable).

**What counts as a successful attempt?** The block is added, model simulates without error, AND output has correct sign and roughly correct magnitude (within ~50% of expected). Final quantitative validation (Phase 10) happens later -- Phase 8 success = block works structurally, not that validation PASS.

**What counts as meaningfully different:** Changing the algorithm (not just a sign flip), trying a different block type, simplifying the logic, or removing a problematic feature. Retrying the exact same code is NOT an attempt -- it's wasted time.

### 8a: Review the component registry

**Before adding any blocks, review the component registry from Phase 2.** List all components with `build_method: programmatic` and status `planned`.

Common programmatic components:
- **Input sources** (inverter switching, voltage waveform generators, driving cycles)
- **Coordinate transforms** (abc<->dq, Park/Clarke, body<->earth frame)
- **Controllers** (PID, autopilot, state feedback)
- **Nonlinear elements** (saturation, rate limiters, dead zones, lookup tables)
- **Output processing** (inverse transforms, signal reconstruction)

**Rule: Build ALL components as visible Simulink blocks.** Never hide signal generation inside InitFcn. InitFcn should only contain parameter assignments and ICs. `From Workspace` blocks are acceptable only for externally-provided test data.

### 8a-scenario: Command/Reference Generation (MANDATORY if spec.primary_scenario.requires includes it)

**If `spec.primary_scenario` exists and its `requires` field includes command/reference generation (e.g., `"command_generation"`, `"speed_control"`, `"trajectory_planning"`), you MUST build it in Phase 8.** This is NOT optional -- without it, the model cannot reproduce the paper's primary scenario in Phase 10.

**Implementation approach:**
1. Read `spec.primary_scenario` to understand what the command generator needs as inputs/outputs
2. Build as a **MATLAB Function block** (complex logic with if/else, phase switching) or **Subsystem with standard blocks** (simple gain + saturation)
3. Wire feedback signals from `spec.primary_scenario.feedback_signals` as inputs
4. Wire outputs to the appropriate control inputs (heading command, speed command, etc.)
5. If the paper uses lookup tables (which are not available), implement a simplified continuous approximation

**The command generator does NOT need to match the paper exactly.** A simplified version that produces qualitatively correct behavior (right direction, right magnitude, stable) is acceptable. Document simplifications in `spec.ambiguities`.

**Phase gate check:** Before declaring Phase 8 complete, verify: "Can the model run the primary scenario?" If `primary_scenario` requires command generation and it's not built, Phase 8 is NOT complete.

### 8b: Block selection — discovery and validation tools

**The LLM selects blocks using training knowledge + runtime verification.** Three helper tools support this process:

| Tool | Purpose | When to call |
|------|---------|-------------|
| `findBlock(keyword)` | Discover exact library path from a keyword | When unsure of exact path (searches live installation) |
| `getBlockInfo(library)` | Read block description, ports, port labels, all params | Before wiring a non-trivial block (understand port contracts) |
| `validateBlock(library, params)` | Pre-flight check: path exists + params valid | Always, before passing to `addProgrammaticBlocks` |

**Workflow for non-trivial blocks:**
```matlab
% 1. Find the block (if path not known from memory)
r = findBlock('transport delay');
%   -> simulink/Continuous/Variable Transport Delay

% 2. Read its documentation (understand ports and math)
info = getBlockInfo('simulink/Continuous/Variable Transport Delay');
%   -> Description: "...second input specifies instantaneous delay time Ti..."
%   -> Ports: 2 in, 1 out
%   -> Params: VariableDelayType, MaximumDelay, MaximumPoints, ...

% 3. Validate chosen params before placement
[ok, msg] = validateBlock('simulink/Continuous/Variable Transport Delay', ...
    struct('VariableDelayType','Variable transport delay','MaximumDelay','20'));
%   -> ok=true

% 4. Place with addProgrammaticBlocks using type 'Block'
blocks(k).type = 'Block';
blocks(k).library = 'simulink/Continuous/Variable Transport Delay';
blocks(k).params = struct('VariableDelayType','Variable transport delay','MaximumDelay','20');
```

**MANDATORY: For any block with 2+ inputs, call `getBlockInfo` and state the unit/meaning of each input port BEFORE wiring.** Do not rely on training knowledge for multi-input blocks — read the live description and verify units match your signals. If the description says "time" and your signal is speed, you need a conversion (e.g., `L/v` to get seconds). This rule exists because the LLM consistently skips reading when it "already knows" — and gets port semantics wrong.

**When to call `getBlockInfo` (beyond the mandatory rule above):**
- Blocks with mode-dependent behavior (e.g., VTD: "Variable time delay" vs "Variable transport delay")
- Any non-standard block (not Gain, Sum, Integrator, Constant, Mux/Demux)
- NOT needed for single-input blocks (Gain, Saturation — LLM knows these well)

### 8b-ii: Controller structural fidelity (MANDATORY for `controller_strategy: 'replicate'`)

**If the paper provides a block diagram or explicit transfer function for a controller, replicate its exact structure.** Do not substitute:
- A static gain for a first-order lag
- A PD for a PI or first-order integrator
- A continuous controller for a discrete one
- A MATLAB Function block for what the paper shows as an integrator chain

**Why this is critical:** Controller gains are designed FOR the paper's structure. `Kg' = 4.95` inside `δ̇ = Kg'·(e - δ)` means something completely different from `δ = Kg'·e`. The parameters become meaningless inside a different topology. If you change the controller structure, you invalidate every parameter the paper provides.

**Consequences of violating this rule:**
- Sign errors that don't exist in the original (requiring parameter hacks to fix)
- Stability problems requiring invented damping terms (like artificial rate feedback)
- Validation that "passes" only because the controller was retuned — proving nothing about the model

**When `controller_strategy: 'replicate'`:**
1. Read the paper's block diagram for the controller (Fig number recorded in `spec.artifacts`)
2. Identify each block: integrator, saturation, rate limiter, gain, summing junction
3. Build those EXACT blocks in Simulink (not a MATLAB Function approximation)
4. Use the paper's gain values directly — they should "just work" with original plant parameters

**When `controller_strategy: 'derive'`:** You design the controller. Document your choices. The paper's parameters may not apply.

### 8b-iii: Implementation rules

1. **Waveform data in InitFcn, not inline.** Time-varying signals defined as `timeseries` in InitFcn.
2. **MATLAB Function block code from the math.** Write transform equations directly from the source.
3. **Block naming: `<function>_<signal>`.** E.g., `Sat_output`, `Step_load`, `PID_speed`.
4. **Wiring order: signal-flow order** (left-to-right, upstream before downstream).

### 8c: Add all programmatic blocks in ONE MATLAB call

**Batch all operations into a single MATLAB evaluate call.** Each MCP tool call has ~2-5s overhead.

**Preferred: Use `addProgrammaticBlocks` helper:**
```matlab
mdl = 'hierarchical_model_name';
blocks(1).name = '<BlockName>';
blocks(1).type = 'MATLABFunction';
blocks(1).code = sprintf('function [y1, y2] = <BlockName>(u1, u2)\n...');
blocks(1).inputs(1) = struct('signal','t','source','clock','port',1);  % auto-creates Clock
blocks(1).inputs(2) = struct('signal','<param>','source','constant','port',2);  % auto-creates Constant
blocks(1).outputs(1) = struct('port',1,'target','replace_constant:PlantDynamics/u1','target_port',1);
nAdded = addProgrammaticBlocks(mdl, blocks);
save_system(mdl);
```

**CPS models:** Wire programmatic blocks to chart input ports (not subsystem internals):
```matlab
chartName = 'my_cps_model_chart';
add_block('simulink/Sources/From Workspace', [mdl '/Throttle_input'], ...
    'VariableName', 'throttle_ts', 'SampleTime', '0', 'Interpolate', 'on');
add_line(mdl, 'Throttle_input/1', [chartName '/1'], 'autorouting', 'smart');
```

### 8c-i: MATLAB Function block gotchas

**Port count:** After adding MATLAB Function blocks, you MUST call `set_param(mdl, 'SimulationCommand', 'update')` before accessing `PortHandles` -- otherwise ports show as 1 regardless of function signature.

**No caching bug in R2025b:** `chart.Script = newCode` triggers recompilation reliably. If simulation output is unchanged after a script edit, the problem is almost certainly that `strrep` silently failed to match (whitespace mismatch between the old string and the actual script content).

**Safe script editing pattern (MANDATORY):**
```matlab
rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','Path',[mdl '/BlockName']);
oldScript = chart.Script;
newScript = strrep(oldScript, oldStr, newStr);
assert(~strcmp(newScript, oldScript), 'strrep did not match — check whitespace');
chart.Script = newScript;
```

**If the assert fires:**
1. Inspect whitespace: `disp(uint8(oldStr))` vs actual script bytes around the target
2. Rewrite the full script from scratch instead of patching

**Do NOT delete/recreate the block** — this breaks all connections (7 inputs + 4 outputs for a typical HydroForces block) and costs ~5 tool calls per iteration to reconnect. The strrep+assert pattern catches failures immediately with zero wasted effort.

**Do NOT delete `slprj`** — unnecessary in R2025b; recompilation is automatic on script assignment.

**Rule of thumb:** If a component can be built with 5 or fewer standard Simulink blocks, prefer pure Simulink over a MATLAB Function block.

### 8d: Component completion check (MANDATORY -- both modes)

**Before leaving Phase 8, print the component registry:**
```
Component Registry Check:
  [built]    Plant dynamics ODEs (odeBuilder, Phase 7)
  [built]    Output algebra (odeBuilder, Phase 7)
  [built]    Input generator (programmatic, Phase 8)
  Result: ALL components built (checkmark)
```

If any component still shows `planned`, build it NOW. If it shows `build_failed` (hit the 10-attempt limit), document and proceed.

### 8e: Re-layout after adding programmatic blocks (MANDATORY — full pipeline only)

**DO NOT SKIP THIS STEP.** Phase 8 adds new top-level blocks (inverter, transforms, sources). Without re-layout, these blocks end up at default positions while subsystems remain at Phase 11 positions — creating a visually broken model with crossing lines and scattered blocks. This is the #1 visual quality issue reported by users.

Re-layout:

```matlab
% 1. Save all top-level connections
topBlocks = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
connections = {};
for i = 1:length(topBlocks)
    ph = get_param(topBlocks{i}, 'PortHandles');
    for p = 1:length(ph.Outport)
        lineH = get_param(ph.Outport(p), 'Line');
        if lineH ~= -1
            dstPorts = get_param(lineH, 'DstPortHandle');
            for j = 1:length(dstPorts)
                if dstPorts(j) ~= -1
                    connections{end+1,1} = ph.Outport(p);
                    connections{end,2} = dstPorts(j);
                end
            end
        end
    end
end

% 2. Delete all top-level lines
lines = find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
for i = 1:length(lines), try delete_line(lines(i)); catch; end, end

% 3. layoutSignalFlow with FULL chain (including Phase 8 blocks)
chain = {'freq_input', 'Integrator', 'InputGenerator', 'Transform', 'PlantDynamics', ...
         'OutputAlgebra', 'CouplingEquation', 'ActuatorDynamics'};
sources(1) = struct('name','Amp','consumer','InputGenerator','position','below');
layoutSignalFlow(mdl, chain, sources);

% 4. Reconnect
for i = 1:size(connections,1)
    try add_line(mdl, connections{i,1}, connections{i,2}); catch; end
end

% 5. Route lines cleanly
lines = find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
Simulink.BlockDiagram.routeLine(lines);
```

---

## Phase 9: Sign Convention Analysis and Loop Closure

**This is the #1 source of bugs in multi-physics models.** Do this analysis BEFORE closing any feedback loops.

### 9a: Document the full sign chain

For each feedback loop in the system, trace the complete signal path and document the sign at each step:

```
<Controller> sign chain:
  ref > measured  ->  positive error
  -> positive control signal
  -> plant gain * control (gain sign?) -> state changes
  -> state feeds back to measurement -> approaches ref
  -> error decreases -> NEGATIVE FEEDBACK (checkmark)
```

If the chain doesn't close as negative feedback, there's a sign error somewhere. Fix it before proceeding.

### 9b: Close loops incrementally

Close ONE loop at a time and verify stability after each:

1. Close the innermost/fastest loop first (e.g., inner plant dynamics)
2. Simulate -- does it stabilize?
3. Close the next loop (e.g., outer controller)
4. Simulate -- still stable?
5. Continue outward to the slowest loops

### 9c: Handle algebraic loops

Closing a feedback loop may create an algebraic loop. Fix with a `Memory` block or `Unit Delay` block:
```matlab
add_block('built-in/Memory', [mdl '/DelayBlock']);
% Wire it into the feedback signal path
```

### 9d: CPS-specific sign analysis (odeBuilder_cps only)

For hybrid/CPS systems, sign analysis applies **within each mode** and **across transitions**:

1. **Within-mode:** Trace sign chains within each mode independently.
2. **Transition guards:** Verify guard conditions fire at the correct physical boundary. Check sign of the guard variable.
3. **Transition actions (resets):** Verify state resets produce physically correct results. A missing negative sign means the ball accelerates through the ground.
4. **Cross-mode consistency:** Verify transition actions correctly map shared state values.

---

## Gate: Phases 8-9 Complete -- Before Phase 10

```
Phase Gate -- Programmatic blocks added and sign analysis complete?
- [ ] Component registry check PASSED (8d): ALL components at 'built' or 'build_failed'
- [ ] No components remain at status 'planned'
- [ ] Any 'build_failed' components documented in spec.assumptions with audit trail
- [ ] [SCENARIO] If spec.primary_scenario.requires includes command generation: BUILT
- [ ] [SCENARIO] Model can run the primary scenario (all feedback signals accessible)
- [ ] Re-layout complete (8e) -- model is visually clean
- [ ] Full sign chain documented for every feedback loop (9a)
- [ ] Loops closed incrementally, stability verified at each step (9b)
- [ ] Algebraic loops resolved if any (9c)
- [ ] [CPS only] Within-mode and cross-mode sign analysis done (9d)
- [ ] Model saved and stable
```
