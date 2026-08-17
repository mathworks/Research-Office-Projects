# Phase 11: Convert to Hierarchical Model

**Requires:** `rules.md`, `bus.md` (bus mandate for subsystem boundaries)
**Inputs:** Flat combined model (open-loop verified), `c`/`m`/`cellEq` in workspace
**Outputs:** Hierarchical model with named ports, verified identical to flat model
**Next stage:** Stage D controllers (Read `stage_D_controllers.md`)

---

**Key function calls this phase:** `createHierarchy`, `cleanupHierarchy`, `layoutSignalFlow`, `verifyConnections`. See SKILL.md "Mandatory Helper Function Calls" table for signatures.

---

**Do this immediately after Phase 7 (flat model open-loop verified), BEFORE Phase 8 (programmatic blocks).** The `c`, `m`, `cellEq` from odeBuilder are guaranteed fresh -- Phase 8 hasn't modified the model yet.

**CPS models (odeBuilder_cps):** Phase 11 does NOT apply. The Stateflow chart IS the hierarchy. For CPS, skip to Phase 8.

**Continuous models (odeBuilder):** Continue with the full Phase 11 below.

**Approach: Top-down (partition the flat model).** `Simulink.BlockDiagram.createSubsystem` preserves all internal wiring automatically.

Hierarchy creation is a **collaboration between the helper, the LLM, and the physics:**
- **`createHierarchy` helper** -- automates mechanics: equation->block mapping, integrator ownership, `createSubsystem` calls
- **LLM** -- decides the physical decomposition, validates against domain knowledge, diagnoses failures
- **Spec + computational graph** -- provide structural data

## 11a: Identify physical subsystems (FROM SPEC — not re-derived)

**Read `physical_subsystems` from the spec JSON.** The decomposition was locked in Phase 2-3. Phase 11 EXECUTES it — it does NOT re-derive, re-evaluate, or "improve" the grouping.

```matlab
spec = jsondecode(fileread(fullfile(outputDir, 'chapter_01', 'section_01_spec.json')));
subs = spec.physical_subsystems;
fprintf('Subsystems from spec (%d total):\n', length(subs));
for i = 1:length(subs)
    eqRef = ''; if isfield(subs(i),'equation_ref'), eqRef = subs(i).equation_ref; end
    fprintf('  %d. %s (%s) — eqs %s\n', i, subs(i).name, eqRef, mat2str(subs(i).equations));
    fprintf('     inports: %s\n', strjoin(subs(i).inports, ', '));
    fprintf('     outports: %s\n', strjoin(subs(i).outports, ', '));
end
```

**If `physical_subsystems` is missing port info (legacy spec):** Fall back to 11a-0 below to derive it — but this should not happen if Phase 2-3 ran correctly.

The rules below (11a-0 through 11a-iv) are ONLY used when:
- The spec was created before this rule existed (no `inports`/`outports` fields), OR
- Derivation mode (no paper — derive from first principles)

Otherwise, skip directly to 11b with the spec's decomposition.

### 11a-0: Primary rule — follow the paper's block diagram (BLOCKING GATE)

**MANDATORY FIRST ACTION: Read the paper's block diagram figure before any decomposition decision.** This is a blocking prerequisite. You MUST visually inspect the paper's system-level figure (convert to PNG if needed) and count the distinct labeled blocks. Do NOT proceed to 11a-i or 11a-ii without completing this step — even in autonomous mode.

**If the source document contains a block diagram or signal-flow diagram, match its structure literally.** The paper's authors chose that decomposition for pedagogical and physical clarity. Do not override it with merge heuristics.

**Count the subsystem blocks in the paper's diagram.** If the paper shows 4 subsystems, create 4 subsystems — not 3, not 5. Each distinct labeled block in the paper's figure becomes its own `createSubsystem` call. Do NOT merge blocks that the paper shows as separate just because they have few equations — a single-equation torque subsystem is correct if the paper shows it separately.

**Example:** If the paper's block diagram shows 4 distinct labeled blocks (e.g., "State Group A", "Algebraic Bridge", "Coupling Term", "Mechanical Dynamics"), create 4 subsystems — not 3 by merging the small ones, not 5 by splitting the large one.

**Gate check (print before proceeding):**
```
11a-0 Block Diagram Check:
- Paper figure number: ___
- Distinct blocks identified: [list names]
- Block count: N
- Proceeding with: N subsystems matching paper
```

If no block diagram exists in the paper, print "No block diagram found — proceeding to 11a-i (equation structure)."

In interactive mode, present the paper's diagram structure to the user and confirm before proceeding. In autonomous mode, follow the paper's structure without asking.

### 11a-i: When no block diagram exists — derive from equation structure

Use these general principles (in priority order):

1. **Each group of ODEs with mutual state coupling = one subsystem.** If states x₁...xₙ all appear on each other's RHS (direct mutual feedback), they form one ODE group. States that only depend on the group's outputs (not internal states) belong to a different group.

2. **Algebraic equations bridging two ODE groups = their own subsystem** if they have ≥2 equations AND the paper treats them as a distinct computation. A single coupling equation (e.g., `Te = f(states)`) becomes its own subsystem only if the paper shows it separately; otherwise attach it to its downstream consumer.

3. **Programmatic components (inverters, transforms, controllers) = their own subsystem** when they represent a physically distinct device with clear input/output boundaries.

4. **Minimum subsystem size: 2 equations** unless the paper explicitly shows a single-equation block.

### 11a-ii: Merge test — LAST RESORT tiebreaker only

**PREREQUISITE: You may ONLY apply the merge test if BOTH conditions are true:**
1. 11a-0 confirmed NO block diagram exists in the paper, AND
2. 11a-i equation-structure rules produce genuinely ambiguous grouping (two valid decompositions)

**If the paper has a block diagram, the merge test DOES NOT APPLY — period.** Do not even evaluate it. The paper showing blocks as separate is the final answer, regardless of wire count or equation count.

**The test (when prerequisites are met):** For each proposed subsystem S, ask: "What signals feed S from outside?" If ALL inports come from a single other subsystem's internal states, consider merging S into that subsystem.

**When TO merge (ONLY if no paper diagram exists):**
- Algebraic equations that ONLY consume outputs from a single upstream subsystem
- The merged subsystem would have ≤8 equations total
- The merge reduces total Inport count by ≥4 ports
- The intermediate signals are truly internal (not physically meaningful interfaces)

**When NOT to merge (hard stops — overrides everything above):**
- The paper's block diagram shows them as separate labeled blocks (this alone is decisive)
- Merging would create a subsystem with >8 equations
- The user has expressed a preference for the separated structure
- The subsystems represent physically distinct devices (e.g., actuator vs. plant, controller vs. process)
- The signals between them represent physical quantities (currents, torques, velocities)

**Default: do NOT merge.** More subsystems matching the paper is always safer than fewer subsystems that are "cleaner" by wire-count metrics.

### 11a-iii: Present decomposition and confirm (MANDATORY)

Before executing createHierarchy, **print the proposed decomposition** showing:
- Each subsystem name
- Which equations belong to it
- Boundary signals between subsystems
- **Which paper figure each subsystem corresponds to** (from 11a-0 gate check)

If the paper has a block diagram, show it side-by-side with the proposed decomposition. Ask: "The paper shows X structure. Should I follow that, or use a different grouping?"

**In autonomous mode:** Default to matching the paper's structure without asking. Only ask if there's a genuine conflict between the paper's diagram and the equation structure. **Autonomous does NOT mean "skip the paper figure check" — it means "don't ask the user, just follow the paper."**

### 11a-iv: Parameter ownership — DUPLICATE, don't share

For each subsystem, identify:
- **Equations** -- which indices (1-based into `cellEq`) belong to it
- **Internal states** -- integrator outputs the subsystem "owns"
- **Boundary signals** -- must be **physical signals** (torque, voltage, speed), not intermediate math variables

**CRITICAL RULE: Parameter Constants should be DUPLICATED inside each subsystem, NEVER shared at top level.**

A Constant block with `Value = 'Lm'` (workspace variable) costs nothing to duplicate. Sharing it via top-level wires creates ugly models with excessive Inports (8+ per subsystem instead of 4-5).

**What crosses subsystem boundaries (Inports):**
- Physical state signals (x1, x2, x3, etc.)
- Computed outputs (z1, y1, y2, etc.)
- External command inputs (u1, u2, d1)

**What stays INSIDE each subsystem (duplicated Constants):**
- System parameters (p1, p2, p3, k, m, b, etc.)
- These are just workspace variable lookups — duplication is free and correct

**After `createHierarchy` runs**, `cleanupHierarchy` handles this automatically — but the LLM must verify: if any parameter Constants ended up at the top level feeding multiple subsystems via Inports, that's wrong. The fix is to delete the top-level Constant, add a duplicate Constant inside each consumer subsystem, and delete the now-unused Inport.

**Target Inport counts:** A well-structured subsystem has 2-5 physical signal Inports (states from other subsystems + external inputs). If a subsystem has 6+ Inports, check whether parameters are leaking to the top level.

## 11b: Map equations to blocks

```matlab
allBlocks = find_system(flatModel, 'SearchDepth', 1, 'Type', 'block');
for i = 1:length(allBlocks)
    btype = get_param(allBlocks{i}, 'BlockType');
    bname = get_param(allBlocks{i}, 'Name');
    fprintf('%s  [%s]\n', bname, btype);
end
```

Verify every block is accounted for. Blocks in multiple groups' equations are **shared** -- stay at top level.

## Gate: Before 11c (MANDATORY)

**CRITICAL: You MUST run this verification code before calling `createHierarchy`.** The #1 hierarchy bug is passing wrong equation indices because the builder's output order may differ from the "logical" paper order. odeBuilder auto-sorts algebraics before ODEs. Never assume indices -- always verify by printing `cellEq`.

```matlab
%% MANDATORY: Print equation-to-index mapping and verify assignments
fprintf('=== Equation Index Verification ===\n');
for i = 1:numel(cellEq)
    fprintf('  cellEq{%2d} = %s\n', i, cellEq{i});
end
fprintf('\nProposed subsystem assignments:\n');
for i = 1:numel(subs)
    fprintf('  %s: indices %s\n', subs(i).name, mat2str(subs(i).equations));
    fprintf('    Contains: ');
    for j = subs(i).equations
        fprintf('%s, ', cellEq{j});
    end
    fprintf('\n');
end
% VERIFY: Each subsystem's equations match its physical role.
% PlantDynamics must contain \dot{xN} equations (ODEs), NOT y1/y2 (algebraics).
% OutputAlgebra must contain y1=..., y2=... (algebraics), NOT \dot{} equations.
```

**If the printout shows ODEs in an algebraic subsystem or vice versa, STOP and fix the indices.** odeBuilder auto-sorts: all algebraic equations come first (indices 1..N_alg), then all ODEs (indices N_alg+1..N_total).

```
Phase 11 Internal Gate -- Ready to create subsystems?
- [ ] physical_subsystems loaded from spec JSON (names, equations, inports, outports)
- [ ] Subsystem count matches spec (N subsystems — no merging, no splitting)
- [ ] Parameter ownership table printed (11a-iv)
- [ ] Flat model blocks enumerated (11b)
- [ ] Boundary signals match spec's inports/outports (same names, same order)
- [ ] **cellEq printed and index assignments verified (code above executed)**
- [ ] Each ODE subsystem's indices point to \dot{} equations (not algebraics)
- [ ] Each algebraic subsystem's indices point to y=f(x) equations (not ODEs)
```

## 11c: Create hierarchy with `createHierarchy`

**One MATLAB call:**
```matlab
subs(1).name = 'PlantDynamics';
subs(1).equations = [1 2 3 4 5];
subs(1).extras = {};

subs(2).name = 'OutputAlgebra';
subs(2).equations = [8 9 10 11];
subs(2).extras = {};

nCreated = createHierarchy(mdl, c, m, cellEq, subs);
save_system(mdl);
```

**IMPORTANT:** The `c`, `m`, `cellEq` variables must still be in MATLAB workspace.

**After 11c, run cleanup, naming, and wiring in this order:**
```matlab
cleanupHierarchy(mdl);     % dedup + internalize + remove unused + renumber + arrange (NO rename)
% Wire cross-subsystem algebraic outputs (z1, y1, y2) if not auto-connected
% LLM-driven port naming (11c-name): set_param calls from (x,u,y,p) table
layoutSignalFlow(mdl, chain, sources);  % 11i -- position only, never touches lines
verifyConnections(mdl);    % must pass before Phase 8
save_system(mdl);
```

**Execution order: 11c -> cleanupHierarchy -> algebraic wiring -> 11c-name -> 11i layout -> verifyConnections -> Phase 8.**

## 11c-fix: Clean subsystem ports (MANDATORY)

```matlab
cleanupHierarchy(mdl);
```

`cleanupHierarchy` handles:
1. **Deduplicate Inports** -- deletes duplicates in **descending port order** (highest first)
2. **Internalize self-feedback**
3. **Remove unused Outports** -- with protection for algebraic outputs
4. **Renumber ports sequentially**
5. **arrangeSystem** inside each subsystem

**Port naming is NOT done by cleanupHierarchy** -- done by LLM in 11c-name.

After cleanup:
```matlab
verifyConnections(mdl);
```

## 11c-name: Port naming from spec (MANDATORY)

**Port names and ordering come from `spec.physical_subsystems[i].inports` and `.outports`.** The spec was locked in Phase 2-3. Phase 11 executes it deterministically — no LLM judgment calls on naming.

**Step 1:** Read port names from spec:
```matlab
for i = 1:length(subs)
    subName = subs(i).name;
    specInports = subs(i).inports;   % cell array of signal names, in order
    specOutports = subs(i).outports; % cell array of signal names, in order
    fprintf('%s: in=[%s] out=[%s]\n', subName, strjoin(specInports,','), strjoin(specOutports,','));
end
```

**Step 2:** Read the current port inventory:
```matlab
inps = find_system(subPath, 'SearchDepth', 1, 'BlockType', 'Inport');
outs = find_system(subPath, 'SearchDepth', 1, 'BlockType', 'Outport');
```

**Step 3:** Rename AND reorder ports to match spec ordering:
```matlab
% Reorder: set temporary port numbers to avoid conflicts, then set final
for k = 1:length(specInports)
    % Find the Inport block that carries signal specInports{k}
    % (may need to trace connections to identify which In# carries which signal)
    set_param([mdl '/' subName '/In' num2str(k)], 'Name', specInports{k});
end
for k = 1:length(specOutports)
    set_param([mdl '/' subName '/Out' num2str(k)], 'Name', specOutports{k});
end
```

**Port ordering rule:** The `Port` number of each Inport/Outport block must match its index in the spec's `inports`/`outports` array. This ensures that when subsystems are placed left-to-right, lines don't cross (matching output port N connects to input port N of the next block).

**Fallback (legacy spec without port info):** If the spec lacks `inports`/`outports`, the LLM names ports from physics knowledge (x, u, y, p analysis). But this is non-deterministic — file an internal note to update the spec.

## 11d: Verify hierarchy

**Structural:** List subsystem contents, verify names, boundary signals.

**Numerical:** Simulate with same constant inputs as Phase 7d. Max diff < 1e-10:
```matlab
simOut_hier = sim(mdl);
maxDiff = max(abs(wr_hier.Data - wr_flat.Data));
assert(maxDiff < 1e-10, 'Hierarchical model differs from flat!');
```

## 11e: Diagnose failures

| Failure | Cause | Fix |
|---------|-------|-----|
| `createHierarchy:staleHandles` error | Model was closed/reloaded after build | **Cannot recover.** Delete model, re-run builder from Phase 7. NEVER close the model between build and createHierarchy. |
| `c`/`m`/`cellEq` not in workspace | Variables cleared | Re-run `[c,m,cellEq] = odeBuilder(...)` |
| "no top-level blocks remaining" for ALL groups | Stale handles (model was reloaded) | Same as staleHandles above |
| `createSubsystem` overlapping blocks | Groups share non-integrator block | Adjust equation indices |
| Wrong block count | Equation index mapping wrong | Enumerate blocks, fix indices, or fall back to 11f |
| Generic Inport names (`In1`) | Port naming not yet done | Run 11c-name |

**ANTI-PATTERN: Never do manual BFS as a workaround for createHierarchy failure.** If `createHierarchy` fails, the root cause is almost always stale handles (model reload) or incorrect equation indices. Manual BFS produces ugly models with 10+ Inports per subsystem because it doesn't understand parameter ownership. Fix the root cause instead of working around it.

**Max 3 retry attempts on hierarchy creation.** A failure = MATLAB error from `createHierarchy` or `createSubsystem` (overlapping blocks, wrong indices, empty subsystem). Port naming issues (fixed in 11c-name) and minor simulation differences (debugged in 11d) do NOT count as failures. After 3 failures, fall back to 11f.

## 11f: Manual fallback

Use when `c`/`m` genuinely cannot be recovered (e.g., MATLAB session crashed and restarted). **Even in fallback mode, you MUST handle parameter ownership correctly:**
- **Single-owner Constants** (used by only one subsystem) → include INSIDE that subsystem's handle set
- **Multi-owner Constants** (used by 2+ subsystems) → leave at top level

Steps: (1) Enumerate blocks, classify by physics. (2) For each Constant block, count how many subsystems use it — single-owner goes inside, multi-owner stays out. (3) Collect handles WITH single-owner constants, call `createSubsystem`, rename. Order: least-connected first. (4) Verify sim after EACH subsystem. (5) Name ports via LLM (11c-name). (6) Compare against flat model.

A subsystem should have at most ~4-6 Inports for physical signals (states from other subsystems + external inputs). If a subsystem has 10+ Inports, parameters are leaking to the top level — fix the ownership.

After fallback, run `cleanupHierarchy(mdl)` then 11c-name.

## 11i: Signal-flow layout (MANDATORY — DO NOT SKIP)

**Never use `arrangeSystem` for top-level layout.** Use `layoutSignalFlow`. This step is NON-OPTIONAL even in autonomous mode. Without it, subsystems end up in random positions from `createHierarchy` and the model looks unprofessional. Skipping this is visible to the user as a jumbled model.

**Call `layoutSignalFlow` IMMEDIATELY after port naming (11c-name), before verifyConnections.** Then call it AGAIN in Phase 8e after adding programmatic blocks.

**The chain order comes from the spec's `physical_subsystems` array order** (signal-flow order was locked in Phase 2-3):
```matlab
% Build chain from spec order (deterministic)
chain = arrayfun(@(s) s.name, spec.physical_subsystems, 'UniformOutput', false);
sources(1) = struct('name','we','consumer',chain{1},'position','below');
layoutSignalFlow(mdl, chain, sources);
save_system(mdl);
```

**The chain array must list ALL subsystem blocks in signal-flow order (left-to-right).** The `sources` array positions external inputs (constants, generators) relative to their consumer. If Phase 8 adds blocks (inverter, transforms), they must be included in the chain when `layoutSignalFlow` is called again in Phase 8e.

---

## Gate: Phase 11 Complete -> Proceed to Phase 8 (MANDATORY)

```matlab
assert(verifyConnections(mdl), 'Fix connections before Phase 8');
```

### Outport Completeness Check (MANDATORY)

**After hierarchy creation, verify all states and scenario signals are accessible:**

```matlab
[pass, missing] = verifyOutportCompleteness(mdl, spec);
if ~pass
    % Add missing outports before proceeding
    for i = 1:numel(missing)
        fprintf('  MISSING: %s/%s — %s\n', missing(i).subsystem, missing(i).signal, missing(i).reason);
    end
    % Fix: add Outport blocks inside subsystems, wire from integrator outputs
end
```

**Why this matters:** Dead-end states (integrators with no outport) cannot be used for:
- Feedback to command generators (Phase 8)
- Validation plotting (Phase 10)
- Paper figure reproduction

If `spec.scenario_signals` lists `["Xo", "Yo"]` but EarthPosition has no outports for these, the primary scenario CANNOT be implemented. Catch this HERE, not in Phase 10.

```
Phase 11 Complete -- Ready for Phase 8?
- [ ] Connection audit PASSED (all ports connected)
- [ ] All subsystems created and named per paper's structure or equation-derived grouping
- [ ] Port cleanup passed (11c-fix): no duplicate Inports, no unused Outports
- [ ] Dead signal check passed: every subsystem output has a downstream consumer
- [ ] Self-feedback internalized
- [ ] LLM-driven port naming done (11c-name): all ports have physical signal names
- [ ] Port counts match physical expectation (print table)
- [ ] Outport completeness PASSED: all states + scenario_signals accessible
- [ ] Signal-flow layout: subsystems ordered left-to-right
- [ ] Open-loop simulation matches flat model (max diff < 1e-6)
```
