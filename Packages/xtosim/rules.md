# Hard Rules — Always/Never (Read after any context compaction)

These rules override any improvised approach. If in doubt, follow this file.

## Data Logging [C-D]
- `executePlan` automatically calls `addSignalLogging(mdl, spec)` — do NOT call it manually
- **NEVER** add To Workspace blocks manually — they clutter the diagram
- Access data via `simOut.logsout.get('signal_name')`
- odeBuilder with `'Logging', true` adds signal logging at build time (flat models)
- For multi-subsystem models, `executePlan` adds flat logging automatically. Hierarchical logging (`addSignalLogging(mdl, spec, 'hierarchical')`) must be called manually in Phase 11 if needed after `createHierarchy`.

## Plotting [E-F]
- **ALWAYS** `autoPlotValidation(mdl, simOut, spec, figDir)` for comprehensive signal coverage (Category 2)
- **ALWAYS** generate paper figure reproductions (Category 1) for every `spec.validation_figures` entry
- **ALWAYS** `plotWithReference(simData, refData, opts)` for every `spec.reference_data` entry (overlay plots)
- **ALWAYS** match the paper's exact axis orientation and units — read the figure before plotting. If paper plots Y_lateral on X-axis and X_longitudinal on Y-axis, do the same. Never swap or rotate axes.
- **ALWAYS** mark start + target on XY trajectory plots (green triangle for start, red asterisk for target with coordinates in legend). Interpolate sparse paper data with `pchip` to show as smooth line — two smooth curves are directly comparable.
- **ALWAYS** hand-craft Category 1 plots for readability — these are presentation deliverables, not checkboxes. Use smooth interpolated lines for paper reference data (not dots), solid line for simulation, zoom to relevant region.
- Paper figure reproductions appear FIRST in the report, signal coverage SECOND
- Overlay plots (plotWithReference) count as Category 1 — they ARE the paper figure reproductions

## Standalone Reproducibility [F]
- **ALWAYS** `writeValidationScript(outputDir, mdl, testConfigs, spec)` in Phase 12 (or Phase 10m)
- The standalone script must reproduce ALL validation tests without the AI agent
- Model must be playable with just the Play button (InitFcn loads all params)

## Eigenvalue Verification (OPTIONAL — only when needed) [E]
- **ONLY IF** paper explicitly states eigenvalues/poles/time constants AND validation shows issues
- **SKIP** if all tests PASS and model behavior matches paper figures
- Convert time constants to eigenvalues: λ = -1/τ
- Convert natural frequencies: λ = -ζωn ± ωn√(ζ²-1)
- Results feed `rpt.subsystem_validation` in the report

## Sensitivity Analysis (OPTIONAL — only when needed) [E]
- **ONLY IF** validation shows CLOSE or FAIL results and root cause is unclear
- **SKIP** if all tests PASS — sensitivity adds time without value
- Run on the 3-5 most uncertain parameters
- Results feed `rpt.sensitivity` + `rpt.sensitivity_meta` in the report
- **NEVER** run as a default step — it re-simulates N×2 times which is expensive

## Report [F]
- **ALWAYS** save ALL figures to `fullfile(outputDir, 'figures')` — NOT `docs/figures` or any other path. `fillReport` emits `<img src="figures/...">` relative to `outputDir/report.html`.
- **ALWAYS** use `finalizeReport` as the SINGLE entry point for Stage F — it calls `buildReportStruct`, `fillReport`, `validateReport`, and all plot/screenshot helpers internally
- **ALWAYS** run `[pass, issues] = validateReport(rpt)` before delivery — fix any issues
- **ALWAYS** include ≥3 tests, (≥1 comparison table OR ≥1 paper figure reference), ≥1 non-circular test
- **NEVER** deliver a report that `validateReport` rejects
- **OPTIONALLY** use `generateReport(outputDir, rpt)` for PDF output (in addition to fillReport)
- **NEVER** use `generateReport` as the only report — it produces generic output
- **NEVER** write report HTML/markup manually (`fopen` + `fprintf` to `.html`) — this bypasses validation, produces incomplete reports, and is untracked
- **NEVER** implement custom test scoring logic — call `evaluateTests`
- **NEVER** call `buildReportStruct`, `fillReport`, or `validateReport` individually outside `finalizeReport` — the function composes them correctly with all required enrichment steps
- **NEVER** skip `finalizeReport` because "it's simpler to write HTML directly" — the function handles ALL edge cases (empty simOut, failed validation, missing figures)

## Paper Fidelity (Document Mode) [A-B]
- **ALWAYS** use the paper's formulation: reference frame, coordinate system, equation form, and model architecture as presented in the source document
- **ALWAYS** follow the paper's block diagram decomposition for **plant dynamics** components — blocks that contain states or algebraic outputs consumed within the feedback loop. If the paper shows separate subsystems (e.g., "Flux Linkages", "Current Algebra", "Torque", "Speed"), use one component per subsystem — do NOT merge them into fewer components. The paper's physical domain boundaries are meaningful.
- **ALWAYS** merge cascaded stateless input-generation blocks into a single programmatic block when the intermediate signals have no other consumer in the model. For example: an inverter followed by an abc-to-synchronous transform, where the abc voltages are not used elsewhere, becomes one block that directly produces dq voltages. The test: if removing the intermediate signal (abc) breaks nothing downstream, merge.
- **ALWAYS** exclude monitoring-only output blocks from the plant loop (`plan.components`). However, if a monitoring-only transform is needed to reproduce a `spec.validation_figures` entry, include it as a programmatic post-processing block in `plan.programmatic` with `role: 'monitoring'`. This block receives loop signals as inputs and produces the figure signal as output, but does NOT feed back into the plant. For example: a synchronous-to-abc transform that produces ia for Fig. 17 reproduction — add it as a MATLAB Function block that computes `ia = ids*cos(thetaE) - iqs*sin(thetaE)`, wired from existing loop signals.
- **ALWAYS** register ALL blocks from the paper's block diagram in `spec.components` during intake for traceability. Mark merged blocks with `status: 'merged_into:<target>'` and monitoring-only blocks with `status: 'not_needed'` plus justification. The plan (Stage B) then only includes structurally necessary components.
- **ALWAYS** use the paper's EXACT equation at the referenced number — do not rewrite into an "equivalent" form that uses different variables. If the paper's torque equation uses currents (iqs, idr), use currents — not a mathematically-equivalent flux-linkage form. Equivalent forms introduce different variables, different denominators, and different numerical sensitivities.
- **ALWAYS** store the paper's original equation numbers in `spec.components(i).eq_numbers` and `plan.components(i).eq_numbers` during intake — these are used for model annotations
- **ALWAYS** run full Phase 1-3 extraction (never manually construct a minimal spec — it misses `equations_raw_latex` and other fields that downstream phases depend on)
- **ALWAYS** identify ALL artifacts in the source (equations, tables, code, figures, algorithms) — not just equations
- **ALWAYS** extract parameter values from code listings when present — code output > table > prose > inferred
- **NEVER** substitute an alternative formulation (e.g., stationary frame when paper uses synchronous frame) based on personal preference or prior experience — even if both are mathematically equivalent
- **NEVER** rewrite an equation into a different variable set (e.g., replacing currents with flux linkages via the inverse inductance matrix) — even if the result is algebraically identical. Different forms have different inputs, different numerical conditioning, and different wiring requirements.
- **NEVER** skip the spec pipeline in a test run — it validates report completeness
- **NEVER** ignore code appendices/listings — they often contain the only correct parameter values

## Building [B-C] — ONE CALL: `executePlan(plan, 'Spec', spec, 'OutputDir', dir)`
- **ALWAYS** decompose into multiple `plan.components` with `plan.wiring` between them. **`validatePlan` BLOCKS single-component plans with ≥ 6 equations.** Each odeBuilder component should contain one physical subsystem (one ODE group + its directly coupled algebraics). This produces a hierarchical model matching the paper's block diagram. Override with `comp.allow_flat = true` ONLY for trivially coupled systems.
- **ALWAYS** use `executePlan(plan, ...)` as the SINGLE entry point for Stages C-F
- **ALWAYS** check the `existing` builder path FIRST: if the component already exists in a loaded model, use `builder = 'existing'` — do not rebuild what's already built
- **ALWAYS** use `odeBuilder` as the builder for novel nonlinear ODEs that don't map to any known block (it auto-sorts algebraics, sets Constants, writes InitFcn, wires algebraics). It is the LAST resort in the decision tree, not the default choice — check all other builder paths first.
- **ALWAYS** use `\dot{stateName}` for ODE left-hand sides — NEVER `d_x = ...` or `dx = ...`. odeBuilder only recognizes `\dot{}` as a derivative; anything else becomes an algebraic equation with a dead-end Outport instead of an Integrator.
- **ALWAYS** include `decision_path` for every component in the plan — `validatePlan` rejects plans without it
- **ALWAYS** call `findBlock` before claiming no standard block exists — the decision tree requires evidence, not assumption
- **ALWAYS** verify `bdIsLoaded(spec.model)` before using the `existing` builder — the source model must be open
- **NEVER** use MATLAB Function blocks UNLESS the decision tree legitimately routes there. Legitimate path requires BOTH: (a) `is_standard_block == false` (no library block exists for this equation) AND (b) `has_tightly_coupled_conditionals == true` (3+ coupled equations with if/else that cannot structurally decompose into separate blocks). Every other path leads to a proper builder.
- **NEVER** use add_block/add_line to hand-build a component that a builder can construct — if you find yourself writing `add_block(...)`, STOP. You are bypassing the pipeline.
- **NEVER** use Mux→MATLABFunction→Demux as a substitute for proper Simulink signal flow (note: this is effectively prevented by the executePlan gating above, but remains an explicit anti-pattern)
- **NEVER** use persistent variables in MATLAB Function blocks for state/mode logic — use Stateflow (stateflowBuilder) or Integrators (odeBuilder) instead
- **NEVER** close the model between build and createHierarchy (stale handles)
- **NEVER** put physical parameters in `buildInterface` ports — interface ports are for signals that flow between components at runtime. Physical parameters (m, Cd, Kp, R, L, etc.) stay as internal Constants driven by InitFcn. However, **operating conditions** (supply frequency, input voltage, load torque, reference commands) that the paper shows as system-level inputs on its block diagram ARE interface signals — expose them as top-level blocks even if they happen to be constant during a particular simulation. The test: does the paper's block diagram show it as an arrow entering the system? If yes → top-level input. If it only appears in a parameter table → internal Constant.
- **NEVER** skip the decision tree — jumping to odeBuilder or MATLAB Function without traversing branches 0-8 first is a plan violation that `validatePlan` will catch
- **NEVER** use `existing` builder as a shortcut to avoid understanding the subsystem — you must still declare the interface (externalInputs/externalOutputs) correctly for wiring to work

## Stage C-F Enforcement [C-F] — `executePlan` is the ONLY path
- **ONE CALL** runs the entire pipeline (C→D→E→F):
  ```matlab
  [topMdl, result] = executePlan(plan, 'Spec', spec, 'OutputDir', outputDir);
  % Runs: validatePlan → build → compose/hierarchy → auditModel → validate → report
  % Gates must pass. If any fails, it errors with diagnosis.
  ```
- **IF `executePlan` fails**: Read the error. Fix `plan.components(i).spec` or `plan.wiring` or `plan.programmatic`. Call `executePlan(plan, ...)` again. **NEVER** work around it by manually building blocks.
- **IF the model needs changes** (wrong architecture, missing component, wrong I/O): Fix the PLAN, `bdclose('all')`, re-run `executePlan`. Do NOT patch the model with manual block manipulation.
- **ONLY the plan is the LLM's interface to the model.** The LLM constructs and modifies the plan struct. `executePlan` is the ONLY function that touches Simulink. This ensures reproducibility: the same plan always produces the same model.
- **NEVER** hand-build when `executePlan` fails — the correct response is fixing the plan input, not abandoning the pipeline
- **NEVER** call `add_block`, `add_line`, `delete_block`, `delete_line`, `set_param` (on model blocks), `new_system`, or write MATLAB Function block code at ANY point after Stage B — **EXCEPT** through `applyFix` (see exception below).
- **NEVER** manually call internal pipeline stages (addSignalLogging, evaluateTests, buildReportStruct, fillReport) individually — use `executePlan` for the happy path or `finalizeReport` for the manual-fix path
- **BANNED MATLAB patterns (Stages C-F)** — if you type any of these, STOP and fix the plan instead:
  - `add_block(...)` 
  - `add_line(...)`
  - `delete_block(...)` / `delete_line(...)`
  - `set_param('Model/Block', ...)` (setting block parameters directly)
  - `new_system(...)`
  - `sfroot` / `chart.Script = ...`
  - `Simulink.BlockDiagram.copyContentsToSubSystem(...)`
  - `Simulink.BlockDiagram.arrangeSystem(...)` (use plan.subsystemOrder + layoutSignalFlow)
- **ALLOWED after executePlan returns** (Stage E-F only):
  - `sim(mdl)` — for additional validation scenarios (e.g., 200Nm test)
  - `set_param(mdl, 'StopTime', ...)` / `set_param('Model/Step_TL', 'After', ...)` — changing **simulation-level** parameters for re-running tests (NOT structural block changes)
  - Plotting commands (`figure`, `plot`, `saveas`)
  - `finalizeReport(mdl, simOut, spec, plan, outputDir, ...)` — standalone Stage F (MANDATORY for manual-path delivery)
  - Reading `logsout` signals for validation
- **CORRECT path for iterating on programmatic code** (Stage E debugging):
  - Update `plan.programmatic{i}.code`, then call `executePlan(..., 'IncrementalUpdate', true)` — ~3s per iteration
  - This is the ONLY sanctioned way to update MATLAB Function / Stateflow chart scripts
  - NEVER use `sfroot` / `chart.Script` directly, even "just to test quickly" — `IncrementalUpdate` IS the fast path

> **`set_param` distinction:** `set_param` on the MODEL ROOT (solver, StopTime) or on source block VALUES (Step.After, Constant.Value) for re-simulation is allowed. `set_param` on internal block STRUCTURE (adding ports, changing block types, renaming) is banned — use `applyFix` instead.

## `applyFix` Exception Path [C-E] — Tracked Manual Fixes

**When `executePlan` hits a compile error that CANNOT be resolved by plan/spec changes** — typically integration-layer issues where masked block port semantics, vector/scalar mismatches, or sign conventions between components are undocumented and only discoverable at compile time — the following controlled exception applies:

### Decision Protocol (MANDATORY — follow in order)

1. **First attempt: Fix the plan.** Change `plan.components(i).spec`, `plan.wiring`, or `plan.programmatic`. Re-run `executePlan`. This is ALWAYS the first response.
2. **Second attempt: If plan fix fails** because the root cause is unknowable from specs (e.g., a masked Aerospace Blockset block expects `[Cforce(3), Cmoment(3), qbar, V]` on 4 ports but this isn't documented anywhere), THEN and ONLY THEN use `applyFix`.
3. **Classify the error.** `applyFix` is ONLY permitted for these categories:
   - `'dimension_mismatch'` — masked block expects vector, receives scalar (or vice versa)
   - `'port_mapping'` — undocumented port semantics of library/masked blocks
   - `'sign_error'` — integration-layer sign convention mismatch (NED vs positive-up, etc.)
   - `'missing_block'` — a glue block needed between components (Gain, Fcn, Mux/Demux)
   - `'parameter_error'` — block parameter incompatible with connected signal
4. **Architecture issues are NOT permitted.** If the fix involves replacing an entire component, changing the decomposition, or rewiring the feedback loop, that is a PLAN change — go back to step 1.

### Usage (MANDATORY syntax — no bare `add_block`/`set_param`)

```matlab
% ALL post-executePlan structural edits go through applyFix:
plan = applyFix(plan, 'ComponentName', 'action_description', ...
    @() your_fix_code_here(), ...
    'Reason', 'why this fix is needed (root cause)', ...
    'Category', 'dimension_mismatch');  % one of the permitted categories

% Examples:
plan = applyFix(plan, 'top_level', 'add_altitude_sign_inversion', ...
    @() add_block('simulink/Math Operations/Gain', [mdl '/Ze_to_alt'], 'Gain', '-1'), ...
    'Reason', 'Atmosphere block needs positive altitude; EOM outputs negative Ze (NED)', ...
    'Category', 'sign_error');

plan = applyFix(plan, 'AeroForceMoments', 'replace_internals', ...
    @() replaceAeroBlock(mdl), ...
    'Reason', 'Masked block port 1 expects [Cforce(3),Cmoment(3)] but receives [all_coefs(6)]', ...
    'Category', 'port_mapping');
```

### Gate: `verifyPlanModelConsistency` (MANDATORY before Stage E)

After all `applyFix` calls and before proceeding to validation:
```matlab
[consistent, issues] = verifyPlanModelConsistency(plan, mdl);
% MUST return consistent==true to proceed to Stage E
% If false: either record the missing changes via applyFix, or revert
```

This ensures NO untracked changes exist. If you called `set_param` or `add_block` without `applyFix`, the gate catches it.

### Constraints on `applyFix` usage

- **Maximum patches per build:** If you accumulate >5 patches, the plan is wrong — `bdclose('all')`, fix the plan architecture, re-run `executePlan`
- **No silent fixes:** Every `applyFix` call prints to console and is recorded in `plan.patches`. The report includes a "Patches Applied" section listing all fixes.
- **Reproducibility:** On rebuild (`executePlan` re-run), all `plan.patches` with `reversible==true` must be re-applied. The plan.mat file persists the patch log.
- **Bare banned commands remain banned:** Typing `add_block(...)` outside of an `applyFix` function handle is STILL a violation. The wrapper is the ONLY permitted path.

## Hierarchy and Layout [D-F]
- **ALWAYS** `createHierarchy` -> `cleanupHierarchy` -> port naming -> `layoutSignalFlow`
- **ALWAYS** call `layoutSignalFlow` AGAIN after Phase 8 adds programmatic blocks (8e)
- **ALWAYS** call `layoutSignalFlow` BEFORE `saveModelScreenshots` — screenshots without layout are unacceptable
- **ALWAYS** include `equation_ref` in plan components/hierarchy when building from a paper — use the IMPLEMENTED equation numbers (not the original form if a substituted version is used)
- **NEVER** use `arrangeSystem` for top-level layout (only inside subsystems)
- **NEVER** merge subsystems that the paper shows as separate blocks
- **NEVER** take screenshots or deliver a model without `layoutSignalFlow` having been called

## InitFcn [C-D]
- **ALWAYS** `writeInitFcn(mdl, params, ics, model_id)` — model must run with just Play button
- **ALWAYS** re-call `writeInitFcn` AFTER Phase 8 adds programmatic blocks with new parameters (gains, time constants, etc.) — the initial call in Phase 7 does not know about Phase 8 parameters yet

## Subsystem Validation (Phase 7b) [C]
- Phase 7b runs **after Stage C build, before Stage D compose** (between build and hierarchy/controllers)
- `executePlan` runs Phase 7b automatically when `spec.incremental_validation == true`
- **ALWAYS** fix subsystem test failures BEFORE proceeding to compose/controllers
- **NEVER** proceed to controllers with a failing plant — controller tuning cannot fix broken physics
- **NEVER** invent subsystem validation targets analytically — only use what the paper provides

## Controller Fidelity (Phase 8) [D]
- **ALWAYS** replicate the paper's exact controller structure when `spec.controller_strategy == 'replicate'`
- **NEVER** substitute a static gain for a first-order lag (or any structural change)
- **NEVER** add invented damping terms (Kr, Kd) not in the paper to "fix" stability
- **NEVER** negate parameter signs without diagnosing the root cause first
- If the system is unstable with the paper's controller AND paper's parameters, the BUG is in your implementation — not in the paper

## Primary Scenario [A-E] (Phases 1-3, 8, 10)
- **ALWAYS** extract `primary_scenario` in Phase 1-3 (the paper's main demo/result)
- **ALWAYS** extract `scenario_signals` (signals needed for feedback and plotting)
- **ALWAYS** build command/reference generation in Phase 8 if `primary_scenario.requires` includes it
- **ALWAYS** run the primary scenario as one of the ≥3 tests in Phase 10
- **ALWAYS** read paper figures for exact timing of steps/pulses/inputs before setting block parameters — the figure is the ground truth, not your assumption about "reasonable" timing
- **NEVER** skip the primary scenario because "it's complex" — implement a simplified version

## Outport Completeness (Phase 11) [D]
- **ALWAYS** run `verifyOutportCompleteness(mdl, spec)` after hierarchy creation
- **ALWAYS** ensure all states and `scenario_signals` are observable — but prefer **signal logging** over top-level Outports
- **NEVER** leave integrator states trapped inside subsystems with no outport AND no signal logging

## Top-Level Outports vs Signal Logging [B-D]
- **Top-level Outports** (`plan.topOutputs`) are for signals that must be ROUTED externally: consumed by programmatic blocks, fed to controllers, or connected to other models. They create visible Outport blocks at the model boundary.
- **Signal logging** (automatic via `addSignalLogging`) is for OBSERVATION: validation plots, report figures, debugging. It does NOT add Outport blocks — data is accessed via `simOut.logsout.get('signal_name')`.
- **RULE: Only add a signal to `plan.topOutputs` if it needs a physical Outport block for wiring.** If you only need to plot or validate the signal, signal logging already handles it (executePlan calls `addSignalLogging` automatically).
- **Typical model:** 1-3 top-level outports (key outputs the user cares about externally), NOT 10+ outports mirroring every internal state.
- **Exception:** If a programmatic block reads a signal via `'signal <- Component.port'`, that port MUST appear in `plan.topOutputs` or `plan.wiring` to survive composition pruning (Check 9 enforces this).

## Validation Integrity (Phase 10) [E]
- **ALWAYS** include at least one open-loop test (no controller) to independently validate plant
- **ALWAYS** use `evaluateTests(tests, 'Setpoints', setpoints)` to detect circular tests
- **NEVER** validate against setpoints your own controller targets (circular validation)
- **NEVER** count circular tests toward the minimum 3 tests
- **NEVER** mark PASS for timing/shape without comparing against the paper's figure
- **ALWAYS** report honest mismatches: "Slower", "Higher overshoot" is more valuable than fake PASS

## API Usage [ALL]
- **ALWAYS** read the phase file for calling conventions BEFORE using odeBuilder, fillReport, or any helper
- **NEVER** guess argument format — check the function header or phase file examples
- **ALWAYS** inspect odeBuilder output after build: check block names, signal names, port handles
- **ALWAYS** use bare function syntax `exp(...)` OR LaTeX `\exp{...}` — both work in odeBuilder
- **ALWAYS** pass raw LaTeX in `rpt.subsystems(i).equations` — fillReport auto-converts to HTML
- **ALWAYS** populate `rpt.sim_setup` (solver, duration, ICs, notes) — fillReport renders it
- **ALWAYS** provide `rpt.overview_image` and `rpt.schematic` if the model has them

## Report Delivery Gate [F]
- **ALWAYS** read the FULL report.html after generation and verify:
  1. Every section heading has content below it (no empty sections)
  2. Every `<img src="...">` file exists in figures/ (cross-check with Glob)
  3. Equations render as readable math (not raw LaTeX backslashes)
  4. Parameter tables have data rows
  5. Validation figures match test descriptions
- **NEVER** declare "done" without this verification pass
- **NEVER** assume fillReport handled something — verify the output

## Interactive Mode — Skip Policy [ALL]

In interactive mode (user is present, giving live feedback), phases are judgment calls. Gates are not.

**Gates: NEVER skip (cheap, high-value, prevent wasted build time)**
- Schema validation (specSchema, validateBuilderSchema) — 5ms
- `validatePlan` before build — catches broken refs AND unreachable programmatic ports (Check 9)
- Per-component smoke test (if `comp.spec.smoke_test` defined) — catches coefficient errors
- Post-build compile check — 2 seconds
- Physical invariants (`spec.invariants`) — checked after first simulation, catches sign errors/divergence
- `diagnoseFailure` after any FAIL — points you at the right artifact

**Phases: Skip when clearly unnecessary (explain in one sentence)**

| Phase | Skip condition | One-sentence justification |
|-------|---------------|---------------------------|
| A (full extraction) | User provides equations/spec directly | "User is the source — no document to extract from." |
| B (decision tree checkpoint) | Single obvious component, user already said which builder | "Only one component, builder obvious, no ambiguity." |
| B (decision_path struct) | User explicitly assigns builder | "User override — tree traversal proof not needed." |
| D (compose) | Single component | "Nothing to wire." |
| E (full ≥3 test suite) | User says "just build it" or prototype mode | "User accepts unvalidated model for iteration." |
| E (sensitivity analysis) | All tests PASS | "No failures to diagnose." |
| F (report/package) | User just wants the model open | "Deliverable is the .slx, not a report." |
| F (standalone script) | Quick prototype, user will iterate | "User owns the model from here." |

**NEVER skip even in interactive mode:**
- Builder schema validation before `executePlan` — a wrong spec format wastes 30+ seconds of build time; catching it in 5ms is always worth it
- `diagnoseFailure` after test failures — without it you iterate blind
- Post-build compile check — a disconnected port caught at compose-time saves the entire re-sim cycle

**The principle:** Gates are free insurance. Phases are work. In interactive mode, do the free insurance, skip the work when the user's presence makes it redundant.

## Stage Progress Messages [ALL] — MANDATORY at each boundary
- **ALWAYS** print a stage summary line in **chat output** (not inside MATLAB fprintf) immediately after each stage completes — before starting the next stage
- **NEVER** batch these at the end or print them retroactively
- **NEVER** skip a stage message — the user relies on these for progress tracking and early error detection
- Format (copy exactly, fill in values):
  - After A: `"✓ Intake: extracted N equations, M parameters from <source>. N components identified."`
  - After B: `"✓ Plan: <component_list> → <builder_list>. Proceeding to build."`
  - After C: `"✓ Build: N/N components built (Xs). Composing..."`
  - After D: `"✓ Compose: N connections wired. Running validation..."`
  - After E: `"✓ Validate: N/M tests PASS. <1-line summary of failures if any>"`
  - After F: `"✓ Delivered: <outputDir>/ — model + report + figures ready."`

## Phase Boundary Rule [ALL]
- **ALWAYS** `Read` the phase file BEFORE starting that phase
- **ALWAYS** re-read current phase file after context compaction
- **ALWAYS** re-read THIS file (`rules.md`) after context compaction
