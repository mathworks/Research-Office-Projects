# Phase 10: Simulate and Validate

**Requires:** `rules.md`
**Inputs:** Complete model (all components built, sign chains documented)
**Outputs:** Validation results (PASS/CLOSE/FAIL), simulation plots, validation report
**Next stage:** Stage F (Read `stage_F_deliver.md`)

---

**Key function calls this phase:** `evaluateTests`, `diagnoseFailure`, `autoPlotValidation`, `plotWithReference`, `verifyLinearization` (optional), `sensitivityAnalysis` (optional). See SKILL.md "Mandatory Helper Function Calls" table for signatures.

---

## Gate: Before Phase 10 (MANDATORY all variants)

```
Phase Gate -- Ready for Phase 10?
- [ ] Model saved (flat for simple path, hierarchical for full, CPS for hybrid)
- [ ] ALL components in registry have status "built" (no "planned" remaining)
- [ ] [Full/CPS only] Phase 8 programmatic blocks added (if any)
- [ ] [Full/CPS only] Sign chain documented (if feedback loops exist)
- [ ] [Simple only] arrangeSystem called on flat model
- [ ] Signal logging enabled via `addSignalLogging(mdl, spec, ...)` (NOT To Workspace blocks)
- [ ] verifyPlanModelConsistency(plan, mdl) returns consistent==true
```

### Model Consistency Gate (MANDATORY — blocks validation if failed)

```matlab
[consistent, issues] = verifyPlanModelConsistency(plan, mdl);
if ~consistent
    fprintf('BLOCKED: %d unrecorded changes detected:\n', numel(issues));
    cellfun(@(s) fprintf('  • %s\n', s), issues);
    error('Cannot proceed to validation with untracked model changes. Use applyFix to record, or revert.');
end
```

This catches ANY structural changes made outside `applyFix` (bare `add_block`, `set_param`, etc.). If the gate fails:
1. Identify the untracked changes from the `issues` list
2. Record each via `applyFix(plan, component, action, @() ..., 'Reason', ...)` 
3. Re-run `verifyPlanModelConsistency` until it passes
4. Only then proceed to 10a

---

## 10a: Design multiple validation test scenarios

### MANDATORY FIRST ACTION: Read spec.simulation and spec.validation (BLOCKING GATE)

**Before designing ANY test, read the spec JSON's `simulation` and `validation` fields.** These were populated in Phase 1-3 by extracting test conditions directly from the paper's figures and text. They are the ground truth for what to simulate and what to compare against.

```matlab
spec = jsondecode(fileread(specFile));
fprintf('=== Spec-defined simulation conditions ===\n');
if isfield(spec, 'simulation')
    simCfg = spec.simulation;
    fprintf('  stop_time: %g s\n', simCfg.stop_time);
    if isfield(simCfg, 'load_torque'), fprintf('  load_torque: %g\n', simCfg.load_torque); end
    if isfield(simCfg, 'load_step_time'), fprintf('  load_step_time: %g s\n', simCfg.load_step_time); end
end
fprintf('\n=== Spec-defined validation targets ===\n');
if isfield(spec, 'validation')
    for i = 1:numel(spec.validation)
        v = spec.validation(i);
        fprintf('  %s: %.4g %s (source: %s)\n', v.name, v.expected, v.unit, v.source);
    end
end
```

**Rules:**
1. **Use `spec.simulation` conditions** (stop_time, load_torque, load_step_time) as the primary test setup — do NOT invent your own test values when the spec defines them.
2. **Use `spec.validation` targets** as the expected values — these come from the paper's figures/tables, not from generic physics formulas.
3. **Match the paper's units** in plots: if `spec.validation` uses rad/s, plot in rad/s (not rpm). If it references a specific figure, your plot must be visually comparable.
4. **You may ADD extra tests** beyond what the spec defines (e.g., locked rotor if not listed), but you must NOT skip or replace the spec-defined ones.
5. If `spec.simulation` or `spec.validation` is missing (legacy spec or derivation mode), fall back to designing tests from physics. But for document-mode builds with a populated spec, the spec is authoritative.

**Gate check (print before proceeding):**
```
10a Spec Validation Check:
- [ ] spec.simulation read: stop_time=___, load conditions=___
- [ ] spec.validation read: N targets from paper figures
- [ ] Test conditions match spec (not improvised)
- [ ] Plot units match spec.validation units
```

---

### No circular validation (MANDATORY)

**Expected values must come from physics or paper figures — NEVER from targets embedded in your own controller.**

If the controller drives a signal to a setpoint (e.g., `Yo_desired = 130`), testing that the signal reaches that setpoint proves NOTHING about the model physics. You are testing the controller tuning, not the plant.

**Valid expected values:**
- Paper states "lateral separation reaches 130 ft by t=100-150s" (from Fig 16) → test timing and shape
- Equilibrium from physics: `V_cruise = sqrt(K1/K2) * N_eq` → test steady-state speed
- Open-loop eigenvalue: "system settles with time constant 6s" → test step response shape

**Invalid expected values:**
- Controller setpoint: `Yo_desired = 130` → "Yo converges to 130" (circular — of course it does)
- Values you tuned to achieve: Kr=5 chosen to make system stable → "system is stable" (self-fulfilling)

**What to test instead of setpoint convergence:**
- **Transient shape:** Does it match the paper's figure? (timing, overshoot, oscillation count)
- **Cross-coupling:** While Y→130, what happens to heading? Speed? (paper shows <8° excursion)
- **Timing:** Paper says convergence by t=150s. If yours takes 400s, controller is wrong even if final value matches.
- **Open-loop subsystem behavior:** Straight-line, rudder step, step response — these can't be faked by tuning.

### Include open-loop tests (Phase 7b results)

**If Phase 7b subsystem validation was performed**, include those results in the Phase 10 report. They provide independent proof that the plant dynamics are correct, separate from the closed-loop behavior. This is particularly valuable for complex systems where closed-loop results alone could mask compensating errors.

---

Design **at least 3 test scenarios** exercising different operating regimes. A single scenario is NEVER sufficient. For each test, define:
1. Name
2. Setup (parameter/input changes)
3. Expected result with source (**from spec.validation or paper, not invented**)
4. Tolerance (+/-5% PASS, +/-25% CLOSE)
5. Duration (**from spec.simulation.stop_time**)

**MINIMUM 3 tests is a HARD RULE.** Even simple models need: (a) a baseline/no-load test, (b) a nominal operating point test, (c) a stressed/overload test. At least ONE test must be open-loop (no controller in the loop) to independently validate plant physics.

### PRIMARY SCENARIO TEST (MANDATORY)

**If `spec.primary_scenario` exists, one of the 3+ tests MUST run that scenario.** This is the paper's main demonstration — the whole point of the model. It cannot be skipped or replaced with a simpler test.

Checklist:
- [ ] `primary_scenario` read from spec
- [ ] Test uses `primary_scenario.stop_time` and `primary_scenario.initial_conditions`
- [ ] Test produces plots comparable to `primary_scenario.paper_figures`
- [ ] Test includes a comparison table (paper vs simulation) with honest PASS/CLOSE/FAIL scoring
- [ ] Timing/shape discrepancies documented with root cause (e.g., "simplified PD vs paper's lookup tables")

### ANTI-CIRCULAR VALIDATION (MANDATORY)

**Use the `'Setpoints'` argument to `evaluateTests`:**
```matlab
setpoints.V_cmd = V_cmd_value;
setpoints.psi_cmd = psi_cmd_value;
% ... all controller setpoints
results = evaluateTests(tests, 'Setpoints', setpoints);
```

If any test is flagged as circular, it does NOT count toward the minimum 3 tests. You must add a non-circular replacement.

### FIGURE CONSISTENCY (automated by `executePlan`)

`executePlan` automatically calls `checkFigureConsistency(simOut, spec, plan)` after invariants pass. This catches the #1 validation failure mode: **model simulates cleanly but doesn't exercise the scenario** (zero/flat outputs, wrong wiring, missing controller).

For each `spec.validation_figures` entry, it verifies:
1. Signal exists in logsout and is non-trivial (not flat/zero)
2. If `y_ranges` defined: signal uses >5% of the expected range
3. If `targets` defined: signal reaches within 50% of target value
4. If `expected_range` defined: final/peak/trough within declared bounds

**To enable this check:** populate `expected_range` in `spec.validation_figures` during Stage A (read approximate values from paper figures, use generous ±30-50% bounds). Even without `expected_range`, flat/zero detection catches the most common failures.

### Common test patterns:
- **Motor** -> locked rotor (stall current) / no-load startup (sync speed) / load step (speed drop, torque match)
- **Vehicle** -> cruise / step steer / braking
- **Ship** -> steady ahead / turning circle / zig-zag (+ paper's primary scenario)
- **Spring-damper** -> free decay / forced / step
- **CPS** -> mode transitions, energy dissipation, terminal behavior, no chatter

## 10b: Run test scenarios

```matlab
set_param(mdl, 'StopTime', 'T');
set_param(mdl, 'SolverType', 'Variable-step', 'Solver', 'VariableStepAuto');
simOut = sim(mdl);
```

**Data access after simulation (signal logging):** When using signal logging (preferred — see Phase 7c), data is in `simOut.logsout`:
```matlab
simOut = sim(mdl);
wr_data = simOut.logsout.get('wr_out').Values.Data;
t = simOut.logsout.get('wr_out').Values.Time;
% Or iterate:
for i = 1:simOut.logsout.numElements
    sig = simOut.logsout.get(i);
    fprintf('%s: final = %.4f\n', sig.Name, sig.Values.Data(end));
end
```

**Data access (To Workspace blocks fallback):** If using `addLogging` with ToWorkspace blocks, data is accessed via `simOut.<varName>` (timeseries) or directly from the base workspace depending on model config. Check `class(simOut)` before indexing.

**`yout` Dataset gotcha:** If using `SaveOutput='on'`, `yout` Dataset elements may have empty `.Name` fields. Access by index (`yout{1}.Values`) not by name (`yout.get('V')`).

## 10c: Extract and plot results

**Figure directory convention (CRITICAL):** `figDir` MUST be `fullfile(outputDir, 'figures')` — NOT `docs/figures` or any other subdirectory. The `fillReport` helper generates `<img src="figures/...">` relative to `outputDir/report.html`, so figures must be at that exact path.

```matlab
figDir = fullfile(outputDir, 'figures');
if ~exist(figDir, 'dir'), mkdir(figDir); end
```

**Two categories of figures are generated, in this priority order:**

### Category 1: Paper figure reproductions (from `spec.validation_figures`)

**These are the primary validation deliverable — hand-craft each one for presentation quality.** Category 1 plots are NOT checkboxes. They are the first thing a reader sees in the report and must be immediately readable without explanation. Spend time on each figure: match the paper's layout, add context markers, ensure axis labels and legend are clear.

Each entry in `spec.validation_figures` defines one figure to generate that matches a specific paper figure. The spec drives everything: signals, layout, axis ranges, units, and time range.

**Presentation rules for Category 1 (MANDATORY):**
- **Trajectory plots (position vs position):** Mark start point (green triangle) and target point (red asterisk with coordinates in legend). Interpolate paper data with `pchip` to show it as a smooth line — two smooth curves are directly comparable. Zoom axis limits to the relevant region (hide oscillation artifacts beyond the maneuver).
- **Time-series comparisons (signal vs time):** Simulation as solid line, paper reference as smooth interpolated line (different color). Both are continuous — use lines, not dots.
- **Sparse paper data as dots:** Only use markers (no line) when the paper data represents discrete measurements (e.g., experimental samples at specific operating points), NOT when it represents a continuous trajectory digitized at sparse intervals.
- **All plots:** Full axis labels with units, legend with ≤4 entries, grid on, title citing paper figure.

```matlab
% Read validation_figures from spec
if isfield(spec, 'validation_figures')
    valFigs = spec.validation_figures;
    fprintf('Generating %d paper-figure reproductions:\n', numel(valFigs));
    for i = 1:numel(valFigs)
        fprintf('  %s: %s (%s)\n', valFigs(i).paper_fig, valFigs(i).title, strjoin(valFigs(i).signals, ', '));
    end
end
```

For each `validation_figures` entry:
1. Extract the listed signals from `simOut.logsout` (apply conversions from `.notes` field)
2. Create figure with `.layout` (`"stacked"` = subplots, `"overlay"` = same axes)
3. Set axis limits from `.y_ranges` and `.t_range`
4. Set axis labels from `.units`
5. Title = `.title` + " (cf. " + `.paper_fig` + ")"
6. Save as `fig_paper_<fig_number>.png`

**Plot units, axes, and orientation must match the paper's figures exactly.** This means:
- If the paper plots Y_lateral on the X-axis and X_longitudinal on the Y-axis (common in trajectory plots), do the same — do NOT swap axes to "look nicer"
- If the paper plots speed in rad/s, use rad/s (not rpm)
- If the paper's Y-axis runs from -3500 to 500, use that range
- The report reader must be able to hold your figure next to the paper's figure and see direct visual correspondence with no mental rotation or axis swapping required
- **READ THE PAPER'S FIGURE** (convert page to PNG if needed) before plotting — never assume axis assignments from the figure title alone

### Category 2: Comprehensive signal coverage (from `autoPlotValidation`)

**Use `autoPlotValidation` helper (MANDATORY):**
```matlab
figFiles = autoPlotValidation(mdl, simOut, spec, figDir);
```

This generates one figure per physical subsystem defined in `spec.physical_subsystems`, plus an optional inputs figure. File naming is `fig_<sanitized_subsystem_name>.png`. The helper is **domain-agnostic** — it works for any dynamic system by reading signal groupings from the spec.

**Generated figures (spec-driven):**
- One PNG per `spec.physical_subsystems[i]` entry, plotting the signals listed in `.outports`
- `fig_inputs.png` — input/disturbance signals (from `spec.inputs` and/or `spec.simulation.load_step_time`)
- If no `physical_subsystems` defined, falls back to plotting all logsout signals individually (up to 8)

**Layout decision (automatic):**
- Signals with similar magnitudes (peak ratio < 10) → overlay with legend
- Signals with very different scales → subplots (one per signal, max 6)
- Single signal → one axes

**Signal discovery:** The helper searches logsout by exact name first, then tries rule-based aliases (Greek prefix splitting, CamelCase boundary insertion, underscore removal, case variants). This handles naming mismatches between spec and model (e.g., `lamds` finds `lam_ds` or `lambda_ds`).

**Display names:** Signal names are auto-formatted for TeX axis labels using Greek prefix detection (e.g., `lamds` → `\lambda_{ds}`, `wr` → `\omega_{r}`, `Te` → `T_e`).

### Figure ordering in report

Paper-figure reproductions (Category 1) appear FIRST in the report's Simulation Results section, followed by the comprehensive per-subsystem plots (Category 2). This puts the direct paper comparison front-and-center.

**MINIMUM figure set:**
1. **All `spec.validation_figures` entries reproduced** (paper figure matches)
2. **ALL state variables** grouped by physical subsystem (one figure per subsystem)
3. **ALL algebraic outputs** grouped by physical subsystem
4. **Key dynamics** visible in transient + steady-state regions

## 10d: Visual comparison with reference figures

**Compare simulation results directly against the paper's figures and data.** This is the most important validation step.

1. **Re-read the paper pages** containing result plots and data tables
2. **For each simulation plot**, identify the corresponding paper figure
3. **Read both images** and compare visually -- shape, scale, key features
4. **Extract approximate numeric values** from reference plots by reading axis values
5. **Carry the paper figure reference into the report** -- every simulation figure caption must cite the paper figure
6. **If the paper has data tables**, extract values as expected values in `tests` struct

For derivation mode, compare against physics-based expectations.

## 10e: Physical plausibility check

Before comparing numbers:
- Are magnitudes reasonable?
- Are signs correct?
- Do steady-state values match known physics?
- Are transient behaviors reasonable?

## 10f: Quantitative validation with PASS / CLOSE / FAIL

```matlab
tests(1).name = 'No-load speed';
tests(1).expected = 1800;
tests(1).simulated = wr_steady * 30/pi;
tests(1).unit = 'rpm';
tests(1).source = 'synchronous speed = 120*f/P';
results = evaluateTests(tests);
```

| Status | Criteria | Meaning |
|--------|----------|---------|
| **PASS** | Within +/-5% | Model accurately reproduces the target |
| **CLOSE** | Within +/-25% but outside +/-5% | Correct trend, minor quantitative discrepancy |
| **FAIL** | Outside +/-25%, wrong sign/trend, or divergent | Significant error |

**CLOSE is acceptable** -- document why.

## 10g: Diagnose discrepancies with `diagnoseFailure` (MANDATORY)

**Step 1: Run automated diagnosis FIRST.**

```matlab
% After evaluateTests shows CLOSE or FAIL:
failIdx = ~strcmp({results.status}, 'PASS');
if any(failIdx)
    diagnosis = diagnoseFailure(topMdl, extraction, simOut, results);
    % diagnosis is a struct array with one entry per failed test
end
```

**Step 2: Read the diagnosis and decide what to fix.**

The diagnosis classifies each failure into a category and tells you WHERE to fix it:

| Category | Fix Target | What to change |
|----------|-----------|----------------|
| `numerical_blowup` | plan (solver) or extraction (IC/equations) | Add solver override, fix ICs, fix algebraic loops |
| `zero_output` | plan (wiring) | Fix disconnected ports, missing IC, dead connections |
| `wrong_sign` | extraction (equations) | Flip sign convention in derivation |
| `integer_multiple` | extraction (parameters/units) | Fix unit conversion (rad↔deg, Hz↔rad/s, etc.) |
| `order_of_magnitude` | extraction (parameters) | Fix SI prefix mismatch (mm↔m, kW↔W) |
| `parameter_error` | extraction (parameters) | Re-read parameter from source |
| `timing_error` | extraction (time constants) | Fix J, B, R, C, or damping values |
| `steady_state_offset` | extraction (equilibrium) | Add missing drag/friction/bias term |
| `dynamics_shape` | extraction (equations) | Wrong damping model or order |
| `unclassified` | both | Manual investigation |

**Step 3: Fix the CORRECT artifact.**

- `fix_target = 'extraction'` → modify the extraction spec (equations, parameters, or ICs). Then regenerate the plan component specs from the updated extraction.
- `fix_target = 'plan'` → modify plan.components (wiring, solver, builder config). The extraction is correct; the BUILD instructions are wrong.
- `fix_target = 'simulation_setup'` → modify plan.stopTime, solver, or test conditions. The model is correct but the test is misconfigured.
- `fix_target = 'both'` → unclear root cause. Use `sensitivityAnalysis` (10l) to isolate.

**Step 4: Reason about the physics before coding a fix:**
1. Trace the signal path: which subsystem's output is wrong?
2. Dynamics problem (wrong equation) or wiring problem (wrong sign, missing conversion)?
3. Coordinate frames and sign conventions
4. Nondimensional vs dimensional quantities

## 10h: Refinement loop (MAX 10 ITERATIONS)

**If validation fails, iterate at most 10 times.** Each iteration is one complete cycle: diagnoseFailure → interpret → fix → rebuild → re-simulate → re-validate. Each iteration must be meaningfully different from the previous one.

**Each iteration:**
1. **`diagnoseFailure`** — run automated classification (already done in first pass; re-run after each fix)
2. **Interpret diagnosis** — does the category match your physics reasoning? If not, override with manual diagnosis (explain why).
3. **Targeted fix** — change the correct artifact (extraction vs plan) per `fix_target`
4. **Rebuild** — `[topMdl, result] = buildModel(plan)` (or `executePlan(plan)`)
5. **Re-simulate and re-validate**
6. **Keep an audit trail** — log: iteration #, diagnosis category, fix applied, result

**Example counting:** Diagnosis points to parameter scaling -> fix parameter -> re-simulate -> still FAIL = 1 iteration spent. Diagnosis points to wrong equation form -> change equation -> re-simulate -> PASS = 2 iterations total. Stop.

**After 10 failed iterations -- terminal failure handling:**
1. Set `spec.status = 'validation_failed'`
2. Document in `spec.assumptions`: what was tried, likely root cause, what might fix it
3. Proceed to Phase 12 -- report generated with **VALIDATION INCOMPLETE** banner
4. Report includes: all plots (even wrong ones), audit trail, clear statement of mismatch
5. In interactive mode, **inform the user** before proceeding

**Do not keep iterating past 10.** Diminishing returns waste time and context.

## 10i: Write validation report

Save as `chapter_##/validation_report.md`:
- All test scenarios with PASS/CLOSE/FAIL
- Target figure/table/text claim from source
- Expected vs. simulated values
- Mismatch summary with root causes
- Audit trail of fixes

## 10j: Paper Figure Overlay Plots (if spec.reference_data exists)

**If `spec.reference_data` is populated,** generate overlay plots showing simulation vs. paper data using `plotWithReference`. This is the strongest visual validation — the reader sees simulation and paper data on the same axes.

```matlab
if isfield(spec, 'reference_data') && ~isempty(spec.reference_data)
    for i = 1:numel(spec.reference_data)
        rd = spec.reference_data(i);
        
        % Build simData struct from logsout
        simData.time = simOut.logsout.get(rd.signals(1).name).Values.Time;
        for s = 1:numel(rd.signals)
            sig = simOut.logsout.get(rd.signals(s).name);
            simData.signals{s} = struct('name', rd.signals(s).name, ...
                'data', sig.Values.Data, 'unit', rd.signals(s).unit);
        end
        
        % Build refData struct from spec
        refData.time = rd.signals(1).time;
        for s = 1:numel(rd.signals)
            refData.signals{s} = struct('name', rd.signals(s).name, ...
                'data', rd.signals(s).data);
        end
        
        % Plot options
        opts.title = sprintf('%s (cf. %s)', rd.signals(1).name, rd.paper_fig);
        opts.paper_fig = rd.paper_fig;
        opts.layout = 'stacked';
        opts.figDir = figDir;
        opts.filename = sprintf('fig_overlay_%s.png', lower(strrep(rd.paper_fig,' ','_')));
        
        plotWithReference(simData, refData, opts);
    end
end
```

**Rules:**
1. Signal names in `reference_data` must match logsout signal names (or use aliases).
2. If a signal is not found in logsout, skip that overlay (don't error).
3. Overlay figures appear in the report BEFORE signal-coverage plots (Category 1 priority).

---

## 10k: Eigenvalue Verification (OPTIONAL — only when diagnosing issues)

**Skip this step if all validation tests PASS and model behavior matches paper figures.** Only run when: (a) validation shows CLOSE/FAIL and you need to isolate whether plant dynamics are correct, or (b) the paper explicitly states eigenvalues and you suspect an implementation error.

```matlab
% Example: paper states steering eigenvalues
if isfield(spec, 'subsystem_tests')
    for i = 1:numel(spec.subsystem_tests)
        st = spec.subsystem_tests(i);
        if any(strcmp({st.checks.type}, 'eigenvalues'))
            % Build or extract A matrix for this subsystem
            % (from linearization or from spec.A_matrices)
            eigCheck = st.checks(strcmp({st.checks.type}, 'eigenvalues'));
            paperEigs = eigCheck.expected;
            eigResults = verifyLinearization(A_subsystem, paperEigs, ...
                struct('verbose', true, 'names', {{st.subsystem}}));
        end
    end
end
```

**When to use:**
- Paper explicitly states eigenvalues → always verify
- Paper gives time constants τ → convert: λ = -1/τ, then verify
- Paper gives natural frequencies ωn, damping ζ → convert: λ = -ζωn ± ωn√(ζ²-1), verify
- Paper gives poles in Laplace domain → use directly

**Results feed the report** as `rpt.subsystem_validation` entries (type = eigenvalue).

---

## 10l: Sensitivity Analysis (OPTIONAL — only when diagnosing CLOSE/FAIL)

**Skip this step if all validation tests PASS.** Only run when validation shows CLOSE or FAIL results and you need to identify which parameter is the likely culprit. This step re-simulates N×2 times and is expensive.

```matlab
% Select metric: e.g., final value of key state
metric_fn = @(simOut) simOut.logsout.get('Yo').Values.Data(end);

% Or struct-based:
metric.signal = 'V';
metric.type = 'final';

% Select parameters to perturb (from spec.parameters, subset of most uncertain)
params = struct('K_psi', K_psi, 'T1', T1, 'a11', a11);

% Run
sensResults = sensitivityAnalysis(mdl, params, metric_fn, ...
    struct('delta', 0.10, 'verbose', true, 'plot', true, ...
    'figDir', figDir, 'filename', 'sensitivity_tornado.png'));
```

**When to run:**
- After validation shows CLOSE or marginal PASS results (identify likely culprit)
- When paper provides approximate parameters (reading from plots) — verify sensitivity to those
- When multiple parameter sources disagree — which matters more?

**Results feed the report** as `rpt.sensitivity` and `rpt.sensitivity_meta` fields.

---

## 10m: Save results and scripts

Save simulation results, standalone simulation script, and validation script.

**Standalone validation script (MANDATORY):**
```matlab
% Generate run_validation.m that reproduces all tests standalone (no AI agent needed)
scriptFile = writeValidationScript(outputDir, mdl, testConfigs, spec);
```

The `testConfigs` struct array must have fields `.name`, `.stop_time`, `.setup_code`, `.extract_code`, `.plot_code`, `.checks` for each test. Build this during Phase 10b/10c as you run each test.

Update spec status to `validated` (or `needs_review` if any FAIL remains).
