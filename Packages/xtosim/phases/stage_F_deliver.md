# Phase 12: Annotations, Layout, InitFcn, and Report

**Requires:** `rules.md`
**Inputs:** Final validated model, simulation plots from Phase 10, validation results
**Outputs:** Annotated model, HTML report, LaTeX report (+ PDF if TeX available)

---

## Entry Point: `finalizeReport`

Whether `executePlan` ran to completion or you operated manually via `applyFix`,
report generation goes through ONE function:

```matlab
[reportFile, pass, issues, rpt] = finalizeReport(mdl, simOut, spec, plan, outputDir, ...
    'SessionStart', sessionStart);
```

This function calls `autoPlotValidation`, `saveModelScreenshots`, `buildReportStruct`,
`fillReport`, `validateReport`, and `writeValidationScript` internally.
Do NOT call these individually or write HTML manually.

| NV Pair | Purpose | Default |
|---------|---------|---------|
| `SessionStart` | ISO timestamp from Stage A start | `''` (omits timing) |
| `TestResults` | Pre-computed test results struct | `[]` (skips test enrichment) |
| `CpsLogsout` | CPS-mode logged signals | `[]` |
| `Validated` | Whether Stage E passed | `true` |
| `Diagnosis` | Failure diagnosis struct (`.root_cause`, `.fix_target`) | `struct()` |
| `BuildTime` | Formatted build duration string | `''` |
| `Verbose` | Print progress messages | `true` |

---

**Key function calls this phase (all called by `finalizeReport` internally):** `saveModelScreenshots`, `fillReport`, `validateReport`, `writeValidationScript`, `generateReport`, `compileLaTeX`. See SKILL.md "Key Helper Functions" for signatures.

---

**Phase 12 adapts to the pipeline variant:**
- **Full pipeline:** Annotate subsystems, color-code, layout with `layoutSignalFlow`, verify InitFcn on hierarchical model.
- **Simple path:** Annotate the flat model directly. Layout is `arrangeSystem`. Verify InitFcn on flat model.
- **CPS:** Annotate the Stateflow chart block and its modes. Layout is `arrangeSystem` on top-level.

**Batch Phase 12 into 2 MATLAB calls:**
1. **Call 1:** Annotations + layout + coloring + auto-arrange + save (12a-12d)
2. **Call 2:** Model screenshots + populate `rpt` struct + `generateReport` + `compileLaTeX` + open HTML (12e)

---

## 12a: Add equation annotations to each subsystem

Use the `'rich'` interpreter with HTML formatting. Simulink does NOT support a `'latex'` interpreter.

**Full pipeline:** Add annotations **both** next to each subsystem block in the top-level model **and** inside each subsystem.

**Simple path:** Add annotations next to key blocks in the flat model.

**CPS models:** Annotate the Stateflow chart block with mode descriptions, guard conditions, and transition actions.

**Full pipeline annotation example:**
```matlab
ann = Simulink.Annotation('top_model/Propulsion_Eq');
ann.Position = [x, y_below_block];
ann.Interpreter = 'rich';
ann.Text = ['<span style="font-size:10pt;">' ...
    '<b>Propulsion (Eq 10a-b)</b><br/>' ...
    'V&#775; = K<sub>1</sub>N<sup>2</sup> &minus; K<sub>2</sub>V<sup>2</sup><br/>' ...
    'N&#775; = K<sub>5</sub>Q<sub>T</sub> &minus; K<sub>3</sub>N &minus; K<sub>4</sub>N<sup>2</sup><br/>' ...
    '<i>V = surge velocity, N = shaft speed</i>' ...
    '</span>'];
```

**HTML entity tips:** `&#775;` for derivative dot, `&psi;`/`&delta;`/`&omega;` for Greek, `<sub>`/`<sup>` for sub/superscripts, `&minus;` for minus.

Each annotation should include:
- Section title and equation numbers
- All governing equations with proper math formatting
- State variable definitions
- Key parameter names and values

## 12b: Layout and beautification

**Full pipeline:** Layout should be clean from Phase 8e. If annotations shifted positions, re-route lines with `Simulink.BlockDiagram.routeLine(lines)`.

**Simple path:** Run `Simulink.BlockDiagram.arrangeSystem(mdl, FullLayout=true)`.

**CPS:** Run `Simulink.BlockDiagram.arrangeSystem(mdl, FullLayout=true)`.

**Cosmetic refinements (one MATLAB call):**
- **[Full only] Subsystem coloring:** Handled automatically by `layoutSignalFlow` (Step 6). No manual action needed.
- **Annotations:** Place below blocks (>=20px gap). No overlaps.
- **[Full only] Internal layout:** `arrangeSystem` inside each subsystem (safe).
- **Final auto-arrange:** `Simulink.BlockDiagram.routeLine` on all top-level lines.

## 12c: Verify parameter initialization

**CRITICAL: The user must be able to open the model and click Play without running any scripts.**

```matlab
save_system('final_model');
close_system('final_model');
clearvars -except mdl  % Clear workspace to simulate fresh open
load_system('final_model');
sim('final_model');  % Should run without errors
```

**Full pipeline only:** If hierarchical model has a different name than flat, re-run `writeInitFcn`.

**Common failure here:** InitFcn was written in Phase 7 with only plant parameters, but programmatic blocks added in Phase 8 reference additional parameters (gains, saturation limits, controller constants) that were never added to InitFcn. If this test fails with "Invalid setting for parameter 'Gain'" or similar, update the InitFcn to include ALL programmatic block parameters.

## 12d: Clean disconnected lines and save the model

**MANDATORY: Remove all disconnected (red) lines before saving.** These are left behind when blocks are deleted without first deleting their connected lines (e.g., removing Outport blocks in Phase 8). One sweep at the end catches all of them regardless of where they originated.

```matlab
% Delete all disconnected lines (src or dst == -1)
allLines = find_system(mdl, 'FindAll', 'on', 'Type', 'line');
for i = 1:numel(allLines)
    try
        src = get_param(allLines(i), 'SrcPortHandle');
        dst = get_param(allLines(i), 'DstPortHandle');
        if src == -1 || all(dst == -1)
            delete_line(allLines(i));
        end
    catch
    end
end

% Route remaining lines cleanly
topLines = find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
Simulink.BlockDiagram.routeLine(topLines);

save_system(mdl);
```

## 12e: Generate reports and open them

Generate **both** an HTML report and a LaTeX report.

### Step 1: Save model screenshots

**Figure directory convention (CRITICAL):** All figures (Phase 10c plots AND model screenshots) MUST be saved to `fullfile(outputDir, 'figures')`. The `fillReport` helper generates `<img src="figures/...">` relative to `outputDir/report.html`. Using any other subdirectory (e.g., `docs/figures`) will cause broken image links in the report.

Simulation PNGs were already saved in Phase 10c -- do NOT regenerate. Capture model screenshots:

```matlab
figDir = fullfile(outputDir, 'figures');  % MUST match Phase 10c figDir
screenshotFiles = saveModelScreenshots('model_name', figDir);
```

This saves `fig_model.png` (top-level) plus `fig_<SubsystemName>.png` for each subsystem.

### Step 2: Generate report — PRIMARY method: `fillReport` (narrative HTML)

**Use `fillReport` for the primary HTML report (MANDATORY).** This produces a narrative-quality document matching the skill_test quality bar. The LLM fills a struct; the helper renders fixed-structure HTML.

```matlab
rpt.title = '<Model Name> (<identifier>)';
rpt.source = '<Author> (<year>), <Title>, <Publication>';
rpt.introduction = 'One-paragraph introduction describing what the model covers.';

% Subsystem descriptions (equations + parameters + notes)
rpt.subsystems(1).name = '<SubsystemName>';
rpt.subsystems(1).eq_ref = 'Eq X';
rpt.subsystems(1).equations = {'<HTML-formatted equation>'};
rpt.subsystems(1).parameters = struct('name', {'<p1>','<p2>'}, 'value', {'<v1>','<v2>'}, 'source', {'<src>','<src>'});
rpt.subsystems(1).notes = '<engineering context>';

% Signal flow and interconnections
rpt.signal_flow = '<Subsys1> &rarr; <Subsys2> &rarr; <Subsys3> &rarr; <Subsys1>';
rpt.interconnections = struct('source',{'<Subsys1>'},'signal',{'<signal_name>'},'destination',{'<Subsys2>'},'notes',{'<unit>'});

% Simulation setup
rpt.sim_setup.solver = '<solver> (variable/fixed step)';
rpt.sim_setup.duration = '<N> seconds';
rpt.sim_setup.initial_conditions = {'<state1>=<value> <unit>', '<state2>=<value> <unit>'};
rpt.sim_setup.notes = 'All parameters loaded via InitFcn';

% Tests (MINIMUM 3, must include primary_scenario, must include comparison table)
rpt.tests(1).name = '<Test Name>';
rpt.tests(1).description = '<What this test checks>';
rpt.tests(1).results = {'<signal>(t_end) = <value> (expected: <ref>) -- <span class="pass">PASS</span>'};
rpt.tests(1).figure = '<test_figure>.png';
rpt.tests(1).caption = 'Figure 1: <description>.';
% ... (test 2, test 3 with comparison table for primary scenario)

rpt.tests(3).comparison = struct('metric',{'Lateral crossing'},'paper',{'~100-150s'},'simulated',{'~587s'},'match',{'Slower'});

% Ambiguities
rpt.ambiguities = {'Sign convention: psi_dot = -rpri*UoL per Cpri = [1 0; 0 -1]'};

% Model screenshot
rpt.model_figure = 'model_toplevel.png';

% Files
rpt.files = {'model.slx -- hierarchical model', 'figures/ -- validation plots'};

% === OPTIONAL ADVANCED SECTIONS (include when data available) ===

% Parameter provenance (if spec has detailed parameter tracing)
% rpt.parameter_provenance = struct('name',{'T1','T2'},'value',{'107.3','18.5'}, ...
%     'unit',{'s','s'},'formula',{'T1_prime*L/U0','T2_prime*L/U0'}, ...
%     'source_line',{'HYDROGEN.M line 42','HYDROGEN.M line 43'});

% Subsystem validation (from Phase 7b results)
% rpt.subsystem_validation = struct('subsystem',{'Steering'},'test',{'Open-loop step'}, ...
%     'expected',{'tau=6s'},'result',{'tau=6.2s'},'status',{'PASS'});

% Sensitivity analysis (from Phase 10l results)
% rpt.sensitivity = sensResults;  % struct array from sensitivityAnalysis()
% rpt.sensitivity_meta = struct('metric_name','Yo settling time','delta',0.10, ...
%     'figure','sensitivity_tornado.png');

% Validate and generate
[pass, issues] = validateReport(rpt);
assert(pass, 'Report validation failed: %s', strjoin(issues, '; '));
fillReport(outputDir, rpt);
```

**Post-generation validation (MANDATORY):**
```matlab
[pass, issues] = validateReport(rpt);
if ~pass
    fprintf('Report validation FAILED:\n');
    for i = 1:numel(issues), fprintf('  %s\n', issues{i}); end
    % Fix issues before proceeding
end
```

### Step 2-ALT: Legacy `generateReport` path (PDF only)

**If PDF output is needed**, use `generateReport` in addition to `fillReport`:

```matlab
generateReport(outputDir, rpt);
```

The `rpt` struct used by `generateReport` is the same one used by `fillReport`. No separate struct builder is needed — construct it directly.

**Report content structure (MANDATORY sections in order):**

| Section | Report heading | Content |
|---------|---------------|---------|
| **Source Citation** | **Source** | **Paper title, authors, journal, year. MANDATORY for document-mode builds.** |
| Abstract | Abstract | 3-5 sentences: what model, states, subsystems, validation |
| Model Structure | 1. Model Structure | Table (subsystem, function, equations) + signal flow table + **model screenshot** |
| Equations | 2. Governing Equations | **Subsystem sub-headers** with numbered LaTeX equations |
| Parameters | 3. Parameters | Table with name, value, unit, description |
| **Comparison with Source** | **4. Comparison with Source** | **Component mapping table**, command generator/controller deviations, **simulation comparison table** (expected vs simulated for each metric) |
| Validation | 5. Validation Results | **5.1 Per-subsystem (open-loop)** + **5.2 System (closed-loop)** tables |
| Figures | 6. Simulation Results | One figure per domain. **Every caption must cite the paper figure** |
| Overall | 7. Overall Result | Component-level PASS/CLOSE/FAIL table + one-line summary |
| **Ambiguities** | **8. Ambiguities and Decisions** | Numbered list of all non-obvious choices made during the build |
| Files | 9. Files | Table of output files |

**Section 4 (Comparison with Source) is MANDATORY for document-mode builds.** It must include:
- A table mapping every thesis/paper component to the corresponding model block
- Explanation of any deviations from the source (different controller, simplified logic, etc.)
- A side-by-side simulation comparison table (metric | paper value | model value | status)

**Section 1 must include the top-level model screenshot and screenshots of each subsystem.**

**Source citation fields (MANDATORY for document-mode builds):**
```matlab
rpt.paper_title = 'Full paper/thesis title';
rpt.paper_authors = 'Author names';
rpt.paper_journal = 'Journal/conference name, volume, issue, year';
```
These appear at the top of the report as a citation block. The abstract must also reference the paper title and authors. If the source is not a paper (derivation mode), set `rpt.paper_title = 'First-principles derivation'` and omit authors/journal.

**System illustration (`rpt.photo`) — MANDATORY for all builds:**
`rpt.photo` places a contextual image at the very top of the report (after Abstract), giving the reader immediate visual context of the physical system. **Always set this field.**
- **Document-mode:** Use a system photo or diagram extracted from the paper.
- **Derivation-mode:** Generate a cartoon illustration using MATLAB figure commands (`patch`, `rectangle`, `line`, `text`). Simple shapes — no precision needed, just "what is this system?". Export with `exportgraphics` or `saveas`.

```matlab
% Document mode:
rpt.photo = struct('path', fullfile(figDir, 'system_photo.png'), 'caption', '<Physical system name>');
% Derivation mode (MATLAB figure):
rpt.photo = struct('path', fullfile(figDir, 'system_illustration.png'), 'caption', '<System description>');
```

**Technical free-body diagram (`rpt.schematic`) — in Section 2 (Governing Equations):**
The annotated diagram showing force vectors, variable labels, coordinate frames, and dimensions. Tool selection depends on environment:

```matlab
% Check if LaTeX is available for TikZ free-body diagrams
[texStatus, ~] = system('pdflatex --version');
hasLatex = (texStatus == 0);
```

- **If LaTeX available:** Use TikZ (precise vector arrows, math labels like `$F_x$`, angle arcs, dimension lines — TikZ's sweet spot). Compile `.tex` → PDF → PNG via pdftoppm.
- **If LaTeX NOT available:** Fall back to MATLAB figure commands (`quiver` for forces, `annotation('textarrow',...)`, `text` with LaTeX interpreter for labels). Less precise but no external dependency.

These are two different images serving different purposes:
- `rpt.photo`: "What is this system?" (contextual, cartoon-like) — always MATLAB
- `rpt.schematic`: "What are the variables and forces?" (technical, annotated) — TikZ if available, MATLAB fallback

If `rpt.photo` is not set, the report will have no first image — this is the most common visual gap in generated reports.

**Critical field type contracts (MUST follow exactly — mismatched types crash `fillReport`):**

| Field | Expected Type | Example | WRONG |
|-------|--------------|---------|-------|
| `rpt.overview_image` | **struct** with `.file` (string) + `.caption` (string) | `struct('file','cartoon_system.png','caption','Wind turbine')` | `'cartoon_system.png'` (plain string crashes) |
| `rpt.model_figure` | **plain string** (filename only) | `'model_toplevel.png'` | `struct('file','model_toplevel.png',...)` (struct crashes) |
| `rpt.tests(i).results` | **cell array of strings** (HTML allowed) | `{'wr = 188.5 rad/s -- <span class="pass">PASS</span>'}` | `struct('status','PASS',...)` (struct crashes) |
| `rpt.tests(i).figure` | **plain string** (filename or empty) | `'fig16_reproduction.png'` | `struct(...)` |
| `rpt.tests(i).comparison` | **struct array** with `.metric`, `.paper`, `.simulated`, `.match` | `struct('metric',{'wr'},'paper',{'188.5'},'simulated',{'188.50'},'match',{'PASS'})` | cell array |
| `rpt.ambiguities` | **cell array of strings** | `{'Sign convention assumed positive clockwise'}` | struct or plain string |

**Key `rpt` struct field guidance:**
- `rpt.model_name` -- Simulink model name. **Always set this** (`rpt.model_name = mdl;`). If `rpt.model_figure` is missing or the file doesn't exist, `generateReport` will automatically capture a screenshot of the top-level model and place it in Section 1.
- `rpt.model_figure` -- (optional) path to top-level model screenshot PNG. Auto-filled from `rpt.model_name` if not provided. Placed in **Section 1 (Model Structure)**, NOT in `rpt.figures`. Do NOT put `fig_model.png` in `rpt.figures` — that section is only for simulation output plots.
- `rpt.equations_latex` -- cell array, one per subsystem group, with `\subsection{}` headers. **Each equation must be on its own line in a numbered `align` environment.** Use `\\` between equations. NEVER concatenate multiple equations on one line — this is the #1 report formatting bug.
  ```matlab
  rpt.equations_latex = {
      ['\subsection{<Subsystem Name>}' newline ...
       '\begin{align}' newline ...
       '\dot{x}_1 &= f_1(x_1, x_2, u) \\' newline ...
       '\dot{x}_2 &= f_2(x_1, x_2, u)' newline ...
       '\end{align}']
  };
  ```
- `rpt.equations_html` -- same structure in HTML
- `rpt.parameters` -- struct array. **Units must be proper LaTeX:** use `\\Omega` not `Omega`, `\\text{mH}` not `mH`, `\\text{kg}\\cdot\\text{m}^2` not `kg*m^2`. Use display values as the user would write them (e.g., `0.8` mH not `8.0000e-04` H).
- `rpt.figures` -- struct array with `.path`, `.caption`, `.label`. Captions must reference paper figures. **Include ALL simulation figures** (minimum 4 for a 5-state model).
- `rpt.tests` -- struct array from `evaluateTests`. **Group by scenario with separate section headers.** Include paper values. Must have at least 3 test scenarios.
- `rpt.interconnections` -- populate from `spec.physical_subsystems`. Renders a Source→Signal→Destination table in Section 1.
- `rpt.ambiguities` -- populate from `spec.ambiguities`. Cell array of strings. Each entry documents one non-obvious decision made during the build.
- `rpt.subsystem_validation` -- set from Phase 7b results. Struct array with `.subsystem`, `.test`, `.signal`, `.expected`, `.actual`, `.status`. Renders in Section 5.1 (before closed-loop validation).

**Timing fields (auto-populated by `executePlan`):**
- `rpt.start_time` -- session start timestamp (auto-set by executePlan)
- `rpt.end_time` -- session end timestamp (auto-set by fillReport if missing)
- `rpt.duration` -- human-readable duration string (e.g., "3m 42s")
- `rpt.build_time` -- build-only time if tracked separately

**Set `rpt.subsystem_validation` from Phase 7b results:**
```matlab
% If Phase 7b was run, capture results for the report
if exist('svResults', 'var') && ~isempty(svResults)
    for i = 1:numel(svResults)
        for c = 1:numel(svResults(i).checks)
            k = numel(rpt.subsystem_validation) + 1;
            rpt.subsystem_validation(k).subsystem = svResults(i).subsystem;
            rpt.subsystem_validation(k).test = svResults(i).test;
            rpt.subsystem_validation(k).signal = svResults(i).checks(c).signal;
            rpt.subsystem_validation(k).expected = num2str(svResults(i).checks(c).expected);
            rpt.subsystem_validation(k).actual = num2str(svResults(i).checks(c).actual);
            rpt.subsystem_validation(k).status = char(string(svResults(i).checks(c).pass == true, 'pass', 'fail'));
        end
    end
end
```

### Report quality standard

The report must meet this quality bar:

**Simulation settings — driven by `spec.validation_figures`:**
- `StopTime` = longest `t_range` from `spec.validation_figures` (not a hard-coded default)
- Disturbance timing = from `spec.simulation_setup.load_step_time` (extracted from paper in Phase 2-3)
- Units in plots = from `spec.validation_figures[i].units` (match the paper's axes exactly)
- If the model uses internal quantities that differ from the paper's plot units (e.g., electrical vs mechanical speed), apply the conversion noted in `spec.validation_figures[i].notes`

**Figures to generate — one per `spec.validation_figures` entry:**
- Each figure reproduces one paper figure as closely as possible
- Use the `layout` field: `"stacked"` = subplots sharing time axis, `"overlay"` = same axes with legend
- Use `y_ranges` for axis limits (approximate — match paper's visual scale)
- Use `t_range` for x-axis limits
- Each caption MUST cite the paper figure (e.g., "cf. Paper Fig 16")
- **Additionally**, generate per-subsystem figures from `autoPlotValidation` for ALL logged signals (comprehensive coverage beyond just the paper's figures)

**Each figure** should be full-size (not subplot grids), with proper axis labels, units, and legends.

**Validation tables:**
- Separate table per test scenario
- Columns: Check | Expected | Simulated | Status
- Expected values reference specific paper figures or tables

**Report structure (strict ordering):**
1. Model Structure (table + screenshot)
2. Governing Equations (LaTeX, numbered, grouped by subsystem)
3. Parameters (table with symbol, value, unit, description)
4. Validation Results (per-scenario tables)
5. Simulation Results (paper-figure reproductions FIRST, then all-signal plots)
6. Overall Result (component pass/fail + summary)
7. Files

**Output format:** Generate both HTML and LaTeX. Compile LaTeX to PDF using `compileLaTeX`.

### Step 3: Ask user if they want a LaTeX PDF report

**MANDATORY: After generating the HTML report and opening it, ask the user:**

> "Would you like a PDF report as well?"

**If yes (or if running in autonomous mode):** Convert the HTML report content to LaTeX and compile to PDF. The LLM reads `report.html`, emits a proper LaTeX document (`report.tex`) with `\begin{align}` equations, `\booktabs` tables, and `\includegraphics` figure references, then compiles:

```matlab
% LLM writes report.tex from rpt struct (same data as HTML)
% Then compile:
texFile = fullfile(outputDir, 'report.tex');
compileLaTeX(texFile);
% Open PDF:
winopen(fullfile(outputDir, 'report.pdf'));
```

**If no:** Skip PDF generation. Document in gate checklist: "PDF skipped (user declined)".

Also open the HTML report:
```matlab
web(fullfile(outputDir, 'report.html'), '-browser');
```

---

## 12e-anim: Animation (MANDATORY ask)

**ALWAYS ask the user:** "Would you like an animation showing the system in motion?"

If the model is a good candidate for animation, recommend it with a brief description of what the animation would show. If the model is NOT a good candidate, still ask but note it may not add value.

**Good candidates (recommend enthusiastically):**
- CPS/hybrid systems with mode transitions (visualizes switching)
- Systems with spatial motion (trajectory, bouncing, projectile, robot arm)
- Oscillatory/periodic systems where time evolution is interesting
- Vehicle/aircraft models (show path, attitude, wheel steering)
- Multi-body systems (linkages, gears, pendulums)

**Weak candidates (ask but note limited value):**
- Pure steady-state convergence (exponential decay to setpoint)
- Parameter sweeps with no spatial interpretation
- MIMO systems with many states but no physical geometry

**If user says yes:** Generate a polished animation that clearly shows the system's dynamic behavior.

**When generating an animation:**

1. **Create video frames** using MATLAB `VideoWriter` ('MPEG-4'):
```matlab
v = VideoWriter(fullfile(figDir, 'animation.mp4'), 'MPEG-4');
v.FrameRate = 25;
v.Quality = 85;
open(v);
% Loop: draw system state at each time step, writeVideo(v, getframe(fig))
close(v);
```

2. **Add sound effects** (optional, for impact/event systems):
```matlab
% Synthesize audio: short decaying tone at each event
Fs = 44100;
audio = zeros(round(duration*Fs), 1);
for each event_time:
    thud = volume * exp(-40*t) .* sin(2*pi*80*t);  % 50ms, 80Hz
    % Insert at correct sample position
end
audiowrite(fullfile(figDir, 'audio.wav'), audio, Fs);
```

3. **Merge video + audio** with ffmpeg:
```matlab
system('ffmpeg -y -i animation.mp4 -i audio.wav -c:v copy -c:a aac -shortest animation_sound.mp4');
```

4. **Embed in HTML report** (inline base64 for self-contained delivery):
```matlab
fid = fopen(videoFile, 'rb'); videoBytes = fread(fid, '*uint8'); fclose(fid);
encoder = java.util.Base64.getEncoder();
videoB64 = char(encoder.encodeToString(videoBytes));
% In HTML: <video controls autoplay loop><source src="data:video/mp4;base64,..." type="video/mp4"></video>
```

**Tips:**
- Keep video under 8 seconds (capture the interesting dynamics, cut the boring tail)
- Use visual cues at events (flash color on impact, mode-colored trail)
- Scale volume/pitch to physical quantities (harder hit = louder/higher)
- 25 fps is sufficient; avoid 60fps (memory issues with VideoWriter)
- Keep file size reasonable (<1 MB for the MP4) to avoid bloating the HTML report

---

## 12f: Self-Review the Report (MANDATORY)

**After generating the report, review it yourself before declaring complete.** Read the HTML report and check for these common issues:

```matlab
% Read the generated HTML report
reportHtml = fileread(fullfile(outputDir, 'report.html'));
```

**Checklist (verify by reading the report content):**

1. **Table 1 (Model Structure):** Does it list all subsystems with their equations? If empty → `spec.equations_raw_latex` was missing (Phase 1-3 was skipped or incomplete)
2. **Section 2 (Governing Equations):** Are equations rendered? If blank → `rpt.equations_latex`/`rpt.equations_html` were not set
3. **Section 3 (Parameters):** Does it include ALL parameters with correct units and values?
4. **Section 4 (Comparison with Source):** [Document mode] Is the component mapping table present? Are deviations documented?
5. **Section 5 (Validation):** Are there at least 3 test scenarios? Do expected values cite paper figures?
6. **Section 6 (Simulation Results):** Are paper-figure reproductions present FIRST? Do captions cite paper figures? Are axis labels and units correct?
7. **Figures:** Do simulation plots visually match the paper's figures in shape, scale, and features?
8. **Source citation:** Is paper title, authors, and journal shown at the top?
9. **Model screenshots:** Are they current (showing final layout with all blocks)?

**If ANY issue is found:** Fix it (rebuild rpt fields, regenerate) and re-check. Do NOT deliver a report with blank sections, missing tables, or stale screenshots.

**Common fixes:**
- Table 1 empty → set `spec.equations_raw_latex` from the extracted equations, rebuild rpt
- Section 2 blank → set `rpt.equations_latex` and `rpt.equations_html`
- Section 4 missing → add `comparison` struct to `rpt` before calling `fillReport`
- Stale screenshots → re-run `saveModelScreenshots` after final `layoutSignalFlow`

---

## Gate: Phase 12 Complete (Final Deliverable Check) -- MANDATORY

```
Final Deliverable Check:
- [ ] Final model .slx saved to output/models/
      (hierarchical for full pipeline, flat for simple path, CPS for hybrid)
- [ ] [Full only] Flat model backup .slx also saved to output/models/
- [ ] Model opens and simulates with just Play button (close + reload + sim test)
- [ ] NO To Workspace blocks in final model (signal logging only via `addSignalLogging`)
- [ ] **[Full only] `layoutSignalFlow` called after final hierarchy/wiring change (BLOCKING)**
      If model has subsystems and layoutSignalFlow was NOT called, STOP and call it now.
      `arrangeSystem` alone is NOT acceptable for top-level layout.
- [ ] report.html exists and opened in browser
- [ ] User asked "Would you like a PDF report?" — if yes, report.tex + report.pdf generated
- [ ] Model screenshots are current (taken AFTER layoutSignalFlow, not before)
- [ ] Report generated via `fillReport` (primary) + `validateReport` passed
- [ ] [If user said yes to PDF] PDF opened and verified
- [ ] Standalone `run_validation.m` generated via `writeValidationScript`
- [ ] Plots include paper figure reproductions (Category 1) + signal coverage (Category 2)
- [ ] [If reference_data exists] Overlay plots generated via `plotWithReference`
- [ ] [Full only] Equation annotations added to subsystems (12a)
- [ ] [Document mode] Source comparison section included in report (Section 4)
- [ ] **Self-review (12f) completed — no blank sections, no missing tables**
- [ ] User asked "Would you like an animation showing the system in motion?"
- [ ] [If user said yes] Animation generated and embedded in report (12e-anim)
- [ ] Inform user of all output file locations
```
