# The MBD Cheat Sheet for AI Agents 🧠🛠️

*House rules that keep your AI agent from tuning a controller against a plant nobody validated.*

A single file you drop into a MATLAB&reg;/Simulink&reg; project so that an AI coding agent works the way a careful
Model-Based Design team does. It writes requirements first, validates the plant against data before designing
anything, derives every number, verifies against requirement IDs, and **stops to ask you** at every decision
that is yours to make.

**It works on top of the Simulink Agentic Toolkit.** The toolkit gives the agent the tools and the know-how for
each task: building, simulating, testing, specifying. This cheat sheet gives it the playbook: the order of the
work, the checkpoints, what is yours to decide, and how to keep a long project coherent. It does not repeat the
toolkit; it tells the agent which toolkit skill to load at each step.

> **Top section: for you.** **Everything below the line: for your agent.**

## Quick start (2 minutes)

Board invite. Do you **You need**
- An AI coding agent that supports MCP tools and skills.
- MATLAB and Simulink with the [Simulink Agentic Toolkit](https://github.com/matlab/simulink-agentic-toolkit)
  installed. It includes the MATLAB Agentic Toolkit and the MATLAB MCP Server.

**Then**
1. **Save this file in your project's root folder**, named after the instruction file your agent reads at
   startup (for example `AGENTS.md`).
2. **Start the agent there and describe the job in a sentence or two, including what kind of deliverable it is:**
   > *"Model a hydraulic cylinder in Simscape with a position controller, validated against the datasheet step
   > response. It's a teaching example, not a product."*
3. **Answer the gate questions.** The agent's first reply names the tier, the milestones and the skills it
   will use. After that it builds one stage at a time and stops at each gate with results, failures and
   assumptions, plus the questions only you can answer.

## How it works

```mermaid
flowchart LR
  M0["M0 Requirements"] --> A{{"Gate A<br/>targets, tolerances"}}
  A --> M1["M1 Interfaces"] --> B{{"Gate B<br/>freeze"}}
  B --> M2["M2 Validated plant"] --> C{{"Gate C<br/>matches data?"}}
  C --> M3["M3 Design"] --> D{{"Gate D<br/>tradeoffs"}}
  D --> M4["M4 Verification"] --> E{{"Gate E<br/>accept / iterate"}}
  E --> M5["M5 Code (if deployed)"] --> M6["M6 Handoff"]
```

| Size of the job | What the agent runs | Where it stops |
|---|---|---|
| **A**: a question or a quick edit | just the task | hard stops only |
| **B**: one component, algorithm or study | M0, M2, M3, M4 (M1, M5 if needed) | Gates A and E, plus hard stops |
| **C**: heading for hardware, production code or certification | M0 to M6 | every gate |

**What stays with you, the engineer:** the objective, every pass/fail limit, physical and safety judgement,
tradeoffs, freezing interfaces. Outward-facing actions also stay with you: code generation, persistent tests,
publishing. These are **hard stops**, so the agent asks every time.

**Seven habits it enforces**
1. Requirements with a value, a tolerance and a *source* before any block is placed.
2. Plant before controller. Every component is validated against evidence, **predicting each result before running**.
3. Every design number traces to a written analysis. A constant with no derivation is a defect.
4. Every task goes through the matching toolkit skill, and that skill's rules apply.
5. Pass/fail per requirement ID, with what was *not* verified listed explicitly.
6. A `RESUME-HERE.md` keeps long projects coherent across sessions.
7. Plain-language questions at every gate, with the agent's own estimates and design calls named as such.

## See it in action

[**Soft pneumatic gripper picking fruit**](https://github.com/mathworks/Research-Office-Projects/tree/main/Examples/soft-gripper-fruit-picking)
is a four-finger soft gripper modelled in Simscape&trade;, validated against published hardware data and
built end to end with this guide.

![Soft gripper grasp](https://raw.githubusercontent.com/mathworks/Research-Office-Projects/main/Examples/soft-gripper-fruit-picking/images/apple_pick_close.gif)

---

# Agent instructions

**Everything from here down is written for the agent.** It is imperative: follow it stage by stage.

**This guide sets the order, the gates and the decisions. The Simulink Agentic Toolkit (SATK) skills say how to
do each task.** Before any task, load the matching skill and follow it. Where this guide and a skill overlap,
**the skill wins**. Never re-derive a toolkit rule from memory; read the skill.

| Part | Read before |
|---|---|
| [Part 1: Operating rules](#part-1-operating-rules) | anything; it is the spine |
| [Part 2: Milestones](#part-2-milestones) | starting any milestone (steps, gate questions, skills to load) |
| [Part 3: Domain profiles](#part-3-domain-profiles) | M2–M4 of controls, DSP, AI, physical, RF, robotics or analysis-only work |
| [Part 4: Conventions](#part-4-conventions) | creating project files (`rationale.md`, `RESUME-HERE.md`) |
| [Part 5: Review checklist](#part-5-review-checklist) | reporting work complete |

## Part 1: Operating rules

**Roles.** The **engineer** owns the objective, every physical, safety, and tradeoff call, and every pass/fail
criterion. The **agent** proposes, builds, runs, and reports, and says plainly when a number is its own estimate
or a design call it made.

### Rules

1. First response: name the **tier**, the **milestone(s)** in order, and the **SATK skills** you will load.
2. One stage at a time. Never design against an unvalidated context. Never generate code before M4 passes.
3. Stop at each 🛑 gate in your tier. Present the exit artifact, state what the run actually showed including
   failures, list assumptions, ask that gate's questions. Physical, safety, and tradeoff calls belong to the
   engineer.
4. If told to run end-to-end, collapse gates into one final report answering every gate's questions and flag each
   assumption made in place of a confirmation. **⛔ hard stops never collapse**, whatever the tier.
5. Every stage ends with an executed check, not an assertion. Run it; paste the result.
6. Model is source of truth. Fix the model and regenerate; never edit generated code.
7. Escalate tier out loud if the job grows. Never silently skip a stage.
8. **Keep a resume file.** Multi-session work keeps `RESUME-HERE.md` at the project root: status, decisions in
   force, next step, open items. Read it first in every session; update it after every step
   ([template](#part-4-conventions)).
9. **Pin the deliverable type at M0**: product, study, or teaching example. It sets the tier, what "done" means,
   and whether a weak result sinks the project. Re-confirm if the scope moves.
10. **Ask in plain words.** When the engineer must decide, give the options in full sentences with the
    consequence of each and a recommendation. Shorthand and IDs belong in the tracking files.
11. **Disclose your own calls.** Any design choice, estimate, or deviation from an approved plan the agent made is
    named as such in the gate report, with the reason.
12. **Show progress early.** A runnable simulation and a plot beat a finished document; keep paperwork as lean
    as the tier allows.

### Tier

| Tier | Scope | Stages | Gates |
|---|---|---|---|
| **A** | Bounded question or edit; one script; "what does this model do" | none formally | ⛔ only |
| **B** | One component, algorithm, or study | M0, M2, M3, M4 (M1 if decomposition non-obvious; M5 only if something deploys) | A, E, ⛔ |
| **C** | Subsystem heading for hardware, production code, or certification | M0–M6 | all |

### Milestones at a glance

Steps, skills and gate questions: [Part 2](#part-2-milestones).

| | Milestone | Exit artifact | Gate |
|---|---|---|---|
| **M0** | Requirements and scenarios, all draft | requirement set with value, tolerance, source; scenarios with pass criteria | 🛑 A |
| **M1** | Decomposition and interfaces | hierarchy passing `model_read` + `model_check`; interface and parameter tables; interfaces frozen | 🛑 B |
| **M2** | Context (plant) model, **validated** | predicted-vs-simulated comparisons; limits of validity | 🛑 C |
| **M3** | Design | `rationale.md` (row per parameter); achieved vs required; ≥2 candidates where a tradeoff exists | 🛑 D |
| **M4** | Verification | pass/fail per requirement ID; unverified requirements listed | 🛑 E |
| **M5** | Implementation (skip if nothing deploys; say so) | equivalence and timing/resource results | 🛑 F |
| **M6** | Integration and handoff | re-run system tests; blast radius of any requirement change; handoff set | — |

**Parameter derivation rule (mandatory).** Every design number follows *analyze → extract quantity → convert to
constraint → choose value → record in `rationale.md`*. **A constant with no such chain is a defect.**

### Which skill, for which task

Load the skill before acting; its own guardrails, review gates and approval steps apply. Names follow the public
toolkit catalogue. **If a name is not found, the installed release may name it differently: pick the skill whose
description matches the task.**

| Task | Skill |
|---|---|
| Any structural model change (Simulink, Simscape, Stateflow, System Composer) | `building-simulink-models` |
| Running simulations, sweeps, inputs | `simulating-simulink-models`, `authoring-simulink-inputs` |
| Logged data, file imports, run comparison | `create-sdi-run` |
| Requirements | `generate-requirement-drafts` |
| Plant / context specs | `specifying-plant-models` |
| Controller, estimator, DSP or logic specs | `specifying-mbd-algorithms` |
| Architecture layers (Tier C) | `building-architecture-models` |
| Projects, paths, dependencies | `managing-simulink-projects` |
| Tests (quick Gherkin or persistent Simulink Test) | `testing-simulink-models` |
| Guidelines and compliance | `checking-model-compliance` |
| Design-error detection | `resolve-design-errors` |
| Coverage gaps | `simulink-explain-missing-coverage` |
| Fault injection, safety analysis | `inject-faults`, `manage-safety-analysis` |
| Linearization, frequency response | `simulink-linearize`, `simulink-frequency-response` |
| Code generation | `simulink-configure-model-for-code-generation`, `simulink-optimizing-embedded-code` |
| Finding a shipped example to start from | `finding-simulink-examples` |
| MATLAB-side work (data, fitting, tests of MATLAB code, reports) | the MATLAB Agentic Toolkit skill for it |

Call the MCP tools (`model_overview`, `model_read`, `model_edit`, `model_check`, `model_test`,
`model_query_params`, `model_resolve_params`) directly, never through `evaluate_matlab_code`.

### ⛔ Hard stops: ask, never assume

**The skills' own approval steps are hard stops here too and never collapse**: persistent test creation and
configuration, applying compliance or design-error fixes, writing compliance justifications, clearing SDI,
generating or building code, saving after code-generation configuration. In addition:

| Action | Rule |
|---|---|
| Deleting or overwriting a model file | Confirm first |
| Changing a frozen interface | Confirm first |
| Spec / architecture phase transitions | Engineer review before the next phase |
| On-target / HIL / physical runs | Confirm, plus safe limits and an abort path |
| Publishing: push, PR, release upload, sharing outside the machine | Explicit instruction, every time; package locally first |
| Any simulation or test | `StopTime` finite, checked before the run. An infinite run hangs the session |

**Also never:** change the MATLAB working folder while a model is open · `clear all`, `clearvars`, `bdclose all`
or `restoredefaultpath` in the user's session · claim a model *is* compliant (report pass/fail) · guess a
pass/fail criterion · model dynamics you cannot validate against available evidence.

### Long simulations

Physical models can take minutes to hours per run. Predict the wall time before launching (a short probe run,
scaled), state it, and set a wall-clock limit. Write check scripts so they are **resumable**: one
`results/<test>_<case>.mat` per case, skip cases already on disk, and an evaluate-only mode that re-scores saved
runs without simulating. A run that stalls is a finding: read the solver step size before changing tolerances.

---

## Part 2: Milestones

One stage at a time; each ends with an executed check. Stop at each 🛑 gate in your tier.

### M0 — Requirements

**Skill:** `generate-requirement-drafts` (format, IDs, EARS wording, draft marking).

1. Write the objective in one sentence, specific enough to imply what matters and what may be ignored ("size
   the motor so the arm reaches 90° in <0.5 s under 20 A", not "model the drive").
2. Every requirement carries a value, a tolerance and a **source** (spec / measurement / standard / simulation /
   guess). Mark every guess.
3. Write the scenario list with pass criteria, including off-nominal, and the signals needed to evaluate each.
4. Pin the deliverable type and where it will live. If it ships to an existing repo, read that repo's
   conventions now (folder layout, entry-point script, README format, product list, size limits).
5. Assess the validation evidence early: which published or measured data can each component be checked
   against? A component with no evidence is a scope question for Gate A, not an M2 surprise.

**Done when** every requirement has an ID, a number, a tolerance and a source, every scenario has a pass
criterion, and the set is marked draft and presented for review.

🛑 **GATE A**: Targets and tolerances correct? Which guesses can become measurements? Safety/certification
standard in play (DO-178C/331, ISO 26262, ISO 25119, EN 50128/50657, IEC 61508/62304)? Existing model, dataset,
or legacy code to reuse?

### M1 — Decomposition and interfaces

**Skills:** `specifying-plant-models` (context side), `specifying-mbd-algorithms` (algorithm side),
`building-architecture-models` (Tier C), `managing-simulink-projects`, then `building-simulink-models` for the
hierarchy. The spec skills own the interface classes, the interface freeze and their review gates.

1. Split: thing being designed / context (rest of system + environment) / test harness, so code, hardware or a
   higher-fidelity context can be swapped in later.
2. Fidelity call per component, justified by the evidence found at M0. The first system model needs only correct
   units, correct channel connections and the dynamics that matter.
3. Interface conversions go at the top level with a recorded justification, never buried in a subsystem.
4. Parameters in `params_*.m` / model workspace / `.sldd`, with source and unit per row.

**Done when** `model_read` shows the intended hierarchy, `model_check` reports no errors, and every interface
row is filled in.

🛑 **GATE B**: Decomposition right? Units, ranges, signal classes correct? Which components need physical
fidelity vs a simple approximation? Target platform and rate? Which components already exist? Ready to freeze?

### M2 — Context model, validated

**Validate the context before designing against it.** Controls: plant before controller. Data-driven:
characterize the dataset before training. DSP: representative signal before filter.

**Skills:** `specifying-plant-models` (validation plan), `building-simulink-models`, `simulating-simulink-models`,
`create-sdi-run`, `testing-simulink-models` (component tests).

| Situation | Route |
|---|---|
| Explicit I/O math relation | Blocks / equations directly |
| Implicit physical relations | Simscape |
| Only measurements, no component data | System Identification or a data-driven surrogate |
| Functional / mode logic | Stateflow |
| Logged data *is* the context | Import, clean, characterize, document coverage gaps |

1. Simplest approximation that answers the objective; add detail iteratively, re-testing; stop at the required
   accuracy.
2. Cite a source for every parameter: value, unit, source, uncertainty, conditions.
3. Validate in three stages (subsystem open-loop → integrated open-loop → closed-loop), and **write the
   prediction before each run**. A component never exercised in isolation is not validated.
4. **Make the prediction independent.** Where you can, solve the same equations outside Simulink (a short MATLAB
   script). Agreement to numerical precision proves the build; any remaining gap to the data is the model form.
5. **Match the boundary conditions of the data.** If a bench disagrees with the data at only some points, check
   that it holds the part the way the experiment did (fixtures, contact, friction) before changing a parameter.
6. **Validate in the loop as well.** Benches can omit loads that exist in the closed loop (a moving base loads the
   actuator's own inertia; a test sequence can measure release instead of formation). When the closed loop
   contradicts a bench, fix the bench and re-calibrate on the closed loop. Record what changed.
7. Write the **limits of validity**: where the model is known wrong, over what envelope it is trusted.

**Done when** every component's predicted vs simulated comparison is recorded, all three stages pass, parameters
are sourced and the limits of validity are written down.

🛑 **GATE C**: Do the responses match physical expectation or measured data? Fidelity sufficient to design
against? Limits of validity acceptable for the envelope? Any parameter you can measure instead of estimate?

### M3 — Design

**Skill:** `specifying-mbd-algorithms` (spec documents and the architecture review gate, item by item), plus the
domain skills in [Part 3](#part-3-domain-profiles).

1. Characterize the context at the operating conditions that matter (dynamics, spectra, statistics, extremes).
2. Define quantitative objectives: tracking, bandwidth, disturbance rejection, detection thresholds, latency.
3. Choose the architecture from the requirements; justify it in one line.
4. **Derive** rates, resolutions and word lengths from that characterization, and state the rule used. Never a
   round number chosen for looks. Worked example: top speed 0.183 m/s and 0.05 m allowed error → 0.27 s →
   sample time 0.25 s.
5. Design the failure paths explicitly: saturation, initialization, invalid inputs, mode logic, faults,
   safe state. Designs fail there, not on the nominal path.
6. Evaluate ≥2 candidates when a tradeoff exists; present them with numbers; let the engineer choose.
7. Record every design number in `rationale.md` (derivation rule above).

**Done when** `rationale.md` has a row per parameter, achieved vs required is tabulated, the skill's review gate
is passed, and the engineer has approved before implementation.

🛑 **GATE D**: Architecture acceptable? Achieved performance and robustness sufficient? Which tradeoff option?
Limits and fault behavior modeled the way hardware behaves? Compatible with the target platform?

### M4 — Verification

**Skills:** `testing-simulink-models` (test approach, syntax, approvals), `checking-model-compliance`,
`resolve-design-errors`, `simulink-explain-missing-coverage`, `inject-faults`, `manage-safety-analysis`.

1. Run the full M0 scenario set: envelope corners, parameter variation and worst-case combinations,
   disturbances, noise, delays, mode transitions.
2. Run off-nominal deliberately: sensor loss, actuator stuck or saturated, supply loss, degraded inputs.
3. Test every operating mode, including startup, fault and degraded.
4. Name the stage of every run (component MIL → integrated MIL → system MIL → robustness → SIL/PIL).
5. Trace tests to requirement IDs. Measure coverage and check guidelines if the project uses them (process depth,
   [Part 4](#part-4-conventions)).
6. Make repeatable runs scriptable, not GUI clicks. Report failures with evidence; list what is not verified.
7. Never guess a pass/fail criterion; ask.

**Done when** each requirement ID has a pass/fail and unverified requirements are listed explicitly.

🛑 **GATE E**: Scenarios sufficient; which further off-nominal cases? For each failure and uncovered requirement:
fix the design, relax the requirement, or accept? Coverage and compliance adequate? Implement or iterate?

### M5 — Implementation

Skip entirely when the deliverable is a study, report or model. Say so; do not invent a deployment step.

| Target | Path |
|---|---|
| Embedded C/C++ (ERT, AUTOSAR) | code-generation skills → SIL → PIL → HIL → production hardware |
| FPGA / ASIC | HDL Coder → HDL verification → FPGA-in-the-loop |
| PLC | Simulink PLC Coder → Structured Text |
| Real-time rig | Simulink Real-Time |
| App / library | MATLAB Compiler, Compiler SDK, packaged toolbox |

1. Configure before generating, through the code-generation skill (it owns the required decisions and the
   approval steps).
2. Verify equivalence in stages (SIL → PIL → HIL), reusing the M4 suite at every stage.
3. On hardware: open-loop checks first (I/O mapping, scaling, timing), then closed loop.
4. Measure timing and resource use against the rate budget.

🛑 **GATE F**: Target, toolchain and data types match the hardware? **On-target and HIL runs move physical
equipment: confirm before running, with safe limits and an abort path.** Equivalence and timing margins
acceptable?

### M6 — Integration and handoff

- Re-run system tests after integration. On a requirement change, follow the trace links, re-run the affected
  suite and **report the blast radius**.
- Keep the model; it is the reusable asset.
- Handoff: model, tests and results, traceability, `rationale.md`, specs, generated code and its configuration.

**Packaging for publication** (examples, shared repos):
1. Find the shipped set by analysis, not memory: `matlab.codetools.requiredFilesAndProducts` on the entry script,
   every check script and every model, then search the code for data files it loads. Copy into a **separate**
   folder; never publish the working folder.
2. Leave out build caches (`slprj/`, `*.slxc`), debug images, superseded runs and internal planning documents.
3. Shrink logs to the signals the replay and plots read, and say so in the file.
4. **Clean-folder test:** from the package folder alone, run the hierarchy check, one short real simulation and
   the replay loader; delete the caches it created; record what it did not cover.
5. README: hero image first, then products, how to run (with run time), what is inside, validation against data,
   limits of validity, how to adapt. Every number comes from the verification report.
6. ⛔ Publishing is outward-facing: only on explicit instruction.

---

## Part 3: Domain profiles

Pick one; it adds to [Part 2](#part-2-milestones). The skills named here hold the method details.

### Controls
- **Context:** plant + environment. Plant before controller, always.
- **Requirements:** rise/settling time, overshoot, steady-state error, bandwidth, margins, disturbance rejection,
  actuator limits, sensor noise, sample rate, fault response.
- **M3:** linearize at the operating points that matter; check bandwidth, time constants, RHP zeros, delays,
  resonances. **Derive the sample time from the plant dynamics** (10–20× closed-loop bandwidth, or a
  discretization-error budget) and state the rule. **Report achieved margins as a deliverable.**
- **Skills:** `simulink-linearize`, `simulink-frequency-response`, `simulink-control-motors`,
  `matlab-identify-linear-system`.

### Signal processing / DSP
- **Context:** measured captures, or synthesized stimulus with the right spectrum, SNR and impairments.
- **Requirements:** passband/stopband, ripple, attenuation, group delay, SNR, THD, latency, sample rate, word
  length.
- **M3:** architecture and order from the spec; rates from the bandwidth and decimation plan; word lengths from
  dynamic range and the noise budget.
- **Skills:** `simulink-streaming-dsp` and the MATLAB signal-processing skills.

### AI / data-driven
- **Context:** the dataset and its distribution. Characterize it and document coverage gaps before training:
  this is M2.
- **Requirements:** metric and threshold, per-regime performance, latency and memory on target, drift and
  out-of-distribution robustness.
- **M4:** held-out and stratified evaluation, ablations, worst-case slices. Report where it fails, not only the
  aggregate.
- **Skills:** the MATLAB AI and statistics skills.

### Physical / multidomain
- **Context:** the assembly and its boundary conditions; rig data where it exists.
- **M3/M4:** sizing from simulation sweeps; state which effects are included and which are neglected; check
  conservation laws, behavior at extremes, solver convergence.
- **Simscape numerics seen in practice:** zero-crossing detection on saturations inside a fast loop can shrink
  the step to ~1e-6 s and stall a run; if the limits are only kinks, turning it off is safe (verify one run both
  ways). A shut gas restriction still conducts heat through its port area, and insulated chambers stay hot after
  filling: check sealed-volume tests against the gas law. Physical-signal outputs read as feedthrough: break
  loops with a state, not a Memory block.
- **Soft and compliant bodies:** a pseudo-rigid-body chain (revolute joints and springs, identified from
  published force data) runs in Simscape Multibody without FEM. State the identified envelope as a limit of
  validity.
- **Skills:** `specifying-plant-models`, `building-simulink-models`.

### RF / mixed-signal / PCB
- **Requirements:** S-parameters, gain, noise figure, linearity, match, losses, eye/BER budget, thermal limits.
- **M3/M4:** topology and dimensions from EM/circuit analysis, not catalog defaults; tolerance corners.
- **Skills:** the MATLAB RF and mixed-signal skills.

### Robotics / autonomy
- **Requirements:** tracking accuracy, planning time, clearance and safety margin, fusion accuracy, degraded
  modes.
- **M3/M4:** state what perception is assumed to provide; sweep scenarios including adversarial and
  degraded-sensor cases.
- **Skills:** the MATLAB robotics and automotive skills.

### Analysis-only study
- Run **M0 → M2 → M4**; skip M3, M5 and M6. Do not manufacture a design stage.
- Deliverable: the reasoning chain **with sensitivities**: which assumptions the conclusion depends on, and how
  far each must move to change it. Tier A or B, one gate at the end.

---

## Part 4: Conventions

File layout for specs, requirements and models, and naming rules: follow the toolkit skills that create them.
This guide adds:

```
rationale.md        # one row per design number: analysis → quantity → constraint → value → requirement
RESUME-HERE.md      # project root: where we are, what is next (template below)
params_<component>.m  or  <project>.sldd     # every parameter, with unit and source
results/            # logged runs, one file per check case (resumable)
```

**`RESUME-HERE.md` template**: short; history goes in the documents it links.

```markdown
# RESUME HERE — read this first in a new session
**Last updated:** <date>. Update after every milestone step.

> **Pick up here:** status (milestone, gate) · last result · next step · open items

## What the project is
One-sentence objective · deliverable type · tier · milestones skipped and why

## Decisions in force
| Decision | Who / when | Where recorded |

## Done so far
Per milestone: artifact, executed check, result (numbers), deviations disclosed

## Open items
What is not verified, what awaits the engineer, known limits
```

**Process depth.** Bring requirement traceability, coverage targets, compliance checking and CI only if (a) the
project already uses them, (b) the work is safety- or certification-relevant, or (c) the engineer asks. Otherwise
offer them as a follow-up.

### Sources

[Simulink Agentic Toolkit](https://github.com/matlab/simulink-agentic-toolkit) skills (authoritative; load before
acting) ·
[Model-Based Design for Embedded Control Systems](https://www.mathworks.com/campaigns/offers/model-based-design-embedded-control-systems.html) (MathWorks white paper) ·
[MBD with Simulink](https://www.mathworks.com/help/simulink/gs/model-based-design.html) ·
[How Engineering Teams Adopt MBD](https://www.mathworks.com/campaigns/offers/next/how-engineering-teams-adopt-model-based-design.html)

---

## Part 5: Review checklist

Run this before reporting work complete, **together with the review gate of every skill you used**.

- Full gate ceremony on a ten-minute task, or a skipped gate on hardware-bound work.
- Blocks placed or analysis written before the objective and requirements exist.
- Designed against an unvalidated context; dynamics modeled with no evidence to validate them.
- Components integrated without individual validation; a prediction written after the run.
- Any number without a `rationale.md` row; any rate or sample time that is a round guess.
- A task done without loading the matching skill, or a toolkit rule applied from memory.
- A skill's approval step skipped, or a ⛔ hard stop collapsed.
- Off-nominal scenarios skipped because nominal passed.
- Simulation → production hardware with no SIL/PIL/HIL.
- A repeatable test runnable only by hand.
- A deployment or design stage invented for a question that has none.
- Traceability, coverage, compliance or CI imposed on a project that uses none of it, unprompted.
- Work reported complete without an executed run and its logged output.
- Fidelity added beyond what the objective requires.
- An agent estimate, design call, or deviation from an approved plan not named as such.
- A number in a README or report that does not match the verification report.
- `RESUME-HERE.md` not updated after the step.
- A package published from the working folder, with build caches, or never run from a clean folder.
