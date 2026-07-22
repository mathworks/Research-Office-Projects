# FRET to Simulink Translator

Translates [NASA FRET](https://github.com/NASA-SW-VnV/fret) temporal-logic requirements into Simulink&reg; verification artifacts — Requirements Table blocks for formal analysis with Simulink&reg; Design Verifier&trade;, and Test Assessment blocks for simulation-based runtime verification with Simulink&reg; Test&trade;.

Unlike [CoCoSim](https://github.com/NASA-SW-VnV/CoCoSim), which generates Verification Subsystem blocks with Proof Objectives for SLDV property proving, this pipeline targets Simulink's newer native artifacts — Requirements Table blocks (R2022a+) and Test Assessment blocks — enabling SLDV completeness/consistency analysis (Design Error Detection) and Simulink Test runtime verification workflows.

## Requirements

| Dependency | Required |
|---|---|
| MATLAB R2023a or later | Yes |
| Simulink | Yes |
| Requirements Toolbox&trade; | Yes |
| Simulink Test | Yes (for Test Assessment blocks) |
| Simulink Design Verifier | Optional (for formal analysis of RT blocks) |

## Quick Start

```matlab
addpath("path/to/fret-to-simulink/helpers")

% Single-component project → one model with one RT block
rtBlk = fretJsonToRT("fsm_reqts_and_vars.json")

% Multi-component project → one model per component (default)
rtBlks = fretJsonToRT("LM_requirements.json", "LMCPS_RT")

% Multi-component project → merged into a single model
rtBlk = fretJsonToRT("LM_requirements.json", "LMCPS_RT", PerComponent=false)

% Test Assessment block
taBlk = fretJsonToTA("fsm_reqts_and_vars.json")
```

When the FRET JSON contains multiple `component_name` values (common with multi-model benchmarks like LMCPS), `fretJsonToRT` creates a separate Simulink model per component by default. This avoids cross-component symbol conflicts and ensures each model can be independently analyzed with SLDV. Set `PerComponent=false` to merge all requirements into one model.

The pipeline loads the FRET JSON, converts each requirement through `fretToSpec`, routes to RT (invariant patterns) or TA (temporal patterns) based on renderability, creates the Simulink model(s), and reports conversion statistics.

For step-by-step examples, see **[GETTING-STARTED.md](GETTING-STARTED.md)**.

AI coding agents with access to the [Simulink Agentic Toolkit](https://github.com/matlab/simulink-agentic-toolkit) MCP server can call the helpers directly via `evaluate_matlab_code`.

## How It Works

```
FRET JSON --> fretToSpec() --> reqCreateSpec struct --> reqCheckRenderability()
                                                       |-- RT-renderable --> reqRenderToRT() --> RT block
                                                       '-- TA-renderable --> reqRenderToTA() --> TA block
```

The `fretToSpec` adapter handles:
- SMV-to-MATLAB syntax conversion (`!` to `~`, `&` to `&&`, `=` to `==`)
- Bi-implication expansion (`<=>` to conjunction of implications)
- Implication splitting (`guard -> prop` to precondition/postcondition)
- Constant substitution from FRET variable mapping (Internal variables)
- FRET function mapping (`absReal` to `abs`, `preBool` to `prev`, `median` expansion)
- FTP (First Time Point) sentinel handling

The TA pipeline uses **if-guard semantics**: guarded requirements emit `if guard; verify(response); end` so that unexercised requirements show UNTESTED (not vacuous PASS), matching the structured assessment editor behavior.

## Supported FRET Templates

| Template Key | Pattern | RT | TA | Notes |
|---|---|---|---|---|
| `null,null,always` | Unconditional invariant | Y | Y | Implication splitting |
| `in,null,always` | Scoped invariant | Y | Y | scope_mode as guard |
| `null,regular,always` | Persistent obligation (edge) | - | Y | Temporal persistence |
| `null,holding,always` | Persistent obligation (level) | - | Y | |
| `null,regular,immediately` | Edge-triggered immediate | Y | Y | RT uses `prev()` pattern |
| `in,null,immediately` | Scope entry immediate | Y | Y | |
| `in,regular,immediately` | Scoped edge-triggered | Y | Y | |
| `null,holding,immediately` | Level-triggered immediate | Y | Y | |
| `null,regular,next` | Next-step response | - | Y | P at t+1 |
| `null,regular,within` | Bounded response | - | Y | |
| `null,null,within` | Unconditional bounded | - | Y | |
| `null,null,for` | Duration constraint | - | Y | |
| `null,regular,until` | Until response | - | Y | |
| `null,null,eventually` | Eventual satisfaction | - | Y | |
| `null,null,never` | Negated invariant | Y | Y | |

## API

| Function | Description |
|---|---|
| `fretJsonToRT(jsonFile, modelName, NV)` | One-call pipeline: FRET JSON to Requirements Table block(s) |
| `fretJsonToTA(jsonFile, modelName, NV)` | One-call pipeline: FRET JSON to Test Assessment block |
| `fretToSpec(id, nl, semantics, vars)` | Convert FRET semantics to reqCreateSpec struct |
| `reqCreateSpec(id, nl, pattern, NV)` | Create a requirement specification struct |
| `reqRenderToRT(spec)` | Render a spec as RT precondition/postcondition |
| `reqRenderToTA(spec)` | Render a spec as TA trigger/response configuration |
| `reqCheckRenderability(spec)` | Check which targets (RT, TA) a spec supports |
| `reqRenderStructuredEnglish(spec)` | Render a spec as human-readable Structured English |

### `fretJsonToRT` Name-Value Options

| Option | Default | Description |
|---|---|---|
| `ReqFilter` | `{}` | Cell array of reqids to include (empty = all) |
| `Tolerance` | `0` | Numeric tolerance for double equality |
| `SLDVReady` | `true` | Configure model for SLDV (fixed-step discrete solver) |
| `PerComponent` | `true` | Create one model per FRET component |

## Validation

Validated on case studies from the [NASA FRET repository](https://github.com/NASA-SW-VnV/fret/tree/master/caseStudies) and the [LMCPS benchmark](https://github.com/hbourbouh/lm_challenges):

| Case Study | Total | RT Rendered | RT Compile | TA Rendered | TA Compile |
|---|---|---|---|---|---|
| FSM (Finite State Machine) | 13 | 11 | 1/1 | 13 | 1/1 |
| Liquid Mixer | 12 | 9 | 1/1 | 12 | 1/1 |
| LMCPS (all 10 challenges) | 97 | 71 | 13/13 | 74 | 13/13 |
| **Total** | **122** | **91** | **15/15** | **99** | **15/15** |

Skipped requirements use features not currently expressible: external function calls (`mag`, `dot`, `det_3x3`), `prev()` with complex expressions, or `persisted()` temporal operators. Each LMCPS component produces a separate model; all compile and pass validation (Update Diagram). See [SUPPORTED-PATTERNS.md](SUPPORTED-PATTERNS.md) for the full pattern coverage matrix.

## References

1. [NASA FRET](https://github.com/NASA-SW-VnV/fret) — Formal Requirements Elicitation Tool
2. [LMCPS Benchmark](https://github.com/hbourbouh/lm_challenges) — Lockheed Martin Cyber-Physical Systems challenges
3. [Simulink Agentic Toolkit](https://github.com/matlab/simulink-agentic-toolkit) — MCP server, tools, and skills for AI coding agents working with MATLAB and Simulink
4. C. Menghi, E. Balai, D. Valovcin, C. Sticksel, A. Rajhans, "Completeness and Consistency of Tabular Requirements: an SMT-Based Verification Approach," *IEEE Transactions on Software Engineering*, vol. 51, no. 2, Feb. 2025. [[IEEE](https://ieeexplore.ieee.org/document/10844918)]
5. A. Rajhans, A. Mavrommati, P. J. Mosterman, and R. G. Valenti, "Specification and Runtime Verification of Temporal Assessments in Simulink," *21st International Conference on Runtime Verification (RV)*, 2021. [[PDF](https://www.mathworks.com/content/dam/mathworks/conference-or-academic-paper/specification-and-runtime-verification-of-temporal-assessments-in-simulink.pdf)]
