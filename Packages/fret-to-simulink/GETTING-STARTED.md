# Getting Started

This guide shows how to try the FRET-to-Simulink translator with example requirements.

## Example 1: FSM Case Study (FRET JSON → RT + TA)

The LMCPS benchmark provides 10 Simulink&reg; models with natural-language requirements created by Lockheed Martin Skunk Works. The models and requirements are [publicly available](https://github.com/hbourbouh/lm_challenges). This example uses the Finite State Machine (Challenge 1) with 13 FRETish requirements.

### Setup

```bash
git clone https://github.com/hbourbouh/lm_challenges.git
```

### Step 1: Export from FRET

In FRET, export the FSM project using "Export with variables" to produce `fsm_reqts_and_vars.json`. This file contains both the requirements (with compiled semantics) and the variable mapping (Input/Output/Internal types with constant assignments).

### Step 2: Convert to Requirements Table

```matlab
addpath('helpers');
rtBlk = fretJsonToRT('fsm_reqts_and_vars.json', 'FSM_RT');
```

The function will report:
- Number of requirements loaded and converted
- Which requirements were skipped (e.g., unsupported templates)
- Symbols added to the RT block

### Step 3: Run SLDV Analysis

```matlab
opts = sldvoptions;
opts.Mode = 'DesignErrorDetection';
[status, files] = sldvrun('FSM_RT', opts);
```

### Step 4: Convert to Test Assessment Block

```matlab
taBlk = fretJsonToTA('fsm_reqts_and_vars.json', 'FSM_TA');
```

### Step 5: Verify Against Model

```matlab
open_system('fsm_12B');
sltest.harness.create('fsm_12B/fsm', ...
    'Name', 'FRET_TA_Harness', ...
    'SeparateAssessment', true);
```

Then populate the harness TA block with the generated assessments and run the test.

---

## Example 2: LMCPS Full Benchmark (97 Requirements, 13 Components)

The full LMCPS benchmark contains 97 requirements across 10 challenge sets and 13 FRET components. Because the JSON contains multiple `component_name` values, the pipeline creates a separate model per component by default.

### Step 1: Convert to Requirements Tables

```matlab
addpath('helpers');
rtBlks = fretJsonToRT('LM_requirements.json', 'LMCPS_RT');
```

This creates 13 models: `LMCPS_RT_Autopilot.slx`, `LMCPS_RT_Euler.slx`, `LMCPS_RT_Tustin_Integrator.slx`, etc.

### Step 2: Compile and Validate

```matlab
% Verify all models compile (Update Diagram)
models = dir('LMCPS_RT_*.slx');
for i = 1:numel(models)
    mdlName = models(i).name(1:end-4);
    load_system(mdlName);
    set_param(mdlName, 'SimulationCommand', 'update');
    close_system(mdlName, 0);
end
```

### What to expect

- 97 requirements loaded, 71 renderable to RT, 26 skipped
- Skipped reasons: external function calls, complex `prev()` expressions, `persisted()` temporal operators, persistence patterns
- 13 separate models created (one per FRET component)
- All 13 models compile successfully
- Vector signals (e.g., NLGuidance) automatically get correct dimensions
- Dot products rewritten as transpose form for scalar postconditions

---

## Example 3: LiquidMixer Case Study (Single Component)

The LiquidMixer case study has 12 requirements in a single component. Since there's only one `component_name`, the pipeline creates a single model regardless of the `PerComponent` setting.

### Convert

```matlab
rtBlk = fretJsonToRT('LM_reqts_and_vars.json', 'LiquidMixer_RT');
```

### What to expect

- 12 requirements loaded, 9 renderable to RT, 3 TA-only (`weak_until` patterns)
- Single model created: `LiquidMixer_RT.slx`
- Compiles successfully

---

## FRET JSON Format

The input JSON must have this structure (FRET's "Export with variables" format):

```json
{
  "requirements": [
    {
      "reqid": "REQ-001",
      "fulltext": "the controller shall always satisfy output >= 0",
      "semantics": {
        "scope": {"type": "null"},
        "condition": "null",
        "timing": "always",
        "post_condition": "output >= 0",
        "component_name": "controller"
      }
    }
  ],
  "variables": [
    {
      "variable_name": "output",
      "idType": "Output",
      "dataType": "double"
    },
    {
      "variable_name": "THRESHOLD",
      "idType": "Internal",
      "assignment": "10.0",
      "dataType": "double"
    }
  ]
}
```

Key points:
- `Internal` variables with `assignment` values are mechanically substituted (constant replacement)
- `Input` variables become RT Input symbols
- `Output` variables become RT Input symbols with `IsDesignOutput = true`
- The `semantics` field must contain FRET's compiled formalization output

---

## References

1. [NASA FRET](https://github.com/NASA-SW-VnV/fret) — Formal Requirements Elicitation Tool
2. [LMCPS Benchmark](https://github.com/hbourbouh/lm_challenges) — Lockheed Martin Cyber-Physical Systems challenges
3. [Simulink Agentic Toolkit](https://github.com/matlab/simulink-agentic-toolkit) — MCP server, tools, and skills for AI coding agents working with MATLAB and Simulink
