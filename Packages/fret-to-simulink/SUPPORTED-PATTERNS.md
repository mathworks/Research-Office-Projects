# Supported FRET Patterns

This document describes which FRET requirement patterns translate to Simulink&reg; Requirements Table (RT) and Test Assessment (TA) blocks, and which are currently unsupported.

## Summary

| Benchmark | Total Reqs | RT Rendered | TA Rendered |
|-----------|-----------|-------------|-------------|
| LMCPS     | 97        | 71 (73%)    | 74 (76%)    |
| FSM       | 13        | 11 (85%)    | 13 (100%)   |
| Liquid Mixer | 12     | 9 (75%)     | 12 (100%)   |

## Supported Patterns

| Pattern | RT | TA | Notes |
|---------|----|----|-------|
| Invariant (always P) | Yes | Yes | Maps to unconditional postcondition / verify() |
| Guarded invariant (if G then P) | Yes | Yes | Precondition + postcondition / if-guard + verify() |
| Trigger-response (when T then P) | No | Yes | TA uses step transitions; RT lacks trigger semantics |
| Edge trigger (when T becomes true) | No | Yes | TA uses hasChangedTo() or manual edge detection |
| Duration trigger (T holds for N sec) | No | Yes | TA uses duration() >= N |
| Response with delay (within N sec) | No | Yes | TA uses after(N, sec) return transitions |
| Weak-until response (P until Q) | No | Yes | TA models via return transition on Q |
| Hold-at-least response (P for >= N) | No | Yes | TA uses after(N, sec) for minimum hold |
| prev(symbol) | Yes | Yes | RT uses prev() natively; TA uses Local variable pattern |
| Boolean operators (and/or/not) | Yes | Yes | Mapped to &/\|/~ |
| Arithmetic comparisons | Yes | Yes | Direct translation |
| Implication (A => B) | Yes | Yes | Expanded to ~A \| B |
| Tolerance equality (abs(x-y) < tol) | Yes | Yes | Optional via Tolerance parameter |

## Unsupported Patterns

| Pattern | RT | TA | Reason |
|---------|----|----|--------|
| persisted(N, expr) | Skip | Skip | FRET temporal operator with no direct Simulink equivalent. Would require counter-based state machine logic. |
| prev(complex_expr) | Skip | Skip | Both RT and TA only support prev() on a single symbol name, not expressions like prev(A + B) or nested prev(prev(x)). |
| External function calls (det_3x3, mag) | Skip | Skip | Requires extrinsic MATLAB function definitions that are not part of the FRET export. |
| Massive nested pre() chains | Skip | Skip | Caught by complex-prev filter. Encodings like neural network weight tables are not expressible in RT/TA. |

## TA-Specific Implementation Details

### prev() Handling

TA Input-scoped symbols do not support the `prev()` operator. The pipeline creates Local variables (`prev_<name>`) with `DataType=double` and `InitialValue=0`, rewrites `prev(x)` to `prev_x` in all expressions, and appends `prev_x = x;` to every step action.

### Vector Signals

Signal dimensions are inferred from indexing patterns (e.g., `x(3)` implies size >= 3) and propagated through multiplications. All Input symbols receive explicit `Size` values since TA cannot infer dimensions from unconnected inports. Vector-vector multiplications are rewritten as dot products (`A' * B`) to produce scalar verify() expressions.

### Edge Detection

Simple triggers on bare identifiers use `hasChangedTo(signal, true)`. Compound trigger expressions (containing operators or arithmetic) use a manual edge-detection pattern with `cur_trig_N` and `prev_trig_N` Local variables.

## RT-Specific Implementation Details

### Design Outputs

RT requires at least one symbol marked as a Design Output. The pipeline infers outputs from postcondition structure: symbols that appear in postconditions but not directly in preconditions (outside prev()) are candidates.

### prev() Handling

RT supports `prev()` natively on Input symbols. Symbols used in prev() receive `InitialValue = '0'` to satisfy the RT block requirement for initial conditions.

### Vector Signals

Same inference as TA: indexing patterns determine size, multiplication propagates dimensions. The RT API uses `sym.Size = 'N'` directly. Vector-vector multiplications are rewritten as `A' * B`.
