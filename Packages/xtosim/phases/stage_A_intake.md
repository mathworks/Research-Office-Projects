# Phases 1-3: Read Source, Extract, and Create Spec

**Requires:** `rules.md`
**Inputs:** Source document path (or system description for derivation mode)
**Outputs:** spec JSON file (`section_01_spec.json`), output directory structure, model_id
**Next stage:** Stage B (Read `stage_B_plan.md`)

---

## Helper Function Signatures (Phase 1-3)

| Function | Call | Key input formats |
|----------|------|-------------------|
| `validateSpec` | `[isValid, issues] = validateSpec(spec)` | `spec` needs: `.paper_id`, `.chapter_id`, `.section_id`, `.model_name`, `.model_type`, `.states` (struct array with `.name`), `.parameters` (struct array with `.name`, `.value`), `.equations_raw_latex` (cell array), `.status`. **Document-mode also needs:** `.paper_title`, `.paper_authors`, `.paper_journal` (warns if missing). |
| `validateExtraction` | `[isClean, flags] = validateExtraction(spec)` | Detects circular ODE↔algebraic dependencies (hallucination pattern). Returns `isClean=true` if no issues. `flags` is struct array with `.ode_index`, `.ode_latex`, `.found_vars`, `.message`. **MANDATORY** — must pass before Phase 5. |

---

## Phase 1: Read and Map the Source (Document Mode)

### 1a: Read the Source Document

Detect the input format from the file extension and use the best reading strategy for each:

**LaTeX (.tex)** -- Best format for equation extraction
- Read directly with the `Read` tool. Equations are already in parseable LaTeX notation.
- Also check for `\input{}` or `\include{}` references to read additional .tex files.

**Images (.png, .jpg, .jpeg, .bmp, .tiff)** -- Excellent for equations
- Read directly with the `Read` tool -- it renders images visually.
- For multi-page content provided as separate images, read all of them.

**Jupyter Notebooks (.ipynb)** -- Equations + companion code in one file
- Read directly with the `Read` tool -- it returns all cells with outputs.

**Word Documents (.docx)** -- Common for theses and internal reports
- Use MATLAB `extractFileText` to get text content:
  ```matlab
  txt = extractFileText('document.docx');
  ```

**PDF (.pdf)** -- Most common, variable quality

**First, detect available tools** (run once at pipeline start):
```bash
where pdftotext 2>/dev/null && echo "pdftotext: YES" || echo "pdftotext: NO"
where pdftoppm 2>/dev/null && echo "pdftoppm: YES" || echo "pdftoppm: NO"
```
Then use the highest available tier:

#### Tier 1: Hybrid text + targeted vision (pdftotext AND pdftoppm available)

**Best quality, fewest tool calls.** Two passes:

**Pass 1 -- Bulk text extraction** (one Bash call):
```bash
pdftotext -layout "paper.pdf" "paper.txt"
```
Read `paper.txt` with the `Read` tool. This gives you: section structure, prose, parameter tables, figure/table captions, page numbers. Equations will appear as garbled Unicode -- that's expected, don't trust them.

**From the text, identify which pages contain equations and key figures** (look for equation numbers, "Fig.", "Table", garbled math symbols). Record these page numbers.

**Pass 2 -- Targeted vision** (only equation/figure pages):
```bash
pdftoppm -png -r 200 -f <first> -l <last> "paper.pdf" "output/pdf_pages/page"
```
Then read ONLY the equation/figure page PNGs with the `Read` tool. Issue parallel reads in batches.

**Why this is better:** A 75-page thesis might have equations on 15 pages and figures on 10 pages. Tier 1 reads 75 pages of text (one tool call) + ~25 page images (3-4 parallel tool calls) instead of 75 image reads.

#### Tier 2: Vision-only (pdftoppm available, no pdftotext)

Convert ALL pages to images, read with vision (multimodal) in parallel chunks:
```bash
pdftoppm -png -r 200 "paper.pdf" "output/pdf_pages/page"
```
Then read page images in **parallel 10-page batches** -- this is a HARD RULE:
```
# MANDATORY: Issue these as PARALLEL tool calls, not sequential:
Read(file_path="output/pdf_pages/page-01.png")  # through page-10.png
Read(file_path="output/pdf_pages/page-11.png")  # through page-20.png
# etc.
```
For short papers (<=10 pages), one batch is fine.

#### Tier 3: MATLAB-only (no poppler tools installed)

Use when neither `pdftotext` nor `pdftoppm` is available.

**Option A -- User-attached PDF** (preferred if in interactive mode):
Ask the user to attach the PDF in the chat message. The agent's vision capability reads it directly.

**Option B -- MATLAB `extractFileText`:**
```matlab
txt = extractFileText('paper.pdf');
```
This extracts raw text but **loses equation structure**. Equations appear as garbled Unicode or are dropped entirely. Flag every reconstructed equation as `"inferred from garbled text"` in the spec JSON.

**Option C -- Ask user to install a PDF tool or provide alternative format:**
If `extractFileText` loses equations (which it will for math-heavy papers), ask the user:

> "PDF extraction tools (`pdftotext`/`pdftoppm`) are not installed, and MATLAB's text extraction loses equation formatting. To proceed, please either:
> 1. Install poppler-utils (`pdftotext`/`pdftoppm`) — recommended for best results
> 2. Provide the equations as LaTeX (.tex), images (.png), or plain text
> 3. Attach the PDF directly in chat so I can read it visually"

Do NOT attempt undocumented internal MATLAB PDF APIs — they are version-dependent and unreliable.

**Plain text / Markdown (.txt, .md)** -- Direct equation input
- Read directly. The LLM interprets the intent and converts to normalized form.

**Multiple files** -- Read all of them and cross-reference.

### 1a-ii: Understand the Paper (Before Extracting Anything)

**After reading the source, form a high-level mental model BEFORE classifying artifacts or extracting equations.** This is how an engineer reads — you understand the goal, then extract what matters.

Answer these questions (hold in working memory, no tool calls needed):

1. **What is this system?** — Physical domain, application, scale (e.g., "6-DOF rigid body", "3-phase electric drive", "thermal network with 4 nodes")
2. **What is the goal?** — What the author is trying to demonstrate or control (e.g., "track a reference trajectory", "regulate output under disturbance", "replicate Table 3 results")
3. **What is the plant?** — The physical dynamics being modeled (e.g., "coupled nonlinear ODEs with state-dependent coefficients")
4. **What is the controller?** — How the system is controlled (e.g., "PD controller + feedforward", "LQR with integral action", "open-loop only")
5. **What does success look like?** — What the paper shows in its results as "working" (e.g., "output converges within 5% in ~2s", "steady-state error < 1%")
6. **How is the paper organized?** — Which chapters/sections carry which information (e.g., "Sec.2 = equations, Sec.3 = architecture, Sec.4 = results, Appendix = parameters")

**Why this matters for extraction:**
- **Equation selection:** Knowing the goal tells you which equations are the "final product" vs derivation scaffolding. If the goal is autopilot simulation, the state-space matrices matter more than the strip-theory derivation that computed them.
- **Parameter relevance:** 80 variables in a code appendix, but you only need the 15 that appear in the model equations.
- **Validation targets:** Knowing "success = converge to 130 ft" tells you what to check in Phase 10 — not just "plot all states."
- **Architecture decisions:** Knowing plant vs controller separation helps Phase 11 hierarchy.
- **Sign conventions and physics:** Understanding the physical scenario catches errors early (e.g., "rudder to port should increase sway to starboard in SNAME convention").

**This comprehension step applies to ALL source types** — not just theses. Even a 6-page journal paper benefits from 30 seconds of "what is this about?" before extracting. The comprehension is especially critical for:
- Multi-chapter theses (different chapters serve different roles)
- Papers with complex controller architectures
- Papers where the math looks routine but the physical scenario has subtleties (sign conventions, nondimensionalization bases, coordinate frame choices)

### 1b: Identify ALL Artifacts in the Source

During the initial scan (text read or vision pass), classify every information-bearing section. Not just equations — every artifact that could feed the build.

| Artifact type | How to recognize | Value to the build |
|---|---|---|
| **Equations** | Numbered math expressions, derivation sections | ODE/algebraic model core (Phase 7) |
| **Parameter tables** | "Table 2: Coefficients", tabular data | InitFcn values (Phase 3) |
| **Block diagrams** | System architecture figures, labeled subsystems | Hierarchy decisions (Phase 11) |
| **Result plots** | Time-series figures, "Fig. X shows..." | Validation targets (Phase 10) |
| **Code listings** | Monospace blocks, `%`/`//`/`#` comments, assignment syntax | Parameters, algorithms, control laws |
| **Algorithm descriptions** | Pseudocode, numbered procedure steps | MATLAB Function block logic (Phase 8) |
| **Data arrays** | Lookup tables, coefficient vectors, calibration curves | From Workspace / lookup blocks |

**Produce an artifact inventory** (held in working memory, recorded in spec):

```
Artifacts found:
  [equations]      pp. 8-15     Steering ODEs, propulsion, kinematics
  [table]          p. 7         Nondimensional coefficients (Table 2.1)
  [block_diagram]  p. 18        Simulink model architecture (Fig 5)
  [result_plot]    pp. 35-41    Trajectory responses (Figs 14-23)
  [code:matlab]    pp. 45-54    HYDROGEN.M — parameter computation
  [code:matlab]    pp. 55-58    CSGEN.M — hull geometry preprocessing
  [code:c]         pp. 60-62    Controller implementation
```

**Every artifact type feeds a specific downstream phase.** The inventory ensures nothing is missed — especially code appendices, which are the #1 source of parameter values in theses.

### 1b-ii: Read block diagrams and figures

Pay special attention to:
- **Block diagrams** -- system architecture and subsystem connections
- **System figures** -- free body diagrams, circuit schematics, flow diagrams
- **Result plots** -- note axis labels, scales, curve shapes, approximate values (validation targets)

### 1b-iii: Read code listings and appendices

**Code listings appear in papers, theses, appendices, and supplementary material.** They can be ANY language (MATLAB, C, Python, Fortran, pseudocode). They serve multiple roles:

| Role | Example | How it feeds the build |
|------|---------|----------------------|
| **Parameter computation** | Script computing hydrodynamic derivatives from hull geometry | Run → use outputs as ground-truth InitFcn values |
| **Algorithm/control law** | C function for autopilot, PID implementation | Translate to MATLAB Function block (Phase 8) |
| **Data source** | Lookup table arrays, calibration vectors | From Workspace blocks or InitFcn arrays |
| **Architecture reference** | Main simulation script calling subsystem functions | Informs hierarchy and signal flow |

**Extraction rules for code:**

1. **Read all code pages** during the vision pass. Code is as important as equations — it carries computable ground truth.
2. **Identify the language** from syntax (`;` endings + `%` comments = MATLAB, `//` or `{}` = C/C++, `def`/`:` = Python, etc.)
3. **Identify the role** from context: Does it compute parameters? Implement a control law? Define data?
4. **Record in artifact inventory** with language, pages, role, and name (if given, e.g., "HYDROGEN.M").
5. **For parameter-computing code:** Extract the final computed values — these are the authoritative parameter source. They override values inferred from prose or partially-visible tables.

**Understanding code in context of the build goal (CRITICAL):**

Don't extract code mechanically — **read it with the model in mind.** The purpose of reading code is to answer: "What does this code tell me that helps build the Simulink model?"

1. **Map code variables → equation symbols.** Code uses programmer names, equations use math notation. Build the mapping explicitly:
   ```
   Code variable    Equation symbol    Role in model
   Yvtot            Y_v                b11 in state matrix A
   Kgpri            K_g'               rudder gain (controller)
   mpri             m'                 nondim mass (a11 denominator)
   ```
   Record this mapping — it prevents wrong-parameter-in-wrong-place bugs.

2. **Understand execution order.** If script A calls script B, B must run first. If a variable is computed from 20 lines of geometry (integration, interpolation), you can't just grab the formula — you need the result.

3. **Identify which outputs feed the model.** A 10-page script may compute 80 intermediate variables. The ones that matter are those that appear in the equations chapter. Cross-reference: look for the variable names from Chapter 2's equations (a11, b1, Kg, etc.) in the code's final output section.

4. **Resolve naming differences.** The code's namespace ≠ the equation namespace ≠ the Simulink block namespace. Make one authoritative mapping. Example:
   - Paper equation: `\dot{v}' = a_{11}v' + a_{12}r' + b_1\delta`
   - HYDROGEN.M: `a11 = mpri - Yvdot; ... Apri = inv(A)*B;`
   - InitFcn: `a11 = <value from code>`
   
5. **Connect dependent scripts.** If CSGEN.M computes `k1, k2, mz` and HYDROGEN.M uses these, note the dependency chain. Run in order, or extract outputs of each stage.

**Parameter priority (highest to lowest):**
1. Code output (run the code or read computed values from listing)
2. Explicit table in the paper
3. Stated in prose/captions
4. Inferred by LLM from context

**Handling code that computes parameters:**

- **MATLAB code:** Attempt to reconstruct the `.m` file from OCR/vision. Run it in MATLAB. If it executes successfully, use the workspace variables as ground-truth parameters. If it fails (missing dependencies, OCR errors), read the final assignment lines to extract values manually. In both cases, build the variable→equation mapping above.
- **Other languages (C, Fortran, Python):** LLM reads and interprets the code. Extract parameter values from assignment statements. If the code does non-trivial computation (integration, iteration), translate the logic to MATLAB and run it.
- **Pseudocode:** Interpret intent, note in spec as `parameter_confidence: "inferred_from_pseudocode"`.

**Handling code that implements algorithms:**

- Record the algorithm logic for Phase 8 (programmatic blocks).
- Control laws in code → MATLAB Function blocks. Map code input/output names to model signal names.
- Signal processing / transforms in code → standard Simulink blocks or MATLAB Function blocks.

### 1b-iv: Implementation Directive Scan (MANDATORY for document mode)

**Before extracting any equations,** scan the paper for explicit build instructions — sentences where the author states which equations to use for the Simulink model. This determines which equation numbers are the "final product" vs derivation scaffolding.

**Search for these patterns** (in text extraction or vision pass):
- "substituting (X) into (Y), producing / yielding / gives..."
- "the model is constructed using (X-Y)"
- "the Simulink model uses (X)..."
- "build the subsystems using (X), (Y), and (Z)"
- "the state equations for simulation are..."
- Any sentence in a "Model Construction" / "Simulation" / "Implementation" section that references equation numbers

**Record the result** in `spec.implementation_ref`:
```json
"implementation_ref": {
  "equations": [5, 6, 7, 8, 9, 10, 15, 19, 20, 21],
  "source_text": "the Simulink model can be constructed using (5-8), (9,10), and (15-31)",
  "page": 4
}
```

If no implementation directive is found (paper only has derivation), set `"equations": []` and note `"source_text": "none found — using final derived form"`.

**Why this matters:** A paper may derive Eq 1-4 (in terms of currents), then say "substitute (11-14) into (1-4), producing Eq 5-8 (in terms of flux only)." If you extract Eq 1-4 you get 7 inputs; if you extract Eq 5-8 (the implementation form) you get 3. The paper already made the engineering decision — follow it.

### 1c: Build the Model Map
Process the source using this hierarchy: Source -> Chapter -> Section -> Model Fragment.

### 1c-ii: Identify physical subsystems early

Regardless of how the source organizes its sections, identify **physical subsystem groupings** from domain knowledge. Use the canonical subsystem tables in **Deterministic Output Rules, section D** of SKILL.md. Group by energy domain: each subsystem represents one conservation law, one constitutive relation set, or one coupling mechanism.

### 1d: Create model_id
Create a deterministic `model_id` from the document title or system name:
- lowercase, underscores instead of spaces, remove punctuation
- Example: `Nonlinear Vehicle Suspension Model` -> `nonlinear_vehicle_suspension_model`

### 1e: Write model_summary.md and model_map.json

**In autonomous mode:** Skip writing these files. Hold the model map and summary in working memory.

**In interactive mode:** Write both files in a **single MATLAB call**.

### Speed rules for Phases 1-3

1. **Do NOT read `builders.md` until Phase 5.** It's 700+ lines of builder syntax reference.
2. **Detect PDF tools first.** Run `where pdftotext` and `where pdftoppm` in one Bash call before choosing a tier.
3. **Parallelize image reads.** Whether Tier 1 (targeted pages) or Tier 2 (all pages), issue parallel Read calls in batches of 10.
4. **One Bash call for all setup.** `mkdir -p output/chapter_01 output/pdf_pages && cp source.pdf output/`.
5. **Single-pass extraction.** Extract everything in working memory, then write the spec JSON in **one MATLAB call**.
6. **No separate `equations.tex` file.** Raw and normalized LaTeX live in the spec JSON.
7. **Ideal tool call sequence (Tier 1, 30-page paper with equations on ~10 pages):**
   ```
   Call 1: where pdftotext && where pdftoppm  <- detect tools
   Call 2: pdftotext + pdftoppm (targeted)    }
   Call 3: mkdir + cp                          } <- parallel with Call 2
   Call 4: Read paper.txt (bulk text)
   --- LLM identifies equation/figure/code pages from text (artifact inventory) ---
   Call 5: Read page-23.png }
   Call 6: Read page-24.png } <- parallel (equations, figures, code pages)
   Call 7: Read page-45.png }
   --- LLM extracts equations + parameters from code + validation targets ---
   Call 8: MATLAB writeSpecJSON  <- one call writes the spec
   Call 9: (if code artifact) MATLAB run extracted .m file <- get parameter values
   ```
8. **Tier 2 fallback sequence (no pdftotext):**
   ```
   Call 1: pdftoppm -png -r 200 paper.pdf output/pdf_pages/page
   Call 2: Read pages 1-10   }
   Call 3: Read pages 11-20  } <- parallel batches
   Call 4: Read pages 21-30  }
   --- LLM thinks: extract everything ---
   Call 5: MATLAB writeSpecJSON
   ```

---

## Phase 1-D: Derivation Mode (No Document -- Derive from First Principles)

**Use this phase instead of Phases 1-3 when the user describes a system to model without providing a document.**

The derivation follows how a human engineer works: **draw first, then derive equations from the drawing.** The diagram defines topology, coordinates, and sign conventions — equations inherit these directly. This guarantees consistency between the schematic shown in the report and the equations used in the model.

### 1-D.a: Identify the system and scope

From the user's description, determine:
- **System type** -- e.g., friction clutch, DC motor, suspension, quadrotor
- **Complexity level** -- single DOF vs. multi-DOF, linear vs. nonlinear
- **Domain** -- mechanical, electrical, thermal, multi-physics
- **What the user likely wants** -- if they say "clutch model" they probably want engine-clutch-load with engagement dynamics

State what you plan to derive and the assumptions you'll make.

### 1-D.b: Draw system schematic and free body diagram

**This step comes BEFORE writing equations.** The diagram is a derivation tool, not a report decoration.

#### 1-D.b.1: Draw the system schematic

Generate a TikZ standalone document that shows the physical system:
- **Topology** -- masses, springs, dampers, circuits, rigid links, rotors (whatever the domain needs)
- **Coordinate frames** -- positive directions for displacement, rotation, force
- **Sign conventions** -- arrows on forces/torques/voltages showing assumed positive direction
- **Labels** -- all parameter names ($m$, $k$, $c$, $J$, $R$, $L$, ...) and state variables ($x$, $\theta$, $i$, ...)
- **Connections** -- how bodies/nodes are coupled (springs between masses, wires between components)

#### 1-D.b.2: Draw free body diagrams (FBDs)

For each body/node, show ALL forces/torques/currents acting on it:
- Applied forces (inputs)
- Reaction forces (springs, dampers, constraints)
- Inertial terms (if using D'Alembert)

Each FBD arrow directly maps to one term in the equation of motion.

#### 1-D.b.3: TikZ conventions (style consistency)

Use these standard styles for all diagrams:
```latex
\documentclass[border=10pt]{standalone}
\usepackage{tikz}
\usetikzlibrary{arrows.meta, decorations.markings, calc, patterns,
                decorations.pathmorphing}

\begin{tikzpicture}[
    force/.style={-{Stealth[length=3mm]}, thick, red!70!black},
    velocity/.style={-{Stealth[length=3mm]}, thick, blue!70!black},
    dimension/.style={|<->|, thin, gray},
    label/.style={font=\small},
    body/.style={fill=gray!20, draw=black, thick, rounded corners=2pt},
    spring/.style={thick, decorate, decoration={zigzag, segment length=4mm, amplitude=2mm}},
    damper/.style={thick}
]
```

Color conventions:
- **Red** (`red!70!black`): forces, torques, applied loads
- **Blue** (`blue!70!black`): velocities, displacements, state variables
- **Gray**: dimensions, structural elements, passive annotations
- **Black**: bodies, connections, ground

#### 1-D.b.4: Compile and verify

Compile the TikZ to PDF and convert to PNG:
```bash
pdflatex -interaction=nonstopmode <diagram>.tex
pdftoppm -png -r 200 <diagram>.pdf <diagram>
```
Visually verify the result with the `Read` tool. Fix any rendering issues before proceeding.

Save the PNG to: `<outputDir>/figures/schematic_<model_id>.png`

#### 1-D.b.5: Reference diagram in spec

The diagram path is stored in `spec.schematic`:
```json
"schematic": "<outputDir>/figures/schematic_<model_id>.png"
```
This feeds `rpt.schematic` in Phase 12, placing the diagram in Section 2 (Governing Equations) of the report.

### 1-D.c: Derive governing equations FROM the diagram

**Every equation must trace back to the FBD.** For each body/node drawn in 1-D.b.2:

1. **Sum forces/torques/voltages** using the sign conventions defined in the diagram
2. **Apply conservation laws** -- Newton's 2nd law, Kirchhoff's laws, energy conservation
3. **Add constitutive relations** -- Hooke's law, Ohm's law, friction models
4. **Write coupling equations** -- torque transmission, gear ratios, transformer relations

For each equation, document:
- Which FBD body it comes from
- Which physical law applies
- Which sign convention (positive direction) is used — inherited from the diagram

**Consistency check:** Every force arrow in the FBD must appear as a term in exactly one equation (the equation for the body it acts on). If you find a term in an equation that has no corresponding arrow in the FBD, the diagram and equations are inconsistent — fix one of them before proceeding.

### 1-D.c2: Lock the builder choice

Based on the derived equations, decide `odeBuilder` (default for continuous) vs `odeBuilder_cps` (hybrid/CPS) now. Record in spec: `spec.pipeline = 'simple'`, `'full'`, or `'bottom_up'` (continuous systems) or `spec.pipeline = 'cps'` (hybrid systems). Pipeline variant is finalized in Phase 4 exit -- this is a preliminary lock on builder choice only.

### 1-D.d: Choose representative parameters

Choose **physically reasonable parameter values** from engineering references. Document the source of each value. Parameters should match the labels used in the schematic (1-D.b).

### 1-D.e: Define validation targets

Without a paper, define physics-based validation checks:
- **Steady-state behavior** -- expected equilibrium
- **Energy conservation** -- decreasing for dissipative systems
- **Known limiting cases**
- **Physical plausibility** -- reasonable magnitudes, correct signs

### 1-D.f: Write report narrative content

**This step captures your physics understanding NOW, while it's freshest.** After executePlan runs, this context may be compacted. Writing it here guarantees the report carries your engineering insight.

See `spec_format.md` §9d for exact field formats.

#### 1-D.f.1: Narrative introduction (`spec.narrative_intro`)

Write 2-3 paragraphs explaining:
- What physical system this models and why it's interesting/useful
- What the key dynamics are (what makes the behavior non-trivial)
- What operating regimes exist and how they differ

**Tone:** Write for a controls engineer who hasn't seen this system before. Not a textbook derivation — a colleague explaining over coffee.

**BAD** (will trigger quality warning — too short, no insight):
> "Wind turbine model. 5-state model with 10 governing equations."

**GOOD** (passes quality check, gives real understanding):
> "This model captures the dynamics of a variable-speed wind turbine operating across below-rated, rated, and above-rated wind conditions. The aerodynamic subsystem uses a Cp-lambda-beta relationship to compute rotor torque from wind speed, rotor speed, and blade pitch angle. The drivetrain represents the torsional flexibility between the rotor hub and generator as a two-mass shaft system. A PI pitch controller regulates generator speed in above-rated wind by adjusting blade pitch angle, while a torque controller tracks optimal power in below-rated conditions."

#### 1-D.f.2: Subsystem context (`spec.subsystem_context`)

For each component, write 2-3 sentences explaining:
- What physical phenomenon this subsystem captures
- Why it's modeled separately (what would be lost without it)
- How it connects to the other subsystems physically (not just signal names)

#### 1-D.f.3: User guide (`spec.user_guide`)

Write practical guidance for someone who receives this model:
- Which parameters to change for what-if analysis
- Which blocks to modify for common variations
- Known limitations and how to extend the model
- What NOT to change (assumptions that would break if violated)

#### 1-D.f.4: Interesting experiments (`spec.interesting_experiments`)

Design 2-3 experiments that go beyond basic validation:
- **Stress test:** Push the system to an extreme — what breaks first?
- **What-if:** Change a key parameter or input — what happens?
- **Edge case:** Operate at a regime boundary — does the model handle it?

Each experiment should teach the user something about the system's behavior that isn't obvious from the equations.

#### 1-D.f.5: System illustration (`spec.illustration_path`)

Generate a MATLAB figure showing the physical system as a simple cartoon:
- Use `patch`, `rectangle`, `line`, `text` — no precision needed
- Label the key components and their physical relationship
- Show energy flow direction or signal flow
- Save to `<outputDir>/figures/system_illustration.png`

This is distinct from the derivation schematic (1-D.b): the illustration is "what is this system?" while the schematic is "where do the equations come from?"

**Note:** For non-mechanical domains, the "derivation diagram" from 1-D.b may serve both purposes. In that case, set `spec.illustration_path = spec.derivation_schematic` (same file).

### 1-D.g: Create model_id and output structure

Use the system name as `model_id`. Create output directory and write spec JSON in a **single MATLAB call**. Include `spec.schematic` pointing to the diagram PNG from step 1-D.b, and all narrative fields from 1-D.f.

### 1-D.h: Report style (user preference)

If the user specified a report preference (e.g., "simple report", "detailed", "just validate"), record it:
```matlab
spec.report_style = 'detailed';  % default if not specified
```

**After Phase 1-D, continue with Phase 4 (Normalize Equations).**

---

## Phases 2-3: Extract and Spec (Single Pass)

**Phases 2 and 3 are executed as a single cognitive pass.** After reading the source in Phase 1, extract all information and produce the spec JSON in one step.

### Spec reuse (determinism checkpoint)

Before extracting, check if a spec JSON already exists:
```matlab
specFile = fullfile(outputDir, 'chapter_01', 'section_01_spec.json');
if isfile(specFile)
    fprintf('Found existing spec: %s\n', specFile);
end
```

If found, **load and reuse it**. Only re-extract if the user explicitly requests it.

### What to extract

For each model-bearing section, extract in your working memory:

**Paper citation (MANDATORY for document-mode)** -- Extract from the first page or header:
- `spec.paper_title` — full title of the paper/thesis
- `spec.paper_authors` — author names (e.g., 'A. Smith, B. Jones')
- `spec.paper_journal` — venue, volume, pages, year (e.g., 'IEEE Trans. Veh. Technol., vol. 64, no. 9, pp. 3894-3907, 2015')

These feed the report's Source Citation block. `validateSpec` will warn if missing.

**Equations** -- governing ODEs, algebraic/output equations, constitutive relations, constraints, event/reset expressions. **Dimensional check**: verify unit consistency. **Implementation directive gate (MANDATORY):** If `spec.implementation_ref.equations` is non-empty, extract ONLY those equation numbers. Do NOT extract derivation-stage equations that the paper superseded with substituted forms. Example: if the paper derives Eq 1-4 then says "substitute, producing Eq 5-8" and `implementation_ref` lists [5,6,7,8], extract Eq 5-8 — not Eq 1-4.

**Variables** -- state names, physical meanings, units, inputs, outputs. **Disambiguate from context**.

**Parameters** -- names, values, units, source location. **Infer when not explicit** and document the inference.

**Simulation metadata** -- initial conditions, time span, input signals, solver hints. **Disturbance inputs:** If the paper applies a step load/force/torque at a specific time (e.g., "a 100 N-m load is applied at t=2s"), record as `load_step_time` and `load_step_value` in `simulation_setup`. Use a Step block — not a pulse generator. This feeds Phase 10 directly.

**Validation targets** -- figures to reproduce (**read approximate values from plots**), tables to match, textual claims.

**Per-subsystem validation data (MANDATORY extraction for document-mode)** -- For each physical subsystem, look for intermediate validation results the paper provides:

- Eigenvalues or poles stated for a subsystem (e.g., "The steering eigenvalues are λ₁ = −0.106, λ₂ = −1.769")
- Equilibrium/steady-state values (e.g., "At cruise, V = 23.646 ft/s, N = 1.503 rps")
- Step response characteristics (settling time, overshoot, time constant)
- Intermediate subsystem plots (e.g., open-loop Bode, step response of a plant block)
- Any numerical result the paper gives for a subsystem IN ISOLATION (not just final closed-loop)

Record these as `subsystem_tests` in the spec:

```json
"subsystem_tests": [
  {
    "subsystem": "PropulsionDynamics",
    "description": "Cruise equilibrium (Table 2)",
    "inputs": {"Qt_cmd": 219597},
    "duration": 300,
    "checks": [
      {"signal": "V", "type": "steady_state", "expected": 23.646, "tolerance": 0.01, "source": "Table 2"},
      {"signal": "N", "type": "steady_state", "expected": 1.503, "tolerance": 0.01, "source": "Table 2"}
    ]
  },
  {
    "subsystem": "SteeringDynamics",
    "description": "Open-loop eigenvalues (Eq 20)",
    "inputs": {"delta": 0.0873},
    "duration": 50,
    "checks": [
      {"signal": "rpri", "type": "settling_time", "expected": 6.0, "tolerance": 0.25, "source": "1/|λ_fast| from Eq 20"}
    ]
  }
]
```

**Rules for `subsystem_tests`:**
1. Only extract tests the paper explicitly provides data for. Do NOT invent analytical targets.
2. If the paper provides no intermediate subsystem data, set `"subsystem_tests": []`.
3. Check types: `steady_state`, `settling_time`, `peak`, `frequency`, `final_value`, `eigenvalues`.
4. Tolerance is fractional (0.01 = ±1%). Use ±1% for stated values, ±25% for approximate (read from plots).

**Validation figures (MANDATORY for document-mode builds)** -- Each result figure in the paper is a reproduction target. For every figure in the paper's results section, record:

```json
"validation_figures": [
  {
    "paper_fig": "Fig N",
    "title": "Disturbance Input and Primary Output",
    "signals": ["d1", "y4"],
    "layout": "stacked",
    "t_range": [0, 4],
    "y_ranges": [[0, 120], [0, 250]],
    "units": ["units_d", "units_y"],
    "expected_range": [
      {"final": [80, 120], "nonzero": true},
      {"final": [180, 200], "peak": [190, 260], "nonzero": true}
    ],
    "notes": "y4 = y4_raw * scale_factor; d1 is pulsed at t=2"
  },
  {
    "paper_fig": "Fig M",
    "title": "Coupling Output and Monitored Signal",
    "signals": ["z1", "m1"],
    "layout": "stacked",
    "t_range": [0, 4],
    "y_ranges": [[-500, 1200], [-600, 600]],
    "units": ["units_z", "units_m"],
    "expected_range": [
      {"final": [800, 1200], "nonzero": true},
      {"peak": [400, 600], "nonzero": true}
    ],
    "notes": "m1 = y1*cos(theta) - y2*sin(theta) (monitoring transform)"
  }
]
```

For each figure entry:
- `paper_fig`: exact figure number from the paper
- `signals`: signal names as they appear in the model (or with conversion note)
- `layout`: `"stacked"` (subplots sharing time axis), `"overlay"` (same axes with legend), or `"single"`
- `t_range`: time axis range `[t_start, t_end]` — drives `StopTime` in Phase 10
- `y_ranges`: one per signal, approximate axis limits read from the paper figure
- `units`: one per signal — the unit shown on the paper's axes
- `expected_range`: **RECOMMENDED** — one struct per signal with `.final`, `.peak`, `.trough` bounds read from the paper figure. This enables `checkFigureConsistency` to catch flat/zero/wrong-magnitude outputs automatically. Read approximate values from the paper plot (generous bounds, ±30-50% is fine). If omitted, only flat/zero detection works.
- `notes`: any conversion needed (e.g., electrical-to-mechanical speed, dq-to-abc transform)

**These entries drive Phase 10 directly:** simulation time comes from the longest `t_range`, plot layout from `layout`, axis ranges from `y_ranges`, signal selection from `signals`. `executePlan` generates one figure per `validation_figures` entry automatically from simulation output — the reader compares visually against the paper.

**Simulation timing rule:** The `t_range` values from `validation_figures` determine `StopTime` and disturbance timing — NOT a generic default. If the paper shows results from 0 to 4s with load at t≈0, that's what Phase 10 uses. If it shows 0 to 10s with load at t=5s, use that instead.

**Primary scenario (MANDATORY for document-mode)** -- Every paper demonstrates its model with a primary scenario (the main simulation result). This is NOT optional to implement. Record:

```json
"primary_scenario": {
  "name": "<descriptive name from paper>",
  "description": "<one sentence: what happens in this simulation>",
  "paper_figures": ["Fig X", "Fig Y"],
  "requires": ["<capability1>", "<capability2>"],
  "feedback_signals": ["<signal1>", "<signal2>"],
  "stop_time": <seconds from paper>,
  "initial_conditions": {"<state1>": <value>, "<state2>": <value>}
}
```

Fields:
- `name`: short descriptive name of the scenario
- `description`: one sentence explaining what happens
- `paper_figures`: which figures show the scenario results
- `requires`: what capabilities the model needs (used by Phase 8 to know what to build)
- `feedback_signals`: signals that must be accessible as subsystem outports for the scenario to work (used by Phase 11 `verifyOutportCompleteness`)
- `stop_time`: simulation duration for this scenario
- `initial_conditions`: IC overrides specific to this scenario (merged with base ICs)

**Rules for `primary_scenario`:**
1. Every document-mode build MUST have one. If the paper has no simulation results, set to `null` and note in ambiguities.
2. The scenario drives Phase 8 (must build command generation if `requires` includes it) and Phase 10 (must run this scenario as one of the 3+ tests).
3. `feedback_signals` drives Phase 11 outport completeness — these signals MUST be exposed at subsystem boundaries.
4. For derivation mode: set `primary_scenario` based on what would meaningfully demonstrate the system (step response, free decay, etc.).

**Scenario signals (derived from primary_scenario):** Add to spec:
```json
"scenario_signals": ["Xo", "Yo", "V", "psi"]
```
This is the union of `primary_scenario.feedback_signals` + key state variables needed for validation plots. Phase 11 uses this to verify outport completeness.

**Reference data for figure overlays (RECOMMENDED for document-mode):** When paper figures contain time-series data that can be digitized (read approximate values from axes), extract key data points for overlay plotting in Phase 10. Store in spec:

```json
"reference_data": [
  {
    "paper_fig": "Fig 16",
    "signals": [
      {
        "name": "Yo",
        "time": [0, 100, 200, 300, 400, 500, 600],
        "data": [0, 50, 100, 125, 130, 130, 130],
        "unit": "ft"
      }
    ]
  },
  {
    "paper_fig": "Fig 15",
    "signals": [
      {
        "name": "psi",
        "time": [0, 50, 100, 200, 400, 600],
        "data": [0, 4, 7, 5, 1, 0],
        "unit": "deg"
      }
    ]
  }
]
```

**Rules for `reference_data`:**
1. Only extract when you can read approximate values from the paper's plots (not from prose or tables — those go in `validation` targets).
2. 5-10 points per signal is sufficient for overlay comparison. Focus on key features: initial slope, peak, settling, final value.
3. Values are approximate (read from pixel position on axis) — that's expected and acceptable.
4. Phase 10 uses these with `plotWithReference.m` to generate simulation-vs-paper overlay figures.
5. If the paper only shows results in text/tables (no plottable figures), omit `reference_data`.

**Do not silently invent missing values.** Record ambiguities.

**Ambiguities and decisions log (MANDATORY):** Record every non-obvious decision in `spec.ambiguities` as a cell array of strings. Each entry should be a one-line statement of what was decided and why. These flow directly into the report's "Ambiguities and Decisions" section.

Examples:
```json
"ambiguities": [
  "Used HYDROGEN.M value I=190000 instead of text Eq 13 value I=200000 (code is authoritative)",
  "Y-positive starboard from Figure 1 coordinate convention",
  "Eq 6a OCR garbled: used code expression Yvh = -pi*(T/L)*Cdo (multiplication not subtraction)"
]
```

Continue appending to `spec.ambiguities` in all subsequent phases whenever a non-obvious choice is made (Phase 4: nondimensionalization decisions, Phase 7: time-scaling choices, Phase 8: controller simplifications).

**Artifacts** -- Record the artifact inventory from Phase 1b in the spec:

```json
"artifacts": [
  {
    "type": "code",
    "language": "matlab",
    "pages": [45, 54],
    "role": "parameter_computation",
    "name": "<FILENAME>.M",
    "description": "<what it computes: coefficients, matrices, lookup data, etc.>"
  },
  {
    "type": "code",
    "language": "matlab",
    "pages": [55, 58],
    "role": "preprocessing",
    "name": "<FILENAME>.M",
    "description": "<what it preprocesses: geometry coefficients, material properties, etc.>"
  },
  {
    "type": "result_plot",
    "pages": [35],
    "figure_id": "Fig 14",
    "signals": ["psi", "Yo"],
    "steady_state": {"Yo": 130}
  }
]
```

Artifact types: `"code"`, `"table"`, `"block_diagram"`, `"result_plot"`, `"algorithm"`, `"data_array"`.
Code roles: `"parameter_computation"`, `"control_law"`, `"preprocessing"`, `"simulation_script"`, `"data_source"`.

**For parameter-computing code artifacts:** After recording the artifact, extract and run (or read) the code to obtain parameter values. Record `"parameter_source": "code_appendix"` in the spec. These values take precedence over all other sources.

### Deterministic equation form selection

**Implementation-first rule:** When a paper has both a derivation section (developing equations step-by-step) and an implementation/simulation section (presenting the final equations used to build the model), **ALWAYS extract from the implementation section.** The implementation equations are what the paper actually verified — they contain substitutions, simplifications, and parameter groupings that the authors found necessary. Derivation equations are intermediate steps, not the final product.

**Cross-check with `spec.implementation_ref`:** If step 1b-iv found an implementation directive (e.g., "build using Eq 5-8"), verify that the equations you are about to extract match those numbers. If you find yourself extracting Eq 3 when the paper says to use Eq 5 (the substituted form of Eq 3), STOP — go read Eq 5 instead. The implementation directive is authoritative.

When the source provides multiple forms, select deterministically:
1. **Use the paper's implementation equations** (the equation numbers in `spec.implementation_ref`). If the paper says "substitute (11-14) into (1-4), producing (5-8)" — extract Eq 5-8, not Eq 1-4. The substitution is intentional engineering: it reduces inputs and eliminates feedback loops.
2. **Preserve the paper's subsystem boundaries.** If the paper's block diagram routes a signal (e.g., currents, forces) between subsystems, that signal is intentional architecture. Do NOT inline algebra that the paper deliberately separates into its own computation block.
3. **Trace the signal chain.** Confirm every subsystem output has a downstream consumer.
4. **Algebraic loop check:** If an algebraic output `y = g(...)` appears in the SAME ODE's derivative (`ẋ = f(x,y)` where `y` depends on `ẋ`), that is a true algebraic loop — substitute to eliminate it. But states → algebra → derivative (`y = g(x)`, `ẋ = f(x,y)`) is NOT an algebraic loop and should NOT be inlined.
5. **Record every choice** in `spec.assumptions`.

### Component Registry

**CRITICAL:** Identify ALL model components -- not just equations, but every functional block:

| Category | Examples |
|----------|---------|
| **Input sources** | Inverters, voltage/current sources, reference signals, driving cycles |
| **Coordinate transforms** | abc<->dq, body<->earth, rotating frames, Euler angles |
| **Plant dynamics** | ODEs, DAEs -- the core model equations |
| **Algebraic equations** | Current calculations, force/torque algebra, kinematics |
| **Controllers** | PID, autopilot, state feedback, gain scheduling |
| **Nonlinear elements** | Saturation, rate limiters, dead zones, lookup tables |
| **Output processing** | Inverse transforms, sensor models, signal conditioning |

For each component, record: name, description, source, build_method (`odeBuilder` or `programmatic`), phase, status (`planned`).

**Every component must reach `built` status before Phase 10.**

### External Boundary Rule (HARD CONSTRAINT)

**A model input may be marked `source: 'external'` ONLY if the paper does NOT describe how the signal is generated.** This is the single most important boundary decision in the pipeline.

**Test:** For each candidate external input, ask: "Does the paper describe ANY mechanism that produces this signal?" If yes — even a brief description, a block in a diagram, a table, or a single sentence — then that mechanism is a required component, and the signal is NOT external.

**External** means: the paper takes this signal as a given. It provides no equations, no block, no logic, and no description of how it is created. Examples:
- "A step disturbance is applied at t=2s" → external (paper gives the value, not a generation mechanism)
- "Supply voltage is 460V" → external (stated as a given, no source described)
- "Ambient temperature is 25C" → external (environmental condition)

**NOT external** means: the paper describes how the signal is produced, even briefly. Examples:
- Paper shows a block/equation/table that outputs this signal → component
- Paper describes switching logic, modulation, transform, filtering, or any computation → component
- Paper's block diagram has a labeled box producing this signal → component

**Why this matters:** A constant input cannot reproduce the behavior of a described source. If the paper's validation figures depend on dynamics from that source (harmonics, transients, switching, feedback), the model will never match.

**Default bias: INCLUDE.** When uncertain, include it as a component. A component can be simplified later (replaced with a constant for debugging). A missing component requires re-architecture.

### Physical subsystem decomposition (DETERMINISTIC -- locked here, used in Phase 11)

**Phase 11 does NOT re-derive the decomposition.** It reads `physical_subsystems` from this spec verbatim. All structural decisions are made HERE, once, and never revisited.

**Step 1: Read the paper's block diagram figure.** Convert to PNG and visually inspect. Count distinct labeled blocks. Record the figure number and block names exactly as the paper labels them.

**Step 2: Assign equations to subsystems** based on the paper's structure:
- Each labeled block in the paper = one entry in `physical_subsystems`
- Use the paper's block names (or closest English equivalent)
- If no block diagram exists: group by energy domain (ODEs by mutual coupling, algebraic by consumers)

**Step 3: Define port ordering** for each subsystem. Ports are ordered top-to-bottom on the block as they appear in the paper's diagram (or alphabetically if not visible):
- `inports`: list of input signal names, in order
- `outports`: list of output signal names, in order

**Step 4: Record paper figure reference** for each subsystem.

### Write spec JSON -- single MATLAB call

```matlab
writeSpecJSON(outputDir, spec);
```
Note: `writeSpecJSON` automatically records `pipeline_start_time` (used by `executePlan` to auto-compute pipeline duration in the report).

The `physical_subsystems` array drives hierarchy creation in Phase 11. **Phase 11 reads this verbatim — it does NOT re-derive groupings.** The array ORDER defines signal-flow layout (left-to-right in the model):

```json
"physical_subsystems": [
  {
    "name": "PlantDynamics",
    "equation_ref": "Eq X-Y",
    "equations": [1, 2, 3, 4, 5],
    "inports": ["u1", "u2", "y4"],
    "outports": ["x1", "x2", "x3", "x4"],
    "extras": []
  },
  {
    "name": "OutputAlgebra",
    "equation_ref": "Eq Z1-Z4",
    "equations": [6, 7, 8, 9],
    "inports": ["x1", "x2", "x3", "x4"],
    "outports": ["y1", "y2", "y3", "y4"],
    "extras": []
  },
  {
    "name": "CouplingEquation",
    "equation_ref": "Eq W",
    "equations": [10],
    "inports": ["y1", "y2", "x1", "x3"],
    "outports": ["z1"],
    "extras": []
  },
  {
    "name": "ActuatorDynamics",
    "equation_ref": "Eq V",
    "equations": [11],
    "inports": ["z1", "d1"],
    "outports": ["y4"],
    "extras": []
  }
]
```

**Source reference rule:** Use `"equation_ref"` (e.g., `"Eq X-Y"`) to cite the paper's equation numbers. Only use `"paper_figure"` when equations were visually derived from a block diagram or schematic figure (no explicit equations in the text). Never put Simulink block names in source references.

**Deterministic rules (no LLM judgment at runtime):**
1. **Array order** = signal-flow order (left-to-right in model layout)
2. **Port ordering** = ALPHABETICAL within each port list. Always. No exceptions. This is the only ordering rule that produces identical results regardless of who reads the paper.
3. **Signal names** = use the variable name from the normalized equations (e.g., `x1` not `state_1`, not `X_ONE`). Same name everywhere a signal appears. SubsystemA `outports: ["x1"]` must match SubsystemB `inports: ["x1"]` exactly.
4. **Subsystem count** = matches paper's PLANT block count. Programmatic blocks (inverters, transforms) go in `components`, not `physical_subsystems`.
5. **Subsystem naming convention (MANDATORY — removes all LLM choice):**
   - Use PascalCase, no spaces, no underscores, no abbreviations
   - Name describes the PHYSICAL FUNCTION using the standard vocabulary below:
     - ODE blocks (contain integrators): name after the state variable domain — `FluxLinkages`, `MechanicalDynamics`, `ThermalDynamics`, `TranslationalDynamics`, `RotationalDynamics`
     - Algebraic blocks (compute outputs from states): name after what they compute — `CurrentAlgebra`, `ForceAlgebra`, `KinematicOutputs`
     - Single-equation coupling: name after the output variable — `ElectromagneticTorque`, `PropulsiveForce`, `FrictionForce`
   - This removes LLM choice: a block computing algebraic outputs from state variables is ALWAYS named by its standard function (e.g., `CurrentAlgebra`), never "Current Model", "Output Calculation", or ad-hoc names
   - **Standard names by domain (use these exact names):**

   | Domain | ODE subsystem | Algebraic subsystem | Coupling equation |
   |--------|--------------|--------------------|---------| 
   | Electrical machines | `FluxLinkages` | `CurrentAlgebra` | `ElectromagneticTorque` |
   | Rotational mechanics | `MechanicalDynamics` | `KinematicOutputs` | — |
   | Translational | `TranslationalDynamics` | `ForceAlgebra` | — |
   | Thermal | `ThermalDynamics` | `HeatFluxAlgebra` | — |
   | Vehicle lateral | `VehicleDynamics` | `TireForces` | — |
   | Ship maneuvering | `HullDynamics` | `HydrodynamicForces` | `PropulsionForce` |
   | Suspension | `SuspensionDynamics` | `SpringDamperForces` | — |

   If a domain isn't listed, compose using: `<PhysicalDomain><Dynamics|Algebra|Force|Torque>`

**For `pipeline: 'simple'`**, set `physical_subsystems` to `[]`.
**For CPS models**, `physical_subsystems` is also `[]` -- use `modes` and `transitions` fields instead.

### Validate spec against schema (MANDATORY)

**Before writing the spec**, read the schema to know the field contract:
```
Read specSchema.json   (in util/ directory)
```

This tells you every field's name, type, required status, and which consumers use it.
Key rules: `parameters(i).value` MUST be numeric. `validation.tests(i).expected` MUST be numeric. `equations_raw_latex` is `cell_of_char`. `chapter_id`/`section_id` are numeric.

**After writing the spec**, validate it:
```matlab
[~, valid, issues] = specSchema(spec);
assert(valid, 'Spec schema validation failed — fix before proceeding');
```

Fix any `[ERROR]` issues (wrong types, missing required fields) before proceeding.

### Validate spec for determinism

After schema validation, run the content validator:
```matlab
[isValid, issues] = validateSpec(spec);
```

Fix any `[ERROR]` issues before proceeding.

### Validate extraction for hallucination (MANDATORY)

**Immediately after `validateSpec`**, run the circular-dependency check:
```matlab
[isClean, flags] = validateExtraction(spec);
```

This detects the most common extraction hallucination: an ODE containing a variable that is also the LHS of an algebraic equation, where that algebraic equation depends on the ODE's own state. This circular pattern means the paper likely uses the substituted form (variable eliminated) but the LLM wrote the un-substituted textbook form from memory.

**If `isClean == false` (flags raised):**
1. **DO NOT proceed to Phase 4.** The extraction is likely wrong.
2. Re-read the specific equation page image (the PNG, not from memory).
3. Look for a LATER section in the paper that presents substituted/inlined equations (e.g., "substituting into...", "the final state model is...", "for simulation we use..."). Papers almost always provide such forms — the flags mean you grabbed the derivation step instead of the implementation step.
4. For each flagged ODE, extract EXACTLY what is printed on the page — character by character.
5. Update `spec.equations_raw_latex` with the corrected (typically inlined) equations.
6. Re-run `validateExtraction(spec)` until clean.

**Override protocol (STRICT):** To override a `validateExtraction` flag and keep the un-substituted form, you MUST cite the specific equation number(s) in the paper that prove no substituted/inlined form exists anywhere in the document. Acceptable evidence: "I searched all pages after the derivation section and found no combined form — the paper only presents Eq. 5-8 as separate ODE + algebraic pairs." If you cannot cite such evidence, you MUST use the substituted form. **Saying "known false positive" or "re-read and confirmed" without equation-number citations is NOT sufficient to override.**

**This check is not optional.** It catches 100% of tested hallucination cases with zero false positives. The check is domain-agnostic because it operates on literal LaTeX fragment matching within the same extraction pass.

Set status to `extracted` after this phase.

---

## Phase 3b: Report Narrative Content (Both Modes)

**Applies to:** Both document-mode and derivation-mode. Do this AFTER extraction/derivation is complete, BEFORE the gate.

**Why here:** Your understanding of the system is deepest right now. After `executePlan` runs, context may be compacted. Capture your engineering insight while you have it.

See `spec_format.md` §9d for exact field formats and examples.

### 3b.1: Report style

Check if the user specified a preference. Common requests:
- "simple report" / "quick" / "just validate" → `spec.report_style = 'simple'`
- "detailed" / "full report" → `spec.report_style = 'detailed'` (default)
- "executive summary" / "short" → `spec.report_style = 'executive'`
- "no report" / "skip report" → `spec.report_style = 'none'`

If not specified, default to `'detailed'`.

### 3b.2: Narrative introduction (`spec.narrative_intro`)

Write 2-3 paragraphs. Cover:
- What physical system this is and its real-world application
- What makes the dynamics interesting (nonlinearities, coupling, regimes)
- What the model captures vs. what it simplifies away

**Document mode:** Summarize the paper's contribution — what did the authors do and why?
**Derivation mode:** Explain the engineering context — when would someone need this model?

### 3b.3: Subsystem context (`spec.subsystem_context`)

For each physical subsystem (from `spec.physical_subsystems`), write 2-3 sentences:
- What physical phenomenon it captures
- Why it needs its own subsystem (what coupling or dynamics it adds)
- How it relates to the other subsystems physically

### 3b.4: User guide (`spec.user_guide`)

Write practical guidance (4-8 sentences):
- Which parameters are most useful to change for what-if studies
- Which blocks to modify for common extensions
- Known limitations and assumptions that constrain validity
- Suggestions for extending the model (add DOFs, switch controllers, etc.)

### 3b.5: Interesting experiments (`spec.interesting_experiments`)

Design 2-3 experiments beyond the paper's validation scenario. Ask yourself:
- "What would a user want to try first after opening this model?"
- "What behavior isn't obvious from the equations alone?"
- "What breaks if you push the system hard?"

Each experiment: name, description, setup instructions, what to check.

### 3b.6: System illustration (derivation mode only)

If not already generated in 1-D.b, create a simple system cartoon using MATLAB:
```matlab
figure('Visible','off'); 
% Draw with patch/rectangle/line/text...
exportgraphics(gcf, fullfile(figDir, 'system_illustration.png'), 'Resolution', 150);
close(gcf);
spec.illustration_path = fullfile(figDir, 'system_illustration.png');
```

For document mode: extract a system diagram from the paper (if available) or skip.

---

## Gate: Before Phase 5

```
Phase Gate -- Ready for Phase 5?
- [ ] spec JSON exists (status: extracted)
- [ ] specSchema(spec) returns valid == true (types correct)
- [ ] validateExtraction(spec) returns isClean == true
- [ ] All equations, parameters, ICs, and validation targets captured in spec
- [ ] Artifact inventory complete (code, tables, plots, diagrams all identified)
- [ ] Code artifacts processed: parameter values extracted (run or read)
- [ ] Parameters sourced from code take precedence over inferred values
- [ ] Physical subsystem decomposition identified (full pipeline) or pipeline: simple decided
- [ ] subsystem_tests populated (or explicitly [] if paper provides no intermediate data)
- [ ] primary_scenario populated (or null if paper has no simulation results)
- [ ] scenario_signals populated (union of feedback_signals + key states for plots)
- [ ] spec.invariants populated (physical plausibility assertions — see spec_format.md §9b)
- [ ] spec.validation_figures populated with signals, layout, t_range, units, expected_range (see spec_format.md §9c)
- [ ] For CPS: modes, transitions, and cps_inputs populated in spec
- [ ] spec.narrative_intro populated (MINIMUM 150 chars / ~2 sentences. Target: 3 paragraphs explaining system, dynamics, regimes)
- [ ] spec.subsystem_context populated (at least 1 entry per physical_subsystem, each >50 chars)
- [ ] spec.user_guide populated (MINIMUM 100 chars. Target: 4-8 sentences on what to tune and extend)
- [ ] spec.interesting_experiments populated (MINIMUM 2 experiments, each with name+description+setup+what_to_check)
- [ ] spec.report_style set (user preference or default 'detailed')
- [ ] [Derivation mode] spec.derivation_schematic: PNG file EXISTS on disk (from 1-D.b compile step)
- [ ] [Derivation mode] spec.illustration_path: PNG file EXISTS on disk (from 1-D.f.5)
- [ ] [BLOCKING] If report_style ~= 'none': run buildReportStruct dry-check — all warnings must be addressed before proceeding
```
