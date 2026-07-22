# Contributing a Project

Best practices for adding or updating projects in this repository.

## Project Structure

Projects live in one of two top-level folders based on their type:

- `Packages/` — Reusable toolboxes, libraries, or block libraries
- `Examples/` — Self-contained demos that illustrate a technique or workflow

Each project gets its own folder (kebab-case name):

```
Packages/my-new-package/
├── README.md          (required)
├── images/            (at least one image or GIF)
│   └── demo.gif
├── src or top-level .m files
└── ...
```

## README Requirements

Every project **must** have a `README.md` in its root folder. The README should include:

1. **Title** — formatted as a top-level heading
2. **One-paragraph summary** — what the project does and why it matters
3. **At least one image or GIF** — embedded in the README (not just in an `images/` folder)
4. **Requirements** — MATLAB version and required toolboxes
5. **Installation / Quick Start** — how to get running
6. **Usage** — key functions, classes, or workflows

### Trademarks and Registered Trademarks

MathWorks product names must use the correct marks **on first use** in the README:

| Product | First use | Subsequent uses |
|---------|-----------|-----------------|
| MATLAB | MATLAB&reg; | MATLAB |
| Simulink | Simulink&reg; | Simulink |
| Simscape | Simscape&trade; | Simscape |
| Stateflow | Stateflow&reg; | Stateflow |
| MATLAB Online | MATLAB&reg; Online&trade; | MATLAB Online |

Use the HTML entities `&reg;` and `&trade;` in Markdown. Only the **first occurrence** in the document needs the mark — subsequent mentions are plain text.

Toolbox names (e.g., "Computer Vision Toolbox", "Image Processing Toolbox") do require trademark symbols.

## Licensing and Copyright

The repository has a **single BSD-3-Clause license** at the repo root (`LICENSE`). Individual project folders **must not** contain:

- Their own `LICENSE` or `LICENSE.md` file
- Copyright headers or notices in source files
- Any other licensing text

This keeps legal attribution consistent and avoids conflicting terms.

## Images

Every project must include **at least one image** (PNG, JPG, or GIF) that visually represents what the project does. This image is used:

- In the project's README
- As the thumbnail on the GitHub Pages showcase site
- In `projects.json` (as a raw GitHub URL)

Good image choices: a demo GIF, a key output figure, an app screenshot, or a system diagram.

Store images in an `images/` subfolder within your project directory.

## Git Workflow

### Adding a New Project

1. **Create a branch** from `main`:
   ```
   git checkout -b add-my-project
   ```

2. **Add your project folder** under `Packages/` or `Examples/`

3. **Update `projects.json`** (see below)

4. **Push and open a Pull Request** — get at least one review before merging

5. **Merge to `main`** when ready — the GitHub Pages site updates automatically

### Updating an Existing Project

Same branch-and-PR workflow. Keep commits focused — separate content changes from `projects.json` metadata updates when possible.

## Updating `projects.json`

When you add or modify a project, update `projects.json` in the repo root. This file drives the GitHub Pages showcase site.

### Schema

Each entry is a JSON object with these fields:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `name` | string | yes | Display name of the project |
| `folder` | string | yes | Path relative to repo root (e.g., `Packages/my-project`) |
| `type` | string | yes | Either `"package"` or `"example"` |
| `summary` | string | yes | 1-3 sentence description for the showcase card |
| `image` | string | yes | Raw GitHub URL to the primary image |
| `tools` | string[] | yes | MATLAB version and required toolboxes/products |
| `topics` | string[] | yes | Granular topic tags (kebab-case) |
| `categories` | string[] | yes | High-level categories for filtering |

### Field Guidelines

**`image`** — Use the raw GitHub URL format:
```
https://raw.githubusercontent.com/mathworks/Research-Office-Projects/main/<folder>/images/<file>
```

**`tools`** — Include the minimum MATLAB version with a `+` suffix, then list required toolboxes. Mark optional dependencies with `(optional)`:
```json
"tools": ["MATLAB R2024b+", "Simulink", "Image Processing Toolbox", "Simulink 3D Animation (optional)"]
```

**`topics`** — Granular, kebab-case tags that describe what the project is about. These appear on cards and are searchable:
```json
"topics": ["optical-flow", "video-processing", "frame-interpolation", "computer-vision"]
```

**`categories`** — Choose from the established set (add new ones sparingly):

- Computer Vision
- 3D & Simulation
- Optimization & Control
- Robotics
- Mathematics

A project can belong to multiple categories. Pick all that genuinely apply.

### Example Entry

```json
{
  "name": "Video Framerate Upsampling",
  "folder": "Packages/video-framerate-upsampling",
  "type": "package",
  "summary": "Increases video frame rate using optical flow frame interpolation. Computes bidirectional flow between consecutive frames, forward-splats pixels with exponential importance weighting, and fills holes with local median filtering.",
  "image": "https://raw.githubusercontent.com/mathworks/Research-Office-Projects/main/Packages/video-framerate-upsampling/comparison.gif",
  "tools": ["MATLAB R2024b+", "Computer Vision Toolbox", "RAFT Optical Flow Model (support package)"],
  "topics": ["optical-flow", "video-processing", "frame-interpolation", "computer-vision"],
  "categories": ["Computer Vision"]
}
```

## Checklist Before Opening a PR

- [ ] Project folder is in the correct location (`Packages/` or `Examples/`)
- [ ] `README.md` exists with proper trademark usage on first mention
- [ ] No `LICENSE` file in the project folder
- [ ] At least one image present and embedded in the README
- [ ] `projects.json` updated with all required fields
- [ ] Image URL in `projects.json` uses the raw GitHub format and is accessible
- [ ] Branch is up to date with `main`
