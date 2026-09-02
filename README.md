# Research Office Projects

Projects from the MathWorks Advanced Research & Technology Office. This repository collects research prototypes, examples, and packages for MATLAB&reg; and Simulink&reg;. The projects cover event-camera simulation, video frame interpolation, 3D rendering, requirements translation, battery charging, robotics, solar planning, and mathematical experiments.

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=mathworks/Research-Office-Projects)

## Structure

**Packages/** contains reusable MATLAB and Simulink code. Each package is self-contained and can be added to your path. For example, the [Event Camera Simulator](Packages/event-camera-simulator/) package generates asynchronous event streams from intensity frame sequences for testing DVS algorithms without physical hardware.

**Examples/** contains ready-to-run demonstrations across multiple toolboxes. For example, the [Battery Fast Charging Optimization](Examples/battery-fast-charging-optimization/) example compares CC-CV, multi-stage, and optimized charging strategies using a physics-based Single Particle Model, with an app for exploring results.

## Current Contents

<!-- project-table:start -->
### Packages

| Name | Description | Download |
|------|-------------|----------|
| [event-camera-simulator](Packages/event-camera-simulator/) | ESIM-based event camera (DVS) simulator for MATLAB and Simulink with noise model and visualization | [ZIP](https://github.com/mathworks/Research-Office-Projects/releases/download/project-downloads/package-event-camera-simulator.zip) |
| [floor-plan-to-3D](Packages/floor-plan-to-3D/) | Convert architectural floor plan images into 3D wall geometry with automatic door, window, and passage detection | [ZIP](https://github.com/mathworks/Research-Office-Projects/releases/download/project-downloads/package-floor-plan-to-3D.zip) |
| [video-framerate-upsampling](Packages/video-framerate-upsampling/) | Increases video frame rate using optical flow (RAFT/Farneback) frame interpolation | [ZIP](https://github.com/mathworks/Research-Office-Projects/releases/download/project-downloads/package-video-framerate-upsampling.zip) |
| [matsuba](Packages/matsuba/) | Physically-based rendering for MATLAB via Mitsuba 3: photorealistic images, differentiable rendering, and transient light transport | [ZIP](https://github.com/mathworks/Research-Office-Projects/releases/download/project-downloads/package-matsuba.zip) |
| [fret-to-simulink](Packages/fret-to-simulink/) | Translate NASA FRET temporal-logic requirements into Simulink Requirements Table and Test Assessment blocks for formal and runtime verification | [ZIP](https://github.com/mathworks/Research-Office-Projects/releases/download/project-downloads/package-fret-to-simulink.zip) |
| [xtosim](Packages/xtosim/) | AI-powered pipeline that converts research papers, equations, and system descriptions into validated Simulink models using multiple specialized builders | [ZIP](https://github.com/mathworks/Research-Office-Projects/releases/download/project-downloads/package-xtosim.zip) |

### Examples

| Name | Description | Download |
|------|-------------|----------|
| [battery-fast-charging-optimization](Examples/battery-fast-charging-optimization/) | Compare CC-CV, multi-stage, and optimized charging strategies for a lithium-ion battery (SPM) with interactive app | [ZIP](https://github.com/mathworks/Research-Office-Projects/releases/download/project-downloads/example-battery-fast-charging-optimization.zip) |
| [shape-from-shading-asteroids](Examples/shape-from-shading-asteroids/) | Enhance stereo depth maps of asteroid Bennu using Shape from Shading | [ZIP](https://github.com/mathworks/Research-Office-Projects/releases/download/project-downloads/example-shape-from-shading-asteroids.zip) |
| [spot-sim3D](Examples/spot-sim3D/) | Keyboard-controlled quadruped walking with Simscape&trade; Multibody&trade; and Unreal Engine&reg; 3D visualization | [ZIP](https://github.com/mathworks/Research-Office-Projects/releases/download/project-downloads/example-spot-sim3D.zip) |
| [finding-pi-with-marbles](Examples/finding-pi-with-marbles/) | Estimate &pi; using a marble bag game developed in a Claude Code and MATLAB MCP workflow | [ZIP](https://github.com/mathworks/Research-Office-Projects/releases/download/project-downloads/example-finding-pi-with-marbles.zip) |
| [solar-power-app](Examples/solar-power-app/) | Interactive rooftop solar panel planner with satellite imagery, tilt optimization, and energy yield estimation | [ZIP](https://github.com/mathworks/Research-Office-Projects/releases/download/project-downloads/example-solar-power-app.zip) |
<!-- project-table:end -->

## License

Licensed under the MathWorks BSD-3-Clause License. See [LICENSE](LICENSE).
