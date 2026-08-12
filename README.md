# Research Office Projects

Projects from the MathWorks Advanced Research & Technology Office. This repository collects research prototypes, examples, and packages for MATLAB&reg; and Simulink&reg;. The projects cover event-camera simulation, video frame interpolation, 3D rendering, requirements translation, battery charging, robotics, solar planning, and mathematical experiments.

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=mathworks/Research-Office-Projects)

## Structure

**Packages/** contains reusable MATLAB and Simulink code. Each package is self-contained and can be added to your path. For example, the [Event Camera Simulator](Packages/event-camera-simulator/) package generates asynchronous event streams from intensity frame sequences for testing DVS algorithms without physical hardware.

**Examples/** contains ready-to-run demonstrations across multiple toolboxes. For example, the [Battery Fast Charging Optimization](Examples/battery-fast-charging-optimization/) example compares CC-CV, multi-stage, and optimized charging strategies using a physics-based Single Particle Model, with an app for exploring results.

## Current Contents

| Type | Name | Description |
|------|------|-------------|
| Package | [event-camera-simulator](Packages/event-camera-simulator/) | ESIM-based event camera (DVS) simulator for MATLAB and Simulink with noise model and visualization |
| Package | [video-framerate-upsampling](Packages/video-framerate-upsampling/) | Increases video frame rate using optical flow (RAFT/Farneback) frame interpolation |
| Package | [floor-plan-to-3D](Packages/floor-plan-to-3D/) | Convert architectural floor plan images into 3D wall geometry with automatic door, window, and passage detection |
| Package | [matsuba](Packages/matsuba/) | Physically-based rendering for MATLAB via Mitsuba 3: photorealistic images, differentiable rendering, and transient light transport |
| Package | [fret-to-simulink](Packages/fret-to-simulink/) | Translate NASA FRET temporal-logic requirements into Simulink Requirements Table and Test Assessment blocks for formal and runtime verification |
| Example | [battery-fast-charging-optimization](Examples/battery-fast-charging-optimization/) | Compare CC-CV, multi-stage, and optimized charging strategies for a lithium-ion battery (SPM) with interactive app |
| Example | [shape-from-shading-asteroids](Examples/shape-from-shading-asteroids/) | Enhance stereo depth maps of asteroid Bennu using Shape from Shading |
| Example | [spot-sim3D](Examples/spot-sim3D/) | Keyboard-controlled quadruped walking with Simscape&trade; Multibody&trade; and Unreal Engine&reg; 3D visualization |
| Example | [finding-pi-with-marbles](Examples/finding-pi-with-marbles/) | Estimate &pi; using a marble bag game developed in a Claude Code and MATLAB MCP workflow |
| Example | [solar-power-app](Examples/solar-power-app/) | Interactive rooftop solar panel planner with satellite imagery, tilt optimization, and energy yield estimation |

## License

Licensed under the MathWorks BSD-3-Clause License. See [LICENSE](LICENSE).
