# Research Office Projects

Open-source packages and examples from the MathWorks Research Office. Each project explores emerging workflows — from event-driven camera simulation to physics-based battery optimization — and ships as a self-contained, ready-to-run package.

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=mathworks/Research-Office-Projects)

## Structure

**Packages/** extend MATLAB&reg; and Simulink&reg; with new capabilities — each is self-contained and can be added to your path and used immediately. For example, the [Event Camera Simulator](Packages/event-camera-simulator/) package lets you generate realistic asynchronous event streams from any intensity frame sequence — useful for testing DVS algorithms without physical hardware.

**Examples/** are ready-to-run demonstrations of Model-Based Design workflows spanning multiple toolboxes. For example, the [Battery Fast Charging Optimization](Examples/battery-fast-charging-optimization/) example compares CC-CV, multi-stage, and fmincon-optimized charging strategies using a physics-based Single Particle Model, with an interactive app for exploring results.

## Current Contents

| Type | Name | Description |
|------|------|-------------|
| Package | [event-camera-simulator](Packages/event-camera-simulator/) | ESIM-based event camera (DVS) simulator for MATLAB and Simulink with noise model and visualization |
| Package | [video-framerate-upsampling](Packages/video-framerate-upsampling/) | Increases video frame rate using optical flow (RAFT/Farneback) frame interpolation |
| Example | [battery-fast-charging-optimization](Examples/battery-fast-charging-optimization/) | Compare CC-CV, multi-stage, and fmincon-optimized charging strategies for a lithium-ion battery (SPM) with interactive app |
| Example | [shape-from-shading-asteroids](Examples/shape-from-shading-asteroids/) | Enhance stereo depth maps of asteroid Bennu using Shape from Shading |
| Example | [spot-sim3D](Examples/spot-sim3D/) | Keyboard-controlled quadruped walking with Simscape&trade; Multibody&trade; and Unreal Engine&reg; 3D visualization |

## License

Licensed under the MathWorks BSD-3-Clause License. See [LICENSE](LICENSE).
