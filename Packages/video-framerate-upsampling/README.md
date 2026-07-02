# Increase Video Frame Rate Using Optical Flow

This example shows how to increase a video's frame rate using optical flow frame interpolation. A video is subsampled to simulate a jerky low-frame-rate input, then new intermediate frames are interpolated using RAFT optical flow to restore smooth motion.

MATLAB&reg; is required along with Computer Vision Toolbox&trade;.

![Side-by-side comparison](comparison.gif)

## Requirements

- MATLAB&reg; R2024b or later
- [Computer Vision Toolbox&trade;](https://www.mathworks.com/products/computer-vision.html)
- [Computer Vision Toolbox&trade; Model for RAFT Optical Flow](https://www.mathworks.com/matlabcentral/fileexchange/128031-computer-vision-toolbox-model-for-raft-optical-flow) (support package)

## Getting Started

Run the example script:

```matlab
IncreaseVideoFrameRateUsingOpticalFlow
```

Or use `temporalUpsample` directly on your own video:

```matlab
% Interpolate frames to 4x the original frame rate
output = temporalUpsample(frames, 4);

% Interpolate using Farneback (no support package needed)
output = temporalUpsample("myVideo.mp4", 2, Method="farneback");

% Use GPU acceleration
output = temporalUpsample(frames, 3, ExecutionEnvironment="GPU");
```

## How It Works

The `temporalUpsample` function performs frame interpolation by:

1. Computing bidirectional optical flow between each pair of consecutive frames using RAFT (or Farneback)
2. Forward-splatting pixels from both neighboring frames to each intermediate time position
3. Using exponential importance weighting so moving objects dominate over static background at contested destinations
4. Blending the two contributions with occlusion-aware temporal weighting
5. Filling any remaining holes with local median filtering

## Files

| File | Description |
|------|-------------|
| `temporalUpsample.m` | Main function — interpolate frames to increase video frame rate |
| `IncreaseVideoFrameRateUsingOpticalFlow.m` | Example script with side-by-side comparison |

## References

This implementation uses techniques from the following papers:

- Niklaus, S. and Liu, F. "Softmax Splatting for Video Frame Interpolation." *CVPR*, 2020. (forward splatting with importance weighting)
- Teed, Z. and Deng, J. "RAFT: Recurrent All-Pairs Field Transforms for Optical Flow." *ECCV*, 2020. (optical flow estimation)

## License

Copyright 2025 The MathWorks, Inc. See [license.txt](license.txt) for details.
