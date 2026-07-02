function outputVideo = temporalUpsample(inputVideo, upsampleFactor, options)
% temporalUpsample Increase video frame rate using optical flow frame interpolation.
%   outputVideo = temporalUpsample(inputVideo, upsampleFactor) interpolates
%   new frames to increase a video's frame rate by the specified integer
%   factor using RAFT optical flow for motion estimation.
%
%   outputVideo = temporalUpsample(___,Method=method) specifies the optical
%   flow algorithm: "raft" (default) or "farneback".
%
%   outputVideo = temporalUpsample(___,ExecutionEnvironment=env) specifies
%   the hardware for optical flow computation: "auto" (default), "CPU", or
%   "GPU". Only applies when Method is "raft".
%
%   Inputs:
%       inputVideo      - Video as H-by-W-by-C-by-N numeric array (RGB or
%                         grayscale), or a path to a video file (string/char).
%       upsampleFactor  - Integer upsampling factor (e.g., 2 doubles the
%                         frame rate).
%
%   Output:
%       outputVideo     - Upsampled video as H-by-W-by-C-by-M array where
%                         M = (N-1)*upsampleFactor + 1.
%
%   Example:
%       % Upsample a video file by 4x
%       output = temporalUpsample("myVideo.mp4", 4);
%
%       % Upsample a numeric array by 2x on GPU
%       output = temporalUpsample(frames, 2, ExecutionEnvironment="GPU");
%
%       % Use Farneback method (no support package required)
%       output = temporalUpsample(frames, 2, Method="farneback");

    arguments
        inputVideo
        upsampleFactor (1,1) {mustBeInteger, mustBeGreaterThan(upsampleFactor,1)}
        options.Method (1,1) string {mustBeMember(options.Method,["raft","farneback"])} = "raft"
        options.ExecutionEnvironment (1,1) string {mustBeMember(options.ExecutionEnvironment,["auto","CPU","GPU"])} = "auto"
        options.Verbose (1,1) logical = false
    end

    % Load video if a file path is provided
    if ischar(inputVideo) || isstring(inputVideo)
        inputVideo = readVideoFile(inputVideo);
    end

    validateattributes(inputVideo, {'uint8','single','double'}, {'ndims',4});

    [H, W, C, N] = size(inputVideo);
    if N < 2
        error("temporalUpsample:TooFewFrames", ...
            "Input video must have at least 2 frames.");
    end

    % Convert to single for processing
    wasUint8 = isa(inputVideo, 'uint8');
    if wasUint8
        frames = im2single(inputVideo);
    else
        frames = single(inputVideo);
    end

    % Initialize optical flow model
    useRAFT = options.Method == "raft";
    if useRAFT
        flowModel = opticalFlowRAFT;
    else
        flowModel = opticalFlowFarneback;
    end

    % Preallocate output: (N-1)*factor + 1 frames
    numOut = (N - 1) * upsampleFactor + 1;
    output = zeros(H, W, C, numOut, 'single');

    % Process each pair of consecutive frames
    for i = 1:N-1
        if options.Verbose
            fprintf("Processing frames %d-%d of %d\n", i, i+1, N);
        end
        frame1 = frames(:,:,:,i);
        frame2 = frames(:,:,:,i+1);

        % Prepare inputs based on method
        if useRAFT
            input1 = frame1;
            input2 = frame2;
        else
            input1 = rgb2gray(frame1);
            input2 = rgb2gray(frame2);
        end

        % Compute forward flow (frame1 -> frame2)
        reset(flowModel);
        if useRAFT
            estimateFlow(flowModel, input1, ExecutionEnvironment=options.ExecutionEnvironment);
            flowFwd = estimateFlow(flowModel, input2, ExecutionEnvironment=options.ExecutionEnvironment);
        else
            estimateFlow(flowModel, input1);
            flowFwd = estimateFlow(flowModel, input2);
        end

        % Compute backward flow (frame2 -> frame1)
        reset(flowModel);
        if useRAFT
            estimateFlow(flowModel, input2, ExecutionEnvironment=options.ExecutionEnvironment);
            flowBwd = estimateFlow(flowModel, input1, ExecutionEnvironment=options.ExecutionEnvironment);
        else
            estimateFlow(flowModel, input2);
            flowBwd = estimateFlow(flowModel, input1);
        end

        % Place the original frame
        outIdx = (i - 1) * upsampleFactor + 1;
        output(:,:,:,outIdx) = frame1;

        % Generate intermediate frames
        [X, Y] = meshgrid(1:W, 1:H);

        % Compute importance metric using brightness constancy error.
        % Low error = flow is reliable = high importance. Pixels with
        % unreliable flow (occlusions) get downweighted.
        % flowBwd at frame1 points to corresponding position in frame2,
        % flowFwd at frame2 points to corresponding position in frame1.
        alpha = 20.0;
        warped2to1 = backwardWarp(frame2, flowBwd.Vx, flowBwd.Vy, X, Y);
        warped1to2 = backwardWarp(frame1, flowFwd.Vx, flowFwd.Vy, X, Y);
        err1 = mean(abs(frame1 - warped2to1), 3);
        err2 = mean(abs(frame2 - warped1to2), 3);
        importance1 = -alpha * err1;
        importance2 = -alpha * err2;

        for k = 1:upsampleFactor-1
            t = k / upsampleFactor; % interpolation parameter in (0,1)

            interpFrame = warpIntermediate(frame1, frame2, flowFwd, flowBwd, t, X, Y, importance1, importance2);
            output(:,:,:,outIdx + k) = interpFrame;
        end
    end

    % Place the last original frame
    output(:,:,:,numOut) = frames(:,:,:,N);

    % Convert back to original data type
    if wasUint8
        outputVideo = im2uint8(output);
    else
        outputVideo = output;
    end
end

function frames = readVideoFile(filePath)
    v = VideoReader(filePath);
    frames = read(v);
end

function result = warpIntermediate(frame1, frame2, flowFwd, flowBwd, t, X, Y, importance1, importance2)
% Synthesize intermediate frame at time t in [0,1] between frame1 and frame2
% using forward splatting with occlusion-aware blending.
%
% Forward splatting avoids the grid-mismatch problem of backward warping:
% each source pixel is placed at its correct intermediate position rather
% than sampling flow at potentially wrong grid locations.

    [H, W, ~] = size(frame1);

    % Forward splat from frame1 toward intermediate time t.
    % MATLAB flow convention: previous_pos = current_pos - flow.
    % flowBwd evaluated at frame1 gives motion from frame1 to frame2 as
    % -flowBwd. Intermediate position = pixel_pos + t * (-flowBwd).
    destX1 = X - t * flowBwd.Vx;
    destY1 = Y - t * flowBwd.Vy;

    % Forward splat from frame2 toward intermediate time t.
    % flowFwd evaluated at frame2 gives motion from frame2 to frame1 as
    % -flowFwd. A pixel at (x,y) in frame2 was at (x - Vx_fwd, y - Vy_fwd)
    % in frame1, so its position at time t is x - (1-t)*Vx_fwd.
    destX2 = X - (1 - t) * flowFwd.Vx;
    destY2 = Y - (1 - t) * flowFwd.Vy;

    % Forward splat with importance weighting. Pixels with reliable flow
    % (low reconstruction error) dominate at contested destinations.
    [warped1, weight1] = forwardSplat(frame1, destX1, destY1, H, W, importance1);
    [warped2, weight2] = forwardSplat(frame2, destX2, destY2, H, W, importance2);

    % Occlusion-aware blending based on splat confidence.
    % Regions with low weight received few contributions — likely occluded.
    reliable1 = weight1 > 0.1;
    reliable2 = weight2 > 0.1;
    holeMask = ~reliable1 & ~reliable2;

    % Temporal distance weighting (closer frame gets more weight)
    w1 = (1 - t) * single(reliable1);
    w2 = t * single(reliable2);

    % Fallback for regions where neither splat is reliable
    w1(holeMask) = 1 - t;
    w2(holeMask) = t;

    % Normalize and blend
    wSum = w1 + w2;
    wSum(wSum == 0) = 1;
    result = (w1 ./ wSum) .* warped1 + (w2 ./ wSum) .* warped2;

    % Fill remaining holes with local median filtering
    if any(holeMask, "all")
        for c = 1:size(result, 3)
            chan = result(:,:,c);
            filled = medfilt2(chan, [5 5]);
            chan(holeMask) = filled(holeMask);
            result(:,:,c) = chan;
        end
    end
end

function [splatted, weights] = forwardSplat(frame, destX, destY, H, W, importance)
% Forward-splat frame pixels to destination positions using bilinear
% distribution with softmax importance weighting. Pixels with higher
% importance (lower reconstruction error) dominate when multiple pixels
% compete for the same destination.

    C = size(frame, 3);
    splatted = zeros(H, W, C, 'single');
    weights = zeros(H, W, 'single');

    % Softmax importance weighting: exp(importance) where importance is
    % pre-scaled (e.g., -alpha * error). Reliable pixels dominate.
    imp = exp(importance);

    % Clamp destinations to valid range
    destX = max(1, min(W, destX));
    destY = max(1, min(H, destY));

    % Bilinear splatting: distribute each pixel to 4 neighbors
    x0 = floor(destX);
    y0 = floor(destY);
    x1 = x0 + 1;
    y1 = y0 + 1;

    wx1 = destX - x0;
    wy1 = destY - y0;
    wx0 = 1 - wx1;
    wy0 = 1 - wy1;

    % Four corner weights multiplied by importance
    w00 = wx0 .* wy0 .* imp;
    w10 = wx1 .* wy0 .* imp;
    w01 = wx0 .* wy1 .* imp;
    w11 = wx1 .* wy1 .* imp;

    % Clamp indices
    x0 = max(1, min(W, x0));
    x1 = max(1, min(W, x1));
    y0 = max(1, min(H, y0));
    y1 = max(1, min(H, y1));

    % Accumulate weighted color
    for c = 1:C
        pixel = frame(:,:,c);
        splatC = accumarray([y0(:), x0(:)], w00(:) .* pixel(:), [H, W]) + ...
                 accumarray([y1(:), x0(:)], w01(:) .* pixel(:), [H, W]) + ...
                 accumarray([y0(:), x1(:)], w10(:) .* pixel(:), [H, W]) + ...
                 accumarray([y1(:), x1(:)], w11(:) .* pixel(:), [H, W]);
        splatted(:,:,c) = splatC;
    end

    % Accumulate weights
    weights = accumarray([y0(:), x0(:)], w00(:), [H, W]) + ...
              accumarray([y1(:), x0(:)], w01(:), [H, W]) + ...
              accumarray([y0(:), x1(:)], w10(:), [H, W]) + ...
              accumarray([y1(:), x1(:)], w11(:), [H, W]);

    % Normalize by accumulated weights
    validMask = weights > 0;
    for c = 1:C
        chan = splatted(:,:,c);
        chan(validMask) = chan(validMask) ./ weights(validMask);
        splatted(:,:,c) = chan;
    end
end

function warped = backwardWarp(frame, Vx, Vy, X, Y)
% Backward-warp frame using flow (Vx, Vy). MATLAB flow convention:
% previous_pos = current_pos - flow, so the source position for each
% destination pixel is (X - Vx, Y - Vy).
    mapX = X - Vx;
    mapY = Y - Vy;
    C = size(frame, 3);
    warped = zeros(size(frame), 'like', frame);
    for c = 1:C
        warped(:,:,c) = interp2(frame(:,:,c), mapX, mapY, "linear", 0);
    end
end
