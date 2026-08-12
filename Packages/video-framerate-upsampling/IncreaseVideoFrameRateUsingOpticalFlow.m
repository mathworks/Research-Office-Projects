%% Increase Video Frame Rate Using Optical Flow
% This example shows how to increase a video's frame rate using optical
% flow frame interpolation. A video is subsampled to simulate a jerky
% low-frame-rate input, then new intermediate frames are interpolated using
% RAFT optical flow to restore smooth motion.

%% Load Video and Subsample to Simulate Low Frame Rate
% Load the shuttle launch video, which has complex distributed motion from
% smoke plumes and the rising shuttle. Subsample every 6th frame to
% simulate a camera running at 5 fps instead of 30 fps.

v = VideoReader("shuttle.avi");
allFrames = read(v, [1 120]);

skipFactor = 6;
jerkyFrames = allFrames(:,:,:,1:skipFactor:end);

originalFps = v.FrameRate;
jerkyFps = originalFps / skipFactor;

fprintf("Original: %d frames at %d fps\n", size(allFrames,4), originalFps)
fprintf("Subsampled: %d frames (%d fps equivalent)\n", size(jerkyFrames,4), jerkyFps)

%% Interpolate Frames Using RAFT Optical Flow
% Use |temporalUpsample| to interpolate intermediate frames between each
% pair of subsampled frames. RAFT estimates dense per-pixel motion, and the
% function forward-splats each pixel to its correct intermediate position.

% Faster: Method = "farneback", high quality: Method = "raft"
upsampledFrames = temporalUpsample(jerkyFrames, skipFactor, ...
    Method="raft", Verbose=true);

fprintf("Upsampled: %d frames\n", size(upsampledFrames,4))

%% Play Side-by-Side Comparison
% Animate the jerky input (left) and upsampled result (right). The left
% panel holds each frame for several ticks to match real time, making the
% jerkiness visible against the smooth interpolation on the right.

numDisplay = min(size(upsampledFrames,4), size(allFrames,4));

figure(Position=[100 100 1320 400])

subplot(1,2,1)
img1 = imshow(jerkyFrames(:,:,:,1));
title("Input " + jerkyFps + " frames per second")

subplot(1,2,2)
img2 = imshow(upsampledFrames(:,:,:,1));
title("Interpolated to " + originalFps + " frames per second")

exportGif = true;
gifFile = "comparison.gif";

for i = 1:numDisplay
    jerkyIdx = min(ceil(i/skipFactor), size(jerkyFrames,4));
    img1.CData = jerkyFrames(:,:,:,jerkyIdx);
    img2.CData = upsampledFrames(:,:,:,i);
    drawnow
    pause(1/originalFps)

    if exportGif
        frame = getframe(gcf);
        [idx,cmap] = rgb2ind(frame.cdata, 256);
        if i == 1
            imwrite(idx, cmap, gifFile, "gif", LoopCount=Inf, DelayTime=1/originalFps);
        else
            imwrite(idx, cmap, gifFile, "gif", WriteMode="append", DelayTime=1/originalFps);
        end
    end
end
