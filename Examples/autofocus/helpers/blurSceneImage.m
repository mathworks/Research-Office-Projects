function blurred = blurSceneImage(rgb, depth, focusDistance)
%blurSceneImage Defocus one AutofocusScene frame at a given focus distance.
%   blurred = blurSceneImage(rgb, depth, focusDistance) applies simulateDefocusBlur
%   to one frame of the camera output, focused at focusDistance metres.

arguments
    rgb (:,:,3) {mustBeNumeric, mustBeReal}
    depth (:,:) {mustBeNumeric, mustBeReal}
    focusDistance (1,1) double {mustBePositive} = 4.5
end

cameraHorizontalFov = 60;   % degrees, set by the Simulation 3D Camera block

params = defocusBlurParams();
params.FocusDistance = focusDistance;
params.FNumber = 0.28;
params.FocalLengthPixels = (size(rgb, 2)/2)/tand(cameraHorizontalFov/2);
blurred = simulateDefocusBlur(rgb, double(depth), params);
end
