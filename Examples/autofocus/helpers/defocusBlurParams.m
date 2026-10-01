function params = defocusBlurParams()
%defocusBlurParams Default parameters for simulateDefocusBlur.
%   params = defocusBlurParams() returns a struct of thin-lens and rendering
%   parameters with defaults that match the camera in AutofocusScene.slx: a
%   640-by-480 image with a focal length of 554 pixels.
%
%   Optical parameters:
%       FocalLength       - Focal length in metres
%       FNumber           - Aperture f-number, f/N. Smaller values blur more
%       FocusDistance     - Distance of the plane in focus, in metres
%       FocalLengthPixels - Focal length in pixels, from the camera intrinsics
%
%   FocalLengthPixels is fixed by the render: it is the field of view the scene
%   was drawn with, and changing it would mean re-rendering. FocalLength is then
%   a statement about sensor size, because the pixel pitch it implies is
%   FocalLength/FocalLengthPixels. The defaults below give a 90 um pitch, so a
%   640-by-480 frame covers 58 by 43 mm: a 50 mm lens on a medium-format sensor.
%
%   How much blur to expect. For a distant surface the radius tends to
%
%       rMax = FocalLengthPixels*FocalLength/(2*FNumber*(FocusDistance - FocalLength))
%
%   pixels, which is about 2.5 px for the defaults. Written in terms of the
%   aperture diameter A = FocalLength/FNumber this is just
%   FocalLengthPixels*A/(2*FocusDistance), so blur grows only as you open the
%   aperture, focus closer, or render more pixels across the same field of view.
%   Reaching a 20 px radius at a 4 m focus distance needs A near 0.29 m. A
%   small-sensor wide-angle camera, say 12 mm at f/2.8, has A = 4 mm and is
%   nearly a pinhole at this resolution, so it correctly produces almost no blur.
%
%   Rendering parameters:
%       MaxDepth          - Depth at or beyond which a pixel counts as sky, in metres
%       NumLayers         - Number of depth layers used to resolve occlusion
%       MaxCoCRadius      - Upper bound on the blur radius, in pixels
%       ApertureBlades    - 0 for a circular aperture, or >= 3 for a polygonal one
%       Gamma             - Transfer function exponent of the image. 1 blurs the
%                           stored values directly, which is physically wrong
%   See also simulateDefocusBlur.

params = struct( ...
    "FocalLength", 0.050, ...
    "FNumber", 1.4, ...
    "FocusDistance", 4, ...
    "FocalLengthPixels", 554, ...
    "MaxDepth", 1000, ...
    "NumLayers", 16, ...
    "MaxCoCRadius", 32, ...
    "ApertureBlades", 0, ...
    "Gamma", 2);
end
