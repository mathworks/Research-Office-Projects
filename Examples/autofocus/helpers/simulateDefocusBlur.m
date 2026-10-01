function [rgbBlurred, cocRadiusMap] = simulateDefocusBlur(rgb, depth, params)
%simulateDefocusBlur Render physically based defocus blur from an RGB image and a depth map.
%   rgbBlurred = simulateDefocusBlur(rgb, depth, params) blurs rgb according to
%   the per-pixel circle of confusion implied by depth and the thin-lens optics
%   in params, and returns an image of the same size and class as rgb. depth is
%   the distance to each pixel in metres, as produced by the Depth output of a
%   Simulation 3D Camera block.
%
%   [rgbBlurred, cocRadiusMap] = simulateDefocusBlur(...) also returns the blur
%   radius in pixels at every pixel, which is useful for checking the optics
%   independently of the rendering.
%
%   Optics. For a thin lens of focal length f focused at distance s, a surface at
%   distance z images to a circle of confusion whose diameter on the sensor is
%
%       C = A*f*|z - s|/((s - f)*z)      with aperture diameter A = f/N
%
%   for f-number N. Multiplying by the pixels-per-metre of the sensor, which is
%   fPixels/f, gives the diameter directly in pixels:
%
%       c = fPixels*f*|z - s|/(N*(s - f)*z)
%
%   Blur therefore responds correctly to aperture, focal length and focus
%   distance rather than to an arbitrary radius. Note that c is linear in the
%   inverse depth 1/z, which is what makes the layering below well conditioned.
%
%   Occlusion. Blurring a depth map in place is wrong wherever objects at
%   different distances meet, because it mixes foreground and background
%   radiance symmetrically. Instead the scene is split into NumLayers layers
%   that are uniform in inverse depth, so each layer spans an equal step in
%   circle of confusion. Every layer carries its own coverage mask, colour and
%   coverage are blurred together with the aperture kernel, and the layers are
%   composited from far to near with the premultiplied over operator
%
%       colour <- colourLayer + (1 - coverageLayer).*colour
%
%   A near, defocused layer therefore spreads over whatever is behind it, a
%   sharp foreground stays hard edged against a blurred background, and the
%   background is correctly seen through the soft edge of a blurred foreground.
%
%   Limitations. A single depth map records nothing about geometry hidden behind
%   a foreground object, yet that geometry is exactly what a real lens sees
%   around the edge of a defocused occluder. Where the composite ends up with
%   less than full coverage, the missing radiance is extrapolated from the
%   visible part of the same layer. This is the standard approximation; it is
%   accurate for small blur radii and degrades as the radius approaches the
%   width of the occluded region.
%   Example:
%       params = defocusBlurParams();
%       params.FNumber = 1.4;
%       params.FocusDistance = 6;
%       blurred = simulateDefocusBlur(rgb, depth, params);
%
%   See also defocusBlurParams, imfilter.

arguments
    rgb {mustBeNumeric, mustBeReal, mustBeNonempty}
    depth (:,:) {mustBeNumeric, mustBeReal, mustBeNonempty}
    params struct = defocusBlurParams()
end

params = mergeWithDefaults(params);
validateInputs(rgb, depth, params);

if isa(rgb, "gpuArray") || isa(depth, "gpuArray")
    rgb = gpuArray(rgb);
    depth = gpuArray(depth);
end

% Blurring is an average of light, so it must happen on numbers proportional to
% light. Doing it on the stored values darkens bokeh and loses the bright
% highlights that make defocus recognisable.
radiance = decodeToRadiance(rgb, params.Gamma);

% Anything past MaxDepth counts as sky, and so does anything that is not a
% distance at all. min does most of the work: MATLAB's min ignores NaN, so it
% maps NaN and Inf to MaxDepth along with the genuinely distant pixels. The
% Simulation 3D Camera reports sky as exactly 1000 m.
z = min(cast(depth, "like", radiance), params.MaxDepth);
z(z <= 0) = params.MaxDepth;

cocRadiusMap = circleOfConfusionRadius(z, params);

% Layer centres are uniform in inverse depth, hence uniform in blur radius.
invDepth = 1./z;
[layerCentres, layerSpacing] = depthLayerCentres(invDepth, params.NumLayers);
numLayers = numel(layerCentres);
layerRadius = circleOfConfusionRadius(1./layerCentres, params);

% Each pixel's continuous coordinate along the stack of layers. Layer l sits at
% coordinate l - 1, so a pixel's weight in it is the hat function
% max(0, 1 - |layerPosition - (l - 1)|), and the weights of the two layers a
% pixel falls between sum to one. Clamping to the span of the centres is what
% makes that pair cover the pixel completely, so no radiance is dropped at the
% extremes of depth. Holding one array rather than an index and a fraction keeps
% the per-layer weight to a single elementwise expression.
layerPosition = (min(max(invDepth, layerCentres(1)), layerCentres(end)) ...
    - layerCentres(1))/layerSpacing;

% Where each layer can live, resolved once and entirely on the host. Four
% whole-frame reductions give the span of layer coordinates present in every row
% and in every column, and intersecting those spans with a layer's support bounds
% its box. Gathering them here, rather than searching per layer, is what leaves
% the loop below free of any read back from the device.
rowPositionSpan = double(gather([min(layerPosition, [], 2) max(layerPosition, [], 2)]));
colPositionSpan = double(gather([min(layerPosition, [], 1)' max(layerPosition, [], 1)']));

accumColour = zeros(size(radiance), "like", radiance);
accumCoverage = zeros(size(z), "like", z);

% Far to near: layerCentres increases with inverse depth, so it decreases in z.
for layer = 1:numLayers
    kernel = apertureKernel(layerRadius(layer), params.ApertureBlades);
    halfWidth = (size(kernel, 1) - 1)/2;

    % A layer occupies one band of depth, which is typically a small part of the
    % frame, so convolve only its bounding box grown by the kernel reach. Outside
    % that box the layer contributes no colour and no coverage, so the composite
    % there is unchanged and the crop costs no accuracy at all.
    rows = spanToRange(rowPositionSpan, layer, halfWidth, size(z, 1));
    cols = spanToRange(colPositionSpan, layer, halfWidth, size(z, 2));
    if isempty(rows) || isempty(cols)
        continue
    end

    % The hat function described above, evaluated on the box alone.
    coverage = max(0, 1 - abs(layerPosition(rows, cols) - (layer - 1)));

    % The blur itself: colour premultiplied by coverage, and coverage on its own,
    % through the same kernel. Zero padding, not replicate, because beyond the box
    % the layer genuinely has no coverage. Where the box meets the edge of the
    % frame, the per-pixel normalization below accounts for the light that falls
    % outside.
    blurredColour = imfilter(radiance(rows, cols, :).*coverage, kernel, 0, "conv");
    blurredCoverage = imfilter(coverage, kernel, 0, "conv");

    accumColour(rows, cols, :) = blurredColour ...
        + (1 - blurredCoverage).*accumColour(rows, cols, :);
    accumCoverage(rows, cols) = blurredCoverage ...
        + (1 - blurredCoverage).*accumCoverage(rows, cols);
end

% Any shortfall in coverage is geometry the depth map could not describe, so
% spread the radiance that is available over the gap.
accumColour = accumColour./max(accumCoverage, 1e-6);

rgbBlurred = encodeFromRadiance(accumColour, rgb, params.Gamma);
end


function radiance = decodeToRadiance(rgb, gamma)
%decodeToRadiance Convert an image on [0 1] to numbers proportional to light.
%   A floating point image keeps its own class, so passing single is a request for
%   single precision that is honoured all the way to the output. An integer image
%   is scaled to double, there being no precision in it to preserve.
if isfloat(rgb)
    radiance = rgb;
else
    radiance = im2double(rgb);
end

radiance = radiance.^gamma;
end


function out = encodeFromRadiance(radiance, prototype, gamma)
%encodeFromRadiance Convert light back to stored values of the input's type.
%   Clamped before the power, because a fractional exponent of a negative number
%   is complex and coverage normalization can leave a hair below zero.
out = castLike(min(max(radiance, 0), 1).^(1/gamma), prototype);
end


function radiusPixels = circleOfConfusionRadius(z, params)
%circleOfConfusionRadius Thin-lens blur radius in pixels at distance z in metres.
diameterPixels = params.FocalLengthPixels*params.FocalLength*abs(z - params.FocusDistance) ...
    ./ (params.FNumber*(params.FocusDistance - params.FocalLength)*z);
radiusPixels = min(0.5*diameterPixels, params.MaxCoCRadius);
end


function [centres, spacing] = depthLayerCentres(invDepth, numLayers)
%depthLayerCentres Layer centres in inverse depth, and the spacing between them.
%   Returned on the host in double wherever invDepth lives. These few numbers set
%   the loop bounds and the kernel sizes, so the host needs them, and this pair of
%   reductions is the only place the render reads back from a device.
numLayers = max(1, round(numLayers));
lowest = double(gather(min(invDepth(:))));
highest = double(gather(max(invDepth(:))));
if numLayers == 1 || highest - lowest < eps(highest)
    centres = 0.5*(lowest + highest);
    spacing = 1;
    return
end
centres = linspace(lowest, highest, numLayers);
spacing = centres(2) - centres(1);
end


function range = spanToRange(positionSpan, layer, halfWidth, extent)
%spanToRange Rows or columns a layer can occupy, grown by the kernel reach.
%   positionSpan holds the lowest and highest layer coordinate found in each row,
%   or in each column. Layer l sits at coordinate l - 1 and its hat function is
%   non-zero only over (l - 2, l), so a line can hold part of the layer only if
%   its span reaches into that interval. Thresholding the spans gives a superset
%   of the layer's lines: never too small, and for depth that varies mainly down
%   the frame, barely too large. Returns empty when no line qualifies.
occupied = find(positionSpan(:,1) < layer & positionSpan(:,2) > layer - 2);
if isempty(occupied)
    range = [];
    return
end
range = max(1, occupied(1) - halfWidth):min(extent, occupied(end) + halfWidth);
end


function kernel = apertureKernel(radiusPixels, blades)
%apertureKernel Area-normalized point spread function of the aperture.
%   The kernel is the shape of the aperture itself: a disc for a perfectly round
%   iris, or a regular polygon for one with straight blades. A Gaussian is not
%   used because it has no hard edge, and the hard edge is what produces the
%   characteristic disc-shaped bokeh of an out-of-focus highlight.
%
%   Built on the host in double even when the image is single or on a device,
%   because imfilter requires a double filter. It is a few hundred elements, so
%   there is nothing to gain by placing it anywhere else.
radius = double(max(radiusPixels, 0));
if radius < 1
    % Rasterizing a disc this small rounds it to a single pixel, which claims a
    % quarter-pixel blur leaves the image untouched. It does not, so match the
    % aperture in the frequency domain instead - see subPixelKernel.
    kernel = subPixelKernel(radius);
    return
end

% Supersample so the rim of the aperture is antialiased. Without this the kernel
% gains or loses whole pixels as the radius grows, which shows up as ringing.
samplesPerSide = 4;
offsets = ((1:samplesPerSide) - 0.5)/samplesPerSide - 0.5;
halfWidth = ceil(radius);
[gridX, gridY] = meshgrid(-halfWidth:halfWidth);

coverage = zeros(size(gridX));
for dy = offsets
    for dx = offsets
        if blades >= 3
            coverage = coverage + insidePolygon(gridX + dx, gridY + dy, radius, blades);
        else
            coverage = coverage + ((gridX + dx).^2 + (gridY + dy).^2 <= radius^2);
        end
    end
end
kernel = coverage/sum(coverage(:));
end


function kernel = subPixelKernel(radius)
%subPixelKernel Kernel for blur narrower than a pixel, matched in the frequency domain.

x = pi*radius;                  % pi*d*f at Nyquist, f = 0.5 cycles/px, with d = 2*radius
jinc = 1 - x^2/8 + x^4/192 - x^6/9216 + x^8/737280;
a = min(max((1 - jinc)/4, 0), 0.25);
line = [a, 1 - 2*a, a];
kernel = line'*line;
end


function inside = insidePolygon(x, y, radius, blades)
%insidePolygon Test points against a regular polygon of the given circumradius.
apothem = radius*cos(pi/blades);
inside = true(size(x));
for edge = 0:blades - 1
    angle = 2*pi*edge/blades;
    inside = inside & (x*cos(angle) + y*sin(angle) <= apothem);
end
end


function out = castLike(unitRange, prototype)
%castLike Convert an image on [0 1] back to the underlying type of the input.
%   underlyingType rather than class, because the class of a gpuArray is
%   "gpuArray" and says nothing about the type the caller passed in.
switch string(underlyingType(prototype))
    case "uint8"
        out = im2uint8(unitRange);
    case "uint16"
        out = im2uint16(unitRange);
    case "int16"
        out = im2int16(unitRange);
    case "single"
        out = single(unitRange);
    otherwise
        out = double(unitRange);
end
end


function params = mergeWithDefaults(params)
%mergeWithDefaults Fill in any parameter the caller left out.
defaults = defocusBlurParams();
known = fieldnames(defaults);

unknown = setdiff(fieldnames(params), known);
if ~isempty(unknown)
    error("simulateDefocusBlur:UnknownParameter", ...
        "params has unrecognized field '%s'. Valid fields are: %s.", ...
        unknown{1}, strjoin(known', ", "));
end

for idx = 1:numel(known)
    if ~isfield(params, known{idx})
        params.(known{idx}) = defaults.(known{idx});
    end
end
end


function validateInputs(rgb, depth, params)
%validateInputs Check sizes and the physical consistency of the optics.
if size(rgb, 3) ~= 3
    error("simulateDefocusBlur:NotRGB", ...
        "rgb must have 3 channels in its third dimension. It has %d.", size(rgb, 3));
end
if ~isequal(size(depth), [size(rgb, 1) size(rgb, 2)])
    error("simulateDefocusBlur:SizeMismatch", ...
        "depth must be %d-by-%d to match rgb. It is %d-by-%d.", ...
        size(rgb, 1), size(rgb, 2), size(depth, 1), size(depth, 2));
end

mustBePositive(params.FocalLength);
mustBePositive(params.FNumber);
mustBePositive(params.FocusDistance);
mustBePositive(params.FocalLengthPixels);
mustBePositive(params.MaxDepth);
mustBePositive(params.NumLayers);
mustBePositive(params.Gamma);
mustBeNonnegative(params.MaxCoCRadius);

if params.FocusDistance <= params.FocalLength
    error("simulateDefocusBlur:FocusInsideFocalLength", ...
        "FocusDistance (%g m) must be greater than FocalLength (%g m) for the lens to form " + ...
        "a real image. Increase FocusDistance.", params.FocusDistance, params.FocalLength);
end
if params.ApertureBlades ~= 0 && params.ApertureBlades < 3
    error("simulateDefocusBlur:TooFewBlades", ...
        "ApertureBlades must be 0 for a circular aperture or at least 3 for a polygonal one. It is %g.", ...
        params.ApertureBlades);
end
end
