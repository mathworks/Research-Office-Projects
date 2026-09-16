function form = formationFromMedia(source, varargin)
%FORMATIONFROMMEDIA Turn a picture, an STL model or a mask into a drone formation.
%
%   form = formationFromMedia(filePath) reads a 2D image (.png/.jpg/.bmp/.tif/.gif)
%   or a 3D mesh (.stl) and returns a candidate point cloud the show can place
%   drones on.
%
%   form = formationFromMedia(mask) takes a logical mask the caller rendered
%   itself instead of a file. formationFromText uses this to push glyphs through
%   exactly the same reduction and normalisation a picture goes through, so text
%   is not a second copy of this code.
%
%   form = formationFromMedia(..., 'Reduce', 'centreline') traces the STROKE
%   CENTRELINES instead of the silhouette outline. Right for anything made of
%   thick strokes -- lettering above all -- where the outline is two parallel
%   curves per stroke and a modest fleet sampled off it lands on both sides of
%   every stroke instead of along it. See the note on cloudsFromMask.
%
%   Returns a struct:
%     .name       display name, from the file name
%     .kind       'image', 'stl' or 'mask'
%     .cloud      [M x 3] candidate points, the PREFERRED sampling
%     .cloudFill  [M2 x 3] fallback for images (the filled silhouette); empty for STL
%     .source     the file it came from
%
%   A CLOUD, NOT N POINTS. The fleet size is not known here and changes freely in
%   the app, so this returns a few thousand candidates in a normalised frame and
%   sampleFormationCloud picks exactly N_uav of them later. Re-sampling on N is
%   what makes the UAV spinner keep working after a shape is loaded; baking in N
%   would silently strand the formation at whatever count was set at load time.
%
%   FRAME. x = East, y = North, z = Up (metres), centred on the bounding box and
%   scaled so the largest half-extent is 1. setupParams applies the real scale
%   (formation_spacing, d_min) and converts to NED, because only it knows those.
%
%   2D pictures are VERTICAL BILLBOARDS: image columns map to East and image rows
%   to Up, with y = 0 for every point. A picture laid flat at show altitude is only
%   readable from directly overhead, which is not where the audience is.
%
%   DETERMINISM. Sampling uses a private RandStream with a fixed seed rather than
%   the global stream, so two calls on the same file give the same formation and
%   nothing else in the session gets its rng reseeded underneath it.
%
%   See also FORMATIONFROMTEXT, SAMPLEFORMATIONCLOUD, SETUPPARAMS.

%% ---- options ----
p = inputParser;
p.addParameter('MaxPoints', 4000, @(x) isnumeric(x) && isscalar(x) && x >= 100);
p.addParameter('Name', '', @(x) ischar(x) || isstring(x));
p.addParameter('Reduce', 'outline', @(x) any(strcmpi(x, {'outline', 'centreline'})));
p.parse(varargin{:});
maxPoints = p.Results.MaxPoints;
reduce = lower(char(p.Results.Reduce));

stream = RandStream('twister', 'Seed', 20260830);
name = char(p.Results.Name);

if islogical(source) || (isnumeric(source) && ~isvector(source))
    % A mask the caller rendered. No file, no thresholding: it is already the
    % silhouette, so it goes straight into the shared reduction.
    [cloud, cloudFill] = cloudsFromMask(logical(source), reduce, maxPoints, ...
        stream, 'the supplied mask');
    kind = 'mask';
    if isempty(name), name = 'Custom'; end
    form = struct('name', name, 'kind', kind, 'cloud', cloud, ...
        'cloudFill', cloudFill, ...
        'source', sprintf('%dx%d mask', size(source, 1), size(source, 2)));
    return;
end

filePath = source;
if ~isfile(filePath)
    error('formationFromMedia:fileNotFound', 'No such file: %s', filePath);
end
[~, baseName, ext] = fileparts(filePath);
ext = lower(ext);

if isempty(name)
    % A formation name ends up in a dropdown string that parseFormationString
    % splits on arrows, so strip anything that would confuse it.
    name = regexprep(baseName, '[^A-Za-z0-9]', '');
    if isempty(name), name = 'Custom'; end
    name = [upper(name(1)) name(2:end)];
end

switch ext
    case '.stl'
        cloud = stlCloud(filePath, maxPoints, stream);
        cloudFill = zeros(0, 3);
        kind = 'stl';
    case {'.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff', '.gif'}
        [cloud, cloudFill] = imageCloud(filePath, reduce, maxPoints, stream);
        kind = 'image';
    otherwise
        error('formationFromMedia:unsupported', ...
            ['"%s" is not a supported format. Use an image ' ...
             '(.png .jpg .bmp .tif .gif) or a 3D mesh (.stl).'], ext);
end

form = struct('name', name, 'kind', kind, 'cloud', cloud, ...
    'cloudFill', cloudFill, 'source', char(filePath));
end

%% ------------------------------------------------------------------------
function [outline, filled] = imageCloud(filePath, reduce, maxPoints, stream)
% Threshold the picture to a silhouette, then hand it to the shared reduction.

[img, ~, alpha] = imread(filePath);

if ~isempty(alpha)
    % A transparent PNG already carries the silhouette its author intended, and it
    % beats any threshold we could guess at.
    mask = double(alpha) > 0.5 * double(max(alpha(:)));
else
    g = im2double(im2gray(img));
    bw = imbinarize(g);
    % Which class is the subject? Whichever one does NOT dominate the border: a
    % photo of a dark logo on white and a white logo on black both have to work,
    % and assuming one polarity gets the negative of the shape half the time.
    border = [bw(1, :), bw(end, :), bw(:, 1)', bw(:, end)'];
    if mean(border) > 0.5
        mask = ~bw;      % bright border => background is the true class
    else
        mask = bw;
    end
end

[outline, filled] = cloudsFromMask(mask, reduce, maxPoints, stream, filePath);
end

%% ------------------------------------------------------------------------
function [preferred, filled] = cloudsFromMask(mask, reduce, maxPoints, stream, label)
% Silhouette -> a preferred cloud and a filled fallback. The one place a mask
% becomes points, whether it came from a file or from rendered text.
%
% The fallback exists because a thin or tiny shape can have fewer edge pixels than
% there are drones; sampleFormationCloud switches to the fill when that happens,
% rather than stacking two drones on one pixel.

% Drop specks: JPEG ringing and stray pixels would otherwise get a drone each,
% and a drone parked on a compression artefact is indistinguishable from a bug.
minBlob = max(4, round(0.00002 * numel(mask)));
mask = bwareaopen(mask, minBlob);
if ~any(mask(:))
    error('formationFromMedia:emptyImage', ...
        ['Thresholding "%s" left nothing to fly. Try an image with a clear ' ...
         'silhouette against a plain background.'], label);
end

% Either way, the reduction runs on the mask AS THRESHOLDED with no imfill first.
% Filling holes closes the counter of a letter "A" and the middle of a ring, and
% those interior edges are most of what makes a logo or a word readable in the air
% -- an imfill'd annulus flies as a single circle of drones, which is not the shape
% that was loaded.
if strcmp(reduce, 'centreline')
    % bwmorph 'thin' rather than bwskel. bwskel's output depends on how much blank
    % space surrounds the shape: on an uncropped canvas a narrow bar -- a capital
    % I -- came back as a SINGLE pixel, so the letter vanished and its drones went
    % to the neighbouring glyphs. thin returns the same 81-pixel medial line for
    % that bar whether it is cropped tight or sitting in a large canvas.
    preferred = maskToCloud(bwmorph(mask, 'thin', Inf), maxPoints, stream);
else
    preferred = maskToCloud(bwperim(mask), maxPoints, stream);
end
filled = maskToCloud(mask, maxPoints, stream);

% Normalise both against the SAME extent, so falling back to the fill does not
% quietly resize the formation relative to the preferred cloud.
[preferred, s, c] = normaliseCloud(preferred);
filled = (filled - c) / s;
end

%% ------------------------------------------------------------------------
function pts = maskToCloud(mask, maxPoints, stream)
% Pixel indices to billboard coordinates: column -> East, row -> Up. Image rows
% count downwards, so the row index is negated or the picture flies upside down.
[r, c] = find(mask);
pts = [c, zeros(numel(c), 1), -r];

if size(pts, 1) > maxPoints
    % Thin out evenly rather than cropping a region: a stride keeps the whole
    % shape, and a random subset of a 4000-point budget keeps it unbiased.
    keep = datasample(stream, 1:size(pts, 1), maxPoints, 'Replace', false);
    pts = pts(sort(keep), :);
end
end

%% ------------------------------------------------------------------------
function pts = stlCloud(filePath, maxPoints, stream)
% Area-weighted samples across the triangle faces, then a grid-average reduction
% (Computer Vision Toolbox) so the points are evenly spread over the surface.
%
% Sampling faces by AREA is the whole point: mesh tessellation is uneven, so
% k-means over raw vertices bunches drones wherever the modeller happened to add
% detail — a low-poly cube gives eight corner clusters and nothing along an edge.

tr = stlread(filePath);
P = tr.Points;
F = tr.ConnectivityList;
if isempty(F)
    error('formationFromMedia:emptyMesh', '"%s" contains no faces.', filePath);
end

v1 = P(F(:, 1), :);
v2 = P(F(:, 2), :);
v3 = P(F(:, 3), :);
areas = 0.5 * vecnorm(cross(v2 - v1, v3 - v1, 2), 2, 2);
areas(~isfinite(areas) | areas <= 0) = eps;

nSamp = min(40000, max(8000, 20 * maxPoints));
fi = datasample(stream, 1:size(F, 1), nSamp, 'Weights', areas);
fi = fi(:);

% Uniform barycentric coordinates on a triangle: draw in the unit square and
% reflect the half that falls outside, which is exact rather than approximate.
u = rand(stream, nSamp, 1);
v = rand(stream, nSamp, 1);
over = (u + v) > 1;
u(over) = 1 - u(over);
v(over) = 1 - v(over);
samples = v1(fi, :) + u .* (v2(fi, :) - v1(fi, :)) + v .* (v3(fi, :) - v1(fi, :));

% Grid-average down to a manageable cloud. The grid step is set from the model's
% own size so it works on a 3 mm part and a 30 m building alike.
span = max(range(samples, 1));
if span <= 0
    error('formationFromMedia:degenerateMesh', ...
        '"%s" has no extent — every vertex is at the same point.', filePath);
end
pc = pcdownsample(pointCloud(samples), 'gridAverage', span / 45);
pts = double(pc.Location);

if size(pts, 1) > maxPoints
    keep = datasample(stream, 1:size(pts, 1), maxPoints, 'Replace', false);
    pts = pts(sort(keep), :);
end

pts = normaliseCloud(pts);
end

%% ------------------------------------------------------------------------
function [pts, s, c] = normaliseCloud(pts)
% Centre on the bounding box and scale so the largest half-extent is 1. Aspect
% ratio is preserved — a tall thin shape must stay tall and thin — so only the
% dominant axis reaches +-1.
lo = min(pts, [], 1);
hi = max(pts, [], 1);
c = (lo + hi) / 2;
s = max((hi - lo) / 2);
if s <= 0, s = 1; end
pts = (pts - c) / s;
end
