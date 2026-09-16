function form = formationFromText(str, varargin)
%FORMATIONFROMTEXT Spell a word out in drones.
%
%   form = formationFromText('HELLO') renders the string with a real font
%   (insertText, Computer Vision Toolbox), traces the stroke centrelines, and
%   returns the same candidate-point-cloud struct formationFromMedia returns. It
%   therefore drops into the show exactly like a loaded picture does: the app adds
%   it to the formation list, setupParams scales and places it, and the fleet size is
%   resolved at plan time.
%
%   form = formationFromText('HAPPY|BIRTHDAY') breaks the text into two lines. A
%   newline in the string does the same thing. WORTH USING on anything long: a
%   single line of 9 characters is a 9:1 letterbox, and once the cloud is
%   normalised there is almost no height left to put drones in. Two lines of 5 read
%   at half the fleet.
%
%   Name-value options:
%     'FontSize'  glyph size in pixels for the render (default 140). This is
%                 rendering resolution only -- the flown size comes from
%                 formation_spacing and d_min in setupParams, not from here.
%     'Font'      TrueType font name. The default picks the first available of a
%                 bold list: a bold face is thicker, so thinning it gives a
%                 cleaner centreline than a hairline face does.
%     'Name'      display name (default: the text, stripped to alphanumerics).
%     'MaxPoints' candidate cloud budget, passed through (default 4000).
%
%   HOW MANY DRONES A WORD NEEDS. About 10 per character before it reads as
%   letters rather than as a scatter; below that there are not enough drones to
%   trace a glyph, whatever the cloud. Measured rather than guessed: "MATLAB" is
%   illegible at 40 drones (6.7 each) and clear at 100 (16.7 each).
%   setupParams says so when a show asks for less.
%
%   CENTRELINES, NOT THE OUTLINE. A glyph is thick strokes, so its outline is two
%   parallel curves per stroke, and drones sampled off it sit on both sides of
%   every stroke instead of along it -- legible as a blob, not as a word. This asks
%   formationFromMedia for the 'centreline' reduction, which is why that option
%   exists.
%
%   See also FORMATIONFROMMEDIA, SAMPLEFORMATIONCLOUD, SETUPPARAMS, INSERTTEXT.

%% ---- options ----
p = inputParser;
p.addParameter('FontSize', 140, @(x) isnumeric(x) && isscalar(x) && x >= 24);
p.addParameter('Font', '', @(x) ischar(x) || isstring(x));
p.addParameter('Name', '', @(x) ischar(x) || isstring(x));
p.addParameter('MaxPoints', 4000, @(x) isnumeric(x) && isscalar(x) && x >= 100);
p.parse(varargin{:});
fontSize = double(p.Results.FontSize);

lines = splitLines(str);
if isempty(lines)
    error('formationFromText:emptyText', ...
        'There is no text to fly -- type something first.');
end
if max(cellfun(@numel, lines)) > 40
    error('formationFromText:tooLong', ...
        ['A line of %d characters is far wider than a fleet can spell. Keep ' ...
         'lines to 40 characters or fewer, and split long text with "|".'], ...
        max(cellfun(@numel, lines)));
end

name = char(p.Results.Name);
if isempty(name)
    % The name becomes an item in a dropdown that parseFormationString splits on
    % arrows, so only alphanumerics survive: "HAPPY|BIRTHDAY" is displayed as
    % "HAPPYBIRTHDAY" even though the glyphs keep the line break.
    name = regexprep(strjoin(lines, ''), '[^A-Za-z0-9]', '');
    if isempty(name), name = 'Text'; end
    if numel(name) > 14, name = name(1:14); end
end

mask = renderMask(lines, fontSize, char(p.Results.Font));

% Everything from here -- speck removal, thinning, the filled fallback,
% normalisation to a half-extent of 1 -- is the picture path, unchanged.
form = formationFromMedia(mask, 'Name', name, 'Reduce', 'centreline', ...
    'MaxPoints', p.Results.MaxPoints);
form.kind = 'text';
form.source = ['text: ' strjoin(lines, ' | ')];
end

%% ------------------------------------------------------------------------
function lines = splitLines(str)
% "|" or a real newline starts a new line. Blank lines are dropped rather than
% flown as a gap, and trailing whitespace would otherwise widen the canvas for
% nothing.
str = char(string(str));
parts = strsplit(str, {'|', newline, char(13)});
parts = strtrim(parts);
lines = parts(~cellfun(@isempty, parts));
end

%% ------------------------------------------------------------------------
function mask = renderMask(lines, fontSize, fontName)
% Black text on a white canvas, thresholded to a silhouette and cropped to the ink.
%
% The canvas is deliberately oversized. insertText CLIPS at the canvas edge rather
% than growing it, and a clipped glyph is a formation with a letter missing, so the
% width allows for the widest line plus margin and the crop throws the slack away.

nLines = numel(lines);
widest = max(cellfun(@numel, lines));
lineStep = round(fontSize * 1.45);          % baseline pitch, a little leading
w = round(fontSize * (widest + 2) * 1.1);
h = lineStep * nLines + 2 * fontSize;
canvas = 255 * ones(h, w, 'uint8');

% One insertText call for the whole block: each line centred on the canvas, so
% lines of different lengths stack centred rather than ragged-left.
pos = [repmat(round(w / 2), nLines, 1), ...
       round(fontSize * 0.5) + (0:nLines-1)' * lineStep];
args = {'FontSize', round(fontSize), 'TextColor', 'black', 'BoxOpacity', 0, ...
        'AnchorPoint', 'CenterTop'};
fontName = pickFont(fontName);
if ~isempty(fontName)
    args = [args, {'Font', fontName}];
end
img = insertText(canvas, pos, lines, args{:});

mask = ~imbinarize(im2gray(img));
if ~any(mask(:))
    error('formationFromText:renderFailed', ...
        'Rendering "%s" produced no ink.', strjoin(lines, ' '));
end

% Crop to the ink. Not required for correctness -- normalising centres on the
% bounding box either way -- but it keeps the speck filter in cloudsFromMask
% honest: its threshold is a fraction of numel(mask), so a big empty canvas would
% raise the bar high enough to delete the dot of an "i".
[r, c] = find(mask);
mask = mask(min(r):max(r), min(c):max(c));
end

%% ------------------------------------------------------------------------
function fontName = pickFont(requested)
% Resolve a font that actually exists on this machine. A missing font is an error
% from insertText, not a substitution, so a hard-coded name would make the whole
% feature machine-dependent.
available = listTrueTypeFonts();
if ~isempty(requested)
    hit = find(strcmpi(requested, available), 1);
    if isempty(hit)
        error('formationFromText:noSuchFont', ...
            '"%s" is not installed. listTrueTypeFonts shows what is.', requested);
    end
    fontName = available{hit};
    return;
end
% Bold first: thinning a bold face gives a clean single centreline per stroke,
% while a hairline face is already close to its own skeleton and thins to a
% ragged, broken one.
for want = {'Arial Bold', 'Helvetica Bold', 'Verdana Bold', 'Tahoma Bold', ...
            'DejaVu Sans Bold', 'Arial', 'Helvetica', 'Verdana'}
    hit = find(strcmpi(want{1}, available), 1);
    if ~isempty(hit)
        fontName = available{hit};
        return;
    end
end
fontName = '';      % none of them: let insertText use its own default
end
