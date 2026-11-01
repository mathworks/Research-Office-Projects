function [q, queueFailure, overflow, boxes] = hmmValidateActions(raw, cfg)
%HMMVALIDATEACTIONS Schema, range and confidence gates on a provider response.
%
%   [q, queueFailure, overflow, boxes] = HMMVALIDATEACTIONS(raw, cfg) unwraps
%   the markdown fence, decodes the JSON, and applies the first five of the
%   twelve validation gates:
%
%       1  MalformedJSON   the payload is not the agreed schema
%       2  UnknownAction   a verb outside {transfer, navigate, home}
%       3  UnknownCamera   an *_image outside 1..cfg.camera.nFixed
%       4  OutOfRange      an *_point outside 0..cfg.camera.pixelNormScale
%       5  LowConfidence   below cfg.provider.confidenceMin
%
%   Six of the remaining seven need the scene and are applied by
%   hmmReconstructTargets, which is where depth and geometry live. The twelfth,
%   Unreachable, is raised by the motion-planning service: only inverse
%   kinematics can say that no admissible arm solution exists.
%
%   A REJECTED ACTION IS KEPT, marked, and counted. It is not deleted. The
%   queue reports the verb slot with its own ItemFailure so a run can be read
%   back as "the provider proposed four actions and the third was outside the
%   camera list", which is a diagnosis. Silently dropping it would leave a queue
%   of three with no record that anything was refused.
%
%   Gate 3 rejects camera 5. The wrist camera is never sent to the provider
%   (cfg.camera.nFixed is the upper bound, not nTotal), so a returned index of 5
%   means the model addressed an image it was never given.
%
%   THREE VERBS, AND THEY NEED DIFFERENT AMOUNTS OF GROUNDING:
%
%       transfer   two points and two camera indices - an object to pick up and
%                  a surface to set it down on
%       navigate   one point and one camera index, stored in the PICK slot
%       home       nothing at all. The pose is cfg.task.homePose, which this
%                  project already knows, so the provider is being asked to
%                  recognise an intent rather than to locate anything. There is
%                  no point to range-check and no image to name, so the only
%                  gate a home action can fail is LowConfidence.
%
%   boxes IS DISPLAY-ONLY AND DELIBERATELY NOT PART OF THE QUEUE. Gemini
%   Robotics-ER answers with box_2d as [ymin xmin ymax xmax] normalised 0..1000,
%   which is exactly cfg.camera.pixelNormScale, so a real box is available for
%   the asking. It is returned here as a separate argument, kept out of the
%   ActionQueue bus, and consumed only by hmmGroundingTap for the operator app:
%   the model acts on POINTS, and a rectangle drawn round an object changes no
%   grasp. A malformed or out-of-range box is discarded to NaN and does NOT fail
%   its action - refusing an otherwise valid grasp because a decoration was
%   badly formed would trade a working robot for a drawing.
%
%   q is a struct of parallel fixed-size arrays matching the ActionQueue bus.
%
%   See also hmmUnwrapFence, hmmGroundingService, hmmReconstructTargets.

%   Copyright 2026 The MathWorks, Inc.

arguments
    raw (1, :) char
    cfg struct = hmmConfig()
end

nQ   = double(cfg.queue.capacity);
nLbl = double(cfg.provider.labelChars);

q.Count       = uint8(0);
q.Verb        = zeros(nQ, 1, 'uint8');
q.Label       = zeros(nLbl, nQ, 'uint8');
q.LabelLen    = zeros(nQ, 1, 'uint8');
q.PickPoint   = zeros(2, nQ);
q.PickImage   = zeros(nQ, 1, 'uint8');
q.PlacePoint  = zeros(2, nQ);
q.PlaceImage  = zeros(nQ, 1, 'uint8');
q.Confidence  = zeros(nQ, 1);
q.ItemFailure = zeros(nQ, 1, 'uint8');

% Display-only, never bussed. NaN means "no box was offered for this slot", which
% is the normal case for navigate and home and a legitimate one for transfer.
boxes.PickBox  = nan(4, nQ);
boxes.PlaceBox = nan(4, nQ);

overflow     = false;
queueFailure = cfg.fail.MalformedJSON;

inner = hmmUnwrapFence(raw);
if isempty(strtrim(inner))
    return
end

% THE SAME ENVELOPE FAULT THE REFINE DECODER HIT, and it is latent here rather than
% fixed because this path has not yet been unlucky. The provider appends a stray
% closing brace often enough to break three of five live refine calls, and this
% decoder has the identical hmmUnwrapFence-then-jsondecode shape, so one extra
% character would cost the whole action queue and be reported as MalformedJSON.
% hmmExtractJsonObject only ever trims AFTER a brace-balanced object, so the
% malformed and truncated fixtures in the replay corpus are still rejected here.
inner = hmmExtractJsonObject(inner);

try
    decoded = jsondecode(inner);
catch
    return
end

if ~isstruct(decoded) || ~isfield(decoded, 'actions')
    return
end

items = decoded.actions;
if isempty(items)
    % An empty list is a VALID answer: the provider was asked to return one
    % when the instruction cannot be grounded in the supplied images.
    queueFailure = cfg.fail.OK;
    return
end
if iscell(items)
    items = items(:);
else
    items = num2cell(items(:));
end

nIn = numel(items);
overflow = nIn > nQ;
n = min(nIn, nQ);

for k = 1:n
    it = items{k};
    if ~isstruct(it) || ~isfield(it, 'action')
        q.Verb(k) = cfg.queue.VERB_NONE;
        q.ItemFailure(k) = cfg.fail.MalformedJSON;
        continue
    end

    conf = 1.0;
    if isfield(it, 'confidence') && isnumeric(it.confidence) && isscalar(it.confidence)
        conf = double(it.confidence);
    end
    q.Confidence(k) = conf;

    switch lower(strtrim(char(it.action)))
        case 'transfer'
            q.Verb(k) = cfg.queue.VERB_TRANSFER;
            if isfield(it, 'object')
                [q.Label(:, k), q.LabelLen(k)] = packLabel(it.object, nLbl);
            end
            [q.PickPoint(:, k),  fPick]  = readPoint(it, 'pick_point',  cfg);
            [q.PickImage(k),     fPickC] = readImage(it, 'pick_image',  cfg);
            [q.PlacePoint(:, k), fPlace] = readPoint(it, 'place_point', cfg);
            [q.PlaceImage(k),    fPlaceC] = readImage(it, 'place_image', cfg);
            q.ItemFailure(k) = firstFailure([fPick, fPickC, fPlace, fPlaceC], cfg);
            boxes.PickBox(:, k)  = readBox(it, 'pick_box',  cfg);
            boxes.PlaceBox(:, k) = readBox(it, 'place_box', cfg);

        case 'navigate'
            q.Verb(k) = cfg.queue.VERB_NAVIGATE;
            % navigate carries a single 'point'/'image' pair. It is stored in
            % the PICK slot so one set of downstream reconstruction wiring
            % serves both verbs; the place slot stays zero and Verb says which
            % slots are meaningful.
            [q.PickPoint(:, k), fPt]  = readPoint(it, 'point', cfg);
            [q.PickImage(k),    fImg] = readImage(it, 'image', cfg);
            q.ItemFailure(k) = firstFailure([fPt, fImg], cfg);
            boxes.PickBox(:, k) = readBox(it, 'box', cfg);

        case 'home'
            % No point, no image, no gate to apply. The pose is already known,
            % so there is nothing here that could be out of range or address a
            % camera that was never sent - the slots stay zero and Verb is the
            % only field a reader needs. An optional 'object' label is still
            % packed, because the provider tends to echo something useful there
            % ("home position") and it costs nothing to keep for the display.
            q.Verb(k) = cfg.queue.VERB_HOME;
            if isfield(it, 'object')
                [q.Label(:, k), q.LabelLen(k)] = packLabel(it.object, nLbl);
            end
            q.ItemFailure(k) = cfg.fail.OK;

        otherwise
            q.Verb(k) = cfg.queue.VERB_NONE;
            q.ItemFailure(k) = cfg.fail.UnknownAction;
    end

    if q.ItemFailure(k) == cfg.fail.OK && conf < cfg.provider.confidenceMin
        q.ItemFailure(k) = cfg.fail.LowConfidence;
    end
end

q.Count = uint8(n);
if overflow
    queueFailure = cfg.fail.QueueOverflow;
else
    queueFailure = cfg.fail.OK;
end

end

%% ------------------------------------------------------------------------
function [pt, failure] = readPoint(it, field, cfg)
%READPOINT A normalised [y x] pair, range-checked against pixelNormScale.
pt = zeros(2, 1);
failure = cfg.fail.OK;
if ~isfield(it, field)
    failure = cfg.fail.MalformedJSON;
    return
end
v = it.(field);
if ~isnumeric(v) || numel(v) ~= 2 || any(~isfinite(v))
    failure = cfg.fail.MalformedJSON;
    return
end
pt = double(v(:));
if any(pt < 0) || any(pt > cfg.camera.pixelNormScale)
    failure = cfg.fail.OutOfRange;
end
end

%% ------------------------------------------------------------------------
function box = readBox(it, field, cfg)
%READBOX Gemini's box_2d as [ymin xmin ymax xmax], or NaN if it is not usable.
%
%   NO FAILURE CODE COMES OUT OF HERE, by design. The box is drawn and nothing
%   else; a bad one is dropped rather than escalated, so the action it decorates
%   is judged only on the point the robot will actually drive to. The ordering
%   check matters as much as the range check - a provider that returns the corners
%   the other way round would otherwise produce a rectangle of negative width,
%   which draws as nothing and reads as a rendering bug rather than a bad answer.
box = nan(4, 1);
if ~isfield(it, field)
    return
end
v = it.(field);
if ~isnumeric(v) || numel(v) ~= 4 || any(~isfinite(v))
    return
end
v = double(v(:));
if any(v < 0) || any(v > cfg.camera.pixelNormScale)
    return
end
if v(3) <= v(1) || v(4) <= v(2)
    return
end
box = v;
end

%% ------------------------------------------------------------------------
function [idx, failure] = readImage(it, field, cfg)
%READIMAGE A 1-based camera index, checked against the FIXED camera count.
idx = uint8(0);
failure = cfg.fail.OK;
if ~isfield(it, field)
    failure = cfg.fail.MalformedJSON;
    return
end
v = it.(field);
if ~isnumeric(v) || ~isscalar(v) || ~isfinite(v) || v ~= fix(v)
    failure = cfg.fail.MalformedJSON;
    return
end
if v < 1 || v > cfg.camera.nFixed
    failure = cfg.fail.UnknownCamera;
    return
end
idx = uint8(v);
end

%% ------------------------------------------------------------------------
function code = firstFailure(codes, cfg)
%FIRSTFAILURE The first non-OK code, so one action reports one cause.
code = cfg.fail.OK;
bad = codes(codes ~= cfg.fail.OK);
if ~isempty(bad)
    code = bad(1);
end
end

%% ------------------------------------------------------------------------
function [codes, len] = packLabel(label, nLbl)
%PACKLABEL Object label as zero-padded character codes with an explicit length.
codes = zeros(nLbl, 1, 'uint8');
s = char(string(label));
s = s(1:min(numel(s), nLbl));
codes(1:numel(s)) = uint8(s);
len = uint8(numel(s));
end
