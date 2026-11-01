function [text, route, httpStatus, latency, failureCode] = ...
    hmmProviderRequest(instruction, images, imageIndices, cfg, kind, note)
%HMMPROVIDERREQUEST Ask the vision-language provider to ground one instruction.
%
%   [text, route, httpStatus, latency, failureCode] =
%       HMMPROVIDERREQUEST(instruction, images, imageIndices, cfg, kind)
%
%   images is a cell array of uint8 [rows cols 3] frames and imageIndices the
%   1-based contract index of each, so the returned *_image field can be read
%   against the same numbering the camera rig uses.
%
%   text is the RAW response body, unparsed and unrepaired. Parsing is
%   hmmValidateActions' job; keeping them apart means a malformed response can
%   be inspected as it arrived.
%
%   TWO KINDS OF QUESTION, ONE TRANSPORT. kind selects what is being asked:
%
%       'actions'  (default) the wide grounding call. Four ceiling cameras, the
%                  {"actions":[...]} schema, and the behaviour of every release
%                  before the wrist refinement existed - byte for byte.
%       'refine'   the wrist call. ONE close-up frame from the end-effector
%                  camera and a single-point schema, asked once just before the
%                  descent commits. See hmmWristRefine.
%
%   IT IS ONE FUNCTION BECAUSE THE CREDENTIAL MUST BE HANDLED IN ONE PLACE. A
%   second requester for the second question would mean a second copy of the key
%   resolution, the x-goog-api-key header and the scrubbing on the way out, and
%   the project's security argument rests on there being exactly one. Adding a
%   prompt and a schema here is a much smaller change than duplicating the part
%   that touches a secret.
%
%   CREDENTIAL HANDLING. The key is resolved at runtime in this order:
%
%       1  GEMINI_API_KEY environment variable        -> ROUTE_ENV
%       2  a key file found by hmmAssetRoots' walk    -> ROUTE_KEYFILE
%       3  a recorded response corpus                 -> ROUTE_REPLAY
%       4  no matching recording                      -> ROUTE_REPLAY, ReplayMiss
%
%   The key is never returned, never assigned to a caller-visible variable,
%   never printed, never written to a file, and never placed in the request URL.
%   It travels in an x-goog-api-key HEADER, not a query parameter, because a URL
%   ends up in exception messages, in webread's own diagnostics and in any proxy
%   log between here and the provider - putting a secret there leaks it through
%   the error path even when the happy path is careful. Only route, which of the
%   four above resolved, is reported to the model.
%
%   note is optional prompt text for the 'actions' call, used on a re-ask to say what was wrong
%   with the previous answer (hmmGroundingService). It never affects the replay match, which is
%   keyed on the instruction alone.
%
%   GEMINI_PROVIDER overrides cfg.provider.mode: 'live' refuses to fall back to
%   replay, 'replay' never contacts the network, 'auto' prefers a live key.
%
%   See also hmmAssetRoots, hmmUnwrapFence, hmmValidateActions, hmmWristRefine.

%   Copyright 2026 The MathWorks, Inc.

arguments
    instruction  (1, :) char
    images       cell
    imageIndices double
    cfg          struct = hmmConfig()
    kind         (1, :) char {mustBeMember(kind, {'actions', 'refine'})} = 'actions'
    note         (1, :) char = ''
end

% EVERY OUTPUT GETS A DEFAULT BEFORE THE FIRST EARLY RETURN, AND ALL FIVE STAY
% EVEN THOUGH TWO ARE PROVABLY OVERWRITTEN TODAY. The function returns early on
% six different failure paths; the defaults are the contract that none of them
% can leave an output unassigned, so removing the two the analyzer can currently
% prove redundant would make the guarantee depend on the paths never changing.
% The pair is suppressed rather than deleted for that reason.
text        = '';
route       = cfg.provider.ROUTE_NONE;                                 %#ok<NASGU>
httpStatus  = uint16(0);
latency     = 0;
failureCode = cfg.fail.ProviderNoKey;                                  %#ok<NASGU>

mode = getenv('GEMINI_PROVIDER');
if isempty(mode)
    mode = cfg.provider.mode;
end
mode = lower(strtrim(mode));

%% Route selection. The key is scoped to this function and cleared on the way out.
[key, route] = hmmProviderKey(mode, cfg);

if isempty(key)
    if strcmp(mode, 'live')
        % An explicit 'live' request must fail loudly rather than quietly
        % returning a recording that would be mistaken for a live answer.
        failureCode = cfg.fail.ProviderNoKey;
        return
    end
    [text, ok] = readReplay(instruction, cfg, kind);
    if ok
        route       = cfg.provider.ROUTE_REPLAY;
        failureCode = cfg.fail.OK;
    else
        route       = cfg.provider.ROUTE_REPLAY;
        failureCode = cfg.fail.ReplayMiss;
    end
    return
end

%% Live request
prompt = buildPrompt(instruction, imageIndices, cfg, kind, note);

parts = {struct('text', prompt)};
for k = 1:numel(images)
    parts{end + 1} = struct('inline_data', ...
        struct('mime_type', 'image/png', 'data', encodePng(images{k})));  %#ok<AGROW>
end

body = struct( ...
    'contents', struct('role', 'user', 'parts', {parts}), ...
    'generationConfig', struct( ...
        'temperature', cfg.provider.temperature, ...
        'responseMimeType', 'application/json'));

url = sprintf('%s/%s:generateContent', cfg.provider.endpoint, cfg.provider.model);

% A 5xx IS RETRIED, AND IT IS THE ONLY THING THAT IS - see cfg.provider.retries5xx for the
% measurement. The attempt loop is here rather than around the whole function because the
% prompt, the body and the key discovery are all identical between attempts: re-entering the
% function would re-read the key off disk and rebuild a PNG for no reason.
%
% THE KEY IS OVERWRITTEN AFTER THE LAST ATTEMPT, NOT AFTER THE FIRST, which is the one thing
% this loop had to get right. The original cleared it inside both branches, so lifting the
% request into a loop without moving the clear would have sent attempt two with a header of
% asterisks and turned a retryable 503 into a permanent 403 - a failure mode invented by the
% fix. The clear now sits after the loop, where it still runs on every path out.
t0 = tic;
attemptsLeft = max(0, double(cfg.provider.retries5xx));
backoff = cfg.provider.retryBackoff;
% A TIMEOUT IS RETRIED ON THE GROUNDING CALL ONLY - see cfg.provider.retriesTimeoutGround. The
% refinement has a depth-only fallback, so a second wait buys it nothing; grounding has none,
% and one lost reply fails the whole task.
timeoutsLeft = 0;
if strcmp(kind, 'actions')
    timeoutsLeft = max(0, double(cfg.provider.retriesTimeoutGround));
end
% A 429 IS WAITED OUT ONCE. A free key's per-minute limits refused whole runs on 2026-10-16 -
% grounding and refinement alike - and a request a minute later went through. Google's 429
% names how long to wait (RetryInfo.retryDelay); that, capped, or a default.
rateLeft = max(0, double(cfg.provider.retries429));
detail = '';
while true
    % matlab.net.http, NOT webwrite: on a 4xx/5xx webwrite throws with the status line only,
    % and the BODY is where Google says which limit was hit and how long to wait - the one
    % thing four refused floor runs needed to know (2026-10-16).
    try
        [reply, status] = postJson(url, body, key, cfg.provider.timeout);
        if status == 200
            [text, failureCode, httpStatus] = extractText(reply, cfg);
            detail = '';
        else
            httpStatus = uint16(status);
            failureCode = cfg.fail.ProviderError;
            % The body is kept for a rate-limit or server-side reply, which carries no
            % credential and is the only place the REASON is written; never for an
            % authentication failure, where some providers echo the presented key back.
            detail = '';
            if status == 429 || (status >= 500 && status <= 599)
                detail = providerDetail(jsonencode(reply));
            end
            fprintf('hmmProviderRequest: request failed, status %d%s.\n', ...
                httpStatus, ternaryDetail(detail));
        end
    catch err
        [failureCode, httpStatus] = classify(err, cfg);
        if contains(err.message, 'timed out', 'IgnoreCase', true)
            failureCode = cfg.fail.ProviderTimeout;
        end
        detail = '';
        fprintf('hmmProviderRequest: request failed (%s), status %d.\n', ...
            err.identifier, httpStatus);
    end

    if httpStatus == 429 && rateLeft > 0
        w = min(retryDelayOf(detail, cfg.provider.retry429Default), cfg.provider.retry429MaxWait);
        fprintf('hmmProviderRequest: HTTP 429 (rate limit), waiting %.0f s and trying once more.\n', w);
        pause(w)
        rateLeft = rateLeft - 1;
        continue
    end

    if failureCode == cfg.fail.ProviderTimeout && timeoutsLeft > 0
        fprintf(['hmmProviderRequest: no reply within %.0f s, trying once more ' ...
                 '(grounding has no fallback).\n'], cfg.provider.timeout);
        timeoutsLeft = timeoutsLeft - 1;
        continue
    end

    % 5xx OTHERWISE. A 0 status means the transport never got an answer - a timeout or a dead
    % socket - and retrying that spends another cfg.provider.timeout on the model's thread.
    if httpStatus < 500 || httpStatus > 599 || attemptsLeft <= 0
        break
    end
    fprintf(['hmmProviderRequest: status %d is transient, retrying in %.1f s ' ...
             '(%d attempt(s) left).\n'], httpStatus, backoff, attemptsLeft);
    pause(backoff)
    backoff = 2 * backoff;
    attemptsLeft = attemptsLeft - 1;
end
latency = toc(t0);                  % the WHOLE transaction, retries and backoff included
key = repmat('*', 1, numel(key));                                      %#ok<NASGU>
clear key
% A FAILED REQUEST RETURNS ITS REASON AS THE TEXT, so the grounding tap and the console show
% "HTTP 429: quota exceeded for ..." instead of an empty reply. Nothing parses text when
% failureCode is set.
if failureCode ~= cfg.fail.OK && ~isempty(detail)
    text = sprintf('[HTTP %d] %s', httpStatus, detail);
end

end

%% ------------------------------------------------------------------------
function [reply, status] = postJson(url, body, key, timeout)
%POSTJSON POST a JSON body and return the decoded reply and the status, whatever it is.
%   The key travels only in the x-goog-api-key header, never in the URL. Throws only for a
%   transport failure (no connection, timeout); any HTTP status, error or not, is returned.
import matlab.net.http.*
hdr = [field.ContentTypeField('application/json'), HeaderField('x-goog-api-key', key)];
req = RequestMessage('POST', hdr, MessageBody(body));
opt = HTTPOptions('ConnectTimeout', timeout, 'ResponseTimeout', timeout);
resp = req.send(matlab.net.URI(url), opt);
status = double(resp.StatusCode);
reply = [];
if ~isempty(resp.Body)
    reply = resp.Body.Data;
end
if ischar(reply) || isstring(reply)
    try
        reply = jsondecode(char(reply));
    catch
    end
end
end

function d = providerDetail(msg)
%PROVIDERDETAIL The provider's own explanation from a failed request, short and on one line.
%   webwrite puts the server's reply in the error message; for Gemini that is a JSON error
%   whose "message" says which limit or which fault. Anything that looks like a key is masked.
d = '';
if isempty(msg)
    return
end
m = regexp(msg, '"message"\s*:\s*"([^"]*)"', 'tokens', 'once');
if ~isempty(m)
    d = m{1};
else
    d = msg;
end
% THE LIMIT ITSELF, which the prose message buries after its links: Google's QuotaFailure
% violations name it (quotaId, e.g. a free-tier requests-per-minute limit) and its value.
q = regexp(msg, '"quotaId"\s*:\s*"([^"]+)"', 'tokens');
v = regexp(msg, '"quotaValue"\s*:\s*"([^"]+)"', 'tokens');
if ~isempty(q)
    lim = strjoin(cellfun(@(c) c{1}, q, 'UniformOutput', false), ', ');
    if ~isempty(v)
        lim = sprintf('%s = %s', lim, strjoin(cellfun(@(c) c{1}, v, 'UniformOutput', false), ', '));
    end
    d = sprintf('limit %s. %s', lim, extractBefore([d ' For more'], ' For more'));
end
retry = regexp(msg, '"retryDelay"\s*:\s*"([0-9.]+)s"', 'tokens', 'once');
if ~isempty(retry)
    d = sprintf('%s (retry in %s s)', d, retry{1});
end
d = regexprep(d, 'AIza[0-9A-Za-z_\-]{30,}', '<key>');      % a Google API key's shape
d = regexprep(d, 'key=[^&\s"]+', 'key=<key>');            % and one passed in a URL
d = regexprep(strtrim(regexprep(d, '\s+', ' ')), '\\n', ' ');
if numel(d) > 300
    d = [d(1:300) '...'];
end
end

function s = ternaryDetail(d)
s = '';
if ~isempty(d)
    s = [': ' d];
end
end

function w = retryDelayOf(detail, fallback)
%RETRYDELAYOF The wait Google asked for, in seconds, or the fallback when it did not say.
w = fallback;
t = regexp(detail, 'retry in ([0-9.]+) s', 'tokens', 'once');
if ~isempty(t)
    w = str2double(t{1});
end
end

%% ------------------------------------------------------------------------
function [text, ok] = readReplay(instruction, cfg, kind)
%READREPLAY Return a recorded response so a run reproduces without a key.
%   Only an exact instruction tag is eligible. An unmatched instruction must
%   never execute an unrelated fixture, including a negative-test recording.
%
%   THE TWO KINDS ARE SEPARATE POOLS, AND THIS IS THE ONE PLACE IT IS ENFORCED.
%   A refine recording answers with a single point and an actions recording with
%   an {"actions":[...]} array, so serving one where the other was asked for
%   produces a parse failure at best and a plausible-looking answer to a
%   different question at worst. The tag alone cannot separate them - it hashes
%   the INSTRUCTION, and both calls are made for the same instruction - so the
%   pools are split by file name instead: a refine recording is named
%   refine_<tag>.json and an actions recording is anything else. Each kind can
%   only ever be served from its own pool and only for the exact instruction.
text = '';
ok = false;
isRefine = strcmp(kind, 'refine');
% The tag is computed by hmmReplayTag, which the corpus generator also uses. A
% second implementation here would let the two drift and cause false misses.
tag = hmmReplayTag(instruction);
roots = hmmAssetRoots();
for r = 1:numel(roots)
    folder = fullfile(roots{r}, 'perception', 'gemini', 'responses');
    if ~isfolder(folder)
        continue
    end
    files = [dir(fullfile(folder, '*.json')); dir(fullfile(folder, '*.txt'))];
    if isempty(files)
        continue
    end
    names = sort(string({files.name}));
    names = names(startsWith(names, 'refine') == isRefine);
    if isempty(names)
        % This root has no recording of the KIND being asked for. Keep walking:
        % another root may, and reporting "no corpus" here would hide it.
        continue
    end
    match = names(endsWith(names, "_" + string(tag) + ".json") | ...
                  endsWith(names, "_" + string(tag) + ".txt"));
    if isempty(match)
        continue
    end
    text = fileread(fullfile(folder, char(match(1))));
    ok = true;
    fprintf('hmmProviderRequest: using the sample response %s.\n', char(match(1)));
    return
end
if ~strcmp(cfg.provider.mode, 'replay')
    fprintf('hmmProviderRequest: no credential and no sample response found.\n');
elseif isRefine
    % WORTH SAYING OUT LOUD, because the refinement is advisory and a missing
    % fixture is otherwise completely silent: the run still completes, the
    % descent still uses the wide call's point, and nothing looks wrong.
    fprintf(['hmmProviderRequest: no refine_*.json sample response found, so the wrist ' ...
             'refinement is inert this run.\n']);
end
end

%% ------------------------------------------------------------------------
function prompt = buildPrompt(instruction, imageIndices, cfg, kind, note)
%BUILDPROMPT Dispatch to the prompt for the kind of question being asked.
if strcmp(kind, 'refine')
    prompt = buildRefinePrompt(instruction, imageIndices, cfg);
else
    prompt = buildActionsPrompt(instruction, imageIndices, cfg, note);
end
end

%% ------------------------------------------------------------------------
function prompt = buildRefinePrompt(instruction, imageIndices, cfg)
%BUILDREFINEPROMPT Ask one close-up question: where in THIS frame is the object.
%
%   THIS IS A DELIBERATELY EASIER QUESTION THAN THE WIDE CALL, and that is the entire
%   justification for asking it. The wide call has to choose a verb, find two places in four
%   rooms and say which camera each is in. This one has a single image, a single answer, and
%   the object already nearly centred in frame because the arm is poised over it. What it buys
%   is precision, from geometry rather than from the model being cleverer: at the pre-descent
%   pose the wrist camera is ~0.119 m away and one pixel is 0.38 mm, against 16.5 mm for a
%   ceiling camera at 3.798 m.
%
%   NO VERB, NO SECOND POINT, NO CAMERA INDEX. Anything the wide call already decided is
%   absent from this schema, because re-asking it invites the provider to contradict a decision
%   the arm has already acted on. The refinement can move the grasp point; it cannot change
%   what is being picked up or what happens next.
%
%   found IS A REQUIRED FIELD AND NOT A COURTESY. The object may genuinely not be in the wrist
%   frame - the wide call may have been wrong, or the arm may be poised over the wrong part of
%   a cluttered surface - and a model with no way to say so will invent a pixel. hmmWristRefine
%   treats found:false as a clean rejection and leaves the coarse point standing, which is the
%   correct outcome and needs to be reachable.
idxList = strjoin(compose('%d', imageIndices(:)'), ', ');
prompt = sprintf([ ...
    'A robot arm is holding its gripper just above an object it is about to pick up. ' ...
    'The attached image is camera %s, a close-up view from a camera mounted on the ' ...
    'wrist, looking down at the object between the open jaws.\n\n' ...
    'The task being carried out is: %s\n\n' ...
    'Point at the CENTRE of the object as it appears in this image - the middle of its ' ...
    'outline seen from above, midway between where the two jaws will touch its sides. Do not ' ...
    'point at its edge, its side, its rim or its handle: the gripper closes around the ' ...
    'object, so the point it needs is the centre between the jaws.\n\n' ...
    'Reply with JSON only, no prose, in exactly this schema:\n' ...
    '{"found": <true|false>, "point": [y, x], "confidence": <0..1>}\n\n' ...
    'The point is [y, x] normalised to 0..%d over the image, y downward from the top ' ...
    'row.\n\n' ...
    'If the object is not visible in this image, reply {"found": false, "confidence": 0} ' ...
    'and do not guess a point. A wrong point is worse than no point here, because the ' ...
    'arm is already in position and will act on it.'], ...
    idxList, instruction, cfg.camera.pixelNormScale);
end

%% ------------------------------------------------------------------------
function prompt = buildActionsPrompt(instruction, imageIndices, cfg, note)
%BUILDACTIONSPROMPT State the schema the model must answer in, and the camera numbering.
%   The schema is stated HERE rather than assumed, because the *_point
%   normalisation and the 1-based *_image index are this project's contract with
%   the provider and both must be restated for every request.
%
%   THE BOXES ARE ASKED FOR IN THE MODEL'S OWN UNITS. Gemini Robotics-ER emits
%   box_2d as [ymin, xmin, ymax, xmax] normalised 0..1000, and
%   cfg.camera.pixelNormScale is 1000, so the box convention here is not a
%   translation of the model's output - it IS the model's output. That is why the
%   boxes can be drawn without inventing an extent from the point: they are
%   optional, and when the provider omits one nothing is drawn.
%
%   place_point IS THE SET-DOWN SPOT, NOT THE SURFACE. Asked for a place, Robotics-ER points at
%   the centre of the thing it recognised: on 2026-10-12 two live transfers to the kitchen island
%   answered [650 340] and [650 350] against a place_box centred at column 352 - the island's
%   middle, which no base pose can serve. The schema pairs a box with a point, so the prompt says
%   what each one means. A location named in the instruction wins; otherwise an open edge is
%   preferred. The provider cannot judge reach, so hmmReconstructTargets still checks the point
%   and slides it a bounded distance along the surface when it must (slideOnSurface).
%
%   HOME IS DESCRIBED, NOT LOCATED. It is the only verb with no point and no
%   image, so the prompt has to say what qualifies - otherwise the model reaches
%   for navigate and guesses a pixel for a place it cannot see.
idxList = strjoin(compose('%d', imageIndices(:)'), ', ');
prompt = sprintf([ ...
    'You are grounding a household manipulation instruction against fixed ' ...
    'ceiling cameras in an apartment.\n\n' ...
    'Instruction: %s\n\n' ...
    'The attached images are cameras %s, in that order. Refer to a camera by ' ...
    'its number from that list.\n\n' ...
    'Reply with JSON only, no prose, in exactly this schema:\n' ...
    '{"actions": [\n' ...
    '  {"action": "transfer", "object": "<short label>", ' ...
    '"pick_point": [y, x], "pick_image": <n>, ' ...
    '"place_point": [y, x], "place_image": <n>, ' ...
    '"pick_box": [ymin, xmin, ymax, xmax], ' ...
    '"place_box": [ymin, xmin, ymax, xmax], "confidence": <0..1>},\n' ...
    '  {"action": "navigate", "point": [y, x], "image": <n>, ' ...
    '"box": [ymin, xmin, ymax, xmax], "confidence": <0..1>},\n' ...
    '  {"action": "home", "confidence": <0..1>}\n' ...
    ']}\n\n' ...
    'Every point is [y, x] and every box is [ymin, xmin, ymax, xmax], both ' ...
    'normalised to 0..%d over the image, y downward from the top row. A box is ' ...
    'optional; omit it rather than guess one.\n\n' ...
    'For a transfer, place_box outlines the receiving surface, but place_point is NOT the ' ...
    'centre of that surface: it is the exact free spot on it where the object will be set ' ...
    'down. If the instruction says where on the surface (an edge, a side, next to something), ' ...
    'put place_point there. Otherwise choose a free spot near an edge of the surface that ' ...
    'faces open floor, clear of other objects and of chairs or stools in front of it. The ' ...
    'robot is about 0.55 m wide and must stand right beside that edge, so the floor in front ' ...
    'of it needs to be at least a metre clear: on long furniture an end with open floor ' ...
    'beside it is usually reachable where the middle of a side facing other furniture ' ...
    '(a table, a bed side against a wall) is not.\n\n' ...
    '%s' ...
    'Choose the action by what is being asked:\n' ...
    '  transfer  move an object from where it is to somewhere else\n' ...
    '  navigate  drive the robot to a place that is visible in an image\n' ...
    '  home      return to the docking or home position. Use this whenever the ' ...
    'instruction says home, base, dock, start or parking spot. It takes no ' ...
    'point and no image, because the robot already knows where home is - do NOT ' ...
    'answer with navigate and a guessed pixel for it.\n\n' ...
    'Use only the actions "transfer", "navigate" and "home". Return at most %d ' ...
    'actions, in execution order. If the instruction cannot be grounded in ' ...
    'these images, return {"actions": []}.'], ...
    instruction, idxList, cfg.camera.pixelNormScale, note, cfg.queue.capacity);
end

%% ------------------------------------------------------------------------
function data = encodePng(rgb)
%ENCODEPNG Base64 PNG of one frame, via a temporary file that is always removed.
f = [tempname '.png'];
imwrite(rgb, f);
cleanup = onCleanup(@() deleteIfPresent(f));
% Read as BYTES. fileread would apply a character encoding to binary data and
% corrupt the PNG for any byte outside the encoding's range.
fid = fopen(f, 'r');
bytes = fread(fid, inf, '*uint8');
fclose(fid);
data = matlab.net.base64encode(bytes);
clear cleanup
end

function deleteIfPresent(f)
if isfile(f)
    delete(f);
end
end

%% ------------------------------------------------------------------------
function [text, failureCode, httpStatus] = extractText(reply, cfg)
%EXTRACTTEXT Pull the response body out of the provider envelope.
%   A reply that does not carry the expected envelope is MalformedJSON: the
%   transport succeeded, so this is a schema problem, not a network one.
text = '';
httpStatus = uint16(200);
failureCode = cfg.fail.MalformedJSON;
if ~isstruct(reply) || ~isfield(reply, 'candidates') || isempty(reply.candidates)
    return
end
cand = reply.candidates;
if iscell(cand)
    cand = cand{1};
else
    cand = cand(1);
end
if ~isfield(cand, 'content') || ~isfield(cand.content, 'parts')
    return
end
p = cand.content.parts;
if iscell(p)
    p = p{1};
else
    p = p(1);
end
if ~isfield(p, 'text')
    return
end
text = char(p.text);
failureCode = cfg.fail.OK;
end

%% ------------------------------------------------------------------------
function [failureCode, httpStatus] = classify(err, cfg)
%CLASSIFY Map a transport exception onto a structured failure code and status.
httpStatus = uint16(0);
id = err.identifier;
if contains(id, 'Timeout', 'IgnoreCase', true)
    failureCode = cfg.fail.ProviderTimeout;
    return
end
failureCode = cfg.fail.ProviderError;
% webservices status identifiers look like MATLAB:webservices:HTTP404StatusCodeError
tok = regexp(id, 'HTTP(\d{3})', 'tokens', 'once');
if ~isempty(tok)
    httpStatus = uint16(str2double(tok{1}));
end
end
