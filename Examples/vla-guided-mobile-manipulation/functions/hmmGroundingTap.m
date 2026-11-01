function hmmGroundingTap(instruction, images, indices, queue, route, state, ...
    httpStatus, latency, simTime, cfg, boxes, answer)
%HMMGROUNDINGTAP Publish one grounding transaction for a user interface to display.
%
%   HMMGROUNDINGTAP(instruction, images, indices, queue, route, state, httpStatus,
%   latency, simTime, cfg, boxes, answer) writes what just happened to root appdata
%   under 'hmmGroundingTap', where an app can poll it while the simulation runs. It
%   returns nothing and it changes nothing.
%
%   boxes IS THE ONE THING HERE THAT IS ON NO SIGNAL AT ALL. Every other field is a
%   copy of something the model also acts on. Gemini Robotics-ER's box_2d is not:
%   hmmValidateActions keeps it out of the ActionQueue bus precisely because the
%   robot drives to points and not to rectangles, so this tap is the whole of its
%   existence. That makes it optional - a caller that has no boxes omits the
%   argument and every action simply reports an empty box.
%
%   A DISPLAY TAP, NOT AN OUTPUT. Everything here is already on
%   hmmGroundingService's outputs and reaches the model through PerceptionStatus and
%   ActionQueue; those signals remain the contract and the only thing the model acts
%   on. What they cannot do is be read WHILE the run is in progress - logged signals
%   materialise when the simulation stops, and the frames that were sent to the
%   provider are not on any port at all. An operator typing a sentence needs to see
%   the answer to that sentence within a second or two of it arriving, so the answer
%   is published to a side channel from the one place that holds all of it at once.
%   Nothing downstream of the model reads this, and deleting it would change no
%   simulation result.
%
%   WHAT IS PUBLISHED, AND WHAT DELIBERATELY IS NOT. The instruction, the images
%   that were sent and their camera indices, the decoded actions with their pixels
%   and confidences, the per-item failures, the route, the HTTP status, the latency
%   and the simulation time. NO CREDENTIAL, IN ANY FORM. The route is a small
%   integer saying which of the four discovery mechanisms resolved a key on this
%   machine, which is a fact about the configuration of this computer; the key
%   itself never enters this function, is not an input to it, and could not be
%   published by it. That distinction is the project's rule and this function is
%   deliberately placed where it cannot break it: hmmProviderRequest overwrites and
%   clears the key before it returns, so by the time anything reaches here there is
%   nothing secret left to leak. err.message is likewise not an input; the five
%   failure modes arrive as codes.
%
%   IT CANNOT FAIL THE SIMULATION. The whole body is guarded, because a display that
%   throws would abort a run that was otherwise fine - and it would do so from
%   inside an extrinsic call, where the error surfaces as a MATLAB Function block
%   fault and reads like a defect in the perception path. A tap that cannot be shown
%   is a blank panel, not a failed run.
%
%   THE HTTP BODY IS STILL NOT HERE, AND answer IS NOT THE HTTP BODY. A failed
%   request's status and failure code say which of the five modes occurred, and the
%   body is withheld because it can carry the echoed request, and echoed requests
%   have been known to contain more than the sender expected. answer is a different
%   thing: it is the text the provider GENERATED, already pulled out of the envelope
%   by hmmProviderRequest/extractText, which is reached only when the transport
%   succeeded. On every error path hmmProviderRequest returns it empty, so no error
%   body can arrive here through this argument.
%
%   WHY IT IS PUBLISHED AT ALL. A MalformedJSON has two quite different causes -
%   either no text part was found in the envelope, or text arrived and did not decode
%   or did not match the schema - and the failure CODE is identical for both. A run
%   that reported "HTTP 200, latency 11.1 s, MalformedJSON" cost a live request and
%   still could not say which, so the console showed a failure it could not explain.
%   An empty answer now means the envelope, a non-empty one means the payload and
%   shows it. Capped at answerChars: this is a diagnosis, not a log, and appdata is
%   written from inside a solver step.
%
%   See also hmmGroundingService, hmmProviderRequest, hmmSubmitTask.

%   Copyright 2026 The MathWorks, Inc.

if nargin < 11 || ~isstruct(boxes)
    boxes = struct('PickBox', [], 'PlaceBox', []);
end
if nargin < 12 || ~(ischar(answer) || isstring(answer))
    answer = '';
end
answerChars = 600;

try
    tap = struct();
    tap.simTime     = simTime;
    tap.instruction = instruction;
    tap.route       = double(route);
    tap.state       = double(state);
    tap.httpStatus  = double(httpStatus);
    tap.latency     = latency;
    tap.taskId      = double(queue.TaskId);
    tap.requestId   = double(queue.RequestId);
    tap.count       = double(queue.Count);
    tap.head        = double(queue.Head);
    tap.overflow    = logical(queue.Overflow);
    tap.failureCode = double(queue.FailureCode);
    tap.failure     = codeName(cfg.fail, tap.failureCode);

    % The images are stored as sent. A UI that wants to draw a pick point on the
    % frame it was measured in needs THAT frame, not a fresh render taken later
    % from a camera the robot has since driven past.
    tap.images  = images;
    tap.indices = double(indices);

    % THE HEAD, NOT THE WHOLE ANSWER, AND THE TRUNCATION SAYS SO. A reply that
    % overran is exactly the kind that fails to decode, so a silent cut would hide
    % the symptom it is here to show.
    a = char(answer);
    tap.answerChars = numel(a);
    if numel(a) > answerChars
        a = [a(1:answerChars) sprintf('... [%d chars in all]', numel(answer))];
    end
    tap.answer = a;

    tap.actions = decodeQueue(queue, cfg, boxes);

    setappdata(0, 'hmmGroundingTap', tap);
catch
    % Silent on purpose, and this is the one place in the project where that is
    % right: there is no channel to report on. Writing to the command window from
    % inside a solver step would scroll a warning past hundreds of times, and
    % raising would abort the run for the sake of a panel.
    return
end

% PUSHED AS WELL AS PUBLISHED, for a viewer that does not poll: hmmGroundingView
% registers 'hmmGroundingListener', the console polls the appdata above. A listener
% that throws must not take the grounding service down with it.
try
    listener = getappdata(0, 'hmmGroundingListener');
    if isa(listener, 'function_handle')
        listener(tap);
        drawnow limitrate
    end
catch
end

end

%% ----------------------------------------------------------------------------
function actions = decodeQueue(queue, cfg, boxes)
%DECODEQUEUE The queue's parallel arrays as one struct per action.
%   ActionQueue is deliberately parallel fixed-size arrays rather than an array of
%   buses, for the reason hmmDefineBuses records. That is right for the model and
%   wrong for a display, so the transposition happens once, here, instead of in
%   every callback that wants to draw a marker.
%
%   The boxes are transposed alongside, from the same slot index, so an action and
%   its rectangle cannot be paired up wrongly by a caller counting separately.
n = double(queue.Count);
actions = struct('verb', {}, 'label', {}, 'pickPoint', {}, 'pickImage', {}, ...
    'placePoint', {}, 'placeImage', {}, 'confidence', {}, ...
    'execState', {}, 'itemFailure', {}, 'failure', {}, ...
    'pickBox', {}, 'placeBox', {});
for k = 1:n
    lab = '';
    nLab = double(queue.LabelLen(k));
    if nLab > 0
        lab = char(queue.Label(1:nLab, k))';
    end
    actions(k) = struct( ...
        'verb',        codeName(cfg.queue, double(queue.Verb(k)), 'VERB_'), ...
        'label',       lab, ...
        'pickPoint',   reshape(double(queue.PickPoint(:, k)), 1, 2), ...
        'pickImage',   double(queue.PickImage(k)), ...
        'placePoint',  reshape(double(queue.PlacePoint(:, k)), 1, 2), ...
        'placeImage',  double(queue.PlaceImage(k)), ...
        'confidence',  double(queue.Confidence(k)), ...
        'execState',   codeName(cfg.queue, double(queue.ExecState(k)), 'EXEC_'), ...
        'itemFailure', double(queue.ItemFailure(k)), ...
        'failure',     codeName(cfg.fail, double(queue.ItemFailure(k))), ...
        'pickBox',     boxSlot(boxes, 'PickBox', k), ...
        'placeBox',    boxSlot(boxes, 'PlaceBox', k));
end
end

%% ----------------------------------------------------------------------------
function b = boxSlot(boxes, field, k)
%BOXSLOT One [ymin xmin ymax xmax] box, or empty when none was offered.
%   EMPTY RATHER THAN NaN AT THE BOUNDARY. NaN is the right internal sentinel - it
%   keeps the array rectangular across the queue's fixed capacity - but a display
%   asks "is there a box" and isempty is the check that cannot be got wrong, where
%   a NaN silently draws a rectangle nowhere.
b = zeros(1, 0);
if ~isfield(boxes, field)
    return
end
v = boxes.(field);
if size(v, 2) < k || size(v, 1) ~= 4
    return
end
col = double(v(:, k));
if any(~isfinite(col))
    return
end
b = reshape(col, 1, 4);
end

%% ----------------------------------------------------------------------------
function name = codeName(s, value, prefix)
%CODENAME The configuration field whose value is this code, for display.
%   SEARCHED RATHER THAN TABULATED. A hard-coded list of verb and failure names
%   here would be a second copy of hmmConfig's constants, free to drift from it
%   silently and read plausibly while doing so - a display naming an action
%   'navigate' when the code says transfer is worse than one showing a number.
%   Falling back to the number is honest when nothing matches.
if nargin < 3
    prefix = '';
end
name = sprintf('%d', value);
try
    f = fieldnames(s);
    if ~isempty(prefix)
        f = f(startsWith(f, prefix));
    end
    for k = 1:numel(f)
        v = s.(f{k});
        if isnumeric(v) && isscalar(v) && double(v) == value
            name = lower(erase(f{k}, prefix));
            return
        end
    end
catch
end
end
