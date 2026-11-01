function [inner, wasFenced] = hmmUnwrapFence(txt)
%HMMUNWRAPFENCE Strip a markdown code fence from a provider response.
%
%   [inner, wasFenced] = HMMUNWRAPFENCE(txt) returns the payload inside a
%   ```json ... ``` fence, or txt unchanged when there is no fence.
%
%   This is ENVELOPE HANDLING, not repair. The distinction matters and it is the
%   whole reason this is a separate function. A fence is a transport wrapper:
%   the model returns valid JSON and then wraps it for display, so removing the
%   wrapper reveals what was already correct. Repair would mean editing the
%   payload - closing an unbalanced brace, guessing a missing field, coercing a
%   string to a number - and any of those inventions would make a malformed
%   response look like a good one. A payload that is still not valid JSON after
%   unwrapping is reported as MalformedJSON and the action is rejected.
%
%   Only the OUTERMOST fence is removed: the FIRST line that opens a fence and
%   the LAST line that closes one, with everything between them taken as the
%   payload. Anything outside that pair - a preamble, a trailing remark - is
%   envelope and is discarded with the fence markers themselves.
%
%   THE PREAMBLE CASE IS THE ONE THAT MATTERS, and requiring the fence to open on
%   the first line is what used to break it. A provider answer that is a
%   markdown-fenced JSON block must have its fence unwrapped and its actions
%   parsed, and the fixture written for that case opens with the line
%
%       Here is the plan.
%
%   before the fence, because that is what a chat-tuned model actually emits.
%   Under the old first-line rule startsWith failed, the text came back with its
%   fence intact, jsondecode threw and hmmValidateActions reported MalformedJSON -
%   so that case was failing and the whole action queue was lost to a courtesy
%   sentence. Prose around a fence is transport decoration by exactly the same
%   argument that makes the fence itself transport decoration.
%
%   This is still envelope handling and not repair. The fence is an unambiguous
%   delimiter the model put there on purpose, so the payload between the markers
%   is returned untouched: no brace is closed and no field is invented. An
%   UNTERMINATED fence - one marker and no partner - is left alone, because
%   guessing where the author meant to close it would be repair. A payload that is
%   still not valid JSON after unwrapping is reported as MalformedJSON and the
%   action is rejected.
%
%   See also hmmValidateActions, hmmExtractJsonObject, hmmGroundingService.

%   Copyright 2026 The MathWorks, Inc.

arguments
    txt (1, :) char
end

inner = strtrim(txt);
wasFenced = false;

if ~contains(inner, '```')
    return
end

lines = splitlines(string(inner));
if numel(lines) < 2
    return
end

% Every fence marker in the text, in order. The opening line may carry a language
% tag (```json); it is discarded with the fence because it describes the envelope,
% not the payload.
marker = find(strncmp(strtrim(lines), '```', 3));

% One marker is an unterminated fence and two are the pair. Fewer than two means
% there is nothing to unwrap - see the header on why a lone marker is left alone.
if numel(marker) < 2
    return
end

openIdx  = marker(1);
closeIdx = marker(end);
if closeIdx <= openIdx + 1
    % An empty fence. Returning '' here would read to the caller as "the provider
    % said nothing", which is a different failure from "the provider said
    % something unusable", so the text is left for jsondecode to reject.
    return
end

body = lines(openIdx + 1:closeIdx - 1);
inner = char(strtrim(strjoin(body, newline)));
wasFenced = true;

end
