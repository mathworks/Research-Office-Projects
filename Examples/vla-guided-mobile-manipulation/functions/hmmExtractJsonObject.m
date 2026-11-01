function [obj, wasTrimmed] = hmmExtractJsonObject(txt)
%HMMEXTRACTJSONOBJECT Take the first complete JSON object out of a provider response.
%
%   [obj, wasTrimmed] = HMMEXTRACTJSONOBJECT(txt) returns the substring from the
%   first '{' to its matching '}', discarding anything that follows it. When txt
%   does not begin with '{', or when the braces never balance, txt is returned
%   unchanged and wasTrimmed is false.
%
%   THIS IS ENVELOPE HANDLING, NOT REPAIR, and the distinction is the same one
%   hmmUnwrapFence draws - read that function's header first. A fence is a
%   transport wrapper around valid JSON; so is a stray character appended AFTER a
%   syntactically complete object. In both cases the payload the model generated
%   was already correct and something in the transport added to it, so removing
%   the addition reveals what was already there. Nothing inside the object is
%   touched: no brace is closed, no field is guessed, no type is coerced.
%
%   The guard that keeps that claim honest is the balance test. This function
%   only ever returns a prefix that is brace-balanced from the first character.
%   An object that is TRUNCATED - '{"found": true, "point": [470,' - never
%   balances, so closeAt stays 0 and the text comes back untouched for the
%   caller's jsondecode to reject as MalformedJSON. Completing that object would
%   be repair, and it would turn a cut-off answer into a confident grasp
%   correction.
%
%   WHY IT EXISTS. gemini-robotics-er-2-preview intermittently appends a stray
%   closing brace to the single-point refine schema. Measured over five live
%   calls on one wrist frame, THREE came back as
%
%       {"found": true, "point": [470, 543], "confidence": 0.9}}
%
%   - a complete object plus one character. jsondecode throws on the whole
%   string, decodeRefine returned ok = false, hmmWristRefine answered
%   STATUS_PROVIDER, and the arm descended to the unrefined room-camera point
%   40.1 mm from the mug's body axis instead of the refined point 1.0 mm from it.
%   A one-character transport fault therefore cost the entire second look, on
%   most live attempts, with nothing in the model saying so louder than a status
%   code. The replay route was unaffected only because the shipped fixture
%   happens to be clean, which is exactly the kind of difference between the
%   tested path and the live path that this project is supposed to close.
%
%   STRING LITERALS ARE TRACKED, because a brace inside a quoted value must not
%   count toward the depth. The refine schema has no string values today, but the
%   wide grounding schema carries object labels and this function is written to be
%   usable there too. Backslash escapes are honoured so that a quoted '\"' does
%   not end the literal early.
%
%   Only the FIRST object is returned when several are concatenated. That mirrors
%   hmmUnwrapFence taking only the outermost fence: one answer was requested, so
%   the first complete one is the answer and the rest is envelope.
%
%   See also hmmUnwrapFence, hmmWristRefine, hmmValidateActions.

%   Copyright 2026 The MathWorks, Inc.

arguments
    txt (1, :) char
end

obj = strtrim(txt);
wasTrimmed = false;

% Anything that does not open with an object brace is not this function's
% business. A bare array, a number or prose is handed straight back so that the
% caller's parser reports it, rather than being silently reshaped here.
if isempty(obj) || obj(1) ~= '{'
    return
end

depth   = 0;
inStr   = false;
escaped = false;
closeAt = 0;

for i = 1:numel(obj)
    c = obj(i);

    if escaped
        % The previous character was a backslash inside a string literal, so this
        % one is data whatever it looks like.
        escaped = false;
        continue
    end

    if inStr
        if c == '\'
            escaped = true;
        elseif c == '"'
            inStr = false;
        end
        continue
    end

    switch c
        case '"'
            inStr = true;
        case '{'
            depth = depth + 1;
        case '}'
            depth = depth - 1;
            if depth == 0
                closeAt = i;
                break
            end
    end
end

% Never balanced: the object is truncated, or a literal was left open. Return the
% text untouched - see the header on why completing it would be repair.
if closeAt == 0
    return
end

if closeAt < numel(obj)
    obj = obj(1:closeAt);
    wasTrimmed = true;
end

end
