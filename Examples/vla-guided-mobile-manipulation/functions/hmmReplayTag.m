function tag = hmmReplayTag(instruction)
%HMMREPLAYTAG Deterministic name tag that binds a recording to an instruction.
%
%   tag = HMMREPLAYTAG(instruction) returns the eight lowercase hex digits that
%   must appear in a replay recording's file name for hmmProviderRequest to serve
%   it for that instruction. The instruction is lowercased and trimmed first, so
%   trailing whitespace and capitalisation do not change which file is picked.
%
%   ONE IMPLEMENTATION FOR BOTH ENDS. hmmProviderRequest uses it to look a
%   recording up, and the sample responses were named with it. If the two hashed
%   the instruction differently, a response would be saved under a name the
%   provider could never find, and the symptom would be a silent fall back to the
%   alphabetically first recording - a right-looking answer to the wrong prompt.
%
%   FNV-1a 32-bit is used because it is short, stable across releases and
%   trivially reimplementable. Nothing about it is cryptographic: it names a
%   fixture, it does not protect anything.
%
%   See also hmmProviderRequest.

%   Copyright 2026 The MathWorks, Inc.

arguments
    instruction (1, :) char
end

s = lower(strtrim(instruction));

h = uint32(2166136261);
for k = 1:numel(s)
    h = bitxor(h, uint32(double(s(k))));
    h = uint32(mod(double(h) * 16777619, 2^32));
end

tag = sprintf('%08x', h);

end
