function name = hmmEnumName(namespace, value, prefix)
%HMMENUMNAME Reverse-look-up a uint8 code in one of hmmConfig's enum namespaces.
%
%   name = HMMENUMNAME(cfg.sup.pl, 4) returns 'PICK'.
%
%   name = HMMENUMNAME(cfg.queue, 1, 'VERB_') returns 'VERB_TRANSFER'.
%
%   THE PREFIX IS USUALLY REQUIRED, NOT OPTIONAL POLISH. Several namespaces hold
%   MORE THAN ONE enumeration - cfg.queue carries both VERB_ and EXEC_, whose
%   codes both start at zero, and cfg.percep mixes EXTENT_ codes in among plain
%   tuning scalars like minClusterHits. Without a prefix the search returns
%   whichever field hmmConfig happens to declare first, which is a name that reads
%   as authoritative and is simply the wrong enumeration.
%
%   The model's states, verbs, outcomes, routes and failure codes are all plain
%   uint8 constants gathered into namespace structs by hmmConfig, because a
%   Stateflow chart and a MATLAB Function block can both read a struct field but
%   neither can share a classdef enumeration without generating one. That makes
%   every logged status field a bare number.
%
%   INVERTING THE STRUCT IS THE POINT. Writing the names out again in a report
%   would create a second, unchecked copy of the enumeration, and the failure
%   mode is a readout that confidently prints the wrong state name after somebody
%   inserts a code in the middle. Reading the names back out of the same struct
%   the model compiled against cannot drift.
%
%   An unmatched value is reported as '<n>?' rather than throwing: a diagnostic
%   readout that errors on unexpected data destroys the evidence it was called to
%   show.
%
%   See also hmmConfig, hmmReportRun.

%   Copyright 2026 The MathWorks, Inc.

arguments
    namespace struct
    value (1, 1) double
    prefix (1, :) char = ''
end

f = fieldnames(namespace);
for k = 1:numel(f)
    if ~startsWith(f{k}, prefix)
        continue
    end
    v = namespace.(f{k});
    if isnumeric(v) && isscalar(v) && double(v) == value
        name = f{k};
        return
    end
end

name = sprintf('%g?', value);

end
