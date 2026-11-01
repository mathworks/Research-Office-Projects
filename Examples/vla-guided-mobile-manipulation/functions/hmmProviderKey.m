function [key, route] = hmmProviderKey(mode, cfg)
%HMMPROVIDERKEY Resolve the provider credential, and report only which route won.
%
%   [key, route] = HMMPROVIDERKEY(mode, cfg) returns the credential and the route
%   that produced it:
%
%       cfg.provider.ROUTE_ENV       GEMINI_API_KEY environment variable
%       cfg.provider.ROUTE_KEYFILE   a key file found by hmmAssetRoots' walk
%       cfg.provider.ROUTE_NONE      nothing; key is ''
%
%   mode is 'auto', 'live' or 'replay'; 'replay' resolves nothing at all, because
%   a replay run must not touch a credential even if one is present.
%
%   THIS FUNCTION EXISTS SO THERE IS EXACTLY ONE CREDENTIAL PATH. Both the
%   grounding request and the model-listing diagnostic need the key, and a second
%   copy of the resolution order is a second place for it to drift or to leak.
%
%   THE CALLER'S OBLIGATIONS, which this function cannot enforce:
%       - never print, log, or return key onward
%       - never write it to a file or a model workspace
%       - never place it in a URL; it belongs in a header
%       - overwrite and clear it as soon as the request is issued
%   Only route may be reported. A route is a fact about this machine's
%   configuration; the key is a secret.
%
%   FILE NAMES ARE PATTERNS, NOT A PATH. The names searched for are generic and
%   are safe to write down. A committed absolute path would be both a leak of
%   where the secret lives and a value that only works on one machine, which is
%   why it is forbidden.
%
%   See also hmmProviderRequest, hmmAssetRoots.

%   Copyright 2026 The MathWorks, Inc.

arguments
    mode (1, :) char = ''
    cfg  struct = hmmConfig()
end

if isempty(mode)
    mode = getenv('GEMINI_PROVIDER');
end
if isempty(mode)
    mode = cfg.provider.mode;
end
mode = strtrim(mode);

key   = '';
route = cfg.provider.ROUTE_NONE;

if strcmpi(mode, 'replay')
    return
end

key = getenv('GEMINI_API_KEY');
if ~isempty(key)
    route = cfg.provider.ROUTE_ENV;
    return
end

names = {'gemini.key', 'GEMINI_API_KEY', '.gemini_api_key', 'gemini_api_key.txt'};
roots = hmmAssetRoots();
for r = 1:numel(roots)
    for n = 1:numel(names)
        p = fullfile(roots{r}, names{n});
        if ~isfile(p)
            continue
        end
        lines = splitlines(string(fileread(p)));
        lines = strtrim(lines(strlength(strtrim(lines)) > 0));
        if ~isempty(lines)
            key   = char(lines(1));
            route = cfg.provider.ROUTE_KEYFILE;
            return
        end
    end
end

end
