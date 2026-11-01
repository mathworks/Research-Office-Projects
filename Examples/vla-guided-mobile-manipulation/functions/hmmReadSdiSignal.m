function [value, series, times, cache] = hmmReadSdiSignal(id, cache, wantSeries)
%HMMREADSDISIGNAL Read only new scalar samples, retaining history on request.
signal = Simulink.sdi.getSignal(id);
count = signal.NumPoints;
if isempty(cache) || cache.id ~= id || count < cache.count || ...
        (wantSeries && ~cache.history)
    cache = struct('id', id, 'count', -1, 'time', -Inf, 'value', NaN, ...
        'history', wantSeries, 'series', [], 'times', []);
end
if count ~= cache.count
    if count <= 100000
        % Range-export overhead outweighs the saved copy on short logs.
        data = signal.Values;
    elseif isfinite(cache.time)
        data = export(signal, cache.time, realmax);
    else
        data = export(signal);
    end
    if ~isempty(data) && ~isempty(data.Time)
        t = double(data.Time(:));
        d = double(data.Data(:));
        assert(numel(t) == numel(d), 'hmm:sdi:NotScalar', ...
            'The live-state reader expects scalar channels.');
        cache.time = t(end);
        cache.value = d(end);
        if cache.history
            % Re-export the boundary to preserve duplicate-time discontinuities.
            keep = cache.times < t(1);
            cache.times = [cache.times(keep); t];
            cache.series = [cache.series(keep); d];
        end
    end
    cache.count = count;
end
value = cache.value;
series = [];
times = [];
if wantSeries
    series = cache.series;
    times = cache.times;
end
end
