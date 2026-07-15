function timestamps = digitize(avalanches, params)
%DIGITIZE Model TDC electronics: jitter, dead time, and quantization.
%
%   timestamps = digitize(avalanches, params)
%
%   Inputs:
%     avalanches - nRows x nCols cell array from simulateSPAD
%     params     - struct from defaultSPADParams
%
%   Output:
%     timestamps - nRows x nCols cell array. Each cell is a struct with:
%       .timestamps - final quantized detection times (s)
%       .rawTimes   - pre-jitter avalanche times (s)
%       .eventType  - string array (source labels, 'dark', 'afterpulse', 'crosstalk')
%       .cycleIndex - which laser cycle each event belongs to

    arguments
        avalanches cell {mustBeNonempty}
        params (1,1) struct
    end

    [nRows, nCols] = size(avalanches);
    nPixels = nRows * nCols;
    avalanchesFlat = avalanches(:)';

    spadSigma = params.spadJitterFWHM / 2.355;
    tdcSigma = params.tdcJitterFWHM / 2.355;
    tdcDeadTime = params.tdcDeadTime;
    tdcResolution = params.tdcResolution;
    cycleTime = params.cycleTime;
    maxHitsPerCycle = params.maxHitsPerCycle;

    timestampsFlat = cell(1, nPixels);

    useParallel = params.useParallel && nPixels > 1 && ~isempty(ver('parallel'));

    if useParallel
        parfor px = 1:nPixels
            timestampsFlat{px} = digitizePixel(avalanchesFlat{px}, ...
                spadSigma, tdcSigma, tdcDeadTime, tdcResolution, cycleTime, maxHitsPerCycle);
        end
    else
        for px = 1:nPixels
            timestampsFlat{px} = digitizePixel(avalanchesFlat{px}, ...
                spadSigma, tdcSigma, tdcDeadTime, tdcResolution, cycleTime, maxHitsPerCycle);
        end
    end

    timestamps = reshape(timestampsFlat, nRows, nCols);

end

function s = digitizePixel(pixelAvalanches, spadSigma, tdcSigma, ...
    tdcDeadTime, tdcResolution, cycleTime, maxHitsPerCycle)

    times = pixelAvalanches.times;
    n = numel(times);

    if n == 0
        s.timestamps = zeros(0, 1);
        s.rawTimes = zeros(0, 1);
        s.eventType = strings(0, 1);
        s.cycleIndex = zeros(0, 1);
        return;
    end

    jitteredTimes = times + spadSigma * randn(n, 1) + tdcSigma * randn(n, 1);

    [jitteredTimes, sortOrder] = sort(jitteredTimes);
    times = times(sortOrder);
    pixelAvalanches.eventType = pixelAvalanches.eventType(sortOrder);

    detIdx = tdcDeadTimeFilter(jitteredTimes, tdcDeadTime);

    jitteredTimes = jitteredTimes(detIdx);
    rawTimes = times(detIdx);
    eventType = pixelAvalanches.eventType(detIdx);
    cycleIndex = floor(rawTimes / cycleTime) + 1;

    % Enforce max hits per cycle
    if isfinite(maxHitsPerCycle)
        keep = enforceMaxHits(cycleIndex, maxHitsPerCycle);
        jitteredTimes = jitteredTimes(keep);
        rawTimes = rawTimes(keep);
        eventType = eventType(keep);
        cycleIndex = cycleIndex(keep);
    end

    s.timestamps = floor(jitteredTimes / tdcResolution) * tdcResolution;
    s.rawTimes = rawTimes;
    s.eventType = eventType;
    s.cycleIndex = cycleIndex;

end

function keep = enforceMaxHits(cycleIndex, maxHits)
% Keep only the first maxHits events per cycle (data is already sorted by time).
    n = numel(cycleIndex);
    keep = true(n, 1);
    count = 1;
    currentCycle = cycleIndex(1);
    for i = 2:n
        if cycleIndex(i) == currentCycle
            count = count + 1;
            if count > maxHits
                keep(i) = false;
            end
        else
            currentCycle = cycleIndex(i);
            count = 1;
        end
    end
end

function detIdx = tdcDeadTimeFilter(times, deadTime)

    n = numel(times);
    if n == 0
        detIdx = zeros(0, 1);
        return;
    end

    detIdx = zeros(n, 1);
    count = 1;
    detIdx(1) = 1;
    readyTime = times(1) + deadTime;

    pos = 1;
    while pos <= n
        lo = pos + 1;
        hi = n;
        next = 0;
        while lo <= hi
            mid = floor((lo + hi) / 2);
            if times(mid) >= readyTime
                next = mid;
                hi = mid - 1;
            else
                lo = mid + 1;
            end
        end
        if next == 0
            break;
        end
        count = count + 1;
        detIdx(count) = next;
        readyTime = times(next) + deadTime;
        pos = next;
    end

    detIdx = detIdx(1:count);

end
