function avalanches = simulateSPAD(arrivals, gateVector, params)
%SIMULATESPAD Model SPAD avalanche physics with dead time and after-pulsing.
%
%   avalanches = simulateSPAD(arrivals, gateVector, params)
%
%   Inputs:
%     arrivals   - nRows x nCols cell array from generateArrivals
%     gateVector - 1 x nBins gate modulation (0 to 1), or [] for free-running
%     params     - struct from defaultSPADParams
%
%   Output:
%     avalanches - nRows x nCols cell array. Each cell is a struct with:
%       .times     - sorted absolute avalanche times (s)
%       .eventType - string array (source labels, 'dark', 'afterpulse', 'crosstalk')

    arguments
        arrivals cell {mustBeNonempty}
        gateVector {mustBeNumeric}
        params (1,1) struct
    end

    if isempty(gateVector)
        gateVector = ones(1, round(params.cycleTime / params.binWidth));
    end

    nBins = numel(gateVector);
    binWidth = params.binWidth;
    cycleTime = params.cycleTime;
    vectorSpan = nBins * binWidth;

    [nRows, nCols] = size(arrivals);
    nPixels = nRows * nCols;

    % Build dynamic type map from arrival labels
    typeMap = buildTypeMap(arrivals);

    if any(params.crosstalkProb > 0, 'all')
        [neighborMap, neighborProbs] = buildNeighborMap(nRows, nCols, params.crosstalkProb, params.crosstalkMap);
        if params.exactCrosstalk
            avalanches = simulateExactCrosstalk(arrivals, gateVector, params, ...
                nBins, binWidth, cycleTime, vectorSpan, neighborMap, neighborProbs, typeMap);
        else
            avalanches = simulateApproxCrosstalk(arrivals, gateVector, params, ...
                nBins, binWidth, cycleTime, vectorSpan, neighborMap, neighborProbs, typeMap);
        end
        return;
    end

    arrivalsFlat = arrivals(:)';
    avalanchesFlat = cell(1, nPixels);

    useParallel = params.useParallel && nPixels > 1 && ~isempty(ver('parallel'));

    if useParallel
        if ~isempty(params.seed)
            streams = createStreams(params.seed, nPixels);
        else
            streams = cell(1, nPixels);
        end
    end

    if useParallel
        parfor px = 1:nPixels
            stream = streams{px};
            avalanchesFlat{px} = processPixelWrapper(arrivalsFlat{px}, ...
                gateVector, params, nBins, binWidth, cycleTime, vectorSpan, stream, typeMap);
        end
    else
        for px = 1:nPixels
            avalanchesFlat{px} = processPixelWrapper(arrivalsFlat{px}, ...
                gateVector, params, nBins, binWidth, cycleTime, vectorSpan, [], typeMap);
        end
    end

    avalanches = reshape(avalanchesFlat, nRows, nCols);

end

%% --- Approximate crosstalk (parallel per-pixel, generational injection) ---

function avalanches = simulateApproxCrosstalk(arrivals, gateVector, params, ...
    nBins, binWidth, cycleTime, vectorSpan, neighborMap, neighborProbs, typeMap)

    [nRows, nCols] = size(arrivals);
    nPixels = nRows * nCols;
    arrivalsFlat = arrivals(:)';

    % Gen 0: process all pixels independently
    avalanchesFlat = cell(1, nPixels);
    for px = 1:nPixels
        avalanchesFlat{px} = processPixelWrapper(arrivalsFlat{px}, ...
            gateVector, params, nBins, binWidth, cycleTime, vectorSpan, [], typeMap);
    end

    maxGens = 10;
    spadDeadTime = params.spadDeadTime;
    ctCode = uint8(find(typeMap == "crosstalk"));

    % Store times and codes separately for efficiency
    pixTimes = cell(1, nPixels);
    pixCodes = cell(1, nPixels);
    for px = 1:nPixels
        pixTimes{px} = avalanchesFlat{px}.times;
        nEv = numel(avalanchesFlat{px}.eventType);
        codes = zeros(nEv, 1, 'uint8');
        for k = 1:numel(typeMap)
            codes(avalanchesFlat{px}.eventType == typeMap(k)) = k;
        end
        pixCodes{px} = codes;
    end

    % Track new detections per generation (just times for propagation)
    newTimes = pixTimes;

    for gen = 1:maxGens
        % Collect crosstalk candidates for each pixel from neighbors
        crosstalkEvents = cell(1, nPixels);

        for px = 1:nPixels
            srcTimes = newTimes{px};
            if isempty(srcTimes), continue; end
            neighbors = neighborMap{px};
            probs = neighborProbs{px};
            for ni = 1:numel(neighbors)
                nbr = neighbors(ni);
                hits = rand(numel(srcTimes), 1) < probs(ni);
                if any(hits)
                    crosstalkEvents{nbr} = [crosstalkEvents{nbr}; srcTimes(hits)];
                end
            end
        end

        totalNew = 0;
        newTimes = cell(1, nPixels);

        for px = 1:nPixels
            ctTimes = crosstalkEvents{px};
            if isempty(ctTimes)
                newTimes{px} = zeros(0, 1);
                continue;
            end

            ctTimes = sort(ctTimes);

            % Gate check (vectorized)
            tInCycle = mod(ctTimes, cycleTime);
            bidx = min(floor(tInCycle / binWidth) + 1, nBins);
            outOfRange = tInCycle >= nBins * binWidth;
            gateVals = gateVector(bidx)';
            gateVals(outOfRange) = 0;
            gateAccept = rand(numel(ctTimes), 1) < gateVals;
            ctTimes = ctTimes(gateAccept);

            if isempty(ctTimes)
                newTimes{px} = zeros(0, 1);
                continue;
            end

            % Merge with existing and re-apply dead time
            nExisting = numel(pixTimes{px});
            nCT = numel(ctTimes);
            mergedTimes = [pixTimes{px}; ctTimes];
            mergedCodes = [pixCodes{px}; repmat(ctCode, nCT, 1)];
            [mergedTimes, si] = sort(mergedTimes);
            mergedCodes = mergedCodes(si);

            % Track which merged entries are new candidates
            isNewEntry = si > nExisting;

            [filtTimes, filtCodes, filtKeep] = applyDeadTimeUint8(mergedTimes, mergedCodes, spadDeadTime);

            % New detections = new entries that survived dead-time filtering
            newMask = isNewEntry(filtKeep);
            newTimes{px} = filtTimes(newMask);
            totalNew = totalNew + sum(newMask);

            pixTimes{px} = filtTimes;
            pixCodes{px} = filtCodes;
        end

        if totalNew == 0
            break;
        end
    end

    % Package output
    for px = 1:nPixels
        avalanchesFlat{px} = struct('times', pixTimes{px}, ...
            'eventType', {typeMap(pixCodes{px})});
    end

    avalanches = reshape(avalanchesFlat, nRows, nCols);

end

%% --- Exact crosstalk (serial all-pixel timeline) ---

function avalanches = simulateExactCrosstalk(arrivals, gateVector, params, ...
    nBins, binWidth, cycleTime, vectorSpan, neighborMap, neighborProbs, typeMap)

    [nRows, nCols] = size(arrivals);
    nPixels = nRows * nCols;
    arrivalsFlat = arrivals(:)';

    spadDeadTime = params.spadDeadTime;
    trapProb = params.trapProb;
    afterPulseDecay = params.afterPulseDecay;
    maxAfterPulseGens = params.maxAfterPulseGens;
    totalTime = params.nCycles * cycleTime;

    apCode = uint8(find(typeMap == "afterpulse"));
    ctCode = uint8(find(typeMap == "crosstalk"));

    % Build global sorted event list with pixel labels
    allTimes = cell(nPixels, 1);
    allCodes = cell(nPixels, 1);
    allPixelIdx = cell(nPixels, 1);
    for px = 1:nPixels
        nEv = numel(arrivalsFlat{px}.times);
        allTimes{px} = arrivalsFlat{px}.times;
        codes = zeros(nEv, 1, 'uint8');
        for k = 1:numel(typeMap)
            codes(arrivalsFlat{px}.eventType == typeMap(k)) = k;
        end
        allCodes{px} = codes;
        allPixelIdx{px} = uint32(px) * ones(nEv, 1, 'uint32');
    end
    eventTimes = vertcat(allTimes{:});
    eventCodes = vertcat(allCodes{:});
    eventPixels = vertcat(allPixelIdx{:});

    [eventTimes, sortIdx] = sort(eventTimes);
    eventCodes = eventCodes(sortIdx);
    eventPixels = eventPixels(sortIdx);

    nEvents = numel(eventTimes);

    % Per-pixel state
    readyTime = -inf(nPixels, 1);

    % Output storage
    detTimes = cell(nPixels, 1);
    detCodes = cell(nPixels, 1);
    detCounts = zeros(nPixels, 1);
    estDet = round(params.nCycles * 1.2);
    for px = 1:nPixels
        detTimes{px} = zeros(estDet, 1);
        detCodes{px} = zeros(estDet, 1, 'uint8');
    end

    % Pending crosstalk events (priority queue via sorted insert)
    pendingTimes = zeros(1000, 1);
    pendingPixels = zeros(1000, 1, 'uint32');
    pendingCount = 0;

    eventPos = 1;

    while true
        nextArrivalTime = inf;
        if eventPos <= nEvents
            nextArrivalTime = eventTimes(eventPos);
        end

        nextPendingTime = inf;
        if pendingCount > 0
            nextPendingTime = pendingTimes(1);
        end

        if nextArrivalTime == inf && nextPendingTime == inf
            break;
        end

        if nextArrivalTime <= nextPendingTime
            t = eventTimes(eventPos);
            px = double(eventPixels(eventPos));
            evCode = eventCodes(eventPos);
            eventPos = eventPos + 1;
        else
            t = pendingTimes(1);
            px = double(pendingPixels(1));
            evCode = ctCode;
            pendingTimes(1:pendingCount-1) = pendingTimes(2:pendingCount);
            pendingPixels(1:pendingCount-1) = pendingPixels(2:pendingCount);
            pendingCount = pendingCount - 1;
        end

        if t < readyTime(px)
            continue;
        end

        % Gate check for crosstalk events
        if evCode == ctCode
            gateVal = lookupGate(t, gateVector, nBins, binWidth, cycleTime, vectorSpan);
            if rand() >= gateVal
                continue;
            end
        end

        % Avalanche fires
        detCounts(px) = detCounts(px) + 1;
        dc = detCounts(px);
        if dc > numel(detTimes{px})
            detTimes{px} = [detTimes{px}; zeros(estDet, 1)];
            detCodes{px} = [detCodes{px}; zeros(estDet, 1, 'uint8')];
        end
        detTimes{px}(dc) = t;
        detCodes{px}(dc) = evCode;
        readyTime(px) = t + spadDeadTime;

        % Crosstalk from this avalanche to neighbors
        neighbors = neighborMap{px};
        probs = neighborProbs{px};
        for ni = 1:numel(neighbors)
            if rand() < probs(ni)
                nbr = neighbors(ni);
                pendingCount = pendingCount + 1;
                if pendingCount > numel(pendingTimes)
                    pendingTimes = [pendingTimes; zeros(1000, 1)];
                    pendingPixels = [pendingPixels; zeros(1000, 1, 'uint32')];
                end
                insertPos = pendingCount;
                while insertPos > 1 && pendingTimes(insertPos-1) > t
                    pendingTimes(insertPos) = pendingTimes(insertPos-1);
                    pendingPixels(insertPos) = pendingPixels(insertPos-1);
                    insertPos = insertPos - 1;
                end
                pendingTimes(insertPos) = t;
                pendingPixels(insertPos) = uint32(nbr);
            end
        end

        % After-pulse chain (candidates drawn from avalanche time, rejected if in dead time)
        if trapProb > 0
            lastAvalancheTime = t;
            genCount = 0;
            while genCount < maxAfterPulseGens
                if rand() >= trapProb
                    break;
                end
                candidate = lastAvalancheTime + exprnd(afterPulseDecay);
                if candidate >= totalTime
                    break;
                end
                if candidate < readyTime(px)
                    break;
                end
                gateVal = lookupGate(candidate, gateVector, nBins, binWidth, cycleTime, vectorSpan);
                if rand() >= gateVal
                    break;
                end
                lastAvalancheTime = candidate;
                readyTime(px) = candidate + spadDeadTime;
                genCount = genCount + 1;

                detCounts(px) = detCounts(px) + 1;
                dc = detCounts(px);
                if dc > numel(detTimes{px})
                    detTimes{px} = [detTimes{px}; zeros(estDet, 1)];
                    detCodes{px} = [detCodes{px}; zeros(estDet, 1, 'uint8')];
                end
                detTimes{px}(dc) = candidate;
                detCodes{px}(dc) = apCode;

                % After-pulse also generates crosstalk
                for ni = 1:numel(neighbors)
                    if rand() < probs(ni)
                        nbr = neighbors(ni);
                        pendingCount = pendingCount + 1;
                        if pendingCount > numel(pendingTimes)
                            pendingTimes = [pendingTimes; zeros(1000, 1)];
                            pendingPixels = [pendingPixels; zeros(1000, 1, 'uint32')];
                        end
                        insertPos = pendingCount;
                        while insertPos > 1 && pendingTimes(insertPos-1) > candidate
                            pendingTimes(insertPos) = pendingTimes(insertPos-1);
                            pendingPixels(insertPos) = pendingPixels(insertPos-1);
                            insertPos = insertPos - 1;
                        end
                        pendingTimes(insertPos) = candidate;
                        pendingPixels(insertPos) = uint32(nbr);
                    end
                end
            end
        end
    end

    % Package output
    avalanchesFlat = cell(1, nPixels);
    for px = 1:nPixels
        dc = detCounts(px);
        t = detTimes{px}(1:dc);
        c = detCodes{px}(1:dc);
        [t, si] = sort(t);
        c = c(si);
        avalanchesFlat{px} = struct('times', t, 'eventType', {typeMap(c)});
    end

    avalanches = reshape(avalanchesFlat, nRows, nCols);

end

%% --- Helpers ---

function s = processPixelWrapper(pixelArrivals, gateVector, params, ...
    nBins, binWidth, cycleTime, vectorSpan, stream, typeMap)

    if ~isempty(stream)
        oldStream = RandStream.setGlobalStream(stream);
    end

    % Convert string types to numeric codes using typeMap
    nEvents = numel(pixelArrivals.eventType);
    typeCodes = zeros(nEvents, 1, 'uint8');
    for i = 1:numel(typeMap)
        typeCodes(pixelArrivals.eventType == typeMap(i)) = i;
    end

    [detTimes, detCodes] = processPixel(pixelArrivals.times, ...
        typeCodes, gateVector, params, nBins, binWidth, ...
        cycleTime, vectorSpan, typeMap);

    s.times = detTimes;
    s.eventType = typeMap(detCodes);

    if ~isempty(stream)
        RandStream.setGlobalStream(oldStream);
    end

end

function [detTimes, detCodes] = processPixel(times, typeCodes, ...
    gateVector, params, nBins, binWidth, cycleTime, vectorSpan, typeMap)

    apCode = uint8(find(typeMap == "afterpulse"));

    if params.exactAfterPulse
        [detTimes, detCodes] = processPixelExact(times, typeCodes, ...
            gateVector, params, nBins, binWidth, cycleTime, vectorSpan, apCode);
    else
        [detTimes, detCodes] = processPixelApprox(times, typeCodes, ...
            gateVector, params, nBins, binWidth, cycleTime, vectorSpan, apCode);
    end

end

function [detTimes, detCodes] = processPixelExact(times, typeCodes, ...
    gateVector, params, nBins, binWidth, cycleTime, vectorSpan, apCode)

    spadDeadTime = params.spadDeadTime;
    trapProb = params.trapProb;
    afterPulseDecay = params.afterPulseDecay;
    maxGens = params.maxAfterPulseGens;
    totalTime = params.nCycles * cycleTime;
    n = numel(times);

    if n == 0
        detTimes = zeros(0, 1);
        detCodes = zeros(0, 1, 'uint8');
        return;
    end

    estDet = min(n, round(params.nCycles * 1.2));
    detTimes = zeros(estDet, 1);
    detCodes = zeros(estDet, 1, 'uint8');
    detCount = 0;

    readyTime = -inf;
    pos = 1;

    while pos <= n
        if times(pos) < readyTime
            lo = pos;
            hi = n;
            nextPos = 0;
            while lo <= hi
                mid = floor((lo + hi) / 2);
                if times(mid) >= readyTime
                    nextPos = mid;
                    hi = mid - 1;
                else
                    lo = mid + 1;
                end
            end
            if nextPos == 0
                break;
            end
            pos = nextPos;
        end

        detCount = detCount + 1;
        if detCount > numel(detTimes)
            detTimes = [detTimes; zeros(estDet, 1)];
            detCodes = [detCodes; zeros(estDet, 1, 'uint8')];
        end
        detTimes(detCount) = times(pos);
        detCodes(detCount) = typeCodes(pos);
        readyTime = times(pos) + spadDeadTime;

        if trapProb > 0
            lastAvalancheTime = times(pos);
            genCount = 0;
            while genCount < maxGens
                if rand() >= trapProb
                    break;
                end

                candidate = lastAvalancheTime + exprnd(afterPulseDecay);

                if candidate >= totalTime
                    break;
                end

                if candidate < readyTime
                    break;
                end

                gateVal = lookupGate(candidate, gateVector, nBins, binWidth, cycleTime, vectorSpan);
                if rand() >= gateVal
                    break;
                end

                lastAvalancheTime = candidate;
                readyTime = candidate + spadDeadTime;
                genCount = genCount + 1;

                detCount = detCount + 1;
                if detCount > numel(detTimes)
                    detTimes = [detTimes; zeros(estDet, 1)];
                    detCodes = [detCodes; zeros(estDet, 1, 'uint8')];
                end
                detTimes(detCount) = candidate;
                detCodes(detCount) = apCode;
            end
        end

        pos = pos + 1;
    end

    detTimes = detTimes(1:detCount);
    detCodes = detCodes(1:detCount);

    [detTimes, sortIdx] = sort(detTimes);
    detCodes = detCodes(sortIdx);

end

function [detTimes, detCodes] = processPixelApprox(times, typeCodes, ...
    gateVector, params, nBins, binWidth, cycleTime, vectorSpan, apCode)

    spadDeadTime = params.spadDeadTime;
    trapProb = params.trapProb;
    afterPulseDecay = params.afterPulseDecay;
    totalTime = params.nCycles * cycleTime;
    n = numel(times);

    if n == 0
        detTimes = zeros(0, 1);
        detCodes = zeros(0, 1, 'uint8');
        return;
    end

    % --- Dead time filter ---
    estDet = min(n, round(params.nCycles * 1.2));
    dtTimes = zeros(estDet, 1);
    dtCodes = zeros(estDet, 1, 'uint8');
    dtCount = 0;

    readyTime = -inf;
    pos = 1;

    while pos <= n
        if times(pos) < readyTime
            lo = pos;
            hi = n;
            nextPos = 0;
            while lo <= hi
                mid = floor((lo + hi) / 2);
                if times(mid) >= readyTime
                    nextPos = mid;
                    hi = mid - 1;
                else
                    lo = mid + 1;
                end
            end
            if nextPos == 0
                break;
            end
            pos = nextPos;
        end

        dtCount = dtCount + 1;
        if dtCount > numel(dtTimes)
            dtTimes = [dtTimes; zeros(estDet, 1)];
            dtCodes = [dtCodes; zeros(estDet, 1, 'uint8')];
        end
        dtTimes(dtCount) = times(pos);
        dtCodes(dtCount) = typeCodes(pos);
        readyTime = times(pos) + spadDeadTime;
        pos = pos + 1;
    end

    dtTimes = dtTimes(1:dtCount);
    dtCodes = dtCodes(1:dtCount);

    if trapProb == 0 || dtCount == 0
        detTimes = dtTimes;
        detCodes = dtCodes;
        return;
    end

    % --- Vectorized after-pulse injection (single generation) ---
    apTriggered = rand(dtCount, 1) < trapProb;
    nAP = sum(apTriggered);

    if nAP == 0
        detTimes = dtTimes;
        detCodes = dtCodes;
        return;
    end

    % Draw from avalanche time; dead time naturally rejects early candidates
    apTimes = dtTimes(apTriggered) + exprnd(afterPulseDecay, nAP, 1);

    % Gate check (vectorized)
    tInCycle = mod(apTimes, cycleTime);
    binIdx = min(floor(tInCycle / binWidth) + 1, nBins);
    outOfRange = tInCycle >= nBins * binWidth;
    apGateVals = gateVector(binIdx)';
    apGateVals(outOfRange) = 0;
    gateAccept = rand(nAP, 1) < apGateVals;

    % Time bounds check
    validTime = apTimes < totalTime;
    keep = gateAccept & validTime;

    apTimes = apTimes(keep);
    nAP = numel(apTimes);

    if nAP == 0
        detTimes = dtTimes;
        detCodes = dtCodes;
        return;
    end

    % Merge and re-apply dead time (rejects after-pulses landing in dead time)
    mergedTimes = [dtTimes; apTimes];
    mergedCodes = [dtCodes; repmat(apCode, nAP, 1)];
    [mergedTimes, si] = sort(mergedTimes);
    mergedCodes = mergedCodes(si);

    % Dead time filter on merged list
    keep = true(numel(mergedTimes), 1);
    readyTime = mergedTimes(1) + spadDeadTime;
    for i = 2:numel(mergedTimes)
        if mergedTimes(i) < readyTime
            keep(i) = false;
        else
            readyTime = mergedTimes(i) + spadDeadTime;
        end
    end

    detTimes = mergedTimes(keep);
    detCodes = mergedCodes(keep);

end


function [filtTimes, filtCodes, filtKeep] = applyDeadTimeUint8(times, codes, deadTime)
    n = numel(times);
    if n == 0
        filtTimes = zeros(0, 1);
        filtCodes = zeros(0, 1, 'uint8');
        filtKeep = zeros(0, 1, 'logical');
        return;
    end
    keep = true(n, 1);
    readyTime = times(1) + deadTime;
    for i = 2:n
        if times(i) < readyTime
            keep(i) = false;
        else
            readyTime = times(i) + deadTime;
        end
    end
    filtTimes = times(keep);
    filtCodes = codes(keep);
    filtKeep = find(keep);
end

function [neighborMap, neighborProbs] = buildNeighborMap(nRows, nCols, crosstalkProb, crosstalkMap)
% Build neighbor connectivity and per-neighbor crosstalk probabilities.
%   crosstalkProb: scalar (uniform to all neighbors) or kernel matrix (odd-sized,
%                  center = self, each entry is probability at that offset).
%   crosstalkMap: explicit nPixels x nPixels connectivity (overrides default grid).
%                 Only used when crosstalkProb is scalar.
    nPixels = nRows * nCols;
    isKernel = ~isscalar(crosstalkProb);

    if isKernel
        kernel = crosstalkProb;
        [kRows, kCols] = size(kernel);
        cr = (kRows + 1) / 2;
        cc = (kCols + 1) / 2;

        neighborMap = cell(1, nPixels);
        neighborProbs = cell(1, nPixels);
        for px = 1:nPixels
            [r, c] = ind2sub([nRows, nCols], px);
            nbrs = [];
            probs = [];
            for dr = 1:kRows
                for dc = 1:kCols
                    if dr == cr && dc == cc, continue; end
                    p = kernel(dr, dc);
                    if p <= 0, continue; end
                    nr = r + (dr - cr);
                    nc = c + (dc - cc);
                    if nr >= 1 && nr <= nRows && nc >= 1 && nc <= nCols
                        nbrs(end+1) = sub2ind([nRows, nCols], nr, nc); %#ok<AGROW>
                        probs(end+1) = p; %#ok<AGROW>
                    end
                end
            end
            neighborMap{px} = nbrs;
            neighborProbs{px} = probs;
        end
        return;
    end

    % Scalar crosstalkProb
    if ~isempty(crosstalkMap)
        neighborMap = cell(1, nPixels);
        neighborProbs = cell(1, nPixels);
        for px = 1:nPixels
            nbrs = find(crosstalkMap(px, :));
            neighborMap{px} = nbrs;
            neighborProbs{px} = crosstalkProb * ones(1, numel(nbrs));
        end
        return;
    end

    neighborMap = cell(1, nPixels);
    neighborProbs = cell(1, nPixels);
    for px = 1:nPixels
        [r, c] = ind2sub([nRows, nCols], px);
        nbrs = [];
        if r > 1,     nbrs(end+1) = sub2ind([nRows, nCols], r-1, c); end
        if r < nRows, nbrs(end+1) = sub2ind([nRows, nCols], r+1, c); end
        if c > 1,     nbrs(end+1) = sub2ind([nRows, nCols], r, c-1); end
        if c < nCols, nbrs(end+1) = sub2ind([nRows, nCols], r, c+1); end
        neighborMap{px} = nbrs;
        neighborProbs{px} = crosstalkProb * ones(1, numel(nbrs));
    end
end

function typeMap = buildTypeMap(arrivals)
% Build type map: [source labels..., "dark", "afterpulse", "crosstalk"]
% Discovers source labels from the arrival data.
    allTypes = strings(0, 1);
    for i = 1:numel(arrivals)
        if ~isempty(arrivals{i}.eventType)
            allTypes = [allTypes; arrivals{i}.eventType];
        end
    end
    sourceLabels = unique(allTypes, 'stable');
    sourceLabels(sourceLabels == "dark") = [];
    % Build map: sources first, then dark, afterpulse, crosstalk
    typeMap = [sourceLabels; "dark"; "afterpulse"; "crosstalk"];
end

function gateVal = lookupGate(t, gateVector, nBins, binWidth, cycleTime, vectorSpan)

    tInCycle = mod(t, cycleTime);
    if tInCycle >= vectorSpan
        gateVal = 0;
        return;
    end
    binIdx = floor(tInCycle / binWidth) + 1;
    binIdx = min(binIdx, nBins);
    gateVal = gateVector(binIdx);

end

function streams = createStreams(seed, nPixels)
    streams = cell(1, nPixels);
    for px = 1:nPixels
        streams{px} = RandStream('Threefry', 'Seed', seed + px - 1);
    end
end
