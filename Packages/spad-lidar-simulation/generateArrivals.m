function arrivals = generateArrivals(rateVector, gateVector, params)
%GENERATEARRIVALS Generate photon and dark count arrivals on absolute timeline.
%
%   arrivals = generateArrivals(rateVector, gateVector, params)
%
%   Inputs:
%     rateVector - One of:
%       (a) nRows x nCols x nBins array of photon arrival rates (photons/s)
%           Labeled as "signal" by default.
%       (b) struct array with fields:
%           .rate  - nRows x nCols x nBins array of arrival rates (photons/s)
%           .label - string label for this source (e.g. "laser", "background")
%     gateVector - 1 x nBins gate modulation (0 to 1), or [] for free-running
%     params     - struct from defaultSPADParams
%
%   Output:
%     arrivals - nRows x nCols cell array. Each cell is a struct with:
%       .times     - sorted absolute arrival times (s)
%       .eventType - string array with source labels or 'dark'

    arguments
        rateVector {mustBeNonempty}
        gateVector {mustBeNumeric}
        params (1,1) struct
    end

    % Normalize input to struct array
    if isstruct(rateVector)
        sources = rateVector;
    else
        sources.rate = rateVector;
        sources.label = "signal";
    end

    nSources = numel(sources);

    % Get array dimensions from first source
    arraySize = size(sources(1).rate);
    nBins = arraySize(end);
    if ndims(sources(1).rate) == 3
        nRows = arraySize(1);
        nCols = arraySize(2);
    elseif ismatrix(sources(1).rate) && arraySize(1) == 1
        nRows = 1;
        nCols = 1;
    else
        nRows = arraySize(1);
        nCols = 1;
    end
    nPixels = nRows * nCols;

    if isempty(gateVector)
        gateVector = ones(1, nBins);
    end

    binWidth = params.binWidth;
    cycleTime = params.cycleTime;
    nCycles = params.nCycles;

    % Compute per-source effective mean counts
    sourceMeans = cell(nSources, 1);
    for si = 1:nSources
        rate2D = reshape(sources(si).rate, nPixels, nBins);
        sourceMeans{si} = rate2D .* params.pde .* gateVector * binWidth; % nPixels x nBins
    end

    % Dark count mean
    darkMean = params.dcr * gateVector * binWidth; % 1 x nBins

    % Build dark CDF (only active bins)
    darkTotal = sum(darkMean);
    if darkTotal > 0
        darkActive = find(darkMean > 0);
        darkProbs = darkMean(darkActive) / darkTotal;
        darkEdges = [0, cumsum(darkProbs)];
        darkEdges(end) = 1;
    else
        darkEdges = [];
        darkActive = [];
    end

    arrivalsFlat = cell(1, nPixels);

    useParallel = params.useParallel && nPixels > 1 && ~isempty(ver('parallel'));

    if useParallel
        if ~isempty(params.seed)
            streams = createStreams(params.seed, nPixels);
        else
            streams = cell(1, nPixels);
        end
    end

    % Extract source labels
    sourceLabels = strings(nSources, 1);
    for si = 1:nSources
        sourceLabels(si) = sources(si).label;
    end

    if useParallel
        parfor px = 1:nPixels
            stream = streams{px};
            pixSourceMeans = cell(nSources, 1);
            for si = 1:nSources
                pixSourceMeans{si} = sourceMeans{si}(px,:);
            end
            arrivalsFlat{px} = generatePixelArrivals(pixSourceMeans, sourceLabels, ...
                darkEdges, darkActive, darkTotal, ...
                nBins, nCycles, binWidth, cycleTime, stream);
        end
    else
        if ~isempty(params.seed)
            rng(params.seed);
        end
        for px = 1:nPixels
            pixSourceMeans = cell(nSources, 1);
            for si = 1:nSources
                pixSourceMeans{si} = sourceMeans{si}(px,:);
            end
            arrivalsFlat{px} = generatePixelArrivals(pixSourceMeans, sourceLabels, ...
                darkEdges, darkActive, darkTotal, ...
                nBins, nCycles, binWidth, cycleTime, []);
        end
    end

    arrivals = reshape(arrivalsFlat, nRows, nCols);

end

function s = generatePixelArrivals(pixSourceMeans, sourceLabels, ...
    darkEdges, darkActive, darkTotal, nBins, nCycles, binWidth, cycleTime, stream)

    if ~isempty(stream)
        oldStream = RandStream.setGlobalStream(stream);
    end

    nSources = numel(sourceLabels);
    allTimes = cell(nSources + 1, 1);
    allTypes = cell(nSources + 1, 1);

    % Generate arrivals for each source
    for si = 1:nSources
        srcTimes = generateFromCDF(pixSourceMeans{si}, nBins, nCycles, binWidth, cycleTime);
        allTimes{si} = srcTimes;
        allTypes{si} = repmat(sourceLabels(si), numel(srcTimes), 1);
    end

    % Dark counts
    darkTimes = generateFromCDFShared(darkEdges, darkActive, darkTotal, nBins, nCycles, binWidth, cycleTime);
    allTimes{nSources + 1} = darkTimes;
    allTypes{nSources + 1} = repmat("dark", numel(darkTimes), 1);

    % Merge and sort
    combinedTimes = vertcat(allTimes{:});
    combinedTypes = vertcat(allTypes{:});

    [combinedTimes, sortIdx] = sort(combinedTimes);
    combinedTypes = combinedTypes(sortIdx);

    s.times = combinedTimes;
    s.eventType = combinedTypes;

    if ~isempty(stream)
        RandStream.setGlobalStream(oldStream);
    end

end

function times = generateFromCDF(meanCounts, nBins, nCycles, binWidth, cycleTime)

    totalMean = sum(meanCounts) * nCycles;
    if totalMean == 0
        times = zeros(0, 1);
        return;
    end

    nArr = poissrnd(totalMean);
    if nArr == 0
        times = zeros(0, 1);
        return;
    end

    % CDF for bin assignment — only include non-zero bins
    activeMask = meanCounts > 0;
    activeBins = find(activeMask);
    activeProbs = meanCounts(activeMask) / sum(meanCounts);
    edges = [0, cumsum(activeProbs)];
    edges(end) = 1;

    % Assign each arrival to an active bin and cycle
    activeIdx = discretize(rand(nArr, 1), edges);
    binAssign = activeBins(activeIdx)';
    cycleAssign = randi(nCycles, nArr, 1) - 1; % 0-based

    % Compute absolute time
    times = cycleAssign * cycleTime + (binAssign - 1) * binWidth + rand(nArr, 1) * binWidth;

end

function times = generateFromCDFShared(edges, activeBins, totalPerCycle, nBins, nCycles, binWidth, cycleTime)

    if isempty(edges) || totalPerCycle == 0
        times = zeros(0, 1);
        return;
    end

    totalMean = totalPerCycle * nCycles;
    nArr = poissrnd(totalMean);
    if nArr == 0
        times = zeros(0, 1);
        return;
    end

    activeIdx = discretize(rand(nArr, 1), edges);
    binAssign = activeBins(activeIdx)';
    cycleAssign = randi(nCycles, nArr, 1) - 1;
    times = cycleAssign * cycleTime + (binAssign - 1) * binWidth + rand(nArr, 1) * binWidth;

end

function streams = createStreams(seed, nPixels)
    streams = cell(1, nPixels);
    for px = 1:nPixels
        streams{px} = RandStream('Threefry', 'Seed', seed + px - 1);
    end
end
