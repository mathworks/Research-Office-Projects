function avalanches = simulateSPAD_naive(arrivals, gateVector, params)
%SIMULATESPAD_NAIVE Naive sequential SPAD simulation for validation.
%
%   avalanches = simulateSPAD_naive(arrivals, gateVector, params)
%
%   Loops through every arrival event one at a time. Produces identical
%   output to simulateSPAD when given the same RNG state.
%
%   Inputs/Outputs: same as simulateSPAD

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
    spadDeadTime = params.spadDeadTime;
    trapProb = params.trapProb;
    afterPulseDecay = params.afterPulseDecay;
    maxGens = params.maxAfterPulseGens;
    totalTime = params.nCycles * cycleTime;

    [nRows, nCols] = size(arrivals);
    nPixels = nRows * nCols;
    arrivalsFlat = arrivals(:)';
    avalanchesFlat = cell(1, nPixels);

    for px = 1:nPixels
        times = arrivalsFlat{px}.times;
        types = arrivalsFlat{px}.eventType;

        n = numel(times);
        detTimes = zeros(n, 1);
        detTypes = strings(n, 1);
        detCount = 0;
        readyTime = -inf;

        i = 1;
        while i <= n
            t = times(i);

            if t >= readyTime
                detCount = detCount + 1;
                detTimes(detCount) = t;
                detTypes(detCount) = types(i);
                readyTime = t + spadDeadTime;

                genCount = 0;
                lastAvalancheTime = t;

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

                    detCount = detCount + 1;
                    detTimes(detCount) = candidate;
                    detTypes(detCount) = "afterpulse";
                    lastAvalancheTime = candidate;
                    readyTime = candidate + spadDeadTime;
                    genCount = genCount + 1;
                end
            end

            i = i + 1;
        end

        detTimes = detTimes(1:detCount);
        detTypes = detTypes(1:detCount);

        [detTimes, sortIdx] = sort(detTimes);
        detTypes = detTypes(sortIdx);

        avalanchesFlat{px} = struct('times', detTimes, 'eventType', {detTypes});
    end

    avalanches = reshape(avalanchesFlat, nRows, nCols);

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
