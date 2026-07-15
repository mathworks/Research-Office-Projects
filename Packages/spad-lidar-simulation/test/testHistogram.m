function tests = testHistogram
%TESTHISTOGRAM Tests for buildHistogram and correctPileup.
    tests = functiontests(localfunctions);
end

function testSinglePixelCounts(testCase)
    params = defaultSPADParams();
    params.nCycles = 10000;
    params.seed = 1;
    params.trapProb = 0;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(1, 1, nBins);
    gateVector = ones(1, nBins);

    result = runSPADSimulation(rateVector, gateVector, params);
    [counts, binEdges, binCenters] = buildHistogram(result, params);

    verifyEqual(testCase, numel(binCenters), numel(binEdges) - 1);
    verifyEqual(testCase, sum(counts), numel(result{1,1}.timestamps), ...
        'Total counts should match number of timestamps');
    verifyGreaterThan(testCase, sum(counts), 0);
end

function testMultiPixelShape(testCase)
    params = defaultSPADParams();
    params.nCycles = 1000;
    params.seed = 2;
    params.trapProb = 0;
    params.useParallel = false;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(3, 4, nBins);
    gateVector = ones(1, nBins);

    result = runSPADSimulation(rateVector, gateVector, params);
    [counts, ~, binCenters] = buildHistogram(result, params);

    verifySize(testCase, counts, [3, 4, numel(binCenters)]);
end

function testPixelIdxSelection(testCase)
    params = defaultSPADParams();
    params.nCycles = 1000;
    params.seed = 3;
    params.trapProb = 0;
    params.useParallel = false;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(2, 2, nBins);
    gateVector = ones(1, nBins);

    result = runSPADSimulation(rateVector, gateVector, params);
    [countsAll, ~, ~] = buildHistogram(result, params);
    [countsSingle, ~, ~] = buildHistogram(result, params, [2, 1]);

    verifyEqual(testCase, countsSingle, squeeze(countsAll(2, 1, :)).');
end

function testStructInput(testCase)
    params = defaultSPADParams();
    params.nCycles = 1000;
    params.seed = 4;
    params.trapProb = 0;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(1, 1, nBins);
    gateVector = ones(1, nBins);

    result = runSPADSimulation(rateVector, gateVector, params);
    pixelStruct = result{1,1};

    [countsCell, ~, ~] = buildHistogram(result, params);
    [countsStruct, ~, ~] = buildHistogram(pixelStruct, params);

    verifyEqual(testCase, countsStruct, squeeze(countsCell(1,1,:)).');
end

function testEmptyTimestamps(testCase)
    params = defaultSPADParams();
    params.nCycles = 100;
    params.seed = 5;
    params.trapProb = 0;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = zeros(1, 1, nBins);
    gateVector = zeros(1, nBins);

    result = runSPADSimulation(rateVector, gateVector, params);
    [counts, ~, ~] = buildHistogram(result, params);

    verifyEqual(testCase, sum(counts), 0);
end

function testBinEdgesSpanCycle(testCase)
    params = defaultSPADParams();
    [~, binEdges, binCenters] = buildHistogram( ...
        struct('timestamps', zeros(0,1)), params);

    verifyEqual(testCase, binEdges(1), 0);
    verifyLessThanOrEqual(testCase, binEdges(end), params.cycleTime);
    verifyTrue(testCase, all(diff(binEdges) > 0), 'Bin edges must be monotonically increasing');
    verifyEqual(testCase, numel(binCenters), numel(binEdges) - 1);
end

%% --- correctPileup tests ---

function testCoatesIdentityAtLowFlux(testCase)
    params = defaultSPADParams();
    params.nCycles = 500000;
    params.seed = 10;
    params.trapProb = 0;
    params.spadDeadTime = 0;
    params.tdcDeadTime = 0;
    params.maxHitsPerCycle = 1;
    params.spadJitterFWHM = 0;
    params.tdcJitterFWHM = 0;
    params.dcr = 0;
    params.pde = 1.0;

    nBins = round(params.cycleTime / params.binWidth);
    gateVector = ones(1, nBins);

    % Very low flux: ~0.05 photons/cycle — negligible pile-up
    rateVector = 5e5 * ones(1, 1, nBins);
    result = runSPADSimulation(rateVector, gateVector, params);
    [counts, ~, ~] = buildHistogram(result, params);

    corrected = correctPileup(counts, params.nCycles);

    % At low flux, correction should be near-identity (cumulative effect
    % grows toward later bins, so use 10% tolerance)
    verifyEqual(testCase, corrected, counts, 'RelTol', 0.1, ...
        'Correction should be minimal at low flux');
end

function testCoatesRecoversTrueShape(testCase)
    params = defaultSPADParams();
    params.nCycles = 500000;
    params.seed = 11;
    params.trapProb = 0;
    params.spadDeadTime = 0;
    params.tdcDeadTime = 0;
    params.maxHitsPerCycle = 1;
    params.spadJitterFWHM = 0;
    params.tdcJitterFWHM = 0;
    params.dcr = 0;
    params.pde = 1.0;

    nBins = round(params.cycleTime / params.binWidth);
    gateVector = ones(1, nBins);
    binCentersTime = ((0:nBins-1) + 0.5) * params.binWidth;

    % Gaussian pulse — high flux to induce pile-up
    pulseCenter = 50e-9;
    pulseSigma = 3e-9;
    pulse = exp(-0.5 * ((binCentersTime - pulseCenter) / pulseSigma).^2);
    peakRate = 5e9;
    rateVector = reshape(peakRate * pulse, 1, 1, []);

    result = runSPADSimulation(rateVector, gateVector, params);
    [counts, ~, ~] = buildHistogram(result, params);

    corrected = correctPileup(counts, params.nCycles);

    % The corrected histogram peak should be closer to true center
    [~, rawPeakBin] = max(counts);
    [~, corrPeakBin] = max(corrected);
    truePeakBin = round(pulseCenter / params.tdcResolution);

    verifyLessThanOrEqual(testCase, abs(corrPeakBin - truePeakBin), ...
        abs(rawPeakBin - truePeakBin), ...
        'Corrected peak should be at least as close to true peak as raw');
end

function testCoatesPreservesTotalCounts(testCase)
    nCycles = 100000;
    counts = [500, 300, 200, 100, 50, 25];
    corrected = correctPileup(counts, nCycles);

    % Corrected values should be >= raw (correction inflates suppressed bins)
    verifyGreaterThanOrEqual(testCase, corrected, counts - 1e-10, ...
        'Corrected values should be >= raw counts');
end

function testCoatesZeroInput(testCase)
    corrected = correctPileup(zeros(1, 100), 10000);
    verifyEqual(testCase, corrected, zeros(1, 100));
end

function testCoates3DInput(testCase)
    params = defaultSPADParams();
    params.nCycles = 10000;
    params.seed = 12;
    params.trapProb = 0;
    params.spadDeadTime = 0;
    params.tdcDeadTime = 0;
    params.maxHitsPerCycle = 1;
    params.useParallel = false;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(2, 2, nBins);
    gateVector = ones(1, nBins);

    result = runSPADSimulation(rateVector, gateVector, params);
    [counts, ~, ~] = buildHistogram(result, params);

    corrected = correctPileup(counts, params.nCycles);
    verifySize(testCase, corrected, size(counts));
end

function testCoatesRejectsAmbiguous2D(testCase)
    verifyError(testCase, @() correctPileup(ones(5, 10), 1000), ...
        'correctPileup:ambiguousInput');
end
