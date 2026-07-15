function tests = testPhysics
%TESTPHYSICS Physical sanity checks for the SPAD simulator.
    tests = functiontests(localfunctions);
end

function testNoDetectionsWhenGateZero(testCase)
    params = defaultSPADParams();
    params.nCycles = 1000;
    params.seed = 1;
    params.trapProb = 0;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e9 * ones(1, 1, nBins);
    gateVector = zeros(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    verifyEmpty(testCase, arrivals{1,1}.times, ...
        'Should have no arrivals when gate is fully closed');
end

function testDeadTimeGreaterThanCycle(testCase)
    params = defaultSPADParams();
    params.nCycles = 10000;
    params.seed = 2;
    params.spadDeadTime = 250e-9;
    params.cycleTime = 100e-9;
    params.trapProb = 0;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e9 * ones(1, 1, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    result = simulateSPAD(arrivals, gateVector, params);

    maxExpected = ceil(params.nCycles / (params.spadDeadTime / params.cycleTime));
    verifyLessThanOrEqual(testCase, numel(result{1,1}.times), maxExpected);

    if numel(result{1,1}.times) > 1
        separations = diff(result{1,1}.times);
        verifyGreaterThanOrEqual(testCase, min(separations), params.spadDeadTime - 1e-15);
    end
end

function testPileUpDistortion(testCase)
    params = defaultSPADParams();
    params.nCycles = 100000;
    params.seed = 3;
    params.spadDeadTime = 100e-9;
    params.trapProb = 0;
    params.spadJitterFWHM = 0;
    params.tdcJitterFWHM = 0;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = zeros(1, 1, nBins);
    midBin = round(nBins / 2);
    rateVector(1, 1, midBin:midBin+3) = 5e8;
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    avalanches = simulateSPAD(arrivals, gateVector, params);
    result = digitize(avalanches, params);

    relTimes = mod(result{1,1}.timestamps, params.cycleTime);
    expectedCenter = midBin * params.binWidth;
    medianDetection = median(relTimes);
    verifyEqual(testCase, medianDetection, expectedCenter, 'AbsTol', 1e-9, ...
        'Detections should cluster near the pulse');
end

function testAfterPulseGeneration(testCase)
    params = defaultSPADParams();
    params.nCycles = 50000;
    params.seed = 4;
    params.trapProb = 0.1;
    params.afterPulseDecay = 30e-9;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(1, 1, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    result = simulateSPAD(arrivals, gateVector, params);

    apMask = result{1,1}.eventType == "afterpulse";
    verifyGreaterThan(testCase, sum(apMask), 0, ...
        'Should generate some after-pulses');

    totalDetections = numel(result{1,1}.times);
    apFraction = sum(apMask) / totalDetections;
    verifyLessThan(testCase, apFraction, 0.2, ...
        'After-pulse fraction should be reasonable');
end

function testTDCDeadTime(testCase)
    params = defaultSPADParams();
    params.nCycles = 10000;
    params.seed = 5;
    params.spadDeadTime = 1e-9;
    params.tdcDeadTime = 50e-9;
    params.trapProb = 0;
    params.spadJitterFWHM = 0;
    params.tdcJitterFWHM = 0;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e9 * ones(1, 1, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    avalanches = simulateSPAD(arrivals, gateVector, params);
    result = digitize(avalanches, params);

    if numel(result{1,1}.timestamps) > 1
        separations = diff(result{1,1}.timestamps);
        verifyGreaterThanOrEqual(testCase, min(separations), ...
            params.tdcDeadTime - params.tdcResolution, ...
            'TDC dead time not enforced');
    end
end

function testMultiPixelIndependence(testCase)
    params = defaultSPADParams();
    params.nCycles = 1000;
    params.seed = 6;
    params.trapProb = 0.02;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = zeros(2, 1, nBins);
    rateVector(1, 1, :) = 1e8;
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    avalanches = simulateSPAD(arrivals, gateVector, params);

    verifyGreaterThan(testCase, numel(avalanches{1,1}.times), 0, ...
        'Pixel (1,1) should have detections');
end

function test2DArrayShape(testCase)
    params = defaultSPADParams();
    params.nCycles = 100;
    params.seed = 8;
    params.trapProb = 0;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(3, 4, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    verifySize(testCase, arrivals, [3, 4]);

    avalanches = simulateSPAD(arrivals, gateVector, params);
    verifySize(testCase, avalanches, [3, 4]);

    result = digitize(avalanches, params);
    verifySize(testCase, result, [3, 4]);
end
