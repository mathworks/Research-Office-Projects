function tests = testEquivalence
%TESTEQUIVALENCE Verify hybrid simulateSPAD matches naive implementation.
    tests = functiontests(localfunctions);
end

function testBasicEquivalence(testCase)
    params = defaultSPADParams();
    params.nCycles = 1000;
    params.seed = 42;
    params.trapProb = 0.05;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = zeros(1, 1, nBins);
    pulseBin = round(10e-9 / params.binWidth);
    rateVector(1, 1, pulseBin:pulseBin+5) = 1e8;
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);

    rng(100);
    resultHybrid = simulateSPAD(arrivals, gateVector, params);

    rng(100);
    resultNaive = simulateSPAD_naive(arrivals, gateVector, params);

    verifyPixelEquivalence(testCase, resultHybrid{1,1}, resultNaive{1,1});
end

function testNoAfterPulseEquivalence(testCase)
    params = defaultSPADParams();
    params.nCycles = 5000;
    params.seed = 7;
    params.trapProb = 0;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 5e7 * ones(1, 1, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);

    rng(200);
    resultHybrid = simulateSPAD(arrivals, gateVector, params);

    rng(200);
    resultNaive = simulateSPAD_naive(arrivals, gateVector, params);

    verifyPixelEquivalence(testCase, resultHybrid{1,1}, resultNaive{1,1});
end

function testHighAfterPulseEquivalence(testCase)
    params = defaultSPADParams();
    params.nCycles = 500;
    params.seed = 99;
    params.trapProb = 0.2;
    params.afterPulseDecay = 20e-9;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 2e8 * ones(1, 1, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);

    rng(300);
    resultHybrid = simulateSPAD(arrivals, gateVector, params);

    rng(300);
    resultNaive = simulateSPAD_naive(arrivals, gateVector, params);

    verifyPixelEquivalence(testCase, resultHybrid{1,1}, resultNaive{1,1});
end

function testGatedEquivalence(testCase)
    params = defaultSPADParams();
    params.nCycles = 1000;
    params.seed = 55;
    params.trapProb = 0.03;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(1, 1, nBins);
    gateVector = zeros(1, nBins);
    gateOn = round(5e-9 / params.binWidth);
    gateOff = round(50e-9 / params.binWidth);
    gateVector(gateOn:gateOff) = 1;

    arrivals = generateArrivals(rateVector, gateVector, params);

    rng(400);
    resultHybrid = simulateSPAD(arrivals, gateVector, params);

    rng(400);
    resultNaive = simulateSPAD_naive(arrivals, gateVector, params);

    verifyPixelEquivalence(testCase, resultHybrid{1,1}, resultNaive{1,1});
end

function testMultiPixelEquivalence(testCase)
    params = defaultSPADParams();
    params.nCycles = 500;
    params.seed = 77;
    params.trapProb = 0.05;
    params.useParallel = false;

    nBins = round(params.cycleTime / params.binWidth);
    rng(77);
    rateVector = 1e8 * rand(2, 2, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);

    rng(500);
    resultHybrid = simulateSPAD(arrivals, gateVector, params);

    rng(500);
    resultNaive = simulateSPAD_naive(arrivals, gateVector, params);

    for r = 1:2
        for c = 1:2
            verifyEqual(testCase, resultHybrid{r,c}.times, resultNaive{r,c}.times, ...
                'AbsTol', 1e-15, sprintf('Pixel (%d,%d) times mismatch', r, c));
            verifyEqual(testCase, resultHybrid{r,c}.eventType, resultNaive{r,c}.eventType, ...
                sprintf('Pixel (%d,%d) event types mismatch', r, c));
        end
    end
end

function verifyPixelEquivalence(testCase, hybrid, naive)
    verifyEqual(testCase, numel(hybrid.times), numel(naive.times), ...
        'Detection count mismatch');
    if numel(hybrid.times) == numel(naive.times)
        verifyEqual(testCase, hybrid.times, naive.times, ...
            'AbsTol', 1e-15, 'Detection times mismatch');
        verifyEqual(testCase, hybrid.eventType, naive.eventType, ...
            'Event types mismatch');
    end
end
