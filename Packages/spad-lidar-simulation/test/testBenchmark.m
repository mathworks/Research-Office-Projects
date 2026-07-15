function tests = testBenchmark
%TESTBENCHMARK Compare wall-clock time of hybrid vs naive simulateSPAD.
    tests = functiontests(localfunctions);
end

function testHybridMatchesNaive(testCase)
    params = defaultSPADParams();
    params.nCycles = 100000;
    params.seed = 42;
    params.trapProb = 0.03;
    params.pde = 0.3;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 5e8 * ones(1, 1, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    fprintf('Total arrivals: %d\n', numel(arrivals{1,1}.times));

    rng(100);
    tic;
    resultHybrid = simulateSPAD(arrivals, gateVector, params);
    tHybrid = toc;

    rng(100);
    tic;
    resultNaive = simulateSPAD_naive(arrivals, gateVector, params);
    tNaive = toc;

    fprintf('Hybrid:  %.3f s (%d detections)\n', tHybrid, numel(resultHybrid{1,1}.times));
    fprintf('Naive:   %.3f s (%d detections)\n', tNaive, numel(resultNaive{1,1}.times));
    fprintf('Speedup: %.1fx\n', tNaive / tHybrid);

    verifyEqual(testCase, numel(resultHybrid{1,1}.times), numel(resultNaive{1,1}.times), ...
        'Detection count mismatch');
    verifyEqual(testCase, resultHybrid{1,1}.times, resultNaive{1,1}.times, ...
        'AbsTol', 1e-15, 'Detection times mismatch');
end

function testLowDeadTimeSpeedup(testCase)
    params = defaultSPADParams();
    params.nCycles = 50000;
    params.seed = 99;
    params.trapProb = 0.05;
    params.pde = 0.3;
    params.spadDeadTime = 20e-9;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e9 * ones(1, 1, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);

    rng(200);
    tic;
    resultHybrid = simulateSPAD(arrivals, gateVector, params);
    tHybrid = toc;

    rng(200);
    tic;
    resultNaive = simulateSPAD_naive(arrivals, gateVector, params);
    tNaive = toc;

    fprintf('Hybrid: %.3f s, Naive: %.3f s, Speedup: %.1fx\n', tHybrid, tNaive, tNaive/tHybrid);
    verifyEqual(testCase, resultHybrid{1,1}.times, resultNaive{1,1}.times, ...
        'AbsTol', 1e-15, 'Results must match');
end
