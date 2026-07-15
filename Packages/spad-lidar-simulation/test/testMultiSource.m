function tests = testMultiSource
%TESTMULTISOURCE Test multi-source rate vector functionality.
    tests = functiontests(localfunctions);
end

function testStructArrayInput(testCase)
    params = defaultSPADParams();
    params.nCycles = 1000;
    params.seed = 10;
    params.trapProb = 0;

    nBins = round(params.cycleTime / params.binWidth);
    gateVector = ones(1, nBins);

    sources(1).rate = 1e8 * ones(1, 1, nBins);
    sources(1).label = "laser";
    sources(2).rate = 1e7 * ones(1, 1, nBins);
    sources(2).label = "background";

    arrivals = generateArrivals(sources, gateVector, params);
    types = arrivals{1,1}.eventType;

    verifyTrue(testCase, any(types == "laser"), 'Should have laser arrivals');
    verifyTrue(testCase, any(types == "background"), 'Should have background arrivals');
    verifyTrue(testCase, ~any(types == "signal"), 'Should not have generic signal label');
end

function testSourceLabelsPreservedThroughPipeline(testCase)
    params = defaultSPADParams();
    params.nCycles = 5000;
    params.seed = 11;
    params.trapProb = 0.05;

    nBins = round(params.cycleTime / params.binWidth);
    gateVector = ones(1, nBins);

    sources(1).rate = 1e8 * ones(1, 1, nBins);
    sources(1).label = "laser";
    sources(2).rate = 5e7 * ones(1, 1, nBins);
    sources(2).label = "ambient";

    result = runSPADSimulation(sources, gateVector, params);
    types = result{1,1}.eventType;

    verifyTrue(testCase, any(types == "laser"), 'Laser label should propagate');
    verifyTrue(testCase, any(types == "ambient"), 'Ambient label should propagate');
    verifyTrue(testCase, any(types == "afterpulse"), 'Should have after-pulses');
end

function testBackwardsCompatibility(testCase)
    params = defaultSPADParams();
    params.nCycles = 1000;
    params.seed = 12;
    params.trapProb = 0;
    params.dcr = 0;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(1, 1, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    verifyTrue(testCase, all(arrivals{1,1}.eventType == "signal"), ...
        'Plain array should label as signal');
end

function testMultiSourceMultiPixel(testCase)
    params = defaultSPADParams();
    params.nCycles = 500;
    params.seed = 13;
    params.trapProb = 0;
    params.useParallel = false;

    nBins = round(params.cycleTime / params.binWidth);
    gateVector = ones(1, nBins);

    sources(1).rate = 1e8 * ones(3, 2, nBins);
    sources(1).label = "laser";
    sources(2).rate = 1e7 * ones(3, 2, nBins);
    sources(2).label = "scatter";

    arrivals = generateArrivals(sources, gateVector, params);
    verifySize(testCase, arrivals, [3, 2]);

    result = simulateSPAD(arrivals, gateVector, params);
    verifySize(testCase, result, [3, 2]);

    for r = 1:3
        for c = 1:2
            verifyTrue(testCase, any(result{r,c}.eventType == "laser"));
        end
    end
end

function testThreeSourcesWithCrosstalk(testCase)
    params = defaultSPADParams();
    params.nCycles = 500;
    params.seed = 14;
    params.trapProb = 0.02;
    params.crosstalkProb = 0.03;
    params.exactCrosstalk = true;
    params.useParallel = false;

    nBins = round(params.cycleTime / params.binWidth);
    gateVector = ones(1, nBins);

    sources(1).rate = 1e8 * ones(2, 2, nBins);
    sources(1).label = "laser";
    sources(2).rate = 1e7 * ones(2, 2, nBins);
    sources(2).label = "ambient";
    sources(3).rate = 5e6 * ones(2, 2, nBins);
    sources(3).label = "fluorescence";

    arrivals = generateArrivals(sources, gateVector, params);
    result = simulateSPAD(arrivals, gateVector, params);

    types = result{1,1}.eventType;
    verifyTrue(testCase, any(types == "laser"));
    verifyTrue(testCase, any(types == "crosstalk"));
end
