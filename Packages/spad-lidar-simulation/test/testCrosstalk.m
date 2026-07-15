function tests = testCrosstalk
%TESTCROSSTALK Test crosstalk simulation modes.
    tests = functiontests(localfunctions);
end

function testExactCrosstalkProducesCrosstalkEvents(testCase)
    params = defaultSPADParams();
    params.nCycles = 5000;
    params.seed = 20;
    params.trapProb = 0;
    params.crosstalkProb = 0.1;
    params.exactCrosstalk = true;
    params.useParallel = false;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(2, 2, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    result = simulateSPAD(arrivals, gateVector, params);

    totalCT = 0;
    for r = 1:2
        for c = 1:2
            totalCT = totalCT + sum(result{r,c}.eventType == "crosstalk");
        end
    end
    verifyGreaterThan(testCase, totalCT, 0, 'Should produce crosstalk events');
end

function testApproxCrosstalkProducesCrosstalkEvents(testCase)
    params = defaultSPADParams();
    params.nCycles = 5000;
    params.seed = 21;
    params.trapProb = 0;
    params.crosstalkProb = 0.1;
    params.exactCrosstalk = false;
    params.useParallel = false;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(2, 2, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    result = simulateSPAD(arrivals, gateVector, params);

    totalCT = 0;
    for r = 1:2
        for c = 1:2
            totalCT = totalCT + sum(result{r,c}.eventType == "crosstalk");
        end
    end
    verifyGreaterThan(testCase, totalCT, 0, 'Should produce crosstalk events');
end

function testNoCrosstalkWhenProbZero(testCase)
    params = defaultSPADParams();
    params.nCycles = 1000;
    params.seed = 22;
    params.trapProb = 0;
    params.crosstalkProb = 0;
    params.useParallel = false;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(2, 2, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    result = simulateSPAD(arrivals, gateVector, params);

    for r = 1:2
        for c = 1:2
            verifyEqual(testCase, sum(result{r,c}.eventType == "crosstalk"), 0);
        end
    end
end

function testDeadTimeEnforcedWithCrosstalk(testCase)
    params = defaultSPADParams();
    params.nCycles = 5000;
    params.seed = 23;
    params.trapProb = 0.05;
    params.crosstalkProb = 0.1;
    params.exactCrosstalk = true;
    params.useParallel = false;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(2, 2, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    result = simulateSPAD(arrivals, gateVector, params);

    for r = 1:2
        for c = 1:2
            if numel(result{r,c}.times) > 1
                minSep = min(diff(result{r,c}.times));
                verifyGreaterThanOrEqual(testCase, minSep, ...
                    params.spadDeadTime - 1e-15, ...
                    sprintf('Dead time violated at pixel (%d,%d)', r, c));
            end
        end
    end
end

function testCustomCrosstalkMap(testCase)
    params = defaultSPADParams();
    params.nCycles = 5000;
    params.seed = 24;
    params.trapProb = 0;
    params.crosstalkProb = 0.2;
    params.exactCrosstalk = true;
    params.useParallel = false;

    % 4 pixels in a row, only connect 1->2 and 3->4
    params.crosstalkMap = [0 1 0 0; 1 0 0 0; 0 0 0 1; 0 0 1 0];

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = zeros(4, 1, nBins);
    rateVector(1, 1, :) = 1e9; % only pixel 1 has signal
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    result = simulateSPAD(arrivals, gateVector, params);

    % Pixel 2 should get crosstalk from pixel 1
    verifyGreaterThan(testCase, sum(result{2,1}.eventType == "crosstalk"), 0, ...
        'Pixel 2 should receive crosstalk from pixel 1');

    % Pixels 3 and 4 should have no crosstalk (not connected to pixel 1)
    verifyEqual(testCase, sum(result{3,1}.eventType == "crosstalk"), 0, ...
        'Pixel 3 should not receive crosstalk');
    verifyEqual(testCase, sum(result{4,1}.eventType == "crosstalk"), 0, ...
        'Pixel 4 should not receive crosstalk');
end

function testCrosstalkWithApproxAfterPulse(testCase)
    params = defaultSPADParams();
    params.nCycles = 2000;
    params.seed = 25;
    params.trapProb = 0.05;
    params.exactAfterPulse = false;
    params.crosstalkProb = 0.05;
    params.exactCrosstalk = false;
    params.useParallel = false;

    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(2, 2, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    result = simulateSPAD(arrivals, gateVector, params);

    % Should have all event types
    allTypes = vertcat(result{:});
    allEventTypes = vertcat(allTypes.eventType);
    verifyTrue(testCase, any(allEventTypes == "signal"));
    verifyTrue(testCase, any(allEventTypes == "afterpulse"));
    verifyTrue(testCase, any(allEventTypes == "crosstalk"));
end
