function tests = testValidation
%TESTVALIDATION Test that argument validation catches invalid inputs.
    tests = functiontests(localfunctions);
end

function testEmptyRateVector(testCase)
    params = defaultSPADParams();
    gateVector = ones(1, 2000);

    verifyError(testCase, @() generateArrivals([], gateVector, params), ...
        'MATLAB:validators:mustBeNonempty');
end

function testNonNumericGate(testCase)
    params = defaultSPADParams();
    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(1, 1, nBins);

    verifyError(testCase, @() generateArrivals(rateVector, "notanumber", params), ...
        'MATLAB:validators:mustBeNumeric');
end

function testParamsNotStruct(testCase)
    nBins = 2000;
    rateVector = 1e8 * ones(1, 1, nBins);
    gateVector = ones(1, nBins);

    verifyError(testCase, @() generateArrivals(rateVector, gateVector, 42), ...
        'MATLAB:validation:UnableToConvert');
end

function testSimulateSPADEmptyArrivals(testCase)
    params = defaultSPADParams();
    gateVector = ones(1, 2000);

    verifyError(testCase, @() simulateSPAD({}, gateVector, params), ...
        'MATLAB:validators:mustBeNonempty');
end

function testDigitizeEmptyArrivals(testCase)
    params = defaultSPADParams();

    verifyError(testCase, @() digitize({}, params), ...
        'MATLAB:validators:mustBeNonempty');
end

function testValidInputsPasses(testCase)
    params = defaultSPADParams();
    nBins = round(params.cycleTime / params.binWidth);
    rateVector = 1e8 * ones(1, 1, nBins);
    gateVector = ones(1, nBins);

    arrivals = generateArrivals(rateVector, gateVector, params);
    verifyClass(testCase, arrivals, 'cell');
end

function testRunSPADSimulationValidates(testCase)
    params = defaultSPADParams();
    gateVector = ones(1, 2000);

    verifyError(testCase, @() runSPADSimulation([], gateVector, params), ...
        'MATLAB:validators:mustBeNonempty');
end
