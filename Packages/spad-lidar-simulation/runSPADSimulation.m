function result = runSPADSimulation(rateVector, gateVector, params)
%RUNSPADSIMULATION Run full dToF SPAD simulation pipeline.
%
%   result = runSPADSimulation(rateVector, gateVector, params)
%
%   Convenience wrapper that chains generateArrivals -> simulateSPAD -> digitize.
%
%   Inputs:
%     rateVector - One of:
%       (a) nRows x nCols x nBins photon arrival rates (photons/s)
%       (b) struct array with .rate and .label fields for multiple sources
%     gateVector - 1 x nBins gate modulation (0 to 1), or [] for free-running
%     params     - struct from defaultSPADParams
%
%   Output:
%     result - nRows x nCols cell array. Each cell is a struct with:
%       .timestamps - final quantized detection times (s)
%       .rawTimes   - pre-jitter avalanche times (s)
%       .eventType  - string array (source labels, 'dark', 'afterpulse', 'crosstalk')
%       .cycleIndex - which laser cycle each event belongs to

    arguments
        rateVector {mustBeNonempty}
        gateVector {mustBeNumeric}
        params (1,1) struct
    end

    arrivals = generateArrivals(rateVector, gateVector, params);
    avalanches = simulateSPAD(arrivals, gateVector, params);
    result = digitize(avalanches, params);

end
