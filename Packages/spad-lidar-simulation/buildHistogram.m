function [counts, binEdges, binCenters] = buildHistogram(result, params, pixelIdx)
%BUILDHISTOGRAM Build TCSPC histogram from simulation results.
%
%   [counts, binEdges, binCenters] = buildHistogram(result, params)
%   [counts, binEdges, binCenters] = buildHistogram(result, params, pixelIdx)
%
%   Inputs:
%     result   - nRows x nCols cell array from digitize/runSPADSimulation,
%                or a single pixel struct
%     params   - struct from defaultSPADParams
%     pixelIdx - (optional) [row, col] index. If omitted and result is a
%                cell array, returns a nRows x nCols x nHistBins array.
%
%   Outputs:
%     counts     - histogram counts (1 x nHistBins for single pixel, or
%                  nRows x nCols x nHistBins for full array)
%     binEdges   - histogram bin edges (s)
%     binCenters - histogram bin centers (s)

    arguments
        result {mustBeNonempty}
        params (1,1) struct
        pixelIdx {mustBePositive, mustBeInteger} = []
    end

    binEdges = 0:params.tdcResolution:params.cycleTime;
    binCenters = (binEdges(1:end-1) + binEdges(2:end)) / 2;
    nHistBins = numel(binCenters);

    if isstruct(result)
        counts = histSinglePixel(result, params.cycleTime, binEdges);
        return;
    end

    if ~isempty(pixelIdx)
        counts = histSinglePixel(result{pixelIdx(1), pixelIdx(2)}, ...
            params.cycleTime, binEdges);
        return;
    end

    [nRows, nCols] = size(result);
    counts = zeros(nRows, nCols, nHistBins);
    for r = 1:nRows
        for c = 1:nCols
            counts(r, c, :) = histSinglePixel(result{r,c}, ...
                params.cycleTime, binEdges);
        end
    end

end

function h = histSinglePixel(pixelResult, cycleTime, binEdges)
    relTimes = mod(pixelResult.timestamps, cycleTime);
    h = histcounts(relTimes, binEdges);
end
