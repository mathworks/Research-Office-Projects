function corrected = correctPileup(counts, nCycles)
%CORRECTPILEUP Coates pile-up correction for single-stop TCSPC histograms.
%
%   corrected = correctPileup(counts, nCycles)
%
%   Applies the Coates (1968) correction to a histogram acquired with a
%   single-stop TDC. The algorithm estimates the true detection probability
%   per bin by accounting for the cumulative probability that a photon was
%   already recorded in an earlier bin during the same cycle.
%
%   Inputs:
%     counts  - 1 x nBins raw histogram counts (or nRows x nCols x nBins)
%     nCycles - number of laser cycles used in the acquisition
%
%   Outputs:
%     corrected - corrected histogram, same size as counts. Values represent
%                 estimated true detection probabilities scaled by nCycles.
%
%   Reference:
%     P.B. Coates, "The correction for photon 'pile-up' in the measurement
%     of radiative lifetimes," J. Phys. E: Sci. Instrum., vol. 1, 1968.

    if ismatrix(counts) && size(counts, 1) > 1 && ndims(counts) <= 2
        error("correctPileup:ambiguousInput", ...
            "For multi-pixel arrays, counts must be nRows x nCols x nBins (3-D).");
    end

    if ndims(counts) == 3
        corrected = zeros(size(counts));
        for r = 1:size(counts, 1)
            for c = 1:size(counts, 2)
                corrected(r, c, :) = correctSingle(squeeze(counts(r, c, :)).', nCycles);
            end
        end
    else
        corrected = correctSingle(counts, nCycles);
    end
end

function corrected = correctSingle(h, N)
    cumPrior = [0, cumsum(h(1:end-1))];
    remaining = N - cumPrior;

    % Bins where all cycles already fired have no information — leave at zero
    valid = remaining > 0;
    corrected = zeros(size(h));
    corrected(valid) = N * (h(valid) ./ remaining(valid));
end
