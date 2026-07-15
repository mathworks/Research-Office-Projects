%% dToF SPAD Simulator — Basic Example
% Simulate a single-pixel SPAD detecting a pulsed laser return.

%% Setup parameters
params = defaultSPADParams();
params.nCycles = 50000;
params.seed = 42;

%% Define rate vector (signal pulse)
nBins = round(params.cycleTime / params.binWidth);

% Gaussian pulse at 5ns (representing a target at ~0.75m)
pulseCenter = 5e-9;
pulseSigma = 200e-12;
binCenters = ((0:nBins-1) + 0.5) * params.binWidth;
rateVector = 1e8 * exp(-0.5 * ((binCenters - pulseCenter) / pulseSigma).^2);

% Add uniform background (ambient light)
backgroundRate = 1e6;
rateVector = rateVector + backgroundRate;

% Reshape to 1 x 1 x nBins for single pixel
rateVector = reshape(rateVector, 1, 1, []);

%% Gate vector (open from 0 to 60ns)
gateVector = zeros(1, nBins);
gateOnBin = 1;
gateOffBin = round(60e-9 / params.binWidth);
gateVector(gateOnBin:gateOffBin) = 1;

%% Run simulation pipeline
arrivals = generateArrivals(rateVector, gateVector, params);
fprintf('Generated %d candidate arrivals\n', numel(arrivals{1,1}.times));

avalanches = simulateSPAD(arrivals, gateVector, params);
fprintf('SPAD produced %d avalanche events\n', numel(avalanches{1,1}.times));

result = digitize(avalanches, params);
fprintf('TDC recorded %d timestamps\n', numel(result{1,1}.timestamps));

%% Build TCSPC histogram
relTimes = mod(result{1,1}.timestamps, params.cycleTime);
histEdges = 0:params.tdcResolution:params.cycleTime;
counts = histcounts(relTimes, histEdges);

%% Plot results
figure;

subplot(2,1,1);
bar(binCenters * 1e9, squeeze(rateVector) * params.pde, 1, 'FaceColor', [0.7 0.7 0.7]);
xlabel('Time (ns)');
ylabel('Detected photon rate (photons/s)');
title('Input: Rate Vector \times PDE');
xlim([0 params.cycleTime*1e9]);

subplot(2,1,2);
histCenters = (histEdges(1:end-1) + histEdges(2:end)) / 2;
bar(histCenters * 1e9, counts, 1, 'FaceColor', [0.2 0.4 0.8]);
xlabel('Time (ns)');
ylabel('Counts');
title(sprintf('Output: TCSPC Histogram (%d cycles)', params.nCycles));
xlim([0 params.cycleTime*1e9]);

%% Event type breakdown
types = result{1,1}.eventType;
nSignal = sum(types == "signal");
nDark = sum(types == "dark");
nAP = sum(types == "afterpulse");
nTotal = numel(types);
fprintf('\nEvent breakdown:\n');
fprintf('  Signal:      %d (%.1f%%)\n', nSignal, 100*nSignal/nTotal);
fprintf('  Dark:        %d (%.1f%%)\n', nDark, 100*nDark/nTotal);
fprintf('  After-pulse: %d (%.1f%%)\n', nAP, 100*nAP/nTotal);
