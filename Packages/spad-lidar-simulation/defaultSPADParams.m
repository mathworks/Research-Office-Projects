function params = defaultSPADParams()
%DEFAULTSPADPARAMS Return default parameter struct for dToF SPAD simulator.

    params.cycleTime       = 100e-9;    % Laser repetition period (s)
    params.nCycles         = 1e5;       % Number of laser cycles
    params.binWidth        = 50e-12;    % Width of each rate/gate vector bin (s)

    params.pde             = 0.25;      % Photon detection efficiency
    params.dcr             = 1000;      % Dark count rate (Hz, before gating)

    params.spadDeadTime    = 100e-9;    % SPAD quench + recharge time (s)
    params.tdcDeadTime     = 5e-9;      % TDC conversion time (s)

    params.trapProb        = 0.01;      % Carrier trapping probability per avalanche
    params.afterPulseDecay = 50e-9;     % Trapped-carrier release time constant (s)
    params.maxAfterPulseGens = 10;      % Safety cap on after-pulse chaining
    params.exactAfterPulse = true;      % true = exact chaining, false = vectorized single-gen

    params.spadJitterFWHM  = 100e-12;   % SPAD timing jitter FWHM (s)
    params.tdcJitterFWHM   = 50e-12;    % TDC timing jitter FWHM (s)
    params.tdcResolution   = 50e-12;    % TDC bin width / quantization step (s)
    params.maxHitsPerCycle = inf;       % Max TDC recordings per laser cycle (inf = unlimited)

    params.crosstalkProb   = 0;         % Scalar: uniform probability to each neighbor. Matrix: kernel of per-offset probabilities (center = self, ignored).
    params.crosstalkMap    = [];       % Connectivity matrix (nPixels x nPixels logical), or [] for nearest-neighbor. Ignored when crosstalkProb is a kernel.
    params.exactCrosstalk  = false;    % true = serial exact, false = parallel approximate

    params.seed            = [];        % RNG seed (empty = no explicit seeding)
    params.useParallel     = true;      % Use parfor for multi-pixel processing

end
