function s = obsBusFromArrays(t, P1, P2, L1, L2, LOS, Valid)
%obsBusFromArrays - pack observable arrays into the playback form of GNSSObservableBus.
%
% Returns the structure that RTKBase/ObservableSource's Measured choice reads through its From
% Workspace block, i.e. what belongs in rtk_measured_obs. One field per bus element, each a
% timeseries, because that is the only From Workspace format that emits a bus.
%
% This exists so that a source of measurements -- a RINEX reader, a logged flight, a receiver
% replay -- has one place to hand its arrays to and does not have to know how From Workspace
% wants a bus laid out. No such reader ships here: the interface is deliberately proven
% source-agnostic with synthesised observables -- the playback path through this function was
% measured bit-identical to the injected one -- because that is a property of the interface and
% does not need real data to establish. Writing the reader is then a self-contained job with a
% known target: fill these six arrays in these shapes. Get the layout wrong and the failure is
% quiet: a
% [nSat x 2 x nEpoch] array is accepted just as happily as [2 x nSat x nEpoch] and puts the
% base row where the rover row belongs, which negates the single difference and leaves the base
% clock in it. The cascade will still resolve integers around that, to a mirrored baseline,
% reporting a confident centimetre sigma the whole way. So every shape is checked here.
%
% Inputs
%   t     - nEpoch-by-1 epoch times, seconds, ascending. Must land on the engine's own
%           correction interval; From Workspace holds between samples with interpolation off,
%           so times off the grid repeat an epoch rather than erroring.
%   P1,P2 - 2-by-nSat-by-nEpoch code pseudoranges, metres. ROW 1 IS THE BASE.
%   L1,L2 - 2-by-nSat-by-nEpoch carrier phase, metres -- cycles TIMES the wavelength, not
%           cycles. A RINEX record is in cycles; forgetting the scaling puts the phase five
%           times too small and the wide lane never closes.
%   LOS   - nSat-by-3 unit line of sight, base to satellite, local ENU. Either one matrix, held
%           for the whole run, or nSat-by-3-by-nEpoch to move it per epoch.
%   Valid - nEpoch-by-nSat, nonzero where that slot was observed. Rows, one per epoch, because
%           that is the natural shape of a visibility table.
%
% See gnssObservableBus for what the elements mean.

nEp = numel(t);
t   = t(:);

if nEp < 1
    error('obsBusFromArrays:empty', 'need at least one epoch');
end
if any(diff(t) <= 0)
    error('obsBusFromArrays:time', 'epoch times must be strictly ascending');
end

nSat = size(P1, 2);
obs  = {'P1', P1; 'P2', P2; 'L1', L1; 'L2', L2};
for k = 1:size(obs, 1)
    if ~isequal(size(obs{k, 2}), [2, nSat, nEp])
        error('obsBusFromArrays:obsShape', ...
            '%s must be 2-by-%d-by-%d (base row first), got %s', ...
            obs{k, 1}, nSat, nEp, mat2str(size(obs{k, 2})));
    end
end

% A single LOS matrix is expanded here rather than left as a 2-D timeseries, so that every
% element of the bus has the same epoch count and a per-epoch constellation costs the caller
% nothing to switch to.
if isequal(size(LOS), [nSat, 3])
    LOS = repmat(LOS, 1, 1, nEp);
elseif ~isequal(size(LOS), [nSat, 3, nEp])
    error('obsBusFromArrays:losShape', ...
        'LOS must be %d-by-3 or %d-by-3-by-%d, got %s', ...
        nSat, nSat, nEp, mat2str(size(LOS)));
end

if ~isequal(size(Valid), [nEp, nSat])
    error('obsBusFromArrays:validShape', ...
        'Valid must be %d-by-%d (one row per epoch), got %s', ...
        nEp, nSat, mat2str(size(Valid)));
end

% Reshaped to nSat-by-1-by-nEpoch to match the bus element, and cast to double: the element is
% declared double, and a logical here fails the bus check rather than being promoted.
V = reshape(double(Valid)', nSat, 1, nEp);

s = struct( ...
    'P1',    ts(P1,  t, [2 nSat]), ...
    'P2',    ts(P2,  t, [2 nSat]), ...
    'L1',    ts(L1,  t, [2 nSat]), ...
    'L2',    ts(L2,  t, [2 nSat]), ...
    'LOS',   ts(LOS, t, [nSat 3]), ...
    'Valid', ts(V,   t, [nSat 1]));
end

function y = ts(data, t, sampleSize)
% Build one timeseries and CHECK which dimension it decided was time.
%
% timeseries infers the time dimension rather than being told: it sets IsTimeFirst when the
% first dimension happens to match the time vector's length, and otherwise takes the last. For
% 3-D data it lands on time-last in every case measured here, including the two genuinely
% ambiguous ones -- a 2-epoch run of a 2-row observable, and an nSat-epoch run of an
% nSat-by-3 LOS. But that is inferred behaviour, not a documented contract, and if it ever
% changed the symptom would be an engine that reads two epochs of a sixty-epoch recording and
% holds the second one forever. So the layout is asserted here instead of assumed.
y = timeseries(data, t);
if ~isequal(y.getdatasamplesize, sampleSize) || y.Length ~= numel(t)
    error('obsBusFromArrays:timeLayout', ...
        ['timeseries read this as %d samples of %s; expected %d samples of %s. ' ...
        'It put time on the wrong dimension.'], ...
        y.Length, mat2str(y.getdatasamplesize), numel(t), mat2str(sampleSize));
end
end
