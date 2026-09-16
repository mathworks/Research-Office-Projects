function bus = gnssObservableBus(nSat)
%gnssObservableBus - the RTK engine's input contract, as a Simulink.Bus object.
%
% RETURNS the bus rather than assigning it, so the definition has exactly one home and every
% caller sizes it for itself. createFleetBusObjects puts the result in the base workspace for
% the model; a caller that wants it somewhere else -- a standalone harness model, say -- can put
% it in that model's OWN workspace instead, and scatter nothing into base.
%
% This is the interface that lets simulated and real measurements drive the same cascade.
% RTKBase/ObservableSource is a variant subsystem over it -- Simulated generates observables
% from the flown rover position, Measured plays back a recording -- and RTKBase/RTKEngine has
% one bus inport typed with this object, so a source that fills the wrong shape or misnames an
% element fails at update-diagram instead of quietly feeding the cascade a transposed matrix.
%
% TWO ROWS, ONE PER RECEIVER. P1/P2/L1/L2 are 2-by-nSat: row 1 is the BASE, row 2 is the
% ROVER. Undifferenced, because a real receiver reports only what it measured -- it has never
% heard of its peer, so the pairing is the ENGINE's job and the engine forms row 2 MINUS row 1.
% That sign is part of the contract: swap the rows and the engine still resolves integers
% perfectly, to a mirrored baseline, with nothing anywhere reporting a problem.
%
% UNITS ARE METRES THROUGHOUT. P1/P2 are code pseudoranges as measured. L1/L2 are carrier
% phase already SCALED BY WAVELENGTH -- a RINEX L1C record is in CYCLES, so a real source must
% multiply by lambda before filling this bus. Both carry the receiver clock offset (~900 m) and
% the satellite-side terms; the engine's differencing is what removes them.
%
% Raw phase lands at ~2.35e7 m, which is safe here and unusable on the radio: eps(single) at
% that magnitude is 2.000 m, and the engine was measured never reaching a fix at all from
% single-precision raw rows. That is why rtcm_payload_bytes carries phase-range RESIDUALS,
% as a real MSM message does, rather than the values on this bus.
%
% COLUMNS ARE SATELLITE SLOTS, NOT SATELLITES. Column s means the same satellite on every
% epoch, and Valid(s) says whether it was observed this one. Whoever fills the bus owns the
% PRN-to-slot mapping; the engine keeps per-slot ambiguity state and drops a slot's state when
% Valid goes false, so a rise/set cycle re-resolves rather than reusing a stale integer.
%
% LOS RATHER THAN SATELLITE POSITIONS. The engine needs the geometry as unit line-of-sight
% vectors, base to satellite, in local ENU -- so that is what the bus carries. A source that
% starts from ephemeris (RINEX nav, or rtk_sat_pos here) computes them once per epoch from the
% base position, which it knows and the engine does not. Carrying positions instead would push
% that conversion into the engine and give it a second job.
%
% Deliberately absent: a valid COUNT (sum(Valid) is derivable, and a redundant copy can
% disagree with the mask it summarises) and an epoch TIMESTAMP (nothing downstream reads it --
% the engine is clocked at rtcm_interval, and a source's own time checks belong in the source).

arguments
    nSat (1,1) double {mustBeInteger, mustBePositive}
end

spec = {
    'P1',    [2 nSat], 'm', 'L1 code pseudorange, base row then rover row, as measured'
    'P2',    [2 nSat], 'm', 'L2 code pseudorange, base row then rover row, as measured'
    'L1',    [2 nSat], 'm', 'L1 carrier phase times lambda1, base row then rover row'
    'L2',    [2 nSat], 'm', 'L2 carrier phase times lambda2, base row then rover row'
    'LOS',   [nSat 3], '1', 'Unit line of sight, base to satellite, local ENU, one row per slot'
    'Valid', [nSat 1], '1', 'Nonzero where the slot was observed this epoch'
    };

for k = size(spec, 1):-1:1
    elems(k) = Simulink.BusElement;           %#ok<AGROW>
    elems(k).Name        = spec{k, 1};
    elems(k).Dimensions  = spec{k, 2};
    elems(k).DataType    = 'double';
    elems(k).Unit        = spec{k, 3};
    elems(k).Description = spec{k, 4};
end

bus = Simulink.Bus;
bus.Elements = elems;
bus.Description = ...
    ['One epoch of raw GNSS observables from a base/rover pair. Undifferenced, ' ...
     'metres, row 1 base and row 2 rover; the engine forms rover minus base.'];
end
