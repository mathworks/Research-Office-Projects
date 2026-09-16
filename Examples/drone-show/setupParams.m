% setupParams.m - Parameter definitions for Multi-UAV Drone Light Show
% This script initializes all parameters needed by MultiUAV_DroneShow.slx

%% Fleet Configuration
if ~exist('N_uav', 'var')
    N_uav = 10;  % Number of UAVs (the app allows 4 to 500; nothing here bounds it)
end
Ts_sim = 0.01;  % Simulation sample time (s)

%% Initial Conditions
% Initial positions are set after formation computation (see below)
% to ensure alignment between guidance model ICs and first formation target.

%% Flight Constraints
if ~exist('v_max', 'var'), v_max = 5.0; end    % Maximum velocity (m/s)
if ~exist('a_max', 'var'), a_max = 3.0; end    % Maximum acceleration (m/s^2)
if ~exist('d_min', 'var'), d_min = 2.0; end    % Minimum inter-UAV separation (m)

%% UAV Physical Parameters
uav_mass = 0.1;  % kg (matches Multi-Instance Guidance Model default)

%% Position Controller Gains
% PositionController computes
%     AccelCmd = Kp_pos*(TargetPos - EstPos) + Kff_pos*TargetVel - Kd_pos*EstVel
% so the feedforward and the damping act on the SAME quantity with opposite signs, and
% Kff_pos = Kd_pos is the value that makes them cancel when the drone is tracking. Any other
% value leaves a standing lag: at a constant commanded speed v the loop settles where
% AccelCmd is zero, which is e_ss = (Kd_pos - Kff_pos)/Kp_pos * v.
%
% Kff_pos was 1.0 against Kd_pos = 2.0, so e_ss was v exactly -- a metre of lag per m/s, on
% every segment of the show, and a drone tracking its reference perfectly was commanded to
% decelerate at Kd_pos*v. Measured lag/v was 0.63 on a plain show (below 1.0 only because
% a min-jerk profile never holds a constant speed long enough to settle) and, more to the
% point, Kp_pos*e reaching 3.51 m/s^2 against the +/-3.0 clamp on
% SatX/SatY -- so the lateral acceleration command was saturating with rotation switched OFF.
% Rotation is what made it visible, by adding a sustained centripetal demand on top.
%
% Setting Kff_pos = Kd_pos removes that lag but exposes the other half of the problem:
% there is no ACCELERATION feedforward, so once the velocity terms cancel the only way
% left to produce acceleration is a position error, e = a_cmd/Kp_pos. The landing is
% min-jerk, whose peak deceleration is 7.513*d/T^2 = 1.69 m/s^2 over a 10 m descent in
% 6.67 s, so at Kp_pos = 1 the fleet sinks 1.7 m BELOW the reference while braking -- i.e.
% through the ground. (At Kff_pos = 1.0 the standing lag held it that far ABOVE the
% descent and cancelled this by accident, which is why the landing checks used to pass.)
%
% The gains are therefore per-axis, because the two axes are limited by different things:
%
%   lateral (x,y): capped by TILT AUTHORITY. SatX/SatY clamp commanded lateral
%     acceleration at +/-3 m/s^2, and that clamp is the model's a_max safety property, so
%     these gains must stay soft enough that tracking transients do not ride the limiter.
%   vertical (z): capped by THRUST, which this model does not limit at all -- there is no
%     Saturate on the vertical channel. So z can carry a much stiffer loop, which is also
%     true of real multirotors: lateral acceleration needs the whole airframe to tilt,
%     vertical needs only a thrust change.
%
% Kp_z is sized from the requirement, not from a measured run: touchdown is allowed 0.3 m,
% so budget half of it, 1.69/Kp_z <= 0.15 => Kp_z >= 11.3.
% Kd_z then follows from critical damping, zeta = Kd/(2*sqrt(Kp)) = 7/(2*sqrt(12)) = 1.01.
% Predicted: 0.14 m at touchdown and 0.16 m of overshoot on the abort path's velocity step
% (v/(w*e) at w = sqrt(Kp_z)), against tolerances of 0.3 m and 0.4 m.
%
% Stiffness is safe here because the plant's own loops are far faster: the Guidance Model
% runs thrust as a P loop at 200 (tau = 5 ms) and attitude as PD at wn = 58.3 rad/s, both
% well above this loop's wn = sqrt(12) = 3.46 rad/s.
Kp_xy = 1.0;     % Lateral position gain -- soft, to stay clear of the SatX/SatY clamp
Kd_xy = 2.0;     % Lateral damping (zeta = 1 against Kp_xy)
Kp_z  = 12.0;    % Vertical position gain -- stiff; see the 0.15 m budget above
Kd_z  = 7.0;     % Vertical damping (zeta = 1.01 against Kp_z)
% Element-wise gains against the [N_uav x 3] ports, so one row per drone and NED xyz
% across the columns. A 1x3 row does NOT broadcast -- Element-wise(K.*u) needs K the same
% size as u -- hence the repmat, matching how the Guidance Model mask sizes its own
% per-drone parameters. setupParams is the only place these are built, so they stay in
% step with N_uav even when the app resizes the fleet.
Kp_pos  = repmat([Kp_xy Kp_xy Kp_z], N_uav, 1);   % Position proportional gain
Kd_pos  = repmat([Kd_xy Kd_xy Kd_z], N_uav, 1);   % Velocity damping gain
Kff_pos = Kd_pos;  % Feedforward gain -- must EQUAL Kd_pos elementwise, never be tuned apart

%% Navigation Sensor Parameters
% GPS horizontal position accuracy by receiver solution mode, in metres. These are
% the three tiers a real RTK receiver walks through as its correction stream ages,
% and they ARE consumed by the model: rtk_sigma_table below turns them into the
% extra position error added to the INS output. Modes: 1=Standard, 2=Float, 3=Fix.
gps_accuracy_standard = 1.5;   % Standard GNSS -- no usable correction
gps_accuracy_rtk_float = 0.3;  % RTK Float -- correction stale, ambiguities lost
gps_accuracy_rtk_fix = 0.02;   % RTK Fix -- fresh correction; the INS dialog value

% nav_mode is GONE, deliberately. The receiver's mode is an OUTPUT of the simulation,
% derived per drone from the age of its last usable correction, not something you dial
% in -- and it never was: a scan of every dialog parameter of every block in the model
% found ZERO references to it. It survived only because the app's RTK Mode dropdown
% wrote it; that dropdown is now the RTK tier READOUT, so nothing writes it and nothing
% reads it. Older diag/prof scripts still assignin it, harmlessly.
% To degrade specific drones use rtk_deny_mask in the RTK section below.

% GPS velocity accuracy
gps_vel_accuracy = 0.1;  % m/s

%% RTK Engine -- carrier-phase double differences
% This section is the RTK CALCULATION. Everything below it in the RTK Correction Link
% section is about the correction LINK -- how a correction gets to a drone and what
% happens when it is late. The two used to be one thing, and that was the defect:
% RTKBase computed
%
%     correction_ned = -lla2ned(measuredLLA, BasePositionLLA, 'flat')
%
% which is position-domain DGPS. It differences two POSITIONS. RTK differences
% OBSERVABLES -- carrier phase, between two receivers and between two satellites --
% and then resolves the integer number of wavelengths in each of those differences.
% Nothing in the old code had an observable, a wavelength or an integer in it, so the
% 0.02 m it claimed was a table entry rather than a result. The tiers below are now
% earned: the numbers in the comments are measured from the cascade this section
% parameterises, not copied from a specification.
%
% DUAL FREQUENCY, and not as a flourish. Single-frequency L1 RTK is honest but cannot
% fix inside a 124 s show: the float ambiguity is buried in code noise and separates
% only as the satellite geometry rotates, which takes minutes. The wide-lane
% combination of L1 and L2 has a 0.862 m wavelength instead of 0.190 m, so its integer
% is ~4.5x easier to pick out, and it resolves in seconds. Real RTK drone receivers
% (u-blox ZED-F9P class) are multi-frequency for exactly this reason.
%
% SYNTHESISED OBSERVABLES BY DEFAULT, BUT NOT BY ASSUMPTION. The observables are
% synthesised from real orbit geometry -- the almanac shipped with Navigation Toolbox,
% through gnssconstellation -- and the engine then does the genuine differencing and
% ambiguity resolution on them. That is the default because a recorded baseline cannot
% fly a show: the show needs observables for wherever the rover actually is this epoch,
% and only a generator can produce those.
%
% What is NOT assumed is that they have to be synthesised. The engine's input is one bus,
% GNSSObservableBus (see createFleetBusObjects), carrying what a receiver actually reports
% -- undifferenced code and phase, one row per receiver. RTKBase/ObservableSource is a
% variant subsystem over that bus, so real measurements drive the identical cascade:
rtk_obs_source = 1;   % 1 = Simulated (generate from the flown rover position)
                      % 2 = Measured  (play back rtk_measured_obs -- see obsBusFromArrays)
rtk_base_lla   = [42.3601, -71.0589, 10];   % surveyed base antenna, WGS84
rtk_epoch      = datetime(2026, 9, 3, 12, 0, 0, 'TimeZone', 'UTC');
rtk_elev_mask  = 10;                        % deg, conventional RTK cut-off

% Carrier frequencies and wavelengths. L1/L2 GPS.
rtk_c        = 299792458;
rtk_f1       = 1575.42e6;
rtk_f2       = 1227.60e6;
rtk_lambda1  = rtk_c / rtk_f1;                % 0.1903 m
rtk_lambda2  = rtk_c / rtk_f2;                % 0.2442 m
rtk_lambda_wl = rtk_c / (rtk_f1 - rtk_f2);    % 0.8619 m -- the wide lane

% Measurement noise, one sigma, per receiver per satellite. The base and the rover are
% NOT the same receiver, and treating them as one is what made time to first fix look
% marginal. A single-differenced observable carries sqrt(sig_base^2 + sig_rover^2), so
% assuming both ends were rover-grade inflated the code noise by 27 %.
%
% The base is the high-fidelity end of this model deliberately: a survey receiver on a
% fixed pillar with a choke-ring antenna, ground-plane shielded against multipath, with no
% vibration and no attitude dynamics. The rover is a patch antenna bolted to an airframe.
%
% Code noise is the load-bearing number in this whole section, because time to first fix
% is dominated by it -- the wide-lane integer is averaged out of the code, so the epochs
% needed scale as its square. Measured over 400 trials, median / 90th-percentile epochs to
% a wide-lane fix, against the single-differenced sigma:
%     0.424 m   (both ends rover-grade)     7 / 16    <- does not fit the window
%     0.335 m   (survey base, as below)     5 / 10
% at 1 Hz corrections into a pre-show window of gps_lock_duration + upload_duration +
% arm_duration = ~14.9 s. Modelling the base honestly is what puts the fix before takeoff.
% The alternative was padding the timeline, and that is a bad trade: 6 s of extra
% pre-flight bought 87.5 % -> 95.5 % coverage, because the tail is long (99th percentile
% 28 epochs, worst seen 37) and no realistic wait covers it.
rtk_sigma_code_base   = 0.15;    % m, pseudorange, survey receiver
rtk_sigma_code_rover  = 0.30;    % m, pseudorange, airborne patch antenna
rtk_sigma_phase_base  = 0.001;   % m, carrier phase -- 1 mm
rtk_sigma_phase_rover = 0.002;   % m, carrier phase -- 2 mm, ~1/100 cycle at L1

% What the engine sees AFTER its own rover-minus-base differencing. The observables now
% arrive undifferenced, one row per receiver, each carrying its own sigma from the pair
% above, so these two are no longer used to generate anything -- they are the amplitude the
% single difference is predicted to come out at, and the tests assert against them. Named
% _sd so they cannot be mistaken for per-receiver sigmas: the difference is a factor of 1.4
% and it would be invisible at a glance.
rtk_sigma_code_sd  = sqrt(rtk_sigma_code_base^2  + rtk_sigma_code_rover^2);   % 0.335 m
rtk_sigma_phase_sd = sqrt(rtk_sigma_phase_base^2 + rtk_sigma_phase_rover^2);  % 0.00224 m

% Receiver clock offsets. Deliberately ENORMOUS -- 3 us is 900 m of range -- because
% cancelling them is the test that separates this from DGPS. A double difference removes
% both receiver clocks exactly; anything that merely differences positions cannot.
rtk_clock_sigma = 3e-6 * rtk_c;   % m, one sigma, per receiver per epoch

% ---- geometry: satellites, line of sight, and the differencing operators -------------
% Static over one show, and that is a statement about DOP rather than about correctness.
% The satellites move ~0.5 deg of line of sight in 124 s, but the SAME geometry matrix
% is used to generate the observables and to solve them, so the motion cancels out of
% the estimate exactly. What a frozen constellation loses is the slow change in DOP over
% the show, which is not what this section exists to demonstrate.
[rtk_sat_pos_all, ~, ~] = gnssconstellation(rtk_epoch);
[rtk_az_all, rtk_el_all, rtk_vis] = lookangles(rtk_base_lla, rtk_sat_pos_all, rtk_elev_mask);
rtk_az    = rtk_az_all(rtk_vis);
rtk_el    = rtk_el_all(rtk_vis);
rtk_n_sat = numel(rtk_az);                  % 8 above 10 deg at this epoch

% The visible satellites' ECEF positions, kept rather than cleared with the rest of the
% constellation, because ObservableGen forms its ranges by calling pseudoranges for the base
% and for the rover and differencing them -- so the model needs the satellites themselves, not
% only the line-of-sight directions derived from them.
rtk_sat_pos = rtk_sat_pos_all(rtk_vis, :);  % rtk_n_sat-by-3, metres

% Unit line-of-sight vectors, ENU, one row per satellite, receiver -> satellite.
% Built from azimuth and elevation rather than by differencing ECEF positions, because
% over a baseline of a few hundred metres the two are identical to well under a
% millimetre and this form needs no coordinate conversion in the model.
rtk_los = [cosd(rtk_el) .* sind(rtk_az), cosd(rtk_el) .* cosd(rtk_az), sind(rtk_el)];

% Pivot satellite = highest elevation. Every double difference is taken against it,
% which is what removes the two receiver clocks. Highest rather than first: the pivot's
% own noise enters every difference, so it should be the least noisy satellite.
[~, rtk_pivot] = max(rtk_el);
rtk_others = setdiff(1:rtk_n_sat, rtk_pivot);
rtk_n_dd   = rtk_n_sat - 1;                 % 7 double differences from 8 satellites

% The double-differencing operator, as a matrix. rtk_D * obs is the vector of double
% differences, so in the model this is one Product block and the differencing is
% visible as data rather than buried in wiring.
rtk_D = zeros(rtk_n_dd, rtk_n_sat);
rtk_D(sub2ind(size(rtk_D), (1:rtk_n_dd)', rtk_others(:))) = 1;
rtk_D(:, rtk_pivot) = -1;

% Design matrix and least-squares gain for the baseline. A row is the difference of two
% line-of-sight vectors, which is why a double difference is sensitive to the baseline at
% all. G is geometry only -- no measurements -- so it is a constant in the model and the
% 3x7 solve costs one Product block instead of an online inverse.
rtk_A = -(rtk_los(rtk_others, :) - rtk_los(rtk_pivot, :));
rtk_G = (rtk_A' * rtk_A) \ rtk_A';          % cond(rtk_A) = 2.52 at this epoch

% Live sigma, as a single gain on the residual norm. The engine reports its own accuracy
% from how well the accepted integers fit. Two facts collapse that to one constant:
%
%   1. Double differences are CORRELATED. Every one of them contains the pivot satellite,
%      so with independent single differences of variance s^2 the double-differenced
%      covariance is s^2 * (rtk_D * rtk_D') = s^2 * (I + ones), not s^2 * I. Treating them
%      as independent -- the obvious thing, and what this line did first -- makes the
%      reported sigma 1.42x optimistic. That is an error in the one direction that
%      matters: the engine would claim more accuracy than it has.
%   2. Both the position covariance and the expected residual scale with that same s^2, so
%      it cancels and the ratio is pure geometry, known before the model runs.
%
%     sigma_pos      = rtk_sigma_gain * norm(residual)
%     rtk_sigma_gain = sqrt( trace(G*Q*G') / trace((I - A*G)*Q) )
%
% so what the model needs is one dot product, one sqrt and one Gain -- no online inverse.
% This is why the reported tier stops being a table entry: it is computed from the fit.
% It was measured against the error actually realised, not just derived.
rtk_Q          = rtk_D * rtk_D';            % DD noise structure: I + ones
% sum(diag(...)) and not trace(...), which is bit-identical (isequal, not merely close --
% trace IS sum(diag)) and cannot be shadowed. This script runs in the BASE workspace, so
% any variable named `trace` left behind by a previously-run script turns these two
% calls into indexing on that variable. It is not hypothetical: it threw
% "Specified key type does not match the type expected for this container." from a
% char-keyed containers.Map named `trace`, surfaced as a warning on this model's PreLoadFcn
% (which is literally 'setupParams'), and brought down an unrelated run some minutes later.
% A numeric `trace` is worse still -- it would silently return an element instead of a
% trace and quietly mis-set the RTK sigma gain. `trace` is a plausible variable name.
rtk_sigma_gain = sqrt(sum(diag(rtk_G * rtk_Q * rtk_G')) / ...
                      sum(diag((eye(rtk_n_dd) - rtk_A * rtk_G) * rtk_Q)));

% ---- validation: residuals above which a HELD integer set is dropped ------------------
% Both fixes are re-validated every epoch, not just at the moment they are accepted. That
% matters more than it sounds: an integer set is latched once and then used for the rest of
% the show, so without continuous validation a single bad latch is permanent and the engine
% spends 124 s reporting a confident, precisely wrong position. The wide lane went
% unvalidated at first and 5 runs in 2000 did exactly that.
%
% One threshold per observable, because the wide lane is 5.74x noisier than L1 -- the
% combination that makes its integer easy to find (0.862 m of wavelength) amplifies phase
% noise by sqrt(f1^2+f2^2)/(f1-f2). Using one number for both is what made 0.05 look
% simultaneously too loose and too tight. Both are set from measurement:
%
%                     correct fit, per-obs residual sigma   one cycle of error
%   L1        (3170 epochs)   0.0031 m mean, 0.0082 m max     0.0550 - 0.0922 m
%   wide lane (4121 epochs)   0.0140 m mean, 0.0424 m max     0.2493 - 0.4176 m
%
% The spread in the one-cycle column is leverage: the same cycle projects differently onto
% each row of the design matrix, so the threshold has to clear the WEAKEST signature.
% 0.05 -- the single value first written here -- sits inside the L1 band, and a marginal
% epoch duly survived the detector while 0.15 m out.
rtk_slip_thresh_l1 = 0.02;   % m, 2.4x above the worst correct fit, 2.75x under one cycle
rtk_slip_thresh_wl = 0.10;   % m, 2.4x above the worst correct fit, 2.5x under one cycle

% ---- the simulated truth the engine is not allowed to see ---------------------------
% Integer ambiguities, per satellite, one per frequency. Deterministic rather than
% random so every run resolves the same integers and the result is reproducible.
%
% These are the BETWEEN-RECEIVER ambiguities: rover minus base. They are what the engine
% actually resolves, whatever the two receivers' own counters happen to read, and so they
% are the reference the resolved integers are judged against. O(100) cycles keeps them at
% the scale of a real between-receiver ambiguity over a short baseline.
rtk_amb_l1 = mod(37 * (1:rtk_n_sat)', 401) - 200;
rtk_amb_l2 = mod(53 * (1:rtk_n_sat)', 401) - 200;

% The two receivers' OWN phase counters. A real receiver initialises its counter when it
% locks the signal, so the absolute value carries no information -- only its difference
% between two receivers does, and that difference is rtk_amb_l1/rtk_amb_l2 above, exactly,
% because these are integers and the rover value is the base value plus the SD value.
%
% They exist because the RTK engine's interface now carries UNDIFFERENCED observables, one
% row per receiver, so that real base and rover measurements can be fed to the same
% cascade. The base counter is deliberately nonzero and different per satellite: zero
% would let something downstream depend on the base row being trivial and
% nobody would notice.
%
% 1e6 cycles is 1.9e5 m at L1, which puts the raw phase observable at 2.35e7 m -- the
% magnitude a RINEX L1 record actually has. That magnitude costs less than it appears to:
% base and rover are within a part in 6774 of each other, so their difference is formed with
% no rounding at all, and what remains is only the cost of representing a raw observable
% (adding a 900 m clock and centimetres of noise to a 2.35e7 m range, then getting the
% difference back out) -- measured at 8.8e-9 m, which is 2.4x eps at that magnitude and
% 4.0e-6 of the phase noise. Arm 15 measures it.
%
% What the magnitude does rule out is packing these values as they stand: at 2.35e7 m a
% SINGLE resolves 2.000 m exactly. So rtcm_payload_bytes carries phase-range RESIDUALS --
% observable minus an approximate range, as a real MSM message does -- never raw phase.
rtk_amb_l1_base  = 1000000 + mod(97 * (1:rtk_n_sat)', 1009);
rtk_amb_l2_base  = 1000000 + mod(89 * (1:rtk_n_sat)', 1009);
rtk_amb_l1_rover = rtk_amb_l1_base + rtk_amb_l1;
rtk_amb_l2_rover = rtk_amb_l2_base + rtk_amb_l2;

% ---- ambiguity validation thresholds ------------------------------------------------
% Plain rounding with a distance-to-integer test, not LAMBDA. With 8 satellites and a
% pivot chosen for elevation the double-differenced ambiguities are only mildly
% correlated, so decorrelation buys little; 0.25 cycles is the standard bootstrapped
% rounding success criterion -- half a cycle is where rounding becomes a coin toss.
rtk_fix_thresh_wl = 0.25;   % cycles, wide lane
rtk_fix_thresh_l1 = 0.30;   % cycles, L1 conditioned on the wide-lane baseline

% A single-epoch distance-to-integer test is enough HERE, and only because the fix is
% re-validated every epoch afterwards (see rtk_slip_thresh_wl below). The obvious
% alternative -- requiring N consecutive epochs within threshold before latching, to stop
% an average being caught while it drifts past the wrong integer -- was built and measured:
% 3 epochs of hold removed every wrong latch (5 in 2000 -> 0), but so did continuous
% validation on its own, and the hold cost 5 epochs of time to first fix (median 5 -> 8,
% 90th percentile 10 -> 15, which no longer fits the pre-show window). It was removed
% rather than kept at 1, because a counter and a comparator in the model that never change
% an outcome are worse than no blocks at all. Detecting a bad latch beats delaying it.

% ---- what the engine achieves, measured -----------------------------------------------
% Measured at the sigmas above, over a rover sweeping 58-69 m from the base. These are
% FALLBACK values: the model reports its sigma live from the residual of whichever solution
% it is holding, so these exist as a reference to check against rather than to be believed.
% Per-epoch position error, rover minus base:
%   code-only double difference (no phase)   0.742 m mean   <- the DGPS-grade answer
%   wide-lane fixed                          0.028 m mean, 0.108 m worst (4121 epochs)
%   L1 fixed                                 0.005 m mean, 0.021 m worst (300 trials)
% Two of those are worth reading twice. The L1 worst case landing at 0.02 m says the tier
% the model always asserted was right, it just had nothing behind it. And code-only lands
% at 0.74 m rather than the 1.5 m of gps_accuracy_standard, which is also correct and not a
% contradiction: this is a DIFFERENTIAL solution with both receiver clocks and the common
% atmosphere already cancelled, so it beats standalone GNSS even before the phase is used.
rtk_sigma_unfixed  = 0.75;    % m, code-only double difference
rtk_sigma_wl_fixed = 0.042;   % m, between the measured mean and worst above
rtk_sigma_l1_fixed = 0.01;    % m, ~2x the measured mean, ~1/2 the measured worst

% Status codes the engine reports, and the value StatusToBaseRate holds before the first slow
% step has run. 3 rather than 1 deliberately: the rate transition's initial output is what the
% launch gate sees on tick one, and seeding it with the BEST state would open the gate for one
% step before the engine has said anything -- the same inversion rtk_sigma_unfixed avoids on
% the sigma path. Note the scale is not monotonic in quality: 1 is best, 3 is worst.
rtk_status_l1      = 1;       % L1 fixed -- the state the launch gate requires
rtk_status_wl      = 2;       % wide-lane fixed
rtk_status_unfixed = 3;       % code-only, and the pre-first-step hold value

% ---- noise amplitude and seeding, per receiver per satellite -------------------------
% Base row then rover row, so ONE Random Number block per observable still covers both
% receivers and the two rows carry genuinely different amplitudes. The Random Number block
% takes its output dimensions from these parameters, which is why the split cost no extra
% blocks: the survey base at 0.15 m and the airborne rover at 0.30 m come out of the same
% block as a 2-by-rtk_n_sat signal.
%
% Variance, not sigma. The block's parameter is variance, and passing a sigma silently gives
% noise at sqrt(sigma) -- 0.55 m where 0.30 m was meant, which still looks like noise.
rtk_mean_obs = zeros(2, rtk_n_sat);
rtk_var_p    = [rtk_sigma_code_base^2;  rtk_sigma_code_rover^2 ] * ones(1, rtk_n_sat);
rtk_var_l    = [rtk_sigma_phase_base^2; rtk_sigma_phase_rover^2] * ones(1, rtk_n_sat);

% One seed per (observable, receiver, satellite), 8*rtk_n_sat + 2 = 66 of them, laid out in
% non-overlapping BLOCKS of 2*rtk_n_sat. The block stride is what matters, and getting it
% wrong is not hypothetical: the previous layout was rtk_seed_p1 = 9101, rtk_seed_p2 = 9102,
% ... with a per-satellite offset of (0:rtk_n_sat-1) added to each, so P1 satellite s+1 drew
% the SAME stream as P2 satellite s -- 32 seeds spanning only 11 distinct values, 21 of the
% streams duplicates of another. That correlated adjacent satellites' Melbourne-Wubbena
% arcs, the one quantity whose averaging assumes independence. It was visible in arm 11 all
% along as a minimum pair spread of 0.612 m against 0.671 m expected (a ratio of sqrt(3)/2,
% the signature of two streams sharing one of their two components) under a threshold loose
% enough to pass it. Arm 16 now checks the count directly instead.
%
% A scalar seed would be worse and quieter still: the block scalar-expands it, giving every
% satellite bit-identical noise, which still looks like noise and still averages down.
rtk_seedm_p1  = 9101 + reshape(0:2 * rtk_n_sat - 1, 2, rtk_n_sat);
rtk_seedm_p2  = rtk_seedm_p1 + 2 * rtk_n_sat;
rtk_seedm_l1  = rtk_seedm_p2 + 2 * rtk_n_sat;
rtk_seedm_l2  = rtk_seedm_l1 + 2 * rtk_n_sat;
rtk_seedm_clk = 9101 + 8 * rtk_n_sat + (0:1)';

% The two receiver clocks, one seed each, drawn independently. rtk_clock_sigma is per
% receiver, so this is where the differential clock the engine has to cancel comes from --
% it is formed by the engine's own rover-minus-base differencing rather than injected
% pre-differenced, which is what makes arm 1 a test of the cascade and not of this line.
rtk_mean_clk = zeros(2, 1);
rtk_var_clk  = rtk_clock_sigma^2 * ones(2, 1);

clear rtk_sat_pos_all rtk_az_all rtk_el_all rtk_vis

%% RTK Correction Link
% TWO INDEPENDENT FAILURE MODES, and keeping them separate is the whole point of
% this section. They were conflated until 2026-08-31: HasGNSSFix was driven by the
% age of the correction, so a stale correction triggered IMU dead reckoning. That
% is the wrong physics. Losing the correction stream does not remove the
% satellites -- the receiver still computes a position, just a worse one, and the
% error is BOUNDED. Only losing GNSS itself (jamming, antenna failure) causes
% unbounded dead reckoning. RTK is a differential technique: with no satellites
% there are no observables to correct, so GNSS availability is the PRECONDITION,
% not a parallel path.
%
%   no GNSS       -> the receiver dead-reckons, error ~ 0.5*dt^2, UNBOUNDED, and
%                    corrections are irrelevant in this state. OUT OF SCOPE, below.
%   GNSS + stale  -> solution degrades 0.02 -> 0.3 -> 1.5 m and STOPS there.
%                    Ruins the formation, does not lose the drone. This is the one
%                    the model implements, per drone.
%
% ONLY the bounded path is modelled here: usable correction -> age -> SigmaLUT ->
% extra error added to the INS position output in DroneFleet/Navigation. It is per drone.
%
% The unbounded path -- actual loss of satellites -- is deliberately NOT modelled, and
% the reason is a property of the INS block rather than a shortcut. That block does
% have a HasGNSSFix input (the dialog flag that exposes it is confusingly named
% TimeInput), but enabling it changes how the block reads its own inputs: stepImpl
% then loops over the ROWS of Position treating them as successive time samples of one
% sensor, with timestamps spread across a single Ts_sim by getSampleTimeForFrame. With
% a vectorised fleet those rows are drones, not instants, so a false element means "no
% fix at sub-sample 5 of this frame" and PositionErrorFactor*t^2 is ~0. Measured: 15 s
% of supposedly total denial moved a drone 0.07 m.
%
% Getting real per-drone dead reckoning therefore needs one INS instance per drone in
% a For Each Subsystem, where numSamples is 1 and HasGNSSFix is a genuine scalar per
% iteration. That is a real option, not a dead end -- it is just a separate decision,
% because it trades the vectorised sensor for N of them at 40-200 drones.
%
% rtcm_interval is the real rate: RTCM corrections go out at ~1 Hz, not once per
% Ts_sim. Broadcasting one every 10 ms (what the model did before) is 100x too
% fast and makes "age of correction" permanently zero, which is why the fleet
% used to sit at 0.02 m no matter what the link did.
%
% rtk_timeout must exceed rtcm_interval or the fix would drop between every
% correction in normal operation. How MUCH it exceeds it decides how many lost
% corrections in a row the fleet rides out, and that is a modelling choice, not a
% cosmetic one. A single lost frame drives corrAge to 1.990 s (measured), so:
%
%   1.5 s  ONE lost frame always drops the fix. The default show broadcasts 62
%          corrections (measured: 62 edges into zero, first at t = 11.84), so at
%          packet_loss_rate = 0.01 the EXPECTED count is 0.6 dropouts per run --
%          but the shipped loss_seed draws 2, at t = 18.50 and t = 66.50. Because
%          the loss is TRANSMITTER-side it takes the whole fleet at once: every
%          drone steps sideways up to 0.8 m within a single Ts_sim. Both the count
%          and where it lands are luck, and t = 18.50 lands mid-climb. On screen
%          that does not read as a modelled RTK dropout, it reads as a glitch.
%   2.5 s  one lost frame is ridden out (1.990 < 2.5); TWO CONSECUTIVE losses still
%          drop it, at 1e-4 per pair instead of 1e-2 per frame.
%
% 2.5 s, and the reason is fidelity rather than appearance: a real receiver does not
% abandon an RTK fix because one 1 Hz correction epoch went missing -- age-of-
% differential runs to seconds routinely. Dropping on the first miss was the
% brittle model, not the conservative one.
%
% Nothing about the degradation is given up, because the two paths into it are
% independent (see WorseOf below). rtk_deny_mask still works exactly as before, one
% second later: a denied drone's age grows without bound, so it crosses 2.5 s and
% then rtk_float_timeout on its way out. Measured with drone 3 denied: loses fix at
% age 2.5 (t = 11.5), Float at 0.299 m, Standard at 1.500 m from t = 18.5 onward,
% fix held only 15 % of the run -- while every undegraded drone now has ZERO
% dropouts. Sustained loss still reaches Standard at 1.5 m; it just takes sustained
% loss to get there, which is what "sustained" ought to mean.
%
% Turn it back to 1.5 to see a lost correction bite on the first frame -- it is a
% legitimate thing to demonstrate, just not with the whole fleet, unannounced, in
% the middle of the takeoff.
rtcm_interval     = 1.0;   % RTCM broadcast period (s)
rtk_timeout       = 2.5;   % age beyond which the RTK fix is lost      -> Float (s)
rtk_float_timeout = 10.0;  % age beyond which Float degrades           -> Standard (s)

% BaseStation's RTCMClock is a sample-based Pulse Generator, so the period is in
% ticks, not seconds. Sample-based rather than time-based deliberately: it pulses
% for exactly one Ts_sim step regardless of solver behaviour, so a correction is
% never double-counted or missed.
rtcm_period_ticks = rtcm_interval / Ts_sim;

% HEARTBEAT and SYSTEM_TIME are 1 Hz messages in MAVLink. Before this, BaseStation sent
% SYSTEM_TIME on 59.8 % of 100 Hz ticks (~60 Hz) and HEARTBEAT on 34.6 % (~35 Hz), because
% both chains fed a MultiPortSwitch that had to select *something* every tick and HEARTBEAT
% was its default. So the rate was the fidelity defect, exactly as it was for RTCM above --
% decimating to 1 Hz makes the model more correct, and only incidentally faster.
mavlink_hb_interval = 1.0;   % HEARTBEAT broadcast period (s)
mavlink_st_interval = 1.0;   % SYSTEM_TIME broadcast period (s)
hb_period_ticks = mavlink_hb_interval / Ts_sim;
st_period_ticks = mavlink_st_interval / Ts_sim;

% The half-period offset is load-bearing, not cosmetic. SYSTEM_TIME outranks HEARTBEAT in
% BaseStation's priority cascade, and during Phase 3 both messages are live. If the two
% clocks pulsed on the same tick, SYSTEM_TIME would win *every* time and the heartbeat would
% go out at 0 Hz for the whole show -- worse than the 35 Hz it replaced. Offsetting makes
% them alternate, so each genuinely gets its 1 Hz.
st_phase_ticks = st_period_ticks / 2;

% GPS_RTCM_DATA carries a fixed 180-byte data field plus a `len` field saying how many of
% those bytes are actually valid. FillRTCM assigns len from this, because the blank message
% defaults it to 0 -- and a frame that declares zero valid bytes is discarded by any real
% MAVLink receiver no matter how correct its payload is.
%
% The payload used to be 12 bytes: the 3x1 NED position correction, as three singles. That
% was the DGPS artefact on the wire, and it was a DGPS artefact in the strict sense -- a
% broadcast position correction cannot carry carrier-phase information at all, because
% ambiguities are per-receiver integers rather than a spatially correlated error, so a drone
% receiving one can only ever reach the code solution. That ceiling was measured:
% 0.7582 m code-only against 0.0049 m L1-fixed, a factor of 154.
%
% A real RTK base does not broadcast a position correction. It broadcasts its OWN observables
% and lets each rover difference them locally -- which is also why RTK scales to a 500-drone
% show: the uplink is empty and the broadcast cost is independent of fleet size. So the
% payload is the base station's four observables per satellite, as singles:
%
%     4 observables (P1, P2, L1, L2) x 4 bytes x rtk_n_sat satellites = 128 bytes
%
% which is the right order of magnitude for MSM4 over 8 GPS satellites and still inside the
% 180-byte field. Derived rather than typed, so it cannot fall out of step with the
% satellite count.
rtcm_data_bytes    = 180;                 % the fixed width of GPS_RTCM_DATA.data
rtcm_payload_bytes = 4 * 4 * rtk_n_sat;   % base observables in GPS_RTCM_DATA.data

% PackRTKDataBlk pads the remainder, so a constellation that outgrew the field would not fail
% as "too big" -- Constant would build zeros(1, negative) as an empty, Concatenate would emit a
% 128-wide vector, and the byte-for-byte 180 the message needs would simply be short. Caught
% here instead, where the arithmetic is visible.
assert(rtcm_payload_bytes <= rtcm_data_bytes, ...
    'setupParams:rtcmPayload', ...
    ['%d satellites need %d payload bytes, but GPS_RTCM_DATA.data holds %d. ' ...
    'Either drop satellites or split the observables across frames, as MSM does.'], ...
    rtk_n_sat, rtcm_payload_bytes, rtcm_data_bytes);

% ---- the rough range the payload is differenced against ------------------------------
% RAW observables cannot go on this wire, and that is measured rather than feared: at
% 2.47e7 m a single resolves 2.000 m (arm 15). So the payload carries RESIDUALS against a
% rough range, exactly as an MSM message does, and this is that rough range.
%
% It is the base-to-satellite range with no clock and no noise, so a drone can RECONSTRUCT
% it -- a real rover has the station position (broadcast as RTCM 1005/1006) and the
% ephemeris, so it can compute this itself and add it back. Differencing against something
% the receiver cannot recompute would not be a residual, it would be data loss.
%
% Constant because the base is surveyed and the constellation is frozen over one show, so
% this costs one Constant block rather than a per-epoch range computation.
%
% Subtracting this is NOT sufficient on its own for the phase rows. Three candidates,
% measured over 25 clock draws against the 2.236e-3 m phase noise that has to survive the
% trip. Magnitudes are worst-case over those draws, so the resolution column is the worst
% the wire can do rather than a typical figure:
%
%      transmitted quantity             magnitude    single() error   vs phase noise
%      raw phase                         2.47e7 m       2.000 m       unusable (arm 15)
%      phase - rough range              ~2.47e5 m       7.8e-3 m      3.5x too COARSE
%      phase - rough range, code-aligned ~2.3e3 m       1.2e-4 m      19x finer, usable
%
% So the phase rows are also CODE-ALIGNED: the base's own accumulated phase counter
% (rtk_amb_l1_base/rtk_amb_l2_base) is folded out before transmission. That is not a
% shortcut, it is what MSM does -- a real receiver reports a phaserange that sits within
% metres of its pseudorange, not 2.4e5 m away from it. The ~1e6-cycle counters above are
% RINEX semantics (accumulated counts); this wire is RTCM semantics.
%
% The middle row was checked live rather than argued: a real epoch packed through the
% shipping blocks and unpacked as a rover would, in BOTH directions -- code-aligned
% residuals recover the base observables well inside the phase noise, and rough-range-only
% residuals do not. The table above is a claim; that round trip is the measurement, so if
% the constellation or the clock model changes it is the first thing to re-measure.
%
% Consequence worth naming: a rover differencing this recovers its OWN ambiguity rather
% than the between-receiver one, because the base's contribution is already removed. That
% changes which integer a future drone-side solve resolves -- not whether it can.
rtk_rough_range = pseudoranges(rtk_base_lla, rtk_sat_pos, 'RangeAccuracy', 0);

% Breakpoints and table for the SigmaLUT block in
% RadioChannel/CorrectionAgeMonitor. Interpolation is Flat, so this
% is a staircase rather than a ramp -- Fix and Float are discrete receiver states,
% not points on a continuum.
rtk_sigma_bp    = [0, rtk_timeout, rtk_float_timeout];

% The INS already contributes gps_accuracy_rtk_fix of noise on its own, so only the
% SHORTFALL is injected, added in quadrature. Without the subtraction the fixed-mode
% error would come out at sqrt(2)*0.02 and the nominal case would fail its own spec.
rtk_sigma_tiers = [gps_accuracy_rtk_fix, gps_accuracy_rtk_float, gps_accuracy_standard];
rtk_sigma_table = sqrt(max(0, rtk_sigma_tiers.^2 - gps_accuracy_rtk_fix^2));

% Recovery is not instant. A receiver that regains satellites, or a correction after
% an outage, has to re-acquire and re-converge (Standard -> Float -> Fix) over
% seconds. SigmaRateLimit therefore lets the error rise instantly but fall only this
% fast. Without it a single late correction snaps the fleet from 1.5 m back to
% 0.02 m in one 10 ms step, which no receiver does.
reconverge_time = 5.0;                                     % s, Standard -> Fix
rtk_fall_slew   = -gps_accuracy_standard / reconverge_time; % m/s, negative

% GNSS position error is correlated over tens of seconds (ionosphere, multipath),
% not white. This matters more than it looks: white noise at 100 Hz is filtered out
% by PositionController and the drone would barely move, so the degradation would be
% implemented and yet invisible. A first-order lag turns the white source into a
% wandering bias:
%     y[k] = gnss_err_pole*y[k-1] + gnss_err_gain*u[k],   u ~ N(0,1)
% Var(y) = gain^2/(1-pole^2), so gain = sqrt(1-pole^2) holds the output at unit
% variance and the sigma from the lookup is the true 1-sigma of the injected error.
gnss_err_tau  = 5.0;                            % error correlation time (s)
gnss_err_pole = exp(-Ts_sim / gnss_err_tau);
gnss_err_gain = sqrt(1 - gnss_err_pole^2);
% Based at 40000 rather than 8000 because this block GROWS WITH THE FLEET -- 3*N_uav
% seeds -- and every other seed block here is fixed-size. At 8000 it ran into the RTK
% observable block at 9101 once N_uav reached 367, and at the 500 the app allows, 66 of
% these were bit-identical to an RTK stream: drones 101-166 drew their DOWN-axis GNSS
% error from the same numbers as a satellite observable. That is the same defect the
% rtk_seedm_* comment above describes fixing WITHIN that block, recurring across the
% boundary between two blocks -- and the distinct-count check that fix added cannot see
% it, because it only counts inside the RTK block. 40000 clears uav_loss_seed (31001+)
% and leaves room for a fleet of 3000 before anything else is in reach.
gnss_err_seed = reshape(40000 + (1:(N_uav * 3)), N_uav, 3);  % per drone, per axis


%% Per-UAV Link Degradation
% Independent per-drone reception of the RTCM broadcast. packet_loss_rate erases the
% whole frame at the transmitter, so it is fleet-wide and perfectly correlated; this
% is the receive-side failure and is uncorrelated between drones. 0 = every drone
% hears every broadcast, which is the default the rest of the model assumes.
if ~exist('uav_loss_rate', 'var'), uav_loss_rate = 0; end
uav_loss_seed = 31000 + (1:N_uav);

% Operator-forced degradation, true = degraded. Both are live-tunable DURING
% simulation: assign in the base workspace and issue
% set_param(mdl,'SimulationCommand','update'). Verified live: only the elements you
% change take effect, and they take effect at the moment you change them, so no
% degrade_time is needed anywhere.
% The numel guard matters because re-running this at a different fleet size -- 40 then
% 200, say -- would otherwise leave a stale 40-element mask in place, and it would
% survive and fail dimension propagation rather than being rebuilt.
if ~exist('rtk_deny_mask', 'var')  || numel(rtk_deny_mask)  ~= N_uav
    rtk_deny_mask  = false(N_uav, 1);   % lose the CORRECTION: bounded, 0.3 -> 1.5 m
end
% NOT an operator control, and must stay all-false until the outage path exists. It
% survives only as the nesting term into RTCMArrived -- a correction is unusable if
% there are no satellites to apply it to -- which is one logical op at all-false and
% saves rebuilding the structure later. Setting an element true today would make that
% drone's corrections unusable and degrade it to the STANDALONE tier, which is bounded
% at 1.5 m; a real GNSS outage is unbounded. That would be wrong physics presented as
% a feature, so the app does not expose it.
if ~exist('gnss_deny_mask', 'var') || numel(gnss_deny_mask) ~= N_uav
    gnss_deny_mask = false(N_uav, 1);
end

%% Communication Parameters
if ~exist('comm_latency', 'var'), comm_latency = 0.02; end    % seconds
if ~exist('comm_jitter', 'var'), comm_jitter = 0.005; end     % seconds
if ~exist('comm_timeout', 'var'), comm_timeout = 3.0; end     % seconds (heartbeat timeout)
if ~exist('packet_loss_rate', 'var'), packet_loss_rate = 0.01; end % 1% loss
% Seed for the RadioChannel Bernoulli packet-loss roll. The app re-rolls this
% before every upload attempt so a retry sees a fresh loss pattern.
if ~exist('loss_seed', 'var'), loss_seed = 12345; end

%% MAVLink Pre-Flight Upload Parameters
% 2 Hz is what a real mission upload looks like: the ground station sends sparse
% waypoints and the flight controller interpolates between them onboard. It also
% costs 5x less wall clock than the 10 Hz it replaced, because the pre-flight
% window shrinks from 6210 to 1250 items on the default show.
% What the modelled link IS: the chain Fill_MII -> Ser_MII -> RadioChannel ->
% Deser_MI -> BufferWrite is scalar end to end, so exactly one MISSION_ITEM_INT
% crosses per Ts_sim tick -- 100 msg/s x 50 B = 5000 B/s, roughly a third of
% mavlink_bandwidth below. So these upload durations are right for ONE 115200-baud
% telemetry radio and slow for a show network, which would be ~100 kB/s of useful
% payload over WiFi/mesh. mavlink_bandwidth is only checked for sufficiency (see
% bandwidth_capacity further down); it does not set the rate, the tick does.
% The rate cannot simply be raised: the MAVLink Serializer returns a fixed 50-byte
% output for one bus struct and the Deserializer returns one message bus per step.
% QueueOutputMsg only buffers frames that arrive together and drains them one per
% step, so a burst of K costs K ticks to decode. More throughput means K parallel
% chains (K radios, not one fast one) or a finer Ts_sim (whole-model cost). The
% cheaper lever is fewer items, which is what the two parameters below do.
upload_waypoint_rate = 2;          % Hz - MAX waypoint density on the wire
% Cap on keyframes per timeline segment. This is what stops the item count from
% scaling with show duration: a segment is described by at most this many points,
% however long it is. 16 is chosen so that no transition in the default show gets
% coarser -- an 8 s transition gets 16 points, which IS 2 Hz -- while a 49 s
% auto-fitted transition gets 16 instead of 98. It is safe because transitions
% are minjerkpolytraj over
% TWO waypoints: a straight line in space with a shared normalized time profile,
% so coarser keyframes move the fleet along the same lines and visit the same
% formation blend shapes. Only the speed profile stair-steps. See the note above
% upload_times for the measured before/after.
upload_keyframes_max = 16;         % points per segment, moving segments
% A ROTATING hold is the one moving segment the paragraph above does not cover, because
% its trajectory is not a straight line: coarse keyframes there do NOT visit the same
% points, they cut the arc's chords. So a rotating hold is sized by angle instead of by
% time -- see upload_seg_kf below. The sag is bounded as a FRACTION of the spin radius
% because that is the quantity that is scale-free: sag and radius grow together, so one
% number covers a 5 m Grid and a 60 m billboard. 1.5% puts a full turn on ~20 sides and
% keeps the absolute error a few centimetres, well under the metres of separation the
% formations are built with.
upload_arc_sag_frac = 0.015;       % max chord sag / spin radius, rotating holds
upload_keyframes_arc_max = 48;     % points per rotating hold
mavlink_bandwidth = 14000;         % bytes/sec effective (115200 baud w/ framing)
mavlink_packet_size = 50;          % bytes per MISSION_ITEM_INT (37 payload + 12 framing)
% No ack_timeout / max_retries here. Retransmission is not timer-driven in this model: the
% BaseStation resends whatever item the drone asks for in MISSION_REQUEST_INT, so gap repair is
% driven by the request stream itself and there is no timeout to tune and no retry ceiling.
gps_lock_duration = 2;             % seconds sitting on ground before upload
arm_duration = 2;                  % seconds between upload complete and show start

% RTK launch gate (RtkGate chart -> ShowSupervisor/RtkReady).
%
% gps_lock_duration above is a PLACEHOLDER TIMER for "GPS is good enough to fly", and 2 s is a
% fiction: the base engine sits code-only (Status 3) for ~13 s while the Melbourne-Wubbena
% running mean converges far enough for the all-or-nothing wide-lane latch to close. Measured:
% 13 epochs at Status 3, then straight to 1, never once 2 -- the wide-lane latch is the gate,
% not L1. Over that window the engine's sigma reaches the fleet through
% RadioChannel/CorrectionAgeMonitor's EngineSigma input, so per-drone posSigma runs
% 0.17-1.60 m and navigation error peaks 4.39 m. One epoch after the fix, posSigma is 0 and
% navigation error is 6 cm.
%
% So the launch is gated on the engine's ACTUAL fix state instead. This is what real operations
% do -- it is a pre-arm check, and it blocks ARMING rather than mission transfer, because an
% operator uploads the mission WHILE the base surveys in and the rovers converge. Gating the
% upload instead would serialise two things that are naturally parallel and cost ~5 s more.
%
% The dwell exists because a fresh fix can be a WRONG fix. The cascade re-validates its held
% integer set on every epoch and un-latches on failure, so an instantaneous Status == 1 test
% could release the show on an epoch that regresses on the next tick. Real receivers require the
% fix held for the same reason. Seconds rather than epochs so the value does not silently change
% meaning if rtcm_interval is retuned.
rtk_lock_dwell = 2.0;              % seconds Status must hold at 1 (L1 fixed) before launch

% There is deliberately NO timeout. A permanent no-fix is unreachable with the simulated
% source: SatValid is a Constant ones(rtk_n_sat,1) and rtk_sat_pos is a fixed 8-by-3, so
% nCand >= 4 always holds, the "cannot solve" branch (Sigma 1e3) is dead, and a running mean
% over stationary noise converges with probability 1. A timeout would therefore be code that
% can never fire -- the exact anti-pattern this model already carries twice (the app's nav_mode
% dropdown, the fading branch). Instead the hold is ANNOUNCED: NoFly is true while the gate is
% closed, which is what a real GCS shows the operator, and a test that denies satellites can
% assert it never clears.

% Gap-repair retransmission (UploadIndexerChart REPAIR state).
% After the bulk item stream, the base station retransmits only the waypoints
% each drone reports missing via MISSION_REQUEST_INT, one per tick. A drone
% that goes silent for repair_stall_limit consecutive ticks is declared failed,
% which drives the supervisor to UPLOAD_FAILED so the operator can retry.
repair_stall_limit = ceil(comm_timeout / Ts_sim);

% MAVLink address banking (UploadIndexerChart -> BaseStation/Transmitter -> DroneFleet/Receiver).
% MAVLink target_system is 8 bits, so one system ID per drone caps a single network at 255
% addresses -- and a fleet above that silently loses its top drones, because uint8() saturates.
% Larger fleets are therefore addressed as (target_system, target_component) pairs: the low byte
% counts drones within a bank, the high byte counts banks. A real show of this size flies several
% radio networks and the bank would be implicit in which radio carried the packet; the model
% collapses them onto one channel and makes the bank explicit instead, so every packet stays
% self-describing on a single simulated link. 250 rather than 255 keeps the low byte clear of the
% top of the range and keeps the arithmetic readable: drones 1-250 are bank 1, 251-500 bank 2.
mav_sys_span = 250;                % drones addressed per MAVLink system-ID bank

% Operator upload request (UploadReqConst -> ShowSupervisor/UploadRequest).
% The supervisor will not leave IDLE until this is asserted, and once it latches
% UPLOAD_FAILED it needs a fresh false->true edge to retry. The app drives this
% from its "Upload Trajectory" button; scripted runs leave it asserted.
if ~exist('upload_request', 'var'), upload_request = true; end

% Operator abort (AbortReqConst -> ShowSupervisor/AbortRequest). Asserting this
% mid-flight sends the fleet straight down from wherever it is, whatever the plan
% said -- see the landing network in DroneFleet/SetpointGenerator/LandingOverride
% (LatchMemory / DropAccum / SwCmdPos). Distinct from Stop, which halts the
% simulation: an abort is a MISSION command and the run continues until the fleet
% is on the ground. The app drives it from "Abort & Land"; scripted runs leave it
% false so the show flies its planned landing instead.
if ~exist('abort_request', 'var'), abort_request = false; end

%% Reference Location (for GPS LLA conversion)
ref_lla = [42.3601, -71.0589, 0];  % Boston, MA (lat, lon, alt)

%% Gravity
gravity = 9.81;  % m/s^2

%% Create Bus Objects for fleet telemetry
% These will be defined when the model initializes. rtk_n_sat is passed too because
% GNSSObservableBus -- the RTK engine's input contract -- is sized by satellite slots,
% not by fleet size.
createFleetBusObjects(N_uav, rtk_n_sat);

disp(['Setup complete: ' num2str(N_uav) ' UAVs configured']);

%% Show Configuration
if ~exist('show_altitude', 'var'), show_altitude = -10; end       % NED (negative = up)
if ~exist('formation_spacing', 'var'), formation_spacing = 5; end % meters between drones
if ~exist('transition_duration', 'var'), transition_duration = 15; end % seconds per transition
if ~exist('hold_duration', 'var'), hold_duration = 10; end         % seconds to hold each formation
num_samples_per_seg = 50; % trajectory samples per transition segment

% Spin each formation about its own vertical axis while the fleet holds it.
%
% A hold used to be a single anchor sample -- the fleet arrives, and the next
% anchor carries the same positions, so the interpolation stands still. Rotation
% turns that one anchor into a sample series and changes NOTHING else: it is a
% plan-side feature end to end, with no model edit, because the trajectory is
% streamed to the drones as waypoints either way.
%
% The rotation is rigid about the horizontal centroid of the formation, so each
% drone traces a horizontal circle whose radius is its own distance from that
% axis -- outer drones sweep wide, central ones barely move. Being rigid makes it
% an isometry, so a rotating hold cannot breach d_min if the static formation
% clears it, and min_sep_achieved does not move when this is switched on.
%
% rotation_turns is a REQUEST, not a promise. Tangential speed is w*r and the
% fleet only tracks v_track_target, so formationHoldSamples sweeps as far as is
% flyable in the hold it is given and reports what it managed. Expect less than a
% full turn at the default 5 s hold: rotation_min_hold below says what the
% request would actually need, and it is usually tens of seconds.
if ~exist('formation_rotate', 'var'), formation_rotate = false; end
if ~exist('rotation_turns', 'var'), rotation_turns = 1; end   % negative = other way

% WHICH formations rotate, and how far each one turns: one entry per step of
% formation_sequence, in DEGREES. Zero means that hold does not rotate at all, and
% the sign is the direction.
%
% Per STEP, not per formation TYPE, and the difference is real: a Grid->Circle->Grid
% show can spin the first Grid and leave the second still. Colour cannot do that
% (lighting_colors is indexed by formation type, see the app's Colour row), but every
% rotation quantity below is already 1-by-num_formations, so the hold is the natural
% granularity for this one and no new bookkeeping is needed to support it.
%
% Degrees rather than turns because a partial sweep is what actually gets flown --
% the caps below trim a full turn to a few tens of degrees at a typical hold -- and
% "90" reads better than "0.25". formationHoldSamples still takes turns, so the two
% call sites divide by 360 there.
%
% Left EMPTY here and expanded once num_formations is known, further down.
% formation_rotate and rotation_turns stay as the fleet-wide shorthand: an empty
% request with the flag set means "every formation, rotation_turns of a turn", which
% is exactly what the single checkbox meant before per-formation angles existed.
if ~exist('rotation_deg_request', 'var'), rotation_deg_request = []; end

% Largest formation radius the planner can still fly, in metres.
%
% Circle and Sphere were laid out by holding the SPACING along a single ring:
% radius = N_uav*spacing/(2*pi). That is linear in the fleet, so the distance a drone
% transits between formations is linear in the fleet too, and the transition time the
% planner needs follows it -- measured at needT ~= 0.686*radius across four fleet sizes
% (79.6 m -> 53.9 s, 159.2 -> 108.8, 318.3 -> 219.1, 795.8 -> 550.7). The app's
% Transition field stops at 120 s, so 120/0.686 = 175 m is where a single ring stops
% being flyable. That, not the geometry, is what pinned the fleet at 200: at spacing 5
% the single-ring law reaches 175 m at N = 220.
%
% Beyond this radius Circle becomes concentric rings and Sphere a thinner shell, both
% sized by AREA instead of circumference, so the extent grows as sqrt(N). The threshold
% is expressed as a radius rather than a fleet size on purpose: it is the transit that
% the planner cannot resolve, and spacing moves it as much as N does. Every fleet size
% at or below the old cap still takes the single-ring branch and is bit-identical.
formation_radius_max = 175;

% Formation sequence: 1=Grid, 2=Circle, 3=Sphere, 4=Text, 5+ = custom shapes
% loaded from a picture or an STL (see formationFromMedia / custom_formations).
if ~exist('formation_sequence', 'var'), formation_sequence = [1, 2, 3, 2, 1]; end

% What formation type 4 spells. Set this and type 4 flies YOUR text -- it is
% rendered with a real font by formationFromText, not picked from a list of
% built-in shapes. "|" starts a second line, which is worth using on anything long:
% one line of 13 characters is a 13:1 letterbox and there is no height left to put
% drones in, while two lines of 6-7 read at half the fleet. The app's "Fly Text"
% button sets this, and can also register a string as its own named formation so
% one show can spell several different words.
if ~exist('formation_text', 'var'), formation_text = 'HI'; end
% num_formations is DERIVED from formation_sequence, which makes the ~exist guard a trap
% the other overridables do not have: a value left in the base workspace by a LONGER show
% survives into a shorter one and indexes past the sequence, and the app assigns the pair
% together so an app session leaves one behind. Overriding it downward is legitimate --
% "fly only the first three steps" -- so the guard stays and the value is clamped instead
% of replaced. DroneShowExample clears it in section 1 for the same reason.
if ~exist('num_formations', 'var'), num_formations = length(formation_sequence); end
num_formations = min(num_formations, length(formation_sequence));

% ---- expand the rotation request to one entry per formation ------------------
% Deferred to here because it has to be num_formations long and that is only known
% now. Three ways in, in order of precedence:
%
%   1. rotation_deg_request given explicitly -- the per-formation spec wins.
%   2. formation_rotate set with no spec -- the old fleet-wide shorthand, every
%      formation asked for rotation_turns of a turn. Unchanged behaviour.
%   3. neither -- nothing rotates.
%
% The length guard RESIZES rather than replacing: the sequence is edited far more
% often than the angles are, so a show whose sequence grew from 3 steps to 5 should
% keep the angles already chosen for steps 1-3 and leave the new holds still. Silently
% replacing the whole thing with zeros would look like the app forgetting the setting.
% (Contrast rtk_deny_mask, whose guard does replace -- which is why the app resizes
% that one itself before calling in.)
if isempty(rotation_deg_request)
    if formation_rotate
        rotation_deg_request = repmat(360 * rotation_turns, 1, num_formations);
    else
        rotation_deg_request = zeros(1, num_formations);
    end
else
    rotation_deg_request = reshape(rotation_deg_request, 1, []);
    if numel(rotation_deg_request) < num_formations
        rotation_deg_request(end+1:num_formations) = 0;
    elseif numel(rotation_deg_request) > num_formations
        rotation_deg_request = rotation_deg_request(1:num_formations);
    end
end

% formation_rotate is now DERIVED: it means "does anything in this show rotate", and
% the places that used it to decide plan shape (the upload sizing, below) ask the
% per-formation flag instead. Kept because it is still the honest answer to that
% question, it is what the app's master checkbox maps onto, and a show with nothing
% rotating has to stay bit-identical to the pre-rotation plan.
rotation_active = rotation_deg_request ~= 0;
formation_rotate = any(rotation_active);

% Samples per hold, one entry per formation. ONE means "do not rotate", and it is the
% shape the plan had before rotation existed -- formationHoldSamples returns the
% identical single anchor -- so a still hold is bit-identical to what it always flew
% rather than merely close to it. That per-hold exactness is what makes a MIXED show
% trustworthy: the formations left alone are not approximately still, they emit the
% same one anchor they always did.
num_samples_per_hold = ones(1, num_formations);
num_samples_per_hold(rotation_active) = num_samples_per_seg;

% Show timeline computation (includes takeoff)
num_transitions = num_formations - 1;

% Floor on the climb, not the climb itself. takeoff_duration is finished off once the
% formations and the launch pads exist -- a formation 30 m tall needs longer than one
% at 10 m, and 5 s of it would be an unflyable command. See "Takeoff duration" below.
takeoff_duration = 5;  % seconds, minimum

% Every show ends with the fleet on the ground. The descent rate is what is set
% here; land_duration falls out of it once the formations are known, so raising
% show_altitude buys a proportionally longer landing instead of a faster one.
% 1.5 m/s is a sedate, real descent rate -- roughly 7 s from 10 m.
if ~exist('land_speed', 'var'), land_speed = 1.5; end   % m/s, average descent rate
if ~exist('land_settle', 'var'), land_settle = 2; end   % s parked on the ground

% And the same rate for the CLIMB, which is the parameter this pair was missing.
%
% The takeoff had no rate at all -- only the 5 s floor above and, from planShow, a
% ceiling that keeps it flyable. So the fastest speed the fleet can track was serving as
% the artistic choice, and a limit makes a poor default: measured on the default show the
% climb and the descent cover the same 10 m, the descent at a mean 1.5 m/s and the climb
% at 2.0 m/s, peaking 4.38 against 3.28. On a 30-drone text billboard it is worse -- the
% same 47.6 m flown in 14.9 s up and 31.6 s down, a peak of exactly 7.00 m/s because
% v_track_target is 7.0 and the sizing lands right on it. The fleet went up 1.33x to 2.13x
% faster than it came down, which is what "the drones take off too fast" was reporting.
%
% Set equal to land_speed rather than to a separately tuned number, because the argument
% for it is the symmetry: the show rises exactly as sedately as it descends, and the rate
% has already been justified once for the descent. Averaged along the flown PATH, same as
% land_speed -- so a drone crossing to its slot as well as climbing gets the time for the
% diagonal, not for the altitude alone.
if ~exist('climb_speed', 'var'), climb_speed = land_speed; end  % m/s, average climb rate

% What the fleet can actually track, and what the plan aims for when it sizes a
% segment itself. Both are measured numbers -- the evidence is with the speed check
% at the end of this script, which is where they used to be set; they are needed up
% here now because the takeoff is sized against them.
if ~exist('v_track_max', 'var'),    v_track_max = 8.0; end     % warn above this
if ~exist('v_track_target', 'var'), v_track_target = 7.0; end  % size the fix for this

% The same pair for ACCELERATION, and they are not redundant with the speed pair: a
% transition can sit well inside v_track_max and still demand more lateral acceleration
% than PositionController is allowed to command, because SatX/SatY clamp it at a_max.
% Sustained saturation is what breaks the fleet -- see the evidence table with the
% acceleration check in planShow. Deliberately below a_max (3.0) rather than equal to it:
% the clamp is per-axis and the position term Kp_pos*e draws on the same ceiling, so a
% plan sized right at a_max leaves the feedback loop nothing.
if ~exist('a_track_max', 'var'),    a_track_max = 2.5; end     % warn above this
if ~exist('a_track_target', 'var'), a_track_target = 2.0; end  % size the fix for this

% Peak-to-average speed of one minjerkpolytraj segment, which is what every segment in
% this plan is: a segment of length d over time T peaks at MINJERK_PEAK_RATIO * d/T,
% so inverting it gives the T that lands on a target speed.
%
% 35/16, not the 15/8 of a quintic. minjerkpolytraj brings jerk as well as velocity and
% acceleration to rest at both ends, which is a 7th-order profile and peaks higher.
% Measured, because the difference is not academic: sizing a 41.15 m climb with 15/8
% gave 11.02 s and the plan came out at 8.16 m/s, exactly 35/16 * 41.15/11.02 -- over
% the limit the sizing was aiming to stay under.
MINJERK_PEAK_RATIO = 35/16;

% The same peak for ACCELERATION, |s''| scaled by T^2. Measured off minjerkpolytraj at two
% different T to confirm it is a constant of the profile and not of the duration.
%
% This used to say it was "needed only by rotation", on the reasoning that a straight segment
% accelerates purely to change speed, so bounding its speed bounds its acceleration by
% implication. That reasoning is wrong, and the acceleration check in planShow exists because
% it is: the two bounds scale differently with the time allowed -- speed as 1/T, acceleration
% as 1/T^2 -- so halving a transition doubles the speed but quadruples the acceleration.
% Every straight transition therefore has a band of durations that are legal on speed and
% illegal on acceleration, and it widens as the fleet grows. Measured: 36 drones over
% Grid-Circle-Grid at 8 s transitions demands 6.97 m/s against the 8.0 m/s limit -- no
% warning -- while asking 2.98 m/s^2 of a 3.0 m/s^2 clamp, and one drone ends 127.66 m out.
% Rotation still needs this constant for the reason it always did: a circle demands a
% centripetal w^2*r for as long as it turns and no speed bound covers that at all.
MINJERK_ACCEL_PEAK_RATIO = 7.513188;

% Acceleration the plan may ask a rotating hold to sustain. NOT a_max: a_max is a check the
% plan is measured against afterwards, whereas this is a budget the plan is built inside,
% and the two must differ because the controller's own tracking error draws on the same
% ceiling. PositionController clamps commanded lateral acceleration at +/-a_max via
% SatX/SatY, and the position term Kp_pos*e adds to whatever the rotation asks for -- that
% term measures 1.50 m/s^2 on a plain show. 60% of a_max leaves the rest for it and for the
% attitude loop's own lag; whether the split is right is a question about the FLOWN result,
% so it has to be judged on a rotating show rather than on the plan.
rotation_accel_frac = 0.6;
rotation_accel_target = rotation_accel_frac * a_max;

% Lighting color sequence (RGB per formation TYPE, not per position in the
% sequence). Rows 1-4 are the built-in formations; 5 onward serve custom shapes --
% a loaded picture or STL, or a string registered by the app's "Fly Text" button --
% one row each. Eight rows was enough when a custom shape meant a file the operator
% had to go and find; typed text makes them cheap to add, and a show that spells
% four words used to run out and give the fifth the same colour as the fourth. The
% lookup clamps on the last row, so it is never out of range, only indistinct.
lighting_colors = [
    1.0, 0.0, 0.0;   % Formation 1: Red        (Grid)
    0.0, 1.0, 0.0;   % Formation 2: Green      (Circle)
    0.0, 0.0, 1.0;   % Formation 3: Blue       (Sphere)
    1.0, 1.0, 0.0;   % Formation 4: Yellow     (Text, from formation_text)
    1.0, 0.0, 1.0;   % Formation 5: Magenta    (1st custom shape)
    0.0, 1.0, 1.0;   % Formation 6: Cyan       (2nd custom shape)
    1.0, 0.5, 0.0;   % Formation 7: Orange     (3rd custom shape)
    1.0, 1.0, 1.0;   % Formation 8: White      (4th custom shape)
    0.6, 1.0, 0.0;   % Formation 9: Lime       (5th custom shape)
    1.0, 0.4, 0.7;   % Formation 10: Pink      (6th custom shape)
    0.5, 0.3, 1.0;   % Formation 11: Violet    (7th custom shape)
    0.0, 0.8, 0.6;   % Formation 12: Teal      (8th custom shape)
];

% An operator's colour choices, row per formation type, pushed by the app's Colour
% picker. A NaN row means "leave the default alone", which is what lets the app hold
% only the choices that were actually made instead of a second copy of the table
% above -- two copies of a show's colours is two answers to what it flew.
% The table is only overridden, never resized: a row past the end of the palette is a
% formation type that does not exist, and the lookup below clamps to the last row
% anyway.
if exist('lighting_colors_override', 'var') && ~isempty(lighting_colors_override)
    lc_ov = lighting_colors_override;
    if isnumeric(lc_ov) && size(lc_ov, 2) == 3
        lc_n = min(size(lc_ov, 1), size(lighting_colors, 1));
        lc_set = all(isfinite(lc_ov(1:lc_n, :)), 2);
        lighting_colors(lc_set, :) = min(1, max(0, lc_ov(lc_set, :)));
    else
        warning('setupParams:badColorOverride', ...
            ['lighting_colors_override must be an N-by-3 numeric table; got %s. ' ...
             'Ignoring it and flying the default palette.'], ...
            class(lc_ov));
    end
    clear lc_ov lc_n lc_set
end

%% Mission Plan
% Everything from here to the trajectory-source switch below used to be inline: ~970
% lines that built the formation geometry, assigned drones to points, generated the
% min-jerk trajectory, and then packetised it for upload. Two things were wrong with
% that, and both are about the INTERFACE rather than the algorithm.
%
% First, every intermediate it computed became a public base-workspace variable. There
% were 96 of them -- cost_matrix, hold_pos, q, qd, idx, span, why -- sitting in base
% next to the 38 results something actually reads, with nothing to distinguish the two
% groups. A caller could not tell what the contract was, and neither could we: that is
% how four dead timeseries objects and 27 unread parameters went unnoticed.
%
% Second, nothing could be run on its own. Re-planning a show meant re-running the
% whole script -- reference location, RTK configuration, link sizing and all.
%
% So the two halves are functions now, and which names had to survive the move is
% DERIVED rather than chosen. Every consumer was scanned -- the model via
% Simulink.findVars, the app's evalin reads, DroneShowExample -- for names that are READ
% WITHOUT FIRST BEING BOUND. Those names are unpacked back into base below, so the
% base-workspace contract is unchanged and anything that seeds or reads it by individual
% name still works.
%
% Two independent checks stand behind that claim, because neither is sufficient alone:
% the moved regions were compared TEXTUALLY against what stood here, line for line with
% every difference declared -- which is what catches a typo on a branch no configuration
% happens to reach -- and the resulting base workspace was compared for VALUE-identity
% across five configurations, which is what catches a wiring mistake that a text
% comparison cannot see.

% cf_margin's default lives here rather than in planShow because a function cannot ask
% its caller what is defined, and this is an operator-settable base variable. The
% derivation of 1.6 stays in planShow, next to the scaling it governs and next to the
% squared-cost bound it depends on.
%
% This is the one place the move changed WHEN a variable exists, so it is worth naming.
% The guard used to sit INSIDE the `if ~isempty(cloud_form)` block, which runs only for a
% formation built from a point cloud -- typed text, or a picture or STL. So a Grid, Circle
% and Sphere show never created cf_margin at all, which is why it reads as a NEW variable
% in four of the five stock configurations. It is now unconditional. Its VALUE is
% unchanged wherever it existed before, nothing reads it
% except planShow, and a parameter that appears only if the show happens to contain text
% is not something an operator could reasonably set.
if ~exist('cf_margin', 'var'), cf_margin = 1.6; end

% One struct in, built by assignment rather than struct(), which is not a style choice:
% struct('f', v) EXPANDS a struct-array or cell v into a struct ARRAY, one element per
% entry, so passing custom_formations through struct() would silently hand planShow a
% 1xN show_cfg instead of one configuration. Field names are the parameter names on
% purpose -- both functions unpack them straight back into same-named locals, which is
% what let their bodies move verbatim.
show_cfg = struct();
show_cfg.N_uav                    = N_uav;
show_cfg.num_formations           = num_formations;
show_cfg.num_transitions          = num_transitions;
show_cfg.formation_sequence       = formation_sequence;
show_cfg.formation_spacing        = formation_spacing;
show_cfg.formation_radius_max     = formation_radius_max;
show_cfg.show_altitude            = show_altitude;
show_cfg.formation_text           = formation_text;
show_cfg.cf_margin                = cf_margin;
show_cfg.d_min                    = d_min;
show_cfg.a_max                    = a_max;
show_cfg.takeoff_duration         = takeoff_duration;
show_cfg.hold_duration            = hold_duration;
show_cfg.transition_duration      = transition_duration;
show_cfg.climb_speed              = climb_speed;
show_cfg.land_speed               = land_speed;
show_cfg.land_settle              = land_settle;
show_cfg.num_samples_per_seg      = num_samples_per_seg;
show_cfg.num_samples_per_hold     = num_samples_per_hold;
show_cfg.MINJERK_PEAK_RATIO       = MINJERK_PEAK_RATIO;
show_cfg.MINJERK_ACCEL_PEAK_RATIO = MINJERK_ACCEL_PEAK_RATIO;
show_cfg.formation_rotate         = formation_rotate;
show_cfg.rotation_active          = rotation_active;
show_cfg.rotation_deg_request     = rotation_deg_request;
show_cfg.rotation_accel_target    = rotation_accel_target;
show_cfg.rotation_accel_frac      = rotation_accel_frac;
show_cfg.v_track_target           = v_track_target;
show_cfg.v_track_max              = v_track_max;
show_cfg.a_track_target           = a_track_target;
show_cfg.a_track_max              = a_track_max;
show_cfg.lighting_colors          = lighting_colors;
show_cfg.upload_waypoint_rate     = upload_waypoint_rate;
show_cfg.upload_keyframes_max     = upload_keyframes_max;
show_cfg.upload_keyframes_arc_max = upload_keyframes_arc_max;
show_cfg.upload_arc_sag_frac      = upload_arc_sag_frac;
show_cfg.mavlink_bandwidth        = mavlink_bandwidth;
show_cfg.mavlink_packet_size      = mavlink_packet_size;
show_cfg.Ts_sim                   = Ts_sim;

% Absent means "no custom shapes", passed as [] rather than left undefined: the isstruct
% test in planShow then carries the whole question, which is what a function can check.
if exist('custom_formations', 'var')
    show_cfg.custom_formations = custom_formations;
else
    show_cfg.custom_formations = [];
end

plan = planShow(show_cfg);
pack = packShowUpload(show_cfg, plan);

% Unpack into base under the original names. Spelled out one per line rather than
% looped over fieldnames, and that matters for more than readability: a loop would
% assign through eval or dynamic fields, and then NO static tool could see these names.
% Simulink.findVars, the export-set scan, and a plain grep for "is this parameter wired"
% would all report them absent. The interface has to be greppable to stay verifiable.
all_formations       = plan.all_formations;
assignment_order     = plan.assignment_order;
init_positions       = plan.init_positions;
land_duration        = plan.land_duration;
land_end_time        = plan.land_end_time;
land_ground_pos      = plan.land_ground_pos;
land_start_time      = plan.land_start_time;
land_total_duration  = plan.land_total_duration;
light_ws_data        = plan.light_ws_data;
lighting_timeline    = plan.lighting_timeline;
max_samples          = plan.max_samples;
min_sep_achieved     = plan.min_sep_achieved;
num_total_samples    = plan.num_total_samples;
rotation_limit       = plan.rotation_limit;
rotation_min_hold    = plan.rotation_min_hold;
rotation_need_hold   = plan.rotation_need_hold;
rotation_peak_accel  = plan.rotation_peak_accel;
rotation_peak_speed  = plan.rotation_peak_speed;
rotation_radius      = plan.rotation_radius;
rotation_sweep       = plan.rotation_sweep;
show_duration        = plan.show_duration;
show_flight_duration = plan.show_flight_duration;
takeoff_dist         = plan.takeoff_dist;
takeoff_duration     = plan.takeoff_duration;   % raised by planShow to fit the climb
time_vector          = plan.time_vector;
timeline_times       = plan.timeline_times;
timeline_types       = plan.timeline_types;
traj_min_transition  = plan.traj_min_transition;
traj_peak_speed      = plan.traj_peak_speed;
traj_peak_accel      = plan.traj_peak_accel;
trajectory_data      = plan.trajectory_data;

num_waypoints        = pack.num_waypoints;
total_packets        = pack.total_packets;
traj_upload          = pack.traj_upload;
traj_upload_flat     = pack.traj_upload_flat;
upload_duration      = pack.upload_duration;
upload_rate_vec      = pack.upload_rate_vec;
upload_times         = pack.upload_times;

% radius exists only when the show contains a Circle or a Sphere -- they are the only
% branches that assign it, and it was absent from base for a Grid-only show before this
% refactor too. Kept conditional rather than zero-filled: two probes read it back to
% report ring geometry, and a fabricated 0 would read as a measurement of a formation
% that has no radius.
if isfield(plan, 'radius')
    radius = plan.radius;
end
%% Trajectory Source
% Which path the waypoints take to reach the drones. The data is the same either
% way — traj_show_ts is built from traj_upload, the very keyframes the BaseStation
% streams — so a healthy link produces an identical show. What differs is whether
% the radio is in the loop.
%
%   1 (TRAJ_WORKSPACE) : From Workspace block feeds the position controller
%                        directly. No MAVLink traffic, so the show can be
%                        iterated on quickly.
%   2 (TRAJ_ONBOARD)   : the drones fly what actually arrived, read back from the
%                        OnboardBuffer data store. Requires the pre-flight upload
%                        to run in the SAME simulation, because data store
%                        contents do not survive across sim() calls.
if ~exist('traj_source', 'var'), traj_source = 2; end

% Start the show straight after GPS lock, skipping the upload and arm phases.
% Only coherent with traj_source == 1: variant 2 needs the upload to fill the
% buffer. The GPS lock is deliberately kept so the INS still converges.
if ~exist('skip_preflight', 'var'), skip_preflight = false; end
if skip_preflight && traj_source == 2
    warning(['skip_preflight ignored: traj_source 2 flies from the OnboardBuffer, ' ...
             'which only the pre-flight upload can fill.']);
    skip_preflight = false;
end

%% Fast mode assumes a perfect correction link
% skip_preflight is the "no MAVLink" mode: no uplink, no downlink, no waiting for the base station.
% The GNSS degradation chain does NOT know that -- it keeps running, so this mode used to fly the
% first 11 s of its show through a metre-class GNSS transient: nav error up to 4.39 m against a 5 m
% formation spacing, visibly wandering and then settling. With nothing being communicated there is no
% correction latency to model, so the mode should simply assume the link is up and fixed.
%
% WHERE THE TRANSIENT COMES FROM. Measured on the live signals, RadioChannel/CorrectionAgeMonitor is
%
%   PosSigma = max(agePath, engPath)                        (WorseOf)
%   agePath  = rateLimit(SigmaLUT(corrAge) * gnssAvail)     (the rtk_sigma_table staircase)
%   engPath  = sqrt(max(0, EngineSigma^2 - insFloor^2)) * gnssAvail
%
% and the two terms are nothing like equal partners:
%   - agePath is 0.0000 m for an undegraded drone at the SHIPPED rtk_timeout of 2.50 s, and that is
%     now a property of the margin rather than of the median. corrAge is usually fresh -- median
%     0.440 s, 95th percentile 0.960 s -- but the TAIL reaches 1.990 s, because packet_loss_rate
%     erases an RTCM frame at the TRANSMITTER, which is fleet-wide and perfectly correlated. 1.990
%     is under 2.50, so the LUT stays on breakpoint 1 and one lost frame costs nothing; it takes two
%     CONSECUTIVE losses to cross. Measured, no deny mask: ZERO fix dropouts in a default run.
%     At the OLD 1.5 s timeout the same tail crossed on every single loss, and because the loss is
%     transmitter-side the whole fleet went to Float at 0.299 m together -- TWO episodes per default
%     run (t = 18.50-19.00 and t = 66.50-67.00, each 0.50 s, all 10 drones), against an EXPECTED 0.6
%     over the run's 62 broadcasts: the shipped loss_seed simply draws on the unlucky side, and one
%     of the two draws lands mid-climb. The model was right; the margin was too tight to watch.
%     Two earlier versions of this comment were wrong in opposite directions -- one claimed
%     "0.0000 m for the WHOLE run" reasoning from the median alone, the next documented the twice-
%     per-run dropout as intended behaviour. It is intended; it was just not intended to land
%     unannounced on the whole fleet in mid-climb. See rtk_timeout above for why 2.5 s.
%   - engPath is the entire transient. EngineSigma is not a constant: it is RTKEngine outport 2, the
%     engine's OWN live sigma, carried in via SigmaToBaseRate. It reads 0.17-1.60 m while Status == 3
%     (code-only, ambiguity not yet resolved) and collapses to ~0.005 m the instant Status == 1 at
%     t = 13 s. The 1.6009 m peak is exactly sqrt(1.6010^2 - 0.02^2) of the raw engine sigma one slow
%     step earlier -- the RateTransition lag, not a modelling error.
%
% So rtk_sigma_table is NOT the lever, and an earlier attempt to zero it here was a measured no-op:
% posSigma stayed at 1.6009 m to four decimals because the term being zeroed was already zero. The
% only lever on this transient is the engine term, and it is a live signal rather than a parameter --
% which is why gating it needs the InjectGate block in CorrectionAgeMonitor rather than a constant.
%
% rtk_inject_enable is that gate, and it is PER DRONE. PosSigma is multiplied by it element-wise, so a
% 0 removes ALL injected degradation for that drone and leaves gps_accuracy_rtk_fix -- exactly the 2 cm
% precision the INS block provides on its own, from t = 0, with no tier to converge through. The INS
% block itself is untouched -- nothing here changes a single INS parameter.
%
% WHY PER DRONE rather than per mode. Fast mode has no base station to be faithful to, so the engine's
% convergence transient is fidelity it is entitled to skip. But the operator's Degrade button is not
% fidelity -- it is a fault the operator explicitly asked for -- and gating it off along with everything
% else made that button a silent no-op in the mode most likely to be running. Opening the gate for
% exactly the denied drones gives both halves: an undegraded drone is bit-identical to the fully gated
% version, and a denied one drifts out to the Standalone tier the way the panel promises.
%
% This separates cleanly only because the two terms behind WorseOf do, as measured live:
%   age path    - 0.0000 m for an undegraded drone in BOTH modes, because SigmaLUT sits on
%                 breakpoint 1: corrAge peaks at 1.990 s when packet_loss_rate erases a broadcast,
%                 and rtk_timeout is 2.50 s, so a single lost correction does not reach the first
%                 breakpoint at all. Measured: zero dropouts in a default run. Two CONSECUTIVE
%                 losses would cross it, at 1e-4 per pair. rtk_deny_mask is therefore what drives
%                 this term in practice, and the only thing that drives it PERMANENTLY: deny a
%                 drone, it gets no corrections at all, and its corrAge runs past the timeout and
%                 stays there, through Float, out to Standalone.
%   engine path - the entire 13 s convergence transient, peaking at 1.6009 m, and nothing whatever to
%                 do with degradation.
% So a denied drone in fast mode sees the engine term too, through the same open gate. That is accepted
% rather than worked around: max(1.6009, tier) differs from the 1.5 m Standalone tier it is heading to
% by under 0.11 m, on a drone the operator has just broken on purpose. Splitting the gate in two to
% chase that 0.11 m would mean restructuring CorrectionAgeMonitor for no observable gain.
%
% Orientation matters. WorseOf emits a 1-D N_uav-wide signal and InjectEnable has VectorParams1D on, so
% this value must be a ROW to multiply element-wise. At N_uav = 1 it collapses to a scalar, which is
% what this gate always was -- the Constant-collapses-at-1 trap does not bite here for that reason.
%
% Placed after the traj_source == 2 override above, because that can force skip_preflight back to
% false and it is the EFFECTIVE mode that has to decide this. The model reads this when it compiles,
% after this script returns, so assigning it here keeps the reason next to the mode switch.
rtk_inject_enable = double(~skip_preflight | rtk_deny_mask(:)');
if skip_preflight
    if any(rtk_deny_mask)
        fprintf(['Fast mode: injected GNSS sigma gated off, so the fleet flies the %.2f m INS floor ' ...
                 'from t=0 -- except UAV %s, degraded on request.\n'], ...
                gps_accuracy_rtk_fix, mat2str(find(rtk_deny_mask(:)')));
    else
        fprintf(['Fast mode: injected GNSS sigma gated off, so the fleet flies the ' ...
                 '%.2f m INS floor from t=0.\n'], gps_accuracy_rtk_fix);
    end
end

% Same mode switch, different cost: the drone-side TELEMETRY serialiser. Every other MAVLink chain in
% the model already carries an enable -- the BaseStation transmitters on their cascade index, the two
% onboard mission encoders on their protocol state, the three deserializers on a message-ID match --
% and DroneFleet/Transmitter/TelemetryEncoder was the one that did not. It built and serialised a
% TELEMETRY message on EVERY tick in EVERY mode, including quick mode, where nothing consumes the
% result: the received table reaches only LogFleetState and the app's live poll, and quick mode draws
% the direct pose instead (see the '[Live - direct]' title string in DroneLightShowApp/runShowPolled).
%
% So this is the drone-side half of "speed on the drone, fidelity on the base station". Gated OFF in
% quick mode and ON in full fidelity, which is the mode whose entire purpose is to fly the real codec.
% Held rather than reset when disabled, matching AckMsgGate: the sequence counter inside TelCounter
% keeps its value across the gate rather than restarting, so a mode switch cannot fake a lost packet.
%
% NOT a fidelity lever for the base station. The base's own receive chain -- Deser_Telemetry in
% BaseStation/Receiver -- is untouched and still ungated; it just decodes a frozen buffer in quick
% mode. Gating that half needs a subsystem wrap, so it is left as a separate, measured decision.
telemetry_tx_enable = double(~skip_preflight);

% Total simulation duration (pre-flight + show). The landing is INSIDE show_duration
% now, so it is not added again here -- land_duration used to be a bare 3 s of
% padding tacked on the end, which bought the fleet three extra seconds of hovering
% because nothing in the model ever read it (Simulink.findVars: not used).
% The RTK launch gate (ARMED -> SHOW) holds the launch until the base engine reports L1 fixed,
% which takes ~13 s of wide-lane convergence plus rtk_lock_dwell. StopTime has to be chosen
% BEFORE the run, so the pre-show budget needs an allowance for a wait whose exact length is
% seed-dependent. That is all rtk_lock_budget is: a SIZING allowance so the run does not end
% mid-show. It is not a gate and nothing in the model reads it -- if the fix arrives sooner the
% show simply starts sooner, and the only cost is a little slack at the end.
%
% Only on the full-communication path. skip_preflight goes IDLE -> SHOW and never enters ARMED,
% so the gate cannot fire there and the budget must not be charged.
%
% TWO DIFFERENT JOBS, AND THEY NEED TWO DIFFERENT NUMBERS. Raising this used to break the show,
% because one expression was doing both jobs at once:
%
%   BOUND  -- StopTime must be long enough that the run cannot end mid-show, whatever the seed
%             draws. This wants the WORST case, and paying for it costs only an idle tail.
%   ANCHOR -- the instant show-relative time 0 happens, used to place traj_show_ts on the absolute
%             time axis and to build the app's time axis. This wants the ACTUAL launch instant, and
%             getting it wrong is a visible defect on every run.
%
% When preflight_duration served as both, raising the budget moved the anchor too. Measured
% with the budget at 28:
%   chart entered SHOW at 16.00 s      (L1 fix 13.00 + dwell 2.0, seen one slow step later)
%   trajectory anchored at 28.00 s     (best-fit anchor 28.00, RMS 0.098 m, against 7.04 m at 16.00)
%   LANDING fired at 39.01 s           = SHOW entry + land_start_time, NOT preShowDelay + it
% i.e. the fleet spent 12 s in SHOW being commanded to the pre-show ground anchor -- visibly moving
% but not in formation -- then snapped into the show, and was ordered down 12 s before the
% trajectory ended. That is a truncated show, not slack at the end.
%
% The onboard path no longer has an anchor to get wrong: OnboardTrajSource is clocked by
% ShowSupervisor's ShowTime outport, which the chart starts on SHOW ENTRY -- i.e. when the RTK gate
% actually opens. Reception drives it, so no constant here can desync it and the budget is free to
% be a pure bound.
%
% The workspace path (traj_source 1) cannot be re-clocked -- a From Workspace block reads simulation
% time, not a signal -- so it still needs a PREDICTED launch instant, which is what rtk_lock_expected
% is. That prediction is exact in fast mode (IDLE -> SHOW on the GPS lock timer, no gate) and is a
% measured typical value otherwise, which is why the combination below is flagged.
%
% The seed sensitivity is real: 2-23 epochs to first fix over 200 trials (median 5, 90th pct 10).
% That is exactly what a bound is for, and 28 covers the tail of it.
rtk_lock_budget   = 28;   % s, BOUND on the gate wait. Only reaches StopTime -- never the anchor.
rtk_lock_expected = 16;   % s, measured gate time (L1 fix 13 + dwell 2, seen one slow step later)

if skip_preflight
    % No gate on this path: the chart leaves IDLE for SHOW on the GPS lock timer alone, so the
    % prediction is not a prediction at all and bound == anchor exactly.
    preflight_duration = gps_lock_duration;
    preShowDelay       = gps_lock_duration;
    total_sim_duration = preflight_duration + show_duration;
    fprintf('Total sim duration: %.1f s (lock %.0f + show %.1f incl. %.1f s landing, pre-flight skipped)\n', ...
        total_sim_duration, gps_lock_duration, show_duration, land_duration + land_settle);
else
    % max(), not +: the upload and arm run WHILE the base converges, exactly as a real operator
    % uploads a mission during survey-in. Adding them would bill for a wait that overlaps.
    preflight_nominal  = gps_lock_duration + upload_duration + arm_duration;
    preflight_duration = max(preflight_nominal, rtk_lock_budget);      % bound  -> StopTime
    preShowDelay       = max(preflight_nominal, rtk_lock_expected);    % anchor -> traj_show_ts, app
    total_sim_duration = preflight_duration + show_duration;
    fprintf(['Total sim duration: %.1f s (pre-flight bound %.1f = max(lock %.0f + upload %.1f + arm %.0f, ' ...
             'rtk budget %.0f) + show %.1f incl. %.1f s landing; show anchor %.1f s)\n'], ...
        total_sim_duration, preflight_duration, gps_lock_duration, upload_duration, ...
        arm_duration, rtk_lock_budget, show_duration, land_duration + land_settle, preShowDelay);
    if traj_source == 1
        % Flagged rather than forbidden: it is a useful combination when the interest is the comms
        % traffic rather than the flown geometry. But the workspace timeseries is anchored at a
        % PREDICTION here while the launch waits for the gate, so anything measuring flown-vs-plan
        % on this combination is measuring the prediction error too.
        fprintf(['  NOTE: traj_source 1 with a full pre-flight anchors the workspace trajectory at ' ...
                 'the predicted\n        launch (%.1f s) while the chart waits for the RTK gate. ' ...
                 'Use traj_source 2 to be reception-driven.\n'], preShowDelay);
    end
end

%% Initial state vector for Multi-Instance Guidance Model
% 13 states per UAV: [x,y,z, vx,vy,vz, phi,theta,psi, p,q,r, thrust]
initialState = zeros(13 * N_uav, 1);
for k = 1:N_uav
    offset = (k-1)*13;
    initialState(offset+1:offset+3) = init_positions(k,:)';  % position
    initialState(offset+13) = uav_mass * gravity;            % hover thrust
end

%% DroneFleet Onboard Trajectory Timeseries (used by From Workspace during show)
% Drone-major flat row: [d1_pos(3), d1_vel(3), d1_yaw(1), d2_pos(3), ...]
% Time-shifted by pre-show delay so a From Workspace block indexed by sim time
% produces ground-hold before the show, then the trajectory. The delay has to
% track what ShowSupervisor actually does: with skip_preflight the chart leaves
% IDLE for SHOW as soon as the GPS lock expires, so upload and arm are not in it.
%
% preShowDelay is set above, next to the bound it is deliberately NOT equal to. It used to be
% assigned here as preflight_duration, which is what coupled the anchor to the StopTime bound and
% made rtk_lock_budget unraisable -- see the two-jobs comment there. The app reads preShowDelay to
% build its playback time axis in BOTH modes, so if this and the chart disagree, every formation
% label and every live marker shifts by the difference -- a whole-show timing defect that looks
% like a labelling bug.
assert(exist('preShowDelay', 'var') == 1, 'preShowDelay must be set with the pre-flight timeline');
traj_show_flat = zeros(num_waypoints, N_uav*7);
for w = 1:num_waypoints
    row = zeros(1, N_uav*7);
    for k = 1:N_uav
        row((k-1)*7 + 1 : k*7) = squeeze(traj_upload(k, w, :))';
    end
    traj_show_flat(w, :) = row;
end
% Pre-show anchor row: all drones at ground init positions (vel=0, yaw=0)
preshow_row = zeros(1, N_uav*7);
for k = 1:N_uav
    preshow_row((k-1)*7 + 1 : (k-1)*7 + 3) = init_positions(k,:);
end
traj_show_full = [preshow_row; preshow_row; traj_show_flat];
traj_show_times = [0; max(preShowDelay - 1e-3, 1e-6); upload_times + preShowDelay];
traj_show_ts = timeseries(traj_show_full, traj_show_times);
traj_show_ts.Name = 'ShowTrajectory';

%% Clean up loop scratch
% This script runs with `run`, so its LOOP VARIABLES land in the caller's workspace -- the base
% workspace when the app or a PreLoadFcn calls it, and a test's own workspace otherwise. They are
% not parameters and nothing outside this script reads them, but leaving them behind has already
% cost real debugging time: a leftover `tr` overwrote a test's containers.Map mid-run and surfaced
% as an unrelated "key type" error, and a leftover `lines` shadowed the built-in.
%
% Exactly the 13 names used as `for <name> =` above, and nothing else. Verified safe before
% clearing, because "unused" had to be proven rather than assumed:
%   * no block parameter expression in the model references any of them (checked across all 1375
%     blocks, every evaluated dialog field -- 0 hits),
%   * the app reads only NAMED parameters out of base via evalin, never a single-letter name.
% Deliberately NOT cleared: the derived values that happen to be loop-assigned (cx, cy, perm,
% sample_idx, ...). Several are read further down this script, and the ones that are not are
% parameters to decide about, not scratch to hide.
clear i j k f s w tr uav_i pad_i k_drone k_seg k_wp t_park


