function pack = packShowUpload(cfg, plan)
%packShowUpload  Downsample a flown plan into the waypoints that go over the link.
%
%   pack = packShowUpload(cfg, plan) turns the dense trajectory planShow produced into
%   the sparse keyframe set a drone is actually sent during pre-flight, and sizes how
%   long sending it takes. The drone interpolates between keyframes onboard, so this is
%   a lossy compression with a measured error budget -- see the keyframing note in the
%   body for the numbers.
%
%   TWO ARGUMENTS, and the split is not cosmetic. cfg carries the constants (link
%   sizing, keyframe caps, sample rate); plan carries what planShow computed. Several
%   names appear on both sides of that line in the ORIGINAL script -- most importantly
%   takeoff_duration, which enters planShow as a floor and leaves it raised to whatever
%   the climb needs. Reading it from cfg here would size the upload against a takeoff
%   the fleet is not flying, and on a tall text formation that is a 6 s error. So
%   anything planShow writes is taken from plan, never from cfg, even where cfg also
%   has the name.
%
%   OUTPUT pack, with the seven fields the model and the app read back:
%          num_waypoints total_packets traj_upload traj_upload_flat upload_duration
%          upload_rate_vec upload_times
%          DERIVED rather than chosen: every consumer was scanned for names it reads
%          without first binding them, and these are what came back. Note `samples` is
%          NOT among them despite looking like a plausible export -- its only other
%          appearance is a printf column header, so it is the interp1 scratch variable
%          it looks like and stays local.
%
%   As in planShow, cfg and plan are unpacked into locals under their original names so
%   the region below stays textually identical to the script it came from.
%
%   See also setupParams, planShow.

% ---- unpack -----------------------------------------------------------------------
% Constants
N_uav                     = cfg.N_uav;
num_formations            = cfg.num_formations;
hold_duration             = cfg.hold_duration;
transition_duration       = cfg.transition_duration;
rotation_active           = cfg.rotation_active;
MINJERK_PEAK_RATIO        = cfg.MINJERK_PEAK_RATIO;
upload_waypoint_rate      = cfg.upload_waypoint_rate;
upload_keyframes_max      = cfg.upload_keyframes_max;
upload_keyframes_arc_max  = cfg.upload_keyframes_arc_max;
upload_arc_sag_frac       = cfg.upload_arc_sag_frac;
mavlink_bandwidth         = cfg.mavlink_bandwidth;
mavlink_packet_size       = cfg.mavlink_packet_size;
Ts_sim                    = cfg.Ts_sim;

% From the plan. takeoff_duration especially -- see the note above.
takeoff_duration      = plan.takeoff_duration;
show_flight_duration  = plan.show_flight_duration;
show_duration         = plan.show_duration;
land_end_time         = plan.land_end_time;
timeline_times        = plan.timeline_times;
timeline_types        = plan.timeline_types;
rotation_radius       = plan.rotation_radius;
rotation_sweep        = plan.rotation_sweep;
time_vector           = plan.time_vector;
trajectory_data       = plan.trajectory_data;

%% Downsampled Trajectory for Pre-Flight Upload
% Models what actually gets sent over MAVLink: sparser waypoints that the
% drone interpolates onboard during the show.
%
% Keyframes are placed per SEGMENT rather than at a fixed rate across the whole
% show, because a uniform grid spends its budget in the wrong places. It gave a
% 5 s hold -- where nothing moves -- the same 11 points as a 5 s takeoff, and it
% made the item count grow linearly with show duration, which is exactly what the
% auto-fitted transition does to big fleets. Per segment:
%
%   moving  : min(upload_keyframes_max, 2 Hz worth), so short segments keep their
%             current density and long ones stop growing
%   holding : 2 anchors. The fleet is stationary, so two points reproduce a hold
%             EXACTLY under linear interpolation -- this is free, not a trade.
%
% Measured by reconstructing the uploaded keyframes the way the drone does and comparing
% the two grids on what the fleet actually flies:
%
%   default 10-UAV show (15 s transitions) : 248 wp -> 90, upload 29.9 s -> 10.9 s
%   app default,  8 UAV  (8 s transitions) : 142 wp -> 90, upload 13.7 s ->  8.7 s
%   200-UAV, 49 s transitions              : 677 wp -> 60, upload 1627 s -> 146 s
%
% Minimum separation comes out IDENTICAL in all three (3.507, 4.103, 1.122 m) and
% peak speed is equal or marginally LOWER (15.544 -> 15.472 m/s on the third),
% because a chord's slope is the average of the quintic's speed over that leg and
% so never exceeds its peak. Deviation from the dense plan is unchanged at 0.093 m
% on the first two -- it lives in the takeoff climb, not the transitions -- and
% rises to 1.37 m only on the 49 s legs, which are hundreds of metres long.
% What is genuinely lost is acceleration continuity at the keyframes.
upload_seg_span  = hold_duration * (timeline_types == 0) + ...
                   transition_duration * (timeline_types == 1);
upload_seg_start = [0, timeline_times, show_flight_duration, land_end_time];
upload_seg_stop  = [takeoff_duration, timeline_times + upload_seg_span, ...
                    land_end_time, show_duration];
% A ROTATING hold is a moving segment. The "2 anchors reproduce a hold exactly"
% argument above rests on the fleet being stationary, and a spinning formation is
% not -- two anchors would upload the start and end orientations and have the drones
% interpolate along the CHORD between them, which for a half turn means flying
% through the middle of the formation. So holds join the moving set when they spin,
% and the uploaded item count (and therefore upload_duration) rises with them.
%
% Per-hold, not fleet-wide: a still hold in a show that rotates elsewhere is still two
% anchors, and giving it a moving segment's keyframe budget would upload dozens of
% identical points. timeline entry 2f-1 is hold f (see the loop that builds
% timeline_types), so the request is scattered onto the odd entries.
timeline_rotates = false(1, numel(timeline_types));
timeline_rotates(1:2:end) = rotation_active;
upload_seg_moves = [true, timeline_types == 1 | timeline_rotates, true, false];

% Keyframes per segment. A rotating hold is sized by ANGLE, not by the clock: what has
% to be resolved is the arc. A step of dtheta uploads as its chord, which sags
% r*(1-cos(dtheta/2)) INWARD of the true circle, so bounding the sag by
% upload_arc_sag_frac bounds dtheta -- and independently of r, which is the whole point
% of making the tolerance a fraction. The count is sized against the LARGEST step,
% MINJERK_PEAK_RATIO times the mean, because the angle profile does its fastest sweeping
% in the middle of the hold and that is exactly where a uniform time grid sags most.
%
% Sizing rotating holds by time got this wrong, and the error was not small. The 16-point
% cap on a 10 s hold left the widest step at 0.34 rad, which sagged 0.28 m at r = 9.5 m --
% twenty times what a by-the-clock estimate predicted, because that estimate used the mean
% step rather than the peak and a gentler sweep than the fleet actually flies. It was found
% by reconstructing the uploaded keyframes the way a drone does and measuring the sag,
% not by reading this code.
upload_seg_kf = zeros(1, numel(upload_seg_start));
for k_seg = 1:numel(upload_seg_start)
    span = upload_seg_stop(k_seg) - upload_seg_start(k_seg);
    % Segment k_seg = 1+j carries timeline entry j, and hold f is entry 2f-1, so the
    % holds are the EVEN segment indices from 2 to 2*num_formations. The landing sits at
    % 2*num_formations+1 and is odd, so it is excluded without a special case.
    is_hold = mod(k_seg, 2) == 0 && k_seg <= 2 * num_formations;
    % rotation_active(f), not the fleet-wide flag: the arc branch has to be taken for the
    % holds that spin and NOT for the ones left still. rotation_radius(f) cannot stand in
    % for it -- it is the formation's geometric extent and is nonzero for a still hold too,
    % so keying on it alone would size a stationary hold by an arc it never sweeps.
    if is_hold && rotation_active(k_seg / 2)
        f = k_seg / 2;
        if rotation_radius(f) > 1e-9
            dtheta_max = 2 * acos(1 - upload_arc_sag_frac);
            n_arc = ceil(MINJERK_PEAK_RATIO * abs(rotation_sweep(f)) / dtheta_max) + 1;
        else
            n_arc = 2;      % nothing off the axis, so there is no arc to resolve
        end
        upload_seg_kf(k_seg) = min(upload_keyframes_arc_max, n_arc);
    elseif upload_seg_moves(k_seg)
        upload_seg_kf(k_seg) = min(upload_keyframes_max, ...
                                   floor(span * upload_waypoint_rate) + 1);
    else
        upload_seg_kf(k_seg) = 2;
    end
end

upload_times = 0;
for k_seg = 1:numel(upload_seg_start)
    t0 = upload_seg_start(k_seg);
    t1 = upload_seg_stop(k_seg);
    if t1 <= t0
        continue;   % a zero-length landing or settle window contributes nothing
    end
    upload_times = [upload_times, ...
                    linspace(t0, t1, max(upload_seg_kf(k_seg), 2))]; %#ok<AGROW>
end
% Segment ends coincide with the next segment's start, so dedupe. Rounding first
% keeps linspace's floating-point endpoints from surviving as near-duplicates a
% few nanoseconds apart, which would put a division by ~0 in upload_rate_vec.
upload_times = unique(round(upload_times(:), 6));
num_waypoints = length(upload_times);

% Per-interval 1/dt for the onboard velocity feedforward. OnboardTrajSource used to
% multiply the waypoint delta by upload_waypoint_rate, which was only Delta_p/Delta_t
% while the grid was uniform; with segment-aware keyframing the interval length
% varies, so the drone looks the reciprocal up by the same PreLookup index it uses
% for position. The last entry repeats the previous one: PreLookup clips its index
% at num_waypoints-1, so the final slot is never read, but leaving it as 1/0 would
% put an Inf in a Constant block.
upload_dt = diff(upload_times);
upload_rate_vec = [1 ./ upload_dt; 1 / upload_dt(end)];
% Deduplicate time_vector for interp1 (segment endpoints overlap with hold anchors)
[t_unique, i_unique] = unique(time_vector, 'stable');
traj_upload = zeros(N_uav, num_waypoints, 7);
for k = 1:N_uav
    for j = 1:7
        samples = squeeze(trajectory_data(k, j, i_unique));
        traj_upload(k, :, j) = interp1(t_unique, samples, upload_times, 'linear', 'extrap');
    end
end

%% Flatten trajectory for BaseStation waypoint lookup
% traj_upload has shape [N_uav x num_waypoints x 7]. Flatten to
% [N_uav*num_waypoints x 7] with linear_idx = drone_idx*num_waypoints + wp_idx
% (drone-major: waypoints 0..num_waypoints-1 for drone 0, then drone 1, etc.)
traj_upload_flat = zeros(N_uav*num_waypoints, 7);
for k_drone = 1:N_uav
    for k_wp = 1:num_waypoints
        lin = (k_drone-1)*num_waypoints + k_wp;
        traj_upload_flat(lin, :) = squeeze(traj_upload(k_drone, k_wp, :))';
    end
end

%% Upload Timing
% UploadIndexerChart emits one MISSION_ITEM_INT per Ts_sim tick.
% Bandwidth is validated as a sanity check but does not increase throughput: the
% tick sets the rate at packets_per_tick*mavlink_packet_size/Ts_sim = 5000 B/s,
% which is the link this model actually represents. See the note by
% mavlink_bandwidth for why that is a telemetry radio, not a show network.
packets_per_tick = 1;
bandwidth_capacity = floor(mavlink_bandwidth * Ts_sim / mavlink_packet_size);
if bandwidth_capacity < packets_per_tick
    warning('Bandwidth (%d B/s) too low for 1 packet/tick at Ts_sim=%g', ...
        mavlink_bandwidth, Ts_sim);
end
total_packets = N_uav * num_waypoints;
% Add MC handshake (N_uav ticks) + finalization margin
upload_duration = (total_packets + N_uav) * Ts_sim * 1.2;

fprintf('Upload phase: %d packets, %.2f s at %d bytes/tick\n', ...
    total_packets, upload_duration, packets_per_tick * mavlink_packet_size);

% ---- pack -------------------------------------------------------------------------
pack = struct( ...
    'num_waypoints',    num_waypoints, ...
    'total_packets',    total_packets, ...
    'traj_upload',      traj_upload, ...
    'traj_upload_flat', traj_upload_flat, ...
    'upload_duration',  upload_duration, ...
    'upload_rate_vec',  upload_rate_vec, ...
    'upload_times',     upload_times);
end
