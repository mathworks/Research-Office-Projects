function plan = planShow(cfg)
%planShow  Build the flown mission plan for a drone light show.
%
%   plan = planShow(cfg) turns a show CONFIGURATION into a flown PLAN: the formation
%   geometry, the launch pads, the drone-to-point assignment for every transition, the
%   min-jerk trajectory through takeoff, holds, transitions and landing, the lighting
%   timeline that rides along with it, and the validation numbers that say whether what
%   came out is flyable.
%
%   This is the mission-planning half of setupParams. It was ~810 lines in the middle of
%   that script, which meant every intermediate it computed -- 96 of them, from `q` and
%   `idx` to `cost_matrix` and `hold_pos` -- became a public base-workspace variable, and
%   there was no way to tell those apart from the 38 results something actually consumes.
%   As a function the intermediates are locals and the interface is the returned struct.
%
%   INPUT  cfg, with the fields unpacked at the top of the function body. setupParams
%          builds it; the field names are exactly the parameter names it uses.
%
%   OUTPUT plan, a struct whose fields are the 38-name export set that the model, the
%          app and DroneShowExample read back. Which names belong in it is DERIVED
%          rather than chosen: every consumer was scanned for names it reads without
%          first binding them, and that scan is the thing to redo before adding or
%          removing a field here. plan.radius is present only when the show
%          contains a Circle or a Sphere, because that is the only branch that
%          assigns it, and the base workspace never had it otherwise either.
%
%   The body below is the original script region moved with as little editing as the move
%   allows. Inputs are unpacked into locals under THEIR ORIGINAL NAMES on purpose: it
%   keeps the ~810 lines that follow textually identical to what was verified, so a
%   transcription error cannot hide inside a rename. Only three things changed, and each
%   is marked MOVED: the `exist` test on custom_formations (a function cannot ask its
%   caller what is defined), the cf_margin default (hoisted to setupParams for the same
%   reason), and the pack at the end.
%
%   See also setupParams, packShowUpload, formationHoldSamples, sampleFormationCloud.

% ---- unpack -----------------------------------------------------------------------
% Geometry and fleet
N_uav                   = cfg.N_uav;
num_formations          = cfg.num_formations;
num_transitions         = cfg.num_transitions;
formation_sequence      = cfg.formation_sequence;
formation_spacing       = cfg.formation_spacing;
formation_radius_max    = cfg.formation_radius_max;
show_altitude           = cfg.show_altitude;
formation_text          = cfg.formation_text;
custom_formations       = cfg.custom_formations;    % [] when none are registered
cf_margin               = cfg.cf_margin;
d_min                   = cfg.d_min;
a_max                   = cfg.a_max;

% Timeline
takeoff_duration        = cfg.takeoff_duration;     % a FLOOR on input; raised below
hold_duration           = cfg.hold_duration;
transition_duration     = cfg.transition_duration;
climb_speed             = cfg.climb_speed;
land_speed              = cfg.land_speed;
land_settle             = cfg.land_settle;

% Sampling
num_samples_per_seg     = cfg.num_samples_per_seg;
num_samples_per_hold    = cfg.num_samples_per_hold;
MINJERK_PEAK_RATIO      = cfg.MINJERK_PEAK_RATIO;
MINJERK_ACCEL_PEAK_RATIO = cfg.MINJERK_ACCEL_PEAK_RATIO;

% Rotation
formation_rotate        = cfg.formation_rotate;
rotation_active         = cfg.rotation_active;
rotation_deg_request    = cfg.rotation_deg_request;
rotation_accel_target   = cfg.rotation_accel_target;
rotation_accel_frac     = cfg.rotation_accel_frac;

% Tracking limits and lighting
v_track_target          = cfg.v_track_target;
v_track_max             = cfg.v_track_max;
a_track_target          = cfg.a_track_target;
a_track_max             = cfg.a_track_max;
lighting_colors         = cfg.lighting_colors;

%% Pre-compute Mission Plan using minjerkpolytraj (UAV Toolbox)
% Generate all formation waypoints
all_formations = zeros(N_uav, 3, num_formations);
for f = 1:num_formations
    ftype = formation_sequence(f);
    % Set by the two branches that work from a candidate POINT CLOUD -- typed text
    % and a shape loaded from a file. Both then go through the same placement block
    % after the switch: sample to N, size it like a Grid, grow it until it clears
    % d_min, swap into NED, lift it off the ground. Text used to be a hard-coded
    % pair of vertical bars precisely because that block lived inside the custom
    % branch and there was no way to reach it.
    cloud_form = [];
    % Ground guard for the FLAT formations -- the counterpart of the sphere's clamp below,
    % which Grid and Circle were missing. Both write show_altitude straight into every
    % drone's z, so a caller who forgets NED is down-positive and asks for +10 gets a whole
    % show ten metres underground, and 0 gets one dragged along the pad at several m/s. In
    % both cases the planner reported separation and speed as usual and said nothing about
    % the altitude. Not reachable from the app -- its field is 5..100 m and it negates on
    % the way in -- but show_altitude comes from the base workspace.
    flat_altitude = min(show_altitude, -1);
    switch ftype
        case 1 % Grid
            cols_f = ceil(sqrt(N_uav));
            for i = 1:N_uav
                row_idx = floor((i-1)/cols_f);
                col_idx = mod((i-1), cols_f);
                cx = (cols_f-1)*formation_spacing/2;
                cy = (ceil(N_uav/cols_f)-1)*formation_spacing/2;
                all_formations(i,:,f) = [col_idx*formation_spacing - cx, ...
                    row_idx*formation_spacing - cy, flat_altitude];
            end
        case 2 % Circle: one ring while that is flyable, concentric rings when it is not
            radius = N_uav * formation_spacing / (2*pi);
            if radius <= formation_radius_max
                angles = linspace(0, 2*pi*(1-1/N_uav), N_uav);
                for i = 1:N_uav
                    all_formations(i,:,f) = [radius*cos(angles(i)), ...
                        radius*sin(angles(i)), flat_altitude];
                end
            else
                % A disc of rings at k*spacing for k = 1..K. Ring k holds ~2*pi*k drones
                % at the spacing, so K rings hold ~pi*K^2 and K = ceil(sqrt(N/pi)) --
                % the radius is now sqrt(N) rather than N. Handing each ring a share of
                % the fleet proportional to k (its circumference) makes the count per
                % ring ~2*pi*k, which is what puts the spacing at ~formation_spacing
                % both along a ring and between rings.
                K = ceil(sqrt(N_uav / pi));
                share = (1:K)';
                per_ring = floor(N_uav * share / sum(share));
                % Largest-remainder, so the ring counts sum to N_uav exactly rather than
                % leaving drones at the origin or overrunning all_formations.
                [~, by_rem] = sort(N_uav * share / sum(share) - per_ring, 'descend');
                short = N_uav - sum(per_ring);
                per_ring(by_rem(1:short)) = per_ring(by_rem(1:short)) + 1;
                radius = K * formation_spacing;   % outer radius, for the record below
                i = 0;
                for k = 1:K
                    r_k = k * formation_spacing;
                    % Half a slot of twist on alternate rings, so drones do not line up
                    % radially -- from the ground a disc of spokes reads as a grid.
                    tw = pi / per_ring(k) * mod(k, 2);
                    for j = 1:per_ring(k)
                        ang = 2*pi*(j-1)/per_ring(k) + tw;
                        i = i + 1;
                        all_formations(i,:,f) = [r_k*cos(ang), r_k*sin(ang), ...
                            flat_altitude];
                    end
                end
            end
        case 3 % Sphere (centered so all points stay above ground in NED)
            radius = N_uav * formation_spacing / (2*pi);
            if radius > formation_radius_max
                % Same move as Circle, one dimension up: hold the spacing over the shell
                % AREA rather than around a great circle. N drones on a sphere of radius
                % r sit sqrt(4*pi*r^2/N) apart, so r = spacing*sqrt(N/(4*pi)) puts them
                % at the spacing, and the extent again grows as sqrt(N). The golden-spiral
                % placement below already distributes evenly over the shell, so only the
                % radius changes -- 500 drones go from a 398 m ball to a 31.5 m one.
                radius = formation_spacing * sqrt(N_uav / (4*pi));
            end
            golden_ratio = (1 + sqrt(5)) / 2;
            sphere_center_z = show_altitude - radius;
            sphere_pts = zeros(N_uav, 3);
            for i = 1:N_uav
                theta = acos(1 - 2*(i-0.5)/N_uav);
                phi = 2*pi*i/golden_ratio;
                sphere_pts(i,:) = [radius*sin(theta)*cos(phi), ...
                    radius*sin(theta)*sin(phi), sphere_center_z + radius*cos(theta)];
            end
            % Sit the LOWEST DRONE on show_altitude, which is what the app's Altitude
            % field promises and what Grid, Circle, the disc and the cloud path all
            % already do.
            %
            % Centring at show_altitude - radius is not quite enough on its own, and the
            % gap is not rounding: the golden-spiral placement above offsets every point
            % by half a step, so cos(theta) spans [-1+1/N, 1-1/N] and NO drone is ever at
            % either pole. The nominal shell bottom is therefore empty and the lowest
            % actual drone sits radius/N above it -- which for the 2*pi radius law is
            % formation_spacing/(2*pi), independent of the fleet size. That is 0.8 m at
            % the default 5 m spacing and 4.8 m at the 30 m the field allows, so it is
            % worth removing rather than tolerating.
            sphere_pts(:,3) = sphere_pts(:,3) - (max(sphere_pts(:,3)) - show_altitude);
            % Ground guard, reachable only from a script: the app cannot ask for less
            % than 5 m, but show_altitude comes from the base workspace. Per-drone here
            % rather than a whole-shape shift because a sphere's clipped cap is hidden
            % inside the shell, whereas flattening a billboard's bottom edge shows.
            sphere_pts(:,3) = min(sphere_pts(:,3), -1);
            all_formations(:,:,f) = sphere_pts;
        case 4 % Text: whatever formation_text says, in a real font
            % This used to be two vertical bars, nominally "HI", built from
            % ceil(N_uav/2) with no string involved anywhere -- so "Text" was just a
            % fourth fixed shape that could not be set to anything. It now renders
            % formation_text with a font and traces the stroke centrelines.
            cloud_form = formationFromText(formation_text);

            % About 10 drones per character before a word reads as letters rather
            % than as a scatter -- measured, not guessed: "MATLAB" is illegible at
            % 40 drones and clear at 100. Advisory, not an error: flying a long
            % word with a small fleet is a legitimate thing to simulate, and
            % whether it is worth flying is the operator's call.
            text_chars = numel(regexprep(formation_text, '[\s|]', ''));
            if text_chars > 0 && N_uav < 10 * text_chars
                fprintf(['Formation %d: "%s" is %d characters for %d drones ' ...
                         '(%.1f each). Below ~10 per character it will not read ' ...
                         'as text -- shorten it, split it with "|", or fly ' ...
                         'more drones.\n'], ...
                    f, formation_text, text_chars, N_uav, N_uav / text_chars);
            end
        otherwise % 5 and up: a shape loaded from a picture or an STL, or typed text
            cf_i = ftype - 4;
            cf_n = 0;
            % MOVED: was `exist('custom_formations', 'var') && isstruct(...)`. A function
            % cannot see whether its caller defined a variable, so setupParams passes []
            % when nothing is registered and the isstruct test carries the whole question.
            if isstruct(custom_formations)
                cf_n = numel(custom_formations);
            end
            if cf_i < 1 || cf_i > cf_n
                % all_formations is preallocated to zeros, so an unhandled type used
                % to fall straight through and leave EVERY drone at the origin, on
                % the ground — a whole formation silently collapsed to one point.
                % Fail loudly instead.
                error('setupParams:unknownFormation', ...
                    ['formation_sequence(%d) = %d is not a known formation type. ' ...
                     '1 (Grid), 2 (Circle), 3 (Sphere) and 4 (Text, spelling ' ...
                     'formation_text) are built in; 5 and up index ' ...
                     'custom_formations, and %d shape(s) are loaded. Add one with ' ...
                     'formationFromMedia or formationFromText, or use the app''s ' ...
                     '"Load Image / STL" or "Fly Text" button.'], f, ftype, cf_n);
            end
            cloud_form = custom_formations(cf_i);
    end

    % ---- Shared placement for anything that came from a point cloud ----
    if ~isempty(cloud_form)
        % The cloud is a set of CANDIDATES, not N points, and the fleet size is
        % resolved HERE -- so changing N_uav re-samples the shape for the new count
        % instead of stranding it at whatever the fleet happened to be when the file
        % was loaded or the text was typed.
        [cf_pts, cf_info] = sampleFormationCloud(cloud_form, N_uav);

        % Size it like a Grid of the same fleet would be, so a loaded shape drops
        % into an existing show without re-tuning formation_spacing. cf_pts is
        % normalised to a half-extent of 1, so this is a straight multiply and the
        % aspect ratio of the original picture, mesh or lettering survives it.
        cf_half = (ceil(sqrt(N_uav)) - 1) * formation_spacing / 2;
        if cf_half <= 0, cf_half = formation_spacing; end
        cf_pts = cf_pts * cf_half;

        % Then grow it until the closest pair clears d_min WITH MARGIN. Scaling is
        % the right lever: nudging individual drones apart would deform the shape,
        % while a bigger picture is still the same picture.
        %
        % The margin has to cover the transition, not just the hold. min_sep_achieved is
        % checked across every sample of the flown plan, and drones lose ground on the way
        % in, so a formation sitting a few percent above d_min flies straight through it --
        % the first version of this used 1.05 and breached at 25 and 40 drones while
        % passing at 12.
        %
        % sqrt(2) is where that stops being trial and error. The assignment at cost_matrix
        % below minimises SQUARED distance, which bounds the flown separation at the
        % tighter formation's own separation divided by sqrt(2) -- see the derivation
        % there. So a cloud grown to d_min*sqrt(2) = 1.414*d_min cannot breach d_min in
        % flight, whatever shape it is and however many drones are in it. 1.6 is that
        % minimum plus 13%, and the 27% loss this comment used to quote as the thing being
        % covered (5.0 m at rest dipping to 3.67 m) was the same bound being measured.
        %
        % MOVED: the `if ~exist('cf_margin','var'), cf_margin = 1.6; end` default that
        % stood here is now in setupParams. It has to be: a function cannot ask its
        % caller what is defined, and cf_margin is an operator-settable base variable.
        % The derivation stays HERE, next to the scaling it justifies and next to the
        % squared-cost bound at cost_matrix that it turns on.
        cf_sep = inf;
        if N_uav > 1
            cf_sep = min(pdist(cf_pts));
            if cf_sep > 0 && cf_sep < d_min * cf_margin
                cf_pts = cf_pts * (d_min * cf_margin / cf_sep);
                cf_sep = min(pdist(cf_pts));
            end
        end

        % Into NED. The cloud frame is East/North/Up and NED is North/East/Down, so
        % the first two columns SWAP -- passing them straight through put the
        % picture in the North-Up plane while every comment claimed East, and it
        % showed up as a billboard facing 90 degrees off.
        %
        % A 2D picture or a line of text has y = 0 for every point, which makes this
        % a VERTICAL BILLBOARD automatically: it stands in the East-Up plane facing
        % the audience, rather than lying flat at altitude where it is readable only
        % from directly overhead. For text that is the difference between a word and
        % a smear.
        %
        % show_altitude is the altitude of the LOWEST DRONE, which is what the app's
        % Altitude field says and what every other formation already means: Grid,
        % Circle and the disc are flat at it, and the Sphere is deliberately centred
        % at show_altitude - radius so its bottom sits there too. So the cloud is
        % referenced from its own lowest point (subtract min, which makes the bottom
        % of the shape the zero of the Up axis) and grows upward from there.
        %
        % This used to centre the shape on show_altitude instead, which meant half of
        % every picture hung BELOW the figure the operator typed. It was invisible
        % because a separate guard then shifted anything within 1 m of the ground back
        % up -- so under about 40 drones it looked fine, and above that the altitude
        % field silently stopped meaning anything at all: every value below the
        % shape's half-height produced the identical flight. Measured for "MATLAB",
        % the half-height passes 10 m at 63 drones and reaches 146 m at 500.
        %
        % Referencing the bottom makes that guard unnecessary rather than merely
        % unlikely to fire: the lowest drone is AT show_altitude by construction, so
        % the shape cannot reach the ground for any altitude the app can request.
        cf_ned = [cf_pts(:,2), cf_pts(:,1), ...
                  show_altitude - (cf_pts(:,3) - min(cf_pts(:,3)))];

        % Only reachable from a script: the app's Altitude field cannot go below 5 m,
        % but setupParams takes show_altitude from the base workspace and a caller can
        % put anything there. Kept as a floor on the whole shape rather than a per-drone
        % clamp -- clamping a billboard flattens the bottom of the picture into a
        % straight line, which is plainly visible from the ground.
        cf_lowest = max(cf_ned(:,3));    % NED: the largest z is the lowest drone
        if cf_lowest > -1
            cf_ned(:,3) = cf_ned(:,3) - (cf_lowest + 1);
        end
        all_formations(:,:,f) = cf_ned;

        % The altitude BAND, not just the span: the shape stands on show_altitude and
        % grows upward, so how high it reaches is the number an operator cannot get
        % from the panel and the one that decides whether the show is airspace-legal.
        fprintf(['Formation %d: "%s" (%s) -- %d candidates%s, ' ...
                 'span E %.1f x N %.1f x up %.1f m, %.1f-%.1f m AGL, ' ...
                 'closest pair %.2f m\n'], ...
            f, cloud_form.name, cloud_form.kind, ...
            cf_info.nCandidates, ...
            repmat(' [filled, outline too sparse]', 1, cf_info.usedFill), ...
            range(cf_ned(:,2)), range(cf_ned(:,1)), range(cf_ned(:,3)), ...
            -max(cf_ned(:,3)), -min(cf_ned(:,3)), ...
            cf_sep);
    end
end

% Launch pads: the first formation projected onto the ground (Z=0 in NED).
%
% Projecting straight down gives one pad per drone only when the first formation has
% a distinct footprint per drone, and a BILLBOARD does not. Typed text and a loaded
% picture stand up vertically, so every drone in a column shares its North and East,
% and the whole column projects onto a SINGLE pad -- the fleet starts the show already
% in collision (measured: 0.00 m pad separation on a text formation flown first, and
% the app offers exactly that as a one-formation show).
%
% So when the projection collides, launch from a proper GROUND GRID instead: the same
% grid formation 1 would be, at formation_spacing, centred on the footprint the show
% occupies. Which is what a real show does -- drones sit on a launch grid, not under
% wherever the first picture happens to put them.
%
% Nudging the pads apart is not enough, and two attempts at it are worth recording so
% they are not tried again. A billboard's projection is degenerate in BOTH ground axes,
% not just one: 30 drones spelling "HI" project onto 10 distinct East coordinates with
% neighbouring ones as little as 0.46 m apart, because the formation clears d_min in
% 3-D by being spread vertically. So fanning each column apart along North still left
% pads 0.46 m apart, and nudging every pad clear of every pad already placed cascaded
% them into one long North row -- which pushed the peak commanded speed to 21.6 m/s,
% worse than the grid's 18.0, because the takeoff has a FIXED duration and every metre
% of pad offset is a metre the fleet has to cover inside it.
%
% Pads that are already clear are left alone, so every Grid, Circle and Sphere show
% launches from exactly where it used to.
init_positions = all_formations(:,:,1);
init_positions(:,3) = 0;  % start on ground

if min(pdist(init_positions)) < d_min
    pad_cols = ceil(sqrt(N_uav));
    pad_cx = (pad_cols - 1) * formation_spacing / 2;
    pad_cy = (ceil(N_uav/pad_cols) - 1) * formation_spacing / 2;
    pad_centre = mean(all_formations(:,1:2,1), 1);
    for pad_i = 1:N_uav
        init_positions(pad_i,:) = [ ...
            pad_centre(1) + mod(pad_i-1, pad_cols) * formation_spacing - pad_cx, ...
            pad_centre(2) + floor((pad_i-1)/pad_cols) * formation_spacing - pad_cy, ...
            0];
    end

    % Which drone gets which pad. The takeoff below flies drone i from pad i to point i
    % of formation 1 with no assignment step of its own -- it never needed one while the
    % pad was directly underneath -- so the pairing has to be right HERE or the fleet
    % crosses itself on the way up. matchpairs minimises total climb path, which is also
    % the cheapest takeoff.
    %
    % Squared distance and a scaled costUnmatched, for the reason spelled out at the
    % transition assignment below: the climb is the same shared-s straight-line
    % interpolation, so the squared cost is what bounds separation during it at
    % min(padSep, formSep)/sqrt(2) rather than merely keeping the paths from crossing.
    pad_cost = pdist2(all_formations(:,:,1), init_positions, 'squaredeuclidean');
    pad_pairs = matchpairs(pad_cost, max(pad_cost(:)) + 1);
    pad_of = zeros(N_uav, 1);
    for i = 1:size(pad_pairs, 1)
        pad_of(pad_pairs(i,1)) = pad_pairs(i,2);
    end
    init_positions = init_positions(pad_of, :);
end

%% Takeoff duration — sized to the climb the fleet actually has to make
% Three terms, and they answer three different questions. The distance is the longest
% pad-to-first-slot PATH in the fleet, not the altitude: a drone crossing as it climbs has
% further to go than the billboard is tall.
takeoff_dist = max(vecnorm(all_formations(:,:,1) - init_positions, 2, 2));

% (1) The floor from cfg, for a show short enough that the other two ask for almost no
%     time at all. (2) How fast the climb should LOOK, from climb_speed, averaged along
%     the path exactly as land_duration is from land_speed -- so the fleet rises at the
%     rate it descends. (3) How fast the climb is ALLOWED to be, by inverting the min-jerk
%     peak against v_track_target, the same way traj_min_transition sizes a transition.
%
% Term 3 was doing term 2's job before climb_speed existed, and that was the defect behind
% "the drones take off too fast": it lengthens a climb only far enough to stay trackable,
% so on anything tall it lands the peak exactly ON v_track_target -- 7.00 m/s measured for
% a 30-drone text billboard -- and on the default show it does not bind at all, leaving the
% flat 5 s floor to put the peak at 4.38 m/s against the descent's 3.28. The fleet went up
% between 1.33x and 2.13x faster than it came down, with nothing in the plan intending it.
%
% Term 3 is kept rather than deleted even though term 2 dominates it at the shipped rate.
% It is not redundant: climb_speed is settable, and the two cross over at
% v_track_target / MINJERK_PEAK_RATIO = 3.2 m/s, above which term 3 is the only thing
% standing between a briskly-set climb and an uncommandable one. Below it, flyability
% comes free -- a climb slow enough to look right is always slow enough to fly.
takeoff_duration = max([takeoff_duration, ...
    takeoff_dist / climb_speed, ...
    MINJERK_PEAK_RATIO * takeoff_dist / v_track_target]);

% Flight time up to the end of the last hold. show_duration is completed after the
% trajectory is built, because the descent length depends on where the fleet
% actually ends up (a sphere finishes higher than its nominal show_altitude).
show_flight_duration = takeoff_duration + hold_duration + num_transitions * (transition_duration + hold_duration);

% Show timeline time vector (offset by takeoff). Built here rather than up with the
% other durations because it starts at takeoff_duration, which is only known now.
timeline_times = zeros(1, 2*num_formations - 1);
timeline_types = zeros(1, 2*num_formations - 1); % 0=hold, 1=transition
t_curr = takeoff_duration;  % formations start after takeoff
for i = 1:num_formations
    idx = 2*i - 1;
    timeline_times(idx) = t_curr;
    timeline_types(idx) = 0; % hold
    t_curr = t_curr + hold_duration;
    if i < num_formations
        idx2 = 2*i;
        timeline_times(idx2) = t_curr;
        timeline_types(idx2) = 1; % transition
        t_curr = t_curr + transition_duration;
    end
end

% Drone-to-point assignment using matchpairs (Optimization Toolbox)
% and minimum-jerk trajectory generation (UAV Toolbox)
% Extra space for the takeoff and landing segments, plus the anchors: one on the
% ground at t=0, one closing the last hold and two parked. Each hold contributes
% num_samples_per_hold(f), which is 1 unless THAT formation rotates -- so this is
% the same number it always was for a still show, and a mixed show pays only for
% the holds that actually spin.
max_samples = num_samples_per_seg * (num_transitions + 2) + ...
              sum(num_samples_per_hold) + 4;

% What the rotation actually achieved, per formation, for the report below and for
% the app to read back. Always defined, zero-filled when rotation is off, so a
% caller never has to test for existence first.
rotation_sweep = zeros(1, num_formations);      % radians swept during each hold
rotation_radius = zeros(1, num_formations);      % largest radius from the spin axis
rotation_need_hold = zeros(1, num_formations);   % hold this formation's request would need
% Which of the two caps bound, per formation. Worth recording rather than re-deriving for
% the report: telling an operator the spin is "speed-limited" when acceleration is what
% stopped it points them at the wrong dial, and a longer hold cures both but a smaller
% formation only cures one.
rotation_limit = repmat({'none'}, 1, num_formations);
% What the spin actually asks of the fleet, per formation. Exported because "the target was
% 7 m/s" stopped being a useful thing to report the moment acceleration became the binding
% cap: the tangential speed a rotating hold reaches is now well under it, and quoting the
% target invites a reader to think the fleet is being pushed to its speed limit when it is
% nowhere near it.
rotation_peak_speed = zeros(1, num_formations);   % m/s, peak tangential, widest drone
rotation_peak_accel = zeros(1, num_formations);   % m/s^2, peak total, widest drone
trajectory_data = zeros(N_uav, 7, max_samples);
time_vector = zeros(1, max_samples);
sample_idx = 1;

% Ground hold (t=0)
trajectory_data(:, 1:3, sample_idx) = init_positions;
trajectory_data(:, 4:6, sample_idx) = 0;
trajectory_data(:, 7, sample_idx) = 0;
time_vector(sample_idx) = 0;
sample_idx = sample_idx + 1;

% Takeoff: min-jerk from ground to first formation
t_seg_takeoff = [0, takeoff_duration];
t_samples_takeoff = linspace(0, takeoff_duration, num_samples_per_seg);
for uav_i = 1:N_uav
    wp = [init_positions(uav_i,:)', all_formations(uav_i,:,1)'];
    [q, qd, ~, ~, ~, ~, ~] = minjerkpolytraj(wp, t_seg_takeoff, num_samples_per_seg);
    for s = 1:num_samples_per_seg
        idx_out = sample_idx + s - 1;
        trajectory_data(uav_i, 1:3, idx_out) = q(:,s)';
        trajectory_data(uav_i, 4:6, idx_out) = qd(:,s)';
        trajectory_data(uav_i, 7, idx_out) = 0;
    end
end
time_vector(sample_idx:sample_idx+num_samples_per_seg-1) = t_samples_takeoff;
sample_idx = sample_idx + num_samples_per_seg;

% Hold at first formation after takeoff. One anchor when still, a rotation series
% when not -- see the twin emit at the end of the transition loop below, which is
% the same four lines and has to stay in step with this one.
[hold_pos, hold_vel, hold_t, hold_info] = formationHoldSamples( ...
    all_formations(:,:,1), takeoff_duration, hold_duration, ...
    num_samples_per_hold(1), rotation_deg_request(1) / 360, ...
    v_track_target, MINJERK_PEAK_RATIO, ...
    rotation_accel_target, MINJERK_ACCEL_PEAK_RATIO);
n_hold = numel(hold_t);
trajectory_data(:, 1:3, sample_idx:sample_idx+n_hold-1) = hold_pos;
trajectory_data(:, 4:6, sample_idx:sample_idx+n_hold-1) = hold_vel;
trajectory_data(:, 7, sample_idx:sample_idx+n_hold-1) = 0;
time_vector(sample_idx:sample_idx+n_hold-1) = hold_t;
sample_idx = sample_idx + n_hold;
rotation_sweep(1) = hold_info.thetaTotal;
rotation_radius(1) = hold_info.radiusMax;
rotation_need_hold(1) = hold_info.holdForFullTurns;
rotation_limit{1} = hold_info.limitedBy;
rotation_peak_speed(1) = hold_info.peakSpeed;
rotation_peak_accel(1) = hold_info.peakAccel;

% Where the fleet is when the hold ENDS, which is where the next transition has to
% start from. Without rotation this is just the formation again; with it, the fleet
% has turned, and using the unrotated formation here would teleport it back at
% every hold-to-transition boundary -- and, worse, would hand the assignment below
% a cost matrix for a departure point the fleet is not at.
hold_end_pos = reshape(hold_pos(:, :, end), N_uav, 3);

assignment_order = zeros(N_uav, num_formations);
assignment_order(:,1) = (1:N_uav)';

for tr = 1:num_transitions
    current_pos = hold_end_pos;
    target_pos = all_formations(:,:,tr+1);

    % Cost matrix for Hungarian assignment (matchpairs), SQUARED distance.
    %
    % Squared, not plain distance, and this is the whole reason the fleet stays apart in
    % flight. Every drone flies the same two-waypoint minjerk over the same interval, so
    % they all sit at the same fraction s(t) in [0,1] of their own straight segment:
    %   p_i(t) = a_i + s(t)*(b_i - a_i)
    % A 2-swap-optimal assignment under SQUARED cost satisfies, for every pair i,j,
    %   |a_i-b_i|^2 + |a_j-b_j|^2 <= |a_i-b_j|^2 + |a_j-b_i|^2   =>   <da, db> >= 0
    % (expand and cancel the |a|^2 and |b|^2 terms; the cross terms are what is left).
    % That non-negative inner product is a real separation guarantee:
    %   |d(s)|^2 = (1-s)^2|da|^2 + s^2|db|^2 + 2s(1-s)<da,db>
    %            >= ((1-s)^2 + s^2) * min(|da|^2,|db|^2)  >=  min(|da|,|db|)^2 / 2
    % so the flown separation can never drop below the tighter of the two formations'
    % separations divided by sqrt(2). At the default 5 m spacing that is 3.54 m against a
    % 2.00 m requirement -- and it is a PROOF, holding at any fleet size and between any
    % two shapes, not a number that happened to come out of a measurement.
    %
    % Plain distance gives only non-crossing paths, with no bound on how close they come.
    % That was invisible for as long as Circle was a single ring: a 174 m ring against a
    % 110 m grid has every drone fanning outward, the difference vectors barely rotate, and
    % the Euclidean and squared optima coincide (measured 3.54-3.59 m, i.e. already at the
    % bound above). The moment Circle became a 45 m disc the flows converged instead of
    % fanning, difference vectors rotated through short directions, and the same code
    % measured 0.78 m at 220 drones and 0.58 m at 500. Lengthening the transition cannot
    % fix that -- s(t) is shared, so the PATH geometry, and therefore the minimum
    % separation, is independent of transition_duration.
    %
    % Rotating formations do not weaken any of this. current_pos is the departure
    % formation as ROTATED by its hold, and the bound only ever refers to the two
    % endpoint sets' own separations -- which a rigid rotation preserves exactly. So
    % the guarantee is the same staticSep/sqrt(2) whether the fleet spun or not. What
    % rotation DOES change is which pairing is cheapest, and that is why current_pos
    % has to be the rotated positions rather than all_formations: hand this the
    % unrotated formation and the assignment is optimal for a departure point the
    % fleet is not standing at.
    cost_matrix = pdist2(current_pos, target_pos, 'squaredeuclidean');

    % costUnmatched has to scale with the cost now. matchpairs leaves a pair unmatched when
    % its cost exceeds 2*costUnmatched, and squared distances run to ~1.7e4 over a 130 m
    % disc, so the old flat 1000 would have started declining to assign drones at all --
    % silently, as a short `assignments` list and a perm with zeros in it.
    assignments = matchpairs(cost_matrix, max(cost_matrix(:)) + 1);
    perm = zeros(N_uav, 1);
    for i = 1:size(assignments, 1)
        perm(assignments(i,1)) = assignments(i,2);
    end
    assignment_order(:, tr+1) = perm;

    % Generate minimum-jerk trajectory for each UAV in this transition
    t_start = timeline_times(2*tr);
    t_seg = [0, transition_duration];
    t_samples = linspace(0, transition_duration, num_samples_per_seg);

    for uav_i = 1:N_uav
        wp = [current_pos(uav_i,:)', target_pos(perm(uav_i),:)'];
        [q, qd, ~, ~, ~, ~, ~] = minjerkpolytraj(wp, t_seg, num_samples_per_seg);

        for s = 1:num_samples_per_seg
            idx_out = sample_idx + s - 1;
            trajectory_data(uav_i, 1:3, idx_out) = q(:,s)';
            trajectory_data(uav_i, 4:6, idx_out) = qd(:,s)';
            trajectory_data(uav_i, 7, idx_out) = 0; % yaw = 0
        end
    end
    time_vector(sample_idx:sample_idx+num_samples_per_seg-1) = t_start + t_samples;
    sample_idx = sample_idx + num_samples_per_seg;

    % Hold at target formation. Twin of the emit above the loop -- keep the two in
    % step. One anchor when THIS formation is still; a rotation series when it spins.
    [hold_pos, hold_vel, hold_t, hold_info] = formationHoldSamples( ...
        target_pos(perm,:), t_start + transition_duration, hold_duration, ...
        num_samples_per_hold(tr+1), rotation_deg_request(tr+1) / 360, ...
        v_track_target, MINJERK_PEAK_RATIO, ...
        rotation_accel_target, MINJERK_ACCEL_PEAK_RATIO);
    n_hold = numel(hold_t);
    trajectory_data(:, 1:3, sample_idx:sample_idx+n_hold-1) = hold_pos;
    trajectory_data(:, 4:6, sample_idx:sample_idx+n_hold-1) = hold_vel;
    trajectory_data(:, 7, sample_idx:sample_idx+n_hold-1) = 0;
    time_vector(sample_idx:sample_idx+n_hold-1) = hold_t;
    sample_idx = sample_idx + n_hold;
    rotation_sweep(tr+1) = hold_info.thetaTotal;
    rotation_radius(tr+1) = hold_info.radiusMax;
    rotation_need_hold(tr+1) = hold_info.holdForFullTurns;
    rotation_limit{tr+1} = hold_info.limitedBy;
    rotation_peak_speed(tr+1) = hold_info.peakSpeed;
    rotation_peak_accel(tr+1) = hold_info.peakAccel;
    hold_end_pos = reshape(hold_pos(:, :, end), N_uav, 3);
end

%% Landing — descend from the last hold to the ground
% The descent lives HERE, in the plan, rather than in the supervisor, because the
% plan is what gets uploaded: upload_times spans show_duration, so folding the
% landing in puts real landing waypoints on the wire for the MAVLink path, into
% traj_show_ts for the Workspace path, and into the planned playback the app draws
% before any simulation runs. One change, and all three land.
%
% The fleet used to finish the show hovering at show_altitude: the plan stopped at
% the *start* of the last hold and let interp1 extrapolate the rest, so there was
% never a sample to descend from. Both are fixed below -- the last hold gets a
% closing anchor, then the descent, then the parked anchors.
land_from_idx = sample_idx - 1;                        % start-of-last-hold anchor
land_from_pos = reshape(trajectory_data(:, 1:3, land_from_idx), N_uav, 3);

% Where each drone lands. This used to drop every drone straight down, and that
% collides for the same reason the launch pads did: a billboard flown LAST stacks
% drones in a column, so a vertical descent puts the whole column on one spot
% (measured 0.00 m against a 2.00 m requirement). Land the fleet back on its launch
% pads instead. They are already d_min apart whatever the show flew, it is what a real
% show does, and matchpairs -- the same assignment the
% transitions use -- picks the pad-per-drone with the least total travel, so the
% descent stays as near vertical as the geometry allows.
% Squared distance and a scaled costUnmatched here too -- same guarantee, same reason.
land_cost = pdist2(land_from_pos, init_positions, 'squaredeuclidean');
land_pairs = matchpairs(land_cost, max(land_cost(:)) + 1);
land_pad_of = zeros(N_uav, 1);
for i = 1:size(land_pairs, 1)
    land_pad_of(land_pairs(i,1)) = land_pairs(i,2);
end
land_ground_pos = init_positions(land_pad_of, :);

% Sized by the longest PATH, not the longest drop: a drone that has to come across as
% well as down would otherwise be told to fly the diagonal in the time the drop alone
% needs, and land_speed would stop meaning anything.
land_dist = max(vecnorm(land_ground_pos - land_from_pos, 2, 2));
land_duration = land_dist / land_speed;
show_duration = show_flight_duration + land_duration + land_settle;

% ShowSupervisor parameters. land_start_time is the ShowTime at which the planned
% descent begins, which is what moves the chart out of SHOW and into LANDING -- so
% the phase readout says "Landing" while the fleet is actually coming down, instead
% of claiming the show is still running until the instant it is parked.
% Touchdown, which is where LANDED begins -- not the end of the show. The fleet
% spends the settle window parked and reported as LANDED, so the phase readout is
% honest at both ends of the descent.
land_start_time = show_flight_duration;
land_end_time = show_flight_duration + land_duration;
land_total_duration = land_duration + land_settle;

% Close the last hold, so the fleet is commanded to stay in formation through it.
%
% Skipped when the formations rotate: the rotation series already spans the whole
% hold and its last sample IS this instant, so emitting another anchor here would
% put two samples on one time. interp1 would survive it (time_vector is deduped
% with unique(...,'stable') before every interpolation) but the duplicate carries
% zero velocity against the rotation's, and which of the two wins would then
% depend on dedup order rather than on anything intended.
if time_vector(land_from_idx) < show_flight_duration - 1e-9
    trajectory_data(:, 1:3, sample_idx) = land_from_pos;
    trajectory_data(:, 4:6, sample_idx) = 0;
    trajectory_data(:, 7, sample_idx) = 0;
    time_vector(sample_idx) = show_flight_duration;
    sample_idx = sample_idx + 1;
end

% Descent. Every drone takes the same land_duration, sized by the longest path, so
% the fleet touches down together rather than in a ragged wave.
% Min-jerk rather than a constant-speed ramp: columns 4:6 are commanded velocity,
% and a ramp would order a non-zero descent rate at the instant of touchdown.
t_samples_land = linspace(0, land_duration, num_samples_per_seg);
for uav_i = 1:N_uav
    wp = [land_from_pos(uav_i,:)', land_ground_pos(uav_i,:)'];
    [q, qd, ~, ~, ~, ~, ~] = minjerkpolytraj(wp, [0, land_duration], num_samples_per_seg);
    for s = 1:num_samples_per_seg
        idx_out = sample_idx + s - 1;
        trajectory_data(uav_i, 1:3, idx_out) = q(:,s)';
        trajectory_data(uav_i, 4:6, idx_out) = qd(:,s)';
        trajectory_data(uav_i, 7, idx_out) = 0;
    end
end
time_vector(sample_idx:sample_idx+num_samples_per_seg-1) = show_flight_duration + t_samples_land;
sample_idx = sample_idx + num_samples_per_seg;

% Parked on the ground, at both ends of the settle window. Two anchors, not one:
% upload_times steps at a fixed rate and rarely divides show_duration exactly, so
% the last uploaded waypoint lands somewhere inside this window and both ends of it
% have to read as "on the ground" -- otherwise the interpolation lifts the fleet
% back off for the final fraction of a second. land_ground_pos is the pad assignment
% computed above, so the parked fleet sits exactly where it launched from.
for t_park = [show_flight_duration + land_duration, show_duration]
    trajectory_data(:, 1:3, sample_idx) = land_ground_pos;
    trajectory_data(:, 4:6, sample_idx) = 0;
    trajectory_data(:, 7, sample_idx) = 0;
    time_vector(sample_idx) = t_park;
    sample_idx = sample_idx + 1;
end

% Trim to actual used samples
num_total_samples = sample_idx - 1;
trajectory_data = trajectory_data(:,:,1:num_total_samples);
time_vector = time_vector(1:num_total_samples);

% Validate separation constraints.
%
% This is a CHECK on a guarantee, not the thing keeping the fleet apart. The squared-cost
% assignment at cost_matrix above bounds the flown separation below by the tightest static
% arrangement's own separation over sqrt(2), so at the default 5 m spacing this prints
% 3.54 m and prints it at 220, 500 and 1000 drones alike -- the number stopped drifting
% with the fleet size when the cost became squared. Read a value materially below
% staticSep/sqrt(2) as a defect in the assignment or in the shared time profile the bound
% assumes, not as a fleet that happens to be tight.
%
% Every pair at every sample, but through pdist rather than a nested loop over i and j.
% The loop form was num_total_samples * N_uav^2/2 interpreted `norm` calls, which is
% 4.8 M at 200 drones and 48 M at 1000 -- minutes of plan build before a solver starts,
% for a check that reports one number. pdist does the same pairs in compiled code and is
% already used above for init_positions, so this adds no dependency.
min_sep_achieved = inf;
for s = 1:num_total_samples
    if N_uav < 2
        break;                      % no pairs to check in a fleet of one
    end
    sep_s = min(pdist(trajectory_data(:, 1:3, s)));
    if sep_s < min_sep_achieved
        min_sep_achieved = sep_s;
    end
end
fprintf('Minimum separation achieved: %.2f m (required: %.2f m)\n', min_sep_achieved, d_min);
if min_sep_achieved < d_min
    warning('Separation constraint violated! Consider increasing formation_spacing.');
end

% Validate that the transitions are actually FLYABLE.
%
% Separation was the only constraint checked here, and it is not the one that
% breaks. Required speed is formation span / transition_duration: the span grows
% with fleet size, transition_duration is whatever the user typed and never moves.
% So a plan that is geometrically perfect can still ask for speeds the guidance
% model cannot track, and the fleet arrives late or -- worse -- overshoots and
% never recovers. That is the "a couple of drones got out of formation at the end"
% failure, and at larger fleets it stops being a couple.
%
% Measured by sweeping transition_duration at fixed geometry, so only the demanded
% speed changes:
%
%   peak demand  peak flown   error at end of show
%      4.83         4.36            0.121 m   tracks
%      5.37         5.08            0.128 m   tracks
%      7.16         6.92            0.295 m   tracks
%      7.25         6.03            0.289 m   tracks
%     10.74        17.57          127.671 m   diverges (overshoots the command)
%     14.50         8.18            8.575 m   diverges
%     21.47        24.84           69.500 m   diverges
%
% Tracking is intact to 7.25 m/s and gone by 10.74. The limit below is set at 8
% m/s: just above the fastest speed measured good, well below the slowest measured
% bad. The band 7.25-10.74 m/s is untested, so treat 8 as advisory rather than a
% cliff edge.
%
% This table used to carry the note "the failure is not saturation", on the grounds
% that at 10.74 m/s demanded the fleet flew 17.57 m/s and so was being driven
% unstable rather than running out of authority. That reading was wrong, and the
% acceleration check below is the correction. Saturation is exactly what it is, and
% the flown speed EXCEEDING the demand is a symptom of it rather than evidence
% against it: once SatX/SatY are pinned the loop is open, so the drone keeps
% whatever velocity it had instead of being pulled back to the commanded one.
% Measured on the escaping drone at 36 drones, off the true-position log:
%
%   - horizontal acceleration ran at 100% of the per-axis clamp corner (4.26 of
%     4.24 m/s^2) for the whole divergence, with one axis pinned 88.4% of samples
%   - only 28% of that full authority pointed at the target -- the delivered
%     acceleration sat 81.7 deg off the direction to the setpoint, so the drone
%     wheeled around its commanded position instead of closing on it
%   - it is DURATION of saturation that separates a recovery from an escape, not
%     the peak. Longest unbroken saturation against final error, same run, same
%     plan: 0.00 s -> 0.86 m (fleet median), 1.36 s -> 5.24 m, 3.05 s -> 5.62 m,
%     6.38 s -> 127.66 m. Saturation duty cycle correlates 0.906 with flown error.
%
% Which is also why a longer final hold makes the error larger: it gives an
% already-open loop more time to coast, not more time to recover.
% Two constants with different jobs. v_track_max is where we start complaining,
% set just above the fastest speed measured good. v_track_target is what the
% RECOMMENDED duration aims for -- sizing the recommendation at v_track_max would
% land the user in the untested 7.25-10.74 band, so aim at a speed that was
% actually measured to track (7.16 m/s -> 0.295 m, 7.25 m/s -> 0.289 m). At
% v_track_target the recommendation reproduces the measured-good cases: N=40 wants
% 9.2 s where 9 s tracked, N=25 wants 6.2 s where 6 s tracked.
% Both are set with the other durations in setupParams, because the takeoff is
% sized against v_track_target before the trajectory exists. The evidence for the
% two numbers is the table above, which is why it lives here.

% Difference the planned positions rather than reading the stored velocity
% columns: this is the same metric the threshold above was measured with, and it
% cannot disagree with the positions the fleet is actually commanded to.
[t_speed_chk, i_speed_chk] = unique(time_vector, 'stable');
dt_speed_chk = diff(t_speed_chk);
traj_peak_speed = 0;
traj_peak_accel = 0;
dt_accel_chk = (dt_speed_chk(1:end-1) + dt_speed_chk(2:end)) / 2;
for uav_i = 1:N_uav
    step_xyz = diff(squeeze(trajectory_data(uav_i, 1:3, i_speed_chk))', 1, 1);
    vel_xyz = step_xyz ./ dt_speed_chk(:);
    traj_peak_speed = max(traj_peak_speed, max(vecnorm(vel_xyz, 2, 2)));

    % HORIZONTAL only, because that is the channel with the clamp: SatX/SatY limit
    % commanded lateral acceleration, whereas the vertical channel is a thrust
    % command with far more authority and much stiffer gains (Kp_z = 12 against
    % Kp_xy = 1). Including the climb would let the takeoff dominate a number whose
    % only recommended cure is a longer TRANSITION.
    accel_xy = diff(vel_xyz(:, 1:2), 1, 1) ./ dt_accel_chk(:);
    traj_peak_accel = max(traj_peak_accel, max(vecnorm(accel_xy, 2, 2)));
end

% Slowest transition that brings each peak inside its limit. Speed scales as 1/T and
% acceleration as 1/T^2 for the same geometry, so both inversions are exact -- and
% because the exponents differ, neither constraint implies the other. Take whichever
% binds: the app's auto-fit reads this one number, so the acceleration limit only
% reaches it if it is folded in here.
traj_min_transition_speed = transition_duration * traj_peak_speed / v_track_target;
traj_min_transition_accel = transition_duration * sqrt(traj_peak_accel / a_track_target);
traj_min_transition = max(traj_min_transition_speed, traj_min_transition_accel);

fprintf('Peak commanded speed: %.2f m/s (trackable: %.2f m/s)\n', ...
    traj_peak_speed, v_track_max);
fprintf('Peak commanded lateral acceleration: %.2f m/s^2 (trackable: %.2f m/s^2, clamp: %.2f)\n', ...
    traj_peak_accel, a_track_max, a_max);
if traj_peak_speed > v_track_max
    warning('setupParams:transitionTooFast', ...
        ['Transitions demand %.2f m/s, more than the %.2f m/s the fleet can ' ...
         'track. Drones will arrive late and may not converge at all. ' ...
         'Increase transition_duration to at least %.1f s (currently %.1f s), ' ...
         'or reduce formation_spacing.'], ...
        traj_peak_speed, v_track_max, ceil(traj_min_transition * 10) / 10, ...
        transition_duration);
end
if traj_peak_accel > a_track_max
    % Separate warning, not folded into the one above, because this one fires on
    % plans whose speed is perfectly legal -- which is the whole reason it exists.
    warning('setupParams:transitionTooSharp', ...
        ['Transitions demand %.2f m/s^2 of lateral acceleration, more than the ' ...
         '%.2f m/s^2 the fleet tracks, against a %.2f m/s^2 clamp. The peak ' ...
         'commanded SPEED (%.2f m/s) is inside its own limit, so this is not a ' ...
         'speed problem: PositionController saturates and drones that stay ' ...
         'saturated for more than about 1.5 s do not recover. Increase ' ...
         'transition_duration to at least %.1f s (currently %.1f s), or reduce ' ...
         'formation_spacing.'], ...
        traj_peak_accel, a_track_max, a_max, traj_peak_speed, ...
        ceil(traj_min_transition * 10) / 10, transition_duration);
end

% Rotation cannot be what trips the warning above, and that is by construction
% rather than by luck. formationHoldSamples caps the peak tangential speed at
% v_track_target (7.0), which is below v_track_max (8.0), so if traj_peak_speed
% exceeds the limit the sample responsible is in a transition, the takeoff or the
% descent -- never a rotating hold. That matters because the cure the warning
% recommends is a longer TRANSITION, and lengthening transitions does nothing
% whatever to a rotation: the spin rate is set by the hold. An uncapped rotation
% would have sent the app's auto-fit off stretching the wrong segment.
%
% In practice the ACCELERATION cap binds first and the speed cap is never reached,
% which makes the paragraph above a belt-and-braces argument rather than the
% operative one -- but it has to keep holding, because rotation_accel_frac is a
% budget someone may reasonably widen later.
%
% The same argument has to be made again for the acceleration warning, and it holds
% for the same reason: formationHoldSamples caps rotation at rotation_accel_target
% (1.8 m/s^2), which is below a_track_target (2.0) and so also below a_track_max
% (2.5). A rotating hold therefore cannot trip that warning either, and cannot drive
% traj_min_transition_accel above transition_duration -- both of which matter,
% because lengthening a transition does nothing to a rotation. Widening
% rotation_accel_frac past 2.0/a_max = 0.67 would break this, and the symptom would
% be the auto-fit stretching transitions to cure a spin.
%
% The flip side is that a hold too short for the requested turns is not an error at
% all -- it just sweeps less far. So say what it managed, per formation, rather than
% leaving the operator to infer it from watching the show.
rotation_min_hold = max(rotation_need_hold);
rotation_deg = rad2deg(rotation_sweep);
% any(rotation_active) as well as the flag: setupParams derives one from the other so the
% two cannot disagree there, but planShow is a public function taking a cfg struct, and a
% caller that sets the flag by hand with no formation actually rotating left rot_f empty
% and this block indexing swept(1) on nothing.
if formation_rotate && any(rotation_active)
    % Report over the ROTATING formations only. Averaging a still hold's 0 deg into the
    % range would read as a spin that nearly stopped -- "0-30 deg per hold" -- when what
    % it actually means is that the operator deliberately left that formation alone, so
    % the still ones are counted separately and named as a count.
    rot_f = find(rotation_active);
    swept = rotation_deg(rot_f);
    asked = abs(rotation_deg_request(rot_f));
    if all(abs(swept - swept(1)) < 0.05)
        sweep_str = sprintf('%.0f deg', swept(1));
    else
        sweep_str = sprintf('%.0f-%.0f deg', min(swept), max(swept));
    end
    if all(abs(asked - asked(1)) < 0.05)
        ask_str = sprintf('%.0f deg requested', asked(1));
    else
        ask_str = sprintf('%.0f-%.0f deg requested', min(asked), max(asked));
    end
    if numel(rot_f) == num_formations
        which_str = sprintf('all %d formations', num_formations);
    else
        which_str = sprintf('formation(s) %s of %d', ...
            strjoin(string(rot_f), ','), num_formations);
    end
    fprintf('Formation rotation: %s over a %.1f s hold, %s (%s).\n', ...
        sweep_str, hold_duration, which_str, ask_str);
    if rotation_min_hold > hold_duration + 1e-9
        % Name the bound that actually bit. Both are cured by a longer hold, so the
        % advice is the same either way, but the REASON is not interchangeable: a spin
        % stopped by acceleration is being held back to keep its drones ON their
        % circles, and in a show where the operator can also shrink the formation that
        % is worth saying rather than blaming a speed that was never reached.
        % The radius quoted has to come from a ROTATING formation. rotation_radius is
        % geometric and is recorded for every hold whether it spins or not, so a plain
        % max() over the whole fleet could name the extent of a formation the operator
        % deliberately left still -- and then blame the sweep on a circle nobody flew.
        rot_radius_max = max(rotation_radius(rot_f));
        if any(strcmp(rotation_limit(rot_f), 'acceleration'))
            why = sprintf(['Acceleration-limited: holding a circle of radius %.1f m ' ...
                           'needs a centripetal\n  pull for the whole hold, and the ' ...
                           'budget is %.1f m/s^2 (%.0f%% of a_max = %.1f).'], ...
                rot_radius_max, rotation_accel_target, ...
                rotation_accel_frac * 100, a_max);
        else
            why = sprintf(['Speed-limited: the widest rotating formation spins at %.1f m ' ...
                           'from its axis,\n  so tangential speed reaches %.1f m/s.'], ...
                rot_radius_max, v_track_target);
        end
        % Quote the request that is actually binding -- the one needing the longest hold
        % -- rather than the largest angle asked for. With per-formation angles those are
        % not the same formation: a 90 deg request on a wide Sphere needs longer than a
        % 180 deg one on a tight Grid.
        [~, i_bind] = max(rotation_need_hold);
        fprintf(['  %s\n  The %.0f deg asked of formation %d would need a %.1f s hold. ' ...
                 'Raise Hold (s) for more\n  sweep, ask for less, or reduce Spacing to ' ...
                 'shrink the radius.\n'], ...
            why, abs(rotation_deg_request(i_bind)), i_bind, ...
            ceil(rotation_min_hold * 10) / 10);
    end
end

% Pre-compute lighting commands timeline [num_total_samples x N_uav x 6]
% Format: [targetR, targetG, targetB, effectType, duration, intensity]
lighting_timeline = zeros(N_uav, 6, num_total_samples);
for s = 1:num_total_samples
    t_s = time_vector(s);
    % Which formation are we on? Look the segment up in the timeline this
    % function already built, rather than re-deriving hold starts: the old
    % (f-1)*(hold_duration + transition_duration) dropped the takeoff climb
    % and so changed colour takeoff_duration seconds too early.
    seg = find(t_s >= timeline_times, 1, 'last');
    if isempty(seg)
        form_idx = 1;               % still climbing to the first formation
    elseif timeline_types(seg) == 0
        form_idx = (seg + 1) / 2;   % holds sit at odd segment indices
    else
        form_idx = seg / 2;         % transitions at even ones
    end
    form_idx = max(1, min(form_idx, num_formations));
    % Colour keys off the formation *type*, not its position in the sequence:
    % Grid -> Circle -> Grid is red, green, red, not red, green, blue.
    cidx = min(formation_sequence(form_idx), size(lighting_colors, 1));
    for uav_i = 1:N_uav
        lighting_timeline(uav_i, 1:3, s) = lighting_colors(cidx,:);
        lighting_timeline(uav_i, 4, s) = 1; % fade effect
        lighting_timeline(uav_i, 5, s) = 1; % 1 second duration
        lighting_timeline(uav_i, 6, s) = 1; % full intensity
    end
end

disp(['Show plan computed: ' num2str(num_total_samples) ' samples, ' ...
    num2str(show_duration) ' seconds']);

%% Flatten the lighting timeline for the model
% Column-major order so a Simulink Reshape produces the correct [N_uav x 6].
light_ws_data = zeros(num_total_samples, N_uav*6);
for s = 1:num_total_samples
    light_ws_data(s,:) = reshape(lighting_timeline(:,:,s), 1, []);
end

% NO timeseries objects are built here any more. This section used to wrap the trajectory, the
% lighting and a show-command sequence into three timeseries "for From Workspace blocks in
% MissionPlanner" -- and there is no MissionPlanner subsystem in the model. The two From Workspace
% blocks that do exist read traj_show_ts (built in setupParams from the uploaded keyframes) and
% rtk_measured_obs, so none of the three was ever a source for anything, and none of them was in
% the export set below. The trajectory one was also the expensive one: an extra
% num_total_samples-by-N_uav*7 copy plus a timeseries wrapper, which is ~8 MB of pure waste at a
% fleet of 500. light_ws_data survives because it IS exported.

% ---- pack -------------------------------------------------------------------------
% The export set, derived across every consumer rather than chosen: the model via
% Simulink.findVars, the app's evalin reads, and DroneShowExample. Anything computed
% above and NOT here is an intermediate and stays a local, which is the point of the move.
plan = struct( ...
    'all_formations',        all_formations, ...
    'assignment_order',      assignment_order, ...
    'init_positions',        init_positions, ...
    'land_duration',         land_duration, ...
    'land_end_time',         land_end_time, ...
    'land_ground_pos',       land_ground_pos, ...
    'land_start_time',       land_start_time, ...
    'land_total_duration',   land_total_duration, ...
    'light_ws_data',         light_ws_data, ...
    'lighting_timeline',     lighting_timeline, ...
    'max_samples',           max_samples, ...
    'min_sep_achieved',      min_sep_achieved, ...
    'num_total_samples',     num_total_samples, ...
    'rotation_limit',        {rotation_limit}, ...
    'rotation_min_hold',     rotation_min_hold, ...
    'rotation_need_hold',    rotation_need_hold, ...
    'rotation_peak_accel',   rotation_peak_accel, ...
    'rotation_peak_speed',   rotation_peak_speed, ...
    'rotation_radius',       rotation_radius, ...
    'rotation_sweep',        rotation_sweep, ...
    'show_duration',         show_duration, ...
    'show_flight_duration',  show_flight_duration, ...
    'takeoff_dist',          takeoff_dist, ...
    'takeoff_duration',      takeoff_duration, ...
    'time_vector',           time_vector, ...
    'timeline_times',        timeline_times, ...
    'timeline_types',        timeline_types, ...
    'traj_min_transition',   traj_min_transition, ...
    'traj_peak_speed',       traj_peak_speed, ...
    'traj_peak_accel',       traj_peak_accel, ...
    'trajectory_data',       trajectory_data);

% radius is assigned ONLY by the Circle and Sphere branches, so a Grid-or-text-only
% show never had it in the base workspace either. Kept conditional rather than
% zero-filled, because callers read it back to report the ring geometry and a
% fabricated 0 would read as a real measurement of a formation that has no radius.
if exist('radius', 'var')
    plan.radius = radius;
end
end
