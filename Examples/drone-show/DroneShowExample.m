%[text] # Multi-UAV Drone Light Show
%[text] A fleet of quadrotors takes off, flies a sequence of formations, and lands — with the whole mission planned on the ground, uploaded over MAVLink, and flown from what actually arrived on board. This script is the scripted route through that pipeline: set the show with the controls below, then run the sections in order and read the report each one produces. `DroneLightShowApp` is the interactive route through the same code and the same model; nothing here reimplements it.
%[text] **The pipeline has four stages, and each one is a separate file so that each can be checked on its own.**
%[text] - `setupParams` — every constant: airframe, gains, sample times, reference location, RTK and radio configuration. It also holds the guards, so anything assigned in the base workspace before it runs wins. That is the only reason the controls below work. \
%[text] - `planShow` — geometry to trajectory. Formation point clouds, drone-to-slot assignment by `matchpairs`, minimum-jerk transitions through `minjerkpolytraj`, keyframing, takeoff and landing. Returns one plan struct. \
%[text] - `packShowUpload` — the plan to the wire. Keyframe selection, MAVLink packetisation, and how long the upload occupies the radio. \
%[text] - `MultiUAV_DroneShow.slx` — the flown system: base station, RTK engine, radio channel with latency and loss, the drone fleet with its INS and position controllers, and a Stateflow supervisor that sequences IDLE to LANDED. \
%[text] The two planning functions are pure: same inputs, same plan, no model needed. So the show can be sized, rejected and re-sized before anything is simulated, which is what the first half of this script does.

%%
%[text] ## 1. Choose the show
%[text] Nine settings, a subset of the app's twenty-seven. The rest keep their `setupParams` defaults, and they are cleared first rather than left alone: `setupParams` guards every overridable parameter with `~exist`, so a value left in the base workspace by an earlier app session — a degraded drone in `rtk_deny_mask`, a packet loss rate, a stale `num_formations` from a different sequence — would silently survive into this run and the report would describe a show nobody asked for.

clear a_max abort_request comm_jitter comm_latency comm_timeout custom_formations ...
      d_min gnss_deny_mask lighting_colors_override loss_seed num_formations ...
      packet_loss_rate rotation_deg_request rotation_turns rtk_deny_mask ...
      rtk_inject_enable uav_loss_rate v_max

N_uav = 20; %[control:slider:0001]{"position":[9,11]}
formation_sequence = [1 2 3 2 1]; %[control:dropdown:0002]{"position":[22,33]}
formation_text = "MATLAB"; %[control:editfield:0003]{"position":[18,26]}
formation_spacing = 5; %[control:slider:0004]{"position":[21,22]}
show_altitude = -10; %[control:slider:0005]{"position":[17,20]}
hold_duration = 10; %[control:slider:0006]{"position":[17,19]}
transition_duration = 15; %[control:slider:0007]{"position":[23,25]}
formation_rotate = false; %[control:checkbox:0008]{"position":[20,25]}
traj_source = 2; %[control:dropdown:0009]{"position":[15,16]}

% Derived, not offered: the delivery path decides both. DroneLightShowApp couples them
% exactly this way, and the combination it never produces -- an upload during a skipped
% pre-flight -- is one the supervisor runs and setupParams then mis-reports.
skip_preflight = traj_source == 1;
upload_request = traj_source == 2;

%[text] `show_altitude` is negative because the model works in NED: down is positive, so up is negative. `formation_text` only matters when the sequence contains a 4. `traj_source` chooses whether the fleet flies the waypoints straight out of the workspace or the ones that arrived in its onboard buffer over the radio — the data is identical when the link is healthy, so a disagreement between the two is a finding, not a setting.
%[text] It is a single control rather than three because the delivery path is one decision. Source 2 is reception-driven: it needs the upload to fill the buffer, so it runs the whole pre-flight. Source 1 reads the workspace, so it needs neither — and asking for a skipped pre-flight *and* an upload gives you both a supervisor that spends nineteen seconds uploading and a `setupParams` that has already reported the pre-flight as skipped and sized the simulation accordingly. The show then enters twenty-one seconds late and is still flying when the solver stops, so its last twenty-one seconds — the whole landing, and the formation before it — never happen. Deriving both from one control is what makes that unreachable.

%%
%[text] ## 2. Plan the show
%[text] One call. `setupParams` fills in every constant, hands the show parameters to `planShow`, hands the resulting plan to `packShowUpload`, and unpacks both back into the workspace under the names the model's block parameters reference. Its own report comes out as it goes: separation, peak commanded speed, packet count, and the simulation budget it just sized.

setupParams

%[text] Read the four lines it printed as a pre-flight check on the *plan*, before a single solver step has run. Separation and speed are the two ways a geometrically sensible show becomes unflyable, and both are decided entirely by the numbers chosen above.

%%
%[text] ## 3. What the planner decided
%[text] The plan is not a rescaling of the request — it is the result of resolving the request against the airframe. Two of the rows below are resolved values rather than requested ones, and reading them is how you find out whether the request survived.
%[text] `takeoff_duration` is a floor, not a duration: `planShow` raises it until the climb from the pad to the first formation fits inside a speed the fleet can actually track. At a small fleet the floor already fits and the number does not move — it moves when the first formation is far enough from the pad, which happens as the fleet grows and the formation spreads. `traj_min_transition` is the shortest transition that keeps the plan inside the band the fleet can track, and on this route it is *advisory*: `planShow` warns and then plans exactly what was asked for. The auto-fit that lengthens transitions to satisfy it belongs to the app, not to the planner, so a script that ignores this row will happily simulate a show the planner already objected to. Transitions demand formation span divided by time allowed, so span grows with the fleet while the requested duration never does. That is why a show that is perfect at twelve drones tears itself apart at forty with the same settings — and why the row to check is this one, not the speed.
%[text] **Two limits go into that row, and the one that binds is usually not the speed.** A transition has to stay inside a trackable speed *and* inside the lateral acceleration `PositionController` is allowed to command, and the two scale differently with the time allowed — speed as 1/T, acceleration as 1/T². Halving a transition doubles the speed and quadruples the acceleration, so every route has a band of durations that are legal on speed and illegal on acceleration. It is not a narrow band: 36 drones over `Grid → Circle → Grid` at 8 s transitions asks 6.97 m/s of an 8.00 m/s limit, comfortably legal, while asking 2.98 m/s² of a 3.00 m/s² clamp — and one drone ends the show 174 m from where it was told to be. `traj_min_transition` is the larger of the two requirements for that reason.

% Which radius law the Circle and Sphere took, COMPUTED rather than asserted. A single ring
% is N*spacing/(2*pi) -- linear in the fleet -- and it is what every fleet up to about 220
% drones actually gets. Only past formation_radius_max does planShow switch to concentric
% rings sized by area, where the extent grows as sqrt(N). Stating "planned by area" flatly
% would be wrong for the default fleet, and wrong in the direction that hides the crossover.
ringRadius = N_uav * formation_spacing / (2 * pi);
if ringRadius > formation_radius_max
    radiusLaw = sprintf('by area: a single ring would be %.0f m, past the %.0f m limit', ...
                        ringRadius, formation_radius_max);
else
    radiusLaw = sprintf('single ring, %.1f m radius (area law starts past %.0f m)', ...
                        ringRadius, formation_radius_max);
end

planReport = table( ...
    ["drones"; "formations in the sequence"; "planned trajectory samples"; ...
     "minimum separation (m)"; "peak commanded speed (m/s)"; ...
     "peak lateral acceleration (m/s²)"; ...
     "shortest safe transition (s)"; "takeoff climb (s)"; ...
     "show incl. landing (s)"; "total simulation (s)"], ...
    [N_uav; num_formations; num_total_samples; min_sep_achieved; traj_peak_speed; ...
     traj_peak_accel; ...
     traj_min_transition; takeoff_duration; show_duration; total_sim_duration], ...
    [string(radiusLaw); ...
     string(mat2str(formation_sequence)); ...
     string(sprintf('%d per transition leg, %s per hold', ...
                    num_samples_per_seg, mat2str(num_samples_per_hold))); ...
     string(sprintf('%.2f m required', d_min)); ...
     string(sprintf('%.2f m/s trackable', v_track_max)); ...
     string(sprintf('%.2f m/s² trackable, %.2f m/s² clamp', a_track_max, a_max)); ...
     string(sprintf('%.1f s requested', transition_duration)); ...
     string(sprintf('%.1f s floor, then sized to climb at %.1f m/s', ...
                    show_cfg.takeoff_duration, climb_speed)); ...
     string(sprintf('%.1f s of landing inside it', land_duration + land_settle)); ...
     string(sprintf('pre-flight %.1f s + show', preflight_duration))], ...
    'VariableNames', ["Quantity", "Value", "Against"])

%[text] Separation is measured on the *planned* samples, so it is an upper bound on what gets flown: the fleet tracks a command with a finite error, and closed-loop separation is always a little worse. The assignment is what makes the bound meaningful — `planShow` matches drones to slots on **squared** distance rather than distance, which guarantees separation through a transition stays at or above the tighter of the two formations' own closest pair, divided by the square root of two. Minimising plain distance gives no bound at all.
%[text] Note *the formation's own closest pair*, not `formation_spacing`. They are the same thing only for a Grid. Circle, Sphere and Text formations are point clouds sampled down to the fleet size, so their closest pair is whatever the sampling produced — `planShow` prints it per formation — and it is usually tighter than the spacing requested. Section 8 computes the bound from the formations actually planned rather than from the setting, which is the difference between a check and a decoration.

%%
%[text] ## 4. The formations, as planned
%[text] Each panel is one hold in the sequence, seen from above and behind, with every drone in the colour it keeps for the whole show. The Sphere and the Text formations are the interesting ones: both come from a point cloud that is sampled down to exactly the fleet size, so the same word flown by twenty drones and by a hundred is not the same formation — it is the same glyph at two resolutions.

formationNames = ["Grid", "Circle", "Sphere", "Text", "Custom"];
droneColours = lines(N_uav);
tiledlayout("flow")
for f = 1:num_formations
    nexttile
    P = all_formations(:, :, f);
    scatter3(P(:, 2), P(:, 1), -P(:, 3), 24, droneColours, "filled")
    axis equal
    grid on
    view(35, 20)
    xlabel("East (m)")
    ylabel("North (m)")
    zlabel("Up (m)")
    title(sprintf("%d: %s", f, formationNames(min(formation_sequence(f), 5))))
end

%%
%[text] ## 5. What goes over the radio
%[text] The fleet does not receive the trajectory — it receives keyframes, and interpolates. `packShowUpload` chooses them, and the choice is not uniform in time: a straight leg is safe with a handful of keyframes, but a rotating hold is a circle, and sampling a circle by the clock lets the chord sag away from the arc. So curved segments are keyframed by swept **angle** instead, which is why the spacing below is not constant when rotation is on.
%[text] The second argument to `packShowUpload` is the plan, not the configuration, and that is load-bearing rather than tidy: `takeoff_duration` enters the planner as a floor and leaves it raised, so sizing the upload from the requested value would bill the radio for a takeoff the fleet is not flying.

uploadReport = table( ...
    ["keyframes per drone"; "packets total"; "bytes per packet"; ...
     "link bandwidth (bytes/tick)"; "upload occupies the radio for (s)"], ...
    [num_waypoints; total_packets; mavlink_packet_size; mavlink_bandwidth; upload_duration], ...
    'VariableNames', ["Quantity", "Value"])

%%
%[text] The gap between consecutive keyframes, across the show. Flat stretches are straight transitions and static holds; anything dense is a curve being resolved finely enough that the flown chord stays inside the sag budget.

stairs(upload_times(1:end-1), diff(upload_times), "LineWidth", 1.2)
grid on
xlabel("Show time (s)")
ylabel("Gap to next keyframe (s)")
title(sprintf("Keyframe spacing — %d keyframes over %.1f s", num_waypoints, upload_times(end)))

%%
%[text] ## 6. Fly it
%[text] `total_sim_duration`, not `show_duration`: the latter is show-relative, so stopping there ends the run while the fleet is still in the air. The show finishes with a landing, and this is what lets it happen. On the full-communication path the pre-flight window is a *bound* rather than a schedule, and it is the bound on **two** things happening, not one. The mission has to finish uploading, and the base station's RTK engine has to report a fixed L1 solution — about thirteen seconds of wide-lane convergence plus a dwell, at a seed-dependent instant. `setupParams` combines them with `max`, not `+`, because they overlap: the operator uploads while the base surveys in, exactly as on a real pad.
%[text] So which one binds moves with the settings, and the line `setupParams` just printed says which. At the defaults the upload chain (lock 2 + upload ≈ 22 + arm 2) sets the trajectory anchor, while the RTK *budget* of 28 s — a tail-covering bound, not an expectation — sets the stop time. Shrink the fleet or the keyframe count and the upload stops mattering; the RTK gate then binds and the anchor stops being predictable. Paying for the worst case costs an idle tail; underpaying truncates the show.

out = sim("MultiUAV_DroneShow", "StopTime", num2str(total_sim_duration))

%%
%[text] ## 7. The mission, phase by phase
%[text] The Stateflow supervisor sequences the mission, and the trace below is the honest account of what the run did — including the wait. On the full path the fleet sits in UPLOAD and then ARMED under a no-fly assertion, and it is released by the *later* of the upload finishing and the RTK gate opening. At the defaults that is the upload: SHOW is entered around 23 s, past the roughly 16 s the RTK gate is measured to need. Read the ARMED stretch as the slack between the two — a long one means the upload finished early and the fleet is waiting on the base station.
%[text] The gate's own instant is deliberately not read off a logged signal here. `rtkFixed` is the obvious-looking tap and it is the wrong one: it is derived from correction *age*, and corrections are fresh from the start, so it reads 1 from `t = 0` regardless of whether the carrier-phase solution has converged. The honest measure is when the navigation error settles onto the 2 cm floor, which is what the fleet actually feels. `setupParams` carries the resulting figure as `rtk_lock_expected`, measured from the cascade rather than assumed, and keeps the separate, larger `rtk_lock_budget` for sizing — because the first-fix epoch is seed-dependent and a bound has to cover the tail, not the median.

phaseNames = ["Idle", "Upload", "Armed", "Show", "Landed", "Landing", "Upload FAILED"];
phase = round(squeeze(out.logPhase.Data));
phaseTime = out.logPhase.Time;
stairs(phaseTime, phase, "LineWidth", 1.4)
yticks(0:numel(phaseNames)-1)
yticklabels(phaseNames)
ylim([-0.5, max(phase) + 0.5])
grid on
xlabel("Simulation time (s)")
ylabel("Supervisor state")
title("Mission phases")

%%
%[text] Where the show actually started, recovered from the trace rather than assumed — and next to it the instant the *plan's* clock starts, which is a different quantity arrived at a different way.
%[text] On the onboard path the fleet replays its buffer from SHOW entry, so the plan's clock is whatever the supervisor decided and the only way to know it is to read it back. On the workspace path the trajectory is a `From Workspace` block indexed by absolute simulation time and shifted by `preShowDelay` — so the plan's clock was fixed before the run started and the supervisor has no say in it. Section 8 compares a flight against a plan, and getting this wrong is not a small error: it is the whole pre-flight, and it appears as tens of metres of "tracking error" that is really a time offset.
%[text] On both paths this script can produce, the two agree — a skipped pre-flight makes `preShowDelay` the GNSS-lock wait, which is also when the supervisor enters SHOW. The branch is here anyway, because they agree by coincidence rather than by construction: `setupParams` will happily shift the workspace trajectory by a *full* pre-flight budget if some other caller asks for source 1 without skipping it, and then the two are tens of seconds apart.
%[text] `anchorGap` is the check on that, and it is only a check on the workspace path — on the onboard path the anchor *is* SHOW entry, so the difference is zero by construction and confirms nothing. `budgetSlack` is the number that says something on both: `preShowDelay` is the anchor `setupParams` **sized** from the pre-flight budget, `showEntry` is when the supervisor **actually** released the fleet, and the supervisor sequences on events rather than on the budget. At the defaults the budget over-provisions by a few seconds — the upload finishes a little early — which is exactly the right sign for a bound. A *negative* slack would mean the sizing underpaid and the trajectory clock started before the fleet was allowed to fly.

% Checked rather than assumed, because the one state this script NAMES above and then has no
% other handling for is a state in which SHOW never happens: the mission upload gives up and
% the supervisor ends in UPLOAD FAILED. An empty showEntry does not fail here -- it makes
% trajAnchor empty, then relTime and inShow empty, and section 8 dies building a table out of
% empties, four sections and tens of lines from the cause.
showEntryIdx = find(phase == 3, 1);
if isempty(showEntryIdx)
    endedIn = phaseNames(min(max(phase(end), 0), numel(phaseNames) - 1) + 1);
    error("DroneShowExample:noShow", ...
        "The supervisor never entered SHOW — the run ended in %s, so there is no " + ...
        "flight to compare against the plan. On the MAVLink path this normally means " + ...
        "the upload did not complete: check the upload report in section 5 and " + ...
        "packet_loss_rate, or set traj_source = 1 to bypass the upload entirely.", endedIn);
end
showEntry = phaseTime(showEntryIdx)
if traj_source == 1
    trajAnchor = preShowDelay;   % the From Workspace time shift; fixed before the run
else
    trajAnchor = showEntry;      % the onboard buffer replays from SHOW entry
end
anchorGap = trajAnchor - showEntry
budgetSlack = preShowDelay - showEntry

%%
%[text] ## 8. Flown against planned
%[text] The plan is a command, and the difference between a command and a flight is the whole point of simulating it. The error below includes everything the plan cannot know: controller lag on the transitions, the INS estimate the position loop is actually closing on, and — on the onboard path — the keyframe interpolation the fleet is doing for itself between the points it received.

pos = out.fleetPositions.Data;                 % [N_uav x 3 x T], NED
simTime = out.fleetPositions.Time;
[planTime, planIdx] = unique(time_vector(1:num_total_samples), "stable");

% unique() is required -- interp1 rejects a repeated sample point -- but it DROPS samples, and
% dropping them silently is how a plan defect turns into a clean-looking plot. A rotating hold
% emits its samples at hold boundaries that can share a timestamp, and every one collapsed here
% is a plan point this comparison never looks at. Report the count instead of absorbing it.
collapsedSamples = num_total_samples - numel(planTime)
if collapsedSamples > 0
    warning("%d of %d plan samples share a timestamp with an earlier one and are " + ...
            "excluded from the comparison below. The interpolated reference draws a " + ...
            "chord across each collapsed instant, so error reported near one is not " + ...
            "attributable to the flight.", collapsedSamples, num_total_samples);
end
planXYZ = permute(trajectory_data(:, 1:3, 1:num_total_samples), [3 1 2]);   % [S x N x 3]

relTime = simTime - trajAnchor;
inShow = relTime >= 0 & relTime <= planTime(end);
trackErr = zeros(sum(inShow), N_uav);
for k = 1:N_uav
    ref = interp1(planTime, squeeze(planXYZ(planIdx, k, :)), relTime(inShow));
    flown = squeeze(pos(k, :, inShow))';
    trackErr(:, k) = vecnorm(flown - ref, 2, 2);
end
plot(relTime(inShow), max(trackErr, [], 2), "LineWidth", 1.2)
hold on
plot(relTime(inShow), mean(trackErr, 2), "LineWidth", 1.2)
hold off
grid on
legend(["worst drone", "fleet mean"], "Location", "best")
xlabel("Show time (s)")
ylabel("Distance from commanded position (m)")
title("Tracking error over the show")

%%
%[text] Separation is the constraint that has to hold on the *flight*, not on the plan, so it is measured again on what was flown. The planned bound above and the flown figure below should differ by tracking error and no more — a flown separation far under the plan means the assignment or the transition timing is wrong, not that the fleet is noisy.

flownIdx = find(inShow);
flownIdx = flownIdx(1:max(1, round(0.1 / median(diff(simTime)))):end);
% NaN, not Inf, at a fleet of one. `min` over an empty set of pairs is `+Inf`, which then
% prints as a separation of Inf metres "against 2.00 m required" and as a bound of
% "Inf m / sqrt(2)" -- three rows of nonsense that read as a spectacularly safe show.
% A single drone has no pair, so the honest value is not-a-number and the honest comparison
% is none at all.
flownSep  = NaN;
staticSep = NaN;
if N_uav > 1
    flownSep  = inf;
    staticSep = inf;
    for i = flownIdx'
        flownSep = min(flownSep, min(pdist(pos(:, :, i))));
    end
    for f = 1:num_formations
        staticSep = min(staticSep, min(pdist(all_formations(:, :, f))));
    end
    sepAgainst   = string(sprintf('%.2f m required', d_min));
    boundAgainst = string(sprintf('bound is %.2f m: tightest static formation %.2f m / sqrt(2)', ...
                                  staticSep / sqrt(2), staticSep));
    plannedSep   = min_sep_achieved;
else
    sepAgainst   = "not applicable: one drone has no pair";
    boundAgainst = "not applicable: one drone has no pair";
    plannedSep   = NaN;
end
flightReport = table( ...
    ["worst tracking error (m)"; "mean tracking error (m)"; ...
     "minimum separation flown (m)"; "minimum separation planned (m)"], ...
    [max(trackErr(:)); mean(trackErr(:)); flownSep; plannedSep], ...
    ["over the whole show, worst drone"; "over the whole show, all drones"; ...
     sepAgainst; boundAgainst], ...
    'VariableNames', ["Quantity", "Value", "Against"])

%%
%[text] ## 9. Replay in the 3D viewer
%[text] The flight, played back through `uavScenario` with a quadrotor mesh per drone. The axes are derived from the plan rather than fixed: a hard-coded box is correct for exactly one fleet size and one formation, and a text billboard is tall enough to leave the fleet entirely outside a box sized for a grid.

planPoints = reshape(permute(trajectory_data(:, 1:3, 1:num_total_samples), [1 3 2]), [], 3);
horizExtent = max(max(abs(planPoints(:, 1:2))));
vertExtent = max(-planPoints(:, 3));
pad = max(2, 0.1 * horizExtent);

frameStep = max(1, round(0.1 / median(diff(simTime))));
frames = ceil(size(pos, 3) / frameStep) + N_uav + 10;
replay = uavScenario("UpdateRate", 10, "ReferenceLocation", ref_lla, "MaxNumFrames", frames);
platforms = cell(1, N_uav);
for k = 1:N_uav
    platforms{k} = uavPlatform("UAV" + k, replay, "InitialPosition", init_positions(k, :));
    updateMesh(platforms{k}, "quadrotor", {1}, droneColours(k, :), [0 0 0], [1 0 0 0]);
end

ax = show3D(replay);
xlim(ax, [-horizExtent - pad, horizExtent + pad])
ylim(ax, [-horizExtent - pad, horizExtent + pad])
zlim(ax, [0, vertExtent + pad])
view(ax, 45, 30)
title(ax, "Drone light show replay")

setup(replay)
for i = 1:frameStep:size(pos, 3)
    for k = 1:N_uav
        p = pos(k, :, i);
        move(platforms{k}, [p(1) p(2) p(3), 0 0 0, 0 0 0, 1 0 0 0, 0 0 0]);
    end
    advance(replay);
    show3D(replay, "Parent", ax, "FastUpdate", true);
    drawnow limitrate
end

%%
%[text] ## Where to go next
%[text] - **Break the show on purpose.** Push `transition_duration` below the shortest safe transition in section 3 and re-run. The planner warns — advisory only on this route — and section 8 shows what the warning was protecting. The tracking error is *not* the interesting number: at 6 s it stayed at a mean of 0.196 m with spikes to 5.66 m, and the spikes sit at the transitions, so the fleet is late into each formation rather than unstable. The number that matters is the row below it. Minimum separation flown fell to **0.809 m against the 2.00 m `d_min` the plan is required to respect** — the plan's own static formations are still legal, and the flight through them is not. That is the failure mode a too-short transition actually produces: not a wobble, a violated safety constraint, visible only because section 8 measures separation on the flown positions instead of trusting the plan. \
%[text] - **Change the source.** Run once with `traj_source` at 2 and once at 1 and compare section 8. The waypoints are identical, and the flights are not: worst-drone error was 0.61 m from the onboard buffer against 0.37 m from the workspace. Attribute that gap carefully — it is *not* only the radio. Source 1 also skips the pre-flight, so the fleet never waits for the RTK fix and flies with the injected GNSS error gated off, on the bare 0.02 m INS floor. Isolating the radio alone means holding the pre-flight fixed and varying `comm_latency`, `packet_loss_rate` or `comm_jitter` instead. All three are on section 1's `clear` list, so assign one *after* that `clear` and `setupParams` will honour it. \
%[text] - **Turn rotation on**, and read section 8 sceptically. Section 5 changes shape, because a rotating hold is keyframed by swept angle rather than by the clock. Rotation is bounded by *acceleration*, not speed — holding a circle needs a centripetal term continuously, where a straight leg does not — so the planner caps the sweep it will ask for. That cap is not a guarantee about the flight. At the 20-drone default with rotation on, one drone of twenty leaves formation during the final rotating hold and ends 141 m from its pad, while the other nineteen stay inside 1.1 m; flown separation drops to 2.31 m against the 3.52 m the assignment bound promises. This is a **known open defect on the rotating path**, not something this script introduces: the plan is bit-identical to the pre-refactor planner, both delivery paths reproduce it, and the uploaded keyframes are clean, which puts it in the plan geometry or the position loop. The existing rotation test flies 6 drones and stops asserting at the end of the last hold, so the landing out of a rotating hold is untested ground. Treat rotation at large fleets as a demonstration of what section 8 is *for*. \
%[text] - **Open the app.** `DroneLightShowApp` drives the same `setupParams`, the same two planning functions and the same model, and adds the things a script cannot show: live telemetry decoded off MAVLink, per-drone GNSS degradation, and an abort. \

%[appendix]{"version":"1.0"}
%---
%[control:slider:0001]
%   data: {"defaultValue":20,"label":"Fleet size","max":60,"min":1,"run":"Section","runOn":"ValueChanged","step":1}
%---
%[control:dropdown:0002]
%   data: {"defaultValue":"[1 2 3 2 1]","itemLabels":["Grid, Circle, Sphere, Circle, Grid","Grid, Circle, Grid (short)","Text, Grid, Text","one of each"],"items":["[1 2 3 2 1]","[1 2 1]","[4 1 4]","[1 2 3 4]"],"label":"Formation sequence","run":"Section"}
%---
%[control:editfield:0003]
%   data: {"defaultValue":"MATLAB","label":"Text to fly","run":"Section","valueType":"Text"}
%---
%[control:slider:0004]
%   data: {"defaultValue":5,"label":"Formation spacing (m)","max":15,"min":2,"run":"Section","runOn":"ValueChanged","step":0.5}
%---
%[control:slider:0005]
%   data: {"defaultValue":-10,"label":"Show altitude (m, NED)","max":-5,"min":-40,"run":"Section","runOn":"ValueChanged","step":1}
%---
%[control:slider:0006]
%   data: {"defaultValue":10,"label":"Hold duration (s)","max":25,"min":4,"run":"Section","runOn":"ValueChanged","step":1}
%---
%[control:slider:0007]
%   data: {"defaultValue":15,"label":"Transition duration (s)","max":40,"min":5,"run":"Section","runOn":"ValueChanged","step":1}
%---
%[control:checkbox:0008]
%   data: {"defaultValue":false,"label":"Rotate the formations","run":"Section"}
%---
%[control:dropdown:0009]
%   data: {"defaultValue":"2","itemLabels":["1 - straight from the workspace (quick)","2 - from the onboard buffer over MAVLink"],"items":["1","2"],"label":"Trajectory source","run":"Section"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
