%[text] # Battery Fast Charging: Strategy Comparison and Optimization
%[text] Compare charging strategies for a lithium-ion battery using the Single Particle Model (SPM) from Simscape Battery. This example evaluates constant current-constant voltage (CC-CV), multi-stage, and optimized charging profiles for speed, safety, and efficiency.

%%
%[text] ## Requirements
%[text] - Simulink
%[text] - Simscape
%[text] - Simscape Battery
%[text] - Stateflow
%[text] - Optimization Toolbox
%[text] - Parallel Computing Toolbox \

%%
%[text] ## Setup
%[text] Load model parameters and configure the simulation environment. The nominal cell capacity is derived from the SPM block's electrochemical parameters using:
%[text] $Q_{nom} = \\min(Q_{anode}, Q_{cathode}), \\quad Q_i = \\frac{\\varepsilon_{s,i} \\cdot L_i \\cdot A \\cdot c_{max,i} \\cdot (\\theta_{max,i} - \\theta_{min,i}) \\cdot F}{3600}$
%[text] where $\\varepsilon_s$ is active material volume fraction, $L$ is electrode thickness, $A$ is plate area, $c_{max}$ is maximum solid-phase concentration, $\\theta_{max} - \\theta_{min}$ is the usable stoichiometry range, and $F = 96485$ C/mol is the Faraday constant. This ensures the coulomb-counting SOC estimator stays in sync with the battery model.

clear; clc; close all;

modelName = 'BatteryFastCharging';
load_system(modelName);
setupParams;

%%
%[text] ## Baseline: CC-CV Charging
%[text] The standard constant current-constant voltage (CC-CV) strategy charges at 1C until the voltage limit is reached, then holds voltage constant while current tapers. Charging terminates when SOC reaches the target (90%). This is the industry-standard method and serves as our performance baseline.
%[text] 
%[text] **Constraints enforced:** Voltage limit (4.2 V, via CV phase), SOC termination target.

in_cccv = Simulink.SimulationInput(modelName);
in_cccv = in_cccv.setVariable('active_controller', 1);
in_cccv = in_cccv.setVariable('SOC_init', SOC_init);
in_cccv = in_cccv.setModelParameter('StopTime', num2str(t_sim));
out_cccv = sim(in_cccv);

%%
%[text] Extract the baseline results from logged signals.

cccv_results = extractResults(out_cccv, SOC_target);

%%
%[text] ## Multi-Stage Charging Sweep
%[text] Multi-stage strategies apply decreasing current levels as SOC increases. Higher currents at low SOC exploit the battery's ability to accept charge quickly, while lower currents near full charge reduce stress. We sweep four profiles to explore this trade-off.
%[text] 
%[text] **Constraints enforced:** Voltage limit (4.2 V, triggers CV phase), temperature limit (50°C, triggers shutdown), anode concentration limit (plating, triggers shutdown), SOC termination target.

n_sweep = size(sweep_profiles, 1);
sweep_results = cell(n_sweep, 1);

for k = 1:n_sweep
    currents_k = sweep_profiles{k,1} * Q_nom;
    thresholds_k = sweep_profiles{k,2};

    in_ms = Simulink.SimulationInput(modelName);
    in_ms = in_ms.setVariable('active_controller', 2);
    in_ms = in_ms.setVariable('ms_currents', currents_k);
    in_ms = in_ms.setVariable('ms_soc_thresholds', thresholds_k);
    in_ms = in_ms.setVariable('SOC_init', SOC_init);
    in_ms = in_ms.setModelParameter('StopTime', num2str(t_sim));
    out_ms = sim(in_ms);

    sweep_results{k} = extractResults(out_ms, SOC_target);
    sweep_results{k}.label = sweep_labels{k};
end

%%
%[text] ## Optimization-Based Charging Profile (Surrogate Optimization)
%[text] Formulate the charging task as a constrained optimization problem using `surrogateopt`. A single continuous simulation evaluates each candidate profile, ensuring physically exact state propagation (concentration gradients, thermal dynamics). The surrogate model (RBF) guides the search efficiently, while nonlinear constraints are evaluated directly from the simulation.

%%
%[text] ### Define Optimization Problem
%[text] Decision variables: current level $I\_k$ for each of the $N=5$ segments (each 6 minutes). Each evaluation runs one continuous simulation via `simulateCharging.m`.
%[text]
%[text] **Objective:** Minimize total time to reach target SOC (90%).
%[text]
%[text] **Nonlinear inequality constraints** (evaluated from full trajectory):
%[text]
%[text] - $V\_{terminal} \\leq 4.2$ V — prevents electrolyte decomposition and gas generation
%[text] - $T\_{cell} \\leq 50\\degree C$ — prevents accelerated SEI growth and thermal runaway risk
%[text] - $c\_{s,anode} \\leq c\_{s,max}$ — prevents lithium plating on the anode surface
%[text]
%[text] **Bound constraints:** Current in each segment is bounded between 0.2C and 2.5C (1–12.5 A).

N = N_opt_segments;
t_segments = linspace(0, t_charge_max, N + 1)';

lb = I_min_opt * ones(1, N);
ub = I_max_opt * ones(1, N);

%%
%[text] ### Run Optimization
%[text] `surrogateopt` builds an RBF surrogate of the objective from simulation evaluations and intelligently samples new points. Fast Restart eliminates model recompilation between evaluations.

simParams.T_ambient = T_ambient;
simParams.V_max = V_max;
simParams.I_charge = I_charge;
simParams.h_conv = h_conv;
simParams.A_cell = A_cell;
simParams.Kp_cv = Kp_cv;
simParams.I_cutoff = I_cutoff;
simParams.T_max = T_max;
simParams.c_s_anode_max = c_s_anode_max;
simParams.ms_currents = ms_currents;
simParams.ms_soc_thresholds = ms_soc_thresholds;

set_param(modelName, 'MaxStep', '1');
set_param(modelName, 'SimMechanicsOpenEditorOnUpdate', 'off');
set_param(modelName, 'FastRestart', 'on');
cleanupFR = onCleanup(@() set_param(modelName, 'FastRestart', 'off'));

opts = optimoptions('surrogateopt', ...
    'MaxFunctionEvaluations', 100, ...
    'Display', 'iter', ...
    'PlotFcn', 'surrogateoptplot');

[I_opt, fval, exitflag, output] = surrogateopt( ...
    @(I) chargingObjConstr(I, t_segments, modelName, SOC_init, SOC_target, Q_nom, T_init, V_max, T_max, c_s_anode_max, simParams), ...
    lb, ub, [], [], [], [], [], opts);

clear cleanupFR;

%%
%[text] ### Simulate Optimal Profile
%[text] Re-simulate the optimal profile to extract full results for comparison plots.

opt_result = simulateCharging(I_opt, t_segments, modelName, SOC_init, SOC_target, Q_nom, T_init, simParams);
t_charge_opt = opt_result.charge_time;

t_opt_profile = [];
I_opt_profile = [];
for k = 1:N
    if t_segments(k) >= t_charge_opt
        break;
    end
    t_end_k = min(t_segments(k+1), t_charge_opt);
    t_opt_profile = [t_opt_profile; t_segments(k); t_end_k-0.1]; %#ok<AGROW>
    I_opt_profile = [I_opt_profile; I_opt(k); I_opt(k)]; %#ok<AGROW>
end
t_opt_profile = [t_opt_profile; t_charge_opt; t_sim];
I_opt_profile = [I_opt_profile; 0; 0];
I_optimal_profile_ts = timeseries(I_opt_profile, t_opt_profile);

in_opt = Simulink.SimulationInput(modelName);
in_opt = in_opt.setVariable('active_controller', 3);
in_opt = in_opt.setVariable('I_optimal_profile', I_optimal_profile_ts);
in_opt = in_opt.setVariable('SOC_init', SOC_init);
in_opt = in_opt.setModelParameter('StopTime', num2str(t_sim));
out_opt = sim(in_opt);

opt_results = extractResults(out_opt, SOC_target);

%%
%[text] ## Comparison Visualization
%[text] Visualize all strategies side-by-side. Row 1: SOC, anode concentration, temperature. Row 2: current, voltage, charging time.

strategies = [{cccv_results}; sweep_results; {opt_results}];
strategy_names = [{'CC-CV'}; sweep_labels(:); {'Optimized'}];
n_strategies = numel(strategies);

colors = lines(n_strategies);

fig = figure('Position', [100, 100, 1400, 700], 'Name', 'Battery Fast Charging Comparison');
tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

%%
%[text] ### SOC Trajectories

nexttile;
hold on;
for k = 1:n_strategies
    r = strategies{k};
    plot(r.time/60, r.soc*100, 'Color', colors(k,:), 'LineWidth', 1.5);
end
yline(SOC_target*100, '--k', 'Target SOC');
hold off;
xlabel('Time (min)');
ylabel('SOC (%)');
title('State of Charge');
grid on;
legend(strategy_names, 'Location', 'southeast', 'FontSize', 7);

%%
%[text] ### Anode Surface Concentration

nexttile;
hold on;
for k = 1:n_strategies
    r = strategies{k};
    plot(r.time/60, r.anode_conc / c_s_anode_max, 'Color', colors(k,:), 'LineWidth', 1.5);
end
yline(1, '--r', 'Plating Limit');
hold off;
xlabel('Time (min)');
ylabel('c_s / c_{s,max}');
title('Anode Concentration (Normalized)');
grid on;
ylim([0 1.2]);

%%
%[text] ### Temperature Rise

nexttile;
hold on;
for k = 1:n_strategies
    r = strategies{k};
    plot(r.time/60, r.temperature - 273.15, 'Color', colors(k,:), 'LineWidth', 1.5);
end
yline(T_max - 273.15, '--r', 'T_{max}');
hold off;
xlabel('Time (min)');
ylabel(['Temperature (' char(176) 'C)']);
title('Cell Temperature');
grid on;

%%
%[text] ### Current Profiles

nexttile;
hold on;
for k = 1:n_strategies
    r = strategies{k};
    plot(r.time/60, r.current, 'Color', colors(k,:), 'LineWidth', 1.5);
end
hold off;
xlabel('Time (min)');
ylabel('Current (A)');
title('Charging Current');
grid on;

%%
%[text] ### Voltage Profiles

nexttile;
hold on;
for k = 1:n_strategies
    r = strategies{k};
    plot(r.time/60, r.voltage, 'Color', colors(k,:), 'LineWidth', 1.5);
end
yline(V_max, '--r', 'V_{max}');
hold off;
xlabel('Time (min)');
ylabel('Voltage (V)');
title('Terminal Voltage');
grid on;

%%
%[text] ### Charging Time Bar Chart

nexttile;
charge_times = zeros(n_strategies, 1);
for k = 1:n_strategies
    charge_times(k) = strategies{k}.charge_time_min;
end
b = barh(charge_times, 'FaceColor', 'flat');
for k = 1:n_strategies
    b.CData(k,:) = colors(k,:);
end
set(gca, 'YTick', 1:n_strategies, 'YTickLabel', strategy_names);
xlabel('Charging Time (min)');
title('Time to Target SOC');
grid on;

exportgraphics(fig, fullfile('images', 'comparison_dashboard.png'), 'Resolution', 150);

%%
%[text] ## Performance Metrics Table
%[text] Quantitative comparison of all charging strategies.

T = table('Size', [n_strategies, 6], ...
    'VariableTypes', {'string','double','double','double','double','double'}, ...
    'VariableNames', {'Strategy','ChargeTime_min','MaxVoltage_V','MaxTemp_C','FinalSOC_pct','TimeSavings_pct'});

baseline_time = cccv_results.charge_time_min;
for k = 1:n_strategies
    r = strategies{k};
    T.Strategy(k) = strategy_names{k};
    T.ChargeTime_min(k) = round(r.charge_time_min, 1);
    T.MaxVoltage_V(k) = round(r.max_voltage, 3);
    T.MaxTemp_C(k) = round(r.max_temp_C, 1);
    T.FinalSOC_pct(k) = round(r.final_soc*100, 1);
    T.TimeSavings_pct(k) = round((1 - r.charge_time_min/baseline_time)*100, 1);
end

T %#ok<NOPTS>

%%
%[text] ## Recommendations
%[text] The optimized profile achieves the fastest charging time while respecting all safety constraints. The multi-stage strategies offer a practical middle ground — simpler to implement than full optimization while still outperforming CC-CV. The "High-Power" multi-stage profile approaches optimal performance but may push thermal limits in sustained operation.

%%
%[text] ## Cleanup

bdclose(modelName);

%%
%[text] ## Helper Functions

%%
function results = extractResults(simOut, SOC_target)
    logs = simOut.logsout;

    soc_ts = logs.get('SOC').Values;
    v_ts = logs.get('V_terminal').Values;
    temp_ts = logs.get('T_battery').Values;
    i_ts = logs.get('I_cmd_active').Values;
    conc_ts = logs.get('c_s_anode').Values;

    results.time = soc_ts.Time;
    results.soc = soc_ts.Data;
    results.voltage = interp1(v_ts.Time, v_ts.Data, soc_ts.Time, 'linear', v_ts.Data(end));
    results.temperature = interp1(temp_ts.Time, temp_ts.Data, soc_ts.Time, 'linear', temp_ts.Data(end));
    results.current = interp1(i_ts.Time, i_ts.Data, soc_ts.Time, 'linear', 0);
    results.anode_conc = interp1(conc_ts.Time, conc_ts.Data, soc_ts.Time, 'linear', conc_ts.Data(end));

    idx_target = find(soc_ts.Data >= SOC_target, 1, 'first');
    if isempty(idx_target)
        results.charge_time_min = soc_ts.Time(end) / 60;
    else
        results.charge_time_min = soc_ts.Time(idx_target) / 60;
    end

    results.max_voltage = max(v_ts.Data);
    results.max_temp_C = max(temp_ts.Data) - 273.15;
    results.max_anode_conc = max(conc_ts.Data);
    results.final_soc = soc_ts.Data(end);
end


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
