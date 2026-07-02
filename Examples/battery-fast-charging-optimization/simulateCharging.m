function result = simulateCharging(I_segments, t_segments, modelName, SOC_init, SOC_target, Q_nom, T_init, params)
%SIMULATECHARGING Simulate full charging profile as a single continuous simulation.
%   Builds a piecewise-constant current timeseries from I_segments, runs one
%   sim() call, and extracts all metrics from the trajectory.

    N = numel(I_segments);
    t_end = t_segments(end);

    % Build piecewise-constant timeseries
    t_profile = zeros(2*N, 1);
    I_profile = zeros(2*N, 1);
    for k = 1:N
        t_profile(2*k-1) = t_segments(k);
        t_profile(2*k) = t_segments(k+1) - 0.1;
        I_profile(2*k-1) = I_segments(k);
        I_profile(2*k) = I_segments(k);
    end
    % Append zero current after final segment
    t_profile = [t_profile; t_segments(end); t_segments(end) + 1];
    I_profile = [I_profile; 0; 0];
    ts = timeseries(I_profile, t_profile);

    try
        w = warning('off');
        cleanupW = onCleanup(@() warning(w));

        in = Simulink.SimulationInput(modelName);
        in = in.setVariable('active_controller', 3);
        in = in.setVariable('I_optimal_profile', ts);
        in = in.setVariable('SOC_init', SOC_init);
        in = in.setVariable('T_init', T_init);
        in = in.setVariable('Q_nom', Q_nom);
        in = in.setVariable('SOC_target', SOC_target);
        in = in.setVariable('T_ambient', params.T_ambient);
        in = in.setVariable('V_max', params.V_max);
        in = in.setVariable('I_charge', params.I_charge);
        in = in.setVariable('h_conv', params.h_conv);
        in = in.setVariable('A_cell', params.A_cell);
        in = in.setVariable('Kp_cv', params.Kp_cv);
        in = in.setVariable('I_cutoff', params.I_cutoff);
        in = in.setVariable('T_max', params.T_max);
        in = in.setVariable('c_s_anode_max', params.c_s_anode_max);
        in = in.setVariable('ms_currents', params.ms_currents);
        in = in.setVariable('ms_soc_thresholds', params.ms_soc_thresholds);
        in = in.setModelParameter('StopTime', num2str(t_end));
        out = sim(in);

        logs = out.logsout;
        soc_data = logs.get('SOC').Values;
        v_data = logs.get('V_terminal').Values;
        temp_data = logs.get('T_battery').Values;
        conc_data = logs.get('c_s_anode').Values;

        result.max_voltage = max(v_data.Data);
        result.max_temp = max(temp_data.Data);
        result.max_anode_conc = max(conc_data.Data);
        result.final_soc = soc_data.Data(end);

        % Time to reach SOC target
        idx_target = find(soc_data.Data >= SOC_target, 1, 'first');
        if isempty(idx_target)
            result.charge_time = t_end;
        else
            result.charge_time = soc_data.Time(idx_target);
        end

        result.success = true;
    catch
        result.max_voltage = 5;
        result.max_temp = 400;
        result.max_anode_conc = 1e6;
        result.final_soc = SOC_init;
        result.charge_time = t_end;
        result.success = false;
    end
end
