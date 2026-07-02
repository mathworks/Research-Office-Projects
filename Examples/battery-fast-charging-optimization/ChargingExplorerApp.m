classdef ChargingExplorerApp < matlab.apps.AppBase

    properties (Access = public)
        UIFigure           matlab.ui.Figure
        % Panels
        ControlPanel       matlab.ui.container.Panel
        ActionsPanel       matlab.ui.container.Panel
        PlaybackPanel      matlab.ui.container.Panel
        % Checkboxes
        Checkboxes         matlab.ui.control.CheckBox
        % Buttons
        BtnRun             matlab.ui.control.Button
        BtnOptimize        matlab.ui.control.Button
        BtnLoadProfile     matlab.ui.control.Button
        BtnSaveProfile     matlab.ui.control.Button
        BtnPlay            matlab.ui.control.Button
        BtnStop            matlab.ui.control.Button
        % Controls
        SpeedDropDown      matlab.ui.control.DropDown
        TimeSlider         matlab.ui.control.Slider
        SpeedLabel         matlab.ui.control.Label
        StatusLabel        matlab.ui.control.Label
        Lamp               matlab.ui.control.Lamp
        % Spinners
        SpinSOCInit        matlab.ui.control.Spinner
        SpinSOCTarget      matlab.ui.control.Spinner
        LblSOCInit         matlab.ui.control.Label
        LblSOCTarget       matlab.ui.control.Label
        % Axes
        AxBattery          matlab.ui.control.UIAxes
        AxCurrent          matlab.ui.control.UIAxes
        AxSOC              matlab.ui.control.UIAxes
        AxVoltage          matlab.ui.control.UIAxes
        AxTemp             matlab.ui.control.UIAxes
        AxConc             matlab.ui.control.UIAxes
    end

    properties (Access = private)
        ModelName       = 'BatteryFastCharging'
        ProjectDir
        DataDir
        Params
        OptimalLoaded   = false
        OptimalData
        Results         = {}
        PlayTimer
        PlaySpeed       = 1
        PlayIdx         = 1
        IsPlaying       = false
        PlayTimes       = []
        TimeLines       = gobjects(0)
    end

    methods (Access = private)

        function loadParameters(app)
            app.ProjectDir = fileparts(mfilename('fullpath'));
            app.DataDir = fullfile(app.ProjectDir, 'data');
            if ~exist(app.DataDir, 'dir'), mkdir(app.DataDir); end

            run(fullfile(app.ProjectDir, 'setupParams.m'));
            app.Params.Q_nom = Q_nom;
            app.Params.V_max = V_max;
            app.Params.V_min = V_min;
            app.Params.T_max = T_max;
            app.Params.T_ambient = T_ambient;
            app.Params.SOC_init = SOC_init;
            app.Params.SOC_target = SOC_target;
            app.Params.I_charge = I_charge;
            app.Params.Kp_cv = Kp_cv;
            app.Params.ms_currents = ms_currents;
            app.Params.ms_soc_thresholds = ms_soc_thresholds;
            app.Params.t_sim = t_sim;
            app.Params.sweep_profiles = sweep_profiles;
            app.Params.sweep_labels = sweep_labels;
            app.Params.t_charge_max = t_charge_max;
            app.Params.N_opt_segments = N_opt_segments;
            app.Params.I_max_opt = I_max_opt;
            app.Params.I_min_opt = I_min_opt;
            app.Params.I_cutoff = I_cutoff;
            app.Params.h_conv = h_conv;
            app.Params.A_cell = A_cell;
            app.Params.c_s_anode_max = c_s_anode_max;
            app.Params.T_init = T_init;

            optFile = app.profileFilename();
            if isfile(optFile)
                app.OptimalLoaded = true;
                app.OptimalData = load(optFile);
            end
        end

        function onRunCompare(app, ~, ~)
            selected = app.getSelectedStrategies();
            if isempty(selected)
                app.StatusLabel.Text = 'Select at least one strategy.';
                return;
            end

            app.Lamp.Color = [0.2 0.8 0.2];
            app.StatusLabel.Text = 'Loading model...';
            drawnow;
            load_system(app.ModelName);

            app.Results = {};
            colors = app.getStrategyColors(numel(selected));
            n = numel(selected);

            liveSOC = zeros(1, n);
            liveTemp = ones(1, n) * app.Params.T_ambient;
            liveMaxTemp = ones(1, n) * app.Params.T_ambient;
            liveNames = cell(1, n);
            liveColors = colors;
            for i = 1:n
                liveSOC(i) = app.Params.SOC_init;
                liveNames{i} = selected{i}.name;
            end

            app.drawBatteriesLive(liveSOC, liveTemp, liveMaxTemp, liveNames, liveColors);
            app.clearPlotAxes();
            app.StatusLabel.Text = sprintf('Simulating %d strategies...', n);
            drawnow;

            for i = 1:n
                app.StatusLabel.Text = sprintf('Simulating: %s (%d/%d)...', selected{i}.name, i, n);
                drawnow;

                try
                    result = app.simulateStrategy(selected{i});
                catch e
                    app.StatusLabel.Text = sprintf('Error simulating %s: %s', selected{i}.name, e.message);
                    app.Lamp.Color = [0.9 0.2 0.2];
                    continue;
                end
                result.name = selected{i}.name;
                result.color = colors(i,:);
                app.Results{end+1} = result;

                app.animateBatteryAndPlots(result, liveSOC, liveTemp, liveMaxTemp, liveNames, liveColors, i);

                liveSOC(i) = result.soc(end);
                liveTemp(i) = result.temperature(end);
                liveMaxTemp(i) = max(result.temperature);
            end

            hold(app.AxSOC, 'on');
            yline(app.AxSOC, app.Params.SOC_target*100, '--k', 'HandleVisibility', 'off');
            hold(app.AxSOC, 'off');
            hold(app.AxVoltage, 'on');
            yline(app.AxVoltage, app.Params.V_max, '--r', 'HandleVisibility', 'off');
            hold(app.AxVoltage, 'off');
            hold(app.AxTemp, 'on');
            yline(app.AxTemp, app.Params.T_max - 273.15, '--r', 'HandleVisibility', 'off');
            hold(app.AxTemp, 'off');
            hold(app.AxConc, 'on');
            yline(app.AxConc, 1, '--r', 'HandleVisibility', 'off');
            hold(app.AxConc, 'off');

            legend(app.AxSOC, cellfun(@(r) r.name, app.Results, 'uni', 0), ...
                'TextColor', [0.2 0.2 0.25], 'Location', 'southeast', 'FontSize', 9);

            tMax = 0;
            for i = 1:numel(app.Results)
                tMax = max(tMax, app.Results{i}.soc_time(end));
            end
            app.PlayTimes = linspace(0, tMax, 300);
            app.PlayIdx = 1;

            for i = 1:numel(app.Results)
                r = app.Results{i};
                r.soc_interp = interp1(r.soc_time, r.soc, app.PlayTimes, 'linear', r.soc(end));
                r.temp_interp = interp1(r.t_time, r.temperature, app.PlayTimes, 'linear', r.temperature(end));
                r.voltage_interp = interp1(r.v_time, r.voltage, app.PlayTimes, 'linear', r.voltage(end));
                r.current_interp = interp1(r.i_time, r.current, app.PlayTimes, 'linear', 0);
                app.Results{i} = r;
            end

            app.addTimeIndicators();

            app.TimeSlider.Limits = [0, 1];
            app.TimeSlider.Value = 1;
            app.drawBatteries(numel(app.PlayTimes));

            app.Lamp.Color = [0.6 0.6 0.6];
            app.StatusLabel.Text = sprintf('Done. %d strategies compared. Use playback to replay.', numel(app.Results));
        end

        function onRunOptimization(app, ~, ~)
            app.Lamp.Color = [0.9 0.7 0.1];
            app.StatusLabel.Text = 'Running optimization (this may take several minutes)...';
            drawnow;

            load_system(app.ModelName);
            p = app.Params;

            N = p.N_opt_segments;
            t_segments = linspace(0, p.t_charge_max, N + 1)';
            lb = p.I_min_opt * ones(1, N);
            ub = p.I_max_opt * ones(1, N);

            mdlName = app.ModelName;
            SOC_init_local = p.SOC_init;
            SOC_target_local = p.SOC_target;
            Q_nom_local = p.Q_nom;
            V_max_local = p.V_max;
            T_max_local = p.T_max;
            T_init_local = p.T_init;
            c_s_max_local = p.c_s_anode_max;

            simParams.T_ambient = p.T_ambient;
            simParams.V_max = p.V_max;
            simParams.I_charge = p.I_charge;
            simParams.h_conv = p.h_conv;
            simParams.A_cell = p.A_cell;
            simParams.Kp_cv = p.Kp_cv;
            simParams.I_cutoff = p.I_cutoff;
            simParams.T_max = p.T_max;
            simParams.c_s_anode_max = p.c_s_anode_max;
            simParams.ms_currents = p.ms_currents;
            simParams.ms_soc_thresholds = p.ms_soc_thresholds;

            opts = optimoptions('surrogateopt', ...
                'MaxFunctionEvaluations', 100, ...
                'Display', 'iter', ...
                'OutputFcn', @(x, optimValues, state) app.optProgress(optimValues, state));

            set_param(mdlName, 'MaxStep', '1');
            set_param(mdlName, 'SimMechanicsOpenEditorOnUpdate', 'off');
            set_param(mdlName, 'FastRestart', 'on');
            try
                [I_opt, fval, exitflag] = surrogateopt( ...
                    @(I) chargingObjConstr(I, t_segments, mdlName, SOC_init_local, SOC_target_local, Q_nom_local, T_init_local, V_max_local, T_max_local, c_s_max_local, simParams), ...
                    lb, ub, [], [], [], [], [], opts);

                app.OptimalData.I_opt = I_opt;
                app.OptimalData.fval = fval;
                app.OptimalData.t_segments = t_segments;
                app.OptimalLoaded = true;
                app.Checkboxes(5).Enable = 'on';
                app.Checkboxes(5).Text = 'Optimized';
                app.BtnSaveProfile.Enable = 'on';
                app.Lamp.Color = [0.6 0.6 0.6];
                app.StatusLabel.Text = sprintf('Optimization complete (exit: %d, time: %.0fs). Profile ready.', exitflag, fval);
            catch e
                app.Lamp.Color = [0.8 0.2 0.2];
                app.StatusLabel.Text = ['Optimization failed: ' e.message];
            end
            set_param(mdlName, 'FastRestart', 'off');
        end

        function stop = optProgress(app, optimValues, state)
            stop = false;
            if isfield(optimValues, 'funccount')
                fcount = optimValues.funccount;
                fval = optimValues.fval;
                app.StatusLabel.Text = sprintf('Optimizing... Evals: %d, best: %.0fs', fcount, fval);
                drawnow;
            end
        end

        function onLoadProfile(app, ~, ~)
            files = dir(fullfile(app.DataDir, 'optimal_SOC*.mat'));
            if isempty(files)
                [file, path] = uigetfile('*.mat', 'Select optimal profile', app.DataDir);
                if file ~= 0
                    app.loadProfileAndSync(fullfile(path, file));
                end
            else
                names = {files.name};
                [idx, ok] = listdlg('ListString', names, ...
                    'SelectionMode', 'single', ...
                    'PromptString', 'Select a saved profile:', ...
                    'ListSize', [300, 200]);
                if ok
                    app.loadProfileAndSync(fullfile(app.DataDir, names{idx}));
                end
            end
        end

        function onSaveProfile(app, ~, ~)
            if app.OptimalLoaded
                saveData = app.OptimalData;
                saveData.SOC_init = app.Params.SOC_init;
                saveData.SOC_target = app.Params.SOC_target;
                saveData.date_generated = datestr(now); %#ok<TNOW1,DATST>
                saveData.description = sprintf('Optimal profile: SOC %d%% to %d%%', ...
                    round(app.Params.SOC_init*100), round(app.Params.SOC_target*100));
                fname = app.profileFilename();
                save(fname, '-struct', 'saveData');
                app.StatusLabel.Text = sprintf('Profile saved: %s', extractAfter(fname, app.DataDir));
            end
        end

        function onPlayPause(app, ~, ~)
            if app.IsPlaying
                app.IsPlaying = false;
                app.BtnPlay.Text = char(9654);
                app.BtnPlay.FontSize = 16;
                if ~isempty(app.PlayTimer) && isvalid(app.PlayTimer)
                    stop(app.PlayTimer);
                end
            else
                if isempty(app.Results), return; end
                if app.PlayIdx >= numel(app.PlayTimes)
                    app.PlayIdx = 1;
                    app.TimeSlider.Value = 0;
                end
                app.IsPlaying = true;
                app.BtnPlay.Text = char(9646) + "" + char(9646);
                app.BtnPlay.FontSize = 12;

                frameInterval = 0.08;
                if ~isempty(app.PlayTimer) && isvalid(app.PlayTimer)
                    stop(app.PlayTimer);
                    delete(app.PlayTimer);
                end
                app.PlayTimer = timer('ExecutionMode', 'fixedRate', ...
                    'Period', frameInterval, ...
                    'BusyMode', 'drop', ...
                    'TimerFcn', @(~,~) app.advanceFrame());
                start(app.PlayTimer);
            end
        end

        function onStopPlayback(app, ~, ~)
            app.IsPlaying = false;
            app.BtnPlay.Text = char(9654);
            app.BtnPlay.FontSize = 16;
            if ~isempty(app.PlayTimer) && isvalid(app.PlayTimer)
                stop(app.PlayTimer);
                delete(app.PlayTimer);
            end
            app.PlayIdx = 1;
            app.TimeSlider.Value = 0;
            if ~isempty(app.Results)
                app.drawBatteries(1);
            end
        end

        function onSliderChange(app, ~, ~)
            if isempty(app.Results), return; end
            val = app.TimeSlider.Value;
            idx = max(1, round(val * numel(app.PlayTimes)));
            app.PlayIdx = idx;
            app.drawBatteries(idx);
            app.updateTimeIndicator(idx);
        end

        function onSOCInitChanged(app, src)
            app.Params.SOC_init = src.Value / 100;
            app.tryAutoLoadProfile();
        end

        function onSOCTargetChanged(app, src)
            app.Params.SOC_target = src.Value / 100;
            app.Params.ms_soc_thresholds(end) = app.Params.SOC_target;
            for k = 1:size(app.Params.sweep_profiles, 1)
                app.Params.sweep_profiles{k,2}(end) = app.Params.SOC_target;
            end
            app.tryAutoLoadProfile();
        end

        function tryAutoLoadProfile(app)
            fname = app.profileFilename();
            if isfile(fname)
                app.loadProfileFromFile(fname);
                app.StatusLabel.Text = sprintf('Auto-loaded profile for SOC %d%%-%d%%.', ...
                    round(app.Params.SOC_init*100), round(app.Params.SOC_target*100));
            elseif app.OptimalLoaded
                app.Checkboxes(5).Text = 'Optimized (mismatched SOC)';
                app.StatusLabel.Text = 'Warning: loaded profile was not optimized for current SOC values.';
                app.Lamp.Color = [0.9 0.7 0.1];
            else
                app.Checkboxes(5).Enable = 'off';
                app.Checkboxes(5).Text = 'Optimized (not available)';
                app.BtnSaveProfile.Enable = 'off';
            end
        end

        function loadProfileFromFile(app, filepath)
            app.OptimalData = load(filepath);
            app.OptimalLoaded = true;
            app.Checkboxes(5).Enable = 'on';
            app.Checkboxes(5).Text = 'Optimized';
            app.BtnSaveProfile.Enable = 'on';
        end

        function loadProfileAndSync(app, filepath)
            app.loadProfileFromFile(filepath);
            if isfield(app.OptimalData, 'SOC_init')
                app.Params.SOC_init = app.OptimalData.SOC_init;
                app.SpinSOCInit.Value = round(app.OptimalData.SOC_init * 100);
            end
            if isfield(app.OptimalData, 'SOC_target')
                app.Params.SOC_target = app.OptimalData.SOC_target;
                app.SpinSOCTarget.Value = round(app.OptimalData.SOC_target * 100);
                app.Params.ms_soc_thresholds(end) = app.Params.SOC_target;
                for k = 1:size(app.Params.sweep_profiles, 1)
                    app.Params.sweep_profiles{k,2}(end) = app.Params.SOC_target;
                end
            end
            app.StatusLabel.Text = sprintf('Loaded profile: SOC %d%% to %d%%.', ...
                round(app.Params.SOC_init*100), round(app.Params.SOC_target*100));
        end

        function fname = profileFilename(app)
            fname = fullfile(app.DataDir, sprintf('optimal_SOC%d_%d.mat', ...
                round(app.Params.SOC_init*100), round(app.Params.SOC_target*100)));
        end

        function advanceFrame(app)
            try
                if ~isvalid(app.UIFigure), return; end
                if ~app.IsPlaying || isempty(app.Results), return; end

                speed = max(1, round(app.SpeedDropDown.Value));
                app.PlayIdx = app.PlayIdx + speed;

                if app.PlayIdx >= numel(app.PlayTimes)
                    app.PlayIdx = numel(app.PlayTimes);
                    app.IsPlaying = false;
                    app.BtnPlay.Text = char(9654);
                    app.BtnPlay.FontSize = 16;
                    if ~isempty(app.PlayTimer) && isvalid(app.PlayTimer)
                        stop(app.PlayTimer);
                    end
                end

                app.TimeSlider.Value = app.PlayIdx / numel(app.PlayTimes);
                app.drawBatteries(app.PlayIdx);
                app.updateTimeIndicator(app.PlayIdx);
            catch
                app.IsPlaying = false;
                app.BtnPlay.Text = char(9654);
                app.BtnPlay.FontSize = 16;
                if ~isempty(app.PlayTimer) && isvalid(app.PlayTimer)
                    stop(app.PlayTimer);
                end
            end
        end

        %% Simulation
        function result = simulateStrategy(app, strategy)
            p = app.Params;
            in = Simulink.SimulationInput(app.ModelName);
            in = in.setVariable('SOC_init', p.SOC_init);
            in = in.setVariable('SOC_target', p.SOC_target);
            in = in.setVariable('T_ambient', p.T_ambient);
            in = in.setVariable('T_init', p.T_init);
            in = in.setVariable('V_max', p.V_max);
            in = in.setVariable('Q_nom', p.Q_nom);
            in = in.setVariable('I_charge', p.I_charge);
            in = in.setVariable('h_conv', p.h_conv);
            in = in.setVariable('A_cell', p.A_cell);
            in = in.setVariable('Kp_cv', p.Kp_cv);
            in = in.setVariable('I_cutoff', p.I_cutoff);
            in = in.setVariable('T_max', p.T_max);
            in = in.setVariable('c_s_anode_max', p.c_s_anode_max);
            in = in.setVariable('ms_currents', p.ms_currents);
            in = in.setVariable('ms_soc_thresholds', p.ms_soc_thresholds);
            in = in.setVariable('I_optimal_profile', timeseries([0;0], [0; p.t_sim]));
            in = in.setModelParameter('StopTime', num2str(p.t_sim));

            switch strategy.type
                case 'cccv'
                    in = in.setVariable('active_controller', 1);

                case 'multistage'
                    in = in.setVariable('active_controller', 2);
                    in = in.setVariable('ms_currents', strategy.currents);
                    in = in.setVariable('ms_soc_thresholds', strategy.thresholds);

                case 'optimal'
                    I_opt_vals = app.OptimalData.I_opt;
                    t_seg = app.OptimalData.t_segments;
                    simP.T_ambient = p.T_ambient;
                    simP.V_max = p.V_max;
                    simP.I_charge = p.I_charge;
                    simP.h_conv = p.h_conv;
                    simP.A_cell = p.A_cell;
                    simP.Kp_cv = p.Kp_cv;
                    simP.I_cutoff = p.I_cutoff;
                    simP.T_max = p.T_max;
                    simP.c_s_anode_max = p.c_s_anode_max;
                    simP.ms_currents = p.ms_currents;
                    simP.ms_soc_thresholds = p.ms_soc_thresholds;
                    optResult = simulateCharging(I_opt_vals, t_seg, ...
                        app.ModelName, p.SOC_init, p.SOC_target, p.Q_nom, p.T_init, simP);
                    t_cutoff = optResult.charge_time;
                    t_prof = [];
                    I_prof = [];
                    for k = 1:numel(I_opt_vals)
                        if t_seg(k) >= t_cutoff
                            break;
                        end
                        t_start_k = t_seg(k);
                        t_end_k = min(t_seg(k+1), t_cutoff);
                        t_prof = [t_prof; t_start_k; t_end_k-0.1]; %#ok<AGROW>
                        I_prof = [I_prof; I_opt_vals(k); I_opt_vals(k)]; %#ok<AGROW>
                    end
                    t_prof = [t_prof; t_cutoff; p.t_sim];
                    I_prof = [I_prof; 0; 0];
                    ts = timeseries(I_prof, t_prof);
                    in = in.setVariable('active_controller', 3);
                    in = in.setVariable('I_optimal_profile', ts);
            end

            out = sim(in);
            logs = out.logsout;

            result.soc_time = logs.get('SOC').Values.Time;
            result.soc = logs.get('SOC').Values.Data;
            result.v_time = logs.get('V_terminal').Values.Time;
            result.voltage = logs.get('V_terminal').Values.Data;
            result.t_time = logs.get('T_battery').Values.Time;
            result.temperature = logs.get('T_battery').Values.Data;
            result.i_time = logs.get('I_cmd_active').Values.Time;
            result.current = logs.get('I_cmd_active').Values.Data;
            result.conc_time = logs.get('c_s_anode').Values.Time;
            result.anode_conc = logs.get('c_s_anode').Values.Data;

            idx = find(result.soc >= p.SOC_target, 1, 'first');
            if isempty(idx)
                result.charge_time_min = result.soc_time(end)/60;
            else
                result.charge_time_min = result.soc_time(idx)/60;
            end
        end

        function animateBatteryAndPlots(app, result, liveSOC, liveTemp, liveMaxTemp, liveNames, liveColors, battIdx)
            nPts = numel(result.soc_time);
            nFrames = 40;
            frameStep = max(1, floor(nPts / nFrames));
            color = result.color;

            hold(app.AxCurrent, 'on');
            hold(app.AxSOC, 'on');
            hold(app.AxVoltage, 'on');
            hold(app.AxTemp, 'on');
            hold(app.AxConc, 'on');

            hCurrent = animatedline(app.AxCurrent, 'Color', color, 'LineWidth', 1.5, 'DisplayName', result.name);
            hSOC = animatedline(app.AxSOC, 'Color', color, 'LineWidth', 1.5, 'DisplayName', result.name);
            hVoltage = animatedline(app.AxVoltage, 'Color', color, 'LineWidth', 1.5, 'DisplayName', result.name);
            hTemp = animatedline(app.AxTemp, 'Color', color, 'LineWidth', 1.5, 'DisplayName', result.name);
            hConc = animatedline(app.AxConc, 'Color', color, 'LineWidth', 1.5, 'DisplayName', result.name);

            i_ds = interp1(result.i_time, result.current, result.soc_time, 'linear', 0);
            v_ds = interp1(result.v_time, result.voltage, result.soc_time, 'linear', result.voltage(end));
            t_ds = interp1(result.t_time, result.temperature, result.soc_time, 'linear', result.temperature(end));
            c_ds = interp1(result.conc_time, result.anode_conc / app.Params.c_s_anode_max, result.soc_time, 'linear', result.anode_conc(end) / app.Params.c_s_anode_max);

            prevFrame = 0;
            for k = 1:frameStep:nPts
                range = (prevFrame+1):k;
                addpoints(hCurrent, result.soc_time(range)/60, i_ds(range));
                addpoints(hSOC, result.soc_time(range)/60, result.soc(range)*100);
                addpoints(hVoltage, result.soc_time(range)/60, v_ds(range));
                addpoints(hTemp, result.soc_time(range)/60, t_ds(range) - 273.15);
                addpoints(hConc, result.soc_time(range)/60, c_ds(range));
                prevFrame = k;

                liveSOC(battIdx) = result.soc(k);
                liveTemp(battIdx) = result.temperature(k);
                liveMaxTemp(battIdx) = max(liveMaxTemp(battIdx), result.temperature(k));
                app.drawBatteriesLive(liveSOC, liveTemp, liveMaxTemp, liveNames, liveColors);

                drawnow limitrate;
            end

            if prevFrame < nPts
                range = (prevFrame+1):nPts;
                addpoints(hCurrent, result.soc_time(range)/60, i_ds(range));
                addpoints(hSOC, result.soc_time(range)/60, result.soc(range)*100);
                addpoints(hVoltage, result.soc_time(range)/60, v_ds(range));
                addpoints(hTemp, result.soc_time(range)/60, t_ds(range) - 273.15);
                addpoints(hConc, result.soc_time(range)/60, c_ds(range));
            end

            liveSOC(battIdx) = result.soc(end);
            liveTemp(battIdx) = result.temperature(end);
            liveMaxTemp(battIdx) = max(liveMaxTemp(battIdx), max(result.temperature));
            app.drawBatteriesLive(liveSOC, liveTemp, liveMaxTemp, liveNames, liveColors);
            drawnow;

            hold(app.AxCurrent, 'off');
            hold(app.AxSOC, 'off');
            hold(app.AxVoltage, 'off');
            hold(app.AxConc, 'off');
            hold(app.AxTemp, 'off');
        end

        %% Battery Drawing
        function drawBatteries(app, idx)
            ax = app.AxBattery;
            cla(ax);
            hold(ax, 'on');

            n = numel(app.Results);
            if n == 0, return; end

            margin = 0.12;
            if n == 1
                positions = 0.5;
            else
                positions = linspace(margin, 1 - margin, n);
            end

            for i = 1:n
                r = app.Results{i};
                soc = r.soc_interp(idx);
                tempC = r.temp_interp(idx) - 273.15;
                maxTempC = max(r.temp_interp(1:idx)) - 273.15;

                app.drawSingleBattery(ax, positions(i), soc, tempC, maxTempC, r.name, r.color);
            end

            t_now = app.PlayTimes(idx);
            text(ax, 0.5, -0.08, sprintf('t = %.1f min', t_now/60), ...
                'HorizontalAlignment', 'center', 'FontSize', 13, ...
                'Color', [0.2 0.2 0.25], 'Units', 'normalized');

            axis(ax, 'off');
            xlim(ax, [-0.02, 1.02]);
            ylim(ax, [-0.12, 1.05]);
            hold(ax, 'off');
        end

        function drawBatteriesLive(app, socValues, tempValues, maxTempValues, names, colors)
            ax = app.AxBattery;
            cla(ax);
            hold(ax, 'on');

            n = numel(socValues);
            if n == 0, return; end

            margin = 0.12;
            if n == 1
                positions = 0.5;
            else
                positions = linspace(margin, 1 - margin, n);
            end

            for i = 1:n
                app.drawSingleBattery(ax, positions(i), socValues(i), ...
                    tempValues(i) - 273.15, maxTempValues(i) - 273.15, names{i}, colors(i,:));
            end

            axis(ax, 'off');
            xlim(ax, [-0.02, 1.02]);
            ylim(ax, [-0.12, 1.05]);
            hold(ax, 'off');
        end

        function drawSingleBattery(app, ax, xCenter, soc, tempC, maxTempC, name, borderColor)
            bWidth = 0.10;
            bHeight = 0.72;
            termWidth = 0.035;
            termHeight = 0.05;

            xLeft = xCenter - bWidth/2;
            yBottom = 0.05;

            T_min = app.Params.T_ambient - 273.15;
            T_max_display = app.Params.T_max - 273.15;
            tNorm = min(1, max(0, (tempC - T_min) / (T_max_display - T_min)));
            fillColor = ChargingExplorerApp.tempToColor(tNorm);

            rectangle(ax, 'Position', [xLeft, yBottom, bWidth, bHeight], ...
                'Curvature', 0.1, 'EdgeColor', borderColor, 'LineWidth', 2, ...
                'FaceColor', [0.92 0.92 0.94]);

            rectangle(ax, 'Position', [xCenter - termWidth/2, yBottom + bHeight, termWidth, termHeight], ...
                'Curvature', 0.3, 'EdgeColor', borderColor, 'LineWidth', 1.5, ...
                'FaceColor', [0.6 0.6 0.62]);

            fillHeight = soc * bHeight * 0.92;
            fillMargin = bWidth * 0.08;
            if fillHeight > 0.001
                rectangle(ax, 'Position', [xLeft + fillMargin, yBottom + bHeight*0.04, ...
                    bWidth - 2*fillMargin, fillHeight], ...
                    'Curvature', 0.05, 'EdgeColor', 'none', 'FaceColor', fillColor);
            end

            xTickL = xLeft + fillMargin;
            xTickR = xLeft + bWidth - fillMargin;
            tickLevels = (0.1:0.1:0.9)';
            yTicks = yBottom + bHeight*0.04 + tickLevels * bHeight * 0.92;
            nTicks = numel(tickLevels);
            xData = [repmat(xTickL, nTicks, 1), repmat(xTickR, nTicks, 1), nan(nTicks, 1)]';
            yData = [yTicks, yTicks, nan(nTicks, 1)]';
            plot(ax, xData(:), yData(:), '-', 'Color', [0.92 0.92 0.94], 'LineWidth', 1.5);

            text(ax, xCenter, yBottom + bHeight*0.54, sprintf('%.0f%%', soc*100), ...
                'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold', ...
                'Color', [0.1 0.1 0.1]);

            text(ax, xCenter, yBottom - 0.03, sprintf('%.1f°C', tempC), ...
                'HorizontalAlignment', 'center', 'FontSize', 9, ...
                'Color', fillColor);
            text(ax, xCenter, yBottom - 0.09, sprintf('max: %.1f°C', maxTempC), ...
                'HorizontalAlignment', 'center', 'FontSize', 8, ...
                'Color', [0.45 0.45 0.5]);

            text(ax, xCenter, yBottom + bHeight + termHeight + 0.03, name, ...
                'HorizontalAlignment', 'center', 'FontSize', 10, ...
                'Color', [0.2 0.2 0.25], 'Interpreter', 'none');
        end

        %% Plot Helpers
        function clearPlotAxes(app)
            cla(app.AxCurrent);
            cla(app.AxSOC);
            cla(app.AxVoltage);
            cla(app.AxTemp);
            cla(app.AxConc);

            xlabel(app.AxCurrent, 'Time (min)', 'Color', [0.3 0.3 0.35]);
            ylabel(app.AxCurrent, 'A', 'Color', [0.3 0.3 0.35]);
            xlabel(app.AxSOC, 'Time (min)', 'Color', [0.3 0.3 0.35]);
            ylabel(app.AxSOC, '%', 'Color', [0.3 0.3 0.35]);
            xlabel(app.AxVoltage, 'Time (min)', 'Color', [0.3 0.3 0.35]);
            ylabel(app.AxVoltage, 'V', 'Color', [0.3 0.3 0.35]);
            xlabel(app.AxTemp, 'Time (min)', 'Color', [0.3 0.3 0.35]);
            ylabel(app.AxTemp, char(176) + "C", 'Color', [0.3 0.3 0.35]);
            xlabel(app.AxConc, 'Time (min)', 'Color', [0.3 0.3 0.35]);
            ylabel(app.AxConc, 'c_s / c_{s,max}', 'Color', [0.3 0.3 0.35]);
        end

        function addTimeIndicators(app)
            app.TimeLines = gobjects(5,1);
            hold(app.AxCurrent, 'on');
            app.TimeLines(1) = xline(app.AxCurrent, 0, '-', 'Color', [0.3 0.3 0.8 0.5], 'LineWidth', 1, 'HandleVisibility', 'off');
            hold(app.AxCurrent, 'off');
            hold(app.AxSOC, 'on');
            app.TimeLines(2) = xline(app.AxSOC, 0, '-', 'Color', [0.3 0.3 0.8 0.5], 'LineWidth', 1, 'HandleVisibility', 'off');
            hold(app.AxSOC, 'off');
            hold(app.AxVoltage, 'on');
            app.TimeLines(3) = xline(app.AxVoltage, 0, '-', 'Color', [0.3 0.3 0.8 0.5], 'LineWidth', 1, 'HandleVisibility', 'off');
            hold(app.AxVoltage, 'off');
            hold(app.AxTemp, 'on');
            app.TimeLines(4) = xline(app.AxTemp, 0, '-', 'Color', [0.3 0.3 0.8 0.5], 'LineWidth', 1, 'HandleVisibility', 'off');
            hold(app.AxTemp, 'off');
            hold(app.AxConc, 'on');
            app.TimeLines(5) = xline(app.AxConc, 0, '-', 'Color', [0.3 0.3 0.8 0.5], 'LineWidth', 1, 'HandleVisibility', 'off');
            hold(app.AxConc, 'off');
        end

        function updateTimeIndicator(app, idx)
            if isempty(app.Results), return; end
            if isempty(app.TimeLines), return; end
            t_min = app.PlayTimes(idx) / 60;
            for k = 1:5
                if isvalid(app.TimeLines(k))
                    app.TimeLines(k).Value = t_min;
                end
            end
        end

        %% Strategy Selection
        function selected = getSelectedStrategies(app)
            p = app.Params;
            selected = {};
            if app.Checkboxes(1).Value
                selected{end+1} = struct('name', 'CC-CV (1C)', 'type', 'cccv');
            end
            for k = 1:3
                if app.Checkboxes(k+1).Value
                    selected{end+1} = struct('name', p.sweep_labels{k}, ...
                        'type', 'multistage', ...
                        'currents', p.sweep_profiles{k,1} * p.Q_nom, ...
                        'thresholds', p.sweep_profiles{k,2}); %#ok<AGROW>
                end
            end
            if app.Checkboxes(5).Value && app.OptimalLoaded
                selected{end+1} = struct('name', 'Optimized', 'type', 'optimal');
            end
        end

    end

    methods (Static, Access = private)
        function c = tempToColor(tNorm)
            if tNorm < 0.5
                t2 = tNorm * 2;
                c = [t2, t2, 1-t2];
            else
                t2 = (tNorm - 0.5) * 2;
                c = [1, 1-t2, 0];
            end
        end

        function colors = getStrategyColors(n)
            baseColors = [
                0.2, 0.6, 0.9;
                0.9, 0.5, 0.1;
                0.3, 0.8, 0.3;
                0.8, 0.2, 0.2;
                0.6, 0.3, 0.8;
                0.9, 0.8, 0.1;
            ];
            colors = baseColors(mod((0:n-1), size(baseColors,1)) + 1, :);
        end
    end

    methods (Access = private)
        function createComponents(app)
            app.UIFigure = uifigure('Name', 'Battery Fast Charging Explorer', ...
                'Position', [50, 50, 1680, 820], ...
                'Color', [0.88 0.88 0.91], ...
                'CloseRequestFcn', @(~,~) app.onClose());

            % --- Left Panel: Charging Strategies ---
            app.ControlPanel = uipanel(app.UIFigure, 'Title', 'Charging Strategies', ...
                'Position', [10, 540, 280, 230], ...
                'BackgroundColor', [0.88 0.88 0.91], ...
                'ForegroundColor', [0.15 0.15 0.2], ...
                'FontSize', 13, 'FontWeight', 'bold');

            strategies = {'CC-CV (1C)', 'Multi-Stage: Baseline', ...
                          'Multi-Stage: Conservative', 'Multi-Stage: High-Power', 'Optimized'};
            app.Checkboxes = matlab.ui.control.CheckBox.empty;
            for i = 1:numel(strategies)
                app.Checkboxes(i) = uicheckbox(app.ControlPanel, ...
                    'Text', strategies{i}, ...
                    'Position', [15, 195 - i*32, 250, 26], ...
                    'FontColor', [0.2 0.2 0.25], ...
                    'FontSize', 11, ...
                    'Value', false);
            end
            app.Checkboxes(1).Value = true;

            if ~app.OptimalLoaded
                app.Checkboxes(5).Enable = 'off';
                app.Checkboxes(5).Text = 'Optimized (not loaded)';
            end

            % --- Actions Panel ---
            app.ActionsPanel = uipanel(app.UIFigure, 'Title', 'Actions', ...
                'Position', [10, 170, 280, 360], ...
                'BackgroundColor', [0.88 0.88 0.91], ...
                'ForegroundColor', [0.15 0.15 0.2], ...
                'FontSize', 13, 'FontWeight', 'bold');

            % SOC parameter spinners
            app.LblSOCInit = uilabel(app.ActionsPanel, 'Text', 'Initial SOC (%)', ...
                'Position', [15, 305, 100, 22], ...
                'FontSize', 11, 'FontColor', [0.3 0.3 0.35]);
            app.SpinSOCInit = uispinner(app.ActionsPanel, ...
                'Position', [140, 305, 70, 22], ...
                'Limits', [0 99], 'Step', 5, ...
                'Value', app.Params.SOC_init * 100, ...
                'ValueDisplayFormat', '%.0f', ...
                'ValueChangedFcn', @(src,~) app.onSOCInitChanged(src));

            app.LblSOCTarget = uilabel(app.ActionsPanel, 'Text', 'Target SOC (%)', ...
                'Position', [15, 278, 100, 22], ...
                'FontSize', 11, 'FontColor', [0.3 0.3 0.35]);
            app.SpinSOCTarget = uispinner(app.ActionsPanel, ...
                'Position', [140, 278, 70, 22], ...
                'Limits', [1 95], 'Step', 5, ...
                'Value', app.Params.SOC_target * 100, ...
                'ValueDisplayFormat', '%.0f', ...
                'ValueChangedFcn', @(src,~) app.onSOCTargetChanged(src));

            uilabel(app.ActionsPanel, 'Text', sprintf('Note: changing these values requires\nre-running the optimization.'), ...
                'Position', [15, 248, 250, 30], ...
                'FontSize', 9, 'FontColor', [0.5 0.5 0.55], 'FontAngle', 'italic');

            app.BtnRun = uibutton(app.ActionsPanel, 'Text', 'Run & Compare', ...
                'Position', [15, 195, 250, 42], ...
                'FontSize', 13, 'FontWeight', 'bold', ...
                'BackgroundColor', [0.2 0.6 0.3], ...
                'FontColor', 'white', ...
                'ButtonPushedFcn', @(src,evt) app.onRunCompare(src, evt));

            app.BtnOptimize = uibutton(app.ActionsPanel, 'Text', 'Run Optimization', ...
                'Position', [15, 145, 250, 42], ...
                'FontSize', 13, 'FontWeight', 'bold', ...
                'BackgroundColor', [0.6 0.4 0.1], ...
                'FontColor', 'white', ...
                'ButtonPushedFcn', @(src,evt) app.onRunOptimization(src, evt));

            app.BtnLoadProfile = uibutton(app.ActionsPanel, 'Text', 'Load Optimal Profile', ...
                'Position', [15, 100, 120, 35], ...
                'FontSize', 10, ...
                'ButtonPushedFcn', @(src,evt) app.onLoadProfile(src, evt));

            app.BtnSaveProfile = uibutton(app.ActionsPanel, 'Text', 'Save Optimal Profile', ...
                'Position', [145, 100, 120, 35], ...
                'FontSize', 10, ...
                'Enable', 'off', ...
                'ButtonPushedFcn', @(src,evt) app.onSaveProfile(src, evt));

            app.Lamp = uilamp(app.ActionsPanel, 'Position', [15, 50, 32, 32], 'Color', [0.6 0.6 0.6]);
            app.StatusLabel = uilabel(app.ActionsPanel, 'Text', 'Ready', ...
                'Position', [48, 35, 220, 60], ...
                'FontSize', 11, 'FontColor', [0.4 0.4 0.45], ...
                'WordWrap', 'on');

            % --- Playback Panel ---
            app.PlaybackPanel = uipanel(app.UIFigure, 'Title', 'Playback', ...
                'Position', [10, 40, 280, 120], ...
                'BackgroundColor', [0.88 0.88 0.91], ...
                'ForegroundColor', [0.15 0.15 0.2], ...
                'FontSize', 13, 'FontWeight', 'bold');

            app.BtnPlay = uibutton(app.PlaybackPanel, 'Text', char(9654), ...
                'Position', [15, 55, 50, 35], ...
                'FontSize', 16, ...
                'ButtonPushedFcn', @(src,evt) app.onPlayPause(src, evt));

            app.BtnStop = uibutton(app.PlaybackPanel, 'Text', char(9632), ...
                'Position', [70, 55, 50, 35], ...
                'FontSize', 20, ...
                'ButtonPushedFcn', @(src,evt) app.onStopPlayback(src, evt));

            app.SpeedLabel = uilabel(app.PlaybackPanel, 'Text', 'Speed:', ...
                'Position', [130, 60, 45, 25], ...
                'FontColor', [0.3 0.3 0.35], 'FontSize', 11);
            app.SpeedDropDown = uidropdown(app.PlaybackPanel, ...
                'Items', {'1x', '2x', '5x', '10x', '20x'}, ...
                'ItemsData', [1, 2, 5, 10, 20], ...
                'Value', 2, ...
                'Position', [175, 60, 80, 25]);

            app.TimeSlider = uislider(app.PlaybackPanel, ...
                'Position', [15, 38, 245, 3], ...
                'Limits', [0, 1], 'Value', 0, ...
                'MajorTicks', [], 'MinorTicks', [], ...
                'ValueChangedFcn', @(src,evt) app.onSliderChange(src, evt));

            % --- Top Row: Battery Animation | SOC | Anode Concentration ---
            app.AxBattery = uiaxes(app.UIFigure, 'Position', [300, 420, 560, 370]);
            app.AxBattery.Color = [0.97 0.97 0.98];
            app.AxBattery.XColor = 'none'; app.AxBattery.YColor = 'none';
            title(app.AxBattery, 'Battery State', 'Color', [0.2 0.2 0.25], 'FontSize', 14);

            app.AxSOC = uiaxes(app.UIFigure, 'Position', [880, 420, 380, 370]);
            app.AxSOC.Color = [0.97 0.97 0.98];
            app.AxSOC.XColor = [0.3 0.3 0.35]; app.AxSOC.YColor = [0.3 0.3 0.35];
            title(app.AxSOC, 'SOC (%)', 'Color', [0.2 0.2 0.25]);
            grid(app.AxSOC, 'on'); app.AxSOC.GridColor = [0.75 0.75 0.78];

            app.AxConc = uiaxes(app.UIFigure, 'Position', [1280, 420, 380, 370]);
            app.AxConc.Color = [0.97 0.97 0.98];
            app.AxConc.XColor = [0.3 0.3 0.35]; app.AxConc.YColor = [0.3 0.3 0.35];
            title(app.AxConc, 'Anode Concentration', 'Color', [0.2 0.2 0.25]);
            grid(app.AxConc, 'on'); app.AxConc.GridColor = [0.75 0.75 0.78];

            % --- Bottom Row: Current | Voltage | Temperature ---
            app.AxCurrent = uiaxes(app.UIFigure, 'Position', [300, 40, 430, 350]);
            app.AxCurrent.Color = [0.97 0.97 0.98];
            app.AxCurrent.XColor = [0.3 0.3 0.35]; app.AxCurrent.YColor = [0.3 0.3 0.35];
            title(app.AxCurrent, 'Current (A)', 'Color', [0.2 0.2 0.25]);
            grid(app.AxCurrent, 'on'); app.AxCurrent.GridColor = [0.75 0.75 0.78];

            app.AxVoltage = uiaxes(app.UIFigure, 'Position', [750, 40, 430, 350]);
            app.AxVoltage.Color = [0.97 0.97 0.98];
            app.AxVoltage.XColor = [0.3 0.3 0.35]; app.AxVoltage.YColor = [0.3 0.3 0.35];
            title(app.AxVoltage, 'Voltage (V)', 'Color', [0.2 0.2 0.25]);
            grid(app.AxVoltage, 'on'); app.AxVoltage.GridColor = [0.75 0.75 0.78];

            app.AxTemp = uiaxes(app.UIFigure, 'Position', [1200, 40, 460, 350]);
            app.AxTemp.Color = [0.97 0.97 0.98];
            app.AxTemp.XColor = [0.3 0.3 0.35]; app.AxTemp.YColor = [0.3 0.3 0.35];
            title(app.AxTemp, ['Temperature (' char(176) 'C)'], 'Color', [0.2 0.2 0.25]);
            grid(app.AxTemp, 'on'); app.AxTemp.GridColor = [0.75 0.75 0.78];
        end

        function onClose(app)
            if ~isempty(app.PlayTimer) && isvalid(app.PlayTimer)
                stop(app.PlayTimer);
                delete(app.PlayTimer);
            end
            delete(app.UIFigure);
        end
    end

    methods (Access = public)
        function app = ChargingExplorerApp()
            app.loadParameters();
            app.createComponents();
            registerApp(app, app.UIFigure);

            if nargout == 0
                clear app
            end
        end

        function delete(app)
            if ~isempty(app.PlayTimer) && isvalid(app.PlayTimer)
                stop(app.PlayTimer);
                delete(app.PlayTimer);
            end
            delete(app.UIFigure);
        end
    end
end
