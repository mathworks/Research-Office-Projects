%% setupParams - Battery Fast Charging Example Parameters

%% Battery Specifications
if bdIsLoaded('BatteryFastCharging')
    spm_path__ = 'BatteryFastCharging/BatteryPlant/SPM';
    F__ = 96485;
    Q_nom = min( ...
        str2double(get_param(spm_path__, 'ActiveMaterialVolumeFractionAnode')) * ...
        str2double(get_param(spm_path__, 'ThicknessAnode')) * ...
        str2double(get_param(spm_path__, 'ElectrodePlateArea')) * ...
        str2double(get_param(spm_path__, 'MaximumConcentrationAnode')) * ...
        (str2double(get_param(spm_path__, 'MaximumStoichiometryAnode')) - str2double(get_param(spm_path__, 'MinimumStoichiometryAnode'))), ...
        str2double(get_param(spm_path__, 'ActiveMaterialVolumeFractionCathode')) * ...
        str2double(get_param(spm_path__, 'ThicknessCathode')) * ...
        str2double(get_param(spm_path__, 'ElectrodePlateArea')) * ...
        str2double(get_param(spm_path__, 'MaximumConcentrationCathode')) * ...
        (str2double(get_param(spm_path__, 'MaximumStoichiometryCathode')) - str2double(get_param(spm_path__, 'MinimumStoichiometryCathode')))) ...
        * F__ / 3600;  % Ah
    c_s_anode_max = str2double(get_param(spm_path__, 'MaximumConcentrationAnode')) * ...
        str2double(get_param(spm_path__, 'MaximumStoichiometryAnode')) - 100;  % mol/m^3 - plating limit
    clear spm_path__ F__
else
    Q_nom = 5;          % Ah - Fallback if model not loaded
    c_s_anode_max = 24344;  % mol/m^3 - Fallback (30555*0.8 - 100)
end
V_max = 4.2;            % V - Maximum cell voltage (CV phase threshold)
V_min = 2.5;            % V - Minimum cell voltage (discharge cutoff)

%% Initial Conditions
SOC_init = 0.2;         % Initial state of charge (20%)
T_init = 298.15;        % K - Initial temperature (25 deg C)

%% Charging Targets
SOC_target = 0.9;       % Target SOC for charge termination
I_cutoff = 0.05*Q_nom;  % A - CV phase taper current cutoff (C/20)

%% Safety Constraints
T_max = 323.15;         % K - Max temperature safety limit (50 deg C)
T_ambient = 298.15;     % K - Ambient temperature (25 deg C)
c_s_anode_max;          % mol/m^3 - Max anode concentration (plating limit, set above from SPM)

%% Thermal Parameters
h_conv = 20;            % W/(m^2*K) - Convective heat transfer coefficient
A_cell = 0.01;          % m^2 - Cell surface area for cooling

%% CC-CV Controller Parameters
I_charge = Q_nom;       % A - CC phase current (1C rate)
Kp_cv = 10;             % Proportional gain for CV phase

%% Multi-Stage Charging Parameters
ms_currents = [2*Q_nom, 1.5*Q_nom, 1*Q_nom];  % A - Stage currents [2C, 1.5C, 1C]
ms_soc_thresholds = [0.5, 0.7, SOC_target];    % SOC thresholds for stage transitions

%% Multi-Stage Sweep Configuration
sweep_profiles = {
    [2.0, 1.5, 1.0], [0.50, 0.70, SOC_target];   % Baseline multi-stage
    [1.5, 1.2, 0.8], [0.55, 0.75, SOC_target];   % Conservative
    [3.0, 2.0, 1.0], [0.40, 0.60, SOC_target];   % High-power
};
sweep_labels = {'Baseline MS', 'Conservative', 'High-Power'};

%% Optimization Parameters (fmincon)
N_opt_segments = 5;            % Number of time segments for piecewise-constant profile
t_charge_max = 1800;           % s - Maximum allowed charging time (30 min)
I_max_opt = 2.5*Q_nom;         % A - Maximum current bound for optimization (2.5C)
I_min_opt = 0.2*Q_nom;         % A - Minimum current bound for optimization (0.2C)

%% Controller Selection
% 1 = CCCV, 2 = Multi-Stage, 3 = Optimal Profile
active_controller = 1;

%% Optimal Profile Controller (placeholder - overwritten by optimization)
t_profile = (0:0.1:t_charge_max)';
I_optimal_default = Q_nom * ones(size(t_profile));
I_optimal_profile = timeseries(I_optimal_default, t_profile);

%% Simulation Settings
t_sim = 3000;                  % s - Default simulation stop time
