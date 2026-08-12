function out = chargingObjConstr(I_segments, t_segments, modelName, SOC_init, SOC_target, Q_nom, T_init, V_max, T_max, c_s_max, params)
%CHARGINGOBJCONSTR Combined objective and constraints for surrogateopt.
%   Runs a single continuous simulation and returns a struct with:
%     Fval - charging time (objective to minimize)
%     Ineq - [voltage_violation; temp_violation; plating_violation] (all <= 0)

    result = simulateCharging(I_segments, t_segments, modelName, SOC_init, SOC_target, Q_nom, T_init, params);

    out.Fval = result.charge_time;
    out.Ineq = [
        result.max_voltage - V_max;
        result.max_temp - T_max;
        result.max_anode_conc - c_s_max;
    ];
end
