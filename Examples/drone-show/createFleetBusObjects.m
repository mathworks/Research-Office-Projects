function createFleetBusObjects(N_uav, nSat)
% createFleetBusObjects - Define bus objects for fleet telemetry
%
% Creates bus objects in the base workspace for use by the
% MultiUAV_DroneShow model.
%
% nSat is the number of satellite slots in GNSSObservableBus (default rtk_n_sat's
% shipped value, 8). It is separate from N_uav because the GNSS interface is a
% property of the base station, not of the fleet.

if nargin < 2
    nSat = 8;
end

%% VehicleStateBus - Per-UAV estimated state
elems = Simulink.BusElement;
elems(1).Name = 'Position';
elems(1).Dimensions = [3 1];
elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 'Velocity';
elems(2).Dimensions = [3 1];
elems(2).DataType = 'double';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Attitude';
elems(3).Dimensions = [3 1];
elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'AngularRate';
elems(4).Dimensions = [3 1];
elems(4).DataType = 'double';

elems(5) = Simulink.BusElement;
elems(5).Name = 'PositionAccuracy';
elems(5).Dimensions = [1 1];
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'RTKMode';
elems(6).Dimensions = [1 1];
elems(6).DataType = 'double';

VehicleStateBus = Simulink.Bus;
VehicleStateBus.Elements = elems;
assignin('base','VehicleStateBus', VehicleStateBus);

%% FleetTelemetryBus - Aggregated fleet state
elems2 = Simulink.BusElement;
elems2(1).Name = 'Positions';
elems2(1).Dimensions = [N_uav 3];
elems2(1).DataType = 'double';

elems2(2) = Simulink.BusElement;
elems2(2).Name = 'Velocities';
elems2(2).Dimensions = [N_uav 3];
elems2(2).DataType = 'double';

elems2(3) = Simulink.BusElement;
elems2(3).Name = 'Attitudes';
elems2(3).Dimensions = [N_uav 3];
elems2(3).DataType = 'double';

elems2(4) = Simulink.BusElement;
elems2(4).Name = 'LEDColors';
elems2(4).Dimensions = [N_uav 3];
elems2(4).DataType = 'double';

elems2(5) = Simulink.BusElement;
elems2(5).Name = 'TrackingErrors';
elems2(5).Dimensions = [N_uav 1];
elems2(5).DataType = 'double';

elems2(6) = Simulink.BusElement;
elems2(6).Name = 'MinSeparation';
elems2(6).Dimensions = [1 1];
elems2(6).DataType = 'double';

elems2(7) = Simulink.BusElement;
elems2(7).Name = 'CommStatus';
elems2(7).Dimensions = [N_uav 1];
elems2(7).DataType = 'double';

FleetTelemetryBus = Simulink.Bus;
FleetTelemetryBus.Elements = elems2;
assignin('base','FleetTelemetryBus', FleetTelemetryBus);

%% MultirotorGuidanceControlBus - Control commands for guidance model (vectorized)
elems3(1) = Simulink.BusElement;
elems3(1).Name = 'Roll';
elems3(1).Dimensions = [1 N_uav];
elems3(1).DataType = 'double';

elems3(2) = Simulink.BusElement;
elems3(2).Name = 'Pitch';
elems3(2).Dimensions = [1 N_uav];
elems3(2).DataType = 'double';

elems3(3) = Simulink.BusElement;
elems3(3).Name = 'YawRate';
elems3(3).Dimensions = [1 N_uav];
elems3(3).DataType = 'double';

elems3(4) = Simulink.BusElement;
elems3(4).Name = 'Thrust';
elems3(4).Dimensions = [1 N_uav];
elems3(4).DataType = 'double';

MultirotorGuidanceControlBus = Simulink.Bus;
MultirotorGuidanceControlBus.Elements = elems3;
assignin('base','MultirotorGuidanceControlBus', MultirotorGuidanceControlBus);

%% MultirotorGuidanceEnvironmentBus - Environment inputs for guidance model (vectorized)
elems4(1) = Simulink.BusElement;
elems4(1).Name = 'Gravity';
elems4(1).Dimensions = [1 N_uav];
elems4(1).DataType = 'double';

MultirotorGuidanceEnvironmentBus = Simulink.Bus;
MultirotorGuidanceEnvironmentBus.Elements = elems4;
assignin('base','MultirotorGuidanceEnvironmentBus', MultirotorGuidanceEnvironmentBus);

%% MultirotorGuidanceStateBus - Output state from guidance model (vectorized)
elems5(1) = Simulink.BusElement;
elems5(1).Name = 'WorldPosition';
elems5(1).Dimensions = [3 N_uav];
elems5(1).DataType = 'double';

elems5(2) = Simulink.BusElement;
elems5(2).Name = 'WorldVelocity';
elems5(2).Dimensions = [3 N_uav];
elems5(2).DataType = 'double';

elems5(3) = Simulink.BusElement;
elems5(3).Name = 'EulerZYX';
elems5(3).Dimensions = [3 N_uav];
elems5(3).DataType = 'double';

elems5(4) = Simulink.BusElement;
elems5(4).Name = 'BodyAngularRateRPY';
elems5(4).Dimensions = [3 N_uav];
elems5(4).DataType = 'double';

elems5(5) = Simulink.BusElement;
elems5(5).Name = 'Thrust';
elems5(5).Dimensions = [1 N_uav];
elems5(5).DataType = 'double';

MultirotorGuidanceStateBus = Simulink.Bus;
MultirotorGuidanceStateBus.Elements = elems5;
assignin('base','MultirotorGuidanceStateBus', MultirotorGuidanceStateBus);

%% GNSSObservableBus - the RTK engine's input contract
% Defined in gnssObservableBus.m rather than inline, so the definition has one home and any
% caller can build the same bus into its own model workspace without writing into base.
% Read that file for the contract: undifferenced, metres, row 1 base and row 2 rover.
assignin('base', 'GNSSObservableBus', gnssObservableBus(nSat));

end
