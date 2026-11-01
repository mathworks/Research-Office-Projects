function data = setupParams(force)
%SETUPPARAMS Parameters for the homeMobileManipulator example.
%
%   data = SETUPPARAMS() prepares everything the model needs to compile and returns it,
%   printing the handful of values a reader of the example should know before pressing Run.
%
%   data = SETUPPARAMS(true) rebuilds from scratch, discarding the cached configuration and
%   robot tree. Use this after editing configuration/hmmConfig.m: several functions under
%   functions/ keep a persistent copy of cfg so that a MATLAB Function block stepping at
%   100 Hz does not rebuild it every tick, and without a forced rebuild an edit to hmmConfig
%   is invisible to precisely the code that consumes it.
%
%   THIS FUNCTION DELIBERATELY DEFINES NOTHING ITSELF. The example's parameters live in
%   configuration/hmmConfig.m, and the derived quantities - the 15-DOF rigidBodyTree, the
%   measured gripper aperture map, the operational-space gain matrices, the Simulink.Bus
%   definitions - are built by hmmSetup, which is wired to the model's PostLoadFcn because
%   the model cannot compile without it. A setupParams that assigned its own gains would be a
%   second source of truth for values the model already resolves from the first one, and the
%   two would diverge the first time either was edited. So this is the standard entry point
%   the example convention asks for, and it forwards.
%
%   Example:
%       params = setupParams;
%       in  = Simulink.SimulationInput('homeMobileManipulator');
%       out = sim(in);
%
%   See also hmmSetup, hmmConfig, HomeMobileManipulatorExample.

%   Copyright 2026 The MathWorks, Inc.

arguments
    force (1,1) logical = false
end

data = hmmSetup(force);
cfg  = data.cfg;

fprintf('\nhomeMobileManipulator is ready.\n');

fprintf('\nTASK\n');
fprintf('  instruction        "%s"\n', cfg.task.instruction);
fprintf('  provider mode      %s (model %s)\n', cfg.provider.mode, cfg.provider.model);

fprintf('\nROBOT\n');
fprintf('  tree               %d bodies, %d movable joints\n', ...
    numel(data.robot.Bodies), numel(homeConfiguration(data.robot)));
fprintf('  arm                %d joints, reach %.2f m\n', cfg.arm.nJoints, cfg.arm.reach);
fprintf('  gripper stroke     %.0f mm\n', 1000 * cfg.gripper.maxOpenPos);
fprintf('  base start pose    [%.2f %.2f] m, %.0f deg\n', ...
    cfg.base.initialPose(1:2), rad2deg(cfg.base.initialPose(3)));

fprintf('\nSAMPLE TIMES (s)\n');
fprintf('  plant %.3f   control %.3f   supervisor %.3f\n', ...
    cfg.rates.plant, cfg.rates.control, cfg.rates.supervisor);
fprintf('  perception %.3f   scene %.3f   planning %.3f\n', ...
    cfg.rates.perception, cfg.rates.scene, cfg.rates.planning);

fprintf('\nSAFETY BOUNDS\n');
% NOT "before e-stop", which is what this line used to claim. SafetyMonitor never reads
% cfg.safety.trackErrMax - the threshold reaches ArmSupervisor and BaseSupervisor as chart
% data (xTrackMax) and aborts a retreat, which is a supervisor decision and not a latch.
% Printing it under an e-stop heading is how a run that exceeds it 6x with EStop clear came
% to look like a monitor defect. See hmmPlotRun/drawArmTracking.
fprintf('  tool tracking      %.2f m before the supervisor aborts a retreat\n', ...
    cfg.safety.trackErrMax);
fprintf('  base envelope      vx %.2f  vy %.2f m/s  wz %.2f rad/s\n', ...
    cfg.base.vxMax, cfg.base.vyMax, cfg.base.omegaMax);
fprintf('\n');

if nargout == 0
    clear data
end

end
