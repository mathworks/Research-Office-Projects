function arm = params_softgripper_arm()
% PARAMS_SOFTGRIPPER_ARM  UR5e (kinematic, motion-driven) parameters for UR5eArm.slx.
%
%   arm = PARAMS_SOFTGRIPPER_ARM()
%
%   The UR5e comes from loadrobot('universalUR5e') through smimport (geometry, inertia, joint frames). The
%   joints are motion-driven: the commanded angle passes a 2nd-order Simulink-PS input filter, which also
%   supplies the velocity and acceleration Simscape needs. Units SI.

arm.tauMotion = 0.02;               % s        EST: input-filter time constant. Must stay well below the
                                    %          planned move durations (about 1 s, M3) so it only smooths steps
arm.qHome = [0 -pi/2 pi/2 -pi/2 -pi/2 0];   % rad  EST: tool pointing down, a common UR pick posture
end
