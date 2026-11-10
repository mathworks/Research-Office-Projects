function finger = params_softgripper_finger()
% PARAMS_SOFTGRIPPER_FINGER  Soft pneumatic finger (pseudo-rigid-body chain) parameters.
%
%   finger = PARAMS_SOFTGRIPPER_FINGER()
%
%   Basis: Zhu et al., Sensors 22(12):4532 (2022), PMC9230056. The 8-chamber Dragon Skin 20 finger is
%   modelled as N_Joint revolute hinges, one per chamber (REQ_GRIP_003). Values marked IDENTIFIED come
%   from fit_zhu_finger_prb.m (cubic spring, Table 1, 29 points, RMS 0.050 N, 2026-10-23). Values marked
%   EST are not published; each is swept (soft-gripper-test-plan.md section 6). Units SI.

finger.N_Joint = 8;                 % -        Zhu: 8 chambers                          REQ_GRIP_003
finger.L = 0.1050;                  % m        IDENTIFIED (not published)
finger.D = 1.594e-6;                % m^3/rad  IDENTIFIED: per-hinge displacement of the Rotational
                                    %          Mechanical Converter (G); torque = D*(p - p_atm)
finger.K = 0.1038;                  % N*m/rad  IDENTIFIED: linear hinge stiffness
finger.K3 = 4.446;                  % N*m/rad^3 IDENTIFIED: cubic stiffening (Simulink correction torque)
finger.p0 = 13.0e3;                 % Pa       IDENTIFIED: wall pre-stretch pressure offset
% Wall pre-stretch p0 is modelled as a PRESSURE DEAD ZONE, exactly as in the identification: net air torque
% per hinge = D*max(p_gauge - p0, 0), the converter's D*p_gauge plus a correction -D*min(p_gauge, p0) from a
% sensed pressure. The spring rests at 0 and there is NO hard stop, so the finger can bend back elastically
% under an external push. (Changed 2026-10-23 after T-F2: the earlier spring-offset + hard-stop form matched
% free bending but forbade the S-shape the identified model takes under the gauge.) Phenomenological: the
% pre-stretch band absorbs gas work without producing torque, standing in for the silicone wall stretch.
finger.l_Link = finger.L / finger.N_Joint;            % m    derived

% ---- not published: estimates, swept --------------------------------------------------------------
finger.m = 15e-3;                   % kg       EST 10-20 g (soft-arm-picking-sourcing.md)
finger.m_Link = finger.m / finger.N_Joint;            % kg   derived
finger.w_Link = 20e-3;              % m        EST cross-section width, along the hinge axis (link Z)
finger.h_Link = 15e-3;              % m        EST cross-section thickness, along the bend direction (link Y)
finger.V0 = 10e-6;                  % m^3      EST dead volume per finger (5-15 mL); the fit identifies
                                    %          only the swept part, D*sum(theta) = 2.9 mL at full bend
% Hinge damping, EST via an assumed damping ratio of the first bending mode (no dynamic data exists):
% one-DOF equivalent: N springs in series K/N, inertia of a uniform rod about its base m*L^2/3.
finger.zeta1 = 0.3;                 % -        EST, swept x0.3 .. x3
Ieq = finger.m * finger.L^2 / 3;
Keq = finger.K / finger.N_Joint;
finger.c = finger.N_Joint * 2 * finger.zeta1 * sqrt(Keq * Ieq);   % N*m*s/rad per hinge, derived from EST

% ---- mounting on the flange -------------------------------------------------------------------------
finger.n_Fingers = 4;               % -        Zhu's gripper (engineer decision 2026-10-23)
% Mounting radius (EST; Zhu give no gripper dimensions). Derived so the largest object fits between the straight
% fingers: mango horizontal semi-axis 47 mm + half the link thickness 7.5 mm + 5 mm clearance = 59.5 -> 60 mm.
% (Was 25 mm until 2026-10-23: then only objects under 17.5 mm radius fitted.) The fingertip still reaches the
% axis: free-bend tip moves inward about 72 mm at 60 kPa.
finger.r_Mount = 60e-3;             % m        EST, derived above
finger.isRingReinforced = false;    % -        engineer: plain fingers in v1
end
