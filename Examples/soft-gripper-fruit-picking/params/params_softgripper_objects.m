function obj = params_softgripper_objects()
% PARAMS_SOFTGRIPPER_OBJECTS  The six modelled objects (REQ_GRIP_015, 016, 017).
%
%   obj = PARAMS_SOFTGRIPPER_OBJECTS()   struct array, one element per object; index = objId
%
%   Source: docs/soft-gripper-object-sourcing.md (per-object references). Masses from Zhu et al. 2022 Table 4
%   and Fig. 18c (raspberry estimated). EST = derived or inferred. Contact stiffness: fruit-only values are
%   halved because Dragon Skin 20 is about as soft as the fruit (combined series stiffness, EST).

combined = 0.5;     % -  EST: finger-fruit series contact stiffness factor
zeta_Contact = 0.5; % -  EST: contact damping ratio (c = 2*zeta*sqrt(k*m))
%            name           shape        semi-axes a b c (m)            mass (kg)  F_Bruise (N)  k_fruit (N/m)  mu_s
T = {"raspberry",     "ellipsoid", [0.0100 0.0100 0.00925],          0.0040,    3.0,          300,           1.0
     "cherry tomato", "sphere",    [0.0150 0.0150 0.0150],           0.0153,    2.5,          5000,          1.4
     "apple",         "sphere",    [0.0400 0.0400 0.0400],           0.2006,    40,           8000,          0.8
     "tomato",        "ellipsoid", [0.04225 0.04225 0.0340],         0.2829,    11,           4000,          1.4
     "pear",          "ellipsoid", [0.0530 0.0390 0.0390],           0.3891,    40,           8000,          0.8
     "mango",         "ellipsoid", [0.0710 0.0470 0.0395],           0.5803,    30,           21000,         0.8};
% Semi-axes are [x y z] in the world frame, z vertical. The pear and the mango LIE ON THEIR SIDE (shortest axis
% vertical, long axis along x), as they rest on a table. Upright (106 / 142 mm tall) the mango has no sideways
% grip (T-I4: 0.02 m/s^2 at 60 kPa) and the pear 1.45 m/s^2 at 58.3 kPa.
estFlags = {"size, mu", "size, F_Bruise", "k", "k", "size, F_Bruise, k, mu", "size, F_Bruise, k, mu"};
holdPressZhu = [NaN, 10e3, 30e3, 35e3, 45e3, 60e3];   % Pa  Zhu Table 4 / Fig. 18c (raspberry: not tested)
graspMode = ["fingertip", "fingertip", "enveloping", "enveloping", "enveloping", "enveloping"];

obj = struct([]);
for k = 1:size(T, 1)
    obj(k).name = T{k,1};
    obj(k).shape = T{k,2};
    obj(k).semiAxes = T{k,3};
    obj(k).mass = T{k,4};
    obj(k).F_Bruise = T{k,5};                      % REQ_GRIP_016
    obj(k).k_Contact = combined * T{k,6};          % N/m   finger-fruit (soft on soft)
    obj(k).k_Table = T{k,6};                       % N/m   fruit on the rigid table or tray
    obj(k).c_Table = 2 * zeta_Contact * sqrt(obj(k).k_Table * obj(k).mass);     % N/(m/s) derived
    obj(k).c_Contact = 2 * zeta_Contact * sqrt(obj(k).k_Contact * obj(k).mass); % N/(m/s) derived
    obj(k).mu_s = T{k,7};
    obj(k).mu_d = 0.8 * T{k,7};                    % EST: dynamic = 0.8 x static
    obj(k).estimated = estFlags{k};
    obj(k).p_HoldZhu = holdPressZhu(k);            % REQ_GRIP_013 reference
    obj(k).graspMode = graspMode(k);
end
end
