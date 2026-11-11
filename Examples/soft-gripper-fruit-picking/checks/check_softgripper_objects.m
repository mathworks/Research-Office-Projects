function R = check_softgripper_objects(ids)
% CHECK_SOFTGRIPPER_OBJECTS  M2 object/contact validation on ObjectBench.slx (T-O1).
%
%   R = CHECK_SOFTGRIPPER_OBJECTS()        all six objects
%   R = CHECK_SOFTGRIPPER_OBJECTS([1 6])   a subset (objId)
%
%   T-O1  each object (ObjectBody: 6-DOF joint + ellipsoid, mass and semi-axes from params_softgripper_objects)
%         is released at rest 5 mm above a rigid table (Spatial Contact Force, k_Table, c_Table, smooth stick-slip).
%         Prediction: static penetration m*g/k_Table. Pass: settles < 0.5 s; penetration within 20 % of m*g/k.
%   Settled = from then on |vz| < 1 mm/s and |z - z_end| < 10 % of the predicted penetration.
%   Read-only on the models; does not cd; SimulationInput only.

arguments
    ids double = 1:6
end
objs = params_softgripper_objects();
bench = struct('tTable', 0.02, 'wContact', 1e-5, 'drop', 5e-3);
g = 9.80665;
fprintf('\n=== T-O1 drop %.0f mm onto the table ===\n', bench.drop*1e3);
in = Simulink.SimulationInput.empty;
for k = ids
    o = objs(k);
    o.posInit = [0 0 o.semiAxes(3) + bench.drop];
    in(end+1) = Simulink.SimulationInput('ObjectBench'); %#ok<AGROW>
    in(end) = in(end).setVariable('P', struct('obj', o, 'bench', bench));
    in(end) = in(end).setModelParameter('StopTime', '1');
    fprintf('  PREDICTION %-13s m %6.1f g, k %6.0f N/m: penetration %.3f mm, contact force %.3f N\n', ...
        o.name, o.mass*1e3, o.k_Table, o.mass*g/o.k_Table*1e3, o.mass*g);
end
out = sim(in, 'ShowProgress', 'off');
R = struct('name', {}, 'pen', {}, 'pred', {}, 'tSettle', {}, 'force', {}, 'pass', {});
for j = 1:numel(ids)
    o = objs(ids(j));
    z = out(j).logsout.get('zObj').Values; v = out(j).logsout.get('vzObj').Values; f = out(j).logsout.get('fTable').Values;
    pred = o.mass * g / o.k_Table;
    pen = o.semiAxes(3) - z.Data(end);
    bad = abs(v.Data(:)) >= 1e-3 | abs(z.Data(:) - z.Data(end)) >= 0.1 * pred;
    kLast = find(bad, 1, 'last');
    if isempty(kLast), tS = 0; else, tS = z.Time(min(kLast + 1, numel(z.Time))); end
    e = (pen - pred) / pred;
    ok = tS < 0.5 && abs(e) <= 0.20;
    fprintf('  %-13s penetration %.3f mm (%+5.1f %%), settles at %.3f s, force %.3f N (m*g %.3f) -> %s\n', ...
        o.name, pen*1e3, 100*e, tS, f.Data(end), o.mass*g, passStr(ok));
    R(j) = struct('name', o.name, 'pen', pen, 'pred', pred, 'tSettle', tS, 'force', f.Data(end), 'pass', ok);
end
fprintf('  T-O1: %d of %d PASS\n', nnz([R.pass]), numel(R));
end

function s = passStr(ok)
s = 'FAIL'; if ok, s = 'PASS'; end
end
