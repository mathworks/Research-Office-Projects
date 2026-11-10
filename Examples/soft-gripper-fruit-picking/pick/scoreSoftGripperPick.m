function R = scoreSoftGripperPick(out, P, tStop)
% SCORESOFTGRIPPERPICK  Score one SoftGripperPick run (REQ_GRIP_016-018) from its SimulationOutput.
%
%   R = SCORESOFTGRIPPERPICK(out, P, tStop)
%   R.timeline (state@entry time), finalState ("Done" | "Failed" | "none"), tFinal, attempts, fMax (peak
%   per-finger contact, N), objEnd (final object position), inTray, slip (max object displacement in the tool0
%   frame between Lift entry and Release entry, m), tEnd. Used by runSoftGripperPick; callable on saved runs.

objs = params_softgripper_objects(); o = objs(P.objId);
R = scorePick(out, P, o, tStop);
end

function R = scorePick(out, P, o, tStop)
ls = out.logsout;
names = ["Settle","Approach","Descend","Close","CheckGrasp","Lift","Transport","Lower","Release","Retreat","Retry","Done","Failed"];
R.timeline = strings(0); R.tEnd = out.tout(end);
tEnter = nan(1, numel(names));
for k = 1:numel(names)
    nm = names(k); if ~any(nm == ["Done", "Failed"]), nm = "Active." + nm; end
    v = logVal(ls, nm);
    if isempty(v), continue, end
    on = find(v.Data(:) > 0, 1);
    if ~isempty(on), tEnter(k) = v.Time(on); end
end
[ts, ord] = sort(tEnter); ok = ~isnan(ts);
R.timeline = compose("%s@%.2f", names(ord(ok))', ts(ok)')';
R.finalState = "none"; R.tFinal = NaN;
if ~isnan(tEnter(names == "Done")), R.finalState = "Done"; R.tFinal = tEnter(names == "Done"); end
if ~isnan(tEnter(names == "Failed")), R.finalState = "Failed"; R.tFinal = tEnter(names == "Failed"); end
R.tRelease = tEnter(names == "Release");          % NaN if never released (REQ_GRIP_018: in the tray by then)
a = logVal(ls, "attempt"); if ~isempty(a), R.attempts = max(a.Data); else, R.attempts = NaN; end
z = logVal(ls, "zTruth"); zd = squeeze(z.Data); if size(zd, 1) ~= numel(z.Time), zd = zd.'; end
fC = zd(:, 1:4); pose = zd(:, 5:10);
R.fMax = max(fC, [], 'all');
R.objEnd = pose(end, 1:3);
sc = P.scene;
R.inTray = all(abs(R.objEnd(1:2) - sc.trayXY) < sc.wTray/2) && R.objEnd(3) < sc.trayZ + 2*o.semiAxes(3) + 5e-3;
% slip: object position in the tool0 frame between CheckGrasp exit (Lift entry) and Release entry
R.slip = NaN;
tA = tEnter(names == "Lift"); tB = tEnter(names == "Release");
if ~isnan(tA) && ~isnan(tB)
    rb = loadrobot('universalUR5e', 'DataFormat', 'row');
    q = logVal(ls, "qArmMeas");
    rel = @(t) relPos(rb, q, z.Time, pose, t);
    tt = linspace(tA, tB, 25); r0 = rel(tA); d = zeros(size(tt));
    for i = 1:numel(tt), d(i) = norm(rel(tt(i)) - r0); end
    R.slip = max(d);
end
R.tStop = tStop;
end

function v = logVal(ls, name)
% First logged element with this name (qArmMeas is logged twice: the root line and its delayed copy).
e = ls.find('Name', char(name));
if e.numElements == 0, v = []; else, v = e{1}.Values; end
end

function r = relPos(rb, q, tz, pose, t)
qd = squeeze(q.Data); if size(qd, 1) ~= numel(q.Time), qd = qd.'; end
qi = interp1(q.Time, qd, t, 'previous');
T = getTransform(rb, qi, 'tool0');
p = interp1(tz, pose(:, 1:3), t);
r = T(1:3, 1:3)' * (p(:) - T(1:3, 4));
end

