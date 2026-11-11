function nFail = verify_softgripper_hierarchy
% VERIFY_SOFTGRIPPER_HIERARCHY  Assert SoftGripperPick.slx against its frozen interface.
%
%   nFail = VERIFY_SOFTGRIPPER_HIERARCHY()
%
%   Read-only: loads the model if needed, changes nothing, does not cd. Checks:
%     - root and Controller/Plant ports match the interface frozen at M1 (2026-10-23; see the README diagram)
%     - REQ_GRIP_021/022: the Controller's inputs come ONLY from the Plant's y outputs and the root
%       objPoseInit; zTruth goes ONLY to the root logging outport (traced through the actual lines)
%     - finite StopTime; code-generation-safe block names
%     - placeholder census (Ground/Terminator blocks whose names end in a phase tag), so "skeleton" is a number

mdl = 'SoftGripperPick';
if ~bdIsLoaded(mdl), load_system(mdl); end
nFail = 0;
fprintf('\n=========== SoftGripperPick vs the frozen interface ===========\n');

ports = @(sys, t) string(get_param(find_system(sys, 'SearchDepth', 1, 'BlockType', t), 'Name'))';
ctl = [mdl '/Controller']; pl = [mdl '/Plant'];
nFail = nFail + claim('root inputs = {objPoseInit, pSupplyGrip}', isequal(sort(ports(mdl,'Inport')), sort(["objPoseInit","pSupplyGrip"])));
nFail = nFail + claim('root output = {zTruth}', isequal(ports(mdl,'Outport'), "zTruth"));
nFail = nFail + claim('Controller in = {qArmMeas, pGripMeas, bendMeas, objPoseInit}', ...
    isequal(sort(ports(ctl,'Inport')), sort(["qArmMeas","pGripMeas","bendMeas","objPoseInit"])));
nFail = nFail + claim('Controller out = {qArmCmd, pGripCmd}', isequal(sort(ports(ctl,'Outport')), sort(["qArmCmd","pGripCmd"])));
nFail = nFail + claim('Plant in = {qArmCmd, pGripCmd, pSupplyGrip}', isequal(sort(ports(pl,'Inport')), sort(["qArmCmd","pGripCmd","pSupplyGrip"])));
nFail = nFail + claim('Plant out = {qArmMeas, pGripMeas, bendMeas, zTruth}', ...
    isequal(sort(ports(pl,'Outport')), sort(["qArmMeas","pGripMeas","bendMeas","zTruth"])));

% ---- truth isolation, traced through the lines ----------------------------------------------------------
ph = get_param(ctl, 'PortHandles'); okSrc = true; srcs = strings(0);
for k = 1:numel(ph.Inport)
    ln = get_param(ph.Inport(k), 'Line');
    srcPort = get_param(ln, 'SrcPortHandle'); srcBlk = get_param(get_param(srcPort, 'Parent'), 'Name');
    srcNum = get_param(srcPort, 'PortNumber');
    if strcmp(srcBlk, 'Plant')
        names = ports(pl, 'Outport');                       % map port number -> Outport name
        ob = find_system(pl, 'SearchDepth', 1, 'BlockType', 'Outport');
        nm = string(get_param(ob(str2double(get_param(ob, 'Port')) == srcNum), 'Name'));
        srcs(end+1) = "Plant." + nm; %#ok<AGROW>
        okSrc = okSrc && nm ~= "zTruth" && ismember(nm, names);
    else
        srcs(end+1) = string(srcBlk); %#ok<AGROW>
        okSrc = okSrc && strcmp(srcBlk, 'objPoseInit');
    end
end
nFail = nFail + claim('REQ_GRIP_021/022: Controller inputs come only from Plant y and objPoseInit', okSrc, strjoin(srcs, ', '));
php = get_param(pl, 'PortHandles');
ob = find_system(pl, 'SearchDepth', 1, 'BlockType', 'Outport');
zIdx = str2double(get_param(ob(strcmp(get_param(ob,'Name'),'zTruth')), 'Port'));
dst = get_param(get_param(php.Outport(zIdx), 'Line'), 'DstBlockHandle');
dstNames = string(get_param(dst, 'Name'))';
nFail = nFail + claim('REQ_GRIP_022: zTruth goes only to the root logging outport', isequal(dstNames, "zTruth"), strjoin(dstNames, ', '));

% ---- configuration and naming --------------------------------------------------------------------------
st = str2double(get_param(mdl, 'StopTime'));
nFail = nFail + claim('StopTime is finite', isfinite(st), sprintf('StopTime = %g s', st));
allBlk = find_system(mdl, 'LookUnderMasks', 'none', 'Type', 'Block');
bad = allBlk(~cellfun(@(n) ~isempty(regexp(n, '^[A-Za-z][A-Za-z0-9]*(_[A-Za-z0-9]+)*$', 'once')), get_param(allBlk, 'Name')));
nFail = nFail + claim('every block name is code-generation safe', isempty(bad), sprintf('%d offending', numel(bad)));

% ---- placeholder census -------------------------------------------------------------------------------------
stub = find_system(mdl, 'Regexp', 'on', 'Name', '^(Gnd|Term).*_(P1|P1d|P2|M3)$');
tags = regexprep(string(get_param(stub, 'Name')), '^.*_', '');
fprintf('--- placeholders still to build: %d (%s) ---\n', numel(stub), strjoin(compose("%s %d", unique(tags), arrayfun(@(t) nnz(tags == t), unique(tags))), ', '));
if nFail == 0
    fprintf('ALL CLAIMS VERIFIED -- the shell matches the frozen interface and truth isolation holds.\n');
else
    fprintf('*** %d CLAIM(S) FAILED ***\n', nFail);
end
end

function fail = claim(what, ok, detail)
if nargin < 3, detail = ''; end
fail = ~ok;
tag = 'PASS'; if fail, tag = 'FAIL'; end
fprintf('  %s  %-66s %s\n', tag, what, detail);
end
