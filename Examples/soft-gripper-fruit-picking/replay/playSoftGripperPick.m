function out = playSoftGripperPick(objId, fps, view, tStop)
% PLAYSOFTGRIPPERPICK  Replay a logged pick in Unreal (SoftGripperScene3D.slx, Simulation 3D Robot). REQ_GRIP_029.
%
%   out = PLAYSOFTGRIPPERPICK(3)                    the apple pick from results/pick_3.mat, 30 frames/s
%   out = PLAYSOFTGRIPPERPICK(3, 30, "close")       close-up of the grasp (images/<fruit>_pick_close*)
%
%   Reads the logged results only (softgripperReplayData) and changes no simulated value: the scene model has no
%   path back to SoftGripperPick. Per-fruit shape and colour go into the Fruit actor's setup script through
%   SimulationInput (the block's setup script accepts literals only); the model file is not modified.

arguments
    objId (1,1) double = 3
    fps (1,1) double = 30
    view (1,1) string = "wide"      % "wide" or "close" (the grasp)
    tStop double = []               % shorter replay (s), e.g. to check one moment
end
replay = softgripperReplayData(objId, fps, view);
if isempty(tStop) && view == "close", tStop = 3.4; end   % the close-up ends after the lift (the arm then leaves)
if ~isempty(tStop), replay.tEnd = min(replay.tEnd, tStop); end
in = Simulink.SimulationInput('SoftGripperScene3D');
in = in.setVariable('replay', replay);
script = sprintf(['createShape(Actor, ''sphere'', [%.4f %.4f %.4f]);\nActor.Color = [%.2f %.2f %.2f];\n' ...
    'Actor.Mobility = sim3d.utils.MobilityTypes.Movable;'], replay.objSize, replay.objColor);
in = in.setBlockParameter('SoftGripperScene3D/Fruit', 'InitScriptText', script);
in = in.setBlockParameter('SoftGripperScene3D/Table', 'InitScriptText', sprintf( ...
    'createShape(Actor, ''box'', [%.3f %.3f %.3f]);\nActor.Color = [0.55 0.40 0.28];', replay.tableSize));
in = in.setBlockParameter('SoftGripperScene3D/Tray', 'InitScriptText', sprintf( ...
    'createShape(Actor, ''box'', [%.3f %.3f %.3f]);\nActor.Color = [0.20 0.35 0.60];', replay.traySize));
in = in.setModelParameter('StopTime', num2str(replay.tEnd));
fprintf('Replaying the %s pick (%.2f s, %d frames) in Unreal...\n', replay.name, replay.tEnd, numel(replay.cfg.Time));
out = sim(in);
% ---- frames -> images/<fruit>_pick.gif and four key PNGs (start, grasp, carry, release) ---------------------------
imgDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'images');
if ~isfolder(imgDir), mkdir(imgDir); end
fr = out.camFrames; F = fr.Data; tF = fr.Time;
nm = strrep(replay.name, " ", "_");
tag = ""; if view == "close", tag = "_close"; end
gifFile = fullfile(imgDir, nm + "_pick" + tag + ".gif");
skip = 2;                                     % the first frames show the previous pose (one-frame lag)
[~, map] = rgb2ind(F(:, :, :, round(size(F, 4) / 2)), 128, 'nodither');   % one shared palette: no speckle
for k = skip+1:size(F, 4)
    A = rgb2ind(F(:, :, :, k), map, 'nodither');
    if k == skip+1, imwrite(A, map, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', replay.camTs);
    else, imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', replay.camTs); end
end
keyT = min([0.4, 2.6, 4.0, 5.4], replay.tEnd);
for i = 1:numel(keyT)
    [~, k] = min(abs(tF - keyT(i)));
    f = fullfile(imgDir, sprintf('%s_pick%s_%d.png', nm, tag, i));
    try, imwrite(F(:, :, :, k), f); catch err, warning('playSoftGripperPick:png', 'could not write %s (%s)', f, err.message); end
end
% contact sheet: 12 frames across the pick, to check the motion (e.g. the gripper's orientation) at a glance
idx = round(linspace(skip+1, size(F, 4), 12));
try, imwrite(imtile(F(:, :, :, idx), 'GridSize', [3 4]), fullfile(imgDir, nm + "_pick" + tag + "_sheet.png")); catch, end
fprintf('Wrote %s and 4 key frames (%d frames).\n', gifFile, size(F, 4) - skip);
end
