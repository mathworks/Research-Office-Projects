function paint = hmmRobotPaint(cfg)
%HMMROBOTPAINT How the rendered robot is coloured, in the form hmmMirrorTree takes.
%
%   paint = HMMROBOTPAINT(cfg) returns
%
%       paint.colours   cfg.render.robotColor: {body-name regular expression, RGB} rows
%       paint.split     one field per UR5e link with a label per triangle (uint8, an index
%                       into the materials) and rgb, one row per material from
%                       cfg.render.armMaterial
%
%   The labels come from robots/meshes/ur5e/materials.mat.
%   Without that file every link keeps the one colour paint.colours gives it.
%
%   See also hmmMirrorTree, hmmConfig.

%   Copyright 2026 The MathWorks, Inc.

arguments
    cfg struct = hmmConfig()
end

paint = struct('colours', {cfg.render.robotColor}, 'split', struct());
f = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'robots', 'meshes', ...
    'ur5e', 'materials.mat');
if ~isfile(f)
    return
end
S = load(f, 'labels', 'materials');
rgb = zeros(numel(S.materials), 3);
for m = 1:numel(S.materials)
    rgb(m, :) = cfg.render.armMaterial.(S.materials{m});
end
for name = fieldnames(S.labels)'
    paint.split.(name{1}) = struct('labels', S.labels.(name{1}), 'rgb', rgb);
end
end
