function mirrored = hmmMirrorTree(robot, paint)
%HMMMIRRORTREE Reflect a rigidBodyTree through its y = 0 plane, for rendering only.
%
%   mirrored = HMMMIRRORTREE(robot) returns a copy of robot in which every body
%   frame is reflected through the base y = 0 plane, so that
%
%       FK_mirrored(q) == M * FK_robot(q) * M,      M = diag([1 -1 1 1])
%
%   for EVERY configuration q, with the same q driving both trees and the same body
%   names throughout.
%
%   mirrored = HMMMIRRORTREE(robot, paint) also repaints it (paint from hmmRobotPaint).
%   paint.colours is an N-by-2 cell of {body-name regular expression, RGB}, first match
%   wins, and a body that matches no row keeps its source colour; paint.split gives a body
%   one visual per material from a label per triangle. It is done HERE because this is the
%   one place every render mesh is re-added, and addVisual takes its colour only then.
%
%   WHY THIS EXISTS. Simulation 3D Robot negates y on its way to Unreal - it stores
%   each link's transform conjugated by M, which is a correct right-handed to
%   left-handed conversion. Every other actor in this model, however, takes its
%   Translation AS GIVEN: hmmSpawnProps passes raw manifest coordinates, and
%   Plant/CameraRig passes raw forward-kinematic translations. So the scene is
%   already in MATLAB coordinates and the robot block's conversion is one frame
%   change too many. Handing the block this mirrored tree cancels it exactly, and
%   the rendered robot then stands where the model commanded it.
%
%   WHAT THIS REPLACES. The defect used to be patched at the base only, by
%   multiplying q(1:3) by cfg.render.baseRenderFlip = [1 -1 -1]. That works for the
%   three base degrees of freedom and CANNOT work for the other twelve, because
%   those are joint ANGLES and no scalar multiplier cancels a frame change. The
%   result was a correctly placed chassis carrying a mirror-image arm: the error is
%   2*|y| in chassis-local coordinates, so it vanishes for a centred arm and grows
%   as the tool reaches sideways, which is precisely what a grasp does. Measured at
%   the grasp instant the rendered tool sat 76.6 mm from the commanded pose, against
%   7.5 mm of jaw clearance per side. It is also why refining the grasp point made
%   the grasp worse rather than better: the coarse point's own lateral error partly
%   cancelled the mirror, and removing that error removed the cancellation.
%
%   THE ONE SIGN THAT IS NOT A CHOICE. A prismatic axis is a true vector and maps to
%   M*a. A revolute axis is a PSEUDOVECTOR: conjugating a rotation by a reflection
%   gives a rotation about det(M)*M*a, and det(M) = -1, so a revolute axis maps to
%   -M*a. Getting that backwards produces a tree that agrees at the home
%   configuration and disagrees everywhere else - the most expensive kind of wrong -
%   which is why the mirror was checked at random configurations rather than one pose.
%
%   THE VISUAL MESHES ARE MIRRORED TOO, AND THEY HAVE TO BE. An earlier version of
%   this function mirrored link frames and carried the geometry across untouched,
%   arguing that mesh handedness was cosmetic. It is not, and the render said so: the
%   gripper read as detached from the wrist, one arm link and all four wheels visibly
%   displaced. The reason is that the block reflects the WHOLE assembled robot, meshes
%   included, so the drawn geometry is
%
%       P_drawn = M * FK_fed * T_visual * V
%
%   and mirroring only FK leaves M sitting on T_visual*V - each mesh reflected about
%   its own body y = 0 plane. That is invisible on a mesh symmetric about that plane
%   and glaring on one that is not, which is exactly the pattern observed: the four
%   wheel meshes span y in [-0.046, 0], so a reflection throws each disc to the far
%   side of its own hub (47 mm), the gripper knuckles move 42-49 mm, wrist_1_link 33 mm
%   and shoulder_link 26 mm, while the chassis, laser, pedestal, upper arm, forearm,
%   finger pads and camera barrel - all symmetric in y - move under 7 mm.
%
%   So this function mirrors the geometry as well, making the returned tree a COMPLETE
%   reflection rather than a half one. That is what makes it provable without knowing
%   what the block does internally: substituting a complete mirror into the expression
%   above gives
%
%       M * (M*FK*M) * (M*T*M) * (M*V) = FK * T * V
%
%   the true robot exactly, for any q. The old half-mirror instead left FK*(M*T*V).
%
%   HOW, GIVEN THAT VISUALS ARE READ-ONLY. Tform cannot be assigned ("Setting the
%   'Tform' property of class 'RigidBodyGeometry' is not supported"), Vertices cannot
%   be assigned, and copy() SHARES the geometry handles with the source body - so
%   mutating them in place would corrupt the real robot. The only writable route is
%   clearVisual followed by addVisual, and addVisual accepts a mesh only as a FILENAME:
%   neither a vertex matrix nor a triangulation, and it rejects a negative Scale
%   ("Expected scales to be positive"), which would otherwise have made this a one-line
%   change. Reflected meshes are therefore written to disk once and cached. clearVisual
%   on the copy provably leaves the source body's visuals intact, which is checked
%   below rather than trusted.
%
%   THE REFLECTION IS PUT IN THE VERTICES, NOT IN THE TRANSFORM. addVisual does accept
%   a tform with det = -1 and stores it verbatim, so M*T with untouched vertices is
%   algebraically just as correct and needs no files at all. It is not used, because the
%   block converts transforms by decomposing them into Euler angles
%   (robotics.slsim3d.internal.util.getInUnrealForm calls sim3d.utils.Math.decomp321),
%   and a reflection has no Euler representation - it would be silently rounded to some
%   rotation. Vertices carry a reflection safely; 4x4 transforms in this pipeline do
%   not. So T' = M*T*M, whose determinant is back to +1, and V' = M*V.
%
%   WINDING IS NOT FLIPPED, AND THAT IS THE EASY MISTAKE HERE. Reflecting a triangle
%   reverses its normal, so the obvious move is to reverse each face's vertex order to
%   put the normals back. That is right for ONE reflection and wrong for this chain,
%   which has TWO: the one applied here and the block's own. Two reflections restore the
%   orientation on their own, so an explicit reversal is one flip too many and leaves the
%   DRAWN facets backwards - geometrically in the right place and lit from the wrong
%   side. A revision of this function did exactly that. The measurement that caught it
%   is worth keeping: addVisual flips winding as it loads a mesh file, uniformly, so all
%   27 source visuals store a NEGATIVE signed volume, and with the reversal in place all
%   27 mirrored visuals stored a positive one - a single net flip where the correct
%   answer is none. It is the same class of error as the defect this whole function
%   exists to cancel: an odd number of frame changes.
%
%   WHAT IS STILL NOT MIRRORED. Collision geometry, which the block only draws when
%   enableCollisionVisualization is on (it is off) and which nothing in this project
%   uses from this tree; and the inertia tensor, as noted below.
%
%   RENDER ONLY. The returned tree is a mirror image and is meaningless as kinematics:
%   its joint angles do not mean what they say and its frames are left-handed. It
%   must reach nothing but the Simulation 3D Robot block's RigidBodyTree parameter.
%   Control, planning, perception, the safety workspace check and every FK
%   measurement keep using the true tree from assembleMobileManipulator, or the
%   mirror simply moves somewhere harder to see.
%
%   See also assembleMobileManipulator, hmmSetup.

%   Copyright 2026 The MathWorks, Inc.

arguments
    robot {mustBeA(robot, 'rigidBodyTree')}
    paint struct = struct('colours', {cell(0, 2)}, 'split', struct())
end

Mh = diag([1 -1 1 1]);
M = Mh(1:3, 1:3);

% Located relative to this file rather than to pwd or to a cfg field, because
% hmmSetup runs this during model initialisation, when the current folder is whatever
% the user left it at.
cacheDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
    'robots', 'meshes', 'mirrored');

mirrored = rigidBodyTree('DataFormat', robot.DataFormat, ...
    'MaxNumBodies', robot.NumBodies + 8);
mirrored.BaseName = robot.BaseName;
mirrored.Gravity = robot.Gravity;

for k = 1:numel(robot.Bodies)
    src = robot.Bodies{k};
    sj = src.Joint;

    % ONLY JointToParentTransform IS CONJUGATED, and that is sufficient rather than
    % sloppy: both fixed transforms are read-only properties reachable only through
    % setFixedTransform, which writes the parent-side one, and a URDF import puts all
    % the geometry there. ChildToJointTransform is the identity on every body of this
    % tree, and M*I*M is I, so there is nothing to carry across. A future revision
    % could break that silently, so it is checked rather than assumed.
    if max(max(abs(sj.ChildToJointTransform - eye(4)))) > 1e-12
        error('hmm:mirrorTree:childToJoint', ...
            ['Body ''%s'' has a non-identity ChildToJointTransform, which cannot ' ...
             'be mirrored because the property is read-only. Fold it into ' ...
             'JointToParentTransform in assembleMobileManipulator first.'], src.Name);
    end

    % THE BODY IS COPIED, NOT REBUILT, AND THAT IS THE WHOLE POINT OF THIS LINE.
    % rigidBody(name) starts with no visuals and no collisions, and an earlier version
    % of this function built bodies that way and silently produced a 37-body tree
    % carrying 0 of the robot's 27 meshes. That is not a cosmetic loss: the block's Mesh
    % parameter is 'RigidBodyTree', so it renders the tree's own visuals, and its
    % RigidBodyTreeCallback REJECTS a tree with none - "Error evaluating
    % 'MaskParameterCallback'", with no mention of meshes. copy() carries Visuals,
    % Collisions, Mass and Inertia across, and survives the joint swap below because
    % those live on the body rather than on the joint. The visuals it carries are then
    % replaced with reflected ones a few lines down.
    nb = copy(src);
    nj = rigidBodyJoint(sj.Name, sj.Type);
    if ~strcmp(sj.Type, 'fixed')
        a = sj.JointAxis(:);
        if strcmp(sj.Type, 'prismatic')
            nj.JointAxis = (M * a)';
        else
            nj.JointAxis = (-M * a)';
        end
        % Limits are copied verbatim. They still bound the same q, because the same q
        % drives both trees; what changes is which physical motion that q describes,
        % and this tree is never asked that question.
        nj.PositionLimits = sj.PositionLimits;
        nj.HomePosition = sj.HomePosition;
    end
    setFixedTransform(nj, Mh * sj.JointToParentTransform * Mh);
    nb.Joint = nj;

    % The centre of mass is a point in the body frame, and that frame is reflected, so
    % it is reflected too. Nothing in this project does dynamics with this tree, so
    % this changes no result today; it is here so that a tree calling itself a mirror
    % is not carrying one unmirrored coordinate for someone to find later. Inertia is
    % deliberately NOT touched: the correct transform is R*I*R' and getting a tensor
    % half-right would be worse than leaving it plainly untransformed with this note.
    nb.CenterOfMass = (M * src.CenterOfMass(:))';

    % ---- the geometry, reflected -------------------------------------------------
    % getVisual is the public read that makes this possible at all: it returns the
    % triangulation, the transform, the scale and the RGBA colour, none of which is
    % reachable through the Visuals property itself.
    if ~isempty(src.Visuals)
        gv = getVisual(src);
        tint = [];
        for r = 1:size(paint.colours, 1)
            if ~isempty(regexp(src.Name, paint.colours{r, 1}, 'once'))
                tint = paint.colours{r, 2};
                break
            end
        end
        clearVisual(nb);
        for j = 1:numel(gv)
            tri = gv(j).Triangulation;
            P = double(tri.Points);
            P(:, 2) = -P(:, 2);
            % WINDING IS DELIBERATELY LEFT ALONE. Reflecting a triangle does reverse its
            % normal, and reversing the vertex order would undo that - but there are TWO
            % reflections in this chain, not one: this one and the block's own. Two
            % reflections restore the orientation by themselves, so reversing the winding
            % here is one flip too many and leaves the DRAWN facets backwards. That was
            % the state for one revision, and it is measurable: with the reversal in, the
            % mirrored bodies stored a signed volume of the opposite sign to every one of
            % the 27 source visuals, uniformly. It is the same class of error as the bug
            % this whole function exists to fix - an odd number of frame changes - which
            % is the reason for stating it here rather than just deleting a line.
            F = tri.ConnectivityList;
            rgb = gv(j).Color(1:3);
            if ~isempty(tint)
                rgb = tint;
            end
            % ONE VISUAL PER MATERIAL where the body carries a label for every triangle of
            % this very mesh (paint.split, from robots/meshes/ur5e/materials.mat). The block
            % merges a body's visuals into one actor and colours each by VERTEX, so this is
            % how a UR5e link gets its blue cap and black ring. A count that no longer
            % matches means the installed mesh changed, and the link keeps one colour.
            parts = {(1:size(F, 1))'};
            partRgb = rgb;
            if isscalar(gv) && isfield(paint.split, src.Name) && ...
                    numel(paint.split.(src.Name).labels) == size(F, 1)
                lab = double(paint.split.(src.Name).labels(:));
                used = unique(lab)';
                parts = arrayfun(@(m) find(lab == m), used, 'UniformOutput', false);
                partRgb = paint.split.(src.Name).rgb(used, :);
            end
            for p = 1:numel(parts)
                if isscalar(parts)
                    meshFile = cachedMirrorMesh(cacheDir, src.Name, j, P, F);
                else
                    [u, ~, ic] = unique(F(parts{p}, :));
                    meshFile = cachedMirrorMesh(cacheDir, src.Name, 10 * j + p, P(u, :), ...
                        reshape(ic, [], 3));
                end
                % SCALE IS DELIBERATELY UNITY RATHER THAN gv(j).Scale. Nine of this robot's
                % visuals report Scale = 0.001 (the Robotiq meshes, authored in millimetres),
                % but the vertices getVisual hands back are already in metres - the scale has
                % been applied, and the property is a record of that rather than a pending
                % multiplier. The reflected mesh written above is therefore already at final
                % size, and passing 0.001 again would render the gripper a thousand times
                % too small. The read-back check below is what actually proves this.
                addVisual(nb, 'Mesh', {meshFile, [1 1 1]}, Mh * gv(j).Tform * Mh, ...
                    'FaceColor', partRgb(p, :), 'FaceAlpha', gv(j).Color(4));
            end
        end
        % copy() SHARES the RigidBodyGeometry handles with src, so this is checked
        % rather than assumed: if clearVisual had operated on the shared list, the real
        % robot would now be a tree with no meshes and the failure would surface far
        % from here, as a mask callback error on a model edit.
        if numel(src.Visuals) ~= numel(gv)
            error('hmm:mirrorTree:sourceMutated', ...
                ['Mirroring body ''%s'' removed visuals from the SOURCE tree (%d left ' ...
                 'of %d). copy() shares geometry handles; the source robot is now ' ...
                 'damaged and must be rebuilt with assembleMobileManipulator.'], ...
                src.Name, numel(src.Visuals), numel(gv));
        end
    end

    addBody(mirrored, nb, src.Parent.Name);
end

end

% ---------------------------------------------------------------------------------
function meshFile = cachedMirrorMesh(cacheDir, bodyName, idx, P, F)
%CACHEDMIRRORMESH Path to a reflected mesh on disk, written on first request.
%   addVisual takes a mesh only as a filename, so the reflected geometry has to exist
%   as a file. The whole set is about 24 MB across 27 meshes (471075 faces), which is
%   why it is cached rather than rewritten on every setup, and why the directory is
%   generated rather than committed.
%
%   STALENESS IS HANDLED BY THE NAME, NOT BY A TIMESTAMP. The digest of the reflected
%   geometry is part of the filename, so changing a mesh, a scale or a primitive's
%   dimensions asks for a file that does not exist yet and it is written. There is no
%   cache to invalidate and no way to silently render last week's robot. Old files are
%   left behind rather than deleted, because deleting by pattern in a directory whose
%   path is assembled at run time is a worse risk than a few stale megabytes.
if ~isfolder(cacheDir)
    mkdir(cacheDir);
end
meshFile = fullfile(cacheDir, sprintf('%s_%d_%s.stl', bodyName, idx, geomDigest(P, F)));
if ~isfile(meshFile)
    stlwrite(triangulation(F, P), meshFile);
end
end

% ---------------------------------------------------------------------------------
function d = geomDigest(P, F)
%GEOMDIGEST Short order-sensitive digest of a mesh, used only to name a cache file.
%   Not a cryptographic hash and not trying to be: a collision would mean reusing a
%   correct-sized mesh for a different body, and the vertex counts in the name make
%   that essentially impossible. Every product and every partial sum below is kept
%   under 2^53 so the result is exact and reproducible across sessions - which matters,
%   because an inconsistent digest would rewrite 24 MB on every setup.
p = 2^31 - 1;
u = double(typecast(single(P(:))', 'uint32'));
c = double(F(:)');
h = mod(sum(mod(u .* (1 + mod(0:numel(u) - 1, 251)), p)), p);
h = mod(h + sum(mod(c .* (1 + mod(0:numel(c) - 1, 257)), p)), p);
d = sprintf('%dv%df%s', size(P, 1), size(F, 1), lower(dec2hex(h, 8)));
end
