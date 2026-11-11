function tree = softgripperFingerTree(P)
% SOFTGRIPPERFINGERTREE  One soft finger as a serial rigidBodyTree, for the 3D replay (REQ_GRIP_029).
%
%   tree = SOFTGRIPPERFINGERTREE(P)      P from softgripperPickParams
%
%   The Simulation 3D Robot block mis-draws BRANCHING trees (in the replay, one fingertip angle moved
%   the palm and the wrist twisted back and forth). So the replay uses one serial robot for the UR5e + palm and one
%   serial robot per finger, whose base is placed every frame at its mount on the palm (softgripperReplayData).
%   Finger frame: base = the mount, [X Y Z] = [tangential, outward radial, finger direction]; hinge i about X, so a
%   positive angle turns the finger inward as in the simulation; link i along Z, length L/N, box mesh visual.
%   Display only: nothing here feeds back into the simulation.

fi = P.finger;
l = fi.L / fi.N_Joint;
tree = rigidBodyTree('DataFormat', 'row');
meshDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'images', 'mesh');
linkStl = fullfile(meshDir, 'finger_link_box.stl');
if ~isfile(linkStl), softgripperReplayTree(P); end      % writes the STL meshes
parent = tree.BaseName;
for i = 1:fi.N_Joint
    link = rigidBody(sprintf('link%d', i));
    jnt = rigidBodyJoint(sprintf('hinge%d', i), 'revolute');
    jnt.JointAxis = [1 0 0];
    if i > 1, setFixedTransform(jnt, trvec2tform([0 0 l])); end
    link.Joint = jnt;
    addVisual(link, 'Mesh', linkStl, trvec2tform([0 0 l/2]));
    m = fi.m_Link; sz = [fi.w_Link fi.h_Link l]; c = [0 0 l/2];
    I = m / 12 * [sz(2)^2 + sz(3)^2, sz(1)^2 + sz(3)^2, sz(1)^2 + sz(2)^2];
    link.Mass = m; link.CenterOfMass = c;
    link.Inertia = [I + m * [c(3)^2, c(3)^2, 0], 0, 0, 0];
    addBody(tree, link, parent);
    parent = link.Name;
end
end
