function robot = robotWith6DoFFloatingBase(dataformat)
%robotWith6DoFFloatingBase Build a rigidBodyTree with a 6-DOF serial floating base.
%
%   robot = robotWith6DoFFloatingBase(dataformat) creates a rigidBodyTree
%   with 6 massless bodies connected by 1-DOF joints (3 prismatic + 3
%   revolute) to emulate a floating base. This allows the Simulation 3D
%   Robot block to accept a flat configuration vector:
%     [x, y, z, roll, pitch, yaw, q1, ..., q12]
%
%   The serial chain replaces a single floating joint (unsupported by
%   Sim3D) with equivalent kinematics:
%     world -> X_trans -> Y_trans -> Z_trans -> X_rev -> Y_rev -> Z_rev
%
%   Input:
%     dataformat - 'row' or 'column' (default: 'column')
%
%   Output:
%     robot - rigidBodyTree with 6 zero-mass bodies. Attach the actual
%             robot subtree to 'baseZRevBody' using addSubtree.

robot = rigidBodyTree();
robot.BaseName = 'world';
if nargin < 1
    dataformat = 'column';
end

robot.DataFormat = dataformat;
robot.Gravity = [0 0 -9.81];

jointAxisName = {'X', 'Y', 'Z'};
jointAxisValue = eye(3);

% Initialize parent name
parentBodyName = robot.BaseName;

% Add the prismatic joints
for i = 1:numel(jointAxisName)
    bodyName = ['base' jointAxisName{i} 'TransBody'];
    jointName = ['base' jointAxisName{i} 'TransJoint'];
    rb = rigidBody(bodyName);
    rb.Mass = 0;
    rb.Inertia = [0 0 0 0 0 0];
    rbJoint = rigidBodyJoint(jointName, 'prismatic');
    rbJoint.JointAxis = jointAxisValue(i,:);
    rbJoint.PositionLimits = [-inf inf];
    rb.Joint = rbJoint;
    
    % Add to robot using previous body as parent
    robot.addBody(rb, parentBodyName);
    parentBodyName = rb.Name; % Update parent body name
end

% Add the revolute joints
for i = 1:numel(jointAxisName)
    bodyName = ['base' jointAxisName{i} 'RevBody'];
    jointName = ['base' jointAxisName{i} 'RevJoint'];
    rb = rigidBody(bodyName);
    rb.Mass = 0;
    rb.Inertia = [0 0 0 0 0 0];
    rbJoint = rigidBodyJoint(jointName, 'revolute');
    rbJoint.JointAxis = jointAxisValue(i,:);
    rbJoint.PositionLimits = [-inf inf];
    rb.Joint = rbJoint;
    
    % Add to robot using previous body as parent
    robot.addBody(rb, parentBodyName);
    parentBodyName = rb.Name; % Update parent body name
end

end