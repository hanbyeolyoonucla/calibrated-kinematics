function T_ST = ForwardKinematics(S,q,M_EF)

% 230126 HBY
% Input
% S: Screw of each joint [S1 S2 ... S6] 6x6 matrix
% q: joint angles 6x1 matrix
% M_EF: SE(3) of end effector at zero position
% Output
% T_BT: SE(3) of end effector with given joint angles q

n_joint = 6;
POE = eye(4);
q(5) = q(5) - pi/2;

% POE e^[Si]qi
for i=1:n_joint
    POE = POE*OneScrewtoSE3(S(:,i),q(i));
end

% End effector position and orientation
T_ST = POE*M_EF;
end