function [S, M_EF] = DefineScrew(L1,L2,L3,L4,L5)

% 230126 HBY
% Input
% Li: link length of robot arm (PUMA-type)
% S: screw of each joint axis
% M_EF: SE(3) of end effector at zero position

n_joint = 6;

% Joint positions when qi = 0
joint_q = cell(n_joint,1);
joint_q{1} = [0;0;0];
joint_q{2} = [0;0;L1];
joint_q{3} = [0;0;L1+L2];
joint_q{4} = [L4/2;0;L1+L2+L3];
joint_q{5} = [L4;0;L1+L2+L3];
joint_q{6} = [L4;0;L1+L2+L3-L5];

% Joint screws when qi = 0
S = zeros(6,6);
joint_w = cell(n_joint,1);
joint_w{1} = [0;0;1];
joint_w{2} = [0;1;0];
joint_w{3} = [0;1;0];
joint_w{4} = [1;0;0];
joint_w{5} = [0;1;0];
joint_w{6} = [0;0;-1];
for i=1:6
    S(:,i) = [joint_w{i} ; - cross(joint_w{i},joint_q{i},1)];
end

% EF frame's SE(3) when qi = 0
EF_q = [L4;0;L1+L2+L3-L5];
M_EF = [roty(pi) EF_q;zeros(1,3) 1];

end