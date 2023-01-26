clc; clear; close all;

% fixed parameters
L1 = 0.135;
L2 = 0.135;
L3 = 0.038;
L4 = 0.120;
L5 = 0.070;

% forward kinematics to generate T_BT
q_FK = pi/3*ones(6,1);
[S, M_EF] = DefineScrew(L1,L2,L3,L4,L5);
T_ST = ForwardKinematics(S,q_FK,M_EF)

% inverse kinematics to test
[q_IK, eflag] = AnalyticIK(L1,L2,L3,L4,L5,T_ST);
T_ST_IK = ForwardKinematics(S,q_IK,M_EF)
disp( norm(T_ST-T_ST_IK))

% we should compute all possible IK solution maybe 8?
% figure out if there is no mistake when we convert joint 5l