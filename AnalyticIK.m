function [q, eflag] = AnalyticIK(L1,L2,L3,L4,L5,T_ST)


    % 230126 HBY
    % Input
    % Li: length of each link
    % T_BT: desired SE(3) of end effector w.r.t. base frame
    % Output
    % q: joint angle of each joint
    % eflag: when joint angles are over joint limits

    n_joint = 6;
    
    % find wrist position
    offsetAngle = atan2(L4,L3);
    L34 = sqrt(L3^2+L4^2);
    p_TW = [0;0;-L5]; % Tool to Wrist
    p_SW = T_ST*[p_TW;1]; % Base to Wrist
    p_SW = p_SW(1:3);
    p_2W = p_SW - [0;0;L1]; % joint2 to Wrist
    
    
    % define screws
    [S, M_EF] = DefineScrew(L1,L2,L3,L4,L5);

    % solve IK
    q = zeros(6,1);
    r = norm(p_2W);
    if r > L2 + L34 || L2 > r + L34 || L34 > L2 + r % check if IK exists
        eflag = 0;
    else
        % positional IK (elbow up)
        q(1) = atan2(p_2W(2),p_2W(1));
        D = (r^2 - L2^2 - L34^2) / (2*L2*L34); % cos
        q(3) = atan2(sqrt(1-D^2),D) - offsetAngle;
        q(2) = pi/2 - atan2(p_2W(3),sqrt(p_2W(1)^2+p_2W(2)^2)) - atan2(L34*sin(q(3)+offsetAngle),L2+L34*cos(q(3)+offsetAngle));

        % angular IK
        M_EF_inv = [M_EF(1:3,1:3)' -M_EF(1:3,1:3)'*M_EF(1:3,4); zeros(1,3) 1];
        T = ScrewstoSE3(S(:,1:3),q(1:3)); % POE of the first three joints
        T_inv = [T(1:3,1:3)' -T(1:3,1:3)'*T(1:3,4); zeros(1,3) 1];
        T_ang = T_inv*T_ST*M_EF_inv; % POE of the last three joints
        R = T_ang(1:3,1:3);
        q(4:6) = rotm2eulXYZ(R);
        q(5) = q(5) + pi/2;
        q(6) = -q(6);
        
        % check joint limit
        if CheckMecaJointLimit(n_joint,q) ~= 0
            eflag = 0;
        else
            eflag = 1;
        end
    end

end