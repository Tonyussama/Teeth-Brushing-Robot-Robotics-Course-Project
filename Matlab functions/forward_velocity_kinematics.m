function  V_F = forward_velocity_kinematics(q, q_dot)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
J= jacobian_matrix(q);
V_F = J * q_dot ;

end