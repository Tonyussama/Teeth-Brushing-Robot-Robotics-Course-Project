function q_dot = inverse_velocity_kinematics(q, V_F)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
J=jacobian_matrix(q);
J_inv=(J.')*inv(J*J.');
q_dot= J_inv * V_F
end