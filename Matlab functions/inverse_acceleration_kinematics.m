function q_double_dot = inverse_acceleration_kinematics(q,q_dot,A_F)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
J=jacobian_matrix(q);
J_dot = jacobian_derivative(q, q_dot);
q_double_dot = pinv(J)*(A_F - (J_dot*q_dot))
end