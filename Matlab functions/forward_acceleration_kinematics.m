function A_F = forward_acceleration_kinematics(q,q_dot,q_double_dot)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
J=jacobian_matrix(q);
J_dot = jacobian_derivative(q, q_dot);
A_F = J_dot*q_dot + J*q_double_dot;
A_F=vpa(A_F,2);
end