function J_dot = jacobian_derivative2(q, q_dot)

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

J = jacobian_matrix(q);

% Symbolic variables for time and joint velocities
syms t real
q_dot_sym = sym('q_dot', size(q));

% Calculate the time derivative using the chain rule
J_dot = jacobian(J*q_dot_sym, [q0, q1, q2, q3])*[q_dot_sym; 0]; 

% Substitute the values of q_dot
J_dot = subs(J_dot, q_dot_sym, q_dot');

% Simplify the expression
J_dot = simplify(J_dot);

end
