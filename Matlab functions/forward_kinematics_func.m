function [X,Y,Z] = forward_kinematics_func()

%  obtain the position of the end effector from...
% the multiplication of the transformation functions

syms q0;
syms q1;
syms q2;
syms q3;
T1=transformation_func(q0,0.066,0,(pi/2));
T2=transformation_func((-(pi/2)+q1),0,0,(pi/2));
T3=transformation_func((-(pi/2)+q2),0.149,0,(pi/2));
T4=transformation_func((-(pi/2)+q3),0,0.144,0);

T_EE=T1*T2*T3*T4;

X=T_EE(1,4);
Y=T_EE(2,4);
Z=T_EE(3,4);

% X=subs(X, q0, q(1));
% X=subs(X, q1, q(2));
% X=subs(X, q2, q(3));
% X=subs(X, q3, q(4));
% 
% Y=subs(Y, q0, q(1));
% Y=subs(Y, q1, q(2));
% Y=subs(Y, q2, q(3));
% Y=subs(Y, q3, q(4));
% 
% Z=subs(Z, q0, q(1));
% Z=subs(Z, q1, q(2));
% Z=subs(Z, q2, q(3));
% Z=subs(Z, q3, q(3));

 X=vpa(X,20);
 Y=vpa(Y,20);
 Z=vpa(Z,20);
end
