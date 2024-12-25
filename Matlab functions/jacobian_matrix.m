function J = jacobian_matrix(q)
% Computes the Jacobian matrix given the angles as inputs 
% Calculates the different transformation matrices and extracts from them
% the relevant information
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);
%syms l1 l2 l3
T1=transformation_func(q0,-0.066,0,(-pi/2));
T2=transformation_func(((-pi/2)+q1),0,0,(pi/2));
T3=transformation_func(((-pi/2)+q2),-0.149,0,(pi/2));
T4=transformation_func(((-pi/2)+q3),0,0.152,0);
T_EE=T1*T2*T3*T4;

J=sym(zeros(6,4));
%for the 1st revolute joint
Jw1=[0,0,1];
Jv1= cross(Jw1, T_EE(1:3,4));
%for the 2nd revolute joint 
Jw2 = T1(1:3,3);
d2=T_EE(1:3,4)-T1(1:3,4);
Jv2 = cross(Jw2,d2);
%for the 3rd revolute joint 
Jw3 = T2(1:3,3);
d3=T_EE(1:3,4)-T2(1:3,4);
Jv3 = cross(Jw3,d3);
%for the 4th revolute joint 
Jw4 = T3(1:3,3);
d4=T_EE(1:3,4)-T3(1:3,4);
Jv4 = cross(Jw4,d4);


J(1:3,1)=Jv1;
J(4:6,1)=Jw1;

J(1:3,2)=Jv2;
J(4:6,2)=Jw2;

J(1:3,3)=Jv3;
J(4:6,3)=Jw3;

J(1:3,4)=Jv4;
J(4:6,4)=Jw4;
J = vpa(J, 2);
end
