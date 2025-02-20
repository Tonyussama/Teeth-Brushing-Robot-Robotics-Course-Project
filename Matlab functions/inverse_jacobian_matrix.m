function J_inv = inverse_jacobian_matrix(q)
 syms q0;
 syms q1;
 syms q2;
 syms q3;
[x,y,z]=forward_kinematics_func();
fq_n=[x,y,z];
fq_inverse= [diff(fq_n(1),q0) , diff(fq_n(1),q1) , diff(fq_n(1),q2) , diff(fq_n(1),q3);
             diff(fq_n(2),q0) , diff(fq_n(2),q1) , diff(fq_n(2),q2) , diff(fq_n(2),q3);
             diff(fq_n(3),q0) , diff(fq_n(3),q1) , diff(fq_n(3),q2) , diff(fq_n(3),q3)];
 fq_inverse=subs(fq_inverse,q0,q(1));
 fq_inverse=subs(fq_inverse,q1,q(2));
 fq_inverse=subs(fq_inverse,q2,q(3));
 fq_inverse=subs(fq_inverse,q3,q(4));
 fq_inverse=vpa(fq_inverse,2);
 fq_inverse=pinv(fq_inverse);
 J_inv=fq_inverse;
 % temp=fq_inverse;
 % fq_inverse=transpose(fq_inverse)*fq_inverse;
 % fq_inverse=fq_inverse^-1;
 % fq_inverse=fq_inverse*transpose(temp);
  % J_inv=vpa(fq_inverse,2);
end

