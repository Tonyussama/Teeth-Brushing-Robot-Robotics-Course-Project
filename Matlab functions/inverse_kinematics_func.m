function q = inverse_kinematics_func(q_0, X)
syms q0;
syms q1;
syms q2;
syms q3;
[x,y,z]=forward_kinematics_func();
fq_n=[x-X(1),y-X(2),z-X(3)];
fq_n=subs(fq_n,q0,q_0(1));
fq_n=subs(fq_n,q1,q_0(2));
fq_n=subs(fq_n,q2,q_0(3));
fq_n=subs(fq_n,q3,q_0(4));
f_j = inverse_jacobian_matrix(q_0);
q_n=(q_0)-(f_j*transpose(fq_n));
i=0;
while(i<20)
q_0=q_n;
fq_n=[x-X(1),y-X(2),z-X(3)];
fq_n=subs(fq_n,q0,q_0(1));
fq_n=subs(fq_n,q1,q_0(2));
fq_n=subs(fq_n,q2,q_0(3));
fq_n=subs(fq_n,q3,q_0(4));
f_j = inverse_jacobian_matrix(q_0);
q_n=q_0-(f_j*transpose(fq_n));
q_n=vpa(q_n,10);
i=i+1;
end    
q=vpa(q_n,4);
end