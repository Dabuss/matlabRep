function out1 = getH06Iiwa(q1,q2,q3,q4,q5,q6)
%GETH06IIWA
%    OUT1 = GETH06IIWA(Q1,Q2,Q3,Q4,Q5,Q6)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    17-May-2017 10:59:54

t2 = cos(q1);
t3 = cos(q4);
t4 = sin(q1);
t5 = sin(q3);
t6 = t4.*t5;
t7 = cos(q2);
t8 = cos(q3);
t13 = t2.*t7.*t8;
t9 = t6-t13;
t10 = sin(q2);
t11 = sin(q4);
t12 = cos(q6);
t14 = t9.*t11;
t15 = t2.*t3.*t10;
t16 = t14+t15;
t17 = sin(q6);
t18 = cos(q5);
t19 = t3.*t9;
t28 = t2.*t10.*t11;
t20 = t19-t28;
t21 = t18.*t20;
t22 = sin(q5);
t23 = t4.*t8;
t24 = t2.*t5.*t7;
t25 = t23+t24;
t26 = t22.*t25;
t27 = t21+t26;
t29 = t2.*t5;
t30 = t4.*t7.*t8;
t31 = t29+t30;
t32 = t11.*t31;
t33 = t32-t3.*t4.*t10;
t34 = t3.*t31;
t35 = t4.*t10.*t11;
t36 = t34+t35;
t37 = t18.*t36;
t38 = t2.*t8;
t42 = t4.*t5.*t7;
t39 = t38-t42;
t40 = t22.*t39;
t41 = t37+t40;
t43 = t7.*t11;
t51 = t3.*t8.*t10;
t44 = t43-t51;
t45 = t18.*t44;
t46 = t5.*t10.*t22;
t47 = t45+t46;
t48 = t3.*t7;
t49 = t8.*t10.*t11;
t50 = t48+t49;
out1 = reshape([-t16.*t17-t12.*t27,t17.*t33+t12.*t41,t12.*t47-t17.*t50,0.0,-t12.*t16+t17.*t27,t12.*t33-t17.*t41,-t12.*t50-t17.*t47,0.0,t20.*t22-t18.*t25,t18.*t39-t22.*t36,-t22.*t44+t5.*t10.*t18,0.0,t2.*t10.*4.2e2+t9.*t11.*4.0e2+t2.*t3.*t10.*4.0e2,t4.*t10.*4.2e2-t11.*t31.*4.0e2+t3.*t4.*t10.*4.0e2,t7.*4.2e2+t3.*t7.*4.0e2+t8.*t10.*t11.*4.0e2+3.6e2,1.0],[4,4]);